"""Pin the v2 decoder `reloc` (RAM-executed-from-ROM) behaviour.

A `reloc <ram_bank> <ram_addr> <rom_bank> <rom_off> <len>` directive
declares a region of code that EXECUTES from RAM (e.g. WRAM $7E) but was
COPIED there from ROM at boot. The mapping is exact/linear:
ram_addr+k == rom_off+k. The decoder must:
  - fetch instruction bytes from the ROM SOURCE,
  - but keep ALL addresses (Insn.addr, DecodeKey.pc, names, emitted
    symbols, call targets) at the RAM EXECUTION address.

Concrete motivation: Star Fox copies ~23KB from ROM $02:8000 into WRAM
$7E:321F at boot and runs irqcode_l there. Today `lorom_offset` asserts
addr>=$8000 and uses bank&0x7F, so a $7E function couldn't be decoded and
$7E call targets were rejected.
"""
import sys
import pathlib

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
if str(REPO / 'recompiler') not in sys.path:
    sys.path.insert(0, str(REPO / 'recompiler'))

import snes65816  # noqa: E402
from snes65816 import addr_to_rom_offset, lorom_offset  # noqa: E402
from v2.decoder import decode_function, addr24  # noqa: E402
from v2.cfg_loader import load_bank_cfg  # noqa: E402
from v2 import codegen  # noqa: E402


# Reloc region used throughout: WRAM $7E:321F <- ROM $02:8000, length $5C00
# (matches the Star Fox irqcode block).
RELOC = [(0x7E, 0x321F, 0x02, 0x8000, 0x5C00)]


def _make_rom_with_routine():
    """Build a LoROM image whose bank $02:8000 holds a known routine.

    The routine (in M=1, 8-bit A) — includes an internal relative branch
    so we can verify branch targets are RELOCATED to the WRAM region:
        $8000  LDA #$12      A9 12
        $8002  BNE $8005     D0 01     (taken target is internal: NOP)
        $8004  RTL           6B        (fall-through path returns)
        $8005  NOP           EA        (branch-taken path)
        $8006  RTL           6B

    Bank $02:8000 is ROM offset (0x02 & 0x7F)*0x8000 + 0 = 0x10000.
    """
    rom = bytearray(0x18000)  # covers banks 0..2 (3 * 0x8000)
    base = lorom_offset(0x02, 0x8000)  # 0x10000
    #             LDA#  BNE+1  RTL   NOP   RTL
    routine = bytes([0xA9, 0x12, 0xD0, 0x01, 0x6B, 0xEA, 0x6B])
    rom[base:base + len(routine)] = routine
    return bytes(rom), routine


def test_addr_to_rom_offset_maps_into_rom_source():
    rom, _ = _make_rom_with_routine()
    # $7E:321F fetches from ROM $02:8000.
    assert addr_to_rom_offset(0x7E, 0x321F, RELOC) == lorom_offset(0x02, 0x8000)
    # +k stays linear.
    assert addr_to_rom_offset(0x7E, 0x3221, RELOC) == lorom_offset(0x02, 0x8000) + 2
    # Address outside the region (still in $7E) is not remapped — falls
    # through to lorom_offset, which would assert for <$8000. A $7E:8000
    # address is fine for lorom_offset (bank&0x7F=$7E*... ), just not relocated.
    assert addr_to_rom_offset(0x00, 0x8000, RELOC) == lorom_offset(0x00, 0x8000)


def test_relocated_function_decodes_same_as_rom_source():
    """Decoding $7E:321F (relocated) yields the SAME instruction stream as
    decoding the ROM source $02:8000 would — but with $7E addresses."""
    rom, routine = _make_rom_with_routine()

    # Decode the SOURCE at its natural ROM location for the oracle.
    src_graph = decode_function(rom, bank=0x02, start=0x8000,
                                entry_m=1, entry_x=1)
    src_insns = sorted(src_graph.insns.values(),
                       key=lambda di: di.insn.addr)
    src_seq = [(di.insn.mnem, di.insn.operand, di.insn.length)
               for di in src_insns]

    # Decode the RELOCATED function at its WRAM execution address.
    reloc_graph = decode_function(rom, bank=0x7E, start=0x321F,
                                  entry_m=1, entry_x=1,
                                  reloc_regions=RELOC)
    reloc_insns = sorted(reloc_graph.insns.values(),
                         key=lambda di: di.insn.addr)
    reloc_seq = [(di.insn.mnem, di.insn.operand, di.insn.length)
                 for di in reloc_insns]

    # The two decodes must be structurally identical: same number of
    # insns, same mnemonics, same lengths, in the same order.
    assert len(reloc_seq) == len(src_seq), \
        f"insn count differs: {reloc_seq} vs {src_seq}"
    assert [s[0] for s in reloc_seq] == [s[0] for s in src_seq], \
        f"mnemonic streams differ: {reloc_seq} vs {src_seq}"
    assert [s[2] for s in reloc_seq] == [s[2] for s in src_seq], \
        f"lengths differ: {reloc_seq} vs {src_seq}"

    # Every relocated operand equals the source operand re-based into $7E:
    # for the in-region relative branch, source $8005 -> $7E reloc $3225
    # ($321F + ($8005-$8000)). The LDA immediate ($12) is unchanged.
    for r, s in zip(reloc_seq, src_seq):
        mnem, r_op, _ = r
        _, s_op, _ = s
        if mnem in ('BNE', 'BEQ', 'BRA', 'BRL', 'JMP', 'JSR'):
            # control-flow operand is an in-region address; relocated.
            assert r_op == 0x321F + (s_op - 0x8000), \
                f"{mnem} operand ${r_op:04X} != relocated ${s_op:04X}"
        else:
            assert r_op == s_op, f"{mnem} operand changed: {r_op} vs {s_op}"

    # The first instruction is LDA #$12 in M=1 (8-bit immediate, length 2).
    lda = next(di for di in reloc_insns if di.insn.mnem == 'LDA')
    assert lda.insn.operand == 0x12
    assert lda.insn.length == 2


def test_relocated_addresses_and_symbol_are_wram():
    """Insn.addr / DecodeKey.pc / the function symbol stay at $7E (the WRAM
    execution address), NOT the ROM source $02."""
    rom, _ = _make_rom_with_routine()
    graph = decode_function(rom, bank=0x7E, start=0x321F,
                            entry_m=1, entry_x=1, reloc_regions=RELOC)

    # Entry key is the WRAM address.
    assert graph.entry.pc == addr24(0x7E, 0x321F) == 0x7E321F

    # Every decoded insn is addressed in bank $7E, none in $02.
    for di in graph.insns.values():
        assert (di.insn.addr >> 16) & 0xFF == 0x7E, \
            f"insn ${di.insn.addr:06X} not addressed at $7E"

    # The internal BNE's target stays in $7E (relocated), not $02.
    bne = next(di for di in graph.insns.values() if di.insn.mnem == 'BNE')
    # BNE at $7E:3221 (LDA is 2 bytes from $321F). Its taken target is the
    # source $8005 re-based into the WRAM region: $321F + ($8005-$8000)
    # = $3224. It stays in $7E, never $02.
    assert bne.insn.addr == 0x7E3221, f"BNE at ${bne.insn.addr:06X}"
    assert bne.insn.operand == 0x321F + (0x8005 - 0x8000), \
        f"BNE target ${bne.insn.operand:04X} should be in $7E region"
    assert 0x321F <= bne.insn.operand < 0x321F + 0x5C00

    # Default C symbol for the entry would be bank_7E_321F.
    pc = graph.entry.pc & 0xFFFF
    bank = (graph.entry.pc >> 16) & 0xFF
    assert f"bank_{bank:02X}_{pc:04X}" == "bank_7E_321F"


def test_external_jsl_into_reloc_region_resolves():
    """A `jsl $7E321F` from a normal bank must NOT be rejected as an
    out-of-ROM target — codegen's _is_invalid_lorom_call_target consults
    the reloc map and accepts it (resolving to bank_7E_321F)."""
    try:
        codegen.set_rom_size(0x18000)
        codegen.set_reloc_regions(RELOC)
        # Inside the region: accepted.
        assert codegen._is_invalid_lorom_call_target(0x7E321F) is False
        assert codegen._addr_in_reloc_region(0x7E321F) is True
        # A $7E address OUTSIDE the region stays rejected (it's RAM, no ROM
        # backing): $7E:0000 is below the region.
        assert codegen._is_invalid_lorom_call_target(0x7E0000) is True
    finally:
        codegen.set_reloc_regions(None)
        codegen.set_rom_size(0)


def test_cfg_loader_parses_reloc_directive(tmp_path=None):
    """The `reloc` directive parses into BankCfg.reloc_regions, and a
    `func` entry below $8000 is accepted (no $8000 assertion at load)."""
    import tempfile
    import os
    text = (
        "bank = 7E\n"
        "reloc 7E 321F 02 8000 5C00\n"
        "func irqcode_l 321F\n"
        "name 7E4F4A runmario_l\n"
    )
    fd, path = tempfile.mkstemp(suffix='.cfg')
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(text)
        cfg = load_bank_cfg(path)
    finally:
        os.unlink(path)
    assert cfg.bank == 0x7E
    assert cfg.reloc_regions == [(0x7E, 0x321F, 0x02, 0x8000, 0x5C00)]
    # func entry retained at its $321F local PC.
    starts = {e.start & 0xFFFF for e in cfg.entries}
    assert 0x321F in starts
    names = {e.name for e in cfg.entries}
    assert 'irqcode_l' in names
