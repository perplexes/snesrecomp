"""Pin v2 decoder: JSR-inline-param skip.

Star Fox's bg-setup helpers (bg2chr/bg2scr/dopalette, BGS.ASM) use the
inline-param idiom: the callee pulls its own return address, reads N bytes
of inline data emitted right after the `jsr`, then `adc #N; pha; rts` to
return to caller+N (skipping the inline data). A naive linear decode
resumes at pc_after_jsr and decodes the N param bytes as garbage
instructions.

`callee_inline_skip={target_pc24: N}` makes the decoder advance the
fall-through PC by N so the param bytes are NOT decoded as instructions.
This pins that behaviour (and the no-map control).
"""
from _helpers import make_lorom_bank0  # noqa: E402

from v2.decoder import (  # noqa: E402
    decode_function, addr24, set_global_inline_skip,
)


def _make_rom():
    """Caller at $8000:
        JSR $9000           ; inline-param helper at $9000, skips 5
        <5 inline data bytes: A9 A9 A9 A9 A9>   ; would decode as LDA #imm
        EA                  ; NOP  (the REAL next instruction, at $8008)
        60                  ; RTS
      Callee body at $9000: just RTS (the runtime pull/adc/pha is not
      modelled here — the test pins decode-time fall-through only).

    The 5 inline bytes are all $A9 (LDA #imm opcode). If the decoder does
    NOT skip them, it decodes an LDA at $8003. If it DOES skip, the first
    decoded post-call insn is the NOP ($EA) at $8008.
    """
    blobs = {
        0x8000: bytes([
            0x20, 0x00, 0x90,   # JSR $9000
            0xA9, 0xA9, 0xA9, 0xA9, 0xA9,   # 5 inline param bytes
            0xEA,               # NOP  (real next insn at $8008)
            0x60,               # RTS
        ]),
        0x9000: bytes([0x60]),  # RTS
    }
    return make_lorom_bank0(blobs)


def test_no_skip_decodes_inline_bytes_as_instructions():
    """Control: without callee_inline_skip, the decoder resumes at $8003
    and decodes the inline $A9 byte as an LDA (the garbage path)."""
    set_global_inline_skip({})  # ensure no leaked global from another test
    rom = _make_rom()
    graph = decode_function(rom, bank=0, start=0x8000, entry_m=1, entry_x=1)
    keys_8003 = [k for k in graph.insns if k.pc == addr24(0, 0x8003)]
    assert keys_8003, "without skip, decode resumes at $8003"
    assert graph.insns[keys_8003[0]].insn.mnem == 'LDA'
    # The real NOP at $8008 is NOT reached as an instruction boundary
    # (it's mid-LDA-operand on the garbage path).
    keys_8008 = [k for k in graph.insns if k.pc == addr24(0, 0x8008)]
    assert not keys_8008, "garbage path should not land cleanly on $8008"


def test_skip_advances_fallthrough_past_inline_bytes():
    """With callee_inline_skip={$9000: 5}, the decoder skips the 5 inline
    bytes and resumes at $8008 (the real NOP) — the param bytes are never
    decoded as instructions."""
    set_global_inline_skip({})
    rom = _make_rom()
    callee_pc24 = addr24(0, 0x9000)
    graph = decode_function(rom, bank=0, start=0x8000, entry_m=1, entry_x=1,
                            callee_inline_skip={callee_pc24: 5})
    keys_8008 = [k for k in graph.insns if k.pc == addr24(0, 0x8008)]
    assert keys_8008, "with skip, decode resumes at $8008 (real NOP)"
    assert graph.insns[keys_8008[0]].insn.mnem == 'NOP'
    # The inline bytes at $8003 must NOT be a decoded instruction boundary.
    keys_8003 = [k for k in graph.insns if k.pc == addr24(0, 0x8003)]
    assert not keys_8003, "inline param bytes must not be decoded"


def test_global_inline_skip_fallback_applies():
    """When no explicit map is threaded, the process-global skip map
    (set by v2_regen) is consulted."""
    rom = _make_rom()
    callee_pc24 = addr24(0, 0x9000)
    set_global_inline_skip({callee_pc24: 5})
    try:
        graph = decode_function(rom, bank=0, start=0x8000,
                                entry_m=1, entry_x=1)
        keys_8008 = [k for k in graph.insns if k.pc == addr24(0, 0x8008)]
        assert keys_8008, "global skip map should advance fall-through"
        assert graph.insns[keys_8008[0]].insn.mnem == 'NOP'
    finally:
        set_global_inline_skip({})  # don't leak into other tests


if __name__ == '__main__':
    import sys
    import traceback
    failed = 0
    for name in [n for n in dir() if n.startswith('test_')]:
        try:
            globals()[name]()
            print(f"PASS  {name}")
        except Exception:
            failed += 1
            print(f"FAIL  {name}")
            traceback.print_exc()
    sys.exit(1 if failed else 0)
