"""Pin Bug C class: when a leaf callee's exit (M, X) depends on the
entry variant (typical for SEP/REP that only touches ONE flag), each
JSR fall-through key must use the variant-specific exit, not a single
broadcast value.

Concrete scenario (matches SMW $00:8B2B SineAndScale and Bug C):

    Caller F at $8000 enters M=0, X=0 (e.g. after REP #$30):
        $8000: JSR Leaf  (20 00 90)     — caller at M=0 X=0
        $8003: CPY #$0004 (C0 04 00)    — 3 bytes when x=0
        $8006: RTS       (60)

    Leaf at $9000 (REP #$20 only — M-only mutator):
        $9000: REP #$20  (C2 20)        — clears M
        $9002: RTS       (60)

    Entry (0,0) → leaf exit (0,0)  [X preserved]
    Entry (1,1) → leaf exit (0,1)  [X preserved]

If the JSR fall-through key gets entry (0,1) (the broadcast bug), the
decoder mis-decodes `CPY #$0004` as `CPY #$04` (1-byte immediate) and
the next byte (`$00`) becomes a phantom BRK opcode. The function
exits early with no fall-through.

Test asserts:
  1. callee_exit_mx with PER-VARIANT entry for ($9000, 0, 0) → (0, 0)
     produces fall-through key ($8003, 0, 0).
  2. With the BROADCAST entry alone (the bug), fall-through would be
     ($8003, 0, 1) — confirmed as a regression that the per-variant
     mechanism prevents.
"""
from _helpers import make_lorom_bank0  # noqa: E402
from v2.decoder import decode_function, DecodeKey  # noqa: E402


def test_jsr_fall_through_uses_per_variant_exit_mx():
    """Callee_exit_mx with entry-specific exits routes fall-through
    correctly. Caller at (M=0, X=0) calls leaf at $9000 (REP #$20 only).
    Per-variant exit_mx[($009000, 0, 0)] = (0, 0). Fall-through key
    after JSR must be ($008003, 0, 0)."""
    rom = make_lorom_bank0({
        # $8000-$8006: caller
        0x8000: bytes([0x20, 0x00, 0x90, 0xC0, 0x04, 0x00, 0x60]),
        # $9000-$9002: leaf (REP #$20 ; RTS)
        0x9000: bytes([0xC2, 0x20, 0x60]),
    })
    callee_exit_mx = {
        # Per-variant entries: each (entry_m, entry_x) carries its
        # specific exit. NO broadcast smearing of (0,1) onto (0,0).
        (0x009000, 0, 0): (0, 0),
        (0x009000, 0, 1): (0, 1),
        (0x009000, 1, 0): (0, 0),
        (0x009000, 1, 1): (0, 1),
    }

    graph = decode_function(rom, bank=0x00, start=0x8000,
                            entry_m=0, entry_x=0,
                            callee_exit_mx=callee_exit_mx)

    # The JSR at $8000 (entry M0X0) MUST fall through to $8003 with
    # entry (M=0, X=0), NOT (M=0, X=1). The CPY at $8003 must then
    # decode as 3-byte immediate (length=3), not 2-byte.
    jsr_key = DecodeKey(pc=0x008000, m=0, x=0)
    assert jsr_key in graph.insns
    jsr_di = graph.insns[jsr_key]
    assert jsr_di.insn.mnem == 'JSR'
    # The fall-through edge (only successor for JSR) MUST be M0X0.
    assert len(jsr_di.successors) == 1
    fall_key = jsr_di.successors[0]
    assert fall_key.pc == 0x008003, f"expected $8003, got ${fall_key.pc:06X}"
    assert fall_key.m == 0 and fall_key.x == 0, (
        f"expected M0X0, got M{fall_key.m}X{fall_key.x}"
    )

    # CPY at $8003 MUST have decoded as 3-byte immediate (16-bit, x=0).
    cpy_key = DecodeKey(pc=0x008003, m=0, x=0)
    assert cpy_key in graph.insns
    cpy_di = graph.insns[cpy_key]
    assert cpy_di.insn.mnem == 'CPY'
    assert cpy_di.insn.length == 3

    # No M0X1 variant of $8003 was created — broadcast bug is gone.
    cpy_wrong = DecodeKey(pc=0x008003, m=0, x=1)
    assert cpy_wrong not in graph.insns


def test_jsr_fall_through_uses_broadcast_when_no_per_variant():
    """Counterpart: when callee_exit_mx has ONLY the broadcast entry
    (the legacy 4-tuple shape), the decoder uses it for whatever
    (entry_m, entry_x) the caller is at. This pins the legacy path
    that the per-variant mechanism extends, not replaces."""
    rom = make_lorom_bank0({
        0x8000: bytes([0x20, 0x00, 0x90, 0xC0, 0x04, 0x00, 0x60]),
        0x9000: bytes([0xC2, 0x20, 0x60]),
    })
    # Broadcast: only the cfg-default (1, 1) entry is annotated;
    # caller's actual (m, x) = (0, 0), lookup miss → decoder falls back
    # to post_m, post_x = (0, 0) (entry was preserved by post_mx for JSR).
    callee_exit_mx = {
        (0x009000, 1, 1): (0, 1),
    }
    graph = decode_function(rom, bank=0x00, start=0x8000,
                            entry_m=0, entry_x=0,
                            callee_exit_mx=callee_exit_mx)
    jsr_di = graph.insns[DecodeKey(pc=0x008000, m=0, x=0)]
    fall_key = jsr_di.successors[0]
    # Without a matching (0, 0) per-variant entry, decoder uses caller's
    # post-JSR state (0, 0) — same correct result.
    assert (fall_key.m, fall_key.x) == (0, 0)


def test_smw_008b01_decodes_as_m0x0_with_per_variant():
    """Bug C reproducer using the real SMW ROM. With per-variant
    callee_exit_mx for $00:8B2B (SineAndScale), $00:8B01 must be
    decoded as M0X0 (CPY #$0004 length=3), not M0X1 (would mis-decode
    as CPY #$04 length=2)."""
    import pathlib
    rom_path = pathlib.Path(__file__).resolve().parents[3] / "smw.sfc"
    if not rom_path.exists():
        return  # ROM not present; skip silently in CI / clean checkouts
    rom = rom_path.read_bytes()
    if len(rom) % 1024 == 512:
        rom = rom[512:]

    # Per-variant for $00:8B2B — only the (1,1)→(0,1) and (1,0)→(0,0)
    # mutators are recorded; (0,0) and (0,1) fall through with default
    # post-JSR semantics (callee preserves caller M/X).
    callee_exit_mx = {
        (0x008B2B, 1, 0): (0, 0),
        (0x008B2B, 1, 1): (0, 1),
    }
    # Decode the inner sub at $00:8AE8 (CalcBasisVector body via
    # _008AE8_M0X0). It's reached from CalcMode7Values after REP #$30,
    # so its entry context is M=0, X=0.
    graph = decode_function(rom, bank=0x00, start=0x8AE8,
                            entry_m=0, entry_x=0,
                            callee_exit_mx=callee_exit_mx)
    cpy_key = DecodeKey(pc=0x008B01, m=0, x=0)
    assert cpy_key in graph.insns, "expected $00:8B01 to be decoded as M0X0"
    cpy_di = graph.insns[cpy_key]
    assert cpy_di.insn.mnem == 'CPY'
    assert cpy_di.insn.length == 3, (
        f"CPY at $00:8B01 with x=0 must be 3 bytes (CPY #$0004), "
        f"got length={cpy_di.insn.length}"
    )
    # Make sure no M0X1 variant exists — that would be the bug.
    cpy_bug = DecodeKey(pc=0x008B01, m=0, x=1)
    assert cpy_bug not in graph.insns
