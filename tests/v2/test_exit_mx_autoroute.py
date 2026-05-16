"""Pin v2 exit_mx_autoroute behaviour (leaf-only auto-detect).

The cfg `exit_mx_at <addr> <m> <x>` directive declares that the callee
at <addr> exits with (M, X) = (m, x). When set, the v2 decoder uses
the declared values for the JSR/JSL fall-through edge so the caller
resumes decoding with the right operand widths. Without it, the
decoder assumes M/X are preserved — wrong whenever the callee runs
an internal SEP/REP that never restores before return.

`v2.exit_mx_autoroute.detect_and_route` is a leaf-only auto-detector:
for every cfg `func` entry F whose decoded body contains NO JSR/JSL,
it analyses the (M, X) state at every RTS/RTL terminator. If those
unanimously differ from the entry state, it appends an
`exit_mx_at` tuple to F's owning BankCfg so the existing builder
picks it up.

Leaf-only restriction is mandatory: an earlier (2026-05-03) attempt at
unrestricted fixpoint regressed GraphicsDecompress into an infinite
loop and was reverted to opt-in.

Tests pin: positive (canonical SEP/RTS), negative (JSR-inside,
ambiguous, no change, no terminator, already-declared, unnamed).
"""
from _helpers import make_lorom_bank0  # noqa: E402

from dataclasses import dataclass, field
from v2.exit_mx_autoroute import detect_and_route  # noqa: E402


@dataclass
class _BankEntry:
    name: str
    start: int
    end: int = None
    entry_m: int = 1
    entry_x: int = 1
    tail_call_pc16: int = None


@dataclass
class _BankCfg:
    bank: int
    entries: list = field(default_factory=list)
    exit_mx_at: list = field(default_factory=list)
    # Per-variant exit annotations (added 2026-05-15). One tuple per
    # mutating (entry_m, entry_x). v2_regen prefers per-variant over
    # the legacy 4-tuple broadcast.
    exit_mx_at_per_variant: list = field(default_factory=list)


def test_detects_sep_then_rts_sets_m_to_1():
    """Canonical leaf: REP #$20 (clear M) then RTS. Per-variant detection
    emits one record per mutating entry — entry M=0 already has M=0 so
    is preserved; entry M=1 has M mutate to 0 regardless of X."""
    # $8000: REP #$20  (C2 20)         — clear M flag
    # $8002: RTS       (60)
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])
    parsed = [(0x00, 'bank00.cfg', cfg)]

    fixes = detect_and_route(parsed, rom)
    # Per-variant: only the two entries where M=1 mutate.
    assert len(fixes) == 2
    by_entry = {(fx.entry_m, fx.entry_x): fx for fx in fixes}
    assert (1, 0) in by_entry and (1, 1) in by_entry
    assert by_entry[(1, 0)].exit_m == 0 and by_entry[(1, 0)].exit_x == 0
    assert by_entry[(1, 1)].exit_m == 0 and by_entry[(1, 1)].exit_x == 1

    # Per-variant list: each mutating entry gets a 6-tuple.
    assert (0x00, 0x8000, 1, 0, 0, 0) in cfg.exit_mx_at_per_variant
    assert (0x00, 0x8000, 1, 1, 0, 1) in cfg.exit_mx_at_per_variant
    # Non-mutating entries (M=0 already) are NOT recorded.
    assert all(t[2] == 1 for t in cfg.exit_mx_at_per_variant
               if (t[0], t[1]) == (0x00, 0x8000))

    # Autoroute does NOT touch the legacy 4-tuple list; per-variant is
    # the only source. Broadcasting would inject a wrong exit for the
    # entries-that-don't-mutate variants and re-create the Bug C class.
    assert cfg.exit_mx_at == []


def test_detects_rep_30_sets_both_to_0():
    """REP #$30 clears both M and X. Per-variant:
       (0,0) -> (0,0)  no mutation, no record
       (0,1) -> (0,0)  X mutates
       (1,0) -> (0,0)  M mutates
       (1,1) -> (0,0)  M and X mutate
    """
    # $8000: REP #$30  (C2 30)
    # $8002: RTS       (60)
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x30, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])
    parsed = [(0x00, 'bank00.cfg', cfg)]

    fixes = detect_and_route(parsed, rom)
    assert len(fixes) == 3
    by_entry = {(fx.entry_m, fx.entry_x): (fx.exit_m, fx.exit_x) for fx in fixes}
    assert by_entry == {(0, 1): (0, 0), (1, 0): (0, 0), (1, 1): (0, 0)}
    # Autoroute writes only to per-variant; legacy 4-tuple stays empty.
    assert cfg.exit_mx_at == []


def test_skips_when_body_has_jsr():
    """Non-leaf: contains JSR. Auto-router must skip (callee dependency
    is exactly what the prior fixpoint attempt regressed on)."""
    # $8000: REP #$20      (C2 20)
    # $8002: JSR $8100     (20 00 81)
    # $8005: RTS           (60)
    # $8100: RTS           (60)
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0x20, 0x00, 0x81, 0x60]),
        0x8100: bytes([0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    G = _BankEntry(name='G', start=0x8100)
    cfg = _BankCfg(bank=0x00, entries=[F, G])
    parsed = [(0x00, 'bank00.cfg', cfg)]

    fixes = detect_and_route(parsed, rom)
    # F has a JSR — skipped. G is a single RTS — no state change, skipped.
    assert fixes == []
    assert cfg.exit_mx_at == []


def test_skips_when_body_has_jsl():
    """Non-leaf: contains JSL. Same reason as JSR."""
    # $8000: REP #$20
    # $8002: JSL $018100  (22 00 81 01)
    # $8006: RTS
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0x22, 0x00, 0x81, 0x01, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])
    parsed = [(0x00, 'bank00.cfg', cfg)]

    fixes = detect_and_route(parsed, rom)
    assert fixes == []


def test_skips_when_no_state_change():
    """A pure leaf with no SEP/REP — exit matches entry, nothing to
    auto-route."""
    # $8000: LDA #$42  (A9 42)         — m=1 entry, so 2-byte LDA
    # $8002: STA $05   (85 05)
    # $8004: RTS       (60)
    rom = make_lorom_bank0({
        0x8000: bytes([0xA9, 0x42, 0x85, 0x05, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])
    parsed = [(0x00, 'bank00.cfg', cfg)]

    fixes = detect_and_route(parsed, rom)
    assert fixes == []


def test_skips_when_already_cfg_declared():
    """Opt-in cfg `exit_mx_at` directive already exists at F's PC —
    auto-router must respect it and not redundantly add another tuple."""
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(
        bank=0x00, entries=[F],
        exit_mx_at=[(0x00, 0x8000, 1, 1)],  # cfg-declared (even if wrong)
    )
    parsed = [(0x00, 'bank00.cfg', cfg)]

    fixes = detect_and_route(parsed, rom)
    assert fixes == []
    # The declared entry is preserved untouched.
    assert cfg.exit_mx_at == [(0x00, 0x8000, 1, 1)]


def test_skips_unnamed_entry():
    """A BankEntry with name=None is scaffolding, not a real function.
    Auto-router skips it."""
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0x60]),
    })
    F = _BankEntry(name=None, start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])
    parsed = [(0x00, 'bank00.cfg', cfg)]

    fixes = detect_and_route(parsed, rom)
    assert fixes == []


def test_skips_ambiguous_exit_state():
    """Two RTS paths exiting with different (M, X) — analyzer returns
    None for the disagreeing component. Auto-router must skip."""
    # $8000: LDA $05            (A5 05)   — m=1 entry, 2 bytes
    # $8002: BEQ $8008          (F0 04)   — branch to 8008
    # $8004: REP #$20           (C2 20)   — clear M
    # $8006: RTS                (60)
    # $8007: NOP                (EA)      — padding
    # $8008: RTS                (60)      — exit M=1 (no SEP/REP)
    # One path: $8006 RTS with M=0. Other path: $8008 RTS with M=1.
    rom = make_lorom_bank0({
        0x8000: bytes([0xA5, 0x05, 0xF0, 0x04, 0xC2, 0x20, 0x60, 0xEA, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])
    parsed = [(0x00, 'bank00.cfg', cfg)]

    fixes = detect_and_route(parsed, rom)
    assert fixes == []
    assert cfg.exit_mx_at == []


def test_skips_when_no_terminator():
    """A function whose decoded graph reaches no RTS/RTL (e.g. infinite
    BRA loop) has no resume point to propagate. Auto-router skips."""
    # $8000: REP #$20
    # $8002: BRA $8002 (80 FE)  — infinite loop, no RTS reached
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0x80, 0xFE]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])
    parsed = [(0x00, 'bank00.cfg', cfg)]

    fixes = detect_and_route(parsed, rom)
    assert fixes == []


def test_multiple_sites_in_one_bank_each_independent():
    """Two unrelated leaf state-mutators in one bank — per-variant
    detection produces independent records for each function."""
    # $8000: REP #$20 ; RTS   — clears M only
    # $9000: REP #$10 ; RTS   — clears X only
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0x60]),
        0x9000: bytes([0xC2, 0x10, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    G = _BankEntry(name='G', start=0x9000)
    cfg = _BankCfg(bank=0x00, entries=[F, G])
    parsed = [(0x00, 'bank00.cfg', cfg)]

    fixes = detect_and_route(parsed, rom)
    # F (REP #$20): mutating entries (1,0)->(0,0) and (1,1)->(0,1) → 2.
    # G (REP #$10): mutating entries (0,1)->(0,0) and (1,1)->(1,0) → 2.
    assert len(fixes) == 4
    f_fixes = sorted(((fx.entry_m, fx.entry_x), (fx.exit_m, fx.exit_x))
                     for fx in fixes if fx.addr16 == 0x8000)
    g_fixes = sorted(((fx.entry_m, fx.entry_x), (fx.exit_m, fx.exit_x))
                     for fx in fixes if fx.addr16 == 0x9000)
    assert f_fixes == [((1, 0), (0, 0)), ((1, 1), (0, 1))]
    assert g_fixes == [((0, 1), (0, 0)), ((1, 1), (1, 0))]
    # Autoroute writes only to per-variant; legacy 4-tuple stays empty.
    assert cfg.exit_mx_at == []
    # Each function contributes its mutating-variant records to per-variant.
    assert (0x00, 0x8000, 1, 0, 0, 0) in cfg.exit_mx_at_per_variant
    assert (0x00, 0x8000, 1, 1, 0, 1) in cfg.exit_mx_at_per_variant
    assert (0x00, 0x9000, 0, 1, 0, 0) in cfg.exit_mx_at_per_variant
    assert (0x00, 0x9000, 1, 1, 1, 0) in cfg.exit_mx_at_per_variant


def test_per_variant_records_variant_specific_exit():
    """The Bug C class: a leaf whose SEP/REP only touches ONE of M/X
    leaves the OTHER at the entry value. Per-variant records preserve
    that — the legacy broadcast would corrupt the M0X0 entry's exit.

    REP #$20 ; RTS (forces M=0, leaves X). Per-variant:
       (0,0) -> (0,0)  preserved (X=0 stays)
       (0,1) -> (0,1)  preserved (X=1 stays)
       (1,0) -> (0,0)  M mutates only
       (1,1) -> (0,1)  M mutates only

    The two records must encode the variant-specific X.
    """
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0x60]),   # REP #$20 ; RTS
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])
    parsed = [(0x00, 'bank00.cfg', cfg)]

    detect_and_route(parsed, rom)
    # Per-variant list MUST distinguish X-preserving variants. Critical
    # assertion: (1,0) → (0,0) and (1,1) → (0,1) — different exit_x.
    assert (0x00, 0x8000, 1, 0, 0, 0) in cfg.exit_mx_at_per_variant
    assert (0x00, 0x8000, 1, 1, 0, 1) in cfg.exit_mx_at_per_variant
    # Non-mutating variants are NOT recorded — the default decoder
    # fall-through (ret_m=post_m, ret_x=post_x) already gives correct
    # state for those.
    pv_for_F = [t for t in cfg.exit_mx_at_per_variant if t[1] == 0x8000]
    assert len(pv_for_F) == 2


def test_sep_then_rep_consistent_path_force_m_to_1():
    """REP #$20 followed by SEP #$20 — final SEP forces M=1 on exit
    REGARDLESS of entry. So entry-M=0 variants mutate (0→1), entry-M=1
    variants pass through. Per-variant must capture this asymmetry —
    legacy single-tuple behavior would commit (0, ...) for cfg-default
    M=1 entry (no mutation) and miss the M=0 variants entirely."""
    # $8000: REP #$20 ; SEP #$20 ; RTS
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0xE2, 0x20, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])
    parsed = [(0x00, 'bank00.cfg', cfg)]

    fixes = detect_and_route(parsed, rom)
    # Entry M=0 mutates to M=1; entry M=1 passes through.
    by_entry = {(fx.entry_m, fx.entry_x): (fx.exit_m, fx.exit_x) for fx in fixes}
    assert by_entry == {(0, 0): (1, 0), (0, 1): (1, 1)}
    # cfg-default (M=1, X=1) does NOT mutate, so legacy 4-tuple is empty.
    assert cfg.exit_mx_at == []


def test_sep_only_x_flag():
    """SEP #$10 only sets X — auto-detect per-variant. Entries with X=0
    mutate to X=1; entries with X=1 already pass through."""
    # $8000: SEP #$10 ; RTS — sets X=1.
    rom = make_lorom_bank0({
        0x8000: bytes([0xE2, 0x10, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000, entry_m=0, entry_x=0)
    cfg = _BankCfg(bank=0x00, entries=[F])
    parsed = [(0x00, 'bank00.cfg', cfg)]

    fixes = detect_and_route(parsed, rom)
    # Only entries with X=0 mutate.
    by_entry = {(fx.entry_m, fx.entry_x): (fx.exit_m, fx.exit_x) for fx in fixes}
    assert by_entry == {(0, 0): (0, 1), (1, 0): (1, 1)}
    # Autoroute writes only to per-variant; legacy 4-tuple stays empty.
    assert cfg.exit_mx_at == []
