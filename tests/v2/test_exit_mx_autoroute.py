"""Pin v2 exit_mx_autoroute behaviour (per-variant, non-leaf-capable).

`detect_and_route` decodes every cfg `func` entry under all four
(entry_m, entry_x) combos. For each variant where the decode succeeds
and the exit (m, x) is unambiguous, it records a per-variant tuple
in `BankCfg.exit_mx_at_per_variant`. Non-leaf functions (with internal
JSR/JSL) are included; iterative fixpoint with monotonic information
flow handles their callee dependencies.

Tests pin:
  - Canonical leaf shapes (REP/SEP-only patterns).
  - Non-leaf SEP-then-RTS dispatcher class (the F9C9 / Layer1_Init
    shape that was the cfg-hint motivation across three sessions).
  - Per-variant recording: X-preserving leafs record only mutating
    entries; non-mutating entries are absent (decoder default
    suffices).
  - Hand-written cfg `exit_mx_at` directives win at seeded keys.
  - Ambiguous exits skip (analyzer returns None for either component).
  - Bounded iteration (no infinite fixpoint).

History: 2026-05-03 first non-leaf attempt regressed GraphicsDecompress
into an infinite loop via unsound intermediate (m, x) propagation
across the PHP/PLP gap. PHP/PLP tracking landed in snesrecomp 73e3d26
(2026-05-15); this allowed dropping the leaf-only restriction safely
in 2026-05-16.
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
    # Hand-written 4-tuple directives. Auto-router does NOT write to
    # this list; it seeds callee_exit_mx from it before its own
    # analysis runs (hand-written wins).
    exit_mx_at: list = field(default_factory=list)
    # Per-variant exits the auto-router commits.
    exit_mx_at_per_variant: list = field(default_factory=list)


def _per_variant_set(cfg):
    """Helper: return a set of (bank, addr16, em, ex, exit_m, exit_x)
    for assertion comparisons."""
    return {tuple(t) for t in cfg.exit_mx_at_per_variant}


# ── Leaf shapes (regression coverage for prior leaf-only behavior) ─────

def test_rep_20_only_records_x_preserving_mutators():
    """REP #$20 ; RTS — exit M=0 always, X preserved.

    Per-variant table:
      (0, 0) -> (0, 0)  no mutation
      (0, 1) -> (0, 1)  no mutation
      (1, 0) -> (0, 0)  M mutates only
      (1, 1) -> (0, 1)  M mutates only

    Auto-router records the two mutating entries; non-mutating are
    omitted (decoder default = preserve = correct).
    """
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])

    fixes = detect_and_route([(0x00, 'bank00.cfg', cfg)], rom)

    pv = _per_variant_set(cfg)
    assert (0x00, 0x8000, 1, 0, 0, 0) in pv  # M-mutates, X stays 0
    assert (0x00, 0x8000, 1, 1, 0, 1) in pv  # M-mutates, X stays 1
    # Non-mutating variants are NOT recorded.
    assert (0x00, 0x8000, 0, 0, 0, 0) not in pv
    assert (0x00, 0x8000, 0, 1, 0, 1) not in pv
    # cfg.exit_mx_at stays empty (auto-router doesn't touch the
    # broadcast list).
    assert cfg.exit_mx_at == []


def test_rep_30_records_three_mutators():
    """REP #$30 ; RTS — exit (m=0, x=0) regardless of entry.

    Per-variant table:
      (0, 0) -> (0, 0)  no mutation
      (0, 1) -> (0, 0)  X mutates
      (1, 0) -> (0, 0)  M mutates
      (1, 1) -> (0, 0)  M and X mutate
    """
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x30, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])

    detect_and_route([(0x00, 'bank00.cfg', cfg)], rom)

    pv = _per_variant_set(cfg)
    assert (0x00, 0x8000, 0, 1, 0, 0) in pv
    assert (0x00, 0x8000, 1, 0, 0, 0) in pv
    assert (0x00, 0x8000, 1, 1, 0, 0) in pv
    assert (0x00, 0x8000, 0, 0, 0, 0) not in pv  # non-mutating
    assert cfg.exit_mx_at == []


def test_sep_only_x_flag():
    """SEP #$10 ; RTS — exit X=1 always, M preserved."""
    rom = make_lorom_bank0({
        0x8000: bytes([0xE2, 0x10, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000, entry_m=0, entry_x=0)
    cfg = _BankCfg(bank=0x00, entries=[F])

    detect_and_route([(0x00, 'bank00.cfg', cfg)], rom)

    pv = _per_variant_set(cfg)
    assert (0x00, 0x8000, 0, 0, 0, 1) in pv  # X-mutates
    assert (0x00, 0x8000, 1, 0, 1, 1) in pv  # X-mutates
    # Non-mutating (entry_x=1) variants are absent.
    assert all(t[3] == 0 for t in cfg.exit_mx_at_per_variant)


def test_two_leafs_in_one_bank_independent():
    """Two unrelated leaf state-mutators in one bank — both detected
    per-variant."""
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0x60]),  # REP #$20 ; RTS — M-only
        0x9000: bytes([0xC2, 0x10, 0x60]),  # REP #$10 ; RTS — X-only
    })
    F = _BankEntry(name='F', start=0x8000)
    G = _BankEntry(name='G', start=0x9000)
    cfg = _BankCfg(bank=0x00, entries=[F, G])

    detect_and_route([(0x00, 'bank00.cfg', cfg)], rom)

    pv = _per_variant_set(cfg)
    assert (0x00, 0x8000, 1, 0, 0, 0) in pv
    assert (0x00, 0x8000, 1, 1, 0, 1) in pv
    assert (0x00, 0x9000, 0, 1, 0, 0) in pv
    assert (0x00, 0x9000, 1, 1, 1, 0) in pv


# ── Non-leaf shape (the F9C9 / Layer1_Init class) ──────────────────────

def test_non_leaf_sep_then_jsl_then_rts():
    """The canonical SMW dispatcher shape:

        SEP #$30       ; force m=x=1
        JSL <leaf>     ; call helper that preserves m,x
        RTS

    Exit (m, x) = (1, 1) regardless of entry. The leaf-only auto-router
    skipped this class entirely because of the JSL — but with the
    leaf-only restriction dropped, the analyzer now sees that all RTS
    paths exit at (1, 1) and records per-variant entries for every
    mutating entry.

    This is the F9C9 / Layer1_Init bug class that motivated three
    cfg hints across three sessions before this fix.
    """
    rom = make_lorom_bank0({
        # F at $8000: SEP #$30 ; JSL $018100 ; RTS
        0x8000: bytes([0xE2, 0x30, 0x22, 0x00, 0x81, 0x01, 0x60]),
        # Leaf callee at $01:8100 — preserves m,x via REP/SEP balance.
        # Won't reach this in this test since callee is in a different
        # bank and not represented; the auto-router needs to handle
        # the unknown-callee case.
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])

    detect_and_route([(0x00, 'bank00.cfg', cfg)], rom)

    pv = _per_variant_set(cfg)
    # Every entry variant should converge at exit (1, 1) because the
    # SEP #$30 forces m=x=1 before the JSL, and the JSL's return state
    # is unknown (decoder default = preserve = (1, 1) at this point).
    # Variants entering at (1, 1) don't mutate; the other three do.
    assert (0x00, 0x8000, 0, 0, 1, 1) in pv
    assert (0x00, 0x8000, 0, 1, 1, 1) in pv
    assert (0x00, 0x8000, 1, 0, 1, 1) in pv
    # (1, 1) entry → (1, 1) exit: no mutation, no record.
    assert (0x00, 0x8000, 1, 1, 1, 1) not in pv


def test_non_leaf_php_plp_balanced_preserves_entry():
    """PHP-bracketed body: SEP inside, PLP restores entry M/X. Exit
    matches entry exactly — no per-variant record needed at any entry.

    Decoder's PHP/PLP tracking (snesrecomp 73e3d26) is what makes this
    sound. Without it, the analyzer would see exit at the post-SEP
    state instead of entry-restored.
    """
    rom = make_lorom_bank0({
        # F at $8000: PHP ; SEP #$30 ; PLP ; RTS
        0x8000: bytes([0x08, 0xE2, 0x30, 0x28, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])

    detect_and_route([(0x00, 'bank00.cfg', cfg)], rom)

    # All four entry variants exit at the same (m, x) as they entered —
    # no mutation, no records.
    assert cfg.exit_mx_at_per_variant == []


# ── Soundness gates ────────────────────────────────────────────────────

def test_ambiguous_exit_skipped():
    """Two RTS paths exiting with different (M, X) — analyzer returns
    None for the disagreeing component. Per-variant record skipped."""
    # $8000: LDA $05        (A5 05)
    # $8002: BEQ $8008      (F0 04)
    # $8004: REP #$20       (C2 20)
    # $8006: RTS            (60)
    # $8007: NOP            (EA)
    # $8008: RTS            (60)   — exit M=entry; the other RTS exits M=0
    rom = make_lorom_bank0({
        0x8000: bytes([0xA5, 0x05, 0xF0, 0x04, 0xC2, 0x20, 0x60, 0xEA, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])

    detect_and_route([(0x00, 'bank00.cfg', cfg)], rom)

    # M is ambiguous (one path exits m=0, the other preserves entry m) →
    # every entry variant produces ambiguous exit → no records.
    assert cfg.exit_mx_at_per_variant == []


def test_no_terminator_skipped():
    """Function with no RTS/RTL — analyzer can't determine an exit."""
    # $8000: BRA $8000   (80 FE)
    rom = make_lorom_bank0({
        0x8000: bytes([0x80, 0xFE]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])

    detect_and_route([(0x00, 'bank00.cfg', cfg)], rom)

    assert cfg.exit_mx_at_per_variant == []


def test_unnamed_entry_skipped():
    """Entries with name=None are scaffolding, not real functions."""
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0x60]),
    })
    F = _BankEntry(name=None, start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])

    detect_and_route([(0x00, 'bank00.cfg', cfg)], rom)

    assert cfg.exit_mx_at_per_variant == []


# ── Hand-written cfg directives win ────────────────────────────────────

def test_cfg_declared_exit_mx_seeds_callee_map_and_suppresses_auto():
    """A hand-written cfg `exit_mx_at` is seeded into callee_exit_mx
    BEFORE the auto-router's analysis runs — all 4 entry variants get
    the broadcast tuple. The auto-router's "skip already-known keys"
    check then leaves the function entirely alone.

    This is the contract: hand-written wins (the user is presumed to
    have a reason — e.g., ROM-specific knowledge the analyzer can't
    derive from bytes alone). Per-variant auto-records are suppressed
    even if the body would otherwise produce them.

    If the user wants per-variant correctness instead, they delete
    the hand-written 4-tuple and let the auto-router populate
    cfg.exit_mx_at_per_variant from scratch.
    """
    # $8000: REP #$20 ; RTS — auto-detected exit would be (m=0, x=entry)
    rom = make_lorom_bank0({
        0x8000: bytes([0xC2, 0x20, 0x60]),
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(
        bank=0x00, entries=[F],
        exit_mx_at=[(0x00, 0x8000, 1, 1)],  # hand-written says (1, 1)
    )

    detect_and_route([(0x00, 'bank00.cfg', cfg)], rom)

    # The hand-written 4-tuple is preserved.
    assert cfg.exit_mx_at == [(0x00, 0x8000, 1, 1)]
    # No per-variant records emitted — the seed claimed all 4 entries.
    assert cfg.exit_mx_at_per_variant == []


# ── Bounded iteration ─────────────────────────────────────────────────

def test_iteration_terminates_for_self_recursive():
    """A function that BRAs to itself is non-terminating in asm; the
    auto-router must NOT spin. With no RTS reachable, the exit is
    indeterminate and gets skipped."""
    rom = make_lorom_bank0({
        0x8000: bytes([0x80, 0xFE]),  # BRA $8000
    })
    F = _BankEntry(name='F', start=0x8000)
    cfg = _BankCfg(bank=0x00, entries=[F])

    # Should return quickly. If it doesn't, the test runner times out
    # before this assertion fires.
    detect_and_route([(0x00, 'bank00.cfg', cfg)], rom)
    assert cfg.exit_mx_at_per_variant == []
