"""snesrecomp.recompiler.v2.exit_mx_autoroute

Auto-detect cfg `exit_mx_at` directive sites — leaf-function variant only.

The cfg `exit_mx_at <addr> <m> <x>` directive declares that the callee at
`<addr>` exits with (M, X) = (m, x). The v2 decoder consults this when
emitting JSR/JSL fall-through edges so the caller resumes decoding with
the right operand widths after the call. Without the directive the
decoder assumes (M, X) are preserved — wrong whenever the callee runs
an internal SEP/REP that never restores before RTS/RTL. SMW's
`$00:F465` is the canonical case (`SEP #$20` first, no restore; root
cause of "Mario dies on slope" 2026-05-03).

The directive is opt-in. An earlier (2026-05-03) attempt to auto-infer
it over EVERY decoded (addr, m, x) variant via fixpoint regressed
GraphicsDecompress into an infinite loop — the fixpoint produced
intermediate exit-(M,X) values that biased the analyzer along
unreachable paths. That attempt was reverted to opt-in.

This pass takes a DIFFERENT, narrower approach: it only auto-detects
LEAF functions (no JSR/JSL anywhere in the decoded body). Leaf
functions' exit (M, X) is determined purely by their own SEP/REP and
entry state — no callee dependency, no fixpoint required, no regression
risk from intermediate state. The trade-off: non-leaf functions whose
exits depend on their callees (like F461/F465 themselves, which call
into deeper routines) are still opt-in via cfg directive.

**Per-variant detection (2026-05-15).** Original implementation
committed ONE exit per function (the cfg-declared entry's exit, or
the convergent exit when all four variants agreed). v2_regen then
broadcast that single tuple to all four entry variants when building
`callee_exit_mx`. That broadcast corrupts post-JSR state for any
entry variant whose actual exit differs from the broadcast — typical
when a leaf does `REP #$20` (only touches M) or `SEP #$10` (only
touches X). Example: `ManipulateMode7Image_008B2B` (SineAndScale, the
Mode-7 multiplier helper) does REP #$20: entry (0,0) → exit (0,0) but
entry (1,1) → exit (0,1). Broadcast applied (0,1) everywhere, so the
M0X0 caller after `JSR SineAndScale` was told the callee returns with
x=1; the decoder then emitted only `L_8B01_M0X1` for the fall-through
PC, the `CPY #$0004` got mis-decoded as `CPY #$04` + phantom BRK, and
`CalculateMode7Values` early-returned without writing
`Mode7ParamA/B/C/D`. The Mode-7 BG matrix never updated and the Iggy
boss-arena tilting platform rendered invisible (Bug C, 2026-05-15).

Fix: decode all four (M, X) combos for every cfg `func` entry; for
each variant where exit ≠ entry, emit a per-variant record in
`BankCfg.exit_mx_at_per_variant` (6-tuple: bank, addr16, entry_m,
entry_x, exit_m, exit_x). v2_regen prefers per-variant entries over
the legacy 4-tuple broadcast when populating callee_exit_mx, so each
JSR fall-through resolves to the correct exit for its specific entry
variant.

The legacy 4-tuple `cfg.exit_mx_at` list still receives the
cfg-declared variant's exit (when it mutates) for backward compat
with any consumers expecting one-tuple-per-fn semantics; per-variant
takes precedence at lookup time.

Public API:
    detect_and_route(parsed, rom) -> List[FixRecord]
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Set, Tuple

_THIS_DIR = Path(__file__).resolve().parent
_RECOMPILER_DIR = _THIS_DIR.parent
for p in (str(_THIS_DIR), str(_RECOMPILER_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

from v2.decoder import decode_function, analyze_function_exit_mx  # noqa: E402


@dataclass(frozen=True)
class FixRecord:
    """One leaf function variant whose exit (M, X) state differs from
    entry. Per-variant since 2026-05-15: a single function may emit
    multiple FixRecords (one per mutating entry variant)."""
    bank: int
    addr16: int             # 16-bit local PC of the leaf function
    fn_name: str
    entry_m: int
    entry_x: int
    exit_m: int
    exit_x: int

    @property
    def addr_24(self) -> int:
        return (self.bank << 16) | (self.addr16 & 0xFFFF)


def _graph_has_call(graph) -> bool:
    """Return True if the decoded graph contains any JSR or JSL."""
    for di in graph.insns.values():
        if di.insn.mnem in ('JSR', 'JSL'):
            return True
    return False


_MX_COMBOS: List[Tuple[int, int]] = [(0, 0), (0, 1), (1, 0), (1, 1)]


def _decode_leaf_exit(rom: bytes, bank: int, addr16: int,
                      em: int, ex: int, end):
    """Decode (bank, addr16) with entry (em, ex). Returns
    (exit_m, exit_x) if the body decoded cleanly AND is a leaf with a
    determinable exit, else None."""
    try:
        graph = decode_function(rom, bank, addr16,
                                entry_m=em, entry_x=ex, end=end)
    except Exception:
        return None
    if not graph.insns:
        return None
    if _graph_has_call(graph):
        return None
    exit_m, exit_x = analyze_function_exit_mx(graph)
    if exit_m is None or exit_x is None:
        return None
    return (exit_m & 1, exit_x & 1)


def detect_and_route(parsed, rom: bytes) -> List[FixRecord]:
    """Auto-detect per-variant leaf-function exit-(M, X) mutations.

    For every cfg `func` entry F that has no cfg-declared `exit_mx_at`:

      1. Decode F under all four entry combos: (0,0), (0,1), (1,0),
         (1,1). For each combo: skip if decode fails, the body has any
         JSR/JSL (non-leaf), or analyze_function_exit_mx returns
         ambiguous (None) for either component.
      2. For each variant where exit ≠ entry, append a per-variant
         entry to `cfg.exit_mx_at_per_variant`:
             (bank, addr16, entry_m, entry_x, exit_m, exit_x).
      3. Additionally, if the cfg-declared entry variant mutates, also
         append its exit to the legacy `cfg.exit_mx_at` 4-tuple list
         for any consumers that still rely on it. v2_regen prefers
         per-variant over the broadcast.

    Per-variant records preserve variant-specific state for leafs that
    only touch one of M/X — e.g. SineAndScale at $00:8B2B does
    `REP #$20` (forces m=0, leaves x at entry value):

        entry (0,0) -> exit (0,0)  ← no mutation, no record
        entry (0,1) -> exit (0,1)  ← no mutation, no record
        entry (1,0) -> exit (0,0)  ← M mutates: record
        entry (1,1) -> exit (0,1)  ← M mutates: record

    Without per-variant records, the legacy broadcast would smear the
    cfg-default's (1,1)→(0,1) exit across ALL four entries, including
    (0,0)'s post-JSR fall-through state — the Bug C class.

    Sites already covered by a cfg-written `exit_mx_at` directive are
    skipped (cfg-declared wins; per-variant auto-detection respects
    explicit cfg facts).
    """
    fixes: List[FixRecord] = []

    declared: Set[Tuple[int, int]] = set()
    for bank, _cfg_path, cfg in parsed:
        for (b_id, addr16, _m, _x) in cfg.exit_mx_at:
            declared.add((b_id & 0xFF, addr16 & 0xFFFF))

    seen_keys: Set[Tuple[int, int]] = set()

    for bank, _cfg_path, cfg in parsed:
        for entry in cfg.entries:
            if not entry.name:
                continue
            addr16 = entry.start & 0xFFFF
            if (bank, addr16) in declared:
                continue  # cfg-declared wins
            if (bank, addr16) in seen_keys:
                continue

            em_in = entry.entry_m & 1
            ex_in = entry.entry_x & 1

            # Decode every (M, X) combo. Skip the function entirely if
            # ANY variant fails to decode cleanly, has a JSR/JSL (non-
            # leaf), or has an ambiguous exit. Conservative: avoids
            # committing partial information that could mislead the
            # decoder for the variant that succeeded.
            entry_exits: List[Tuple[int, int, int, int]] = []
            ok = True
            for em, ex in _MX_COMBOS:
                e = _decode_leaf_exit(rom, bank, addr16, em, ex,
                                      entry.end)
                if e is None:
                    ok = False
                    break
                entry_exits.append((em, ex, e[0], e[1]))
            if not ok:
                continue

            seen_keys.add((bank, addr16))

            # Emit one per-variant record for each variant that mutates.
            # The non-mutating variants don't need a record — the
            # decoder's default `ret_m, ret_x = post_m, post_x` after
            # JSR/JSL already gives the correct fall-through state for
            # them (callee preserves the caller's M/X).
            #
            # Deliberately NOT also appending to the legacy 4-tuple
            # `cfg.exit_mx_at`. That list broadcasts ONE exit to ALL
            # entry variants of a target, which corrupts the very
            # variants we're trying to preserve. The original Bug C
            # symptom is exactly this: cfg-default M1X1→M0X1 was
            # broadcast to the M0X0 caller, poisoning the post-JSR
            # fall-through key. Per-variant alone is sufficient —
            # mutating entries get explicit records, non-mutating
            # entries rely on the decoder default.
            for em, ex, exit_m, exit_x in entry_exits:
                if em == exit_m and ex == exit_x:
                    continue
                cfg.exit_mx_at_per_variant.append(
                    (bank, addr16, em, ex, exit_m, exit_x))
                fixes.append(FixRecord(
                    bank=bank, addr16=addr16, fn_name=entry.name,
                    entry_m=em, entry_x=ex,
                    exit_m=exit_m, exit_x=exit_x,
                ))

    return fixes


def format_fix_summary(fixes: List[FixRecord]) -> str:
    """Build a human-readable summary block."""
    if not fixes:
        return "  no leaf-function exit-(M, X) mutations detected"
    lines = [f"  detected {len(fixes)} leaf-function exit-(M, X) mutation(s); auto-routed:"]
    for fx in fixes:
        lines.append(
            f"    ${fx.bank:02X}:{fx.addr16:04X} {fx.fn_name!r} "
            f"entry M={fx.entry_m} X={fx.entry_x} "
            f"-> exit M={fx.exit_m} X={fx.exit_x}"
        )
    return "\n".join(lines)
