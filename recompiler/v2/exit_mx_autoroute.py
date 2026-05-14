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

Mutates each `BankCfg.exit_mx_at` list in place with the inferred
tuples, so the existing builder at `v2_regen.py:342+` picks them up
when constructing `callee_exit_mx`.

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
    """One leaf function whose exit (M, X) state differs from entry."""
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


def detect_and_route(parsed, rom: bytes) -> List[FixRecord]:
    """Auto-detect leaf-function exit-(M, X) state mutations.

    For each cfg `func` entry F:
      1. Decode F with its declared (entry_m, entry_x).
      2. Skip if the decoded body contains any JSR or JSL (non-leaf).
      3. Compute exit (M, X) via `analyze_function_exit_mx`.
      4. Skip if either component is ambiguous (None) or matches the
         entry state (no inference needed).
      5. Skip if a cfg `exit_mx_at` directive already exists at F's PC.
      6. Otherwise, append `(bank, F.start & 0xFFFF, exit_m, exit_x)`
         to F's owning BankCfg's `exit_mx_at` list.

    Returns the list of applied fixes.
    """
    fixes: List[FixRecord] = []

    # Index of (bank, addr16) → BankCfg, used so we can mutate the
    # owning cfg's exit_mx_at list. Also collect already-declared sites
    # to skip them.
    by_bank = {bank: cfg for (bank, _path, cfg) in parsed}
    declared: Set[Tuple[int, int]] = set()
    for bank, _cfg_path, cfg in parsed:
        for (b_id, addr16, _m, _x) in cfg.exit_mx_at:
            declared.add((b_id & 0xFF, addr16 & 0xFFFF))

    # Avoid double-detecting the same function via multiple cfg entries
    # at the same PC (defensive — shouldn't happen but cheap to guard).
    seen_keys: Set[Tuple[int, int, int, int]] = set()

    for bank, _cfg_path, cfg in parsed:
        for entry in cfg.entries:
            if not entry.name:
                continue
            addr16 = entry.start & 0xFFFF
            if (bank, addr16) in declared:
                continue  # cfg-declared wins
            em_in = entry.entry_m & 1
            ex_in = entry.entry_x & 1
            key = (bank, addr16, em_in, ex_in)
            if key in seen_keys:
                continue

            try:
                graph = decode_function(
                    rom, bank, addr16,
                    entry_m=em_in, entry_x=ex_in,
                    end=entry.end,
                )
            except Exception:
                continue
            if not graph.insns:
                continue

            # Leaf restriction: any JSR/JSL means the function's exit
            # state depends on a callee's exit state. Skip — the
            # fixpoint that would resolve it is exactly what regressed
            # the prior attempt.
            if _graph_has_call(graph):
                continue

            exit_m, exit_x = analyze_function_exit_mx(graph)
            if exit_m is None or exit_x is None:
                continue
            if exit_m == em_in and exit_x == ex_in:
                continue

            seen_keys.add(key)
            # Mutate the owning cfg so the existing builder picks it up.
            cfg.exit_mx_at.append((bank, addr16, exit_m & 1, exit_x & 1))
            fixes.append(FixRecord(
                bank=bank, addr16=addr16, fn_name=entry.name,
                entry_m=em_in, entry_x=ex_in,
                exit_m=exit_m & 1, exit_x=exit_x & 1,
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
