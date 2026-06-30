#!/usr/bin/env python3
"""coverage.py — a fuzz coverage ledger keyed by (mnem, addressing mode, m, x).

A passing snippet weakly verifies every (opcode, mode, m, x) it contains (they
matched bsnes jointly); a failing one marks them suspect. Accumulated across runs
(JSON file), this turns "the last sweep found nothing" into "these N combos are
verified, these M are still dark" — so green means something.

The target space is the generator's PALETTE x reachable (m,x). `report` decodes
each palette op at each (m,x) to get its canonical mode, then diffs against what
the ledger has seen.
"""
from __future__ import annotations
import json, pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import phase_b


def _key(insn) -> str:
    return f"{insn.mnem}|{insn.mode}|m{insn.m_flag}|x{insn.x_flag}"


def _op_bytes(op, kind, m, x) -> bytes:
    """Minimal valid encoding of a palette op at width (m,x), for decode."""
    if kind == "A":   return bytes([op, 0]) if m else bytes([op, 0, 0])
    if kind == "I":   return bytes([op, 0]) if x else bytes([op, 0, 0])
    if kind in ("dp", "dpx"): return bytes([op, 0x10])
    if kind == "wid": return bytes([op, 0x30])
    return bytes([op])


class Ledger:
    def __init__(self, path):
        self.path = pathlib.Path(path)
        self.d = json.loads(self.path.read_text()) if self.path.exists() else {}

    def record(self, insns, passed: bool):
        for i in insns:
            e = self.d.setdefault(_key(i), [0, 0])
            e[0 if passed else 1] += 1

    def save(self):
        self.path.write_text(json.dumps(self.d, indent=0, sort_keys=True))

    def target(self, palette):
        keys = set()
        for op, kind, mnem in palette:
            for m in (0, 1):
                for x in (0, 1):
                    try:
                        insns = phase_b.decode_snippet(_op_bytes(op, kind, m, x), m, x)
                        keys.add(_key(insns[0]))
                    except Exception:
                        pass
        return keys

    def report(self, palette):
        tgt = self.target(palette)
        seen = set(self.d.keys())
        verified = {k for k, v in self.d.items() if v[0] > 0 and v[1] == 0}
        suspect = sorted(k for k, v in self.d.items() if v[1] > 0)
        dark = sorted(tgt - seen)
        cov = len(verified & tgt)
        print(f"coverage: {cov}/{len(tgt)} target combos verified "
              f"({100*cov/max(1,len(tgt)):.0f}%), {len(suspect)} suspect, {len(dark)} dark")
        if suspect:
            print("  suspect:", ", ".join(suspect[:20]))
        if dark:
            print("  dark   :", ", ".join(dark[:30]) + (" ..." if len(dark) > 30 else ""))
        return cov, len(tgt)


if __name__ == "__main__":
    import argparse, phase_b_gen
    ap = argparse.ArgumentParser()
    ap.add_argument("--ledger", default="fuzz/coverage.json")
    ap.add_argument("--mem", action="store_true")
    a = ap.parse_args()
    pal = ([p for p in phase_b_gen.PALETTE if p[2] not in phase_b_gen._MEM_UNSAFE_MNEM]
           if a.mem else [p for p in phase_b_gen.PALETTE if p[1] not in ("dp", "dpx")])
    Ledger(a.ledger).report(pal)
