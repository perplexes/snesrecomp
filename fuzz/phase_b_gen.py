#!/usr/bin/env python3
"""phase_b_gen.py — generate random 65816 snippets and differential-fuzz them
against the bsnes oracle (phase_b machinery).

Palette is register/width/ALU/shift/flag ops with immediate operands — no memory,
no branches, no stack — so every snippet runs clean in the WRAM-only harness and
exercises exactly the width/flag/mirror logic where recomp codegen bugs live (the
class the index-high-byte bug belonged to). M/X is tracked as the sequence is
built so immediates get the correct width on both sides. Deterministic per --seed.

Usage:
  python phase_b_gen.py --count 24 [--seed 1] [--len 8] [--verbose]
"""
from __future__ import annotations
import argparse, random, sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import phase_b, bsnes_oracle

# opcode, kind. kind drives operand width: 'A'=m-width imm, 'I'=x-width imm,
# 'impl'=no operand, 'wid'=REP/SEP (operand is the mask).
PALETTE = [
    (0xA9, "A", "LDA"), (0x69, "A", "ADC"), (0xE9, "A", "SBC"),
    (0x29, "A", "AND"), (0x09, "A", "ORA"), (0x49, "A", "EOR"),
    (0xC9, "A", "CMP"), (0x89, "A", "BIT"),
    (0xA2, "I", "LDX"), (0xA0, "I", "LDY"), (0xE0, "I", "CPX"), (0xC0, "I", "CPY"),
    (0x0A, "impl", "ASL"), (0x4A, "impl", "LSR"), (0x2A, "impl", "ROL"), (0x6A, "impl", "ROR"),
    (0xE8, "impl", "INX"), (0xC8, "impl", "INY"), (0xCA, "impl", "DEX"), (0x88, "impl", "DEY"),
    (0x1A, "impl", "INA"), (0x3A, "impl", "DEA"),
    (0xAA, "impl", "TAX"), (0xA8, "impl", "TAY"), (0x8A, "impl", "TXA"), (0x98, "impl", "TYA"),
    (0x9B, "impl", "TXY"), (0xBB, "impl", "TYX"), (0x5B, "impl", "TCD"), (0x7B, "impl", "TDC"),
    (0x18, "impl", "CLC"), (0x38, "impl", "SEC"), (0xB8, "impl", "CLV"),
    (0xEA, "impl", "NOP"),
    (0xC2, "wid", "REP"), (0xE2, "wid", "SEP"),
]

def gen_snippet(rng, length):
    """Return (bytes, init, disasm[]). Tracks m/x for immediate widths."""
    init = {"A": rng.randint(0, 0xFFFF), "X": rng.randint(0, 0xFFFF),
            "Y": rng.randint(0, 0xFFFF), "DB": rng.randint(0, 0xFF),
            "m": rng.randint(0, 1), "x": rng.randint(0, 1)}
    m, x = init["m"], init["x"]
    out, dis = bytearray(), []
    for _ in range(length):
        op, kind, mnem = rng.choice(PALETTE)
        if kind == "A":
            if m: out += bytes([op, rng.randint(0, 0xFF)]); dis.append(f"{mnem} #imm8")
            else: v = rng.randint(0, 0xFFFF); out += bytes([op, v & 0xFF, v >> 8]); dis.append(f"{mnem} #imm16")
        elif kind == "I":
            if x: out += bytes([op, rng.randint(0, 0xFF)]); dis.append(f"{mnem} #imm8")
            else: v = rng.randint(0, 0xFFFF); out += bytes([op, v & 0xFF, v >> 8]); dis.append(f"{mnem} #imm16")
        elif kind == "wid":
            mask = rng.choice([0x10, 0x20, 0x30])
            out += bytes([op, mask]); dis.append(f"{mnem} #${mask:02X}")
            if op == 0xC2:  # REP -> clear
                if mask & 0x20: m = 0
                if mask & 0x10: x = 0
            else:           # SEP -> set
                if mask & 0x20: m = 1
                if mask & 0x10: x = 1
        else:
            out += bytes([op]); dis.append(mnem)
    return bytes(out), init, dis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=24)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--len", type=int, default=8)
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    rng = random.Random(a.seed)

    print(f"phase_b_gen: {a.count} snippets, seed={a.seed}, len={a.len}")
    fails = diverged = noora = 0
    for i in range(a.count):
        snip, init, dis = gen_snippet(rng, a.len)
        rec = phase_b.run_recomp(snip, init)
        if "error" in rec:
            print(f"#{i:03d} BUILDERR {rec.get('stderr','')[:160]}"); fails += 1; continue
        rom, trap = bsnes_oracle.build_snippet_rom(snip, init)
        ora = bsnes_oracle.run_oracle(rom, trap)
        if ora is None:
            noora += 1
            if a.verbose: print(f"#{i:03d} NO-ORACLE")
            continue
        def norm(st):
            st = dict(st)
            if st.get("x") == 1: st["X"] &= 0xFF; st["Y"] &= 0xFF
            if st.get("m") == 1: st["A"] &= 0xFFFF  # A keeps B; compare full
            return st
        r, o = norm(rec), norm(ora)
        diffs = [f"{k}:rec={r.get(k):#x}!=ora={o.get(k):#x}"
                 for k in phase_b.FIELDS if r.get(k) != o.get(k)]
        if diffs:
            diverged += 1
            print(f"#{i:03d} DIVERGE  init={{m:{init['m']} x:{init['x']} A:{init['A']:04X} X:{init['X']:04X} Y:{init['Y']:04X}}}")
            print(f"        snippet: {' ; '.join(dis)}")
            print(f"        bytes:   {snip.hex()}")
            print(f"        {' '.join(diffs)}")
        elif a.verbose:
            print(f"#{i:03d} OK   {' ; '.join(dis)}")
    print(f"\nran {a.count}: {diverged} diverged, {fails} builderr, {noora} no-oracle, "
          f"{a.count - diverged - fails - noora} OK")
    return 1 if (diverged or fails) else 0


if __name__ == "__main__":
    sys.exit(main())
