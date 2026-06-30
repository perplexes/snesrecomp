#!/usr/bin/env python3
"""phase_b_gen.py — generate random 65816 snippets and differential-fuzz them
against the bsnes oracle (phase_b machinery).

Default (reg) mode: register/width/ALU/shift/flag ops with immediates — no memory,
exercising the width/flag/mirror logic where codegen bugs live. --mem mode: DP-read
addressing into a seeded scratch region (D=0, no D-movers or indexed reads) so every
read lands in WRAM and the register/flag compare catches address-computation bugs.
M/X is tracked so immediates get the right width. Deterministic per --seed.
--batch N runs N snippets per bsnes boot (the throughput unlock).

Usage:
  python phase_b_gen.py --count 30 [--seed 1] [--len 8] [--mem] [--batch 30]
"""
from __future__ import annotations
import argparse, random, sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import phase_b, bsnes_oracle
import coverage as _cov

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
    # memory reads from the seeded scratch region ($10..$3F). The loaded value /
    # flags land in registers, so the register+P compare catches address-comp bugs
    # (DP, DP-indexed wrap, index-width in the effective address).
    (0xA5, "dp", "LDA $dp"), (0x65, "dp", "ADC $dp"), (0xE5, "dp", "SBC $dp"),
    (0x25, "dp", "AND $dp"), (0x05, "dp", "ORA $dp"), (0x45, "dp", "EOR $dp"),
    (0xC5, "dp", "CMP $dp"), (0x24, "dp", "BIT $dp"),
    (0xA6, "dp", "LDX $dp"), (0xA4, "dp", "LDY $dp"),
    (0xE4, "dp", "CPX $dp"), (0xC4, "dp", "CPY $dp"),
    (0xB5, "dpx", "LDA $dp,X"), (0x75, "dpx", "ADC $dp,X"), (0x35, "dpx", "AND $dp,X"),
    (0x55, "dpx", "EOR $dp,X"), (0xD5, "dpx", "CMP $dp,X"), (0xB4, "dpx", "LDY $dp,X"),
]

# Ops that move the effective DP base (D) or use an uncontrolled index — they can
# push a memory read out of WRAM scratch into MMIO ($00:2000+), where the
# flat-buffer recomp harness and bsnes legitimately differ. Excluded from --mem.
_MEM_UNSAFE_MNEM = {"TCD", "TDC"}     # change D
_MEM_UNSAFE_KIND = {"dpx"}            # index could exceed scratch -> MMIO

def gen_snippet(rng, length, mem=False):
    """Return (bytes, init, disasm[]). Tracks m/x for immediate widths.
    mem=False: register/width/flag/ALU palette, NO memory (the broad default).
    mem=True:  DP and DP,X addressing into the seeded scratch. To keep every
               effective address inside WRAM (never MMIO), x is pinned to 1 (8-bit
               index, so dp<=$3F + X<=$FF <= $13E), D pinned 0, D-movers excluded,
               and REP may not clear x (only its m bit)."""
    if mem:
        palette = [p for p in PALETTE if p[2] not in _MEM_UNSAFE_MNEM]
    else:
        palette = [p for p in PALETTE if p[1] not in ("dp", "dpx")]
    init = {"A": rng.randint(0, 0xFFFF), "X": rng.randint(0, 0xFF if mem else 0xFFFF),
            "Y": rng.randint(0, 0xFFFF), "DB": rng.randint(0, 0xFF),
            "D": 0,  # D held 0 so DP reads land in the seeded scratch
            "m": rng.randint(0, 1), "x": 1 if mem else rng.randint(0, 1)}
    m, x = init["m"], init["x"]
    out, dis = bytearray(), []
    for _ in range(length):
        op, kind, mnem = rng.choice(palette)
        if kind == "A":
            if m: out += bytes([op, rng.randint(0, 0xFF)]); dis.append(f"{mnem} #imm8")
            else: v = rng.randint(0, 0xFFFF); out += bytes([op, v & 0xFF, v >> 8]); dis.append(f"{mnem} #imm16")
        elif kind == "I":
            if x: out += bytes([op, rng.randint(0, 0xFF)]); dis.append(f"{mnem} #imm8")
            else: v = rng.randint(0, 0xFFFF); out += bytes([op, v & 0xFF, v >> 8]); dis.append(f"{mnem} #imm16")
        elif kind in ("dp", "dpx"):
            out += bytes([op, rng.randint(0x10, 0x3F)]); dis.append(mnem)
        elif kind == "wid":
            # in mem mode REP must not clear x (would let a 16-bit index escape to
            # MMIO); allow only its m bit. SEP is fine (it only sets bits).
            masks = ([0x20] if (mem and op == 0xC2) else [0x10, 0x20, 0x30])
            mask = rng.choice(masks)
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


def report(i, init, dis, snip, diffs, diverged_box, verbose):
    if diffs:
        diverged_box[0] += 1
        print(f"#{i:03d} DIVERGE  init={{m:{init['m']} x:{init['x']} A:{init['A']:04X} X:{init['X']:04X} Y:{init['Y']:04X}}}")
        print(f"        snippet: {' ; '.join(dis)}")
        print(f"        bytes:   {snip.hex()}")
        print(f"        {' '.join(diffs)}")
    elif verbose:
        print(f"#{i:03d} OK   {' ; '.join(dis)}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=24)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--len", type=int, default=8)
    ap.add_argument("--mem", action="store_true", help="memory-addressing mode (DP reads, D=0, clean scratch)")
    ap.add_argument("--batch", type=int, default=0,
                    help="snippets per bsnes boot (0=one-per-boot). Batched is ~Nx faster.")
    ap.add_argument("--ledger", default="", help="coverage-ledger JSON to accumulate into + report")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    rng = random.Random(a.seed)
    print(f"phase_b_gen: {a.count} snippets, seed={a.seed}, len={a.len}, batch={a.batch}, mode={'mem' if a.mem else 'reg'}")
    cases = [gen_snippet(rng, a.len, mem=a.mem) for _ in range(a.count)]   # (snip, init, dis)
    fails = noora = 0
    dbox = [0]
    led = _cov.Ledger(a.ledger) if a.ledger else None

    def handle(i, snip, init, dis, rec, ora):
        nonlocal fails
        if "error" in rec:
            print(f"#{i:03d} BUILDERR {rec.get('stderr','')[:160]}"); fails += 1; return
        diffs = phase_b.compare(rec, ora)
        report(i, init, dis, snip, diffs, dbox, a.verbose)
        if led is not None:
            led.record(phase_b.decode_snippet(snip, init["m"], init["x"]), not diffs)

    if a.batch > 0:
        for g in range(0, a.count, a.batch):
            grp = cases[g:g + a.batch]
            recs = [phase_b.run_recomp(s, ini) for (s, ini, _) in grp]
            rom, trap = bsnes_oracle.build_batch_rom([(s, ini) for (s, ini, _) in grp])
            oras = bsnes_oracle.run_oracle_batch(rom, trap, len(grp))
            if oras is None:
                noora += len(grp); print(f"group @{g}: NO-ORACLE (batch dump failed)"); continue
            for j, ((snip, init, dis), rec, ora) in enumerate(zip(grp, recs, oras)):
                handle(g + j, snip, init, dis, rec, ora)
    else:
        for i, (snip, init, dis) in enumerate(cases):
            rec = phase_b.run_recomp(snip, init)
            rom, trap = bsnes_oracle.build_snippet_rom(snip, init)
            ora = bsnes_oracle.run_oracle(rom, trap)
            if ora is None and "error" not in rec:
                noora += 1
                if a.verbose: print(f"#{i:03d} NO-ORACLE")
                continue
            handle(i, snip, init, dis, rec, ora)

    diverged = dbox[0]
    print(f"\nran {a.count}: {diverged} diverged, {fails} builderr, {noora} no-oracle, "
          f"{a.count - diverged - fails - noora} OK")
    if led is not None:
        led.save()
        pal = ([p for p in PALETTE if p[2] not in _MEM_UNSAFE_MNEM] if a.mem
               else [p for p in PALETTE if p[1] not in ("dp", "dpx")])
        led.report(pal)
    return 1 if (diverged or fails) else 0


if __name__ == "__main__":
    sys.exit(main())
