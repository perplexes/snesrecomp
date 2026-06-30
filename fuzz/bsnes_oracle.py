#!/usr/bin/env python3
"""bsnes_oracle.py — generate ground-truth final CPU state for a 65816 snippet by
running it on the bsnes-plus oracle (the accurate Super-FX SNES core).

This is the oracle side of the long-planned Phase-B differential fuzz
(docs/PHASE_B_DIFFERENTIAL_FUZZ.md), pointed at bsnes instead of the snes9x core
(a Star Fox dead end) AND instead of hand-computed expected values — a
hand-computed oracle can encode the same misunderstanding as the bug it is meant
to catch. The index-high-byte-on-SEP bug (fixed 2026-06-30) is the case in point:
the curated v2_stale_shadow.py snippets all did an `LDX` right after the `SEP`, so
they exercised the write-zero-extend path and never the SEP-transition-clear path.

Mechanism (emulator-agnostic, no debugger state injection): each test is wrapped
into a tiny synthetic LoROM:
    reset -> PROLOGUE (seed A/X/Y/D/DB and the m/x mode from immediates)
          -> SNIPPET under test
          -> TRAP (bra self) at a known PC
bsnes runs from reset to the trap; SF_BSNES_DUMP_AT=<trap> + SF_BSNES_WRAM_INIT=00
dump the final registers (and, with SF_BSNES_DUMP_MEM, one WRAM word).

Usage (library): rom = build_snippet_rom(snippet_bytes, init); st = run_oracle(rom, ...)
Usage (CLI self-test): python bsnes_oracle.py            # runs the SEP regression case
"""
from __future__ import annotations
import os, re, subprocess, pathlib, tempfile, time

BSNES = os.environ.get("SF_BSNES_BIN",
                       "/home/shoes/2026-glm-pi/bsnes-plus/bsnes/out/bsnes")
BSNES_DIR = pathlib.Path(BSNES).parent
TRACE_LOG = "/tmp/sf_snip-trace.log"

# --- minimal 65816 byte emitters for the prologue ---------------------------
def _imm16(op, v): return bytes([op, v & 0xFF, (v >> 8) & 0xFF])
def _imm8(op, v):  return bytes([op, v & 0xFF])

def _prologue(init):
    """Seed registers + mode from immediates. Native mode throughout."""
    A  = init.get("A", 0); X = init.get("X", 0); Y = init.get("Y", 0)
    D  = init.get("D", 0); DB = init.get("DB", 0)
    m  = init.get("m", 0); x = init.get("x", 0)
    b = bytearray()
    b += bytes([0x18, 0xFB])                 # clc, xce  -> native (e=0)
    b += bytes([0xC2, 0x30])                 # rep #$30  -> 16-bit A/X/Y
    b += _imm16(0xA9, D); b += bytes([0x5B]) # lda #D; tcd
    b += bytes([0xE2, 0x20])                 # sep #$20  -> 8-bit A (for plb)
    b += _imm8(0xA9, DB); b += bytes([0x48, 0xAB])  # lda #DB; pha; plb
    b += bytes([0xC2, 0x30])                 # rep #$30  -> 16-bit again
    b += _imm16(0xA9, A)                     # lda #A
    b += _imm16(0xA2, X)                     # ldx #X
    b += _imm16(0xA0, Y)                     # ldy #Y
    # set final mode: SEP the bits that should be 1, REP the bits that should be 0.
    set_bits   = (0x20 if m else 0) | (0x10 if x else 0)
    clear_bits = (0x20 if not m else 0) | (0x10 if not x else 0)
    if set_bits:   b += bytes([0xE2, set_bits])    # sep
    if clear_bits: b += bytes([0xC2, clear_bits])  # rep (only the index/mem bits)
    return bytes(b)

LOROM_RESET = 0x8000   # code entry = $00:8000 = file offset 0

def build_snippet_rom(snippet: bytes, init: dict):
    """Return (rom_bytes, trap_pc24). 32 KiB LoROM, headerless."""
    code = bytearray(_prologue(init))
    code += snippet
    trap_off = len(code)
    trap_pc = LOROM_RESET + trap_off                 # $00:80xx
    code += bytes([0x80, 0xFE])                       # bra self (2-byte loop)
    rom = bytearray(b"\x00" * 0x8000)                 # 32 KiB bank-0
    rom[0:len(code)] = code
    # LoROM header @ $7FC0-$7FFF
    title = b"SNIPPET FUZZ ORACLE  "[:21].ljust(21, b" ")
    rom[0x7FC0:0x7FD5] = title
    rom[0x7FD5] = 0x20    # LoROM, slow
    rom[0x7FD6] = 0x00    # ROM only
    rom[0x7FD7] = 0x05    # 32 KiB
    # native + emulation vectors: RESET -> $8000, others -> $8000 (unused)
    for off in (0x7FE4, 0x7FE6, 0x7FE8, 0x7FEA, 0x7FEC, 0x7FEE,   # native
                0x7FF4, 0x7FF8, 0x7FFA, 0x7FFC, 0x7FFE):           # emu (FFFC=reset)
        rom[off] = LOROM_RESET & 0xFF; rom[off + 1] = (LOROM_RESET >> 8) & 0xFF
    return bytes(rom), trap_pc | 0x000000

_DUMP_RE = re.compile(
    r"DUMP @ \$([0-9A-Fa-f]+)\s+A:([0-9A-Fa-f]+) X:([0-9A-Fa-f]+) Y:([0-9A-Fa-f]+) "
    r"S:([0-9A-Fa-f]+) DB:([0-9A-Fa-f]+) P:([0-9A-Fa-f]+)")

def run_oracle(rom: bytes, trap_pc: int, mem_addr: int | None = None, secs: int = 12):
    """Run the snippet ROM on bsnes; return final {A,X,Y,S,DB,P,(mem)}."""
    subprocess.run(["pkill", "-9", "bsnes"], capture_output=True); time.sleep(0.6)
    try: os.remove(TRACE_LOG)
    except OSError: pass
    with tempfile.NamedTemporaryFile(suffix=".sfc", delete=False, dir="/tmp") as f:
        f.write(rom); rompath = f.name
    # bsnes writes <basename>-trace.log next to the ROM's data dir; force our path
    env = dict(os.environ)
    env.update({"SF_BSNES_WRAM_INIT": "00",
                "SF_BSNES_DUMP_AT": f"{trap_pc:06x}:1",
                "SDL_VIDEODRIVER": "x11", "QT_QPA_PLATFORM": "xcb", "DISPLAY": ":0"})
    if mem_addr is not None:
        env["SF_BSNES_DUMP_MEM"] = f"{mem_addr:04x}"
    # the trace log lands at <data>/<rombasename>-trace.log; symlink-free: find it
    subprocess.run(["timeout", "-s", "KILL", str(secs), BSNES, rompath],
                   env=env, cwd=BSNES_DIR, capture_output=True)
    log = pathlib.Path("/tmp") / (pathlib.Path(rompath).stem + "-trace.log")
    txt = ""
    for cand in (log, pathlib.Path(TRACE_LOG)):
        if cand.exists(): txt = cand.read_text(errors="replace"); break
    m = _DUMP_RE.search(txt)
    if not m:
        return None
    g = m.groups()
    out = {"A": int(g[1], 16), "X": int(g[2], 16), "Y": int(g[3], 16),
           "S": int(g[4], 16), "DB": int(g[5], 16), "P": int(g[6], 16)}
    out["m"] = (out["P"] >> 5) & 1; out["x"] = (out["P"] >> 4) & 1
    mm = re.search(r"ram\[\$[0-9A-Fa-f]+\]=([0-9A-Fa-f]+)", txt)
    if mm: out["mem"] = int(mm.group(1), 16)
    return out


if __name__ == "__main__":
    # Regression case: the SEP-transition-clear gap the curated fuzz missed.
    #   rep #$30; ldx #$02ff; sep #$30   -> hardware forces X high byte to 0
    init = {"A": 0, "X": 0, "Y": 0, "m": 0, "x": 0}
    snippet = bytes([0xC2, 0x30,         # rep #$30  (16-bit)
                     0xA2, 0xFF, 0x02,   # ldx #$02FF
                     0xE2, 0x30])        # sep #$30  (8-bit index; X.high must clear)
    rom, trap = build_snippet_rom(snippet, init)
    print(f"trap_pc=${trap:06X}  rom={len(rom)}B")
    st = run_oracle(rom, trap)
    print("oracle final:", st)
    if st is None:
        print("FAIL: no DUMP (DISPLAY/bsnes?)")
    elif (st["X"] & 0xFF) == 0xFF and (st["X"] >> 8) == 0x00 and st["x"] == 1:
        print(f"PASS: bsnes X=${st['X']:04X} (high byte cleared by SEP, x={st['x']}) — correct ground truth")
    else:
        print(f"UNEXPECTED: X=${st['X']:04X} x={st['x']}")
