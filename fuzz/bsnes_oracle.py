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
    """Seed registers + mode + flags from immediates. Native mode throughout.

    The full P byte is set LAST via PEA+PLP — PEA pushes a 16-bit immediate
    touching no register and no flag, and PLP pulls the low byte into P — so the
    final flag state is exactly P = (m,x bits, all data flags 0), matching the
    recomp harness's `cpu->P = build_p(m,x); cpu_p_to_mirrors`. Critically this
    avoids `clc; xce` leaving C=1 (XCE swaps carry with the emulation flag), which
    otherwise diverges every carry-dependent op from the recomp's C=0 seed.
    Registers are seeded 16-bit; if x=1, PLP clears X/Y high on hardware exactly as
    cpu_p_to_mirrors does on the recomp side."""
    A  = init.get("A", 0); X = init.get("X", 0); Y = init.get("Y", 0)
    D  = init.get("D", 0); DB = init.get("DB", 0)
    m  = init.get("m", 0); x = init.get("x", 0)
    P  = (0x20 if m else 0) | (0x10 if x else 0)   # data flags 0
    b = bytearray()
    b += bytes([0x18, 0xFB])                 # clc, xce  -> native (e=0); C now =old E
    b += bytes([0xC2, 0x30])                 # rep #$30  -> 16-bit A/X/Y
    b += _imm16(0xA9, D); b += bytes([0x5B]) # lda #D; tcd
    b += bytes([0xE2, 0x20])                 # sep #$20  -> 8-bit A (for plb)
    b += _imm8(0xA9, DB); b += bytes([0x48, 0xAB])  # lda #DB; pha; plb
    b += bytes([0xC2, 0x30])                 # rep #$30  -> 16-bit again
    b += bytes([0xA2, 0xFF, 0x01, 0x9A])     # ldx #$01FF; txs (re-init S each snippet — batch safe)
    b += _imm16(0xA9, A)                     # lda #A
    b += _imm16(0xA2, X)                     # ldx #X
    b += _imm16(0xA0, Y)                     # ldy #Y
    b += _imm16(0xF4, P | (P << 8))          # pea #PP (no reg/flag side effects)
    b += bytes([0x28])                       # plp  -> P = low byte; sets m/x + flags
    return bytes(b)

# --- batched capture: store final A/X/Y/P to a per-snippet result block --------
RESULT_BASE = 0x000200    # bank-00 WRAM; block i at RESULT_BASE + i*8
RESULT_STRIDE = 8

def _capture(i):
    """Store final A/X/Y/P to result block i via DB-independent long stores
    (X/Y routed through A in 16-bit so widths don't matter). Stack-neutral
    (php/pla balanced) so it doesn't disturb the next snippet."""
    base = RESULT_BASE + i * RESULT_STRIDE
    def stl(a): return bytes([0x8F, a & 0xFF, (a >> 8) & 0xFF, (a >> 16) & 0xFF])  # STA long
    b = bytearray()
    b += bytes([0x08])              # php  (save final P)
    b += bytes([0xC2, 0x30])       # rep #$30 (16-bit)
    b += stl(base + 0)             # sta long -> A
    b += bytes([0x8A]); b += stl(base + 2)   # txa; sta -> X (16-bit after rep)
    b += bytes([0x98]); b += stl(base + 4)   # tya; sta -> Y
    b += bytes([0xE2, 0x20])       # sep #$20 (8-bit)
    b += bytes([0x68]); b += stl(base + 6)   # pla (P); sta -> P
    b += bytes([0x8B, 0x68]); b += stl(base + 7)  # phb; pla; sta -> DB
    return bytes(b)

LOROM_RESET = 0x8000   # code entry = $00:8000 = file offset 0

# Scratch WRAM region for memory-read snippets: $00:0010..$00:003F seeded so
# byte[a] == a. Both the oracle ROM and the recomp harness seed it identically,
# so a DP / DP-indexed read lands the same value IFF the address computation
# matches hardware — the register/flag compare then catches address bugs.
SCRATCH_LO, SCRATCH_HI = 0x10, 0x40
# emulation-mode (reset state, 8-bit) seed loop: for x in $10..$40: $00,x = x
_SEED_SCRATCH = bytes([0xA2, SCRATCH_LO,        # ldx #$10
                       0x8A, 0x95, 0x00, 0xE8,  # txa; sta $00,x; inx
                       0xE0, SCRATCH_HI,         # cpx #$40
                       0xD0, 0xF8])              # bne -8

def _finalize_rom(code: bytearray):
    """Place code at $00:8000, append a bra-self trap, write LoROM header+vectors.
    Returns (rom_bytes, trap_pc24)."""
    trap_off = len(code)
    trap_pc = LOROM_RESET + trap_off
    code += bytes([0x80, 0xFE])                       # bra self
    assert len(code) <= 0x7FC0, f"snippet code {len(code)}B overflows bank-0 (max ~32KB)"
    rom = bytearray(b"\x00" * 0x8000)
    rom[0:len(code)] = code
    rom[0x7FC0:0x7FD5] = b"SNIPPET FUZZ ORACLE  "[:21].ljust(21, b" ")
    rom[0x7FD5] = 0x20; rom[0x7FD6] = 0x00; rom[0x7FD7] = 0x05   # LoROM/slow, ROM-only, 32KiB
    for off in (0x7FE4, 0x7FE6, 0x7FE8, 0x7FEA, 0x7FEC, 0x7FEE,
                0x7FF4, 0x7FF8, 0x7FFA, 0x7FFC, 0x7FFE):
        rom[off] = LOROM_RESET & 0xFF; rom[off + 1] = (LOROM_RESET >> 8) & 0xFF
    return bytes(rom), trap_pc & 0xFFFFFF

def build_snippet_rom(snippet: bytes, init: dict):
    """One snippet per ROM; final state read from the DUMP at the trap."""
    code = bytearray(_SEED_SCRATCH); code += _prologue(init); code += snippet
    return _finalize_rom(code)

def build_batch_rom(cases):
    """Many (snippet, init) per ROM: each runs prologue -> snippet -> capture,
    storing A/X/Y/P to a per-snippet result block. One bsnes boot for N snippets.
    Snippets only READ scratch (no writes in the palette), so one seed suffices."""
    code = bytearray(_SEED_SCRATCH)
    for i, (snip, init) in enumerate(cases):
        code += _prologue(init); code += snip; code += _capture(i)
    return _finalize_rom(code)

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


_RANGE_RE = re.compile(r"ram\[\$[0-9A-Fa-f]+:\d+\]=([0-9A-Fa-f]+)")

def run_oracle_batch(rom: bytes, trap_pc: int, n: int, secs: int = 14):
    """Run a batch ROM; return a list of n {A,X,Y,P,m,x} from the result blocks
    (or None on failure). One bsnes boot for all n snippets."""
    subprocess.run(["pkill", "-9", "bsnes"], capture_output=True); time.sleep(0.6)
    with tempfile.NamedTemporaryFile(suffix=".sfc", delete=False, dir="/tmp") as f:
        f.write(rom); rompath = f.name
    env = dict(os.environ)
    env.update({"SF_BSNES_WRAM_INIT": "00",
                "SF_BSNES_DUMP_AT": f"{trap_pc:06x}:1",
                "SF_BSNES_DUMP_MEM": f"{RESULT_BASE & 0xFFFF:04x}:{n * RESULT_STRIDE}",
                "SDL_VIDEODRIVER": "x11", "QT_QPA_PLATFORM": "xcb", "DISPLAY": ":0"})
    subprocess.run(["timeout", "-s", "KILL", str(secs), BSNES, rompath],
                   env=env, cwd=BSNES_DIR, capture_output=True)
    log = pathlib.Path("/tmp") / (pathlib.Path(rompath).stem + "-trace.log")
    txt = log.read_text(errors="replace") if log.exists() else ""
    m = _RANGE_RE.search(txt)
    if not m:
        return None
    raw = bytes.fromhex(m.group(1))
    if len(raw) < n * RESULT_STRIDE:
        return None
    out = []
    for i in range(n):
        b = raw[i * RESULT_STRIDE:(i + 1) * RESULT_STRIDE]
        P = b[6]
        out.append({"A": b[0] | (b[1] << 8), "X": b[2] | (b[3] << 8),
                    "Y": b[4] | (b[5] << 8), "P": P, "DB": b[7],
                    "m": (P >> 5) & 1, "x": (P >> 4) & 1})
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
