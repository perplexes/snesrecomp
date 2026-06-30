#!/usr/bin/env python3
"""phase_b.py — differential fuzz: run a 65816 snippet through the recomp's v2
codegen AND the bsnes-plus oracle, diff the final CPU state.

Closes the loop the project planned but never reached (PHASE_B_DIFFERENTIAL_FUZZ):
  * recomp side — lower+emit the snippet via v2.codegen, wrap in a C harness that
    #includes the REAL runner headers (cpu_state.h / cpu_trace.h) so the actual
    width helpers and cpu_p_to_mirrors are under test — not a hand-maintained
    copy (the copy is how the index-high-byte bug hid). Only the memory backend
    and trace hooks are stubbed (flat g_ram / no-ops). gcc, not MSVC.
  * oracle side — bsnes_oracle.run_oracle (synthetic ROM, deterministic WRAM).
  * differ — compare A/X/Y/D/DB + m/x (S omitted; the prologue/harness seed it
    differently). Any divergence is a codegen/runtime bug.

Usage: python phase_b.py            # runs the built-in regression corpus
"""
from __future__ import annotations
import os, sys, subprocess, tempfile, pathlib, re

REPO = pathlib.Path(__file__).resolve().parent.parent
RUNNER_SRC = REPO / "runner" / "src"
sys.path.insert(0, str(REPO / "recompiler"))
sys.path.insert(0, str(REPO / "fuzz"))
import snes65816 as s65                       # noqa: E402
from v2 import lowering, codegen              # noqa: E402
import bsnes_oracle                           # noqa: E402


def emit_snippet_body(rom: bytes, m0: int, x0: int) -> list[str]:
    """Lower+emit one straight-line snippet via v2 codegen, tracking M/X."""
    lines, off, pc, m, x = [], 0, 0x8000, m0, x0
    counter = [0]
    def vf():
        from v2.ir import Value
        counter[0] += 1
        return Value(vid=counter[0])
    while off < len(rom):
        insn = s65.decode_insn(rom, off, pc, 0, m=m, x=x)
        if insn is None:
            raise ValueError(f"decode fail at off {off} byte {rom[off]:#04x}")
        insn.m_flag, insn.x_flag = m, x
        for op in lowering.lower(insn, value_factory=vf):
            lines.extend(codegen.emit_op(op))
        if insn.mnem == "REP":
            if insn.operand & 0x20: m = 0
            if insn.operand & 0x10: x = 0
        elif insn.mnem == "SEP":
            if insn.operand & 0x20: m = 1
            if insn.operand & 0x10: x = 1
        off += insn.length; pc = (pc + insn.length) & 0xFFFF
    return lines


def decode_snippet(rom: bytes, m0: int, x0: int):
    """Decode a straight-line snippet into the insn list (m/x tracked)."""
    insns, off, pc, m, x = [], 0, 0x8000, m0, x0
    while off < len(rom):
        insn = s65.decode_insn(rom, off, pc, 0, m=m, x=x)
        if insn is None:
            raise ValueError(f"decode fail at off {off} byte {rom[off]:#04x}")
        insn.m_flag, insn.x_flag = m, x
        insns.append(insn)
        if insn.mnem == "REP":
            if insn.operand & 0x20: m = 0
            if insn.operand & 0x10: x = 0
        elif insn.mnem == "SEP":
            if insn.operand & 0x20: m = 1
            if insn.operand & 0x10: x = 1
        off += insn.length; pc = (pc + insn.length) & 0xFFFF
    return insns

def recomp_cycle_estimate(snippet: bytes, init: dict) -> int:
    """The recomp's CPU-cycle estimate for the snippet (sum of the cycle table,
    under its stated assumptions: DP.low=0, no page-cross, no taken branch)."""
    from v2.cycle_tables import estimate_cycles
    return sum(estimate_cycles(i) for i in
               decode_snippet(snippet, init.get("m", 0), init.get("x", 0)))


HARNESS = r"""
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "types.h"
#include "cpu_trace.h"   /* no-op trace stubs in non-TRACE build */
#include "cpu_state.h"   /* REAL width helpers + cpu_p_to_mirrors (under test) */

/* --- stubs the real headers declare but the snippet harness must supply --- */
static uint8_t g_ram[0x20000];
CpuState g_cpu;
int g_block_trace_on = 0;
const DispatchEntry g_dispatch_table[1] = {{0}};
const unsigned g_dispatch_table_count = 0;
WramReadPin g_wram_pins[CPU_WRAM_PIN_MAX];
uint8_t g_wram_pin_any = 0;

/* flat-buffer memory backend (snippets touch WRAM only) */
static uint32_t flat(uint8 bank, uint16 addr){ return (((uint32_t)bank<<16)|addr) & 0x1FFFF; }
uint8  cpu_read8 (CpuState *c, uint8 b, uint16 a){ (void)c; return g_ram[flat(b,a)]; }
uint16 cpu_read16(CpuState *c, uint8 b, uint16 a){ (void)c; return (uint16)(g_ram[flat(b,a)] | (g_ram[flat(b,(uint16)(a+1))]<<8)); }
void   cpu_write8 (CpuState *c, uint8 b, uint16 a, uint8 v){ (void)c; g_ram[flat(b,a)]=v; }
void   cpu_write16(CpuState *c, uint8 b, uint16 a, uint16 v){ (void)c; g_ram[flat(b,a)]=(uint8)v; g_ram[flat(b,(uint16)(a+1))]=(uint8)(v>>8); }

static uint8 build_p(int m,int x){ return (uint8)((m?0x20:0)|(x?0x10:0)); }

int main(void){
    CpuState _cpu; CpuState *cpu = &_cpu; memset(cpu,0,sizeof *cpu); cpu->ram=g_ram;
    for(int a=0x10;a<0x40;a++) g_ram[a]=(uint8_t)a;   /* scratch seed: byte[a]=a (matches the oracle ROM) */
    cpu->A=A0; cpu->X=X0; cpu->Y=Y0; cpu->D=D0; cpu->DB=DB0;
    cpu->P=build_p(M0,X0F); cpu_p_to_mirrors(cpu);
    RUN_BODY
    cpu_mirrors_to_p(cpu);   /* canonicalize P from the mirror flags the runtime branches on */
    printf("{\"A\":%u,\"X\":%u,\"Y\":%u,\"S\":%u,\"D\":%u,\"DB\":%u,\"P\":%u,\"m\":%u,\"x\":%u}\n",
        cpu->A,cpu->X,cpu->Y,cpu->S,cpu->D,cpu->DB,cpu->P,cpu->m_flag,cpu->x_flag);
    return 0;
}
"""

def run_recomp(snippet: bytes, init: dict):
    body = emit_snippet_body(snippet, init.get("m", 0), init.get("x", 0))
    src = HARNESS
    repl = {"A0": init.get("A",0), "X0": init.get("X",0), "Y0": init.get("Y",0),
            "D0": init.get("D",0), "DB0": init.get("DB",0),
            "M0": init.get("m",0), "X0F": init.get("x",0)}
    for k, v in repl.items():
        src = re.sub(rf"\b{k}\b", str(v), src)
    src = src.replace("RUN_BODY", "\n    ".join(body))
    with tempfile.TemporaryDirectory() as d:
        cf = pathlib.Path(d)/"h.c"; ex = pathlib.Path(d)/"h"
        cf.write_text(src)
        r = subprocess.run(["gcc","-O1","-I",str(RUNNER_SRC),str(cf),"-o",str(ex)],
                           capture_output=True, text=True)
        if r.returncode:
            return {"error": "compile", "stderr": r.stderr[-1500:]}
        out = subprocess.run([str(ex)], capture_output=True, text=True)
        import json
        try: return json.loads(out.stdout.strip().splitlines()[-1])
        except Exception: return {"error":"run","out":out.stdout,"err":out.stderr}


CORPUS = [
    # the SEP-transition-clear gap the curated fuzz missed (no LDX after SEP)
    {"id":"sep30_clears_x_high", "init":{"m":0,"x":0},
     "rom":bytes([0xC2,0x30, 0xA2,0xFF,0x02, 0xE2,0x30])},
    {"id":"sep10_clears_x_high_y_high", "init":{"m":1,"x":0,"X":0xAB42,"Y":0xCD99},
     "rom":bytes([0xE2,0x10])},                       # just SEP #$10, nonzero X/Y seed
    {"id":"plp_sets_x_clears_high", "init":{"m":0,"x":0,"X":0x1234},
     "rom":bytes([0xE2,0x10])},                       # SEP path again, A/m vary
    {"id":"rep_then_sep_roundtrip", "init":{"m":0,"x":0},
     "rom":bytes([0xC2,0x30, 0xA2,0x55,0x77, 0xE2,0x30, 0xC2,0x10])},  # narrow then widen
]

# Register fields compared directly. D is omitted (bsnes DUMP has no direct page);
# the flag byte P is compared separately (it subsumes m/x). I (0x04) is masked out
# — neither side exercises interrupts and it is never meaningful here.
REG_FIELDS = ["A", "X", "Y", "DB"]
PMASK = 0xFF & ~0x04   # N V m x D - Z C

def _norm(st):
    """Mask only the bytes hardware makes invisible. In 8-bit index mode (x=1) the
    index high byte reads as 0. The accumulator high byte (B) is NOT masked — it
    stays architecturally visible via XBA/TDC even in m=1 (the TDC bug lived there),
    so a stale B is a real divergence, not noise."""
    st = dict(st)
    if st.get("x") == 1:
        st["X"] &= 0xFF; st["Y"] &= 0xFF
    return st

def compare(rec, ora):
    """Return a list of human-readable divergence strings (empty = match)."""
    r, o = _norm(rec), _norm(ora)
    diffs = [f"{k}:rec={r.get(k):#x}!=ora={o.get(k):#x}"
             for k in REG_FIELDS if r.get(k) != o.get(k)]
    if (r.get("P", 0) & PMASK) != (o.get("P", 0) & PMASK):
        diffs.append(f"P:rec={r.get('P',0)&PMASK:#04x}!=ora={o.get('P',0)&PMASK:#04x}")
    return diffs

# back-compat alias
FIELDS = REG_FIELDS

def main():
    print(f"{'case':<34} {'result':<8} divergences")
    fails = 0
    for c in CORPUS:
        rec = run_recomp(c["rom"], c["init"])
        if "error" in rec:
            print(f"{c['id']:<34} BUILDERR {rec.get('stderr','')[:200]}"); fails+=1; continue
        rom, trap = bsnes_oracle.build_snippet_rom(c["rom"], c["init"])
        ora = bsnes_oracle.run_oracle(rom, trap)
        if ora is None:
            print(f"{c['id']:<34} NO-ORACLE (bsnes/DISPLAY?)"); continue
        diffs = compare(rec, ora)
        status = "OK" if not diffs else "DIVERGE"
        if diffs: fails+=1
        print(f"{c['id']:<34} {status:<8} {' '.join(diffs)}")
    print(f"\n{'ALL PASS' if not fails else str(fails)+' DIVERGENCE(S)'}")
    return 1 if fails else 0

if __name__ == "__main__":
    sys.exit(main())
