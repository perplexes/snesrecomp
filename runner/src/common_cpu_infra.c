#include "common_cpu_infra.h"
#include "sched.h"
#include "framedump.h"
#include "types.h"
#include "common_rtl.h"
#include "recomp_hw.h"
#include "snes/cpu.h"
#include "snes/snes.h"
#include "snes/msu1.h"
#include "util.h"
#include "cpu_trace.h"
#include "debug_server.h"
#include <setjmp.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>

Snes *g_snes;
Cpu *g_snes_cpu;

bool g_fail;
const RtlGameInfo *g_rtl_game_info;

void RtlRegisterGame(const RtlGameInfo *info) {
  g_rtl_game_info = info;
  /* Arm MSU-1 from the environment for every game, with no per-game
   * wiring. Inert (default-OFF) unless SNESRECOMP_MSU1 is set. A game's
   * main.c may additionally call msu1_set_rom_path() to enable the
   * "auto" base-from-ROM-name mode. */
  msu1_init();
}

uint8_t *SnesRomPtr(uint32 v) {
  return (uint8 *)RomPtr(v);
}

// Apply the native-mode CPU state the real ROM's reset vector would
// have established. See header comment.
void SnesEnterNativeMode(void) {
  g_snes_cpu->e = false;
  g_snes_cpu->sp = 0x01FF;
  g_snes_cpu->dp = 0;
  g_snes_cpu->mf = false;
  g_snes_cpu->xf = false;
  g_snes_cpu->d = false;
  g_snes_cpu->i = true;
}

// Resolve a 16-bit-indirect-through-DP pointer using the current
// data bank register. See comment in common_rtl.h for why this
// matters for `(dp)`, `(dp),Y`, `(dp,X)` addressing modes.
uint8_t *IndirPtrDB(uint8 dp_addr, uint16 offs) {
  LongPtr p = MAKE_LONG((uint16)g_ram[dp_addr] | ((uint16)g_ram[dp_addr + 1] << 8),
                        g_snes_cpu->db);
  return IndirPtr(p, offs);
}

// Debug: recomp function call stack for watchdog diagnostics.
const char *g_last_recomp_func = "(none)";
// Tier 1.5 call-trace depth cap. Originally 16; bumped to 64 because
// SMW peak call depth is ~10 but Tier 1.5 attribution silently
// degrades past the cap (g_last_recomp_func and parent fields go
// stale). 64 gives 6x headroom for any conceivable call chain at
// negligible memory cost (8 bytes/slot * 48 extra slots = 384 bytes).
#define RECOMP_STACK_DEPTH 64
const char *g_recomp_stack[RECOMP_STACK_DEPTH];
int g_recomp_stack_top = 0;

/* Per-frame 65816 stack-entry level (cpu->S at function entry), parallel
 * to g_recomp_stack and indexed by the same g_recomp_stack_top. The
 * function prologue records _entry_s here; pops are implicit (top--).
 * Used by cpu_resolve_ancestor_skip() to turn a return-to-ancestor RTS
 * (manual PLA/PLX/PLB rebalance to an ancestor's entry level, then RTS)
 * into a SKIP_N non-local return through the existing call-site
 * decrement contract. See ISSUES.md "shared-tail multi-level non-local
 * return" (the fish-explosion OAM wipe). */
uint16_t g_cpu_entry_s[RECOMP_STACK_DEPTH];
static uint8_t g_tailcall_context_valid;
static uint16_t g_tailcall_entry_s;
static uint8_t g_tailcall_hrv;

void cpu_tailcall_inherit_return_context(uint16_t entry_s, uint8_t hrv) {
  g_tailcall_entry_s = entry_s;
  g_tailcall_hrv = hrv;
  g_tailcall_context_valid = 1;
}

int cpu_take_tailcall_return_context(uint16_t *entry_s, uint8_t *hrv) {
  if (!g_tailcall_context_valid) return 0;
  if (entry_s) *entry_s = g_tailcall_entry_s;
  if (hrv) *hrv = g_tailcall_hrv;
  g_tailcall_context_valid = 0;
  return 1;
}

int cpu_resolve_ancestor_skip(uint16_t ret_s) {
  /* The current (top-1) frame is the one whose RTS we are resolving; it
   * is NOT a match (its entry_s != ret_s, else the balanced host-return
   * path handled it). Scan STRICT ancestors for the nearest frame whose
   * entry_s == ret_s — that frame should host-return NORMAL to its
   * caller (which resumes at its natural continuation). Return the SKIP
   * count = how many RECOMP_RETURN levels to unwind to reach it; -1 if
   * none (caller falls back to the normal dispatch-miss path, no change
   * in behavior). */
  int top = g_recomp_stack_top;
  if (top < 2 || top > RECOMP_STACK_DEPTH) return -1;
  int _found = -1;
  for (int i = top - 2; i >= 0; i--) {
    if (g_cpu_entry_s[i] == ret_s) { _found = (top - 1) - i; break; }
  }
  /* skip==1 guard (2026-06-25, codex gpt-5.5 + opus advisor convergence): a
   * resolution to the IMMEDIATE C-parent (top-2, _found==1) is ALWAYS wrong.
   * A direct parent resumes correctly via the normal dispatch-miss fallback
   * (cpu_dispatch_pc_from restores S=entry_s+frame_sz and returns NORMAL, so
   * the parent's call-site continues at its natural continuation) — it never
   * needs to be *abandoned* via SKIP. Genuine multi-level non-local returns
   * (the fish-explosion OAM-wipe the mechanism exists for) are always skip>=2.
   * Without this, a guest-S drift injected upstream (e.g. a strat coroutine
   * ancestor-skip leaving S off by one return frame) makes a plain jsl/rtl
   * mis-resolve as SKIP_1, and the caller's `return (_r-1)` ABANDONS its own
   * frame instead of resuming — which is why the boot front-end runs intro_l
   * then returns out of I_RESET, skipping titleseq/briefing/planetseq.
   * Forcing -1 here routes skip==1 to the (correct) dispatch fallback. */
  if (_found == 1) _found = -1;
  /* SF_TRACE_BOOT: dump the recomp call stack when an ancestor-skip is resolved
   * during boot — to see if the frame-boundary unwind overshoots gameloop2 to
   * the I_RESET root. Ungated; one-line per resolution, capped. */
  {
    static int s_tb = -1, s_n = 0;
    if (s_tb < 0) s_tb = getenv("SF_TRACE_BOOT") ? 1 : 0;
    if (s_tb && s_n < 30) {
      s_n++;
      fprintf(stderr, "[anc] ret_s=%04x top=%d skip=%d stack=[", ret_s, top, _found);
      for (int i = 0; i < top && i < RECOMP_STACK_DEPTH; i++)
        fprintf(stderr, "%s%s(s=%04x)", i ? "," : "",
                g_recomp_stack[i] ? g_recomp_stack[i] : "?", g_cpu_entry_s[i]);
      fprintf(stderr, "]\n");
    }
  }
  /* SF_SKIP_TRACE: name the function whose RTL resolves a (>=2) ancestor-skip,
   * the skip count, ret_s, and the target ancestor frame — so the FIRST emitter
   * of a SKIP_N is identified AT ITS SOURCE, not where it leaks (e.g. past
   * I_RESET). A resolution to/below the stack root is flagged: that is the
   * signature of a corrupt ret_s (a garbage continuation pointer popped by an
   * RTL) coincidentally matching a deep ancestor's entry_s. */
  {
    static int s_st = -1, s_n = 0;
    if (s_st < 0) s_st = getenv("SF_SKIP_TRACE") ? 1 : 0;
    if (s_st && _found >= 2 && s_n < 200) {
      s_n++;
      int ai = (top - 1) - _found;  /* target ancestor stack index */
      const char *cur = (top >= 1 && top <= RECOMP_STACK_DEPTH)
                          ? g_recomp_stack[top - 1] : "?";
      const char *anc = (ai >= 0 && ai < RECOMP_STACK_DEPTH)
                          ? g_recomp_stack[ai] : "(below root)";
      fprintf(stderr,
              "[skip] SKIP_%d from %s (ret_s=%04x) -> ancestor[%d] %s%s\n",
              _found, cur ? cur : "?", ret_s, ai, anc ? anc : "?",
              (ai <= 0)
                ? "  <-- resolves to/below root (corrupt ret_s? garbage RTL pop)"
                : "");
    }
  }
  return _found;
}

// Function-boundary WRAM snapshot history (Phase B koopa-stomp).
// When a TCP client sets g_recomp_snap_on_func to a non-NULL name,
// every RecompStackPush whose name matches captures the LOW 8KB of
// WRAM ($0000-$1FFF — DP + game-state region used by SMW for all
// sprite/level/player state) into a ring buffer of 256 slots.
//
// Ring keeps the most recent 256 calls; older entries get overwritten.
// Each slot has: absolute call index (the count at capture time),
// frame number at capture, and the 8KB WRAM slice. Total: 256 × 8KB
// = 2 MB per side. Fits comfortably; 256 calls ≈ 4 seconds at 60Hz
// and ≈ 256 frames in SMW (one HandlePlayerPhysics call per frame).
//
// Probes use func_snap_get_n <call_idx> to fetch a specific historic
// snapshot and bisect for the first diverging call.
#define RECOMP_SNAP_SLICE_LEN  0x2000  /* $0000-$1FFF */
#define RECOMP_SNAP_RING_LEN   256

const char *g_recomp_snap_on_func = NULL;
int        g_recomp_snap_count    = 0;     /* total calls observed */
int        g_recomp_snap_frame    = -1;    /* most recent capture's frame */
typedef struct {
    int     call_idx;                       /* g_recomp_snap_count value at capture */
    int     frame;
    uint8_t wram_slice[RECOMP_SNAP_SLICE_LEN];
} recomp_snap_entry;
recomp_snap_entry g_recomp_snap_ring[RECOMP_SNAP_RING_LEN];

/* Lookup an entry by absolute call index. Returns NULL if the index
 * is out of the ring's current window. */
const recomp_snap_entry* recomp_snap_lookup(int call_idx) {
    if (call_idx < 1) return NULL;
    int slot = (call_idx - 1) % RECOMP_SNAP_RING_LEN;
    if (g_recomp_snap_ring[slot].call_idx != call_idx) return NULL;
    return &g_recomp_snap_ring[slot];
}

void RecompStackPush(const char *name) {
  if (g_recomp_stack_top < RECOMP_STACK_DEPTH)
    g_recomp_stack[g_recomp_stack_top++] = name;
  g_last_recomp_func = name;
  debug_server_profile_push(name);
  /* SF_TRACE_BOOT: depth-limited top-level call trace — see the front-end boot
   * sequence (intro/title/briefing/planetseq/gameloop) without strat noise. */
  {
    static int s_tbp = -1, s_tbp_n = 0;
    if (s_tbp < 0) s_tbp = getenv("SF_TRACE_BOOT") ? 1 : 0;
    if (s_tbp && s_tbp_n < 400 && g_recomp_stack_top <= 5 && name) {
      s_tbp_n++;
      fprintf(stderr, "[call d%d] %s\n", g_recomp_stack_top, name);
    }
  }
  /* SF_FUNC_WATCH=<substr[,substr...]>: log every function entry whose
   * name contains any watched substring. Decisive "does routine X run" probe
   * for the no-vision agent (works for direct C calls AND dispatches, since
   * RecompStackPush fires at every entry). */
  {
    static int s_fw = -1; static char s_pats[8][40]; static int s_npats;
    if (s_fw < 0) { const char *e = getenv("SF_FUNC_WATCH"); s_fw = e ? 1 : 0;
      if (e) { char b[256]; strncpy(b,e,sizeof(b)-1); b[sizeof(b)-1]=0;
        char *t=strtok(b,",;"); while(t && s_npats<8) { strncpy(s_pats[s_npats],t,39); s_pats[s_npats][39]=0; s_npats++; t=strtok(0,",;"); } } }
    if (s_fw && name) {
      for (int i=0;i<s_npats;i++) if (strstr(name, s_pats[i])) {
        static int n=0; if (n++ < 400) fprintf(stderr, "[fwatch] %s (depth=%d S=%04x)\n", name, g_recomp_stack_top, 0);
        break;
      }
    }
  }
  /* SF_TRACE_STRAT: log entries to the player-setup / strat-dispatch path
   * (always-on; this push fires for every recompiled func). Matches both
   * symbol-named and address-named (bank_BB_AAAA) forms. */
  {
    static int s_st = -1;
    if (s_st < 0) { const char *e = getenv("SF_TRACE_STRAT"); s_st = e ? atoi(e) : 0; }
    if (s_st >= 2 && name) {
      /* once initgame_strats_l is entered, log the next 60 entries verbatim so
       * we see exactly what the player object's strat dispatch resolves to */
      static int armed = 0, na = 0;
      if (!armed && (strstr(name, "bank_21_8000") || strstr(name, "INITGAME_STRATS"))) armed = 1;
      if (armed && na++ < 60) fprintf(stderr, "[all] %s\n", name);
    }
    if (s_st && name) {
      static const char *const pats[] = {
        "INITGAME_L", "bank_02_DEB0",            /* initgame_l */
        "INITGAME_STRATS", "bank_21_8000",       /* initgame_strats_l */
        "INIT_STRATS_L", "bank_21_81CC",         /* init_strats_l */
        "DO_STRAT_L", "bank_1F_D283",            /* do_strat_l */
        "PLAYER_ISTRAT", "bank_0B_B53C",         /* player_Istrat */
        "bank_0B_D1FF", "PLAYERMOVE_INIT",       /* playermove_init */
        "NEWOBJS", "bank_03_E183", "NEWOBJEX", "bank_03_E18D",
        "KILL_LIST", "bank_02_F018", "INITMEM", "bank_0A_F922",
        "ISTRAT", "bank_0B_B53C", NULL };
      for (int i = 0; pats[i]; i++) {
        if (strstr(name, pats[i])) {
          static int n = 0;
          if (n++ < 500) fprintf(stderr, "[strat] %s\n", name);
          break;
        }
      }
    }
  }
  /* SF_PROBE_NEWOBJS: log mapptr/mapcnt every time the map interpreter is
   * entered (NEWOBJS_L $03E183 / NEWOBJEX $03E18D / mapwait $03E24A). Decisive
   * for whether the intro map script ADVANCES per frame (mapptr changes) or is
   * STUCK re-walking the same command. Read-only WRAM probe. */
  {
    static int s_np = -1, s_nn = 0;
    if (s_np < 0) s_np = getenv("SF_PROBE_NEWOBJS") ? 1 : 0;
    if (s_np && name && s_nn < 4000 &&
        (strstr(name, "E183") || strstr(name, "E18D") || strstr(name, "E24A") ||
         strstr(name, "NEWOBJS") || strstr(name, "NEWOBJEX") || strstr(name, "MAPWAIT"))) {
      s_nn++;
      unsigned mapptr = g_ram[0x1782] | (g_ram[0x1783] << 8);
      unsigned mapcnt = g_ram[0x1780] | (g_ram[0x1781] << 8);
      unsigned mapbank = g_ram[0x1af7];
      fprintf(stderr, "[newobjs d%d] %s mapbank=%02x mapptr=%04x mapcnt=%04x\n",
              g_recomp_stack_top, name, mapbank, mapptr, mapcnt);
    }
  }
  // Boundary auditor (always-on; no-op when SNESRECOMP_TRACE=0).
  // Recorded AFTER the stack push so stack_depth reflects post-push state.
  boundary_audit_record_entry(name);
  // Function-boundary snapshot: if a client set a target function
  // name, and this push matches it, capture WRAM. Frame execution
  // continues afterward — no longjmp. Compare the snapshot at
  // matching points across recomp + oracle for sub-frame-precise
  // state diff regardless of NMI ordering.
  if (g_recomp_snap_on_func) {
    extern int snes_frame_counter;
    int match;
    if (name == g_recomp_snap_on_func) match = 1;
    else if (strcmp(g_recomp_snap_on_func, name) == 0) {
      g_recomp_snap_on_func = name;  /* cache pointer for fast path */
      match = 1;
    } else {
      match = 0;
    }
    if (match) {
      g_recomp_snap_count++;
      g_recomp_snap_frame = snes_frame_counter;
      int slot = (g_recomp_snap_count - 1) % RECOMP_SNAP_RING_LEN;
      g_recomp_snap_ring[slot].call_idx = g_recomp_snap_count;
      g_recomp_snap_ring[slot].frame    = snes_frame_counter;
      memcpy(g_recomp_snap_ring[slot].wram_slice, g_ram, RECOMP_SNAP_SLICE_LEN);
    }
  }
}

void RecompStackDump(void) {
  fprintf(stderr, "Recomp call stack (%d deep):\n", g_recomp_stack_top);
  for (int i = g_recomp_stack_top - 1; i >= 0 && i >= g_recomp_stack_top - RECOMP_STACK_DEPTH; i--)
    fprintf(stderr, "  [%d] %s\n", g_recomp_stack_top - 1 - i, g_recomp_stack[i]);
}

void RecompStackPop(void) {
  // Record exit BEFORE the pop so stack_depth reflects pre-pop state and
  // the function name is still the topmost entry. Defensive against
  // empty stack: the auditor must NOT consume an entry_seq it didn't push.
  if (g_recomp_stack_top > 0) {
    boundary_audit_record_exit(g_recomp_stack[g_recomp_stack_top - 1]);
    g_recomp_stack_top--;
  }
  g_last_recomp_func = g_recomp_stack_top > 0 ? g_recomp_stack[g_recomp_stack_top - 1] : "(none)";
}

// Frame watchdog: detect infinite loops in generated code.
// Set before calling run_frame, checked by generated code periodically.
clock_t g_frame_start_clock;
static int g_watchdog_enabled;
static int g_watchdog_counter;
jmp_buf g_watchdog_jmp;
int g_watchdog_tripped;
CoopIrqPumpFunc g_coop_irq_pump;  /* game-registered cooperative IRQ pump */
HostPresentFunc g_host_present_hook; /* host-registered frame present hook */
GsuFrameDoneFunc g_gsu_frame_done; /* game-registered GSU/coproc frame presenter */
DmaSuppressFunc g_dma_suppress;    /* game-registered MDMAEN channel suppressor */
int g_in_coop_pump;               /* reentrancy guard (pump runs recompiled code) */

/* Cycle-faithful clock accumulator. The recompiler emits cpu_cycle_tick(cpu, N)
 * per block; we sum here and WatchdogCheck() flushes the real total into
 * sched_tick() (replacing the old fixed-cost heuristic). */
uint64_t g_pending_cycles = 0;
void cpu_cycle_tick(struct CpuState *cpu, uint32_t n) {
  (void)cpu;
  g_pending_cycles += n;
}

void WatchdogFrameStart(void) {
  g_frame_start_clock = clock();
  g_watchdog_enabled = 1;
  g_watchdog_tripped = 0;
  g_watchdog_counter = 0;
  g_recomp_stack_top = 0;
  g_tailcall_context_valid = 0;
  /* Initialise scheduler on first call. Reads SF_SCHED env var once. */
  static int s_sched_init = 0;
  if (!s_sched_init) {
    s_sched_init = 1;
    /* Env overrides the game-set default (a game enables the scheduler by
     * setting g_sched_enabled=1 before reset; SF does, SMW leaves it 0 for
     * its flipbook path). SF_NO_SCHED forces off; SF_SCHED=0/1 forces. */
    if (getenv("SF_NO_SCHED"))
      g_sched_enabled = 0;
    else if (getenv("SF_SCHED"))
      g_sched_enabled = (getenv("SF_SCHED")[0] == '1');
    /* Allow runtime override of block cost for calibration.
     * SF_SCHED_BLOCK_COST=N sets N cycles per WatchdogCheck tick. */
    if (g_sched_enabled) {
      const char *bc_env = getenv("SF_SCHED_BLOCK_COST");
      if (bc_env && bc_env[0]) {
        long bc = atol(bc_env);
        if (bc > 0 && bc <= 1000000)
          g_sched_block_cost = (uint32_t)bc;
      }
    }
  }
  if (g_sched_enabled) {
    sched_frame_start();
  }
}

// Called at loop headers in generated code — detect infinite loops
void WatchdogCheck(void) {
  /* RECOMP_STACK_WATCH=lo[:hi]: generic stack-pointer drift detector.
   * Stack imbalance is a recurring static-recompiler failure mode: a function
   * that returns via the non-local-return path can leave pushed return bytes
   * on the hardware stack, so cpu->S leaks downward and eventually overruns
   * data RAM. This reports each NEW minimum S within the watch band [lo,hi),
   * naming the recompiled function — pinpointing where the leak happens.
   * Default band [0x0300,0x0400) catches a stack that has run below page-1
   * usage into low data RAM. Off unless the env var is set. */
  {
    static int s_init = 0;
    static uint16 s_lo = 0, s_hi = 0;
    if (!s_init) {
      s_init = 1;
      const char *e = getenv("RECOMP_STACK_WATCH");
      if (e && e[0]) {
        unsigned lo = 0x0300, hi = 0x0400;
        sscanf(e, "%x:%x", &lo, &hi);
        s_lo = (uint16)lo; s_hi = (uint16)hi;
      }
    }
    if (s_hi) {
      extern CpuState g_cpu;
      static uint16 s_minS = 0xffff;
      static int s_hits = 0;
      uint16 nowS = g_cpu.S;
      if (s_minS == 0xffff) s_minS = s_hi;
      if (nowS >= s_lo && nowS < s_minS && s_hits < 16) {
        s_hits++;
        s_minS = nowS;
        extern const char *g_last_recomp_func;
        extern const char *g_recomp_stack[]; extern int g_recomp_stack_top;
        fprintf(stderr, "[stack-watch] new-min S=%04x in %s callstack:\n",
                nowS, g_last_recomp_func);
        for (int i = g_recomp_stack_top - 1; i >= 0 && i > g_recomp_stack_top - 12; i--)
          fprintf(stderr, "    [%d] %s\n", g_recomp_stack_top-1-i, g_recomp_stack[i]);
      }
    }
  }
  if (!g_watchdog_enabled) return;
  // Only check clock() every 10000 iterations to avoid overhead
  if (++g_watchdog_counter < 10000) return;
  g_watchdog_counter = 0;
  /* Scheduler path (SF_SCHED=1): advance the cycle/scanline clock and deliver
   * NMI/IRQ at the architecturally correct scanline instead of at every ~10k
   * block tick. sched_tick() calls g_coop_irq_pump internally when the IRQ
   * scanline is reached.
   *
   * The scheduler activates as soon as V-IRQ is enabled (g_snes->vIrqEnabled).
   * This is the correct transition point for Star Fox: it writes $4200=$20
   * (V-IRQ enable, NMI off) when it is ready for normal frame-driven operation,
   * so the scheduler takes over exactly then. During boot (APU upload, setup)
   * vIrqEnabled is false and the old cooperative pump handles spin-waits.
   *
   * This replaces the previous gate (snes_frame_counter > 0) which was
   * NMI-based and never advanced for games that use only V-IRQ (like Star Fox).
   *
   * The old pump path runs during boot and when SF_SCHED is off, so the
   * pump does not also fire unconditionally every 10k blocks once the scheduler
   * is active.
   *
   * IMPORTANT: skip sched_tick when we are inside the pump (g_in_coop_pump).
   * irqcode_l calls WatchdogCheck from its block prologues, which would advance
   * g_sched_prev_scanline and potentially mark VBlank as "already past" before
   * the outer sched_tick can detect the crossing. The IRQ handler runs at
   * simulated cycle-time 207 and should not advance the clock further; all
   * cycle advancement happens in the outer (non-pump) WatchdogCheck calls. */
  if (g_sched_enabled && g_snes && !g_in_coop_pump) {
    /* FREE-RUNNING CLOCK (Phase 2 switchover): advance the cycle/scanline clock
     * on EVERY block tick, regardless of vIrqEnabled. sched_tick() gates IRQ
     * *delivery* internally on vIrqEnabled, so no V-IRQ fires until the game
     * enables it -- but the clock, the VBlank edge (snes_frame_counter +
     * g_sched_frame_hook present), and the hang watchdog all run from cold boot.
     * This fixes the early-boot deadlock: previously the scheduler refused to
     * tick until vIrqEnabled, so a spin before V-IRQ enable advanced nothing. */
    int pre_frame = snes_frame_counter;
    /* Cycle-faithful clock: advance by the REAL cycles the recompiled blocks
     * reported since the last tick (cpu_cycle_tick accumulation), not a fixed
     * heuristic cost. This makes the simulated beam track actual work, so V-IRQ
     * lands when the game expects it and spin-waits resolve every frame. Fall
     * back to SCHED_BLOCK_COST if no blocks reported (e.g. pure HLE stretch) so
     * the clock never stalls. */
    uint32_t cyc = (uint32_t)g_pending_cycles;
    g_pending_cycles = 0;
    /* Zero-cycle fallback floor (CONSOLIDATION DECISION: KEEP — intended, not
     * scaffolding; see SCHED_BLOCK_COST definition in sched.h). */
    if (cyc == 0) cyc = SCHED_BLOCK_COST;
    sched_tick(cyc);
    if (snes_frame_counter != pre_frame) {
      g_frame_start_clock = clock();
    }
    /* No early-boot cooperative pump here (Tier-1 retirement, NO-CARVES policy).
     * Before the game enables V-IRQ, hardware takes no interrupt (guest P.I set,
     * no timer armed), so the scheduler correctly delivers nothing and the
     * free-running beam clock above covers the early raster spins. The removed
     * pump injected spurious irqcode_l runs hardware would never execute. The
     * scheduled V-IRQ delivery (sched.c, post-vIrqEnabled) is unaffected. */
    /* Fall through to the hang check -- the scheduler path still needs the
     * 5-second watchdog to fire on genuine hangs. */
    goto watchdog_hang_check;
  }
  /* (Tier-1 retirement) The legacy non-scheduler cooperative IRQ pump path that
   * ran here when g_sched_enabled==0 has been removed. Star Fox always enables
   * the scheduler (sf_rtl.c) and branches past this point, so the path was dead
   * for the shipping line; g_coop_irq_pump is NULL for every other game. */
  /* ---- Phase-D status (consolidation closeout) ----
   * - Per-frame trampoline (SfCallMasterLoop / SF_TRAMPOLINE, game-side
   *   src/sf_rtl.c): no longer the active drive path. The real boot self-loops
   *   inside I_RESET on the scheduler; the trampoline is demoted to an opt-in
   *   debug aid (SF_TRAMPOLINE=1), not gone. NOT removable yet.
   * - Stale "carve entry point" cfg comments: NONE remain in recomp/ or runner/.
   * - SCHED_BLOCK_COST zero-cycle floor: KEEP-decided (see sched.h).
   * - ONLY remaining Phase-D-ish item: SfCoopIrqPump Tier-2 retirement (#75),
   *   DEFERRED (high risk). It needs the engine level-trigger path (b19e5dc),
   *   must stay display-neutral, and touches the live IRQ path that drives the
   *   working self-drive — so it is not part of this closeout. */
watchdog_hang_check:;
  double elapsed = (double)(clock() - g_frame_start_clock) / CLOCKS_PER_SEC;
  /* Boot has no watchdog. I_RESET runs once and uploads the SPC
   * engine + samples through the IPL handshake, which is real-time
   * paced by the audio thread (the SPC bootROM can only echo bytes
   * at ~1 MHz). For SMW the upload is ~12 KB and naturally takes
   * tens of seconds wall time; that's expected, not a hang. After
   * I_RESET returns the runtime falls into the normal per-frame
   * cadence (I_NMI + Internal) which completes comfortably under 5 s.
   *
   * Detecting "still in boot" via snes_frame_counter == 0 is robust:
   * the recompiled NMI handler increments snes_frame_counter, and
   * the very first NMI only fires after I_RESET returns and frame 1
   * starts. */
  if (snes_frame_counter == 0) return;
  if (elapsed > 5.0) {
    fprintf(stderr,
      "\n=== WATCHDOG: Frame %d exceeded %.1fs ===\n"
      "Game mode: %d | WatchdogCheck calls: %d\n"
      "Call stack (most recent first):\n",
      snes_frame_counter, elapsed, g_ram[0x100], g_watchdog_counter * 10000);
    for (int i = g_recomp_stack_top - 1; i >= 0; i--)
      fprintf(stderr, "  [%d] %s\n", g_recomp_stack_top - 1 - i, g_recomp_stack[i]);
    if (g_recomp_stack_top == 0)
      fprintf(stderr, "  (empty — last was %s)\n", g_last_recomp_func);
    fprintf(stderr, "\n");
    fflush(stderr);
    g_watchdog_enabled = 0;
    g_watchdog_tripped = 1;
    { extern int snes_frame_counter;
      debug_server_profile_latch(snes_frame_counter); }
    longjmp(g_watchdog_jmp, 1);
  }
}

Snes *SnesInit(const uint8 *data, int data_size) {
  g_snes = snes_init(g_ram);
  g_snes_cpu = g_snes->cpu;
  g_dma = g_snes->dma;
  g_ppu = g_snes->ppu;

  if (data_size != 0) {
    bool loaded = snes_loadRom(g_snes, data, data_size);
    if (!loaded) {
      return NULL;
    }
    g_rom = g_snes->cart->rom;

    assert(g_rtl_game_info && "RtlRegisterGame must be called before SnesInit");

    if (g_rtl_game_info->initialize)
      g_rtl_game_info->initialize();
    snes_reset(g_snes, true); // reset after loading
    SnesEnterNativeMode();
  } else {
    g_snes->cart->ramSize = 2048;
    g_snes->cart->ram = calloc(1, 2048);
    assert(g_rtl_game_info && "RtlRegisterGame must be called before SnesInit");
    if (g_rtl_game_info->initialize)
      g_rtl_game_info->initialize();
    ppu_reset(g_snes->ppu);
    dma_reset(g_snes->dma);
  }

  g_sram = g_snes->cart->ram;
  g_sram_size = g_snes->cart->ramSize;
  return g_snes;
}

