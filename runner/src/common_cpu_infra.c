#include "common_cpu_infra.h"
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
  for (int i = top - 2; i >= 0; i--) {
    if (g_cpu_entry_s[i] == ret_s) return (top - 1) - i;
  }
  return -1;
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
static clock_t g_frame_start_clock;
static int g_watchdog_enabled;
static int g_watchdog_counter;
jmp_buf g_watchdog_jmp;
int g_watchdog_tripped;
CoopIrqPumpFunc g_coop_irq_pump;  /* game-registered cooperative IRQ pump */
HostPresentFunc g_host_present_hook; /* host-registered frame present hook */
GsuFrameDoneFunc g_gsu_frame_done; /* game-registered GSU/coproc frame presenter */
DmaSuppressFunc g_dma_suppress;    /* game-registered MDMAEN channel suppressor */
static int g_in_coop_pump;        /* reentrancy guard (pump runs recompiled code) */

void WatchdogFrameStart(void) {
  g_frame_start_clock = clock();
  g_watchdog_enabled = 1;
  g_watchdog_tripped = 0;
  g_watchdog_counter = 0;
  g_recomp_stack_top = 0;
  g_tailcall_context_valid = 0;
}

// Called at loop headers in generated code — detect infinite loops
void WatchdogCheck(void) {
  if (!g_watchdog_enabled) return;
  // Only check clock() every 10000 iterations to avoid overhead
  if (++g_watchdog_counter < 10000) return;
  g_watchdog_counter = 0;
  /* Cooperative IRQ pump: advance interrupt-only hardware so spin-waits on
   * IRQ-set flags fall through (see CoopIrqPumpFunc in the header). Guarded
   * against reentrancy because the pump itself runs recompiled code whose
   * blocks call WatchdogCheck. Runs during boot too (before the
   * frame_counter==0 gate below), since Star Fox's boot init already waits
   * on the IRQ-driven transfer_l double-buffer flags. */
  if (g_coop_irq_pump && !g_in_coop_pump) {
    g_in_coop_pump = 1;
    int progressed = g_coop_irq_pump();
    g_in_coop_pump = 0;
    if (progressed) {
      g_frame_start_clock = clock();  /* we advanced; reset the hang timer */
      return;
    }
  }
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

