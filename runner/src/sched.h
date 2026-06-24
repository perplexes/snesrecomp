#pragma once
/*
 * sched.h — cycle/scanline scheduler for SNES static recompiler.
 *
 * Replaces the ad-hoc cooperative watchdog pump with a time-driven scheduler
 * that advances a cycle counter on every WatchdogCheck tick and delivers
 * NMI (at VBlank, scanline 225) and IRQ (at the VTIMER target) at the
 * architecturally correct scanline, rather than at an arbitrary ~10k-block
 * cadence.
 *
 * Engine-agnostic: no game-specific constants, no direct calls to irqcode_l.
 * IRQ delivery is delegated to g_coop_irq_pump (already the correct handler
 * for the current game layer). NMI delivery increments snes_frame_counter and
 * the hardware inNmi/inVblank flags; NMI handler invocation is left to the
 * game layer via g_sched_nmi_handler (NULL = skip handler call, just update
 * counters).
 *
 * Enable at runtime with SF_SCHED=1. The old pump path runs when SF_SCHED is
 * not set — build always compiles both paths.
 *
 * NTSC timing constants:
 *   262 scanlines × 1364 dots/scanline = 357,368 master cycles/frame
 *   VBlank starts at scanline 225 (scanline 0 = first visible line)
 *   Each scanline = 1364 master cycles
 */

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Cost assigned to each WatchdogCheck tick when running under the scheduler.
 * This is intentionally coarse — the recompiled blocks don't count real
 * cycles, so we use a round number that gives reasonable interrupt cadence.
 * 10 cycles/tick × ~35,700 ticks/frame = ~357,000 cycles/frame ≈ NTSC exact.
 */
#define SCHED_BLOCK_COST  10u

/* NTSC scanline geometry */
#define SCHED_CYCLES_PER_SCANLINE  1364u
#define SCHED_VBLANK_SCANLINE       225u   /* first VBlank scanline (0-based) */
#define SCHED_TOTAL_SCANLINES       262u
#define SCHED_CYCLES_PER_FRAME  \
    ((uint32_t)(SCHED_CYCLES_PER_SCANLINE) * (uint32_t)(SCHED_TOTAL_SCANLINES))

/* 1 when SF_SCHED=1 env var was set at startup; checked in WatchdogCheck. */
extern int g_sched_enabled;

/*
 * Optional NMI handler callback — game layer registers this if it wants the
 * scheduler to call a game-specific NMI body (e.g. run the NMI handler that
 * reads joypad / increments the game frame counter). NULL = scheduler only
 * updates snes_frame_counter + hardware flags, does not call any handler.
 *
 * The callback runs with reentrancy guarded (g_in_coop_pump held); it MUST
 * NOT call sched_tick re-entrantly. The game layer is responsible for any
 * CpuState save/restore the NMI body requires.
 */
typedef void (*SchedNmiFunc)(void);
extern SchedNmiFunc g_sched_nmi_handler;

/* Reset the per-frame cycle/scanline accumulators. Call once at the start of
 * each host frame (before WatchdogFrameStart or alongside it). */
void sched_frame_start(void);

/*
 * Advance the scheduler clock by block_cost master cycles. Checks whether the
 * new cycle position has crossed a VBlank or VTIMER boundary and delivers the
 * corresponding interrupt if conditions are met:
 *
 *   NMI: fires once when cycle position crosses into scanline 225+ (VBlank).
 *        Edge-latched: only one delivery per VBlank entry regardless of how
 *        many ticks land above scanline 225. Reentrancy-guarded.
 *        Calls g_sched_nmi_handler() if non-NULL; increments snes_frame_counter.
 *
 *   IRQ: fires when the tick's scanline range crosses vTimer (prev < vTimer <=
 *        current) AND g_snes->vIrqEnabled AND cpu._flag_I == 0 AND !g_in_coop_pump.
 *        Delegates to g_coop_irq_pump() (same handler the watchdog pump uses).
 *        Edge-latched per vTimer value per frame: only one delivery even if
 *        multiple ticks land in the same scanline neighborhood.
 *
 * Must only be called from WatchdogCheck (i.e., the game's recompiled
 * execution thread). Not thread-safe.
 */
void sched_tick(uint32_t block_cost);

#ifdef __cplusplus
}
#endif
