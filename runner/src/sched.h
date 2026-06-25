#pragma once
/*
 * sched.h -- cycle/scanline scheduler for SNES static recompiler.
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
 * snes_frame_counter is incremented on EVERY VBlank crossing regardless of
 * nmiEnabled. This ensures it advances for games that use V-IRQ only (like
 * Star Fox, which never enables NMI) so the watchdog hang-check is active.
 *
 * Hardware model: V-IRQ fires at most once per frame when the beam crosses
 * the target scanline. The game-registered pump (g_coop_irq_pump) is
 * responsible for running the correct number of IRQ phases per delivery.
 *
 * Enable at runtime with SF_SCHED=1. The old pump path runs when SF_SCHED is
 * not set -- build always compiles both paths.
 *
 * Activation gate (WatchdogCheck): the scheduler activates when
 * g_snes->vIrqEnabled is true, replacing the previous snes_frame_counter>0
 * gate. This correctly handles games (like Star Fox) that use V-IRQ without
 * NMI: the scheduler starts driving when the game enables V-IRQ.
 *
 * NTSC timing constants:
 *   262 scanlines x 1364 dots/scanline = 357,368 master cycles/frame
 *   VBlank starts at scanline 225 (scanline 0 = first visible line)
 *   Each scanline = 1364 master cycles
 *
 * Star Fox IRQ cadence (empirically measured 2026-06-23):
 *   vTimer = 207 (constant, never re-armed to a different scanline)
 *   Phase sequence per frame: trans_flag 02->04->06->00 (3 phases, 1 per frame)
 *   Each VBlank: ONE IRQ delivery at scanline 207 advances one phase
 */

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Cost assigned to each WatchdogCheck tick when running under the scheduler.
 *
 * Calibration (empirically measured 2026-06-23 against Star Fox boot):
 *   The scheduler tick fires every 10,000 WatchdogCheck calls (threshold in
 *   WatchdogCheck). Star Fox's tight transfer_l spin-wait runs at approximately
 *   357 million WatchdogCheck calls per wall-clock second, which means ~35,700
 *   scheduler ticks per second. To advance one NTSC frame (357,368 cycles) per
 *   wall-clock second of real-time execution, each tick must account for:
 *     357,368 cycles / 35,700 ticks ≈ 10 cycles/tick   (was original value)
 *
 *   But empirical observation showed the scheduler running at ~1 scheduler-fps
 *   rather than ~60. Root cause: each WatchdogCheck call (triggering once per
 *   10,000 blocks) advances only 10 simulated cycles. With ~357M WatchdogCheck-
 *   triggering blocks per wall-clock second, the scheduler sees only:
 *     (357M / 10,000) × 10 = 357,000 simulated cycles per wall-clock second
 *   That is exactly 1 NTSC frame per second — confirming the 60× shortfall.
 *
 *   Fix: increase SCHED_BLOCK_COST so the scheduler advances ~60 frames per
 *   wall-clock second. Target: 357,000 cycles per 1/60 second = 5,950,000
 *   simulated cycles per second. Each sched_tick fires 35,700 times/sec (same
 *   threshold), so cost per tick = 5,950,000 / 35,700 ≈ 167 cycles.
 *   Rounded to 500 for margin (allows for variable spin density):
 *     ~500 × 35,700 = 17,850,000 simulated cycles/sec ≈ 50 NTSC frames/sec
 *
 *   This value is a HEURISTIC. Real blocks vary wildly in cycle cost. The
 *   scheduler should not be used for cycle-accurate timing; its role is to
 *   deliver IRQ at approximately the right game-time scanline so spin-waits
 *   on IRQ-set flags resolve correctly and the game progresses.
 *
 *   If the game runs too fast (IRQs fire before spin completes): reduce cost.
 *   If the game runs too slow (same problem as before): increase cost.
 *
 *   Overridable via SF_SCHED_BLOCK_COST env var (parsed in WatchdogCheck init).
 */
#define SCHED_BLOCK_COST_DEFAULT  500u
/* Active value: overridden by SF_SCHED_BLOCK_COST env var if set. */
extern uint32_t g_sched_block_cost;
#define SCHED_BLOCK_COST  g_sched_block_cost

/* NTSC scanline geometry */
#define SCHED_CYCLES_PER_SCANLINE  1364u
#define SCHED_VBLANK_SCANLINE       225u   /* first VBlank scanline (0-based) */
#define SCHED_TOTAL_SCANLINES       262u
#define SCHED_CYCLES_PER_FRAME  \
    ((uint32_t)(SCHED_CYCLES_PER_SCANLINE) * (uint32_t)(SCHED_TOTAL_SCANLINES))

/* 1 when SF_SCHED=1 env var was set at startup; checked in WatchdogCheck. */
extern int g_sched_enabled;

/*
 * Optional NMI handler callback -- game layer registers this if it wants the
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

/*
 * Per-frame present hook -- fired on EVERY VBlank edge regardless of nmiEnabled
 * (Star Fox uses V-IRQ only and never enables NMI, so the nmi handler path
 * above does not fire for it). The game registers this to present one frame to
 * the host per simulated frame while the real game loop runs to never-return
 * inside I_RESET. It MUST be "boring": no recompiled CPU re-entry, no game-RAM
 * mutation (debug instrumentation aside), no interrupt delivery. It may block
 * for frame pacing (SDL_Delay). Reentrancy-guarded by the caller.
 */
typedef void (*SchedFrameFunc)(void);
extern SchedFrameFunc g_sched_frame_hook;

/* Reset the per-frame cycle/scanline accumulators. Call once at the start of
 * each host frame (before WatchdogFrameStart or alongside it). */
void sched_frame_start(void);

/*
 * Advance the scheduler clock by block_cost master cycles. Checks whether the
 * new cycle position has crossed a VBlank or VTIMER boundary and delivers the
 * corresponding interrupt if conditions are met:
 *
 *   VBlank: fires once when cycle position crosses into scanline 225+ (VBlank).
 *        Edge-latched: only one delivery per VBlank entry regardless of how
 *        many ticks land above scanline 225. Reentrancy-guarded.
 *        Always increments snes_frame_counter (regardless of nmiEnabled).
 *        If nmiEnabled: sets inNmi, calls g_sched_nmi_handler() if non-NULL.
 *
 *   IRQ: fires when the tick's scanline range crosses vTimer (prev < vTimer <=
 *        current) AND g_snes->vIrqEnabled AND cpu._flag_I == 0 AND !g_in_coop_pump.
 *        Delegates to g_coop_irq_pump() (same handler the watchdog pump uses).
 *        Edge-latched per vTimer value per frame: only one delivery even if
 *        multiple ticks land in the same scanline neighborhood.
 *        Latch resets each VBlank so delivery happens once per frame.
 *
 * Must only be called from WatchdogCheck (i.e., the game's recompiled
 * execution thread). Not thread-safe.
 */
void sched_tick(uint32_t block_cost);

#ifdef __cplusplus
}
#endif
