/*
 * sched.c -- cycle/scanline scheduler for SNES static recompiler.
 *
 * See sched.h for design rationale and timing constants.
 *
 * IMPLEMENTATION NOTES
 * --------------------
 * Cycle counter:  uint32_t g_sched_cycles -- accumulates within the current frame;
 *                 reset by sched_frame_start() each frame (called from
 *                 WatchdogFrameStart). Frame wrap is handled inside sched_tick()
 *                 to keep latch resets in sync with the internal counter reset.
 *
 * Previous scanline: g_sched_prev_scanline -- the scanline at the END of the
 *                 previous sched_tick() call. Used for crossing detection so
 *                 that IRQ/NMI are not missed when a tick straddles a boundary
 *                 (e.g. advances from scanline 99 to 101, crossing vTimer=100).
 *
 * VBlank / snes_frame_counter:
 *                 snes_frame_counter is incremented on EVERY VBlank crossing
 *                 regardless of whether NMI is enabled. This allows games that
 *                 use V-IRQ only (not NMI) -- like Star Fox -- to advance the
 *                 watchdog hang counter without needing NMI delivery. The
 *                 NMI handler is only called when nmiEnabled is set.
 *
 * NMI latch:      g_sched_nmi_fired -- set when VBlank is processed this frame;
 *                 cleared in sched_frame_start() (and on internal frame wrap).
 *
 * IRQ latch:      g_sched_irq_scanline -- records the scanline on which IRQ
 *                 was last delivered; prevents re-firing on the same scanline.
 *                 Reset to 0xFFFF in sched_frame_start() (and on internal wrap).
 *                 This matches real hardware: V-IRQ fires at most once per frame
 *                 when the beam crosses the target scanline.
 *
 * Reentrancy:     Both NMI and IRQ delivery check/set g_in_coop_pump (the
 *                 same guard WatchdogCheck already uses) so the scheduler
 *                 cannot nest interrupt delivery. NMI blocked by the guard is
 *                 NOT marked consumed -- the latch stays clear so the next tick
 *                 retries delivery once the guard drops.
 *
 * IRQ delivery:   Routed through g_coop_irq_pump (the existing game-registered
 *                 handler). This keeps the engine game-agnostic: the engine
 *                 knows nothing about irqcode_l or Star Fox's register ABI.
 *                 The pump is responsible for delivering the correct number of
 *                 IRQ phases (one per call for real-hardware behaviour; or a
 *                 full chain for accelerated cooperative mode via SF_FULLCHAIN).
 *
 * IRQ cadence for Star Fox (empirically measured 2026-06-23):
 *   - vTimer ALWAYS = 207; it never changes between phases.
 *   - Phase sequence per frame: trans_flag 02->04->06->00 (3 phases).
 *   - On real hardware: ONE IRQ fires at scanline 207 per frame; the chain
 *     advances ONE phase per frame (3 frames to complete: 02->04->06->00).
 *   - With SF_FULLCHAIN pump: all 3 phases delivered in one pump call.
 *   - Scheduler fires ONE IRQ at scanline 207 per frame (correct hardware model).
 *
 * BUGS ADDRESSED (post-Codex review and Phase 2A empirical testing)
 * -----------------------------------------------------------------
 * - Large block_cost could advance cycles by >> 1 frame: use modulo not
 *   single subtraction, and reset latches on every internal frame crossing.
 * - Scanline equality too fragile: use prev->cur crossing detection.
 * - NMI edge missed when tick wraps past scanline 225: checked before modulo.
 * - Frame-wrap/latch desync: latches are reset inside sched_tick on wrap so
 *   they always stay in sync with the internal counter reset.
 * - NMI blocked by g_in_coop_pump: don't consume the latch; retry next tick.
 * - snes_frame_counter never incremented for NMI-less games (Star Fox uses
 *   V-IRQ only): now always incremented on VBlank crossing (Phase 2A fix).
 * - Scheduler never activated: was gated on snes_frame_counter>0 (NMI-based),
 *   now gated on g_snes->vIrqEnabled in WatchdogCheck (Phase 2A fix).
 */

#include "sched.h"
#include "common_cpu_infra.h"
#include "cpu_state.h"
#include "snes/snes.h"      /* Snes struct: nmiEnabled, vIrqEnabled, vTimer, inNmi, inVblank */

#include <stdlib.h>
#include <stdio.h>

/* -- Public globals -------------------------------------------------------- */

int           g_sched_enabled     = 0;
SchedNmiFunc  g_sched_nmi_handler = NULL;
SchedFrameFunc g_sched_frame_hook = NULL;
uint32_t      g_sched_block_cost  = SCHED_BLOCK_COST_DEFAULT;

/* -- Internal state -------------------------------------------------------- */

static uint32_t g_sched_cycles        = 0;       /* master cycles within current frame */
static uint16_t g_sched_prev_scanline = 0;       /* scanline at end of last tick */
static int      g_sched_nmi_fired     = 0;       /* edge latch: VBlank processed this frame */
static uint16_t g_sched_irq_scanline  = 0xFFFF;  /* scanline of last V-IRQ delivery */
static uint16_t g_sched_hirq_scanline = 0xFFFF;  /* scanline of last H-IRQ delivery */

/* -- External symbols (defined elsewhere in the engine) -------------------- */

extern CpuState   g_cpu;            /* cpu_state.c */
extern Snes      *g_snes;           /* common_rtl.c */
extern int        snes_frame_counter; /* snes/snes.h -- defined in snes.c */

/* g_in_coop_pump and g_coop_irq_pump are declared in common_cpu_infra.h. */

/* -- Internal helpers ------------------------------------------------------- */

/* Reset per-frame latch state. Called by sched_frame_start() and also
 * internally when sched_tick() detects a frame wrap (so the latches always
 * stay in sync with g_sched_cycles regardless of how often the caller
 * explicitly calls sched_frame_start). */
static void sched_reset_latches(void) {
    g_sched_nmi_fired    = 0;
    g_sched_irq_scanline = 0xFFFF;
    g_sched_hirq_scanline = 0xFFFF;
    g_sched_prev_scanline = 0;
}

/* -- API -------------------------------------------------------------------- */

/* Current intra-scanline H-counter (0..SCHED_CYCLES_PER_SCANLINE-1) derived
 * from the master clock. Lets $4212 H-blank polling read a real beam position
 * instead of a synthetic fixed-step counter. Valid while the scheduler is the
 * active timekeeper (g_sched_enabled); callers fall back to the legacy model
 * otherwise. */
uint16_t sched_current_hpos(void) {
    /* Include cycles accumulated since the last WatchdogCheck flush
     * (g_pending_cycles, fed by cpu_cycle_tick per recompiled block). The
     * scheduler clock g_sched_cycles only advances on the ~10k-block watchdog
     * flush, so a tight $4212 H-blank busy-wait would otherwise read a frozen
     * beam for thousands of iterations between flushes. Adding the unflushed
     * pending cycles makes the H-counter advance every loop iteration, which is
     * both more truthful and what the game's H-blank polls expect. This only
     * READS g_pending_cycles (does not consume it); the normal flush is
     * unaffected, so there is no double-counting. */
    return (uint16_t)((g_sched_cycles + (uint32_t)g_pending_cycles)
                      % SCHED_CYCLES_PER_SCANLINE);
}

void sched_frame_start(void) {
    g_sched_cycles = 0;
    sched_reset_latches();
}

void sched_tick(uint32_t block_cost) {
    if (!block_cost) return;

    /* -- Advance cycle counter with frame-wrap handling -------------------- */
    /* Use modulo so arbitrarily large block_cost values (including
     * block_cost >= 2*SCHED_CYCLES_PER_FRAME) are handled correctly. */
    g_sched_cycles += block_cost;

    /* Count how many full frames were crossed. Reset latches for each full
     * frame crossing so they are always in sync with the cycle counter. */
    int frames_crossed = 0;
    while (g_sched_cycles >= SCHED_CYCLES_PER_FRAME) {
        g_sched_cycles -= SCHED_CYCLES_PER_FRAME;
        frames_crossed++;
        /* Reset latches on each full-frame wrap. */
        sched_reset_latches();
    }

    /* Current and previous scanlines for crossing detection. */
    uint16_t scanline = (uint16_t)(g_sched_cycles / SCHED_CYCLES_PER_SCANLINE);

    /* g_sched_prev_scanline is maintained across calls. After a frame wrap
     * (sched_reset_latches cleared it to 0), use 0 as the prior position. */
    uint16_t prev_sl = g_sched_prev_scanline;
    g_sched_prev_scanline = scanline;  /* update for next call */

    /* -- VBlank (NMI / frame counter) -------------------------------------- */
    /* A VBlank edge occurred this tick if:
     *   - we are now at or above line 225 AND we were below it last tick, OR
     *   - a full frame crossing happened this tick (we wrapped through line 225).
     *
     * On every VBlank edge, regardless of nmiEnabled:
     *   - Increment snes_frame_counter (so the hang watchdog works for games
     *     that use V-IRQ only and never enable NMI, like Star Fox).
     *   - Set inNmi/inVblank if nmiEnabled, and call g_sched_nmi_handler.
     *
     * NMI is not gated on P.I (65816 hardware: NMI fires regardless of I flag).
     */

    int vblank_edge = (frames_crossed > 0) ||
                      (scanline >= SCHED_VBLANK_SCANLINE && prev_sl < SCHED_VBLANK_SCANLINE);

    if (vblank_edge && !g_sched_nmi_fired) {
        if (!g_in_coop_pump) {
            /* Consume the edge latch first so a re-entrant tick (from the
             * NMI handler) does not attempt a second VBlank processing. */
            g_sched_nmi_fired = 1;

            /* Always advance the host frame counter on VBlank crossing,
             * regardless of nmiEnabled. This ensures snes_frame_counter
             * advances for games that use V-IRQ only (like Star Fox, which
             * never enables NMI), keeping the watchdog hang-check alive. */
            snes_frame_counter++;

            if (g_snes) {
                g_snes->inVblank = true;
            }

            /* Per-frame present hook: fire once per VBlank edge REGARDLESS of
             * nmiEnabled (Star Fox is V-IRQ only). This drives host frame
             * presentation while the real game loop runs to never-return inside
             * I_RESET. Guarded by g_in_coop_pump (held here) so a re-entrant
             * tick cannot double-fire; the hook must not run recompiled CPU. */
            if (g_sched_frame_hook) {
                g_in_coop_pump = 1;
                g_sched_frame_hook();
                g_in_coop_pump = 0;
            }

            /* If NMI is enabled, also signal the NMI hardware flag and
             * call the registered NMI handler. */
            if (g_snes && g_snes->nmiEnabled) {
                /* $4210 bit 7 (inNmi) is set on VBlank start; read-clear
                 * by the game ($4210 read clears it). */
                g_snes->inNmi = true;

                if (g_sched_nmi_handler) {
                    g_in_coop_pump = 1;
                    g_sched_nmi_handler();
                    g_in_coop_pump = 0;
                }
            }
        }
        /* If g_in_coop_pump is set (we're inside an IRQ handler), do NOT
         * set g_sched_nmi_fired -- let the next tick retry delivery once
         * the guard drops. Hardware NMI is an edge; model it as pending. */
    }

    /* Clear inVblank when we return to the active display area.
     * This must fire regardless of frames_crossed: if a frame was wrapped,
     * g_sched_cycles is now in the new frame and scanline < SCHED_VBLANK_SCANLINE
     * already (the active area), so inVblank should be cleared. */
    if (scanline < SCHED_VBLANK_SCANLINE && g_snes && g_snes->inVblank) {
        g_snes->inVblank = false;
    }

    /* -- IRQ (VTIMER) ------------------------------------------------------ */
    /* Crossing detection: IRQ fires when vTimer is crossed or landed on.
     * "Crossed" means prev_sl < vTimer <= scanline (forward direction).
     * Frame-boundary case: if frames_crossed > 0 AND the target scanline
     * is between 0 and the new scanline, also trigger.
     *
     * Gated on: vIrqEnabled, P.I clear (!_flag_I), not in pump, pump registered,
     * and edge-latch (irq_scanline != vTimer this frame).
     *
     * Hardware model: V-IRQ fires at most ONCE per frame when the beam crosses
     * the target scanline. The latch g_sched_irq_scanline prevents re-delivery
     * at the same scanline within the same frame. It is reset each VBlank
     * (sched_reset_latches), allowing delivery again in the next frame.
     *
     * The game-registered pump (g_coop_irq_pump) is responsible for delivering
     * the right number of IRQ phases. With SF_FULLCHAIN it runs all phases
     * in one call; without it, it runs one phase per call (real-hardware model). */
    if (getenv("SF_SCHED_TRACE")) {
        static long s_n = 0;
        if ((s_n++ % 1000) == 0) {
            fprintf(stderr, "[sched %ld] cycles=%u sl=%u prev_sl=%u vtimer=%u vIrqEn=%d _flag_I=%d in_pump=%d irq_latch=%u frame=%d\n",
                    s_n, g_sched_cycles, scanline, prev_sl,
                    g_snes ? g_snes->vTimer : 0xFFFF,
                    g_snes ? (int)g_snes->vIrqEnabled : -1,
                    g_cpu._flag_I, g_in_coop_pump, g_sched_irq_scanline,
                    snes_frame_counter);
            fflush(stderr);
        }
    }
    if (g_snes && g_snes->vIrqEnabled && !g_in_coop_pump && g_coop_irq_pump) {
        uint16_t vtimer = g_snes->vTimer;
        int irq_edge = 0;

        if (frames_crossed > 0) {
            /* Wrapped -- IRQ target was somewhere in the frame we wrapped through.
             * Fire if target is in [0, scanline] (already passed in new frame)
             * or if it was in [prev_sl, TOTAL-1] (passed in old frame). Either
             * way, fire once now (edge-latch below prevents duplicates). */
            irq_edge = 1;
        } else {
            /* Normal forward crossing: prev_sl < vtimer <= scanline. */
            irq_edge = (prev_sl < vtimer && vtimer <= scanline);
            /* Also catch the degenerate case where we land exactly on vTimer
             * from scanline 0 (prev_sl == 0) without having crossed it
             * "from below" -- e.g. first tick of the frame. */
            if (!irq_edge && prev_sl == 0 && vtimer == 0) {
                irq_edge = 1;
            }
        }

        if (irq_edge && vtimer != g_sched_irq_scanline && !g_cpu._flag_I) {
            g_sched_irq_scanline = vtimer;  /* latch: one IRQ per vTimer crossing per frame */

            /* $4211 bit 7 (inIrq): set before handler, hardware clears on $4211
             * read. Set it here; the game's $4211 read-clear path in the
             * hardware emulator will clear it. We also clear it after the pump
             * returns as a safety net for games that don't $4211-read-clear. */
            g_snes->inIrq = true;

            /* Deliver via the game-registered pump (CpuState save/run/restore). */
            g_in_coop_pump = 1;
            g_coop_irq_pump();
            g_in_coop_pump = 0;

            /* Safety-net clear (harmless if the handler already $4211-read-cleared). */
            g_snes->inIrq = false;
        }
    }

    /* -- IRQ (HTIMER) ------------------------------------------------------ */
    /* H-timer IRQ. NMITIMEN ($4200) bit 4 = hIrqEnabled. Hardware modes:
     *   - H-IRQ only (bit4, not bit5): IRQ on EVERY scanline when the
     *     H-counter reaches hTimer (262 deliveries per frame).
     *   - H-IRQ + V-IRQ (bit4 + bit5): IRQ once per frame at scanline vTimer,
     *     H-counter hTimer. The V-IRQ block above already delivers one IRQ per
     *     frame at vTimer, so the combined mode is handled there; we exclude it
     *     here (`!vIrqEnabled`) to avoid a double fire on the same crossing.
     *
     * Granularity: the scheduler ticks every ~10k recompiled blocks, so the
     * H-counter is sampled, not dot-exact. We therefore model H-IRQ as "fires
     * once when the beam advances onto a new scanline while H-IRQ is enabled",
     * i.e. one delivery per scanline crossing. Edge-latched on the scanline so
     * a tick that does not advance the scanline does not re-fire; P.I-gated and
     * reentrancy-guarded, exactly mirroring the V-IRQ path. The latch resets
     * each VBlank (sched_reset_latches) so delivery resumes every frame.
     *
     * Delivered through the SAME g_coop_irq_pump shim as V-IRQ -- the engine
     * stays game-agnostic; the pump runs the game's IRQ body. */
    if (g_snes && g_snes->hIrqEnabled && !g_snes->vIrqEnabled &&
        !g_in_coop_pump && g_coop_irq_pump) {
        int h_edge = 0;
        if (frames_crossed > 0) {
            /* Wrapped at least one frame -- scanline(s) certainly advanced. */
            h_edge = 1;
        } else if (scanline != prev_sl) {
            /* Advanced onto a new scanline -- one H-IRQ for the crossing. */
            h_edge = 1;
        }

        if (h_edge && scanline != g_sched_hirq_scanline && !g_cpu._flag_I) {
            g_sched_hirq_scanline = scanline;  /* latch: one H-IRQ per scanline */

            g_snes->inIrq = true;

            g_in_coop_pump = 1;
            g_coop_irq_pump();
            g_in_coop_pump = 0;

            g_snes->inIrq = false;
        }
    }
}
