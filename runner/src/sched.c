/*
 * sched.c — cycle/scanline scheduler for SNES static recompiler.
 *
 * See sched.h for design rationale and timing constants.
 *
 * IMPLEMENTATION NOTES
 * ────────────────────
 * Cycle counter:  uint32_t g_sched_cycles — accumulates within the current frame;
 *                 reset by sched_frame_start() each frame (called from
 *                 WatchdogFrameStart). Frame wrap is handled inside sched_tick()
 *                 to keep latch resets in sync with the internal counter reset.
 *
 * Previous scanline: g_sched_prev_scanline — the scanline at the END of the
 *                 previous sched_tick() call. Used for crossing detection so
 *                 that IRQ/NMI are not missed when a tick straddles a boundary
 *                 (e.g. advances from scanline 99 → 101, crossing vTimer=100).
 *
 * NMI latch:      g_sched_nmi_fired — set when NMI is delivered this VBlank;
 *                 cleared in sched_frame_start() (and on internal frame wrap).
 *
 * IRQ latch:      g_sched_irq_scanline — records the scanline on which IRQ
 *                 was last delivered; prevents re-firing on the same scanline.
 *                 Reset to 0xFFFF in sched_frame_start() (and on internal wrap).
 *
 * Reentrancy:     Both NMI and IRQ delivery check/set g_in_coop_pump (the
 *                 same guard WatchdogCheck already uses) so the scheduler
 *                 cannot nest interrupt delivery. NMI blocked by the guard is
 *                 NOT marked consumed — the latch stays clear so the next tick
 *                 retries delivery once the guard drops.
 *
 * IRQ delivery:   Routed through g_coop_irq_pump (the existing game-registered
 *                 handler). This keeps the engine game-agnostic: the engine
 *                 knows nothing about irqcode_l or Star Fox's register ABI.
 *                 The pump saves/runs/restores CpuState so scheduler delivery
 *                 is transparent to the interrupted spin.
 *
 * BUGS ADDRESSED (post-Codex review)
 * ────────────────────────────────────
 * - Large block_cost could advance cycles by >> 1 frame: use modulo not
 *   single subtraction, and reset latches on every internal frame crossing.
 * - Scanline equality too fragile: use prev→cur crossing detection.
 * - NMI edge missed when tick wraps past scanline 225: checked before modulo.
 * - Frame-wrap/latch desync: latches are reset inside sched_tick on wrap so
 *   they always stay in sync with the internal counter.
 * - NMI blocked by g_in_coop_pump: don't consume the latch; retry next tick.
 */

#include "sched.h"
#include "common_cpu_infra.h"
#include "cpu_state.h"
#include "snes/snes.h"      /* Snes struct: nmiEnabled, vIrqEnabled, vTimer, inNmi, inVblank */

#include <stdlib.h>
#include <stdio.h>

/* ── Public globals ────────────────────────────────────────────────────── */

int           g_sched_enabled     = 0;
SchedNmiFunc  g_sched_nmi_handler = NULL;

/* ── Internal state ────────────────────────────────────────────────────── */

static uint32_t g_sched_cycles        = 0;       /* master cycles within current frame */
static uint16_t g_sched_prev_scanline = 0;       /* scanline at end of last tick */
static int      g_sched_nmi_fired     = 0;       /* edge latch: NMI delivered this VBlank */
static uint16_t g_sched_irq_scanline  = 0xFFFF;  /* scanline of last IRQ delivery */

/* ── External symbols (defined elsewhere in the engine) ────────────────── */

extern CpuState   g_cpu;            /* cpu_state.c */
extern Snes      *g_snes;           /* common_rtl.c */
extern int        snes_frame_counter; /* snes/snes.h — defined in snes.c */

/* g_in_coop_pump and g_coop_irq_pump are declared in common_cpu_infra.h. */

/* ── Internal helpers ───────────────────────────────────────────────────── */

/* Reset per-frame latch state. Called by sched_frame_start() and also
 * internally when sched_tick() detects a frame wrap (so the latches always
 * stay in sync with g_sched_cycles regardless of how often the caller
 * explicitly calls sched_frame_start). */
static void sched_reset_latches(void) {
    g_sched_nmi_fired    = 0;
    g_sched_irq_scanline = 0xFFFF;
    g_sched_prev_scanline = 0;
}

/* ── API ────────────────────────────────────────────────────────────────── */

void sched_frame_start(void) {
    g_sched_cycles = 0;
    sched_reset_latches();
}

void sched_tick(uint32_t block_cost) {
    if (!block_cost) return;

    /* ── Advance cycle counter with frame-wrap handling ─────────────────── */
    /* Use modulo so arbitrarily large block_cost values (including
     * block_cost >= 2*SCHED_CYCLES_PER_FRAME) are handled correctly.
     * We process at most two frame boundaries per tick: the common case
     * (cost < 1 frame) handles zero frame wraps; an extreme cost wraps once
     * or more but always lands inside [0, SCHED_CYCLES_PER_FRAME). */
    uint32_t prev_cycles = g_sched_cycles;
    g_sched_cycles += block_cost;

    /* Count how many full frames were crossed. Reset latches for each full
     * frame crossing so they are always in sync with the cycle counter. */
    int frames_crossed = 0;
    while (g_sched_cycles >= SCHED_CYCLES_PER_FRAME) {
        g_sched_cycles -= SCHED_CYCLES_PER_FRAME;
        frames_crossed++;
        /* Reset latches on each full-frame wrap so they stay synchronised.
         * A game that stalls for > 1 frame without returning to host is
         * pathological; this is a safety valve rather than a normal path. */
        sched_reset_latches();
    }

    /* Current and previous scanlines for crossing detection. */
    uint16_t scanline = (uint16_t)(g_sched_cycles / SCHED_CYCLES_PER_SCANLINE);

    /* g_sched_prev_scanline is maintained across calls. After a frame wrap
     * (sched_reset_latches cleared it to 0), use 0 as the prior position. */
    uint16_t prev_sl = g_sched_prev_scanline;
    g_sched_prev_scanline = scanline;  /* update for next call */

    /* ── NMI (VBlank) ──────────────────────────────────────────────────── */
    /* Deliver at most once per VBlank entry. NMI is not gated on P.I — it
     * fires whenever nmiEnabled is set, regardless of the interrupt-disable
     * flag (65816 hardware contract). Edge detected by scanline crossing into
     * the SCHED_VBLANK_SCANLINE+ region from below. */

    /* A VBlank edge occurred this tick if:
     *   - we are now at or above line 225 AND we were below it last tick, OR
     *   - a full frame crossing happened this tick (we wrapped through line 225). */
    int vblank_edge = (frames_crossed > 0) ||
                      (scanline >= SCHED_VBLANK_SCANLINE && prev_sl < SCHED_VBLANK_SCANLINE);

    if (vblank_edge && !g_sched_nmi_fired) {
        if (g_snes && g_snes->nmiEnabled && !g_in_coop_pump) {
            /* Consume the edge latch first so a re-entrant tick (from the
             * NMI handler) does not attempt to deliver a second NMI. */
            g_sched_nmi_fired = 1;

            /* $4210 bit 7 (inNmi) is set on VBlank start; $4212 bit 7
             * (inVblank) tracks the VBlank window. inNmi is read-clear
             * by the game ($4210 read clears it), so set it here and let
             * the hardware emulation clear it on $4210 read. We do NOT
             * clear inNmi ourselves after delivery. */
            g_snes->inNmi    = true;
            g_snes->inVblank = true;

            /* Advance the host frame counter. This is what the watchdog
             * hang-check uses to distinguish boot (== 0) from running. */
            snes_frame_counter++;

            /* Call the optional game-registered NMI handler. The guard is
             * shared with the pump so NMI cannot nest within IRQ. */
            if (g_sched_nmi_handler) {
                g_in_coop_pump = 1;
                g_sched_nmi_handler();
                g_in_coop_pump = 0;
            }
        } else if (!g_snes || !g_snes->nmiEnabled) {
            /* nmiEnabled is off — consume the edge so we don't retry on
             * every tick above line 225 for this VBlank window. */
            g_sched_nmi_fired = 1;
        }
        /* If g_in_coop_pump is set (we're inside an IRQ handler), do NOT
         * set g_sched_nmi_fired — let the next tick retry delivery once
         * the guard drops. Hardware NMI is an edge; model it as pending. */
    }

    /* Clear inVblank when we return to the active display area.
     * Only do this when no frame crossing happened this tick (a wrap means
     * we are back in the active area by definition, and VBlank was already
     * handled by the edge logic above). */
    if (frames_crossed == 0 && scanline < SCHED_VBLANK_SCANLINE &&
        g_snes && g_snes->inVblank) {
        g_snes->inVblank = false;
    }

    /* ── IRQ (VTIMER) ──────────────────────────────────────────────────── */
    /* Crossing detection: IRQ fires when vTimer is crossed or landed on.
     * "Crossed" means prev_sl < vTimer <= scanline (forward direction).
     * Frame-boundary case: if frames_crossed > 0 AND the target scanline
     * is between 0 and the new scanline, also trigger.
     *
     * Gated on: vIrqEnabled, P.I clear (!_flag_I), not in pump, pump registered,
     * and edge-latch (irq_scanline != vTimer this frame). */
    if (g_snes && g_snes->vIrqEnabled && !g_in_coop_pump && g_coop_irq_pump) {
        uint16_t vtimer = g_snes->vTimer;
        int irq_edge = 0;

        if (frames_crossed > 0) {
            /* Wrapped — IRQ target was somewhere in the frame we wrapped through.
             * Fire if target is in [0, scanline] (already passed in new frame)
             * or if it was in [prev_sl, TOTAL-1] (passed in old frame). Either
             * way, fire once now (edge-latch below prevents duplicates). */
            irq_edge = 1;
        } else {
            /* Normal forward crossing: prev_sl < vtimer <= scanline. */
            irq_edge = (prev_sl < vtimer && vtimer <= scanline);
            /* Also catch the degenerate case where we land exactly on vTimer
             * from scanline 0 (prev_sl == 0) without having crossed it
             * "from below" — e.g. first tick of the frame. */
            if (!irq_edge && prev_sl == 0 && vtimer == 0) {
                irq_edge = 1;
            }
        }

        if (irq_edge && vtimer != g_sched_irq_scanline && !g_cpu._flag_I) {
            g_sched_irq_scanline = vtimer;  /* latch: one IRQ per vTimer crossing */

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
}
