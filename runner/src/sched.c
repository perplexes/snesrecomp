/*
 * sched.c — cycle/scanline scheduler for SNES static recompiler.
 *
 * See sched.h for design rationale and timing constants.
 *
 * IMPLEMENTATION NOTES
 * ────────────────────
 * Cycle counter:  uint32_t g_sched_cycles — rolls over at SCHED_CYCLES_PER_FRAME.
 * Scanline:       derived as g_sched_cycles / SCHED_CYCLES_PER_SCANLINE.
 *
 * NMI latch:      g_sched_nmi_fired — set when NMI is delivered this VBlank;
 *                 cleared in sched_frame_start() so NMI fires again next frame.
 *
 * IRQ latch:      g_sched_irq_scanline — records the scanline on which IRQ
 *                 was last delivered; prevents re-firing on the same scanline.
 *                 Reset to 0xFFFF in sched_frame_start().
 *
 * Reentrancy:     Both NMI and IRQ delivery check/set g_in_coop_pump (the
 *                 same guard WatchdogCheck already uses) so the scheduler
 *                 cannot nest interrupt delivery.
 *
 * IRQ delivery:   Routed through g_coop_irq_pump (the existing game-registered
 *                 handler). This keeps the engine game-agnostic: the engine
 *                 knows nothing about irqcode_l or Star Fox's register ABI.
 *                 The pump already does push/run/restore correctly.
 */

#include "sched.h"
#include "common_cpu_infra.h"
#include "cpu_state.h"
#include "snes/snes.h"      /* Snes struct: nmiEnabled, vIrqEnabled, vTimer, inNmi, inVblank */

#include <stdlib.h>         /* getenv */
#include <stdio.h>

/* ── Public globals ────────────────────────────────────────────────────── */

int           g_sched_enabled     = 0;
SchedNmiFunc  g_sched_nmi_handler = NULL;

/* ── Internal state ────────────────────────────────────────────────────── */

static uint32_t g_sched_cycles        = 0;   /* master cycles into current frame */
static int      g_sched_nmi_fired     = 0;   /* edge latch: NMI delivered this VBlank */
static uint16_t g_sched_irq_scanline  = 0xFFFF; /* scanline of last IRQ delivery */

/* ── External symbols (defined elsewhere in the engine) ────────────────── */

extern CpuState   g_cpu;            /* cpu_state.c */
extern Snes      *g_snes;           /* common_cpu_infra.c / common_rtl.c */
extern int        snes_frame_counter; /* snes/snes.h / common_rtl.c */

/* g_in_coop_pump and g_coop_irq_pump are declared in common_cpu_infra.h,
 * which is included above via sched.h's transitive includes. No re-declaration
 * needed here. */

/* ── API ────────────────────────────────────────────────────────────────── */

void sched_frame_start(void) {
    g_sched_cycles       = 0;
    g_sched_nmi_fired    = 0;
    g_sched_irq_scanline = 0xFFFF;
}

void sched_tick(uint32_t block_cost) {
    /* Advance cycle accumulator, wrapping at one frame. */
    g_sched_cycles += block_cost;
    if (g_sched_cycles >= SCHED_CYCLES_PER_FRAME) {
        g_sched_cycles -= SCHED_CYCLES_PER_FRAME;
    }

    uint16_t scanline = (uint16_t)(g_sched_cycles / SCHED_CYCLES_PER_SCANLINE);

    /* ── NMI (VBlank) ──────────────────────────────────────────────────── */
    /* Deliver at most once per VBlank entry. NMI is not gated on P.I — it
     * fires whenever nmiEnabled is set, regardless of the interrupt-disable
     * flag (that is the hardware contract for NMI on the 65816). */
    if (!g_sched_nmi_fired && scanline >= SCHED_VBLANK_SCANLINE) {
        /* Only deliver if NMITIMEN bit 7 (nmiEnabled) is set and we are not
         * already inside an interrupt handler. */
        if (g_snes && g_snes->nmiEnabled && !g_in_coop_pump) {
            g_sched_nmi_fired = 1;

            /* Update hardware flags the game can read ($4210/$4212). */
            g_snes->inNmi    = true;
            g_snes->inVblank = true;

            /* Advance the global frame counter (mirrors the hardware NMI
             * incrementing the game's per-frame counter). This is what the
             * watchdog check gates on (snes_frame_counter == 0 = boot). */
            snes_frame_counter++;

            /* Call the game-registered NMI handler if provided. The
             * reentrancy guard is shared with g_coop_irq_pump to prevent
             * NMI from nested within IRQ and vice-versa. */
            if (g_sched_nmi_handler) {
                g_in_coop_pump = 1;
                g_sched_nmi_handler();
                g_in_coop_pump = 0;
            }
        } else if (!g_snes || !g_snes->nmiEnabled) {
            /* nmiEnabled is off — still set the latch so we don't spam on
             * every tick above line 225 when NMI is disabled. */
            g_sched_nmi_fired = 1;
        }
        /* If g_in_coop_pump: skip silently; we'll try again on the next
         * tick. Do NOT set g_sched_nmi_fired so we retry. */
    }

    /* Clear inVblank when we wrap back into the active display area. */
    if (scanline < SCHED_VBLANK_SCANLINE && g_snes && g_snes->inVblank) {
        g_snes->inVblank = false;
    }

    /* ── IRQ (VTIMER) ──────────────────────────────────────────────────── */
    /* Fire when the current scanline matches vTimer, vIrqEnabled is set,
     * the 65816 interrupt-disable flag is CLEAR (P.I == 0), and we have
     * not already fired IRQ on this scanline this frame. */
    if (g_snes && g_snes->vIrqEnabled &&
        scanline == g_snes->vTimer &&
        scanline != g_sched_irq_scanline &&
        !g_cpu._flag_I &&
        !g_in_coop_pump &&
        g_coop_irq_pump)
    {
        g_sched_irq_scanline = scanline;   /* latch: one IRQ per scanline crossing */

        /* Mark IRQ pending in hardware register ($4211). */
        if (g_snes) g_snes->inIrq = true;

        /* Deliver via the game-registered pump (save/run/restore pattern). */
        g_in_coop_pump = 1;
        g_coop_irq_pump();
        g_in_coop_pump = 0;

        /* Clear IRQ pending after handler consumed it. */
        if (g_snes) g_snes->inIrq = false;
    }
}
