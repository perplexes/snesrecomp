#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * capture.h -- generic, env-gated per-frame capture (engine-side, game-agnostic).
 *
 * Purpose: record EVERY presented host frame together with its faithful
 * EMULATED-TIME timestamp so an offline assembler can build a smoothly-paced
 * video whose per-frame display durations come from real emulated timing
 * (including fast-forward gaps), instead of a naive constant FPS.
 *
 * Activation: set the environment variable RECOMP_CAPTURE_DIR=<dir>. When unset
 * the capture path is completely inert (zero cost; no behavior change). Optional
 * RECOMP_CAPTURE_MAX=<n> caps the number of captured frames (0/unset = no cap).
 *
 * Per captured frame (in present order, never overwritten, never skipped):
 *   <dir>/frameNNNNNN.png            -- lossless PNG (RGB8) of the host framebuffer
 *   one appended line in <dir>/manifest.csv:
 *       frame_index,sched_cycles,emu_ms,wall_ms,snes_frame
 *   where:
 *     frame_index  present-order index (0-based)
 *     sched_cycles g_sched_total_cycles snapshot (monotonic master cycles)
 *     emu_ms       sched_cycles / 21477.272  (faithful emulated milliseconds)
 *     wall_ms      host wall-clock ms since the first captured frame
 *     snes_frame   snes_frame_counter snapshot (cross-check)
 *
 * The caller passes its finished framebuffer as packed 32-bit pixels in
 * B,G,R,pad byte order (the runner's g_my_pixels layout), `pitch_bytes` per row.
 */

/* 1 if RECOMP_CAPTURE_DIR is set (capture armed), else 0. Cheap after first call. */
int Capture_Enabled(void);

/* Capture one presented frame. No-op when capture is disabled or the frame cap
 * was reached. `pixels` is width*height packed B,G,R,pad; `pitch_bytes` is the
 * byte stride of one row (typically width*4). */
void Capture_Frame(const uint8_t *pixels, int width, int height, size_t pitch_bytes);

#ifdef __cplusplus
}
#endif
