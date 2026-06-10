#ifndef AUDIO_TRACE_H
#define AUDIO_TRACE_H

/* Always-on audio observability rings.
 *
 * Captures, continuously from process start (Release/Production too, per
 * the ring-buffer observability discipline):
 *
 *   1. PCM ring   — every native DSP output sample dsp_cycle produces,
 *                   recorded BEFORE the output-ring overflow check, so
 *                   samples dropped at the dsp->sampleBuffer cap are
 *                   still visible here.
 *   2. Event ring — every DSP register write (KON/KOF/pitch/volume/...),
 *                   overflow-drop runs, and consume (dsp_getSamples)
 *                   events, all timestamped in native-sample time.
 *   3. Counters   — produced/dropped/consumed totals, producer
 *                   attribution (CPU-thread catch-up vs audio-thread
 *                   top-up), output-ring occupancy high-water, and a
 *                   once-per-second snapshot ring for rate analysis.
 *
 * All record hooks run under RtlApuLock (dsp_cycle / dsp_write /
 * dsp_getSamples are only reached with the APU lock held), so plain
 * fields suffice. Dump/query entry points take RtlApuLock themselves.
 */

#include <stdint.h>

/* 2^22 native samples ~= 131 s @ 32 kHz (16 MiB, stereo int16). */
#define AUDIO_TRACE_PCM_RING   (1u << 22)
/* DSP register writes run a few hundred per frame at most; 2^19 entries
 * (~8 MiB) holds a full multi-minute session without evicting boot. */
#define AUDIO_TRACE_EVENT_RING (1u << 19)
/* Once-per-second stat snapshots: ~68 min. */
#define AUDIO_TRACE_SNAP_RING  (1u << 12)

enum {
  AUDIO_TRACE_EV_REG     = 1, /* DSP register write: addr, val            */
  AUDIO_TRACE_EV_DROP    = 2, /* output-ring overflow run: aux = run len  */
  AUDIO_TRACE_EV_CONSUME = 3, /* dsp_getSamples: aux = avail after read   */
};

/* Producer attribution for samples (who is cycling the APU). */
enum {
  AUDIO_TRACE_PRODUCER_UNKNOWN = 0,
  AUDIO_TRACE_PRODUCER_CPU     = 1, /* snes_catchupApu (CPU thread)       */
  AUDIO_TRACE_PRODUCER_AUDIO   = 2, /* RtlRenderAudio top-up (audio thread)*/
};

typedef struct AudioTraceEvent {
  uint64_t sample_idx; /* native-sample clock when the event occurred */
  uint32_t aux;        /* DROP: run length; CONSUME: ring avail after */
  uint8_t  type;
  uint8_t  addr;       /* REG only */
  uint8_t  val;        /* REG only */
  uint8_t  producer;   /* who was cycling the APU at the time */
} AudioTraceEvent;

typedef struct AudioTraceSnap {
  uint64_t wall_ms;
  uint64_t produced;
  uint64_t dropped;
  uint64_t consumed;
  uint32_t occupancy;  /* output-ring fill at snapshot time */
} AudioTraceSnap;

typedef struct AudioTraceStats {
  uint64_t produced;          /* total native samples generated         */
  uint64_t produced_cpu;      /* ... by CPU-thread catch-up             */
  uint64_t produced_audio;    /* ... by audio-thread top-up             */
  uint64_t dropped;           /* total samples lost to ring overflow    */
  uint64_t drop_runs;         /* number of distinct drop bursts         */
  uint64_t consumed;          /* total native samples read for output   */
  uint64_t consume_calls;     /* dsp_getSamples calls (audio callbacks) */
  uint64_t reg_writes;        /* DSP register writes                    */
  uint64_t kon_writes;        /* writes to $4C (KON)                    */
  uint32_t occupancy_highwater;
  uint64_t event_count;       /* events recorded (monotonic)            */
  uint64_t snap_count;        /* snapshots recorded (monotonic)         */
  /* APU catch-up pacing (rtl_accumulate_apu_catchup):                  */
  uint64_t pace_baseline_cycles;  /* wall-clock cycles injected while   */
                                  /* no consumer was draining the ring  */
  uint64_t pace_accumulate_calls; /* catch-up accumulations (APU touches)*/
  uint32_t pace_consumer_active;  /* consumer draining at last catch-up */
} AudioTraceStats;

/* ---- record hooks (call sites: dsp.c, snes.c, common_rtl.c) ---- */
void audio_trace_on_sample(int16_t l, int16_t r, int dropped, uint32_t ring_fill);
void audio_trace_on_reg_write(uint8_t addr, uint8_t val);
void audio_trace_on_consume(uint64_t read_idx, uint32_t count, uint32_t avail_after);
void audio_trace_set_producer(int producer);
/* Per catch-up accumulation: consumer state + wall-clock baseline cycles
 * injected (0 when a consumer is draining or no wall time elapsed). */
void audio_trace_on_pace(int consumer_active, uint32_t baseline_cycles);

/* Monotonic wall-clock milliseconds — the same timebase the snapshot ring
 * uses, exported so the catch-up pacer and any analysis share one clock. */
uint64_t audio_trace_wall_ms(void);

/* ---- query/dump (any thread; takes RtlApuLock internally) ---- */
void audio_trace_get_stats(AudioTraceStats *out);
/* Copy events [first_idx, first_idx+max) into out; returns count copied.
 * first/oldest available index is written to *oldest. */
uint32_t audio_trace_copy_events(uint64_t first_idx, uint32_t max,
                                 AudioTraceEvent *out, uint64_t *oldest);
uint32_t audio_trace_copy_snaps(uint64_t first_idx, uint32_t max,
                                AudioTraceSnap *out, uint64_t *oldest);
/* Write a 32 kHz stereo 16-bit WAV of PCM-ring samples
 * [start_idx, start_idx+count). start_idx<0 / count==0 mean "everything
 * still in the ring". Returns 0 on success, writes the actually-dumped
 * range to *out_start/*out_count. */
int audio_trace_dump_wav(const char *path, int64_t start_idx, uint64_t count,
                         uint64_t *out_start, uint64_t *out_count);

#endif
