#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "audio_trace.h"

#ifdef _WIN32
#include <windows.h>
static uint64_t wall_ms(void) { return (uint64_t)GetTickCount64(); }
#else
#include <time.h>
static uint64_t wall_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000);
}
#endif

/* Provided by the game's main.c — serialises against both APU producers. */
void RtlApuLock(void);
void RtlApuUnlock(void);

static int16_t s_pcm[AUDIO_TRACE_PCM_RING * 2];
static AudioTraceEvent s_events[AUDIO_TRACE_EVENT_RING];
static AudioTraceSnap s_snaps[AUDIO_TRACE_SNAP_RING];
static AudioTraceStats s_stats;
static int s_producer = AUDIO_TRACE_PRODUCER_UNKNOWN;
/* Open drop run: index into s_events of the DROP event being extended,
 * or UINT64_MAX when the last recorded sample was not dropped. */
static uint64_t s_open_drop_event = UINT64_MAX;
static uint64_t s_last_snap_ms = 0;

static AudioTraceEvent *push_event(uint8_t type) {
  AudioTraceEvent *e = &s_events[s_stats.event_count & (AUDIO_TRACE_EVENT_RING - 1)];
  s_stats.event_count++;
  e->sample_idx = s_stats.produced;
  e->aux = 0;
  e->type = type;
  e->addr = 0;
  e->val = 0;
  e->producer = (uint8_t)s_producer;
  return e;
}

static void maybe_snap(uint32_t ring_fill) {
  uint64_t now = wall_ms();
  if (now - s_last_snap_ms < 1000) return;
  s_last_snap_ms = now;
  AudioTraceSnap *s = &s_snaps[s_stats.snap_count & (AUDIO_TRACE_SNAP_RING - 1)];
  s_stats.snap_count++;
  s->wall_ms = now;
  s->produced = s_stats.produced;
  s->dropped = s_stats.dropped;
  s->consumed = s_stats.consumed;
  s->occupancy = ring_fill;
}

void audio_trace_set_producer(int producer) {
  s_producer = producer;
}

void audio_trace_on_sample(int16_t l, int16_t r, int dropped, uint32_t ring_fill) {
  uint32_t w = (uint32_t)(s_stats.produced & (AUDIO_TRACE_PCM_RING - 1));
  s_pcm[w * 2] = l;
  s_pcm[w * 2 + 1] = r;
  if (dropped) {
    if (s_open_drop_event != UINT64_MAX &&
        s_stats.event_count - s_open_drop_event <= AUDIO_TRACE_EVENT_RING) {
      s_events[s_open_drop_event & (AUDIO_TRACE_EVENT_RING - 1)].aux++;
    } else {
      s_open_drop_event = s_stats.event_count;
      push_event(AUDIO_TRACE_EV_DROP)->aux = 1;
      s_stats.drop_runs++;
    }
    s_stats.dropped++;
  } else {
    s_open_drop_event = UINT64_MAX;
  }
  s_stats.produced++;
  if (s_producer == AUDIO_TRACE_PRODUCER_CPU) s_stats.produced_cpu++;
  else if (s_producer == AUDIO_TRACE_PRODUCER_AUDIO) s_stats.produced_audio++;
  if (ring_fill > s_stats.occupancy_highwater) s_stats.occupancy_highwater = ring_fill;
  maybe_snap(ring_fill);
}

void audio_trace_on_reg_write(uint8_t addr, uint8_t val) {
  AudioTraceEvent *e = push_event(AUDIO_TRACE_EV_REG);
  e->addr = addr;
  e->val = val;
  s_stats.reg_writes++;
  if (addr == 0x4c && val != 0) s_stats.kon_writes++;
  s_open_drop_event = UINT64_MAX;
}

void audio_trace_on_pace(int consumer_active, uint32_t baseline_cycles) {
  s_stats.pace_consumer_active = (uint32_t)(consumer_active != 0);
  s_stats.pace_baseline_cycles += baseline_cycles;
  s_stats.pace_accumulate_calls++;
}

uint64_t audio_trace_wall_ms(void) {
  return wall_ms();
}

void audio_trace_on_consume(uint64_t read_idx, uint32_t count, uint32_t avail_after) {
  AudioTraceEvent *e = push_event(AUDIO_TRACE_EV_CONSUME);
  e->aux = avail_after;
  (void)read_idx;
  s_stats.consumed += count;
  s_stats.consume_calls++;
  s_open_drop_event = UINT64_MAX;
}

void audio_trace_get_stats(AudioTraceStats *out) {
  RtlApuLock();
  *out = s_stats;
  RtlApuUnlock();
}

uint32_t audio_trace_copy_events(uint64_t first_idx, uint32_t max,
                                 AudioTraceEvent *out, uint64_t *oldest) {
  RtlApuLock();
  uint64_t total = s_stats.event_count;
  uint64_t old = total > AUDIO_TRACE_EVENT_RING ? total - AUDIO_TRACE_EVENT_RING : 0;
  if (oldest) *oldest = old;
  if (first_idx < old) first_idx = old;
  uint32_t n = 0;
  while (first_idx + n < total && n < max) {
    out[n] = s_events[(first_idx + n) & (AUDIO_TRACE_EVENT_RING - 1)];
    n++;
  }
  RtlApuUnlock();
  return n;
}

uint32_t audio_trace_copy_snaps(uint64_t first_idx, uint32_t max,
                                AudioTraceSnap *out, uint64_t *oldest) {
  RtlApuLock();
  uint64_t total = s_stats.snap_count;
  uint64_t old = total > AUDIO_TRACE_SNAP_RING ? total - AUDIO_TRACE_SNAP_RING : 0;
  if (oldest) *oldest = old;
  if (first_idx < old) first_idx = old;
  uint32_t n = 0;
  while (first_idx + n < total && n < max) {
    out[n] = s_snaps[(first_idx + n) & (AUDIO_TRACE_SNAP_RING - 1)];
    n++;
  }
  RtlApuUnlock();
  return n;
}

int audio_trace_dump_wav(const char *path, int64_t start_idx, uint64_t count,
                         uint64_t *out_start, uint64_t *out_count) {
  /* Snapshot the write head under the lock; the slice [oldest, total) is
   * then stable without the lock — ring slots are append-only and only
   * lapped after a full 131 s revolution, far longer than any dump. */
  RtlApuLock();
  uint64_t total = s_stats.produced;
  RtlApuUnlock();
  uint64_t oldest = total > AUDIO_TRACE_PCM_RING ? total - AUDIO_TRACE_PCM_RING : 0;
  uint64_t start = (start_idx < 0) ? oldest : (uint64_t)start_idx;
  if (start < oldest) start = oldest;
  if (start > total) start = total;
  uint64_t avail = total - start;
  if (count == 0 || count > avail) count = avail;

  FILE *f = fopen(path, "wb");
  if (!f) return -1;
  uint32_t data_bytes = (uint32_t)(count * 4);
  uint32_t sample_rate = 32000; /* host plays the native stream 1:1 at 32000 */
  uint32_t byte_rate = sample_rate * 4;
  uint32_t riff_size = 36 + data_bytes;
  uint16_t fmt16;
  uint32_t fmt32;
  fwrite("RIFF", 1, 4, f);
  fwrite(&riff_size, 4, 1, f);
  fwrite("WAVEfmt ", 1, 8, f);
  fmt32 = 16;          fwrite(&fmt32, 4, 1, f); /* fmt chunk size  */
  fmt16 = 1;           fwrite(&fmt16, 2, 1, f); /* PCM             */
  fmt16 = 2;           fwrite(&fmt16, 2, 1, f); /* stereo          */
  fwrite(&sample_rate, 4, 1, f);
  fwrite(&byte_rate, 4, 1, f);
  fmt16 = 4;           fwrite(&fmt16, 2, 1, f); /* block align     */
  fmt16 = 16;          fwrite(&fmt16, 2, 1, f); /* bits per sample */
  fwrite("data", 1, 4, f);
  fwrite(&data_bytes, 4, 1, f);
  for (uint64_t i = 0; i < count; ) {
    uint32_t r = (uint32_t)((start + i) & (AUDIO_TRACE_PCM_RING - 1));
    /* contiguous run up to the ring wrap point */
    uint64_t run = AUDIO_TRACE_PCM_RING - r;
    if (run > count - i) run = count - i;
    fwrite(&s_pcm[(uint64_t)r * 2], 4, (size_t)run, f);
    i += run;
  }
  fclose(f);
  if (out_start) *out_start = start;
  if (out_count) *out_count = count;
  return 0;
}
