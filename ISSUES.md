# snesrecomp — Known Issues (runner / recompiler level)

Game-specific issues live in each game repo's ISSUES.md; this file tracks
issues in the shared runner and recompiler that affect every port.

## OPEN: Music command can drop under turbo (APU port-write scheduler vs uncapped game clock)

**Status:** OPEN 2026-06-11. Deferred by choice ("maybe another time").
Runner-level — affects all games. Observed on Zelda ALttP: overworld music
did not start after entering the overworld while holding Turbo (Tab).
Self-heals at the next music transition. Normal-speed audio is unaffected
(post-fix validation: SMW 100% across two runs, MMX no misses, Zelda clean
at normal speed).

**Symptom:** while turboing, a one-shot music command written to an APU port
can be silently lost — area music stays silent until the next track change.
One-shot SFX can drop under turbo too (inaudible in practice at 5-10x speed).

**Mechanism (understood, not a regression):** since `bf64f0d` the runner
schedules CPU APU-port writes in APU-sample time (write-clock targets spaced
by the wall-time gap between writes; floor = produced clock, ceiling =
produced + 3 audio-callback quanta). Turbo runs the emulated game uncapped
while the audio device keeps consuming at real time, so port writes arrive at
5-10x wall rate against an APU advancing at 1x. The write stream compresses
against the latency ceiling; back-to-back writes to the same port can apply
with near-zero engine time between them, and the SPC engine (polling every
~64 samples of its own time) never observes the overwritten value. Pre-fix
behavior was equally lossy under turbo (wall-time port mutation gave a
command microseconds of APU time); the scheduler makes the loss bounded and
characterizable.

**Evidence path if it recurs:** keep the process alive and query the
always-on port rings (`audio_events filter=2` on the debug server, or SMW's
`tools/sfx_probe.py chain` pointed at the game's debug port — SMW 4377,
Zelda 4378, MMX 4379) — every command's fate (SEEN / LOST, with apply
spacing) is in the ring.

**Proposed hardening (when picked up):** in `RtlApuWrite`
(`runner/src/common_rtl.c`), when the latency ceiling is clamping (turbo
pressure), enforce a minimum ~2-engine-tick spacing (~128 samples) between
DISTINCT values applied to the same port and drop middle values of a burst
instead of compressing all spacing to zero — the engine then reliably sees
the last command of every burst, which is the one that matters for music.
Keep total latency bounded; do not stretch the wall clock (the MMX issue-4
"never bound catch-up to real time" rule still applies).
