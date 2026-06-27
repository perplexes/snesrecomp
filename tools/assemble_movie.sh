#!/usr/bin/env bash
# assemble_movie.sh — build a smoothly-paced video from a capture directory
# produced by the engine capture harness (RECOMP_CAPTURE_DIR; see
# runner/src/capture.c).
#
#   tools/assemble_movie.sh <capture_dir> <out.mp4>
#
# The capture dir holds frameNNNNNN.png (present order) + manifest.csv with
# columns: frame_index,sched_cycles,emu_ms,wall_ms,snes_frame
#
# Each frame is shown for the REAL emulated duration (emu_ms delta to the next
# frame), so fast-forward spans hold their frame longer instead of the movie
# being uniformly stretched. ffmpeg's concat demuxer consumes those per-frame
# durations (variable frame rate); we then resample to a constant FPS H.264
# (yuv420p, even dims) so it plays smoothly everywhere — holding/duplicating
# frames as needed to honor the emulated timing.
#
# Env overrides:
#   FPS    output constant frame rate (default 60)
#   SCALE  integer nearest-neighbor upscale of the 256x224 frames (default 3)
#   FALLBACK_FPS  per-frame rate used if emu_ms is unusable (default 60.0988)
set -euo pipefail

DIR="${1:?usage: assemble_movie.sh <capture_dir> <out.mp4>}"
OUT="${2:?usage: assemble_movie.sh <capture_dir> <out.mp4>}"
FPS="${FPS:-60}"
SCALE="${SCALE:-3}"
FALLBACK_FPS="${FALLBACK_FPS:-60.0988}"

MAN="$DIR/manifest.csv"
[ -f "$MAN" ] || { echo "ERROR: no manifest at $MAN" >&2; exit 1; }
command -v ffmpeg >/dev/null || { echo "ERROR: ffmpeg not found" >&2; exit 1; }

LIST="$(mktemp /tmp/sf_concat_XXXXXX.txt)"
trap 'rm -f "$LIST"' EXIT

# Build the ffmpeg concat list (absolute frame paths + per-frame durations from
# emu_ms deltas). Python handles the last-frame duration + an emu_ms sanity
# fallback to a fixed per-frame period.
python3 - "$DIR" "$MAN" "$LIST" "$FALLBACK_FPS" <<'PY'
import csv, os, sys
cap_dir, man, listpath, fallback_fps = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
rows = []
with open(man) as f:
    for r in csv.DictReader(f):
        rows.append((int(r["frame_index"]), float(r["emu_ms"])))
rows.sort()
n = len(rows)
if n == 0:
    sys.exit("ERROR: manifest has no frames")

# Decide timing source: real emu_ms deltas, or a fixed fallback period.
emu_span = rows[-1][1] - rows[0][1]
use_emu = emu_span > 1.0
fb = 1000.0 / fallback_fps

# Per-frame durations (seconds).
durs = []
for i in range(n):
    if use_emu and i < n - 1:
        d_ms = rows[i+1][1] - rows[i][1]
    else:
        d_ms = fb
    if d_ms < 1.0:        # guard against zero/negative (clamp to ~1kHz)
        d_ms = fb if not use_emu else max(d_ms, 1.0)
    durs.append(d_ms / 1000.0)

# Last frame: hold for the median real delta so it doesn't flash by.
if use_emu and n >= 2:
    deltas = sorted(durs[:-1])
    durs[-1] = deltas[len(deltas)//2]

with open(listpath, "w") as out:
    for i, (idx, _ms) in enumerate(rows):
        png = os.path.join(cap_dir, f"frame{idx:06d}.png")
        if not os.path.exists(png):
            continue
        out.write("file '%s'\n" % png)
        out.write("duration %.6f\n" % durs[i])
    # concat demuxer quirk: repeat the last file so its duration is applied.
    last = os.path.join(cap_dir, f"frame{rows[-1][0]:06d}.png")
    out.write("file '%s'\n" % last)

total = sum(durs)
sys.stderr.write("[assemble] %d frames, timing=%s, total=%.2fs (emu_span=%.1fms)\n"
                 % (n, "emu_ms" if use_emu else f"fallback {fallback_fps}fps", total, emu_span))
PY

# VFR concat -> CFR H.264. Nearest-neighbor upscale preserves the pixel art;
# even dims required for yuv420p (256x224 * SCALE stays even for integer SCALE).
VF="scale=iw*${SCALE}:ih*${SCALE}:flags=neighbor,fps=${FPS},format=yuv420p"
ffmpeg -hide_banner -loglevel warning -y \
  -f concat -safe 0 -i "$LIST" \
  -vf "$VF" -fps_mode cfr -r "$FPS" \
  -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p \
  -movflags +faststart "$OUT"

echo "[assemble] wrote $OUT"
ffprobe -v error -select_streams v:0 \
  -show_entries stream=width,height,r_frame_rate,nb_read_frames,duration \
  -count_frames -of default=noprint_wrappers=1 "$OUT" || true
