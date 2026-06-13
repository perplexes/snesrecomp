# MSU-1 support (feat/msu-1)

MSU-1 is near's SNES streaming-audio + data coprocessor. It is two
independent channels exposed through registers `$2000-$2007`:

- **Audio** — 44.1 kHz signed-16 stereo PCM tracks (`<base>-<N>.pcm`),
  selected/played/looped via the registers and mixed on top of the
  S-DSP output.
- **Data** — a byte-addressable file (`<base>.msu`) read through a seek
  pointer + auto-incrementing read port. (Used by full enhancements;
  most music packs ship an empty/absent `.msu`.)

## What this branch implements (the hardware side)

The chip itself lives entirely in the shared runtime — one new file plus
three small hooks — so it works for **every** recompiled game at once:

| Piece | Location |
|---|---|
| Chip core (registers, `.pcm` loader, PCM ring/resample, `.msu` data channel) | `runner/src/snes/msu1.{c,h}` |
| Register dispatch (`$2000-$2007` → `msu1_read`/`msu1_write`) | `runner/src/common_rtl.c` `ReadReg`/`WriteReg` |
| Audio mix (after `dsp_getSamples`, under the APU lock) | `runner/src/common_rtl.c` `RtlRenderAudio` |
| Arm-from-env at startup (every game, no per-game wiring) | `runner/src/common_cpu_infra.c` `RtlRegisterGame` |
| Build source list | `runner/runner.cmake` |

Why these points: `is_hw_reg()` already classifies `$2000-$5FFF` as
hardware registers, so reads/writes to `$2000-$2007` already route to
`ReadReg`/`WriteReg` (they previously returned open-bus `0`). And
`RtlRenderAudio` is the single place final stereo samples are produced.

### Threading

`msu1_read`/`msu1_write` (CPU thread) take the existing APU lock
(`RtlApuLock`, a recursive SDL mutex) to serialise against the audio
thread. `msu1_mix` runs **inside** `RtlRenderAudio`'s already-held lock,
so it must not re-take it. No new dependency, no new mutex.

### Sync / resampling

Each `RtlRenderAudio` call is exactly 1/60 s of audio. MSU therefore
consumes `44100/60 = 735` source frames per call and linearly resamples
them to the output block size — staying locked to the same 60 Hz block
clock as the S-DSP and adapting to any host output rate automatically.

## Default-OFF guarantee

With `SNESRECOMP_MSU1` unset, `msu1_enabled()` is false: `ReadReg`
returns `0` and `WriteReg`/`RtlRenderAudio` no-op for the MSU range —
**byte-identical** to pre-branch behaviour. Verified that the disabled
path matches the prior fall-through exactly.

## Usage

```
# Pack base prefix: tracks resolve as <prefix>-<N>.pcm, data as <prefix>.msu
SNESRECOMP_MSU1=C:/msu/alttp      # loads C:/msu/alttp-1.pcm, alttp-2.pcm, ...

# Or enable and derive the base from the ROM name (needs a one-line
# msu1_set_rom_path(rom_path) call in the game's main.c — see below):
SNESRECOMP_MSU1=auto
```

`.pcm` format: 8-byte header = `"MSU1"` magic + uint32 little-endian loop
point (in stereo sample-frames), then raw int16-LE stereo @ 44100 Hz.

## Register map implemented

Read: `$2000` status (`d a r p` + revision), `$2001` data port
(auto-inc), `$2002-$2007` ID `"S-MSU1"`.
Write: `$2000-$2003` 32-bit data seek (commits on `$2003`), `$2004-$2005`
16-bit track select (commits + loads on `$2005`), `$2006` volume,
`$2007` control (bit0 play, bit1 repeat). A missing/invalid track sets
the status error bit so a driver can fall back to native SPC music.

## Remaining work (game side — NOT in this branch)

The chip is inert without a ROM that drives it. Vanilla SMW/ALttP have
no MSU-1 driver, so to actually hear it:

1. **Per-game MSVC build:** add `msu1.c` to each game's `src/<game>.vcxproj`
   source list (CMake auto-picks it up via `runner.cmake`; the `.vcxproj`
   lists sources explicitly — same step the shadow-audio feature needed).
2. **Driver:** recompile from an MSU-1-patched ROM (the complete/faithful
   path — e.g. the ALttP/SM combo-randomizer MSU asm, or the SMW
   MSU-1(+) hack). This is a different ROM → new pin + full re-validation.
   The patch's new asm must recompile cleanly.
3. *(Optional)* one line in each game's `main.c` after ROM resolution:
   `msu1_set_rom_path(rom_path_buf);` to support `SNESRECOMP_MSU1=auto`.

## Test status

Compile-validated (`gcc -Wall -Wextra`, via PowerShell — the Bash sandbox
suppresses compiler stderr/exit codes here): `msu1.c`,
`common_cpu_infra.c`, and `common_rtl.c` (with SDL + game config.h)
all build clean. Not yet exercised against a live pack — that needs a
game build + an MSU pack (game-side step above).
