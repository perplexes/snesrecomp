# snesrecomp

A static recompiler for SNES (Super Famicom) games. Translates 65816
machine code into native C ahead-of-time, so the recompiled game runs
as a normal binary rather than under interpretation.

> ## ⚠️ Heavily Work-In-Progress
>
> This project is **early, unstable, and not yet usable as a general
> tool**. Internal debugging notes and design docs reference
> incomplete passes, half-built infrastructure, and known
> regressions. Branches are noisy. APIs change without warning.
> Tests are not green on every branch. Expect things to be broken.
>
> Treat anything you see here as a snapshot of in-progress work, not
> a release.

## Status

snesrecomp's primary driving test case is:

- **Super Mario World** — recompiled via the companion repo
  [mstan/SuperMarioWorldRecomp](https://github.com/mstan/SuperMarioWorldRecomp).
  **Believed fully playable** as of 2026-05-16: hand-verified
  end-to-end through Worlds 1–2 (Yoshi's Island + Donut Plains,
  including the Iggy castle boss) and into World 3 (Vanilla Dome).
  Two always-on runtime tripwires (M/X claim verifier and async
  cpu->m_flag/x_flag-write detector) have not latched on the
  verified worlds. Worlds 4–7 + special content are not yet
  hand-verified but expected to play similarly.

The intent is for snesrecomp to be **game-agnostic** — adding a
second game (Zelda: A Link to the Past, Mega Man X, Super Metroid,
…) should cost mostly per-game `.cfg` work, not months of framework
patching. SMW exercised the framework hard during 2026-04/05 and
surfaced the bug classes the framework now handles permanently:
per-variant exit-(M, X) inference with order-independent fixpoint,
dispatch-terminator JSL recognition, PHP/PLP-bracketed M/X tracking,
wrapper-bypass autorouter, tail-call autorouter, and a full
runtime tripwire suite catalogued in
[`docs/TRIPWIRES.md`](docs/TRIPWIRES.md).

Game #2 work in progress: separate sibling repos for ALttP and
Mega Man X, each junctioning this snesrecomp checkout, are being
scaffolded.

## What's in this repo

- `recompiler/` — Python code that decodes 65816 ROM bytes,
  reconstructs control flow, and emits C.
- `runner/` — C runtime that the generated code links against (CPU
  state, memory mapping, debug server). Embeds a snes9x-derived
  oracle for differential testing.
- `tests/` — framework tests (decoder, CFG, SSA placement, etc.) and
  L3 fixtures.
- `fuzz/` — differential fuzzer that diffs synthetic 65816 snippets
  through the recompiler vs. an embedded snes9x.
- `tools/` — scripts for regen, oracle diffing, etc.

## Status of public API / docs

There isn't a public API. There aren't user-facing docs. Internal
docs assume context from active development sessions and will not
make sense without it. This will change once the framework
stabilizes.

## License

Not yet declared. Code in this repo is original; the snes9x core
under `runner/snes9x-core/` is upstream from
[libretro/snes9x](https://github.com/libretro/snes9x) and retains
its own license.
