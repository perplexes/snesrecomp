# snesrecomp-regen (Rust)

Rust port of the snesrecomp v2 static recompiler ("regen") pipeline — the thing
that turns a ROM + per-bank `.cfg` files into one generated C file per bank, a
`dispatch_v2.c`, and `recomp/funcs.h`. Replaces `tools/v2_regen.py` and the
`recompiler/v2/*` package.

**Why:** speed (the Python regen runs against a fixed 1.5 h watchdog; decode cache
is disabled) and a clean typed core (the Python pipeline threads ~15 process-global
mutable singletons). See `docs/` and the plan in the StarFoxRecomp repo.

**Fidelity target:** *functional* equivalence with the Python output — generated C
must compile and the game must boot/render/test the same — not byte-identity.

Full usage, the engineer workflow, and the contributor's equivalence gate are in
the project wiki: **`StarFoxRecomp/wiki/tools/regen-rs.md`** and
**`wiki/adapting-a-new-rom.md`**.

## Binaries (`target/release/`)

| Binary | Replaces | Use |
|---|---|---|
| `regen` | `tools/v2_regen.py` | `regen --rom game.sfc --cfg-dir recomp --out-dir src/gen [--jobs N] [--banks 00,07]` |
| `sync-funcs-h` | `tools/v2_sync_funcs_h.py` | `sync-funcs-h --cfg-dir recomp --out recomp/funcs.h` |
| `chase-misses` | `tools/strat_autoroute.py` | route a dispatch-miss profile into cfg `func` entries (`--apply` to write) |
| `dump-decode` / `dump-emit` / `dump-autoroute` | (oracles) | dump the decode / emit / autoroute result as JSON for the differential gate |

`SF_REGEN_TIMING=1 regen …` prints per-phase wall-clock + decode/emit cache hit rates.

## Layout

- `src/` — `rom`, `insn`, `ir`, `widths`, `cycles`, `cfg`, `decoder` (incl.
  `DecodeCache`), `cfgbuild`, `lowering`, `codegen` (`EmitCtx`/`EmitOutcome`),
  `emit` (incl. `EmitCache`), `autoroute`. Globals replaced by threaded
  `DecodeEnv`/`EmitCtx`; per-bank emit returns its outcome (rayon-safe).
- `scripts/{dump,diff}_{decode,emit,autoroute}.py` + `make-golden.sh` /
  `diff-golden.sh` — differential oracles vs the Python (dev aids).

## Build / test

```bash
cargo build --release && cargo test          # 41 unit tests
```

## Status — COMPLETE (output-equivalent + deterministic)

All phases done and verified against the live Python on a frozen cfg snapshot:
identical function set, byte-identical `dispatch_v2.c` / `unresolved_stubs_v2.c`
/ `funcs.h`, deterministic across runs. **Full Star Fox regen ~9 s vs ~160 s in
Python (≈17×).** Tracks the live Python directives (`force_host_return:<sites>`
+ clone inheritance ported).

- [x] Phase 0–5 — rom/insn/ir/widths/cycles/cfg, decoder+CFG, lowering/codegen/
  emit, autoroute, orchestrator + funcs.h.
- [x] Perf — exit-mx fixpoint `DecodeCache` (the ~95 %-of-wall-clock bottleneck)
  + per-function `EmitCache`.
- [x] `chase-misses` (Rust port of `strat_autoroute.py`).
- [ ] **Cutover** — the Python `v2_regen.py` is still the build default. Wiring
  `regen.sh --engine=rust` + flipping the default is deferred until the Python
  pipeline stabilises. See the wiki.

> **The [regen-cascade hazard] applies identically** — a full regen re-routes
> exit-M/X globally; re-test the working carves headless after any regen.
