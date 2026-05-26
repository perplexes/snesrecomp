# snesrecomp framework improvements

Backlog of toolchain / framework improvements that are NOT part of any
active in-flight task. Add new entries as they surface during work;
graduate to in-flight tasks only when explicitly scheduled.

---

## IN FLIGHT: Option-1 cpu->S model revival (2026-05-26) — de-risked & runtime-ready

Branch `feat/cpu-s-stack-model`. Reviving the "Abandoned path" below, now
de-risked. This is the complete fix for the JSR/JSL ↔ RTS/RTL ↔ PEI-trampoline
stack-model divergence that produced the MMX Dr Light freeze, Chill Penguin
softlock, AND the Launch-Octopus fish softlock (see MegamanXRecomp
`ISSUES.md` — all three are one root cause: `D56F`'s `PLP` reads a byte
clobbered by a callee touching the absent JSR return frame).

### Key de-risking finding: the RUNTIME is already done

The Option-2 "PEI-trampoline returns" work (commit `bf8a34b`) already landed
the entire runtime dispatch surface — so Option-1 is now a **codegen-only**
change:

- `runner/src/cpu_state.c:289-343`: `g_dispatch_table` (sorted by pc24),
  `_cpu_dispatch_lookup` (binary search + variant pick by runtime m/x +
  LoROM bank-mirror fallback), `cpu_dispatch_pc_from(cpu, pc24,
  entry_s_for_miss_restore, source_pc24)` (miss → return NORMAL → host C
  stack unwinds). `tools/v2_regen.py` already emits `<prefix>_dispatch_v2.c`
  and the vcxprojs already compile it. Nothing on the runtime/build side
  needs to change.

### Push design (constants from `op.source_pc24`)

Return addr = site + insn_len; pushed value = return − 1 (RTS/RTL add 1 on
pop, matching `_emit_return`'s existing pop arithmetic).

- **JSR** (3 bytes): push `(site+2)&0xFFFF` as 2 bytes (PCH then PCL).
- **JSL** (4 bytes): push PBR=`(site>>16)&0xFF`, then `(site+3)&0xFFFF` (PCH
  then PCL) — 3 bytes.
- Synthesized call with no `source_pc24` → push a correctly-SIZED sentinel
  (balanced callees pop+ignore; the rare trampoline dispatches to a
  lookup-miss → NORMAL → C unwind).

### Invoke-path audit — EVERY path must push, or a callee pops a frame nobody pushed (this is what broke the prior attempt)

1. `codegen._emit_call` (codegen.py:1421) — 4 shapes: pinned-JSL, pinned-JSR,
   regular-JSL (PB save/restore + m/x switch), regular-JSR (m/x switch). Push
   at the start of each; pop one frame in the SKIP-N propagation block.
2. `codegen._emit_dispatch` (codegen.py:1331) — the JSL-jump-table dispatcher
   is ITSELF a trampoline: it pops its own JSL return and the handler returns
   to the dispatcher's caller's-caller. Under Option-1 it must POP its own
   pushed frame, then invoke the handler (which pops the next frame). Care
   needed: `call_with_pb_save` does not push today.
3. The two JMP/JML indirect-dispatch emitters (codegen.py ~1172 / ~1278) are
   TAIL-calls — they must NOT push (callee inherits the caller's frame).
4. `emit_function.py` tail-calls past `end:` (sibling inheritance + cross-bank,
   incl. the NLR-aware `_tail_call_stmt`) — tail-calls, NO push; `_entry_s`
   must reflect the OUTER caller's pre-push S.

### Return path: generalize the existing trampoline branch to ALL returns

`codegen._emit_return` (codegen.py:1585) already has the 3-path shape for
`_TRAMPOLINE_RETURNS`-flagged sites (NLR `_ps` / `cpu->S != _entry_s` dispatch
via `cpu_dispatch_pc_from` / balanced). For Option-1: drop the `is_trampoline`
gate so EVERY RTS/RTL pops its frame — balanced (`cpu->S == _entry_s`) pops
`frame_size` and returns NORMAL (C stack unwinds); trampoline pops+dispatches.
The `_classify_trampoline_returns` intra-procedural detector becomes
unnecessary (cpu->S frames make trampolines work naturally).

### THE high-risk decision (validate by test, not reasoning): NLR ↔ cpu->S symmetry

`_pending_skip` (PLA*N NLR ABI) currently SKIPS the literal PLAs and signals
skip-count up the C stack. With real frames on `cpu->S` there are two ways:

- **(A) Keep `_pending_skip`** and make `_emit_return`'s NLR path + every
  `_emit_call` SKIP-N propagation level pop exactly one frame. This is what the
  prior attempt did — and the double-pop/under-pop symmetry is almost certainly
  what leaked bytes and killed controller input. Must get the per-level pop
  count EXACTLY right.
- **(B) Remove `_pending_skip` for NLR** and let the PLA*N execute as real
  `cpu->S` pops + RTS-as-`cpu_dispatch_pc` — NLR then "just works" through the
  stack. Cleaner / most-complete, bigger change.

Lean (B) (most-complete per the global rule), but decide empirically: implement,
regen MMX, repro the fish softlock, and read the boundary ring to confirm
`cpu->S` stays balanced frame-to-frame (D56F enters at a stable S, exits m=1).

### Validation order (per the cross-game rule)

1. MMX: fish softlock fixed AND controller input still works AND attract/boot
   still reaches Highway (the prior attempt's regression). Measure via boundary
   ring: D56F exits m=1; no per-frame cpu->S drift.
2. Only then: SMW + LttP full regen + build + smoke test (final cross-game
   guard).

---

## Abandoned path: full cpu->S model for JSR/JSL/RTS/RTL (2026-05-24)

**Status:** Rolled back. The narrower PEI-trampoline detector (ISSUES.md
"Option 2" — static decoder-side pattern flag) was chosen as the next
attempt for the Dr Light freeze.

**What was attempted:** Model JSR/JSL pushes faithfully on cpu->S
(JSR pre-push 2 bytes = pc+2; JSL pre-push 3 bytes = PB + pc+3) and
emit RTS/RTL with a three-path exit:

1. **NLR** (`_pending_skip != NORMAL`): pop 1 frame, return `_ps`.
2. **Trampoline** (`cpu->S != _entry_s`): pop the topmost frame from
   cpu->S, compute (PB:PC+1), tail-call a new runtime helper
   `cpu_dispatch_pc(cpu, pc24)`. The helper binary-searches a
   per-game dispatch table (`<prefix>_dispatch_v2.c`) emitted by
   `v2_regen.py` after the autopromote loop. Lookup miss → return
   NORMAL → host C stack unwinds.
3. **Balanced**: pop the JSR/JSL pre-pushed frame, return NORMAL.

Symmetric pop in `_emit_call`'s SKIP-N propagation block (each
propagating level pops one frame).

**What worked:** The original Dr Light BCS-self-spin freeze (NMI
walker spinning at `$00:BA48` because `bank_04_9A02`'s PEI-trampoline
RTL leaked 6 bytes onto cpu->S, eventually corrupting the DMA queue
tail at `$00:00A5/A6`) was fixed. After the fix the same TCP-driven
repro (boot → loadstate 0 → `p1=right` 4s) no longer produces the
spin signature; call stack stays in normal cooperative-task yield.

**What broke:** X did not respond to controller input at all (frames
advanced, scheduler ticked, but X never moved — neither right nor
jump). Same behaviour on a fresh boot (no loadstate). Diagnosis (not
fully confirmed via boundary audit because the regression was caught
before that scope): the cpu->S push/pop model is symmetric for direct
JSR/JSL → RTS/RTL pairs, but several *other* emit paths invoke
recompiled functions WITHOUT going through `_emit_call`, so they
push 0 but the callee's `_emit_return` pops 2 or 3 bytes. Each such
mismatch leaks bytes on cpu->S; at hundreds of calls per frame the
scheduler's state corrupts quickly.

**Specific non-`_emit_call` entry paths that broke:**

1. `codegen._emit_dispatch` (ExecutePtr-style synthesized switch over
   a dispatch table). Each `case i:` invokes the handler via
   `emitter_helpers.call_with_pb_save`, which does NOT pre-push.
   Handler's balanced RTL pops 3 bytes from cpu->S that belong to a
   different frame.
2. `emit_function.py` tail-call-past-`end:` blocks (sibling-function
   inheritance and cross-bank tail-calls). `{ RecompReturn _tc =
   sibling(cpu); RecompStackPop(); return _tc; }` — no pre-push,
   tail-callee pops anyway.
3. JSR-caller → RTL-callee or JSL-caller → RTS-callee frame-size
   mismatch through indirect dispatch. Caller pushed 2/3, callee
   pops 3/2 → off by 1 per dispatch.

**What would need to happen to revive this approach:**

- Audit every recompiled-function entry point and ensure it goes
  through a uniform "pre-push" surface (either `_emit_call` or an
  equivalent that knows the frame size).
- Track the calling site's frame size (JSR=2, JSL=3) on every
  synthesized invocation so the callee's RTS/RTL pop matches.
- Resolve the tail-call / cross-fn-inheritance shape:
  the tail-callee's `_entry_s` must reflect the same cpu->S state
  that the original outer caller pre-pushed for, not the
  intermediate state at the tail-call site.
- Decide what to do about M/X variants that exit with a different
  return type than they entered (RTL where the caller JSR'd, etc.) —
  these are likely real ROM bugs in the asm but our model has to
  cope without crashing the host C call chain.

**Reverted scope (rollback boundary):**

- `recompiler/v2/codegen.py`: `_emit_jsr_jsl_push`,
  `_emit_skip_propagation_pop`, `_JSR_FRAME_BYTES`,
  `_JSL_FRAME_BYTES`, modifications to `_emit_call`'s four paths
  (force_variant_at JSL/JSR + regular JSL/JSR), the three-path
  rewrite of `_emit_return`.
- `recompiler/v2/emit_function.py`: `_entry_s` prologue line, HLE
  wrapper frame-pop.
- `runner/src/cpu_state.h`: `DispatchEntry` typedef,
  `cpu_dispatch_pc` declaration, `g_dispatch_table` extern, the
  full rewrite of the "Non-local return signaling" comment.
- `runner/src/cpu_state.c`: `cpu_dispatch_pc` body +
  `_cpu_dispatch_lookup` helper.
- `runner/src/cpu_trace.h`: `BD_EXIT_KIND_TRAMPOLINE` enum value.
- `tools/v2_regen.py`: the per-game `<prefix>_dispatch_v2.c` emit
  block (forward decls + sorted dispatch table + sentinel for
  empty-cfg case).
- Per-game vcxproj entries adding `gen\<prefix>_dispatch_v2.c`
  ClCompile (MMX, SMW, LttP).
- Each game's `src/gen/<prefix>_dispatch_v2.c` (delete; regen
  with reverted code won't emit it).

**Preserved (NOT reverted — these landed earlier in this session
chain, BEFORE Option 1):**

- `force_variant_at` cfg directive framework (cfg_loader,
  codegen, lowering, v2_regen Call.source_pc24 plumbing).
- snes9x-oracle disabled-by-game guardrail (cpu_state /
  emu_oracle_cmds / snes_oracle_backend) + the per-game MMX
  opt-out (config.h/.c, main.c, mmx.ini).

---

## Regen iteration time

Today (2026-05-24): a full MMX regen takes ~25 min. The pipeline
spends almost all of that in Python (`v2_regen.py`) — variant
discovery + iterative auto-promote + emit per bank. The MSVC Oracle
build on top of regen output is the smaller slice (~5 min from
clean). For sessions that iterate on `recompiler/v2/*.py` (codegen,
decoder, lowering, emit_function), the 25-min regen is the dominant
cost per loop. Two improvement lanes:

### Priority 1 — Parallel regen across banks (HIGH PRIORITY)

The 8 MMX banks (similar surface in SMW, LttP) are mostly independent
within a single auto-promote pass: each bank's `emit_bank` call
decodes + lowers + emits its own ROM range, producing a per-bank C
file plus a set of cross-bank Call demands. Coordination between
banks is only required at the pass boundary (to merge demand sets
and decide which entries auto-promote into which cfg).

Concretely: within `v2_regen.py`'s emit loop, replace the serial
`for bank, ... in parsed:` over `emit_bank(...)` with a worker pool
(`concurrent.futures.ProcessPoolExecutor`, or `multiprocessing.Pool`
sized to `cpu_count` minus 1). Each worker emits one bank's .c file
and returns the build report (suppressions, demands, const-Z folds,
etc.). Main process aggregates after all banks finish, then runs the
auto-promote merge serially before the next pass.

Expected impact: 2-4x speedup on 8-bank MMX. SMW (10 banks) and
LttP (~20 banks) scale similarly with core count.

Caveats to nail down during implementation:
- `set_name_resolver` is a module-level global in codegen.py; each
  worker process gets its own copy via fork/spawn — fine for
  process-pool, NOT thread-pool.
- `_NAME_RESOLVER` / `_UNRESOLVED_CALL_TARGETS` / etc. are
  process-local. Workers must serialize their final state back so
  the main process can union them.
- The auto-promote pass between iterations is fundamentally serial
  (merges all banks' demands), so the speedup applies to the emit
  phase only, not to the full convergence loop. Still substantial
  since emit dominates per-pass cost.

This is the higher-leverage win because it helps every workflow —
codegen.py iteration, cfg iteration, ROM-change iteration alike.

### Priority 2 — Incremental regen via bank-level dependency cache (DISCOVERY SPIKE)

Possible but the wins concentrate on a workflow that isn't the
current pain point. Approach:

- The variant-discovery pass at the top of `v2_regen.py` walks the
  ROM cheaply (no emit, no auto-promote) and produces the full
  `variants: dict[pc24 -> set[(m,x)]]` map. Partition by source bank
  to get an explicit "bank A demands (target, m, x) tuples from
  bank B" graph.
- Cache key per bank: SHA of (cfg file + relevant ROM byte range +
  cross-bank demand set + codegen.py SHA + framework version).
- Cache hit → reuse the existing emitted .c file. Cache miss → re-emit.

When this pays off:
- **cfg edit on a single bank**: typically a 1-bank rebuild. **Big
  win**, frequent workflow when humans add `name` / `exit_mx` /
  `indirect_call_table` directives.

When this does NOT pay off:
- **codegen.py edit**: framework SHA changes → every bank's cache
  invalidates → full rebuild. **No win.** This is the workflow we
  were in during the PEI-trampoline fix.
- **ROM change**: rare for static recomp.

Status: not scheduled. Worth a spike to validate the dep-graph
extraction is precise (i.e. there are no implicit cross-bank deps
the discovery pass misses). If the spike says yes, prioritize on
the next cfg-iteration-heavy session. If it says no — drop and
revisit only with new evidence.

---

## Other ideas, low priority

- Split each bank's C file into smaller TUs (one per `func` block,
  say) so MSVC `//m` parallelizes harder. Would help Oracle build
  time even without parallel regen.
- Emit LLVM IR directly + invoke `clang` instead of MSVC. Skips the
  C parser entirely. Architecturally interesting; weeks of work.

---

## Incremental re-emit per pass — gated by a callers index (2026-05-25)

**Status:** Explored and rolled back. The naive form (skip banks
whose `cfg.entries` did not grow; force all banks dirty when
`callee_exit_mx` changes) is a no-op for MMX because the auto-
promote refresh adds new `exit_mx` routes EVERY pass, so the
"force all dirty" branch always fires. Measured 298s vs A1-only
297s on MMX1 (no skips observed in 8 passes).

**What's needed for the complete form:** a per-pass callers-index,
keyed by callee `pc24` → set of caller-bank IDs. Updated when
`emit_bank` processes a bank's decoded graphs (drain every JSR/JSL
target into the index). Between passes:

1. Diff `callee_exit_mx` against the prior pass to compute the set
   of changed callee `pc24`s (new keys + changed values + removed).
2. Use the callers-index to map changed callees → set of caller
   banks whose decode would now see different post-JSR state.
3. Union with banks whose own `cfg.entries` grew during this pass's
   auto-promote.
4. The union is the dirty set for the next pass.

**Estimated win for MMX:** ~30% reduction on passes 4–7 of the
auto-promote loop, since most banks ARE clean once the
`callee_exit_mx` churn is filtered to its actual blast radius.
Passes 0–3 already small. Translates to ~100s saved on top of A1.

**Why not now:** the callers-index adds real surface area (build
during emit, drain via worker return for parallel A2, maintain
across the auto-promote refresh). For the current target (Chill
Penguin debug iteration), A1's 5× and A2's projected additional
2–4× are the bigger wins; this can wait.

**When to revisit:** if the iteration cadence after A1+A2 still
feels slow (>3 min total cycle), or when a debug session has many
cfg-only edits (those don't change `callee_exit_mx` at all, so the
naive A3 might suddenly start saving — but at that point the
disk-cache idea in §"Per-bank cache hashed on inputs" is even
stronger because it skips the decode entirely).
