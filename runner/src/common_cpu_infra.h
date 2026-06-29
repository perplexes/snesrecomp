#pragma once

#include "types.h"

#ifdef _MSC_VER
#pragma warning(disable: 4013 4028 4033 4090 4133 4305 4715 4716)
#endif

typedef struct Snes Snes;
typedef struct Cpu Cpu;

extern Snes *g_snes;
extern Cpu *g_snes_cpu;
extern bool g_fail;

Snes *SnesInit(const uint8 *data, int data_size);
uint8_t *SnesRomPtr(uint32 v);

// Apply the native-mode CPU state the real ROM's reset vector would
// have established (CLC;XCE / REP #$38 / TCD / TCS / SEI). The recomp
// path never executes those opcodes, so RtlReset and SnesInit invoke
// this after snes_reset to pick up where the ROM would be at $8028.
void SnesEnterNativeMode(void);

typedef void CpuInfraInitializeFunc(void);
typedef void RunOneFrameOfGameFunc(void);

void WatchdogCheck(void);
void WatchdogFrameStart(void);

/* Cycle-faithful clock: the recompiler emits cpu_cycle_tick(cpu, N) per block
 * with N = the block's estimated 65C816 master-cycle cost. We accumulate into
 * g_pending_cycles and WatchdogCheck() flushes the real total into sched_tick()
 * so NMI/V-IRQ/H-IRQ fire at the architecturally correct simulated beam
 * position — instead of a fixed heuristic cost per block tick. The `cpu` arg is
 * unused today (kept for ABI symmetry / future per-CPU clocks). */
struct CpuState;
extern uint64_t g_pending_cycles;
void cpu_cycle_tick(struct CpuState *cpu, uint32_t n);

/* Cooperative IRQ pump hook (game-agnostic). Some games spin-wait on RAM
 * flags that ONLY advance inside an interrupt handler (e.g. a vblank/raster
 * IRQ that DMAs a framebuffer and bumps a double-buffer flag). With no async
 * interrupt model, the recompiled CPU would spin forever. A game layer may
 * register a pump here: WatchdogCheck() calls it on each ~10k-iteration tick
 * (boot included) so the handler runs and the flag advances, letting the
 * spin fall through. Return nonzero if the pump made progress (resets the
 * 5s hang timer). Reentrancy is guarded by the framework. NULL = disabled. */
typedef int (*CoopIrqPumpFunc)(void);
extern CoopIrqPumpFunc g_coop_irq_pump;
/* Reentrancy guard: set to 1 while g_coop_irq_pump (or a scheduler-delivered
 * interrupt) is running. Prevents WatchdogCheck from recursively firing
 * another interrupt from within the recompiled interrupt handler.
 * Exposed so sched.c can read and set it alongside the pump. */
extern int g_in_coop_pump;

/* Host frame-present hook (game-agnostic). When a game's whole boot + main
 * loop runs synchronously inside I_RESET and never returns to the host frame
 * loop (so the SDL present + event pump never run), the cooperative pump can
 * call this each time it completes a displayed frame to render + present the
 * current PPU output and pump OS events, keeping the window live and pacing to
 * ~60 fps. The host (launcher) registers it after the renderer is up; headless
 * runs leave it NULL. Return nonzero to request shutdown (window closed). */
typedef int (*HostPresentFunc)(void);
extern HostPresentFunc g_host_present_hook;

/* GSU/coprocessor frame-done hook (game-agnostic). The engine runs the Super FX
 * to completion on an R15 launch (see common_rtl.c). A coprocessor game renders
 * into Game Pak RAM and then transfers the result to VRAM through a timing-
 * sensitive, often beam-raced path the static recomp can't reproduce. When a
 * launch completes, the engine calls this hook with the launch entry PC so a
 * game layer can HLE-present the finished framebuffer (copy coproc RAM -> VRAM,
 * set display state) WITHOUT the engine knowing any game specifics. entry_pc is
 * the coprocessor PC captured at launch, before the run mutated it. NULL = the
 * engine presents nothing (faithful path). */
typedef void (*GsuFrameDoneFunc)(uint16 entry_pc);
extern GsuFrameDoneFunc g_gsu_frame_done;

/* DMA channel suppression hook (game-agnostic). Returns nonzero if the channel
 * described by (bAdr = $21xx B-bus dest low byte, aBank = A-bus source bank)
 * should be MASKED OUT of an MDMAEN ($420B) trigger. A game layer that HLE-
 * presents a coprocessor framebuffer registers this to stop the game's own
 * now-redundant/broken transfer from overwriting the presented frame. The game
 * owns any "presenting yet" state. NULL = no suppression (all channels run). */
typedef int (*DmaSuppressFunc)(uint8 bAdr, uint8 aBank, uint16 aAdr);
extern DmaSuppressFunc g_dma_suppress;

void RecompStackPush(const char *name);
void RecompStackPop(void);
/* Per-frame 65816 entry-S tracking for return-to-ancestor RTS resolution
 * (see common_cpu_infra.c). The emitted function prologue records
 * _entry_s into g_cpu_entry_s[g_recomp_stack_top-1]. */
extern int g_recomp_stack_top;
extern uint16_t g_cpu_entry_s[];
int cpu_resolve_ancestor_skip(uint16_t ret_s);
void cpu_tailcall_inherit_return_context(uint16_t entry_s, uint8_t hrv);
int cpu_take_tailcall_return_context(uint16_t *entry_s, uint8_t *hrv);
#include <setjmp.h>
extern jmp_buf g_watchdog_jmp;
extern int g_watchdog_tripped;

typedef struct RtlGameInfo {
  const char *title;
  CpuInfraInitializeFunc *initialize;
  RunOneFrameOfGameFunc *run_frame;
  RunOneFrameOfGameFunc *draw_ppu_frame;
  // Filename prefix used by RtlSaveLoad, e.g. "save" produces
  // "saves/save%d.sav". If NULL, framework uses "%s_save" with title.
  const char *save_name_prefix;
} RtlGameInfo;

extern const RtlGameInfo *g_rtl_game_info;

// Called by the game-layer before SnesInit so the framework knows
// which game it's running. Framework itself names no specific game.
void RtlRegisterGame(const RtlGameInfo *info);
