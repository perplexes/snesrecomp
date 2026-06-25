/*
 * cpu_state.c — implementations for the v2 runtime CpuState.
 *
 * Address routing for the byte/word memory helpers:
 *   $00-$3F:0000-$1FFF / $7E:0000-$1FFF       -> g_ram (low WRAM mirror)
 *   $7E:0000-$FFFF                            -> g_ram[0x00000-0x0FFFF]
 *   $7F:0000-$FFFF                            -> g_ram[0x10000-0x1FFFF]
 *   $00-$3F:2000-$5FFF / $80-$BF:2000-$5FFF   -> SNES hardware regs
 *                                                (PPU, APU, joypad, DMA)
 *                                                routed via WriteReg/ReadReg
 *   $70-$7D:0000-$7FFF / $F0-$FD:0000-$7FFF   -> LoROM battery SRAM
 *                                                (cart->ram via g_sram)
 *   $00-$3F:6000-$7FFF / $80-$BF:6000-$7FFF   -> HiROM battery SRAM
 *                                                (cart->ram via g_sram)
 *   $00-$7D:8000-$FFFF / $80-$FF:8000-$FFFF   -> ROM (reads via RomPtr;
 *                                                writes are NOPs)
 *
 * The hardware-register routing is what unblocks boot: every PPU/APU/DMA
 * register write the recompiled code emits goes through WriteReg, so
 * INIDISP / NMITIMEN / OBSEL / DMA setup actually take effect. Without
 * it, $2100 stays at the snes9x default (forced-blank ON) and the
 * screen never lights up.
 *
 * The SRAM routing is what unblocks save/menu: every read against the
 * cart's battery RAM (SMW's VerifySaveFile, save data writes, password
 * tables, etc.) goes through g_sram so save data lives in cart->ram
 * instead of tripping RomPtr-invalid.
 */

#include "cpu_state.h"
#include "common_rtl.h"
#include "cpu_trace.h"

CpuState g_cpu;

/* WRAM read-pins (game-registered). When a pin is active, cpu_read8 returns the
 * pinned value for that g_ram offset instead of the backing byte. A generic
 * primitive: the engine holds no game addresses; a game registers offsets it
 * needs held (e.g. to HLE a flag a host-driven boot/carve can't establish). The
 * g_wram_pin_any flag keeps the read hot path free when nothing is pinned. */
WramReadPin g_wram_pins[CPU_WRAM_PIN_MAX];
uint8_t     g_wram_pin_any = 0;

void cpu_pin_wram_read(uint32_t off, uint8 val) {
    for (int i = 0; i < CPU_WRAM_PIN_MAX; i++) {
        if (g_wram_pins[i].active && g_wram_pins[i].off == off) {
            g_wram_pins[i].val = val; return;  /* update existing */
        }
    }
    for (int i = 0; i < CPU_WRAM_PIN_MAX; i++) {
        if (!g_wram_pins[i].active) {
            g_wram_pins[i].off = off; g_wram_pins[i].val = val;
            g_wram_pins[i].active = 1; g_wram_pin_any = 1; return;
        }
    }
}

void cpu_unpin_wram_read(uint32_t off) {
    int any = 0;
    for (int i = 0; i < CPU_WRAM_PIN_MAX; i++) {
        if (g_wram_pins[i].active && g_wram_pins[i].off == off) g_wram_pins[i].active = 0;
        if (g_wram_pins[i].active) any = 1;
    }
    g_wram_pin_any = (uint8_t)any;
}

/* Map a 24-bit logical address onto a g_ram offset. Returns -1 for
 * addresses that are NOT WRAM — the caller routes those to the HW-reg
 * helpers (WriteReg/ReadReg) or to ROM. */
static int sf_db_drift_bank(uint8 bank);  /* fwd: Super-FX DB-drift safety net */

static int cpu_ram_offset(uint8 bank, uint16 addr) {
    if (bank == 0x7E) return (int)addr;
    if (bank == 0x7F) return 0x10000 + (int)addr;
    if (addr < 0x2000 && (bank <= 0x3F || (bank >= 0x80 && bank <= 0xBF))) {
        return (int)addr;
    }
    /* Super-FX DB-drift safety net — see sf_db_drift_bank(). A drifted-DB read
     * of low WRAM ($0000-$1FFF, e.g. transbmp1 $1939) through a pure-ROM bank
     * resolves to WRAM instead of ROM, so transfer_l observes the IRQ-set
     * double-buffer flag and boot advances past the frame-sync spin. */
    if (addr < 0x2000 && sf_db_drift_bank(bank)) {
        return (int)addr;
    }
    return -1;
}

/* Super-FX DB-drift safety net (Star Fox bring-up). A pre-existing stack/DB-
 * tracking drift in the recompiled boot+init path leaves the data bank at a
 * ROM-region bank (observed $E7 at transfer_l, $58 at WAITDMA_L) when the game
 * reads low memory through DB. Such an access — low WRAM ($0000-$1FFF) or the
 * hardware-register window ($2000-$5FFF) — is never legitimate through a
 * pure-ROM bank: on hardware those banks expose neither WRAM mirror nor the
 * register window, so the only way the game reaches here is the drift. Routing
 * it to the address it *meant* (WRAM / hw regs, as if DB were a low bank)
 * unblocks the frame-sync and raster-wait spins. Scoped to Super-FX carts and
 * to the pure-ROM banks $40-$6F / $C0-$EF, excluding the SRAM/GSU-RAM banks
 * ($70-$7D / $F0-$FD) which legitimately decode low addresses.
 * TODO(starfox): remove once the boot-path DB-tracking drift is fixed at the
 * recompiler level. See starfox-boot-progress. */
static int sf_db_drift_bank(uint8 bank) {
    return g_gsu_full_ram
        && ((bank >= 0x40 && bank <= 0x6F) || (bank >= 0xC0 && bank <= 0xEF));
}

/* True when (bank, addr) addresses an SNES hardware register that should
 * be routed through the framework's WriteReg/ReadReg dispatch. The HW
 * register window is $2000-$5FFF in low banks ($00-$3F, $80-$BF) — plus, for
 * Super-FX carts, drifted-DB pure-ROM banks (see sf_db_drift_bank). */
static int is_hw_reg(uint8 bank, uint16 addr) {
    if (addr < 0x2000 || addr >= 0x6000) return 0;
    if (bank <= 0x3F) return 1;
    if (bank >= 0x80 && bank <= 0xBF) return 1;
    if (sf_db_drift_bank(bank)) return 1;
    return 0;
}

/* Map a 24-bit logical address onto a g_sram offset for cart battery
 * RAM. Returns -1 if (bank, addr) is NOT SRAM. Mirrors snes9x's
 * cart_readLorom and cart_readHirom SRAM mappings so save-data
 * accesses route to cart->ram instead of falling through to RomPtr
 * (which would trip the RomPtr-invalid off-rails detector). */
static int cpu_sram_offset(uint8 bank, uint16 addr) {
    if (g_sram_size == 0 || g_sram == NULL) return -1;
    /* Super FX Game Pak RAM: banks $70-$71 (and mirrors $F0-$F1) map the
     * FULL bank ($0000-$FFFF), not just $0000-$7FFF — Star Fox's GSU
     * framebuffer lives at $70:AC00 (above $8000). Shared with the GSU
     * core (gsu_set_memory gets this same buffer). Guarded so non-Super-FX
     * games keep the stock LoROM/HiROM SRAM windows below. */
    if (g_gsu_full_ram && ((bank & 0x7E) == 0x70)) {
        return (int)((((bank & 0x1) << 16) | addr) & (g_sram_size - 1));
    }
    /* LoROM SRAM: banks $70-$7D + $F0-$FD, addr $0000-$7FFF. */
    if (((bank >= 0x70 && bank < 0x7E) || (bank >= 0xF0 && bank < 0xFE))
        && addr < 0x8000) {
        return (int)((((bank & 0xF) << 15) | addr) & (g_sram_size - 1));
    }
    /* HiROM SRAM: banks $00-$3F + $80-$BF, addr $6000-$7FFF. */
    if ((bank < 0x40 || (bank >= 0x80 && bank < 0xC0))
        && addr >= 0x6000 && addr < 0x8000) {
        return (int)((((bank & 0x3F) << 13) | (addr & 0x1FFF))
                     & (g_sram_size - 1));
    }
    return -1;
}

/* APU pacing: every HW-register touch advances the main-CPU cycle
 * estimate. v1 did this in `debug_on_block_enter` (RDB_BLOCK_HOOK); v2
 * doesn't emit those, so without this bump g_main_cpu_cycles_estimate
 * stays at 0, snes_catchupApu never advances the SPC, and SMW's
 * "wait for $2140 == $BBAA" poll loop spins forever waiting for a
 * response that the APU can't produce.
 *
 * Per-touch granularity is overshooting reality (real CPU does ~6
 * cycles per insn, far less than 24 per touch) but the SPC handshake
 * doesn't care about precise timing — it just needs *some* cycles to
 * elapse so the IPL ROM runs to the point of writing $BBAA. */
#include <stdio.h>
/* APU pacing: every HW-register touch advances the main-CPU cycle
 * estimate. v1 did this in `debug_on_block_enter`; v2 doesn't emit
 * those, so without this bump the SPC never advances and SMW's
 * "wait for $2140 == $BBAA" handshake spins forever.
 *
 * The 256-cycle increment is tuned to roughly match v1's per-block
 * pacing amortised over the recomp's tight CPU read loops. The
 * minimum-cycle floor in snes_catchupApu (snes.c) ensures the SPC
 * actually progresses on each call.
 *
 * Only APU-port touches ($2140-$217F) feed the APU catch-up counter
 * (issue #4): general HW touches massively over-count during load-heavy
 * phases ($2118 decompression spam) and used to convert into SPC-cycle
 * bursts that overflowed the DSP output ring at scene transitions,
 * audibly skipping the music. Handshake loops poll the ports themselves,
 * so APU touches alone keep every upload/ack protocol self-pacing. The
 * all-touch estimate stays for trace timestamps and diagnostics. */
static inline void cpu_pace_cycles(uint16 addr) {
    g_main_cpu_cycles_estimate += 256;
    if (addr >= 0x2140 && addr <= 0x217F)
        g_apu_pace_cycles_estimate += 256;
}

/* Optional debug — disabled in release. Set BUILD_CPU_HW_LOG=1 in the
 * build to enable verbose per-touch logging. */
#define BUILD_CPU_HW_LOG 0
static uint64_t s_hw_touch_count = 0;
static uint16 s_last_hw_addr = 0;
static int s_last_hw_was_read = 0;
static int s_apu_writes_logged = 0;

/* Last generated function entered — set on every entry so a hang can be
 * attributed to the function the CPU is spinning in (SF bring-up). */
const char *g_last_funcname = "(none)";
/* Logger reachable from generated code. Disabled at release. */
void cpu_dbg_funcname(const char *name) {
    g_last_funcname = name;
    (void)name;
#if BUILD_CPU_HW_LOG
    static int n = 0;
    if (n++ < 50) {
        fprintf(stderr, "[func#%d] %s (touch=%llu)\n",
                n, name, (unsigned long long)s_hw_touch_count);
        fflush(stderr);
    }
#endif
}
static void cpu_hw_log(uint16 addr, int is_read, uint16 val) {
    s_last_hw_addr = addr;
    s_last_hw_was_read = is_read;
    if (!is_read && addr >= 0x2140 && addr <= 0x2143) {
        s_apu_writes_logged++;
    }
    s_hw_touch_count++;
#if BUILD_CPU_HW_LOG
    (void)val;
    if (s_hw_touch_count % 1000000 == 0) {
        fprintf(stderr, "[hw-pace] touches=%llu\n", (unsigned long long)s_hw_touch_count);
        fflush(stderr);
    }
#else
    (void)val;
#endif
}

uint8 cpu_read8(CpuState *cpu, uint8 bank, uint16 addr) {
    if ((addr == 0x4218 || addr == 0x4219) && 0) {
        static int n = 0;
        if (n++ < 20) fprintf(stderr, "[joyrd8] addr=%04x DB=%02x ramoff=%d hwreg=%d\n",
                              addr, bank, cpu_ram_offset(bank, addr), is_hw_reg(bank, addr));
    }
    int off = cpu_ram_offset(bank, addr);
    if (off >= 0) {
        /* WRAM read-pins: a game may pin specific WRAM offsets to a fixed value
         * (see cpu_pin_wram_read). The g_wram_pin_any flag keeps this branch
         * free when no pins are armed. Used to HLE state a host-driven boot/carve
         * can't otherwise establish (e.g. a fade-complete flag). Engine carries
         * no game addresses — the game registers them. */
        if (g_wram_pin_any) {
            for (int i = 0; i < CPU_WRAM_PIN_MAX; i++)
                if (g_wram_pins[i].active && g_wram_pins[i].off == (uint32_t)off)
                    return g_wram_pins[i].val;
        }
        return cpu->ram[off];
    }
    if (is_hw_reg(bank, addr)) { cpu_pace_cycles(addr); cpu_hw_log(addr, 1, 0); return ReadReg(addr); }
    int sram = cpu_sram_offset(bank, addr);
    if (sram >= 0) return g_sram[sram];
    /* ROM read. RomPtr requires the global g_rom pointer to be live. */
    return *RomPtr(((uint32)bank << 16) | addr);
}

uint16 cpu_read16(CpuState *cpu, uint8 bank, uint16 addr) {
    if ((addr == 0x4218 || addr == 0x4219) && 0) {
        static int n = 0;
        if (n++ < 20) fprintf(stderr, "[joyrd16] addr=%04x DB=%02x ramoff=%d hwreg=%d\n",
                              addr, bank, cpu_ram_offset(bank, addr), is_hw_reg(bank, addr));
    }
    int off = cpu_ram_offset(bank, addr);
    if (off >= 0 && off + 1 < 0x20000)
        return (uint16)cpu->ram[off] | ((uint16)cpu->ram[off + 1] << 8);
    if (is_hw_reg(bank, addr)) { cpu_pace_cycles(addr); cpu_hw_log(addr, 1, 0); return ReadRegWord(addr); }
    int sram_lo = cpu_sram_offset(bank, addr);
    if (sram_lo >= 0) {
        /* Compose word from two byte fetches. If the high byte crosses
         * out of SRAM (e.g. word read at $70:$7FFF), fall through to
         * cpu_read8 for that byte so the boundary is handled by the
         * same routing logic. */
        int sram_hi = cpu_sram_offset(bank, (uint16)(addr + 1));
        uint8 hi = (sram_hi >= 0)
            ? g_sram[sram_hi]
            : cpu_read8(cpu, bank, (uint16)(addr + 1));
        return (uint16)g_sram[sram_lo] | ((uint16)hi << 8);
    }
    /* ROM word read. */
    const uint8 *p = RomPtr(((uint32)bank << 16) | addr);
    return (uint16)p[0] | ((uint16)p[1] << 8);
}

void cpu_write8(CpuState *cpu, uint8 bank, uint16 addr, uint8 v) {
    if ((addr == 0x1290 || addr == 0x1292) && v && getenv("SF_DBG_JOYRD")) {
        static int n = 0;
        if (n++ < 60) fprintf(stderr, "[cont0wr] addr=%04x DB=%02x val=%02x\n", addr, bank, v);
    }
    if ((addr == 0x12c3 || addr == 0x12c4) && getenv("SF_DBG_PLAYPT")) {
        static int n = 0;
        if (n++ < 80) fprintf(stderr, "[playpt8] addr=%04x DB=%02x val=%02x\n", addr, bank, v);
    }
    int off = cpu_ram_offset(bank, addr);
    if (off >= 0) {
        uint8 old = cpu->ram[off];
        cpu->ram[off] = v;
        cpu_trace_wram_write_check(cpu, bank, addr, off,
                                   (uint16)old, (uint16)v, 1);
        /* Also route through the dedicated 1M-entry WRAM-only ring so
         * writes survive when the main cpu_trace ring gets buried by
         * unrelated events (e.g. a BCS-self-loop block firing millions
         * of BLOCK events). IndirWriteByte/Word (common_rtl.h) already
         * does this for indirect stores; mirror it here for the
         * cpu_write8/16 path. */
#if SNESRECOMP_REVERSE_DEBUG
        extern void debug_on_wram_write_byte(uint32_t, uint8_t, uint8_t);
        debug_on_wram_write_byte((uint32_t)off, old, v);
#endif
        return;
    }
    if (is_hw_reg(bank, addr)) { cpu_pace_cycles(addr); cpu_hw_log(addr, 0, v); WriteReg(addr, v); return; }
    int sram = cpu_sram_offset(bank, addr);
    if (sram >= 0) { g_sram[sram] = v; return; }
    /* ROM / unmapped write: drop. */
}

void cpu_write16(CpuState *cpu, uint8 bank, uint16 addr, uint16 v) {
    if (addr == 0x12c3 && getenv("SF_DBG_PLAYPT")) {
        static int n = 0;
        if (n++ < 80) fprintf(stderr, "[playpt16] addr=%04x DB=%02x val=%04x\n", addr, bank, v);
    }
    int off = cpu_ram_offset(bank, addr);
    if (off >= 0 && off + 1 < 0x20000) {
        uint16 old = (uint16)cpu->ram[off]
                   | ((uint16)cpu->ram[off + 1] << 8);
        cpu->ram[off]     = (uint8)(v & 0xFF);
        cpu->ram[off + 1] = (uint8)(v >> 8);
        cpu_trace_wram_write_check(cpu, bank, addr, off, old, v, 2);
#if SNESRECOMP_REVERSE_DEBUG
        extern void debug_on_wram_write_word(uint32_t, uint16_t, uint16_t);
        debug_on_wram_write_word((uint32_t)off, old, v);
#endif
        return;
    }
    if (is_hw_reg(bank, addr)) { cpu_pace_cycles(addr); cpu_hw_log(addr, 0, v); WriteRegWord(addr, v); return; }
    int sram_lo = cpu_sram_offset(bank, addr);
    if (sram_lo >= 0) {
        g_sram[sram_lo] = (uint8)(v & 0xFF);
        int sram_hi = cpu_sram_offset(bank, (uint16)(addr + 1));
        if (sram_hi >= 0) g_sram[sram_hi] = (uint8)(v >> 8);
        else cpu_write8(cpu, bank, (uint16)(addr + 1), (uint8)(v >> 8));
        return;
    }
    /* ROM / unmapped write: drop. */
}

/* ── PEI-trampoline dispatch helper (2026-05-24, narrow detector) ──────
 *
 * Called from _emit_return on trampoline-flagged Returns when the
 * runtime balance check (cpu->S != _entry_s) fires. The caller has
 * already popped the topmost frame from cpu->S and computed
 * (PB, PC+1) as `pc24`. We binary-search g_dispatch_table for an
 * entry matching `pc24` and, if found, call the variant fnptr for
 * the runtime (m, x) flags.
 *
 * Not-found case: pc24 doesn't correspond to a known function entry.
 * Returning NORMAL lets the host C call stack unwind back through the
 * chain of `return cpu_dispatch_pc(...)` tail calls to the original
 * site, which then resumes naturally.
 */

/* Diagnostic ring for dispatch events — instrumentation added during
 * MMX Dr Light "sprite vanish" diagnosis (2026-05-24). Each entry
 * records (pc24, mx_idx, found, frame) for one cpu_dispatch_pc call.
 * Always-on (small fixed allocation, no perf concern). TCP cmd
 * `dispatch_log_get` dumps the ring. */
typedef struct DispatchLogEntry {
    uint32_t pc24;
    uint32_t source_pc24;
    const char *func_name;
    uint8_t  mx_idx;     /* (m<<1)|x */
    uint8_t  found;      /* 1 if entry found in table, 0 if miss */
    uint8_t  mirror;     /* 1 if found only via LoROM bank-mirror lookup */
    uint8_t  pad;
    uint32_t frame;
} DispatchLogEntry;

#define DISPATCH_LOG_CAP 1024
static DispatchLogEntry g_dispatch_log[DISPATCH_LOG_CAP];
static unsigned g_dispatch_log_idx;  /* monotonic; modulo via CAP for storage */

extern int snes_frame_counter;  /* common_rtl.c — game frame number */
extern const char *g_last_recomp_func;

static void _dispatch_log_record(uint32 pc24, uint32 source_pc24,
                                 unsigned mx_idx,
                                 int found, int via_mirror) {
    unsigned slot = g_dispatch_log_idx % DISPATCH_LOG_CAP;
    g_dispatch_log[slot].pc24 = pc24;
    g_dispatch_log[slot].source_pc24 = source_pc24;
    g_dispatch_log[slot].func_name = g_last_recomp_func;
    g_dispatch_log[slot].mx_idx = (uint8_t)mx_idx;
    g_dispatch_log[slot].found = (uint8_t)(found ? 1 : 0);
    g_dispatch_log[slot].mirror = (uint8_t)(via_mirror ? 1 : 0);
    g_dispatch_log[slot].pad = 0;
    g_dispatch_log[slot].frame = (uint32_t)snes_frame_counter;
    g_dispatch_log_idx++;
}

unsigned cpu_dispatch_log_count(void) {
    return g_dispatch_log_idx;
}

const DispatchLogEntry *cpu_dispatch_log_at(unsigned i) {
    if (i >= g_dispatch_log_idx) return NULL;
    if (g_dispatch_log_idx > DISPATCH_LOG_CAP &&
        i < g_dispatch_log_idx - DISPATCH_LOG_CAP) return NULL;
    return &g_dispatch_log[i % DISPATCH_LOG_CAP];
}

/* --- Dispatch-miss profiler (SF_DUMP_MISSES) ----------------------------
 * Accumulates the unique set of (target, source, mx) dispatch misses and
 * writes them, grouped by source, to the named file at exit. Used to
 * enumerate coroutine resume PCs that aren't yet registered func entries. */
#define DISPATCH_MISS_CAP 8192
typedef struct { uint32_t pc24, source_pc24; uint8_t mx_idx; } DispatchMiss;
static DispatchMiss g_dispatch_miss[DISPATCH_MISS_CAP];
static unsigned g_dispatch_miss_count;
static int g_dispatch_miss_atexit_armed;

static void _dispatch_miss_dump(void) {
    const char *path = getenv("SF_DUMP_MISSES");
    if (!path) return;
    FILE *f = fopen(path, "w");
    if (!f) return;
    fprintf(f, "# unique dispatch misses (target <- source) — resume-PC worklist\n");
    fprintf(f, "# count=%u\n", g_dispatch_miss_count);
    /* simple grouping: sort by source then target via insertion print */
    for (unsigned i = 0; i < g_dispatch_miss_count; i++) {
        fprintf(f, "%06X mx=%u from=%06X\n",
                g_dispatch_miss[i].pc24, g_dispatch_miss[i].mx_idx,
                g_dispatch_miss[i].source_pc24);
    }
    fclose(f);
}

void cpu_record_dispatch_miss(uint32_t pc24, uint32_t source_pc24, unsigned mx_idx) {
    if (!g_dispatch_miss_atexit_armed) {
        atexit(_dispatch_miss_dump);
        g_dispatch_miss_atexit_armed = 1;
    }
    for (unsigned i = 0; i < g_dispatch_miss_count; i++) {
        if (g_dispatch_miss[i].pc24 == pc24 &&
            g_dispatch_miss[i].source_pc24 == source_pc24 &&
            g_dispatch_miss[i].mx_idx == (uint8_t)mx_idx)
            return; /* already recorded */
    }
    if (g_dispatch_miss_count >= DISPATCH_MISS_CAP) return;
    g_dispatch_miss[g_dispatch_miss_count].pc24 = pc24;
    g_dispatch_miss[g_dispatch_miss_count].source_pc24 = source_pc24;
    g_dispatch_miss[g_dispatch_miss_count].mx_idx = (uint8_t)mx_idx;
    g_dispatch_miss_count++;
    /* Re-dump on every new unique miss: the process runs forever and is
     * killed with SIGKILL (atexit never fires), so persist incrementally.
     * Uniques converge quickly, so the rewrite cost is bounded. */
    _dispatch_miss_dump();
}

static RecompReturn (*_cpu_dispatch_lookup(CpuState *cpu, uint32 pc24))(CpuState *) {
    unsigned lo = 0;
    unsigned hi = g_dispatch_table_count;
    while (lo < hi) {
        unsigned mid = lo + (hi - lo) / 2;
        uint32 mid_pc = g_dispatch_table[mid].pc24;
        if (mid_pc == pc24) {
            unsigned idx = (unsigned)(((cpu->m_flag & 1) << 1) | (cpu->x_flag & 1));
            return g_dispatch_table[mid].variant[idx];
        }
        if (mid_pc < pc24) lo = mid + 1;
        else               hi = mid;
    }
    return NULL;
}

RecompReturn cpu_dispatch_pc_from(CpuState *cpu, uint32 pc24,
                                  uint16 entry_s_for_miss_restore,
                                  uint32 source_pc24) {
    pc24 &= 0xFFFFFFu;
    source_pc24 &= 0xFFFFFFu;
    unsigned mx_idx = (unsigned)(((cpu->m_flag & 1) << 1) | (cpu->x_flag & 1));
    int via_mirror = 0;
    RecompReturn (*fp)(CpuState *) = _cpu_dispatch_lookup(cpu, pc24);
    if (fp == NULL) {
        /* LoROM bank-mirror fallback: $00-$3F and $80-$BF share bytes.
         * Cfg may declare a function in one bank while the trampoline
         * popped (PB:PC) lands on the mirror. Try the other bank
         * before giving up — matches set_name_resolver's alias. */
        uint8 bank = (uint8)((pc24 >> 16) & 0xFF);
        if (bank < 0x40 || (bank >= 0x80 && bank < 0xC0)) {
            fp = _cpu_dispatch_lookup(cpu, pc24 ^ 0x800000u);
            if (fp != NULL) via_mirror = 1;
        }
    }
    _dispatch_log_record(pc24, source_pc24, mx_idx, fp != NULL, via_mirror);
    /* SF_TRACE_BOOT: temporary boot-flow probe — log the first N cross-function
     * dispatches (target, source, hit, mx) to see exactly where the real boot
     * sequence stops/unwinds. Ungated (cpu_state.c is always compiled). */
    {
        static int s_tb = -1;
        static int s_tb_n = 0;
        static int s_tb_max = 0;
        if (s_tb < 0) {
            const char *e = getenv("SF_TRACE_BOOT");
            s_tb = e ? 1 : 0;
            s_tb_max = e ? (int)strtol(e, NULL, 10) : 0;
            if (s_tb_max <= 0) s_tb_max = 250;
        }
        if (s_tb && s_tb_n < s_tb_max) {
            s_tb_n++;
            fprintf(stderr, "[boot] #%d pc=$%06X mx=%u found=%d mirror=%d from=$%06X\n",
                    s_tb_n, pc24, mx_idx, fp != NULL, via_mirror, source_pc24);
        }
    }
    /* SF_DUMP_MISSES: accumulate the unique set of dispatch MISSES (target,
     * source, mx) and dump them to the named file at exit. Drives the
     * coroutine-resume enumeration: a manufactured-RTL dispatcher (do_strat,
     * dostrats, jumptostate) that resumes a computed pointer lands here as a
     * miss when the resume PC isn't yet a registered func entry. The dump,
     * grouped by source, is the worklist of resume PCs to register. */
    if (fp == NULL) {
        static int s_dm = -1;
        static const char *s_dm_path = NULL;
        if (s_dm < 0) {
            s_dm_path = getenv("SF_DUMP_MISSES");
            s_dm = s_dm_path ? 1 : 0;
        }
        if (s_dm) {
            cpu_record_dispatch_miss(pc24, source_pc24, mx_idx);
        }
    }
    /* SF_TRACE_DISP: log dispatches to the player-istrat region ($0BB5xx) and
     * any dispatch MISS, to catch silently-skipped strats (e.g. player_Istrat
     * $0BB53C not registered for the dispatched mx mode → playpt never set). */
    {
        static int s_td = -1;
        if (s_td < 0) s_td = getenv("SF_TRACE_DISP") ? 1 : 0;
        if (s_td) {
            int near_player = (pc24 >= 0x0BB500u && pc24 <= 0x0BB600u);
            if (near_player || fp == NULL) {
                static int n = 0;
                if (n++ < 120)
                    fprintf(stderr, "[disp] pc=$%06X mx=%u found=%d mirror=%d from=$%06X\n",
                            pc24, mx_idx, fp != NULL, via_mirror, source_pc24);
            }
        }
    }
    if (fp == NULL) {
        /* Not found: the popped (PB:PC) is a normal mid-caller return addr,
         * not a known function entry. Unwind by restoring cpu->S to the value
         * the caller expects after THIS function returns and returning NORMAL.
         * The caller passes entry_s_for_miss_restore = entry_s + frame_size
         * (the S after this function pops its own return frame) — so a
         * balanced hrv=0 callee returns with its frame correctly popped, and a
         * PEI trampoline discards its residual params up to that point. Passing
         * bare entry_s here would under-pop by frame_size and leak the caller's
         * frame on every miss (the heavy-load DMA-queue-corruption softlock;
         * cf. MMX Dr Light "sprite vanish" 2026-05-24). */
        cpu->S = entry_s_for_miss_restore;
        return RECOMP_RETURN_NORMAL;
    }
    /* Option-1: a dispatched entry has no paired host-C caller. The target
     * runs with host_return_valid=0 so its RTS/RTL re-dispatches on the
     * popped PC rather than host-returning into this dispatch frame. The
     * chain unwinds when a dispatch misses (S restored above) -> NORMAL. */
    cpu->host_return_valid = 0;
    return fp(cpu);
}

RecompReturn cpu_dispatch_pc(CpuState *cpu, uint32 pc24,
                              uint16 entry_s_for_miss_restore) {
    return cpu_dispatch_pc_from(cpu, pc24, entry_s_for_miss_restore, 0xFFFFFFu);
}

/* Read-only probe: would a dispatch to pc24 find a function entry?
 * Mirrors cpu_dispatch_pc_from's lookup + LoROM bank-mirror fallback so
 * the RTS-decision trace can classify a popped PC as DISPATCH (entry
 * exists) vs MISS_UNWIND (host-return continuation) without side effects. */
int cpu_dispatch_has_entry(CpuState *cpu, uint32 pc24) {
    pc24 &= 0xFFFFFFu;
    if (_cpu_dispatch_lookup(cpu, pc24) != NULL) return 1;
    uint8 bank = (uint8)((pc24 >> 16) & 0xFF);
    if (bank < 0x40 || (bank >= 0x80 && bank < 0xC0))
        if (_cpu_dispatch_lookup(cpu, pc24 ^ 0x800000u) != NULL) return 1;
    return 0;
}

void cpu_state_init(CpuState *cpu, uint8 *ram) {
    cpu->A = 0;
    /* No cpu->B init — B is derived from (A >> 8) and has no separate state. */
    cpu->X = 0;
    cpu->Y = 0;
    cpu->S = 0x01FF;
    cpu->D = 0;
    cpu->DB = 0;
    cpu->PB = 0;
    /* Reset state per 65816 spec: emulation=1, M=X=I=1 (P=0x34). */
    cpu->P = CPU_P_M | CPU_P_X | CPU_P_I;
    cpu->m_flag = 1;
    cpu->x_flag = 1;
    cpu->emulation = 1;
    cpu->host_return_valid = 0;
    cpu->_flag_N = 0;
    cpu->_flag_V = 0;
    cpu->_flag_Z = 0;
    cpu->_flag_C = 0;
    cpu->_flag_I = 1;
    cpu->_flag_D = 0;
    cpu->ram = ram;
    /* NLR pending-skip is NOT on CpuState — it's a function-local in
     * each emitted v2 function. See cpu_state.h for design rationale. */
}
