// Super FX (GSU / MARIO Chip 1) emulator core.
//
// Canonical Super FX behavior, implemented against the well-documented spec
// (fullsnes; bsnes fxemu; snes9x fxemu). Opcode numbers are cited inline.
//
// Architecture summary
// --------------------
// * 16 general 16-bit registers R0..R15. R15 = program counter.
//   - Writing R15 sets the GO bit and starts execution.
//   - R14 reads trigger a ROM read (ROMBR-banked) into ROMDR.
// * Source/Dest register selection via Sreg/Dreg, set by the prefix opcodes:
//   - 0x10-0x1F = TO/MOVE (sets Dreg) or, with the B flag, WITH (sets both
//     Sreg and Dreg and keeps the B flag for the next op).
//   - FROM (also 0x10-0x1F under ALT1) sets Sreg.
//   After every non-prefix instruction Sreg and Dreg reset to R0.
// * ALT1/ALT2 prefix latches select opcode variants; cleared after the next
//   non-prefix instruction. 0x3D=ALT1, 0x3E=ALT2, 0x3F=ALT3(=ALT1|ALT2).
// * SFR carries Z, CY, S, OV, GO, R, IRQ + prefix latches.
//
// Memory
// ------
// * Code fetch: ROM banked by PBR (program bank register). $00-$5F map to
//   ROM offset (bank<<16)|addr for addr>=$8000 style LoROM; we use a simple
//   linear model bank*0x10000 + (addr) masked into romSize, which matches how
//   the GSU's program counter walks contiguous ROM. (The host wires real
//   arbitration later; for code-in-place this linear model is what the
//   MARIO routines assume.)
// * R14 data read: ROM banked by ROMBR.
// * Game Pak RAM: banked by RAMBR for LM/SM/load/store and the framebuffer.
//
// PLOT / framebuffer
// ------------------
// Implemented for 256-color (8bpp) mode first (Star Fox uses 256-color).
// The Super FX bitmap is char/tile organized. For 8bpp the per-pixel address
// math (see plot below) writes into the 8 bitplanes of the relevant 8x8 char
// at SCBR-based screen base. 4/16-color simplified (see CMODE notes).

#include "gsu.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

struct Gsu {
  // General registers. R15 = PC.
  uint16_t r[16];

  uint16_t sfr;   // status/flags
  uint8_t  pbr;   // program bank
  uint8_t  rombr; // ROM bank (R14 reads)
  uint8_t  rambr; // RAM bank
  uint8_t  cbr_hi, cbr_lo; // cache base ($303E/$303F)
  uint8_t  scbr;  // screen base
  uint8_t  scmr;  // screen mode
  uint8_t  clsr;  // clock select
  uint8_t  cfgr;  // config (bit5 = MS1 fast multiply, bit7 = IRQ disable)
  uint8_t  vcr;   // version code
  uint8_t  bramr; // backup RAM enable
  uint8_t  por;   // plot option register (set by CMODE)

  uint16_t romdr; // R14 ROM data latch
  uint8_t  romdr_byte; // last GETC color byte

  // Prefix / instruction state.
  uint8_t  sreg;  // source register index
  uint8_t  dreg;  // dest register index
  uint16_t lastRamAdr; // last RAM address touched by LDW/STW/LM/SM/LMS/SMS (for SBK)
  uint16_t color; // COLOR register (plot color)
  uint8_t  pipe;  // prefetched opcode (pipeline)
  bool     pipe_valid;

  // Branch delay slot. The GSU is pipelined: it prefetches the byte at R15
  // while executing the current instruction, so a taken branch/JMP/LOOP runs
  // the instruction immediately FOLLOWING it (the delay slot) before control
  // reaches the target. We model this by deferring the redirect: the branch
  // sets branch_pending + branch_target instead of writing R15; the gsu_step
  // wrapper lets the next instruction (the delay slot) execute, then applies
  // the target. branch_pbr (>=0) also defers a PBR change for LJMP.
  bool     branch_pending;
  uint16_t branch_target;
  int      branch_pbr;

  // High-byte latch for IWT/IBT immediate-word loads handled inline.

  // Memory.
  uint8_t* rom;
  uint32_t romSize;
  uint8_t* ram;
  uint32_t ramSize;

  // For LMULT/FMULT 32-bit results, the high word goes to R4.
};

// ---- SFR flag helpers --------------------------------------------------

static inline void set_flag(Gsu* g, uint16_t mask, int cond) {
  if (cond) g->sfr |= mask; else g->sfr &= ~mask;
}
static inline int get_flag(Gsu* g, uint16_t mask) {
  return (g->sfr & mask) != 0;
}

// ---- Memory access -----------------------------------------------------

static inline uint8_t rom_read(Gsu* g, uint8_t bank, uint16_t addr) {
  if (!g->rom || g->romSize == 0) return 0;
  // LoROM mapping — MUST match the CPU's RomPtr (common_rtl.c): Star Fox is a
  // LoROM Super FX cart. Banks $00-$3F see ROM at $8000-$FFFF; the GSU program
  // counter (PBR:R15) addresses the same ROM bytes the CPU does, so the linear
  // (bank<<16)|addr model fetched the WRONG bytes (e.g. $01:8295 read rom
  // offset $18295 instead of $8295) — the GSU executed garbage. Use the LoROM
  // offset = (bank<<15) | (addr & 0x7fff), mirrored to real ROM size.
  uint32_t off = (((uint32_t)bank << 15) | (addr & 0x7fff)) % g->romSize;
  return g->rom[off];
}

static inline uint8_t ram_read(Gsu* g, uint8_t bank, uint16_t addr) {
  if (!g->ram || g->ramSize == 0) return 0;
  uint32_t off = ((uint32_t)bank << 16) | addr;
  uint8_t v = g->ram[off & (g->ramSize - 1)];
  // Cache the env lookup once: getenv() takes a locked linear scan of the
  // environment, and this is the GSU's per-pixel/per-access hot path — calling
  // it every RAM read throttled the GSU to ~1 fps and stalled boot.
  static long s_rd_lim = -1;
  if (s_rd_lim < 0) { const char* e = getenv("GSU_RAMRD"); s_rd_lim = e ? atol(e) : 0; }
  if (s_rd_lim > 0) {
    static long s_n = 0;
    if (s_n < s_rd_lim) { s_n++;
      fprintf(stderr, "[ramrd] rambr=%02x addr=%04x off=%05x val=%02x pc=%02x:%04x\n",
              bank, addr, off & (g->ramSize - 1), v, g->pbr, g->r[15]); }
  }
  return v;
}
static inline void ram_write(Gsu* g, uint8_t bank, uint16_t addr, uint8_t v) {
  if (!g->ram || g->ramSize == 0) return;
  uint32_t off = ((uint32_t)bank << 16) | addr;
  // Cache the env lookup once (see ram_read): this is the GSU's per-pixel write
  // hot path; an unconditional getenv() here is a severe boot-time slowdown.
  static long s_wr_lim = -1;
  if (s_wr_lim < 0) { const char* e = getenv("GSU_RAMWR"); s_wr_lim = e ? atol(e) : 0; }
  if (s_wr_lim > 0) {
    static long s_nw = 0;
    if (s_nw < s_wr_lim) { s_nw++;
      fprintf(stderr, "[ramwr] rambr=%02x addr=%04x off=%05x val=%02x pc=%02x:%04x\n",
              bank, addr, off & (g->ramSize - 1), v, g->pbr, g->r[15]); }
  }
  g->ram[off & (g->ramSize - 1)] = v;
}

// Fetch next opcode byte from PBR:R15 and advance R15.
static inline uint8_t fetch(Gsu* g) {
  uint8_t b = rom_read(g, g->pbr, g->r[15]);
  g->r[15]++;
  return b;
}

// ---- Prefix bookkeeping ------------------------------------------------

// After each non-prefix instruction, reset Sreg/Dreg to R0 and clear the
// ALT1/ALT2/B latches.
static inline void clear_prefixes(Gsu* g) {
  g->sreg = 0;
  g->dreg = 0;
  g->sfr &= ~(GSU_SFR_ALT1 | GSU_SFR_ALT2 | GSU_SFR_B);
}

// Latch the ROM data byte/word at ROMBR:R14 into ROMDR. On real hardware a
// write to R14 starts a ROM fetch; the byte lands in ROMDR a few cycles later
// and GETB/GETC read it. We model it as an immediate latch (frame-level recomp
// has no GSU cycle clock). MUST be called whenever R14 changes, else GETB/GETC
// read a stale/zero ROMDR and the GSU processes garbage data (observed: 3D
// data streamed as all-zero -> a bad LOOP count -> the render loop never ends).
static inline void gsu_latch_romdr(Gsu* g) {
  // Data reads use the same LoROM mapping as code fetch (rom_read): the GSU
  // addresses ROM data through ROMBR:R14 just as it fetches code through
  // PBR:R15.
  uint8_t lo = rom_read(g, g->rombr, g->r[14]);
  uint8_t hi = rom_read(g, g->rombr, (uint16_t)(g->r[14] + 1));
  g->romdr_byte = lo;
  g->romdr = (uint16_t)(lo | (hi << 8));
}

// Defer a write to R15 (the program counter) by one instruction. The GSU is
// pipelined: writing R15 does NOT take effect immediately — the instruction
// already prefetched at the OLD R15 (the "delay slot") executes first, then
// control reaches the new target. This applies to EVERY R15 write (ALU/TO/MOVE/
// IWT/IBT/LM/LMS targeting R15), not just the explicit branch opcodes. We model
// it with the same branch_pending machinery the branch/JMP/LOOP ops use: the
// gsu_step wrapper runs the next instruction, then applies branch_target.
// (Missing this made `mcall` (= LINK + IWT R15) skip its delay slot `with rd2`,
// so MDECRUNCH's `sub rd2` never cleared rd2 across iterations → runaway count.)
static inline void set_r15_deferred(Gsu* g, uint16_t val) {
  g->branch_target = val;
  g->branch_pending = true;
}

// Write a value to the destination register; if Dreg==R15 this is a branch
// (alters PC) and is deferred one instruction (delay slot). Then apply the
// standard post-instruction prefix reset. A write to R14 triggers the ROM data
// prefetch, but that is handled uniformly by the gsu_step wrapper (any R14
// change re-latches ROMDR), so we don't special-case it here — that also covers
// INC/DEC/IWT/IBT/LM writes that bypass write_dreg.
static inline void write_dreg(Gsu* g, uint16_t val) {
  if (g->dreg == 15) set_r15_deferred(g, val);
  else g->r[g->dreg] = val;
}

// ---- Flag computation for ALU ops --------------------------------------

static inline void set_zs16(Gsu* g, uint16_t res) {
  set_flag(g, GSU_SFR_Z, res == 0);
  set_flag(g, GSU_SFR_S, (res & 0x8000) != 0);
}

// ---- PLOT (256-color / 8bpp) -------------------------------------------
//
// The Super FX bitmap is organized as 8x8 chars, each char being 8 rows of
// bitplanes. For 8bpp a char occupies 32 bytes (8 planes packed as 4 plane
// pairs of 16 bytes... bsnes models the per-pixel write as setting one bit in
// each of the 8 bitplanes for that pixel). We implement the documented OBJ
// (char-based) bitmap layout:
//
//   plotX in R1, plotY in R2 (by convention; we read R1/R2).
//   char column = plotX >> 3, char row = plotY >> 3
//   chars-per-row depends on screen height bits in SCMR; Star Fox uses the
//   "OBJ 128" / 256-wide style. We use 16 chars per row for the common
//   128/256 mode unless SCMR height bits indicate otherwise.
//
// Base address of the bitmap = SCBR << 10 within RAMBR-banked RAM.
//
// For each pixel we write the COLOR register's 8 bits across the 8 planes.
// Assumptions are documented; this is the standard plot-to-bitmap math.

// Color depth (bits per pixel) from SCMR.md (bits 0-1), per bsnes:
//   md 0 -> 2bpp (4 color), md 1 -> 4bpp (16 color), md 3 -> 8bpp (256 color).
// Star Fox's 3D bitmap runs in md 1 (4bpp): SCMR=$39 -> md=1. The char byte
// size scales with depth (2bpp=16, 4bpp=32, 8bpp=64), and the plot writes one
// bit into each of `bpp` planes. Using the wrong depth makes every char stride
// wrong, so adjacent chars overwrite each other and the bitmap is garbage.
static int gsu_bpp(Gsu* g) {
  switch (g->scmr & 3) { case 0: return 2; case 1: return 4; default: return 8; }
}

static uint32_t plot_char_base(Gsu* g, int charX, int charY) {
  // Screen base in 1KB units.
  uint32_t base = (uint32_t)g->scbr << 10;
  // Determine chars-per-column ("screen height") from SCMR bits.
  // mm_sch (bit2) and mm_sch1 (bit5) encode height: 128/160/192/OBJ.
  int height_sel = ((g->scmr >> 2) & 1) | (((g->scmr >> 5) & 1) << 1);
  int charsPerCol;
  switch (height_sel) {
    case 0: charsPerCol = 16; break; // 128 pixels tall
    case 1: charsPerCol = 20; break; // 160
    case 2: charsPerCol = 24; break; // 192
    default: charsPerCol = 16; break; // OBJ mode: treated as columns of 16
  }
  // Chars are laid out in column-major strips (each column is `charsPerCol`
  // chars tall). Char byte size scales with color depth: 2bpp=16, 4bpp=32,
  // 8bpp=64 (= bpp * 8 bytes).
  uint32_t charNum = (uint32_t)charX * charsPerCol + charY;
  return base + charNum * (uint32_t)(gsu_bpp(g) * 8);
}

// 8bpp char: 8 rows. Each row is represented (bsnes-style) as two pairs of
// planes: bytes [row*2 + plane_pair*16]. Bit (7 - (x&7)) selects the column.
// Diagnostic: total PLOT pixel writes since process start (GSU_PLOT_LOG).
unsigned long g_gsu_plot_count = 0;
static void plot_pixel_8bpp(Gsu* g, int x, int y, uint8_t color) {
  g_gsu_plot_count++;
  int charX = x >> 3, charY = y >> 3;
  int px = x & 7, py = y & 7;
  uint32_t cbase = plot_char_base(g, charX, charY);
  uint8_t bitmask = 0x80 >> px;
  int nplanes = gsu_bpp(g);
  // Plane n byte = cbase + ((n>>1)*16) + (n&1) + py*2 (bsnes layout). For 4bpp
  // only planes 0..3 exist (char is 32 bytes); for 8bpp planes 0..7 (64 bytes).
  for (int plane = 0; plane < nplanes; plane++) {
    uint32_t addr = cbase + ((plane >> 1) * 16) + (plane & 1) + py * 2;
    // addr may exceed 16 bits; fold the high bits into the RAM bank.
    uint8_t bank = g->rambr + (uint8_t)(addr >> 16);
    uint16_t a16 = (uint16_t)addr;
    uint8_t cur = ram_read(g, bank, a16);
    if (color & (1 << plane)) cur |= bitmask; else cur &= ~bitmask;
    ram_write(g, bank, a16, cur);
  }
}

static uint8_t rpix_pixel_8bpp(Gsu* g, int x, int y) {
  int charX = x >> 3, charY = y >> 3;
  int px = x & 7, py = y & 7;
  uint32_t cbase = plot_char_base(g, charX, charY);
  uint8_t bitmask = 0x80 >> px;
  uint8_t color = 0;
  int nplanes = gsu_bpp(g);
  for (int plane = 0; plane < nplanes; plane++) {
    uint32_t addr = cbase + ((plane >> 1) * 16) + (plane & 1) + py * 2;
    uint8_t bank = g->rambr + (uint8_t)(addr >> 16);
    uint16_t a16 = (uint16_t)addr;
    uint8_t cur = ram_read(g, bank, a16);
    if (cur & bitmask) color |= (1 << plane);
  }
  return color;
}

// ---- Lifecycle ---------------------------------------------------------

long g_gsu_trace_budget = 0;

Gsu* gsu_init(void) {
  Gsu* g = (Gsu*)calloc(1, sizeof(Gsu));
  const char* t = getenv("GSU_TRACE");
  if (t) g_gsu_trace_budget = atol(t);
  gsu_reset(g);
  return g;
}

void gsu_free(Gsu* g) { free(g); }

void gsu_reset(Gsu* g) {
  uint8_t* rom = g->rom; uint32_t rs = g->romSize;
  uint8_t* ram = g->ram; uint32_t ms = g->ramSize;
  memset(g, 0, sizeof(*g));
  g->rom = rom; g->romSize = rs; g->ram = ram; g->ramSize = ms;
  g->vcr = 0x04; // arbitrary version code
  g->branch_pbr = -1; // no deferred PBR change pending
}

void gsu_set_memory(Gsu* g, uint8_t* rom, uint32_t romSize,
                    uint8_t* ram, uint32_t ramSize) {
  g->rom = rom; g->romSize = romSize;
  g->ram = ram; g->ramSize = ramSize;
}

bool gsu_is_running(Gsu* g) { return get_flag(g, GSU_SFR_GO); }

uint16_t gsu_get_reg(Gsu* g, int n) { return g->r[n & 15]; }
void gsu_set_reg(Gsu* g, int n, uint16_t v) { g->r[n & 15] = v; }
uint16_t gsu_get_sfr(Gsu* g) { return g->sfr; }

// ---- Register window read/write ($3000-$303F) --------------------------

uint8_t gsu_read(Gsu* g, uint16_t reg) {
  uint16_t a = reg & 0x3ff; // mask into window; care about low offset
  if (a <= 0x1f) {
    // R0..R15 (two bytes each).
    int idx = a >> 1;
    return (a & 1) ? (g->r[idx] >> 8) : (g->r[idx] & 0xff);
  }
  switch (a) {
    case 0x30: return g->sfr & 0xff;
    case 0x31: return (g->sfr >> 8) & 0xff;
    case 0x33: return g->bramr;
    case 0x34: return g->pbr;
    case 0x36: return g->rombr;
    case 0x37: return g->cfgr;
    case 0x38: return g->scbr;
    case 0x39: return g->clsr;
    case 0x3a: return g->scmr;
    case 0x3b: return g->vcr;
    case 0x3c: return g->rambr;
    case 0x3e: return g->cbr_hi;
    case 0x3f: return g->cbr_lo;
    default: return 0;
  }
}

void gsu_write(Gsu* g, uint16_t reg, uint8_t val) {
  uint16_t a = reg & 0x3ff;
  if (a <= 0x1f) {
    int idx = a >> 1;
    if (a & 1) g->r[idx] = (g->r[idx] & 0x00ff) | ((uint16_t)val << 8);
    else       g->r[idx] = (g->r[idx] & 0xff00) | val;
    // Writing the HIGH byte of R15 ($301F) launches the GSU (R15 = PC).
    if (idx == 15 && (a & 1)) {
      g->sfr |= GSU_SFR_GO;
      g->pipe_valid = false;
      clear_prefixes(g);
      if (getenv("GSU_RAMDUMP") && g->ram) {
        fprintf(stderr, "[ramdump] launch pbr=%02x r15=%04x — cart->ram[0:0x80]:\n", g->pbr, g->r[15]);
        for (int row = 0; row < 0x80; row += 16) {
          fprintf(stderr, "  %05x:", row);
          for (int c = 0; c < 16; c++) fprintf(stderr, " %02x", g->ram[row + c]);
          fprintf(stderr, "\n");
        }
      }
      // Debug: dump the launch register state (env-gated, first N launches).
      if (getenv("GSU_LAUNCH_TRACE")) {
        static int s_n = 0;
        long lim = atol(getenv("GSU_LAUNCH_TRACE"));
        if (s_n < lim) {
          fprintf(stderr, "[gsu launch %d] pbr=%02x r15=%04x rombr=%02x scbr=%02x scmr=%02x\n",
                  s_n, g->pbr, g->r[15], g->rombr, g->scbr, g->scmr);
          for (int i = 0; i < 16; i += 4)
            fprintf(stderr, "   R%-2d=%04x R%-2d=%04x R%-2d=%04x R%-2d=%04x\n",
                    i, g->r[i], i+1, g->r[i+1], i+2, g->r[i+2], i+3, g->r[i+3]);
          s_n++;
        }
      }
    }
    return;
  }
  switch (a) {
    case 0x30: g->sfr = (g->sfr & 0xff00) | val; break;
    case 0x31: g->sfr = (g->sfr & 0x00ff) | ((uint16_t)val << 8); break;
    case 0x33: g->bramr = val; break;
    case 0x34: g->pbr = val; break;
    case 0x37: g->cfgr = val; break;
    case 0x38: g->scbr = val; break;
    case 0x39: g->clsr = val; break;
    case 0x3a: g->scmr = val; break;
    default: break;
  }
}

// ---- Branch helper ------------------------------------------------------

static void do_branch(Gsu* g, int take) {
  int8_t disp = (int8_t)fetch(g); // signed displacement byte; R15 now at delay slot
  if (take) {
    // Defer the redirect: the delay-slot instruction (at the current R15)
    // executes first, then gsu_step applies branch_target.
    g->branch_target = (uint16_t)(g->r[15] + disp);
    g->branch_pending = true;
  }
}

// ---- The instruction step ----------------------------------------------
//
// Executes one opcode. Returns the number of GSU cycles (approx 1 here).
static int gsu_step_inner(Gsu* g) {
#ifndef GSU_NO_TRACE
  /* Env-gated single-step trace (GSU_TRACE=N logs the first N instructions to
   * stderr as "GSU pbr:r15 op=.. sfr=.."). Debug aid for boot bring-up. */
  extern long g_gsu_trace_budget;
  if (g_gsu_trace_budget > 0) {
    g_gsu_trace_budget--;
    fprintf(stderr, "GSU %02x:%04x op=%02x sfr=%04x r1=%04x r2=%04x r3=%04x r4=%04x r9=%04x r11=%04x r12=%04x r14=%04x romdr=%04x\n",
            g->pbr, g->r[15], rom_read(g, g->pbr, g->r[15]), g->sfr,
            g->r[1], g->r[2], g->r[3], g->r[4], g->r[9], g->r[11], g->r[12], g->r[14], g->romdr);
  }
#endif
  uint8_t op = fetch(g);
  // Snapshot prefix state for THIS instruction.
  int alt1 = get_flag(g, GSU_SFR_ALT1);
  int alt2 = get_flag(g, GSU_SFR_ALT2);
  int bflag = get_flag(g, GSU_SFR_B);
  int sreg = g->sreg;

  switch (op) {
    // 0x00 STOP
    case 0x00:
      g->sfr &= ~GSU_SFR_GO;
      if (!(g->cfgr & 0x80)) g->sfr |= GSU_SFR_IRQ; // IRQ unless disabled
      clear_prefixes(g);
      return 1;

    // 0x01 NOP
    case 0x01:
      clear_prefixes(g);
      return 1;

    // 0x02 CACHE (we have no cache model; treat as NOP, set CBR=R15&0xfff0)
    case 0x02:
      g->cbr_lo = g->r[15] & 0xf0;
      g->cbr_hi = (g->r[15] >> 8) & 0xff;
      clear_prefixes(g);
      return 1;

    // 0x03 LSR (logical shift right)
    case 0x03: {
      uint16_t s = g->r[sreg];
      set_flag(g, GSU_SFR_CY, s & 1);
      uint16_t res = s >> 1;
      set_zs16(g, res);
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0x04 ROL (rotate left through carry)
    case 0x04: {
      uint16_t s = g->r[sreg];
      uint16_t res = (uint16_t)((s << 1) | (get_flag(g, GSU_SFR_CY) ? 1 : 0));
      set_flag(g, GSU_SFR_CY, s & 0x8000);
      set_zs16(g, res);
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0x05-0x0F: branches. Branch opcodes do NOT clear the register-select /
    // ALT / B prefixes: on the GSU the prefix set immediately before a branch
    // carries across the branch into its (always-executed) delay-slot
    // instruction, which then clears it. Star Fox's polygon renderer relies on
    // this exact idiom to zero an edge height:
    //     with rdy2        ; prefix: Sreg=Dreg=R3
    //     bra  mnewx1init
    //     sub  rdy2        ; delay slot: R3 = R3 - R3 = 0  (uses the prefix)
    // If the branch cleared the prefix, the delay-slot `sub` would target R0
    // instead and leave rdy2 holding stale scratch (a leftover high byte from
    // the decompressor), producing a degenerate ~991-scanline polygon. For a
    // branch NOT preceded by a prefix the latches are already clear, so this is
    // a no-op there. (Branches still defer the PC redirect via do_branch.)
    // 0x05 BRA (always)
    case 0x05: do_branch(g, 1); return 1;
    // 0x06 BGE  (S == OV)
    case 0x06: do_branch(g, get_flag(g,GSU_SFR_S)==get_flag(g,GSU_SFR_OV)); return 1;
    // 0x07 BLT  (S != OV)
    case 0x07: do_branch(g, get_flag(g,GSU_SFR_S)!=get_flag(g,GSU_SFR_OV)); return 1;
    // 0x08 BNE  (Z == 0)
    case 0x08: do_branch(g, !get_flag(g,GSU_SFR_Z)); return 1;
    // 0x09 BEQ  (Z == 1)
    case 0x09: do_branch(g, get_flag(g,GSU_SFR_Z)); return 1;
    // 0x0A BPL  (S == 0)
    case 0x0a: do_branch(g, !get_flag(g,GSU_SFR_S)); return 1;
    // 0x0B BMI  (S == 1)
    case 0x0b: do_branch(g, get_flag(g,GSU_SFR_S)); return 1;
    // 0x0C BCC  (CY == 0)
    case 0x0c: do_branch(g, !get_flag(g,GSU_SFR_CY)); return 1;
    // 0x0D BCS  (CY == 1)
    case 0x0d: do_branch(g, get_flag(g,GSU_SFR_CY)); return 1;
    // 0x0E BVC  (OV == 0)
    case 0x0e: do_branch(g, !get_flag(g,GSU_SFR_OV)); return 1;
    // 0x0F BVS  (OV == 1)
    case 0x0f: do_branch(g, get_flag(g,GSU_SFR_OV)); return 1;

    // 0x10-0x1F: TO/MOVE (set Dreg) or WITH (B flag set) -> set Sreg & Dreg.
    // Under ALT1, MOVES; here both behave as register-prefix selects.
    case 0x10: case 0x11: case 0x12: case 0x13:
    case 0x14: case 0x15: case 0x16: case 0x17:
    case 0x18: case 0x19: case 0x1a: case 0x1b:
    case 0x1c: case 0x1d: case 0x1e: case 0x1f: {
      int rn = op & 0x0f;
      if (bflag) {
        // MOVE Rn <- SREG: a real register move (no flag change), terminating.
        // Reached when a preceding WITH (0x20-0x2F) set the B flag.
        g->r[rn] = g->r[sreg];
        clear_prefixes(g); // also clears B
        return 1;
      } else {
        // TO Rn: prefix that selects the destination register for the
        // following ALU op. Does not reset prefixes / ALT latches.
        g->dreg = rn;
        return 1;
      }
    }

    // 0x20-0x2F: WITH (set B + Sreg=Dreg=rn) — prefix.
    case 0x20: case 0x21: case 0x22: case 0x23:
    case 0x24: case 0x25: case 0x26: case 0x27:
    case 0x28: case 0x29: case 0x2a: case 0x2b:
    case 0x2c: case 0x2d: case 0x2e: case 0x2f: {
      int rn = op & 0x0f;
      g->sreg = rn;
      g->dreg = rn;
      g->sfr |= GSU_SFR_B; // mark B for the following op
      return 1; // prefix
    }

    // 0x30-0x3B: STW/STB (store) (ALT2=byte) — store Sreg to (Rn) in RAM.
    case 0x30: case 0x31: case 0x32: case 0x33:
    case 0x34: case 0x35: case 0x36: case 0x37:
    case 0x38: case 0x39: case 0x3a: case 0x3b: {
      int rn = op & 0x0f;
      uint16_t addr = g->r[rn];
      g->lastRamAdr = addr;
      uint16_t v = g->r[sreg];
      if (alt1) {
        // STB: store low byte.
        ram_write(g, g->rambr, addr, v & 0xff);
      } else {
        // STW: store word (little-endian).
        ram_write(g, g->rambr, addr, v & 0xff);
        ram_write(g, g->rambr, addr + 1, v >> 8);
      }
      clear_prefixes(g);
      return 1;
    }

    // 0x3C LOOP: dec R12, branch to R13 if R12 != 0. Has a delay slot like the
    // other control-flow ops, so defer the redirect (see gsu_step).
    case 0x3c: {
      g->r[12]--;
      set_flag(g, GSU_SFR_Z, g->r[12] == 0);
      set_flag(g, GSU_SFR_S, g->r[12] & 0x8000);
      if (!get_flag(g, GSU_SFR_Z)) {
        g->branch_target = g->r[13];
        g->branch_pending = true;
      }
      clear_prefixes(g);
      return 1;
    }

    // 0x3D ALT1, 0x3E ALT2, 0x3F ALT3 — prefixes (do NOT reset).
    case 0x3d: g->sfr |= GSU_SFR_ALT1; return 1;
    case 0x3e: g->sfr |= GSU_SFR_ALT2; return 1;
    case 0x3f: g->sfr |= GSU_SFR_ALT1 | GSU_SFR_ALT2; return 1;

    // 0x40-0x4B: LDW/LDB (load) (ALT1=byte) — load (Rn) into Dreg.
    case 0x40: case 0x41: case 0x42: case 0x43:
    case 0x44: case 0x45: case 0x46: case 0x47:
    case 0x48: case 0x49: case 0x4a: case 0x4b: {
      int rn = op & 0x0f;
      uint16_t addr = g->r[rn];
      g->lastRamAdr = addr;
      uint16_t v;
      if (alt1) {
        v = ram_read(g, g->rambr, addr); // LDB byte
      } else {
        v = ram_read(g, g->rambr, addr) | ((uint16_t)ram_read(g, g->rambr, addr+1) << 8);
      }
      write_dreg(g, v);
      clear_prefixes(g);
      return 1;
    }

    // 0x4C PLOT / RPIX (ALT1=RPIX)
    case 0x4c: {
      if (alt1) {
        // RPIX: read pixel at (R1,R2) into Dreg.
        uint8_t c = rpix_pixel_8bpp(g, g->r[1], g->r[2]);
        write_dreg(g, c);
        set_zs16(g, c);
      } else {
        // PLOT: plot COLOR at (R1,R2); then R1++ (advance X).
        plot_pixel_8bpp(g, g->r[1], g->r[2], (uint8_t)g->color);
        g->r[1]++;
      }
      clear_prefixes(g);
      return 1;
    }

    // 0x4D SWAP (swap bytes of Sreg into Dreg)
    case 0x4d: {
      uint16_t s = g->r[sreg];
      uint16_t res = (uint16_t)((s >> 8) | (s << 8));
      set_zs16(g, res);
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0x4E COLOR / CMODE (ALT1=CMODE)
    case 0x4e: {
      if (alt1) {
        // CMODE: set plot option register from Sreg.
        g->por = g->r[sreg] & 0xff;
      } else {
        // COLOR: COLOR = Sreg low byte (with high-nibble handling per POR;
        // simplified: take low byte).
        g->color = g->r[sreg] & 0xff;
      }
      clear_prefixes(g);
      return 1;
    }

    // 0x4F NOT
    case 0x4f: {
      uint16_t res = ~g->r[sreg];
      set_zs16(g, res);
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0x50-0x5F: ADD/ADC (ALT1=ADC, ALT2=ADD #imm, ALT3=ADC #imm)
    case 0x50: case 0x51: case 0x52: case 0x53:
    case 0x54: case 0x55: case 0x56: case 0x57:
    case 0x58: case 0x59: case 0x5a: case 0x5b:
    case 0x5c: case 0x5d: case 0x5e: case 0x5f: {
      int rn = op & 0x0f;
      uint32_t a = g->r[sreg];
      uint32_t b = (alt2) ? (uint32_t)rn : (uint32_t)g->r[rn]; // #imm if ALT2
      uint32_t carry = (alt1) ? (get_flag(g, GSU_SFR_CY) ? 1 : 0) : 0;
      uint32_t res = a + b + carry;
      set_flag(g, GSU_SFR_CY, res > 0xffff);
      set_flag(g, GSU_SFR_OV, (~(a ^ b) & (a ^ res) & 0x8000) != 0);
      set_zs16(g, (uint16_t)res);
      write_dreg(g, (uint16_t)res);
      clear_prefixes(g);
      return 1;
    }

    // 0x60-0x6F: SUB/SBC/CMP (ALT1=SBC, ALT2=SUB #imm, ALT3=CMP)
    case 0x60: case 0x61: case 0x62: case 0x63:
    case 0x64: case 0x65: case 0x66: case 0x67:
    case 0x68: case 0x69: case 0x6a: case 0x6b:
    case 0x6c: case 0x6d: case 0x6e: case 0x6f: {
      int rn = op & 0x0f;
      uint32_t a = g->r[sreg];
      uint32_t b = (alt2 && !alt1) ? (uint32_t)rn : (uint32_t)g->r[rn]; // #imm if ALT2-only
      uint32_t borrow = (alt1 && !alt2) ? (get_flag(g, GSU_SFR_CY) ? 0 : 1) : 0; // SBC
      uint32_t res = a - b - borrow;
      set_flag(g, GSU_SFR_CY, !(res & 0x10000)); // CY=1 means no borrow
      set_flag(g, GSU_SFR_OV, ((a ^ b) & (a ^ res) & 0x8000) != 0);
      set_zs16(g, (uint16_t)res);
      // CMP (ALT3 = alt1&alt2) sets flags only, no writeback.
      if (!(alt1 && alt2)) write_dreg(g, (uint16_t)res);
      clear_prefixes(g);
      return 1;
    }

    // 0x70 MERGE
    case 0x70: {
      uint16_t res = (uint16_t)((g->r[7] & 0xff00) | (g->r[8] >> 8));
      // flags: bsnes sets OV/S/CY/Z based on high bits; simplified set_zs16.
      set_zs16(g, res);
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0x71-0x7F: AND/BIC (ALT1=BIC, ALT2=AND#imm, ALT3=BIC#imm)
    case 0x71: case 0x72: case 0x73:
    case 0x74: case 0x75: case 0x76: case 0x77:
    case 0x78: case 0x79: case 0x7a: case 0x7b:
    case 0x7c: case 0x7d: case 0x7e: case 0x7f: {
      int rn = op & 0x0f;
      uint16_t a = g->r[sreg];
      uint16_t b = alt2 ? (uint16_t)rn : g->r[rn];
      if (alt1) b = ~b; // BIC: AND with complement
      uint16_t res = a & b;
      set_zs16(g, res);
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0x80-0x8F: MULT/UMULT (ALT1=UMULT, ALT2=MULT#imm, ALT3=UMULT#imm)
    case 0x80: case 0x81: case 0x82: case 0x83:
    case 0x84: case 0x85: case 0x86: case 0x87:
    case 0x88: case 0x89: case 0x8a: case 0x8b:
    case 0x8c: case 0x8d: case 0x8e: case 0x8f: {
      int rn = op & 0x0f;
      uint16_t res;
      if (alt1) {
        // UMULT: unsigned 8x8.
        uint16_t bv = alt2 ? (uint16_t)rn : (g->r[rn] & 0xff);
        res = (uint16_t)((g->r[sreg] & 0xff) * bv);
      } else {
        // MULT: signed 8x8.
        int16_t sv = (int8_t)(g->r[sreg] & 0xff);
        int16_t bv = alt2 ? (int16_t)(int8_t)rn : (int16_t)(int8_t)(g->r[rn] & 0xff);
        res = (uint16_t)(sv * bv);
      }
      set_zs16(g, res);
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0x90 SBK: store SREG word back to the last RAM address touched by a
    // LDW/STW/LM/SM/LMS/SMS (the GSU's RAM-address latch).
    case 0x90: {
      uint16_t addr = g->lastRamAdr;
      ram_write(g, g->rambr, addr, g->r[sreg] & 0xff);
      ram_write(g, g->rambr, addr + 1, g->r[sreg] >> 8);
      clear_prefixes(g);
      return 1;
    }

    // 0x91-0x94 LINK #n: R11 = R15 + n
    case 0x91: case 0x92: case 0x93: case 0x94: {
      int n = op & 0x0f;
      g->r[11] = g->r[15] + n;
      clear_prefixes(g);
      return 1;
    }

    // 0x95 SEX (sign-extend byte of Sreg)
    case 0x95: {
      uint16_t res = (uint16_t)(int16_t)(int8_t)(g->r[sreg] & 0xff);
      set_zs16(g, res);
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0x96 ASR / DIV2 (ALT1=DIV2) — arithmetic shift right.
    case 0x96: {
      int16_t s = (int16_t)g->r[sreg];
      set_flag(g, GSU_SFR_CY, s & 1);
      int16_t res = s >> 1;
      set_zs16(g, (uint16_t)res);
      write_dreg(g, (uint16_t)res);
      clear_prefixes(g);
      return 1;
    }

    // 0x97 ROR (rotate right through carry)
    case 0x97: {
      uint16_t s = g->r[sreg];
      uint16_t res = (uint16_t)((s >> 1) | (get_flag(g,GSU_SFR_CY) ? 0x8000 : 0));
      set_flag(g, GSU_SFR_CY, s & 1);
      set_zs16(g, res);
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0x98-0x9D JMP Rn / LJMP (ALT1=LJMP sets PBR too). Both have a delay slot
    // (the instruction following the JMP executes before control reaches Rn),
    // so defer the redirect via branch_pending (see gsu_step).
    case 0x98: case 0x99: case 0x9a: case 0x9b:
    case 0x9c: case 0x9d: {
      int rn = op & 0x0f;
      if (alt1) {
        // LJMP Rn (canonical Super FX): the OPERAND register Rn supplies the
        // program bank (PBR = Rn & 0x7f), and SREG supplies the offset
        // (R15 = SREG). These were previously swapped, corrupting every
        // cross-bank GSU jump (Star Fox's rasterizer dispatch uses LJMP).
        g->branch_target = g->r[sreg];
        g->branch_pbr    = g->r[rn] & 0x7f;
      } else {
        // JMP Rn: R15 = Rn, same program bank.
        g->branch_target = g->r[rn];
        g->branch_pbr    = -1;
      }
      g->branch_pending = true;
      clear_prefixes(g);
      return 1;
    }

    // 0x9E LOB (low byte of Sreg, zero high)
    case 0x9e: {
      uint16_t res = g->r[sreg] & 0x00ff;
      set_flag(g, GSU_SFR_Z, res == 0);
      set_flag(g, GSU_SFR_S, (res & 0x80) != 0);
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0x9F FMULT / LMULT (ALT1=LMULT) signed 16x16 fractional multiply.
    case 0x9f: {
      int32_t prod = (int32_t)(int16_t)g->r[sreg] * (int32_t)(int16_t)g->r[6];
      uint16_t hi = (uint16_t)(prod >> 16);
      if (alt1) {
        // LMULT: low word -> R4, high word -> Dreg.
        g->r[4] = (uint16_t)(prod & 0xffff);
      }
      set_flag(g, GSU_SFR_CY, (prod >> 15) & 1);
      set_zs16(g, hi);
      write_dreg(g, hi);
      clear_prefixes(g);
      return 1;
    }

    // 0xA0-0xAF IBT Rn,#imm / LMS / SMS (ALT1=LMS load, ALT2=SMS store).
    // (Canonical: base=IBT, ALT1=LMS, ALT2=SMS — these were previously swapped.)
    case 0xa0: case 0xa1: case 0xa2: case 0xa3:
    case 0xa4: case 0xa5: case 0xa6: case 0xa7:
    case 0xa8: case 0xa9: case 0xaa: case 0xab:
    case 0xac: case 0xad: case 0xae: case 0xaf: {
      int rn = op & 0x0f;
      if (alt1) {
        // LMS Rn,(yy): load Rn word from RAM at (#imm<<1).
        uint8_t pp = fetch(g);
        uint16_t addr = (uint16_t)pp << 1;
        g->lastRamAdr = addr;
        uint16_t v = ram_read(g, g->rambr, addr) | ((uint16_t)ram_read(g, g->rambr, addr+1) << 8);
        if (rn == 15) set_r15_deferred(g, v); else g->r[rn] = v;
      } else if (alt2) {
        // SMS (yy),Rn: store Rn word to RAM at (#imm<<1).
        uint8_t pp = fetch(g);
        uint16_t addr = (uint16_t)pp << 1;
        g->lastRamAdr = addr;
        ram_write(g, g->rambr, addr, g->r[rn] & 0xff);
        ram_write(g, g->rambr, addr + 1, g->r[rn] >> 8);
      } else {
        // IBT Rn,#imm : sign-extended byte immediate.
        int8_t imm = (int8_t)fetch(g);
        if (rn == 15) set_r15_deferred(g, (uint16_t)(int16_t)imm);
        else g->r[rn] = (uint16_t)(int16_t)imm;
      }
      clear_prefixes(g);
      return 1;
    }

    // 0xB0-0xBF: FROM (set Sreg) or MOVES (B flag).
    case 0xb0: case 0xb1: case 0xb2: case 0xb3:
    case 0xb4: case 0xb5: case 0xb6: case 0xb7:
    case 0xb8: case 0xb9: case 0xba: case 0xbb:
    case 0xbc: case 0xbd: case 0xbe: case 0xbf: {
      int rn = op & 0x0f;
      if (bflag) {
        // MOVES: Dreg = Rn, set flags. (B was set by a preceding WITH/0x20.)
        uint16_t v = g->r[rn];
        set_flag(g, GSU_SFR_OV, (v & 0x80) != 0);
        set_zs16(g, v);
        write_dreg(g, v);
        clear_prefixes(g);
        return 1;
      } else {
        // FROM: set Sreg = rn (prefix).
        g->sreg = rn;
        return 1; // prefix
      }
    }

    // 0xC0 HIB (high byte of Sreg moved to low byte of Dreg)
    case 0xc0: {
      uint16_t res = (g->r[sreg] >> 8) & 0xff;
      set_flag(g, GSU_SFR_Z, res == 0);
      set_flag(g, GSU_SFR_S, (res & 0x80) != 0);
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0xC1-0xCF: OR/XOR (ALT1=XOR, ALT2=OR#imm, ALT3=XOR#imm)
    case 0xc1: case 0xc2: case 0xc3:
    case 0xc4: case 0xc5: case 0xc6: case 0xc7:
    case 0xc8: case 0xc9: case 0xca: case 0xcb:
    case 0xcc: case 0xcd: case 0xce: case 0xcf: {
      int rn = op & 0x0f;
      uint16_t a = g->r[sreg];
      uint16_t b = alt2 ? (uint16_t)rn : g->r[rn];
      uint16_t res = alt1 ? (a ^ b) : (a | b);
      set_zs16(g, res);
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0xD0-0xDE: INC Rn
    case 0xd0: case 0xd1: case 0xd2: case 0xd3:
    case 0xd4: case 0xd5: case 0xd6: case 0xd7:
    case 0xd8: case 0xd9: case 0xda: case 0xdb:
    case 0xdc: case 0xdd: case 0xde: {
      int rn = op & 0x0f;
      g->r[rn]++;
      set_zs16(g, g->r[rn]);
      clear_prefixes(g);
      return 1;
    }

    // 0xDF: GETC / RAMB / ROMB (ALT2=RAMB, ALT3=ROMB)
    case 0xdf: {
      if (alt2 && alt1) {
        // ROMB: set ROM bank = Sreg low byte.
        g->rombr = g->r[sreg] & 0xff;
      } else if (alt2) {
        // RAMB: set RAM bank = Sreg low byte.
        g->rambr = g->r[sreg] & 0xff;
      } else {
        // GETC: COLOR = the latched ROM data byte (ROMDR). The latch is
        // (re)loaded whenever R14 changes (see gsu_step); GETC does NOT
        // re-sample ROM here — a ROMB between the R14 write and GETC must NOT
        // reload the buffer (hardware quirk Doom relies on).
        g->color = g->romdr_byte;
      }
      clear_prefixes(g);
      return 1;
    }

    // 0xE0-0xEE: DEC Rn
    case 0xe0: case 0xe1: case 0xe2: case 0xe3:
    case 0xe4: case 0xe5: case 0xe6: case 0xe7:
    case 0xe8: case 0xe9: case 0xea: case 0xeb:
    case 0xec: case 0xed: case 0xee: {
      int rn = op & 0x0f;
      g->r[rn]--;
      set_zs16(g, g->r[rn]);
      clear_prefixes(g);
      return 1;
    }

    // 0xEF: GETB / GETBH / GETBL / GETBS (ALT variants) — read the latched ROM
    // byte (ROMDR). The latch is (re)loaded whenever R14 changes (see
    // gsu_step); GETB does NOT re-sample ROM here.
    case 0xef: {
      uint8_t b = g->romdr_byte;
      uint16_t res;
      if (alt1 && alt2) {
        // GETBS: sign-extend byte.
        res = (uint16_t)(int16_t)(int8_t)b;
      } else if (alt1) {
        // GETBH: byte into high.
        res = (uint16_t)((g->r[sreg] & 0x00ff) | ((uint16_t)b << 8));
      } else if (alt2) {
        // GETBL: byte into low.
        res = (uint16_t)((g->r[sreg] & 0xff00) | b);
      } else {
        // GETB: zero-extend byte.
        res = b;
      }
      write_dreg(g, res);
      clear_prefixes(g);
      return 1;
    }

    // 0xF0-0xFF: IWT Rn,#imm16 / LM / SM (canonical: ALT1=LM load, ALT2=SM
    // store — same load=ALT1/store=ALT2 convention as LMS/SMS at 0xA0). These
    // were previously swapped, which inverted mbumwipe's `lm r14,[m_wintabptr]`
    // (load) into a store and `sm [m_wintabptr],r14` (store) into a load, so the
    // GSU never loaded the wipe-table pointer and the title/intro wipe (and
    // every other LM/SM-using routine) walked garbage and never advanced.
    case 0xf0: case 0xf1: case 0xf2: case 0xf3:
    case 0xf4: case 0xf5: case 0xf6: case 0xf7:
    case 0xf8: case 0xf9: case 0xfa: case 0xfb:
    case 0xfc: case 0xfd: case 0xfe: case 0xff: {
      int rn = op & 0x0f;
      if (alt1) {
        // LM Rn,(xx): load Rn word from RAM at absolute word addr.
        uint16_t lo = fetch(g);
        uint16_t hi = fetch(g);
        uint16_t addr = lo | (hi << 8);
        uint16_t v = ram_read(g, g->rambr, addr) | ((uint16_t)ram_read(g, g->rambr, addr+1) << 8);
        if (rn == 15) set_r15_deferred(g, v); else g->r[rn] = v;
      } else if (alt2) {
        // SM (xx),Rn: store Rn word to RAM at absolute word addr.
        uint16_t lo = fetch(g);
        uint16_t hi = fetch(g);
        uint16_t addr = lo | (hi << 8);
        ram_write(g, g->rambr, addr, g->r[rn] & 0xff);
        ram_write(g, g->rambr, addr + 1, g->r[rn] >> 8);
      } else {
        // IWT Rn,#imm16 : load immediate word (little-endian).
        uint16_t lo = fetch(g);
        uint16_t hi = fetch(g);
        uint16_t v = lo | (hi << 8);
        // IWT R15 (= `miwt pc,addr`, the jump in mcall/mlbeq) is a deferred
        // branch: the delay-slot instruction runs before control transfers.
        if (rn == 15) set_r15_deferred(g, v); else g->r[rn] = v;
      }
      clear_prefixes(g);
      return 1;
    }

    default:
      // Unknown / unimplemented opcode: log once and treat as NOP.
      fprintf(stderr, "[gsu] unimplemented opcode 0x%02X at PBR:%02X PC:%04X\n",
              op, g->pbr, (uint16_t)(g->r[15] - 1));
      clear_prefixes(g);
      return 1;
  }
}

// Execute one opcode and model the R14-triggered ROM prefetch: whenever an
// instruction changes R14 (write, INC, DEC, ALU, IWT/IBT/LM, ...), the GSU
// starts a ROM read at ROMBR:R14 whose byte lands in ROMDR for the next
// GETB/GETC. Detecting the change here re-latches uniformly regardless of which
// opcode wrote R14, using the ROMBR in effect at that moment (so a later ROMB
// without an R14 write does NOT reload — the documented hardware quirk).
static int gsu_step(Gsu* g) {
  uint16_t r14_before = g->r[14];
  // A branch/JMP/LOOP from the PREVIOUS step deferred its redirect so that the
  // delay-slot instruction (the one immediately after the branch) runs first.
  // Capture that pending redirect, run this instruction (the delay slot), then
  // apply the target — unless the delay slot itself branched (rare; its branch
  // wins). This mirrors the GSU's prefetch pipeline (snes9x FX_STEP).
  bool had_pending = g->branch_pending;
  uint16_t target = g->branch_target;
  int target_pbr = g->branch_pbr;
  g->branch_pending = false;
  g->branch_pbr = -1;

  int cycles = gsu_step_inner(g);
  if (g->r[14] != r14_before) gsu_latch_romdr(g);

  if (had_pending && !g->branch_pending) {
    g->r[15] = target;
    if (target_pbr >= 0) g->pbr = (uint8_t)target_pbr;
  }
  return cycles;
}

void gsu_run(Gsu* g, int maxCycles) {
  int cycles = 0;
  uint16_t entry_r15 = g->r[15];  // launch PC, for GSU_DUMP_RAM targeting
  // Diagnostic watchdog: if a single synchronous run exceeds GSU_WATCHDOG
  // cycles, it almost certainly means the GSU program is looping without
  // reaching STOP. Print a small PC histogram so the loop site is visible,
  // then force-return so the CPU/IRQ pump can make progress. Env-gated; the
  // lookup is cached so production builds pay only one int compare per call.
  static long s_wd = -1;
  if (s_wd < 0) { const char* e = getenv("GSU_WATCHDOG"); s_wd = e ? atol(e) : 0; }
  if (s_wd > 0) {
    long steps = 0;
    while (gsu_is_running(g) && cycles < maxCycles) {
      cycles += gsu_step(g);
      steps++;
      if ((steps % s_wd) == 0)
        fprintf(stderr, "[gsu-mon] s=%ld pc=%02x:%04x sfr=%04x R0..15=%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x r13t=%04x\n",
                steps, g->pbr, g->r[15], g->sfr,
                g->r[0],g->r[1],g->r[2],g->r[3],g->r[4],g->r[5],g->r[6],g->r[7],
                g->r[8],g->r[9],g->r[10],g->r[11],g->r[12],g->r[13],g->r[14],g->r[15], g->r[13]);
    }
    fprintf(stderr, "[gsu-mon] RETURNED naturally: steps=%ld cyc=%d GO=%d\n",
            steps, cycles, gsu_is_running(g));
    return;
  }
  while (gsu_is_running(g) && cycles < maxCycles) {
    cycles += gsu_step(g);
  }
  // GSU_DUMP_RAM=N: after the first run that plots >= N pixels, dump the full
  // Game Pak RAM and log SCBR/RAMBR/SCMR so we can locate the rendered back
  // buffer and compare it to the VRAM transfer source.
  // SF_RENDER_TRACE=N: after each of the first N MDO_3D_DISPLAY runs, log the
  // nonzero/distinct state of the $AC00 bitmap region so we can order it against
  // the DMA-time sample (SF_DMA_TRACE) and see if a clear wipes it post-render.
  if (entry_r15 == 0xac21) { static long s_rt = -2, s_rn = 0;
    if (s_rt == -2) { const char* e = getenv("SF_RENDER_TRACE"); s_rt = e ? atol(e) : 0; }
    if (s_rt > 0 && s_rn < s_rt && g->ram) { s_rn++;
      long nz=0; uint8_t seen[256]; for(int k=0;k<256;k++) seen[k]=0; long dis=0;
      for (uint32_t k=0xac00; k<0xae00; k++){ uint8_t bb=g->ram[k & (g->ramSize-1)]; if(bb)nz++; if(!seen[bb]){seen[bb]=1;dis++;} }
      fprintf(stderr, "[sf-render #%ld] mdo_3d_display done: $AC00[0:0x200] nz=%ld distinct=%ld scbr=%02x\n",
              s_rn, nz, dis, g->scbr); fflush(stderr); }
  }
  // GSU_DUMP_RAM: dump Game Pak RAM right after the Nth MDO_3D_DISPLAY ($ac21)
  // run (the actual 3D scene render), capturing the SCBR that routine used, so
  // we locate the real rendered bitmap rather than an arbitrary math/dust run.
  { static long s_dr = -2; if (s_dr == -2) { const char* e = getenv("GSU_DUMP_RAM"); s_dr = e ? atol(e) : 0; }
    if (s_dr > 0 && entry_r15 == 0xac21) { extern unsigned long g_gsu_plot_count;
      static long s_seen = 0; static int s_done = 0; s_seen++;
      if (!s_done && s_seen >= s_dr) {
        s_done = 1;
        FILE* f = fopen("/tmp/sf_gpram.bin", "wb");
        if (f && g->ram) { fwrite(g->ram, 1, g->ramSize, f); fclose(f); }
        fprintf(stderr, "[gsu-dump] AFTER mdo_3d_display #%ld: scbr=%02x rambr=%02x scmr=%02x plots=%lu -> /tmp/sf_gpram.bin\n",
                s_seen, g->scbr, g->rambr, g->scmr, g_gsu_plot_count); fflush(stderr);
      }
    }
  }
}

// Force the GSU to halt as if it had executed STOP. Used as a safety valve when
// a synchronous run exceeds its cycle budget (a non-terminating render would
// otherwise wedge the CPU's SFR busy-wait). Mirrors the 0x00 STOP opcode.
void gsu_force_stop(Gsu* g) {
  g->sfr &= ~GSU_SFR_GO;
  if (!(g->cfgr & 0x80)) g->sfr |= GSU_SFR_IRQ;
  clear_prefixes(g);
}
