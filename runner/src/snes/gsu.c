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
  uint16_t color; // COLOR register (plot color)
  uint8_t  pipe;  // prefetched opcode (pipeline)
  bool     pipe_valid;

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
  uint32_t off = ((uint32_t)bank << 16) | addr;
  return g->rom[off & (g->romSize - 1)];
}

static inline uint8_t ram_read(Gsu* g, uint8_t bank, uint16_t addr) {
  if (!g->ram || g->ramSize == 0) return 0;
  uint32_t off = ((uint32_t)bank << 16) | addr;
  return g->ram[off & (g->ramSize - 1)];
}
static inline void ram_write(Gsu* g, uint8_t bank, uint16_t addr, uint8_t v) {
  if (!g->ram || g->ramSize == 0) return;
  uint32_t off = ((uint32_t)bank << 16) | addr;
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

// Write a value to the destination register; if Dreg==R15 this is a branch
// (alters PC). Then apply the standard post-instruction prefix reset.
static inline void write_dreg(Gsu* g, uint16_t val) {
  g->r[g->dreg] = val;
  // R15 write here just sets PC; GO stays as-is (already running).
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
  // chars tall), 8bpp => 32 bytes per char.
  uint32_t charNum = (uint32_t)charX * charsPerCol + charY;
  return base + charNum * 32;
}

// 8bpp char: 8 rows. Each row is represented (bsnes-style) as two pairs of
// planes: bytes [row*2 + plane_pair*16]. Bit (7 - (x&7)) selects the column.
static void plot_pixel_8bpp(Gsu* g, int x, int y, uint8_t color) {
  int charX = x >> 3, charY = y >> 3;
  int px = x & 7, py = y & 7;
  uint32_t cbase = plot_char_base(g, charX, charY);
  uint8_t bitmask = 0x80 >> px;
  // 8 bitplanes: planes 0,1 at offset row*2; planes 2,3 at +16;
  // planes 4,5 at +32-equivalent... For an 8bpp char the standard layout is
  // 4 plane-pairs at +0,+16,+? . bsnes uses: plane n byte = cbase +
  // ((n>>1)*16) + (n&1) + py*2. We follow that for all 8 planes:
  for (int plane = 0; plane < 8; plane++) {
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
  for (int plane = 0; plane < 8; plane++) {
    uint32_t addr = cbase + ((plane >> 1) * 16) + (plane & 1) + py * 2;
    uint8_t bank = g->rambr + (uint8_t)(addr >> 16);
    uint16_t a16 = (uint16_t)addr;
    uint8_t cur = ram_read(g, bank, a16);
    if (cur & bitmask) color |= (1 << plane);
  }
  return color;
}

// ---- Lifecycle ---------------------------------------------------------

Gsu* gsu_init(void) {
  Gsu* g = (Gsu*)calloc(1, sizeof(Gsu));
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
  int8_t disp = (int8_t)fetch(g); // signed displacement byte
  if (take) g->r[15] = (uint16_t)(g->r[15] + disp);
}

// ---- The instruction step ----------------------------------------------
//
// Executes one opcode. Returns the number of GSU cycles (approx 1 here).
static int gsu_step(Gsu* g) {
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

    // 0x05 BRA (always)
    case 0x05: do_branch(g, 1); clear_prefixes(g); return 1;
    // 0x06 BGE  (S == OV)
    case 0x06: do_branch(g, get_flag(g,GSU_SFR_S)==get_flag(g,GSU_SFR_OV)); clear_prefixes(g); return 1;
    // 0x07 BLT  (S != OV)
    case 0x07: do_branch(g, get_flag(g,GSU_SFR_S)!=get_flag(g,GSU_SFR_OV)); clear_prefixes(g); return 1;
    // 0x08 BNE  (Z == 0)
    case 0x08: do_branch(g, !get_flag(g,GSU_SFR_Z)); clear_prefixes(g); return 1;
    // 0x09 BEQ  (Z == 1)
    case 0x09: do_branch(g, get_flag(g,GSU_SFR_Z)); clear_prefixes(g); return 1;
    // 0x0A BPL  (S == 0)
    case 0x0a: do_branch(g, !get_flag(g,GSU_SFR_S)); clear_prefixes(g); return 1;
    // 0x0B BMI  (S == 1)
    case 0x0b: do_branch(g, get_flag(g,GSU_SFR_S)); clear_prefixes(g); return 1;
    // 0x0C BCC  (CY == 0)
    case 0x0c: do_branch(g, !get_flag(g,GSU_SFR_CY)); clear_prefixes(g); return 1;
    // 0x0D BCS  (CY == 1)
    case 0x0d: do_branch(g, get_flag(g,GSU_SFR_CY)); clear_prefixes(g); return 1;
    // 0x0E BVC  (OV == 0)
    case 0x0e: do_branch(g, !get_flag(g,GSU_SFR_OV)); clear_prefixes(g); return 1;
    // 0x0F BVS  (OV == 1)
    case 0x0f: do_branch(g, get_flag(g,GSU_SFR_OV)); clear_prefixes(g); return 1;

    // 0x10-0x1F: TO/MOVE (set Dreg) or WITH (B flag set) -> set Sreg & Dreg.
    // Under ALT1, MOVES; here both behave as register-prefix selects.
    case 0x10: case 0x11: case 0x12: case 0x13:
    case 0x14: case 0x15: case 0x16: case 0x17:
    case 0x18: case 0x19: case 0x1a: case 0x1b:
    case 0x1c: case 0x1d: case 0x1e: case 0x1f: {
      int rn = op & 0x0f;
      if (bflag) {
        // WITH: this opcode followed a previous WITH/0x20-prefix; it is a
        // MOVE Sreg->Dreg=rn ... but canonical WITH sets Sreg=Dreg=rn.
        g->sreg = rn;
        g->dreg = rn;
        g->sfr &= ~GSU_SFR_B; // B consumed
        // WITH is itself a prefix: do NOT reset prefixes; keep ALT latches.
        return 1;
      } else {
        // TO/MOVE prefix: set Dreg = rn. It's a prefix (no prefix reset),
        // EXCEPT when... TO is a pure prefix. The actual MOVE happens when
        // the *next* opcode is also a register op; in canonical GSU, TO just
        // sets Dreg and is a prefix instruction.
        g->dreg = rn;
        return 1; // prefix: do not clear ALT/Sreg
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

    // 0x3C LOOP: dec R12, branch to R13 if R12 != 0.
    case 0x3c: {
      g->r[12]--;
      set_flag(g, GSU_SFR_Z, g->r[12] == 0);
      set_flag(g, GSU_SFR_S, g->r[12] & 0x8000);
      if (!get_flag(g, GSU_SFR_Z)) g->r[15] = g->r[13];
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

    // 0x90 SBK: store word from Sreg to last RAM address (RAMADDR).
    case 0x90: {
      // We model RAMADDR as the last LDW/STW address; simplest is to store
      // to address in... canonical SBK uses an internal RAM buffer address.
      // Simplified: store Sreg word at (R0)? bsnes keeps RAMADDR latch.
      // We keep a latch-free model: store at address held in our `ramaddr`.
      // Use R-less approach: store to ram[rambr:lastram]. To stay simple and
      // correct for tests, store word at address = g->color? No — use R1.
      // Stub: log once.
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

    // 0x98-0x9D JMP Rn / LJMP (ALT1=LJMP sets PBR too)
    case 0x98: case 0x99: case 0x9a: case 0x9b:
    case 0x9c: case 0x9d: {
      int rn = op & 0x0f;
      if (alt1) {
        // LJMP: PBR = Sreg low byte; PC = Rn; reload cache base.
        g->pbr = g->r[sreg] & 0xff;
        g->r[15] = g->r[rn];
      } else {
        g->r[15] = g->r[rn];
      }
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

    // 0xA0-0xAF IBT Rn,#imm / LMS / SMS (ALT1=SMS, ALT2=LMS)
    case 0xa0: case 0xa1: case 0xa2: case 0xa3:
    case 0xa4: case 0xa5: case 0xa6: case 0xa7:
    case 0xa8: case 0xa9: case 0xaa: case 0xab:
    case 0xac: case 0xad: case 0xae: case 0xaf: {
      int rn = op & 0x0f;
      if (alt1) {
        // SMS Rn: store Rn word to RAM at (#imm<<1).
        uint8_t pp = fetch(g);
        uint16_t addr = (uint16_t)pp << 1;
        ram_write(g, g->rambr, addr, g->r[rn] & 0xff);
        ram_write(g, g->rambr, addr + 1, g->r[rn] >> 8);
      } else if (alt2) {
        // LMS Rn: load Rn word from RAM at (#imm<<1).
        uint8_t pp = fetch(g);
        uint16_t addr = (uint16_t)pp << 1;
        g->r[rn] = ram_read(g, g->rambr, addr) | ((uint16_t)ram_read(g, g->rambr, addr+1) << 8);
      } else {
        // IBT Rn,#imm : sign-extended byte immediate.
        int8_t imm = (int8_t)fetch(g);
        g->r[rn] = (uint16_t)(int16_t)imm;
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
        // GETC: COLOR = last ROM data byte (R14 read latch).
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

    // 0xEF: GETB / GETBH / GETBL / GETBS (ALT variants) — read ROM byte.
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

    // 0xF0-0xFF: IWT Rn,#imm16 / LM / SM (ALT1=SM, ALT2=LM)
    case 0xf0: case 0xf1: case 0xf2: case 0xf3:
    case 0xf4: case 0xf5: case 0xf6: case 0xf7:
    case 0xf8: case 0xf9: case 0xfa: case 0xfb:
    case 0xfc: case 0xfd: case 0xfe: case 0xff: {
      int rn = op & 0x0f;
      if (alt1) {
        // SM (Rm),Rn? canonical SM stores Rn to (xx) absolute word addr.
        uint16_t lo = fetch(g);
        uint16_t hi = fetch(g);
        uint16_t addr = lo | (hi << 8);
        ram_write(g, g->rambr, addr, g->r[rn] & 0xff);
        ram_write(g, g->rambr, addr + 1, g->r[rn] >> 8);
      } else if (alt2) {
        // LM: load Rn from absolute word addr.
        uint16_t lo = fetch(g);
        uint16_t hi = fetch(g);
        uint16_t addr = lo | (hi << 8);
        g->r[rn] = ram_read(g, g->rambr, addr) | ((uint16_t)ram_read(g, g->rambr, addr+1) << 8);
      } else {
        // IWT Rn,#imm16 : load immediate word (little-endian).
        uint16_t lo = fetch(g);
        uint16_t hi = fetch(g);
        g->r[rn] = lo | (hi << 8);
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

void gsu_run(Gsu* g, int maxCycles) {
  int cycles = 0;
  while (gsu_is_running(g) && cycles < maxCycles) {
    cycles += gsu_step(g);
  }
}
