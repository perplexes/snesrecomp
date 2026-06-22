// Standalone unit test for the Super FX (GSU) core.
//
//   cc -std=c11 -Wall test_gsu.c gsu.c -o /tmp/test_gsu && /tmp/test_gsu
//
// No framework dependencies beyond gsu.h + the C standard library.

#include "gsu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <time.h>

// Register-window byte offsets within $3000.
#define R(n)   ((n) * 2)        // low byte of Rn
#define RHI(n) ((n) * 2 + 1)    // high byte of Rn
#define SFR_L  0x30
#define SFR_H  0x31
#define PBR    0x34
#define SCBR   0x38
#define SCMR   0x3a

static int g_tests = 0;

#define CHECK(cond) do { \
  g_tests++; \
  if (!(cond)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #cond); \
    abort(); \
  } \
} while (0)

// ----------------------------------------------------------------------
// Helper: load a tiny program into a fake ROM bank 0 and launch the GSU.
// The program lives at PBR=0, PC=0. We run to completion.
// ----------------------------------------------------------------------

static uint8_t g_rom[0x20000];
static uint8_t g_ram[0x20000];

static Gsu* make_gsu_with_program(const uint8_t* prog, int len) {
  Gsu* g = gsu_init();
  memset(g_rom, 0, sizeof(g_rom));
  memset(g_ram, 0, sizeof(g_ram));
  memcpy(g_rom, prog, len);
  gsu_set_memory(g, g_rom, sizeof(g_rom), g_ram, sizeof(g_ram));
  gsu_reset(g);
  return g;
}

// Launch: set PBR=0, PC=0 (write R15), run.
static void launch_and_run(Gsu* g) {
  gsu_write(g, PBR, 0x00);
  // Write R15 low then high; the high-byte write sets GO.
  gsu_write(g, R(15), 0x00);
  gsu_write(g, RHI(15), 0x00);
  CHECK(gsu_is_running(g)); // GO set after R15 high-byte write
  gsu_run(g, 100000);
  CHECK(!gsu_is_running(g)); // GO cleared at STOP
}

// ======================================================================

static void test_register_window(void) {
  Gsu* g = gsu_init();
  gsu_reset(g);

  // R0..R15 read/write through the window.
  for (int i = 0; i < 16; i++) {
    uint16_t v = (uint16_t)(0x1000 + i * 0x111);
    gsu_write(g, R(i), v & 0xff);
    if (i != 15) gsu_write(g, RHI(i), v >> 8);
    if (i != 15) {
      CHECK(gsu_read(g, R(i)) == (v & 0xff));
      CHECK(gsu_read(g, RHI(i)) == (v >> 8));
      CHECK(gsu_get_reg(g, i) == v);
    }
  }

  // PBR, SCBR, SCMR.
  gsu_write(g, PBR, 0x42);
  CHECK(gsu_read(g, PBR) == 0x42);
  gsu_write(g, SCBR, 0x60);
  CHECK(gsu_read(g, SCBR) == 0x60);
  gsu_write(g, SCMR, 0x18);
  CHECK(gsu_read(g, SCMR) == 0x18);

  // SFR low/high readable.
  gsu_write(g, SFR_L, GSU_SFR_Z);
  CHECK((gsu_read(g, SFR_L) & GSU_SFR_Z) == GSU_SFR_Z);

  gsu_free(g);
  printf("  test_register_window OK\n");
}

static void test_go_bit_and_stop(void) {
  // Program: NOP, STOP.
  uint8_t prog[] = { 0x01, 0x00 };
  Gsu* g = make_gsu_with_program(prog, sizeof(prog));

  gsu_write(g, PBR, 0x00);
  gsu_write(g, R(15), 0x00);
  CHECK(!gsu_is_running(g)); // low-byte write does not launch
  gsu_write(g, RHI(15), 0x00);
  CHECK(gsu_is_running(g));   // GO set on R15 high-byte write
  CHECK((gsu_get_sfr(g) & GSU_SFR_GO) != 0);

  gsu_run(g, 1000);
  CHECK(!gsu_is_running(g));  // STOP cleared GO
  CHECK((gsu_get_sfr(g) & GSU_SFR_IRQ) != 0); // STOP raised IRQ

  gsu_free(g);
  printf("  test_go_bit_and_stop OK\n");
}

static void test_iwt_immediate_load(void) {
  // IWT R3,#$1234 ; STOP
  // IWT Rn = 0xF0|n ; then lo, hi.
  uint8_t prog[] = { 0xF3, 0x34, 0x12, 0x00 };
  Gsu* g = make_gsu_with_program(prog, sizeof(prog));
  launch_and_run(g);
  CHECK(gsu_get_reg(g, 3) == 0x1234);
  gsu_free(g);
  printf("  test_iwt_immediate_load OK\n");
}

static void test_ibt_immediate_byte(void) {
  // IBT R5,#$FF (sign-extended -> 0xFFFF) ; STOP
  // IBT Rn = 0xA0|n ; then imm byte.
  uint8_t prog[] = { 0xA5, 0xFF, 0x00 };
  Gsu* g = make_gsu_with_program(prog, sizeof(prog));
  launch_and_run(g);
  CHECK(gsu_get_reg(g, 5) == 0xFFFF);
  gsu_free(g);
  printf("  test_ibt_immediate_byte OK\n");
}

static void test_add(void) {
  // IWT R1,#$0010 ; IWT R2,#$0005 ; FROM R1 ; TO R3 ; ADD R2 ; STOP
  // FROM Rn = 0xB0|n (sets Sreg). TO Rn = 0x10|n (sets Dreg). ADD Rn=0x50|n.
  uint8_t prog[] = {
    0xF1, 0x10, 0x00,   // IWT R1,#$0010
    0xF2, 0x05, 0x00,   // IWT R2,#$0005
    0xB1,               // FROM R1 (Sreg=1)
    0x13,               // TO R3   (Dreg=3)
    0x52,               // ADD R2  -> R3 = R1 + R2
    0x00,               // STOP
  };
  Gsu* g = make_gsu_with_program(prog, sizeof(prog));
  launch_and_run(g);
  CHECK(gsu_get_reg(g, 3) == 0x0015);
  gsu_free(g);
  printf("  test_add OK\n");
}

static void test_sub(void) {
  // IWT R1,#$0020 ; IWT R2,#$0008 ; FROM R1 ; TO R4 ; SUB R2 ; STOP
  // SUB Rn = 0x60|n.
  uint8_t prog[] = {
    0xF1, 0x20, 0x00,
    0xF2, 0x08, 0x00,
    0xB1,               // FROM R1
    0x14,               // TO R4
    0x62,               // SUB R2 -> R4 = R1 - R2 = 0x18
    0x00,
  };
  Gsu* g = make_gsu_with_program(prog, sizeof(prog));
  launch_and_run(g);
  CHECK(gsu_get_reg(g, 4) == 0x0018);
  // CY set (no borrow) since 0x20 >= 0x08.
  CHECK((gsu_get_sfr(g) & GSU_SFR_CY) != 0);
  gsu_free(g);
  printf("  test_sub OK\n");
}

static void test_to_from_with_mechanics(void) {
  // Verify default Sreg=Dreg=R0 and TO/FROM prefix selection.
  // IWT R0,#$0001 ; IWT R7,#$00F0 ; FROM R7 ; ADD R0 ; STOP
  //   With no TO prefix, Dreg=R0 default. Sreg=R7 via FROM.
  //   R0 = R7 + R0 = 0xF0 + 0x01 = 0xF1
  uint8_t prog[] = {
    0xF0, 0x01, 0x00,   // IWT R0,#1
    0xF7, 0xF0, 0x00,   // IWT R7,#$F0
    0xB7,               // FROM R7
    0x50,               // ADD R0  (Dreg default R0)
    0x00,
  };
  Gsu* g = make_gsu_with_program(prog, sizeof(prog));
  launch_and_run(g);
  CHECK(gsu_get_reg(g, 0) == 0x00F1);

  // WITH mechanics: WITH R5 sets Sreg=Dreg=R5, then an op uses R5 for both.
  // IWT R5,#$0003 ; WITH R5 ; ADD R5 ; STOP  -> R5 = R5 + R5 = 6
  uint8_t prog2[] = {
    0xF5, 0x03, 0x00,   // IWT R5,#3
    0x25,               // WITH R5 (0x20|5) Sreg=Dreg=5, B set
    0x55,               // ADD R5  -> R5 = R5 + R5
    0x00,
  };
  Gsu* g2 = make_gsu_with_program(prog2, sizeof(prog2));
  launch_and_run(g2);
  CHECK(gsu_get_reg(g2, 5) == 0x0006);

  gsu_free(g); gsu_free(g2);
  printf("  test_to_from_with_mechanics OK\n");
}

static void test_branch(void) {
  // BEQ taken, skipping an instruction — WITH the GSU branch delay slot. The
  // instruction immediately after the branch (the delay slot) ALWAYS executes,
  // so to skip code a NOP sits in the delay slot and the skipped instruction
  // follows it. The branch displacement is measured from the byte after the
  // displacement (= the delay slot).
  //  0: F1 01 00   IWT R1,#1
  //  3: F2 01 00   IWT R2,#1
  //  6: B1         FROM R1        (sreg=R1)
  //  7: 13         TO R3          (dreg=R3)
  //  8: 62         SUB R2         (R3 = R1-R2 = 0, Z=1)
  //  9: 09 04      BEQ +4   (disp at 10; delay slot at 11; target = 11 + 4 = 15)
  // 11: 01         NOP            <- delay slot (always runs; harmless)
  // 12: F4 AD 0B   IWT R4,#$0BAD  <- skipped when taken
  // 15: F5 0D 60   IWT R5,#$600D  <- branch target
  // 18: 00         STOP
  uint8_t prog[] = {
    0xF1, 0x01, 0x00,
    0xF2, 0x01, 0x00,
    0xB1,
    0x13,
    0x62,
    0x09, 0x04,         // BEQ +4 (target = delay-slot addr 11 + 4 = 15)
    0x01,               // NOP (delay slot)
    0xF4, 0xAD, 0x0B,   // IWT R4,#$0BAD (skipped when taken)
    0xF5, 0x0D, 0x60,   // IWT R5,#$600D (target)
    0x00,
  };
  Gsu* g = make_gsu_with_program(prog, sizeof(prog));
  launch_and_run(g);
  CHECK(gsu_get_reg(g, 3) == 0);
  CHECK(gsu_get_reg(g, 4) == 0);       // skipped, stays 0
  CHECK(gsu_get_reg(g, 5) == 0x600D);  // executed
  gsu_free(g);
  printf("  test_branch OK\n");
}

// The GSU is pipelined: the instruction immediately following a branch (the
// delay slot) executes before control reaches the target, even when the branch
// is taken. Regression test — without delay-slot modeling, Star Fox's GSU 3D
// loops use the wrong register-prefix context and never converge.
static void test_branch_delay_slot(void) {
  // Layout (BRA = opcode + 1 displacement byte; delay slot is the byte after):
  //  0: 05 03   BRA +3   (disp at 1; delay slot at 2; target = 2 + 3 = 5)
  //  2: D1      INC R1    <- delay slot: ALWAYS runs (R1 0->1)
  //  3: F2 AD   IWT R2,.. <- jumped over (never executed; bytes are dead)
  //  5: D3      INC R3    <- branch target (R3 0->1)
  //  6: 00      STOP
  uint8_t prog[] = {
    0x05, 0x03,         // 0: BRA +3
    0xD1,               // 2: INC R1 (delay slot)
    0xF2, 0xAD,         // 3: IWT R2 opcode + imm (skipped)
    0xD3,               // 5: INC R3 (branch target)
    0x00,               // 6: STOP
  };
  Gsu* g = make_gsu_with_program(prog, sizeof(prog));
  launch_and_run(g);
  CHECK(gsu_get_reg(g, 1) == 1);   // delay slot executed
  CHECK(gsu_get_reg(g, 2) == 0);   // skipped by the branch
  CHECK(gsu_get_reg(g, 3) == 1);   // branch target executed
  gsu_free(g);
  printf("  test_branch_delay_slot OK\n");
}

static void test_iwt_r15_delay_slot(void) {
  // Writing R15 via IWT (= `miwt pc,addr`, the jump emitted by the `mcall` /
  // `mlbeq` macros) is a DEFERRED branch like JMP/BRA: the instruction in the
  // delay slot (the byte right after the IWT immediate) must execute before
  // control transfers. MDECRUNCH's `mcall mgetbits` puts `with rd2` in that
  // delay slot so the callee's opening `sub rd2` clears rd2; skipping it made
  // rd2 accumulate garbage and the decompressor never terminated (boot hang).
  //   0: FF 06 00  IWT R15,#6  (imm at 1-2; delay slot at 3; target = 6)
  //   3: D1        INC R1      <- delay slot: ALWAYS runs (R1 0->1)
  //   4: D2        INC R2      <- jumped over (dead)
  //   5: 01        NOP         <- jumped over (dead)
  //   6: D3        INC R3      <- branch target (R3 0->1)
  //   7: 00        STOP
  uint8_t prog[] = {
    0xFF, 0x06, 0x00,   // 0: IWT R15,#6
    0xD1,               // 3: INC R1 (delay slot)
    0xD2,               // 4: INC R2 (skipped)
    0x01,               // 5: NOP    (skipped)
    0xD3,               // 6: INC R3 (target)
    0x00,               // 7: STOP
  };
  Gsu* g = make_gsu_with_program(prog, sizeof(prog));
  launch_and_run(g);
  CHECK(gsu_get_reg(g, 1) == 1);   // delay slot executed
  CHECK(gsu_get_reg(g, 2) == 0);   // skipped by the jump
  CHECK(gsu_get_reg(g, 3) == 1);   // jump target executed
  gsu_free(g);
  printf("  test_iwt_r15_delay_slot OK\n");
}

static void test_nop(void) {
  uint8_t prog[] = { 0x01, 0x01, 0x01, 0x00 };
  Gsu* g = make_gsu_with_program(prog, sizeof(prog));
  launch_and_run(g);
  CHECK(!gsu_is_running(g));
  gsu_free(g);
  printf("  test_nop OK\n");
}

static void test_plot(void) {
  // Set up: SCBR=0 (framebuffer at RAM bank 0, offset 0), 256-color mode.
  // Plot a pixel at (R1=0, R2=0) with COLOR=0xA5, then RPIX it back.
  //
  // Program:
  //   IWT R1,#0     ; plot X
  //   IWT R2,#0     ; plot Y
  //   IWT R6,#$A5   ; value to load into COLOR via FROM R6 / COLOR
  //   FROM R6 ; COLOR        (COLOR = R6 low byte = 0xA5)
  //   PLOT                   (writes pixel; R1++)
  //   ALT1 ; FROM <n> ; TO R7 ; RPIX   -> reads back pixel at (R1,R2)
  // But RPIX reads at current (R1,R2); PLOT incremented R1 to 1. Reset R1=0
  // before RPIX.
  //
  // COLOR opcode = 0x4E (no ALT). PLOT = 0x4C (no ALT). RPIX = ALT1 + 0x4C.
  uint8_t prog[] = {
    0xF1, 0x00, 0x00,   // IWT R1,#0
    0xF2, 0x00, 0x00,   // IWT R2,#0
    0xF6, 0xA5, 0x00,   // IWT R6,#$A5
    0xB6,               // FROM R6 (Sreg=6)
    0x4E,               // COLOR -> COLOR = 0xA5
    0x4C,               // PLOT at (0,0); R1 -> 1
    0xF1, 0x00, 0x00,   // IWT R1,#0  (reset X)
    0x17,               // TO R7 (Dreg=7)
    0x3D,               // ALT1
    0x4C,               // RPIX -> R7 = pixel color at (0,0)
    0x00,               // STOP
  };
  Gsu* g = make_gsu_with_program(prog, sizeof(prog));
  // SCBR=0, SCMR 256-color (set scm bits to 256: scm0|scm1 cleared-> docs
  // say 0/1/2 -> 4/16/256; value 0 in our height calc is fine for layout).
  gsu_write(g, SCBR, 0x00);
  gsu_write(g, SCMR, 0x00);
  launch_and_run(g);

  // RPIX should read back the plotted color.
  CHECK(gsu_get_reg(g, 7) == 0xA5);

  // Also verify the framebuffer bytes directly. For pixel (0,0), color 0xA5
  // (bits 1010 0101), bit (0x80) is set in planes where the color bit is 1.
  // Planes 0..7 map to bytes {0,1,16,17,32,33,48,49}. color bit set ->
  // that byte has 0x80.
  const int plane_byte[8] = { 0, 1, 16, 17, 32, 33, 48, 49 };
  for (int plane = 0; plane < 8; plane++) {
    int expect = (0xA5 & (1 << plane)) ? 0x80 : 0x00;
    CHECK(g_ram[plane_byte[plane]] == expect);
  }

  gsu_free(g);
  printf("  test_plot OK\n");
}

// GETB reads the ROM data byte at ROMBR:R14. Regression test for the bug where
// the ROMDR latch was never populated, so GETB/GETC always returned 0 (the GSU
// processed all-zero model data and the 3D render loop never terminated).
static void test_getb_rom_data(void) {
  // FE 00 02   IWT R14,#$0200   ; point the ROM data pointer at $0200
  // EF         GETB             ; R0 = rom[ROMBR:$0200]
  // FE 01 02   IWT R14,#$0201   ; advance the pointer
  // 11         TO R1            ; direct next result to R1
  // EF         GETB             ; R1 = rom[ROMBR:$0201]
  // 00         STOP
  uint8_t prog[] = { 0xFE, 0x00, 0x02, 0xEF, 0xFE, 0x01, 0x02, 0x11, 0xEF, 0x00 };
  Gsu* g = make_gsu_with_program(prog, sizeof(prog));
  // ROMBR defaults to 0 after reset; place known data bytes in bank 0.
  g_rom[0x0200] = 0x12;
  g_rom[0x0201] = 0x34;
  launch_and_run(g);
  CHECK(gsu_get_reg(g, 0) == 0x12);  // first GETB
  CHECK(gsu_get_reg(g, 1) == 0x34);  // GETB after R14 advanced (on-demand refetch)
  gsu_free(g);
  printf("  test_getb_rom_data OK\n");
}

// ======================================================================
// Performance tests. The GSU core runs Star Fox's per-frame 3D pipeline
// (MDECRUNCH decompress + transform/project/plot), so its raw instruction
// and PLOT throughput bound the achievable frame rate. These benchmarks
// time a large fixed workload and assert a conservative throughput floor —
// they guard against catastrophic perf regressions (e.g. accidentally
// reintroducing a per-instruction malloc or O(n) rescan) rather than
// pin an exact number, and they print the measured rate for tracking.
// ----------------------------------------------------------------------

// Relaunch an already-loaded GSU program from PBR:0 / PC:0 and run to STOP.
static void relaunch_and_run(Gsu* g) {
  gsu_write(g, PBR, 0x00);
  gsu_write(g, R(15), 0x00);
  gsu_write(g, RHI(15), 0x00);
  gsu_run(g, 1 << 30);  // cap far above the workload; returns at STOP
}

static void perf_alu_throughput(void) {
  // A straight-line block of INC R1 (1 byte, real ALU work, no operand
  // fetch) terminated by STOP. Running it many times measures sustained
  // interpreter dispatch + ALU throughput.
  enum { BODY = 4096, RUNS = 4000 };
  static uint8_t prog[BODY + 1];
  memset(prog, 0xD1, BODY);   // INC R1
  prog[BODY] = 0x00;          // STOP
  Gsu* g = make_gsu_with_program(prog, BODY + 1);

  clock_t t0 = clock();
  for (int r = 0; r < RUNS; r++)
    relaunch_and_run(g);
  double secs = (double)(clock() - t0) / CLOCKS_PER_SEC;

  double insns = (double)BODY * RUNS;
  double mips = insns / (secs > 0 ? secs : 1e-9) / 1e6;
  printf("  [perf] ALU throughput: %.1f M insn/s (%.0f insns in %.3fs)\n", mips, insns, secs);
  CHECK(secs >= 0.0);     // ran to completion
  CHECK(mips > 1.0);      // floor: catches catastrophic regressions only
  gsu_free(g);
}

static void perf_plot_throughput(void) {
  // PLOT is the rendering-critical op: it writes the framebuffer (bitplane
  // scatter) and advances R1. Benchmark a block of back-to-back PLOTs after
  // setting COLOR once. R1 wraps within the line; the framebuffer writes
  // exercise the same scatter path Star Fox's rasterizer hits per pixel.
  enum { PLOTS = 2048, RUNS = 700 };
  static uint8_t prog[16 + PLOTS + 1];
  int n = 0;
  prog[n++] = 0xF1; prog[n++] = 0x00; prog[n++] = 0x00;  // IWT R1,#0  (X)
  prog[n++] = 0xF2; prog[n++] = 0x00; prog[n++] = 0x00;  // IWT R2,#0  (Y)
  prog[n++] = 0xF6; prog[n++] = 0xA5; prog[n++] = 0x00;  // IWT R6,#$A5
  prog[n++] = 0xB6;                                       // FROM R6 (Sreg=6)
  prog[n++] = 0x4E;                                       // COLOR = 0xA5
  for (int i = 0; i < PLOTS; i++) prog[n++] = 0x4C;       // PLOT (R1++)
  prog[n++] = 0x00;                                       // STOP
  Gsu* g = make_gsu_with_program(prog, n);
  gsu_write(g, SCBR, 0x00);
  gsu_write(g, SCMR, 0x00);

  clock_t t0 = clock();
  for (int r = 0; r < RUNS; r++)
    relaunch_and_run(g);
  double secs = (double)(clock() - t0) / CLOCKS_PER_SEC;

  double plots = (double)PLOTS * RUNS;
  double mpps = plots / (secs > 0 ? secs : 1e-9) / 1e6;
  printf("  [perf] PLOT throughput: %.1f M plot/s (%.0f plots in %.3fs)\n", mpps, plots, secs);
  CHECK(mpps > 0.5);      // floor: catches catastrophic regressions only
  gsu_free(g);
}

int main(void) {
  printf("GSU unit tests:\n");
  test_register_window();
  test_go_bit_and_stop();
  test_iwt_immediate_load();
  test_ibt_immediate_byte();
  test_add();
  test_sub();
  test_to_from_with_mechanics();
  test_branch();
  test_nop();
  test_plot();
  test_getb_rom_data();
  test_branch_delay_slot();
  test_iwt_r15_delay_slot();
  printf("GSU performance tests:\n");
  perf_alu_throughput();
  perf_plot_throughput();
  printf("ALL %d CHECKS PASSED\n", g_tests);
  return 0;
}
