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
  // Test BNE taken and not-taken.
  // IWT R1,#1 ; IWT R2,#1 ; FROM R1 ; TO R3 ; SUB R2 (R3=0, Z set) ;
  // BEQ +2 (skip the next IWT) ; IWT R4,#$BAD ; IWT R5,#$600D ; STOP
  // BEQ disp is measured from the byte AFTER the displacement.
  // Layout offsets:
  //  0: F1 01 00
  //  3: F2 01 00
  //  6: B1
  //  7: 13
  //  8: 62          (SUB R2 -> R3=0, Z=1)
  //  9: 09 dd       (BEQ, disp)  next byte after disp is offset 11
  // 11: F4 AD 0B    (IWT R4,#$0BAD)  <- skipped if branch taken
  // 14: F5 0D 60    (IWT R5,#$600D)
  // 17: 00          STOP
  // To skip the 3-byte IWT R4, disp = 3 (11 + 3 = 14).
  uint8_t prog[] = {
    0xF1, 0x01, 0x00,
    0xF2, 0x01, 0x00,
    0xB1,
    0x13,
    0x62,
    0x09, 0x03,         // BEQ +3
    0xF4, 0xAD, 0x0B,   // IWT R4,#$0BAD (should be skipped)
    0xF5, 0x0D, 0x60,   // IWT R5,#$600D
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
  printf("ALL %d CHECKS PASSED\n", g_tests);
  return 0;
}
