// Super FX (GSU / MARIO Chip 1) emulator core for snesrecomp.
//
// The GSU is emulated as "the rest of the silicon" alongside the PPU/APU.
// The recompiled 65C816 host writes the GSU register window ($3000-$303F),
// writing R15 (mr15) launches the chip, and the host busy-waits on the GO
// bit in SFR. We run the GSU synchronously to completion (until STOP) so the
// wait loop falls through.
//
// Implemented against the canonical Super FX spec (fullsnes / bsnes fxemu /
// snes9x). Opcode numbers are cited inline in gsu.c.
//
// The header deliberately depends only on stdint/stdbool so the standalone
// unit test (test_gsu.c) needs no framework headers.

#ifndef GSU_H
#define GSU_H

#include <stdint.h>
#include <stdbool.h>

typedef struct Gsu Gsu;

// SFR (Status/Flag Register, $3030/$3031) bit masks.
enum {
  GSU_SFR_Z   = 0x0002, // Zero
  GSU_SFR_CY  = 0x0004, // Carry
  GSU_SFR_S   = 0x0008, // Sign
  GSU_SFR_OV  = 0x0010, // Overflow
  GSU_SFR_GO  = 0x0020, // GO: 1 = GSU running
  GSU_SFR_R   = 0x0040, // R14 ROM-read pending
  GSU_SFR_ALT1 = 0x0100,
  GSU_SFR_ALT2 = 0x0200,
  GSU_SFR_IL  = 0x0400, // immediate low latch
  GSU_SFR_IH  = 0x0800, // immediate high latch
  GSU_SFR_B   = 0x1000, // WITH/prefix "B" flag
  GSU_SFR_IRQ = 0x8000, // IRQ raised (e.g. on STOP)
};

Gsu*    gsu_init(void);
void    gsu_free(Gsu* gsu);
void    gsu_reset(Gsu* gsu);

// Provide ROM/RAM base pointers + sizes (cart Game Pak ROM and RAM).
void    gsu_set_memory(Gsu* gsu, uint8_t* rom, uint32_t romSize,
                       uint8_t* ram, uint32_t ramSize);

// Register window access. `reg` is masked to the $3000-$33FF window; the
// low byte selects the register/byte. R15 write launches the GSU.
uint8_t gsu_read(Gsu* gsu, uint16_t reg);
void    gsu_write(Gsu* gsu, uint16_t reg, uint8_t val);

// Step the GSU. Runs until STOP/GO cleared or `maxCycles` instructions
// consumed (whichever first). Pass a large value to run to completion.
void    gsu_run(Gsu* gsu, int maxCycles);

// True while the GO bit is set.
bool    gsu_is_running(Gsu* gsu);

// Test/introspection helpers (used by the unit test).
uint16_t gsu_get_reg(Gsu* gsu, int n);
void     gsu_set_reg(Gsu* gsu, int n, uint16_t val);
uint16_t gsu_get_sfr(Gsu* gsu);

#endif // GSU_H
