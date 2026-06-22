# Super FX (GSU / MARIO Chip) — Star Fox CPU↔GSU interface

Working notes for adding a Super FX emulator (`runner/src/snes/gsu.c`) so Star Fox
(a Super FX game) can run under snesrecomp. The framework recompiles the 65C816 host;
the GSU is emulated as "the rest of the silicon," like the PPU/APU. Star Fox uses the
**standard, documented Super FX register layout** — implement `gsu.c` against the
canonical GSU spec (fullsnes / bsnes / snes9x); the details below are what Star Fox
specifically relies on, confirmed from the leaked source (`/Users/colin/src/ultrastarfox`).

## Register window `$3000–$303E` (mirrored in banks $00–$3F and $80–$BF)

From `SF/INC/MREGS.INC`:

| Addr | Name | Dir | Use |
|------|------|-----|-----|
| `$3000–$301E` | `mr0`–`mr15` (16×16-bit) | R/W | GSU general regs. `mr15` = PC: **writing `mr15` launches the GSU** |
| `$3030` | `m_sfr` | R/W | Status/flags. **bit5 `$20` = GO** (1=running). bit15 = IRQ. bits0-4 = ZCSV/E |
| `$3033` | `m_bramr` | W | Backup-RAM enable |
| `$3034` | `m_pbr` | W | Program Bank Register — ROM bank GSU executes from |
| `$3036` | `m_rombr` | R | ROM Bank (current R14 read bank) |
| `$3037` | `m_cfgr` | W | Config: bit7 `$80` IRQ-disable, bit5 `$20` MS1 fast-multiply |
| `$3038` | `m_scbr` | W | Screen Base (framebuffer base >> ...) |
| `$3039` | `m_clsr` | W | Clock speed (0=10.7MHz,1=21.4MHz) |
| `$303A` | `m_scmr` | W | Screen Mode (see below) |
| `$303B` | `m_vcr` | R | Version code |
| `$303C` | `m_rambr` | R | RAM Bank |
| `$303E` | `m_cbr` | R | Cache Base |

`m_sfr` flag bits (`SF/INC/MARIOI.INC`): `mf_z=2 mf_c=4 mf_s=8 mf_v=16 mf_g=32(GO)
mf_r=64 mf_irq=$8000`.

`m_scmr` bits: `mm_scm0=1 mm_scm1=2` (color: 0/1/2 → 4/16/256-color),
`mm_sch=4` height bit, `mm_ramn=8` (RAM "nasty"/access arbitration), `mm_romn=16`
(ROM nasty), `mm_sch1=32`. Star Fox uses **256-color**, sets RAM+ROM nasty (`$18`)
during GSU runs.

## Launch / sync sequence (`SF/ASM/RAMSTUFF.ASM` `runmario_l`)

```
sta.l m_pbr          ; A = program bank
lda  mario_draw_mode
ora  #$18            ; RAM+ROM nasty
sta  m_scmr
stx  mr15            ; X = entry PC  -> LAUNCHES the GSU
.dowait
lda  m_sfr
and  #$20            ; test GO
bne  .dowait         ; busy-wait until GSU stops
sta  m_scmr          ; restore draw mode
```

Caller (`SF/ASM/MAIN.ASM`): `lda #routine>>16 : ldx #routine&WM : jsl runmario_l`.
Main 3D entry point: `mdo_3d_display` (`SF/MARIO/MDRAWLIS.MC`).

**Implication for the emulator:** the 65C816 busy-waits on the GO bit. We can run the GSU
to completion synchronously inside the `mr15` write (or step it and clear GO when it
hits STOP), so the wait loop falls through immediately. The GSU's `STOP` instruction
clears GO (and may raise the IRQ flag).

## Code & memory

- **GSU code executes from ROM in place** (no copy). `m_pbr` selects the bank;
  MARIO code lives in the ROM (`mariobegin` in `SF/BANK/BANK1.ASM`; find its physical
  address in `SYMBOLS.TXT`). Determine the exact MARIO bank extent from `SYMBOLS.TXT`
  + `BANKS.CSV` before writing `.cfg` so those banks are NOT decoded as 65C816.
- **Shared Game Pak RAM at `$70:0000`+** (`SF/INC/ALCS.INC`): `zmmempt=$700000`,
  `mmempt=$700200`, `bitmapbase=$700000`. Framebuffer `bitmap1 = 64K − gameBMPSZ`
  (`gameBMPSZ = 28*24*32`). **NB the framebuffer offset is > `$8000`**, so the
  framework's stock LoROM RAM window (banks `$70–7D`, `adr<$8000`) is insufficient —
  Super FX maps the **full** `$70–$71` banks. `gsu.c` cart integration must cover that.
- **Framebuffer**: 256-color (8bpp), tile/char-organized (28 col × 24 row × 32 B/char),
  at `$700000 + bitmap1`. The 65C816 DMAs it to VRAM each NMI
  (`SF/ASM/IRQ.ASM`: `dmaxvram 0,(bitmapbase+bitmap1),planbmpsz`).

## Per-frame flow

1. CPU builds the display list / object state in shared RAM.
2. CPU `jsl runmario_l` with `mdo_3d_display` → GSU renders 3D into the framebuffer in RAM.
3. CPU busy-waits GO, then on NMI DMAs the framebuffer → VRAM; PPU shows it.

## Integration points in the framework

- `runner/src/snes/snes.h` — add `Gsu* gsu;` to `struct Snes`.
- `runner/src/common_rtl.c` (ReadReg/WriteReg) — route `$3000–$303F` to `gsu_read/write`.
- `runner/src/snes/cart.c` — Super FX RAM window (full `$70–$71`) + ROM access shared
  with the GSU; arbitration via `m_scmr` nasty bits (loose at first).
- `runner/src/snes/snes_other.c` — enable GSU for Super FX ROMs (header `romtype $14`),
  size Game Pak RAM appropriately (header SRAM byte is `$00`; set GSU work-RAM size).
- `runner/runner.cmake` — add `src/snes/gsu.c`.

See also `docs/MSU1.md` for the existing "bolt-on coprocessor" integration pattern.
