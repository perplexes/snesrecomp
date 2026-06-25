"""Per-instruction 65C816 master-cycle estimates for the cycle-faithful clock.

The runner advances a master-cycle clock as the recompiled CPU runs, and the
scheduler fires NMI / V-IRQ / H-IRQ when the simulated beam crosses the game's
programmed scanline. For that to land at the right game-moment, each emitted
block must say roughly how many CPU cycles it represents.

This is a PROPORTIONAL estimate, not cycle-perfect. A static recompiler cannot
know runtime-dependent penalties (direct-page not page-aligned, index page
crossing, branch taken/not-taken, DMA/HDMA stalls, MVN/MVP byte counts). Those
are deliberately omitted: interrupt *edges* only need to land in roughly the
right place, and the table is calibrated as a whole (see SF_SCHED notes), not
per-opcode. Values follow the WDC 65C816 datasheet "standard" timings with the
common assumptions: DP low byte == 0 (no +1), no page cross (no +1), branch
not taken (no +1). The 8/16-bit width adjustments (`+m`, `+x`) ARE applied
because the decoder pins `m_flag`/`x_flag` per instruction.
"""

from snes65816 import (  # noqa: E402
    IMP, ACC, IMM, DP, DP_X, DP_Y, ABS, ABS_X, ABS_Y,
    LONG, LONG_X, REL, REL16, STK, INDIR, INDIR_X, INDIR_Y, INDIR_LY,
    INDIR_L, INDIR_DPX, DP_INDIR, STK_IY,
)

# --- ALU / load / store (group "read or write one operand") -----------------
# Base cycles by addressing mode. +1 when the operand is 16-bit (m=0 for the
# accumulator class, x=0 for the index class).
_ALU_MODE = {
    IMM:        2,
    DP:         3,
    DP_X:       4,
    DP_Y:       4,
    ABS:        4,
    ABS_X:      4,
    ABS_Y:      4,
    LONG:       5,
    LONG_X:     5,
    DP_INDIR:   5,   # (dp)
    INDIR_Y:    5,   # (dp),y
    INDIR_DPX:  6,   # (dp,x)
    INDIR_L:    6,   # [dp]
    INDIR_LY:   6,   # [dp],y
    STK:        4,   # d,s
    STK_IY:     7,   # (d,s),y
}

# Accumulator-width ALU/load/store ops: +1 cycle when m=0 (16-bit memory/acc).
_ALU_A = {
    'LDA', 'STA', 'ADC', 'SBC', 'AND', 'ORA', 'EOR', 'CMP', 'BIT', 'STZ',
}
# Index-width ops: +1 cycle when x=0 (16-bit index).
_ALU_X = {'LDX', 'LDY', 'STX', 'STY', 'CPX', 'CPY'}

# --- read-modify-write (shift / inc / dec / test-and-set) -------------------
# +2 cycles when m=0 (16-bit RMW reads+writes an extra byte).
_RMW_MODE = {ACC: 2, DP: 5, DP_X: 6, ABS: 6, ABS_X: 7}
_RMW = {'ASL', 'LSR', 'ROL', 'ROR', 'INC', 'DEC', 'TSB', 'TRB'}

# --- fixed-cost mnemonics (mode-independent) --------------------------------
_FIXED = {
    # implied / transfer / flag ops
    'TAX': 2, 'TXA': 2, 'TAY': 2, 'TYA': 2, 'TSX': 2, 'TXS': 2,
    'TXY': 2, 'TYX': 2, 'TCD': 2, 'TDC': 2, 'TCS': 2, 'TSC': 2,
    'DEX': 2, 'DEY': 2, 'INX': 2, 'INY': 2,
    'NOP': 2, 'XBA': 3, 'XCE': 2, 'WDM': 2,
    'CLC': 2, 'SEC': 2, 'CLD': 2, 'SED': 2, 'CLI': 2, 'SEI': 2, 'CLV': 2,
    'REP': 3, 'SEP': 3,
    # stack pushes / pulls
    'PHA': 3, 'PHX': 3, 'PHY': 3, 'PHP': 3, 'PHB': 3, 'PHK': 3, 'PHD': 4,
    'PEA': 5, 'PEI': 6, 'PER': 6,
    'PLA': 4, 'PLX': 4, 'PLY': 4, 'PLP': 4, 'PLB': 4, 'PLD': 5,
    # control transfer
    'JSR': 6, 'JSL': 8, 'RTS': 6, 'RTL': 6, 'RTI': 7,
    'BRK': 7, 'COP': 7,
    'BRL': 3,
    # block move (per-byte loop; static can't know count — nominal)
    'MVN': 7, 'MVP': 7,
    # wait/stop (spin head; nominal so it still advances the clock a little)
    'WAI': 3, 'STP': 3,
}

# Branches (relative): 2 base; taken/page-cross penalty omitted (proportional).
_BRANCH = {'BCC', 'BCS', 'BEQ', 'BNE', 'BMI', 'BPL', 'BVC', 'BVS', 'BRA'}

# Jumps by mode (the mnemonic alone doesn't disambiguate JMP vs JML variants).
_JMP_MODE = {
    ABS:     3,   # JMP abs
    LONG:    4,   # JML long
    INDIR:   5,   # JMP (abs)
    INDIR_X: 6,   # JMP (abs,x)
    INDIR_L: 6,   # JML [abs]
}

_DEFAULT = 2  # conservative fallback for anything unmodeled


def estimate_cycles(insn) -> int:
    """Return an estimated master-cycle cost for one decoded instruction.

    Robust to unknown mnemonics/modes — falls back to a small constant so the
    clock always advances rather than stalling on an unmodeled op.
    """
    mnem = insn.mnem
    mode = insn.mode
    m = getattr(insn, 'm_flag', 1)
    x = getattr(insn, 'x_flag', 1)

    if mnem in _ALU_A:
        c = _ALU_MODE.get(mode, 4)
        if m == 0:
            c += 1
        return c
    if mnem in _ALU_X:
        c = _ALU_MODE.get(mode, 4)
        if x == 0:
            c += 1
        return c
    if mnem in _RMW:
        c = _RMW_MODE.get(mode, 6)
        if mode != ACC and m == 0:
            c += 2
        return c
    if mnem in _BRANCH:
        return 2
    if mnem == 'JMP' or mnem == 'JML':
        return _JMP_MODE.get(mode, 3)
    c = _FIXED.get(mnem)
    if c is not None:
        return c
    return _DEFAULT


def estimate_block_cycles(insns) -> int:
    """Sum estimated cycles over a block's decoded instructions (each has
    `.insn`). Returns 0 for an empty block."""
    total = 0
    for di in insns:
        total += estimate_cycles(di.insn)
    return total
