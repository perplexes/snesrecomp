"""snesrecomp.recompiler.v2.decoder

Worklist-driven 65816 decoder keyed by (PC, M, X) entry state.

REPLACES THE V1 DECODE BUG: v1's `decode_func` (recomp.py:52-354) tracks
M/X as linear scalars and stores branch-target mode hints in
`pending_flags: Dict[PC, (m, x)]` with explicit last-writer-wins overwrite
(recomp.py:298-300 comment makes this explicit). When two predecessors
reach the same PC with different (m, x), one is silently dropped and that
PC ends up decoded with the wrong mode — which is invalid for 65816
because variable-length immediate operands (LDA #imm in M=1 vs M=0) are
2 bytes vs 3 bytes, so the dropped mode can corrupt every subsequent
instruction's PC offset.

In v2, every instruction is identified by `DecodeKey(pc, m, x)`. Two
predecessors with different mode states produce two distinct
DecodedInsn records at the same PC — both are preserved. Downstream
(v2 cfg / IR / codegen) treats them as two separate blocks.

The opcode table in `snes65816.py` and the per-instruction
`decode_insn(rom, off, pc, bank, m, x)` helper are reused as-is — they
already correctly compute variable-length immediates *given* an (m, x)
input. The bug was always in the v1 caller, not in `decode_insn`.

Public API:
    decode_function(rom, bank, start, entry_m, entry_x, *, end=None)
        -> FunctionDecodeGraph
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import sys
import pathlib

# Allow `from snesrecomp.recompiler.v2 import ...` and standalone test imports.
_THIS_DIR = pathlib.Path(__file__).resolve().parent
_RECOMPILER_DIR = _THIS_DIR.parent
if str(_RECOMPILER_DIR) not in sys.path:
    sys.path.insert(0, str(_RECOMPILER_DIR))

from snes65816 import (  # noqa: E402
    decode_insn, lorom_offset, Insn,
    ABS, INDIR, INDIR_X, LONG, IMM,
)


def addr24(bank: int, pc: int) -> int:
    """Pack bank + 16-bit PC into a 24-bit address (matches Insn.addr)."""
    return ((bank & 0xFF) << 16) | (pc & 0xFFFF)


def _dispatch_target_is_padding(rom: bytes, bank: int, pc16: int,
                                window: int = 16) -> bool:
    """Return True iff the dispatch-table entry's target bytes look like
    unmapped ROM padding (all $FF) or cleared region (all $00).

    Used by the auto-detected dispatch-table reader to terminate a
    table when an entry points into bytes that can't be the start of
    any real handler. SMW's PLA/PLY-indirect-JMP dispatchers
    occasionally have shorter true tables than the auto-detector reads
    (the table's actual count is implicit in the asm code that loads
    the index), and the trailing entries fall on data/padding bytes
    that we then mistakenly auto-promote into phantom functions.

    A target whose first 16 bytes are all $FF is in unmapped ROM
    padding (between real ROM regions, post-end-of-bank, etc.). A
    target whose first 16 bytes are all $00 is similarly suspect.
    Real 65816 code at the target's entry point produces non-uniform
    byte sequences (mix of opcodes + operands).

    NOT a heuristic for code/data classification in general — only
    used to STOP a dispatch table when an obviously-invalid entry is
    encountered. Real code at the target falls through to the
    standard accept path.
    """
    try:
        off = lorom_offset(bank, pc16)
    except AssertionError:
        return True
    if off + window > len(rom):
        return True
    blob = rom[off:off + window]
    if all(b == 0xFF for b in blob):
        return True
    if all(b == 0x00 for b in blob):
        return True
    return False


def _addr_in_data_regions(data_regions, bank: int, pc16: int) -> bool:
    """Return True iff (bank, pc16) is inside any cfg-declared
    `data_region <bank> <start> <end>` range.

    cfg `data_region` directives encode a real ROM fact: this byte
    range is a data table, not executable code. Used by the dispatch-
    table reader to halt at entries whose targets land inside data,
    and by the auto-promote pass to refuse synthesizing function
    entries inside data ranges. The classic case is a JSL dispatcher
    whose table overruns into a sibling data table — without the
    cfg fact the decoder can't tell those bytes apart from real
    handlers.

    `data_regions` is the list[tuple[int, int, int]] of (bank, start,
    end_exclusive) tuples produced by cfg_loader. None / empty list
    is a no-op (returns False).
    """
    if not data_regions:
        return False
    pc16 &= 0xFFFF
    bank &= 0xFF
    for (b, s, e) in data_regions:
        if (b & 0xFF) != bank:
            continue
        if (s & 0xFFFF) <= pc16 < (e & 0xFFFF):
            return True
    return False


@dataclass(frozen=True)
class DecodeKey:
    """Identifies a decoded instruction by 24-bit address + entry M/X.

    Two DecodeKeys are equal iff (pc, m, x) all match. Same `pc` with
    different `m` or `x` is two distinct keys → two distinct decoded
    instances in the graph.
    """
    pc: int   # 24-bit ((bank << 16) | local_pc)
    m: int    # entry M flag, 0 or 1
    x: int    # entry X flag, 0 or 1


@dataclass
class DecodedInsn:
    """One instruction decoded at one specific (pc, m, x) entry state."""
    key: DecodeKey
    insn: Insn               # the underlying snes65816.Insn (m_flag/x_flag set to entry m/x)
    successors: List[DecodeKey]


@dataclass
class SuppressedIndirectCall:
    """Bookkeeping entry for a JSR (abs,X) site whose fall-through edge
    was severed because cfg has no `indirect_call_table` directive
    authorising it.

    cfg-required-dispatch-or-kill rule (2026-05-03): the v2 decoder
    refuses to follow the fall-through of an indirect JSR (a,X) when
    cfg hasn't declared a static dispatch table for it. The insn is
    still placed in the graph (so predecessors' successor edges
    resolve), but with successors=[] — that severs the post-JSR
    decode chain so phantom M=0 paths through SMW's SMC-dispatch byte
    sequences don't pollute downstream codegen.

    Each suppressed site is recorded here for the build report. Any
    reach of `site_pc24` at runtime is caught by the always-armed
    phantom-PC trap (runner/src/cpu_trace.c).
    """
    site_pc24: int
    table_base: int
    function_entry_pc24: int
    entry_m: int
    entry_x: int


@dataclass
class DispatchTargetSuppressed:
    """Bookkeeping for a dispatch-table entry the decoder REFUSED to
    accept because the target lands inside a cfg `data_region` (an
    explicit ROM-structure fact saying "these bytes are data, not
    code"). The dispatch table is truncated at this entry; the
    target never becomes a callable handler. Recorded so the build
    report can list every suppression — never silent.
    """
    site_pc24: int       # PC of the dispatcher JSL/JML
    target_pc24: int     # 24-bit address of the rejected entry
    reason: str          # 'data_region' (extensible)
    table_index: int     # 0-based index of the entry that triggered the stop


@dataclass
class ConstZFold:
    """Bookkeeping entry for a BEQ/BNE rewritten to an unconditional Goto
    by `_apply_constant_z_fold`. Recorded for the build report so each
    fold is visible/auditable rather than silently absorbed.
    """
    branch_pc24: int          # PC of the BEQ/BNE
    prev_pc24: int            # PC of the preceding LDA/LDX/LDY #imm
    branch_mnem: str          # 'BEQ' | 'BNE'
    prev_mnem: str            # 'LDA' | 'LDX' | 'LDY'
    prev_imm: int             # masked immediate value used for Z
    width_bits: int           # 8 or 16 (op width at the load)
    z_value: int              # 0 or 1
    taken_kind: str           # 'jump' (live edge is the explicit target)
                              # or 'fall' (live edge is fall-through PC)
    live_pc24: int            # surviving successor's 24-bit PC
    dead_pc24: int            # pruned successor's 24-bit PC
    func_entry_pc24: int      # decode_function's entry PC for context
    entry_m: int
    entry_x: int


@dataclass
class FunctionDecodeGraph:
    """Output of `decode_function` for one function entry.

    Attributes:
        entry: the DecodeKey we started at.
        insns: dict keyed by DecodeKey. Two entries may share `key.pc`
            iff they have different `key.m` or `key.x` — that means the
            same PC was decoded twice, once per reaching mode-state, and
            both are preserved. (This is the central correctness fix.)
        suppressed_indirect_calls: list of JSR (abs,X) sites whose
            fall-through edge was severed because cfg has no
            `indirect_call_table` authorisation. See class
            SuppressedIndirectCall above.
        const_z_folds: list of BEQ/BNE rewrites by the constant-Z fold
            post-pass. Each entry records the original branch + the
            statically-proven Z + the surviving and pruned edges.
    """
    entry: DecodeKey
    insns: Dict[DecodeKey, DecodedInsn] = field(default_factory=dict)
    suppressed_indirect_calls: List[SuppressedIndirectCall] = field(default_factory=list)
    const_z_folds: List[ConstZFold] = field(default_factory=list)
    dispatch_targets_suppressed: List[DispatchTargetSuppressed] = field(default_factory=list)

    def keys_at_pc(self, pc24: int) -> List[DecodeKey]:
        """Return all DecodeKeys with this 24-bit PC (across entry mode states)."""
        return [k for k in self.insns if k.pc == pc24]

    def insns_at_pc(self, pc24: int) -> List[DecodedInsn]:
        return [self.insns[k] for k in self.keys_at_pc(pc24)]


# Mnemonics with no fall-through successor.
_TERMINATORS = frozenset({'RTS', 'RTL', 'RTI', 'STP', 'WAI', 'BRK'})

# Mnemonics with two successors: fall-through AND taken-branch target.
_COND_BRANCHES = frozenset({'BPL', 'BMI', 'BVC', 'BVS', 'BCC', 'BCS', 'BNE', 'BEQ'})


def post_mx(insn: Insn, in_m: int, in_x: int) -> Tuple[int, int]:
    """Compute (m, x) AFTER executing `insn`, given entry (in_m, in_x).

    REP/SEP clear/set M and X bits independently per the operand bitmask.
    Other instructions don't touch M/X (XCE, PLP, RTI are unmodeled at
    this layer — they keep the entry mode; later phases may refine).
    """
    if insn.mnem == 'REP':
        m = 0 if (insn.operand & 0x20) else in_m
        x = 0 if (insn.operand & 0x10) else in_x
        return m, x
    if insn.mnem == 'SEP':
        m = 1 if (insn.operand & 0x20) else in_m
        x = 1 if (insn.operand & 0x10) else in_x
        return m, x
    return in_m, in_x


def _successors(insn: Insn, key: DecodeKey, bank: int,
                callee_exit_mx: Optional[Dict] = None) -> List[DecodeKey]:
    """Compute successor DecodeKeys for one decoded instruction.

    Returns plain DecodeKey list (kind-agnostic) for callers that only
    need successors. See `_labeled_successors` for the (key, kind)
    variant used by `decode_function`'s end: gating logic.
    """
    return [k for (k, _kind) in
            _labeled_successors(insn, key, bank,
                                callee_exit_mx=callee_exit_mx)]


def _labeled_successors(insn: Insn, key: DecodeKey, bank: int,
                        callee_exit_mx: Optional[Dict] = None):
    """Compute (DecodeKey, edge_kind) tuples for one decoded instruction.

    `edge_kind` is one of:
        'jump'        — control-flow edge whose TARGET is named explicitly
                        in the insn (BRA/BRL/cond-branch-taken/JMP-ABS).
                        These edges may cross the cfg-declared end:
                        boundary because the asm explicitly transfers
                        there — the original routine's lifetime extends
                        across them, even though `end:` says the next
                        cfg function starts at that PC.
        'fall'        — natural fall-through to the next instruction
                        (linear, JSR/JSL-after-call, cond-branch-not-
                        taken). These edges respect end:; falling
                        through past end: would pull the next function's
                        body into this one and is forbidden.
        terminator    -> [] (no successors)
        JMP INDIR/(X) -> [] (table-driven; caller's job)
        JMP LONG/JML  -> [] (cross-bank; caller's job)

    The distinction matters for the inline-cross-fn-blocks model
    (control-flow correctness fix, 2026-05-02): a BRA into a label past
    end: must IMPORT that label's blocks into the current function so
    PHB/PLB pairs and other stack-lifetime invariants stay matched
    within one C function scope. Treating arbitrary jump targets as new
    C functions (the prior auto-promote behavior) split asm routines
    across multiple C bodies and stranded their PHBs without their
    matching PLBs — root cause of DB=$C0 at dispatch entry.
    """
    post_m, post_x = post_mx(insn, key.m, key.x)
    pc = insn.addr & 0xFFFF
    next_pc = (pc + insn.length) & 0xFFFF

    mnem = insn.mnem

    if mnem in _TERMINATORS:
        return []

    if mnem in ('BRA', 'BRL'):
        return [(DecodeKey(addr24(bank, insn.operand), post_m, post_x), 'jump')]

    if mnem in _COND_BRANCHES:
        return [
            (DecodeKey(addr24(bank, next_pc), post_m, post_x), 'fall'),
            (DecodeKey(addr24(bank, insn.operand), post_m, post_x), 'jump'),
        ]

    if mnem == 'JMP':
        if insn.mode == ABS:
            return [(DecodeKey(addr24(bank, insn.operand), post_m, post_x), 'jump')]
        # INDIR / INDIR_X (table-dispatch) and LONG (cross-bank) — no
        # static successors at this layer.
        return []

    # Long-jump (JML) is decoded as JMP+LONG above; JSL is its own mnem.
    # Both are cross-routine calls; only the fall-through (return site)
    # is decoded into THIS function. The callee is a separate cfg entry.
    #
    # If `callee_exit_mx` provides this callee's exit (m, x) under the
    # entry variant we're calling with, use it for the fall-through key.
    # Without that, we'd assume m/x are preserved across the JSR — wrong
    # whenever the callee has an internal SEP/REP that doesn't restore
    # before returning (e.g. SMW's $00:F465 sets m=1 via SEP #$20,
    # leaving caller in m=1 even though caller had m=0 pre-call). The
    # decoder previously kept caller's (m, x), causing it to mis-decode
    # subsequent operand widths and synthesise phantom branch targets
    # at mid-instruction bytes (root cause of the RunPlayerBlockCode
    # -1 stack drift / "Mario dies on slope" bug, 2026-05-03).
    if mnem in ('JSR', 'JSL'):
        ret_m, ret_x = post_m, post_x
        if callee_exit_mx is not None:
            target_pc24: Optional[int] = None
            if mnem == 'JSR' and insn.length == 3:
                target_pc24 = addr24(bank, insn.operand & 0xFFFF)
            elif mnem == 'JSL':
                target_pc24 = insn.operand & 0xFFFFFF
            if target_pc24 is not None:
                # Lookup keyed by (target_pc24, entry_m, entry_x) — same
                # entry variant we're invoking. Different variants of
                # the same callee may have different exit (m, x).
                key_lookup = (target_pc24, post_m, post_x)
                hit = callee_exit_mx.get(key_lookup)
                if hit is not None:
                    em, ex = hit
                    if em is not None and ex is not None:
                        ret_m, ret_x = em & 1, ex & 1
        return [(DecodeKey(addr24(bank, next_pc), ret_m, ret_x), 'fall')]

    # Default: linear fall-through with post-instruction mode.
    return [(DecodeKey(addr24(bank, next_pc), post_m, post_x), 'fall')]


def classify_dispatch_helper(rom: bytes, bank: int, addr: int):
    """Identify whether the subroutine at (bank, addr) is a JSL-jump-table
    dispatch helper. Returns 'short' (16-bit table entries), 'long'
    (24-bit table entries), or None.

    Pattern (canonical SMW + general 65816 ExecutePtr-style):
      - body PULAs/PLYs the JSL return PC off the SNES stack
      - body computes a table index and JMPs through (table,X) / [abs]
      - between the first `ASL A` and the next `TAY/TAX`, the presence
        of `ADC` distinguishes 24-bit vs 16-bit entries

    Ported from v1 recomp.py:_classify_dispatch_helper. Tracks REP/SEP
    so AND #imm decodes at correct width — without that tracking, the
    AND #$FFFF in $00:86DF gets sliced into AND #$FF + BRK $0A, eating
    the ASL A that's the classifier's signature.
    """
    from snes65816 import (decode_insn, lorom_offset, ACC, INDIR, INDIR_X,
                            INDIR_L)
    insns = []
    pc = addr & 0xFFFF
    m, x = 1, 1
    safety = 0
    while safety < 256:
        safety += 1
        if not (0x8000 <= pc <= 0xFFFF):
            return None
        try:
            offset = lorom_offset(bank, pc)
        except AssertionError:
            return None
        if offset >= len(rom):
            return None
        try:
            ins = decode_insn(rom, offset, pc, bank, m=m, x=x)
        except Exception:
            return None
        if ins is None:
            return None
        insns.append(ins)
        # Update mode for subsequent decodes.
        if ins.mnem == 'REP':
            if ins.operand & 0x20: m = 0
            if ins.operand & 0x10: x = 0
        elif ins.mnem == 'SEP':
            if ins.operand & 0x20: m = 1
            if ins.operand & 0x10: x = 1
        if ins.mnem in ('RTS', 'RTL', 'RTI', 'BRA', 'BRL', 'JMP', 'JML', 'STP'):
            break
        pc = (pc + ins.length) & 0xFFFF

    if not insns:
        return None
    # Must pull return address off stack.
    if not any(i.mnem in ('PLA', 'PLY') for i in insns):
        return None
    # Must end with an indirect jump.
    last = insns[-1]
    if not (last.mnem in ('JMP', 'JML') and
            last.mode in (INDIR, INDIR_X, INDIR_L)):
        return None
    # Width: ASL A ... TAY/TAX, with ADC in between → long.
    asl_seen = False
    has_adc = False
    for ins in insns:
        if not asl_seen:
            if ins.mnem == 'ASL' and ins.mode == ACC:
                asl_seen = True
            continue
        if ins.mnem == 'ADC':
            has_adc = True
        if ins.mnem in ('TAY', 'TAX'):
            return 'long' if has_adc else 'short'
    return None


def decode_function(rom: bytes, bank: int, start: int,
                    entry_m: int, entry_x: int,
                    *, end: Optional[int] = None,
                    max_insns: int = 4000,
                    dispatch_helpers: Optional[Dict[int, str]] = None,
                    indirect_call_tables: Optional[Dict[int, dict]] = None,
                    data_regions: Optional[List[Tuple[int, int, int]]] = None,
                    callee_exit_mx: Optional[Dict] = None,
                    ) -> FunctionDecodeGraph:
    """Decode a function starting at (bank, start) with entry (m, x) state.

    Worklist over DecodeKey tuples. Each key is decoded at most once;
    same PC with divergent (m, x) produces multiple keys → multiple
    DecodedInsn records.

    `dispatch_helpers`: optional map of {target_addr_24 -> 'short'|'long'}.
    When a JSL/JML hits a target in this map, the bytes immediately AFTER
    the JSL are decoded as a function-pointer TABLE (not as instructions).
    Each table entry is recorded as a successor key (so the dispatched
    handlers get decoded too) and the ORIGINAL JSL is marked with
    `insn.dispatch_entries` for downstream codegen. Decode resumes
    AFTER the table (not at the JSL+length offset). Without this hook,
    SMW's "JSL Foo; .dw target0, target1, ..." pattern at $00:9325 would
    decode the TABLE BYTES as garbage instructions.

    `indirect_call_tables`: optional map of {site_pc24 -> dict} where
    each value is `{'base': int, 'count': int, 'kind': 'short'|'long'}`.
    Authorises a JSR (abs,X) site as a real indirect dispatch. When
    set, the decoder reads `count` table entries at `bank:base`,
    stamps `insn.dispatch_entries`, and adds the entries as decode
    successors so handlers get decoded too. Without an entry, JSR
    (abs,X) is treated as cfg-unauthorised: the insn is placed in the
    graph with no successors (severing fall-through) and recorded in
    graph.suppressed_indirect_calls for the build report. See the
    cfg-required-dispatch-or-kill rule documented on
    SuppressedIndirectCall.
    """
    entry_m &= 1
    entry_x &= 1
    entry_key = DecodeKey(addr24(bank, start), entry_m, entry_x)
    graph = FunctionDecodeGraph(entry=entry_key)

    # Worklist holds (key, edge_kind, pred_pc). edge_kind is
    # 'entry' for the initial seed, 'jump' for BRA/BRL/JMP-ABS/cond-
    # branch-target, 'fall' for linear next-PC after non-control or
    # JSR/JSL. pred_pc is the predecessor PC (-1 for entry seed).
    #
    # end: gates the boundary CROSSING, not the imported territory:
    # we reject a fall-through edge whose SOURCE is inside [start,end)
    # and whose TARGET is past end:. That stops natural drift of the
    # entry's body into the NEXT cfg function. Inside imported
    # territory (source.pc >= end, reached via prior 'jump'), all
    # successors are decoded — fall-through within an imported routine
    # is part of that routine's lifetime, not a boundary crossing.
    worklist: List = [(entry_key, 'entry', -1)]

    while worklist:
        if len(graph.insns) >= max_insns:
            raise RuntimeError(
                f"v2 decoder exceeded max_insns={max_insns} at "
                f"function ${addr24(bank, start):06X}"
            )
        key, edge_kind, pred_pc = worklist.pop()
        if key in graph.insns:
            continue

        pc = key.pc & 0xFFFF
        # Boundary-crossing fall-through: predecessor was inside the
        # nominal range, and this fall-through would land past end: in
        # the next function's body. Reject — that's exactly what end:
        # was put in cfg to prevent. (Jump targets past end: were
        # already accepted by the same predecessor; this only blocks
        # the unintended drift.)
        if (end is not None
                and pc >= end
                and edge_kind == 'fall'
                and pred_pc >= 0
                and pred_pc < end):
            continue
        if not (0x8000 <= pc <= 0xFFFF):
            # Out-of-bank reference; surface upstream by skipping here.
            continue

        try:
            offset = lorom_offset(bank, pc)
        except AssertionError:
            continue
        if offset >= len(rom):
            continue

        insn = decode_insn(rom, offset, pc, bank, m=key.m, x=key.x)
        if insn is None:
            raise ValueError(
                f"v2 decoder: unknown opcode ${rom[offset]:02X} at "
                f"${bank:02X}:{pc:04X} entry_mx=({key.m},{key.x})"
            )

        # Stamp entry mode on the Insn so downstream consumers (cfg, IR,
        # codegen) see the entry state without needing the DecodeKey.
        insn.m_flag = key.m
        insn.x_flag = key.x

        # JSL/JML dispatch-table detection: if the call target is a
        # registered dispatch helper, decode the bytes immediately
        # following as the target table and record successors.
        is_jsl_or_jml = (insn.mnem == 'JSL' or
                         (insn.mnem == 'JMP' and insn.length == 4))  # JML
        helper_kind = None
        if dispatch_helpers and is_jsl_or_jml:
            helper_kind = dispatch_helpers.get(insn.operand & 0xFFFFFF)
        if helper_kind is not None:
            entries = []
            entry_size = 3 if helper_kind == 'long' else 2
            tbl_pc = (pc + insn.length) & 0xFFFF
            while len(entries) < 256 and tbl_pc + entry_size - 1 <= 0xFFFF:
                try:
                    tbl_off = lorom_offset(bank, tbl_pc)
                except AssertionError:
                    break
                if tbl_off + entry_size - 1 >= len(rom):
                    break
                lo = rom[tbl_off]
                hi = rom[tbl_off + 1]
                addr16 = lo | (hi << 8)
                if helper_kind == 'long':
                    eb = rom[tbl_off + 2]
                    if addr16 == 0 and eb == 0:
                        entries.append(0)
                        tbl_pc += entry_size
                        continue
                    if addr16 < 0x8000 or eb != bank:
                        break
                    # Validity gate: stop the table if the entry points
                    # into all-FF or all-00 bytes. See
                    # `_dispatch_target_is_padding` doc.
                    if _dispatch_target_is_padding(rom, eb, addr16):
                        break
                    # cfg `data_region:` gate — explicit ROM fact that
                    # (bank, pc16) range is data. Trumps any addr-range
                    # heuristic since the directive is ground truth.
                    if _addr_in_data_regions(data_regions, eb, addr16):
                        graph.dispatch_targets_suppressed.append(
                            DispatchTargetSuppressed(
                                site_pc24=(bank << 16) | pc,
                                target_pc24=(eb << 16) | addr16,
                                reason='data_region',
                                table_index=len(entries),
                            ))
                        break
                    full_entry = (eb << 16) | addr16
                else:
                    if addr16 == 0:
                        entries.append(0)
                        tbl_pc += entry_size
                        continue
                    if addr16 < 0x8000:
                        break
                    if _dispatch_target_is_padding(rom, bank, addr16):
                        break
                    if _addr_in_data_regions(data_regions, bank, addr16):
                        graph.dispatch_targets_suppressed.append(
                            DispatchTargetSuppressed(
                                site_pc24=(bank << 16) | pc,
                                target_pc24=(bank << 16) | addr16,
                                reason='data_region',
                                table_index=len(entries),
                            ))
                        break
                    full_entry = (bank << 16) | addr16
                # NOTE: do NOT bound the entry value by the dispatching
                # function's [start, end) range. The TABLE bytes live in
                # the function's range, but the table ENTRIES point to
                # OTHER handlers (e.g. GameMode00 at \$00:9391, well past
                # the dispatcher's $937D end). v1's recomp.py applied a
                # similar range check ONLY as a fallback when the entry
                # wasn't in `dispatch_known_addrs`; v2 doesn't have that
                # set yet, so any range bound here would terminate the
                # table at zero entries — exactly what was happening to
                # the SMW GameMode dispatch at $00:9325 before this fix.
                entries.append(full_entry if helper_kind == 'long' else addr16)
                tbl_pc += entry_size
            if entries:
                # Stash on the insn for codegen. Don't add dispatch
                # entries as decode successors — they're CROSS-FUNCTION
                # calls (auto-promote will pick them up). The JSL itself
                # is a TERMINATOR (no fall-through past the table because
                # the dispatcher returns to the dispatched handler's
                # caller, not to bytes after the JSL).
                insn.dispatch_entries = entries
                insn.dispatch_kind = helper_kind
                graph.insns[key] = DecodedInsn(key=key, insn=insn, successors=[])
                continue

        # cfg-required-dispatch-or-kill for JSR (abs,X). See class
        # SuppressedIndirectCall above and the regression test at
        # tests/v2/test_decoder_smc_phantom_suppression.py.
        if insn.mnem == 'JSR' and insn.mode == INDIR_X:
            site_pc24 = (bank << 16) | pc
            auth = (indirect_call_tables or {}).get(site_pc24)
            if auth is not None:
                # AUTHORISED: read the static dispatch table from
                # `bank:base`, register entries as decode successors,
                # stamp `insn.dispatch_entries` for codegen.
                base = int(auth['base']) & 0xFFFF
                count = int(auth['count'])
                kind = auth.get('kind', 'short')
                entry_size = 3 if kind == 'long' else 2
                entries = []
                tbl_pc = base
                for _i in range(count):
                    if tbl_pc + entry_size - 1 > 0xFFFF:
                        break
                    try:
                        tbl_off = lorom_offset(bank, tbl_pc)
                    except AssertionError:
                        break
                    if tbl_off + entry_size - 1 >= len(rom):
                        break
                    addr16 = rom[tbl_off] | (rom[tbl_off + 1] << 8)
                    if kind == 'long':
                        eb = rom[tbl_off + 2]
                        entries.append((eb << 16) | addr16)
                    else:
                        entries.append(addr16)
                    tbl_pc += entry_size
                insn.dispatch_entries = entries
                insn.dispatch_kind = kind
                # Fall-through edge IS preserved for an authorised JSR
                # (the call returns to the next insn, like any JSR).
                # Table entries are added as decode successors (jump
                # edges) so handlers get auto-promoted.
                labeled_succ = _labeled_successors(insn, key, bank,
                                           callee_exit_mx=callee_exit_mx)
                # Append jump-kind edges to the in-bank handlers.
                for e in entries:
                    e16 = e & 0xFFFF
                    eb = (e >> 16) & 0xFF if kind == 'long' else bank
                    if eb == bank and 0x8000 <= e16 <= 0xFFFF:
                        labeled_succ.append(
                            (DecodeKey(addr24(eb, e16), key.m, key.x), 'jump')
                        )
                succ = [k for (k, _) in labeled_succ]
                graph.insns[key] = DecodedInsn(key=key, insn=insn, successors=succ)
                for s, sk in labeled_succ:
                    if s not in graph.insns:
                        worklist.append((s, sk, pc))
                continue
            # UNAUTHORISED: drop fall-through; record for build report.
            # The insn lives in the graph (so predecessors' successor
            # edges still resolve) but with no outgoing successors.
            graph.insns[key] = DecodedInsn(key=key, insn=insn, successors=[])
            graph.suppressed_indirect_calls.append(SuppressedIndirectCall(
                site_pc24=site_pc24,
                table_base=insn.operand & 0xFFFF,
                function_entry_pc24=addr24(bank, start),
                entry_m=key.m,
                entry_x=key.x,
            ))
            continue

        labeled_succ = _labeled_successors(insn, key, bank,
                                           callee_exit_mx=callee_exit_mx)
        succ = [k for (k, _) in labeled_succ]
        graph.insns[key] = DecodedInsn(key=key, insn=insn, successors=succ)

        for s, sk in labeled_succ:
            if s not in graph.insns:
                worklist.append((s, sk, pc))

    # Constant-Z branch fold + reachability prune. Runs once after the
    # worklist drains so predecessor counts are stable. See
    # `_apply_constant_z_fold` for the narrow scope.
    _apply_constant_z_fold(graph)

    return graph


def analyze_function_exit_mx(graph: 'FunctionDecodeGraph'
                             ) -> 'Tuple[Optional[int], Optional[int]]':
    """Compute the (m, x) state at which a function returns to its caller.

    Walks every decoded RTS/RTL/RTI in `graph` and takes the meet of
    their entry (m, x) — RTS/RTL/RTI don't modify M/X, so each
    terminator's `(insn.m_flag, insn.x_flag)` IS the (m, x) at the
    moment of return.

    If all return paths exit with the same (m, x), returns that pair.
    If any two return paths disagree, the corresponding component is
    `None` (ambiguous — the caller's decoder should fall back to its
    pre-call assumption rather than commit to a wrong width).

    Functions with no terminators (e.g. infinite loops, table-only)
    return `(None, None)` — no callable resume state to propagate.
    """
    exit_m: Optional[int] = None
    exit_x: Optional[int] = None
    have_any = False
    m_ambig = False
    x_ambig = False
    for di in graph.insns.values():
        ins = di.insn
        if ins.mnem not in ('RTS', 'RTL', 'RTI'):
            continue
        em = ins.m_flag & 1
        ex = ins.x_flag & 1
        if not have_any:
            exit_m, exit_x = em, ex
            have_any = True
            continue
        if not m_ambig and exit_m != em:
            m_ambig = True
        if not x_ambig and exit_x != ex:
            x_ambig = True
    if m_ambig:
        exit_m = None
    if x_ambig:
        exit_x = None
    if not have_any:
        return (None, None)
    return (exit_m, exit_x)


def _apply_constant_z_fold(graph: FunctionDecodeGraph) -> None:
    """Decoder post-pass: rewrite BEQ/BNE successors to a single live
    edge when the same-block predecessor is an immediate LDA/LDX/LDY
    that makes Z statically known.

    Narrow scope (deliberate — see project_constant_z_fold spec):
        * Predecessor must be LDA/LDX/LDY in IMM addressing mode.
        * Predecessor's only successor must be this branch (no other
          edge can land on the load between it and the branch).
        * Branch must have exactly ONE predecessor (the load) and
          exactly TWO successors (fall + jump from _labeled_successors).
        * Op width follows m for LDA, x for LDX/LDY (entry mode of
          the load, which is what the decoder used to read its bytes).
        * Only Z-flag branches (BEQ/BNE). N/V/C are explicitly out of
          scope for this initial fold; SEP/REP/PLP/ALU are out too.

    On match:
        * graph.insns[branch_key].successors becomes [live_edge_only].
        * insn.const_z_fold_unconditional is set so lowering emits a
          `Goto` IR op (single successor) rather than a `CondBranch`
          (two-successor flag test).
        * insn.const_z_fold_dead_pc24 records the pruned target for
          the build report.
        * Reachability is recomputed from graph.entry; insns reachable
          ONLY through the pruned edge are removed from graph.insns
          (and therefore from cfg block construction + codegen). Their
          unresolvable-goto markers, if any, vanish along with them —
          that is the point of the fold.
        * graph.const_z_folds gets a record for the build report.
    """
    if not graph.insns:
        return

    # Build predecessors map.
    preds: Dict[DecodeKey, set] = {}
    for k, di in graph.insns.items():
        for s in di.successors:
            preds.setdefault(s, set()).add(k)

    # Apply folds. Iterate over a snapshot of keys because we mutate
    # graph.insns mid-loop.
    for k in list(graph.insns.keys()):
        di = graph.insns.get(k)
        if di is None:
            continue
        insn = di.insn
        if insn.mnem not in ('BEQ', 'BNE'):
            continue
        my_preds = preds.get(k, set())
        if len(my_preds) != 1:
            continue
        pred_key = next(iter(my_preds))
        pred_di = graph.insns.get(pred_key)
        if pred_di is None:
            continue
        pred_insn = pred_di.insn
        if pred_insn.mnem not in ('LDA', 'LDX', 'LDY'):
            continue
        if pred_insn.mode != IMM:
            continue
        if len(pred_di.successors) != 1 or pred_di.successors[0] != k:
            continue
        if len(di.successors) != 2:
            # Already pruned (defensive — shouldn't reach this branch).
            continue

        # Compute Z from the masked immediate. LDA uses m-width;
        # LDX/LDY use x-width. Use the LOAD's entry flags (the mode
        # under which decode_insn read its operand bytes).
        if pred_insn.mnem == 'LDA':
            width_bits = 8 if pred_insn.m_flag == 1 else 16
        else:
            width_bits = 8 if pred_insn.x_flag == 1 else 16
        mask = (1 << width_bits) - 1
        masked = pred_insn.operand & mask
        z = 1 if masked == 0 else 0

        # successors order from _labeled_successors for cond branch:
        # [(fall, 'fall'), (jump, 'jump')].
        fall_succ, jump_succ = di.successors[0], di.successors[1]
        if insn.mnem == 'BEQ':
            taken = (z == 1)
        else:  # BNE
            taken = (z == 0)
        live = jump_succ if taken else fall_succ
        dead = fall_succ if taken else jump_succ

        # Rewrite successors to single live edge.
        graph.insns[k] = DecodedInsn(key=k, insn=insn, successors=[live])
        insn.const_z_fold_unconditional = True
        insn.const_z_fold_dead_pc24 = dead.pc & 0xFFFFFF

        # Build a context-rich record for the report.
        graph.const_z_folds.append(ConstZFold(
            branch_pc24=insn.addr & 0xFFFFFF,
            prev_pc24=pred_insn.addr & 0xFFFFFF,
            branch_mnem=insn.mnem,
            prev_mnem=pred_insn.mnem,
            prev_imm=masked,
            width_bits=width_bits,
            z_value=z,
            taken_kind='jump' if taken else 'fall',
            live_pc24=live.pc & 0xFFFFFF,
            dead_pc24=dead.pc & 0xFFFFFF,
            func_entry_pc24=graph.entry.pc & 0xFFFFFF,
            entry_m=graph.entry.m & 1,
            entry_x=graph.entry.x & 1,
        ))

    # Reachability prune. Walk from entry; drop any insn no longer
    # reachable. Without this the dead-path insns linger in graph.insns
    # and cfg.build picks them up as orphan blocks (carrying any
    # unresolvable-goto markers they accumulated).
    reachable: set = set()
    work = [graph.entry]
    while work:
        cur = work.pop()
        if cur in reachable:
            continue
        if cur not in graph.insns:
            continue
        reachable.add(cur)
        for s in graph.insns[cur].successors:
            work.append(s)
    for k in list(graph.insns.keys()):
        if k not in reachable:
            del graph.insns[k]
