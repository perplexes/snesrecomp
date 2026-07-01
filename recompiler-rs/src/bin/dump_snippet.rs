//! dump-snippet — emit the C body for a straight-line 65816 snippet via the Rust
//! codegen, tracking M/X across REP/SEP. The Rust counterpart to the Phase-B
//! fuzzer's old Python emit_snippet_body, so the fuzzer tests the SHIPPING
//! (Rust) codegen instead of the retired Python one.
//!
//! Usage: dump-snippet --bytes <hex> --m <0|1> --x <0|1>
//! Prints one C line per emitted IR op to stdout.
use snesrecomp_regen::codegen::{emit_op, EmitCtx, EmitOutcome};
use snesrecomp_regen::insn::decode_insn;
use snesrecomp_regen::lowering::{lower, ValueFactory};

fn arg(args: &[String], f: &str) -> Option<String> {
    args.iter().position(|a| a == f).and_then(|i| args.get(i + 1).cloned())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let hex = arg(&args, "--bytes").expect("--bytes <hex> required");
    let rom: Vec<u8> = (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).expect("bad hex"))
        .collect();
    let mut m: u8 = arg(&args, "--m").and_then(|s| s.parse().ok()).unwrap_or(0);
    let mut x: u8 = arg(&args, "--x").and_then(|s| s.parse().ok()).unwrap_or(0);
    // --meta: emit one "mnem|mode|m|x" line per decoded insn instead of C (drives
    // the fuzzer's coverage ledger without a Python decoder).
    let meta = args.iter().any(|a| a == "--meta");

    let ctx = EmitCtx::default();
    let mut vf = ValueFactory::new();
    let mut outcome = EmitOutcome::default();
    let (mut off, mut pc) = (0usize, 0x8000u32);
    while off < rom.len() {
        let mut insn = decode_insn(&rom, off, pc, 0, m, x)
            .unwrap_or_else(|| panic!("decode fail at offset {off}"));
        insn.m_flag = m;
        insn.x_flag = x;
        if meta {
            println!("{}|{:?}|{}|{}", insn.mnem, insn.mode, m, x);
        } else {
            for op in lower(&insn, &mut vf) {
                for ln in emit_op(&ctx, &op, &mut outcome, None) {
                    println!("{ln}");
                }
            }
        }
        if insn.mnem == "REP" {
            if insn.operand & 0x20 != 0 { m = 0; }
            if insn.operand & 0x10 != 0 { x = 0; }
        } else if insn.mnem == "SEP" {
            if insn.operand & 0x20 != 0 { m = 1; }
            if insn.operand & 0x10 != 0 { x = 1; }
        }
        off += insn.length as usize;
        pc = (pc + insn.length as u32) & 0xFFFF;
    }
}
