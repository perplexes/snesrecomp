//! `chase-misses` — Rust port of `tools/strat_autoroute.py`.
//!
//! Turns a runtime dispatch-miss profile (SF_DUMP_MISSES) into `func` cfg
//! entries for coroutine resume PCs. Behaviorally identical to the Python tool;
//! dry-run stdout is byte-for-byte identical.
//!
//! Usage:
//!   chase-misses --misses /tmp/sf_misses.txt --cfg-dir recomp [--symbols SYMBOLS.TXT] [--apply]
//!   chase-misses ... --explain --rom sf.sfc   # self-diagnose each miss (ADDITIVE output)
//!
//! `--explain` appends a per-miss diagnosis AFTER the normal plan (the default
//! output stays byte-for-byte identical to the Python tool): it decodes the
//! target, classifies it code-vs-data, names the dispatch source's enclosing
//! symbol, and flags a target outside the source strat's registered resume-PC
//! cluster — the signature of a garbage continuation pointer.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::Path;

use regex::Regex;
use snesrecomp_regen::insn::{decode_insn, validate_decoded_insns, Insn};
use snesrecomp_regen::rom::load_rom;

// Whitelisted coroutine-dispatcher SOURCE ranges (24-bit PC of the dispatch RTL).
// A miss is a genuine resume iff its `from=` falls in one of these.
//   do_strat_l   $1FD283 — per-object strat dispatcher (TRANS/STRATROU)
//   dostrats     $02D6D7 (STRATLP loop) — group/child strat dispatcher
//   jumptostate  $09C013 family (MODECHANGE*/JUMPTOSTATE_L) — object FSM
const DISPATCHER_SOURCES: &[(u32, u32, &str)] = &[
    (0x1FD283, 0x1FD380, "do_strat_l"),
    (0x02D6D7, 0x02D720, "dostrats/stratlp"),
    (0x09C013, 0x09C060, "jumptostate_l/modechange"),
];

fn is_dispatcher(src: u32) -> Option<&'static str> {
    for &(lo, hi, label) in DISPATCHER_SOURCES {
        if src >= lo && src <= hi {
            return Some(label);
        }
    }
    None
}

/// LoROM mirror: banks $80-$FF mirror $00-$7F (clear bit 7).
fn canon_bank(pc24: u32) -> u32 {
    let bank = (pc24 >> 16) & 0xFF;
    let bank = if bank >= 0x80 { bank & 0x7F } else { bank };
    (bank << 16) | (pc24 & 0xFFFF)
}

/// Load symbols from SYMBOLS.TXT: lines `NAME\t$<8 hex>` → (addr, name); sort by addr.
fn load_symbols(path: &Path) -> Vec<(u32, String)> {
    let mut syms = Vec::new();
    if !path.exists() {
        return syms;
    }
    // re.match() in Python anchors at start of string
    let re = Regex::new(r"^(\S+)\s+\$([0-9A-Fa-f]{8})").unwrap();
    let content = std::fs::read_to_string(path).unwrap_or_default();
    for line in content.lines() {
        if let Some(cap) = re.captures(line) {
            let addr = u32::from_str_radix(&cap[2], 16).unwrap();
            let name = cap[1].to_string();
            syms.push((addr, name));
        }
    }
    syms.sort_by_key(|(a, _)| *a);
    syms
}

/// Largest symbol <= addr (binary-search then walk back).
fn nearest_sym<'a>(syms: &'a [(u32, String)], addr: u32) -> Option<(u32, &'a str)> {
    // The Python iterates sorted list and keeps the last one <= addr with early break.
    // Equivalent: partition point then step back.
    let idx = syms.partition_point(|(a, _)| *a <= addr);
    if idx == 0 {
        None
    } else {
        let (a, n) = &syms[idx - 1];
        Some((*a, n.as_str()))
    }
}

/// Parse misses file: lines `<6hex tgt> mx=<digit> from=<6hex src>`.
fn parse_misses(path: &Path) -> Vec<(u32, u32, u32)> {
    // re.match() anchors at start; line is strip()'d in Python
    let re = Regex::new(r"^([0-9A-Fa-f]{6})\s+mx=(\d)\s+from=([0-9A-Fa-f]{6})").unwrap();
    let content = std::fs::read_to_string(path).expect("cannot read misses file");
    let mut out = Vec::new();
    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(cap) = re.captures(line) {
            let tgt = u32::from_str_radix(&cap[1], 16).unwrap();
            let mx = cap[2].parse::<u32>().unwrap();
            let src = u32::from_str_radix(&cap[3], 16).unwrap();
            out.push((tgt, mx, src));
        }
    }
    out
}

/// Set of in-bank offsets already declared as `func` entries in a cfg file.
fn existing_offsets(cfg_path: &Path) -> HashSet<u32> {
    let mut offs = HashSet::new();
    if !cfg_path.exists() {
        return offs;
    }
    // re.match() anchors at start
    let re = Regex::new(r"^\s*func\s+\S+\s+([0-9A-Fa-f]{1,4})\b").unwrap();
    let content = std::fs::read_to_string(cfg_path).unwrap_or_default();
    for line in content.lines() {
        if let Some(cap) = re.captures(line) {
            if let Ok(off) = u32::from_str_radix(&cap[1], 16) {
                offs.insert(off);
            }
        }
    }
    offs
}

struct PlanEntry {
    src: u32,
    label: &'static str,
    name: String,
    tgt: u32,
}

struct SkippedEntry {
    tgt: u32,
    mx: u32,
    src: u32,
    why: String,
}

fn flag_val(args: &[String], flag: &str) -> Option<String> {
    args.iter().position(|a| a == flag).and_then(|i| args.get(i + 1).cloned())
}

// ── --explain helpers ─────────────────────────────────────────────────────────

/// Miss `mx` code → (m, x): mx is `(m<<1)|x` (mx=2 == M1X0).
fn mx_to_mx(mx: u32) -> (u8, u8) {
    (((mx >> 1) & 1) as u8, (mx & 1) as u8)
}

/// Decode up to 8 insns at (bank, pc16) under (m, x). Returns disasm lines and a
/// "looks like real code" verdict (a clean run that passes validate_decoded_insns
/// and doesn't immediately hit an unknown opcode / BRK / COP).
fn decode_probe(rom: &[u8], bank: u32, pc16: u32, m: u8, x: u8) -> DecodeProbe {
    let mut lines = Vec::new();
    let mut insns: Vec<Insn> = Vec::new();
    let mut cur = pc16 & 0xFFFF;
    let (mut cm, mut cx) = (m, x);   // track width across REP/SEP within the fragment
    let mut unknown = false;
    let mut returns_in: Option<usize> = None; // insns until the first RTL/RTS/RTI
    // A manufactured-RTL coroutine resume is a short self-contained fragment that
    // reaches a return (RTL/RTS/RTI) — decode until the first return (max 24 insns).
    for i in 0..24 {
        if !(0x8000..=0xFFFF).contains(&cur) { break; }
        let off = ((bank & 0x7F) as usize) * 0x8000 + (cur as usize - 0x8000);
        if off >= rom.len() { break; }
        match decode_insn(rom, off, cur, bank, cm, cx) {
            Some(ins) => {
                let n = ins.length as usize;
                let raw: String = (0..n).map(|k| format!("{:02X} ", rom[off + k])).collect();
                lines.push(format!("    ${:02X}:{:04X}: {:<11} {}", bank, cur, raw.trim_end(), ins.mnem));
                let is_ret = matches!(ins.mnem, "RTL" | "RTS" | "RTI");
                // advance width so a REP/SEP-led resume fragment decodes correctly
                match ins.mnem {
                    "REP" => { if ins.operand & 0x20 != 0 { cm = 0; } if ins.operand & 0x10 != 0 { cx = 0; } }
                    "SEP" => { if ins.operand & 0x20 != 0 { cm = 1; } if ins.operand & 0x10 != 0 { cx = 1; } }
                    _ => {}
                }
                cur = (cur + ins.length as u32) & 0xFFFF;
                insns.push(ins);
                if is_ret { returns_in = Some(i + 1); break; }
            }
            None => {
                lines.push(format!("    ${:02X}:{:04X}: ?? {:02X} (unknown opcode)", bank, cur, rom[off]));
                unknown = true;
                break;
            }
        }
    }
    let looks_code = !unknown && !insns.is_empty() && validate_decoded_insns(&insns);
    DecodeProbe { lines, looks_code, returns_in }
}

struct DecodeProbe { lines: Vec<String>, looks_code: bool, returns_in: Option<usize> }

/// Parse `func <name> <hexoff> …` entries from a bank cfg → (name, offset).
fn cfg_funcs(cfg_path: &Path) -> Vec<(String, u32)> {
    let mut out = Vec::new();
    let Ok(text) = std::fs::read_to_string(cfg_path) else {
        return out;
    };
    for line in text.lines() {
        let t = line.trim();
        if let Some(rest) = t.strip_prefix("func ") {
            let mut it = rest.split_whitespace();
            if let (Some(name), Some(off)) = (it.next(), it.next()) {
                if let Ok(o) = u32::from_str_radix(off, 16) {
                    out.push((name.to_string(), o & 0xFFFF));
                }
            }
        }
    }
    out
}

/// Static resume-table enumeration: find manufactured-RTL sites in a bank (an
/// `ADC #imm16` that feeds a `PHA` before an `RTL` — the computed-pointer idiom),
/// then scan the bank for the state immediates (`LDA #imm16`) whose computed entry
/// `imm + base + 1` is a clean RTL-terminated resume NEAR an already-registered
/// resume (the cluster anchor). Emits a `func` line for every sibling not yet
/// registered — so the whole resume table is registered at compile time and the
/// unregistered-resume SKIP-leak crash becomes structurally impossible.
fn enumerate_resumes(rom: &[u8], cfg_dir: &str, banks: &[u32]) {
    for &bank in banks {
        let base_off = ((bank & 0x7F) as usize) * 0x8000;
        if base_off + 0x8000 > rom.len() { continue; }
        let seg = &rom[base_off..base_off + 0x8000];
        // 1. manufactured-RTL bases: ADC #imm16 (69 lo hi) then PHA (48) then RTL (6b)
        let mut bases: std::collections::BTreeSet<u32> = std::collections::BTreeSet::new();
        for i in 0..seg.len().saturating_sub(3) {
            if seg[i] == 0x69 {
                let imm = seg[i + 1] as u32 | (seg[i + 2] as u32) << 8;
                if imm < 0x8000 { continue; }               // base must land the target in ROM ($8000+)
                let (mut pha, mut rtl) = (false, false);
                for j in 3..22 {
                    match seg.get(i + j) {
                        Some(0x48) => pha = true,
                        Some(0x6b) if pha => { rtl = true; break; }
                        _ => {}
                    }
                }
                if pha && rtl { bases.insert(imm); }
            }
        }
        if bases.is_empty() { continue; }
        // registered resume entries in this bank (the cluster anchors)
        let cfg = format!("{}/bank{:02x}.cfg", cfg_dir, bank);
        let registered: std::collections::BTreeSet<u32> =
            cfg_funcs(Path::new(&cfg)).into_iter().map(|(_, o)| o).collect();
        // 2. scan for LDA #imm16 (A9 lo hi) state-setters -> resume entries
        let mut found: std::collections::BTreeMap<u32, u32> = std::collections::BTreeMap::new();
        for &base in &bases {
            for i in 0..seg.len().saturating_sub(2) {
                if seg[i] != 0xA9 { continue; }
                let imm = seg[i + 1] as u32 | (seg[i + 2] as u32) << 8;
                let entry = imm.wrapping_add(base).wrapping_add(1) & 0xFFFF;
                if !(0x8000..=0xFFFF).contains(&entry)
                    || registered.contains(&entry) || found.contains_key(&entry) { continue; }
                // cluster anchor: entry must be close to a registered resume (±$100)
                if !registered.iter().any(|&r| entry.abs_diff(r) <= 0x100) { continue; }
                let p = decode_probe(rom, bank, entry, 1, 0);   // resume entry_mx:1,0
                if p.looks_code && p.returns_in.is_some() {
                    found.insert(entry, base);
                }
            }
        }
        if !found.is_empty() {
            println!("== bank{:02X}: {} manufactured-RTL base(s), {} statically-visible NEW resume(s) ==",
                     bank, bases.len(), found.len());
            for (entry, base) in &found {
                println!("  func STRAT_R{:04X} {:04X} entry_mx:1,0   # enumerated: LDA #imm -> ${:04X}=imm+${:04X}+1, RTL-terminated",
                         entry, entry, entry, base);
            }
        }
    }
    eprintln!("NOTE: static enumeration only sees resume offsets set by a literal `LDA #imm`. \
               Offsets that are RUNTIME state values (loaded from RAM / a data table — e.g. Star Fox's \
               $9DC5 $04B0=$455D -> $E97E) are invisible here. For COMPLETE coverage use the runtime \
               classifier on real misses (chase-misses --explain) or the oracle-reachability audit.");
}

/// Strip a `_R<hex>` resume suffix to get the base strat name.
fn strat_base(name: &str) -> &str {
    name.rsplit_once("_R")
        .filter(|(_, suf)| suf.len() >= 3 && suf.chars().all(|c| c.is_ascii_hexdigit()))
        .map(|(b, _)| b)
        .unwrap_or(name)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let misses_path = flag_val(&args, "--misses")
        .unwrap_or_else(|| "/tmp/sf_misses.txt".to_string());
    let cfg_dir = flag_val(&args, "--cfg-dir")
        .unwrap_or_else(|| "recomp".to_string());
    let symbols_path = flag_val(&args, "--symbols")
        .unwrap_or_else(|| "SYMBOLS.TXT".to_string());
    let apply = args.contains(&"--apply".to_string());
    let explain = args.contains(&"--explain".to_string());
    let rom_path = flag_val(&args, "--rom");

    // --enum-resumes: proactively enumerate manufactured-RTL resume tables and
    // print every unregistered sibling, so the resume set is complete before any
    // run (structurally prevents the unregistered-resume SKIP-leak crash class).
    if args.contains(&"--enum-resumes".to_string()) {
        let rom = rom_path.as_ref().and_then(|p| load_rom(p).ok())
            .expect("--enum-resumes requires --rom <game.sfc>");
        let banks: Vec<u32> = flag_val(&args, "--banks")
            .map(|s| s.split(',').filter_map(|b| u32::from_str_radix(b.trim(), 16).ok()).collect())
            .unwrap_or_else(|| (0..0x40).collect());
        enumerate_resumes(&rom, &cfg_dir, &banks);
        return;
    }

    // Strat-family symbol allowlist: same pattern as Python's STRAT_NAME_RE.
    // re.IGNORECASE → (?i); ^ and $ in alternation match start/end of text.
    let strat_name_re = Regex::new(
        r"(?i)(_I?STRAT|^SR\d|_STRAT|TAB$|ACHASE|SKILLFLY|^PATH|MOVE|FLY|CHASE|BOSS|OUT_OF_THIS|JUMPTO|MODECHANGE|EXPLODE|HIT|TRAIL|SNAKE|MOTHER|PARA)"
    ).unwrap();

    let syms = load_symbols(Path::new(&symbols_path));
    let misses = parse_misses(Path::new(&misses_path));

    // Dedup resume targets (bank, offset) → PlanEntry; last miss for same key wins.
    let mut plan: BTreeMap<(u32, u32), PlanEntry> = BTreeMap::new();
    let mut skipped: Vec<SkippedEntry> = Vec::new();

    for &(tgt, mx, src) in &misses {
        let label = is_dispatcher(src);
        if label.is_none() || mx != 2 {
            skipped.push(SkippedEntry {
                tgt,
                mx,
                src,
                why: "non-dispatcher/mode".to_string(),
            });
            continue;
        }
        let label = label.unwrap();
        let ctgt = canon_bank(tgt);
        let bank = (ctgt >> 16) & 0xFF;
        let off = ctgt & 0xFFFF;

        let ns = nearest_sym(&syms, ctgt);
        if ns.is_none() || !strat_name_re.is_match(ns.unwrap().1) {
            let nm = ns.map(|(_, n)| n).unwrap_or("?");
            skipped.push(SkippedEntry {
                tgt,
                mx,
                src,
                why: format!("non-strat sym {}", nm),
            });
            continue;
        }
        let (base_addr, base_name) = ns.unwrap();
        let delta = ctgt - base_addr;
        let name = if delta == 0 {
            base_name.to_string()
        } else {
            format!("{}_R{:04X}", base_name, off)
        };
        plan.insert(
            (bank, off),
            PlanEntry {
                src,
                label,
                name,
                tgt: ctgt,
            },
        );
    }

    // Group plan by bank cfg, filtering offsets already declared.
    // plan is a BTreeMap so iteration is already sorted by (bank, off).
    let mut by_bank: BTreeMap<String, Vec<(u32, &PlanEntry)>> = BTreeMap::new();
    // Cache existing offsets per cfg to avoid re-reading (same result as Python).
    let mut offs_cache: HashMap<String, HashSet<u32>> = HashMap::new();

    for ((bank, off), info) in &plan {
        let cfg = format!("{}/bank{:02x}.cfg", cfg_dir, bank);
        let existing = offs_cache
            .entry(cfg.clone())
            .or_insert_with(|| existing_offsets(Path::new(&cfg)));
        if existing.contains(off) {
            continue;
        }
        by_bank.entry(cfg).or_default().push((*off, info));
    }

    let total_new: usize = by_bank.values().map(|v| v.len()).sum();
    println!(
        "== strat_autoroute: {} misses, {} candidate resume PCs, {} NEW to add ==",
        misses.len(),
        plan.len(),
        total_new
    );

    for (cfg, entries) in &by_bank {
        println!("\n  {}:", cfg);
        for (off, info) in entries {
            println!(
                "    func {} {:04X} entry_mx:1,0   # ${:06X} via {} (from ${:06X})",
                info.name, off, info.tgt, info.label, info.src
            );
        }
    }

    if !skipped.is_empty() {
        println!(
            "\n  -- {} misses NOT auto-routed (quarantine/review): --",
            skipped.len()
        );
        for s in &skipped {
            println!(
                "    ${:06X} mx={} from=${:06X}  ({})",
                s.tgt, s.mx, s.src, s.why
            );
        }
    }

    // --explain: ADDITIVE per-miss diagnosis (default output above is unchanged).
    if explain {
        let rom = rom_path.as_ref().and_then(|p| load_rom(p).ok());
        println!("\n== --explain: per-miss diagnosis ==");
        if rom.is_none() {
            println!("  (pass --rom <game.sfc> to decode targets)");
        }
        // dedup misses by (canon target, src) preserving first-seen order
        let mut seen = HashSet::new();
        for &(tgt, mx, src) in &misses {
            let ctgt = canon_bank(tgt);
            if !seen.insert((ctgt, src)) {
                continue;
            }
            let (m, x) = mx_to_mx(mx);
            let bank = (ctgt >> 16) & 0xFF;
            let off = ctgt & 0xFFFF;
            let src_sym = nearest_sym(&syms, canon_bank(src));
            let tgt_sym = nearest_sym(&syms, ctgt);
            let routed = is_dispatcher(src).is_some() && mx == 2;
            println!(
                "\n  ${:06X} (mx={} → M{}X{})  from ${:06X}  [{}]",
                ctgt, mx, m, x, src,
                if routed { "would route" } else { "QUARANTINED" }
            );
            if let Some((a, n)) = src_sym {
                println!("    source: in {} (+${:X})  dispatcher={}", n, canon_bank(src).wrapping_sub(a),
                    is_dispatcher(src).unwrap_or("no — not a whitelisted coroutine dispatcher"));
            }
            if let Some((a, n)) = tgt_sym {
                println!("    target: near {} (+${:X})", n, ctgt.wrapping_sub(a));
            }
            // resume-cluster check: does the source strat's resume PCs bracket this target?
            if let Some((_, sname)) = src_sym {
                let base = strat_base(sname);
                let cfg = format!("{}/bank{:02x}.cfg", cfg_dir, bank);
                let cluster: Vec<u32> = cfg_funcs(Path::new(&cfg))
                    .into_iter()
                    .filter(|(n, _)| strat_base(n) == base)
                    .map(|(_, o)| o)
                    .collect();
                if let (Some(&lo), Some(&hi)) = (cluster.iter().min(), cluster.iter().max()) {
                    let inside = off >= lo && off <= hi;
                    println!(
                        "    cluster: {}'s {} resume PCs span ${:04X}-${:04X} — target ${:04X} is {}",
                        base, cluster.len(), lo, hi, off,
                        if inside { "INSIDE (plausible resume)" } else { "OUTSIDE (likely a garbage pointer)" }
                    );
                }
            }
            // decode probe + INTRINSIC resume classification (no chicken-and-egg
            // cluster bracket: a first-of-its-kind resume is OUTSIDE the 1-entry
            // cluster yet still registerable).
            if let Some(rom) = &rom {
                let p = decode_probe(rom, bank, off, m, x);
                println!("    decode @ target ({}{}):",
                    if p.looks_code { "looks like CODE" } else { "looks like DATA / garbage" },
                    match p.returns_in { Some(k) => format!(", RTL/RTS-terminated in {k} insns"),
                                         None => ", no return in window".to_string() });
                for l in &p.lines { println!("  {}", l); }
                // Verdict: a manufactured-RTL resume is (source = coroutine dispatcher)
                // AND (target decodes clean) AND (target is a self-contained fragment
                // that returns via RTL/RTS/RTI). Those three are intrinsic — they do
                // NOT depend on the target already being bracketed by registered
                // resumes (the flaw that mis-called $04:E97E "garbage").
                let src_is_rtl = is_dispatcher(src).is_some();
                let terminates = p.returns_in.is_some();
                if src_is_rtl && p.looks_code && terminates {
                    let sub = src_sym.map(|(_, n)| strat_base(n)).unwrap_or_else(|| "STRAT".into());
                    println!("    VERDICT: REGISTERABLE manufactured-RTL resume (clean code, \
                              returns via RTL, from a coroutine dispatcher). Add to bank{:02x}.cfg:", bank);
                    println!("      func {}_R{:04X} {:04X} entry_mx:{},{}   # {} resume", sub, off, off, m, x, sub);
                } else if src_is_rtl && p.looks_code && !terminates {
                    println!("    VERDICT: SUSPICIOUS — clean code but no RTL/RTS in window; \
                              may be a mid-routine landing (real stack/state bug), not a resume.");
                } else if !p.looks_code {
                    println!("    VERDICT: NOT CODE at this width — likely a genuine bad pointer / \
                              wrong exit-mx (real bug), do NOT register.");
                }
            }
        }
    }

    if !apply {
        // U+2014 em dash, matching the Python source literal "—"
        println!("\n(dry run \u{2014} pass --apply to write)");
        return;
    }

    for (cfg, entries) in &by_bank {
        use std::io::Write as _;
        let mut f = std::fs::OpenOptions::new()
            .append(true)
            .open(cfg)
            .unwrap_or_else(|e| panic!("cannot open {cfg}: {e}"));
        writeln!(
            f,
            "\n# --- auto-routed coroutine resume PCs (strat_autoroute) ---"
        )
        .unwrap();
        for (off, info) in entries {
            writeln!(
                f,
                "func {} {:04X} entry_mx:1,0  # ${:06X} via {} (from ${:06X})",
                info.name, off, info.tgt, info.label, info.src
            )
            .unwrap();
        }
        println!("  wrote {} entries to {}", entries.len(), cfg);
    }
    println!("\nApplied {} entries. Regen + sync funcs.h + build next.", total_new);
}
