//! `chase-misses` — Rust port of `tools/strat_autoroute.py`.
//!
//! Turns a runtime dispatch-miss profile (SF_DUMP_MISSES) into `func` cfg
//! entries for coroutine resume PCs. Behaviorally identical to the Python tool;
//! dry-run stdout is byte-for-byte identical.
//!
//! Usage:
//!   chase-misses --misses /tmp/sf_misses.txt --cfg-dir recomp [--symbols SYMBOLS.TXT] [--apply]

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::Path;

use regex::Regex;

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

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let misses_path = flag_val(&args, "--misses")
        .unwrap_or_else(|| "/tmp/sf_misses.txt".to_string());
    let cfg_dir = flag_val(&args, "--cfg-dir")
        .unwrap_or_else(|| "recomp".to_string());
    let symbols_path = flag_val(&args, "--symbols")
        .unwrap_or_else(|| "SYMBOLS.TXT".to_string());
    let apply = args.contains(&"--apply".to_string());

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
