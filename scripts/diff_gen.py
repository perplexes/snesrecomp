#!/usr/bin/env python3
"""diff_gen.py — whole-pipeline output diff for the Rust regen vs Python regen.

Compares two generated-C output dirs file-by-file. For each output file reports:
  - byte-identical?
  - if not: the SET of emitted function names (RecompReturn NAME_MxXy defs)
    present in each side (functional equivalence = same function set + same
    bodies, even if function ORDER differs), plus a short unified-diff sample.

Also diffs funcs.h if present in both dirs.

Usage:
    python3 scripts/diff_gen.py /tmp/py_gen /tmp/rust_gen [--funcs-h A.h B.h]
"""
import argparse
import difflib
import pathlib
import re
import sys

DEF_RE = re.compile(r'^RecompReturn\s+([A-Za-z0-9_]+_M[01]X[01])\(CpuState')
# void alias defs (one per named entry): `void NAME(CpuState *cpu) {`
ALIAS_RE = re.compile(r'^void\s+([A-Za-z0-9_]+)\s*\(CpuState')

NAMED_DEF_RE = re.compile(r'RecompReturn ([A-Za-z_][A-Za-z0-9_]*_M[01]X[01])\(')
SYNTH_RE = re.compile(r'^bank_[0-9a-fA-F]{2}_[0-9a-fA-F]{4}_M[01]X[01]$')


def named_variant_set(out_dir: pathlib.Path) -> set:
    """Global set of NAMED (non-synthetic) emitted RecompReturn variant defs
    across every .c file. Synthetic `bank_XX_YYYY_MmXx` are excluded — they are
    auto-promoted bodies whose presence/ordering is order-driven. A canonical
    cfg-declared variant MUST appear here on both sides; any asymmetry is a real
    functional divergence (e.g. a dropped canonical variant -> dispatch MISS)."""
    s = set()
    for p in out_dir.glob('*.c'):
        for line in p.read_text(encoding='utf-8', errors='replace').split('\n'):
            for m in NAMED_DEF_RE.finditer(line):
                nm = m.group(1)
                if not SYNTH_RE.match(nm):
                    s.add(nm)
    return s


def func_set(text: str) -> set:
    """Set of emitted function names (variant bodies + void aliases)."""
    s = set()
    for line in text.split('\n'):
        m = DEF_RE.match(line)
        if m:
            s.add(m.group(1))
            continue
        m = ALIAS_RE.match(line)
        if m:
            s.add('void:' + m.group(1))
    return s


def body_map(text: str) -> dict:
    """name -> body text (def line until the next def/alias or EOF). Lets us
    compare bodies independent of function ORDER within the file."""
    lines = text.split('\n')
    bodies = {}
    cur_name = None
    cur = []
    for line in lines:
        md = DEF_RE.match(line)
        ma = ALIAS_RE.match(line)
        if md or ma:
            if cur_name is not None:
                bodies[cur_name] = '\n'.join(cur)
            cur_name = md.group(1) if md else ('void:' + ma.group(1))
            cur = [line]
        elif cur_name is not None:
            cur.append(line)
    if cur_name is not None:
        bodies[cur_name] = '\n'.join(cur)
    return bodies


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('py_dir')
    ap.add_argument('rust_dir')
    ap.add_argument('--funcs-h', nargs=2, default=None,
                    help='Two funcs.h paths to diff (py rust).')
    ap.add_argument('--sample', type=int, default=12,
                    help='Lines of diff sample to show per differing file.')
    args = ap.parse_args()

    py = pathlib.Path(args.py_dir)
    rust = pathlib.Path(args.rust_dir)

    py_files = {p.name for p in py.glob('*.c')}
    rust_files = {p.name for p in rust.glob('*.c')}

    only_py = sorted(py_files - rust_files)
    only_rust = sorted(rust_files - py_files)
    common = sorted(py_files & rust_files)

    print(f"=== OUTPUT FILE SET ===")
    print(f"  py: {len(py_files)}  rust: {len(rust_files)}  common: {len(common)}")
    if only_py:
        print(f"  ONLY IN PY:   {only_py}")
    if only_rust:
        print(f"  ONLY IN RUST: {only_rust}")

    identical = 0
    same_funcset_diff_body = []
    diff_funcset = []
    body_diff_total = 0

    for name in common:
        a = (py / name).read_text(encoding='utf-8', errors='replace')
        b = (rust / name).read_text(encoding='utf-8', errors='replace')
        if a == b:
            identical += 1
            continue
        fa, fb = func_set(a), func_set(b)
        if fa == fb:
            # Same function set; compare bodies order-independently.
            ba, bb = body_map(a), body_map(b)
            differing = sorted(n for n in (set(ba) & set(bb)) if ba[n] != bb[n])
            body_diff_total += len(differing)
            same_funcset_diff_body.append((name, len(differing), differing[:6]))
        else:
            diff_funcset.append((name, sorted(fa - fb)[:8], sorted(fb - fa)[:8]))

    print()
    print(f"=== PER-FILE RESULT ({len(common)} common files) ===")
    print(f"  byte-identical:            {identical}/{len(common)}")
    print(f"  same func-set, body diffs: {len(same_funcset_diff_body)} files, "
          f"{body_diff_total} differing bodies")
    print(f"  DIFFERENT func-set:        {len(diff_funcset)} files")

    if diff_funcset:
        print()
        print("=== FUNC-SET DIFFERENCES (functional inequivalence) ===")
        for name, only_a, only_b in diff_funcset:
            print(f"  {name}:")
            if only_a:
                print(f"    only in PY:   {only_a}")
            if only_b:
                print(f"    only in RUST: {only_b}")

    if same_funcset_diff_body:
        print()
        print("=== SAME FUNC-SET, BODY DIFFS (sample) ===")
        for name, n, sample in same_funcset_diff_body[:20]:
            print(f"  {name}: {n} differing bodies; e.g. {sample}")
        # Show one concrete body diff sample.
        nm0, _, sample0 = same_funcset_diff_body[0]
        if sample0:
            a = (py / nm0).read_text(errors='replace')
            b = (rust / nm0).read_text(errors='replace')
            ba, bb = body_map(a), body_map(b)
            fn = sample0[0]
            print()
            print(f"--- sample body diff: {nm0} :: {fn} ---")
            d = list(difflib.unified_diff(
                ba[fn].split('\n'), bb[fn].split('\n'),
                'py', 'rust', lineterm=''))
            for line in d[:args.sample * 2]:
                print('  ' + line)

    # funcs.h
    if args.funcs_h:
        pa, pb = args.funcs_h
        ta = pathlib.Path(pa).read_text(errors='replace')
        tb = pathlib.Path(pb).read_text(errors='replace')
        print()
        print("=== funcs.h ===")
        if ta == tb:
            print("  byte-identical")
        else:
            la = ta.split('\n')
            lb = tb.split('\n')
            sa = {x for x in la if x.startswith(('void ', 'RecompReturn '))}
            sb = {x for x in lb if x.startswith(('void ', 'RecompReturn '))}
            print(f"  decl-line count py={len(sa)} rust={len(sb)}")
            print(f"  only py:   {len(sa - sb)}")
            print(f"  only rust: {len(sb - sa)}")
            for x in sorted(sa - sb)[:8]:
                print(f"    -PY   {x}")
            for x in sorted(sb - sa)[:8]:
                print(f"    +RUST {x}")

    # ── Global NAMED-variant gate (authoritative) ──────────────────────────
    # A canonical cfg-declared variant dropped on one side is a real functional
    # divergence (NULL dispatch slot -> runtime MISS). Excludes synthetic
    # auto-promoted bodies (order-driven).
    na, nb = named_variant_set(py), named_variant_set(rust)
    only_py_named = sorted(na - nb)
    only_rust_named = sorted(nb - na)
    print()
    print("=== NAMED-VARIANT SET (synthetic excluded) ===")
    print(f"  py named: {len(na)}  rust named: {len(nb)}")
    print(f"  only in PY:   {len(only_py_named)}")
    print(f"  only in RUST: {len(only_rust_named)}")
    for x in only_py_named[:20]:
        print(f"    -PY   {x}")
    for x in only_rust_named[:20]:
        print(f"    +RUST {x}")

    named_ok = not only_py_named and not only_rust_named
    ok = (not only_py and not only_rust and not diff_funcset and named_ok)
    print()
    print(f"RESULT: {'FUNCTIONAL-EQUIVALENT' if ok else 'DIVERGENT'} "
          f"(byte-identical {identical}/{len(common)}; "
          f"named-variant gate {'PASS' if named_ok else 'FAIL'})")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
