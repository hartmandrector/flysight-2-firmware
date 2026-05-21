"""
mfx_query.py - Query an ARM Thumb-2 objdump disassembly for MotionFX library analysis.

The intended workflow when porting a new MotionFX function:
  1. Dump disassembly once:
       arm-none-eabi-objdump -d MotionFX_CM4F_wc32_ot_hard.a > Docs/MotionFX/mfx_disasm.txt
  2. Find every function's entry guards (conditional returns near top):
       python Scripts/mfx_query.py --extract SomeFunctionName
  3. For each guard, find what sets / clears that flag:
       python Scripts/mfx_query.py --writes-to 72
  4. Resolve the library global state pointer from a function's literal pool:
       python Scripts/mfx_query.py --pool 0x10c --function MotionFX_MagCal_init

Usage:
  python mfx_query.py [DISASM_FILE] [options]

  DISASM_FILE  path to disassembly (default: Docs/MotionFX/mfx_disasm.txt
               relative to this script's parent directory)

Options:
  --writes-to  N    find all store instructions that write [rx, #N]
  --reads-from N    find all load instructions that read [rx, #N]
  --pool       OFF  show literal pool entries at fn+OFF in every function
  --list-functions  list every function name and instruction count
  --extract    NAME extract all lines for functions whose name contains NAME
  --trace-ldr       list all PC-relative ldr/vldr instructions and their pool
                    targets; marks RELOCATION SLOT for entries not in the .a
  --function   NAME restrict --writes-to / --reads-from / --pool / --trace-ldr
                    to functions whose name contains NAME

Offsets N / OFF can be decimal (72) or hex (0x48).

Notes:
  - Pool entries for global variable addresses show as "..." in the pre-linked
    .a disassembly (relocation slots, value unknown until link time).  At
    runtime, read the linked value with: *(uint32_t*)((fn_ptr & ~1) + OFFSET)
  - Float/integer constants DO appear as .word in the .a pool.
"""

import re
import sys
import os
import argparse

# ---------------------------------------------------------------------------
# Default path: <repo>/Docs/MotionFX/mfx_disasm.txt
# ---------------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DISASM = os.path.normpath(
    os.path.join(_SCRIPT_DIR, '..', 'Docs', 'MotionFX', 'mfx_disasm.txt')
)

# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------
# Function header:  "00000000 <Name>:"
_FUNC_HDR = re.compile(r'^[0-9a-f]+ <([^>]+)>:\s*$')

# Instruction line: "   b0:   f884 6048   strb.w  r6, [r4, #72]"
# Also captures literal pool words:
#                  "  10c:   2000029c   .word   0x2000029c"
_INSN_LINE = re.compile(r'^\s+([0-9a-f]+):\s+(?:[0-9a-f]{4,8}\s+)+(.+)')


def _parse(path):
    """Return list of (func_name, [(offset_int, full_line_stripped), ...])."""
    functions = []
    cur_name = None
    cur_lines = []

    with open(path, 'r', errors='replace') as fh:
        for raw in fh:
            m = _FUNC_HDR.match(raw)
            if m:
                if cur_name is not None:
                    functions.append((cur_name, cur_lines))
                cur_name = m.group(1)
                cur_lines = []
                continue
            m = _INSN_LINE.match(raw)
            if m and cur_name is not None:
                cur_lines.append((int(m.group(1), 16), raw.rstrip()))

    if cur_name is not None:
        functions.append((cur_name, cur_lines))
    return functions


def _norm(s):
    """Parse a decimal or hex integer string."""
    s = s.strip()
    return int(s, 16) if s.lower().startswith('0x') else int(s)


def _offset_re(n):
    """Regex that matches [<reg>, #N] in either decimal or hex form."""
    return re.compile(
        rf'\[\s*\w+\s*,\s*#(?:{n}|0x{n:x})\b',
        re.IGNORECASE,
    )


def _store_re():
    return re.compile(r'str', re.IGNORECASE)


def _load_re():
    return re.compile(r'ldr', re.IGNORECASE)


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def cmd_writes_to(functions, n, func_filter):
    pat_off = _offset_re(n)
    pat_op = _store_re()
    found = False
    for fname, lines in functions:
        if func_filter and func_filter.lower() not in fname.lower():
            continue
        for off, raw in lines:
            # mnemonic is the first token after the hex dump
            parts = raw.split()
            # parts[0] is the offset "b0:" — find mnemonic after hex words
            # raw format: "   b0:   f884 6048   strb.w  r6, [r4, #72]"
            # split on the first non-hex token after the colon
            after = re.split(r'[0-9a-f]{4,8}\s+', raw, maxsplit=8)[-1].strip()
            mnemonic = after.split()[0] if after else ''
            if pat_op.match(mnemonic) and pat_off.search(after):
                print(f"  [{fname}+0x{off:x}]  {after}")
                found = True
    if not found:
        print(f"  (no stores to offset #{n} / #0x{n:x} found)")


def cmd_reads_from(functions, n, func_filter):
    pat_off = _offset_re(n)
    pat_op = _load_re()
    found = False
    for fname, lines in functions:
        if func_filter and func_filter.lower() not in fname.lower():
            continue
        for off, raw in lines:
            after = re.split(r'[0-9a-f]{4,8}\s+', raw, maxsplit=8)[-1].strip()
            mnemonic = after.split()[0] if after else ''
            if pat_op.match(mnemonic) and pat_off.search(after):
                print(f"  [{fname}+0x{off:x}]  {after}")
                found = True
    if not found:
        print(f"  (no loads from offset #{n} / #0x{n:x} found)")


def cmd_pool(functions, pool_off, func_filter):
    """Show the instruction / .word at fn+pool_off (literal pool resolution)."""
    found = False
    for fname, lines in functions:
        if func_filter and func_filter.lower() not in fname.lower():
            continue
        for off, raw in lines:
            if off == pool_off:
                after = re.split(r'[0-9a-f]{4,8}\s+', raw, maxsplit=8)[-1].strip()
                print(f"  [{fname}+0x{off:x}]  {after}")
                found = True
    if not found:
        print(f"  (nothing at offset 0x{pool_off:x})")


def cmd_list_functions(functions, func_filter):
    for fname, lines in functions:
        if func_filter and func_filter.lower() not in fname.lower():
            continue
        print(f"  {fname}  ({len(lines)} lines)")


def cmd_extract(functions, name):
    matched = 0
    for fname, lines in functions:
        if name.lower() not in fname.lower():
            continue
        print(f"\n{'='*70}")
        print(f"{fname}:")
        print('='*70)
        for _off, raw in lines:
            print(raw)
        matched += 1
    if not matched:
        print(f"  (no function matching '{name}' found)")


# Matches the objdump annotation in two forms:
#   ldr:  @ (10c <FuncName+0x10c>)   — with parentheses
#   vldr: @ 108 <FuncName+0x108>     — without parentheses
_POOL_ANNOT = re.compile(r'@\s+\(?([0-9a-f]+)\s+<[^>]+>\)?')

def cmd_trace_ldr(functions, func_filter):
    """Find all PC-relative ldr/vldr and show their pool targets."""
    ldr_re = re.compile(r'^v?ldr', re.IGNORECASE)
    pc_re  = re.compile(r'\[pc\b', re.IGNORECASE)

    # Build a lookup: (func_name, pool_offset) → raw line
    pool_lookup = {}
    for fname, lines in functions:
        for off, raw in lines:
            pool_lookup[(fname, off)] = raw

    found = False
    for fname, lines in functions:
        if func_filter and func_filter.lower() not in fname.lower():
            continue
        for off, raw in lines:
            after = re.split(r'[0-9a-f]{4,8}\s+', raw, maxsplit=8)[-1].strip()
            mnemonic = after.split()[0] if after else ''
            if not (ldr_re.match(mnemonic) and pc_re.search(after)):
                continue

            # Extract pool target offset from annotation
            m = _POOL_ANNOT.search(raw)
            if not m:
                continue
            pool_off = int(m.group(1), 16)

            pool_raw = pool_lookup.get((fname, pool_off))
            if pool_raw:
                pool_after = re.split(r'[0-9a-f]{4,8}\s+', pool_raw, maxsplit=8)[-1].strip()
                pool_desc = pool_after
            else:
                pool_desc = "RELOCATION SLOT  (value only known at link time)"

            print(f"  [{fname}+0x{off:x}]  {after}")
            print(f"      → pool[0x{pool_off:x}] = {pool_desc}")
            found = True

    if not found:
        print("  (no PC-relative loads found)")



# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description='Query an ARM Thumb-2 disassembly (MotionFX library analysis)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument('disasm', nargs='?', default=DEFAULT_DISASM,
                    help='Path to objdump disassembly file')
    ap.add_argument('--writes-to', metavar='N',
                    help='Find store instructions to [rx, #N]')
    ap.add_argument('--reads-from', metavar='N',
                    help='Find load instructions from [rx, #N]')
    ap.add_argument('--pool', metavar='OFFSET',
                    help='Show literal pool entries at fn+OFFSET')
    ap.add_argument('--list-functions', action='store_true',
                    help='List all function names')
    ap.add_argument('--extract', metavar='NAME',
                    help='Print all instructions for functions containing NAME')
    ap.add_argument('--trace-ldr', action='store_true',
                    help='List all PC-relative ldr/vldr instructions and pool targets')
    ap.add_argument('--function', metavar='NAME',
                    help='Restrict output to functions containing NAME')
    args = ap.parse_args()

    if not os.path.exists(args.disasm):
        print(f"ERROR: disassembly file not found: {args.disasm}", file=sys.stderr)
        print(f"Generate it with:", file=sys.stderr)
        print(f"  arm-none-eabi-objdump -d MotionFX_CM4F_wc32_ot_hard.a > mfx_disasm.txt",
              file=sys.stderr)
        sys.exit(1)

    print(f"Loading {args.disasm} ...", file=sys.stderr)
    functions = _parse(args.disasm)
    print(f"  {len(functions)} functions parsed", file=sys.stderr)

    any_cmd = False

    if args.writes_to:
        n = _norm(args.writes_to)
        print(f"\nStores to [rx, #{n}]  (0x{n:x}):")
        cmd_writes_to(functions, n, args.function)
        any_cmd = True

    if args.reads_from:
        n = _norm(args.reads_from)
        print(f"\nLoads from [rx, #{n}]  (0x{n:x}):")
        cmd_reads_from(functions, n, args.function)
        any_cmd = True

    if args.pool:
        off = _norm(args.pool)
        print(f"\nLiteral pool at fn+0x{off:x}:")
        cmd_pool(functions, off, args.function)
        any_cmd = True

    if args.list_functions:
        print("\nFunctions:")
        cmd_list_functions(functions, args.function)
        any_cmd = True

    if args.extract:
        print(f"\nExtracting '{args.extract}':")
        cmd_extract(functions, args.extract)
        any_cmd = True

    if args.trace_ldr:
        print("\nPC-relative loads (ldr/vldr [pc, #N]) and pool targets:")
        cmd_trace_ldr(functions, args.function)
        any_cmd = True

    if not any_cmd:
        ap.print_help()


if __name__ == '__main__':
    main()
