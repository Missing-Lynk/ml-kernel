#!/usr/bin/env python3
"""
Map the vendor's AWB implementation in libmpp_service.so.

The algorithm never runs on this hardware. Four captures spanning warm lamp,
cold lamp and daylight (out/au-awb/) leave wb, ccm1, lsc and the awbs_stats
bank bit-identical; the awbs_stats enable bit at 0x6c00 is clear and the blob
gate at 0x0bbe98 reads 0. So this is a map of code the vendor ships and gates
off, kept because implementing AWB is a beyond-vendor item that will need it,
and NOT an input to vendor parity. There is no oracle to check it against, and
a capture cannot produce one while the gate is closed.

Attribution here is NOT the per-file walk isp-gates.py uses for the stage
gates. That walk works because every isp_sub_*.c materialises its own path for
dlog_printf_full(level, file, func, line, fmt); the 3A algorithm files log
through a macro that passes only __func__, so the file walk finds one function
per 3A file and stops. Names come from archive/re/symbols/namemap.py instead,
which resolves the __func__ argument itself and is what mapped the AE track.

For each named function this also reports, from isp-gates.py's walker:

  the ISP registers it stores to through a mapped bank, so the banks AWB
  would drive are a measurement rather than the inference in
  plans/isp-vendor-parity.md that it drives wb, ccm1 and the LSC group
  selection;

  the tuning-file offsets it reads through the tuning-manager chain.

The library is proprietary and is not in the repository.

    kernel/scripts/isp/awb-map.py \\
        --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so
"""

import argparse
import importlib.util
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
NAMEMAP = HERE.parent.parent.parent / 'archive' / 're' / 'symbols' / 'namemap.py'

# The span the AWB algorithm occupies, from a whole-image namemap run: it opens
# after the AE library and closes where the AF entry points begin. Widened here
# by a page at each end so a different build cannot silently truncate the map.
AWB_LO, AWB_HI = 0x26C000, 0x27B000

# Names that are AWB's rather than a neighbour's. namemap attributes by call
# site, so an AE or AF helper called from this span shows up here too.
KEEP = ('awb', 'cct', 'cluster', 'white', 'grey', 'noon', 'lum_distance',
        'constrct', 'construct', 'filter_result')


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)

    return mod


def disassemble(lib, out, raw):
    """objdump the library; namemap needs the encoding column, the walker does not."""
    cmd = ['aarch64-linux-gnu-objdump', '-d']
    if raw:
        cmd.append('--show-raw-insn')

    with open(out, 'w') as f:
        subprocess.run(cmd + [lib], stdout=f, check=True)

    return out


def isp_writes(g, lib, func):
    """
    ISP register offsets this function stores to through a mapped bank.

    Same base tracking as the gate walk: a pointer from ar_dev_pa2va(), plus
    the displacement of the store. Offsets are reported undecoded, because
    which banks AWB drives is the question and a decode would assume it.
    """
    base, out = {}, set()
    for _addr, op, a in lib.body.get(func, []):
        if op == 'bl' and a and a[0] == lib.plt.get('ar_dev_pa2va'):
            base = {'x0': 0}
            continue

        if op in ('str', 'stur') and len(a) >= 2:
            m = g.MEM.match(a[1])
            if m and m.group(1) in base:
                out.add(base[m.group(1)] + int(m.group(2) or 0))
            continue

        if op in ('mov', 'add') and len(a) >= 2 and a[1] in base:
            delta = g.imm(a[2]) if len(a) > 2 else 0
            if delta is not None:
                base[a[0]] = base[a[1]] + delta
            continue

        if op not in g.READS_ONLY and a:
            base.pop(a[0], None)

    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lib', required=True, help='libmpp_service.so')
    ap.add_argument('--all', action='store_true',
                    help='keep every name in the span, not only AWB\'s')
    args = ap.parse_args()

    nm = load(NAMEMAP, 'namemap')
    g = load(HERE / 'isp-gates.py', 'isp_gates')

    with tempfile.TemporaryDirectory() as tmp:
        dis = disassemble(args.lib, Path(tmp) / 'raw.dis', raw=True)
        code = nm.load_code(str(dis))
        _hits, sites = nm.attribute(code, nm.load_names(args.lib),
                                    nm.function_starts(code))

    # One row per name at its earliest call site: a name reaches its argument
    # register once per logging call, and the first is inside the function that
    # owns it.
    first = {}
    for addr, name in sites:
        if AWB_LO <= addr < AWB_HI and (addr < first.get(name, addr + 1)):
            first[name] = addr

    if not args.all:
        first = {n: a for n, a in first.items()
                 if any(k in n for k in KEEP)}

    if not first:
        sys.exit('no AWB names in the span; the image may differ')

    lib = g.Library(args.lib)

    print(f'{"addr":10s} {"function":42s} isp / tuning')
    print('-' * 78)
    regs_all, offs_all = set(), set()
    for name, addr in sorted(first.items(), key=lambda kv: kv[1]):
        func = lib.func_of(addr)
        regs = isp_writes(g, lib, func) if func else set()
        offs = {o for o in g.tuning_offsets(lib, func)} if func else set()
        regs_all |= regs
        offs_all |= offs

        note = []
        if regs:
            note.append('isp ' + ' '.join(f'0x{r:04x}'
                                          for r in sorted(regs)[:6]))
        if offs:
            note.append('tuning ' + ' '.join(f'0x{o:06x}'
                                             for o in sorted(offs)[:4]))
        print(f'0x{addr:06x}   {name:42s} {"  ".join(note)}')

    print(f'\nISP registers written: '
          f'{" ".join(f"0x{r:04x}" for r in sorted(regs_all)) or "none"}')
    print(f'tuning offsets read: '
          f'{" ".join(f"0x{o:06x}" for o in sorted(offs_all)) or "none"}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
