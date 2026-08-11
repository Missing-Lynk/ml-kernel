#!/usr/bin/env python3
"""
Locate the vendor code that writes a given ISP register.

Every submodule holds a shadow copy of its register bank, installs the
ISP-init template image into it, patches it from the command handlers, and
pushes the whole thing with isp_memcpy(bank_va, shadow, length). There are 233
such call sites. So a register whose value differs from its template image was
patched by a store somewhere in that module's own handlers, at an immediate
equal to the register's bank offset.

This finds those stores. For each register audit-provenance.py still reports as
unexplained it resolves the owning module, follows the module constructor to
its three handlers, and reports every store in them at the matching offset.
That turns each remaining register from a search into a read.

Two things it does not do. It does not say what the stored value is: an `stp`
writes two registers at once, and tracing the operand back to a tuning field or
a frame dimension is the actual recovery. And it reports stores to the stack
frame separately, because a spill to [x29, #48] matches offset 48 without
having anything to do with bank+0x30.

Needs the vendor library, which is not in the tree, and aarch64 objdump:

    kernel/scripts/isp/find-register-writer.py \\
        --library out/air-gather/vendor-root/usr/lib/libmpp_service.so
    kernel/scripts/isp/find-register-writer.py --library ... --register 0x4824
"""

import argparse
import importlib.util
import pathlib
import re
import shutil
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
OBJDUMP = 'aarch64-linux-gnu-objdump'

# How far past a handler entry to scan. The largest submodule dispatcher seen
# is a little over 900 instructions.
HANDLER_WINDOW = 950

# The struct offsets a constructor stores its handlers at.
HANDLER_SLOTS = (472, 480, 488, 496)

# Frame-pointer and stack-pointer bases. A store through either is a register
# spill that happens to share an immediate with a bank offset.
STACK_BASES = ('x29', 'sp')

# Bank name in isp-regdiff.py to the constructor that maps that bank, where the
# two spellings differ.
CREAT_ALIASES = {
    'lnr': 'lee_lnr',
    'raw_hist_stats': 'raw_his_stats',
    'rgb_hist_stats': 'rgb_his_stats',
    'rro_face_stats': 'rro_face_stats',
}


def load_audit():
    path = HERE / 'audit-provenance.py'
    spec = importlib.util.spec_from_file_location('ar_isp_audit', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    source = path.read_text().replace(
        "if __name__ == '__main__':\n    sys.exit(main())", '')
    exec(compile(source, str(path), 'exec'), mod.__dict__)

    return mod


def disassemble(library):
    if not shutil.which(OBJDUMP):
        sys.exit(f'{OBJDUMP} not found. It reads the vendor library, which is '
                 f'the only source for where a register is written.')

    out = subprocess.run([OBJDUMP, '-d', '--no-show-raw-insn', str(library)],
                         capture_output=True, text=True)
    if out.returncode:
        sys.exit(f'{OBJDUMP} failed on {library}: {out.stderr.strip()}')

    return out.stdout.splitlines()


def symbols(library):
    out = subprocess.run([OBJDUMP, '-T', str(library)],
                         capture_output=True, text=True).stdout
    found = {}
    for line in out.splitlines():
        hit = re.match(r'^([0-9a-f]+)\s+\S+\s+DF\s+\.text\s+\S+\s+\S+\s+(\w+)',
                       line)
        if hit:
            found[hit.group(2)] = int(hit.group(1), 16)

    return found


def index_by_address(asm):
    out = {}
    for i, line in enumerate(asm):
        hit = re.match(r'^\s*([0-9a-f]+):', line)
        if hit:
            out[int(hit.group(1), 16)] = i

    return out


def handlers_of(asm, at, entry):
    """
    The handler addresses a constructor stores into its module struct.

    Each is built with an adrp/add pair, so the page and the offset have to be
    folded before the store is read.
    """
    if entry not in at:
        return []

    pages, found = {}, []
    for line in asm[at[entry]:at[entry] + 70]:
        hit = re.search(r'adrp\s+(x\d+), ([0-9a-f]+)', line)
        if hit:
            pages[hit.group(1)] = int(hit.group(2), 16)
            continue

        hit = re.search(r'add\s+(x\d+), (x\d+), #(0x[0-9a-f]+)', line)
        if hit and hit.group(2) in pages:
            pages[hit.group(1)] = pages[hit.group(2)] + int(hit.group(3), 16)
            continue

        hit = re.search(r'str\s+(x\d+), \[x19, #(\d+)\]', line)
        if hit and hit.group(1) in pages:
            if int(hit.group(2)) in HANDLER_SLOTS:
                found.append(pages[hit.group(1)])

    return found


def stores_at(asm, at, entry, offset):
    """Every store in one handler whose immediate is `offset`."""
    if entry not in at:
        return [], []

    real, spills = [], []
    pattern = re.compile(
        r'^\s*([0-9a-f]+):\s+(str|stp|strb|strh)\s+\S+(?:, \S+)?, '
        r'\[(\w+)(?:, #%d)?\]' % offset)
    exact = re.compile(r'\[(\w+), #%d\]' % offset)
    for line in asm[at[entry]:at[entry] + HANDLER_WINDOW]:
        hit = pattern.match(line)
        if not hit:
            continue

        # Offset zero is written as [xN] with no immediate; anything else has
        # to match the immediate exactly.
        if offset and not exact.search(line):
            continue

        (spills if hit.group(3) in STACK_BASES else real).append(line.strip())

    return real, spills


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--library', required=True,
                    help='the vendor libmpp_service.so')
    ap.add_argument('--register', type=lambda v: int(v, 0),
                    help='one ISP register, instead of every unexplained one')
    ap.add_argument('--spills', action='store_true',
                    help='also list stores through the frame or stack pointer')
    args = ap.parse_args()

    library = pathlib.Path(args.library)
    if not library.exists():
        sys.exit(f'{library}: not found. The vendor library is a capture '
                 f'artifact and is deliberately not in the tree.')

    audit = load_audit()
    library_image, final, _origin = audit.load_tables()
    derived = audit.derived_registers()
    masks = audit.gate_masks()
    banks = audit.bank_lookup()

    def classify(off):
        value = final[off]
        if off in derived or library_image.get(off) == value:
            return 'accounted'
        if off in audit.EXPLAINED:
            return 'accounted'
        if value and off in masks and value & ~masks[off] == 0:
            return 'accounted'
        if not value or audit.DMA_LO <= value < audit.DMA_HI:
            return 'accounted'
        if audit.is_geometry(value) or audit.disabled_span(off):
            return 'accounted'

        return 'UNEXPLAINED'

    def bank_of(off):
        for base, name in banks:
            if off >= base:
                return base, name

        return 0, 'base'

    asm = disassemble(library)
    at = index_by_address(asm)
    syms = symbols(library)

    wanted = ([args.register] if args.register is not None
              else sorted(o for o in final if classify(o) == 'UNEXPLAINED'))
    if args.register is not None and args.register not in final:
        sys.exit(f'{args.register:#06x}: the driver does not write this '
                 f'register, so no module patches it')

    print(f'{len(wanted)} register(s), against {library.name}\n')
    located = 0
    for off in wanted:
        base, name = bank_of(off)
        rel = off - base
        creat = f'isp_sub_{CREAT_ALIASES.get(name, name)}_creat'
        entry = syms.get(creat)

        print(f'{off:#06x} = {final[off]:#010x}  {name} bank+{rel:#05x}')
        if entry is None:
            print(f'  no constructor named {creat}; the bank belongs to the '
                  f'ISP top level or to a module under another name\n')
            continue

        # Handler windows overlap when a module's handlers sit close
        # together, so the same store can be seen more than once.
        hits, spills = [], []
        for handler in handlers_of(asm, at, entry):
            real, spilled = stores_at(asm, at, handler, rel)
            hits += [line for line in real if line not in hits]
            spills += [line for line in spilled if line not in spills]

        if hits:
            located += 1
            for line in hits:
                print(f'  {line}')
        else:
            print('  no store at that offset in its own handlers')

        if args.spills and spills:
            print(f'  ({len(spills)} stack spill(s) share the immediate)')

        print()

    print(f'{located} of {len(wanted)} have a located writer')

    return 0


if __name__ == '__main__':
    sys.exit(main())
