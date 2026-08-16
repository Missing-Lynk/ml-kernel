#!/usr/bin/env python3
"""
Measure which ar_isp_vendor_trim writes actually reach the hardware.

The trim pass replays values measured off a streaming vendor unit. Some of
those writes land, some are overwritten by a later applier, and some go to
registers the block owns and never take effect at all. Only the last group can
be dropped safely, and telling the three apart needs the device: a snapshot
alone cannot distinguish "the write did nothing" from "something wrote it
afterwards".

This is the A/B. Take one register sweep with the trim pass on and one with it
off, and compare:

    a register that DIFFERS between the two           the trim write reaches it
    a register that MATCHES, at the trim value        a later applier writes the
                                                      same value; trim is
                                                      redundant but not wrong
    a register that MATCHES, at some other value      the trim write never takes
                                                      effect and can be skipped

ar-isp-main.c already records one run of this over 92 entries. It left 0x7054
in place because the write lands in its high half only, and nine entries were
on pages that capture did not dump. The sweep layout in use now covers all of
them, so a rerun settles the remainder.

On the device, with the module already loaded:

    echo 1 > /sys/module/ar_isp/parameters/trim   (then restart the pipeline)
    ... capture a sweep, save as trim-on.txt
    echo 0 > /sys/module/ar_isp/parameters/trim   (then restart the pipeline)
    ... capture a sweep, save as trim-off.txt

Both sweeps must come from the same boot and the same scene, or the counters
move for reasons that have nothing to do with the trim pass.

    kernel/scripts/isp/check-trim-effect.py --on trim-on.txt --off trim-off.txt
"""

import argparse
import importlib.util
import pathlib
import re
import sys
from types import ModuleType

HERE = pathlib.Path(__file__).resolve().parent

# The registers this audit cannot source without a device, from
# audit-provenance.py's HARDWARE_OWNED and DMA_NOT_OVERWRITTEN. These are what
# the rerun is for; everything else is reported for completeness.
WANTED = (0x00EC, 0x6060, 0x6478, 0x647C, 0x6514, 0x6518, 0x651C, 0x7054,
          0x705C, 0x1D68, 0x1DC0, 0x1E5C, 0x1F88, 0x5810, 0x5828, 0x5840,
          0x5858, 0x6C90, 0x6D38, 0x7574, 0x758C, 0x75A0, 0x75BC)


def load_audit() -> ModuleType:
    path = HERE / 'audit-provenance.py'
    spec = importlib.util.spec_from_file_location('ar_isp_audit', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    source = path.read_text().replace(
        "if __name__ == '__main__':\n    sys.exit(main())", '')
    exec(compile(source, str(path), 'exec'), mod.__dict__)

    return mod


def read_sweep(path: pathlib.Path) -> dict[int, int]:
    """
    Every register a capture dumps, by absolute offset.

    Two capture layouts are in the tree and both are read here. The snapshot
    scripts emit "--- isp +0x1800 (64 words) ---" with offsets relative to that
    base; au-chain-capture.sh emits "SECTION isp-18" with offsets relative to the
    page. The A/B sweeps in out/au-trim-ab/ are the second kind, so reading only
    the first made the one measurement this script exists for unrunnable.
    """
    out: dict[int, int] = {}
    base = None
    for line in path.read_text().splitlines():
        hit = re.match(r'---\s*isp\s*\+(0x[0-9a-f]+)', line)
        if hit:
            base = int(hit.group(1), 16)
            continue

        hit = re.match(r'SECTION isp-([0-9a-f]{2})\b', line)
        if hit:
            base = int(hit.group(1), 16) << 8
            continue

        # Any other section is a different block; its offsets are not ours.
        if line.startswith('SECTION'):
            base = None
            continue

        hit = re.match(r'\s*\+(0x[0-9a-f]+):\s*(.*)', line)
        if hit and base is not None:
            off = base + int(hit.group(1), 16)
            for i, word in enumerate(hit.group(2).split()):
                if re.fullmatch(r'[0-9a-f]{8}', word):
                    out[off + 4 * i] = int(word, 16)

    if not out:
        sys.exit(f'{path}: no "--- isp +0x...." or "SECTION isp-.." blocks '
                 f'found, so this is not a register sweep in either format '
                 f'this script reads')

    return out


def load_generator() -> ModuleType:
    """gen-isp-defaults.py, for the exclusion list and its reasons."""
    path = HERE / 'gen-isp-defaults.py'
    spec = importlib.util.spec_from_file_location('ar_isp_gen', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)

    return mod


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--on', required=True, type=pathlib.Path,
                    help='a sweep taken with the trim pass enabled')
    ap.add_argument('--off', required=True, type=pathlib.Path,
                    help='a sweep taken with it disabled, same boot')
    ap.add_argument('--control', nargs=2, type=pathlib.Path,
                    metavar=('ONE', 'TWO'),
                    help='two sweeps taken at the SAME trim setting. Any '
                         'register that moves between them is free-running, '
                         'and the A/B verdict for it is withheld rather than '
                         'reported as a response to the trim pass')
    ap.add_argument('--all', action='store_true',
                    help='report every trim entry, not just the ones that '
                         'block the provenance audit')
    args = ap.parse_args()

    for path in (args.on, args.off, *(args.control or ())):
        if not path.exists():
            sys.exit(f'{path}: not found')

    on, off = read_sweep(args.on), read_sweep(args.off)

    # Two identical sweeps make every register "match", which this script would
    # then read as "the write never takes effect" for anything whose value is
    # not the trim value. That is the one way to get a confidently wrong skip
    # list out of it, so refuse rather than report.
    shared = on.keys() & off.keys()
    if shared and all(on[r] == off[r] for r in shared):
        sys.exit(f'{args.on.name} and {args.off.name} agree on all '
                 f'{len(shared)} shared registers. Either the trim parameter '
                 f'did not change between the two captures, or the pipeline '
                 f'was not restarted so the second sweep still shows the '
                 f'first configuration. This is not a valid A/B and the skip '
                 f'list it would produce would be wrong.')
    audit = load_audit()
    trim = audit.reg_arrays(audit.DEFAULTS).get('ar_isp_vendor_trim')
    if not trim:
        sys.exit('ar_isp_vendor_trim is not in the generated header, so there '
                 'is no trim pass to measure')

    written = dict(trim)
    excluded = load_generator().TRIM_EXCLUDED
    look = sorted(written) if args.all else [r for r in WANTED if r in written]

    # A register that was measured and then dropped leaves the trim table, and
    # reporting nothing about it would make the exclusion untestable from the
    # moment it lands. The sweeps still cover it, so its current state is shown
    # against the reason it was excluded for.
    gone = [r for r in WANTED if r not in written and r in excluded]
    if gone and not args.all:
        print('excluded from the trim table by gen-isp-defaults.py. The A/B no '
              'longer has a written value to compare, so this is the state the '
              'exclusion leaves behind:\n')
        for reg in gone:
            a, b = on.get(reg), off.get(reg)
            state = ('not in the sweep' if a is None or b is None
                     else f'on {a:#010x}, off {b:#010x}')
            print(f'  {reg:#06x}  {state}\n          {excluded[reg]}')

        print()

    absent = [f'{r:#06x}' for r in WANTED
              if r not in written and r not in excluded and not args.all]
    if absent:
        print(f'not in the trim table and not excluded, so this A/B says '
              f'nothing about them: {", ".join(absent)}\n')

    control: set[int] = set()
    if args.control:
        c1, c2 = (read_sweep(p) for p in args.control)
        shared = c1.keys() & c2.keys()
        control = {r for r in shared if c1[r] != c2[r]}
        print(f'control pair: {len(shared)} shared registers, {len(control)} '
              f'move between two captures at the same trim setting. Those are '
              f'free-running and this A/B cannot judge them.\n')

    verdicts: dict[str, list[int]] = {}
    print(f'{"reg":>8} {"trim writes":>12} {"on":>11} {"off":>11}  verdict')
    for reg in look:
        a, b = on.get(reg), off.get(reg)
        if a is None or b is None:
            verdict = 'not in the sweep'
        elif reg in control:
            verdict = ('FREE-RUNNING: it moves across the control pair too, so '
                       'this A/B cannot judge it')
        elif a != b:
            verdict = ('the write reaches it' if a == written[reg]
                       else f'changes, but settles on {a:#x} not the written '
                            f'value')
        elif a == written[reg]:
            verdict = 'a later applier writes the same value; trim redundant'
        else:
            verdict = 'THE WRITE NEVER TAKES EFFECT: safe to skip'

        verdicts.setdefault(verdict.split(':')[0], []).append(reg)
        def show(v: int | None) -> str:
            return '--' if v is None else f'{v:#010x}'

        print(f'{reg:#08x} {written[reg]:>#12x} {show(a):>11} {show(b):>11}  '
              f'{verdict}')

    skippable = verdicts.get('THE WRITE NEVER TAKES EFFECT', [])
    print(f'\n{len(skippable)} of {len(look)} can be dropped from the trim '
          f'table with no observable change')
    if skippable:
        print('  ' + ', '.join(f'{r:#06x}' for r in skippable))
        print('\nAdd exactly these to TRIM_EXCLUDED in gen-isp-defaults.py, '
              'each with the verdict this run produced, and drop the same rows '
              'from the generated header. The exclusion has to live in the '
              'generator: an edit to the header alone is undone by the next '
              'regeneration, and audit-provenance.py fails when the two '
              'disagree. Do not add any register this run reports differently, '
              'and do not add one the sweep did not cover.')

    partial = [r for r in look if on.get(r) is not None and r not in control
               and on[r] != off.get(r) and on[r] != written[r]]
    if partial:
        print(f'\n{len(partial)} differ between the two runs without settling '
              f'on the written value: ' + ', '.join(f'{r:#06x}' for r in partial))
        print('  **For a free-running counter this test cannot conclude '
              'anything.** A counter differs between any two captures whether '
              'or not the trim pass ran, so "it changed" is not evidence the '
              'write mattered. Separating the two needs a control: two '
              'captures at the SAME trim setting. A register that differs '
              'across that control is free-running and this A/B cannot judge '
              'it; one that holds still there is genuinely responding to trim.')

    return 0


if __name__ == '__main__':
    sys.exit(main())
