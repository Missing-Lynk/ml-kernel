#!/usr/bin/env python3
"""
Measure how much of the sensor tuning blob this stack decodes.

The blob is 0xd6c58 bytes and the vendor loads it whole: `get_current_res_setting`
at 0x17fea0 compares the file length exactly against that, allocates it, reads it
in one go, and caches it per resolution. There is no checksum and no table of
contents, so the structure has to come from the code that reads it.

Two layers are reported.

Both layers come from `blob-layout.toml`, which is the one place a blob offset is written down.
This file used to carry its own copies, which is how four offsets the kernel decodes came to be
absent from the report.

**Records.** Every submodule reads its enable gate out of the blob through one
fixed chain (`isp_get_tuning_manager`, the per-sensor array at +544, that
sensor's image at +24, then a literal displacement). `isp-gates.py` derives those
displacements from the library, and in ascending order they bound the records:
each gate is the head of its module's record, so the next gate is the end of it.
A ladder's enable word is its gate, so the anchors are the layout's gate sections
plus each header's `enable` field. Fewer are recovered than the 54 banks, so a
span here holds one record or several. Treat a record size as an upper bound,
not a struct member.

**Fields.** Every section of the layout: spans with a known extent, and mostly a
known interpretation. This is the number that says how much of the file is
understood, and a section of kind `opaque` counts as extent-only.

Raw percentage-of-file is the wrong headline: for any one sensor most of the blob
is zero, because the file carries slots for stages this unit does not run. The
denominators that mean something are the non-zero bytes and, given a second and
third sensor's blob, the bytes that differ between them, which is the actual
per-sensor tuning payload.

    kernel/scripts/isp/blob-coverage.py \\
        --tuning out/air-gather/vendor-root/usr/usrdata/tunning/nt99235_tuning_preview_fpv.bin \\
        --compare out/air-gather/vendor-root/usr/usrdata/tunning/sc231_tuning_preview_fpv.bin \\
                  out/air-gather/vendor-root/usr/usrdata/tunning/sc2210_tuning_preview_fpv.bin
"""

import argparse
import pathlib
import sys

from blob_layout import Layout

# The layout is the only place a blob offset is written down. This file used to carry its own
# FIELDS and FLAGS tables, which is how four offsets the kernel decodes came to be missing from
# the coverage report.
LAYOUT = Layout.load()
BLOB_SIZE = LAYOUT.size
FLAGS = LAYOUT.anchors()
FIELDS = LAYOUT.spans()

# Gap folding uses one record header as its scale; the lsc record's is the smallest we know.
LSC_HEADER = 0x38


def merge(spans):
    """Overlapping and touching spans folded into disjoint ones."""
    out = []
    for start, end in sorted(spans):
        if out and start <= out[-1][1]:
            out[-1][1] = max(out[-1][1], end)
        else:
            out.append([start, end])

    return out


def owner(offset):
    """The record a byte falls in, or the unnamed head of the file."""
    name = '(head)'
    for candidate, at in FLAGS:
        if at > offset:
            return name

        name = candidate

    return name


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tuning', required=True, type=pathlib.Path,
                    help='the sensor tuning blob this stack ships against')
    ap.add_argument('--compare', nargs='*', default=(), type=pathlib.Path,
                    help='other sensors\' blobs, which separate per-sensor '
                         'tuning from structure shared by all of them')
    ap.add_argument('--records', action='store_true',
                    help='print the record table as well as the totals')
    args = ap.parse_args()

    for path in (args.tuning, *args.compare):
        if not path.exists():
            sys.exit(f'{path}: not found. The tuning blobs are capture '
                     f'artifacts and are deliberately not in the tree.')

    blob = args.tuning.read_bytes()
    if len(blob) != BLOB_SIZE:
        sys.exit(f'{args.tuning}: {len(blob)} bytes, not the {BLOB_SIZE} the '
                 f'vendor loader compares against, so this is not a tuning '
                 f'blob of the shape everything here assumes')

    others = []
    for path in args.compare:
        data = path.read_bytes()
        if len(data) != BLOB_SIZE:
            sys.exit(f'{path}: {len(data)} bytes, not {BLOB_SIZE}')

        others.append(data)

    covered = merge([[start, start + n] for start, n, _ in FIELDS])
    if covered[-1][1] > BLOB_SIZE:
        sys.exit(f'a field span ends at {covered[-1][1]:#x}, past the end of '
                 f'the blob, so the field table is wrong')

    decoded = bytearray(BLOB_SIZE)
    for start, end in covered:
        decoded[start:end] = b'\x01' * (end - start)

    print(f'{args.tuning.name}: {BLOB_SIZE} bytes, {len(FLAGS)} records '
          f'anchored, {len(FIELDS)} field spans decoded\n')

    if args.records:
        bounds = [at for _, at in FLAGS] + [BLOB_SIZE]
        print('record        offset      size   decoded')
        for i, (name, at) in enumerate(FLAGS):
            end = bounds[i + 1]
            got = sum(decoded[at:end])
            share = 100.0 * got / (end - at)
            print(f'  {name:<16} {at:#08x} {end - at:>9} {got:>9} '
                  f'{share:5.1f}%  {"#" * int(share / 2.5)}')

        print()

    def report(label, indices):
        indices = list(indices)
        if not indices:
            print(f'  {label:<32} {"none":>21}')
            return

        got = sum(1 for i in indices if decoded[i])
        print(f'  {label:<32} {got:>8} / {len(indices):<8} {100.0 * got / len(indices):5.1f}%')

    print('decoded, against each denominator:')
    report('every byte', range(BLOB_SIZE))
    nonzero = [i for i in range(BLOB_SIZE) if blob[i]]
    report('non-zero for this sensor', nonzero)

    if others:
        varying = [i for i in range(BLOB_SIZE)
                   if any(other[i] != blob[i] for other in others)]
        report(f'differing across {len(others) + 1} sensors', varying)
        payload = [i for i in varying if blob[i]]
        report('non-zero and sensor-specific', payload)

        # Fold gaps separated by less than a record header, which are one
        # undecoded structure with zero fields inside it rather than two. Never
        # fold across a record boundary: that would attribute a span to the
        # wrong module, which is the one error this report must not make.
        folded = []
        for start, end in merge([[i, i + 1] for i in payload if not decoded[i]]):
            if (folded and start - folded[-1][1] <= LSC_HEADER * 4
                    and owner(start) == owner(folded[-1][0])):
                folded[-1][1] = end
            else:
                folded.append([start, end])

        print(f'\nundecoded sensor-specific regions: {len(folded)}, largest '
              f'first')
        for start, end in sorted(folded, key=lambda g: g[1] - g[0],
                                 reverse=True)[:12]:
            print(f'  {start:#08x} .. {end - 1:#08x} {end - start:>8} bytes  '
                  f'in the {owner(start)} record')

    silent = [name for name, at in FLAGS
              if sum(decoded[at:at + 8]) and not sum(decoded[at + 8:
                     next((o for _, o in FLAGS if o > at), BLOB_SIZE)])]
    print(f'\n{len(silent)} records where only the gate is decoded:')
    print('  ' + ', '.join(silent))

    return 0


if __name__ == '__main__':
    sys.exit(main())
