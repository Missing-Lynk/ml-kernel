#!/usr/bin/env python3
"""
Measure how much of the sensor tuning blob this stack decodes.

The blob is 0xd6c58 bytes and the vendor loads it whole: `get_current_res_setting`
at 0x17fea0 compares the file length exactly against that, allocates it, reads it
in one go, and caches it per resolution. There is no checksum and no table of
contents, so the structure has to come from the code that reads it.

Two layers are reported.

**Records.** Every submodule reads its enable gate out of the blob through one
fixed chain (`isp_get_tuning_manager`, the per-sensor array at +544, that
sensor's image at +24, then a literal displacement). `isp-gates.py` derives those
displacements from the library, and in ascending order they bound the records:
each gate is the head of its module's record, so the next gate is the end of it.
29 are recovered, which is fewer than the 54 banks, so a span here holds one
record or several. Treat a record size as an upper bound, not a struct member.

**Fields.** Spans a value is actually read out of, with a known meaning and
extent, taken from the generators and checkers in this directory. This is the
number that says how much of the file is understood.

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

BLOB_SIZE = 0xD6C58

# Module gate offsets, ascending. Reproduce with:
#   isp-gates.py --lib libmpp_service.so --blob <tuning> --all
# and read the flag column. They are derived from the library, not guessed.
FLAGS = (
    ('blc', 0x000024), ('compander', 0x006CC4), ('dpc', 0x006CE8),
    ('rnr', 0x0079D8), ('decompander', 0x00906C), ('lsc', 0x009090),
    ('drc', 0x017A88), ('gib', 0x0243E4), ('ccm1', 0x0253FC),
    ('ccm2', 0x02595C), ('gamma', 0x026AFC), ('gtm2', 0x07ABD8),
    ('lut3d', 0x07B634), ('cm', 0x089CFC), ('lee_lnr', 0x089E88),
    ('cnf', 0x08E198), ('cm2', 0x0A1304), ('hdr_lsc', 0x0A16C0),
    ('acm', 0x0B00B8), ('lms', 0x0B288C), ('ir_rnr', 0x0B2BF4),
    ('birnr', 0x0B3550), ('digigain1', 0x0B4000), ('ir_lms_horz', 0x0B4EE4),
    ('raw_3dnr', 0x0B500C), ('binning_filter', 0x0B6348),
    ('derolling_stats', 0x0B64F8), ('awbs_stats', 0x0BBE98),
    ('af_stats', 0x0D5484),
)

# The lsc record array: four populated records, records 4 and up all zero. Each
# is a 0x38 header then four 100-point float32 grids. Only two grids per record
# reach the hardware page, so the other two are stored and unread.
LSC_RECORD = 0x90D4
LSC_STRIDE = 0x678
LSC_HEADER = 0x38
LSC_GRID = 100 * 4

# (start, length, what) for every span a value is read out of.
FIELDS = (
    (0x000024, 4, 'blc gate'),
    (0x000034, 8 * 5, 'blc gain ladder, 5 float pairs'),
    (0x0000B4, 0x20 * 5, 'blc entries, 5 x 0x20'),
    (0x006CC4, 4, 'compander gate'),
    (0x006CE8, 4, 'dpc gate'),
    (0x0079D8, 0x14, 'rnr ladder header'),
    (0x0079EC, 0x80, 'rnr band abscissas'),
    (0x007A6C, 0x160 * 12, 'rnr ladder payload, 12 rows'),
    (0x00906C, 4, 'decompander gate'),
    (0x009090, 4, 'lsc gate'),
    (0x009104, 4, 'lsc shading strength'),
    (0x009108, 4, 'lsc shading enable'),
    (0x00910C, LSC_GRID, 'lsc record 0 grid 0, the third field of each point'),
    (0x00929C, LSC_GRID, 'lsc record 0 grid 1, the duplicated field'),
    (0x017A88, 4, 'drc gate'),
    # Six profiles carry sensor-specific data; the first four are populated and
    # profiles 4 and 5 hold under a dozen non-zero bytes each.
    (0x017B1C, 0xC8C * 6, 'drc profiles, 6 x 0xc8c'),
    (0x0243E4, 4, 'gib gate'),
    (0x024548, 0x10, 'cfa ladder header'),
    (0x024558, 0x80, 'cfa band abscissas'),
    (0x0245D8, 0xA4 * 5, 'cfa ladder payload, 5 rows'),
    (0x0253FC, 4, 'ccm1 gate'),
    (0x025438, 0x50 * 8, 'ccm illuminant ladder, 8 entries'),
    (0x02595C, 4, 'ccm2 gate'),
    (0x026AFC, 4, 'gamma gate'),
    (0x026B90, 0x4000 * 5, 'gamma curves, 5 x 4096 u32'),
    (0x07ABD8, 4, 'gtm2 gate'),
    (0x07B634, 4, 'lut3d gate'),
    (0x089CFC, 4, 'cm gate'),
    (0x089D70, 5 * 7 * 4, 'wb, 5 AEC rows by 7 CT columns'),
    (0x089E88, 0x10, 'lnr ladder header'),
    (0x089E98, 0x80, 'lnr band abscissas'),
    (0x089F18, 0x428 * 11, 'lnr ladder payload, 11 rows'),
    (0x08E198, 4, 'cnf gate'),
    (0x08E1DC, 0x80, 'cnf band abscissas'),
    (0x08E25C, 0x80C * 11, 'cnf ladder payload, 11 rows'),
    (0x09631C, 0x10, 'de3d ladder header'),
    (0x09632C, 0x80, 'de3d band abscissas'),
    (0x0963AC, 0x2F8 * 12, 'de3d ladder payload, 12 rows'),
    (0x0A1304, 4, 'cm2 gate'),
    (0x0A1308, 4, 'cm2 interpolate gate'),
    (0x0A1378, 0x20 * 8, 'cm2 clamp bounds, 8 records'),
    (0x0A16C0, 4, 'hdr_lsc gate'),
    (0x0B00B8, 4, 'acm gate'),
    (0x0B288C, 4, 'lms gate'),
    (0x0B2BF4, 4, 'ir_rnr gate'),
    (0x0B3550, 4, 'birnr gate'),
    (0x0B35E4, 0x100, 'birnr ladder row 1, all zero'),
    (0x0B4000, 4, 'digigain1 gate'),
    (0x0B4EE4, 4, 'ir_lms_horz gate'),
    (0x0B500C, 4, 'raw_3dnr gate'),
    (0x0B6348, 4, 'binning_filter gate'),
    (0x0B64F8, 4, 'derolling_stats gate'),
    (0x0BBE98, 4, 'awbs gate, the same flag awb reads'),
    (0x0D5484, 4, 'af_stats gate'),
    (0x0D5BD0, 1348 * 3, 'af mode ladder, 3 rows'),
)


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
