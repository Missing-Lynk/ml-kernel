#!/usr/bin/env python3
"""
Prove the af_stats metering window and its mode word.

Autofocus meters a region of interest, not the whole frame, and its geometry
registers are that region expressed four ways. The region itself is a fixed
four-float constant in the vendor library at 0x36ddd0, `{0.25, 0.25, 0.5,
0.5}`: the first pair places the window and the second sizes it, both as
fractions of half the frame. Only the `0x2405` command changes it.

`isp_af_stats_set_format` at 0x1f4f38 builds all three geometry registers from
that constant and the frame, at 0x1f5024:

	lsr    w0, w22, #1        half the height
	lsr    w1, w23, #1        half the width
	scvtf / fmul / fcvtzs     scale each by its fraction, truncating

giving an offset of `(width/2 * roi[0], height/2 * roi[1])` and a region of
`(width/2 * roi[2], height/2 * roi[3])`. The region is then divided down by
four different constants, each a multiply-and-shift rather than a division:

	x_skip       roi_width  / 16     asr #4 at 0x1f50c4
	y_skip       roi_height /  9     0x38e38e39, asr 33
	block_width  roi_width  / 17     0x78787879, asr 35
	block_height roi_height / 10     0x66666667, asr 34

Each register packs its pair into two fields, and the masks the code applies
are what fixes the field widths: 13 bits for the offsets, 9 and 10 bits for
the rest.

The mode word at bank+0x08 is different work: six bitfields packed from a
tuning-blob ladder by `isp_sub_process_reg_compute` at 0x1f6178, the af
analogue of the cm2 packer, indexed by the AEC trigger. The word alignment is
not assumed: the same row's earlier words rebuild bank+0x04, which the vendor
image independently carries.

Needs the vendor library and the tuning blob, neither of which is in the tree:

    kernel/scripts/isp/check-af-stats.py \\
        --library out/air-gather/vendor-root/usr/lib/libmpp_service.so \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin
"""

import argparse
import importlib.util
import pathlib
import struct
import sys

HERE = pathlib.Path(__file__).resolve().parent

BANK = 0x7400
FRAME_W, FRAME_H = 1920, 1080

# The region-of-interest constant, as four floats: x and y placement, then
# width and height, each a fraction of half the frame.
ROI_AT = 0x36DDD0

# Divisors the vendor applies to the region, as multiply-and-shift.
X_SKIP_DIV, Y_SKIP_DIV = 16, 9
BLOCK_W_DIV, BLOCK_H_DIV = 17, 10

# Field widths, from the masks each store applies.
OFFSET_BITS = 13
LOW_BITS, HIGH_BITS = 9, 10

# The mode-word ladder in the tuning blob: an enable flag, then rows indexed by
# the AEC trigger. Word 0 of a row is unused by these fields.
ENABLE_FLAG = 0xD5484
LADDER = 0xD5BD0
ROW_STRIDE = 1348
ROWS = 3

# Bit position to the row word that supplies it, for bank+0x08.
MODE_FIELDS = ((8, 5, 2), (5, 6, 3), (4, 7, 1), (2, 8, 2), (1, 9, 1), (0, 10, 1))

# bank+0x04 is built from the same row's words 1 to 4, one byte each, and the
# vendor image carries it. That is what fixes the word alignment.
BYTE_WORD_REG = 0x04
BYTE_WORDS = (1, 2, 3, 4)

# Constants the per-frame re-arm stores, with the site that builds each.
CONSTANTS = {0x170: (0x40, '0x1f55bc, one phase of an A/B toggle on '
                           'priv+824; the other phase stores zero here')}


def load_audit():
    """The driver's register tables, via audit-provenance.py."""
    path = HERE / 'audit-provenance.py'
    spec = importlib.util.spec_from_file_location('ar_isp_audit', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    source = path.read_text().replace(
        "if __name__ == '__main__':\n    sys.exit(main())", '')
    exec(compile(source, str(path), 'exec'), mod.__dict__)

    return mod


def geometry(roi, frame_w, frame_h):
    """The four derived quantities, with the vendor's truncation."""
    half_w, half_h = frame_w >> 1, frame_h >> 1

    # fcvtzs truncates toward zero, and every operand here is positive.
    return {
        'x_offset': int(half_w * roi[0]),
        'y_offset': int(half_h * roi[1]),
        'roi_width': int(half_w * roi[2]),
        'roi_height': int(half_h * roi[3]),
    }


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--library', required=True,
                    help='the vendor libmpp_service.so')
    ap.add_argument('--tuning', required=True,
                    help='the sensor tuning blob the vendor ships')
    args = ap.parse_args()

    for path in (args.library, args.tuning):
        if not pathlib.Path(path).exists():
            sys.exit(f'{path}: not found. Both are capture artifacts and are '
                     f'deliberately not in the tree.')

    library = pathlib.Path(args.library).read_bytes()
    blob = pathlib.Path(args.tuning).read_bytes()
    audit = load_audit()
    image, final, _origin = audit.load_tables()

    wanted = (0x08, 0x0C, 0x10, 0x14, *CONSTANTS)
    missing = [f'{BANK + off:#06x}' for off in wanted if BANK + off not in final]
    if missing:
        sys.exit(f'the driver writes no value at {", ".join(missing)}, so '
                 f'there is nothing to prove the packer against')

    failures = []

    roi = struct.unpack_from('<4f', library, ROI_AT)
    print(f'region of interest at {ROI_AT:#x}: {roi}, as fractions of half the '
          f'frame\n')

    g = geometry(roi, FRAME_W, FRAME_H)
    print(f'frame {FRAME_W} x {FRAME_H} gives offset '
          f'({g["x_offset"]}, {g["y_offset"]}) and region '
          f'{g["roi_width"]} x {g["roi_height"]}\n')

    packed = {
        0x0C: ((g['y_offset'], g['x_offset']), OFFSET_BITS,
               'the metering offset'),
        0x10: ((g['roi_height'] // Y_SKIP_DIV, g['roi_width'] // X_SKIP_DIV),
               None, f'the skip, region over {Y_SKIP_DIV} and {X_SKIP_DIV}'),
        0x14: ((g['roi_height'] // BLOCK_H_DIV, g['roi_width'] // BLOCK_W_DIV),
               None, f'the block size, region over {BLOCK_H_DIV} and '
                     f'{BLOCK_W_DIV}'),
    }

    for off, ((high, low), bits, what) in packed.items():
        expected = (high << 16) | low
        got = final[BANK + off]
        print(f'  {BANK + off:#06x}  ({high} << 16) | {low} = {expected:#010x}  '
              f'driver {got:#010x}  {what}')
        # The offsets share one width; the other two are 9 bits low and 10
        # high, which the masks 0xfffffe00 and 0xfc00ffff at each store fix.
        widths = ((high, HIGH_BITS, 'high'), (low, LOW_BITS, 'low')) \
            if bits is None else ((high, bits, 'high'), (low, bits, 'low'))
        for value, width, edge in widths:
            if value >> width:
                failures.append(
                    f'{BANK + off:#06x}: the {edge} field is {value}, which '
                    f'does not fit the {width} bits its store masks for')

        if got != expected:
            failures.append(f'{BANK + off:#06x}: the packer gives '
                            f'{expected:#010x} and the driver installs '
                            f'{got:#010x}')

    # The mode word, from the tuning ladder.
    enable = struct.unpack_from('<I', blob, ENABLE_FLAG)[0]
    rows = [struct.unpack_from('<11I', blob, LADDER + ROW_STRIDE * i)
            for i in range(ROWS)]
    live = [i for i, row in enumerate(rows) if any(row)]
    print(f'\nmode-word ladder at {LADDER:#x}, stride {ROW_STRIDE}: '
          f'{len(live)} of {ROWS} row(s) carry data, enable flag at '
          f'{ENABLE_FLAG:#x} reads {enable}')
    if not enable:
        failures.append(f'the af enable flag at blob+{ENABLE_FLAG:#x} reads 0, '
                        f'so this ladder should not be reaching the bank')

    if not live:
        sys.exit('every ladder row is zero, so there is nothing to rebuild the '
                 'mode word from')

    row = rows[live[0]]

    # The alignment check: the same row's earlier words rebuild a register the
    # vendor image independently carries. Without this the field mapping below
    # could be fitted to any row.
    rebuilt = 0
    for word in BYTE_WORDS:
        rebuilt = (rebuilt << 8) | (row[word] & 0xFF)

    carried = image.get(BANK + BYTE_WORD_REG)
    print(f'  words {BYTE_WORDS} rebuild {BANK + BYTE_WORD_REG:#06x} = '
          f'{rebuilt:#010x}, vendor image carries {carried:#010x}'
          if carried is not None else
          f'  words {BYTE_WORDS} rebuild {rebuilt:#010x}, but no vendor image '
          f'covers {BANK + BYTE_WORD_REG:#06x}')
    if carried is None or rebuilt != carried:
        failures.append(
            f'{BANK + BYTE_WORD_REG:#06x}: row {live[0]} rebuilds '
            f'{rebuilt:#010x} where the vendor image carries {carried}, so the '
            f'word alignment into the ladder row is not established')

    value = 0
    for shift, word, bits in MODE_FIELDS:
        field = row[word] & ((1 << bits) - 1)
        if row[word] != field:
            failures.append(
                f'ladder word {word} is {row[word]}, wider than the {bits} bit '
                f'field at bit {shift} that consumes it')

        value |= field << shift

    got = final[BANK + 0x08]
    print(f'  {BANK + 0x08:#06x}  six fields from row {live[0]} = '
          f'{value:#010x}  driver {got:#010x}  the mode word')
    if got != value:
        failures.append(f'{BANK + 0x08:#06x}: the ladder gives {value:#010x} '
                        f'and the driver installs {got:#010x}')

    print()
    for off, (expected, where) in CONSTANTS.items():
        got = final[BANK + off]
        print(f'  {BANK + off:#06x} = {got:#010x}  constant, {where}')
        if got != expected:
            failures.append(f'{BANK + off:#06x}: the re-arm stores '
                            f'{expected:#010x} and the driver installs '
                            f'{got:#010x}')

    if failures:
        print()
        for line in failures:
            print(f'FAIL: {line}')

        return 1

    print('\nthe metering window follows from the frame and the region '
          'constant, and the mode word from the ladder row that also rebuilds '
          f'{BANK + BYTE_WORD_REG:#06x}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
