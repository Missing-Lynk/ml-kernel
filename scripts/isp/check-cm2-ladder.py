#!/usr/bin/env python3
"""
Prove cm2's clamp bounds and their two reciprocals come from the tuning blob.

cm2 holds two clamp windows, each a low and a high bound, and two reciprocals
of the window widths so the block can rescale by multiplying. All six sit in
one run at bank+0x1c through bank+0x30, and all six come from one table in the
sensor tuning blob.

The packer is conver_cm2_tuning_pra_to_snr1_reg at 0x1a0578 in
libmpp_service.so. Its caller, isp_sub_process_reg_compute at 0x1a07c8, selects
a 24-byte record from a two-dimensional ladder in the blob and hands it over.
The tail of the packer is the whole claim:

    ldur q0, [x24, #-236]      the four bounds, 16 bytes
    str  q0, [x23, #1072]      stored verbatim to bank+0x1c
    sub  w0, w3, w10           hi1 - lo1
    sub  w2, w8, w9            hi2 - lo2
    mov  w1, #0x400            1024
    sdiv w0, w1, w0
    sdiv w1, w1, w2
    stp  w0, w1, [x19, #44]    bank+0x2c and bank+0x30

So the two reciprocals are not independent values at all: they are 1024 divided
by the widths of the two windows stored three words above them, with sdiv
truncating toward zero. That is checked here against the driver's own tables,
which is a relation between four measured registers and two others.

The bounds themselves are read out of the blob. The ladder is at 0xa1378: rows
of 168 bytes indexed by the AEC trigger, records of 24 bytes within a row
indexed by the colour temperature, with the live extent of each axis given by
the two counts at 0xa130c and 0xa1310. Three of the four bounds are constant
down every live row, so the installed value is that constant verbatim. The
fourth varies, and the blob's interpolation gate at 0xa1308 is set, so the
installed value is an interpolation between rows and is checked to lie in the
range the rows span.

This is why the fourth bound reads 1006 where no row holds 1006: the capture
caught one AE operating point. bank+0x24 and bank+0x30 move with the scene.

Needs the tuning blob, which is a capture artifact and not in the tree:

    kernel/scripts/isp/check-cm2-ladder.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin
"""

import argparse
import importlib.util
import pathlib
import struct
import sys

HERE = pathlib.Path(__file__).resolve().parent

BANK = 0x4800

# Bank-relative offsets of the run the packer writes.
LO1, HI1, LO2, HI2 = 0x1C, 0x20, 0x24, 0x28
RECIP1, RECIP2 = 0x2C, 0x30

# `mov w1, #0x400` at 0x1a067c, divided by each window width.
NUMERATOR = 1024

# The ladder in the tuning blob. TABLE is the base of the records; the two
# counts bound the live extent of each axis, and the gate says whether the
# packer's caller interpolates between rows or copies one verbatim.
INTERPOLATE_GATE = 0xA1308
AEC_COUNT, CT_COUNT = 0xA130C, 0xA1310
TABLE = 0xA1378
ROW_STRIDE, RECORD_STRIDE = 168, 24

# Byte offsets of the four bounds inside a 24-byte record. The two floats
# ahead of them are a gain and a rotation angle, which reach other registers.
RECORD_BOUNDS = 8


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


def ladder(blob):
    """The four bounds of every live record, by (aec, ct) index."""
    aec = struct.unpack_from('<I', blob, AEC_COUNT)[0]
    ct = struct.unpack_from('<I', blob, CT_COUNT)[0]
    rows = {}
    for i in range(aec):
        for j in range(ct):
            at = TABLE + ROW_STRIDE * i + RECORD_STRIDE * j + RECORD_BOUNDS
            rows[(i, j)] = struct.unpack_from('<4i', blob, at)

    return rows, aec, ct


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tuning', required=True,
                    help='the sensor tuning blob the vendor ships')
    args = ap.parse_args()

    path = pathlib.Path(args.tuning)
    if not path.exists():
        sys.exit(f'{path}: not found. The tuning blob is a capture artifact '
                 f'and is deliberately not in the tree.')

    blob = path.read_bytes()
    audit = load_audit()
    _library, final, _origin = audit.load_tables()

    wanted = (LO1, HI1, LO2, HI2, RECIP1, RECIP2)
    missing = [f'{BANK + off:#06x}' for off in wanted if BANK + off not in final]
    if missing:
        sys.exit(f'the driver writes no value at {", ".join(missing)}, so '
                 f'there is nothing to prove the packer against')

    installed = {off: final[BANK + off] for off in wanted}
    failures = []

    rows, aec, ct = ladder(blob)
    interpolates = struct.unpack_from('<I', blob, INTERPOLATE_GATE)[0]
    print(f'ladder at {TABLE:#x}: {aec} AEC row(s) by {ct} CT column(s), '
          f'interpolation gate at {INTERPOLATE_GATE:#x} reads {interpolates}\n')

    print(f'{"record":<10}{"lo1":>6}{"hi1":>6}{"lo2":>6}{"hi2":>6}')
    for (i, j), bounds in sorted(rows.items()):
        print(f'{f"({i},{j})":<10}' + ''.join(f'{v:>6}' for v in bounds))

    print(f'\n{"driver":<10}' + ''.join(f'{installed[off]:>6}'
                                        for off in (LO1, HI1, LO2, HI2)))

    # Three bounds are constant down the ladder, so the installed value has to
    # be that constant. The one that varies is an interpolation output, so it
    # has to fall inside the range the ladder spans.
    print()
    for slot, off, what in ((0, LO1, 'lo1'), (1, HI1, 'hi1'),
                            (2, LO2, 'lo2'), (3, HI2, 'hi2')):
        seen = {bounds[slot] for bounds in rows.values()}
        got = installed[off]
        if len(seen) == 1:
            source = f'constant {seen.pop()} down every live record'
            if got not in {v[slot] for v in rows.values()}:
                failures.append(
                    f'{BANK + off:#06x}: the driver installs {got} where the '
                    f'ladder holds one value, {source}')
        else:
            lo, hi = min(seen), max(seen)
            source = f'varies over {lo}..{hi}, interpolated'
            if not lo <= got <= hi:
                failures.append(
                    f'{BANK + off:#06x}: the driver installs {got}, outside '
                    f'the {lo}..{hi} the ladder spans, so it is not an '
                    f'interpolation of these records')
            elif not interpolates:
                failures.append(
                    f'{BANK + off:#06x}: the driver installs {got}, which no '
                    f'record holds, but the interpolation gate is clear')

        print(f'  {BANK + off:#06x} = {got:<6} {what}, blob {source}')

    # The two reciprocals against the four bounds, which is the packer's own
    # arithmetic reproduced from the driver's tables alone.
    print()
    for off, low, high, what in ((RECIP1, LO1, HI1, 'first'),
                                 (RECIP2, LO2, HI2, 'second')):
        width = installed[high] - installed[low]
        if width <= 0:
            failures.append(f'{BANK + off:#06x}: the window is {width} wide, '
                            f'so the vendor\'s sdiv would trap or go negative')
            continue

        # sdiv truncates toward zero, and both operands here are positive.
        expected = NUMERATOR // width
        got = installed[off]
        print(f'  {BANK + off:#06x}  {NUMERATOR} / ({installed[high]} - '
              f'{installed[low]}) = {NUMERATOR} / {width:<3} = {expected:<5} '
              f'driver {got:<5} {what} window')
        if got != expected:
            failures.append(f'{BANK + off:#06x}: the packer gives {expected} '
                            f'and the driver installs {got}')

    if failures:
        print()
        for line in failures:
            print(f'FAIL: {line}')

        return 1

    print(f'\ncm2 bank+{LO1:#04x}..+{HI2:#04x} are the blob ladder\'s bounds, '
          f'and bank+{RECIP1:#04x} and bank+{RECIP2:#04x} are {NUMERATOR} '
          f'divided by their widths')

    return 0


if __name__ == '__main__':
    sys.exit(main())
