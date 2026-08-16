#!/usr/bin/env python3
"""
Prove cm's gain field comes from the tuning blob's AE-indexed table.

cm packs a saturation gain into the low seven bits of ISP 0x483c:

    field = floor(32 * gain)

The gain comes from a 5 AEC row by 7 colour-temperature column table in the
NT99235 tuning blob at 0x89d70. Each record is two float32 values: gain and
angle. The current driver installs 0x483c = 33, which matches row 1 of that
table. Two columns in that row hold gain 1.05, so the live colour-temperature
column is narrowed to {0, 3} by this check and still needs a later capture to
pin uniquely.

Needs the tuning blob, which is a capture artifact and not in the tree:

    kernel/scripts/isp/check-cm-ladder.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin
"""

import argparse
import importlib.util
import math
import pathlib
import struct
import sys
from types import ModuleType

HERE = pathlib.Path(__file__).resolve().parent

BANK = 0x4834
GAIN_FIELD = 0x08
UNITY_FIELD = 0x0C
ANGLE_FIELD = 0x10
COS_FIELD = 0x14
COS_FIELD_2 = 0x18

GAIN_MASK = 0x7F
HEADER = 0x89CFC
HDR_INTERP = 0x04
HDR_AEC_COUNT = 0x08
HDR_CT_COUNT = 0x0C
AEC_AXIS = 0x89D10
TABLE = 0x89D70
AEC_ROWS = 5
CT_COLUMNS = 7
RECORD_STRIDE = 8
ROW_STRIDE = CT_COLUMNS * RECORD_STRIDE
Q24_ONE = 1 << 24


def load_audit() -> ModuleType:
    """The driver's register tables, via audit-provenance.py."""
    path = HERE / 'audit-provenance.py'
    spec = importlib.util.spec_from_file_location('ar_isp_audit', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    source = path.read_text().replace(
        "if __name__ == '__main__':\n    sys.exit(main())", '')
    exec(compile(source, str(path), 'exec'), mod.__dict__)

    return mod


def f32_q16(bits: int) -> int:
    """Mirror ar_isp_f32_q16."""
    mant = (bits & 0x7FFFFF) | 0x800000
    exp = ((bits >> 23) & 0xFF) - 127
    shift = 7 - exp

    if bits & 0x80000000:
        return 0

    if shift >= 32:
        return 0

    if shift < -8:
        return 0xFFFFFFFF

    if shift <= 0:
        return mant << -shift

    return mant >> shift


def f32_q8(bits: int) -> int:
    """Mirror ar_isp_cm_f32_q8."""
    mant = (bits & 0x7FFFFF) | 0x800000
    exp = ((bits >> 23) & 0xFF) - 127
    shift = 15 - exp

    if bits & 0x80000000:
        return 0

    if shift >= 32:
        return 0

    if shift < -16:
        return 0xFFFFFFFF

    if shift <= 0:
        return mant << -shift

    return mant >> shift


def blend(a: int, b: int, t_q24: int) -> int:
    """Mirror ar_isp_ladder_blend."""
    return (a * (Q24_ONE - t_q24) + b * t_q24) >> 24


def select_row(blob: bytes, gain_q8: int) -> tuple[int, int]:
    """Mirror ar_isp_cm_select."""
    count = struct.unpack_from('<I', blob, HEADER + HDR_AEC_COUNT)[0]
    interp = struct.unpack_from('<I', blob, HEADER + HDR_INTERP)[0]
    if count < 1 or count > AEC_ROWS:
        count = AEC_ROWS

    row = count - 1
    for i in range(count - 1):
        hi = f32_q8(struct.unpack_from('<I', blob, AEC_AXIS + i * 8 + 4)[0])
        if gain_q8 <= hi:
            row = i
            break

    t_q24 = 0
    if interp and row > 0:
        lo = f32_q8(struct.unpack_from('<I', blob, AEC_AXIS + row * 8)[0])
        prev_hi = f32_q8(struct.unpack_from('<I', blob,
                                            AEC_AXIS + row * 8 - 4)[0])
        if gain_q8 < lo and lo > prev_hi:
            t_q24 = ((gain_q8 - prev_hi) << 24) // (lo - prev_hi)

    return row, t_q24


def cm_table(blob: bytes) -> dict[tuple[int, int], tuple[float, float]]:
    """The live 5x7 table as {(aec, ct): (gain, angle)}."""
    rows: dict[tuple[int, int], tuple[float, float]] = {}
    for aec in range(AEC_ROWS):
        for ct in range(CT_COLUMNS):
            at = TABLE + ROW_STRIDE * aec + RECORD_STRIDE * ct
            rows[(aec, ct)] = struct.unpack_from('<ff', blob, at)
    return rows


def gain_field_from_blob(blob: bytes, gain_q8: int, ct: int = 0) -> int:
    """Mirror ar_isp_cm_gain_field_from_blob."""
    ct_count = struct.unpack_from('<I', blob, HEADER + HDR_CT_COUNT)[0]
    if ct_count < 1 or ct_count > CT_COLUMNS:
        ct_count = CT_COLUMNS

    if ct >= ct_count:
        ct = 0

    row, t_q24 = select_row(blob, gain_q8)
    at = TABLE + ROW_STRIDE * row + RECORD_STRIDE * ct
    gain_q16 = f32_q16(struct.unpack_from('<I', blob, at)[0])
    if t_q24:
        prev = TABLE + ROW_STRIDE * (row - 1) + RECORD_STRIDE * ct
        gain_q16 = blend(f32_q16(struct.unpack_from('<I', blob, prev)[0]),
                         gain_q16, t_q24)

    return (gain_q16 >> 11) & GAIN_MASK


def encode_gain(gain: float) -> int:
    """The vendor's low-seven-bit saturation field."""
    return math.floor(gain * 32.0) & GAIN_MASK


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

    wanted = (GAIN_FIELD, UNITY_FIELD, ANGLE_FIELD, COS_FIELD, COS_FIELD_2)
    missing = [f'{BANK + off:#06x}' for off in wanted if BANK + off not in final]
    if missing:
        sys.exit(f'the driver writes no value at {", ".join(missing)}, so '
                 f'there is nothing to prove the packer against')

    installed = {off: final[BANK + off] for off in wanted}
    installed_gain = installed[GAIN_FIELD] & GAIN_MASK
    rows = cm_table(blob)
    candidates = [
        (aec, ct, gain, angle)
        for (aec, ct), (gain, angle) in sorted(rows.items())
        if encode_gain(gain) == installed_gain
    ]
    failures = []

    print(f'cm table at {TABLE:#x}: {AEC_ROWS} AEC rows by '
          f'{CT_COLUMNS} CT columns\n')
    print(f'{"row":>3} {"ct":>2} {"gain":>8} {"angle":>8} {"field":>5}')
    for (aec, ct), (gain, angle) in sorted(rows.items()):
        print(f'{aec:>3} {ct:>2} {gain:>8.5g} {angle:>8.5g} '
              f'{encode_gain(gain):>5}')

    print(f'\ninstalled 0x{BANK + GAIN_FIELD:04x} low field: '
          f'{installed_gain}')
    print('candidate record(s): ' + ', '.join(
        f'row {aec} ct {ct} gain {gain:.5g} angle {angle:.5g}'
        for aec, ct, gain, angle in candidates))

    if not candidates:
        failures.append(f'no cm table record encodes field {installed_gain}')

    candidate_rows = {aec for aec, _ct, _gain, _angle in candidates}
    if candidate_rows != {1}:
        failures.append(f'field {installed_gain} maps to AEC rows '
                        f'{sorted(candidate_rows)}, expected only row 1 from '
                        f'the cm2-pinned operating point')

    candidate_cols = {ct for _aec, ct, _gain, _angle in candidates}
    if candidate_cols != {0, 3}:
        failures.append(f'field {installed_gain} maps to CT columns '
                        f'{sorted(candidate_cols)}, expected the current '
                        f'known ambiguity {{0, 3}}')

    for off, expected, what in (
        (UNITY_FIELD, 0x20, 'second gain field, unity'),
        (ANGLE_FIELD, 0x00, 'angle field, zero'),
        (COS_FIELD, 0x200, 'zero-angle cosine field'),
        (COS_FIELD_2, 0x200, 'second zero-angle cosine field'),
    ):
        got = installed[off]
        print(f'  0x{BANK + off:04x} = {got:#010x}  {what}')
        if got != expected:
            failures.append(f'0x{BANK + off:04x}: expected {expected:#x}, '
                            f'driver installs {got:#x}')

    if failures:
        print()
        for line in failures:
            print(f'FAIL: {line}')

        return 1

    print('\ncm 0x483c matches row 1 of the blob table; the live CT column is '
          'still narrowed to columns 0 and 3')

    return 0


if __name__ == '__main__':
    sys.exit(main())
