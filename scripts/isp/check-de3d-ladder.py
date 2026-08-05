#!/usr/bin/env python3
"""
Prove the de3d ladder transform in ar-isp-de3d.h against both measured points.

The de3d driver in the vendor service (0x1c6c10, packer 0x1c61f8) recomputes
the bank 0x2e00 registers from a ladder in the tuning file whenever the 3A
loop moves the gain: a header (enable, interpolate, count, selector), twelve
[low, high] float32 bands, and a 0x2f8-byte payload record per band. Inside a
band the record is used verbatim; between bands the word fields and the
byte-per-word curve are blended linearly and truncated toward zero, the four
flag words at 0x2e0..0x2ec come verbatim from the upper record, and
working-block word 0x90 blends the lower record's 0x8c against the upper
record's 0x90. The two knee registers divide 65532 by a blended field
difference, truncating toward zero.

Fields 0x04/0x08 and 0x7c..0x88 come verbatim from the upper record (hdf-071);
the control word 0x2e00 bits [3:2], the strength words 0x2e1c/0x2e20 (identity
at the vendor's default strength 50), 0x2e90's flag bits and 0x2eb4's low byte
are packer outputs recovered in hdf-073. 0x2e98 is the block's hardware
line-time latch and is deliberately absent.

This script mirrors the kernel header's integer arithmetic exactly (Q16 band
edges, Q24 blend fraction, one truncation on the blended sum, truncating
division) and refuses to pass unless:

  the header reads (enable 1, interpolate 1, count 12, selector 0) and the
  band edges are strictly increasing;

  the transform reproduces every capture-covered register of the vendor's
  live bank at both measured abscissas, masked to the ladder-owned bits
  (breath-light1 at 15.3828125, breath-light2-bright at 5.59375; the capture
  window ends at bank +0xdc, so the last 23 curve registers validate by the
  same per-byte arithmetic as the covered nine rather than directly).

The tuning file is proprietary and is not in the repository.

    kernel/scripts/isp/check-de3d-ladder.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin
"""

import argparse
import struct
import sys

HEADER = 0x9631C
BANDS = 0x9632C
PAYLOAD = 0x963AC
STRIDE = 0x2F8
COUNT = 12

# ar-isp-de3d.h calls this AR_ISP_DE3D_SLOPE_NUM.
SLOPE_NUM = 65532

REGS = [
    (0x2E00, 0x0000000C), (0x2E10, 0xFFFFFFFF), (0x2E14, 0xFFFFFFFF),
    (0x2E18, 0xFFFFFFFF), (0x2E1C, 0x00003FFF), (0x2E20, 0x01FF00FF),
    (0x2E28, 0x000001FF), (0x2E2C, 0xFFFFFFFF), (0x2E4C, 0x00003FFF),
    (0x2E90, 0x3C010077), (0x2E94, 0x00FFFFFF), (0x2E9C, 0x00FFFFFF),
    (0x2EA0, 0xFFFFFFFF), (0x2EA4, 0x03FF1FFF), (0x2EA8, 0x03FF0FFF),
    (0x2EAC, 0x000001FF), (0x2EB0, 0xFFFFFFFF), (0x2EB4, 0x000000FF),
] + [(0x2EBC + i * 4, 0xFFFFFFFF) for i in range(32)]

MEASURED = {
    366592: {
        0x2E00: 0x0000000C, 0x2E10: 0x17450A15, 0x2E14: 0x1E3C96B4,
        0x2E18: 0x17450A15, 0x2E1C: 0x00000008, 0x2E20: 0x000900F5,
        0x2E28: 0x0000000A, 0x2E2C: 0x00000100, 0x2E4C: 0x00003FFF,
        0x2E90: 0x0C010000, 0x2E94: 0x00273940, 0x2E9C: 0x00101033,
        0x2EA0: 0x150A0F0A, 0x2EA4: 0x00240045, 0x2EA8: 0x00240045,
        0x2EAC: 0x0000000A, 0x2EB0: 0x00011000, 0x2EB4: 0x000000BC,
        0x2EBC: 0x00000102, 0x2EC0: 0x03040505, 0x2EC4: 0x06070809,
        0x2EC8: 0x0A0B0C0D, 0x2ECC: 0x0E0F1010, 0x2ED0: 0x10101010,
        0x2ED4: 0x10111111, 0x2ED8: 0x11111112, 0x2EDC: 0x12131313,
    },
    1008128: {
        0x2E00: 0x0000000C, 0x2E10: 0x11100E1D, 0x2E14: 0x1E3C5EB4,
        0x2E18: 0x11100E1D, 0x2E1C: 0x0000000B, 0x2E20: 0x000B00F5,
        0x2E28: 0x0000000A, 0x2E2C: 0x00000100, 0x2E4C: 0x00003FFF,
        0x2E90: 0x0C010000, 0x2E94: 0x00273940, 0x2E9C: 0x00131333,
        0x2EA0: 0x1D131C13, 0x2EA4: 0x00210055, 0x2EA8: 0x00210055,
        0x2EAC: 0x0000000A, 0x2EB0: 0x00061B00, 0x2EB4: 0x00000079,
        0x2EBC: 0x00010203, 0x2EC0: 0x04050607, 0x2EC4: 0x09090B0B,
        0x2EC8: 0x0B0B0C0D, 0x2ECC: 0x0E0F1010, 0x2ED0: 0x10101010,
        0x2ED4: 0x10111111, 0x2ED8: 0x12121213, 0x2EDC: 0x13141414,
    },
}


def f32_q16(bits: int) -> int:
    """Mirror ar_isp_f32_q16: unsigned Q16 truncated toward zero."""
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


def blend_s32(a: int, b: int, t_q24: int) -> int:
    """Mirror ar_isp_ladder_blend_s32: one Q24 sum, truncation toward zero."""
    acc = a * ((1 << 24) - t_q24) + b * t_q24
    return acc // (1 << 24) if acc >= 0 else -((-acc) // (1 << 24))


def word(blob: bytes, band: int, off: int) -> int:
    return struct.unpack_from("<i", blob, PAYLOAD + band * STRIDE + off)[0]


def select(blob: bytes, gain_q16: int) -> tuple[int, int]:
    """Mirror ar_isp_ladder_select on ar_isp_de3d_ladder."""
    count = struct.unpack_from("<I", blob, HEADER + 0x8)[0]
    interp = struct.unpack_from("<I", blob, HEADER + 0x4)[0]
    count = COUNT if not 1 <= count <= COUNT else count

    band = count - 1
    for i in range(count - 1):
        hi = f32_q16(struct.unpack_from("<I", blob, BANDS + i * 8 + 4)[0])
        if gain_q16 <= hi:
            band = i
            break

    t_q24 = 0
    if interp and band > 0:
        lo = f32_q16(struct.unpack_from("<I", blob, BANDS + band * 8)[0])
        prev_hi = f32_q16(struct.unpack_from("<I", blob,
                                             BANDS + band * 8 - 4)[0])
        if gain_q16 < lo and lo > prev_hi:
            t_q24 = ((gain_q16 - prev_hi) << 24) // (lo - prev_hi)

    return band, t_q24


def de3d_from_blob(blob: bytes, gain_q16: int) -> list[int]:
    """Mirror ar_isp_de3d_from_blob register for register."""
    band, t_q24 = select(blob, gain_q16)

    def fld(off: int, width: int, t: int | None = None) -> int:
        t = t_q24 if t is None else t
        value = word(blob, band, off)
        if t:
            value = blend_s32(word(blob, band - 1, off), value, t)

        return value & ((1 << width) - 1)

    def slope(hi_off: int, lo_off: int) -> int:
        def blended(off: int) -> int:
            value = word(blob, band, off)
            if t_q24:
                value = blend_s32(word(blob, band - 1, off), value, t_q24)

            return value

        span = blended(hi_off) - blended(lo_off)
        if not span:
            return 0

        # Truncating toward zero, as the vendor's sdiv does.
        quotient = SLOPE_NUM // span if span > 0 else -(SLOPE_NUM // -span)
        return quotient & 0xFFFF

    asym = word(blob, band, 0x90)
    if t_q24:
        asym = blend_s32(word(blob, band - 1, 0x8C), asym, t_q24)

    asym &= 0xFF

    out = [
        (fld(0x04, 1, 0) << 3) | (fld(0x08, 1, 0) << 2),
        (slope(0x18, 0x14) << 16) | (fld(0x14, 8) << 8) | fld(0x18, 8),
        (fld(0x1C, 8) << 24) | (fld(0x20, 8) << 16) | (fld(0x24, 8) << 8) |
        fld(0x28, 8),
        (slope(0x30, 0x2C) << 16) | (fld(0x2C, 8) << 8) | fld(0x30, 8),
        fld(0x34, 14),
        (fld(0x38, 9) << 16) | fld(0x40, 8),
        fld(0x44, 9),
        (fld(0x48, 8) << 24) | (fld(0x4C, 8) << 16) | (fld(0x50, 8) << 8) |
        fld(0x54, 8),
        fld(0x5C, 14),
        (fld(0xC8, 1) << 26) | (fld(0xCC, 1) << 27) | (fld(0xD0, 1) << 28) |
        (fld(0xD4, 1) << 29) | (fld(0xD8, 3) << 4) | fld(0xDC, 3) |
        (1 << 16),
        (fld(0x6C, 8) << 16) | (fld(0x70, 8) << 8) | fld(0x74, 8),
        fld(0x7C, 1, 0) | (fld(0x80, 1, 0) << 1) | (fld(0x84, 1, 0) << 2) |
        (fld(0x88, 1, 0) << 3) | (fld(0x2E0, 1, 0) << 4) |
        (fld(0x2E4, 1, 0) << 5) | (fld(0x2E8, 1, 0) << 6) |
        (fld(0x2EC, 1, 0) << 7) | (fld(0x2F0, 8) << 8) |
        (fld(0x2F4, 8) << 16),
        fld(0x8C, 8) | (asym << 8) | (fld(0x94, 8) << 16) |
        (fld(0x98, 8) << 24),
        fld(0x9C, 13) | (fld(0xA0, 10) << 16),
        fld(0xA4, 12) | (fld(0xA8, 10) << 16),
        fld(0xAC, 9),
        (fld(0xB0, 8) << 24) | (fld(0xB4, 8) << 16) | (fld(0xB8, 8) << 8) |
        fld(0xBC, 8),
        fld(0xC0, 8),
    ]

    for i in range(32):
        base = 0xE0 + i * 16
        out.append((fld(base, 8) << 24) | (fld(base + 4, 8) << 16) |
                   (fld(base + 8, 8) << 8) | fld(base + 12, 8))

    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tuning", required=True, help="nt99235 tuning blob")
    args = ap.parse_args()

    with open(args.tuning, "rb") as handle:
        blob = handle.read()

    header = struct.unpack_from("<4I", blob, HEADER)
    if header != (1, 1, COUNT, 0):
        sys.exit(f"de3d header reads {header}, expected (1, 1, {COUNT}, 0)")

    edges = struct.unpack_from(f"<{COUNT * 2}f", blob, BANDS)
    if list(edges) != sorted(edges) or len(set(edges)) != COUNT * 2:
        sys.exit("band edges are not strictly increasing")

    for gain_q16, want in MEASURED.items():
        band, t_q24 = select(blob, gain_q16)
        got = de3d_from_blob(blob, gain_q16)
        bad = 0
        for (reg, mask), value in zip(REGS, got, strict=True):
            if reg not in want:
                continue

            if (value & mask) != want[reg]:
                bad += 1
                print(f"  {reg:#06x}: want {want[reg]:#010x} "
                      f"got {value & mask:#010x}")
        if bad:
            sys.exit(f"abscissa {gain_q16 / 65536} misses {bad} registers")

        print(f"abscissa {gain_q16 / 65536:9.6f} -> bands {band - 1},{band} "
              f"t {t_q24 / (1 << 24):.7f}, all {len(want)} covered "
              f"registers exact")

    print("\nde3d ladder agrees with ar-isp-de3d.h and the two measured "
          "points")
    return 0


if __name__ == "__main__":
    sys.exit(main())
