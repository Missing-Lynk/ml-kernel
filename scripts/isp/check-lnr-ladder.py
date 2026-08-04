#!/usr/bin/env python3
"""
Check the lnr ladder transform against the two measured vendor register images.

This mirrors ar_isp_lnr_from_blob in ar-isp-ladder.h: Q16 band edges, a Q24
blend fraction, signed payload words, truncation toward zero, and register
packing with preserved bits seeded from the captured register image. Register
0x3d10 is never written by the vendor packer, and 0x3d14 is intentionally left
on the replay path because it has an unresolved fixed bias before pack.

The tuning file and captures are proprietary and are not in the repository.

    kernel/scripts/isp/check-lnr-ladder.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin \\
        --bright out/au-vendor-session/breath-light2-bright.txt \\
        --dark out/au-vendor-session/breath-light1.txt
"""

import argparse
import re
import struct
import sys

HEADER = 0x89E88
BANDS = 0x89E98
PAYLOAD = 0x89F18
STRIDE = 0x428
COUNT = 11
BANK = 0x3CC8
REGS = 86
SKIP = {0x3D10, 0x3D14}

POINTS = (
    ("bright", 5.59375),
    ("dark", 15.38281),
)


def f32_q16(bits):
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


def read_s32(blob, off):
    return struct.unpack_from("<i", blob, off)[0]


def read_u32(blob, off):
    return struct.unpack_from("<I", blob, off)[0]


def blend(a, b, t_q24):
    return (a * ((1 << 24) - t_q24) + b * t_q24) // (1 << 24)


def trunc_blend(a, b, t_q24):
    num = a * ((1 << 24) - t_q24) + b * t_q24
    if num < 0:
        return -((-num) // (1 << 24))
    return num // (1 << 24)


def select(blob, gain_q16):
    count = read_u32(blob, HEADER + 0x8)
    interp = read_u32(blob, HEADER + 0x4)
    count = COUNT if not 1 <= count <= COUNT else count

    edges = [f32_q16(read_u32(blob, BANDS + i * 4)) for i in range(count * 2)]
    band = count - 1
    for i in range(count - 1):
        if gain_q16 <= edges[i * 2 + 1]:
            band = i
            break

    t_q24 = 0
    if interp and band > 0:
        lo = edges[band * 2]
        prev_hi = edges[band * 2 - 1]
        if gain_q16 < lo and lo > prev_hi:
            t_q24 = ((gain_q16 - prev_hi) << 24) // (lo - prev_hi)

    return band, t_q24


def word(blob, band, off, t_q24):
    v = read_s32(blob, PAYLOAD + band * STRIDE + off)
    if t_q24:
        v = trunc_blend(read_s32(blob, PAYLOAD + (band - 1) * STRIDE + off),
                        v, t_q24)
    return v & 0xFFFFFFFF


def pack_field(regs, blob, band, t_q24, reg, off, shift, width):
    mask = 0xFFFFFFFF if width == 32 else (1 << width) - 1
    regs[reg] &= ~(mask << shift) & 0xFFFFFFFF
    regs[reg] |= (word(blob, band, off, t_q24) & mask) << shift
    regs[reg] &= 0xFFFFFFFF


def lnr_from_blob(blob, seed, gain_q16):
    regs = seed[:]
    band, t_q24 = select(blob, gain_q16)

    for off, shift, width in ((0x000, 2, 1), (0x004, 6, 1), (0x008, 8, 1),
                              (0x00C, 9, 1), (0x010, 12, 1), (0x014, 24, 8)):
        pack_field(regs, blob, band, t_q24, 0, off, shift, width)

    pairs = (
        (1, 0x018, 0x028), (2, 0x038, 0x048), (3, 0x01C, 0x02C),
        (4, 0x03C, 0x04C), (5, 0x020, 0x030), (6, 0x040, 0x050),
        (7, 0x024, 0x034), (8, 0x044, 0x054), (9, 0x058, 0x068),
        (10, 0x078, 0x05C), (11, 0x06C, 0x07C), (12, 0x060, 0x070),
        (13, 0x080, 0x064), (14, 0x074, 0x084),
    )
    for reg, low, high in pairs:
        pack_field(regs, blob, band, t_q24, reg, low, 0, 13)
        pack_field(regs, blob, band, t_q24, reg, high, 16, 13)

    for reg, low in ((15, 0x210), (16, 0x218), (17, 0x220)):
        pack_field(regs, blob, band, t_q24, reg, low, 0, 8)
        pack_field(regs, blob, band, t_q24, reg, low + 4, 8, 8)

    specs = (
        (20, 0x090, 0x094, 16), (21, 0x098, 0x09C, 10),
        (22, 0x0A0, 0x0A4, 10), (23, 0x0A8, 0x0AC, 10),
        (24, 0x0B0, 0x0B4, 10), (29, 0x0D0, 0x0D4, 10),
        (30, 0x0D8, 0x0DC, 10), (31, 0x0E0, 0x0E4, 10),
        (32, 0x0E8, 0x0EC, 10), (37, 0x108, 0x10C, 16),
    )
    for reg, low, high, high_shift in specs:
        pack_field(regs, blob, band, t_q24, reg, low, 0, 10)
        pack_field(regs, blob, band, t_q24, reg, high, high_shift, 10)

    for reg, off in ((25, 0x0B8), (33, 0x0F0)):
        pack_field(regs, blob, band, t_q24, reg, off, 0, 10)

    for reg, low in ((26, 0x0BC), (27, 0x0C4), (34, 0x0F4), (35, 0x0FC)):
        pack_field(regs, blob, band, t_q24, reg, low, 0, 16)
        pack_field(regs, blob, band, t_q24, reg, low + 4, 16, 16)

    for reg, off in ((28, 0x0CC), (36, 0x104)):
        pack_field(regs, blob, band, t_q24, reg, off, 0, 16)

    for i in range(48):
        reg = 38 + i
        off = 0x110 + i * 16 if i < 16 else 0x228 + (i - 16) * 16
        for lane in range(4):
            pack_field(regs, blob, band, t_q24, reg, off + lane * 4,
                       lane * 8, 8)

    return regs, band, t_q24


def read_capture(path):
    regs = []
    in_lnr = False
    line_re = re.compile(r"^\+0x[0-9a-f]+:\s+(.+)$")

    with open(path) as handle:
        for line in handle:
            if line.startswith("--- "):
                in_lnr = line.startswith("--- isp lnr 0x3cc8")
                continue
            if not in_lnr:
                continue
            match = line_re.match(line.strip())
            if not match:
                continue
            regs.extend(int(v, 16) for v in match.group(1).split())

    if len(regs) < REGS:
        sys.exit(f"{path}: only {len(regs)} lnr words")
    return regs[:REGS]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tuning", required=True, help="nt99235 tuning blob")
    ap.add_argument("--bright", required=True, help="bright vendor capture")
    ap.add_argument("--dark", required=True, help="dark vendor capture")
    args = ap.parse_args()

    with open(args.tuning, "rb") as handle:
        blob = handle.read()

    header = struct.unpack_from("<4I", blob, HEADER)
    if header != (1, 1, COUNT, 0):
        sys.exit(f"lnr header reads {header}, expected (1, 1, {COUNT}, 0)")

    paths = {"bright": args.bright, "dark": args.dark}
    for name, gain in POINTS:
        measured = read_capture(paths[name])
        predicted, band, t_q24 = lnr_from_blob(blob, measured, int(gain * 65536))
        misses = []

        for i, (want, got) in enumerate(zip(measured, predicted)):
            reg = BANK + i * 4
            if reg in SKIP:
                continue
            if want != got:
                misses.append((reg, want, got))

        if misses:
            for reg, want, got in misses:
                print(f"{name}: {reg:#06x} measured {want:#010x} predicted {got:#010x}")
            sys.exit(f"{name}: {len(misses)} lnr mismatches")

        print(f"{name:6s} gain {gain:.5f} band {band - 1},{band} "
              f"t_q24 {t_q24} matched {REGS - len(SKIP)} ladder registers")

    print("\nlnr ladder agrees with ar-isp-ladder.h and both measured points")
    return 0


if __name__ == "__main__":
    sys.exit(main())
