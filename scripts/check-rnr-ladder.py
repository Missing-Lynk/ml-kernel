#!/usr/bin/env python3
"""
Prove the rnr ladder transform in ar-isp-ladder.h against both measured points.

The rnr driver in the vendor service (0x1993b8) recomputes the twelve packed
registers at ISP 0x1808..0x1834 from a ladder in the tuning file whenever the
3A loop moves the gain: a header (enable, interpolate, mode, count, selector),
twelve [low, high] float32 bands, and a 352-byte payload record per band.
Inside a band the payload is verbatim; between bands it is blended linearly
and truncated toward zero. Register k packs payload word 14+k in the high half
and word 2+k in the low half.

This script mirrors the kernel header's integer arithmetic exactly (Q16 band
edges, Q24 blend fraction, one truncation on the blended sum) and refuses to
pass unless:

  the header reads (enable 1, interpolate 1, mode 0, count 12, selector 0)
  and the band edges are strictly increasing;

  the transform at abscissa 1.0 reproduces 0x000f000a in all twelve
  registers, which is the cold bank the register replay carries and the value
  captured on our own hardware;

  the transform reproduces 0x002e002d, the vendor's live registers captured
  streaming at saturated AE, for some abscissa in the 13.6 to 14.2 window the
  band arithmetic demands (plans/au-blend-engine-and-notch.md section 2).

The tuning file is proprietary and is not in the repository.

    kernel/scripts/check-rnr-ladder.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin
"""

import argparse
import struct
import sys

HEADER = 0x79D8
BANDS = 0x79EC
PAYLOAD = 0x7A6C
STRIDE = 0x160
COUNT = 12
LO_WORD = 2
HI_WORD = 14
REGS = 12

COLD_REG = 0x000F000A
VENDOR_REG = 0x002E002D
VENDOR_WINDOW = (13.6, 14.2)


def f32_q16(bits):
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


def blend(a, b, t_q24):
    """Mirror ar_isp_ladder_blend: one Q24 sum, one truncation."""
    return (a * ((1 << 24) - t_q24) + b * t_q24) >> 24


def rnr_from_blob(blob, gain_q16):
    """Mirror ar_isp_rnr_from_blob word for word."""
    count = struct.unpack_from("<I", blob, HEADER + 0xC)[0]
    interp = struct.unpack_from("<I", blob, HEADER + 0x4)[0]
    count = COUNT if not 1 <= count <= COUNT else count

    edges = [f32_q16(struct.unpack_from("<I", blob, BANDS + i * 4)[0])
             for i in range(count * 2)]

    band = count - 1
    for i in range(count - 1):
        if gain_q16 <= edges[i * 2 + 1]:
            band = i
            break

    t_q24 = 0
    if interp and band > 0:
        lo, prev_hi = edges[band * 2], edges[band * 2 - 1]
        if gain_q16 < lo and lo > prev_hi:
            t_q24 = ((gain_q16 - prev_hi) << 24) // (lo - prev_hi)

    regs = []
    for k in range(REGS):
        lo, hi = (struct.unpack_from("<I", blob, PAYLOAD + band * STRIDE +
                                     (col + k) * 4)[0]
                  for col in (LO_WORD, HI_WORD))
        if t_q24:
            prev = PAYLOAD + (band - 1) * STRIDE
            lo = blend(struct.unpack_from("<I", blob, prev + (LO_WORD + k) * 4)[0],
                       lo, t_q24)
            hi = blend(struct.unpack_from("<I", blob, prev + (HI_WORD + k) * 4)[0],
                       hi, t_q24)

        regs.append(((hi << 16) | (lo & 0xFFFF)) & 0xFFFFFFFF)

    return regs


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tuning", required=True, help="nt99235 tuning blob")
    args = ap.parse_args()

    with open(args.tuning, "rb") as handle:
        blob = handle.read()

    header = struct.unpack_from("<5I", blob, HEADER)
    if header != (1, 1, 0, COUNT, 0):
        sys.exit(f"rnr header reads {header}, expected (1, 1, 0, {COUNT}, 0)")

    edges = struct.unpack_from(f"<{COUNT * 2}f", blob, BANDS)
    if list(edges) != sorted(edges) or len(set(edges)) != COUNT * 2:
        sys.exit("band edges are not strictly increasing")

    cold = rnr_from_blob(blob, 1 << 16)
    if cold != [COLD_REG] * REGS:
        sys.exit(f"abscissa 1.0 gives {cold[0]:#010x}, "
                 f"expected {COLD_REG:#010x} in all twelve")
    print(f"abscissa 1.0    -> {COLD_REG:#010x} x{REGS}, the replayed cold bank")

    hits = [g / 256 for g in range(int(VENDOR_WINDOW[0] * 256),
                                   int(VENDOR_WINDOW[1] * 256) + 1)
            if rnr_from_blob(blob, (g << 16) // 256) == [VENDOR_REG] * REGS]
    if not hits:
        sys.exit(f"no abscissa in {VENDOR_WINDOW} reproduces "
                 f"{VENDOR_REG:#010x}")
    print(f"abscissa {hits[0]:.3f}..{hits[-1]:.3f} -> {VENDOR_REG:#010x} x{REGS}, "
          f"the vendor's live bank")

    print("\nrnr ladder agrees with ar-isp-ladder.h and the two measured points")
    return 0


if __name__ == "__main__":
    sys.exit(main())
