#!/usr/bin/env python3
"""
Prove the cnf ladder transform in ar-isp-cnf.h against the measured register.

The cnf driver in the vendor service (0x1a1f68, packer 0x1a1c28) recomputes its
strength from a ladder in the tuning file whenever the 3A loop moves the gain:
eleven [low, high] float32 bands on the powers-of-two anchor layout, and a
0x80c-byte payload record per band holding a set flag and the strength value.
Inside a band the value is used verbatim; between bands it blends linearly and
truncates toward zero. The packer writes it into 0x3c64 bits 4:1 with bit 5 set.

This ladder carries no four-word header. The words preceding the edge array are
(0, 0x80, 0x80, 0x100), which is not the header shape, so the band count and the
interpolate flag come from the packer's own code and are constants in the header
and here. Locating a header for them is open work, and this script asserts the
absence rather than letting a future reader assume one was read.

The measured state is one streaming-vendor capture,
out/au-snapshot/registers.txt, whose abscissa the capture does not record. This
script derives the interval the capture is consistent with, which is the cnf
half of the joint bracket that check-cfa-ladder.py uses: cfa pins the abscissa
at or below 8.0 and cnf pins it at or above 4.0.

Refuses to pass unless:

  the band edges are strictly increasing, every record holds exactly two
  non-zero words at the flag and strength offsets, and the strength ramp is
  non-decreasing and inside the field's range;

  no four-word ladder header precedes the edge array;

  the abscissas consistent with the captured field form exactly one interval,
  and that interval is [4.0, 16.0);

  the reciprocal-square normalisation reproduces 0x3c84 and 0x3c88 at two
  strengths, the live vendor's and the library static image's, and its integer
  form agrees with the vendor's float arithmetic;

  the transform reproduces the captured register at every abscissa in the
  bracket the two stages jointly recover.

The blend arithmetic runs inside that bracket but proves little: both records it
blends carry the same strength, so any fraction returns the same value. A
capture at a second abscissa is what exercises it, and this script says so.

The tuning file is proprietary and is not in the repository.

    kernel/scripts/isp/check-cnf-ladder.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin
"""

import argparse
import struct
import sys

from blob_layout import Layout

_LAY = Layout.load()


BANDS = _LAY["cnf_bands"].offset
PAYLOAD = _LAY["cnf_payload"].offset
STRIDE = _LAY["cnf_payload"].stride
COUNT = _LAY["cnf_payload"].count
INTERP = 1
WORD_FLAG = 0x00
WORD_STRENGTH = 0x04
STRENGTH_MIN = 1
STRENGTH_MAX = 6

REG = 0x3C64
REG_MASK = 0x000000FE
REG_SHIFT = 1
REG_BITS = 0xF
ENABLE_BITS = 0x20

# The two normalised registers, packer 0x1a1c28 tail.
NORM_A = 0x3C84
NORM_B = 0x3C88
NORM_A_BIT = 1 << 17
NORM_SHIFT_POS = 12
NORM_QUOT_MAX = 0xFFE
NORM_SHIFT_MAX = 30
NORM_SHIFT_MIN = 9
NORM_CONST_B = 2

# 0x3c84 and 0x3c88 in the module's static register image, which the library
# carries at strength 4. Both the live vendor at strength 2 and this pair must
# come out of the same law, which is what makes it two points rather than one.
LIBRARY_STRENGTH = 4
LIBRARY_NORM_A = 0x0002F800
LIBRARY_NORM_B = 0x0000D800
MEASURED_NORM_A = 0x0002D800
MEASURED_NORM_B = 0x0000D800

# The joint bracket recovered from cfa and cnf together, in Q16.
BRACKET = (4 << 16, 8 << 16)

# The interval this script must derive from the captured field alone, in Q16.
CONSISTENT = (4 << 16, 16 << 16)

MEASURED = 0x000A0D25

# The gain-independent run 0x3c8c..0x3ca0, packed from the tuning file at
# 0x8e1a8..0x8e1d4. The library's static register image is the oracle: the
# packer is what produced it, so the blob must reproduce it word for word.
STATIC_REG = 0x3C8C
_S = _LAY["cnf_statics"]
STATICS = [
    (_S.field_offset(lo) + i * 4, _S.field_offset(hi) + i * 4, mask)
    for i in range(3)
    for lo, hi, mask in (("wide_lo", "wide_hi", 0x003FF800),
                         ("narrow_lo", "narrow_hi", 0x000FF800))
]
LIBRARY_STATIC = [0x00080100, 0x00000100, 0x0022CC01,
                  0x000401C5, 0x0025B966, 0x00040000]


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


def walk(blob: bytes, gain_q16: int) -> tuple[int, int]:
    """Mirror ar_isp_ladder_walk with cnf's constant count and interpolate."""
    band = COUNT - 1
    for i in range(COUNT - 1):
        hi = f32_q16(struct.unpack_from("<I", blob, BANDS + i * 8 + 4)[0])
        if gain_q16 <= hi:
            band = i
            break

    t_q24 = 0
    if INTERP and band > 0:
        lo = f32_q16(struct.unpack_from("<I", blob, BANDS + band * 8)[0])
        prev_hi = f32_q16(struct.unpack_from("<I", blob,
                                             BANDS + band * 8 - 4)[0])
        if gain_q16 < lo and lo > prev_hi:
            t_q24 = ((gain_q16 - prev_hi) << 24) // (lo - prev_hi)

    return band, t_q24


def strength(blob: bytes, gain_q16: int) -> int:
    """Mirror ar_isp_cnf_strength_from_blob."""
    band, t_q24 = walk(blob, gain_q16)
    value = word(blob, band, WORD_STRENGTH)
    if t_q24:
        value = blend_s32(word(blob, band - 1, WORD_STRENGTH), value, t_q24)

    return min(max(value, STRENGTH_MIN), STRENGTH_MAX)


def pack(value: int) -> int:
    """Mirror ar_isp_cnf_pack."""
    return ((value & REG_BITS) << REG_SHIFT) | ENABLE_BITS


def normalise(v: int) -> tuple[int, int]:
    """Mirror ar_isp_cnf_normalise, in the vendor's float arithmetic."""
    square = float(v) * float(v)
    shift = NORM_SHIFT_MAX
    while True:
        # float32 divide, then + 0.5 in double and truncate: round half up.
        quot = int((1 << shift) / square + 0.5)
        if quot <= NORM_QUOT_MAX or shift == NORM_SHIFT_MIN:
            return quot, shift

        shift -= 1


def norm_pack(v: int) -> int:
    """Mirror ar_isp_cnf_norm_pack."""
    quot, shift = normalise(v)
    return (quot & 0xFFF) | (shift << NORM_SHIFT_POS)


def check_statics(blob: bytes) -> None:
    """Mirror ar_isp_cnf_static_pack against the library's static image."""
    for i, (lo_off, hi_off, hi_mask) in enumerate(STATICS):
        lo = struct.unpack_from("<I", blob, lo_off)[0] & 0x7FF
        hi = struct.unpack_from("<I", blob, hi_off)[0] << 11
        got = lo | (hi & hi_mask)
        want = LIBRARY_STATIC[i] & (0x7FF | hi_mask)
        if got != want:
            sys.exit(f"{STATIC_REG + i * 4:#06x}: blob gives {got:#010x}, "
                     f"library image has {want:#010x}")

    print(f"statics: {len(STATICS)} registers {STATIC_REG:#06x}.."
          f"{STATIC_REG + (len(STATICS) - 1) * 4:#06x} reproduce the library "
          f"image from the tuning file")


def check_normalisation() -> None:
    """
    The normalisation must reproduce both known points: the live vendor at the
    strength its own 0x3c64 encodes, and the library's static image at the
    strength its 0x3c64 encodes. Two strengths, one law.
    """
    live = (MEASURED & REG_MASK) >> REG_SHIFT & REG_BITS
    for name, strength, want_a, want_b in (
        ("vendor", live, MEASURED_NORM_A, MEASURED_NORM_B),
        ("library", LIBRARY_STRENGTH, LIBRARY_NORM_A, LIBRARY_NORM_B),
    ):
        got_a = norm_pack(strength) | NORM_A_BIT
        got_b = norm_pack(NORM_CONST_B)
        if got_a != want_a:
            sys.exit(f"{name} strength {strength}: {NORM_A:#06x} want "
                     f"{want_a:#010x} got {got_a:#010x}")

        if got_b != want_b:
            sys.exit(f"{name} strength {strength}: {NORM_B:#06x} want "
                     f"{want_b:#010x} got {got_b:#010x}")

        print(f"normalise: {name} strength {strength} -> {NORM_A:#06x} "
              f"{got_a:#010x}, {NORM_B:#06x} {got_b:#010x}, both exact")

    # The kernel has no FPU, so the header does this in integers. Same answers.
    for strength in range(1, STRENGTH_MAX * 4):
        square = strength * strength
        shift = NORM_SHIFT_MAX
        while True:
            quot = ((1 << shift) * 2 + square) // (2 * square)
            if quot <= NORM_QUOT_MAX or shift == NORM_SHIFT_MIN:
                break

            shift -= 1

        if (quot, shift) != normalise(strength):
            sys.exit(f"integer normalisation diverges at strength {strength}: "
                     f"{(quot, shift)} against {normalise(strength)}")

    print(f"normalise: the integer form matches the float law for strengths "
          f"1..{STRENGTH_MAX * 4 - 1}")


def check_records(blob: bytes) -> list[int]:
    ramp = []
    for band in range(COUNT):
        record = blob[PAYLOAD + band * STRIDE:PAYLOAD + (band + 1) * STRIDE]
        nonzero = [off for off in range(0, STRIDE - 3, 4)
                   if struct.unpack_from("<I", record, off)[0]]
        if nonzero != [WORD_FLAG, WORD_STRENGTH]:
            sys.exit(f"record {band} has non-zero words at "
                     f"{[hex(o) for o in nonzero]}, expected the flag and "
                     f"strength pair")

        value = word(blob, band, WORD_STRENGTH)
        if not STRENGTH_MIN <= value <= STRENGTH_MAX:
            sys.exit(f"record {band} strength {value} is outside "
                     f"[{STRENGTH_MIN}, {STRENGTH_MAX}]")

        ramp.append(value)

    if ramp != sorted(ramp):
        sys.exit(f"strength ramp {ramp} is not non-decreasing")

    print(f"records: {COUNT} bands, two words each, strength ramp {ramp}")
    return ramp


def check_no_header(blob: bytes) -> None:
    header = struct.unpack_from("<4I", blob, BANDS - 0x10)
    if header == (1, 1, COUNT, 0):
        sys.exit(f"a four-word header reading {header} precedes the edge "
                 f"array; the count and interpolate constants should be read "
                 f"from it")

    print(f"header: the four words before the edges read {header}, so the "
          f"count of {COUNT} and interpolate {INTERP} stay constants")


def check_consistent_interval(blob: bytes) -> None:
    """The abscissas whose packed field matches the capture, as an interval."""
    want = MEASURED & REG_MASK
    step = 1 << 10
    hits = [g for g in range(1 << 16, 32 << 16, step)
            if pack(strength(blob, g)) == want]
    if not hits:
        sys.exit("no abscissa reproduces the captured field")

    runs = 1 + sum(1 for a, b in zip(hits, hits[1:], strict=False)
                   if b - a != step)
    if runs != 1:
        sys.exit(f"the captured field is reproduced on {runs} disjoint "
                 f"intervals, so it does not identify one")

    lo, hi = hits[0], hits[-1] + step
    if (lo, hi) != CONSISTENT:
        sys.exit(f"consistent interval is [{lo / 65536}, {hi / 65536}), "
                 f"expected [{CONSISTENT[0] / 65536}, "
                 f"{CONSISTENT[1] / 65536})")

    print(f"capture: field {want >> REG_SHIFT & REG_BITS} is reproduced on "
          f"exactly [{lo / 65536}, {hi / 65536}), one interval")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tuning", required=True, help="nt99235 tuning blob")
    args = ap.parse_args()

    with open(args.tuning, "rb") as handle:
        blob = handle.read()

    edges = struct.unpack_from(f"<{COUNT * 2}f", blob, BANDS)
    if list(edges) != sorted(edges) or len(set(edges)) != COUNT * 2:
        sys.exit("band edges are not strictly increasing")

    check_records(blob)
    check_no_header(blob)
    check_consistent_interval(blob)
    check_normalisation()
    check_statics(blob)

    lo, hi = BRACKET
    for gain_q16 in (lo, (lo + hi) // 2, hi):
        band, t_q24 = walk(blob, gain_q16)
        got = (MEASURED & ~REG_MASK) | pack(strength(blob, gain_q16))
        if got != MEASURED:
            sys.exit(f"abscissa {gain_q16 / 65536}: want {MEASURED:#010x} "
                     f"got {got:#010x}")

        print(f"abscissa {gain_q16 / 65536:9.6f} -> band {band} "
              f"t {t_q24 / (1 << 24):.7f}, {REG:#06x} = {got:#010x} exact")

    print("\ncnf ladder agrees with ar-isp-cnf.h and the measured register")
    print("Weakly exercised: the blend, because both records it spans in the "
          "bracket carry the same strength.")
    print("A capture at abscissa 15.3828125 blends 2 into 3 and closes that "
          "gap.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
