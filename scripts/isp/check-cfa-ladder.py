#!/usr/bin/env python3
"""
Prove the cfa ladder transform in ar-isp-cfa.h against the measured bank.

The cfa driver in the vendor service (0x1b1f28, packer 0x1b1d90) recomputes the
bank 0x0800 registers from a ladder in the tuning file whenever the 3A loop
moves the gain: a header (enable, interpolate, count, selector), five
[low, high] float32 bands, and a 0xa4-byte payload record per band.

The driver has two paths, and they are not the same arithmetic. Inside a band
it memcpy's the whole record into its config block, so the record reaches the
bank verbatim. Across a band gap it blends in binary32: scvtf on each word,
fmul for the upper term, a fused fmadd, then fcvtzs. That is reproduced here in
Python float32 and in the kernel by ar-isp-softfloat.h, because an exact
integer blend is wrong by one wherever the true value lands just under an
integer, including where both records are equal. Blending 75 into 75 gives 74
on hardware.

Two words in the vendor's unrolled float block take their operands from the
wrong record index: word 6 reads its low operand from word 4, and word 28 reads
both operands from word 26. Both are measured, in two gaps at two different
fractions. They are quirks to reproduce, not bugs to fix.

The whole record lands in the bank as four ascending runs of consecutive
registers, each of the 41 words used once, stepping over 13 registers that keep
their replayed values. That map is this script's first assertion, because
hdf-076 published it off by one register at two of the run boundaries; it was
later confirmed directly from the packer's disassembly.

This script refuses to pass unless:

  the header reads (enable 1, interpolate 1, count 5, selector 0), the band
  edges are strictly increasing, and the sixth payload slot is zero padding;

  the four runs consume the 41 payload words exactly once each, in ascending
  order, and cover 41 of the packer's 54 registers;

  the transform reproduces all 41 registers of the original streaming-vendor
  capture across the [4.0, 8.0] bracket cfa and cnf jointly recover;

  every capture in CAPTURES is reproduced exactly somewhere inside the abscissa
  bracket the rnr ladder derived from the same breath. Two of those are gap
  captures, so the blend and both quirks are exercised rather than assumed.

The tuning file is proprietary and is not in the repository.

    kernel/scripts/isp/check-cfa-ladder.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin
"""

import argparse
import struct
import sys

from blob_layout import Layout

_LAY = Layout.load()


HEADER = _LAY["cfa_header"].offset
BANDS = _LAY["cfa_bands"].offset
PAYLOAD = _LAY["cfa_payload"].offset
STRIDE = _LAY["cfa_payload"].stride
COUNT = _LAY["cfa_payload"].count
BANK = 0x0800

# ar_isp_cfa_runs, as (bank offset, record offset, register count).
RUNS = [(0x00, 0x00, 4), (0x3C, 0x10, 7), (0x5C, 0x2C, 19), (0xAC, 0x78, 11)]

# The packer's extent, 0x0800..0x08d4 inclusive.
PACKER_REGS = 54


# The abscissa bracket recovered from cfa and cnf jointly, in Q16.
BRACKET = (4 << 16, 8 << 16)

MEASURED = {
    0x0800: 0x0000001E, 0x0804: 0x0000001E, 0x0808: 0x0000001E,
    0x080C: 0x0000001E, 0x083C: 0x00000021, 0x0840: 0x0000001A,
    0x0844: 0x0000001D, 0x0848: 0x0000004B, 0x084C: 0x00000035,
    0x0850: 0x00000025, 0x0854: 0x00000012, 0x085C: 0x000001F4,
    0x0860: 0x00000FA0, 0x0864: 0x00002710, 0x0868: 0x000DBBA0,
    0x086C: 0x00000001, 0x0870: 0x00000080, 0x0874: 0x00000000,
    0x0878: 0x00000028, 0x087C: 0x00000006, 0x0880: 0x00000080,
    0x0884: 0x00000000, 0x0888: 0x00000040, 0x088C: 0x00000040,
    0x0890: 0x00000080, 0x0894: 0x00000000, 0x0898: 0x00000040,
    0x089C: 0x00000040, 0x08A0: 0x00000073, 0x08A4: 0x0000000D,
    0x08AC: 0x00000001, 0x08B0: 0x00000001, 0x08B4: 0x00000078,
    0x08B8: 0x000000FF, 0x08BC: 0x00000001, 0x08C0: 0x00000032,
    0x08C4: 0x00000064, 0x08C8: 0x00000001, 0x08CC: 0x000007D0,
    0x08D0: 0x00001F40, 0x08D4: 0x0000000F,
}


def word(blob: bytes, band: int, off: int) -> int:
    return struct.unpack_from("<i", blob, PAYLOAD + band * STRIDE + off)[0]


def check_run_map() -> None:
    """The runs must consume every payload word once, in ascending order."""
    words = []
    regs = []
    for reg, rec, count in RUNS:
        words.extend(rec + k * 4 for k in range(count))
        regs.extend(BANK + reg + k * 4 for k in range(count))

    if words != sorted(words) or len(set(words)) != len(words):
        sys.exit("run map reuses or reorders payload words")

    if words != list(range(0, STRIDE, 4)):
        sys.exit(f"run map covers {len(words)} words, not the record's "
                 f"{STRIDE // 4}")

    if len(regs) != len(set(regs)):
        sys.exit("run map writes a register twice")

    if max(regs) - BANK >= PACKER_REGS * 4:
        sys.exit("run map leaves the packer's extent")

    print(f"run map: {len(words)} payload words into {len(regs)} of the "
          f"packer's {PACKER_REGS} registers, each word once")


def f32(x: float) -> float:
    """Round to binary32, which is the precision the vendor computes in."""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def fma32(a: float, b: float, c: float) -> float:
    """One rounding, as fmadd does. Doubles hold a*b+c exactly here."""
    return f32(float(a) * float(b) + float(c))


# The two source-index quirks in the vendor's unrolled float block. Word 6 takes
# its low operand from word 4; word 28 takes both operands from word 26. Both
# are measured, in two gaps at two different fractions.
QUIRK_LOW = {6: 4, 28: 26}
QUIRK_HIGH = {28: 26}


# The driver issues 37 fmadd for 41 words: these four are copied verbatim from
# the upper record instead of blended. The header blends all 41, which is only
# safe while the two agree, so check_schedule_equivalence asserts that on the
# file in hand rather than letting a future tuning file diverge silently.
SCHEDULE_VERBATIM = (30, 31, 34, 37)


def check_schedule_equivalence(blob: bytes) -> None:
    """Blending the verbatim-scheduled words must equal copying them."""
    steps = 1 << 12
    for band in range(1, COUNT):
        for step in range(1, steps):
            t = f32(step / steps)
            for index in SCHEDULE_VERBATIM:
                upper = word(blob, band, index * 4)
                blended = int(fma32(f32(1.0 - t),
                                    word(blob, band - 1, index * 4),
                                    f32(t * upper)))
                if blended != upper:
                    sys.exit(f"band {band} word {index}: blend {blended} "
                             f"differs from the packer's verbatim {upper} "
                             f"at t {t:.6f}")

    print(f"schedule: blending the {len(SCHEDULE_VERBATIM)} "
          f"verbatim-scheduled words agrees with copying them, at "
          f"{steps - 1} fractions in each of {COUNT - 1} band gaps")



def vendor_select(blob: bytes, gain_q16: int) -> tuple[int, float]:
    """Band and gap fraction, in binary32 as isp_sub_cfa computes them."""
    count = struct.unpack_from("<I", blob, HEADER + 0x8)[0]
    interp = struct.unpack_from("<I", blob, HEADER + 0x4)[0]
    count = COUNT if not 1 <= count <= COUNT else count
    gain = f32(gain_q16 / 65536.0)

    band = count - 1
    for i in range(count - 1):
        hi = struct.unpack_from("<f", blob, BANDS + i * 8 + 4)[0]
        if gain <= hi:
            band = i
            break

    t = 0.0
    if interp and band > 0:
        lo = struct.unpack_from("<f", blob, BANDS + band * 8)[0]
        prev_hi = struct.unpack_from("<f", blob, BANDS + band * 8 - 4)[0]
        if gain < lo and lo > prev_hi:
            t = f32(f32(gain - prev_hi) / f32(lo - prev_hi))

    return band, t


def vendor_word(blob: bytes, band: int, index: int, t: float) -> int:
    """Verbatim inside a band; the fused float blend across a gap."""
    if not t:
        return word(blob, band, index * 4) & 0xFFFFFFFF

    lo = word(blob, band - 1, QUIRK_LOW.get(index, index) * 4)
    hi = word(blob, band, QUIRK_HIGH.get(index, index) * 4)

    return int(fma32(f32(1.0 - t), lo, f32(t * hi))) & 0xFFFFFFFF


def vendor_bank(blob: bytes, gain_q16: int) -> list[tuple[int, int]]:
    """Mirror ar_isp_cfa_from_blob, returning (register, value) in run order."""
    band, t = vendor_select(blob, gain_q16)
    out = []

    for reg, rec, count in RUNS:
        for k in range(count):
            out.append((BANK + reg + k * 4,
                        vendor_word(blob, band, rec // 4 + k, t)))

    return out


# Measured banks in run order, with the abscissa bracket the rnr ladder derived
# for the same breath. The two gap captures are what proves the blend; the two
# in-band ones guard the verbatim path.
CAPTURES = (
    ("breath-light1  band 2, verbatim", 22.6094, 22.7344, (
        30, 30, 30, 30, 33, 26, 29, 75, 53, 37, 18, 500, 4000, 10000,
        3000000, 1, 128, 0, 40, 6, 128, 0, 64, 64, 128, 0, 64, 64, 115,
        13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)),
    ("breath-covered band 3, verbatim", 63.5781, 63.9844, (
        30, 30, 30, 30, 33, 26, 29, 75, 53, 37, 18, 500, 4000, 10000,
        3000000, 1, 128, 0, 40, 6, 128, 0, 64, 64, 128, 0, 64, 64, 115,
        13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)),
    ("breath-gap44-b gap 42.1..48.0", 44.4375, 44.8594, (
        29, 29, 29, 29, 33, 26, 31, 74, 53, 37, 18, 500, 4000, 10000,
        3000000, 1, 128, 0, 40, 6, 128, 0, 64, 64, 128, 0, 64, 64, 64,
        13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)),
    ("breath-l2      gap 12.1..16.0", 12.9648, 13.5664, (
        30, 30, 30, 30, 33, 26, 31, 75, 53, 37, 18, 500, 4000, 10000,
        3000000, 1, 128, 0, 40, 6, 128, 0, 64, 64, 128, 0, 64, 64, 64,
        3, 0, 0, 7, 179, 0, 0, 1, 0, 5616, 10531, 4)),
)


def check_captures(blob: bytes) -> None:
    """
    Every captured bank must be reproduced somewhere inside the bracket an
    independent ladder derived for that capture. The gap rows are the ones that
    exercise the float blend and the two source quirks; an integer blend misses
    both of them, which is how the defect was found.
    """
    for name, lo, hi, want in CAPTURES:
        hit = None
        for gain_q16 in range(int(lo * 65536), int(hi * 65536) + 1):
            got = [v for _, v in vendor_bank(blob, gain_q16)]
            if got == list(want):
                hit = gain_q16
                break

        if hit is None:
            sys.exit(f"{name}: no abscissa in [{lo}, {hi}] reproduces the "
                     f"captured bank")

        band, t = vendor_select(blob, hit)
        kind = "gap" if t else "verbatim"
        print(f"{name}: 41/41 at abscissa {hit / 65536:.6f}, band {band} "
              f"t {t:.6f} [{kind}]")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tuning", required=True, help="nt99235 tuning blob")
    args = ap.parse_args()

    with open(args.tuning, "rb") as handle:
        blob = handle.read()

    header = struct.unpack_from("<4I", blob, HEADER)
    if header != (1, 1, COUNT, 0):
        sys.exit(f"cfa header reads {header}, expected (1, 1, {COUNT}, 0)")

    edges = struct.unpack_from(f"<{COUNT * 2}f", blob, BANDS)
    if list(edges) != sorted(edges) or len(set(edges)) != COUNT * 2:
        sys.exit("band edges are not strictly increasing")

    padding = blob[PAYLOAD + COUNT * STRIDE:PAYLOAD + (COUNT + 1) * STRIDE]
    if any(padding):
        sys.exit(f"payload slot {COUNT} is not zero padding")

    check_run_map()
    check_schedule_equivalence(blob)

    lo, hi = BRACKET
    for gain_q16 in (lo, (lo + hi) // 2, hi):
        band, t = vendor_select(blob, gain_q16)
        bad = 0
        for reg, value in vendor_bank(blob, gain_q16):
            if value != MEASURED[reg]:
                bad += 1
                print(f"  {reg:#06x}: want {MEASURED[reg]:#010x} "
                      f"got {value:#010x}")
        if bad:
            sys.exit(f"abscissa {gain_q16 / 65536} misses {bad} registers")

        print(f"abscissa {gain_q16 / 65536:9.6f} -> band {band} "
              f"t {t:.7f}, all {len(MEASURED)} captured registers exact")

    check_captures(blob)

    print("\ncfa ladder agrees with ar-isp-cfa.h and every measured bank, "
          "in band and in gap")
    return 0


if __name__ == "__main__":
    sys.exit(main())
