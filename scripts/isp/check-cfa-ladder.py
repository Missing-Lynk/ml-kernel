#!/usr/bin/env python3
"""
Prove the cfa ladder transform in ar-isp-cfa.h against the measured bank.

The cfa driver in the vendor service (0x1b1f28, packer 0x1b1d90) recomputes the
bank 0x0800 registers from a ladder in the tuning file whenever the 3A loop
moves the gain: a header (enable, interpolate, count, selector), five
[low, high] float32 bands, and a 0xa4-byte payload record per band. Inside a
band the record is used verbatim; between bands the words blend linearly and
truncate toward zero.

The whole record lands in the bank as four ascending runs of consecutive
registers, each of the 41 words used once, stepping over 13 registers that keep
their replayed values. That map is this script's first assertion, because
hdf-076 published it off by one register at two of the run boundaries.

The measured state is one streaming-vendor capture, out/au-snapshot/registers.txt,
whose abscissa the capture does not record. cfa and cnf recover it
independently and agree on the bracket [4.0, 8.0]: cfa reproduces band 0
including a word that any nonzero blend would move, and cnf's packed field
reads 2, which the truncation reaches only from 4.0 and holds until 16.0. The
whole bracket lies inside cfa band 0, so every abscissa in it selects that band
verbatim and the check runs across the bracket rather than at a point.

This script mirrors the kernel header's integer arithmetic exactly (Q16 band
edges, Q24 blend fraction, one truncation on the blended sum) and refuses to
pass unless:

  the header reads (enable 1, interpolate 1, count 5, selector 0), the band
  edges are strictly increasing, and the sixth payload slot is zero padding;

  the four runs consume the 41 payload words exactly once each, in ascending
  order, and cover 41 of the packer's 54 registers;

  taking working offsets 0x78, 0x7c, 0x88 and 0x94 verbatim from the upper
  record, which is what the packer's own schedule does, agrees with blending
  them at every fraction in every band gap, so the header's uniform blend
  cannot differ from the vendor on this file;

  the transform reproduces all 41 captured registers at every abscissa in the
  recovered bracket.

The blend arithmetic itself is NOT exercised: the one capture is a
verbatim-record selection. That gap closes with a capture at a second abscissa,
and this script reports it rather than passing over it silently.

The tuning file is proprietary and is not in the repository.

    kernel/scripts/isp/check-cfa-ladder.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin
"""

import argparse
import struct
import sys

HEADER = 0x24548
BANDS = 0x24558
PAYLOAD = 0x245D8
STRIDE = 0xA4
COUNT = 5
BANK = 0x0800

# ar_isp_cfa_runs, as (bank offset, record offset, register count).
RUNS = [(0x00, 0x00, 4), (0x3C, 0x10, 7), (0x5C, 0x2C, 19), (0xAC, 0x78, 11)]

# The packer's extent, 0x0800..0x08d4 inclusive.
PACKER_REGS = 54

# Working offsets the packer's schedule copies verbatim from the upper record.
SCHEDULE_VERBATIM = (0x78, 0x7C, 0x88, 0x94)

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
    """Mirror ar_isp_ladder_select on ar_isp_cfa_ladder."""
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


def cfa_from_blob(blob: bytes, gain_q16: int) -> list[tuple[int, int]]:
    """Mirror ar_isp_cfa_from_blob, returning (register, value) in run order."""
    band, t_q24 = select(blob, gain_q16)
    out = []

    for reg, rec, count in RUNS:
        for k in range(count):
            value = word(blob, band, rec + k * 4)
            if t_q24:
                value = blend_s32(word(blob, band - 1, rec + k * 4), value,
                                  t_q24)

            out.append((BANK + reg + k * 4, value & 0xFFFFFFFF))

    return out


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


def check_schedule_equivalence(blob: bytes) -> None:
    """
    The packer copies four late words verbatim from the upper record where the
    header blends them. Assert the two agree everywhere on this file, so the
    uniform blend cannot diverge from the vendor.
    """
    steps = 1 << 12
    for band in range(1, COUNT):
        for step in range(1, steps):
            t_q24 = (step << 24) // steps
            for off in SCHEDULE_VERBATIM:
                blended = blend_s32(word(blob, band - 1, off),
                                    word(blob, band, off), t_q24)
                if blended != word(blob, band, off):
                    sys.exit(f"band {band} offset {off:#x}: blend "
                             f"{blended} differs from the packer's verbatim "
                             f"{word(blob, band, off)} at t "
                             f"{t_q24 / (1 << 24):.6f}")

    print(f"schedule: blending {len(SCHEDULE_VERBATIM)} verbatim-scheduled "
          f"words agrees with copying them, at {steps - 1} fractions in each "
          f"of {COUNT - 1} band gaps")


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
        band, t_q24 = select(blob, gain_q16)
        bad = 0
        for reg, value in cfa_from_blob(blob, gain_q16):
            if value != MEASURED[reg]:
                bad += 1
                print(f"  {reg:#06x}: want {MEASURED[reg]:#010x} "
                      f"got {value:#010x}")
        if bad:
            sys.exit(f"abscissa {gain_q16 / 65536} misses {bad} registers")

        print(f"abscissa {gain_q16 / 65536:9.6f} -> band {band} "
              f"t {t_q24 / (1 << 24):.7f}, all {len(MEASURED)} captured "
              f"registers exact")

    print("\ncfa ladder agrees with ar-isp-cfa.h and the measured bank")
    print("NOT proven: the blend, because the one capture is a "
          "verbatim-record selection.")
    print("A capture at abscissa 15.3828125 blends bands 1 and 2 and closes "
          "that gap.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
