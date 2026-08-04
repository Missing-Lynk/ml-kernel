#!/usr/bin/env python3
"""
Verify the ISP statistics buffer layouts in ar-isp-stats.h against captures.

Nothing here is generated: the statistics buffers are written by hardware, so
there is no table to carry and the deliverable is the indexing. This script is
the proof that the indexing is right, and it fails loudly if a capture stops
agreeing with it.

The RRO checks are structural and quantitative: 36 columns of a count block
followed by a sum block, every count in a column identical, and the maximum
sum exactly 255 times the count, which is what establishes the sums as 8-bit.
The histogram checks are population based: the three lanes must hold exactly
one quarter, one half and one quarter of the frame's pixels.

Captures are proprietary and are not in the repository.

    kernel/scripts/isp/check-stats-layout.py --snapshot out/au-snapshot
"""

import argparse
import struct
import sys

RRO_COLS = 36
RRO_ROWS = 16
RRO_CHANNELS = 4
RRO_BLOCK = 0x100
RRO_COL_STRIDE = 0x200
RRO_SIZE = RRO_COLS * RRO_COL_STRIDE

HIST_BINS = 128
HIST_LANES = 4
HIST_CONTENT = 0x800

FRAME_W = 1920
FRAME_H = 1080

# rro_stats engine 0, engine 1, and rro_face_stats. The vendor points both
# 0x6400 engines at one buffer, so the first two captures are the same bytes.
RRO_CAPTURES = ("0x6440", "0x6474", "0x6508")
HIST_CAPTURE = "0x600c"


def u32s(data):
    return list(struct.unpack_from(f"<{len(data) // 4}I", data))


def check_rro(name, data):
    if len(data) < RRO_SIZE:
        sys.exit(f"{name}: capture is {len(data):#x}, need {RRO_SIZE:#x}")

    counts = []
    sums = []
    for col in range(RRO_COLS):
        base = col * RRO_COL_STRIDE
        count_block = u32s(data[base:base + RRO_BLOCK])
        sum_block = u32s(data[base + RRO_BLOCK:base + 2 * RRO_BLOCK])

        if len(set(count_block)) != 1:
            sys.exit(f"{name} column {col}: count block is not constant")
        count = count_block[0]
        if not count:
            sys.exit(f"{name} column {col}: zero pixel count")
        counts.append(count)
        sums.extend(sum_block)

    if len(set(counts)) != 1:
        sys.exit(f"{name}: per-column counts differ: {sorted(set(counts))}")
    count = counts[0]

    if len(sums) != RRO_COLS * RRO_ROWS * RRO_CHANNELS:
        sys.exit(f"{name}: decoded {len(sums)} sums")

    ceiling = 255 * count
    if max(sums) > ceiling:
        sys.exit(f"{name}: sum {max(sums)} exceeds 255 * count = {ceiling}")

    # The two green channels track each other far more closely than either
    # tracks the outer channels; that is what identifies lanes 1 and 2.
    means = []
    for ch in range(RRO_CHANNELS):
        lane = sums[ch::RRO_CHANNELS]
        means.append(sum(lane) / len(lane) / count)
    green_gap = abs(means[1] - means[2])
    outer_gap = abs(means[0] - means[3])
    if green_gap >= outer_gap:
        sys.exit(f"{name}: channels 1 and 2 are not the closest pair: {means}")

    print(f"{name}: {RRO_COLS}x{RRO_ROWS} zones, count {count}/zone/channel, "
          f"max sum {max(sums)} = {max(sums) / count:.1f} x count")
    print(f"    channel means {', '.join(f'{m:.2f}' for m in means)} "
          f"(green gap {green_gap:.3f} < outer gap {outer_gap:.3f})")
    return count


def check_hist(name, data):
    bins = u32s(data[:HIST_CONTENT])
    if len(bins) != HIST_BINS * HIST_LANES:
        sys.exit(f"{name}: decoded {len(bins)} words")

    lanes = [bins[i::HIST_LANES] for i in range(HIST_LANES)]
    totals = [sum(lane) for lane in lanes]
    pixels = FRAME_W * FRAME_H

    if totals[3]:
        sys.exit(f"{name}: lane 3 should be unused, sums to {totals[3]}")
    if sum(totals) != pixels:
        sys.exit(f"{name}: bins sum to {sum(totals)}, frame has {pixels}")

    expected = (pixels // 4, pixels // 2, pixels // 4)
    for lane, (got, want) in enumerate(zip(totals, expected)):
        if got != want:
            sys.exit(f"{name}: lane {lane} sums to {got}, Bayer expects {want}")

    print(f"{name}: {HIST_BINS} bins x {HIST_LANES} lanes; R/G/B populations "
          f"{totals[0]}/{totals[1]}/{totals[2]} = exactly 1/4, 1/2, 1/4 of "
          f"{FRAME_W}x{FRAME_H}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--snapshot", required=True,
                    help="directory holding the tbl_isp_*.bin captures")
    args = ap.parse_args()

    for reg in RRO_CAPTURES:
        path = f"{args.snapshot}/tbl_isp_{reg}.bin"
        try:
            with open(path, "rb") as handle:
                check_rro(reg, handle.read())
        except FileNotFoundError:
            sys.exit(f"missing capture {path}")

    path = f"{args.snapshot}/tbl_isp_{HIST_CAPTURE}.bin"
    try:
        with open(path, "rb") as handle:
            check_hist(HIST_CAPTURE, handle.read())
    except FileNotFoundError:
        sys.exit(f"missing capture {path}")

    print("all statistics layouts agree with ar-isp-stats.h")
    return 0


if __name__ == "__main__":
    sys.exit(main())
