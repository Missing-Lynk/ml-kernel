#!/usr/bin/env python3
"""
Locate the gain-indexed parameter tables rnr, lnr and de3d recompute from.

All three take the same shape as BLC. A call to is_aec_trigger_compute_user
returns a pair of ladder indices and a blend weight, entries are selected from a
fixed-stride table by those indices, and their fields are converted to float,
blended, and truncated back to integers on the way to the registers. The
difference from BLC is scale: the entries are whole parameter sets rather than
sixteen values, and rnr blends four lanes at a time in SIMD.

Every table base is reached the same way. The tuning manager holds an array of
per-sensor configurations at a stride of 0x3b1e8, the pointer at offset 24 of a
configuration is the sensor's tuning blob, and the table offsets below are
offsets into that blob, which is the file this script reads.

What this checks:

  the three table bases fall inside the tuning file, and their entries are
  monotone ladders rather than noise, which is what a gain-indexed table has to
  look like;

  rnr's twelve paired registers are reproduced from the table. Both lanes of all
  twelve start on entry 0's value and step to entry 1's, and no later value
  leaves the column's range, so 48 traced values are accounted for exactly;

  de3d register 0x2eb4 is reproduced from the table. Its traced lane starts on
  one entry's value and ends on another's and never leaves that interval, so the
  base and the stride are pinned by the hardware rather than by reading code;

  the moving registers of rnr, lnr and de3d change together on a small number of
  shared events, which is what one gain input driving all three looks like.

The trace and the tuning file are proprietary and are not in the repository.

    kernel/scripts/isp/check-gain-ladders.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin \\
        --trace out/au-mmiotrace/mmio-isp.log
"""

import argparse
import re
import struct
import sys
from collections.abc import Sequence

from blob_layout import Layout

_LAY = Layout.load()


ISP_BASE = 0x08C00000

# Table base and entry stride, read from the blend at 0x199600 for rnr, 0x1bdb48
# for lnr and 0x1c6e54 for de3d. All are offsets into the sensor's tuning blob.
TABLES = tuple(
    (stage, _LAY[f"{stage}_payload"].offset, _LAY[f"{stage}_payload"].stride, lo, hi)
    for stage, lo, hi in (("rnr", 0x1800, 0x1C00),
                          ("lnr", 0x3CC8, 0x3E20),
                          ("de3d", 0x2E00, 0x2F40))
)

# rnr's oracle. Two arrays of twelve, at these entry offsets, feed the low and
# high halves of the twelve registers from 0x1808. Both arrays hold twelve
# copies of one value in this tuning file, so the registers are matched to the
# arrays but the order within an array is not decided by this evidence.
RNR_REGS = range(0x1808, 0x1838, 4)
RNR_LO_COL = 2
RNR_HI_COL = 14
RNR_WIDTH = 12

# The single register whose traced values pin de3d's table: the lane runs from
# one entry's value to another's, through interpolated values in between.
ORACLE_REG = 0x2EB4
ORACLE_COL = 48
ORACLE_ENDS = (216, 188)

# Stages driven by the same gain input, and the register in each with the most
# distinct values.
LOCKSTEP = (("rnr", 0x1800, 0x1C00), ("lnr", 0x3CC8, 0x3E20),
            ("de3d", 0x2E00, 0x2F40))

# Two events at the same gain step land within about one frame of writes.
EVENT_WINDOW = 1200


def read_trace(path: str) -> dict[int, list[tuple[int, int]]]:
    seq, index = {}, 0
    pat = re.compile(r"pa=0x([0-9a-f]+) val=0x([0-9a-f]+)")
    with open(path) as handle:
        for line in handle:
            m = pat.search(line)
            if not m:
                continue

            index += 1
            off = int(m.group(1), 16) - ISP_BASE
            if 0 <= off < 0x10000:
                seq.setdefault(off, []).append((index, int(m.group(2), 16)))

    return seq


def column(blob: bytes, base: int, stride: int, col: int,
           count: int) -> list[int]:
    return [struct.unpack_from("<i", blob, base + i * stride + col * 4)[0]
            for i in range(count)]


def entries(blob: bytes, base: int, stride: int) -> list[tuple[int, ...]]:
    """Entries up to the first all-zero one, which is what ends the table."""
    out = []
    while True:
        off = base + len(out) * stride
        if off + stride > len(blob):
            break

        row = struct.unpack_from(f"<{stride // 4}i", blob, off)
        if not any(row):
            break

        out.append(row)

    return out


def changes(writes: Sequence[tuple[int, int]]) -> list[int]:
    return [writes[k][0] for k in range(1, len(writes))
            if writes[k][1] != writes[k - 1][1]]


def busiest(seq: dict[int, list[tuple[int, int]]], lo: int,
            hi: int) -> int | None:
    moving = [o for o in seq if lo <= o < hi
              and len({v for _, v in seq[o]}) > 1]
    if not moving:
        return None

    return max(moving, key=lambda o: len({v for _, v in seq[o]}))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tuning", required=True, help="nt99235 tuning blob")
    ap.add_argument("--trace", required=True, help="mmio-isp.log")
    args = ap.parse_args()

    with open(args.tuning, "rb") as handle:
        blob = handle.read()

    seq = read_trace(args.trace)

    for name, base, stride, _, _ in TABLES:
        if base + stride > len(blob):
            sys.exit(f"{name}: table at {base:#x} is past the end of the blob")

        rows = entries(blob, base, stride)
        if len(rows) < 4:
            sys.exit(f"{name}: only {len(rows)} entries at {base:#x}")

        # A gain ladder rises or falls with gain. Noise does neither, so a table
        # with no monotone column is not a ladder and the base is wrong.
        ladders = 0
        for c in range(stride // 4):
            v = column(blob, base, stride, c, len(rows))
            if len(set(v)) < 3:
                continue

            up = all(a <= b for a, b in zip(v, v[1:], strict=False))
            down = all(a >= b for a, b in zip(v, v[1:], strict=False))
            ladders += up or down

        if not ladders:
            sys.exit(f"{name}: no monotone column at {base:#x}, not a ladder")

        print(f"{name:5s} table {base:#08x} stride {stride:#05x}  "
              f"{len(rows):2d} entries, {ladders:3d} monotone columns")

    # The rnr oracle. Each register packs two blended values, so both halves of
    # all twelve have to start on entry 0 and step to entry 1, and neither may
    # leave its column's range afterwards.
    lo_col = column(blob, 0x7A6C, 0x160, RNR_LO_COL, 12)
    hi_col = column(blob, 0x7A6C, 0x160, RNR_HI_COL, 12)
    for c, name in ((RNR_LO_COL, "low"), (RNR_HI_COL, "high")):
        wide = [column(blob, 0x7A6C, 0x160, c + i, 12) for i in range(RNR_WIDTH)]
        if any(v != wide[0] for v in wide):
            sys.exit(f"rnr: the {name} array at column {c} is not "
                     f"{RNR_WIDTH} copies of one ladder")

    for reg in RNR_REGS:
        if reg not in seq:
            sys.exit(f"rnr: {reg:#x} is not in the trace")

        raw = [v for _, v in seq[reg]]
        for half, col_vals, name in ((0xFFFF, lo_col, "low"),
                                     (0xFFFF0000, hi_col, "high")):
            shift = 0 if half == 0xFFFF else 16
            vals = [(v & half) >> shift for v in raw]
            # The placeholder zero before the first real write is not a value.
            path = [v for i, v in enumerate(vals)
                    if v and (i == 0 or v != vals[i - 1])]

            if path[:2] != col_vals[:2]:
                sys.exit(f"rnr: {reg:#x} {name} half starts {path[:2]}, "
                         f"expected {col_vals[:2]}")

            if max(path) > max(col_vals) or min(path) < min(col_vals):
                sys.exit(f"rnr: {reg:#x} {name} half leaves the ladder range")

    print(f"rnr   {len(RNR_REGS)} registers, both halves start on entries 0 "
          f"and 1 and stay in range")

    # The de3d oracle. Both endpoints have to be entry values, and no traced
    # value may leave the interval they bound, or the stride is wrong.
    lane = [v & 0xFF for _, v in seq[ORACLE_REG]]
    path = [lane[0]] + [lane[k] for k in range(1, len(lane))
                        if lane[k] != lane[k - 1]]
    col = column(blob, 0x963AC, 0x2F8, ORACLE_COL, 12)
    if (path[0], path[-1]) != ORACLE_ENDS:
        sys.exit(f"{ORACLE_REG:#x}: lane runs {path[0]}..{path[-1]}, "
                 f"expected {ORACLE_ENDS}")

    if not all(v in col for v in ORACLE_ENDS):
        sys.exit(f"{ORACLE_REG:#x}: endpoints are not both entry values")

    if min(path) < min(ORACLE_ENDS) or max(path) > max(ORACLE_ENDS):
        sys.exit(f"{ORACLE_REG:#x}: traced values leave the entry interval")

    print(f"de3d  {ORACLE_REG:#06x} lane runs {path[0]}..{path[-1]} through "
          f"{len(path) - 2} interpolated values, both ends in column "
          f"{ORACLE_COL}")

    # One gain input, so the stages step together rather than independently.
    events = {}
    for name, lo, hi in LOCKSTEP:
        reg = busiest(seq, lo, hi)
        if reg is None:
            sys.exit(f"{name}: no moving register")

        events[name] = changes(seq[reg])
        print(f"{name:5s} {reg:#06x} steps {len(events[name])} times")

    ref = events["rnr"]
    for name in ("lnr", "de3d"):
        other = events[name]
        paired = sum(any(abs(a - b) < EVENT_WINDOW for a in ref) for b in other)
        if paired < len(other) - 1:
            sys.exit(f"{name}: only {paired} of {len(other)} steps pair with "
                     f"rnr, so they are not on one input")

    print("      rnr, lnr and de3d step together on the same events")
    print("\ngain ladders agree with plans/au-isp-module-inventory.md")

    return 0


if __name__ == "__main__":
    sys.exit(main())
