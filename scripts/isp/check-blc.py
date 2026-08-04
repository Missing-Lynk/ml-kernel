#!/usr/bin/env python3
"""
Check the BLC layout in ar-isp-blc.h against the tuning file and the trace.

BLC has no captured DMA page, so the oracle is the write trace: sixteen
registers at CVISP 0x4200, written once each. The check is that the values the
hardware received are reachable from the tuning file under the packing the
header describes, that the shift is 6 and not something else that happens to
fit, and that the entry table really is five entries and stops there.

The tuning file and the trace are proprietary and are not in the repository.

    kernel/scripts/isp/check-blc.py \\
        --blob out/air-gather/camera/nt99235_tuning_preview_fpv.bin \\
        --trace out/au-mmiotrace/wide-sweep.log
"""

import argparse
import re
import struct
import sys
from collections.abc import Sequence

CVISP_BASE = 0x08E00000
BANK = 0x4200
BLOCK = 0x40

LADDER = 0x34
TABLE = 0xB4
ENTRIES = 5
ENTRY_SIZE = 0x20
LANE = 4
SCALE_SHIFT = 6

REG_SCALE = 0x00
REG_LEVEL = 0x20


def entries(blob: bytes) -> list[tuple[list[int], list[int]]]:
    out = []
    for i in range(ENTRIES):
        off = TABLE + i * ENTRY_SIZE
        vals = struct.unpack_from("<8I", blob, off)
        out.append((list(vals[:4]), list(vals[4:])))

    return out


def ladder(blob: bytes) -> list[tuple[float, float]]:
    return [struct.unpack_from("<2f", blob, LADDER + 8 * i)[0:2]
            for i in range(ENTRIES)]


def bank_values(trace: str) -> dict[int, int]:
    last = {}
    pat = re.compile(r"pa=0x([0-9a-f]+) val=0x([0-9a-f]+)")
    with open(trace) as handle:
        for line in handle:
            m = pat.search(line)
            if not m:
                continue

            off = int(m.group(1), 16) - CVISP_BASE
            if BANK <= off < BANK + BLOCK:
                last[off - BANK] = int(m.group(2), 16)

    return last


def check_table(blob: bytes) -> list[tuple[list[int], list[int]]]:
    table = entries(blob)
    for i, (scale, level) in enumerate(table):
        if min(level) < 1 or max(level) > 0xFFFF:
            sys.exit(f"entry {i}: level {level} outside a sane black level")
        if min(scale) < 1:
            sys.exit(f"entry {i}: scale {scale} has a zero lane")

    # Both groups rise with the entry index, which is what makes the ladder a
    # gain ordering rather than an arbitrary list.
    for i in range(1, ENTRIES):
        if table[i][0][0] <= table[i - 1][0][0]:
            sys.exit(f"entry {i}: scale does not rise")
        if table[i][1][0] < table[i - 1][1][0]:
            sys.exit(f"entry {i}: level falls")

    # The table must END here, or the entry count is wrong.
    past = struct.unpack_from("<8I", blob, TABLE + ENTRIES * ENTRY_SIZE)
    if any(past):
        sys.exit(f"a {ENTRIES + 1}th entry follows: {list(past)}")

    print(f"table: {ENTRIES} entries of {ENTRY_SIZE:#x} at {TABLE:#x}, "
          f"scale {table[0][0][0]}..{table[-1][0][0]}, "
          f"level {table[0][1][0]}..{table[-1][1][0]}")
    print(f"       nothing follows entry {ENTRIES - 1}")
    return table


def check_ladder(blob: bytes) -> list[tuple[float, float]]:
    pairs = ladder(blob)
    for i, (a, b) in enumerate(pairs):
        if not 0 < a < 1e5 or not 0 < b < 1e5 or b <= a:
            sys.exit(f"ladder pair {i}: {a}, {b} is not an increasing window")
    for i in range(1, ENTRIES):
        if pairs[i][0] <= pairs[i - 1][1]:
            sys.exit(f"ladder pair {i} overlaps pair {i - 1}")
    print(f"ladder: {ENTRIES} float pairs at {LADDER:#x}, "
          f"{', '.join(f'({a:g},{b:g})' for a, b in pairs)}")
    return pairs


def check_trace(table: Sequence[tuple[list[int], list[int]]],
                last: dict[int, int]) -> None:
    if len(last) != BLOCK // 4:
        sys.exit(f"trace holds {len(last)} bank registers, expected "
                 f"{BLOCK // 4}")

    level = [last[REG_LEVEL + 4 * i] for i in range(LANE)]
    scale = [last[REG_SCALE + 4 * i] for i in range(LANE)]

    # The level lane is stored unshifted, so it must appear in the table as it
    # is. This is the byte-level link from the tuning file to the hardware.
    seen = {v for _, lv in table for v in lv}
    for v in level:
        if v not in seen:
            sys.exit(f"level {v} ({v:#x}) is in no table entry; levels are "
                     f"{sorted(seen)}")

    # The scale lane is stored shifted, so it must divide exactly and land
    # inside the table's range.
    span = [v for sc, _ in table for v in sc]
    for v in scale:
        if v % (1 << SCALE_SHIFT):
            sys.exit(f"scale {v:#x} is not a multiple of "
                     f"{1 << SCALE_SHIFT}, so the shift is not {SCALE_SHIFT}")
        u = v >> SCALE_SHIFT
        if not min(span) <= u <= max(span):
            sys.exit(f"scale {v:#x} unshifts to {u}, outside the table range "
                     f"{min(span)}..{max(span)}")

    print(f"trace: level lane {level[0]} ({level[0]:#x}) is table entry data, "
          f"unshifted")
    print(f"       scale lane {scale[0]:#x} >> {SCALE_SHIFT} = "
          f"{scale[0] >> SCALE_SHIFT}, inside {min(span)}..{max(span)}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--blob", required=True, help="nt99235 tuning file")
    ap.add_argument("--trace", help="a trace covering CVISP 0x4200")
    args = ap.parse_args()

    with open(args.blob, "rb") as handle:
        blob = handle.read()

    table = check_table(blob)
    check_ladder(blob)

    if args.trace:
        last = bank_values(args.trace)
        if not last:
            sys.exit(f"no writes to CVISP {BANK:#x} in {args.trace}")
        check_trace(table, last)

    print("BLC layout agrees with ar-isp-blc.h")
    return 0


if __name__ == "__main__":
    sys.exit(main())
