#!/usr/bin/env python3
"""Measure where the exposure table bands under mains-flickering light, and what a fix would cost.

Artificial light driven from the mains ripples at twice the mains frequency. A rolling shutter
integrates each row over the exposure time, so rows that start at different phases of that ripple
receive different amounts of light and the frame carries horizontal bands. The band depth is the
residual of averaging the ripple over the exposure window:

    depth = |sinc(T / Tp)|,  T the exposure time, Tp = 1 / (2 * mains)

which is zero exactly when the exposure is a whole number of ripple periods, and approaches one as
the exposure shrinks below a period.

This reads the shipped exposure table and reports the depth at every operating point, the set of
line counts that are flicker-safe, and what it costs to move to one. Nothing here is a proposal:
it is the measurement a proposal has to be judged against, and the oracle for any anti-flicker
actuation added to ml-aed.

    check-ae-flicker.py --tuning nt99235.bin [--mains 50] [--fps 60] [--vts 1125]
"""

import argparse
import math
import pathlib
import struct
import sys

from blob_layout import Layout

_LAY = Layout.load()
_TABLE = _LAY["ae_exposure_table"]


def read_table(blob: bytes) -> list[tuple[int, int]]:
    """The exposure table as {gain Q8, line count}."""
    out: list[tuple[int, int]] = []
    for i in range(_TABLE.count):
        gain, lines = struct.unpack_from("<II", blob, _TABLE.offset + i * _TABLE.stride)

        out.append((gain, lines))

    return out


def band_depth(lines: int, line_us: float, mains_hz: float) -> float:
    """Residual ripple after integrating over the exposure, 0 is clean and 1 is worst."""
    t = lines * line_us / 1e6
    period = 1.0 / (2 * mains_hz)
    x = t / period

    if x == 0:
        return 1.0

    return abs(math.sin(math.pi * x) / (math.pi * x))


def safe_lines(line_us: float, mains_hz: float, vts: int) -> list[int]:
    """Line counts whose exposure is a whole number of ripple periods, within one frame."""
    period_ms = 1000.0 / (2 * mains_hz)

    out: list[int] = []
    for n in range(1, vts + 1):
        ms = n * line_us / 1000.0
        rem = ms % period_ms

        if min(rem, period_ms - rem) < line_us / 1000.0:
            out.append(n)

    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tuning", required=True, type=pathlib.Path)
    ap.add_argument("--mains", type=float, default=50.0, help="mains frequency in Hz")
    ap.add_argument("--fps", type=float, default=60.0)
    ap.add_argument("--vts", type=int, default=1125, help="frame length in lines")
    ap.add_argument("--threshold", type=float, default=2.0,
                    help="band depth in percent above which an entry counts as banding")
    args = ap.parse_args()

    table = read_table(args.tuning.read_bytes())
    line_us = (1_000_000.0 / args.fps) / args.vts

    print(f"{args.fps:g} fps, {args.vts} lines per frame, line time {line_us:.3f} us")
    print(f"full-frame exposure {args.vts * line_us / 1000:.3f} ms, "
          f"mains {args.mains:g} Hz, ripple period {1000 / (2 * args.mains):.3f} ms\n")

    safe = safe_lines(line_us, args.mains, args.vts)
    print(f"flicker-safe line counts at or below one frame: {safe}")

    ceiling = band_depth(args.vts, line_us, args.mains)
    print(f"band depth at the {args.vts}-line ceiling, which is where AE sits in dim light: "
          f"{ceiling * 100:.1f}%\n")

    banding = [i for i, (_g, lines) in enumerate(table)
               if band_depth(lines, line_us, args.mains) * 100 > args.threshold]
    print(f"{len(banding)} of {len(table)} table entries band above {args.threshold:g}%")

    print(f"\n{'index':>7} {'lines':>6} {'exp ms':>8} {'gain':>7} {'depth':>7}   "
          f"{'safe lines':>10} {'gain needed':>12}")

    step = max(1, len(table) // 12)

    for i in list(range(0, len(table), step)) + [len(table) - 1]:
        gain, lines = table[i]
        depth = band_depth(lines, line_us, args.mains)
        below = [n for n in safe if n <= lines] or [0]
        target = below[-1]
        want = gain / 256 * lines / target if target else float("inf")
        note = "" if want <= max(g for g, _l in table) / 256 else "  beyond sensor gain"
        print(f"{i:7d} {lines:6d} {lines * line_us / 1000:8.2f} {gain / 256:7.2f} "
              f"{depth * 100:6.1f}% {target:10d} {want:12.2f}{note}")

    if ceiling * 100 > args.threshold:
        target = [n for n in safe if n <= args.vts][-1]
        cost = math.log2(args.vts / target)
        print(f"\nCost of correcting the ceiling: drop {args.vts} lines to {target}, "
              f"which is {cost:.2f} stops that gain has to recover. At the darkest table entry "
              f"the sensor is already at its gain ceiling, so those stops are lost rather than "
              f"recovered.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
