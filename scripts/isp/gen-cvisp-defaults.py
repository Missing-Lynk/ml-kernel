#!/usr/bin/env python3
"""
Extract the CVISP configuration from a wide MMIO write trace of the streaming vendor
and emit it as a kernel header.

CVISP is the block at 0x08e00000, ISP base + 0x200000, immediately past the ISP's 2 MiB
reg range. It is absent from the vendor device tree; the name comes from the cvisp_* stack
in the vendor's unstripped libmpp_service.so. It is NOT the DTS scaler@08840000 or
gdc@08848000. In the vendor's design CVISP, not the ISP, writes frames to DRAM.

Four tables come out, because the vendor drives the block at different cadences:

  setup    once, ending at the output enable 0x8000 = 0x00800806
  late     once, a tail of registers the vendor writes just after the enable
  ring     five buffer sets, one Y/U/V triplet per frame, round robin
  tick     eight registers, written once per ring wrap rather than once per frame

The setup boundary is the enable write, not the first per-frame VIF acknowledge that the ISP
tables use: the VIF acknowledge lands several frames later, so cutting there pulls ring
rotations into the setup table. The late tail lands with the first frames already in flight;
whether that ordering is required is not established, so the two tables stay separate.

Usage:
    kernel/scripts/isp/gen-cvisp-defaults.py --trace out/au-mmiotrace/wide-sweep.log \\
        > kernel/overlay/drivers/media/artosyn/vendor-tables/ar-cvisp-defaults.h
"""

import argparse
import collections
import sys
from collections.abc import Sequence

import arlib

BLK_BASE = 0x08E00000
BLK_SIZE = 0x10000

# Output enable. The vendor stages this register 0x00800800 -> 0x00800802 -> 0x00800806 and
# then never writes it again, so the final value marks the end of setup.
CVISP_CONTROL = 0x8000
CVISP_ENABLE_DONE = 0x00800806

# Output plane bases, Y/U/V, rewritten in lockstep once per frame.
CVISP_PLANE = (0x8098, 0x8174, 0x8194)

# A trace line is "wNNNNNN wWW pa=0xADDR val=0xVAL"; anything shorter is not a write record.
TRACE_FIELDS = 4

# Steady-state write count above which a register counts as part of the wrap-cadence tick
# group. The capture runs thousands of frames, so a tick register (one write per five
# frames) is orders of magnitude above this, while the handful of registers the vendor
# rewrites a few times aperiodically are far below it. Anything under the threshold is
# reported but not tabled.
TICK_MIN_WRITES = 100

# Registers named in a reconstruction-mismatch warning before it is truncated.
WARN_MAX_REGS = 8


def load_writes(path: str) -> list[tuple[int, int]]:
    """Ordered (phys, value) writes from the trace."""
    writes = []
    with open(path) as trace:
        for line in trace:
            fields = line.split()
            if len(fields) < TRACE_FIELDS or fields[0][0] != 'w':
                continue

            phys = int(fields[2].split('=')[1], 16)
            val = int(fields[3].split('=')[1], 16)
            writes.append((phys, val))

    return writes


def block_writes(writes: Sequence[tuple[int, int]]) -> list[tuple[int, int]]:
    """The trace's writes to this block, rebased to block-relative offsets."""
    return [(phys - BLK_BASE, val) for phys, val in writes
            if BLK_BASE <= phys < BLK_BASE + BLK_SIZE]


def split_at_enable(writes: Sequence[tuple[int, int]]) -> int:
    """Index just past the final output-enable write."""
    for i in range(len(writes) - 1, -1, -1):
        if writes[i] == (CVISP_CONTROL, CVISP_ENABLE_DONE):
            return i + 1

    sys.exit(f"no output-enable write (0x{CVISP_CONTROL:04x} = "
             f"0x{CVISP_ENABLE_DONE:08x}) in the trace")


def collapse(writes: Sequence[tuple[int, int]]) -> list[tuple[int, int]]:
    """
    Drop consecutive duplicate writes to the same register.

    Order is kept: the block has a staged enable whose result depends on the sequence,
    exactly as the ISP does. Only runs of the identical write to the identical register
    are removed.
    """
    collapsed = []
    for off, val in writes:
        if collapsed and collapsed[-1] == (off, val):
            continue

        collapsed.append((off, val))

    return collapsed


def extract_ring(steady: Sequence[tuple[int, int]]) -> list[tuple[int, ...]]:
    """The distinct Y/U/V buffer sets, in the order the vendor first queues them."""
    sets, seen = [], set()
    pending = {}
    for off, val in steady:
        if off not in CVISP_PLANE:
            continue

        pending[off] = val
        if len(pending) == len(CVISP_PLANE):
            triplet = tuple(pending[plane] for plane in CVISP_PLANE)
            if triplet not in seen:
                seen.add(triplet)
                sets.append(triplet)

            pending = {}

    return sets


def emit(name: str, rows: Sequence[tuple[int, int]], comment: str) -> None:
    print(f"\n/*\n{comment}\n */")
    print(f"static const struct ar_cvisp_reg {name}[] = {{")
    for off, val in rows:
        print(f"\t{{ 0x{off:04x}, 0x{val:08x} }},")

    print("};")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--trace', required=True,
                    help='wide MMIO write trace covering 0x08e00000')
    args = ap.parse_args()

    writes = block_writes(load_writes(args.trace))
    if not writes:
        sys.exit(f"{args.trace}: no writes to 0x{BLK_BASE:08x}; "
                 "was the trace window wide enough?")

    enable_end = split_at_enable(writes)
    setup = collapse(writes[:enable_end])
    steady = writes[enable_end:]

    ring = extract_ring(steady)

    write_count = collections.Counter(off for off, _ in steady)

    # The late tail: registers the vendor writes exactly once after the enable. They form a
    # contiguous run at the head of the steady state, interleaved with the first frames'
    # plane and tick writes, which is why they are selected by write count rather than by a
    # position cut.
    late = [(off, val) for off, val in steady if write_count[off] == 1]

    # The tick group: the wrap-cadence registers, one write per five frames. The threshold
    # separates them from a handful of registers the vendor rewrites only a few times
    # across the whole capture, which are reported but not tabled.
    tick_val, aperiodic = {}, {}
    for off, val in steady:
        if off in CVISP_PLANE or write_count[off] == 1:
            continue

        target = tick_val if write_count[off] > TICK_MIN_WRITES else aperiodic
        target.setdefault(off, val)

    tick = sorted(tick_val.items())

    # Self-check: replaying the setup table in order must reproduce the vendor's final
    # setup value for every register it touches.
    final = {}
    for off, val in writes[:enable_end]:
        final[off] = val

    replay = {}
    for off, val in setup:
        replay[off] = val

    mismatched = [off for off in final if replay.get(off) != final[off]]

    guard_open, guard_close = arlib.guard("AR_CVISP_DEFAULTS_H")
    print(arlib.banner("kernel/scripts/isp/gen-cvisp-defaults.py", (
        "CVISP register configuration, recovered from a wide MMIO write trace of the",
        "streaming vendor. CVISP is the block at 0x08e00000; the name is from the",
        "cvisp_* stack in the vendor's unstripped libmpp_service.so. It is absent from",
        "the vendor device tree.",
        "",
        "In the vendor's design this block, not the ISP, writes frames to DRAM. The",
        "ISP feeds it and CVISP owns the output queue.",
        "",
        "Geometry registers carry both 0x04380780 (1080 x 1920) and 0x021c03c0",
        "(540 x 960); 0x8008 is written with the smaller value during setup and ends at",
        "the larger one, so which stage is scaled is NOT established here. 0x021c03c0 is",
        "also what ISP 0x7080 reads on the streaming vendor.",
        "",
        "The trace was captured writes-only, so the absence of reads below says nothing",
        "about whether the vendor polls this block.",
    )), end="")
    print()
    print(guard_open, end="")
    print()
    print("struct ar_cvisp_reg {")
    print("\tu16 off;")
    print("\tu32 val;")
    print("};")
    print()
    print("struct ar_cvisp_bufset {")
    print("\tu32 y;")
    print("\tu32 u;")
    print("\tu32 v;")
    print("};")

    emit('ar_cvisp_setup', setup,
         " * Setup, in write order with consecutive duplicates collapsed, ending at the\n"
         " * output enable. The last entries stage 0x8000 through 0x00800800, 0x00800802,\n"
         " * 0x00800806; bits 1 and 2 are undecoded. The plane bases left behind are ring\n"
         " * set 0, so applying this table alone arms the first frame.")

    emit('ar_cvisp_late', late,
         " * The tail written once just after the enable, in order: the arbitration table\n"
         " * on page 0x0000, per-channel geometry and limits on page 0x4000, then the\n"
         " * upper halves of the two tick banks. Page 0x4000 carries 0x780 x 0x438\n"
         " * (1920 x 1080), so this stage is not the scaled one. Lands with frames already\n"
         " * in flight; whether the ordering matters is not established.")

    print("\n/*\n * The output queue: five Y/U/V buffer sets, one triplet written per frame in\n"
          " * round robin. Vendor DRAM addresses; a driver allocating its own buffers\n"
          " * replaces these and must keep the relative plane spacing.\n */")
    print("static const struct ar_cvisp_bufset ar_cvisp_ring[] = {")

    for y, u, v in ring:
        print(f"\t{{ 0x{y:08x}, 0x{u:08x}, 0x{v:08x} }},")

    print("};")

    emit('ar_cvisp_tick', tick,
         " * Written once per ring wrap, after every fifth triplet: two banks of four\n"
         " * registers, all taking 0x00000100. Purpose undecoded.")

    print(f"\n/* setup: {len(setup)} ordered writes over {len(final)} registers", end='')
    print(", reconstruction exact" if not mismatched
          else f", {len(mismatched)} reconstruction mismatches", end='')
    print(" */")
    print(f"/* late: {len(late)} registers */")
    print(f"/* ring: {len(ring)} buffer sets */")

    if tick:
        wraps = write_count[tick[0][0]]
        print(f"/* tick: {len(tick)} registers, {wraps} wraps over "
              f"{write_count[CVISP_PLANE[0]]} frames */")

    if aperiodic:
        print("/* not tabled, rewritten a few times aperiodically: "
              + ", ".join(f"0x{off:04x}={val:#010x} ({write_count[off]}x)"
                          for off, val in sorted(aperiodic.items())) + " */")

    print()
    print(guard_close, end="")

    print(f"setup: {len(setup)} writes over {len(final)} registers", file=sys.stderr)
    print(f"late: {len(late)} registers", file=sys.stderr)
    print(f"ring: {len(ring)} buffer sets", file=sys.stderr)
    print(f"tick: {len(tick)} registers", file=sys.stderr)

    if aperiodic:
        print("aperiodic, not tabled: "
              + ", ".join(f"0x{off:04x}" for off in sorted(aperiodic)), file=sys.stderr)

    if mismatched:
        print(f"WARNING: {len(mismatched)} registers do not reconstruct: "
              + ", ".join(f"0x{off:04x}" for off in mismatched[:WARN_MAX_REGS]),
              file=sys.stderr)
    else:
        print("reconstruction exact", file=sys.stderr)


if __name__ == '__main__':
    main()
