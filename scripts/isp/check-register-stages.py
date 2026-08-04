#!/usr/bin/env python3
"""
Classify the register-file ISP stages and check them against the driver replay.

A stage in this class has no coefficient page, so the question is not what its
bytes are but whether its registers hold one value for the whole stream or get
recomputed. Registers the vendor rewrites with a new value during streaming are
not configuration and cannot be replayed from a static image; registers it
writes once are, and those are the ones ar-isp-defaults.h has to cover.

A count of distinct values does not separate the two on its own, because a
register that moves while auto-exposure converges and then holds looks the same
as one that moves throughout. The position of the last change decides it, so it
is reported as a percentage of the trace: an early figure means the stage
tracked a value that settled, a late one means it is still being driven.

Three things are reported per stage. Against the trace: how many of the bank's
registers are constant, how many vary, and where the last of those changes
falls. Against the header: how many of the constant ones the driver's replay
reaches, and whether it reaches them with the vendor's value.

Stages on the CVISP block are classified the same way, from --wide, but their
registers are outside ar-isp-defaults.h so only the trace half applies. Note
that mmio-combined.log carries no CVISP writes at all; wide-sweep.log is the
only trace that does.

The traces and the header's source library are proprietary. The header is in
the repository, so --header alone still checks coverage.

    kernel/scripts/isp/check-register-stages.py \\
        --trace out/au-mmiotrace/mmio-isp.log \\
        --wide out/au-mmiotrace/wide-sweep.log
"""

import argparse
import pathlib
import re
import sys
from collections.abc import Sequence

ISP_BASE = 0x08C00000
CVISP_BASE = 0x08E00000
DEFAULTS = "overlay/drivers/media/artosyn/ar-isp-defaults.h"

# Bank spans from plans/au-isp-module-inventory.md. The end is the next mapped
# bank on the same block, which is where the trace's contiguous span for this
# bank stops. CVISP stages are marked so they are read from the wide trace and
# excluded from the replay check.
ISP, CVISP = "isp", "cvisp"

STAGES = (
    ("cfa", ISP, 0x800, 0x8C0),
    ("dpc", ISP, 0xC00, 0x1800),
    ("rnr", ISP, 0x1800, 0x1C00),
    ("de3d", ISP, 0x2E00, 0x2F40),
    ("rgb2yuv", ISP, 0x3C00, 0x3C30),
    ("cnf", ISP, 0x3C64, 0x3CC8),
    ("lnr", ISP, 0x3CC8, 0x3E20),
    ("raw_3dnr", ISP, 0x4000, 0x4800),
    ("cm2", ISP, 0x4800, 0x4834),
    ("cm", ISP, 0x4834, 0x4C00),
    ("wb", ISP, 0x5000, 0x5400),
    ("qgg", ISP, 0x7230, 0x7264),
    ("lms", ISP, 0x7264, 0x7278),
    ("acm", ISP, 0x7600, 0x7800),
    ("tg", CVISP, 0x4000, 0x4200),
    ("blc", CVISP, 0x4200, 0x4400),
    ("digigain1", CVISP, 0x4600, 0x4700),
    ("digigain2", CVISP, 0x4700, 0x4800),
)

# The tables the driver applies, in the order it applies them.
APPLIED = ("ar_isp_recovered", "ar_isp_setup_1080p60", "ar_isp_vendor_trim",
           "ar_isp_output_fix")

# ar_isp_vendor_trim is measured from a live read-back rather than derived from
# what the vendor wrote, so where it disagrees with the trace it is targeting a
# register that does not read back what was written. That is intentional and is
# reported apart from a real replay gap.
TRIM = "ar_isp_vendor_trim"

TABLE_RE = re.compile(r"^static const struct ar_isp_reg (\w+)\[\]")
ENTRY_RE = re.compile(r"\{\s*(0x[0-9a-f]+),\s*(0x[0-9a-f]+)\s*\}")


def read_header(path: str) -> dict[str, dict[int, int]]:
    tables, cur = {}, None
    with open(path) as handle:
        for line in handle:
            m = TABLE_RE.match(line)
            if m:
                cur = m.group(1)
                tables[cur] = {}
                continue
            m = ENTRY_RE.search(line)
            if m and cur:
                tables[cur][int(m.group(1), 16)] = int(m.group(2), 16)
    missing = [t for t in APPLIED if t not in tables]
    if missing:
        sys.exit(f"{path}: no table named {', '.join(missing)}")
    return tables


def read_trace(path: str,
               base: int) -> tuple[dict[int, list[tuple[int, int]]], int]:
    """Values per register offset, each paired with its index in the trace."""
    seq, index = {}, 0
    pat = re.compile(r"pa=0x([0-9a-f]+) val=0x([0-9a-f]+)")
    with open(path) as handle:
        for line in handle:
            m = pat.search(line)
            if not m:
                continue
            index += 1
            off = int(m.group(1), 16) - base
            if 0 <= off < 0x10000:
                seq.setdefault(off, []).append((index, int(m.group(2), 16)))
    return seq, index


def last_change(writes: Sequence[tuple[int, int]]) -> int:
    """Trace index of the last write that changed the register's value."""
    return max((writes[k][0] for k in range(1, len(writes))
                if writes[k][1] != writes[k - 1][1]), default=0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--header", help="ar-isp-defaults.h")
    ap.add_argument("--trace", help="mmio-isp.log")
    ap.add_argument("--wide", help="wide-sweep.log, the only trace with CVISP")
    args = ap.parse_args()

    # The default is resolved against the kernel tree rather than the working
    # directory, so the script runs from the repo root as well as from kernel/.
    # This file lives in kernel/scripts/isp/, so the tree root is three levels up.
    header = args.header or str(pathlib.Path(__file__).resolve().parents[2]
                                / DEFAULTS)
    tables = read_header(header)
    applied = {}
    for name in APPLIED:
        applied.update(tables[name])

    if not args.trace:
        for name, block, lo, hi in STAGES:
            if block != ISP:
                continue
            n = sum(1 for o in applied if lo <= o < hi)
            print(f"{name:9s} {lo:#06x}..{hi:#06x}  "
                  f"replay covers {n:3d} registers")
        return 0

    seq, total = {}, {}
    seq[ISP], total[ISP] = read_trace(args.trace, ISP_BASE)
    if args.wide:
        seq[CVISP], total[CVISP] = read_trace(args.wide, CVISP_BASE)

    fail = []
    for name, block, lo, hi in STAGES:
        if block not in seq:
            continue

        regs = sorted(o for o in seq[block] if lo <= o < hi)
        if not regs:
            print(f"{name:9s} {block:5s} {lo:#06x}..{hi:#06x}  not configured")
            continue

        values = {o: [v for _, v in seq[block][o]] for o in regs}
        const = [o for o in regs if len(set(values[o])) == 1]
        vary = [o for o in regs if len(set(values[o])) > 1]
        spread = max((len(set(values[o])) for o in vary), default=1)
        settle = max((last_change(seq[block][o]) for o in vary), default=0)

        print(f"{name:9s} {block:5s} {lo:#06x}..{hi:#06x}  "
              f"traced {len(regs):3d}  constant {len(const):3d}  "
              f"recomputed {len(vary):3d}  widest {spread:2d} values  "
              f"settled by {100 * settle / total[block]:5.1f}%")

        # Only the ISP block is covered by ar-isp-defaults.h; the CVISP banks
        # are programmed by their own stages, so a replay gap there is expected
        # rather than a defect.
        if block != ISP:
            continue

        gap = [o for o in const if o not in applied]
        wrong = [o for o in const
                 if o in applied and applied[o] != values[o][-1]
                 and o not in tables[TRIM]]
        trimmed = [o for o in const
                   if o in tables[TRIM] and tables[TRIM][o] != values[o][-1]]

        print(f"      replay: {len(gap)} of the constant registers unreached, "
              f"{len(wrong)} reached with a different value")
        if gap:
            print("        unreached: " + " ".join(f"{o:#x}" for o in gap))
        if wrong:
            print("        differs:   " + " ".join(
                f"{o:#x} trace {values[o][-1]:#x} replay {applied[o]:#x}"
                for o in wrong))
        if trimmed:
            print("        read-back overrides: " + " ".join(
                f"{o:#x} written {values[o][-1]:#x} "
                f"trimmed to {tables[TRIM][o]:#x}"
                for o in trimmed))
        # A constant register the replay misses or contradicts is a real defect;
        # a recomputed one is out of the replay's reach by construction.
        fail += [(name, o) for o in gap + wrong]

    if fail:
        print(f"\n{len(fail)} constant registers are not replayed correctly")
        return 1

    print("\nevery constant register in these stages is replayed correctly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
