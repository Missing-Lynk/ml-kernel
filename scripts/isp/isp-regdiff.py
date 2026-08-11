#!/usr/bin/env python3
"""
Diff two ISP register sweeps and classify every difference.

The sweeps in out/au-snapshot are a sequence of windows, each introduced by a
"--- <block> +0x<base> (<n> words) ---" header followed by rows whose offset is
relative to that window, not to the block. Reading a row offset as absolute puts
every window after the first at the wrong address; the lnr curve region at
0x3d80 reads as 0x0f80 and lands in dpc. The window base is added here.

Differences are sorted into four classes against the driver's own tables in
ar-isp-defaults.h:

  A  prefix-reachable  the vendor's live value is already in the setup table
                       past the applied prefix, or in vendor_trim. Extending
                       the prefix or the trim closes these with no RE.
  B  runtime-adapted   our value is what the tables specify and the vendor's
                       live value has moved since. AE or AWB drove it; no
                       static replay can reach it.
  C  address           a DMA address in our carveout against theirs. Benign.
  D  unexplained       in neither class. Either a register the tables never
                       carry, or one the vendor moved away from a value we do
                       not hold either.

Usage: isp-regdiff.py <vendor-sweep> <our-sweep> [--all]
"""
import re
import sys
from collections import Counter

HDR = re.compile(r"^--- (\w+) \+0x([0-9a-f]+) \((\d+) words\) ---")
ROW = re.compile(r"^\+0x([0-9a-f]+): (.*)$")
ENT = re.compile(r"^\s*\{\s*0x([0-9a-f]+),\s*0x([0-9a-f]+)\s*\},")
ARR = re.compile(r"^static const struct ar_isp_reg (\w+)\[\]")

DEFAULTS = "overlay/drivers/media/artosyn/vendor-tables/ar-isp-defaults.h"
PREFIX = 1475

"""Bank bases, from each module's attach handler (the ar_dev_pa2va offset), not
from symbol adjacency. Searched top down, so every gap belongs to the bank
below it.

An earlier version of this table put af_stats at 0x6c00, which is awbs_stats,
and carried no entry between lms 0x7264 and acm 0x7600. Everything from 0x7264
to 0x75fc therefore reported as lms, including the whole 0x7400 af_stats bank.
lms is 20 registers wide and measured off, so a large block of statistics
registers was being read as a gain-keyed image stage.

The same gap existed between awbs_stats 0x6c00 and nuc_dpc 0x7200. The ISP
open path maps base+0x7000 as its own page, at 0x1ca260 in the vendor library,
and awbs_stats' image is 105 registers ending at 0x6da0, so everything at
0x70xx was reporting as awbs_stats. That mattered more than a mislabel:
audit-provenance.py excuses awbs_stats as a disabled stage, so ten registers
on a page that does run were being excused with it.
"""
BANKS = [
    (0x8600, "ir_lms_horz"),
    (0x8400, "ir_rnr"),
    (0x7600, "acm"),
    (0x7400, "af_stats"),
    (0x7278, "ir_gtm"),
    (0x7264, "lms"),
    (0x7230, "qgg"),
    (0x7200, "nuc_dpc"),
    (0x7000, "isp_input"),
    (0x6C00, "awbs_stats"),
    (0x64C8, "rro_face_stats"),
    (0x6400, "rro_stats"),
    (0x6000, "raw_hist_stats"),
    (0x5C00, "rgb_hist_stats"),
    (0x5800, "lut3d"),
    (0x5400, "rgb_max_stats"),
    (0x5000, "wb"),
    (0x4C00, "lsc"),
    (0x4834, "cm"),
    (0x4800, "cm2"),
    (0x4000, "raw_3dnr"),
    (0x3CC8, "lnr"),
    (0x3C64, "cnf"),
    (0x3C30, "binning"),
    (0x3C00, "rgb2yuv"),
    (0x3900, "derolling_stats"),
    (0x3800, "ccm2"),
    (0x3400, "ccm1"),
    (0x3000, "drc"),
    (0x2E00, "de3d"),
    (0x2870, "ltm_v1"),
    (0x2800, "ltm"),
    (0x2400, "gib"),
    (0x1FFC, "hdr_lsc"),
    (0x1F98, "hdr"),
    (0x1F40, "hdr_rro_face_stats"),
    (0x1E44, "hdr_awbs_stats"),
    (0x1DD0, "hdr_lsc"),
    (0x1D78, "hdr_rro_1_stats"),
    (0x1D20, "hdr_rro_0_stats"),
    (0x1C00, "hdr"),
    (0x1800, "rnr"),
    (0x0C00, "dpc"),
    (0x0800, "cfa"),
    (0x0000, "base"),
]

# Statistics banks are read-only accumulators and thresholds: they cannot alter
# the image, so a difference in one is never a defect. Flagged rather than
# dropped, because they still say something about 3A state.
STATS = {
    "af_stats", "awbs_stats", "rro_stats", "rro_face_stats", "raw_hist_stats",
    "rgb_hist_stats", "rgb_max_stats", "derolling_stats", "hdr_awbs_stats",
    "hdr_rro_0_stats", "hdr_rro_1_stats", "hdr_rro_face_stats",
}


def bank(off: int) -> str:
    for base, name in BANKS:
        if off >= base:
            return name

    return "?"


def load_sweep(path: str) -> dict[tuple[str, int], int]:
    out, blk, wbase = {}, None, 0
    with open(path) as sweep:
        for line in sweep:
            m = HDR.match(line)
            if m:
                blk, wbase = m.group(1), int(m.group(2), 16)
                continue

            m = ROW.match(line)
            if m and blk:
                base = wbase + int(m.group(1), 16)
                for i, word in enumerate(m.group(2).split()):
                    out[(blk, base + i * 4)] = int(word, 16)

    return out


def load_tables(path: str) -> dict[str, list[tuple[int, int]]]:
    arrays, cur = {}, None
    with open(path) as header:
        for line in header:
            m = ARR.match(line)
            if m:
                cur = m.group(1)
                arrays[cur] = []
                continue

            m = ENT.match(line)
            if m and cur:
                arrays[cur].append((int(m.group(1), 16), int(m.group(2), 16)))

    return arrays


def classify(off: int, vendor: int, ours: int, tail: dict[int, int],
             trim: dict[int, int]) -> str:
    if (vendor >> 24) in (0x2A, 0x2B):
        return "C"

    if tail.get(off) == vendor or trim.get(off) == vendor:
        return "A"

    if tail.get(off) == ours or trim.get(off) == ours:
        return "B"

    return "D"


def main() -> int:
    show_all = "--all" in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) != 2:
        # The docstring opens on the line after its quotes, so the usage banner would
        # start with a blank line.
        print(__doc__.lstrip("\n"))
        return 1

    vend, ours = load_sweep(args[0]), load_sweep(args[1])
    try:
        arrays = load_tables(DEFAULTS)
    except OSError:
        print(f"run from the kernel tree: {DEFAULTS} not found")
        return 1

    tail = dict(arrays["ar_isp_setup_1080p60"][PREFIX:])
    trim = dict(arrays["ar_isp_vendor_trim"])

    keys = sorted(set(vend) | set(ours))
    diff = [(k, vend.get(k), ours.get(k)) for k in keys
            if vend.get(k) != ours.get(k)]
    print(f"{len(vend)} words vendor, {len(ours)} words ours, {len(diff)} differ")
    print("  " + " ".join(f"{b}:{n}" for b, n in
                          sorted(Counter(k[0] for k, _, _ in diff).items())))

    groups = {"A": [], "B": [], "C": [], "D": []}
    for (blk, off), va, vb in diff:
        if blk != "isp":
            continue

        groups[classify(off, va, vb, tail, trim)].append((off, va, vb))

    for name in "ABCD":
        rows = groups[name]
        counts = Counter(bank(o) for o, _, _ in rows)
        nstats = sum(n for b, n in counts.items() if b in STATS)
        print(f"\n### class {name}: {len(rows)} isp registers"
              f" ({nstats} in statistics banks, image-inert)")
        print("    " + " ".join(f"{b}:{n}" for b, n in sorted(counts.items())))
        if name in ("A", "D") or show_all:
            for off, va, vb in rows:
                b = bank(off)
                tag = "  stats" if b in STATS else ""
                print(f"    0x{off:04x} {b:19s} "
                      f"vendor {va:08x}  ours {vb:08x}{tag}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
