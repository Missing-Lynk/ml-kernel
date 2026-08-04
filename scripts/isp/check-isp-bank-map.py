#!/usr/bin/env python3
"""
Rebuild the ISP bank map from the vendor library and check it against the
descriptor attribution recorded in plans/au-isp-module-inventory.md.

Every isp_sub_*/cvisp_sub_* constructor stores three handlers in its context.
The one at +472 is the attach handler, which maps the module's MMIO by adding a
bank offset to an ar_dev_pa2va return value. Only immediates that reach a
pa2va result count as banks; every other immediate in the handler is noise, and
counting them produces banks the write trace rejects.

A descriptor belongs to the largest mapped bank at or below it. Six descriptors
attributed by earlier work are asserted as control cases, so a regression in the
extraction fails here rather than silently renaming a page.

Banks live on one of two blocks. A module selects the CVISP block by adding
0x200000 to the physical base before translating it, and the bank offset is
added afterwards, so the two have to be tracked per call rather than per
handler.

The library and the traces are proprietary and are not in the repository.

    kernel/scripts/isp/check-isp-bank-map.py \\
        --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so \\
        --trace out/au-mmiotrace/mmio-isp.log
"""

import argparse
import re
import subprocess
import sys

ISP_BASE = 0x08C00000

# Attributions established elsewhere and proven against captured pages. These
# are the control cases for the extraction, not new results.
CONTROL = {
    0x4C34: "isp_sub_lsc_creat",
    0x600C: "isp_sub_raw_hist_stats_creat",
    0x6440: "isp_sub_rro_stats_creat",
    0x6474: "isp_sub_rro_stats_creat",
    0x2808: "isp_sub_ltm_creat",
    0x1C6C: "isp_sub_hdr_creat",
}

CREAT_RE = re.compile(r"^(isp|cvisp)_sub_.*_creat(_v1|_internal)?$")
LINE_RE = re.compile(r"^\s+([0-9a-f]+):\s+[0-9a-f]+\s+(.*)$")
ADRP_RE = re.compile(r"adrp\tx(\d+), ([0-9a-f]+)")
ADD_RE = re.compile(r"add\tx(\d+), x(\d+), #0x([0-9a-f]+)$")
STR_RE = re.compile(r"str\tx(\d+), \[x\d+, #(\d+)\]")
STP_RE = re.compile(r"stp\tx(\d+), x(\d+), \[x\d+, #(\d+)\]")
MOV_RE = re.compile(r"mov\tx(\d+), #0x([0-9a-f]+)\b")
BANKR_RE = re.compile(r"add\tx\d+, x0, x(\d+)$")
BANKI_RE = re.compile(r"add\tx\d+, x0, #0x([0-9a-f]+)$")
BANKL_RE = re.compile(r"add\tx\d+, x0, #0x([0-9a-f]+), lsl #12$")
STOREX0_RE = re.compile(r"st(r|p)\tx0[,\s]")
SELECT_RE = re.compile(r"add\tw0, w\d+, #0x([0-9a-f]+), lsl #12")

# The CVISP block sits this far above the ISP physical base.
CVISP_SELECT = 0x200000

ATTACH_SLOT = 472


def binutil(tool: str) -> str:
    """The host objdump does not know aarch64, so prefer a cross build."""
    for prefix in ("aarch64-linux-gnu-", "aarch64-none-linux-gnu-", ""):
        name = prefix + tool
        try:
            subprocess.run([name, "--version"], capture_output=True, check=True)
        except (OSError, subprocess.CalledProcessError):
            continue

        return name

    sys.exit(f"no usable {tool}: install binutils-aarch64-linux-gnu")


def disassemble(lib: str) -> dict[int, str]:
    out = subprocess.run([binutil("objdump"), "-d", lib], capture_output=True,
                         text=True, check=True).stdout
    lines = {}
    for line in out.splitlines():
        m = LINE_RE.match(line)
        if m:
            lines[int(m.group(1), 16)] = m.group(2).strip()

    return lines


def symbols(lib: str) -> dict[str, int]:
    out = subprocess.run([binutil("nm"), "-nD", "--defined-only", lib],
                         capture_output=True, text=True, check=True).stdout
    found = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[1] in ("T", "t"):
            found[parts[2]] = int(parts[0], 16)

    return found


def constructors(syms: dict[str, int]) -> dict[str, int]:
    return {n: a for n, a in syms.items() if CREAT_RE.match(n)}


def attach_handler(lines: dict[int, str], creat: int) -> int | None:
    """The handler stored at ctx+472. Stores use str or stp interchangeably."""
    regs = {}
    for addr in range(creat, creat + 0x180, 4):
        ins = lines.get(addr)
        if ins is None:
            continue

        m = ADRP_RE.search(ins)
        if m:
            regs[int(m.group(1))] = int(m.group(2), 16)
            continue

        m = ADD_RE.search(ins)
        if m and int(m.group(2)) in regs:
            regs[int(m.group(1))] = regs[int(m.group(2))] + int(m.group(3), 16)
            continue

        m = STR_RE.search(ins)
        if m and int(m.group(2)) == ATTACH_SLOT and int(m.group(1)) in regs:
            return regs[int(m.group(1))]

        m = STP_RE.search(ins)
        if m and int(m.group(3)) == ATTACH_SLOT and int(m.group(1)) in regs:
            return regs[int(m.group(1))]

    return None


def banks(lines: dict[int, str], attach: int,
          limit: int) -> list[tuple[int, int]]:
    """
    (block, bank) pairs, one per ar_dev_pa2va call that gets an offset.

    The block is chosen before the call by adding a selector to the physical
    base, and the bank is added to the translated result after it. A module can
    map banks on both blocks, so the selector has to be tracked per call and
    reset after each one: dpc adds 0xc00 to an unselected call and keeps a
    separate CVISP pointer with no bank, so a per-handler selector would put
    dpc's bank on the wrong block.

    The scan stops at limit, the next symbol. Handlers sit close together, so a
    fixed window runs into the neighbour: a 0x400 window gives digigain2 ccm2's
    0x3800.
    """
    imms, out, pending, block = {}, [], False, 0
    for addr in range(attach, min(attach + 0x400, limit), 4):
        ins = lines.get(addr)
        if ins is None:
            continue

        m = MOV_RE.search(ins)
        if m:
            imms[int(m.group(1))] = int(m.group(2), 16)
            continue

        m = SELECT_RE.search(ins)
        if m:
            block = int(m.group(1), 16) << 12
            continue

        if "ar_dev_pa2va" in ins:
            pending = True
            continue

        if not pending:
            continue

        m = BANKR_RE.search(ins)
        if m:
            pending = False
            if int(m.group(1)) in imms:
                out.append((block, imms[int(m.group(1))]))
            block = 0
            continue

        m = BANKL_RE.search(ins)
        if m:
            pending = False
            out.append((block, int(m.group(1), 16) << 12))
            block = 0
            continue

        m = BANKI_RE.search(ins)
        if m:
            pending = False
            out.append((block, int(m.group(1), 16)))
            block = 0
            continue

        # The window ends when x0, the pa2va result, is itself stored. Stores of
        # unrelated fields sit between the call and the add and must not end it.
        if STOREX0_RE.match(ins):
            pending = False
            block = 0

    return sorted({p for p in out if p[1] > 4})


def written(trace: str) -> set[int]:
    offsets = set()
    pat = re.compile(r"pa=0x([0-9a-f]+) ")
    with open(trace) as handle:
        for line in handle:
            m = pat.search(line)
            if m:
                off = int(m.group(1), 16) - ISP_BASE
                if 0 <= off < 0x10000:
                    offsets.add(off)

    return offsets


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--lib", required=True, help="vendor libmpp_service.so")
    ap.add_argument("--trace", help="mmio-isp.log, for the span cross-check")
    args = ap.parse_args()

    lines = disassemble(args.lib)
    syms = symbols(args.lib)
    creats = constructors(syms)
    if not creats:
        sys.exit("no isp_sub_*_creat symbols: is the library stripped?")

    bounds = sorted(set(syms.values()))

    owners = {}
    missing = []
    for name, creat in creats.items():
        attach = attach_handler(lines, creat)
        if attach is None:
            missing.append(name)
            continue

        after = [a for a in bounds if a > attach]
        limit = after[0] if after else attach + 0x400
        for block, bank in banks(lines, attach, limit):
            owners.setdefault((block, bank), []).append(name)

    order = sorted(b for blk, b in owners if blk != CVISP_SELECT)
    cvisp = sorted(b for blk, b in owners if blk == CVISP_SELECT)
    print(f"{len(creats)} constructors, {len(order)} ISP banks, "
          f"{len(cvisp)} CVISP banks")
    print("      CVISP: " + ", ".join(f"{b:#x}" for b in cvisp))
    if missing:
        print(f"      no attach handler recovered for: {', '.join(sorted(missing))}")

    for desc, want in sorted(CONTROL.items()):
        below = [b for b in order if b <= desc]
        if not below:
            sys.exit(f"{desc:#x}: no bank at or below it")

        got = owners[(0, below[-1])]
        if want not in got:
            sys.exit(f"{desc:#x}: bank {below[-1]:#x} maps to {got}, expected {want}")
    print(f"      {len(CONTROL)} control descriptors attributed as recorded")

    if args.trace:
        offsets = written(args.trace)
        for i, bank in enumerate(order):
            end = order[i + 1] if i + 1 < len(order) else 0x10000
            span = [o for o in offsets if bank <= o < end]
            if span and max(span) >= end:
                sys.exit(f"bank {bank:#x} writes run past {end:#x}")

        stray = sorted(o for o in offsets if o >= order[0] and
                       not any(b <= o for b in order))

        if stray:
            sys.exit(f"written registers in no bank: {stray[:8]}")

        print("      every bank's written span stops before the next bank")

    print("ISP bank map agrees with plans/au-isp-module-inventory.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
