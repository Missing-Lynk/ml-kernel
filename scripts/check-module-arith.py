#!/usr/bin/env python3
"""
Count the trigger and arithmetic machinery inside each ISP module's own code.

This answers one question per module: does the stage recompute its registers
from the tuning file, and if so off which trigger. A stage that reaches
is_aec_trigger_compute and converts between integer and float is interpolating a
gain-indexed table; one that reaches neither is static configuration.

Two things make a naive count wrong, and both produced false zeroes before.

A module's helpers are not laid out next to the constructor that registers them,
so a symbol-adjacency range is not the module. The three handlers stored at
ctx+472, +480 and +488 are the only entry points the ISP core calls, so the
module is what those reach through direct bl targets that are not themselves
exported.

Bounding a function at the next bl target anywhere in the binary is also wrong,
because a branch island inside a large handler is such an address and the scan
stops there, part way through. A function ends at a ret that lies beyond every
local branch target seen so far, which survives islands.

Following only bl targets misses the code that matters most. rnr's blend is a
callback its command handler materialises with adrp and add and hands to another
module's dispatcher along with its own context, so nothing in the binary calls
it directly and no call graph built from bl reaches it. A module's code is its
three handlers plus any function they publish that way, each bounded at the next
handler, callback or exported symbol. That is a closed definition and needs no
transitive walk, which is what makes it stable: a walk that chases calls runs
into shared subsystems and reports the same hundred thousand instructions for
every module.

A published callback almost always lies inside the span from a handler to the
next exported symbol, so recovering it moves an instruction from one extent to
another and leaves the body identical. It only changes the body when it lies
outside every handler's span, which is why recovery is driven by coverage: an
address already inside the body is not a new callback, and the count of them is
a diagnostic rather than a structural property of the module.

Detecting a callback by its prologue is the fourth trap. rgb2yuv publishes a
frameless leaf, which no stp x29 or sub sp begins, so a prologue test drops it
and undercounts the body. A function start is an address preceded by ret or by
an unconditional branch, which frameless leaves satisfy. The uncov column counts
materialised code addresses that end up in neither the body nor the callback
set, so a module whose recovery came up short is never silently classified.

Counting only scalar mnemonics is the fifth trap: rnr blends four lanes at a
time, so its conversions are scvtf v5.4s and fcvtzs v5.4s and a scalar-only
pattern reports zero float work for a module that is entirely float work.

EXPECT below encodes conclusions already drawn in the inventory, so agreement
with it is a regression guard against the traps above and not independent
confirmation of the verdicts.

The library is proprietary and is not in the repository.

    kernel/scripts/check-module-arith.py \\
        --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so
"""

import argparse
import re
import subprocess
import sys

SLOTS = (472, 480, 488)

# Stages whose classification rests on these counts, with the expected verdict.
# recompute means the module reaches a trigger and converts between integer and
# float; static means it does neither.
EXPECT = {
    "rnr": "recompute",
    "lnr": "recompute",
    "de3d": "recompute",
    "blc": "recompute",
    "cm2": "recompute",
    "cnf": "recompute",
    "acm": "recompute",
    "cfa": "recompute",
    "dpc": "static",
    "qgg": "static",
    "rgb2yuv": "static",
}

LINE_RE = re.compile(r"^\s+([0-9a-f]+):\s+[0-9a-f]+\s+(.*)$")
ADRP_RE = re.compile(r"adrp\tx(\d+), ([0-9a-f]+)")
ADD_RE = re.compile(r"add\tx(\d+), x(\d+), #0x([0-9a-f]+)$")
STR_RE = re.compile(r"str\tx(\d+), \[x\d+, #(\d+)\]")
STP_RE = re.compile(r"stp\tx(\d+), x(\d+), \[x\d+, #(\d+)\]")
BL_RE = re.compile(r"^bl\t([0-9a-f]+)")
BRANCH_RE = re.compile(r"^b(\.\w+)?\t([0-9a-f]+)")
CREAT_RE = re.compile(r"^isp_sub_(\w+)_creat$")

# Both the scalar and the vector forms of every conversion that matters.
CONVERT_RE = re.compile(r"^(fcvtz[su]|[su]cvtf)\b")
MULTIPLY_RE = re.compile(r"^(fmul|fmla|fmadd|fdiv|fsub|fadd)\b")


def binutil(tool):
    """The host objdump does not know aarch64, so prefer a cross build."""
    for prefix in ("aarch64-linux-gnu-", "aarch64-none-linux-gnu-", ""):
        name = prefix + tool
        try:
            subprocess.run([name, "--version"], capture_output=True, check=True)
        except (OSError, subprocess.CalledProcessError):
            continue

        return name

    sys.exit(f"no usable {tool}: install binutils-aarch64-linux-gnu")


def disassemble(lib):
    out = subprocess.run([binutil("objdump"), "-d", lib], capture_output=True,
                         text=True, check=True).stdout
    code = {}
    for line in out.splitlines():
        m = LINE_RE.match(line)
        if m:
            code[int(m.group(1), 16)] = m.group(2).strip()
    if not code:
        sys.exit(f"{lib}: no instructions disassembled")

    return code


def symbols(lib):
    out = subprocess.run([binutil("nm"), "-nD", "--defined-only", lib],
                         capture_output=True, text=True, check=True).stdout
    found = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[1] in ("T", "t"):
            found[parts[2]] = int(parts[0], 16)

    return found


def handlers(code, creat):
    """The three handler addresses a constructor stores, by context slot."""
    regs, out = {}, {}
    for addr in range(creat, creat + 0x180, 4):
        ins = code.get(addr)
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
        if m and int(m.group(2)) in SLOTS and int(m.group(1)) in regs:
            out[int(m.group(2))] = regs[int(m.group(1))]
            continue

        m = STP_RE.search(ins)
        if m:
            base = int(m.group(3))
            for group, slot in ((1, base), (2, base + 8)):
                if slot in SLOTS and int(m.group(group)) in regs:
                    out[slot] = regs[int(m.group(group))]

    return out


RET_RE = re.compile(r"^b\t[0-9a-f]+")


def is_start(code, exported, addr):
    """A function begins after a ret or an unconditional branch.

    Frameless leaves have no prologue to key on, so the preceding instruction is
    what identifies them.
    """
    before = code.get(addr - 4, "")

    return addr in exported or before == "ret" or bool(RET_RE.match(before))


def extents(seeds, stops):
    """The span of each seed, bounded at the next handler, callback or symbol."""
    return {s: min([a for a in stops if a > s] + [s + 0x4000]) for s in seeds}


def materialised(code, ext):
    """Every code address a module's own instructions form with adrp and add."""
    low, high = min(code), max(code)
    found = set()
    for start, limit in ext.items():
        regs = {}
        for addr in range(start, limit, 4):
            ins = code.get(addr, "")
            m = ADRP_RE.search(ins)
            if m:
                regs[int(m.group(1))] = int(m.group(2), 16)
                continue

            m = ADD_RE.search(ins)
            if m and int(m.group(2)) in regs:
                value = regs[int(m.group(2))] + int(m.group(3), 16)
                regs[int(m.group(1))] = value
                if low <= value <= high and value in code:
                    found.add(value)

    return found


def recover(code, exported, handlers_found):
    """Grow the seed set until no materialised address falls outside the body.

    An address already covered by an extent is reached anyway, so only one that
    lies outside every span is a callback that changes what the module is.
    """
    seeds, stops = list(handlers_found), sorted(exported)
    for _ in range(8):
        ext = extents(seeds, sorted(exported | set(seeds)))
        outside = {a for a in materialised(code, ext)
                   if not any(s <= a < e for s, e in ext.items())}
        fresh = {a for a in outside if is_start(code, exported, a)}
        if not fresh:
            return sorted(seeds), len(outside - fresh)

        seeds = sorted(set(seeds) | fresh)

    return sorted(seeds), 0


def module_body(code, ext):
    """Every instruction of a module's handlers and published callbacks."""
    return [code[a] for start, limit in ext.items()
            for a in range(start, limit, 4) if a in code]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--lib", required=True, help="vendor libmpp_service.so")
    ap.add_argument("--limit", type=lambda v: int(v, 0), default=0x400000,
                    help="highest address a module's code may occupy")
    args = ap.parse_args()

    code = disassemble(args.lib)
    syms = symbols(args.lib)
    exported = set(syms.values())

    modules = {}
    for name, addr in syms.items():
        m = CREAT_RE.match(name)
        if m:
            modules[m.group(1)] = addr
    if not modules:
        sys.exit("no isp_sub_*_creat symbols: is the library stripped?")

    print(f"{'module':10s} {'hnd':>4s} {'cb':>3s} {'uncov':>6s} {'ins':>7s} "
          f"{'aec':>4s} {'awb':>4s} {'tune':>5s} {'cvt':>5s} {'mul':>5s}  verdict")

    wrong, short = [], []
    for name in sorted(EXPECT):
        creat = modules.get(name)
        if creat is None:
            sys.exit(f"{name}: no constructor")

        found = sorted(handlers(code, creat).values())
        if not found:
            sys.exit(f"{name}: no handlers recovered from {creat:#x}")

        seeds, uncov = recover(code, exported, found)
        block = module_body(code, extents(seeds, sorted(exported | set(seeds))))
        if uncov:
            short.append((name, uncov))
        aec = sum("is_aec_trigger_compute" in i for i in block)
        awb = sum("is_awb_trigger_compute" in i for i in block)
        tune = sum("isp_get_tuning_manager" in i for i in block)
        cvt = sum(bool(CONVERT_RE.match(i)) for i in block)
        mul = sum(bool(MULTIPLY_RE.match(i)) for i in block)

        got = "recompute" if (aec or awb) and cvt else "static"
        if got != EXPECT[name]:
            wrong.append((name, got))

        print(f"{name:10s} {len(found):4d} {len(seeds) - len(found):3d} "
              f"{uncov:6d} {len(block):7d} {aec:4d} {awb:4d} "
              f"{tune:5d} {cvt:5d} {mul:5d}  {got}")

    if wrong:
        for name, got in wrong:
            print(f"\n{name}: measured {got}, inventory records {EXPECT[name]}")
        return 1

    if short:
        for name, uncov in short:
            print(f"\n{name}: {uncov} materialised addresses outside the body, "
                  f"so the verdict rests on an incomplete module")
        return 1

    print("\nno module was classified on a body with unreached code; every "
          "verdict matches plans/au-isp-module-inventory.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
