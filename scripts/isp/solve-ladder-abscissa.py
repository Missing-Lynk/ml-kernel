#!/usr/bin/env python3
"""
Solve a captured ISP bank for the ladder abscissa that produced it.

The rnr, lnr and de3d ladders select and blend on an abscissa the vendor calls
a gain. Recovering the transforms did not recover that input: the driver takes
it through module parameters and nothing derives it. This inverts the
transforms instead. Because they reproduce the vendor bit-exactly, an abscissa
that makes every captured register match is a measurement of what the vendor
was running at, not a fit.

The answer is an interval rather than a point. The blend truncates, so a range
of abscissas produces identical registers, and the honest output is the widest
range with zero bit errors. A narrow interval is a sharp measurement; a wide
one means the capture was inside a band where the record applies verbatim and
the abscissa is only bounded, not determined.

Use it to test a candidate input: solve the capture, then ask whether the
quantity you think drives the ladder lands inside the interval. That is how the
sensor gain table was ruled out as the abscissa on its own, having looked like
a match on a single earlier data point.

    kernel/scripts/isp/solve-ladder-abscissa.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin \\
        --capture out/au-chain/slotA.txt
"""

import argparse
import contextlib
import importlib.util
import pathlib
import re
import sys
from collections.abc import Callable
from types import ModuleType

HERE = pathlib.Path(__file__).resolve().parent

# The three stages, each with the shape its own check script exposes. They do
# not share an interface: rnr returns a flat bank and carries a second tail
# ladder, lnr packs into a seed of preserved bits and skips two registers. An
# adapter per stage is honest about that; a common signature would not be.
#
# Each adapter returns [(offset, mask, value)] for one abscissa.


# What the three per-stage adapters below all look like: one abscissa in,
# the (register, mask, value) triples the stage would write out.
Adapter = Callable[[ModuleType, bytes, dict[int, int], int],
                   list[tuple[int, int, int]]]


def de3d_regs(mod: ModuleType, blob: bytes, live: dict[int, int],
              q16: int) -> list[tuple[int, int, int]]:
    got = mod.de3d_from_blob(blob, q16)
    return [(r, m, v) for (r, m), v in zip(mod.REGS, got, strict=True)]


def rnr_regs(mod: ModuleType, blob: bytes, live: dict[int, int],
             q16: int) -> list[tuple[int, int, int]]:
    out = [(0x1808 + 4 * i, 0xFFFFFFFF, v)
           for i, v in enumerate(mod.rnr_from_blob(blob, q16))]
    out += [(mod.TAIL_BASE + 4 * i, 0xFFFFFFFF, v)
            for i, v in enumerate(mod.tail_from_blob(blob, q16))]

    return out


def lnr_regs(mod: ModuleType, blob: bytes, live: dict[int, int],
             q16: int) -> list[tuple[int, int, int]]:
    # lnr packs into a seed of bits it does not own, which the check script
    # takes from the captured image. Seeding from the same capture means those
    # bits match by construction, so only the ladder-owned bits discriminate
    # here. That is the same basis the shipped check uses.
    seed = [live.get(mod.BANK + 4 * i, 0) for i in range(mod.REGS)]
    regs, _band, _t = mod.lnr_from_blob(blob, seed, q16)

    return [(mod.BANK + 4 * i, 0xFFFFFFFF, v)
            for i, v in enumerate(regs)
            if mod.BANK + 4 * i not in mod.SKIP]


STAGES = (
    ('de3d', 'check-de3d-ladder.py', de3d_regs),
    ('rnr', 'check-rnr-ladder.py', rnr_regs),
    ('lnr', 'check-lnr-ladder.py', lnr_regs),
)

# The search runs over this range in Q16. Wider than any band table, so a stage
# pinned at a clamp is visible as an interval running to an edge.
LO, HI, STEP = 1 << 16, 64 << 16, 16


def load(name: str) -> ModuleType | None:
    path = HERE / name
    if not path.exists():
        return None

    spec = importlib.util.spec_from_file_location(path.stem.replace('-', '_'),
                                                  path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    with contextlib.suppress(SystemExit):
        spec.loader.exec_module(mod)

    return mod


def capture(path: pathlib.Path) -> dict[int, int]:
    """Every ISP register in an au-chain capture, by absolute offset."""
    out: dict[int, int] = {}
    page = None
    with open(path) as handle:
        for line in handle:
            hit = re.match(r'SECTION isp-([0-9a-f]{2})', line)
            if hit:
                page = int(hit.group(1), 16) << 8
                continue

            if line.startswith('SECTION'):
                page = None
                continue

            hit = re.match(r'\+0x([0-9a-f]{4}):\s+(.*)', line)
            if hit and page is not None:
                base = page + int(hit.group(1), 16)
                for i, word in enumerate(hit.group(2).split()):
                    out[base + 4 * i] = int(word, 16)

    return out


def solve(mod: ModuleType, adapter: Adapter, blob: bytes,
          live: dict[int, int]) -> tuple[tuple[int, int] | None, int]:
    """The widest zero-error abscissa interval, and the register count."""
    probe = adapter(mod, blob, live, LO)
    covered = [(r, m) for r, m, _ in probe if r in live]
    if not covered:
        return None, 0

    def exact(q16: int) -> bool:
        for r, m, v in adapter(mod, blob, live, q16):
            if r in live and (v & m) != (live[r] & m):
                return False

        return True

    lo = hi = None
    for q16 in range(LO, HI, STEP):
        if exact(q16):
            lo = q16 if lo is None else lo
            hi = q16
        elif lo is not None:
            break           # first contiguous run; the transforms are monotone

    return (lo, hi), len(covered)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tuning', required=True, help='nt99235 tuning blob')
    ap.add_argument('--capture', required=True, help='an au-chain capture')
    args = ap.parse_args()

    blob = pathlib.Path(args.tuning).read_bytes()
    live = capture(args.capture)
    if not live:
        sys.exit(f'{args.capture}: no ISP register sections')

    print(f'{args.capture}: {len(live)} ISP registers\n')
    print(f'{"stage":6s} {"regs":>5s}  abscissa interval with zero bit errors')

    found = {}
    for name, script, adapter in STAGES:
        mod = load(script)
        if mod is None:
            print(f'{name:6s} {"-":>5s}  no transform available ({script})')
            continue

        span, n = solve(mod, adapter, blob, live)
        if span is None or span[0] is None:
            print(f'{name:6s} {n:5d}  no abscissa reproduces this capture')
            continue

        lo, hi = span
        found[name] = span
        print(f'{name:6s} {n:5d}  {lo / 65536:8.4f}x .. {hi / 65536:8.4f}x')

    if len(found) > 1:
        lo = max(s[0] for s in found.values())
        hi = min(s[1] for s in found.values())
        print()
        if lo <= hi:
            print(f'the stages agree: one abscissa in {lo / 65536:.4f}x .. '
                  f'{hi / 65536:.4f}x explains all of them')
        else:
            print('the stages DISAGREE: no single abscissa explains them, so '
                  'either they are driven separately or a transform is wrong')

    return 0


if __name__ == '__main__':
    sys.exit(main())
