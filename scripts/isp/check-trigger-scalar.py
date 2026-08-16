#!/usr/bin/env python3
"""
Measure the AEC trigger scalar from a capture's colour registers.

The vendor's AE hands every triggered ISP stage a payload carrying TWO
abscissae, and each stage picks one from a flag in its own tuning header. The
axes in the tuning file separate them without ambiguity:

  linear gain, 1 to 2048     rnr, lnr, de3d, cfa, cnf, in powers of two
  trigger scalar, 0 to 550   gamma, drc, cm, cm2

That is why the gamma curve and DRC profile bands are on a 0 to 500 axis while
the noise ladders are on a gain: they are keyed on different quantities. The
scalar's producer is the open item in plans/isp-tone-selector.md.

This script is the measuring instrument for it. cm and cm2 are keyed on the same
scalar as the tone pages, they interpolate, and they are ordinary registers a
capture dumps. Inverting them turns any register sweep off a streaming vendor
into a numeric interval for the scalar at that moment, which is far tighter than
what the gamma and DRC bands alone can say: a page tells you only which band the
scalar was in, while cm2's interpolated bound tells you where inside the gap it
sat.

The inversion runs the shipped C headers through ladder-dump.c, so what is
inverted is the driver's own arithmetic rather than a restatement of it.

A captured DRC page measures the scalar harder still. Its dynamic banks are the
blob's own 20-bit samples with no transform in between, so a blend of two
profiles either reproduces a page exactly or it does not, and the weight that
does places the scalar inside the gap between their bands. --drc-page reports
that, and out/au-tone-tables/pre-drc.bin fits profile 3 to profile 4 at weight
exactly 0.2500, which is a scalar of exactly 275.

The same sweep that carries cm and cm2 also carries rnr, which is keyed on the
OTHER abscissa. So one capture measures both, with nothing paired to a heap dump
and no assumption about when either was taken, and --exp-table turns the gain
into the AE exposure-table index for comparison.

    kernel/scripts/isp/check-trigger-scalar.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin \\
        --drc-page out/au-tone-tables/pre-drc.bin \\
        --capture out/au-snapshot/registers.txt \\
        --capture out/au-chain/slotA.txt \\
        --exp-table

Add --gamma-curve/--drc-profile to intersect a capture whose tone pages were
also identified; the script then reports whether the two agree, which is the
cross-check that the four stages really do share one scalar.
"""

import argparse
import os
import pathlib
import re
import struct
import subprocess
import sys
import tempfile

from blob_layout import Blob

HERE = pathlib.Path(__file__).resolve().parent
DRIVERS = HERE.parent.parent / 'overlay' / 'drivers' / 'media' / 'artosyn'

# Where each stage's abscissa axis lives, and how many entries it has. A band is
# a pair of floats; a stage is keyed on the gain when its axis climbs in powers
# of two and on the trigger scalar when it spans 0 to 550.
AXES = (
    ('rnr', 0x79EC, 12, 'gain'),
    ('lnr', 0x89E98, 11, 'gain'),
    ('de3d', 0x9632C, 12, 'gain'),
    ('cfa', 0x24558, 5, 'gain'),
    ('cnf', 0x8E1DC, 11, 'gain'),
    ('gamma', 0x26B0C, 5, 'trigger'),
    ('drc', 0x17A9C, 6, 'trigger'),
    ('cm', 0x89D10, 5, 'trigger'),
    ('cm2', 0xA1318, 5, 'trigger'),
)

# The separation, stated as a test rather than as a reading of the table. The
# crisp part is the first band's lower edge: a gain cannot go below unity and
# every gain axis starts at exactly 1.0, while every trigger axis starts at 0.
# The tops are a second, looser guard, because cfa's gain axis stops at 100.1
# where the others run to 1024 or beyond.
GAIN_AXIS_BOTTOM = 1.0
TRIGGER_AXIS_BOTTOM = 0.0
GAIN_AXIS_TOP = 100.0
TRIGGER_AXIS_TOP = 600.0

# The cm and cm2 registers the inversion reads, by absolute ISP offset, and the
# ladder-dump emit position each corresponds to.
CM_FIELD = (0x483C, ('cm', 0))
CM2_LO2 = (0x4824, ('cm2', 3))
CM2_RECIP2 = (0x4830, ('cm2', 6))

# rnr's twelve ladder registers, which invert to the OTHER abscissa, the gain.
# A capture that dumps both banks therefore measures both scalars at one moment
# with no heap dump and no pairing assumption, which is what makes the two
# comparable at all.
RNR_LADDER = 0x1808
RNR_REGS = 12
RNR_GAIN_TOP = 70.0

# cm packs its gain into the low seven bits and shares the register.
CM_FIELD_MASK = 0x7F

# The sweep. A quarter of a count is finer than any band edge in the blob and
# keeps the run under a few seconds.
SWEEP_TOP = 560.0
SWEEP_STEP_Q8 = 64

# The DRC page, from ar-isp-codec.h. Its dynamic banks are the blob's samples
# verbatim, with no transform between file and page, which is what makes the
# blend fit exact rather than approximate.
DRC_BLOB_OFFSET = 0x17B1C
DRC_BLOB_STRIDE = 0xC8C
DRC_BLOB_BANK1 = 0x404
DRC_PROFILES = 6
DRC_BANK = 0x800
DRC_RECORDS = 128
DRC_SAMPLES = 257
DRC_LANE_MAX = 0xFFFFF

# The blend-weight search grid. 1/1024 is finer than the smallest step any
# 20-bit sample pair can resolve.
BLEND_STEPS = 1024

# The gamma page, for the reproduction check.
GAMMA_PAGE = 0x800
GAMMA_SAMPLES = 512

# Gamma's decimation residual against the vendor, as a ratchet. The driver takes
# every eighth entry of the stored 4096-entry curve, and the vendor's page comes
# out at or just under that, never over: 0 to 5 counts of 4095, mean near 1, on
# both captures and identically whether the page is blended or a pure selection.
# So it is the sampling model, not the blend. Bounded and measured rather than
# closed: no curve pair, no blend weight and no blend against either of the
# library's own gamma curves reproduces the capture exactly.
GAMMA_RESIDUAL_MAX = 5
GAMMA_RESIDUAL_MEAN = 1.1


def build(out: pathlib.Path) -> pathlib.Path:
    """Compile ladder-dump.c against the driver headers, as check-ladder-c does."""
    cc = os.environ.get('CC', 'cc')
    cmd = [cc, '-O2', '-Wall', '-Wextra', '-I', str(DRIVERS), '-o', out,
           str(HERE / 'ladder-dump.c')]
    try:
        subprocess.run(cmd, check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        sys.exit(f'building ladder-dump failed: {exc}')

    return out


def run(binary: pathlib.Path, tuning: pathlib.Path,
        q8: int) -> dict[tuple[str, int], int]:
    """The C output at one abscissa, as {(stage, index): value}."""
    proc = subprocess.run([binary, tuning, str(q8)], check=True,
                          capture_output=True, text=True)
    out: dict[tuple[str, int], int] = {}
    for line in proc.stdout.splitlines():
        stage, index, value = line.split()
        out[(stage, int(index))] = int(value, 16)

    return out


def drc_profile_samples(blob: bytes, profile: int) -> list[list[int]]:
    """A profile's two dynamic banks, 257 samples each, straight from the blob."""
    banks = []
    for b in range(2):
        base = (DRC_BLOB_OFFSET + profile * DRC_BLOB_STRIDE + b * DRC_BLOB_BANK1)
        banks.append([struct.unpack_from('<I', blob, base + 4 * i)[0]
                      for i in range(DRC_SAMPLES)])

    return banks


def drc_unpack(page: bytes) -> list[list[int]]:
    """A captured DRC page's two dynamic banks, back to 257 samples each.

    The record format is in ar-isp-codec.h: three 20-bit samples per 16-byte
    record, consecutive records overlapping by one. Nothing is transformed on
    the way in, so a captured page and the blob's own samples are directly
    comparable and an exact fit means exact.
    """
    banks = []
    for b in range(2):
        base = b * DRC_BANK
        out = [0] * DRC_SAMPLES
        for i in range(DRC_RECORDS):
            rec = base + i * 16
            w0, w1 = struct.unpack_from('<II', page, rec)
            half = struct.unpack_from('<H', page, rec + 8)[0]
            out[2 * i + 0] = w0 & DRC_LANE_MAX
            out[2 * i + 1] = ((w0 >> 20) | ((w1 & 0xFF) << 12)) & DRC_LANE_MAX
            out[2 * i + 2] = (((w1 >> 28) & 0xF) | (half << 4)) & DRC_LANE_MAX

        banks.append(out)

    return banks


def fit_drc_blend(blob: bytes,
                  page: bytes) -> tuple[int, int, float, int] | None:
    """The (low, high, weight) blend of blob profiles that reproduces a page."""
    got = drc_unpack(page)
    profiles = [drc_profile_samples(blob, p) for p in range(DRC_PROFILES)]
    best = None
    for a in range(DRC_PROFILES):
        for b in range(DRC_PROFILES):
            for w in range(BLEND_STEPS + 1):
                worst = 0
                for k in range(2):
                    pa, pb, gk = profiles[a][k], profiles[b][k], got[k]
                    for i in range(DRC_SAMPLES):
                        v = (pa[i] * (BLEND_STEPS - w) + pb[i] * w) // BLEND_STEPS
                        d = abs(v - gk[i])
                        if d > worst:
                            worst = d

                    if best and worst > best[3]:
                        break

                if best is None or worst < best[3]:
                    best = (a, b, w / BLEND_STEPS, worst)

    return best


def read_capture(path: pathlib.Path) -> dict[int, int]:
    """Every ISP register a capture dumps, by absolute offset."""
    out: dict[int, int] = {}
    base = None
    for line in pathlib.Path(path).read_text().splitlines():
        hit = re.match(r'---\s*isp\s*\+(0x[0-9a-f]+)', line)
        if hit:
            base = int(hit.group(1), 16)
            continue

        hit = re.match(r'SECTION isp-([0-9a-f]{2})\b', line)
        if hit:
            base = int(hit.group(1), 16) << 8
            continue

        if line.startswith('SECTION'):
            base = None
            continue

        hit = re.match(r'\s*\+(0x[0-9a-f]+):\s*(.*)', line)
        if hit and base is not None:
            off = base + int(hit.group(1), 16)
            for i, word in enumerate(hit.group(2).split()):
                if re.fullmatch(r'[0-9a-f]{8}', word):
                    out[off + 4 * i] = int(word, 16)

    return out


def floats(blob: bytes, off: int, count: int) -> list[float]:
    return [struct.unpack_from('<f', blob, off + 4 * i)[0] for i in range(count)]


def report_axes(blob: bytes) -> None:
    """Print each stage's axis and prove the two families stay separable."""
    print('stage abscissa axes, from the tuning file:\n')
    bad = []
    for name, off, count, kind in AXES:
        v = floats(blob, off, 2 * count)
        top = max(v)
        bands = ' '.join(f'[{v[2 * i]:g},{v[2 * i + 1]:g}]' for i in range(count))
        print(f'  {name:6} {kind:8} {bands}')
        if kind == 'gain' and (v[0] != GAIN_AXIS_BOTTOM or top < GAIN_AXIS_TOP):
            bad.append(f'{name}: a gain axis starting at {v[0]:g} and '
                       f'stopping at {top:g}')
        if kind == 'trigger' and (v[0] != TRIGGER_AXIS_BOTTOM
                                  or top > TRIGGER_AXIS_TOP):
            bad.append(f'{name}: a trigger axis starting at {v[0]:g} and '
                       f'reaching {top:g}')

    print()
    if bad:
        for line in bad:
            print(f'  {line}')

        sys.exit('the two abscissa families are no longer separable by their '
                 'axes, so this blob does not support the split this script '
                 'and the cm/cm2 appliers rest on')

    print(f'  the two families are disjoint: every gain axis starts at '
          f'{GAIN_AXIS_BOTTOM:g} and passes {GAIN_AXIS_TOP:g}, every trigger '
          f'axis starts at {TRIGGER_AXIS_BOTTOM:g} and stops short of '
          f'{TRIGGER_AXIS_TOP:g}\n')


def build_tone_dump(out: pathlib.Path) -> pathlib.Path:
    """Compile tone-dump.c, the shipped selector and page builders host-side."""
    cc = os.environ.get('CC', 'cc')
    cmd = [cc, '-O2', '-Wall', '-Wextra', '-I', str(DRIVERS), '-o', out,
           str(HERE / 'tone-dump.c')]
    try:
        subprocess.run(cmd, check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        sys.exit(f'building tone-dump failed: {exc}')

    return out


def run_tone_dump(binary: pathlib.Path, tuning: pathlib.Path, scalar_q8: int,
                  tmp: str) -> tuple[bytes, bytes, list[int]]:
    """Both pages the driver would build at one scalar, plus its selection."""
    gamma = os.path.join(tmp, 'gamma.bin')
    drc = os.path.join(tmp, 'drc.bin')
    proc = subprocess.run([binary, tuning, str(scalar_q8), gamma, drc],
                          check=True, capture_output=True, text=True)
    pick = [int(v) for v in proc.stdout.split()]

    return (pathlib.Path(gamma).read_bytes(),
            pathlib.Path(drc).read_bytes(), pick)


def gamma_samples(page: bytes) -> list[int]:
    """A packed gamma page back to its 512 twelve-bit samples."""
    out = []
    for i in range(GAMMA_SAMPLES // 4):
        w0, w1, w2 = struct.unpack_from('<III', page, i * 16)
        out += [w0 & 0xFFF, (w0 >> 12) & 0xFFF,
                (w1 >> 4) & 0xFFF, (w2 >> 8) & 0xFFF]

    return out


def report_pages(blob: bytes, tuning: pathlib.Path,
                 pairs: list[tuple[str, str]]) -> bool:
    """
    Rebuild each captured page pair from the scalar its own DRC page measures.

    The loop this closes: the DRC page measures the scalar, the scalar goes
    through the shipped selector, the shipped builders produce both pages, and
    the DRC one has to come back byte for byte against the capture it started
    from. Nothing is fitted twice, because the selector and the builder never
    see the capture.
    """
    if not pairs:
        return False

    bands = floats(blob, 0x17A9C, 2 * DRC_PROFILES)
    failed = False
    print('page reproduction from the measured scalar:\n')
    with tempfile.TemporaryDirectory() as tmp:
        binary = build_tone_dump(os.path.join(tmp, 'tone-dump'))
        for gamma_path, drc_path in pairs:
            cap_drc = pathlib.Path(drc_path).read_bytes()
            low, high, weight, worst = fit_drc_blend(blob, cap_drc)
            if worst:
                failed = True
                print(f'  {drc_path}: no blend reproduces it, closest is '
                      f'profile {low} to {high} off by {worst}')
                continue

            if low == high or weight in (0.0, 1.0):
                pure = high if weight == 1.0 else low
                # Any scalar inside the band selects it with no blend; the low
                # edge is the one value guaranteed to be in range.
                scalar = bands[2 * pure]
            else:
                gap = (bands[2 * low + 1], bands[2 * high])
                scalar = gap[0] + weight * (gap[1] - gap[0])

            q8 = int(round(scalar * 256))
            built_gamma, built_drc, pick = run_tone_dump(binary, tuning, q8, tmp)

            ok = built_drc[:len(cap_drc)] == cap_drc
            failed = failed or not ok
            print(f'  scalar {scalar:g} (Q8 {q8}), selector picks gamma '
                  f'{pick[0]} to {pick[1]} weight {pick[2]}, DRC {pick[3]} to '
                  f'{pick[4]} weight {pick[5]}')
            print(f'    {drc_path}\n      '
                  f'{"BYTE-EXACT over " + str(len(cap_drc)) + " bytes" if ok else "DIFFERS"}')

            if not gamma_path:
                continue

            cap_gamma = pathlib.Path(gamma_path).read_bytes()
            mine = gamma_samples(built_gamma)
            got = gamma_samples(cap_gamma)
            diff = [m - g for m, g in zip(mine, got, strict=True)]
            worst_d = max(abs(d) for d in diff)
            mean_d = sum(abs(d) for d in diff) / len(diff)
            over = worst_d > GAMMA_RESIDUAL_MAX or mean_d > GAMMA_RESIDUAL_MEAN
            failed = failed or over
            print(f'    {gamma_path}\n      decimation residual: mean '
                  f'{mean_d:.4f}, worst {worst_d}, signed range '
                  f'[{min(diff)}, {max(diff)}], '
                  f'{sum(1 for d in diff if d)} of {len(diff)} samples'
                  f'{"  OVER THE RECORDED BOUND" if over else ""}')

    print()

    return failed


def invert_gain(binary: pathlib.Path, tuning: pathlib.Path,
                want: list[int]) -> tuple[int | None, int | None]:
    """The gain interval whose rnr ladder reproduces a capture's twelve words."""
    lo = hi = None
    q = 256
    while q <= int(RNR_GAIN_TOP * 256):
        got = run(binary, tuning, q)
        if [got[('rnr', i)] for i in range(RNR_REGS)] == want:
            if lo is None:
                lo = q
            hi = q
        elif lo is not None:
            break

        q += 1

    return lo, hi


def read_exp_table(blob: bytes) -> list[int]:
    """The vendor exposure table, as {gain Q8} per index.

    Read from the tuning blob through the shared layout, not from a generated header: the same
    366 entries the ISP driver and ml-aed use, at the one offset blob-layout.toml records.
    """
    return [int.from_bytes(record[:4], "little")
            for record in Blob(blob).records("ae_exposure_table")]


def report_drc_pages(blob: bytes, paths: list[pathlib.Path]) -> bool:
    """Fit each captured DRC page as a blend and report the scalar it implies.

    This is the tightest measurement of the scalar available anywhere, and the
    reason is that nothing is quantised on the way: the page carries the blob's
    own 20-bit samples, so a blend either reproduces it exactly or it does not.
    A weight lands the scalar inside the gap between the two profiles' bands,
    linearly, which is how every other blended stage in this driver reads its
    ladder.
    """
    if not paths:
        return False

    bands = floats(blob, 0x17A9C, 2 * DRC_PROFILES)
    print('scalar measured from captured DRC pages:\n')
    failed = False
    for path in paths:
        page = pathlib.Path(path).read_bytes()
        if len(page) < 2 * DRC_BANK:
            print(f'  {path}: {len(page)} bytes, too short for two banks')
            failed = True
            continue

        low, high, weight, worst = fit_drc_blend(blob, page)
        if worst:
            failed = True
            print(f'  {path}\n    no blend reproduces it: closest is profile '
                  f'{low} to {high} at {weight:.4f}, worst sample off by '
                  f'{worst}')
            continue

        if low == high or weight in (0.0, 1.0):
            pure = high if weight == 1.0 else low
            print(f'  {path}\n    profile {pure} exactly, no blend -> scalar in '
                  f'[{bands[2 * pure]:g}, {bands[2 * pure + 1]:g}]')
            continue

        gap = (bands[2 * low + 1], bands[2 * high])
        scalar = gap[0] + weight * (gap[1] - gap[0])
        print(f'  {path}\n    profile {low} to {high} at weight {weight:.4f}, '
              f'exactly -> scalar {scalar:g}, in the gap '
              f'({gap[0]:g}, {gap[1]:g})')

    print()

    return failed


def invert(binary: pathlib.Path, tuning: pathlib.Path,
           want: list[int]) -> tuple[int | None, int | None]:
    """Every quarter-count scalar whose C output reproduces the capture."""
    lo = hi = None
    q = 0
    while q <= int(SWEEP_TOP * 256):
        got = run(binary, tuning, q)
        if all(got[key] == value for key, value in want):
            if lo is None:
                lo = q
            hi = q
        elif lo is not None:
            # Past the run: the model is piecewise monotone, so the first gap
            # after a hit ends the interval. Reporting a second one would hide
            # that the registers do not pin a single band.
            break

        q += SWEEP_STEP_Q8

    return lo, hi


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tuning', required=True, help='sensor tuning file')
    ap.add_argument('--capture', action='append', default=[],
                    help='a register sweep to invert; repeatable')
    ap.add_argument('--gamma-curve', type=int,
                    help='the gamma curve identified for the LAST capture given')
    ap.add_argument('--drc-profile', type=int,
                    help='the DRC profile identified for the LAST capture given')
    ap.add_argument('--drc-page', action='append', default=[],
                    help='a captured DRC page to fit as a blend; repeatable')
    ap.add_argument('--pair', action='append', default=[], nargs=2,
                    metavar=('GAMMA_PAGE', 'DRC_PAGE'),
                    help='a gamma and DRC page captured together. The DRC page '
                         'measures the scalar, then both are rebuilt from it '
                         'through the shipped selector and builders and '
                         'compared back; repeatable')
    ap.add_argument('--exp-table', action='store_true',
                    help='report the gain a capture inverts to as an AE exposure-table index, '
                         'read from the tuning blob through blob-layout.toml')
    args = ap.parse_args()

    blob = pathlib.Path(args.tuning).read_bytes()
    report_axes(blob)

    failed_pages = report_drc_pages(blob, args.drc_page)
    failed_pages = report_pages(blob, args.tuning, args.pair) or failed_pages

    if not args.capture:
        return 1 if failed_pages else 0

    with tempfile.TemporaryDirectory() as tmp:
        binary = build(os.path.join(tmp, 'ladder-dump'))
        results = []
        for path in args.capture:
            regs = read_capture(path)
            missing = [f'{off:#06x}' for off, _ in (CM_FIELD, CM2_LO2, CM2_RECIP2)
                       if off not in regs]
            if missing:
                print(f'{path}: no cm/cm2 registers ({", ".join(missing)} '
                      f'absent), skipped')
                continue

            want = ((CM_FIELD[1], regs[CM_FIELD[0]] & CM_FIELD_MASK),
                    (CM2_LO2[1], regs[CM2_LO2[0]]),
                    (CM2_RECIP2[1], regs[CM2_RECIP2[0]]))
            lo, hi = invert(binary, args.tuning, want)

            gain = (None, None)
            if all(RNR_LADDER + 4 * i in regs for i in range(RNR_REGS)):
                gain = invert_gain(binary, args.tuning,
                                   [regs[RNR_LADDER + 4 * i]
                                    for i in range(RNR_REGS)])

            results.append((path, want, lo, hi, gain))

    table = read_exp_table(blob) if args.exp_table else None

    print('scalar measured from cm and cm2:\n')
    failed = False
    for path, want, lo, hi, gain in results:
        cm, lo2, recip = (v for _, v in want)
        if lo is None:
            failed = True
            print(f'  {path}\n    cm {cm}, cm2 lo2 {lo2}, recip2 {recip}: NO '
                  f'scalar in [0, {SWEEP_TOP:g}] reproduces these, so either '
                  f'the capture is not this tuning file or the model is wrong')
            continue

        print(f'  {path}\n    cm {cm}, cm2 lo2 {lo2}, recip2 {recip} '
              f'-> scalar in [{lo / 256:g}, {hi / 256:g}]')

        # The same sweep also carries rnr, which is keyed on the other
        # abscissa. Reporting them together is the whole point: one capture,
        # both scalars, no pairing to a heap dump and nothing assumed about
        # when either was taken.
        if gain[0] is None:
            continue

        print(f'    rnr ladder in the same sweep -> gain in '
              f'[{gain[0] / 256:g}, {gain[1] / 256:g}]', end='')
        if table:
            idx = [i for i, g in enumerate(table) if gain[0] <= g <= gain[1]]
            print(f', exposure-table index {idx if idx else "none"}')
        else:
            print()

    if results and (args.gamma_curve is not None or args.drc_profile is not None):
        path, _, lo, hi, _gain = results[-1]
        window = [0.0, SWEEP_TOP]
        for name, index, off, count in (('gamma', args.gamma_curve, 0x26B0C, 5),
                                        ('drc', args.drc_profile, 0x17A9C, 6)):
            if index is None:
                continue

            if not 0 <= index < count:
                sys.exit(f'{name} index {index} is outside the blob\'s {count}')

            band = floats(blob, off + 8 * index, 2)
            window = [max(window[0], band[0]), min(window[1], band[1])]
            print(f'\n  {name} {index} selected -> band [{band[0]:g}, {band[1]:g}]')

        print(f'  tone pages allow [{window[0]:g}, {window[1]:g}]')
        if lo is None or lo / 256 > window[1] or hi / 256 < window[0]:
            failed = True
            print('  DISAGREEMENT: the colour pair and the tone pages do not '
                  'overlap, so they are not keyed on one scalar')
        else:
            both = (max(window[0], lo / 256), min(window[1], hi / 256))
            print(f'  both agree, and together they pin the scalar to '
                  f'[{both[0]:g}, {both[1]:g}]')

    if failed or failed_pages:
        return 1

    print('\nEvery capture inverts, so the scalar is a measurable quantity '
          'rather than an inferred one. A recovered selector formula has to '
          'reproduce these intervals from AE state.')

    return 0


if __name__ == '__main__':
    sys.exit(main())
