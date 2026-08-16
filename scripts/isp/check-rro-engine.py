#!/usr/bin/env python3
"""
Prove the rro statistics engine's internal structure across its five instances.

The ISP instantiates one zone-metering engine five times: `rro_stats` and
`rro_face_stats` on the main path, and `hdr_rro_0_stats`, `hdr_rro_1_stats` and
`hdr_rro_face_stats` on the HDR path. Each `isp_sub_*_creat` in
libmpp_service.so builds its own copy of the same three handlers and maps its
own bank with the bank offset as an immediate, so the five are separate code
carrying one register layout.

The three HDR instances place that layout eight bytes further into their bank
than the two main-path ones do. With that shift applied, every instance
exposes the same fields, and the fields hold between them:

    engine+0x30 == engine+0x24        the zone width, written a second time
    engine+0x38 == engine+0x28        the zone height, written a second time
    engine+0x3c == 0xff               the saturation threshold

These are what the vendor moves away from its own static image, which carries
0x72, 0x16 and 0xfa there. `rro_face_stats` is what makes this a measurement
rather than a restatement: it meters a different window, so its zone is 36 x 10
where every other instance is 118 x 28, and the relations still hold.

The zone dimensions also predict the accumulator's divisor. Each zone
accumulates one sample per 2x2 Bayer quad per channel, so a zone of w by h
pixels contributes w * h / 4 samples to each channel's count, and that count is
stored inline in the statistics buffer. Both measured counts follow:

    rro_stats        118 x 28 / 4 = 826
    rro_face_stats     36 x 10 / 4 =  90

The zone dimensions themselves come from the grid and the metered window. The
grid is fixed at 16 by 36 in every instance, and each zone is the window
divided by it, rounded up to even. The two instances differ only in the window
and in one term:

    rro_stats       the whole frame, floor division, then two pixels of guard
    rro_face_stats  30 per cent of the frame, ceiling division, no guard

so 1920 / 16 rounded even less two is 118, 1080 / 36 rounded even less two is
28, and on the 576 by 324 window 576 / 16 is 36 and 324 / 36 rounded up even
is 10. The 30 per cent is a hard-coded float quadruple in the library at
0x36e3c0, not a tuning value. Both grids clear the frame: 16 x 118 = 1888 and
36 x 28 = 1008.

    kernel/scripts/isp/check-rro-engine.py
"""

import importlib.util
import pathlib
import sys
from types import ModuleType

HERE = pathlib.Path(__file__).resolve().parent

# Bank base and the offset of the engine layout inside it, per instance. The
# shift is the finding: the HDR instances carry two extra words up front.
INSTANCES = (
    ('rro_stats', 0x6400, 0),
    ('rro_face_stats', 0x64C8, 0),
    ('hdr_rro_0_stats', 0x1D20, 8),
    ('hdr_rro_1_stats', 0x1D78, 8),
    ('hdr_rro_face_stats', 0x1F40, 8),
)

# Engine-relative field offsets, from the bank map in libmpp_service.so's
# attach handlers and confirmed by every instance agreeing on them. The grid
# pair reads 36 at 0x1c and 16 at 0x20, and the vendor's own arithmetic pairs
# the 36 with the vertical extent and the 16 with the horizontal, so 0x1c is
# the row count.
ROWS, COLUMNS = 0x1C, 0x20
ZONE_W, ZONE_H = 0x24, 0x28
FRAME_W, FRAME_H = 0x2C, 0x34
COPY_W, COPY_H = 0x30, 0x38
THRESHOLD = 0x3C
BUFFER = 0x40

THRESHOLD_VALUE = 0xFF

# The vendor's own static image for the three fields it then moves, which is
# what makes them show up as a difference rather than as a default.
IMAGE_BEFORE = {COPY_W: 0x72, COPY_H: 0x16, THRESHOLD: 0xFA}

# Counts read out of captured statistics buffers, in check-stats-layout.py.
MEASURED_COUNTS = {'rro_stats': 826, 'rro_face_stats': 90}

# One sample per 2x2 Bayer quad reaches each channel's accumulator.
BAYER_QUAD = 4

# The frame the driver configures, and how each instance derives its zone from
# it. The face instance meters a fraction of the frame, a hard-coded float
# quadruple in the library at 0x36e3c0, and rounds up where the others round
# down; the others take two pixels of guard. Instances absent here are the HDR
# ones, which reuse the rro_stats arithmetic.
CONFIGURED_W, CONFIGURED_H = 1920, 1080
ZONE_RULES = {
    'rro_stats': (1.0, False, 2),
    'rro_face_stats': (0.3, True, 0),
}


def zone(frame_w: int, frame_h: int, columns: int, rows: int,
         fraction: float, round_up: bool, guard: int) -> tuple[int, int]:
    """The zone the vendor's configure routine computes, in pixels."""
    width, height = int(frame_w * fraction), int(frame_h * fraction)

    def divide(extent: int, count: int) -> int:
        # fcvtps ceilings; the integer path truncates. Both then round up to
        # even, and the whole-frame path subtracts a guard.
        exact = -(-extent // count) if round_up else extent // count

        return ((exact + 1) & ~1) - guard

    return divide(width, columns), divide(height, rows)


def load_audit() -> ModuleType:
    """The driver's register tables, via audit-provenance.py."""
    path = HERE / 'audit-provenance.py'
    spec = importlib.util.spec_from_file_location('ar_isp_audit', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    source = path.read_text().replace(
        "if __name__ == '__main__':\n    sys.exit(main())", '')
    exec(compile(source, str(path), 'exec'), mod.__dict__)

    return mod


def main() -> int:
    audit = load_audit()
    library, final, _origin = audit.load_tables()

    failures = []
    print(f'{"instance":<20}{"cols":>6}{"rows":>6}{"zone":>10}{"frame":>12}'
          f'{"threshold":>11}')

    for name, bank, shift in INSTANCES:
        def reg(off: int, base: int = bank, delta: int = shift) -> int | None:
            return final.get(base + delta + off)

        fields = {off: reg(off) for off in
                  (COLUMNS, ROWS, ZONE_W, ZONE_H, FRAME_W, FRAME_H,
                   COPY_W, COPY_H, THRESHOLD)}
        missing = [f'{off:#04x}' for off, value in fields.items()
                   if value is None]
        if missing:
            failures.append(f'{name}: the driver writes no value at engine '
                            f'offsets {", ".join(missing)}, so the layout does '
                            f'not hold at bank {bank:#06x} shifted by {shift}')
            continue

        print(f'{name:<20}{fields[COLUMNS]:>6}{fields[ROWS]:>6}'
              f'{fields[ZONE_W]:>5} x{fields[ZONE_H]:>3}'
              f'{fields[FRAME_W]:>7} x{fields[FRAME_H]:>4}'
              f'{fields[THRESHOLD]:>11}')

        if fields[COPY_W] != fields[ZONE_W]:
            failures.append(
                f'{name}: engine+{COPY_W:#04x} is {fields[COPY_W]:#x} where '
                f'engine+{ZONE_W:#04x} is {fields[ZONE_W]:#x}; the second copy '
                f'of the zone width no longer tracks the first')

        if fields[COPY_H] != fields[ZONE_H]:
            failures.append(
                f'{name}: engine+{COPY_H:#04x} is {fields[COPY_H]:#x} where '
                f'engine+{ZONE_H:#04x} is {fields[ZONE_H]:#x}; the second copy '
                f'of the zone height no longer tracks the first')

        if fields[THRESHOLD] != THRESHOLD_VALUE:
            failures.append(
                f'{name}: engine+{THRESHOLD:#04x} is {fields[THRESHOLD]:#x}, '
                f'not the {THRESHOLD_VALUE:#x} every instance carries')

        # The three fields have to differ from the vendor's static image, or
        # the relation above is satisfied by the image alone and proves nothing.
        for off, before in IMAGE_BEFORE.items():
            if library.get(bank + shift + off) != before:
                failures.append(
                    f'{name}: the library image at engine+{off:#04x} is '
                    f'{library.get(bank + shift + off)}, not the {before:#x} '
                    f'the vendor moves away from; the shift or the bank is '
                    f'wrong')

    # The zone against the grid and the metered window, which is what makes it
    # derived rather than read out of the configuration.
    print()
    for name, bank, shift in INSTANCES:
        if name not in ZONE_RULES:
            continue

        fraction, round_up, guard = ZONE_RULES[name]
        columns = final[bank + shift + COLUMNS]
        rows = final[bank + shift + ROWS]
        want_w, want_h = zone(CONFIGURED_W, CONFIGURED_H, columns, rows,
                              fraction, round_up, guard)
        got_w = final[bank + shift + ZONE_W]
        got_h = final[bank + shift + ZONE_H]
        window = f'{int(CONFIGURED_W * fraction)} x {int(CONFIGURED_H * fraction)}'
        print(f'{name}: window {window} over {columns} x {rows}, '
              f'{"ceiling" if round_up else "floor"}, guard {guard} gives '
              f'{want_w} x {want_h}, bank holds {got_w} x {got_h}')
        if (want_w, want_h) != (got_w, got_h):
            failures.append(
                f'{name}: the configure routine gives {want_w} x {want_h} and '
                f'the bank holds {got_w} x {got_h}, so the zone does not '
                f'follow from the grid and the window')

        if columns * got_w > CONFIGURED_W or rows * got_h > CONFIGURED_H:
            failures.append(
                f'{name}: {columns} x {got_w} by {rows} x {got_h} does not fit '
                f'inside {CONFIGURED_W} x {CONFIGURED_H}, which the vendor '
                f'clamps for')

    print()
    for name, bank, shift in INSTANCES:
        if name not in MEASURED_COUNTS:
            continue

        zone_w = final[bank + shift + ZONE_W]
        zone_h = final[bank + shift + ZONE_H]
        predicted = zone_w * zone_h // BAYER_QUAD
        measured = MEASURED_COUNTS[name]
        print(f'{name}: zone {zone_w} x {zone_h} over a {BAYER_QUAD}-pixel '
              f'quad predicts {predicted} samples, buffer holds {measured}')
        if predicted != measured:
            failures.append(
                f'{name}: the zone predicts {predicted} samples per channel '
                f'and the captured buffer holds {measured}, so the divisor is '
                f'not one sample per Bayer quad')

    if failures:
        print()
        for line in failures:
            print(f'FAIL: {line}')

        return 1

    print('\nthe rro engine layout holds across all five instances, and the '
          'zone dimensions predict both measured accumulator counts')

    return 0


if __name__ == '__main__':
    sys.exit(main())
