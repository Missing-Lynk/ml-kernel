#!/usr/bin/env python3
"""
Prove the ltm tile grid and the eight reciprocal tile areas on bank 0x2800.

Local tone mapping divides the frame into tiles and equalises each one, so
every tile histogram has to be normalised by the number of pixels in that tile.
Division is expensive per tile per frame, so the vendor precomputes the
reciprocals once at configure time and the block multiplies by them.

The packer is at 0x18c418..0x18c4b4 in libmpp_service.so. It loads the frame
width and height, doubles the configured tile counts, divides to get the tile
size, and takes the remainder into the last column and the last row:

    tw = W / nx                th = H / ny
    tw_last = W % nx + tw      th_last = H % ny + th

It then loads 0x4000000, which is 2^26, and stores eight quotients of it into
bank+0x10 through bank+0x2c. Every one is `sdiv`, so each truncates toward
zero. The three doubled and quadrupled variants are the same tile area scaled,
which is what a block reading the same histogram at three summed resolutions
would want.

The tile grid is not configured anywhere the driver can see it, so this solves
for it instead: it searches every grid up to 32 by 32 and reports which
reproduce all eight registers. Exactly one does, and its tile count is the 64
curves the coefficient page independently holds.

    kernel/scripts/isp/check-ltm-tiles.py
"""

import importlib.util
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent

BANK = 0x2800

# The frame the driver configures. The packer reads these from the module.
FRAME_W, FRAME_H = 1920, 1080

# 0x4000000 in the packer. The quotients are reciprocal areas in Q26.
RECIPROCAL_SHIFT = 26

# How far to search for the tile grid, and the tile count the coefficient page
# holds independently of any of this, from check-ltm-page.py.
GRID_LIMIT = 32
PAGE_TILES = 64


def tile_areas(frame_w, frame_h, grid_x, grid_y):
    """
    The eight products the packer divides 2^26 by, in bank order.

    The vendor doubles both tile counts before dividing. Whatever that
    doubling means for the grid, it is what the arithmetic does, and the
    solved grid below is in the packer's own pre-doubling units.
    """
    columns, rows = grid_x * 2, grid_y * 2
    if columns > frame_w or rows > frame_h:
        return None

    tile_w, tile_h = frame_w // columns, frame_h // rows
    if not tile_w or not tile_h:
        return None

    last_w = frame_w % columns + tile_w
    last_h = frame_h % rows + tile_h

    return (
        (0x10, tile_w * tile_h, 'tile'),
        (0x14, 2 * tile_w * tile_h, 'tile x2'),
        (0x18, 4 * tile_w * tile_h, 'tile x4'),
        (0x1C, tile_h * last_w, 'last column'),
        (0x20, 2 * tile_h * last_w, 'last column x2'),
        (0x24, tile_w * last_h, 'last row'),
        (0x28, 2 * tile_w * last_h, 'last row x2'),
        (0x2C, last_w * last_h, 'corner'),
    )


def reciprocals(frame_w, frame_h, grid_x, grid_y):
    areas = tile_areas(frame_w, frame_h, grid_x, grid_y)
    if areas is None:
        return None

    # sdiv truncates toward zero, and every operand here is positive.
    return [((1 << RECIPROCAL_SHIFT) // area, off, area, what)
            for off, area, what in areas]


def load_audit():
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
    _library, final, _origin = audit.load_tables()

    offsets = [off for off, _area, _what in
               tile_areas(FRAME_W, FRAME_H, 1, 1)]
    missing = [f'{BANK + off:#06x}' for off in offsets
               if BANK + off not in final]
    if missing:
        sys.exit(f'the driver writes no value at {", ".join(missing)}, so '
                 f'there is nothing to prove the packer against')

    measured = [final[BANK + off] for off in offsets]

    solved = [(x, y)
              for x in range(1, GRID_LIMIT + 1)
              for y in range(1, GRID_LIMIT + 1)
              if (r := reciprocals(FRAME_W, FRAME_H, x, y)) is not None
              and [value for value, _o, _a, _w in r] == measured]

    print(f'searched every tile grid up to {GRID_LIMIT} by {GRID_LIMIT} '
          f'against the eight measured registers')
    if len(solved) != 1:
        print(f'FAIL: {len(solved)} grids reproduce them: {solved}. A unique '
              f'solution is what makes the grid recovered rather than assumed.')
        return 1

    grid_x, grid_y = solved[0]
    tiles = grid_x * grid_y
    print(f'exactly one: {grid_x} by {grid_y}, so {tiles} tiles\n')

    if tiles != PAGE_TILES:
        print(f'FAIL: the solved grid is {tiles} tiles and the coefficient '
              f'page holds {PAGE_TILES} curves. Two independent readings of '
              f'the same geometry disagree.')
        return 1

    columns, rows = grid_x * 2, grid_y * 2
    print(f'frame {FRAME_W} x {FRAME_H} over {columns} x {rows} gives tiles of '
          f'{FRAME_W // columns} x {FRAME_H // rows}, last column '
          f'{FRAME_W % columns + FRAME_W // columns}, last row '
          f'{FRAME_H % rows + FRAME_H // rows}\n')

    failures = []
    for value, off, area, what in reciprocals(FRAME_W, FRAME_H,
                                              grid_x, grid_y):
        got = final[BANK + off]
        print(f'  {BANK + off:#06x}  2^{RECIPROCAL_SHIFT} / {area:<6} = '
              f'{value:<6}  driver {got:<6}  {what}')
        if got != value:
            failures.append(f'{BANK + off:#06x}: the packer gives {value} and '
                            f'the driver installs {got}')

    if failures:
        print()
        for line in failures:
            print(f'FAIL: {line}')

        return 1

    print(f'\nall eight reciprocal tile areas reproduce, and the {tiles}-tile '
          f'grid they solve to is the one the coefficient page holds')

    return 0


if __name__ == '__main__':
    sys.exit(main())
