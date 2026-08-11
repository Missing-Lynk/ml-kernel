#!/usr/bin/env python3
"""
Prove the de3d geometry transform in ar-isp-de3d-geom.h against the vendor.

The de3d ladder recomputes the tuning-driven half of bank 0x2e00 on every gain
move. Thirteen registers are not in it: the vendor computes them once from the
frame geometry and the sensor line length, in the configuration function at
0x1c3e00 in libmpp_service.so and the pair at 0x1c4e5c. They do not depend on
gain and they do not read the tuning file, which is why no ladder covers them
and why they sat in the unexplained set while the ladder was already exact.

What the vendor does, register by register, and where it does it:

  the working stride (0x2e38, 0x2e40, 0x2e5c, 0x2e64) is the padded width in
  tiles of 20, sixteen bytes per tile, rounded up to 256: at 1928 that is
  ceil(1928/20) = 97, times 16 is 1552, rounded up is 1792. The buffer 2
  stride (0x2e84, 0x2e8c) rounds ceil(1928/8) = 241 up the same way, to 256.
  0x1c42a0 and 0x1c4e40 respectively, both `adds #0xff` then `and ~0xff`;

  the line delay (0x2e24, 0x2e48 bits 28:16) is the horizontal blanking plus
  five percent of it plus 200. The blanking is hts - width, so this is the one
  register that needs the sensor's line length rather than the frame. The
  scale is a double 0.05 at 0x364b78 and the bias is `add w1, w1, #0xc8`;

  0x2e54 packs ceil(width/16) - 1 into bits 31:24 and half the width less
  seven of those blocks less one into bits 23:16, with a constant 4 low;

  0x2e68 is ceil(width/320), and 0x2e74 carries the pad that would square the
  width to a multiple of 20 in bits 13:8 plus a channel bit at 0. That bit is
  set at 0x1c5930 and cleared at 0x1c5ad8, and it clears when the module's
  flags at +824 and +992 both read zero: the single-channel path every
  register here was derived from, and the one the vendor streams on;

  0x2e78 and 0x2e7c are flag bits, the second one bit 0 of the work mode.

The masks matter. Several of these share a word with the submodule's static
image and the vendor read-modify-writes, so the pass owns some bits of a
register and the image owns the rest. 0x2e24 is the clearest case: only bits
28:16 and bit 31 are the vendor's here, and its low field stays at the image's
0x2d.

This script mirrors the header's integer arithmetic exactly and refuses to
pass unless every one of the thirteen reproduces the live vendor bank, masked
to the bits this pass owns.

    kernel/scripts/isp/check-de3d-geometry.py
"""

import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
DRIVERS = HERE.parent.parent / 'overlay' / 'drivers' / 'media' / 'artosyn'
GEOM = DRIVERS / 'ar-isp-de3d-geom.h'
LIBRARY = DRIVERS / 'vendor-tables' / 'ar-isp-library.h'

BANK = 0x2E00
REGS_H = DRIVERS / 'ar-isp-regs.h'
SENSOR = DRIVERS / 'nt99235.c'

# The vendor's own allocation gaps, from the three addresses it packs at
# 0x2b439200, 0x2b614200 and 0x2b703200. Only the first two are bounded: nothing
# sits above the third, so its gap says nothing.
GAP = {'BUF0': 0x1DB000, 'BUF1': 0xEF000, 'BUF2': None}
PAGE = 0x1000

# The operating point the vendor bank was captured at: 1920 wide plus the
# spatial filter's eight-pixel border, 1080 high, and the 1080p60 line length
# of 2200 (148.5 MHz / 60 / 1125). Work mode 1.
WIDTH, HEIGHT, HTS, MODE = 1928, 1080, 2200, 1

# The vendor's live bank, from the streaming slot A capture in
# out/au-chain/slotA.txt. Kept here so the check needs no capture on disk.
LIVE = {
    0x2E24: 0x01E5002D, 0x2E38: 0x10010700, 0x2E40: 0x80110700,
    0x2E48: 0x01E50032, 0x2E54: 0x78740004, 0x2E5C: 0x00000700,
    0x2E64: 0x00000700, 0x2E68: 0x00000007, 0x2E74: 0x00000C00,
    0x2E78: 0x8282A6A6, 0x2E7C: 0x00000001, 0x2E84: 0x00000100,
    0x2E8C: 0x00000100,
}


def ceil_div(a: int, b: int) -> int:
    return -(-a // b)


def align256(v: int) -> int:
    return (v + 0xFF) & ~0xFF


def pack(width: int, height: int, hts: int, mode: int) -> list[int]:
    """Mirror ar_isp_de3d_geom_pack word for word."""
    del height                      # see the header: taken, not used

    tiles = ceil_div(width, 20)
    stride = align256(tiles * 16)
    stride2 = align256(ceil_div(width, 8))

    blank = hts - width
    delay = blank + (blank * 5) // 100 + 200
    blocks = ceil_div(width, 16)

    return [
        delay << 16,
        (1 << 28) | (1 << 16) | (stride & 0xFFFF),
        (1 << 31) | (1 << 20) | (1 << 16) | (stride & 0xFFFF),
        (delay << 16) | 50,
        ((blocks - 1) << 24) | (((width // 2) - 7 * blocks - 1) << 16) | 4,
        stride,
        stride,
        ceil_div(width, 320),
        ((20 - (width % 20)) % 20) << 8,
        (1 << 31) | (1 << 23),
        mode & 1,
        stride2,
        stride2,
    ]


def header_regs() -> list[tuple[int, int]]:
    """The (offset, mask) table the driver compiles, read from the header."""
    body = re.search(r'ar_isp_de3d_geom_regs\[\]\s*=\s*\{(.*?)\n\};',
                     GEOM.read_text(), re.S)
    if not body:
        sys.exit(f'{GEOM.name}: no ar_isp_de3d_geom_regs table')

    return [(int(o, 16), int(m, 16)) for o, m in
            re.findall(r'\{\s*(0x[0-9a-f]+),\s*(0x[0-9a-f]+)\s*\}', body.group(1))]


def image() -> dict[int, int]:
    """The de3d submodule's static image, for the bits this pass does not own."""
    body = re.search(r'/\* de3d:.*?\n(.*?)\n\t/\*', LIBRARY.read_text(), re.S)
    if not body:
        sys.exit(f'{LIBRARY.name}: no de3d image')

    return {int(o, 16): int(v, 16) for o, v in
            re.findall(r'\{\s*(0x[0-9a-f]+),\s*(0x[0-9a-f]+)\s*\}', body.group(1))}


def main() -> int:
    regs = header_regs()
    values = pack(WIDTH, HEIGHT, HTS, MODE)
    if len(regs) != len(values):
        sys.exit(f'{len(regs)} registers in the header, {len(values)} packed')

    img = image()
    bad = 0
    print(f'width {WIDTH}, height {HEIGHT}, hts {HTS}, mode {MODE}\n')
    print(f'{"reg":8s} {"mask":>10s} {"derived":>10s} {"vendor":>10s}')
    for (off, mask), value in zip(regs, values, strict=True):
        reg = BANK + off
        if reg not in LIVE:
            sys.exit(f'{reg:#06x} is in the header but has no live value')

        # The bits this pass does not own must come from the static image, or
        # the comparison is not against a register the driver can produce.
        want = LIVE[reg]
        got = (img.get(reg, 0) & ~mask) | (value & mask)
        if got != want:
            bad += 1
            print(f'{reg:#06x} {mask:#010x} {got:#010x} {want:#010x}  MISMATCH')
        else:
            print(f'{reg:#06x} {mask:#010x} {got:#010x} {want:#010x}')

    if bad:
        sys.exit(f'\n{bad} of {len(regs)} do not reproduce the vendor')

    print(f'\nall {len(regs)} de3d geometry registers reproduce the streaming '
          f'vendor bit-exactly')

    return check_sizes() or check_line_length()


def check_line_length() -> int:
    """Every sensor mode must program the line length the delay is built on."""
    hit = re.search(r'AR_ISP_DE3D_LINE_LENGTH_PCK\s+(\d+)', GEOM.read_text())
    if not hit:
        sys.exit(f'{GEOM.name}: no AR_ISP_DE3D_LINE_LENGTH_PCK')

    want = int(hit.group(1))
    src = SENSOR.read_text()
    modes = re.findall(r'nt99235_reg (nt99235_regs_\w+)\[\] = \{(.*?)\n\};',
                       src, re.S)
    if not modes:
        sys.exit(f'{SENSOR.name}: no mode tables found')

    bad = 0
    print()
    for name, body in modes:
        regs = {int(r, 16): int(v, 16) for r, v in
                re.findall(r'\{\s*(0x[0-9a-f]+),\s*(0x[0-9a-f]+)\s*\}', body)}
        got = (regs.get(0x0342, 0) << 8) | regs.get(0x0343, 0)
        if got != want:
            bad += 1
            print(f'{name}: LINE_LENGTH_PCK {got}, expected {want}  MISMATCH')
        else:
            print(f'{name}: LINE_LENGTH_PCK {got}')

    if bad:
        sys.exit(f'\n{bad} sensor modes program a line length the de3d delay '
                 f'register was not derived for')

    print(f'\nall {len(modes)} sensor modes program LINE_LENGTH_PCK {want}, '
          f'so the de3d line delay holds for every one')

    return 0


def check_sizes() -> int:
    """The three working-buffer sizes, from the same two strides."""
    stride = align256(ceil_div(WIDTH, 20) * 16)
    stride2 = align256(ceil_div(WIDTH, 8))
    want = {
        'BUF0': stride * HEIGHT,
        'BUF1': stride * HEIGHT // 2,
        'BUF2': stride2 * (ceil_div(HEIGHT, 8) + 2),
    }

    header = REGS_H.read_text()
    bad = 0
    print()
    for name, size in want.items():
        rounded = (size + PAGE - 1) & ~(PAGE - 1)
        hit = re.search(rf'AR_ISP_DE3D_{name}_SIZE\s+(0x[0-9a-f]+)', header)
        if not hit:
            sys.exit(f'ar-isp-regs.h: no AR_ISP_DE3D_{name}_SIZE')

        carried = int(hit.group(1), 16)
        note = ''
        gap = GAP[name]
        if gap is not None:
            # The derived size has to fit the vendor's own layout, and so does
            # the rounded-up constant the driver allocates.
            if size > gap or carried > gap:
                bad += 1
                note = f'  OVERRUNS the {gap:#08x} gap'
            else:
                note = f'  fits the {gap:#08x} gap, {gap - size} B spare'

        if carried != rounded:
            bad += 1
            note += f'  header carries {carried:#08x}, expected {rounded:#08x}'

        print(f'{name}: derived {size:#08x} -> page {rounded:#08x}{note}')

    if bad:
        sys.exit(f'\n{bad} buffer-size problems')

    print('\nall three de3d buffer sizes derive from the programmed strides')

    return 0


if __name__ == '__main__':
    sys.exit(main())
