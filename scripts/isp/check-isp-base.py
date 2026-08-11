#!/usr/bin/env python3
"""
Prove the ISP top-level bank's geometry words and interrupt setup.

The top-level bank has no submodule and so no static image behind it: it is
written directly by the ISP open path, `isp_input_creat` and the routine at
0x1d3560 that follows it. Those writers reach it as `g_hw_info+4`, the ISP
physical base, through `ar_dev_pa2va`. This matters because the same offsets
exist on VIF, reached as `g_hw_info+12`, and most `+0xc4`-style writers in the
library are VIF ones that have nothing to do with these registers.

Two kinds of value are checked, both against the library rather than against a
capture.

**The two frame words.** 0x0004 packs `{mode[27:26], height[25:13],
width[12:0]}`. The layout is not assumed: four read-modify-write sites build it
field by field, and their masks are the layout. At 0x1d2000:

	and w1, w1, #0xffffe000     clear bits[12:0]
	orr w1, w1, w3              width
	and w1, w1, #0xfc001fff     clear bits[25:13]
	orr w1, w1, w2, lsl #13     height

0x0004 decodes to the frame padded by four in each dimension, which is the
input the VIF measures, and 0x0008 under the same layout decodes to the active
frame exactly. Only 0x0004's layout is proven by a decoding instruction; no
pointer to `base+0x8` is ever formed, and it is committed from the register
shadow at 0x1d4670 with no attributable producer. It is checked here because
the layout that was proven on its neighbour reproduces the configured frame
exactly, and the two share an untouched top nibble.

**The literals.** The remaining values are constants the open path stores, and
each is rebuilt here from the `mov`/`movk` pair in the library that builds it,
so the check fails if the driver's table and the vendor's code drift apart.
0x00c8 and 0x00d0 are the two interrupt-enable masks, taken from the branch
`get_start_opt()->[12308]` selects; the other branch is a debug one enabling
everything.

Needs the vendor library, which is not in the tree, and aarch64 objdump:

    kernel/scripts/isp/check-isp-base.py \\
        --library out/air-gather/vendor-root/usr/lib/libmpp_service.so
"""

import argparse
import importlib.util
import pathlib
import re
import struct
import shutil
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
OBJDUMP = 'aarch64-linux-gnu-objdump'

# The frame the driver configures.
FRAME_W, FRAME_H = 1920, 1080

# 0x0004 measures the input, which carries four extra pixels in each dimension.
INPUT_PAD = 4

# The field layout of 0x0004, from the masks at 0x1d2000 and 0x1d201c.
WIDTH_BITS, HEIGHT_BITS = 13, 13
FIELD = (1 << WIDTH_BITS) - 1
MODE_SHIFT, MODE_BITS = 26, 2

# Register to the window of library code that stores its literal, and what the
# literal is. Each window is disassembled and every constant a mov/movk pair
# builds in it is collected; the driver's value has to be one of them.
LITERALS = {
    0x0090: (0x1D3A28, 0x1D3A64, 'a bare constant, with no derivation behind '
                                 'it in the open path'),
    0x00C4: (0x1D397C, 0x1D3990, 'a bare constant'),
    0x00C8: (0x1D3E0C, 0x1D3E28, 'the first interrupt-enable mask, on the '
                                 'branch get_start_opt()->[12308] selects'),
    0x00D0: (0x1D3E0C, 0x1D3E28, 'the second interrupt-enable mask, stored by '
                                 'the same branch'),
}

# Entry 49 of the ar9311 ISP-init template array, which is the top-level bank's
# own static image and covers 0x0004 to 0x0068 exactly. It is installed by an
# isp_memcpy at 0x25aa4c on the cvisp side, not by any submodule. The gamma,
# drc, raw_crop, ltm and ccm2 sites that also reference this entry only restore
# their own four-word DMA descriptor out of it before overriding, which is why
# the entry looked like it belonged to two unrelated modules.
#
# The array is addressed by VMA and the file is read by offset, so the segment
# skew has to come off first.
TEMPLATE_ARRAY = 0x48C770
TEMPLATE_ENTRY = 49
DATA_SKEW = 0x10000
IMAGE_FIRST_REG = 0x0004

# Registers the image supplies whole, and one it supplies a bit of.
FROM_IMAGE = (0x0010, 0x0014)
IMAGE_PLUS_BIT = {0x0018: (0x10000, 'raw_crop, or-ed in at 0x1a5560 and '
                                    'cleared again at 0x1a5930')}

# Written by the raw_crop enable path rather than carried by the image, which
# holds zero here. The disable path writes zero back.
RAW_CROP_CONST = {0x0068: (0x100, 0x1A5548)}

# The top-level enable word: a base value stored whole after the image install,
# then one bit per module that comes up. The base is planted into the template
# structure at runtime by the SoC accessor at 0x1f4de0, so it exists only as an
# instruction constant.
ENABLE_REG = 0x0000
ENABLE_BASE = 0x90000000
ENABLE_BITS = {
    0x00000002: 'dpc_v1, 0x1c93f4',
    0x00000010: 'decompander, 0x19dd38',
    0x00000040: 'wb, 0x1aebf8',
    0x00080000: 'raw_crop, 0x1a5604',
    0x00200000: 'ccm1, 0x187448',
    0x20000000: 'ltm_v1 and gamma, 0x194258',
}

# 0x00b8 is not a literal: the open path writes all ones, reads back, and
# clears one bit. The mask is the `and` at 0x1d3948.
READ_BACK = 0x00B8
READ_BACK_MASK = 0xFFFFBFFF
READ_BACK_SITE = (0x1D3918, 0x1D3958)


def load_audit():
    """The driver's register tables, via audit-provenance.py."""
    path = HERE / 'audit-provenance.py'
    spec = importlib.util.spec_from_file_location('ar_isp_audit', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    source = path.read_text().replace(
        "if __name__ == '__main__':\n    sys.exit(main())", '')
    exec(compile(source, str(path), 'exec'), mod.__dict__)

    return mod


def window(library, start, stop):
    """One span of disassembly."""
    if not shutil.which(OBJDUMP):
        sys.exit(f'{OBJDUMP} not found. It reads the vendor library, which is '
                 f'the only source for what these values are.')

    out = subprocess.run(
        [OBJDUMP, '-d', '--no-show-raw-insn',
         f'--start-address={start:#x}', f'--stop-address={stop:#x}',
         str(library)],
        capture_output=True, text=True)
    if out.returncode:
        sys.exit(f'{OBJDUMP} failed on {library}: {out.stderr.strip()}')

    return out.stdout.splitlines()


def constants(lines):
    """
    Every 32-bit constant a mov/movk pair builds in a span.

    A `movk` folds into whatever `mov` last targeted that register, so the two
    halves have to be tracked per register rather than per instruction.
    """
    built, seen = {}, set()
    for line in lines:
        hit = re.search(r'\bmov\s+(w\d+), #(0x[0-9a-f]+|-?\d+)', line)
        if hit:
            built[hit.group(1)] = int(hit.group(2), 0) & 0xFFFFFFFF
            seen.add(built[hit.group(1)])
            continue

        hit = re.search(r'\bmovk\s+(w\d+), #(0x[0-9a-f]+|\d+), lsl #16', line)
        if hit and hit.group(1) in built:
            built[hit.group(1)] = ((built[hit.group(1)] & 0xFFFF)
                                   | (int(hit.group(2), 0) << 16))
            seen.add(built[hit.group(1)])

    return seen


def masks(lines):
    """Every immediate an `and` applies in a span."""
    return {int(m, 0) & 0xFFFFFFFF for m in
            re.findall(r'\band\s+w\d+, w\d+, #(0x[0-9a-f]+)', '\n'.join(lines))}


def decode(value):
    return (value & FIELD, (value >> WIDTH_BITS) & FIELD,
            (value >> MODE_SHIFT) & ((1 << MODE_BITS) - 1))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--library', required=True,
                    help='the vendor libmpp_service.so')
    args = ap.parse_args()

    library = pathlib.Path(args.library)
    if not library.exists():
        sys.exit(f'{library}: not found. The vendor library is a capture '
                 f'artifact and is deliberately not in the tree.')

    audit = load_audit()
    _image, final, _origin = audit.load_tables()

    wanted = [0x0004, 0x0008, READ_BACK, *sorted(LITERALS)]
    missing = [f'{off:#06x}' for off in wanted if off not in final]
    if missing:
        sys.exit(f'the driver writes no value at {", ".join(missing)}, so '
                 f'there is nothing to prove the open path against')

    failures = []

    # The two frame words, against the frame the driver configures.
    print(f'frame words, {WIDTH_BITS} + {HEIGHT_BITS} bits and a '
          f'{MODE_BITS}-bit mode:\n')
    for off, expect_w, expect_h, what in (
            (0x0004, FRAME_W + INPUT_PAD, FRAME_H + INPUT_PAD,
             f'the input, padded by {INPUT_PAD}'),
            (0x0008, FRAME_W, FRAME_H, 'the active frame')):
        value = final[off]
        width, height, mode = decode(value)
        print(f'  {off:#06x} = {value:#010x}  {width} x {height}, mode {mode}, '
              f'top nibble {value >> 28:#x}  {what}')
        if (width, height) != (expect_w, expect_h):
            failures.append(
                f'{off:#06x}: decodes to {width} x {height} where the '
                f'configured frame gives {expect_w} x {expect_h}, so the field '
                f'layout does not hold at this frame size')

    if (final[0x0004] >> 28) != (final[0x0008] >> 28):
        failures.append(
            f'{0x0004:#06x} and {0x0008:#06x} no longer share a top nibble, '
            f'which is what carried the layout from the proven word to the '
            f'unproven one')

    # The field layout itself, from the masks of the site that builds it.
    packing = masks(window(library, 0x1D1FDC, 0x1D2028))
    for mask, what in ((0xFFFFFFFF ^ FIELD, 'the width field'),
                       (0xFFFFFFFF ^ (FIELD << WIDTH_BITS), 'the height field'),
                       (0xFFFFFFFF ^ (((1 << MODE_BITS) - 1) << MODE_SHIFT),
                        'the mode field')):
        if mask not in packing:
            failures.append(
                f'the packing site clears no {mask:#010x}, so {what} is not '
                f'where this script places it')

    print(f'\n  the packing site at 0x1d1fdc clears {len(packing)} field(s), '
          f'and all three this layout needs are among them')

    # The literals, rebuilt from the library's own mov/movk pairs.
    print('\nconstants the open path stores, rebuilt from the library:\n')
    for off in sorted(LITERALS):
        start, stop, what = LITERALS[off]
        value = final[off]
        print(f'  {off:#06x} = {value:#010x}  built at {start:#x}  {what}')
        if value not in constants(window(library, start, stop)):
            failures.append(
                f'{off:#06x}: no mov/movk pair in {start:#x}..{stop:#x} builds '
                f'{value:#010x}, so the driver\'s value is not the one the '
                f'vendor\'s open path stores')

    # The read-back register, which is all ones with one bit cleared.
    value = final[READ_BACK]
    print(f'\n  {READ_BACK:#06x} = {value:#010x}  all ones with bit '
          f'{(~READ_BACK_MASK & 0xFFFFFFFF).bit_length() - 1} cleared, written '
          f'then read back at {READ_BACK_SITE[0]:#x}')
    if value != READ_BACK_MASK:
        failures.append(
            f'{READ_BACK:#06x}: the driver installs {value:#010x} where the '
            f'open path\'s read-modify-write gives {READ_BACK_MASK:#010x}')
    elif READ_BACK_MASK not in masks(window(library, *READ_BACK_SITE)):
        failures.append(
            f'{READ_BACK:#06x}: the site at {READ_BACK_SITE[0]:#x} no longer '
            f'applies {READ_BACK_MASK:#010x}')

    # The bank's own static image, template entry 49.
    raw = library.read_bytes()
    descriptor = TEMPLATE_ARRAY - DATA_SKEW + TEMPLATE_ENTRY * 16
    pointer, length = struct.unpack_from('<QQ', raw, descriptor)
    image_at = pointer - DATA_SKEW
    last = IMAGE_FIRST_REG + length - 4
    print(f'\ntemplate entry {TEMPLATE_ENTRY}: {length} bytes at {pointer:#x}, '
          f'covering {IMAGE_FIRST_REG:#06x} to {last:#06x}, installed by the '
          f'isp_memcpy at 0x25aa4c\n')

    def image(off):
        return struct.unpack_from('<I', raw, image_at + off - IMAGE_FIRST_REG)[0]

    for off in FROM_IMAGE:
        want, got = image(off), final[off]
        print(f'  {off:#06x} = {got:#010x}  the image word verbatim')
        if want != got:
            failures.append(f'{off:#06x}: the image carries {want:#010x} and '
                            f'the driver installs {got:#010x}')

    for off, (bit, who) in IMAGE_PLUS_BIT.items():
        want, got = image(off) | bit, final[off]
        print(f'  {off:#06x} = {got:#010x}  the image {image(off):#010x} with '
              f'{bit:#x} from {who}')
        if want != got:
            failures.append(f'{off:#06x}: the image with {bit:#x} gives '
                            f'{want:#010x} and the driver installs {got:#010x}')

    for off, (value, where) in RAW_CROP_CONST.items():
        got = final[off]
        print(f'  {off:#06x} = {got:#010x}  a constant from the raw_crop '
              f'enable path at {where:#x}; the image holds {image(off):#010x}')
        if got != value:
            failures.append(f'{off:#06x}: the enable path stores {value:#x} '
                            f'and the driver installs {got:#010x}')
        elif value not in constants(window(library, where, where + 8)):
            failures.append(f'no mov at {where:#x} builds {value:#x}, so the '
                            f'enable path is not the source of this value')

    # The enable word, bit by bit.
    got = final[ENABLE_REG]
    accumulated = ENABLE_BASE
    print(f'\n  {ENABLE_REG:#06x}  {ENABLE_BASE:#010x}  the base word, stored '
          f'whole at 0x25aa58')
    for bit, who in ENABLE_BITS.items():
        accumulated |= bit
        print(f'          | {bit:#010x}  {who}')

    print(f'          = {accumulated:#010x}  driver {got:#010x}')
    if accumulated != got:
        failures.append(
            f'{ENABLE_REG:#06x}: the base word and the six module bits give '
            f'{accumulated:#010x} and the driver installs {got:#010x}, so the '
            f'set of stages that came up is not the one this lists')

    if failures:
        print()
        for line in failures:
            print(f'FAIL: {line}')

        return 1

    print('\nboth frame words decode to the configured frame under the layout '
          'the packing site builds, every constant is the one the vendor open '
          'path stores, and the enable word is its base plus one bit per stage '
          'that came up')

    return 0


if __name__ == '__main__':
    sys.exit(main())
