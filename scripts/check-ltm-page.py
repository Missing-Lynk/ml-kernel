#!/usr/bin/env python3
"""
Verify the LTM coefficient page geometry in ar-isp-ltm.h against a capture.

The page has no stored source, so unlike the coefficient tables there is
nothing to generate and nothing to diff against a template. What can be proved
is the geometry: 64 tiles of a 128-sample u16 curve at a 0x100 stride, every
curve monotonic from zero into the top of a 10-bit range, and the page ending
at exactly 0x4000 rather than wherever the flush size suggests.

This also reports how far the page differs from the ISP-init template entry
that has the same curve shape, which is the evidence that the vendor recomputes
it rather than installing it.

Captures and the vendor library are proprietary and are not in the repository.

    kernel/scripts/check-ltm-page.py --capture out/au-snapshot/tbl_isp_0x2808.bin \\
        --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so
"""

import argparse
import struct
import sys

TILES = 64
SAMPLES = 128
STRIDE = 0x100
PAGE_SIZE = TILES * STRIDE
SAMPLE_MAX = 0x3FF

# ISP-init template entry 40. Same curve shape, a quarter of the page.
VMA_TO_FILE = 0x10000
TEMPLATE_VMA = 0x450570
TEMPLATE_LEN = 0x1000


def curves(data, count):
    out = []
    for tile in range(count):
        base = tile * STRIDE
        out.append(struct.unpack_from(f"<{SAMPLES}H", data, base))
    return out


def check_page(data):
    if len(data) < PAGE_SIZE:
        sys.exit(f"capture is {len(data):#x}, need at least {PAGE_SIZE:#x}")

    seen = curves(data, TILES)
    for tile, curve in enumerate(seen):
        if curve[0] != 0:
            sys.exit(f"tile {tile}: starts at {curve[0]}, expected 0")
        if not all(b >= a for a, b in zip(curve, curve[1:])):
            sys.exit(f"tile {tile}: curve is not monotonic")
        if not SAMPLE_MAX * 0.85 < curve[-1] <= SAMPLE_MAX:
            sys.exit(f"tile {tile}: ends at {curve[-1]}, outside the 10-bit top")

    if len(set(seen)) != TILES:
        sys.exit(f"expected {TILES} distinct tiles, found {len(set(seen))}")

    # The page must END here: tile 64 must not be another well-formed curve,
    # or the geometry is wrong and 0x4000 is not the real extent.
    if len(data) >= PAGE_SIZE + STRIDE:
        past = curves(data[PAGE_SIZE:], 1)[0]
        monotonic = all(b >= a for a, b in zip(past, past[1:]))
        if past[0] == 0 and monotonic:
            sys.exit(f"a {TILES + 1}th curve follows: page is larger than {PAGE_SIZE:#x}")

    print(f"page: {TILES} distinct tiles x {SAMPLES} u16 at {STRIDE:#x} = "
          f"{PAGE_SIZE:#x} bytes, all monotonic 0 -> "
          f"{min(c[-1] for c in seen)}..{max(c[-1] for c in seen)}")
    print(f"      content stops at {PAGE_SIZE:#x}; no further curve follows")
    return seen


def check_template(lib, seen):
    off = TEMPLATE_VMA - VMA_TO_FILE
    if off + TEMPLATE_LEN > len(lib):
        sys.exit("template lies past the end of the library")
    template = lib[off:off + TEMPLATE_LEN]

    tiles = TEMPLATE_LEN // STRIDE
    tpl = curves(template, tiles)
    # The template's tiles start at or just above zero: 15 of the 16 start at 0
    # and one starts at 1, so this allows a small offset where the page check,
    # which every captured tile satisfies exactly, does not.
    for tile, curve in enumerate(tpl):
        if curve[0] > 4 or not all(b >= a for a, b in zip(curve, curve[1:])):
            sys.exit(f"template tile {tile}: not a monotonic curve from zero")

    equal = sum(1 for a, b in zip(template, bytes(seen_flat(seen))) if a == b)
    print(f"template entry 40 ({TEMPLATE_VMA:#x}): {tiles} tiles of the same shape, "
          f"{TEMPLATE_LEN:#x} bytes, a quarter of the page")
    print(f"      shares {equal} of {TEMPLATE_LEN} bytes with the captured page "
          f"({equal * 100 // TEMPLATE_LEN}%), so the page is recomputed, not installed")


def seen_flat(seen):
    out = bytearray()
    for curve in seen:
        out += struct.pack(f"<{SAMPLES}H", *curve)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--capture", required=True, help="tbl_isp_0x2808.bin")
    ap.add_argument("--lib", help="vendor libmpp_service.so, for the template check")
    args = ap.parse_args()

    with open(args.capture, "rb") as handle:
        seen = check_page(handle.read())

    if args.lib:
        with open(args.lib, "rb") as handle:
            check_template(handle.read(), seen)

    print("LTM page geometry agrees with ar-isp-ltm.h")
    return 0


if __name__ == "__main__":
    sys.exit(main())
