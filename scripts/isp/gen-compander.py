#!/usr/bin/env python3
"""
Extract the ISP compander table from the vendor library as a kernel header.

The compander is the ISP's front-end stage, first in the descriptor order at isp +0x0020.
Its 0x7800 page is not generated at runtime and is not in the tuning file: it is installed
verbatim at ISP init from a static template in the service library. The template is entry 6
of the descriptor array at VMA 0x472600, a list of {u64 source, u64 length} pairs, and its
body at VMA 0x46a3b0 is byte-identical to the page captured off a streaming vendor unit and
to the page resident in DRAM on a RAM-booted unit; the tuning file never feeds it.

Most of the page is structure rather than data: three quarters is one repeated 16-byte
unity record (0x100 / 0x10000) and a further 0x700 bytes are zero. Only 0x900 bytes at the
start and 0x800 at 0x1000 carry content, so those are what this emits;
ar_isp_compander_fill in ar-isp-codec.h rebuilds the rest, and the reconstruction is
checked against the original before emitting.

The library is proprietary and not in the repository; supply it with --lib. The generated
header is checked in, so this script is rerun only if the vendor library changes.

    kernel/scripts/isp/gen-compander.py \\
        --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so \\
        > overlay/drivers/media/artosyn/vendor-tables/ar-isp-compander.h
"""

import argparse
import struct
import sys

import arlib

# Entry of the ISP-init template array (arlib.TEMPLATE_ARRAY_VMA).
COMPANDER_ENTRY = 6

SIZE = 0x7800
HEAD_LEN = 0x900
ZERO_OFF, ZERO_LEN = 0x900, 0x700
MID_OFF, MID_LEN = 0x1000, 0x800
FILL_OFF = 0x1800
FILL = bytes.fromhex("00010000000001000000000000000000")


# Emitted table width.
ROWS_PER_LINE = 6

def rebuild(head: bytes, mid: bytes) -> bytes:
    """The reconstruction ar_isp_compander_fill performs, for checking against the original."""
    out = bytearray(SIZE)
    out[0:HEAD_LEN] = head
    out[MID_OFF:MID_OFF + MID_LEN] = mid
    for off in range(FILL_OFF, SIZE, len(FILL)):
        out[off:off + len(FILL)] = FILL

    return bytes(out)


def emit(head: bytes, mid: bytes) -> None:
    guard_open, guard_close = arlib.guard("AR_ISP_COMPANDER_H")
    print(arlib.banner("kernel/scripts/isp/gen-compander.py", (
        "The ISP compander page, carried from the vendor service library.",
        "",
        "Installed verbatim at ISP init from entry 6 of the descriptor array at VMA",
        "0x472600, so unlike gamma and DRC it has no tuning-file source and no runtime",
        "generator. Byte-identical to the page captured off a streaming vendor unit.",
        "",
        "Only the two regions that carry content are here. The zero span at 0x900 and",
        "the repeated unity record from 0x1800 are rebuilt by ar_isp_compander_fill.",
    )), end="")
    print()
    print(guard_open, end="")
    for name, body in (("head", head), ("mid", mid)):
        words = struct.unpack(f"<{len(body) // 4}I", body)
        print()
        print(f"static const u32 ar_isp_compander_{name}[{len(words)}] = {{")
        print(arlib.rows(words, ROWS_PER_LINE, "#010x"), end="")
        print("};")

    print()
    print(guard_close, end="")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--lib", required=True, help="vendor libmpp_service.so")

    args = ap.parse_args()
    with open(args.lib, "rb") as handle:
        lib = handle.read()

    page = arlib.template_payload(lib, COMPANDER_ENTRY, SIZE, args.lib)

    # The reconstruction is only correct if the spans it does not carry really are
    # constant, so check them.
    if any(page[ZERO_OFF:ZERO_OFF + ZERO_LEN]):
        sys.exit(f"{args.lib}: 0x{ZERO_OFF:x} is not zero; the layout has changed")

    for pos in range(FILL_OFF, SIZE, len(FILL)):
        if page[pos:pos + len(FILL)] != FILL:
            sys.exit(f"{args.lib}: record at 0x{pos:x} is not the unity fill; "
                     "the layout has changed")

    head = page[0:HEAD_LEN]
    mid = page[MID_OFF:MID_OFF + MID_LEN]
    if rebuild(head, mid) != page:
        sys.exit(f"{args.lib}: the reconstruction does not reproduce the page")

    emit(head, mid)
    return 0


if __name__ == "__main__":
    sys.exit(main())
