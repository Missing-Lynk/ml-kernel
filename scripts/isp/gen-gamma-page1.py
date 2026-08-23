#!/usr/bin/env python3
"""
Emit the ISP gamma table's second page as a kernel header.

The gamma allocation's first 0x1000 bytes are the only part the hardware fetches, and they
are two 0x800 pages. Page 0 is AE-selected out of the tuning file and is generated at
runtime by ar-isp-codec.h. Page 1 is in no tuning file in any form: the vendor's service
library carries it prepacked, and the dynamic gamma handler copies that constant into its
working allocation before the packer overwrites page 0 and leaves page 1 alone.

That constant is the Rec.709 transfer function at twelve bits, so this computes it rather
than copying the library's bytes. The function and its constants are in isp_curves.py;
kernel/scripts/isp/check-table-closed-forms.py compares the result against the library
sample by sample, which is what makes computing it safe.

Emitted decoded, as a sample array rather than packed bytes, because the packing already
lives in ar-isp-codec.h and a curve is reviewable where a blob is not.

    kernel/scripts/isp/gen-gamma-page1.py \\
        > overlay/drivers/media/artosyn/vendor-tables/ar-isp-gamma-page1.h
"""

import argparse
import sys
from collections.abc import Sequence

import arlib
import isp_curves

# Emitted table width.
ROWS_PER_LINE = 8


def emit(samples: Sequence[int], tail: int) -> None:
    guard_open, guard_close = arlib.guard("AR_ISP_GAMMA_PAGE1_H")
    print(arlib.banner("kernel/scripts/isp/gen-gamma-page1.py", (
        "The gamma table's second page.",
        "",
        "Page 0 is AE-selected from the tuning file and generated at runtime. This one is",
        "in no tuning file in any form; the vendor service library carries it prepacked,",
        "byte-identical to page 1 of every capture on file.",
        "",
        "It is the Rec.709 transfer function with its 0.45 exponent replaced by 1/2.2,",
        "sampled at the page's own 512 points and truncated to twelve bits:",
        "",
        "\tf(x) = 4.5 * x                       for x < 0.018",
        "\t     = 1.099 * x^(1/2.2) - 0.099     otherwise",
        "",
        "\tsample[i] = floor(4095 * f(i / 512))",
        "",
        "That reproduces all 512 samples of the library's copy. The toe covers the first",
        "ten. Only the exponent departs from stock Rec.709, so check that before assuming",
        "any other curve behaves the same way.",
        "",
        "AR_ISP_GAMMA_PAGE1_TAIL is the same function one step past the page, at x = 1.",
        "The last record mirrors it forward and the hardware reads it there, so",
        "ar_isp_gamma_pack_page writes it.",
    )), end="")
    print()
    print(guard_open, end="")
    print()
    print(f"#define AR_ISP_GAMMA_PAGE1_TAIL\t\t{tail}")
    print()
    print("static const u16 ar_isp_gamma_page1[AR_ISP_GAMMA_SAMPLES] = {")
    print(arlib.rows(samples, ROWS_PER_LINE, "4d"), end="")
    print("};")
    print()
    print(guard_close, end="")


def main() -> int:
    argparse.ArgumentParser(description=__doc__).parse_args()

    samples = isp_curves.gamma_page1_samples()

    # The hardware reads a transfer curve, so a sample below its predecessor inverts the
    # response over that interval. The function is monotonic; this catches a constant
    # edited to something that is not.
    for i in range(1, len(samples)):
        if samples[i] < samples[i - 1]:
            sys.exit(f"sample {i} decreases; this is not a transfer curve")

    emit(samples, isp_curves.gamma_page1_tail())
    return 0


if __name__ == "__main__":
    sys.exit(main())
