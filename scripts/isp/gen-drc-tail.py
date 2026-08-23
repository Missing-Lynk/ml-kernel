#!/usr/bin/env python3
"""
Emit the static half of the Artosyn ISP's DRC page as a kernel header.

The DRC DMA page is 0x2000 bytes of four 257-sample curves. The first two are rebuilt
from the tuning file on every AE update and are generated at runtime by ar-isp-codec.h.
The second two are never recomputed: the vendor service library carries them prepacked,
reached through the ISP-init descriptor array as {source 0x467460, length 0x2000}, and
the runtime writes only the first half of what that descriptor loaded.

Both are an exponential in the square root of the sample index, so this computes them
rather than copying the library's bytes. The functions and their constants are in
isp_curves.py; kernel/scripts/isp/check-table-closed-forms.py compares the result against
the library sample by sample, which is what makes computing them safe.

Emitted decoded, as sample arrays rather than packed bytes, because the packing is
already in ar-isp-codec.h and a curve is reviewable where a blob is not.

    kernel/scripts/isp/gen-drc-tail.py \\
        > overlay/drivers/media/artosyn/vendor-tables/ar-isp-drc-tail.h
"""

import argparse
import sys
from collections.abc import Sequence

import arlib
import isp_curves

# Emitted table width.
ROWS_PER_LINE = 6


def emit(banks: Sequence[Sequence[int]]) -> None:
    guard_open, guard_close = arlib.guard("AR_ISP_DRC_TAIL_H")
    print(arlib.banner("kernel/scripts/isp/gen-drc-tail.py", (
        "The half of the DRC page the vendor never recomputes.",
        "",
        "Two 257-sample curves. The page's other two banks are rebuilt from the tuning",
        "file on every AE update; these two appear in it in no form, and the vendor",
        "service library carries them prepacked.",
        "",
        "Both are an exponential in the square root of the normalised sample index, at",
        "one amplitude of 0x18000, exactly 1.5 in the Q16 the banks carry gains in:",
        "",
        "\tu = sqrt(i / 256)",
        "\tbank0[i] = floor(0x18000 * exp(-0.3624559 * u))",
        "\tbank1[i] = floor(0x18000 * (exp(-5.8 * u) - exp(-5.8)))",
        "",
        "The subtraction is what takes bank1 to exactly zero at full scale. Two samples",
        "lie off the curves and are carried as the library holds them: sample 0 of both",
        "banks is a clamp at 15.9375 in Q16, where a gain curve evaluated at black would",
        "run away, and bank1 sample 1 sits 283 counts above its curve for a reason that",
        "is not recovered.",
        "",
        "The same two forms generate the tuning file's own DRC profiles, with a linear",
        "abscissa where these use the square root, which is what says the shape is the",
        "vendor's arithmetic rather than a curve that happens to fit. The amplitude is",
        "pinned by the samples to 0.04 counts and bank1's rate is exactly 5.8. bank0's",
        "rate was solved from this table with no meaning found behind the number, so",
        "these curves are vendor data in compact form rather than a derivation.",
    )), end="")
    print()
    print(guard_open, end="")
    for bank, samples in enumerate(banks):
        print()
        print(f"static const u32 ar_isp_drc_tail_bank{bank}[AR_ISP_DRC_SAMPLES] = {{")
        print(arlib.rows(samples, ROWS_PER_LINE, "#07x"), end="")
        print("};")

    print()
    print(guard_close, end="")


def main() -> int:
    argparse.ArgumentParser(description=__doc__).parse_args()

    banks = [isp_curves.drc_tail_bank0(), isp_curves.drc_tail_bank1()]

    # The lanes the packer writes are 20 bits wide. The curves stay well inside that;
    # this catches an amplitude edited to something that would silently truncate.
    for bank, samples in enumerate(banks):
        for i, value in enumerate(samples):
            if not 0 <= value <= isp_curves.DRC_LANE_MAX:
                sys.exit(f"bank{bank} sample {i} is {value:#x}, outside a 20-bit lane")

    emit(banks)
    return 0


if __name__ == "__main__":
    sys.exit(main())
