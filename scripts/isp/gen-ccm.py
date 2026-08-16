#!/usr/bin/env python3
"""
Extract the Artosyn ISP's CCM init blocks and emit them as a kernel header.

The two CCM register banks (ccm1 at ISP +0x3400, ccm2 at +0x3800) are 0x50-byte
register files, not DMA pages. At init the vendor service copies entries 33 and
34 of its ISP-init template array into them verbatim: an identity matrix pair
for ccm1 and a fixed colour matrix pair for ccm2. The vendor code also has a
ccm1 runtime pack path that can write an interpolated tuning-file matrix into
its first copy.

The emitted tables come from the library alone. Nothing the tuning file holds
reaches the header.

The tuning file is read to guard a different piece of code: the driver's runtime
CCM path in ar-isp-colour.h, which is what actually consumes the blob when ccm1
is repacked on the live device. f32_q8sm() and pack_matrix() here are Python
mirrors of that packing.

That packing is checked by computing it a second, independent way: f32_q8sm()
decodes the IEEE-754 fields by hand, q8sm_reference() does the same conversion in
float arithmetic, and all four populated illuminant banks must pack identically
under both. What this pins is the rounding mode, truncation toward zero rather
than round to nearest, which is the one place the two implementations can
plausibly diverge and the one the hardware is sensitive to. The remaining checks
pin the surrounding structure: both template blocks hold two identical matrix
copies and nothing outside them, every populated matrix preserves white, the ccm1
tuning gate reads 1 and the ccm2 gate reads 0, the illuminant ladder is still
ascending kelvin, and banks 4 to 7 are still empty.

Every expectation this script asserts is derived from --lib or --blob, so a
checkout plus those two files reproduces all of them. Nothing is transcribed from
a capture artifact.

The library and tuning file are proprietary and are not in the repository.

    kernel/scripts/isp/gen-ccm.py --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so \\
        --blob out/air-gather/camera/nt99235_tuning_preview_fpv.bin \\
        > overlay/drivers/media/artosyn/vendor-tables/ar-isp-ccm-init.h
"""

import argparse
import struct
import sys
from collections.abc import Sequence

import arlib

# Entries of the ISP-init template array (arlib.TEMPLATE_ARRAY_VMA).
CCM1_ENTRY = 33
CCM2_ENTRY = 34
BLOCK = 0x50

# A packed matrix is six words: two 16-bit coefficients in each even word, one in each odd.
MATRIX_WORDS = 6
MATRIX_ROWS = 3
MATRIX_COLS = 3
MATRIX_COEFFS = MATRIX_ROWS * MATRIX_COLS

# The odd words of a packed matrix, whose upper halves carry no coefficient.
MATRIX_ODD_WORDS = (1, 3, 5)

# Both copies of the matrix live in one BLOCK, the second COPY_WORDS words after the first.
COPY_WORDS = 8

# Words of a BLOCK that lie outside the two matrix copies.
PAD_WORDS = (6, 7, 14, 15, 16, 17, 18, 19)

# Emitted table width.
ROWS_PER_LINE = 4

# IEEE-754 binary32 field layout, decoded by hand because the value is packed, not converted.
F32_MANT_BITS = 23
F32_MANT_MASK = (1 << F32_MANT_BITS) - 1
F32_MANT_IMPLICIT = 1 << F32_MANT_BITS
F32_EXP_MASK = 0xFF
F32_EXP_BIAS = 127
F32_SIGN = 1 << 31
F32_ABS_MASK = F32_SIGN - 1
F32_ONE = 0x3F800000

# Coefficient format: Q8 magnitude in the low 15 bits, sign in bit 15. The mantissa carries
# F32_MANT_BITS fraction bits and the target carries Q8_FRAC_BITS, so the right shift for an
# exponent of zero is the difference.
Q8_FRAC_BITS = 8
Q8_SHIFT_AT_EXP0 = F32_MANT_BITS - Q8_FRAC_BITS
Q8_MAG_MAX = 0x7FFF
Q8_SIGN = 0x8000
Q8_ONE = 1 << Q8_FRAC_BITS

GATE_CCM1 = 0x253FC
GATE_CCM2 = 0x2595C
LADDER = 0x25438
ILLUMINANTS = 8
BANKS = 0x25470
BANK_STRIDE = 0x24
BANKS_USED = 4

# Kelvin bounds the illuminant ladder has to stay inside to still be a colour-temperature
# sequence rather than a misread offset.
KELVIN_MIN = 1000
KELVIN_MAX = 20000


def f32_q8sm(bits: int) -> int:
    """Mirror ar_isp_f32_q8sm: Q8 truncated toward zero, sign-magnitude."""
    mant = (bits & F32_MANT_MASK) | F32_MANT_IMPLICIT
    exp = ((bits >> F32_MANT_BITS) & F32_EXP_MASK) - F32_EXP_BIAS
    shift = Q8_SHIFT_AT_EXP0 - exp

    if not bits & F32_ABS_MASK:
        return 0

    if shift >= 32:
        mag = 0
    elif shift <= 0:
        mag = Q8_MAG_MAX
    else:
        mag = mant >> shift

    mag = min(mag, Q8_MAG_MAX)
    return (Q8_SIGN | mag) if bits & F32_SIGN else mag


def q8sm_reference(value: float) -> int:
    """
    Q8 sign-magnitude truncation, computed in float arithmetic.

    A second route to f32_q8sm's result that shares none of its code: that one decodes the
    IEEE-754 exponent and mantissa by hand and shifts, this one multiplies and truncates.
    Requiring the two to agree is what pins the rounding mode.
    """
    mag = min(int(abs(value) * Q8_ONE), Q8_MAG_MAX)
    return (Q8_SIGN | mag) if value < 0 else mag


def pack_matrix(bits: Sequence[int]) -> tuple[int, ...]:
    """Mirror ar_isp_ccm_pack: six words, two coefficients per even word."""
    coeff = [f32_q8sm(f32) for f32 in bits]
    return (coeff[0] | coeff[1] << 16, coeff[2],
            coeff[3] | coeff[4] << 16, coeff[5],
            coeff[6] | coeff[7] << 16, coeff[8])


def unpack_matrix(words: Sequence[int]) -> tuple[int, ...]:
    """Inverse of pack_matrix: nine signed Q8 coefficients, row major."""
    codes = []
    for i in range(0, MATRIX_WORDS, 2):
        codes += [words[i] & 0xFFFF, words[i] >> 16, words[i + 1] & 0xFFFF]

    return tuple(-(code & Q8_MAG_MAX) if code & Q8_SIGN else code for code in codes)


def check_matrix_shape(words: Sequence[int], what: str) -> None:
    """A BLOCK holds two identical matrix copies and nothing else."""
    if words[0:COPY_WORDS] != words[COPY_WORDS:2 * COPY_WORDS]:
        sys.exit(f"{what} copies differ")

    if any(words[i] for i in PAD_WORDS):
        sys.exit(f"{what} has data outside the two matrix copies")

    if any(words[i] >> 16 for i in MATRIX_ODD_WORDS):
        sys.exit(f"{what} has data above the coefficient in an odd word")


def check_white_preserving(coeff: Sequence[int], what: str) -> None:
    """
    A colour matrix maps white to white, so each row sums to unity.

    Compared in Q8, where the sum carries the row's three truncations. Each moves its
    coefficient toward zero by less than one LSB, so a row lands within MATRIX_COLS of
    Q8_ONE in whichever direction the row's signs push it.
    """
    for row in range(MATRIX_ROWS):
        total = sum(coeff[row * MATRIX_COLS:(row + 1) * MATRIX_COLS])
        if abs(total - Q8_ONE) > MATRIX_COLS:
            sys.exit(f"{what} row {row} sums to {total}/{Q8_ONE}, which is not unity")


def check_ccm1_template(words: Sequence[int]) -> None:
    identity = pack_matrix([F32_ONE, 0, 0, 0, F32_ONE, 0, 0, 0, F32_ONE])
    if (tuple(words[:MATRIX_WORDS]) != identity or
            tuple(words[COPY_WORDS:COPY_WORDS + MATRIX_WORDS]) != identity):
        sys.exit("ccm1 template is not an identity pair under the recovered layout")

    check_matrix_shape(words, "ccm1 template")


def check_ccm2_template(words: Sequence[int]) -> None:
    check_matrix_shape(words, "ccm2 template")
    check_white_preserving(unpack_matrix(words[:MATRIX_WORDS]), "ccm2 template")


def check_blob(blob: bytes) -> None:
    gate1 = struct.unpack_from("<I", blob, GATE_CCM1)[0]
    if gate1 != 1:
        sys.exit(f"ccm1 gate at 0x{GATE_CCM1:x} reads {gate1}, expected 1")

    gate2 = struct.unpack_from("<I", blob, GATE_CCM2)[0]
    if gate2 != 0:
        sys.exit(f"ccm2 gate at 0x{GATE_CCM2:x} reads {gate2}, expected 0")

    ladder = struct.unpack_from(f"<{ILLUMINANTS}f", blob, LADDER)
    if list(ladder) != sorted(ladder) or not KELVIN_MIN < ladder[0] < ladder[-1] < KELVIN_MAX:
        sys.exit("illuminant ladder is not an ascending kelvin sequence")

    for bank in range(BANKS_USED, ILLUMINANTS):
        off = BANKS + bank * BANK_STRIDE
        if any(blob[off:off + BANK_STRIDE]):
            sys.exit(f"matrix bank {bank} expected empty, has data")

    for bank in range(BANKS_USED):
        off = BANKS + bank * BANK_STRIDE
        bits = struct.unpack_from(f"<{MATRIX_COEFFS}I", blob, off)
        values = struct.unpack_from(f"<{MATRIX_COEFFS}f", blob, off)

        if tuple(f32_q8sm(b) for b in bits) != tuple(q8sm_reference(v) for v in values):
            sys.exit(f"matrix bank {bank}: f32_q8sm and q8sm_reference disagree")

        check_white_preserving(unpack_matrix(pack_matrix(bits)), f"matrix bank {bank}")


def emit(ccm1: Sequence[int], ccm2: Sequence[int]) -> None:
    guard_open, guard_close = arlib.guard("AR_ISP_CCM_INIT_H")
    print(arlib.banner("kernel/scripts/isp/gen-ccm.py", (
        "CCM register-bank init blocks, 20 words each: ccm1 at ISP +0x3400 (an",
        "identity matrix pair) and ccm2 at +0x3800 (the vendor's fixed matrix",
        "pair), lifted verbatim from the vendor service's ISP-init template.",
        "The ccm2 block is word-identical to the traced init writes; the ccm1",
        "block decodes to an identity under the packing in ar-isp-colour.h.",
    )), end="")
    print()
    print(guard_open, end="")
    for name, words in (("ar_isp_ccm1_init", ccm1), ("ar_isp_ccm2_init", ccm2)):
        print()
        print(f"static const u32 {name}[{len(words)}] = {{")
        print(arlib.rows(words, ROWS_PER_LINE, "#010x"), end="")
        print("};")

    print()
    print(guard_close, end="")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--lib", required=True, help="vendor libmpp_service.so")
    ap.add_argument("--blob", required=True, help="vendor tuning file")
    args = ap.parse_args()

    with open(args.lib, "rb") as handle:
        lib = handle.read()

    with open(args.blob, "rb") as handle:
        blob = handle.read()

    ccm1 = struct.unpack(f"<{BLOCK // 4}I",
                         arlib.template_payload(lib, CCM1_ENTRY, BLOCK))
    ccm2 = struct.unpack(f"<{BLOCK // 4}I",
                         arlib.template_payload(lib, CCM2_ENTRY, BLOCK))

    check_ccm1_template(ccm1)
    check_ccm2_template(ccm2)
    check_blob(blob)

    emit(ccm1, ccm2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
