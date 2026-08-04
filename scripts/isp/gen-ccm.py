#!/usr/bin/env python3
"""
Extract the Artosyn ISP's CCM init blocks and emit them as a kernel header.

The two CCM register banks (ccm1 at ISP +0x3400, ccm2 at +0x3800) are 0x50-byte
register files, not DMA pages. At init the vendor service copies entries 33 and
34 of its ISP-init template array into them verbatim: an identity matrix pair
for ccm1 and a fixed colour matrix pair for ccm2. At runtime only ccm1 moves,
when the AWB path packs an interpolated tuning-file matrix into its first copy.

The emitted tables come from the library alone. Nothing the tuning file holds
reaches the header.

The tuning file is read to guard a different piece of code: the driver's runtime
CCM path in ar-isp-colour.h, which is what actually consumes the blob when AWB
repacks ccm1 on the live device. f32_q8sm() and pack_matrix() here are Python
mirrors of that packing. The script packs the blob's first illuminant bank with
them and requires the result to equal the ccm1 registers the vendor's MMIO trace
recorded, so if the driver's quantisation or the blob's layout ever drifts, this
script fails rather than the picture changing silently on hardware. The other
blob checks pin the surrounding assumptions: the ccm1 tuning gate reads 1 and the
ccm2 gate reads 0 (so ccm1 is AWB-driven and ccm2 static), the illuminant ladder
is still ascending kelvin, and banks 4 to 7 are still empty.

That guard is only as good as its oracle, and the oracle is the weak part.
TRACED_CCM1_RUNTIME and TRACED_CCM2 are register values transcribed by hand from
an MMIO trace. They are a third source, neither the library nor the blob, and
unlike those two they are not re-derivable from anything an operator supplies on
the command line: the trace they came from is a capture artifact and is not in
the tree. A regenerated trace with different values cannot be told from a
transcription error. Closing this means either checking in the trace excerpt they
came from or deriving them from the library, and until then a failure of
check_blob() needs the trace re-read before it is believed.

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

# Both copies of the matrix live in one BLOCK, the second COPY_WORDS words after the first.
COPY_WORDS = 8

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

GATE_CCM1 = 0x253FC
GATE_CCM2 = 0x2595C
LADDER = 0x25438
ILLUMINANTS = 8
BANKS = 0x25470
BANK_STRIDE = 0x24
BANKS_USED = 4

# Oracles, not inputs: last-write register values transcribed from an ISP MMIO
# write trace. The runtime ccm1 matrix the AWB path packed (writes w0006d6..w0006db,
# ISP +0x3400) and the ccm2 init block (writes w0001b4..w0001c7, ISP +0x3800).
#
# Hand-transcribed and not regenerable from --lib or --blob; see the module
# docstring. Re-read the trace before trusting a mismatch against these.
TRACED_CCM1_RUNTIME = (0x80400159, 0x00008019, 0x0140800C,
                       0x00008033, 0x804C0014, 0x00000138)
TRACED_CCM2 = (0x80F7020C, 0x00008017, 0x01978051, 0x00008048, 0x80868003,
               0x00000187, 0x00000000, 0x00000000, 0x80F7020C, 0x00008017,
               0x01978051, 0x00008048, 0x80868003, 0x00000187, 0x00000000,
               0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000)


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


def pack_matrix(bits: Sequence[int]) -> tuple[int, ...]:
    """Mirror ar_isp_ccm_pack: six words, two coefficients per even word."""
    coeff = [f32_q8sm(f32) for f32 in bits]
    return (coeff[0] | coeff[1] << 16, coeff[2],
            coeff[3] | coeff[4] << 16, coeff[5],
            coeff[6] | coeff[7] << 16, coeff[8])


def check_ccm1_template(words: Sequence[int]) -> None:
    identity = pack_matrix([F32_ONE, 0, 0, 0, F32_ONE, 0, 0, 0, F32_ONE])
    if (tuple(words[:MATRIX_WORDS]) != identity or
            tuple(words[COPY_WORDS:COPY_WORDS + MATRIX_WORDS]) != identity):
        sys.exit("ccm1 template is not an identity pair under the recovered layout")

    if any(words[i] for i in (6, 7, 14, 15, 16, 17, 18, 19)):
        sys.exit("ccm1 template has data outside the two matrix copies")


def check_ccm2_template(words: Sequence[int]) -> None:
    if tuple(words) != TRACED_CCM2:
        sys.exit("ccm2 template no longer matches the traced init block")
    if words[0:8] != words[8:16]:
        sys.exit("ccm2 template copies differ")


def check_blob(blob: bytes) -> None:
    gate1 = struct.unpack_from("<I", blob, GATE_CCM1)[0]
    if gate1 != 1:
        sys.exit(f"ccm1 gate at 0x{GATE_CCM1:x} reads {gate1}, expected 1")

    gate2 = struct.unpack_from("<I", blob, GATE_CCM2)[0]
    if gate2 != 0:
        sys.exit(f"ccm2 gate at 0x{GATE_CCM2:x} reads {gate2}, expected 0")

    ladder = struct.unpack_from(f"<{ILLUMINANTS}f", blob, LADDER)
    if list(ladder) != sorted(ladder) or not 1000 < ladder[0] < ladder[-1] < 20000:
        sys.exit("illuminant ladder is not an ascending kelvin sequence")

    for bank in range(BANKS_USED, ILLUMINANTS):
        off = BANKS + bank * BANK_STRIDE
        if any(blob[off:off + BANK_STRIDE]):
            sys.exit(f"matrix bank {bank} expected empty, has data")

    bank0 = struct.unpack_from("<9I", blob, BANKS)
    if pack_matrix(bank0) != TRACED_CCM1_RUNTIME:
        sys.exit("packed bank 0 no longer matches the traced ccm1 registers")


def emit(ccm1: Sequence[int], ccm2: Sequence[int]) -> None:
    print("/* SPDX-License-Identifier: GPL-2.0 */")
    print("/* Generated by kernel/scripts/isp/gen-ccm.py. Do not edit. */")
    print("/*")
    print(" * CCM register-bank init blocks, 20 words each: ccm1 at ISP +0x3400 (an")
    print(" * identity matrix pair) and ccm2 at +0x3800 (the vendor's fixed matrix")
    print(" * pair), lifted verbatim from the vendor service's ISP-init template.")
    print(" * The ccm2 block is word-identical to the traced init writes; the ccm1")
    print(" * block decodes to an identity under the packing in ar-isp-colour.h.")
    print(" */")
    print()
    print("#ifndef AR_ISP_CCM_INIT_H")
    print("#define AR_ISP_CCM_INIT_H")
    for name, words in (("ar_isp_ccm1_init", ccm1), ("ar_isp_ccm2_init", ccm2)):
        print()
        print(f"static const u32 {name}[{len(words)}] = {{")
        for i in range(0, len(words), 4):
            row = ", ".join(f"0x{v:08x}" for v in words[i:i + 4])
            print(f"\t{row},")

        print("};")

    print()
    print("#endif /* AR_ISP_CCM_INIT_H */")


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
