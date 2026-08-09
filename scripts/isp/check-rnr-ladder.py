#!/usr/bin/env python3
"""
Prove the rnr ladder transform in ar-isp-rnr.h against both measured points.

The rnr driver in the vendor service (0x1993b8) recomputes the twelve packed
registers at ISP 0x1808..0x1834 from a ladder in the tuning file whenever the
3A loop moves the gain: a header (enable, interpolate, mode, count, selector),
twelve [low, high] float32 bands, and a 352-byte payload record per band.
Inside a band the payload is verbatim; between bands it is blended linearly
and truncated toward zero. Register k packs payload word 14+k in the high half
and word 2+k in the low half.

This script mirrors the kernel header's integer arithmetic exactly (Q16 band
edges, Q24 blend fraction, one truncation on the blended sum) and refuses to
pass unless:

  the header reads (enable 1, interpolate 1, mode 0, count 12, selector 0)
  and the band edges are strictly increasing;

  the transform at abscissa 1.0 reproduces 0x000f000a in all twelve
  registers, which is the cold bank the register replay carries and the value
  captured on our own hardware;

  the transform reproduces 0x002e002d, the vendor's live registers captured
  streaming at saturated AE, for some abscissa in the 13.6 to 14.2 window the
  band arithmetic demands (plans/au-blend-engine-and-notch.md section 2);

  the tail transform reproduces all twenty-two of the vendor's writes to
  0x1838..0x188c at every abscissa in the 4.0 to 8.0 bracket that cfa and cnf
  jointly pin the capture to, and that at least one of them has moved by 16.0,
  which is what makes deriving the run different from replaying it.

The tuning file is proprietary and is not in the repository.

    kernel/scripts/isp/check-rnr-ladder.py \\
        --tuning out/air-gather/camera/nt99235_tuning_preview_fpv.bin
"""

import argparse
import struct
import sys

HEADER = 0x79D8
BANDS = 0x79EC
PAYLOAD = 0x7A6C
STRIDE = 0x160
COUNT = 12
LO_WORD = 2
HI_WORD = 14
REGS = 12

COLD_REG = 0x000F000A
VENDOR_REG = 0x002E002D
VENDOR_WINDOW = (13.6, 14.2)

# The rest of the bank, 0x1838..0x188c: four triples of seven payload words
# then two blocks of seventeen. See the ar-isp-rnr.h comment for the packer
# evidence that fixes the word mapping.
TAIL_BASE = 0x1838
TAIL_REGS = 22
TAIL_WORD = 26
TRIPLES = 4
TRIPLE_WORDS = 7
BLOCKS = 2
BLOCK_WORDS = 17

# The vendor's own writes to the tail, from the final ISP run it issues once
# the receiver is live. Transcribed here so this script needs no capture file.
TAIL_MEASURED = (
    0x06400C80, 0x00000258, 0x060A0D10,
    0x06400C80, 0x00000258, 0x060A0D10,
    0x06400C80, 0x00000258, 0x060A0D10,
    0x06400C80, 0x00000258, 0x060A0D10,
    0x3C281A0D, 0xC8A0785A, 0x00080806, 0x000C0A0A, 0x00100E0E,
    0x3C281A0D, 0xC8A0785A, 0x00080806, 0x000C0A0A, 0x00100E0E,
)

# The abscissa the cfa and cnf ladders jointly bracket the setup-trace capture to.
BRACKET = (4.0, 8.0)

# A second measured point: the streaming vendor's live bank, captured on slot A
# with the goggle bound and the link delivering. Its twelve ladder registers all
# read 0x002c002d, which the ladder alone satisfies over 12.355..12.961, and the
# tail below narrows that to 12.355..12.613. The point of carrying it is that
# 12.4 is past the fourth band, so these tail values are ones the frozen replay
# could not produce: it holds 0x00080806 at 0x1870 where the vendor holds
# 0x000b0c0b.
LIVE_LADDER_REG = 0x002C002D
LIVE_TAIL = (
    0x06400C80, 0x00000258, 0x060A0D10,
    0x06400C80, 0x00000258, 0x060A0D10,
    0x06400C80, 0x00000258, 0x060A0D10,
    0x06400C80, 0x00000258, 0x060A0D10,
    0x3C281A0D, 0xC8A0785A, 0x000B0C0B, 0x00090A0B, 0x00080809,
    0x3C281A0D, 0xC8A0785A, 0x000B0C0B, 0x00090A0B, 0x00080809,
)
LIVE_WINDOW = (12.0, 13.5)


def f32_q16(bits: int) -> int:
    """Mirror ar_isp_f32_q16: unsigned Q16 truncated toward zero."""
    mant = (bits & 0x7FFFFF) | 0x800000
    exp = ((bits >> 23) & 0xFF) - 127
    shift = 7 - exp

    if bits & 0x80000000:
        return 0

    if shift >= 32:
        return 0

    if shift < -8:
        return 0xFFFFFFFF

    if shift <= 0:
        return mant << -shift

    return mant >> shift


def blend(a: int, b: int, t_q24: int) -> int:
    """Mirror ar_isp_ladder_blend: one Q24 sum, one truncation."""
    return (a * ((1 << 24) - t_q24) + b * t_q24) >> 24


def rnr_from_blob(blob: bytes, gain_q16: int) -> list[int]:
    """Mirror ar_isp_rnr_from_blob word for word."""
    count = struct.unpack_from("<I", blob, HEADER + 0xC)[0]
    interp = struct.unpack_from("<I", blob, HEADER + 0x4)[0]
    count = COUNT if not 1 <= count <= COUNT else count

    edges = [f32_q16(struct.unpack_from("<I", blob, BANDS + i * 4)[0])
             for i in range(count * 2)]

    band = count - 1
    for i in range(count - 1):
        if gain_q16 <= edges[i * 2 + 1]:
            band = i
            break

    t_q24 = 0
    if interp and band > 0:
        lo, prev_hi = edges[band * 2], edges[band * 2 - 1]
        if gain_q16 < lo and lo > prev_hi:
            t_q24 = ((gain_q16 - prev_hi) << 24) // (lo - prev_hi)

    regs = []
    for k in range(REGS):
        lo, hi = (struct.unpack_from("<I", blob, PAYLOAD + band * STRIDE +
                                     (col + k) * 4)[0]
                  for col in (LO_WORD, HI_WORD))

        if t_q24:
            prev = PAYLOAD + (band - 1) * STRIDE
            lo = blend(struct.unpack_from("<I", blob, prev + (LO_WORD + k) * 4)[0],
                       lo, t_q24)
            hi = blend(struct.unpack_from("<I", blob, prev + (HI_WORD + k) * 4)[0],
                       hi, t_q24)

        regs.append(((hi << 16) | (lo & 0xFFFF)) & 0xFFFFFFFF)

    return regs


def select(blob: bytes, gain_q16: int) -> tuple[int, int]:
    """The band and blend fraction, the same walk rnr_from_blob does."""
    interp = struct.unpack_from("<I", blob, HEADER + 0x4)[0]
    edges = [f32_q16(struct.unpack_from("<I", blob, BANDS + i * 4)[0])
             for i in range(COUNT * 2)]

    band = COUNT - 1
    for i in range(COUNT - 1):
        if gain_q16 <= edges[i * 2 + 1]:
            band = i
            break

    t_q24 = 0
    if interp and band > 0:
        lo, prev_hi = edges[band * 2], edges[band * 2 - 1]
        if gain_q16 < lo and lo > prev_hi:
            t_q24 = ((gain_q16 - prev_hi) << 24) // (lo - prev_hi)

    return band, t_q24


def word(blob: bytes, band: int, t_q24: int, index: int) -> int:
    """One payload word, blended into the previous band's when mid-gap."""
    value = struct.unpack_from("<I", blob, PAYLOAD + band * STRIDE + index * 4)[0]
    if t_q24:
        prev = struct.unpack_from("<I", blob,
                                  PAYLOAD + (band - 1) * STRIDE + index * 4)[0]
        value = blend(prev, value, t_q24)

    return value


def tail_from_blob(blob: bytes, gain_q16: int) -> list[int]:
    """Mirror ar_isp_rnr_tail_from_blob word for word."""
    band, t_q24 = select(blob, gain_q16)

    def pack(index: int, n: int, mask: int, step: int) -> int:
        out = 0
        for i in range(n):
            out |= (word(blob, band, t_q24, index + i) & mask) << (i * step)

        return out

    regs: list[int] = []
    index = TAIL_WORD
    for _ in range(TRIPLES):
        regs.append(pack(index, 2, 0xFFF, 16))
        regs.append(pack(index + 2, 1, 0xFFF, 0))
        regs.append(pack(index + 3, 4, 0x1F, 8))
        index += TRIPLE_WORDS

    for _ in range(BLOCKS):
        regs.append(pack(index, 4, 0xFF, 8))
        regs.append(pack(index + 4, 4, 0xFF, 8))
        regs.append(pack(index + 8, 3, 0x1F, 8))
        regs.append(pack(index + 11, 3, 0x1F, 8))
        regs.append(pack(index + 14, 3, 0x1F, 8))
        index += BLOCK_WORDS

    return regs


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tuning", required=True, help="nt99235 tuning blob")
    args = ap.parse_args()

    with open(args.tuning, "rb") as handle:
        blob = handle.read()

    header = struct.unpack_from("<5I", blob, HEADER)
    if header != (1, 1, 0, COUNT, 0):
        sys.exit(f"rnr header reads {header}, expected (1, 1, 0, {COUNT}, 0)")

    edges = struct.unpack_from(f"<{COUNT * 2}f", blob, BANDS)
    if list(edges) != sorted(edges) or len(set(edges)) != COUNT * 2:
        sys.exit("band edges are not strictly increasing")

    cold = rnr_from_blob(blob, 1 << 16)
    if cold != [COLD_REG] * REGS:
        sys.exit(f"abscissa 1.0 gives {cold[0]:#010x}, "
                 f"expected {COLD_REG:#010x} in all twelve")
    print(f"abscissa 1.0    -> {COLD_REG:#010x} x{REGS}, the replayed cold bank")

    hits = [g / 256 for g in range(int(VENDOR_WINDOW[0] * 256),
                                   int(VENDOR_WINDOW[1] * 256) + 1)
            if rnr_from_blob(blob, (g << 16) // 256) == [VENDOR_REG] * REGS]
    if not hits:
        sys.exit(f"no abscissa in {VENDOR_WINDOW} reproduces "
                 f"{VENDOR_REG:#010x}")

    print(f"abscissa {hits[0]:.3f}..{hits[-1]:.3f} -> {VENDOR_REG:#010x} x{REGS}, "
          f"the vendor's live bank")

    lo, hi = (int(b * 256) for b in BRACKET)
    bad = [g / 256 for g in range(lo, hi + 1)
           if tail_from_blob(blob, (g << 16) // 256) != list(TAIL_MEASURED)]
    if bad:
        sys.exit(f"the tail disagrees with the vendor's writes at abscissa "
                 f"{bad[0]} ({len(bad)} of {hi - lo + 1} in the bracket)")

    print(f"tail 0x{TAIL_BASE:04x}..0x{TAIL_BASE + 4 * (TAIL_REGS - 1):04x} "
          f"reproduces all {TAIL_REGS} vendor writes across the whole "
          f"{BRACKET[0]} to {BRACKET[1]} bracket")

    # The point of deriving the tail rather than replaying it: past the fourth
    # band the packed fields move, and a replay cannot follow them.
    moved = sum(1 for a, b in zip(tail_from_blob(blob, 16 << 16), TAIL_MEASURED,
                                  strict=True)
                if a != b)
    if not moved:
        sys.exit("the tail does not move by abscissa 16.0, so the ladder walk "
                 "is not reaching the later bands")

    print(f"      and {moved} of them move by abscissa 16.0, which is what the "
          f"replay had frozen")

    # The live slot-A point. The ladder and the tail must agree on an abscissa
    # jointly, and the tail must be the one that narrows the window: if it
    # accepted everything the ladder does, it would not be constraining the
    # arithmetic at all.
    lo, hi = (int(b * 256) for b in LIVE_WINDOW)
    from_ladder = [g / 256 for g in range(lo, hi + 1)
                   if rnr_from_blob(blob, (g << 16) // 256) == [LIVE_LADDER_REG] * REGS]
    joint = [g for g in from_ladder
             if tail_from_blob(blob, int(g * 65536)) == list(LIVE_TAIL)]
    if not joint:
        sys.exit(f"no abscissa in {LIVE_WINDOW} satisfies both the live ladder "
                 f"and the live tail")

    if len(joint) >= len(from_ladder):
        sys.exit("the tail accepts every abscissa the ladder does, so it is not "
                 "constraining the arithmetic")

    print(f"live slot-A bank: ladder alone allows {from_ladder[0]:.3f}.."
          f"{from_ladder[-1]:.3f}, ladder and tail together allow "
          f"{joint[0]:.3f}..{joint[-1]:.3f}")

    frozen = sum(1 for a, b in zip(LIVE_TAIL, TAIL_MEASURED, strict=True) if a != b)
    print(f"      {frozen} of the 22 differ from the setup-trace values, so the "
          f"replay was wrong at this gain")

    print("\nrnr ladder agrees with ar-isp-rnr.h and the three measured points")
    return 0


if __name__ == "__main__":
    sys.exit(main())
