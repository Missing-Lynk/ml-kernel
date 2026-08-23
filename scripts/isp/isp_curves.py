"""
The closed forms behind the ISP's carried curve tables.

Three of the tables the vendor's service library carries as prepacked constants are
elementary functions of the sample index. Gamma page 1 and both DRC tail banks are
reproduced here exactly, every sample, so their generators compute the table instead of
slicing the library and unpacking bit fields. The compander's curve is reproduced to one
count with its smooth form, and exactly with a small correction set. Its other three
tables need no corrections at all, but the page is still carried because that correction
set and three of the square law's samples are unexplained.

Two separate claims hold these tables up, and they are checked in two places:

    formula == vendor library     kernel/scripts/isp/check-table-closed-forms.py
    header  == formula            kernel/scripts/isp/check-regenerable.py

The first is the one that matters here. A generator that computes can emit a wrong table
from a mistyped constant where one that copies bytes cannot, so the comparison against
libmpp_service.so is what makes computing safe, and it is not optional.

What "exactly" is worth here: the closest any of these values comes to a floor boundary is
0.00024 counts, so double precision clears the requirement by nine orders of magnitude,
while a fixed-point evaluator would need about 26 bits of relative accuracy. That is why
this module is host-side Python producing a table, and the driver reads the table.

Each constant below says where it came from, because they are not all the same kind of
number. A published standard's constant, an integer the samples pin to a fraction of a
count, and a decimal solved for by least squares carry different weight, and only the
first two make a table a derivation rather than vendor data in compact form.
"""

import math

# Gamma page 1 is the Rec.709 opto-electronic transfer function with its 0.45 exponent
# replaced by 1/2.2. The toe slope, the breakpoint and the gain/offset pair are the
# published constants unchanged, so none of them was solved for; only the exponent
# differs from stock Rec.709, which is the one thing to check against a reference before
# assuming another curve of this shape behaves the same way. Writing them as exact
# fractions is available and declined: 9/2, 9/500 and 99/1000 are the same doubles, and
# the decimals are how the standard spells them and so how a reader recognises them.
#
# The abscissa is i/512, the page's own sample count, and not the i*8/4095 that
# decimating a 4096-entry tuning-file curve by eight would give. Under that second axis
# the same form matches 294 of 512, so the vendor computed this page directly at 512
# points, which is consistent with it appearing in no tuning file.

GAMMA_SAMPLES: int = 512
GAMMA_MAX: int = 4095
GAMMA_TOE_SLOPE: float = 4.5
GAMMA_TOE_LIMIT: float = 0.018
GAMMA_GAIN: float = 1.099
GAMMA_OFFSET: float = 0.099
GAMMA_EXPONENT: float = 1 / 2.2


def gamma_transfer(x: float) -> float:
    """The transfer function at `x` in [0, 1], normalised to [0, 1]."""
    if x < GAMMA_TOE_LIMIT:
        return GAMMA_TOE_SLOPE * x

    return GAMMA_GAIN * math.pow(x, GAMMA_EXPONENT) - GAMMA_OFFSET


def gamma_page1_samples() -> list[int]:
    """The 512 twelve-bit samples of gamma page 1."""
    return [math.floor(GAMMA_MAX * gamma_transfer(i / GAMMA_SAMPLES))
            for i in range(GAMMA_SAMPLES)]


def gamma_page1_tail() -> int:
    """
    The sample after the page, which the last record mirrors forward.

    The hardware reads this field, so the packer has to write it. It is the transfer
    function one step past the page, at x = 1 exactly, which is full scale.
    """
    return math.floor(GAMMA_MAX * gamma_transfer(1.0))


# Banks 2 and 3 of the DRC page are an exponential in the square root of the normalised
# sample index, at one shared amplitude:
#
#     bank0[i] = floor(A * exp(-k0 * u))                 u = sqrt(i / 256)
#     bank1[i] = floor(A * (exp(-k1 * u) - exp(-k1)))
#
# The subtraction is what takes bank1 to exactly zero at full scale.
#
# The same two forms generate the tuning file's own DRC profiles, with a linear abscissa
# where these use the square root and rates of 1.0, 0.75, 16 and 12. That agreement is
# what says these are the vendor's arithmetic rather than a curve that happens to fit.
#
# On where the three numbers come from, since two are solid and one is not:
#
#   A is an integer, and the samples pin it. Holding k free, the amplitudes that
#   reproduce every sample span 98304.00 to 98304.02 on bank0 and 98303.99 to 98304.03
#   on bank1, two independent windows around one value: 0x18000, exactly 1.5 in the Q16
#   the banks carry gains in.
#
#   k1 is exactly 5.8. Least squares returned 5.8002, and the two-digit decimal
#   reproduces all 255 samples on its own.
#
#   k0 is a decimal solved from the table, with no meaning found behind it. Its exact
#   window is (0.3624558324, 0.3624559652], and nothing simple lies inside: the simplest
#   rational is 1954/5391, exp(k0) is 3197/2225, k0/ln(2) is 639/1222. Every candidate
#   with a reading falls outside it. k1/16 = 0.3625 reproduces 6 samples of 256;
#   ln(A / 68416), the geometric interpolation onto the last sample, reproduces 225; no
#   clean rate reproduces the table even with a free additive term. k1/k0 is 16.00194,
#   which is near 16 and is not 16. So the shortest exact spelling of k0 is the decimal
#   itself, and this bank stays vendor data in compact form rather than a derivation.
#
# Two samples lie off the curves and are carried. Sample 0 of both banks is a clamp,
# 15.9375 in Q16, where a gain curve evaluated at black would run away. bank1 sample 1
# sits 283 counts above its curve for a reason that is not recovered.

DRC_SAMPLES: int = 257
DRC_AMPLITUDE: int = 0x18000
# The width of the packed lane each sample lands in, which bounds every value here.
DRC_LANE_MAX: int = 0xFFFFF
DRC_BANK0_RATE: float = 0.3624559
DRC_BANK1_RATE: float = 5.8
DRC_CLAMP: int = 0xFF000
DRC_BANK1_SAMPLE1: int = 68398


def drc_tail_bank0() -> list[int]:
    """Bank 2 of the DRC page: a decay from the clamp to 1.0439 in Q16."""
    samples: list[int] = [DRC_CLAMP]
    for i in range(1, DRC_SAMPLES):
        u = math.sqrt(i / (DRC_SAMPLES - 1))
        samples.append(math.floor(DRC_AMPLITUDE * math.exp(-DRC_BANK0_RATE * u)))

    return samples


def drc_tail_bank1() -> list[int]:
    """Bank 3 of the DRC page: a faster decay, reaching exactly zero at full scale."""
    floor_term = math.exp(-DRC_BANK1_RATE)
    samples: list[int] = [DRC_CLAMP, DRC_BANK1_SAMPLE1]
    for i in range(2, DRC_SAMPLES):
        u = math.sqrt(i / (DRC_SAMPLES - 1))
        samples.append(math.floor(DRC_AMPLITUDE *
                                  (math.exp(-DRC_BANK1_RATE * u) - floor_term)))

    return samples


# The compander page's main curve: 257 samples of 16 bits at page offset 0x400, stored
# as overlapping pairs, one pair per 32-bit word. Two square-root branches meet at a knee
# at index 64, across which the slope jumps, so this is two curves and not one with a
# bend. Each branch is written through the endpoint it reaches, so the only number solved
# for is the second branch's shift:
#
#     i <= 64:  round(KNEE_VALUE * sqrt(i / 64))
#     i >  64:  round(FULL_SCALE * sqrt((i - SHIFT) / (256 - SHIFT)))
#
# No single branch amplitude, shift or rounding mode reproduces every sample. The vendor
# table is exactly the smooth model with the 15 one-count-low samples named below. This is
# a compact exact model, not a recovered producer, so ar-isp-compander.h still carries the
# words.

COMPANDER_SAMPLES: int = 257
COMPANDER_KNEE: int = 64
COMPANDER_KNEE_VALUE: int = 16270
COMPANDER_FULL_SCALE: int = 0xFFFF
COMPANDER_SHIFT: float = 52.159
COMPANDER_CURVE_LOW_CORRECTIONS: tuple[int, ...] = (
    3, 4, 14, 34, 40, 46, 52, 55, 58, 59, 65, 82, 96, 113, 114,
)


def compander_curve() -> list[int]:
    """The exact 257 samples of the compander's square-root curve region."""
    last = COMPANDER_SAMPLES - 1
    samples: list[int] = []
    for i in range(COMPANDER_SAMPLES):
        if i <= COMPANDER_KNEE:
            value = COMPANDER_KNEE_VALUE * math.sqrt(i / COMPANDER_KNEE)
        else:
            value = COMPANDER_FULL_SCALE * math.sqrt((i - COMPANDER_SHIFT) /
                                                     (last - COMPANDER_SHIFT))

        sample = round(value)
        if i in COMPANDER_CURVE_LOW_CORRECTIONS:
            sample -= 1

        samples.append(sample)

    return samples


# The compander head opens with a 33-sample 20-bit table in the same overlapping triple
# packing as DRC. It is two straight segments meeting at sample 2:
#
#     i <= 2:  i * 0x20732
#     i >  2:  0x40e64 + round((2^20 - 0x40e64) * (i - 2) / 30)
#
# clipped to the 20-bit lane maximum, which only the last sample reaches.
#
# The second segment runs to 2^20, one count past the largest value the lane can hold,
# and saturates there. That is the same arithmetic the square-law table at page offset
# 0x1000 uses, where 16 * 256^2 is also 2^20: both 20-bit tables in this page are built
# against full scale and clipped by the lane. Anchoring the line on 0xfffff instead moves
# it half a count and costs six one-count corrections, so the anchor is what makes this
# exact with nothing carried.

COMPANDER_HEAD_SAMPLES: int = 33
COMPANDER_HEAD_KNEE: int = 2
COMPANDER_HEAD_STEP: int = 0x20732
COMPANDER_HEAD_TOP: int = 1 << 20


def compander_head_ramp() -> list[int]:
    """The compander head table: two long steps, then a line to full scale."""
    start = COMPANDER_HEAD_KNEE * COMPANDER_HEAD_STEP
    span = COMPANDER_HEAD_TOP - start
    intervals = COMPANDER_HEAD_SAMPLES - 1 - COMPANDER_HEAD_KNEE
    samples = [i * COMPANDER_HEAD_STEP for i in range(COMPANDER_HEAD_KNEE + 1)]
    for k in range(1, intervals + 1):
        samples.append(min(start + (span * k + intervals // 2) // intervals,
                           DRC_LANE_MAX))

    return samples


# The compander head has a second 16-bit overlapping-pair table at page offset 0x800: 33
# samples, of which the first 32 are a ramp in 0x800 steps. Unlike the square-root curve
# above, this reproduces the library's samples exactly.
#
# The last sample is 0xfb00, which is 0x300 rather than 0x800 above the one before it, and
# it is carried for the same reason the DRC banks carry theirs. A saturation would explain
# it and nothing supports one: it is below the 0xffff the region could reach, the clamp
# would bind on no other sample, and the page's two neighbouring tables both do run to
# their own full scale, the square-root curve to 0xffff and the 20-bit table at offset 0 to
# 0xfffff. So the value is recorded rather than derived from a rule.

COMPANDER_RAMP_SAMPLES: int = 33
COMPANDER_RAMP_STEP: int = 0x800
COMPANDER_RAMP_LAST: int = 0xFB00


def compander_ramp() -> list[int]:
    """The exact 33-sample compander ramp at page offset 0x800."""
    ramp = [i * COMPANDER_RAMP_STEP for i in range(COMPANDER_RAMP_SAMPLES - 1)]

    return [*ramp, COMPANDER_RAMP_LAST]


# The compander mid span is a 257-sample 20-bit table in the same overlapping triple
# packing as DRC. Its body is a square law:
#
#     sample[i] = 16 * i^2 + round(i^2 / 2114)
#
# capped to the 20-bit lane maximum, which is what the last sample reaches. The square is
# (i / 256)^2 at 20 bits, so this table companding by a square is the counterpart of the
# square-root curve at page offset 0x400.
#
# The correction is at most 29 counts in 1048575 and its divisor is fitted. The samples
# admit any value in (2113.1, 2116.8], nothing in that window has a reading, and 2048,
# which the square's own scale would suggest, lies outside it. So the correction is
# recorded rather than derived, and the bias is the divisor halved rather than a second
# free constant.
#
# Samples 0 to 7 are a linear toe at 127 counts per step, which the square law meets at
# sample 8.
#
# Samples 249, 250 and 251 are carried and unexplained. They leave the square, climb about
# 21500 a step to reach full scale at 251, and then sample 252 drops 32481 counts back
# onto the square law, which runs on to the end. That makes the table non-monotonic across
# one interval, and a transfer curve that reverses inverts the response over it, the same
# hazard ar_isp_gamma_from_blob clamps for. Whether the vendor intended a shoulder here is
# not recovered, so the three values are recorded and the break is left visible instead of
# being folded into the formula.

COMPANDER_MID_SAMPLES: int = 257
COMPANDER_MID_SQUARE_SCALE: int = 16
COMPANDER_MID_CORRECTION_DIVISOR: int = 2114
COMPANDER_MID_TOE: tuple[int, ...] = (1, 128, 255, 382, 510, 637, 764, 891)
COMPANDER_MID_ANOMALY: dict[int, int] = {249: 0xF5730, 250: 0xFAB26, 251: 0xFFFFF}


def compander_mid() -> list[int]:
    """The compander mid table: a square law, a linear toe and three carried samples."""
    samples: list[int] = list(COMPANDER_MID_TOE)
    for i in range(len(COMPANDER_MID_TOE), COMPANDER_MID_SAMPLES):
        if i in COMPANDER_MID_ANOMALY:
            samples.append(COMPANDER_MID_ANOMALY[i])
            continue

        square = COMPANDER_MID_SQUARE_SCALE * i * i
        correction = ((i * i + COMPANDER_MID_CORRECTION_DIVISOR // 2) //
                      COMPANDER_MID_CORRECTION_DIVISOR)
        samples.append(min(square + correction, DRC_LANE_MAX))

    return samples
