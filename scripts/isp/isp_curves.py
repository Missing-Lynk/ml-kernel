"""
The closed forms behind the ISP's carried curve tables.

Three of the tables the vendor's service library carries as prepacked constants are
elementary functions of the sample index. Gamma page 1 and both DRC tail banks are
reproduced here exactly, every sample, so their generators compute the table instead of
slicing the library and unpacking bit fields. The compander's curve is reproduced to one
count and so is carried, with its form recorded here for review.

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


# The compander page's one region that carries a curve rather than a ramp: 257 samples of
# 16 bits at page offset 0x400, stored as overlapping pairs, one pair per 32-bit word. Two
# square-root branches meet at a knee at index 64, across which the slope jumps, so this
# is two curves and not one with a bend. Each branch is written through the endpoint it
# reaches, so the only number solved for is the second branch's shift:
#
#     i <= 64:  round(KNEE_VALUE * sqrt(i / 64))
#     i >  64:  round(FULL_SCALE * sqrt((i - SHIFT) / (256 - SHIFT)))
#
# That reproduces 242 of 257 samples and never misses by more than one count, which is
# close enough to say what the page is and not close enough to generate it, so
# ar-isp-compander.h carries the words. This function exists for the check that keeps
# that claim honest.

COMPANDER_SAMPLES: int = 257
COMPANDER_KNEE: int = 64
COMPANDER_KNEE_VALUE: int = 16270
COMPANDER_FULL_SCALE: int = 0xFFFF
COMPANDER_SHIFT: float = 52.159
COMPANDER_TOLERANCE: int = 1


def compander_curve() -> list[int]:
    """The 257 samples of the compander's curve region, to within COMPANDER_TOLERANCE."""
    last = COMPANDER_SAMPLES - 1
    samples: list[int] = []
    for i in range(COMPANDER_SAMPLES):
        if i <= COMPANDER_KNEE:
            value = COMPANDER_KNEE_VALUE * math.sqrt(i / COMPANDER_KNEE)
        else:
            value = COMPANDER_FULL_SCALE * math.sqrt((i - COMPANDER_SHIFT) /
                                                     (last - COMPANDER_SHIFT))

        samples.append(round(value))

    return samples
