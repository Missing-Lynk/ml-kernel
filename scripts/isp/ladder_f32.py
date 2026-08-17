"""
The vendor's ladder arithmetic in binary32, shared by the stage check scripts.

The gain-keyed drivers (isp_sub_rnr 0x1993b8, isp_sub_lee_lnr 0x1bd8a8,
isp_sub_de3d 0x1c6c10, isp_sub_cfa 0x1b1f28) all blend in float: scvtf on each
record word, fmul for the upper term, a fused fmadd, then fcvtzs. An exact
integer blend differs by one wherever the true value lands just under an
integer, including where both records are equal, so this is not a rounding
preference. The kernel side is ar-isp-softfloat.h; this is the independent
Python restatement the checks compare against.

isp_sub_cm at 0x19fe28 and isp_sub_cm2 at 0x1a14ec use no float at all, so they
keep the integer helpers in their own scripts.
"""

import struct


def f32(x: float) -> float:
    """Round to binary32, which is the precision the vendor computes in."""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def fma32(a: float, b: float, c: float) -> float:
    """One rounding, as fmadd does. Doubles hold a*b+c exactly here."""
    return f32(float(a) * float(b) + float(c))


def edge(blob: bytes, bands: int, band: int, half: int) -> float:
    """A band edge, which the tuning file stores as binary32."""
    return struct.unpack_from("<f", blob, bands + band * 8 + half)[0]


def select(blob: bytes, bands: int, count: int, interp: int,
           gain_q16: int) -> tuple[int, float]:
    """
    Band and gap fraction. A zero fraction means the record is used verbatim,
    which is the memcpy path the vendor takes inside a band.
    """
    gain = f32(gain_q16 / 65536.0)

    band = count - 1
    for i in range(count - 1):
        if gain <= edge(blob, bands, i, 4):
            band = i
            break

    t = 0.0
    if interp and band > 0:
        lo = edge(blob, bands, band, 0)
        prev_hi = edge(blob, bands, band - 1, 4)
        if gain < lo and lo > prev_hi:
            t = f32(f32(gain - prev_hi) / f32(lo - prev_hi))

    return band, t


def blend(lo: int, hi: int, t: float) -> int:
    """One fused blend, truncated toward zero as fcvtzs does."""
    if not t:
        return hi

    return int(fma32(f32(1.0 - t), lo, f32(t * hi)))
