/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-softfloat.h - the slice of binary32 the ISP ladders need.
 *
 * The ladder stages blend in binary32: scvtf on each record word, fmul for the
 * upper term, a fused fmadd, then fcvtzs. Kernel code cannot use the FPU, so
 * the same arithmetic is done here in integers. An exact integer blend is not
 * a substitute: it differs by one wherever the true value lands just under an
 * integer, including where both records are equal, so blending 75 into 75
 * gives 74.
 *
 * Not a general soft-float library. Only the operations the ladders issue, over
 * the domain they use:
 *
 *   - finite operands; no NaN, no infinity, no subnormal inputs
 *   - values converted from s32, which is what scvtf takes; lnr's records carry
 *     negative words, so the sign is not optional
 *   - magnitudes far inside the range where these routines stay exact
 *
 * Every routine rounds to nearest with ties to even, which is what the hardware
 * does at default FPCR. ar_f32_fma rounds once, because fmadd is fused.
 */

#ifndef AR_ISP_SOFTFLOAT_H
#define AR_ISP_SOFTFLOAT_H

#include <linux/types.h>
#ifdef __KERNEL__
#include <linux/bitops.h>
#include <linux/math64.h>
#endif

/* A binary32 bit pattern. */
typedef u32 ar_f32;

#define AR_F32_MANT_BITS	24
#define AR_F32_BIAS		127

/* Where ar_f32_norm parks the significand's top bit, leaving room to add. */
#define AR_F32_NORM_BIT		62

/* An unpacked value, (-1)^sign * sig * 2^exp. sig == 0 is the zero. */
struct ar_f32_up {
	u64 sig;
	int exp;
	int sign;
};

static inline struct ar_f32_up ar_f32_unpack(ar_f32 a)
{
	struct ar_f32_up r;
	u32 mant = a & 0x7fffff;
	int biased = (a >> 23) & 0xff;

	r.sign = (a >> 31) & 1;

	if (!biased) {
		/* Zero, or a subnormal the ladder domain never produces. */
		r.sig = mant;
		r.exp = 1 - AR_F32_BIAS - 23;

		return r;
	}

	r.sig = mant | 0x800000;
	r.exp = biased - AR_F32_BIAS - 23;

	return r;
}

/*
 * Round sig to AR_F32_MANT_BITS, nearest-even, and pack. sticky carries
 * anything already shifted out below the significand.
 */
static inline ar_f32 ar_f32_round_pack(int sign, u64 sig, int exp, int sticky)
{
	u32 packed;
	int shift;
	u64 rem;

	if (!sig)
		return sign ? 0x80000000 : 0;

	shift = fls64(sig) - AR_F32_MANT_BITS;

	if (shift > 0) {
		rem = sig & (((u64)1 << shift) - 1);
		sig >>= shift;
		exp += shift;

		if (rem > ((u64)1 << (shift - 1)) ||
		    (rem == ((u64)1 << (shift - 1)) && (sticky || (sig & 1))))
			sig++;

		if (sig >> AR_F32_MANT_BITS) {
			sig >>= 1;
			exp++;
		}
	} else if (shift < 0) {
		sig <<= -shift;
		exp += shift;
	}

	packed = (u32)((exp + AR_F32_BIAS + 23) << 23) | (u32)(sig & 0x7fffff);

	return sign ? (packed | 0x80000000) : packed;
}

/* scvtf. Exact for every value the ladders convert; general RN otherwise. */
static inline ar_f32 ar_f32_from_s32(s32 v)
{
	int sign = v < 0;
	u64 mag = sign ? -(s64)v : (s64)v;

	if (!mag)
		return 0;

	return ar_f32_round_pack(sign, mag, 0, 0);
}

/* v / 2^shift, exact: the exponent moves, the significand does not. */
static inline ar_f32 ar_f32_scale_down(ar_f32 a, int shift)
{
	if (!(a & 0x7f800000))
		return a;

	return a - ((u32)shift << 23);
}

static inline ar_f32 ar_f32_mul(ar_f32 a, ar_f32 b)
{
	struct ar_f32_up x = ar_f32_unpack(a);
	struct ar_f32_up y = ar_f32_unpack(b);

	if (!x.sig || !y.sig)
		return (x.sign ^ y.sign) ? 0x80000000 : 0;

	return ar_f32_round_pack(x.sign ^ y.sign, x.sig * y.sig,
				 x.exp + y.exp, 0);
}

/* Park the top bit at AR_F32_NORM_BIT so two values can be compared and added. */
static inline void ar_f32_norm(struct ar_f32_up *v)
{
	int shift;

	if (!v->sig)
		return;

	shift = AR_F32_NORM_BIT + 1 - fls64(v->sig);
	v->sig <<= shift;
	v->exp -= shift;
}

/*
 * x + y with one rounding, both already unpacked. The smaller operand is
 * shifted down and its lost bits become sticky, so nothing here can overflow;
 * cancellation only happens when the exponents are within one, where the shift
 * is small and the subtraction is exact.
 */
static inline ar_f32 ar_f32_add_up(struct ar_f32_up x, struct ar_f32_up y)
{
	int sticky = 0;
	int sign, d;
	u64 sig;

	/*
	 * Signed zero, the one place the sign rules are not the obvious ones:
	 * the sum of two zeros is negative only if both are, so (+0) + (-0) is
	 * +0 at round-to-nearest, and so is an exact cancellation below.
	 */
	if (!x.sig && !y.sig)
		return (x.sign && y.sign) ? 0x80000000 : 0;

	if (!x.sig)
		return ar_f32_round_pack(y.sign, y.sig, y.exp, 0);

	if (!y.sig)
		return ar_f32_round_pack(x.sign, x.sig, x.exp, 0);

	ar_f32_norm(&x);
	ar_f32_norm(&y);

	d = x.exp - y.exp;

	if (d > 0) {
		if (d > 63) {
			y.sig = 0;
			sticky = 1;
		} else {
			sticky = (y.sig & (((u64)1 << d) - 1)) != 0;
			y.sig >>= d;
		}
		y.exp = x.exp;
	} else if (d < 0) {
		if (-d > 63) {
			x.sig = 0;
			sticky = 1;
		} else {
			sticky = (x.sig & (((u64)1 << -d) - 1)) != 0;
			x.sig >>= -d;
		}
		x.exp = y.exp;
	}

	if (x.sign == y.sign) {
		sign = x.sign;
		sig = x.sig + y.sig;
	} else if (x.sig >= y.sig) {
		sign = x.sign;
		sig = x.sig - y.sig;

		/* The dropped bits belong to y, so they raise the result. */
		if (sticky && sig)
			sig--;
	} else {
		sign = y.sign;
		sig = y.sig - x.sig;

		if (sticky && sig)
			sig--;
	}

	if (!sig)
		return 0;

	return ar_f32_round_pack(sign, sig, x.exp, sticky);
}

static inline ar_f32 ar_f32_add(ar_f32 a, ar_f32 b)
{
	return ar_f32_add_up(ar_f32_unpack(a), ar_f32_unpack(b));
}

static inline ar_f32 ar_f32_sub(ar_f32 a, ar_f32 b)
{
	return ar_f32_add(a, b ^ 0x80000000);
}

static inline ar_f32 ar_f32_div(ar_f32 a, ar_f32 b)
{
	struct ar_f32_up x = ar_f32_unpack(a);
	struct ar_f32_up y = ar_f32_unpack(b);
	u64 q, rem;

	if (!x.sig || !y.sig)
		return (x.sign ^ y.sign) ? 0x80000000 : 0;

	/*
	 * 30 extra bits leave the quotient with at least 30 significant bits,
	 * comfortably above the 24 the result keeps, and the remainder carries
	 * the sticky bit that makes the rounding correct.
	 */
	q = div64_u64_rem(x.sig << 30, y.sig, &rem);

	return ar_f32_round_pack(x.sign ^ y.sign, q, x.exp - y.exp - 30,
				 rem != 0);
}

/*
 * fma(a, b, c), rounded once. This is the operation the vendor's fmadd
 * performs; rounding the product separately would give a different bank on
 * some words.
 */
static inline ar_f32 ar_f32_fma(ar_f32 a, ar_f32 b, ar_f32 c)
{
	struct ar_f32_up x = ar_f32_unpack(a);
	struct ar_f32_up y = ar_f32_unpack(b);
	struct ar_f32_up p;

	/*
	 * No early return on a zero product: fma(1, 0, -0) is +0, which only
	 * falls out if the zero goes through the addition's sign rules.
	 */
	p.sign = x.sign ^ y.sign;
	p.sig = x.sig * y.sig;
	p.exp = x.exp + y.exp;

	return ar_f32_add_up(p, ar_f32_unpack(c));
}

/* fcvtzs: truncate toward zero. */
static inline s32 ar_f32_to_s32_trunc(ar_f32 a)
{
	struct ar_f32_up x = ar_f32_unpack(a);
	u64 mag;

	if (!x.sig)
		return 0;

	if (x.exp >= 0) {
		if (x.exp > 40)
			return x.sign ? S32_MIN : S32_MAX;

		mag = x.sig << x.exp;
	} else {
		if (-x.exp > 63)
			return 0;

		mag = x.sig >> -x.exp;
	}

	if (mag > (u64)S32_MAX)
		return x.sign ? S32_MIN : S32_MAX;

	return x.sign ? -(s32)mag : (s32)mag;
}

#endif /* AR_ISP_SOFTFLOAT_H */
