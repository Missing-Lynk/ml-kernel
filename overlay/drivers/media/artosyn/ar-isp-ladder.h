/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-ladder.h - the shared shape of the gain-keyed tuning-file ladders.
 *
 * The gain-keyed stages are register banks. Each recomputes its bank from a
 * per-stage ladder in the tuning file whenever the 3A loop moves the gain.
 * The shared shape, from the per-stage drivers in libmpp_service.so (rnr's is
 * 0x1993b8): a header of enable, interpolate, mode, band count and abscissa
 * selector; a band array of [low, high] float32 pairs; one payload record per
 * band. A gain inside a band selects its payload verbatim; a gain between
 * band i's high edge and band i+1's low edge blends the two payloads linearly
 * and truncates toward zero on the way to the registers.
 *
 * The abscissa is a linear gain multiplier with unity at 1.0: the band edges
 * are powers of two from 1 to 2048. It is the 3A loop's commanded gain, the
 * Q8 exposure-table value of the selected entry divided by 256. Callers pass
 * it explicitly, in Q16.
 *
 * Five stages are carried, one header each: ar-isp-rnr.h, ar-isp-lnr.h,
 * ar-isp-de3d.h, ar-isp-cfa.h and ar-isp-cnf.h. Each holds its stage's blob
 * offsets, its register bank map and its packer, and instantiates one struct
 * ar_isp_ladder for the selector here, or calls the band walk directly where
 * its ladder carries no header. The user-strength stages of the vendor drivers
 * are the identity at their default of 50 and nothing in this stack sets a
 * strength, so they are not carried.
 *
 * Pure data transforms: no register access and no kernel API beyond the
 * integer types, so the same source can be compiled host-side against the
 * captures.
 */

#ifndef AR_ISP_LADDER_H
#define AR_ISP_LADDER_H

#include "vendor-tables/ar-isp-blob.h"
#include "ar-isp-bytes.h"
#include "ar-isp-softfloat.h"

/* Header words at the same offset in all three stages. */

/* One band-edge record: a [low, high] float32 pair. */
#define AR_ISP_LADDER_BAND_STRIDE	8
#define AR_ISP_LADDER_BAND_LO		0
#define AR_ISP_LADDER_BAND_HI		4

/* Band edges and the gain abscissa are unsigned Q16. */
#define AR_ISP_LADDER_Q			16

/* The blend fraction is unsigned Q24. */
#define AR_ISP_LADDER_T_SHIFT		24
#define AR_ISP_LADDER_T_ONE		(1u << AR_ISP_LADDER_T_SHIFT)

/*
 * One stage's ladder, as blob offsets. Band count sits at a different header
 * word per stage because rnr's header carries an extra mode word.
 */
struct ar_isp_ladder {
	u32 hdr;		/* header */
	u32 bands;		/* band-edge array */
	u32 payload;		/* first payload record */
	u32 stride;		/* payload record size */
	u16 count_off;		/* band count, within the header */
	u16 max_bands;		/* payload records allocated */
};

/*
 * A band edge from an IEEE-754 single, as unsigned Q16 truncated toward zero.
 * Edges run 1.0 to 2048.1, well inside the u32 Q16 range; a negative or a
 * value at or beyond 32768 is a corrupt file and clamps.
 */
static inline u32 ar_isp_f32_q16(u32 bits)
{
	u32 mant = ar_isp_f32_mant(bits);
	int shift = (AR_ISP_F32_MANT_BITS - AR_ISP_LADDER_Q) - ar_isp_f32_exp(bits);

	if (bits & AR_ISP_F32_SIGN)
		return 0;

	if (shift >= 32)
		return 0;

	if (shift < -8)
		return 0xffffffff;

	if (shift <= 0)
		return mant << -shift;

	return mant >> shift;
}

/* The low and high edges of a band, as unsigned Q16. */
static inline u32 ar_isp_ladder_lo(const u8 *bands, unsigned int band)
{
	return ar_isp_f32_q16(ar_isp_get_le32(bands +
					      band * AR_ISP_LADDER_BAND_STRIDE));
}

static inline u32 ar_isp_ladder_hi(const u8 *bands, unsigned int band)
{
	return ar_isp_f32_q16(ar_isp_get_le32(bands +
					      band * AR_ISP_LADDER_BAND_STRIDE +
					      AR_ISP_LADDER_BAND_HI));
}

/*
 * from*(1-t) + to*t, truncated toward zero. Evaluated as one Q24 sum before the
 * shift so the truncation lands on the blended value, as the vendor's fcvtzs
 * does on its float result; both operands are non-negative in every payload,
 * so the floor of the shift is the same truncation.
 */
static inline u32 ar_isp_ladder_blend(u32 from, u32 to, u32 t_q24)
{
	return (u32)(((u64)from * (AR_ISP_LADDER_T_ONE - t_q24) +
		      (u64)to * t_q24) >> AR_ISP_LADDER_T_SHIFT);
}

static inline s32 ar_isp_ladder_blend_s32(s32 from, s32 to, u32 t_q24)
{
	return (s32)(((s64)from * (AR_ISP_LADDER_T_ONE - t_q24) +
		      (s64)to * t_q24) / AR_ISP_LADDER_T_ONE);
}

/*
 * The band a gain falls in, and the fraction into the gap above it.
 *
 * Band selection walks the [low, high] edges: at or below a band's high edge
 * the gain belongs to that band, verbatim if it is also at or above the low
 * edge. Between two bands the fraction is (g - high_i) / (low_i+1 - high_i).
 * A gain below band 0 or above the last high edge clamps to the end band; the
 * vendor's own behaviour outside the ladder is unmeasured, and the clamp is
 * the interpolation's natural limit.
 *
 * The Q16/Q24 arithmetic here is NOT the vendor's. isp_sub_cfa blends in float
 * and truncates, so an exact integer blend is wrong by one wherever the true
 * value lands just under an integer, including where both records are equal:
 * blending 75 into 75 gives 74 on hardware. cfa carries the faithful version in
 * ar-isp-cfa.h on top of ar-isp-softfloat.h, measured against two gap captures.
 *
 * rnr, lnr, de3d and cfa use these. Their drivers issue the same five
 * instructions per word:
 *
 *     scvtf s3, w4            the upper record's word
 *     scvtf s2, w3            the lower record's word
 *     fmul  s3, s3, s0        s0 is t
 *     fmadd s2, s1, s2, s3    s1 is 1 - t, and the add is fused
 *     fcvtzs w3, s2           truncate toward zero
 *
 * The integer forms above serve cm and cm2, whose drivers at 0x19fe28 and
 * 0x1a14ec carry no float instruction.
 */
static inline void ar_isp_ladder_walk(const u8 *bands, u32 count, u32 interp,
				      u32 gain_q16, unsigned int *band_out,
				      u32 *t_q24_out)
{
	unsigned int band;
	u32 t_q24 = 0;

	/*
	 * band + 1 < count rather than band < count - 1: count is unsigned, so
	 * an empty ladder would underflow the bound and walk the whole u32
	 * range off the end of the array. The two forms agree for every count
	 * a caller passes.
	 */
	for (band = 0; band + 1 < count; band++)
		if (gain_q16 <= ar_isp_ladder_hi(bands, band))
			break;

	if (interp && band > 0) {
		u32 lo_edge = ar_isp_ladder_lo(bands, band);
		u32 prev_hi_edge = ar_isp_ladder_hi(bands, band - 1);

		if (gain_q16 < lo_edge && lo_edge > prev_hi_edge)
			t_q24 = (u32)(((u64)(gain_q16 - prev_hi_edge) <<
				       AR_ISP_LADDER_T_SHIFT) /
				      (lo_edge - prev_hi_edge));
	}

	*band_out = band;
	*t_q24_out = t_q24;
}

/*
 * The same walk for a stage whose ladder carries the standard four-word header,
 * taking the band count and the interpolate flag from the file. A count outside
 * the allocated band array is a corrupt file and falls back to the allocation.
 */
static inline void ar_isp_ladder_select(const struct ar_isp_ladder *ladder,
					const u8 *blob, u32 gain_q16,
					unsigned int *band_out, u32 *t_q24_out)
{
	const u8 *hdr = blob + ladder->hdr;
	u32 count = ar_isp_get_le32(hdr + ladder->count_off);

	if (count < 1 || count > ladder->max_bands)
		count = ladder->max_bands;

	ar_isp_ladder_walk(blob + ladder->bands, count,
			   ar_isp_get_le32(hdr + AR_ISP_LADDER_HDR_INTERP),
			   gain_q16, band_out, t_q24_out);
}

/*
 * The vendor's arithmetic: band selection and the gap fraction in binary32,
 * then a fused blend truncated toward zero. See ar-isp-softfloat.h for why the
 * integer forms above are not interchangeable with these.
 *
 * A zero t means the record is used verbatim, which is the memcpy path the
 * vendor takes inside a band.
 */
struct ar_isp_ladder_frac {
	ar_f32 t;
	ar_f32 omt;
};

/* A band edge as its raw binary32, which is how the tuning file stores it. */
static inline ar_f32 ar_isp_ladder_edge(const u8 *bands, unsigned int band,
					unsigned int half)
{
	return ar_isp_get_le32(bands + band * AR_ISP_LADDER_BAND_STRIDE + half);
}

/*
 * IEEE binary32 orders exactly as its unsigned bit pattern over non-negative
 * values, and both the abscissa and every band edge are non-negative, so the
 * edge comparisons need no unpacking.
 */
static inline void ar_isp_ladder_walk_f32(const u8 *bands, u32 count,
					  u32 interp, u32 gain_q16,
					  unsigned int *band_out,
					  struct ar_isp_ladder_frac *frac)
{
	ar_f32 gain = ar_f32_scale_down(ar_f32_from_s32((s32)gain_q16), 16);
	unsigned int band;

	for (band = 0; band + 1 < count; band++)
		if (gain <= ar_isp_ladder_edge(bands, band,
					       AR_ISP_LADDER_BAND_HI))
			break;

	frac->t = 0;
	frac->omt = 0;

	if (interp && band > 0) {
		ar_f32 lo = ar_isp_ladder_edge(bands, band,
					       AR_ISP_LADDER_BAND_LO);
		ar_f32 prev_hi = ar_isp_ladder_edge(bands, band - 1,
						    AR_ISP_LADDER_BAND_HI);

		if (gain < lo && lo > prev_hi) {
			frac->t = ar_f32_div(ar_f32_sub(gain, prev_hi),
					     ar_f32_sub(lo, prev_hi));
			frac->omt = ar_f32_sub(ar_f32_from_s32(1), frac->t);
		}
	}

	*band_out = band;
}

static inline void ar_isp_ladder_select_f32(const struct ar_isp_ladder *ladder,
					    const u8 *blob, u32 gain_q16,
					    unsigned int *band_out,
					    struct ar_isp_ladder_frac *frac)
{
	const u8 *hdr = blob + ladder->hdr;
	u32 count = ar_isp_get_le32(hdr + ladder->count_off);

	if (count < 1 || count > ladder->max_bands)
		count = ladder->max_bands;

	ar_isp_ladder_walk_f32(blob + ladder->bands, count,
			       ar_isp_get_le32(hdr + AR_ISP_LADDER_HDR_INTERP),
			       gain_q16, band_out, frac);
}

/* One fused blend, as the vendor's fmul-then-fmadd pair computes it. */
static inline s32 ar_isp_ladder_blend_f32(s32 from, s32 to,
					  struct ar_isp_ladder_frac frac)
{
	return ar_f32_to_s32_trunc(ar_f32_fma(frac.omt, ar_f32_from_s32(from),
					      ar_f32_mul(frac.t,
							 ar_f32_from_s32(to))));
}

/* One payload word at the selected band, blended into the band above it. */
static inline u32 ar_isp_ladder_read_word_f32(const struct ar_isp_ladder *ladder,
					      const u8 *payload,
					      unsigned int band,
					      unsigned int off,
					      struct ar_isp_ladder_frac frac)
{
	s32 word = (s32)ar_isp_get_le32(payload + band * ladder->stride + off);

	if (frac.t) {
		s32 prev = (s32)ar_isp_get_le32(payload +
						(band - 1) * ladder->stride +
						off);

		word = ar_isp_ladder_blend_f32(prev, word, frac);
	}

	return (u32)word;
}

/* One payload word at the selected band, blended into the band above it. */
static inline u32 ar_isp_ladder_read_word(const struct ar_isp_ladder *ladder,
					  const u8 *payload, unsigned int band,
					  unsigned int off, u32 t_q24)
{
	s32 word = (s32)ar_isp_get_le32(payload + band * ladder->stride + off);

	if (t_q24) {
		s32 prev = (s32)ar_isp_get_le32(payload +
						(band - 1) * ladder->stride +
						off);

		word = ar_isp_ladder_blend_s32(prev, word, t_q24);
	}

	return (u32)word;
}

#endif /* AR_ISP_LADDER_H */
