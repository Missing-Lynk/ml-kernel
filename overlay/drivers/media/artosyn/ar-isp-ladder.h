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
 * are powers of two from 1 to 2048. It is delivered by the 3A object, and
 * which physical gain quantity feeds it is not settled; the vendor's live
 * registers demand a value 4.5x below the sensor's analog multiplier
 * (plans/au-blend-engine-and-notch.md section 2). Callers therefore pass the
 * abscissa explicitly.
 *
 * Three stages are carried, one header each: ar-isp-rnr.h, ar-isp-lnr.h and
 * ar-isp-de3d.h. Each holds its stage's blob offsets, its register bank map
 * and its packer, and instantiates one struct ar_isp_ladder for the selector
 * here. The user-strength stages of the vendor drivers are the identity at
 * their default of 50 and nothing in this stack sets a strength, so they are
 * not carried.
 *
 * Pure data transforms: no register access and no kernel API beyond the
 * integer types, so the same source can be compiled host-side against the
 * captures.
 */

#ifndef AR_ISP_LADDER_H
#define AR_ISP_LADDER_H

#include "ar-isp-bytes.h"

/* Header words at the same offset in all three stages. */
#define AR_ISP_LADDER_HDR_ENABLE	0x00
#define AR_ISP_LADDER_HDR_INTERP	0x04

/* One band-edge record: a [low, high] float32 pair. */
#define AR_ISP_LADDER_BAND_STRIDE	8
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
 * The Q16/Q24 arithmetic differs from the vendor's float32 by less than one
 * part in 2^16 of a band width, which moves a truncated payload word only if
 * the exact blend lands within that distance of an integer; every measured
 * register state reproduces exactly.
 */
static inline void ar_isp_ladder_select(const struct ar_isp_ladder *ladder,
					const u8 *blob, u32 gain_q16,
					unsigned int *band_out, u32 *t_q24_out)
{
	const u8 *hdr = blob + ladder->hdr;
	const u8 *bands = blob + ladder->bands;
	u32 count = ar_isp_get_le32(hdr + ladder->count_off);
	u32 interp = ar_isp_get_le32(hdr + AR_ISP_LADDER_HDR_INTERP);
	unsigned int band;
	u32 t_q24 = 0;

	if (count < 1 || count > ladder->max_bands)
		count = ladder->max_bands;

	for (band = 0; band < count - 1; band++)
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
