/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-ladder.h - gain-keyed register ladders from the tuning file.
 *
 * The gain-keyed stages are register banks, not DMA pages. Each recomputes its
 * bank from a per-stage ladder in the tuning file whenever the 3A loop moves
 * the gain. The shared shape, from the per-stage drivers in libmpp_service.so
 * (rnr's is 0x1993b8): a header of enable, interpolate, mode, band count and
 * abscissa selector; a band array of [low, high] float32 pairs; one payload
 * record per band. A gain inside a band selects its payload verbatim; a gain
 * between band i's high edge and band i+1's low edge blends the two payloads
 * linearly and truncates toward zero on the way to the registers.
 *
 * The abscissa is a linear gain multiplier with unity at 1.0: the band edges
 * are powers of two from 1 to 2048. It is delivered by the 3A object, and
 * which physical gain quantity feeds it is not settled; the vendor's live
 * registers demand a value 4.5x below the sensor's analog multiplier
 * (plans/au-blend-engine-and-notch.md section 2). Callers therefore pass the
 * abscissa explicitly.
 *
 * Only rnr is carried. Its transform is validated against both measured
 * points: our replayed cold bank is band 0 verbatim, and the vendor's live
 * 0x002e002d registers reproduce at abscissa 13.6 to 14.2. The user-strength
 * stage of the vendor driver is the identity at its default of 50 and nothing
 * in this stack sets a strength, so it is not carried.
 *
 * The runnable proof is kernel/scripts/check-rnr-ladder.py: the same
 * arithmetic in Python, refusing to pass if it stops reproducing both
 * measured register states from the tuning file.
 *
 * Pure data transforms: no register access and no kernel API beyond the
 * integer types, so the same source can be compiled host-side against the
 * captures.
 */

#ifndef AR_ISP_LADDER_H
#define AR_ISP_LADDER_H

#include "ar-isp-bytes.h"

/*
 * rnr register bank. The twelve ladder-fed registers sit at +0x08; each packs
 * one payload pair as (high << 16) | low. Bank +0x00 bit 1 carries the header
 * mode flag, which reads 0 in this blob and is already 0 in the replayed bank
 * word, so the apply path leaves +0x00 to the replay.
 */
#define AR_ISP_RNR_BANK			0x1800
#define AR_ISP_RNR_LADDER		0x08
#define AR_ISP_RNR_REGS			12

/*
 * rnr ladder in the tuning file. The header words are enable, interpolate,
 * mode, band count, abscissa selector. Twelve bands are allocated; the layout
 * is the fixed structure shared by all three sensors' files.
 *
 * Payload words 2..13 feed the low register halves and 14..25 the high
 * halves, one word per register.
 */
#define AR_ISP_RNR_BLOB_HEADER		0x79d8
#define AR_ISP_RNR_HDR_ENABLE		0x00
#define AR_ISP_RNR_HDR_INTERP		0x04
#define AR_ISP_RNR_HDR_MODE		0x08
#define AR_ISP_RNR_HDR_COUNT		0x0c
#define AR_ISP_RNR_HDR_SELECT		0x10
#define AR_ISP_RNR_BLOB_BANDS		0x79ec
#define AR_ISP_RNR_BLOB_PAYLOAD		0x7a6c
#define AR_ISP_RNR_BLOB_STRIDE		0x160
#define AR_ISP_RNR_BANDS		12
#define AR_ISP_RNR_LO_WORD		2
#define AR_ISP_RNR_HI_WORD		14

#define AR_ISP_LADDER_T_ONE		(1u << 24)

/*
 * A band edge from an IEEE-754 single, as unsigned Q16 truncated toward zero.
 * Edges run 1.0 to 2048.1, well inside the u32 Q16 range; a negative or a
 * value at or beyond 32768 is a corrupt file and clamps.
 */
static inline u32 ar_isp_f32_q16(u32 bits)
{
	u32 mant = (bits & 0x7fffff) | 0x800000;
	int exp = (int)((bits >> 23) & 0xff) - 127;
	int shift = 7 - exp;

	if (bits & 0x80000000)
		return 0;

	if (shift >= 32)
		return 0;

	if (shift < -8)
		return 0xffffffff;

	if (shift <= 0)
		return mant << -shift;

	return mant >> shift;
}

/*
 * a*(1-t) + b*t, truncated toward zero. Evaluated as one Q24 sum before the
 * shift so the truncation lands on the blended value, as the vendor's fcvtzs
 * does on its float result; both operands are non-negative in every payload,
 * so the floor of the shift is the same truncation.
 */
static inline u32 ar_isp_ladder_blend(u32 a, u32 b, u32 t_q24)
{
	return (u32)(((u64)a * (AR_ISP_LADDER_T_ONE - t_q24) +
		      (u64)b * t_q24) >> 24);
}

/*
 * Build the twelve rnr ladder registers from the tuning file at the given
 * abscissa.
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
 * the exact blend lands within that distance of an integer; both measured
 * register states reproduce exactly.
 */
static inline void ar_isp_rnr_from_blob(u32 *dst, const u8 *blob, u32 gain_q16)
{
	const u8 *hdr = blob + AR_ISP_RNR_BLOB_HEADER;
	const u8 *bands = blob + AR_ISP_RNR_BLOB_BANDS;
	const u8 *pay = blob + AR_ISP_RNR_BLOB_PAYLOAD;
	u32 count = ar_isp_get_le32(hdr + AR_ISP_RNR_HDR_COUNT);
	u32 interp = ar_isp_get_le32(hdr + AR_ISP_RNR_HDR_INTERP);
	u32 t_q24 = 0;
	unsigned int band, k;

	if (count < 1 || count > AR_ISP_RNR_BANDS)
		count = AR_ISP_RNR_BANDS;

	for (band = 0; band < count - 1; band++)
		if (gain_q16 <= ar_isp_f32_q16(ar_isp_get_le32(bands + band * 8 + 4)))
			break;

	if (interp && band > 0) {
		u32 lo = ar_isp_f32_q16(ar_isp_get_le32(bands + band * 8));
		u32 prev_hi = ar_isp_f32_q16(ar_isp_get_le32(bands + (band - 1) * 8 + 4));

		if (gain_q16 < lo && lo > prev_hi)
			t_q24 = (u32)(((u64)(gain_q16 - prev_hi) << 24) /
				      (lo - prev_hi));
	}

	for (k = 0; k < AR_ISP_RNR_REGS; k++) {
		u32 lo = ar_isp_get_le32(pay + band * AR_ISP_RNR_BLOB_STRIDE +
					 (AR_ISP_RNR_LO_WORD + k) * 4);
		u32 hi = ar_isp_get_le32(pay + band * AR_ISP_RNR_BLOB_STRIDE +
					 (AR_ISP_RNR_HI_WORD + k) * 4);

		if (t_q24) {
			const u8 *prev = pay + (band - 1) * AR_ISP_RNR_BLOB_STRIDE;

			lo = ar_isp_ladder_blend(ar_isp_get_le32(prev +
					(AR_ISP_RNR_LO_WORD + k) * 4), lo, t_q24);
			hi = ar_isp_ladder_blend(ar_isp_get_le32(prev +
					(AR_ISP_RNR_HI_WORD + k) * 4), hi, t_q24);
		}

		dst[k] = (hi << 16) | (lo & 0xffff);
	}
}

#endif /* AR_ISP_LADDER_H */
