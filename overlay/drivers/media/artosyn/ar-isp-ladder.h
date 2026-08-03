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
 * Three stages are carried: rnr (driver 0x1993b8, validated against the cold
 * bank at abscissa 1.0 and the vendor's live 0x002e002d at 13.6 to 14.2),
 * lnr (0x1bd8a8, 84 of 85 registers bit-exact at abscissas 5.59375 and
 * 15.3828125 against the vendor captures) and de3d (driver 0x1c6c10, packer
 * 0x1c61f8, every capture-covered register bit-exact at the same two
 * abscissas). The user-strength stages of the vendor drivers are the identity
 * at their default of 50 and nothing in this stack sets a strength, so they
 * are not carried.
 *
 * The runnable proofs are kernel/scripts/check-rnr-ladder.py,
 * check-lnr-ladder.py and check-de3d-ladder.py: the same arithmetic in
 * Python, refusing to pass if it stops reproducing the measured register
 * states from the tuning file.
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

/*
 * lnr register bank. The ladder owns 85 of the 86 words in 0x3cc8..0x3e1c:
 * 0x3d10 is never written by the vendor packer, and 0x3d14 stays on the
 * replay path because the vendor applies an unresolved fixed bias before pack.
 */
#define AR_ISP_LNR_BANK			0x3cc8
#define AR_ISP_LNR_REGS			86
#define AR_ISP_LNR_SKIP_NEVER_WRITTEN	((0x3d10 - AR_ISP_LNR_BANK) / 4)
#define AR_ISP_LNR_SKIP_BIASED		((0x3d14 - AR_ISP_LNR_BANK) / 4)

#define AR_ISP_LNR_BLOB_HEADER		0x89e88
#define AR_ISP_LNR_HDR_ENABLE		0x00
#define AR_ISP_LNR_HDR_INTERP		0x04
#define AR_ISP_LNR_HDR_COUNT		0x08
#define AR_ISP_LNR_HDR_SELECT		0x0c
#define AR_ISP_LNR_BLOB_BANDS		0x89e98
#define AR_ISP_LNR_BLOB_PAYLOAD		0x89f18
#define AR_ISP_LNR_BLOB_STRIDE		0x428
#define AR_ISP_LNR_BANDS		11

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

static inline s32 ar_isp_ladder_blend_s32(s32 a, s32 b, u32 t_q24)
{
	return (s32)(((s64)a * (AR_ISP_LADDER_T_ONE - t_q24) +
		      (s64)b * t_q24) / AR_ISP_LADDER_T_ONE);
}

static inline u32 ar_isp_ladder_read_word(const u8 *pay, unsigned int band,
					  unsigned int stride, unsigned int off,
					  u32 t_q24)
{
	s32 v = (s32)ar_isp_get_le32(pay + band * stride + off);

	if (t_q24) {
		s32 prev = (s32)ar_isp_get_le32(pay + (band - 1) * stride + off);

		v = ar_isp_ladder_blend_s32(prev, v, t_q24);
	}

	return (u32)v;
}

static inline void ar_isp_lnr_pack_field(u32 *dst, const u8 *pay,
					 unsigned int band, u32 t_q24,
					 unsigned int reg, unsigned int off,
					 unsigned int shift, unsigned int width)
{
	u32 mask = width == 32 ? 0xffffffff : (1u << width) - 1;
	u32 v = ar_isp_ladder_read_word(pay, band, AR_ISP_LNR_BLOB_STRIDE,
					off, t_q24);

	dst[reg] &= ~(mask << shift);
	dst[reg] |= (v & mask) << shift;
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

static inline void ar_isp_lnr_select(const u8 *blob, u32 gain_q16,
				     unsigned int *band_out, u32 *t_q24_out)
{
	const u8 *hdr = blob + AR_ISP_LNR_BLOB_HEADER;
	const u8 *bands = blob + AR_ISP_LNR_BLOB_BANDS;
	u32 count = ar_isp_get_le32(hdr + AR_ISP_LNR_HDR_COUNT);
	u32 interp = ar_isp_get_le32(hdr + AR_ISP_LNR_HDR_INTERP);
	unsigned int band;
	u32 t_q24 = 0;

	if (count < 1 || count > AR_ISP_LNR_BANDS)
		count = AR_ISP_LNR_BANDS;

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

	*band_out = band;
	*t_q24_out = t_q24;
}

static inline void ar_isp_lnr_from_blob(u32 *dst, const u8 *blob, u32 gain_q16)
{
	const u8 *pay = blob + AR_ISP_LNR_BLOB_PAYLOAD;
	unsigned int band;
	u32 t_q24;
	unsigned int i;

	ar_isp_lnr_select(blob, gain_q16, &band, &t_q24);

	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 0, 0x000, 2, 1);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 0, 0x004, 6, 1);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 0, 0x008, 8, 1);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 0, 0x00c, 9, 1);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 0, 0x010, 12, 1);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 0, 0x014, 24, 8);

	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 1, 0x018, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 1, 0x028, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 2, 0x038, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 2, 0x048, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 3, 0x01c, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 3, 0x02c, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 4, 0x03c, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 4, 0x04c, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 5, 0x020, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 5, 0x030, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 6, 0x040, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 6, 0x050, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 7, 0x024, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 7, 0x034, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 8, 0x044, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 8, 0x054, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 9, 0x058, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 9, 0x068, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 10, 0x078, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 10, 0x05c, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 11, 0x06c, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 11, 0x07c, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 12, 0x060, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 12, 0x070, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 13, 0x080, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 13, 0x064, 16, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 14, 0x074, 0, 13);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 14, 0x084, 16, 13);

	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 15, 0x210, 0, 8);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 15, 0x214, 8, 8);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 16, 0x218, 0, 8);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 16, 0x21c, 8, 8);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 17, 0x220, 0, 8);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 17, 0x224, 8, 8);

	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 20, 0x090, 0, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 20, 0x094, 16, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 21, 0x098, 0, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 21, 0x09c, 10, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 22, 0x0a0, 0, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 22, 0x0a4, 10, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 23, 0x0a8, 0, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 23, 0x0ac, 10, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 24, 0x0b0, 0, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 24, 0x0b4, 10, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 25, 0x0b8, 0, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 26, 0x0bc, 0, 16);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 26, 0x0c0, 16, 16);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 27, 0x0c4, 0, 16);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 27, 0x0c8, 16, 16);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 28, 0x0cc, 0, 16);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 29, 0x0d0, 0, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 29, 0x0d4, 10, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 30, 0x0d8, 0, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 30, 0x0dc, 10, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 31, 0x0e0, 0, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 31, 0x0e4, 10, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 32, 0x0e8, 0, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 32, 0x0ec, 10, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 33, 0x0f0, 0, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 34, 0x0f4, 0, 16);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 34, 0x0f8, 16, 16);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 35, 0x0fc, 0, 16);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 35, 0x100, 16, 16);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 36, 0x104, 0, 16);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 37, 0x108, 0, 10);
	ar_isp_lnr_pack_field(dst, pay, band, t_q24, 37, 0x10c, 16, 10);

	for (i = 0; i < 48; i++) {
		unsigned int reg = 38 + i;
		unsigned int off = i < 16 ? 0x110 + i * 16 : 0x228 + (i - 16) * 16;

		ar_isp_lnr_pack_field(dst, pay, band, t_q24, reg, off, 0, 8);
		ar_isp_lnr_pack_field(dst, pay, band, t_q24, reg, off + 4, 8, 8);
		ar_isp_lnr_pack_field(dst, pay, band, t_q24, reg, off + 8, 16, 8);
		ar_isp_lnr_pack_field(dst, pay, band, t_q24, reg, off + 12, 24, 8);
	}
}

/*
 * de3d register bank. The ladder owns 45 registers: thirteen bit-packed
 * scalar registers between 0x2e10 and 0x2eb0 and the 32-register byte-packed
 * curve at 0x2ebc..0x2f38. Seven of the scalars carry bits owned by other
 * producers (the user-strength cluster, self-preserved control bits), so each
 * register carries a mask of the ladder-owned bits and the apply path
 * read-modify-writes. The buffer-address registers stay with
 * ar_isp_de3d_publish and the strength registers with the replay.
 *
 * The payload record is 0x2f8 bytes: word fields to 0xdc, a byte-per-word
 * curve at 0xe0..0x2dc, four flag words at 0x2e0..0x2ec and two more fields
 * at 0x2f0/0x2f4. Between bands the vendor blends the words and the curve;
 * the four flag words come verbatim from the upper record, and working-block
 * word 0x90 blends the lower record's 0x8c against the upper record's 0x90
 * (the packer's one asymmetric input, measured at both capture points). The
 * two knee registers divide 65532 by a blended field difference, truncating
 * toward zero as the vendor's sdiv does.
 */
#define AR_ISP_DE3D_BANK		0x2e00
#define AR_ISP_DE3D_REGS		45
#define AR_ISP_DE3D_CURVE_REGS		32

#define AR_ISP_DE3D_BLOB_HEADER		0x9631c
#define AR_ISP_DE3D_HDR_ENABLE		0x00
#define AR_ISP_DE3D_HDR_INTERP		0x04
#define AR_ISP_DE3D_HDR_COUNT		0x08
#define AR_ISP_DE3D_HDR_SELECT		0x0c
#define AR_ISP_DE3D_BLOB_BANDS		0x9632c
#define AR_ISP_DE3D_BLOB_PAYLOAD	0x963ac
#define AR_ISP_DE3D_BLOB_STRIDE		0x2f8
#define AR_ISP_DE3D_BANDS		12

struct ar_isp_de3d_reg {
	u16 off;
	u32 mask;
};

static const struct ar_isp_de3d_reg ar_isp_de3d_regs[AR_ISP_DE3D_REGS] = {
	{ 0x10, 0xffffffff },
	{ 0x14, 0xffffffff },
	{ 0x18, 0xffffffff },
	{ 0x28, 0x000001ff },
	{ 0x2c, 0xffffffff },
	{ 0x4c, 0x00003fff },
	{ 0x94, 0x00ffffff },
	{ 0x9c, 0x00ffffff },
	{ 0xa0, 0xffffffff },
	{ 0xa4, 0x03ff1fff },
	{ 0xa8, 0x03ff0fff },
	{ 0xac, 0x000001ff },
	{ 0xb0, 0xffffffff },
	{ 0xbc, 0xffffffff },
	{ 0xc0, 0xffffffff },
	{ 0xc4, 0xffffffff },
	{ 0xc8, 0xffffffff },
	{ 0xcc, 0xffffffff },
	{ 0xd0, 0xffffffff },
	{ 0xd4, 0xffffffff },
	{ 0xd8, 0xffffffff },
	{ 0xdc, 0xffffffff },
	{ 0xe0, 0xffffffff },
	{ 0xe4, 0xffffffff },
	{ 0xe8, 0xffffffff },
	{ 0xec, 0xffffffff },
	{ 0xf0, 0xffffffff },
	{ 0xf4, 0xffffffff },
	{ 0xf8, 0xffffffff },
	{ 0xfc, 0xffffffff },
	{ 0x100, 0xffffffff },
	{ 0x104, 0xffffffff },
	{ 0x108, 0xffffffff },
	{ 0x10c, 0xffffffff },
	{ 0x110, 0xffffffff },
	{ 0x114, 0xffffffff },
	{ 0x118, 0xffffffff },
	{ 0x11c, 0xffffffff },
	{ 0x120, 0xffffffff },
	{ 0x124, 0xffffffff },
	{ 0x128, 0xffffffff },
	{ 0x12c, 0xffffffff },
	{ 0x130, 0xffffffff },
	{ 0x134, 0xffffffff },
	{ 0x138, 0xffffffff },
};

static inline void ar_isp_de3d_select(const u8 *blob, u32 gain_q16,
				      unsigned int *band_out, u32 *t_q24_out)
{
	const u8 *hdr = blob + AR_ISP_DE3D_BLOB_HEADER;
	const u8 *bands = blob + AR_ISP_DE3D_BLOB_BANDS;
	u32 count = ar_isp_get_le32(hdr + AR_ISP_DE3D_HDR_COUNT);
	u32 interp = ar_isp_get_le32(hdr + AR_ISP_DE3D_HDR_INTERP);
	unsigned int band;
	u32 t_q24 = 0;

	if (count < 1 || count > AR_ISP_DE3D_BANDS)
		count = AR_ISP_DE3D_BANDS;

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

	*band_out = band;
	*t_q24_out = t_q24;
}

static inline u32 ar_isp_de3d_field(const u8 *pay, unsigned int band,
				    u32 t_q24, unsigned int off,
				    unsigned int width)
{
	u32 v = ar_isp_ladder_read_word(pay, band, AR_ISP_DE3D_BLOB_STRIDE,
					off, t_q24);

	return v & ((1u << width) - 1);
}

static inline u32 ar_isp_de3d_slope(const u8 *pay, unsigned int band,
				    u32 t_q24, unsigned int hi_off,
				    unsigned int lo_off)
{
	s32 d = (s32)ar_isp_ladder_read_word(pay, band, AR_ISP_DE3D_BLOB_STRIDE,
					     hi_off, t_q24) -
		(s32)ar_isp_ladder_read_word(pay, band, AR_ISP_DE3D_BLOB_STRIDE,
					     lo_off, t_q24);

	if (!d)
		return 0;

	return (u32)(65532 / d) & 0xffff;
}

static inline void ar_isp_de3d_from_blob(u32 *dst, const u8 *blob, u32 gain_q16)
{
	const u8 *pay = blob + AR_ISP_DE3D_BLOB_PAYLOAD;
	unsigned int band, i;
	u32 t_q24;
	u32 asym;

	ar_isp_de3d_select(blob, gain_q16, &band, &t_q24);

	/*
	 * The asymmetric input: the lower record's 0x8c against the upper
	 * record's 0x90. Verbatim inside a band, like every other field.
	 */
	asym = (u32)ar_isp_get_le32(pay + band * AR_ISP_DE3D_BLOB_STRIDE + 0x90);
	if (t_q24)
		asym = (u32)ar_isp_ladder_blend_s32(
			(s32)ar_isp_get_le32(pay +
				(band - 1) * AR_ISP_DE3D_BLOB_STRIDE + 0x8c),
			(s32)asym, t_q24);
	asym &= 0xff;

#define F(off, w)	ar_isp_de3d_field(pay, band, t_q24, (off), (w))
#define V(off, w)	ar_isp_de3d_field(pay, band, 0, (off), (w))
#define S(hi, lo)	ar_isp_de3d_slope(pay, band, t_q24, (hi), (lo))
	dst[0] = (S(0x18, 0x14) << 16) | (F(0x14, 8) << 8) | F(0x18, 8);
	dst[1] = (F(0x1c, 8) << 24) | (F(0x20, 8) << 16) | (F(0x24, 8) << 8) |
		 F(0x28, 8);
	dst[2] = (S(0x30, 0x2c) << 16) | (F(0x2c, 8) << 8) | F(0x30, 8);
	dst[3] = F(0x44, 9);
	dst[4] = (F(0x48, 8) << 24) | (F(0x4c, 8) << 16) | (F(0x50, 8) << 8) |
		 F(0x54, 8);
	dst[5] = F(0x5c, 14);
	dst[6] = (F(0x6c, 8) << 16) | (F(0x70, 8) << 8) | F(0x74, 8);
	dst[7] = F(0x7c, 1) | (F(0x80, 1) << 1) | (F(0x84, 1) << 2) |
		 (F(0x88, 1) << 3) | (V(0x2e0, 1) << 4) | (V(0x2e4, 1) << 5) |
		 (V(0x2e8, 1) << 6) | (V(0x2ec, 1) << 7) | (F(0x2f0, 8) << 8) |
		 (F(0x2f4, 8) << 16);
	dst[8] = F(0x8c, 8) | (asym << 8) | (F(0x94, 8) << 16) |
		 (F(0x98, 8) << 24);
	dst[9] = F(0x9c, 13) | (F(0xa0, 10) << 16);
	dst[10] = F(0xa4, 12) | (F(0xa8, 10) << 16);
	dst[11] = F(0xac, 9);
	dst[12] = (F(0xb0, 8) << 24) | (F(0xb4, 8) << 16) | (F(0xb8, 8) << 8) |
		  F(0xbc, 8);

	for (i = 0; i < AR_ISP_DE3D_CURVE_REGS; i++) {
		unsigned int base = 0xe0 + i * 16;

		dst[13 + i] = (F(base, 8) << 24) | (F(base + 4, 8) << 16) |
			      (F(base + 8, 8) << 8) | F(base + 12, 8);
	}
#undef F
#undef V
#undef S
}

#endif /* AR_ISP_LADDER_H */
