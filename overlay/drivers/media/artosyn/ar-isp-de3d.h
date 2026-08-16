/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-de3d.h - the de3d gain ladder.
 *
 * Recovered from the de3d driver in libmpp_service.so at 0x1c6c10 and its
 * packer at 0x1c61f8: every capture-covered register comes out bit-exact at
 * abscissas 5.59375 and 15.3828125. The runnable proof is
 * kernel/scripts/isp/check-de3d-ladder.py, which reruns this arithmetic in
 * Python and refuses to pass if it stops reproducing the measured register
 * states from the tuning file.
 *
 * Pure data transforms: no register access and no kernel API beyond the
 * integer types, so the same source can be compiled host-side against the
 * captures. See ar-isp-ladder.h for the ladder shape.
 */

#ifndef AR_ISP_DE3D_H
#define AR_ISP_DE3D_H

#include "vendor-tables/ar-isp-blob.h"
#include "ar-isp-ladder.h"

/*
 * Register bank. The ladder owns 50 registers: the control word at +0x00 (two
 * enable bits), eighteen bit-packed scalar registers between 0x2e10 and
 * 0x2eb4 and the 32-register byte-packed curve at 0x2ebc..0x2f38. Registers
 * with bits owned elsewhere carry a mask of the ladder-owned bits and the
 * apply path read-modify-writes. The buffer-address registers stay with
 * ar_isp_de3d_publish. 0x2e98 is the block's input line-time latch,
 * hardware-written and never stored by the vendor packer, so it is not here.
 * Bit 16 of 0x2e90 is set once by the vendor's init path and then preserved by
 * its apply helper; both captures read it set, so the pack carries it as a
 * constant.
 *
 * The vendor's strength stage on 0x2e1c[13:0] and 0x2e20[24:16] (ctx words 772
 * and 764, the v*s/50 law with a saturating upper branch) is the identity at
 * its default of 50 and nothing in this stack sets a strength, so it is not
 * carried, like rnr's and lnr's.
 */
#define AR_ISP_DE3D_BANK		0x2e00
#define AR_ISP_DE3D_REGS		51
#define AR_ISP_DE3D_CURVE_REGS		32

/*
 * Where the curve starts in the register array: the control word plus the
 * seventeen scalars. ar_isp_de3d_from_blob writes the scalars by index, so the
 * two halves have to add up to the whole array.
 */
#define AR_ISP_DE3D_SCALAR_REGS		19

_Static_assert(AR_ISP_DE3D_SCALAR_REGS + AR_ISP_DE3D_CURVE_REGS ==
	       AR_ISP_DE3D_REGS, "de3d scalar and curve counts must cover the bank");

/*
 * Ladder in the tuning file. The header words are enable, interpolate, band
 * count, abscissa selector.
 *
 * The payload record is 0x2f8 bytes: word fields to 0xdc, a byte-per-word
 * curve at 0xe0..0x2dc, four flag words at 0x2e0..0x2ec and two more fields at
 * 0x2f0/0x2f4. Between bands the vendor blends the words and the curve; fields
 * 0x04/0x08 and 0x7c..0x88, the four flag words and the curve tail come
 * verbatim from the upper record, and working-block word 0x90 blends the lower
 * record's 0x8c against the upper record's 0x90 (the packer's one asymmetric
 * input, measured at both capture points).
 */

/* The asymmetric blend: the lower record's 0x8c against the upper's 0x90. */
#define AR_ISP_DE3D_ASYM_LO		0x8c
#define AR_ISP_DE3D_ASYM_HI		0x90

/*
 * The two knee registers divide this by a blended field difference, truncating
 * toward zero as the vendor's sdiv does.
 */
#define AR_ISP_DE3D_SLOPE_NUM		65532

static const struct ar_isp_ladder ar_isp_de3d_ladder = {
	.hdr		= AR_ISP_DE3D_BLOB_HEADER,
	.bands		= AR_ISP_DE3D_BLOB_BANDS,
	.payload	= AR_ISP_DE3D_BLOB_PAYLOAD,
	.stride		= AR_ISP_DE3D_BLOB_STRIDE,
	.count_off	= AR_ISP_DE3D_HDR_COUNT,
	.max_bands	= AR_ISP_DE3D_BANDS,
};

struct ar_isp_de3d_reg {
	u16 off;
	u32 mask;
};

static const struct ar_isp_de3d_reg ar_isp_de3d_regs[AR_ISP_DE3D_REGS] = {
	{ 0x00, 0x0000000c },
	{ 0x10, 0xffffffff },
	{ 0x14, 0xffffffff },
	{ 0x18, 0xffffffff },
	{ 0x1c, 0x00003fff },
	{ 0x20, 0x01ff00ff },
	{ 0x28, 0x000001ff },
	{ 0x2c, 0xffffffff },
	{ 0x30, 0x000000ff },
	{ 0x4c, 0x00003fff },
	{ 0x90, 0x3c010077 },
	{ 0x94, 0x00ffffff },
	{ 0x9c, 0x00ffffff },
	{ 0xa0, 0xffffffff },
	{ 0xa4, 0x03ff1fff },
	{ 0xa8, 0x03ff0fff },
	{ 0xac, 0x000001ff },
	{ 0xb0, 0xffffffff },
	{ 0xb4, 0x000000ff },
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

/* One payload word, masked to the register field it feeds. */
static inline u32 ar_isp_de3d_field(const u8 *payload, unsigned int band,
				    u32 t_q24, unsigned int off,
				    unsigned int width)
{
	u32 word = ar_isp_ladder_read_word(&ar_isp_de3d_ladder, payload, band,
					   off, t_q24);

	return word & ((1u << width) - 1);
}

/* A knee slope: the numerator over the span between two payload fields. */
static inline u32 ar_isp_de3d_slope(const u8 *payload, unsigned int band,
				    u32 t_q24, unsigned int hi_off,
				    unsigned int lo_off)
{
	s32 span = (s32)ar_isp_ladder_read_word(&ar_isp_de3d_ladder, payload,
						band, hi_off, t_q24) -
		   (s32)ar_isp_ladder_read_word(&ar_isp_de3d_ladder, payload,
						band, lo_off, t_q24);

	if (!span)
		return 0;

	return (u32)(AR_ISP_DE3D_SLOPE_NUM / span) & 0xffff;
}

/* Build the 50 de3d ladder registers from the tuning file at the abscissa. */
static inline void ar_isp_de3d_from_blob(u32 *dst, const u8 *blob, u32 gain_q16)
{
	const u8 *payload = blob + AR_ISP_DE3D_BLOB_PAYLOAD;
	unsigned int band, i;
	u32 t_q24;

	ar_isp_ladder_select(&ar_isp_de3d_ladder, blob, gain_q16, &band, &t_q24);

	/* Verbatim inside a band, like every other field. */
	u32 asym = ar_isp_get_le32(payload + band * AR_ISP_DE3D_BLOB_STRIDE +
				   AR_ISP_DE3D_ASYM_HI);
	if (t_q24)
		asym = (u32)ar_isp_ladder_blend_s32(
			(s32)ar_isp_get_le32(payload + (band - 1) *
					     AR_ISP_DE3D_BLOB_STRIDE +
					     AR_ISP_DE3D_ASYM_LO),
			(s32)asym, t_q24);
	asym &= 0xff;

	/*
	 * F blends a field between bands, V takes it verbatim from the band's
	 * own record, S is a knee slope. Undefined again below the pack.
	 */
#define F(off, w)	ar_isp_de3d_field(payload, band, t_q24, (off), (w))
#define V(off, w)	ar_isp_de3d_field(payload, band, 0, (off), (w))
#define S(hi, lo)	ar_isp_de3d_slope(payload, band, t_q24, (hi), (lo))

	/*
	 * Register per line, in ar_isp_de3d_regs order. The trailing address is
	 * the bank register the slot lands in; the offsets inside each
	 * expression are payload fields, which are a different space.
	 */
	dst[0] = (V(0x04, 1) << 3) | (V(0x08, 1) << 2);				/* 0x2e00 */
	dst[1] = (S(0x18, 0x14) << 16) | (F(0x14, 8) << 8) | F(0x18, 8);	/* 0x2e10 */
	dst[2] = (F(0x1c, 8) << 24) | (F(0x20, 8) << 16) | (F(0x24, 8) << 8) |
		 F(0x28, 8);							/* 0x2e14 */
	dst[3] = (S(0x30, 0x2c) << 16) | (F(0x2c, 8) << 8) | F(0x30, 8);	/* 0x2e18 */
	dst[4] = F(0x34, 14);							/* 0x2e1c */
	dst[5] = (F(0x38, 9) << 16) | F(0x40, 8);				/* 0x2e20 */
	dst[6] = F(0x44, 9);							/* 0x2e28 */
	dst[7] = (F(0x48, 8) << 24) | (F(0x4c, 8) << 16) | (F(0x50, 8) << 8) |
		 F(0x54, 8);							/* 0x2e2c */
	/*
	 * A byte store in the vendor packer, strb w15, [x21, #48], so only the
	 * low byte moves and the rest of the register keeps its init value.
	 * Field 0x58 reads 252 in all twelve bands, which is why this is
	 * constant across the gain range rather than a ladder in practice.
	 */
	dst[8] = F(0x58, 8);							/* 0x2e30 */
	dst[9] = F(0x5c, 14);							/* 0x2e4c */
	dst[10] = (F(0xc8, 1) << 26) | (F(0xcc, 1) << 27) | (F(0xd0, 1) << 28) |
		 (F(0xd4, 1) << 29) | (F(0xd8, 3) << 4) | F(0xdc, 3) |
		 (1u << 16);							/* 0x2e90 */
	dst[11] = (F(0x6c, 8) << 16) | (F(0x70, 8) << 8) | F(0x74, 8);		/* 0x2e94 */
	dst[12] = V(0x7c, 1) | (V(0x80, 1) << 1) | (V(0x84, 1) << 2) |
		  (V(0x88, 1) << 3) | (V(0x2e0, 1) << 4) | (V(0x2e4, 1) << 5) |
		  (V(0x2e8, 1) << 6) | (V(0x2ec, 1) << 7) | (F(0x2f0, 8) << 8) |
		  (F(0x2f4, 8) << 16);						/* 0x2e9c */
	dst[13] = F(0x8c, 8) | (asym << 8) | (F(0x94, 8) << 16) |
		  (F(0x98, 8) << 24);						/* 0x2ea0 */
	dst[14] = F(0x9c, 13) | (F(0xa0, 10) << 16);				/* 0x2ea4 */
	dst[15] = F(0xa4, 12) | (F(0xa8, 10) << 16);				/* 0x2ea8 */
	dst[16] = F(0xac, 9);							/* 0x2eac */
	dst[17] = (F(0xb0, 8) << 24) | (F(0xb4, 8) << 16) | (F(0xb8, 8) << 8) |
		  F(0xbc, 8);							/* 0x2eb0 */
	dst[18] = F(0xc0, 8);							/* 0x2eb4 */

	/* The byte-packed curve, 0x2ebc..0x2f38. */
	for (i = 0; i < AR_ISP_DE3D_CURVE_REGS; i++) {
		unsigned int base = AR_ISP_DE3D_CURVE_OFF + i * 16;

		dst[AR_ISP_DE3D_SCALAR_REGS + i] =
			(F(base, 8) << 24) | (F(base + 4, 8) << 16) |
			(F(base + 8, 8) << 8) | F(base + 12, 8);
	}
#undef F
#undef V
#undef S
}

#endif /* AR_ISP_DE3D_H */
