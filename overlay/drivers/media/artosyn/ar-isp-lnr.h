/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-lnr.h - the lnr gain ladder.
 *
 * Recovered from the lnr driver in libmpp_service.so at 0x1bd8a8: 84 of 85
 * registers come out bit-exact at abscissas 5.59375 and 15.3828125 against the
 * vendor captures. The runnable proof is kernel/scripts/isp/check-lnr-ladder.py,
 * which reruns this arithmetic in Python and refuses to pass if it stops
 * reproducing the measured register states from the tuning file.
 *
 * Pure data transforms: no register access and no kernel API beyond the
 * integer types, so the same source can be compiled host-side against the
 * captures. See ar-isp-ladder.h for the ladder shape.
 */

#ifndef AR_ISP_LNR_H
#define AR_ISP_LNR_H

#include "ar-isp-ladder.h"

/*
 * Register bank. The ladder owns 85 of the 86 words in 0x3cc8..0x3e1c: 0x3d10
 * is never written by the vendor packer, and 0x3d14 stays on the replay path
 * because the vendor applies an unresolved fixed bias before pack.
 */
#define AR_ISP_LNR_BANK			0x3cc8
#define AR_ISP_LNR_REGS			86
#define AR_ISP_LNR_SKIP_NEVER_WRITTEN	((0x3d10 - AR_ISP_LNR_BANK) / 4)
#define AR_ISP_LNR_SKIP_BIASED		((0x3d14 - AR_ISP_LNR_BANK) / 4)

/*
 * Ladder in the tuning file. The header words are enable, interpolate, band
 * count, abscissa selector.
 */
#define AR_ISP_LNR_BLOB_HEADER		0x89e88
#define AR_ISP_LNR_HDR_COUNT		0x08
#define AR_ISP_LNR_HDR_SELECT		0x0c
#define AR_ISP_LNR_BLOB_BANDS		0x89e98
#define AR_ISP_LNR_BLOB_PAYLOAD		0x89f18
#define AR_ISP_LNR_BLOB_STRIDE		0x428
#define AR_ISP_LNR_BANDS		11

/*
 * The byte-packed curve: registers 38 to 85, four payload words each. The
 * records are contiguous in two runs, the first sixteen at 0x110 and the rest
 * at 0x228.
 */
#define AR_ISP_LNR_CURVE_REGS		48
#define AR_ISP_LNR_CURVE_FIRST_REG	38
#define AR_ISP_LNR_CURVE_OFF_LOW	0x110
#define AR_ISP_LNR_CURVE_OFF_HIGH	0x228
#define AR_ISP_LNR_CURVE_SPLIT		16
#define AR_ISP_LNR_CURVE_STRIDE		16

static const struct ar_isp_ladder ar_isp_lnr_ladder = {
	.hdr		= AR_ISP_LNR_BLOB_HEADER,
	.bands		= AR_ISP_LNR_BLOB_BANDS,
	.payload	= AR_ISP_LNR_BLOB_PAYLOAD,
	.stride		= AR_ISP_LNR_BLOB_STRIDE,
	.count_off	= AR_ISP_LNR_HDR_COUNT,
	.max_bands	= AR_ISP_LNR_BANDS,
};

/*
 * One scalar field: the payload word at @off drives bits @width wide at @shift
 * in bank register @reg. Rows appear in the order the vendor packer writes
 * them, and cover registers 0 to 17 and 20 to 37.
 */
struct ar_isp_lnr_field {
	u8 reg;
	u16 off;
	u8 shift;
	u8 width;
};

static const struct ar_isp_lnr_field ar_isp_lnr_fields[] = {
	/* Register 0: five enable bits and an 8-bit field. */
	{  0, 0x000,  2,  1 },
	{  0, 0x004,  6,  1 },
	{  0, 0x008,  8,  1 },
	{  0, 0x00c,  9,  1 },
	{  0, 0x010, 12,  1 },
	{  0, 0x014, 24,  8 },

	/* Registers 1 to 14: 13-bit pairs, low half then high. */
	{  1, 0x018,  0, 13 },
	{  1, 0x028, 16, 13 },
	{  2, 0x038,  0, 13 },
	{  2, 0x048, 16, 13 },
	{  3, 0x01c,  0, 13 },
	{  3, 0x02c, 16, 13 },
	{  4, 0x03c,  0, 13 },
	{  4, 0x04c, 16, 13 },
	{  5, 0x020,  0, 13 },
	{  5, 0x030, 16, 13 },
	{  6, 0x040,  0, 13 },
	{  6, 0x050, 16, 13 },
	{  7, 0x024,  0, 13 },
	{  7, 0x034, 16, 13 },
	{  8, 0x044,  0, 13 },
	{  8, 0x054, 16, 13 },
	{  9, 0x058,  0, 13 },
	{  9, 0x068, 16, 13 },
	{ 10, 0x078,  0, 13 },
	{ 10, 0x05c, 16, 13 },
	{ 11, 0x06c,  0, 13 },
	{ 11, 0x07c, 16, 13 },
	{ 12, 0x060,  0, 13 },
	{ 12, 0x070, 16, 13 },
	{ 13, 0x080,  0, 13 },
	{ 13, 0x064, 16, 13 },
	{ 14, 0x074,  0, 13 },
	{ 14, 0x084, 16, 13 },

	/* Registers 15 to 17: 8-bit pairs from the 0x210 run. */
	{ 15, 0x210,  0,  8 },
	{ 15, 0x214,  8,  8 },
	{ 16, 0x218,  0,  8 },
	{ 16, 0x21c,  8,  8 },
	{ 17, 0x220,  0,  8 },
	{ 17, 0x224,  8,  8 },

	/* Registers 20 to 37: mixed 10- and 16-bit fields. */
	{ 20, 0x090,  0, 10 },
	{ 20, 0x094, 16, 10 },
	{ 21, 0x098,  0, 10 },
	{ 21, 0x09c, 10, 10 },
	{ 22, 0x0a0,  0, 10 },
	{ 22, 0x0a4, 10, 10 },
	{ 23, 0x0a8,  0, 10 },
	{ 23, 0x0ac, 10, 10 },
	{ 24, 0x0b0,  0, 10 },
	{ 24, 0x0b4, 10, 10 },
	{ 25, 0x0b8,  0, 10 },
	{ 26, 0x0bc,  0, 16 },
	{ 26, 0x0c0, 16, 16 },
	{ 27, 0x0c4,  0, 16 },
	{ 27, 0x0c8, 16, 16 },
	{ 28, 0x0cc,  0, 16 },
	{ 29, 0x0d0,  0, 10 },
	{ 29, 0x0d4, 10, 10 },
	{ 30, 0x0d8,  0, 10 },
	{ 30, 0x0dc, 10, 10 },
	{ 31, 0x0e0,  0, 10 },
	{ 31, 0x0e4, 10, 10 },
	{ 32, 0x0e8,  0, 10 },
	{ 32, 0x0ec, 10, 10 },
	{ 33, 0x0f0,  0, 10 },
	{ 34, 0x0f4,  0, 16 },
	{ 34, 0x0f8, 16, 16 },
	{ 35, 0x0fc,  0, 16 },
	{ 35, 0x100, 16, 16 },
	{ 36, 0x104,  0, 16 },
	{ 37, 0x108,  0, 10 },
	{ 37, 0x10c, 16, 10 },
};

#define AR_ISP_LNR_FIELDS	(sizeof(ar_isp_lnr_fields) / \
				 sizeof(ar_isp_lnr_fields[0]))

/*
 * One payload word into one bit field. The width test keeps a 32-bit field
 * from shifting by its own width, which is undefined.
 */
static inline void ar_isp_lnr_pack_field(u32 *dst, const u8 *payload,
					 unsigned int band, u32 t_q24,
					 unsigned int reg, unsigned int off,
					 unsigned int shift, unsigned int width)
{
	u32 mask = width == 32 ? 0xffffffff : (1u << width) - 1;
	u32 field = ar_isp_ladder_read_word(&ar_isp_lnr_ladder, payload, band,
					    off, t_q24);

	dst[reg] &= ~(mask << shift);
	dst[reg] |= (field & mask) << shift;
}

/*
 * Build the lnr bank from the tuning file at the given abscissa. @dst carries
 * the current register contents in: every field is read-modify-written into it
 * because bits outside the table are owned elsewhere.
 */
static inline void ar_isp_lnr_from_blob(u32 *dst, const u8 *blob, u32 gain_q16)
{
	const u8 *payload = blob + AR_ISP_LNR_BLOB_PAYLOAD;
	unsigned int band, i;
	u32 t_q24;

	ar_isp_ladder_select(&ar_isp_lnr_ladder, blob, gain_q16, &band, &t_q24);

	for (i = 0; i < AR_ISP_LNR_FIELDS; i++) {
		const struct ar_isp_lnr_field *f = &ar_isp_lnr_fields[i];

		ar_isp_lnr_pack_field(dst, payload, band, t_q24, f->reg, f->off,
				      f->shift, f->width);
	}

	for (i = 0; i < AR_ISP_LNR_CURVE_REGS; i++) {
		unsigned int reg = AR_ISP_LNR_CURVE_FIRST_REG + i;
		unsigned int off = i < AR_ISP_LNR_CURVE_SPLIT ?
				   AR_ISP_LNR_CURVE_OFF_LOW +
				   i * AR_ISP_LNR_CURVE_STRIDE :
				   AR_ISP_LNR_CURVE_OFF_HIGH +
				   (i - AR_ISP_LNR_CURVE_SPLIT) *
				   AR_ISP_LNR_CURVE_STRIDE;

		ar_isp_lnr_pack_field(dst, payload, band, t_q24, reg, off, 0, 8);
		ar_isp_lnr_pack_field(dst, payload, band, t_q24, reg, off + 4, 8, 8);
		ar_isp_lnr_pack_field(dst, payload, band, t_q24, reg, off + 8, 16, 8);
		ar_isp_lnr_pack_field(dst, payload, band, t_q24, reg, off + 12, 24, 8);
	}
}

#endif /* AR_ISP_LNR_H */
