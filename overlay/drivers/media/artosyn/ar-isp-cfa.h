/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-cfa.h - the cfa gain ladder.
 *
 * Recovered from the cfa driver in libmpp_service.so at 0x1b1f28 and its packer
 * at 0x1b1d90. The runnable proof is kernel/scripts/isp/check-cfa-ladder.py,
 * which reruns this arithmetic in Python and refuses to pass if it stops
 * reproducing the measured register state from the tuning file.
 *
 * Pure data transforms: no register access and no kernel API beyond the
 * integer types, so the same source can be compiled host-side against the
 * captures. See ar-isp-ladder.h for the ladder shape.
 *
 * Where every value in bank 0x0800 comes from, all 54 the packer owns:
 *
 *   41  computed here from the tuning file
 *    9  the module's static register image, which the vendor service carries
 *       in its own data and installs with one memcpy; the replay tables hold
 *       the same nine words
 *    2  frame geometry and a mode selector, 0x082c and 0x0858: the vendor
 *       builds 0x082c as (width << 15) | (height << 2) | mode, which
 *       reassembles the measured 0x03c010e0 exactly at 1920 x 1080 mode 0,
 *       and writes the mode into 0x0858 bits 1:0 from the same subcommand
 *    2  hardware-written, 0x0834 and 0x08a8: a known value written to either
 *       reads back as something else on two independent boots, and the packer
 *       stores to neither offset
 */

#ifndef AR_ISP_CFA_H
#define AR_ISP_CFA_H

#include "vendor-tables/ar-isp-blob.h"
#include "ar-isp-ladder.h"
#include "ar-isp-softfloat.h"

/*
 * Register bank. The packer owns 54 registers, 0x0800 to 0x08d4, and the whole
 * 0xa4-byte payload record lands in 41 of them as four ascending runs, each
 * word used once. The run boundaries and the two registers the runs step over
 * inside their span, 0x0858 and 0x08a8, are the packer's own store schedule.
 *
 * The 13 registers outside the runs take their values from the module's static
 * register image, which the packer memcpy's whole when no recomputation is due.
 * Eleven of them read back that image on the streaming vendor. The other two,
 * 0x0834 and 0x08a8, are hardware-written: a known value written to them reads
 * back as something else on two independent boots, and the packer never stores
 * to either offset.
 *
 * Register 0x0800 carries the record's first word in its low half. The vendor
 * fills the high half from the static image's own first word; the apply path
 * preserves what the replay installed there, which is that same image.
 */
#define AR_ISP_CFA_BANK			0x0800
#define AR_ISP_CFA_REGS			41
#define AR_ISP_CFA_RUNS			4
#define AR_ISP_CFA_HEAD_MASK		0x0000ffff

/*
 * Ladder in the tuning file. The header words are enable, interpolate, band
 * count, abscissa selector. Five bands are active; the sixth payload slot at
 * 0x2490c is zero padding and the header's count of 5 excludes it.
 *
 * The payload record is 41 words, all of which feed registers, and the driver
 * has two paths:
 *
 *   in a band  the whole 0xa4-byte record is memcpy'd into the module's config
 *              block and the packer copies it out, so the record reaches the
 *              bank verbatim
 *
 *   in a gap   each word is blended in float and truncated
 *
 * The gap path is binary32, not an integer blend: scvtf on each word, fmul for
 * the upper term, a fused fmadd, then fcvtzs. See ar-isp-softfloat.h.
 *
 * Two words take their operands from a different record index than the one they
 * feed:
 *
 *   word 6   (register 0x0844) takes its low operand from word 4
 *   word 28  (register 0x08a0) takes both operands from word 26
 *
 * So word 28's own record value never reaches the bank.
 */

_Static_assert(AR_ISP_CFA_REGS * 4 == AR_ISP_CFA_BLOB_STRIDE,
	       "cfa runs must consume the whole payload record");

static const struct ar_isp_ladder ar_isp_cfa_ladder = {
	.hdr		= AR_ISP_CFA_BLOB_HEADER,
	.bands		= AR_ISP_CFA_BLOB_BANDS,
	.payload	= AR_ISP_CFA_BLOB_PAYLOAD,
	.stride		= AR_ISP_CFA_BLOB_STRIDE,
	.count_off	= AR_ISP_CFA_HDR_COUNT,
	.max_bands	= AR_ISP_CFA_BANDS,
};

/*
 * One run of consecutive registers fed by consecutive payload words. Both
 * offsets are byte offsets, the first within the bank and the second within the
 * record.
 */
struct ar_isp_cfa_run {
	u16 reg;
	u16 word;
	u16 count;
};

static const struct ar_isp_cfa_run ar_isp_cfa_runs[AR_ISP_CFA_RUNS] = {
	{ 0x00, 0x00, 4 },
	{ 0x3c, 0x10, 7 },
	{ 0x5c, 0x2c, 19 },
	{ 0xac, 0x78, 11 },
};

/* The two source-index substitutions, on the blend path only. */
#define AR_ISP_CFA_QUIRK_LOW_WORD	6
#define AR_ISP_CFA_QUIRK_LOW_SOURCE	4
#define AR_ISP_CFA_QUIRK_BOTH_WORD	28
#define AR_ISP_CFA_QUIRK_BOTH_SOURCE	26

static inline unsigned int ar_isp_cfa_low_source(unsigned int word)
{
	if (word == AR_ISP_CFA_QUIRK_LOW_WORD)
		return AR_ISP_CFA_QUIRK_LOW_SOURCE;

	if (word == AR_ISP_CFA_QUIRK_BOTH_WORD)
		return AR_ISP_CFA_QUIRK_BOTH_SOURCE;

	return word;
}

static inline unsigned int ar_isp_cfa_high_source(unsigned int word)
{
	if (word == AR_ISP_CFA_QUIRK_BOTH_WORD)
		return AR_ISP_CFA_QUIRK_BOTH_SOURCE;

	return word;
}

/*
 * One payload word. Verbatim inside a band; on the gap path the vendor's fused
 * float blend, with the two source quirks applied.
 */
static inline u32 ar_isp_cfa_word(const u8 *payload, unsigned int band,
				  unsigned int word,
				  struct ar_isp_ladder_frac frac)
{
	u32 stride = AR_ISP_CFA_BLOB_STRIDE;
	ar_f32 lo, hi;

	if (!frac.t)
		return ar_isp_get_le32(payload + band * stride + word * 4);

	lo = ar_f32_from_s32((s32)ar_isp_get_le32(payload + (band - 1) * stride +
					     ar_isp_cfa_low_source(word) * 4));
	hi = ar_f32_from_s32((s32)ar_isp_get_le32(payload + band * stride +
					     ar_isp_cfa_high_source(word) * 4));

	return (u32)ar_f32_to_s32_trunc(ar_f32_fma(frac.omt, lo,
						   ar_f32_mul(frac.t, hi)));
}

/*
 * Build the 41 cfa ladder registers from the tuning file at the abscissa, in
 * run order, which is the order ar_isp_cfa_runs walks the bank.
 */
static inline void ar_isp_cfa_from_blob(u32 *dst, const u8 *blob, u32 gain_q16)
{
	const u8 *payload = blob + AR_ISP_CFA_BLOB_PAYLOAD;
	struct ar_isp_ladder_frac frac;
	unsigned int band, i = 0;

	ar_isp_ladder_select_f32(&ar_isp_cfa_ladder, blob, gain_q16, &band,
				 &frac);

	for (unsigned int run = 0; run < AR_ISP_CFA_RUNS; run++)
		for (unsigned int k = 0; k < ar_isp_cfa_runs[run].count; k++)
			dst[i++] = ar_isp_cfa_word(payload, band,
						   ar_isp_cfa_runs[run].word / 4 + k,
						   frac);
}

#endif /* AR_ISP_CFA_H */
