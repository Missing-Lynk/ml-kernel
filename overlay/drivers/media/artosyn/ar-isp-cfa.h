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

#include "ar-isp-ladder.h"

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
 * The payload record is 41 words, all of which feed registers. Between bands
 * every word blends linearly and truncates toward zero. The packer's own
 * schedule takes working offsets 0x78, 0x7c, 0x88 and 0x94 verbatim from the
 * upper record instead; across this file the two agree at every fraction in
 * every band gap, because those four words only ever step from 1 to 0 and the
 * blend of 1 into 0 truncates to 0 for any nonzero fraction. The blend is used
 * for all 41, and check-cfa-ladder.py asserts that equivalence so a future
 * tuning file that breaks it fails rather than diverging silently.
 */
#define AR_ISP_CFA_BLOB_HEADER		0x24548
#define AR_ISP_CFA_HDR_COUNT		0x08
#define AR_ISP_CFA_HDR_SELECT		0x0c
#define AR_ISP_CFA_BLOB_BANDS		0x24558
#define AR_ISP_CFA_BLOB_PAYLOAD		0x245d8
#define AR_ISP_CFA_BLOB_STRIDE		0xa4
#define AR_ISP_CFA_BANDS		5

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

/*
 * Build the 41 cfa ladder registers from the tuning file at the abscissa, in
 * run order, which is the order ar_isp_cfa_runs walks the bank.
 */
static inline void ar_isp_cfa_from_blob(u32 *dst, const u8 *blob, u32 gain_q16)
{
	const u8 *payload = blob + AR_ISP_CFA_BLOB_PAYLOAD;
	unsigned int band, i = 0;
	u32 t_q24;

	ar_isp_ladder_select(&ar_isp_cfa_ladder, blob, gain_q16, &band, &t_q24);

	for (unsigned int run = 0; run < AR_ISP_CFA_RUNS; run++)
		for (unsigned int k = 0; k < ar_isp_cfa_runs[run].count; k++)
			dst[i++] = ar_isp_ladder_read_word(&ar_isp_cfa_ladder,
							   payload, band,
							   ar_isp_cfa_runs[run].word + k * 4,
							   t_q24);
}

#endif /* AR_ISP_CFA_H */
