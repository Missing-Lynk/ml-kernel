/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-rnr.h - the rnr gain ladder.
 *
 * Recovered from the rnr driver in libmpp_service.so at 0x1993b8 and validated
 * against the cold bank at abscissa 1.0 and the vendor's live 0x002e002d at
 * 13.6 to 14.2. The runnable proof is kernel/scripts/isp/check-rnr-ladder.py,
 * which reruns this arithmetic in Python and refuses to pass if it stops
 * reproducing the measured register states from the tuning file.
 *
 * Pure data transforms: no register access and no kernel API beyond the
 * integer types, so the same source can be compiled host-side against the
 * captures. See ar-isp-ladder.h for the ladder shape.
 */

#ifndef AR_ISP_RNR_H
#define AR_ISP_RNR_H

#include "vendor-tables/ar-isp-blob.h"
#include "ar-isp-ladder.h"

/*
 * Register bank. The twelve ladder-fed registers sit at +0x08; each packs one
 * payload pair as (high << 16) | low. Bank +0x00 bit 1 carries the header mode
 * flag, which reads 0 in this blob and is already 0 in the replayed bank word,
 * so the apply path leaves +0x00 to the replay.
 */
#define AR_ISP_RNR_BANK			0x1800
#define AR_ISP_RNR_LADDER		0x08
#define AR_ISP_RNR_REGS			12

/*
 * Ladder in the tuning file. The header words are enable, interpolate, mode,
 * band count, abscissa selector. Twelve bands are allocated; the layout is the
 * fixed structure shared by all three sensors' files.
 *
 * Payload words 2..13 feed the low register halves and 14..25 the high halves,
 * one word per register.
 */
#define AR_ISP_RNR_LO_WORD		2
#define AR_ISP_RNR_HI_WORD		14

static const struct ar_isp_ladder ar_isp_rnr_ladder = {
	.hdr		= AR_ISP_RNR_BLOB_HEADER,
	.bands		= AR_ISP_RNR_BLOB_BANDS,
	.payload	= AR_ISP_RNR_BLOB_PAYLOAD,
	.stride		= AR_ISP_RNR_BLOB_STRIDE,
	.count_off	= AR_ISP_RNR_HDR_COUNT,
	.max_bands	= AR_ISP_RNR_BANDS,
};

/*
 * Build the twelve rnr ladder registers from the tuning file at the given
 * abscissa. Each register packs a payload pair, so this reads both words from
 * one record and blends them together.
 */
static inline void ar_isp_rnr_from_blob(u32 *dst, const u8 *blob, u32 gain_q16)
{
	const u8 *payload = blob + AR_ISP_RNR_BLOB_PAYLOAD;
	unsigned int band;
	u32 t_q24;

	ar_isp_ladder_select(&ar_isp_rnr_ladder, blob, gain_q16, &band, &t_q24);

	for (unsigned int reg = 0; reg < AR_ISP_RNR_REGS; reg++) {
		u32 lo = ar_isp_get_le32(payload + band * AR_ISP_RNR_BLOB_STRIDE +
					 (AR_ISP_RNR_LO_WORD + reg) * 4);
		u32 hi = ar_isp_get_le32(payload + band * AR_ISP_RNR_BLOB_STRIDE +
					 (AR_ISP_RNR_HI_WORD + reg) * 4);

		if (t_q24) {
			const u8 *prev = payload + (band - 1) * AR_ISP_RNR_BLOB_STRIDE;

			lo = ar_isp_ladder_blend(ar_isp_get_le32(prev +
					(AR_ISP_RNR_LO_WORD + reg) * 4), lo, t_q24);
			hi = ar_isp_ladder_blend(ar_isp_get_le32(prev +
					(AR_ISP_RNR_HI_WORD + reg) * 4), hi, t_q24);
		}

		dst[reg] = (hi << 16) | (lo & 0xffff);
	}
}

/*
 * The rest of the bank, 0x1838 to 0x188c, from the same payload record.
 *
 * The packer at 0x19b2c0 reads its inputs from the module's working block, and
 * from word 26 on that block is the payload record in order: the store to
 * bank +0x38 takes its two 12-bit fields from working offsets 3572 and 3576,
 * the store to +0x64 takes its four 5-bit fields from 3668 onward, and the
 * store to +0x68 takes its four 8-bit fields from 3684 onward. Those are
 * exactly words 26, 27, 50 and 54 at four bytes each from a common base, which
 * is what fixes the mapping. The ladder words 2 to 25 are the exception: the
 * blend leaves them in the block as (high, low) pairs, which is why
 * ar_isp_rnr_from_blob reads them by their own offsets.
 *
 * The layout is two runs of repeated groups:
 *
 *   four triples, seven words each, at words 26 to 53. Per triple: two 12-bit
 *   fields at bits 0 and 16 of the first register, one 12-bit field of the
 *   second, and four 5-bit fields at bits 0, 8, 16 and 24 of the third.
 *
 *   two blocks of five, seventeen words each, at words 54 to 87. Per block:
 *   four 8-bit fields for each of the first two registers, then three 5-bit
 *   fields for each of the remaining three.
 *
 * Every word of both runs is band-keyed except 54 to 61 and 71 to 78, so the
 * run moves with gain and a replay of it is pinned at one abscissa.
 */
#define AR_ISP_RNR_TAIL			0x38
#define AR_ISP_RNR_TAIL_REGS		22
#define AR_ISP_RNR_TAIL_WORD		26

#define AR_ISP_RNR_TRIPLES		4
#define AR_ISP_RNR_TRIPLE_WORDS		7
#define AR_ISP_RNR_BLOCKS		2
#define AR_ISP_RNR_BLOCK_WORDS		17

#define AR_ISP_RNR_F12			0xfff
#define AR_ISP_RNR_F8			0xff
#define AR_ISP_RNR_F5			0x1f

/*
 * One packed register: `n` fields of `width` bits taken from consecutive
 * payload words starting at `word`, placed at `width`-independent byte or
 * halfword positions the packer's masks give.
 */
static inline u32 ar_isp_rnr_pack(const u8 *payload, unsigned int band,
				  u32 t_q24, unsigned int word,
				  unsigned int n, u32 mask, unsigned int step)
{
	u32 out = 0;

	for (unsigned int i = 0; i < n; i++) {
		const u8 *rec = payload + band * AR_ISP_RNR_BLOB_STRIDE;
		u32 v = ar_isp_get_le32(rec + (word + i) * 4);

		if (t_q24) {
			const u8 *prev = payload +
					 (band - 1) * AR_ISP_RNR_BLOB_STRIDE;

			v = ar_isp_ladder_blend(ar_isp_get_le32(prev +
					(word + i) * 4), v, t_q24);
		}

		out |= (v & mask) << (i * step);
	}

	return out;
}

/*
 * Build the twenty-two tail registers. Returns them in bank order starting at
 * AR_ISP_RNR_BANK + AR_ISP_RNR_TAIL.
 */
static inline void ar_isp_rnr_tail_from_blob(u32 *dst, const u8 *blob,
					     u32 gain_q16)
{
	const u8 *payload = blob + AR_ISP_RNR_BLOB_PAYLOAD;
	unsigned int band, word = AR_ISP_RNR_TAIL_WORD, reg = 0, i;
	u32 t_q24;

	ar_isp_ladder_select(&ar_isp_rnr_ladder, blob, gain_q16, &band, &t_q24);

	for (i = 0; i < AR_ISP_RNR_TRIPLES; i++) {
		dst[reg++] = ar_isp_rnr_pack(payload, band, t_q24, word,
					     2, AR_ISP_RNR_F12, 16);
		dst[reg++] = ar_isp_rnr_pack(payload, band, t_q24, word + 2,
					     1, AR_ISP_RNR_F12, 0);
		dst[reg++] = ar_isp_rnr_pack(payload, band, t_q24, word + 3,
					     4, AR_ISP_RNR_F5, 8);
		word += AR_ISP_RNR_TRIPLE_WORDS;
	}

	for (i = 0; i < AR_ISP_RNR_BLOCKS; i++) {
		dst[reg++] = ar_isp_rnr_pack(payload, band, t_q24, word,
					     4, AR_ISP_RNR_F8, 8);
		dst[reg++] = ar_isp_rnr_pack(payload, band, t_q24, word + 4,
					     4, AR_ISP_RNR_F8, 8);
		dst[reg++] = ar_isp_rnr_pack(payload, band, t_q24, word + 8,
					     3, AR_ISP_RNR_F5, 8);
		dst[reg++] = ar_isp_rnr_pack(payload, band, t_q24, word + 11,
					     3, AR_ISP_RNR_F5, 8);
		dst[reg++] = ar_isp_rnr_pack(payload, band, t_q24, word + 14,
					     3, AR_ISP_RNR_F5, 8);
		word += AR_ISP_RNR_BLOCK_WORDS;
	}
}

#endif /* AR_ISP_RNR_H */
