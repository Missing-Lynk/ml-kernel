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
#define AR_ISP_RNR_BLOB_HEADER		0x79d8
#define AR_ISP_RNR_HDR_MODE		0x08
#define AR_ISP_RNR_HDR_COUNT		0x0c
#define AR_ISP_RNR_HDR_SELECT		0x10
#define AR_ISP_RNR_BLOB_BANDS		0x79ec
#define AR_ISP_RNR_BLOB_PAYLOAD		0x7a6c
#define AR_ISP_RNR_BLOB_STRIDE		0x160
#define AR_ISP_RNR_BANDS		12
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

#endif /* AR_ISP_RNR_H */
