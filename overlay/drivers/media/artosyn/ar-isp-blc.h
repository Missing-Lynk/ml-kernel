/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-blc.h - black level correction on CVISP bank 0x4200.
 *
 * Sixteen registers loaded by a verbatim 64-byte copy: one memcpy once the 64
 * bytes are computed.
 *
 * The payload comes from the tuning file. Five calibration entries hold
 * per-Bayer-channel values; a ladder of float pairs blends two adjacent
 * entries by the current sensor gain. The first group of four is shifted left
 * by 6 into the registers, the second is not.
 *
 * Proof: kernel/scripts/isp/check-blc.py.
 */

#ifndef AR_ISP_BLC_H
#define AR_ISP_BLC_H

#include "ar-isp-bytes.h"

/* The bank is on CVISP: ISP physical base + 0x200000, then + 0x4200. */
#define AR_ISP_CVISP_OFFSET		0x200000
#define AR_ISP_BLC_BANK			0x4200
#define AR_ISP_BLC_BLOCK		0x40

/*
 * Four lanes of four u32. Lane 0 holds the blended entry's first group shifted
 * left by 6, lane 2 its second group unshifted. Lanes 1 and 3 hold the same
 * values on the vendor's single-exposure capture, written there by another
 * stage.
 */
#define AR_ISP_BLC_REG_SCALE		0x00
#define AR_ISP_BLC_REG_SCALE_ALT	0x10
#define AR_ISP_BLC_REG_LEVEL		0x20
#define AR_ISP_BLC_REG_LEVEL_ALT	0x30
#define AR_ISP_BLC_LANE			4

/* Applied to the first group only. */
#define AR_ISP_BLC_SCALE_SHIFT		6

/*
 * Tuning file layout. Five 32-byte entries, each two groups of four u32 in
 * Bayer channel order. The ladder is five float32 pairs at an 8-byte stride,
 * one per entry, and selects and blends them.
 */
#define AR_ISP_BLC_BLOB_LADDER		0x34
#define AR_ISP_BLC_BLOB_TABLE		0xb4
#define AR_ISP_BLC_ENTRIES		5
#define AR_ISP_BLC_ENTRY_SIZE		0x20
#define AR_ISP_BLC_GROUP_OFF		0x10

struct ar_isp_blc_entry {
	u32 scale[AR_ISP_BLC_LANE];
	u32 level[AR_ISP_BLC_LANE];
};

static inline void ar_isp_blc_entry(const u8 *blob, unsigned int index,
				    struct ar_isp_blc_entry *out)
{
	const u8 *entry = blob + AR_ISP_BLC_BLOB_TABLE +
			  index * AR_ISP_BLC_ENTRY_SIZE;

	for (unsigned int i = 0; i < AR_ISP_BLC_LANE; i++) {
		out->scale[i] = ar_isp_get_le32(entry + 4 * i);
		out->level[i] = ar_isp_get_le32(entry + AR_ISP_BLC_GROUP_OFF +
						4 * i);
	}
}

/*
 * blend is the low entry's weight in Q16. The vendor blends in float32 and
 * converts with fcvtzu, which truncates toward zero, so this truncates too.
 */
#define AR_ISP_BLC_BLEND_ONE		65536u

static inline u32 ar_isp_blc_mix(u32 lo, u32 hi, u32 blend)
{
	u64 sum = (u64)lo * blend + (u64)hi * (AR_ISP_BLC_BLEND_ONE - blend);

	return (u32)(sum / AR_ISP_BLC_BLEND_ONE);
}

/* Fill the 64-byte block the bank is loaded from. Lanes 1 and 3 take the same
 * values as lanes 0 and 2.
 */
static inline void ar_isp_blc_fill(u8 *block,
				   const struct ar_isp_blc_entry *lo,
				   const struct ar_isp_blc_entry *hi,
				   u32 blend)
{
	for (unsigned int i = 0; i < AR_ISP_BLC_LANE; i++) {
		u32 scale = ar_isp_blc_mix(lo->scale[i], hi->scale[i], blend);
		u32 level = ar_isp_blc_mix(lo->level[i], hi->level[i], blend);

		scale <<= AR_ISP_BLC_SCALE_SHIFT;

		ar_isp_put_le32(block + AR_ISP_BLC_REG_SCALE + 4 * i, scale);
		ar_isp_put_le32(block + AR_ISP_BLC_REG_SCALE_ALT + 4 * i, scale);
		ar_isp_put_le32(block + AR_ISP_BLC_REG_LEVEL + 4 * i, level);
		ar_isp_put_le32(block + AR_ISP_BLC_REG_LEVEL_ALT + 4 * i, level);
	}
}

#endif /* AR_ISP_BLC_H */
