/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-blc.h - black level correction on CVISP bank 0x4200.
 *
 * BLC is a register file, not a DMA table. Sixteen registers are filled by a
 * verbatim 64-byte copy of a structure the vendor builds in DRAM, so the whole
 * stage is one memcpy once its 64 bytes are computed.
 *
 * The payload comes from the tuning file, not from a library template. Five
 * calibration entries hold per-Bayer-channel values, and a ladder of float
 * pairs blends between two adjacent entries by the current sensor gain. The
 * first group of four is shifted left by 6 on its way to the registers and the
 * second group is not.
 *
 * The proof is kernel/scripts/check-blc.py.
 */

#ifndef AR_ISP_BLC_H
#define AR_ISP_BLC_H

#include "ar-isp-bytes.h"

/*
 * The bank lives on the CVISP block, which the vendor reaches by adding
 * 0x200000 to the ISP physical base before translating it, then 0x4200 after.
 */
#define AR_ISP_CVISP_OFFSET		0x200000
#define AR_ISP_BLC_BANK			0x4200
#define AR_ISP_BLC_BLOCK		0x40

/*
 * Register block, four lanes of four u32. Lane 0 takes the first group of the
 * blended entry shifted left by 6, lane 2 takes the second group unshifted.
 * Lanes 1 and 3 mirror them on the vendor's single-exposure capture and are
 * filled by a path outside this stage.
 */
#define AR_ISP_BLC_REG_SCALE		0x00
#define AR_ISP_BLC_REG_SCALE_ALT	0x10
#define AR_ISP_BLC_REG_LEVEL		0x20
#define AR_ISP_BLC_REG_LEVEL_ALT	0x30
#define AR_ISP_BLC_LANE			4

/* The first group is stored at this shift; the second is stored as it is. */
#define AR_ISP_BLC_SCALE_SHIFT		6

/*
 * Tuning file layout. The entry table is five 32-byte entries, each two groups
 * of four u32 in Bayer channel order. The ladder is five pairs of float32 at an
 * 8-byte stride, one pair per entry, and selects and blends the entries.
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
	const u8 *e = blob + AR_ISP_BLC_BLOB_TABLE +
		      index * AR_ISP_BLC_ENTRY_SIZE;
	unsigned int i;

	for (i = 0; i < AR_ISP_BLC_LANE; i++) {
		out->scale[i] = ar_isp_get_le32(e + 4 * i);
		out->level[i] = ar_isp_get_le32(e + AR_ISP_BLC_GROUP_OFF +
						4 * i);
	}
}

/*
 * The vendor blends in float32 and converts with fcvtzu, which truncates
 * toward zero, so the fixed-point form has to truncate too. blend is the
 * weight of the low entry in Q16.
 */
#define AR_ISP_BLC_BLEND_ONE		65536u

static inline u32 ar_isp_blc_mix(u32 lo, u32 hi, u32 blend)
{
	u64 v = (u64)lo * blend + (u64)hi * (AR_ISP_BLC_BLEND_ONE - blend);

	return (u32)(v / AR_ISP_BLC_BLEND_ONE);
}

/*
 * Fill the 64-byte block the bank is loaded from. Lanes 1 and 3 are written to
 * match lanes 0 and 2, which is what the vendor's single-exposure capture
 * holds.
 */
static inline void ar_isp_blc_fill(u8 *block,
				   const struct ar_isp_blc_entry *lo,
				   const struct ar_isp_blc_entry *hi,
				   u32 blend)
{
	unsigned int i;

	for (i = 0; i < AR_ISP_BLC_LANE; i++) {
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
