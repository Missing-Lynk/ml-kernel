/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-cm2.h - the cm2 AE-indexed colour row.
 *
 * Recovered from the cm2 driver in libmpp_service.so at 0x1a14ec and its
 * packer at 0x1a0578. The runnable proof is
 * kernel/scripts/isp/check-cm2-ladder.py, with host-side C parity through
 * kernel/scripts/isp/check-ladder-c.py.
 *
 * The abscissa is the AEC trigger scalar on the 0 to 550 axis the gamma and DRC
 * band tables share, NOT the linear gain the rnr, lnr, de3d, cfa and cnf
 * ladders key on. The vendor's trigger payload carries both and each stage
 * picks one from a flag in its own tuning header. Feeding this a gain selects
 * the wrong row: at the captured operating points it lands on row 0 where the
 * vendor sat on row 1.
 *
 * Pure data transforms: no register access and no kernel API beyond the
 * integer types, so the same source can be compiled host-side against the
 * captures.
 */

#ifndef AR_ISP_CM2_H
#define AR_ISP_CM2_H

#include "vendor-tables/ar-isp-blob.h"
#include "ar-isp-cm.h"

#define AR_ISP_CM2_BANK			0x4800
#define AR_ISP_CM2_GAIN_REG		0x4804
#define AR_ISP_CM2_GAIN_MASK		0x0000007f
#define AR_ISP_CM2_LO1_REG		0x481c
#define AR_ISP_CM2_HI1_REG		0x4820
#define AR_ISP_CM2_LO2_REG		0x4824
#define AR_ISP_CM2_HI2_REG		0x4828
#define AR_ISP_CM2_RECIP1_REG		0x482c
#define AR_ISP_CM2_RECIP2_REG		0x4830

#define AR_ISP_CM2_CT_COLUMNS		1
#define AR_ISP_CM2_RECORD_STRIDE	24
#define AR_ISP_CM2_ROW_STRIDE		168
#define AR_ISP_CM2_BOUNDS_OFF		8
#define AR_ISP_CM2_RECIP_NUMERATOR	1024

struct ar_isp_cm2_row {
	u32 gain_field;
	u32 lo1;
	u32 hi1;
	u32 lo2;
	u32 hi2;
	u32 recip1;
	u32 recip2;
};

static inline u32 ar_isp_cm2_gain_q16(const u8 *blob, unsigned int row,
				      unsigned int ct)
{
	u32 off = AR_ISP_CM2_TABLE + row * AR_ISP_CM2_ROW_STRIDE +
		  ct * AR_ISP_CM2_RECORD_STRIDE;

	return ar_isp_f32_q16(ar_isp_get_le32(blob + off));
}

static inline s32 ar_isp_cm2_bound(const u8 *blob, unsigned int row,
				   unsigned int ct, unsigned int slot)
{
	u32 off = AR_ISP_CM2_TABLE + row * AR_ISP_CM2_ROW_STRIDE +
		  ct * AR_ISP_CM2_RECORD_STRIDE +
		  AR_ISP_CM2_BOUNDS_OFF + slot * 4;

	return (s32)ar_isp_get_le32(blob + off);
}

static inline void ar_isp_cm2_select(const u8 *blob, u32 trigger_q8,
				     unsigned int *row_out, u32 *t_q24_out)
{
	const u8 *hdr = blob + AR_ISP_CM2_HEADER;
	u32 count = ar_isp_get_le32(hdr + AR_ISP_CM2_HDR_AEC_COUNT);
	u32 interp = ar_isp_get_le32(hdr + AR_ISP_CM2_HDR_INTERP);
	unsigned int row;
	u32 t_q24 = 0;

	if (count < 1 || count > AR_ISP_CM2_AEC_ROWS)
		count = AR_ISP_CM2_AEC_ROWS;

	for (row = 0; row + 1 < count; row++) {
		u32 hi = ar_isp_cm_f32_q8(ar_isp_get_le32(blob +
				      AR_ISP_CM2_AEC_AXIS + row * 8 + 4));

		if (trigger_q8 <= hi)
			break;
	}

	if (interp && row > 0) {
		u32 lo = ar_isp_cm_f32_q8(ar_isp_get_le32(blob +
				      AR_ISP_CM2_AEC_AXIS + row * 8));
		u32 prev_hi = ar_isp_cm_f32_q8(ar_isp_get_le32(blob +
					   AR_ISP_CM2_AEC_AXIS + row * 8 - 4));

		if (trigger_q8 < lo && lo > prev_hi)
			t_q24 = (u32)(((u64)(trigger_q8 - prev_hi) <<
				       AR_ISP_LADDER_T_SHIFT) /
				      (lo - prev_hi));
	}

	*row_out = row;
	*t_q24_out = t_q24;
}

static inline s32 ar_isp_cm2_blend_bound(const u8 *blob, unsigned int row,
					 unsigned int ct, unsigned int slot,
					 u32 t_q24)
{
	s32 value = ar_isp_cm2_bound(blob, row, ct, slot);

	if (t_q24)
		value = ar_isp_ladder_blend_s32(ar_isp_cm2_bound(blob, row - 1,
								 ct, slot),
						value, t_q24);

	return value;
}

static inline void ar_isp_cm2_from_blob(struct ar_isp_cm2_row *dst,
					const u8 *blob, u32 trigger_q8,
					unsigned int ct)
{
	u32 ct_count = ar_isp_get_le32(blob + AR_ISP_CM2_HEADER +
				       AR_ISP_CM2_HDR_CT_COUNT);
	unsigned int row;
	u32 width1, width2;
	u32 gain_q16;
	u32 t_q24;

	if (ct_count < 1 || ct_count > AR_ISP_CM2_CT_COLUMNS)
		ct_count = AR_ISP_CM2_CT_COLUMNS;

	if (ct >= ct_count)
		ct = 0;

	ar_isp_cm2_select(blob, trigger_q8, &row, &t_q24);
	gain_q16 = ar_isp_cm2_gain_q16(blob, row, ct);

	if (t_q24)
		gain_q16 = ar_isp_ladder_blend(ar_isp_cm2_gain_q16(blob, row - 1, ct),
					       gain_q16, t_q24);

	dst->gain_field = (gain_q16 >> 11) & AR_ISP_CM2_GAIN_MASK;
	dst->lo1 = (u32)ar_isp_cm2_blend_bound(blob, row, ct, 0, t_q24);
	dst->hi1 = (u32)ar_isp_cm2_blend_bound(blob, row, ct, 1, t_q24);
	dst->lo2 = (u32)ar_isp_cm2_blend_bound(blob, row, ct, 2, t_q24);
	dst->hi2 = (u32)ar_isp_cm2_blend_bound(blob, row, ct, 3, t_q24);

	width1 = dst->hi1 - dst->lo1;
	width2 = dst->hi2 - dst->lo2;

	dst->recip1 = width1 ? AR_ISP_CM2_RECIP_NUMERATOR / width1 : 0;
	dst->recip2 = width2 ? AR_ISP_CM2_RECIP_NUMERATOR / width2 : 0;
}

#endif /* AR_ISP_CM2_H */
