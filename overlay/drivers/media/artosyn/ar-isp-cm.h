/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-cm.h - the cm AE-indexed colour row.
 *
 * Recovered from the cm driver in libmpp_service.so at 0x19fe28 and its gain
 * packer at 0x19f23c. The runnable proof is
 * kernel/scripts/isp/check-cm-ladder.py, with host-side C parity through
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

#ifndef AR_ISP_CM_H
#define AR_ISP_CM_H

#include "vendor-tables/ar-isp-blob.h"
#include "ar-isp-ladder.h"

#define AR_ISP_CM_BANK			0x4834
#define AR_ISP_CM_GAIN_REG		0x483c
#define AR_ISP_CM_GAIN_MASK		0x0000007f

#define AR_ISP_CM_CT_COLUMNS		7
#define AR_ISP_CM_RECORD_STRIDE	8
#define AR_ISP_CM_ROW_STRIDE \
	(AR_ISP_CM_CT_COLUMNS * AR_ISP_CM_RECORD_STRIDE)

static inline u32 ar_isp_cm_f32_q8(u32 bits)
{
	u32 mant = ar_isp_f32_mant(bits);
	int shift = (AR_ISP_F32_MANT_BITS - 8) - ar_isp_f32_exp(bits);

	if (bits & AR_ISP_F32_SIGN)
		return 0;

	if (shift >= 32)
		return 0;

	if (shift < -16)
		return 0xffffffff;

	if (shift <= 0)
		return mant << -shift;

	return mant >> shift;
}

static inline u32 ar_isp_cm_gain_q16(const u8 *blob, unsigned int row,
				     unsigned int ct)
{
	u32 off = AR_ISP_CM_TABLE + row * AR_ISP_CM_ROW_STRIDE +
		  ct * AR_ISP_CM_RECORD_STRIDE;

	return ar_isp_f32_q16(ar_isp_get_le32(blob + off));
}

static inline void ar_isp_cm_select(const u8 *blob, u32 trigger_q8,
				    unsigned int *row_out, u32 *t_q24_out)
{
	const u8 *hdr = blob + AR_ISP_CM_HEADER;
	u32 count = ar_isp_get_le32(hdr + AR_ISP_CM_HDR_AEC_COUNT);
	u32 interp = ar_isp_get_le32(hdr + AR_ISP_CM_HDR_INTERP);
	unsigned int row;
	u32 t_q24 = 0;

	if (count < 1 || count > AR_ISP_CM_AEC_ROWS)
		count = AR_ISP_CM_AEC_ROWS;

	for (row = 0; row + 1 < count; row++) {
		u32 hi = ar_isp_cm_f32_q8(ar_isp_get_le32(blob +
				      AR_ISP_CM_AEC_AXIS + row * 8 + 4));

		if (trigger_q8 <= hi)
			break;
	}

	if (interp && row > 0) {
		u32 lo = ar_isp_cm_f32_q8(ar_isp_get_le32(blob +
				      AR_ISP_CM_AEC_AXIS + row * 8));
		u32 prev_hi = ar_isp_cm_f32_q8(ar_isp_get_le32(blob +
					   AR_ISP_CM_AEC_AXIS + row * 8 - 4));

		if (trigger_q8 < lo && lo > prev_hi)
			t_q24 = (u32)(((u64)(trigger_q8 - prev_hi) <<
				       AR_ISP_LADDER_T_SHIFT) /
				      (lo - prev_hi));
	}

	*row_out = row;
	*t_q24_out = t_q24;
}

static inline u32 ar_isp_cm_gain_field_from_blob(const u8 *blob, u32 trigger_q8,
						 unsigned int ct)
{
	u32 count = ar_isp_get_le32(blob + AR_ISP_CM_HEADER +
				    AR_ISP_CM_HDR_CT_COUNT);
	unsigned int row;
	u32 gain_q16;
	u32 t_q24;

	if (count < 1 || count > AR_ISP_CM_CT_COLUMNS)
		count = AR_ISP_CM_CT_COLUMNS;

	if (ct >= count)
		ct = 0;

	ar_isp_cm_select(blob, trigger_q8, &row, &t_q24);
	gain_q16 = ar_isp_cm_gain_q16(blob, row, ct);

	if (t_q24)
		gain_q16 = ar_isp_ladder_blend(ar_isp_cm_gain_q16(blob, row - 1, ct),
					       gain_q16, t_q24);

	return (gain_q16 >> 11) & AR_ISP_CM_GAIN_MASK;
}

#endif /* AR_ISP_CM_H */
