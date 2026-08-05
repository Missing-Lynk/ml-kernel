/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-bytes.h - byte-wise little-endian accessors for the ISP table codecs.
 *
 * Byte-wise so the codecs are endian-independent and compile anywhere: no
 * kernel API beyond the integer types, so the same source can be built
 * host-side against captured buffers.
 */

#ifndef AR_ISP_BYTES_H
#define AR_ISP_BYTES_H

/*
 * IEEE-754 single fields. A stage converting to QN shifts the mantissa by
 * AR_ISP_F32_MANT_BITS - N - exponent, so the mantissa width is exported
 * alongside the accessors.
 */
#define AR_ISP_F32_MANT_BITS	23
#define AR_ISP_F32_SIGN		0x80000000u

static inline u32 ar_isp_get_le32(const u8 *p)
{
	return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) |
	       ((u32)p[3] << 24);
}

static inline void ar_isp_put_le32(u8 *p, u32 v)
{
	p[0] = v;
	p[1] = v >> 8;
	p[2] = v >> 16;
	p[3] = v >> 24;
}

static inline u16 ar_isp_get_le16(const u8 *p)
{
	return (u16)p[0] | ((u16)p[1] << 8);
}

static inline void ar_isp_put_le16(u8 *p, u16 v)
{
	p[0] = v;
	p[1] = v >> 8;
}

/*
 * The tuning file stores gains, matrix coefficients and ladder bounds as
 * float32 and the kernel has no FPU, so every stage decodes them from the bit
 * pattern. Each stage's own quantisation differs; only the field extraction is
 * shared.
 */

/* The mantissa with its implicit leading one restored. */
static inline u32 ar_isp_f32_mant(u32 bits)
{
	return (bits & 0x7fffff) | 0x800000;
}

/* The unbiased exponent. */
static inline int ar_isp_f32_exp(u32 bits)
{
	return (int)((bits >> AR_ISP_F32_MANT_BITS) & 0xff) - 127;
}

#endif /* AR_ISP_BYTES_H */
