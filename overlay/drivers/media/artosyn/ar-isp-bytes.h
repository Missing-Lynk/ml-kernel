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

static inline void ar_isp_put_le16(u8 *p, u16 v)
{
	p[0] = v;
	p[1] = v >> 8;
}

#endif /* AR_ISP_BYTES_H */
