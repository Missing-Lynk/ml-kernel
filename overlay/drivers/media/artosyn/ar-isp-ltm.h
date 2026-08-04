/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-ltm.h - the local tone mapping stage on bank 0x2800.
 *
 * Bank 0x2800 carries three things: a module control word, a coefficient page
 * the vendor's ltm and gtm2 modules publish at +0x08, and the ltm_stats output
 * the hardware writes at +0x0c. This header describes the coefficient page and
 * the buffer extents; there is no packing function because there is nothing to
 * pack from.
 *
 * The page has no stored source. It is recomputed every frame by a SIMD loop in
 * the vendor service, which emits one 128-sample curve per tile into a 0x100
 * slot, so reproducing it is 3A-class work rather than table work. What is
 * established here is the geometry a producer has to fill and the extents a
 * driver has to allocate, both measured against a captured page.
 *
 * The proof is kernel/scripts/isp/check-ltm-page.py.
 */

#ifndef AR_ISP_LTM_H
#define AR_ISP_LTM_H

#include "ar-isp-bytes.h"

#define AR_ISP_LTM_BANK			0x2800
#define AR_ISP_LTM_REG_CTRL		0x00
#define AR_ISP_LTM_REG_DIMS		0x04
#define AR_ISP_LTM_REG_PAGE		0x08
#define AR_ISP_LTM_REG_STATS		0x0c

/*
 * Module control word. The vendor's working configuration settles on
 * 0x00060f70, which has both of these set; the value it passes through on the
 * way, 0x00060770, has bit 4 but not bit 11.
 */
#define AR_ISP_LTM_CTRL_EN4		BIT(4)
#define AR_ISP_LTM_CTRL_EN11		BIT(11)
#define AR_ISP_LTM_CTRL_STREAMING	0x00060f70

/*
 * Coefficient page: 64 tiles, each a 128-sample transfer curve of u16 in a
 * 0x100 slot, so the page is exactly 0x4000. Every curve starts at 0 and rises
 * monotonically to just under 1024, which is a 10-bit output range.
 *
 * The captured page holds 64 well-formed curves and turns to unrelated data at
 * exactly 0x4000, and the producer loop writes its tile count times 0x100 into
 * the same buffer.
 *
 * All 64 curves are distinct. Their arrangement over the frame is not
 * established; 64 tiles is what the page holds, an 8x8 grid is only the obvious
 * reading of it.
 */
#define AR_ISP_LTM_TILES		64
#define AR_ISP_LTM_CURVE_SAMPLES	128
#define AR_ISP_LTM_CURVE_STRIDE		0x100
#define AR_ISP_LTM_PAGE_SIZE		(AR_ISP_LTM_TILES * AR_ISP_LTM_CURVE_STRIDE)
#define AR_ISP_LTM_SAMPLE_MAX		0x3ff

/*
 * Buffer extents, measured from the vendor's own allocation layout: each
 * descriptor alternates between two addresses and the gap between them is the
 * buffer. The coefficient page and the statistics buffer are both double
 * buffered and both strides are 0x80000, which is the allocation the vendor
 * makes rather than the bytes it fills.
 *
 * af_stats is included because a driver still has to publish an address for it
 * even though the module is disabled and nothing writes there.
 */
#define AR_ISP_LTM_PAGE_ALLOC		0x80000
#define AR_ISP_LTM_STATS_ALLOC		0x80000
#define AR_ISP_AF_STATS_ALLOC		0x1200

static inline u16 ar_isp_ltm_sample(const u8 *page, unsigned int tile,
				    unsigned int sample)
{
	return ar_isp_get_le16(page + tile * AR_ISP_LTM_CURVE_STRIDE + sample * 2);
}

static inline void ar_isp_ltm_put_sample(u8 *page, unsigned int tile,
					 unsigned int sample, u16 value)
{
	ar_isp_put_le16(page + tile * AR_ISP_LTM_CURVE_STRIDE + sample * 2, value);
}

/*
 * An identity page: every tile mapped to a straight ramp over the sample range.
 * Not a reconstruction of what the vendor computes; it leaves the stage
 * tone-neutral rather than dark.
 */
static inline void ar_isp_ltm_fill_identity(u8 *page)
{
	for (unsigned int tile = 0; tile < AR_ISP_LTM_TILES; tile++)
		for (unsigned int i = 0; i < AR_ISP_LTM_CURVE_SAMPLES; i++)
			ar_isp_ltm_put_sample(page, tile, i,
					      i * AR_ISP_LTM_SAMPLE_MAX /
					      (AR_ISP_LTM_CURVE_SAMPLES - 1));
}

#endif /* AR_ISP_LTM_H */
