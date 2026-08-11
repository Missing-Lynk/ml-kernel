/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-de3d-geom.h - de3d's geometry-derived bank registers.
 *
 * The de3d ladder in ar-isp-de3d.h recomputes the tuning-driven half of bank
 * 0x2e00 when the gain moves. This is the other half: thirteen registers the
 * vendor computes once from the frame geometry and the sensor line length, in
 * the configuration function at 0x1c3e00 in libmpp_service.so, plus the pair
 * at 0x1c4e5c. None of them depends on gain or on the tuning file.
 *
 * All thirteen reproduce the streaming vendor's live bank bit-exactly. The
 * runnable proof is kernel/scripts/isp/check-de3d-geometry.py.
 *
 * Three inputs, all of which the driver already knows:
 *
 *   width   the padded frame width, 1920 + 8 for the spatial filter's border.
 *           This is the same value the block's own 0x2e04 carries, so it is
 *           read back rather than assumed.
 *   height  the frame height.
 *   hts     the sensor's line length in pixels, 2200 at 1080p60: the pixel
 *           clock divided by the frame rate and the frame length in lines.
 *
 * Pure data transforms: no register access and no kernel API beyond the
 * integer types, so the same source compiles host-side against the captures.
 */

#ifndef AR_ISP_DE3D_GEOM_H
#define AR_ISP_DE3D_GEOM_H

#define AR_ISP_DE3D_GEOM_REGS		13

/*
 * The vendor computes these in float and converts with fcvtps, which rounds
 * toward positive infinity. For the non-negative inputs here that is a ceiling
 * division, which is what these do in integer arithmetic.
 */
#define AR_ISP_DE3D_CEIL(a, b)		(((a) + (b) - 1) / (b))
#define AR_ISP_DE3D_ALIGN256(v)		(((v) + 0xff) & ~0xffu)

/* fmul by 0.0625, 0.125 and the divides by 20 and 320, as the vendor spells
 * them. Named so the arithmetic reads as the block's own units rather than as
 * magic numbers.
 */
#define AR_ISP_DE3D_BLOCK16		16
#define AR_ISP_DE3D_BLOCK8		8
#define AR_ISP_DE3D_TILE20		20
#define AR_ISP_DE3D_TILE320		320

/* The 5% and +200 the line-delay register adds to the horizontal blanking. */
#define AR_ISP_DE3D_DELAY_PCT		5
#define AR_ISP_DE3D_DELAY_BIAS		200

/* 0x2e48's low field, w9 in the vendor, a constant 50 rather than a datum. */
#define AR_ISP_DE3D_THRESH50		50

/*
 * The line length the delay register is built on, in the ISP's pixel units.
 *
 * It belongs to the sensor, not to the ISP, and no ISP register carries it, so
 * it is stated here against its source rather than read back. The nt99235
 * programs LINE_LENGTH_PCK (SMIA 0x0342/0x0343) to 0x044c in every one of its
 * four mode tables, and the ISP counts two pixels per sensor clock: at 1080p60
 * the sensor's 60 x 1125 x 1100 is a 74.25 MHz pixel clock, and the ISP's line
 * is the 148.5 MHz half-period count, 2200.
 *
 * kernel/scripts/isp/check-de3d-geometry.py reads nt99235.c's mode tables and
 * fails if any of them stops programming AR_ISP_DE3D_LINE_LENGTH_PCK, so a new
 * mode with a different line length is caught rather than silently mis-delayed.
 */
#define AR_ISP_DE3D_LINE_LENGTH_PCK	1100
#define AR_ISP_DE3D_ISP_PIXELS_PER_PCK	2
#define AR_ISP_DE3D_HTS			(AR_ISP_DE3D_LINE_LENGTH_PCK * \
					 AR_ISP_DE3D_ISP_PIXELS_PER_PCK)

/* The work mode the vendor passes; only bit 0 reaches 0x2e7c. */
#define AR_ISP_DE3D_WORK_MODE		1

struct ar_isp_de3d_geom_reg {
	u16 off;	/* from AR_ISP_DE3D_BANK */
	u32 mask;	/* the bits this pass owns; the rest are left alone */
};

/*
 * Register order matches ar_isp_de3d_geom_pack's output. Masks matter: several
 * of these share a word with the submodule's static image, and the vendor
 * read-modify-writes rather than replacing the word.
 */
static const struct ar_isp_de3d_geom_reg ar_isp_de3d_geom_regs[] = {
	{ 0x24, 0x9fff0000 },	/* line delay, bits 28:16; bit 31 cleared */
	{ 0x38, 0x1001ffff },	/* buffer 0 descriptor and stride */
	{ 0x40, 0x8011ffff },	/* buffer 1 descriptor and stride */
	{ 0x48, 0x1fff1fff },	/* line delay again, plus the constant 50 */
	{ 0x54, 0xffff0f07 },	/* block counts and mode */
	{ 0x5c, 0xffffffff },	/* buffer 0 stride, second copy */
	{ 0x64, 0xffffffff },	/* buffer 1 stride, second copy */
	{ 0x68, 0x0000003f },	/* tile count across the padded width */
	{ 0x74, 0x00003f01 },	/* the tile pad, and the channel bit below */
	{ 0x78, 0x80f00000 },	/* buffer 2 descriptor bits */
	{ 0x7c, 0x00000001 },	/* work-mode bit */
	{ 0x84, 0xffffffff },	/* buffer 2 stride */
	{ 0x8c, 0xffffffff },	/* buffer 2 stride, second copy */
};

/*
 * ar_isp_de3d_geom_pack - the thirteen values, in ar_isp_de3d_geom_regs order.
 *
 * @dst:    AR_ISP_DE3D_GEOM_REGS words out
 * @width:  padded frame width in pixels
 * @height: frame height in lines
 * @hts:    sensor line length in pixels
 * @mode:   the work mode the vendor passes as its third argument; only bit 0
 *          reaches a register
 *
 * @height is taken because the block's geometry pair is derived from it and a
 * caller that has one has the other; it does not feed these thirteen, which is
 * why it is unused here and the pair at 0x2e08 stays with the geometry class.
 */
static inline void ar_isp_de3d_geom_pack(u32 *dst, u32 width, u32 height,
					 u32 hts, u32 mode)
{
	/*
	 * The working stride. The vendor takes the padded width in tiles of
	 * 20, gives each tile 16 bytes, and rounds the line up to 256. Both
	 * buffer 0 and buffer 1 use it; buffer 2 carries a block-of-8 stride
	 * instead, so it rounds a different quotient up to the same boundary.
	 */
	u32 tiles = AR_ISP_DE3D_CEIL(width, AR_ISP_DE3D_TILE20);
	u32 stride = AR_ISP_DE3D_ALIGN256(tiles * AR_ISP_DE3D_BLOCK16);
	u32 stride2 = AR_ISP_DE3D_ALIGN256(AR_ISP_DE3D_CEIL(width,
							    AR_ISP_DE3D_BLOCK8));

	/*
	 * The line delay: horizontal blanking, plus five percent of it, plus a
	 * fixed 200. The vendor truncates the percentage on its own before
	 * adding, so the two terms are computed separately here as well.
	 */
	u32 blank = hts - width;
	u32 delay = blank + (blank * AR_ISP_DE3D_DELAY_PCT) / 100 +
		    AR_ISP_DE3D_DELAY_BIAS;

	/* Blocks of 16 across the padded width, and the residue after seven
	 * of those are taken out of half the width.
	 */
	u32 blocks = AR_ISP_DE3D_CEIL(width, AR_ISP_DE3D_BLOCK16);

	(void)height;

	dst[0] = delay << 16;
	dst[1] = (1u << 28) | (1u << 16) | (stride & 0xffff);
	dst[2] = (1u << 31) | (1u << 20) | (1u << 16) | (stride & 0xffff);
	dst[3] = (delay << 16) | AR_ISP_DE3D_THRESH50;
	dst[4] = ((blocks - 1) << 24) |
		 (((width / 2) - 7 * blocks - 1) << 16) | 4;
	dst[5] = stride;
	dst[6] = stride;
	dst[7] = AR_ISP_DE3D_CEIL(width, AR_ISP_DE3D_TILE320);
	/*
	 * Bits 13:8 are the pad. Bit 0 is a channel bit the vendor's second
	 * de3d function sets on one path and clears on the other, at 0x1c5930
	 * and 0x1c5ad8: it clears when the module's two flags at +824 and +992
	 * both read zero, which is the single-channel path every register in
	 * this pass was derived from and the one the streaming vendor is on.
	 */
	dst[8] = ((AR_ISP_DE3D_TILE20 - (width % AR_ISP_DE3D_TILE20)) %
		  AR_ISP_DE3D_TILE20) << 8;
	dst[9] = (1u << 31) | (1u << 23);
	dst[10] = mode & 1;
	dst[11] = stride2;
	dst[12] = stride2;
}

#endif /* AR_ISP_DE3D_GEOM_H */
