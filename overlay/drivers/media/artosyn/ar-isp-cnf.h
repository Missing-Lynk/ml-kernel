/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-cnf.h - the cnf gain ladder.
 *
 * Recovered from the cnf driver in libmpp_service.so at 0x1a1f68 and its packer
 * at 0x1a1c28. The runnable proof is kernel/scripts/isp/check-cnf-ladder.py,
 * which reruns this arithmetic in Python and refuses to pass if it stops
 * reproducing the measured register state from the tuning file.
 *
 * Pure data transforms: no register access and no kernel API beyond the
 * integer types, so the same source can be compiled host-side against the
 * captures. See ar-isp-ladder.h for the ladder shape.
 *
 * Where every value in this bank comes from, all 16 registers:
 *
 *   9  computed here from the tuning file: the strength in 0x3c64, its
 *      normalisation in 0x3c84 and 0x3c88, and the static run 0x3c8c to
 *      0x3ca0
 *   6  the module's static register image, which the vendor service carries
 *      in its own data: 0x3c68 to 0x3c80, the second copy of the strength
 *      group and its two companions
 *   1  0x3c74, whose bit 0 the vendor clears in the same basic block that
 *      sets bit 0 of 0x3c64, at 0x1a25c8 against 0x1a25bc. No instruction in
 *      the library ever sets it, and the packer never writes this offset, so
 *      it is neither an enable nor a copy of the strength.
 */

#ifndef AR_ISP_CNF_H
#define AR_ISP_CNF_H

#include "vendor-tables/ar-isp-blob.h"
#include "ar-isp-ladder.h"

/*
 * Register bank. Three registers carry ladder output, all keyed on the same
 * strength: 0x3c64 holds it in bits 4:1 with bits 7:5 set to 1, and 0x3c84 and
 * 0x3c88 hold a reciprocal-square normalisation of it. The apply path
 * read-modify-writes each under its own mask.
 *
 * The remaining registers 0x3c8c through 0x3ca0 are packed from a static
 * parameter block in the tuning file at 0x8e1a8..0x8e1d4, with no gain key.
 * Each takes an 11-bit field from one blob word and a second field from
 * another; the two field widths alternate down the run.
 */
#define AR_ISP_CNF_STRENGTH_REG		0x3c64
#define AR_ISP_CNF_STRENGTH_MASK	0x000000fe
#define AR_ISP_CNF_STRENGTH_SHIFT	1
#define AR_ISP_CNF_STRENGTH_BITS	0xf
#define AR_ISP_CNF_ENABLE_BITS		0x00000020

/*
 * The two normalised registers. Each holds a 12-bit quotient in bits 11:0 and
 * the shift that produced it in bits 16:12; 0x3c84 additionally carries bit 17.
 * 0x3c84 normalises the strength itself and 0x3c88 the constant 2.0, so only
 * the first moves with gain.
 */
#define AR_ISP_CNF_NORM_REG_A		0x3c84
#define AR_ISP_CNF_NORM_REG_B		0x3c88
#define AR_ISP_CNF_NORM_MASK		0x0003ffff
#define AR_ISP_CNF_NORM_B_MASK		0x0001ffff
#define AR_ISP_CNF_NORM_QUOT_BITS	0xfff
#define AR_ISP_CNF_NORM_SHIFT_POS	12
#define AR_ISP_CNF_NORM_A_BIT		0x00020000
#define AR_ISP_CNF_NORM_CONST_B		2

/* The search bounds the packer uses: shift from 30 down, quotient ceiling. */
#define AR_ISP_CNF_NORM_SHIFT_MAX	30
#define AR_ISP_CNF_NORM_SHIFT_MIN	9
#define AR_ISP_CNF_NORM_QUOT_MAX	0xffe

/*
 * Ladder in the tuning file. The eleven band edges are the powers-of-two anchor
 * layout, 1.0 to 1024.1, and each 0x80c payload record holds two words: a set
 * flag and the strength value. The strength ramps 1, 1, 2, 2, 3, 4, 5, 5, 5, 5,
 * 6 across the bands and the rest of the stride is padding.
 *
 * The band count and the interpolate flag are constants here rather than file
 * reads, because this ladder carries no four-word header: the words preceding
 * the edge array are (0, 0x80, 0x80, 0x100), which is not the header shape, and
 * the packer takes both quantities from its own code. Locating a header for
 * them is open work.
 *
 * The vendor's strength law scales the value toward 6 or toward 1 by a user
 * strength around a neutral 50, so it is the identity at the constructed
 * default, like rnr's and lnr's, and only its clamp is carried. That clamp is
 * what bounds a corrupt file to a value the field can hold.
 */
#define AR_ISP_CNF_INTERP		1
#define AR_ISP_CNF_WORD_STRENGTH	0x04
#define AR_ISP_CNF_STRENGTH_MIN		1
#define AR_ISP_CNF_STRENGTH_MAX		6

/*
 * The cnf strength at the abscissa, clamped to the range the register field and
 * the vendor's own law both bound it to.
 */
static inline u32 ar_isp_cnf_strength_from_blob(const u8 *blob, u32 gain_q16)
{
	const u8 *payload = blob + AR_ISP_CNF_BLOB_PAYLOAD;
	unsigned int band;
	u32 t_q24;
	s32 value;

	ar_isp_ladder_walk(blob + AR_ISP_CNF_BLOB_BANDS, AR_ISP_CNF_BANDS,
			   AR_ISP_CNF_INTERP, gain_q16, &band, &t_q24);

	value = (s32)ar_isp_get_le32(payload + band * AR_ISP_CNF_BLOB_STRIDE +
				     AR_ISP_CNF_WORD_STRENGTH);

	if (t_q24) {
		s32 prev = (s32)ar_isp_get_le32(payload +
						(band - 1) * AR_ISP_CNF_BLOB_STRIDE +
						AR_ISP_CNF_WORD_STRENGTH);

		value = ar_isp_ladder_blend_s32(prev, value, t_q24);
	}

	if (value < AR_ISP_CNF_STRENGTH_MIN)
		value = AR_ISP_CNF_STRENGTH_MIN;

	if (value > AR_ISP_CNF_STRENGTH_MAX)
		value = AR_ISP_CNF_STRENGTH_MAX;

	return (u32)value;
}

/* The strength packed into the bits of 0x3c64 that the packer owns. */
static inline u32 ar_isp_cnf_pack(u32 strength)
{
	return ((strength & AR_ISP_CNF_STRENGTH_BITS) <<
		AR_ISP_CNF_STRENGTH_SHIFT) | AR_ISP_CNF_ENABLE_BITS;
}

/*
 * The packer's normalisation: the largest shift whose rounded quotient of
 * 2^shift over v squared still fits the 12-bit field, walking down from 30 and
 * stopping at 9.
 *
 * The vendor divides in float32, adds 0.5 in double and truncates, which is
 * round-half-up. The integer form here is exact over the strength range: the
 * accepted quotient is at most 4094, well inside float32's exact range, and no
 * square of an integer strength puts the true quotient on a half boundary.
 */
static inline u32 ar_isp_cnf_norm_quot(u32 square, u32 shift)
{
	return (u32)((((u64)1 << shift) * 2 + square) / (2 * square));
}

static inline void ar_isp_cnf_normalise(u32 v, u32 *quot_out, u32 *shift_out)
{
	u32 square, shift, quot;

	/*
	 * Both call sites pass a strength the ladder has already clamped to
	 * AR_ISP_CNF_STRENGTH_MIN..MAX, so the square is 1..36 and neither the
	 * multiply nor the divisor can go wrong. A header inline cannot rely on
	 * its callers for that, and a zero divisor here is a division fault
	 * rather than a wrong register, so it is floored.
	 */
	if (v < AR_ISP_CNF_STRENGTH_MIN)
		v = AR_ISP_CNF_STRENGTH_MIN;

	square = v * v;
	shift = AR_ISP_CNF_NORM_SHIFT_MAX;
	quot = ar_isp_cnf_norm_quot(square, shift);

	while (quot > AR_ISP_CNF_NORM_QUOT_MAX &&
	       shift > AR_ISP_CNF_NORM_SHIFT_MIN) {
		shift--;
		quot = ar_isp_cnf_norm_quot(square, shift);
	}

	*quot_out = quot;
	*shift_out = shift;
}

/* One normalised register: quotient in bits 11:0, its shift in bits 16:12. */
static inline u32 ar_isp_cnf_norm_pack(u32 v)
{
	u32 quot, shift;

	ar_isp_cnf_normalise(v, &quot, &shift);

	return (quot & AR_ISP_CNF_NORM_QUOT_BITS) |
	       (shift << AR_ISP_CNF_NORM_SHIFT_POS);
}

/*
 * The gain-independent run, 0x3c8c to 0x3ca0. Each register takes bits 10:0
 * from one blob word and a wider field at bit 11 from another; the upper field
 * is 11 bits on the even entries and 9 on the odd ones, which is what the two
 * masks below distinguish.
 */
#define AR_ISP_CNF_STATIC_REG		0x3c8c
#define AR_ISP_CNF_STATIC_REGS		6
#define AR_ISP_CNF_STATIC_HI_POS	11
#define AR_ISP_CNF_STATIC_LO_BITS	0x7ff
#define AR_ISP_CNF_STATIC_WIDE_MASK	0x003ff800
#define AR_ISP_CNF_STATIC_NARROW_MASK	0x000ff800
#define AR_ISP_CNF_STATIC_MASK		0x000007ff

struct ar_isp_cnf_static {
	u32 lo;			/* blob offset feeding bits 10:0 */
	u32 hi;			/* blob offset feeding the field at bit 11 */
	u32 hi_mask;		/* which of the two upper field widths */
};

static const struct ar_isp_cnf_static
ar_isp_cnf_statics[AR_ISP_CNF_STATIC_REGS] = {
	/* B is the run's base; the wide words come first, then the narrow pair. */
#define B AR_ISP_CNF_STATIC_BLOB
	{ B + 0x00, B + 0x0c, AR_ISP_CNF_STATIC_WIDE_MASK },	/* 0x3c8c */
	{ B + 0x18, B + 0x24, AR_ISP_CNF_STATIC_NARROW_MASK },	/* 0x3c90 */
	{ B + 0x04, B + 0x10, AR_ISP_CNF_STATIC_WIDE_MASK },	/* 0x3c94 */
	{ B + 0x1c, B + 0x28, AR_ISP_CNF_STATIC_NARROW_MASK },	/* 0x3c98 */
	{ B + 0x08, B + 0x14, AR_ISP_CNF_STATIC_WIDE_MASK },	/* 0x3c9c */
	{ B + 0x20, B + 0x2c, AR_ISP_CNF_STATIC_NARROW_MASK },	/* 0x3ca0 */
#undef B
};

/* One register of that run, and the mask of the bits it owns. */
static inline u32 ar_isp_cnf_static_pack(const u8 *blob, unsigned int i,
					 u32 *mask_out)
{
	const struct ar_isp_cnf_static *e = &ar_isp_cnf_statics[i];
	u32 lo = ar_isp_get_le32(blob + e->lo) & AR_ISP_CNF_STATIC_LO_BITS;
	u32 hi = ar_isp_get_le32(blob + e->hi) << AR_ISP_CNF_STATIC_HI_POS;

	*mask_out = AR_ISP_CNF_STATIC_MASK | e->hi_mask;

	return lo | (hi & e->hi_mask);
}

#endif /* AR_ISP_CNF_H */
