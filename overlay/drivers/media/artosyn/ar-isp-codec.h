/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-codec.h - packing formats for the ISP's DMA coefficient tables.
 *
 * The ISP reads DMA buffers whose physical addresses are published to
 * descriptor registers and fetched on a write to 0x0014. Filling those buffers
 * is a software job, done by vendor userspace from the tuning file; these are
 * the packing formats it uses.
 *
 * Recovered from libmpp_service.so and validated byte-for-byte against buffers
 * captured off a streaming vendor unit. Reference implementations and capture
 * checks: glue/isp/gamma-codec.py and glue/isp/drc-codec.py.
 *
 * Pure data transforms: no register access and no kernel API beyond the integer
 * types, so the same source can be compiled host-side against the captures.
 */

#ifndef AR_ISP_CODEC_H
#define AR_ISP_CODEC_H

#include "ar-isp-bytes.h"

/* Gamma: a 0x4000 allocation whose first two 0x800 pages are transfer curves. */
#define AR_ISP_GAMMA_SIZE		0x4000
#define AR_ISP_GAMMA_PAGE		0x800
#define AR_ISP_GAMMA_RECORDS		128
#define AR_ISP_GAMMA_SAMPLES		512

/*
 * Curve source in the tuning file: five 0x4000-byte curves of 4096 little-endian
 * u32, selected by an AE-derived index. Only every eighth entry is used, which is
 * how 4096 blob entries become the 512 the packed page holds.
 */
#define AR_ISP_GAMMA_BLOB_OFFSET	0x26b90
#define AR_ISP_GAMMA_BLOB_STRIDE	0x4000
#define AR_ISP_GAMMA_BLOB_CURVES	5
#define AR_ISP_GAMMA_BLOB_ENTRIES	4096
#define AR_ISP_GAMMA_DECIMATE		8

/* DRC: a 0x2000 allocation of four banks, each a 257-sample curve. */
#define AR_ISP_DRC_SIZE			0x2000
#define AR_ISP_DRC_BANK			0x800
#define AR_ISP_DRC_RECORDS		128
#define AR_ISP_DRC_SAMPLES		257
#define AR_ISP_DRC_LANE_MAX		0xfffff

/*
 * DRC profile table in the tuning file, selected by the same AE-derived index.
 * A profile holds the two banks the dynamic half of the page is built from, at
 * its start and at +0x404.
 */
#define AR_ISP_DRC_BLOB_OFFSET		0x17b1c
#define AR_ISP_DRC_BLOB_STRIDE		0xc8c
#define AR_ISP_DRC_BLOB_BANK1		0x404

/*
 * LSC: a 0x680 fetch in three parts. The first 0x340 is a 10x10 lens-shading
 * grid and is generated here. The 0x2c0 after it is scene-adaptive runtime state
 * with no stored source, and the 0x80 tail is zero.
 *
 * The owner is the vendor's isp_sub_lsc on bank 0x4c00: its command handler
 * publishes to +0x34 with the valid bit at +0x3c and a length of 0x34 units,
 * which is this descriptor, and its tuning path reads the same 0x9090 gate.
 *
 * The grid is two 100-entry float32 arrays in the tuning file, stored back to
 * back just past the LSC enable gate at 0x9090. Each entry is a gain, unity at
 * the frame centre and rising to about 3.9 at the corners.
 */
#define AR_ISP_LSC_SIZE			0x680
#define AR_ISP_LSC_REGION_A		0x340
#define AR_ISP_LSC_GRID			100
#define AR_ISP_LSC_BLOB_Y		0x910c
#define AR_ISP_LSC_BLOB_X		0x929c
#define AR_ISP_LSC_Q			11

/*
 * Compander: a 0x7800 page, mostly structure. Two spans carry content, a 0x700
 * span is zero, and everything from 0x1800 is one 16-byte unity record repeated.
 * The carried spans are in vendor-tables/ar-isp-compander.h; the rest is
 * rebuilt below.
 */
#define AR_ISP_COMPANDER_SIZE		0x7800
#define AR_ISP_COMPANDER_HEAD		0x900
#define AR_ISP_COMPANDER_MID_OFF	0x1000
#define AR_ISP_COMPANDER_MID		0x800
#define AR_ISP_COMPANDER_FILL_OFF	0x1800
#define AR_ISP_COMPANDER_FILL0		0x00000100
#define AR_ISP_COMPANDER_FILL1		0x00010000

/*
 * Pack 512 twelve-bit samples into one gamma page.
 *
 * 128 records of 16 bytes hold four samples each. The surplus bits are
 * redundant overlap fields the hardware also reads, so an encoder has to write
 * them or the table is malformed.
 *
 *	sample[4i+0] =  word0        & 0xfff
 *	sample[4i+1] = (word0 >> 12) & 0xfff
 *	sample[4i+2] = (word1 >> 16) & 0xfff
 *	sample[4i+3] = (word2 >>  8) & 0xfff
 *
 *	word0[31:24] = sample[4i+1][7:0]
 *	word1[15:0]  = sample[4i+1][11:8] | sample[4i+2] << 4
 *	word2[7:0]   = sample[4i+3] >> 4
 *	word2[31:20] = the NEXT record's first sample
 *	word3        = 0
 *
 * The last record has no next sample inside the page, so its forward field is
 * passed in as tail. It must not be left zero: the hardware interpolates the
 * top curve segment towards it, and a zero maps the brightest inputs to black
 * (hardware-observed as every highlight in the frame wrapping).
 */
static inline void ar_isp_gamma_pack_page(u8 *dst, const u16 *samples, u16 tail)
{
	for (unsigned int i = 0; i < AR_ISP_GAMMA_RECORDS; i++) {
		u32 s0 = samples[4 * i + 0] & 0xfff;
		u32 s1 = samples[4 * i + 1] & 0xfff;
		u32 s2 = samples[4 * i + 2] & 0xfff;
		u32 s3 = samples[4 * i + 3] & 0xfff;
		u32 next;
		u8 *rec = dst + i * 16;

		if (i + 1 < AR_ISP_GAMMA_RECORDS)
			next = samples[4 * (i + 1)] & 0xfff;
		else
			next = tail & 0xfff;

		ar_isp_put_le32(rec + 0, s0 | (s1 << 12) | ((s1 & 0xff) << 24));
		ar_isp_put_le32(rec + 4, ((s1 >> 8) & 0xf) | (s2 << 4) |
					 (s2 << 16) | ((s3 & 0xf) << 28));
		ar_isp_put_le32(rec + 8, (s3 >> 4) | (s3 << 8) | (next << 20));
		ar_isp_put_le32(rec + 12, 0);
	}
}

/*
 * Build one gamma page from the tuning file.
 *
 * The decimation rounds down systematically, by at most 5 counts in 4090 and
 * typically one, always low: sub-LSB at the output's precision (mean 0.047 on
 * 8-bit luma against the vendor's captured buffer).
 *
 * The monotonic clamp matters: the hardware reads a transfer curve, and a
 * non-monotonic entry inverts the response over that interval.
 */
static inline void ar_isp_gamma_from_blob(u8 *dst, const u8 *blob,
					  unsigned int index)
{
	const u8 *curve = blob + AR_ISP_GAMMA_BLOB_OFFSET + index * AR_ISP_GAMMA_BLOB_STRIDE;
	u16 samples[AR_ISP_GAMMA_SAMPLES];
	unsigned int i;

	for (i = 0; i < AR_ISP_GAMMA_SAMPLES; i++)
		samples[i] = ar_isp_get_le32(curve + i * AR_ISP_GAMMA_DECIMATE * 4) & 0xfff;

	for (i = 1; i < AR_ISP_GAMMA_SAMPLES; i++)
		if (samples[i] < samples[i - 1])
			samples[i] = samples[i - 1];

	/*
	 * The sample after the last one. Decimation puts the last stored sample
	 * at entry 4088, past the decimated range; the vendor uses the curve's
	 * final entry, which reproduces its forward field of 4093 on both
	 * captured curves. Clamped after the loop above, so it cannot sit below
	 * the last sample the clamp raised.
	 */
	u16 tail = ar_isp_get_le32(curve + (AR_ISP_GAMMA_BLOB_ENTRIES - 1) * 4) & 0xfff;

	if (tail < samples[AR_ISP_GAMMA_SAMPLES - 1])
		tail = samples[AR_ISP_GAMMA_SAMPLES - 1];

	ar_isp_gamma_pack_page(dst, samples, tail);
}

/*
 * Pack a 257-sample curve into one DRC bank.
 *
 * 128 records of 16 bytes, each carrying three unsigned 20-bit samples in its
 * first ten bytes. Consecutive records overlap by one sample, so record i is
 * [sample[2i], sample[2i+1], sample[2i+2]] and 128 records carry 257 samples.
 *
 *	sample[2i+0] = word0[19:0]
 *	sample[2i+1] = word0[31:20] | word1[7:0] << 12
 *	sample[2i+2] = word1[31:28] | halfword[8][15:0] << 4
 *	word1[27:8]  = a second complete copy of sample[2i+1]
 *
 * Bytes 10 to 15 are left alone by the vendor packer, retaining preloaded
 * template data; that data is zero across all 256 records of both the static
 * template and a live capture, so this writes the whole record and needs no
 * template.
 */
static inline void ar_isp_drc_pack_bank(u8 *dst, const u32 *samples)
{
	for (unsigned int i = 0; i < AR_ISP_DRC_RECORDS; i++) {
		u32 s0 = samples[2 * i + 0] & AR_ISP_DRC_LANE_MAX;
		u32 s1 = samples[2 * i + 1] & AR_ISP_DRC_LANE_MAX;
		u32 s2 = samples[2 * i + 2] & AR_ISP_DRC_LANE_MAX;
		u8 *rec = dst + i * 16;

		ar_isp_put_le32(rec + 0, s0 | (s1 << 20));
		ar_isp_put_le32(rec + 4, (s1 >> 12) | (s1 << 8) | (s2 << 28));
		ar_isp_put_le16(rec + 8, s2 >> 4);
		ar_isp_put_le16(rec + 10, 0);
		ar_isp_put_le32(rec + 12, 0);
	}
}

/*
 * Build the dynamic half of the DRC page from the tuning file.
 *
 * Builds the first 0x1000 of the 0x2000 page. The second half is constant and
 * lives in vendor-tables/ar-isp-drc-tail.h.
 *
 * The neutral-strength form. The vendor blends the source against one of two
 * service tables once the user DRC strength moves off 50.
 */
static inline void ar_isp_drc_from_blob(u8 *dst, const u8 *blob,
					unsigned int profile)
{
	u32 samples[AR_ISP_DRC_SAMPLES];

	for (unsigned int b = 0; b < 2; b++) {
		const u8 *bank = blob + AR_ISP_DRC_BLOB_OFFSET +
				 profile * AR_ISP_DRC_BLOB_STRIDE +
				 b * AR_ISP_DRC_BLOB_BANK1;

		for (unsigned int i = 0; i < AR_ISP_DRC_SAMPLES; i++)
			samples[i] = ar_isp_get_le32(bank + i * 4);

		ar_isp_drc_pack_bank(dst + b * AR_ISP_DRC_BANK, samples);
	}
}

/*
 * floor(f * 2048) for an IEEE-754 single, in integer arithmetic.
 *
 * The kernel has no FPU and the tuning file stores the lens-shading grid as
 * float32, so the vendor's quantisation is reproduced from the bit pattern.
 * Truncated toward zero: truncation matches all 100 grid entries against a
 * captured page, rounding only 55.
 *
 * Every grid entry is a positive gain between 1 and 4, so the general cases
 * cannot occur for a valid tuning file; the clamp exists so a corrupt one
 * cannot write past the 16-bit field.
 */
static inline u16 ar_isp_f32_scale(u32 bits)
{
	u32 mant = ar_isp_f32_mant(bits);
	int shift = (AR_ISP_F32_MANT_BITS - AR_ISP_LSC_Q) - ar_isp_f32_exp(bits);
  u32 scaled;

	if (bits & 0x80000000)
		return 0;

	if (shift >= 32)
		return 0;

	if (shift <= 0)
		return 0xffff;

	scaled = mant >> shift;
	return scaled > 0xffff ? 0xffff : (u16)scaled;
}

/*
 * Build the LSC lens-shading grid from the tuning file.
 *
 * 100 grid points over the frame, packed two to a 16-byte record as a pair of
 * (x, x, y) triplets. The duplicated first element is the same redundant
 * overlap field gamma and DRC carry: the hardware reads it, so an encoder has
 * to write it.
 *
 * 50 records hold the grid; records 50 and 51 are zero, as are the last four
 * bytes of every record. Zeroing the region first covers all three.
 *
 * Covers region A. The 0x2c0 of scene-adaptive state after it is computed at
 * runtime from image statistics.
 */
static inline void ar_isp_lsc_from_blob(u8 *dst, const u8 *blob)
{
	unsigned int i;

	for (i = 0; i < AR_ISP_LSC_REGION_A; i += 4)
		ar_isp_put_le32(dst + i, 0);

	for (i = 0; i < AR_ISP_LSC_GRID; i++) {
		u16 x = ar_isp_f32_scale(ar_isp_get_le32(blob + AR_ISP_LSC_BLOB_X + i * 4));
		u16 y = ar_isp_f32_scale(ar_isp_get_le32(blob + AR_ISP_LSC_BLOB_Y + i * 4));
		u8 *rec = dst + (i / 2) * 16 + (i % 2) * 6;

		ar_isp_put_le16(rec + 0, x);
		ar_isp_put_le16(rec + 2, x);
		ar_isp_put_le16(rec + 4, y);
	}
}

/*
 * Rebuild the compander page from its two carried spans.
 *
 * The vendor installs this page verbatim from a static template, so there is
 * no curve to encode. This reconstructs the three quarters of the page that is
 * constant by structure.
 *
 * The unity record writes only its first two words; the remaining eight bytes
 * are zero in all 1536 template records.
 */
static inline void ar_isp_compander_fill(u8 *dst, const u32 *head, const u32 *mid)
{
	unsigned int i;

	for (i = 0; i < AR_ISP_COMPANDER_HEAD / 4; i++)
		ar_isp_put_le32(dst + i * 4, head[i]);

	for (i = AR_ISP_COMPANDER_HEAD; i < AR_ISP_COMPANDER_MID_OFF; i += 4)
		ar_isp_put_le32(dst + i, 0);

	for (i = 0; i < AR_ISP_COMPANDER_MID / 4; i++)
		ar_isp_put_le32(dst + AR_ISP_COMPANDER_MID_OFF + i * 4, mid[i]);

	for (i = AR_ISP_COMPANDER_FILL_OFF; i < AR_ISP_COMPANDER_SIZE; i += 16) {
		ar_isp_put_le32(dst + i + 0, AR_ISP_COMPANDER_FILL0);
		ar_isp_put_le32(dst + i + 4, AR_ISP_COMPANDER_FILL1);
		ar_isp_put_le32(dst + i + 8, 0);
		ar_isp_put_le32(dst + i + 12, 0);
	}
}

#endif /* AR_ISP_CODEC_H */
