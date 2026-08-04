/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-colour.h - the ISP's colour correction and 3D-LUT stages.
 *
 * Unlike the tone tables these stages are not DMA pages fetched through
 * descriptors. CCM is a register file: two banks of 0x50 bytes at ISP offsets
 * 0x3400 (ccm1) and 0x3800 (ccm2), each holding two packed 3x3 matrices. LUT3D
 * is a module-local descriptor bank at 0x5800 with four DMA payload banks; the
 * streaming vendor leaves the whole module disabled.
 *
 * Recovered from the module code in libmpp_service.so (isp_sub_ccm1_creat
 * 0x187a50, isp_sub_ccm2_creat 0x1af6e8, isp_sub_lut3d_creat 0x1c3400; each
 * module's second registered handler maps its register bank, its third is the
 * command handler that fills it) and validated word-for-word against the
 * vendor MMIO trace. The runnable proof is kernel/scripts/isp/gen-ccm.py, which
 * packs the tuning-file matrix with this format and refuses to emit if it
 * stops matching the traced registers.
 *
 * Pure data transforms: no register access and no kernel API beyond the integer
 * types, so the same source can be compiled host-side against the captures.
 */

#ifndef AR_ISP_COLOUR_H
#define AR_ISP_COLOUR_H

#include "ar-isp-bytes.h"

/*
 * CCM register banks. Each is 0x50 bytes: a packed matrix at +0x00, a second
 * packed matrix at +0x20, and a 0x10 zero tail at +0x40. The vendor's runtime
 * matrix updates write only the first copy; on ccm1 the second copy keeps the
 * identity it was initialised with, on ccm2 both copies carry the same values.
 */
#define AR_ISP_CCM1_BANK		0x3400
#define AR_ISP_CCM2_BANK		0x3800
#define AR_ISP_CCM_BLOCK		0x50
#define AR_ISP_CCM_COPY_B		0x20
#define AR_ISP_CCM_WORDS		6

/*
 * CCM source data in the tuning file. The gate enables the AWB-driven ccm1
 * path; the ladder is eight illuminants in kelvin and the matrix banks are
 * unpacked float32 3x3, row major. Eight slots are allocated and four hold
 * data, which matches the bank count in the selector block.
 *
 * ccm2 has its own gate at 0x2595c; it reads 0 in this blob, so ccm2 keeps its
 * static init block (carried in vendor-tables/ar-isp-ccm-init.h) for the whole
 * session.
 */
#define AR_ISP_CCM_BLOB_GATE		0x253fc
#define AR_ISP_CCM_BLOB_LADDER		0x25438
#define AR_ISP_CCM_BLOB_ILLUMINANTS	8
#define AR_ISP_CCM_BLOB_BANKS		0x25470
#define AR_ISP_CCM_BLOB_BANK_STRIDE	0x24
#define AR_ISP_CCM_BLOB_BANKS_USED	4
#define AR_ISP_CCM2_BLOB_GATE		0x2595c

/*
 * LUT3D module-local descriptor bank. +0x00 is the module control word, bit 0
 * enable; the vendor's working configuration writes it to 0 and the bank reads
 * all zero on the streaming unit, so the module is present but off. Four
 * descriptor records of 0x18 bytes start at +0x04: length in 16-byte records
 * at +0x00 (the vendor writes 0x280, an over-fetch of the 0x2680 payload),
 * DMA address at +0x0c, valid bit 0 at +0x14.
 *
 * The payloads have no tuning-file source: the init handler copies entries 42
 * to 45 of the ISP-init template array verbatim (VMAs 0x458960, 0x4562e0,
 * 0x453c60, 0x4515e0, 0x2680 each, flushed as 0x2800). They are not carried
 * in-tree while the module stays disabled.
 */
#define AR_ISP_LUT3D_BANK		0x5800
#define AR_ISP_LUT3D_CTRL		0x00
#define AR_ISP_LUT3D_DESC0		0x04
#define AR_ISP_LUT3D_DESC_STRIDE	0x18
#define AR_ISP_LUT3D_DESC_LEN		0x00
#define AR_ISP_LUT3D_DESC_ADDR		0x0c
#define AR_ISP_LUT3D_DESC_VALID		0x14
#define AR_ISP_LUT3D_BANKS		4
#define AR_ISP_LUT3D_LEN_RECORDS	0x280
#define AR_ISP_LUT3D_PAYLOAD		0x2680
#define AR_ISP_LUT3D_FLUSH		0x2800

/*
 * A CCM coefficient from an IEEE-754 single, in integer arithmetic.
 *
 * The vendor converts with fcvtzs #8, so the magnitude is Q8 truncated toward
 * zero, and encodes the sign separately: a negative coefficient v becomes
 * 0x8000 + |v|. Sign-magnitude, not two's complement; 0x8040 is -0.25.
 * Truncation, not rounding: all nine traced ccm1 coefficients match it, and
 * several (-0.1, -0.05, -0.2) would differ under rounding.
 *
 * Valid tuning matrices stay well inside +/-4, so the clamp exists only so a
 * corrupt file cannot spill the sign bit.
 */
static inline u16 ar_isp_f32_q8sm(u32 bits)
{
	u32 mant = (bits & 0x7fffff) | 0x800000;
	int exp = (int)((bits >> 23) & 0xff) - 127;
	int shift = 15 - exp;
	u32 mag;

	if (!(bits & 0x7fffffff))
		return 0;

	if (shift >= 32)
		mag = 0;
	else if (shift <= 0)
		mag = 0x7fff;
	else
		mag = mant >> shift;

	if (mag > 0x7fff)
		mag = 0x7fff;

	return (bits & 0x80000000) ? (u16)(0x8000 | mag) : (u16)mag;
}

/*
 * Pack one 3x3 matrix into the six-word register form.
 *
 * Word layout, from the ccm1 pack routine at library VMA 0x1867c8:
 *
 *	word0 = c0 | c1 << 16
 *	word1 = c2
 *	word2 = c3 | c4 << 16
 *	word3 = c5
 *	word4 = c6 | c7 << 16
 *	word5 = c8
 *
 * The vendor preserves the high halves of words 1, 3 and 5 when it rewrites a
 * matrix; they are zero in both init templates and in every traced write, so
 * writing whole words with zero high halves reproduces the register file.
 *
 * bits holds the nine row-major coefficients as float32 bit patterns, read
 * straight out of a tuning-file bank.
 */
static inline void ar_isp_ccm_pack(u8 *dst, const u32 *bits)
{
	for (unsigned int row = 0; row < 3; row++) {
		u16 a = ar_isp_f32_q8sm(bits[3 * row + 0]);
		u16 b = ar_isp_f32_q8sm(bits[3 * row + 1]);
		u16 c = ar_isp_f32_q8sm(bits[3 * row + 2]);

		ar_isp_put_le32(dst + 8 * row, (u32)a | ((u32)b << 16));
		ar_isp_put_le32(dst + 8 * row + 4, c);
	}
}

/*
 * Pack a tuning-file matrix bank. @bank is 0 to 3; the blob stores the nine
 * float32 coefficients contiguously at the bank's offset.
 */
static inline void ar_isp_ccm_from_blob(u8 *dst, const u8 *blob,
					unsigned int bank)
{
	const u8 *src = blob + AR_ISP_CCM_BLOB_BANKS +
			bank * AR_ISP_CCM_BLOB_BANK_STRIDE;
	u32 bits[9];

	for (unsigned int i = 0; i < 9; i++)
		bits[i] = ar_isp_get_le32(src + i * 4);

	ar_isp_ccm_pack(dst, bits);
}

#endif /* AR_ISP_COLOUR_H */
