/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ladder-dump.c - run the shipped gain-ladder headers host-side.
 *
 * The five gain-keyed stage headers are written as pure data transforms so the
 * kernel source itself can be compiled on the host; this is the program that
 * does it. It includes the gain-keyed stage headers unmodified out of the
 * driver directory, runs each stage's from_blob at one abscissa and prints what
 * the applier would write.
 *
 * The check scripts beside it each restate a stage's arithmetic in Python and
 * prove that restatement against captured register state. That proves the
 * recovery, not the driver: a divergence between the Python and the C is
 * invisible to them, and the C is what runs on the hardware.
 * check-ladder-c.py closes that by diffing this output against the same
 * Python models.
 *
 * Output is one "stage index value" line per register, hex without a prefix,
 * so the reader needs no bank knowledge. lnr is seeded with zeros rather than
 * a register readback: it packs fields into a bank it read back first, so its
 * output is only defined relative to a seed, and both sides use the same one.
 *
 *   cc -I<driver dir> -o ladder-dump ladder-dump.c
 *   ./ladder-dump <tuning.bin> <gain-q8>
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * The headers use the kernel's fixed-width names and nothing else, so the
 * whole compatibility layer is these typedefs.
 */
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int32_t s32;
typedef int64_t s64;

#include "ar-isp-rnr.h"
#include "ar-isp-lnr.h"
#include "ar-isp-de3d.h"
#include "ar-isp-cfa.h"
#include "ar-isp-cnf.h"
#include "ar-isp-cm.h"
#include "ar-isp-cm2.h"

/* Large enough for every sensor's tuning file; the shipped one is 859 KiB. */
#define BLOB_MAX (2 * 1024 * 1024)

static void emit(const char *stage, unsigned int index, u32 value)
{
	printf("%s %u %08x\n", stage, index, value);
}

static u8 *read_blob(const char *path, size_t *len_out)
{
	u8 *blob = calloc(1, BLOB_MAX);
	FILE *f = fopen(path, "rb");

	if (!blob || !f) {
		perror(path);
		return NULL;
	}

	*len_out = fread(blob, 1, BLOB_MAX, f);
	fclose(f);

	return blob;
}

int main(int argc, char **argv)
{
	u32 rnr[AR_ISP_RNR_REGS], tail[AR_ISP_RNR_TAIL_REGS];
	u32 lnr[AR_ISP_LNR_REGS], de3d[AR_ISP_DE3D_REGS];
	u32 cfa[AR_ISP_CFA_REGS];
	struct ar_isp_cm2_row cm2;
	unsigned int i, run, k;
	u32 gain_q8, gain_q16, strength;
	size_t len;
	u8 *blob;

	if (argc != 3) {
		fprintf(stderr, "usage: %s <tuning.bin> <gain-q8>\n", argv[0]);
		return 2;
	}

	blob = read_blob(argv[1], &len);
	if (!blob)
		return 1;

	gain_q8 = (u32)strtoul(argv[2], NULL, 0);
	gain_q16 = gain_q8 << 8;

	ar_isp_rnr_from_blob(rnr, blob, gain_q16);
	ar_isp_rnr_tail_from_blob(tail, blob, gain_q16);

	for (i = 0; i < AR_ISP_RNR_REGS; i++)
		emit("rnr", i, rnr[i]);

	for (i = 0; i < AR_ISP_RNR_TAIL_REGS; i++)
		emit("rnr_tail", i, tail[i]);

	/* Seeded with zeros; see the file comment. */
	memset(lnr, 0, sizeof(lnr));
	ar_isp_lnr_from_blob(lnr, blob, gain_q16);

	for (i = 0; i < AR_ISP_LNR_REGS; i++)
		emit("lnr", i, lnr[i]);

	ar_isp_de3d_from_blob(de3d, blob, gain_q16);

	for (i = 0; i < AR_ISP_DE3D_REGS; i++)
		emit("de3d", i, de3d[i] & ar_isp_de3d_regs[i].mask);

	/*
	 * cfa is indexed in run order, the order the applier walks the four
	 * ascending runs in, so the index is a position in the emitted sequence
	 * rather than a bank offset. The Python model returns the same order.
	 */
	ar_isp_cfa_from_blob(cfa, blob, gain_q16);

	for (run = 0, i = 0; run < AR_ISP_CFA_RUNS; run++)
		for (k = 0; k < ar_isp_cfa_runs[run].count; k++, i++)
			emit("cfa", i, cfa[i]);

	strength = ar_isp_cnf_strength_from_blob(blob, gain_q16);
	emit("cnf_strength", 0, strength);
	emit("cnf", 0, ar_isp_cnf_pack(strength));
	emit("cnf", 1, ar_isp_cnf_norm_pack(strength) | AR_ISP_CNF_NORM_A_BIT);
	emit("cnf", 2, ar_isp_cnf_norm_pack(AR_ISP_CNF_NORM_CONST_B));

	for (i = 0; i < AR_ISP_CNF_STATIC_REGS; i++) {
		u32 mask;
		u32 v = ar_isp_cnf_static_pack(blob, i, &mask);

		emit("cnf_static", i, v & mask);
	}

	emit("cm", 0, ar_isp_cm_gain_field_from_blob(blob, gain_q8, 0));

	ar_isp_cm2_from_blob(&cm2, blob, gain_q8, 0);
	emit("cm2", 0, cm2.gain_field);
	emit("cm2", 1, cm2.lo1);
	emit("cm2", 2, cm2.hi1);
	emit("cm2", 3, cm2.lo2);
	emit("cm2", 4, cm2.hi2);
	emit("cm2", 5, cm2.recip1);
	emit("cm2", 6, cm2.recip2);

	free(blob);

	return 0;
}
