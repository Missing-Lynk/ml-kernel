// SPDX-License-Identifier: GPL-2.0
/*
 * cfa-gap-test.c - drive ar-isp-cfa.h against the measured vendor banks.
 *
 * The header's gap path is the one that was wrong until 2026-08-17, and it is
 * only exercised at abscissas that fall between two bands. This test sweeps the
 * abscissa and requires that each captured bank is reproduced exactly at some
 * point in the bracket an independent ladder (rnr) derived for that capture.
 *
 * Usage: cfa-gap-test <tuning.bin>
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int32_t s32;
typedef int64_t s64;

#define U32_MAX 0xffffffffu
#define S32_MAX 0x7fffffff
#define S32_MIN (-0x7fffffff - 1)

static inline int fls64(u64 v)
{
	return v ? 64 - __builtin_clzll(v) : 0;
}

static inline u64 div64_u64_rem(u64 a, u64 b, u64 *rem)
{
	*rem = a % b;
	return a / b;
}

#include "../../overlay/drivers/media/artosyn/ar-isp-softfloat.h"

#include "../../overlay/drivers/media/artosyn/ar-isp-ladder.h"
#include "../../overlay/drivers/media/artosyn/ar-isp-cfa.h"

/*
 * The measured banks, in run order, and the abscissa bracket rnr derived for
 * each from the same breath. The two gap captures are the ones that matter; the
 * in-band pair guards the verbatim path against a regression.
 */
struct capture {
	const char *name;
	double lo, hi;
	bool gap;
	u32 reg[41];
};

static const struct capture CAPTURES[] = {
	{ "breath-light1  band 2, verbatim", 22.6094, 22.7344, false,
	  { 30, 30, 30, 30, 33, 26, 29, 75,
	    53, 37, 18, 500, 4000, 10000, 3000000, 1,
	    128, 0, 40, 6, 128, 0, 64, 64,
	    128, 0, 64, 64, 115, 13, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0,
	    0 } },
	{ "breath-covered band 3, verbatim", 63.5781, 63.9844, false,
	  { 30, 30, 30, 30, 33, 26, 29, 75,
	    53, 37, 18, 500, 4000, 10000, 3000000, 1,
	    128, 0, 40, 6, 128, 0, 64, 64,
	    128, 0, 64, 64, 115, 13, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0,
	    0 } },
	{ "breath-gap44-b gap 42.1..48.0", 44.4375, 44.8594, true,
	  { 29, 29, 29, 29, 33, 26, 31, 74,
	    53, 37, 18, 500, 4000, 10000, 3000000, 1,
	    128, 0, 40, 6, 128, 0, 64, 64,
	    128, 0, 64, 64, 64, 13, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0,
	    0 } },
	{ "breath-l2      gap 12.1..16.0", 12.9648, 13.5664, true,
	  { 30, 30, 30, 30, 33, 26, 31, 75,
	    53, 37, 18, 500, 4000, 10000, 3000000, 1,
	    128, 0, 40, 6, 128, 0, 64, 64,
	    128, 0, 64, 64, 64, 3, 0, 0,
	    7, 179, 0, 0, 1, 0, 5616, 10531,
	    4 } },
};
#define NCAPTURES (sizeof(CAPTURES) / sizeof(CAPTURES[0]))

int main(int argc, char **argv)
{
	u8 *blob;
	long size;
	FILE *f;
	unsigned int c;
	int rc = 0;

	if (argc != 2) {
		fprintf(stderr, "usage: %s <tuning.bin>\n", argv[0]);
		return 2;
	}

	f = fopen(argv[1], "rb");
	if (!f) {
		perror(argv[1]);
		return 2;
	}

	fseek(f, 0, SEEK_END);
	size = ftell(f);
	fseek(f, 0, SEEK_SET);
	blob = malloc(size);
	if (!blob || fread(blob, 1, size, f) != (size_t)size) {
		fprintf(stderr, "cannot read %s\n", argv[1]);
		return 2;
	}
	fclose(f);

	for (c = 0; c < NCAPTURES; c++) {
		const struct capture *cap = &CAPTURES[c];
		u32 lo = (u32)(cap->lo * 65536.0);
		u32 hi = (u32)(cap->hi * 65536.0);
		int best = 42;
		u32 best_g = 0;
		u32 g;

		for (g = lo; g <= hi; g++) {
			u32 out[41];
			int bad = 0;
			int i;

			ar_isp_cfa_from_blob(out, blob, g);

			for (i = 0; i < 41; i++)
				if (out[i] != cap->reg[i])
					bad++;

			if (bad < best) {
				best = bad;
				best_g = g;
			}

			if (!bad)
				break;
		}

		printf("%-36s %2d/41 exact at abscissa %.6f%s\n", cap->name,
		       41 - best, best_g / 65536.0,
		       cap->gap ? "  [gap path]" : "  [verbatim path]");

		if (best) {
			u32 out[41];
			int i;

			ar_isp_cfa_from_blob(out, blob, best_g);
			for (i = 0; i < 41; i++)
				if (out[i] != cap->reg[i])
					printf("    word %2d predicted %u measured %u\n",
					       i, out[i], cap->reg[i]);
			rc = 1;
		}
	}

	printf("%s\n", rc ? "FAIL" : "ok: every captured cfa bank reproduced");

	return rc;
}
