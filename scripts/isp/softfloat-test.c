// SPDX-License-Identifier: GPL-2.0
/*
 * softfloat-test.c - drive ar-isp-softfloat.h against the host FPU.
 *
 * The header exists because the kernel has no FPU and the vendor's ladder
 * blend is float. The host does have one, so the oracle here is the hardware
 * itself: every routine is compared against the native float operation,
 * including fmaf for the fused step, over the domain the ladders use.
 *
 * Build and run directly:
 *   cc -O2 -Wall -I../../overlay/drivers/media/artosyn -o /tmp/sft softfloat-test.c -lm && /tmp/sft
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef uint32_t u32;
typedef uint64_t u64;

#define U32_MAX 0xffffffffu
#define S32_MAX 0x7fffffff
#define S32_MIN (-0x7fffffff - 1)
typedef int32_t s32;
typedef int64_t s64;

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

static float bits_to_f(u32 b)
{
	float f;

	memcpy(&f, &b, sizeof(f));
	return f;
}

static u32 f_to_bits(float f)
{
	u32 b;

	memcpy(&b, &f, sizeof(b));
	return b;
}

static unsigned long checked, failed;

static void check(const char *op, u32 got, u32 want, float a, float b, float c)
{
	checked++;

	if (got == want)
		return;

	if (failed++ < 12) {
		printf("  %s mismatch: a=%.9g b=%.9g c=%.9g got %#010x (%.9g) want %#010x (%.9g)\n",
		       op, a, b, c, got, bits_to_f(got), want, bits_to_f(want));
	}
}

/* Record words and band edges the ladders actually carry. */
static const s32 VALUES0[] = {
	0, 1, 2, 3, 6, 10, 13, 26, 29, 30, 31, 40, 50, 64, 70, 74, 75, 100,
	102, 115, 120, 122, 128, 179, 200, 255, 256, 264, 500, 550, 1000,
	1024, 2000, 5616, 8000, 10531, 15000, 16328, 65535, 900000, 3000000,
};
#define NV0 (sizeof(VALUES0) / sizeof(VALUES0[0]))
/* lnr carries 298 negative record words, so both signs must be covered. */
static s32 VALUES[2 * NV0];
#define NVALUES (2 * NV0)
static void build_values(void)
{
	unsigned int i;

	for (i = 0; i < NV0; i++) {
		VALUES[2 * i] = VALUES0[i];
		VALUES[2 * i + 1] = -VALUES0[i];
	}
}

int main(void)
{
	unsigned int i, j, k;

	build_values();

	/* Conversion. */
	for (i = 0; i < NVALUES; i++) {
		s32 v = VALUES[i];

		check("from_s32", ar_f32_from_s32(v), f_to_bits((float)v),
		      (float)v, 0, 0);
	}

	/* The abscissa conversion: gain_q16 scaled by 2^-16. */
	for (i = 1; i < (1u << 23); i += 9973) {
		ar_f32 got = ar_f32_scale_down(ar_f32_from_s32(i), 16);

		check("scale_down", got, f_to_bits((float)i / 65536.0f),
		      (float)i, 0, 0);
	}

	/* mul, sub, div, fma over the ladder domain. */
	for (i = 0; i < NVALUES; i++) {
		for (j = 0; j < NVALUES; j++) {
			float fa = (float)VALUES[i];
			float fb = (float)VALUES[j];
			ar_f32 a = ar_f32_from_s32(VALUES[i]);
			ar_f32 b = ar_f32_from_s32(VALUES[j]);

			check("mul", ar_f32_mul(a, b), f_to_bits(fa * fb),
			      fa, fb, 0);

			check("sub", ar_f32_sub(a, b),
				      f_to_bits(fa - fb), fa, fb, 0);

			if (VALUES[j])
				check("div", ar_f32_div(a, b),
				      f_to_bits(fa / fb), fa, fb, 0);
		}
	}

	/*
	 * The blend itself: t across the unit interval against every pair of
	 * record words, which is exactly the shape isp_sub_cfa evaluates.
	 */
	for (k = 0; k <= 4096; k++) {
		float ft = (float)k / 4096.0f;
		ar_f32 t = ar_f32_div(ar_f32_from_s32(k),
				      ar_f32_from_s32(4096));
		ar_f32 omt = ar_f32_sub(ar_f32_from_s32(1), t);
		float fomt = 1.0f - ft;

		check("t", t, f_to_bits(ft), ft, 0, 0);
		check("1-t", omt, f_to_bits(fomt), ft, 0, 0);

		for (i = 0; i < NVALUES; i++) {
			for (j = 0; j < NVALUES; j++) {
				float lo = (float)VALUES[i];
				float hi = (float)VALUES[j];
				ar_f32 flo = ar_f32_from_s32(VALUES[i]);
				ar_f32 fhi = ar_f32_from_s32(VALUES[j]);
				ar_f32 up = ar_f32_mul(t, fhi);
				ar_f32 got = ar_f32_fma(omt, flo, up);
				float want = fmaf(fomt, lo, ft * hi);

				check("fma", got, f_to_bits(want), fomt, lo,
				      ft * hi);
				check("trunc",
				      (u32)ar_f32_to_s32_trunc(got),
				      (u32)(s32)want, fomt, lo, ft * hi);
			}
		}
	}

	printf("%s: %lu comparisons, %lu mismatches\n",
	       failed ? "FAIL" : "ok", checked, failed);

	return failed ? 1 : 0;
}
