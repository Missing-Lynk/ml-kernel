// SPDX-License-Identifier: GPL-2.0
/*
 * de3d-strength-test.c - prove the integer de3d strength knob against the vendor's double form.
 *
 * The kernel has no FPU, so ar_isp_de3d_strength() computes the vendor's
 * double-precision transform (libmpp_service.so 0x1c69f4) in integers. This
 * checks the two forms agree bit-for-bit over both fields and every strength,
 * and that 50 is exactly identity. The vendor form here is the reference read
 * off the instruction stream; it is not compiled into the driver.
 *
 * Usage: de3d-strength-test   (no arguments, no device)
 */
#include <stdint.h>
#include <stdio.h>

typedef uint32_t u32;
typedef int64_t s64;

/* The shipped integer form, copied from ar-isp-de3d.h. */
static u32 knob(u32 v, int strength, u32 ceil)
{
	s64 t;

	if (strength == 50)
		return v;

	if (strength < 50)
		t = (s64)strength * v;
	else
		t = 50 * (s64)ceil - (100 - strength) * ((s64)ceil - v);

	return (u32)(t / 50);
}

/* The vendor reference: double precision, fcvtzs = truncate toward zero. */
static u32 vendor(u32 v, int p, double c)
{
	double out;

	if (p <= 50)
		out = (double)v - (50 - p) * (double)v / 50.0;
	else
		out = c - (100 - p) * (c - (double)v) / 50.0;

	return (u32)out;
}

static int sweep(const char *name, u32 vmax, u32 vstep, u32 mask, double ceil)
{
	unsigned int fails = 0;

	for (int p = 0; p <= 100; p++)
		for (u32 v = 0; v <= vmax; v += vstep) {
			u32 a = knob(v, p, (u32)ceil) & mask;
			u32 b = vendor(v, p, ceil) & mask;

			if (a != b) {
				if (fails < 6)
					printf("  %s MISMATCH v=%u p=%d int=%u vendor=%u\n",
					       name, v, p, a, b);
				fails++;
			}
		}

	printf("%s: %s\n", name, fails ? "FAIL" : "ok");

	return fails ? 1 : 0;
}

int main(void)
{
	int bad = 0;

	bad |= sweep("14-bit (ceil 200)", 0x3fff, 1, 0x3fff, 200.0);
	bad |= sweep("9-bit (ceil 120)", 0x1ff, 1, 0x1ff, 120.0);

	for (u32 v = 0; v <= 0x3fff; v++)
		if (knob(v, 50, 200) != v) {
			printf("  identity FAIL at v=%u\n", v);
			bad = 1;
			break;
		}

	if (!bad)
		printf("de3d-strength OK\n");

	return bad;
}
