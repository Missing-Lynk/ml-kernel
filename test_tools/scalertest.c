// SPDX-License-Identifier: GPL-2.0
/*
 * scalertest - on-device correctness test for the open ar_scaler (/dev/arscaler).
 *
 * The vendor's closed libhal_scaler is not present on the open stack, so this is
 * the self-contained oracle for the recovered driver. It allocates real MMZ
 * buffers (/dev/mmz_userdev), paints a known pattern into the source, drives
 * /dev/arscaler with SINGLE crop/resize ops, and checks the destination pixels.
 *
 * The test is an escalating ladder, strongest claim first:
 *
 *   T1  identity full-frame  (cropW==dstW, cropH==dstH, no offset)
 *   T2  identity cropped     (crop_x/crop_y offset, still 1:1)
 *        -> both must be BIT-EXACT: at scale ratio 1.0 the Q16 phase step is
 *           exactly one source pixel per output pixel (delta==0, phase 0), so a
 *           correct engine copies the cropped source verbatim. This needs NO
 *           interpolation model and is the real correctness gate: it proves the
 *           clock bring-up, completion IRQ, phys addressing, stride/crop maths
 *           and the byte-exact register packing are all right.
 *   T3  2:1 downscale        (cropW==2*dstW, cropH==2*dstH)
 *        -> compared against a software 2x2 box reference with a generous
 *           tolerance. We have no golden filter (the engine is a polyphase
 *           resampler with a 1024-byte LUT and an unknown sampling origin), so
 *           T3 is a plausibility check: it catches all-zero / garbage / wrong-
 *           layout output, but small filter/phase differences are tolerated.
 *
 * All tests use channels=1 (8-bit grayscale, 1 byte/pixel) so the reference is
 * unambiguous; stride is byte count rounded up to 16 (the ABI requirement).
 *
 * Run ON THE DEVICE (aarch64) after `../modules/load.sh` (ar_osal + ar_scaler):
 *   scalertest                 # default 256x128
 *   scalertest W H             # override source dims (W,H even)
 *
 * Exit 0 = PASS, 1 = setup/ioctl error, 2 = pixel mismatch (driver/HW fault).
 * On any failure, /proc/arscaler/state (dumped below) shows the programmed regs.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "../modules/ar_uapi.h"		/* IOC_MMB_* + struct mmb_info */

/* mirror of struct ar_scaler_op in ../modules/ar_scaler.c (64 bytes, all physical). */
struct scaler_op {
	unsigned int srcphy, srcw, srch, srcstride;
	unsigned int crop_x, crop_y, cropw, croph;
	unsigned int dstphy, dstw, dsth, dststride;
	unsigned int channels, control, interp, ctrl3c;
};
#define SCALER_IOC_SINGLE	_IOWR('Z', 1, struct scaler_op)	/* 0xc0405a01 */

#define ALIGN16(x)	(((x) + 15u) & ~15u)
#define T3_TOL_MEAN	24.0	/* mean abs 8-bit error allowed for the box-ref downscale */

/* An MMZ block mapped into this process (write-combine, via /dev/mmz_userdev). */
struct buf {
	unsigned long	phys;
	size_t		size;
	unsigned char  *va;
};

static int g_mmz = -1;	/* /dev/mmz_userdev */
static int g_sc  = -1;	/* /dev/arscaler   */

/* alloc size bytes from the anonymous MMZ zone and mmap it WC into our address
 * space (offset == phys, exactly like mmztest.c). Returns 0 on success.
 */
static int buf_alloc(struct buf *b, size_t size)
{
	struct mmb_info mi;

	memset(b, 0, sizeof(*b));
	memset(&mi, 0, sizeof(mi));
	mi.size = size;
	mi.align = 0x1000;
	strncpy(mi.mmb_name, "scalertest", sizeof(mi.mmb_name) - 1);
	if (ioctl(g_mmz, IOC_MMB_ALLOC, &mi)) {
		perror("IOC_MMB_ALLOC");
		return -1;
	}
	b->phys = (unsigned long)mi.phys_addr;
	b->size = size;
	b->va = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, g_mmz,
		     (off_t)b->phys);
	if (b->va == MAP_FAILED) {
		perror("mmap");
		ioctl(g_mmz, IOC_MMB_FREE, &mi);	/* best-effort */
		b->va = NULL;
		return -1;
	}
	return 0;
}

static void buf_free(struct buf *b)
{
	struct mmb_info mi;

	if (b->va) {
		munmap(b->va, b->size);
		b->va = NULL;
	}
	if (b->phys) {
		memset(&mi, 0, sizeof(mi));
		mi.phys_addr = b->phys;
		mi.size = b->size;
		if (ioctl(g_mmz, IOC_MMB_FREE, &mi))
			perror("IOC_MMB_FREE");
		b->phys = 0;
	}
}

/* deterministic, non-separable-looking grayscale pattern (gradients on both axes
 * plus a diagonal term so a transposed/mis-strided copy is visible).
 */
static unsigned char pat(unsigned int x, unsigned int y)
{
	return (unsigned char)((x * 7u + y * 13u + ((x ^ y) << 1)) & 0xffu);
}

static void fill_src(struct buf *s, unsigned int w, unsigned int h, unsigned int stride)
{
	unsigned int x, y;

	memset(s->va, 0, s->size);
	for (y = 0; y < h; y++)
		for (x = 0; x < w; x++)
			s->va[y * stride + x] = pat(x, y);
}

/* issue one SINGLE op and block; returns 0 or -1 (errno set / printed). */
static int run_op(const struct scaler_op *op)
{
	/* WC backing has no dirty cache lines, but order our fills before the DMA. */
	__sync_synchronize();
	ioctl(g_mmz, IOC_DCACHE_FLUSH_ALL, 0);	/* 'c',41 -> wmb() in-kernel */

	errno = 0;
	if (ioctl(g_sc, SCALER_IOC_SINGLE, op)) {
		fprintf(stderr, "  SCALER_IOC_SINGLE failed errno=%d (%s)\n",
			errno, strerror(errno));
		if (errno == ETIMEDOUT)
			fprintf(stderr, "  (-ETIMEDOUT: completion IRQ never fired - "
					"clock seq or register packing wrong)\n");
		return -1;
	}
	__sync_synchronize();		/* order the engine's writes before our reads */
	return 0;
}

/* Compare a 1:1 (identity) copy: dst[y][x] must equal src[(sy0+y)][(sx0+x)].
 * Returns mismatch count; prints the first mismatch.
 */
static long verify_identity(const struct buf *s, unsigned int sstride,
			    unsigned int sx0, unsigned int sy0,
			    const struct buf *d, unsigned int dstride,
			    unsigned int w, unsigned int h)
{
	unsigned int x, y;
	long bad = 0;

	for (y = 0; y < h; y++) {
		for (x = 0; x < w; x++) {
			unsigned char got = d->va[y * dstride + x];
			unsigned char exp = s->va[(sy0 + y) * sstride + (sx0 + x)];

			if (got != exp) {
				if (!bad)
					fprintf(stderr,
						"  first mismatch @ (%u,%u): got 0x%02x exp 0x%02x\n",
						x, y, got, exp);
				bad++;
			}
		}
	}
	return bad;
}

/* Software 2x2 box reference for the 2:1 downscale; returns mean abs error and
 * sets *maxerr. dst is dw x dh; source region is the full 2dw x 2dh crop.
 */
static double verify_box_2x(const struct buf *s, unsigned int sstride,
			    const struct buf *d, unsigned int dstride,
			    unsigned int dw, unsigned int dh,
			    unsigned int *maxerr, int *has_structure)
{
	unsigned int x, y;
	double acc = 0.0;
	unsigned int mn = 255, mx = 0;

	*maxerr = 0;
	for (y = 0; y < dh; y++) {
		for (x = 0; x < dw; x++) {
			unsigned int sx = x * 2, sy = y * 2;
			unsigned int ref = (s->va[sy * sstride + sx] +
					    s->va[sy * sstride + sx + 1] +
					    s->va[(sy + 1) * sstride + sx] +
					    s->va[(sy + 1) * sstride + sx + 1] + 2) / 4;
			unsigned int got = d->va[y * dstride + x];
			unsigned int e = got > ref ? got - ref : ref - got;

			acc += e;
			if (e > *maxerr)
				*maxerr = e;
			if (got < mn) mn = got;
			if (got > mx) mx = got;
		}
	}
	*has_structure = (mx - mn) > 16;	/* not a flat/blank frame */
	return acc / ((double)dw * dh);
}

/* Print the on-device oracle so a failure is diagnosable from the log alone. */
static void dump_state(void)
{
	char line[256];
	FILE *f = fopen("/proc/arscaler/state", "r");

	if (!f) {
		fprintf(stderr, "  (/proc/arscaler/state unreadable: %s)\n", strerror(errno));
		return;
	}
	fprintf(stderr, "  --- /proc/arscaler/state ---\n");
	while (fgets(line, sizeof(line), f))
		fprintf(stderr, "  %s", line);
	fclose(f);
}

int main(int argc, char **argv)
{
	unsigned int W = 256, H = 128;
	unsigned int sstride, dw, dh, dstride;
	struct buf src, dst;
	struct scaler_op op;
	long bad;
	unsigned int maxerr;
	int has_struct, fail = 0;

	if (argc >= 3) {
		W = (unsigned int)strtoul(argv[1], NULL, 0) & ~1u;
		H = (unsigned int)strtoul(argv[2], NULL, 0) & ~1u;
		if (W < 4 || H < 4) {
			fprintf(stderr, "W/H too small\n");
			return 1;
		}
	}
	sstride = ALIGN16(W);

	g_mmz = open("/dev/mmz_userdev", O_RDWR);
	if (g_mmz < 0) { perror("open /dev/mmz_userdev"); return 1; }
	g_sc = open("/dev/arscaler", O_RDWR);
	if (g_sc < 0) { perror("open /dev/arscaler"); close(g_mmz); return 1; }

	/* One source (WxH) + one dst big enough for the largest test (identity WxH). */
	if (buf_alloc(&src, (size_t)sstride * H))
		return 1;
	if (buf_alloc(&dst, (size_t)sstride * H)) {
		buf_free(&src);
		return 1;
	}
	printf("src phys=0x%lx dst phys=0x%lx  base %ux%u stride %u (channels=1)\n",
	       src.phys, dst.phys, W, H, sstride);

	fill_src(&src, W, H, sstride);

	/* ---- T1: identity full-frame (bit-exact) ---------------------------- */
	memset(&op, 0, sizeof(op));
	op.srcphy = src.phys; op.srcw = W; op.srch = H; op.srcstride = sstride;
	op.crop_x = 0; op.crop_y = 0; op.cropw = W; op.croph = H;
	op.dstphy = dst.phys; op.dstw = W; op.dsth = H; op.dststride = sstride;
	op.channels = 1;
	memset(dst.va, 0xAA, dst.size);		/* sentinel: catch "engine wrote nothing" */
	printf("T1 identity full-frame %ux%u ... ", W, H); fflush(stdout);
	if (run_op(&op)) { fail = 1; dump_state(); goto done; }
	bad = verify_identity(&src, sstride, 0, 0, &dst, sstride, W, H);
	if (bad) { printf("FAIL (%ld px differ)\n", bad); fail = 2; dump_state(); }
	else printf("PASS (bit-exact)\n");

	/* ---- T2: identity cropped sub-rect at 1:1 (bit-exact) --------------- */
	{
		unsigned int cx = 16, cy = 8, cw = W / 2, ch = H / 2;

		if (cx + cw > W) cw = W - cx;
		if (cy + ch > H) ch = H - cy;
		dstride = ALIGN16(cw);
		memset(&op, 0, sizeof(op));
		op.srcphy = src.phys; op.srcw = W; op.srch = H; op.srcstride = sstride;
		op.crop_x = cx; op.crop_y = cy; op.cropw = cw; op.croph = ch;
		op.dstphy = dst.phys; op.dstw = cw; op.dsth = ch; op.dststride = dstride;
		op.channels = 1;
		memset(dst.va, 0xAA, dst.size);
		printf("T2 identity crop @%u,%u %ux%u ... ", cx, cy, cw, ch); fflush(stdout);
		if (run_op(&op)) { fail = 1; dump_state(); goto done; }
		bad = verify_identity(&src, sstride, cx, cy, &dst, dstride, cw, ch);
		if (bad) { printf("FAIL (%ld px differ)\n", bad); fail = 2; dump_state(); }
		else printf("PASS (bit-exact)\n");
	}

	/* ---- T3: 2:1 downscale vs software box reference (tolerance) -------- */
	{
		double mean;

		dw = W / 2; dh = H / 2; dstride = ALIGN16(dw);
		memset(&op, 0, sizeof(op));
		op.srcphy = src.phys; op.srcw = W; op.srch = H; op.srcstride = sstride;
		op.crop_x = 0; op.crop_y = 0; op.cropw = W; op.croph = H;
		op.dstphy = dst.phys; op.dstw = dw; op.dsth = dh; op.dststride = dstride;
		op.channels = 1;
		memset(dst.va, 0xAA, dst.size);
		printf("T3 downscale %ux%u -> %ux%u ... ", W, H, dw, dh); fflush(stdout);
		if (run_op(&op)) { fail = 1; dump_state(); goto done; }
		mean = verify_box_2x(&src, sstride, &dst, dstride, dw, dh, &maxerr, &has_struct);
		printf("mean|err|=%.2f max=%u %s\n", mean, maxerr,
		       has_struct ? "" : "(FLAT!)");
		if (!has_struct || mean > T3_TOL_MEAN) {
			printf("T3 FAIL (blank or > box-ref tolerance %.0f)\n", T3_TOL_MEAN);
			fail = fail ? fail : 2;
			dump_state();
		} else {
			printf("T3 PASS (plausible downscale, within box-ref tolerance)\n");
		}
	}

done:
	buf_free(&dst);
	buf_free(&src);
	close(g_sc);
	close(g_mmz);
	printf("\n%s\n", fail ? "FAIL" : "PASS");
	return fail;
}
