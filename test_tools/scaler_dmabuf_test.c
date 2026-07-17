// SPDX-License-Identifier: GPL-2.0
/*
 * scaler_dmabuf_test - on-device test for ar_scaler's dmabuf ABI family
 * (SCALER_IOC_SINGLE_DMABUF / SCALER_IOC_BATCH_DMABUF).
 *
 * Where scalertest.c proves the vendor phys-by-value path on MMZ buffers, this
 * proves the open fd path on CMA dma-buf-heap buffers - the pipeline's world:
 * the DVR record branch hands the scaler its composite frame and a downscale
 * pool buffer as dmabuf fds, three planes (Y/U/V) per batch.
 *
 * Ladder (strongest claim first), all on an I420-layout buffer:
 *
 *   T1  single-op identity on the Y plane          -> BIT-EXACT
 *   T2  3-plane batch identity (Y,U,V, offsets)    -> BIT-EXACT per plane
 *        -> proves fd resolution, per-plane offset addressing, and the batch
 *           descriptor block against the CMA heap; at ratio 1.0 a correct
 *           engine copies verbatim (no interpolation model needed).
 *   T3  3-plane batch 2:1 downscale                -> 2x2 box ref, tolerance
 *   T4  (iters > 0) 3-plane 2:3 downscale timing   -> the real DVR shape
 *        (1920x1080 -> 1280x720 when run as `scaler_dmabuf_test 1920 1080 N`);
 *        prints ms/frame + fps ceiling, output checked non-flat only (no box
 *        ref at a fractional ratio).
 *
 * CPU access to the buffers is bracketed with DMA_BUF_IOCTL_SYNC: the kernel
 * caches the scaler's mapping per open fd, so the heap's begin/end_cpu_access
 * sync over live attachments is what keeps caches coherent after the first op.
 *
 * Run ON THE DEVICE (aarch64) with ar_scaler.ko loaded (standalone);
 * needs CONFIG_DMABUF_HEAPS_CMA (configs/display.config).
 *
 *   scaler_dmabuf_test               # default 256x128
 *   scaler_dmabuf_test W H           # override dims (W,H multiples of 4)
 *   scaler_dmabuf_test W H ITERS     # ...then time ITERS 2:3-downscale frames
 *
 * Exit 0 = PASS, 1 = setup/ioctl error, 2 = pixel mismatch (driver/HW fault).
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>

#include "../modules/ar_scaler.h"

/* dma-buf heap alloc + CPU-access sync UAPI, defined locally so the build does
 * not depend on the host kernel headers (matches ml_dmablit_test.c).
 */
struct dma_heap_allocation_data {
	uint64_t len;
	uint32_t fd;
	uint32_t fd_flags;
	uint64_t heap_flags;
};
#define DMA_HEAP_IOCTL_ALLOC	_IOWR('H', 0x0, struct dma_heap_allocation_data)

struct dma_buf_sync {
	uint64_t flags;
};
#define DMA_BUF_SYNC_READ	(1 << 0)
#define DMA_BUF_SYNC_WRITE	(2 << 0)
#define DMA_BUF_SYNC_START	(0 << 2)
#define DMA_BUF_SYNC_END	(1 << 2)
#define DMA_BUF_IOCTL_SYNC	_IOW('b', 0, struct dma_buf_sync)

#define ALIGN16(x)	(((x) + 15u) & ~15u)
#define T3_TOL_MEAN	24.0	/* mean abs 8-bit error allowed vs the 2x2 box ref */

/* One I420 image inside one contiguous dma-buf: fd + mmap + plane layout. */
struct img {
	int		fd;
	uint8_t	       *va;
	size_t		size;
	unsigned int	w, h;
	unsigned int	ystride, cstride;
	unsigned int	off[3];		/* Y, U, V byte offsets */
};

static int g_sc = -1;	/* /dev/arscaler */

/* Open the first usable /dev/dma_heap device and allocate a len-byte contiguous dma-buf. */
static int heap_alloc(size_t len)
{
	struct dma_heap_allocation_data alloc = { .len = len, .fd_flags = O_RDWR | O_CLOEXEC };
	char path[280];
	struct dirent *de;
	DIR *d = opendir("/dev/dma_heap");
	int hfd = -1;

	if (!d) {
		perror("/dev/dma_heap (CONFIG_DMABUF_HEAPS_CMA needed)");
		return -1;
	}

	/* Prefer a CMA heap (the pipeline's world; readdir order would happily
	 * hand us the mmz heap), falling back to whatever opens.
	 */
	for (int want_cma = 1; want_cma >= 0 && hfd < 0; want_cma--) {
		rewinddir(d);
		while ((de = readdir(d))) {
			if (de->d_name[0] == '.')
				continue;

			if (want_cma && !strstr(de->d_name, "cma"))
				continue;

			snprintf(path, sizeof(path), "/dev/dma_heap/%s", de->d_name);
			hfd = open(path, O_RDWR | O_CLOEXEC);
			if (hfd >= 0) {
				break;
			}
		}
	}
	closedir(d);

	if (hfd < 0) {
		fprintf(stderr, "no usable heap under /dev/dma_heap\n");
		return -1;
	}

	if (ioctl(hfd, DMA_HEAP_IOCTL_ALLOC, &alloc)) {
		perror("DMA_HEAP_IOCTL_ALLOC");
		close(hfd);
		return -1;
	}

	close(hfd);
	return (int)alloc.fd;
}

static void sync_buf(int fd, uint64_t rw, int start)
{
	struct dma_buf_sync s = { .flags = rw | (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END) };

	ioctl(fd, DMA_BUF_IOCTL_SYNC, &s);
}

/* Allocate + mmap one I420 image for w x h (both even). */
static int img_alloc(struct img *im, unsigned int w, unsigned int h)
{
	memset(im, 0, sizeof(*im));
	im->w = w;
	im->h = h;
	im->ystride = ALIGN16(w);
	im->cstride = ALIGN16(w / 2);
	im->off[0] = 0;
	im->off[1] = im->ystride * h;
	im->off[2] = im->off[1] + im->cstride * (h / 2);
	im->size = im->off[2] + (size_t)im->cstride * (h / 2);

	im->fd = heap_alloc(im->size);
	if (im->fd < 0) {
		return -1;
	}

	im->va = mmap(NULL, im->size, PROT_READ | PROT_WRITE, MAP_SHARED, im->fd, 0);
	if (im->va == MAP_FAILED) {
		perror("mmap");
		close(im->fd);
		im->va = NULL;
		return -1;
	}

	return 0;
}

static void img_free(struct img *im)
{
	if (im->va) {
		munmap(im->va, im->size);
	}

	if (im->fd >= 0) {
		close(im->fd);
	}
}

/* plane p dims: Y = w x h, U/V = w/2 x h/2 */
static unsigned int pw(const struct img *im, int p) { return p ? im->w / 2 : im->w; }
static unsigned int ph(const struct img *im, int p) { return p ? im->h / 2 : im->h; }
static unsigned int pstride(const struct img *im, int p) { return p ? im->cstride : im->ystride; }

/* deterministic grayscale pattern (gradients + diagonal term so transposed or
 * mis-strided output is visible); each plane offset by 31 so a plane swap shows.
 */
static uint8_t pat(unsigned int x, unsigned int y, int p)
{
	return (uint8_t)((x * 7u + y * 13u + ((x ^ y) << 1) + (unsigned int)p * 31u) & 0xffu);
}

static void fill_src(struct img *im)
{
	sync_buf(im->fd, DMA_BUF_SYNC_WRITE, 1);
	for (int p = 0; p < 3; p++) {
		for (unsigned int y = 0; y < ph(im, p); y++) {
			for (unsigned int x = 0; x < pw(im, p); x++) {
				im->va[im->off[p] + y * pstride(im, p) + x] = pat(x, y, p);
			}
		}
	}
	sync_buf(im->fd, DMA_BUF_SYNC_WRITE, 0);
}

static void poison_dst(struct img *im)
{
	sync_buf(im->fd, DMA_BUF_SYNC_WRITE, 1);
	memset(im->va, 0xAA, im->size);
	sync_buf(im->fd, DMA_BUF_SYNC_WRITE, 0);
}

/* Build one plane op: plane p of src (full plane crop) -> dw x dh at plane p of dst. */
static void plane_op(struct ar_scaler_dmabuf_op *dop,
		     const struct img *s, const struct img *d, int p,
		     unsigned int dw, unsigned int dh)
{
	memset(dop, 0, sizeof(*dop));
	dop->src_fd = s->fd;
	dop->dst_fd = d->fd;
	dop->src_off = s->off[p];
	dop->dst_off = d->off[p];
	dop->op.srcw = pw(s, p);
	dop->op.srch = ph(s, p);
	dop->op.srcstride = pstride(s, p);
	dop->op.cropw = pw(s, p);
	dop->op.croph = ph(s, p);
	dop->op.dstw = dw;
	dop->op.dsth = dh;
	dop->op.dststride = pstride(d, p);
	dop->op.channels = 1;
}

/* Print the on-device oracle so a failure is diagnosable from the log alone. */
static void dump_state(void)
{
	char line[256];
	FILE *f = fopen("/proc/arscaler/state", "r");

	if (!f) {
		return;
	}

	fprintf(stderr, "  --- /proc/arscaler/state ---\n");
	while (fgets(line, sizeof(line), f)) {
		fprintf(stderr, "  %s", line);
	}

	fclose(f);
}

static int run_ioctl(unsigned long cmd, void *arg, const char *what)
{
	errno = 0;
	if (ioctl(g_sc, cmd, arg)) {
		fprintf(stderr, "  %s failed errno=%d (%s)\n", what, errno, strerror(errno));
		if (errno == ETIMEDOUT) {
			fprintf(stderr, "  (-ETIMEDOUT: completion IRQ never fired)\n");
		}

		dump_state();
		return -1;
	}

	return 0;
}

/* Bit-exact identity compare of plane p: dst must equal src verbatim. */
static long verify_plane_identity(const struct img *s, const struct img *d, int p)
{
	long bad = 0;

	for (unsigned int y = 0; y < ph(s, p); y++) {
		for (unsigned int x = 0; x < pw(s, p); x++) {
			uint8_t got = d->va[d->off[p] + y * pstride(d, p) + x];
			uint8_t exp = s->va[s->off[p] + y * pstride(s, p) + x];

			if (got != exp) {
				if (!bad) {
					fprintf(stderr,
						"  plane %d first mismatch @ (%u,%u): got 0x%02x exp 0x%02x\n",
						p, x, y, got, exp);
				}
				bad++;
			}
		}
	}

	return bad;
}

/* 2x2 box reference for the 2:1 downscale of plane p; mean abs error + structure flag. */
static double verify_plane_box2x(const struct img *s, const struct img *d, int p,
				 unsigned int *maxerr, int *has_structure)
{
	unsigned int dw = pw(s, p) / 2, dh = ph(s, p) / 2;
	unsigned int sstride = pstride(s, p), dstride = pstride(d, p);
	const uint8_t *sp = s->va + s->off[p];
	const uint8_t *dp = d->va + d->off[p];
	unsigned int mn = 255, mx = 0;
	double acc = 0.0;

	*maxerr = 0;
	for (unsigned int y = 0; y < dh; y++) {
		for (unsigned int x = 0; x < dw; x++) {
			unsigned int sx = x * 2, sy = y * 2;
			unsigned int ref = (sp[sy * sstride + sx] +
					    sp[sy * sstride + sx + 1] +
					    sp[(sy + 1) * sstride + sx] +
					    sp[(sy + 1) * sstride + sx + 1] + 2) / 4;
			unsigned int got = dp[y * dstride + x];
			unsigned int e = got > ref ? got - ref : ref - got;

			acc += e;
			if (e > *maxerr) {
				*maxerr = e;
			}

			if (got < mn) {
				mn = got;
			}

			if (got > mx) {
				mx = got;
			}
		}
	}

	*has_structure = (mx - mn) > 16;
	return acc / ((double)dw * dh);
}

/* Fill a 3-plane batch: every plane of src full-crop into the same plane of dst. */
static void batch_ops(struct ar_scaler_dmabuf_batch *batch,
		      const struct img *s, const struct img *d)
{
	memset(batch, 0, sizeof(*batch));
	batch->count = 3;
	for (int p = 0; p < 3; p++)
		plane_op(&batch->ops[p], s, d, p, pw(d, p), ph(d, p));
}

/* Each rung returns 0 = pass, 1 = ioctl error (abort the ladder), 2 = pixel mismatch. */

/* T1: single-op Y-plane identity (bit-exact). */
static int t1_single_identity(const struct img *src, struct img *dst)
{
	struct ar_scaler_dmabuf_op dop;
	long bad;

	poison_dst(dst);
	plane_op(&dop, src, dst, 0, src->w, src->h);
	printf("T1 single dmabuf identity Y %ux%u ... ", src->w, src->h);
	fflush(stdout);
	if (run_ioctl(SCALER_IOC_SINGLE_DMABUF, &dop, "SCALER_IOC_SINGLE_DMABUF"))
		return 1;

	sync_buf(dst->fd, DMA_BUF_SYNC_READ, 1);
	bad = verify_plane_identity(src, dst, 0);
	sync_buf(dst->fd, DMA_BUF_SYNC_READ, 0);
	if (bad) {
		printf("FAIL (%ld px differ)\n", bad);
		dump_state();
		return 2;
	}

	printf("PASS (bit-exact)\n");
	return 0;
}

/* T2: 3-plane batch identity (bit-exact per plane). */
static int t2_batch_identity(const struct img *src, struct img *dst)
{
	struct ar_scaler_dmabuf_batch batch;
	long bad = 0;

	poison_dst(dst);
	batch_ops(&batch, src, dst);
	printf("T2 batch identity Y+U+V ... ");
	fflush(stdout);
	if (run_ioctl(SCALER_IOC_BATCH_DMABUF, &batch, "SCALER_IOC_BATCH_DMABUF"))
		return 1;

	sync_buf(dst->fd, DMA_BUF_SYNC_READ, 1);
	for (int p = 0; p < 3; p++)
		bad += verify_plane_identity(src, dst, p);
	sync_buf(dst->fd, DMA_BUF_SYNC_READ, 0);
	if (bad) {
		printf("FAIL (%ld px differ)\n", bad);
		dump_state();
		return 2;
	}

	printf("PASS (bit-exact, all planes)\n");
	return 0;
}

/* T3: 3-plane batch 2:1 downscale vs the 2x2 box reference (tolerance). */
static int t3_batch_downscale(const struct img *src, struct img *half)
{
	struct ar_scaler_dmabuf_batch batch;
	int bad = 0;

	poison_dst(half);
	batch_ops(&batch, src, half);
	printf("T3 batch 2:1 downscale -> %ux%u ... ", half->w, half->h);
	fflush(stdout);
	if (run_ioctl(SCALER_IOC_BATCH_DMABUF, &batch, "SCALER_IOC_BATCH_DMABUF"))
		return 1;

	sync_buf(half->fd, DMA_BUF_SYNC_READ, 1);
	for (int p = 0; p < 3; p++) {
		unsigned int maxerr;
		int has_struct;
		double mean = verify_plane_box2x(src, half, p, &maxerr, &has_struct);

		printf("[p%d mean|err|=%.2f max=%u%s] ", p, mean, maxerr,
		       has_struct ? "" : " FLAT!");
		if (!has_struct || mean > T3_TOL_MEAN)
			bad = 1;
	}
	sync_buf(half->fd, DMA_BUF_SYNC_READ, 0);
	if (bad) {
		printf("FAIL (blank or > box-ref tolerance %.0f)\n", T3_TOL_MEAN);
		dump_state();
		return 2;
	}

	printf("PASS\n");
	return 0;
}

/* T4: 2:3 downscale timing (the DVR 1080p -> 720p shape); output checked
 * non-flat only (a fractional ratio has no simple software reference).
 */
static int t4_timing(const struct img *src, struct img *third, int iters)
{
	struct ar_scaler_dmabuf_batch batch;
	struct timespec t0, t1;
	unsigned int mn = 255, mx = 0;
	double secs, per_ms;

	batch_ops(&batch, src, third);
	printf("T4 timing %d x 3-plane %ux%u -> %ux%u ... ",
	       iters, src->w, src->h, third->w, third->h);
	fflush(stdout);

	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (int it = 0; it < iters; it++)
		if (run_ioctl(SCALER_IOC_BATCH_DMABUF, &batch, "T4 batch"))
			return 1;
	clock_gettime(CLOCK_MONOTONIC, &t1);

	secs = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
	per_ms = secs * 1e3 / iters;
	printf("%.2f ms/frame (%.1f fps ceiling)\n", per_ms, 1e3 / per_ms);

	sync_buf(third->fd, DMA_BUF_SYNC_READ, 1);
	for (unsigned int y = 0; y < third->h; y++) {
		for (unsigned int x = 0; x < third->w; x++) {
			uint8_t v = third->va[y * third->ystride + x];

			if (v < mn)
				mn = v;
			if (v > mx)
				mx = v;
		}
	}
	sync_buf(third->fd, DMA_BUF_SYNC_READ, 0);
	if (mx - mn <= 16) {
		printf("T4 FAIL (flat output)\n");
		dump_state();
		return 2;
	}

	return 0;
}

int main(int argc, char **argv)
{
	unsigned int W = 256, H = 128;
	int iters = 0, fail, r;
	struct img src, dst, half, third;

	if (argc >= 3) {
		W = (unsigned int)strtoul(argv[1], NULL, 0) & ~3u;
		H = (unsigned int)strtoul(argv[2], NULL, 0) & ~3u;
		if (W < 8 || H < 8) {
			fprintf(stderr, "W/H too small\n");
			return 1;
		}
	}

	if (argc >= 4)
		iters = atoi(argv[3]);

	g_sc = open("/dev/arscaler", O_RDWR);
	if (g_sc < 0) {
		perror("open /dev/arscaler (load.sh run?)");
		return 1;
	}

	/* src + identity dst at W x H, plus a half-size and a 2/3-size dst */
	if (img_alloc(&src, W, H))
		return 1;

	if (img_alloc(&dst, W, H))
		return 1;

	if (img_alloc(&half, W / 2, H / 2))
		return 1;

	if (img_alloc(&third, (W * 2 / 3) & ~3u, (H * 2 / 3) & ~3u))
		return 1;

	printf("I420 %ux%u (ystride %u cstride %u, %zu B/buf) via dmabuf fds\n",
	       W, H, src.ystride, src.cstride, src.size);

	fill_src(&src);

	/* The ladder: an ioctl error (1) aborts; a mismatch (2) keeps going so
	 * one run reports every broken rung. The first failure sets the exit code.
	 */
	r = t1_single_identity(&src, &dst);
	fail = r;
	if (r != 1) {
		r = t2_batch_identity(&src, &dst);
		fail = fail ? fail : r;
	}

	if (r != 1) {
		r = t3_batch_downscale(&src, &half);
		fail = fail ? fail : r;
	}

	if (r != 1 && iters > 0) {
		r = t4_timing(&src, &third, iters);
		fail = fail ? fail : r;
	}

	img_free(&third);
	img_free(&half);
	img_free(&dst);
	img_free(&src);
	close(g_sc);
	printf("\n%s\n", fail ? "FAIL" : "PASS");

	return fail;
}
