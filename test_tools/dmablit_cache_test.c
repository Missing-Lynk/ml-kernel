// SPDX-License-Identifier: GPL-2.0
/*
 * dmablit_cache_test - prove ML_DMABLIT_CACHE does what the compositor's cross-fade needs, and
 * measure what it costs against the whole-buffer alternative.
 *
 * The seam cross-fade CPU-reads and CPU-writes ~90 KB of a 3 MB composite that the DMA engine
 * also writes. Correctness rests on two claims this checks directly, because a wrong answer
 * shows up as a subtly stale band on the panel rather than as an error:
 *
 *   1. a ranged INVALIDATE makes a DMA write visible to a CPU read that had the range cached,
 *   2. a ranged CLEAN makes a CPU write visible to a DMA read.
 *
 * It also times a 90 KB ranged invalidate against DMA_BUF_IOCTL_SYNC over the whole buffer,
 * which is the only alternative the dma-buf ABI offers and the reason this ioctl exists.
 *
 *   dmablit_cache_test [heap]        # default default_cma_region
 */
#include <linux/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "../modules/ml_dmablit.h"

struct dma_heap_allocation_data {
	__u64 len;
	__u32 fd;
	__u32 fd_flags;
	__u64 heap_flags;
};
#define DMA_HEAP_IOCTL_ALLOC _IOWR('H', 0x0, struct dma_heap_allocation_data)

struct dma_buf_sync { __u64 flags; };
#define DMA_BUF_SYNC_READ	(1 << 0)
#define DMA_BUF_SYNC_START	(0 << 2)
#define DMA_BUF_IOCTL_SYNC	_IOW('b', 0, struct dma_buf_sync)

/* The composite and its band, as the compositor sizes them. */
#define BUF_LEN		(1920 * 1088 * 3 / 2)
#define BAND_OFF	(528 * 1920)
#define BAND_LEN	(32 * 1920)
#define ITERS		200

static int fails;

static void check(int ok, const char *what)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
	if (!ok)
		fails++;
}

static int heap_alloc(const char *heap, size_t len)
{
	struct dma_heap_allocation_data a = { .len = len, .fd_flags = O_RDWR | O_CLOEXEC };
	char path[128];
	int hfd, ret;

	snprintf(path, sizeof(path), "/dev/dma_heap/%s", heap);
	hfd = open(path, O_RDWR | O_CLOEXEC);
	if (hfd < 0) {
		fprintf(stderr, "%s: %s\n", path, strerror(errno));
		return -1;
	}

	ret = ioctl(hfd, DMA_HEAP_IOCTL_ALLOC, &a);
	close(hfd);
	if (ret) {
		fprintf(stderr, "DMA_HEAP_IOCTL_ALLOC: %s\n", strerror(errno));
		return -1;
	}

	return a.fd;
}

static int cache_range(int blit, int fd, __u32 off, __u32 len, __u32 op)
{
	struct ml_dmablit_cache rq = { .fd = fd, .off = off, .len = len, .op = op };

	return ioctl(blit, ML_DMABLIT_CACHE, &rq);
}

/* One plane-sized copy, the shape the compositor submits. */
static int blit_copy(int blit, int dst_fd, int src_fd, __u32 off, __u32 len)
{
	struct ml_dmablit_req req = { .dst_fd = dst_fd, .n = 1 };

	req.copy[0] = (struct ml_dmablit_copy){ src_fd, off, off, len };

	return ioctl(blit, ML_DMABLIT_SUBMIT, &req);
}

static double now_us(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);

	return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
}

int main(int argc, char **argv)
{
	const char *heap = argc > 1 ? argv[1] : "default_cma_region";
	unsigned char *sm, *dm;
	int blit, src, dst, i;
	double t0, ranged, whole;

	blit = open("/dev/ml-dmablit", O_RDWR | O_CLOEXEC);
	if (blit < 0) {
		fprintf(stderr, "/dev/ml-dmablit: %s\n", strerror(errno));
		return 1;
	}

	src = heap_alloc(heap, BUF_LEN);
	dst = heap_alloc(heap, BUF_LEN);
	if (src < 0 || dst < 0)
		return 1;

	sm = mmap(NULL, BUF_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, src, 0);
	dm = mmap(NULL, BUF_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, dst, 0);
	if (sm == MAP_FAILED || dm == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	printf("dmablit_cache_test on %s, band %d bytes at %d\n", heap, BAND_LEN, BAND_OFF);

	/* The ioctl has to exist at all before any of the rest means anything. */
	if (cache_range(blit, dst, BAND_OFF, BAND_LEN, ML_DMABLIT_CLEAN)) {
		fprintf(stderr, "ML_DMABLIT_CACHE: %s (old ml_dmablit.ko?)\n", strerror(errno));
		return 1;
	}

	printf("\nargument checking\n");
	check(cache_range(blit, dst, BAND_OFF + 1, BAND_LEN, ML_DMABLIT_INVALIDATE) < 0 &&
	      errno == EINVAL, "unaligned invalidate offset is rejected");
	check(cache_range(blit, dst, BAND_OFF, BAND_LEN - 1, ML_DMABLIT_INVALIDATE) < 0 &&
	      errno == EINVAL, "unaligned invalidate length is rejected");
	check(cache_range(blit, dst, BUF_LEN - 64, 128, ML_DMABLIT_CLEAN) < 0 &&
	      errno == EINVAL, "a range past the end is rejected");
	check(cache_range(blit, dst, BAND_OFF, 0, ML_DMABLIT_CLEAN) < 0 &&
	      errno == EINVAL, "a zero length is rejected");
	check(cache_range(blit, dst, BAND_OFF, BAND_LEN, 99) < 0 &&
	      errno == EINVAL, "an unknown op is rejected");

	printf("\nclean: a CPU write reaches the DMA engine\n");
	memset(sm + BAND_OFF, 0xa5, BAND_LEN);
	memset(dm + BAND_OFF, 0x00, BAND_LEN);
	cache_range(blit, dst, BAND_OFF, BAND_LEN, ML_DMABLIT_CLEAN);
	check(cache_range(blit, src, BAND_OFF, BAND_LEN, ML_DMABLIT_CLEAN) == 0,
	      "clean of the source range succeeds");
	check(blit_copy(blit, dst, src, BAND_OFF, BAND_LEN) == 0, "DMA copy submits");
	check(cache_range(blit, dst, BAND_OFF, BAND_LEN, ML_DMABLIT_INVALIDATE) == 0,
	      "invalidate of the destination range succeeds");
	check(dm[BAND_OFF] == 0xa5 && dm[BAND_OFF + BAND_LEN - 1] == 0xa5,
	      "the DMA moved what the CPU wrote");

	printf("\ninvalidate: a DMA write reaches the CPU\n");
	/* Pull the destination band into the cache with a read the compiler cannot drop, so a
	 * missing invalidate would leave these lines stale and visible below.
	 */
	{
		volatile unsigned char sink = 0;

		for (i = 0; i < BAND_LEN; i += 64)
			sink ^= dm[BAND_OFF + i];

		(void)sink;
	}

	memset(sm + BAND_OFF, 0x5c, BAND_LEN);
	cache_range(blit, src, BAND_OFF, BAND_LEN, ML_DMABLIT_CLEAN);
	check(blit_copy(blit, dst, src, BAND_OFF, BAND_LEN) == 0, "second DMA copy submits");
	cache_range(blit, dst, BAND_OFF, BAND_LEN, ML_DMABLIT_INVALIDATE);
	check(dm[BAND_OFF] == 0x5c && dm[BAND_OFF + BAND_LEN / 2] == 0x5c &&
	      dm[BAND_OFF + BAND_LEN - 1] == 0x5c,
	      "the CPU sees the new DMA write, not cached bytes");

	printf("\nthe range is a range: bytes outside it are untouched\n");
	memset(dm, 0x11, BAND_OFF);
	cache_range(blit, dst, 0, BAND_OFF, ML_DMABLIT_CLEAN);
	cache_range(blit, dst, BAND_OFF, BAND_LEN, ML_DMABLIT_INVALIDATE);
	check(dm[0] == 0x11 && dm[BAND_OFF - 1] == 0x11,
	      "a clean CPU write below the band survives the band invalidate");

	printf("\ncost, mean of %d\n", ITERS);
	t0 = now_us();
	for (i = 0; i < ITERS; i++)
		cache_range(blit, dst, BAND_OFF, BAND_LEN, ML_DMABLIT_INVALIDATE);

	ranged = (now_us() - t0) / ITERS;

	{
		struct dma_buf_sync s = { .flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ };

		t0 = now_us();
		for (i = 0; i < ITERS; i++)
			ioctl(dst, DMA_BUF_IOCTL_SYNC, &s);

		whole = (now_us() - t0) / ITERS;
	}

	printf("  ranged invalidate, %d KiB : %7.1f us\n", BAND_LEN / 1024, ranged);
	printf("  whole-buffer sync, %d KiB : %7.1f us\n", BUF_LEN / 1024, whole);
	if (ranged > 0)
		printf("  ratio                      : %7.1fx\n", whole / ranged);

	printf("\n%s (%d failure(s))\n", fails ? "FAILED" : "OK", fails);

	return fails ? 1 : 0;
}
