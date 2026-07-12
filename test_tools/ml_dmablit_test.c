// SPDX-License-Identifier: GPL-2.0
/*
 * ml_dmablit_test - on-device bring-up test for /dev/ml-dmablit (ml_dmablit.ko).
 *
 * Drives the device_prep_dma_memcpy path through the ML_DMABLIT_SUBMIT ioctl
 * (the dmaengine path userspace compositors use). Allocates two contiguous
 * dma-buf-heap (CMA) buffers - one
 * source, one destination - fills the source with a known pattern, submits a batch
 * of copies through ML_DMABLIT_SUBMIT, then verifies the destination byte-for-byte.
 *
 *   ml_dmablit_test               # default: 512 KiB, 3-segment batch (mimics Y/U/V)
 *   ml_dmablit_test SIZE NSEG     # SIZE bytes total, split into NSEG copy entries
 *   ml_dmablit_test SIZE NSEG IT  # ...then time IT ioctl-only iterations (throughput)
 *
 * Exit 0 = PASS (destination matches), 1 = setup error, 2 = data mismatch.
 * Needs CONFIG_DMABUF_HEAPS_CMA (configs/display.config) + ml_dmablit.ko loaded.
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

#include "../modules/ml_dmablit.h"

/* dma-buf heap alloc + CPU-access sync UAPI, defined locally so the build does not
 * depend on the host kernel headers (matches prime_test.c).
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

/* Open the first usable /dev/dma_heap/ device and allocate a len-byte contiguous dma-buf. */
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

	while ((de = readdir(d))) {
		if (de->d_name[0] == '.')
			continue;

		snprintf(path, sizeof(path), "/dev/dma_heap/%s", de->d_name);
		hfd = open(path, O_RDWR | O_CLOEXEC);
		if (hfd >= 0) {
			printf("heap: %s\n", path);
			break;
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
	return alloc.fd;
}

static void sync_buf(int fd, uint64_t rw, int start)
{
	struct dma_buf_sync s = { .flags = rw | (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END) };

	ioctl(fd, DMA_BUF_IOCTL_SYNC, &s);
}

int main(int argc, char **argv)
{
	size_t size = (argc > 1) ? strtoul(argv[1], NULL, 0) : (512 * 1024);
	int nseg = (argc > 2) ? atoi(argv[2]) : 3;
	int iters = (argc > 3) ? atoi(argv[3]) : 0;
	int df, sf, dd, rc = 0;
	uint8_t *sp, *dp;
	struct ml_dmablit_req req;

	if (nseg < 1 || nseg > ML_DMABLIT_MAX_COPIES) {
		fprintf(stderr, "nseg must be 1..%d\n", ML_DMABLIT_MAX_COPIES);
		return 1;
	}

	size &= ~((size_t)(4 * nseg - 1));	/* keep every segment 4-byte aligned */
	if (size == 0) {
		fprintf(stderr, "size too small\n");
		return 1;
	}

	printf("ml_dmablit_test: %zu bytes, %d segment(s)\n", size, nseg);

	dd = open("/dev/ml-dmablit", O_RDWR | O_CLOEXEC);
	if (dd < 0) {
		perror("/dev/ml-dmablit (ml_dmablit.ko loaded?)");
		return 1;
	}

	sf = heap_alloc(size);
	df = heap_alloc(size);
	if (sf < 0 || df < 0)
		return 1;

	sp = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, sf, 0);
	dp = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, df, 0);
	if (sp == MAP_FAILED || dp == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	/* Fill source with a position-dependent pattern; poison destination so a
	 * missed byte is obvious. Bracket CPU writes with the dma-buf sync ioctl.
	 */
	sync_buf(sf, DMA_BUF_SYNC_WRITE, 1);
	sync_buf(df, DMA_BUF_SYNC_WRITE, 1);

	for (size_t i = 0; i < size; i++) {
		sp[i] = (uint8_t)(i * 131 + 7);
		dp[i] = 0xA5;
	}

	sync_buf(sf, DMA_BUF_SYNC_WRITE, 0);
	sync_buf(df, DMA_BUF_SYNC_WRITE, 0);

	/* Build the batch: nseg contiguous slices, all from the one source fd (the
	 * fd-dedup path the compositor hits with Y/U/V of a tile).
	 */
	memset(&req, 0, sizeof(req));
	req.dst_fd = df;
	req.n = nseg;
	size_t seg = size / nseg;

	seg &= ~(size_t)3;
	for (int i = 0; i < nseg; i++) {
		uint32_t off = (uint32_t)(i * seg);
		uint32_t len = (i == nseg - 1) ? (uint32_t)(size - off) : (uint32_t)seg;

		req.copy[i].src_fd = sf;
		req.copy[i].src_off = off;
		req.copy[i].dst_off = off;
		req.copy[i].len = len & ~3u;
	}

	if (ioctl(dd, ML_DMABLIT_SUBMIT, &req)) {
		fprintf(stderr, "ML_DMABLIT_SUBMIT: %s\n", strerror(errno));
		return 1;
	}

	/* Verify. Sync destination for CPU read first. */
	sync_buf(df, DMA_BUF_SYNC_READ, 1);
	size_t bad = 0, first_bad = 0;

	for (size_t i = 0; i < size; i++) {
		if (dp[i] != sp[i]) {
			if (bad == 0)
				first_bad = i;
			bad++;
		}
	}
	sync_buf(df, DMA_BUF_SYNC_READ, 0);

	if (bad) {
		printf("ml_dmablit_test: FAIL - %zu/%zu bytes differ (first at %zu: got 0x%02x want 0x%02x)\n",
		       bad, size, first_bad, dp[first_bad], sp[first_bad]);
		rc = 2;
	} else {
		printf("ml_dmablit_test: PASS - %zu bytes copied correctly via dw-axi-dmac\n", size);
	}

	/* Throughput: time the ioctl alone (no CPU verify competing for the bus), which
	 * is the real per-frame compose cost the pipeline will pay.
	 */
	if (rc == 0 && iters > 0) {
		struct timespec t0, t1;

		clock_gettime(CLOCK_MONOTONIC, &t0);
		for (int it = 0; it < iters; it++) {
			if (ioctl(dd, ML_DMABLIT_SUBMIT, &req)) {
				fprintf(stderr, "iter %d: %s\n", it, strerror(errno));
				rc = 1;
				break;
			}
		}

		clock_gettime(CLOCK_MONOTONIC, &t1);
		if (rc == 0) {
			double secs = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
			double per_ms = secs * 1e3 / iters;
			double mbps = (double)size * iters / secs / 1e6;

			printf("ml_dmablit_test: %d iters, %.3f ms/submit, %.0f MB/s (%.1f fps ceiling)\n",
			       iters, per_ms, mbps, 1e3 / per_ms);
		}
	}

	munmap(sp, size);
	munmap(dp, size);
	close(sf);
	close(df);
	close(dd);

	return rc;
}
