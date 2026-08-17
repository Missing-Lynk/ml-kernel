// SPDX-License-Identifier: GPL-2.0
/*
 * dmaheap_yield - how many buffers of a given size a dma-heap will hand out.
 *
 * Answers "did this size change cost us pool buffers?" without a pipeline in the way. The
 * composite pool allocates until the heap refuses, so its yield depends on the buffer size,
 * on what else holds CMA, and on allocations from earlier generations that have not been
 * freed yet - and /proc/meminfo CmaFree is sampled too early to separate those. This holds
 * every buffer until it exits, so the count is the real simultaneous yield.
 *
 *   dmaheap_yield <heap> <bytes> [max] [hold_seconds]
 *   dmaheap_yield default_cma_region 3133440 24     # what the heap yields at this size
 *   dmaheap_yield default_cma_region 3133440 4 90   # pin 4 buffers for 90s, to squeeze a
 *                                                   # consumer into a chosen amount of headroom
 */
#include <linux/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

struct dma_heap_allocation_data {
	__u64 len;
	__u32 fd;
	__u32 fd_flags;
	__u64 heap_flags;
};
#define DMA_HEAP_IOCTL_ALLOC _IOWR('H', 0x0, struct dma_heap_allocation_data)

#define MAX_FDS 256

static long cma_free(void)
{
	FILE *f = fopen("/proc/meminfo", "r");
	char key[64];
	long val;

	if (!f)
		return -1;

	while (fscanf(f, "%63s %ld kB\n", key, &val) == 2)
		if (!strcmp(key, "CmaFree:")) {
			fclose(f);
			return val;
		}

	fclose(f);
	return -1;
}

int main(int argc, char **argv)
{
	char path[256];
	int fds[MAX_FDS];
	int hfd, n = 0, max, hold;
	unsigned long long len;

	if (argc < 3) {
		fprintf(stderr, "usage: %s <heap> <bytes> [max]\n", argv[0]);
		return 2;
	}

	len = strtoull(argv[2], NULL, 0);
	max = argc > 3 ? atoi(argv[3]) : 24;
	hold = argc > 4 ? atoi(argv[4]) : 0;
	if (max > MAX_FDS)
		max = MAX_FDS;

	snprintf(path, sizeof path, "/dev/dma_heap/%s", argv[1]);
	hfd = open(path, O_RDWR | O_CLOEXEC);
	if (hfd < 0) {
		fprintf(stderr, "%s: %s\n", path, strerror(errno));
		return 1;
	}

	printf("heap %s, %llu bytes each, CmaFree start %ld kB\n", path, len, cma_free());

	while (n < max) {
		struct dma_heap_allocation_data a = { .len = len, .fd_flags = O_RDWR | O_CLOEXEC };

		if (ioctl(hfd, DMA_HEAP_IOCTL_ALLOC, &a)) {
			printf("  stopped at %d: %s\n", n, strerror(errno));
			break;
		}

		fds[n++] = a.fd;
	}

	printf("YIELD %d buffers (%llu KiB total), CmaFree now %ld kB\n",
	       n, (unsigned long long)n * len / 1024, cma_free());
	fflush(stdout);

	if (hold) {
		printf("holding %ds\n", hold);
		fflush(stdout);
		sleep(hold);
	}

	while (n--)
		close(fds[n]);

	close(hfd);
	return 0;
}
