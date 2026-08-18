// SPDX-License-Identifier: GPL-2.0
/*
 * dmablit_race_test - provoke and count the dw-axi-dmac DMAC_CFG.INT_EN loss.
 *
 * DMAC_CFG (0x08800010) is the one controller-wide register holding DMAC_EN (bit 0) and
 * INT_EN (bit 1). Mainline dw-axi-dmac read-modify-writes it from two contexts with nothing
 * serializing them: dw_axi_dma_interrupt() clears INT_EN on entry and restores it on exit,
 * and axi_chan_block_xfer_start() sets DMAC_EN on every channel start. On SMP the two
 * interleave and one update is lost. Only the handler's exit path ever sets INT_EN, so the
 * loss is permanent: the controller keeps completing transfers and keeps setting per-channel
 * INTSTATUS, but raises no further interrupt, and every dmaengine client blocks to its
 * timeout. Fixed by kernel/patches/0500 + 0510 (chip->cfg_lock).
 *
 * This tool maximises the one interleave a userspace caller can steer. ml_dmablit deals
 * copy[k] of each round to channel k and starts channel 0 first, so a TINY copy[0] against a
 * LARGE copy[1] puts channel 0's completion interrupt right inside channel 1's
 * axi_dma_enable() read-modify-write. The small size is swept so the completion time walks
 * across that window instead of sitting at one offset.
 *
 * The process pins itself to a CPU that does NOT take the dw_axi_dmac interrupts (the race
 * needs the submitter and the handler on different CPUs; on one CPU the handler is atomic
 * with respect to the code it interrupts and no update can be lost).
 *
 * DMAC_CFG is read after every submit, so a hit is caught the moment it happens rather than
 * inferred from the next timeout. Each hit is repaired (INT_EN written back) and the run
 * continues, which turns a one-shot wedge into a measurable hit RATE - the thing to compare
 * before and after the fix.
 *
 *   dmablit_race_test              # 60 s, default size sweep
 *   dmablit_race_test SECS         # run for SECS seconds
 *   dmablit_race_test SECS CPU     # ...pinned to CPU instead of the auto-detected one.
 *                                  # Passing the interrupt's own CPU is the control leg.
 *
 * Exit 0 = no hit (expected WITH the fix), 3 = at least one hit (expected WITHOUT it),
 * 1 = setup error. Needs /dev/ml-dmablit, a dma_heap, and /dev/mem.
 */
/* cpu_set_t / sched_setaffinity */
#define _GNU_SOURCE

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <errno.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>

#include "../modules/ml_dmablit.h"

#define DMAC_BASE	0x08800000UL
#define DMAC_CFG	0x010
#define DMAC_EN_MASK	0x1
#define INT_EN_MASK	0x2

/* Big enough that channel 1 is still transferring when channel 0's interrupt lands, so the
 * hit is not masked by both channels finishing together.
 */
#define BIG_LEN		(512 * 1024)

/* Swept length for copy[0], the one dealt to channel 0. The window being hunted is the few
 * hundred ns between the read and the write of DMAC_CFG in channel 1's start, roughly 1-3 us
 * after channel 0 is enabled, so the sweep brackets that.
 */
static const uint32_t small_len[] = {
	4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768,
};
#define NSMALL	(int)(sizeof(small_len) / sizeof(small_len[0]))

struct dma_heap_allocation_data {
	uint64_t len;
	uint32_t fd;
	uint32_t fd_flags;
	uint64_t heap_flags;
};
#define DMA_HEAP_IOCTL_ALLOC	_IOWR('H', 0x0, struct dma_heap_allocation_data)

/* Open the first usable /dev/dma_heap/ device and allocate a len-byte contiguous dma-buf. */
static int heap_alloc(size_t len)
{
	struct dma_heap_allocation_data alloc = { .len = len, .fd_flags = O_RDWR | O_CLOEXEC };
	char path[280];
	struct dirent *de;
	DIR *d = opendir("/dev/dma_heap");
	int hfd = -1;

	if (!d) {
		perror("/dev/dma_heap");
		return -1;
	}

	while ((de = readdir(d))) {
		if (de->d_name[0] == '.')
			continue;

		snprintf(path, sizeof(path), "/dev/dma_heap/%s", de->d_name);
		hfd = open(path, O_RDWR | O_CLOEXEC);
		if (hfd >= 0)
			break;
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

/* Which CPU services the dw_axi_dmac interrupts, from the per-CPU columns of
 * /proc/interrupts. Returns -1 if no dw_axi_dmac line has ever counted, which means the
 * engine has taken no interrupt at all and the run would prove nothing.
 */
static int irq_cpu(int *ncpu_out)
{
	char line[1024];
	FILE *f = fopen("/proc/interrupts", "r");
	unsigned long best = 0;
	int best_cpu = -1, ncpu = 0;

	if (!f)
		return -1;

	/* Header line: one column label per online CPU. */
	if (fgets(line, sizeof(line), f)) {
		char *p = line;

		while ((p = strstr(p, "CPU"))) {
			ncpu++;
			p += 3;
		}
	}

	while (fgets(line, sizeof(line), f)) {
		char *p;
		int cpu;

		if (!strstr(line, "dw_axi_dmac"))
			continue;

		p = strchr(line, ':');
		if (!p)
			continue;

		p++;
		for (cpu = 0; cpu < ncpu; cpu++) {
			unsigned long v = strtoul(p, &p, 10);

			if (v > best) {
				best = v;
				best_cpu = cpu;
			}
		}
	}

	fclose(f);
	*ncpu_out = ncpu;
	return best_cpu;
}

static double now_s(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec / 1e9;
}

int main(int argc, char **argv)
{
	double secs = (argc > 1) ? atof(argv[1]) : 60.0;
	int pin_cpu = (argc > 2) ? atoi(argv[2]) : -1;
	unsigned long hits_by_size[NSMALL] = { 0 };
	unsigned long iters = 0, hits = 0, timeouts = 0;
	int dd, sf, df, mf, ncpu = 0, dmairq, i;
	volatile uint32_t *cfg;
	cpu_set_t set;
	void *map;
	double t0;

	dmairq = irq_cpu(&ncpu);
	if (dmairq < 0) {
		fprintf(stderr, "no dw_axi_dmac interrupt has ever fired - nothing to race against\n");
		return 1;
	}

	if (pin_cpu < 0)
		pin_cpu = (dmairq + 1) % (ncpu > 0 ? ncpu : 1);

	/* Pinning onto the interrupt's own CPU is the control leg, not a mistake: the handler is
	 * atomic with respect to the code it interrupts, so no update can be lost there. A run
	 * that hits cross-CPU and not same-CPU is what identifies the interleave as the mechanism.
	 */
	if (pin_cpu == dmairq)
		printf("dmablit_race_test: CONTROL LEG - submitting from the interrupt's own CPU, "
		       "expect no loss\n");

	CPU_ZERO(&set);
	CPU_SET(pin_cpu, &set);
	if (sched_setaffinity(0, sizeof(set), &set)) {
		perror("sched_setaffinity");
		return 1;
	}

	dd = open("/dev/ml-dmablit", O_RDWR | O_CLOEXEC);
	if (dd < 0) {
		perror("/dev/ml-dmablit");
		return 1;
	}

	mf = open("/dev/mem", O_RDWR | O_SYNC);
	if (mf < 0) {
		perror("/dev/mem");
		return 1;
	}

	map = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ | PROT_WRITE, MAP_SHARED, mf, DMAC_BASE);
	if (map == MAP_FAILED) {
		perror("mmap /dev/mem");
		return 1;
	}
	cfg = (volatile uint32_t *)((char *)map + DMAC_CFG);

	sf = heap_alloc(BIG_LEN);
	df = heap_alloc(BIG_LEN);
	if (sf < 0 || df < 0)
		return 1;

	if ((*cfg & INT_EN_MASK) == 0) {
		printf("NOTE: INT_EN was already clear at start (0x%08x) - repairing before the run\n",
		       *cfg);
		*cfg = DMAC_EN_MASK | INT_EN_MASK;
	}

	printf("dmablit_race_test: dmac irq on cpu%d, submitting from cpu%d, %d cpus\n",
	       dmairq, pin_cpu, ncpu);
	printf("dmablit_race_test: copy[0] swept %u..%u B against copy[1] %u B, %.0f s\n",
	       small_len[0], small_len[NSMALL - 1], (unsigned)BIG_LEN, secs);

	t0 = now_s();
	while (now_s() - t0 < secs) {
		struct ml_dmablit_req req = { .dst_fd = df, .n = 2 };
		int sweep = (int)(iters % NSMALL);
		uint32_t v;

		/* copy[0] -> channel 0 (started first, completes almost at once),
		 * copy[1] -> channel 1 (still being programmed when that interrupt lands).
		 */
		req.copy[0] = (struct ml_dmablit_copy){ sf, 0, 0, small_len[sweep] };
		req.copy[1] = (struct ml_dmablit_copy){ sf, 0, BIG_LEN / 2, BIG_LEN / 2 };

		if (ioctl(dd, ML_DMABLIT_SUBMIT, &req) != 0) {
			if (errno == ETIMEDOUT)
				timeouts++;
			else {
				perror("ML_DMABLIT_SUBMIT");
				return 1;
			}
		}
		iters++;

		v = *cfg;
		if ((v & INT_EN_MASK) == 0) {
			hits++;
			hits_by_size[sweep]++;
			printf("HIT %lu at iter %lu, copy[0]=%u B: DMAC_CFG=0x%08x, repairing\n",
			       hits, iters, small_len[sweep], v);
			fflush(stdout);
			*cfg = DMAC_EN_MASK | INT_EN_MASK;
		}
	}

	printf("\n%lu submits in %.1f s (%.0f/s), %lu INT_EN losses, %lu submit timeouts\n",
	       iters, now_s() - t0, iters / (now_s() - t0), hits, timeouts);
	printf("loss rate: %.2f per million submits, %.2f per minute\n",
	       iters ? hits * 1e6 / iters : 0.0, hits * 60.0 / (now_s() - t0));

	if (hits) {
		printf("losses by copy[0] size:\n");
		for (i = 0; i < NSMALL; i++)
			if (hits_by_size[i])
				printf("  %6u B  %lu\n", small_len[i], hits_by_size[i]);
	}

	/* Never leave the controller wedged for the next user of the device. */
	*cfg = DMAC_EN_MASK | INT_EN_MASK;

	printf("%s\n", hits ? "RACE REPRODUCED (unfixed kernel)" : "no loss seen");
	return hits ? 3 : 0;
}
