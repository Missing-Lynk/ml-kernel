// SPDX-License-Identifier: GPL-2.0
/*
 * mmztest - userspace smoke test for the open ar_osal (MMZ) + ar_sys modules.
 * Run ON THE DEVICE (aarch64) after `../modules/load.sh`, as Phase 1/2 of
 * ../modules/HW-BRINGUP.md. Exercises the /dev/mmz_userdev alloc/mmap/free +
 * cache ioctls and the /dev/ar_sys PTS path with the byte-exact ABI from
 * ../modules/ar_uapi.h.
 *
 * It does NOT exercise the engines (codec/scaler) - those need the closed
 * userspace. This just proves the foundation ioctl/mmap contracts work on HW.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "../modules/ar_uapi.h"

#define MB(x) ((unsigned long)(x) << 20)

static int test_mmz(void)
{
	struct mmb_info mi;
	int fd, rc, fail = 0;
	void *p;
	const unsigned long sz = MB(1);

	fd = open("/dev/mmz_userdev", O_RDWR);
	if (fd < 0) {
		perror("open mmz_userdev");
		return 1;
	}

	/* alloc */
	memset(&mi, 0, sizeof(mi));
	mi.size = sz;
	mi.align = 0x1000;
	strncpy(mi.mmb_name, "mmztest", sizeof(mi.mmb_name) - 1);
	rc = ioctl(fd, IOC_MMB_ALLOC, &mi);
	if (rc) {
		perror("IOC_MMB_ALLOC");
		close(fd);
		return 1;
	}

	printf("alloc: phys=0x%llx size=0x%llx\n",
	       (unsigned long long)mi.phys_addr, (unsigned long long)mi.size);
	if (mi.phys_addr < 0x29400000ULL || mi.phys_addr >= 0x30000000ULL) {
		printf("  WARN: phys outside the anonymous zone\n");
		fail++;
	}

	/* mmap at offset = phys, write/read a pattern (WC: barrier via volatile) */
	p = mmap(NULL, sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
		 (off_t)mi.phys_addr);
	if (p == MAP_FAILED) {
		perror("mmap");
		fail++;
	} else {
		volatile unsigned int *u = p;
		size_t i, n = sz / sizeof(*u);

		for (i = 0; i < n; i++)
			u[i] = (unsigned int)(i * 2654435761u);
		__sync_synchronize();

		for (i = 0; i < n; i++) {
			if (u[i] != (unsigned int)(i * 2654435761u)) {
				printf("  MISMATCH at %zu: 0x%x\n", i, u[i]);
				fail++;
				break;
			}
		}

		if (!fail)
			printf("mmap+rw: 1 MiB pattern verified\n");
		munmap(p, sz);
	}

	/* free */
	rc = ioctl(fd, IOC_MMB_FREE, &mi);
	if (rc) {
		perror("IOC_MMB_FREE");
		fail++;
	} else {
		printf("free: ok\n");
	}

	close(fd);
	return fail;
}

static int test_pts(void)
{
	int fd, fail = 0;
	unsigned long long base = 1000000, a = 0, b = 0;

	fd = open("/dev/ar_sys", O_RDWR);
	if (fd < 0) {
		perror("open ar_sys");
		return 1;
	}

	if (ioctl(fd, IOC_SYS_INIT_PTS_BASE, &base)) {
		perror("INIT_PTS_BASE");
		fail++;
	}

	if (ioctl(fd, IOC_SYS_GET_CUR_PTS, &a)) {
		perror("GET_CUR_PTS");
		fail++;
	}

	usleep(1000);

	if (ioctl(fd, IOC_SYS_GET_CUR_PTS, &b)) {
		perror("GET_CUR_PTS");
		fail++;
	}

	printf("pts: a=%llu b=%llu (delta=%lld us)\n", a, b, (long long)(b - a));
	if (b < a) {
		printf("  PTS went backwards!\n");
		fail++;
	}

	close(fd);
	return fail;
}

int main(void)
{
	int fail = 0;

	printf("== MMZ ==\n");
	fail += test_mmz();
	printf("== PTS ==\n");
	fail += test_pts();
	printf("\n%s (%d failures)\n", fail ? "FAIL" : "PASS", fail);

	return fail ? 1 : 0;
}
