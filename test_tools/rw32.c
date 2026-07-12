// SPDX-License-Identifier: GPL-2.0
/*
 * rw32: minimal /dev/mem 32-bit register peek/poke (busybox devmem is absent
 * from the slot-B rootfs).
 *
 *   rw32 <addr>            read one 32-bit register
 *   rw32 <addr> <value>    write it
 *
 * Addresses/values in hex or decimal (strtoul base 0).
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

int main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: rw32 <addr> [value]\n");
		return 1;
	}

	uint64_t addr = strtoull(argv[1], NULL, 0);
	long pagesz = sysconf(_SC_PAGESIZE);
	uint64_t page = addr & ~(uint64_t)(pagesz - 1);

	int fd = open("/dev/mem", O_RDWR | O_SYNC);

	if (fd < 0) {
		perror("open /dev/mem");
		return 1;
	}

	volatile uint32_t *map = mmap(NULL, pagesz, PROT_READ | PROT_WRITE,
				      MAP_SHARED, fd, page);
	if (map == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	volatile uint32_t *reg = map + (addr - page) / 4;

	if (argc > 2)
		*reg = (uint32_t)strtoul(argv[2], NULL, 0);

	printf("0x%08x\n", *reg);
	return 0;
}
