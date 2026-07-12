// SPDX-License-Identifier: GPL-2.0
// cgu_dump - read-only dump of the AR9311 CGU register banks via /dev/mem.
//
// Validation instrument for the CGU clock table: dumps the raw registers so
// the host-side decoder can reconstruct the clock tree and diff it against the
// extracted table and the vendor captures.
//
// STRICTLY read-only: /dev/mem is opened O_RDONLY and mapped
// PROT_READ, so this cannot write a clock register by construction.
//
//   cgu_dump            # prints "0xADDR 0xVALUE" per 32-bit word, all three banks

#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

static const struct { uint64_t base; size_t len; } banks[] = {
	{ 0x0a100000, 0x420 },	/* CPU/debug bank: comp_a53 +0x00/+0x08/+0x408, cs_dbg +0x10 */
	{ 0x0a104000, 0x210 },	/* leaf gate+mux bank incl. +0x200/+0x204 */
	{ 0x0a108000, 0x420 },	/* PLL bank (shared with ADC/protemp; reads only) */
};

int main(void)
{
	long page = sysconf(_SC_PAGESIZE);
	int fd = open("/dev/mem", O_RDONLY | O_SYNC);
	unsigned int i;
	size_t off;

	if (fd < 0) {
		perror("/dev/mem");
		return 1;
	}

	for (i = 0; i < sizeof(banks) / sizeof(banks[0]); i++) {
		uint64_t base = banks[i].base & ~((uint64_t)page - 1);
		size_t pgoff = banks[i].base - base;
		size_t len = (banks[i].len + pgoff + page - 1) & ~((size_t)page - 1);
		volatile uint32_t *map = mmap(NULL, len, PROT_READ, MAP_SHARED, fd, base);

		if (map == MAP_FAILED) {
			perror("mmap");
			return 1;
		}

		for (off = 0; off < banks[i].len; off += 4)
			printf("0x%08" PRIx64 " 0x%08x\n", banks[i].base + off,
			       map[(pgoff + off) / 4]);

		munmap((void *)map, len);
	}

	return 0;
}
