// SPDX-License-Identifier: GPL-2.0
/*
 * gpio_verify.c - prove a gpio line actually drives its pad. Requests the line as
 * output via the cdev v2 ABI, drives it 0 then 1, and after each read the controller's
 * DAT (pad-level) register via /dev/mem - so we can see whether the physical pad
 * follows the driven value (confirms the pinmux + line mapping are correct, e.g. that
 * GPIO23 == the AR8030 reset pad). Usage: gpio_verify <chip-label> <line> <dat-phys-addr> <bit>
 * e.g. gpio_verify ar-gpio1 0 0x0A10A0D4 0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <dirent.h>
#include <linux/gpio.h>

/* Max length of a "/dev/<dirent>" path: the prefix plus a full directory-entry name. */
#define GPIOCHIP_PATH_MAX (sizeof("/dev/") + sizeof(((struct dirent *)0)->d_name))

static uint32_t mem_read(unsigned long addr)
{
	int fd = open("/dev/mem", O_RDONLY | O_SYNC);

	if (fd < 0)
		return 0xdeadbeef;

	unsigned long page = addr & ~0xfffUL;
	unsigned long off = addr & 0xfffUL;
	void *m = mmap(NULL, 0x1000, PROT_READ, MAP_SHARED, fd, page);
	uint32_t v = 0xdeadbeef;

	if (m != MAP_FAILED) {
		v = *(volatile uint32_t *)((char *)m + off);
		munmap(m, 0x1000);
	}

	close(fd);
	return v;
}

static int find_chip(const char *label, char *path, size_t plen)
{
	DIR *d = opendir("/dev");
	struct dirent *e;

	if (!d)
		return -1;

	while ((e = readdir(d))) {
		if (strncmp(e->d_name, "gpiochip", 8) != 0)
			continue;

		char p[GPIOCHIP_PATH_MAX];

		snprintf(p, sizeof(p), "/dev/%s", e->d_name);
		int fd = open(p, O_RDONLY);

		if (fd < 0)
			continue;

		struct gpiochip_info ci;

		if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &ci) == 0 &&
		    strcmp(ci.label, label) == 0) {
			close(fd);
			snprintf(path, plen, "%s", p);
			closedir(d);

			return 0;
		}
		close(fd);
	}

	closedir(d);
	return -1;
}

int main(int argc, char **argv)
{
	if (argc < 5) {
		fprintf(stderr, "usage: %s <chip-label> <line> <dat-phys> <bit>\n", argv[0]);
		return 2;
	}

	const char *label = argv[1];
	unsigned int line = strtoul(argv[2], NULL, 0);
	unsigned long dat = strtoul(argv[3], NULL, 0);
	unsigned int bit = strtoul(argv[4], NULL, 0);

	char path[GPIOCHIP_PATH_MAX];

	if (find_chip(label, path, sizeof(path)) < 0) {
		fprintf(stderr, "chip '%s' not found\n", label);
		return 1;
	}

	int cfd = open(path, O_RDONLY);

	if (cfd < 0) {
		perror("open chip");
		return 1;
	}

	struct gpio_v2_line_request req;

	memset(&req, 0, sizeof(req));
	req.offsets[0] = line;
	req.num_lines = 1;
	req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
	snprintf(req.consumer, sizeof(req.consumer), "gpio_verify");
	if (ioctl(cfd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
		perror("get line");
		close(cfd);
		return 1;
	}

	struct gpio_v2_line_values vals;

	for (int v = 0; v <= 1; v++) {
		memset(&vals, 0, sizeof(vals));
		vals.mask = 1;
		vals.bits = v ? 1 : 0;
		ioctl(req.fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals);
		usleep(50000);
		uint32_t d = mem_read(dat);

		printf("drive %d -> DAT[0x%lx]=0x%08x  bit%u=%u  %s\n",
		       v, dat, d, bit, (d >> bit) & 1,
		       (((d >> bit) & 1) == (unsigned int)v) ? "(pad follows)" : "(PAD DOES NOT FOLLOW)");
	}

	close(req.fd);
	close(cfd);
	return 0;
}
