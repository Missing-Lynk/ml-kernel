// SPDX-License-Identifier: GPL-2.0
/*
 * gpio_reset.c - replicate the vendor start_ar813x.sh reset sequence EXACTLY:
 * drive the line low (assert), then high (deassert), then RELEASE IT TO INPUT
 * (high-Z) - matching the vendor's "echo 0; echo 1; unexport", which leaves the
 * AR8030 reset as an input held by the external pull (slot A runtime state).
 * gpio_pulse left it output-high, which actively fights the line. Then read the
 * pad-level (DAT) register so we see the released level.
 * Usage: gpio_reset <chip-label> <line> [dat-phys] [bit]
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

	unsigned long page = addr & ~0xfffUL, off = addr & 0xfffUL;
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
		if (strncmp(e->d_name, "gpiochip", 8))
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
	if (argc < 3) {
		fprintf(stderr, "usage: %s <chip-label> <line> [dat-phys] [bit]\n", argv[0]);
		return 2;
	}

	const char *label = argv[1];
	unsigned int line = strtoul(argv[2], NULL, 0);
	unsigned long dat = argc > 3 ? strtoul(argv[3], NULL, 0) : 0;
	unsigned int bit = argc > 4 ? strtoul(argv[4], NULL, 0) : 0;

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

	/* request as OUTPUT, initial high */
	struct gpio_v2_line_request req;

	memset(&req, 0, sizeof(req));
	req.offsets[0] = line;
	req.num_lines = 1;
	req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
	req.config.num_attrs = 1;
	req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
	req.config.attrs[0].attr.values = 1;
	req.config.attrs[0].mask = 1;
	snprintf(req.consumer, sizeof(req.consumer), "gpio_reset");
	if (ioctl(cfd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
		perror("get line");
		close(cfd);
		return 1;
	}

	struct gpio_v2_line_values vals;

	/* assert (low) */
	memset(&vals, 0, sizeof(vals));
	vals.mask = 1; vals.bits = 0;
	ioctl(req.fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals);
	usleep(20000);

	/* deassert (high) */
	vals.bits = 1;
	ioctl(req.fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals);
	usleep(20000);

	/* RELEASE TO INPUT (high-Z) - the crucial step the vendor's unexport does */
	struct gpio_v2_line_config cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.flags = GPIO_V2_LINE_FLAG_INPUT;
	if (ioctl(req.fd, GPIO_V2_LINE_SET_CONFIG_IOCTL, &cfg) < 0)
		perror("set config input");

	usleep(20000);

	if (dat) {
		uint32_t d = mem_read(dat);

		printf("reset pulsed + released to INPUT; DAT[0x%lx]=0x%08x bit%u=%u\n",
		       dat, d, bit, (d >> bit) & 1);
	} else {
		printf("reset pulsed + released to INPUT (line %u)\n", line);
	}

	close(req.fd);
	close(cfd);
	return 0;
}
