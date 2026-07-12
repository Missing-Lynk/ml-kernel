// SPDX-License-Identifier: GPL-2.0
// gpio_hold <chip-label> <line> <value> <seconds> - drive a gpio line to <value> (output) and HOLD.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

int main(int argc, char **argv)
{
	const char *want = argc > 1 ? argv[1] : "ar-gpio1";
	unsigned int line = argc > 2 ? atoi(argv[2]) : 0;
	int val = argc > 3 ? atoi(argv[3]) : 0;
	int secs = argc > 4 ? atoi(argv[4]) : 30;
	struct gpio_v2_line_request req;
	struct gpio_v2_line_values v;
	char path[40];
	int i, fd = -1, lfd;

	for (i = 0; i < 16; i++) {
		struct gpiochip_info ci;
		int c;

		snprintf(path, sizeof(path), "/dev/gpiochip%d", i);
		c = open(path, O_RDWR);
		if (c < 0)
			continue;

		memset(&ci, 0, sizeof(ci));
		if (ioctl(c, GPIO_GET_CHIPINFO_IOCTL, &ci) == 0 && strcmp(ci.label, want) == 0) {
			fd = c;
			break;
		}

		close(c);
	}

	if (fd < 0) {
		fprintf(stderr, "chip '%s' not found\n", want);
		return 1;
	}

	memset(&req, 0, sizeof(req));
	req.offsets[0] = line;
	req.num_lines = 1;
	strncpy(req.consumer, "gpio_hold", sizeof(req.consumer) - 1);
	req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
	if (ioctl(fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
		perror("GET_LINE");
		return 1;
	}

	lfd = req.fd;

	v.mask = 1;
	v.bits = val ? 1 : 0;
	ioctl(lfd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v);
	printf("HOLDING %s line %u = %d for %ds\n", want, line, val, secs);
	fflush(stdout);
	sleep(secs);
	close(lfd);
	close(fd);

	return 0;
}
