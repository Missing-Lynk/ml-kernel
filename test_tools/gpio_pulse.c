// SPDX-License-Identifier: GPL-2.0
/*
 * gpio_pulse - pulse a gpio line low->high via the GPIO chardev v2 ABI (no sysfs gpio
 * on this kernel). Used to release the AR8030 reset (GPIO23 = the artosyn_gpio
 * "ar-gpio1" bank, line 0), the open equivalent of the vendor start_ar813x.sh pulse.
 *
 * Usage: gpio_pulse [chip-label] [line]   (default: ar-gpio1 0)
 * Build: aarch64-linux-gnu-gcc -static -O2 gpio_pulse.c -o gpio_pulse
 */
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
			printf("found %s at %s (%u lines)\n", want, path, ci.lines);
			break;
		}

		close(c);
	}

	if (fd < 0) {
		fprintf(stderr, "gpiochip labelled '%s' not found\n", want);
		return 1;
	}

	memset(&req, 0, sizeof(req));
	req.offsets[0] = line;
	req.num_lines = 1;
	strncpy(req.consumer, "ar8030_reset", sizeof(req.consumer) - 1);

	req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
	if (ioctl(fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
		perror("GPIO_V2_GET_LINE_IOCTL");
		return 1;
	}
	lfd = req.fd;

	v.mask = 1;

	v.bits = 0;					/* assert (low) */
	ioctl(lfd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v);
	usleep(5000);

	v.bits = 1;					/* release (high) */
	ioctl(lfd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v);
	usleep(5000);

	printf("pulsed %s line %u low->high (AR8030 reset released)\n", want, line);
	close(lfd);
	close(fd);

	return 0;
}
