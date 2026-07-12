// SPDX-License-Identifier: GPL-2.0
/*
 * vblank_rate: measure the panel refresh rate from DRM vblank timestamp deltas.
 *
 * The validation instrument for the pixel clock path in the clk-ar9311-cgu
 * provider (formerly clk-ar9311-pixel): the CRTC must be scanning out (run i420_test /
 * prime_test / fbcon first), then this tool waits N vblanks and prints the
 * mean/min/max period and the implied refresh in Hz. Steering the pixel clock
 * (clk_set_rate on pclk, or stepping the divider field) must move the number
 * this prints; the ratio of two measurements pins the divider encoding.
 *
 *   vblank_rate [count]     default 120 vblanks (~2 s at 60 Hz)
 *
 * Self-contained (no drm_util): DRM_IOCTL_WAIT_VBLANK needs no master and no
 * modeset, only an enabled CRTC (crtc 0 / the first pipe on this SoC).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <sys/ioctl.h>

/* Local UAPI mirror (drm.h subset) so the build does not depend on host headers. */
struct drm_wait_vblank_request {
	int type;
	unsigned int sequence;
	unsigned long signal;
};

struct drm_wait_vblank_reply {
	int type;
	unsigned int sequence;
	long tval_sec;
	long tval_usec;
};

union drm_wait_vblank {
	struct drm_wait_vblank_request request;
	struct drm_wait_vblank_reply reply;
};

#define DRM_VBLANK_RELATIVE 0x1
#define DRM_IOCTL_WAIT_VBLANK _IOWR('d', 0x3a, union drm_wait_vblank)

int main(int argc, char **argv)
{
	int count = (argc > 1) ? atoi(argv[1]) : 120;
	int fd, i;
	double prev = 0, sum = 0, sum2 = 0, min = 1e18, max = 0;
	int n = 0;

	fd = open("/dev/dri/card0", O_RDWR);
	if (fd < 0) {
		perror("open /dev/dri/card0");
		return 1;
	}

	for (i = 0; i <= count; i++) {
		union drm_wait_vblank vbl;

		memset(&vbl, 0, sizeof(vbl));
		vbl.request.type = DRM_VBLANK_RELATIVE;
		vbl.request.sequence = 1;
		if (ioctl(fd, DRM_IOCTL_WAIT_VBLANK, &vbl) < 0) {
			fprintf(stderr, "WAIT_VBLANK: %s (is the CRTC scanning out?)\n",
				strerror(errno));
			return 1;
		}

		double t = vbl.reply.tval_sec * 1e6 + vbl.reply.tval_usec;

		if (i > 0) {	/* first wait only anchors the timestamp */
			double d = t - prev;

			sum += d;
			sum2 += d * d;
			if (d < min)
				min = d;

			if (d > max)
				max = d;

			n++;
		}
		prev = t;
	}
	close(fd);

	if (!n)
		return 1;

	double mean = sum / n;
	double var = sum2 / n - mean * mean;

	printf("%d vblanks: period mean %.2f us (min %.2f, max %.2f, stddev %.2f) = %.4f Hz\n",
	       n, mean, min, max, var > 0 ? sqrt(var) : 0, 1e6 / mean);

	return 0;
}
