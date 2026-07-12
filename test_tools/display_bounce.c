// SPDX-License-Identifier: GPL-2.0
/* display_bounce - DVD-style colour-cycling box bouncing on the primary DRM scanout buffer.
 * Coherent dumb-buffer mmap; redraws only the changed rows each frame and cleans them to RAM.
 */
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include "drm_util.h"

int main(void)
{
	int fd = drm_open_card();
	struct drm_fbmap fbm;

	if (fd < 0 || drm_map_scanout(fd, &fbm))
		return 1;

	uint32_t *m = fbm.px;
	uint32_t W = fbm.width;
	uint32_t H = fbm.height;
	uint32_t sp = fbm.pitch / 4;

	uint32_t bg = 0x00101828;

	for (size_t i = 0; i < fbm.size / 4; i++)
		m[i] = bg;
	drm_clean(m, fbm.size);

	int lw = 150;
	int lh = 90;
	int x = 220;
	int y = 160;
	int vx = 7;
	int vy = 5;
	int ox = x;
	int oy = y;
	uint32_t pal[] = {0x00ff5a5a, 0x0050e0a0, 0x005aa0ff, 0x00ffcf50, 0x00e070ff, 0x0050e0e0};
	int ci = 0;

	printf("bounce: %ux%u pitch=%u fb=%u\n", W, H, fbm.pitch, fbm.fb_id);
	for (;;) {
		for (int j = 0; j < lh; j++) {
			for (int i = 0; i < lw; i++)
				m[(size_t)(oy + j) * sp + ox + i] = bg;
		}

		for (int j = 0; j < lh; j++) {
			for (int i = 0; i < lw; i++)
				m[(size_t)(y + j) * sp + x + i] = pal[ci];
		}

		int y0 = oy < y ? oy : y;
		int y1 = (oy > y ? oy : y) + lh;

		drm_clean(m + (size_t)y0 * sp, (size_t)(y1 - y0) * fbm.pitch);

		ox = x;
		oy = y;
		x += vx;
		y += vy;

		if (x <= 0 || x + lw >= (int)W) {
			vx = -vx;
			x += vx;
			ci = (ci + 1) % 6;
		}

		if (y <= 0 || y + lh >= (int)H) {
			vy = -vy;
			y += vy;
			ci = (ci + 1) % 6;
		}
		usleep(16000);
	}
}
