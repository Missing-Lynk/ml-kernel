// SPDX-License-Identifier: GPL-2.0
/* overlay_test - primary colour bars plus an ARGB4444 box on the OVERLAY plane, exercising
 * the artosyn_vo two-plane compositor and per-pixel alpha through the real DRM plane ABI. A
 * transparent overlay pixel lets the bars show through; the box is drawn with the given
 * ARGB4444 value (default 0x8fff = half-alpha white).
 *   usage: overlay_test [argb4444_hex] [box_y] [box_height]
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "drm_util.h"

int main(int argc, char **argv)
{
	uint16_t px = argc > 1 ? (uint16_t)strtoul(argv[1], 0, 16) : 0x8fff;
	int y0 = argc > 2 ? atoi(argv[2]) : 490;
	int bh = argc > 3 ? atoi(argv[3]) : 100;

	int fd = drm_open_card();

	if (fd < 0)
		return 1;

	/* primary: fill the current scanout fb with 8 colour bars */
	struct drm_fbmap prim;

	if (!drm_map_scanout(fd, &prim)) {
		uint32_t *p = prim.px;
		uint32_t sp = prim.pitch / 4;
		static const uint32_t bar[8] = {0xffffff, 0xffff00, 0x00ffff, 0x00ff00,
			0xff00ff, 0xff0000, 0x0000ff, 0x000000};
		for (uint32_t y = 0; y < prim.height; y++) {
			for (uint32_t x = 0; x < prim.width; x++)
				p[(size_t)y * sp + x] = bar[(x * 8) / prim.width];
		}
		drm_clean(p, prim.size);
	}

	uint32_t crtc = drm_crtc(fd);
	uint32_t ovl = drm_find_overlay(fd);

	if (!ovl) {
		fprintf(stderr, "no ARGB4444 overlay plane found\n");
		return 1;
	}

	/* overlay: an ARGB4444 buffer, transparent except a full-width box */
	struct drm_fbmap ov;

	if (drm_make_argb4444(fd, 1920, 1080, &ov))
		return 1;

	uint16_t *o = ov.px;
	uint32_t wp = ov.pitch / 2;

	for (size_t i = 0; i < ov.size / 2; i++)
		o[i] = 0;

	for (int y = y0; y < y0 + bh && y < (int)ov.height; y++) {
		for (int x = 0; x < (int)ov.width; x++)
			o[(size_t)y * wp + x] = px;
	}
	drm_clean(o, ov.size);

	if (drm_setplane(fd, ovl, crtc, &ov))
		return 1;

	printf("primary=bars, overlay plane %u: box px=0x%04x y=%d h=%d\n", ovl, px, y0, bh);
	pause();
	return 0;
}
