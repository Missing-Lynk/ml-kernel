// SPDX-License-Identifier: GPL-2.0
/* display_orient - draw a large "F" orientation marker into the DRM scanout, then force a
 * modeset. The "F" is asymmetric on both axes and not mirror-symmetric, so it reads out the
 * exact panel transform at a glance:
 *   upright + top-left, readable   -> correct
 *   bottom-right, upside-down      -> full 180 rotation
 *   mirrored left-right            -> source (horizontal) scan inverted
 *   flipped top-bottom             -> gate (vertical) scan inverted
 * The forced CRTC off/on re-runs the panel driver's prepare(), so it re-reads the panel
 * module's live `flip` param (/sys/module/panel_qy45043a0/parameters/flip). usage: display_orient
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>
#include "drm_util.h"

static void fill(uint32_t *p, uint32_t sp, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1,
	uint32_t c)
{
	for (uint32_t y = y0; y < y1; y++) {
		for (uint32_t x = x0; x < x1; x++)
			p[(size_t)y * sp + x] = c;
	}
}

int main(void)
{
	const uint32_t WHITE = 0x00ffffff;
	const uint32_t BLACK = 0x00000000;

	setbuf(stdout, NULL);

	int fd = drm_open_card();

	if (fd < 0)
		return 1;

	struct drm_mode_card_res res = {0};
	uint64_t conn_ids[16];
	uint64_t crtc_ids[16];

	ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res);
	res.connector_id_ptr = (uint64_t)conn_ids;
	res.crtc_id_ptr = (uint64_t)crtc_ids;
	res.count_connectors = res.count_connectors > 16 ? 16 : res.count_connectors;
	res.count_crtcs = res.count_crtcs > 16 ? 16 : res.count_crtcs;
	res.count_encoders = res.count_fbs = 0;
	ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res);
	if (!res.count_crtcs || !res.count_connectors) {
		fprintf(stderr, "no crtc/conn\n");
		return 1;
	}

	struct drm_mode_crtc gc = { .crtc_id = crtc_ids[0] };

	if (ioctl(fd, DRM_IOCTL_MODE_GETCRTC, &gc) || !gc.fb_id) {
		fprintf(stderr, "crtc has no current fb\n");
		return 1;
	}

	struct drm_fbmap fb = {0};

	if (drm_map_scanout(fd, &fb))
		return 1;

	uint32_t *p = fb.px;
	uint32_t sp = fb.pitch / 4;
	uint32_t w = fb.width;
	uint32_t h = fb.height;

	/* black field, then a 4px white border so the panel edges/corners are visible too */
	fill(p, sp, 0, 0, w, h, BLACK);
	fill(p, sp, 0, 0, w, 4, WHITE);
	fill(p, sp, 0, h - 4, w, h, WHITE);
	fill(p, sp, 0, 0, 4, h, WHITE);
	fill(p, sp, w - 4, 0, w, h, WHITE);

	/* big "F" anchored to the top-left: vertical stem + top bar + middle bar */
	fill(p, sp, 200, 150, 280, 750, WHITE);   /* stem */
	fill(p, sp, 200, 150, 560, 230, WHITE);   /* top bar */
	fill(p, sp, 200, 420, 500, 500, WHITE);   /* middle bar */

	drm_clean(p, fb.size);

	/* force a real modeset: disable then re-enable, re-using the existing fb, so the panel
	 * driver's prepare() re-runs and re-reads its live `flip` param.
	 */
	struct drm_mode_crtc off = { .crtc_id = crtc_ids[0], .fb_id = 0, .count_connectors = 0,
		.mode_valid = 0 };
	if (ioctl(fd, DRM_IOCTL_MODE_SETCRTC, &off))
		perror("setcrtc-off");

	struct drm_mode_crtc on = { .crtc_id = crtc_ids[0], .fb_id = gc.fb_id,
		.set_connectors_ptr = (uint64_t)conn_ids, .count_connectors = 1,
		.mode = gc.mode, .mode_valid = 1 };
	if (ioctl(fd, DRM_IOCTL_MODE_SETCRTC, &on)) {
		perror("setcrtc-on");
		return 1;
	}

	printf("drew F marker %ux%u, forced modeset\n", w, h);
	return 0;
}
