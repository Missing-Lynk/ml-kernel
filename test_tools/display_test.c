// SPDX-License-Identifier: GPL-2.0
/* display_test - reuse the existing DRM scanout fb, fill the real GEM coherently, and force a
 * modeset. Validates the artosyn_vo primary-plane path end to end (it keeps a debug readback
 * and a forced disable/enable modeset from bring-up). usage: display_test 0xAARRGGBB | bars
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>
#include "drm_util.h"

int main(int argc, char **argv)
{
	uint32_t color = 0x00ff0000;
	int bars = 0;

	if (argc > 1) {
		if (!strcmp(argv[1], "bars"))
			bars = 1;
		else
			color = strtoul(argv[1], 0, 16);
	}
	setbuf(stdout, NULL);

	int fd = drm_open_card();

	if (fd < 0)
		return 1;

	struct drm_mode_card_res res = {0};
	uint64_t conn_ids[16];
	uint64_t enc_ids[16];
	uint64_t crtc_ids[16];
	uint64_t fb_ids[16];

	ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res);
	res.connector_id_ptr = (uint64_t)conn_ids;
	res.encoder_id_ptr = (uint64_t)enc_ids;
	res.crtc_id_ptr = (uint64_t)crtc_ids;
	res.fb_id_ptr = (uint64_t)fb_ids;

	if (res.count_connectors > 16)
		res.count_connectors = 16;

	if (res.count_crtcs > 16)
		res.count_crtcs = 16;

	ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res);
	if (!res.count_crtcs || !res.count_connectors) {
		fprintf(stderr, "no crtc/conn\n");
		return 1;
	}

	struct drm_mode_crtc gc = { .crtc_id = crtc_ids[0] };

	if (ioctl(fd, DRM_IOCTL_MODE_GETCRTC, &gc)) {
		perror("getcrtc");
		return 1;
	}

	printf("crtc %llu fb=%u mode=%s %ux%u valid=%u\n", (unsigned long long)crtc_ids[0],
		gc.fb_id, gc.mode.name, gc.mode.hdisplay, gc.mode.vdisplay, gc.mode_valid);

	if (!gc.fb_id) {
		fprintf(stderr, "crtc has no current fb\n");
		return 1;
	}

	struct drm_mode_fb_cmd gf = { .fb_id = gc.fb_id };

	if (ioctl(fd, DRM_IOCTL_MODE_GETFB, &gf)) {
		perror("getfb");
		return 1;
	}

	printf("fb %ux%u pitch=%u bpp=%u handle=%u\n", gf.width, gf.height, gf.pitch, gf.bpp, gf.handle);
	if (!gf.handle) {
		fprintf(stderr, "getfb gave no handle\n");
		return 1;
	}

	struct drm_mode_map_dumb md = { .handle = gf.handle };

	if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &md)) {
		perror("mapdumb");
		return 1;
	}

	size_t size = (size_t)gf.pitch * gf.height;
	uint32_t *p = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, md.offset);

	if (p == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	int yv12 = argc > 1 && !strcmp(argv[1], "yv12");

	if (yv12) {
		// YV12 layout at stride 1920: Y plane (vertical gradient), then V, U (neutral 0x80)
		uint8_t *b = (uint8_t *)p;
		uint32_t W = 1920;
		uint32_t Hh = 1080;
		uint32_t ys = 1920;

		for (uint32_t y = 0; y < Hh; y++) {
			uint8_t Y = (uint8_t)(y * 256 / Hh);   // dark top -> bright bottom

			for (uint32_t x = 0; x < W; x++)
				b[(size_t)y * ys + x] = Y;
		}

		// chroma planes start after Y (stock offsets U=+0x21c000, V=+0x2a3000), set neutral
		for (size_t i = 0x1fa400; i < 0x300000 && i < size; i++)
			b[i] = 0x80;
	} else {
		uint32_t sp = gf.pitch / 4;

		for (uint32_t y = 0; y < gf.height; y++) {
			for (uint32_t x = 0; x < gf.width; x++) {
				uint32_t c = color;

				if (bars) {
					static const uint32_t bar[8] = {0xffffff, 0xffff00, 0x00ffff, 0x00ff00,
						0xff00ff, 0xff0000, 0x0000ff, 0x000000};
					c = bar[(x * 8) / gf.width];
				}
				p[(size_t)y * sp + x] = c;
			}
		}
	}

	/* Flush the pixels to DDR so the non-coherent DC sees them (see drm_clean / drm_util.c). */
	drm_clean(p, size);

	/*
	 * Debug-only readback: clean+invalidate (dc civac) the first few lines so the loads below
	 * miss the cache and come from DDR, proving what actually landed in RAM for the DC to
	 * fetch. isb after the barrier so the reads observe the invalidated state.
	 */
	for (uintptr_t a = (uintptr_t)p & ~63UL; a < (uintptr_t)p + 256; a += 64)
		asm volatile("dc civac, %0" :: "r"(a) : "memory");
	asm volatile("dsb sy; isb" ::: "memory");
	printf("RAM readback (post-invalidate) p[0]=0x%08x p[1]=0x%08x\n", p[0], p[1]);

	if (argc > 2 && !strcmp(argv[2], "noset")) {
		printf("filled+flushed, no modeset\n");
		return 0;
	}

	// force a REAL modeset: disable, then re-enable -> guarantees ar_vo_pipe_enable runs (XRGB8888)
	struct drm_mode_crtc off = { .crtc_id = crtc_ids[0], .fb_id = 0, .count_connectors = 0, .mode_valid = 0 };

	if (ioctl(fd, DRM_IOCTL_MODE_SETCRTC, &off))
		perror("setcrtc-off");
	else
		printf("crtc disabled\n");

	struct drm_mode_crtc sc = { .crtc_id = crtc_ids[0], .fb_id = gc.fb_id,
		.set_connectors_ptr = (uint64_t)conn_ids, .count_connectors = 1,
		.mode = gc.mode, .mode_valid = 1 };

	if (ioctl(fd, DRM_IOCTL_MODE_SETCRTC, &sc))
		perror("setcrtc-on");
	else
		printf("crtc re-enabled (forced modeset)\n");

	printf("filled real GEM: 0x%08x bars=%d\n", color, bars);
	return 0;
}
