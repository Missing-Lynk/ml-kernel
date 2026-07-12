// SPDX-License-Identifier: GPL-2.0
/* display_demo - a two-plane + animation sample for artosyn_vo:
 *   PRIMARY plane: a DVD-style colour-cycling square bouncing on a dark background.
 *   OVERLAY plane (ARGB4444, per-pixel alpha): a live "FRAME NNNNNN / FPS NNN" HUD.
 * Exercises both DRM planes, live per-frame updates on each, and per-pixel alpha
 * compositing. Loops until killed (pkill display_demo).
 *
 * Usage: display_demo [fps]   default fps = 60
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "drm_util.h"
#include "drm_glyphs.h"

static long ts_diff_ns(struct timespec a, struct timespec b)
{
	return (b.tv_sec - a.tv_sec) * 1000000000L + (b.tv_nsec - a.tv_nsec);
}

int main(int argc, char **argv)
{
	int target_fps = 60;

	if (argc > 1) {
		target_fps = atoi(argv[1]);
		if (target_fps < 1)
			target_fps = 1;

		if (target_fps > 240)
			target_fps = 240;
	}
	long frame_interval_us = 1000000L / target_fps;

	int fd = drm_open_card();
	struct drm_fbmap prim;
	struct drm_fbmap ov;

	if (fd < 0 || drm_map_scanout(fd, &prim))
		return 1;

	uint32_t *P  = prim.px;
	uint32_t  W  = prim.width;
	uint32_t  H  = prim.height;
	uint32_t  psp = prim.pitch / 4;
	uint32_t  bg = 0x00101828;

	for (size_t i = 0; i < prim.size / 4; i++)
		P[i] = bg;
	drm_clean(P, prim.size);

	uint32_t crtc  = drm_crtc(fd);
	uint32_t plane = drm_find_overlay(fd);

	if (!plane || drm_make_argb4444(fd, W, H, &ov)) {
		fprintf(stderr, "no overlay plane / buffer\n");
		return 1;
	}

	uint16_t *O   = ov.px;
	uint32_t  osp = ov.pitch / 2;

	for (size_t i = 0; i < ov.size / 2; i++)
		O[i] = 0;

	drm_clean(O, ov.size);
	drm_setplane(fd, plane, crtc, &ov);

	/* HUD geometry: two lines of text at scale 4. */
	int tx = 48, ty = 40, sc = 4;
	int th     = 8 * sc;                /* glyph height in pixels */
	int tw1    = 12 * 8 * sc;           /* "FRAME 000000" width */
	int tw2    = 7  * 8 * sc;           /* "FPS 060" width */
	int ty2    = ty + th + 4;           /* second line Y */
	int hud_y0 = ty - 8;
	int hud_h  = (ty2 + th + 8) - hud_y0;
	int hud_w  = (tw1 > tw2 ? tw1 : tw2) + 16;

	/* Animation. */
	int lw = 150, lh = 90;
	int x = 220, y = 160, vx = 7, vy = 5, ox = x, oy = y;
	uint32_t pal[] = {0x00ff5a5a, 0x0050e0a0, 0x005aa0ff, 0x00ffcf50, 0x00e070ff, 0x0050e0e0};
	int ci = 0;
	unsigned long frame = 0;

	/* FPS measurement: rolling average over 16 frames. The ring must be pre-filled with the
	 * same ideal frame time ns_total assumes, or the phantom baseline never drains and the
	 * displayed FPS reads exactly half forever.
	 */
	long ns_ring[16];
	int  ns_head = 0;
	long ns_total = (1000000000L / target_fps) * 16;

	for (int i = 0; i < 16; i++)
		ns_ring[i] = 1000000000L / target_fps;

	int  fps_disp = target_fps;
	struct timespec t_frame;

	clock_gettime(CLOCK_MONOTONIC, &t_frame);

	for (;;) {
		struct timespec t_prev = t_frame;

		/* Primary: erase old square, draw new one, dirty-rect clean. */
		for (int j = 0; j < lh; j++)
			for (int i = 0; i < lw; i++)
				P[(size_t)(oy + j) * psp + ox + i] = bg;

		for (int j = 0; j < lh; j++)
			for (int i = 0; i < lw; i++)
				P[(size_t)(y + j) * psp + x + i] = pal[ci];

		int y0 = oy < y ? oy : y;
		int y1 = (oy > y ? oy : y) + lh;

		drm_clean(P + (size_t)y0 * psp, (size_t)(y1 - y0) * prim.pitch);
		ox = x; oy = y;

		/* Overlay HUD: clear, draw frame counter and FPS. */
		for (int j = 0; j < hud_h; j++)
			for (int i = 0; i < hud_w; i++)
				O[(size_t)(hud_y0 + j) * osp + (tx - 8 + i)] = 0x8000;

		char s1[32], s2[32];

		snprintf(s1, sizeof(s1), "FRAME %06lu", frame % 1000000);
		snprintf(s2, sizeof(s2), "FPS %03d", fps_disp > 999 ? 999 : fps_disp);

		for (int k = 0; s1[k]; k++)
			gl16(O, osp, tx + k * 8 * sc, ty,  sc, 0xffff, glyph(s1[k]));

		for (int k = 0; s2[k]; k++)
			gl16(O, osp, tx + k * 8 * sc, ty2, sc, 0xffff, glyph(s2[k]));

		drm_clean((char *)O + (size_t)hud_y0 * ov.pitch, (size_t)hud_h * ov.pitch);

		/* Physics. */
		x += vx; y += vy;
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
		frame++;

		/* Adaptive sleep to approach the target FPS. */
		struct timespec t_now;

		clock_gettime(CLOCK_MONOTONIC, &t_now);
		long render_ns = ts_diff_ns(t_prev, t_now);
		long sleep_us  = frame_interval_us - render_ns / 1000;

		if (sleep_us > 0)
			usleep((unsigned long)sleep_us);

		/* Update rolling FPS measurement. */
		clock_gettime(CLOCK_MONOTONIC, &t_frame);
		long frame_ns = ts_diff_ns(t_prev, t_frame);

		ns_total -= ns_ring[ns_head];
		ns_ring[ns_head] = frame_ns;
		ns_total += frame_ns;
		ns_head = (ns_head + 1) % 16;
		fps_disp = (int)(16000000000LL / ns_total);
	}
}
