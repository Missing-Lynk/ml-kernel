// SPDX-License-Identifier: GPL-2.0
/* yuv_demo - animated YUV420 primary plane with HUD in the ARGB4444 overlay.
 *   PRIMARY plane : a colour-cycling rectangle bouncing on a mid-gray background,
 *                   DOUBLE-BUFFERED I420 with vsync page flips (native YUV scanout, P1,
 *                   plus the real-vblank flip path). Rendering goes to the back buffer
 *                   only, so the DC never scans a half-drawn frame (no flicker/tear).
 *   OVERLAY plane : live "FRAME NNNNNN / FPS NNN" drawn in ARGB4444 (single-buffered;
 *                   the HUD delta per frame is small enough not to visibly tear).
 * Exercises both planes, page-flip + vblank-event pacing, and per-pixel alpha.
 *
 * Usage: yuv_demo [fps]   default fps = 60; the flip loop is vsync-locked, so values
 *                         above the panel's 60 Hz still render 60.
 * Run after: echo 0 > /sys/class/vtconsole/vtcon1/bind
 *
 * Buffer layout: LSTRIDE=2048 (luma), CSTRIDE=960 (chroma = W/2, tight).
 * The DC hardcodes chroma stride = W/2 and ignores VO_U/V_STRIDE (confirmed by
 * i420_test diagnosis: CSTRIDE=1024 produced a 64-byte/row diagonal drift).
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <drm/drm_fourcc.h>
#include "drm_util.h"
#include "drm_glyphs.h"

#define W	1920
#define H	1080
#define LSTRIDE	2048
#define CSTRIDE	960
#define Y_SIZE	(LSTRIDE * H)
#define U_OFF	Y_SIZE
#define V_OFF	(U_OFF + CSTRIDE * (H / 2))
#define BUF_SIZE (V_OFF + CSTRIDE * (H / 2))

/* Background: mid-dark neutral gray. */
#define BG_Y	80
#define BG_U	128
#define BG_V	128

/* 6-entry YUV palette, BT.709 full-range.  Colors: red, green, blue, yellow, magenta, cyan. */
static const uint8_t pal_y[6] = {  88, 157,  99, 208,  84, 164 };
static const uint8_t pal_u[6] = { 107,  70, 199,  32, 201, 148 };
static const uint8_t pal_v[6] = { 218,  60,  97, 142, 214,  43 };

static long ts_diff_ns(struct timespec a, struct timespec b)
{
	return (b.tv_sec - a.tv_sec) * 1000000000L + (b.tv_nsec - a.tv_nsec);
}

static int find_connector(int fd, uint32_t *conn_out, struct drm_mode_modeinfo *mode_out)
{
	struct drm_mode_card_res res = {0};
	uint64_t conns[32];

	ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res);
	res.connector_id_ptr = (uint64_t)conns;
	res.count_connectors = res.count_connectors > 32 ? 32 : res.count_connectors;
	res.count_crtcs = res.count_encoders = res.count_fbs = 0;
	if (ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res))
		return -1;

	for (uint32_t i = 0; i < res.count_connectors; i++) {
		struct drm_mode_get_connector gc = { .connector_id = (uint32_t)conns[i] };
		struct drm_mode_modeinfo modes[32];

		ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &gc);
		if (gc.connection != 1 || !gc.count_modes)
			continue;

		gc.modes_ptr    = (uint64_t)modes;
		gc.count_modes  = gc.count_modes > 32 ? 32 : gc.count_modes;
		gc.count_props  = gc.count_encoders = 0;
		if (ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &gc) || !gc.count_modes)
			continue;

		*conn_out = (uint32_t)conns[i];
		*mode_out = modes[0];
		return 0;
	}

	return -1;
}

static void yuv_fill_bg(uint8_t *ybuf, uint8_t *ubuf, uint8_t *vbuf)
{
	memset(ybuf, BG_Y, LSTRIDE * H);
	memset(ubuf, BG_U, CSTRIDE * (H / 2));
	memset(vbuf, BG_V, CSTRIDE * (H / 2));
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

	struct drm_mode_fb_cmd2     fb[2];
	struct drm_mode_modeinfo    mode;
	struct drm_mode_crtc        set = {0};
	uint32_t crtc, conn, conn_list[1];
	uint8_t *buf[2];
	struct drm_fbmap ovl = {0};
	uint32_t plane = 0;
	int has_overlay = 0;
	int fd;

	printf("yuv_demo: starting (target %d fps)\n", target_fps);
	fflush(stdout);

	fd = drm_open_card();
	if (fd < 0)
		return 1;

	crtc = drm_crtc(fd);
	if (!crtc) {
		fprintf(stderr, "yuv_demo: no CRTC\n");
		return 1;
	}

	if (find_connector(fd, &conn, &mode)) {
		fprintf(stderr, "yuv_demo: no connected connector with modes\n");
		return 1;
	}

	printf("yuv_demo: CRTC %u connector %u mode %ux%u\n",
	       crtc, conn, mode.hdisplay, mode.vdisplay);
	fflush(stdout);

	/* Two I420 buffers: render into the back one, page-flip, repeat. */
	for (int i = 0; i < 2; i++) {
		struct drm_mode_create_dumb cd = { .width = W, .height = H + H / 2, .bpp = 8 };
		struct drm_mode_map_dumb mreq = {0};

		if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &cd)) {
			perror("yuv_demo: create_dumb");
			return 1;
		}

		if (cd.size < BUF_SIZE) {
			fprintf(stderr, "yuv_demo: dumb buffer too small: %llu < %d\n",
					(unsigned long long)cd.size, BUF_SIZE);
			return 1;
		}

		mreq.handle = cd.handle;
		if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq)) {
			perror("yuv_demo: map_dumb");
			return 1;
		}

		buf[i] = mmap(0, cd.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mreq.offset);
		if (buf[i] == MAP_FAILED) {
			perror("yuv_demo: mmap");
			return 1;
		}

		yuv_fill_bg(buf[i], buf[i] + U_OFF, buf[i] + V_OFF);
		drm_clean(buf[i], BUF_SIZE);

		memset(&fb[i], 0, sizeof(fb[i]));
		fb[i].width        = W;
		fb[i].height       = H;
		fb[i].pixel_format = DRM_FORMAT_YUV420;
		fb[i].handles[0]   = fb[i].handles[1] = fb[i].handles[2] = cd.handle;
		fb[i].pitches[0]   = LSTRIDE;
		fb[i].pitches[1]   = CSTRIDE;
		fb[i].pitches[2]   = CSTRIDE;
		fb[i].offsets[1]   = U_OFF;
		fb[i].offsets[2]   = V_OFF;
		if (ioctl(fd, DRM_IOCTL_MODE_ADDFB2, &fb[i])) {
			perror("yuv_demo: addfb2");
			return 1;
		}
	}

	conn_list[0]            = conn;
	set.crtc_id             = crtc;
	set.fb_id               = fb[0].fb_id;
	set.set_connectors_ptr  = (uint64_t)conn_list;
	set.count_connectors    = 1;
	set.mode_valid          = 1;
	set.mode                = mode;
	if (ioctl(fd, DRM_IOCTL_MODE_SETCRTC, &set)) {
		perror("yuv_demo: setcrtc");
		return 1;
	}

	printf("yuv_demo: YUV420 primary active\n");
	fflush(stdout);

	plane = drm_find_overlay(fd);
	if (plane && !drm_make_argb4444(fd, W, H, &ovl)) {
		memset(ovl.px, 0, ovl.size);
		drm_clean(ovl.px, ovl.size);
		if (drm_setplane(fd, plane, crtc, &ovl) == 0) {
			has_overlay = 1;
			printf("yuv_demo: overlay active\n");
		}
	}

	if (!has_overlay)
		printf("yuv_demo: no overlay (HUD disabled)\n");
	fflush(stdout);

	uint16_t *O   = has_overlay ? ovl.px : NULL;
	uint32_t  osp = has_overlay ? ovl.pitch / 2 : 0;

	/* HUD geometry: two text lines at scale 4. */
	int tx = 48, ty = 40, sc = 4;
	int th     = 8 * sc;
	int tw1    = 12 * 8 * sc;           /* "FRAME 000000" */
	int tw2    = 7  * 8 * sc;           /* "FPS 060"      */
	int ty2    = ty + th + 4;
	int hud_y0 = ty - 8;
	int hud_h  = (ty2 + th + 8) - hud_y0;
	int hud_w  = (tw1 > tw2 ? tw1 : tw2) + 16;

	/* Animation state. */
	int bx = 240, by = 180, vx = 7, vy = 5;
	int bw = 360, bh = 240;
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

	printf("yuv_demo: entering loop\n");
	fflush(stdout);

	int back = 1;               /* fb[0] is on the CRTC; draw into fb[1] first */

	for (;;) {
		struct timespec t_prev = t_frame;
		uint8_t *ybuf = buf[back];
		uint8_t *ubuf = buf[back] + U_OFF;
		uint8_t *vbuf = buf[back] + V_OFF;

		/* Primary (back buffer): background fill then draw the coloured box. */
		yuv_fill_bg(ybuf, ubuf, vbuf);
		uint8_t box_y = pal_y[ci], box_u = pal_u[ci], box_v = pal_v[ci];

		for (int y = by; y < by + bh; y++)
			memset(ybuf + y * LSTRIDE + bx, box_y, bw);

		for (int y = by / 2; y < (by + bh) / 2; y++) {
			memset(ubuf + y * CSTRIDE + bx / 2, box_u, bw / 2);
			memset(vbuf + y * CSTRIDE + bx / 2, box_v, bw / 2);
		}
		drm_clean(buf[back], BUF_SIZE);

		/* Overlay HUD. */
		if (has_overlay) {
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

			drm_clean((char *)O + (size_t)hud_y0 * ovl.pitch, (size_t)hud_h * ovl.pitch);
		}

		/* Physics. */
		bx += vx; by += vy;
		if (bx <= 0 || bx + bw >= W) {
			vx = -vx;
			bx += vx;
			ci = (ci + 1) % 6;
		}

		if (by <= 0 || by + bh >= H) {
			vy = -vy;
			by += vy;
			ci = (ci + 1) % 6;
		}

		frame++;

		/* Queue the back buffer on the vsync shadow latch and block until the flip
		 * completes (the driver arms the event on the real VSYNC IRQ). This is what
		 * makes the demo tear-free: the front buffer is never written.
		 */
		struct drm_mode_crtc_page_flip flip = {
			.crtc_id = crtc,
			.fb_id   = fb[back].fb_id,
			.flags   = DRM_MODE_PAGE_FLIP_EVENT,
		};

		if (ioctl(fd, DRM_IOCTL_MODE_PAGE_FLIP, &flip)) {
			perror("yuv_demo: page_flip");
			return 1;
		}

		char ev[128];

		if (read(fd, ev, sizeof(ev)) <= 0) {     /* blocks until FLIP_COMPLETE */
			perror("yuv_demo: read flip event");
			return 1;
		}

		back ^= 1;

		/* Adaptive sleep for targets below the panel rate (above it, the flip wait
		 * above already vsync-locks the loop).
		 */
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
