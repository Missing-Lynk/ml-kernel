// SPDX-License-Identifier: GPL-2.0
/*
 * i420_test - display a full-screen I420 (YUV420 planar) frame on the primary plane, to
 * validate the artosyn_vo driver's native YUV scanout. Independent of the video decoder:
 * it feeds the DC a static I420 buffer the same way a decoded frame would.
 *
 *   i420_test              # 100% colour bars (tests luma, chroma, and the YUV->RGB CSC)
 *   i420_test frame.i420   # display a raw 1920x1080 I420 file (e.g. a captured frame)
 *
 * The DC scans 3-plane I420 (format enum 15) with a 2048-px luma stride (1024 chroma), the
 * layout recovered from stock. We allocate one dumb buffer
 * holding Y (2048x1080) then U then V (1024x540 each), ADDFB2 it as DRM_FORMAT_YUV420 with the
 * three plane offsets, and modeset it onto the CRTC (a full modeset drives the driver's YUV
 * enable path, format + strides + CSC). No 180 pre-rotation: the panel scans mirrored H+V via
 * its DT scan-direction straps, so content is fed upright.
 *
 * Build: linked with drm_util.c by the Makefile (raw DRM ioctls, static aarch64).
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <drm/drm_fourcc.h>

#include "drm_util.h"

#define W	1920
#define H	1080
#define LSTRIDE	2048		/* luma stride the DC wants (ALIGN(1920,512)) */
#define CSTRIDE	960		/* chroma stride = W/2; DC hardcodes this, ignoring VO_U/V_STRIDE */
#define Y_SIZE	(LSTRIDE * H)
#define U_OFF	Y_SIZE
#define V_OFF	(U_OFF + CSTRIDE * (H / 2))
#define BUF_SIZE (V_OFF + CSTRIDE * (H / 2))	/* Y + U + V actual data size */

/* BT.709 full-range RGB->YUV (matches the CSC the driver programs for the primary). */
static void rgb_to_yuv(int r, int g, int b, uint8_t *y, uint8_t *u, uint8_t *v)
{
	int yy = (2126 * r + 7152 * g + 722 * b) / 10000;
	int uu = (-1146 * r - 3854 * g + 5000 * b) / 10000 + 128;
	int vv = (5000 * r - 4542 * g - 458 * b) / 10000 + 128;

	*y = yy < 0 ? 0 : yy > 255 ? 255 : yy;
	*u = uu < 0 ? 0 : uu > 255 ? 255 : uu;
	*v = vv < 0 ? 0 : vv > 255 ? 255 : vv;
}

/* Fill the mapped buffer with 8 vertical 100%-colour bars. */
static void fill_colour_bars(uint8_t *buf)
{
	static const int bars[8][3] = {
		{ 255, 255, 255 }, { 255, 255, 0 }, { 0, 255, 255 }, { 0, 255, 0 },
		{ 255, 0, 255 },   { 255, 0, 0 },   { 0, 0, 255 },   { 0, 0, 0 },
	};
	uint8_t *yp = buf;
	uint8_t *up = buf + U_OFF;
	uint8_t *vp = buf + V_OFF;

	for (int y = 0; y < H; y++) {
		for (int x = 0; x < W; x++) {
			uint8_t yy, uu, vv;
			const int *c = bars[x * 8 / W];

			rgb_to_yuv(c[0], c[1], c[2], &yy, &uu, &vv);
			yp[y * LSTRIDE + x] = yy;
			if (!(y & 1) && !(x & 1)) {
				up[(y / 2) * CSTRIDE + (x / 2)] = uu;
				vp[(y / 2) * CSTRIDE + (x / 2)] = vv;
			}
		}
	}
}

/* Load a tightly-packed 1920x1080 I420 file, re-striding each plane into the DC layout. */
static int load_i420_file(const char *path, uint8_t *buf)
{
	int fd = open(path, O_RDONLY);
	uint8_t row[W];

	if (fd < 0) {
		perror("open i420 file");
		return -1;
	}

	for (int y = 0; y < H; y++) {			/* Y: 1920 -> 2048 stride */
		if (read(fd, row, W) != W)
			goto short_read;
		memcpy(buf + y * LSTRIDE, row, W);
	}

	for (int y = 0; y < H / 2; y++) {		/* U: 960 -> 1024 stride */
		if (read(fd, row, W / 2) != W / 2)
			goto short_read;
		memcpy(buf + U_OFF + y * CSTRIDE, row, W / 2);
	}

	for (int y = 0; y < H / 2; y++) {		/* V: 960 -> 1024 stride */
		if (read(fd, row, W / 2) != W / 2)
			goto short_read;
		memcpy(buf + V_OFF + y * CSTRIDE, row, W / 2);
	}

	close(fd);
	return 0;

short_read:
	fprintf(stderr, "%s: short read (need a 1920x1080 I420 file)\n", path);
	close(fd);
	return -1;
}

/* Find a connected connector and its first mode (for the modeset). Returns 0 on success. */
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

		gc.modes_ptr = (uint64_t)modes;
		gc.count_modes = gc.count_modes > 32 ? 32 : gc.count_modes;
		gc.count_props = gc.count_encoders = 0;
		if (ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &gc) || !gc.count_modes)
			continue;

		*conn_out = (uint32_t)conns[i];
		*mode_out = modes[0];
		return 0;
	}

	return -1;
}

int main(int argc, char **argv)
{
	struct drm_mode_create_dumb create = { .width = W, .height = H + H / 2, .bpp = 8 };
	struct drm_mode_map_dumb mreq = {0};
	struct drm_mode_fb_cmd2 fb = {0};
	struct drm_mode_modeinfo mode;
	struct drm_mode_crtc set = {0};
	uint32_t crtc, conn, conn_list[1];
	uint8_t *buf;
	int fd;

	fd = drm_open_card();
	if (fd < 0)
		return 1;

	crtc = drm_crtc(fd);
	if (!crtc || find_connector(fd, &conn, &mode)) {
		fprintf(stderr, "no CRTC/connector\n");
		return 1;
	}

	if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &create)) {
		perror("create dumb");
		return 1;
	}

	/* The driver pads pitch to 2048; the dumb buffer is then 2048 * 1620 bytes. */
	if (create.size < BUF_SIZE) {
		fprintf(stderr, "dumb buffer too small: %llu < %d\n",
			(unsigned long long)create.size, BUF_SIZE);
		return 1;
	}

	mreq.handle = create.handle;
	if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq)) {
		perror("map dumb");
		return 1;
	}

	buf = mmap(0, create.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mreq.offset);
	if (buf == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	memset(buf, 0, create.size);
	if (argc > 1) {
		if (load_i420_file(argv[1], buf))
			return 1;
	} else {
		fill_colour_bars(buf);
	}
	drm_clean(buf, BUF_SIZE);	/* push pixels to DDR; the DC does not snoop the cache */

	fb.width = W;
	fb.height = H;
	fb.pixel_format = DRM_FORMAT_YUV420;
	fb.handles[0] = fb.handles[1] = fb.handles[2] = create.handle;
	fb.pitches[0] = LSTRIDE;
	fb.pitches[1] = CSTRIDE;
	fb.pitches[2] = CSTRIDE;
	fb.offsets[0] = 0;
	fb.offsets[1] = U_OFF;
	fb.offsets[2] = V_OFF;
	if (ioctl(fd, DRM_IOCTL_MODE_ADDFB2, &fb)) {
		perror("addfb2 (does the driver advertise DRM_FORMAT_YUV420?)");
		return 1;
	}

	conn_list[0] = conn;
	set.crtc_id = crtc;
	set.fb_id = fb.fb_id;
	set.set_connectors_ptr = (uint64_t)conn_list;
	set.count_connectors = 1;
	set.mode_valid = 1;
	set.mode = mode;
	if (ioctl(fd, DRM_IOCTL_MODE_SETCRTC, &set)) {
		perror("setcrtc");
		return 1;
	}

	printf("I420 frame on the primary plane (%s). Ctrl-C to exit.\n",
	       argc > 1 ? argv[1] : "colour bars");
	pause();

	return 0;
}
