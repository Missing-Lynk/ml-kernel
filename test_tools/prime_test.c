// SPDX-License-Identifier: GPL-2.0
/*
 * prime_test - validate the artosyn_vo PRIME/dma-buf import path.
 * The decode-to-scanout contract is "the VO primary scans a contiguous I420 buffer some
 * other subsystem allocated, zero-copy". The open decoder isn't producing frames yet, so
 * this proves the display half with decoder-shaped buffers:
 *
 *   prime_test            # self round-trip: dumb buffer -> PRIME export -> mmap via the
 *                         #   dma-buf fd -> re-import -> ADDFB2 -> scanout. Runs on any
 *                         #   kernel with the driver loaded. (Same-device import resolves
 *                         #   to the same GEM object; it proves export + fb-from-imported-
 *                         #   handle, not the cross-exporter attach path.)
 *   prime_test --heap     # the cross-exporter proof: allocate the I420 buffer from the CMA
 *                         #   DMA-BUF heap (under /dev/dma_heap, a foreign exporter, contiguous
 *                         #   like a vb2/decoder buffer), PRIME-import it into the DRM
 *                         #   device (dma_buf attach/map + contiguity check), ADDFB2 the
 *                         #   imported handle as DRM_FORMAT_YUV420, scan it out.
 *                         #   Needs CONFIG_DMABUF_HEAPS_CMA (configs/display.config).
 *
 * Success = colour bars with a moving diagonal marker band on the panel (distinct from
 * i420_test's static bars, so the two tests can't be confused). Ctrl-C to exit.
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
#include <dirent.h>
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
#define BUF_SIZE (V_OFF + CSTRIDE * (H / 2))

/* DMA-BUF heap + sync UAPI, defined locally so the build does not depend on the host's
 * kernel headers being new enough. Layouts match include/uapi/linux/dma-heap.h and
 * include/uapi/linux/dma-buf.h.
 */
struct dma_heap_allocation_data {
	uint64_t len;
	uint32_t fd;
	uint32_t fd_flags;
	uint64_t heap_flags;
};
#define DMA_HEAP_IOCTL_ALLOC	_IOWR('H', 0x0, struct dma_heap_allocation_data)

struct dma_buf_sync {
	uint64_t flags;
};
#define DMA_BUF_SYNC_WRITE	(2 << 0)
#define DMA_BUF_SYNC_START	(0 << 2)
#define DMA_BUF_SYNC_END	(1 << 2)
#define DMA_BUF_IOCTL_SYNC	_IOW('b', 0, struct dma_buf_sync)

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

/* 8 colour bars plus a white diagonal band, so this test is visually distinct. */
static void fill_pattern(uint8_t *buf)
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

			if (((x + y) / 64) % 8 == 0)		/* diagonal marker band */
				c = bars[0];

			rgb_to_yuv(c[0], c[1], c[2], &yy, &uu, &vv);
			yp[y * LSTRIDE + x] = yy;
			if (!(y & 1) && !(x & 1)) {
				up[(y / 2) * CSTRIDE + (x / 2)] = uu;
				vp[(y / 2) * CSTRIDE + (x / 2)] = vv;
			}
		}
	}
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

/* Open the first /dev/dma_heap/ device and allocate a len-byte dma-buf from it. */
static int heap_alloc(size_t len)
{
	struct dma_heap_allocation_data alloc = { .len = len, .fd_flags = O_RDWR | O_CLOEXEC };
	char path[280];
	struct dirent *de;
	DIR *d = opendir("/dev/dma_heap");
	int hfd = -1;

	if (!d) {
		perror("/dev/dma_heap (CONFIG_DMABUF_HEAPS_CMA kernel needed)");
		return -1;
	}

	while ((de = readdir(d))) {
		if (de->d_name[0] == '.')
			continue;

		snprintf(path, sizeof(path), "/dev/dma_heap/%s", de->d_name);
		hfd = open(path, O_RDWR | O_CLOEXEC);
		if (hfd >= 0) {
			printf("heap: %s\n", path);
			break;
		}
	}
	closedir(d);

	if (hfd < 0) {
		fprintf(stderr, "no usable heap under /dev/dma_heap\n");
		return -1;
	}

	if (ioctl(hfd, DMA_HEAP_IOCTL_ALLOC, &alloc)) {
		perror("DMA_HEAP_IOCTL_ALLOC");
		close(hfd);
		return -1;
	}
	close(hfd);

	return alloc.fd;
}

int main(int argc, char **argv)
{
	int use_heap = argc > 1 && !strcmp(argv[1], "--heap");
	struct drm_mode_fb_cmd2 fb = {0};
	struct drm_mode_modeinfo mode;
	struct drm_mode_crtc set = {0};
	uint32_t crtc, conn, conn_list[1];
	int fd, buf_fd;
	uint8_t *buf;

	setvbuf(stdout, NULL, _IOLBF, 0);	/* keep progress visible when logged to a file */

	fd = drm_open_card();
	if (fd < 0)
		return 1;

	/* 1. The driver must advertise PRIME import+export. */
	struct drm_get_cap cap = { .capability = DRM_CAP_PRIME };

	if (ioctl(fd, DRM_IOCTL_GET_CAP, &cap)) {
		perror("GET_CAP(PRIME)");
		return 1;
	}

	printf("DRM_CAP_PRIME = 0x%llx (import:%s export:%s)\n",
	       (unsigned long long)cap.value,
	       cap.value & DRM_PRIME_CAP_IMPORT ? "yes" : "NO",
	       cap.value & DRM_PRIME_CAP_EXPORT ? "yes" : "NO");
	if (!(cap.value & DRM_PRIME_CAP_IMPORT)) {
		fprintf(stderr, "no PRIME import: P2 fails here\n");
		return 1;
	}

	crtc = drm_crtc(fd);
	if (!crtc || find_connector(fd, &conn, &mode)) {
		fprintf(stderr, "no CRTC/connector\n");
		return 1;
	}

	/* 2. Obtain the I420 buffer as a dma-buf fd, from a foreign exporter (--heap)
	 * or from our own dumb buffer (self round-trip).
	 */
	if (use_heap) {
		buf_fd = heap_alloc(BUF_SIZE);
		if (buf_fd < 0)
			return 1;
	} else {
		struct drm_mode_create_dumb create = { .width = W, .height = H + H / 2, .bpp = 8 };
		struct drm_prime_handle exp = { .flags = O_RDWR | O_CLOEXEC };

		if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &create)) {
			perror("create dumb");
			return 1;
		}

		exp.handle = create.handle;
		if (ioctl(fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &exp)) {
			perror("PRIME_HANDLE_TO_FD (export)");
			return 1;
		}

		printf("exported dumb handle %u as dma-buf fd %d\n", create.handle, exp.fd);
		buf_fd = exp.fd;
	}

	/* 3. Fill it through the dma-buf fd's own mmap, with a CPU-access sync bracket
	 * (the CMA heap maps cached; the sync END flushes to DDR for the non-snooping DC).
	 */
	buf = mmap(0, BUF_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, buf_fd, 0);
	if (buf == MAP_FAILED) {
		perror("mmap dma-buf fd");
		return 1;
	}

	struct dma_buf_sync sync = { .flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE };

	ioctl(buf_fd, DMA_BUF_IOCTL_SYNC, &sync);	/* advisory on GEM; required on heap */
	fill_pattern(buf);
	sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
	if (ioctl(buf_fd, DMA_BUF_IOCTL_SYNC, &sync))
		drm_clean(buf, BUF_SIZE);		/* GEM path: no sync ioctl, flush by hand */

	/* 4. PRIME-import the dma-buf into the DRM device. */
	struct drm_prime_handle imp = { .fd = buf_fd };

	if (ioctl(fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &imp)) {
		perror("PRIME_FD_TO_HANDLE (import; non-contiguous or attach failure?)");
		return 1;
	}

	printf("imported dma-buf fd %d as GEM handle %u\n", buf_fd, imp.handle);

	/* 5. Wrap the imported handle as an I420 framebuffer and scan it out. */
	fb.width = W;
	fb.height = H;
	fb.pixel_format = DRM_FORMAT_YUV420;
	fb.handles[0] = fb.handles[1] = fb.handles[2] = imp.handle;
	fb.pitches[0] = LSTRIDE;
	fb.pitches[1] = CSTRIDE;
	fb.pitches[2] = CSTRIDE;
	fb.offsets[0] = 0;
	fb.offsets[1] = U_OFF;
	fb.offsets[2] = V_OFF;
	if (ioctl(fd, DRM_IOCTL_MODE_ADDFB2, &fb)) {
		perror("addfb2 on imported handle");
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

	printf("PRIME %s buffer scanning out as I420 (bars + diagonal band). Ctrl-C to exit.\n",
	       use_heap ? "heap-imported" : "self-round-trip");
	pause();
	return 0;
}
