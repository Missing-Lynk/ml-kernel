// SPDX-License-Identifier: GPL-2.0
/* drm_util - shared cache-maintenance and DRM/KMS helpers for the display test tools. */
#include "drm_util.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <drm/drm_fourcc.h>

void drm_clean(void *p, size_t len)
{
	/*
	 * dc cvac, <addr>  Data Cache Clean by VA to Point of Coherency: write the dirty cache
	 *                  line at that VA back to DDR. One cache line is 64 bytes.
	 * dsb sy           Data Synchronisation Barrier: block until the cleans have reached DDR.
	 * volatile + "memory" stop the compiler reordering or dropping the ops.
	 */
	uintptr_t a = (uintptr_t)p & ~63UL;
	uintptr_t e = (uintptr_t)p + len;

	for (; a < e; a += 64)
		asm volatile("dc cvac, %0" :: "r"(a) : "memory");
	asm volatile("dsb sy" ::: "memory");
}

int drm_open_card(void)
{
	int fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);

	if (fd < 0) {
		perror("open /dev/dri/card0");
		return -1;
	}

	ioctl(fd, DRM_IOCTL_SET_MASTER, 0);
	return fd;
}

uint32_t drm_crtc(int fd)
{
	struct drm_mode_card_res res = {0};
	uint64_t cr[16];

	ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res);
	res.crtc_id_ptr = (uint64_t)cr;
	res.count_crtcs = res.count_crtcs > 16 ? 16 : res.count_crtcs;
	res.count_connectors = res.count_encoders = res.count_fbs = 0;
	if (ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res) || !res.count_crtcs)
		return 0;

	return (uint32_t)cr[0];
}

int drm_map_scanout(int fd, struct drm_fbmap *out)
{
	uint32_t crtc = drm_crtc(fd);
	struct drm_mode_crtc gc = { .crtc_id = crtc };
	struct drm_mode_fb_cmd gf = {0};
	struct drm_mode_map_dumb md = {0};
	void *p;

	if (!crtc)
		return -1;

	if (ioctl(fd, DRM_IOCTL_MODE_GETCRTC, &gc) || !gc.fb_id) {
		fprintf(stderr, "crtc has no current fb\n");
		return -1;
	}

	gf.fb_id = gc.fb_id;
	if (ioctl(fd, DRM_IOCTL_MODE_GETFB, &gf) || !gf.handle) {
		perror("getfb");
		return -1;
	}

	md.handle = gf.handle;
	ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &md);
	out->size = (size_t)gf.pitch * gf.height;

	p = mmap(0, out->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, md.offset);
	if (p == MAP_FAILED) {
		perror("mmap");
		return -1;
	}

	out->px = p;
	out->width = gf.width;
	out->height = gf.height;
	out->pitch = gf.pitch;
	out->fb_id = gf.fb_id;
	out->handle = gf.handle;

	return 0;
}

uint32_t drm_find_overlay(int fd)
{
	struct drm_mode_get_plane_res pr = {0};
	uint64_t pl[16];

	ioctl(fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &pr);
	pr.plane_id_ptr = (uint64_t)pl;
	pr.count_planes = pr.count_planes > 16 ? 16 : pr.count_planes;
	ioctl(fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &pr);

	for (uint32_t i = 0; i < pr.count_planes; i++) {
		struct drm_mode_get_plane gp = { .plane_id = pl[i] };
		uint32_t ft[32];

		ioctl(fd, DRM_IOCTL_MODE_GETPLANE, &gp);
		gp.format_type_ptr = (uint64_t)ft;
		gp.count_format_types = gp.count_format_types > 32 ? 32 : gp.count_format_types;
		ioctl(fd, DRM_IOCTL_MODE_GETPLANE, &gp);
		for (uint32_t j = 0; j < gp.count_format_types; j++) {
			if (ft[j] == DRM_FORMAT_ARGB4444)
				return (uint32_t)pl[i];
		}
	}

	return 0;
}

int drm_make_argb4444(int fd, uint32_t w, uint32_t h, struct drm_fbmap *out)
{
	struct drm_mode_create_dumb cd = { .width = w, .height = h, .bpp = 16 };
	struct drm_mode_map_dumb md = {0};
	struct drm_mode_fb_cmd2 fb = { .width = w, .height = h, .pixel_format = DRM_FORMAT_ARGB4444 };
	void *p;

	if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &cd)) {
		perror("create_dumb");
		return -1;
	}

	md.handle = cd.handle;
	ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &md);

	p = mmap(0, cd.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, md.offset);
	if (p == MAP_FAILED) {
		perror("mmap");
		return -1;
	}

	fb.handles[0] = cd.handle;
	fb.pitches[0] = cd.pitch;

	if (ioctl(fd, DRM_IOCTL_MODE_ADDFB2, &fb)) {
		perror("addfb2");
		return -1;
	}

	out->px = p;
	out->width = w;
	out->height = h;
	out->pitch = cd.pitch;
	out->size = cd.size;
	out->fb_id = fb.fb_id;
	out->handle = cd.handle;

	return 0;
}

int drm_setplane(int fd, uint32_t plane, uint32_t crtc, const struct drm_fbmap *fb)
{
	struct drm_mode_set_plane sp = {
		.plane_id = plane, .crtc_id = crtc, .fb_id = fb->fb_id,
		.crtc_w = fb->width, .crtc_h = fb->height,
		.src_w = fb->width << 16, .src_h = fb->height << 16,
	};

	if (ioctl(fd, DRM_IOCTL_MODE_SETPLANE, &sp)) {
		perror("setplane");
		return -1;
	}

	return 0;
}
