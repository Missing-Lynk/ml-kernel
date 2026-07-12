/* SPDX-License-Identifier: GPL-2.0 */
/*
 * drm_util - shared helpers for the artosyn_vo display test tools. This is userspace
 * test-tool support, NOT driver code: cache maintenance for the non-coherent DC plus the
 * common DRM/KMS plumbing (open the card, map the scanout framebuffer, drive the overlay
 * plane). Linked into the display tools by the Makefile.
 */
#ifndef DRM_UTIL_H
#define DRM_UTIL_H

#include <stdint.h>
#include <stddef.h>

/* A mapped framebuffer: CPU pointer + geometry + the DRM ids that own it. */
struct drm_fbmap {
	void    *px;      /* mmap base; cast to uint32_t* (XRGB8888) or uint16_t* (ARGB4444) */
	uint32_t width, height;
	uint32_t pitch;   /* bytes per row */
	size_t   size;    /* bytes */
	uint32_t fb_id;
	uint32_t handle;
};

/*
 * Flush the CPU cache to DDR for [p, p+len) so the non-coherent DC sees our writes. The DRM
 * dumb buffer is write-back cacheable but the DC reads straight from DDR without snooping the
 * cache, so drawn pixels must be pushed out or it scans stale data.
 *
 * Implemented as dc cvac per 64-byte line + dsb sy
 */
void drm_clean(void *p, size_t len);

/* Open /dev/dri/card0 and become DRM master. Returns the fd, or -1 on failure. */
int drm_open_card(void);

/* The (single) CRTC id, or 0 if none. */
uint32_t drm_crtc(int fd);

/* Map the CRTC's current scanout framebuffer (the fbdev fb). Returns 0 on success. */
int drm_map_scanout(int fd, struct drm_fbmap *out);

/* Plane id of the ARGB4444 overlay plane (found by advertised format), or 0 if none. */
uint32_t drm_find_overlay(int fd);

/*
 * Create a full-screen ARGB4444 dumb buffer, map it, and ADDFB2. On success out->px is the
 * mapped surface (use as uint16_t*) and out->fb_id is ready for drm_setplane. Returns 0 on
 * success. The buffer starts uninitialised; fill it (transparent = 0x0000) and drm_clean it.
 */
int drm_make_argb4444(int fd, uint32_t w, uint32_t h, struct drm_fbmap *out);

/* SETPLANE the fb onto the plane at (0,0), sized to the fb. Returns 0 on success. */
int drm_setplane(int fd, uint32_t plane, uint32_t crtc, const struct drm_fbmap *fb);

#endif /* DRM_UTIL_H */
