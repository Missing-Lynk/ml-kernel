// SPDX-License-Identifier: GPL-2.0
/*
 * ar_framebuffer.ko - open reimplementation of the Artosyn OSD framebuffer
 * (/dev/fb0): fb_mmz-backed allocation, a standard ARGB4444 fbdev, pan.
 *
 * ABI recovered byte-exact from the vendor .ko disassembly (geometry, params,
 * 'F' ioctls). Production load: width=1920 height=1080 format=4 number=3 ->
 * ARGB4444, stride 2048px (line_length 4096B), triple-buffered yres_virtual=3240.
 * Backing store from MMZ zone "fb_mmz". Consumers use only the generic fbdev ABI
 * (GET_V/FSCREENINFO, mmap WC, PAN) - all reproduced here.
 *
 * NOTE: the vendor gates register_framebuffer on the userspace `ar_overlay` CUSE
 * compositor and forwards every flip to it (QBuf cmds 0x4101/4104/4107/4108/4109)
 * - that is what actually reaches the panel. The inner ar_overlay protocol is
 * recovered from the vendor .ko disassembly; the struct definitions + call
 * sites are wired in below (see "ar_overlay" section). The kernel->CUSE
 * transport (opening /dev/ar_overlay from the kernel and issuing the ioctl
 * through the ar_osal CUSE client) is not implemented - ar_overlay_xfer() is a
 * clearly-marked stub for it (TODO) - so /dev/fb0 has no scanout path on the
 * open stack. register_framebuffer is deliberately NOT gated on the handshake
 * (unlike the vendor): the node + geometry + mmap + pan must come up for the
 * OSD renderer even with the transport stubbed.
 */
#define pr_fmt(fmt) "ar_framebuffer: " fmt

#include <linux/module.h>
#include <linux/fb.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/build_bug.h>
#include <linux/stddef.h>
#include <linux/types.h>

#include "ar_mmz.h"

static int width  = 1920;
static int height = 1080;
static int number = 3;
static int format;		/* default 0; production loads 4 */
static int stream = 1;
module_param(width,  int, 0);
module_param(height, int, 0);
module_param(number, int, 0);
module_param(format, int, 0);
module_param(stream, int, 0);

/* vendor 'F' ioctls (all _IOW) - cursor/overlay/flush, forwarded to ar_overlay. */
#define ARFB_SET_ICON	_IOW('F', 0x81, u32)	/* 0x40044681 */
#define ARFB_SET_CURSOR	_IOW('F', 0x82, u32)	/* 0x40044682 */
#define ARFB_OVERLAY_OP	_IOW('F', 0x83, u64)	/* 0x40084683 */
#define ARFB_FLUSH_ZONE	_IOW('F', 0x84, u64)	/* 0x40084684 {u32 line_off,line_num} */

/* --------------------------------------------------------------------- *
 *  ar_overlay CUSE compositor protocol
 *
 *  All recovered byte-exact from ar_framebuffer.ko (objdump). Every request is
 *  one cuse_dev_ioctl(fd, AR_OVERLAY_IOC, &env); env.ptr (+0x10) points at a
 *  per-command payload. See the doc for the PINNED-vs-INFERRED breakdown.
 * ---------------------------------------------------------------------
 */

/* Outer envelope. 0xc018410f = _IOWR('A', 0x0f, 24). Layout PINNED: cmd@0,
 * size@4, ptr@0x10; the 8 bytes at +0x08 are never written by the vendor.
 */
struct ar_overlay_env {
	u32	cmd;		/* +0x00 inner command */
	u32	size;		/* +0x04 payload size in bytes */
	u64	rsv;		/* +0x08 NOT written by vendor; server scratch (INFERRED) */
	u64	ptr;		/* +0x10 payload pointer (PINNED at +0x10) */
};
#define AR_OVERLAY_IOC		_IOWR('A', 0x0f, struct ar_overlay_env)	/* 0xc018410f */

/* Inner commands. Pipeline cmds carry the full IOC word (0xc01841NN); the OSD/
 * cursor cmds use the bare nr - see doc §1. (PINNED constants.)
 */
#define AR_OVL_SET_RES		0xc0184101u	/* payload struct ar_overlay_set_res (16B) */
#define AR_OVL_SET_FMT		0xc0184104u	/* payload u32 format enum (4B) */
#define AR_OVL_QBUF		0xc0184107u	/* payload struct ar_overlay_buf (208B) */
#define AR_OVL_DQBUF		0xc0184108u	/* payload struct ar_overlay_buf (208B, reused) */
#define AR_OVL_STREAM_ON	0xc0184109u	/* payload u32 = 1 (4B) */
#define AR_OVL_STREAM_OFF	0xc018410au	/* payload u32 = 1 (4B) */
#define AR_OVL_CURSOR_ICON	0x00001024u	/* payload = NUL-terminated path, strlen+1 */
#define AR_OVL_CURSOR_POS	0x00001025u	/* payload u32 = 0 (cursor disable; arg NULL) */
#define AR_OVL_CURSOR_EN	0x00001026u	/* payload u32 = 1 (cursor enable; arg set) */
#define AR_OVL_OP		0x00001028u	/* payload = user void* (8B blob) */

/* SET_RES payload, 16B. printk "set resolution w %d : h %d : y %d : chroma : %d". */
struct ar_overlay_set_res {
	u32	width;		/* +0x00 var.xres */
	u32	height;		/* +0x04 var.yres */
	u32	stride;		/* +0x08 fix.line_length (luma stride, bytes) */
	u32	chroma_stride;	/* +0x0c stride >> 1 */
};

/* QBUF/DQBUF buffer descriptor, 208B (0xd0). Only the PINNED fields below are
 * written; the rest are zeroed (INFERRED reserved / DQBUF return area). On DQBUF
 * the server writes the dequeued buffer's phys back into plane[0].phys.
 */
struct ar_overlay_plane {		/* 24B */
	u64	phys;		/* +0x00 plane physical base (PINNED) */
	u64	rsv0;		/* +0x08 INFERRED (size/offset/stride?) */
	u64	rsv1;		/* +0x10 INFERRED */
};
struct ar_overlay_buf {			/* 208B (0xd0) */
	u64	field0;		/* +0x00 = 0 (buffer id/timestamp? INFERRED) */
	u64	field1;		/* +0x08 = 0 (INFERRED) */
	u8	rsv_10[12];	/* +0x10 .. +0x1b not written (INFERRED) */
	u32	num_planes;	/* +0x1c 1 (packed RGB) or 3 (YUV420 planar) (PINNED) */
	struct ar_overlay_plane plane[3];	/* +0x20, +0x38, +0x50 (PINNED) */
	u8	rsv_68[104];	/* +0x68 .. +0xcf server scratch / DQBUF return (INFERRED) */
};
/* Layout asserts against the vendor disassembly offsets. */
static_assert(sizeof(struct ar_overlay_env) == 24);
static_assert(offsetof(struct ar_overlay_env, ptr) == 0x10);
static_assert(sizeof(struct ar_overlay_set_res) == 16);
static_assert(sizeof(struct ar_overlay_plane) == 24);
static_assert(offsetof(struct ar_overlay_buf, num_planes) == 0x1c);
static_assert(offsetof(struct ar_overlay_buf, plane) == 0x20);
static_assert(sizeof(struct ar_overlay_buf) == 0xd0);

struct arfb {
	struct fb_info	*info;
	struct mmb	*mmb;
	void		*screen;
	phys_addr_t	phys;
	size_t		smem_len;
	int		overlay_fd;	/* ar_overlay CUSE fd (par+0x00); -1 until transport exists */
	u32		fourcc;		/* V4L2 fourcc (par+0x10) */
};
static struct arfb fb;

/* --------------------------------------------------------------------- *
 *  ar_overlay forwarding shim
 *
 *  ar_overlay_xfer() is the single choke point that marshals an inner command
 *  into the 24-byte envelope and issues it to the compositor. The actual
 *  kernel->CUSE transport (cuse_dev_open("ar_overlay") + cuse_dev_ioctl via the
 *  ar_osal CUSE client) is NOT yet implemented in the open tree, so this is a
 *  documented STUB: it builds the correct envelope and returns success without
 *  transmitting. Wiring the real transport is then mechanical (see TODO).
 * ---------------------------------------------------------------------
 */
static int ar_overlay_xfer(int fd, u32 cmd, u32 size, void *payload)
{
	struct ar_overlay_env env = {
		.cmd  = cmd,
		.size = size,
		.ptr  = (u64)(uintptr_t)payload,
	};

	/* TODO(transport): when the ar_osal CUSE client is ported, replace the
	 * stub below with:
	 *     return cuse_dev_ioctl(fd, AR_OVERLAY_IOC, &env);
	 * and have ar_fb_init()/the probe kthread open "ar_overlay" into
	 * fb.overlay_fd (cuse_dev_open) the way the vendor does. Until then there
	 * is no compositor to talk to; the OSD surface is still mappable/pannable
	 * via the generic fbdev ABI.
	 */
	(void)env;
	(void)fd;
	return 0;	/* STUB: pretend the compositor accepted the request */
}

/* Map an fb `format` param to the V4L2 fourcc the overlay expects.
 * Returns 0 for an unmapped format.
 */
static u32 ar_fb_fourcc(int fmt)
{
	switch (fmt) {
	case 0: {
		return 0x34325241;	/* "AR24" ARGB8888 */
	}
	case 1: {
		return 0x33524742;	/* "BGR3" RGB888 (overlay rejects in SET_RES) */
	}
	case 2: {
		return 0x50424752;	/* "RGBP" RGB565 */
	}
	case 3: {
		return 0x35315241;	/* "AR15" ARGB1555 */
	}
	case 4: {
		return 0x32315241;	/* "AR12" ARGB4444 */
	}
	default: {
		return 0;
	}
	}
}

/* fourcc -> overlay SET_FMT enum (doc §3.2). 0 = unsupported by overlay. */
static u32 ar_overlay_fmt_code(u32 fourcc)
{
	switch (fourcc) {
	case 0x32315241: {
		return 1;	/* AR12 ARGB4444 */
	}
	case 0x35315241: {
		return 3;	/* AR15 ARGB1555 */
	}
	case 0x50424752: {
		return 4;	/* RGBP RGB565 */
	}
	case 0x34325241: {
		return 6;	/* AR24 ARGB8888 */
	}
	case 0x32315559: {
		return 15;	/* YU12 YUV420 */
	}
	default: {
		return 0;	/* incl. BGR3 RGB888 -> overlay unsupported */
	}
	}
}

/* Fill an ar_overlay_buf for QBUF: plane base(s) from `base` phys. num_planes is
 * 1 for packed RGB, 3 for YU12 planar (I420). Matches ar_fb_pan_display.part.1.
 */
static void ar_overlay_fill_buf(struct ar_overlay_buf *buf, phys_addr_t base,
				u32 fourcc, u32 line_length, u32 height)
{
	memset(buf, 0, sizeof(*buf));
	buf->plane[0].phys = base;
	if (fourcc == 0x32315559) {	/* YU12 planar (INFERRED-unexercised path) */
		buf->num_planes = 3;
		buf->plane[1].phys = base + (u64)line_length * height;
		buf->plane[2].phys = buf->plane[1].phys +
				     (u64)(line_length / 2) * (height / 2);
	} else {
		buf->num_planes = 1;
	}
}

/* format -> bpp + RGBA bitfields (vendor table). */
static void set_format(struct fb_var_screeninfo *v, int fmt)
{
	struct fb_bitfield r, g, b, a;

	switch (fmt) {
	case 1: {	/* RGB888 */
		v->bits_per_pixel = 24;
		r = (struct fb_bitfield){16, 8, 0};
		g = (struct fb_bitfield){8, 8, 0};
		b = (struct fb_bitfield){0, 8, 0};
		a = (struct fb_bitfield){0, 0, 0};
	}
	break;
	case 2: {	/* RGB565 */
		v->bits_per_pixel = 16;
		r = (struct fb_bitfield){11, 5, 0};
		g = (struct fb_bitfield){5, 6, 0};
		b = (struct fb_bitfield){0, 5, 0};
		a = (struct fb_bitfield){0, 0, 0};
	}
	break;
	case 3: {	/* ARGB1555 */
		v->bits_per_pixel = 16;
		r = (struct fb_bitfield){10, 5, 0};
		g = (struct fb_bitfield){5, 5, 0};
		b = (struct fb_bitfield){0, 5, 0};
		a = (struct fb_bitfield){15, 1, 0};
	}
	break;
	case 4: {	/* ARGB4444 */
		v->bits_per_pixel = 16;
		r = (struct fb_bitfield){8, 4, 0};
		g = (struct fb_bitfield){4, 4, 0};
		b = (struct fb_bitfield){0, 4, 0};
		a = (struct fb_bitfield){12, 4, 0};
	}
	break;
	default: {	/* 0: ARGB8888 */
		v->bits_per_pixel = 32;
		r = (struct fb_bitfield){16, 8, 0};
		g = (struct fb_bitfield){8, 8, 0};
		b = (struct fb_bitfield){0, 8, 0};
		a = (struct fb_bitfield){24, 8, 0};
	}
	break;
	}
	v->red = r;
	v->green = g;
	v->blue = b;
	v->transp = a;
}

static int ar_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct ar_overlay_buf buf;
	phys_addr_t base;
	u32 line_length = info->fix.line_length;
	int ret, guard;

	if (var->yoffset + var->yres > var->yres_virtual)
		return -EINVAL;
	info->var.yoffset = var->yoffset;
	info->var.xoffset = var->xoffset;

	/* Forward the flip to the ar_overlay compositor: QBUF the new scanout
	 * base, then DQBUF in a loop to drain completed buffers (cmds
	 * 0x4107/0x4108). Mirrors ar_fb_pan_display.part.1 - see doc §3.4.
	 * base = smem_start + yoffset*line_length.
	 */
	base = info->fix.smem_start + (phys_addr_t)var->yoffset * line_length;
	ar_overlay_fill_buf(&buf, base, fb.fourcc, line_length, info->var.yres);
	ret = ar_overlay_xfer(fb.overlay_fd, AR_OVL_QBUF, sizeof(buf), &buf);
	if (ret < 0)
		return ret;

	/* DQBUF drain: the server returns each completed buffer's phys in
	 * plane[0].phys; loop until it returns 0 (bounded to avoid a runaway
	 * loop if a future transport misbehaves).
	 */
	for (guard = 0; guard < 16; guard++) {
		buf.plane[0].phys = 0;
		ret = ar_overlay_xfer(fb.overlay_fd, AR_OVL_DQBUF, sizeof(buf), &buf);
		if (ret < 0)
			break;
		if (!buf.plane[0].phys)
			break;
	}
	return 0;
}

/* Bring-up handshake (ar_fb_remote_init): SET_RES -> SET_FMT -> STREAM_ON ->
 * initial QBUF(base=smem_start). Returns <0 on failure (e.g. unsupported fmt).
 * NOTE: NOT used to gate register_framebuffer (unlike the vendor) - see file
 * banner. With the transport stubbed every step trivially succeeds.
 */
static int ar_overlay_remote_init(struct fb_info *info)
{
	struct ar_overlay_set_res res;
	struct ar_overlay_buf buf;
	u32 line_length = info->fix.line_length;
	u32 fmt = ar_overlay_fmt_code(fb.fourcc);
	u32 enable = 1;
	int ret;

	if (!fmt) {
		pr_warn("ar_overlay: fourcc %#x unsupported by compositor\n",
			fb.fourcc);
		return -EINVAL;
	}

	res.width	  = info->var.xres;
	res.height	  = info->var.yres;
	res.stride	  = line_length;
	res.chroma_stride = line_length >> 1;
	ret = ar_overlay_xfer(fb.overlay_fd, AR_OVL_SET_RES, sizeof(res), &res);
	if (ret < 0)
		return ret;

	ret = ar_overlay_xfer(fb.overlay_fd, AR_OVL_SET_FMT, sizeof(fmt), &fmt);
	if (ret < 0)
		return ret;

	ret = ar_overlay_xfer(fb.overlay_fd, AR_OVL_STREAM_ON, sizeof(enable),
			      &enable);
	if (ret < 0)
		return ret;

	ar_overlay_fill_buf(&buf, info->fix.smem_start, fb.fourcc, line_length,
			    info->var.yres);
	return ar_overlay_xfer(fb.overlay_fd, AR_OVL_QBUF, sizeof(buf), &buf);
}

/* write-combine mmap. */
static int ar_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	size_t len = vma->vm_end - vma->vm_start;

	if (!fb.phys || off > fb.smem_len || len > fb.smem_len - off)
		return -EINVAL;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);
	return remap_pfn_range(vma, vma->vm_start,
			       vma->vm_pgoff + (fb.phys >> PAGE_SHIFT),
			       len, vma->vm_page_prot);
}

static int ar_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	u32 val;

	/* These forward cursor/OSD inner cmds to ar_overlay (doc §3.5-3.8).
	 * The forwarding shim is the documented transport stub.
	 */
	switch (cmd) {
	case ARFB_SET_ICON: {		/* 'F' 0x81 -> inner 0x1024, payload = path str */
		if (!uarg)
			return -EINVAL;
		/* Vendor passes the *user* pointer straight through as env.ptr with
		 * size strlen(path)+1; that requires the CUSE client to copy from
		 * the caller's address space. With the stub we just forward the user
		 * pointer (TODO: real transport must marshal it).
		 */
		return ar_overlay_xfer(fb.overlay_fd, AR_OVL_CURSOR_ICON,
				       strnlen_user(uarg, PATH_MAX), uarg);
	}
	case ARFB_SET_CURSOR: {		/* 'F' 0x82 -> 0x1026 (enable) / 0x1025 (disable) */
		if (uarg) {
			val = 1;
			return ar_overlay_xfer(fb.overlay_fd, AR_OVL_CURSOR_EN,
					       sizeof(val), &val);
		}
		val = 0;
		return ar_overlay_xfer(fb.overlay_fd, AR_OVL_CURSOR_POS,
				       sizeof(val), &val);
	}
	case ARFB_OVERLAY_OP: {		/* 'F' 0x83 -> inner 0x1028, 8-byte user blob */
		if (!uarg)
			return -EINVAL;
		return ar_overlay_xfer(fb.overlay_fd, AR_OVL_OP, 8, uarg);
	}
	case ARFB_FLUSH_ZONE: {		/* 'F' 0x84 -> validate-only, NOT forwarded (doc §3.8) */
		struct { u32 line_offset; u32 line_num; } zone;

		if (!uarg)
			return -EINVAL;
		if (copy_from_user(&zone, uarg, sizeof(zone)))
			return -EFAULT;
		if (zone.line_offset + zone.line_num > info->var.yres_virtual)
			return -EINVAL;
		return 0;
	}
	default: {
		return -ENOTTY;	/* generic FB ioctls handled by fbmem core */
	}
	}
}

/*
 * Draw ops. This kernel has CONFIG_FRAMEBUFFER_CONSOLE=y, so fbcon binds at
 * register_framebuffer() and calls fb_imageblit (via soft_cursor) / fb_fillrect /
 * fb_copyarea. Leaving them NULL NULL-derefs (confirmed on HW: soft_cursor ->
 * pc=0x0 panic). The cfb_* helpers (CONFIG_FB_CFB_*) are trimmed out of this
 * kernel, so we can't use them. The goggle panel is driven by the OSD/overlay +
 * the mmap consumers, NOT the Linux VT, so
 * we do not want the text console rendered onto fb0 anyway. These no-op ops keep
 * fbcon from faulting while leaving fb0 entirely to the OSD. (Proper long-term:
 * disable CONFIG_FRAMEBUFFER_CONSOLE, or restore CONFIG_FB_CFB_* + the cfb_* ops;
 * see KERNEL-REQUIREMENTS.md.)
 */
static void ar_fb_fillrect(struct fb_info *info, const struct fb_fillrect *r) { }
static void ar_fb_copyarea(struct fb_info *info, const struct fb_copyarea *a) { }
static void ar_fb_imageblit(struct fb_info *info, const struct fb_image *img) { }

static const struct fb_ops ar_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_pan_display	= ar_fb_pan_display,
	.fb_fillrect	= ar_fb_fillrect,
	.fb_copyarea	= ar_fb_copyarea,
	.fb_imageblit	= ar_fb_imageblit,
	.fb_ioctl	= ar_fb_ioctl,
	.fb_compat_ioctl = ar_fb_ioctl,
	.fb_mmap	= ar_fb_mmap,
};

static int __init ar_fb_init(void)
{
	struct fb_info *info;
	u32 bpp, line_length, smem_len;
	int ret;

	info = framebuffer_alloc(0, NULL);
	if (!info)
		return -ENOMEM;
	fb.info = info;

	info->var.xres = width;
	info->var.yres = height;
	info->var.xres_virtual = width;
	info->var.yres_virtual = height * number;
	set_format(&info->var, format);
	info->var.activate = FB_ACTIVATE_NOW;

	/* ar_overlay state. The vendor opens the "ar_overlay" CUSE device into
	 * par+0x00 here; no kernel CUSE client yet, so fd stays -1 (the xfer shim
	 * is a stub). fourcc drives SET_FMT + QBUF plane count.
	 */
	fb.overlay_fd = -1;
	fb.fourcc = ar_fb_fourcc(format);

	bpp = info->var.bits_per_pixel;
	/* Pad the visible width to a 256-pixel boundary for the stride, matching
	 * the vendor: 1920 -> 2048 px, i.e. line_length = 4096 B at 16bpp.
	 * (Byte-aligning 3840 gives no padding.)
	 */
	line_length = ALIGN(info->var.xres_virtual, 256) * bpp / 8;
	smem_len = PAGE_ALIGN(line_length * info->var.yres_virtual);

	/* MMZ-backed contiguous backing store, zone "fb_mmz". */
	fb.mmb = hil_mmb_alloc_v2("fb_mmz", smem_len, PAGE_SIZE, "", NULL, 0);
	if (!fb.mmb) {
		ret = -ENOMEM;
		goto err_release;
	}
	fb.phys = fb.mmb->phys;
	fb.smem_len = smem_len;
	fb.screen = hil_mmb_map2kern(fb.mmb);
	if (fb.screen)
		memset(fb.screen, 0, smem_len);

	strscpy(info->fix.id, "arfb", sizeof(info->fix.id));
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.xpanstep = 0;
	info->fix.ypanstep = 1;
	info->fix.ywrapstep = 0;
	info->fix.line_length = line_length;
	info->fix.smem_start = fb.phys;
	info->fix.smem_len = smem_len;
	info->fix.accel = FB_ACCEL_NONE;

	info->fbops = &ar_fb_ops;
	info->screen_base = fb.screen;
	info->screen_size = smem_len;
	info->flags = FBINFO_VIRTFB;

	if (fb_alloc_cmap(&info->cmap, 256, 0)) {
		ret = -ENOMEM;
		goto err_mmz;
	}

	/* ar_overlay bring-up handshake (SET_RES/SET_FMT/STREAM_ON/QBUF). The
	 * vendor GATES register_framebuffer on this succeeding; we deliberately do
	 * NOT - the node must come up for the OSD renderer even with the transport
	 * stubbed. Log but ignore the result.
	 */
	ret = ar_overlay_remote_init(info);
	if (ret < 0) {
		pr_warn("ar_overlay: remote_init failed (%d); registering anyway\n",
			ret);
	}

	ret = register_framebuffer(info);
	if (ret)
		goto err_cmap;

	pr_info("fb%d: %dx%d fmt=%d bpp=%u stride=%u smem=%#x phys=%pa\n",
		info->node, width, height, format, bpp, line_length, smem_len,
		&fb.phys);
	return 0;

err_cmap:
	fb_dealloc_cmap(&info->cmap);
err_mmz:
	if (fb.screen)
		hil_mmb_unmap(fb.mmb);
	hil_mmb_free(fb.mmb);
err_release:
	framebuffer_release(info);
	return ret;
}

static void __exit ar_fb_exit(void)
{
	if (fb.info) {
		u32 enable = 1;	/* vendor sends 1 to STREAM_OFF too (ar_fb_remote_release) */

		ar_overlay_xfer(fb.overlay_fd, AR_OVL_STREAM_OFF,
				sizeof(enable), &enable);
		unregister_framebuffer(fb.info);
		fb_dealloc_cmap(&fb.info->cmap);
		if (fb.screen)
			hil_mmb_unmap(fb.mmb);
		hil_mmb_free(fb.mmb);
		framebuffer_release(fb.info);
	}
}

module_init(ar_fb_init);
module_exit(ar_fb_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("missinglynk (open reimpl)");
MODULE_DESCRIPTION("Artosyn OSD framebuffer /dev/fb0 (open)");
