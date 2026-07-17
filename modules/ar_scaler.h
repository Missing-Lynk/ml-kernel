/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar_scaler - userspace ABI for /dev/arscaler (open reimplementation of the
 * Artosyn MPP scaler / crop-resize engine).
 *
 * Two ABI families share the 'Z' magic:
 *
 * Vendor family (nr 0..2, byte-exact from the vendor .ko so the shipped closed
 * userspace keeps working unchanged): addresses are raw PHYSICAL, passed by
 * value inside the 64-byte op descriptor.
 *   0xffc45a00 _IOWR('Z',0,16324)  batch {u32 count; op[255]}
 *   0xc0405a01 _IOWR('Z',1,64)     single crop/resize
 *   0xc0045a02 _IOWR('Z',2,4)      set frequency (int)
 *
 * Open family (nr 3..4, this project's addition): buffers are dmabuf fds
 * (CMA dma-heap, vb2-dma-contig, DRM GEM-DMA - anything physically contiguous),
 * resolved and pinned in-kernel for exactly the ioctl's duration semantics of
 * ml_dmablit: the mapping is cached per open fd, so CPU access to the buffers
 * must be bracketed with DMA_BUF_IOCTL_SYNC (or be uncached/WC). A
 * multi-segment buffer is rejected with -EOPNOTSUPP. srcphy/dstphy in the
 * embedded op must be 0; the kernel fills them from fd + offset.
 */
#ifndef _AR_SCALER_H
#define _AR_SCALER_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define SCALER_MAGIC		'Z'		/* 0x5a */

#define SCALER_BATCH_MAX	255

/*
 * 64-byte op descriptor (single = one element; batch = {u32 count; op[255]}).
 * Field offsets recovered from the vendor driver. All addresses are PHYSICAL,
 * passed by value. Strides are byte counts, multiples of 16.
 */
struct ar_scaler_op {
	__u32	srcphy;		/* 0x00 != 0 */
	__u32	srcw;		/* 0x04 != 0 */
	__u32	srch;		/* 0x08 != 0 */
	__u32	srcstride;	/* 0x0c multiple of 16 */
	__u32	crop_x;		/* 0x10 */
	__u32	crop_y;		/* 0x14 */
	__u32	cropw;		/* 0x18 != 0 */
	__u32	croph;		/* 0x1c != 0 */
	__u32	dstphy;		/* 0x20 != 0 */
	__u32	dstw;		/* 0x24 != 0 */
	__u32	dsth;		/* 0x28 != 0 */
	__u32	dststride;	/* 0x2c multiple of 16 */
	__u32	channels;	/* 0x30 != 0 */
	__u32	control;	/* 0x34 */
	__u32	interp;		/* 0x38 */
	__u32	ctrl3c;		/* 0x3c (read by the batch-register packer) */
};

/* 4 + 255*64 = 16324 bytes (=0x3fc4). */
struct ar_scaler_batch {
	__u32			count;
	struct ar_scaler_op	ops[SCALER_BATCH_MAX];
};

/* ioctl command numbers (byte-exact from the vendor .ko). */
#define SCALER_IOC_BATCH	_IOWR(SCALER_MAGIC, 0, struct ar_scaler_batch)
#define SCALER_IOC_SINGLE	_IOWR(SCALER_MAGIC, 1, struct ar_scaler_op)
#define SCALER_IOC_SETFREQ	_IOWR(SCALER_MAGIC, 2, int)

/* --- open dmabuf family ---------------------------------------------------- */

/* One op addressed by dmabuf fd + byte offset instead of raw phys. `op` carries
 * the geometry exactly as the vendor descriptor (srcphy/dstphy must be 0; the
 * kernel resolves src_fd+src_off / dst_fd+dst_off into them). Offsets are
 * multiples of 16, and the declared images (off + h*stride) must lie inside
 * their buffers. src and dst may be planes of the same dmabuf.
 */
struct ar_scaler_dmabuf_op {
	__s32	src_fd;
	__s32	dst_fd;
	__u32	src_off;
	__u32	dst_off;
	struct ar_scaler_op op;
};

/* Enough for a 3-plane (Y/U/V) frame op with headroom; NOT the vendor's 255
 * (the batch is a stack/heap copy per call, not a DMA-block-sized commitment).
 */
#define SCALER_DMABUF_BATCH_MAX	8

struct ar_scaler_dmabuf_batch {
	__u32				count;
	__u32				reserved;	/* must be 0 */
	struct ar_scaler_dmabuf_op	ops[SCALER_DMABUF_BATCH_MAX];
};

#define SCALER_IOC_SINGLE_DMABUF _IOWR(SCALER_MAGIC, 3, struct ar_scaler_dmabuf_op)
#define SCALER_IOC_BATCH_DMABUF	 _IOWR(SCALER_MAGIC, 4, struct ar_scaler_dmabuf_batch)

#endif /* _AR_SCALER_H */
