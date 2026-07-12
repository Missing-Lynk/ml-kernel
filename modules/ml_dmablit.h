/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ml_dmablit - userspace ABI for the open DMA-blit shim over dmaengine.
 *
 * A char device (/dev/ml-dmablit) that submits a small batch of physical
 * memcpy copies to the DesignWare AXI DMA engine (mainline dw-axi-dmac, built
 * in - see configs/dma.config + the axidma@8800000 DT node). It exists
 * because dmaengine has no userspace interface; userspace compositors batch
 * dmabuf copies through this device to move decoded tiles into the composite
 * DRM buffer off-CPU.
 *
 * Contract: every buffer is a dmabuf fd. Each copy reads [src_off, src_off+len)
 * of a source dmabuf and writes it to [dst_off, dst_off+len) of the single
 * destination dmabuf. The kernel splits the batch across the (up to 2) engine
 * channels. Buffers MUST be physically contiguous (single DMA segment) - the
 * decoder's vb2-dma-contig output and DRM GEM-DMA/CMA buffers both are; a
 * multi-segment buffer is rejected with -EOPNOTSUPP so userspace falls back to
 * a CPU copy rather than ever mis-copying.
 */
#ifndef _ML_DMABLIT_H
#define _ML_DMABLIT_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define ML_DMABLIT_MAX_COPIES 8

struct ml_dmablit_copy {
	__s32 src_fd;		/* dmabuf fd of the source (e.g. a decoded tile) */
	__u32 src_off;		/* byte offset within the source buffer */
	__u32 dst_off;		/* byte offset within the destination buffer */
	__u32 len;		/* byte length of this copy (4-byte aligned, > 0) */
};

struct ml_dmablit_req {
	__s32 dst_fd;		/* dmabuf fd of the single destination (composite) */
	__u32 n;		/* number of entries in copy[] (1..ML_DMABLIT_MAX_COPIES) */
	struct ml_dmablit_copy copy[ML_DMABLIT_MAX_COPIES];
};

#define ML_DMABLIT_IOC_MAGIC 'D'
/* Submit a batch and block until every copy has completed on the hardware. */
#define ML_DMABLIT_SUBMIT _IOW(ML_DMABLIT_IOC_MAGIC, 0x01, struct ml_dmablit_req)
/* Explicitly clean (flush to DDR) the CPU cache for a dmabuf, via the DMA engine's own
 * (non-coherent) device - the equivalent of the vendor's ar_hal_sys_mmz_flush_cache_pa. Use
 * after CPU-writing a region of a buffer the engine will scan out / DMA. Arg = the dmabuf fd.
 */
#define ML_DMABLIT_FLUSH _IOW(ML_DMABLIT_IOC_MAGIC, 0x02, __s32)

#endif /* _ML_DMABLIT_H */
