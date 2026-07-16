// SPDX-License-Identifier: GPL-2.0
/*
 * ml_dmablit.ko - a thin char-device shim exposing the mainline dw-axi-dmac
 * copy engine to userspace as a batched physical memcpy.
 *
 * dmaengine is a kernel-only API; userspace compositors batch dmabuf copies
 * through this device to move decoded tiles into the composite DRM buffer
 * without the CPU. Each batch of {src dmabuf, dst dmabuf, off, off, len}
 * entries becomes device_prep_dma_memcpy submissions, spread across the (up
 * to 2) engine channels - 2 being the measured sweet spot
 * (bench-measured; 3 regresses). One ioctl per displayed
 * frame carries up to 6 plane copies.
 *
 * ABI: ml_dmablit.h.
 *
 * Buffers must be physically contiguous (single mapped DMA segment). Our
 * sources (vb2-dma-contig decoder output) and destination (DRM GEM-DMA/CMA
 * dumb buffer) both are; a multi-segment buffer is rejected (-EOPNOTSUPP) so
 * the caller falls back to a CPU copy instead of silently corrupting.
 */
#define pr_fmt(fmt) "ml_dmablit: " fmt

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/dmaengine.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

#include "ml_dmablit.h"

#define ML_DMABLIT_WANT_CHANS	2	/* the sweet spot; the engine has 3 */

/* A 6 MB frame copies in << 1 ms; 1 s is a fault ceiling. Runtime-settable so the
 * timeout/terminate path can be exercised deliberately (a value below the real copy
 * time forces it on every round; the caller falls back to CPU blits).
 */
static unsigned int timeout_ms = 1000;
module_param(timeout_ms, uint, 0644);
MODULE_PARM_DESC(timeout_ms, "per-round completion timeout in ms (default 1000)");

static struct dma_chan *g_chans[ML_DMABLIT_WANT_CHANS];
static int g_nchan;
static DEFINE_MUTEX(g_lock);		/* one compositor thread, but serialize to be safe */

/* One imported dmabuf: attached to the engine device and mapped to a single
 * contiguous DMA segment. base/size describe that segment.
 */
struct ml_bufmap {
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	enum dma_data_direction dir;
	dma_addr_t base;
	size_t size;
};

/* Per-open mapping cache. The compositor recycles a small fixed set of buffers
 * (2 decoders x ~9 capture bufs + 16 composite bufs + staging), so attach/map
 * once and reuse: a fresh map_attachment per submit costs sg-table churn plus a
 * full-buffer streaming-DMA cache clean. Keyed by the underlying struct dma_buf
 * (fd numbers can be reused); each entry holds its own dma_buf ref so pointer
 * identity is stable while cached.
 *
 * Coherency contract: reusing a mapping skips the per-submit cache sync, so
 * CPU access to these buffers must be uncached/WC (the reserved dma-heap, DRM
 * dumb-buffer mmap) or explicitly cleaned via ML_DMABLIT_FLUSH before the
 * engine reads it.
 */
#define ML_DMABLIT_CACHE 40

struct ml_client {
	struct ml_bufmap ent[ML_DMABLIT_CACHE];	/* dmabuf == NULL -> free slot */
	u64 stamp[ML_DMABLIT_CACHE];
	u64 tick;
};

static int map_db(struct dma_buf *db, enum dma_data_direction dir,
		  struct ml_bufmap *bm)
{
	struct device *dev = g_chans[0]->device->dev;
	struct dma_buf_attachment *at;
	struct sg_table *sgt;
	int ret;

	at = dma_buf_attach(db, dev);
	if (IS_ERR(at))
		return PTR_ERR(at);
	sgt = dma_buf_map_attachment_unlocked(at, dir);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto detach;
	}
	/* Contiguity is required: the copy engine takes a base+len, not a list. */
	if (sgt->nents != 1) {
		ret = -EOPNOTSUPP;
		goto unmap;
	}
	bm->dmabuf = db;
	bm->attach = at;
	bm->sgt = sgt;
	bm->dir = dir;
	bm->base = sg_dma_address(sgt->sgl);
	bm->size = sg_dma_len(sgt->sgl);
	return 0;

unmap:
	dma_buf_unmap_attachment_unlocked(at, sgt, dir);
detach:
	dma_buf_detach(db, at);
	return ret;
}

/* Drops the mapping AND the entry's dma_buf ref. */
static void unmap_buf(struct ml_bufmap *bm)
{
	dma_buf_unmap_attachment_unlocked(bm->attach, bm->sgt, bm->dir);
	dma_buf_detach(bm->dmabuf, bm->attach);
	dma_buf_put(bm->dmabuf);
	bm->dmabuf = NULL;
}

/* Look up fd in the client's cache (by underlying dma_buf identity), mapping
 * and inserting on miss (LRU eviction when full). A cached entry whose
 * direction can't serve this request is remapped BIDIRECTIONAL.
 */
static struct ml_bufmap *map_fd_cached(struct ml_client *cl, int fd,
				       enum dma_data_direction dir)
{
	struct dma_buf *db;
	struct ml_bufmap *bm;
	int i, slot = -1;
	int ret;

	db = dma_buf_get(fd);
	if (IS_ERR(db))
		return ERR_CAST(db);

	for (i = 0; i < ML_DMABLIT_CACHE; i++) {
		bm = &cl->ent[i];
		if (bm->dmabuf == db) {
			if (bm->dir != dir && bm->dir != DMA_BIDIRECTIONAL) {
				unmap_buf(bm);	/* also dropped its ref; ours transfers below */
				dir = DMA_BIDIRECTIONAL;
				slot = i;
				break;
			}
			dma_buf_put(db);	/* the cache entry keeps its own ref */
			cl->stamp[i] = ++cl->tick;
			return bm;
		}
		if (slot < 0 && !bm->dmabuf)
			slot = i;
	}
	if (slot < 0) {
		u64 oldest = U64_MAX;

		for (i = 0; i < ML_DMABLIT_CACHE; i++) {
			if (cl->stamp[i] < oldest) {
				oldest = cl->stamp[i];
				slot = i;
			}
		}
		unmap_buf(&cl->ent[slot]);
	}
	bm = &cl->ent[slot];
	ret = map_db(db, dir, bm);
	if (ret) {
		dma_buf_put(db);
		return ERR_PTR(ret);
	}
	cl->stamp[slot] = ++cl->tick;
	return bm;
}

static void ml_blit_done(void *param)
{
	complete(param);
}

/* Block until each used channel's descriptor completes: single sleep per
 * channel, woken from the engine's completion IRQ (the descriptors carry
 * DMA_PREP_INTERRUPT), replacing the old usleep_range poll loop (~40 timer
 * wakeups per 2-3 ms blit at 120 blits/s was real sys time).
 */
static int wait_chans(struct completion *done, const bool *used)
{
	int ch;

	for (ch = 0; ch < g_nchan; ch++)
		if (used[ch])
			dma_async_issue_pending(g_chans[ch]);
	for (ch = 0; ch < g_nchan; ch++) {
		if (!used[ch])
			continue;
		if (!wait_for_completion_timeout(&done[ch],
				msecs_to_jiffies(timeout_ms)))
			return -ETIMEDOUT;
	}
	return 0;
}

static long ml_dmablit_submit(struct ml_client *cl, struct ml_dmablit_req *req)
{
	struct ml_bufmap *dst;
	/* Function scope, not per-round: on the terminate path a late descriptor
	 * callback can still complete() these until dmaengine_terminate_sync()
	 * returns, so they must stay live past the round block.
	 */
	struct completion done[ML_DMABLIT_WANT_CHANS];
	bool used[ML_DMABLIT_WANT_CHANS];
	int i, ch;
	long ret;

	if (req->n < 1 || req->n > ML_DMABLIT_MAX_COPIES)
		return -EINVAL;

	/* DMA_BIDIRECTIONAL, not DMA_FROM_DEVICE: the destination is a COMPOSITE that the CPU also
	 * writes into (the GStreamer compositor DMA-blits one tile but CPU-blits the other into the
	 * same buffer). DMA_FROM_DEVICE's cache sync only INVALIDATES, discarding the CPU's dirty
	 * lines for the region the DMA does not touch (arrival-order-dependent garble on the CPU
	 * tile). BIDIRECTIONAL does clean-then-invalidate, writing those dirty lines back to DDR
	 * first - the kernel-side equivalent of the vendor's ar_hal_sys_mmz_flush_cache_pa.
	 * NOTE: the sync only runs when the mapping is first cached (see struct ml_client) -
	 * per-frame CPU writes must go through uncached/WC mappings or ML_DMABLIT_FLUSH.
	 */
	dst = map_fd_cached(cl, req->dst_fd, DMA_BIDIRECTIONAL);
	if (IS_ERR(dst))
		return PTR_ERR(dst);

	/* Validate every entry BEFORE submitting any, so a bad batch never leaves
	 * half the copies issued. Bounds against the mapped segment sizes; the
	 * engine's memcpy wants 4-byte alignment.
	 */
	for (i = 0; i < (int)req->n; i++) {
		struct ml_dmablit_copy *c = &req->copy[i];
		struct ml_bufmap *s = map_fd_cached(cl, c->src_fd, DMA_TO_DEVICE);

		if (IS_ERR(s))
			return PTR_ERR(s);
		if (c->len == 0 || (c->len & 3) || (c->src_off & 3) || (c->dst_off & 3))
			return -EINVAL;
		if ((u64)c->src_off + c->len > s->size ||
		    (u64)c->dst_off + c->len > dst->size)
			return -EINVAL;
	}

	/* Submit in ROUNDS: at most one descriptor per channel per round, issued and
	 * waited before the next. This is a workaround for a dw-axi-dmac limitation in
	 * this kernel: its non-cyclic completion handler (axi_chan_block_xfer_complete)
	 * completes a descriptor but does NOT start the next queued one - only the error
	 * path calls axi_chan_start_first_queued - so a 2nd descriptor queued on the same
	 * channel is stranded and never completes. Rounds keep at most one outstanding
	 * descriptor per channel while still running g_nchan copies concurrently (2 = the
	 * throughput sweet spot, bench-measured), so we lose nothing.
	 * A one-line driver fix would allow deeper queuing but we do not need it.
	 */
	for (i = 0; i < (int)req->n; i += g_nchan) {
		int k;

		for (k = 0; k < g_nchan; k++)
			used[k] = false;

		for (k = 0; k < g_nchan && (i + k) < (int)req->n; k++) {
			struct ml_dmablit_copy *c = &req->copy[i + k];
			struct ml_bufmap *s = map_fd_cached(cl, c->src_fd, DMA_TO_DEVICE);
			struct dma_async_tx_descriptor *tx;

			if (IS_ERR(s)) {
				ret = PTR_ERR(s);
				goto terminate;
			}
			/* channel k gets this round's k-th copy (already validated) */
			tx = dmaengine_prep_dma_memcpy(g_chans[k], dst->base + c->dst_off,
						       s->base + c->src_off, c->len,
						       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
			if (!tx) {
				ret = -EIO;
				goto terminate;
			}
			init_completion(&done[k]);
			tx->callback = ml_blit_done;
			tx->callback_param = &done[k];
			if (dma_submit_error(dmaengine_submit(tx))) {
				ret = -EIO;
				goto terminate;
			}
			used[k] = true;
		}
		ret = wait_chans(done, used);
		if (ret)
			goto terminate;
	}
	return 0;

terminate:
	/* Flush any queued work so the channels are clean for the next call. */
	for (ch = 0; ch < g_nchan; ch++)
		dmaengine_terminate_sync(g_chans[ch]);
	return ret;
}

/* Clean the CPU cache for a whole dmabuf to DDR by mapping it TO_DEVICE on the engine's device
 * (arch_sync_dma_for_device -> dcache clean). Real flush even when the dma-heap's own
 * begin/end_cpu_access no-ops the sync (the vendor's ar_hal_sys_mmz_flush_cache_pa analog).
 * Deliberately BYPASSES the mapping cache: a fresh map/unmap is the whole point here.
 */
static long ml_dmablit_flush(int fd)
{
	struct ml_bufmap bm;
	struct dma_buf *db = dma_buf_get(fd);
	long ret;

	if (IS_ERR(db))
		return PTR_ERR(db);
	ret = map_db(db, DMA_TO_DEVICE, &bm);
	if (ret) {
		dma_buf_put(db);
		return ret;
	}
	unmap_buf(&bm);
	return 0;
}

static int ml_dmablit_open(struct inode *inode, struct file *f)
{
	struct ml_client *cl = kzalloc(sizeof(*cl), GFP_KERNEL);

	if (!cl)
		return -ENOMEM;
	f->private_data = cl;
	return 0;
}

static int ml_dmablit_release(struct inode *inode, struct file *f)
{
	struct ml_client *cl = f->private_data;
	int i;

	mutex_lock(&g_lock);
	for (i = 0; i < ML_DMABLIT_CACHE; i++)
		if (cl->ent[i].dmabuf)
			unmap_buf(&cl->ent[i]);
	mutex_unlock(&g_lock);
	kfree(cl);
	return 0;
}

static long ml_dmablit_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct ml_dmablit_req *req;
	long ret;

	if (cmd == ML_DMABLIT_FLUSH) {
		__s32 fd;

		if (copy_from_user(&fd, (void __user *)arg, sizeof(fd)))
			return -EFAULT;
		mutex_lock(&g_lock);
		ret = ml_dmablit_flush(fd);
		mutex_unlock(&g_lock);
		return ret;
	}
	if (cmd != ML_DMABLIT_SUBMIT)
		return -ENOTTY;
	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;
	if (copy_from_user(req, (void __user *)arg, sizeof(*req))) {
		ret = -EFAULT;
		goto out;
	}
	mutex_lock(&g_lock);
	ret = ml_dmablit_submit(f->private_data, req);
	mutex_unlock(&g_lock);
out:
	kfree(req);
	return ret;
}

static const struct file_operations ml_dmablit_fops = {
	.owner		= THIS_MODULE,
	.open		= ml_dmablit_open,
	.release	= ml_dmablit_release,
	.unlocked_ioctl	= ml_dmablit_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
};

static struct miscdevice ml_dmablit_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "ml-dmablit",
	.fops	= &ml_dmablit_fops,
};

static int __init ml_dmablit_init(void)
{
	dma_cap_mask_t mask;
	int i, ret;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	for (i = 0; i < ML_DMABLIT_WANT_CHANS; i++) {
		struct dma_chan *ch = dma_request_channel(mask, NULL, NULL);

		if (!ch)
			break;
		g_chans[i] = ch;
		g_nchan++;
	}
	if (g_nchan == 0) {
		pr_err("no DMA_MEMCPY channel available (is dw-axi-dmac probed?)\n");
		return -ENODEV;
	}

	ret = misc_register(&ml_dmablit_dev);
	if (ret)
		goto release;
	pr_info("ready: /dev/ml-dmablit on %d channel(s)\n", g_nchan);
	return 0;

release:
	while (g_nchan-- > 0)
		dma_release_channel(g_chans[g_nchan]);
	return ret;
}

static void __exit ml_dmablit_exit(void)
{
	misc_deregister(&ml_dmablit_dev);
	while (g_nchan-- > 0)
		dma_release_channel(g_chans[g_nchan]);
}

module_init(ml_dmablit_init);
module_exit(ml_dmablit_exit);

/* dma_buf_* live in the DMA_BUF symbol namespace; importing it is required to link. */
MODULE_IMPORT_NS("DMA_BUF");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("missinglynk");
MODULE_DESCRIPTION("Batched dmabuf memcpy over dw-axi-dmac");
