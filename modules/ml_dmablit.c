// SPDX-License-Identifier: GPL-2.0
/*
 * ml_dmablit.ko - a thin char-device shim exposing the mainline dw-axi-dmac
 * copy engine to userspace as a batched physical memcpy.
 *
 * dmaengine is a kernel-only API; userspace compositors batch dmabuf copies
 * through this device to move decoded tiles into the composite DRM buffer
 * without the CPU. Each batch of {src dmabuf, dst dmabuf, off, off, len}
 * entries becomes ONE dw-axi-dmac descriptor - a single LLI chain spanning
 * every copy - issued on one channel, which the engine enables once and
 * completes once. One ioctl per displayed frame carries up to 6 plane copies.
 *
 * That shape is the reason this module reaches past dmaengine into the
 * controller driver's own header for dw_axi_dma_prep_memcpy_batch(). The
 * generic device_prep_dma_memcpy takes one copy, so a batch would be one
 * descriptor and one channel enable per copy, and on this SoC a channel
 * re-enabled within microseconds of its own completion, under a concurrent
 * display fetch, wedges the memory fabric: frozen frame, no printk, watchdog
 * reset. Consecutive batches rotate through the channels for the same reason.
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
#include <linux/cache.h>

/* The batch entry point (dw_axi_dma_prep_memcpy_batch) and its entry struct live in the
 * controller driver's private header; Kbuild adds the include path for this file only.
 */
#include "dw-axi-dmac.h"

#include "ml_dmablit.h"
#include "ml_dmabuf_map.h"

#define ML_DMABLIT_WANT_CHANS	3	/* every channel the engine has, rotated one batch at a time */

/* A 6 MB frame copies in << 1 ms; 1 s is a fault ceiling. Runtime-settable so the
 * timeout/terminate path can be exercised deliberately (a value below the real copy
 * time forces it on every batch; the caller falls back to CPU blits).
 */
static unsigned int timeout_ms = 1000;
module_param(timeout_ms, uint, 0644);
MODULE_PARM_DESC(timeout_ms, "per-batch completion timeout in ms (default 1000)");

static struct dma_chan *g_chans[ML_DMABLIT_WANT_CHANS];
static int g_nchan;
static unsigned int g_next_chan;	/* round-robin cursor, see ml_dmablit_submit */
static DEFINE_MUTEX(g_lock);		/* one compositor thread, but serialize to be safe */

/* Per-open mapping cache (struct ml_bufcache, ml_dmabuf_map.h). The compositor
 * recycles a small fixed set of buffers (2 decoders x ~9 capture bufs + 16
 * composite bufs + staging), so mappings are attached once and reused; CPU
 * access must be uncached/WC (the reserved dma-heap, DRM dumb-buffer mmap) or
 * explicitly cleaned via ML_DMABLIT_FLUSH before the engine reads it.
 */
static struct ml_bufmap *map_fd_cached(struct ml_bufcache *cl, int fd,
				       enum dma_data_direction dir)
{
	return ml_bufcache_map_fd(cl, g_chans[0]->device->dev, fd, dir);
}

/* The outcome of one batch: the completion the submit waits on, plus whether the engine
 * reported the transfer good. The engine's error path completes a descriptor exactly as its
 * success path does, so the callback alone says only that the descriptor is finished, not that
 * the bytes moved; the result the engine passes here is what separates the two.
 */
struct ml_blit_slot {
	struct completion done;
	enum dmaengine_tx_result result;
};

static void ml_blit_done(void *param, const struct dmaengine_result *res)
{
	struct ml_blit_slot *slot = param;

	slot->result = res ? res->result : DMA_TRANS_NOERROR;
	complete(&slot->done);
}

static long ml_dmablit_submit(struct ml_bufcache *cl, struct ml_dmablit_req *req)
{
	struct ml_bufmap *dst;

	/* One entry per copy, handed to the engine as a single LLI chain. */
	struct axi_dma_memcpy_entry entries[ML_DMABLIT_MAX_COPIES];

	/* Function scope, not inside the submit block: on the terminate path a late
	 * descriptor callback can still complete() this until dmaengine_terminate_sync()
	 * returns, so it must stay live past the wait.
	 */
	struct ml_blit_slot slot;

	struct dma_async_tx_descriptor *tx;
	struct dma_chan *chan;
	int i;
	long ret;

	if (req->n < 1 || req->n > ML_DMABLIT_MAX_COPIES)
		return -EINVAL;

	/* DMA_BIDIRECTIONAL, not DMA_FROM_DEVICE: the destination is a COMPOSITE that the CPU also
	 * writes into (the GStreamer compositor DMA-blits one tile but CPU-blits the other into the
	 * same buffer). DMA_FROM_DEVICE's cache sync only INVALIDATES, discarding the CPU's dirty
	 * lines for the region the DMA does not touch (arrival-order-dependent garble on the CPU
	 * tile). BIDIRECTIONAL does clean-then-invalidate, writing those dirty lines back to DDR
	 * first - the kernel-side equivalent of the vendor's ar_hal_sys_mmz_flush_cache_pa.
	 * NOTE: the sync only runs when the mapping is first cached (see struct ml_bufcache) -
	 * per-frame CPU writes must go through uncached/WC mappings or ML_DMABLIT_FLUSH.
	 */
	dst = map_fd_cached(cl, req->dst_fd, DMA_BIDIRECTIONAL);
	if (IS_ERR(dst))
		return PTR_ERR(dst);

	/* Validate every entry BEFORE submitting any, so a bad batch never leaves
	 * half the copies issued. Bounds against the mapped segment sizes; the
	 * engine's memcpy wants 4-byte alignment.
	 *
	 * The mapping pointers stay valid for the rest of the ioctl: at most n+1 <= 9
	 * fresh-stamped cache entries vs ML_BUFCACHE_ENTRIES slots, so LRU eviction
	 * cannot reach them (same argument as ar_scaler's static_assert). Resolving the
	 * fd once here is also what keeps the bounds check honest: re-resolving it at
	 * submit would re-run dma_buf_get on a table another thread can retarget (dup2)
	 * between the two passes.
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

		entries[i].dst = dst->base + c->dst_off;
		entries[i].src = s->base + c->src_off;
		entries[i].len = c->len;
	}

	/* Rotate the channel from batch to batch. The batch is one descriptor and one
	 * channel enable, so the only enable left that can land close behind a completion
	 * is the next batch's, and rotating puts every other channel between the two.
	 * Enabling a channel microseconds after its own completion, while the display
	 * controller fetches from the same DDR, wedges the memory fabric with no printk.
	 */
	chan = g_chans[g_next_chan];
	g_next_chan = (g_next_chan + 1) % (unsigned int)g_nchan;

	tx = dw_axi_dma_prep_memcpy_batch(chan, entries, req->n,
					  DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx)
		return -EIO;

	init_completion(&slot.done);
	slot.result = DMA_TRANS_NOERROR;
	tx->callback_result = ml_blit_done;
	tx->callback_param = &slot;
	if (dma_submit_error(dmaengine_submit(tx))) {
		ret = -EIO;
		goto terminate;
	}

	/* Single sleep for the whole batch, woken from the engine's completion IRQ (the
	 * descriptor carries DMA_PREP_INTERRUPT), which fires once at the end of the chain.
	 *
	 * A transfer the engine refused returns -EIO, which sends the caller down the same fallback
	 * as a rejected submit: the caller copies that region on the CPU instead of presenting
	 * whatever the destination happened to hold. Rate-limited, because the condition repeats per
	 * batch while it lasts and one line per batch buries the log it belongs in.
	 */
	dma_async_issue_pending(chan);
	if (!wait_for_completion_timeout(&slot.done, msecs_to_jiffies(timeout_ms))) {
		ret = -ETIMEDOUT;
		goto terminate;
	}

	if (slot.result != DMA_TRANS_NOERROR) {
		pr_warn_ratelimited("batch reported transfer result %d\n", slot.result);
		ret = -EIO;
		goto terminate;
	}

	return 0;

terminate:
	/* Flush any queued work so the channel is clean for the next call. */
	dmaengine_terminate_sync(chan);

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

	ret = ml_bufmap_map(g_chans[0]->device->dev, db, DMA_TO_DEVICE, &bm);
	if (ret) {
		dma_buf_put(db);
		return ret;
	}

	ml_bufmap_unmap(&bm);
	return 0;
}

/* Cache maintenance over one range of a dmabuf. Uses the cached mapping the copies already
 * hold, so it costs no attach and no remap: DMA_BIDIRECTIONAL is the direction the destination
 * is mapped with, and asking for a different one would force ml_bufcache_map_fd to remap.
 *
 * An invalidate discards whatever is in the range, so it is rejected unless both ends are
 * cache-line aligned - a partial line would take a neighbouring dirty byte with it.
 */
static long ml_dmablit_cache_range(struct ml_bufcache *cl,
				   const struct ml_dmablit_cache *rq)
{
	struct device *dev = g_chans[0]->device->dev;
	struct ml_bufmap *bm;

	if (!rq->len)
		return -EINVAL;

	if (rq->op != ML_DMABLIT_CLEAN && rq->op != ML_DMABLIT_INVALIDATE)
		return -EINVAL;

	if (rq->op == ML_DMABLIT_INVALIDATE &&
	    ((rq->off | rq->len) & (cache_line_size() - 1)))
		return -EINVAL;

	bm = map_fd_cached(cl, rq->fd, DMA_BIDIRECTIONAL);
	if (IS_ERR(bm))
		return PTR_ERR(bm);

	if (rq->off > bm->size || rq->len > bm->size - rq->off)
		return -EINVAL;

	if (rq->op == ML_DMABLIT_INVALIDATE)
		dma_sync_single_for_cpu(dev, bm->base + rq->off, rq->len,
					DMA_FROM_DEVICE);
	else
		dma_sync_single_for_device(dev, bm->base + rq->off, rq->len,
					   DMA_TO_DEVICE);

	return 0;
}

static int ml_dmablit_open(struct inode *inode, struct file *f)
{
	struct ml_bufcache *cl = kzalloc(sizeof(*cl), GFP_KERNEL);

	if (!cl)
		return -ENOMEM;

	f->private_data = cl;
	return 0;
}

static int ml_dmablit_release(struct inode *inode, struct file *f)
{
	struct ml_bufcache *cl = f->private_data;

	mutex_lock(&g_lock);
	ml_bufcache_release(cl);
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

	if (cmd == ML_DMABLIT_CACHE) {
		struct ml_dmablit_cache rq;

		if (copy_from_user(&rq, (void __user *)arg, sizeof(rq)))
			return -EFAULT;

		mutex_lock(&g_lock);
		ret = ml_dmablit_cache_range(f->private_data, &rq);
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
	} else {
		mutex_lock(&g_lock);
		ret = ml_dmablit_submit(f->private_data, req);
		mutex_unlock(&g_lock);
	}

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
	if (ret) {
		while (g_nchan-- > 0)
			dma_release_channel(g_chans[g_nchan]);
		return ret;
	}

	pr_info("ready: /dev/ml-dmablit on %d channel(s)\n", g_nchan);
	return 0;
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

MODULE_DESCRIPTION("Batched dmabuf memcpy over dw-axi-dmac");
MODULE_AUTHOR("missinglynk");
MODULE_LICENSE("GPL");
