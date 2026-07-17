/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ml_dmabuf_map.h - shared dma-buf-fd -> contiguous-DMA-segment resolver.
 *
 * Factored out of ml_dmablit.c so every module that consumes userspace dmabuf
 * fds on a base+len engine (ml_dmablit's dw-axi-dmac memcpy, ar_scaler's
 * crop/resize) resolves and pins them the same way: dma_buf_get + attach +
 * map_attachment with a single-segment contiguity gate. These engines take one
 * physical span, not a scatter list; a multi-segment buffer is rejected with
 * -EOPNOTSUPP so callers fall back to a CPU path instead of mis-transferring.
 * No IOMMU on this SoC, so sg_dma_address == CPU phys.
 *
 * The cache: consumers recycle a small fixed set of buffers (decoder capture
 * bufs, composite/scaler pools), so mappings are cached per client: attach and
 * map once, reuse. A fresh map_attachment per submit costs sg-table churn plus
 * a full-buffer streaming-DMA cache clean. Keyed by the underlying struct
 * dma_buf (fd numbers can be reused); each entry holds its own dma_buf ref so
 * pointer identity is stable while cached. LRU eviction when full.
 *
 * Coherency contract: reusing a cached mapping skips the per-map cache sync,
 * so CPU access to these buffers must be uncached/WC or explicitly synced
 * (DMA_BUF_IOCTL_SYNC reaches live heap attachments; ml_dmablit has
 * ML_DMABLIT_FLUSH) before the engine touches the bytes.
 *
 * Locking is the caller's: hold one lock around every ml_bufcache_* call AND
 * across the DMA the resolved addresses feed, so an LRU eviction cannot unmap
 * a segment the hardware is still reading. Modules including this must
 * MODULE_IMPORT_NS("DMA_BUF").
 */
#ifndef _ML_DMABUF_MAP_H
#define _ML_DMABUF_MAP_H

#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/errno.h>
#include <linux/err.h>

/* One imported dmabuf: attached to a device and mapped to a single contiguous
 * DMA segment. base/size describe that segment.
 */
struct ml_bufmap {
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	enum dma_data_direction dir;
	dma_addr_t base;
	size_t size;
};

static inline int ml_bufmap_map(struct device *dev, struct dma_buf *db,
				enum dma_data_direction dir,
				struct ml_bufmap *bm)
{
	struct dma_buf_attachment *at;
	struct sg_table *sgt;

	at = dma_buf_attach(db, dev);
	if (IS_ERR(at))
		return PTR_ERR(at);

	sgt = dma_buf_map_attachment_unlocked(at, dir);
	if (IS_ERR(sgt)) {
		dma_buf_detach(db, at);
		return PTR_ERR(sgt);
	}

	/* Contiguity is required: the consumers take a base+len, not a list. */
	if (sgt->nents != 1) {
		dma_buf_unmap_attachment_unlocked(at, sgt, dir);
		dma_buf_detach(db, at);
		return -EOPNOTSUPP;
	}

	bm->dmabuf = db;
	bm->attach = at;
	bm->sgt = sgt;
	bm->dir = dir;
	bm->base = sg_dma_address(sgt->sgl);
	bm->size = sg_dma_len(sgt->sgl);

	return 0;
}

/* Drops the mapping AND the entry's dma_buf ref. */
static inline void ml_bufmap_unmap(struct ml_bufmap *bm)
{
	dma_buf_unmap_attachment_unlocked(bm->attach, bm->sgt, bm->dir);
	dma_buf_detach(bm->dmabuf, bm->attach);
	dma_buf_put(bm->dmabuf);
	bm->dmabuf = NULL;
}

#define ML_BUFCACHE_ENTRIES 40

/* Per-client (per-open-fd) mapping cache. Embed in the client struct; zero on
 * allocation (a NULL ent[].dmabuf marks a free slot).
 */
struct ml_bufcache {
	struct ml_bufmap ent[ML_BUFCACHE_ENTRIES];
	u64 stamp[ML_BUFCACHE_ENTRIES];
	u64 tick;
};

/* Look up fd in the cache (by underlying dma_buf identity), mapping onto dev
 * and inserting on miss (LRU eviction when full). A cached entry whose
 * direction can't serve this request is remapped BIDIRECTIONAL.
 */
static inline struct ml_bufmap *ml_bufcache_map_fd(struct ml_bufcache *cl,
						   struct device *dev, int fd,
						   enum dma_data_direction dir)
{
	struct dma_buf *db;
	struct ml_bufmap *bm;
	int i, slot = -1;
	int ret;

	db = dma_buf_get(fd);
	if (IS_ERR(db))
		return ERR_CAST(db);

	for (i = 0; i < ML_BUFCACHE_ENTRIES; i++) {
		bm = &cl->ent[i];
		if (bm->dmabuf == db) {
			if (bm->dir != dir && bm->dir != DMA_BIDIRECTIONAL) {
				ml_bufmap_unmap(bm);	/* also dropped its ref; ours transfers below */
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

		for (i = 0; i < ML_BUFCACHE_ENTRIES; i++) {
			if (cl->stamp[i] < oldest) {
				oldest = cl->stamp[i];
				slot = i;
			}
		}
		ml_bufmap_unmap(&cl->ent[slot]);
	}
	bm = &cl->ent[slot];
	ret = ml_bufmap_map(dev, db, dir, bm);

	if (ret) {
		dma_buf_put(db);
		return ERR_PTR(ret);
	}

	cl->stamp[slot] = ++cl->tick;
	return bm;
}

/* Drop every cached mapping (client teardown). */
static inline void ml_bufcache_release(struct ml_bufcache *cl)
{
	int i;

	for (i = 0; i < ML_BUFCACHE_ENTRIES; i++)
		if (cl->ent[i].dmabuf)
			ml_bufmap_unmap(&cl->ent[i]);
}

#endif /* _ML_DMABUF_MAP_H */
