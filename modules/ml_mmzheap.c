// SPDX-License-Identifier: GPL-2.0
/*
 * ml_mmzheap.ko - dma-heap "mmz" (/dev/dma_heap/mmz) over the shared MMZ
 * carveout.
 *
 * The platform device binds the mmz@29400000 reserved-memory pool via its
 * memory-region phandle, which assigns it the pool's per-device coherent
 * allocator (kernel/dma/coherent.c). rmem_dma_device_init caches one
 * dma_coherent_mem per reserved-memory node, so this device and the wave5
 * codec allocate from the SAME page-granular first-fit bitmap (patch
 * 0011-dma-coherent-page-granular) - heap buffers and codec buffers
 * coordinate without partitioning the region. Callers bound their own
 * footprint so the codec keeps headroom (ml-pipeline: ML_COMP_MAX).
 *
 * The pool is no-map and memremap'd MEMREMAP_WC: CPU writes land in DDR
 * directly, so the (non-snooping) display controller and the ml_dmablit DMA
 * engine see them without cache maintenance. begin/end_cpu_access are
 * therefore omitted (DMA_BUF_IOCTL_SYNC no-ops), and mmap is
 * pgprot_writecombine.
 *
 * The buffers have no struct pages (no-map region), so map_dma_buf hands the
 * importer a single page-less DMA segment (dma_address/dma_len only). Both
 * consumers only read those fields: drm_gem_dma prime import and
 * ml_dmablit's map_db.
 *
 * mainline has no dma_heap_del, so the heap cannot be unregistered: no
 * module_exit, the module does not support unload.
 */
#define pr_fmt(fmt) "ml_mmzheap: " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/mm.h>

struct mmz_heap {
	struct dma_heap *heap;
	struct device *dev;
};

/* One exported buffer: a coherent-pool allocation. daddr == phys (the rmem
 * pool is mapped 1:1, no dma-ranges on this SoC).
 */
struct mmz_buffer {
	struct mmz_heap *mh;
	void *vaddr;
	dma_addr_t daddr;
	size_t len;
};

static struct sg_table *mmz_map_dma_buf(struct dma_buf_attachment *at,
					enum dma_data_direction dir)
{
	struct mmz_buffer *b = at->dmabuf->priv;
	struct sg_table *sgt;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	if (sg_alloc_table(sgt, 1, GFP_KERNEL)) {
		kfree(sgt);
		return ERR_PTR(-ENOMEM);
	}

	/* Page-less segment: WC coherent memory needs no cache maintenance,
	 * and both importers consume only the DMA address/length. */
	sg_dma_address(sgt->sgl) = b->daddr;
	sg_dma_len(sgt->sgl) = b->len;
	sgt->sgl->length = b->len;

	return sgt;
}

static void mmz_unmap_dma_buf(struct dma_buf_attachment *at,
			      struct sg_table *sgt,
			      enum dma_data_direction dir)
{
	sg_free_table(sgt);
	kfree(sgt);
}

static int mmz_mmap(struct dma_buf *db, struct vm_area_struct *vma)
{
	struct mmz_buffer *b = db->priv;
	size_t size = vma->vm_end - vma->vm_start;

	/* vm_pgoff is user-controlled; an unbounded offset would map physical
	 * pages of other allocations in the same pool.
	 */
	if (vma->vm_pgoff > PHYS_PFN(b->len) ||
	    PHYS_PFN(size) > PHYS_PFN(b->len) - vma->vm_pgoff)
		return -EINVAL;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	return remap_pfn_range(vma, vma->vm_start,
			       PHYS_PFN(b->daddr) + vma->vm_pgoff,
			       size, vma->vm_page_prot);
}

static void mmz_release(struct dma_buf *db)
{
	struct mmz_buffer *b = db->priv;

	dma_free_coherent(b->mh->dev, b->len, b->vaddr, b->daddr);
	kfree(b);
}

static const struct dma_buf_ops mmz_buf_ops = {
	.map_dma_buf = mmz_map_dma_buf,
	.unmap_dma_buf = mmz_unmap_dma_buf,
	.mmap = mmz_mmap,
	.release = mmz_release,
};

static struct dma_buf *mmz_allocate(struct dma_heap *heap, unsigned long len,
				    u32 fd_flags, u64 heap_flags)
{
	struct mmz_heap *mh = dma_heap_get_drvdata(heap);
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct mmz_buffer *b;
	struct dma_buf *db;

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		return ERR_PTR(-ENOMEM);

	b->mh = mh;
	b->len = PAGE_ALIGN(len);
	b->vaddr = dma_alloc_coherent(mh->dev, b->len, &b->daddr, GFP_KERNEL);
	if (!b->vaddr) {
		kfree(b);
		return ERR_PTR(-ENOMEM);
	}

	exp_info.exp_name = dma_heap_get_name(heap);
	exp_info.ops = &mmz_buf_ops;
	exp_info.size = b->len;
	exp_info.flags = fd_flags;
	exp_info.priv = b;

	db = dma_buf_export(&exp_info);
	if (IS_ERR(db)) {
		dma_free_coherent(mh->dev, b->len, b->vaddr, b->daddr);
		kfree(b);
	}

	return db;
}

static const struct dma_heap_ops mmz_heap_ops = {
	.allocate = mmz_allocate,
};

static int mmz_heap_probe(struct platform_device *pdev)
{
	struct dma_heap_export_info exp_info = {
		.name = "mmz",
		.ops = &mmz_heap_ops,
	};
	struct mmz_heap *mh;
	int ret;

	mh = devm_kzalloc(&pdev->dev, sizeof(*mh), GFP_KERNEL);
	if (!mh)
		return -ENOMEM;

	mh->dev = &pdev->dev;

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "no memory-region pool\n");

	exp_info.priv = mh;
	mh->heap = dma_heap_add(&exp_info);
	if (IS_ERR(mh->heap)) {
		of_reserved_mem_device_release(&pdev->dev);
		return PTR_ERR(mh->heap);
	}

	dev_info(&pdev->dev, "dma-heap \"mmz\" on the shared MMZ pool\n");

	return 0;
}

static const struct of_device_id mmz_heap_of_match[] = {
	{ .compatible = "missinglynk,mmz-heap" },
	{ }
};
MODULE_DEVICE_TABLE(of, mmz_heap_of_match);

static struct platform_driver mmz_heap_driver = {
	.probe = mmz_heap_probe,
	.driver = {
		.name = "ml_mmzheap",
		.of_match_table = mmz_heap_of_match,
		.suppress_bind_attrs = true,
	},
};
/* Init-only (no module_exit): dma_heap_add is irreversible, so the module
 * must refuse unload to keep the heap's ops from dangling.
 */
static int __init mmz_heap_init(void)
{
	return platform_driver_register(&mmz_heap_driver);
}
module_init(mmz_heap_init);

MODULE_IMPORT_NS("DMA_BUF");
MODULE_IMPORT_NS("DMA_BUF_HEAP");
MODULE_DESCRIPTION("dma-heap over the shared MMZ reserved-memory pool");
MODULE_LICENSE("GPL");
