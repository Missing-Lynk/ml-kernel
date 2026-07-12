/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar_mmz.h - open reimplementation of the Artosyn/HiSilicon MMZ kernel API.
 *
 * This is the in-kernel contract exported by ar_osal.ko and consumed by ar_vb.ko
 * and ar_sys.ko (and, in principle, any other open MPP module). It is NOT a
 * userspace ABI: the closed media libraries reach MMZ only through the
 * /dev/mmz_userdev ioctl + mmap interface (see ar_uapi.h / ar_osal.c), never
 * through these symbols. Therefore the signatures here are ours to define - we
 * own both producer (ar_osal) and consumers - and we keep the historical hil_*
 * names purely for fidelity with the vendor driver.
 *
 * Memory model: each "zone" (struct mmz_zone) owns a fixed physical range carved
 * out of RAM (the DTS reserved-memory mmz@29400000 / sram regions, no-map). The
 * driver sub-allocates "blocks" (struct mmb) from a zone with first-fit. Mappings
 * are write-combine (kernel: memremap MEMREMAP_WC; user: pgprot_writecombine over
 * remap_pfn_range) so the carveout is DMA-coherent with the codec/scaler engines
 * without explicit cache maintenance. (A cached fast-path is a documented TODO
 * needing exported arm64 cache ops; see ar_osal.c.)
 */
#ifndef _AR_MMZ_H
#define _AR_MMZ_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/mutex.h>

#define MMB_NAME_LEN 48
#define MMZ_NAME_LEN 64

struct mmz_zone {
	char			name[MMZ_NAME_LEN];
	phys_addr_t		base;		/* physical start */
	size_t			size;		/* bytes */
	u32			gfp;		/* allocation-policy flags (vendor: gfp) */
	void __iomem		*kvirt;		/* whole-zone WC kernel mapping */
	struct list_head	mmb_list;	/* allocated blocks, sorted by phys */
	struct list_head	node;		/* g_zone_list */
};

struct mmb {
	char			name[MMB_NAME_LEN];
	phys_addr_t		phys;
	size_t			size;
	u32			align;
	struct mmz_zone		*zone;
	void			*kvirt;		/* derived from zone->kvirt when mapped */
	int			map_kref;	/* kernel-map refcount (map2kern/unmap) */
	int			user_kref;	/* user-map refcount (REMAP/UNMAP) */
	atomic_t		refcount;	/* hil_mmb_get/put */
	struct list_head	node;		/* in zone->mmb_list */
};

/* Global lock guarding the zone list, every zone's mmb_list, and the user-map
 * registry. Held by ar_osal internals; exported so ar_sys can serialise too.
 */
extern struct mutex ar_mmz_lock;

/* ---- zone management ---- */
struct mmz_zone *hil_mmz_create(const char *name, u32 gfp,
				phys_addr_t start, size_t size);
struct mmz_zone *hil_mmz_create_v2(const char *name, u32 gfp,
				   phys_addr_t start, size_t size, u32 type);
int		 hil_mmz_destroy(struct mmz_zone *zone);
struct mmz_zone *hil_mmz_find(phys_addr_t phys);
phys_addr_t	 hil_mmz_get_phys(struct mmz_zone *zone);

/* ---- block alloc/free ---- */
struct mmb *hil_mmb_alloc(const char *name, unsigned long size,
			  unsigned long align, u32 gfp, const char *zone_name);
struct mmb *hil_mmb_alloc_v2(const char *name, unsigned long size,
			     unsigned long align, const char *zone_name,
			     void *priv, u32 gfp);
int	    hil_mmb_free(struct mmb *b);

/* ---- kernel mapping (write-combine; cached==nocache for now, see ar_osal.c) ---- */
void *hil_mmb_map2kern(struct mmb *b);
void *hil_mmb_map2kern_cached(struct mmb *b);
int   hil_mmb_unmap(struct mmb *b);
/* "mmf" = map-by-phys variants (frame fast-path used by the codec libs) */
void *hil_mmf_map2kern_cache(phys_addr_t phys, size_t size);
void *hil_mmf_map2kern_nocache(phys_addr_t phys, size_t size);
int   hil_mmf_unmap(phys_addr_t phys);

/* ---- lookup ---- */
struct mmb *hil_mmb_getby_phys(phys_addr_t phys);
struct mmb *hil_mmb_getby_phys_2(phys_addr_t phys, struct mmz_zone **zone_out);
struct mmb *hil_mmb_getby_kvirt(const void *kvirt);

/* ---- refcount (internal to ar_osal, not exported) ---- */
int hil_mmb_get(struct mmb *b);
int hil_mmb_put(struct mmb *b);

/* ---- cache maintenance (WC backing => ordering barriers; see ar_osal.c) ---- */
int hil_mmb_flush_dcache_byaddr(void *kvirt, phys_addr_t phys, size_t size);
int hil_mmb_flush_dcache_byaddr_safe(phys_addr_t phys, void *kvirt, size_t size);
int hil_mmb_invalid_cache_byaddr(void *kvirt, phys_addr_t phys, size_t size);

/* ---- zone/range validation ---- */
bool hil_is_phys_in_mmz(phys_addr_t phys, size_t size);
int  hil_map_mmz_check_phys(phys_addr_t phys, size_t size);
int  hil_vma_check(unsigned long vaddr, size_t size);

/* Resolve a userspace virtual address (into an mmap'd MMZ buffer) to its physical
 * address by searching the registry of active user mappings. Works for our
 * VM_PFNMAP mappings where get_user_pages() can't (no struct page).
 */
phys_addr_t usr_virt_to_phys(unsigned long uvaddr);

/* User-map registry hooks, called by the /dev/mmz_userdev + /dev/ar_sys mmap
 * paths so usr_virt_to_phys() and unmap can resolve vaddr<->phys.
 */
int  ar_mmz_user_map_add(unsigned long uvaddr, phys_addr_t phys, size_t size);
void ar_mmz_user_map_del(unsigned long uvaddr);

#endif /* _AR_MMZ_H */
