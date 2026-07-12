// SPDX-License-Identifier: GPL-2.0
/*
 * ar_osal.ko - open reimplementation of the Artosyn/HiSilicon MMZ allocator and
 * its /dev/mmz_userdev char device, for the modern (6.18) kernel port.
 *
 * ABI recovered + re-verified from the vendor .ko / libhal disassembly (struct
 * mmb_info, ioctl table, mmap contract, proc/params). The closed media libraries
 * reach this driver ONLY through /dev/mmz_userdev (ioctl + mmap) - so that ABI is
 * byte-exact (ar_uapi.h). The exported hil_* symbols (ar_mmz.h) are inter-module
 * only (ar_vb, ar_sys) and are ours to shape.
 *
 * Memory: each zone owns a fixed physical carveout (DTS reserved-memory, no-map).
 * Blocks are first-fit sub-allocations. Mappings are write-combine on both the
 * kernel (memremap MEMREMAP_WC) and user (pgprot_writecombine) sides, so the pool
 * is DMA-coherent with the codec/scaler engines without explicit cache ops; the
 * cache-maintenance ioctls degrade to ordering barriers. A cached fast-path
 * (matching the vendor's cached map2kern) is a TODO that needs arm64 cache ops
 * exported to modules (arch_sync_dma_for_{device,cpu} are not, as of 6.18).
 */
#define pr_fmt(fmt) "ar_osal: " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/string.h>

#include "ar_mmz.h"
#include "ar_uapi.h"

/* ---- module params (vendor-compatible) ---- */
static char *mmz;		/* "name,gfp,start,size,type,eqsize:[next...]" */
static char *mmz_allocator = "hisi";
static char *map_mmz;		/* pre-registered phys regions, "start,size:[..]" */
static int   anony = 1;
module_param(mmz, charp, 0444);
module_param(mmz_allocator, charp, 0444);
module_param(map_mmz, charp, 0444);
module_param(anony, int, 0444);

/* Default zones when no mmz= param is supplied - the boot values from RE §8. */
#define DEF_ANON_BASE	0x29400000UL
#define DEF_ANON_SIZE	0x06c00000UL
#define DEF_SRAM_BASE	0x00100000UL
#define DEF_SRAM_SIZE	0x00100000UL

DEFINE_MUTEX(ar_mmz_lock);
EXPORT_SYMBOL(ar_mmz_lock);

static LIST_HEAD(g_zone_list);

/* Registry of active user mappings, for usr_virt_to_phys() over VM_PFNMAP vmas. */
struct user_map {
	unsigned long	uvaddr;
	phys_addr_t	phys;
	size_t		size;
	struct list_head node;
};
static LIST_HEAD(g_user_maps);

/* fwd decls (used by the on-demand mapping helpers before their definitions) */
static struct mmb *__getby_phys_2(phys_addr_t phys, struct mmz_zone **zone_out);

/* ------------------------------------------------------------------------ */
/* zone + block allocator						    */
/* ------------------------------------------------------------------------ */

static struct mmz_zone *zone_find_by_name(const char *name)
{
	struct mmz_zone *z;

	if (!name || !name[0]) {
		/* empty name => the anonymous (first) zone */
		return list_first_entry_or_null(&g_zone_list, struct mmz_zone, node);
	}
	list_for_each_entry(z, &g_zone_list, node)
		if (!strncmp(z->name, name, MMZ_NAME_LEN))
			return z;
	return NULL;
}

struct mmz_zone *hil_mmz_create(const char *name, u32 gfp,
				phys_addr_t start, size_t size)
{
	struct mmz_zone *z;

	if (!size)
		return NULL;
	z = kzalloc(sizeof(*z), GFP_KERNEL);
	if (!z)
		return NULL;
	strscpy(z->name, name ? name : "anonymous", sizeof(z->name));
	z->base = start;
	z->size = size;
	z->gfp = gfp;
	z->kvirt = NULL;	/* zones are pure phys bookkeeping; blocks map on demand */
	INIT_LIST_HEAD(&z->mmb_list);
	mutex_lock(&ar_mmz_lock);
	list_add_tail(&z->node, &g_zone_list);
	mutex_unlock(&ar_mmz_lock);
	pr_info("MMZ new zone <%s> PHYS(%pa,+%#zx)\n", z->name, &z->base, z->size);
	return z;
}
EXPORT_SYMBOL(hil_mmz_create);

struct mmz_zone *hil_mmz_create_v2(const char *name, u32 gfp,
				   phys_addr_t start, size_t size, u32 type)
{
	return hil_mmz_create(name, gfp, start, size);
}
EXPORT_SYMBOL(hil_mmz_create_v2);

int hil_mmz_destroy(struct mmz_zone *z)
{
	if (!z)
		return -EINVAL;
	mutex_lock(&ar_mmz_lock);
	if (!list_empty(&z->mmb_list)) {
		mutex_unlock(&ar_mmz_lock);
		pr_warn("mmbs not free, zone <%s> cannot be unregistered\n", z->name);
		return -EBUSY;
	}
	list_del(&z->node);
	mutex_unlock(&ar_mmz_lock);
	if (z->kvirt)
		memunmap(z->kvirt);
	kfree(z);
	return 0;
}
EXPORT_SYMBOL(hil_mmz_destroy);

/* Lock-free zone lookup; caller MUST hold ar_mmz_lock. */
static struct mmz_zone *__mmz_find(phys_addr_t phys)
{
	struct mmz_zone *z;

	list_for_each_entry(z, &g_zone_list, node)
		if (phys >= z->base && phys < z->base + z->size)
			return z;
	return NULL;
}

struct mmz_zone *hil_mmz_find(phys_addr_t phys)
{
	struct mmz_zone *z;

	mutex_lock(&ar_mmz_lock);
	z = __mmz_find(phys);
	mutex_unlock(&ar_mmz_lock);
	return z;
}
EXPORT_SYMBOL(hil_mmz_find);

phys_addr_t hil_mmz_get_phys(struct mmz_zone *z)
{
	return z ? z->base : 0;
}
EXPORT_SYMBOL(hil_mmz_get_phys);

/* First-fit over a zone's gaps. Caller holds ar_mmz_lock. */
static struct mmb *zone_alloc_locked(struct mmz_zone *z, const char *name,
				     unsigned long size, unsigned long align)
{
	struct mmb *b, *nb;
	phys_addr_t cur;

	if (!align)
		align = PAGE_SIZE;
	align = max_t(unsigned long, align, PAGE_SIZE);
	size = ALIGN(size, PAGE_SIZE);
	cur = ALIGN(z->base, align);

	/* mmb_list is kept sorted by phys; walk gaps. */
	list_for_each_entry(b, &z->mmb_list, node) {
		if (cur + size <= b->phys)
			break;			/* fits before this block */
		cur = ALIGN(b->phys + b->size, align);
	}
	if (cur + size > z->base + z->size)
		return NULL;			/* out of room */

	nb = kzalloc(sizeof(*nb), GFP_KERNEL);
	if (!nb)
		return NULL;
	strscpy(nb->name, name && name[0] ? name : "mmb", sizeof(nb->name));
	nb->phys = cur;
	nb->size = size;
	nb->align = align;
	nb->zone = z;
	atomic_set(&nb->refcount, 1);
	/* insert keeping sort order (before the block we stopped at, or tail) */
	list_for_each_entry(b, &z->mmb_list, node)
		if (b->phys > nb->phys)
			break;
	list_add_tail(&nb->node, &b->node);
	return nb;
}

struct mmb *hil_mmb_alloc(const char *name, unsigned long size,
			  unsigned long align, u32 gfp, const char *zone_name)
{
	struct mmz_zone *z;
	struct mmb *b;

	mutex_lock(&ar_mmz_lock);
	z = zone_find_by_name(zone_name);
	if (!z) {
		mutex_unlock(&ar_mmz_lock);
		pr_warn("alloc: zone <%s> not found\n", zone_name ? zone_name : "");
		return NULL;
	}
	b = zone_alloc_locked(z, name, size, align);
	mutex_unlock(&ar_mmz_lock);
	if (!b)
		pr_warn("alloc: no room in <%s> for %#lx\n", z->name, size);
	return b;
}
EXPORT_SYMBOL(hil_mmb_alloc);

struct mmb *hil_mmb_alloc_v2(const char *name, unsigned long size,
			     unsigned long align, const char *zone_name,
			     void *priv, u32 gfp)
{
	return hil_mmb_alloc(name, size, align, gfp, zone_name);
}
EXPORT_SYMBOL(hil_mmb_alloc_v2);

int hil_mmb_free(struct mmb *b)
{
	void *k;

	if (!b)
		return -EINVAL;
	mutex_lock(&ar_mmz_lock);
	list_del(&b->node);
	k = b->kvirt;		/* drop any lingering kernel mapping with the block */
	b->kvirt = NULL;
	mutex_unlock(&ar_mmz_lock);
	if (k)
		memunmap(k);
	kfree(b);
	return 0;
}
EXPORT_SYMBOL(hil_mmb_free);

/* ---- kernel mapping: per-block, on demand (write-combine) ----
 *
 * memremap() can sleep (vmalloc + page-table alloc); ar_mmz_lock is a mutex and is
 * never taken from atomic context, so holding it across the map is fine. Blocks are
 * mapped lazily, on demand; never map the whole 108 MiB carveout at init.
 */
static void *__map_block_locked(struct mmb *b)
{
	if (!b->kvirt) {
		void *k = memremap(b->phys, b->size, MEMREMAP_WC);

		if (!k) {
			pr_warn("map2kern: memremap(%pa,+%#zx) failed\n",
				&b->phys, b->size);
			return NULL;
		}
		b->kvirt = k;
	}
	b->map_kref++;
	return b->kvirt;
}

void *hil_mmb_map2kern(struct mmb *b)
{
	void *k;

	if (!b)
		return NULL;
	mutex_lock(&ar_mmz_lock);
	k = __map_block_locked(b);
	mutex_unlock(&ar_mmz_lock);
	return k;
}
EXPORT_SYMBOL(hil_mmb_map2kern);

void *hil_mmb_map2kern_cached(struct mmb *b)
{
	/* TODO: real cached mapping needs exported arm64 cache ops; WC for now. */
	return hil_mmb_map2kern(b);
}
EXPORT_SYMBOL(hil_mmb_map2kern_cached);

/* caller holds ar_mmz_lock */
static void __unmap_block_locked(struct mmb *b)
{
	if (b->map_kref > 0 && --b->map_kref == 0 && b->kvirt) {
		memunmap(b->kvirt);
		b->kvirt = NULL;
	}
}

int hil_mmb_unmap(struct mmb *b)
{
	if (!b)
		return -EINVAL;
	mutex_lock(&ar_mmz_lock);
	__unmap_block_locked(b);
	mutex_unlock(&ar_mmz_lock);
	return 0;
}
EXPORT_SYMBOL(hil_mmb_unmap);

/* map-by-phys fast path: maps the owning block and returns the offset pointer.
 * Reuses the block's mapping + refcount, so hil_mmf_unmap() balances it.
 */
void *hil_mmf_map2kern_cache(phys_addr_t phys, size_t size)
{
	struct mmb *b;
	void *k = NULL;

	mutex_lock(&ar_mmz_lock);
	b = __getby_phys_2(phys, NULL);
	if (b) {
		void *base = __map_block_locked(b);

		if (base)
			k = (u8 *)base + (phys - b->phys);
	}
	mutex_unlock(&ar_mmz_lock);
	return k;
}
EXPORT_SYMBOL(hil_mmf_map2kern_cache);

void *hil_mmf_map2kern_nocache(phys_addr_t phys, size_t size)
{
	return hil_mmf_map2kern_cache(phys, size);
}
EXPORT_SYMBOL(hil_mmf_map2kern_nocache);

int hil_mmf_unmap(phys_addr_t phys)
{
	struct mmb *b;

	mutex_lock(&ar_mmz_lock);
	b = __getby_phys_2(phys, NULL);
	if (b)
		__unmap_block_locked(b);
	mutex_unlock(&ar_mmz_lock);
	return 0;
}
EXPORT_SYMBOL(hil_mmf_unmap);

/* ---- lookup ---- */
/* Lock-free block lookup; caller MUST hold ar_mmz_lock. The returned mmb is only
 * valid while the lock is held (it can be freed once the lock is dropped).
 */
static struct mmb *__getby_phys_2(phys_addr_t phys, struct mmz_zone **zone_out)
{
	struct mmz_zone *z;
	struct mmb *b;

	list_for_each_entry(z, &g_zone_list, node) {
		list_for_each_entry(b, &z->mmb_list, node) {
			if (phys >= b->phys && phys < b->phys + b->size) {
				if (zone_out)
					*zone_out = z;
				return b;
			}
		}
	}
	return NULL;
}

struct mmb *hil_mmb_getby_phys_2(phys_addr_t phys, struct mmz_zone **zone_out)
{
	struct mmb *b;

	mutex_lock(&ar_mmz_lock);
	b = __getby_phys_2(phys, zone_out);
	mutex_unlock(&ar_mmz_lock);
	return b;
}
EXPORT_SYMBOL(hil_mmb_getby_phys_2);

struct mmb *hil_mmb_getby_phys(phys_addr_t phys)
{
	return hil_mmb_getby_phys_2(phys, NULL);
}
EXPORT_SYMBOL(hil_mmb_getby_phys);

struct mmb *hil_mmb_getby_kvirt(const void *kvirt)
{
	struct mmz_zone *z;
	struct mmb *b;

	mutex_lock(&ar_mmz_lock);
	list_for_each_entry(z, &g_zone_list, node) {
		list_for_each_entry(b, &z->mmb_list, node) {
			if (b->kvirt && kvirt >= b->kvirt &&
			    kvirt < (const void *)((u8 *)b->kvirt + b->size)) {
				mutex_unlock(&ar_mmz_lock);
				return b;
			}
		}
	}
	mutex_unlock(&ar_mmz_lock);
	return NULL;
}
EXPORT_SYMBOL(hil_mmb_getby_kvirt);

int hil_mmb_get(struct mmb *b)
{
	return b ? atomic_inc_return(&b->refcount) : -EINVAL;
}

int hil_mmb_put(struct mmb *b)
{
	if (!b)
		return -EINVAL;
	return atomic_dec_return(&b->refcount);
}

/* ---- cache maintenance: WC backing => ordering barrier ---- */
int hil_mmb_flush_dcache_byaddr(void *kvirt, phys_addr_t phys, size_t size)
{
	wmb();	/* drain write-combine buffers before engine reads */
	return 0;
}
EXPORT_SYMBOL(hil_mmb_flush_dcache_byaddr);

int hil_mmb_flush_dcache_byaddr_safe(phys_addr_t phys, void *kvirt, size_t size)
{
	wmb();	/* drain write-combine buffers before engine reads */
	return 0;
}
EXPORT_SYMBOL(hil_mmb_flush_dcache_byaddr_safe);

int hil_mmb_invalid_cache_byaddr(void *kvirt, phys_addr_t phys, size_t size)
{
	rmb();	/* order subsequent CPU reads after engine writes */
	return 0;
}
EXPORT_SYMBOL(hil_mmb_invalid_cache_byaddr);

/* ---- validation ---- */
bool hil_is_phys_in_mmz(phys_addr_t phys, size_t size)
{
	struct mmz_zone *z;
	bool ok;

	mutex_lock(&ar_mmz_lock);
	z = __mmz_find(phys);
	ok = z && (phys + size) <= (z->base + z->size);
	mutex_unlock(&ar_mmz_lock);
	return ok;
}
EXPORT_SYMBOL(hil_is_phys_in_mmz);

int hil_map_mmz_check_phys(phys_addr_t phys, size_t size)
{
	return hil_is_phys_in_mmz(phys, size) ? 0 : -EINVAL;
}
EXPORT_SYMBOL(hil_map_mmz_check_phys);

int hil_vma_check(unsigned long vaddr, size_t size)
{
	return access_ok((void __user *)vaddr, size) ? 0 : -EFAULT;
}
EXPORT_SYMBOL(hil_vma_check);

/* ---- user-map registry / virt->phys ----
 *
 * user_kref is maintained *here*, symmetrically: a successful mmap/REMAP adds a
 * registry entry and bumps the backing mmb's user_kref; the matching unmap (the
 * vma .close - which fires on explicit munmap AND on process exit, covering every
 * teardown path) drops both. This is what makes the 'r',31 PUT auto-free safe: a
 * block that is still user-mapped has user_kref > 0 and is therefore never freed
 * out from under a live mapping.
 */
int ar_mmz_user_map_add(unsigned long uvaddr, phys_addr_t phys, size_t size)
{
	struct user_map *m = kzalloc(sizeof(*m), GFP_KERNEL);
	struct mmb *b;

	if (!m)
		return -ENOMEM;
	m->uvaddr = uvaddr;
	m->phys = phys;
	m->size = size;
	mutex_lock(&ar_mmz_lock);
	list_add(&m->node, &g_user_maps);
	b = __getby_phys_2(phys, NULL);	/* may be NULL for map_mmz regions */
	if (b)
		b->user_kref++;
	mutex_unlock(&ar_mmz_lock);
	return 0;
}
EXPORT_SYMBOL(ar_mmz_user_map_add);

void ar_mmz_user_map_del(unsigned long uvaddr)
{
	struct user_map *m, *t;

	mutex_lock(&ar_mmz_lock);
	list_for_each_entry_safe(m, t, &g_user_maps, node) {
		if (m->uvaddr == uvaddr) {
			struct mmb *b = __getby_phys_2(m->phys, NULL);

			if (b && b->user_kref > 0)
				b->user_kref--;
			list_del(&m->node);
			kfree(m);
			break;
		}
	}
	mutex_unlock(&ar_mmz_lock);
}
EXPORT_SYMBOL(ar_mmz_user_map_del);

phys_addr_t usr_virt_to_phys(unsigned long uvaddr)
{
	struct user_map *m;
	phys_addr_t phys = 0;

	mutex_lock(&ar_mmz_lock);
	list_for_each_entry(m, &g_user_maps, node) {
		if (uvaddr >= m->uvaddr && uvaddr < m->uvaddr + m->size) {
			phys = m->phys + (uvaddr - m->uvaddr);
			break;
		}
	}
	mutex_unlock(&ar_mmz_lock);
	return phys;
}
EXPORT_SYMBOL(usr_virt_to_phys);

/* ------------------------------------------------------------------------ */
/* /dev/mmz_userdev						            */
/* ------------------------------------------------------------------------ */

struct mmz_userdev_info {
	struct mutex	lock;
	struct list_head maps;		/* per-fd struct user_node */
};

struct user_node {
	struct mmb	*b;
	unsigned long	uvaddr;
	struct list_head node;
};

static int mmz_userdev_open(struct inode *ino, struct file *f)
{
	struct mmz_userdev_info *p = kzalloc(sizeof(*p), GFP_KERNEL);

	pr_info("MMZ open by %s (pid %d)\n", current->comm, current->pid);	/* TODO(debug) */
	if (!p)
		return -ENOMEM;
	mutex_init(&p->lock);
	INIT_LIST_HEAD(&p->maps);
	f->private_data = p;
	return 0;
}

static int mmz_userdev_release(struct inode *ino, struct file *f)
{
	struct mmz_userdev_info *p = f->private_data;
	struct user_node *n, *t;

	/* free any per-fd bookkeeping nodes */
	list_for_each_entry_safe(n, t, &p->maps, node) {
		list_del(&n->node);
		kfree(n);
	}
	kfree(p);
	return 0;
	/*
	 * Per-fd auto-free of leaked blocks is intentionally not implemented;
	 * leaks are bounded by the daemon lifetime (carveout reclaimed on module
	 * unload). TODO: fd->mmb tracking needs an mmb->owner_node back-pointer
	 * removed on every free path.
	 */
}

/* mmap contract: vm_pgoff is the requested phys page; validate it lies in
 * a registered MMZ zone, map write-combine, and record it for usr_virt_to_phys.
 */
static void mmz_vma_close(struct vm_area_struct *vma)
{
	ar_mmz_user_map_del(vma->vm_start);
}

static const struct vm_operations_struct mmz_vm_ops = {
	.close = mmz_vma_close,
};

static int mmz_userdev_mmap(struct file *f, struct vm_area_struct *vma)
{
	phys_addr_t phys = (phys_addr_t)vma->vm_pgoff << PAGE_SHIFT;
	size_t size = vma->vm_end - vma->vm_start;

	pr_info("MMZ mmap phys=%pa size=%#zx pgoff=%#lx\n", &phys, size, vma->vm_pgoff);


	/* reject a pgoff so large the phys shift overflows (validation would then
	 * check a wrapped phys while remap_pfn_range maps the real, huge pgoff).
	 */
	if ((phys >> PAGE_SHIFT) != vma->vm_pgoff)
		return -EINVAL;
	if (hil_map_mmz_check_phys(phys, size)) {
		pr_warn("mmap: phys %pa size %#zx outside any MMZ zone\n", &phys, size);
		return -EINVAL;
	}
	/* WC matches the kernel-side memremap(MEMREMAP_WC); avoids attribute mismatch. */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_ops = &mmz_vm_ops;
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size,
			    vma->vm_page_prot)) {
		return -EAGAIN;
	}
	ar_mmz_user_map_add(vma->vm_start, phys, size);
	return 0;
}

/* map an mmb into the calling process via vm_mmap onto our own fd (offset=phys),
 * which routes through mmz_userdev_mmap. Returns the user vaddr.
 */
static long do_user_remap(struct file *f, struct mmb_info *mi, int cached)
{
	unsigned long uaddr;

	uaddr = vm_mmap(f, 0, mi->size, PROT_READ | PROT_WRITE, MAP_SHARED,
			(unsigned long)mi->phys_addr);
	if (IS_ERR_VALUE(uaddr))
		return (long)uaddr;
	mi->mapped_addr = uaddr;
	return 0;
}

static long handle_m(struct file *f, unsigned int nr, void __user *arg,
		     unsigned int sz)
{
	struct mmb_info mi;
	struct mmb *b;
	long ret = 0;

	if (sz != sizeof(mi))
		return -EINVAL;
	if (copy_from_user(&mi, arg, sizeof(mi)))
		return -EFAULT;

	/* TODO(debug, remove after Tier-1 mmz ABI is confirmed): trace the vendor
	 * libhal_sys requests so a struct-offset or zone mismatch shows up as garbage.
	 */
	pr_info("MMZ 'm' nr=%u size=%#llx align=%#llx gfp=%#x flags=%#x zone='%.32s' name='%.48s' phys=%#llx\n",
		nr, mi.size, mi.align, mi.gfp, mi.flags, mi.mmz_name, mi.mmb_name, mi.phys_addr);

	switch (nr) {
	case 10:	/* IOC_MMB_ALLOC */
	case 13: {	/* IOC_MMB_ALLOC_V2 */
		b = hil_mmb_alloc(mi.mmb_name, mi.size, mi.align, mi.gfp,
				  mi.mmz_name);
		if (!b) {
			pr_warn("MMZ ALLOC failed (size=%#llx zone='%.32s')\n", mi.size, mi.mmz_name);
			return -ENOMEM;
		}
		mi.phys_addr = b->phys;
		pr_info("MMZ ALLOC ok phys=%#llx size=%#llx\n", (u64)b->phys, (u64)b->size);
	}
	break;
	case 11: {	/* IOC_MMB_ATTR (get-info) */
		/* lookup + field reads under the lock: b can be freed concurrently. */
		mutex_lock(&ar_mmz_lock);
		b = __getby_phys_2((phys_addr_t)mi.phys_addr, NULL);
		if (!b) {
			mutex_unlock(&ar_mmz_lock);
			return -EINVAL;
		}
		mi.size = b->size;
		mi.align = b->align;
		strscpy(mi.mmb_name, b->name, sizeof(mi.mmb_name));
		strscpy(mi.mmz_name, b->zone->name, sizeof(mi.mmz_name));
		mutex_unlock(&ar_mmz_lock);
	}
	break;
	case 12: {	/* IOC_MMB_FREE */
		/* Atomic lookup+unlink under the lock (no TOCTOU vs a concurrent free),
		 * and refuse if the block is still mapped to avoid freeing live memory.
		 */
		mutex_lock(&ar_mmz_lock);
		b = __getby_phys_2((phys_addr_t)mi.phys_addr, NULL);
		if (!b) {
			mutex_unlock(&ar_mmz_lock);
			return -EINVAL;
		}
		if (b->user_kref > 0 || b->map_kref > 0) {
			mutex_unlock(&ar_mmz_lock);
			return -EBUSY;	/* unmap (ioctl 22 / munmap / kern unmap) first */
		}
		list_del(&b->node);
		mutex_unlock(&ar_mmz_lock);
		kfree(b);
	}
	break;
	case 20: {	/* IOC_MMB_USER_REMAP (uncached) */
		ret = do_user_remap(f, &mi, 0);
	}
	break;
	case 21: {	/* IOC_MMB_USER_REMAP_CACHED */
		ret = do_user_remap(f, &mi, 1);
	}
	break;
	case 22: {	/* IOC_MMB_USER_UNMAP */
		if (mi.mapped_addr) {
			vm_munmap((unsigned long)mi.mapped_addr, mi.size);
			ar_mmz_user_map_del((unsigned long)mi.mapped_addr);
		}
	}
	break;
	case 23: {	/* IOC_MMB_VIRT_GET_PHYS */
		mi.phys_addr = usr_virt_to_phys((unsigned long)mi.mapped_addr);
		if (!mi.phys_addr)
			return -EINVAL;
	}
	break;
	case 24: {	/* IOC_MMZ_QUERY - minimal: resolve one block */
		mutex_lock(&ar_mmz_lock);
		b = __getby_phys_2((phys_addr_t)mi.phys_addr, NULL);
		if (b) {
			mi.size = b->size;
			strscpy(mi.mmz_name, b->zone->name, sizeof(mi.mmz_name));
		}
		mutex_unlock(&ar_mmz_lock);
	}
	break;
	default: {
		return -EINVAL;
	}
	}
	if (ret)
		return ret;
	if (copy_to_user(arg, &mi, sizeof(mi)))
		return -EFAULT;
	return 0;
}

static long handle_r(unsigned int nr, unsigned long handle)
{
	struct mmb *b;
	bool freed = false;
	phys_addr_t phys;
	size_t size;

	/* The whole lookup+refcount+conditional-unlink runs under one lock so the
	 * handle cannot be freed mid-operation and the auto-free decision is atomic.
	 */
	mutex_lock(&ar_mmz_lock);
	b = __getby_phys_2((phys_addr_t)handle, NULL);
	if (!b) {
		mutex_unlock(&ar_mmz_lock);
		return -EINVAL;
	}
	switch (nr) {
	case 30: {	/* get */
		atomic_inc(&b->refcount);
	}
	break;
	case 31: {	/* put, auto-free at 0 iff not mapped (user or kernel) */
		if (atomic_dec_return(&b->refcount) <= 0 &&
		    !b->user_kref && !b->map_kref) {
			list_del(&b->node);
			freed = true;
		}
	}
	break;
	case 40: {	/* flush whole mmb */
		phys = b->phys;
		size = b->size;
		mutex_unlock(&ar_mmz_lock);
		return hil_mmb_flush_dcache_byaddr(NULL, phys, size);
	}
	default: {
	}
	break;		/* lenient: unknown 'r' nr is a no-op */
	}
	mutex_unlock(&ar_mmz_lock);
	if (freed)
		kfree(b);
	return 0;
}

static long mmz_userdev_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct mmz_userdev_info *p = f->private_data;
	unsigned int type = _IOC_TYPE(cmd), nr = _IOC_NR(cmd), sz = _IOC_SIZE(cmd);
	long ret;

	mutex_lock(&p->lock);
	switch (type) {
	case MMZ_IOC_MAGIC_M: {
		ret = handle_m(f, nr, (void __user *)arg, sz);
	}
	break;
	case MMZ_IOC_MAGIC_R: {
		ret = handle_r(nr, arg);
	}
	break;
	case MMZ_IOC_MAGIC_C: {	/* flush-all */
		wmb();
		ret = 0;
	}
	break;
	case MMZ_IOC_MAGIC_D:	/* range clean */
	case MMZ_IOC_MAGIC_I:	/* range invalidate */
	case MMZ_IOC_MAGIC_T: {	/* full-mmb flush */
		/* WC backing => ordering barrier; accept any nr/size for these. */
		mb();
		ret = 0;
	}
	break;
	default: {
		ret = -EINVAL;
	}
	break;
	}
	mutex_unlock(&p->lock);
	return ret;
}

static const struct file_operations mmz_userdev_fops = {
	.owner		= THIS_MODULE,
	.open		= mmz_userdev_open,
	.release	= mmz_userdev_release,
	.unlocked_ioctl	= mmz_userdev_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.mmap		= mmz_userdev_mmap,
};

static struct miscdevice mmz_userdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "mmz_userdev",
	.fops	= &mmz_userdev_fops,
};

/* ------------------------------------------------------------------------ */
/* /proc/media-mem						            */
/* ------------------------------------------------------------------------ */

static int media_mem_show(struct seq_file *s, void *v)
{
	struct mmz_zone *z;
	struct mmb *b;

	mutex_lock(&ar_mmz_lock);
	list_for_each_entry(z, &g_zone_list, node) {
		size_t used = 0;

		list_for_each_entry(b, &z->mmb_list, node)
			used += b->size;
		seq_printf(s, "+-- ZONE: <%s> PHYS(0x%08llx, 0x%08llx) used=%#zx free=%#zx\n",
			   z->name, (u64)z->base, (u64)(z->base + z->size - 1),
			   used, z->size - used);
		list_for_each_entry(b, &z->mmb_list, node)
			seq_printf(s, "    MB: phys(0x%08llx) length=%#zx ref=%d name=%s\n",
				   (u64)b->phys, b->size,
				   atomic_read(&b->refcount), b->name);
	}
	mutex_unlock(&ar_mmz_lock);
	return 0;
}

static int media_mem_open(struct inode *ino, struct file *f)
{
	return single_open(f, media_mem_show, NULL);
}

static const struct proc_ops media_mem_proc_ops = {
	.proc_open	= media_mem_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

/* ------------------------------------------------------------------------ */
/* init / exit							            */
/* ------------------------------------------------------------------------ */

/* Parse mmz= "name,gfp,start,size[,type,eqsize]:[next...]". */
static void parse_mmz_param(const char *s)
{
	char *buf, *p, *tok;

	buf = kstrdup(s, GFP_KERNEL);
	if (!buf)
		return;
	p = buf;
	while ((tok = strsep(&p, ":"))) {
		char *f, *name;
		u64 gfp = 0, start = 0, size = 0;

		if (!*tok)
			continue;
		name = strsep(&tok, ",");
		f = strsep(&tok, ",");
		if (f && kstrtou64(f, 0, &gfp))
			gfp = 0;
		f = strsep(&tok, ",");
		if (f && kstrtou64(f, 0, &start))
			start = 0;
		f = strsep(&tok, ",");
		if (f && kstrtou64(f, 0, &size))
			size = 0;
		if (size)
			hil_mmz_create(name, gfp, start, size);
	}
	kfree(buf);
}

/* Tear down all zones (and any blocks/registry entries). Used by module exit and
 * by the init error path (module_exit is NOT called when init returns failure, so
 * a partial setup must unwind its own zones here).
 */
static void ar_osal_teardown(void)
{
	struct mmz_zone *z, *tz;
	struct mmb *b, *tb;
	struct user_map *m, *tm;

	mutex_lock(&ar_mmz_lock);
	list_for_each_entry_safe(m, tm, &g_user_maps, node) {
		list_del(&m->node);
		kfree(m);
	}
	list_for_each_entry_safe(z, tz, &g_zone_list, node) {
		list_for_each_entry_safe(b, tb, &z->mmb_list, node) {
			list_del(&b->node);
			if (b->kvirt) {		/* drop any on-demand per-block WC mapping */
				memunmap(b->kvirt);
			}
			kfree(b);
		}
		list_del(&z->node);
		kfree(z);		/* z->kvirt is always NULL now (no whole-zone map) */
	}
	mutex_unlock(&ar_mmz_lock);
}

static int __init ar_osal_init(void)
{
	int ret;

	BUILD_BUG_ON(sizeof(struct mmb_info) != 104);
	BUILD_BUG_ON(sizeof(struct mmb_dcache_range) != 24);

	if (strcmp(mmz_allocator, "hisi") && strcmp(mmz_allocator, "cma")) {
		pr_warn("mmz_allocator should be \"cma\" or \"hisi\" (got %s)\n",
			mmz_allocator);
	}

	pr_info("init: creating zones (allocator=%s)\n", mmz_allocator);
	if (mmz) {
		parse_mmz_param(mmz);
	} else if (anony) {
		/*
		 * Only the anonymous pool is backed by reserved-memory on this board
		 * (DTS mmz@29400000, nomap). The vendor's on-chip "sram" zone at
		 * 0x00100000 is NOT reserved here, so we do not create it - and zones
		 * no longer memremap at init anyway, so an unbacked zone would only
		 * fault later when a block from it is mapped. Pass mmz= to override.
		 */
		hil_mmz_create("anonymous", 0, DEF_ANON_BASE, DEF_ANON_SIZE);
	}

	pr_info("init: zones ready, creating /proc/media-mem\n");
	proc_create_data("media-mem", 0444, NULL, &media_mem_proc_ops, NULL);

	pr_info("init: registering /dev/mmz_userdev\n");
	ret = misc_register(&mmz_userdev);
	if (ret) {
		pr_err("misc_register(mmz_userdev) failed: %d\n", ret);
		remove_proc_entry("media-mem", NULL);
		ar_osal_teardown();	/* unwind zones created above (no module_exit on init fail) */
		return ret;
	}
	pr_info("ready (allocator=%s)\n", mmz_allocator);
	return 0;
}

static void __exit ar_osal_exit(void)
{
	misc_deregister(&mmz_userdev);
	remove_proc_entry("media-mem", NULL);
	ar_osal_teardown();
}

module_init(ar_osal_init);
module_exit(ar_osal_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("missinglynk (open reimpl)");
MODULE_DESCRIPTION("Artosyn MMZ allocator + /dev/mmz_userdev (open)");
