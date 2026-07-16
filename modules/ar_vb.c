// SPDX-License-Identifier: GPL-2.0
/*
 * ar_vb.ko - open reimplementation of the Artosyn /dev/ar_vb video-buffer pool.
 * A pure bookkeeping layer over the MMZ allocator (ar_osal): pools are carved
 * from MMZ, blocks handed out / returned, with phys<->handle<->poolid helpers.
 *
 * ABI: magic 'b' (real API) + 't' (alloc/free self-test). The command numbers,
 * struct sizes, field offsets and handle encoding below match the vendor ABI
 * (recovered from the GPL vendor module ar_vb.ko), so the shipped vendor
 * userspace runs against this module unchanged.
 *
 * The vendor ioctl numbers + the module's per-case "VB_xxx: ..." error strings
 * give the nr assignment: SETCONF=0, GETCONF=1, then a 2-slot gap, CRTPL=4,
 * DESTPL=5, GETBLK=6, RLSBLK=7, PA2HL=8, HL2PA=9, HL2PID=10, GETPLINFO=11, ...
 * (full table in the enum below). Sizes are {4,8,40,56,64,80,648}.
 *
 * Handle encoding (proved by HL2PID @ar_vb.ko:0x2534 `lsr w,#16` and the pool-id
 * range checks `cmp #0x1ff`): handle = (pool_id << 16) | block_index, valid iff
 * handle <= 0x1ffffff (pool_id <= 0x1ff = 511, index in the low 16 bits).
 */
#define pr_fmt(fmt) "ar_vb: " fmt

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/bitmap.h>
#include <linux/list.h>
#include <linux/overflow.h>

#include "ar_mmz.h"

#define VB_MAGIC	'b'
#define VB_MAGIC_T	't'

/*
 * 'b' command numbers (vendor ABI: cmd = _IOC(dir,'b',nr,size); nr = cmd & 0xff),
 * recovered from the GPL vendor module ar_vb.ko (jump table @ .rodata+0 + the
 * per-case "VB_xxx:" size-check strings). The shipped vendor userspace uses these
 * unchanged.
 *
 *   nr  operation               size  dir
 *   0   set_config              648   W
 *   1   get_config              648   R
 *   2,3 (reserved - jump slots exist, unused)
 *   4   create_pool             40    W   -> ret = pool_id
 *   5   destroy_pool            4     W
 *   6   get_block_ex           64    RW   -> ret = handle
 *   7   release_block_ex       64    W
 *   8   physaddr2handle         8    W    -> ret = handle
 *   9   handle2physaddr        64    RW   (phys written back @+0x10)
 *   10  handle2poolid           4    W    -> ret = pool_id
 *   11  get_pool_info          80    RW
 *   14  init_modcommpool        4    W
 *   15  exit_modcommpool        4    W
 *   16  set_modpoolconfig      64    W
 *   17  get_modpoolconfig      64    W
 *   18  inquire_usercnt         4    W
 *   19  get_supplementaddr     64    W
 *   20  set_supplementconfig    8    W
 *   21  get_supplementconfig    8    W
 *   22  user_add               64    W
 *   23  user_sub               64    W
 *   24  create_pool_v2         56    W
 *   25  destroy_pool_v2         4    W
 *   26  get_pool_info_v2       80    RW
 *   27  user_add_by_pa         64    W
 *   28  user_sub_by_pa         64    W
 */
enum {
	VB_SETCONF	= 0,
	VB_GETCONF	= 1,
	VB_CRTPL	= 4,
	VB_DESTPL	= 5,
	VB_GETBLK	= 6,
	VB_RLSBLK	= 7,
	VB_PA2HL	= 8,
	VB_HL2PA	= 9,
	VB_HL2PID	= 10,
	VB_GETPLINFO	= 11,
	VB_INITMCPL	= 14,
	VB_EXITMCPL	= 15,
	VB_SETMCPLC	= 16,
	VB_GETMCPLC	= 17,
	VB_INQUCNT	= 18,
	VB_GETSUPPLEADDR = 19,
	VB_SETSUPPLECFG	= 20,
	VB_GETSUPPLECFG	= 21,
	VB_USERADD	= 22,
	VB_USERSUB	= 23,
	VB_CRTPL_V2	= 24,
	VB_DESTPL_V2	= 25,
	VB_GETPLINFO_V2	= 26,
	VB_USERADD_PA	= 27,
	VB_USERSUB_PA	= 28,
	VB_CMD_MAX	= 29,	/* kernel bound: `cmp w,#0x1c; b.ls` (nr <= 28) */
};

/* handle = (pool_id << 16) | block_index. */
#define VB_POOLID_MAX		0x1ff		/* `cmp #0x1ff` pool-id guard */
#define VB_HANDLE_MAX		0x1ffffff	/* `cmp #0x1ffffff` handle guard */
#define VB_HANDLE(pid, idx)	(((u32)(pid) << 16) | ((u32)(idx) & 0xffff))
#define VB_HANDLE_POOLID(h)	((u32)(h) >> 16)
#define VB_HANDLE_INDEX(h)	((u32)(h) & 0xffff)
#define VB_UID_MAX		0x17		/* `cmp #0x17` vb_uid (owner) guard */
#define VB_NAME_LEN		16

/*
 * ---- per-command user structs, field offsets pinned per the cited handler ----
 */

/* VB_CRTPL (nr4, 40B / 0x28). ar_vb.ko handler @0x2d48 -> ar_vb_create_pool
 * @0x1200: reads +0x00 (x2: blk_size, rounded to align then stored as the pool
 * block size), +0x08 (w1: blk_cnt -> multiplied by blk_size for the mmz alloc),
 * +0x0c (w5: comm-pool mode 0/1/2), +0x10 (x3: name, strncpy 16). The created
 * pool_id is the ioctl *return value* (no copy-back; cmd is _IOW).
 */
struct vb_pool_cfg {		/* 40 bytes */
	__u64	blk_size;	/* +0x00 */
	__u32	blk_cnt;	/* +0x08 */
	__u32	pool_type;	/* +0x0c  0 = normal, 1/2 = mod-common-pool modes */
	char	name[VB_NAME_LEN];	/* +0x10 (char[16]) */
	__u8	_resv[8];	/* +0x20 .. 0x28 */
};

/* VB_GETBLK (nr6, 64B) & VB_RLSBLK (nr7, 64B) share this block descriptor.
 * GETBLK (handler @0x2550, ar_vb_get_blk @0x9d8) reads: +0x00 pool_id (-1 =
 * any; else <=0x1ff), +0x08 requested blk_size (must be <= pool's block size),
 * +0x18 vb_uid (owner, <=0x17), +0x20 user ptr to a 16-byte "supplement" blob
 * (copied in separately). The granted handle is the ioctl *return value*.
 * RLSBLK (handler @0x2358, ar_vb_release_blk @0xa78) reads: +0x04 handle
 * (<=0x1ffffff), +0x18 vb_uid.
 */
struct vb_blk {			/* 64 bytes (0x40) */
	__u32	pool_id;	/* +0x00 GETBLK in (-1 = any pool) */
	__u32	handle;		/* +0x04 RLSBLK in (packed pool_id<<16|index) */
	__u64	blk_size;	/* +0x08 GETBLK requested size */
	__u8	_r10[8];	/* +0x10 */
	__u32	vb_uid;		/* +0x18 owner id (<=0x17) */
	__u32	_r1c;
	__u64	suppl_ptr;	/* +0x20 user ptr to 16B supplement (GETBLK) */
	__u8	_r28[24];	/* +0x28 .. 0x40 */
};

/* VB_PA2HL (nr8, 8B). handler @0x2de4 -> ar_vb_phy_to_handle @0x50: reads the
 * phys at +0x00 and returns the packed handle as the ioctl *return value*.
 */
struct vb_pa2hl {		/* 8 bytes */
	__u64	phys;		/* +0x00 in */
};

/* VB_HL2PA (nr9, 64B). handler @0x3008: reads handle at +0x04, writes the
 * resolved phys to +0x10, then copy_to_user (cmd is _IOWR).
 */
struct vb_hl2pa {		/* 64 bytes (0x40) */
	__u32	_r0;		/* +0x00 */
	__u32	handle;		/* +0x04 in */
	__u8	_r8[8];		/* +0x08 */
	__u64	phys;		/* +0x10 out */
	__u8	_r18[40];	/* +0x18 .. 0x40 */
};

/* VB_HL2PID (nr10, 4B). handler @0x2514: reads handle at +0x00, returns
 * pool_id = handle >> 16 as the ioctl *return value*.
 */
struct vb_hl2pid {		/* 4 bytes */
	__u32	handle;		/* +0x00 in */
};

/* VB_DESTPL (nr5, 4B). handler @0x2188 -> ar_vb_destory_pool @0xc70: reads the
 * pool_id at +0x00 (<=0x1ff).
 */
struct vb_destpl {		/* 4 bytes */
	__u32	pool_id;	/* +0x00 in */
};

/* VB_GETPLINFO (nr11, 80B / 0x50). handler @0x23ac: reads pool_id at +0x00,
 * then fills +0x04 blk_cnt, +0x08 blk_cnt (mini/free mirror), +0x10 blk_size,
 * +0x18 total_size (= blk_size*blk_cnt), +0x20 phys base, +0x38 is_comm flag,
 * then copy_to_user (cmd is _IOWR).
 */
struct vb_plinfo {		/* 80 bytes (0x50) */
	__u32	pool_id;	/* +0x00 in */
	__u32	blk_cnt;	/* +0x04 out */
	__u32	blk_cnt2;	/* +0x08 out (vendor mirrors a second count here) */
	__u32	_r0c;
	__u64	blk_size;	/* +0x10 out */
	__u64	total_size;	/* +0x18 out */
	__u64	phys;		/* +0x20 out (pool base) */
	__u8	_r28[16];	/* +0x28 .. 0x38 */
	__u32	is_comm;	/* +0x38 out */
	__u8	_r3c[20];	/* +0x3c .. 0x50 */
};

/* compile-time guards: these sizes are part of the verified ABI. */
static_assert(sizeof(struct vb_pool_cfg) == 40);
static_assert(sizeof(struct vb_blk)      == 64);
static_assert(sizeof(struct vb_pa2hl)    == 8);
static_assert(sizeof(struct vb_hl2pa)    == 64);
static_assert(sizeof(struct vb_hl2pid)   == 4);
static_assert(sizeof(struct vb_destpl)   == 4);
static_assert(sizeof(struct vb_plinfo)   == 80);

#define MAX_POOLS	512		/* pool_id range 0..0x1ff (DESTPL guard) */

struct vb_pool {
	bool		used;
	u32		id;
	struct mmb	*mmb;
	phys_addr_t	base;
	u64		blk_size;
	u32		blk_cnt;
	u32		pool_type;	/* 0 = normal, 1/2 = mod-common-pool */
	unsigned long	*free_map;	/* bit set = free */
	char		name[VB_NAME_LEN];
};

static DEFINE_MUTEX(g_vb_lock);
static struct vb_pool g_pools[MAX_POOLS];
static atomic_t g_ioctl_cnt;

static struct vb_pool *pool_by_id(u32 id)
{
	return (id < MAX_POOLS && g_pools[id].used) ? &g_pools[id] : NULL;
}

static struct vb_pool *pool_by_phys(phys_addr_t phys)
{
	int i;

	for (i = 0; i < MAX_POOLS; i++) {
		if (g_pools[i].used && phys >= g_pools[i].base &&
		    phys < g_pools[i].base + g_pools[i].blk_size * g_pools[i].blk_cnt) {
			return &g_pools[i];
		}
	}
	return NULL;
}

/* VB_CRTPL (nr4). Returns the new pool_id (>=0) or -errno. */
static long vb_crtpl(void __user *arg, unsigned int sz)
{
	struct vb_pool_cfg cfg;
	struct vb_pool *p = NULL;
	u64 total;
	int i;

	if (sz != sizeof(cfg) || copy_from_user(&cfg, arg, sizeof(cfg)))
		return -EFAULT;
	if (!cfg.blk_size || !cfg.blk_cnt || cfg.blk_cnt > 0xffff)
		return -EINVAL;
	/* blk_size is an unchecked user u64: a wrapping product would create a
	 * tiny allocation with large logical pool geometry, and getblk/hl2pa
	 * would hand out physical addresses far past it.
	 */
	if (check_mul_overflow(cfg.blk_size, (u64)cfg.blk_cnt, &total))
		return -EINVAL;
	/* name arrives unterminated from userspace; strscpy over-reads. */
	cfg.name[sizeof(cfg.name) - 1] = '\0';
	for (i = 0; i < MAX_POOLS; i++) {
		if (!g_pools[i].used) {
			p = &g_pools[i];
			break;
		}
	}
	if (!p)
		return -ENOSPC;

	p->mmb = hil_mmb_alloc(cfg.name, total, PAGE_SIZE, 0, "");
	if (!p->mmb)
		return -ENOMEM;
	p->free_map = bitmap_alloc(cfg.blk_cnt, GFP_KERNEL);
	if (!p->free_map) {
		hil_mmb_free(p->mmb);
		return -ENOMEM;
	}
	bitmap_fill(p->free_map, cfg.blk_cnt);
	p->used = true;
	p->id = i;
	p->base = p->mmb->phys;
	p->blk_size = cfg.blk_size;
	p->blk_cnt = cfg.blk_cnt;
	p->pool_type = cfg.pool_type;
	strscpy(p->name, cfg.name, sizeof(p->name));

	/* pool_id is the ioctl return value (cmd is _IOW; no copy-back). */
	return i;
}

/* VB_DESTPL (nr5). */
static long vb_destpl(void __user *arg, unsigned int sz)
{
	struct vb_destpl req;
	struct vb_pool *p;

	if (sz != sizeof(req) || copy_from_user(&req, arg, sizeof(req)))
		return -EFAULT;
	if (req.pool_id > VB_POOLID_MAX)
		return -EINVAL;
	p = pool_by_id(req.pool_id);
	if (!p)
		return -EINVAL;
	hil_mmb_free(p->mmb);
	bitmap_free(p->free_map);
	memset(p, 0, sizeof(*p));
	return 0;
}

/* VB_GETBLK (nr6). Returns the granted handle (>=0) or -errno. */
static long vb_getblk(void __user *arg, unsigned int sz)
{
	struct vb_blk blk;
	struct vb_pool *p = NULL;
	int idx, i;

	if (sz != sizeof(blk) || copy_from_user(&blk, arg, sizeof(blk)))
		return -EFAULT;
	if (blk.vb_uid > VB_UID_MAX)
		return -EINVAL;

	if (blk.pool_id == (u32)-1) {
		/* pool_id == -1: pick any pool with a free block large enough. */
		for (i = 0; i < MAX_POOLS; i++) {
			if (!g_pools[i].used)
				continue;
			if (blk.blk_size && blk.blk_size > g_pools[i].blk_size)
				continue;
			if (!bitmap_empty(g_pools[i].free_map, g_pools[i].blk_cnt)) {
				p = &g_pools[i];
				break;
			}
		}
	} else {
		if (blk.pool_id > VB_POOLID_MAX)
			return -EINVAL;
		p = pool_by_id(blk.pool_id);
	}
	if (!p)
		return -EINVAL;
	if (blk.blk_size && blk.blk_size > p->blk_size)
		return -EINVAL;

	idx = find_first_bit(p->free_map, p->blk_cnt);
	if (idx >= p->blk_cnt)
		return -ENOSPC;
	clear_bit(idx, p->free_map);

	/* handle is the ioctl return value (the HAL returns it directly). */
	return VB_HANDLE(p->id, idx);
}

/* VB_RLSBLK (nr7). */
static long vb_rlsblk(void __user *arg, unsigned int sz)
{
	struct vb_blk blk;
	struct vb_pool *p;
	u32 idx;

	if (sz != sizeof(blk) || copy_from_user(&blk, arg, sizeof(blk)))
		return -EFAULT;
	if (blk.handle > VB_HANDLE_MAX)
		return -EINVAL;
	p = pool_by_id(VB_HANDLE_POOLID(blk.handle));
	if (!p)
		return -EINVAL;
	idx = VB_HANDLE_INDEX(blk.handle);
	if (idx < p->blk_cnt)
		set_bit(idx, p->free_map);
	return 0;
}

/* VB_PA2HL (nr8). Returns the packed handle (>=0) or -errno. */
static long vb_pa2hl(void __user *arg, unsigned int sz)
{
	struct vb_pa2hl x;
	struct vb_pool *p;
	u32 idx;

	if (sz != sizeof(x) || copy_from_user(&x, arg, sizeof(x)))
		return -EFAULT;
	p = pool_by_phys((phys_addr_t)x.phys);
	if (!p)
		return -EINVAL;
	idx = (u32)(((phys_addr_t)x.phys - p->base) / p->blk_size);
	return VB_HANDLE(p->id, idx);
}

/* VB_HL2PA (nr9). Writes phys at +0x10 and copies the struct back. */
static long vb_hl2pa(void __user *arg, unsigned int sz)
{
	struct vb_hl2pa x;
	struct vb_pool *p;
	u32 idx;

	if (sz != sizeof(x) || copy_from_user(&x, arg, sizeof(x)))
		return -EFAULT;
	if (x.handle > VB_HANDLE_MAX)
		return -EINVAL;
	p = pool_by_id(VB_HANDLE_POOLID(x.handle));
	if (!p)
		return -EINVAL;
	idx = VB_HANDLE_INDEX(x.handle);
	if (idx >= p->blk_cnt)
		return -EINVAL;
	x.phys = p->base + (u64)idx * p->blk_size;
	if (copy_to_user(arg, &x, sizeof(x)))
		return -EFAULT;
	return 0;
}

/* VB_HL2PID (nr10). Returns pool_id (>=0) or -errno. */
static long vb_hl2pid(void __user *arg, unsigned int sz)
{
	struct vb_hl2pid x;

	if (sz != sizeof(x) || copy_from_user(&x, arg, sizeof(x)))
		return -EFAULT;
	if (x.handle > VB_HANDLE_MAX)
		return -EINVAL;
	return VB_HANDLE_POOLID(x.handle);
}

/* VB_GETPLINFO (nr11). Fills the pool descriptor and copies it back. */
static long vb_getplinfo(void __user *arg, unsigned int sz)
{
	struct vb_plinfo info;
	struct vb_pool *p;

	if (sz != sizeof(info) || copy_from_user(&info, arg, sizeof(info)))
		return -EFAULT;
	if (info.pool_id > VB_POOLID_MAX)
		return -EINVAL;
	p = pool_by_id(info.pool_id);
	if (!p)
		return -EINVAL;

	memset(&info, 0, sizeof(info));
	info.pool_id = p->id;
	info.blk_cnt = p->blk_cnt;
	info.blk_cnt2 = p->blk_cnt;
	info.blk_size = p->blk_size;
	info.total_size = p->blk_size * p->blk_cnt;
	info.phys = p->base;
	info.is_comm = (p->pool_type != 0);
	if (copy_to_user(arg, &info, sizeof(info)))
		return -EFAULT;
	return 0;
}

static long vb_ioctl_b(unsigned int nr, void __user *arg, unsigned int sz)
{
	atomic_inc(&g_ioctl_cnt);
	switch (nr) {
	case VB_CRTPL: {
		return vb_crtpl(arg, sz);
	}
	case VB_DESTPL: {
		return vb_destpl(arg, sz);
	}
	case VB_GETBLK: {
		return vb_getblk(arg, sz);
	}
	case VB_RLSBLK: {
		return vb_rlsblk(arg, sz);
	}
	case VB_PA2HL: {
		return vb_pa2hl(arg, sz);
	}
	case VB_HL2PA: {
		return vb_hl2pa(arg, sz);
	}
	case VB_HL2PID: {
		return vb_hl2pid(arg, sz);
	}
	case VB_GETPLINFO:
	case VB_GETPLINFO_V2: {
		return vb_getplinfo(arg, sz);
	}
	default: {
		/* SETCONF/GETCONF (648B), the MCPL, USER/SUPPLE and *_V2 create
		 * families: not on the live-video data path. Accept and stub to
		 * success so the vendor userspace's optional calls don't fail hard.
		 * TODO: implement the remaining field layouts and validate against the
		 * vendor userspace (CRTPL_V2 nr24=56B, the 648B VB_CONFIG_S for
		 * SET/GETCONF, the 64B mod-pool/user/supplement descriptors). Sizes are
		 * known; the field layouts of these non-data-path structs are not yet
		 * pinned.
		 */
		return 0;
	}
	}
}

/* 't' self-test: nr0 alloc 0x1000/align8, nr1 free. */
static long vb_ioctl_t(unsigned int nr, unsigned long arg)
{
	static struct mmb *test;

	if (nr == 0) {
		test = hil_mmb_alloc("vb_selftest", 0x1000, 8, 0, "");
		return test ? 0 : -ENOMEM;
	}
	if (nr == 1 && test) {
		hil_mmb_free(test);
		test = NULL;
	}
	return 0;
}

static long ar_vb_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	unsigned int type = _IOC_TYPE(cmd), nr = _IOC_NR(cmd), sz = _IOC_SIZE(cmd);
	long ret;

	mutex_lock(&g_vb_lock);
	switch (type) {
	case VB_MAGIC: {
		ret = vb_ioctl_b(nr, (void __user *)arg, sz);
	}
	break;
	case VB_MAGIC_T: {
		ret = vb_ioctl_t(nr, arg);
	}
	break;
	default: {
		ret = -EINVAL;
	}
	break;
	}
	mutex_unlock(&g_vb_lock);
	return ret;
}

/* read/write dump the pool stats table ("PoolId PA VA ... BlkSz BlkCnt Free"). */
static ssize_t ar_vb_read(struct file *f, char __user *u, size_t n, loff_t *off)
{
	char line[128];
	int i, len = 0;
	size_t copied = 0;

	if (*off)
		return 0;
	mutex_lock(&g_vb_lock);
	for (i = 0; i < MAX_POOLS && copied < n; i++) {
		if (!g_pools[i].used)
			continue;
		len = scnprintf(line, sizeof(line),
				"%2d PA=0x%08llx BlkSz=%llu BlkCnt=%u Free=%u\n",
				g_pools[i].id, (u64)g_pools[i].base,
				g_pools[i].blk_size, g_pools[i].blk_cnt,
				(unsigned int)bitmap_weight(g_pools[i].free_map,
							    g_pools[i].blk_cnt));
		if (copied + len > n)
			break;
		if (copy_to_user(u + copied, line, len)) {
			copied = -EFAULT;
			goto out;
		}
		copied += len;
	}
out:
	mutex_unlock(&g_vb_lock);
	if ((ssize_t)copied > 0)
		*off += copied;
	return copied;
}

static const struct file_operations ar_vb_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= ar_vb_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.read		= ar_vb_read,
};

static struct miscdevice ar_vb_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "ar_vb",
	.fops	= &ar_vb_fops,
};

static int __init ar_vb_init(void)
{
	return misc_register(&ar_vb_dev);
}

static void __exit ar_vb_exit(void)
{
	int i;

	misc_deregister(&ar_vb_dev);
	for (i = 0; i < MAX_POOLS; i++) {
		if (g_pools[i].used) {
			hil_mmb_free(g_pools[i].mmb);
			bitmap_free(g_pools[i].free_map);
		}
	}
}

module_init(ar_vb_init);
module_exit(ar_vb_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("missinglynk (open reimpl)");
MODULE_DESCRIPTION("Artosyn /dev/ar_vb video-buffer pool (open)");
