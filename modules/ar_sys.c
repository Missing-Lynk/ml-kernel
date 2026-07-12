// SPDX-License-Identifier: GPL-2.0
/*
 * ar_sys.ko - open reimplementation of the Artosyn /dev/ar_sys char device:
 * MMZ cache flush + timezone + GPS ('y') and the PTS/time-base + MPP loglevel
 * ('p') interface used by the media pipeline.
 *
 * ABI recovered byte-exact from the vendor disassembly ('y'/'p' tables). The PTS unit
 * is microseconds of sched_clock(), re-anchored by a userspace-supplied base.
 * Depends on ar_osal for MMZ validation + virt->phys.
 */
#define pr_fmt(fmt) "ar_sys: " fmt

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/sched/clock.h>
#include <linux/ktime.h>

#include "ar_mmz.h"
#include "ar_uapi.h"

static DEFINE_MUTEX(g_sys_lock);

/* ---- PTS state ---- */
static u64 pts_base_val;		/* userspace-anchored base (µs) */
static u64 pts_base_pts;		/* local PTS snapshot at anchor time */
static u64 pts_last_returned;		/* monotonic guard */
static u64 pts_wall_offset;		/* base_val*1000 - ktime_get_raw() (ns) */

static s32 g_time_zone;			/* seconds, ±86400 */
static struct ar_sys_gps_info g_gps;

/* sched_clock ns -> µs, refusing to go backwards (re get_local_curPTs @0xa8). */
static u64 get_local_cur_pts(void)
{
	static u64 last;
	u64 us = div_u64(sched_clock(), 1000);

	if (us < last)
		return last;
	last = us;
	return us;
}

/* ---- 'y' set ---- */
static long handle_y(unsigned int nr, void __user *arg, unsigned int sz)
{
	switch (nr) {
	case 20: {	/* flush_cache: 24-byte {arg0, vaddr, size} */
		struct ar_sys_flush_cache fc;
		phys_addr_t phys;

		if (sz != sizeof(fc) || copy_from_user(&fc, arg, sizeof(fc)))
			return -EFAULT;
		if (fc.size & 0x3f) {		/* vendor requires 64-byte alignment */
			return -EINVAL;
		}
		phys = usr_virt_to_phys((unsigned long)fc.vaddr);
		if (!phys)
			return -EINVAL;
		return hil_mmb_flush_dcache_byaddr_safe(phys, NULL, fc.size);
	}
	case 23: {	/* set timezone (i32 seconds, ±86400) */
		s32 tz;

		if (sz != sizeof(tz) || copy_from_user(&tz, arg, sizeof(tz)))
			return -EFAULT;
		/* ±86400s. Compare directly: "tz + 86400" can overflow s32 (UB) for
		 * a hostile tz near INT_MAX; this form is equivalent and overflow-free.
		 */
		if (tz < -86400 || tz > 86400)
			return -EINVAL;
		g_time_zone = tz;
		return 0;
	}
	case 24: {	/* get timezone */
		if (sz != sizeof(s32) ||
		    copy_to_user(arg, &g_time_zone, sizeof(s32))) {
			return -EFAULT;
		}
		return 0;
	}
	case 25: {	/* set GPS (68-byte blob) */
		struct ar_sys_gps_info g;

		if (sz != sizeof(g) || copy_from_user(&g, arg, sizeof(g)))
			return -EFAULT;
		/* validate refs: lat_ref@0='N'/'S', lon_ref@0x1c='E'/'W',
		 * alt_ref@0x38<=1. Blob otherwise stored opaque.
		 */
		if (g.raw[0] != 'N' && g.raw[0] != 'S')
			return -EINVAL;
		if (g.raw[0x1c] != 'E' && g.raw[0x1c] != 'W')
			return -EINVAL;
		if (g.raw[0x38] > 1)
			return -EINVAL;
		g_gps = g;
		return 0;
	}
	case 26: {	/* get GPS */
		if (sz != sizeof(g_gps) ||
		    copy_to_user(arg, &g_gps, sizeof(g_gps))) {
			return -EFAULT;
		}
		return 0;
	}
	default: {
		return -EINVAL;
	}
	}
}

/* ---- 'p' set (PTS) ---- */
static long handle_p(unsigned int nr, void __user *arg, unsigned int sz)
{
	switch (nr) {
	case 21: {	/* init_pts_base */
		u64 base;

		if (sz != sizeof(base) || copy_from_user(&base, arg, sizeof(base)))
			return -EFAULT;
		pts_base_val = base;
		pts_base_pts = get_local_cur_pts();
		pts_wall_offset = base * 1000 - ktime_get_raw_ns();
		return 0;
	}
	case 23: {	/* sync_pts (re-anchor) */
		u64 base;

		if (sz != sizeof(base) || copy_from_user(&base, arg, sizeof(base)))
			return -EFAULT;
		pts_base_val = base;
		pts_base_pts = get_local_cur_pts();
		pts_wall_offset = base * 1000 - ktime_get_raw_ns();
		return 0;
	}
	case 22: {	/* get_cur_pts */
		u64 pts = (pts_base_val - pts_base_pts) + get_local_cur_pts();

		if (pts < pts_last_returned)
			pts = pts_last_returned;
		pts_last_returned = pts;
		if (sz != sizeof(pts) || copy_to_user(arg, &pts, sizeof(pts)))
			return -EFAULT;
		return 0;
	}
	case 24: {	/* get_pts_offset */
		if (sz != sizeof(pts_wall_offset) ||
		    copy_to_user(arg, &pts_wall_offset, sizeof(pts_wall_offset))) {
			return -EFAULT;
		}
		return 0;
	}
	case 25:	/* loglevel get */
	case 26:	/* loglevel set */
	case 27:	/* loglevel by tag */
	case 28:	/* dump all loglevels */
	case 29: {	/* mpp_service query */
		/* MPP debug-log subsystem - non-critical for video, stubbed. */
		return 0;
	}
	default: {
		return -EINVAL;
	}
	}
}

static long ar_sys_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	unsigned int type = _IOC_TYPE(cmd), nr = _IOC_NR(cmd), sz = _IOC_SIZE(cmd);
	long ret;

	mutex_lock(&g_sys_lock);
	switch (type) {
	case AR_SYS_IOC_MAGIC_Y: {
		ret = handle_y(nr, (void __user *)arg, sz);
	}
	break;
	case AR_SYS_IOC_MAGIC_P: {
		ret = handle_p(nr, (void __user *)arg, sz);
	}
	break;
	default: {
		ret = -1;	/* vendor returns -1 for unknown magic */
	}
	break;
	}
	mutex_unlock(&g_sys_lock);
	return ret;
}

/* The registry entry added on mmap MUST be removed when the vma goes away (explicit
 * munmap or process exit), else g_user_maps leaks stale vaddr->phys entries (which
 * could later alias a reused vaddr) and the backing mmb's user_kref never drops so
 * it can never be freed. The mmz_userdev path has this; ar_sys must too.
 */
static void ar_sys_vma_close(struct vm_area_struct *vma)
{
	ar_mmz_user_map_del(vma->vm_start);
}

static const struct vm_operations_struct ar_sys_vm_ops = {
	.close = ar_sys_vma_close,
};

/* mmap: same MMZ-validated contract as mmz_userdev, always cached-equivalent (WC). */
static int ar_sys_mmap(struct file *f, struct vm_area_struct *vma)
{
	phys_addr_t phys = (phys_addr_t)vma->vm_pgoff << PAGE_SHIFT;
	size_t size = vma->vm_end - vma->vm_start;

	if ((phys >> PAGE_SHIFT) != vma->vm_pgoff)
		return -EINVAL;
	if (hil_map_mmz_check_phys(phys, size))
		return -EINVAL;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_ops = &ar_sys_vm_ops;
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size,
			    vma->vm_page_prot)) {
		return -EAGAIN;
	}
	ar_mmz_user_map_add(vma->vm_start, phys, size);
	return 0;
}

static const struct file_operations ar_sys_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= ar_sys_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.mmap		= ar_sys_mmap,
};

static struct miscdevice ar_sys_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "ar_sys",
	.fops	= &ar_sys_fops,
};

static int __init ar_sys_init(void)
{
	return misc_register(&ar_sys_dev);
}

static void __exit ar_sys_exit(void)
{
	misc_deregister(&ar_sys_dev);
}

module_init(ar_sys_init);
module_exit(ar_sys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("missinglynk (open reimpl)");
MODULE_DESCRIPTION("Artosyn /dev/ar_sys: PTS/timezone/GPS/cache (open)");
