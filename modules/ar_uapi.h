/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar_uapi.h - the userspace-facing ABI of the open Artosyn kernel modules.
 *
 * THIS is the contract that must stay byte-exact with the shipped closed media
 * libraries (libhal_*, libmpi_*, ar_lowdelay). Every struct layout and ioctl
 * number here is recovered + re-verified from the vendor .ko disassembly -
 * do NOT reorder fields or renumber commands.
 *
 * Covers: /dev/mmz_userdev (ar_osal), /dev/ar_sys (ar_sys). /dev/ar_vb uses its
 * own 'b'/'t' tables (ar_vb.c). /dev/ar_sysctl uses 'S' (ar_sysctl.c).
 */
#ifndef _AR_UAPI_H
#define _AR_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* -------------------------------------------------------------------------
 * ar_osal - /dev/mmz_userdev, magic 'm'/'r'/'c'/'d'/'i'/'t'
 * -------------------------------------------------------------------------
 */

/*
 * struct mmb_info - the 104-byte (0x68) ioctl argument. Offsets are proven byte-exact
 * against this firmware's vendor libhal_sys.so (out/P1_GND/rootfs/usr/lib): the
 * 'm'-magic MMZ ioctls are issued with _IOC_SIZE = 0x68 = 104, not 136. Every offset
 * below is cited to a load/store/strncpy in libhal_sys.
 *
 * Field evidence (addresses in libhal_sys.so):
 *   phys_addr  0x00: ldr [struct+0x00] -> *out at 0x58b0/0x58c0 (alloc), str at 0x6010 (free)
 *   align      0x08: stp x5(align) at 0x6180 (alloc_align, frame+112 = struct+0x08)
 *   size       0x10: str size at 0x5814 (frame+104 = struct+0x10), read back at 0x815c (query)
 *   mapped_addr 0x20: ldr [struct+0x20] -> *out vaddr at 0x58c4/0x58c8 (alloc remap),
 *                     str vaddr at 0x6014 (unmap nr22, frame+104 = struct+0x20)
 *   flags      0x28: strb 0x03 + BFI(bit8) => 0x103 at 0x5820..0x5848 (constant, all alloc paths)
 *   mmb_name   0x30: strncpy(struct+0x30, ., 15) at 0x5808
 *   mmz_name   0x40: strncpy(struct+0x40, ., 31) at 0x57f4
 *   gfp        0x18: never written by the vendor (zeroed); hil_mmb_alloc ignores it, so 0 is fine.
 */
struct mmb_info {
	__u64 phys_addr;	/* 0x00 OUT: allocated phys; IN: handle key for free/remap/etc */
	__u64 align;		/* 0x08 alloc alignment */
	__u64 size;		/* 0x10 alloc size */
	__u32 gfp;		/* 0x18 alloc flags/order (vendor leaves 0; ignored by allocator) */
	__u32 _resv0;		/* 0x1c */
	__u64 mapped_addr;	/* 0x20 OUT: user vaddr from REMAP / kvirt; IN for unmap/virt2phys */
	__u32 flags;		/* 0x28 vendor packed alloc word (0x103); traced only, no logic depends on it */
	__u32 _resv1;		/* 0x2c */
	char  mmb_name[16];	/* 0x30..0x3f block name (vendor strncpy n=15) */
	char  mmz_name[40];	/* 0x40..0x67 zone name (vendor strncpy n=31; "" => anonymous) */
};
/* sizeof must be 104 */

/* 24-byte range descriptor for 'd'/'i' cache-maintenance. */
struct mmb_dcache_range {
	__u64 phys;	/* 0x00 phys or handle */
	__u64 vaddr;	/* 0x08 user vaddr */
	__u32 size;	/* 0x10 byte count */
	__u32 _pad;	/* 0x14 */
};

#define MMZ_IOC_MAGIC_M	'm'
#define MMZ_IOC_MAGIC_R	'r'
#define MMZ_IOC_MAGIC_C	'c'
#define MMZ_IOC_MAGIC_D	'd'
#define MMZ_IOC_MAGIC_I	'i'
#define MMZ_IOC_MAGIC_T	't'

/* 'm' - block alloc/attr/free/remap/unmap/virt2phys/query (arg = struct mmb_info) */
#define IOC_MMB_ALLOC		_IOWR('m', 10, struct mmb_info)
#define IOC_MMB_ATTR		_IOWR('m', 11, struct mmb_info)
#define IOC_MMB_FREE		_IOW('m', 12, struct mmb_info)
#define IOC_MMB_ALLOC_V2	_IOWR('m', 13, struct mmb_info)
#define IOC_MMB_USER_REMAP	_IOWR('m', 20, struct mmb_info)	/* uncached/WC */
#define IOC_MMB_USER_REMAP_CACHED _IOWR('m', 21, struct mmb_info)
#define IOC_MMB_USER_UNMAP	_IOW('m', 22, struct mmb_info)
#define IOC_MMB_VIRT_GET_PHYS	_IOWR('m', 23, struct mmb_info)
#define IOC_MMZ_QUERY		_IOWR('m', 24, struct mmb_info)

/* 'r' - arg is the handle/phys VALUE itself (no struct copy); _IO style.
 * The dispatch keys on _IOC_NR only; we ignore _IOC_SIZE for this magic.
 */
#define IOC_MMB_GET		_IO('r', 30)	/* refcount++ */
#define IOC_MMB_PUT		_IO('r', 31)	/* refcount--, auto-free at 0 if unmapped */
#define IOC_MMB_FLUSH_ALL_DC	_IO('r', 40)	/* flush whole mmb */

/* 'c' - flush-all (arg NULL) */
#define IOC_DCACHE_FLUSH_ALL	_IO('c', 41)

/* 'd'/'i' - range clean / invalidate (arg = struct mmb_dcache_range). The 'i'/'t'
 * NRs were not byte-pinned in RE; for these cache magics the dispatcher accepts
 * the documented NR and treats any other NR under the same magic as a success
 * no-op (WC backing makes range maintenance an ordering barrier).
 */
#define IOC_DCACHE_CLEAN_RANGE	_IOW('d', 50, struct mmb_dcache_range)
#define IOC_DCACHE_INVAL_RANGE	_IOW('i', 50, struct mmb_dcache_range)

/* 't' - full-mmb dirty flush (arg = struct mmb_info, size 0x88). NR not pinned. */
#define IOC_MMB_FLUSH_DC	_IOW('t', 50, struct mmb_info)

/* -------------------------------------------------------------------------
 * ar_sys - /dev/ar_sys, magic 'y' (cache/tz/gps) + 'p' (PTS/loglevel)
 * -------------------------------------------------------------------------
 */

/* 'y',20 flush_cache: 24-byte {u64 arg0; u64 vaddr; u64 size}. */
struct ar_sys_flush_cache {
	__u64 arg0;	/* phys or aux */
	__u64 vaddr;	/* user vaddr */
	__u64 size;	/* byte count (must be 64-aligned per vendor) */
};

/* 'y',25/26 GPS blob: 68 bytes, EXIF-style. Stored/returned opaque; only the
 * ref bytes + rational denominators are validated.
 */
struct ar_sys_gps_info {
	__u8  raw[68];
};

#define AR_SYS_IOC_MAGIC_Y	'y'
#define AR_SYS_IOC_MAGIC_P	'p'

#define IOC_SYS_FLUSH_CACHE	_IOW('y', 20, struct ar_sys_flush_cache)
#define IOC_SYS_SET_TIMEZONE	_IOW('y', 23, __s32)
#define IOC_SYS_GET_TIMEZONE	_IOR('y', 24, __s32)
#define IOC_SYS_SET_GPS		_IOW('y', 25, struct ar_sys_gps_info)
#define IOC_SYS_GET_GPS		_IOR('y', 26, struct ar_sys_gps_info)

#define IOC_SYS_INIT_PTS_BASE	_IOW('p', 21, __u64)
#define IOC_SYS_GET_CUR_PTS	_IOR('p', 22, __u64)
#define IOC_SYS_SYNC_PTS	_IOW('p', 23, __u64)
#define IOC_SYS_GET_PTS_OFFSET	_IOR('p', 24, __u64)
/* 'p',25..29: MPP loglevel/service control - optional, stubbed to success/ENOSYS. */

#endif /* _AR_UAPI_H */
