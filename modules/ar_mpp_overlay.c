// SPDX-License-Identifier: GPL-2.0
/*
 * ar_mpp_overlay.ko - applies a small DT overlay that adds the ahb_dma (8 ch) and
 * axi_dma (3 ch) engines as children of the live ar_mpp node. ar_mpp_drv's nr6
 * driver-info query reports those children's IRQ counts, which libmpp_service asserts
 * on during SYS_Init (total_irq_num == 8 / == 3). Load it BEFORE ar_mpp_drv so the
 * driver enumerates the children at probe. The permanent home for this topology is
 * the in-tree dts/proxima-9311.dts.
 *
 * The overlay blob is generated from ar_mpp_overlay.dts (see ar_mpp_overlay_dtbo.h).
 */
#define pr_fmt(fmt) "ar_mpp_overlay: " fmt

#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "ar_mpp_overlay_dtbo.h"	/* ar_mpp_overlay_dtbo[] + ar_mpp_overlay_dtbo_len */

static int ovcs_id;
static void *fdt_copy;

static int __init ar_mpp_overlay_init(void)
{
	int ret;

	/* of_overlay_fdt_apply / libfdt require the blob to be 8-byte aligned; a static
	 * const char array is not guaranteed to be, which fails fdt_check_header (-EINVAL).
	 * kmemdup into kmalloc memory (ARCH_KMALLOC_MINALIGN >= 8) gives the alignment.
	 */
	fdt_copy = kmemdup(ar_mpp_overlay_dtbo, ar_mpp_overlay_dtbo_len, GFP_KERNEL);
	if (!fdt_copy)
		return -ENOMEM;
	ret = of_overlay_fdt_apply(fdt_copy, ar_mpp_overlay_dtbo_len, &ovcs_id, NULL);
	if (ret) {
		pr_err("of_overlay_fdt_apply failed: %d\n", ret);
		kfree(fdt_copy);
		fdt_copy = NULL;
		return ret;
	}
	pr_info("applied ar_mpp ahb_dma/axi_dma overlay (ovcs id %d); load ar_mpp_drv now\n",
		ovcs_id);
	return 0;
}

static void __exit ar_mpp_overlay_exit(void)
{
	of_overlay_remove(&ovcs_id);
	kfree(fdt_copy);
}

module_init(ar_mpp_overlay_init);
module_exit(ar_mpp_overlay_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("missinglynk (open reimpl)");
MODULE_DESCRIPTION("Runtime DT overlay: ar_mpp ahb_dma/axi_dma children for the nr6 query");
