// SPDX-License-Identifier: GPL-2.0
/*
 * ar_dtbo_sdio.ko - runtime DT overlay adding the mmc@1b00000 (AR8030 SDIO host),
 * mmc@1c00000 (microSD) and gpio@a10a000 nodes under /soc. Same delivery
 * mechanism as ar_mpp_overlay.c; the permanent home for these nodes is
 * dts/proxima-9311.dts.
 *
 * Load order: this module first, then artosyn_gpio (binds gpio@a10a000), then
 * dw_mci-artosyn (binds both mmc nodes).
 *
 * The overlay blob is generated from ar_dtbo_sdio.dts (see ar_dtbo_sdio_dtbo.h).
 */
#define pr_fmt(fmt) "ar_dtbo_sdio: " fmt

#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "ar_dtbo_sdio_dtbo.h"	/* ar_dtbo_sdio_dtbo[] + ar_dtbo_sdio_dtbo_len */

static int ovcs_id;
static void *fdt_copy;

static int __init ar_dtbo_sdio_init(void)
{
	int ret;

	/* of_overlay_fdt_apply / libfdt require an 8-byte-aligned blob; a static
	 * const char array is not guaranteed to be. kmemdup into kmalloc memory
	 * (ARCH_KMALLOC_MINALIGN >= 8) gives the alignment.
	 */
	fdt_copy = kmemdup(ar_dtbo_sdio_dtbo, ar_dtbo_sdio_dtbo_len, GFP_KERNEL);
	if (!fdt_copy)
		return -ENOMEM;
	ret = of_overlay_fdt_apply(fdt_copy, ar_dtbo_sdio_dtbo_len, &ovcs_id, NULL);
	if (ret) {
		pr_err("of_overlay_fdt_apply failed: %d\n", ret);
		kfree(fdt_copy);
		fdt_copy = NULL;
		return ret;
	}
	pr_info("applied mmc0/mmc1/gpio overlay (ovcs id %d); load artosyn_gpio + dw_mci-artosyn now\n",
		ovcs_id);
	return 0;
}

static void __exit ar_dtbo_sdio_exit(void)
{
	of_overlay_remove(&ovcs_id);
	kfree(fdt_copy);
}

module_init(ar_dtbo_sdio_init);
module_exit(ar_dtbo_sdio_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("missinglynk (open reimpl)");
MODULE_DESCRIPTION("Runtime DT overlay: AR8030 SDIO + microSD mmc nodes + gpio controller");
