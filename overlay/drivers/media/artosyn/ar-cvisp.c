// SPDX-License-Identifier: GPL-2.0
/*
 * ar-cvisp.c - Artosyn CVISP output stage, bring-up driver.
 *
 * CVISP is the block at 0x08e00000, ISP base + 0x200000. It is absent from the
 * vendor device tree; the name comes from the cvisp_* stack exported by the
 * vendor's unstripped libmpp_service.so. It is not the DTS scaler@08840000 or
 * gdc@08848000, which are different addresses.
 *
 * It matters because in the vendor's design this block, not the ISP, writes
 * frames to DRAM. That resolves the standing contradiction in the ISP work: the
 * ISP is configured to match the vendor register for register, measurably
 * receives pixels, reaches the same master-control value, and still writes
 * nothing. It was never the writer. Every earlier trace had narrowed the tracer
 * window to one block at a time, while the vendor maps all 256 MiB of register
 * space in a single call, so CVISP was driven throughout and never recorded.
 *
 * This driver applies the recovered configuration and exposes the output queue.
 * Like ar-isp.c it allocates no buffers and offers no V4L2 interface: the tables
 * carry the vendor's own DRAM addresses and the device tree reserves that range
 * no-map so they can be replayed as they are. That is enough to answer whether
 * the block produces a frame, which is what this driver is for.
 *
 * Cadences, from the trace and reflected in the tables:
 *
 *   setup  once, ending at the staged output enable
 *   late   once, just after the enable, with frames already in flight
 *   ring   one Y/U/V triplet per frame, round robin over five buffer sets
 *   tick   eight registers, once per ring wrap rather than once per frame
 *
 * Not implemented here:
 *
 *  - Automatic queueing. The vendor rotates the ring once per frame; here it is
 *    driven by hand through debugfs. A first frame needs no rotation at all,
 *    because the setup table leaves ring set 0 armed.
 *  - The interrupt. libmpp_service.so has cvisp_device_irq_process and
 *    cvisp_dispatch_irq, so the block has its own completion path, but the
 *    hardware IRQ number and its acknowledge register are still behind the
 *    vendor's generic event layer. No interrupt is claimed and none is asserted
 *    in the device tree.
 *  - A reset line. No CVISP reset write appears in the trace and no reset leaf
 *    has been identified, so none is declared rather than inventing one.
 *
 * Configuration provenance is in ar-cvisp-defaults.h. See
 * ../../../../docs/camera-stack.md.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "ar-cvisp-defaults.h"

/*
 * Output control. The vendor stages this 0x00800800 -> 0x00800802 -> 0x00800806
 * at the end of setup and never writes it again; bits 1 and 2 are the launch
 * candidates. The staging is embedded in the setup table rather than driven
 * separately, so that the table alone reproduces the vendor's sequence.
 */
#define AR_CVISP_CONTROL		0x8000

/*
 * Output geometry. Both are written during setup with 0x021c03c0 (540 x 960)
 * and 0x8008 ends at 0x04380780 (1080 x 1920), while the page 0x4000 registers
 * below carry 1920 x 1080 throughout. Which stage is scaled is not established,
 * so these are dumped rather than interpreted.
 */
#define AR_CVISP_OUT_SIZE		0x8008
#define AR_CVISP_OUT_SIZE2		0x8010
#define AR_CVISP_IN_SIZE		0x8028

/* Output plane bases, Y/U/V, rewritten in lockstep once per frame. */
#define AR_CVISP_PLANE_Y		0x8098
#define AR_CVISP_PLANE_U		0x8174
#define AR_CVISP_PLANE_V		0x8194

/* Channel geometry from the late table: width, height, then the crop origin. */
#define AR_CVISP_CH_WIDTH		0x4000
#define AR_CVISP_CH_HEIGHT		0x4004
#define AR_CVISP_CROP_X			0x4100
#define AR_CVISP_CROP_W			0x4104
#define AR_CVISP_CROP_Y			0x4108
#define AR_CVISP_CROP_H			0x410c

/*
 * Off by default: see the probe. The vendor never enables this clock and the
 * boot leaves its gate set, so asserting it is an experiment, not a dependency.
 */
static bool assert_clk;
module_param(assert_clk, bool, 0444);
MODULE_PARM_DESC(assert_clk,
		 "take ownership of cgu_rsz_clk instead of inheriting boot state (default off)");

struct ar_cvisp {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	bool clk_asserted;
	struct dentry *debugfs;
	bool configured;
	unsigned int next;	/* ring slot to arm next */
	unsigned int frames;	/* triplets written since configure */
};

static const struct {
	u16 off;
	const char *name;
} ar_cvisp_dump_regs[] = {
	{ AR_CVISP_CONTROL,	"control" },
	{ AR_CVISP_IN_SIZE,	"in_size" },
	{ AR_CVISP_OUT_SIZE,	"out_size" },
	{ AR_CVISP_OUT_SIZE2,	"out_size2" },
	{ AR_CVISP_PLANE_Y,	"plane_y" },
	{ AR_CVISP_PLANE_U,	"plane_u" },
	{ AR_CVISP_PLANE_V,	"plane_v" },
	{ AR_CVISP_CH_WIDTH,	"ch_width" },
	{ AR_CVISP_CH_HEIGHT,	"ch_height" },
	{ AR_CVISP_CROP_X,	"crop_x" },
	{ AR_CVISP_CROP_W,	"crop_w" },
	{ AR_CVISP_CROP_Y,	"crop_y" },
	{ AR_CVISP_CROP_H,	"crop_h" },
};

static void ar_cvisp_apply(struct ar_cvisp *cv, const struct ar_cvisp_reg *tbl,
			   size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		writel(tbl[i].val, cv->base + tbl[i].off);
}

/* Arm one ring slot. The three plane bases go in the vendor's own order. */
static void ar_cvisp_arm(struct ar_cvisp *cv, unsigned int slot)
{
	const struct ar_cvisp_bufset *b = &ar_cvisp_ring[slot];

	writel(b->y, cv->base + AR_CVISP_PLANE_Y);
	writel(b->u, cv->base + AR_CVISP_PLANE_U);
	writel(b->v, cv->base + AR_CVISP_PLANE_V);
}

/*
 * Advance the output queue by one frame, matching the vendor's cadence: a plane
 * triplet every frame, and the tick group once per wrap of the five-slot ring.
 *
 * The vendor runs this from its own completion path. Here it is manual, so the
 * timing is wrong by construction; what it establishes is whether rotation is
 * needed for a second frame at all, not that this is how a driver should do it.
 */
static void ar_cvisp_queue(struct ar_cvisp *cv)
{
	ar_cvisp_arm(cv, cv->next);
	cv->next = (cv->next + 1) % ARRAY_SIZE(ar_cvisp_ring);
	cv->frames++;

	if (!cv->next)
		ar_cvisp_apply(cv, ar_cvisp_tick, ARRAY_SIZE(ar_cvisp_tick));
}

/*
 * Apply the whole recovered configuration.
 *
 * The setup table runs in write order and ends with the staged enable, so the
 * block is live once it returns, with ring set 0 armed. The late table is what
 * the vendor writes immediately afterwards, with its first frames already in
 * flight: the arbitration table on page 0x0000 and the channel geometry on page
 * 0x4000. Whether that ordering is required or merely what the vendor's
 * threading produced is not established, which is why the tables are separate
 * and the late one can be held back.
 */
static void ar_cvisp_configure(struct ar_cvisp *cv, bool late)
{
	ar_cvisp_apply(cv, ar_cvisp_setup, ARRAY_SIZE(ar_cvisp_setup));

	if (late)
		ar_cvisp_apply(cv, ar_cvisp_late, ARRAY_SIZE(ar_cvisp_late));

	/* Setup leaves ring set 0 armed, so the next rotation starts at 1. */
	cv->next = 1 % ARRAY_SIZE(ar_cvisp_ring);
	cv->frames = 1;
	cv->configured = true;

	dev_info(cv->dev,
		 "configured: %zu setup + %zu late, control 0x%08x, plane_y 0x%08x\n",
		 ARRAY_SIZE(ar_cvisp_setup), late ? ARRAY_SIZE(ar_cvisp_late) : 0,
		 readl(cv->base + AR_CVISP_CONTROL),
		 readl(cv->base + AR_CVISP_PLANE_Y));
}

/*
 * Writing 1 applies setup and late; 2 applies setup only, to test whether the
 * late tail has to follow the enable or can be held back.
 */
static int ar_cvisp_configure_set(void *data, u64 val)
{
	struct ar_cvisp *cv = data;

	if (val != 1 && val != 2)
		return -EINVAL;

	ar_cvisp_configure(cv, val == 1);
	return 0;
}

static int ar_cvisp_configure_get(void *data, u64 *val)
{
	struct ar_cvisp *cv = data;

	*val = cv->configured;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ar_cvisp_configure_fops, ar_cvisp_configure_get,
			 ar_cvisp_configure_set, "%llu\n");

/* Writing n advances the queue n frames. Reading reports frames armed so far. */
static int ar_cvisp_queue_set(void *data, u64 val)
{
	struct ar_cvisp *cv = data;

	if (!cv->configured)
		return -EAGAIN;

	while (val--)
		ar_cvisp_queue(cv);

	return 0;
}

static int ar_cvisp_queue_get(void *data, u64 *val)
{
	struct ar_cvisp *cv = data;

	*val = cv->frames;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ar_cvisp_queue_fops, ar_cvisp_queue_get,
			 ar_cvisp_queue_set, "%llu\n");

static int ar_cvisp_regs_show(struct seq_file *s, void *unused)
{
	struct ar_cvisp *cv = s->private;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ar_cvisp_dump_regs); i++)
		seq_printf(s, "%-12s 0x%04x 0x%08x\n", ar_cvisp_dump_regs[i].name,
			   ar_cvisp_dump_regs[i].off,
			   readl(cv->base + ar_cvisp_dump_regs[i].off));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ar_cvisp_regs);

static int ar_cvisp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ar_cvisp *cv;
	int ret;

	cv = devm_kzalloc(dev, sizeof(*cv), GFP_KERNEL);
	if (!cv)
		return -ENOMEM;

	cv->dev = dev;

	cv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cv->base))
		return PTR_ERR(cv->base);

	/*
	 * cgu_rsz_clk, referenced but deliberately NOT enabled by default.
	 *
	 * The evidence says the vendor never enables it: there is no clock
	 * request anywhere in the CVISP path in libmpp_service.so, the trace
	 * contains no CGU write for this block, and stock-A baselines read
	 * 0x12011100 at 0x0a104014 with gate bit 12 already set. The boot
	 * firmware leaves it on and the vendor inherits it.
	 *
	 * Taking ownership would be actively harmful on the way out: the leaf is
	 * gate-modelled, so clk_disable_unprepare on remove would clear a gate
	 * the boot had set and leave the block unclocked for whatever touches it
	 * next. Register access with the clock gated hangs the SoC on this
	 * family. So the reference is kept as an annotated hypothesis, and
	 * asserting it is opt-in.
	 */
	cv->clk = devm_clk_get_optional(dev, "rsz");
	if (IS_ERR(cv->clk))
		return dev_err_probe(dev, PTR_ERR(cv->clk), "bad rsz clock\n");

	if (assert_clk && cv->clk) {
		ret = clk_prepare_enable(cv->clk);
		if (ret)
			return dev_err_probe(dev, ret, "cannot enable rsz clock\n");
		cv->clk_asserted = true;
	}

	platform_set_drvdata(pdev, cv);

	cv->debugfs = debugfs_create_dir("ar-cvisp", NULL);
	debugfs_create_file_unsafe("configure", 0600, cv->debugfs, cv,
				   &ar_cvisp_configure_fops);
	debugfs_create_file_unsafe("queue", 0600, cv->debugfs, cv,
				   &ar_cvisp_queue_fops);
	debugfs_create_file("regs", 0400, cv->debugfs, cv, &ar_cvisp_regs_fops);

	/*
	 * Deliberately no register read here. If the clock assumption above is
	 * wrong, the first access hangs the SoC into a watchdog reset, and a
	 * probe-time read would make that unavoidable on every boot. Reading
	 * debugfs regs is the first touch, and it is a deliberate one.
	 */
	dev_info(dev, "probed, %zu setup + %zu late registers, %zu ring slots\n",
		 ARRAY_SIZE(ar_cvisp_setup), ARRAY_SIZE(ar_cvisp_late),
		 ARRAY_SIZE(ar_cvisp_ring));

	return 0;
}

static void ar_cvisp_remove(struct platform_device *pdev)
{
	struct ar_cvisp *cv = platform_get_drvdata(pdev);

	debugfs_remove_recursive(cv->debugfs);

	/* Only ever release what this driver asserted. Clearing an inherited gate
	 * leaves the block unclocked for the next thing that touches it.
	 */
	if (cv->clk_asserted)
		clk_disable_unprepare(cv->clk);
}

static const struct of_device_id ar_cvisp_of_match[] = {
	{ .compatible = "artosyn,cvisp" },
	{ }
};
MODULE_DEVICE_TABLE(of, ar_cvisp_of_match);

static struct platform_driver ar_cvisp_driver = {
	.probe = ar_cvisp_probe,
	.remove = ar_cvisp_remove,
	.driver = {
		.name = "ar-cvisp",
		.of_match_table = ar_cvisp_of_match,
	},
};
module_platform_driver(ar_cvisp_driver);

MODULE_DESCRIPTION("Artosyn CVISP output stage");
MODULE_LICENSE("GPL");
