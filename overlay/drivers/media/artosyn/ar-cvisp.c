// SPDX-License-Identifier: GPL-2.0
/*
 * ar-cvisp.c - Artosyn CVISP output stage, bring-up driver.
 *
 * CVISP is the block at 0x08e00000, ISP base + 0x200000. It is absent from the
 * vendor device tree; the name comes from the cvisp_* stack exported by the
 * vendor's unstripped libmpp_service.so. It is not the DTS scaler@08840000 or
 * gdc@08848000. In the vendor's design this block, not the ISP, writes frames
 * to DRAM; the ISP feeds it and CVISP owns the output queue.
 *
 * This driver applies the recovered configuration and exposes the output queue.
 * Like ar-isp.c it allocates no buffers and offers no V4L2 interface: the tables
 * carry the vendor's own DRAM addresses and the device tree reserves that range
 * no-map so they can be replayed as they are. That is enough to answer whether
 * the block produces a frame.
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
 *    driven by hand through debugfs. A first frame needs no rotation, because
 *    the setup table leaves ring set 0 armed.
 *  - The interrupt. The block has its own completion path (cvisp_dispatch_irq
 *    in libmpp_service.so), but the IRQ number and acknowledge register are
 *    still behind the vendor's generic event layer. None is claimed.
 *  - A reset line. No CVISP reset write appears in the trace and no reset leaf
 *    has been identified, so none is declared.
 *
 * Configuration provenance is in ar-cvisp-defaults.h. See
 * ../../../../docs/camera-stack.md.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "ar-cvisp-defaults.h"
#include "ar-isp-blc.h"

/*
 * The tuning file BLC reads. Same file ar-isp loads; requested here rather than
 * shared because the two drivers own different blocks, and it is released as
 * soon as the 64-byte block has been built.
 */
#define AR_CVISP_TUNING_FIRMWARE	"artosyn/nt99235-tuning-preview-fpv.bin"

static bool blc = true;
module_param(blc, bool, 0644);
MODULE_PARM_DESC(blc,
		 "generate black level correction from the tuning file (default on)");

/*
 * Sensor gain in the tuning file's ladder units, which span 1 to 2100. The
 * default is the vendor's traced operating point and reproduces its registers
 * exactly; auto-exposure will drive this once it owns gain.
 */
static unsigned int blc_gain = 187;
module_param(blc_gain, uint, 0644);
MODULE_PARM_DESC(blc_gain,
		 "sensor gain BLC blends for, in ladder units (default 187, the traced point)");

/*
 * Output control. The vendor stages this 0x00800800 -> 0x00800802 -> 0x00800806
 * at the end of setup and never writes it again; bits 1 and 2 are undecoded.
 * The staging is embedded in the setup table so the table alone reproduces the
 * vendor's sequence.
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
 * Fixed-pattern-noise control word. The stage's only writer in the vendor
 * library clears bit 4 on every tuning apply (0x1b125c, commands 0xb10 and
 * 0xb13) and nothing anywhere sets it, so clear is the vendor's steady state.
 * The stage has no init fill path, its DMA correction surface is never armed,
 * and no replay table or sweep covered the register, so before this write a
 * cold boot ran on the hardware reset value.
 */
#define AR_CVISP_FPN_CTRL		0x4400
#define AR_CVISP_FPN_ENABLE		BIT(4)

static bool fpn = true;
module_param(fpn, bool, 0644);
MODULE_PARM_DESC(fpn,
		 "clear fpn's enable bit as the vendor's tuning apply does (default on)");

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
	for (size_t i = 0; i < n; i++)
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
 * The vendor runs this from its completion path; here it is manual, so the
 * timing is wrong by construction. It establishes whether rotation is needed
 * for a second frame, nothing more.
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
 * A float32 from the tuning file as a truncated unsigned integer.
 *
 * The kernel has no FPU, so the ladder's bounds are decoded from their IEEE-754
 * bit patterns. Every value in this ladder is a small positive integer, so the
 * general cases only exist to keep a corrupt file from producing nonsense.
 */
static u32 ar_cvisp_f32_to_u32(u32 bits)
{
	u32 mant = (bits & 0x7fffff) | 0x800000;
	int exp = (int)((bits >> 23) & 0xff) - 127;

	if (bits & 0x80000000 || exp < 0)
		return 0;
	if (exp > 30)
		return U32_MAX;
	if (exp >= 23)
		return mant << (exp - 23);

	return mant >> (23 - exp);
}

/*
 * Black level correction, from the tuning file, as a function of sensor gain.
 *
 * The stage is sixteen registers on this block filled by a verbatim 64-byte
 * copy. Its payload is five calibration entries selected by a ladder of float
 * pairs: a gain inside a pair's own range uses that entry alone, and a gain
 * between one pair's second bound and the next pair's first bound blends the
 * two entries across that band. Recovered from the selection at 0x1bfef8 and
 * the blend at 0x1c0048; formats are in ar-isp-blc.h.
 *
 *	t = (gain - ladder[lo].second) / (ladder[hi].first - ladder[lo].second)
 *	out = entry[hi] * t + entry[lo] * (1 - t)
 *
 * The default gain reproduces the vendor exactly. At 187 the band is 130 to
 * 510, t is 0.150000, and all four lanes come out at 961 and 272, which are the
 * 0xf040 and 0x110 the vendor was traced writing. That is the whole reason this
 * runs after the late table rather than instead of it: the constants that table
 * carries are the answer for one operating point, and this reproduces them from
 * the tuning file instead of carrying them.
 *
 * BLC recomputes with gain on the vendor, so once auto-exposure moves gain this
 * has to be re-applied with it. Nothing does that yet, which is safe only while
 * gain is static.
 */
static void ar_cvisp_blc_apply(struct ar_cvisp *cv)
{
	const struct firmware *fw;
	struct ar_isp_blc_entry lo, hi;
	u8 block[AR_ISP_BLC_BLOCK];
	u32 bound_lo, bound_hi, blend;
	unsigned int i, sel = 0;
	const u8 *ladder;
	int ret;

	if (!blc)
		return;

	ret = request_firmware(&fw, AR_CVISP_TUNING_FIRMWARE, cv->dev);
	if (ret) {
		dev_info(cv->dev, "blc: no %s (%d), leaving the replayed constants\n",
			 AR_CVISP_TUNING_FIRMWARE, ret);
		return;
	}

	if (fw->size < AR_ISP_BLC_BLOB_TABLE +
		       AR_ISP_BLC_ENTRIES * AR_ISP_BLC_ENTRY_SIZE) {
		dev_warn(cv->dev, "blc: tuning file too short, leaving the replayed constants\n");
		release_firmware(fw);
		return;
	}

	ladder = fw->data + AR_ISP_BLC_BLOB_LADDER;

	/*
	 * Find the band the gain sits in. Walking upwards and stopping at the
	 * first pair whose second bound is above the gain puts a gain inside a
	 * pair's own range on that pair, where the two indices coincide and the
	 * entry is used without blending, exactly as the vendor's equal-index
	 * case does.
	 */
	for (i = 0; i + 1 < AR_ISP_BLC_ENTRIES; i++) {
		if (blc_gain < ar_cvisp_f32_to_u32(ar_isp_get_le32(ladder + i * 8 + 4)))
			break;
		sel = i;
	}

	bound_lo = ar_cvisp_f32_to_u32(ar_isp_get_le32(ladder + sel * 8 + 4));
	bound_hi = ar_cvisp_f32_to_u32(ar_isp_get_le32(ladder + (sel + 1) * 8));

	ar_isp_blc_entry(fw->data, sel, &lo);
	ar_isp_blc_entry(fw->data, sel + 1, &hi);

	/*
	 * ar_isp_blc_mix weights the low entry, so it takes 1 - t. A gain below
	 * the band uses the low entry alone and one above it the high entry,
	 * which is what clamping the weight to the Q16 endpoints does.
	 */
	if (blc_gain <= bound_lo || bound_hi <= bound_lo)
		blend = AR_ISP_BLC_BLEND_ONE;
	else if (blc_gain >= bound_hi)
		blend = 0;
	else
		blend = AR_ISP_BLC_BLEND_ONE -
			(u32)div_u64((u64)(blc_gain - bound_lo) * AR_ISP_BLC_BLEND_ONE,
				     bound_hi - bound_lo);

	ar_isp_blc_fill(block, &lo, &hi, blend);

	for (i = 0; i < AR_ISP_BLC_BLOCK; i += 4)
		writel(ar_isp_get_le32(block + i), cv->base + AR_ISP_BLC_BANK + i);

	dev_info(cv->dev,
		 "blc: gain %u in band %u..%u, entries %u/%u, scale 0x%08x level 0x%08x\n",
		 blc_gain, bound_lo, bound_hi, sel, sel + 1,
		 readl(cv->base + AR_ISP_BLC_BANK + AR_ISP_BLC_REG_SCALE),
		 readl(cv->base + AR_ISP_BLC_BANK + AR_ISP_BLC_REG_LEVEL));

	release_firmware(fw);
}

/*
 * Apply the whole recovered configuration. The setup table ends with the staged
 * enable, so the block is live once it returns, with ring set 0 armed. The late
 * table follows with frames already in flight; whether that ordering is
 * required is not established, which is why it can be held back.
 */
static void ar_cvisp_configure(struct ar_cvisp *cv, bool late)
{
	ar_cvisp_apply(cv, ar_cvisp_setup, ARRAY_SIZE(ar_cvisp_setup));

	if (late)
		ar_cvisp_apply(cv, ar_cvisp_late, ARRAY_SIZE(ar_cvisp_late));

	/* After the late table, which carries the vendor's own BLC constants. */
	ar_cvisp_blc_apply(cv);

	if (fpn)
		writel(readl(cv->base + AR_CVISP_FPN_CTRL) & ~AR_CVISP_FPN_ENABLE,
		       cv->base + AR_CVISP_FPN_CTRL);

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

	for (unsigned int i = 0; i < ARRAY_SIZE(ar_cvisp_dump_regs); i++)
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
	 * cgu_rsz_clk, referenced but deliberately NOT enabled by default. The
	 * vendor never enables it: no clock request in the CVISP path of
	 * libmpp_service.so, no CGU write in the trace, and stock-A baselines
	 * read 0x12011100 at 0x0a104014 with gate bit 12 already set by the
	 * boot firmware.
	 *
	 * Taking ownership would be harmful on the way out: the leaf is
	 * gate-modelled, so clk_disable_unprepare on remove would clear a gate
	 * the boot had set, and register access with the clock gated hangs the
	 * SoC on this family. Asserting it is opt-in.
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
	 * wrong, the first access hangs the SoC, and a probe-time read would
	 * make that unavoidable on every boot. Reading debugfs regs is the
	 * first touch, and it is a deliberate one.
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
