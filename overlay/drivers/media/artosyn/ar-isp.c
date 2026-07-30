// SPDX-License-Identifier: GPL-2.0
/*
 * ar-isp.c - Artosyn ISP capture path, bring-up driver.
 *
 * The VIF has two output routes. The bypass "view" path writes frames straight
 * to DDR and is what ar-vif.c arms; the ISP path hands frames to this block,
 * and it is the one the vendor uses. A write trace of the streaming vendor
 * shows it configures the views, sets the per-view reset, and then captures
 * every frame through the ISP.
 *
 * This driver applies the vendor's register configuration for the 2-lane
 * 1080p60 sensor mode and starts the block. The configuration carries the
 * vendor's own DDR addresses and the device tree reserves that range no-map, so
 * the capture buffers are used as they are. That is enough to answer whether the
 * block produces a frame at all, which is what this driver is for. It is not the
 * final shape.
 *
 * The coefficient tables are the exception: gamma and DRC are allocated here,
 * generated, and published at addresses this driver owns. Every byte the hardware
 * fetches from either is produced from the vendor tuning file, except gamma's
 * second page and the static half of the DRC page, which are not in that file in
 * any form and are carried as decoded curves. Everything else still runs on
 * replayed vendor addresses. That matters because slot B is RAM-booted from a
 * streaming slot A, so those pages hold the vendor's own tables and an unfilled
 * buffer is not obviously wrong; on a cold boot it would be. Formats are in
 * ar-isp-codec.h.
 *
 * Not implemented here:
 *
 *  - The per-frame loop. The vendor re-arms seven statistics buffer addresses
 *    and runs three indirect-port transactions on every frame, driven by the
 *    VIF frame-start interrupt that ar-vif.c already owns. The output planes
 *    are programmed once during setup and are not part of that loop, so the
 *    first frame should land without it. Later frames will overwrite the same
 *    buffer and the statistics will go stale.
 *  - Buffer allocation, geometry other than 1920x1080, and any V4L2 interface.
 *
 * Configuration provenance is in ar-isp-defaults.h. Two thirds of it comes
 * from static per-submodule default blocks in the vendor library and one third
 * from the write trace; applying both reproduces the vendor's final register
 * state exactly. See ../../../../docs/camera-stack.md.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include "ar-isp-defaults.h"
#include "ar-isp-codec.h"
#include "ar-isp-drc-tail.h"
#include "ar-isp-gamma-page1.h"
#include "ar-isp-compander.h"

/*
 * Master control. The block is brought up by a staged sequence of writes to
 * this register, which is embedded in the setup table in the right places
 * rather than driven separately.
 */
#define AR_ISP_CONTROL			0x0000

/* Input geometry, width in the low half. */
#define AR_ISP_INPUT_SIZE		0x0004

/*
 * Indirect access port. The vendor writes an address to 0x00cc and a value to
 * 0x00d4; three such transactions run in every frame of the per-frame loop.
 * The port is exercised during setup by the table.
 */
#define AR_ISP_INDIRECT_ADDR		0x00cc
#define AR_ISP_INDIRECT_DATA		0x00d4

/* Interrupt status and mask. Read-only use here. */
#define AR_ISP_INTR_STATUS		0x0090
#define AR_ISP_INTR_MASK		0x00b8

/*
 * Output plane addresses, identified by their placeholder values in the
 * vendor's static defaults: the block ships with implausible addresses which
 * the runtime replaces with real buffers. Listed for the register dump; the
 * setup table programs them.
 */
#define AR_ISP_OUT_PLANE0_SET0		0x2e3c
#define AR_ISP_OUT_PLANE1_SET0		0x2e44
#define AR_ISP_OUT_PLANE0_SET1		0x2e58
#define AR_ISP_OUT_PLANE1_SET1		0x2e60
#define AR_ISP_OUT_PLANE2_SET0		0x2e80
#define AR_ISP_OUT_PLANE2_SET1		0x2e88

/* Output stage geometry and commit. */
#define AR_ISP_OUT_SIZE			0x2e04
#define AR_ISP_OUT_COMMIT		0x2e90

/*
 * Read-only status reporting what the block sees on its input. Both are the
 * probe for whether the ISP is receiving video at all, and both are more useful
 * than looking at the output buffer, because they report on the input side.
 *
 * AR_ISP_IN_GEOMETRY reads back the measured frame as (height << 16) | width.
 * With the VIF streaming and the ISP otherwise untouched it reads 0x043c0784,
 * 1084 x 1924, matching the VIF's own measurement at VIF 0x1f0. A streaming
 * vendor reads 0x04380780, 1080 x 1920, the active area. Zero means the block
 * is seeing nothing.
 *
 * AR_ISP_IN_LINETIME latches the incoming line time and is static, not a
 * counter: a streaming vendor holds 0x134e across repeated samples while VIF
 * 0x1f8 measures about the same. Zero means no line timing is arriving.
 */
#define AR_ISP_IN_GEOMETRY		0x706c
#define AR_ISP_IN_LINETIME		0x2e98

/*
 * Coefficient table descriptors. Each holds the physical address of a DMA buffer
 * the block fetches when the matching bit is written to AR_ISP_TABLE_COMMIT.
 *
 * The commit is write-to-trigger, not a set-then-clear pulse: clearing the bit
 * afterwards cancels the fetch. Bit 16 is always set alongside. From the vendor
 * trace, which publishes an address and then commits, one table at a time:
 *
 *	0x0020 = 0x2b2e0c00   0x0014 = 0x00010001    compander
 *	0x0060 = 0x2b2e9200   0x0014 = 0x00010010    DRC
 *	0x0030/0x0040/0x0050 = 0x2b2ec600
 *	                      0x0014 = 0x0001000e    gamma, three descriptors
 */
#define AR_ISP_TABLE_COMMIT		0x0014
#define AR_ISP_TABLE_COMMIT_ENABLE	0x00010000
#define AR_ISP_TABLE_COMPANDER		0x0020
#define AR_ISP_TABLE_COMPANDER_BIT	0x00000001
#define AR_ISP_TABLE_GAMMA0		0x0030
#define AR_ISP_TABLE_GAMMA1		0x0040
#define AR_ISP_TABLE_GAMMA2		0x0050
#define AR_ISP_TABLE_GAMMA_BITS		0x0000000e
#define AR_ISP_TABLE_DRC		0x0060
#define AR_ISP_TABLE_DRC_BIT		0x00000010

/*
 * The compander allocation, which is deliberately larger than the table.
 *
 * Its length register at 0x0024 reads 0x780. In the units gamma proves, 32 bytes,
 * that is a 0xf000 fetch, twice the 0x7800 the table occupies. The vendor cannot
 * satisfy that either: its compander sits at 0x2b2e0c00 and 0xf000 later would run
 * past the gamma page at 0x2b2ec600, so whatever the block reads beyond the table
 * is its neighbours' memory and cannot be meaningful as compander data. The same
 * over-fetch is visible on GTM2, which holds 0xa00 of content and fetches 0x1000.
 *
 * So the excess is ignored by the block, and the only thing that matters here is
 * that the DMA stays inside memory we own. Allocating the full fetch length and
 * zeroing the tail costs 60 KiB of a 32 MiB pool and removes the question.
 */
#define AR_ISP_COMPANDER_ALLOC		0xf000

/*
 * The addresses the replayed configuration arms, inside the isp_cma reservation.
 * They are the vendor's own buffers, not ours. Reading them is how a table is
 * seeded from whatever the vendor left in DRAM across a RAM-boot.
 */
#define AR_ISP_VENDOR_GAMMA_PHYS	0x2b2ec600
#define AR_ISP_VENDOR_DRC_PHYS		0x2b2e9200

/*
 * The vendor tuning file, verbatim. Its length is checked because the vendor
 * loader checks it: every offset in ar-isp-codec.h is an absolute position in a
 * fixed-layout record, so a file of a different size is a different format and
 * not something to parse on a best-effort basis.
 */
#define AR_ISP_TUNING_FIRMWARE		"artosyn/nt99235-tuning-preview-fpv.bin"
#define AR_ISP_TUNING_SIZE		0xd6c58

/* The measured correction pass. Off disables it, for an A/B of its effect. */
static bool trim = true;
module_param(trim, bool, 0644);
MODULE_PARM_DESC(trim,
		 "apply the measured correction against the streaming vendor (default on)");

static bool tables = true;
module_param(tables, bool, 0644);
MODULE_PARM_DESC(tables,
		 "own the coefficient DMA buffers instead of arming the vendor's (default on)");

/*
 * Only reachable now when the tuning file is missing or a table is switched off:
 * both gamma and DRC generate every byte the hardware reads. Kept because that
 * fallback is the difference between a degraded camera and no camera, and
 * because turning it off is how a bring-up proves the generation really is
 * standalone rather than quietly leaning on what the vendor left behind.
 */
static bool seed = true;
module_param(seed, bool, 0644);
MODULE_PARM_DESC(seed,
		 "fall back to the vendor's inherited pages when a table cannot be generated (default on)");

/*
 * Which curve to select out of the tuning file. The vendor derives this from AE
 * and interpolates between adjacent curves; we have no AE, so it is pinned.
 *
 * Curve 3 reproduces one of our captures to within 5 counts of 4095 and curve 2
 * reproduces another, which is the AE selection visible directly in our own
 * data. Any pinned value is therefore an operating point, not a correct answer.
 */
static int gamma_curve = 3;
module_param(gamma_curve, int, 0644);
MODULE_PARM_DESC(gamma_curve, "tuning-file gamma curve, 0-4, or -1 to leave the page alone");

static int drc_profile = 3;
module_param(drc_profile, int, 0644);
MODULE_PARM_DESC(drc_profile, "tuning-file DRC profile, or -1 to leave the page alone");

static bool compander = true;
module_param(compander, bool, 0644);
MODULE_PARM_DESC(compander,
		 "own the compander page and fill it from the carried template (default on)");

struct ar_isp {
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data clks[2];
	struct dentry *debugfs;
	bool configured;

	const struct firmware *tuning;
	void *gamma;
	dma_addr_t gamma_dma;
	void *drc;
	dma_addr_t drc_dma;
	void *compander;
	dma_addr_t compander_dma;
};

static const struct {
	u16 off;
	const char *name;
} ar_isp_dump_regs[] = {
	{ AR_ISP_CONTROL,		"control" },
	{ AR_ISP_INPUT_SIZE,		"input_size" },
	{ AR_ISP_INTR_STATUS,		"intr_status" },
	{ AR_ISP_INTR_MASK,		"intr_mask" },
	{ AR_ISP_OUT_SIZE,		"out_size" },
	{ AR_ISP_OUT_PLANE0_SET0,	"out_plane0_set0" },
	{ AR_ISP_OUT_PLANE1_SET0,	"out_plane1_set0" },
	{ AR_ISP_OUT_PLANE2_SET0,	"out_plane2_set0" },
	{ AR_ISP_OUT_PLANE0_SET1,	"out_plane0_set1" },
	{ AR_ISP_OUT_PLANE1_SET1,	"out_plane1_set1" },
	{ AR_ISP_OUT_PLANE2_SET1,	"out_plane2_set1" },
	{ AR_ISP_OUT_COMMIT,		"out_commit" },
	{ AR_ISP_IN_GEOMETRY,		"in_geometry" },
	{ AR_ISP_IN_LINETIME,		"in_linetime" },
};

/* The vendor's final ISP run: the 69 writes it issues AFTER re-initialising the CSI-2
 * receiver and the VIF, and before the first per-frame acknowledge. This is where the output
 * stage is armed and the master enable reaches its final value.
 *
 * Carried verbatim from out/au-mmiotrace/mmio-combined.log rather than sliced out of
 * ar_isp_setup_1080p60: that table was generated from a different trace with consecutive
 * duplicates collapsed, so its ordering does not align here (18 of 69 match at the best
 * offset).
 *
 * The vendor's startup topology is ISP bulk -> receiver brought live -> output armed. Ours
 * configures the receiver first and replays the ISP afterwards, so that hand-off has never been
 * reproduced. Applying this table on its own, after the receiver is streaming, is what tests it.
 */
static const struct ar_isp_reg ar_isp_output_arm[] = {
	{ 0x00cc, 0x00000000 },
	{ 0x00d4, 0x10000600 },
	{ 0x00cc, 0x00000000 },
	{ 0x00d4, 0x00000100 },
	{ 0x2e74, 0x00000c00 },
	{ 0x2e00, 0x1f070002 },
	{ 0x2e90, 0x03000100 },
	{ 0x75a0, 0x2a660400 },
	{ 0x75bc, 0x2a660400 },
	{ 0x6440, 0x2a66a200 },
	{ 0x6474, 0x2a66a200 },
	{ 0x600c, 0x2a6a1200 },
	{ 0x280c, 0x2a723200 },
	{ 0x6508, 0x2a7ac200 },
	{ 0x1d1c, 0x00000000 },
	{ 0x1c08, 0x00000000 },
	{ 0x1df0, 0x00000000 },
	{ 0x0000, 0xb0280052 },
	{ 0x0000, 0xb0280052 },
	{ 0x0c10, 0x00000003 },
	{ 0x0c10, 0x00000002 },
	{ 0x1800, 0x000000e0 },
	{ 0x1804, 0x04380780 },
	{ 0x1808, 0x000f000a },
	{ 0x180c, 0x000f000a },
	{ 0x1810, 0x000f000a },
	{ 0x1814, 0x000f000a },
	{ 0x1818, 0x000f000a },
	{ 0x181c, 0x000f000a },
	{ 0x1820, 0x000f000a },
	{ 0x1824, 0x000f000a },
	{ 0x1828, 0x000f000a },
	{ 0x182c, 0x000f000a },
	{ 0x1830, 0x000f000a },
	{ 0x1834, 0x000f000a },
	{ 0x1838, 0x06400c80 },
	{ 0x183c, 0x00000258 },
	{ 0x1840, 0x060a0d10 },
	{ 0x1844, 0x06400c80 },
	{ 0x1848, 0x00000258 },
	{ 0x184c, 0x060a0d10 },
	{ 0x1850, 0x06400c80 },
	{ 0x1854, 0x00000258 },
	{ 0x1858, 0x060a0d10 },
	{ 0x185c, 0x06400c80 },
	{ 0x1860, 0x00000258 },
	{ 0x1864, 0x060a0d10 },
	{ 0x1868, 0x3c281a0d },
	{ 0x186c, 0xc8a0785a },
	{ 0x1870, 0x00080806 },
	{ 0x1874, 0x000c0a0a },
	{ 0x1878, 0x00100e0e },
	{ 0x187c, 0x3c281a0d },
	{ 0x1880, 0xc8a0785a },
	{ 0x1884, 0x00080806 },
	{ 0x1888, 0x000c0a0a },
	{ 0x188c, 0x00100e0e },
	{ 0x1890, 0x0001030c },
	{ 0x1800, 0x000000e0 },
	{ 0x1800, 0x000000e0 },
	{ 0x0000, 0xb0280052 },
	{ 0x4c40, 0x00010040 },
	{ 0x4c34, 0x2b2e8600 },
	{ 0x4c24, 0x00000034 },
	{ 0x4c30, 0x00000034 },
	{ 0x4c28, 0x00000034 },
	{ 0x4c3c, 0x00000001 },
	{ 0x4c00, 0x00000001 },
	{ 0x3000, 0x00000000 },
};

static void ar_isp_apply(struct ar_isp *isp, const struct ar_isp_reg *tbl,
			 size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		writel(tbl[i].val, isp->base + tbl[i].off);
}

/*
 * Copy the page the vendor left at a fixed physical address into one of our
 * buffers.
 *
 * This is a transitional crutch and is named as one. It only has anything to
 * copy because slot B is RAM-booted from a slot A whose camera was streaming, so
 * the vendor's tables are still resident in DRAM. On a cold boot these pages hold
 * nothing, which is exactly the dependency owning the buffers is meant to remove.
 * It exists so a first bring-up can change one thing at a time: with seeding on,
 * only the addresses move; with it off, the content is ours too.
 */
static bool ar_isp_seed_from_vendor(struct ar_isp *isp, void *dst,
				    phys_addr_t phys, size_t size)
{
	void *src;

	src = memremap(phys, size, MEMREMAP_WB);
	if (!src) {
		dev_warn(isp->dev, "cannot map vendor page at %pa to seed from\n",
			 &phys);
		return false;
	}

	memcpy(dst, src, size);
	memunmap(src);
	return true;
}

/*
 * Fill the owned buffers and hand them to the block.
 *
 * Runs after the register replay, which arms the vendor's addresses and commits
 * them. Republishing ours and committing again is the same sequence the vendor
 * itself issues on every AE update, so this is not a special case for the block.
 *
 * Compander has no generator to recover: the vendor installs its 0x7800 page
 * verbatim from a static template in the service library and never recomputes
 * it, so the page is carried and filled here like any other constant.
 */
static void ar_isp_tables_apply(struct ar_isp *isp)
{
	const u8 *blob = isp->tuning ? isp->tuning->data : NULL;
	bool gamma_seeded = false, drc_seeded = false;
	bool gamma_built = false, drc_built = false;
	bool compander_built = false;
	u8 *page;

	if (!tables)
		return;

	if (isp->gamma) {
		if (seed)
			gamma_seeded = ar_isp_seed_from_vendor(isp, isp->gamma,
							       AR_ISP_VENDOR_GAMMA_PHYS,
							       AR_ISP_GAMMA_SIZE);
		else
			memset(isp->gamma, 0, AR_ISP_GAMMA_SIZE);

		if (blob && gamma_curve >= 0 &&
		    gamma_curve < AR_ISP_GAMMA_BLOB_CURVES) {
			page = isp->gamma;
			ar_isp_gamma_from_blob(page, blob, gamma_curve);

			/*
			 * Page 1 is not an AE selection and is not in the tuning
			 * file: it is a carried constant. Both pages are written
			 * here, so gamma no longer depends on anything inherited.
			 *
			 * Nothing fills the rest of the allocation because nothing
			 * reads it. The descriptor length at 0x0034/0x0044/0x0054
			 * is in units of 32 bytes and the replay leaves it at 0x80,
			 * so the fetch is 0x1000. The allocation stays 0x4000 to
			 * match the vendor's, not because the block needs it.
			 */
			ar_isp_gamma_pack_page(page + AR_ISP_GAMMA_PAGE,
					       ar_isp_gamma_page1,
					       AR_ISP_GAMMA_PAGE1_TAIL);
			gamma_built = true;
		}
	}

	if (isp->drc) {
		if (seed)
			drc_seeded = ar_isp_seed_from_vendor(isp, isp->drc,
							     AR_ISP_VENDOR_DRC_PHYS,
							     AR_ISP_DRC_SIZE);
		else
			memset(isp->drc, 0, AR_ISP_DRC_SIZE);

		if (blob && drc_profile >= 0) {
			/*
			 * The whole page, both halves: the first two banks from
			 * the tuning file, the second two from the constant the
			 * vendor never recomputes. Nothing here is inherited,
			 * which is why the seed above is redundant when this
			 * runs and is left in place only so the two can be
			 * compared in one bring-up.
			 */
			page = isp->drc;
			ar_isp_drc_from_blob(page, blob, drc_profile);
			ar_isp_drc_pack_bank(page + 2 * AR_ISP_DRC_BANK,
					     ar_isp_drc_tail_bank0);
			ar_isp_drc_pack_bank(page + 3 * AR_ISP_DRC_BANK,
					     ar_isp_drc_tail_bank1);
			drc_built = true;
		}
	}

	if (isp->compander) {
		/*
		 * No seed path and no tuning file: the page is the same bytes on
		 * every unit and in every scene, so there is nothing to fall back
		 * to and nothing to select. The tail past the table is zeroed
		 * because the fetch length covers it; see AR_ISP_COMPANDER_ALLOC.
		 */
		memset(isp->compander, 0, AR_ISP_COMPANDER_ALLOC);
		ar_isp_compander_fill(isp->compander, ar_isp_compander_head,
				      ar_isp_compander_mid);
		compander_built = true;
	}

	/*
	 * The buffers are coherent, so there is no cache to flush, but the writes
	 * above must be visible before the address that makes the block fetch
	 * them.
	 */
	wmb();

	/*
	 * The descriptors are 32-bit. dma_alloc_coherent is bounded by the mask
	 * set in ar_isp_tables_prepare, so this cannot truncate, but the cast is
	 * written out rather than left implicit.
	 */
	if (isp->gamma) {
		u32 addr = lower_32_bits(isp->gamma_dma);

		writel(addr, isp->base + AR_ISP_TABLE_GAMMA0);
		writel(addr, isp->base + AR_ISP_TABLE_GAMMA1);
		writel(addr, isp->base + AR_ISP_TABLE_GAMMA2);
		writel(AR_ISP_TABLE_COMMIT_ENABLE | AR_ISP_TABLE_GAMMA_BITS,
		       isp->base + AR_ISP_TABLE_COMMIT);
	}

	if (isp->drc) {
		writel(lower_32_bits(isp->drc_dma), isp->base + AR_ISP_TABLE_DRC);
		writel(AR_ISP_TABLE_COMMIT_ENABLE | AR_ISP_TABLE_DRC_BIT,
		       isp->base + AR_ISP_TABLE_COMMIT);
	}

	if (isp->compander) {
		writel(lower_32_bits(isp->compander_dma),
		       isp->base + AR_ISP_TABLE_COMPANDER);
		writel(AR_ISP_TABLE_COMMIT_ENABLE | AR_ISP_TABLE_COMPANDER_BIT,
		       isp->base + AR_ISP_TABLE_COMMIT);
	}

	dev_info(isp->dev,
		 "tables: gamma %pad %s, drc %pad %s, compander %pad %s\n",
		 &isp->gamma_dma,
		 gamma_built ? "built" : (gamma_seeded ? "seeded" : "zeroed"),
		 &isp->drc_dma,
		 drc_built ? "built" : (drc_seeded ? "seeded" : "zeroed"),
		 &isp->compander_dma,
		 compander_built ? "built" : "on the vendor's page");
}

/*
 * Allocate the owned buffers and load the tuning file.
 *
 * Both are optional: a failure here leaves the replayed vendor addresses armed,
 * which is what the driver did before it owned anything, so the camera still
 * comes up. The tuning file is not in the repository and has to be installed on
 * the device; without it the buffers are still ours but carry only seeded data.
 */
static void ar_isp_tables_prepare(struct ar_isp *isp)
{
	struct device *dev = isp->dev;
	int ret;

	ret = of_reserved_mem_device_init(dev);
	if (ret) {
		dev_warn(dev, "no isp memory pool (%d), not owning tables\n", ret);
		return;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_warn(dev, "no 32-bit dma mask (%d), not owning tables\n", ret);
		return;
	}

	isp->gamma = dma_alloc_coherent(dev, AR_ISP_GAMMA_SIZE, &isp->gamma_dma,
					GFP_KERNEL);
	isp->drc = dma_alloc_coherent(dev, AR_ISP_DRC_SIZE, &isp->drc_dma,
				      GFP_KERNEL);
	if (compander)
		isp->compander = dma_alloc_coherent(dev, AR_ISP_COMPANDER_ALLOC,
						    &isp->compander_dma,
						    GFP_KERNEL);
	if (!isp->gamma || !isp->drc || (compander && !isp->compander))
		dev_warn(dev, "coefficient buffers unavailable, falling back to the vendor's\n");

	ret = request_firmware(&isp->tuning, AR_ISP_TUNING_FIRMWARE, dev);
	if (ret) {
		dev_warn(dev, "no %s (%d), tables cannot be generated\n",
			 AR_ISP_TUNING_FIRMWARE, ret);
		isp->tuning = NULL;
		return;
	}

	if (isp->tuning->size != AR_ISP_TUNING_SIZE) {
		dev_warn(dev, "%s is %zu bytes, expected %u; ignoring it\n",
			 AR_ISP_TUNING_FIRMWARE, isp->tuning->size,
			 AR_ISP_TUNING_SIZE);
		release_firmware(isp->tuning);
		isp->tuning = NULL;
	}
}

static void ar_isp_tables_release(struct ar_isp *isp)
{
	if (isp->gamma)
		dma_free_coherent(isp->dev, AR_ISP_GAMMA_SIZE, isp->gamma,
				  isp->gamma_dma);
	if (isp->drc)
		dma_free_coherent(isp->dev, AR_ISP_DRC_SIZE, isp->drc,
				  isp->drc_dma);
	if (isp->compander)
		dma_free_coherent(isp->dev, AR_ISP_COMPANDER_ALLOC,
				  isp->compander, isp->compander_dma);
	release_firmware(isp->tuning);
}

/*
 * Apply the whole configuration. The recovered table goes first: those are
 * registers that have a static default in the vendor library but which the
 * vendor never writes, because it pushes its shadow image with a
 * write-only-if-changed primitive and they already held the right value. They
 * are absent from any trace by construction, so nothing else would set them.
 *
 * The setup table then runs in write order. Order matters: it contains the
 * staged master enable and several arm-then-load registers whose result
 * depends on the sequence. It carries no timing, only order.
 *
 * The vendor interleaves CSI-2 and VIF writes into this sequence. Those blocks
 * are configured by their own drivers here, so only the ISP writes are
 * replayed and the interleaving is assumed not to matter. If the block turns
 * out to need a VIF or CSI-2 register at a particular point, this is where it
 * would show up.
 */
static void ar_isp_configure(struct ar_isp *isp)
{
	ar_isp_apply(isp, ar_isp_recovered, ARRAY_SIZE(ar_isp_recovered));
	ar_isp_apply(isp, ar_isp_setup_1080p60,
		     ARRAY_SIZE(ar_isp_setup_1080p60));

	/*
	 * Correct the result against the streaming vendor. The tables above are
	 * derived from a write trace, which fixes ordering but not the final
	 * value: the trace has to be cut somewhere, and the vendor keeps
	 * configuring past any cut. This pass is measured, not derived, and
	 * includes registers the trace never showed us writing at all.
	 */
	if (trim)
		ar_isp_apply(isp, ar_isp_vendor_trim,
			     ARRAY_SIZE(ar_isp_vendor_trim));

	ar_isp_apply(isp, ar_isp_output_fix, ARRAY_SIZE(ar_isp_output_fix));

	/*
	 * After the replay, which arms the vendor's buffer addresses. Ours
	 * replace them.
	 */
	ar_isp_tables_apply(isp);

	isp->configured = true;

	dev_info(isp->dev,
		 "configured: %zu recovered + %zu ordered + %zu trim + %zu output fix, control 0x%08x\n",
		 ARRAY_SIZE(ar_isp_recovered),
		 ARRAY_SIZE(ar_isp_setup_1080p60),
		 trim ? ARRAY_SIZE(ar_isp_vendor_trim) : 0,
		 ARRAY_SIZE(ar_isp_output_fix),
		 readl(isp->base + AR_ISP_CONTROL));
}

/*
 * Apply the vendor's output-arm run on its own.
 *
 * The vendor's startup order is ISP bulk, then a full re-initialisation of the
 * CSI-2 receiver and the VIF, then these 69 writes. Ours configures the receiver
 * first and replays the whole ISP table afterwards, so the hand-off the vendor
 * performs, arming the output only once the receiver is live, has never been
 * reproduced. Applying this after the stream is running is what tests that.
 *
 * Deliberately does not touch the recovered or setup tables, so it can follow a
 * prefix without undoing it.
 */
static void ar_isp_arm_output(struct ar_isp *isp)
{
	ar_isp_apply(isp, ar_isp_output_arm, ARRAY_SIZE(ar_isp_output_arm));

	dev_info(isp->dev,
		 "output arm: %zu writes, control 0x%08x, in_geometry 0x%08x\n",
		 ARRAY_SIZE(ar_isp_output_arm),
		 readl(isp->base + AR_ISP_CONTROL),
		 readl(isp->base + AR_ISP_IN_GEOMETRY));
}

/*
 * Apply only the first n writes of the setup table, for bisecting.
 *
 * With the VIF streaming and the ISP untouched, AR_ISP_IN_GEOMETRY reports the
 * measured input. After the full setup table it reads zero: one of the writes
 * stops the block seeing its input. Applying a prefix and reading that register
 * back finds which one, in about eleven steps over the 2082 entries.
 *
 * Each probe needs a clean block, so reload the capture modules and restart the
 * stream between probes rather than issuing several prefixes in a row. The trim
 * pass is deliberately not applied here: it would reintroduce the writes being
 * bisected.
 */
static void ar_isp_configure_prefix(struct ar_isp *isp, size_t n)
{
	if (n > ARRAY_SIZE(ar_isp_setup_1080p60))
		n = ARRAY_SIZE(ar_isp_setup_1080p60);

	ar_isp_apply(isp, ar_isp_recovered, ARRAY_SIZE(ar_isp_recovered));
	ar_isp_apply(isp, ar_isp_setup_1080p60, n);

	/*
	 * Applied here as well as in the full path. A prefix is not only a bisect
	 * tool: it is how the capture harness configures the block for every run,
	 * and the correction these two carry lives at entries 1773/1774, past every
	 * prefix in practical use. Without this a prefix bring-up is always crushed.
	 *
	 * Unlike the trim pass this does not interfere with the bisect it sits in,
	 * which chases the setup entry that kills the input geometry at 0x7070.
	 */
	ar_isp_apply(isp, ar_isp_output_fix, ARRAY_SIZE(ar_isp_output_fix));
	ar_isp_tables_apply(isp);

	isp->configured = true;

	dev_info(isp->dev,
		 "prefix %zu of %zu: in_geometry 0x%08x, in_linetime 0x%08x, control 0x%08x\n",
		 n, ARRAY_SIZE(ar_isp_setup_1080p60),
		 readl(isp->base + AR_ISP_IN_GEOMETRY),
		 readl(isp->base + AR_ISP_IN_LINETIME),
		 readl(isp->base + AR_ISP_CONTROL));
}

static int ar_isp_prefix_set(void *data, u64 val)
{
	ar_isp_configure_prefix(data, val);
	return 0;
}

static int ar_isp_prefix_get(void *data, u64 *val)
{
	*val = ARRAY_SIZE(ar_isp_setup_1080p60);
	return 0;
}

static int ar_isp_arm_set(void *data, u64 val)
{
	struct ar_isp *isp = data;

	if (val)
		ar_isp_arm_output(isp);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ar_isp_arm_fops, NULL, ar_isp_arm_set, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(ar_isp_prefix_fops, ar_isp_prefix_get,
			 ar_isp_prefix_set, "%llu\n");

static int ar_isp_configure_set(void *data, u64 val)
{
	struct ar_isp *isp = data;

	if (!val)
		return -EINVAL;

	ar_isp_configure(isp);
	return 0;
}

static int ar_isp_configure_get(void *data, u64 *val)
{
	struct ar_isp *isp = data;

	*val = isp->configured;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ar_isp_configure_fops, ar_isp_configure_get,
			 ar_isp_configure_set, "%llu\n");

static int ar_isp_regs_show(struct seq_file *s, void *unused)
{
	struct ar_isp *isp = s->private;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ar_isp_dump_regs); i++)
		seq_printf(s, "%-16s 0x%04x 0x%08x\n", ar_isp_dump_regs[i].name,
			   ar_isp_dump_regs[i].off,
			   readl(isp->base + ar_isp_dump_regs[i].off));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ar_isp_regs);

static int ar_isp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ar_isp *isp;
	int ret;

	isp = devm_kzalloc(dev, sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;

	isp->dev = dev;

	isp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(isp->base))
		return PTR_ERR(isp->base);

	isp->clks[0].id = "isp";
	isp->clks[1].id = "isp_hdr";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(isp->clks), isp->clks);
	if (ret)
		return dev_err_probe(dev, ret, "no isp clocks\n");

	/*
	 * The clocks stay on from probe. Register access with the block's
	 * clock gated hangs the SoC on this family, the same way VIF register
	 * reads do, so there is no safe point at which to gate them while the
	 * debugfs files exist.
	 */
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(isp->clks), isp->clks);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable isp clocks\n");

	platform_set_drvdata(pdev, isp);

	ar_isp_tables_prepare(isp);

	isp->debugfs = debugfs_create_dir("ar-isp", NULL);
	debugfs_create_file_unsafe("configure", 0600, isp->debugfs, isp,
				   &ar_isp_configure_fops);
	debugfs_create_file("regs", 0400, isp->debugfs, isp, &ar_isp_regs_fops);
	/*
	 * Reading this reports the table size, so a bisect script can discover
	 * the upper bound without hardcoding it.
	 */
	debugfs_create_file_unsafe("configure_upto", 0600, isp->debugfs, isp,
				   &ar_isp_prefix_fops);
	debugfs_create_file_unsafe("arm", 0600, isp->debugfs, isp,
				   &ar_isp_arm_fops);

	dev_info(dev, "probed, %zu registers available to apply\n",
		 ARRAY_SIZE(ar_isp_recovered) +
		 ARRAY_SIZE(ar_isp_setup_1080p60));

	return 0;
}

static void ar_isp_remove(struct platform_device *pdev)
{
	struct ar_isp *isp = platform_get_drvdata(pdev);

	debugfs_remove_recursive(isp->debugfs);
	ar_isp_tables_release(isp);
	clk_bulk_disable_unprepare(ARRAY_SIZE(isp->clks), isp->clks);
}

static const struct of_device_id ar_isp_of_match[] = {
	{ .compatible = "artosyn,isp" },
	{ }
};
MODULE_DEVICE_TABLE(of, ar_isp_of_match);

static struct platform_driver ar_isp_driver = {
	.probe = ar_isp_probe,
	.remove = ar_isp_remove,
	.driver = {
		.name = "ar-isp",
		.of_match_table = ar_isp_of_match,
	},
};
module_platform_driver(ar_isp_driver);

MODULE_DESCRIPTION("Artosyn ISP capture path");
MODULE_LICENSE("GPL");
