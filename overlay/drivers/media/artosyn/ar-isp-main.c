// SPDX-License-Identifier: GPL-2.0
/*
 * ar-isp-main.c - Artosyn ISP capture path, bring-up driver.
 *
 * The VIF has two output routes. The bypass "view" path writes frames straight
 * to DDR and is what ar-vif.c arms; the ISP path hands frames to this block,
 * and it is the one the vendor uses. A write trace of the streaming vendor
 * shows it configures the views, sets the per-view reset, and then captures
 * every frame through the ISP.
 *
 * This driver applies the recovered register configuration for the 2-lane
 * 1080p60 sensor mode and starts the block. The setup replay still carries
 * some vendor DDR addresses for quiescent stages, but the active coefficient,
 * temporal-filter and statistics buffers used by the open path are allocated
 * and published by ar-isp-tables.c.
 *
 * The buffers the block fetches from DRAM, and the register stages recomputed
 * from the tuning blob alongside them, are in ar-isp-tables.c. Both files share
 * struct ar_isp through ar-isp-priv.h.
 *
 * Current scope:
 *
 *  - The VIF frame-start interrupt republishes the driver-owned statistics
 *    buffers when pingpong is enabled.
 *  - ml-aed consumes the AE statistics and drives sensor exposure, gain and the
 *    ISP ladder abscissas from userspace.
 *  - Geometry is fixed at the vendor's 1920x1080 mode, and CVISP owns the V4L2
 *    capture interface.
 *
 * Configuration provenance is audited by scripts/isp/audit-provenance.py. The
 * values come from the tuning blob, static data in the vendor library, recovered
 * packers, geometry and the driver's own allocations; camera-isp-recovery.md
 * records the remaining device-sourced groups.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "vendor-tables/ar-isp-defaults.h"
#include "vendor-tables/ar-isp-gates.h"
#include "ar-isp-priv.h"
#include "ar-camera-hook.h"
#include "ar-isp-stats.h"
#include "ar-isp-regs.h"

/*
 * Registers the hardware writes, which the correction pass must not.
 *
 * That pass is a state diff against the streaming vendor, not a write diff, and
 * where the block owns a register its state is not configuration. All three
 * registers known to be hardware-written reached the table this way, and the
 * vendor is never seen writing any of them: 0x0834 and 0x08a8 read back values
 * other than the ones written to them on two independent boots, and 0x2e98 is
 * de3d's line-time latch, which the whole write trace never touches once.
 *
 * Writing a latch back is meaningless at best. The offsets stay in the
 * generated table, because that table records what was measured; this is where
 * the driver declines to replay it.
 *
 * Confirmed on hardware once the writes stopped: 0x0834 and 0x08a8 read back
 * live values that track the vendor's without matching them, which is what a
 * measurement does and what a configuration register cannot do. 0x0834 reads
 * 0x00fc against the vendor's 0x0215, 0x08a8 reads 0x0319 against 0x0164.
 *
 * The other six come from reading every entry twice with the configuration
 * held still: these change between two identical reads, so the hardware is
 * writing them. 0x2e98 was already known and the test found it unprompted,
 * which is the control on the method.
 *
 * That same A/B, toggling the trim parameter, measured what the pass is worth.
 * Of the 92 entries the capture covered, 75 are inert: the register reads the
 * same whether the pass runs or not, because a later applier writes it
 * afterwards. Six are these hardware-owned ones. Only eleven change anything,
 * and ten of those land exactly on the value written. 0x7054 lands in its high
 * half only, so it is left in place rather than skipped on half a result.
 *
 * Nine entries were on pages the capture did not dump and are unmeasured:
 * 0x1c1c, 0x1c38, 0x3060, 0x4c3c, 0x6514, 0x6db4, 0x757c and the two above.
 */
static const u16 ar_isp_hw_owned[] = {
	0x00cc, 0x00f0, 0x0834, 0x08a8, 0x2838, 0x2e98, 0x3ca8, 0x3cac,
};

static bool ar_isp_is_hw_owned(u16 off)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(ar_isp_hw_owned); i++)
		if (ar_isp_hw_owned[i] == off)
			return true;

	return false;
}

static void ar_isp_apply_trim(struct ar_isp *isp)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(ar_isp_vendor_trim); i++)
		if (!ar_isp_is_hw_owned(ar_isp_vendor_trim[i].off))
			writel(ar_isp_vendor_trim[i].val,
			       isp->base + ar_isp_vendor_trim[i].off);
}

/* The measured correction pass. Off disables it, for an A/B of its effect. */
static bool trim = true;
module_param(trim, bool, 0644);
MODULE_PARM_DESC(trim,
		 "apply the measured correction against the streaming vendor (default on)");

bool tables = true;
module_param(tables, bool, 0644);
MODULE_PARM_DESC(tables,
		 "own the coefficient DMA buffers instead of arming the vendor's (default on)");

bool hdr = true;
module_param(hdr, bool, 0644);
MODULE_PARM_DESC(hdr,
		 "own the HDR page, which shares an allocation with the compander (default on)");

bool lsc = true;
module_param(lsc, bool, 0644);
MODULE_PARM_DESC(lsc,
		 "own the LSC page and generate its lens-shading grid (default on)");

bool stats = true;
module_param(stats, bool, 0644);
MODULE_PARM_DESC(stats,
		 "own the AE statistics buffers instead of the vendor's (default on)");

bool pingpong = true;
module_param(pingpong, bool, 0644);
MODULE_PARM_DESC(pingpong,
		 "republish the statistics buffers per frame from the interrupt, alternating halves (default on)");

static bool gib = true;
module_param(gib, bool, 0644);
MODULE_PARM_DESC(gib,
		 "set gib's bypass bit as the vendor's tuning apply does (default on)");

/*
 * Setup-table entries applied when this driver brings the chain up itself.
 *
 * A prefix. The capture harness has configured every validated bring-up
 * through configure_upto with this length, so it is the measured operating
 * configuration and it belongs to the driver. The cut has consequences the
 * driver covers on its own: the replay's colour correction matrix sits at entry
 * 1718, past this, so ccm installs it directly (see the ccm bank comment), and
 * the lnr strength curves past 0x3d7c are left at zero.
 *
 * The remaining entries are suspect: after the full
 * table AR_ISP_IN_GEOMETRY reads zero, so one of them stops the block seeing
 * its input. Which one is not established.
 */
static unsigned int setup_entries = 1475;
module_param(setup_entries, uint, 0444);
MODULE_PARM_DESC(setup_entries,
		 "setup-table entries applied on self bring-up (default 1475)");

static bool use_irq = true;
module_param(use_irq, bool, 0444);
MODULE_PARM_DESC(use_irq,
		 "service the ISP interrupt: acknowledge the status words and count events (default on; the vendor has no poll mode here at all)");

static const struct {
	u16 off;
	const char *name;
} ar_isp_dump_regs[] = {
	{ AR_ISP_CONTROL,		"control" },
	{ AR_ISP_INPUT_SIZE,		"input_size" },
	{ AR_ISP_INTR_STATUS,		"intr_status" },
	{ AR_ISP_INTR_MASK,		"intr_mask" },
	{ AR_ISP_OUT_SIZE,		"out_size" },
	{ AR_ISP_DE3D_BUF0_A,		"de3d_buf0_a" },
	{ AR_ISP_DE3D_BUF1_A,		"de3d_buf1_a" },
	{ AR_ISP_DE3D_BUF2_A,		"de3d_buf2_a" },
	{ AR_ISP_DE3D_BUF0_B,		"de3d_buf0_b" },
	{ AR_ISP_DE3D_BUF1_B,		"de3d_buf1_b" },
	{ AR_ISP_DE3D_BUF2_B,		"de3d_buf2_b" },
	{ AR_ISP_OUT_COMMIT,		"out_commit" },
	{ AR_ISP_IN_GEOMETRY,		"in_geometry" },
	{ AR_ISP_IN_LINETIME,		"in_linetime" },
};

/* The vendor's final ISP run: the 69 writes it issues AFTER re-initialising the CSI-2
 * receiver and the VIF, and before the first per-frame acknowledge. This is where the output
 * stage is armed and the master enable reaches its final value.
 *
 * Carried verbatim from out/au-mmiotrace/mmio-combined.log. ar_isp_setup_1080p60 was
 * generated from a different trace with consecutive duplicates collapsed, so its ordering
 * does not align here (18 of 69 match at the best offset).
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

/*
 * The one lnr word the ladder does not fully own. The lnr ladder writes only
 * bits [19:0] of 0x3d44; the vendor streams 0xfff in [31:20], written by the
 * setup table past every prefix in practical use, so the prefix path would
 * otherwise leave those bits at the earlier entry's zero. Applied before the
 * ladder, which preserves them through its read-modify-write.
 */
static const struct ar_isp_reg ar_isp_lnr_fix[] = {
	{ 0x3d44, 0xfffdf800 },
};

/*
 * Two registers that no vendor submodule image covers and that the block
 * nonetheless keeps.
 *
 * A submodule default block is the length its descriptor in the ISP-init
 * template array gives, not the 64-register page an earlier reading assumed.
 * vendor-tables/ar-isp-library.h carries the images at their true length, and
 * scripts/isp/check-isp-library.py measures how far a page-sized reading
 * overruns each one.
 *
 * These two lie in that overrun, so no vendor image backs their values. They
 * are kept because they read back what was written, on the vendor device and
 * on this one; the rest of the overrun reads back zero, which is the block
 * discarding it. Their provenance is a recording, and audit-provenance.py
 * counts them as such.
 */
static const struct ar_isp_reg ar_isp_kept[] = {
	{ 0x6518, 0x00000001 },
	{ 0x651c, 0x00000001 },
};

/*
 * Apply the whole configuration.
 *
 * The setup table runs in write order. Order matters: it contains the
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
	ar_isp_apply(isp, ar_isp_kept, ARRAY_SIZE(ar_isp_kept));
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
		ar_isp_apply_trim(isp);

	ar_isp_apply(isp, ar_isp_output_fix, ARRAY_SIZE(ar_isp_output_fix));
	ar_isp_apply(isp, ar_isp_lnr_fix, ARRAY_SIZE(ar_isp_lnr_fix));

	/*
	 * After the replay, which arms the vendor's buffer addresses. Ours
	 * replace them.
	 */
	ar_isp_tables_apply(isp);

	/*
	 * Last, because the correction pass above replays 60 registers these
	 * stages derive. See ar_isp_ladders_apply.
	 */
	ar_isp_rgb2yuv_apply(isp);
	ar_isp_ladders_apply(isp, false);
	ar_isp_de3d_geom_apply(isp, false);

	isp->configured = true;

	dev_info(isp->dev,
		 "configured: %zu kept + %zu ordered + %zu trim + %zu output fix, control 0x%08x\n",
		 ARRAY_SIZE(ar_isp_kept),
		 ARRAY_SIZE(ar_isp_setup_1080p60),
		 trim ? ARRAY_SIZE(ar_isp_vendor_trim) : 0,
		 ARRAY_SIZE(ar_isp_output_fix),
		 readl(isp->base + AR_ISP_CONTROL));
}

/*
 * The per-frame tick handed to the output stage. See ar-camera-hook.h for why
 * the ISP owns it.
 *
 * The lock is held across the call, not just the pointer read, so a module that
 * unregisters is guaranteed no callback is still running when it returns. The
 * callback is short (a few register writes) so holding a spinlock across it in
 * hard interrupt context is acceptable.
 */
static DEFINE_SPINLOCK(ar_isp_hook_lock);
static void (*ar_isp_frame_hook)(void *ctx);
static void *ar_isp_frame_hook_ctx;

void ar_isp_set_frame_hook(void (*fn)(void *ctx), void *ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&ar_isp_hook_lock, flags);
	ar_isp_frame_hook = fn;
	ar_isp_frame_hook_ctx = ctx;
	spin_unlock_irqrestore(&ar_isp_hook_lock, flags);
}
EXPORT_SYMBOL_GPL(ar_isp_set_frame_hook);

/*
 * Service the ISP interrupt the way the vendor's handler does: acknowledge
 * every status word by unconditional write-back, then dispatch. The line is
 * level triggered, so an unacknowledged source storms it, which is what
 * killed the VIF line before its handler serviced the full set. Nothing is
 * re-armed here, matching the vendor; this version acknowledges and counts,
 * and the per-frame statistics service will hang off the bit-8 event once it
 * exists. The clocks are on from probe, so the registers are safe to touch
 * whenever the line fires.
 */
static irqreturn_t ar_isp_irq(int irq, void *data)
{
	struct ar_isp *isp = data;
	u32 s0, s1;

	s0 = readl(isp->base + AR_ISP_INTR_STATUS0);
	s1 = readl(isp->base + AR_ISP_INTR_STATUS1);

	if (!s0 && !s1)
		return IRQ_NONE;

	writel(s0, isp->base + AR_ISP_INTR_STATUS0);
	writel(s1, isp->base + AR_ISP_INTR_STATUS1);

	isp->irq_events++;
	isp->irq_seen0 |= s0;
	isp->irq_seen1 |= s1;

	if (s1 & AR_ISP_INTR_STATS_EVENT) {
		isp->irq_stats_events++;

		/*
		 * One statistics event per frame, so this is the frame tick the
		 * output stage rotates its ring on.
		 */
		spin_lock(&ar_isp_hook_lock);
		if (ar_isp_frame_hook)
			ar_isp_frame_hook(ar_isp_frame_hook_ctx);

		spin_unlock(&ar_isp_hook_lock);

		/*
		 * The frame that just ended wrote stats_cur, so that half is
		 * now the readable one and the engines are pointed at the
		 * other for the frame starting now. Arming before publishing
		 * the index is what keeps a reader off the live half: the
		 * moment stats_done names a half, the hardware has already
		 * been sent elsewhere.
		 */
		if (pingpong) {
			unsigned int next;

			/*
			 * Against a process-context ar_isp_stats_publish, which
			 * arms the same registers and resets these indices.
			 * Already in hard interrupt context, so the plain form.
			 */
			spin_lock(&isp->stats_lock);
			next = isp->stats_cur ^ 1;
			ar_isp_stats_arm(isp, next);
			WRITE_ONCE(isp->stats_done, isp->stats_cur);
			isp->stats_cur = next;
			WRITE_ONCE(isp->stats_valid, true);
			isp->stats_flips++;
			spin_unlock(&isp->stats_lock);
		}
	}

	return IRQ_HANDLED;
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

	/*
	 * Re-publish after the setup table, not only from ar_isp_tables_apply.
	 * The table carries the vendor's own statistics addresses and applying
	 * it, or any prefix of it past entry 1425, overwrites ours. Measured:
	 * publishing at table time and then running a 1475-entry prefix left
	 * 0x6440 reading the vendor's 0x2a662200, so the engines wrote the
	 * vendor's carveout and our buffers stayed empty.
	 */
	ar_isp_stats_publish(isp);
	ar_isp_de3d_publish(isp);
	ar_isp_ccm_apply(isp);
	ar_isp_ladders_apply(isp, true);
	ar_isp_dpc_apply(isp);

	if (gib)
		writel(readl(isp->base + AR_ISP_GIB_CTRL) | AR_ISP_GIB_BYPASS,
		       isp->base + AR_ISP_GIB_CTRL);

	/*
	 * The vendor's steady state holds both hdr-path module-local valid bits
	 * clear: arm, fetch, de-validate. Ours stayed set after publish, which
	 * the state diff flagged as the two hdr-path differences against the
	 * streaming vendor.
	 */
	writel(0, isp->base + AR_ISP_TABLE_HDR_VALID);
	writel(0, isp->base + AR_ISP_TABLE_HDR_LSC_VALID);

	/*
	 * The setup table carries the vendor's own LSC descriptor at entries
	 * around 317, so the table-time publish suffers the same overwrite as
	 * the statistics addresses: a mid-stream sweep read 0x4c34 holding the
	 * vendor's 0x2b2e8600 with our page never fetched, which on a cold boot
	 * is dead memory and shades the frame with zero gains. Republish after
	 * the replay, exactly like the statistics publish above.
	 */
	if (isp->lsc) {
		writel(lower_32_bits(isp->lsc_dma), isp->base + AR_ISP_TABLE_LSC);
		writel(1, isp->base + AR_ISP_TABLE_LSC_VALID);
	}

	/* Safe only now: the block is configured, so the status words assert
	 * at the vendor's event rate instead of continuously.
	 */
	if (use_irq && isp->irq > 0 && !isp->irq_requested) {
		if (request_irq(isp->irq, ar_isp_irq, 0, "ar-isp", isp))
			dev_warn(isp->dev, "isp interrupt request failed, counters stay zero\n");
		else
			isp->irq_requested = true;
	}

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
 * stream between probes, one prefix per boot. The trim
 * pass is deliberately not applied here: it would reintroduce the writes being
 * bisected.
 */
static void ar_isp_configure_prefix(struct ar_isp *isp, size_t n)
{
	if (n > ARRAY_SIZE(ar_isp_setup_1080p60))
		n = ARRAY_SIZE(ar_isp_setup_1080p60);

	ar_isp_apply(isp, ar_isp_kept, ARRAY_SIZE(ar_isp_kept));
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
	ar_isp_apply(isp, ar_isp_lnr_fix, ARRAY_SIZE(ar_isp_lnr_fix));
	ar_isp_tables_apply(isp);
	ar_isp_rgb2yuv_apply(isp);
	ar_isp_ladders_apply(isp, false);
	ar_isp_de3d_geom_apply(isp, false);

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

/*
 * Ladder-only re-latch for the userspace AE loop: recompute the
 * gain-keyed banks from the current Q8 parameters without the full output
 * re-arm, in the vendor's apply order. The vendor issues the equivalent
 * register traffic on every AE gain move at frame rate, so this is safe
 * mid-stream and cheap enough to call per decision.
 */
static int ar_isp_ladders_set(void *data, u64 val)
{
	struct ar_isp *isp = data;

	if (val) {
		ar_isp_ladders_apply(isp, false);
		ar_isp_de3d_geom_apply(isp, false);
	}

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ar_isp_ladders_fops, NULL, ar_isp_ladders_set, "%llu\n");

/*
 * Tone-table re-latch. This rebuilds the gamma and DRC pages from the tuning
 * blob and republishes the descriptors without replaying the whole ISP register
 * table.
 *
 * Write 1 for the loop's cadence: the pages are rebuilt only when the scalar
 * has moved the selection, which is the minority of frames and the reason the
 * comparison is made in the driver rather than in userspace. Write 2 to force a
 * rebuild, which is what a forced sweep over gamma_curve and drc_profile needs,
 * because those parameters change the selection without changing the scalar.
 */
static int ar_isp_tone_set(void *data, u64 val)
{
	struct ar_isp *isp = data;

	if (val)
		ar_isp_tone_apply(isp, val > 1, false);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ar_isp_tone_fops, NULL, ar_isp_tone_set, "%llu\n");

/*
 * The one ISP, for the exported bring-up call. Single block, single node.
 */
static struct ar_isp *ar_isp_instance;

/*
 * Configure the block and arm its output, as one call for a driver that is
 * bringing the whole chain up.
 *
 * Uses the prefix, the configuration every validated bring-up has run: the
 * capture harness has driven configure_upto with this length throughout. The
 * full table is a different configuration and is still unvalidated, so choosing
 * it here would silently change what the camera runs.
 *
 * The caller must have the pixel domain live already: this reads registers back.
 */
int ar_isp_pipeline_start(void)
{
	struct ar_isp *isp = READ_ONCE(ar_isp_instance);

	if (!isp)
		return -ENODEV;

	ar_isp_configure_prefix(isp, setup_entries);
	ar_isp_arm_output(isp);

	return 0;
}
EXPORT_SYMBOL_GPL(ar_isp_pipeline_start);

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

	for (unsigned int i = 0; i < ARRAY_SIZE(ar_isp_dump_regs); i++)
		seq_printf(s, "%-16s 0x%04x 0x%08x\n", ar_isp_dump_regs[i].name,
			   ar_isp_dump_regs[i].off,
			   readl(isp->base + ar_isp_dump_regs[i].off));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ar_isp_regs);

/*
 * The gain-keyed banks, live against a fresh recomputation. ar-isp-tables.c owns
 * the comparison because the gain parameters and the stage builders live there.
 */
static int ar_isp_ladder_banks_show(struct seq_file *s, void *unused)
{
	return ar_isp_ladders_show(s, s->private);
}
DEFINE_SHOW_ATTRIBUTE(ar_isp_ladder_banks);

/*
 * Every stage's gate, read back and compared against the tuning file.
 *
 * The recovery in vendor-tables/ar-isp-gates.h says which register and bit
 * gates each submodule and which way round it reads; the tuning file says
 * what the vendor asks for. This reads the register and reports the two side
 * by side, and nothing else: no gate is written here, so the node is a parity
 * oracle rather than a control, and it is what confirms a polarity on hardware
 * instead of encoding an unverified one.
 *
 * A stage with no tuning flag, an unresolved polarity or a whole-word gate
 * reports its state with no expectation. Word gates carry no expectation
 * because the vendor's "on" value is computed rather than constant, so a
 * non-zero word means running and nothing further can be asserted from a table.
 * Nor does a stage flagged AR_ISP_STAGE_HW_RB, whose gate register the hardware
 * drives: comparing a readback against a written value there measures the
 * block, not the configuration.
 *
 * A mismatch is not automatically a defect. A stage the tuning file wants on
 * and this driver never configures reads its bank as zero and reports one
 * correctly, and two stages sharing a bit under opposite polarities make one of
 * them mismatch whichever way the bit goes.
 */
static int ar_isp_gates_show(struct seq_file *s, void *unused)
{
	struct ar_isp *isp = s->private;
	const u8 *blob = isp->tuning ? isp->tuning->data : NULL;
	unsigned int mismatches = 0;

	seq_printf(s, "%-16s %-8s %-6s %-11s %-5s %-5s %s\n",
		   "stage", "gate", "bit", "live", "state", "blob", "verdict");

	for (unsigned int i = 0; i < ARRAY_SIZE(ar_isp_stages); i++) {
		const struct ar_isp_stage *st = &ar_isp_stages[i];
		int want = -1;

		if (blob && st->blob_gate &&
		    st->blob_gate + 4 <= isp->tuning->size)
			want = ar_isp_get_le32(blob + st->blob_gate) ? 1 : 0;

		for (unsigned int g = 0; g < st->n_gates; g++) {
			const struct ar_isp_gate *gate = &st->gates[g];
			u32 word = readl(isp->base + gate->reg);
			const char *verdict = "";
			int on = -1;
			char bit[8];

			switch (gate->kind) {
			case AR_ISP_GATE_BIT_ENABLE:
				on = word & gate->set_mask ? 1 : 0;
				break;

			case AR_ISP_GATE_BIT_BYPASS:
				on = word & gate->set_mask ? 0 : 1;
				break;

			case AR_ISP_GATE_WORD:
				on = word ? 1 : 0;
				break;
			}

			if (gate->set_mask)
				scnprintf(bit, sizeof(bit), "%u",
					  __ffs(gate->set_mask));
			else
				strscpy(bit, "word", sizeof(bit));

			/*
			 * Only a bit gate with a recovered polarity and a
			 * tuning flag can disagree. Everything else is
			 * reported without a verdict rather than judged
			 * against a state that was never recovered.
			 */
			if (want >= 0 && on >= 0 &&
			    !(st->flags & AR_ISP_STAGE_HW_RB) &&
			    gate->kind != AR_ISP_GATE_WORD &&
			    gate->kind != AR_ISP_GATE_UNKNOWN) {
				verdict = on == want ? "ok" : "MISMATCH";
				if (on != want)
					mismatches++;
			}

			seq_printf(s, "%-16s 0x%04x   %-6s 0x%08x  %-5d %-5d %s\n",
				   g ? "" : st->name, gate->reg, bit, word, on,
				   want, verdict);
		}
	}

	if (!blob)
		seq_puts(s, "\nno tuning file: blob column is -1 throughout\n");

	seq_printf(s, "\n%u gate%s disagree with the tuning file\n",
		   mismatches, mismatches == 1 ? "" : "s");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ar_isp_gates);

/*
 * The AE zone grid, decoded.
 *
 * Prints the per-zone luma mean as a 36-column by 16-row map, plus the frame
 * mean and the count the sums were accumulated over. The count is read from the
 * buffer, because the count is a function of the programmed
 * geometry and a stale divisor would silently scale every mean.
 *
 * An all-zero map with a zero count means the hardware is not writing here,
 * which is the check that matters after publishing our own address: the engines
 * keep whatever address was last written to them, so a buffer that stays zero
 * says the vendor's per-frame cycle overwrote the pointer.
 *
 * Luma is the mean of the two greens. Channels 1 and 2 are the greens, measured
 * by their tracking across the grid; which of 0 and 3 is red is not established,
 * so neither is used here.
 */
static int ar_isp_stats_show(struct seq_file *s, void *unused)
{
	struct ar_isp *isp = s->private;
	const void *rro;
	u64 total = 0;
	u32 count;

	if (!isp->rro) {
		seq_puts(s, "not owned\n");
		return 0;
	}

	rro = ar_isp_stats_ready(isp, isp->rro, AR_ISP_RRO_SIZE);
	if (!rro) {
		seq_puts(s, "no completed frame\n");
		return 0;
	}

	count = ar_isp_rro_count(rro, 0);
	seq_printf(s, "count %u per zone per channel\n", count);

	for (unsigned int row = 0; row < AR_ISP_RRO_ROWS; row++) {
		for (unsigned int col = 0; col < AR_ISP_RRO_COLS; col++) {
			u32 g0 = ar_isp_rro_mean(rro, col, row, 1);
			u32 g1 = ar_isp_rro_mean(rro, col, row, 2);
			u32 luma = (g0 + g1) / 2;

			total += luma;
			seq_printf(s, "%4u", luma);
		}

		seq_putc(s, '\n');
	}

	seq_printf(s, "frame mean %llu\n", total / AR_ISP_RRO_ZONES);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ar_isp_stats);

/*
 * Raw statistics export for the userspace 3A loop: one read returns the flip
 * sequence, the AE zone-grid half and the histogram half the hardware is not
 * writing, then the sequence again. The halves flip at most once per frame,
 * so a reader that sees the two sequence words differ retries the read and
 * the second attempt is coherent. A poll of the stats_flips file tells the
 * reader when a new frame's statistics exist without copying them.
 */
static int ar_isp_stats_raw_show(struct seq_file *s, void *unused)
{
	struct ar_isp *isp = s->private;
	const void *rro, *hist;
	u32 seq = READ_ONCE(isp->stats_flips);

	rro = ar_isp_stats_ready(isp, isp->rro, AR_ISP_RRO_SIZE);
	hist = ar_isp_stats_ready(isp, isp->hist, AR_ISP_HIST_SIZE);
	if (!rro || !hist)
		return -EAGAIN;

	seq_write(s, &seq, sizeof(seq));
	seq_write(s, rro, AR_ISP_RRO_SIZE);
	seq_write(s, hist, AR_ISP_HIST_SIZE);
	seq = READ_ONCE(isp->stats_flips);
	seq_write(s, &seq, sizeof(seq));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ar_isp_stats_raw);

static int ar_isp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ar_isp *isp;
	int ret;

	isp = devm_kzalloc(dev, sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;

	isp->dev = dev;
	spin_lock_init(&isp->stats_lock);

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
	WRITE_ONCE(ar_isp_instance, isp);

	ar_isp_tables_prepare(isp);

	isp->debugfs = debugfs_create_dir("ar-isp", NULL);
	debugfs_create_file_unsafe("configure", 0600, isp->debugfs, isp,
				   &ar_isp_configure_fops);
	debugfs_create_file("regs", 0400, isp->debugfs, isp, &ar_isp_regs_fops);
	debugfs_create_file("gates", 0400, isp->debugfs, isp,
			    &ar_isp_gates_fops);
	debugfs_create_file("stats", 0400, isp->debugfs, isp, &ar_isp_stats_fops);
	debugfs_create_file("stats_raw", 0400, isp->debugfs, isp,
			    &ar_isp_stats_raw_fops);
	/*
	 * Reading this reports the table size, so a bisect script can discover
	 * the upper bound without hardcoding it.
	 */
	debugfs_create_file_unsafe("configure_upto", 0600, isp->debugfs, isp,
				   &ar_isp_prefix_fops);
	debugfs_create_file_unsafe("arm", 0600, isp->debugfs, isp,
				   &ar_isp_arm_fops);
	debugfs_create_file_unsafe("ladders", 0600, isp->debugfs, isp,
				   &ar_isp_ladders_fops);
	debugfs_create_file("ladder_banks", 0400, isp->debugfs, isp,
			    &ar_isp_ladder_banks_fops);
	debugfs_create_file_unsafe("tone", 0600, isp->debugfs, isp,
				   &ar_isp_tone_fops);
	debugfs_create_u32("irq_events", 0400, isp->debugfs, &isp->irq_events);
	debugfs_create_u32("irq_stats_events", 0400, isp->debugfs,
			   &isp->irq_stats_events);
	debugfs_create_x32("irq_seen0", 0400, isp->debugfs, &isp->irq_seen0);
	debugfs_create_x32("irq_seen1", 0400, isp->debugfs, &isp->irq_seen1);
	debugfs_create_u32("stats_flips", 0400, isp->debugfs, &isp->stats_flips);

	/* Only the number here. The line is requested at output arm, never at
	 * probe: the bring-up order is receiver first and ISP after, so between
	 * stream start and the arm the block is unconfigured with input
	 * arriving, its sources reassert as fast as they are acknowledged, and
	 * a requested line livelocks the CPU in the handler. Measured as a
	 * hard hang the moment streaming started. The vendor never has that
	 * window because it configures the ISP before the receiver.
	 */
	if (use_irq)
		isp->irq = platform_get_irq(pdev, 0);

	dev_info(dev, "probed, %zu registers available to apply\n",
		 ARRAY_SIZE(ar_isp_kept) +
		 ARRAY_SIZE(ar_isp_setup_1080p60));

	return 0;
}

static void ar_isp_remove(struct platform_device *pdev)
{
	struct ar_isp *isp = platform_get_drvdata(pdev);

	/* Before anything else: once the clocks drop, a late interrupt's
	 * register access hangs the SoC, so the line must be gone first. The
	 * exported bring-up call reaches this instance from another module and
	 * is cleared for the same reason.
	 */
	WRITE_ONCE(ar_isp_instance, NULL);

	if (isp->irq_requested)
		free_irq(isp->irq, isp);

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
