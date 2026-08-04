// SPDX-License-Identifier: GPL-2.0
/*
 * ar-vif.c - Artosyn VIF capture front end.
 *
 * The VIF receives frames from the MIPI CSI-2 receiver's IPI output and writes
 * them to DDR. It exposes eight independent capture "views"; this driver uses
 * view 0, which is the one the vendor stack uses.
 *
 * The register values below reproduce a capture of the vendor stack streaming
 * 1920x1080 RAW12 over two lanes, taken by diffing its live register window
 * against the same window with nothing running. Several front-end registers
 * are opaque constants for a given sensor mode and are written verbatim; the
 * geometry, stride and buffer-size registers are computed.
 *
 * The per-view buffer address registers do not self-clear: an address is
 * written and latched by pulsing bit12 of the view control on the first arm
 * only; each further frame is re-armed by rewriting the address registers
 * alone from the frame-done handler. Completion is signalled by the W1C
 * status at 0x17c (bit v buffer done, bit 8+v frame done), not by the
 * address readback.
 *
 * The view path this driver arms is NOT how the vendor captures. A write trace
 * of the streaming vendor shows it configures the views, then sets the per-view
 * reset at 0x2bc and captures every frame through the ISP instead, re-arming an
 * ISP buffer pair from its own frame handler. The view DMA has never written a
 * byte to DDR. See ../../../../docs/camera-stack.md for the vendor sequence.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/string.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>

#include "ar-camera-hook.h"

/* The view this driver uses. Path view numbers 2..9 map to views 0..7. */
#define AR_VIF_VIEW			0

/* Per-view registers, indexed by view. */
#define AR_VIF_VIEW_CONTROL(v)		(0x000 + (v) * 4)
#define AR_VIF_VIEW_ADDR_Y(v)		(0x020 + (v) * 4)
#define AR_VIF_VIEW_ADDR_U(v)		(0x040 + (v) * 4)
#define AR_VIF_VIEW_ADDR_V(v)		(0x060 + (v) * 4)
#define AR_VIF_VIEW_STRIDE0(v)		(0x200 + (v) * 8)
#define AR_VIF_VIEW_STRIDE1(v)		(0x204 + (v) * 8)
#define AR_VIF_VIEW_DDR_SIZE_Y(v)	(0x340 + (v) * 4)
#define AR_VIF_VIEW_DDR_SIZE_U(v)	(0x360 + (v) * 4)
#define AR_VIF_VIEW_DDR_SIZE_V(v)	(0x380 + (v) * 4)

/* Shared registers. */
#define AR_VIF_VIEW_MUX			0x080
/* 0x084 is the per-view frame-stable control, NOT a view enable. bit(24+view)
 * SET = stable-check DISABLED (the streaming state; DDR writes run autonomously);
 * writing 0 here CLEARS bit24 and re-ENABLES the check, which re-gates DDR. Do
 * not "disable" this register by writing 0.
 */
#define AR_VIF_VIEW_STABLE_CHECK	0x084
#define AR_VIF_VIEW_THRESHOLD		0x08c
#define AR_VIF_VIEW_FIFO_BURST(v)	(0x25c + (v) * 4)
#define AR_VIF_INTR_STATUS		0x1b0

/* Bypass-path interrupt status words, write-1-to-clear: the vendor acknowledges
 * by writing the read value back. Each pairs with a mask, 0x17c with 0x178 and
 * 0x184 with 0x180. Reading them requires the AXI clock: a read with the clock
 * gated hangs the SoC.
 */
#define AR_VIF_BP_INTR_STATUS		0x17c
#define AR_VIF_BP_INTR_STATUS_B		0x184
#define AR_VIF_INTR_MASK_A		0x178
#define AR_VIF_INTR_MASK_B		0x180
#define AR_VIF_BP_VIEW_FULL(v)		BIT(2 + (v))

/* Input-path format select in the ISP-input stage; the streaming vendor holds
 * 5 here while the init table's value is 0x27.
 */
#define AR_VIF_INPUT_FORMAT_SELECT	0x1d0
#define AR_VIF_INPUT_FORMAT_SELECT_VALUE 0x00000005

/* Interrupt status bank, five words the vendor's ISR pre-reads. In word 0:
 * bit view is buffer-done (this is what drives the vendor's re-arm), bit
 * (8 + view) is frame-done, bit (16 + view) is the error bit.
 */
#define AR_VIF_INTR_BANK(n)		(0x100 + (n) * 4)
#define AR_VIF_INTR_BUFFER_DONE(v)	BIT(v)
#define AR_VIF_INTR_FRAME_DONE(v)	(0x100 << (v))

/* Bit30 of the stride register commits the view's static configuration; the
 * vendor strobes it after writing the stride.
 */
#define AR_VIF_STRIDE_COMMIT		BIT(30)

/* Front-end input configuration. The vendor library calls this group the ISP
 * path, but it is the shared VIF input stage: it is programmed whether or not
 * the ISP is used.
 */
#define AR_VIF_FE_CONTROL		0x0c0
#define AR_VIF_FE_TIMING		0x0c4
#define AR_VIF_FE_FORMAT		0x0c8
#define AR_VIF_FE_DATATYPE0		0x0cc
#define AR_VIF_FE_DATATYPE1		0x0d0
#define AR_VIF_FE_GEOMETRY_MARGIN	0x0d4
#define AR_VIF_FE_GEOMETRY		0x0d8
#define AR_VIF_FE_HBLANK		0x0dc
#define AR_VIF_FE_VBLANK		0x0e0
#define AR_VIF_FE_LIMIT			0x0e8
#define AR_VIF_FE_EXTRA			0x0ec
#define AR_VIF_FE_INTR_MASK		0x0f0

/* Line-buffer HDR mode. The vendor writes 0 here on every non-HDR capture
 * start; the register does not reset to 0 on its own.
 */
#define AR_VIF_LINEBUFFER_HDR_MODE	0x320

/* Line-buffer input config, in the same cluster as the HDR mode and the ispif
 * enable at 0x328. The running vendor streams with 0xfff here; the driver left
 * it at its reset default, the one value that differs from a live vendor
 * capture across the whole 0x000-0x37f window. Function is not decoded.
 */
#define AR_VIF_LINEBUFFER_INPUT		0x32c
#define AR_VIF_LINEBUFFER_INPUT_VALUE	0x00000fff

/* Per-view frame-end configuration, one register per view. Bit31 enables the
 * frame-end check and bit30 selects its mode. The low half carries the frame
 * width and the upper half the frame height.
 */
#define AR_VIF_VIEW_FRAME_END(v)	(0x3a0 + (v) * 4)
#define AR_VIF_FRAME_END_ENABLE		BIT(31)

/* Per-view crop window, two registers per view. The vertical register holds the
 * first and last line, the horizontal register the first and last column.
 * Bit31 enables cropping; the window is still programmed when it is disabled.
 */
#define AR_VIF_VIEW_CROP_V(v)		(0x3c0 + (v) * 8)
#define AR_VIF_VIEW_CROP_H(v)		(0x3c4 + (v) * 8)

/* Raw Bayer moves through the front end three pixels at a time, so the width
 * fields of the crop and frame-end registers count groups of three pixels.
 */
#define AR_VIF_PIXELS_PER_GROUP		3

/* Per-view FIFO partition: three 10-bit allocations in bits 29 through 0, with
 * bits 31 and 30 encoding the AXI burst length as 0 for 4, 1 for 8 and 2 for
 * 16. A raw capture gets an equal allocation across the three fields.
 */
#define AR_VIF_VIEW_FIFO(v)		(0x140 + (v) * 4)
#define AR_VIF_FIFO_ALLOCATION		480
#define AR_VIF_FIFO_BURST_16		BIT(31)

/* Buffer-address latch pulse in the view control register. */
#define AR_VIF_VIEW_LATCH		BIT(12)

/* Per-view frame backpressure, value in bits 29:15, committed by a bit30
 * set-then-clear strobe during arming.
 */
#define AR_VIF_VIEW_FRAME_BP(v)		(0x280 + (v) * 4)
#define AR_VIF_FRAME_BP_COMMIT		BIT(30)

/* Registers touched by the post-init and AXI-enable steps. */
#define AR_VIF_SOFT_RESET		0x2bc
#define AR_VIF_FRAME_CTRL		0x2b0
#define AR_VIF_BLOCK_ENABLE		0x194
#define AR_VIF_AXI_CONFIG		0x190
#define AR_VIF_LINE_INTR_CLEAR		0x420
#define AR_VIF_LINE_INTR_MASK		0x424
#define AR_VIF_EXTRA_INTR_MASK		0x5c0
#define AR_VIF_EXTRA_INTR_CLEAR		0x5c4

/* AXI master enable (bit0) and its run bit (bit30), set by vif_axi_config. */
#define AR_VIF_AXI_ENABLE		BIT(0)
#define AR_VIF_AXI_RUN			BIT(30)

#define AR_VIF_FRAME_BOUNDARY		0x2a8
#define AR_VIF_FRAME_BOUNDARY_VALUE	0x0000b0ff
#define AR_VIF_DDR_CLIP			0x2ac
#define AR_VIF_DDR_CLIP_VALUE		0x000001ff
/* 0x2bc: bit0 = global block soft reset, bit(view + 1) = per-view soft reset.
 * A write trace of the running vendor settles the polarity: it clears this
 * register to 0 while arming the capture and only writes 0x2 when tearing the
 * capture down. A snapshot of a streaming unit reads 0x2 because the vendor had
 * already torn its capture down again, so the value a register holds at rest is
 * the reset state, not the running one. Fully released = 0.
 */
#define AR_VIF_SOFT_RESET_RELEASED	0x00000000

/* The register at 0x194 holds views off while set: the vendor releases it
 * progressively (0xffff, then 0xff, then 0) and the streaming vendor reads 0.
 * Leaving any bit set keeps the engine inert with every register writable.
 */
#define AR_VIF_BLOCK_ENABLE_VALUE	0x00000000

/*
 * Constants captured from the vendor's running 1920x1080 RAW12 two-lane
 * configuration. Their internal fields are not decoded; they are properties of
 * the sensor mode and are reproduced verbatim. A different mode would need its
 * own set, which is why the driver accepts only the geometry it has values for.
 */
#define AR_VIF_VIEW_CONTROL_VALUE	0x0000c068
#define AR_VIF_VIEW_MUX_VALUE		0xffffffff

/* Per-view FIFO partition the capture is configured with: the enable bit plus
 * three 480-entry fields. The init-table value is what the register reads back
 * once the vendor has torn the capture down again.
 */
#define AR_VIF_VIEW_FIFO_SPLIT		0x1e0781e0
#define AR_VIF_VIEW_FIFO_ENABLE		BIT(31)

/* Per-view threshold nibbles, as the capture configures them. */
#define AR_VIF_VIEW_THRESHOLD_VALUE	0x88888880
#define AR_VIF_VIEW_STABLE_CHECK_DISABLE	0x01000000
#define AR_VIF_FE_CONTROL_VALUE		0x00061588
#define AR_VIF_FE_TIMING_VALUE		0x00640064
#define AR_VIF_FE_FORMAT_VALUE		0x00d6002c
#define AR_VIF_FE_HBLANK_VALUE		0x00000282
#define AR_VIF_FE_VBLANK_VALUE		0x000000d6
#define AR_VIF_FE_LIMIT_VALUE		0x000005ff
#define AR_VIF_FE_EXTRA_VALUE		0x00008000

/* The stride and length fields count 64-byte units. A 1920 pixel line at two
 * bytes per pixel is 3840 bytes, which the vendor programs as 60.
 */
#define AR_VIF_STRIDE_UNIT		64
#define AR_VIF_STRIDE_FIELD_MASK	0x3ff
#define AR_VIF_STRIDE_ENABLE		BIT(31)

/* The only geometry this driver has vendor-captured constants for. */
#define AR_VIF_WIDTH			1920
#define AR_VIF_HEIGHT			1080

/* 12-bit Bayer arrives unpacked into 16-bit containers. */
#define AR_VIF_BYTES_PER_PIXEL		2

/* Input-pipe registers, written around the FIFO reset in the post-init step. */
#define AR_VIF_INPUT_PIPE_PRESET	0x13c
#define AR_VIF_INPUT_PIPE_CTRL		0x1c0
#define AR_VIF_INPUT_PIPE_CONFIG	0x1c4
#define AR_VIF_INPUT_PIPE_MODE		0x1c8

/* RGB2YUV coefficient and clip bank for the bypass views, seven words. */
#define AR_VIF_RGB2YUV(n)		(0x090 + (n) * 4)

/* ISP-interface enable, in the line-buffer cluster. Bit1 gates the front end
 * into the ISP path and is released after the view is armed.
 */
#define AR_VIF_ISPIF_ENABLE		0x328
#define AR_VIF_ISPIF_RUN		BIT(1)

/* Path 0 test pattern generator. */
#define AR_VIF_TEST_PATTERN		0x0f4
#define AR_VIF_TEST_PATTERN_ENABLE	BIT(0)

/* ISP-path status registers sampled by the event census. */
#define AR_VIF_ISP_PROBES		4

/* Status polling interval. Frames arrive at 60 Hz, so this samples every
 * frame several times over without meaningful cost.
 */
#define AR_VIF_POLL_INTERVAL_MS		4

/*
 * One-time block initialisation, transcribed from the vendor's own table (a
 * list of offset/value pairs walked by its vif_reg_init). This is separate
 * from the per-capture configuration below: the vendor applies it when its
 * driver loads, not when a capture starts.
 *
 * It was missed by the register diff that produced the per-capture constants,
 * because that diff compared a streaming vendor stack against an idle one on
 * the same kernel, where this had already been applied in both. Without it the
 * front end accepts no data: the line counters never advance and the CSI-2
 * receiver's IPI output backs up and reports an error.
 */
struct ar_vif_init_reg {
	u16 offset;
	u32 value;
};

static const struct ar_vif_init_reg ar_vif_init_table[] = {
	{ 0x0c0, 0x00000188 }, { 0x0c4, 0x00640064 }, { 0x0c8, 0x00002c2c },
	{ 0x0cc, 0xaaaaaaac }, { 0x0d0, 0xaaaaaaaa }, { 0x0d4, 0x07800438 },
	{ 0x0d8, 0x07800438 }, { 0x0dc, 0x02800280 }, { 0x0e0, 0x000000d6 },
	{ 0x0e4, 0x00320032 }, { 0x0e8, 0x00000200 }, { 0x0ec, 0x00008000 },
	{ 0x0f0, 0xffffffff }, { 0x13c, 0x00020002 }, { 0x190, 0x00000101 },
	{ 0x1c0, 0x00000822 }, { 0x1c4, 0x08020010 }, { 0x1c8, 0x00000001 },
	{ 0x1d0, 0x00000027 }, { 0x328, 0x00000002 },
	{ 0x140, 0x86018060 }, { 0x144, 0x0c0300c0 },
	{ 0x148, 0x86018060 }, { 0x14c, 0x0c0300c0 },
	{ 0x150, 0x86018060 }, { 0x154, 0x0c0300c0 },
	{ 0x158, 0x86018060 }, { 0x15c, 0x0c0300c0 },
};

/* RGB2YUV coefficients and clip values the running vendor block holds. The
 * block default clip of 0 differs from the running vendor value, and the output
 * stage first showed completion status on hardware only once these matched.
 */
static const u32 ar_vif_rgb2yuv[] = {
	0x0027403f, 0x001c10bb, 0x00f98ea5, 0x00e67fd7,
	0x000001c1, 0x00000104, 0xc0f00eb0,
};

/*
 * Frame completion source. The interrupt is the default and the vendor's own
 * mode: the ISR services the same three W1C words the poll path services and
 * is validated at frame rate with no storm. Polling is kept as the debugging
 * fallback; it also leaves the status words latched between polls, which is
 * what an external sampler needs when chasing a status bit.
 *
 * The interrupt is requested only when this is set, so with polling the line
 * stays masked at the interrupt controller and an asserted VIF interrupt is
 * harmless.
 */
static bool use_irq = true;
module_param(use_irq, bool, 0444);
MODULE_PARM_DESC(use_irq,
		 "complete frames from the frame-done interrupt (default); 0 falls back to polling for debugging");

/* Event census: the poll acknowledges the W1C status words every few
 * milliseconds, so rare events vanish before an external sampler can see
 * them. With event_census set, every nonzero status read is logged and
 * OR-accumulated, and a summary is printed at stream stop.
 */
static bool event_census;
module_param(event_census, bool, 0444);
MODULE_PARM_DESC(event_census, "log every nonzero status word and print an event summary at stream stop");

static u32 census_or[3];
static u32 census_nonzero;
static u32 census_polls;


/* ISP-path status registers. The vendor's vif_ispintr_process reads 0x104 and
 * 0x108 for specific status bits, and 0x10c plus 0x110 on the ISP frame event
 * (bit 24), discarding all of them. Nothing in 0x104-0x138 is ever written by
 * the vendor, so these are read-only and sampling them cannot perturb the
 * block.
 *
 * They are the only visibility into whether pixels cross from the VIF front end
 * into the ISP, a hop that frame-start delimiters do not prove.
 * Whether they are clear-on-read or free-running is unestablished, so each is
 * read twice per poll and both values kept: a clear-on-read register reports
 * the accumulation since the previous poll and then near zero, while a
 * free-running counter reports the same value twice.
 *
 * Sampled here rather than inside the log call below because
 * dev_info_ratelimited() evaluates its arguments only when the rate limiter
 * admits the message, which would make the sample interval irregular.
 */
static const u16 ar_vif_isp_probe_reg[AR_VIF_ISP_PROBES] = {
	0x104, 0x108, 0x10c, 0x110
};

static u32 census_isp_or[AR_VIF_ISP_PROBES];
static u64 census_isp_sum[AR_VIF_ISP_PROBES];
static u32 census_isp_last[AR_VIF_ISP_PROBES][2];

/* Path 0 test pattern generator: frames are fabricated inside the block,
 * independent of the sensor and receiver. Isolates the view DMA path.
 */
static bool test_pattern;
module_param(test_pattern, bool, 0444);
MODULE_PARM_DESC(test_pattern, "feed view 0 from the path 0 test pattern generator");

/* View-control videoformat nibble override (register 0x000 bits[6:3]). The driver
 * writes the observed vendor word 0xc068, whose nibble is 13, the DPCM/HDR-encoder
 * format. The plain uncompressed RAW12 the IPI delivers has videoformat code 4, so a
 * -1 default keeps 0xc068 verbatim and a non-negative value overrides only the nibble.
 */
static int view_format = -1;
module_param(view_format, int, 0444);
MODULE_PARM_DESC(view_format, "override view-control videoformat nibble [6:3] (-1 keeps 0xc068, 4 is plain RAW12)");

/* The A-vs-B input-statistics measurement shows the bypass path delivers
 * lines three times the vendor's clock count from bit-identical input:
 * unpacked pixels into a view configured for packed 3-pixel groups
 * (videoformat 13, geometry in width/3 units). With unpacked set, the view
 * is programmed for what the pipeline actually delivers: videoformat 4
 * (plain RAW12, unless view_format overrides) and crop/frame-end in full
 * pixel units. Byte-based stride and DDR sizes are unit-independent.
 */
static bool unpacked;
module_param(unpacked, bool, 0444);
MODULE_PARM_DESC(unpacked, "program view geometry for unpacked pixels (full-width units, videoformat 4)");

/* Three values a write trace of the running vendor shows its capture is
 * configured with, and which a register snapshot cannot see because the vendor
 * restores all three when it tears the capture down: the view source mux, the
 * FIFO partition and the threshold nibbles. Each is separately switchable so a
 * failure can be bisected.
 */
static int view_mux = 8;
module_param(view_mux, int, 0444);
MODULE_PARM_DESC(view_mux, "view source mux nibble, where 8 + n selects MIPI receive pipe n (-1 leaves every nibble at 0xf)");

static bool fifo_split = true;
module_param(fifo_split, bool, 0444);
MODULE_PARM_DESC(fifo_split, "program the per-view FIFO partition the capture uses instead of the init-table value");

static bool view_th = true;
module_param(view_th, bool, 0444);
MODULE_PARM_DESC(view_th, "program the per-view threshold nibbles instead of leaving them zero");

/* Log every register write, so the driver's own sequence can be diffed against
 * a write trace of the running vendor.
 */
static bool trace_writes;
module_param(trace_writes, bool, 0444);
MODULE_PARM_DESC(trace_writes, "log every register write in order");

struct ar_vif {
	struct device *dev;
	void __iomem *base;
	struct clk *axi_clk;
	int irq;

	/*
	 * A v4l2 and media device with no video node of its own. The sensor and
	 * the CSI-2 receiver register their subdevs against it, which is what
	 * gives userspace their controls; the frames go out through the CVISP
	 * capture node instead.
	 */
	struct v4l2_device v4l2_dev;
	struct media_device media_dev;

	spinlock_t buffer_lock;		/* guards the armed view */

	/* Polling completion path: the work item samples the status register
	 * while streaming. streaming also gates the interrupt handler, so a
	 * spurious interrupt outside a capture touches no registers.
	 */
	struct delayed_work poll_work;
	bool streaming;
	u32 last_status;

	struct v4l2_pix_format format;

	/*
	 * Liveness counters, exported through debugfs.
	 *
	 * These exist because there is no honest liveness signal in the VIF
	 * register file. The status words a completion asserts are write-1-to
	 * clear, so under interrupt completion the handler clears them
	 * microseconds after they assert and an external sampler reads zero on
	 * a perfectly healthy pipeline. 0x1f8 was used as a frame counter on
	 * the assumption that it was one; it is not, it oscillates (0x1354,
	 * 0x134c, 0x134e on consecutive reads while streaming), so a
	 * strict-increase test on it both passes dead pipelines and fails live
	 * ones. That cost a night of misdiagnosis.
	 *
	 * Software counters have neither problem: monotonic, owned by us, and
	 * nothing can acknowledge them away. irq_events is the honest liveness
	 * signal for this pipeline and is what the bring-up harness gates on.
	 */
	struct dentry *debugfs;
	u32 irq_events;
	/*
	 * Frame completions the block signalled. Nothing consumes the frame:
	 * the view is armed at a scratch page so the block asserts at all, and
	 * the pixels leave through the ISP. Counted because it is a second
	 * liveness signal alongside irq_events, at frame rate rather than per
	 * interrupt.
	 */
	u32 frames;

	/*
	 * A frame the block can write when no v4l2 consumer has queued one. The
	 * view must be armed for the block to assert any interrupt, and the
	 * exported bring-up has no buffers of its own, so it points the write
	 * master here. Nothing reads it: the real pixel path runs sensor to ISP
	 * to CVISP and never comes back through this driver.
	 */
	void *scratch;
	dma_addr_t scratch_addr;

	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *source;
	u16 source_pad;
};

static void ar_vif_write(struct ar_vif *vif, u32 offset, u32 value)
{
	if (trace_writes)
		dev_info(vif->dev, "wr VIF+0x%03x = 0x%08x\n", offset, value);

	writel(value, vif->base + offset);
}

static u32 ar_vif_read(struct ar_vif *vif, u32 offset)
{
	return readl(vif->base + offset);
}

/* Point the view at a frame buffer.
 *
 * The address registers are consumed at the frame latch, so this must be called
 * once per frame. The vendor pulses the view-control latch bit only on the
 * initial arm; per-frame re-arms write the addresses alone and the hardware
 * latches them at the next frame boundary.
 */
static void ar_vif_arm_buffer(struct ar_vif *vif, dma_addr_t addr, bool pulse)
{
	u32 control;

	ar_vif_write(vif, AR_VIF_VIEW_ADDR_Y(AR_VIF_VIEW), (u32)addr);
	ar_vif_write(vif, AR_VIF_VIEW_ADDR_U(AR_VIF_VIEW), 0);
	ar_vif_write(vif, AR_VIF_VIEW_ADDR_V(AR_VIF_VIEW), 0);

	if (!pulse)
		return;

	control = ar_vif_read(vif, AR_VIF_VIEW_CONTROL(AR_VIF_VIEW));
	ar_vif_write(vif, AR_VIF_VIEW_CONTROL(AR_VIF_VIEW),
		     control | AR_VIF_VIEW_LATCH);
	ar_vif_write(vif, AR_VIF_VIEW_CONTROL(AR_VIF_VIEW), control);
}

/* Read-modify-write, clearing then setting the given bits. */
static void ar_vif_update(struct ar_vif *vif, u32 offset, u32 clear, u32 set)
{
	u32 value = ar_vif_read(vif, offset);

	ar_vif_write(vif, offset, (value & ~clear) | set);
}

/*
 * The block's one-time initialisation.
 *
 * Reproduces the vendor's vif_reg_init table walk, then its vif_reg_init_post
 * sequence, then vif_axi_config, then the block enable that vif_init writes
 * last. Run once per streaming session, with the block's clock already
 * running, because the block loses this state whenever its clock is gated.
 */
static void ar_vif_global_init(struct ar_vif *vif)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(ar_vif_init_table); i++) {
		ar_vif_write(vif, ar_vif_init_table[i].offset,
			     ar_vif_init_table[i].value);
	}

	/* Software reset pulse. */
	ar_vif_update(vif, AR_VIF_SOFT_RESET, 0, BIT(0));
	ar_vif_update(vif, AR_VIF_SOFT_RESET, BIT(0), 0);

	ar_vif_update(vif, AR_VIF_FRAME_CTRL, BIT(0), 0);
	ar_vif_update(vif, AR_VIF_FRAME_CTRL, 0xff000000, 0);

	ar_vif_update(vif, AR_VIF_FE_CONTROL, 0x000003ff, 0);
	ar_vif_update(vif, AR_VIF_VIEW_STABLE_CHECK, 0x0000ffff, 0);

	ar_vif_write(vif, AR_VIF_VIEW_MUX, 0xffffffff);
	ar_vif_write(vif, AR_VIF_BP_INTR_STATUS, 0xffffffff);
	ar_vif_write(vif, AR_VIF_BP_INTR_STATUS_B, 0xffffffff);

	ar_vif_write(vif, AR_VIF_BLOCK_ENABLE, 0x0000ffff);
	ar_vif_write(vif, AR_VIF_BLOCK_ENABLE, 0x000000ff);

	/* Acknowledge-then-arm on the two interrupt banks. */
	ar_vif_write(vif, AR_VIF_LINE_INTR_MASK, 0xffffffff);
	ar_vif_write(vif, AR_VIF_LINE_INTR_MASK, 0);
	ar_vif_write(vif, AR_VIF_EXTRA_INTR_MASK, 0xffffffff);
	ar_vif_write(vif, AR_VIF_EXTRA_INTR_MASK, 0);

	ar_vif_write(vif, AR_VIF_INTR_MASK_A, 0xc0000000);
	ar_vif_write(vif, AR_VIF_INTR_MASK_B, 0x00080000);

	for (unsigned int i = 0; i < ARRAY_SIZE(ar_vif_rgb2yuv); i++)
		ar_vif_write(vif, AR_VIF_RGB2YUV(i), ar_vif_rgb2yuv[i]);

	/* Input-pipe transient writes, issued between the init table and the
	 * FIFO reset. The 0x1c0 bit31 write is a pulse: the running block reads
	 * 0x822 and 0x1 here, and those values are restored below. 0x1d0 stays
	 * at 0x22 until the capture configuration selects the input format.
	 */
	ar_vif_write(vif, AR_VIF_INPUT_PIPE_PRESET, 0x00020002);
	ar_vif_write(vif, AR_VIF_INPUT_PIPE_CTRL, 0x80000000);
	ar_vif_update(vif, AR_VIF_INPUT_PIPE_MODE, 0x0000ffff, 0x00000020);
	ar_vif_update(vif, AR_VIF_INPUT_FORMAT_SELECT, 0x000000ff, 0x00000022);

	/* Reset every view's output FIFO with a bit31 set-then-clear pulse.
	 * This is the only reset the view output stage ever gets; it is not in
	 * the init table, and a view whose output FIFO was never reset accepts
	 * input yet writes nothing to DDR.
	 */
	for (unsigned int view = 0; view < 8; view++) {
		u32 value = ar_vif_read(vif, AR_VIF_VIEW_FIFO_BURST(view));

		ar_vif_write(vif, AR_VIF_VIEW_FIFO_BURST(view),
			     value | BIT(31));
		ar_vif_write(vif, AR_VIF_VIEW_FIFO_BURST(view),
			     value & ~BIT(31));
	}

	/* Restore the running block's input-pipe values after the transient
	 * writes above. 0x1c4 is wiped along with its neighbours (the init
	 * table value does not survive to a streaming readout) and reads
	 * 0x08020010 on the running block.
	 */
	ar_vif_write(vif, AR_VIF_INPUT_PIPE_CTRL, 0x00000822);
	ar_vif_write(vif, AR_VIF_INPUT_PIPE_CONFIG, 0x08020010);
	ar_vif_write(vif, AR_VIF_INPUT_PIPE_MODE, 0x00000001);

	ar_vif_write(vif, AR_VIF_EXTRA_INTR_CLEAR, 0);
	ar_vif_write(vif, AR_VIF_LINE_INTR_CLEAR, 0);

	/* Enable the AXI master, then set its run bit. */
	ar_vif_update(vif, AR_VIF_AXI_CONFIG, 0, AR_VIF_AXI_ENABLE);
	ar_vif_update(vif, AR_VIF_AXI_CONFIG, 0, AR_VIF_AXI_RUN);

	/* Frame-boundary and DDR-clip configuration; the vendor's frame-boundary
	 * and DDR-clip values clear the receiver's frame-boundary and packet error
	 * flags. Then fully release the block: global and every per-view soft reset
	 * cleared, so the view geometry ar_vif_configure() writes next latches into
	 * a live view rather than one still held in reset.
	 */
	ar_vif_write(vif, AR_VIF_FRAME_BOUNDARY, AR_VIF_FRAME_BOUNDARY_VALUE);
	ar_vif_write(vif, AR_VIF_DDR_CLIP, AR_VIF_DDR_CLIP_VALUE);
	ar_vif_write(vif, AR_VIF_SOFT_RESET, AR_VIF_SOFT_RESET_RELEASED);

	ar_vif_write(vif, AR_VIF_BLOCK_ENABLE, AR_VIF_BLOCK_ENABLE_VALUE);

	dev_info(vif->dev,
		 "global init done: axi 0x%08x, enable 0x%08x, fifo0 0x%08x\n",
		 ar_vif_read(vif, AR_VIF_AXI_CONFIG),
		 ar_vif_read(vif, AR_VIF_BLOCK_ENABLE),
		 ar_vif_read(vif, 0x140));
}

/* Program the front end and view 0 for the current format. */
static void ar_vif_configure(struct ar_vif *vif)
{
	u32 width = vif->format.width;
	u32 height = vif->format.height;
	u32 units = vif->format.bytesperline / AR_VIF_STRIDE_UNIT;
	u32 groups = DIV_ROUND_UP(width, AR_VIF_PIXELS_PER_GROUP);
	u32 stride;
	u32 view_control;

	if (unpacked)
		groups = width;

	/* Front end: input format, geometry and blanking. */
	ar_vif_write(vif, AR_VIF_FE_CONTROL, AR_VIF_FE_CONTROL_VALUE);
	ar_vif_write(vif, AR_VIF_FE_TIMING, AR_VIF_FE_TIMING_VALUE);
	ar_vif_write(vif, AR_VIF_FE_FORMAT, AR_VIF_FE_FORMAT_VALUE);
	ar_vif_write(vif, AR_VIF_FE_DATATYPE0, MIPI_CSI2_DT_RAW12);
	ar_vif_write(vif, AR_VIF_FE_DATATYPE1, MIPI_CSI2_DT_RAW12);

	/* The margin register carries the same geometry plus four pixels and
	 * four lines, matching the vendor's configuration.
	 */
	ar_vif_write(vif, AR_VIF_FE_GEOMETRY_MARGIN,
		     ((width + 4) << 16) | (height + 4));
	ar_vif_write(vif, AR_VIF_FE_GEOMETRY, (width << 16) | height);

	ar_vif_write(vif, AR_VIF_FE_HBLANK, AR_VIF_FE_HBLANK_VALUE);
	ar_vif_write(vif, AR_VIF_FE_VBLANK, AR_VIF_FE_VBLANK_VALUE);
	ar_vif_write(vif, AR_VIF_FE_LIMIT, AR_VIF_FE_LIMIT_VALUE);
	ar_vif_write(vif, AR_VIF_FE_EXTRA, AR_VIF_FE_EXTRA_VALUE);

	/* View 0, in the vendor's set_format order. The stride strobe is the
	 * first per-view touch: bit30 alone, then zero, then the final value
	 * with the enable bit.
	 */
	stride = (units & AR_VIF_STRIDE_FIELD_MASK) |
		 ((units & AR_VIF_STRIDE_FIELD_MASK) << 10) |
		 AR_VIF_STRIDE_ENABLE;

	ar_vif_write(vif, AR_VIF_VIEW_STRIDE0(AR_VIF_VIEW),
		     AR_VIF_STRIDE_COMMIT);
	ar_vif_write(vif, AR_VIF_VIEW_STRIDE0(AR_VIF_VIEW), 0);
	ar_vif_write(vif, AR_VIF_VIEW_STRIDE0(AR_VIF_VIEW), stride);

	/* The second stride word's bit31 is an edge, not a level: it is cleared,
	 * the view control is written, and only then is it set, so the rising
	 * edge commits the view configuration. Setting it before the view control
	 * leaves the bit already high, no edge is produced and nothing commits,
	 * while the register still reads the same value either way.
	 */
	ar_vif_write(vif, AR_VIF_VIEW_STRIDE1(AR_VIF_VIEW), 0);

	view_control = AR_VIF_VIEW_CONTROL_VALUE;
	if (view_format >= 0)
		view_control = (view_control & ~(0xfu << 3)) |
			       ((view_format & 0xf) << 3);
	else if (unpacked)
		view_control = (view_control & ~(0xfu << 3)) | (4u << 3);
	ar_vif_write(vif, AR_VIF_VIEW_CONTROL(AR_VIF_VIEW), view_control);

	ar_vif_write(vif, AR_VIF_VIEW_STRIDE1(AR_VIF_VIEW), AR_VIF_STRIDE_ENABLE);

	/* FIFO partition, in the vendor's two-step order: the split without the
	 * enable bit, then the same value with it. A register snapshot of the
	 * streaming vendor reads the init value here because the vendor rewrites
	 * it during a later teardown; the write trace shows the split is what the
	 * capture is configured with.
	 */
	if (fifo_split) {
		ar_vif_write(vif, AR_VIF_VIEW_FIFO(AR_VIF_VIEW),
			     AR_VIF_VIEW_FIFO_SPLIT);
		ar_vif_write(vif, AR_VIF_VIEW_FIFO(AR_VIF_VIEW),
			     AR_VIF_VIEW_FIFO_SPLIT | AR_VIF_VIEW_FIFO_ENABLE);
	} else {
		ar_vif_write(vif, AR_VIF_VIEW_FIFO(AR_VIF_VIEW), 0x86018060);
	}

	ar_vif_write(vif, AR_VIF_VIEW_FIFO_BURST(AR_VIF_VIEW),
		     AR_VIF_FIFO_ALLOCATION);

	/* View source mux: the nibble for this view selects its input, where
	 * 8 + n is MIPI receive pipe n. The reset and teardown value leaves every
	 * nibble at 0xf, which a snapshot cannot distinguish from a configured
	 * view; the write trace shows the capture runs with the camera's pipe
	 * selected.
	 */
	if (view_mux >= 0)
		ar_vif_write(vif, AR_VIF_VIEW_MUX,
			     (AR_VIF_VIEW_MUX_VALUE & ~(0xfu << (AR_VIF_VIEW * 4))) |
			     ((view_mux & 0xf) << (AR_VIF_VIEW * 4)));
	else
		ar_vif_write(vif, AR_VIF_VIEW_MUX, AR_VIF_VIEW_MUX_VALUE);
	ar_vif_write(vif, AR_VIF_VIEW_STABLE_CHECK, AR_VIF_VIEW_STABLE_CHECK_DISABLE);
	ar_vif_write(vif, AR_VIF_VIEW_THRESHOLD,
		     view_th ? AR_VIF_VIEW_THRESHOLD_VALUE : 0);

	/* Report what the crop and frame-end registers held before they are
	 * programmed. Nothing else in the driver writes them, so these are the
	 * block's own defaults.
	 */
	dev_info(vif->dev,
		 "view %u defaults: crop_v 0x%08x, crop_h 0x%08x, frame_end 0x%08x, fifo 0x%08x\n",
		 AR_VIF_VIEW,
		 ar_vif_read(vif, AR_VIF_VIEW_CROP_V(AR_VIF_VIEW)),
		 ar_vif_read(vif, AR_VIF_VIEW_CROP_H(AR_VIF_VIEW)),
		 ar_vif_read(vif, AR_VIF_VIEW_FRAME_END(AR_VIF_VIEW)),
		 ar_vif_read(vif, AR_VIF_VIEW_FIFO(AR_VIF_VIEW)));

	/* Crop window covering the whole frame, with cropping itself left
	 * disabled. The block does not default to the full frame, so a view that
	 * never writes these registers has no window to capture into.
	 */
	ar_vif_write(vif, AR_VIF_VIEW_CROP_V(AR_VIF_VIEW), height);
	ar_vif_write(vif, AR_VIF_VIEW_CROP_H(AR_VIF_VIEW), groups - 1);

	/* Frame-end check, which tells the view how large a frame to expect
	 * before it signals completion.
	 */
	ar_vif_write(vif, AR_VIF_VIEW_FRAME_END(AR_VIF_VIEW),
		     AR_VIF_FRAME_END_ENABLE | (height << 16) | groups);

	ar_vif_write(vif, AR_VIF_LINEBUFFER_HDR_MODE, 0);
	ar_vif_write(vif, AR_VIF_LINEBUFFER_INPUT, AR_VIF_LINEBUFFER_INPUT_VALUE);

	/* Input-path format select. The init table parks it at 0x27; the vendor
	 * writes 5 at capture configuration and streams with 5.
	 */
	ar_vif_write(vif, AR_VIF_INPUT_FORMAT_SELECT,
		     AR_VIF_INPUT_FORMAT_SELECT_VALUE);

	ar_vif_write(vif, AR_VIF_FE_INTR_MASK, 0xffffffff);

	if (test_pattern) {
		ar_vif_update(vif, AR_VIF_TEST_PATTERN, 0,
			      AR_VIF_TEST_PATTERN_ENABLE);
		dev_info(vif->dev, "path 0 test pattern enabled\n");
	}
}

static void ar_vif_stop(struct ar_vif *vif)
{
	if (event_census) {
		dev_info(vif->dev,
			 "census summary: bp-or 0x%08x intr-or 0x%08x w184-or 0x%08x (%u nonzero of %u polls)\n",
			 census_or[0], census_or[1], census_or[2],
			 census_nonzero, census_polls);

		/* An all-zero sum on every probe means nothing crossed into the
		 * ISP path for the whole session, whatever the registers count.
		 */
		for (unsigned int i = 0; i < AR_VIF_ISP_PROBES; i++)
			dev_info(vif->dev,
				 "census isp 0x%03x: or 0x%08x sum %llu last 0x%08x/0x%08x\n",
				 ar_vif_isp_probe_reg[i], census_isp_or[i],
				 census_isp_sum[i], census_isp_last[i][0],
				 census_isp_last[i][1]);

		census_or[0] = 0;
		census_or[1] = 0;
		census_or[2] = 0;
		census_nonzero = 0;
		census_polls = 0;
		memset(census_isp_or, 0, sizeof(census_isp_or));
		memset(census_isp_sum, 0, sizeof(census_isp_sum));
		memset(census_isp_last, 0, sizeof(census_isp_last));
	}

	ar_vif_write(vif, AR_VIF_FE_INTR_MASK, 0);
	ar_vif_write(vif, AR_VIF_VIEW_STRIDE0(AR_VIF_VIEW), 0);
	ar_vif_write(vif, AR_VIF_VIEW_CONTROL(AR_VIF_VIEW), 0);
	ar_vif_write(vif, AR_VIF_VIEW_STABLE_CHECK, 0);

	/* Input-path disable in the vendor's stop order: front-end control
	 * bit8, extra bit15, line-buffer input enable, then the ispif reset.
	 */
	ar_vif_update(vif, AR_VIF_FE_CONTROL, BIT(8), 0);
	ar_vif_update(vif, AR_VIF_FE_EXTRA, BIT(15), 0);
	ar_vif_update(vif, AR_VIF_ISPIF_ENABLE, AR_VIF_ISPIF_RUN, 0);

	/* Stop the AXI master and assert the block reset so no write is
	 * outstanding on the interconnect when the clock gates. An armed view
	 * whose DMA never completed otherwise leaves the master mid-transaction
	 * across the clock gate.
	 */
	ar_vif_update(vif, AR_VIF_AXI_CONFIG,
		      AR_VIF_AXI_RUN | AR_VIF_AXI_ENABLE, 0);
	ar_vif_write(vif, AR_VIF_SOFT_RESET, 0);
	ar_vif_update(vif, AR_VIF_SOFT_RESET, 0, BIT(0));
	ar_vif_update(vif, AR_VIF_SOFT_RESET, BIT(0), 0);
}

/* Detect completed frames.
 *
 * The default completion path. Completion is signalled by the W1C status at
 * 0x17c: bit view is view frame done, bit 8 + view is frame stable. The
 * armed address and bank 0 are read for diagnostics only; 0x184 is the
 * second W1C status word, serviced for the view-full recovery.
 */
static void ar_vif_poll_work(struct work_struct *work)
{
	struct ar_vif *vif = container_of(to_delayed_work(work), struct ar_vif,
					  poll_work);
	u32 armed_address;
	u32 bank0;
	u32 bp_status;
	u32 intr_status;
	u32 view_full;
	u32 done;

	if (!vif->streaming)
		return;

	armed_address = ar_vif_read(vif, AR_VIF_VIEW_ADDR_Y(AR_VIF_VIEW));

	/* Frame completion is carried in the bypass-view status at 0x17c and the
	 * block status at 0x1b0 (both W1C), where bit v is view v buffer done and
	 * bit 8 + v is view v frame done. 0x100 is NOT a live status here: it
	 * reads a constant 0x00ffffff on this block, so it must not gate
	 * completion (doing so fires a false done every poll). It is read for the
	 * diagnostic log only.
	 */
	bank0 = ar_vif_read(vif, AR_VIF_INTR_BANK(0));
	bp_status = ar_vif_read(vif, AR_VIF_BP_INTR_STATUS);
	intr_status = ar_vif_read(vif, AR_VIF_INTR_STATUS);

	if (bp_status)
		ar_vif_write(vif, AR_VIF_BP_INTR_STATUS, bp_status);
	if (intr_status)
		ar_vif_write(vif, AR_VIF_INTR_STATUS, intr_status);

	/* Second status word: on view full, recover the view's output FIFO with
	 * the bit31 pulse, as the vendor handler does.
	 */
	view_full = ar_vif_read(vif, AR_VIF_BP_INTR_STATUS_B);

	if (event_census) {
		census_polls++;
		census_or[0] |= bp_status;
		census_or[1] |= intr_status;
		census_or[2] |= view_full;

		for (unsigned int i = 0; i < AR_VIF_ISP_PROBES; i++) {
			u32 first = ar_vif_read(vif, ar_vif_isp_probe_reg[i]);
			u32 second = ar_vif_read(vif, ar_vif_isp_probe_reg[i]);

			census_isp_or[i] |= first;
			census_isp_sum[i] += first;
			census_isp_last[i][0] = first;
			census_isp_last[i][1] = second;
		}

		if (bp_status || intr_status || view_full) {
			census_nonzero++;
			dev_info_ratelimited(vif->dev,
					     "census: bp 0x%08x intr 0x%08x w184 0x%08x isp %08x/%08x %08x/%08x %08x/%08x %08x/%08x\n",
					     bp_status, intr_status, view_full,
					     census_isp_last[0][0], census_isp_last[0][1],
					     census_isp_last[1][0], census_isp_last[1][1],
					     census_isp_last[2][0], census_isp_last[2][1],
					     census_isp_last[3][0], census_isp_last[3][1]);
		}
	}

	if (view_full) {
		ar_vif_write(vif, AR_VIF_BP_INTR_STATUS_B, view_full);
		if (view_full & AR_VIF_BP_VIEW_FULL(AR_VIF_VIEW)) {
			u32 fifo = ar_vif_read(vif,
					AR_VIF_VIEW_FIFO_BURST(AR_VIF_VIEW));

			ar_vif_write(vif, AR_VIF_VIEW_FIFO_BURST(AR_VIF_VIEW),
				     fifo | BIT(31));
			ar_vif_write(vif, AR_VIF_VIEW_FIFO_BURST(AR_VIF_VIEW),
				     fifo & ~BIT(31));
			dev_info_ratelimited(vif->dev,
					     "view full: fifo reset, status 0x%08x\n",
					     view_full);
		}
	}

	done = AR_VIF_INTR_BUFFER_DONE(AR_VIF_VIEW) |
	       AR_VIF_INTR_FRAME_DONE(AR_VIF_VIEW);

	if ((bp_status | intr_status) & done) {
		if (vif->frames < 3)
			dev_info(vif->dev,
				 "frame done: addr 0x%08x, bank0 0x%08x, bp 0x%08x, intr 0x%08x\n",
				 armed_address, bank0, bp_status, intr_status);

		vif->frames++;
	}

	schedule_delayed_work(&vif->poll_work,
			      msecs_to_jiffies(AR_VIF_POLL_INTERVAL_MS));
}

/* Frame done, from the interrupt. Only used when use_irq is set.
 *
 * The line is level triggered, so every asserted W1C source must be
 * acknowledged in the handler or the line never deasserts and the spurious
 * detector kills it at 100000 events. The three status words serviced here
 * are the same three the poll path services: completion is carried in 0x17c,
 * 0x1b0 alone can read zero while 0x17c asserts, and 0x184 carries the
 * view-full condition with its FIFO recovery.
 */
static irqreturn_t ar_vif_irq(int irq, void *data)
{
	struct ar_vif *vif = data;
	u32 bp_status;
	u32 intr_status;
	u32 view_full;
	u32 done;

	/* Never touch a register outside a capture: the block's clock is off
	 * and the line is level triggered, so a stray interrupt here would
	 * otherwise hang or storm.
	 */
	if (!vif->streaming)
		return IRQ_NONE;

	bp_status = ar_vif_read(vif, AR_VIF_BP_INTR_STATUS);
	intr_status = ar_vif_read(vif, AR_VIF_INTR_STATUS);
	view_full = ar_vif_read(vif, AR_VIF_BP_INTR_STATUS_B);

	if (!bp_status && !intr_status && !view_full)
		return IRQ_NONE;

	vif->irq_events++;

	if (bp_status)
		ar_vif_write(vif, AR_VIF_BP_INTR_STATUS, bp_status);
	if (intr_status)
		ar_vif_write(vif, AR_VIF_INTR_STATUS, intr_status);

	if (view_full) {
		ar_vif_write(vif, AR_VIF_BP_INTR_STATUS_B, view_full);
		if (view_full & AR_VIF_BP_VIEW_FULL(AR_VIF_VIEW)) {
			u32 fifo = ar_vif_read(vif,
					AR_VIF_VIEW_FIFO_BURST(AR_VIF_VIEW));

			ar_vif_write(vif, AR_VIF_VIEW_FIFO_BURST(AR_VIF_VIEW),
				     fifo | BIT(31));
			ar_vif_write(vif, AR_VIF_VIEW_FIFO_BURST(AR_VIF_VIEW),
				     fifo & ~BIT(31));
		}
	}

	done = AR_VIF_INTR_BUFFER_DONE(AR_VIF_VIEW) |
	       AR_VIF_INTR_FRAME_DONE(AR_VIF_VIEW);

	if ((bp_status | intr_status) & done)
		vif->frames++;

	return IRQ_HANDLED;
}

static void ar_vif_sync_format(struct ar_vif *vif);

/*
 * The one VIF, for the exported input-path calls.
 *
 * There is a single instance: one block, one node in the device tree, and the
 * callers are the other drivers in this camera stack rather than anything that
 * could hold a device pointer of its own.
 */
static struct ar_vif *ar_vif_instance;

/*
 * Bring the input path up.
 *
 * Everything the block needs to receive frames. Arming a view buffer is part of
 * the sequence rather than bracketing it: the addresses have to be latched
 * between the frame-backpressure strobe and the plane length.
 *
 * That step is not optional even though this driver never reads the buffer
 * back. Brought up with no address armed and no control latch pulsed, the block
 * asserts no interrupt at all: measured, with the sensor powered and the front
 * end enabled, irq_events stays at zero indefinitely. A caller with no buffers
 * of its own supplies the scratch page for this reason.
 */
static int ar_vif_input_on(struct ar_vif *vif)
{
	unsigned long flags;
	int ret;

	if (!vif->source)
		return -ENODEV;

	if (vif->streaming)
		return -EBUSY;

	ret = clk_prepare_enable(vif->axi_clk);
	if (ret)
		return ret;

	vif->frames = 0;

	ar_vif_sync_format(vif);
	ar_vif_global_init(vif);
	ar_vif_configure(vif);

	/* Clear any interrupt status left asserted by an earlier run; a stale
	 * pending status could hold off the view's address latch.
	 */
	ar_vif_write(vif, AR_VIF_BP_INTR_STATUS,
		     ar_vif_read(vif, AR_VIF_BP_INTR_STATUS));

	/* 0x1c4 is rewritten here because the init-time value does not survive
	 * to this point (the running block reads 0x08020010), while a write
	 * made this late holds.
	 */
	ar_vif_write(vif, AR_VIF_INPUT_PIPE_CONFIG, 0x08020010);

	/* The vendor's arming sequence: re-assert the frame-end check, strobe
	 * the frame-backpressure commit, latch the first buffer, then write the
	 * plane length and re-assert the stable-check bit.
	 */
	ar_vif_write(vif, AR_VIF_VIEW_FRAME_END(AR_VIF_VIEW),
		     AR_VIF_FRAME_END_ENABLE | (vif->format.height << 16) |
		     (unpacked ? vif->format.width :
		      DIV_ROUND_UP(vif->format.width, AR_VIF_PIXELS_PER_GROUP)));

	ar_vif_update(vif, AR_VIF_VIEW_FRAME_BP(AR_VIF_VIEW), 0,
		      AR_VIF_FRAME_BP_COMMIT);
	ar_vif_update(vif, AR_VIF_VIEW_FRAME_BP(AR_VIF_VIEW),
		      AR_VIF_FRAME_BP_COMMIT, 0);

	/*
	 * Arm the view at the scratch page. Nothing reads it, but the block
	 * asserts no interrupt at all with no address latched and no control
	 * pulse: measured, with the sensor powered and the front end enabled,
	 * irq_events stays at zero indefinitely.
	 */
	if (vif->scratch) {
		spin_lock_irqsave(&vif->buffer_lock, flags);
		ar_vif_arm_buffer(vif, vif->scratch_addr, true);
		spin_unlock_irqrestore(&vif->buffer_lock, flags);
	}

	ar_vif_write(vif, AR_VIF_VIEW_DDR_SIZE_Y(AR_VIF_VIEW),
		     vif->format.sizeimage);
	ar_vif_write(vif, AR_VIF_VIEW_DDR_SIZE_U(AR_VIF_VIEW), 0);
	ar_vif_write(vif, AR_VIF_VIEW_DDR_SIZE_V(AR_VIF_VIEW), 0);
	ar_vif_write(vif, AR_VIF_VIEW_STABLE_CHECK, AR_VIF_VIEW_STABLE_CHECK_DISABLE);

	/* Input-path enable, issued only after the view is armed: ispif reset
	 * release, front-end control and extra rewrite, line-buffer input
	 * enable. The values are the running block's; the position after the
	 * address latch is what this sequence adds.
	 */
	ar_vif_update(vif, AR_VIF_FE_INTR_MASK, 0, BIT(0));
	ar_vif_write(vif, AR_VIF_FE_CONTROL, AR_VIF_FE_CONTROL_VALUE);
	ar_vif_write(vif, AR_VIF_FE_EXTRA, AR_VIF_FE_EXTRA_VALUE);
	ar_vif_update(vif, AR_VIF_ISPIF_ENABLE, 0, AR_VIF_ISPIF_RUN);

	/* The completion path is armed only now, with the block configured and
	 * its clock running.
	 */
	vif->last_status = 0;
	vif->streaming = true;

	if (use_irq)
		enable_irq(vif->irq);
	else
		schedule_delayed_work(&vif->poll_work,
				      msecs_to_jiffies(AR_VIF_POLL_INTERVAL_MS));

	/* The sensor and receiver start last: the vendor's filter graph starts
	 * the sink-side VIF first and the sensor/CSI source last, so the first
	 * frame arrives with the view fully configured and armed.
	 */
	ret = v4l2_subdev_call(vif->source, video, s_stream, 1);
	if (ret) {
		vif->streaming = false;

		if (use_irq)
			disable_irq(vif->irq);
		else
			cancel_delayed_work_sync(&vif->poll_work);

		ar_vif_stop(vif);
		clk_disable_unprepare(vif->axi_clk);
		return ret;
	}

	dev_info(vif->dev, "streaming: %ux%u, completion by %s\n",
		 vif->format.width, vif->format.height,
		 use_irq ? "interrupt" : "polling");

	return 0;
}

/* Take the input path back down, in the reverse order it came up. */
static void ar_vif_input_off(struct ar_vif *vif)
{
	if (!vif->streaming)
		return;

	if (vif->source)
		v4l2_subdev_call(vif->source, video, s_stream, 0);

	/* Disarm completion before the block loses its clock. */
	vif->streaming = false;

	if (use_irq)
		disable_irq(vif->irq);
	else
		cancel_delayed_work_sync(&vif->poll_work);

	ar_vif_stop(vif);
	clk_disable_unprepare(vif->axi_clk);
}

/*
 * Seconds to wait for the pixel domain to come live, and the sampling interval.
 * The camera is time dependent and a first frame has been seen to take several
 * seconds, so this is generous; it only costs that long when the sensor is not
 * delivering at all, which is a failure either way.
 */
#define AR_VIF_LIVE_TIMEOUT_MS		8000
#define AR_VIF_LIVE_POLL_MS		20

/*
 * Bring the input path up and return only once frames are confirmed flowing.
 *
 * The caller's next step configures the ISP, which reads registers, and a read
 * with the pixel domain dead hangs the SoC with no diagnostic. So this waits
 * rather than returning as soon as the sequence has been issued.
 *
 * The liveness signal is irq_events, which is monotonic and cannot be
 * acknowledged away; the status word a completion asserts is write-1-to-clear
 * and the handler clears it microseconds later, so sampling that reads zero on
 * a healthy pipeline. Under polling completion there is no such counter, so
 * that mode falls back to waiting out the timeout.
 */
int ar_vif_input_start(void)
{
	struct ar_vif *vif = ar_vif_instance;
	unsigned int waited;
	u32 before;
	int ret;

	if (!vif)
		return -ENODEV;

	before = READ_ONCE(vif->irq_events);

	ret = ar_vif_input_on(vif);
	if (ret)
		return ret;

	for (waited = 0; waited < AR_VIF_LIVE_TIMEOUT_MS;
	     waited += AR_VIF_LIVE_POLL_MS) {
		msleep(AR_VIF_LIVE_POLL_MS);

		if (!use_irq)
			continue;

		if (READ_ONCE(vif->irq_events) != before) {
			dev_info(vif->dev, "input live after %u ms\n", waited);
			return 0;
		}
	}

	if (use_irq) {
		dev_err(vif->dev, "no frame event in %u ms, input is not live\n",
			AR_VIF_LIVE_TIMEOUT_MS);
		ar_vif_input_off(vif);
		return -ETIMEDOUT;
	}

	dev_info(vif->dev, "input settled for %u ms, polling has no counter to gate on\n",
		 AR_VIF_LIVE_TIMEOUT_MS);

	return 0;
}
EXPORT_SYMBOL_GPL(ar_vif_input_start);

void ar_vif_input_stop(void)
{
	if (ar_vif_instance)
		ar_vif_input_off(ar_vif_instance);
}
EXPORT_SYMBOL_GPL(ar_vif_input_stop);

static void ar_vif_sync_format(struct ar_vif *vif)
{
	struct v4l2_subdev_format source_format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	if (!vif->source)
		return;

	source_format.pad = vif->source_pad;

	if (v4l2_subdev_call_state_active(vif->source, pad, get_fmt,
					  &source_format))
		return;

	if (!source_format.format.width || !source_format.format.height)
		return;

	vif->format.width = source_format.format.width;
	vif->format.height = source_format.format.height;
	vif->format.bytesperline = vif->format.width * AR_VIF_BYTES_PER_PIXEL;
	vif->format.sizeimage = vif->format.bytesperline * vif->format.height;
}

/* The CSI-2 receiver has appeared; remember it.
 *
 * The link to our video device cannot be created here: the video device's
 * entity is only registered with the media device by video_register_device,
 * which runs later from the complete callback. Linking an unregistered entity
 * dereferences a list head that does not exist yet.
 */
static int ar_vif_notify_bound(struct v4l2_async_notifier *notifier,
			       struct v4l2_subdev *subdev,
			       struct v4l2_async_connection *asc)
{
	struct ar_vif *vif = container_of(notifier, struct ar_vif, notifier);
	int pad;

	pad = media_entity_get_fwnode_pad(&subdev->entity, asc->match.fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0) {
		dev_err(vif->dev, "%s has no source pad\n", subdev->name);
		return pad;
	}

	vif->source = subdev;
	vif->source_pad = pad;

	dev_info(vif->dev, "bound source %s pad %d\n", subdev->name, pad);

	return 0;
}

static void ar_vif_notify_unbind(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *subdev,
				 struct v4l2_async_connection *asc)
{
	struct ar_vif *vif = container_of(notifier, struct ar_vif, notifier);

	vif->source = NULL;
}

/* Every subdev is present; expose their nodes and the media graph. */
static int ar_vif_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct ar_vif *vif = container_of(notifier, struct ar_vif, notifier);
	int ret;

	ret = v4l2_device_register_subdev_nodes(&vif->v4l2_dev);
	if (ret)
		return ret;

	/* No pad link to create: there is no video node at this end, and the
	 * receiver's sink is the ISP, which is not a media entity.
	 */
	ret = media_device_register(&vif->media_dev);
	if (ret)
		return ret;

	dev_info(vif->dev, "graph complete: input ready, %ux%u SRGGB12\n",
		 vif->format.width, vif->format.height);

	return 0;
}

static const struct v4l2_async_notifier_operations ar_vif_notify_ops = {
	.bound = ar_vif_notify_bound,
	.unbind = ar_vif_notify_unbind,
	.complete = ar_vif_notify_complete,
};

static int ar_vif_register_source(struct ar_vif *vif)
{
	struct v4l2_async_connection *asc;
	struct fwnode_handle *endpoint;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(vif->dev), NULL);
	if (!endpoint) {
		dev_err(vif->dev, "no endpoint in the device tree node\n");
		return -ENXIO;
	}

	v4l2_async_nf_init(&vif->notifier, &vif->v4l2_dev);
	vif->notifier.ops = &ar_vif_notify_ops;

	asc = v4l2_async_nf_add_fwnode_remote(&vif->notifier, endpoint,
					      struct v4l2_async_connection);
	fwnode_handle_put(endpoint);

	if (IS_ERR(asc)) {
		v4l2_async_nf_cleanup(&vif->notifier);
		return PTR_ERR(asc);
	}

	ret = v4l2_async_nf_register(&vif->notifier);
	if (ret)
		v4l2_async_nf_cleanup(&vif->notifier);

	return ret;
}

static void ar_vif_set_default_format(struct ar_vif *vif)
{
	vif->format.width = AR_VIF_WIDTH;
	vif->format.height = AR_VIF_HEIGHT;
	vif->format.pixelformat = V4L2_PIX_FMT_SRGGB12;
	vif->format.field = V4L2_FIELD_NONE;
	vif->format.colorspace = V4L2_COLORSPACE_RAW;
	vif->format.bytesperline = AR_VIF_WIDTH * AR_VIF_BYTES_PER_PIXEL;
	vif->format.sizeimage = vif->format.bytesperline * AR_VIF_HEIGHT;
}

/* Undo the v4l2 and media device registration, in reverse order. */
static void ar_vif_unregister_devices(struct ar_vif *vif)
{
	v4l2_device_unregister(&vif->v4l2_dev);
	media_device_cleanup(&vif->media_dev);
}

static void ar_vif_release_rmem(void *dev)
{
	of_reserved_mem_device_release(dev);
}

static int ar_vif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ar_vif *vif;
	int ret;

	/* Capture buffers must come from the low-RAM pool the AXI write master
	 * can reach; without the pool the default CMA sits above 0x28000000 and
	 * every DMA write vanishes.
	 */
	ret = of_reserved_mem_device_init(dev);
	if (ret)
		dev_warn(dev,
			 "no dedicated capture pool (%d); buffers may be unreachable\n",
			 ret);

	/* Unwound through devres, not from remove, so it runs after the scratch
	 * frame allocated from this pool. Releasing in remove would clear
	 * dev->dma_mem first and the deferred dmam free would then take the
	 * generic path. Registered even when the init failed: the release
	 * matches on the device and does nothing when nothing was attached.
	 */
	ret = devm_add_action_or_reset(dev, ar_vif_release_rmem, dev);
	if (ret)
		return ret;

	vif = devm_kzalloc(dev, sizeof(*vif), GFP_KERNEL);
	if (!vif)
		return -ENOMEM;

	vif->dev = dev;
	spin_lock_init(&vif->buffer_lock);
	ar_vif_set_default_format(vif);

	vif->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vif->base))
		return PTR_ERR(vif->base);

	vif->axi_clk = devm_clk_get(dev, "axi");
	if (IS_ERR(vif->axi_clk))
		return dev_err_probe(dev, PTR_ERR(vif->axi_clk),
				     "failed to get the axi clock\n");

	INIT_DELAYED_WORK(&vif->poll_work, ar_vif_poll_work);

	/* Requesting the interrupt is itself a hardware action: the line is
	 * level triggered and the block is unclocked at this point, so an
	 * already-asserted interrupt would fire into a handler that cannot
	 * safely read a register. It is requested only when asked for, and
	 * left disabled until streaming configures the block.
	 */
	if (use_irq) {
		vif->irq = platform_get_irq(pdev, 0);
		if (vif->irq < 0)
			return vif->irq;

		ret = devm_request_irq(dev, vif->irq, ar_vif_irq,
				       IRQF_NO_AUTOEN, "ar-vif", vif);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request the irq\n");
	}

	/* The capture graph needs a media device: the sensor and the CSI-2
	 * receiver are subdevs and the link between them has to be described.
	 */
	vif->media_dev.dev = dev;
	strscpy(vif->media_dev.model, "Artosyn VIF",
		sizeof(vif->media_dev.model));
	media_device_init(&vif->media_dev);

	vif->v4l2_dev.mdev = &vif->media_dev;

	dev_info(dev, "probe: registering the v4l2 device\n");

	ret = v4l2_device_register(dev, &vif->v4l2_dev);
	if (ret) {
		media_device_cleanup(&vif->media_dev);
		return ret;
	}

	dev_info(dev, "probe: registering the async notifier for the source\n");

	ret = ar_vif_register_source(vif);
	if (ret) {
		ar_vif_unregister_devices(vif);
		return ret;
	}

	vif->debugfs = debugfs_create_dir("ar-vif", NULL);
	debugfs_create_u32("irq_events", 0400, vif->debugfs, &vif->irq_events);
	debugfs_create_u32("frames", 0400, vif->debugfs, &vif->frames);

	/* Allocated once here rather than on the streaming path, so a bring-up
	 * cannot fail on an allocation. It comes from the same reservation the
	 * capture buffers do, which the write master can reach.
	 */
	vif->scratch = dmam_alloc_coherent(dev, vif->format.sizeimage,
					   &vif->scratch_addr, GFP_KERNEL);
	if (!vif->scratch)
		dev_warn(dev,
			 "no scratch frame: a bring-up with no queued buffer will not arm the view\n");

	dev_info(dev, "probe: complete\n");

	platform_set_drvdata(pdev, vif);
	ar_vif_instance = vif;

	return 0;
}

static void ar_vif_remove(struct platform_device *pdev)
{
	struct ar_vif *vif = platform_get_drvdata(pdev);

	/* Before anything else: the exported entry points reach this instance
	 * from another module, and one arriving during teardown would touch a
	 * block that is losing its clock.
	 */
	ar_vif_instance = NULL;

	/* Stop the completion path before anything is torn down. A polling work
	 * item left scheduled here runs after the module's code has been freed,
	 * which panics the machine on unload.
	 */
	vif->streaming = false;
	cancel_delayed_work_sync(&vif->poll_work);

	debugfs_remove_recursive(vif->debugfs);

	media_device_unregister(&vif->media_dev);
	v4l2_async_nf_unregister(&vif->notifier);
	v4l2_async_nf_cleanup(&vif->notifier);
	v4l2_device_unregister(&vif->v4l2_dev);
	media_device_cleanup(&vif->media_dev);
}

static const struct of_device_id ar_vif_of_match[] = {
	{ .compatible = "artosyn,vif" },
	{ }
};
MODULE_DEVICE_TABLE(of, ar_vif_of_match);

static struct platform_driver ar_vif_driver = {
	.probe = ar_vif_probe,
	.remove = ar_vif_remove,
	.driver = {
		.name = "ar-vif",
		.of_match_table = ar_vif_of_match,
	},
};
module_platform_driver(ar_vif_driver);

MODULE_DESCRIPTION("Artosyn VIF capture front end");
MODULE_LICENSE("GPL");
