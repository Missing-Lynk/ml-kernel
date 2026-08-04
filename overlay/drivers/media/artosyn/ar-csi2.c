// SPDX-License-Identifier: GPL-2.0
/*
 * ar-csi2.c - Artosyn MIPI CSI-2 receiver subdev.
 *
 * Each receiver instance is a Synopsys DesignWare MIPI CSI-2 host controller
 * (the VERSION register reads ASCII "120*", version 1.20), wrapped in a small
 * Artosyn glue block that handles lane merging, IPI muxing and interrupt
 * aggregation. The DesignWare register map is therefore used as documented;
 * only the wrapper registers are Artosyn-specific.
 *
 * The block hosts eight instances arranged in four pairs. Each pair occupies
 * 0x1000: the wrapper at the pair base, and two DesignWare cores at +0x400 and
 * +0x800. A four-lane link merges both cores of a pair. This board uses one
 * two-lane instance.
 *
 * Register values match a capture of the vendor stack streaming 1920x1080
 * RAW12 over two lanes.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Wrapper registers, relative to the pair base. */
#define AR_CSI_WRAP_SCENARIO		0x008	/* bit0: merge both cores of the pair */
#define AR_CSI_WRAP_IPI_DATA_MUX	0x014
#define AR_CSI_WRAP_IRQ_MASK		0x028
#define AR_CSI_WRAP_IPI_CLOCK_MUX	0x03c	/* wrapper 0 only */
#define AR_CSI_WRAP_LANE_MERGE		0x040
#define AR_CSI_WRAP_NUM_LANES		0x04c	/* lanes - 1, mirrors the core */
#define AR_CSI_WRAP_INIT_ZERO		0x098

/* Free-running per-PHY activity counters, named by the vendor's debug dump:
 * clkbyte, hs_c, hs_d0, hs_d1, term_c, term_d0, term_d1 for each of the pair's
 * two PHYs. The data-lane HS counters advance only while that physical lane
 * carries high-speed bursts, which makes them a direct routing probe: sample
 * one twice while the sensor streams and any advance separates a routed lane
 * from an unrouted one. This is how the board's four-lane wiring was measured,
 * a question PHY_RX cannot answer (clock-lane bits, identical at any lane
 * count) and PHY_STOPSTATE on the first core cannot either (that core's two
 * lanes only, ceiling 0x3 regardless of wiring).
 *
 * Reference only; nothing here reads them. Kept because they are the sole way
 * to ask the routing question again on a new board.
 */
#define AR_CSI_WRAP_PHY0_HS_D0		0x060
#define AR_CSI_WRAP_PHY0_HS_D1		0x064
#define AR_CSI_WRAP_PHY1_HS_D0		0x07c
#define AR_CSI_WRAP_PHY1_HS_D1		0x080

/* DesignWare CSI-2 host registers, relative to the core base. */
#define DW_CSI2_VERSION			0x000
#define DW_CSI2_N_LANES			0x004	/* lanes - 1 */
#define DW_CSI2_RESETN			0x008
#define DW_CSI2_INT_ST_MAIN		0x00c
#define DW_CSI2_PHY_SHUTDOWNZ		0x040
#define DW_CSI2_DPHY_RSTZ		0x044
#define DW_CSI2_PHY_RX			0x048
#define DW_CSI2_PHY_STOPSTATE		0x04c	/* one bit per data lane */
#define DW_CSI2_PHY_TEST_CTRL0		0x050	/* bit0: testclr, bit1: testclk */
#define DW_CSI2_PHY_TEST_CTRL1		0x054	/* bit16: testen, bits 7:0: testdin */
#define DW_CSI2_IPI_MODE		0x080
#define DW_CSI2_IPI_VCID		0x084
#define DW_CSI2_IPI_DATA_TYPE		0x088
#define DW_CSI2_IPI_MEM_FLUSH		0x08c
#define DW_CSI2_IPI_HSA_TIME		0x090
#define DW_CSI2_IPI_HBP_TIME		0x094
#define DW_CSI2_IPI_HSD_TIME		0x098
#define DW_CSI2_IPI_HLINE_TIME		0x09c
#define DW_CSI2_IPI_ADV_FEATURES	0x0ac
#define DW_CSI2_IPI_VSA_LINES		0x0b0
#define DW_CSI2_IPI_VBP_LINES		0x0b4
#define DW_CSI2_IPI_VFP_LINES		0x0b8
#define DW_CSI2_IPI_VACTIVE_LINES	0x0bc

#define DW_CSI2_PHY_TEST_CLEAR		BIT(0)
#define DW_CSI2_PHY_TEST_CLOCK		BIT(1)
#define DW_CSI2_PHY_TEST_ENABLE		BIT(16)

/* testdout comes back in bits 15:8 of the test control register. */
#define DW_CSI2_PHY_TEST_DOUT_SHIFT	8
#define DW_CSI2_PHY_TEST_DOUT_MASK	0xff

/*
 * D-PHY internal registers, addressed through the test interface. The four
 * written with fixed values are part of the vendor's power-up sequence; the
 * two deskew registers get bit5 set after the range is programmed.
 */
#define AR_CSI_PHY_REG_FREQ_RANGE	0x00
#define AR_CSI_PHY_REG_SETUP_A		0x4d
#define AR_CSI_PHY_REG_SETUP_B		0x4e
#define AR_CSI_PHY_REG_SETUP_C		0x4f
#define AR_CSI_PHY_REG_SETUP_D		0x50
#define AR_CSI_PHY_REG_LANE_A		0x25
#define AR_CSI_PHY_REG_LANE_B		0x26
#define AR_CSI_PHY_REG_DESKEW_A		0x3d
#define AR_CSI_PHY_REG_DESKEW_B		0x45

#define AR_CSI_PHY_SETUP_A_VALUE	0x00
#define AR_CSI_PHY_SETUP_B_VALUE	0x08
#define AR_CSI_PHY_SETUP_C_VALUE	0x00
#define AR_CSI_PHY_SETUP_D_VALUE	0x08
#define AR_CSI_PHY_LANE_VALUE		0x07
#define AR_CSI_PHY_DESKEW_ENABLE	0x20

/* IPI_MODE fields. Camera timing (bit0 clear) takes line timing from the
 * incoming stream; controller timing would additionally need HLINE and the
 * four V-line registers.
 */
#define DW_CSI2_IPI_MODE_ENABLE		BIT(24)
#define DW_CSI2_IPI_MODE_CUT_THROUGH	BIT(16)
#define DW_CSI2_IPI_MODE_COLOR_48BIT	BIT(8)
#define DW_CSI2_IPI_MODE_CONTROLLER	BIT(0)

/* IPI_MEM_FLUSH bit8 flushes the IPI memory automatically on each frame. */
#define DW_CSI2_IPI_MEM_AUTO_FLUSH	BIT(8)

/* Written unconditionally by the vendor and read back verbatim on hardware. */
#define DW_CSI2_IPI_ADV_FEATURES_VALUE	0x013b0000

/* IPI horizontal timings. In camera timing mode these set the blanking the IPI
 * inserts; they are a property of the sensor mode rather than of the receiver.
 * Vendor values for the 1920x1080 two-lane RAW12 mode, read back from
 * hardware.
 */
#define AR_CSI_IPI_HSA_TIME		0x1c
#define AR_CSI_IPI_HBP_TIME		0x1c
#define AR_CSI_IPI_HSD_TIME		0x39

/* The version this driver has been checked against. */
#define DW_CSI2_VERSION_1_20		0x3132302a

/* Reset settle, from the vendor sequence. */
#define AR_CSI_RESET_SETTLE_US		10

/* RAW12 on every mode this receiver is used with. */
#define AR_CSI_BITS_PER_PIXEL		12

/* The two DesignWare cores, relative to the pair base. */
#define AR_CSI_CORE0_OFFSET		0x400
#define AR_CSI_CORE1_OFFSET		0x800

enum ar_csi2_pads {
	AR_CSI2_PAD_SINK,
	AR_CSI2_PAD_SOURCE,
	AR_CSI2_PAD_COUNT,
};

/* Per-instance interrupt masks, from the vendor's init table. Every entry is
 * written as zero to mask the source, then as its real value to unmask it.
 */
struct ar_csi2_irq_mask {
	u16 offset;
	u32 value;
};

static const struct ar_csi2_irq_mask ar_csi2_irq_masks[] = {
	{ 0x0e4, 0x0000000f },	/* PHY fatal */
	{ 0x0f4, 0x0001000d },	/* packet fatal */
	{ 0x104, 0x00010100 },	/* frame fatal */
	{ 0x114, 0x00030003 },	/* PHY */
	{ 0x124, 0x00010001 },	/* packet */
	{ 0x134, 0x00030001 },	/* line */
	{ 0x144, 0x0000001f },	/* IPI */
	{ 0x154, 0x0000001f },
	{ 0x164, 0x0000001f },
	{ 0x174, 0x0000001f },
};

/* Lane count override. The device tree declares what the vendor uses, a lower
 * bound rather than a measurement: undriven lanes read idle whether or not
 * they are connected.
 */
static int lanes;
module_param(lanes, int, 0444);
MODULE_PARM_DESC(lanes, "override the data lane count from the device tree (0 to use it)");

/* Overrides the D-PHY frequency range otherwise derived from the source
 * subdev's link frequency, which lets a range be tried without rebuilding.
 */
static int phy_range = -1;
module_param(phy_range, int, 0444);
MODULE_PARM_DESC(phy_range,
		 "D-PHY frequency range 0-7, or -1 to derive it from the source link frequency");

/* HS-settle for D-PHY internal registers 0x03 and 0x0a. Not optional: the PHY
 * reset default samples the HS burst at the wrong instant, giving SoT sync
 * errors on every lane (INT_ST_PHY bits 0 and 1), double-ECC and
 * frame-boundary errors, no measured geometry in the VIF front end, and no
 * frame starts. 0x03 is the vendor value. Zero skips the write, for bisecting
 * only.
 */
static int hs_settle = 0x03;
module_param(hs_settle, int, 0444);
MODULE_PARM_DESC(hs_settle,
		 "D-PHY HS-settle byte for internal registers 0x03/0x0a (vendor value 0x03; 0 skips the write and breaks the link)");

struct ar_csi2 {
	struct v4l2_subdev sd;
	struct media_pad pads[AR_CSI2_PAD_COUNT];
	struct device *dev;

	void __iomem *base;		/* pair base; the core sits at +0x400 */
	void __iomem *core;

	struct clk *csi_clk;
	struct clk *pcs_clk;

	u8 num_data_lanes;

	/* The sensor, bound asynchronously through the port 0 endpoint. */
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *sensor;
	u16 sensor_source_pad;
};

static inline struct ar_csi2 *to_ar_csi2(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar_csi2, sd);
}

static void ar_csi2_write(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

static u32 ar_csi2_read(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

/* Write every interrupt mask, or zero to mask all. */
static void ar_csi2_set_irq_masks(struct ar_csi2 *csi2, bool unmask)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(ar_csi2_irq_masks); i++) {
		ar_csi2_write(csi2->core, ar_csi2_irq_masks[i].offset,
			      unmask ? ar_csi2_irq_masks[i].value : 0);
	}
}

/* Read-modify-write a single register bit. */
static void ar_csi2_update(void __iomem *base, u32 offset, u32 bit, bool set)
{
	u32 value = ar_csi2_read(base, offset);

	if (set)
		value |= bit;
	else
		value &= ~bit;

	ar_csi2_write(base, offset, value);
}

/* Write one D-PHY internal register. The D-PHY has no memory-mapped registers;
 * the address is clocked in with testen asserted, then the data with testen
 * clear.
 */
static void ar_csi2_phy_test_write(void __iomem *core, u8 code, u8 value)
{
	ar_csi2_write(core, DW_CSI2_PHY_TEST_CTRL1, code);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL1, DW_CSI2_PHY_TEST_ENABLE, true);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLOCK, true);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLOCK, false);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL1, DW_CSI2_PHY_TEST_ENABLE, false);

	ar_csi2_write(core, DW_CSI2_PHY_TEST_CTRL1, value);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLOCK, true);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLOCK, false);
}

/* Read one D-PHY internal register. */
static u8 ar_csi2_phy_test_read(void __iomem *core, u8 code)
{
	u32 value;

	ar_csi2_write(core, DW_CSI2_PHY_TEST_CTRL1, code);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL1, DW_CSI2_PHY_TEST_ENABLE, true);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLOCK, true);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLOCK, false);

	ar_csi2_write(core, DW_CSI2_PHY_TEST_CTRL1, 0);
	value = ar_csi2_read(core, DW_CSI2_PHY_TEST_CTRL1);

	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL1, DW_CSI2_PHY_TEST_ENABLE, false);

	return (value >> DW_CSI2_PHY_TEST_DOUT_SHIFT) & DW_CSI2_PHY_TEST_DOUT_MASK;
}

/* Write the fixed D-PHY setup registers for one core. */
static void ar_csi2_phy_write_setup(void __iomem *core)
{
	ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_SETUP_A, AR_CSI_PHY_SETUP_A_VALUE);
	ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_SETUP_B, AR_CSI_PHY_SETUP_B_VALUE);
	ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_SETUP_C, AR_CSI_PHY_SETUP_C_VALUE);
	ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_SETUP_D, AR_CSI_PHY_SETUP_D_VALUE);
}

/* Pick the D-PHY frequency range for a lane rate. The receiver selects one of
 * eight coarse ranges rather than the finer standard encoding. Rates are
 * megabits per second on a single data lane.
 */
static int ar_csi2_phy_range_code(unsigned int rate_mbps)
{
	static const struct {
		unsigned int min;
		unsigned int max;
		u8 code;
	} ranges[] = {
		{    0,  160, 0 },
		{  160,  240, 1 },
		{  240,  360, 2 },
		{  360,  480, 3 },
		{  480,  640, 4 },
		{  640,  960, 5 },
		{  960, 1600, 6 },
		{ 1600, 2500, 7 },
	};

	for (unsigned int i = 0; i < ARRAY_SIZE(ranges); i++) {
		if (rate_mbps >= ranges[i].min && rate_mbps <= ranges[i].max)
			return ranges[i].code;
	}

	return -ERANGE;
}

/* The sensor's per-lane bit rate, from the source subdev's link frequency
 * control: half the per-lane bit rate, the D-PHY clocks data on both edges.
 * This must be the rate the source's PLL drives the lanes at. The link idles
 * between lines, so a rate derived from the frame rate averages too low and
 * selects too low a range, which shows up as unrecoverable header ECC errors
 * rather than a dead link.
 */
static int ar_csi2_phy_lane_rate_mbps(struct ar_csi2 *csi2)
{
	s64 link_freq;

	if (!csi2->sensor)
		return -ENODEV;

	link_freq = v4l2_get_link_freq(&csi2->sensor->entity.pads[csi2->sensor_source_pad],
				       AR_CSI_BITS_PER_PIXEL,
				       csi2->num_data_lanes);
	if (link_freq < 0)
		return link_freq;

	return div_u64(2 * link_freq, 1000000);
}

/* Bring one core's D-PHY up: hold the PHY in reset, pulse the test interface
 * clear, program the internal setup registers and the frequency range, then
 * release shutdown and reset. A PHY released without the range write samples
 * against its default range, which corrupts packet headers and shows up as
 * ECC and frame errors rather than a dead link.
 */
static void ar_csi2_phy_power_on_core(struct ar_csi2 *csi2, void __iomem *core)
{
	int rate_mbps;
	int code;

	/* Hold the PHY down and clear the test interface. */
	ar_csi2_write(core, DW_CSI2_PHY_SHUTDOWNZ, 0);
	ar_csi2_write(core, DW_CSI2_DPHY_RSTZ, 0);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLOCK, false);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLEAR, true);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLEAR, false);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLOCK, true);

	udelay(AR_CSI_RESET_SETTLE_US);

	/* A dummy test-interface cycle, which leaves the interface idle with the
	 * clock low so the writes below start from a known state.
	 */
	ar_csi2_write(core, DW_CSI2_PHY_TEST_CTRL1, 0);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL1, DW_CSI2_PHY_TEST_ENABLE, true);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLOCK, false);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL1, DW_CSI2_PHY_TEST_ENABLE, false);
	ar_csi2_write(core, DW_CSI2_PHY_TEST_CTRL1, 0);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLOCK, true);
	ar_csi2_update(core, DW_CSI2_PHY_TEST_CTRL0, DW_CSI2_PHY_TEST_CLOCK, false);

	/* Fixed setup registers, written before the range. */
	ar_csi2_phy_write_setup(core);

	ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_LANE_A, AR_CSI_PHY_LANE_VALUE);
	ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_LANE_B, AR_CSI_PHY_LANE_VALUE);

	/* The frequency range, either measured from the source or overridden. */
	if (phy_range >= 0) {
		code = phy_range;
		dev_info(csi2->dev, "d-phy range forced to %d\n", code);
	} else {
		rate_mbps = ar_csi2_phy_lane_rate_mbps(csi2);
		if (rate_mbps < 0) {
			dev_warn(csi2->dev,
				 "no link frequency from the source (%d); leaving the d-phy range at its default\n",
				 rate_mbps);
			code = -1;
		} else {
			code = ar_csi2_phy_range_code(rate_mbps);
			if (code < 0)
				dev_warn(csi2->dev,
					 "%d Mbps per lane is outside every d-phy range\n",
					 rate_mbps);
			else
				dev_info(csi2->dev,
					 "d-phy range %d for %d Mbps per lane\n",
					 code, rate_mbps);
		}
	}

	if (code >= 0)
		ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_FREQ_RANGE, code);

	/* HS-settle, after the range write and before the deskew writes. */
	if (hs_settle > 0) {
		ar_csi2_phy_test_write(core, 0x03, hs_settle);
		ar_csi2_phy_test_write(core, 0x0a, hs_settle);
		dev_dbg(csi2->dev, "d-phy hs-settle 0x%02x\n", hs_settle);
	} else {
		dev_warn(csi2->dev,
			 "d-phy hs-settle write skipped: the link will not deliver valid packets\n");
	}

	ar_csi2_write(core, DW_CSI2_PHY_SHUTDOWNZ, 1);

	udelay(AR_CSI_RESET_SETTLE_US);

	ar_csi2_write(core, DW_CSI2_DPHY_RSTZ, 1);
}

/* Bring up every D-PHY the link uses. Each core of the pair owns its own
 * two-lane PHY; a link wider than two lanes merges both. Powering only the
 * first core leaves lanes 2 and 3 physically shut down no matter what the
 * sensor drives.
 */
static void ar_csi2_phy_power_on(struct ar_csi2 *csi2)
{
	ar_csi2_phy_power_on_core(csi2, csi2->core);

	if (csi2->num_data_lanes > 2)
		ar_csi2_phy_power_on_core(csi2, csi2->base + AR_CSI_CORE1_OFFSET);
}

/* Set or clear the deskew bit on one core. */
static void ar_csi2_phy_set_deskew_core(void __iomem *core, bool enable)
{
	u8 value = ar_csi2_phy_test_read(core, AR_CSI_PHY_REG_DESKEW_A);

	if (enable)
		value |= AR_CSI_PHY_DESKEW_ENABLE;
	else
		value &= ~AR_CSI_PHY_DESKEW_ENABLE;

	ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_DESKEW_A, value);
	value = ar_csi2_phy_test_read(core, AR_CSI_PHY_REG_DESKEW_B);

	if (enable)
		value |= AR_CSI_PHY_DESKEW_ENABLE;
	else
		value &= ~AR_CSI_PHY_DESKEW_ENABLE;

	ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_DESKEW_B, value);
}

/* Set the deskew bit in two D-PHY registers. A separate step after the PHY is
 * released; on a merged link it runs once
 * per core.
 */
static void ar_csi2_phy_enable_deskew(struct ar_csi2 *csi2)
{
	ar_csi2_phy_set_deskew_core(csi2->core, true);
	if (csi2->num_data_lanes > 2)
		ar_csi2_phy_set_deskew_core(csi2->base + AR_CSI_CORE1_OFFSET, true);
}

/* Clear the deskew bit again; it is enabled only for the duration of init.
 * Left set, the receiver re-acquires the clock lane on every frame, which
 * corrupts the first packet header after each re-entry. The first packet of a
 * frame is frame start, so every frame is lost before it opens: the IPI FIFO
 * overflows and no frame ever completes, while the error registers show only
 * a single unrecoverable header ECC per frame.
 */
static void ar_csi2_phy_disable_deskew(struct ar_csi2 *csi2)
{
	ar_csi2_phy_set_deskew_core(csi2->core, false);
	if (csi2->num_data_lanes > 2)
		ar_csi2_phy_set_deskew_core(csi2->base + AR_CSI_CORE1_OFFSET, false);
}

static void ar_csi2_phy_power_off(struct ar_csi2 *csi2)
{
	ar_csi2_write(csi2->core, DW_CSI2_DPHY_RSTZ, 0);
	ar_csi2_write(csi2->core, DW_CSI2_PHY_SHUTDOWNZ, 0);
	ar_csi2_write(csi2->core, DW_CSI2_PHY_TEST_CTRL0, 1);
}

/* Program the wrapper and the core for one CSI-2 link. data_type is the CSI-2
 * data type the sensor emits (0x2c for RAW12).
 */
static void ar_csi2_configure(struct ar_csi2 *csi2, u8 data_type)
{
	u32 scenario;

	/* Core reset, held briefly. Register 0xcc reads 1 on a streaming
	 * vendor capture; leave it alone.
	 */
	ar_csi2_write(csi2->core, DW_CSI2_RESETN, 0);
	udelay(AR_CSI_RESET_SETTLE_US);
	ar_csi2_write(csi2->core, DW_CSI2_RESETN, 1);

	ar_csi2_write(csi2->base, AR_CSI_WRAP_INIT_ZERO, 0);

	/* Lane merge: a link wider than two lanes uses both cores of the pair. */
	scenario = ar_csi2_read(csi2->base, AR_CSI_WRAP_SCENARIO);
	if (csi2->num_data_lanes > 2)
		scenario |= BIT(0);
	else
		scenario &= ~BIT(0);

	ar_csi2_write(csi2->base, AR_CSI_WRAP_SCENARIO, scenario);
	ar_csi2_write(csi2->base, AR_CSI_WRAP_NUM_LANES, csi2->num_data_lanes - 1);
	ar_csi2_write(csi2->core, DW_CSI2_N_LANES, csi2->num_data_lanes - 1);

	/* Mask every interrupt source while the link is being brought up. */
	ar_csi2_set_irq_masks(csi2, false);
	ar_csi2_write(csi2->base, AR_CSI_WRAP_IRQ_MASK, 0xffffffff);

	ar_csi2_phy_power_on(csi2);

	/* A merged link also resets the second core's protocol layer and turns
	 * the wrapper's merge on: after the PHYs are up, before the deskew
	 * step.
	 */
	if (csi2->num_data_lanes > 2) {
		void __iomem *core1 = csi2->base + AR_CSI_CORE1_OFFSET;

		ar_csi2_write(core1, DW_CSI2_RESETN, 0);
		udelay(AR_CSI_RESET_SETTLE_US);
		ar_csi2_write(core1, DW_CSI2_RESETN, 1);

		ar_csi2_write(csi2->base, AR_CSI_WRAP_LANE_MERGE, 1);
	}

	ar_csi2_phy_enable_deskew(csi2);

	/* IPI output: camera timing, cut-through, enabled. */
	ar_csi2_write(csi2->core, DW_CSI2_IPI_MODE,
		      DW_CSI2_IPI_MODE_ENABLE | DW_CSI2_IPI_MODE_CUT_THROUGH);
	ar_csi2_write(csi2->core, DW_CSI2_IPI_ADV_FEATURES, DW_CSI2_IPI_ADV_FEATURES_VALUE);
	ar_csi2_write(csi2->core, DW_CSI2_IPI_VCID, 0);
	ar_csi2_write(csi2->core, DW_CSI2_IPI_DATA_TYPE, data_type);
	ar_csi2_write(csi2->core, DW_CSI2_IPI_MEM_FLUSH, DW_CSI2_IPI_MEM_AUTO_FLUSH);
	ar_csi2_write(csi2->core, DW_CSI2_IPI_HSA_TIME, AR_CSI_IPI_HSA_TIME);
	ar_csi2_write(csi2->core, DW_CSI2_IPI_HBP_TIME, AR_CSI_IPI_HBP_TIME);
	ar_csi2_write(csi2->core, DW_CSI2_IPI_HSD_TIME, AR_CSI_IPI_HSD_TIME);

	/* Route the IPI output. Both mux registers read zero on a running
	 * vendor link with this single-instance configuration.
	 */
	ar_csi2_write(csi2->base, AR_CSI_WRAP_IPI_DATA_MUX, 0);
	ar_csi2_write(csi2->base, AR_CSI_WRAP_IPI_CLOCK_MUX, 0);

	ar_csi2_set_irq_masks(csi2, true);
	ar_csi2_write(csi2->base, AR_CSI_WRAP_IRQ_MASK, 0);

	/* The deskew bit set during PHY bring-up is cleared before the link
	 * carries traffic.
	 */
	ar_csi2_phy_disable_deskew(csi2);
}

static void ar_csi2_stop(struct ar_csi2 *csi2)
{
	ar_csi2_write(csi2->core, DW_CSI2_IPI_MODE, 0);
	ar_csi2_set_irq_masks(csi2, false);
	ar_csi2_phy_power_off(csi2);
	ar_csi2_write(csi2->core, DW_CSI2_RESETN, 0);
}

static int ar_csi2_clks_enable(struct ar_csi2 *csi2)
{
	int ret;

	ret = clk_prepare_enable(csi2->csi_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(csi2->pcs_clk);
	if (ret)
		clk_disable_unprepare(csi2->csi_clk);

	return ret;
}

static void ar_csi2_clks_disable(struct ar_csi2 *csi2)
{
	clk_disable_unprepare(csi2->pcs_clk);
	clk_disable_unprepare(csi2->csi_clk);
}

/* Log the per-lane D-PHY stop state. PHY_STOPSTATE carries one bit per data
 * lane. A configured lane that never leaves stop state is either not driven by
 * the sensor or not routed on the board.
 */
static void ar_csi2_report_lanes(struct ar_csi2 *csi2)
{
	u32 expected = GENMASK(csi2->num_data_lanes - 1, 0);
	u32 stopstate = ar_csi2_read(csi2->core, DW_CSI2_PHY_STOPSTATE);

	if ((stopstate & expected) != expected) {
		dev_warn(csi2->dev,
			 "only lanes 0x%x of the expected 0x%x reached D-PHY stop state\n",
			 stopstate & expected, expected);
	} else {
		dev_dbg(csi2->dev, "all %u data lanes in stop state (0x%x)\n",
			csi2->num_data_lanes, stopstate);
	}
}

static int ar_csi2_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar_csi2 *csi2 = to_ar_csi2(sd);
	int ret;

	if (!csi2->sensor)
		return -ENODEV;

	if (!enable) {
		v4l2_subdev_call(csi2->sensor, video, s_stream, 0);
		ar_csi2_stop(csi2);
		ar_csi2_clks_disable(csi2);

		return 0;
	}

	ret = ar_csi2_clks_enable(csi2);
	if (ret)
		return ret;

	/* RAW12 is the only format the sensor emits, so the data type is
	 * fixed; a multi-format sensor would derive it from the pad format.
	 */
	ar_csi2_configure(csi2, MIPI_CSI2_DT_RAW12);

	ret = v4l2_subdev_call(csi2->sensor, video, s_stream, 1);
	if (ret) {
		ar_csi2_stop(csi2);
		ar_csi2_clks_disable(csi2);

		return ret;
	}

	ar_csi2_report_lanes(csi2);

	return 0;
}

/* The receiver does not transform the stream, so both pads carry the sensor's
 * format and the default state simply propagates it.
 */
static int ar_csi2_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state)
{
	struct v4l2_mbus_framefmt *sink;
	struct v4l2_mbus_framefmt *source;

	sink = v4l2_subdev_state_get_format(sd_state, AR_CSI2_PAD_SINK);
	source = v4l2_subdev_state_get_format(sd_state, AR_CSI2_PAD_SOURCE);

	sink->width = 1920;
	sink->height = 1080;
	sink->code = MEDIA_BUS_FMT_SRGGB12_1X12;
	sink->field = V4L2_FIELD_NONE;
	sink->colorspace = V4L2_COLORSPACE_RAW;

	*source = *sink;

	return 0;
}

static int ar_csi2_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *sink;
	struct v4l2_mbus_framefmt *source;

	/* The source pad mirrors the sink; only the sink is settable. */
	if (fmt->pad == AR_CSI2_PAD_SOURCE)
		return v4l2_subdev_get_fmt(sd, sd_state, fmt);

	sink = v4l2_subdev_state_get_format(sd_state, AR_CSI2_PAD_SINK);
	source = v4l2_subdev_state_get_format(sd_state, AR_CSI2_PAD_SOURCE);

	*sink = fmt->format;
	*source = fmt->format;

	return 0;
}

/* The receiver does not transform the stream, so the active format on both
 * pads is whatever the sensor currently emits. Forwarding the sensor's answer
 * keeps the pipeline truthful when a module parameter selects a sensor mode
 * without any userspace format negotiation.
 */
static int ar_csi2_get_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_format *fmt)
{
	struct ar_csi2 *csi2 = to_ar_csi2(sd);
	struct v4l2_subdev_format sensor_format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && csi2->sensor) {
		sensor_format.pad = csi2->sensor_source_pad;
		ret = v4l2_subdev_call_state_active(csi2->sensor, pad, get_fmt, &sensor_format);
		if (ret == 0) {
			fmt->format = sensor_format.format;
			return 0;
		}
	}

	return v4l2_subdev_get_fmt(sd, sd_state, fmt);
}

static const struct v4l2_subdev_video_ops ar_csi2_video_ops = {
	.s_stream = ar_csi2_set_stream,
};

static const struct v4l2_subdev_pad_ops ar_csi2_pad_ops = {
	.get_fmt = ar_csi2_get_pad_format,
	.set_fmt = ar_csi2_set_pad_format,
};

static const struct v4l2_subdev_ops ar_csi2_subdev_ops = {
	.video = &ar_csi2_video_ops,
	.pad = &ar_csi2_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar_csi2_internal_ops = {
	.init_state = ar_csi2_init_state,
};

/* The sensor subdev has appeared; link it to our sink pad. */
static int ar_csi2_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_connection *asc)
{
	struct ar_csi2 *csi2 = container_of(notifier, struct ar_csi2, notifier);
	int pad;

	pad = media_entity_get_fwnode_pad(&subdev->entity, subdev->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0) {
		dev_err(csi2->dev, "%s has no source pad\n", subdev->name);
		return pad;
	}

	csi2->sensor = subdev;
	csi2->sensor_source_pad = pad;

	return media_create_pad_link(&subdev->entity, pad, &csi2->sd.entity,
				     AR_CSI2_PAD_SINK,
				     MEDIA_LNK_FL_ENABLED |
				     MEDIA_LNK_FL_IMMUTABLE);
}

static void ar_csi2_notify_unbind(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_connection *asc)
{
	struct ar_csi2 *csi2 = container_of(notifier, struct ar_csi2, notifier);

	csi2->sensor = NULL;
}

/* No complete callback: v4l2-async only calls it on the root notifier, and this
 * is a subdev notifier, so anything placed there would never run.
 */
static const struct v4l2_async_notifier_operations ar_csi2_notify_ops = {
	.bound = ar_csi2_notify_bound,
	.unbind = ar_csi2_notify_unbind,
};

/* Bind the sensor on the port 0 endpoint. */
static int ar_csi2_parse_and_register_sensor(struct ar_csi2 *csi2)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct v4l2_async_connection *asc;
	struct fwnode_handle *endpoint;
	unsigned int declared;
	int ret;

	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(csi2->dev), AR_CSI2_PAD_SINK, 0,
						   FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!endpoint) {
		dev_err(csi2->dev, "no sink endpoint in the device tree node\n");
		return -ENXIO;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &bus_cfg);
	if (ret) {
		fwnode_handle_put(endpoint);
		return ret;
	}

	declared = bus_cfg.bus.mipi_csi2.num_data_lanes;
	v4l2_fwnode_endpoint_free(&bus_cfg);

	if (lanes > 0) {
		dev_info(csi2->dev,
			 "lane count overridden: device tree declares %u, using %d\n",
			 declared, lanes);
		declared = lanes;
	}

	if (declared != 1 && declared != 2 && declared != 4) {
		dev_err(csi2->dev, "%u data lanes is not supported\n", declared);
		fwnode_handle_put(endpoint);
		return -EINVAL;
	}

	csi2->num_data_lanes = declared;

	v4l2_async_subdev_nf_init(&csi2->notifier, &csi2->sd);
	csi2->notifier.ops = &ar_csi2_notify_ops;

	asc = v4l2_async_nf_add_fwnode_remote(&csi2->notifier, endpoint,
					      struct v4l2_async_connection);
	fwnode_handle_put(endpoint);

	if (IS_ERR(asc)) {
		v4l2_async_nf_cleanup(&csi2->notifier);
		return PTR_ERR(asc);
	}

	ret = v4l2_async_nf_register(&csi2->notifier);
	if (ret) {
		v4l2_async_nf_cleanup(&csi2->notifier);
		return ret;
	}

	return 0;
}

static int ar_csi2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ar_csi2 *csi2;
	u32 version;
	int ret;

	csi2 = devm_kzalloc(dev, sizeof(*csi2), GFP_KERNEL);
	if (!csi2)
		return -ENOMEM;

	csi2->dev = dev;

	csi2->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(csi2->base))
		return PTR_ERR(csi2->base);

	/* Instance 0: the wrapper is at the pair base, its first core at
	 * +0x400. This board wires a single receiver.
	 */
	csi2->core = csi2->base + AR_CSI_CORE0_OFFSET;

	csi2->csi_clk = devm_clk_get(dev, "csi");
	if (IS_ERR(csi2->csi_clk))
		return dev_err_probe(dev, PTR_ERR(csi2->csi_clk),
				     "failed to get the csi clock\n");

	csi2->pcs_clk = devm_clk_get(dev, "pcs");
	if (IS_ERR(csi2->pcs_clk))
		return dev_err_probe(dev, PTR_ERR(csi2->pcs_clk),
				     "failed to get the pcs clock\n");

	/* The version register is readable without the link running and
	 * identifies the core; a mismatch means the register map below does
	 * not describe this hardware.
	 */
	ret = clk_prepare_enable(csi2->csi_clk);
	if (ret)
		return ret;

	version = ar_csi2_read(csi2->core, DW_CSI2_VERSION);
	clk_disable_unprepare(csi2->csi_clk);

	if (version != DW_CSI2_VERSION_1_20) {
		dev_err(dev, "unexpected CSI-2 host version 0x%08x\n", version);
		return -ENODEV;
	}

	v4l2_subdev_init(&csi2->sd, &ar_csi2_subdev_ops);
	csi2->sd.internal_ops = &ar_csi2_internal_ops;
	csi2->sd.owner = THIS_MODULE;
	csi2->sd.dev = dev;
	csi2->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	csi2->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	strscpy(csi2->sd.name, "ar-csi2", sizeof(csi2->sd.name));
	v4l2_set_subdevdata(&csi2->sd, csi2);

	csi2->pads[AR_CSI2_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2->pads[AR_CSI2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&csi2->sd.entity, AR_CSI2_PAD_COUNT, csi2->pads);
	if (ret)
		return ret;

	ret = v4l2_subdev_init_finalize(&csi2->sd);
	if (ret) {
		media_entity_cleanup(&csi2->sd.entity);

		return ret;
	}

	ret = ar_csi2_parse_and_register_sensor(csi2);
	if (ret) {
		v4l2_subdev_cleanup(&csi2->sd);
		media_entity_cleanup(&csi2->sd.entity);

		return ret;
	}

	ret = v4l2_async_register_subdev(&csi2->sd);
	if (ret) {
		v4l2_async_nf_unregister(&csi2->notifier);
		v4l2_async_nf_cleanup(&csi2->notifier);
		v4l2_subdev_cleanup(&csi2->sd);
		media_entity_cleanup(&csi2->sd.entity);

		return ret;
	}

	platform_set_drvdata(pdev, csi2);

	dev_info(dev, "DesignWare CSI-2 host 1.20 up on %u data lanes\n", csi2->num_data_lanes);

	return 0;
}

static void ar_csi2_remove(struct platform_device *pdev)
{
	struct ar_csi2 *csi2 = platform_get_drvdata(pdev);

	v4l2_async_unregister_subdev(&csi2->sd);
	v4l2_async_nf_unregister(&csi2->notifier);
	v4l2_async_nf_cleanup(&csi2->notifier);
	v4l2_subdev_cleanup(&csi2->sd);
	media_entity_cleanup(&csi2->sd.entity);
}

static const struct of_device_id ar_csi2_of_match[] = {
	{ .compatible = "artosyn,mipi" },
	{ }
};
MODULE_DEVICE_TABLE(of, ar_csi2_of_match);

static struct platform_driver ar_csi2_driver = {
	.probe = ar_csi2_probe,
	.remove = ar_csi2_remove,
	.driver = {
		.name = "ar-csi2",
		.of_match_table = ar_csi2_of_match,
	},
};
module_platform_driver(ar_csi2_driver);

MODULE_DESCRIPTION("Artosyn MIPI CSI-2 receiver");
MODULE_LICENSE("GPL");
