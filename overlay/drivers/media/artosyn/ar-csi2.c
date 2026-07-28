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
 * The programming sequence reproduces the vendor's, and the register values it
 * produces were checked against a capture of the vendor stack streaming
 * 1920x1080 RAW12 over two lanes.
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
 * carries high-speed bursts, which makes them a direct routing probe.
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

/* IPI_MODE fields. Camera timing (bit0 clear) takes line timing from the
 * incoming stream, which is what a sensor link wants; controller timing would
 * additionally need HLINE and the four V-line registers.
 */
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

#define DW_CSI2_IPI_MODE_ENABLE		BIT(24)
#define DW_CSI2_IPI_MODE_CUT_THROUGH	BIT(16)
#define DW_CSI2_IPI_MODE_COLOR_48BIT	BIT(8)
#define DW_CSI2_IPI_MODE_CONTROLLER	BIT(0)

/* IPI_MEM_FLUSH bit8 flushes the IPI memory automatically on each frame. */
#define DW_CSI2_IPI_MEM_AUTO_FLUSH	BIT(8)

/* Written unconditionally by the vendor and read back verbatim on hardware. */
#define DW_CSI2_IPI_ADV_FEATURES_VALUE	0x013b0000

/* The version this driver has been checked against. */
#define DW_CSI2_VERSION_1_20		0x3132302a

/* Reset settle, from the vendor sequence. */
#define AR_CSI_RESET_SETTLE_US		10

/* RAW12 on every mode this receiver is used with. */
#define AR_CSI_BITS_PER_PIXEL		12

/* The two DesignWare cores, relative to the pair base. */
#define AR_CSI_CORE0_OFFSET		0x400
#define AR_CSI_CORE1_OFFSET		0x800

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

/*
 * IPI horizontal timings. In camera timing mode these set the blanking the IPI
 * inserts; they are a property of the sensor mode rather than of the receiver.
 * These are the values the vendor programs for the 1920x1080 two-lane RAW12
 * mode, read back from hardware.
 */
#define AR_CSI_IPI_HSA_TIME		0x1c
#define AR_CSI_IPI_HBP_TIME		0x1c
#define AR_CSI_IPI_HSD_TIME		0x39

#define AR_CSI2_PAD_SINK		0
#define AR_CSI2_PAD_SOURCE		1
#define AR_CSI2_PAD_COUNT		2

/*
 * Lane count override, for determining how many MIPI lanes the board actually
 * routes. The device tree declares what the vendor uses, which is a lower bound
 * rather than a measurement: lanes that are never driven read idle whether or
 * not they are connected.
 */
static int lanes;
module_param(lanes, int, 0444);
MODULE_PARM_DESC(lanes,
		 "override the data lane count from the device tree (0 to use it)");

/*
 * Run a lane-routing test once the sensor binds, instead of waiting for a
 * capture. The receiver is configured, the sensor is told to stream, and the
 * D-PHY per-lane stop state is sampled: a lane that is not routed can never
 * leave stop state, so it never reports. The stream is stopped again
 * immediately, and nothing is written to the board that a power cycle does not
 * clear.
 *
 * Pair this with the sensor driver's force_mode so the sensor actually drives
 * the lanes being tested; testing four lanes with the sensor in a two-lane mode
 * measures nothing.
 */
static bool lane_test;
module_param(lane_test, bool, 0444);
MODULE_PARM_DESC(lane_test,
		 "on bind, stream briefly and report which data lanes carry HS bursts");

/*
 * Overrides the D-PHY frequency range, which is otherwise derived from the
 * source subdev's reported link frequency. Present so a range can be tried
 * without rebuilding, since the derived value depends on the source getting
 * its own link frequency right.
 */
static int phy_range = -1;
module_param(phy_range, int, 0444);
MODULE_PARM_DESC(phy_range,
		 "D-PHY frequency range 0-7, or -1 to derive it from the source link frequency");

/* HS-settle for D-PHY internal registers 0x03 and 0x0a. Not optional: the PHY
 * reset default samples the HS burst at the wrong instant, which shows up as
 * SoT sync errors on every lane (INT_ST_PHY bits 0 and 1) plus double-ECC and
 * frame-boundary errors, no measured geometry in the VIF front end, and no
 * frame starts. 0x03 is the value the streaming vendor writes, recovered from
 * its D-PHY test-interface writes in the MMIO trace, and hardware-confirmed:
 * with it the error banks read all-zero in steady state exactly like the
 * vendor, the front end measures 1924x1084, and frame starts arrive at frame
 * rate. Zero restores the old skip-the-write behaviour, for bisecting only.
 */
static int hs_settle = 0x03;
module_param(hs_settle, int, 0444);
MODULE_PARM_DESC(hs_settle,
		 "D-PHY HS-settle byte for internal registers 0x03/0x0a (vendor value 0x03; 0 skips the write and breaks the link)");

/* How long to let the link settle after stream-on, and how far apart the two
 * counter samples sit. At 60 fps every active lane bursts thousands of times
 * over the window, so any advance at all separates routed from unrouted.
 */
#define AR_CSI_LANE_TEST_SETTLE_MS	50
#define AR_CSI_LANE_TEST_SAMPLE_MS	500

/* Lets the rest of the graph finish binding before the test streams. */
#define AR_CSI_LANE_TEST_DELAY_MS	1000

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

	/* Runs the optional lane test outside the async binding callbacks. */
	struct delayed_work lane_test_work;
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

/* ar_csi2_set_irq_masks - write every interrupt mask, or zero to mask all. */
static void ar_csi2_set_irq_masks(struct ar_csi2 *csi2, bool unmask)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ar_csi2_irq_masks); i++) {
		ar_csi2_write(csi2->core, ar_csi2_irq_masks[i].offset,
			      unmask ? ar_csi2_irq_masks[i].value : 0);
	}
}

/* ar_csi2_phy_power_on - bring the D-PHY out of reset.
 *
 * The DesignWare sequence: hold the core and PHY in reset with the test
 * interface cleared, release the test interface, then power the PHY and
 * release its reset. The vendor reaches the same end state through its own
 * PHY vtable; the register values below match a hardware capture of a running
 * link.
 */
/* ar_csi2_update - read-modify-write a single register bit. */
static void ar_csi2_update(void __iomem *base, u32 offset, u32 bit, bool set)
{
	u32 value = ar_csi2_read(base, offset);

	if (set)
		value |= bit;
	else
		value &= ~bit;

	ar_csi2_write(base, offset, value);
}

/*
 * ar_csi2_phy_test_write - write one D-PHY internal register.
 *
 * The D-PHY has no memory-mapped registers of its own. They are reached
 * through a two-phase test interface: the address is clocked in with testen
 * asserted, then the data with testen clear.
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

/* ar_csi2_phy_test_read - read one D-PHY internal register. */
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

/* ar_csi2_phy_write_setup - the fixed D-PHY setup registers for one core. */
static void ar_csi2_phy_write_setup(void __iomem *core)
{
	ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_SETUP_A, AR_CSI_PHY_SETUP_A_VALUE);
	ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_SETUP_B, AR_CSI_PHY_SETUP_B_VALUE);
	ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_SETUP_C, AR_CSI_PHY_SETUP_C_VALUE);
	ar_csi2_phy_test_write(core, AR_CSI_PHY_REG_SETUP_D, AR_CSI_PHY_SETUP_D_VALUE);
}

/*
 * ar_csi2_phy_range_code - pick the D-PHY frequency range for a lane rate.
 *
 * The receiver selects one of eight coarse ranges rather than the finer
 * standard encoding. Rates are megabits per second on a single data lane.
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
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ranges); i++) {
		if (rate_mbps >= ranges[i].min && rate_mbps <= ranges[i].max)
			return ranges[i].code;
	}

	return -ERANGE;
}

/*
 * ar_csi2_phy_lane_rate_mbps - the sensor's per-lane bit rate.
 *
 * Taken from the source subdev's link frequency control, which is half the
 * per-lane bit rate because the D-PHY clocks data on both edges.
 *
 * This has to be the rate the source's PLL actually drives the lanes at, not
 * the average implied by the frame rate. The link idles between lines, so the
 * average is lower and selects too low a frequency range, which shows up as
 * unrecoverable header ECC errors rather than as an obviously dead link.
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

/*
 * ar_csi2_phy_power_on - bring the D-PHY up.
 *
 * Reproduces the vendor's PHY object: hold the PHY in reset, pulse the test
 * interface clear, program the internal setup registers and the frequency
 * range, then release shutdown and reset. Programming the range is the part
 * that matters. A PHY released without it samples against whatever its default
 * range implies, which corrupts packet headers and shows up as ECC and frame
 * errors rather than as an obviously dead link.
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

	/* HS-settle, immediately after the range write and before the deskew
	 * writes, which is where the vendor places it.
	 */
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

/*
 * ar_csi2_phy_power_on - bring up every D-PHY the link uses.
 *
 * Each core of the pair owns its own two-lane PHY; a link wider than two lanes
 * merges both, and the vendor runs the full power-up once per core. Powering
 * only the first core leaves lanes 2 and 3 physically shut down no matter what
 * the sensor drives.
 */
static void ar_csi2_phy_power_on(struct ar_csi2 *csi2)
{
	ar_csi2_phy_power_on_core(csi2, csi2->core);

	if (csi2->num_data_lanes > 2)
		ar_csi2_phy_power_on_core(csi2,
					  csi2->base + AR_CSI_CORE1_OFFSET);
}

/* ar_csi2_phy_set_deskew_core - set or clear the deskew bit on one core. */
static void ar_csi2_phy_set_deskew_core(void __iomem *core, bool enable)
{
	u8 value;

	value = ar_csi2_phy_test_read(core, AR_CSI_PHY_REG_DESKEW_A);
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

/*
 * ar_csi2_phy_enable_deskew - set the deskew bit in two D-PHY registers.
 *
 * The vendor performs this as a separate step after the PHY has been released,
 * not as part of the power-up sequence, so the ordering is kept. On a merged
 * link it runs once per core.
 */
static void ar_csi2_phy_enable_deskew(struct ar_csi2 *csi2)
{
	ar_csi2_phy_set_deskew_core(csi2->core, true);

	if (csi2->num_data_lanes > 2)
		ar_csi2_phy_set_deskew_core(csi2->base + AR_CSI_CORE1_OFFSET,
					    true);
}

/*
 * ar_csi2_phy_disable_deskew - clear the deskew bit again.
 *
 * The vendor's receiver init ends by clearing the bit it set during PHY
 * bring-up; the enabled state exists only for the duration of initialisation.
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
		ar_csi2_phy_set_deskew_core(csi2->base + AR_CSI_CORE1_OFFSET,
					    false);
}

static void ar_csi2_phy_power_off(struct ar_csi2 *csi2)
{
	ar_csi2_write(csi2->core, DW_CSI2_DPHY_RSTZ, 0);
	ar_csi2_write(csi2->core, DW_CSI2_PHY_SHUTDOWNZ, 0);
	ar_csi2_write(csi2->core, DW_CSI2_PHY_TEST_CTRL0, 1);
}

/* ar_csi2_configure - program the wrapper and the core for one CSI-2 link.
 *
 * @data_type: the CSI-2 data type the sensor emits (0x2c for RAW12).
 */
static void ar_csi2_configure(struct ar_csi2 *csi2, u8 data_type)
{
	u32 scenario;
	u32 mode;

	/* Core reset, held briefly. An earlier revision forced register 0xcc
	 * to 0 here, from an idle-vendor observation; the streaming-vendor
	 * state capture reads 0xcc = 1, so the register is left alone.
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
	 * the wrapper's merge on, in the vendor's order: after the PHYs are up,
	 * before the deskew step.
	 */
	if (csi2->num_data_lanes > 2) {
		void __iomem *core1 = csi2->base + AR_CSI_CORE1_OFFSET;

		ar_csi2_write(core1, DW_CSI2_RESETN, 0);
		udelay(AR_CSI_RESET_SETTLE_US);
		ar_csi2_write(core1, DW_CSI2_RESETN, 1);

		ar_csi2_write(csi2->base, AR_CSI_WRAP_LANE_MERGE, 1);
	}

	ar_csi2_phy_enable_deskew(csi2);

	/* IPI output: camera timing, cut-through, 48-bit colour, enabled. */
	mode = DW_CSI2_IPI_MODE_ENABLE | DW_CSI2_IPI_MODE_CUT_THROUGH;
	ar_csi2_write(csi2->core, DW_CSI2_IPI_MODE, mode);
	ar_csi2_write(csi2->core, DW_CSI2_IPI_ADV_FEATURES,
		      DW_CSI2_IPI_ADV_FEATURES_VALUE);
	ar_csi2_write(csi2->core, DW_CSI2_IPI_VCID, 0);
	ar_csi2_write(csi2->core, DW_CSI2_IPI_DATA_TYPE, data_type);
	ar_csi2_write(csi2->core, DW_CSI2_IPI_MEM_FLUSH,
		      DW_CSI2_IPI_MEM_AUTO_FLUSH);
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

	/* The last step of the vendor's receiver init: the deskew bit set during
	 * PHY bring-up is cleared before the link carries traffic.
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

/* ar_csi2_report_lanes - log the per-lane D-PHY stop state.
 *
 * PHY_STOPSTATE carries one bit per data lane. A lane that is configured but
 * never leaves stop state is either not driven by the sensor or not routed on
 * the board, which is otherwise a difficult failure to tell apart from a
 * timing problem.
 */
static void ar_csi2_report_lanes(struct ar_csi2 *csi2)
{
	u32 expected = GENMASK(csi2->num_data_lanes - 1, 0);
	u32 stopstate;

	stopstate = ar_csi2_read(csi2->core, DW_CSI2_PHY_STOPSTATE);

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
		clk_disable_unprepare(csi2->pcs_clk);
		clk_disable_unprepare(csi2->csi_clk);

		return 0;
	}

	ret = clk_prepare_enable(csi2->csi_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(csi2->pcs_clk);
	if (ret)
		goto error_csi_clk;

	/* RAW12 is the only format the sensor emits, so the data type is
	 * fixed; a multi-format sensor would derive it from the pad format.
	 */
	ar_csi2_configure(csi2, MIPI_CSI2_DT_RAW12);

	ret = v4l2_subdev_call(csi2->sensor, video, s_stream, 1);
	if (ret)
		goto error_stop;

	ar_csi2_report_lanes(csi2);

	return 0;

error_stop:
	ar_csi2_stop(csi2);
	clk_disable_unprepare(csi2->pcs_clk);

error_csi_clk:
	clk_disable_unprepare(csi2->csi_clk);

	return ret;
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
		ret = v4l2_subdev_call_state_active(csi2->sensor, pad, get_fmt,
						    &sensor_format);
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

/* ar_csi2_notify_bound - the sensor subdev has appeared; link it to our sink. */
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

	/* The lane test streams the sensor, which cannot be done from inside a
	 * binding callback, and the notifier's complete callback never runs for
	 * a subdev notifier: v4l2-async only calls it on the root notifier.
	 */
	if (lane_test)
		schedule_delayed_work(&csi2->lane_test_work,
				      msecs_to_jiffies(AR_CSI_LANE_TEST_DELAY_MS));

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

/* ar_csi2_run_lane_test - stream briefly and report which lanes are alive.
 *
 * Samples PHY_STOPSTATE repeatedly and combines the results: a routed lane
 * reports stop state whenever it is between high-speed bursts, while an
 * unrouted one never does. The error status is reported alongside, because a
 * lane count the board cannot support shows up there too.
 */
static void ar_csi2_run_lane_test(struct ar_csi2 *csi2)
{
	static const u32 counter_offsets[] = {
		AR_CSI_WRAP_PHY0_HS_D0,		/* lane 0 */
		AR_CSI_WRAP_PHY0_HS_D1,		/* lane 1 */
		AR_CSI_WRAP_PHY1_HS_D0,		/* lane 2 */
		AR_CSI_WRAP_PHY1_HS_D1,		/* lane 3 */
	};
	u32 expected = GENMASK(csi2->num_data_lanes - 1, 0);
	u32 before[ARRAY_SIZE(counter_offsets)];
	u32 seen = 0;
	unsigned int i;
	int ret;

	dev_info(csi2->dev, "lane test: configuring %u lanes\n",
		 csi2->num_data_lanes);

	ret = clk_prepare_enable(csi2->csi_clk);
	if (ret)
		return;

	ret = clk_prepare_enable(csi2->pcs_clk);
	if (ret)
		goto error_csi_clk;

	ar_csi2_configure(csi2, MIPI_CSI2_DT_RAW12);

	/*
	 * Stream, then measure lane activity from the wrapper's per-PHY HS
	 * counters. Each PHY of the pair owns two physical lanes; a counter
	 * whose value advances between two samples means that lane is routed
	 * and carrying high-speed bursts, one that stays put means it is not.
	 *
	 * This replaces two earlier methods that could not answer the
	 * question. Sampling PHY_RX reads the clock lane's status bits, which
	 * look identical at any lane count. Sampling PHY_STOPSTATE on the
	 * first core reads only that core's two lanes, so its ceiling is 0x3
	 * regardless of how the board is wired, and an idle sensor has not
	 * been given the mode's lane configuration yet in any case.
	 */
	ret = v4l2_subdev_call(csi2->sensor, video, s_stream, 1);
	if (ret) {
		dev_err(csi2->dev, "lane test: the sensor refused to stream (%d)\n",
			ret);
		goto error_stop;
	}

	msleep(AR_CSI_LANE_TEST_SETTLE_MS);

	for (i = 0; i < ARRAY_SIZE(counter_offsets); i++)
		before[i] = ar_csi2_read(csi2->base, counter_offsets[i]);

	msleep(AR_CSI_LANE_TEST_SAMPLE_MS);

	seen = 0;
	for (i = 0; i < ARRAY_SIZE(counter_offsets); i++) {
		if (ar_csi2_read(csi2->base, counter_offsets[i]) != before[i])
			seen |= BIT(i);
	}

	dev_info(csi2->dev,
		 "lane test: active lanes 0x%x of the configured 0x%x (per-PHY HS counters)\n",
		 seen, expected);
	dev_info(csi2->dev,
		 "lane test: streaming errors: main 0x%08x, stopstate core0 0x%08x core1 0x%08x\n",
		 ar_csi2_read(csi2->core, DW_CSI2_INT_ST_MAIN),
		 ar_csi2_read(csi2->core, DW_CSI2_PHY_STOPSTATE),
		 ar_csi2_read(csi2->base + AR_CSI_CORE1_OFFSET,
			      DW_CSI2_PHY_STOPSTATE));

	if ((seen & expected) == expected)
		dev_info(csi2->dev,
			 "lane test: all %u configured lanes carry data\n",
			 csi2->num_data_lanes);
	else
		dev_info(csi2->dev,
			 "lane test: lanes 0x%x carried no data; not routed, or the sensor does not drive them in this mode\n",
			 expected & ~seen);

	v4l2_subdev_call(csi2->sensor, video, s_stream, 0);

error_stop:
	ar_csi2_stop(csi2);
	clk_disable_unprepare(csi2->pcs_clk);

error_csi_clk:
	clk_disable_unprepare(csi2->csi_clk);
}

/* ar_csi2_lane_test_work - run the lane test away from the binding callbacks. */
static void ar_csi2_lane_test_work(struct work_struct *work)
{
	struct ar_csi2 *csi2 = container_of(to_delayed_work(work), struct ar_csi2,
					    lane_test_work);

	ar_csi2_run_lane_test(csi2);
}

/* No complete callback: v4l2-async only calls it on the root notifier, and this
 * is a subdev notifier, so anything placed there would never run.
 */
static const struct v4l2_async_notifier_operations ar_csi2_notify_ops = {
	.bound = ar_csi2_notify_bound,
	.unbind = ar_csi2_notify_unbind,
};

/* ar_csi2_parse_and_register_sensor - bind the sensor on the port 0 endpoint. */
static int ar_csi2_parse_and_register_sensor(struct ar_csi2 *csi2)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct v4l2_async_connection *asc;
	struct fwnode_handle *endpoint;
	unsigned int declared;
	int ret;

	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(csi2->dev),
						   AR_CSI2_PAD_SINK, 0,
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

	INIT_DELAYED_WORK(&csi2->lane_test_work, ar_csi2_lane_test_work);

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
	dev_info(dev, "probe: enabling the csi clock\n");

	ret = clk_prepare_enable(csi2->csi_clk);
	if (ret)
		return ret;

	dev_info(dev, "probe: reading the version register\n");

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
	snprintf(csi2->sd.name, sizeof(csi2->sd.name), "ar-csi2");
	v4l2_set_subdevdata(&csi2->sd, csi2);

	csi2->pads[AR_CSI2_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2->pads[AR_CSI2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&csi2->sd.entity, AR_CSI2_PAD_COUNT,
				     csi2->pads);
	if (ret)
		return ret;

	ret = v4l2_subdev_init_finalize(&csi2->sd);
	if (ret)
		goto error_media_entity;

	ret = ar_csi2_parse_and_register_sensor(csi2);
	if (ret)
		goto error_subdev;

	ret = v4l2_async_register_subdev(&csi2->sd);
	if (ret)
		goto error_notifier;

	platform_set_drvdata(pdev, csi2);

	dev_info(dev, "DesignWare CSI-2 host 1.20 up on %u data lanes\n",
		 csi2->num_data_lanes);

	return 0;

error_notifier:
	v4l2_async_nf_unregister(&csi2->notifier);
	v4l2_async_nf_cleanup(&csi2->notifier);

error_subdev:
	v4l2_subdev_cleanup(&csi2->sd);

error_media_entity:
	media_entity_cleanup(&csi2->sd.entity);

	return ret;
}

static void ar_csi2_remove(struct platform_device *pdev)
{
	struct ar_csi2 *csi2 = platform_get_drvdata(pdev);

	/* A pending test would otherwise run against freed module code. */
	cancel_delayed_work_sync(&csi2->lane_test_work);

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
