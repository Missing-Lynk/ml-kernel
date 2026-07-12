// SPDX-License-Identifier: GPL-2.0
/*
 * artosyn_dsi.c - glue driver binding the mainline Synopsys dw-mipi-dsi bridge to the
 * Artosyn MIPI-DSI host @0x08850000. The DSI host registers (0x00..0xa4) are standard
 * DesignWare, so the mainline dw_mipi_dsi driver drives them; this glue supplies the
 * vendor phy_ops: lane bitrate, the custom Artosyn D-PHY register block at 0x2c4..0x3a4
 * (NOT a Synopsys gen-2 PHY), and the PHY reset. recovered from libmpp_service.so dsi_set_timing
 * / dphy_freq_conf_get. See ../../../../../docs/display-backlight.md.
 *
 * Values below are for the QY45043A0 1920x1080@60 case (24bpp, 4 lanes -> 891 Mbps/lane).
 * TODO: the per-bitrate D-PHY timing should be computed from the dphy_freq_conf buckets
 * rather than hardcoded; and several values are RE-[inferred] and need on-hardware tuning.
 */
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>

/* Custom Artosyn D-PHY block (offsets in the DSI host window). 891 Mbps bucket. */
#define DSI_PHY_RSTZ		0xa0
#define DSI_PHY_IF_CFG		0xa4

struct ar_dsi {
	struct device		*dev;
	void __iomem		*base;	/* 0x8850000; used for the vendor PHY block */
	struct clk		*pclk;
	struct dw_mipi_dsi	*dmd;
	struct dw_mipi_dsi_plat_data pdata;
	unsigned int		lanes;
};

/*
 * Custom Artosyn D-PHY timing block (0x2c4..0x3a4), captured verbatim from the working
 * stock display at 1080p60/891Mbps (the dphy_freq_conf RE got the layout right but ~half
 * the values wrong; these are the confirmed hardware values). Four per-lane sub-banks.
 */
static const struct { u16 off; u32 val; } ar_dphy_1080p60[] = {
	{ 0x2c4, 0x0410b016 }, { 0x2c8, 0x002468ac }, { 0x2cc, 0x0000000a },
	{ 0x2d0, 0x00003284 }, { 0x2d4, 0x00000064 }, { 0x2d8, 0x1e080b00 },
	{ 0x2dc, 0x110c1f0a }, { 0x2e0, 0x000a0000 }, { 0x2e4, 0x00000011 },
	{ 0x2e8, 0x0000206a }, { 0x2ec, 0x000000c4 }, { 0x2f0, 0x00c803e8 },
	{ 0x2f4, 0x0f080b00 }, { 0x2f8, 0x0000110c }, { 0x2fc, 0x003c1131 },
	{ 0x300, 0x000a0000 }, { 0x304, 0x00000011 }, { 0x308, 0x0000206a },
	{ 0x30c, 0x000000c4 }, { 0x310, 0x0f080b00 }, { 0x314, 0x0000110c },
	{ 0x318, 0x00000008 }, { 0x32c, 0x0000000a }, { 0x330, 0x00003284 },
	{ 0x334, 0x00000064 }, { 0x338, 0x1e080b00 }, { 0x33c, 0x110c1f0a },
	{ 0x340, 0x000a0000 }, { 0x344, 0x00000011 }, { 0x348, 0x0000206a },
	{ 0x34c, 0x000000c4 }, { 0x350, 0x00c803e8 }, { 0x354, 0x0f080b00 },
	{ 0x358, 0x0000110c }, { 0x35c, 0x003c1131 }, { 0x360, 0x000a0000 },
	{ 0x364, 0x00000011 }, { 0x368, 0x0000206a }, { 0x36c, 0x000000c4 },
	{ 0x370, 0x0f080b00 }, { 0x374, 0x0000110c }, { 0x378, 0x00000008 },
	{ 0x38c, 0x00011400 }, { 0x394, 0x00000200 }, { 0x398, 0x00008000 },
	{ 0x3a0, 0x00000200 }, { 0x3a4, 0x00008000 },
};

/*
 * Stock DSI-host register values for 1080p60, captured from the working (panel-lit) display and
 * diffed against the mainline dw-mipi-dsi config. The mainline programs several of these
 * DIFFERENTLY from stock, which keeps the panel dark:
 *  - 0x98/0x9c (PHY_TMR): mainline writes placeholder HS timing; stock leaves them 0 because the
 *    Artosyn custom D-PHY block (0x2c4..) owns the timing. The mainline values FIGHT that block,
 *    so the panel can't decode the HS data lanes -> black even though everything else is right.
 *  - 0x2c (PCKHDL_CFG): mainline enables EOTP/ECC/CRC (0x1d); stock uses BTA only (0x04).
 *  - 0x80/0x88/0x8c (LP timeout counts) and 0x18 (DPI LP cmd timing) also differ.
 * These run in ar_dsi_phy_init (phy_ops->init), which the bridge calls AFTER its own mode_set
 * writes, so these overrides win.
 */
static const struct { u16 off; u32 val; } ar_dsi_vid_1080p60[] = {
	{ 0x08, 0x00000811 },	/* CLKMGR_CFG */
	{ 0x18, 0x00000000 },	/* DPI_LP_CMD_TIM (stock 0, mainline 0x100004) */
	{ 0x2c, 0x00000004 },	/* PCKHDL_CFG = BTA_EN only; no EOTP/ECC/CRC (stock) */
	{ 0x38, 0x00003f01 },	/* VID_MODE_CFG (sync-event, all-LP) */
	{ 0x48, 0x00000020 },	/* VID_HSA_TIME */
	{ 0x4c, 0x00000020 },	/* VID_HBP_TIME */
	{ 0x50, 0x00000620 },	/* VID_HLINE_TIME */
	{ 0x54, 0x00000001 },	/* VID_VSA_LINES */
	{ 0x58, 0x0000001f },	/* VID_VBP_LINES */
	{ 0x5c, 0x00000020 },	/* VID_VFP_LINES */
	{ 0x80, 0x00000020 },	/* LP_RD_TO_CNT (stock) */
	{ 0x88, 0x00000020 },	/* LP_WR_TO_CNT (stock) */
	{ 0x8c, 0x00000020 },	/* BTA_TO_CNT (stock) */
	{ 0x94, 0x00000001 },	/* LPCLK_CTRL (continuous HS clock) */
	{ 0x98, 0x00000000 },	/* PHY_TMR_LPCLK_CFG = 0 (vendor D-PHY block owns timing) */
	{ 0x9c, 0x00000000 },	/* PHY_TMR_CFG = 0 (do NOT let mainline fight the vendor block) */
	{ 0xb4, 0x00000001 },	/* stock=1 (mainline 0) */
};

/* MIPI_TX_PLL Q8.24 divider at DSI offset 0x2c0 - the byte/lane clock the D-PHY locks to.
 * Without it PHY_STATUS bit0 (lock) never sets and the panel stays blank (confirmed live).
 * Value captured from stock (~864 Mbps/lane); the recovered clean-1080p60 891 MHz figure would
 * be 0x047d453e (libmpp_service.so ar9311_clk_set_freq, id 0x26).
 */
#define AR_DSI_MIPI_PLL_DIV	0x04a54972

static int ar_dsi_phy_init(void *priv)
{
	struct ar_dsi *dsi = priv;
	unsigned int i;

	writel(AR_DSI_MIPI_PLL_DIV, dsi->base + 0x2c0);

	for (i = 0; i < ARRAY_SIZE(ar_dphy_1080p60); i++)
		writel(ar_dphy_1080p60[i].val, dsi->base + ar_dphy_1080p60[i].off);

	for (i = 0; i < ARRAY_SIZE(ar_dsi_vid_1080p60); i++)
		writel(ar_dsi_vid_1080p60[i].val, dsi->base + ar_dsi_vid_1080p60[i].off);

	/* Lane count (PHY_IF_CFG = n_lanes - 1) and PHY reset deassert. */
	writel(dsi->lanes - 1, dsi->base + DSI_PHY_IF_CFG);
	writel(0xd, dsi->base + DSI_PHY_RSTZ);
	writel(0xf, dsi->base + DSI_PHY_RSTZ);

	return 0;
}

static int ar_dsi_get_lane_mbps(void *priv, const struct drm_display_mode *mode,
				unsigned long mode_flags, u32 lanes, u32 format,
				unsigned int *lane_mbps)
{
	int bpp = mipi_dsi_pixel_format_to_bpp(format);

	if (bpp < 0)
		bpp = 24;
	/* per-lane bitrate = pixelclock * bpp / lanes (891 Mbps for 1080p60/24bpp/4). */
	*lane_mbps = DIV_ROUND_UP(mode->clock * bpp, lanes * 1000);
	return 0;
}

/*
 * The firmware leaves PHY_TMR_CFG / PHY_TMR_LPCLK_CFG at reset defaults. The mainline
 * driver still wants get_timing; supply conservative LP<->HS transition counts. TODO:
 * tune on hardware (these gate DSI lock).
 */
static int ar_dsi_get_timing(void *priv, unsigned int lane_mbps,
			     struct dw_mipi_dsi_dphy_timing *timing)
{
	timing->clk_lp2hs = 0x40;
	timing->clk_hs2lp = 0x40;
	timing->data_lp2hs = 0x40;
	timing->data_hs2lp = 0x40;

	return 0;
}

static const struct dw_mipi_dsi_phy_ops ar_dsi_phy_ops = {
	.init		= ar_dsi_phy_init,
	.get_lane_mbps	= ar_dsi_get_lane_mbps,
	.get_timing	= ar_dsi_get_timing,
};

static int ar_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct ar_dsi *dsi;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->dev = dev;
	dsi->lanes = 4;	/* TODO: read "dsi,lanes"/panel; RE confirms n_lanes-1 at 0xa4. */
	platform_set_drvdata(pdev, dsi);

	/* Map the host window ourselves (no request) for the vendor PHY block; the
	 * mainline dw_mipi_dsi maps the same range for the standard registers.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	dsi->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!dsi->base)
		return -ENOMEM;

	dsi->pclk = devm_clk_get_optional_enabled(dev, "pclk");
	if (IS_ERR(dsi->pclk))
		return PTR_ERR(dsi->pclk);

	dsi->pdata.max_data_lanes = 4;
	dsi->pdata.phy_ops = &ar_dsi_phy_ops;
	dsi->pdata.priv_data = dsi;

	dsi->dmd = dw_mipi_dsi_probe(pdev, &dsi->pdata);
	if (IS_ERR(dsi->dmd))
		return dev_err_probe(dev, PTR_ERR(dsi->dmd), "dw_mipi_dsi_probe failed\n");

	return 0;
}

static void ar_dsi_remove(struct platform_device *pdev)
{
	struct ar_dsi *dsi = platform_get_drvdata(pdev);

	dw_mipi_dsi_remove(dsi->dmd);
}

static const struct of_device_id ar_dsi_match[] = {
	{ .compatible = "artosyn,dsi" },
	{}
};
MODULE_DEVICE_TABLE(of, ar_dsi_match);

static struct platform_driver ar_dsi_driver = {
	.probe	= ar_dsi_probe,
	.remove	= ar_dsi_remove,
	.driver = {
		.name		= "artosyn-dsi",
		.of_match_table	= ar_dsi_match,
	},
};
module_platform_driver(ar_dsi_driver);

MODULE_DESCRIPTION("Artosyn MIPI-DSI host glue for dw-mipi-dsi");
MODULE_LICENSE("GPL");
