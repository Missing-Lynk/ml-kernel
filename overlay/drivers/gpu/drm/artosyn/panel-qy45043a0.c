// SPDX-License-Identifier: GPL-2.0
/*
 * panel-qy45043a0.c - DRM panel driver for the QY45043A0 MIPI-DSI panel used in the
 * Artosyn/Proxima goggle (1920x1080@60, 4-lane, RGB888). The power-on GPIO sequence and
 * the 519-command DCS init table are recovered from the vendor ar_lowdelay (QY45043A0_power_on
 * @0x442830 + the init table at rodata 0x4c7a10). See ../../../../../docs/display-backlight.md.
 *
 * The init table (panel-qy45043a0.h) ends with Sleep-Out + Display-On, so the whole panel
 * power-up runs in .prepare(); .enable() only ungates the backlight via drm_panel.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <video/mipi_display.h>

#include "panel-qy45043a0.h"

struct qy45043a0 {
	struct drm_panel	panel;
	struct mipi_dsi_device	*dsi;
	struct gpio_desc	*reset;		/* global GPIO 43 */
	struct gpio_desc	*enable;	/* global GPIO 95 */
};

/* 1920x1080@60, standard CEA-861 1080p timing (recovered from get_vo_mipi_dsi_pub_attr):
 * HFP 88 / HSA 44 / HBP 148 (Htotal 2200), VFP 4 / VSA 5 / VBP 36 (Vtotal 1125),
 * pixel clock 148.5 MHz.
 */
static const struct drm_display_mode qy45043a0_mode = {
	.clock		= 148500,
	.hdisplay	= 1920,
	.hsync_start	= 1920 + 88,
	.hsync_end	= 1920 + 88 + 44,
	.htotal		= 1920 + 88 + 44 + 148,
	.vdisplay	= 1080,
	.vsync_start	= 1080 + 4,
	.vsync_end	= 1080 + 4 + 5,
	.vtotal		= 1080 + 4 + 5 + 36,
	/* TODO: physical panel dimensions not measured; placeholder (affects DPI only). */
	.width_mm	= 60,
	.height_mm	= 34,
};

static inline struct qy45043a0 *to_qy(struct drm_panel *panel)
{
	return container_of(panel, struct qy45043a0, panel);
}

static int qy_send(struct mipi_dsi_device *dsi, const struct qy_cmd *cmds, unsigned int n)
{
	unsigned int i;
	int ret;

	for (i = 0; i < n; i++) {
		u8 buf[2] = { cmds[i].cmd, cmds[i].param };

		ret = mipi_dsi_dcs_write_buffer(dsi, buf, 1 + cmds[i].plen);
		if (ret < 0)
			return ret;

		if (cmds[i].delay)
			msleep(cmds[i].delay);
	}

	return 0;
}

static int qy45043a0_prepare(struct drm_panel *panel)
{
	struct qy45043a0 *ctx = to_qy(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	int ret;

	/* Power/reset sequence (vendor QY45043A0_power_on): both low, raise reset, raise
	 * the second rail, 10 ms between steps. GPIOs are driven at raw level.
	 */
	gpiod_set_value_cansleep(ctx->enable, 0);
	gpiod_set_value_cansleep(ctx->reset, 0);
	usleep_range(10000, 11000);

	gpiod_set_value_cansleep(ctx->reset, 1);
	usleep_range(10000, 11000);

	gpiod_set_value_cansleep(ctx->enable, 1);
	usleep_range(10000, 11000);

	dev_info(&dsi->dev, "qy45043a0 prepare: reset done, sending %u init cmds\n",
		 (unsigned int)ARRAY_SIZE(qy45043a0_init));

	/* The whole sequence including the trailing Sleep-Out (0x11) + Display-On (0x29) runs here in
	 * COMMAND mode, before the bridge switches to video - this is the stock order (RE: ar_lowdelay
	 * customerHmLcdDetect -> all DCS in cmd mode -> AR_MPI_VO_Dsi_Enable flips to video). The panel
	 * scan direction is NOT set by this table; it is latched from two power-on strap GPIOs (see the
	 * panel-scan-* hogs in the DT).
	 */
	ret = qy_send(dsi, qy45043a0_init, ARRAY_SIZE(qy45043a0_init));
	if (ret < 0) {
		dev_err(&dsi->dev, "init table failed: %d\n", ret);
		return ret;
	}

	dev_info(&dsi->dev, "qy45043a0 prepare: init done (incl Sleep-Out + Display-On)\n");
	return 0;
}

static int qy45043a0_unprepare(struct drm_panel *panel)
{
	struct qy45043a0 *ctx = to_qy(panel);

	mipi_dsi_dcs_set_display_off(ctx->dsi);
	mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	msleep(20);

	/* Vendor power_off drops the reset line. */
	gpiod_set_value_cansleep(ctx->reset, 0);

	return 0;
}

static int qy45043a0_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &qy45043a0_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	return 1;
}

static const struct drm_panel_funcs qy45043a0_funcs = {
	.prepare	= qy45043a0_prepare,
	.unprepare	= qy45043a0_unprepare,
	.get_modes	= qy45043a0_get_modes,
};

static int qy45043a0_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct qy45043a0 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* Non-burst SYNC-EVENT mode (not sync-pulse): stock drives this panel with DSI
	 * VID_MODE_CFG = 0x3f01 (vid_mode_type=1). Declaring SYNC_PULSE made the bridge program
	 * 0x3f00 (type 0), which the panel won't lock video on.
	 */
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM;

	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset), "no reset gpio\n");

	ctx->enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable))
		return dev_err_probe(dev, PTR_ERR(ctx->enable), "no enable gpio\n");

	drm_panel_init(&ctx->panel, dev, &qy45043a0_funcs, DRM_MODE_CONNECTOR_DSI);

	/*
	 * The DSI host must be powered up and switched to command mode BEFORE we send the panel
	 * init sequence. Without this, drm_atomic_bridge_chain_pre_enable runs the panel's
	 * prepare() (the 519 DCS init commands) before the dw-mipi-dsi bridge's pre_enable, so the
	 * commands are transmitted on a dead link and silently lost - the panel never initializes
	 * and the screen stays black even though the VO/DSI registers are all correct.
	 */
	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "mipi_dsi_attach failed\n");
	}

	return 0;
}

static void qy45043a0_remove(struct mipi_dsi_device *dsi)
{
	struct qy45043a0 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id qy45043a0_of_match[] = {
	{ .compatible = "qiyang,qy45043a0" },
	{}
};
MODULE_DEVICE_TABLE(of, qy45043a0_of_match);

static struct mipi_dsi_driver qy45043a0_driver = {
	.probe	= qy45043a0_probe,
	.remove	= qy45043a0_remove,
	.driver = {
		.name		= "panel-qy45043a0",
		.of_match_table	= qy45043a0_of_match,
	},
};
module_mipi_dsi_driver(qy45043a0_driver);

MODULE_DESCRIPTION("QY45043A0 MIPI-DSI panel");
MODULE_LICENSE("GPL");
