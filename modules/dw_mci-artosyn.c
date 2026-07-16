// SPDX-License-Identifier: GPL-2.0
/*
 * dw_mci-artosyn.c - DesignWare MMC (dw_mmc) platform glue for the Artosyn
 * Proxima-9311 SDMMC controllers, open reimplementation of the vendor
 * "artosyn,proxima-dw-mshc" glue. It exists so the AR8030 SDIO host (mmc@1b00000)
 * gets a real controller clock: the SoC has no mainline clock driver and U-Boot
 * never used mmc0, so the generic snps,dw-mshc binding leaves the bus unclocked and
 * the AR8030 never answers. This glue programs the SDMMC clock the way the vendor
 * dw_mci_proxima_{parse_dt,init} do, then hands off to the dw_mci core.
 *
 * Built on the dw_mci core's exported platform API (dw_mci_pltfm_register etc.), so
 * it loads as an out-of-tree MODULE - no kernel rebuild. Bind it by giving the mmc
 * DT node compatible = "artosyn,proxima-dw-mshc" (dts/proxima-9311.dts provides the
 * mmc@1b00000 + mmc@1c00000 nodes). The clk_sel/clk_cfg/bus_hz module params are
 * debug-only knobs; the defaults (SEL 0x80, phase 0, DT bus_hz) are the values every
 * boot runs and are hardware-proven on both controllers.
 *
 * Clock model (recovered from dw_mci_proxima_init, Proxima-9311):
 *   - AR_CLK_CFG (0x0A10808C): phase/config; init clears its low 5 bits.
 *   - AR_CLK_SEL (0x0A108088): divider-select keyed on bus_hz -
 *       25 MHz -> 0x00, 50 MHz -> 0x40, 100 MHz -> 0x80, 200 MHz -> 0xc0 (default).
 *   The per-speed CM/DLL phase (execute_tuning/set_phase) is for HS/UHS modes and is
 *   NOT needed to enumerate; it is a TODO below.
 *
 * Board addresses are Proxima-9311-specific (see ../tools/board-vr04.h / BOARD-CONFIG.md).
 */
#define pr_fmt(fmt) "dw_mci-artosyn: " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mmc/host.h>

#include "dw_mmc.h"
#include "dw_mmc-pltfm.h"

/* Proxima-9311 SDMMC clock registers (board-specific; see BOARD-CONFIG.md). Values
 * confirmed by diffing a running stock (slot A) system against ours.
 */
#define AR_SDMMC_CLK_EN		0x0A108000UL	/* NEVER written - see ar_dwmmc_init */
#define AR_SDMMC_CLK_38		0x0A108038UL	/* NEVER written - see ar_dwmmc_init */
/* Per-controller clock SEL/CFG (the vendor dw_mci_proxima_parse_dt branches on dwmmc0
 * vs dwmmc1): mmc0 @0x1b00000 uses 0x88/0x8c, mmc1 @0x1c00000 (SD) uses 0xc0/0xc4.
 */
#define AR_SDMMC0_CLK_SEL	0x0A108088UL
#define AR_SDMMC0_CLK_CFG	0x0A10808CUL
#define AR_SDMMC1_CLK_SEL	0x0A1080C0UL
#define AR_SDMMC1_CLK_CFG	0x0A1080C4UL
#define AR_SDMMC0_BASE		0x01B00000ULL	/* mmc0: AR8030 RF SDIO */
#define AR_SDMMC1_BASE		0x01C00000ULL	/* mmc1: microSD card slot */
/* Replicate the full stock slot-A SDMMC0 clock state (confirmed by register diff). */
/* SEL = the bus_hz tap only (100 MHz -> 0x80), phase lives in CFG low-5 (init = 0).
 * The vendor proxima_init uses the bare tap. (0x108000/0x38 are NOT written at
 * all - see ar_dwmmc_init.)
 */
#define AR_CLK_SEL_VAL		0x80u

/* CGU clock GATES for the SDMMC source clock, recovered from the vendor dw_mci_proxima
 * clock-descriptor setup (gate reg + enable bit). The vendor's CGU clock driver
 * enables these at boot; our open kernel has no CGU clock driver, so the SDMMC
 * card-clock SOURCE is left gated -> the divider has no input -> the AR8030 never
 * sees a clock and CMD5 times out (-110). Enable them here.
 */
#define AR_CGU_SDMMC_GATE0	0x0A104024UL	/* SDMMC clock gate; enable = bit 22 */
#define AR_CGU_SDMMC_GATE1	0x0A104028UL	/* SDMMC clock gate; enable = bit 23 */
#define AR_CGU_GATE0_BIT	BIT(22)
#define AR_CGU_GATE1_BIT	BIT(23)

struct ar_dwmmc {
	void __iomem	*clk_en;	/* AR_SDMMC_CLK_EN */
	void __iomem	*clk_38;	/* AR_SDMMC_CLK_38 */
	void __iomem	*clk_sel;	/* AR_SDMMC_CLK_SEL */
	void __iomem	*clk_cfg;	/* AR_SDMMC_CLK_CFG */
	void __iomem	*cgu_gate0;	/* AR_CGU_SDMMC_GATE0 */
	void __iomem	*cgu_gate1;	/* AR_CGU_SDMMC_GATE1 */
};

/* Bring-up experiment knob: force a specific CLK_SEL value instead of the bus_hz
 * mapping (-1 = use the mapping). Lets us probe whether any tap yields a live clock.
 */
static int clk_sel = -1;
module_param(clk_sel, int, 0444);
/* Companion knob for the CFG/phase register (0x8c / 0xc4). */
static int clk_cfg = -1;
module_param(clk_cfg, int, 0444);
/* Override host->bus_hz (the ciu input freq dw_mci uses for divider math). The DT says
 * 100 MHz but slot-A's true value was unreadable (/dev/mem blocked); sweep to find the
 * freq where a card actually inits = the real ciu clock. 0 = leave the DT value.
 */
static int bus_hz;
module_param(bus_hz, int, 0444);

static int ar_dwmmc_parse_dt(struct dw_mci *host)
{
	struct ar_dwmmc *priv;

	priv = devm_kzalloc(host->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Pick the per-controller clock registers by the controller's physical base,
	 * exactly as the vendor parse_dt distinguishes dwmmc0 from dwmmc1.
	 */
	unsigned long sel = AR_SDMMC0_CLK_SEL;
	unsigned long cfg = AR_SDMMC0_CLK_CFG;
	struct resource res;

	if (host->dev->of_node &&
	    of_address_to_resource(host->dev->of_node, 0, &res) == 0 &&
	    res.start == AR_SDMMC1_BASE) {
		sel = AR_SDMMC1_CLK_SEL;
		cfg = AR_SDMMC1_CLK_CFG;
		dev_info(host->dev, "SDMMC1 (SD card) clock regs (SEL 0x%lx)\n", sel);
	} else {
		dev_info(host->dev, "SDMMC0 (AR8030) clock regs (SEL 0x%lx)\n", sel);
	}

	priv->clk_en  = devm_ioremap(host->dev, AR_SDMMC_CLK_EN, 4);
	priv->clk_38  = devm_ioremap(host->dev, AR_SDMMC_CLK_38, 4);
	priv->clk_sel = devm_ioremap(host->dev, sel, 4);
	priv->clk_cfg = devm_ioremap(host->dev, cfg, 4);
	priv->cgu_gate0 = devm_ioremap(host->dev, AR_CGU_SDMMC_GATE0, 4);
	priv->cgu_gate1 = devm_ioremap(host->dev, AR_CGU_SDMMC_GATE1, 4);
	if (!priv->clk_en || !priv->clk_38 || !priv->clk_sel || !priv->clk_cfg ||
	    !priv->cgu_gate0 || !priv->cgu_gate1) {
		dev_err(host->dev, "failed to map SDMMC clock registers\n");
		return -ENOMEM;
	}

	host->priv = priv;
	return 0;
}

static int ar_dwmmc_init(struct dw_mci *host)
{
	struct ar_dwmmc *priv = host->priv;
	u32 sel = (clk_sel >= 0) ? (u32)clk_sel : AR_CLK_SEL_VAL;
	u32 phase = (clk_cfg >= 0) ? ((u32)clk_cfg & 0x1fu) : 0;
	u32 c;

	/* Program EXACTLY what the vendor dw_mci_proxima_init does, no more:
	 *   CFG (0x10808c): clear the low-5 phase bits, OR in the phase, PRESERVE high bits.
	 *   SEL (0x108088): the bus_hz tap (100 MHz -> 0x80).
	 * CRITICAL: do NOT write 0x108000 or 0x108038. U-Boot and the vendor leave 0x108000
	 * at 0; force-writing it corrupts the SDMMC clock so the card receives commands but
	 * never frames a valid response (CMD8 -> response-error/timeout). Leaving it 0 is
	 * what makes the card enumerate.
	 */
	c = readl(priv->clk_cfg);
	writel((c & ~0x1fu) | phase, priv->clk_cfg);
	dsb(st);
	writel(sel, priv->clk_sel);
	dsb(st);
	udelay(200);

	if (bus_hz > 0)
		host->bus_hz = bus_hz;

	dev_info(host->dev, "SDMMC clock: bus_hz=%u SEL=0x%02x CFG-phase=0x%02x (0x108000 untouched)\n",
		 host->bus_hz, sel, phase);
	return 0;
}

/* TODO(HS/UHS): per-speed CM/DLL phase tuning (dw_mci_artosyn_execute_tuning +
 * set_phase) and dw_mci_proxima_switch_voltage (the vol-domain GPIO). Not required
 * to enumerate at default speed; add once the basic clock path is validated.
 */

static const struct dw_mci_drv_data ar_proxima_drv_data = {
	.init		= ar_dwmmc_init,
	.parse_dt	= ar_dwmmc_parse_dt,
};

static const struct of_device_id ar_dwmmc_match[] = {
	{ .compatible = "artosyn,proxima-dw-mshc", .data = &ar_proxima_drv_data },
	{}
};
MODULE_DEVICE_TABLE(of, ar_dwmmc_match);

static int ar_dwmmc_probe(struct platform_device *pdev)
{
	const struct dw_mci_drv_data *drv_data;

	drv_data = of_device_get_match_data(&pdev->dev);
	if (!drv_data)
		return -EINVAL;
	return dw_mci_pltfm_register(pdev, drv_data);
}

static struct platform_driver ar_dwmmc_driver = {
	.probe		= ar_dwmmc_probe,
	.remove		= dw_mci_pltfm_remove,
	.driver = {
		.name		= "dwmmc_artosyn",
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table	= ar_dwmmc_match,
		.pm		= &dw_mci_pltfm_pmops,
	},
};
module_platform_driver(ar_dwmmc_driver);

MODULE_DESCRIPTION("Artosyn Proxima-9311 DesignWare MMC glue");
MODULE_LICENSE("GPL");
