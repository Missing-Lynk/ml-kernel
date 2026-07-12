// SPDX-License-Identifier: GPL-2.0
/*
 * clk-ar9311-cgu.c - CCF provider for the AR9311 (Proxima) clock generation unit.
 *
 * The tree is registered read-only (recalc/get_parent/is_enabled only, every
 * clock CLK_IGNORE_UNUSED, no enable/disable ops wired anywhere), with exactly
 * two settable clocks: the pixel PLL (slew-limited, +/-5 percent clamped
 * fractional-word set_rate, hardware-validated) and cgu_cpu_clk (the cpufreq
 * path, vendor bank-flip sequence, see its section below). The register map
 * was extracted from libmpp_service.so and live-validated on hardware.
 *
 * Tree shape (all names are the vendor cgu_* and source names so clk_summary
 * diffs against vendor captures and the table doc):
 *
 *   osc24m (DT phandle) -> cgu_oscin_clk
 *   fix_pll_clk{50..2000}, adc_pll_clk{20..600}   fixed PLL taps (rates by name)
 *   pixel_pll -> pixel_pll_clk, pixel_pll_clk1..4 the settable pixel PLL + taps
 *   sd and emmc tap sources, audio sources       placeholders, rate 0 (variable,
 *                                                 programmed via the MMC/audio regs)
 *   49 leaves: 3-bit parent mux + gate in one register half (see table doc):
 *     low half  = sel bits[10:8],  gate bit12
 *     high half = sel bits[26:24], gate bit28
 *     dual-bank (axi/sys/npu/cpu) = both halves, bit31 selects the active bank;
 *     cgu_cpu_clk adds bit30 (0 = 24 MHz bypass) and has no gate.
 *   Leaves have NO dividers: rate = selected parent's rate (pre-divided taps).
 *
 * The DC pixel leaf cgu_dvp_sub_1_2x_pix_clk (id 0x29) is the one leaf with a
 * consumer (artosyn_vo's pclk, provider index 0). It is registered as a mux
 * composite over its real parent list with CLK_SET_RATE_PARENT, so
 * clk_set_rate(pclk) propagates through the selected pixel_pll_clkN tap into
 * the PLL word. The bits[10:8] field at 0x0a104000+0x4c is the leaf's parent
 * mux (live sel 3 = pixel_pll_clk3 = PLL/4).
 *
 * Pixel PLL rate model (hardware-validated by vblank measurement): the +0x410
 * feedback word is a period word, rate = ref * KNUM / word; KNUM calibrated so
 * the leaf rate is the true pixel clock through the /4 tap (stock word
 * 0x0592eab1 -> 148.5005 MHz).
 * pixel_pll_clkN taps are modelled /1 /2 /4 /8 (only the /4 tap is validated;
 * the 2^(N-1) vs (N+1) divisor question for the other taps is open and
 * irrelevant until something consumes them).
 *
 * Safety: the banks are mapped non-exclusively (devm_ioremap) because ADC and
 * protemp own parts of 0x0a108000. clk_disable_unused can touch nothing: no
 * plain leaf has enable/disable ops, and everything carries CLK_IGNORE_UNUSED.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

/* reg index 0: bank 0x0a100000 (cgu_cpu_clk +0x08, cgu_cs_dbg_clk +0x10) */
/* reg index 1: leaf bank 0x0a104000 (+0x00..+0x5c, +0x200, +0x204) */
#define AR_PIX_CRG_REG		0x4c
#define AR_PIX_MUX_SHIFT	8
#define AR_PIX_MUX_WIDTH	3
#define AR_PIX_GATE_BIT		12
/* reg index 2: PLL bank 0x0a108000 */
#define AR_PLL_POWER		0x00
#define AR_PLL_RANGE		0x98
#define AR_PLL_FBWORD		0x410

/* Calibrated VCO-model constant: rate = parent * AR_PLL_KNUM / word (see header). */
#define AR_PLL_KNUM		2314489657ULL
/* Steering band around the boot nominal, in tenths of a percent. */
#define AR_PLL_BAND_PERMILLE	50
/*
 * Per-write slew limit, hardware-measured via the DSI INT_ST1 bit7 latch: single
 * word steps >= ~0.2 percent fire a transient DPI-FIFO error in either direction
 * (the panel sometimes visibly drops sync on it); steps <= 0.05 percent are
 * transient-free. set_rate therefore ramps to the target in steps of 1/2048th
 * of the current word (~0.049 percent), one per frame interval, keeping the
 * loop pure integer.
 */
#define AR_PLL_SLEW_SHIFT	11

static DEFINE_SPINLOCK(ar9311_cgu_lock);

/* ---------------- pixel PLL (the only settable element, unchanged) --------- */

struct ar9311_pixel_pll {
	struct clk_hw hw;
	void __iomem *base;		/* PLL bank */
	unsigned long boot_rate;	/* model rate from the boot-time word */
};

#define to_ar9311_pixel_pll(_hw) container_of(_hw, struct ar9311_pixel_pll, hw)

static unsigned long ar9311_pixel_pll_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct ar9311_pixel_pll *pll = to_ar9311_pixel_pll(hw);
	u32 word = readl(pll->base + AR_PLL_FBWORD);

	if (!word)
		return 0;

	return (unsigned long)div64_u64((u64)parent_rate * AR_PLL_KNUM, word);
}

static unsigned long ar9311_pixel_pll_clamp(struct ar9311_pixel_pll *pll,
					    unsigned long rate)
{
	unsigned long lo = pll->boot_rate / 1000 * (1000 - AR_PLL_BAND_PERMILLE);
	unsigned long hi = pll->boot_rate / 1000 * (1000 + AR_PLL_BAND_PERMILLE);

	return clamp(rate, lo, hi);
}

static int ar9311_pixel_pll_determine_rate(struct clk_hw *hw,
					   struct clk_rate_request *req)
{
	struct ar9311_pixel_pll *pll = to_ar9311_pixel_pll(hw);

	req->rate = ar9311_pixel_pll_clamp(pll, req->rate);
	return 0;
}

static int ar9311_pixel_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long parent_rate)
{
	struct ar9311_pixel_pll *pll = to_ar9311_pixel_pll(hw);
	unsigned long flags;
	u32 word, cur;

	if (!parent_rate)
		return -EINVAL;

	rate = ar9311_pixel_pll_clamp(pll, rate);
	/* Period word: bigger word = slower clock. */
	word = (u32)div64_u64((u64)parent_rate * AR_PLL_KNUM, rate);

	/*
	 * Ramp to the target in slew-limited steps (see AR_PLL_SLEW_SHIFT). A
	 * plain word write suffices per step, no relock kick: the fractional
	 * word takes effect on the fly (hardware-validated; the vendor pacing
	 * applier does the same every few vblanks). Pacing-loop calls move well
	 * under one step and take the single-write path with no sleep.
	 * CCF serializes set_rate and this driver's spinlocked write is the
	 * only writer of the word, so one read up front suffices; each step
	 * then moves cur at least 1 toward word, bounding the loop.
	 */
	cur = readl(pll->base + AR_PLL_FBWORD);
	while (cur != word) {
		u32 max_step = max(cur >> AR_PLL_SLEW_SHIFT, 1U);

		if (cur < word)
			cur = min(word, cur + max_step);
		else
			cur = max(word, cur - max_step);

		spin_lock_irqsave(&ar9311_cgu_lock, flags);
		writel(cur, pll->base + AR_PLL_FBWORD);
		spin_unlock_irqrestore(&ar9311_cgu_lock, flags);

		if (cur == word)
			break;

		/* one frame-ish settle between steps (measured safe at 1 s;
		 * the vendor steps once per vblank, ~16 ms)
		 */
		usleep_range(16000, 18000);
	}

	return 0;
}

static const struct clk_ops ar9311_pixel_pll_ops = {
	.recalc_rate	= ar9311_pixel_pll_recalc_rate,
	.determine_rate	= ar9311_pixel_pll_determine_rate,
	.set_rate	= ar9311_pixel_pll_set_rate,
};

/*
 * Debugfs steering knob, /sys/kernel/debug/ar9311_pixclk_rate: read = current leaf
 * rate, write = clk_set_rate on the leaf (propagates to the PLL through the
 * read-only mux, clamped to the +/-5 percent band). Mainline's writable
 * clk debugfs needs a compile-time define, so validation and userspace pacing
 * prototypes get their own knob here instead.
 */
static struct clk *ar9311_pixclk_leaf;

static int ar9311_pixclk_rate_get(void *data, u64 *val)
{
	*val = clk_get_rate(ar9311_pixclk_leaf);
	return 0;
}

static int ar9311_pixclk_rate_set(void *data, u64 val)
{
	return clk_set_rate(ar9311_pixclk_leaf, (unsigned long)val);
}

DEFINE_DEBUGFS_ATTRIBUTE(ar9311_pixclk_rate_fops, ar9311_pixclk_rate_get,
			 ar9311_pixclk_rate_set, "%llu\n");

/* ---------------- fixed sources ------------------------------------------- */

static const struct {
	const char *name;
	unsigned long rate;
} ar9311_cgu_sources[] = {
	/* fixed system-PLL taps (rates by name; 333/666 use the vendor kernel's
	 * exact thirds)
	 */
	{ "fix_pll_clk50",    50000000 },	/* unnamed id 0x0e in libmpp */
	{ "fix_pll_clk100",  100000000 },
	{ "fix_pll_clk125",  125000000 },
	{ "fix_pll_clk250",  250000000 },
	{ "fix_pll_clk333",  333333333 },
	{ "fix_pll_clk400",  400000000 },
	{ "fix_pll_clk500",  500000000 },
	{ "fix_pll_clk600",  600000000 },
	{ "fix_pll_clk666",  666666666 },
	{ "fix_pll_clk800",  800000000 },
	{ "fix_pll_clk1000", 1000000000 },
	{ "fix_pll_clk2000", 2000000000 },

	/* ADC-PLL taps */
	{ "adc_pll_clk20",    20000000 },
	{ "adc_pll_clk25",    25000000 },
	{ "adc_pll_clk60",    60000000 },
	{ "adc_pll_clk100",  100000000 },
	{ "adc_pll_clk150",  150000000 },
	{ "adc_pll_clk300",  300000000 },
	{ "adc_pll_clk400",  400000000 },
	{ "adc_pll_clk600",  600000000 },

	/* variable sources whose rate lives in other register blocks (MMC clock
	 * taps, audio PLL); rate 0 = honestly unknown until those are modelled
	 */
	{ "sd0_fix_clk", 0 }, { "sd0_sample_clk", 0 }, { "sd0_drv_clk", 0 },
	{ "sd1_fix_clk", 0 }, { "sd1_sample_clk", 0 }, { "sd1_drv_clk", 0 },
	{ "emmc_fix_clk", 0 }, { "emmc_sample_clk", 0 }, { "emmc_drv_clk", 0 },
	{ "clk_audio_dig", 0 }, { "audio_pll_clk", 0 },

	/* D-PHY PLL / 2 (891 Mbps lane / 2, INFERRED; lives in the DSI window) */
	{ "mipi_tx_pll_div2", 445500000 },
};

/* ---------------- the 49 leaves ------------------------------------------- */

enum ar9311_leaf_half {
	AR_HALF_LO,		/* sel [10:8],  gate bit12 */
	AR_HALF_HI,		/* sel [26:24], gate bit28 */
	AR_HALF_DUAL,		/* both halves, bit31 = active bank */
	AR_HALF_BIT15,		/* +0x204 style: single parent, gate-ish bit15 */
	AR_HALF_BIT31,		/* +0x204 style: single parent, gate-ish bit31 */
};

struct ar9311_leaf_desc {
	const char *name;
	u16 off;		/* offset within its bank */
	u8 bank;		/* reg index the offset applies to (0 or 1) */
	u8 half;		/* enum ar9311_leaf_half */
	const char *parents[8];	/* by sel value */
};

/* "cgu_invalid" = a sel value the vendor parent table marks invalid (0x28
 * filler). Never registered; a leaf actually selecting it shows up as an
 * orphan in clk_summary, which is exactly the alarm we want.
 */
#define INV "cgu_invalid"

/* Vendor parent lists, verbatim from ar9311_clock_list +36 (see the table doc). */
#define PAR_GENERIC_OSC \
	{ "cgu_oscin_clk", "fix_pll_clk500", "fix_pll_clk400", "fix_pll_clk333", \
	  "adc_pll_clk600", "adc_pll_clk400", "pixel_pll_clk", "audio_pll_clk" }
#define PAR_GENERIC_666 \
	{ "fix_pll_clk666", "fix_pll_clk500", "fix_pll_clk400", "fix_pll_clk333", \
	  "adc_pll_clk600", "adc_pll_clk400", "pixel_pll_clk", "audio_pll_clk" }
#define PAR_SENSOR \
	{ "cgu_oscin_clk", "pixel_pll_clk1", "pixel_pll_clk2", "pixel_pll_clk3", \
	  "pixel_pll_clk4", "pixel_pll_clk", INV, INV }
#define PAR_DVP \
	{ "adc_pll_clk600", "pixel_pll_clk1", "pixel_pll_clk2", "pixel_pll_clk3", \
	  "pixel_pll_clk4", "fix_pll_clk1000", "fix_pll_clk666", "mipi_tx_pll_div2" }
#define PAR_NPU \
	{ "cgu_oscin_clk", "fix_pll_clk800", "fix_pll_clk666", "fix_pll_clk500", \
	  "fix_pll_clk400", "adc_pll_clk600", "adc_pll_clk400", "pixel_pll_clk" }
#define PAR_CPU \
	{ "cgu_oscin_clk", "fix_pll_clk1000", "fix_pll_clk800", "fix_pll_clk666", \
	  "adc_pll_clk600", "adc_pll_clk400", "pixel_pll_clk", INV }
#define PAR_ONE(p) { p, INV, INV, INV, INV, INV, INV, INV }

static const struct ar9311_leaf_desc ar9311_leaves[] = {
	{ "cgu_axi_clk",     0x00,  1, AR_HALF_DUAL, PAR_GENERIC_OSC },
	{ "cgu_sys_clk",     0x04,  1, AR_HALF_DUAL,
	  { "cgu_oscin_clk", "adc_pll_clk300", "fix_pll_clk400", "fix_pll_clk333",
	    "adc_pll_clk600", "adc_pll_clk400", "pixel_pll_clk", "audio_pll_clk" } },
	{ "cgu_isp_clk",     0x0c,  1, AR_HALF_LO,   PAR_GENERIC_666 },
	{ "cgu_isp_hdr_clk", 0x0c,  1, AR_HALF_HI,   PAR_GENERIC_666 },
	{ "cgu_vif_axi_clk", 0x10,  1, AR_HALF_LO,   PAR_GENERIC_666 },
	{ "cgu_dla_clk",     0x10,  1, AR_HALF_HI,
	  { "fix_pll_clk800", "fix_pll_clk666", "fix_pll_clk500", "fix_pll_clk400",
	    "adc_pll_clk600", "adc_pll_clk400", "pixel_pll_clk", "mipi_tx_pll_div2" } },
	{ "cgu_rsz_clk",     0x14,  1, AR_HALF_LO,   PAR_GENERIC_OSC },
	{ "cgu_de_clk",      0x14,  1, AR_HALF_HI,   PAR_GENERIC_OSC },
	{ "cgu_venc_clk",    0x18,  1, AR_HALF_LO,   PAR_GENERIC_OSC },
	{ "cgu_jpeg_clk",    0x18,  1, AR_HALF_HI,   PAR_GENERIC_OSC },
	{ "cgu_mipi_csi_0_clk", 0x1c, 1, AR_HALF_LO, PAR_GENERIC_OSC },
	{ "cgu_mipi_csi_1_clk", 0x1c, 1, AR_HALF_HI, PAR_GENERIC_OSC },
	{ "cgu_mipi_pcs_clk", 0x20, 1, AR_HALF_LO,   PAR_GENERIC_OSC },
	{ "cgu_sd0_fix_clk", 0x20,  1, AR_HALF_HI,   PAR_ONE("sd0_fix_clk") },
	{ "cgu_sd0_sample_clk", 0x24, 1, AR_HALF_LO, PAR_ONE("sd0_sample_clk") },
	{ "cgu_sd0_drv_clk", 0x24,  1, AR_HALF_HI,   PAR_ONE("sd0_drv_clk") },
	{ "cgu_sd1_fix_clk", 0x28,  1, AR_HALF_LO,   PAR_ONE("sd1_fix_clk") },
	{ "cgu_sd1_sample_clk", 0x28, 1, AR_HALF_HI, PAR_ONE("sd1_sample_clk") },
	{ "cgu_sd1_drv_clk", 0x2c,  1, AR_HALF_LO,   PAR_ONE("sd1_drv_clk") },
	{ "cgu_emmc_fix_clk", 0x2c, 1, AR_HALF_HI,   PAR_ONE("emmc_fix_clk") },
	{ "cgu_emmc_sample_clk", 0x30, 1, AR_HALF_LO, PAR_ONE("emmc_sample_clk") },
	{ "cgu_emmc_drv_clk", 0x30, 1, AR_HALF_HI,   PAR_ONE("emmc_drv_clk") },
	{ "cgu_qspi_clk",    0x34,  1, AR_HALF_LO,   PAR_GENERIC_666 },
	{ "cgu_i2s_mclk",    0x34,  1, AR_HALF_HI,
	  { "cgu_oscin_clk", "audio_pll_clk", INV, INV, INV, INV, INV, INV } },
	{ "cgu_i2s_mst0_sclk", 0x204, 1, AR_HALF_BIT15, PAR_ONE("cgu_i2s_mclk") },
	{ "cgu_i2s_mst1_sclk", 0x204, 1, AR_HALF_BIT31, PAR_ONE("cgu_i2s_mclk") },
	{ "cgu_usb_phy0_clk", 0x38, 1, AR_HALF_LO,   PAR_ONE("cgu_oscin_clk") },
	{ "cgu_audio_300m",  0x38,  1, AR_HALF_HI,   PAR_ONE("clk_audio_dig") },
	{ "cgu_gmac_core_clk", 0x3c, 1, AR_HALF_LO,
	  { "cgu_oscin_clk", "fix_pll_clk500", "fix_pll_clk1000", INV, INV, INV, INV, INV } },
	{ "cgu_gmac_phy_clk", 0x3c, 1, AR_HALF_HI,
	  { "cgu_oscin_clk", "fix_pll_clk500", "fix_pll_clk1000", INV, INV, INV, INV, INV } },
	{ "cgu_hdecomp_clk", 0x40,  1, AR_HALF_LO,
	  { "cgu_oscin_clk", "fix_pll_clk666", "fix_pll_clk500", "fix_pll_clk400",
	    "adc_pll_clk600", "adc_pll_clk400", "pixel_pll_clk", "audio_pll_clk" } },
	{ "cgu_efuse_clk",   0x40,  1, AR_HALF_HI,
	  { "cgu_oscin_clk", "fix_pll_clk100", "fix_pll_clk400", "fix_pll_clk333",
	    "fix_pll_clk250", INV, INV, INV } },
	{ "cgu_sensor_mclk0", 0x44, 1, AR_HALF_LO,   PAR_SENSOR },
	{ "cgu_sensor_mclk1", 0x44, 1, AR_HALF_HI,   PAR_SENSOR },
	{ "cgu_sensor_mclk2", 0x48, 1, AR_HALF_LO,   PAR_SENSOR },
	{ "cgu_dvp_pattern_clk", 0x48, 1, AR_HALF_HI, PAR_DVP },

	/* id 0x29 cgu_dvp_sub_1_2x_pix_clk (the DC pixel leaf) is registered
	 * separately as the settable composite, see probe step 3.
	 */
	{ "cgu_nuc_clk",     0x4c,  1, AR_HALF_HI,
	  { "adc_pll_clk600", "pixel_pll_clk1", "pixel_pll_clk2", "pixel_pll_clk3",
	    "pixel_pll_clk4", "fix_pll_clk666", "fix_pll_clk500", "mipi_tx_pll_div2" } },
	{ "cgu_scgmac_ptp_clk", 0x50, 1, AR_HALF_LO,
	  { "cgu_oscin_clk", "fix_pll_clk1000", "fix_pll_clk500", "fix_pll_clk250",
	    INV, INV, INV, INV } },
	{ "cgu_scgmac_rgmiitx_clk", 0x50, 1, AR_HALF_HI,
	  { "cgu_oscin_clk", "fix_pll_clk1000", "fix_pll_clk500", "fix_pll_clk250",
	    "adc_pll_clk25", "adc_pll_clk20", INV, INV } },
	{ "cgu_scgmac_mdc_clk", 0x54, 1, AR_HALF_LO,
	  { "cgu_oscin_clk", "fix_pll_clk250", "fix_pll_clk125", "fix_pll_clk50",
	    "adc_pll_clk25", "adc_pll_clk20", INV, INV } },
	{ "cgu_npu_clk",     0x08,  1, AR_HALF_DUAL, PAR_NPU },
	{ "cgu_npu_aclk",    0x58,  1, AR_HALF_LO,   PAR_NPU },
	{ "cgu_ifc_clk",     0x58,  1, AR_HALF_HI,   PAR_GENERIC_OSC },
	{ "cgu_eis_clk",     0x5c,  1, AR_HALF_LO,   PAR_GENERIC_OSC },
	{ "cgu_scaler_clk",  0x5c,  1, AR_HALF_HI,   PAR_GENERIC_OSC },
	{ "cgu_gen_clk",     0x200, 1, AR_HALF_LO,
	  { "fix_pll_clk500", "fix_pll_clk100", "sd0_fix_clk", "adc_pll_clk600",
	    "adc_pll_clk25", "audio_pll_clk", "pixel_pll_clk", "pixel_pll_clk1" } },

	/* cgu_cpu_clk (0xa100000+0x08) is registered separately as the settable
	 * CPU clock, see probe step 4 and the ar9311_cpu_* ops.
	 */
	{ "cgu_cs_dbg_clk",  0x10,  0, AR_HALF_LO,   PAR_CPU },
};

struct ar9311_leaf {
	struct clk_hw hw;
	void __iomem *reg;
	u8 half;
};

#define to_ar9311_leaf(_hw) container_of(_hw, struct ar9311_leaf, hw)

static u8 ar9311_leaf_get_parent(struct clk_hw *hw)
{
	struct ar9311_leaf *leaf = to_ar9311_leaf(hw);
	u32 v = readl(leaf->reg);
	bool hi;

	switch (leaf->half) {
	case AR_HALF_DUAL:
		hi = v & BIT(31);
		break;

	case AR_HALF_HI:
		hi = true;
		break;

	case AR_HALF_LO:
		hi = false;
		break;

	default:		/* BIT15/BIT31: single parent */
		return 0;
	}

	/* The 3-bit parent select sits at bits[26:24] in the register's high
	 * half, bits[10:8] in the low half; hi picked the half this clock owns
	 * (or, for dual-bank clocks, the bank the hardware marked active). The
	 * raw field value doubles as the CCF parent index because the parents[]
	 * arrays are declared in sel order.
	 */
	return hi ? (v >> 24) & 0x7 : (v >> 8) & 0x7;
}

static int ar9311_leaf_is_enabled(struct clk_hw *hw)
{
	struct ar9311_leaf *leaf = to_ar9311_leaf(hw);
	u32 v = readl(leaf->reg);

	switch (leaf->half) {
	case AR_HALF_LO:
		return !!(v & BIT(12));

	case AR_HALF_HI:
		return !!(v & BIT(28));

	case AR_HALF_DUAL:
		return !!(v & ((v & BIT(31)) ? BIT(28) : BIT(12)));

	case AR_HALF_BIT15:
		return !!(v & BIT(15));

	case AR_HALF_BIT31:
	default:
		return !!(v & BIT(31));
	}
}

/* Read-only on purpose: no set_parent, no enable/disable, no rate ops (a leaf's
 * rate IS the selected parent's rate; NULL recalc_rate inherits it).
 */
static const struct clk_ops ar9311_leaf_ops = {
	.get_parent	= ar9311_leaf_get_parent,
	.is_enabled	= ar9311_leaf_is_enabled,
};

/* ---------------- the CPU clock (the cpufreq write path) ------------------- */

/*
 * cgu_cpu_clk, register 0x0a100000+0x08 with an SPL-programmed mirror at
 * +0x408 (kept in lockstep, matching the invariant both stock and open SPLs
 * leave: identical values in both).
 *
 * Layout (dual-bank composite, settled by the slot-A live read, see the table
 * doc "Slot-A live read"): bit30 = 0 means 24 MHz oscillator bypass; bit31
 * selects the active bank; bank 0 = gate bit12 + mux bits[10:8] + divider
 * bits[6:0]; bank 1 = gate bit28 + mux bits[26:24] + divider bits[22:16];
 * rate = parent / (div_field + 1). No gate from software's point of view
 * (the vendor disable handler for this clock is a no-op).
 *
 * set_rate reproduces the vendor kernel's glitch-free sequence verbatim
 * (artosyn_composite_set_rate, vmlinux 0xffffff80083262b0): write the IDLE
 * bank's mux+div with its gate bit already set, barrier, then a second write
 * that flips the active-bank/bypass bit. The core never sees an intermediate
 * state because the running bank's fields are preserved by the write mask
 * until the atomic flip.
 *
 * Only parents 0..5 participate in rate selection (24M/1000M/800M/666M/
 * adc600/adc400); index 6 is the variable pixel PLL, which the vendor never
 * uses for the CPU and we exclude for the same reason.
 */
#define AR_CPU_RATE_PARENTS	6
#define AR_CPU_MAX_DIV		128	/* 7-bit field, encoding div = field + 1 */

struct ar9311_cpu_clk {
	struct clk_hw hw;
	void __iomem *reg;
	void __iomem *mirror;
};

#define to_ar9311_cpu_clk(_hw) container_of(_hw, struct ar9311_cpu_clk, hw)

static u8 ar9311_cpu_get_parent(struct clk_hw *hw)
{
	u32 v = readl(to_ar9311_cpu_clk(hw)->reg);

	if (!(v & BIT(30)))
		return 0;	/* bypass = parent 0 = cgu_oscin_clk */

	return (v & BIT(31)) ? (v >> 24) & 0x7 : (v >> 8) & 0x7;
}

static unsigned long ar9311_cpu_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	u32 v = readl(to_ar9311_cpu_clk(hw)->reg);
	u32 div;

	if (!(v & BIT(30)))
		return parent_rate;	/* bypass: raw 24 MHz osc */

	div = ((v & BIT(31)) ? (v >> 16) & 0x7f : v & 0x7f) + 1;
	return parent_rate / div;
}

static int ar9311_cpu_determine_rate(struct clk_hw *hw,
				     struct clk_rate_request *req)
{
	unsigned long best_rate = 0, best_prate = 0;
	int best_idx = -1;
	int i;

	for (i = 0; i < AR_CPU_RATE_PARENTS; i++) {
		struct clk_hw *parent = clk_hw_get_parent_by_index(hw, i);
		unsigned long prate, rate;
		u32 div, d;

		if (!parent)
			continue;

		prate = clk_hw_get_rate(parent);
		if (!prate)
			continue;

		/* closest divider: floor and floor+1 bracket the target */
		div = req->rate ? prate / req->rate : AR_CPU_MAX_DIV;
		for (d = max(div, 1U); d <= min(div + 1, (u32)AR_CPU_MAX_DIV); d++) {
			rate = prate / d;
			if (abs((long)(rate - req->rate)) <
			    abs((long)(best_rate - req->rate))) {
				best_rate = rate;
				best_prate = prate;
				best_idx = i;
			}
		}
	}

	if (best_idx < 0)
		return -EINVAL;

	req->best_parent_hw = clk_hw_get_parent_by_index(hw, best_idx);
	req->best_parent_rate = best_prate;
	req->rate = best_rate;
	return 0;
}

/* The vendor's two-write flip on one register: program the idle bank (gate bit
 * included, active bank's fields preserved by the mask), barrier, flip.
 */
static void ar9311_cpu_program_one(void __iomem *reg, u32 mux, u32 div_m1)
{
	u32 v = readl(reg);
	u32 val;

	if (!(v & BIT(30))) {
		/* leaving bypass: program bank 0, then set bit30 (bank31=0) */
		val = (v & 0x7fffe000) | div_m1 | (mux << 8) | BIT(12);
		writel(val, reg);
		writel(val | BIT(30), reg);
	} else if (!(v & BIT(31))) {
		/* bank 0 running: program bank 1, then flip bit31 on */
		val = (v & 0xe000ffff) | (div_m1 << 16) | (mux << 24) | BIT(28);
		writel(val, reg);
		writel(val | BIT(31), reg);
	} else {
		/* bank 1 running: program bank 0, then flip bit31 off */
		val = (v & 0xffffe000) | div_m1 | (mux << 8) | BIT(12);
		writel(val, reg);
		writel(val & ~BIT(31), reg);
	}

	readl(reg);	/* post the flip before returning */
}

static int ar9311_cpu_program(struct clk_hw *hw, u32 mux, u32 div_m1)
{
	struct ar9311_cpu_clk *cpu = to_ar9311_cpu_clk(hw);
	unsigned long flags;

	spin_lock_irqsave(&ar9311_cgu_lock, flags);
	ar9311_cpu_program_one(cpu->reg, mux, div_m1);
	ar9311_cpu_program_one(cpu->mirror, mux, div_m1);
	spin_unlock_irqrestore(&ar9311_cgu_lock, flags);

	return 0;
}

static u32 ar9311_cpu_cur_div_m1(struct clk_hw *hw)
{
	u32 v = readl(to_ar9311_cpu_clk(hw)->reg);

	if (!(v & BIT(30)))
		return 0;

	return (v & BIT(31)) ? (v >> 16) & 0x7f : v & 0x7f;
}

static int ar9311_cpu_set_parent(struct clk_hw *hw, u8 index)
{
	return ar9311_cpu_program(hw, index, ar9311_cpu_cur_div_m1(hw));
}

static int ar9311_cpu_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	u32 div;

	if (!rate || !parent_rate)
		return -EINVAL;

	div = clamp(DIV_ROUND_CLOSEST(parent_rate, rate), 1UL, (unsigned long)AR_CPU_MAX_DIV);
	return ar9311_cpu_program(hw, ar9311_cpu_get_parent(hw), div - 1);
}

static int ar9311_cpu_set_rate_and_parent(struct clk_hw *hw, unsigned long rate,
					  unsigned long parent_rate, u8 index)
{
	u32 div;

	if (!rate || !parent_rate)
		return -EINVAL;

	div = clamp(DIV_ROUND_CLOSEST(parent_rate, rate), 1UL, (unsigned long)AR_CPU_MAX_DIV);
	return ar9311_cpu_program(hw, index, div - 1);
}

static const struct clk_ops ar9311_cpu_ops = {
	.get_parent		= ar9311_cpu_get_parent,
	.set_parent		= ar9311_cpu_set_parent,
	.recalc_rate		= ar9311_cpu_recalc_rate,
	.determine_rate		= ar9311_cpu_determine_rate,
	.set_rate		= ar9311_cpu_set_rate,
	.set_rate_and_parent	= ar9311_cpu_set_rate_and_parent,
};

static const char * const ar9311_cpu_parents[8] = PAR_CPU;

/* Pixel-leaf mux parents = the id 0x29 vendor parent list (sel is the raw field). */
static const char * const ar9311_pixclk_parents[8] = PAR_DVP;

static int ar9311_cgu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *cpu_base, *crg_base, *pll_base;
	struct ar9311_pixel_pll *pll;
	struct ar9311_cpu_clk *cpu;
	struct clk_hw_onecell_data *data;
	struct clk_hw *hw, *pix_hw;
	struct clk_mux *mux;
	struct clk_gate *gate;
	struct clk_init_data init = {};
	const char *osc_name;
	struct resource *res;
	void __iomem *bases[3];
	u32 crg;
	int i, ret;

	/* Non-exclusive maps: ADC/protemp own parts of 0x0a108000. */
	for (i = 0; i < 3; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			return -EINVAL;

		bases[i] = devm_ioremap(dev, res->start, resource_size(res));
		if (!bases[i])
			return -ENOMEM;
	}
	cpu_base = bases[0];
	crg_base = bases[1];
	pll_base = bases[2];

	osc_name = of_clk_get_parent_name(dev->of_node, 0);
	if (!osc_name)
		return -EINVAL;

	/* 1. Fixed sources. cgu_oscin_clk tracks the DT osc so the tree has one
	 * root; the PLL taps are fixed rates per the table doc.
	 */
	hw = devm_clk_hw_register_fixed_factor(dev, "cgu_oscin_clk", osc_name, 0, 1, 1);
	if (IS_ERR(hw))
		return PTR_ERR(hw);

	for (i = 0; i < (int)ARRAY_SIZE(ar9311_cgu_sources); i++) {
		hw = devm_clk_hw_register_fixed_rate(dev, ar9311_cgu_sources[i].name,
						     NULL, 0,
						     ar9311_cgu_sources[i].rate);
		if (IS_ERR(hw))
			return PTR_ERR(hw);
	}

	/* 2. The pixel PLL (the only settable element) + its taps. "pixel_pll_clk"
	 * (no suffix) is the direct PLL output named in the vendor parent lists.
	 */
	pll = devm_kzalloc(dev, sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return -ENOMEM;

	pll->base = pll_base;

	init.name = "pixel_pll";
	init.ops = &ar9311_pixel_pll_ops;
	init.parent_names = &osc_name;
	init.num_parents = 1;
	pll->hw.init = &init;
	ret = devm_clk_hw_register(dev, &pll->hw);
	if (ret)
		return ret;

	pll->boot_rate = clk_hw_get_rate(&pll->hw);

	hw = devm_clk_hw_register_fixed_factor(dev, "pixel_pll_clk", "pixel_pll",
					       CLK_SET_RATE_PARENT, 1, 1);
	if (IS_ERR(hw))
		return PTR_ERR(hw);

	for (i = 0; i < 4; i++) {
		char name[16];

		snprintf(name, sizeof(name), "pixel_pll_clk%d", i + 1);
		hw = devm_clk_hw_register_fixed_factor(dev, name, "pixel_pll",
						       CLK_SET_RATE_PARENT,
						       1, 1 << i);

		if (IS_ERR(hw))
			return PTR_ERR(hw);
	}

	/* 3. The DC pixel leaf (id 0x29), the one leaf with a consumer. Mux is
	 * READ-ONLY; the gate is operable (artosyn_vo enables it); rate requests
	 * propagate through the selected tap into the PLL: with no rate hw in
	 * the composite and CLK_SET_RATE_PARENT set, CCF forwards round/set_rate
	 * to the current parent (leaf rate == tap rate == PLL/4 on this hardware).
	 */
	mux = devm_kzalloc(dev, sizeof(*mux), GFP_KERNEL);
	gate = devm_kzalloc(dev, sizeof(*gate), GFP_KERNEL);
	if (!mux || !gate)
		return -ENOMEM;

	mux->reg = crg_base + AR_PIX_CRG_REG;
	mux->shift = AR_PIX_MUX_SHIFT;
	mux->mask = BIT(AR_PIX_MUX_WIDTH) - 1;
	mux->lock = &ar9311_cgu_lock;

	gate->reg = crg_base + AR_PIX_CRG_REG;
	gate->bit_idx = AR_PIX_GATE_BIT;
	gate->lock = &ar9311_cgu_lock;

	/* CLK_IGNORE_UNUSED: artosyn_vo.ko (the only consumer) loads after
	 * clk_disable_unused would gate the leaf off; keep the boot state.
	 * Non-devm on purpose: no devm string-parent composite helper exists,
	 * and a builtin_platform_driver never unbinds.
	 */
	pix_hw = clk_hw_register_composite(dev, "cgu_dvp_sub_1_2x_pix_clk",
					   ar9311_pixclk_parents,
					   ARRAY_SIZE(ar9311_pixclk_parents),
					   &mux->hw, &clk_mux_ro_ops,
					   NULL, NULL,
					   &gate->hw, &clk_gate_ops,
					   CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED);

	if (IS_ERR(pix_hw))
		return PTR_ERR(pix_hw);

	/* 4. The CPU clock, the cpufreq write path (register + SPL mirror). */
	cpu = devm_kzalloc(dev, sizeof(*cpu), GFP_KERNEL);
	if (!cpu)
		return -ENOMEM;

	cpu->reg = cpu_base + 0x08;
	cpu->mirror = cpu_base + 0x408;

	memset(&init, 0, sizeof(init));
	init.name = "cgu_cpu_clk";
	init.ops = &ar9311_cpu_ops;
	init.parent_names = ar9311_cpu_parents;
	init.num_parents = ARRAY_SIZE(ar9311_cpu_parents);
	/* GET_RATE_NOCACHE like the vendor (the SPL owns the boot value);
	 * IGNORE_UNUSED as everywhere in this driver.
	 */
	init.flags = CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED;
	cpu->hw.init = &init;

	ret = devm_clk_hw_register(dev, &cpu->hw);
	if (ret)
		return ret;

	/* 5. The remaining read-only leaves. */
	data = devm_kzalloc(dev,
			    struct_size(data, hws, ARRAY_SIZE(ar9311_leaves) + 3),
			    GFP_KERNEL);

	if (!data)
		return -ENOMEM;

	data->num = ARRAY_SIZE(ar9311_leaves) + 3;
	data->hws[0] = pix_hw;		/* <&cgu 0> = the pixel leaf, VO's pclk */
	data->hws[1] = &pll->hw;	/* <&cgu 1> = pixel_pll (debug) */
	data->hws[2] = &cpu->hw;	/* <&cgu 2> = cgu_cpu_clk (the CPU nodes) */

	for (i = 0; i < (int)ARRAY_SIZE(ar9311_leaves); i++) {
		const struct ar9311_leaf_desc *d = &ar9311_leaves[i];
		struct ar9311_leaf *leaf;
		struct clk_init_data li = {};

		leaf = devm_kzalloc(dev, sizeof(*leaf), GFP_KERNEL);
		if (!leaf)
			return -ENOMEM;

		leaf->reg = bases[d->bank] + d->off;
		leaf->half = d->half;

		li.name = d->name;
		li.ops = &ar9311_leaf_ops;
		li.parent_names = d->parents;
		li.num_parents = ARRAY_SIZE(d->parents);
		li.flags = CLK_IGNORE_UNUSED;
		leaf->hw.init = &li;

		ret = devm_clk_hw_register(dev, &leaf->hw);
		if (ret)
			return ret;

		data->hws[3 + i] = &leaf->hw;
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
	if (ret)
		return ret;

	/* No cpufreq-dt platform device registered here: cpufreq-dt-platdev
	 * auto-registers it on every DT machine not on its blocklist (verified
	 * on 6.18: a manual registration collides with -EEXIST).
	 */

	ar9311_pixclk_leaf = clk_hw_get_clk(pix_hw, "pixclk-debugfs");
	if (!IS_ERR_OR_NULL(ar9311_pixclk_leaf))
		debugfs_create_file("ar9311_pixclk_rate", 0644, NULL, NULL,
				    &ar9311_pixclk_rate_fops);

	crg = readl(crg_base + AR_PIX_CRG_REG);
	dev_info(dev,
		 "CGU up: %zu leaves read-only + settable cpu clk (%lu Hz, reg=0x%08x mirror=0x%08x); pixel fbword=0x%08x (model %lu Hz), sel=%lu gate=%s\n",
		 ARRAY_SIZE(ar9311_leaves) + 1, clk_hw_get_rate(&cpu->hw),
		 readl(cpu->reg), readl(cpu->mirror),
		 readl(pll_base + AR_PLL_FBWORD), pll->boot_rate,
		 (crg >> AR_PIX_MUX_SHIFT) & (BIT(AR_PIX_MUX_WIDTH) - 1UL),
		 (crg & BIT(AR_PIX_GATE_BIT)) ? "on" : "off");

	return 0;
}

static const struct of_device_id ar9311_cgu_of_match[] = {
	{ .compatible = "artosyn,ar9311-cgu" },
	{ }
};

static struct platform_driver ar9311_cgu_driver = {
	.probe = ar9311_cgu_probe,
	.driver = {
		.name = "clk-ar9311-cgu",
		.of_match_table = ar9311_cgu_of_match,
	},
};
builtin_platform_driver(ar9311_cgu_driver);
