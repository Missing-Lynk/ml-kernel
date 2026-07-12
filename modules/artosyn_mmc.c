// SPDX-License-Identifier: GPL-2.0
/*
 * artosyn_mmc.c - standalone Artosyn (Proxima-9311) DesignWare MSHC host driver.
 *
 * A minimal diagnostic/self-test MMC controller exercise, not the production
 * driver (dw_mci-artosyn.c is): controller init + clock + polled
 * command/response, plus a load-time self-test that sends CMD0 then CMD8
 * (arg 0x1aa, R7) and prints the response (a clean 0x000001aa passes).
 *
 * Point it at a controller with the `base` param: 0x01c00000 = SD (mmc1, easiest to test
 * with a card inserted), 0x01b00000 = AR8030 SDIO (mmc0). Clock regs are chosen to match.
 */
#define pr_fmt(fmt) "artosyn_mmc: " fmt

#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/iopoll.h>

/* DesignWare MSHC register offsets (from the controller base). */
#define REG_CTRL	0x00
#define  CTRL_RESET	BIT(0)
#define  CTRL_FIFO_RESET BIT(1)
#define  CTRL_DMA_RESET	BIT(2)
#define  CTRL_INT_ENABLE BIT(4)
#define REG_PWREN	0x04
#define REG_CLKDIV	0x08
#define REG_CLKSRC	0x0c
#define REG_CLKENA	0x10
#define  CLKENA_CCLK	BIT(0)
#define  CLKENA_LP	BIT(16)
#define REG_TMOUT	0x14
#define REG_CTYPE	0x18
#define REG_BLKSIZ	0x1c
#define REG_BYTCNT	0x20
#define REG_INTMASK	0x24
#define REG_CMDARG	0x28
#define REG_CMD		0x2c
#define  CMD_START	BIT(31)
#define  CMD_USE_HOLD	BIT(29)
#define  CMD_UPD_CLK	BIT(21)
#define  CMD_INIT	BIT(15)
#define  CMD_PRV_DAT_WAIT BIT(13)
#define  CMD_DAT_WR	BIT(10)
#define  CMD_DAT_EXP	BIT(9)
#define  CMD_RESP_CRC	BIT(8)
#define  CMD_RESP_LONG	BIT(7)
#define  CMD_RESP_EXP	BIT(6)
#define REG_RESP0	0x30
#define REG_RINTSTS	0x44
#define  INT_RE		BIT(1)	/* response error */
#define  INT_CMD_DONE	BIT(2)
#define  INT_RCRC	BIT(6)	/* response CRC error */
#define  INT_RTO	BIT(8)	/* response timeout */
#define  INT_HLE	BIT(12)	/* hardware locked error */
#define REG_STATUS	0x48
#define  STATUS_DATA_BUSY BIT(9)
#define REG_FIFOTH	0x4c
#define REG_CDETECT	0x50
#define REG_UHS		0x74
#define REG_RST_N	0x78	/* card hardware-reset (active-low, per-slot bit) - SDIO chips
				 * are held in/out of reset by this; SD cards ignore it.
				 */

/* Artosyn SDMMC clock block (separate from the controller). */
#define CLK_EN_ADDR	0x0A108000UL	/* master-enable = 0x81 */
#define CLK_38_ADDR	0x0A108038UL	/* = 0x70 */
#define CLK0_SEL_ADDR	0x0A108088UL	/* mmc0 SEL|phase */
#define CLK0_CFG_ADDR	0x0A10808CUL	/* mmc0 CFG */
#define CLK1_SEL_ADDR	0x0A1080C0UL	/* mmc1 SEL|phase */
#define CLK1_CFG_ADDR	0x0A1080C4UL	/* mmc1 CFG */

static unsigned long base = 0x01c00000;	/* default: SD controller (mmc1) */
module_param(base, ulong, 0444);
static int src_hz = 100000000;		/* CIU input clock fed by the 0x108088 tap */
module_param(src_hz, int, 0444);
static int clk_sel = 0x80;		/* SEL|phase value for the clock block */
module_param(clk_sel, int, 0444);
static int clk_cfg = 0x02;
module_param(clk_cfg, int, 0444);
static int use_hold = 1;	/* SDMMC_CMD_USE_HOLD_REG: shifts cmd/resp timing a phase */
module_param(use_hold, int, 0444);
static int en_shift = -1;	/* controller ENABLE_SHIFT (+0x108): sample-point shift */
module_param(en_shift, int, 0444);
static int uhs_reg = -1;	/* controller UHS_REG (+0x74) */
module_param(uhs_reg, int, 0444);
static int clken = -1;		/* 0x108000 master-enable: >=0 write it, <0 = leave U-Boot's value */
module_param(clken, int, 0444);
static int skip_cfg;		/* 1 = don't touch CFG (0x8c) either - leave U-Boot's */
module_param(skip_cfg, int, 0444);
static int skip_sel;		/* 1 = don't touch SEL (0x88) - leave U-Boot's */
module_param(skip_sel, int, 0444);
static int sdio;		/* 1 = SDIO probe (CMD5) for the AR8030 instead of the SD sequence */
module_param(sdio, int, 0444);
static int boot_delay_ms;	/* ms of CONTINUOUS card clock before CMD0/CMD5 (AR8030 ROM boot) */
module_param(boot_delay_ms, int, 0444);
static int rst_n = 1;		/* pulse the controller card-reset line RST_n(0x78) before CMD0
				 * (vendor dw_mci_hw_reset does this; releases the AR8030).
				 */
module_param(rst_n, int, 0444);

static void __iomem *ctl;	/* controller base */
static void __iomem *clk_en, *clk_sel_r, *clk_cfg_r;

static inline u32 rd(int off) { return readl(ctl + off); }
static inline void wr(u32 v, int off) { writel(v, ctl + off); }

/* Issue a "update clock registers only" command and wait for it to retire. */
static int upd_clk(void)
{
	u32 cmd = CMD_START | CMD_UPD_CLK | CMD_PRV_DAT_WAIT;
	u32 v;

	wr(cmd, REG_CMD);
	return readl_poll_timeout_atomic(ctl + REG_CMD, v, !(v & CMD_START), 10, 100000);
}

static void set_clock(unsigned int hz)
{
	u32 div;

	/* disable card clock, apply */
	wr(0, REG_CLKENA);
	upd_clk();
	/* divider: card clk = src_hz / (2*div); div 0 = bypass */
	if (hz >= (unsigned int)src_hz)
		div = 0;
	else
		div = DIV_ROUND_UP(src_hz, 2 * hz);
	wr(div, REG_CLKDIV);
	wr(0, REG_CLKSRC);
	upd_clk();
	/* enable card clock (no low-power gating during bring-up) */
	wr(CLKENA_CCLK, REG_CLKENA);
	upd_clk();
	pr_info("%s: req %u Hz, src %d, CLKDIV=%u -> ~%u Hz\n",
		__func__, hz, src_hz, div, div ? src_hz / (2 * div) : src_hz);
}

/* Send one command, poll for completion, return 0 / -ETIMEDOUT / -EILSEQ. resp[4] filled. */
static int send_cmd(u8 idx, u32 arg, u32 flags, u32 *resp)
{
	u32 v;
	int ret;

	/* clear stale interrupts */
	wr(0xffffffff, REG_RINTSTS);
	/* wait for data-not-busy */
	readl_poll_timeout_atomic(ctl + REG_STATUS, v, !(v & STATUS_DATA_BUSY), 10, 100000);

	wr(arg, REG_CMDARG);
	wr(CMD_START | (use_hold ? CMD_USE_HOLD : 0) | flags | idx, REG_CMD);

	/* wait for command-done */
	ret = readl_poll_timeout_atomic(ctl + REG_RINTSTS, v, v & (INT_CMD_DONE | INT_RTO | INT_HLE),
					10, 500000);
	if (ret) {
		pr_info("CMD%u: no cmd-done (RINTSTS=0x%08x)\n", idx, rd(REG_RINTSTS));
		return -ETIMEDOUT;
	}
	v = rd(REG_RINTSTS);
	if (resp) {
		resp[0] = rd(REG_RESP0);
		resp[1] = rd(REG_RESP0 + 4);
		resp[2] = rd(REG_RESP0 + 8);
		resp[3] = rd(REG_RESP0 + 12);
	}
	pr_info("CMD%u: RINTSTS=0x%08x RESP=%08x %08x %08x %08x\n", idx, v,
		resp ? resp[0] : 0, resp ? resp[1] : 0, resp ? resp[2] : 0, resp ? resp[3] : 0);
	wr(0xffffffff, REG_RINTSTS);
	if (v & INT_RTO)
		return -ETIMEDOUT;
	if ((flags & CMD_RESP_CRC) && (v & INT_RCRC))
		return -EILSEQ;
	return 0;
}

static void ctrl_init(void)
{
	u32 v;

	/* reset controller + FIFO + DMA */
	wr(CTRL_RESET | CTRL_FIFO_RESET | CTRL_DMA_RESET, REG_CTRL);
	readl_poll_timeout_atomic(ctl + REG_CTRL, v,
				  !(v & (CTRL_RESET | CTRL_FIFO_RESET | CTRL_DMA_RESET)),
				  10, 100000);
	wr(0xffffffff, REG_RINTSTS);	/* clear all interrupts */
	wr(0, REG_INTMASK);		/* polled: mask all */
	wr(0xffffffff, REG_TMOUT);	/* max response/data timeout */
	wr(0x00ffffff, REG_FIFOTH);	/* permissive FIFO thresholds */
	wr(0, REG_CTYPE);		/* 1-bit bus for init */
	if (en_shift >= 0)
		wr(en_shift, 0x108);	/* ENABLE_SHIFT: sample-point shift */
	if (uhs_reg >= 0)
		wr(uhs_reg, REG_UHS);
	wr(1, REG_PWREN);		/* power slot 0 */
	udelay(1000);
}

static int __init artosyn_mmc_init(void)
{
	u32 resp[4];
	int ret, i;
	bool is_sd1 = (base == 0x01c00000);

	pr_info("STAGE1 self-test on controller 0x%lx (%s)\n", base, is_sd1 ? "SD/mmc1" : "mmc0");

	ctl       = ioremap(base, 0x1000);
	clk_en    = ioremap(CLK_EN_ADDR, 4);
	clk_sel_r = ioremap(is_sd1 ? CLK1_SEL_ADDR : CLK0_SEL_ADDR, 4);
	clk_cfg_r = ioremap(is_sd1 ? CLK1_CFG_ADDR : CLK0_CFG_ADDR, 4);
	if (!ctl || !clk_en || !clk_sel_r || !clk_cfg_r) {
		pr_err("ioremap failed\n");
		return -ENOMEM;
	}

	/* Artosyn SDMMC clock block, EXACTLY as dw_mci_proxima_init does:
	 *   CFG (0x8c/0xc4): clear low 5 bits, OR in the phase (table is identity 0..15),
	 *                    PRESERVE the high bits (DLL/clock config).
	 *   SEL (0x88/0xc0): write the bus_hz tap directly (100 MHz -> 0x80).
	 */
	pr_info("clock regs BEFORE (U-Boot): EN(0x108000)=0x%08x SEL=0x%08x CFG=0x%08x\n",
		readl(clk_en), readl(clk_sel_r), readl(clk_cfg_r));
	if (clken >= 0)
		writel(clken, clk_en);		/* else leave U-Boot's master-enable */
	if (!skip_cfg) {
		u32 c = readl(clk_cfg_r);

		writel((c & ~0x1fu) | ((u32)clk_cfg & 0x1fu), clk_cfg_r);
	}
	dsb(st);
	if (!skip_sel)
		writel(clk_sel, clk_sel_r);
	dsb(st);
	udelay(200);
	pr_info("clock regs AFTER: EN=0x%08x SEL=0x%08x CFG=0x%08x\n",
		readl(clk_en), readl(clk_sel_r), readl(clk_cfg_r));

	ctrl_init();
	pr_info("CDETECT=0x%08x (bit0=0 means card present)\n", rd(REG_CDETECT));
	set_clock(400000);	/* 400 kHz identification clock */
	mdelay(2);

	/* Pulse the controller's card hardware-reset line, EXACTLY as the vendor
	 * dw_mci_hw_reset does. ctrl_init's CTRL=reset drops RST_n to its asserted
	 * default; an SDIO baseband (AR8030) stays held in reset until we deassert it.
	 * SD cards ignore RST_n (which is why the SD enumerated without this).
	 */
	if (rst_n) {
		u32 r = rd(REG_RST_N);

		pr_info("RST_n(0x78) before=0x%08x -> pulse low/high (release card reset)\n", r);
		wr(r & ~1u, REG_RST_N);	/* assert (active-low) */
		mdelay(10);
		wr(r | 1u, REG_RST_N);	/* deassert -> chip boots; boot_delay_ms lets ROM come up */
		mdelay(2);
	}

	/* The card clock now runs CONTINUOUSLY (CLKENA=CCLK, no low-power gate). An active
	 * SDIO baseband (AR8030) boots its ROM off this clock and only answers once booted;
	 * a passive SD card needs no such wait. Give the ROM time before probing.
	 */
	if (boot_delay_ms) {
		pr_info("holding continuous card clock %d ms for chip ROM boot...\n", boot_delay_ms);
		mdelay(boot_delay_ms);
	}

	/* 74+ init clocks then CMD0 GO_IDLE (no response) */
	ret = send_cmd(0, 0, CMD_INIT, NULL);
	pr_info("CMD0 (GO_IDLE): ret=%d\n", ret);
	mdelay(2);

	if (sdio) {
		/* SDIO probe: CMD5 IO_SEND_OP_COND. R4 (48-bit, NO CRC). arg=0 queries the
		 * OCR; then resend with the voltage window until the chip reports ready (bit31).
		 * If the AR8030 answers this at all, the chip is alive (was the whole problem).
		 */
		/* CMD5 (R4) takes NO PRV_DAT_WAIT - the vendor only sets it for data cmds.
		 * With it set, the controller defers the command until STATUS.busy clears,
		 * which an un-reset chip may never do -> the command never goes out.
		 */
		ret = send_cmd(5, 0, CMD_RESP_EXP, resp);
		if (ret) {
			pr_info("*** CMD5 (SDIO query) FAILED ret=%d - AR8030 still silent ***\n", ret);
		} else {
			pr_info("*** CMD5 (SDIO query) ANSWERED: R4=0x%08x - AR8030 IS ALIVE ***\n", resp[0]);
			for (i = 0; i < 20; i++) {
				ret = send_cmd(5, resp[0] & 0x00ffffff,
					       CMD_RESP_EXP | CMD_PRV_DAT_WAIT, resp);
				if (ret) {
					pr_info("CMD5 (set OCR) failed ret=%d (iter %d)\n", ret, i);
					break;
				}
				if (resp[0] & 0x80000000) {
					pr_info("*** SDIO ready: OCR=0x%08x, %u IO functions (iter %d) ***\n",
						resp[0], (resp[0] >> 28) & 7, i);
					break;
				}
				mdelay(10);
			}
		}
		goto done;
	}

	/* CMD8 SEND_IF_COND, arg 0x1aa, R7 (short response, CRC checked) */
	ret = send_cmd(8, 0x1aa, CMD_RESP_EXP | CMD_RESP_CRC | CMD_PRV_DAT_WAIT, resp);
	if (ret == 0 && (resp[0] & 0xfff) == 0x1aa)
		pr_info("*** CMD8 SUCCESS - card echoes 0x1aa ***\n");

	/* SD identification: ACMD41 (CMD55 then CMD41) until the card powers up. */
	for (i = 0; i < 20; i++) {
		ret = send_cmd(55, 0, CMD_RESP_EXP | CMD_RESP_CRC | CMD_PRV_DAT_WAIT, resp);
		if (ret) {
			pr_info("CMD55 failed ret=%d (iter %d)\n", ret, i);
			break;
		}
		/* ACMD41: R3, NO CRC check. HCS=bit30, voltage window 0xff8000. */
		ret = send_cmd(41, 0x40ff8000, CMD_RESP_EXP | CMD_PRV_DAT_WAIT, resp);
		if (ret) {
			pr_info("ACMD41 failed ret=%d (iter %d)\n", ret, i);
			break;
		}
		if (resp[0] & 0x80000000) {	/* card ready (busy bit set) */
			pr_info("*** ACMD41 done: OCR=0x%08x, card powered up (iter %d) ***\n",
				resp[0], i);
			break;
		}
		mdelay(20);
	}

	/* CMD2 ALL_SEND_CID (R2, long response) then CMD3 SEND_RELATIVE_ADDR (R6). */
	ret = send_cmd(2, 0, CMD_RESP_EXP | CMD_RESP_LONG | CMD_RESP_CRC | CMD_PRV_DAT_WAIT, resp);
	pr_info("CMD2 (ALL_SEND_CID): ret=%d\n", ret);
	ret = send_cmd(3, 0, CMD_RESP_EXP | CMD_RESP_CRC | CMD_PRV_DAT_WAIT, resp);
	pr_info("CMD3 (SEND_RCA): ret=%d RCA=0x%04x %s\n", ret, resp[0] >> 16,
		(ret == 0 && (resp[0] >> 16)) ? "*** CARD IDENTIFIED ***" : "");

done:
	return 0;	/* stay loaded so registers can be inspected */
}

static void __exit artosyn_mmc_exit(void)
{
	if (ctl)
		iounmap(ctl);
	if (clk_en)
		iounmap(clk_en);
	if (clk_sel_r)
		iounmap(clk_sel_r);
	if (clk_cfg_r)
		iounmap(clk_cfg_r);
}

module_init(artosyn_mmc_init);
module_exit(artosyn_mmc_exit);
MODULE_DESCRIPTION("Artosyn Proxima standalone DesignWare MMC host (re-impl of vendor path)");
MODULE_LICENSE("GPL");
