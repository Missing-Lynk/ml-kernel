// SPDX-License-Identifier: GPL-2.0
/*
 * artosyn_vo.c - DRM/KMS driver for the custom Artosyn "VO" display controller
 * (@0x08810000, IRQ 102 = GIC_SPI 70). Scans a contiguous framebuffer out over the DPI
 * interface to the DesignWare MIPI-DSI host (mainline dw-mipi-dsi, separate glue driver)
 * and the QY45043A0 panel. No mainline driver; register map recovered from libmpp_service.so
 * (reg_panel_set / reg_dpi_set / reg_framebuffer_set). See ../../../../../docs/display-backlight.md.
 *
 * Uses drm_simple_display_pipe (single CRTC + primary plane + encoder) and attaches the
 * DSI bridge found via of_graph, plus three overlay planes on the DC overlay register
 * banks: two positionable YUV video planes (NV12/I420, banks 0/1) and the ARGB4444 HUD
 * (bank 2, per-pixel alpha, topmost). Framebuffers are DMA/CMA (the VO scans physical
 * memory); the video planes take PRIME-imported wave5 decoder dmabufs.
 */
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/timer.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

/* --- VO register map (offsets from 0x08810000), recovered from libmpp_service.so --- */
#define VO_GLOBAL	0x0000	/* BIT(11) AXI enable, BIT(12) soft-reset pulse */
#define  VO_GLOBAL_AXI_EN	BIT(11)
#define  VO_GLOBAL_SRESET	BIT(12)
#define VO_FB_BASE0	0x1400	/* plane0 framebuffer DMA base (Y/luma for YUV) */
#define VO_FB_STRIDE	0x1408	/* Y/luma plane stride in bytes */
#define VO_U_STRIDE	0x1508	/* U (Cb) plane stride in bytes, YUV */
#define VO_V_STRIDE	0x1510	/* V (Cr) plane stride in bytes, YUV */
#define VO_PANEL_CTRL	0x1418	/* panel/sync-polarity control word */
#define VO_H_ACT_FP	0x1430	/* HACT | (HFP << 16) */
#define VO_H_SA_BP	0x1438	/* 0x40000000 | HSA | (HBP << 15) */
#define VO_V_ACT_FP	0x1440	/* VACT | (VFP << 16) */
#define VO_V_SA_BP	0x1448	/* 0x40000000 | VSA | (VBP << 15) */
#define VO_INT_STATUS	0x147c
#define VO_INT_ENABLE	0x1480	/* BIT(0) frame/vsync-done */
#define  VO_INT_VSYNC		BIT(0)
#define VO_DPI_FORMAT	0x14b8	/* DPI data format (RGB mode) */
#define VO_MAIN_CTRL	0x1518	/* BIT(3) config-lock, BIT(4) display-start, BIT(8) clear-en */
#define  VO_MAIN_LOCK		BIT(3)	/* shadow-register hold; addr latches on the 1->0 edge */
#define  VO_MAIN_START		BIT(4)
#define VO_FB_FORMAT	0x1520	/* dcregFrameBufferScaleConfig (tile/scale nibbles); 0 = unscaled */
#define VO_LAYER_CONFIG	0x1528	/* layer-config word; stock writes 1 for the YUV video layer */
#define VO_PLANE2_BASE	0x1530	/* plane2 base = U (Cb) for YUV; = fb base for packed RGB */
#define VO_PLANE3_BASE	0x1538	/* plane3 base = V (Cr) for YUV; = fb base for packed RGB */
#define VO_FB_SIZE	0x1810	/* (w & 0x7fff) | (h << 15) */
#define VO_FIFO_THRESH0	0x1800	/* layer FIFO/prefetch threshold (0x3c0); 0 stalls emit */
#define VO_FIFO_THRESH1	0x1808
#define VO_WIN_X	0x1828	/* layer window X origin (0 = top-left; size != origin!) */
#define VO_WIN_Y	0x1830	/* layer window Y origin */
#define VO_LAYER_ALPHA	0x1a18	/* background/clear ARGB; alpha in bits[31:24] */
#define VO_SCALER_RATIO	0x1a20	/* H|V scale ratio Q1.15, 0x8000 = unity */
#define VO_LAYER_CSC	0x1e40	/* primary-layer YUV->RGB CSC: 16 regs, step 8 (0x1e40..0x1eb8) */
#define VO_QOS		0xff00	/* AXI read QoS */
#define VO_DPI_CTRL	0x1f40	/* DPI polarity/enable (bit20 = DPI master en) */

/* Overlay register banks - each overlay layer is one bank of registers, composited over the
 * primary. The offsets below are BANK 0; bank N adds 4*N to every offset (reg_overlay1_set
 * @0x2e4f90 is byte-for-byte reg_overlay_set with every offset +4). Banks 0 and 1 are
 * CONFIRMED from the disassembly; bank 2 (+8) is INFERRED from the +4 array stride (the
 * vendor firmware only ever drives two overlays) and needs HW validation.
 * RE: ../../../../../docs/artosyn-vo-video-path.md ("The overlay plane"). reg_overlay_set @0x2e4df8.
 */
#define VO_OVL_CONFIG	0x1540	/* FORMAT bits[21:16], swizzle bits[7:5], EMIT bit24 */
#define  VO_OVL_EMIT		BIT(24)
#define VO_OVL_ALPHA	0x1580	/* alpha-blend cfg; 0x3140 = per-pixel source-over */
#define VO_OVL_BASE	0x15c0	/* overlay Y / packed-RGB base (shadow-latched via 0x1518 bit3) */
#define VO_OVL_STRIDE	0x1600	/* Y / packed stride in BYTES */
#define VO_OVL_TL	0x1640	/* top-left: X | (Y<<15) */
#define VO_OVL_BR	0x1680	/* bottom-right: X | (Y<<15) */
#define VO_OVL_SRC_GCOLOR 0x16c0 /* src global colour (only used in GLOBAL alpha mode) */
#define VO_OVL_DST_GCOLOR 0x1700 /* dst global colour */
#define VO_OVL_CKEY	0x1740	/* colour-key low */
#define VO_OVL_CKEY_HI	0x1780	/* colour-key high */
#define VO_OVL_SIZE	0x17c0	/* size: W | (H<<15) */
#define VO_OVL_UPLANE	0x1840	/* U (Cb) plane base; NV12: the interleaved UV base (INFERRED) */
#define VO_OVL_VPLANE	0x1880	/* V (Cr) plane base; NV12: unused */
#define VO_OVL_USTRIDE	0x18c0	/* U stride in bytes; NV12: UV stride */
#define VO_OVL_VSTRIDE	0x1900	/* V stride in bytes */
#define VO_OVL_CLEAR	0x1940	/* overlay clear / background ARGB */
#define  VO_OVL_ALPHA_PERPIXEL	0x00003140 /* SRC/DST_GLOBAL_ALPHA_MODE=NORMAL, source-over */
#define  VO_OVL_ALPHA_OPAQUE	0x00008120 /* SRC_BLENDING=ONE, DST_BLENDING=ZERO (stock's
					    * RGB565/YV12/X-format branch, video-path 9.3)
					    */
#define  VO_OVL_SWIZZLE		(1 << 5)   /* bits[7:5] = 1, constant in the stock builder */
#define  VO_OVL_FMT_ARGB4444	((1 << 16) | VO_OVL_SWIZZLE) /* FORMAT=1 (A4R4G4B4), emit off */
#define VO_CURSOR_CFG	0x1468	/* hw cursor; FORMAT bits[1:0], 0 = disabled */

/*
 * Overlay bank allocation, HW-validated 2026-07-05: the DC has exactly TWO overlay banks
 * (a bank-2 write is accepted but never scans out - the silicon wires primary + 2 overlays,
 * matching the VO channel count), and the inter-overlay stacking IS ascending bank order
 * (bank 1 composites over bank 0; proven with an ARGB4444 HUD on bank 1 over video on
 * bank 0). Consequence: the two RF video tiles (banks 0 + 1) and the HUD cannot all have a
 * plane at once - full-frame two-tile video runs without the DRM HUD, and the HUD-over-video
 * shape uses video0 + hud only. hud_bank defaults to 1; a HUD commit on bank 1 clashes with
 * video1, so userspace enables either video1 or the HUD, never both.
 */
#define VO_BANK_VIDEO0	0
#define VO_BANK_VIDEO1	1
#define VO_BANK_HUD	1

static unsigned int hud_bank = VO_BANK_HUD;
module_param(hud_bank, uint, 0444);
MODULE_PARM_DESC(hud_bank, "overlay register bank for the ARGB4444 HUD plane (default 1 - the DC has only banks 0/1; the HUD clashes with the video plane on the same bank)");

#define VO_SYNC_VALID	0x40000000	/* bit30 in H_SA_BP / V_SA_BP = sync valid */

/*
 * DC subsystem CRG (clock/reset) page, physical 0x0a106000. The DC AXI fetch-master
 * enable lives HERE, not in the DC block - libmpp's dc_subsys_power_on (0x2ddcb0) writes
 * CRG+0x18 to clock and un-reset the fetch master. Without it the fetch DMA returns zeros
 * for every address and the panel shows the format-default colour regardless of the buffer.
 * RE: ../../../../../docs/artosyn-vo-video-path.md ("The DC cold bring-up that engages the fetch master").
 */
#define CRG_BASE	0x0a106000
#define CRG_SIZE	0x1000
#define CRG_DC		0x0018	/* bits 9/10 hard-reset pulse; bit0/1 clk-enable; bit2 reset; bit12 AXI master */

/* One DRM plane backed by one overlay register bank. */
struct ar_vo_plane {
	struct drm_plane	base;
	u32			bank;	/* overlay bank index; every register offset gets +4*bank */
};

struct ar_vo {
	struct drm_device		drm;
	struct drm_simple_display_pipe	pipe;
	struct ar_vo_plane		hud;		/* OSD/menu, ARGB4444, per-pixel alpha, topmost */
	struct ar_vo_plane		video[2];	/* positionable YUV tiles (NV12/I420), banks 0/1 */
	void __iomem			*regs;
	void __iomem			*crg;
	struct clk			*pclk;
	u32				cur_format;	/* DRM fourcc currently programmed on the primary layer */
	u32				cur_cfg;	/* clean VO_MAIN_CTRL word for cur_format (no LOCK, no START) */

	/* 1 Hz stall watchdog, logging only: a dead vsync IRQ is otherwise unobservable
	 * once userspace stops committing.
	 */
	struct timer_list		dbg_timer;
	atomic_t			irq_vsync;	/* handled vsync IRQs */
	atomic_t			irq_total;	/* ISR entries */
	atomic_t			irq_none;	/* ISR entries returning IRQ_NONE */
	u32				last_vsync;	/* counter snapshots at the previous tick */
	u32				last_total;
	bool				snap_done;	/* one-shot register snapshot logged */
};

#define to_ar_vo(d) container_of(d, struct ar_vo, drm)
#define to_ar_vo_plane(p) container_of(p, struct ar_vo_plane, base)

/* Overlay-bank register write: bank N lives at every bank-0 offset +4*N. */
static void ar_vo_ovl_wr(struct ar_vo *vo, u32 bank, u32 reg, u32 val)
{
	writel(val, vo->regs + reg + 4 * bank);
}

static u32 ar_vo_ovl_rd(struct ar_vo *vo, u32 bank, u32 reg)
{
	return readl(vo->regs + reg + 4 * bank);
}

static const u32 ar_vo_formats[] = {
	DRM_FORMAT_XRGB8888,	/* fbdev console / menu-as-primary */
	DRM_FORMAT_ARGB8888,	/* opaque primary; alpha ignored (bottom layer) */
	DRM_FORMAT_YUV420,	/* I420, 3-plane: the native DC video format (enum 15), the live/still path */
};

static void ar_vo_set_fb_addr(struct ar_vo *vo, struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	unsigned int n = fb->format->num_planes;
	u32 y = lower_32_bits(drm_fb_dma_get_gem_addr(fb, state, 0));
	u32 u = n > 1 ? lower_32_bits(drm_fb_dma_get_gem_addr(fb, state, 1)) : y;
	u32 v = n > 2 ? lower_32_bits(drm_fb_dma_get_gem_addr(fb, state, 2)) : u;
	/* Use the stored clean config (not a readback). The hardware sets bit6 in VO_MAIN_CTRL after
	 * we write the FORMAT word; propagating it to the LOCK and START writes causes alternating-row
	 * banding in I420 mode (bit6 appears to enable a tiled-fetch or interlaced mode). Stock avoids
	 * this by writing FORMAT+START in a single shot and never reading back. We store cur_cfg so the
	 * LOCK window and the final START write both see only the bits we actually want.
	 */
	u32 cfg = vo->cur_cfg;

	/*
	 * VO_FB_BASE0 / PLANE2 / PLANE3 are shadow registers. They only load into the
	 * active fetch set when written inside the config-lock (bit3) window: set bit3,
	 * write the addresses, clear bit3 -> the new address latches at the next frame-start.
	 * Writing them with bit3 clear leaves the active address at its reset value (0), so
	 * the layer fetches zeros and the panel shows a format-default colour instead of the
	 * framebuffer. recovered from libmpp dc_set_frameaddr2 (reg_config_lock/unlock).
	 *
	 * For 3-plane I420, plane2/plane3 carry the U (Cb) and V (Cr) planes. Packed RGB has
	 * one plane, so all three point at the same buffer (stock does this too). TODO: the live
	 * capture shows bit0 set on the chroma bases (a per-plane flag, not address); we write
	 * page-aligned addresses with bit0 clear, revisit if chroma fetch is wrong,
	 * as with the I420-vs-YV12 U/V plane order.
	 */
	writel(cfg | VO_MAIN_LOCK, vo->regs + VO_MAIN_CTRL);
	writel(y, vo->regs + VO_FB_BASE0);
	writel(u, vo->regs + VO_PLANE2_BASE);
	writel(v, vo->regs + VO_PLANE3_BASE);
	writel(cfg & ~VO_MAIN_LOCK, vo->regs + VO_MAIN_CTRL);
}

/*
 * Register values for 1080p60, captured verbatim from the working stock display via
 * /dev/mem (the statically recovered field encodings were wrong; these are confirmed to make the
 * VO scan out on hardware). The panel is fixed 1080p60, so they are hardcoded.
 * TODO: decode the timing-word encoding to derive these from drm_display_mode for other modes.
 */
#define VO_1080P60_H_ACT_FP	0x082b0780
#define VO_1080P60_H_SA_BP	0x440007d5
#define VO_1080P60_V_ACT_FP	0x04780438
#define VO_1080P60_V_SA_BP	0x422c8458
#define VO_1080P60_FB_SIZE	0x021c0780
#define VO_1080P60_DPI_FORMAT	0x00000005	/* 24-bit RGB888 */

/* 0x1520 is NOT the pixel format - it is the Vivante dcregFrameBufferScaleConfig (scaler).
 * For an unscaled 1:1 layer leave it 0. (Stock's 0x33 here is the video scaler config.)
 */
#define VO_1080P60_SCALE_CFG	0x00000000
#define VO_1080P60_PANEL_CTRL	0x00000311
#define VO_1080P60_DPI_CTRL	0x00120000
#define VO_1080P60_GLOBAL	0x00000100

/*
 * 0x1518 = Vivante dcregFrameBufferConfig. The PIXEL FORMAT is bits[31:26] (Vivante surface
 * enum: X4R4G4B4=0, A4R4G4B4=1, X1R5G5B5=2, A1R5G5B5=3, R5G6B5=4, X8R8G8B8=5, A8R8G8B8=6,
 * YV12=15). Stock 0x3c004011 = FORMAT 15 = YV12 (its decoded-video layer) - copying it told the
 * DC our RGB framebuffer was YUV, hence the black screen. For the DRM XRGB8888 fbdev use
 * X8R8G8B8=5 (opaque, ignores the top byte): (5<<26) | 0x11 = 0x14000011, with the YUV-only
 * bit14 cleared and bit0 OUTPUT / bit4 set. This word is written LAST (the go/commit).
 *
 * Note(180 rotation): the panel is physically mounted rotated 180. The DC does NOT rotate
 * (a full 0x08810000 stock-vs-open register diff is identical), and stock does NOT pre-rotate
 * content. The 180 is done at the PANEL: its GIP latches scan direction from two power-on GPIO
 * straps (gpio42 = H mirror, gpio107 = V mirror) set in the device tree; H+V mirror = 180, so
 * content is fed upright and the panel scans it out flipped. The fix belongs in the DT panel
 * straps, not here. Do not set bits[13:11] (a tile/fetch field; setting it garbles the fetch).
 */
#define VO_1080P60_FB_CONFIG	0x14000011	/* = ar_vo_fb_config(XRGB8888); kept for the notes above */

/*
 * Map a DRM fourcc to the DC surface-format enum (0x1518 bits[31:26]) and flag YUV.
 * RGB and I420=15 confirmed; NV12=17 inferred (the firmware never
 * programs it, converting NV12 to I420 upstream). NV12 is not advertised on the primary; the
 * video overlay planes advertise it (needs HW validation, I420 is the proven fallback).
 */
#define VO_FB_CONFIG_YUV	BIT(14)		/* surface YUV flag; set for YUV, clear for RGB (live 0x3c004011) */

static u32 ar_vo_surface_enum(u32 fourcc, bool *is_yuv)
{
	*is_yuv = false;
	switch (fourcc) {
	case DRM_FORMAT_YUV420:
	case DRM_FORMAT_YVU420:		/* same surface enum; Cb/Cr swap handled at the address level */
		*is_yuv = true;
		return 15;			/* I420, 3-plane (CONFIRMED) */
	case DRM_FORMAT_NV12:
		*is_yuv = true;
		return 17;			/* 2-plane (INFERRED, unadvertised) */
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
	default:
		return 5;			/* X8R8G8B8, opaque primary */
	}
}

/* dcregFrameBufferConfig (0x1518): FORMAT[31:26], bit0 OUTPUT, bit4 display-start (held off
 * during the address latch, asserted last by the caller), bit14 for YUV surfaces.
 */
static u32 ar_vo_fb_config(u32 fourcc)
{
	bool yuv;
	u32 cfg = (ar_vo_surface_enum(fourcc, &yuv) << 26) | 0x11;

	if (yuv)
		cfg |= VO_FB_CONFIG_YUV;

	return cfg;
}

/*
 * YUV-to-RGB layer CSC for the primary (full-range BT.709 family, mode 1), captured verbatim
 * from stock with a live I420 frame. The DC applies it only
 * to a YUV surface; the RGB path leaves the block at reset, so it is programmed only for YUV.
 */
static const u32 ar_vo_layer_csc_bt709_full[16] = {
	0x00000401, 0x00000001, 0x00000629, 0x007cecfb,
	0x00000401, 0x00001f45, 0x00001e2b, 0x00014aa1,
	0x00000401, 0x00000745, 0x00001fff, 0x007c60c5,
	0x00000001, 0x00ff0001, 0x00ff0001, 0x00ff0001,
};

static void ar_vo_set_layer_csc(struct ar_vo *vo)
{
	u32 cfg = vo->cur_cfg;	/* same reason as ar_vo_set_fb_addr: avoid propagating HW-set bits */
	int i;

	/*
	 * Commit at vsync via the same 0x1518 bit3 shadow latch as the address/format writes.
	 * TODO: stock byte-packs the last three regs as RMW preserving 0xff00ff00; writing the
	 * captured full value works, revisit if colour is off (dc-yuv-primary.md 3.2).
	 */
	writel(cfg | VO_MAIN_LOCK, vo->regs + VO_MAIN_CTRL);
	for (i = 0; i < 16; i++)
		writel(ar_vo_layer_csc_bt709_full[i], vo->regs + VO_LAYER_CSC + i * 8);

	writel(cfg & ~VO_MAIN_LOCK, vo->regs + VO_MAIN_CTRL);
}

/*
 * Program the layer's per-plane strides, the layer-config word, the FORMAT (with display-start
 * held off), and the YUV CSC. The caller latches the plane addresses (ar_vo_set_fb_addr) and
 * asserts display-start last. Called from pipe enable and, on a format switch, from pipe update.
 */
static void ar_vo_program_layer(struct ar_vo *vo, struct drm_framebuffer *fb)
{
	bool yuv = fb->format->is_yuv;
	u32 cfg = ar_vo_fb_config(fb->format->format) & ~VO_MAIN_START;

	writel(fb->pitches[0], vo->regs + VO_FB_STRIDE);
	if (yuv) {
		writel(fb->pitches[1], vo->regs + VO_U_STRIDE);
		writel(fb->pitches[2], vo->regs + VO_V_STRIDE);
		writel(1, vo->regs + VO_LAYER_CONFIG);
	}

	/* Store cfg BEFORE writing to the register: ar_vo_set_layer_csc uses cur_cfg via
	 * the LOCK bracket and must not read back the HW-modified register value.
	 */
	vo->cur_cfg = cfg;
	writel(cfg, vo->regs + VO_MAIN_CTRL);

	if (yuv)
		ar_vo_set_layer_csc(vo);

	vo->cur_format = fb->format->format;
}

static void ar_vo_pipe_enable(struct drm_simple_display_pipe *pipe,
			      struct drm_crtc_state *crtc_state,
			      struct drm_plane_state *plane_state)
{
	struct ar_vo *vo = to_ar_vo(pipe->crtc.dev);
	struct drm_framebuffer *fb = plane_state->fb;
	u32 g;

	/*
	 * DC subsystem bring-up at the CRG (libmpp dc_subsys_power_on @0x2ddcb0 then dc_hard_reset
	 * @0x2d70d0) - the load-bearing step the open driver previously omitted. Bits 0/1 clock the
	 * DC AXI fetch master, bit2 releases its reset, bit12 enables the master; then a bits-9/10
	 * reset pulse. Without this the fetch DMA reads zeros for EVERY framebuffer address and the
	 * panel shows the format-default colour (green/black) regardless of buffer content. This is
	 * a CRG register (0x0a106018), NOT the DC block. RE: ../../../../../docs/artosyn-vo-video-path.md.
	 */
	writel(readl(vo->crg + CRG_DC) | BIT(0),  vo->crg + CRG_DC);
	writel(readl(vo->crg + CRG_DC) | BIT(1),  vo->crg + CRG_DC);
	writel(readl(vo->crg + CRG_DC) & ~BIT(2), vo->crg + CRG_DC);
	writel(readl(vo->crg + CRG_DC) | BIT(12), vo->crg + CRG_DC);
	writel(readl(vo->crg + CRG_DC) & ~0x600,  vo->crg + CRG_DC);
	writel(readl(vo->crg + CRG_DC) | 0x600,   vo->crg + CRG_DC);

	/* Disable the hardware cursor (dcregCursorConfig FORMAT[1:0]=0); a default/garbage
	 * cursor bitmap otherwise shows as a small stray box. RE: artosyn-vo-video-path.md ("The hardware cursor").
	 */
	writel(readl(vo->regs + VO_CURSOR_CFG) & ~0x3, vo->regs + VO_CURSOR_CFG);

	/*
	 * Stock cold-inits the DC before programming it (libmpp reg_soft_reset / dc_init):
	 * pulse the core soft-reset, clear the config word, then enable the AXI read path with
	 * a read-modify-write. Writing VO_GLOBAL wholesale (the old VO_1080P60_GLOBAL=0x100)
	 * clobbers reset-default bits the fetch engine needs, which left the scanout shadow
	 * address stuck at 0 - the layer fetched zeros and emitted a format-default colour
	 * instead of the framebuffer. AXI read is enabled by CLEARING bit11 (VO_GLOBAL_AXI_EN).
	 */
	g = readl(vo->regs + VO_GLOBAL);
	writel(g | VO_GLOBAL_SRESET, vo->regs + VO_GLOBAL);
	writel(g, vo->regs + VO_GLOBAL);
	udelay(5);
	writel(0, vo->regs + VO_MAIN_CTRL);
	writel(readl(vo->regs + VO_GLOBAL) & ~VO_GLOBAL_AXI_EN, vo->regs + VO_GLOBAL);

	writel(VO_1080P60_H_ACT_FP, vo->regs + VO_H_ACT_FP);
	writel(VO_1080P60_H_SA_BP, vo->regs + VO_H_SA_BP);
	writel(VO_1080P60_V_ACT_FP, vo->regs + VO_V_ACT_FP);
	writel(VO_1080P60_V_SA_BP, vo->regs + VO_V_SA_BP);

	/* Layer surface geometry. Window origin is (0,0); the FIFO thresholds must be non-zero or
	 * the DC fetches but never emits. Per-plane strides, format, and CSC are in ar_vo_program_layer
	 * below (the Y stride at 0x1408 was previously written here).
	 */
	writel(VO_1080P60_FB_SIZE, vo->regs + VO_FB_SIZE);
	writel(0, vo->regs + VO_WIN_X);
	writel(0, vo->regs + VO_WIN_Y);
	writel(0x3c0, vo->regs + VO_FIFO_THRESH0);
	writel(0x3c0, vo->regs + VO_FIFO_THRESH1);
	writel(0xff000000, vo->regs + VO_LAYER_ALPHA);
	writel(0x80008000, vo->regs + VO_SCALER_RATIO);

	writel(VO_1080P60_SCALE_CFG, vo->regs + VO_FB_FORMAT);	/* 0x1520 = scaler cfg, not format */
	writel(VO_1080P60_DPI_FORMAT, vo->regs + VO_DPI_FORMAT);
	writel(VO_1080P60_PANEL_CTRL, vo->regs + VO_PANEL_CTRL);
	writel(VO_1080P60_DPI_CTRL, vo->regs + VO_DPI_CTRL);
	writel(0xff, vo->regs + VO_QOS);

	/*
	 * Program dcregFrameBufferConfig (0x1518) with the pixel FORMAT and OUTPUT bit but
	 * with display-start (emit) OFF, then latch the framebuffer address through the
	 * config-lock window, then assert display-start LAST. This mirrors stock's
	 * reg_framebuffer_set -> dc_set_frameaddr2 -> reg_display_start order.
	 */
	ar_vo_program_layer(vo, fb);
	ar_vo_set_fb_addr(vo, plane_state);
	vo->cur_cfg |= VO_MAIN_START;	/* keep START in cur_cfg: every later LOCK/unlock
					 * bracket rewrites 0x1518 with cur_cfg, and a write
					 * without START stops scanout (black screen)
					 */
	writel(vo->cur_cfg, vo->regs + VO_MAIN_CTRL);

	drm_crtc_vblank_on(&pipe->crtc);
}

static void ar_vo_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct ar_vo *vo = to_ar_vo(pipe->crtc.dev);

	drm_crtc_vblank_off(&pipe->crtc);
	writel(0, vo->regs + VO_MAIN_CTRL);

	/* Drop START from the shadow config: every later LOCK/unlock bracket
	 * rewrites VO_MAIN_CTRL with cur_cfg, and a stale START would restart
	 * scanout into the disabled DSI. pipe_enable re-adds it.
	 */
	vo->cur_cfg &= ~VO_MAIN_START;
}

static void ar_vo_pipe_update(struct drm_simple_display_pipe *pipe,
			      struct drm_plane_state *old_state)
{
	struct ar_vo *vo = to_ar_vo(pipe->crtc.dev);
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_crtc *crtc = &pipe->crtc;

	if (state->fb) {
		/* Reprogram geometry/format/CSC only on a format switch (e.g. RGB console ->
		 * YUV video); steady-state video just re-latches the plane addresses.
		 */
		if (state->fb->format->format != vo->cur_format) {
			ar_vo_program_layer(vo, state->fb);
			ar_vo_set_fb_addr(vo, state);
			vo->cur_cfg |= VO_MAIN_START;	/* see ar_vo_pipe_enable */
			writel(vo->cur_cfg, vo->regs + VO_MAIN_CTRL);
		} else {
			ar_vo_set_fb_addr(vo, state);
		}
	}

	/* Arm the flip event on the real vsync: the shadow registers latch at frame start,
	 * so the IRQ after the latch is the true "new frame is scanning out" edge.
	 */
	if (crtc->state->event) {
		spin_lock_irq(&crtc->dev->event_lock);
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, crtc->state->event);
		else
			drm_crtc_send_vblank_event(crtc, crtc->state->event);

		crtc->state->event = NULL;
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static int ar_vo_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct ar_vo *vo = to_ar_vo(pipe->crtc.dev);

	writel(readl(vo->regs + VO_INT_ENABLE) | VO_INT_VSYNC, vo->regs + VO_INT_ENABLE);
	return 0;
}

static void ar_vo_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct ar_vo *vo = to_ar_vo(pipe->crtc.dev);

	writel(readl(vo->regs + VO_INT_ENABLE) & ~VO_INT_VSYNC, vo->regs + VO_INT_ENABLE);
}

static const struct drm_simple_display_pipe_funcs ar_vo_pipe_funcs = {
	.enable		= ar_vo_pipe_enable,
	.disable	= ar_vo_pipe_disable,
	.update		= ar_vo_pipe_update,
	.enable_vblank	= ar_vo_enable_vblank,
	.disable_vblank	= ar_vo_disable_vblank,
};

/*
 * Overlay planes, one per overlay register bank (see the VO_BANK_* map above):
 *
 *  - hud (bank 2): the OSD/menu surface, ARGB4444, composited with per-pixel source-over
 *    alpha (0x1580 = 0x3140). A transparent (alpha-nibble 0) pixel lets the layers below
 *    show through.
 *  - video0/video1 (banks 0/1): positionable opaque YUV tiles for the RF stream (two wave5
 *    decoder dmabufs, e.g. 1920x560 at y=0 and 1920x552 at y=528, composing one 1080-row
 *    frame in hardware). NV12 uses surface-format enum 17 (INFERRED, DC8000 standard, never
 *    programmed by stock); I420/YV12 uses enum 15 (CONFIRMED, the live primary format).
 *
 * All planes are positionable, 1:1 only (scaling rejected in atomic_check). Registers per
 * ../../../../../docs/artosyn-vo-video-path.md ("The overlay plane").
 */
static const u32 ar_vo_hud_formats[] = {
	DRM_FORMAT_ARGB4444,
};

static const u32 ar_vo_vid_formats[] = {
	DRM_FORMAT_NV12,	/* 2-plane, wave5 default output; enum 17 INFERRED */
	DRM_FORMAT_YUV420,	/* I420 3-plane, wave5 V4L2_PIX_FMT_YUV420; enum 15 CONFIRMED */
	DRM_FORMAT_YVU420,	/* YV12: same as I420 with the chroma planes swapped */
};

static int ar_vo_ovl_check(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct drm_plane_state *ns = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *cs;

	if (!ns->crtc)
		return 0;

	cs = drm_atomic_get_new_crtc_state(state, ns->crtc);
	return drm_atomic_helper_check_plane_state(ns, cs, DRM_PLANE_NO_SCALING,
						   DRM_PLANE_NO_SCALING, true, true);
}

/*
 * Common tail of every overlay-bank update: position/size, then the address + position
 * latch through the 0x1518 bit3 shadow bracket (same vsync latch as the primary), then
 * EMIT. Plane addresses y/u/v are the physical bases; for packed RGB u/v repeat y (the
 * chroma registers are don't-care), for NV12 u carries the interleaved UV base.
 * Uses the stored clean primary config for the bracket, never a readback (see
 * ar_vo_set_fb_addr: propagating hardware-set bit6 causes I420 banding, and a stale
 * readback could drop START).
 */
static void ar_vo_ovl_commit(struct ar_vo *vo, u32 bank, struct drm_plane_state *ns,
			     u32 y, u32 u, u32 v)
{
	u32 cfg = vo->cur_cfg;
	int dx = ns->dst.x1;
	int dy = ns->dst.y1;
	int dw = drm_rect_width(&ns->dst);
	int dh = drm_rect_height(&ns->dst);

	ar_vo_ovl_wr(vo, bank, VO_OVL_SRC_GCOLOR, 0);	/* keep GLOBAL alpha unused -> no tint */
	ar_vo_ovl_wr(vo, bank, VO_OVL_DST_GCOLOR, 0);
	ar_vo_ovl_wr(vo, bank, VO_OVL_CKEY, 0);
	ar_vo_ovl_wr(vo, bank, VO_OVL_CKEY_HI, 0);
	ar_vo_ovl_wr(vo, bank, VO_OVL_CLEAR, 0);
	/* 1:1 only (atomic_check rejects scaling), so the surface size is the dest size. */
	ar_vo_ovl_wr(vo, bank, VO_OVL_SIZE, (dw & 0x7fff) | ((dh & 0x7fff) << 15));

	writel(cfg | VO_MAIN_LOCK, vo->regs + VO_MAIN_CTRL);
	ar_vo_ovl_wr(vo, bank, VO_OVL_BASE, y);
	ar_vo_ovl_wr(vo, bank, VO_OVL_UPLANE, u);
	ar_vo_ovl_wr(vo, bank, VO_OVL_VPLANE, v);
	ar_vo_ovl_wr(vo, bank, VO_OVL_TL, (dx & 0x7fff) | ((dy & 0x7fff) << 15));
	ar_vo_ovl_wr(vo, bank, VO_OVL_BR,
		     ((dx + dw) & 0x7fff) | (((dy + dh) & 0x7fff) << 15));
	writel(cfg & ~VO_MAIN_LOCK, vo->regs + VO_MAIN_CTRL);

	ar_vo_ovl_wr(vo, bank, VO_OVL_CONFIG,
		     ar_vo_ovl_rd(vo, bank, VO_OVL_CONFIG) | VO_OVL_EMIT);
}

static void ar_vo_hud_update(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct ar_vo *vo = to_ar_vo(plane->dev);
	struct drm_plane_state *ns = drm_atomic_get_new_plane_state(state, plane);
	u32 bank = to_ar_vo_plane(plane)->bank;
	u32 addr;

	/* Skip bank writes while the CRTC is off: pipe_enable's SRESET would
	 * clobber them anyway, and the commit's LOCK bracket must not touch
	 * VO_MAIN_CTRL on a stopped pipe. The bank is reprogrammed by the
	 * plane's next atomic update after enable (ml-hud commits every frame).
	 */
	if (!ns->fb || !ns->crtc || !ns->crtc->state->active)
		return;

	addr = lower_32_bits(drm_fb_dma_get_gem_addr(ns->fb, ns, 0));

	ar_vo_ovl_wr(vo, bank, VO_OVL_STRIDE, ns->fb->pitches[0]);
	ar_vo_ovl_wr(vo, bank, VO_OVL_ALPHA, VO_OVL_ALPHA_PERPIXEL);

	/* Format set, emit still off; ar_vo_ovl_commit turns EMIT on after the latch. */
	ar_vo_ovl_wr(vo, bank, VO_OVL_CONFIG, VO_OVL_FMT_ARGB4444);

	ar_vo_ovl_commit(vo, bank, ns, addr, addr, addr);
}

static void ar_vo_vid_update(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct ar_vo *vo = to_ar_vo(plane->dev);
	struct drm_plane_state *ns = drm_atomic_get_new_plane_state(state, plane);
	u32 bank = to_ar_vo_plane(plane)->bank;
	bool yuv;
	u32 fmt, y, u, v, us, vs;

	/* Same CRTC-off skip as ar_vo_hud_update. */
	if (!ns->fb || !ns->crtc || !ns->crtc->state->active)
		return;

	fmt = ar_vo_surface_enum(ns->fb->format->format, &yuv);
	y = lower_32_bits(drm_fb_dma_get_gem_addr(ns->fb, ns, 0));

	if (ns->fb->format->format == DRM_FORMAT_NV12) {
		/* Semi-planar: the interleaved UV plane goes through the U registers, the V
		 * registers are unused (mirrored to U here, harmless). Inferred mapping from
		 * the DC8000 semi-planar path.
		 */
		u = lower_32_bits(drm_fb_dma_get_gem_addr(ns->fb, ns, 1));
		v = u;
		us = ns->fb->pitches[1];
		vs = us;
	} else {
		/* 3-plane I420/YV12. The DC "U" register is taken to be Cb (the vendor fills
		 * the primary's U register with the plane after Y for I420 data and colours
		 * are right); DRM YVU420 has Cr first, so swap.
		 */
		bool yvu = ns->fb->format->format == DRM_FORMAT_YVU420;
		u32 p1 = lower_32_bits(drm_fb_dma_get_gem_addr(ns->fb, ns, 1));
		u32 p2 = lower_32_bits(drm_fb_dma_get_gem_addr(ns->fb, ns, 2));

		u = yvu ? p2 : p1;
		v = yvu ? p1 : p2;
		us = ns->fb->pitches[yvu ? 2 : 1];
		vs = ns->fb->pitches[yvu ? 1 : 2];
	}

	ar_vo_ovl_wr(vo, bank, VO_OVL_STRIDE, ns->fb->pitches[0]);
	ar_vo_ovl_wr(vo, bank, VO_OVL_USTRIDE, us);
	ar_vo_ovl_wr(vo, bank, VO_OVL_VSTRIDE, vs);

	/* Opaque blend: video tiles fully replace whatever is below them (stock's
	 * non-alpha-format branch, 0x8120).
	 */
	ar_vo_ovl_wr(vo, bank, VO_OVL_ALPHA, VO_OVL_ALPHA_OPAQUE);

	/* Format set, emit still off; ar_vo_ovl_commit turns EMIT on after the latch. */
	ar_vo_ovl_wr(vo, bank, VO_OVL_CONFIG, (fmt << 16) | VO_OVL_SWIZZLE);

	ar_vo_ovl_commit(vo, bank, ns, y, u, v);
}

static void ar_vo_ovl_disable(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct ar_vo *vo = to_ar_vo(plane->dev);
	u32 bank = to_ar_vo_plane(plane)->bank;

	ar_vo_ovl_wr(vo, bank, VO_OVL_CONFIG,
		     ar_vo_ovl_rd(vo, bank, VO_OVL_CONFIG) & ~VO_OVL_EMIT);
}

static const struct drm_plane_helper_funcs ar_vo_hud_helper_funcs = {
	.atomic_check	= ar_vo_ovl_check,
	.atomic_update	= ar_vo_hud_update,
	.atomic_disable	= ar_vo_ovl_disable,
};

static const struct drm_plane_helper_funcs ar_vo_vid_helper_funcs = {
	.atomic_check	= ar_vo_ovl_check,
	.atomic_update	= ar_vo_vid_update,
	.atomic_disable	= ar_vo_ovl_disable,
};

static const struct drm_plane_funcs ar_vo_ovl_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

static irqreturn_t ar_vo_irq(int irq, void *data)
{
	struct ar_vo *vo = data;
	u32 status = readl(vo->regs + VO_INT_STATUS);

	atomic_inc(&vo->irq_total);
	if (!(status & VO_INT_VSYNC)) {
		atomic_inc(&vo->irq_none);
		return IRQ_NONE;
	}

	/* Write-back clears: confirmed on device 2026-07-03 (bit forced on via /dev/mem with
	 * this ISR live: steady 62 IRQ/s for 5 s, no level-retrigger storm, no spurious-IRQ
	 * disable). Safe to depend on for vblank/flip signalling.
	 */
	writel(status, vo->regs + VO_INT_STATUS);
	atomic_inc(&vo->irq_vsync);
	drm_crtc_handle_vblank(&vo->pipe.crtc);

	return IRQ_HANDLED;
}

/* 1 Hz stall watchdog tick: silent while vsync IRQs flow (or vblank is off), one warning
 * per second once they stop. Reports the rates, the IRQ_NONE count, the live status and
 * enable registers, and the DRM vblank count.
 */
static void ar_vo_dbg_tick(struct timer_list *t)
{
	struct ar_vo *vo = timer_container_of(vo, t, dbg_timer);
	u32 vsync = atomic_read(&vo->irq_vsync);
	u32 total = atomic_read(&vo->irq_total);
	u32 ie = readl(vo->regs + VO_INT_ENABLE);

	/* One-shot register snapshot on the first tick after the pipe is up (vblank
	 * enabled): a boot that stays black with all software state healthy can then be
	 * diffed against a good boot at register level from dmesg alone.
	 */
	if (!vo->snap_done && (ie & VO_INT_VSYNC)) {
		vo->snap_done = true;
		dev_info(vo->drm.dev,
			 "VO snap: global=0x%08x main=0x%08x fb=0x%08x/0x%08x/0x%08x stride=%u size=0x%08x\n",
			 readl(vo->regs + VO_GLOBAL), readl(vo->regs + VO_MAIN_CTRL),
			 readl(vo->regs + VO_FB_BASE0), readl(vo->regs + VO_PLANE2_BASE),
			 readl(vo->regs + VO_PLANE3_BASE), readl(vo->regs + VO_FB_STRIDE),
			 readl(vo->regs + VO_FB_SIZE));
		dev_info(vo->drm.dev,
			 "VO snap: dpi=0x%08x/0x%08x panel=0x%08x h=0x%08x/0x%08x v=0x%08x/0x%08x fifo=0x%x/0x%x crg=0x%08x\n",
			 readl(vo->regs + VO_DPI_CTRL), readl(vo->regs + VO_DPI_FORMAT),
			 readl(vo->regs + VO_PANEL_CTRL),
			 readl(vo->regs + VO_H_ACT_FP), readl(vo->regs + VO_H_SA_BP),
			 readl(vo->regs + VO_V_ACT_FP), readl(vo->regs + VO_V_SA_BP),
			 readl(vo->regs + VO_FIFO_THRESH0), readl(vo->regs + VO_FIFO_THRESH1),
			 readl(vo->crg + CRG_DC));
	}

	if ((ie & VO_INT_VSYNC) && vsync == vo->last_vsync)
		dev_warn(vo->drm.dev,
			 "VO stall: vsync/s=%u irq/s=%u none=%u status=0x%08x ie=0x%08x vblcnt=%llu\n",
			 vsync - vo->last_vsync, total - vo->last_total,
			 atomic_read(&vo->irq_none),
			 readl(vo->regs + VO_INT_STATUS), ie,
			 drm_crtc_vblank_count(&vo->pipe.crtc));

	vo->last_vsync = vsync;
	vo->last_total = total;
	mod_timer(&vo->dbg_timer, jiffies + HZ);
}

DEFINE_DRM_GEM_DMA_FOPS(ar_vo_fops);

/*
 * RGB (32 bpp): pad the dumb-buffer pitch to a 512-px-aligned line stride, matching what
 * ar_vo_pipe_enable writes to 0x1408 for the validated RGB path (1920 -> 2048 px = 8192 B,
 * matches ar_disp_get_buffer_width in the firmware).
 *
 * 8-bpp allocations are YUV planes: the DC scans a 64-px-aligned luma stride cleanly
 * (verified on-panel 2026-07-05: i420_test with LSTRIDE=1920 renders identically to 2048),
 * and the chroma stride is HARDCODED to width/2 in hardware (VO_U/V_STRIDE ignored).
 * Generic DRM clients (kmssink, dma-buf importers) derive the chroma stride as
 * luma-pitch/2, so the luma pitch must NOT be padded, or their chroma layout lands at
 * pitch/2 != width/2 and shears (the "green stair"). 64-px alignment keeps luma-pitch/2
 * equal to width/2 for any 128-px-aligned width, and matches the wave5 decoder's native
 * output stride so decoder dma-bufs import 1:1.
 */
static int ar_vo_dumb_create(struct drm_file *file, struct drm_device *dev,
			     struct drm_mode_create_dumb *args)
{
	if (args->bpp == 8)
		args->pitch = ALIGN(args->width, 64);
	else
		args->pitch = ALIGN(args->width, 512) * DIV_ROUND_UP(args->bpp, 8);

	return drm_gem_dma_dumb_create_internal(file, dev, args);
}

static const struct drm_driver ar_vo_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &ar_vo_fops,
	DRM_GEM_DMA_DRIVER_OPS_WITH_DUMB_CREATE(ar_vo_dumb_create),
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name			= "artosyn-vo",
	.desc			= "Artosyn VO display controller",
	.major			= 1,
	.minor			= 0,
};

static const struct drm_mode_config_funcs ar_vo_mode_config_funcs = {
	.fb_create	= drm_gem_fb_create,
	.atomic_check	= drm_atomic_helper_check,
	.atomic_commit	= drm_atomic_helper_commit,
};

static int ar_vo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_bridge *bridge;
	struct ar_vo *vo;
	int ret, irq, i;

	vo = devm_drm_dev_alloc(dev, &ar_vo_drm_driver, struct ar_vo, drm);
	if (IS_ERR(vo))
		return PTR_ERR(vo);

	platform_set_drvdata(pdev, vo);

	vo->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vo->regs))
		return PTR_ERR(vo->regs);

	/* DC subsystem CRG page (the fetch-master enable at CRG_DC lives here, not in the DC
	 * block). A fixed physical block rather than a DT resource. See CRG_BASE.
	 */
	vo->crg = devm_ioremap(dev, CRG_BASE, CRG_SIZE);
	if (!vo->crg)
		return -ENOMEM;

	vo->pclk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(vo->pclk))
		return PTR_ERR(vo->pclk);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* The DSI bridge (dw-mipi-dsi glue) is the of_graph remote endpoint on port 0. */
	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge), "no DSI bridge\n");

	ret = drmm_mode_config_init(&vo->drm);
	if (ret)
		return ret;

	vo->drm.mode_config.min_width = 0;
	vo->drm.mode_config.min_height = 0;
	vo->drm.mode_config.max_width = 1920;
	vo->drm.mode_config.max_height = 1080;
	vo->drm.mode_config.funcs = &ar_vo_mode_config_funcs;

	ret = drm_vblank_init(&vo->drm, 1);
	if (ret)
		return ret;

	ret = drm_simple_display_pipe_init(&vo->drm, &vo->pipe, &ar_vo_pipe_funcs,
					   ar_vo_formats, ARRAY_SIZE(ar_vo_formats),
					   NULL, NULL);
	if (ret)
		return ret;

	ret = drm_simple_display_pipe_attach_bridge(&vo->pipe, bridge);
	if (ret)
		return ret;

	/*
	 * Overlay planes on the same CRTC. Creation order keeps the HUD's DRM plane id stable
	 * (it was the second plane created before the video planes existed); the video planes
	 * get the next ids after it. zpos is immutable and informational: the actual stacking
	 * is the hardware blend chain, INFERRED ascending by bank (primary < bank0 < bank1 <
	 * bank2), which the zpos values mirror via the VO_BANK_* map.
	 */
	vo->hud.bank = hud_bank;
	ret = drm_universal_plane_init(&vo->drm, &vo->hud.base,
				       drm_crtc_mask(&vo->pipe.crtc), &ar_vo_ovl_funcs,
				       ar_vo_hud_formats, ARRAY_SIZE(ar_vo_hud_formats),
				       NULL, DRM_PLANE_TYPE_OVERLAY, "hud");
	if (ret)
		return ret;

	drm_plane_helper_add(&vo->hud.base, &ar_vo_hud_helper_funcs);

	for (i = 0; i < 2; i++) {
		vo->video[i].bank = i;	/* VO_BANK_VIDEO0 / VO_BANK_VIDEO1 */
		ret = drm_universal_plane_init(&vo->drm, &vo->video[i].base,
					       drm_crtc_mask(&vo->pipe.crtc), &ar_vo_ovl_funcs,
					       ar_vo_vid_formats, ARRAY_SIZE(ar_vo_vid_formats),
					       NULL, DRM_PLANE_TYPE_OVERLAY, "video%d", i);
		if (ret)
			return ret;

		drm_plane_helper_add(&vo->video[i].base, &ar_vo_vid_helper_funcs);
		drm_plane_create_zpos_immutable_property(&vo->video[i].base, 1 + i);
	}

	drm_plane_create_zpos_immutable_property(&vo->pipe.plane, 0);
	drm_plane_create_zpos_immutable_property(&vo->hud.base, 1 + vo->hud.bank);

	drm_mode_config_reset(&vo->drm);

	ret = devm_request_irq(dev, irq, ar_vo_irq, 0, "artosyn-vo", vo);
	if (ret)
		return ret;

	ret = drm_dev_register(&vo->drm, 0);
	if (ret)
		return ret;

	/* Armed only after registration succeeds: an earlier probe failure would leave
	 * the timer running on devm-freed memory.
	 */
	timer_setup(&vo->dbg_timer, ar_vo_dbg_tick, 0);
	mod_timer(&vo->dbg_timer, jiffies + HZ);

	drm_client_setup(&vo->drm, NULL);

	return 0;
}

static void ar_vo_remove(struct platform_device *pdev)
{
	struct ar_vo *vo = platform_get_drvdata(pdev);

	timer_delete_sync(&vo->dbg_timer);
	drm_dev_unregister(&vo->drm);
	drm_atomic_helper_shutdown(&vo->drm);
}

static const struct of_device_id ar_vo_match[] = {
	{ .compatible = "artosyn,vo" },
	{}
};
MODULE_DEVICE_TABLE(of, ar_vo_match);

static struct platform_driver ar_vo_driver = {
	.probe	= ar_vo_probe,
	.remove	= ar_vo_remove,
	.driver = {
		.name		= "artosyn-vo",
		.of_match_table	= ar_vo_match,
	},
};
module_platform_driver(ar_vo_driver);

MODULE_DESCRIPTION("Artosyn VO display controller (DRM/KMS)");
MODULE_LICENSE("GPL");
