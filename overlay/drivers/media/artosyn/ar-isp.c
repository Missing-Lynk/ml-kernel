// SPDX-License-Identifier: GPL-2.0
/*
 * ar-isp.c - Artosyn ISP capture path, bring-up driver.
 *
 * The VIF has two output routes. The bypass "view" path writes frames straight
 * to DDR and is what ar-vif.c arms; the ISP path hands frames to this block,
 * and it is the one the vendor uses. A write trace of the streaming vendor
 * shows it configures the views, sets the per-view reset, and then captures
 * every frame through the ISP.
 *
 * This driver applies the vendor's register configuration for the 2-lane
 * 1080p60 sensor mode and starts the block. The configuration carries the
 * vendor's own DDR addresses and the device tree reserves that range no-map, so
 * the capture buffers are used as they are. That is enough to answer whether the
 * block produces a frame at all, which is what this driver is for. It is not the
 * final shape.
 *
 * The coefficient tables are the exception: gamma and DRC are allocated here,
 * generated, and published at addresses this driver owns. Every byte the hardware
 * fetches from either is produced from the vendor tuning file, except gamma's
 * second page and the static half of the DRC page, which are not in that file in
 * any form and are carried as decoded curves. Everything else still runs on
 * replayed vendor addresses. That matters because slot B is RAM-booted from a
 * streaming slot A, so those pages hold the vendor's own tables and an unfilled
 * buffer is not obviously wrong; on a cold boot it would be. Formats are in
 * ar-isp-codec.h.
 *
 * The AE statistics buffers are owned here too. The zone grid, its second
 * smaller-window instance and the Bayer histogram are allocated and published
 * at addresses this driver owns, so their contents can be read; debugfs "stats"
 * decodes the grid. Layouts are in ar-isp-stats.h.
 *
 * Not implemented here:
 *
 *  - The per-frame loop. The vendor re-arms the statistics buffer addresses and
 *    runs three indirect-port transactions on every frame, driven by the VIF
 *    frame-start interrupt that ar-vif.c already owns. The output planes are
 *    programmed once during setup and are not part of that loop, so the first
 *    frame lands without it. Until that loop moves in here, ml-isploop drives
 *    it from userspace and re-arms the vendor's statistics addresses over the
 *    ones published here, so owning them only holds with the cycle disabled.
 *  - Auto-exposure itself. The statistics are readable; nothing consumes them.
 *  - Buffer allocation, geometry other than 1920x1080, and any V4L2 interface.
 *
 * Configuration provenance is in ar-isp-defaults.h. Two thirds of it comes
 * from static per-submodule default blocks in the vendor library and one third
 * from the write trace; applying both reproduces the vendor's final register
 * state exactly. See ../../../../docs/camera-stack.md.
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
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include "ar-isp-defaults.h"
#include "ar-isp-codec.h"
#include "ar-isp-stats.h"
#include "ar-isp-drc-tail.h"
#include "ar-isp-gamma-page1.h"
#include "ar-isp-compander.h"
#include "ar-isp-colour.h"
#include "ar-isp-ccm-init.h"
#include "ar-isp-ladder.h"

/*
 * Master control. The block is brought up by a staged sequence of writes to
 * this register, which is embedded in the setup table in the right places
 * rather than driven separately.
 */
#define AR_ISP_CONTROL			0x0000

/* Input geometry, width in the low half. */
#define AR_ISP_INPUT_SIZE		0x0004

/*
 * Indirect access port. The vendor writes an address to 0x00cc and a value to
 * 0x00d4; three such transactions run in every frame of the per-frame loop.
 * The port is exercised during setup by the table.
 */
#define AR_ISP_INDIRECT_ADDR		0x00cc
#define AR_ISP_INDIRECT_DATA		0x00d4

/* Interrupt status and mask. Read-only use here. */
#define AR_ISP_INTR_STATUS		0x0090
#define AR_ISP_INTR_MASK		0x00b8

/*
 * de3d's working buffers, on its bank 0x2e00. These were called output planes
 * here on the strength of their placeholder values; they are not. The ISP does
 * not write frames at all, CVISP does, and the bank belongs to isp_sub_de3d.
 *
 * Three distinct addresses, each published to two registers, each register
 * written twice as placeholder then address, so they are armed once and never
 * re-armed. The captures rule out raster content: adjacent-byte correlation is
 * +0.03, row correlation is flat from stride 16 to 3840 with no peak at any
 * width, and the autocorrelation peak is at lag 4 with lag 2 anti-correlated,
 * which is a 32-bit record array. What they hold beyond that is not
 * established, so they are named for the module that owns them.
 */
#define AR_ISP_DE3D_BUF0_A		0x2e3c
#define AR_ISP_DE3D_BUF1_A		0x2e44
#define AR_ISP_DE3D_BUF0_B		0x2e58
#define AR_ISP_DE3D_BUF1_B		0x2e60
#define AR_ISP_DE3D_BUF2_A		0x2e80
#define AR_ISP_DE3D_BUF2_B		0x2e88

/*
 * Sizes, and the honest state of them. The vendor packs the three at
 * 0x2b439200, 0x2b614200 and 0x2b703200, so the gaps bound the first two at
 * 0x1db000 and 0xef000: it cannot have used more without overlapping. Nothing
 * sits above the third, so its gap says nothing and the size here is a guess
 * carried at the larger of the two measured bounds.
 *
 * Getting this wrong is a hardware DMA writing past a buffer we own. The
 * isp_cma reservation is no-map, so an overrun stays inside the reserved region
 * and cannot reach kernel memory, but it would quietly corrupt our own tables.
 * A dump of the vendor's layout above 0x2b703200 would settle it.
 */
#define AR_ISP_DE3D_BUF0_SIZE		0x1db000
#define AR_ISP_DE3D_BUF1_SIZE		0xef000
#define AR_ISP_DE3D_BUF2_SIZE		0x1db000

/* Output stage geometry and commit. */
#define AR_ISP_OUT_SIZE			0x2e04
#define AR_ISP_OUT_COMMIT		0x2e90

/*
 * Read-only status reporting what the block sees on its input. Both are the
 * probe for whether the ISP is receiving video at all, and both are more useful
 * than looking at the output buffer, because they report on the input side.
 *
 * AR_ISP_IN_GEOMETRY reads back the measured frame as (height << 16) | width.
 * With the VIF streaming and the ISP otherwise untouched it reads 0x043c0784,
 * 1084 x 1924, matching the VIF's own measurement at VIF 0x1f0. A streaming
 * vendor reads 0x04380780, 1080 x 1920, the active area. Zero means the block
 * is seeing nothing.
 *
 * AR_ISP_IN_LINETIME latches the incoming line time and is static, not a
 * counter: a streaming vendor holds 0x134e across repeated samples while VIF
 * 0x1f8 measures about the same. Zero means no line timing is arriving.
 */
#define AR_ISP_IN_GEOMETRY		0x706c
#define AR_ISP_IN_LINETIME		0x2e98

/*
 * Coefficient table descriptors. Each holds the physical address of a DMA buffer
 * the block fetches when the matching bit is written to AR_ISP_TABLE_COMMIT.
 *
 * The commit is write-to-trigger, not a set-then-clear pulse: clearing the bit
 * afterwards cancels the fetch. Bit 16 is always set alongside. From the vendor
 * trace, which publishes an address and then commits, one table at a time:
 *
 *	0x0020 = 0x2b2e0c00   0x0014 = 0x00010001    compander
 *	0x0060 = 0x2b2e9200   0x0014 = 0x00010010    DRC
 *	0x0030/0x0040/0x0050 = 0x2b2ec600
 *	                      0x0014 = 0x0001000e    gamma, three descriptors
 */
#define AR_ISP_TABLE_COMMIT		0x0014
#define AR_ISP_TABLE_COMMIT_ENABLE	0x00010000
#define AR_ISP_TABLE_COMPANDER		0x0020
#define AR_ISP_TABLE_COMPANDER_BIT	0x00000001
#define AR_ISP_TABLE_GAMMA0		0x0030
#define AR_ISP_TABLE_GAMMA1		0x0040
#define AR_ISP_TABLE_GAMMA2		0x0050
#define AR_ISP_TABLE_GAMMA_BITS		0x0000000e
#define AR_ISP_TABLE_DRC		0x0060
#define AR_ISP_TABLE_DRC_BIT		0x00000010

/*
 * The HDR page and the compander share one allocation, in the vendor's layout.
 *
 * On the vendor these are not independent buffers: the compander sits at
 * 0x2b2e0c00 and the HDR page at 0x2b2e0200, exactly 0xa00 lower, and the HDR
 * descriptor length makes it fetch 0x1000. So its last 0x600 bytes ARE the
 * compander's first 0x600, read because the fetch overruns its own content.
 * Measured on captured pages: zero differing bytes across all 1536.
 *
 * Allocating one block in that relationship reproduces the fetched bytes for
 * both descriptors without copying anything twice, and makes the overlap
 * explicit rather than something a later reader has to rediscover.
 *
 * The compander span is the 0xf000 its length register at 0x0024 implies rather
 * than the 0x7800 the table occupies. The vendor cannot satisfy that either:
 * 0xf000 past its compander runs into the gamma page, so what the block reads
 * beyond the table is its neighbours' memory and cannot be meaningful. The
 * excess is therefore ignored, and allocating it only keeps the DMA inside
 * memory we own.
 */
#define AR_ISP_HDR_SIZE		0x1000
#define AR_ISP_HDR_COMPANDER		0xa00
#define AR_ISP_COMPANDER_ALLOC		0xf000
#define AR_ISP_TONE_ALLOC		(AR_ISP_HDR_COMPANDER + AR_ISP_COMPANDER_ALLOC)

/*
 * The descriptor on bank 0x1c00. Like LSC's it is module-local, with its own
 * valid bit and no 0x0014 commit. Unlike LSC's, the block does not clear it
 * after fetching. Bank 0x1c00 belongs to the vendor's isp_sub_hdr: its attach
 * handler at 0x196900 maps 0x1c00, 0x1f98, 0x1fc0 and 0x1fe0, and the trace's
 * writes in that range stop before hdr_rro_0_stats at 0x1d20. The driver called
 * this GTM2 until that was settled; the vendor's gtm2 is a different module on
 * bank 0x2800. No bytes change with the name: the page is zero over its whole
 * extent on the vendor and this driver writes zero too.
 */
#define AR_ISP_TABLE_HDR		0x1c6c
#define AR_ISP_TABLE_HDR_VALID		0x1c60

/*
 * The addresses the replayed configuration arms, inside the isp_cma reservation.
 * They are the vendor's own buffers, not ours. Reading them is how a table is
 * seeded from whatever the vendor left in DRAM across a RAM-boot.
 */
#define AR_ISP_VENDOR_GAMMA_PHYS	0x2b2ec600
#define AR_ISP_VENDOR_DRC_PHYS		0x2b2e9200
#define AR_ISP_VENDOR_LSC_PHYS		0x2b2e8600

/*
 * LSC descriptor, owned by the vendor's isp_sub_lsc on bank 0x4c00. Unlike
 * compander, DRC and gamma this one does not go through the 0x0014 commit: it
 * is a module-local record with its own valid bit, which the block clears once
 * it has fetched.
 */
#define AR_ISP_TABLE_LSC		0x4c34
#define AR_ISP_TABLE_LSC_VALID		0x4c3c

/*
 * hdr_lsc descriptor, a second shading stage on bank 0x1dd0. Module-local like
 * LSC's: length at 0x1e2c carries the same 0x34 sixteen-byte records, address
 * at 0x1e38, valid at 0x1e40. The replayed configuration arms the vendor's
 * page at 0x2b2e8c00, 0x600 above the LSC page. The stage's true payload
 * source is not established (the fourth tuning grid per illuminant group is a
 * candidate). Filling it with the LSC grid applies shading twice, measured on
 * hardware as blown-out corners, so the unseeded fill is zero.
 */
#define AR_ISP_TABLE_HDR_LSC		0x1e38
#define AR_ISP_TABLE_HDR_LSC_VALID	0x1e40
#define AR_ISP_VENDOR_HDR_LSC_PHYS	0x2b2e8c00

/*
 * Green-imbalance control word on gib's bank 0x2400. Bit 30 bypasses the
 * stage; the vendor's tuning apply takes the flag-zero branch at 0x1bb304
 * (the gate at blob 0x243e4 reads 0) and sets it. No replay table or sweep
 * ever wrote the register, so before this write a cold boot ran on the
 * hardware reset value.
 */
#define AR_ISP_GIB_CTRL			0x2408
#define AR_ISP_GIB_BYPASS		BIT(30)

/*
 * Interrupt status words, W1C, acknowledged by writing the read value back.
 * The vendor's handler (0x1d2c80) acks three context-held pointers; the trace
 * resolves two of them here: 0x00cc and 0x00d4 are written about three times
 * per frame with sparse bit patterns, 0x00d4's isolated 0x100 event is the
 * bit-8 pre-step the vendor's router services first and alone, and on a stack
 * that never acks, both read as the latched union of every value the vendor's
 * acks carried. 0x00e8 mirrors 0x00cc bit for bit and is the candidate third
 * word; the vendor reads it only under a gate that is zero in every working
 * configuration, so it is left alone until that gate is understood.
 */
#define AR_ISP_INTR_STATUS0		0x00cc
#define AR_ISP_INTR_STATUS1		0x00d4
#define AR_ISP_INTR_STATS_EVENT		BIT(8)

/*
 * Statistics buffer addresses, derived from the bank map in ar-isp-stats.h
 * rather than carried as literals. The two RRO engines sit on one bank at a
 * 0x34 stride and rro_face is a third instance of the same block, so all four
 * fall out of a base plus the engine's own address register. They match the
 * addresses the vendor's per-frame cycle writes, which is the check that the
 * bank map is right.
 *
 * These are DMA targets the hardware writes, not tables it fetches, so there is
 * no commit and no valid bit: publishing an address is the whole protocol.
 */
#define AR_ISP_STATS_RRO0	(AR_ISP_RRO_BANK + AR_ISP_RRO_REG_ADDR)
#define AR_ISP_STATS_RRO1	(AR_ISP_RRO_BANK + AR_ISP_RRO_ENGINE_STRIDE + \
				 AR_ISP_RRO_REG_ADDR)
#define AR_ISP_STATS_RRO_FACE	(AR_ISP_RRO_FACE_BANK + AR_ISP_RRO_REG_ADDR)
#define AR_ISP_STATS_HIST	(AR_ISP_HIST_BANK + AR_ISP_HIST_REG_ADDR)

/*
 * Bank 0x2800, shared by the vendor's gtm2 and ltm modules. 0x2808 is the
 * coefficient page the block fetches: 64 tiles of a 128-sample u16 curve at a
 * 0x100 stride, every tile monotonic from 0 to a measured maximum of 1003.
 * 0x280c is the ltm_stats buffer the hardware writes, filling its whole
 * measured 0x80000 extent. Both are plain address publishes on the vendor's
 * per-frame list, with no valid bit.
 *
 * The vendor recomputes the page every frame from ltm_stats. Publishing a
 * fixed identity page forgoes local tone adaptation but is scene-safe, which
 * is what a cold boot needs: the replay otherwise arms the vendor's address,
 * and on a boot where slot A never streamed that memory is junk applied as
 * per-tile tone curves.
 */
#define AR_ISP_LTM_PAGE_ADDR		0x2808
#define AR_ISP_LTM_STATS_ADDR		0x280c
#define AR_ISP_LTM_PAGE_SIZE		0x4000
#define AR_ISP_LTM_STATS_SIZE		0x80000
#define AR_ISP_LTM_TILES		64
#define AR_ISP_LTM_SAMPLES		128
#define AR_ISP_LTM_TILE_STRIDE		0x100
#define AR_ISP_LTM_CURVE_MAX		1003

/*
 * The vendor tuning file, verbatim. Its length is checked because the vendor
 * loader checks it: every offset in ar-isp-codec.h is an absolute position in a
 * fixed-layout record, so a file of a different size is a different format and
 * not something to parse on a best-effort basis.
 */
#define AR_ISP_TUNING_FIRMWARE		"artosyn/nt99235-tuning-preview-fpv.bin"
#define AR_ISP_TUNING_SIZE		0xd6c58

/* The measured correction pass. Off disables it, for an A/B of its effect. */
static bool trim = true;
module_param(trim, bool, 0644);
MODULE_PARM_DESC(trim,
		 "apply the measured correction against the streaming vendor (default on)");

static bool tables = true;
module_param(tables, bool, 0644);
MODULE_PARM_DESC(tables,
		 "own the coefficient DMA buffers instead of arming the vendor's (default on)");

/*
 * Only reachable now when the tuning file is missing or a table is switched off:
 * both gamma and DRC generate every byte the hardware reads. Kept because that
 * fallback is the difference between a degraded camera and no camera, and
 * because turning it off is how a bring-up proves the generation really is
 * standalone rather than quietly leaning on what the vendor left behind.
 */
static bool seed = true;
module_param(seed, bool, 0644);
MODULE_PARM_DESC(seed,
		 "fall back to the vendor's inherited pages when a table cannot be generated (default on)");

/*
 * Which curve to select out of the tuning file. The vendor derives this from AE
 * and interpolates between adjacent curves; we have no AE, so it is pinned.
 *
 * Curve 3 reproduces one of our captures to within 5 counts of 4095 and curve 2
 * reproduces another, which is the AE selection visible directly in our own
 * data. Any pinned value is therefore an operating point, not a correct answer.
 */
static int gamma_curve = 3;
module_param(gamma_curve, int, 0644);
MODULE_PARM_DESC(gamma_curve, "tuning-file gamma curve, 0-4, or -1 to leave the page alone");

static int drc_profile = 3;
module_param(drc_profile, int, 0644);
MODULE_PARM_DESC(drc_profile, "tuning-file DRC profile, or -1 to leave the page alone");

static bool compander = true;
module_param(compander, bool, 0644);
MODULE_PARM_DESC(compander,
		 "own the compander page and fill it from the carried template (default on)");

static bool hdr = true;
module_param(hdr, bool, 0644);
MODULE_PARM_DESC(hdr,
		 "own the HDR page, which shares an allocation with the compander (default on)");

static bool lsc = true;
module_param(lsc, bool, 0644);
MODULE_PARM_DESC(lsc,
		 "own the LSC page and generate its lens-shading grid (default on)");

static bool hdr_lsc = true;
module_param(hdr_lsc, bool, 0644);
MODULE_PARM_DESC(hdr_lsc,
		 "own the hdr_lsc page, zero-filled until its payload is recovered (default on)");

static bool ccm = true;
module_param(ccm, bool, 0644);
MODULE_PARM_DESC(ccm,
		 "own the CCM register banks and pack the tuning matrix (default on)");

static unsigned int ccm_bank;
module_param(ccm_bank, uint, 0644);
MODULE_PARM_DESC(ccm_bank,
		 "which tuning-file illuminant bank to install, 0 to 3 (default 0, the traced one)");

/*
 * Unlike the other ownership switches, this one does not reproduce a value the
 * vendor computes; it hands the hardware three working buffers of our own. The
 * sizes are bounds derived from the gaps between the vendor's allocations
 * rather than measured extents.
 */
static bool de3d = true;
module_param(de3d, bool, 0644);
MODULE_PARM_DESC(de3d,
		 "own de3d's three working buffers instead of the vendor's (default on)");

static bool stats = true;
module_param(stats, bool, 0644);
MODULE_PARM_DESC(stats,
		 "own the AE statistics buffers instead of the vendor's (default on)");

static bool ltm = true;
module_param(ltm, bool, 0644);
MODULE_PARM_DESC(ltm,
		 "own the LTM page and statistics buffer, publishing an identity curve (default on)");

/*
 * The ladder abscissa is 3A state the vendor computes per frame; without an AE
 * loop the operating point stands in for it. 1.0 selects band 0 verbatim,
 * which is byte-identical to the replayed cold bank, so the default changes no
 * register value, only its provenance. The abscissa's physical unit is not the
 * sensor analog multiplier (plans/au-blend-engine-and-notch.md section 2), so
 * no value derived from the gain code is wired up until that unit is pinned.
 */
static int rnr_gain = 256;
module_param(rnr_gain, int, 0644);
MODULE_PARM_DESC(rnr_gain,
		 "rnr ladder abscissa as a Q8 linear gain (default 256 = 1.0, the cold band; -1 leaves the replayed bank alone)");

static bool gib = true;
module_param(gib, bool, 0644);
MODULE_PARM_DESC(gib,
		 "set gib's bypass bit as the vendor's tuning apply does (default on)");

static bool use_irq = true;
module_param(use_irq, bool, 0444);
MODULE_PARM_DESC(use_irq,
		 "service the ISP interrupt: acknowledge the status words and count events (default on; the vendor has no poll mode here at all)");

struct ar_isp {
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data clks[2];
	struct dentry *debugfs;
	bool configured;

	int irq;
	bool irq_requested;
	u32 irq_events;
	u32 irq_stats_events;
	u32 irq_seen0;
	u32 irq_seen1;

	const struct firmware *tuning;
	void *gamma;
	dma_addr_t gamma_dma;
	void *drc;
	dma_addr_t drc_dma;
	void *tone;
	dma_addr_t tone_dma;
	void *lsc;
	dma_addr_t lsc_dma;
	void *hdr_lsc;
	dma_addr_t hdr_lsc_dma;

	/*
	 * Statistics targets. rro is the AE zone grid and both bank-0x6400
	 * engines are pointed at it, reproducing the vendor, which publishes
	 * one buffer to both. rro_face is the second, smaller-window grid and
	 * hist is the Bayer histogram.
	 *
	 * af_stats is deliberately absent: it is disabled on this sensor so
	 * nothing writes its buffer, and its module keeps the vendor's address
	 * rather than being given a guessed allocation.
	 */
	void *rro;
	dma_addr_t rro_dma;
	void *rro_face;
	dma_addr_t rro_face_dma;

	/* Bank 0x2800: the coefficient page at 0x2808 published as a fixed
	 * identity, and the ltm_stats scribble target at 0x280c, whose 0x80000
	 * extent is measured rather than guessed.
	 */
	void *ltm_page;
	dma_addr_t ltm_page_dma;
	void *ltm_stats;
	dma_addr_t ltm_stats_dma;
	void *hist;
	dma_addr_t hist_dma;

	/*
	 * de3d's three working buffers. Hardware-written, armed once, and until
	 * now left pointing at the vendor's own memory across the RAM-boot.
	 */
	void *de3d[3];
	dma_addr_t de3d_dma[3];
};

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
 * Carried verbatim from out/au-mmiotrace/mmio-combined.log rather than sliced out of
 * ar_isp_setup_1080p60: that table was generated from a different trace with consecutive
 * duplicates collapsed, so its ordering does not align here (18 of 69 match at the best
 * offset).
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
	{ 0x75a0, 0x2a660400 },
	{ 0x75bc, 0x2a660400 },
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

static void ar_isp_apply(struct ar_isp *isp, const struct ar_isp_reg *tbl,
			 size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		writel(tbl[i].val, isp->base + tbl[i].off);
}

/*
 * Copy the page the vendor left at a fixed physical address into one of our
 * buffers.
 *
 * This is a transitional crutch and is named as one. It only has anything to
 * copy because slot B is RAM-booted from a slot A whose camera was streaming, so
 * the vendor's tables are still resident in DRAM. On a cold boot these pages hold
 * nothing, which is exactly the dependency owning the buffers is meant to remove.
 * It exists so a first bring-up can change one thing at a time: with seeding on,
 * only the addresses move; with it off, the content is ours too.
 */
static bool ar_isp_seed_from_vendor(struct ar_isp *isp, void *dst,
				    phys_addr_t phys, size_t size)
{
	void *src;

	src = memremap(phys, size, MEMREMAP_WB);
	if (!src) {
		dev_warn(isp->dev, "cannot map vendor page at %pa to seed from\n",
			 &phys);
		return false;
	}

	memcpy(dst, src, size);
	memunmap(src);
	return true;
}

/*
 * Point the statistics engines at buffers this driver owns.
 *
 * The setup table arms the vendor's addresses, inside the vendor's carveout,
 * which is memory we neither allocated nor can read. Re-publishing after the
 * table has run redirects the writes without disturbing anything else: these
 * are plain DMA targets, so an address is the whole protocol and the engines
 * pick it up on the next frame they complete.
 *
 * Both bank-0x6400 engines get the same buffer, because that is what the vendor
 * does and the two captures are byte-identical because of it.
 *
 * Called from ar_isp_tables_apply, so a reconfigure re-points them too. The
 * per-frame cycle re-arms these same registers; until that cycle moves into
 * this driver, ml-isploop keeps writing the vendor's addresses over ours every
 * frame, so stats=1 only holds for a run with the cycle disabled.
 */
/*
 * Point de3d at buffers this driver owns.
 *
 * Each buffer goes to two registers, which is how the vendor arms them: the
 * pair takes one address, not two halves of a range. Published after the setup
 * table for the same reason the statistics buffers are, since that table also
 * carries the vendor's addresses for these.
 *
 * The buffers are handed over zeroed rather than seeded. de3d is temporal, so
 * whatever it accumulates it rebuilds from the frames it sees; starting from
 * zero means the first frames run against an empty history instead of against
 * the vendor's, which is the honest cold-boot behaviour rather than inherited
 * state that happens to look right.
 */
static void ar_isp_de3d_publish(struct ar_isp *isp)
{
	static const u16 reg[3][2] = {
		{ AR_ISP_DE3D_BUF0_A, AR_ISP_DE3D_BUF0_B },
		{ AR_ISP_DE3D_BUF1_A, AR_ISP_DE3D_BUF1_B },
		{ AR_ISP_DE3D_BUF2_A, AR_ISP_DE3D_BUF2_B },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(reg); i++) {
		if (!isp->de3d[i])
			continue;
		writel(lower_32_bits(isp->de3d_dma[i]), isp->base + reg[i][0]);
		writel(lower_32_bits(isp->de3d_dma[i]), isp->base + reg[i][1]);
	}

	if (isp->de3d[0])
		dev_info(isp->dev, "de3d: %pad, %pad, %pad\n",
			 &isp->de3d_dma[0], &isp->de3d_dma[1], &isp->de3d_dma[2]);
}

static void ar_isp_stats_publish(struct ar_isp *isp)
{
	if (isp->rro) {
		writel(lower_32_bits(isp->rro_dma), isp->base + AR_ISP_STATS_RRO0);
		writel(lower_32_bits(isp->rro_dma), isp->base + AR_ISP_STATS_RRO1);
	}

	if (isp->rro_face)
		writel(lower_32_bits(isp->rro_face_dma),
		       isp->base + AR_ISP_STATS_RRO_FACE);

	if (isp->hist)
		writel(lower_32_bits(isp->hist_dma), isp->base + AR_ISP_STATS_HIST);

	if (isp->ltm_page)
		writel(lower_32_bits(isp->ltm_page_dma),
		       isp->base + AR_ISP_LTM_PAGE_ADDR);

	if (isp->ltm_stats)
		writel(lower_32_bits(isp->ltm_stats_dma),
		       isp->base + AR_ISP_LTM_STATS_ADDR);

	if (isp->rro || isp->hist)
		dev_info(isp->dev, "stats: rro %pad, rro_face %pad, hist %pad\n",
			 &isp->rro_dma, &isp->rro_face_dma, &isp->hist_dma);

	if (isp->ltm_page)
		dev_info(isp->dev, "ltm: identity page %pad, stats %pad\n",
			 &isp->ltm_page_dma, &isp->ltm_stats_dma);
}

/*
 * Colour correction. Two register banks, not a DMA page: 0x50 bytes each
 * holding two packed 3x3 matrices, one at +0x00 and a second copy at +0x20.
 *
 * The vendor's sequence is an identity pair installed into ccm1 at init, a
 * fixed matrix pair installed into ccm2, and then the AWB path overwriting
 * ccm1's FIRST copy only with an interpolated tuning-file matrix. ccm2 never
 * moves, because its tuning gate reads 0 in this blob.
 *
 * Doing it here rather than leaving it to the register replay is not just
 * ownership. The replay carries the vendor's runtime matrix, but at entry 1718
 * of the setup table, and a bring-up that applies a prefix shorter than that
 * stops at the identity the earlier entries wrote. The 1475-entry prefix the
 * camera harness uses does exactly that, so every bring-up so far has run with
 * colour correction switched off and nobody noticed, because an identity CCM
 * produces a plausible picture rather than an obviously broken one.
 *
 * Without AWB there is nothing to interpolate between, so the driver installs
 * one illuminant bank verbatim. Bank 0 is what the vendor was traced writing,
 * which makes this reproduce the traced register state exactly rather than
 * approximate it. When AWB exists it selects the bank and blends two of them;
 * the packing does not change.
 */
static void ar_isp_ccm_apply(struct ar_isp *isp)
{
	u8 packed[AR_ISP_CCM_WORDS * 4];
	const u8 *blob;
	unsigned int i;

	if (!ccm)
		return;

	for (i = 0; i < ARRAY_SIZE(ar_isp_ccm1_init); i++)
		writel(ar_isp_ccm1_init[i], isp->base + AR_ISP_CCM1_BANK + i * 4);
	for (i = 0; i < ARRAY_SIZE(ar_isp_ccm2_init); i++)
		writel(ar_isp_ccm2_init[i], isp->base + AR_ISP_CCM2_BANK + i * 4);

	if (!isp->tuning) {
		dev_info(isp->dev, "ccm: init blocks only, no tuning file\n");
		return;
	}

	blob = isp->tuning->data;

	/*
	 * The gate the vendor's AWB path checks before it touches ccm1. If the
	 * blob has it clear, ccm1 keeps the identity, which is the vendor's own
	 * behaviour and not a failure.
	 */
	if (ar_isp_get_le32(blob + AR_ISP_CCM_BLOB_GATE) != 1) {
		dev_info(isp->dev, "ccm: tuning gate clear, ccm1 left at identity\n");
		return;
	}

	if (ccm_bank >= AR_ISP_CCM_BLOB_BANKS_USED) {
		dev_warn(isp->dev, "ccm: bank %u out of range, using 0\n", ccm_bank);
		ccm_bank = 0;
	}

	ar_isp_ccm_from_blob(packed, blob, ccm_bank);

	for (i = 0; i < AR_ISP_CCM_WORDS; i++)
		writel(ar_isp_get_le32(packed + i * 4),
		       isp->base + AR_ISP_CCM1_BANK + i * 4);

	dev_info(isp->dev, "ccm: bank %u packed into ccm1 copy A\n", ccm_bank);
}

/*
 * Noise reduction. The replay carries the vendor's cold bank; this recomputes
 * the twelve ladder registers from the tuning file the way the vendor's rnr
 * driver does on every gain move, so the values are derived rather than
 * replayed. At the default abscissa of 1.0 the result is byte-identical to the
 * replay. The bank control word stays with the replay: its mode bit mirrors
 * the blob's header flag, and both read 0.
 */
static void ar_isp_rnr_apply(struct ar_isp *isp)
{
	u32 regs[AR_ISP_RNR_REGS];
	const u8 *blob;
	unsigned int i;

	if (rnr_gain < 0)
		return;

	if (!isp->tuning) {
		dev_info(isp->dev, "rnr: replayed bank only, no tuning file\n");
		return;
	}

	blob = isp->tuning->data;

	/*
	 * The gate the vendor's rnr driver checks before it recomputes. Clear
	 * means the stage runs on its replayed state, which is the vendor's own
	 * behaviour and not a failure.
	 */
	if (ar_isp_get_le32(blob + AR_ISP_RNR_BLOB_HEADER +
			    AR_ISP_RNR_HDR_ENABLE) != 1) {
		dev_info(isp->dev, "rnr: tuning gate clear, bank left replayed\n");
		return;
	}

	ar_isp_rnr_from_blob(regs, blob, (u32)rnr_gain << 8);

	for (i = 0; i < AR_ISP_RNR_REGS; i++)
		writel(regs[i], isp->base + AR_ISP_RNR_BANK +
		       AR_ISP_RNR_LADDER + i * 4);

	dev_info(isp->dev, "rnr: ladder at gain %d.%03u, 0x%04x..0x%04x = 0x%08x\n",
		 rnr_gain >> 8, (rnr_gain & 0xff) * 1000 / 256,
		 AR_ISP_RNR_BANK + AR_ISP_RNR_LADDER,
		 AR_ISP_RNR_BANK + AR_ISP_RNR_LADDER + (AR_ISP_RNR_REGS - 1) * 4,
		 regs[0]);
}

/*
 * Fill the owned buffers and hand them to the block.
 *
 * Runs after the register replay, which arms the vendor's addresses and commits
 * them. Republishing ours and committing again is the same sequence the vendor
 * itself issues on every AE update, so this is not a special case for the block.
 *
 * Compander has no generator to recover: the vendor installs its 0x7800 page
 * verbatim from a static template in the service library and never recomputes
 * it, so the page is carried and filled here like any other constant.
 */
static void ar_isp_tables_apply(struct ar_isp *isp)
{
	const u8 *blob = isp->tuning ? isp->tuning->data : NULL;
	bool gamma_seeded = false, drc_seeded = false;
	bool gamma_built = false, drc_built = false;
	bool tone_built = false;
	bool lsc_seeded = false, lsc_built = false;
	bool hdr_lsc_seeded = false;
	u8 *page;

	if (!tables)
		return;

	if (isp->gamma) {
		if (seed)
			gamma_seeded = ar_isp_seed_from_vendor(isp, isp->gamma,
							       AR_ISP_VENDOR_GAMMA_PHYS,
							       AR_ISP_GAMMA_SIZE);
		else
			memset(isp->gamma, 0, AR_ISP_GAMMA_SIZE);

		if (blob && gamma_curve >= 0 &&
		    gamma_curve < AR_ISP_GAMMA_BLOB_CURVES) {
			page = isp->gamma;
			ar_isp_gamma_from_blob(page, blob, gamma_curve);

			/*
			 * Page 1 is not an AE selection and is not in the tuning
			 * file: it is a carried constant. Both pages are written
			 * here, so gamma no longer depends on anything inherited.
			 *
			 * Nothing fills the rest of the allocation because nothing
			 * reads it. The descriptor length at 0x0034/0x0044/0x0054
			 * is in units of 32 bytes and the replay leaves it at 0x80,
			 * so the fetch is 0x1000. The allocation stays 0x4000 to
			 * match the vendor's, not because the block needs it.
			 */
			ar_isp_gamma_pack_page(page + AR_ISP_GAMMA_PAGE,
					       ar_isp_gamma_page1,
					       AR_ISP_GAMMA_PAGE1_TAIL);
			gamma_built = true;
		}
	}

	if (isp->drc) {
		if (seed)
			drc_seeded = ar_isp_seed_from_vendor(isp, isp->drc,
							     AR_ISP_VENDOR_DRC_PHYS,
							     AR_ISP_DRC_SIZE);
		else
			memset(isp->drc, 0, AR_ISP_DRC_SIZE);

		if (blob && drc_profile >= 0) {
			/*
			 * The whole page, both halves: the first two banks from
			 * the tuning file, the second two from the constant the
			 * vendor never recomputes. Nothing here is inherited,
			 * which is why the seed above is redundant when this
			 * runs and is left in place only so the two can be
			 * compared in one bring-up.
			 */
			page = isp->drc;
			ar_isp_drc_from_blob(page, blob, drc_profile);
			ar_isp_drc_pack_bank(page + 2 * AR_ISP_DRC_BANK,
					     ar_isp_drc_tail_bank0);
			ar_isp_drc_pack_bank(page + 3 * AR_ISP_DRC_BANK,
					     ar_isp_drc_tail_bank1);
			drc_built = true;
		}
	}

	if (isp->tone) {
		/*
		 * The compander has no seed path and no tuning file: the page is
		 * the same bytes on every unit and in every scene, so there is
		 * nothing to fall back to and nothing to select.
		 *
		 * The HDR page's own 0xa00 is left zero. Its first 0x800 is zero on the
		 * vendor too; the 0x200 after that is scene-varying runtime state,
		 * measured at 117 of 512 bytes differing between two captures, so
		 * it has no stored source to reproduce. Zeroing it was measured on
		 * hardware to move 6.3% of pixels by more than 8 levels, against a
		 * 94.5% frame-to-frame floor from scene motion alone: no effect.
		 */
		memset(isp->tone, 0, AR_ISP_TONE_ALLOC);
		ar_isp_compander_fill(isp->tone + AR_ISP_HDR_COMPANDER,
				      ar_isp_compander_head,
				      ar_isp_compander_mid);
		tone_built = true;
	}

	if (isp->ltm_page) {
		__le16 *page = isp->ltm_page;
		unsigned int t, i;

		/*
		 * Every tile the same linear curve, monotonic from 0 to the
		 * measured maximum, truncating as every quantisation in this
		 * pipeline does. The stride leaves a gap after each tile's 128
		 * samples; the whole page is zeroed first so the gaps match the
		 * vendor's.
		 */
		memset(isp->ltm_page, 0, AR_ISP_LTM_PAGE_SIZE);
		for (t = 0; t < AR_ISP_LTM_TILES; t++)
			for (i = 0; i < AR_ISP_LTM_SAMPLES; i++)
				page[t * AR_ISP_LTM_TILE_STRIDE / 2 + i] =
					cpu_to_le16(i * AR_ISP_LTM_CURVE_MAX /
						    (AR_ISP_LTM_SAMPLES - 1));
	}

	if (isp->ltm_stats)
		memset(isp->ltm_stats, 0, AR_ISP_LTM_STATS_SIZE);

	if (isp->lsc) {
		/*
		 * Only region A, the lens-shading grid, is generated. The
		 * scene-adaptive 0x2c0 after it has no stored source anywhere and
		 * is left to the seed, so with seeding off it is zero and the
		 * block runs on shading alone.
		 */
		if (seed)
			lsc_seeded = ar_isp_seed_from_vendor(isp, isp->lsc,
							     AR_ISP_VENDOR_LSC_PHYS,
							     AR_ISP_LSC_SIZE);
		else
			memset(isp->lsc, 0, AR_ISP_LSC_SIZE);

		if (blob) {
			ar_isp_lsc_from_blob(isp->lsc, blob);
			lsc_built = true;
		}
	}

	if (isp->hdr_lsc) {
		/*
		 * The page is owned but not built: filling it with the LSC grid
		 * applies shading twice and was measured on hardware to blow out
		 * the corners. The stage's real payload is unrecovered, so the
		 * fill is zero until it is.
		 */
		if (seed)
			hdr_lsc_seeded = ar_isp_seed_from_vendor(isp, isp->hdr_lsc,
								 AR_ISP_VENDOR_HDR_LSC_PHYS,
								 AR_ISP_LSC_SIZE);
		else
			memset(isp->hdr_lsc, 0, AR_ISP_LSC_SIZE);
	}

	/*
	 * The buffers are coherent, so there is no cache to flush, but the writes
	 * above must be visible before the address that makes the block fetch
	 * them.
	 */
	wmb();

	/*
	 * The descriptors are 32-bit. dma_alloc_coherent is bounded by the mask
	 * set in ar_isp_tables_prepare, so this cannot truncate, but the cast is
	 * written out rather than left implicit.
	 */
	if (isp->gamma) {
		u32 addr = lower_32_bits(isp->gamma_dma);

		writel(addr, isp->base + AR_ISP_TABLE_GAMMA0);
		writel(addr, isp->base + AR_ISP_TABLE_GAMMA1);
		writel(addr, isp->base + AR_ISP_TABLE_GAMMA2);
		writel(AR_ISP_TABLE_COMMIT_ENABLE | AR_ISP_TABLE_GAMMA_BITS,
		       isp->base + AR_ISP_TABLE_COMMIT);
	}

	if (isp->drc) {
		writel(lower_32_bits(isp->drc_dma), isp->base + AR_ISP_TABLE_DRC);
		writel(AR_ISP_TABLE_COMMIT_ENABLE | AR_ISP_TABLE_DRC_BIT,
		       isp->base + AR_ISP_TABLE_COMMIT);
	}

	if (isp->tone) {
		writel(lower_32_bits(isp->tone_dma + AR_ISP_HDR_COMPANDER),
		       isp->base + AR_ISP_TABLE_COMPANDER);
		writel(AR_ISP_TABLE_COMMIT_ENABLE | AR_ISP_TABLE_COMPANDER_BIT,
		       isp->base + AR_ISP_TABLE_COMMIT);

		if (hdr) {
			writel(lower_32_bits(isp->tone_dma),
			       isp->base + AR_ISP_TABLE_HDR);
			writel(1, isp->base + AR_ISP_TABLE_HDR_VALID);
		}
	}

	if (isp->lsc) {
		writel(lower_32_bits(isp->lsc_dma), isp->base + AR_ISP_TABLE_LSC);
		writel(1, isp->base + AR_ISP_TABLE_LSC_VALID);
	}

	if (isp->hdr_lsc) {
		writel(lower_32_bits(isp->hdr_lsc_dma),
		       isp->base + AR_ISP_TABLE_HDR_LSC);
		writel(1, isp->base + AR_ISP_TABLE_HDR_LSC_VALID);
	}

	ar_isp_stats_publish(isp);
	ar_isp_de3d_publish(isp);

	dev_info(isp->dev,
		 "tables: gamma %pad %s, drc %pad %s, compander %pad %s, lsc %pad %s\n",
		 &isp->gamma_dma,
		 gamma_built ? "built" : (gamma_seeded ? "seeded" : "zeroed"),
		 &isp->drc_dma,
		 drc_built ? "built" : (drc_seeded ? "seeded" : "zeroed"),
		 &isp->tone_dma,
		 tone_built ? "hdr+compander built" : "on the vendor's page",
		 &isp->lsc_dma,
		 lsc_built ? "shading built" : (lsc_seeded ? "seeded" : "zeroed"));

	if (isp->hdr_lsc)
		dev_info(isp->dev, "hdr_lsc: %pad %s\n", &isp->hdr_lsc_dma,
			 hdr_lsc_seeded ? "seeded" : "zeroed");
}

/*
 * Allocate the owned buffers and load the tuning file.
 *
 * Both are optional: a failure here leaves the replayed vendor addresses armed,
 * which is what the driver did before it owned anything, so the camera still
 * comes up. The tuning file is not in the repository and has to be installed on
 * the device; without it the buffers are still ours but carry only seeded data.
 */
static void ar_isp_tables_prepare(struct ar_isp *isp)
{
	struct device *dev = isp->dev;
	int ret;

	ret = of_reserved_mem_device_init(dev);
	if (ret) {
		dev_warn(dev, "no isp memory pool (%d), not owning tables\n", ret);
		return;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_warn(dev, "no 32-bit dma mask (%d), not owning tables\n", ret);
		return;
	}

	isp->gamma = dma_alloc_coherent(dev, AR_ISP_GAMMA_SIZE, &isp->gamma_dma,
					GFP_KERNEL);
	isp->drc = dma_alloc_coherent(dev, AR_ISP_DRC_SIZE, &isp->drc_dma,
				      GFP_KERNEL);
	if (compander)
		isp->tone = dma_alloc_coherent(dev, AR_ISP_TONE_ALLOC,
					       &isp->tone_dma, GFP_KERNEL);
	if (lsc)
		isp->lsc = dma_alloc_coherent(dev, AR_ISP_LSC_SIZE,
					      &isp->lsc_dma, GFP_KERNEL);
	if (hdr_lsc)
		isp->hdr_lsc = dma_alloc_coherent(dev, AR_ISP_LSC_SIZE,
						  &isp->hdr_lsc_dma, GFP_KERNEL);
	if (!isp->gamma || !isp->drc || (compander && !isp->tone) ||
	    (lsc && !isp->lsc) || (hdr_lsc && !isp->hdr_lsc))
		dev_warn(dev, "coefficient buffers unavailable, falling back to the vendor's\n");

	if (stats) {
		isp->rro = dma_alloc_coherent(dev, AR_ISP_RRO_SIZE,
					      &isp->rro_dma, GFP_KERNEL);
		isp->rro_face = dma_alloc_coherent(dev, AR_ISP_RRO_SIZE,
						   &isp->rro_face_dma, GFP_KERNEL);
		isp->hist = dma_alloc_coherent(dev, AR_ISP_HIST_SIZE,
					       &isp->hist_dma, GFP_KERNEL);
		if (!isp->rro || !isp->rro_face || !isp->hist)
			dev_warn(dev, "statistics buffers unavailable, falling back to the vendor's\n");
	}

	if (ltm) {
		isp->ltm_page = dma_alloc_coherent(dev, AR_ISP_LTM_PAGE_SIZE,
						   &isp->ltm_page_dma,
						   GFP_KERNEL);
		isp->ltm_stats = dma_alloc_coherent(dev, AR_ISP_LTM_STATS_SIZE,
						    &isp->ltm_stats_dma,
						    GFP_KERNEL);
		if (!isp->ltm_page || !isp->ltm_stats)
			dev_warn(dev, "ltm buffers unavailable, falling back to the vendor's\n");
	}

	if (de3d) {
		static const size_t sz[3] = {
			AR_ISP_DE3D_BUF0_SIZE, AR_ISP_DE3D_BUF1_SIZE,
			AR_ISP_DE3D_BUF2_SIZE,
		};
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(sz); i++)
			isp->de3d[i] = dma_alloc_coherent(dev, sz[i],
							  &isp->de3d_dma[i],
							  GFP_KERNEL);

		if (!isp->de3d[0] || !isp->de3d[1] || !isp->de3d[2])
			dev_warn(dev, "de3d buffers unavailable, falling back to the vendor's\n");
	}

	ret = request_firmware(&isp->tuning, AR_ISP_TUNING_FIRMWARE, dev);
	if (ret) {
		dev_warn(dev, "no %s (%d), tables cannot be generated\n",
			 AR_ISP_TUNING_FIRMWARE, ret);
		isp->tuning = NULL;
		return;
	}

	if (isp->tuning->size != AR_ISP_TUNING_SIZE) {
		dev_warn(dev, "%s is %zu bytes, expected %u; ignoring it\n",
			 AR_ISP_TUNING_FIRMWARE, isp->tuning->size,
			 AR_ISP_TUNING_SIZE);
		release_firmware(isp->tuning);
		isp->tuning = NULL;
	}
}

static void ar_isp_tables_release(struct ar_isp *isp)
{
	if (isp->gamma)
		dma_free_coherent(isp->dev, AR_ISP_GAMMA_SIZE, isp->gamma,
				  isp->gamma_dma);
	if (isp->drc)
		dma_free_coherent(isp->dev, AR_ISP_DRC_SIZE, isp->drc,
				  isp->drc_dma);
	if (isp->tone)
		dma_free_coherent(isp->dev, AR_ISP_TONE_ALLOC, isp->tone,
				  isp->tone_dma);
	if (isp->lsc)
		dma_free_coherent(isp->dev, AR_ISP_LSC_SIZE, isp->lsc,
				  isp->lsc_dma);
	if (isp->hdr_lsc)
		dma_free_coherent(isp->dev, AR_ISP_LSC_SIZE, isp->hdr_lsc,
				  isp->hdr_lsc_dma);
	if (isp->rro)
		dma_free_coherent(isp->dev, AR_ISP_RRO_SIZE, isp->rro,
				  isp->rro_dma);
	if (isp->rro_face)
		dma_free_coherent(isp->dev, AR_ISP_RRO_SIZE, isp->rro_face,
				  isp->rro_face_dma);
	if (isp->hist)
		dma_free_coherent(isp->dev, AR_ISP_HIST_SIZE, isp->hist,
				  isp->hist_dma);
	if (isp->ltm_page)
		dma_free_coherent(isp->dev, AR_ISP_LTM_PAGE_SIZE, isp->ltm_page,
				  isp->ltm_page_dma);
	if (isp->ltm_stats)
		dma_free_coherent(isp->dev, AR_ISP_LTM_STATS_SIZE,
				  isp->ltm_stats, isp->ltm_stats_dma);
	{
		static const size_t sz[3] = {
			AR_ISP_DE3D_BUF0_SIZE, AR_ISP_DE3D_BUF1_SIZE,
			AR_ISP_DE3D_BUF2_SIZE,
		};
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(sz); i++)
			if (isp->de3d[i])
				dma_free_coherent(isp->dev, sz[i], isp->de3d[i],
						  isp->de3d_dma[i]);
	}
	release_firmware(isp->tuning);
}

/*
 * Apply the whole configuration. The recovered table goes first: those are
 * registers that have a static default in the vendor library but which the
 * vendor never writes, because it pushes its shadow image with a
 * write-only-if-changed primitive and they already held the right value. They
 * are absent from any trace by construction, so nothing else would set them.
 *
 * The setup table then runs in write order. Order matters: it contains the
 * staged master enable and several arm-then-load registers whose result
 * depends on the sequence. It carries no timing, only order.
 *
 * The vendor interleaves CSI-2 and VIF writes into this sequence. Those blocks
 * are configured by their own drivers here, so only the ISP writes are
 * replayed and the interleaving is assumed not to matter. If the block turns
 * out to need a VIF or CSI-2 register at a particular point, this is where it
 * would show up.
 */
/*
 * The lnr strength curves the 1475-entry prefix truncates. lnr carries four
 * 64-entry byte curves from 0x3d60 at stride 0x40, normalised to 0x40, and the
 * prefix cut leaves everything past 0x3d7c zero, which is gain zero rather
 * than neutral. Values are the streaming vendor's, read from the register
 * sweep; the state diff classifies all 35 as reachable static configuration,
 * not AE-moved state.
 */
static const struct ar_isp_reg ar_isp_lnr_fix[] = {
	{ 0x3d44, 0xfffdf800 },
	{ 0x3d60, 0x00000000 },
	{ 0x3d80, 0x40404040 },
	{ 0x3d84, 0x40404040 },
	{ 0x3d88, 0x40404040 },
	{ 0x3d8c, 0x40404040 },
	{ 0x3d90, 0x40404040 },
	{ 0x3d94, 0x40404040 },
	{ 0x3d98, 0x40404040 },
	{ 0x3d9c, 0x40404040 },
	{ 0x3da8, 0x40404040 },
	{ 0x3dac, 0x40404040 },
	{ 0x3db0, 0x40404040 },
	{ 0x3db4, 0x40404040 },
	{ 0x3db8, 0x40404040 },
	{ 0x3dbc, 0x40404040 },
	{ 0x3dc0, 0x40404040 },
	{ 0x3dc4, 0x40404040 },
	{ 0x3dc8, 0x40404040 },
	{ 0x3dcc, 0x40404040 },
	{ 0x3dd0, 0x40404040 },
	{ 0x3dd4, 0x40404040 },
	{ 0x3dd8, 0x40404040 },
	{ 0x3ddc, 0x40404040 },
	{ 0x3df4, 0x40404040 },
	{ 0x3df8, 0x40404040 },
	{ 0x3dfc, 0x40404040 },
	{ 0x3e00, 0x40404040 },
	{ 0x3e04, 0x40404040 },
	{ 0x3e08, 0x40404040 },
	{ 0x3e0c, 0x40404040 },
	{ 0x3e10, 0x40404040 },
	{ 0x3e14, 0x40404040 },
	{ 0x3e18, 0x38404040 },
	{ 0x3e1c, 0x0c182430 },
};

static void ar_isp_configure(struct ar_isp *isp)
{
	ar_isp_apply(isp, ar_isp_recovered, ARRAY_SIZE(ar_isp_recovered));
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
		ar_isp_apply(isp, ar_isp_vendor_trim,
			     ARRAY_SIZE(ar_isp_vendor_trim));

	ar_isp_apply(isp, ar_isp_output_fix, ARRAY_SIZE(ar_isp_output_fix));
	ar_isp_apply(isp, ar_isp_lnr_fix, ARRAY_SIZE(ar_isp_lnr_fix));

	/*
	 * After the replay, which arms the vendor's buffer addresses. Ours
	 * replace them.
	 */
	ar_isp_tables_apply(isp);

	isp->configured = true;

	dev_info(isp->dev,
		 "configured: %zu recovered + %zu ordered + %zu trim + %zu output fix, control 0x%08x\n",
		 ARRAY_SIZE(ar_isp_recovered),
		 ARRAY_SIZE(ar_isp_setup_1080p60),
		 trim ? ARRAY_SIZE(ar_isp_vendor_trim) : 0,
		 ARRAY_SIZE(ar_isp_output_fix),
		 readl(isp->base + AR_ISP_CONTROL));
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
	if (s1 & AR_ISP_INTR_STATS_EVENT)
		isp->irq_stats_events++;

	return IRQ_HANDLED;
}

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
	ar_isp_rnr_apply(isp);

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
 * stream between probes rather than issuing several prefixes in a row. The trim
 * pass is deliberately not applied here: it would reintroduce the writes being
 * bisected.
 */
static void ar_isp_configure_prefix(struct ar_isp *isp, size_t n)
{
	if (n > ARRAY_SIZE(ar_isp_setup_1080p60))
		n = ARRAY_SIZE(ar_isp_setup_1080p60);

	ar_isp_apply(isp, ar_isp_recovered, ARRAY_SIZE(ar_isp_recovered));
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
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ar_isp_dump_regs); i++)
		seq_printf(s, "%-16s 0x%04x 0x%08x\n", ar_isp_dump_regs[i].name,
			   ar_isp_dump_regs[i].off,
			   readl(isp->base + ar_isp_dump_regs[i].off));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ar_isp_regs);

/*
 * The AE zone grid, decoded.
 *
 * Prints the per-zone luma mean as a 36-column by 16-row map, plus the frame
 * mean and the count the sums were accumulated over. The count is read from the
 * buffer rather than computed, because it is a function of the programmed
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
	unsigned int col, row;
	u64 total = 0;
	u32 count;

	if (!isp->rro) {
		seq_puts(s, "not owned\n");
		return 0;
	}

	count = ar_isp_rro_count(isp->rro, 0);
	seq_printf(s, "count %u per zone per channel\n", count);

	for (row = 0; row < AR_ISP_RRO_ROWS; row++) {
		for (col = 0; col < AR_ISP_RRO_COLS; col++) {
			u32 g0 = ar_isp_rro_mean(isp->rro, col, row, 1);
			u32 g1 = ar_isp_rro_mean(isp->rro, col, row, 2);
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

static int ar_isp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ar_isp *isp;
	int ret;

	isp = devm_kzalloc(dev, sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;

	isp->dev = dev;

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

	ar_isp_tables_prepare(isp);

	isp->debugfs = debugfs_create_dir("ar-isp", NULL);
	debugfs_create_file_unsafe("configure", 0600, isp->debugfs, isp,
				   &ar_isp_configure_fops);
	debugfs_create_file("regs", 0400, isp->debugfs, isp, &ar_isp_regs_fops);
	debugfs_create_file("stats", 0400, isp->debugfs, isp, &ar_isp_stats_fops);
	/*
	 * Reading this reports the table size, so a bisect script can discover
	 * the upper bound without hardcoding it.
	 */
	debugfs_create_file_unsafe("configure_upto", 0600, isp->debugfs, isp,
				   &ar_isp_prefix_fops);
	debugfs_create_file_unsafe("arm", 0600, isp->debugfs, isp,
				   &ar_isp_arm_fops);
	debugfs_create_u32("irq_events", 0400, isp->debugfs, &isp->irq_events);
	debugfs_create_u32("irq_stats_events", 0400, isp->debugfs,
			   &isp->irq_stats_events);
	debugfs_create_x32("irq_seen0", 0400, isp->debugfs, &isp->irq_seen0);
	debugfs_create_x32("irq_seen1", 0400, isp->debugfs, &isp->irq_seen1);

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
		 ARRAY_SIZE(ar_isp_recovered) +
		 ARRAY_SIZE(ar_isp_setup_1080p60));

	return 0;
}

static void ar_isp_remove(struct platform_device *pdev)
{
	struct ar_isp *isp = platform_get_drvdata(pdev);

	/* Before anything else: once the clocks drop, a late interrupt's
	 * register access hangs the SoC, so the line must be gone first.
	 */
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
