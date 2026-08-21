// SPDX-License-Identifier: GPL-2.0
/*
 * ar-isp-tables.c - the coefficient and statistics buffers the ISP fetches.
 *
 * Everything the block reads out of DRAM rather than out of a register: the
 * gamma and DRC coefficient pages, the compander and LSC pages, de3d's three
 * working buffers, and the AE statistics targets. This file allocates them,
 * fills them from the vendor tuning file, publishes their addresses to the
 * hardware, and releases them.
 *
 * Every byte the hardware fetches from gamma or DRC is produced from the tuning
 * file, except gamma's second page and the static half of the DRC page, which
 * are not in that file in any form and are carried as decoded curves. Formats
 * are in ar-isp-codec.h; statistics layouts are in ar-isp-stats.h.
 *
 * The register stages that read a ladder out of the tuning blob live here too,
 * because they are recomputed from the same blob on the same gain moves as the
 * pages are: rnr, lnr, de3d, cfa, cnf, cm, cm2, ccm and dpc.
 *
 * ar-isp-main.c owns probe, configure, the interrupt, debugfs and remove, and
 * calls into this file through the declarations in ar-isp-priv.h.
 */

#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/seq_file.h>
#include <linux/string.h>

#include "ar-isp-priv.h"
#include "ar-isp-codec.h"
#include "ar-isp-dpc.h"
#include "ar-isp-stats.h"
#include "ar-isp-regs.h"
#include "vendor-tables/ar-isp-drc-tail.h"
#include "vendor-tables/ar-isp-gamma-page1.h"
#include "vendor-tables/ar-isp-compander.h"
#include "ar-isp-colour.h"
#include "vendor-tables/ar-isp-ccm-init.h"
#include "ar-isp-rnr.h"
#include "vendor-tables/ar-isp-rgb2yuv.h"
#include "ar-isp-lnr.h"
#include "ar-isp-de3d.h"
#include "ar-isp-de3d-geom.h"
#include "ar-isp-cfa.h"
#include "ar-isp-cnf.h"
#include "ar-isp-cm.h"
#include "ar-isp-cm2.h"
#include "ar-isp-tone.h"

/*
 * Only reachable now when the tuning file is missing or a table is switched off:
 * both gamma and DRC generate every byte the hardware reads. Kept because that
 * fallback is the difference between a degraded camera and no camera, and
 * because turning it off is how a bring-up proves the generation really is
 * standalone.
 */
static bool seed = true;
module_param(seed, bool, 0644);
MODULE_PARM_DESC(seed,
		 "fall back to the vendor's inherited pages when a table cannot be generated (default on)");

/*
 * The tone pages. The vendor derives both from one AE scalar, which selects a
 * low and a high entry out of the stage's band table and a weight to blend
 * them by; a scalar inside a band selects that entry alone. Neither page is
 * therefore always one of the tuning file's entries, and a driver that can only
 * pin an index cannot reach the ones between.
 *
 * `tone_scalar` is the vendor's path: give it the AEC trigger scalar in Q8 and
 * both stages select and blend as the vendor does. It is off by default because
 * the scalar's producer is not recovered yet; `plans/isp-tone-selector.md` has
 * the measurements and the standing candidate.
 *
 * With it off, `gamma_curve` and `drc_profile` pin one entry each with no
 * blend, which is what a forced sweep needs.
 *
 * The four defaults below are one operating point expressed two ways, and they
 * have to stay that way. AR_ISP_TRIGGER_DEFAULT is the scalar the vendor's own
 * traced cm and cm2 state measures; gamma curve 3 spans [280, 330] and DRC
 * profile 4 spans [290, 380], so that scalar selects both verbatim with no
 * blend, and the pinned pair is the same picture the scalar path would build.
 *
 * A pinned pair that no scalar can select is not a conservative default, it is
 * an unreachable one: curve 3 against profile 3 was such a pair, because
 * profile 3 ends at 270 and curve 3 does not start until 280.
 */
#define AR_ISP_TRIGGER_DEFAULT		(291 * 256)

static int gamma_curve = 3;
module_param(gamma_curve, int, 0644);
MODULE_PARM_DESC(gamma_curve, "tuning-file gamma curve, 0-4, or -1 to leave the page alone; ignored when tone_scalar is set");

static int drc_profile = 4;
module_param(drc_profile, int, 0644);
MODULE_PARM_DESC(drc_profile, "tuning-file DRC profile, 0-5, or -1 to leave the page alone; ignored when tone_scalar is set");

static int tone_scalar = -1;
module_param(tone_scalar, int, 0644);
MODULE_PARM_DESC(tone_scalar,
		 "AEC trigger scalar in Q8, driving gamma and DRC selection and blending as the vendor does; -1 pins gamma_curve and drc_profile instead");

static bool compander = true;
module_param(compander, bool, 0644);
MODULE_PARM_DESC(compander,
		 "own the compander page and fill it from the carried template (default on)");

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

static bool dpc = true;
module_param(dpc, bool, 0644);
MODULE_PARM_DESC(dpc,
		 "apply the carried vendor defect-correction payload over the replay (default on)");

static bool ltm = true;
module_param(ltm, bool, 0644);
MODULE_PARM_DESC(ltm,
		 "own the LTM page and statistics buffer, publishing an identity curve (default on)");

/*
 * Which of the four colour-space matrices the library carries to install. The
 * default is the one measured on the streaming vendor, so it changes no
 * register value, only where the value comes from.
 *
 * The other three are the levers this makes reachable: full range puts black at
 * 0 and white at 255, limited range at 16 and 235, and BT.709 is the usual
 * choice at HD where the vendor picked BT.601. Anything but the default is a
 * departure from the vendor and has to be judged end to end, because the
 * receiving side has to agree.
 */
static int csc = AR_ISP_CSC_DEFAULT;
module_param(csc, int, 0644);
MODULE_PARM_DESC(csc,
		 "colour-space matrix index: 0 bt601_full (the vendor's), 1 bt709_full, 2 bt601_limited, 3 bt709_limited; -1 leaves the replayed bank alone");

/*
 * The ladder abscissa is 3A state the vendor computes per frame: the commanded
 * gain, the Q8 exposure-table value of the AE's selected entry divided by 256.
 * The loop drives these parameters at frame rate through the ladders hook; the
 * default of 1.0 is the operating point a boot starts from, and it selects band
 * 0 verbatim, which is byte-identical to the replayed cold bank, so the default
 * changes no register value, only its provenance.
 */
static int rnr_gain = 256;
module_param(rnr_gain, int, 0644);
MODULE_PARM_DESC(rnr_gain,
		 "rnr ladder abscissa as a Q8 linear gain (default 256 = 1.0, the cold band; -1 leaves the replayed bank alone)");

static int lnr_gain = 256;
module_param(lnr_gain, int, 0644);
MODULE_PARM_DESC(lnr_gain,
		 "lnr ladder abscissa as a Q8 linear gain (default 256 = 1.0, the cold band; -1 leaves the replayed bank alone)");

static int de3d_gain = 256;
module_param(de3d_gain, int, 0644);
MODULE_PARM_DESC(de3d_gain,
		 "de3d ladder abscissa as a Q8 linear gain (default 256 = 1.0, the cold band; -1 leaves the replayed bank alone)");

/*
 * The de3d user strength knobs, the vendor's two-field pair (privctx+764/+772,
 * transform at libmpp_service.so 0x1c69f4). 0..100, 50 is neutral and exactly
 * identity, so the default changes no register and matches every capture. The
 * 14-bit field saturates toward 200 at 100 and the 9-bit field toward 120; the
 * vendor never moves either on the shipped path. See ar_isp_de3d_strength.
 */
static int de3d_strength = 50;
module_param(de3d_strength, int, 0644);
MODULE_PARM_DESC(de3d_strength,
		 "de3d 14-bit denoise field strength, 0..100 (50 = neutral, the shipped state)");

static int de3d_strength9 = 50;
module_param(de3d_strength9, int, 0644);
MODULE_PARM_DESC(de3d_strength9,
		 "de3d 9-bit denoise field strength, 0..100 (50 = neutral, the shipped state)");

static int cfa_gain = 256;
module_param(cfa_gain, int, 0644);
MODULE_PARM_DESC(cfa_gain,
		 "cfa ladder abscissa as a Q8 linear gain (default 256 = 1.0, the cold band; -1 leaves the replayed bank alone)");

static int cnf_gain = 256;
module_param(cnf_gain, int, 0644);
MODULE_PARM_DESC(cnf_gain,
		 "cnf ladder abscissa as a Q8 linear gain (default 256 = 1.0, the cold band; -1 leaves the replayed bank alone)");

/*
 * cm and cm2 are NOT keyed on the gain the five stages above use. The vendor's
 * trigger payload carries two scalars and each stage picks one from a flag in
 * its own tuning header, so the axes say plainly which is which: rnr, lnr,
 * de3d, cfa and cnf all key on a powers-of-two ladder over 1 to 2048, a linear
 * gain, while cm and cm2 key on 0 to 550, the same axis the gamma curve and DRC
 * profile bands use. That is the AEC trigger scalar of plans/isp-tone-selector.md,
 * whose producer is not recovered yet.
 *
 * Driving them off the gain reproduces neither: at the two captured operating
 * points the gain selects row 0 where the vendor sat on row 1, which would
 * install cm 0x483c = 36 against the vendor's 33.
 *
 * Both default to AR_ISP_TRIGGER_DEFAULT rather than to -1. -1 leaves the
 * replayed bank, and that bank is NOT the vendor's running state: the setup
 * replay applies a 1475-entry prefix and these stages' runtime values sit past
 * the cut, the same way ccm1's matrix does. Measured on the air unit, the
 * replayed bank holds cm 0x483c = 41 where the streaming vendor holds 33, and
 * cm2's bounds read 16/32/512/768 against the vendor's 1/2/1006/1023. Deriving
 * at the default scalar installs the vendor's values on all of them.
 */
static int cm_trigger = AR_ISP_TRIGGER_DEFAULT;
module_param(cm_trigger, int, 0644);
MODULE_PARM_DESC(cm_trigger,
		 "cm ladder abscissa as a Q8 AEC trigger scalar, NOT a gain (0 to 550 axis); -1 leaves the replayed bank alone");

static int cm2_trigger = AR_ISP_TRIGGER_DEFAULT;
module_param(cm2_trigger, int, 0644);
MODULE_PARM_DESC(cm2_trigger,
		 "cm2 ladder abscissa as a Q8 AEC trigger scalar, NOT a gain (0 to 550 axis); -1 leaves the replayed bank alone");

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
	void *src = memremap(phys, size, MEMREMAP_WB);

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
 * pair takes the same full address. Published after the setup
 * table for the same reason the statistics buffers are, since that table also
 * carries the vendor's addresses for these.
 *
 * The buffers are handed over zeroed. de3d is temporal, so whatever it
 * accumulates it rebuilds from the frames it sees; the first frames run against
 * an empty history, which is cold-boot behaviour.
 */
/*
 * ar_isp_de3d_geom_apply - the thirteen registers de3d derives from geometry.
 *
 * Separate from the ladder because it is not gain-keyed: the vendor computes
 * these once per configuration from the frame and the sensor line length, and
 * they do not read the tuning file. Read-modify-write, because several share a
 * word with the submodule's static image.
 *
 * The padded width is taken from the block's own 0x2e04 rather than assumed,
 * so a geometry change the driver makes elsewhere carries through here without
 * a second source of truth. See ar-isp-de3d-geom.h and
 * kernel/scripts/isp/check-de3d-geometry.py.
 */
void ar_isp_de3d_geom_apply(struct ar_isp *isp, bool verbose)
{
	u32 regs[AR_ISP_DE3D_GEOM_REGS];
	u32 size = readl(isp->base + AR_ISP_DE3D_BANK + 0x04);
	u32 width = size >> 16;
	u32 height = size & 0x1fff;

	if (!width || !height) {
		if (verbose)
			dev_info(isp->dev,
				 "de3d: geometry register reads 0x%08x, pass skipped\n",
				 size);

		return;
	}

	ar_isp_de3d_geom_pack(regs, width, height, AR_ISP_DE3D_HTS,
			      AR_ISP_DE3D_WORK_MODE);

	for (unsigned int i = 0; i < AR_ISP_DE3D_GEOM_REGS; i++) {
		u32 off = AR_ISP_DE3D_BANK + ar_isp_de3d_geom_regs[i].off;
		u32 mask = ar_isp_de3d_geom_regs[i].mask;
		u32 v = readl(isp->base + off);

		writel((v & ~mask) | (regs[i] & mask), isp->base + off);
	}

	if (verbose)
		dev_info(isp->dev,
			 "de3d: geometry %ux%u hts %u, %u registers derived\n",
			 width, height, AR_ISP_DE3D_HTS,
			 AR_ISP_DE3D_GEOM_REGS);
}

void ar_isp_de3d_publish(struct ar_isp *isp)
{
	static const u16 reg[3][2] = {
		{ AR_ISP_DE3D_BUF0_A, AR_ISP_DE3D_BUF0_B },
		{ AR_ISP_DE3D_BUF1_A, AR_ISP_DE3D_BUF1_B },
		{ AR_ISP_DE3D_BUF2_A, AR_ISP_DE3D_BUF2_B },
	};

	for (unsigned int i = 0; i < ARRAY_SIZE(reg); i++) {
		if (!isp->de3d[i])
			continue;

		writel(lower_32_bits(isp->de3d_dma[i]), isp->base + reg[i][0]);
		writel(lower_32_bits(isp->de3d_dma[i]), isp->base + reg[i][1]);
	}

	if (isp->de3d[0])
		dev_info(isp->dev, "de3d: %pad, %pad, %pad\n",
			 &isp->de3d_dma[0], &isp->de3d_dma[1], &isp->de3d_dma[2]);
}

/*
 * Point the statistics engines at one half of each buffer. Register writes
 * only: this runs in hard interrupt context on every frame, so it must not log,
 * sleep or touch anything the reader also writes.
 *
 * The two 0x6400 engines are given the same address, which is what the vendor
 * publishes, so both halves stay a single pair.
 */
void ar_isp_stats_arm(struct ar_isp *isp, unsigned int half)
{
	if (isp->rro) {
		dma_addr_t a = isp->rro_dma + half * AR_ISP_RRO_SIZE;

		writel(lower_32_bits(a), isp->base + AR_ISP_STATS_RRO0);
		writel(lower_32_bits(a), isp->base + AR_ISP_STATS_RRO1);
	}

	if (isp->rro_face)
		writel(lower_32_bits(isp->rro_face_dma + half * AR_ISP_RRO_SIZE),
		       isp->base + AR_ISP_STATS_RRO_FACE);

	if (isp->hist)
		writel(lower_32_bits(isp->hist_dma + half * AR_ISP_HIST_SIZE),
		       isp->base + AR_ISP_STATS_HIST);
}

/* The half a consumer may read, and whether reading it means anything yet. */
const void *ar_isp_stats_ready(struct ar_isp *isp, const void *buf,
				      size_t size)
{
	if (!buf)
		return NULL;

	if (!pingpong)
		return buf;

	if (!READ_ONCE(isp->stats_valid))
		return NULL;

	return (const u8 *)buf + READ_ONCE(isp->stats_done) * size;
}

void ar_isp_stats_publish(struct ar_isp *isp)
{
	unsigned long flags;

	/*
	 * Against the interrupt, which arms the same three address registers and
	 * advances the same indices on every frame. Reachable from configure and
	 * from output arm, either of which can run with the line already live.
	 * The LTM descriptors below are outside the lock: nothing else writes
	 * them.
	 */
	spin_lock_irqsave(&isp->stats_lock, flags);
	isp->stats_cur = 0;
	isp->stats_done = 0;
	isp->stats_valid = false;
	ar_isp_stats_arm(isp, isp->stats_cur);
	spin_unlock_irqrestore(&isp->stats_lock, flags);

	if (isp->ltm_page)
		writel(lower_32_bits(isp->ltm_page_dma +
				     isp->ltm_page_half * AR_ISP_LTM_PAGE_SIZE),
		       isp->base + AR_ISP_LTM_PAGE_ADDR);

	if (isp->ltm_stats)
		writel(lower_32_bits(isp->ltm_stats_dma),
		       isp->base + AR_ISP_LTM_STATS_ADDR);

	/*
	 * The mesh pass count the vendor's set_format leaves behind. The static
	 * image carries zero here and the streaming vendor reads 4, which is the
	 * only configuration difference across this whole bank.
	 */
	writel(AR_ISP_LTM_MESH_PASS_SECOND, isp->base + AR_ISP_LTM_MESH_PASSES);

	if (isp->rro || isp->hist)
		dev_info(isp->dev, "stats: rro %pad, rro_face %pad, hist %pad\n",
			 &isp->rro_dma, &isp->rro_face_dma, &isp->hist_dma);

	if (isp->ltm_page)
		dev_info(isp->dev, "ltm: identity page %pad, stats %pad\n",
			 &isp->ltm_page_dma, &isp->ltm_stats_dma);
}

/*
 * Colour correction. Two register banks of 0x50 bytes each,
 * holding two packed 3x3 matrices, one at +0x00 and a second copy at +0x20.
 *
 * The vendor's sequence is an identity pair installed into ccm1 at init, a
 * fixed matrix pair installed into ccm2, and then the AWB path overwriting
 * ccm1's FIRST copy only with an interpolated tuning-file matrix. ccm2 never
 * moves, because its tuning gate reads 0 in this blob.
 *
 * Doing it here is more than ownership. The replay carries the vendor's runtime matrix, but at entry 1718
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
void ar_isp_ccm_apply(struct ar_isp *isp)
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
 * Defect pixel correction. The payload is a register image the vendor copies
 * wholesale out of init_config, so it is carried verbatim; see
 * ar-isp-dpc.h for the two defects in the replay it supersedes.
 */
void ar_isp_dpc_apply(struct ar_isp *isp)
{
	if (!dpc)
		return;

	ar_isp_apply(isp, ar_isp_dpc_payload, ARRAY_SIZE(ar_isp_dpc_payload));
}

/*
 * Noise reduction. The replay carries the vendor's cold bank; this recomputes
 * the twelve ladder registers and the twenty-two that follow them from the
 * tuning file the way the vendor's rnr driver does on every gain move, so the
 * values are derived rather than replayed. At the default abscissa of 1.0 the
 * result is byte-identical to the replay, and the tail stays identical to it
 * up to 8.09 because the fields it packs are the same in the first four bands.
 * Past that six of the tail registers move, which is what the replay could not
 * follow. The bank control word stays with the replay: its mode bit mirrors
 * the blob's header flag, and both read 0.
 */
void ar_isp_rnr_apply(struct ar_isp *isp, bool verbose)
{
	u32 regs[AR_ISP_RNR_REGS];
	u32 tail[AR_ISP_RNR_TAIL_REGS];
	const u8 *blob;

	if (rnr_gain < 0)
		return;

	if (!isp->tuning) {
		if (verbose)
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
			    AR_ISP_LADDER_HDR_ENABLE) != 1) {
		if (verbose)
			dev_info(isp->dev, "rnr: tuning gate clear, bank left replayed\n");
		return;
	}

	ar_isp_rnr_from_blob(regs, blob, (u32)rnr_gain << 8);
	ar_isp_rnr_tail_from_blob(tail, blob, (u32)rnr_gain << 8);

	for (unsigned int i = 0; i < AR_ISP_RNR_REGS; i++)
		writel(regs[i], isp->base + AR_ISP_RNR_BANK +
		       AR_ISP_RNR_LADDER + i * 4);

	for (unsigned int i = 0; i < AR_ISP_RNR_TAIL_REGS; i++)
		writel(tail[i], isp->base + AR_ISP_RNR_BANK +
		       AR_ISP_RNR_TAIL + i * 4);

	if (verbose)
		dev_info(isp->dev, "rnr: ladder at gain %d.%03u, 0x%04x..0x%04x = 0x%08x\n",
			 rnr_gain >> 8, (rnr_gain & 0xff) * 1000 / 256,
			 AR_ISP_RNR_BANK + AR_ISP_RNR_LADDER,
			 AR_ISP_RNR_BANK + AR_ISP_RNR_LADDER + (AR_ISP_RNR_REGS - 1) * 4,
			 regs[0]);
}

/*
 * Local noise reduction. The vendor recomputes this bank from the lnr ladder
 * on the same gain moves as rnr. Register 0x3d10 stays with the replay because
 * the vendor packer never writes it.
 */
void ar_isp_lnr_apply(struct ar_isp *isp, bool verbose)
{
	u32 regs[AR_ISP_LNR_REGS];
	const u8 *blob;
	unsigned int i;

	if (lnr_gain < 0)
		return;

	if (!isp->tuning) {
		if (verbose)
			dev_info(isp->dev, "lnr: replayed bank only, no tuning file\n");

		return;
	}

	blob = isp->tuning->data;

	if (ar_isp_get_le32(blob + AR_ISP_LNR_BLOB_HEADER +
			    AR_ISP_LADDER_HDR_ENABLE) != 1) {
		if (verbose)
			dev_info(isp->dev, "lnr: tuning gate clear, bank left replayed\n");

		return;
	}

	for (i = 0; i < AR_ISP_LNR_REGS; i++)
		regs[i] = readl(isp->base + AR_ISP_LNR_BANK + i * 4);

	ar_isp_lnr_from_blob(regs, blob, (u32)lnr_gain << 8);

	for (i = 0; i < AR_ISP_LNR_REGS; i++) {
		if (i == AR_ISP_LNR_SKIP_NEVER_WRITTEN)
			continue;

		writel(regs[i], isp->base + AR_ISP_LNR_BANK + i * 4);
	}

	if (verbose)
		dev_info(isp->dev, "lnr: ladder at gain %d.%03u, 0x%04x..0x%04x = 0x%08x\n",
			 lnr_gain >> 8, (lnr_gain & 0xff) * 1000 / 256,
			 AR_ISP_LNR_BANK,
			 AR_ISP_LNR_BANK + (AR_ISP_LNR_REGS - 1) * 4,
			 regs[0]);
}

/*
 * Temporal denoise. The vendor recomputes this bank from the de3d ladder on
 * the same gain moves as rnr and lnr; the frozen replay ran the temporal
 * blend with one exposure's thresholds at every exposure (the black smears
 * around moving objects) and with the pre-streaming control and strength
 * state (the block mosaics on motion). Registers whose bits belong to other
 * producers are read-modify-written under the ladder mask; the buffer
 * addresses stay with ar_isp_de3d_publish, and 0x2e98 is the block's
 * hardware line-time latch, never written.
 */
void ar_isp_de3d_apply(struct ar_isp *isp, bool verbose)
{
	u32 regs[AR_ISP_DE3D_REGS];
	const u8 *blob;

	if (de3d_gain < 0)
		return;

	if (!isp->tuning) {
		if (verbose)
			dev_info(isp->dev, "de3d: replayed bank only, no tuning file\n");

		return;
	}

	blob = isp->tuning->data;
	if (ar_isp_get_le32(blob + AR_ISP_DE3D_BLOB_HEADER +
			    AR_ISP_LADDER_HDR_ENABLE) != 1) {
		if (verbose)
			dev_info(isp->dev, "de3d: tuning gate clear, bank left replayed\n");

		return;
	}

	ar_isp_de3d_from_blob(regs, blob, (u32)de3d_gain << 8);

	/*
	 * The user strength knobs ride on top of the ladder: reg 0x2e1c (regs[4])
	 * is the whole 14-bit field, and the 9-bit field lives in bits 16..24 of
	 * 0x2e20 (regs[5]). Clamped to the recovered 0..100; 50 is a no-op.
	 */
	if (de3d_strength >= 0 && de3d_strength <= 100 && de3d_strength != 50)
		regs[4] = ar_isp_de3d_strength(regs[4] & 0x3fff,
					       de3d_strength, 200) & 0x3fff;

	if (de3d_strength9 >= 0 && de3d_strength9 <= 100 && de3d_strength9 != 50) {
		u32 f = ar_isp_de3d_strength((regs[5] >> 16) & 0x1ff,
					     de3d_strength9, 120) & 0x1ff;

		regs[5] = (regs[5] & ~(0x1ffu << 16)) | (f << 16);
	}

	for (unsigned int i = 0; i < AR_ISP_DE3D_REGS; i++) {
		u32 off = AR_ISP_DE3D_BANK + ar_isp_de3d_regs[i].off;
		u32 mask = ar_isp_de3d_regs[i].mask;
		u32 v = readl(isp->base + off);

		writel((v & ~mask) | (regs[i] & mask), isp->base + off);
	}

	if (verbose)
		dev_info(isp->dev, "de3d: ladder at gain %d.%03u, %u registers in 0x%04x..0x%04x\n",
			 de3d_gain >> 8, (de3d_gain & 0xff) * 1000 / 256,
			 AR_ISP_DE3D_REGS,
			 AR_ISP_DE3D_BANK + ar_isp_de3d_regs[0].off,
			 AR_ISP_DE3D_BANK + ar_isp_de3d_regs[AR_ISP_DE3D_REGS - 1].off);
}

/*
 * Demosaic. The vendor recomputes this bank from the cfa ladder on the same
 * gain moves as rnr, lnr and de3d. The whole payload record lands in 41 of the
 * bank's registers as four ascending runs; the registers the runs step over
 * keep their replayed values, and 0x0800 shares its word with bits owned
 * elsewhere, so it is read-modify-written under the ladder mask.
 */
void ar_isp_cfa_apply(struct ar_isp *isp, bool verbose)
{
	u32 regs[AR_ISP_CFA_REGS];
	const u8 *blob;
	unsigned int run, k, i = 0;

	if (cfa_gain < 0)
		return;

	if (!isp->tuning) {
		if (verbose)
			dev_info(isp->dev, "cfa: replayed bank only, no tuning file\n");

		return;
	}

	blob = isp->tuning->data;
	if (ar_isp_get_le32(blob + AR_ISP_CFA_BLOB_HEADER +
			    AR_ISP_LADDER_HDR_ENABLE) != 1) {
		if (verbose)
			dev_info(isp->dev, "cfa: tuning gate clear, bank left replayed\n");

		return;
	}

	ar_isp_cfa_from_blob(regs, blob, (u32)cfa_gain << 8);

	for (run = 0; run < AR_ISP_CFA_RUNS; run++) {
		for (k = 0; k < ar_isp_cfa_runs[run].count; k++, i++) {
			u32 off = AR_ISP_CFA_BANK + ar_isp_cfa_runs[run].reg +
				  k * 4;

			if (off == AR_ISP_CFA_BANK) {
				ar_isp_rmw(isp, off, AR_ISP_CFA_HEAD_MASK,
					   regs[i]);
				continue;
			}

			writel(regs[i], isp->base + off);
		}
	}

	if (verbose)
		dev_info(isp->dev, "cfa: ladder at gain %d.%03u, %u registers in 0x%04x..0x%04x\n",
			 cfa_gain >> 8, (cfa_gain & 0xff) * 1000 / 256,
			 AR_ISP_CFA_REGS, AR_ISP_CFA_BANK,
			 AR_ISP_CFA_BANK + ar_isp_cfa_runs[AR_ISP_CFA_RUNS - 1].reg +
			 (ar_isp_cfa_runs[AR_ISP_CFA_RUNS - 1].count - 1) * 4);
}

/*
 * Chroma noise filter. Three registers carry ladder output, all keyed on one
 * strength: the strength itself and two reciprocal-square normalisations of it.
 * Each shares its word with bits owned elsewhere, so each write is a
 * read-modify-write under its own mask. This ladder carries no header, so there
 * is no tuning gate to read: the module parameter is the only gate.
 */
void ar_isp_cnf_apply(struct ar_isp *isp, bool verbose)
{
	u32 strength;

	if (cnf_gain < 0)
		return;

	if (!isp->tuning) {
		if (verbose)
			dev_info(isp->dev, "cnf: replayed bank only, no tuning file\n");

		return;
	}

	strength = ar_isp_cnf_strength_from_blob(isp->tuning->data,
						 (u32)cnf_gain << 8);

	ar_isp_rmw(isp, AR_ISP_CNF_STRENGTH_REG, AR_ISP_CNF_STRENGTH_MASK,
		   ar_isp_cnf_pack(strength));
	ar_isp_rmw(isp, AR_ISP_CNF_NORM_REG_A, AR_ISP_CNF_NORM_MASK,
		   ar_isp_cnf_norm_pack(strength) | AR_ISP_CNF_NORM_A_BIT);
	ar_isp_rmw(isp, AR_ISP_CNF_NORM_REG_B, AR_ISP_CNF_NORM_B_MASK,
		   ar_isp_cnf_norm_pack(AR_ISP_CNF_NORM_CONST_B));

	for (unsigned int i = 0; i < AR_ISP_CNF_STATIC_REGS; i++) {
		u32 mask;
		u32 val = ar_isp_cnf_static_pack(isp->tuning->data, i, &mask);

		ar_isp_rmw(isp, AR_ISP_CNF_STATIC_REG + i * 4, mask, val);
	}

	if (verbose)
		dev_info(isp->dev, "cnf: ladder at gain %d.%03u, strength %u, 0x%04x/0x%04x/0x%04x\n",
			 cnf_gain >> 8, (cnf_gain & 0xff) * 1000 / 256, strength,
			 AR_ISP_CNF_STRENGTH_REG, AR_ISP_CNF_NORM_REG_A,
			 AR_ISP_CNF_NORM_REG_B);
}

/*
 * Colour manipulation. The vendor recomputes cm and cm2 on the AE trigger from
 * their own two-dimensional tables in the tuning file. cm currently uses the
 * known CT column 0, which is one of the two columns that encode the measured
 * live row; cm2 has one live CT column in this blob. Both stages pack their
 * gain into a low-seven-bit field and preserve the surrounding static image.
 *
 * The abscissa is the AEC trigger scalar, not the gain the noise ladders use;
 * see the cm_trigger comment above. The captured registers measure it: cm2's
 * installed lo2 of 1006 is an interpolation between the blob's 1022 and 1000,
 * and reproducing it together with cm's row-1 field needs a scalar in
 * [290.47, 291.81]. Both of those come from the same capture as the gamma
 * curve 3 and DRC profile 4 pages, whose bands allow [290, 330], so the four
 * stages agree on one scalar and the colour pair measures it 40 times more
 * tightly than the tone pages do.
 */
void ar_isp_cm_apply(struct ar_isp *isp, bool verbose)
{
	u32 field;
	const u8 *blob;

	if (cm_trigger < 0)
		return;

	if (!isp->tuning) {
		if (verbose)
			dev_info(isp->dev, "cm: replayed bank only, no tuning file\n");

		return;
	}

	blob = isp->tuning->data;
	if (ar_isp_get_le32(blob + AR_ISP_CM_HEADER) != 1) {
		if (verbose)
			dev_info(isp->dev, "cm: tuning gate clear, bank left replayed\n");

		return;
	}

	field = ar_isp_cm_gain_field_from_blob(blob, (u32)cm_trigger, 0);
	ar_isp_rmw(isp, AR_ISP_CM_GAIN_REG, AR_ISP_CM_GAIN_MASK, field);

	if (verbose)
		dev_info(isp->dev, "cm: ladder at trigger %d.%03u, field %u\n",
			 cm_trigger >> 8, (cm_trigger & 0xff) * 1000 / 256, field);
}

void ar_isp_cm2_apply(struct ar_isp *isp, bool verbose)
{
	struct ar_isp_cm2_row row;
	const u8 *blob;

	if (cm2_trigger < 0)
		return;

	if (!isp->tuning) {
		if (verbose)
			dev_info(isp->dev, "cm2: replayed bank only, no tuning file\n");

		return;
	}

	blob = isp->tuning->data;
	if (ar_isp_get_le32(blob + AR_ISP_CM2_HEADER) != 1) {
		if (verbose)
			dev_info(isp->dev, "cm2: tuning gate clear, bank left replayed\n");

		return;
	}

	ar_isp_cm2_from_blob(&row, blob, (u32)cm2_trigger, 0);

	ar_isp_rmw(isp, AR_ISP_CM2_GAIN_REG, AR_ISP_CM2_GAIN_MASK,
		   row.gain_field);
	writel(row.lo1, isp->base + AR_ISP_CM2_LO1_REG);
	writel(row.hi1, isp->base + AR_ISP_CM2_HI1_REG);
	writel(row.lo2, isp->base + AR_ISP_CM2_LO2_REG);
	writel(row.hi2, isp->base + AR_ISP_CM2_HI2_REG);
	writel(row.recip1, isp->base + AR_ISP_CM2_RECIP1_REG);
	writel(row.recip2, isp->base + AR_ISP_CM2_RECIP2_REG);

	if (verbose)
		dev_info(isp->dev,
			 "cm2: ladder at trigger %d.%03u, field %u, lo2 %u, recip2 %u\n",
			 cm2_trigger >> 8, (cm2_trigger & 0xff) * 1000 / 256,
			 row.gain_field, row.lo2, row.recip2);
}

/*
 * The colour-space matrix, from the four the vendor library carries.
 *
 * Not gain-keyed and not 3A-driven: the vendor picks one matrix at set_csc time
 * and never moves it, so this runs once per configure alongside the rest of the
 * static state. Owning it replaces six replayed words with the coefficients they
 * were packed from, and makes the choice of matrix a parameter instead of an
 * accident of which capture the replay came from.
 *
 * The levels consequence is the reason the alternatives are carried rather than
 * dropped: the full-range matrices put black at 0 and white at 255, the limited
 * ones at 16 and 235, and that decision reaches everything downstream of the
 * ISP including the encoder and the DVR file.
 */
void ar_isp_rgb2yuv_apply(struct ar_isp *isp)
{
	const struct ar_isp_csc *m;

	if (csc < 0)
		return;

	if (csc >= (int)ARRAY_SIZE(ar_isp_csc_matrices)) {
		dev_warn(isp->dev, "csc=%d out of range, leaving the bank replayed\n",
			 csc);
		return;
	}

	m = &ar_isp_csc_matrices[csc];

	for (unsigned int i = 0; i < AR_ISP_RGB2YUV_REGS; i++)
		writel(m->regs[i], isp->base + AR_ISP_RGB2YUV_BANK + i * 4);
}

/*
 * Recompute the gain-keyed banks from the tuning file.
 *
 * Must run after every register replay, not only at output arm. The measured
 * correction pass carries 60 of its 101 entries on registers these stages
 * derive: rnr's whole ladder, 22 of lnr's bank, 24 of de3d's and 2 of cnf's,
 * each frozen at the abscissa its capture was taken at. A configure that stops
 * after the replay leaves those banks holding the recording, which is the state
 * the derivation exists to replace, and the divergence is invisible until the
 * gain moves off the captured point.
 *
 * Replay first, derive last, which is the order output arm already used and the
 * order the vendor's own apply follows.
 */
void ar_isp_ladders_apply(struct ar_isp *isp, bool verbose)
{
	ar_isp_rnr_apply(isp, verbose);
	ar_isp_lnr_apply(isp, verbose);
	ar_isp_de3d_apply(isp, verbose);
	ar_isp_cfa_apply(isp, verbose);
	ar_isp_cnf_apply(isp, verbose);
	ar_isp_cm_apply(isp, verbose);
	ar_isp_cm2_apply(isp, verbose);
}

/*
 * One register of the ladder parity oracle: what the hardware holds, what a
 * fresh recomputation asks for, and the mask the stage writes under.
 *
 * Only the masked bits are compared. Every ladder register the appliers
 * read-modify-write shares its word with another producer, so the bits outside
 * the mask carry state this derivation does not own and a difference there is
 * not a divergence.
 */
static void ar_isp_ladder_cmp(struct seq_file *s, const char *stage, u32 off,
			      u32 live, u32 want, u32 mask, unsigned int *bad)
{
	bool ok = ((live ^ want) & mask) == 0;

	if (!ok)
		(*bad)++;

	seq_printf(s, "%-5s 0x%04x 0x%08x 0x%08x 0x%08x %s\n", stage, off, live,
		   want, mask, ok ? "ok" : "MISMATCH");
}

/*
 * Report why a stage contributes no registers. A parameter of -1, a clear
 * tuning gate and a missing tuning file all leave the replayed bank in place on
 * purpose, so none of them is a failure.
 */
static void ar_isp_ladder_skip(struct seq_file *s, const char *stage,
			       const char *why)
{
	seq_printf(s, "%-5s %s\n", stage, why);
}

/*
 * The gain-keyed banks, read back and compared against a recomputation from the
 * tuning file at the parameter each stage currently carries.
 *
 * A parity oracle rather than a control: nothing here writes a register, so the
 * node measures whether the arithmetic that ran on this hardware agrees with the
 * arithmetic the host models reproduce from the same tuning file. Set a stage's
 * gain parameter, latch it through the ladders node, and read this.
 *
 * lnr is seeded from the live bank because that is what its applier does. Its
 * untouched fields therefore compare equal by construction and only the fields
 * it packs are measured, which is the same coverage the host cross-check has.
 *
 * cm and cm2 are keyed on the AEC trigger scalar rather than the noise ladders'
 * gain, so they answer to cm_trigger and cm2_trigger.
 */
int ar_isp_ladders_show(struct seq_file *s, struct ar_isp *isp)
{
	unsigned int bad = 0;
	const u8 *blob;

	seq_printf(s, "%-5s %-6s %-10s %-10s %-10s %s\n", "stage", "off",
		   "live", "want", "mask", "verdict");

	if (!isp->tuning) {
		seq_puts(s, "no tuning file: every bank is the replay\n");

		return 0;
	}

	blob = isp->tuning->data;

	if (rnr_gain < 0) {
		ar_isp_ladder_skip(s, "rnr", "parameter -1, bank left replayed");
	} else if (ar_isp_get_le32(blob + AR_ISP_RNR_BLOB_HEADER +
				   AR_ISP_LADDER_HDR_ENABLE) != 1) {
		ar_isp_ladder_skip(s, "rnr", "tuning gate clear, bank left replayed");
	} else {
		u32 regs[AR_ISP_RNR_REGS];
		u32 tail[AR_ISP_RNR_TAIL_REGS];

		ar_isp_rnr_from_blob(regs, blob, (u32)rnr_gain << 8);
		ar_isp_rnr_tail_from_blob(tail, blob, (u32)rnr_gain << 8);

		for (unsigned int i = 0; i < AR_ISP_RNR_REGS; i++) {
			u32 off = AR_ISP_RNR_BANK + AR_ISP_RNR_LADDER + i * 4;

			ar_isp_ladder_cmp(s, "rnr", off,
					  readl(isp->base + off), regs[i],
					  U32_MAX, &bad);
		}

		for (unsigned int i = 0; i < AR_ISP_RNR_TAIL_REGS; i++) {
			u32 off = AR_ISP_RNR_BANK + AR_ISP_RNR_TAIL + i * 4;

			ar_isp_ladder_cmp(s, "rnr", off,
					  readl(isp->base + off), tail[i],
					  U32_MAX, &bad);
		}
	}

	if (lnr_gain < 0) {
		ar_isp_ladder_skip(s, "lnr", "parameter -1, bank left replayed");
	} else if (ar_isp_get_le32(blob + AR_ISP_LNR_BLOB_HEADER +
				   AR_ISP_LADDER_HDR_ENABLE) != 1) {
		ar_isp_ladder_skip(s, "lnr", "tuning gate clear, bank left replayed");
	} else {
		u32 regs[AR_ISP_LNR_REGS];
		u32 live[AR_ISP_LNR_REGS];

		for (unsigned int i = 0; i < AR_ISP_LNR_REGS; i++) {
			live[i] = readl(isp->base + AR_ISP_LNR_BANK + i * 4);
			regs[i] = live[i];
		}

		ar_isp_lnr_from_blob(regs, blob, (u32)lnr_gain << 8);

		for (unsigned int i = 0; i < AR_ISP_LNR_REGS; i++) {
			if (i == AR_ISP_LNR_SKIP_NEVER_WRITTEN)
				continue;

			ar_isp_ladder_cmp(s, "lnr", AR_ISP_LNR_BANK + i * 4,
					  live[i], regs[i], U32_MAX, &bad);
		}
	}

	if (de3d_gain < 0) {
		ar_isp_ladder_skip(s, "de3d", "parameter -1, bank left replayed");
	} else if (ar_isp_get_le32(blob + AR_ISP_DE3D_BLOB_HEADER +
				   AR_ISP_LADDER_HDR_ENABLE) != 1) {
		ar_isp_ladder_skip(s, "de3d", "tuning gate clear, bank left replayed");
	} else {
		u32 regs[AR_ISP_DE3D_REGS];

		ar_isp_de3d_from_blob(regs, blob, (u32)de3d_gain << 8);

		for (unsigned int i = 0; i < AR_ISP_DE3D_REGS; i++) {
			u32 off = AR_ISP_DE3D_BANK + ar_isp_de3d_regs[i].off;

			ar_isp_ladder_cmp(s, "de3d", off,
					  readl(isp->base + off), regs[i],
					  ar_isp_de3d_regs[i].mask, &bad);
		}
	}

	if (cfa_gain < 0) {
		ar_isp_ladder_skip(s, "cfa", "parameter -1, bank left replayed");
	} else if (ar_isp_get_le32(blob + AR_ISP_CFA_BLOB_HEADER +
				   AR_ISP_LADDER_HDR_ENABLE) != 1) {
		ar_isp_ladder_skip(s, "cfa", "tuning gate clear, bank left replayed");
	} else {
		u32 regs[AR_ISP_CFA_REGS];
		unsigned int i = 0;

		ar_isp_cfa_from_blob(regs, blob, (u32)cfa_gain << 8);

		for (unsigned int run = 0; run < AR_ISP_CFA_RUNS; run++) {
			for (unsigned int k = 0; k < ar_isp_cfa_runs[run].count;
			     k++, i++) {
				u32 off = AR_ISP_CFA_BANK +
					  ar_isp_cfa_runs[run].reg + k * 4;
				u32 mask = off == AR_ISP_CFA_BANK ?
					   AR_ISP_CFA_HEAD_MASK : U32_MAX;

				ar_isp_ladder_cmp(s, "cfa", off,
						  readl(isp->base + off),
						  regs[i], mask, &bad);
			}
		}
	}

	if (cnf_gain < 0) {
		ar_isp_ladder_skip(s, "cnf", "parameter -1, bank left replayed");
	} else {
		u32 strength = ar_isp_cnf_strength_from_blob(blob,
							     (u32)cnf_gain << 8);

		ar_isp_ladder_cmp(s, "cnf", AR_ISP_CNF_STRENGTH_REG,
				  readl(isp->base + AR_ISP_CNF_STRENGTH_REG),
				  ar_isp_cnf_pack(strength),
				  AR_ISP_CNF_STRENGTH_MASK, &bad);
		ar_isp_ladder_cmp(s, "cnf", AR_ISP_CNF_NORM_REG_A,
				  readl(isp->base + AR_ISP_CNF_NORM_REG_A),
				  ar_isp_cnf_norm_pack(strength) |
				  AR_ISP_CNF_NORM_A_BIT,
				  AR_ISP_CNF_NORM_MASK, &bad);
		ar_isp_ladder_cmp(s, "cnf", AR_ISP_CNF_NORM_REG_B,
				  readl(isp->base + AR_ISP_CNF_NORM_REG_B),
				  ar_isp_cnf_norm_pack(AR_ISP_CNF_NORM_CONST_B),
				  AR_ISP_CNF_NORM_B_MASK, &bad);
	}

	if (cm_trigger < 0) {
		ar_isp_ladder_skip(s, "cm", "parameter -1, bank left replayed");
	} else if (ar_isp_get_le32(blob + AR_ISP_CM_HEADER) != 1) {
		ar_isp_ladder_skip(s, "cm", "tuning gate clear, bank left replayed");
	} else {
		ar_isp_ladder_cmp(s, "cm", AR_ISP_CM_GAIN_REG,
				  readl(isp->base + AR_ISP_CM_GAIN_REG),
				  ar_isp_cm_gain_field_from_blob(blob,
								 (u32)cm_trigger,
								 0),
				  AR_ISP_CM_GAIN_MASK, &bad);
	}

	if (cm2_trigger < 0) {
		ar_isp_ladder_skip(s, "cm2", "parameter -1, bank left replayed");
	} else if (ar_isp_get_le32(blob + AR_ISP_CM2_HEADER) != 1) {
		ar_isp_ladder_skip(s, "cm2", "tuning gate clear, bank left replayed");
	} else {
		struct ar_isp_cm2_row row;
		static const u32 offs[] = {
			AR_ISP_CM2_LO1_REG, AR_ISP_CM2_HI1_REG,
			AR_ISP_CM2_LO2_REG, AR_ISP_CM2_HI2_REG,
			AR_ISP_CM2_RECIP1_REG, AR_ISP_CM2_RECIP2_REG,
		};
		u32 vals[ARRAY_SIZE(offs)];

		static const u32 masks[] = {
			U32_MAX, U32_MAX, U32_MAX, U32_MAX,
			AR_ISP_CM2_RECIP_MASK, AR_ISP_CM2_RECIP_MASK,
		};

		ar_isp_cm2_from_blob(&row, blob, (u32)cm2_trigger, 0);

		vals[0] = row.lo1;
		vals[1] = row.hi1;
		vals[2] = row.lo2;
		vals[3] = row.hi2;
		vals[4] = row.recip1;
		vals[5] = row.recip2;

		ar_isp_ladder_cmp(s, "cm2", AR_ISP_CM2_GAIN_REG,
				  readl(isp->base + AR_ISP_CM2_GAIN_REG),
				  row.gain_field, AR_ISP_CM2_GAIN_MASK, &bad);

		for (unsigned int i = 0; i < ARRAY_SIZE(offs); i++)
			ar_isp_ladder_cmp(s, "cm2", offs[i],
					  readl(isp->base + offs[i]), vals[i],
					  masks[i], &bad);
	}

	seq_printf(s, "\n%u mismatches\n", bad);

	return 0;
}

/*
 * Which entries each tone stage builds from, and how they blend.
 *
 * One helper for both the scalar path and the pinned one, so a forced sweep
 * exercises the same builder the vendor path uses. A pinned index is the
 * degenerate blend: low equals high and the weight is zero.
 *
 * Returns false when the stage should be left alone, which is what a pinned
 * index of -1 asks for.
 */
static bool ar_isp_tone_gamma_pick(const u8 *blob, struct ar_isp_tone_pick *pick)
{
	if (tone_scalar >= 0) {
		ar_isp_tone_pick_gamma(pick, blob, (u32)tone_scalar);

		return true;
	}

	if (gamma_curve < 0 || gamma_curve >= AR_ISP_GAMMA_BLOB_CURVES)
		return false;

	pick->low = gamma_curve;
	pick->high = gamma_curve;
	pick->t_q12 = 0;

	return true;
}

static bool ar_isp_tone_drc_pick(const u8 *blob, struct ar_isp_tone_pick *pick)
{
	if (tone_scalar >= 0) {
		ar_isp_tone_pick_drc(pick, blob, (u32)tone_scalar);

		return true;
	}

	if (drc_profile < 0 || drc_profile >= AR_ISP_DRC_BLOB_PROFILES)
		return false;

	pick->low = drc_profile;
	pick->high = drc_profile;
	pick->t_q12 = 0;

	return true;
}

/*
 * No selection. A stage carrying this either has never been filled or was
 * filled from something other than a band selection, and must be rebuilt before
 * its page can be trusted. No real pick can collide with it: low and high are
 * band indices and t_q12 is a 12-bit weight.
 */
static const struct ar_isp_tone_pick ar_isp_tone_none = { ~0u, ~0u, ~0u };

/*
 * Whether a rebuild would change anything.
 *
 * The loop writes the scalar every decision but the selection only moves when
 * it crosses a band edge, which on the vendor's own cadence is a small minority
 * of frames. Rebuilding a page the block is fetching has a cost and a hazard,
 * so the comparison lives here, where the selection is known, rather than in
 * userspace, which would need the blob's band tables to make the same call.
 *
 * Compares against what the pages hold rather than against the last call, so a
 * configure that refilled them behind this path is seen. The recording is the
 * caller's, next to the fill it describes.
 */
static bool ar_isp_tone_changed(const struct ar_isp *isp,
				const struct ar_isp_tone_pick *gamma,
				const struct ar_isp_tone_pick *drc, bool force)
{
	return force ||
	       memcmp(&isp->last_gamma, gamma, sizeof(*gamma)) ||
	       memcmp(&isp->last_drc, drc, sizeof(*drc));
}

void ar_isp_tone_apply(struct ar_isp *isp, bool force, bool verbose)
{
	const u8 *blob = isp->tuning ? isp->tuning->data : NULL;
	struct ar_isp_tone_pick gamma = ar_isp_tone_none;
	struct ar_isp_tone_pick drc = ar_isp_tone_none;
	bool gamma_built = false, drc_built = false;
	bool want_gamma, want_drc;

	if (!tables || !blob)
		return;

	want_gamma = isp->gamma && ar_isp_tone_gamma_pick(blob, &gamma);
	want_drc = isp->drc && ar_isp_tone_drc_pick(blob, &drc);

	/*
	 * A stage that will not be filled carries the sentinel on both sides of
	 * the comparison, so it neither reports a change nor hides one made to
	 * the other stage.
	 */
	if (!want_gamma)
		gamma = ar_isp_tone_none;

	if (!want_drc)
		drc = ar_isp_tone_none;

	if (!ar_isp_tone_changed(isp, &gamma, &drc, force)) {
		if (verbose)
			dev_info(isp->dev, "tone: selection unchanged, pages left\n");

		return;
	}

	if (want_gamma) {
		ar_isp_gamma_from_blob(isp->gamma, blob, gamma.low, gamma.high,
				       gamma.t_q12);
		ar_isp_gamma_pack_page(isp->gamma + AR_ISP_GAMMA_PAGE,
				       ar_isp_gamma_page1,
				       AR_ISP_GAMMA_PAGE1_TAIL);
		gamma_built = true;
		if (verbose)
			dev_info(isp->dev, "tone: gamma %u to %u, weight %u/4096\n",
				 gamma.low, gamma.high, gamma.t_q12);
	}

	if (want_drc) {
		ar_isp_drc_from_blob(isp->drc, blob, drc.low, drc.high,
				     drc.t_q12);
		ar_isp_drc_pack_bank(isp->drc + 2 * AR_ISP_DRC_BANK,
				     ar_isp_drc_tail_bank0);
		ar_isp_drc_pack_bank(isp->drc + 3 * AR_ISP_DRC_BANK,
				     ar_isp_drc_tail_bank1);
		drc_built = true;
		if (verbose)
			dev_info(isp->dev, "tone: DRC %u to %u, weight %u/4096\n",
				 drc.low, drc.high, drc.t_q12);
	}

	isp->last_gamma = gamma;
	isp->last_drc = drc;

	/*
	 * The buffers are coherent, so there is no cache to flush, but the page
	 * fills above must be visible before the descriptor writes below expose
	 * them to the block's fetch.
	 */
	wmb();

	if (gamma_built) {
		u32 addr = lower_32_bits(isp->gamma_dma);

		writel(addr, isp->base + AR_ISP_TABLE_GAMMA0);
		writel(addr, isp->base + AR_ISP_TABLE_GAMMA1);
		writel(addr, isp->base + AR_ISP_TABLE_GAMMA2);
		writel(AR_ISP_TABLE_COMMIT_ENABLE | AR_ISP_TABLE_GAMMA_BITS,
		       isp->base + AR_ISP_TABLE_COMMIT);
	}

	if (drc_built) {
		writel(lower_32_bits(isp->drc_dma), isp->base + AR_ISP_TABLE_DRC);
		writel(AR_ISP_TABLE_COMMIT_ENABLE | AR_ISP_TABLE_DRC_BIT,
		       isp->base + AR_ISP_TABLE_COMMIT);
	}

	if (verbose)
		dev_info(isp->dev, "tone: gamma %s, DRC %s, from %s\n",
			 gamma_built ? "built" : "left",
			 drc_built ? "built" : "left",
			 tone_scalar >= 0 ? "the trigger scalar" : "pinned indices");
}

/*
 * Where each page's content came from, for the one line that reports it.
 * "built" is generated from the tuning file, "seeded" is inherited from the
 * vendor's residual DRAM, and a page that is neither is zero.
 */
struct ar_isp_fill_status {
	bool gamma_built;
	bool gamma_seeded;
	bool drc_built;
	bool drc_seeded;
	bool tone_built;
	bool lsc_built;
	bool lsc_seeded;
	bool hdr_lsc_seeded;
};

/*
 * The fill half. These write memory and never a register, so nothing here is
 * visible to the block until the barrier and the publishes that follow them.
 */
static void ar_isp_fill_gamma(struct ar_isp *isp, const u8 *blob,
			      struct ar_isp_fill_status *st)
{
	struct ar_isp_tone_pick pick;
	u8 *page;

	if (!isp->gamma)
		return;

	if (seed)
		st->gamma_seeded = ar_isp_seed_from_vendor(isp, isp->gamma,
							  AR_ISP_VENDOR_GAMMA_PHYS,
							  AR_ISP_GAMMA_SIZE);
	else
		memset(isp->gamma, 0, AR_ISP_GAMMA_SIZE);

	if (!blob || !ar_isp_tone_gamma_pick(blob, &pick))
		return;

	page = isp->gamma;
	ar_isp_gamma_from_blob(page, blob, pick.low, pick.high, pick.t_q12);

	/*
	 * Page 1 is not an AE selection and is not in the tuning file: it is a
	 * carried constant. Both pages are written here, so gamma no longer
	 * depends on anything inherited.
	 *
	 * The descriptor length at 0x0034/0x0044/0x0054 is in 16-byte records.
	 * 0x80 records fetch one 0x800 page. The allocation stays 0x4000 to
	 * match the vendor's arena.
	 */
	ar_isp_gamma_pack_page(page + AR_ISP_GAMMA_PAGE, ar_isp_gamma_page1,
			       AR_ISP_GAMMA_PAGE1_TAIL);
	st->gamma_built = true;
	isp->last_gamma = pick;
}

static void ar_isp_fill_drc(struct ar_isp *isp, const u8 *blob,
			    struct ar_isp_fill_status *st)
{
	struct ar_isp_tone_pick pick;
	u8 *page;

	if (!isp->drc)
		return;

	if (seed)
		st->drc_seeded = ar_isp_seed_from_vendor(isp, isp->drc,
							AR_ISP_VENDOR_DRC_PHYS,
							AR_ISP_DRC_SIZE);
	else
		memset(isp->drc, 0, AR_ISP_DRC_SIZE);

	if (!blob || !ar_isp_tone_drc_pick(blob, &pick))
		return;

	/*
	 * The whole page, both halves: the first two banks from the tuning file,
	 * the second two from the constant the vendor never recomputes. Nothing
	 * here is inherited, which is why the seed above is redundant when this
	 * runs and is left in place only so the two can be compared in one
	 * bring-up.
	 */
	page = isp->drc;
	ar_isp_drc_from_blob(page, blob, pick.low, pick.high, pick.t_q12);
	ar_isp_drc_pack_bank(page + 2 * AR_ISP_DRC_BANK, ar_isp_drc_tail_bank0);
	ar_isp_drc_pack_bank(page + 3 * AR_ISP_DRC_BANK, ar_isp_drc_tail_bank1);
	st->drc_built = true;
	isp->last_drc = pick;
}

static void ar_isp_fill_tone(struct ar_isp *isp, struct ar_isp_fill_status *st)
{
	if (!isp->tone)
		return;

	/*
	 * The compander has no seed path and no tuning file: the page is the
	 * same bytes on every unit and in every scene, so there is nothing to
	 * fall back to and nothing to select.
	 *
	 * The HDR page's own 0xa00 is left zero. Its first 0x800 is zero on the
	 * vendor too; the 0x200 after that is scene-varying runtime state,
	 * measured at 117 of 512 bytes differing between two captures, so it has
	 * no stored source to reproduce. Zeroing it was measured on hardware to
	 * move 6.3% of pixels by more than 8 levels, against a 94.5%
	 * frame-to-frame floor from scene motion alone: no effect.
	 */
	memset(isp->tone, 0, AR_ISP_TONE_ALLOC);
	ar_isp_compander_fill(isp->tone + AR_ISP_HDR_COMPANDER,
			      ar_isp_compander_head, ar_isp_compander_mid);
	st->tone_built = true;
}

static void ar_isp_fill_ltm(struct ar_isp *isp)
{
	__le16 *page = isp->ltm_page;

	if (page) {
		/*
		 * Every tile the same linear curve, monotonic from 0 to the
		 * measured maximum, truncating as every quantisation in this
		 * pipeline does. The stride leaves a gap after each tile's 128
		 * samples; the whole page is zeroed first so the gaps match the
		 * vendor's. Both halves get the identity, so the descriptor is
		 * sane whichever one it names before the first publish.
		 */
		memset(isp->ltm_page, 0, AR_ISP_LTM_PAGE_SIZE * 2);
		for (unsigned int i = 0; i < AR_ISP_LTM_TILES; i++)
			for (unsigned int j = 0; j < AR_ISP_LTM_SAMPLES; j++)
				page[i * AR_ISP_LTM_TILE_STRIDE / 2 + j] =
					cpu_to_le16(j * AR_ISP_LTM_CURVE_MAX /
						    (AR_ISP_LTM_SAMPLES - 1));
		memcpy((u8 *)isp->ltm_page + AR_ISP_LTM_PAGE_SIZE,
		       isp->ltm_page, AR_ISP_LTM_PAGE_SIZE);
	}

	if (isp->ltm_stats)
		memset(isp->ltm_stats, 0, AR_ISP_LTM_STATS_SIZE);
}

/*
 * Install a new coefficient page the way the vendor does: fill the half the
 * descriptor does not currently name, then point the descriptor at it. The
 * hardware latches the address when it fetches, so the half being fetched
 * from is never the one being written (plans/done/au-ltm-page-algorithm.md
 * section 3). writel orders the copy ahead of the descriptor update.
 *
 * Serialised by ltm_lock: publishers are configure and the debugfs feed, both
 * process context.
 */
int ar_isp_ltm_page_publish(struct ar_isp *isp, const void *page)
{
	unsigned int next;

	if (!isp->ltm_page)
		return -ENODEV;

	mutex_lock(&isp->ltm_lock);
	next = isp->ltm_page_half ^ 1;
	memcpy((u8 *)isp->ltm_page + next * AR_ISP_LTM_PAGE_SIZE, page,
	       AR_ISP_LTM_PAGE_SIZE);
	writel(lower_32_bits(isp->ltm_page_dma + next * AR_ISP_LTM_PAGE_SIZE),
	       isp->base + AR_ISP_LTM_PAGE_ADDR);
	isp->ltm_page_half = next;
	mutex_unlock(&isp->ltm_lock);

	return 0;
}

static void ar_isp_fill_lsc(struct ar_isp *isp, const u8 *blob,
			    struct ar_isp_fill_status *st)
{
	if (!isp->lsc)
		return;

	/*
	 * Only region A, the lens-shading grid, is generated. The scene-adaptive
	 * 0x2c0 after it has no stored source anywhere and is left to the seed,
	 * so with seeding off it is zero and the block runs on shading alone.
	 */
	if (seed)
		st->lsc_seeded = ar_isp_seed_from_vendor(isp, isp->lsc,
							AR_ISP_VENDOR_LSC_PHYS,
							AR_ISP_LSC_SIZE);
	else
		memset(isp->lsc, 0, AR_ISP_LSC_SIZE);

	if (!blob)
		return;

	ar_isp_lsc_from_blob(isp->lsc, blob);
	st->lsc_built = true;
}

static void ar_isp_fill_hdr_lsc(struct ar_isp *isp,
				struct ar_isp_fill_status *st)
{
	if (!isp->hdr_lsc)
		return;

	/*
	 * The page is owned but not built: filling it with the LSC grid applies
	 * shading twice and was measured on hardware to blow out the corners.
	 * The stage's real payload is unrecovered, so the fill is zero until it
	 * is.
	 */
	if (seed)
		st->hdr_lsc_seeded = ar_isp_seed_from_vendor(isp, isp->hdr_lsc,
							     AR_ISP_VENDOR_HDR_LSC_PHYS,
							     AR_ISP_LSC_SIZE);
	else
		memset(isp->hdr_lsc, 0, AR_ISP_LSC_SIZE);
}

/*
 * The publish half. These write descriptor registers and never memory.
 *
 * The descriptors are 32-bit. dma_alloc_coherent is bounded by the mask set in
 * ar_isp_tables_prepare, so this cannot truncate, but the cast is written out
 * rather than left implicit.
 */
static void ar_isp_publish_gamma(struct ar_isp *isp)
{
	u32 addr;

	if (!isp->gamma)
		return;

	addr = lower_32_bits(isp->gamma_dma);
	writel(addr, isp->base + AR_ISP_TABLE_GAMMA0);
	writel(addr, isp->base + AR_ISP_TABLE_GAMMA1);
	writel(addr, isp->base + AR_ISP_TABLE_GAMMA2);
	writel(AR_ISP_TABLE_COMMIT_ENABLE | AR_ISP_TABLE_GAMMA_BITS,
	       isp->base + AR_ISP_TABLE_COMMIT);
}

static void ar_isp_publish_drc(struct ar_isp *isp)
{
	if (!isp->drc)
		return;

	writel(lower_32_bits(isp->drc_dma), isp->base + AR_ISP_TABLE_DRC);
	writel(AR_ISP_TABLE_COMMIT_ENABLE | AR_ISP_TABLE_DRC_BIT,
	       isp->base + AR_ISP_TABLE_COMMIT);
}

static void ar_isp_publish_tone(struct ar_isp *isp)
{
	if (!isp->tone)
		return;

	writel(lower_32_bits(isp->tone_dma + AR_ISP_HDR_COMPANDER),
	       isp->base + AR_ISP_TABLE_COMPANDER);
	writel(AR_ISP_TABLE_COMMIT_ENABLE | AR_ISP_TABLE_COMPANDER_BIT,
	       isp->base + AR_ISP_TABLE_COMMIT);

	if (!hdr)
		return;

	writel(lower_32_bits(isp->tone_dma), isp->base + AR_ISP_TABLE_HDR);
	writel(1, isp->base + AR_ISP_TABLE_HDR_VALID);
}

static void ar_isp_publish_lsc(struct ar_isp *isp)
{
	if (!isp->lsc)
		return;

	writel(lower_32_bits(isp->lsc_dma), isp->base + AR_ISP_TABLE_LSC);
	writel(1, isp->base + AR_ISP_TABLE_LSC_VALID);
}

static void ar_isp_publish_hdr_lsc(struct ar_isp *isp)
{
	if (!isp->hdr_lsc)
		return;

	writel(lower_32_bits(isp->hdr_lsc_dma), isp->base + AR_ISP_TABLE_HDR_LSC);
	writel(1, isp->base + AR_ISP_TABLE_HDR_LSC_VALID);
}

static void ar_isp_tables_report(struct ar_isp *isp,
				 const struct ar_isp_fill_status *st)
{
	dev_info(isp->dev,
		 "tables: gamma %pad %s, drc %pad %s, compander %pad %s, lsc %pad %s\n",
		 &isp->gamma_dma,
		 st->gamma_built ? "built" : (st->gamma_seeded ? "seeded" : "zeroed"),
		 &isp->drc_dma,
		 st->drc_built ? "built" : (st->drc_seeded ? "seeded" : "zeroed"),
		 &isp->tone_dma,
		 st->tone_built ? "hdr+compander built" : "on the vendor's page",
		 &isp->lsc_dma,
		 st->lsc_built ? "shading built" : (st->lsc_seeded ? "seeded" : "zeroed"));

	if (isp->hdr_lsc)
		dev_info(isp->dev, "hdr_lsc: %pad %s\n", &isp->hdr_lsc_dma,
			 st->hdr_lsc_seeded ? "seeded" : "zeroed");
}

/*
 * Fill the owned buffers and hand them to the block.
 *
 * Runs after the register replay, which arms the vendor's addresses and commits
 * them. Republishing ours and committing again is the same sequence the vendor
 * itself issues on every AE update.
 *
 * Every fill runs before the barrier and every publish after it. That split is
 * the reason the two halves are separate passes rather than one pass per
 * buffer: a barrier per buffer would order each page only against its own
 * descriptor, and the block fetches all of them.
 */
void ar_isp_tables_apply(struct ar_isp *isp)
{
	const u8 *blob = isp->tuning ? isp->tuning->data : NULL;
	struct ar_isp_fill_status st = { };

	if (!tables)
		return;

	/*
	 * Both pages are about to be refilled from whatever the parameters
	 * select now. Anything either of them held before is gone, so the
	 * recorded selection starts at the sentinel and only the stages actually
	 * filled below record a pick.
	 */
	isp->last_gamma = ar_isp_tone_none;
	isp->last_drc = ar_isp_tone_none;

	ar_isp_fill_gamma(isp, blob, &st);
	ar_isp_fill_drc(isp, blob, &st);
	ar_isp_fill_tone(isp, &st);
	ar_isp_fill_ltm(isp);
	ar_isp_fill_lsc(isp, blob, &st);
	ar_isp_fill_hdr_lsc(isp, &st);

	/*
	 * The buffers are coherent, so there is no cache to flush, but the fills
	 * above must be visible before the addresses below make the block fetch
	 * them.
	 */
	wmb();

	ar_isp_publish_gamma(isp);
	ar_isp_publish_drc(isp);
	ar_isp_publish_tone(isp);
	ar_isp_publish_lsc(isp);
	ar_isp_publish_hdr_lsc(isp);
	ar_isp_stats_publish(isp);
	ar_isp_de3d_publish(isp);

	ar_isp_tables_report(isp, &st);
}

/*
 * Allocate the owned buffers and load the tuning file.
 *
 * Both are optional: a failure here leaves the replayed vendor addresses armed,
 * which is what the driver did before it owned anything, so the camera still
 * comes up. The tuning file is not in the repository and has to be installed on
 * the device; without it the buffers are still ours but carry only seeded data.
 */
/*
 * One owned buffer, named if it cannot be had.
 *
 * A failure is not fatal and does not stop the others: the stage's descriptor
 * keeps the replayed vendor address, so it runs on the vendor's memory rather
 * than not at all. The name is the only thing that says which stage that
 * happened to, which is why the warning is here and not a count at the end.
 */
static void *ar_isp_alloc(struct ar_isp *isp, const char *name, size_t size,
			  dma_addr_t *dma)
{
	void *buf = dma_alloc_coherent(isp->dev, size, dma, GFP_KERNEL);

	if (buf)
		return buf;

	dev_warn(isp->dev,
		 "%s buffer (%zu bytes) unavailable, falling back to the vendor's\n",
		 name, size);

	return NULL;
}

void ar_isp_tables_prepare(struct ar_isp *isp)
{
	struct device *dev = isp->dev;
	const char *fw;
	int ret;

	/* Above the early returns below, and not left to the zeroed allocation:
	 * an all-zero pick is curve 0 with no blend, which is a selection a page
	 * can legitimately hold.
	 */
	isp->last_gamma = ar_isp_tone_none;
	isp->last_drc = ar_isp_tone_none;

	/* Mask before the pool, matching vif and cvisp: the descriptor registers
	 * carry a 32-bit address, so the DMA API is constrained before anything
	 * is attached or allocated.
	 */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_warn(dev, "no 32-bit dma mask (%d), not owning tables\n", ret);
		return;
	}

	ret = of_reserved_mem_device_init(dev);
	if (ret) {
		dev_warn(dev, "no isp memory pool (%d), not owning tables\n", ret);
		return;
	}

	isp->gamma = ar_isp_alloc(isp, "gamma", AR_ISP_GAMMA_SIZE,
				  &isp->gamma_dma);
	isp->drc = ar_isp_alloc(isp, "drc", AR_ISP_DRC_SIZE, &isp->drc_dma);

	/* The debugfs views stay empty until the pages exist, so a read before
	 * this point returns nothing rather than a stale or wild pointer.
	 */
	isp->gamma_blob.data = isp->gamma;
	isp->gamma_blob.size = isp->gamma ? AR_ISP_GAMMA_SIZE : 0;
	isp->drc_blob.data = isp->drc;
	isp->drc_blob.size = isp->drc ? AR_ISP_DRC_SIZE : 0;

	if (compander)
		isp->tone = ar_isp_alloc(isp, "compander", AR_ISP_TONE_ALLOC,
					 &isp->tone_dma);

	if (lsc)
		isp->lsc = ar_isp_alloc(isp, "lsc", AR_ISP_LSC_SIZE,
					&isp->lsc_dma);

	if (hdr_lsc)
		isp->hdr_lsc = ar_isp_alloc(isp, "hdr_lsc", AR_ISP_LSC_SIZE,
					    &isp->hdr_lsc_dma);

	if (stats) {
		/*
		 * Two halves per buffer in one allocation, so a half is an
		 * offset rather than a second address to keep track of. The
		 * halves are allocated even with pingpong off, which costs
		 * 40 KiB and keeps the parameter a pure behaviour switch: the
		 * A/B then differs only in whether the interrupt flips.
		 */
		isp->rro = ar_isp_alloc(isp, "rro",
					AR_ISP_RRO_SIZE * AR_ISP_STATS_HALVES,
					&isp->rro_dma);
		isp->rro_face = ar_isp_alloc(isp, "rro_face",
					     AR_ISP_RRO_SIZE * AR_ISP_STATS_HALVES,
					     &isp->rro_face_dma);
		isp->hist = ar_isp_alloc(isp, "hist",
					 AR_ISP_HIST_SIZE * AR_ISP_STATS_HALVES,
					 &isp->hist_dma);
	}

	if (ltm) {
		isp->ltm_page = ar_isp_alloc(isp, "ltm_page",
					     AR_ISP_LTM_PAGE_SIZE * 2,
					     &isp->ltm_page_dma);
		isp->ltm_stats = ar_isp_alloc(isp, "ltm_stats",
					      AR_ISP_LTM_STATS_SIZE,
					      &isp->ltm_stats_dma);
		isp->ltm_stats_blob.data = isp->ltm_stats;
		isp->ltm_stats_blob.size = isp->ltm_stats ?
					   AR_ISP_LTM_HIST_SIZE : 0;
		isp->ltm_stats_full_blob.data = isp->ltm_stats;
		isp->ltm_stats_full_blob.size = isp->ltm_stats ?
						AR_ISP_LTM_STATS_SIZE : 0;
	}

	if (de3d) {
		static const struct {
			const char *name;
			size_t size;
		} buf[3] = {
			{ "de3d0", AR_ISP_DE3D_BUF0_SIZE },
			{ "de3d1", AR_ISP_DE3D_BUF1_SIZE },
			{ "de3d2", AR_ISP_DE3D_BUF2_SIZE },
		};

		for (unsigned int i = 0; i < ARRAY_SIZE(buf); i++)
			isp->de3d[i] = ar_isp_alloc(isp, buf[i].name,
						    buf[i].size,
						    &isp->de3d_dma[i]);
	}

	/* The layout is the same for all three sensors; the values are the board's. */
	if (of_property_read_string(dev->of_node, "artosyn,tuning-firmware", &fw))
		fw = AR_ISP_TUNING_FIRMWARE;

	ret = request_firmware(&isp->tuning, fw, dev);
	if (ret) {
		dev_warn(dev, "no %s (%d), tables cannot be generated\n",
			 fw, ret);
		isp->tuning = NULL;

		return;
	}

	if (isp->tuning->size != AR_ISP_TUNING_SIZE) {
		dev_warn(dev, "%s is %zu bytes, expected %u; ignoring it\n",
			 fw, isp->tuning->size,
			 AR_ISP_TUNING_SIZE);
		release_firmware(isp->tuning);
		isp->tuning = NULL;
	}
}

void ar_isp_tables_release(struct ar_isp *isp)
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
		dma_free_coherent(isp->dev,
				  AR_ISP_RRO_SIZE * AR_ISP_STATS_HALVES,
				  isp->rro, isp->rro_dma);

	if (isp->rro_face)
		dma_free_coherent(isp->dev,
				  AR_ISP_RRO_SIZE * AR_ISP_STATS_HALVES,
				  isp->rro_face, isp->rro_face_dma);

	if (isp->hist)
		dma_free_coherent(isp->dev,
				  AR_ISP_HIST_SIZE * AR_ISP_STATS_HALVES,
				  isp->hist, isp->hist_dma);

	if (isp->ltm_page)
		dma_free_coherent(isp->dev, AR_ISP_LTM_PAGE_SIZE * 2,
				  isp->ltm_page, isp->ltm_page_dma);

	if (isp->ltm_stats)
		dma_free_coherent(isp->dev, AR_ISP_LTM_STATS_SIZE,
				  isp->ltm_stats, isp->ltm_stats_dma);
	{
		static const size_t sz[3] = {
			AR_ISP_DE3D_BUF0_SIZE, AR_ISP_DE3D_BUF1_SIZE,
			AR_ISP_DE3D_BUF2_SIZE,
		};

		for (unsigned int i = 0; i < ARRAY_SIZE(sz); i++)
			if (isp->de3d[i])
				dma_free_coherent(isp->dev, sz[i], isp->de3d[i],
						  isp->de3d_dma[i]);
	}

	release_firmware(isp->tuning);

	/*
	 * Matches the of_reserved_mem_device_init in ar_isp_tables_prepare.
	 * Unconditional because the release matches on the device and does
	 * nothing when nothing was attached, so the two early returns in
	 * prepare need no state of their own.
	 */
	of_reserved_mem_device_release(isp->dev);
}
