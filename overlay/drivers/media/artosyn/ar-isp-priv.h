/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-priv.h - driver state and the register accessor, shared inside ar-isp.
 *
 * struct ar_isp is shared rather than split because the two halves of the
 * driver own opposite ends of the same buffers: the table half allocates them,
 * fills them and publishes their addresses, and the core half owns their
 * lifetime through probe and remove.
 *
 * Nothing outside ar-isp.ko includes this. The interface other modules use is
 * ar-camera-hook.h.
 */

#ifndef AR_ISP_PRIV_H
#define AR_ISP_PRIV_H

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/types.h>

#include "vendor-tables/ar-isp-defaults.h"
#include "vendor-tables/ar-isp-library.h"

struct device;
struct dentry;
struct firmware;

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
	 * Ping-pong state. stats_cur is the half the hardware is writing now,
	 * so it is the half published to the address registers. stats_done is
	 * the half the previous frame finished, which is what a consumer may
	 * read; it means nothing until stats_valid, because before the first
	 * flip no frame has completed into either half.
	 */
	unsigned int stats_cur;
	unsigned int stats_done;
	bool stats_valid;
	u32 stats_flips;

	/*
	 * de3d's three working buffers. Hardware-written, armed once, and until
	 * now left pointing at the vendor's own memory across the RAM-boot.
	 */
	void *de3d[3];
	dma_addr_t de3d_dma[3];
};

static inline void ar_isp_apply(struct ar_isp *isp, const struct ar_isp_reg *tbl,
				size_t n)
{
	for (size_t i = 0; i < n; i++)
		writel(tbl[i].val, isp->base + tbl[i].off);
}

/*
 * One register whose bits are shared between producers: the stage owns the mask
 * and every other bit keeps the value the replay left.
 */
static inline void ar_isp_rmw(struct ar_isp *isp, u32 off, u32 mask, u32 val)
{
	u32 old = readl(isp->base + off);

	writel((old & ~mask) | (val & mask), isp->base + off);
}

/*
 * ar-isp-tables.c. The whole crossing surface: everything else in that file is
 * reached through one of these.
 */
void ar_isp_de3d_publish(struct ar_isp *isp);
void ar_isp_stats_arm(struct ar_isp *isp, unsigned int half);
const void *ar_isp_stats_ready(struct ar_isp *isp, const void *buf, size_t size);
void ar_isp_stats_publish(struct ar_isp *isp);
void ar_isp_ccm_apply(struct ar_isp *isp);
void ar_isp_dpc_apply(struct ar_isp *isp);
void ar_isp_rnr_apply(struct ar_isp *isp, bool verbose);
void ar_isp_lnr_apply(struct ar_isp *isp, bool verbose);
void ar_isp_de3d_apply(struct ar_isp *isp, bool verbose);
void ar_isp_cfa_apply(struct ar_isp *isp, bool verbose);
void ar_isp_cnf_apply(struct ar_isp *isp, bool verbose);
void ar_isp_rgb2yuv_apply(struct ar_isp *isp);
void ar_isp_ladders_apply(struct ar_isp *isp, bool verbose);
void ar_isp_tables_apply(struct ar_isp *isp);
void ar_isp_tables_prepare(struct ar_isp *isp);
void ar_isp_tables_release(struct ar_isp *isp);

/*
 * The module parameters read on both sides of the cut. All are mode 0644 and
 * are re-read on the next configure, so they cannot be snapshotted into
 * struct ar_isp at probe without turning a live lever into a boot-time
 * constant. Defined in ar-isp-main.c; the ones read on one side only stay
 * static in whichever file uses them.
 */
extern bool tables;
extern bool hdr;
extern bool lsc;
extern bool stats;
extern bool pingpong;

#endif /* AR_ISP_PRIV_H */
