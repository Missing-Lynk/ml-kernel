/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-regs.h - the ISP register map, and what each register was measured to
 * do.
 *
 * Register offsets from the ISP block base at 0x08c00000, the buffer sizes and
 * physical addresses the replayed vendor configuration arms, and the tuning
 * file's name and length. The per-stage banks live with their stage
 * (ar-isp-stats.h, ar-isp-colour.h, ar-isp-rnr.h and the rest); what is here is
 * the block-level map the driver programs directly.
 */

#ifndef AR_ISP_REGS_H
#define AR_ISP_REGS_H

#include "vendor-tables/ar-isp-blob.h"
#include <linux/bits.h>

#include "ar-isp-stats.h"

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
 * de3d's working buffers, on its bank 0x2e00, which belongs to isp_sub_de3d.
 * CVISP is the block that writes frames to DRAM.
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
 * Sizes, derived from the vendor's own arithmetic rather than from the gaps
 * between its allocations.
 *
 * The de3d module computes its memory requirement in libmpp_service.so at
 * 0x1c4b6c and 0x1c4e08, from the same two strides it programs into the bank:
 *
 *   buffer 0 is stride x height, stride being the padded width in tiles of 20
 *   at sixteen bytes each rounded up to 256 (1792 at 1928 wide), so 1792 x
 *   1080;
 *
 *   buffer 1 is half of that. The module adds `w0 + (w0 >> 1)` for the pair at
 *   0x1c4bbc, which is buffer 0 plus half of it, and that ratio is the only
 *   thing that term can mean;
 *
 *   buffer 2 is the block-grid stride, ceil(width / 8) rounded up to 256, times
 *   ceil(height / 8) + 2 rows. That product is computed outright at 0x1c4e64.
 *
 * The two that can be checked against the vendor's own layout both land just
 * inside it: 0x1d8800 against the 0x1db000 gap and 0xec400 against 0xef000,
 * each about ten kilobytes short, which is the slack of a page-aligned
 * allocator. Nothing sits above the third, so its gap never bounded anything
 * and the previous size here was the larger of the other two carried over as a
 * guess; the derivation puts it at 0x8900.
 *
 * Rounded up to a page, because the vendor's own allocations are and because
 * the failure mode is one-sided: over-allocating wastes reserved memory, while
 * under-allocating is a hardware DMA writing past a buffer we own. The isp_cma
 * reservation is no-map, so an overrun stays inside the reserved region and
 * cannot reach kernel memory, but it would quietly corrupt our own tables.
 *
 * kernel/scripts/isp/check-de3d-geometry.py recomputes all three and fails if
 * either of the two bounded ones stops fitting.
 */
#define AR_ISP_DE3D_BUF0_SIZE		0x1d9000
#define AR_ISP_DE3D_BUF1_SIZE		0x0ed000
#define AR_ISP_DE3D_BUF2_SIZE		0x009000

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
 * AR_ISP_IN_LINETIME latches the incoming line time and holds it: a streaming
 * vendor reads 0x134e across repeated samples while VIF
 * 0x1f8 measures about the same. Zero means no line timing is arriving.
 */
#define AR_ISP_IN_GEOMETRY		0x706c
#define AR_ISP_IN_LINETIME		0x2e98

/*
 * Coefficient table descriptors. Each holds the physical address of a DMA buffer
 * the block fetches when the matching bit is written to AR_ISP_TABLE_COMMIT.
 *
 * The commit is write-to-trigger: clearing the bit afterwards cancels the
 * fetch. Bit 16 is always set alongside. From the vendor
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
 * On the vendor the two overlap: the compander sits at
 * 0x2b2e0c00 and the HDR page at 0x2b2e0200, exactly 0xa00 lower, and the HDR
 * descriptor length makes it fetch 0x1000. So its last 0x600 bytes ARE the
 * compander's first 0x600, read because the fetch overruns its own content.
 * Measured on captured pages: zero differing bytes across all 1536.
 *
 * Allocating one block in that relationship reproduces the fetched bytes for
 * both descriptors without copying anything twice, and makes the overlap
 * explicit.
 *
 * The length register at 0x0024 implies a 0xf000 compander span; the table
 * occupies 0x7800. The vendor cannot satisfy that either:
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
 * These belong to the vendor. Reading them is how a table is
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
 * These are DMA targets the hardware writes: no commit and no valid bit,
 * publishing an address is the whole protocol.
 */
#define AR_ISP_STATS_RRO0	(AR_ISP_RRO_BANK + AR_ISP_RRO_REG_ADDR)
#define AR_ISP_STATS_RRO1	(AR_ISP_RRO_BANK + AR_ISP_RRO_ENGINE_STRIDE + \
				 AR_ISP_RRO_REG_ADDR)
#define AR_ISP_STATS_RRO_FACE	(AR_ISP_RRO_FACE_BANK + AR_ISP_RRO_REG_ADDR)
#define AR_ISP_STATS_HIST	(AR_ISP_HIST_BANK + AR_ISP_HIST_REG_ADDR)

/*
 * Statistics ping-pong. The vendor keeps one reference index per statistics
 * block in its private context (privctx[0x12948 + id * 4]) and flips it on
 * subcommand 0x2c02, which its interrupt handler issues once per frame gated on
 * a single status bit. The consumer then reads whichever half the index does
 * not name, so a reader never races the engine writing the next frame.
 *
 * What is reproduced here is the mechanism: an
 * index per buffer this driver owns, flipped from the frame's statistics event,
 * with each buffer allocated at twice its geometry so the two halves are one
 * allocation. The vendor's 0x8000 stride is its own allocation granularity and
 * carries no meaning for buffers of our sizes.
 *
 * ltm_stats is deliberately single-buffered. Nothing reads it while the LTM
 * page is a fixed identity, so doubling its measured 0x80000 extent would spend
 * half a megabyte of coherent memory to serve no consumer. It joins the
 * ping-pong when the CLAHE port gives it one.
 */
#define AR_ISP_STATS_HALVES		2

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

#endif /* AR_ISP_REGS_H */
