/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ar-isp-stats.h - layout of the ISP's hardware-written statistics buffers.
 *
 * These buffers are filled by the ISP over DMA, not by software, so unlike the
 * coefficient tables there is nothing to pack: the driver publishes an address
 * and a geometry and then reads what the hardware left. What this header
 * describes is how to index the result.
 *
 * Two producers matter for auto-exposure. The RRO engine writes a 36x16 grid of
 * per-zone, per-channel sums, and the raw histogram writes a 128-bin Bayer
 * histogram of the whole frame. Both were decoded from buffers captured
 * mid-stream off a streaming vendor unit and cross-checked against the geometry
 * the vendor programmed into the hardware registers.
 *
 * Module-to-bank attribution comes from each isp_sub_*_creat's second
 * registered handler, which maps the module's bank with the bank offset as an
 * immediate beside its ar_dev_pa2va calls. The runnable proof of everything
 * here is kernel/scripts/check-stats-layout.py.
 *
 * Pure indexing arithmetic: no register access, so the same source can be
 * compiled host-side against the captures.
 */

#ifndef AR_ISP_STATS_H
#define AR_ISP_STATS_H

/*
 * RRO: the zone metering engine behind auto-exposure.
 *
 * Bank 0x6400 holds two independent engines at a 0x34 stride, and rro_face is a
 * third instance of the same block at 0x64c8. The vendor programs all three to
 * the same 36x16 geometry and points both 0x6400 engines at one buffer.
 */
#define AR_ISP_RRO_BANK			0x6400
#define AR_ISP_RRO_ENGINE_STRIDE	0x34
#define AR_ISP_RRO_ENGINES		2
#define AR_ISP_RRO_FACE_BANK		0x64c8

/* Geometry registers, relative to an engine's base. */
#define AR_ISP_RRO_REG_COLS		0x1c
#define AR_ISP_RRO_REG_ROWS		0x20
#define AR_ISP_RRO_REG_WIDTH		0x2c
#define AR_ISP_RRO_REG_HEIGHT		0x34
#define AR_ISP_RRO_REG_ADDR		0x40

/*
 * Buffer layout. One column per grid column, each column a 0x100 count block
 * followed by a 0x100 sum block. A block is 16 zones by 4 channels of u32, so
 * the grid is stored column major: the 16 entries of a column are contiguous.
 *
 * Every u32 in a count block holds the same value, the number of pixels the
 * engine accumulated per zone per channel. Dividing a sum by it gives that
 * zone's channel mean, and in the reference capture those means span 4.2 to
 * exactly 255.0 with the maximum exactly 255 * count, which is what establishes
 * the sums as 8-bit samples rather than the sensor's 10.
 *
 * Channels 1 and 2 track each other closely across the whole grid and channels
 * 0 and 3 sit either side of them, which identifies 1 and 2 as the two greens.
 * Which of 0 and 3 is red and which is blue is not established here.
 */
#define AR_ISP_RRO_COLS			36
#define AR_ISP_RRO_ROWS			16
#define AR_ISP_RRO_ZONES		(AR_ISP_RRO_COLS * AR_ISP_RRO_ROWS)
#define AR_ISP_RRO_CHANNELS		4
#define AR_ISP_RRO_BLOCK		0x100
#define AR_ISP_RRO_COL_STRIDE		0x200
#define AR_ISP_RRO_SIZE			(AR_ISP_RRO_COLS * AR_ISP_RRO_COL_STRIDE)

/*
 * Raw histogram: 128 bins over the full frame, three populated lanes.
 *
 * 4 u32 per bin. Lane 0 is red, lane 1 green and lane 2 blue; lane 3 is always
 * zero. The lanes sum to exactly one quarter, one half and one quarter of the
 * frame's pixel count, which is the Bayer population and is what identifies
 * them. All three lanes together account for every pixel in the frame.
 *
 * The buffer is 0x1000 with only the first 0x800 written; the rest is zero.
 * Bin width is the sensor range divided by 128 and is not pinned down here,
 * because the capture alone cannot distinguish a 10-bit input binned by 8 from
 * any other range that leaves the populated bins where they are.
 */
#define AR_ISP_HIST_BANK		0x6000
#define AR_ISP_HIST_REG_ADDR		0x0c
#define AR_ISP_HIST_BINS		128
#define AR_ISP_HIST_LANES		4
#define AR_ISP_HIST_CONTENT		0x800
#define AR_ISP_HIST_SIZE		0x1000
#define AR_ISP_HIST_LANE_R		0
#define AR_ISP_HIST_LANE_G		1
#define AR_ISP_HIST_LANE_B		2

static inline u32 ar_isp_stats_get_le32(const u8 *p)
{
	return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) |
	       ((u32)p[3] << 24);
}

/*
 * The accumulated sum for one zone and channel. @col is 0 to 35 and @row is
 * 0 to 15, indexing the grid as the hardware lays it out.
 */
static inline u32 ar_isp_rro_sum(const u8 *buf, unsigned int col,
				 unsigned int row, unsigned int channel)
{
	const u8 *block = buf + col * AR_ISP_RRO_COL_STRIDE + AR_ISP_RRO_BLOCK;

	return ar_isp_stats_get_le32(block + (row * AR_ISP_RRO_CHANNELS +
					      channel) * 4);
}

/*
 * The pixel count a column's sums were accumulated over. Constant across the
 * whole buffer in every capture, but read per column rather than assumed: it is
 * a function of the programmed geometry, and a driver that changes the grid
 * must not carry a stale divisor.
 */
static inline u32 ar_isp_rro_count(const u8 *buf, unsigned int col)
{
	return ar_isp_stats_get_le32(buf + col * AR_ISP_RRO_COL_STRIDE);
}

/*
 * A zone's channel mean, in the 0 to 255 range the sums are accumulated in.
 * Returns 0 for a zero count so a misprogrammed grid cannot divide by zero.
 */
static inline u32 ar_isp_rro_mean(const u8 *buf, unsigned int col,
				  unsigned int row, unsigned int channel)
{
	u32 count = ar_isp_rro_count(buf, col);

	if (!count)
		return 0;

	return ar_isp_rro_sum(buf, col, row, channel) / count;
}

static inline u32 ar_isp_hist_bin(const u8 *buf, unsigned int lane,
				  unsigned int bin)
{
	return ar_isp_stats_get_le32(buf + (bin * AR_ISP_HIST_LANES + lane) * 4);
}

#endif /* AR_ISP_STATS_H */
