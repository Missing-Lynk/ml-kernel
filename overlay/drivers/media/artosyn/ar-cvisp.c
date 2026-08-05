// SPDX-License-Identifier: GPL-2.0
/*
 * ar-cvisp.c - Artosyn CVISP output stage, bring-up driver.
 *
 * CVISP is the block at 0x08e00000, ISP base + 0x200000. It is absent from the
 * vendor device tree; the name comes from the cvisp_* stack exported by the
 * vendor's unstripped libmpp_service.so. It is not the DTS scaler@08840000 or
 * gdc@08848000. In the vendor's design this block writes frames to DRAM; the
 * ISP feeds it and CVISP owns the output queue.
 *
 * This driver applies the recovered configuration and exposes the output queue,
 * both as a V4L2 capture node and, while that node is not streaming, as the
 * vendor's own five-slot ring of fixed DRAM addresses driven from debugfs. The
 * second form is kept as the capture path that does not depend on this driver's
 * buffer handling.
 *
 * Cadences, from the trace and reflected in the tables:
 *
 *   setup  once, ending at the staged output enable
 *   late   once, just after the enable, with frames already in flight
 *   ring   one Y/U/V triplet per frame, round robin over five buffer sets
 *   tick   eight registers, once per ring wrap rather than once per frame
 *
 * Not implemented here:
 *
 *  - Bringing the rest of the chain up. STREAMON takes over the output queue,
 *    but the sensor, VIF and ISP are still started from outside this driver.
 *  - The interrupt. The block has its own completion path (cvisp_dispatch_irq
 *    in libmpp_service.so), but the IRQ number and acknowledge register are
 *    still behind the vendor's generic event layer. None is claimed. The queue
 *    is rotated from the ISP's per-frame tick instead; see ar-camera-hook.h.
 *  - A reset line. No CVISP reset write appears in the trace and no reset leaf
 *    has been identified, so none is declared.
 *
 * Configuration provenance is in vendor-tables/ar-cvisp-defaults.h. See
 * ../../../../docs/camera-stack.md.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>

#include "ar-camera-hook.h"
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "vendor-tables/ar-cvisp-defaults.h"
#include "ar-isp-blc.h"

/*
 * The tuning file BLC reads. Same file ar-isp loads; requested here rather than
 * shared because the two drivers own different blocks, and it is released as
 * soon as the 64-byte block has been built.
 */
#define AR_CVISP_TUNING_FIRMWARE	"artosyn/nt99235-tuning-preview-fpv.bin"

/*
 * Output control. The vendor stages this 0x00800800 -> 0x00800802 -> 0x00800806
 * at the end of setup and never writes it again; bits 1 and 2 are undecoded.
 * The staging is embedded in the setup table so the table alone reproduces the
 * vendor's sequence.
 */
#define AR_CVISP_CONTROL		0x8000

/*
 * Output geometry. Both are written during setup with 0x021c03c0 (540 x 960)
 * and 0x8008 ends at 0x04380780 (1080 x 1920), while the page 0x4000 registers
 * below carry 1920 x 1080 throughout. Which stage is scaled is not established,
 * so these are dumped rather than interpreted.
 */
#define AR_CVISP_OUT_SIZE		0x8008
#define AR_CVISP_OUT_SIZE2		0x8010
#define AR_CVISP_IN_SIZE		0x8028

/* Output plane bases, Y/U/V, rewritten in lockstep once per frame. */
#define AR_CVISP_PLANE_Y		0x8098
#define AR_CVISP_PLANE_U		0x8174
#define AR_CVISP_PLANE_V		0x8194

/*
 * Output geometry as the block is actually configured, which is what the
 * capture node has to describe.
 *
 * The strides are measured rather than assumed: in a dumped luma plane the gap
 * between bright runs is exactly 2048 and every sub-gap pair sums to it, and
 * dumping width x height instead truncates at row 1012 of 1080. The chroma
 * planes are half of both. 0x8038 reads 0x08000800 in the setup table, two
 * copies of 2048, which agrees.
 *
 * The plane sizes are stride x height, measured. The vendor's ring spaces U at
 * Y + 0x21e000 and V at U + 0x89000, four luma and eight chroma rows past the
 * frame; the block does not write that tail. ml-v4l2grab -m fills a buffer
 * with a position-keyed pattern before queueing it and reports what survived:
 * 518400 luma words, 1920 bytes on each of 1080 rows with the stride padding
 * untouched, and 138240 chroma words, the full 1024-byte stride on each of 540
 * rows. Luma writes its width, chroma writes its stride.
 *
 * Take that measurement from a buffer marked while the stream is running. The
 * first buffer of a stream is marked before STREAMON, while the vendor ring is
 * still rotating into the same reservation; ring slot 4 at 0x28014000 falls
 * inside it and its overhang writes exactly the four luma and eight chroma
 * rows of tail, so that buffer reports the vendor's extents.
 */
#define AR_CVISP_WIDTH			1920
#define AR_CVISP_HEIGHT			1080
#define AR_CVISP_Y_STRIDE		2048
#define AR_CVISP_C_STRIDE		1024

/* Y, U and V: the block writes three separate plane bases. */
#define AR_CVISP_PLANES			3
#define AR_CVISP_Y_SIZE			(AR_CVISP_Y_STRIDE * AR_CVISP_HEIGHT)
#define AR_CVISP_C_SIZE			(AR_CVISP_C_STRIDE * (AR_CVISP_HEIGHT / 2))

/*
 * Rows the buffers hold, above the rows the block writes.
 *
 * The wave5 encoder rounds its raw height up to W5_ENC_RAW_STEP_HEIGHT, 16, and
 * refuses an imported plane shorter than its stride times that height: 1080 rows
 * becomes 1088 and a buffer sized to the frame is 8 luma and 4 chroma rows short.
 * Allocating those rows is what lets a capture buffer be handed to the encoder
 * with no copy.
 *
 * Only the allocation grows. The block still writes AR_CVISP_HEIGHT rows, which
 * is what the payload reports, so nothing reads the padding as image data.
 */
#define AR_CVISP_ALLOC_HEIGHT		ALIGN(AR_CVISP_HEIGHT, 16)
#define AR_CVISP_Y_ALLOC		(AR_CVISP_Y_STRIDE * AR_CVISP_ALLOC_HEIGHT)
#define AR_CVISP_C_ALLOC		(AR_CVISP_C_STRIDE * (AR_CVISP_ALLOC_HEIGHT / 2))

/*
 * Deepest buffer hold the driver will accept, and so the most buffers the hardware can be
 * holding at once. See the depth parameter.
 */
#define AR_CVISP_MAX_DEPTH		4

/* Channel geometry from the late table: width, height, then the crop origin. */
#define AR_CVISP_CH_WIDTH		0x4000
#define AR_CVISP_CH_HEIGHT		0x4004
#define AR_CVISP_CROP_X			0x4100
#define AR_CVISP_CROP_W			0x4104
#define AR_CVISP_CROP_Y			0x4108
#define AR_CVISP_CROP_H			0x410c

/*
 * Fixed-pattern-noise control word. The stage's only writer in the vendor
 * library clears bit 4 on every tuning apply (0x1b125c, commands 0xb10 and
 * 0xb13) and nothing anywhere sets it, so clear is the vendor's steady state.
 * The stage has no init fill path, its DMA correction surface is never armed,
 * and no replay table or sweep covered the register, so before this write a
 * cold boot ran on the hardware reset value.
 */
#define AR_CVISP_FPN_CTRL		0x4400
#define AR_CVISP_FPN_ENABLE		BIT(4)

static bool fpn = true;
module_param(fpn, bool, 0644);
MODULE_PARM_DESC(fpn,
		 "clear fpn's enable bit as the vendor's tuning apply does (default on)");

/* Reads AR_CVISP_TUNING_FIRMWARE. */
static bool blc = true;
module_param(blc, bool, 0644);
MODULE_PARM_DESC(blc,
		 "generate black level correction from the tuning file (default on)");

/*
 * Sensor gain in the tuning file's ladder units, which span 1 to 2100. The
 * default is the vendor's traced operating point and reproduces its registers
 * exactly; auto-exposure will drive this once it owns gain.
 */
static unsigned int blc_gain = 187;
module_param(blc_gain, uint, 0644);
MODULE_PARM_DESC(blc_gain,
		 "sensor gain BLC blends for, in ladder units (default 187, the traced point)");

/*
 * Off by default: see the probe. The vendor never enables this clock and the
 * boot leaves its gate set, so asserting it is an experiment, not a dependency.
 */
static bool rotate = true;
module_param(rotate, bool, 0644);
MODULE_PARM_DESC(rotate,
		 "advance the output ring once per frame from the ISP frame tick (default on)");

/*
 * Frame ticks a buffer is held before it is handed back.
 *
 * The tick is the ISP's statistics event, so the output DMA for a frame is not
 * known to have drained when it fires. At 2 a
 * buffer is held a further full frame period; the cost is a buffer, and the
 * hardware holds one per unit of depth out of a pool of five.
 *
 * Measured at both settings with ml-v4l2grab -m, which fills a buffer with a
 * position-keyed pattern before queueing it and reports what survived: at 1
 * and at 2 alike a completed buffer carries 518400 luma words and 138240 per
 * chroma plane, its full written extent, with no marker left at the bottom.
 * The write has drained by the next tick.
 *
 * That measures the DMA, not the consumer. A consumer that still holds the
 * buffer when the next tick re-arms it reads while the hardware writes: the
 * wave5 encoder imports these buffers by dmabuf and does exactly that, so at
 * depth 1 it encodes torn frames while the drop counter stays clean. The
 * default is therefore 3, which is what the recorder and the air-unit video
 * path are validated at; 1 remains selectable for the drain measurement above
 * and costs a buffer less out of the pool of five.
 *
 * Take the measurement from a buffer marked while the stream is running. See
 * the geometry block for what the first buffer of a stream reports instead.
 */
static unsigned int depth = 3;
module_param(depth, uint, 0444);
MODULE_PARM_DESC(depth,
		 "frame ticks a buffer is held before completion, 1 to 4 (default 3)");

/*
 * Bring the sensor, input path and ISP up from STREAMON, so opening the node is
 * the whole bring-up and v4l2src works with nothing else driving debugfs.
 *
 * Off leaves the chain entirely to the caller, which is how the capture harness
 * ran before this existed and is the fallback if self bring-up misbehaves.
 */
static bool chain = true;
module_param(chain, bool, 0644);
MODULE_PARM_DESC(chain,
		 "bring the sensor, VIF and ISP up from STREAMON (default on)");

static bool assert_clk;
module_param(assert_clk, bool, 0444);
MODULE_PARM_DESC(assert_clk,
		 "take ownership of cgu_rsz_clk instead of inheriting boot state (default off)");

struct ar_cvisp_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	u32 plane[AR_CVISP_PLANES];	/* Y/U/V bases, as the block wants them */
};

struct ar_cvisp {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	bool clk_asserted;
	struct dentry *debugfs;
	u32 rotations;
	bool configured;
	unsigned int next;	/* ring slot to arm next */
	unsigned int frames;	/* triplets written since configure */

	/* The capture node. */
	struct v4l2_device v4l2_dev;
	struct video_device video_dev;
	struct vb2_queue queue;
	struct mutex lock;	/* serialises ioctls and vb2 operations */

	spinlock_t buffer_lock;	/* guards the list and the armed buffers */
	struct list_head buffer_list;
	/*
	 * The buffers the block may still be writing, youngest first: held[0]
	 * was programmed at the last tick, held[1] at the one before it. A
	 * buffer leaves the far end after depth ticks and only then is old
	 * enough to hand back; see ar_cvisp_frame_tick.
	 */
	struct ar_cvisp_buffer *held[AR_CVISP_MAX_DEPTH];
	bool streaming;
	bool chain_up;		/* the chain ahead of this block has been started */
	unsigned int sequence;
	u32 completions;	/* buffers handed back through the node */
	u32 drops;		/* ticks with nothing queued to arm */
};

static inline struct ar_cvisp_buffer *to_ar_cvisp_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct ar_cvisp_buffer, vb);
}

static const struct {
	u16 off;
	const char *name;
} ar_cvisp_dump_regs[] = {
	{ AR_CVISP_CONTROL,	"control" },
	{ AR_CVISP_IN_SIZE,	"in_size" },
	{ AR_CVISP_OUT_SIZE,	"out_size" },
	{ AR_CVISP_OUT_SIZE2,	"out_size2" },
	{ AR_CVISP_PLANE_Y,	"plane_y" },
	{ AR_CVISP_PLANE_U,	"plane_u" },
	{ AR_CVISP_PLANE_V,	"plane_v" },
	{ AR_CVISP_CH_WIDTH,	"ch_width" },
	{ AR_CVISP_CH_HEIGHT,	"ch_height" },
	{ AR_CVISP_CROP_X,	"crop_x" },
	{ AR_CVISP_CROP_W,	"crop_w" },
	{ AR_CVISP_CROP_Y,	"crop_y" },
	{ AR_CVISP_CROP_H,	"crop_h" },
};

static void ar_cvisp_apply(struct ar_cvisp *cv, const struct ar_cvisp_reg *tbl,
			   size_t n)
{
	for (size_t i = 0; i < n; i++)
		writel(tbl[i].val, cv->base + tbl[i].off);
}

/* Arm one ring slot. The three plane bases go in the vendor's own order. */
static void ar_cvisp_arm(struct ar_cvisp *cv, unsigned int slot)
{
	const struct ar_cvisp_bufset *b = &ar_cvisp_ring[slot];

	writel(b->y, cv->base + AR_CVISP_PLANE_Y);
	writel(b->u, cv->base + AR_CVISP_PLANE_U);
	writel(b->v, cv->base + AR_CVISP_PLANE_V);
}

/*
 * Advance the vendor's own ring by one frame, matching its cadence: a plane
 * triplet every frame, and the tick group once per wrap of the five slots.
 *
 * This is the path taken whenever the capture node is not streaming, driven
 * either from the frame tick or by hand through debugfs. It writes to the
 * vendor's fixed DRAM addresses, which is what makes a capture read out of
 * /dev/mem possible and is why it is kept once the node exists.
 */
/*
 * Advance the frame cycle by one and issue the vendor's once-per-five-frames
 * maintenance write.
 *
 * Separate from the ring walk: the trace has the tick group written 496 times
 * over 2477 frames, once every five frames. Its coincidence with a wrap of the
 * vendor's five-slot ring follows from the ring having five slots; whether
 * the block reads the ring position is unestablished. Driven by the frame
 * count, so it runs the
 * same whether the output queue is the vendor's ring or the node's buffers.
 */
static void ar_cvisp_cycle(struct ar_cvisp *cv)
{
	cv->next = (cv->next + 1) % ARRAY_SIZE(ar_cvisp_ring);
	cv->frames++;

	if (!cv->next)
		ar_cvisp_apply(cv, ar_cvisp_tick, ARRAY_SIZE(ar_cvisp_tick));
}

static void ar_cvisp_queue(struct ar_cvisp *cv)
{
	ar_cvisp_arm(cv, cv->next);
	ar_cvisp_cycle(cv);
}

/* Point the block at one vb2 buffer's three planes. */
static void ar_cvisp_arm_buffer(struct ar_cvisp *cv,
				const struct ar_cvisp_buffer *buf)
{
	writel(buf->plane[0], cv->base + AR_CVISP_PLANE_Y);
	writel(buf->plane[1], cv->base + AR_CVISP_PLANE_U);
	writel(buf->plane[2], cv->base + AR_CVISP_PLANE_V);
}

/*
 * The ISP's per-frame tick. Runs in hard interrupt context.
 *
 * Without this the block is armed once at setup and never re-armed, so it
 * writes a single frame and then leaves the plane static. That is what made a
 * boot yield exactly one usable capture, and it is why every later bring-up
 * re-read the first one's frame.
 *
 * While the capture node streams, the rotation walks the queued vb2 buffers
 * instead of the vendor's fixed ring, and a buffer is handed back depth ticks
 * after it was armed.
 *
 * A tick with nothing queued arms nothing and completes nothing. The block
 * keeps writing over the buffer it already has, dropping a frame; completing
 * one instead would either hand back a partly written buffer or leave the
 * write master pointed at memory returned to the allocator.
 */
static void ar_cvisp_frame_tick(void *ctx)
{
	struct ar_cvisp *cv = ctx;
	struct ar_cvisp_buffer *done;
	struct ar_cvisp_buffer *next;

	if (!rotate)
		return;

	spin_lock(&cv->buffer_lock);

	if (!cv->streaming) {
		ar_cvisp_queue(cv);
		cv->rotations++;
		spin_unlock(&cv->buffer_lock);
		return;
	}

	next = list_first_entry_or_null(&cv->buffer_list, struct ar_cvisp_buffer,
					list);
	if (!next) {
		cv->drops++;
		spin_unlock(&cv->buffer_lock);
		return;
	}

	list_del(&next->list);

	/* Shift the pipeline: the oldest leaves, the new one enters at the front. */
	done = cv->held[depth - 1];
	for (unsigned int i = depth - 1; i > 0; i--)
		cv->held[i] = cv->held[i - 1];

	cv->held[0] = next;

	ar_cvisp_arm_buffer(cv, next);
	ar_cvisp_cycle(cv);
	cv->rotations++;

	if (done) {
		done->vb.sequence = cv->sequence++;
		cv->completions++;
	}

	spin_unlock(&cv->buffer_lock);

	if (done) {
		done->vb.vb2_buf.timestamp = ktime_get_ns();
		done->vb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&done->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}
}

/*
 * A float32 from the tuning file as a truncated unsigned integer.
 *
 * The kernel has no FPU, so the ladder's bounds are decoded from their IEEE-754
 * bit patterns. Every value in this ladder is a small positive integer, so the
 * general cases only exist to keep a corrupt file from producing nonsense.
 */
static u32 ar_cvisp_f32_to_u32(u32 bits)
{
	u32 mant = ar_isp_f32_mant(bits);
	int exp = ar_isp_f32_exp(bits);

	if (bits & AR_ISP_F32_SIGN || exp < 0)
		return 0;

	if (exp > 30)
		return U32_MAX;

	if (exp >= AR_ISP_F32_MANT_BITS)
		return mant << (exp - AR_ISP_F32_MANT_BITS);

	return mant >> (AR_ISP_F32_MANT_BITS - exp);
}

/*
 * Black level correction, from the tuning file, as a function of sensor gain.
 *
 * The stage is sixteen registers on this block filled by a verbatim 64-byte
 * copy. Its payload is five calibration entries selected by a ladder of float
 * pairs: a gain inside a pair's own range uses that entry alone, and a gain
 * between one pair's second bound and the next pair's first bound blends the
 * two entries across that band. Recovered from the selection at 0x1bfef8 and
 * the blend at 0x1c0048; formats are in ar-isp-blc.h.
 *
 *	t = (gain - ladder[lo].second) / (ladder[hi].first - ladder[lo].second)
 *	out = entry[hi] * t + entry[lo] * (1 - t)
 *
 * The default gain reproduces the vendor exactly. At 187 the band is 130 to
 * 510, t is 0.150000, and all four lanes come out at 961 and 272, which are the
 * 0xf040 and 0x110 the vendor was traced writing. That is the whole reason this
 * runs after the late table rather than instead of it: the constants that table
 * carries are the answer for one operating point, and this reproduces them from
 * the tuning file instead of carrying them.
 *
 * BLC recomputes with gain on the vendor, so once auto-exposure moves gain this
 * has to be re-applied with it. Nothing does that yet, which is safe only while
 * gain is static.
 */
static void ar_cvisp_blc_apply(struct ar_cvisp *cv)
{
	const struct firmware *fw;
	struct ar_isp_blc_entry lo, hi;
	u8 block[AR_ISP_BLC_BLOCK];
	u32 bound_lo, bound_hi, blend;
	unsigned int i, sel = 0;
	const u8 *ladder;

	if (!blc)
		return;

	int ret = request_firmware(&fw, AR_CVISP_TUNING_FIRMWARE, cv->dev);

	if (ret) {
		dev_info(cv->dev, "blc: no %s (%d), leaving the replayed constants\n",
			 AR_CVISP_TUNING_FIRMWARE, ret);
		return;
	}

	if (fw->size < AR_ISP_BLC_BLOB_TABLE +
		       AR_ISP_BLC_ENTRIES * AR_ISP_BLC_ENTRY_SIZE) {
		dev_warn(cv->dev, "blc: tuning file too short, leaving the replayed constants\n");
		release_firmware(fw);
		return;
	}

	ladder = fw->data + AR_ISP_BLC_BLOB_LADDER;

	/*
	 * Find the band the gain sits in. Walking upwards and stopping at the
	 * first pair whose second bound is above the gain puts a gain inside a
	 * pair's own range on that pair, where the two indices coincide and the
	 * entry is used without blending, exactly as the vendor's equal-index
	 * case does.
	 */
	for (i = 0; i + 1 < AR_ISP_BLC_ENTRIES; i++) {
		if (blc_gain < ar_cvisp_f32_to_u32(ar_isp_get_le32(ladder + i * 8 + 4)))
			break;

		sel = i;
	}

	bound_lo = ar_cvisp_f32_to_u32(ar_isp_get_le32(ladder + sel * 8 + 4));
	bound_hi = ar_cvisp_f32_to_u32(ar_isp_get_le32(ladder + (sel + 1) * 8));

	ar_isp_blc_entry(fw->data, sel, &lo);
	ar_isp_blc_entry(fw->data, sel + 1, &hi);

	/*
	 * ar_isp_blc_mix weights the low entry, so it takes 1 - t. A gain below
	 * the band uses the low entry alone and one above it the high entry,
	 * which is what clamping the weight to the Q16 endpoints does.
	 */
	if (blc_gain <= bound_lo || bound_hi <= bound_lo)
		blend = AR_ISP_BLC_BLEND_ONE;
	else if (blc_gain >= bound_hi)
		blend = 0;
	else
		blend = AR_ISP_BLC_BLEND_ONE -
			(u32)div_u64((u64)(blc_gain - bound_lo) * AR_ISP_BLC_BLEND_ONE,
				     bound_hi - bound_lo);

	ar_isp_blc_fill(block, &lo, &hi, blend);

	for (i = 0; i < AR_ISP_BLC_BLOCK; i += 4)
		writel(ar_isp_get_le32(block + i), cv->base + AR_ISP_BLC_BANK + i);

	dev_info(cv->dev,
		 "blc: gain %u in band %u..%u, entries %u/%u, scale 0x%08x level 0x%08x\n",
		 blc_gain, bound_lo, bound_hi, sel, sel + 1,
		 readl(cv->base + AR_ISP_BLC_BANK + AR_ISP_BLC_REG_SCALE),
		 readl(cv->base + AR_ISP_BLC_BANK + AR_ISP_BLC_REG_LEVEL));

	release_firmware(fw);
}

/*
 * Apply the whole recovered configuration. The setup table ends with the staged
 * enable, so the block is live once it returns, with ring set 0 armed. The late
 * table follows with frames already in flight; whether that ordering is
 * required is not established, which is why it can be held back.
 */
static void ar_cvisp_configure(struct ar_cvisp *cv, bool late)
{
	ar_cvisp_apply(cv, ar_cvisp_setup, ARRAY_SIZE(ar_cvisp_setup));

	if (late)
		ar_cvisp_apply(cv, ar_cvisp_late, ARRAY_SIZE(ar_cvisp_late));

	/* After the late table, which carries the vendor's own BLC constants. */
	ar_cvisp_blc_apply(cv);

	if (fpn)
		writel(readl(cv->base + AR_CVISP_FPN_CTRL) & ~AR_CVISP_FPN_ENABLE,
		       cv->base + AR_CVISP_FPN_CTRL);

	/* Setup leaves ring set 0 armed, so the next rotation starts at 1. */
	cv->next = 1 % ARRAY_SIZE(ar_cvisp_ring);
	cv->frames = 1;
	cv->configured = true;

	dev_info(cv->dev,
		 "configured: %zu setup + %zu late, control 0x%08x, plane_y 0x%08x\n",
		 ARRAY_SIZE(ar_cvisp_setup), late ? ARRAY_SIZE(ar_cvisp_late) : 0,
		 readl(cv->base + AR_CVISP_CONTROL),
		 readl(cv->base + AR_CVISP_PLANE_Y));
}

/*
 * Writing 1 applies setup and late; 2 applies setup only, to test whether the
 * late tail has to follow the enable or can be held back.
 */
static int ar_cvisp_configure_set(void *data, u64 val)
{
	if (val != 1 && val != 2)
		return -EINVAL;

	struct ar_cvisp *cv = data;

	ar_cvisp_configure(cv, val == 1);

	return 0;
}

static int ar_cvisp_configure_get(void *data, u64 *val)
{
	struct ar_cvisp *cv = data;

	*val = cv->configured;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ar_cvisp_configure_fops, ar_cvisp_configure_get,
			 ar_cvisp_configure_set, "%llu\n");

/* Writing n advances the queue n frames. Reading reports frames armed so far. */
static int ar_cvisp_queue_set(void *data, u64 val)
{
	struct ar_cvisp *cv = data;
	unsigned long flags;

	if (!cv->configured)
		return -EAGAIN;

	/* Against the frame tick, which drives the same ring from interrupt
	 * context and would otherwise interleave its plane writes with these.
	 */
	spin_lock_irqsave(&cv->buffer_lock, flags);

	if (cv->streaming) {
		spin_unlock_irqrestore(&cv->buffer_lock, flags);
		return -EBUSY;
	}

	while (val--)
		ar_cvisp_queue(cv);

	spin_unlock_irqrestore(&cv->buffer_lock, flags);

	return 0;
}

static int ar_cvisp_queue_get(void *data, u64 *val)
{
	struct ar_cvisp *cv = data;

	*val = cv->frames;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ar_cvisp_queue_fops, ar_cvisp_queue_get,
			 ar_cvisp_queue_set, "%llu\n");

static int ar_cvisp_regs_show(struct seq_file *s, void *unused)
{
	struct ar_cvisp *cv = s->private;

	for (unsigned int i = 0; i < ARRAY_SIZE(ar_cvisp_dump_regs); i++)
		seq_printf(s, "%-12s 0x%04x 0x%08x\n", ar_cvisp_dump_regs[i].name,
			   ar_cvisp_dump_regs[i].off,
			   readl(cv->base + ar_cvisp_dump_regs[i].off));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ar_cvisp_regs);

/*
 * The capture node.
 *
 * Three planes at three independently programmed base addresses is exactly what
 * the block wants, so the node is multiplanar and the plane spacing the vendor's
 * ring happens to use stops mattering: every base is written every frame from
 * the same table the vendor writes it from, so nothing derives U or V from Y.
 *
 * Geometry is not negotiable. It is the geometry ar_cvisp_setup configures the
 * block for, and changing it would mean changing that replay.
 */
static void ar_cvisp_fill_format(struct v4l2_pix_format_mplane *pix)
{
	memset(pix, 0, sizeof(*pix));

	pix->width = AR_CVISP_WIDTH;
	pix->height = AR_CVISP_HEIGHT;
	pix->pixelformat = V4L2_PIX_FMT_YUV420M;
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = AR_CVISP_PLANES;

	/*
	 * Not established. The RGB to YUV stage has not been located:
	 * ar-isp-colour.h covers the colour correction matrix, which is RGB to
	 * RGB, and no CSC bank has been identified in the ISP or in this block.
	 * Reported as unknown so no consumer inherits an invented value.
	 *
	 * Its luma coefficients decide it: 0.2126/0.7152/0.0722 is 709,
	 * 0.299/0.587/0.114 is 601.
	 */
	pix->colorspace = V4L2_COLORSPACE_DEFAULT;

	/* sizeimage is the buffer, which carries the padding rows; the payload set
	 * at prepare is the written extent.
	 */
	pix->plane_fmt[0].bytesperline = AR_CVISP_Y_STRIDE;
	pix->plane_fmt[0].sizeimage = AR_CVISP_Y_ALLOC;
	pix->plane_fmt[1].bytesperline = AR_CVISP_C_STRIDE;
	pix->plane_fmt[1].sizeimage = AR_CVISP_C_ALLOC;
	pix->plane_fmt[2].bytesperline = AR_CVISP_C_STRIDE;
	pix->plane_fmt[2].sizeimage = AR_CVISP_C_ALLOC;
}

static int ar_cvisp_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
				unsigned int *num_planes, unsigned int sizes[],
				struct device *alloc_devs[])
{
	static const unsigned int plane_size[AR_CVISP_PLANES] = {
		AR_CVISP_Y_ALLOC,
		AR_CVISP_C_ALLOC,
		AR_CVISP_C_ALLOC,
	};

	/* A non-zero plane count is a layout to validate, from CREATE_BUFS with a
	 * format; zero is a request for the one layout this block produces.
	 */
	if (*num_planes) {
		if (*num_planes != AR_CVISP_PLANES)
			return -EINVAL;

		for (unsigned int i = 0; i < ARRAY_SIZE(plane_size); i++)
			if (sizes[i] < plane_size[i])
				return -EINVAL;

		return 0;
	}

	*num_planes = AR_CVISP_PLANES;
	for (unsigned int i = 0; i < ARRAY_SIZE(plane_size); i++)
		sizes[i] = plane_size[i];

	return 0;
}

static int ar_cvisp_buffer_prepare(struct vb2_buffer *vb)
{
	static const unsigned int plane_alloc[AR_CVISP_PLANES] = {
		AR_CVISP_Y_ALLOC,
		AR_CVISP_C_ALLOC,
		AR_CVISP_C_ALLOC,
	};
	static const unsigned int plane_written[AR_CVISP_PLANES] = {
		AR_CVISP_Y_SIZE,
		AR_CVISP_C_SIZE,
		AR_CVISP_C_SIZE,
	};
	struct ar_cvisp_buffer *buf = to_ar_cvisp_buffer(to_vb2_v4l2_buffer(vb));

	for (unsigned int i = 0; i < ARRAY_SIZE(plane_alloc); i++) {
		if (vb2_plane_size(vb, i) < plane_alloc[i])
			return -EINVAL;

		vb2_set_plane_payload(vb, i, plane_written[i]);

		/*
		 * The block takes a 32-bit base. Buffers come from the cvisp_cma
		 * reservation, which is well inside 4 GiB, so this only catches a
		 * device tree that has been changed out from under the driver.
		 */
		dma_addr_t addr = vb2_dma_contig_plane_dma_addr(vb, i);

		if (upper_32_bits(addr))
			return -EINVAL;

		buf->plane[i] = lower_32_bits(addr);
	}

	return 0;
}

static void ar_cvisp_buffer_queue(struct vb2_buffer *vb)
{
	struct ar_cvisp *cv = vb2_get_drv_priv(vb->vb2_queue);
	struct ar_cvisp_buffer *buf = to_ar_cvisp_buffer(to_vb2_v4l2_buffer(vb));
	unsigned long flags;

	spin_lock_irqsave(&cv->buffer_lock, flags);
	list_add_tail(&buf->list, &cv->buffer_list);
	spin_unlock_irqrestore(&cv->buffer_lock, flags);
}

/* Hand every buffer this driver holds back in the given state. */
static void ar_cvisp_return_buffers(struct ar_cvisp *cv,
				    enum vb2_buffer_state state)
{
	struct ar_cvisp_buffer *buf;
	struct ar_cvisp_buffer *tmp;
	unsigned long flags;
	LIST_HEAD(pending);

	spin_lock_irqsave(&cv->buffer_lock, flags);

	for (unsigned int i = 0; i < AR_CVISP_MAX_DEPTH; i++) {
		if (cv->held[i])
			list_add_tail(&cv->held[i]->list, &pending);

		cv->held[i] = NULL;
	}

	list_splice_tail_init(&cv->buffer_list, &pending);

	spin_unlock_irqrestore(&cv->buffer_lock, flags);

	list_for_each_entry_safe(buf, tmp, &pending, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

/*
 * Bring the chain up ahead of this block, once.
 *
 * The order is the one every validated bring-up uses: the VIF input path and
 * the sensor first, then the ISP. That is not the vendor's order, which starts
 * its sinks before its source; it is this stack's, and it exists because the
 * ISP configuration reads registers and a read with the pixel domain dead hangs
 * the SoC. ar_vif_input_start returns only once frames are confirmed flowing,
 * which is what makes the ISP step safe rather than a race against a sleep.
 *
 * Done once and never undone. A warm re-bring-up of these blocks completes
 * without error and then never writes DRAM again, so tearing the chain down at
 * STREAMOFF would make the second STREAMON of a boot silently produce nothing.
 * The cost is that the sensor keeps running between streams.
 */
static int ar_cvisp_chain_start(struct ar_cvisp *cv)
{
	if (cv->chain_up)
		return 0;

	int ret = ar_vif_input_start();

	if (ret == -EBUSY) {
		/* Already brought up from outside, which is how the capture
		 * harness runs. Nothing to do and not an error.
		 */
		dev_info(cv->dev, "input path already running\n");
	} else if (ret) {
		dev_err(cv->dev, "cannot start the input path (%d)\n", ret);
		return ret;
	}

	ret = ar_isp_pipeline_start();
	if (ret) {
		dev_err(cv->dev, "cannot start the isp (%d)\n", ret);
		return ret;
	}

	cv->chain_up = true;

	return 0;
}

/*
 * STREAMON takes the output queue over from the vendor ring, and brings the
 * chain up ahead of it if nothing else has. The block itself has to be
 * configured before its ring means anything, so that happens here too when
 * nobody has done it.
 */
static int ar_cvisp_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct ar_cvisp *cv = vb2_get_drv_priv(q);
	struct ar_cvisp_buffer *first;
	unsigned long flags;

	if (!rotate) {
		dev_err(cv->dev, "rotate=0: nothing would advance the queue\n");
		ar_cvisp_return_buffers(cv, VB2_BUF_STATE_QUEUED);
		return -EINVAL;
	}

	if (chain) {
		int ret = ar_cvisp_chain_start(cv);

		if (ret) {
			ar_cvisp_return_buffers(cv, VB2_BUF_STATE_QUEUED);
			return ret;
		}
	}

	if (!cv->configured)
		ar_cvisp_configure(cv, true);

	spin_lock_irqsave(&cv->buffer_lock, flags);

	first = list_first_entry_or_null(&cv->buffer_list, struct ar_cvisp_buffer,
					 list);
	if (!first) {
		spin_unlock_irqrestore(&cv->buffer_lock, flags);
		ar_cvisp_return_buffers(cv, VB2_BUF_STATE_QUEUED);
		return -EINVAL;
	}

	list_del(&first->list);
	memset(cv->held, 0, sizeof(cv->held));
	cv->held[0] = first;
	cv->sequence = 0;
	ar_cvisp_arm_buffer(cv, first);

	/* Last, so the tick cannot see a half-set-up queue. */
	cv->streaming = true;

	spin_unlock_irqrestore(&cv->buffer_lock, flags);

	dev_info(cv->dev, "streaming: %ux%u YUV420M, %u buffers\n",
		 AR_CVISP_WIDTH, AR_CVISP_HEIGHT, count);

	return 0;
}

static void ar_cvisp_stop_streaming(struct vb2_queue *q)
{
	struct ar_cvisp *cv = vb2_get_drv_priv(q);
	unsigned long flags;

	spin_lock_irqsave(&cv->buffer_lock, flags);

	cv->streaming = false;

	/*
	 * Put the vendor ring back under the block before the buffers go. The
	 * tick keeps firing after STREAMOFF, and it must not be left pointing at
	 * memory that has been handed back to the allocator.
	 *
	 * Under the same lock as the tick: its not-streaming branch writes this
	 * plane triplet and this index from ar_cvisp_queue, so an unlocked write
	 * here interleaves with it and publishes a mixed slot.
	 */
	ar_cvisp_arm(cv, 0);
	cv->next = 1 % ARRAY_SIZE(ar_cvisp_ring);

	spin_unlock_irqrestore(&cv->buffer_lock, flags);

	ar_cvisp_return_buffers(cv, VB2_BUF_STATE_ERROR);

	dev_info(cv->dev, "stopped: %u completions, %u drops\n",
		 cv->completions, cv->drops);
}

static const struct vb2_ops ar_cvisp_vb2_ops = {
	.queue_setup = ar_cvisp_queue_setup,
	.buf_prepare = ar_cvisp_buffer_prepare,
	.buf_queue = ar_cvisp_buffer_queue,
	.start_streaming = ar_cvisp_start_streaming,
	.stop_streaming = ar_cvisp_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int ar_cvisp_querycap(struct file *file, void *priv,
			     struct v4l2_capability *cap)
{
	strscpy(cap->driver, "ar-cvisp", sizeof(cap->driver));
	strscpy(cap->card, "Artosyn CVISP", sizeof(cap->card));

	return 0;
}

static int ar_cvisp_enum_fmt(struct file *file, void *priv,
			     struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_YUV420M;

	return 0;
}

static int ar_cvisp_get_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	ar_cvisp_fill_format(&f->fmt.pix_mp);

	return 0;
}

static int ar_cvisp_enum_framesizes(struct file *file, void *priv,
				    struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || fsize->pixel_format != V4L2_PIX_FMT_YUV420M)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = AR_CVISP_WIDTH;
	fsize->discrete.height = AR_CVISP_HEIGHT;

	return 0;
}

static const struct v4l2_ioctl_ops ar_cvisp_ioctl_ops = {
	.vidioc_querycap = ar_cvisp_querycap,
	.vidioc_enum_fmt_vid_cap = ar_cvisp_enum_fmt,
	.vidioc_enum_framesizes = ar_cvisp_enum_framesizes,
	/* One geometry, so get, set and try are the same answer. */
	.vidioc_g_fmt_vid_cap_mplane = ar_cvisp_get_fmt,
	.vidioc_s_fmt_vid_cap_mplane = ar_cvisp_get_fmt,
	.vidioc_try_fmt_vid_cap_mplane = ar_cvisp_get_fmt,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations ar_cvisp_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

static int ar_cvisp_register_node(struct ar_cvisp *cv)
{
	int ret = v4l2_device_register(cv->dev, &cv->v4l2_dev);

	if (ret)
		return ret;

	struct vb2_queue *q = &cv->queue;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = cv;
	q->buf_struct_size = sizeof(struct ar_cvisp_buffer);
	q->ops = &ar_cvisp_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &cv->lock;
	q->dev = cv->dev;

	/*
	 * One more than the hardware holds. With exactly as many, all are
	 * consumed before any has been handed back and the next tick finds an
	 * empty queue, which userspace can only refill from a buffer it has not
	 * been given.
	 */
	q->min_queued_buffers = depth + 1;

	ret = vb2_queue_init(q);
	if (ret) {
		v4l2_device_unregister(&cv->v4l2_dev);
		return ret;
	}

	cv->video_dev.fops = &ar_cvisp_fops;
	cv->video_dev.ioctl_ops = &ar_cvisp_ioctl_ops;
	cv->video_dev.v4l2_dev = &cv->v4l2_dev;
	cv->video_dev.queue = q;
	cv->video_dev.lock = &cv->lock;
	cv->video_dev.release = video_device_release_empty;
	cv->video_dev.device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
				    V4L2_CAP_STREAMING;
	cv->video_dev.vfl_dir = VFL_DIR_RX;
	strscpy(cv->video_dev.name, "ar-cvisp", sizeof(cv->video_dev.name));
	video_set_drvdata(&cv->video_dev, cv);

	ret = video_register_device(&cv->video_dev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		/*
		 * The queue is initialised but no video device took ownership of
		 * it, so vb2_video_unregister_device will never run for it. This
		 * is the only path that has to release the queue by hand.
		 */
		vb2_queue_release(q);
		v4l2_device_unregister(&cv->v4l2_dev);
		return ret;
	}

	return 0;
}

static void ar_cvisp_release_rmem(void *dev)
{
	of_reserved_mem_device_release(dev);
}

static int ar_cvisp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ar_cvisp *cv;

	/*
	 * Capture buffers must come out of cvisp_cma, the no-map reservation the
	 * vendor's own ring addresses sit in: the block's AXI write master
	 * reaches it and it has no cacheable kernel alias, so no speculative
	 * writeback can land over a frame. Without the pool vb2 would fall back
	 * to the default CMA, which is ordinary kernel RAM.
	 */
	int ret;

	/*
	 * The plane registers carry a 32-bit address, so constrain the DMA API
	 * before the pool is attached and before vb2 can allocate or import.
	 * ar_cvisp_buffer_prepare still rejects an unreachable address: the pool
	 * is placed by the DTS, which the mask does not bound.
	 */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA mask\n");

	ret = of_reserved_mem_device_init(dev);
	if (ret)
		dev_warn(dev,
			 "no dedicated capture pool (%d); buffers may be unreachable\n",
			 ret);

	/*
	 * Unwound through devres so every probe failure below releases it too.
	 * Registered even when the init failed: the release matches on the
	 * device and does nothing when nothing was attached.
	 */
	ret = devm_add_action_or_reset(dev, ar_cvisp_release_rmem, dev);
	if (ret)
		return ret;

	cv = devm_kzalloc(dev, sizeof(*cv), GFP_KERNEL);
	if (!cv)
		return -ENOMEM;

	if (depth < 1 || depth > AR_CVISP_MAX_DEPTH)
		return dev_err_probe(dev, -EINVAL, "depth must be 1 or %u\n",
				     AR_CVISP_MAX_DEPTH);

	cv->dev = dev;
	mutex_init(&cv->lock);
	spin_lock_init(&cv->buffer_lock);
	INIT_LIST_HEAD(&cv->buffer_list);

	cv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cv->base))
		return PTR_ERR(cv->base);

	/*
	 * cgu_rsz_clk, referenced but deliberately NOT enabled by default. The
	 * vendor never enables it: no clock request in the CVISP path of
	 * libmpp_service.so, no CGU write in the trace, and stock-A baselines
	 * read 0x12011100 at 0x0a104014 with gate bit 12 already set by the
	 * boot firmware.
	 *
	 * Taking ownership would be harmful on the way out: the leaf is
	 * gate-modelled, so clk_disable_unprepare on remove would clear a gate
	 * the boot had set, and register access with the clock gated hangs the
	 * SoC on this family. Asserting it is opt-in.
	 */
	cv->clk = devm_clk_get_optional(dev, "rsz");
	if (IS_ERR(cv->clk))
		return dev_err_probe(dev, PTR_ERR(cv->clk), "bad rsz clock\n");

	if (assert_clk && cv->clk) {
		ret = clk_prepare_enable(cv->clk);
		if (ret)
			return dev_err_probe(dev, ret, "cannot enable rsz clock\n");
		cv->clk_asserted = true;
	}

	platform_set_drvdata(pdev, cv);

	cv->debugfs = debugfs_create_dir("ar-cvisp", NULL);
	debugfs_create_file_unsafe("configure", 0600, cv->debugfs, cv,
				   &ar_cvisp_configure_fops);
	debugfs_create_file_unsafe("queue", 0600, cv->debugfs, cv,
				   &ar_cvisp_queue_fops);
	debugfs_create_file("regs", 0400, cv->debugfs, cv, &ar_cvisp_regs_fops);
	debugfs_create_u32("rotations", 0400, cv->debugfs, &cv->rotations);
	debugfs_create_u32("completions", 0400, cv->debugfs, &cv->completions);
	debugfs_create_u32("drops", 0400, cv->debugfs, &cv->drops);

	ret = ar_cvisp_register_node(cv);
	if (ret) {
		debugfs_remove_recursive(cv->debugfs);
		if (cv->clk_asserted)
			clk_disable_unprepare(cv->clk);

		return dev_err_probe(dev, ret, "cannot register the video node\n");
	}

	/*
	 * Last, so the tick cannot fire into a driver whose queue is not up yet.
	 */
	ar_isp_set_frame_hook(ar_cvisp_frame_tick, cv);

	/*
	 * Deliberately no register read here. If the clock assumption above is
	 * wrong, the first access hangs the SoC, and a probe-time read would
	 * make that unavoidable on every boot. Reading debugfs regs is the
	 * first touch, and it is a deliberate one.
	 */
	dev_info(dev, "probed, %zu setup + %zu late registers, %zu ring slots, capture on %s\n",
		 ARRAY_SIZE(ar_cvisp_setup), ARRAY_SIZE(ar_cvisp_late),
		 ARRAY_SIZE(ar_cvisp_ring),
		 video_device_node_name(&cv->video_dev));

	return 0;
}

static void ar_cvisp_remove(struct platform_device *pdev)
{
	struct ar_cvisp *cv = platform_get_drvdata(pdev);

	/*
	 * First, and before the clock is dropped. The hook runs from the ISP's
	 * interrupt and writes this block's registers, so a tick arriving after
	 * teardown would touch a clock-gated block and hang the SoC. The ISP
	 * holds its lock across the call, so this returns only once no callback
	 * is still running.
	 */
	ar_isp_set_frame_hook(NULL, NULL);

	/*
	 * Only now, with no tick left running, is it safe to take the queue
	 * down: vb2_video_unregister_device stops any stream in progress and
	 * returns the buffers, and the tick writes their addresses.
	 */
	vb2_video_unregister_device(&cv->video_dev);
	v4l2_device_unregister(&cv->v4l2_dev);

	debugfs_remove_recursive(cv->debugfs);

	/* Only ever release what this driver asserted. Clearing an inherited gate
	 * leaves the block unclocked for the next thing that touches it.
	 */
	if (cv->clk_asserted)
		clk_disable_unprepare(cv->clk);
}

static const struct of_device_id ar_cvisp_of_match[] = {
	{ .compatible = "artosyn,cvisp" },
	{ }
};
MODULE_DEVICE_TABLE(of, ar_cvisp_of_match);

static struct platform_driver ar_cvisp_driver = {
	.probe = ar_cvisp_probe,
	.remove = ar_cvisp_remove,
	.driver = {
		.name = "ar-cvisp",
		.of_match_table = ar_cvisp_of_match,
	},
};
module_platform_driver(ar_cvisp_driver);

MODULE_DESCRIPTION("Artosyn CVISP output stage");
MODULE_LICENSE("GPL");
