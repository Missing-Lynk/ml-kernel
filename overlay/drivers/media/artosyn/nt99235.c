// SPDX-License-Identifier: GPL-2.0
/*
 * nt99235.c - Novatek NT99235 MIPI CSI-2 image sensor subdev.
 *
 * The sensor sits on i2c0 at 7-bit address 0x1a on the BetaFPV VR04 air unit
 * and emits 12-bit linear Bayer over MIPI CSI-2 (data type 0x2c, RAW12).
 *
 * There is no public datasheet. The mode register sequences below were
 * extracted from the vendor sensor library libsns_nt99235.so, which performs
 * them as inline register writes rather than table walks; the tables here are a
 * mechanical transcription of those writes in program order, so they are opaque
 * by nature and are deliberately not annotated beyond the register groups the
 * SMIA standard names.
 *
 * Four modes exist, distinguished by the lane count each one programs into the
 * SMIA lane-mode register 0x0114 (0x01 for two lanes, 0x03 for four) as part of
 * its own table. The air unit runs the 2-lane 1920x1080 mode; the
 * 4-lane modes are carried because the sensor supports them, but whether this
 * board routes four lanes is unconfirmed, so they are rejected unless the
 * device tree endpoint declares four data lanes.
 */

#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Chip identity. The vendor probe reads NT99235_REG_MODEL_ID_ALT and only
 * checks that the transfer succeeds; registers 0x0000/0x0001 carry the real
 * 0x9235 model id, confirmed by reading them on hardware.
 */
#define NT99235_REG_MODEL_ID_HI		0x0000
#define NT99235_REG_MODEL_ID_LO		0x0001
#define NT99235_MODEL_ID		0x9235

/* SMIA-standard controls used outside the opaque mode tables. */
#define NT99235_REG_MODE_SELECT		0x0100
#define NT99235_MODE_STANDBY		0x00
#define NT99235_MODE_STREAMING		0x01

/*
 * Exposure and gain, recovered from the vendor's AE commit list in
 * libsns_nt99235.so (cmos_get_sns_regs_info). Both are SMIA addresses, like
 * everything else the mode tables touch outside the 0x3xxx and 0x8xxx Novatek
 * ranges.
 *
 * Exposure is a 16-bit big-endian count of line periods, clamped by the vendor
 * to vts - 2. Gain is a code written identically to both 0x0206 and 0x0207.
 *
 * The group hold is required: the vendor brackets every dirty
 * entry of a commit with 0x0104 = 1 then 0x0104 = 0, so a frame never sees a
 * half-updated exposure and gain pair.
 *
 * Not to be confused with 0x8250-0x826c and 0x8550-0x855c, which drift at
 * runtime on a streaming vendor and look like the obvious candidates. They are
 * sensor-MCU lens shading tables driven by the AWB path, and reloaded with
 * 0x8201. They have nothing to do with brightness.
 */
#define NT99235_REG_GROUP_HOLD		0x0104
#define NT99235_REG_EXPOSURE_HI		0x0202
#define NT99235_REG_EXPOSURE_LO		0x0203
#define NT99235_REG_AGAIN_0		0x0206
#define NT99235_REG_AGAIN_1		0x0207

/*
 * Analogue gain is an index into a 97-entry quantisation table in the vendor
 * library, not a linear register value. The control carries the raw sensor code,
 * so nothing here has to reproduce the table.
 *
 * The mapping is closed form, and this is measured rather than assumed. The
 * whole table was extracted from libsns_nt99235.so (virtual address 0x1a130,
 * file offset 0xa130 through the second PT_LOAD, 97 u32 entries with 1024
 * meaning 1x) and compared against
 *
 *	gain = 2^(code >> 4) * (16 + (code & 0xf)) / 16
 *
 * which reproduces all 97 entries exactly: four-bit mantissa, four-bit binary
 * exponent, 1x at code 0x00 rising to 64x at code 0x60. An AE loop may use the
 * formula directly; it reproduces the table exactly.
 */
#define NT99235_AGAIN_MIN		0x00
#define NT99235_AGAIN_MAX		0x60
#define NT99235_AGAIN_DEFAULT		0x3c	/* the vendor's own operating point */

/*
 * Vendor's observed indoor starting point, in lines. Deliberately near the
 * ceiling: the sensor runs at power-on defaults otherwise and the resulting
 * frame is black, so erring bright makes a first capture legible. Clamped to
 * the mode's vts - 2 in any case.
 */
#define NT99235_EXPOSURE_DEFAULT	1123
#define NT99235_EXPOSURE_MIN		1

/*
 * Live exposure and gain, as writable module parameters.
 *
 * These duplicate the V4L2 controls on purpose. The controls are the right
 * interface and are what an application would use, but nothing on the device
 * can reach them: the rootfs has no v4l2-ctl and ml-v4l2grab does not implement
 * VIDIOC_S_CTRL, so on a bring-up unit the controls are only ever set from
 * their compiled-in defaults. Writing
 *
 *	echo 400 > /sys/module/nt99235/parameters/exposure
 *
 * retunes a running sensor, which is what sweeping for a working value needs
 * when the alternative is a rebuild and a reboot per sample.
 *
 * The setter commits to hardware and syncs the matching V4L2 control, so the
 * two interfaces cannot disagree.
 */
static struct nt99235 *nt99235_bound;
static DEFINE_MUTEX(nt99235_bound_lock);

static int exposure = NT99235_EXPOSURE_DEFAULT;
static int gain = NT99235_AGAIN_DEFAULT;

/* Master clock rate the vendor programs before releasing reset. */
#define NT99235_MCLK_RATE		24000000

/* Power-on settle times, from nt99235_cmos_power_on. */
#define NT99235_RESET_SETTLE_US		10000

/*
 * Bring-up staging. Probing this driver is the first thing that ever drives
 * the sensor's clock and control lines on an open kernel, and any one of those
 * steps can wedge the machine if a device tree value is wrong. The module
 * therefore does nothing to the hardware by default: each stage is opted into
 * explicitly, so a hang identifies its own cause without needing a bisect
 * across reboots.
 *
 *   0  register the subdev only, touch no hardware (default)
 *   1  additionally enable the master clock
 *   2  additionally drive the power and reset lines
 *   3  additionally read the chip id over i2c
 *
 * Streaming always performs the full sequence regardless of this setting; it
 * only governs what probe does.
 */
#define NT99235_BRINGUP_NONE		0
#define NT99235_BRINGUP_CLOCK		1
#define NT99235_BRINGUP_GPIO		2
#define NT99235_BRINGUP_DETECT		3

static int bringup = NT99235_BRINGUP_NONE;
module_param(bringup, int, 0444);
MODULE_PARM_DESC(bringup,
		 "probe-time hardware bring-up stage: 0 none, 1 clock, 2 gpio, 3 detect");

/*
 * Mode override. Modes are normally filtered by the lane count the device tree
 * endpoint declares, so the four-lane modes are hidden on a board wired for
 * two. Selecting one explicitly bypasses that filter, which is what a test of
 * how many lanes the board actually routes needs.
 */
static int force_mode = -1;
module_param(force_mode, int, 0444);
MODULE_PARM_DESC(force_mode,
		 "select a sensor mode by index regardless of the declared lane count (-1 to choose automatically)");

/*
 * Test pattern. Nothing sets exposure or gain on this stack, so a correct
 * capture is black, and a black frame cannot be told apart from a broken one by
 * looking at it. A generated pattern removes the sensor's optics and exposure
 * from the question entirely: content of known shape arriving in DRAM proves
 * sensor, CSI-2, VIF, ISP and CVISP end to end, and gives known-good data to
 * check the recovered geometry, stride and plane layout against.
 *
 * The register is INFERRED, not confirmed. The vendor library never writes it,
 * in any mode, which is what production firmware would do either way. The
 * inference is that the NT99235 carries the SMIA register block: the mode tables
 * use SMIA addresses throughout (0x0100 mode select, 0x0340 frame length, 0x0342
 * line length, 0x0344-0x034b analogue crop, 0x034c output size, 0x0381-0x0387
 * subsampling, 0x0408-0x040f digital crop, 0x0900 binning), and in that map
 * TEST_PATTERN_MODE is a 16-bit register at 0x0600.
 *
 * nt99235_apply_test_pattern reads the register back and logs it, so a run
 * settles whether the register exists at all without needing to judge an image:
 * a readback matching what was written means it is implemented, and a readback
 * stuck at zero means the inference is wrong and the address is elsewhere.
 */
#define NT99235_REG_TEST_PATTERN_HI	0x0600
#define NT99235_REG_TEST_PATTERN_LO	0x0601

#define NT99235_TEST_PATTERN_OFF	0
#define NT99235_TEST_PATTERN_SOLID	1
#define NT99235_TEST_PATTERN_BARS	2
#define NT99235_TEST_PATTERN_FADE	3
#define NT99235_TEST_PATTERN_PN9	4

/*
 * Dump the SMIA identification and capability blocks at stream-on. Off by
 * default: it is 21 extra I2C reads and it only needs running once per sensor.
 */
static bool dump_smia;
module_param(dump_smia, bool, 0644);
MODULE_PARM_DESC(dump_smia,
		 "log the SMIA read-only ID and capability registers at stream-on (default off)");

static int test_pattern = NT99235_TEST_PATTERN_OFF;
module_param(test_pattern, int, 0644);
MODULE_PARM_DESC(test_pattern,
		 "SMIA test pattern at stream-on: 0 off, 1 solid, 2 colour bars, 3 fade to grey, 4 PN9 (register inferred, readback is logged)");

struct nt99235_reg {
	u16 address;
	u8 value;
};

/* 960x540 @ 120, 2 lanes; 200 writes, verbatim from libsns_nt99235.so 0x01b0c. */
static const struct nt99235_reg nt99235_regs_960x540p120[] = {
	{ 0x3611, 0x30 }, { 0x3612, 0x00 }, { 0x3247, 0x00 }, { 0x36e7, 0x01 },
	{ 0x3537, 0x10 }, { 0x3539, 0x04 }, { 0x303b, 0x03 }, { 0x3672, 0x07 },
	{ 0x3673, 0x10 }, { 0x36b2, 0x74 }, { 0x36b3, 0x83 }, { 0x36b4, 0xf3 },
	{ 0x36b5, 0xf0 }, { 0x3682, 0x51 }, { 0x3683, 0x13 }, { 0x3684, 0x71 },
	{ 0x3685, 0x71 }, { 0x3686, 0x71 }, { 0x3687, 0x02 }, { 0x325c, 0x1f },
	{ 0x303a, 0x04 }, { 0x303b, 0x05 }, { 0x323e, 0x10 }, { 0x36a2, 0x00 },
	{ 0x325d, 0x16 }, { 0x325e, 0x0f }, { 0x32ca, 0x01 }, { 0x32cb, 0x0c },
	{ 0x32cc, 0x03 }, { 0x32cd, 0x28 }, { 0x32d6, 0x01 }, { 0x32d7, 0x0c },
	{ 0x32d8, 0x1b }, { 0x32d9, 0xbf }, { 0x3342, 0x2a }, { 0x3343, 0x10 },
	{ 0x336a, 0x18 }, { 0x3307, 0x00 }, { 0x3308, 0x3c }, { 0x3316, 0x00 },
	{ 0x3317, 0x5f }, { 0x3325, 0x2c }, { 0x3332, 0x0f }, { 0x335a, 0x1a },
	{ 0x336c, 0x00 }, { 0x336d, 0x2d }, { 0x336e, 0x00 }, { 0x336f, 0x88 },
	{ 0x36c0, 0x33 }, { 0x36c1, 0x33 }, { 0x32f3, 0x02 }, { 0x32f5, 0x03 },
	{ 0x32f6, 0x84 }, { 0x3669, 0x53 }, { 0x3681, 0x73 }, { 0x3692, 0x07 },
	{ 0x36e1, 0xff }, { 0x36e3, 0xbf }, { 0x36e4, 0x07 }, { 0x3327, 0x00 },
	{ 0x3328, 0x5a }, { 0x3334, 0x00 }, { 0x3335, 0x4c }, { 0x337a, 0x00 },
	{ 0x337b, 0x2d }, { 0x3618, 0x86 }, { 0x3619, 0x77 }, { 0x361b, 0x02 },
	{ 0x3690, 0x43 }, { 0x3252, 0x10 }, { 0x3316, 0x00 }, { 0x3318, 0x46 },
	{ 0x32d6, 0x01 }, { 0x32d7, 0xce }, { 0x337a, 0x00 }, { 0x337b, 0x50 },
	{ 0x3317, 0x1e }, { 0x331d, 0x50 }, { 0x32e1, 0x03 }, { 0x32c9, 0x00 },
	{ 0x3532, 0x03 }, { 0x3a02, 0xa1 }, { 0x3a0a, 0x03 }, { 0x0b04, 0x01 },
	{ 0x0138, 0x01 }, { 0x82e3, 0x03 }, { 0x833f, 0x01 }, { 0x0b06, 0x01 },
	{ 0x803f, 0x20 }, { 0x0350, 0x00 }, { 0x0100, 0x00 }, { 0x0136, 0x18 },
	{ 0x0137, 0x00 }, { 0x0114, 0x01 }, { 0x0303, 0x01 }, { 0x0305, 0x02 },
	{ 0x0307, 0x63 }, { 0x0309, 0x0a }, { 0x030a, 0x08 }, { 0x030b, 0x01 },
	{ 0x030f, 0x4b }, { 0x302d, 0x04 }, { 0x3500, 0x80 }, { 0x3502, 0xd0 },
	{ 0x0824, 0x14 }, { 0x3530, 0x40 }, { 0x3325, 0x2b }, { 0x0342, 0x04 },
	{ 0x0343, 0x4c }, { 0x0340, 0x02 }, { 0x0341, 0x32 }, { 0x0344, 0x00 },
	{ 0x0345, 0x00 }, { 0x0346, 0x00 }, { 0x0347, 0x00 }, { 0x0348, 0x07 },
	{ 0x0349, 0x7f }, { 0x034a, 0x04 }, { 0x034b, 0x37 }, { 0x0381, 0x01 },
	{ 0x0381, 0x01 }, { 0x0385, 0x01 },
	{ 0x0385, 0x01 }, { 0x0387, 0x03 }, { 0x0387, 0x03 }, { 0x0900, 0x01 },
	{ 0x0901, 0x21 }, { 0x0408, 0x00 }, { 0x0409, 0x00 }, { 0x040a, 0x00 },
	{ 0x040b, 0x00 }, { 0x040c, 0x03 }, { 0x040d, 0xc4 }, { 0x040e, 0x02 },
	{ 0x040f, 0x20 }, { 0x034c, 0x03 }, { 0x034d, 0xc4 }, { 0x034e, 0x02 },
	{ 0x034f, 0x20 }, { 0x0280, 0x01 }, { 0x8205, 0x03 }, { 0x82bb, 0x67 },
	{ 0x82b8, 0x06 }, { 0x82b9, 0x06 }, { 0x82bc, 0x38 }, { 0x82cb, 0x20 },
	{ 0x82bd, 0x03 }, { 0x82be, 0x03 }, { 0x82bf, 0x50 }, { 0x82c0, 0x40 },
	{ 0x82c1, 0x40 }, { 0x82c2, 0x40 }, { 0x82c3, 0x00 }, { 0x82c4, 0x3f },
	{ 0x82c5, 0x0f }, { 0x82c6, 0x1f }, { 0x82d3, 0x02 }, { 0x82d4, 0xa3 },
	{ 0x82d5, 0x95 }, { 0x82d6, 0x02 }, { 0x82d7, 0x33 }, { 0x82d8, 0x91 },
	{ 0x82e3, 0x03 }, { 0x8741, 0x3f }, { 0x8250, 0x29 }, { 0x8251, 0x15 },
	{ 0x8252, 0x00 }, { 0x8253, 0x38 }, { 0x8254, 0x23 }, { 0x8255, 0x30 },
	{ 0x8256, 0x1d }, { 0x8257, 0x26 }, { 0x8258, 0x13 }, { 0x8259, 0x3b },
	{ 0x825a, 0x26 }, { 0x825b, 0x2a }, { 0x825c, 0x15 }, { 0x825d, 0x80 },
	{ 0x825e, 0x04 }, { 0x8550, 0x2b }, { 0x8551, 0x17 }, { 0x8552, 0x00 },
	{ 0x8553, 0x3a }, { 0x8554, 0x25 }, { 0x8555, 0x32 }, { 0x8556, 0x1f },
	{ 0x8557, 0x28 }, { 0x8558, 0x15 }, { 0x8559, 0x3c }, { 0x855a, 0x27 },
	{ 0x855b, 0x2b }, { 0x855c, 0x17 }, { 0x826c, 0x80 }, { 0x826d, 0x04 },
	{ 0x826c, 0x20 }, { 0x9040, 0x09 }, { 0x9023, 0x60 }, { 0x8201, 0x0f },
};

/* 1920x1080 @ 60, 2 lanes (the mode the vendor runs); 194 writes, verbatim from libsns_nt99235.so 0x03440. */
static const struct nt99235_reg nt99235_regs_1920x1080p60[] = {
	{ 0x3611, 0x30 }, { 0x3612, 0x00 }, { 0x3247, 0x00 }, { 0x36e7, 0x01 },
	{ 0x3537, 0x10 }, { 0x3539, 0x04 }, { 0x303b, 0x03 }, { 0x3672, 0x07 },
	{ 0x3673, 0x10 }, { 0x36b2, 0x74 }, { 0x36b3, 0x83 }, { 0x36b4, 0xf3 },
	{ 0x36b5, 0xf0 }, { 0x3682, 0x51 }, { 0x3683, 0x13 }, { 0x3684, 0x71 },
	{ 0x3685, 0x71 }, { 0x3686, 0x71 }, { 0x3687, 0x02 }, { 0x325c, 0x1f },
	{ 0x303a, 0x04 }, { 0x303b, 0x05 }, { 0x323e, 0x10 }, { 0x36a2, 0x00 },
	{ 0x325d, 0x16 }, { 0x325e, 0x0f }, { 0x32ca, 0x01 }, { 0x32cb, 0x0c },
	{ 0x32cc, 0x03 }, { 0x32cd, 0x28 }, { 0x32d6, 0x01 }, { 0x32d7, 0x0c },
	{ 0x32d8, 0x1b }, { 0x32d9, 0xbf }, { 0x3342, 0x2a }, { 0x3343, 0x10 },
	{ 0x336a, 0x18 }, { 0x3307, 0x00 }, { 0x3308, 0x3c }, { 0x3316, 0x00 },
	{ 0x3317, 0x5f }, { 0x3325, 0x2c }, { 0x3332, 0x0f }, { 0x335a, 0x1a },
	{ 0x336c, 0x00 }, { 0x336d, 0x2d }, { 0x336e, 0x00 }, { 0x336f, 0x88 },
	{ 0x36c0, 0x33 }, { 0x36c1, 0x33 }, { 0x32f3, 0x02 }, { 0x32f5, 0x03 },
	{ 0x32f6, 0x84 }, { 0x3669, 0x53 }, { 0x3681, 0x73 }, { 0x3692, 0x07 },
	{ 0x36e1, 0xff }, { 0x36e3, 0xbf }, { 0x36e4, 0x07 }, { 0x3327, 0x00 },
	{ 0x3328, 0x5a }, { 0x3334, 0x00 }, { 0x3335, 0x4c }, { 0x337a, 0x00 },
	{ 0x337b, 0x2d }, { 0x3618, 0x86 }, { 0x361b, 0x02 }, { 0x3690, 0x43 },
	{ 0x3252, 0x10 }, { 0x3316, 0x00 }, { 0x3318, 0x46 }, { 0x32d6, 0x01 },
	{ 0x32d7, 0xce }, { 0x337a, 0x00 }, { 0x337b, 0x50 }, { 0x3317, 0x1e },
	{ 0x331d, 0x50 }, { 0x32e1, 0x03 }, { 0x32c9, 0x00 }, { 0x3532, 0x03 },
	{ 0x3a02, 0xa1 }, { 0x3a0a, 0x03 }, { 0x0b04, 0x01 }, { 0x0138, 0x01 },
	{ 0x82e3, 0x03 }, { 0x833f, 0x01 }, { 0x0b06, 0x01 }, { 0x803f, 0x20 },
	{ 0x0350, 0x00 }, { 0x0136, 0x18 }, { 0x0137, 0x00 }, { 0x0114, 0x01 },
	{ 0x0303, 0x01 }, { 0x0305, 0x02 }, { 0x0307, 0x63 }, { 0x0309, 0x0a },
	{ 0x030a, 0x08 }, { 0x030b, 0x01 }, { 0x030f, 0x4b }, { 0x302d, 0x04 },
	{ 0x3500, 0x80 }, { 0x3502, 0xd0 }, { 0x0824, 0x14 }, { 0x3530, 0x40 },
	{ 0x3325, 0x2b }, { 0x0342, 0x04 }, { 0x0343, 0x4c }, { 0x0340, 0x04 },
	{ 0x0341, 0x65 }, { 0x0344, 0x00 }, { 0x0345, 0x02 }, { 0x0346, 0x00 },
	{ 0x0347, 0x02 }, { 0x0348, 0x07 }, { 0x0349, 0x85 }, { 0x034a, 0x04 },
	{ 0x034b, 0x3d }, { 0x0381, 0x01 }, { 0x0383, 0x01 }, { 0x0385, 0x01 },
	{ 0x0387, 0x01 }, { 0x0900, 0x00 }, { 0x0901, 0x11 }, { 0x0408, 0x00 },
	{ 0x0409, 0x00 }, { 0x040a, 0x00 }, { 0x040b, 0x00 }, { 0x040c, 0x07 },
	{ 0x040d, 0x84 }, { 0x040e, 0x04 }, { 0x040f, 0x3c }, { 0x034c, 0x07 },
	{ 0x034d, 0x84 }, { 0x034e, 0x04 }, { 0x034f, 0x3c }, { 0x0280, 0x01 },
	{ 0x8205, 0x03 }, { 0x82bb, 0x67 }, { 0x82b8, 0x06 }, { 0x82b9, 0x06 },
	{ 0x82bc, 0x38 }, { 0x82cb, 0x20 }, { 0x82bd, 0x03 }, { 0x82be, 0x03 },
	{ 0x82bf, 0x50 }, { 0x82c0, 0x4a }, { 0x82c1, 0x4a }, { 0x82c2, 0x4a },
	{ 0x82c3, 0x00 }, { 0x82c4, 0x3f }, { 0x82c5, 0x0f }, { 0x82c6, 0x1f },
	{ 0x82d3, 0x02 }, { 0x82d4, 0xa3 }, { 0x82d5, 0x95 }, { 0x82d6, 0x02 },
	{ 0x82d7, 0x33 }, { 0x82d8, 0x91 }, { 0x82e3, 0x03 }, { 0x8741, 0x44 },
	{ 0x8250, 0x23 }, { 0x8251, 0x09 }, { 0x8252, 0x00 }, { 0x8253, 0x3a },
	{ 0x8254, 0x20 }, { 0x8255, 0x23 }, { 0x8256, 0x09 }, { 0x8257, 0x3a },
	{ 0x8258, 0x20 }, { 0x8259, 0x23 }, { 0x825a, 0x09 }, { 0x825b, 0x3a },
	{ 0x825c, 0x20 }, { 0x825d, 0x40 }, { 0x825e, 0x04 }, { 0x8550, 0x4d },
	{ 0x8551, 0x09 }, { 0x8552, 0x00 }, { 0x8553, 0x6b }, { 0x8554, 0x2d },
	{ 0x8555, 0x53 }, { 0x8556, 0x13 }, { 0x8557, 0x4d }, { 0x8558, 0x09 },
	{ 0x8559, 0x64 }, { 0x855a, 0x33 }, { 0x855b, 0x42 }, { 0x855c, 0x04 },
	{ 0x826c, 0x60 }, { 0x826d, 0x04 }, { 0x826c, 0x20 }, { 0x9040, 0x09 },
	{ 0x9023, 0x60 }, { 0x8201, 0x0f },
};

/* 1920x1080 @ 60, 4 lanes; 196 writes, verbatim from libsns_nt99235.so 0x027b0. */
static const struct nt99235_reg nt99235_regs_1920x1080p60_4lane[] = {
	{ 0x3611, 0x30 }, { 0x3612, 0x00 }, { 0x3247, 0x00 }, { 0x36e7, 0x01 },
	{ 0x3537, 0x10 }, { 0x3539, 0x04 }, { 0x303b, 0x03 }, { 0x3672, 0x07 },
	{ 0x3673, 0x10 }, { 0x36b2, 0x74 }, { 0x36b3, 0x83 }, { 0x36b4, 0xf3 },
	{ 0x36b5, 0xf7 }, { 0x3682, 0x51 }, { 0x3683, 0x13 }, { 0x3684, 0x71 },
	{ 0x3685, 0x71 }, { 0x3686, 0x71 }, { 0x3687, 0x02 }, { 0x325c, 0x1f },
	{ 0x303a, 0x04 }, { 0x303b, 0x05 }, { 0x323e, 0x10 }, { 0x36a2, 0x00 },
	{ 0x325d, 0x16 }, { 0x325e, 0x0f }, { 0x32ca, 0x01 }, { 0x32cb, 0x0c },
	{ 0x32cc, 0x03 }, { 0x32cd, 0x28 }, { 0x32d6, 0x01 }, { 0x32d7, 0x0c },
	{ 0x32d8, 0x1b }, { 0x32d9, 0xbf }, { 0x3342, 0x2a }, { 0x3343, 0x10 },
	{ 0x336a, 0x18 }, { 0x3307, 0x00 }, { 0x3308, 0x3c }, { 0x3316, 0x00 },
	{ 0x3317, 0x5f }, { 0x3325, 0x2c }, { 0x3332, 0x0f }, { 0x335a, 0x1a },
	{ 0x336c, 0x00 }, { 0x336d, 0x2d }, { 0x336e, 0x00 }, { 0x336f, 0x88 },
	{ 0x36c0, 0x33 }, { 0x36c1, 0x33 }, { 0x32f3, 0x02 }, { 0x32f5, 0x03 },
	{ 0x32f6, 0x84 }, { 0x3669, 0x53 }, { 0x3681, 0x73 }, { 0x3692, 0x07 },
	{ 0x36e1, 0xff }, { 0x36e3, 0xbf }, { 0x36e4, 0x07 }, { 0x3327, 0x00 },
	{ 0x3328, 0x5a }, { 0x3334, 0x00 }, { 0x3335, 0x4c }, { 0x337a, 0x00 },
	{ 0x337b, 0x2d }, { 0x3618, 0x86 }, { 0x361b, 0x02 }, { 0x3690, 0x43 },
	{ 0x3252, 0x10 }, { 0x3316, 0x00 }, { 0x3318, 0x46 }, { 0x32d6, 0x01 },
	{ 0x32d7, 0xce }, { 0x337a, 0x00 }, { 0x337b, 0x50 }, { 0x3317, 0x1e },
	{ 0x331d, 0x50 }, { 0x32e1, 0x03 }, { 0x32c9, 0x00 }, { 0x3532, 0x03 },
	{ 0x3a02, 0xa1 }, { 0x3a0a, 0x03 }, { 0x0b04, 0x01 }, { 0x0138, 0x01 },
	{ 0x82e3, 0x03 }, { 0x833f, 0x01 }, { 0x0b06, 0x01 }, { 0x803f, 0x20 },
	{ 0x0350, 0x00 }, { 0x0100, 0x00 }, { 0x0136, 0x18 }, { 0x0137, 0x00 },
	{ 0x0114, 0x03 }, { 0x0303, 0x01 }, { 0x0305, 0x02 }, { 0x0307, 0x63 },
	{ 0x0309, 0x0a }, { 0x030a, 0x08 }, { 0x030b, 0x01 }, { 0x030f, 0x26 },
	{ 0x302d, 0x04 }, { 0x3500, 0x80 }, { 0x3502, 0xd0 }, { 0x0824, 0x14 },
	{ 0x3530, 0x40 }, { 0x3325, 0x2b }, { 0x0342, 0x04 }, { 0x0343, 0x4c },
	{ 0x0340, 0x04 }, { 0x0341, 0x65 }, { 0x0344, 0x00 }, { 0x0345, 0x02 },
	{ 0x0346, 0x00 }, { 0x0347, 0x02 }, { 0x0348, 0x07 }, { 0x0349, 0x85 },
	{ 0x034a, 0x04 }, { 0x034b, 0x3d }, { 0x0381, 0x01 }, { 0x0383, 0x01 },
	{ 0x0385, 0x01 }, { 0x0387, 0x01 }, { 0x0900, 0x00 }, { 0x0901, 0x11 },
	{ 0x0408, 0x00 }, { 0x0409, 0x00 }, { 0x040a, 0x00 }, { 0x040b, 0x00 },
	{ 0x040c, 0x07 }, { 0x040d, 0x84 }, { 0x040e, 0x04 }, { 0x040f, 0x3c },
	{ 0x034c, 0x07 }, { 0x034d, 0x84 }, { 0x034e, 0x04 }, { 0x034f, 0x3c },
	{ 0x0280, 0x01 }, { 0x8205, 0x03 }, { 0x82bb, 0x67 }, { 0x82b8, 0x06 },
	{ 0x82b9, 0x06 }, { 0x82bc, 0x38 }, { 0x82cb, 0x20 }, { 0x82bd, 0x03 },
	{ 0x82be, 0x03 }, { 0x82bf, 0x50 }, { 0x82c0, 0x4a }, { 0x82c1, 0x4a },
	{ 0x82c2, 0x4a }, { 0x82c3, 0x00 }, { 0x82c4, 0x3f }, { 0x82c5, 0x0f },
	{ 0x82c6, 0x1f }, { 0x82d3, 0x02 }, { 0x82d4, 0xa3 }, { 0x82d5, 0x95 },
	{ 0x82d6, 0x02 }, { 0x82d7, 0x33 }, { 0x82d8, 0x91 }, { 0x82e3, 0x03 },
	{ 0x8741, 0x44 }, { 0x8250, 0x29 }, { 0x8251, 0x15 }, { 0x8252, 0x00 },
	{ 0x8253, 0x38 }, { 0x8254, 0x23 }, { 0x8255, 0x30 }, { 0x8256, 0x1d },
	{ 0x8257, 0x26 }, { 0x8258, 0x13 }, { 0x8259, 0x3b }, { 0x825a, 0x26 },
	{ 0x825b, 0x2a }, { 0x825c, 0x15 }, { 0x825d, 0x80 }, { 0x825e, 0x04 },
	{ 0x8550, 0x2b }, { 0x8551, 0x17 }, { 0x8552, 0x00 }, { 0x8553, 0x3a },
	{ 0x8554, 0x25 }, { 0x8555, 0x32 }, { 0x8556, 0x1f }, { 0x8557, 0x28 },
	{ 0x8558, 0x15 }, { 0x8559, 0x3c }, { 0x855a, 0x27 }, { 0x855b, 0x2b },
	{ 0x855c, 0x17 }, { 0x826c, 0x80 }, { 0x826d, 0x04 }, { 0x826c, 0x20 },
	{ 0x9040, 0x09 }, { 0x9020, 0x00 }, { 0x9023, 0x80 }, { 0x8201, 0x0f },
};

/* 1280x720 @ 90, 4 lanes; 196 writes, verbatim from libsns_nt99235.so 0x040b0. */
static const struct nt99235_reg nt99235_regs_1280x720p90[] = {
	{ 0x3611, 0x30 }, { 0x3612, 0x00 }, { 0x3247, 0x00 }, { 0x36e7, 0x01 },
	{ 0x3537, 0x10 }, { 0x3539, 0x04 }, { 0x303b, 0x03 }, { 0x3672, 0x07 },
	{ 0x3673, 0x10 }, { 0x36b2, 0x74 }, { 0x36b3, 0x83 }, { 0x36b4, 0xf3 },
	{ 0x36b5, 0xf7 }, { 0x3682, 0x51 }, { 0x3683, 0x13 }, { 0x3684, 0x71 },
	{ 0x3685, 0x71 }, { 0x3686, 0x71 }, { 0x3687, 0x02 }, { 0x325c, 0x1f },
	{ 0x303a, 0x04 }, { 0x303b, 0x05 }, { 0x323e, 0x10 }, { 0x36a2, 0x00 },
	{ 0x325d, 0x16 }, { 0x325e, 0x0f }, { 0x32ca, 0x01 }, { 0x32cb, 0x0c },
	{ 0x32cc, 0x03 }, { 0x32cd, 0x28 }, { 0x32d6, 0x01 }, { 0x32d7, 0x0c },
	{ 0x32d8, 0x1b }, { 0x32d9, 0xbf }, { 0x3342, 0x2a }, { 0x3343, 0x10 },
	{ 0x336a, 0x18 }, { 0x3307, 0x00 }, { 0x3308, 0x3c }, { 0x3316, 0x00 },
	{ 0x3317, 0x5f }, { 0x3325, 0x2c }, { 0x3332, 0x0f }, { 0x335a, 0x1a },
	{ 0x336c, 0x00 }, { 0x336d, 0x2d }, { 0x336e, 0x00 }, { 0x336f, 0x88 },
	{ 0x36c0, 0x33 }, { 0x36c1, 0x33 }, { 0x32f3, 0x02 }, { 0x32f5, 0x03 },
	{ 0x32f6, 0x84 }, { 0x3669, 0x53 }, { 0x3681, 0x73 }, { 0x3692, 0x07 },
	{ 0x36e1, 0xff }, { 0x36e3, 0xbf }, { 0x36e4, 0x07 }, { 0x3327, 0x00 },
	{ 0x3328, 0x5a }, { 0x3334, 0x00 }, { 0x3335, 0x4c }, { 0x337a, 0x00 },
	{ 0x337b, 0x2d }, { 0x3618, 0x86 }, { 0x361b, 0x02 }, { 0x3690, 0x43 },
	{ 0x3252, 0x10 }, { 0x3316, 0x00 }, { 0x3318, 0x46 }, { 0x32d6, 0x01 },
	{ 0x32d7, 0xce }, { 0x337a, 0x00 }, { 0x337b, 0x50 }, { 0x3317, 0x1e },
	{ 0x331d, 0x50 }, { 0x32e1, 0x03 }, { 0x32c9, 0x00 }, { 0x3532, 0x03 },
	{ 0x3a02, 0xa1 }, { 0x3a0a, 0x03 }, { 0x0b04, 0x01 }, { 0x0138, 0x01 },
	{ 0x82e3, 0x03 }, { 0x833f, 0x01 }, { 0x0b06, 0x01 }, { 0x803f, 0x20 },
	{ 0x0350, 0x00 }, { 0x0100, 0x00 }, { 0x0136, 0x18 }, { 0x0137, 0x00 },
	{ 0x0114, 0x03 }, { 0x0303, 0x01 }, { 0x0305, 0x02 }, { 0x0307, 0x63 },
	{ 0x0309, 0x0a }, { 0x030a, 0x08 }, { 0x030b, 0x01 }, { 0x030f, 0x26 },
	{ 0x302d, 0x04 }, { 0x3500, 0x80 }, { 0x3502, 0xd0 }, { 0x0824, 0x14 },
	{ 0x3530, 0x40 }, { 0x3325, 0x2b }, { 0x0342, 0x04 }, { 0x0343, 0x4c },
	{ 0x0340, 0x02 }, { 0x0341, 0xf0 }, { 0x0344, 0x01 }, { 0x0345, 0x44 },
	{ 0x0346, 0x00 }, { 0x0347, 0xb8 }, { 0x0348, 0x06 }, { 0x0349, 0x43 },
	{ 0x034a, 0x03 }, { 0x034b, 0x87 }, { 0x0381, 0x01 },
	{ 0x0385, 0x01 }, { 0x0387, 0x01 }, { 0x0900, 0x00 }, { 0x0901, 0x11 },
	{ 0x0408, 0x00 }, { 0x0409, 0x00 }, { 0x040a, 0x00 }, { 0x040b, 0x00 },
	{ 0x040c, 0x05 }, { 0x040d, 0x04 }, { 0x040e, 0x02 }, { 0x040f, 0xd4 },
	{ 0x034c, 0x05 }, { 0x034d, 0x04 }, { 0x034e, 0x02 }, { 0x034f, 0xd4 },
	{ 0x0280, 0x01 }, { 0x8206, 0x03 }, { 0x8205, 0x03 }, { 0x82bb, 0x67 },
	{ 0x82b8, 0x06 }, { 0x82b9, 0x06 }, { 0x82bc, 0x38 }, { 0x82cb, 0x20 },
	{ 0x82bd, 0x03 }, { 0x82be, 0x03 }, { 0x82bf, 0x50 }, { 0x82c0, 0x41 },
	{ 0x82c1, 0x41 }, { 0x82c2, 0x41 }, { 0x82c3, 0x00 }, { 0x82c4, 0x3f },
	{ 0x82c5, 0x0f }, { 0x82c6, 0x1f }, { 0x82d3, 0x02 }, { 0x82d4, 0xa3 },
	{ 0x82d5, 0x95 }, { 0x82d6, 0x02 }, { 0x82d7, 0x33 }, { 0x82d8, 0x91 },
	{ 0x82e3, 0x03 }, { 0x8741, 0x40 }, { 0x8250, 0x29 }, { 0x8251, 0x15 },
	{ 0x8252, 0x00 }, { 0x8253, 0x38 }, { 0x8254, 0x23 }, { 0x8255, 0x30 },
	{ 0x8256, 0x1d }, { 0x8257, 0x26 }, { 0x8258, 0x13 }, { 0x8259, 0x3b },
	{ 0x825a, 0x26 }, { 0x825b, 0x2a }, { 0x825c, 0x15 }, { 0x825d, 0x80 },
	{ 0x825e, 0x04 }, { 0x8550, 0x2b }, { 0x8551, 0x17 }, { 0x8552, 0x00 },
	{ 0x8553, 0x3a }, { 0x8554, 0x25 }, { 0x8555, 0x32 }, { 0x8556, 0x1f },
	{ 0x8557, 0x28 }, { 0x8558, 0x15 }, { 0x8559, 0x3c }, { 0x855a, 0x27 },
	{ 0x855b, 0x2b }, { 0x855c, 0x17 }, { 0x826c, 0x80 }, { 0x826d, 0x04 },
	{ 0x826c, 0x20 }, { 0x9040, 0x09 }, { 0x9023, 0x60 }, { 0x8201, 0x0f },
};

/* The four modes the sensor library supports. cmos_set_image_mode rejects any
 * other index, so this table is complete.
 */
struct nt99235_mode {
	const char *name;
	u32 width;
	u32 height;
	u32 frame_interval_num;
	u32 frame_interval_den;
	u8 lanes;
	/*
	 * Frame length in lines, the value the mode table writes to
	 * 0x0340/0x0341. Exposure is counted in lines and the vendor clamps it
	 * to vts - 2, so the ceiling moves with the mode: 1125 at 1080p60 but
	 * 562 at 960x540p120. Carried here so the clamp follows the mode rather
	 * than being a constant that is only right for one of them.
	 */
	u16 vts;
	const struct nt99235_reg *regs;
	unsigned int num_regs;
};

#define NT99235_MODE(_name, _w, _h, _fps, _lanes, _vts, _regs) {	\
	.name = _name,						\
	.width = _w,						\
	.height = _h,						\
	.frame_interval_num = 1,				\
	.frame_interval_den = _fps,				\
	.lanes = _lanes,					\
	.vts = _vts,						\
	.regs = _regs,						\
	.num_regs = ARRAY_SIZE(_regs),				\
}

static const struct nt99235_mode nt99235_modes[] = {
	NT99235_MODE("1920x1080p60", 1920, 1080, 60, 2, 0x0465,
		     nt99235_regs_1920x1080p60),
	NT99235_MODE("960x540p120", 960, 540, 120, 2, 0x0232,
		     nt99235_regs_960x540p120),
	NT99235_MODE("1920x1080p60-4lane", 1920, 1080, 60, 4, 0x0465,
		     nt99235_regs_1920x1080p60_4lane),
	NT99235_MODE("1280x720p90", 1280, 720, 90, 4, 0x02f0,
		     nt99235_regs_1280x720p90),
};

/*
 * Bayer order is unconfirmed: it is not recoverable from the vendor library
 * (the ISP is told the order out of band) and is cheapest to settle by looking
 * at a captured frame. SRGGB12 is the placeholder, matching the order the
 * vendor raw-dump API names in STREAM_FORMAT_RAW_UNPACKED_12BIT_RGGB.
 */
#define NT99235_MBUS_CODE	MEDIA_BUS_FMT_SRGGB12_1X12

/*
 * The link rate comes from the sensor's PLL, not from the frame timing. The
 * frame timing gives the average pixel rate, but the MIPI link runs faster than
 * that and idles between lines, so sizing the receiver's D-PHY from it selects
 * too low a frequency range and the link returns unrecoverable header ECC
 * errors.
 *
 * From the mode register sets, which program the same PLL in every mode:
 *
 *   extclk           0x0136   24 MHz
 *   pre_pll_clk_div  0x0305   2
 *   pll_multiplier   0x0307   99
 *   op_sys_clk_div   0x030b   1
 *
 * so the PLL output is 24 / 2 * 99 = 1188 MHz, which is the total link bit
 * rate. Divided across the lanes that is 594 Mbit/s per lane on two lanes and
 * 297 on four.
 *
 * The link frequency is half the per-lane bit rate, because the D-PHY clocks
 * data on both edges.
 *
 * The pixel rate is the frame rate figure: every mode carries a line length of
 * 1100 with the frame length tracking the frame rate (1125 at 60 fps, 562 at
 * 120, 752 at 90), giving 74.25 Mpixel/s.
 */
#define NT99235_PIXEL_RATE		74250000ULL

/* Per-lane DDR clock, half the serial bit rate. The vendor sensor library
 * libsns_nt99235.so carries the per-lane serial rate as a literal in each mode
 * descriptor: 900 Mbps for the 2-lane 1920x1080 and 960x540 modes, 456 Mbps for
 * the 4-lane 1920x1080 and 1280x720 modes. Both give the same ~1.8 Gbps of
 * pixel bandwidth (900*2 = 456*4) and match the receiver's free-running
 * byte-clock counters (894 / 453 Mbps measured). At 900 Mbps the DesignWare
 * hsfreqrange lands in range 5, at 456 Mbps in range 3.
 */
#define NT99235_LINK_FREQ_2LANE		450000000LL
#define NT99235_LINK_FREQ_4LANE		228000000LL

static const s64 nt99235_link_freqs[] = {
	NT99235_LINK_FREQ_2LANE,
	NT99235_LINK_FREQ_4LANE,
};

/* Index into nt99235_link_freqs for a mode's lane count. */
#define NT99235_LINK_FREQ_INDEX(lanes)	((lanes) > 2 ? 1 : 0)

struct nt99235 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct device *dev;

	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;

	/* Data lanes declared by the device tree endpoint; picks which modes
	 * are usable.
	 */
	u8 num_data_lanes;

	/* Reports the link frequency so the receiver can program its D-PHY. */
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *again;

	const struct nt99235_mode *mode;
};

static inline struct nt99235 *to_nt99235(struct v4l2_subdev *sd)
{
	return container_of(sd, struct nt99235, sd);
}

/* nt99235_write - write one 8-bit value to a 16-bit register address.
 *
 * The sensor uses 16-bit register addressing, which the SMBus helpers cannot
 * express, so every access is a raw 3-byte i2c message.
 */
static int nt99235_write(struct nt99235 *nt99235, u16 address, u8 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(&nt99235->sd);
	u8 buf[3] = { address >> 8, address & 0xff, value };

	int ret = i2c_master_send(client, buf, sizeof(buf));

	if (ret != sizeof(buf)) {
		dev_err(nt99235->dev, "write 0x%04x = 0x%02x failed (%d)\n",
			address, value, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

/* nt99235_read - read one 8-bit value from a 16-bit register address. */
static int nt99235_read(struct nt99235 *nt99235, u16 address, u8 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(&nt99235->sd);
	u8 addr_buf[2] = { address >> 8, address & 0xff };
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(addr_buf),
			.buf = addr_buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = value,
		},
	};

	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(nt99235->dev, "read 0x%04x failed (%d)\n", address, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int nt99235_write_regs(struct nt99235 *nt99235,
			      const struct nt99235_reg *regs,
			      unsigned int num_regs)
{
	for (unsigned int i = 0; i < num_regs; i++) {
		int ret = nt99235_write(nt99235, regs[i].address, regs[i].value);

		if (ret)
			return ret;
	}

	return 0;
}

/* nt99235_power_on - the vendor power-on sequence, verbatim.
 *
 * Both control lines are driven to their asserted (low) level, the sensor is
 * given 10 ms to settle, the 24 MHz master clock is started, and only then is
 * reset released. Releasing reset before the master clock is running leaves
 * the sensor unresponsive on i2c.
 *
 * The enable line stays asserted for the whole session: nt99235_cmos_power_off
 * releases reset and stops the clock but never touches it.
 */
static int nt99235_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct nt99235 *nt99235 = to_nt99235(sd);
	int ret;

	/* Each step announces itself before it runs, so a machine that stops
	 * responding names the operation that stopped it.
	 */
	dev_info(dev, "power on: asserting enable (gpio) and reset (gpio)\n");
	gpiod_direction_output(nt99235->enable_gpio, 1);
	gpiod_direction_output(nt99235->reset_gpio, 1);

	usleep_range(NT99235_RESET_SETTLE_US, NT99235_RESET_SETTLE_US + 1000);

	dev_info(dev, "power on: enabling the %u Hz master clock\n",
		 NT99235_MCLK_RATE);
	ret = clk_prepare_enable(nt99235->mclk);
	if (ret) {
		dev_err(dev, "failed to enable the master clock (%d)\n", ret);
		gpiod_set_value_cansleep(nt99235->enable_gpio, 0);
		return ret;
	}

	dev_info(dev, "power on: releasing reset\n");
	gpiod_set_value_cansleep(nt99235->reset_gpio, 0);

	usleep_range(NT99235_RESET_SETTLE_US, NT99235_RESET_SETTLE_US + 1000);

	dev_info(dev, "power on: complete\n");

	return 0;
}

static int nt99235_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct nt99235 *nt99235 = to_nt99235(sd);

	gpiod_set_value_cansleep(nt99235->reset_gpio, 1);
	clk_disable_unprepare(nt99235->mclk);

	return 0;
}

/* nt99235_detect - confirm the part is present and is an NT99235. */
static int nt99235_detect(struct nt99235 *nt99235)
{
	u8 hi, lo;
	u16 id;

	int ret = nt99235_read(nt99235, NT99235_REG_MODEL_ID_HI, &hi);

	if (ret)
		return ret;

	ret = nt99235_read(nt99235, NT99235_REG_MODEL_ID_LO, &lo);
	if (ret)
		return ret;

	id = (hi << 8) | lo;
	if (id != NT99235_MODEL_ID) {
		dev_err(nt99235->dev, "unexpected model id 0x%04x\n", id);
		return -ENODEV;
	}

	return 0;
}

/*
 * Dump the SMIA read-only identification and capability blocks.
 *
 * Every register this driver uses outside the opaque 0x3xxx and 0x8xxx Novatek
 * ranges is at an SMIA address, and two of them have been confirmed to behave
 * as SMIA specifies rather than merely to exist: 0x0600 = 2 produced 100 per
 * cent colour bars on hardware, and the vendor's own AE commit list uses
 * 0x0202/0x0203 for line-based integration under a 0x0104 group hold. That is
 * strong, but it is still inference from behaviour.
 *
 * SMIA settles it directly. 0x0005 is SMIA_VERSION, and 0x0084-0x0097 describe
 * analogue gain: the code minimum, maximum and step, plus the m0, c0, m1 and c1
 * coefficients of gain = (m0 * code + c0) / (m1 * code + c1). If those read
 * sensibly the sensor is SMIA-compliant, and the gain mapping comes from the
 * sensor itself rather than from a table read out of the vendor library, which
 * is the one part of the exposure path we currently take on trust.
 *
 * Reads only. Registers that are not implemented typically read zero, so a
 * block of zeros is the answer "not SMIA here" rather than a failure.
 */
struct nt99235_cap_reg {
	u16 addr;
	u8 len;
	const char *name;
};

static const struct nt99235_cap_reg nt99235_smia_caps[] = {
	{ 0x0000, 2, "model_id" },
	{ 0x0002, 1, "revision_number" },
	{ 0x0004, 1, "manufacturer_id" },
	{ 0x0005, 1, "smia_version" },
	{ 0x0007, 1, "pixel_order" },
	{ 0x0040, 1, "frame_format_model_type" },
	{ 0x0041, 1, "frame_format_model_subtype" },
	{ 0x0080, 2, "analogue_gain_capability" },
	{ 0x0084, 2, "analogue_gain_code_min" },
	{ 0x0086, 2, "analogue_gain_code_max" },
	{ 0x0088, 2, "analogue_gain_code_step" },
	{ 0x008a, 2, "analogue_gain_type" },
	{ 0x008c, 2, "analogue_gain_m0" },
	{ 0x008e, 2, "analogue_gain_c0" },
	{ 0x0090, 2, "analogue_gain_m1" },
	{ 0x0092, 2, "analogue_gain_c1" },
	{ 0x1000, 1, "integration_time_capability" },
	{ 0x1004, 2, "coarse_integration_time_min" },
	{ 0x1006, 2, "coarse_integration_time_max_margin" },
	{ 0x1100, 1, "digital_gain_capability" },
};

static void nt99235_dump_smia_caps(struct nt99235 *nt99235)
{
	u32 nonzero = 0;

	for (unsigned int i = 0; i < ARRAY_SIZE(nt99235_smia_caps); i++) {
		const struct nt99235_cap_reg *c = &nt99235_smia_caps[i];
		u32 val = 0;
		u8 b;

		for (unsigned int j = 0; j < c->len; j++) {
			if (nt99235_read(nt99235, c->addr + j, &b)) {
				dev_warn(nt99235->dev, "smia: %s read failed\n",
					 c->name);
				return;
			}

			val = (val << 8) | b;
		}

		if (val)
			nonzero++;

		dev_info(nt99235->dev, "smia: %-34s 0x%04x = 0x%0*x (%u)\n",
			 c->name, c->addr, c->len * 2, val, val);
	}

	dev_info(nt99235->dev,
		 "smia: %u of %zu capability registers non-zero\n",
		 nonzero, ARRAY_SIZE(nt99235_smia_caps));
}

/*
 * Commit exposure and gain inside the vendor's group hold, so a frame never
 * sees one of the pair updated without the other.
 */
static int nt99235_commit_exposure(struct nt99235 *nt99235, u32 lines, u32 gain)
{
	u16 max = nt99235->mode->vts - 2;
	int ret;

	if (lines > max)
		lines = max;

	ret = nt99235_write(nt99235, NT99235_REG_GROUP_HOLD, 1);
	if (ret)
		return ret;

	ret = nt99235_write(nt99235, NT99235_REG_EXPOSURE_HI, (lines >> 8) & 0xff);
	if (!ret)
		ret = nt99235_write(nt99235, NT99235_REG_EXPOSURE_LO, lines & 0xff);

	/* The vendor writes the same code to both gain bytes. */
	if (!ret)
		ret = nt99235_write(nt99235, NT99235_REG_AGAIN_0, gain & 0xff);

	if (!ret)
		ret = nt99235_write(nt99235, NT99235_REG_AGAIN_1, gain & 0xff);

	/* Release the hold even if a write failed, or the sensor stays held. */
	if (nt99235_write(nt99235, NT99235_REG_GROUP_HOLD, 0) && !ret)
		ret = -EIO;

	return ret;
}

/*
 * Apply the current module-parameter values to a bound, powered sensor, and
 * keep the V4L2 controls showing the same thing.
 *
 * Does nothing when no sensor is bound or it is not powered, which is the
 * normal case for a parameter set at insmod: stream-on commits then.
 */
static int nt99235_apply_live(void)
{
	struct nt99235 *nt99235;
	int ret = 0;

	guard(mutex)(&nt99235_bound_lock);

	nt99235 = nt99235_bound;
	if (!nt99235)
		return 0;

	if (nt99235->exposure)
		__v4l2_ctrl_s_ctrl(nt99235->exposure, exposure);

	if (nt99235->again)
		__v4l2_ctrl_s_ctrl(nt99235->again, gain);

	if (!pm_runtime_get_if_in_use(nt99235->dev))
		return 0;

	ret = nt99235_commit_exposure(nt99235, exposure, gain);
	pm_runtime_put(nt99235->dev);

	return ret;
}

static int nt99235_param_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/*
	 * Clamp here rather than rejecting: a sweep script that walks past the
	 * ceiling should saturate, not fail halfway and leave the sensor at a
	 * value the operator no longer knows.
	 */
	if (exposure < NT99235_EXPOSURE_MIN)
		exposure = NT99235_EXPOSURE_MIN;

	if (gain < NT99235_AGAIN_MIN)
		gain = NT99235_AGAIN_MIN;

	if (gain > NT99235_AGAIN_MAX)
		gain = NT99235_AGAIN_MAX;

	return nt99235_apply_live();
}

static const struct kernel_param_ops nt99235_param_ops = {
	.set = nt99235_param_set,
	.get = param_get_int,
};

module_param_cb(exposure, &nt99235_param_ops, &exposure, 0644);
MODULE_PARM_DESC(exposure,
		 "integration time in lines, clamped to the mode's frame length minus 2; writable while streaming");

module_param_cb(gain, &nt99235_param_ops, &gain, 0644);
MODULE_PARM_DESC(gain,
		 "analogue gain code 0x00-0x60, gain = 2^(code>>4) * (16 + (code&15)) / 16, so 1x to 64x; writable while streaming");

static int nt99235_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct nt99235 *nt99235 =
		container_of(ctrl->handler, struct nt99235, ctrl_handler);

	/*
	 * Only touch the sensor when it is powered. Controls set while it is
	 * off are applied by stream-on, which commits both together.
	 */
	if (!pm_runtime_get_if_in_use(nt99235->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_ANALOGUE_GAIN:
		nt99235_commit_exposure(nt99235, nt99235->exposure->val,
					nt99235->again->val);
		break;
	}

	pm_runtime_put(nt99235->dev);

	return 0;
}

static const struct v4l2_ctrl_ops nt99235_ctrl_ops = {
	.s_ctrl = nt99235_s_ctrl,
};

/*
 * Program the test pattern and report what the sensor makes of it. Deliberately
 * not fatal: the address is an inference from the SMIA register map, so a device
 * that does not implement it should leave capture exactly as it was rather than
 * failing stream-on. The readback is the actual result of this function.
 */
static void nt99235_apply_test_pattern(struct nt99235 *nt99235)
{
	u8 hi = 0, lo = 0;

	int ret = nt99235_write(nt99235, NT99235_REG_TEST_PATTERN_HI,
				(test_pattern >> 8) & 0xff);
	if (!ret)
		ret = nt99235_write(nt99235, NT99235_REG_TEST_PATTERN_LO,
				    test_pattern & 0xff);

	if (ret) {
		dev_warn(nt99235->dev, "test pattern %d: write failed (%d)\n",
			 test_pattern, ret);
		return;
	}

	if (nt99235_read(nt99235, NT99235_REG_TEST_PATTERN_HI, &hi) ||
	    nt99235_read(nt99235, NT99235_REG_TEST_PATTERN_LO, &lo)) {
		dev_warn(nt99235->dev, "test pattern %d: readback failed\n",
			 test_pattern);
		return;
	}

	if (((hi << 8) | lo) == test_pattern)
		dev_info(nt99235->dev,
			 "test pattern %d set, reads back 0x%02x%02x: the register is implemented\n",
			 test_pattern, hi, lo);
	else
		dev_warn(nt99235->dev,
			 "test pattern %d wrote 0x%04x but reads back 0x%02x%02x: 0x0600 is not it\n",
			 test_pattern, test_pattern, hi, lo);
}

/* nt99235_start_stream - program the mode and start the sensor.
 *
 * Split from nt99235_set_stream so the runtime-PM reference is taken and
 * released in one place: every failure here is unwound by the caller's single
 * put, rather than by a goto from each step.
 */
static int nt99235_start_stream(struct nt99235 *nt99235)
{
	int ret;

	ret = nt99235_write_regs(nt99235, nt99235->mode->regs,
				 nt99235->mode->num_regs);
	if (ret)
		return ret;

	/* The vendor programs the table and issues stream-on from two separate
	 * framework phases, so the sensor PLL and MCU settle for tens of
	 * milliseconds before streaming; back-to-back writes are a case the
	 * vendor never exercises.
	 */
	msleep(20);

	/*
	 * After the mode table, so it is not overwritten by it, and before
	 * stream-on, so the first frame already carries the pattern.
	 */
	if (dump_smia)
		nt99235_dump_smia_caps(nt99235);

	/*
	 * Unconditional, including the off value. Skipping the write when off
	 * left whatever was already in the sensor's register, so once a pattern
	 * had been selected every later stream came up still showing it and
	 * there was no way back to a live scene short of a power cycle.
	 */
	nt99235_apply_test_pattern(nt99235);

	/*
	 * The mode table does not set exposure or gain, so commit the control
	 * values before streaming rather than waiting for someone to write one.
	 */
	ret = nt99235_commit_exposure(nt99235, exposure, gain);
	if (ret)
		return ret;

	return nt99235_write(nt99235, NT99235_REG_MODE_SELECT,
			     NT99235_MODE_STREAMING);
}

static int nt99235_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct nt99235 *nt99235 = to_nt99235(sd);
	int ret;

	if (!enable) {
		ret = nt99235_write(nt99235, NT99235_REG_MODE_SELECT,
				    NT99235_MODE_STANDBY);
		pm_runtime_put(nt99235->dev);
		return ret;
	}

	ret = pm_runtime_resume_and_get(nt99235->dev);
	if (ret)
		return ret;

	ret = nt99235_start_stream(nt99235);
	if (ret)
		pm_runtime_put(nt99235->dev);

	return ret;
}

/* nt99235_fill_format - describe the currently selected mode. */
static void nt99235_fill_format(const struct nt99235_mode *mode,
				struct v4l2_mbus_framefmt *format)
{
	format->width = mode->width;
	format->height = mode->height;
	format->code = NT99235_MBUS_CODE;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;
}

/* nt99235_has_usable_lanes - can this mode run on the lanes the board wires?
 *
 * A mode that wants more lanes than the endpoint declares would leave the
 * receiver waiting on lanes that may not exist, so it is hidden entirely
 * rather than offered and left to fail at stream time. A mode that wants fewer
 * is fine: the extra lanes simply stay idle.
 */
static bool nt99235_has_usable_lanes(const struct nt99235 *nt99235,
				     const struct nt99235_mode *mode)
{
	return mode->lanes <= nt99235->num_data_lanes;
}

static int nt99235_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = NT99235_MBUS_CODE;

	return 0;
}

static int nt99235_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct nt99235 *nt99235 = to_nt99235(sd);
	unsigned int seen = 0;

	if (fse->code != NT99235_MBUS_CODE)
		return -EINVAL;

	for (unsigned int i = 0; i < ARRAY_SIZE(nt99235_modes); i++) {
		const struct nt99235_mode *mode = &nt99235_modes[i];

		if (!nt99235_has_usable_lanes(nt99235, mode))
			continue;

		if (seen++ != fse->index)
			continue;

		fse->min_width = mode->width;
		fse->max_width = mode->width;
		fse->min_height = mode->height;
		fse->max_height = mode->height;

		return 0;
	}

	return -EINVAL;
}

/* nt99235_find_mode - the usable mode closest to a requested frame size. */
static const struct nt99235_mode *nt99235_find_mode(struct nt99235 *nt99235,
						    u32 width, u32 height)
{
	const struct nt99235_mode *best = NULL;
	unsigned int best_error = UINT_MAX;

	for (unsigned int i = 0; i < ARRAY_SIZE(nt99235_modes); i++) {
		const struct nt99235_mode *mode = &nt99235_modes[i];
		unsigned int error;

		if (!nt99235_has_usable_lanes(nt99235, mode))
			continue;

		error = abs((int)mode->width - (int)width) +
			abs((int)mode->height - (int)height);

		if (error < best_error) {
			best_error = error;
			best = mode;
		}
	}

	return best;
}

static int nt99235_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_format *fmt)
{
	struct nt99235 *nt99235 = to_nt99235(sd);
	const struct nt99235_mode *mode;

	mode = nt99235_find_mode(nt99235, fmt->format.width,
				 fmt->format.height);
	if (!mode)
		return -EINVAL;

	nt99235_fill_format(mode, &fmt->format);
	*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		nt99235->mode = mode;

		/* The link frequency follows the lane count, so the reported
		 * value has to track the selected mode.
		 */
		if (nt99235->link_freq)
			__v4l2_ctrl_s_ctrl(nt99235->link_freq,
					   NT99235_LINK_FREQ_INDEX(mode->lanes));
	}

	return 0;
}

static int nt99235_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state)
{
	struct nt99235 *nt99235 = to_nt99235(sd);

	nt99235_fill_format(nt99235->mode,
			    v4l2_subdev_state_get_format(sd_state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops nt99235_video_ops = {
	.s_stream = nt99235_set_stream,
};

static const struct v4l2_subdev_pad_ops nt99235_pad_ops = {
	.enum_mbus_code = nt99235_enum_mbus_code,
	.enum_frame_size = nt99235_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = nt99235_set_pad_format,
};

static const struct v4l2_subdev_ops nt99235_subdev_ops = {
	.video = &nt99235_video_ops,
	.pad = &nt99235_pad_ops,
};

static const struct v4l2_subdev_internal_ops nt99235_internal_ops = {
	.init_state = nt99235_init_state,
};

/* nt99235_parse_endpoint - read the CSI-2 lane count out of the device tree.
 *
 * The lane count decides which modes are offered, so a missing or unsupported
 * endpoint is fatal rather than defaulted.
 */
static int nt99235_parse_endpoint(struct nt99235 *nt99235)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *fwnode = dev_fwnode(nt99235->dev);
	struct fwnode_handle *endpoint;
	unsigned int lanes;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!endpoint) {
		dev_err(nt99235->dev, "no endpoint in the device tree node\n");
		return -ENXIO;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &bus_cfg);
	fwnode_handle_put(endpoint);
	if (ret)
		return ret;

	lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
	v4l2_fwnode_endpoint_free(&bus_cfg);

	if (lanes != 2 && lanes != 4) {
		dev_err(nt99235->dev, "%u data lanes is not supported\n", lanes);
		return -EINVAL;
	}

	nt99235->num_data_lanes = lanes;

	return 0;
}

/* nt99235_probe_bringup - run the probe-time hardware stages the user asked for.
 *
 * Stage 0 leaves the hardware untouched, which is the default: registering the
 * subdev is enough for the capture graph to form, and the full power-on
 * sequence runs when streaming starts. The higher stages exist so that a step
 * which wedges the machine can be identified from the console without
 * bisecting across reboots.
 */
static int nt99235_probe_bringup(struct nt99235 *nt99235)
{
	struct device *dev = nt99235->dev;
	int ret;

	if (bringup <= NT99235_BRINGUP_NONE) {
		dev_info(dev, "probe bring-up stage 0: hardware untouched\n");
		return 0;
	}

	if (bringup == NT99235_BRINGUP_CLOCK) {
		dev_info(dev, "probe bring-up stage 1: enabling the master clock\n");
		ret = clk_prepare_enable(nt99235->mclk);
		if (ret)
			return ret;

		dev_info(dev, "probe bring-up stage 1: master clock at %lu Hz\n",
			 clk_get_rate(nt99235->mclk));

		return 0;
	}

	dev_info(dev, "probe bring-up stage %d: full power-on sequence\n",
		 bringup);

	ret = nt99235_power_on(dev);
	if (ret)
		return ret;

	if (bringup < NT99235_BRINGUP_DETECT)
		return 0;

	dev_info(dev, "probe bring-up stage 3: reading the chip id over i2c\n");

	ret = nt99235_detect(nt99235);
	if (ret) {
		nt99235_power_off(dev);
		return ret;
	}

	return 0;
}

/*
 * nt99235_init_controls - publish the link frequency and pixel rate.
 *
 * The receiver needs the link frequency to pick its D-PHY frequency range, and
 * reads it through the standard control rather than being told the sensor's
 * mode table. Both controls are read-only: they follow the selected mode.
 */
static int nt99235_init_controls(struct nt99235 *nt99235)
{
	struct v4l2_ctrl_handler *handler = &nt99235->ctrl_handler;
	struct v4l2_ctrl *pixel_rate;

	int ret = v4l2_ctrl_handler_init(handler, 4);

	if (ret)
		return ret;

	nt99235->link_freq =
		v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(nt99235_link_freqs) - 1,
				       NT99235_LINK_FREQ_INDEX(nt99235->mode->lanes),
				       nt99235_link_freqs);

	if (nt99235->link_freq)
		nt99235->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
				       NT99235_PIXEL_RATE, NT99235_PIXEL_RATE,
				       1, NT99235_PIXEL_RATE);
	if (pixel_rate)
		pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Writable, unlike the two above. Nothing else sets exposure or gain on
	 * this stack: the vendor drives them from its 3A layer, which does not
	 * exist here, so without these the sensor stays at power-on defaults and
	 * every capture is black.
	 */
	nt99235->exposure =
		v4l2_ctrl_new_std(handler, &nt99235_ctrl_ops, V4L2_CID_EXPOSURE,
				  NT99235_EXPOSURE_MIN, nt99235->mode->vts - 2,
				  1, min_t(u32, exposure,
					   nt99235->mode->vts - 2));

	nt99235->again =
		v4l2_ctrl_new_std(handler, &nt99235_ctrl_ops,
				  V4L2_CID_ANALOGUE_GAIN, NT99235_AGAIN_MIN,
				  NT99235_AGAIN_MAX, 1, gain);

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	nt99235->sd.ctrl_handler = handler;

	return 0;
}

static int nt99235_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct nt99235 *nt99235;
	unsigned long mclk_rate;
	int ret;

	nt99235 = devm_kzalloc(dev, sizeof(*nt99235), GFP_KERNEL);
	if (!nt99235)
		return -ENOMEM;

	nt99235->dev = dev;
	v4l2_i2c_subdev_init(&nt99235->sd, client, &nt99235_subdev_ops);
	nt99235->sd.internal_ops = &nt99235_internal_ops;

	ret = nt99235_parse_endpoint(nt99235);
	if (ret)
		return ret;

	if (force_mode >= 0) {
		if (force_mode >= (int)ARRAY_SIZE(nt99235_modes)) {
			dev_err(dev, "mode %d does not exist\n", force_mode);
			return -EINVAL;
		}

		nt99235->mode = &nt99235_modes[force_mode];
		dev_info(dev, "mode forced to %d (%s, %u lanes)\n", force_mode,
			 nt99235->mode->name, nt99235->mode->lanes);
	} else {
		/* The first mode usable on this board's lane count. */
		nt99235->mode = nt99235_find_mode(nt99235, nt99235_modes[0].width,
						  nt99235_modes[0].height);
		if (!nt99235->mode) {
			dev_err(dev, "no sensor mode runs on %u data lanes\n",
				nt99235->num_data_lanes);
			return -EINVAL;
		}
	}

	nt99235->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(nt99235->mclk))
		return dev_err_probe(dev, PTR_ERR(nt99235->mclk),
				     "failed to get the master clock\n");

	mclk_rate = clk_get_rate(nt99235->mclk);
	if (mclk_rate != NT99235_MCLK_RATE)
		dev_warn(dev, "master clock is %lu Hz, the sensor expects %u Hz\n",
			 mclk_rate, NT99235_MCLK_RATE);

	/* Both lines are active low. Claimed as-is rather than driven: getting
	 * a descriptor must not change the pin state, because probe is not
	 * where this driver is allowed to touch the hardware.
	 */
	nt99235->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_ASIS);
	if (IS_ERR(nt99235->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(nt99235->enable_gpio),
				     "failed to get the enable gpio\n");

	nt99235->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(nt99235->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(nt99235->reset_gpio),
				     "failed to get the reset gpio\n");

	ret = nt99235_probe_bringup(nt99235);
	if (ret)
		return ret;

	ret = nt99235_init_controls(nt99235);
	if (ret) {
		dev_err(dev, "failed to register the controls (%d)\n", ret);
		goto error_power_off;
	}

	nt99235->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	nt99235->pad.flags = MEDIA_PAD_FL_SOURCE;
	nt99235->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&nt99235->sd.entity, 1, &nt99235->pad);
	if (ret)
		goto error_controls;

	ret = v4l2_subdev_init_finalize(&nt99235->sd);
	if (ret)
		goto error_media_entity;

	/* The sensor is left powered down unless a bring-up stage was asked
	 * for, so runtime PM starts suspended and the first resume performs
	 * the power-on sequence.
	 */
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&nt99235->sd);
	if (ret)
		goto error_pm;

	/* Publish for the live exposure and gain parameters. One sensor. */
	scoped_guard(mutex, &nt99235_bound_lock)
		nt99235_bound = nt99235;

	dev_info(dev, "NT99235 registered on %u data lanes, default mode %s, exposure %d gain 0x%02x\n",
		 nt99235->num_data_lanes, nt99235->mode->name, exposure, gain);

	return 0;

error_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&nt99235->sd);

error_media_entity:
	media_entity_cleanup(&nt99235->sd.entity);

error_controls:
	v4l2_ctrl_handler_free(&nt99235->ctrl_handler);

error_power_off:
	if (bringup > NT99235_BRINGUP_NONE)
		nt99235_power_off(dev);

	return ret;
}

static void nt99235_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nt99235 *nt99235 = to_nt99235(sd);
	struct device *dev = &client->dev;

	/* Before anything is torn down, so a concurrent parameter write cannot
	 * reach a half-freed sensor.
	 */
	scoped_guard(mutex, &nt99235_bound_lock)
		nt99235_bound = NULL;

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&nt99235->ctrl_handler);

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		nt99235_power_off(dev);

	pm_runtime_set_suspended(dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(nt99235_pm_ops, nt99235_power_off,
				 nt99235_power_on, NULL);

static const struct of_device_id nt99235_of_match[] = {
	{ .compatible = "novatek,nt99235" },
	{ }
};
MODULE_DEVICE_TABLE(of, nt99235_of_match);

static struct i2c_driver nt99235_driver = {
	.probe = nt99235_probe,
	.remove = nt99235_remove,
	.driver = {
		.name = "nt99235",
		.pm = pm_ptr(&nt99235_pm_ops),
		.of_match_table = nt99235_of_match,
	},
};
module_i2c_driver(nt99235_driver);

MODULE_DESCRIPTION("Novatek NT99235 image sensor subdev");
MODULE_LICENSE("GPL");
