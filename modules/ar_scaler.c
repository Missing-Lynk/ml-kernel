// SPDX-License-Identifier: GPL-2.0
/*
 * ar_scaler.ko - open reimplementation of the Artosyn MPP scaler / crop-resize
 * engine (/dev/arscaler). It reproduces the vendor driver's userspace ABI
 * byte-for-byte so the shipped vendor userspace keeps working unchanged.
 *
 * ABI (byte-exact immediates from the vendor .ko disassembly): misc device "arscaler"
 * (MISC_DYNAMIC_MINOR), magic 'Z' (0x5a), three _IOWR ioctls:
 *   0xc0405a01 _IOWR('Z',1,64)     single crop/resize  -> ar_scaler_start_single
 *   0xffc45a00 _IOWR('Z',0,16324)  batch {u32 count; op[255]} -> ar_scaler_start_batch
 *   0xc0045a02 _IOWR('Z',2,4)      set frequency (int)
 * No mmap, no compat_ioctl. Userspace passes physical addresses by value inside
 * a 64-byte op descriptor; no shared-buffer ABI.
 *
 * On top of the vendor family, this open port adds a dmabuf-fd family
 * ('Z' nr 3/4, ar_scaler.h): the same op geometry addressed by dmabuf fd +
 * offset, resolved in-kernel via the shared ml_dmabuf_map.h resolver (per-open
 * mapping cache, single-segment contiguity gate). The vendor commands and
 * their register choreography are untouched.
 *
 * HARDWARE STATUS: the ABI front-end AND the in-kernel register choreography are
 * now recovered 1:1 from the vendor ar_scaler.ko (unstripped AArch64, symbols
 * present) by static disassembly - every register offset, the Q16 ratio math,
 * the clock bring-up values/timing, and the 1024-byte interpolation LUT are taken
 * verbatim from the binary, with the source .text/.rodata address cited at each
 * site. The descriptor field map lives at regbase+0x1C (single) / batch_kvaddr at
 * stride 0x100 (batch); clock magic 0x00ACCE55 at ctrlbase+0x6200; LUT extracted
 * from .rodata `scaler_lut_table` (offset 0, 1024 B). HARDWARE-VALIDATED: single
 * and batch ops bit-exact, downscale within box-filter tolerance, both ABI
 * families (see test_tools/scaler_dmabuf_test.c and modules/HW-BRINGUP.md).
 *
 * The internal LUT and batch-descriptor buffers come from dma_alloc_coherent
 * (the vendor driver used its MMZ allocator); the module has no ar_osal
 * dependency.
 */
#define pr_fmt(fmt) "ar_scaler: " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/bits.h>
#include <linux/pm.h>
#include <linux/dma-mapping.h>

#include "ar_scaler.h"
#include "ml_dmabuf_map.h"

/* --- scaler core registers (regbase, 0x1000 window) ----------------------- *
 * RECOVERED EXACTLY from ar_scaler.ko (.text addresses cited per field).
 *
 * The control/trigger/status registers live in the first 0x1C bytes; the
 * per-frame "descriptor image" (16 packed fields, 256-byte stride) starts at
 * regbase+0x1C and is filled by scaler_set_batch_registers (vendor passes it
 * regbase+0x1C for single ops, or batch_kvaddr+i*0x100 for batch DMA - see
 * ar_scaler_start_single @0xa40 `add x0,x0,#0x1c` and ar_scaler_start_batch
 * @0x710 `w3=i<<8`).
 */
#define SC_REG_CTRL		0x00	/* bit0: 0=single 1=batch; trigger clears/sets it */
#define SC_REG_BATCHADDR	0x04	/* batch DMA descriptor-block phys (batch mode) */
#define SC_REG_BATCHCFG		0x0c	/* [31:16] = batch op count (batch mode) */
#define SC_REG_IRQEN		0x10	/* hw_init: =0 then |=bit31 (completion IRQ enable) */
#define SC_REG_STATUS		0x14	/* W1C; bit31 = completion (irq_handler @0x560) */
#define SC_REG_TRIGGER		0x18	/* write 1 to launch (single & batch) */

/* hw_init also pre-seeds two descriptor-image words at their live offsets: */
#define SC_REG_CONF		0x28	/* = SC_DESC_BASE + SD_CONF;  hw_init default 0x1040d */
#define SC_REG_BURST		0x40	/* = SC_DESC_BASE + SD_BURST; hw_init default 0x7804 */
#define SC_REG_LUTADDR		0x70	/* = SC_DESC_BASE + SD_LUTPHY; hw_init writes lut_phys */

#define SC_STATUS_DONE		BIT(31)

/* Per-frame descriptor image base + stride (re ar_scaler_start_{single,batch}). */
#define SC_DESC_BASE		0x1c	/* live single-op descriptor at regbase+0x1C */
#define SC_DESC_STRIDE		0x100	/* batch DMA per-op stride (256 B) */

/* Descriptor field byte-offsets (relative to the descriptor base), recovered
 * 1:1 from scaler_set_batch_registers @.text 0x2b8.
 */
#define SD_SRCPHY		0x00	/* @0x2c8  srcphy */
#define SD_SRCSTR		0x04	/* @0x2cc/0x2e0  [15:0]=srcstride [31:16]=ctrl3c */
#define SD_SRCDIM		0x08	/* @0x2f4/0x308  [12:0]=srcw [28:16]=srch */
#define SD_CONF			0x0c	/* @0x31c..0x368 0xc|(control<<5)|(channels<<16) */
#define SD_CROPXY		0x10	/* @0x36c/0x380  [12:0]=crop_x [28:16]=crop_y */
#define SD_CROPDIM		0x14	/* @0x394/0x3a8  [12:0]=cropw [28:16]=croph */
#define SD_BURST		0x24	/* @0x3bc..0x428 [7:0]=burst [14:8]=0x78 bit16=1 */
#define SD_DSTDIM		0x28	/* @0x42c/0x440  [12:0]=dstw [28:16]=dsth */
#define SD_HRATIO		0x2c	/* @0x480  Q16 (cropw<<16)/dstw */
#define SD_HDELTA_HI		0x30	/* @0x488  bits[36:32] of (Hratio-1.0, wrapped) */
#define SD_HDELTA_LO		0x34	/* @0x48c  bits[31:0]  of (Hratio-1.0, wrapped) */
#define SD_VRATIO		0x3c	/* @0x4ac  Q16 (croph<<16)/dsth */
#define SD_VDELTA_HI		0x40	/* @0x4b4  bits[36:32] of (Vratio-1.0, wrapped) */
#define SD_VDELTA_LO		0x44	/* @0x4b8  bits[31:0]  of (Vratio-1.0, wrapped) */
#define SD_DSTPHY		0x4c	/* @0x458  dstphy */
#define SD_DSTSTR		0x50	/* @0x460  dststride */
#define SD_LUTPHY		0x54	/* @0x4bc  LUT phys (3rd arg) */

/* CoordinateConvetor_axi / inline ratio constants (@.text 0x290, 0x41c). */
#define SC_Q16_ONE		0x10000		/* 1.0 in Q16 */
#define SC_DELTA_WRAP		0x1fffff0000ULL	/* 2^37 - 2^16, negative-wrap bias */

/* --- control (clock/power) registers (ctrlbase, 0x8000 window) ------------ *
 * Recovered from ar_scaler_hw_init @0xb0, ar_scaler_poweroff @0x268 and
 * ar_scaler_set_frequency.part.0 @0x0.
 */
#define CTRL_CLK_GATE		0x6010	/* clock-enable gate; bits 0x1900 */
#define CTRL_CLK_MAGIC		0x6200	/* unlock magic 0x00ACCE55 */
#define CTRL_CLK_CFG		0x6204	/* clear bit19, set 0x808 */
#define CTRL_FREQ_DIV		0x405c	/* [15:0] = divider code from frequency */
#define CTRL_CLK_GATE_BITS	0x1900	/* enable bits in CTRL_CLK_GATE */
#define CTRL_CLK_CFG_CLR	0x00080000	/* bit19 cleared in CTRL_CLK_CFG */
#define CTRL_CLK_CFG_SET	0x00000808	/* bits set in CTRL_CLK_CFG */
#define CTRL_MAGIC		0x00ACCE55	/* unlock magic written to CTRL_CLK_MAGIC */

#define CTRL_DEFAULT_PHYS	0x0A100000	/* DT default if "control" absent */
#define CTRL_MAP_SIZE		0x8000
#define SCALER_REG_MIN_SIZE	0x1000

/* Internal DMA buffer sizes (matching the vendor driver's MMZ allocations). */
#define SCALER_LUT_SIZE		0x400		/* ar_scaler_get_lut_size() = 1024 */
#define SCALER_LUT_BUF_SIZE	(SCALER_LUT_SIZE + 0x200)	/* 1536 */
#define SCALER_BATCH_BUF_SIZE	0xff00		/* 65280 */

#define SCALER_WAIT_JIFFIES	500		/* completion timeout (jiffies) */

/* Recovered scaler_dev (248 B) layout (fields relevant to the open port). */
struct scaler_dev {
	void __iomem		*regbase;	/* scaler core regs (0x1000) */
	void __iomem		*ctrlbase;	/* clock/power regs (0x8000) */
	u32			frequency;	/* DT "frequency" / setfreq */
	void			*lut_kvaddr;
	dma_addr_t		lut_phys;
	void			*batch_kvaddr;
	dma_addr_t		batch_phys;
	struct miscdevice	misc;
	int			irq;
	struct completion	completion;
	struct mutex		lock;
	struct proc_dir_entry	*proc_dir;
	bool			suspended;
	bool			dead;		/* set under lock in remove; ops bail */
	struct device		*dev;

	/* last-submitted state, exported via /proc/arscaler/state (oracle). */
	struct ar_scaler_op	last_op;
	u32			last_ctrl;
	bool			last_batch;
	u32			last_batch_cnt;
};

static struct scaler_dev g_scaler;
static bool g_scaler_bound;
static bool g_scaler_initted;

static int scaler_debug_level;
module_param(scaler_debug_level, int, 0644);
MODULE_PARM_DESC(scaler_debug_level, "verbosity of scaler register field dumps");

/*
 * The real 1024-byte interpolation coefficient LUT, extracted verbatim from the
 * vendor ar_scaler.ko local symbol `scaler_lut_table` (.rodata offset 0x0, size
 * 0x400). ar_scaler_hw_init memcpy's these raw bytes into the LUT DMA buffer and
 * points the engine at it (regbase+0x70). Copied byte-for-byte (it is a u8 array
 * moved by memcpy, so there is no endianness ambiguity).
 */
static const u8 scaler_lut_table[SCALER_LUT_SIZE] = {
	0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xfd, 0x1f, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0xf9, 0x2f, 0x00, 0x00, 0x00, 0xf5, 0x3f,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf1, 0x4f, 0x00,
	0x00, 0xff, 0xec, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xfe, 0xec, 0x7f, 0x00, 0x00, 0xfe, 0xe8, 0x8f, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xfd, 0xe4, 0xaf, 0x00, 0x00, 0xfe, 0xe0, 0xbf,
	0xc0, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xe0, 0xdf, 0xc0,
	0xff, 0xfb, 0xdc, 0xff, 0xc0, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xfb, 0xd8, 0x0f, 0xc1, 0xff, 0xf9, 0xd8, 0x2f, 0xc1, 0xff, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xf8, 0xd4, 0x4f, 0xc1, 0xff, 0xf9, 0xd0, 0x5f,
	0x81, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf7, 0xd0, 0x7f, 0x81,
	0xff, 0xf6, 0xcc, 0x9f, 0x81, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xf4, 0xcc, 0xbf, 0x81, 0xff, 0xf3, 0xc8, 0xdf, 0x81, 0xff, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xf2, 0xc8, 0xff, 0x41, 0xff, 0xf1, 0xc4, 0x1f,
	0x42, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef, 0xc4, 0x3f, 0x42,
	0xff, 0xed, 0xc4, 0x5f, 0x42, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xed, 0xc0, 0x7f, 0x02, 0xff, 0xea, 0xc0, 0xaf, 0x02, 0xff, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xe9, 0xbc, 0xcf, 0x02, 0xff, 0xe7, 0xbc, 0xef,
	0x02, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe6, 0xbc, 0x0f, 0xc3,
	0xfe, 0xe3, 0xbc, 0x3f, 0xc3, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xe2, 0xb8, 0x5f, 0xc3, 0xfe, 0xe0, 0xb8, 0x8f, 0x83, 0xfe, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xde, 0xb8, 0xaf, 0x83, 0xfe, 0xdc, 0xb8, 0xcf,
	0x83, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xda, 0xb8, 0xff, 0x43,
	0xfe, 0xd8, 0xb8, 0x1f, 0x44, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xd6, 0xb4, 0x4f, 0x44, 0xfe, 0xd4, 0xb4, 0x7f, 0x04, 0xfe, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xd2, 0xb4, 0x9f, 0x04, 0xfe, 0xcf, 0xb4, 0xcf,
	0x04, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xce, 0xb4, 0xef, 0xc4,
	0xfd, 0xcb, 0xb4, 0x1f, 0xc5, 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xc8, 0xb4, 0x4f, 0xc5, 0xfd, 0xc7, 0xb4, 0x6f, 0x85, 0xfd, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xc4, 0xb4, 0x9f, 0x85, 0xfd, 0xc1, 0xb4, 0xcf,
	0x85, 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xb4, 0xef, 0x45,
	0xfd, 0xbd, 0xb4, 0x1f, 0x46, 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xba, 0xb4, 0x4f, 0x46, 0xfd, 0xb9, 0xb4, 0x6f, 0x06, 0xfd, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xb6, 0xb4, 0x9f, 0x06, 0xfd, 0xb2, 0xb8, 0xcf,
	0x06, 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0xb8, 0xff, 0xc6,
	0xfc, 0xad, 0xb8, 0x2f, 0xc7, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xab, 0xb8, 0x4f, 0xc7, 0xfc, 0xa8, 0xb8, 0x7f, 0xc7, 0xfc, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xa6, 0xb8, 0xaf, 0x87, 0xfc, 0xa3, 0xb8, 0xdf,
	0x87, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0xbc, 0xff, 0x87,
	0xfc, 0x9e, 0xbc, 0x2f, 0x48, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x9b, 0xbc, 0x5f, 0x48, 0xfc, 0x98, 0xbc, 0x8f, 0x48, 0xfc, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x95, 0xc0, 0xaf, 0x48, 0xfc, 0x93, 0xc0, 0xdf,
	0x08, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0xc0, 0x0f, 0x09,
	0xfc, 0x8d, 0xc0, 0x3f, 0x09, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x8a, 0xc4, 0x5f, 0x09, 0xfc, 0x88, 0xc4, 0x8f, 0xc9, 0xfb, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x85, 0xc4, 0xbf, 0xc9, 0xfb, 0x82, 0xc4, 0xef,
	0xc9, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xc8, 0x0f, 0xca,
	0xfb, 0x7d, 0xc8, 0x3f, 0x8a, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x7a, 0xc8, 0x6f, 0x8a, 0xfb, 0x77, 0xcc, 0x8f, 0x8a, 0xfb, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x74, 0xcc, 0xbf, 0x8a, 0xfb, 0x71, 0xcc, 0xef,
	0x8a, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6f, 0xcc, 0x0f, 0x8b,
	0xfb, 0x6b, 0xd0, 0x3f, 0x8b, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x6a, 0xd0, 0x5f, 0x4b, 0xfb, 0x67, 0xd0, 0x8f, 0x4b, 0xfb, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x64, 0xd4, 0xaf, 0x4b, 0xfb, 0x61, 0xd4, 0xdf,
	0x4b, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0xd4, 0xff, 0x4b,
	0xfb, 0x5b, 0xd8, 0x2f, 0x4c, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x59, 0xd8, 0x4f, 0x4c, 0xfb, 0x57, 0xd8, 0x6f, 0x4c, 0xfb, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x53, 0xdc, 0x9f, 0x4c, 0xfb, 0x51, 0xdc, 0xbf,
	0x4c, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4f, 0xdc, 0xdf, 0x4c,
	0xfb, 0x4c, 0xe0, 0xff, 0x4c, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x49, 0xe0, 0x2f, 0x4d, 0xfb, 0x47, 0xe0, 0x4f, 0x4d, 0xfb, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x44, 0xe4, 0x6f, 0x4d, 0xfb, 0x41, 0xe4, 0x8f,
	0x8d, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xe4, 0xaf, 0x8d,
	0xfb, 0x3c, 0xe8, 0xcf, 0x8d, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x3a, 0xe8, 0xef, 0x8d, 0xfb, 0x38, 0xe8, 0x0f, 0x8e, 0xfb, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x35, 0xec, 0x2f, 0x8e, 0xfb, 0x32, 0xec, 0x4f,
	0xce, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xec, 0x5f, 0xce,
	0xfb, 0x2e, 0xf0, 0x7f, 0xce, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x2c, 0xf0, 0x9f, 0xce, 0xfb, 0x2a, 0xf0, 0xaf, 0x0e, 0xfc, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x28, 0xf0, 0xcf, 0x0e, 0xfc, 0x24, 0xf4, 0xef,
	0x4e, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0xf4, 0xff, 0x4e,
	0xfc, 0x22, 0xf4, 0x0f, 0x4f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x1f, 0xf4, 0x2f, 0x8f, 0xfc, 0x1d, 0xf8, 0x3f, 0x8f, 0xfc, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x1b, 0xf8, 0x4f, 0xcf, 0xfc, 0x19, 0xf8, 0x6f,
	0xcf, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xf8, 0x7f, 0x0f,
	0xfd, 0x16, 0xf8, 0x8f, 0x0f, 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x13, 0xfc, 0x9f, 0x4f, 0xfd, 0x11, 0xfc, 0xaf, 0x8f, 0xfd, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x10, 0xfc, 0xbf, 0x8f, 0xfd, 0x0e, 0xfc, 0xcf,
	0xcf, 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0xfc, 0xcf, 0x0f,
	0xfe, 0x0c, 0xfc, 0xdf, 0x0f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x09, 0x00, 0xe0, 0x4f, 0xfe, 0x08, 0x00, 0xe0, 0x8f, 0xfe, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0xf0, 0xcf, 0xfe, 0x06, 0x00, 0xf0,
	0xcf, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0xf0, 0x0f,
	0xff, 0x03, 0x00, 0x00, 0x50, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x02, 0x00, 0x00, 0x90, 0xff, 0x01, 0x00, 0x00, 0xd0, 0xff, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
};

static u32 ar_scaler_get_lut_size(void)
{
	return SCALER_LUT_SIZE;
}

/* ------------------------------------------------------------------------- */

static int ar_scaler_validate(const struct ar_scaler_op *op)
{
	if (!op->srcphy || !op->srcw || !op->srch)
		return -EINVAL;

	/* multiple of 16 */
	if (op->srcstride & 0xf)
		return -EINVAL;

	if (!op->cropw || !op->croph)
		return -EINVAL;

	if (!op->dstphy || !op->dstw || !op->dsth)
		return -EINVAL;

	/* multiple of 16 */
	if (op->dststride & 0xf)
		return -EINVAL;

	if (!op->channels)
		return -EINVAL;

	return 0;
}

/*
 * CoordinateConvetor_axi (@.text 0x290) - the Q16 scale-ratio helper. It is
 * inlined into scaler_set_batch_registers (@0x464 / 0x490) but reproduced here
 * verbatim for the H and V axes:
 *
 *   ratio = ((s64)src << 16) / dst;                       // Q16, stored as u32
 *   delta = (ratio < 0x10000) ? ratio + 0x1fffff0000      // (ratio - 1.0) wrapped
 *                             : ratio - 0x10000;          // into a 37-bit field
 *
 * The wrap bias 0x1fffff0000 == 2^37 - 2^16 turns a would-be-negative
 * (ratio - 1.0) into its 37-bit two's-complement representation; the engine
 * consumes the result as a 5-bit high word [36:32] + a 32-bit low word.
 */
static u32 coordinate_convert(u32 src, u32 dst, u32 *delta_hi, u32 *delta_lo)
{
	s64 ratio = ((s64)(s32)src << 16) / (s32)dst;	/* sxtw/sbfiz + sdiv @0x464 */
	u64 delta;

	if (ratio - SC_Q16_ONE < 0) {			/* subs ...,#0x10000 ; csel mi */
		delta = (u64)ratio + SC_DELTA_WRAP;
	} else {
		delta = (u64)(ratio - SC_Q16_ONE);
	}

	*delta_hi = (u32)((delta >> 32) & 0x1f);		/* ubfx #32,#5 */
	*delta_lo = (u32)delta;

	return (u32)ratio;
}

/*
 * Build one 256-byte descriptor "register image" from an op descriptor, exactly
 * as scaler_set_batch_registers (@.text 0x2b8) does. `w` is indexed by byte
 * offset/4 within the descriptor and must be pre-zeroed. Used for both the live
 * single-op regs (regbase+0x1C) and the batch DMA block (stride 0x100). Each
 * field expression mirrors the vendor and/or-mask sequence at the cited address.
 */
static void scaler_build_image(u32 *w, const struct ar_scaler_op *op, u32 lut_phys)
{
	u32 burst, vmetric, conf;

	/* burst control [7:0] (@0x3cc..0x4e4): 16/channels only when downscaling
	 * horizontally AND the vertical metric stays <= 0xc7f, else 8/channels.
	 * validate() guarantees dstw/dsth/channels != 0.
	 */
	vmetric = (op->croph * 100u / op->dsth) * op->channels;
	if (op->cropw >= op->dstw && vmetric <= 0xc7f)
		burst = 16u / op->channels;
	else
		burst = 8u / op->channels;

	/* CONF (@0x31c..0x368): base 0x1000c, then |4 |8 &~2 (net 0x1000c), then
	 * insert control at [6:5] and channels at [20:16] (bit16 of the base is
	 * cleared by the channels insert). Reproduced as the exact RMW chain.
	 */
	conf = 0x1000c;
	conf |= 0x4;
	conf |= 0x8;
	conf &= ~0x2u;
	conf = (conf & 0xffffff9fu) | (op->control << 5);
	conf = (conf & 0xffe0ffffu) | (op->channels << 16);

	w[SD_SRCPHY / 4]  = op->srcphy;					/* @0x2c8 */
	w[SD_SRCSTR / 4]  = (op->srcstride & 0xffff) | (op->ctrl3c << 16); /* @0x2cc/0x2e0 */
	w[SD_SRCDIM / 4]  = (op->srcw & 0xe000ffff) | (op->srch << 16);	/* @0x2f4/0x308 */
	w[SD_CONF / 4]    = conf;
	w[SD_CROPXY / 4]  = (op->crop_x & 0xe000ffff) | (op->crop_y << 16); /* @0x36c/0x380 */
	w[SD_CROPDIM / 4] = (op->cropw & 0xe000ffff) | (op->croph << 16); /* @0x394/0x3a8 */
	w[SD_BURST / 4]   = (burst & 0xff) | (0x78 << 8) | 0x10000;	/* @0x3bc..0x428 */
	w[SD_DSTDIM / 4]  = (op->dstw & 0xe000ffff) | (op->dsth << 16);	/* @0x42c/0x440 */

	w[SD_HRATIO / 4]  = coordinate_convert(op->cropw, op->dstw,	/* @0x464..0x48c */
					       &w[SD_HDELTA_HI / 4],
					       &w[SD_HDELTA_LO / 4]);
	w[SD_VRATIO / 4]  = coordinate_convert(op->croph, op->dsth,	/* @0x490..0x4b8 */
					       &w[SD_VDELTA_HI / 4],
					       &w[SD_VDELTA_LO / 4]);

	w[SD_DSTPHY / 4]  = op->dstphy;					/* @0x458 */
	w[SD_DSTSTR / 4]  = op->dststride;				/* @0x460 */
	w[SD_LUTPHY / 4]  = lut_phys;					/* @0x4bc */
}

/*
 * Pack one op into the LIVE single-op descriptor registers at regbase+0x1C
 * (single mode; the batch path stages into the DMA block instead - see
 * ar_scaler_start_batch). Mirrors ar_scaler_start_single @0xa40 calling
 * scaler_set_batch_registers with base = regbase+0x1C and lut_phys = sc->lut_phys.
 */
static void scaler_set_batch_registers(struct scaler_dev *sc,
				       const struct ar_scaler_op *op, u32 count)
{
	void __iomem *d = sc->regbase + SC_DESC_BASE;
	u32 img[SC_DESC_STRIDE / 4];

	memset(img, 0, sizeof(img));
	scaler_build_image(img, op, lower_32_bits(sc->lut_phys));

	writel(img[SD_SRCPHY / 4],    d + SD_SRCPHY);
	writel(img[SD_SRCSTR / 4],    d + SD_SRCSTR);
	writel(img[SD_SRCDIM / 4],    d + SD_SRCDIM);
	writel(img[SD_CONF / 4],      d + SD_CONF);
	writel(img[SD_CROPXY / 4],    d + SD_CROPXY);
	writel(img[SD_CROPDIM / 4],   d + SD_CROPDIM);
	writel(img[SD_BURST / 4],     d + SD_BURST);
	writel(img[SD_DSTDIM / 4],    d + SD_DSTDIM);
	writel(img[SD_HRATIO / 4],    d + SD_HRATIO);
	writel(img[SD_HDELTA_HI / 4], d + SD_HDELTA_HI);
	writel(img[SD_HDELTA_LO / 4], d + SD_HDELTA_LO);
	writel(img[SD_VRATIO / 4],    d + SD_VRATIO);
	writel(img[SD_VDELTA_HI / 4], d + SD_VDELTA_HI);
	writel(img[SD_VDELTA_LO / 4], d + SD_VDELTA_LO);
	writel(img[SD_DSTPHY / 4],    d + SD_DSTPHY);
	writel(img[SD_DSTSTR / 4],    d + SD_DSTSTR);
	writel(img[SD_LUTPHY / 4],    d + SD_LUTPHY);

	/* book-keeping for the /proc oracle */
	sc->last_op = *op;
	sc->last_ctrl = img[SD_CONF / 4];
	sc->last_batch = (count > 1);
	sc->last_batch_cnt = count;

	if (scaler_debug_level) {
		dev_info(sc->dev,
			 "pack: src %08x %ux%u/%u crop %u,%u %ux%u dst %08x %ux%u/%u hr=%08x vr=%08x ch=%u cnt=%u\n",
			 op->srcphy, op->srcw, op->srch, op->srcstride,
			 op->crop_x, op->crop_y, op->cropw, op->croph,
			 op->dstphy, op->dstw, op->dsth, op->dststride,
			 img[SD_HRATIO / 4], img[SD_VRATIO / 4], op->channels, count);
	}
}

static void ar_scaler_dump_reg(struct scaler_dev *sc)
{
	void __iomem *d = sc->regbase + SC_DESC_BASE;

	dev_err(sc->dev,
		"reg dump: CTRL=%08x STATUS=%08x BATCHCFG=%08x SRCPHY=%08x SRCDIM=%08x DSTPHY=%08x DSTDIM=%08x CROPDIM=%08x HR=%08x VR=%08x\n",
		readl(sc->regbase + SC_REG_CTRL),
		readl(sc->regbase + SC_REG_STATUS),
		readl(sc->regbase + SC_REG_BATCHCFG),
		readl(d + SD_SRCPHY),
		readl(d + SD_SRCDIM),
		readl(d + SD_DSTPHY),
		readl(d + SD_DSTDIM),
		readl(d + SD_CROPDIM),
		readl(d + SD_HRATIO),
		readl(d + SD_VRATIO));
}

/*
 * Submit (single or already-staged batch) and block on completion.
 * Lock order / sequence per the vendor driver: lock; suspended->-EAGAIN; validate; pack;
 * trigger (clear CTRL bit0, write TRIGGER=1); wait_for_completion_timeout.
 */
static void ar_scaler_hw_init(struct scaler_dev *sc);

/*
 * Timeout recovery, called under sc->lock: the engine is still running and its
 * late DONE IRQ would satisfy the next op's completion wait. Mask the IRQ,
 * clock-gate reset the core (stops the transfer), clear any latched status and
 * drop any complete() that already slipped in.
 */
static void ar_scaler_timeout_reset(struct scaler_dev *sc)
{
	writel(0, sc->regbase + SC_REG_IRQEN);
	synchronize_irq(sc->irq);
	ar_scaler_hw_init(sc);

	/* hw_init re-enables the IRQ; keep it masked until any status latched
	 * across the clock-gate reset is cleared and the completion is fresh,
	 * or a late DONE could complete() after the reinit below.
	 */
	writel(0, sc->regbase + SC_REG_IRQEN);
	writel(readl(sc->regbase + SC_REG_STATUS),
	       sc->regbase + SC_REG_STATUS);
	synchronize_irq(sc->irq);
	reinit_completion(&sc->completion);
	writel(0x80000000u, sc->regbase + SC_REG_IRQEN);
}

static long ar_scaler_run(struct scaler_dev *sc, const struct ar_scaler_op *op,
			  u32 count)
{
	unsigned long left;
	u32 ctrl;
	long ret;

	ret = mutex_lock_interruptible(&sc->lock);
	if (ret)
		return -EINTR;

	if (sc->suspended || sc->dead) {
		ret = sc->dead ? -ENODEV : -EAGAIN;
		mutex_unlock(&sc->lock);
		return ret;
	}

	ret = ar_scaler_validate(op);
	if (ret) {
		mutex_unlock(&sc->lock);
		return ret;
	}

	reinit_completion(&sc->completion);

	/* Single mode (CTRL bit0 = 0) must be selected BEFORE packing the live
	 * descriptor image: with bit0 still set from a previous batch op the
	 * engine ignores writes to the regbase+0x1C descriptor registers, and
	 * the trigger runs a stale op that completes (IRQ fires) without
	 * writing the destination (HW-observed 2026-07-17). The vendor driver
	 * packs first and clears bit0 only at trigger time; its userspace
	 * never mixes modes, so it never hits this.
	 */
	ctrl = readl(sc->regbase + SC_REG_CTRL);
	ctrl &= ~1u;
	writel(ctrl, sc->regbase + SC_REG_CTRL);

	scaler_set_batch_registers(sc, op, count);

	/* trigger: reg[24] = 1 */
	writel(1, sc->regbase + SC_REG_TRIGGER);

	left = wait_for_completion_timeout(&sc->completion, SCALER_WAIT_JIFFIES);
	if (!left) {
		ar_scaler_dump_reg(sc);
		ar_scaler_timeout_reset(sc);
		mutex_unlock(&sc->lock);
		return -ETIMEDOUT;
	}

	mutex_unlock(&sc->lock);
	return 0;
}

static long ar_scaler_start_single(struct scaler_dev *sc, void __user *arg)
{
	struct ar_scaler_op op;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	return ar_scaler_run(sc, &op, 1);
}

/*
 * Validate, stage and run `count` ops through batch mode; blocks on completion.
 * Shared by the vendor SCALER_IOC_BATCH and the dmabuf batch (which resolves
 * its fds into ops[] before calling).
 */
static long ar_scaler_run_batch(struct scaler_dev *sc,
				const struct ar_scaler_op *ops, u32 count)
{
	long ret;
	u32 i;

	if (count == 0 || count > SCALER_BATCH_MAX)
		return -EINVAL;

	for (i = 0; i < count; i++) {
		ret = ar_scaler_validate(&ops[i]);
		if (ret)
			return ret;
	}

	ret = mutex_lock_interruptible(&sc->lock);
	if (ret)
		return -EINTR;

	if (sc->suspended || sc->dead) {
		ret = sc->dead ? -ENODEV : -EAGAIN;
		mutex_unlock(&sc->lock);
		return ret;
	}

	/* Stage the descriptor block the HW walks in batch mode. Recovered from
	 * ar_scaler_start_batch @0x744..0x828: zero the whole 0xff00 block, then
	 * pack each op into a 256-byte register image at stride 0x100 with
	 * scaler_build_image (same packer as the single-op live regs).
	 */
	memset(sc->batch_kvaddr, 0, SCALER_BATCH_BUF_SIZE);
	for (i = 0; i < count; i++) {
		scaler_build_image((u32 *)((u8 *)sc->batch_kvaddr +
					   i * SC_DESC_STRIDE),
				   &ops[i], lower_32_bits(sc->lut_phys));
	}

	/* /proc oracle book-keeping (no live single-op regs are written in
	 * batch mode; the HW DMAs the staged block instead).
	 */
	sc->last_op = ops[0];
	sc->last_batch = true;
	sc->last_batch_cnt = count;

	reinit_completion(&sc->completion);

	{
		/* batch trigger (recovered @0x834..0x874): set CTRL bit0 = batch
		 * mode, point at the DMA block, write the op count into
		 * BATCHCFG[31:16], then strobe TRIGGER.
		 */
		u32 ctrl = readl(sc->regbase + SC_REG_CTRL);
		u32 cfg;
		unsigned long left;

		ctrl |= 1u;
		writel(ctrl, sc->regbase + SC_REG_CTRL);
		writel(lower_32_bits(sc->batch_phys),
		       sc->regbase + SC_REG_BATCHADDR);
		cfg = readl(sc->regbase + SC_REG_BATCHCFG) & 0xffff;
		cfg |= count << 16;
		writel(cfg, sc->regbase + SC_REG_BATCHCFG);
		writel(1, sc->regbase + SC_REG_TRIGGER);

		left = wait_for_completion_timeout(&sc->completion,
						   SCALER_WAIT_JIFFIES);
		if (!left) {
			ar_scaler_dump_reg(sc);
			ar_scaler_timeout_reset(sc);
			ret = -ETIMEDOUT;
		} else {
			ret = 0;
		}
	}

	mutex_unlock(&sc->lock);
	return ret;
}

static long ar_scaler_start_batch(struct scaler_dev *sc, void __user *arg)
{
	struct ar_scaler_batch *b;
	long ret;

	/* The batch arg is 16324 B. The vendor uses kmalloc_order;
	 * plain kmalloc is fine on this kernel.
	 */
	b = kmalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	if (copy_from_user(b, arg, sizeof(*b)))
		ret = -EFAULT;
	else
		ret = ar_scaler_run_batch(sc, b->ops, b->count);

	kfree(b);
	return ret;
}

/*
 * Frequency -> clock-divider code, recovered verbatim from
 * ar_scaler_set_frequency.part.0 (@.text 0x0). The threshold ladder and codes
 * are exact; the code is written into CTRL_FREQ_DIV[15:0] (RMW, high half kept).
 */
static u32 scaler_freq_to_div(u32 freq)
{
	/* > 0x257 */
	if (freq > 599)
		return 0x1400;

	/* 0x1f4..0x257 */
	if (freq > 499)
		return 0x1100;

	/* 0x190..0x1f3 */
	if (freq > 399)
		return 0x1200;

	/* 0x14d..0x18f */
	if (freq > 332)
		return 0x1300;

	/* 0x12c..0x14c */
	if (freq > 299)
		return 0x1401;

	/* 0xc8..0x12b */
	if (freq > 199)
		return 0x1402;

	/* 0x96..0xc7 */
	if (freq > 149)
		return 0x1403;

	/* 0x64..0x95 */
	if (freq > 99)
		return 0x1405;

	/* <= 0x63 */
	return 0x1207;
}

static void ar_scaler_set_freq_div(struct scaler_dev *sc, u32 freq)
{
	u32 v = readl(sc->ctrlbase + CTRL_FREQ_DIV) & 0xffff0000;	/* @0x28 */

	v |= scaler_freq_to_div(freq);
	writel(v, sc->ctrlbase + CTRL_FREQ_DIV);			/* @0x44 */
}

static long ar_scaler_set_frequency(struct scaler_dev *sc, void __user *arg)
{
	int freq;
	if (copy_from_user(&freq, arg, sizeof(freq)))
		return -EFAULT;

	mutex_lock(&sc->lock);
	if (sc->dead) {
		mutex_unlock(&sc->lock);
		return -ENODEV;
	}

	/* ar_scaler_set_frequency @0xcb8: only programs the divider when != 0. */
	sc->frequency = (u32)freq;
	if (freq)
		ar_scaler_set_freq_div(sc, (u32)freq);
	mutex_unlock(&sc->lock);

	return 0;
}

/* ------------------------------------------------------------------------- */

/* Per-open state for the dmabuf ABI family (SCALER_IOC_*_DMABUF): the fd ->
 * contiguous-phys mapping cache plus its lock. The vendor phys-by-value
 * ioctls never touch it.
 */
struct scaler_client {
	struct mutex		lock;
	struct ml_bufcache	cache;
};

/* A batch pins up to 2 mappings per op; they must all fit in the cache with
 * this batch's stamps newest, or the LRU could evict an earlier op's mapping
 * before the HW runs.
 */
static_assert(2 * SCALER_DMABUF_BATCH_MAX < ML_BUFCACHE_ENTRIES);

/*
 * Resolve one dmabuf op into a phys-addressed vendor op. Called under
 * cl->lock, which the caller holds across the HW run too: the resolved phys
 * stay pinned while cached, and an LRU eviction (only possible under cl->lock)
 * can never unpin a segment the engine is still using.
 */
static long ar_scaler_resolve_dmabuf_op(struct scaler_dev *sc,
					struct scaler_client *cl,
					const struct ar_scaler_dmabuf_op *dop,
					struct ar_scaler_op *op)
{
	struct ml_bufmap *src, *dst;
	u64 phys;

	*op = dop->op;

	/* the fds are the address source; a stray phys is a caller bug */
	if (op->srcphy || op->dstphy)
		return -EINVAL;

	if ((dop->src_off | dop->dst_off) & 0xf)
		return -EINVAL;

	/* geometry the bounds math below depends on (ar_scaler_validate runs
	 * later, but only after these fields have already been multiplied)
	 */
	if (!op->srch || !op->srcstride || !op->dsth || !op->dststride ||
	    !op->channels)
		return -EINVAL;

	if ((u64)op->srcw * op->channels > op->srcstride ||
	    (u64)op->dstw * op->channels > op->dststride)
		return -EINVAL;

	if ((u64)op->crop_x + op->cropw > op->srcw ||
	    (u64)op->crop_y + op->croph > op->srch)
		return -EINVAL;

	src = ml_bufcache_map_fd(&cl->cache, sc->dev, dop->src_fd,
				 DMA_TO_DEVICE);
	if (IS_ERR(src))
		return PTR_ERR(src);

	/* src_fd == dst_fd remaps the SAME cache slot BIDIRECTIONAL, so src's
	 * base/size are only valid read after this call: keep every use of
	 * src below the dst map.
	 */
	dst = ml_bufcache_map_fd(&cl->cache, sc->dev, dop->dst_fd,
				 DMA_FROM_DEVICE);
	if (IS_ERR(dst))
		return PTR_ERR(dst);

	/* whole declared images inside their mapped segments */
	if ((u64)dop->src_off + (u64)op->srch * op->srcstride > src->size)
		return -EINVAL;

	if ((u64)dop->dst_off + (u64)op->dsth * op->dststride > dst->size)
		return -EINVAL;

	/* the descriptor takes 32-bit phys */
	phys = (u64)src->base + dop->src_off;
	if (upper_32_bits(phys))
		return -ERANGE;

	op->srcphy = lower_32_bits(phys);
	phys = (u64)dst->base + dop->dst_off;
	if (upper_32_bits(phys))
		return -ERANGE;

	op->dstphy = lower_32_bits(phys);
	return 0;
}

static long ar_scaler_start_single_dmabuf(struct scaler_dev *sc,
					  struct scaler_client *cl,
					  void __user *arg)
{
	struct ar_scaler_dmabuf_op dop;
	struct ar_scaler_op op;
	long ret;

	if (copy_from_user(&dop, arg, sizeof(dop)))
		return -EFAULT;

	mutex_lock(&cl->lock);
	ret = ar_scaler_resolve_dmabuf_op(sc, cl, &dop, &op);
	if (!ret)
		ret = ar_scaler_run(sc, &op, 1);
	mutex_unlock(&cl->lock);

	return ret;
}

static long ar_scaler_start_batch_dmabuf(struct scaler_dev *sc,
					 struct scaler_client *cl,
					 void __user *arg)
{
	struct ar_scaler_dmabuf_batch b;
	struct ar_scaler_op ops[SCALER_DMABUF_BATCH_MAX];
	long ret;
	u32 i;

	if (copy_from_user(&b, arg, sizeof(b)))
		return -EFAULT;

	if (b.count == 0 || b.count > SCALER_DMABUF_BATCH_MAX || b.reserved)
		return -EINVAL;

	mutex_lock(&cl->lock);
	ret = 0;
	for (i = 0; i < b.count && !ret; i++)
		ret = ar_scaler_resolve_dmabuf_op(sc, cl, &b.ops[i], &ops[i]);
	if (!ret)
		ret = ar_scaler_run_batch(sc, ops, b.count);
	mutex_unlock(&cl->lock);

	return ret;
}

/* ------------------------------------------------------------------------- */

static irqreturn_t ar_scaler_irq_handler(int irq, void *dev_id)
{
	struct scaler_dev *sc = dev_id;
	u32 status;

	status = readl(sc->regbase + SC_REG_STATUS);
	writel(status, sc->regbase + SC_REG_STATUS);	/* W1C */

	if (status & SC_STATUS_DONE)
		complete(&sc->completion);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------- */

/*
 * Clock bring-up + LUT upload + core-register init, recovered 1:1 from
 * ar_scaler_hw_init (@.text 0xb0). Sequence:
 *   - if frequency set, program the divider (matches @0xdc/0x238);
 *   - gate clock OFF (CTRL_CLK_GATE &= ~0x1900), udelay(100);
 *   - read CTRL_CLK_CFG, clear bit19 (held);
 *   - write the unlock magic 0x00ACCE55 to CTRL_CLK_MAGIC;
 *   - write CTRL_CLK_CFG = (read & ~bit19) | 0x808; msleep(5);
 *   - gate clock ON (CTRL_CLK_GATE |= 0x1900), udelay(100);
 *   - memcpy the 1024-byte LUT into the LUT DMA buffer;
 *   - seed the scaler core registers (defaults + LUT phys + IRQ enable).
 * Getting this wrong means the completion IRQ never fires and every op times
 * out at SCALER_WAIT_JIFFIES.
 */
static void ar_scaler_hw_init(struct scaler_dev *sc)
{
	u32 v, cfg;

	if (!sc->ctrlbase) {		/* cbz x20 -> printk(-1) @0x240 */
		dev_err(sc->dev, "no control block; clock bring-up skipped\n");
		return;
	}

	if (sc->frequency) {		/* @0xd8 cbnz -> set_frequency.part.0 */
		ar_scaler_set_freq_div(sc, sc->frequency);
	}

	/* gate clock off (@0xe8..0xfc), then settle */
	v = readl(sc->ctrlbase + CTRL_CLK_GATE);
	v &= ~(u32)CTRL_CLK_GATE_BITS;
	writel(v, sc->ctrlbase + CTRL_CLK_GATE);
	udelay(100);			/* __const_udelay(0x68dbc) */

	/* read cfg, clear bit19 (held in cfg); write magic; write cfg|0x808 */
	cfg = readl(sc->ctrlbase + CTRL_CLK_CFG);	/* @0x118 */
	cfg &= ~(u32)CTRL_CLK_CFG_CLR;
	writel(CTRL_MAGIC, sc->ctrlbase + CTRL_CLK_MAGIC);	/* @0x13c */
	cfg |= CTRL_CLK_CFG_SET;
	writel(cfg, sc->ctrlbase + CTRL_CLK_CFG);		/* @0x154 */
	usleep_range(5000, 6000);

	/* gate clock on (@0x160..0x174), then settle */
	v = readl(sc->ctrlbase + CTRL_CLK_GATE);
	v |= CTRL_CLK_GATE_BITS;
	writel(v, sc->ctrlbase + CTRL_CLK_GATE);
	udelay(100);

	/* upload the interpolation LUT (@0x18c memcpy, 1024 bytes) */
	if (sc->lut_kvaddr)
		memcpy(sc->lut_kvaddr, scaler_lut_table, ar_scaler_get_lut_size());

	/* core register seed (@0x194..0x220), all on regbase */
	v = readl(sc->regbase + SC_REG_CTRL);
	v &= ~1u;					/* clear mode/start bit0 */
	writel(v, sc->regbase + SC_REG_CTRL);

	/* CONF default 0x1040d, then |4|8 &~2 &~0x60 (net 0x1040d) */
	writel(0x1040d, sc->regbase + SC_REG_CONF);
	v = readl(sc->regbase + SC_REG_CONF);
	v |= 0x4;
	v |= 0x8;
	v &= ~0x2u;
	v &= ~0x60u;
	writel(v, sc->regbase + SC_REG_CONF);

	/* BURST default: [7:0]=4 then [14:8]=0x78 (net 0x7804) */
	v = readl(sc->regbase + SC_REG_BURST);
	v = (v & ~0xffu) | 0x4;
	writel(v, sc->regbase + SC_REG_BURST);
	v = readl(sc->regbase + SC_REG_BURST);
	v = (v & ~0x7f00u) | 0x7800;
	writel(v, sc->regbase + SC_REG_BURST);

	/* LUT phys (also re-written per-op by the packer) */
	writel(lower_32_bits(sc->lut_phys), sc->regbase + SC_REG_LUTADDR);

	/* completion-IRQ enable: IRQEN = 0 then |= bit31 */
	writel(0, sc->regbase + SC_REG_IRQEN);
	v = readl(sc->regbase + SC_REG_IRQEN);
	v |= 0x80000000u;
	writel(v, sc->regbase + SC_REG_IRQEN);

	/* CONF |= 8 (final step @0x218) */
	v = readl(sc->regbase + SC_REG_CONF);
	v |= 0x8;
	writel(v, sc->regbase + SC_REG_CONF);
}

static void ar_scaler_poweroff(struct scaler_dev *sc)
{
	/* clear the clock-enable bits (recovered @0x268: reg &= ~0x1900) */
	u32 v = readl(sc->ctrlbase + CTRL_CLK_GATE);

	v &= ~(u32)CTRL_CLK_GATE_BITS;
	writel(v, sc->ctrlbase + CTRL_CLK_GATE);
}

/* ------------------------------------------------------------------------- */

static int ar_scaler_open(struct inode *ino, struct file *f)
{
	struct scaler_client *cl = kzalloc(sizeof(*cl), GFP_KERNEL);

	if (!cl)
		return -ENOMEM;

	mutex_init(&cl->lock);
	f->private_data = cl;

	return 0;
}

static int ar_scaler_release(struct inode *ino, struct file *f)
{
	struct scaler_client *cl = f->private_data;

	ml_bufcache_release(&cl->cache);
	kfree(cl);
	return 0;
}

static ssize_t ar_scaler_read(struct file *f, char __user *u, size_t n,
			      loff_t *off)
{
	return 0;
}

static ssize_t ar_scaler_write(struct file *f, const char __user *u, size_t n,
			       loff_t *off)
{
	return n;
}

static long ar_scaler_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct scaler_dev *sc = &g_scaler;
	void __user *uarg = (void __user *)arg;
	unsigned int sz = _IOC_SIZE(cmd);

	if (_IOC_TYPE(cmd) != SCALER_MAGIC)
		return -EINVAL;

	switch (cmd) {
	case SCALER_IOC_SINGLE: {
		if (sz != sizeof(struct ar_scaler_op))
			return -EINVAL;

		return ar_scaler_start_single(sc, uarg);
	}

	case SCALER_IOC_BATCH: {
		if (sz != sizeof(struct ar_scaler_batch))
			return -EINVAL;

		return ar_scaler_start_batch(sc, uarg);
	}

	case SCALER_IOC_SETFREQ: {
		if (sz != sizeof(int))
			return -EINVAL;

		return ar_scaler_set_frequency(sc, uarg);
	}

	case SCALER_IOC_SINGLE_DMABUF: {
		if (sz != sizeof(struct ar_scaler_dmabuf_op))
			return -EINVAL;

		return ar_scaler_start_single_dmabuf(sc, f->private_data, uarg);
	}

	case SCALER_IOC_BATCH_DMABUF: {
		if (sz != sizeof(struct ar_scaler_dmabuf_batch))
			return -EINVAL;

		return ar_scaler_start_batch_dmabuf(sc, f->private_data, uarg);
	}

	default: {
		return -EINVAL;
	}
	}
}

static const struct file_operations ar_scaler_fops = {
	.owner		= THIS_MODULE,
	.open		= ar_scaler_open,
	.release	= ar_scaler_release,
	.read		= ar_scaler_read,
	.write		= ar_scaler_write,
	.unlocked_ioctl	= ar_scaler_ioctl,
	/* NO mmap, NO compat_ioctl (regs not exposed to userspace). */
};

/* ------------------------------------------------------------------------- */

static int ar_scaler_state_show(struct seq_file *m, void *v)
{
	struct scaler_dev *sc = m->private;
	const struct ar_scaler_op *op = &sc->last_op;

	mutex_lock(&sc->lock);
	seq_puts(m, "------------------- scaler state -------------------\n");
	seq_printf(m, "frequency   : %u\n", sc->frequency);
	seq_printf(m, "suspended   : %s\n", sc->suspended ? "Y" : "N");
	seq_printf(m, "mode        : %s\n", sc->last_batch ? "Batch" : "Single");
	seq_printf(m, "batch num   : %u\n", sc->last_batch_cnt);
	seq_printf(m, "src img     : Addr 0x%08x  W %u  H %u  Stride %u\n",
		   op->srcphy, op->srcw, op->srch, op->srcstride);
	seq_printf(m, "crop        : X %u  Y %u  W %u  H %u\n",
		   op->crop_x, op->crop_y, op->cropw, op->croph);
	seq_printf(m, "dst img     : Addr 0x%08x  W %u  H %u  Stride %u\n",
		   op->dstphy, op->dstw, op->dsth, op->dststride);
	seq_printf(m, "channels    : %u\n", op->channels);
	seq_printf(m, "ctrl word   : 0x%08x (interp %u, ctrl3c 0x%x)\n",
		   sc->last_ctrl, op->interp, op->ctrl3c);
	seq_printf(m, "LUT phys    : 0x%08x\n", lower_32_bits(sc->lut_phys));
	seq_printf(m, "batch phys  : 0x%08x\n", lower_32_bits(sc->batch_phys));
	mutex_unlock(&sc->lock);
	return 0;
}

/* ------------------------------------------------------------------------- */

/* The LUT and the batch descriptor block are read by the engine over DMA;
 * dma_alloc_coherent gives a CPU-uncoherent-safe mapping and a 32-bit handle
 * (no IOMMU, handle == phys), same semantics as the vendor's WC-mapped MMZ
 * blocks.
 */
static int ar_scaler_alloc_bufs(struct scaler_dev *sc)
{
	sc->lut_kvaddr = dma_alloc_coherent(sc->dev, SCALER_LUT_BUF_SIZE,
					    &sc->lut_phys, GFP_KERNEL);
	if (!sc->lut_kvaddr)
		return -ENOMEM;

	sc->batch_kvaddr = dma_alloc_coherent(sc->dev, SCALER_BATCH_BUF_SIZE,
					      &sc->batch_phys, GFP_KERNEL);
	if (!sc->batch_kvaddr) {
		dma_free_coherent(sc->dev, SCALER_LUT_BUF_SIZE,
				  sc->lut_kvaddr, sc->lut_phys);
		sc->lut_kvaddr = NULL;

		return -ENOMEM;
	}

	return 0;
}

static void ar_scaler_free_bufs(struct scaler_dev *sc)
{
	if (sc->batch_kvaddr) {
		dma_free_coherent(sc->dev, SCALER_BATCH_BUF_SIZE,
				  sc->batch_kvaddr, sc->batch_phys);
		sc->batch_kvaddr = NULL;
	}

	if (sc->lut_kvaddr) {
		dma_free_coherent(sc->dev, SCALER_LUT_BUF_SIZE,
				  sc->lut_kvaddr, sc->lut_phys);
		sc->lut_kvaddr = NULL;
	}
}

static int ar_scaler_probe(struct platform_device *pdev)
{
	struct scaler_dev *sc = &g_scaler;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	u32 control_phys = CTRL_DEFAULT_PHYS;
	int ret;

	/* g_scaler is a singleton; a second bind would re-init a live mutex
	 * and clobber the first instance's mappings/DMA buffers.
	 */
	if (g_scaler_bound)
		return -EBUSY;

	sc->dev = dev;
	sc->dead = false;
	/* One-time init: a rebind must not re-init a mutex a stale fd's ioctl
	 * may still be holding across its dead-check.
	 */
	if (!g_scaler_initted) {
		mutex_init(&sc->lock);
		init_completion(&sc->completion);
		g_scaler_initted = true;
	}

	/* (1) scaler core registers from DT reg[0], min size 0x1000.
	 *
	 * Map NON-exclusively (devm_ioremap, not devm_platform_ioremap_resource):
	 * the scaler is a functional sub-block of the VO display controller, and
	 * on a DTB whose vo reg window still spans 0x8810000+0x40000 an exclusive
	 * request here fails -EBUSY (vo holds the region). Current DTBs shrink vo
	 * to 0x10000 so the windows are disjoint, but the plain ioremap (same as
	 * the "control"/CGU window below) keeps the module loadable on either DTB.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	if (resource_size(res) < SCALER_REG_MIN_SIZE) {
		dev_err(dev, "scaler reg window too small (%pa)\n", &res->start);
		return -EINVAL;
	}

	sc->regbase = devm_ioremap(dev, res->start, resource_size(res));
	if (!sc->regbase)
		return -ENOMEM;

	/* (2) "control" clock/power block: raw phys from a u32 DT prop (not a
	 * reg resource), default 0x0A100000, mapped size 0x8000.
	 */
	if (of_property_read_u32(np, "control", &control_phys))
		control_phys = CTRL_DEFAULT_PHYS;

	sc->ctrlbase = devm_ioremap(dev, control_phys, CTRL_MAP_SIZE);
	if (!sc->ctrlbase)
		return -ENOMEM;

	/* (3) frequency hint. */
	if (of_property_read_u32(np, "frequency", &sc->frequency))
		sc->frequency = 0;

	/* (4) completion IRQ (GIC 107 / DT SPI 75). */
	sc->irq = platform_get_irq(pdev, 0);
	if (sc->irq < 0)
		return sc->irq;

	ret = devm_request_threaded_irq(dev, sc->irq, ar_scaler_irq_handler,
					NULL, 0, "arscaler", sc);
	if (ret) {
		dev_err(dev, "request_irq %d failed: %d\n", sc->irq, ret);
		return ret;
	}

	/* (5) internal DMA buffers (LUT + batch descriptor block). The engine
	 * takes 32-bit addresses.
	 */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = ar_scaler_alloc_bufs(sc);
	if (ret) {
		dev_err(dev, "DMA buffer alloc failed: %d\n", ret);
		return ret;
	}

	/* (6) clock bring-up + LUT upload. */
	ar_scaler_hw_init(sc);

	/* (7) misc device /dev/arscaler. Root-only: the vendor ABI passes raw
	 * physical source/destination addresses into the DMA engine with no
	 * range check, so whoever can open this node reads and writes
	 * arbitrary physical memory.
	 */
	sc->misc.minor = MISC_DYNAMIC_MINOR;
	sc->misc.name  = "arscaler";
	sc->misc.mode  = 0600;
	sc->misc.fops  = &ar_scaler_fops;
	ret = misc_register(&sc->misc);
	if (ret) {
		dev_err(dev, "misc_register failed: %d\n", ret);

		/* hw_init already clocked the engine, enabled the IRQ, and wrote
		 * lut_phys into SC_REG_LUTADDR; power off before freeing so the
		 * block is not left live with a dangling LUT pointer.
		 */
		ar_scaler_poweroff(sc);
		ar_scaler_free_bufs(sc);
		return ret;
	}

	/* (8) /proc/arscaler/state oracle. */
	sc->proc_dir = proc_mkdir("arscaler", NULL);
	if (sc->proc_dir) {
		proc_create_single_data("state", 0444, sc->proc_dir,
					ar_scaler_state_show, sc);
	}

	platform_set_drvdata(pdev, sc);
	g_scaler_bound = true;
	dev_info(dev, "ready: regs %pa, control 0x%08x, irq %d, freq %u\n",
		 &res->start, control_phys, sc->irq, sc->frequency);
	return 0;
}

static void ar_scaler_remove(struct platform_device *pdev)
{
	struct scaler_dev *sc = platform_get_drvdata(pdev);

	if (sc->proc_dir) {
		remove_proc_entry("state", sc->proc_dir);
		remove_proc_entry("arscaler", NULL);
		sc->proc_dir = NULL;
	}
	misc_deregister(&sc->misc);

	/*
	 * Teardown gate: taking the lock waits out an op in flight (ops hold it
	 * across the completion wait); dead makes every later ioctl bail before
	 * touching the registers devm unmaps or the DMA buffers freed below.
	 */
	mutex_lock(&sc->lock);
	sc->dead = true;
	ar_scaler_poweroff(sc);
	ar_scaler_free_bufs(sc);
	mutex_unlock(&sc->lock);
	g_scaler_bound = false;
}

static int ar_scaler_suspend(struct device *dev)
{
	struct scaler_dev *sc = dev_get_drvdata(dev);

	mutex_lock(&sc->lock);
	ar_scaler_poweroff(sc);
	sc->suspended = true;
	mutex_unlock(&sc->lock);
	dev_info(dev, "Scaler suspended.\n");

	return 0;
}

static int ar_scaler_resume(struct device *dev)
{
	struct scaler_dev *sc = dev_get_drvdata(dev);

	mutex_lock(&sc->lock);
	ar_scaler_hw_init(sc);
	sc->suspended = false;
	mutex_unlock(&sc->lock);
	dev_info(dev, "Scaler resumed.\n");

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ar_scaler_pm_ops,
				ar_scaler_suspend, ar_scaler_resume);

static const struct of_device_id ar_scaler_of[] = {
	{ .compatible = "artosyn,scaler" },
	{ }
};
MODULE_DEVICE_TABLE(of, ar_scaler_of);

static struct platform_driver ar_scaler_driver = {
	.probe	= ar_scaler_probe,
	.remove	= ar_scaler_remove,
	.driver	= {
		.name		= "ar_scaler",
		.of_match_table	= ar_scaler_of,
		.pm		= pm_sleep_ptr(&ar_scaler_pm_ops),
	},
};
module_platform_driver(ar_scaler_driver);

/* dma_buf_* (the SCALER_IOC_*_DMABUF resolver) live in the DMA_BUF namespace. */
MODULE_IMPORT_NS("DMA_BUF");

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("missinglynk (open reimpl)");
MODULE_DESCRIPTION("Artosyn /dev/arscaler MPP scaler / crop-resize engine (open)");
