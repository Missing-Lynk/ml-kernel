// SPDX-License-Identifier: GPL-2.0
/*
 * artosyn_sdio.ko - open reimplementation of the closed Artosyn AR8030 RF-link
 * SDIO driver - the goggle-side endpoint of the FPV link (video/audio downlink
 * in; RC/telemetry/commands + a bidirectional IPv4 tunnel both ways) for
 * missinglynk on Linux 6.18.
 *
 * It reproduces the vendor module's userspace ABI and on-wire contract so the
 * closed userspace (ar_lowdelay, libar_minirtsp.so) and the signed baseband
 * firmware blob (bb_demo_gnd_d.img on this goggle) run unchanged on top:
 *
 *   - SDIO function match by VID/PID 4152:8030 (download fw) / 4152:8031 (skip).
 *   - "SD"-framed ROM-loader firmware uploader (request_firmware, 64-B header to
 *     0x2f0040, <=0xff4 chunks with a 12-B packet header, addr=0/len=0 finalize).
 *   - /dev/artosyn_sdio misc char device with the 7 'v'-type ioctls (the bb_ioctl
 *     primitive + raw register/CCCR peek/poke + a 16-bit mailbox).
 *   - the sdio0 NOARP IPv4 point-to-point tunnel (IP-header compression + a
 *     0x12345678/0x87654321/CRC-32 fragment/video wire format).
 *
 * Recovered from the vendor driver disassembly.
 *
 * TODO(live-trace): 0x13/0x68 status bit meanings; CMD53 fixed-vs-incrementing
 * addressing at IO addr 0; bb_ioctl command form; on-wire fragment layout.
 */
#define pr_fmt(fmt) "artosyn_sdio: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <net/ip.h>
#include <net/checksum.h>
#include <linux/crc32.h>
#include <linux/firmware.h>
#include <linux/string.h>
#include <linux/atomic.h>
#include <linux/refcount.h>
#include <linux/completion.h>
#include <linux/unaligned.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>

/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

#define ARSDIO_VENDOR		0x4152		/* "AR" */
#define ARSDIO_DEV_DOWNLOAD	0x8030		/* needs firmware download */
#define ARSDIO_DEV_SKIPFW	0x8031		/* already programmed */

#define ARSDIO_BLOCK		512		/* SDIO block size for CMD53 */
#define ARSDIO_FIFO_ADDR	0x00		/* bulk data FIFO (CMD53 IO addr) */

/*
 * Host-controller register map (CMD52). The IRQ handler reads 0x13 (busy) in a
 * loop, then reads the 0x68 event bitmap and dispatches bits 0..3 to data
 * registers 0x54/0x58/0x5c/0x60. 0x44/0x48 are the mailbox-post registers
 * (MSG ioctl nr 2), NOT a bb_ioctl doorbell.
 */
#define REG_STATUS		0x13		/* global IRQ/busy status (bit4) */
#define REG_CTRL		0x14		/* function/IO control enable */
#define REG_MBOX_TX0		0x44		/* host->chip mailbox msg, channel 0 (MSG post) */
#define REG_MBOX_TX1		0x48		/* host->chip mailbox msg, channel 1 (MSG post) */
#define REG_MBOX_RX0		0x54		/* chip->host mailbox msg, channel 0 (0x68 bit0) */
#define REG_MBOX_RX1		0x58		/* chip->host mailbox msg, channel 1 (0x68 bit1) */
#define REG_RX_COUNT		0x5c		/* RX blocks available  (0x68 bit2), byte<<9 = bytes */
#define REG_TX_CREDIT		0x60		/* TX blocks of credit  (0x68 bit3), byte<<9 = bytes */
#define REG_EVT_STATUS		0x68		/* event-status / IRQ source bitmap (bits 0..3) */

#define STATUS_BUSY_BIT		BIT(4)		/* IRQ drains while set */
#define EVT_MBOX0		BIT(0)		/* reg 0x54 has a message */
#define EVT_MBOX1		BIT(1)		/* reg 0x58 has a message */
#define EVT_RX			BIT(2)		/* reg 0x5c: RX data available */
#define EVT_TX			BIT(3)		/* reg 0x60: TX credit granted */

/* Firmware ROM-loader ("SD"-framed) protocol, §B of the spec doc. */
#define SPL_ROM_HEADER_ADDR	0x2f0040	/* dest for the 64-B SPL header */
#define SD_FRAME_MAGIC		0x4453		/* "SD" little-endian */
#define SD_FRAME_HDR_LEN	12		/* u16 magic + u16 len + u64 addr */
#define SD_FRAME_PAYLOAD_MAX	0xff4		/* 4084 = 4096-12 (one 8-block frame) */
#define SD_FRAME_BLK_PAYLOAD	(ARSDIO_BLOCK - SD_FRAME_HDR_LEN)  /* 500 = 512-12 (one block) */
#define CCCR_READY_REG		0x03		/* func-0 reg gating ROM loader */
#define CCCR_READY_BIT		BIT(1)		/* set => ROM loader ready */

#define ARSDIO_SCRATCH		4096		/* ROM-loader frame scratch (<=4 KB) */
#define ARSDIO_STAGING		65536		/* TX staging (64 KB) */
#define ARSDIO_RX_BUF		0x21000		/* RX buffer >= max burst (255 blocks << 9 = 130560) */
#define ARSDIO_MAX_RUNS		64		/* max block runs recovered from one merged RX read */
#define ARSDIO_INFLIGHT_CAP	1024		/* 0x400 TX in-flight cap */
#define ARSDIO_MAX_CHANNELS	32

/*
 * Link-frame trailer. Every SDIO transfer (TX and RX) ends with a 4-byte
 * trailer { 0x00, type, len_lo, len_hi } occupying the last 4 bytes of the
 * final 512-byte block. `len` is the total length of the meaningful payload
 * that precedes the trailer in the block run. The type byte selects the
 * traffic class:
 */
#define LINK_TYPE_IP		0xCC	/* IPv4 tunnel data (data_txq / netif_rx) */
#define LINK_TYPE_CMD		0xDD	/* bb command / response (cmd_txq / cmd_rxq) */
#define LINK_TYPE_ACK		0xEE	/* flow-control ACK: big-endian u32 acked-byte count */
#define LINK_TRAILER_LEN	4
#define IP_HEAD_SHRINK		8	/* build_ip_head() compresses the IPv4 header 20 -> 12 */

/*
 * The IPv4 tunnel ships a 12-byte compressed header: the 20-byte IPv4 header
 * minus the 8 leading bytes, with only the last octet of each address carried
 * (the link is point-to-point 10.0.0.x). See artosyn_sdio_build/restore.
 */
#define IP_COMPRESSED_HDR	12

/* On-wire video header magics, §E (video_packet_header_t). */
#define VPH_MAGIC		0x12345678u
#define VPH_TAIL_MAGIC		0x87654321u
#define VPH_CRC_LEN		32		/* CRC-32 covers the first 32 B */

/* video/other port (type) bytes, §2/§E */
#define PORT_VIDEO_A		0x03
#define PORT_VIDEO_B		0x23
#define PORT_OTHER_A		0x04
#define PORT_OTHER_B		0x24

/* -------------------------------------------------------------------------- */
/* ioctl ABI (type 'v' = 0x76), §3/§F - byte-exact                            */
/* -------------------------------------------------------------------------- */

struct artosyn_cmd_args {		/* 32 bytes */
	__u32	opcode;			/* 0x1xxxxxx RX / 0x2xxxxxx TX command */
	__u32	arg;			/* input argument word */
	__u32	flags;
	__u32	retries;		/* SDIO round-trip retry count */
	__u8	resp[16];		/* response payload to userspace */
};

struct artosyn_rw_args {		/* 12 bytes */
	__u32	addr;
	__u32	length;
	__u32	val;
};

struct artosyn_msg_args {		/* 4 bytes */
	__u16	valid;
	__u16	message;
};

#define ARSDIO_IOC_CMD		_IOWR('v', 0, struct artosyn_cmd_args)	/* 0xC0207600 */
#define ARSDIO_IOC_REG_RD	_IOR('v', 1, struct artosyn_rw_args)	/* 0x800C7601 */
#define ARSDIO_IOC_REG_WR	_IOW('v', 1, struct artosyn_rw_args)	/* 0x400C7601 */
#define ARSDIO_IOC_MSG_GET	_IOWR('v', 2, struct artosyn_msg_args)	/* 0xC0047602 */
#define ARSDIO_IOC_MSG_POST	_IOW('v', 2, struct artosyn_msg_args)	/* 0x40047602 */
#define ARSDIO_IOC_CCCR_RD	_IOR('v', 3, struct artosyn_rw_args)	/* 0x800C7603 */
#define ARSDIO_IOC_CCCR_WR	_IOW('v', 3, struct artosyn_rw_args)	/* 0x400C7603 */

/* -------------------------------------------------------------------------- */
/* Firmware container header (_spl_header_s, 64 B), §5/§B                      */
/* -------------------------------------------------------------------------- */

struct spl_header {			/* 64 bytes */
	__u32	magic;			/* +0  */
	__u32	img_type;		/* +4  */
	__u32	header_len;		/* +8  */
	__u32	header_checksum;	/* +12 */
	__u32	img_version;		/* +16 */
	__u32	flag;			/* +20 */
	__u32	sig_load_addr;		/* +24 signature segment load addr */
	__u32	sig_len;		/* +28 signature segment length */
	__u32	spl_load_addr;		/* +32 */
	__u32	spl_len;		/* +36 */
	__u32	troot_load_addr;	/* +40 */
	__u32	troot_len;		/* +44 */
	__u32	spl_dtb_offset;		/* +48 */
	__u32	resv[3];		/* +52 .. +60 (checksum etc.) */
};

/* -------------------------------------------------------------------------- */
/* On-wire video header (video_packet_header_t, 36 B), §E                     */
/* -------------------------------------------------------------------------- */

/* video_packet_header_t (36 bytes). Field at +20 is spelled "u32TimeStap"
 * (sic) in the vendor source.
 */
struct video_packet_header {		/* 36 bytes */
	__u32	MagicCode;		/* +0  = 0x12345678 (u32MagicCode) */
	__u32	StreamLen;		/* +4  payload length (u32StreamLen) */
	__u32	ChnIndex;		/* +8  channel index, valid 0..4 (u32ChnIndex) */
	__u32	isIdrStream;		/* +12 IDR keyframe flag */
	__u32	FrameId;		/* +16 (u32FrameId) */
	__u32	TimeStap;		/* +20 timestamp (u32TimeStap, sic) */
	__u32	Resolution;		/* +24 (u32Resolution) */
	__u32	TailMagicCode;		/* +28 = 0x87654321 (u32TailMagicCode) */
	__u32	CrcCode;		/* +32 CRC-32/poly 0xedb88320 over the first 32 B */
};

/*
 * video_packet_t (52 bytes). The vendor's streaming reassembly accumulator,
 * NOT an on-wire layout: header_len/packet_len/tail_len are running byte
 * counters while the byte stream is walked to rebuild header[36] then the
 * trailing 4-byte tail.
 */
struct video_packet {			/* 52 bytes */
	__u32	header_len;		/* +0  header bytes accumulated so far */
	__u32	packet_len;		/* +4  payload bytes accumulated so far */
	__u32	tail_len;		/* +8  tail bytes accumulated so far */
	struct video_packet_header header; /* +12 (36 B, ends @48) */
	__u32	tail_TailMagicCode;	/* +48 video_packet_tail_t = { u32TailMagicCode } */
};

/* -------------------------------------------------------------------------- */
/* Driver state                                                               */
/* -------------------------------------------------------------------------- */

/* Per-SDIO-function driver context (kzalloc of the fields we model). */
struct artosyn_sdio_device {
	struct sdio_func	*func;
	struct net_device	*ndev;
	struct miscdevice	misc;
	bool			misc_registered;

	struct mutex		io_lock;	/* serialises bulk IO (ctx "removed" gate) */
	bool			removed;

	/*
	 * fd-lifetime vs SDIO removal. refs = probe + one per open fd; the
	 * struct is freed on the last put, so a still-open /dev/artosyn_sdio
	 * fd never dereferences freed memory after the func is removed.
	 * active_ops counts fd ops currently inside the driver; remove()
	 * drains it to zero (ops_drained) before tearing down func/ndev.
	 */
	refcount_t		refs;
	atomic_t		active_ops;
	struct completion	ops_drained;

	wait_queue_head_t	cmd_wq;		/* "q" waitqueue, woken by IRQ events */

	/*
	 * Two-channel 8-bit bidirectional mailbox (artosyn_msg_args, ioctl nr 2).
	 * Chip raises 0x68 bit0/bit1; the IRQ reads regs 0x54/0x58 into
	 * mailbox_msg[0]/[1] and latches mailbox_valid[]. MSG_GET returns+clears
	 * them; MSG_POST writes the host message bytes to regs 0x44 (ch0) / 0x48
	 * (ch1).
	 */
	u8			mailbox_msg[2];
	bool			mailbox_valid[2];

	/*
	 * RX/TX flow control. 0x68 bit2 -> read reg 0x5c = number of 512-byte
	 * blocks the chip has queued for us; 0x68 bit3 -> read reg 0x60 = blocks
	 * of TX credit. Both are << 9 to get bytes.
	 */
	/*
	 * RX/TX byte accumulators (vendor RXR+12/+16 and +88/+92). The IRQ sets
	 * *_write_acc = (reg 0x5c / 0x60) << 9 only when the previous batch is
	 * consumed (read == write); recv (RX) and the TX path consume by setting
	 * read = write. len-to-drain = write_acc - read_acc.
	 */
	u32			rx_read_acc;
	u32			rx_write_acc;
	u32			tx_read_acc;
	u32			tx_write_acc;

	/* per-channel RX frame-id continuity, §E (video_channel_t array). */
	u32			chan_frame_id[ARSDIO_MAX_CHANNELS];
	bool			chan_seen[ARSDIO_MAX_CHANNELS];

	atomic_t		inflight;	/* TX in-flight count (cap 0x400) */

	/* DMA-able staging buffers (separately allocated, 512-aligned). */
	u8			*tx_buf;
	u8			*rx_buf;
	u8			*fw_buf;

	/* stats (surfaced by the vendor via artosyn_sdio_stats_str). */
	u64			total_tx_pkts;
	u64			total_tx_bytes;
	u64			total_rx_pkts;
	u64			total_rx_bytes;
};

/* Per-netdev state. */
struct artosyn_sdio_eth {
	__be32				ip_addr;
	struct artosyn_sdio_device	*sdio_dev;
	struct net_device		*net_dev;

	struct sk_buff_head		data_txq;	/* IP/bulk TX */
	struct sk_buff_head		cmd_txq;	/* command TX */
	struct sk_buff_head		cmd_rxq;	/* command RX */
	struct sk_buff_head		unacked_txq;	/* lightweight ACK layer */
	int				unacked_max;

	struct workqueue_struct		*workwq;
	struct work_struct		work;
	struct delayed_work		delay_work;

	bool				net_dev_opened;
	bool				send_null_cmd;

	/*
	 * RX reassembly carry-over. When a compressed IP frame spans more than
	 * one RX transfer, the half-built skb is parked here and continued on the
	 * next artosyn_sdio_recv() call.
	 */
	struct sk_buff			*partial_skb;
	u32				partial_skb_left;
	/*
	 * The 12-byte compressed header itself can straddle two RX transfers.
	 * The head bytes are parked here, NEVER discarded: dropping them loses
	 * stream sync and the 0x45 resync then fabricates frames out of HEVC
	 * payload bytes.
	 */
	u8				hdr_carry[IP_COMPRESSED_HDR];
	u32				hdr_carry_len;

	u32				unacked_bytes;	/* bytes pending an 0xEE ACK */
};

/* Module parameters (insmod fw_name=bb_demo_gnd_d.img cfg_name=bb_config_gnd.json). */
static char *fw_name = "bb_demo_gnd_d.img";
module_param(fw_name, charp, 0444);
MODULE_PARM_DESC(fw_name, "AR8030 baseband firmware image (SPL blob)");

static char *cfg_name = "bb_config_gnd.json";
module_param(cfg_name, charp, 0444);
MODULE_PARM_DESC(cfg_name, "AR8030 merged config JSON blob");

/* forward decls */
static void artosyn_sdio_recv(struct artosyn_sdio_device *dev);
static void artosyn_sdio_send_data(struct artosyn_sdio_device *dev);
static void artosyn_sdio_send_cmd(struct artosyn_sdio_device *dev);

/* -------------------------------------------------------------------------- */
/* CMD52 register helpers (host assumed claimed unless noted)                 */
/* -------------------------------------------------------------------------- */

static inline u8 ar_readb(struct sdio_func *func, unsigned int addr)
{
	int err = 0;
	u8 v = sdio_readb(func, addr, &err);

	if (err)
		pr_debug("readb 0x%02x failed: %d\n", addr, err);
	return v;
}

static inline void ar_writeb(struct sdio_func *func, u8 val, unsigned int addr)
{
	int err = 0;

	sdio_writeb(func, val, addr, &err);
	if (err)
		pr_debug("writeb 0x%02x=0x%02x failed: %d\n", addr, val, err);
}

/* -------------------------------------------------------------------------- */
/* Firmware uploader (devid 0x8030), §B                                       */
/* -------------------------------------------------------------------------- */

/*
 * Push a firmware segment to the AR8030 ROM loader as a run of "SD" frames.
 * Frame = { u16 magic=0x4453, u16 len, u64 dst_addr, payload }, all written to
 * SDIO func-1 address 0 (the real dest is in the header, NOT the CMD53 addr).
 * Host claimed by the caller.
 *
 * Rule: each frame is emitted at its EXACT size, never padded up to a 512-byte
 * block; <512 must go out byte-mode. sdio_memcpy_toio uses block mode iff the
 * exact size is a 512 multiple, else byte mode.
 *
 * Frame sizing:
 *   - remaining >= 0xff4        : 4096-B block frame (12 + 4084), len-field=4084
 *   - remaining >= 500          : 512-B  block frame (12 + 500),  len-field=500
 *   - tiny whole segment (off==0, remaining < 500, e.g. the 64-B SPL header):
 *                                 BYTE-mode 12 + chunk + 12 trailing zeros,
 *                                 len-field = chunk + 12
 *   - tail of a multi-frame seg : BYTE-mode 12 + remaining, len-field=remaining
 *   - finalize (len==0)         : bare 12-byte header (len-field=0) = jump/boot frame
 */
static int sdio_rom_send(struct artosyn_sdio_device *dev, u64 dst_addr,
			 const u8 *payload, u32 len)
{
	struct sdio_func *func = dev->func;
	u8 *buf = dev->fw_buf;
	u32 off = 0;
	int ret;

	do {
		u32 remaining = len - off;
		u32 chunk, len_field, wire;

		if (len == 0) {					/* finalize / jump frame */
			chunk = 0;
			len_field = 0;
			wire = SD_FRAME_HDR_LEN;
		} else if (off == 0 && remaining < SD_FRAME_BLK_PAYLOAD) {
			chunk = remaining;			/* tiny whole segment: +12 zero tail */
			len_field = chunk + SD_FRAME_HDR_LEN;
			wire = SD_FRAME_HDR_LEN + chunk + SD_FRAME_HDR_LEN;
		} else if (remaining >= SD_FRAME_PAYLOAD_MAX) {
			chunk = SD_FRAME_PAYLOAD_MAX;		/* 4084 -> 4096 block frame */
			len_field = chunk;
			wire = SD_FRAME_HDR_LEN + chunk;
		} else if (remaining >= SD_FRAME_BLK_PAYLOAD) {
			chunk = SD_FRAME_BLK_PAYLOAD;		/* 500 -> 512 block frame */
			len_field = chunk;
			wire = SD_FRAME_HDR_LEN + chunk;
		} else {
			chunk = remaining;			/* tail -> byte mode */
			len_field = chunk;
			wire = SD_FRAME_HDR_LEN + chunk;
		}

		memset(buf, 0, wire);
		put_unaligned_le16(SD_FRAME_MAGIC, buf + 0);
		put_unaligned_le16((u16)len_field, buf + 2);
		put_unaligned_le64(dst_addr + off, buf + 4);
		if (chunk)
			memcpy(buf + SD_FRAME_HDR_LEN, payload + off, chunk);

		/* EXACT wire size: <512 -> byte mode (never a padded 512 block). */
		ret = sdio_memcpy_toio(func, ARSDIO_FIFO_ADDR, buf, wire);
		if (ret) {
			pr_err("uploading frame at off %u (wire %u) error: %d\n", off, wire, ret);
			return ret;
		}
		off += chunk;
	} while (off < len);

	return 0;
}

static int artosyn_upload_segment(struct artosyn_sdio_device *dev, u64 addr,
				  const u8 *data, u32 len, const char *what)
{
	int ret;

	if (!len)
		return 0;
	ret = sdio_rom_send(dev, addr, data, len);
	if (ret)
		pr_err("segment %s upload failed: %d\n", what, ret);
	return ret;
}

/*
 * Validate the _spl_header and stream the firmware (and config) segments into
 * the AR8030 ROM loader.  Host claimed by the caller.
 */
static int sdio_request_fw(struct artosyn_sdio_device *dev,
			   const struct firmware *fw,
			   const struct firmware *cfg)
{
	struct sdio_func *func = dev->func;
	const struct spl_header *hdr;
	const u8 *base;
	u8 cccr;
	u32 off;
	int err = 0;
	int ret;

	if (!fw || !fw->data || fw->size < sizeof(*hdr)) {
		pr_err("firmware is null / too small\n");
		return -EINVAL;
	}

	/* Readiness gate: CCCR/func-0 reg 3, bit 1 set => ROM loader ready. */
	cccr = sdio_f0_readb(func, CCCR_READY_REG, &err);
	if (err || !(cccr & CCCR_READY_BIT)) {
		pr_err("ROM loader not ready (cccr[3]=0x%02x err=%d)\n", cccr, err);
		return -ENODEV;
	}

	hdr = (const struct spl_header *)fw->data;
	base = fw->data;

	pr_info("firmware %s: magic=0x%08x ver=0x%08x spl=%u troot=%u sig=%u\n",
		fw_name, hdr->magic, hdr->img_version,
		hdr->spl_len, hdr->troot_len, hdr->sig_len);

	/*
	 * Segments are laid out sequentially in the image, each 512-aligned: the
	 * file offset is forced to 512 after the header and rounded up after every
	 * segment. `off` tracks the running file offset.
	 */
	off = 0;

	/* (1) 64-byte header to ROM addr 0x2f0040. */
	ret = sdio_rom_send(dev, SPL_ROM_HEADER_ADDR, base, sizeof(*hdr));
	if (ret)
		return ret;
	off = ARSDIO_BLOCK;			/* header occupies the first block */

	/* (2) SPL segment. */
	ret = artosyn_upload_segment(dev, hdr->spl_load_addr,
				     base + off, hdr->spl_len, "spl");
	if (ret)
		return ret;
	off += ALIGN(hdr->spl_len, ARSDIO_BLOCK);

	/* (3) troot segment. */
	ret = artosyn_upload_segment(dev, hdr->troot_load_addr,
				     base + off, hdr->troot_len, "troot");
	if (ret)
		return ret;
	off += ALIGN(hdr->troot_len, ARSDIO_BLOCK);

	/* (4) signature segment. */
	ret = artosyn_upload_segment(dev, hdr->sig_load_addr,
				     base + off, hdr->sig_len, "sig");
	if (ret)
		return ret;

	/*
	 * (5) config blob (bb_config_gnd.json): streamed verbatim to
	 * troot_load_addr (the kernel does not parse the JSON; the chip firmware
	 * consumes it).
	 */
	if (cfg && cfg->data && cfg->size) {
		pr_info("uploading cfg %s: %u bytes -> 0x%08x\n",
			cfg_name, (u32)cfg->size, hdr->troot_load_addr);
		ret = artosyn_upload_segment(dev, hdr->troot_load_addr,
					     cfg->data, cfg->size, "cfg");
		if (ret)
			return ret;
	} else {
		pr_warn("NO config uploaded (cfg=%p) - chip runs with DEFAULT config, will not associate!\n",
			cfg);
	}

	/* (6) addr=0,len=0 finalize/jump. */
	ret = sdio_rom_send(dev, 0, NULL, 0);
	if (ret)
		pr_err("firmware finalize failed: %d\n", ret);

	return ret;
}

/* -------------------------------------------------------------------------- */
/* IPv4 header (de)compression, §D/§2                                         */
/* -------------------------------------------------------------------------- */

/*
 * TX compression: an IPv4 header (IHL=5, first byte 0x45) is shrunk from 20 to
 * 12 bytes in place, then the first 8 bytes are skb_pull()ed away. The bytes
 * that must survive the pull are relocated in this exact order, then 8 pulled:
 *
 *   d[18]=d[15]; d[17]=d[7]; d[16]=d[6]; d[15]=d[5];
 *   d[14]=d[4]; d[13]=d[3]; d[12]=d[2]; d[8]=d[0];
 *
 * After skb_pull(skb, 8) the 12-byte compressed header is:
 *   [0]=0x45 [1]=proto [2..3]=checksum [4..5]=total_len [6..7]=id
 *   [8..9]=flags/frag [10]=src-IP last octet [11]=dst-IP last octet
 * (the 10.0.0 network prefix is implicit and rebuilt on RX).
 */
static void artosyn_sdio_build_ip_head(struct sk_buff *skb)
{
	u8 *d = skb->data;

	d[18] = d[15];
	d[17] = d[7];
	d[16] = d[6];
	d[15] = d[5];
	d[14] = d[4];
	d[13] = d[3];
	d[12] = d[2];
	d[8]  = d[0];
	skb_pull(skb, 8);
}

/*
 * RX decompression. skb->data points at the 12-byte compressed header;
 * skb_push(8) re-opens the 20-byte IPv4 header, the carried fields are moved
 * back, the 10.0.0 prefix octets come from eth->ip_addr, and the header
 * checksum is recomputed. `pfx` is the first three octets of the link IP.
 */
static void artosyn_sdio_restore_ip_head(const u8 pfx[3], struct sk_buff *skb)
{
	struct iphdr *iph;
	u8 *d;

	skb_push(skb, 8);
	d = skb->data;

	d[0] = 0x45;		/* version 4, IHL 5 */
	d[1] = 0;		/* DSCP/ECN */
	d[2] = d[12];		/* total length hi */
	d[3] = d[13];		/* total length lo */
	d[4] = d[14];		/* identification hi */
	d[5] = d[15];		/* identification lo */
	d[6] = d[16];		/* flags/frag hi */
	d[7] = d[17];		/* flags/frag lo */
	d[8] = 0x40;		/* TTL = 64 */
	/* d[9] (protocol) survives untouched from the compressed header */
	d[10] = 0;		/* checksum (recomputed below) */
	d[11] = 0;
	d[12] = pfx[0];		/* source IP = pfx.X */
	d[13] = pfx[1];
	d[14] = pfx[2];
	d[15] = d[18];		/* source IP last octet (carried) */
	d[16] = pfx[0];		/* dest IP = pfx.Y */
	d[17] = pfx[1];
	d[18] = pfx[2];
	/* d[19] (dest IP last octet) survives untouched */

	iph = (struct iphdr *)d;
	iph->check = 0;
	iph->check = ip_fast_csum((const void *)iph, iph->ihl);
}

/* -------------------------------------------------------------------------- */
/* Video-vs-IP classification, §2/§E                                          */
/* -------------------------------------------------------------------------- */

static bool artosyn_crc_ok(const struct video_packet_header *h)
{
	u32 crc = crc32_le(0xffffffffu, (const u8 *)h, VPH_CRC_LEN) ^ 0xffffffffu;

	return crc == h->CrcCode;
}

static bool artosyn_is_video_port(u8 port)
{
	return port == PORT_VIDEO_A || port == PORT_VIDEO_B;
}

/*
 * Returns true if the buffer holds a valid video packet (magic+tail+crc good).
 * The port/type byte selects video (0x3/0x23) vs other (0x4/0x24).
 */
static bool artosyn_sdio_check_video_packet(const u8 *buf, u32 len, u8 port)
{
	const struct video_packet_header *h = (const void *)buf;

	if (len < sizeof(*h))
		return false;
	if (h->MagicCode != VPH_MAGIC || h->TailMagicCode != VPH_TAIL_MAGIC)
		return false;
	if (!artosyn_crc_ok(h))
		return false;
	return artosyn_is_video_port(port);
}

static bool artosyn_sdio_check_video_skb(const struct sk_buff *skb, u8 port)
{
	return artosyn_sdio_check_video_packet(skb->data, skb->len, port);
}

/* -------------------------------------------------------------------------- */
/* RX path, §D/§E                                                             */
/* -------------------------------------------------------------------------- */

/* Push a fully reassembled compressed-IP frame up the stack. */
static void artosyn_rx_finish_ip(struct artosyn_sdio_device *dev,
				 const u8 pfx[3], struct sk_buff *skb)
{
	artosyn_sdio_restore_ip_head(pfx, skb);
	skb->protocol = htons(ETH_P_IP);
	skb_reset_mac_header(skb);
	skb_reset_network_header(skb);
	dev->total_rx_pkts++;
	dev->total_rx_bytes += skb->len;
	dev->ndev->stats.rx_packets++;		/* so /sys .../rx_bytes reflects real flow */
	dev->ndev->stats.rx_bytes += skb->len;
	netif_rx(skb);	/* netif_rx_ni() removed in 5.18; netif_rx is BH-safe now */
}

/*
 * The largest legitimate tunnel frame: the link MTU is 4096 (sdio0 on both
 * ends) plus 256 B headroom. A larger "total length" is payload bytes misread
 * as a header after a lost sync; the headroom keeps a slightly odd vendor
 * header accounting from being rejected at the exact MTU edge.
 */
#define ARSDIO_IP_FRAME_MAX	4352

/*
 * A frame-start candidate must look like a real tunnel frame: IPv4/IHL5
 * marker, a protocol the link actually carries, and a sane total length.
 * Compressed layout: [0]=0x45 [1]=proto [4..5]=total_len (BE).
 */
static bool artosyn_ip_hdr_plausible(const u8 *p, u32 totlen)
{
	if (p[0] != 0x45)
		return false;
	if (p[1] != IPPROTO_UDP && p[1] != IPPROTO_TCP &&
	    p[1] != IPPROTO_ICMP && p[1] != IPPROTO_IGMP)
		return false;
	return totlen > sizeof(struct iphdr) && totlen <= ARSDIO_IP_FRAME_MAX;
}

/*
 * Walk a 0xCC (IP) transfer payload and emit each compressed IPv4 frame, with
 * carry-over for frames that span more than one RX transfer: take the total
 * length from compressed bytes [4..5] and split header(12) + payload. Frames
 * are back-to-back, so in a healthy stream every frame starts exactly where
 * the previous one ended; the 0x45 resync scan is a recovery path only, and
 * every candidate must pass artosyn_ip_hdr_plausible() before it is trusted.
 */
static void artosyn_rx_ip_stream(struct artosyn_sdio_device *dev,
				 const u8 *buf, u32 len)
{
	struct net_device *ndev = dev->ndev;
	struct artosyn_sdio_eth *eth = netdev_priv(ndev);
	u8 pfx[3];
	u32 off = 0;
	u32 junk = 0;

	memcpy(pfx, &eth->ip_addr, sizeof(pfx));	/* 10.0.0 network prefix */

	/* Finish a header parked from the previous transfer (split < 12 B). */
	if (eth->hdr_carry_len) {
		u32 take = min((u32)IP_COMPRESSED_HDR - eth->hdr_carry_len, len);
		u32 totlen, need;
		struct sk_buff *skb;

		memcpy(eth->hdr_carry + eth->hdr_carry_len, buf, take);
		eth->hdr_carry_len += take;
		if (eth->hdr_carry_len < IP_COMPRESSED_HDR)
			return;				/* still incomplete */
		off += take;
		eth->hdr_carry_len = 0;
		totlen = ((u32)eth->hdr_carry[4] << 8) | eth->hdr_carry[5];
		if (artosyn_ip_hdr_plausible(eth->hdr_carry, totlen)) {
			need = IP_COMPRESSED_HDR + (totlen - sizeof(struct iphdr));
			skb = netdev_alloc_skb(ndev, need);
			if (skb) {
				skb_put_data(skb, eth->hdr_carry, IP_COMPRESSED_HDR);
				/* body continues below via the partial path */
				eth->partial_skb = skb;
				eth->partial_skb_left = need - IP_COMPRESSED_HDR;
			}
		} else {
			junk += IP_COMPRESSED_HDR;	/* carried bytes were not a header */
		}
	}

	/* Finish a frame parked from the previous transfer. */
	if (eth->partial_skb) {
		u32 take = min(eth->partial_skb_left, len - off);

		skb_put_data(eth->partial_skb, buf + off, take);
		eth->partial_skb_left -= take;
		off += take;
		if (eth->partial_skb_left)
			return;				/* still incomplete */
		artosyn_rx_finish_ip(dev, pfx, eth->partial_skb);
		eth->partial_skb = NULL;
	}

	while (off < len) {
		const u8 *p = buf + off;
		u32 avail = len - off;
		u32 totlen, need;
		struct sk_buff *skb;

		if (p[0] != 0x45) {		/* resync to the next IPv4 marker */
			off++;
			junk++;
			continue;
		}
		if (avail < IP_COMPRESSED_HDR) {
			/* Header split across transfers: PARK it (discarding these
			 * bytes desyncs the stream - see hdr_carry above).
			 */
			memcpy(eth->hdr_carry, p, avail);
			eth->hdr_carry_len = avail;
			return;
		}

		totlen = ((u32)p[4] << 8) | p[5];	/* IPv4 total length */
		if (!artosyn_ip_hdr_plausible(p, totlen)) {
			off++;			/* payload byte mimicking a marker */
			junk++;
			continue;
		}
		/* on-wire size = 12-byte compressed header + (totlen-20) payload */
		need = IP_COMPRESSED_HDR + (totlen - sizeof(struct iphdr));

		skb = netdev_alloc_skb(ndev, need);
		if (!skb)
			break;

		if (need > avail) {
			/* frame continues in the next transfer; park it */
			skb_put_data(skb, p, avail);
			eth->partial_skb = skb;
			eth->partial_skb_left = need - avail;
			return;
		}

		skb_put_data(skb, p, need);
		artosyn_rx_finish_ip(dev, pfx, skb);
		off += need;
	}

	if (junk) {
		/* One counter tick per transfer with skipped bytes; visible in
		 * `ip -s link` as frame errors so a desync is never silent.
		 */
		ndev->stats.rx_frame_errors++;
		net_dbg_ratelimited("rx: skipped %u junk byte(s) in IP stream\n", junk);
	}
}

/*
 * Diagnostic: hex-log frames crossing /dev/artosyn_sdio and the RX trailer
 * types. Enable with `frame_log=1` (runtime-writable module param).
 */
static bool frame_log;
module_param(frame_log, bool, 0644);
MODULE_PARM_DESC(frame_log, "hex-log /dev/artosyn_sdio frames + RX trailer types (diagnostic)");

/* Diagnostic: log the TX credit path (EVT_TX grants accept/ignore, no-credit stalls,
 * insufficient-credit requeues) WITHOUT the frame_log hex spam. Ratelimited.
 */
static bool credit_log;
module_param(credit_log, bool, 0644);
MODULE_PARM_DESC(credit_log, "log TX credit grant/consume/stall decisions (diagnostic)");

/* TX in-flight window in bytes = eth->unacked_max. 0xEE ACKs gate only the
 * goggle->air uplink; 0 disables the gate; vendor value 40960 (0xa000). See
 * artosyn_sdio_send_data() for the gate itself.
 *
 * Default 40960 to MATCH THE VENDOR: with the window off (0) a sustained goggle->air
 * push (e.g. an SSH file copy) races ahead of what the air actually drains, over-runs a
 * chip-side buffer the per-grant TX credit does not fully account for, and wedges the
 * baseband association (RX goes to 0, "slot 0 disconnect"). The window backpressures the
 * uplink to the ~air-drain rate; the empty-0xDD keepalive in send_cmd() keeps credit/0xEE
 * ACKs cycling so a throttled window drains and reopens (vendor pairs the two).
 */
static unsigned int tx_window = 40960;
module_param(tx_window, uint, 0644);
MODULE_PARM_DESC(tx_window, "TX in-flight window bytes (unacked_max); vendor 40960; 0=disabled");

static void ml_framelog(const char *tag, const u8 *d, size_t n)
{
	if (!frame_log)
		return;
	pr_info("ml-frame %s len=%zu\n", tag, n);
	print_hex_dump(KERN_INFO, "ml-frame  ", DUMP_PREFIX_OFFSET, 32, 1,
		       d, min(n, (size_t)256), false);
}

/*
 * RX bottom half (host claimed by the IRQ). The chip signals via 0x68 bit2 and
 * the IRQ stashes (reg 0x5c << 9) bytes in rx_write_acc (len = write-read). We DMA that many
 * bytes out of FIFO 0 in one CMD53 burst, then read the 4-byte link trailer
 * { 0x00, type, len_lo, len_hi } from the tail of the final 512-byte block and
 * dispatch by type: 0xCC -> IP tunnel, 0xDD -> command response, 0xEE -> ACK.
 */
static void artosyn_sdio_recv(struct artosyn_sdio_device *dev)
{
	struct sdio_func *func = dev->func;
	struct artosyn_sdio_eth *eth;
	struct sk_buff *skb;
	u8 *buf = dev->rx_buf;
	u32 len;
	int ret;

	if (!dev->ndev)
		return;
	eth = netdev_priv(dev->ndev);

	/*
	 * Drain the chip FIFO UNCONDITIONALLY. The firmware withholds TX credit until
	 * the host consumes the frames it sends, so we must drain even before sdio0 is
	 * brought up - otherwise the chip retransmits forever and never grants credit.
	 * The vendor recv @0x1f80 has no net_dev_opened gate here; only the IP-tunnel
	 * (0xCC) dispatch below is gated on it.
	 */
	len = dev->rx_write_acc - dev->rx_read_acc;
	dev->rx_read_acc = dev->rx_write_acc;	/* consume the batch (vendor @0x20ac) */
	if (!len)
		return;
	if (len > ARSDIO_RX_BUF)
		len = ARSDIO_RX_BUF;		/* safety only; a 255-block (130560 B) burst fits */
	if (len < LINK_TRAILER_LEN)
		return;

	ret = sdio_memcpy_fromio(func, buf, ARSDIO_FIFO_ADDR, len);
	if (ret) {
		pr_debug("recv fromio failed: %d\n", ret);
		return;
	}

	/*
	 * Each block run ends in a { 0x00, type, len_lo, len_hi } trailer in the
	 * last 4 bytes of its final 512-byte block - but ONE read can cover
	 * SEVERAL runs when the chip posts more blocks while earlier ones are
	 * still undrained. Parsing a merged read as a single run truncates and
	 * mixes IP frames, so recover the
	 * run boundaries by walking the trailers back from the end (each run
	 * occupies ALIGN(payload + 4, 512) bytes). Any inconsistency falls back
	 * to the legacy whole-read-is-one-run interpretation.
	 */
	{
		struct { u32 start; u32 size; u32 payload; u8 type; } runs[ARSDIO_MAX_RUNS];
		int nruns = 0, i;
		u32 pos = len;

		while (pos >= ARSDIO_BLOCK && nruns < ARSDIO_MAX_RUNS) {
			u8 t = buf[pos - 3];
			u32 pl = (u32)buf[pos - 2] | ((u32)buf[pos - 1] << 8);
			u32 rsize = ALIGN(pl + LINK_TRAILER_LEN, ARSDIO_BLOCK);

			if (buf[pos - 4] != 0x00 || rsize > pos ||
			    (t != LINK_TYPE_IP && t != LINK_TYPE_CMD && t != LINK_TYPE_ACK)) {
				break;
			}
			runs[nruns].start = pos - rsize;
			runs[nruns].size = rsize;
			runs[nruns].payload = pl;
			runs[nruns].type = t;
			nruns++;
			pos -= rsize;
		}
		if (pos != 0 || nruns == 0) {
			/* Unparseable (or an unknown trailer type): legacy single run. */
			nruns = 1;
			runs[0].start = 0;
			runs[0].size = len;
			runs[0].type = buf[len - 3];
			runs[0].payload = (u32)buf[len - 2] | ((u32)buf[len - 1] << 8);
			if (runs[0].payload > len - LINK_TRAILER_LEN)
				runs[0].payload = len - LINK_TRAILER_LEN;
		}

		/* runs[] filled back-to-front; dispatch in stream order. */
		for (i = nruns - 1; i >= 0; i--) {
			const u8 *rbuf = buf + runs[i].start;
			u32 rend = runs[i].start + runs[i].size;
			u32 payload = runs[i].payload;
			u8 type = runs[i].type;

			/* Diagnostic: log every run's trailer type + size. */
			if (frame_log) {
				pr_info("ml-rx type=0x%02x len=%u payload=%u run=%d/%d\n",
					type, runs[i].size, payload, nruns - i, nruns);
				if (payload >= 512) {		/* video-sized frame - dump the head */
					ml_framelog("RX-BIG", rbuf, payload);
				}
			}

			switch (type) {
			case LINK_TYPE_IP: {
				if (eth->net_dev_opened) {	/* IP tunnel only once sdio0 is up */
					artosyn_rx_ip_stream(dev, rbuf, payload);
				}
			}
			break;
			case LINK_TYPE_CMD: {
				skb = netdev_alloc_skb(dev->ndev, payload);
				if (skb) {
					skb_put_data(skb, rbuf, payload);
					skb_queue_tail(&eth->cmd_rxq, skb);
					wake_up(&dev->cmd_wq);	/* unblock a pending bb_ioctl */
				}
			}
			break;
			case LINK_TYPE_ACK: {
				if (runs[i].size >= 8) {
					u32 acked = ((u32)buf[rend - 8] << 24) |
						    ((u32)buf[rend - 7] << 16) |
						    ((u32)buf[rend - 6] << 8)  | buf[rend - 5];

					u32 before = eth->unacked_bytes;

					/* `acked` is the chip's CURRENT still-pending byte count (vendor
					 * @0x26e4), NOT a delta. The vendor frees (unacked_bytes - acked)
					 * worth of the oldest in-flight bytes, leaving ~acked pending; if we
					 * already believe fewer are pending than the chip reports, it leaves
					 * the count unchanged. So converge our window to the chip's view: when
					 * the chip drains (acked -> ~0) the window fully reopens.
					 */
					if (eth->unacked_bytes > acked)
						eth->unacked_bytes = acked;
					if (frame_log)
						pr_info_ratelimited("ml-tx: 0xEE ack=%u  unacked %u -> %u\n",
								    acked, before, eth->unacked_bytes);
				}
			}
			break;
			default: {
				/* Log dropped unrecognized trailer types (ratelimited). */
				pr_info_ratelimited("rx: UNKNOWN trailer type 0x%02x len=%u payload=%u (dropped)\n",
						    type, runs[i].size, payload);
			}
			break;
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
/* TX paths, §D                                                               */
/* -------------------------------------------------------------------------- */

/*
 * Append the 4-byte link trailer { 0x00, type, len_lo, len_hi } to the last
 * four bytes of the ALIGN(used+4, 512) block run staged in tx_buf, then CMD53
 * it to FIFO 0. `used` is the total length of all packets packed ahead of the
 * trailer.
 */
static int artosyn_tx_flush(struct artosyn_sdio_device *dev, u32 used, u8 type)
{
	u8 *buf = dev->tx_buf;
	u32 blk = ALIGN(used + LINK_TRAILER_LEN, ARSDIO_BLOCK);

	buf[blk - 4] = 0x00;
	buf[blk - 3] = type;
	buf[blk - 2] = (u8)(used & 0xff);
	buf[blk - 1] = (u8)((used >> 8) & 0xff);
	return sdio_memcpy_toio(dev->func, ARSDIO_FIFO_ADDR, buf, blk);
}

/*
 * TX bulk: drain data_txq, compress each IPv4 frame in place (build_ip_head +
 * skb_pull 8), pack the compressed frames back-to-back into tx_buf, then flush
 * the whole batch with one 0xCC trailer. Host claimed by the caller.
 */
static void artosyn_sdio_send_data(struct artosyn_sdio_device *dev)
{
	struct artosyn_sdio_eth *eth;
	struct sk_buff *skb;
	u8 *buf = dev->tx_buf;
	u32 used = 0;

	if (!dev->ndev)
		return;
	eth = netdev_priv(dev->ndev);

	/* TX credit gate (vendor @0x2ae0): nothing leaves until the chip grants credit
	 * on 0x60 (EVT_TX -> tx_write_acc); line ~944 packs only within that credit and
	 * line ~975 consumes it.
	 */
	if (dev->tx_write_acc == dev->tx_read_acc) {
		if (credit_log && !skb_queue_empty(&eth->data_txq))
			pr_info_ratelimited("ml-credit: send_data NO-CREDIT, %u pkts stuck (rd=%u wr=%u)\n",
					    skb_queue_len(&eth->data_txq),
					    dev->tx_read_acc, dev->tx_write_acc);
		return;
	}

	/* In-flight cap (vendor @0x2b10, `b.hi`, threshold 40960): do not START a new send batch
	 * while more than unacked_max bytes are still pending an 0xEE ACK; drained by the
	 * LINK_TYPE_ACK handler above. 0xEE ACKs gate only the goggle->air uplink; unacked_max == 0
	 * (the default) disables the gate; vendor value 40960. (Vendor `tx_write_acc == 0x400`
	 * gate @0x2af8 is still undecoded, deliberately not reproduced.)
	 */
	if (eth->unacked_max && eth->unacked_bytes > eth->unacked_max) {
		if (credit_log && !skb_queue_empty(&eth->data_txq))
			pr_info_ratelimited("ml-credit: THROTTLED unacked=%u > max=%u (%u pkts waiting)\n",
					    eth->unacked_bytes, eth->unacked_max,
					    skb_queue_len(&eth->data_txq));
		return;
	}

	while ((skb = skb_dequeue(&eth->data_txq))) {
		u32 clen;

		/* IPv4-only (first byte 0x45). The classifier is invoked but its
		 * result does not reroute here; keep the call.
		 */
		(void)artosyn_sdio_check_video_skb(skb, skb->data[9]);
		if (skb->len < sizeof(struct iphdr) || skb->data[0] != 0x45) {
			net_dbg_ratelimited("tx: unsupported ip version %02x\n",
					    skb->data[0]);
			/* start_xmit inc'd inflight for this skb; every other consume path decs it.
			 * Missing the dec here leaks inflight upward on any IPv4-with-options /
			 * malformed packet until it pins at the cap and the TX queue stops forever. */
			atomic_dec_if_positive(&dev->inflight);
			kfree_skb(skb);
			continue;
		}

		/* Fit test against the current grant. The size that will ACTUALLY hit the wire is
		 * (skb->len - IP_HEAD_SHRINK) after build_ip_head() compresses the header below, plus
		 * the one per-batch trailer - i.e. skb->len - 4, NOT skb->len + 4. The vendor aligns
		 * exactly this (@0x2b90: `+ 0x1fb` = ALIGN(len + used - 4, 512)); checking skb->len + 4
		 * overstated a full-MTU (4096) packet as needing 4608 > 4096-byte grant and DROPPED every
		 * data packet while only tiny TCP ACKs got through (SSH copies stalled at ~0 bytes). */
		if (ALIGN(used + skb->len - IP_HEAD_SHRINK + LINK_TRAILER_LEN, ARSDIO_BLOCK)
		    > dev->tx_write_acc) {
			if (used) {
				/* Batch full for this grant (vendor @0x2ba4): flush what we
				 * packed and retry the rest on the next grant. The flush below
				 * consumes the credit (rd=wr), reopening the EVT_TX guard.
				 */
				skb_queue_head(&eth->data_txq, skb);
				break;
			}
			/* Head packet alone exceeds this grant (genuinely bigger than any grant can
			 * hold). The vendor (@0x2ba8) DROPS it and keeps draining; requeuing it strands
			 * tx_read_acc != tx_write_acc with an empty batch, so the (correct) EVT_TX guard
			 * then ignores every future grant -> permanent goggle->air TX deadlock. Drop on.
			 */
			if (credit_log)
				pr_info_ratelimited("ml-credit: DROP head pkt need=%u > credit=%u (would-wedge)\n",
						    (unsigned int)ALIGN(skb->len - IP_HEAD_SHRINK + LINK_TRAILER_LEN, ARSDIO_BLOCK),
						    dev->tx_write_acc);
			dev->ndev->stats.tx_dropped++;
			atomic_dec_if_positive(&dev->inflight);
			kfree_skb(skb);
			continue;
		}
		artosyn_sdio_build_ip_head(skb);	/* 20 -> 12 byte header */
		clen = skb->len;

		/* Flush the batch if this frame would overflow tx_buf. */
		if (used && used + clen + LINK_TRAILER_LEN > ARSDIO_STAGING) {
			if (artosyn_tx_flush(dev, used, LINK_TYPE_IP))
				pr_debug("send_data toio failed\n");
			used = 0;
		}
		if (clen + LINK_TRAILER_LEN > ARSDIO_STAGING) {
			/* Unreachable while MTU-sized frames fit ARSDIO_STAGING, but
			 * this consume path must dec inflight like every other one or
			 * a future STAGING/MTU change stalls TX forever (see the drop
			 * paths above).
			 */
			atomic_dec_if_positive(&dev->inflight);
			kfree_skb(skb);			/* single frame too big */
			continue;
		}

		memcpy(buf + used, skb->data, clen);
		used += clen;
		dev->total_tx_pkts++;
		dev->total_tx_bytes += clen;
		atomic_dec_if_positive(&dev->inflight);
		kfree_skb(skb);
	}

	if (used) {
		if (artosyn_tx_flush(dev, used, LINK_TYPE_IP))
			pr_debug("send_data toio failed\n");
		dev->tx_read_acc = dev->tx_write_acc;	/* consume credit (vendor @0x2dd4) */
		eth->unacked_bytes += used;		/* flow-control window */
		if (frame_log)
			pr_info_ratelimited("ml-tx: flushed IP used=%u -> unacked=%u\n",
					    used, eth->unacked_bytes);
	}

	if (atomic_read(&dev->inflight) < ARSDIO_INFLIGHT_CAP)
		netif_tx_wake_queue(netdev_get_tx_queue(dev->ndev, 0));
}

/*
 * TX cmd: pack cmd_txq skbs back-to-back into tx_buf, flush with one 0xDD
 * trailer, then wake any bb_ioctl waiter. Host claimed by the caller.
 */
static void artosyn_sdio_send_cmd(struct artosyn_sdio_device *dev)
{
	struct artosyn_sdio_eth *eth;
	struct sk_buff *skb;
	u8 *buf = dev->tx_buf;
	u32 used = 0;

	if (!dev->ndev)
		return;
	eth = netdev_priv(dev->ndev);

	/* Vendor TX credit gate (@0xa68): nothing leaves until the chip has granted
	 * credit (the IRQ records it from 0x68 bit3 -> 0x60<<9 into tx_write_acc).
	 */
	if (dev->tx_write_acc == dev->tx_read_acc)
		return;

	while ((skb = skb_dequeue(&eth->cmd_txq))) {
		u32 len = skb->len;

		/* Pack only while the rounded running total still fits the credit. */
		if (ALIGN(used + len + LINK_TRAILER_LEN, ARSDIO_BLOCK) > dev->tx_write_acc) {
			if (used) {
				skb_queue_head(&eth->cmd_txq, skb);
				break;
			}
			/* Head cmd alone exceeds the grant: drop like the vendor (@0xaf0),
			 * never requeue an unfittable head (see send_data for why).
			 */
			if (credit_log)
				pr_info_ratelimited("ml-credit: DROP head cmd need=%u > credit=%u (would-wedge)\n",
						    (unsigned int)ALIGN(len + LINK_TRAILER_LEN, ARSDIO_BLOCK),
						    dev->tx_write_acc);
			kfree_skb(skb);

			/* The drop may have emptied the queue with used == 0, in which
			 * case neither wake below fires and a writer blocked on the
			 * queue-empty wait in artosyn_do_write would miss its wakeup.
			 */
			wake_up(&dev->cmd_wq);
			continue;
		}
		if (used && used + len + LINK_TRAILER_LEN > ARSDIO_STAGING) {
			if (artosyn_tx_flush(dev, used, LINK_TYPE_CMD))
				pr_debug("send_cmd toio failed\n");
			used = 0;
		}
		if (len + LINK_TRAILER_LEN > ARSDIO_STAGING) {
			kfree_skb(skb);
			wake_up(&dev->cmd_wq);	/* same missed-wakeup case as the drop above */
			continue;
		}
		memcpy(buf + used, skb->data, len);
		used += len;
		kfree_skb(skb);
	}

	if (used) {
		if (artosyn_tx_flush(dev, used, LINK_TYPE_CMD))
			pr_debug("send_cmd toio failed\n");
		dev->tx_read_acc = dev->tx_write_acc;	/* consume the credit (vendor @0xc8c) */
		wake_up(&dev->cmd_wq);
	} else if (!skb_queue_empty(&eth->data_txq)
		   && dev->tx_write_acc != dev->tx_read_acc) {
		/* Empty-0xDD keepalive (vendor send_cmd @0xb00): nothing to send on cmd, but IP
		 * data is queued and throttled (send_data bailed on the unacked window, so it did
		 * NOT consume this grant). Emit a bare 0xDD/len-0 frame and consume the grant to
		 * keep the chip cycling credit and 0xEE ACKs, so unacked_bytes drains and the
		 * window reopens - otherwise a throttled uplink can stall until the next grant.
		 */
		if (artosyn_tx_flush(dev, 0, LINK_TYPE_CMD))
			pr_debug("send_cmd keepalive toio failed\n");
		dev->tx_read_acc = dev->tx_write_acc;	/* consume the credit (vendor @0xd14) */
		if (credit_log)
			pr_info_ratelimited("ml-credit: 0xDD keepalive (data throttled, unacked=%u)\n",
					    eth->unacked_bytes);
	}
}

/* -------------------------------------------------------------------------- */
/* Workqueue (TX off the IRQ/xmit hot path)                                   */
/* -------------------------------------------------------------------------- */

static void artosyn_sdio_eth_worker(struct work_struct *w)
{
	struct artosyn_sdio_eth *eth = container_of(w, struct artosyn_sdio_eth, work);
	struct artosyn_sdio_device *dev = eth->sdio_dev;

	sdio_claim_host(dev->func);
	/* DATA before CMD (matches the vendor eth_worker @0x3718 and our own IRQ handler).
	 * The TX credit granted per EVT_TX is shared and each send_*() consumes the whole
	 * grant; CMD-first would let a busy cmd_txq starve the IP data path of credit.
	 */
	artosyn_sdio_send_data(dev);
	artosyn_sdio_send_cmd(dev);
	sdio_release_host(dev->func);
}

static void artosyn_sdio_eth_delay_worker(struct work_struct *w)
{
	struct artosyn_sdio_eth *eth =
		container_of(to_delayed_work(w), struct artosyn_sdio_eth, delay_work);

	if (eth->net_dev_opened)
		queue_work(eth->workwq, &eth->work);
}

/* -------------------------------------------------------------------------- */
/* IRQ handler, §D                                                            */
/* -------------------------------------------------------------------------- */

/*
 * SDIO IRQ (host already claimed). Verbatim reimplementation of the vendor
 * artosyn_sdio_irqhandler @0x2e08:
 *
 *   while ((val13 = sdio_readb(0x13)) != 0) {   // spin until 0x13 reads zero
 *       if (!(val13 & BIT4)) continue;          // bit4 only gates reading 0x68
 *       bitmap = sdio_readb(0x68);
 *       if bit0: msg[0]=sdio_readb(0x54); valid[0]=1; wake(q)   // the read = clear
 *       if bit1: msg[1]=sdio_readb(0x58); valid[1]=1; wake(q)
 *       if bit2: cnt=sdio_readb(0x5c)<<9; if(!do_rx && r==w){rx_write=cnt; do_rx=1}
 *       if bit3: cnt=sdio_readb(0x60)<<9; if(!do_tx && r==w){tx_write=cnt; do_tx=1}
 *   }
 *   if (do_rx) recv();
 *   if (do_tx) { send_data(); send_cmd(); }
 *
 * CRITICAL: each data register (0x54/0x58/0x5c/0x60) is read UNCONDITIONALLY
 * whenever its 0x68 bit is set - that read IS the event ack/clear. The do_*
 * latch and the read==write guard gate only the in-memory count store, NEVER
 * the register read; skipping the read leaves the bit set and the interrupt
 * re-fires forever. 0x5c/0x60 are block counts (one byte, <<9 == *512). The
 * vendor loop is uncapped; the guard here is only a hard-hang backstop.
 */
static void artosyn_sdio_irqhandler(struct sdio_func *func)
{
	struct artosyn_sdio_device *dev = sdio_get_drvdata(func);
	bool do_rx = false, do_tx = false;
	int guard = 0;

	if (!dev)
		return;

	for (;;) {
		u8 val13 = ar_readb(func, REG_STATUS);
		u8 bitmap;

		if (val13 == 0)
			break;
		if (++guard > 1000) {
			pr_warn_ratelimited("ar-irq: 0x13 stuck=0x%02x, bailing\n", val13);
			break;
		}
		if (!(val13 & STATUS_BUSY_BIT))
			continue;	/* re-read 0x13 (vendor busy-spins until it clears) */
		bitmap = ar_readb(func, REG_EVT_STATUS);

		/* Diagnostic: log the raw 0x68 event bitmap (with frame_log on). */
		if (frame_log) {
			static u8 ml_seen_bits;

			/* summary line: each newly-seen bit, once. */
			if (bitmap & ~ml_seen_bits) {
				ml_seen_bits |= bitmap;
				pr_info("ml-irq: 0x68 bits seen so far = 0x%02x\n", ml_seen_bits);
			}
			/* Log any bit outside the four handled (0x0F), every time it
			 * fires (ratelimited).
			 */
			if (bitmap & ~0x0Fu) {
				pr_info_ratelimited("ml-irq: UNHANDLED 0x68=0x%02x - chip signaling on a path we DROP\n",
						    bitmap);
			}
		}

		if (bitmap & EVT_MBOX0) {
			dev->mailbox_msg[0] = ar_readb(func, REG_MBOX_RX0);
			dev->mailbox_valid[0] = true;
			wake_up(&dev->cmd_wq);
		}
		if (bitmap & EVT_MBOX1) {
			dev->mailbox_msg[1] = ar_readb(func, REG_MBOX_RX1);
			dev->mailbox_valid[1] = true;
			wake_up(&dev->cmd_wq);
		}
		if (bitmap & EVT_RX) {
			u32 cnt = (u32)ar_readb(func, REG_RX_COUNT) << 9;

			if (!do_rx && dev->rx_read_acc == dev->rx_write_acc) {
				dev->rx_read_acc = 0;
				dev->rx_write_acc = cnt;
				do_rx = true;
			}
		}
		if (bitmap & EVT_TX) {
			u32 cnt = (u32)ar_readb(func, REG_TX_CREDIT) << 9;

			if (!do_tx && dev->tx_read_acc == dev->tx_write_acc) {
				dev->tx_read_acc = 0;
				dev->tx_write_acc = cnt;
				do_tx = true;
			} else if (credit_log) {
				/* Credit granted but not taken. If rd!=wr persists across IRQs
				 * (prev grant never consumed), this is the deadlock we suspect.
				 */
				pr_info_ratelimited("ml-credit: EVT_TX grant cnt=%u IGNORED (do_tx=%d rd=%u wr=%u)\n",
						    cnt, do_tx, dev->tx_read_acc, dev->tx_write_acc);
			}
		}
		/* bitmap == 0: fall through and re-read 0x13 (vendor does not break). */
	}

	if (do_rx)
		artosyn_sdio_recv(dev);
	if (do_tx) {
		artosyn_sdio_send_data(dev);
		artosyn_sdio_send_cmd(dev);
	}
}

/* -------------------------------------------------------------------------- */
/* netdev ops, §2                                                             */
/* -------------------------------------------------------------------------- */

static int artosyn_sdio_net_open(struct net_device *ndev)
{
	struct artosyn_sdio_eth *eth = netdev_priv(ndev);

	eth->net_dev_opened = true;
	netif_tx_wake_all_queues(ndev);
	netif_carrier_on(ndev);
	return 0;
}

static int artosyn_sdio_net_close(struct net_device *ndev)
{
	struct artosyn_sdio_eth *eth = netdev_priv(ndev);
	struct sk_buff *skb;

	eth->net_dev_opened = false;
	netif_tx_stop_all_queues(ndev);
	cancel_work_sync(&eth->work);
	cancel_delayed_work_sync(&eth->delay_work);

	/* Drain, don't purge: every queued skb carries an inflight count from
	 * start_xmit, and skb_queue_purge frees without decrementing - the
	 * residue ratchets to ARSDIO_INFLIGHT_CAP and stalls TX permanently.
	 * The worker is cancelled above, so consume paths don't race the drain
	 * (and the queue ops are locked regardless).
	 */
	while ((skb = skb_dequeue(&eth->data_txq)) != NULL) {
		atomic_dec_if_positive(&eth->sdio_dev->inflight);
		kfree_skb(skb);
	}

  skb_queue_purge(&eth->cmd_txq);
	netif_carrier_off(ndev);

  return 0;
}

static netdev_tx_t artosyn_sdio_net_start_xmit(struct sk_buff *skb,
					       struct net_device *ndev)
{
	struct artosyn_sdio_eth *eth = netdev_priv(ndev);
	struct artosyn_sdio_device *dev = eth->sdio_dev;

	/* IPv4-only tunnel: first nibble must be 4 (header byte 0x45 for the
	 * common no-options case). Non-v4 frames are dropped.
	 */
	if (skb->len < sizeof(struct iphdr) || (skb->data[0] & 0xf0) != 0x40) {
		net_dbg_ratelimited("unsupported ip version %02x\n", skb->data[0]);
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	/* Honor the 1024 in-flight cap. */
	if (atomic_inc_return(&dev->inflight) > ARSDIO_INFLIGHT_CAP) {
		atomic_dec(&dev->inflight);
		if (credit_log)
			pr_info_ratelimited("ml-credit: xmit BUSY (inflight cap, qlen=%u unacked=%u)\n",
					    skb_queue_len(&eth->data_txq), eth->unacked_bytes);
		netif_tx_stop_queue(netdev_get_tx_queue(ndev, skb_get_queue_mapping(skb)));
		return NETDEV_TX_BUSY;
	}

	skb_queue_tail(&eth->data_txq, skb);
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
	queue_work(eth->workwq, &eth->work);
	return NETDEV_TX_OK;
}

static u16 artosyn_sdio_select_queue(struct net_device *ndev, struct sk_buff *skb,
				     struct net_device *sb_dev)
{
	return 0;
}

static int artosyn_sdio_change_mtu(struct net_device *ndev, int new_mtu)
{
	/* Permissive stub (the vendor accepts any MTU; userspace sets 4096). */
	ndev->mtu = new_mtu;
	return 0;
}

static const struct net_device_ops artosyn_sdio_netdev_ops = {
	.ndo_open	= artosyn_sdio_net_open,
	.ndo_stop	= artosyn_sdio_net_close,
	.ndo_start_xmit	= artosyn_sdio_net_start_xmit,
	.ndo_select_queue = artosyn_sdio_select_queue,
	.ndo_change_mtu	= artosyn_sdio_change_mtu,
};

/*
 * The 0xCC IP frames are compressed on the RF link: the shared 10.0.0 network
 * prefix is stripped (only the last src/dst octet is carried) and must be rebuilt
 * on RX from the local interface address (artosyn_sdio_restore_ip_head reads it
 * from eth->ip_addr). The vendor .ko captured that address when userspace ran
 * `ifconfig sdio0 10.0.0.1`; reproduce it with an inetaddr notifier. Without this
 * eth->ip_addr stays 0 and every received frame is rebuilt as 0.0.0.x (wrong
 * prefix, self-consistent checksum) and then martian-dropped by the local stack.
 */
static int artosyn_sdio_inetaddr_event(struct notifier_block *nb,
				       unsigned long event, void *ptr)
{
	struct in_ifaddr *ifa = ptr;
	struct net_device *ndev = ifa->ifa_dev ? ifa->ifa_dev->dev : NULL;
	struct artosyn_sdio_eth *eth;

	if (!ndev || ndev->netdev_ops != &artosyn_sdio_netdev_ops)
		return NOTIFY_DONE;

	eth = netdev_priv(ndev);
	switch (event) {
	case NETDEV_UP:
		/* __be32 in network order; its low 3 octets are the 10.0.0 prefix
		 * that restore_ip_head applies to rebuilt RX headers.
		 */
		eth->ip_addr = ifa->ifa_local;
		netdev_info(ndev, "link IP %pI4 -> RX prefix rebuild enabled\n",
			    &eth->ip_addr);
		break;
	case NETDEV_DOWN:
		eth->ip_addr = 0;
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block artosyn_sdio_inetaddr_nb = {
	.notifier_call = artosyn_sdio_inetaddr_event,
};

static void artosyn_sdio_net_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &artosyn_sdio_netdev_ops;
	ndev->flags |= IFF_NOARP | IFF_BROADCAST;	/* 0x82 */
	ndev->type = ARPHRD_NONE;			/* 0xfffe */
	ndev->hard_header_len = 18;			/* room for compressed IP hdr */
	ndev->addr_len = 0;
	ndev->tx_queue_len = 1000;
	ndev->mtu = 4096;
	ndev->min_mtu = 0;
	ndev->max_mtu = 65535;
}

/* -------------------------------------------------------------------------- */
/* Char control device (/dev/artosyn_sdio), §3                               */
/* -------------------------------------------------------------------------- */

static void artosyn_dev_put(struct artosyn_sdio_device *dev)
{
	if (refcount_dec_and_test(&dev->refs))
		kfree(dev);
}

/*
 * fd-op gate vs SDIO removal. An op bracketed by op_enter/op_exit may use
 * dev->func and dev->ndev: remove() sets removed, wakes the cmd_wq sleepers
 * and drains active_ops to zero before tearing either down. Ops arriving
 * after removal bail here with the device intact (freed only on last put).
 */
static bool artosyn_op_enter(struct artosyn_sdio_device *dev)
{
	atomic_inc(&dev->active_ops);
	smp_mb__after_atomic();	/* inc visible before the removed check */
	if (!READ_ONCE(dev->removed))
		return true;
	if (atomic_dec_and_test(&dev->active_ops))
		complete(&dev->ops_drained);
	return false;
}

static void artosyn_op_exit(struct artosyn_sdio_device *dev)
{
	if (atomic_dec_and_test(&dev->active_ops) && READ_ONCE(dev->removed))
		complete(&dev->ops_drained);
}

/* Mark removed, wake blocked read/write sleepers, wait out in-flight ops. */
static void artosyn_quiesce(struct artosyn_sdio_device *dev)
{
	WRITE_ONCE(dev->removed, true);
	smp_mb();	/* removed visible before sampling active_ops */
	wake_up_all(&dev->cmd_wq);
	while (atomic_read(&dev->active_ops))
		wait_for_completion_timeout(&dev->ops_drained, HZ);
}

static int artosyn_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *m = filp->private_data;
	struct artosyn_sdio_device *dev =
		container_of(m, struct artosyn_sdio_device, misc);

	refcount_inc(&dev->refs);
	filp->private_data = dev;
	return 0;
}

static int artosyn_close(struct inode *inode, struct file *filp)
{
	artosyn_dev_put(filp->private_data);
	return 0;
}

/*
 * bb_ioctl primitive (ioctl nr 0): a raw MMC-command passthrough, not a
 * 0x44/0x48 doorbell. The first 16 bytes of struct artosyn_cmd_args ARE an
 * mmc_command's {opcode, arg, flags, retries}, copied straight in:
 *
 *   cmd.opcode = c->opcode;  cmd.arg = c->arg;
 *   cmd.flags  = c->flags;   cmd.retries = c->retries;
 *   mmc_wait_for_cmd(func->card->host, &cmd, 0);   // retries arg is 0
 *
 * After the round-trip it checks the SDIO R5 response error bits in resp[0]
 * (bit 11 -> -EIO, bit 9 -> -EINVAL, bit 8 -> -ERANGE) and copies the 16-byte
 * cmd.resp[] back into c->resp[]. The opcode meaning lives in the chip
 * firmware; the kernel just shuttles the command.
 */
static long artosyn_ioctl_cmd(struct artosyn_sdio_device *dev,
			      struct artosyn_cmd_args *c)
{
	struct sdio_func *func = dev->func;
	struct mmc_command cmd = { 0 };
	int ret;

	cmd.opcode = c->opcode;
	cmd.arg = c->arg;
	cmd.flags = c->flags;
	cmd.retries = c->retries;	/* mmc_wait_for_cmd overwrites with arg 0 */

	ret = mmc_wait_for_cmd(func->card->host, &cmd, 0);
	if (ret)
		return ret;

	/* SDIO R5 response status byte error bits. */
	if (cmd.resp[0] & BIT(11))
		return -EIO;
	if (cmd.resp[0] & BIT(9))
		return -EINVAL;
	if (cmd.resp[0] & BIT(8))
		return -ERANGE;

	memcpy(c->resp, cmd.resp, sizeof(c->resp));	/* 16-byte R-response */
	return 0;
}

static long artosyn_do_ioctl(struct artosyn_sdio_device *dev,
			     unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct sdio_func *func = dev->func;
	long ret = 0;
	int err = 0;

	switch (cmd) {
	case ARSDIO_IOC_CMD: {
		struct artosyn_cmd_args c;

		if (copy_from_user(&c, uarg, sizeof(c)))
			return -EFAULT;
		sdio_claim_host(func);
		ret = artosyn_ioctl_cmd(dev, &c);
		sdio_release_host(func);
		if (ret)
			return ret;
		if (copy_to_user(uarg, &c, sizeof(c)))
			return -EFAULT;
		return 0;
	}
	case ARSDIO_IOC_REG_RD: {
		struct artosyn_rw_args rw;

		if (copy_from_user(&rw, uarg, sizeof(rw)))
			return -EFAULT;
		sdio_claim_host(func);
		rw.val = sdio_readb(func, rw.addr, &err);
		sdio_release_host(func);
		if (err)
			return err;
		if (copy_to_user(uarg, &rw, sizeof(rw)))
			return -EFAULT;
		return 0;
	}
	case ARSDIO_IOC_REG_WR: {
		struct artosyn_rw_args rw;

		if (copy_from_user(&rw, uarg, sizeof(rw)))
			return -EFAULT;
		sdio_claim_host(func);
		sdio_writeb(func, (u8)rw.val, rw.addr, &err);
		sdio_release_host(func);
		return err;
	}
	case ARSDIO_IOC_CCCR_RD: {
		struct artosyn_rw_args rw;

		if (copy_from_user(&rw, uarg, sizeof(rw)))
			return -EFAULT;
		sdio_claim_host(func);
		rw.val = sdio_f0_readb(func, rw.addr, &err);
		sdio_release_host(func);
		if (err)
			return err;
		if (copy_to_user(uarg, &rw, sizeof(rw)))
			return -EFAULT;
		return 0;
	}
	case ARSDIO_IOC_CCCR_WR: {
		struct artosyn_rw_args rw;

		if (copy_from_user(&rw, uarg, sizeof(rw)))
			return -EFAULT;
		sdio_claim_host(func);
		sdio_f0_writeb(func, (u8)rw.val, rw.addr, &err);
		sdio_release_host(func);
		return err;
	}
	case ARSDIO_IOC_MSG_GET: {
		/*
		 * Two-channel mailbox read. The 4-byte struct is byte-addressed:
		 * valid.b[0]/.b[1] select channel 0/1 on input; on output each
		 * requested channel that has a latched message returns it (and is
		 * consumed). Latched by the IRQ from regs 0x54 (ch0) / 0x58 (ch1).
		 */
		struct artosyn_msg_args m;
		u8 *vb, *mb;
		int ch;

		if (copy_from_user(&m, uarg, sizeof(m)))
			return -EFAULT;
		vb = (u8 *)&m.valid;
		mb = (u8 *)&m.message;
		for (ch = 0; ch < 2; ch++) {
			if (!vb[ch])
				continue;
			vb[ch] = dev->mailbox_valid[ch] ? 1 : 0;
			if (!vb[ch])
				continue;
			mb[ch] = dev->mailbox_msg[ch];
			dev->mailbox_msg[ch] = 0;
			dev->mailbox_valid[ch] = false;
		}
		if (copy_to_user(uarg, &m, sizeof(m)))
			return -EFAULT;
		return 0;
	}
	case ARSDIO_IOC_MSG_POST: {
		/*
		 * Two-channel mailbox write: valid.b[0] gates message.b[0] -> reg
		 * 0x44, valid.b[1] gates message.b[1] -> reg 0x48 (host->chip).
		 */
		struct artosyn_msg_args m;
		u8 *vb, *mb;

		if (copy_from_user(&m, uarg, sizeof(m)))
			return -EFAULT;
		vb = (u8 *)&m.valid;
		mb = (u8 *)&m.message;
		sdio_claim_host(func);
		if (vb[0])
			sdio_writeb(func, mb[0], REG_MBOX_TX0, &err);
		if (vb[1])
			sdio_writeb(func, mb[1], REG_MBOX_TX1, &err);
		sdio_release_host(func);
		return err;
	}
	default: {
		return -ENOTTY;
	}
	}
}

static long artosyn_unlocked_ioctl(struct file *filp, unsigned int cmd,
				   unsigned long arg)
{
	struct artosyn_sdio_device *dev = filp->private_data;
	long ret;

	if (!artosyn_op_enter(dev))
		return -ENODEV;
	ret = artosyn_do_ioctl(dev, cmd, arg);
	artosyn_op_exit(dev);
	return ret;
}

/*
 * Char-device .write: the daemon's bb_socket TX. The user buffer is an opaque
 * command payload - the kernel cmd channel is a single byte pipe (slot/port
 * multiplexing lives in the daemon, not here). Queue it to cmd_txq and kick the
 * worker, which packs it with a 0xDD trailer and CMD53s it out. Depth-1 flow
 * control: block until the previous cmd batch has drained. Returns count so the
 * daemon's "wrote == requested" success test passes.
 */
static ssize_t artosyn_do_write(struct artosyn_sdio_device *dev,
				struct file *filp, const char __user *buf,
				size_t count)
{
	struct artosyn_sdio_eth *eth;
	struct sk_buff *skb;

	if (!dev->ndev)
		return -EIO;
	if (!count)
		return 0;
	eth = netdev_priv(dev->ndev);

	if (!skb_queue_empty(&eth->cmd_txq)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(dev->cmd_wq,
				skb_queue_empty(&eth->cmd_txq) || dev->removed)) {
			return -ERESTARTSYS;
		}
		if (dev->removed)
			return -EIO;
	}

	skb = netdev_alloc_skb(dev->ndev, count);
	if (!skb)
		return -ENOMEM;
	if (copy_from_user(skb_put(skb, count), buf, count)) {
		kfree_skb(skb);
		return -EFAULT;
	}
	ml_framelog("TX", skb->data, count);
	skb_queue_tail(&eth->cmd_txq, skb);
	queue_work(eth->workwq, &eth->work);
	return count;
}

static ssize_t artosyn_write(struct file *filp, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct artosyn_sdio_device *dev = filp->private_data;
	ssize_t ret;

	if (!artosyn_op_enter(dev))
		return -EIO;
	ret = artosyn_do_write(dev, filp, buf, count);
	artosyn_op_exit(dev);
	return ret;
}

/*
 * Char-device .read: the daemon's bb_socket RX. Returns one queued command
 * frame (filled by artosyn_sdio_recv on a 0xDD trailer). Blocks until one is
 * available; a short read pushes the remainder back at the queue head.
 */
static ssize_t artosyn_do_read(struct artosyn_sdio_device *dev,
			       struct file *filp, char __user *buf,
			       size_t count)
{
	struct artosyn_sdio_eth *eth;
	struct sk_buff *skb;
	size_t n;

	if (!dev->ndev)
		return -EIO;
	if (!count)
		return 0;
	eth = netdev_priv(dev->ndev);

	if (skb_queue_empty(&eth->cmd_rxq)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(dev->cmd_wq,
				!skb_queue_empty(&eth->cmd_rxq) || dev->removed)) {
			return -ERESTARTSYS;
		}
		if (dev->removed)
			return -EIO;
	}

	skb = skb_dequeue(&eth->cmd_rxq);
	if (!skb)
		return -EIO;
	n = min_t(size_t, skb->len, count);
	ml_framelog("RX", skb->data, n);
	if (copy_to_user(buf, skb->data, n)) {
		skb_queue_head(&eth->cmd_rxq, skb);
		return -EFAULT;
	}
	if (n >= skb->len) {
		kfree_skb(skb);
	} else {
		skb_pull(skb, n);
		skb_queue_head(&eth->cmd_rxq, skb);
	}
	return n;
}

static ssize_t artosyn_read(struct file *filp, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct artosyn_sdio_device *dev = filp->private_data;
	ssize_t ret;

	if (!artosyn_op_enter(dev))
		return -EIO;
	ret = artosyn_do_read(dev, filp, buf, count);
	artosyn_op_exit(dev);
	return ret;
}

/*
 * Char-device .poll: POLLIN when a cmd frame is queued, POLLOUT when the cmd TX
 * queue has drained. The daemon's RX thread poll()s for POLLIN before each read;
 * without this fop poll() defaults to always-ready and the daemon busy-spins.
 */
static __poll_t artosyn_poll(struct file *filp, struct poll_table_struct *pt)
{
	struct artosyn_sdio_device *dev = filp->private_data;
	struct artosyn_sdio_eth *eth;
	__poll_t mask = 0;

	if (!artosyn_op_enter(dev))
		return EPOLLERR;
	if (!dev->ndev) {
		artosyn_op_exit(dev);
		return EPOLLERR;
	}
	eth = netdev_priv(dev->ndev);

	poll_wait(filp, &dev->cmd_wq, pt);
	if (!skb_queue_empty(&eth->cmd_rxq))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (skb_queue_empty(&eth->cmd_txq))
		mask |= EPOLLOUT | EPOLLWRNORM;
	if (dev->mailbox_valid[0] || dev->mailbox_valid[1])
		mask |= EPOLLPRI;
	/*
	 * Vendor artosyn_poll @0x30b8: when nothing is ready, pump the IRQ handler
	 * once (under the host claim) so a poll()-driven consumer drains the chip
	 * even when the hardware IRQ is quiet. recv's wake_up re-triggers this poll,
	 * which then reports POLLIN.
	 */
	if (!mask) {
		sdio_claim_host(dev->func);
		artosyn_sdio_irqhandler(dev->func);
		sdio_release_host(dev->func);
	}
	artosyn_op_exit(dev);
	return mask;
}

static const struct file_operations artosyn_fops = {
	.owner		= THIS_MODULE,
	.open		= artosyn_open,
	.release	= artosyn_close,
	.read		= artosyn_read,
	.write		= artosyn_write,
	.poll		= artosyn_poll,
	.unlocked_ioctl	= artosyn_unlocked_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	/* no_llseek was removed in 6.12; default (no .llseek) is fine. */
};

/* -------------------------------------------------------------------------- */
/* SDIO probe / remove, §1                                                    */
/* -------------------------------------------------------------------------- */

static int artosyn_alloc_buffers(struct artosyn_sdio_device *dev)
{
	dev->tx_buf = kzalloc(ARSDIO_STAGING, GFP_KERNEL);
	dev->rx_buf = kzalloc(ARSDIO_RX_BUF, GFP_KERNEL);
	dev->fw_buf = kzalloc(ARSDIO_SCRATCH, GFP_KERNEL);
	if (!dev->tx_buf || !dev->rx_buf || !dev->fw_buf)
		return -ENOMEM;
	return 0;
}

static void artosyn_free_buffers(struct artosyn_sdio_device *dev)
{
	kfree(dev->tx_buf);
	kfree(dev->rx_buf);
	kfree(dev->fw_buf);
}

static int artosyn_setup_netdev(struct artosyn_sdio_device *dev)
{
	struct artosyn_sdio_eth *eth;
	struct net_device *ndev;
	int ret;

	/* 8 TX queues, 1 RX queue; priv is artosyn_sdio_eth. */
	ndev = alloc_netdev_mqs(sizeof(struct artosyn_sdio_eth), "sdio%d",
				NET_NAME_UNKNOWN, artosyn_sdio_net_setup, 8, 1);
	if (!ndev)
		return -ENOMEM;

	eth = netdev_priv(ndev);
	eth->sdio_dev = dev;
	eth->net_dev = ndev;
	eth->unacked_max = tx_window;	/* 0 (default) = in-flight gate DISABLED; see tx_window */
	skb_queue_head_init(&eth->data_txq);
	skb_queue_head_init(&eth->cmd_txq);
	skb_queue_head_init(&eth->cmd_rxq);
	skb_queue_head_init(&eth->unacked_txq);
	INIT_WORK(&eth->work, artosyn_sdio_eth_worker);
	INIT_DELAYED_WORK(&eth->delay_work, artosyn_sdio_eth_delay_worker);

	eth->workwq = alloc_workqueue("artosyn_sdio_eth_wq", WQ_MEM_RECLAIM, 1);
	if (!eth->workwq) {
		free_netdev(ndev);
		return -ENOMEM;
	}

	dev->ndev = ndev;
	netif_carrier_off(ndev);

	ret = register_netdev(ndev);
	if (ret) {
		pr_err("register_netdev failed: %d\n", ret);
		destroy_workqueue(eth->workwq);
		free_netdev(ndev);
		dev->ndev = NULL;
		return ret;
	}
	return 0;
}

static int sdio_artosyn_probe(struct sdio_func *func,
			      const struct sdio_device_id *id)
{
	struct artosyn_sdio_device *dev;
	const struct firmware *fw = NULL;
	const struct firmware *cfg = NULL;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->func = func;
	mutex_init(&dev->io_lock);
	init_waitqueue_head(&dev->cmd_wq);
	atomic_set(&dev->inflight, 0);
	refcount_set(&dev->refs, 1);	/* probe ref; dropped in remove */
	atomic_set(&dev->active_ops, 0);
	init_completion(&dev->ops_drained);
	sdio_set_drvdata(func, dev);

	ret = artosyn_alloc_buffers(dev);
	if (ret)
		goto err_free;

	/* Bring-up: claim, enable function, and force the CMD53 block size to 512 on
	 * BOTH the 0x8030 upload chip and the live 0x8031 chip. The whole flow-control
	 * is 512-granular (RX count 0x5c<<9, TX credit 0x60<<9, ALIGN(used,512)); the
	 * 0x8031 re-enumeration is a fresh sdio_func, so without this it keeps the core
	 * default (!= 512) and large multi-block bulk reads (0xCC IP/video -> sdio0)
	 * desync the chip FIFO -> sdio0 rx stuck at 0, while small/byte-mode traffic
	 * (credit, bb_ioctl GETs, small 0xDD cmds) still works.
	 */
	sdio_claim_host(func);
	ret = sdio_enable_func(func);
	if (ret) {
		sdio_release_host(func);
		pr_err("artosyn function IO is NOT ready (enable_func=%d)\n", ret);
		goto err_buf;
	}
	ret = sdio_set_block_size(func, ARSDIO_BLOCK);

	/* Bring-up pokes: 0x14 <- 0x01 (IO control enable), 0x68 <- 0xF0
	 * (arm/clear the event-status upper nibble).
	 */
	ar_writeb(func, 0x01, REG_CTRL);
	ar_writeb(func, 0xF0, REG_EVT_STATUS);
	sdio_release_host(func);

	/* misc char device. */
	dev->misc.minor = MISC_DYNAMIC_MINOR;
	dev->misc.name = "artosyn_sdio";
	dev->misc.fops = &artosyn_fops;
	ret = misc_register(&dev->misc);
	if (ret) {
		pr_err("misc_register() failed: ret=%d\n", ret);
		goto err_buf;
	}
	dev->misc_registered = true;

	/* Device-ID fork: 0x8030 uploads firmware then RETURNS. The finalize frame boots the
	 * SPL/firmware; the chip resets and re-enumerates as 0x8031, which re-probes here and
	 * takes the path below to build the netdev. Do not build the netdev on the still-
	 * rebooting 0x8030 chip (it is about to disappear).
	 */
	if (func->device == ARSDIO_DEV_DOWNLOAD) {
		ret = request_firmware(&fw, fw_name, &func->dev);
		if (ret) {
			pr_err("request_firmware %s failed! ret %d\n", fw_name, ret);
			goto err_misc;
		}
		ret = request_firmware(&cfg, cfg_name, &func->dev);
		if (ret) {
			pr_err("request cfg %s failed! ret %d\n", cfg_name, ret);
			release_firmware(fw);
			goto err_misc;
		}

		sdio_claim_host(func);
		ret = sdio_request_fw(dev, fw, cfg);	/* block size already set (512) above */
		sdio_release_host(func);

		release_firmware(cfg);
		release_firmware(fw);
		if (ret) {
			pr_err("sdio_request_fw failed: %d\n", ret);
			goto err_misc;
		}

		/* Boot frame sent; let the chip reset and re-enumerate as 0x8031, and kick the
		 * host to rescan so that re-enumeration is detected. The 0x8031 re-probe builds
		 * the netdev.
		 */
		pr_info("firmware uploaded; chip re-enumerating as 0x%04x\n", ARSDIO_DEV_SKIPFW);
		mmc_detect_change(func->card->host, msecs_to_jiffies(100));
		return 0;
	}

	pr_info("devid 0x%04x: firmware already running, bringing up netdev\n", func->device);
	ret = artosyn_setup_netdev(dev);
	if (ret)
		goto err_misc;

	/* Claim the SDIO IRQ last (handler dereferences dev->ndev). */
	sdio_claim_host(func);
	ret = sdio_claim_irq(func, artosyn_sdio_irqhandler);
	sdio_release_host(func);
	if (ret) {
		pr_err("sdio_claim_irq failed: %d\n", ret);
		goto err_netdev;
	}

	pr_info("probed AR8030 (devid 0x%04x), iface %s\n",
		func->device, dev->ndev->name);
	return 0;

err_netdev:
	artosyn_quiesce(dev);	/* an fd may already sit in read/write */
	unregister_netdev(dev->ndev);
	destroy_workqueue(((struct artosyn_sdio_eth *)netdev_priv(dev->ndev))->workwq);
	free_netdev(dev->ndev);
	dev->ndev = NULL;
err_misc:
	artosyn_quiesce(dev);
	misc_deregister(&dev->misc);
	dev->misc_registered = false;
err_buf:
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
	artosyn_free_buffers(dev);
err_free:
	sdio_set_drvdata(func, NULL);
	artosyn_dev_put(dev);
	return ret;
}

static void sdio_artosyn_remove(struct sdio_func *func)
{
	struct artosyn_sdio_device *dev = sdio_get_drvdata(func);
	struct artosyn_sdio_eth *eth;

	if (!dev)
		return;

	/*
	 * Quiesce the char-device fds before anything is torn down: after
	 * this no fd op is inside the driver and new ones bail, so func and
	 * ndev may go away. The fds themselves keep dev alive via refs.
	 */
	artosyn_quiesce(dev);

	sdio_claim_host(func);
	sdio_release_irq(func);
	sdio_release_host(func);

	if (dev->ndev) {
		eth = netdev_priv(dev->ndev);
		unregister_netdev(dev->ndev);
		if (eth->workwq)
			destroy_workqueue(eth->workwq);

		free_netdev(dev->ndev);
		dev->ndev = NULL;
	}

	if (dev->misc_registered)
		misc_deregister(&dev->misc);

	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);

	artosyn_free_buffers(dev);
	sdio_set_drvdata(func, NULL);
	artosyn_dev_put(dev);

	pr_info("removed\n");
}

/* -------------------------------------------------------------------------- */
/* SDIO driver registration                                                   */
/* -------------------------------------------------------------------------- */

static const struct sdio_device_id artosyn_sdio_ids[] = {
	{ SDIO_DEVICE(ARSDIO_VENDOR, ARSDIO_DEV_DOWNLOAD) },	/* 4152:8030 */
	{ SDIO_DEVICE(ARSDIO_VENDOR, ARSDIO_DEV_SKIPFW) },	/* 4152:8031 */
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(sdio, artosyn_sdio_ids);

static struct sdio_driver sdio_artosyn_driver = {
	.name		= "artosyn_sdio",
	.id_table	= artosyn_sdio_ids,
	.probe		= sdio_artosyn_probe,
	.remove		= sdio_artosyn_remove,
};

static int __init artosyn_sdio_init(void)
{
	int ret;

	register_inetaddr_notifier(&artosyn_sdio_inetaddr_nb);
	ret = sdio_register_driver(&sdio_artosyn_driver);
	if (ret)
		unregister_inetaddr_notifier(&artosyn_sdio_inetaddr_nb);
	return ret;
}

static void __exit artosyn_sdio_exit(void)
{
	sdio_unregister_driver(&sdio_artosyn_driver);
	unregister_inetaddr_notifier(&artosyn_sdio_inetaddr_nb);
}

module_init(artosyn_sdio_init);
module_exit(artosyn_sdio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("missinglynk (open reimplementation of Artosyn artosyn_sdio)");
MODULE_DESCRIPTION("Open AR8030 RF-link SDIO driver (FPV video downlink) for missinglynk");
