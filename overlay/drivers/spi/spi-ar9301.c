// SPDX-License-Identifier: GPL-2.0
/*
 * Artosyn AR9301 / Proxima-9311 QSPI controller (spi-mem).
 *
 * spi-mem controller driver for the Artosyn AR9301 QSPI block, which fronts the
 * on-board SPI-NAND. This driver implements only the controller; the mainline
 * SPI-NAND core, MTD, UBI and squashfs stack binds on top of it unchanged. It
 * drives the always-present PIO/FIFO data path (no IRQ, no AHB-DMAC fast path).
 *
 * The registers and bitfields are described inline below. A wrong offset or bit
 * stops the SPI-NAND from probing, so change them with care.
 *
 * Style: full braces even on single-statement bodies (house style); checkpatch
 * warns ("braces not necessary"), not an error.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

/* Register block at "qspi_base". */
#define AR9301_CTRL		0x04	/* mode control; bits 17/19 (uncertain) */
#define AR9301_LANE		0x10	/* per-phase bus-width + enable; written last */
#define AR9301_CMD		0x14	/* opcode + addr/dummy counts + direction */
#define AR9301_ADDR		0x18	/* 32-bit flash address */
#define AR9301_DATALEN		0x1c	/* data-phase length in bytes */
#define AR9301_DMACTRL		0x20	/* DMA request enable (PIO: kept 0) */
#define AR9301_STATUS0		0x30	/* bit0 = controller busy */
#define AR9301_FIFOSTAT		0x34	/* RX/TX FIFO levels + RX-empty flag */
#define AR9301_IRQ_EN		0x40	/* interrupt enable mask (PIO: kept 0) */
#define AR9301_IRQ_STATUS	0x44	/* bit0 = transfer-complete, W1C */
#define AR9301_RX_FIFO		0x100	/* RX data read port */
#define AR9301_TX_FIFO		0x200	/* TX data write port */

/* AR9301_CMD fields. */
#define AR9301_CMD_OPCODE	GENMASK(7, 0)
#define AR9301_CMD_ADDR_BYTES	GENMASK(10, 8)
#define AR9301_CMD_DUMMY_BYTES	GENMASK(15, 12)
#define AR9301_CMD_WRITE	BIT(24)

/*
 * AR9301_LANE fields. Each phase's bus width is encoded as a 2-bit lane code
 * (1->0, 2->1, 4->2). The low field (bits[3:2]) is 0 for a single-lane command
 * and otherwise the address code; bits[5:4] is the address code. Real spi-nand
 * traffic is 1-1-1 / 1-1-4, so only the data field (bits[9:8]) is ever nonzero;
 * the command/address fields are the least certain. For all-single ops every
 * field is 0 and LANE == AR9301_LANE_ENABLE.
 */
#define AR9301_LANE_ENABLE	BIT(0)
#define AR9301_LANE_CMDADDR	GENMASK(3, 2)	/* 0 if cmd single, else addr code */
#define AR9301_LANE_ADDR	GENMASK(5, 4)	/* address lane code */
#define AR9301_LANE_DUMMY	GENMASK(7, 6)	/* dummy lane code */
#define AR9301_LANE_DATA	GENMASK(9, 8)	/* data lane code */

/* AR9301_STATUS0. */
#define AR9301_STATUS0_BUSY	BIT(0)

/*
 * AR9301_FIFOSTAT. bit7 is RX-FIFO-EMPTY, NOT a "done" flag: a sub-word tail
 * read must spin WHILE bit7 is set and pop the partial word once it clears (a
 * word is present); a fully drained read leaves bit7 SET. Polling it as "done"
 * (waiting for it to SET before the tail read) deadlocks whenever a partial word
 * is queued, which is every sub-4-byte read such as READID.
 */
#define AR9301_FIFOSTAT_RX_LEVEL	GENMASK(6, 0)	/* words available to read */
#define AR9301_FIFOSTAT_RX_EMPTY	BIT(7)		/* RX FIFO empty */
#define AR9301_FIFOSTAT_TX_FREE		GENMASK(14, 8)	/* words of TX space */

/* FIFO depth in 32-bit words (TX-free saturates at 0x40). */
#define AR9301_FIFO_WORDS	64

/* PIO data-phase cap (0x3ffc bytes). */
#define AR9301_MAX_DATA_LEN	0x3ffc

/* Poll bounds: the bus runs at up to 150 MHz, so even a full page drains fast. */
#define AR9301_POLL_US		2
#define AR9301_POLL_TIMEOUT_US	100000

struct ar9301_qspi {
	struct device		*dev;
	void __iomem		*base;		/* qspi_base */
	void __iomem		*cgu_base;	/* mapped, unused (bootloader owns it) */
	void __iomem		*dma_base;	/* mapped, unused (DMAC fast path omitted) */
	void __iomem		*hs_sel_base;	/* mapped, unused (bootloader owns it) */
	struct clk		*clk;		/* optional */
	struct mutex		lock;		/* serialises exec_op */
};

/* Map an spi-mem bus width to the controller's 2-bit lane code. */
static u32 ar9301_lane_code(u8 buswidth)
{
	switch (buswidth) {
	case 4:
		return 2;

	case 2:
		return 1;

	default:
		return 0;
	}
}

/*
 * Program the command, address, length and lane registers for one op.
 * AR9301_LANE is written last because that write appears to arm the engine;
 * do not reorder these stores.
 */
static int ar9301_send_cmd(struct ar9301_qspi *qspi, const struct spi_mem_op *op)
{
	u32 cmd;
	u32 lane;
	u32 addr_code;
	u32 status;
	int ret;

	/* Wait for any previous command to retire before touching the regs. */
	ret = readl_poll_timeout(qspi->base + AR9301_STATUS0, status,
				 !(status & AR9301_STATUS0_BUSY),
				 AR9301_POLL_US, AR9301_POLL_TIMEOUT_US);
	if (ret) {
		dev_err(qspi->dev, "timeout waiting for controller idle\n");
		return ret;
	}

	cmd = FIELD_PREP(AR9301_CMD_OPCODE, op->cmd.opcode);
	cmd |= FIELD_PREP(AR9301_CMD_ADDR_BYTES, op->addr.nbytes);
	cmd |= FIELD_PREP(AR9301_CMD_DUMMY_BYTES, op->dummy.nbytes);
	if (op->data.dir == SPI_MEM_DATA_OUT)
		cmd |= AR9301_CMD_WRITE;

	writel(cmd, qspi->base + AR9301_CMD);

	if (op->addr.nbytes)
		writel(lower_32_bits(op->addr.val), qspi->base + AR9301_ADDR);

	writel(op->data.nbytes, qspi->base + AR9301_DATALEN);

	/*
	 * Lane field layout: bits[3:2] are 0 when the command phase is single-lane
	 * (the normal case) and otherwise carry the address code; bits[5:4] always
	 * carry the address code.
	 */
	addr_code = ar9301_lane_code(op->addr.buswidth);
	lane = AR9301_LANE_ENABLE;
	lane |= FIELD_PREP(AR9301_LANE_CMDADDR, op->cmd.buswidth == 1 ? 0 : addr_code);
	lane |= FIELD_PREP(AR9301_LANE_ADDR, addr_code);
	lane |= FIELD_PREP(AR9301_LANE_DUMMY, ar9301_lane_code(op->dummy.buswidth));
	lane |= FIELD_PREP(AR9301_LANE_DATA, ar9301_lane_code(op->data.buswidth));
	writel(lane, qspi->base + AR9301_LANE);

	return 0;
}

/* Push the data buffer into the TX FIFO, words first then a sub-word tail. */
static int ar9301_write_fifo(struct ar9301_qspi *qspi, const u8 *buf, unsigned int len)
{
	unsigned int words = len / 4;
	unsigned int tail = len % 4;
	u32 fifostat;
	int ret;
	int i;

	while (words) {
		unsigned int free;
		unsigned int n;

		ret = readl_poll_timeout(qspi->base + AR9301_FIFOSTAT, fifostat,
					 FIELD_GET(AR9301_FIFOSTAT_TX_FREE, fifostat),
					 AR9301_POLL_US, AR9301_POLL_TIMEOUT_US);
		if (ret) {
			dev_err(qspi->dev, "timeout waiting for TX FIFO space\n");
			return ret;
		}

		free = FIELD_GET(AR9301_FIFOSTAT_TX_FREE, fifostat);
		n = min(free, words);
		while (n--) {
			u32 word;

			memcpy(&word, buf, 4);
			writel(word, qspi->base + AR9301_TX_FIFO);
			buf += 4;
			words--;
		}
	}

	/* Sub-word tail: the controller accepts byte-wide stores to the port. */
	for (i = 0; i < tail; i++)
		writeb(buf[i], qspi->base + AR9301_TX_FIFO);

	return 0;
}

/* Drain the RX FIFO into the data buffer, words first then a sub-word tail. */
static int ar9301_read_fifo(struct ar9301_qspi *qspi, u8 *buf, unsigned int len)
{
	unsigned int words = len / 4;
	unsigned int tail = len % 4;
	u32 fifostat;
	u32 word;
	int ret;
	int i;

	while (words) {
		unsigned int avail;
		unsigned int n;

		ret = readl_poll_timeout(qspi->base + AR9301_FIFOSTAT, fifostat,
					 FIELD_GET(AR9301_FIFOSTAT_RX_LEVEL, fifostat),
					 AR9301_POLL_US, AR9301_POLL_TIMEOUT_US);
		if (ret) {
			dev_err(qspi->dev, "timeout waiting for RX FIFO data\n");
			return ret;
		}

		avail = FIELD_GET(AR9301_FIFOSTAT_RX_LEVEL, fifostat);
		n = min(avail, words);
		while (n--) {
			word = readl(qspi->base + AR9301_RX_FIFO);
			memcpy(buf, &word, 4);
			buf += 4;
			words--;
		}
	}

	if (!tail)
		return 0;

	/*
	 * The partial final word does not advance the word-granular RX_LEVEL
	 * counter, so wait on the RX-empty flag instead: spin until it CLEARS
	 * (the word has landed), then pop it. The vendor read_fifo does exactly
	 * this (asm 0x1770-0x1790). Waiting for it to SET here was the bug that
	 * hung every sub-4-byte read (e.g. the 2-byte SPI-NAND READID).
	 */
	ret = readl_poll_timeout(qspi->base + AR9301_FIFOSTAT, fifostat,
				 !(fifostat & AR9301_FIFOSTAT_RX_EMPTY),
				 AR9301_POLL_US, AR9301_POLL_TIMEOUT_US);
	if (ret) {
		dev_err(qspi->dev, "timeout waiting for RX tail\n");
		return ret;
	}

	word = readl(qspi->base + AR9301_RX_FIFO);
	for (i = 0; i < tail; i++)
		buf[i] = word >> (8 * i);

	return 0;
}

/*
 * Block until the TX FIFO has fully drained to the flash. The vendor signals
 * write completion by polling TX-free up to the full FIFO depth (asm 0x1fa8:
 * FIFOSTAT[14:8] == 0x40), not by any "done" bit.
 */
static int ar9301_wait_tx_drained(struct ar9301_qspi *qspi)
{
	u32 fifostat;
	int ret;

	ret = readl_poll_timeout(qspi->base + AR9301_FIFOSTAT, fifostat,
				 FIELD_GET(AR9301_FIFOSTAT_TX_FREE, fifostat) == AR9301_FIFO_WORDS,
				 AR9301_POLL_US, AR9301_POLL_TIMEOUT_US);
	if (ret)
		dev_err(qspi->dev, "timeout waiting for TX FIFO drain\n");

	return ret;
}

/*
 * After a read, confirm the RX FIFO is fully drained (RX-empty set). The vendor
 * exec_op makes this a hard requirement (asm 0x2000: error if bit7 is clear),
 * which catches an under-read where data was left in the FIFO.
 */
static int ar9301_check_rx_drained(struct ar9301_qspi *qspi)
{
	u32 fifostat = readl(qspi->base + AR9301_FIFOSTAT);

	if (!(fifostat & AR9301_FIFOSTAT_RX_EMPTY)) {
		dev_err(qspi->dev, "RX FIFO not empty after read (stat %#x)\n", fifostat);
		return -EIO;
	}

	return 0;
}

static int ar9301_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct ar9301_qspi *qspi = spi_controller_get_devdata(mem->spi->controller);
	int ret;

	mutex_lock(&qspi->lock);

	ret = ar9301_send_cmd(qspi, op);
	if (ret)
		goto out;

	if (op->data.nbytes && op->data.dir == SPI_MEM_DATA_OUT) {
		ret = ar9301_write_fifo(qspi, op->data.buf.out, op->data.nbytes);
		if (ret)
			goto out;
		ret = ar9301_wait_tx_drained(qspi);
	} else if (op->data.nbytes) {
		ret = ar9301_read_fifo(qspi, op->data.buf.in, op->data.nbytes);
		if (ret)
			goto out;
		ret = ar9301_check_rx_drained(qspi);
	}
	/*
	 * A no-data op (e.g. WRITE ENABLE, BLOCK ERASE issue) needs no completion
	 * wait here: the vendor returns straight after send_cmd, and the next
	 * send_cmd's STATUS0-busy poll serialises against the engine.
	 */

out:
	/* Keep the DMA/IRQ machinery quiescent after each op. */
	writel(0, qspi->base + AR9301_DMACTRL);
	writel(0, qspi->base + AR9301_IRQ_EN);
	mutex_unlock(&qspi->lock);

	return ret;
}

static int ar9301_adjust_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	if (op->data.nbytes > AR9301_MAX_DATA_LEN)
		op->data.nbytes = AR9301_MAX_DATA_LEN;

	return 0;
}

static bool ar9301_supports_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	/*
	 * Accept every op the spi-mem core proposes; keep the default
	 * spi-mem sanity (1/2/4-lane only, byte-aligned, etc.); the controller
	 * has no further constraints that real spi-nand traffic hits.
	 */
	return spi_mem_default_supports_op(mem, op);
}

static const struct spi_controller_mem_ops ar9301_mem_ops = {
	.exec_op	= ar9301_exec_op,
	.adjust_op_size	= ar9301_adjust_op_size,
	.supports_op	= ar9301_supports_op,
};

static int ar9301_qspi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct ar9301_qspi *qspi;
	int ret;

	ctlr = devm_spi_alloc_host(dev, sizeof(*qspi));
	if (!ctlr)
		return -ENOMEM;

	qspi = spi_controller_get_devdata(ctlr);
	qspi->dev = dev;
	mutex_init(&qspi->lock);

	qspi->base = devm_platform_ioremap_resource_byname(pdev, "qspi_base");
	if (IS_ERR(qspi->base))
		return PTR_ERR(qspi->base);

	/*
	 * cgu_base and hs_sel_base are mapped for parity with the vendor binding
	 * but are programmed by the bootloader; dma_base belongs to the optional
	 * DMAC fast path we do not use. Mapping them keeps the DT contract intact
	 * and reserves the regions. All three are optional from our perspective.
	 */
	qspi->cgu_base = devm_platform_ioremap_resource_byname(pdev, "cgu_base");
	if (IS_ERR(qspi->cgu_base))
		qspi->cgu_base = NULL;

	qspi->dma_base = devm_platform_ioremap_resource_byname(pdev, "dma_base");
	if (IS_ERR(qspi->dma_base))
		qspi->dma_base = NULL;

	qspi->hs_sel_base = devm_platform_ioremap_resource_byname(pdev, "hs_sel_base");
	if (IS_ERR(qspi->hs_sel_base))
		qspi->hs_sel_base = NULL;

	/*
	 * The bootloader already gates the QSPI clock in the CGU, so the clock is
	 * optional; enable it when the DT provides one to keep us robust if it did
	 * not provide one.
	 */
	qspi->clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(qspi->clk))
		return dev_err_probe(dev, PTR_ERR(qspi->clk), "failed to get clock\n");

	/* Quiesce the control/IRQ/DMA registers at probe. */
	writel(0, qspi->base + AR9301_IRQ_EN);
	writel(0, qspi->base + AR9301_CTRL);
	writel(0, qspi->base + AR9301_CMD);
	writel(0, qspi->base + AR9301_DMACTRL);

	ctlr->dev.of_node = dev->of_node;
	ctlr->mem_ops = &ar9301_mem_ops;
	ctlr->mode_bits = SPI_TX_DUAL | SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8);
	ctlr->num_chipselect = 1;
	/* No ctlr->max_message_size: it is a callback, not a scalar, and adjust_op_size
	 * already caps each data phase at AR9301_MAX_DATA_LEN (the controller's PIO limit).
	 */

	ret = devm_spi_register_controller(dev, ctlr);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register controller\n");

	return 0;
}

static const struct of_device_id ar9301_qspi_of_match[] = {
	{ .compatible = "artosyn,ar9301-qspi" },
	{ }
};
MODULE_DEVICE_TABLE(of, ar9301_qspi_of_match);

static struct platform_driver ar9301_qspi_driver = {
	.probe	= ar9301_qspi_probe,
	.driver	= {
		.name		= "ar9301-qspi",
		.of_match_table	= ar9301_qspi_of_match,
	},
};
module_platform_driver(ar9301_qspi_driver);

MODULE_DESCRIPTION("Artosyn AR9301 / Proxima-9311 QSPI (spi-mem) controller");
MODULE_LICENSE("GPL");
