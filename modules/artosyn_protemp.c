// SPDX-License-Identifier: GPL-2.0
/*
 * artosyn_protemp.c - open reimplementation of the vendor "artosyn,protemp" SoC
 * temperature sensor (@0x0A108000), needed so the menu's SoC-temperature feature
 * (which reads /sys/bus/iio/devices/temperature/in_temp_scale directly) has an IIO
 * temperature device to read.
 *
 * The protemp block is a separate temperature ADC that lives inside the same MMIO
 * window as the "artosyn,adc" SAR ADC (artosyn_adc.c). It is NOT the same engine:
 * its temperature code lives at different offsets (0xcc/0xd0, a free-running 9-bit
 * value) and it uses a linear code->Celsius conversion, unlike the SAR ADC's
 * 10-bit DATA at 0x210. recovered from the vendor driver in
 * the reconstructed vendor kernel (artosynts_probe / artosynts_read_raw /
 * artosynts_get_raw.isra.2, the platform_driver whose driver.name = "protemp" and
 * of_device_id compatible = "artosyn,protemp"); full writeup in
 * ../docs/artosyn-protemp.md. All facts below are [confirmed] unless marked
 * otherwise.
 *
 * A NOTE ON THE SHARED MMIO WINDOW: this node's reg (0xa108000 0xd4) overlaps the
 * ADC node's (0xa108000 0x21c). The ADC driver claims its window exclusively via
 * devm_platform_ioremap_resource (request_mem_region). devm_platform_ioremap_resource
 * here would collide with -EBUSY, so - exactly like the vendor protemp probe - this
 * driver maps the window with a plain devm_ioremap (platform_get_resource + devm_ioremap),
 * which does not reserve the region and therefore does not fight the ADC's claim.
 */
#define pr_fmt(fmt) "artosyn_protemp: " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/iio/iio.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/sort.h>

#define AR_PROTEMP_CTRL		0x00	/* probe sets bit0 to enable the temp ADC */
#define AR_PROTEMP_CTRL_EN	BIT(0)
#define AR_PROTEMP_CFG		0x38	/* probe sets bits[6:4]; analog enable [inferred] */
#define AR_PROTEMP_CFG_BITS	0x70
#define AR_PROTEMP_DATA_LO	0xcc	/* bits[7:0] of the temperature code */
#define AR_PROTEMP_DATA_HI	0xd0	/* bit0 = bit8 of the temperature code (9-bit total) */

#define AR_PROTEMP_MAX_SAMPLES	64	/* cap for the on-stack averaging buffer */

/*
 * Linear code->Celsius conversion, recovered bit-exactly from the vendor's
 * read_raw SCALE path: temperature_C = (raw * 5320 - 1373400) / 10000, whole
 * degrees. The vendor computes the /10000 with an unsigned 64-bit reciprocal
 * (magic 0x346dc5d63886594b, >>75); that underflows to garbage for raw < 260
 * (i.e. temperature below 0 C). Plain signed integer arithmetic here is identical
 * for the normal (>= 0 C) range and degrades gracefully to a small negative
 * instead of garbage below it. [confirmed] constants, [inferred] the signed vs
 * unsigned choice below 0 C.
 */
#define AR_PROTEMP_CONV_MULT	5320
#define AR_PROTEMP_CONV_OFFSET	1373400
#define AR_PROTEMP_CONV_DIV	10000

struct ar_protemp {
	void __iomem	*regs;
	struct mutex	lock;	/* serialises the multi-sample read burst */
	u32		samp_count;
};

static int ar_protemp_cmp(const void *a, const void *b)
{
	return (int)*(const u16 *)a - (int)*(const u16 *)b;
}

/*
 * Reduce a burst of raw codes to one value. The actual "artosyn,protemp" binary
 * (artosynts_get_raw.isra.2) always takes a PLAIN mean, even at samp-count 8. We
 * diverge here for samp-count > 3 and take the trimmed mean instead (sort, drop
 * the lowest and highest eighth, mean the middle) - this is exactly what the
 * vendor's SIBLING driver "artosyn,temperature" (artosynts_partial_average) does,
 * and it rejects sample outliers better at N = 8 than the plain mean the ADC
 * driver (artosyn_adc.c) uses. Harmless when N <= 3 (falls back to plain mean).
 */
static u16 ar_protemp_reduce(u16 *buf, unsigned int n)
{
	unsigned int drop;
	unsigned int lo;
	unsigned int hi;
	unsigned int i;
	u32 sum = 0;

	if (n <= 3) {
		for (i = 0; i < n; i++)
			sum += buf[i];
		return sum / n;
	}

	sort(buf, n, sizeof(*buf), ar_protemp_cmp, NULL);

	drop = n >> 3;
	lo = drop;
	hi = n - drop;

	for (i = lo; i < hi; i++)
		sum += buf[i];
	return sum / (hi - lo);
}

/* One reading of the free-running temperature ADC; returns the averaged 9-bit code. */
static u16 ar_protemp_sample(struct ar_protemp *ts)
{
	u16 buf[AR_PROTEMP_MAX_SAMPLES];
	unsigned int n = ts->samp_count;
	unsigned int i;

	/*
	 * The temperature code free-runs: the vendor reads 0xcc/0xd0 back-to-back
	 * with no start bit, warm-up, or status poll (unlike the SAR ADC's
	 * SAMPLE_EN gate + 16 warm-up reads). We keep that exactly.
	 */
	for (i = 0; i < n; i++) {
		u32 code_lo = readl(ts->regs + AR_PROTEMP_DATA_LO) & 0xff;
		u32 code_hi = (readl(ts->regs + AR_PROTEMP_DATA_HI) & 0x1) << 8;

		buf[i] = (u16)(code_lo | code_hi);
	}

	return ar_protemp_reduce(buf, n);
}

static int ar_protemp_to_celsius(u16 raw)
{
	return ((int)raw * AR_PROTEMP_CONV_MULT - AR_PROTEMP_CONV_OFFSET) / AR_PROTEMP_CONV_DIV;
}

static int ar_protemp_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	struct ar_protemp *ts = iio_priv(indio_dev);
	u16 raw;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&ts->lock);
		*val = ar_protemp_sample(ts);
		mutex_unlock(&ts->lock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		/*
		 * Non-standard IIO semantics, kept to match the vendor and the menu:
		 * in_temp_scale returns the whole converted temperature in degrees C
		 * (NOT a scale factor). The menu reads in_temp_scale and uses it as
		 * the temperature directly.
		 */
		mutex_lock(&ts->lock);
		raw = ar_protemp_sample(ts);
		mutex_unlock(&ts->lock);
		*val = ar_protemp_to_celsius(raw);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ar_protemp_info = {
	.read_raw = ar_protemp_read_raw,
};

static const struct iio_chan_spec ar_protemp_channels[] = {
	{
		.type = IIO_TEMP,
		.indexed = 0,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
		},
	},
};

static void ar_protemp_hw_init(struct ar_protemp *ts)
{
	u32 v;

	v = readl(ts->regs + AR_PROTEMP_CTRL);
	writel(v | AR_PROTEMP_CTRL_EN, ts->regs + AR_PROTEMP_CTRL);

	v = readl(ts->regs + AR_PROTEMP_CFG);
	writel(v | AR_PROTEMP_CFG_BITS, ts->regs + AR_PROTEMP_CFG);
}

static int ar_protemp_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct ar_protemp *ts;
	struct resource *res;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*ts));
	if (!indio_dev)
		return -ENOMEM;

	ts = iio_priv(indio_dev);
	mutex_init(&ts->lock);

	/*
	 * Plain devm_ioremap (not devm_platform_ioremap_resource) so we do not
	 * request_mem_region over the ADC's overlapping, already-reserved window.
	 * Matches the vendor protemp probe.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	ts->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!ts->regs)
		return -ENOMEM;

	if (of_property_read_u32(pdev->dev.of_node, "samp-count", &ts->samp_count) ||
	    ts->samp_count == 0) {
		ts->samp_count = 1;
	}
	if (ts->samp_count > AR_PROTEMP_MAX_SAMPLES)
		ts->samp_count = AR_PROTEMP_MAX_SAMPLES;

	ar_protemp_hw_init(ts);

	/*
	 * The vendor names the device "temperature" (dev_set_name), which is where
	 * the menu expects the sysfs directory. Modern IIO forces the sysfs dir to
	 * iio:deviceN regardless, so the "name" attribute is set here and the exact
	 * path mapping is left to userspace/udev. See docs/artosyn-protemp.md [open].
	 */
	indio_dev->name = "temperature";
	indio_dev->info = &ar_protemp_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ar_protemp_channels;
	indio_dev->num_channels = ARRAY_SIZE(ar_protemp_channels);

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id ar_protemp_match[] = {
	{ .compatible = "artosyn,protemp" },
	{}
};
MODULE_DEVICE_TABLE(of, ar_protemp_match);

static struct platform_driver ar_protemp_driver = {
	.probe	= ar_protemp_probe,
	.driver = {
		.name		= "artosyn-protemp",
		.of_match_table	= ar_protemp_match,
	},
};
module_platform_driver(ar_protemp_driver);

MODULE_DESCRIPTION("Artosyn Proxima SoC temperature sensor (protemp)");
MODULE_LICENSE("GPL");
