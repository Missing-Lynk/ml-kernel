// SPDX-License-Identifier: GPL-2.0
/*
 * artosyn_adc.c - open reimplementation of the vendor "artosyn,adc" SAR ADC
 * (@0x0A108000), needed so the in-kernel adc-keys driver can read the button
 * voltage ladder on channel 0. The SoC has no mainline ADC driver, so this
 * registers an IIO provider that adc-keys consumes via io-channels.
 *
 * Register map and sample sequence recovered from the vendor driver in
 * the reconstructed vendor kernel (adc_probe / adc_read_raw / adc_get_data /
 * adc_REG_DATA_get_data / adc_REG_CALIBRATION_set_en); full writeup in
 * ../docs/artosyn-adc.md. Offsets below are the "9311" variant, selected by
 * the vendor when the reg window is <= 0x80f (this board: reg = <0xa108000 0x21c>).
 *
 * Voltage calibration follows the vendor: each channel is a 2-point linear fit between
 * the full-scale seed (code 1024 -> 2025 mV) and an on-chip 900 mV reference that the
 * channel self-measures at probe (calibration bit set, one sample -> code900). PROCESSED
 * reads interpolate between those two points, which captures the per-channel gain AND
 * offset - the offset the older gain-only SCALE dropped. RAW and SCALE are still exposed;
 * SCALE is the gain-only fallback for consumers that do not read PROCESSED, and PROCESSED
 * falls back to it if a channel's calibration comes back implausible.
 */
#define pr_fmt(fmt) "artosyn_adc: " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/iio/iio.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of.h>

#define AR_ADC_CTRL		0x00	/* bit7 = ADC enable/power */
#define AR_ADC_CTRL_EN		BIT(7)
#define AR_ADC_SAMPLE_EN	0x18	/* write 0 to begin a burst, 1 when done */
#define AR_ADC_CHAN_SEL		0x24	/* channel index 0..7 */
#define AR_ADC_CALIB		0x2c	/* bit3 = calibration enable */
#define AR_ADC_CALIB_EN		BIT(3)
#define AR_ADC_DATA_RESET	0x200	/* write 0 at init (9341 variant: 0x800) */
#define AR_ADC_DATA		0x210	/* 16-bit sample output (9341 variant: 0x810) */

#define AR_ADC_WARMUP_READS	16	/* vendor discards 16 reads before sampling */
#define AR_ADC_MAX_CHANNELS	8

/*
 * Full-scale reference in millivolts, the upper anchor (code 1024) of every
 * channel's 2-point line: measured 2113 mV; vendor nominal 2025.
 */
#define AR_ADC_FULLSCALE_MV	2113
#define AR_ADC_FULLSCALE_CODE	1024

/*
 * On-chip calibration reference in millivolts, the lower anchor of each
 * channel's 2-point line: measured 894 mV; vendor nominal 900.
 */
#define AR_ADC_CAL_REF_MV	894

struct ar_adc {
	void __iomem	*regs;
	struct mutex	lock;	/* serialises the channel-select + sample burst */
	u32		samp_count;
	u16		code_ref[AR_ADC_MAX_CHANNELS];	/* per-channel code at AR_ADC_CAL_REF_MV, 0 = uncalibrated */
};

/* One conversion of the selected channel; returns the averaged 10-bit code. */
static u16 ar_adc_sample(struct ar_adc *adc, unsigned int channel)
{
	unsigned int i;
	u32 sum = 0;

	writel(channel & 0xff, adc->regs + AR_ADC_CHAN_SEL);
	writel(0, adc->regs + AR_ADC_SAMPLE_EN);

	for (i = 0; i < AR_ADC_WARMUP_READS; i++)
		readl(adc->regs + AR_ADC_DATA);

	for (i = 0; i < adc->samp_count; i++)
		sum += readl(adc->regs + AR_ADC_DATA) & 0xffff;

	writel(1, adc->regs + AR_ADC_SAMPLE_EN);

	return sum / adc->samp_count;
}

/*
 * Calibrated millivolts for one channel from its averaged code, interpolating the
 * per-channel 2-point line: (code_ref, AR_ADC_CAL_REF_MV) .. (FULLSCALE_CODE, FULLSCALE_MV).
 * If the channel's reference code is implausible (calibration failed / not run), fall back
 * to the gain-only line through the origin, which is what SCALE reports.
 */
static int ar_adc_code_to_mv(struct ar_adc *adc, unsigned int channel, u16 code)
{
	int ref = adc->code_ref[channel];

	if (ref <= 0 || ref >= AR_ADC_FULLSCALE_CODE)
		return (int)code * AR_ADC_FULLSCALE_MV / AR_ADC_FULLSCALE_CODE;

	return AR_ADC_CAL_REF_MV +
	       DIV_ROUND_CLOSEST(((int)code - ref) * (AR_ADC_FULLSCALE_MV - AR_ADC_CAL_REF_MV),
				 AR_ADC_FULLSCALE_CODE - ref);
}

static int ar_adc_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct ar_adc *adc = iio_priv(indio_dev);
	u16 code;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&adc->lock);
		*val = ar_adc_sample(adc, chan->channel);
		mutex_unlock(&adc->lock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&adc->lock);
		code = ar_adc_sample(adc, chan->channel);
		mutex_unlock(&adc->lock);
		*val = ar_adc_code_to_mv(adc, chan->channel, code);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		/* gain-only fallback: millivolts = raw * AR_ADC_FULLSCALE_MV / AR_ADC_FULLSCALE_CODE */
		*val = AR_ADC_FULLSCALE_MV;
		*val2 = AR_ADC_FULLSCALE_CODE;
		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ar_adc_info = {
	.read_raw = ar_adc_read_raw,
};

#define AR_ADC_CHANNEL(idx) {						\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = (idx),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_PROCESSED),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
}

static const struct iio_chan_spec ar_adc_channels[AR_ADC_MAX_CHANNELS] = {
	AR_ADC_CHANNEL(0), AR_ADC_CHANNEL(1), AR_ADC_CHANNEL(2), AR_ADC_CHANNEL(3),
	AR_ADC_CHANNEL(4), AR_ADC_CHANNEL(5), AR_ADC_CHANNEL(6), AR_ADC_CHANNEL(7),
};

static void ar_adc_hw_init(struct ar_adc *adc)
{
	u32 v;

	writel(0, adc->regs + AR_ADC_DATA_RESET);

	v = readl(adc->regs + AR_ADC_CALIB);
	writel(v & ~AR_ADC_CALIB_EN, adc->regs + AR_ADC_CALIB);

	writel(1, adc->regs + AR_ADC_SAMPLE_EN);

	v = readl(adc->regs + AR_ADC_CTRL);
	writel(v | AR_ADC_CTRL_EN, adc->regs + AR_ADC_CTRL);
}

/*
 * Self-measure each channel against the on-chip AR_ADC_CAL_REF_MV reference (vendor probe
 * step 7): with the calibration bit set, one sample of the channel returns the code that
 * corresponds to the reference voltage. Stored per channel and paired with the full-scale
 * seed to give PROCESSED its 2-point line. Run once at probe, ADC already enabled.
 */
static void ar_adc_calibrate(struct ar_adc *adc, u32 channels)
{
	unsigned int ch;
	u32 v;

	for (ch = 0; ch < channels; ch++) {
		mutex_lock(&adc->lock);

		v = readl(adc->regs + AR_ADC_CALIB);
		writel(v | AR_ADC_CALIB_EN, adc->regs + AR_ADC_CALIB);

		adc->code_ref[ch] = ar_adc_sample(adc, ch);

		v = readl(adc->regs + AR_ADC_CALIB);
		writel(v & ~AR_ADC_CALIB_EN, adc->regs + AR_ADC_CALIB);

		mutex_unlock(&adc->lock);

		pr_info("channel %u reference code %u (%u mV)\n",
			ch, adc->code_ref[ch], AR_ADC_CAL_REF_MV);
	}
}

static int ar_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct ar_adc *adc;
	u32 channels = AR_ADC_MAX_CHANNELS;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	mutex_init(&adc->lock);

	adc->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->regs))
		return PTR_ERR(adc->regs);

	if (of_property_read_u32(pdev->dev.of_node, "samp-count", &adc->samp_count) ||
	    adc->samp_count == 0)
		adc->samp_count = 1;

	of_property_read_u32(pdev->dev.of_node, "channels", &channels);
	if (channels == 0 || channels > AR_ADC_MAX_CHANNELS)
		channels = AR_ADC_MAX_CHANNELS;

	ar_adc_hw_init(adc);
	ar_adc_calibrate(adc, channels);

	indio_dev->name = "artosyn-adc";
	indio_dev->info = &ar_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ar_adc_channels;
	indio_dev->num_channels = channels;

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id ar_adc_match[] = {
	{ .compatible = "artosyn,adc" },
	{}
};
MODULE_DEVICE_TABLE(of, ar_adc_match);

static struct platform_driver ar_adc_driver = {
	.probe	= ar_adc_probe,
	.driver = {
		.name		= "artosyn-adc",
		.of_match_table	= ar_adc_match,
	},
};
module_platform_driver(ar_adc_driver);

MODULE_DESCRIPTION("Artosyn Proxima SAR ADC (button voltage ladder)");
MODULE_LICENSE("GPL");
