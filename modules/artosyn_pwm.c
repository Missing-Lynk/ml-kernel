// SPDX-License-Identifier: GPL-2.0
/*
 * artosyn_pwm.c - open reimplementation of the vendor "artosyn,ar9301-pwm" controller
 * (@0x01000000 and @0x01002000). Drives the goggle's LCD backlight (pwmchip0 channel 1)
 * and the buzzer; no mainline driver exists. Register map recovered from the vendor kernel's
 * artosyn_pwm_apply / artosyn_pwm_get_state (the reconstructed vendor kernel).
 *
 * Per channel N the controller programs the OFF (low) and ON (duty/high) durations as
 * separate clock-cycle counts; the period is OFF+ON. The two counts live in different
 * register regions:
 *   OFF count  @ base + N*0x14 + 0x00   (period - duty, in clk cycles)
 *   CTRL       @ base + N*0x14 + 0x08   (enable = set bits 0,1,3 -> mask 0xb)
 *   ON count   @ base + 0xb0 + N*0x04   (duty, in clk cycles)
 * count = round(time_ns * clk_rate / 1e9). The functional clock is host_ref (150 MHz).
 */
#define pr_fmt(fmt) "artosyn_pwm: " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/math64.h>

#define AR_PWM_CH_STRIDE	0x14
#define AR_PWM_OFF(n)		((n) * AR_PWM_CH_STRIDE + 0x00)	/* low (period-duty) count */
#define AR_PWM_CTRL(n)		((n) * AR_PWM_CH_STRIDE + 0x08)
#define AR_PWM_DUTY(n)		(0xb0 + (n) * 0x04)		/* high (duty) count */
#define AR_PWM_CTRL_EN		0xb				/* bits 0,1,3 */

#define AR_PWM_NPWM		8

struct ar_pwm {
	void __iomem	*base;
	struct clk	*clk;
};

static inline struct ar_pwm *to_ar_pwm(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

/* nanoseconds -> clock cycles, round to nearest (matches the vendor's +5e8 / 1e9). */
static u32 ar_pwm_ns_to_cycles(unsigned long rate, u64 ns)
{
	return (u32)DIV_ROUND_CLOSEST_ULL(ns * rate, NSEC_PER_SEC);
}

static int ar_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			const struct pwm_state *state)
{
	struct ar_pwm *ap = to_ar_pwm(chip);
	unsigned int n = pwm->hwpwm;
	unsigned long rate = clk_get_rate(ap->clk);
	u64 duty = min(state->duty_cycle, state->period);
	u32 ctrl;

	writel(ar_pwm_ns_to_cycles(rate, state->period - duty), ap->base + AR_PWM_OFF(n));
	writel(ar_pwm_ns_to_cycles(rate, duty), ap->base + AR_PWM_DUTY(n));

	ctrl = readl(ap->base + AR_PWM_CTRL(n));
	if (state->enabled)
		ctrl |= AR_PWM_CTRL_EN;
	else
		ctrl &= ~AR_PWM_CTRL_EN;
	writel(ctrl, ap->base + AR_PWM_CTRL(n));

	return 0;
}

static int ar_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			    struct pwm_state *state)
{
	struct ar_pwm *ap = to_ar_pwm(chip);
	unsigned int n = pwm->hwpwm;
	unsigned long rate = clk_get_rate(ap->clk);
	u32 off, on;

	if (!rate)
		return -EINVAL;

	off = readl(ap->base + AR_PWM_OFF(n));
	on  = readl(ap->base + AR_PWM_DUTY(n));

	state->period = DIV_ROUND_CLOSEST_ULL((u64)(off + on) * NSEC_PER_SEC, rate);
	state->duty_cycle = DIV_ROUND_CLOSEST_ULL((u64)on * NSEC_PER_SEC, rate);
	state->polarity = PWM_POLARITY_NORMAL;
	state->enabled = !!(readl(ap->base + AR_PWM_CTRL(n)) & AR_PWM_CTRL_EN);

	return 0;
}

static const struct pwm_ops ar_pwm_ops = {
	.apply		= ar_pwm_apply,
	.get_state	= ar_pwm_get_state,
};

static int ar_pwm_probe(struct platform_device *pdev)
{
	struct pwm_chip *chip;
	struct ar_pwm *ap;
	int ret;

	chip = devm_pwmchip_alloc(&pdev->dev, AR_PWM_NPWM, sizeof(*ap));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	ap = to_ar_pwm(chip);

	ap->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ap->base))
		return PTR_ERR(ap->base);

	ap->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(ap->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(ap->clk), "no functional clock\n");

	chip->ops = &ar_pwm_ops;

	ret = devm_pwmchip_add(&pdev->dev, chip);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to add pwmchip\n");

	return 0;
}

static const struct of_device_id ar_pwm_match[] = {
	{ .compatible = "artosyn,ar9301-pwm" },
	{}
};
MODULE_DEVICE_TABLE(of, ar_pwm_match);

static struct platform_driver ar_pwm_driver = {
	.probe	= ar_pwm_probe,
	.driver = {
		.name		= "artosyn-pwm",
		.of_match_table	= ar_pwm_match,
	},
};
module_platform_driver(ar_pwm_driver);

MODULE_DESCRIPTION("Artosyn Proxima ar9301 PWM (backlight/buzzer)");
MODULE_LICENSE("GPL");
