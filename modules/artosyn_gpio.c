// SPDX-License-Identifier: GPL-2.0
/*
 * artosyn_gpio.c - open reimplementation of the vendor "artosyn,gpio" controller
 * (@0x0A10A000), needed to drive the AR8030 RF reset line (GPIO23). The SoC has no
 * mainline gpio driver; ad-hoc /dev/mem pokes don't work because the direction
 * register is effectively write-only/unreadable. This registers proper gpiochips so
 * GPIO23 can be driven via gpiolib (the vendor's start_ar813x.sh reset pulse).
 *
 * Register map recovered from artosyn_gpio_probe (bgpio_init args): the controller has 7
 * banks; per bank N the register block is at base + 0xBC + N*0xC, with
 *   SET   @+0  : output-value latch (read/write, RMW)
 *   DIRIN @+4  : direction, 1 = input  (write-only on this SoC -> we shadow it)
 *   DAT   @+8  : input read
 * NOTE: base offset is 0xBC (vendor of_match data[0], confirmed by disasm).
 * (0x00..0x3c is pinmux, owned by the bootloader.) Bank gpio bases/sizes match the
 * vendor numbering so sysfs/DT gpio 23 == the AR8030 reset, as the vendor scripts use.
 *
 * Loads as a module: gpiochip_add_data_with_key + devm_gpiochip_add_data_with_key are
 * exported (bgpio_init is not, so we use minimal custom ops). See ../tools/.
 */
#define pr_fmt(fmt) "artosyn_gpio: " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define AR_BANK_REG_BASE	0xBC
#define AR_BANK_STRIDE		0xC
#define AR_REG_SET		0x0	/* output-value latch */
#define AR_REG_DIRIN		0x4	/* 1 = input */
#define AR_REG_DAT		0x8	/* input read */

struct ar_gpio_bank {
	struct gpio_chip	gc;
	void __iomem		*regs;		/* base + 0xBC + N*0xC */
	spinlock_t		lock;
	u32			dir_shadow;	/* DIRIN is unreadable; mirror it */
	u32			out_shadow;	/* SET readback mixes in input-pin
						 * state, so a read-modify-write
						 * corrupts other pins; shadow the
						 * output latch like gpio-mmio does.
						 */
};

/* gpio number base + count per bank, matching the vendor numbering. */
static const struct {
	unsigned int base;
	unsigned int ngpio;
} ar_banks[] = {
	{ 0, 23 }, { 23, 22 }, { 45, 26 }, { 71, 6 }, { 77, 6 }, { 83, 11 }, { 94, 16 },
};

static int ar_gpio_get(struct gpio_chip *gc, unsigned int off)
{
	struct ar_gpio_bank *b = gpiochip_get_data(gc);

	return !!(readl(b->regs + AR_REG_DAT) & BIT(off));
}

static int ar_gpio_set(struct gpio_chip *gc, unsigned int off, int val)
{
	struct ar_gpio_bank *b = gpiochip_get_data(gc);
	unsigned long flags;

	spin_lock_irqsave(&b->lock, flags);
	/* Drive from the shadow, never read-modify-write the SET register (its
	 * readback mixes in input-pin levels and would clobber other outputs).
	 */
	if (val)
		b->out_shadow |= BIT(off);
	else
		b->out_shadow &= ~BIT(off);
	writel(b->out_shadow, b->regs + AR_REG_SET);
	spin_unlock_irqrestore(&b->lock, flags);
	return 0;
}

static int ar_gpio_dir_in(struct gpio_chip *gc, unsigned int off)
{
	struct ar_gpio_bank *b = gpiochip_get_data(gc);
	unsigned long flags;

	spin_lock_irqsave(&b->lock, flags);
	b->dir_shadow |= BIT(off);
	writel(b->dir_shadow, b->regs + AR_REG_DIRIN);
	spin_unlock_irqrestore(&b->lock, flags);
	return 0;
}

static int ar_gpio_dir_out(struct gpio_chip *gc, unsigned int off, int val)
{
	struct ar_gpio_bank *b = gpiochip_get_data(gc);
	unsigned long flags;

	ar_gpio_set(gc, off, val);
	spin_lock_irqsave(&b->lock, flags);
	b->dir_shadow &= ~BIT(off);
	writel(b->dir_shadow, b->regs + AR_REG_DIRIN);
	spin_unlock_irqrestore(&b->lock, flags);
	return 0;
}

static int ar_gpio_get_dir(struct gpio_chip *gc, unsigned int off)
{
	struct ar_gpio_bank *b = gpiochip_get_data(gc);

	/* DIRIN is write-only here; report from the shadow (1 = input). */
	return (b->dir_shadow & BIT(off)) ?
		GPIO_LINE_DIRECTION_IN : GPIO_LINE_DIRECTION_OUT;
}

/*
 * All 7 banks share the single "artosyn,gpio" DT node, so a DT reference <&gpio N flags>
 * passes the vendor GLOBAL line number N; the bank whose [base, base+ngpio) covers N
 * claims it (gpiolib tries every bank's of_xlate until one returns >= 0). This lets the
 * panel/RF nodes address lines by the same numbering the vendor scripts use (e.g. 43, 95).
 */
static int ar_gpio_of_xlate(struct gpio_chip *gc,
			    const struct of_phandle_args *spec, u32 *flags)
{
	unsigned int num = spec->args[0];

	if (num < gc->base || num >= gc->base + gc->ngpio)
		return -EINVAL;
	if (flags)
		*flags = spec->args[1];
	return num - gc->base;
}

static int ar_gpio_probe(struct platform_device *pdev)
{
	void __iomem *base;
	unsigned int i;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	for (i = 0; i < ARRAY_SIZE(ar_banks); i++) {
		struct ar_gpio_bank *b;

		b = devm_kzalloc(&pdev->dev, sizeof(*b), GFP_KERNEL);
		if (!b)
			return -ENOMEM;

		b->regs = base + AR_BANK_REG_BASE + i * AR_BANK_STRIDE;
		spin_lock_init(&b->lock);
		/* DIRIN is unreadable; seed the shadow as all-input (safe default). */
		b->dir_shadow = 0xffffffff;
		/* Seed the output shadow from the current latch so the first write
		 * preserves pins the bootloader/regulators already drive.
		 */
		b->out_shadow = readl(b->regs + AR_REG_SET);

		b->gc.label = devm_kasprintf(&pdev->dev, GFP_KERNEL, "ar-gpio%u", i);
		b->gc.parent = &pdev->dev;
		b->gc.owner = THIS_MODULE;
		b->gc.base = ar_banks[i].base;
		b->gc.ngpio = ar_banks[i].ngpio;
		b->gc.get = ar_gpio_get;
		b->gc.set = ar_gpio_set;
		b->gc.direction_input = ar_gpio_dir_in;
		b->gc.direction_output = ar_gpio_dir_out;
		b->gc.get_direction = ar_gpio_get_dir;
		b->gc.of_xlate = ar_gpio_of_xlate;
		b->gc.of_gpio_n_cells = 2;
		b->gc.can_sleep = false;

		if (devm_gpiochip_add_data(&pdev->dev, &b->gc, b)) {
			dev_err(&pdev->dev, "failed to add gpiochip %u\n", i);
			return -ENODEV;
		}
	}

	dev_info(&pdev->dev, "registered %zu gpio banks (0-109)\n", ARRAY_SIZE(ar_banks));
	return 0;
}

static const struct of_device_id ar_gpio_match[] = {
	{ .compatible = "artosyn,gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, ar_gpio_match);

static struct platform_driver ar_gpio_driver = {
	.probe	= ar_gpio_probe,
	.driver = {
		.name		= "artosyn-gpio",
		.of_match_table	= ar_gpio_match,
	},
};
module_platform_driver(ar_gpio_driver);

MODULE_DESCRIPTION("Artosyn Proxima GPIO controller");
MODULE_LICENSE("GPL");
