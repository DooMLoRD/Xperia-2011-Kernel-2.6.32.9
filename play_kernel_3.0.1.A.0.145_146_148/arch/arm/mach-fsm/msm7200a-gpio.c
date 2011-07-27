/*
 * Driver for Qualcomm MSM7200a and related SoC GPIO.
 * Supported chipset families include:
 * MSM7x01(a), MSM7x25, MSM7x27, MSM7x30, QSD8x50(a)
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "msm7200a-gpio.h"

enum {
	IRQ_MASK_NORMAL = 0,
	IRQ_MASK_WAKE_ON,
	IRQ_MASK_MAX
};

struct msm_gpio_dev {
	struct gpio_chip		gpio_chip;
	spinlock_t			lock;
	unsigned			irq_base;
	unsigned			irq_summary;
	bool				latch_level_irqs;
	struct msm7200a_gpio_regs	regs;
	u32				irq_masks[IRQ_MASK_MAX];
	u32				dual_edge_irq_ena;
	int				nsuspend;
};

static inline struct msm_gpio_dev *to_msm_gpio_dev(struct gpio_chip *chip)
{
	return container_of(chip, struct msm_gpio_dev, gpio_chip);
}

/*
 * This function assumes that msm_gpio_dev::lock is held.
 */
static inline void set_gpio_bit(unsigned n, void __iomem *reg)
{
	writel(readl(reg) | BIT(n), reg);
}

/*
 * This function assumes that msm_gpio_dev::lock is held.
 */
static inline void clr_gpio_bit(unsigned n, void __iomem *reg)
{
	writel(readl(reg) & ~BIT(n), reg);
}

/*
 * This function assumes that msm_gpio_dev::lock is held.
 */
static inline void
msm_gpio_write(struct msm_gpio_dev *dev, unsigned n, unsigned on)
{
	if (on)
		set_gpio_bit(n, dev->regs.out);
	else
		clr_gpio_bit(n, dev->regs.out);
}

static int gpio_chip_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct msm_gpio_dev *msm_gpio = to_msm_gpio_dev(chip);
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	clr_gpio_bit(offset, msm_gpio->regs.oe);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return 0;
}

static int
gpio_chip_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct msm_gpio_dev *msm_gpio = to_msm_gpio_dev(chip);
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	msm_gpio_write(msm_gpio, offset, value);
	set_gpio_bit(offset, msm_gpio->regs.oe);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return 0;
}

static int gpio_chip_get(struct gpio_chip *chip, unsigned offset)
{
	struct msm_gpio_dev *msm_gpio = to_msm_gpio_dev(chip);

	return readl(msm_gpio->regs.in) & BIT(offset) ? 1 : 0;
}

static void gpio_chip_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct msm_gpio_dev *msm_gpio = to_msm_gpio_dev(chip);
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	msm_gpio_write(msm_gpio, offset, value);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);
}

static int gpio_chip_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct msm_gpio_dev *msm_gpio = to_msm_gpio_dev(chip);
	return msm_gpio->irq_base + offset;
}

static void forget_level_irq(struct msm_gpio_dev *msm_gpio, unsigned offset)
{
	if (!msm_gpio->latch_level_irqs) {
		unsigned v = readl(msm_gpio->regs.int_edge);
		unsigned b = BIT(offset);

		if (!(v & b))
			writel(b, msm_gpio->regs.int_clear);
	}
}

static void msm_gpio_irq_mask(unsigned int irq)
{
	unsigned long irq_flags;
	struct msm_gpio_dev *msm_gpio = get_irq_chip_data(irq);
	unsigned offset = irq - msm_gpio->irq_base;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	forget_level_irq(msm_gpio, offset);
	msm_gpio->irq_masks[IRQ_MASK_NORMAL] &= ~BIT(offset);
	writel(msm_gpio->irq_masks[IRQ_MASK_NORMAL], msm_gpio->regs.int_en);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);
}

static void msm_gpio_irq_unmask(unsigned int irq)
{
	unsigned long irq_flags;
	struct msm_gpio_dev *msm_gpio = get_irq_chip_data(irq);
	unsigned offset = irq - msm_gpio->irq_base;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	forget_level_irq(msm_gpio, offset);
	msm_gpio->irq_masks[IRQ_MASK_NORMAL] |= BIT(offset);
	writel(msm_gpio->irq_masks[IRQ_MASK_NORMAL], msm_gpio->regs.int_en);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);
}

/* For dual-edge interrupts in software, since the hardware has no
 * such support:
 *
 * At appropriate moments, this function may be called to flip the polarity
 * settings of both-edge irq lines to try and catch the next edge.
 *
 * The attempt is considered successful if:
 * 1. the input value of the gpio doesn't change during the attempt, or
 * 2. the status bit goes high, indicating that an edge was caught.
 * If the value changes twice during the process, that would cause the first
 * test to fail (falsely) but would force the second, as two opposite
 * transitions would cause a detection no matter the polarity setting.
 *
 * The do-loop tries to sledge-hammer closed the timing hole between
 * the initial value-read and the polarity-write - if the line value changes
 * during that window, an interrupt is lost, the new polarity setting is
 * incorrect, and the first success test will fail.
 */
static void update_dual_edge_pos(struct msm_gpio_dev *msm_gpio)
{
	int loop_limit = 100;
	unsigned pol, val, val2, intstat;

	do {
		pol = readl(msm_gpio->regs.int_pos);
		val = readl(msm_gpio->regs.in);
		pol = (pol & ~msm_gpio->dual_edge_irq_ena) |
		      (~val & msm_gpio->dual_edge_irq_ena);
		writel(pol, msm_gpio->regs.int_pos);
		intstat = readl(msm_gpio->regs.int_status);
		val2 = readl(msm_gpio->regs.in);
		if (!((val ^ val2) & msm_gpio->dual_edge_irq_ena & ~intstat))
			return;
	} while (loop_limit-- > 0);
	pr_err("%s: dual-edge irq emulation failed to stabilize, "
	       "interrupts will be dropped. %08x != %08x\n",
	       __func__, val, val2);
}

static int msm_gpio_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	unsigned long irq_flags;
	struct msm_gpio_dev *msm_gpio = get_irq_chip_data(irq);
	unsigned offset = irq - msm_gpio->irq_base;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);

	if (flow_type & IRQ_TYPE_EDGE_BOTH) {
		set_gpio_bit(offset, msm_gpio->regs.int_edge);
		irq_desc[irq].handle_irq = handle_edge_irq;

		if ((flow_type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH) {
			msm_gpio->dual_edge_irq_ena |= BIT(offset);
			update_dual_edge_pos(msm_gpio);
		} else {
			msm_gpio->dual_edge_irq_ena &= ~BIT(offset);
			if (flow_type & IRQF_TRIGGER_RISING)
				set_gpio_bit(offset, msm_gpio->regs.int_pos);
			else
				clr_gpio_bit(offset, msm_gpio->regs.int_pos);
		}
	} else {
		clr_gpio_bit(offset, msm_gpio->regs.int_edge);
		irq_desc[irq].handle_irq = handle_level_irq;

		msm_gpio->dual_edge_irq_ena &= ~BIT(offset);
		if (flow_type & IRQF_TRIGGER_HIGH)
			set_gpio_bit(offset, msm_gpio->regs.int_pos);
		else
			clr_gpio_bit(offset, msm_gpio->regs.int_pos);
	}

	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return 0;
}

static void msm_gpio_irq_mask_ack(unsigned int irq)
{
	msm_gpio_irq_mask(irq);
}

static int msm_gpio_irq_set_wake(unsigned int irq, unsigned int on)
{
	struct msm_gpio_dev *msm_gpio = get_irq_chip_data(irq);
	unsigned offset = irq - msm_gpio->irq_base;
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	if (on)
		msm_gpio->irq_masks[IRQ_MASK_WAKE_ON] |= BIT(offset);
	else
		msm_gpio->irq_masks[IRQ_MASK_WAKE_ON] &= ~BIT(offset);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return set_irq_wake(msm_gpio->irq_summary, on);
}

static irqreturn_t msm_gpio_irq_handler(int irq, void *dev)
{
	struct msm_gpio_dev *msm_gpio = dev;
	unsigned e, s, triggered_irqs;
	int b;

	/*
	 * The int_status register latches trigger events whether or not
	 * the gpio line is enabled as an interrupt source.  Therefore,
	 * the set of pins which defines the interrupts which need to fire
	 * is the intersection of int_status and int_en - int_status
	 * alone provides an incomplete picture.
	 */
	spin_lock(&msm_gpio->lock);
	s = readl(msm_gpio->regs.int_status);
	e = readl(msm_gpio->regs.int_en);
	triggered_irqs = s & e;
	if (triggered_irqs) {
		writel(triggered_irqs, msm_gpio->regs.int_clear);
		update_dual_edge_pos(msm_gpio);
	}
	spin_unlock(&msm_gpio->lock);

	if (!triggered_irqs)
		return IRQ_NONE;

	while (triggered_irqs) {
		b = ffs(triggered_irqs) - 1;
		triggered_irqs &= ~BIT(b);
		generic_handle_irq(msm_gpio->irq_base + b);
	}
	return IRQ_HANDLED;
}

static struct irq_chip msm_gpio_irq_chip = {
	.name			= "msm_gpio",
	.mask			= msm_gpio_irq_mask,
	.mask_ack		= msm_gpio_irq_mask_ack,
	.unmask			= msm_gpio_irq_unmask,
	.set_type		= msm_gpio_irq_set_type,
	.set_wake		= msm_gpio_irq_set_wake,
};

static int __devinit msm_gpio_probe(struct platform_device *dev)
{
	struct msm_gpio_dev *msm_gpio;
	struct msm7200a_gpio_platform_data *pdata = dev->dev.platform_data;
	int i, irq, ret;

	if (!pdata)
		return -EINVAL;

	msm_gpio = kzalloc(sizeof(struct msm_gpio_dev), GFP_KERNEL);
	if (!msm_gpio)
		return -ENOMEM;

	spin_lock_init(&msm_gpio->lock);
	platform_set_drvdata(dev, msm_gpio);

	msm_gpio->regs                       = pdata->regs;
	msm_gpio->gpio_chip.label            = dev->name;
	msm_gpio->gpio_chip.base             = pdata->gpio_base;
	msm_gpio->gpio_chip.ngpio            = pdata->ngpio;
	msm_gpio->gpio_chip.direction_input  = gpio_chip_direction_input;
	msm_gpio->gpio_chip.direction_output = gpio_chip_direction_output;
	msm_gpio->gpio_chip.get              = gpio_chip_get;
	msm_gpio->gpio_chip.set              = gpio_chip_set;
	msm_gpio->gpio_chip.to_irq           = gpio_chip_to_irq;
	msm_gpio->irq_base                   = pdata->irq_base;
	msm_gpio->irq_summary                = pdata->irq_summary;
	msm_gpio->latch_level_irqs           = pdata->latch_level_irqs;

	ret = gpiochip_add(&msm_gpio->gpio_chip);
	if (ret < 0)
		goto err_post_malloc;

	for (i = 0; i < msm_gpio->gpio_chip.ngpio; ++i) {
		irq = msm_gpio->irq_base + i;
		set_irq_chip_data(irq, msm_gpio);
		set_irq_chip(irq, &msm_gpio_irq_chip);
		set_irq_handler(irq, handle_level_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	/*
	 * We use a level-triggered interrupt because of the nature
	 * of the shared GPIO-group interrupt.
	 *
	 * Many GPIO chips may be sharing the same group IRQ line, and
	 * it is possible for GPIO interrupt to re-occur while the system
	 * is still servicing the group interrupt associated with it.
	 * The group IRQ line would not de-assert and re-assert, and
	 * we'd get no second edge to cause the group IRQ to be handled again.
	 *
	 * Using a level interrupt guarantees that the group IRQ handlers
	 * will continue to be called as long as any GPIO chip in the group
	 * is asserting, even if the condition began while the group
	 * handler was in mid-pass.
	 */
	ret = request_irq(msm_gpio->irq_summary,
			  msm_gpio_irq_handler,
			  IRQF_SHARED | IRQF_TRIGGER_HIGH,
			  dev_name(&dev->dev),
			  msm_gpio);
	if (ret < 0)
		goto err_post_gpiochip_add;

	return ret;
err_post_gpiochip_add:
	/*
	 * Under no circumstances should a line be held on a gpiochip
	 * which hasn't finished probing.
	 */
	BUG_ON(gpiochip_remove(&msm_gpio->gpio_chip) < 0);
err_post_malloc:
	platform_set_drvdata(dev, NULL);
	kfree(msm_gpio);
	return ret;
}

static int __devexit msm_gpio_remove(struct platform_device *dev)
{
	struct msm_gpio_dev *msm_gpio = platform_get_drvdata(dev);
	int ret = gpiochip_remove(&msm_gpio->gpio_chip);

	if (ret < 0)
		return ret;

	free_irq(msm_gpio->irq_summary, msm_gpio);
	kfree(msm_gpio);

	return 0;
}

#ifdef CONFIG_PM
static int msm_gpio_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_gpio_dev *msm_gpio = platform_get_drvdata(pdev);
	unsigned long irq_flags;
	unsigned long irq_mask;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	if ((msm_gpio->nsuspend)++ == 0) {
		irq_mask = msm_gpio->irq_masks[IRQ_MASK_NORMAL] &
			   msm_gpio->irq_masks[IRQ_MASK_WAKE_ON];
		writel(irq_mask, msm_gpio->regs.int_en);
	}
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return 0;
}

static int msm_gpio_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_gpio_dev *msm_gpio = platform_get_drvdata(pdev);
	unsigned long irq_flags;

	spin_lock_irqsave(&msm_gpio->lock, irq_flags);
	if (--(msm_gpio->nsuspend) == 0)
		writel(msm_gpio->irq_masks[IRQ_MASK_NORMAL],
		       msm_gpio->regs.int_en);
	spin_unlock_irqrestore(&msm_gpio->lock, irq_flags);

	return 0;
}
#else
#define msm_gpio_suspend NULL
#define msm_gpio_resume NULL
#endif

static SIMPLE_DEV_PM_OPS(msm_gpio_pm_ops, msm_gpio_suspend, msm_gpio_resume);

static struct platform_driver msm_gpio_driver = {
	.probe = msm_gpio_probe,
	.remove = __devexit_p(msm_gpio_remove),
	.driver = {
		.name = "msm7200a-gpio",
		.owner = THIS_MODULE,
		.pm = &msm_gpio_pm_ops,
	},
};

static int __init msm_gpio_init(void)
{
	return platform_driver_register(&msm_gpio_driver);
}

static void __exit msm_gpio_exit(void)
{
	platform_driver_unregister(&msm_gpio_driver);
}

postcore_initcall(msm_gpio_init);
module_exit(msm_gpio_exit);

MODULE_AUTHOR("Gregory Bean <gbean@codeaurora.org>");
MODULE_DESCRIPTION("Driver for Qualcomm MSM 7200a-family SoC GPIOs");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msm7200a-gpio");
