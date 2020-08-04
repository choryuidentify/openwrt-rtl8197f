/*
 *  Realtek GPIO Driver support
 *  Based on Atheros AR71XX GPIO API support
 *
 *  Copyright (C) 2019 Gaspare Bruno <gaspare@anlix.io>
 *  Copyright (C) 2015 Alban Bedel <albeu@free.fr>
 *  Copyright (C) 2010-2011 Jaiganesh Narayanan <jnarayanan@atheros.com>
 *  Copyright (C) 2008-2011 Gabor Juhos <juhosg@openwrt.org>
 *  Copyright (C) 2008 Imre Kaloz <kaloz@openwrt.org>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <linux/gpio/driver.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/irq.h>

struct realtek_gpio_ctrl {
	struct gpio_chip gc;
	void __iomem *base;
	char gpio_pin_int_masks[32];
};

static u32 realtek_gpio_read(struct realtek_gpio_ctrl *ctrl, unsigned reg)
{
	return readl(ctrl->base + reg);
}

static void realtek_gpio_write(struct realtek_gpio_ctrl *ctrl, unsigned reg, u32 val)
{
	return writel(val, ctrl->base + reg);
}

static int realtek_gpio_request(struct gpio_chip *chip, unsigned gpio_pin)
{
	unsigned long flags;
	unsigned long pinmask;
	struct realtek_gpio_ctrl *ctrl = container_of(chip, struct realtek_gpio_ctrl, gc);

	if (gpio_pin >= chip->ngpio)
		return -EINVAL;

	spin_lock_irqsave(&chip->bgpio_lock, flags);
	pinmask = realtek_gpio_read(ctrl, 0x0);
	pinmask &= ~(BIT(gpio_pin));
	realtek_gpio_write(ctrl, 0x0, pinmask);
	spin_unlock_irqrestore(&chip->bgpio_lock, flags);
	return 0;
}

static void realtek_gpio_irq_mask(struct irq_data *data)
{
	unsigned int irq = data->irq;
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct realtek_gpio_ctrl *ctrl = container_of(gc, struct realtek_gpio_ctrl, gc);
	unsigned int pin = irq_linear_revmap(gc->irqdomain, irq);
	unsigned int pinmask = 0;
	unsigned int pinreg = 0;
	u32 mask;

	if(pin<16){
		pinreg = 0x14; // Interrupt Mask (0-15)
		pinmask = pin;
	}
	else {
		pinreg = 0x18; // Interrupt Mask (16-31)
		pinmask = pin-16; 
	}

	mask = realtek_gpio_read(ctrl, pinreg);

	mask |= (ctrl->gpio_pin_int_masks[pin] << pinmask);

	realtek_gpio_write(ctrl, pinreg, mask);
}

static void realtek_gpio_irq_unmask(struct irq_data *data)
{
	unsigned int irq = data->irq;
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct realtek_gpio_ctrl *ctrl = container_of(gc, struct realtek_gpio_ctrl, gc);
	unsigned int pin = irq_linear_revmap(gc->irqdomain, irq);
	unsigned int pinmask = 0;
	unsigned int pinreg = 0;
	u32 mask;

	if(pin<16){
		pinreg = 0x14; // Interrupt Mask (0-15)
		pinmask = pin;
	}
	else {
		pinreg = 0x18; // Interrupt Mask (16-31)
		pinmask = pin-16; 
	}

	mask = realtek_gpio_read(ctrl, pinreg);

	ctrl->gpio_pin_int_masks[pin] = (mask >> pinmask) & 0x3;
	mask &= ~(0x3 << pinmask);

	realtek_gpio_write(ctrl, pinreg, mask);
}

static int realtek_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	unsigned int irq = data->irq;
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct realtek_gpio_ctrl *ctrl = container_of(gc, struct realtek_gpio_ctrl, gc);
	unsigned int pin = irq_linear_revmap(gc->irqdomain, irq);
	unsigned int pinmask = 0;
	unsigned int pinreg = 0;
	unsigned int newvalue = 0;
	u32 mask;

	if(pin<16){
		pinreg = 0x14; // Interrupt Mask (0-15)
		pinmask = pin;
	}
	else {
		pinreg = 0x18; // Interrupt Mask (16-31)
		pinmask = pin-16; 
	}

	mask = realtek_gpio_read(ctrl, pinreg);

	switch(type) {
		case IRQ_TYPE_EDGE_RISING:
			newvalue = 0x2;
			break;
		case IRQ_TYPE_EDGE_FALLING:
			newvalue = 0x1;
			break;
		case IRQ_TYPE_EDGE_BOTH:
			newvalue = 0x3;
			break;
		default:
			printk(KERN_ERR "No such irq type %d", type);
			return -EINVAL;
	}

	ctrl->gpio_pin_int_masks[pin] = newvalue;
	mask &= ~(0x3 << (pinmask*2));
	mask |= (newvalue << (pinmask*2));

	realtek_gpio_write(ctrl, pinreg, mask);

	return 0;
}

static struct irq_chip realtek_gpio_irqchip = {
	.name = "gpio-realtek",
//	.irq_enable = realtek_gpio_irq_enable,
//	.irq_disable = realtek_gpio_irq_disable,
	.irq_mask = realtek_gpio_irq_mask,
	.irq_unmask = realtek_gpio_irq_unmask,
	.irq_set_type = realtek_gpio_irq_set_type,
	.flags = IRQCHIP_SET_TYPE_MASKED,
};

static void realtek_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	struct realtek_gpio_ctrl *ctrl = container_of(gc, struct realtek_gpio_ctrl, gc);
	u32 status, mask1, mask2, mask3 = 0;
	unsigned long pending;
	int irq, i;

	chained_irq_enter(irqchip, desc);

	status = realtek_gpio_read(ctrl, 0x10); // Interrupt Status
	mask1  = realtek_gpio_read(ctrl, 0x14); // Interrupt Mask (0-15)
	mask2  = realtek_gpio_read(ctrl, 0x18); // Interrupt Mask (16-31)

	/* get the mask for interrupt status register by ourself */
	for(i = 0; i < 16; i++) 
		mask3 |= ((0x3 << (i*2)) & mask1 ? 1 : 0) << i;
	for(i = 16; i < 32; i++) 
		mask3 |= ((0x3 << ((i-16)*2)) & mask2 ? 1 : 0) << i;
	
	/* mask the pins which don't have interrupt */
	pending = status & mask3;

	if (pending) {
		for_each_set_bit(irq, &pending, 32)
			generic_handle_irq(irq_linear_revmap(gc->irqdomain, irq));
	}

	chained_irq_exit(irqchip, desc);
}

static const struct of_device_id realtek_gpio_of_match[] = {
	{ .compatible = "realtek,realtek-gpio" },
	{ .compatible = "realtek,rtl8197f-gpio" },
	{},
};
MODULE_DEVICE_TABLE(of, realtek_gpio_of_match);

static int realtek_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct realtek_gpio_ctrl *ctrl;
	struct resource *res;
	int err, irq=0;

	ctrl = devm_kzalloc(&pdev->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	platform_set_drvdata(pdev, ctrl);

	if (!np) {
		dev_err(&pdev->dev, "No DT node or platform data found\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctrl->base = devm_ioremap_nocache(&pdev->dev, res->start, resource_size(res));
	if (!ctrl->base)
		return -ENOMEM;

	err = bgpio_init(&ctrl->gc, &pdev->dev, 4,
		ctrl->base + 0x0C,	// DAT Register
		NULL,
		NULL,
		ctrl->base + 0x08,	// DIRECTION OUT
		NULL,				// DIRECTION IN
		0);
	if (err) {
		dev_err(&pdev->dev, "bgpio_init failed\n");
		return err;
	}

	ctrl->gc.request = realtek_gpio_request;
	if(np && of_property_read_bool(np, "base"))
	{
		u8 base=0;
		of_property_read_u8(np, "base", &base);
		ctrl->gc.base = base;
	} else {
		// use autodetect
		ctrl->gc.base = -1;
	}

	memset(ctrl->gpio_pin_int_masks, 0, sizeof(ctrl->gpio_pin_int_masks));

	err = gpiochip_add_data(&ctrl->gc, ctrl);
	if (err) {
		dev_err(&pdev->dev,"cannot add Realtek GPIO chip, error=%d", err);
		return err;
	}

	if (np && !of_property_read_bool(np, "interrupt-controller"))
	{
		printk("%s: Realtek GPIO controller driver\n", ctrl->gc.label);
		return 0;
	} else {
		irq = platform_get_irq(pdev, 0);
		printk("%s: Realtek GPIO controller driver at IRQ %d\n", ctrl->gc.label, irq);
	}

	err = gpiochip_irqchip_add(&ctrl->gc, &realtek_gpio_irqchip, 0,
		handle_edge_irq, IRQ_TYPE_NONE);
	if (err) {
		dev_err(&pdev->dev, "failed to add gpiochip_irqchip\n");
		goto gpiochip_remove;
	}

	gpiochip_set_chained_irqchip(&ctrl->gc, &realtek_gpio_irqchip,
		platform_get_irq(pdev, 0), realtek_gpio_irq_handler);

	return 0;

gpiochip_remove:
	gpiochip_remove(&ctrl->gc);
	return err;
}

static int realtek_gpio_remove(struct platform_device *pdev)
{
	struct realtek_gpio_ctrl *ctrl = platform_get_drvdata(pdev);

	gpiochip_remove(&ctrl->gc);
	return 0;
}

static struct platform_driver realtek_gpio_driver = {
	.driver = {
		.name = "realtek-gpio",
		.of_match_table = realtek_gpio_of_match,
	},
	.probe = realtek_gpio_probe,
	.remove = realtek_gpio_remove,
};

module_platform_driver(realtek_gpio_driver);

MODULE_DESCRIPTION("Realtek GPIO Driver");
MODULE_LICENSE("GPL v2");