/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2019 Gaspare Bruno <gaspare@anlix.io>
 *
 * Realtek IRQ handler
 * This IRQ driver have the 8 common MIPS CPU irqs
 * RLX driver have an aditional 8 hardware irqs
 * TODO: Implement the RLX additional irqs
 */

#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>

#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <asm/time.h>

#include <asm/mach-realtek/realtek_mem.h>

__iomem void *_intc_membase;
static u32 mips_chip_irqs;

#define ic_w32(val, reg) __raw_writel(val, _intc_membase + reg)
#define ic_r32(reg)      __raw_readl(_intc_membase + reg)

// Interrupt Registers and bits
#define REALTEK_IC_REG_MASK			0x00
#define REALTEK_IC_REG_STATUS		0x04
#define REALTEK_IC_REG2_MASK		0x20
#define REALTEK_IC_REG2_STATUS		0x24

#define REALTEK_IC_REG_IRR0			0x08
#define REALTEK_IC_REG_IRR1			0x0C
#define REALTEK_IC_REG_IRR2			0x10
#define REALTEK_IC_REG_IRR3			0x14
#define REALTEK_IC_REG_IRR4			0x28
#define REALTEK_IC_REG_IRR5			0x2C
#define REALTEK_IC_REG_IRR6			0x30
#define REALTEK_IC_REG_IRR7			0x34

#define REALTEK_INTC_IRQ_COUNT	32

#define REALTEK_IRQ_GENERIC		2   

#define REALTEK_IRQ_TIMER		7
#define REALTEK_IRQ_NET			4
#define REALTEK_IRQ_WIFI		6
#define REALTEK_IRQ_PCI0		5
#define REALTEK_IRQ_PCI1		REALTEK_IRQ_WIFI
#define REALTEK_IRQ_UART0		REALTEK_IRQ_GENERIC  
#define REALTEK_IRQ_GPIO1		REALTEK_IRQ_GENERIC
#define REALTEK_IRQ_GPIO2		REALTEK_IRQ_GENERIC

/* Definition for SoCs
   RTL8196E and RTL8197D have a RLX chipset
     They use 32 SoC IRQs abose the hardware 
   RTL8197F Have 64 SoC IRQs, but dont have
     the additional 8 hardware irqs
*/

#ifdef CONFIG_SOC_RTL8197F

#define REALTEK_INTC_IRQ_BASE 8

u32 realtek_soc_irq_init(void) 
{
	ic_w32((0), 
		REALTEK_IC_REG_IRR0);

	ic_w32((REALTEK_IRQ_NET 	<< 28) |
		   (REALTEK_IRQ_UART0 	<< 4), 
		REALTEK_IC_REG_IRR1);

	ic_w32((REALTEK_IRQ_PCI0 << 20) |
		   (REALTEK_IRQ_GPIO2 << 4) |
		   (REALTEK_IRQ_GPIO1 << 0), 
		REALTEK_IC_REG_IRR2);

	ic_w32((REALTEK_IRQ_WIFI << 20), 
		REALTEK_IC_REG_IRR3);

	ic_w32((0), 
		REALTEK_IC_REG_IRR4);

	ic_w32((REALTEK_IRQ_TIMER << 28), 
		REALTEK_IC_REG_IRR5);

	ic_w32((0), 
		REALTEK_IC_REG_IRR6);

	ic_w32((0), 
		REALTEK_IC_REG_IRR7);

	// map high priority interrupts to mips irq controler
	// BIT 15 on MASK1 is network switch
	// BIT 21 on MASK1 is PCIE (wifi5g)
	// BIT 29 on MASK1 is wifi 2.4G
	// BIT 15 on MASK2 is R4K Timer
	ic_w32(BIT(15)|BIT(21)|BIT(29), REALTEK_IC_REG_MASK);
	ic_w32(BIT(15), REALTEK_IC_REG2_MASK);

	// Return only MARK1
	return BIT(15)|BIT(21)|BIT(29);
}

#else

// Above RLX driver (8 for Mips + 8 for hardware RLX)
#define REALTEK_INTC_IRQ_BASE 16

u32 realtek_soc_irq_init(void) 
{
	ic_w32((0), 
		REALTEK_IC_REG_IRR0);

	ic_w32((REALTEK_IRQ_TIMER << 0  | 
			REALTEK_IRQ_UART0 << 16 |
			REALTEK_IRQ_NET   << 28 ), 
		REALTEK_IC_REG_IRR1);


	ic_w32((
		REALTEK_IRQ_PCI0 << 20
#ifdef CONFIG_SOC_RTL8197D
		| REALTEK_IRQ_PCI1 << 24
#endif
		), 
		REALTEK_IC_REG_IRR2);

	ic_w32((0), 
		REALTEK_IC_REG_IRR3);

	// map high priority interrupts to mips irq controler
	// TC0 (Timer) (BIT8) to mips
	// Network Switch (BIT15)
	// PCIE0 (wifi0) (BIT21)
	ic_w32(BIT(8)|BIT(15)|BIT(21)
#ifdef CONFIG_SOC_RTL8197D
		|BIT(22)
#endif
		, REALTEK_IC_REG_MASK);

	return BIT(8)|BIT(15)|BIT(21)
#ifdef CONFIG_SOC_RTL8197D
		|BIT(22)
#endif
		;
}

#endif

static void realtek_soc_irq_unmask(struct irq_data *d)
{
	u32 t;

	t = ic_r32(REALTEK_IC_REG_MASK);
	ic_w32(t | BIT(d->hwirq), REALTEK_IC_REG_MASK);
}

static void realtek_soc_irq_mask(struct irq_data *d)
{
	u32 t;

	t = ic_r32(REALTEK_IC_REG_MASK);
	ic_w32(t & ~BIT(d->hwirq), REALTEK_IC_REG_MASK);
}

static struct irq_chip realtek_soc_irq_chip = {
	.name			= "SOC",
	.irq_unmask 	= realtek_soc_irq_unmask,
	.irq_mask 		= realtek_soc_irq_mask,
};

static void realtek_soc_irq_handler(struct irq_desc *desc)
{
	u32 pending;
	struct irq_domain *domain;

	pending = ic_r32(REALTEK_IC_REG_MASK) &
			  ic_r32(REALTEK_IC_REG_STATUS);

	if (pending & mips_chip_irqs) {
		/*
		 * interrupts routed to mips core found here
		 * clear these bits as they can't be handled here
		 */
		ic_w32(mips_chip_irqs, REALTEK_IC_REG_STATUS);
		pending &= ~mips_chip_irqs;

		if (!pending)
		        return;
	}

	if (!pending) {
		spurious_interrupt();
		return;
	}

	domain = irq_desc_get_handler_data(desc);
	while (pending) {
		int bit = __ffs(pending);
		generic_handle_irq(irq_find_mapping(domain, bit));
		pending &= ~BIT(bit);
	}
}

static int intc_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	irq_set_chip_and_handler(irq, &realtek_soc_irq_chip, handle_level_irq);

	return 0;
}

static const struct irq_domain_ops irq_domain_ops = {
	.xlate = irq_domain_xlate_onecell,
	.map = intc_map,
};

asmlinkage void plat_irq_dispatch(void)
{
	unsigned long pending;

	pending = read_c0_status() & read_c0_cause() & ST0_IM;

	if (pending & STATUSF_IP7) 
		do_IRQ(REALTEK_IRQ_TIMER);

	else if (pending & STATUSF_IP4) 
		do_IRQ(REALTEK_IRQ_NET);

	else if (pending & STATUSF_IP5) 
		do_IRQ(REALTEK_IRQ_PCI0);

	else if (pending & STATUSF_IP6) 
		do_IRQ(REALTEK_IRQ_PCI1);

	else if (pending & STATUSF_IP2)
		do_IRQ(REALTEK_IRQ_GENERIC);

	else
		spurious_interrupt();
}

static int __init intc_of_init(struct device_node *node,
			       struct device_node *parent)
{
	struct resource res;
	struct irq_domain *domain;

	if (of_address_to_resource(node, 0, &res))
		panic("Failed to get resource for %s", node->name);

	_intc_membase = ioremap_nocache(res.start, resource_size(&res));
	if(!_intc_membase)
		panic("Failed to map memory for %s", node->name);

#ifdef CONFIG_SOC_RTL8197F
	// 8197F uses CEVT and Timers from 4K
	// Just need to redirect the right irq...
	cp0_compare_irq = REALTEK_IRQ_TIMER;
    cp0_perfcount_irq = REALTEK_IRQ_TIMER;

	mips_hpt_frequency = 1000000000 / 2;
#endif

	// Map Interrupts according to SoC
	mips_chip_irqs = realtek_soc_irq_init();

	domain = irq_domain_add_legacy(node, REALTEK_INTC_IRQ_COUNT,
			REALTEK_INTC_IRQ_BASE, 0, &irq_domain_ops, NULL);
	if (!domain)
		panic("Failed to add irqdomain");

	irq_set_chained_handler_and_data(REALTEK_IRQ_GENERIC, realtek_soc_irq_handler, domain);

	return 0;
}

static struct of_device_id __initdata of_irq_ids[] = {
	{ .compatible = "mti,cpu-interrupt-controller", .data = mips_cpu_irq_of_init },
	{ .compatible = "realtek,rtl819x-intc", .data = intc_of_init },
	{},
};

void __init arch_init_irq(void)
{
	of_irq_init(of_irq_ids);
}

