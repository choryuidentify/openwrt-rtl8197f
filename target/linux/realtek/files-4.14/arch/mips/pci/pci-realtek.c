/*
 *  Realtek RLX based SoC PCI host controller driver
 *
 *  Copyright (C) 2019 Gaspare Bruno <gaspare@anlix.io>
 *  Copyright (C) 2017 Weijie Gao <hackpascal@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/mach-realtek/realtek_mem.h>

struct realtek_pci_controller {
	void __iomem *rc_cfg_base;
	void __iomem *rc_ext_base;
	void __iomem *dev_cfg0_base;
	void __iomem *dev_cfg1_base;

	int link_up;

	u32 bus_number;
	struct device_node *np;
	struct pci_controller pci_controller;
	struct resource io_res;
	struct resource mem_res;
	struct clk *clk;
};

static inline struct realtek_pci_controller *
pci_bus_to_realtek_pci_controller(struct pci_bus *bus)
{
	struct pci_controller *hose;

	hose = (struct pci_controller *) bus->sysdata;
	return container_of(hose, struct realtek_pci_controller, pci_controller);
}

static inline void realtek_pcie_mdio_write(struct realtek_pci_controller *rpc, u32 reg, u32 data) 
{
	u32 val;

	val = ((reg&0x1f)<<8) | ((data&0xffff)<<16) | BIT(0);
	__raw_writel(val, rpc->rc_ext_base);
	mdelay(2);
}

static inline
#if defined(CONFIG_CPU_RLX)
__attribute__ ((section(".iram")))
#endif
int realtek_pci_raw_read(void __iomem *mem, int where, int size, uint32_t *value){
	u32 data;
	int s;

	data = __raw_readl(mem + (where & ~3));
	switch (size) {
		case 1:
		case 2:
			s = ((where & 3) * 8);
			data >>= s;
			data &= (size==1?0xff:0xffff);
			break;
		case 4:
			break;
		default:
			return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	if (value)
		*value = data;

	return PCIBIOS_SUCCESSFUL;
}

static 
#if defined(CONFIG_CPU_RLX)
__attribute__ ((section(".iram")))
#endif
int realtek_pci_read(struct pci_bus *bus, unsigned int devfn, int where,
			    int size, uint32_t *value)
{
	struct realtek_pci_controller *rpc;

	rpc = pci_bus_to_realtek_pci_controller(bus);
	if (!rpc->link_up)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if(bus && rpc->bus_number == 0xff)
		rpc->bus_number = bus->number;

	if(bus->number == rpc->bus_number) {
		/* PCIE host controller */
		if (PCI_SLOT(devfn) == 0) {
			if (value)
				realtek_pci_raw_read(rpc->rc_cfg_base, where, size, value);
		}
		else return PCIBIOS_DEVICE_NOT_FOUND;
	}
	else
	if(bus->number == rpc->bus_number+1) {
		/* PCIE devices directly connected */
		if (PCI_SLOT(devfn) == 0){
			if(value)
				realtek_pci_raw_read(rpc->dev_cfg0_base + (PCI_FUNC(devfn) << 12), where, size, value);
		}
		else return PCIBIOS_DEVICE_NOT_FOUND;
	}
	else {
		/* Devices connected through bridge (Max 4 devices in SDK)*/
		if (PCI_SLOT(devfn) < 4){
			// (0xc = PCIE0 IPCFG)
			__raw_writel(((bus->number) << 8) | (PCI_SLOT(devfn) << 3) | PCI_FUNC(devfn), rpc->rc_ext_base+0x0c);
			if(value)
				realtek_pci_raw_read(rpc->dev_cfg1_base, where, size, value);
		}	
	}

	return PCIBIOS_SUCCESSFUL;
}

static inline 
#if defined(CONFIG_CPU_RLX)
__attribute__ ((section(".iram")))
#endif
int realtek_pci_raw_write(void __iomem *mem, int where, int size, uint32_t value){
	u32 data;
	int s,v;

	switch (size) {
		case 1:
		case 2:
			data = __raw_readl(mem + (where & ~3));
			s = ((where & 3) * 8);
			v = ~(size==1?0xff:0xffff) << s;
			data = (data & v) | (value << s);
			break;
		case 4:
			data = value;
			break;
		default:
			return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	__raw_writel(data, mem + (where & ~3));

	return PCIBIOS_SUCCESSFUL;
}

static 
#if defined(CONFIG_CPU_RLX)
__attribute__ ((section(".iram")))
#endif
int realtek_pci_write(struct pci_bus *bus, unsigned int devfn, int where,
			    int size, uint32_t value)
{
	struct realtek_pci_controller *rpc;

	rpc = pci_bus_to_realtek_pci_controller(bus);
	if (!rpc->link_up)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if(bus && rpc->bus_number == 0xff)
		rpc->bus_number = bus->number;

	if(bus->number == rpc->bus_number) {
		/* PCIE host controller */
		if (PCI_SLOT(devfn) == 0) {
			realtek_pci_raw_write(rpc->rc_cfg_base, where, size, value);
		}
		else return PCIBIOS_DEVICE_NOT_FOUND;
	}
	else
	if(bus->number == rpc->bus_number+1) {
		/* PCIE devices directly connected */
		if (PCI_SLOT(devfn) == 0){
			realtek_pci_raw_write(rpc->dev_cfg0_base + (PCI_FUNC(devfn) << 12), where, size, value);
		}
		else return PCIBIOS_DEVICE_NOT_FOUND;
	}
	else {
		/* Devices connected through bridge (Max 4 devices in SDK)*/
		if (PCI_SLOT(devfn) < 4){
			// (0xc = PCIE0 IPCFG)
			__raw_writel(((bus->number) << 8) | (PCI_SLOT(devfn) << 3) | PCI_FUNC(devfn), rpc->rc_ext_base+0x0c);
			realtek_pci_raw_write(rpc->dev_cfg1_base, where, size, value);
		}	
	}

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops realtek_pci_ops = {
	.read	= realtek_pci_read,
	.write	= realtek_pci_write,
};


static int realtek_pcie_check_link(struct realtek_pci_controller *rpc)
{
	int i = 20;
	u32 val = 0;

	do
	{
		val = __raw_readl(rpc->rc_cfg_base + 0x728);
		if((val & 0x1f) == 0x11)
			return 1;

		mdelay(100);
	} while (i--);

	return 0;
}

static inline void realtek_pcie_device_reset(struct realtek_pci_controller *rpc)
{
	u32 val;

	val = sr_r32(REALTEK_SR_CLKMANAGE);
	val &= (~BIT(26)); // PERST=0 for PCI port0
	sr_w32(val, REALTEK_SR_CLKMANAGE);
	mdelay(100); // SDK wait 1s! too much?
	val = sr_r32(REALTEK_SR_CLKMANAGE);
	val |= BIT(26); // PERST=1 for PCI port0
	sr_w32(val, REALTEK_SR_CLKMANAGE);

	//TODO: Need reset for second card (97D)!
}

static inline void realtek_pcie_mdio_reset(struct realtek_pci_controller *rpc)
{
	sr_w32(BIT(3), REALTEK_SR_PCIE_PHY0); 				// mdio reset 0
	sr_w32(BIT(3)|BIT(0), REALTEK_SR_PCIE_PHY0); 		// mdio reset 1
	sr_w32(BIT(3)|BIT(1)|BIT(0), REALTEK_SR_PCIE_PHY0); // load done
}

static inline void realtek_pcie_phy_reset(struct realtek_pci_controller *rpc)
{
	//(0x8 = PCIE0 PWRCR)
	__raw_writel(0x01, rpc->rc_ext_base+0x8);	//bit7:PHY reset=0   bit0: Enable LTSSM=1
	__raw_writel(0x81, rpc->rc_ext_base+0x8);	//bit7:PHY reset=1   bit0: Enable LTSSM=1
}

static void realtek_pcie_reset(struct realtek_pci_controller *rpc)
{
	u32 val;

	//first, Turn On PCIE IP
	val = sr_r32(REALTEK_SR_CLKMANAGE);
	val |= BIT(14);
	sr_w32(val, REALTEK_SR_CLKMANAGE);

	val = sr_r32(REALTEK_SR_CLKMANAGE);
	val |= BIT(26);
	sr_w32(val, REALTEK_SR_CLKMANAGE);

	val = sr_r32(REALTEK_SR_CLKMANAGE);
	val |= BIT(12)|BIT(13)|BIT(18)|BIT(19)|BIT(20);
	sr_w32(val, REALTEK_SR_CLKMANAGE);

	mdelay(100);

	realtek_pcie_mdio_reset(rpc);

	// wait to stabilize (SDK use 500 here. too much?)
	mdelay(100);

	// Configure mdio
	realtek_pcie_mdio_write(rpc, 0x00, 0xd087);
	realtek_pcie_mdio_write(rpc, 0x01, 0x0003);
	realtek_pcie_mdio_write(rpc, 0x02, 0x4d19); //0x4d18); 
	realtek_pcie_mdio_write(rpc, 0x04, 0x5000);

	if (clk_get_rate(rpc->clk) == 40000000) {
		// 40MHz PCI clock
		realtek_pcie_mdio_write(rpc, 0x05, 0x0bcb);
		realtek_pcie_mdio_write(rpc, 0x06, 0x2148); //0xf148);
		realtek_pcie_mdio_write(rpc, 0x07, 0x41ff); // From wireless driver
		realtek_pcie_mdio_write(rpc, 0x08, 0x13f6);
	} else {
		// 25MHz PCI clock
		realtek_pcie_mdio_write(rpc, 0x06, 0xf848);
		realtek_pcie_mdio_write(rpc, 0x07, 0xa7ff);
		realtek_pcie_mdio_write(rpc, 0x08, 0x0c56);
	}

	realtek_pcie_mdio_write(rpc, 0x09, 0x539c); 
	realtek_pcie_mdio_write(rpc, 0x0a, 0x20eb);
	realtek_pcie_mdio_write(rpc, 0x0d, 0x1766);
	realtek_pcie_mdio_write(rpc, 0x0b, 0x0711);

#ifdef CONFIG_SOC_RTL8196E
	realtek_pcie_mdio_write(rpc, 0x0f, 0x0f0f); 
#else
	realtek_pcie_mdio_write(rpc, 0x0f, 0x0a00); 
#endif

	realtek_pcie_mdio_write(rpc, 0x19, 0xfce0);
	realtek_pcie_mdio_write(rpc, 0x1a, 0x7e40); //0x7e4f);
	realtek_pcie_mdio_write(rpc, 0x1b, 0xfc01);	 
	realtek_pcie_mdio_write(rpc, 0x1e, 0xc280);

	realtek_pcie_device_reset(rpc);
	realtek_pcie_phy_reset(rpc);
	mdelay(200);
}

static void load_ranges(struct pci_controller *hose, struct device_node *node)
{
	struct of_pci_range range;
	struct of_pci_range_parser parser;

	hose->of_node = node;

	if (of_pci_range_parser_init(&parser, node))
		return;

	for_each_of_pci_range(&parser, &range) {
		switch (range.flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_IO:
			hose->io_map_base =
				(unsigned long)ioremap(range.cpu_addr, range.size);
			hose->io_resource->flags = range.flags;
			hose->io_resource->name = node->full_name;
			hose->io_resource->start = range.cpu_addr;
			hose->io_resource->end = range.cpu_addr + range.size - 1;
			break;
		case IORESOURCE_MEM:
			hose->mem_resource->flags = range.flags;
			hose->mem_resource->name = node->full_name;
			hose->mem_resource->start = range.cpu_addr;
			hose->mem_resource->end = range.cpu_addr + range.size - 1;
			break;
		}
	}
}

static int realtek_pci_probe(struct platform_device *pdev)
{
	struct realtek_pci_controller *rpc;
	struct resource *res;
	int id;
	u32 val;
	u16 cmd;
	u8 v8;

	id = pdev->id;
	if (id == -1)
		id = 0;

	rpc = devm_kzalloc(&pdev->dev, sizeof(struct realtek_pci_controller),
			    GFP_KERNEL);
	if (!rpc)
		return -ENOMEM;
	rpc->bus_number=0xff;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rc_cfg_base");
	rpc->rc_cfg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rpc->rc_cfg_base))
		return PTR_ERR(rpc->rc_cfg_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rc_ext_base");
	rpc->rc_ext_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rpc->rc_ext_base))
		return PTR_ERR(rpc->rc_ext_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dev_cfg0_base");
	rpc->dev_cfg0_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rpc->dev_cfg0_base))
		return PTR_ERR(rpc->dev_cfg0_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dev_cfg1_base");
	rpc->dev_cfg1_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rpc->dev_cfg1_base))
		return PTR_ERR(rpc->dev_cfg1_base);

	rpc->clk = devm_clk_get(&pdev->dev, NULL);
	if(!rpc->clk)
		return PTR_ERR(rpc->clk);

	iomem_resource.start = 0;
	iomem_resource.end = ~0;
	ioport_resource.start = 0;
	ioport_resource.end = ~0;

	rpc->np = pdev->dev.of_node;
	rpc->pci_controller.pci_ops = &realtek_pci_ops;
	rpc->pci_controller.io_resource = &rpc->io_res;
	rpc->pci_controller.mem_resource = &rpc->mem_res;
	load_ranges(&rpc->pci_controller, pdev->dev.of_node);

	realtek_pcie_reset(rpc);

	rpc->link_up = realtek_pcie_check_link(rpc);
	if (!rpc->link_up)
		dev_warn(&pdev->dev, "PCIe link is down\n");

	cmd = __raw_readw(rpc->rc_cfg_base + PCI_COMMAND);
	cmd = PCI_COMMAND_MASTER | PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
	__raw_writew(cmd, rpc->rc_cfg_base + PCI_COMMAND);

	// Set MAX_PAYLOAD_SIZE to 128B,default
	v8 = __raw_readb(rpc->dev_cfg0_base + 0x78);
	v8 &= ~(PCI_EXP_DEVCTL_PAYLOAD);
	__raw_writeb(v8, rpc->rc_cfg_base + 0x78);

	mdelay(100);

	register_pci_controller(&rpc->pci_controller);

	return 0;
}

int pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct realtek_pci_controller *rpc;
	u16 cmd;
	int irq = 5;

	rpc = pci_bus_to_realtek_pci_controller(dev->bus);
	if(!rpc)
		return 0;

	//TODO: Implement second pcie (get irq from dt)
	// Bus:1 Slot:0 Pin:1 -> first wireless card

	/* setup the slot */
	cmd = PCI_COMMAND_MASTER | PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
	pci_write_config_word(dev, PCI_COMMAND, cmd);
	pci_write_config_byte(dev, PCI_INTERRUPT_LINE, irq);

	return irq;
}

int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}

static const struct of_device_id realtek_pci_ids[] = {
	{ .compatible = "realtek,rtl8196b-pci" },
	{},
};

static struct platform_driver realtek_pci_driver = {
	.probe = realtek_pci_probe,
	.driver = {
		.name = "realtek-pci",
		.of_match_table = of_match_ptr(realtek_pci_ids),
	},
};

static int __init realtek_pci_init(void)
{
	return platform_driver_register(&realtek_pci_driver);
}

postcore_initcall(realtek_pci_init);
