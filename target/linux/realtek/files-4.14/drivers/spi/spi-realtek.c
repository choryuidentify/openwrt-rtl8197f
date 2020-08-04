/*
 * SPI controller driver for the Realtek SoCs
 *
 * Copyright (C) 2017 Weijie Gao <hackpascal@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <linux/err.h>

#define DRIVER_NAME	"realtek-spi"

#define RTK_SPI_CONFIG_OFFSET					0x00

#define RTK_SPI_CLK_DIV_SHIFT					29

#define RTK_SPI_READ_BYTE_ORDER					BIT(28)
#define RTK_SPI_WRITE_BYTE_ORDER				BIT(27)

#define RTK_SPI_CS_DESELECT_TIME_SHIFT				22


#define RTK_SPI_CONTROL_STATUS_OFFSET				0x08

#define RTK_SPI_CS_0_HIGH					BIT(31)
#define RTK_SPI_CS_1_HIGH					BIT(30)
#define RTK_SPI_CS_ALL_HIGH					(RTK_SPI_CS_0_HIGH | RTK_SPI_CS_1_HIGH)

#define RTK_SPI_DATA_LENGTH_SHIFT				28
#define RTK_SPI_DATA_LENGTH_MASK				0x3

#define RTK_SPI_READY						BIT(27)


#define RTK_SPI_DATA_OFFSET					0x0c


#define RTK_SPI_DEFAULT_CLOCK_DIVIDER_INDEX			3

struct realtek_spi_data {
	struct spi_master *master;
	u32			ioc_base;
	void __iomem		*base;
};

static const u32 realtek_spi_clk_div_table[] = {2, 4, 6, 8, 10, 12, 14, 16};

#ifdef CONFIG_CPU_BIG_ENDIAN
static inline u32 realtek_spi_make_data(u32 data, u32 bytes)
{
	return data << ((4 - bytes) << 3);
}

static inline u32 realtek_spi_resolve_data(u32 data, u32 bytes)
{
	return data >> ((4 - bytes) << 3);
}
#else
static inline u32 realtek_spi_make_data(u32 data, u32 bytes)
{
	return data;
}

static inline u32 realtek_spi_resolve_data(u32 data, u32 bytes)
{
	return data;
}
#endif /* CONFIG_CPU_BIG_ENDIAN */

static inline u32 realtek_spi_rr(struct realtek_spi_data *rsd, unsigned reg)
{
	return ioread32(rsd->base + reg);
}

static inline void realtek_spi_wr(struct realtek_spi_data *rsd, unsigned reg, u32 val)
{
	iowrite32(val, rsd->base + reg);
}

static void realtek_spi_set_speed_default(struct realtek_spi_data *rsd)
{
	realtek_spi_wr(rsd, RTK_SPI_CONFIG_OFFSET,
		(7 << RTK_SPI_CLK_DIV_SHIFT) |
		(31 << RTK_SPI_CS_DESELECT_TIME_SHIFT)
#ifdef CONFIG_CPU_BIG_ENDIAN
		| RTK_SPI_READ_BYTE_ORDER | RTK_SPI_WRITE_BYTE_ORDER
#endif
);
}

static void realtek_spi_poll(struct realtek_spi_data *rsd)
{
	while ((realtek_spi_rr(rsd, RTK_SPI_CONTROL_STATUS_OFFSET) & RTK_SPI_READY) == 0);
}

static void realtek_spi_set_cs(struct spi_device *spi, bool cs_high)
{
	struct realtek_spi_data *rsd = spi_master_get_devdata(spi->master);
	
	cs_high = (spi->mode & SPI_CS_HIGH) ? !cs_high : cs_high;

	if (cs_high) {
		switch (spi->chip_select) {
		case 0:
			rsd->ioc_base = RTK_SPI_CS_0_HIGH;
			break;
		case 1:
			rsd->ioc_base = RTK_SPI_CS_1_HIGH;
			break;
		default:
			rsd->ioc_base = 0;
		}
	} else {
		switch (spi->chip_select) {
		case 0:
			rsd->ioc_base = RTK_SPI_CS_1_HIGH;
			break;
		case 1:
			rsd->ioc_base = RTK_SPI_CS_0_HIGH;
			break;
		default:
			rsd->ioc_base = RTK_SPI_CS_ALL_HIGH;
		}
	}

	rsd->ioc_base |= RTK_SPI_READY;

	realtek_spi_wr(rsd, RTK_SPI_CONTROL_STATUS_OFFSET, rsd->ioc_base);
}

static void realtek_spi_set_txrx_size(struct realtek_spi_data *rsd, u32 size)
{
	realtek_spi_wr(rsd, RTK_SPI_CONTROL_STATUS_OFFSET,
		rsd->ioc_base | ((size - 1) << RTK_SPI_DATA_LENGTH_SHIFT));
}

static void realtek_spi_read(struct realtek_spi_data *rsd, u8 *buf, unsigned len)
{
	if ((size_t) buf % 4)
	{
		realtek_spi_set_txrx_size(rsd, 1);

		while (((size_t) buf % 4) && len)
		{
			realtek_spi_poll(rsd);
			*buf = realtek_spi_resolve_data(realtek_spi_rr(rsd, RTK_SPI_DATA_OFFSET), 1);
			buf++;
			len--;
		}
	}

	realtek_spi_set_txrx_size(rsd, 4);

	while (len >= 4)
	{
		realtek_spi_poll(rsd);
		*(u32 *) buf = realtek_spi_resolve_data(realtek_spi_rr(rsd, RTK_SPI_DATA_OFFSET), 4);
		buf += 4;
		len -= 4;
	}

	realtek_spi_set_txrx_size(rsd, 1);

	while (len)
	{
		realtek_spi_poll(rsd);
		*buf = realtek_spi_resolve_data(realtek_spi_rr(rsd, RTK_SPI_DATA_OFFSET), 1);
		buf++;
		len--;
	}
}

static void realtek_spi_write(struct realtek_spi_data *rsd, const u8 *buf, unsigned len)
{
	if ((size_t) buf % 4)
	{
		realtek_spi_set_txrx_size(rsd, 1);

		while (((size_t) buf % 4) && len)
		{
			realtek_spi_wr(rsd, RTK_SPI_DATA_OFFSET, realtek_spi_make_data(*buf, 1));
			realtek_spi_poll(rsd);
			buf++;
			len--;
		}
	}

	realtek_spi_set_txrx_size(rsd, 4);

	while (len >= 4)
	{
		realtek_spi_wr(rsd, RTK_SPI_DATA_OFFSET, realtek_spi_make_data(*(const u32 *) buf, 4));
		realtek_spi_poll(rsd);
		buf += 4;
		len -= 4;
	}

	realtek_spi_set_txrx_size(rsd, 1);

	while (len)
	{
		realtek_spi_wr(rsd, RTK_SPI_DATA_OFFSET, realtek_spi_make_data(*buf, 1));
		realtek_spi_poll(rsd);
		buf++;
		len--;
	}
}

static int realtek_spi_transfer_one(struct spi_master *master,
				     struct spi_device *spi,
				     struct spi_transfer *xfer)
{
	struct realtek_spi_data *rsd = spi_master_get_devdata(spi->master);

	realtek_spi_set_speed_default(rsd);

	if (xfer->tx_buf && xfer->rx_buf) {
		dev_err(&spi->dev, "Read and write operation cannot be performed simultaneously\n");
		return -EPERM;
	}

	if (xfer->tx_buf)
		realtek_spi_write(rsd, (const u8 *) xfer->tx_buf, xfer->len);

	if (xfer->rx_buf)
		realtek_spi_read(rsd, (u8 *) xfer->rx_buf, xfer->len);

	return 0;
}

static int realtek_spi_probe(struct platform_device *pdev)
{
	struct realtek_spi_data *rsd;
	struct spi_master *master;
	struct resource *res;
	int ret;

	master = spi_alloc_master(&pdev->dev, sizeof(*rsd));
	if (!master)
		return -ENOMEM;

	rsd = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, rsd);

	rsd->master = master;

	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = 0;
	master->num_chipselect = 2;
	master->mode_bits = SPI_CPOL | SPI_CPHA;
	master->flags = SPI_MASTER_HALF_DUPLEX;
	master->bits_per_word_mask = SPI_BPW_MASK(32) | SPI_BPW_MASK(24) |
		SPI_BPW_MASK(16) | SPI_BPW_MASK(8);

	master->transfer_one = realtek_spi_transfer_one;
	master->set_cs = realtek_spi_set_cs;

	master->min_speed_hz = 190000000/16; //realtek_spi_calc_speed(rsd, RTK_SPI_CLK_DIV_MAX_INDEX);
	master->max_speed_hz = 190000000/2;  //realtek_spi_calc_speed(rsd, 0);

	dev_set_drvdata(&pdev->dev, master);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rsd->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rsd->base)) {
		ret = PTR_ERR(rsd->base);
		goto err_out;
	}

	/* Set default operation clock */
	realtek_spi_set_speed_default(rsd);

	/* Deassert all chip selects */
	realtek_spi_wr(rsd, RTK_SPI_CONTROL_STATUS_OFFSET, RTK_SPI_CS_ALL_HIGH | RTK_SPI_READY);

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret)
		goto err_out;

	return 0;

err_out:
	spi_master_put(master);

	return ret;
}

static int realtek_spi_remove(struct platform_device *pdev)
{
	struct realtek_spi_data *rsd = platform_get_drvdata(pdev);

	/* Reset to default operation clock */
	realtek_spi_set_speed_default(rsd);

	/* Deassert all chip selects */
	realtek_spi_wr(rsd, RTK_SPI_CONTROL_STATUS_OFFSET, RTK_SPI_CS_ALL_HIGH | RTK_SPI_READY);

	spi_master_put(rsd->master);

	return 0;
}

static void realtek_spi_shutdown(struct platform_device *pdev)
{
	realtek_spi_remove(pdev);
}

static const struct of_device_id realtek_spi_match[] = {
	{ .compatible = "realtek,rtl819x-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, realtek_spi_match);

static struct platform_driver realtek_spi_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = realtek_spi_match,
	},
	.probe		= realtek_spi_probe,
	.remove		= realtek_spi_remove,
	.shutdown	= realtek_spi_shutdown,
};
module_platform_driver(realtek_spi_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Weijie Gao <hackpascal@gmail.com>");
MODULE_DESCRIPTION("Realtek SoC SPI controller driver");
MODULE_ALIAS("platform:" DRIVER_NAME);
