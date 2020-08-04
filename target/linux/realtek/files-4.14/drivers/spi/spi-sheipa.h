/*
 * SHEIPA SPI controller driver
 *
 * Author: Realtek PSP Group
 *
 * Copyright 2015, Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_SHEIPA_H
#define _SPI_SHEIPA_H

#include <linux/io.h>
#include <linux/scatterlist.h>

/* SPIC config */
#define ps_DW_APB_SPI_FLASH_BASE BSP_PS_I_SSI_BASE

#define ps_CC_SPI_FLASH_NUM_SLAVES            1
#define ps_CC_SPI_FLASH_HAS_DMA               0
#define ps_CC_SPI_FLASH_DMA_TX_SGL_STATUS     1
#define ps_CC_SPI_FLASH_DMA_RX_SGL_STATUS     1
#define ps_CC_SPI_FLASH_RX_FIFO_DEPTH         0x20
#define ps_CC_SPI_FLASH_TX_FIFO_DEPTH         0x20
#define ps_CC_SPI_FLASH_RX_ABW                6
#define ps_CC_SPI_FLASH_TX_ABW                6
#define ps_CC_SPI_FLASH_INTR_POL              1
#define ps_CC_SPI_FLASH_INTR_IO               1
#define ps_CC_SPI_FLASH_INDIVIDUAL            0
#define ps_CC_SPI_FLASH_ID                    0x0
#define ps_CC_SPI_FLASH_HC_FRF                0
#define ps_CC_SPI_FLASH_DFLT_FRF              0x0
#define ps_CC_SPI_FLASH_DFLT_SCPOL            0x0
#define ps_CC_SPI_FLASH_DFLT_SCPH             0x0
#define ps_CC_SPI_FLASH_CLK_PERIOD            400
#define ps_CC_SPI_FLASH_VERSION_ID            0x0
#define ps_CC_SPI_FLASH_DFLT_SCLK             0x0

/* DW error code*/

#define DW_EPERM            1	/* operation not permitted */
#define DW_EIO              5	/* I/O error */
#define DW_ENXIO            6	/* no such device or address */
#define DW_ENOMEM           12	/* out of memory */
#define DW_EACCES           13	/* permission denied */
#define DW_EBUSY            16	/* device or resource busy */
#define DW_ENODEV           19	/* no such device */
#define DW_EINVAL           22	/* invalid argument */
#define DW_ENOSPC           28	/* no space left on device */
#define DW_ENOSYS           38	/* function not implemented/supported */
#define DW_ECHRNG           44	/* channel number out of range */
#define DW_ENODATA          61	/* no data available */
#define DW_ETIME            62	/* timer expired */
#define DW_EPROTO           71	/* protocol error */

/* Returns the width of the specified bit-field. */
#define DW_BIT_WIDTH(__bfws)    ((uint32_t) (bfw ## __bfws))

/* Returns the offset of the specified bit-field. */
#define DW_BIT_OFFSET(__bfws)   ((uint32_t) (bfo ## __bfws))

/* Returns a mask with the bits to be addressed set and all others cleared. */
#define DW_BITS_MASK(__bfws, __bits) ((uint32_t) ((__bfws) == 32) ?  \
	0x0 : (((0xffffffff) >> (32 - __bits)) << (__bfws)))

#define DW_BIT_MASK(__bfws) ((uint32_t) ((__bfws) == 32) ?  \
	0x0 : (0x1 << (__bfws)))

#define DW_BIT_MASK_WIDTH(__bfws, __bits) ((uint32_t) ((__bfws) == 32) ?  \
	0xFFFFFFFF : (((1 << (__bits)) - 1) << (__bfws)))

/* Clear the specified bits. */
#define DW_BITS_CLEAR(__datum, __bfws, __bits)                      \
	((__datum) = ((uint32_t) (__datum) & ~DW_BITS_MASK(__bfws, __bits)))

#define DW_BIT_CLEAR(__datum, __bfws)                               \
	((__datum) = ((uint32_t) (__datum) & ~DW_BIT_MASK(__bfws)))

/*
 * Returns the relevant bits masked from the data word, still at their
 * original offset.
 */
#define DW_BIT_GET_UNSHIFTED(__datum, __bfws)                       \
	((uint32_t) ((__datum) & DW_BIT_MASK(__bfws)))

/*
 * Returns the relevant bits masked from the data word shifted to bit
 * zero (i.e. access the specifed bits from a word of data as an
 * integer value).
 */
#define DW_BIT_GET(__datum, __bfws)				\
	((uint32_t) (((__datum) & DW_BIT_MASK(__bfws)) >>	\
	(__bfws)))

/*
 * Place the specified value into the specified bits of a word of data
 * (first the data is read, and the non-specified bits are re-written).
 */
#define DW_BITS_SET(__datum, __bfws, __bits)				\
	((__datum) = ((uint32_t) (__datum) &				\
				~DW_BITS_MASK(__bfws, __bits)) |	\
	(DW_BITS_MASK(__bfws, __bits)))

#define DW_BIT_SET(__datum, __bfws)					\
	((__datum) = ((uint32_t) (__datum) & ~DW_BIT_MASK(__bfws)) |	\
	(DW_BIT_MASK(__bfws)))

#define DW_BITS_SET_VAL(__datum, __bfws, __val, bit_num)		\
	((__datum) = ((uint32_t) (__datum) &				\
				~DW_BIT_MASK_WIDTH(__bfws, bit_num)) |	\
	((__val << (__bfws)) & DW_BIT_MASK_WIDTH(__bfws, bit_num)))

/*
 * Place the specified value into the specified bits of a word of data
 * without reading first - for sensitive interrupt type registers
 */
#define DW_BIT_SET_NOREAD(__datum, __bfws, __val)                   \
	((uint32_t) ((__datum) = (((__val) << (bfo ## __bfws)) &        \
					DW_BIT_MASK(__bfws))))

/* Shift the specified value into the desired bits. */
#define DW_BIT_BUILD(__bfws, __val)                                 \
	((uint32_t) (((__val) << (bfo ## __bfws)) & DW_BIT_MASK(__bfws)))

/* system endian */
// #define BIG_ENDIAN 0

/* FLASH base address for auto mode */
#define	FIFO_SIZE	64
#define	FIFO_HALF_SIZE	(FIFO_SIZE / 2)

/* Soc tunning dummy cycle only. */
#define DEF_RD_TUNING_DUMMY_CYCLE  0x2
#define DEF_WR_BLOCK_BOUND         256

/* General flash opcode. */
#define WRSR		0x01
#define PP		0x02
#define NORM_READ	0x03
#define WRDI		0x04	/* write disable */
#define RDSR		0x05
#define WREN		0x06
#define FAST_READ       0x0b
#define RDID		0x9f
#define CE		0xc7	/* chip erase */
#define BE_4K		0x20	/* erase 4KiB Block */
#define SE		0xd8	/* sector erase(usually 64KiB) */

/*Spansion 4byte op code*/
#define PP_4B 0x12		/* Spansion Page Program (4-byte Address) */
#define SE_4B 0xdc		/* Spansion Erase 256 kB (4-byte Address) */
#define BRWR 0x17		/* Spansion Bank Register Write */

/* Support auto mode flash dummy and type info only. */
#define DEF_RD_DUAL_TYPE           RD_DUAL_IO
#define DEF_RD_QUAD_TYPE           RD_QUAD_IO
#define DEF_WR_DUAL_TYPE           WR_MULTI_NONE
#define DEF_WR_QUAD_TYPE           WR_QUAD_II
#define DEF_RD_DUAL_DUMMY_CYCLE    0x4
#define DEF_RD_QUAD_DUMMY_CYCLE    0x6
#define DEF_RD_FAST_DUMMY_CYCLE    0x8

/* Support auto mode flash opcode only. */
#define PPX2_I		0x02
#define PPX2_II		0x02
#define PPX4_I		0x02
#define	PPX4_II		0x38
#define READX2_I	0x03
#define READX2_IO	0xbb	/* data and addr channel */
#define READX4_I	0x03
#define READX4_IO	0xeb

/* support auto mode */
#define AUTO_MODE	0xf5

/* Support address 4 byte opcode for large size flash */
#define EN4B		0xb7	/* Enter 4 byte mode */
#define EX4B		0xe9	/* Exit  4 byte mode */
/*
 *  Used in conjunction with bitops.h to access register bitfields.
 *  They are defined as bit offset/mask pairs for each DMA register
 *  bitfield.
 */
#define bfoSPI_FLASH_CTRLR0_FAST_RD         ((uint32_t)   20)
#define bfwSPI_FLASH_CTRLR0_FAST_RD         ((uint32_t)    1)
#define bfoSPI_FLASH_CTRLR0_DATA_CH         ((uint32_t)   18)
#define bfwSPI_FLASH_CTRLR0_DATA_CH         ((uint32_t)    2)
#define bfoSPI_FLASH_CTRLR0_ADDR_CH         ((uint32_t)   16)
#define bfwSPI_FLASH_CTRLR0_ADDR_CH         ((uint32_t)    2)
#define bfoSPI_FLASH_CTRLR0_CFS             ((uint32_t)   12)
#define bfwSPI_FLASH_CTRLR0_CFS             ((uint32_t)    3)
#define bfoSPI_FLASH_CTRLR0_SRL             ((uint32_t)   11)
#define bfwSPI_FLASH_CTRLR0_SRL             ((uint32_t)    1)
#define bfoSPI_FLASH_CTRLR0_SLV_OE          ((uint32_t)   10)
#define bfwSPI_FLASH_CTRLR0_SLV_OE          ((uint32_t)    1)
#define bfoSPI_FLASH_CTRLR0_TMOD            ((uint32_t)    8)
#define bfwSPI_FLASH_CTRLR0_TMOD            ((uint32_t)    2)
#define bfoSPI_FLASH_CTRLR0_SCPOL           ((uint32_t)    7)
#define bfwSPI_FLASH_CTRLR0_SCPOL           ((uint32_t)    1)
#define bfoSPI_FLASH_CTRLR0_FRF             ((uint32_t)    4)
#define bfwSPI_FLASH_CTRLR0_FRF             ((uint32_t)    2)
#define bfoSPI_FLASH_CTRLR0_DFS             ((uint32_t)    0)
#define bfwSPI_FLASH_CTRLR0_DFS             ((uint32_t)    4)
#define bfoSPI_FLASH_CTRLR1_NDF             ((uint32_t)    0)
#define bfwSPI_FLASH_CTRLR1_NDF             ((uint32_t)   16)
#define bfoSPI_FLASH_SSIENR_SSI_EN          ((uint32_t)    0)
#define bfwSPI_FLASH_SSIENR_SSI_EN          ((uint32_t)    1)
#define bfoSPI_FLASH_MWCR_MHS               ((uint32_t)    2)
#define bfwSPI_FLASH_MWCR_MHS               ((uint32_t)    1)
#define bfoSPI_FLASH_MWCR_MDD               ((uint32_t)    1)
#define bfwSPI_FLASH_MWCR_MDD               ((uint32_t)    1)
#define bfoSPI_FLASH_MWCR_MWMOD             ((uint32_t)    0)
#define bfwSPI_FLASH_MWCR_MWMOD             ((uint32_t)    1)
#define bfoSPI_FLASH_SER                    ((uint32_t)    0)
#define bfwSPI_FLASH_SER                    ((uint32_t)    4)
#define bfoSPI_FLASH_BAUDR_SCKDV            ((uint32_t)    0)
#define bfwSPI_FLASH_BAUDR_SCKDV            ((uint32_t)   16)
#define bfoSPI_FLASH_TXFTLR_TFT             ((uint32_t)    0)
#define bfwSPI_FLASH_TXFTLR_TFT             ((uint32_t)    3)
#define bfoSPI_FLASH_RXFTLR_RFT             ((uint32_t)    0)
#define bfwSPI_FLASH_RXFTLR_RFT             ((uint32_t)    3)
#define bfoSPI_FLASH_TXFLR_TXTFL            ((uint32_t)    0)
#define bfwSPI_FLASH_TXFLR_TXTFL            ((uint32_t)    3)
#define bfoSPI_FLASH_RXFLR_RXTFL            ((uint32_t)    0)
#define bfwSPI_FLASH_RXFLR_RXTFL            ((uint32_t)    3)
#define bfoSPI_FLASH_SR_BUSY                ((uint32_t)    0)
#define bfwSPI_FLASH_SR_BUSY                ((uint32_t)    1)
#define bfoSPI_FLASH_SR_TFNF                ((uint32_t)    1)
#define bfwSPI_FLASH_SR_TFNF                ((uint32_t)    1)
#define bfoSPI_FLASH_SR_TFE                 ((uint32_t)    2)
#define bfwSPI_FLASH_SR_TFE                 ((uint32_t)    1)
#define bfoSPI_FLASH_SR_RFNE                ((uint32_t)    3)
#define bfwSPI_FLASH_SR_RFNE                ((uint32_t)    1)
#define bfoSPI_FLASH_SR_RFF                 ((uint32_t)    4)
#define bfwSPI_FLASH_SR_RFF                 ((uint32_t)    1)
#define bfoSPI_FLASH_SR_TXE                 ((uint32_t)    5)
#define bfwSPI_FLASH_SR_TXE                 ((uint32_t)    1)
#define bfoSPI_FLASH_SR_DCOL                ((uint32_t)    6)
#define bfwSPI_FLASH_SR_DCOL                ((uint32_t)    1)
#define bfoSPI_FLASH_IMR_TXEIM              ((uint32_t)     0)
#define bfwSPI_FLASH_IMR_TXEIM              ((uint32_t)     1)
#define bfoSPI_FLASH_IMR_TXOIM              ((uint32_t)     1)
#define bfwSPI_FLASH_IMR_TXOIM              ((uint32_t)     1)
#define bfoSPI_FLASH_IMR_RXUIM              ((uint32_t)     2)
#define bfwSPI_FLASH_IMR_RXUIM              ((uint32_t)     1)
#define bfoSPI_FLASH_IMR_RXOIM              ((uint32_t)     3)
#define bfwSPI_FLASH_IMR_RXOIM              ((uint32_t)     1)
#define bfoSPI_FLASH_IMR_RXFIM              ((uint32_t)     4)
#define bfwSPI_FLASH_IMR_RXFIM              ((uint32_t)     1)
#define bfoSPI_FLASH_IMR_FSEIM              ((uint32_t)     5)
#define bfwSPI_FLASH_IMR_FSEIM              ((uint32_t)     1)
#define bfoSPI_FLASH_IMR_WBEIM              ((uint32_t)     6)
#define bfwSPI_FLASH_IMR_WBEIM              ((uint32_t)     1)
#define bfoSPI_FLASH_IMR_BYEIM              ((uint32_t)     7)
#define bfwSPI_FLASH_IMR_BYEIM              ((uint32_t)     1)
#define bfoSPI_FLASH_IMR_ACTIM              ((uint32_t)     8)
#define bfwSPI_FLASH_IMR_ACTIM              ((uint32_t)     1)
#define bfoSPI_FLASH_IMR_TXEIM_PEND         ((uint32_t)     9)
#define bfwSPI_FLASH_IMR_TXEIM_PEND         ((uint32_t)     1)
#define bfoSPI_FLASH_ISR_TXEIS              ((uint32_t)     0)
#define bfwSPI_FLASH_ISR_TXEIS              ((uint32_t)     1)
#define bfoSPI_FLASH_ISR_TXOIS              ((uint32_t)     1)
#define bfwSPI_FLASH_ISR_TXOIS              ((uint32_t)     1)
#define bfoSPI_FLASH_ISR_RXUIS              ((uint32_t)     2)
#define bfwSPI_FLASH_ISR_RXUIS              ((uint32_t)     1)
#define bfoSPI_FLASH_ISR_RXOIS              ((uint32_t)     3)
#define bfwSPI_FLASH_ISR_RXOIS              ((uint32_t)     1)
#define bfoSPI_FLASH_ISR_RXFIS              ((uint32_t)     4)
#define bfwSPI_FLASH_ISR_RXFIS              ((uint32_t)     1)
#define bfoSPI_FLASH_ISR_FSEIS              ((uint32_t)     5)
#define bfwSPI_FLASH_ISR_FSEIS              ((uint32_t)     1)
#define bfoSPI_FLASH_ISR_WBEIS              ((uint32_t)     6)
#define bfwSPI_FLASH_ISR_WBEIS              ((uint32_t)     1)
#define bfoSPI_FLASH_ISR_BYEIS              ((uint32_t)     7)
#define bfwSPI_FLASH_ISR_BYEIS              ((uint32_t)     1)
#define bfoSPI_FLASH_ISR_ACTIS              ((uint32_t)     8)
#define bfwSPI_FLASH_ISR_ACTIS              ((uint32_t)     1)
#define bfoSPI_FLASH_ISR_TXEIS_PEND         ((uint32_t)     9)
#define bfwSPI_FLASH_ISR_TXEIS_PEND         ((uint32_t)     1)
#define bfoSPI_FLASH_RISR_TXEIR             ((uint32_t)     0)
#define bfwSPI_FLASH_RISR_TXEIR             ((uint32_t)     1)
#define bfoSPI_FLASH_RISR_TXOIR             ((uint32_t)     1)
#define bfwSPI_FLASH_RISR_TXOIR             ((uint32_t)     1)
#define bfoSPI_FLASH_RISR_RXUIR             ((uint32_t)     2)
#define bfwSPI_FLASH_RISR_RXUIR             ((uint32_t)     1)
#define bfoSPI_FLASH_RISR_RXOIR             ((uint32_t)     3)
#define bfwSPI_FLASH_RISR_RXOIR             ((uint32_t)     1)
#define bfoSPI_FLASH_RISR_RXFIR             ((uint32_t)     4)
#define bfwSPI_FLASH_RISR_RXFIR             ((uint32_t)     1)
#define bfoSPI_FLASH_RISR_FSEIR             ((uint32_t)     5)
#define bfwSPI_FLASH_RISR_FSEIR             ((uint32_t)     1)
#define bfoSPI_FLASH_RISR_WBEIR             ((uint32_t)     6)
#define bfwSPI_FLASH_RISR_WBEIR             ((uint32_t)     1)
#define bfoSPI_FLASH_RISR_BYEIR             ((uint32_t)     7)
#define bfwSPI_FLASH_RISR_BYEIR             ((uint32_t)     1)
#define bfoSPI_FLASH_RISR_ACTIR             ((uint32_t)     8)
#define bfwSPI_FLASH_RISR_ACTIR             ((uint32_t)     1)
#define bfoSPI_FLASH_TXOICR_TXOICR          ((uint32_t)     0)
#define bfwSPI_FLASH_TXOICR_TXOICR          ((uint32_t)     1)
#define bfoSPI_FLASH_RXOICR_RXOICR          ((uint32_t)     0)
#define bfwSPI_FLASH_RXOICR_RXOICR          ((uint32_t)     1)
#define bfoSPI_FLASH_RXUICR_RXUICR          ((uint32_t)     0)
#define bfwSPI_FLASH_RXUICR_RXUICR          ((uint32_t)     1)
#define bfoSPI_FLASH_MSTICR_MSTICR          ((uint32_t)     0)
#define bfwSPI_FLASH_MSTICR_MSTICR          ((uint32_t)     1)
#define bfoSPI_FLASH_ICR_ICR                ((uint32_t)     0)
#define bfwSPI_FLASH_ICR_ICR                ((uint32_t)     1)
#define bfoSPI_FLASH_DMACR_RDMAE            ((uint32_t)     0)
#define bfwSPI_FLASH_DMACR_RDMAE            ((uint32_t)     1)
#define bfoSPI_FLASH_DMACR_TDMAE            ((uint32_t)     1)
#define bfwSPI_FLASH_DMACR_TDMAE            ((uint32_t)     1)
#define bfoSPI_FLASH_DMATDLR_DMATDL         ((uint32_t)     0)
#define bfwSPI_FLASH_DMATDLR_DMATDL         ((uint32_t)     3)
#define bfoSPI_FLASH_DMARDLR_DMARDL         ((uint32_t)     0)
#define bfwSPI_FLASH_DMARDLR_DMARDL         ((uint32_t)     3)
#define bfoSPI_FLASH_DR0_dr0                ((uint32_t)     0)
#define bfwSPI_FLASH_DR0_dr0                ((uint32_t)    16)
#define bfoSPI_FLASH_DR1_dr1                ((uint32_t)     0)
#define bfwSPI_FLASH_DR1_dr1                ((uint32_t)    16)
#define bfoSPI_FLASH_DR2_dr2                ((uint32_t)     0)
#define bfwSPI_FLASH_DR2_dr2                ((uint32_t)    16)
#define bfoSPI_FLASH_DR3_dr3                ((uint32_t)     0)
#define bfwSPI_FLASH_DR3_dr3                ((uint32_t)    16)
#define bfoSPI_FLASH_DR4_dr4                ((uint32_t)     0)
#define bfwSPI_FLASH_DR4_dr4                ((uint32_t)    16)
#define bfoSPI_FLASH_DR5_dr5                ((uint32_t)     0)
#define bfwSPI_FLASH_DR5_dr5                ((uint32_t)    16)
#define bfoSPI_FLASH_DR6_dr6                ((uint32_t)     0)
#define bfwSPI_FLASH_DR6_dr6                ((uint32_t)    16)
#define bfoSPI_FLASH_DR7_dr7                ((uint32_t)     0)
#define bfwSPI_FLASH_DR7_dr7                ((uint32_t)    16)
#define bfoSPI_FLASH_DR8_dr8                ((uint32_t)     0)
#define bfwSPI_FLASH_DR8_dr8                ((uint32_t)    16)
#define bfoSPI_FLASH_DR9_dr9                ((uint32_t)     0)
#define bfwSPI_FLASH_DR9_dr9                ((uint32_t)    16)
#define bfoSPI_FLASH_DR10_dr10              ((uint32_t)     0)
#define bfwSPI_FLASH_DR10_dr10              ((uint32_t)    16)
#define bfoSPI_FLASH_DR11_dr11              ((uint32_t)     0)
#define bfwSPI_FLASH_DR11_dr11              ((uint32_t)    16)
#define bfoSPI_FLASH_DR12_dr12              ((uint32_t)     0)
#define bfwSPI_FLASH_DR12_dr12              ((uint32_t)    16)
#define bfoSPI_FLASH_DR13_dr13              ((uint32_t)     0)
#define bfwSPI_FLASH_DR13_dr13              ((uint32_t)    16)
#define bfoSPI_FLASH_DR14_dr14              ((uint32_t)     0)
#define bfwSPI_FLASH_DR14_dr14              ((uint32_t)    16)
#define bfoSPI_FLASH_DR15_dr15              ((uint32_t)     0)
#define bfwSPI_FLASH_DR15_dr15              ((uint32_t)    16)
#define bfoSPI_FLASH_AUTO_LEN_ADDR          ((uint32_t)    16)
#define bfwSPI_FLASH_AUTO_LEN_ADDR          ((uint32_t)     2)
#define bfoSPI_FLASH_AUTO_LEN_DUM           ((uint32_t)     0)
#define bfwSPI_FLASH_AUTO_LEN_DUM           ((uint32_t)    16)

/*  This macro is used to initialize a spi_flash_param structure.  To use
 *  this macro, the relevant C header file must also be included.  This
 *  is generated when a DesignWare device is synthesized.
 */
#define CC_DEFINE_SPI_FLASH_PARAMS(prefix) { \
	prefix ## CC_SPI_FLASH_NUM_SLAVES            ,\
	prefix ## CC_SPI_FLASH_TX_FIFO_DEPTH         ,\
	prefix ## CC_SPI_FLASH_RX_FIFO_DEPTH         ,\
	prefix ## CC_SPI_FLASH_ID                    ,\
	prefix ## CC_SPI_FLASH_DFLT_SCPOL            ,\
	prefix ## CC_SPI_FLASH_DFLT_SCPH             ,\
	prefix ## CC_SPI_FLASH_CLK_PERIOD            ,\
	prefix ## CC_SPI_FLASH_VERSION_ID            ,\
	prefix ## CC_SPI_FLASH_DFLT_SCLK			 \
}

/*
 *  This data type is used to describe read type with multi_channel
 */
enum spi_flash_byte_num {
	DATA_BYTE = 0,
	DATA_HALF = 1,
	DATA_WORD = 2
};

enum flash_rd_multi_type {
	RD_MULTI_NONE = 0x00,
	RD_DUAL_O = 0x01,
	RD_DUAL_IO = 0x02,
	RD_QUAD_O = 0x03,
	RD_QUAD_IO = 0x04
};

/*
 * This data type is used to describe write type with multi_channel
 */
enum flash_wr_multi_type {
	WR_MULTI_NONE = 0x00,
	WR_DUAL_I = 0x01,
	WR_DUAL_II = 0x02,
	WR_QUAD_I = 0x03,
	WR_QUAD_II = 0x04
};

/*
 * This data type is used to describe m25p80 mode type
 */
enum flash_mode_type {
	M25P80_NORMAL = 0,
	M25P80_FAST,
	M25P80_QUAD,
	M25P80_DUAL,
	M25P80_AUTO,
	M25P80_QUAD_WRITE,
	M25P80_DUAL_WRITE,
	M25P80_NORMAL_WRITE,
	M25P80_AUTO_WRITE
};

struct sheipa_spi {
	struct spi_master *master;
	void __iomem *regs;
	void __iomem *auto_regs;
	void *comp_param;
	u32 max_freq;
};

/*
 * This is the structure used for accessing the spi_flash register
 * portmap.
 */
struct spi_flash_portmap {
/* Channel registers
 * The offset address for each of the channel registers
 * is shown for channel 0. For other channel numbers
 * use the following equation.
 * offset = (channel_num * 0x058) + channel_0 offset
 */
	struct {
		volatile uint32_t ctrlr0; /* Control Reg 0 (0x000) */
		volatile uint32_t ctrlr1;
		volatile uint32_t ssienr; /* SPIC enable Reg1 (0x008) */
		volatile uint32_t mwcr;
		volatile uint32_t ser;	/* Slave enable Reg (0x010) */
		volatile uint32_t baudr;
		volatile uint32_t txftlr; /* TX_FIFO threshold level (0x018) */
		volatile uint32_t rxftlr;
		volatile uint32_t txflr; /* TX_FIFO threshold level (0x020) */
		volatile uint32_t rxflr;
		volatile uint32_t sr; /* Destination Status Reg (0x028) */
		volatile uint32_t imr;
		volatile uint32_t isr; /* Interrupt Stauts Reg (0x030) */
		volatile uint32_t risr;

		/* TX_FIFO overflow_INT clear (0x038) */
		volatile uint32_t txoicr;
		volatile uint32_t rxoicr;
		/* RX_FIFO underflow_INT clear (0x040) */
		volatile uint32_t rxuicr;
		volatile uint32_t msticr;
		volatile uint32_t icr;	/* Interrupt clear Reg (0x048) */
		volatile uint32_t dmacr;
		volatile uint32_t dmatdlr; /* DMA TX_data level (0x050) */
		volatile uint32_t dmardlr;
		volatile uint32_t idr;	/* Identiation Scatter Reg (0x058) */
		volatile uint32_t spi_flash_version;
		union {
			volatile uint8_t byte;
			volatile uint16_t half;
			volatile uint32_t word;
		} dr[32];
		volatile uint32_t rd_fast_single;
		/* Read dual data cmd Reg (0x0e4) */
		volatile uint32_t rd_dual_o;
		volatile uint32_t rd_dual_io;
		/* Read quad data cnd Reg (0x0ec) */
		volatile uint32_t rd_quad_o;
		volatile uint32_t rd_quad_io;
		volatile uint32_t wr_single; /* write single cmd Reg (0x0f4) */
		volatile uint32_t wr_dual_i;
		/* write dual addr/data cmd(0x0fc) */
		volatile uint32_t wr_dual_ii;
		volatile uint32_t wr_quad_i;
		/* write quad addr/data cnd(0x104) */
		volatile uint32_t wr_quad_ii;
		volatile uint32_t wr_enable;
		volatile uint32_t rd_status; /* read status cmd Reg (0x10c) */
		volatile uint32_t ctrlr2;
		volatile uint32_t fbaudr; /* fast baud rate Reg (0x114) */
		volatile uint32_t addr_length;
		/* Auto addr length Reg (0x11c) */
		volatile uint32_t auto_length;
		volatile uint32_t valid_cmd;
		volatile uint32_t flash_size; /* Flash size Reg (0x124) */
		volatile uint32_t flush_fifo;
	};
};

struct spi_flash_param {
	uint32_t spi_flash_num_slaves;	/* slaves number */
	uint32_t spi_flash_tx_fifo_depth;	/* TX fifo depth number */
	uint32_t spi_flash_rx_fifo_depth;	/* RX fifo depth number */
	uint32_t spi_flash_idr;	/* ID code */
	uint32_t spi_flash_scpol;	/* Serial clock polarity */
	uint32_t spi_flash_scph;	/* Serial clock phase */
	uint32_t spi_flash_clk_period;	/* serial clock period */
	uint32_t spi_flash_version_id;	/* spi flash ID */
	uint32_t spi_flash_sclk;	/* Serial clock */
};

/* This function is used to wait the SSI is not at busy state. */
static void flash_wait_busy(struct sheipa_spi *dev);
/* This function is used to read status of flash. */
static uint8_t flash_get_status(struct sheipa_spi *dev);

#endif
