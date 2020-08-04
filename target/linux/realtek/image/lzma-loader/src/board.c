/*
 * LZMA compressed kernel loader for Realtek SoCs based boards
 *
 * Copyright (C) 2011 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (C) 2017 Weijie Gao <hackpascal@gmail.com>
 * Copyright (C) 2019 Gaspare Bruno <gaspare@anlix.io>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <stddef.h>

#define READREG(r)	*(volatile unsigned int *)(r)
#define WRITEREG(r,v)	*(volatile unsigned int *)(r) = (v)

#define KSEG1ADDR(_x)	(((_x) & 0x1fffffff) | 0xa0000000)

#ifdef __RLX__
#define UART_BASE	0xb8002000
#define UART_TX		0x0
#define UART_SHIFT	24
#else
// For mips RTL8197F
#define UART_BASE	0xb8147000
#define UART_TX		(((READREG(0xB8000000) & 0xFFFFF000) == 0x8197F000) ? 0x024 : 0x0)
#define UART_SHIFT	0
#endif

#define UART_LSR	    0x14
#define UART_LSR_THRE   0x20

#define UART_READ(r)		(READREG(UART_BASE + (r)) >> UART_SHIFT)
#define UART_WRITE(r,v)		WRITEREG(UART_BASE + (r), (v) << UART_SHIFT)

void board_putc(int ch)
{
	while (((UART_READ(UART_LSR)) & UART_LSR_THRE) == 0);
	UART_WRITE(UART_TX, ch);
	while (((UART_READ(UART_LSR)) & UART_LSR_THRE) == 0);
}

void board_init(void)
{

}
