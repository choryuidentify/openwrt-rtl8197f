/*
 * LZMA compressed kernel loader for Realtek SoCs based boards
 *
 * Copyright (C) 2011 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (C) 2017 Weijie Gao <hackpascal@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 */

#include "cacheops.h"

#ifdef __RLX__

#define LEXRA_CCTL_BARRIER						\
	do {								\
		__asm__ __volatile__ ( "" : : : "memory");		\
	} while (0)

void flush_cache(unsigned long start_addr, unsigned long size)
{
	write_c0_cctl(0);
	LEXRA_CCTL_BARRIER;
	write_c0_cctl(CCTL_DWB);
	LEXRA_CCTL_BARRIER;
	write_c0_cctl(0);
	LEXRA_CCTL_BARRIER;
	write_c0_cctl(CCTL_DInval);
	LEXRA_CCTL_BARRIER;
	write_c0_cctl(0);
	LEXRA_CCTL_BARRIER;
	write_c0_cctl(CCTL_IInval);
}

#else

#define cache_op(op,addr)						\
	__asm__ __volatile__(						\
	"	.set	push					\n"	\
	"	.set	noreorder				\n"	\
	"	.set	mips3\n\t				\n"	\
	"	cache	%0, %1					\n"	\
	"	.set	pop					\n"	\
	:								\
	: "i" (op), "R" (*(unsigned char *)(addr)))

void flush_cache(unsigned long start_addr, unsigned long size)
{
	unsigned long lsize = 32;
	unsigned long addr = start_addr & ~(lsize - 1);
	unsigned long aend = (start_addr + size - 1) & ~(lsize - 1);

	while (1) {
		cache_op(Hit_Writeback_Inv_D, addr);
		cache_op(Hit_Invalidate_I, addr);
		if (addr == aend)
			break;
		addr += lsize;
	}
}

#endif

