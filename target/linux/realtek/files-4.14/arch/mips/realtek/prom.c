#include <linux/init.h>
#include <asm/setup.h>
#include <asm/io.h>

/* 	RTL8197F Uses a non standard uart from
	DesignWare APB (Synopsys)
	Basicly, UART0_TX is 0x24, not 0x0
*/

#ifdef CONFIG_SOC_RTL8197F
#define READREG(r)	*(volatile unsigned int *)(r)

#define REALTEK_UART0_BASE	0xB8147000
#define UART_TX		(((READREG(0xB8000000) & 0xFFFFF000) == 0x8197F000) ? 0x024 : 0x0)

#define BSP_UART0_FCR	((volatile void *)(REALTEK_UART0_BASE + 0x008))
#define BSP_UART0_LSR	((volatile void *)(REALTEK_UART0_BASE + 0x014))
#define BSP_UART0_TX	((volatile void *)(REALTEK_UART0_BASE + UART_TX))

#define BSP_TXRST				0x04
#define BSP_LSR_THRE			0x20

void prom_putchar(char c)
{
	unsigned int busy_cnt = 0;
	do
	{
		/* Prevent Hanging */
		if (busy_cnt++ >= 30000)
		{
			/* Reset Tx FIFO */
			writeb(BSP_TXRST | 0xC0, BSP_UART0_FCR);
			return;
		}
	} while ((readb(BSP_UART0_LSR) & BSP_LSR_THRE) == 0);
	/* Send Character */
	writeb(c, BSP_UART0_TX);
	return;
}

#else
#define REALTEK_UART0_BASE	0xB8002000
#endif

void __init prom_init(void)
{
#ifndef CONFIG_SOC_RTL8197F
	// Standard uart only for 8196E and 8197D
	setup_8250_early_printk_port((unsigned long)REALTEK_UART0_BASE, 2, 30000);
#endif
}

void __init prom_free_prom_memory(void)
{
}
