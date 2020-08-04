/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2019 - Gaspare Bruno <gaspare@anlix.io>
 *
 *   Realtek RTL8196E and RTL8197D have two clocks of 28 bits
 */

#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/clk-provider.h>
#include <linux/sched_clock.h>
#include <linux/clk.h>

#include <asm/mach-realtek/realtek_mem.h>

__iomem void *_timer_membase;
#define tc_w32(val, reg) __raw_writel(val, _timer_membase + reg)
#define tc_r32(reg)		 __raw_readl(_timer_membase + reg)

// Timer Registers and bits
#define REALTEK_TC_REG_DATA0		0x00
#define REALTEK_TC_REG_DATA1		0x04
#define REALTEK_TC_REG_COUNT0		0x08
#define REALTEK_TC_REG_COUNT1		0x0c
#define REALTEK_TC_REG_CTRL			0x10
#define REALTEK_TC_CTRL_TC0_EN		BIT(31)	// Enable Timer0
#define REALTEK_TC_CTRL_TC0_MODE	BIT(30)	// 0 Counter, 1 Timer
#define REALTEK_TC_CTRL_TC1_EN		BIT(29)	// Enable Timer1
#define REALTEK_TC_CTRL_TC1_MODE	BIT(28) // 0 Counter, 1 Timer
#define REALTEK_TC_REG_IR			0x14
#define REALTEK_TC_IR_TC0_EN		BIT(31) // Enable Timer Interrupts
#define REALTEK_TC_IR_TC1_EN		BIT(30)
#define REALTEK_TC_IR_TC0_PENDING	BIT(29)
#define REALTEK_TC_IR_TC1_PENDING	BIT(28)
#define REALTEK_TC_REG_CLOCK_DIV	0x18	// Clock Divider N TimerClock=(BaseClock/N)

// Only the 28 higher bits are valid in the timer register counter
#define REALTEK_TIMER_RESOLUTION 28
#define RTLADJ_TICK(x)  (x>>(32-REALTEK_TIMER_RESOLUTION))

static u64 rtl819x_tc1_count_read(struct clocksource *cs)
{
	return RTLADJ_TICK(tc_r32(REALTEK_TC_REG_COUNT1));
}

static u64 __maybe_unused notrace rtl819x_read_sched_clock(void)
{
	return RTLADJ_TICK(tc_r32(REALTEK_TC_REG_COUNT1));
}

static struct clocksource rtl819x_clocksource = {
	.name	= "RTL819X counter",
	.read	= rtl819x_tc1_count_read,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

void rtl819x_clocksource_init(unsigned long freq)
{
	u32 val;

	// Use Second clock as monotonic
	tc_w32(0xfffffff0, REALTEK_TC_REG_DATA1);

	val = tc_r32(REALTEK_TC_REG_CTRL);
	val |= REALTEK_TC_CTRL_TC1_EN | REALTEK_TC_CTRL_TC1_MODE;
	tc_w32(val, REALTEK_TC_REG_CTRL);

	val = tc_r32(REALTEK_TC_REG_IR);
	val |= REALTEK_TC_IR_TC1_PENDING;
	val &= ~REALTEK_TC_IR_TC1_EN;
	tc_w32(val, REALTEK_TC_REG_IR);

	rtl819x_clocksource.rating = 200;
	rtl819x_clocksource.mask = CLOCKSOURCE_MASK(REALTEK_TIMER_RESOLUTION),

	clocksource_register_hz(&rtl819x_clocksource, freq);

#ifndef CONFIG_CPU_FREQ
	sched_clock_register(rtl819x_read_sched_clock, REALTEK_TIMER_RESOLUTION, freq);
#endif
}


static int rtl819x_set_state_shutdown(struct clock_event_device *cd)
{
	u32 val;

	// Disable Timer 
	val = tc_r32(REALTEK_TC_REG_CTRL);
	val &= ~(REALTEK_TC_CTRL_TC0_EN);
	tc_w32(val, REALTEK_TC_REG_CTRL);

	// Disable Interrupts
	val = tc_r32(REALTEK_TC_REG_IR);
	val &= ~REALTEK_TC_IR_TC0_EN;
	tc_w32(val, REALTEK_TC_REG_IR);
	return 0;
}

static int rtl819x_set_state_oneshot(struct clock_event_device *cd)
{
	u32 val;

	// Disable Timer and Set as Counter (It will be auto enabled)
	val = tc_r32(REALTEK_TC_REG_CTRL);
	val &= ~(REALTEK_TC_CTRL_TC0_EN | REALTEK_TC_CTRL_TC0_MODE);
	tc_w32(val, REALTEK_TC_REG_CTRL);

	// Enable Interrupts
	val = tc_r32(REALTEK_TC_REG_IR);
	val |= REALTEK_TC_IR_TC0_EN | REALTEK_TC_IR_TC0_PENDING;
	tc_w32(val, REALTEK_TC_REG_IR);
	return 0;
}

static int rtl819x_timer_set_next_event(unsigned long delta, struct clock_event_device *evt)
{
	u32 val;

	// Disable Timer
	val = tc_r32(REALTEK_TC_REG_CTRL);
	val &= ~REALTEK_TC_CTRL_TC0_EN;
	tc_w32(val, REALTEK_TC_REG_CTRL);

	tc_w32(delta<<(32-REALTEK_TIMER_RESOLUTION), REALTEK_TC_REG_DATA0);

	// Reenable Timer
	val |= REALTEK_TC_CTRL_TC0_EN;
	tc_w32(val, REALTEK_TC_REG_CTRL);

	return 0;
}

static irqreturn_t rtl819x_timer_interrupt(int irq, void *dev_id)
{
	u32 tc0_irs;
	struct clock_event_device *cd = dev_id;

	/* TC0 interrupt acknowledge */
	tc0_irs = tc_r32(REALTEK_TC_REG_IR);
	tc0_irs |= REALTEK_TC_IR_TC0_PENDING;
	tc_w32(tc0_irs, REALTEK_TC_REG_IR);

	cd->event_handler(cd);

	return IRQ_HANDLED;
}

static struct clock_event_device rtl819x_clockevent = {
	.rating			= 100,
	.features		= CLOCK_EVT_FEAT_ONESHOT,
	.set_next_event = rtl819x_timer_set_next_event,
	.set_state_oneshot = rtl819x_set_state_oneshot,
	.set_state_shutdown = rtl819x_set_state_shutdown,
};

static struct irqaction rtl819x_timer_irqaction = {
	.handler = rtl819x_timer_interrupt,
	.flags = IRQF_TIMER,
	.dev_id = &rtl819x_clockevent,
};

static int __init rtl819x_timer_init(struct device_node *np)
{
	struct resource res;
	struct clk *clk;
	unsigned long timer_rate;
	u32 div_fac;

	if (of_address_to_resource(np, 0, &res))
		panic("Failed to get resource for %s", np->name);

	_timer_membase = ioremap_nocache(res.start, resource_size(&res));
	if(!_timer_membase)
		panic("Failed to map memory for %s", np->name);

	rtl819x_clockevent.name = np->name;
	rtl819x_timer_irqaction.name = np->name;

	rtl819x_clockevent.irq = irq_of_parse_and_map(np, 0);
	rtl819x_clockevent.cpumask = cpumask_of(0);

	clk = of_clk_get(np, 0);
	if(!clk)
		panic("Cant find reference clock for timer!\n");

 	timer_rate = clk_get_rate(clk);

	// Realtek use a default bus rate of 200MHz
	div_fac = 200000000/timer_rate;

	// Higher 16 bits are the divider factor
	tc_w32(div_fac<<16, REALTEK_TC_REG_CLOCK_DIV);

	rtl819x_clocksource_init(timer_rate);

	clockevents_config_and_register(&rtl819x_clockevent, timer_rate, 0x300, 0x7fffffff);

	setup_irq(rtl819x_clockevent.irq, &rtl819x_timer_irqaction);

	pr_info("%s: running - mult: %d, shift: %d, IRQ: %d, CLK: %lu.%03luMHz\n",
		np->name, rtl819x_clockevent.mult, rtl819x_clockevent.shift, 
		rtl819x_clockevent.irq, timer_rate / 1000000, (timer_rate / 1000) % 1000);

	return 0;
}

TIMER_OF_DECLARE(rtl819x_timer, "realtek,rtl819x-timer", rtl819x_timer_init);
