/*
 * MOXA ART SoCs timer handling.
 *
 * Copyright (C) 2013 Jonas Jensen
 *
 * Jonas Jensen <jonas.jensen@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/clocksource.h>
#include <linux/bitops.h>

#define TIMER1_BASE		0x00
#define TIMER2_BASE		0x10
#define TIMER3_BASE		0x20

#define REG_COUNT		0x0 /* writable */
#define REG_LOAD		0x4
#define REG_MATCH1		0x8
#define REG_MATCH2		0xC

#define TIMER_CR		0x30
#define TIMER_INTR_STATE	0x34
#define TIMER_INTR_MASK		0x38

/*
 * Moxart TIMER_CR flags:
 *
 * TIMEREG_CR_*_CLOCK	0: PCLK, 1: EXT1CLK
 * TIMEREG_CR_*_INT	overflow interrupt enable bit
 */
#define MOXART_CR_1_ENABLE	BIT(0)
#define MOXART_CR_1_CLOCK	BIT(1)
#define MOXART_CR_1_INT		BIT(2)
#define MOXART_CR_2_ENABLE	BIT(3)
#define MOXART_CR_2_CLOCK	BIT(4)
#define MOXART_CR_2_INT		BIT(5)
#define MOXART_CR_3_ENABLE	BIT(6)
#define MOXART_CR_3_CLOCK	BIT(7)
#define MOXART_CR_3_INT		BIT(8)
#define MOXART_CR_COUNT_UP	BIT(9)

#define MOXART_TIMER1_ENABLE	(MOXART_CR_2_ENABLE | MOXART_CR_1_ENABLE)
#define MOXART_TIMER1_DISABLE	(MOXART_CR_2_ENABLE)

/*
 * The ASpeed variant of the IP block has a different layout
 * for the control register
 */
#define ASPEED_CR_1_ENABLE	BIT(0)
#define ASPEED_CR_1_CLOCK	BIT(1)
#define ASPEED_CR_1_INT		BIT(2)
#define ASPEED_CR_2_ENABLE	BIT(4)
#define ASPEED_CR_2_CLOCK	BIT(5)
#define ASPEED_CR_2_INT		BIT(6)
#define ASPEED_CR_3_ENABLE	BIT(8)
#define ASPEED_CR_3_CLOCK	BIT(9)
#define ASPEED_CR_3_INT		BIT(10)

#define ASPEED_TIMER1_ENABLE	(ASPEED_CR_2_ENABLE | ASPEED_CR_1_ENABLE)
#define ASPEED_TIMER1_DISABLE	(ASPEED_CR_2_ENABLE)

static void __iomem *base;
static unsigned int clock_count_per_tick;
static unsigned int t1_disable_val, t1_enable_val;

static int moxart_shutdown(struct clock_event_device *evt)
{
	writel(t1_disable_val, base + TIMER_CR);
	return 0;
}

static int moxart_set_oneshot(struct clock_event_device *evt)
{
	writel(t1_disable_val, base + TIMER_CR);
	writel(~0, base + TIMER1_BASE + REG_LOAD);
	return 0;
}

static int moxart_set_periodic(struct clock_event_device *evt)
{
	writel(t1_disable_val, base + TIMER_CR);
	writel(clock_count_per_tick, base + TIMER1_BASE + REG_LOAD);
	writel(0, base + TIMER1_BASE + REG_MATCH1);
	writel(t1_enable_val, base + TIMER_CR);

	return 0;
}

static int moxart_clkevt_next_event(unsigned long cycles,
				    struct clock_event_device *unused)
{
	u32 u;

	writel(t1_disable_val, base + TIMER_CR);

	u = readl(base + TIMER1_BASE + REG_COUNT) - cycles;
	writel(u, base + TIMER1_BASE + REG_MATCH1);

	writel(t1_enable_val, base + TIMER_CR);

	return 0;
}

static struct clock_event_device moxart_clockevent = {
	.name			= "moxart_timer",
	.rating			= 200,
	.features		= CLOCK_EVT_FEAT_PERIODIC |
				  CLOCK_EVT_FEAT_ONESHOT,
	.set_state_shutdown	= moxart_shutdown,
	.set_state_periodic	= moxart_set_periodic,
	.set_state_oneshot	= moxart_set_oneshot,
	.tick_resume		= moxart_set_oneshot,
	.set_next_event		= moxart_clkevt_next_event,
};

static irqreturn_t moxart_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static struct irqaction moxart_timer_irq = {
	.name		= "moxart-timer",
	.flags		= IRQF_TIMER,
	.handler	= moxart_timer_interrupt,
	.dev_id		= &moxart_clockevent,
};

static void __init __moxart_timer_init(struct device_node *node)
{
	int ret, irq;
	unsigned long pclk;
	struct clk *clk;

	base = of_iomap(node, 0);
	if (!base)
		panic("%s: of_iomap failed\n", node->full_name);

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0)
		panic("%s: irq_of_parse_and_map failed\n", node->full_name);

	ret = setup_irq(irq, &moxart_timer_irq);
	if (ret)
		panic("%s: setup_irq failed\n", node->full_name);

	clk = of_clk_get(node, 0);
	if (IS_ERR(clk))
		panic("%s: of_clk_get failed\n", node->full_name);

	pclk = clk_get_rate(clk);

	if (clocksource_mmio_init(base + TIMER2_BASE + REG_COUNT,
				  "moxart_timer", pclk, 200, 32,
				  clocksource_mmio_readl_down))
		panic("%s: clocksource_mmio_init failed\n", node->full_name);

	clock_count_per_tick = DIV_ROUND_CLOSEST(pclk, HZ);

	/* Clean up match registers we will still get an occasional interrupt
	 * from timer 2 but I haven't enabled it for now
	 */
	writel(0, base + TIMER1_BASE + REG_MATCH1);
	writel(0, base + TIMER1_BASE + REG_MATCH2);
	writel(0, base + TIMER2_BASE + REG_MATCH1);
	writel(0, base + TIMER2_BASE + REG_MATCH2);

	/* Start  timer 2 rolling as our main wall clock source, keep timer 1
	 * disabled
	 */	
	writel(0, base + TIMER_CR);
	writel(~0, base + TIMER2_BASE + REG_LOAD);
	writel(t1_disable_val, base + TIMER_CR);


	moxart_clockevent.cpumask = cpumask_of(0);
	moxart_clockevent.irq = irq;

	/*
	 * documentation is not publicly available:
	 * min_delta / max_delta obtained by trial-and-error,
	 * max_delta 0xfffffffe should be ok because count
	 * register size is u32
	 */
	clockevents_config_and_register(&moxart_clockevent, pclk,
					0x4, 0xfffffffe);
}

static void __init moxart_timer_init(struct device_node *node)
{
	t1_enable_val = MOXART_TIMER1_ENABLE;
	t1_disable_val = MOXART_TIMER1_DISABLE;
	__moxart_timer_init(node);
}

static void __init aspeed_timer_init(struct device_node *node)
{
	t1_enable_val = ASPEED_TIMER1_ENABLE;
	t1_disable_val = ASPEED_TIMER1_DISABLE;
	__moxart_timer_init(node);
}

CLOCKSOURCE_OF_DECLARE(moxart, "moxa,moxart-timer", moxart_timer_init);
CLOCKSOURCE_OF_DECLARE(aspeed, "aspeed,timer", aspeed_timer_init);
