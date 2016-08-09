#include <linux/init.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/reset.h>

static DEFINE_SPINLOCK(aspeed_scu_lock);

/*
 * aspeed_toggle_scu_reset - reset SoC micro-controller.
 * Resets the requested micro-controller.
 */
void aspeed_toggle_scu_reset(u32 mask, u32 delay)
{
	u32 val;

	spin_lock(&aspeed_scu_lock);
	val = readl(AST_IO(AST_BASE_SCU) + AST_SCU_RESET);
	writel(SCU_PROTECT_UNLOCK, AST_IO(AST_BASE_SCU));
	writel(val | mask, AST_IO(AST_BASE_SCU) + AST_SCU_RESET);
	udelay(delay);
	val = readl(AST_IO(AST_BASE_SCU) + AST_SCU_RESET);
	writel(val & ~mask, AST_IO(AST_BASE_SCU) + AST_SCU_RESET);
	spin_unlock(&aspeed_scu_lock);
}
EXPORT_SYMBOL(aspeed_toggle_scu_reset);

/*
 * aspeed_scu_multi_func_reset - Multi-function Pin Control.
 * Resets the requested mutli-function pins.
 */
void aspeed_scu_multi_func_reset(u32 reg, u32 amask, u32 omask)
{
	u32 val = 0;

	spin_lock(&aspeed_scu_lock);
	val = readl(AST_IO(AST_BASE_SCU) + reg);
	val =~ amask;
	writel(SCU_PROTECT_UNLOCK, AST_IO(AST_BASE_SCU));
	writel(val | omask, AST_IO(AST_BASE_SCU) + reg);
	spin_unlock(&aspeed_scu_lock);
}
EXPORT_SYMBOL(aspeed_scu_multi_func_reset);
