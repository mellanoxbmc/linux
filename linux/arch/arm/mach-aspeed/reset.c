/*
 * arch/arm/mach-aspeed/reset.c
 * Copyright (c) 2016 Vadim Pasternak <vadimp@mellanox.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/init.h>
#include <linux/module.h>
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
	val &= ~amask;
	writel(SCU_PROTECT_UNLOCK, AST_IO(AST_BASE_SCU));
	writel(val | omask, AST_IO(AST_BASE_SCU) + reg);
	spin_unlock(&aspeed_scu_lock);
}
EXPORT_SYMBOL(aspeed_scu_multi_func_reset);

MODULE_AUTHOR("Vadim Pasternak (vadimp@mellanox.com)");
MODULE_DESCRIPTION("Aspeed SoC controllers reset");
MODULE_LICENSE("GPL v2");
