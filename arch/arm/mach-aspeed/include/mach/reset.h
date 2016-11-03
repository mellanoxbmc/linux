/*
 * arch/arm/mach-aspeed/include/mach/reset.h
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

#ifndef __ASM_ARCH_ASPEED_RESET_H
#define __ASM_ARCH_ASPEED_RESET_H

#define AST_BASE_SCU            0x1E6E2000
#define AST_SCU_RESET		0x04	/* System reset control register */
#define AST_SCU_RESET2		0xD4	/* System reset control register  2*/

#define SCU_PROTECT_UNLOCK	0x1688A8A8 /* Unlock SCU registers: Write
					    * 0x1688A8A8 to this register.
					    * When register is unlocked, the
					    * read back value is 0x00000001,
					    * when locked is 0x00000000.
					    * Writing any non 0x1688A8A8 to
					    * this register will lock it.
					    */

/* AST_SCU_RESET 0x04 Reset control register */
#define SCU_RESET_H264		(0x1 << 26)
#define SCU_RESET_XDMA		(0x1 << 25)
#define SCU_RESET_MCTP		(0x1 << 24)
#define SCU_RESET_P2X		(0x1 << 24)
#define SCU_RESET_ADC		(0x1 << 23)
#define SCU_RESET_JTAG		(0x1 << 22)
#define SCU_RESET_PCIE_DIR	(0x1 << 21)
#define SCU_RESET_PCIE		(0x1 << 19)
#define SCU_RESET_MIC		(0x1 << 18)
#define SCU_RESET_RFX		(0x1 << 17)
#define SCU_RESET_SD		(0x1 << 16)
#define SCU_RESET_USB11		(0x1 << 15)
#define SCU_RESET_USB20		(0x1 << 14)
#define SCU_RESET_CRT		(0x1 << 13)
#define SCU_RESET_MAC1		(0x1 << 12)
#define SCU_RESET_MAC0		(0x1 << 11)
#define SCU_RESET_PECI		(0x1 << 10)
#define SCU_RESET_PWM		(0x1 << 9)
#define SCU_PCI_VGA_DIS		(0x1 << 8)
#define SCU_RESET_2D		(0x1 << 7)
#define SCU_RESET_VIDEO		(0x1 << 6)
#define SCU_RESET_LPC		(0x1 << 5)
#define SCU_RESET_HACE		(0x1 << 4)
#define SCU_RESET_USB_P1	(0x1 << 3)
#define SCU_RESET_I2C		(0x1 << 2)
#define SCU_RESET_AHB		(0x1 << 1)
#define SCU_RESET_SRAM_CTRL	(0x1)

/* AST_SCU_RESET2 0xD4	Reset Control register set 2 */
#define SCU_RESET_CRT3		(0x1 << 8)
#define SCU_RESET_CRT2		(0x1 << 7)
#define SCU_RESET_CRT1		(0x1 << 6)
#define SCU_RESET_CRT0		(0x1 << 5)
#define SCU_RESET_NIC1		(0x1 << 4)
#define SCU_RESET_NIC0		(0x1 << 3)
#define SCU_RESET_RFXDEC	(0x1 << 2)
#define SCU_RESET_BITBLT	(0x1 << 1)
#define SCU_RESET_RFXCMQ	(0x1)

#define SCU_PROTECT_LOCK	0x123456789 /* Lock SCU registers: Write
					    * 0x123456789 to this register
					    */
#define AST_IO_VA		0xf0000000
#define AST_IO(__pa)	((void __iomem *)(((__pa) & 0x001fffff) | AST_IO_VA))

void aspeed_toggle_scu_reset(u32 mask, u32 delay);
void aspeed_scu_multi_func_reset(u32 reg, u32 amask, u32 omask);

#endif /* __ASM_ARCH_ASPEED_RESET_H */
