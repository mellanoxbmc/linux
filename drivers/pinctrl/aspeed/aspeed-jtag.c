/*
 * Copyright (c) 2017 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2017 Oleksandr Shamray <oleksandrs@mellanox.com>
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

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <linux/aspeed_jtag.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <mach/reset.h>

#define AST_JTAG_DATA			0x00
#define AST_JTAG_INST			0x04
#define AST_JTAG_CTRL			0x08
#define AST_JTAG_ISR			0x0C
#define AST_JTAG_SW			0x10
#define AST_JTAG_TCK			0x14
#define AST_JTAG_IDLE			0x18

/* AST_JTAG_CTRL - 0x08 : Engine Control */
#define JTAG_ENG_EN			(0x1 << 31)
#define JTAG_ENG_OUT_EN			(0x1 << 30)
#define JTAG_FORCE_TMS			(0x1 << 29)
#define JTAG_IR_UPDATE			(0x1 << 26)
#define JTAG_INST_LEN_MASK		(0x3f << 20)
#define JTAG_SET_INST_LEN(x)		(x << 20)
#define JTAG_SET_INST_MSB		(0x1 << 19)
#define JTAG_TERMINATE_INST		(0x1 << 18)
#define JTAG_LAST_INST			(0x1 << 17)
#define JTAG_INST_EN			(0x1 << 16)
#define JTAG_DATA_LEN_MASK		(0x3f << 4)
#define JTAG_DR_UPDATE			(0x1 << 10)
#define JTAG_DATA_LEN(x)		(x << 4)
#define JTAG_SET_DATA_MSB		(0x1 << 3)
#define JTAG_TERMINATE_DATA		(0x1 << 2)
#define JTAG_LAST_DATA			(0x1 << 1)
#define JTAG_DATA_EN			(0x1)

/* AST_JTAG_ISR	- 0x0C : INterrupt status and enable */
#define JTAG_INST_PAUSE			(0x1 << 19)
#define JTAG_INST_COMPLETE		(0x1 << 18)
#define JTAG_DATA_PAUSE			(0x1 << 17)
#define JTAG_DATA_COMPLETE		(0x1 << 16)

#define JTAG_INST_PAUSE_EN		(0x1 << 3)
#define JTAG_INST_COMPLETE_EN		(0x1 << 2)
#define JTAG_DATA_PAUSE_EN		(0x1 << 1)
#define JTAG_DATA_COMPLETE_EN		(0x1)

/* AST_JTAG_SW	- 0x10 : Software Mode and Status */
#define JTAG_SW_MODE_EN			(0x1 << 19)
#define JTAG_SW_MODE_TCK		(0x1 << 18)
#define JTAG_SW_MODE_TMS		(0x1 << 17)
#define JTAG_SW_MODE_TDIO		(0x1 << 16)
#define JTAG_STS_INST_PAUSE		(0x1 << 2)
#define JTAG_STS_DATA_PAUSE		(0x1 << 1)
#define JTAG_STS_ENG_IDLE		(0x1)

/* AST_JTAG_TCK	- 0x14 : TCK Control */
#define JTAG_TCK_INVERSE		(0x1 << 31)
#define JTAG_TCK_DIVISOR_MASK		(0x7ff)
#define JTAG_GET_TCK_DIVISOR(x)		(x & 0x7ff)

/*  AST_JTAG_IDLE - 0x18 : Ctroller set for go to IDLE */
#define JTAG_GO_IDLE			(0x1)

struct aspeed_jtag_info {
	void __iomem		*reg_base;
	struct device		*dev;
	struct clk		*pclk;
	u8			status; /* 0: idle,
					 * 1: irpause,
					 * 2:drpause
					 */
	int			irq; /* JTAG IRQ number */
	u32			flag;
	wait_queue_head_t	jtag_wq;
	bool			is_open;
};

struct aspeed_jtag_info *aspeed_jtag;
static char *end_status_str[] = {"idle", "ir pause", "drpause"};

static DEFINE_SPINLOCK(jtag_state_lock);

static inline u32
aspeed_jtag_read(struct aspeed_jtag_info *aspeed_jtag, u32 reg)
{
	int val;

	val = readl(aspeed_jtag->reg_base + reg);
	dev_dbg(aspeed_jtag->dev, "RD reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
}

static inline void
aspeed_jtag_write(struct aspeed_jtag_info *aspeed_jtag, u32 val, u32 reg)
{
	dev_dbg(aspeed_jtag->dev, "WR reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, aspeed_jtag->reg_base + reg);
}

u32 aspeed_get_pclk(struct clk *pclk)
{
	unsigned long timer_freq;

	timer_freq = clk_get_rate(pclk);

	return timer_freq;
}

void
aspeed_jtag_set_freq(struct aspeed_jtag_info *aspeed_jtag, unsigned int freq)
{
	u16 i = 0;
	u32 apb_frq = 0;

	dev_dbg(aspeed_jtag->dev, "set frq : %d\n", freq);

	apb_frq = aspeed_get_pclk(aspeed_jtag->pclk);

	i = (apb_frq / freq);
	if (apb_frq % freq == 0)
		i--;

	aspeed_jtag_write(aspeed_jtag,
			  ((aspeed_jtag_read(aspeed_jtag, AST_JTAG_TCK) &
					      ~JTAG_TCK_DIVISOR_MASK) | i),
					      AST_JTAG_TCK);
}

unsigned int aspeed_jtag_get_freq(struct aspeed_jtag_info *aspeed_jtag)
{
	return aspeed_get_pclk(aspeed_jtag->pclk) /
	       (JTAG_GET_TCK_DIVISOR(aspeed_jtag_read(aspeed_jtag,
						      AST_JTAG_TCK)) + 1);
}

void dummy(struct aspeed_jtag_info *aspeed_jtag, unsigned int cnt)
{
	int i = 0;

	for (i = 0; i < cnt; i++)
		aspeed_jtag_read(aspeed_jtag, AST_JTAG_SW);
}

static u8 tck_cycle(struct aspeed_jtag_info *aspeed_jtag, u8 TMS, u8 TDI)
{
	u8 tdo;

	/* TCK = 0 */
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN |
			  (TMS * JTAG_SW_MODE_TMS) | (TDI * JTAG_SW_MODE_TDIO),
			  AST_JTAG_SW);

	dummy(aspeed_jtag, 10);

	/* TCK = 1 */
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TCK |
			  (TMS * JTAG_SW_MODE_TMS) | (TDI * JTAG_SW_MODE_TDIO),
			  AST_JTAG_SW);

	if (aspeed_jtag_read(aspeed_jtag, AST_JTAG_SW) & JTAG_SW_MODE_TDIO)
		tdo = 1;
	else
		tdo = 0;

	dummy(aspeed_jtag, 10);

	/* TCK = 0 */
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | (TMS *
			  JTAG_SW_MODE_TMS) | (TDI * JTAG_SW_MODE_TDIO),
			  AST_JTAG_SW);

	return tdo;
}

void aspeed_jtag_wait_instruction_pause(
		struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag ==
				 JTAG_INST_PAUSE));

	aspeed_jtag->flag = 0;
}

void aspeed_jtag_wait_instruction_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag ==
				 JTAG_INST_COMPLETE));

	aspeed_jtag->flag = 0;
}

void aspeed_jtag_wait_data_pause_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag ==
				 JTAG_DATA_PAUSE));

	aspeed_jtag->flag = 0;
}

void aspeed_jtag_wait_data_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag ==
				 JTAG_DATA_COMPLETE));

	aspeed_jtag->flag = 0;
}

/* JTAG_reset() is to generate at least 9 TMS high and 1 TMS low to force
 * devices into Run-Test/Idle State.
 */
void aspeed_jtag_run_test_idle(struct aspeed_jtag_info *aspeed_jtag,
			       struct runtest_idle *runtest)
{
	int i = 0;

	dev_dbg(aspeed_jtag->dev,
		"aspeed_jtag runtest, status:%d, mode:%s, end:%s, reset:%d, tck:%d\n",
		aspeed_jtag->status,
		runtest->mode ? "SW" : "HW",
		end_status_str[runtest->end],
		runtest->reset,
		runtest->tck);

	if (runtest->mode) {
		/* SW mode from idle , from pause,  -- > to pause, to idle */
		if (runtest->reset) {
			for (i = 0; i < 10; i++)
				tck_cycle(aspeed_jtag, 1, 0);
		}

		switch (aspeed_jtag->status) {
		case 0:
			if (runtest->end == JTAG_STATE_IRPAUSE) {
				tck_cycle(aspeed_jtag, 1, 0); /* DRSCan */
				tck_cycle(aspeed_jtag, 1, 0); /* IRSCan */
				tck_cycle(aspeed_jtag, 0, 0); /* IRCap */
				tck_cycle(aspeed_jtag, 1, 0); /* IRExit1 */
				tck_cycle(aspeed_jtag, 0, 0); /* IRPause */
				aspeed_jtag->status = 1;
			} else if (runtest->end == JTAG_STATE_DRPAUSE) {
				tck_cycle(aspeed_jtag, 1, 0); /* DRSCan */
				tck_cycle(aspeed_jtag, 0, 0); /* DRCap */
				tck_cycle(aspeed_jtag, 1, 0); /* DRExit1 */
				tck_cycle(aspeed_jtag, 0, 0); /* DRPause */
				aspeed_jtag->status = 1;
			} else { /* JTAG_STATE_IDLE*/
				tck_cycle(aspeed_jtag, 0, 0);/* IDLE */
				aspeed_jtag->status = 0;
			}
			break;

		case 1:
			/* from IR/DR Pause */
			if (runtest->end == JTAG_STATE_IRPAUSE) {
				tck_cycle(aspeed_jtag, 1, 0); /* Exit2 IR/DR */
				tck_cycle(aspeed_jtag, 1, 0); /* Updt IR/DR */
				tck_cycle(aspeed_jtag, 1, 0); /* DRSCan */
				tck_cycle(aspeed_jtag, 1, 0); /* IRSCan */
				tck_cycle(aspeed_jtag, 0, 0); /* IRCap */
				tck_cycle(aspeed_jtag, 1, 0); /* IRExit1 */
				tck_cycle(aspeed_jtag, 0, 0); /* IRPause */
				aspeed_jtag->status = 1;
			} else if (runtest->end == JTAG_STATE_DRPAUSE) {
				tck_cycle(aspeed_jtag, 1, 0); /* Exit2 IR/DR */
				tck_cycle(aspeed_jtag, 1, 0); /* Update IR/DR */
				tck_cycle(aspeed_jtag, 1, 0); /* DRSCan */
				tck_cycle(aspeed_jtag, 0, 0); /* DRCap */
				tck_cycle(aspeed_jtag, 1, 0); /* DRExit1 */
				tck_cycle(aspeed_jtag, 0, 0); /* DRPause */
				aspeed_jtag->status = 1;
			} else {
				tck_cycle(aspeed_jtag, 1, 0); /* Exit2 IR/DR */
				tck_cycle(aspeed_jtag, 1, 0); /* Updt IR/DR */
				tck_cycle(aspeed_jtag, 0, 0); /* IDLE */
				aspeed_jtag->status = 0;
			}
			break;

		default:
			dev_err(aspeed_jtag->dev,
				"aspeed_jtag_run_test_idle error\n");
			break;
		}

		/* stay on IDLE for at least  TCK cycle */
		for (i = 0; i < runtest->tck; i++)
			tck_cycle(aspeed_jtag, 0, 0);
	} else {
		/* dis sw mode */
		aspeed_jtag_write(aspeed_jtag, 0, AST_JTAG_SW);
		mdelay(1);
		/* x TMS high + 1 TMS low */
		if (runtest->reset)
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN |
					  JTAG_ENG_OUT_EN | JTAG_FORCE_TMS,
					  AST_JTAG_CTRL);
		else
			aspeed_jtag_write(aspeed_jtag, JTAG_GO_IDLE,
					  AST_JTAG_IDLE);
		mdelay(1);
		aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN |
				  JTAG_SW_MODE_TDIO, AST_JTAG_SW);
		aspeed_jtag->status = 0;
	}
}

int aspeed_jtag_sir_xfer(struct aspeed_jtag_info *aspeed_jtag,
			 struct sir_xfer *sir)
{
	int i = 0;

	dev_dbg(aspeed_jtag->dev,
		"aspeed_jtag SIR xfer, mode:%s, TDI:%x, ENDIR : %d, len:%d\n",
		sir->mode ? "SW" : "HW", sir->tdi, sir->endir, sir->length);

	if (sir->mode) {
		aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN |
				  JTAG_SW_MODE_TDIO, AST_JTAG_SW);
		if (aspeed_jtag->status) {
			/* from IR/DR Pause */
			tck_cycle(aspeed_jtag, 1, 0); /* Exit2 IR / DR */
			tck_cycle(aspeed_jtag, 1, 0); /* Update IR /DR */
		}

		tck_cycle(aspeed_jtag, 1, 0); /* DRSCan */
		tck_cycle(aspeed_jtag, 1, 0); /* IRSCan */
		tck_cycle(aspeed_jtag, 0, 0); /* CapIR */
		tck_cycle(aspeed_jtag, 0, 0); /* ShiftIR */

		sir->tdo = 0;
		for (i = 0; i < sir->length; i++) {
			if (i == (sir->length - 1)) {
				/* IRExit1 */
				sir->tdo |= tck_cycle(aspeed_jtag, 1,
						      sir->tdi & 0x1);
			} else {
				/* ShiftIR */
				sir->tdo |= tck_cycle(aspeed_jtag, 0,
						      sir->tdi & 0x1);
				sir->tdi >>= 1;
				sir->tdo <<= 1;
			}
		}

		tck_cycle(aspeed_jtag, 0, 0); /* IRPause */

		/* stop pause */
		if (sir->endir == JTAG_STATE_IDLE) {
			/* go to idle */
			tck_cycle(aspeed_jtag, 1, 0); /* IRExit2 */
			tck_cycle(aspeed_jtag, 1, 0); /* IRUpdate */
			tck_cycle(aspeed_jtag, 0, 0); /* IDLE */
		}
	} else {
		/* hw mode - disable sw mode */
		aspeed_jtag_write(aspeed_jtag, 0, AST_JTAG_SW);
		aspeed_jtag_write(aspeed_jtag, sir->tdi, AST_JTAG_INST);

		if (sir->endir != JTAG_STATE_IDLE) {
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN |
					  JTAG_ENG_OUT_EN |
					  JTAG_SET_INST_LEN(sir->length),
					  AST_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN |
					  JTAG_ENG_OUT_EN |
					  JTAG_SET_INST_LEN(sir->length) |
					  JTAG_INST_EN, AST_JTAG_CTRL);
			aspeed_jtag_wait_instruction_pause(aspeed_jtag);
		} else {
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN |
					  JTAG_ENG_OUT_EN | JTAG_LAST_INST |
					  JTAG_SET_INST_LEN(sir->length),
					  AST_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN |
					  JTAG_ENG_OUT_EN | JTAG_LAST_INST |
					  JTAG_SET_INST_LEN(sir->length) |
					  JTAG_INST_EN, AST_JTAG_CTRL);
			aspeed_jtag_wait_instruction_complete(aspeed_jtag);
		}

		sir->tdo = aspeed_jtag_read(aspeed_jtag, AST_JTAG_INST);
		if (sir->endir == JTAG_STATE_IDLE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN |
				  JTAG_SW_MODE_TDIO, AST_JTAG_SW);
		}
	}

	aspeed_jtag->status = sir->endir;

	return 0;
}

unsigned int *copy_from_user_bit_data(void *udata, unsigned int bit_size)
{
	void *kdata;
	int size = 0;
	int ret = 0;

	size = (bit_size + 7) / 8;
	kdata = kzalloc(size, GFP_KERNEL);

	if (!kdata)
		return NULL;

	ret = copy_from_user(kdata, udata, size);

	if (ret) {
		kfree(kdata);
		return NULL;
	}
	return kdata;
}

int copy_to_user_bit_data(void *udata, void *kdata, unsigned int bit_size)
{
	int ret = 0;
	int size = 0;

	size  = (bit_size + 7) / 8;
	ret = copy_to_user(udata, kdata, size);
	return ret;
}

void aspeed_jtag_push_data(struct aspeed_jtag_info *aspeed_jtag,
			   u32 shift_bits)
{
	dev_dbg(aspeed_jtag->dev, "shit bits %d\n", shift_bits);
	aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
			  JTAG_DATA_LEN(shift_bits),
			  AST_JTAG_CTRL);
	aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
			  JTAG_DATA_LEN(shift_bits) | JTAG_DATA_EN,
			  AST_JTAG_CTRL);
	aspeed_jtag_wait_data_pause_complete(aspeed_jtag);
}

void aspeed_jtag_push_data_last(struct aspeed_jtag_info *aspeed_jtag,
				u32 shift_bits,
				enum aspeed_jtag_enstate enddr)
{
	if (enddr != JTAG_STATE_IDLE) {
		dev_dbg(aspeed_jtag->dev,
			"DR Keep Pause\n");
		aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
			JTAG_DR_UPDATE | JTAG_DATA_LEN(shift_bits),
			AST_JTAG_CTRL);
		aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
			JTAG_DR_UPDATE | JTAG_DATA_LEN(shift_bits) |
			JTAG_DATA_EN,
			AST_JTAG_CTRL);
		aspeed_jtag_wait_data_pause_complete(aspeed_jtag);
	} else {
		dev_dbg(aspeed_jtag->dev,
			"DR go IDLE\n");
		aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
				  JTAG_LAST_DATA | JTAG_DATA_LEN(shift_bits),
				  AST_JTAG_CTRL);
		aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
				  JTAG_LAST_DATA | JTAG_DATA_LEN(shift_bits) |
				  JTAG_DATA_EN,
				  AST_JTAG_CTRL);
		aspeed_jtag_wait_data_complete(aspeed_jtag);

		aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN |
				  JTAG_SW_MODE_TDIO,
				  AST_JTAG_SW);
	}
}

int aspeed_jtag_sdr_xfer(struct aspeed_jtag_info *aspeed_jtag,
			 struct sdr_xfer *sdr)
{
	unsigned int index = 0;
	unsigned int i = 0;
	u32 shift_bits = 0;
	u32 tdo = 0;
	u32 remain_xfer = sdr->length;
	char dbg_str[256];
	unsigned int *kdata;
	unsigned int pos = 0;
	int ret = 0;

	kdata = copy_from_user_bit_data(sdr->tdio, sdr->length);
	if (!kdata)
		return -1;

	for (index = 0, i = 0; index < sdr->length; index += 32, i++)
		pos += sprintf(&dbg_str[pos], "0x%08x ", kdata[i]);

	dev_dbg(aspeed_jtag->dev,
		"aspeed_jtag SDR xfer, mode:%s, ENDDR:%d, len:%d, TDI[ %s ]\n",
		sdr->mode ? "SW" : "HW", sdr->enddr, remain_xfer, dbg_str);
	index = 0;

	if (sdr->mode) {
		/* SW mode */
		if (aspeed_jtag->status) {
			/* from IR/DR Pause */
			tck_cycle(aspeed_jtag, 1, 0); /* Exit2 IR / DR */
			tck_cycle(aspeed_jtag, 1, 0); /* Update IR /DR */
		}

		tck_cycle(aspeed_jtag, 1, 0); /* DRScan */
		tck_cycle(aspeed_jtag, 0, 0); /* DRCap */
		tck_cycle(aspeed_jtag, 0, 0); /* DRShift */

		while (remain_xfer) {
			if (sdr->direct) {
				/* write */
				if ((shift_bits % 32) == 0)
					dev_dbg(aspeed_jtag->dev,
						"W dr->dr_data[%d]: %x\n",
						index, kdata[index]);

				tdo = (kdata[index] >> (shift_bits % 32)) &
				       0x1;
				dev_dbg(aspeed_jtag->dev, "%d\n", tdo);
				if (remain_xfer == 1)
					/* DRExit1 */
					tck_cycle(aspeed_jtag, 1, tdo);
				else
					/* DRShit */
					tck_cycle(aspeed_jtag, 0, tdo);
			} else {
				/* read */
				if (remain_xfer == 1)
					/* DRExit1 */
					tdo = tck_cycle(aspeed_jtag, 1, tdo);
				else
					/* DRShit */
					tdo = tck_cycle(aspeed_jtag, 0, tdo);

				dev_dbg(aspeed_jtag->dev, "%d\n", tdo);
				kdata[index] |= (tdo << (shift_bits % 32));

				if ((shift_bits % 32) == 0)
					dev_dbg(aspeed_jtag->dev,
						"R dr->dr_data[%d]: %x\n",
						index, kdata[index]);
			}
			shift_bits++;
			remain_xfer--;
			if ((shift_bits % 32) == 0)
				index++;
		}

		tck_cycle(aspeed_jtag, 0, 0); /* DRPause */

		if (sdr->enddr == JTAG_STATE_IDLE) {
			tck_cycle(aspeed_jtag, 1, 0); /* DRExit2 */
			tck_cycle(aspeed_jtag, 1, 0); /* DRUpdate */
			tck_cycle(aspeed_jtag, 0, 0); /* IDLE */
		}
	} else {
		/* hw mode */
		aspeed_jtag_write(aspeed_jtag, 0, AST_JTAG_SW);
		while (remain_xfer) {
			if (sdr->direct) {
				dev_dbg(aspeed_jtag->dev,
					"W dr->dr_data[%d]: %x\n",
					index, kdata[index]);
				aspeed_jtag_write(aspeed_jtag,
						  kdata[index],
						  AST_JTAG_DATA);
			} else {
				aspeed_jtag_write(aspeed_jtag, 0,
						  AST_JTAG_DATA);
			}

			if (remain_xfer > 32) {
				shift_bits = 32;
				/* read bytes were not equals to column length
				 * => Pause-DR
				 */

				aspeed_jtag_push_data(aspeed_jtag,
						      shift_bits);
			} else {
				/* read bytes equals to column length =>
				 * Update-DR
				 */
				shift_bits = remain_xfer;
				aspeed_jtag_push_data_last(aspeed_jtag,
							   shift_bits,
							   sdr->enddr);
			}

			if (!sdr->direct) {
				if (shift_bits < 32)
					kdata[index] =
						aspeed_jtag_read(aspeed_jtag,
							AST_JTAG_DATA) >>
							(32 - shift_bits);
				else
					kdata[index] =
						aspeed_jtag_read(aspeed_jtag,
							AST_JTAG_DATA);
				dev_dbg(aspeed_jtag->dev,
					"R dr->dr_data[%d]: %x\n",
					index, kdata[index]);
			}

			remain_xfer = remain_xfer - shift_bits;
			index++;
			dev_dbg(aspeed_jtag->dev, "remain_xfer %d\n",
				remain_xfer);
		}
	}

	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN |
			  JTAG_SW_MODE_TDIO, AST_JTAG_SW);

	ret = copy_to_user_bit_data(sdr->tdio, kdata, sdr->length);
	aspeed_jtag->status = sdr->enddr;
	kfree(kdata);

	return ret;
}

static irqreturn_t aspeed_jtag_interrupt(int this_irq, void *dev_id)
{
	u32 status;
	struct aspeed_jtag_info *aspeed_jtag = dev_id;

	status = aspeed_jtag_read(aspeed_jtag, AST_JTAG_ISR);
	dev_dbg(aspeed_jtag->dev, "status %x\n", status);

	if (status & JTAG_INST_PAUSE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_INST_PAUSE |
				(status & 0xf), AST_JTAG_ISR);
		aspeed_jtag->flag = JTAG_INST_PAUSE;
	}

	if (status & JTAG_INST_COMPLETE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_INST_COMPLETE |
				  (status & 0xf), AST_JTAG_ISR);
		aspeed_jtag->flag = JTAG_INST_COMPLETE;
	}

	if (status & JTAG_DATA_PAUSE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_DATA_PAUSE |
				  (status & 0xf), AST_JTAG_ISR);
		aspeed_jtag->flag = JTAG_DATA_PAUSE;
	}

	if (status & JTAG_DATA_COMPLETE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_DATA_COMPLETE |
				  (status & 0xf), AST_JTAG_ISR);
		aspeed_jtag->flag = JTAG_DATA_COMPLETE;
	}

	if (aspeed_jtag->flag) {
		wake_up_interruptible(&aspeed_jtag->jtag_wq);
		return IRQ_HANDLED;
	} else {
		dev_err(aspeed_jtag->dev, "aspeed_jtag irq status (%x)\n",
			status);
		return IRQ_NONE;
	}
}

static long aspeed_jtag_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	struct aspeed_jtag_info *aspeed_jtag = file->private_data;
	void __user *argp = (void __user *)arg;
	struct sir_xfer sir;
	struct sdr_xfer sdr;
	struct runtest_idle run_idle;
	int ret = 0;
	unsigned int frq = 0;

	switch (cmd) {
	case AST_JTAG_GIOCFREQ:
		ret = __put_user(aspeed_jtag_get_freq(aspeed_jtag),
				 (unsigned int __user *)arg);
		break;

	case AST_JTAG_SIOCFREQ:
		ret = __get_user(frq, (unsigned int __user *)argp);

		if (frq == 0) {
			ret = -EFAULT;
			break;
		}

		if (frq > aspeed_get_pclk(aspeed_jtag->pclk))
			ret = -EFAULT;
		else
			aspeed_jtag_set_freq(aspeed_jtag, frq);
		break;

	case AST_JTAG_IOCRUNTEST:
		if (copy_from_user(&run_idle, argp,
				   sizeof(struct runtest_idle)))
			ret = -EFAULT;
		else
			aspeed_jtag_run_test_idle(aspeed_jtag, &run_idle);

		break;

	case AST_JTAG_IOCSIR:
		if (copy_from_user(&sir, argp, sizeof(struct sir_xfer)))
			ret = -EFAULT;
		else
			aspeed_jtag_sir_xfer(aspeed_jtag, &sir);

		if (copy_to_user(argp, &sir, sizeof(struct sir_xfer)))
			ret = -EFAULT;

		break;

	case AST_JTAG_IOCSDR:
		if (copy_from_user(&sdr, argp, sizeof(struct sdr_xfer)))
			ret = -EFAULT;
		else
			aspeed_jtag_sdr_xfer(aspeed_jtag, &sdr);

		if (copy_to_user(argp, &sdr, sizeof(struct sdr_xfer)))
			ret = -EFAULT;

		break;
	default:

		return -ENOTTY;
	}

	return ret;
}

static int aspeed_jtag_open(struct inode *inode, struct file *file)
{
	spin_lock(&jtag_state_lock);
	if (aspeed_jtag->is_open) {
		spin_unlock(&jtag_state_lock);
		return -EBUSY;
	}

	aspeed_jtag->is_open = true;
	file->private_data = aspeed_jtag;
	spin_unlock(&jtag_state_lock);

	return 0;
}

static int aspeed_jtag_release(struct inode *inode, struct file *file)
{
	struct aspeed_jtag_info *drvdata = file->private_data;

	spin_lock(&jtag_state_lock);
	drvdata->is_open = false;
	spin_unlock(&jtag_state_lock);

	return 0;
}

static ssize_t show_status(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", aspeed_jtag->status ? "Pause" : "Idle");
}

static DEVICE_ATTR(status, S_IRUGO, show_status, NULL);

static ssize_t show_frequency(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);

	return sprintf(buf, "Frequency (%d)\n",
		       aspeed_get_pclk(aspeed_jtag->pclk) /
		       (JTAG_GET_TCK_DIVISOR(aspeed_jtag_read(aspeed_jtag,
					     AST_JTAG_TCK)) + 1));
}

static ssize_t store_frequency(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	unsigned long input_val;
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);
	int ret;

	ret = kstrtoul(buf, 10, &input_val);
	if (ret)
		return ret;

	aspeed_jtag_set_freq(aspeed_jtag, input_val);

	return count;
}

static DEVICE_ATTR(freq, S_IRUGO | S_IWUSR, show_frequency, store_frequency);

static struct attribute *jtag_sysfs_entries[] = {
	&dev_attr_freq.attr,
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group jtag_attribute_group = {
	.attrs = jtag_sysfs_entries,
};

static const struct file_operations aspeed_jtag_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= aspeed_jtag_ioctl,
	.open		= aspeed_jtag_open,
	.release	= aspeed_jtag_release,
};

struct miscdevice aspeed_jtag_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "aspeed-jtag",
	.fops	= &aspeed_jtag_fops,
};

static void aspeed_jtag_remove_sysfs_group(void *_dev)
{
	struct device *dev = _dev;

	sysfs_remove_group(&dev->kobj, &jtag_attribute_group);
}

static int aspeed_jtag_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;

	if (!of_device_is_compatible(pdev->dev.of_node,
				     "aspeed,aspeed2500-jtag"))
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	aspeed_jtag = devm_kzalloc(&pdev->dev, sizeof(*aspeed_jtag),
				   GFP_KERNEL);
	if (!aspeed_jtag)
		return -ENOMEM;

	aspeed_jtag->pclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_jtag->pclk)) {
		dev_err(&pdev->dev, "devm_clk_get failed\n");
		return PTR_ERR(aspeed_jtag->pclk);
	}

	aspeed_jtag->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!aspeed_jtag->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	aspeed_jtag->irq = platform_get_irq(pdev, 0);
	if (aspeed_jtag->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	/* Init JTAG */
	aspeed_toggle_scu_reset(SCU_RESET_JTAG, 3);

	/* Eanble Clock */
	aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN,
			  AST_JTAG_CTRL);
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO,
			  AST_JTAG_SW);

	ret = devm_request_irq(&pdev->dev, aspeed_jtag->irq,
			       aspeed_jtag_interrupt, 0,
			       "aspeed-jtag", aspeed_jtag);
	if (ret) {
		dev_info(&pdev->dev, "aspeed_jtag unable to get IRQ");
		goto out_region;
	}

	aspeed_jtag_write(aspeed_jtag, JTAG_INST_PAUSE | JTAG_INST_COMPLETE |
			  JTAG_DATA_PAUSE | JTAG_DATA_COMPLETE |
			  JTAG_INST_PAUSE_EN | JTAG_INST_COMPLETE_EN |
			  JTAG_DATA_PAUSE_EN | JTAG_DATA_COMPLETE_EN,
			  AST_JTAG_ISR);

	aspeed_jtag->flag = 0;
	init_waitqueue_head(&aspeed_jtag->jtag_wq);

	ret = misc_register(&aspeed_jtag_misc);
	if (ret) {
		dev_err(&pdev->dev, "aspeed_jtag : failed to register misc\n");
		goto out_region;
	}

	aspeed_jtag->dev = &pdev->dev;
	platform_set_drvdata(pdev, aspeed_jtag);

	ret = sysfs_create_group(&pdev->dev.kobj, &jtag_attribute_group);
	if (ret) {
		dev_err(&pdev->dev,
			"aspeed_jtag : failed to create sysfs attributes.\n");
		return -1;
	}
	ret = devm_add_action(&pdev->dev, aspeed_jtag_remove_sysfs_group,
			      &pdev->dev);
	if (ret) {
		aspeed_jtag_remove_sysfs_group(&pdev->dev);
		dev_err(&pdev->dev,
			"Failed to add sysfs cleanup action (%d)\n",
			ret);
		return ret;
	}

	dev_info(&pdev->dev,
		 "aspeed_jtag : driver successfully loaded.\n");

	return 0;

out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	dev_warn(&pdev->dev,
		 "aspeed_jtag : driver init failed ret (%d)\n", ret);
	return ret;
}

static int aspeed_jtag_remove(struct platform_device *pdev)
{
	misc_deregister(&aspeed_jtag_misc);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int
aspeed_jtag_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
aspeed_jtag_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define aspeed_jtag_suspend        NULL
#define aspeed_jtag_resume         NULL
#endif

static const struct of_device_id aspeed_jtag_of_table[] = {
	{ .compatible = "aspeed,aspeed2500-jtag", },
	{ },
};
MODULE_DEVICE_TABLE(of, aspeed_jtag_of_table);

static struct platform_driver aspeed_jtag_driver = {
	.probe		= aspeed_jtag_probe,
	.remove		= aspeed_jtag_remove,
	.suspend	= aspeed_jtag_suspend,
	.resume		= aspeed_jtag_resume,
	.driver         = {
		.name	= KBUILD_MODNAME,
		.owner	= THIS_MODULE,
		.of_match_table = aspeed_jtag_of_table,
	},
};

module_platform_driver(aspeed_jtag_driver);

MODULE_AUTHOR("Oleksandr Shamray <oleksandrs@mellanox.com>");
MODULE_DESCRIPTION("Aspeed JTAG platform driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("aspeed-jtag");

