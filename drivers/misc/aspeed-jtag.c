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
#include <linux/aspeed-jtag.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
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

#define ASPEED_JTAG_DATA		0x00
#define ASPEED_JTAG_INST		0x04
#define ASPEED_JTAG_CTRL		0x08
#define ASPEED_JTAG_ISR			0x0C
#define ASPEED_JTAG_SW			0x10
#define ASPEED_JTAG_TCK			0x14
#define ASPEED_JTAG_IDLE		0x18

/* ASPEED_JTAG_CTRL - 0x08 : Engine Control */
#define ASPEED_JTAG_ENG_EN		BIT(31)
#define ASPEED_JTAG_ENG_OUT_EN		BIT(30)
#define ASPEED_JTAG_FORCE_TMS		BIT(29)
#define ASPEED_JTAG_INST_LEN(x)		(x << 20)
#define ASPEED_JTAG_LASPEED_INST	BIT(17)
#define ASPEED_JTAG_INST_EN		BIT(16)
#define ASPEED_JTAG_DR_UPDATE		BIT(10)
#define ASPEED_JTAG_DATA_LEN(x)		(x << 4)
#define ASPEED_JTAG_LASPEED_DATA	BIT(1)
#define ASPEED_JTAG_DATA_EN		BIT(0)

/* ASPEED_JTAG_ISR	- 0x0C : Interrupt status and enable */
#define ASPEED_JTAG_INST_PAUSE		BIT(19)
#define ASPEED_JTAG_INST_COMPLETE	BIT(18)
#define ASPEED_JTAG_DATA_PAUSE		BIT(17)
#define ASPEED_JTAG_DATA_COMPLETE	BIT(16)
#define ASPEED_JTAG_INST_PAUSE_EN	BIT(3)
#define ASPEED_JTAG_INST_COMPLETE_EN	BIT(2)
#define ASPEED_JTAG_DATA_PAUSE_EN	BIT(1)
#define ASPEED_JTAG_DATA_COMPLETE_EN	BIT(0)
#define ASPEED_JTAG_INT_EN_MASK		GENMASK(3, 0)
#define ASPEED_JTAG_INT_MASK		GENMASK(19, 16)

/* ASPEED_JTAG_SW	- 0x10 : Software Mode and Status */
#define ASPEED_JTAG_SW_MODE_EN		BIT(19)
#define ASPEED_JTAG_SW_MODE_TCK		BIT(18)
#define ASPEED_JTAG_SW_MODE_TMS		BIT(17)
#define ASPEED_JTAG_SW_MODE_TDIO	BIT(16)

/* ASPEED_JTAG_TCK	- 0x14 : TCK Control */
#define ASPEED_JTAG_TCK_DIVISOR_MASK	GENMASK(10, 0)
#define ASPEED_JTAG_GET_TCK_DIV(x)	(x & ASPEED_JTAG_TCK_DIVISOR_MASK)

/* ASPEED_JTAG_IDLE - 0x18 : Controller set for go to IDLE */
#define ASPEED_JTAG_GO_IDLE		BIT(0)

#define ASPEED_JTAG_IOUT_LEN(len)	(ASPEED_JTAG_ENG_EN |\
					 ASPEED_JTAG_ENG_OUT_EN |\
					 ASPEED_JTAG_INST_LEN(len))

#define ASPEED_JTAG_DOUT_LEN(len)	(ASPEED_JTAG_ENG_EN |\
					 ASPEED_JTAG_ENG_OUT_EN |\
					 ASPEED_JTAG_DATA_LEN(len))

struct aspeed_jtag_info {
	void __iomem			*reg_base;
	struct device			*dev;
	struct clk			*pclk;
	enum aspeed_jtag_endstate	status;
	int				irq;
	u32				flag;
	wait_queue_head_t		jtag_wq;
	bool				is_open;
};

static struct aspeed_jtag_info *aspeed_jtag;
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

static u32 aspeed_get_pclk(struct clk *pclk)
{
	unsigned long timer_freq;

	timer_freq = clk_get_rate(pclk);

	return timer_freq;
}

static void
aspeed_jtag_set_freq(struct aspeed_jtag_info *aspeed_jtag, unsigned int freq)
{
	u16 i = 0;
	u32 apb_frq = 0;

	dev_dbg(aspeed_jtag->dev, "set frq : %d\n", freq);

	apb_frq = aspeed_get_pclk(aspeed_jtag->pclk);

	i = (apb_frq % freq == 0) ? (apb_frq / freq) - 1 : (apb_frq / freq);

	aspeed_jtag_write(aspeed_jtag,
			  ((aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_TCK) &
					     ASPEED_JTAG_TCK_DIVISOR_MASK) | i),
					     ASPEED_JTAG_TCK);
}

static unsigned int aspeed_jtag_get_freq(struct aspeed_jtag_info *aspeed_jtag)
{
	u32 pclk;
	u32 tck;

	pclk = aspeed_get_pclk(aspeed_jtag->pclk);
	tck = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_TCK);

	return pclk / (ASPEED_JTAG_GET_TCK_DIV(tck) + 1);
}

static void aspeed_jtag_sw_dummy(struct aspeed_jtag_info *aspeed_jtag,
				 unsigned int cnt)
{
	int i;

	for (i = 0; i < cnt; i++)
		aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW);
}

static u8 aspeed_jtag_tck_cycle(struct aspeed_jtag_info *aspeed_jtag,
				u8 tms, u8 tdi)
{
	u8 tdo = 0;

	/* TCK = 0 */
	aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_SW_MODE_EN |
			  (tms * ASPEED_JTAG_SW_MODE_TMS) |
			  (tdi * ASPEED_JTAG_SW_MODE_TDIO),
			  ASPEED_JTAG_SW);

	aspeed_jtag_sw_dummy(aspeed_jtag, 10);

	/* TCK = 1 */
	aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_SW_MODE_EN |
			  ASPEED_JTAG_SW_MODE_TCK |
			  (tms * ASPEED_JTAG_SW_MODE_TMS) |
			  (tdi * ASPEED_JTAG_SW_MODE_TDIO),
			  ASPEED_JTAG_SW);

	if (aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW) &
	    ASPEED_JTAG_SW_MODE_TDIO)
		tdo = 1;

	aspeed_jtag_sw_dummy(aspeed_jtag, 10);

	/* TCK = 0 */
	aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_SW_MODE_EN |
			  (tms * ASPEED_JTAG_SW_MODE_TMS) |
			  (tdi * ASPEED_JTAG_SW_MODE_TDIO),
			  ASPEED_JTAG_SW);

	return tdo;
}

static void
aspeed_jtag_wait_instruction_pause(
		struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag &
				 ASPEED_JTAG_INST_PAUSE));

	aspeed_jtag->flag &= ~ASPEED_JTAG_INST_PAUSE;
}

static void
aspeed_jtag_wait_instruction_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag &
				 ASPEED_JTAG_INST_COMPLETE));

	aspeed_jtag->flag &= ~ASPEED_JTAG_INST_COMPLETE;
}

static void
aspeed_jtag_wait_data_pause_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag &
				 ASPEED_JTAG_DATA_PAUSE));

	aspeed_jtag->flag &= ~ASPEED_JTAG_DATA_PAUSE;
}

static void
aspeed_jtag_wait_data_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag &
				 ASPEED_JTAG_DATA_COMPLETE));

	aspeed_jtag->flag &= ~ASPEED_JTAG_DATA_COMPLETE;
}

static void
aspeed_jtag_sm_cycle(struct aspeed_jtag_info *aspeed_jtag, u8 *tms, u8 len)
{
	u8 i;

	for (i = 0; i < len; i++)
		aspeed_jtag_tck_cycle(aspeed_jtag, tms[i], 0);
}

static void
aspeed_jtag_run_test_idle_sw(struct aspeed_jtag_info *aspeed_jtag,
			     struct aspeed_jtag_runtest_idle *runtest)
{
	u8 sm_pause_irpause[] = {1, 1, 1, 1, 0, 1, 0};
	u8 sm_pause_drpause[] = {1, 1, 1, 0, 1, 0};
	u8 sm_idle_irpause[] = {1, 1, 0, 1, 0};
	u8 sm_idle_drpause[] = {1, 0, 1, 0};
	u8 sm_pause_idle[] = {1, 1, 0};
	int i = 0;

	/* SW mode from idle/pause-> to pause/idle */
	if (runtest->reset) {
		for (i = 0; i < 10; i++)
			aspeed_jtag_tck_cycle(aspeed_jtag, 1, 0);
	}

	switch (aspeed_jtag->status) {
	case ASPEED_JTAG_STATE_IDLE:
		switch (runtest->end) {
		case ASPEED_JTAG_STATE_IRPAUSE:
			/* ->DRSCan->IRSCan->IRCap->IRExit1->IRPause */
			aspeed_jtag_sm_cycle(aspeed_jtag,
					     sm_idle_irpause,
					     sizeof(sm_idle_irpause));

			aspeed_jtag->status = ASPEED_JTAG_STATE_IRPAUSE;
			break;

		case ASPEED_JTAG_STATE_DRPAUSE:
			/* ->DRSCan->DRCap->DRExit1->DRPause */
			aspeed_jtag_sm_cycle(aspeed_jtag,
					     sm_idle_drpause,
					     sizeof(sm_idle_drpause));

			aspeed_jtag->status = ASPEED_JTAG_STATE_DRPAUSE;
			break;

		case ASPEED_JTAG_STATE_IDLE:
			/* IDLE */
			aspeed_jtag_tck_cycle(aspeed_jtag, 0, 0);
			aspeed_jtag->status = ASPEED_JTAG_STATE_IDLE;
			break;

		default:
			break;
		}
		break;

	case ASPEED_JTAG_STATE_IRPAUSE:
	case ASPEED_JTAG_STATE_DRPAUSE:
		/* from IR/DR Pause */
		switch (runtest->end) {
		case ASPEED_JTAG_STATE_IRPAUSE:
			/*
			 * to Exit2 IR/DR->Updt IR/DR->DRSCan->IRSCan->IRCap->
			 * IRExit1->IRPause
			 */
			aspeed_jtag_sm_cycle(aspeed_jtag,
					     sm_pause_irpause,
					     sizeof(sm_pause_irpause));

			aspeed_jtag->status = ASPEED_JTAG_STATE_IRPAUSE;
			break;

		case ASPEED_JTAG_STATE_DRPAUSE:
			/* to Exit2 IR/DR->Updt IR/DR->DRSCan->DRCap->
			 * DRExit1->DRPause
			 */
			aspeed_jtag_sm_cycle(aspeed_jtag,
					     sm_pause_drpause,
					     sizeof(sm_pause_drpause));
			aspeed_jtag->status = ASPEED_JTAG_STATE_DRPAUSE;
			break;
		case ASPEED_JTAG_STATE_IDLE:
			/* -to Exit2 IR/DR->Updt IR/DR->IDLE */
			aspeed_jtag_sm_cycle(aspeed_jtag,
					     sm_pause_idle,
					     sizeof(sm_pause_idle));
			aspeed_jtag->status = ASPEED_JTAG_STATE_IDLE;
			break;

		default:
			break;
		}
		break;

	default:
		dev_err(aspeed_jtag->dev, "aspeed_jtag_run_test_idle error\n");
		break;
	}

	/* stay on IDLE for at least  TCK cycle */
	for (i = 0; i < runtest->tck; i++)
		aspeed_jtag_tck_cycle(aspeed_jtag, 0, 0);
}

/**
 * aspeed_jtag_run_test_idle:
 * JTAG reset: generates at least 9 TMS high and 1 TMS low to force
 * devices into Run-Test/Idle State.
 */
static void aspeed_jtag_run_test_idle(struct aspeed_jtag_info *aspeed_jtag,
				      struct aspeed_jtag_runtest_idle *runtest)
{
	dev_dbg(aspeed_jtag->dev, "aspeed_jtag runtest, status:%d, mode:%s, end:%s, reset:%d, tck:%d\n",
		aspeed_jtag->status, runtest->mode ? "SW" : "HW",
		end_status_str[runtest->end], runtest->reset, runtest->tck);

	if (runtest->mode) {
		aspeed_jtag_run_test_idle_sw(aspeed_jtag, runtest);
		return;
	}

	/* disable sw mode */
	aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW);
	mdelay(1);
	/* x TMS high + 1 TMS low */
	if (runtest->reset)
		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_ENG_EN |
				ASPEED_JTAG_ENG_OUT_EN |
				ASPEED_JTAG_FORCE_TMS,
				ASPEED_JTAG_CTRL);
	else
		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_GO_IDLE,
				  ASPEED_JTAG_IDLE);
	mdelay(1);
	aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_SW_MODE_EN |
			  ASPEED_JTAG_SW_MODE_TDIO, ASPEED_JTAG_SW);
	aspeed_jtag->status = ASPEED_JTAG_STATE_IDLE;
}

static int aspeed_jtag_sir_xfer(struct aspeed_jtag_info *aspeed_jtag,
				struct aspeed_jtag_sir_xfer *sir)
{
	u8 sm_update_siftir[] = {1, 1, 0, 0};
	u8 sm_pause_idle[] = {1, 1, 0};
	u8 sm_pause_update[] = {1, 1};
	int i = 0;

	dev_dbg(aspeed_jtag->dev, "aspeed_jtag SIR xfer, mode:%s, TDI:%x, ENDIR : %d, len:%d\n",
		sir->mode ? "SW" : "HW", sir->tdi, sir->endir, sir->length);

	if (sir->mode) {
		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_SW_MODE_EN |
				  ASPEED_JTAG_SW_MODE_TDIO, ASPEED_JTAG_SW);
		if (aspeed_jtag->status != ASPEED_JTAG_STATE_IDLE) {
			/* from IR/DR Pause->Exit2 IR / DR->Update IR /DR */
			aspeed_jtag_sm_cycle(aspeed_jtag,
					     sm_pause_update,
					     sizeof(sm_pause_update));
		}

		/* to DRSCan->IRSCan->CapIR->ShiftIR */
		aspeed_jtag_sm_cycle(aspeed_jtag,
				     sm_update_siftir,
				     sizeof(sm_update_siftir));

		sir->tdo = 0;

		for (i = 0; i < sir->length - 1; i++) {
			/* ShiftIR */
			sir->tdo |= aspeed_jtag_tck_cycle(aspeed_jtag, 0,
							  sir->tdi & 0x1);
			sir->tdi >>= 1;
			sir->tdo <<= 1;
		}

		/* IRExit1 */
		sir->tdo |= aspeed_jtag_tck_cycle(aspeed_jtag, 1,
						  sir->tdi & 0x1);

		/* IRPause */
		aspeed_jtag_tck_cycle(aspeed_jtag, 0, 0);

		/* stop pause */
		if (sir->endir == ASPEED_JTAG_STATE_IDLE) {
			/* to IRExit2->IRUpdate->IDLE */
			aspeed_jtag_sm_cycle(aspeed_jtag,
					     sm_pause_idle,
					     sizeof(sm_pause_idle));
		}
	} else {
		/* hw mode - disable sw mode */
		aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW);
		aspeed_jtag_write(aspeed_jtag, sir->tdi, ASPEED_JTAG_INST);

		if (sir->endir != ASPEED_JTAG_STATE_IDLE) {
			aspeed_jtag_write(aspeed_jtag,
					  ASPEED_JTAG_IOUT_LEN(sir->length),
					  ASPEED_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag,
					  ASPEED_JTAG_IOUT_LEN(sir->length) |
					  ASPEED_JTAG_INST_EN,
					  ASPEED_JTAG_CTRL);
			aspeed_jtag_wait_instruction_pause(aspeed_jtag);
		} else {
			aspeed_jtag_write(aspeed_jtag,
					  ASPEED_JTAG_IOUT_LEN(sir->length) |
					  ASPEED_JTAG_LASPEED_INST,
					  ASPEED_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag,
					  ASPEED_JTAG_IOUT_LEN(sir->length) |
					  ASPEED_JTAG_LASPEED_INST |
					  ASPEED_JTAG_INST_EN,
					  ASPEED_JTAG_CTRL);
			aspeed_jtag_wait_instruction_complete(aspeed_jtag);
		}

		sir->tdo = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_INST);
		if (sir->endir == ASPEED_JTAG_STATE_IDLE) {
			aspeed_jtag_write(aspeed_jtag,
					  ASPEED_JTAG_SW_MODE_EN |
					  ASPEED_JTAG_SW_MODE_TDIO,
					  ASPEED_JTAG_SW);
		}
	}

	aspeed_jtag->status = sir->endir;

	return 0;
}

static unsigned int
*copy_from_user_bit_data(void __user *udata, unsigned int bit_size)
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

static int
copy_to_user_bit_data(void __user *udata, void *kdata, unsigned int bit_size)
{
	int size = 0;
	int ret = 0;

	size  = (bit_size + 7) / 8;
	ret = copy_to_user(udata, kdata, size);

	return ret;
}

static void
aspeed_jtag_push_data(struct aspeed_jtag_info *aspeed_jtag, u32 shift_bits)
{
	dev_dbg(aspeed_jtag->dev, "shift bits %d\n", shift_bits);

	aspeed_jtag_write(aspeed_jtag,
			  ASPEED_JTAG_DOUT_LEN(shift_bits),
			  ASPEED_JTAG_CTRL);
	aspeed_jtag_write(aspeed_jtag,
			  ASPEED_JTAG_DOUT_LEN(shift_bits) |
			  ASPEED_JTAG_DATA_EN,
			  ASPEED_JTAG_CTRL);
	aspeed_jtag_wait_data_pause_complete(aspeed_jtag);
}

static void
aspeed_jtag_push_data_last(struct aspeed_jtag_info *aspeed_jtag,
			   u32 shift_bits, enum aspeed_jtag_endstate enddr)
{
	if (enddr != ASPEED_JTAG_STATE_IDLE) {
		dev_dbg(aspeed_jtag->dev, "DR Keep Pause\n");

		aspeed_jtag_write(aspeed_jtag,
				  ASPEED_JTAG_DOUT_LEN(shift_bits) |
				  ASPEED_JTAG_DR_UPDATE,
				  ASPEED_JTAG_CTRL);
		aspeed_jtag_write(aspeed_jtag,
				  ASPEED_JTAG_DOUT_LEN(shift_bits) |
				  ASPEED_JTAG_DR_UPDATE |
				  ASPEED_JTAG_DATA_EN,
				  ASPEED_JTAG_CTRL);
		aspeed_jtag_wait_data_pause_complete(aspeed_jtag);
	} else {
		dev_dbg(aspeed_jtag->dev, "DR go IDLE\n");

		aspeed_jtag_write(aspeed_jtag,
				  ASPEED_JTAG_DOUT_LEN(shift_bits) |
				  ASPEED_JTAG_LASPEED_DATA,
				  ASPEED_JTAG_CTRL);
		aspeed_jtag_write(aspeed_jtag,
				  ASPEED_JTAG_DOUT_LEN(shift_bits) |
				  ASPEED_JTAG_LASPEED_DATA |
				  ASPEED_JTAG_DATA_EN,
				  ASPEED_JTAG_CTRL);
		aspeed_jtag_wait_data_complete(aspeed_jtag);

		aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_SW_MODE_EN |
				  ASPEED_JTAG_SW_MODE_TDIO,
				  ASPEED_JTAG_SW);
	}
}

static void
aspeed_jtag_sdr_xfer_sw(struct aspeed_jtag_info *aspeed_jtag,
			struct aspeed_jtag_sdr_xfer *sdr, u32 *kdata)
{
	u32 remain_xfer = sdr->length;
	u32 shift_bits = 0;
	u32 index = 0;
	u32 tdo = 0;
	u8  tms = 0;

	while (remain_xfer) {
		if (sdr->direct) {
			/* write */
			if ((shift_bits % 32) == 0)
				dev_dbg(aspeed_jtag->dev, "W dr->dr_data[%d]: %x\n",
					index, kdata[index]);

			tdo = (kdata[index] >> (shift_bits % 32)) & 0x1;
			dev_dbg(aspeed_jtag->dev, "%d\n", tdo);

			/* change state to DRExit1 if last bit is shift*/
			tms = (remain_xfer == 1) ? 1 : 0;
			aspeed_jtag_tck_cycle(aspeed_jtag, tms, tdo);
		} else {
			/* read */

			/* change state to DRExit1 if last bit is shift*/
			tms = (remain_xfer == 1) ? 1 : 0;
			tdo = aspeed_jtag_tck_cycle(aspeed_jtag, tms, tdo);

			dev_dbg(aspeed_jtag->dev, "%d\n", tdo);

			kdata[index] |= (tdo << (shift_bits % 32));

			if ((shift_bits % 32) == 0)
				dev_dbg(aspeed_jtag->dev, "R dr->dr_data[%d]: %x\n",
					index, kdata[index]);
		}
		shift_bits++;
		remain_xfer--;
		if ((shift_bits % 32) == 0)
			index++;
	}
}

static void
aspeed_jtag_sdr_xfer_hw(struct aspeed_jtag_info *aspeed_jtag,
			struct aspeed_jtag_sdr_xfer *sdr, u32 *kdata)
{
	u32 remain_xfer = sdr->length;
	u32 shift_bits = 0;
	u32 index = 0;

	while (remain_xfer) {
		if (sdr->direct) {
			dev_dbg(aspeed_jtag->dev, "W dr->dr_data[%d]: %x\n",
				index, kdata[index]);

			aspeed_jtag_write(aspeed_jtag, kdata[index],
					  ASPEED_JTAG_DATA);
		} else {
			aspeed_jtag_write(aspeed_jtag, 0,
					  ASPEED_JTAG_DATA);
		}

		if (remain_xfer > 32) {
			shift_bits = 32;

			/*
			 * read bytes were not equals to column length
			 * and go to Pause-DR
			 */
			aspeed_jtag_push_data(aspeed_jtag,
					      shift_bits);
		} else {
			/*
			 * read bytes equals to column length =>
			 * Update-DR
			 */
			shift_bits = remain_xfer;
			aspeed_jtag_push_data_last(aspeed_jtag,
						   shift_bits,
						   sdr->enddr);
		}

		if (!sdr->direct) {
			if (shift_bits < 32)
				kdata[index] = aspeed_jtag_read(aspeed_jtag,
							ASPEED_JTAG_DATA) >>
							(32 - shift_bits);
			else
				kdata[index] =
						aspeed_jtag_read(aspeed_jtag,
							ASPEED_JTAG_DATA);
			dev_dbg(aspeed_jtag->dev, "R dr->dr_data[%d]: %x\n",
				index, kdata[index]);
		}

		remain_xfer = remain_xfer - shift_bits;
		index++;
		dev_dbg(aspeed_jtag->dev, "remain_xfer %d\n", remain_xfer);
	}
}

static int
aspeed_jtag_sdr_xfer(struct aspeed_jtag_info *aspeed_jtag,
		     struct aspeed_jtag_sdr_xfer *sdr)
{
	u8 sm_update_drshift[] = {1, 0, 0};
	u8 sm_pause_idle[] = {1, 1, 0};
	u8 sm_pause_update[] = {1, 1};
	u32 remain_xfer = sdr->length;
	char dbg_str[256];
	u32 index = 0;
	u32 pos = 0;
	int ret = 0;
	u32 *kdata;
	u32 i = 0;

	kdata = copy_from_user_bit_data((void __user *)sdr->tdio, sdr->length);
	if (!kdata)
		return -ENOMEM;

	for (index = 0, i = 0; index < sdr->length; index += 32, i++)
		pos += sprintf(&dbg_str[pos], "0x%08x ", kdata[i]);

	dev_dbg(aspeed_jtag->dev, "aspeed_jtag SDR xfer, mode:%s, ENDDR:%d, len:%d, TDI[ %s ]\n",
		sdr->mode ? "SW" : "HW", sdr->enddr, remain_xfer, dbg_str);

	if (sdr->mode) {
		/* SW mode */
		if (aspeed_jtag->status != ASPEED_JTAG_STATE_IDLE) {
			/*IR/DR Pause->Exit2 IR / DR->Update IR /DR */
			aspeed_jtag_sm_cycle(aspeed_jtag,
					     sm_pause_update,
					     sizeof(sm_pause_update));
		}

		/* ->DRScan->DRCap->DRShift */
		aspeed_jtag_sm_cycle(aspeed_jtag,
				     sm_update_drshift,
				     sizeof(sm_update_drshift));

		aspeed_jtag_sdr_xfer_sw(aspeed_jtag, sdr, kdata);

		aspeed_jtag_tck_cycle(aspeed_jtag, 0, 0); /* DRPause */

		if (sdr->enddr == ASPEED_JTAG_STATE_IDLE) {
			/* ->DRExit2->DRUpdate->IDLE */
			aspeed_jtag_sm_cycle(aspeed_jtag,
					     sm_pause_idle,
					     sizeof(sm_pause_idle));
		}
	} else {
		/* hw mode */
		aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW);
		aspeed_jtag_sdr_xfer_hw(aspeed_jtag, sdr, kdata);
	}

	aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_SW_MODE_EN |
			  ASPEED_JTAG_SW_MODE_TDIO, ASPEED_JTAG_SW);

	ret = copy_to_user_bit_data((void __user *)sdr->tdio,
				    kdata, sdr->length);
	aspeed_jtag->status = sdr->enddr;
	kfree(kdata);

	return ret;
}

static irqreturn_t aspeed_jtag_interrupt(int this_irq, void *dev_id)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_id;
	u32 status;
	irqreturn_t ret;

	status = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_ISR);
	dev_dbg(aspeed_jtag->dev, "status %x\n", status);

	if (status & ASPEED_JTAG_INT_MASK) {
		aspeed_jtag_write(aspeed_jtag, (status & ASPEED_JTAG_INT_MASK)
				  | (status & ASPEED_JTAG_INT_EN_MASK),
				  ASPEED_JTAG_ISR);
		aspeed_jtag->flag |= status & ASPEED_JTAG_INT_MASK;
	}

	if (aspeed_jtag->flag) {
		wake_up_interruptible(&aspeed_jtag->jtag_wq);
		ret = IRQ_HANDLED;
	} else {
		dev_err(aspeed_jtag->dev, "aspeed_jtag irq status (%x)\n",
			status);
		ret = IRQ_NONE;
	}
	return ret;
}

static long aspeed_jtag_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	struct aspeed_jtag_info *aspeed_jtag = file->private_data;
	void __user *argp = (void __user *)arg;
	struct aspeed_jtag_runtest_idle run_idle;
	struct aspeed_jtag_sir_xfer sir;
	struct aspeed_jtag_sdr_xfer sdr;
	unsigned int frq = 0;
	int ret = 0;

	switch (cmd) {
	case ASPEED_JTAG_GIOCFREQ:
		ret = __put_user(aspeed_jtag_get_freq(aspeed_jtag),
				 (unsigned int __user *)arg);
		break;

	case ASPEED_JTAG_SIOCFREQ:
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

	case ASPEED_JTAG_IOCRUNTEST:
		if (copy_from_user(&run_idle, argp,
				   sizeof(struct aspeed_jtag_runtest_idle)))
			ret = -EFAULT;
		else
			aspeed_jtag_run_test_idle(aspeed_jtag, &run_idle);

		break;

	case ASPEED_JTAG_IOCSIR:
		if (copy_from_user(&sir, argp,
				   sizeof(struct aspeed_jtag_sir_xfer)))
			ret = -EFAULT;
		else
			aspeed_jtag_sir_xfer(aspeed_jtag, &sir);

		if (copy_to_user(argp, &sir,
				 sizeof(struct aspeed_jtag_sir_xfer)))
			ret = -EFAULT;

		break;

	case ASPEED_JTAG_IOCSDR:
		if (copy_from_user(&sdr, argp,
				   sizeof(struct aspeed_jtag_sdr_xfer)))
			ret = -EFAULT;
		else
			aspeed_jtag_sdr_xfer(aspeed_jtag, &sdr);

		if (copy_to_user(argp, &sdr,
				 sizeof(struct aspeed_jtag_sdr_xfer)))
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

static ssize_t aspeed_jtag_show_status(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);
	ssize_t len = 0;

	len = sprintf(buf, "%s\n",
		      (aspeed_jtag->status == ASPEED_JTAG_STATE_IDLE) ?
		      "Pause" : "Idle");

	return len;
}

static DEVICE_ATTR(status, S_IRUGO, aspeed_jtag_show_status, NULL);

static ssize_t aspeed_jtag_show_frequency(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);
	int pclk;
	int div;
	int frq;

	pclk = aspeed_get_pclk(aspeed_jtag->pclk);
	div = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_TCK);
	frq = pclk / (ASPEED_JTAG_GET_TCK_DIV(div) + 1);

	return sprintf(buf, "Frequency (%d)\n", frq);
}

static ssize_t aspeed_jtag_store_frequency(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);
	unsigned long input_val;
	int ret;

	ret = kstrtoul(buf, 10, &input_val);
	if (ret)
		return ret;

	aspeed_jtag_set_freq(aspeed_jtag, input_val);

	return count;
}

static DEVICE_ATTR(freq, S_IRUGO | S_IWUSR, aspeed_jtag_show_frequency,
		   aspeed_jtag_store_frequency);

static struct attribute *aspeed_jtag_jtag_sysfs_entries[] = {
	&dev_attr_freq.attr,
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group aspeed_jtag_attribute_group = {
	.attrs = aspeed_jtag_jtag_sysfs_entries,
};

static const struct file_operations aspeed_jtag_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= aspeed_jtag_ioctl,
	.open		= aspeed_jtag_open,
	.release	= aspeed_jtag_release,
};

static struct miscdevice aspeed_jtag_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "aspeed-jtag",
	.fops	= &aspeed_jtag_fops,
};

static void aspeed_jtag_remove_sysfs_group(void *_dev)
{
	struct device *dev = _dev;

	sysfs_remove_group(&dev->kobj, &aspeed_jtag_attribute_group);
}

static int aspeed_jtag_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;

	if (!of_device_is_compatible(pdev->dev.of_node,
				     "aspeed,aspeed-jtag"))
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

	/* Eanble Clock */
	aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_ENG_EN |
			  ASPEED_JTAG_ENG_OUT_EN,
			  ASPEED_JTAG_CTRL);
	aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_SW_MODE_EN |
			  ASPEED_JTAG_SW_MODE_TDIO,
			  ASPEED_JTAG_SW);

	ret = devm_request_irq(&pdev->dev, aspeed_jtag->irq,
			       aspeed_jtag_interrupt, 0,
			       "aspeed-jtag", aspeed_jtag);
	if (ret) {
		dev_info(&pdev->dev, "aspeed_jtag unable to get IRQ");
		goto out_region;
	}

	aspeed_jtag_write(aspeed_jtag, ASPEED_JTAG_INST_PAUSE |
			  ASPEED_JTAG_INST_COMPLETE |
			  ASPEED_JTAG_DATA_PAUSE |
			  ASPEED_JTAG_DATA_COMPLETE |
			  ASPEED_JTAG_INST_PAUSE_EN |
			  ASPEED_JTAG_INST_COMPLETE_EN |
			  ASPEED_JTAG_DATA_PAUSE_EN |
			  ASPEED_JTAG_DATA_COMPLETE_EN,
			  ASPEED_JTAG_ISR);

	aspeed_jtag->flag = 0;
	init_waitqueue_head(&aspeed_jtag->jtag_wq);

	ret = misc_register(&aspeed_jtag_misc);
	if (ret) {
		dev_err(&pdev->dev, "aspeed_jtag : failed to register misc\n");
		goto out_region;
	}

	aspeed_jtag->dev = &pdev->dev;
	platform_set_drvdata(pdev, aspeed_jtag);

	ret = sysfs_create_group(&pdev->dev.kobj, &aspeed_jtag_attribute_group);
	if (ret) {
		dev_err(&pdev->dev,
			"aspeed_jtag : failed to create sysfs attributes.\n");
		return ret;
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
	{ .compatible = "aspeed,aspeed-jtag", },
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

