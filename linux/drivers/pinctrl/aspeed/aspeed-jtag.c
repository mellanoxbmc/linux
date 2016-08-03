/*
 *  driver/pincntrl/aspeed_jtag.c
 *
 *  ASPEED JTAG controller driver
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *   2012.08.06: Initial version [Ryan Chen]
 *   2016.0806: Porting to kernel 4.7
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/delay.h>

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

enum aspeed_jtag_xfer_mode {
	JTAG_XFER_HW_MODE = 0,
	JTAG_XFER_SW_MODE = 1,
};

struct runtest_idle {
	enum aspeed_jtag_xfer_mode	mode;	/* 0 :HW mode, 1: SW mode */
	unsigned char			reset;	/* Test Logic Reset */
	unsigned char			end;	/* o: idle, 1: ir pause, 2: drpause */
	unsigned char			tck;	/* keep tck */
};

struct sir_xfer {
	enum aspeed_jtag_xfer_mode	mode;	/* 0 :HW mode, 1: SW mode */
	unsigned short			length;	/* bits */
	unsigned int			tdi;
	unsigned int			tdo;
	unsigned char			endir;	/* 0: idle, 1:pause */
};

struct sdr_xfer {
	enum aspeed_jtag_xfer_mode	mode;	/* 0 :HW mode, 1: SW mode */
	unsigned char			direct; /* 0 ; read , 1 : write */
	unsigned short			length;	/* bits */
	unsigned int			*tdio;
	unsigned char			enddr;	/* 0: idle, 1:pause */
};

#define JTAGIOC_BASE       'T'

#define AST_JTAG_IOCRUNTEST	_IOW(JTAGIOC_BASE, 0, struct runtest_idle)
#define AST_JTAG_IOCSIR		_IOWR(JTAGIOC_BASE, 1, struct sir_xfer)
#define AST_JTAG_IOCSDR		_IOWR(JTAGIOC_BASE, 2, struct sdr_xfer)
#define AST_JTAG_SIOCFREQ	_IOW(JTAGIOC_BASE, 3, unsigned int)
#define AST_JTAG_GIOCFREQ	_IOR(JTAGIOC_BASE, 4, unsigned int)

struct aspeed_jtag_info {
	void __iomem		*reg_base;
	struct device		*dev;
	u8			sts; /* 0: idle, 1:irpause 2:drpause */
	int			irq; /* JTAG IRQ number */
	u32			flag;
	wait_queue_head_t	jtag_wq;
	bool 			is_open;
};

struct aspeed_jtag_info *aspeed_jtag;
static DEFINE_SPINLOCK(jtag_state_lock);

static inline u32
aspeed_jtag_read(struct aspeed_jtag_info *aspeed_jtag, u32 reg)
{
	return readl(aspeed_jtag->reg_base + reg);
}

static inline void
aspeed_jtag_write(struct aspeed_jtag_info *aspeed_jtag, u32 val, u32 reg)
{
	dev_err(aspeed_jtag->dev, "reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, aspeed_jtag->reg_base + reg);
}

u32 ast_get_pclk(void)
{
	return 0;
}

void
aspeed_jtag_set_freq(struct aspeed_jtag_info *aspeed_jtag, unsigned int freq)
{
	u16 i;

	for (i = 0; i < 0x7ff; i++) {
		if ((ast_get_pclk() / (i + 1)) <= freq)
			break;
	}
	aspeed_jtag_write(aspeed_jtag,
			  ((aspeed_jtag_read(aspeed_jtag, AST_JTAG_TCK) &
					      ~JTAG_TCK_DIVISOR_MASK) | i),
					      AST_JTAG_TCK);
}

unsigned int aspeed_jtag_get_freq(struct aspeed_jtag_info *aspeed_jtag)
{
	return ast_get_pclk() /
	       (JTAG_GET_TCK_DIVISOR(aspeed_jtag_read(aspeed_jtag,
						      AST_JTAG_TCK)) + 1);
}

void dummy(struct aspeed_jtag_info *aspeed_jtag, unsigned int cnt)
{
	int i = 0;

	for (i = 0; i < cnt; i++)
		aspeed_jtag_read(aspeed_jtag, AST_JTAG_SW);
}

static u8 TCK_Cycle(struct aspeed_jtag_info *aspeed_jtag, u8 TMS, u8 TDI)
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

void aspeed_jtag_wait_instruction_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag ==
				 JTAG_INST_COMPLETE));
	dev_err(aspeed_jtag->dev, "\n");
	aspeed_jtag->flag = 0;
}

void aspeed_jtag_wait_data_pause_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag ==
				 JTAG_DATA_PAUSE));
	dev_err(aspeed_jtag->dev, "\n");
	aspeed_jtag->flag = 0;
}

void aspeed_jtag_wait_data_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag ==
				 JTAG_DATA_COMPLETE));
	dev_err(aspeed_jtag->dev, "\n");
	aspeed_jtag->flag = 0;
}

/* JTAG_reset() is to generate at least 9 TMS high and 1 TMS low to force
 * devices into Run-Test/Idle State.
 */
void aspeed_jtag_run_test_idle(struct aspeed_jtag_info *aspeed_jtag,
			       struct runtest_idle *runtest)
{
	int i = 0;

	dev_dbg(aspeed_jtag->dev, ":%s mode\n", runtest->mode ? "SW":"HW");

	if (runtest->mode) {
		/* SW mode from idle , from pause,  -- > to pause, to idle */
		if (runtest->reset) {
			for (i = 0; i < 10; i++)
				TCK_Cycle(aspeed_jtag, 1, 0);
		}

		switch (aspeed_jtag->sts) {
		case 0:
			if (runtest->end == 1) {
				TCK_Cycle(aspeed_jtag, 1, 0); /* DRSCan */
				TCK_Cycle(aspeed_jtag, 1, 0); /* IRSCan */
				TCK_Cycle(aspeed_jtag, 0, 0); /* IRCap */
				TCK_Cycle(aspeed_jtag, 1, 0); /* IRExit1 */
				TCK_Cycle(aspeed_jtag, 0, 0); /* IRPause */
				aspeed_jtag->sts = 1;
			} else if (runtest->end == 2) {
				TCK_Cycle(aspeed_jtag, 1, 0); /* DRSCan */
				TCK_Cycle(aspeed_jtag, 0, 0); /* DRCap */
				TCK_Cycle(aspeed_jtag, 1, 0); /* DRExit1 */
				TCK_Cycle(aspeed_jtag, 0, 0); /* DRPause */
				aspeed_jtag->sts = 1;
			} else {
				TCK_Cycle(aspeed_jtag, 0, 0);/* IDLE */
				aspeed_jtag->sts = 0;
			}
			break;

		case 1:
			/* from IR/DR Pause */
			if (runtest->end == 1) {
				TCK_Cycle(aspeed_jtag, 1, 0); /* Exit2 IR/DR */
				TCK_Cycle(aspeed_jtag, 1, 0); /* Updt IR/DR */
				TCK_Cycle(aspeed_jtag, 1, 0); /* DRSCan */
				TCK_Cycle(aspeed_jtag, 1, 0); /* IRSCan */
				TCK_Cycle(aspeed_jtag, 0, 0); /* IRCap */
				TCK_Cycle(aspeed_jtag, 1, 0); /* IRExit1 */
				TCK_Cycle(aspeed_jtag, 0, 0); /* IRPause */
				aspeed_jtag->sts = 1;
			} else if (runtest->end == 2) {
				TCK_Cycle(aspeed_jtag, 1, 0); /* Exit2 IR/DR */
				TCK_Cycle(aspeed_jtag, 1, 0); /* Update IR/DR */
				TCK_Cycle(aspeed_jtag, 1, 0); /* DRSCan */
				TCK_Cycle(aspeed_jtag, 0, 0); /* DRCap */
				TCK_Cycle(aspeed_jtag, 1, 0); /* DRExit1 */
				TCK_Cycle(aspeed_jtag, 0, 0); /* DRPause */
				aspeed_jtag->sts = 1;
			} else {
				TCK_Cycle(aspeed_jtag, 1, 0); /* Exit2 IR/DR */
				TCK_Cycle(aspeed_jtag, 1, 0); /* Updt IR/DR */
				TCK_Cycle(aspeed_jtag, 0, 0); /* IDLE */
				aspeed_jtag->sts = 0;
			}
			break;

		default:
			dev_err(aspeed_jtag->dev,
				"aspeed_jtag_run_test_idle error\n");
			break;
		}

		/* stay on IDLE for at least  TCK cycle */
		for (i = 0; i < runtest->tck; i++)
			TCK_Cycle(aspeed_jtag, 0, 0);
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
		aspeed_jtag->sts = 0;
	}
}

int aspeed_jtag_sir_xfer(struct aspeed_jtag_info *aspeed_jtag,
			 struct sir_xfer *sir)
{
	int i = 0;

	dev_err(aspeed_jtag->dev, "%s mode, ENDIR : %d, len : %d\n",
		sir->mode ? "SW":"HW", sir->endir, sir->length);

	if (sir->mode) {
		if (aspeed_jtag->sts) {
			/* from IR/DR Pause */
			TCK_Cycle(aspeed_jtag, 1, 0); /* Exit2 IR / DR */
			TCK_Cycle(aspeed_jtag, 1, 0); /* Update IR /DR */
		}

		TCK_Cycle(aspeed_jtag, 1, 0); /* DRSCan */
		TCK_Cycle(aspeed_jtag, 1, 0); /* IRSCan */
		TCK_Cycle(aspeed_jtag, 0, 0); /* CapIR */
		TCK_Cycle(aspeed_jtag, 0, 0); /* ShiftIR */

		sir->tdo = 0;
		for (i = 0; i < sir->length; i++) {
			if (i == (sir->length - 1)) {
				/* IRExit1 */
				sir->tdo |= TCK_Cycle(aspeed_jtag, 1,
						      sir->tdi & 0x1);
			} else {
				/* ShiftIR */
				sir->tdo |= TCK_Cycle(aspeed_jtag, 0,
						      sir->tdi & 0x1);
				sir->tdi >>= 1;
				sir->tdo <<= 1;
			}
		}

		TCK_Cycle(aspeed_jtag, 0, 0); /* IRPause */

		/* stop pause */
		if (sir->endir == 0) {
			/* go to idle */
			TCK_Cycle(aspeed_jtag, 1, 0); /* IRExit2 */
			TCK_Cycle(aspeed_jtag, 1, 0); /* IRUpdate */
			TCK_Cycle(aspeed_jtag, 0, 0); /* IDLE */
		}
	} else {
		/* hw mode - disable sw mode */
		aspeed_jtag_write(aspeed_jtag, 0, AST_JTAG_SW);
		aspeed_jtag_write(aspeed_jtag, sir->tdi, AST_JTAG_INST);

		if (sir->endir) {
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN |
					  JTAG_ENG_OUT_EN |
					  JTAG_SET_INST_LEN(sir->length),
					  AST_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN |
					  JTAG_ENG_OUT_EN |
					  JTAG_SET_INST_LEN(sir->length) |
					  JTAG_INST_EN, AST_JTAG_CTRL);
		} else {
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN |
					  JTAG_ENG_OUT_EN | JTAG_LAST_INST |
					  JTAG_SET_INST_LEN(sir->length),
					  AST_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN |
					  JTAG_ENG_OUT_EN | JTAG_LAST_INST |
					  JTAG_SET_INST_LEN(sir->length) |
					  JTAG_INST_EN, AST_JTAG_CTRL);
		}

		aspeed_jtag_wait_instruction_complete(aspeed_jtag);
		sir->tdo = aspeed_jtag_read(aspeed_jtag, AST_JTAG_INST);
		aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN |
				  JTAG_SW_MODE_TDIO, AST_JTAG_SW);
	}

	aspeed_jtag->sts = sir->endir;

	return 0;
}

int aspeed_jtag_sdr_xfer(struct aspeed_jtag_info *aspeed_jtag,
			 struct sdr_xfer *sdr)
{
	unsigned int index = 0;
	u32 shift_bits = 0;
	u32 tdo = 0;
	u32 remain_xfer = sdr->length;

	dev_err(aspeed_jtag->dev, "%s mode, ENDDR : %d, len : %d\n",
		sdr->mode ? "SW":"HW", sdr->enddr, sdr->length);

	if (sdr->mode) {
		/* SW mode */
		if (aspeed_jtag->sts) {
			/* from IR/DR Pause */
			TCK_Cycle(aspeed_jtag, 1, 0); /* Exit2 IR / DR */
			TCK_Cycle(aspeed_jtag, 1, 0); /* Update IR /DR */
		}

		TCK_Cycle(aspeed_jtag, 1, 0); /* DRScan */
		TCK_Cycle(aspeed_jtag, 0, 0); /* DRCap */
		TCK_Cycle(aspeed_jtag, 0, 0); /* DRShift */

		while (remain_xfer) {
			if (sdr->direct) {
				/* write */
				if ((shift_bits % 32) == 0)
					dev_err(aspeed_jtag->dev, "W dr->dr_data[%d]: %x\n",
						index, sdr->tdio[index]);

				tdo = (sdr->tdio[index] >> (shift_bits % 32)) &
				      0x1;
				dev_err(aspeed_jtag->dev, "%d\n", tdo);
				if (remain_xfer == 1)
					/* DRExit1 */
					TCK_Cycle(aspeed_jtag, 1, tdo);
				else
					/* DRShit */
					TCK_Cycle(aspeed_jtag, 0, tdo);
			} else {
				/* read */
				if (remain_xfer == 1)
					/* DRExit1 */
					tdo = TCK_Cycle(aspeed_jtag, 1, tdo);
				else
					/* DRShit */
					tdo = TCK_Cycle(aspeed_jtag, 0, tdo);

				dev_err(aspeed_jtag->dev, "%d\n", tdo);
				sdr->tdio[index] |= (tdo << (shift_bits % 32));

				if ((shift_bits % 32) == 0)
					dev_err(aspeed_jtag->dev, "R dr->dr_data[%d]: %x\n",
						  index, sdr->tdio[index]);
			}
			shift_bits++;
			remain_xfer--;
			if ((shift_bits % 32) == 0)
				index++;
		}

		TCK_Cycle(aspeed_jtag, 0, 0); /* DRPause */

		if (sdr->enddr == 0) {
			TCK_Cycle(aspeed_jtag, 1, 0); /* DRExit2 */
			TCK_Cycle(aspeed_jtag, 1, 0); /* DRUpdate */
			TCK_Cycle(aspeed_jtag, 0, 0); /* IDLE */
		}
	} else {
		/* hw mode */
		aspeed_jtag_write(aspeed_jtag, 0, AST_JTAG_SW);
		while (remain_xfer) {
			if (sdr->direct) {
				dev_err(aspeed_jtag->dev,
					"W dr->dr_data[%d]: %x\n", index,
					  sdr->tdio[index]);
				aspeed_jtag_write(aspeed_jtag,
						  sdr->tdio[index],
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
				dev_err(aspeed_jtag->dev, "shit bits %d\n",
					shift_bits);
				aspeed_jtag_write(aspeed_jtag,
					JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_DATA_LEN(shift_bits),
					AST_JTAG_CTRL);
				aspeed_jtag_write(aspeed_jtag,
					JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_DATA_LEN(shift_bits) |
						      JTAG_DATA_EN,
						      AST_JTAG_CTRL);
				aspeed_jtag_wait_data_pause_complete(
								aspeed_jtag);
			} else {
				/* read bytes equals to column length =>
				 * Update-DR
				 */
				shift_bits = remain_xfer;
				dev_err(aspeed_jtag->dev,
					"shit bits %d with last\n",
					shift_bits);
				if (sdr->enddr) {
					dev_err(aspeed_jtag->dev, "DR Keep Pause\n");
						aspeed_jtag_write(aspeed_jtag,
						JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						JTAG_DR_UPDATE |
						JTAG_DATA_LEN(shift_bits),
						AST_JTAG_CTRL);
						aspeed_jtag_write(aspeed_jtag,
						JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						JTAG_DR_UPDATE |
						JTAG_DATA_LEN(shift_bits) |
						JTAG_DATA_EN, AST_JTAG_CTRL);
				} else {
					dev_err(aspeed_jtag->dev, "DR go IDLE\n");
						aspeed_jtag_write(aspeed_jtag,
						JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						JTAG_LAST_DATA |
						JTAG_DATA_LEN(shift_bits),
						AST_JTAG_CTRL);
						aspeed_jtag_write(aspeed_jtag,
						JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						JTAG_LAST_DATA |
						JTAG_DATA_LEN(shift_bits) |
						JTAG_DATA_EN, AST_JTAG_CTRL);
				}
				aspeed_jtag_wait_data_complete(aspeed_jtag);
			}

			if (!sdr->direct) {
				if (shift_bits < 32)
					sdr->tdio[index] =
						aspeed_jtag_read(aspeed_jtag,
							AST_JTAG_DATA) >>
							(32 - shift_bits);
				else
					sdr->tdio[index] =
						aspeed_jtag_read(aspeed_jtag,
							AST_JTAG_DATA);
				dev_err(aspeed_jtag->dev,
						"R dr->dr_data[%d]: %x\n",
						index, sdr->tdio[index]);
			}

			remain_xfer = remain_xfer - shift_bits;
			index++;
			dev_err(aspeed_jtag->dev, "remain_xfer %d\n",
				remain_xfer);
		}
		aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN |
				  JTAG_SW_MODE_TDIO, AST_JTAG_SW);
	}

	aspeed_jtag->sts = sdr->enddr;

	return 0;
}

static irqreturn_t aspeed_jtag_interrupt(int this_irq, void *dev_id)
{
	u32 status;
	struct aspeed_jtag_info *aspeed_jtag = dev_id;

	status = aspeed_jtag_read(aspeed_jtag, AST_JTAG_ISR);
	dev_err(aspeed_jtag->dev, "sts %x\n", status);

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
		dev_err("aspeed_jtag->dev, "aspeed_jtag irq status (%x)\n",
			status);
		return IRQ_NONE;
	}
}

static long jtag_ioctl(struct file *file, unsigned int cmd,
		       unsigned long arg)
{
	struct aspeed_jtag_info *aspeed_jtag = file->private_data;
	void __user *argp = (void __user *)arg;
	struct sir_xfer sir;
	struct sdr_xfer sdr;
	struct runtest_idle run_idle;
	int ret = 0;

	switch (cmd) {
	case AST_JTAG_GIOCFREQ:
		ret = __put_user(aspeed_jtag_get_freq(aspeed_jtag),
				 (unsigned int __user *)arg);
		break;

	case AST_JTAG_SIOCFREQ:
		if ((unsigned int )arg > ast_get_pclk())
			ret = -EFAULT;
		else
			aspeed_jtag_set_freq(aspeed_jtag, (unsigned int)arg);
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

		if (copy_to_user(argp, &sir, sizeof(struct sdr_xfer)))
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

static int jtag_open(struct inode *inode, struct file *file)
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

static int jtag_release(struct inode *inode, struct file *file)
{
	struct aspeed_jtag_info *drvdata = file->private_data;

	spin_lock(&jtag_state_lock);
	drvdata->is_open = false;
	spin_unlock(&jtag_state_lock);

	return 0;
}

static ssize_t show_sts(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", aspeed_jtag->sts ? "Pause" : "Idle");
}

static DEVICE_ATTR(sts, S_IRUGO, show_sts, NULL);

static ssize_t show_frequency(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);

	return sprintf(buf, "Frequency (%d)\n", ast_get_pclk() /
		       (JTAG_GET_TCK_DIVISOR(aspeed_jtag_read(aspeed_jtag,
		       AST_JTAG_TCK)) + 1));
}

static ssize_t store_frequency(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 input_val;
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
	&dev_attr_sts.attr,
	NULL
};

static struct attribute_group jtag_attribute_group = {
	.attrs = jtag_sysfs_entries,
};

static const struct file_operations aspeed_jtag_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= jtag_ioctl,
	.open		= jtag_open,
	.release	= jtag_release,
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

	/* ast_scu_init_jtag(); */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	if (!request_mem_region(res->start, resource_size(res), res->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out;
	}

	aspeed_jtag = devm_kzalloc(&pdev->dev, sizeof(*aspeed_jtag),
				   GFP_KERNEL);
	if (!aspeed_jtag)
		return -ENOMEM;

	aspeed_jtag->reg_base = ioremap(res->start, resource_size(res));
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
	aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN,
			  AST_JTAG_CTRL);
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO,
			  AST_JTAG_SW);

	ret = devm_request_irq(&pdev->dev, aspeed_jtag->irq,
			       aspeed_jtag_interrupt, 0,
			       "aspeed-jtag", aspeed_jtag);
	if (ret) {
		dev_info(&pdev->dev, "aspeed_jtag Unable to get IRQ");
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
	dev_set_drvdata(aspeed_jtag_misc.this_device, aspeed_jtag);

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

	dev_info(&pdev->dev, "aspeed_jtag : driver successfully loaded.\n");

	return 0;

out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	dev_warn(&pdev->dev, "aspeed_jtag : driver init failed ret (%d)\n", ret);
	return ret;
}

static int aspeed_jtag_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_jtag_info *aspeed_jtag = platform_get_drvdata(pdev);

	dev_err(aspeed_jtag->dev, "aspeed_jtag_remove\n");

	misc_deregister(&aspeed_jtag_misc);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(aspeed_jtag->reg_base);
	platform_set_drvdata(pdev, NULL);
	release_mem_region(res->start, res->end - res->start + 1);

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

static struct platform_driver aspeed_jtag_driver = {
	.remove		= aspeed_jtag_remove,
	.suspend	= aspeed_jtag_suspend,
	.resume		= aspeed_jtag_resume,
	.driver         = {
		.name	= "aspeed-jtag",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver_probe(aspeed_jtag_driver, aspeed_jtag_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST JTAG LIB Driver");
MODULE_LICENSE("GPL");
