/*
 * driver/misc/bt-device.c
 *
 * ASpeed BT device interface driver.
 *
 * Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * History:
 * 2013.05.15: Initial version [Ryan Chen]
 * 2016.11.15: Porting to kernel 4.7
 */

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/hwmon-sysfs.h>
#include <linux/ipmi.h>
#include "aspeed-regs-lpc.h"

/* Results of SMI events. */
enum si_sm_result {
	SI_SM_CALL_WITHOUT_DELAY, /* Call the driver again immediately */
	SI_SM_CALL_WITH_DELAY,	/* Delay some before calling again. */
	SI_SM_CALL_WITH_TICK_DELAY,/* Delay >=1 tick before calling again. */
	SI_SM_TRANSACTION_COMPLETE, /* A transaction is finished. */
	SI_SM_IDLE,		/* The SM is in idle state. */
	SI_SM_HOSED,		/* The hardware violated the state machine. */
	/*
	 * The hardware is asserting attn and the state machine is
	 * idle.
	 */
	SI_SM_ATTN,
	SI_SM_READ_COMPLETE
};

/*
 * Typical "Get BT Capabilities" values are 2-3 retries, 5-10 seconds,
 * and 64 byte buffers.  However, one HP implementation wants 255 bytes of
 * buffer (with a documented message of 160 bytes) so go for the max.
 * Since the Open IPMI architecture is single-message oriented at this
 * stage, the queue depth of BT is of no concern.
 */

#define BT_NORMAL_TIMEOUT	5	/* seconds */
#define BT_NORMAL_RETRY_LIMIT	2
#define BT_RESET_DELAY		6	/* seconds after warm reset */

enum bt_states {
	BT_STATE_IDLE = 0,	/* Order is critical in this list */
	BT_STATE_XACTION_START,
	BT_STATE_WRITE_BYTES,
	BT_STATE_WRITE_CONSUME,
	BT_STATE_READ_WAIT,
	BT_STATE_CLEAR_B2H,
	BT_STATE_CLEAR_H2B,
	BT_STATE_READ_BYTES,
	BT_STATE_RESET1,	/* These must come last */
	BT_STATE_RESET2,
	BT_STATE_RESET3,
	BT_STATE_RESTART,
	BT_STATE_PRINTME,
	BT_STATE_CAPABILITIES_BEGIN,
	BT_STATE_CAPABILITIES_END,
	BT_STATE_LONG_BUSY	/* BT doesn't get hosed :-) */
};

struct aspeed_lpc_bus_info {
	u8 lpc_bus_mode; /* 1: host mode , 0: dev mode*/
	u8 scan_node;
	u8 lpc_mode; /* 0: lpc , 1: lpc+ */
	u8 ipmi_bt_enable;
	u32 bridge_phy_addr;
};

struct aspeed_bt_data {
	u32			str;
	u32			fifo;
	enum bt_states	state;
	unsigned char	seq;		/* BT sequence number */
	unsigned char	write_data[IPMI_MAX_MSG_LENGTH];
	int			write_count;
	unsigned char	read_data[IPMI_MAX_MSG_LENGTH];
	int			read_count;
	int			truncated;
	long			timeout;	/* microseconds countdown */
	int			error_retries;	/* end of "common" fields */
	int			nonzero_status;	/* hung BMCs stay all 0 */
	enum bt_states	complete;	/* to divert the state machine */
	int			BT_CAP_outreqs;
	long			BT_CAP_req2rsp;
	int			BT_CAP_retries;	/* Recommended retries */
	unsigned char	completion_code[IPMI_MAX_MSG_LENGTH];
	unsigned int	completion_code_length;
};

struct aspeed_lpc_driver_data {
	struct platform_device		*pdev;
	void __iomem			*reg_base;
	int					irq;
	u32					bus_id;
	struct aspeed_lpc_bus_info	*bus_info;
	struct aspeed_bt_data		bt_data;
	struct aspeed_bt_data		ibt_data;
};

static inline u32 aspeed_lpc_read(struct aspeed_lpc_driver_data *aspeed_lpc,
	u32 reg)
{
	return readl(aspeed_lpc->reg_base + reg);
}

static inline void aspeed_lpc_write(struct aspeed_lpc_driver_data *aspeed_lpc,
	u32 val, u32 reg)
{
	writel(val, aspeed_lpc->reg_base + reg);
}

static void write_all_bytes(struct aspeed_lpc_driver_data *aspeed_lpc,
	struct aspeed_bt_data *bt)
{
	int i;

	for (i = 0; i < bt->write_count; i++)
		dev_info(&aspeed_lpc->pdev->dev, " %02x", bt->write_data[i]);
	dev_info(&aspeed_lpc->pdev->dev, "\n");

	for (i = 0; i < bt->write_count; i++)
		aspeed_lpc_write(aspeed_lpc, bt->write_data[i], bt->fifo);
}

static int read_all_bytes(struct aspeed_lpc_driver_data *aspeed_lpc,
	struct aspeed_bt_data *bt)
{
	unsigned char i;
	int max;

	/*
	 * length is "framing info", minimum = 3: NetFn, Seq, Cmd
	 * Keep layout of first four bytes aligned with write_data[]
	 */

	bt->read_data[0] = aspeed_lpc_read(aspeed_lpc, bt->fifo);
	bt->read_count = bt->read_data[0];
	dev_info(&aspeed_lpc->pdev->dev, "in read_all_bytes bt->read_count = %x\n",
		bt->read_count);

	if (bt->read_count < 3 || bt->read_count >= IPMI_MAX_MSG_LENGTH) {
		dev_warn(&aspeed_lpc->pdev->dev, "BT: bad raw rsp len=%d\n",
			bt->read_count);
		bt->truncated = 1;
		return 1;	/* let next XACTION START clean it up */
	}

	for (i = 1; i <= bt->read_count; i++)
		bt->read_data[i] = aspeed_lpc_read(aspeed_lpc, bt->fifo);
	bt->read_count++;	/* Account internally for length byte */

	dev_info(&aspeed_lpc->pdev->dev, "bt->read_data = ");
	for (i = 0; i < bt->read_count; i++)
		dev_info(&aspeed_lpc->pdev->dev, "%x ", bt->read_data[i]);

	max = bt->read_count;

	dev_warn(&aspeed_lpc->pdev->dev, "BT: got %d bytes seq=0x%02X",
		max, bt->read_data[2]);
	if (max > 16)
		max = 16;

	for (i = 0; i < max; i++)
		dev_info(&aspeed_lpc->pdev->dev, " %02x", bt->read_data[i]);
	dev_info(&aspeed_lpc->pdev->dev, "%s\n",
		bt->read_count == max ? "" : " ...");

	return 0;
}

/* Check status and (usually) take action and change this state machine. */
static enum si_sm_result ibt_event(struct aspeed_lpc_driver_data *aspeed_lpc,
	struct aspeed_bt_data *bt)
{
	unsigned char status;
	static enum bt_states laspeed_printed = BT_STATE_PRINTME;
	int i;

	status = aspeed_lpc_read(aspeed_lpc, bt->str);
	bt->nonzero_status |= status;

	if ((bt->state < BT_STATE_WRITE_BYTES) && (status & BT_B2H_ATN))
		dev_warn(&aspeed_lpc->pdev->dev, "Buffer was not empty!\n");

	switch (bt->state) {

	/*
	 * Idle state first checks for asynchronous messages from another
	 * channel, then does some opportunistic housekeeping.
	 */

	case BT_STATE_IDLE:
		if (status & BT_B_BUSY)		/* clear a leftover B_BUSY */
			aspeed_lpc_write(aspeed_lpc, BT_B_BUSY, bt->str);

		bt->state = BT_STATE_READ_WAIT;
		return SI_SM_CALL_WITHOUT_DELAY;

	case BT_STATE_XACTION_START:
		if (status & (BT_H_BUSY | BT_B2H_ATN)) {
			laspeed_printed = BT_STATE_PRINTME;
			return SI_SM_CALL_WITH_DELAY;
		}
		if (aspeed_lpc_read(aspeed_lpc, bt->str) & BT_B_BUSY)
			aspeed_lpc_write(aspeed_lpc, BT_B_BUSY, bt->str);

		bt->state = BT_STATE_WRITE_BYTES;
		return SI_SM_CALL_WITHOUT_DELAY;

	case BT_STATE_WRITE_BYTES:
		if (status & BT_B_BUSY)
			aspeed_lpc_write(aspeed_lpc, BT_B_BUSY, bt->str);
		write_all_bytes(aspeed_lpc, bt);
		aspeed_lpc_write(aspeed_lpc, BT_B2H_ATN, bt->str);

		bt->state = bt->complete;
		return bt->state == BT_STATE_IDLE ?	/* where to next? */
			SI_SM_TRANSACTION_COMPLETE :	/* normal */
			SI_SM_CALL_WITHOUT_DELAY;	/* Startup magic */

	case BT_STATE_READ_WAIT:
		if (!(status & BT_H2B_ATN)) {
			laspeed_printed = BT_STATE_PRINTME;
			return SI_SM_CALL_WITH_DELAY;
		}
		aspeed_lpc_write(aspeed_lpc, BT_B_BUSY, bt->str);
		/* clear it to ACK the host */
		aspeed_lpc_write(aspeed_lpc, BT_B2H_ATN, bt->str);

		bt->state = BT_STATE_CLEAR_H2B;
		return SI_SM_CALL_WITHOUT_DELAY;

	case BT_STATE_CLEAR_H2B:
		if (status & BT_H2B_ATN) {
			/* keep hitting it */
			aspeed_lpc_write(aspeed_lpc, BT_B2H_ATN, bt->str);
			laspeed_printed = BT_STATE_PRINTME;
			return SI_SM_CALL_WITH_DELAY;
		}

		bt->state = BT_STATE_READ_BYTES;
		return SI_SM_CALL_WITHOUT_DELAY;

	case BT_STATE_READ_BYTES:
		/* check in case of retry */
		if (!(status & BT_B_BUSY))
			aspeed_lpc_write(aspeed_lpc, BT_B_BUSY, bt->str);
		i = read_all_bytes(aspeed_lpc, bt);
		aspeed_lpc_write(aspeed_lpc, BT_B_BUSY, bt->str);

		bt->state = BT_STATE_IDLE;
		return SI_SM_READ_COMPLETE;

	default:
		return SI_SM_CALL_WITH_DELAY;
	}
	return SI_SM_CALL_WITH_DELAY;
}

static void aspeed_lpc_ipmi_bt_handle(struct aspeed_lpc_driver_data
	*aspeed_lpc, struct aspeed_bt_data *bt_data)
{
	u8 result;
	unsigned int count = 0;

	do {
		result = ibt_event(aspeed_lpc, bt_data);
		count++;
	} while (((result != SI_SM_READ_COMPLETE) && (count <= 100000)));

	if (count >= 100000)
		dev_info(&aspeed_lpc->pdev->dev, "SI_SM_READ_NOT_COMPLETE\n");

	count = 0;
	dev_info(&aspeed_lpc->pdev->dev, "driver finished read\n");
}

static inline void aspeed_lpc_enable(struct aspeed_lpc_driver_data
	*aspeed_lpc, int enable)
{
	if (enable) {
		aspeed_lpc_write(aspeed_lpc, aspeed_lpc_read(aspeed_lpc,
			AST_LPC_HICR0) | LPC_LPC3_EN, AST_LPC_HICR0);
	} else {
		aspeed_lpc_write(aspeed_lpc, aspeed_lpc_read(aspeed_lpc,
			AST_LPC_HICR0) & ~LPC_LPC3_EN, AST_LPC_HICR0);
	}
}

static int aspeed_lpc_get_ipmi_bt_enable(struct aspeed_lpc_driver_data
	 *aspeed_lpc)
{
	return (aspeed_lpc_read(aspeed_lpc, AST_LPC_HICR0) & LPC_LPC3_EN)
		&& (aspeed_lpc_read(aspeed_lpc, AST_LPC_HICR4) &
		LPC_HICS_BTENBL);
}

static void aspeed_lpc_set_ipmi_bt_enable(struct aspeed_lpc_driver_data
	*aspeed_lpc, int enable)
{
	if (enable) {
		aspeed_lpc_enable(aspeed_lpc, enable);
		aspeed_lpc_write(aspeed_lpc, aspeed_lpc_read(aspeed_lpc,
			AST_LPC_HICR4) | LPC_HICS_BTENBL, AST_LPC_HICR4);
	} else {
		aspeed_lpc_write(aspeed_lpc, aspeed_lpc_read(aspeed_lpc,
			AST_LPC_HICR4) & ~LPC_HICS_BTENBL, AST_LPC_HICR4);
	}
}

static u16 aspeed_lpc_get_ipmi_bt_addr(struct aspeed_lpc_driver_data
	*aspeed_lpc)
{
	return ((aspeed_lpc_read(aspeed_lpc, AST_LPC_LADR3H) << 8) |
		aspeed_lpc_read(aspeed_lpc, AST_LPC_LADR3L)) + 2;
}

static void aspeed_lpc_set_ipmi_bt_addr(struct aspeed_lpc_driver_data
	*aspeed_lpc, u16 bt_addr)
{
	aspeed_lpc_write(aspeed_lpc, (bt_addr - 2) >> 8, AST_LPC_LADR3H);
	aspeed_lpc_write(aspeed_lpc, (bt_addr - 2) & 0xff, AST_LPC_LADR3L);
}

static int aspeed_lpc_get_ipmi_ibt_enable(struct aspeed_lpc_driver_data
	*aspeed_lpc)
{
	return aspeed_lpc_read(aspeed_lpc, AST_LPC_IBTCR0) & LPC_iBT_ENABLE;
}

static void aspeed_lpc_set_ipmi_ibt_enable(struct aspeed_lpc_driver_data
	*aspeed_lpc, int enable)
{
	if (enable) {
		/* Enable BT H2B interrupt Interrupt */
		aspeed_lpc_write(aspeed_lpc, aspeed_lpc_read(aspeed_lpc,
			AST_LPC_IBTCR1) | LPC_iBT_H2B_RISING_ISR,
			AST_LPC_IBTCR1);
		/* Enable BT interface */
		aspeed_lpc_write(aspeed_lpc, aspeed_lpc_read(aspeed_lpc,
			AST_LPC_IBTCR0) | LPC_iBT_ENABLE | LPC_iBT_ClrSvRdP_EN |
			LPC_iBT_ClrSvWrP_EN, AST_LPC_IBTCR0);
	} else {
		/* Enable BT H2B interrupt */
		aspeed_lpc_write(aspeed_lpc, aspeed_lpc_read(aspeed_lpc,
			AST_LPC_IBTCR1) & ~LPC_iBT_H2B_RISING_ISR,
			AST_LPC_IBTCR1);
		/* Enable BT interface */
		aspeed_lpc_write(aspeed_lpc, aspeed_lpc_read(aspeed_lpc,
			AST_LPC_IBTCR0) & ~(LPC_iBT_ENABLE |
			LPC_iBT_ClrSvRdP_EN | LPC_iBT_ClrSvWrP_EN),
			AST_LPC_IBTCR0);
	}
}

static u16 aspeed_lpc_get_ipmi_ibt_addr(struct aspeed_lpc_driver_data
	*aspeed_lpc)
{
	return LPC_iBT_GET_ADDR(aspeed_lpc_read(aspeed_lpc, AST_LPC_IBTCR0));
}

static void aspeed_lpc_set_ipmi_ibt_addr(struct aspeed_lpc_driver_data
	*aspeed_lpc, u16 bt_addr)
{
	aspeed_lpc_write(aspeed_lpc, (aspeed_lpc_read(aspeed_lpc,
		AST_LPC_IBTCR0) & ~LPC_iBT_ADDR_MASK) |
		LPC_iBT_SET_ADDR(bt_addr), AST_LPC_IBTCR0);
}

static ssize_t aspeed_lpc_show_ipmi_bt_enable(struct device *dev,
	struct device_attribute *attr, char *sysfsbuf)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_get_drvdata(dev);

	return sprintf(sysfsbuf, "%s\n",
		aspeed_lpc_get_ipmi_bt_enable(aspeed_lpc) ?
		"enabled" : "disabled");
}

static ssize_t aspeed_lpc_store_ipmi_bt_enable(struct device *dev,
	struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_get_drvdata(dev);
	int val;

	kstrtoint(sysfsbuf, 10, &val);
	aspeed_lpc_set_ipmi_bt_enable(aspeed_lpc, val);

	return count;
}

static ssize_t aspeed_lpc_show_ipmi_bt_addr(struct device *dev,
	struct device_attribute *attr, char *sysfsbuf)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_get_drvdata(dev);

	return sprintf(sysfsbuf, "0x%x\n",
		aspeed_lpc_get_ipmi_bt_addr(aspeed_lpc));
}

static ssize_t aspeed_lpc_store_ipmi_bt_addr(struct device *dev,
	struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_get_drvdata(dev);
	u16 addr;

	if (kstrtouint(sysfsbuf, 16, &addr) < 0)
		return -EINVAL;

	aspeed_lpc_set_ipmi_bt_addr(aspeed_lpc, addr);

	dev_info(&aspeed_lpc->pdev->dev, "IPMI BT address was set to 0x%x.\n",
		addr);

	return count;
}

static ssize_t aspeed_lpc_show_ipmi_ibt_enable(struct device *dev,
	struct device_attribute *attr, char *sysfsbuf)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_get_drvdata(dev);

	return sprintf(sysfsbuf, "%s\n",
		aspeed_lpc_get_ipmi_ibt_enable(aspeed_lpc) ?
		"enabled" : "disabled");
}

static ssize_t aspeed_lpc_store_ipmi_ibt_enable(struct device *dev,
	struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_get_drvdata(dev);

	int val;

	kstrtoint(sysfsbuf, 10, &val);
	aspeed_lpc_set_ipmi_ibt_enable(aspeed_lpc, val);

	return count;
}

static ssize_t aspeed_lpc_show_ipmi_ibt_addr(struct device *dev,
	struct device_attribute *attr, char *sysfsbuf)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_get_drvdata(dev);

	return sprintf(sysfsbuf, "0x%x\n",
		aspeed_lpc_get_ipmi_ibt_addr(aspeed_lpc));
}

static ssize_t aspeed_lpc_store_ipmi_ibt_addr(struct device *dev,
	struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_get_drvdata(dev);
	u16 addr;

	kstrtouint(sysfsbuf, 10, &addr);
	aspeed_lpc_set_ipmi_ibt_addr(aspeed_lpc, addr);

	dev_info(&aspeed_lpc->pdev->dev, "IPMI iBT address was set to 0x%x.\n",
		addr);

	return count;
}

static struct device_attribute attr_ipmi_bt_en =
	__ATTR(
		ipmi_bt_status, S_IRUGO | S_IWUSR,
		aspeed_lpc_show_ipmi_bt_enable,
		aspeed_lpc_store_ipmi_bt_enable
	);

static struct device_attribute attr_ipmi_bt_addr =
	__ATTR(
		ipmi_bt_addr, S_IRUGO | S_IWUSR,
		aspeed_lpc_show_ipmi_bt_addr,
		aspeed_lpc_store_ipmi_bt_addr
	);

static struct device_attribute attr_ipmi_ibt_en =
	__ATTR(
		ipmi_ibt_status, S_IRUGO | S_IWUSR,
		aspeed_lpc_show_ipmi_ibt_enable,
		aspeed_lpc_store_ipmi_ibt_enable
	);

static struct device_attribute attr_ipmi_ibt_addr =
	__ATTR(
		ipmi_ibt_addr, S_IRUGO | S_IWUSR,
		aspeed_lpc_show_ipmi_ibt_addr,
		aspeed_lpc_store_ipmi_ibt_addr
	);

static struct attribute *ipmi_bt_attributes[] = {
	&attr_ipmi_bt_en.attr,
	&attr_ipmi_bt_addr.attr,
	NULL
};

static struct attribute *ipmi_ibt_attributes[] = {
	&attr_ipmi_ibt_en.attr,
	&attr_ipmi_ibt_addr.attr,
	NULL
};

static const struct attribute_group ipmi_bt_attribute_groups[] = {
	{ .attrs = ipmi_bt_attributes },
	{ .attrs = ipmi_ibt_attributes },
};

static irqreturn_t aspeed_lpc_isr(int this_irq, void *dev_id)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_id;

	/* BT */
	if (aspeed_lpc_read(aspeed_lpc, AST_LPC_HICR4) & LPC_HICS_BTENBL)
		aspeed_lpc_ipmi_bt_handle(aspeed_lpc, &aspeed_lpc->bt_data);

	/* iBT */
	if (aspeed_lpc_read(aspeed_lpc, AST_LPC_IBTCR0) & LPC_iBT_ENABLE) {
		if (aspeed_lpc_read(aspeed_lpc, AST_LPC_IBTCR2) &
			 LPC_iBT_H2B_RISING_ISR) {
			aspeed_lpc_write(aspeed_lpc, LPC_iBT_H2B_RISING_ISR,
				AST_LPC_IBTCR2);
			aspeed_lpc_ipmi_bt_handle(aspeed_lpc,
				&aspeed_lpc->ibt_data);
		}
	}

	return IRQ_HANDLED;
}

static ssize_t show_lpc2ahb_en(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_get_drvdata(dev);

	if (aspeed_lpc_read(aspeed_lpc, AST_LPC_HICR5) & LPC_HICR5_ENL2H)
		return sprintf(buf, "enabled\n");
	else
		return sprintf(buf, "disabled\n");
}

static ssize_t store_lpc2ahb_en(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_get_drvdata(dev);

	kstrtouint(buf, 10, &val);

	if (val)
		aspeed_lpc_write(aspeed_lpc, aspeed_lpc_read(aspeed_lpc,
			AST_LPC_HICR5) | LPC_HICR5_ENL2H, AST_LPC_HICR5);
	else
		aspeed_lpc_write(aspeed_lpc, aspeed_lpc_read(aspeed_lpc,
			AST_LPC_HICR5) & ~LPC_HICR5_ENL2H, AST_LPC_HICR5);

	aspeed_lpc_write(aspeed_lpc, 0xffff0000, AST_LPC_HICR8);

	return count;
}

static DEVICE_ATTR(
	lpc2ahb_en, S_IRUGO | S_IWUSR,
	show_lpc2ahb_en, store_lpc2ahb_en);

static ssize_t show_lpc_mmio_space(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_get_drvdata(dev);
	int reg = 0;

	u32 *lbuf = (u32 *) buf;

	while (reg < 0x260) {
		*lbuf++ = readl(aspeed_lpc->reg_base + reg);
		reg += 4;
	}

	return reg;
}

static ssize_t store_lpc_mmio_space(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aspeed_lpc_driver_data *aspeed_lpc = dev_get_drvdata(dev);
	unsigned long reg, val;

	if (sscanf(buf, "l%lxw%lx", &reg, &val) >= 2)
		writel(val, aspeed_lpc->reg_base + reg);
	else if (sscanf(buf, "b%lxw%lx", &reg, &val) >= 2)
		writeb(val, aspeed_lpc->reg_base + reg);
	else
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(
	mmio, S_IRUGO | S_IWUSR,
	show_lpc_mmio_space, store_lpc_mmio_space);

static struct attribute *aspeed_lpc_attributes[] = {
	&dev_attr_lpc2ahb_en.attr,
	&dev_attr_mmio.attr,
	NULL
};

static const struct attribute_group lpc_attribute_group = {
	.attrs = aspeed_lpc_attributes,
};

static int aspeed_lpc_probe(struct platform_device *pdev)
{
	static struct aspeed_lpc_driver_data *aspeed_lpc;
	const struct device_node *bt_if_node, *ibt_if_node;
	const __be32 *lpc_addr;
	struct resource *res;
	int ret = 0;
	int i = 0;

	if (!of_device_is_compatible(pdev->dev.of_node, "aspeed,bt-device"))
		return -ENODEV;

	aspeed_lpc = kzalloc(sizeof(struct aspeed_lpc_driver_data),
		GFP_KERNEL);
	if (aspeed_lpc == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	aspeed_lpc->pdev = pdev;
	aspeed_lpc->bus_info = pdev->dev.platform_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free;
	}

	aspeed_lpc->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (aspeed_lpc->reg_base == NULL) {
		dev_err(&pdev->dev, "failed to map registers\n");
		ret = -EIO;
		goto err_free_mem;
	}

	aspeed_lpc->irq = platform_get_irq(pdev, 0);
	if (aspeed_lpc->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto err_free_mem;
	}

	ret = devm_request_irq(&pdev->dev, aspeed_lpc->irq, aspeed_lpc_isr,
		0, "bt-device", aspeed_lpc);

	if (ret) {
		dev_err(&pdev->dev, "unable to get IRQ");
		goto err_free_mem;
	}

	platform_set_drvdata(pdev, aspeed_lpc);
	dev_set_drvdata(&pdev->dev, aspeed_lpc);

	ret = sysfs_create_group(&pdev->dev.kobj, &lpc_attribute_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs attributes.\n");
		goto err_free_mem;
	}

	aspeed_lpc->bt_data.str = AST_LPC_BTDTR;
	aspeed_lpc->bt_data.fifo = AST_LPC_BTR1;
	aspeed_lpc->bt_data.seq = 0;
	aspeed_lpc->bt_data.state = BT_STATE_IDLE;	/* start here */
	aspeed_lpc->bt_data.complete = BT_STATE_IDLE;	/* end here */
	aspeed_lpc->bt_data.BT_CAP_req2rsp = BT_NORMAL_TIMEOUT * 1000000;
	aspeed_lpc->bt_data.BT_CAP_retries = BT_NORMAL_RETRY_LIMIT;

	aspeed_lpc->ibt_data.str = AST_LPC_IBTCR4;
	aspeed_lpc->ibt_data.fifo = AST_LPC_IBTCR5;
	aspeed_lpc->ibt_data.seq = 0;
	aspeed_lpc->ibt_data.state = BT_STATE_IDLE;	/* start here */
	aspeed_lpc->ibt_data.complete = BT_STATE_IDLE;	/* end here */
	aspeed_lpc->ibt_data.BT_CAP_req2rsp = BT_NORMAL_TIMEOUT * 1000000;
	aspeed_lpc->ibt_data.BT_CAP_retries = BT_NORMAL_RETRY_LIMIT;

	for (i = 0; i < 2; i++) {
		ret = sysfs_create_group(&pdev->dev.kobj,
			&ipmi_bt_attribute_groups[i]);
		if (ret)
			goto err_free_mem;
	}

	if (of_property_match_string(pdev->dev.of_node, "status",
		"disabled") >= 0) {
		aspeed_lpc_enable(aspeed_lpc, 0);
	} else {
		lpc_addr = of_get_property(pdev->dev.of_node, "addr", NULL);
		if (lpc_addr) {
			const u16 addr = be32_to_cpup(lpc_addr);

			aspeed_lpc_set_ipmi_bt_addr(aspeed_lpc, addr);
			dev_info(&pdev->dev,
				"lpc i/o address was successfully configured to 0x%x.\n",
				addr);
		}
		aspeed_lpc_write(aspeed_lpc, aspeed_lpc_read(aspeed_lpc,
			AST_LPC_HICR2) | (1 << 3), AST_LPC_HICR2);
		aspeed_lpc_enable(aspeed_lpc, 1);
	}

	bt_if_node = of_get_child_by_name(pdev->dev.of_node, "bt-if");

	if (bt_if_node && of_property_match_string(bt_if_node,
			"status", "disabled") >= 0) {
		aspeed_lpc_set_ipmi_bt_enable(aspeed_lpc, 0);
	} else {
		aspeed_lpc_set_ipmi_bt_enable(aspeed_lpc, 1);
		dev_info(&pdev->dev,
			"IPMI BT interface was successfully enabled.\n");
	}

	ibt_if_node = of_get_child_by_name(pdev->dev.of_node, "ibt-if");

	if (ibt_if_node && of_property_match_string(ibt_if_node, "status",
			"disabled") >= 0) {
		aspeed_lpc_set_ipmi_ibt_enable(aspeed_lpc, 0);
	} else {
		const u32 *addrp = of_get_property(ibt_if_node, "addr", NULL);

		if (addrp) {
			const u16 addr = be32_to_cpup(addrp);

			aspeed_lpc_set_ipmi_ibt_addr(aspeed_lpc, addr);
			aspeed_lpc_set_ipmi_ibt_enable(aspeed_lpc, 1);
			dev_info(&pdev->dev,
				"IPMI iBT interface was successfully enabled (@ 0x%x).\n",
				addr);
		}
	}

	dev_info(&pdev->dev, "bt-device: driver successfully loaded.\n");

	return 0;

err_free_mem:
	release_mem_region(res->start, resource_size(res));
err_free:
	kfree(aspeed_lpc);

	return ret;
}

static const struct of_device_id aspeed_lpc_of_table[] = {
	{ .compatible = "aspeed,bt-device", },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_lpc_of_table);

static struct platform_driver aspeed_lpc_driver = {
	.driver		= {
		.name		= "bt-device",
		.owner	= THIS_MODULE,
		.of_match_table = aspeed_lpc_of_table,
	},
	.probe		= aspeed_lpc_probe,
};

static int __init aspeed_lpc_init(void)
{
	return platform_driver_register(&aspeed_lpc_driver);
}

arch_initcall(aspeed_lpc_init);

MODULE_AUTHOR("Yaniv Abraham-Rabinovitch <yanivab@mellanox.com>");
MODULE_DESCRIPTION("ASpeed BT Device Interface Driver");
MODULE_LICENSE("GPL");
