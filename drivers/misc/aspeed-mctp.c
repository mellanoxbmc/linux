/*
 * Copyright (c) 2016 Mellanox Technologies. All rights reserved.
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

#include <linux/aspeed_mctp.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define ASPEED_MCTP_CTRL		0x00
#define ASPEED_MCTP_TX_CMD		0x04
#define ASPEED_MCTP_RX_CMD		0x08
#define ASPEED_MCTP_ISR			0x0c
#define ASPEED_MCTP_IER			0x10
#define ASPEED_MCTP_EID			0x14

/* ASPEED_MCTP_CTRL */
#define ASPEED_MCTP_RX_RDY		BIT(4)
#define ASPEED_MCTP_TX			BIT(0)

/* ASPEED_MCTP_ISR */
#define ASPEED_MCTP_RX_NO_CMD		BIT(9)
#define ASPEED_MCTP_RX_COMPLETE		BIT(8)
#define ASPEED_MCTP_TX_LAST		BIT(1)
#define ASPEED_MCTP_TX_COMPLETE		BIT(0)
#define ASPEED_MCTP_ISR_MASK		(ASPEED_MCTP_TX_LAST | \
					 ASPEED_MCTP_TX_COMPLETE | \
					 ASPEED_MCTP_RX_COMPLETE | \
					 ASPEED_MCTP_RX_NO_CMD)

/* TX CMD desc0 */
#define ASPEED_MCTP_PCI_BUS(x)		(((x) & 0xff) << 24)
#define ASPEED_MCTP_PCI_DEV(x)		(((x) & 0x1f) << 19)
#define ASPEED_MCTP_PCI_FUNC(x)		(((x) & 0x7) << 16)
#define ASPEED_MCTP_TX_INT_EN		BIT(15)
#define ASPEED_MCTP_TX_TAG_OWNER	BIT(13)
#define ASPEED_MCTP_ROUTING_TYPE_L(x)	(((x) & 0x1) << 14)
#define ASPEED_MCTP_ROUTING_TYPE_H(x)	((((x) & 0x2) >> 1) << 12)
#define ASPEED_MCTP_PKG_SIZE(x)		(((x) & 0x7ff) << 2)
#define ASPEED_MCTP_PAD_LEN(x)		((x) & 0x3)

/* TX CMD desc1 */
#define ASPEED_MCTP_LAST_CMD		BIT(31)
#define ASPEED_MCTP_TX_DATA_ADDR(x)	((((x) >> 7) & 0x7fffff) << 8)
#define ASPEED_MCTP_DEST_EP_ID(x)	((x) & 0xff)

/* RX CMD desc0 */
#define ASPEED_MCTP_GET_PKG_LEN(x)	(((x) >> 24) & 0x7f)
#define ASPEED_MCTP_GET_SRC_EP_ID(x)	(((x) >> 16) & 0xff)
#define ASPEED_MCTP_GET_ROUTING_TYPE(x)	(((x) >> 14) & 0x7)
#define ASPEED_MCTP_GET_SEQ_NO(x)	(((x) >> 11) & 0x3)
#define ASPEED_MCTP_SOM			BIT(7)
#define ASPEED_MCTP_EOM			BIT(6)
#define ASPEED_MCTP_GET_PAD_LEN(x)	(((x) >> 4) & 0x3)
#define ASPEED_MCTP_CMD_UPDATE		BIT(0)
/* RX CMD desc1 */
#define ASPEED_MCTP_RX_DATA_ADDR(x)	((((x) >> 7) & 0x3fffff) << 7)

/*
 * Available memory size 4092 bytes:
 * 1st 1024 : cmd desc - 0~511 : tx desc , 512 ~ 1024 : rx desc
 * 2nd 1024 : tx data
 * 3rd 1024 : rx data - 8 - 0x00, 0x80, 0x100, 0x180, each 128 align
 * 4th 1024 : rx data combine
 */
#define ASPEED_MCTP_RX_BUF_NUM		8
#define ASPEED_MCTP_DMA_CHUNK1_OFF	1024
#define ASPEED_MCTP_DMA_CHUNK2_OFF	2048
#define ASPEED_MCTP_DMA_CHUNK3_OFF	3072
#define ASPEED_MCTP_RX_DATA_BLOCK1	0x080
#define ASPEED_MCTP_RX_DATA_BLOCK2	0x100
#define ASPEED_MCTP_RX_DATA_BLOCK3	0x180
#define ASPEED_MCTP_DMA_SIZE		4096
#define ASPEED_MCTP_DESC_BUF_SIZE	64
#define ASPEED_MCTP_DESC_SIZE		512

struct aspeed_mctp_cmd_desc {
	unsigned int desc0;
	unsigned int desc1;
};

struct aspeed_mctp_tx {
	struct aspeed_mctp_cmd_desc	*tx_cmd_desc;
	dma_addr_t			tx_cmd_desc_dma;
	u8				*tx_data;
	dma_addr_t			tx_data_dma;
};

struct aspeed_mctp_rx {
	struct aspeed_mctp_cmd_desc	*rx_cmd_desc;
	dma_addr_t			rx_cmd_desc_dma;
	u8				*rx_data;
	dma_addr_t			rx_data_dma;
	u32				rx_ind;
	u8				*rx_fifo;
	u8				rx_fifo_index;
	bool				rx_fifo_done;
	u32				rx_len;
};

struct aspeed_mctp_info {
	void __iomem			*reg_base;
	int				irq;
	u32				dram_base;
	wait_queue_head_t		mctp_wq;
	u8				*mctp_dma;
	dma_addr_t			mctp_dma_addr;
	struct aspeed_mctp_tx		tx;
	struct aspeed_mctp_rx		rx;
	u32				flag;
	bool				is_open;
	u32				state;
	u8				ep_id;
	u8				ret;
	u8				seq_no;
};

static DEFINE_SPINLOCK(mctp_state_lock);

static inline u32
aspeed_mctp_read(struct aspeed_mctp_info *aspeed_mctp, u32 reg)
{
	u32 val;

	val = readl(aspeed_mctp->reg_base + reg);

	return val;
}

static inline void
aspeed_mctp_write(struct aspeed_mctp_info *aspeed_mctp, u32 val, u32 reg)
{
	writel(val, aspeed_mctp->reg_base + reg);
}

void aspeed_mctp_wait_tx_complete(struct aspeed_mctp_info *aspeed_mctp)
{
	wait_event_interruptible(aspeed_mctp->mctp_wq, (aspeed_mctp->flag ==
				 ASPEED_MCTP_TX_LAST));
	aspeed_mctp->flag &= ~ASPEED_MCTP_TX_LAST;
}

void aspeed_mctp_wait_rx_complete(struct aspeed_mctp_info *aspeed_mctp)
{
	wait_event_interruptible(aspeed_mctp->mctp_wq, (aspeed_mctp->flag ==
				 ASPEED_MCTP_SOM));
	aspeed_mctp->flag &= ~ASPEED_MCTP_SOM;
}

#define TX_DESC		aspeed_mctp->tx.tx_cmd_desc
#define TX_DESC0	aspeed_mctp->tx.tx_cmd_desc->desc0
#define TX_DESC1	aspeed_mctp->tx.tx_cmd_desc->desc1
#define TX_DESC_DMA	aspeed_mctp->tx.tx_cmd_desc_dma
#define TX_DATA		aspeed_mctp->tx.tx_data
#define TX_DATA_DMA	aspeed_mctp->tx.tx_data_dma

static void
aspeed_mctp_tx_xfer(struct aspeed_mctp_info *aspeed_mctp,
		    struct aspeed_mctp_xfer *mctp_xfer)
{
	u32 xfer_len = (mctp_xfer->xfer_len / 4);
	u32 padding_len = 0;

	if (mctp_xfer->xfer_len % 4) {
		xfer_len++;
		padding_len = 4 - ((mctp_xfer->xfer_len) % 4);
	}

	/* Routing type [desc0 bit 12, desc0 bit 14], bit 15 : irq enable */
	TX_DESC0 = ASPEED_MCTP_TX_INT_EN | ASPEED_MCTP_TX_TAG_OWNER |
		   ASPEED_MCTP_ROUTING_TYPE_H(mctp_xfer->ret) |
		   ASPEED_MCTP_ROUTING_TYPE_L(mctp_xfer->ret) |
		   ASPEED_MCTP_PKG_SIZE(xfer_len) |
		   ASPEED_MCTP_PCI_BUS(mctp_xfer->pci_bus) |
		   ASPEED_MCTP_PCI_DEV(mctp_xfer->pci_dev) |
		   ASPEED_MCTP_PCI_FUNC(mctp_xfer->pci_fun) |
		   ASPEED_MCTP_PAD_LEN(padding_len);

	TX_DESC1 |= ASPEED_MCTP_LAST_CMD | ASPEED_MCTP_DEST_EP_ID(0);
	memset(TX_DATA, 0, ASPEED_MCTP_XFER_BUFF_SIZE);
	memcpy(TX_DATA, mctp_xfer->xfer_buff,
	       mctp_xfer->xfer_len);

	/* Trigger TX */
	aspeed_mctp_write(aspeed_mctp, aspeed_mctp_read(aspeed_mctp,
							ASPEED_MCTP_CTRL) |
			  ASPEED_MCTP_TX, ASPEED_MCTP_CTRL);

	/* Wait interrupt */
	aspeed_mctp_wait_tx_complete(aspeed_mctp);
}

#define RX_DESC		aspeed_mctp->rx.rx_cmd_desc
#define RX_DESC0 \
		aspeed_mctp->rx.rx_cmd_desc[aspeed_mctp->rx.rx_ind].desc0
#define RX_DESC_DMA	aspeed_mctp->rx.rx_cmd_desc_dma
#define RX_FIFO		aspeed_mctp->rx.rx_fifo
#define RX_FIFO_ID	aspeed_mctp->rx.rx_fifo_index
#define RX_FIFO_DONE	aspeed_mctp->rx.rx_fifo_done
#define RX_DATA		aspeed_mctp->rx.rx_data
#define RX_DATA_DMA	aspeed_mctp->rx.rx_data_dma

static void aspeed_mctp_rx_combine_data(struct aspeed_mctp_info *aspeed_mctp)
{
	u32 padding_len = 0;
	u32 rx_len = 0;
	int i;

	for (i = 0; i < ASPEED_MCTP_RX_BUF_NUM; i++) {
		aspeed_mctp->rx.rx_ind %= ASPEED_MCTP_RX_BUF_NUM;

		if (!(RX_DESC0 & ASPEED_MCTP_CMD_UPDATE))
			break;

		if (!RX_FIFO_DONE) {
			if (ASPEED_MCTP_SOM & RX_DESC0) {
				RX_FIFO_ID = 0;
				RX_FIFO_DONE = true;
				padding_len = ASPEED_MCTP_GET_PAD_LEN(RX_DESC0);
				aspeed_mctp->flag |= ASPEED_MCTP_SOM;
			}

			rx_len = ASPEED_MCTP_GET_PKG_LEN(RX_DESC0) * 4;
			rx_len -= padding_len;

			memcpy(RX_FIFO +
			      (ASPEED_MCTP_DESC_BUF_SIZE * RX_FIFO_ID),
			       RX_DATA + (aspeed_mctp->rx.rx_ind *
			       ASPEED_MCTP_RX_DATA_BLOCK1), rx_len);
			RX_FIFO_ID++;
			aspeed_mctp->rx.rx_len += rx_len;
			aspeed_mctp->ep_id =
					ASPEED_MCTP_GET_SRC_EP_ID(RX_DESC0);
			aspeed_mctp->ret =
					ASPEED_MCTP_GET_ROUTING_TYPE(RX_DESC0);
			aspeed_mctp->seq_no = ASPEED_MCTP_GET_SEQ_NO(RX_DESC0);
		}

		/* RX CMD desc0 */
		RX_DESC0 = 0;
		aspeed_mctp->rx.rx_ind++;
	}
}

static irqreturn_t aspeed_mctp_isr(int this_irq, void *dev_id)
{
	struct aspeed_mctp_info *aspeed_mctp = dev_id;
	u32 status = aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_ISR) &
				      ASPEED_MCTP_ISR_MASK;

	if (status)
		aspeed_mctp_write(aspeed_mctp, status, ASPEED_MCTP_ISR);

	if (status & ASPEED_MCTP_TX_LAST)
		aspeed_mctp->flag |= ASPEED_MCTP_TX_LAST;

	if (status & ASPEED_MCTP_RX_COMPLETE)
		aspeed_mctp_rx_combine_data(aspeed_mctp);

	if (status & ASPEED_MCTP_RX_NO_CMD)
		aspeed_mctp->flag |= ASPEED_MCTP_RX_NO_CMD;

	if (!aspeed_mctp->flag)
		return IRQ_NONE;

	wake_up_interruptible(&aspeed_mctp->mctp_wq);

	return IRQ_HANDLED;
}

static void aspeed_mctp_ctrl_init(struct aspeed_mctp_info *aspeed_mctp)
{
	int i = 0;

	aspeed_mctp_write(aspeed_mctp, aspeed_mctp->dram_base,
			  ASPEED_MCTP_EID);

	/* TX init */
	aspeed_mctp->mctp_dma = dma_alloc_coherent(NULL, ASPEED_MCTP_DMA_SIZE,
						   &aspeed_mctp->mctp_dma_addr,
						   GFP_KERNEL);

	TX_DESC = (struct aspeed_mctp_cmd_desc *)aspeed_mctp->mctp_dma;
	TX_DESC_DMA = aspeed_mctp->mctp_dma_addr;
	TX_DATA = (u8 *)(aspeed_mctp->mctp_dma + ASPEED_MCTP_DMA_CHUNK1_OFF);
	TX_DATA_DMA = aspeed_mctp->mctp_dma_addr + ASPEED_MCTP_DMA_CHUNK1_OFF;

	for (i = 0; i < ASPEED_MCTP_XFER_BUFF_SIZE; i++)
		TX_DATA[i] = i;

	TX_DESC1 |= ASPEED_MCTP_TX_DATA_ADDR(TX_DATA_DMA);
	aspeed_mctp_write(aspeed_mctp, TX_DESC_DMA, ASPEED_MCTP_TX_CMD);

	/* 8 RX buffer */
	RX_DESC = (struct aspeed_mctp_cmd_desc *)(aspeed_mctp->mctp_dma +
						  ASPEED_MCTP_DESC_SIZE);
	RX_DESC_DMA = aspeed_mctp->mctp_dma_addr + ASPEED_MCTP_DESC_SIZE;
	RX_DATA = (u8 *)(aspeed_mctp->mctp_dma + ASPEED_MCTP_DMA_CHUNK2_OFF);
	RX_DATA_DMA = aspeed_mctp->mctp_dma_addr + ASPEED_MCTP_DMA_CHUNK2_OFF;
	aspeed_mctp->rx.rx_ind = 0;
	RX_FIFO = (u8 *)(aspeed_mctp->mctp_dma + ASPEED_MCTP_DMA_CHUNK3_OFF);
	RX_FIFO_DONE = false;
	RX_FIFO_ID = 0;
	aspeed_mctp->rx.rx_len = 0;
	memset(RX_FIFO, 0, ASPEED_MCTP_XFER_BUFF_SIZE);

	for (i = 0; i < ASPEED_MCTP_RX_BUF_NUM; i++) {
		RX_DESC[i].desc0 = 0;
		RX_DESC[i].desc1 =
			ASPEED_MCTP_RX_DATA_ADDR((RX_DATA_DMA +
						 (ASPEED_MCTP_RX_DATA_BLOCK1 *
						  i)));
	}

	RX_DESC[ASPEED_MCTP_RX_BUF_NUM - 1].desc1 |= ASPEED_MCTP_LAST_CMD;

	aspeed_mctp_write(aspeed_mctp, RX_DESC_DMA, ASPEED_MCTP_RX_CMD);
	aspeed_mctp_write(aspeed_mctp, aspeed_mctp_read(aspeed_mctp,
							ASPEED_MCTP_CTRL) |
							ASPEED_MCTP_RX_RDY,
							ASPEED_MCTP_CTRL);
	aspeed_mctp_write(aspeed_mctp, ASPEED_MCTP_RX_COMPLETE |
			  ASPEED_MCTP_TX_LAST, ASPEED_MCTP_IER);
}

static long
mctp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = dev_get_drvdata(c->this_device);
	void __user *argp = (void __user *)arg;
	struct aspeed_mctp_xfer *xfer;

	switch (cmd) {
	case ASPEED_MCTP_IOCTX:
		xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
		if (!xfer)
			return -ENOMEM;

		if (copy_from_user(&xfer, argp,
				   sizeof(struct aspeed_mctp_xfer)))
			return -EFAULT;

		aspeed_mctp_tx_xfer(aspeed_mctp, xfer);

		break;

	case ASPEED_MCTP_IOCRX:
		/* Wait interrupt */
		aspeed_mctp_wait_rx_complete(aspeed_mctp);

		xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
		if (!xfer)
			return -ENOMEM;

		xfer->xfer_len = aspeed_mctp->rx.rx_len;
		memcpy(xfer->xfer_buff, RX_FIFO,  aspeed_mctp->rx.rx_len);

		if (copy_to_user(argp, xfer, sizeof(struct aspeed_mctp_xfer)))
			return -EFAULT;

		RX_FIFO_DONE = false;
		aspeed_mctp->rx.rx_len = 0;
		memset(RX_FIFO, 0, ASPEED_MCTP_XFER_BUFF_SIZE);

		break;

	default:
		return -ENOTTY;
	}

	kfree(xfer);

	return 0;
}

static int mctp_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = dev_get_drvdata(c->this_device);

	spin_lock(&mctp_state_lock);

	if (aspeed_mctp->is_open) {
		spin_unlock(&mctp_state_lock);
		return -EBUSY;
	}

	aspeed_mctp->is_open = true;

	spin_unlock(&mctp_state_lock);

	return 0;
}

static int mctp_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = dev_get_drvdata(c->this_device);

	spin_lock(&mctp_state_lock);

	aspeed_mctp->is_open = false;

	spin_unlock(&mctp_state_lock);

	return 0;
}

static const struct file_operations aspeed_mctp_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= mctp_ioctl,
	.open		= mctp_open,
	.release	= mctp_release,
};

struct miscdevice aspeed_mctp_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "aspeed-mctp",
	.fops	= &aspeed_mctp_fops,
};

static int aspeed_mctp_probe(struct platform_device *pdev)
{
	struct aspeed_mctp_info *aspeed_mctp;
	struct resource *res;
	int ret;

	if (!of_device_is_compatible(pdev->dev.of_node,
				     "aspeed,aspeed-mctp"))
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		return -ENOENT;
	}

	if (!devm_request_mem_region(&pdev->dev, res->start,
				     resource_size(res), res->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		return -EBUSY;
	}

	aspeed_mctp = devm_kzalloc(&pdev->dev, sizeof(*aspeed_mctp),
				   GFP_KERNEL);
	if (!aspeed_mctp)
		return -ENOMEM;

	aspeed_mctp->reg_base = devm_ioremap(&pdev->dev, res->start,
					     resource_size(res));
	if (!aspeed_mctp->reg_base) {
		dev_err(&pdev->dev, "unable to remap MMIO\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_BUS, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_BUS\n");
		return -ENOENT;
	}

	aspeed_mctp->dram_base = (u32)res->start;

	aspeed_mctp->irq = platform_get_irq(pdev, 0);
	if (aspeed_mctp->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return -ENOENT;
	}

	ret = devm_request_irq(&pdev->dev, aspeed_mctp->irq, aspeed_mctp_isr,
			       IRQF_TRIGGER_NONE, "aspeed-mctp", aspeed_mctp);
	if (ret) {
		dev_err(&pdev->dev, "unable to get IRQ");
		return ret;
	}

	aspeed_mctp->flag = 0;
	init_waitqueue_head(&aspeed_mctp->mctp_wq);

	ret = misc_register(&aspeed_mctp_misc);
	if (ret) {
		dev_err(&pdev->dev, "failed to register misc device\n");
		return ret;
	}

	platform_set_drvdata(pdev, aspeed_mctp);
	dev_set_drvdata(aspeed_mctp_misc.this_device, aspeed_mctp);

	aspeed_mctp_ctrl_init(aspeed_mctp);

	return 0;
}

static int aspeed_mctp_remove(struct platform_device *pdev)
{
	misc_deregister(&aspeed_mctp_misc);

	return 0;
}

#ifdef CONFIG_PM
static int
aspeed_mctp_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
aspeed_mctp_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define aspeed_mctp_suspend        NULL
#define aspeed_mctp_resume         NULL
#endif

static const struct of_device_id aspeed_mctp_of_table[] = {
	{ .compatible = "aspeed,aspeed-mctp", },
	{ },
};
MODULE_DEVICE_TABLE(platform, aspeed_mctp_of_table);

static struct platform_driver aspeed_mctp_driver = {
	.probe		= aspeed_mctp_probe,
	.remove		= aspeed_mctp_remove,
	.suspend        = aspeed_mctp_suspend,
	.resume         = aspeed_mctp_resume,
	.driver         = {
		.name   = "aspeed-mctp",
		.owner  = THIS_MODULE,
		.of_match_table	= aspeed_mctp_of_table,
	},
};

module_platform_driver(aspeed_mctp_driver);

MODULE_AUTHOR("Vadim Pasternak <vadimp@mellanox.com>");
MODULE_DESCRIPTION("Aspeed MCTP driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:aspeed-mctp");
