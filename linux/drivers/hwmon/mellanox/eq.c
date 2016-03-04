/*
 * Copyright (C) Mellanox Technologies, Ltd. 2010-2015 ALL RIGHTS RESERVED.
 *
 * This software product is a proprietary product of Mellanox Technologies, Ltd.
 * (the "Company") and all right, title, and interest in and to the software product,
 * including all associated intellectual property rights, are and shall
 * remain exclusively with the Company.
 *
 * This software product is governed by the End User License Agreement
 * provided with the software product.
 *
 */

/************************************************
 * Includes                                     *
 ***********************************************/

#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/mlx_sx/device.h>
#include <linux/mlx_sx/cmd.h>
#include "sx.h"
#include "eq.h"
#include "cq.h"
#include "alloc.h"

#define SX_DBELL_EQ_CI_OFFSET	0x600
#define SX_DBELL_EQ_ARM_OFFSET	0xa00
#define SX_CLR_INT_SIZE			0x00004

extern atomic_t cq_backup_polling_enabled;

static void sx_intr_tasklet_handler(unsigned long data);

/************************************************
 * Functions                                    *
 ***********************************************/

static void sx_eq_set_ci(struct sx_eq *eq, int req_not)
{
	__raw_writel((__force u32)
			cpu_to_be32((eq->cons_index + eq->nent) & 0xffff),
			eq->ci_db);
	/* We want to make sure the first one is written before the second one */
	mb();

	if (req_not)
		__raw_writel((__force u32) cpu_to_be32(eq->cons_index & 0xffff),
						       eq->arm_db);
	/* We still want ordering, just not swabbing, so add a barrier */
	mb();
}

static struct sx_eqe *sx_get_eqe(struct sx_eq *eq, u32 entry)
{
	unsigned long off = (entry & (eq->nent - 1)) * sizeof(struct sx_eqe);
	return eq->page_list[off / PAGE_SIZE].buf + off % PAGE_SIZE;
}

static struct sx_eqe *sx_next_eqe_sw(struct sx_eq *eq)
{
	struct sx_eqe *eqe = sx_get_eqe(eq, eq->cons_index);
	return (eqe->owner & 0x1) ^ !!(eq->cons_index & eq->nent) ? NULL : eqe;
}

static u16 sx_iterate_eq(struct sx_dev *dev, struct sx_eq *eq,
		struct sx_bitmap *cq_bitmap, int *set_ci)
{
	struct sx_eqe *eqe;
	u16 num_current_cqs = 0;

	/* In Pelican it's possible that we get an interrupt
	 * before the EQe is written. So we will ignore it and
	 * just update the ci, so another interrupt will happen
	 * when the EQe is written */
	while ((eqe = sx_next_eqe_sw(eq))) {
		/*
		 * Make sure we read EQ entry contents after we've
		 * checked the ownership bit.
		 */
		rmb();

		switch (eqe->type) {
		case SX_EVENT_TYPE_COMP:
			if (!cq_bitmap) /* This should never happen */
				break;
			if (sx_bitmap_test(cq_bitmap, eqe->cqn))
				break; /* This CQ's bit is already set */
			sx_bitmap_set(cq_bitmap, eqe->cqn);
			++num_current_cqs;
			break;

		case SX_EVENT_TYPE_CMD:
			sx_cmd_event(dev,
				     be16_to_cpu(eqe->event.cmd.token),
				     eqe->event.cmd.status,
				     be64_to_cpu(eqe->event.cmd.out_param));
			break;
		default:
			sx_warn(dev, "Unhandled event %02x(%02x) on EQ %d "
			"at index %u\n", eqe->type, eqe->subtype, eq->eqn,
			eq->cons_index);
			break;
		};

		++eq->cons_index;
		++(*set_ci);

		/*
		 * The HCA will think the queue has overflowed if we
		 * don't tell it we've been processing events.  We
		 * create our EQs with SX_NUM_SPARE_EQE extra
		 * entries, so we must update our consumer index at
		 * least that often.
		 */
		if (unlikely((*set_ci) >= SX_NUM_SPARE_EQE)) {
			/*
			 * Conditional on hca_type is OK here because
			 * this is a rare case, not the fast path.
			 */
			sx_eq_set_ci(eq, 0);
			*set_ci = 0;
		}
	}

	return num_current_cqs;
}

static void sx_eq_int(struct sx_dev *dev, struct sx_eq *eq)
{
	struct sx_bitmap cq_bitmap;
	int i;
	int err = 0;
	u16 weight;
	u16 num_current_cqs = 0;
	int set_ci = 0;
	int cq_bkp_poll_mode;
    struct sx_priv *priv = sx_priv(dev);

	if (eq->eqn == SX_EQ_ASYNC) {
		sx_iterate_eq(dev, eq, NULL, &set_ci);
		goto out;
	}

	err = sx_bitmap_init(&cq_bitmap, dev->dev_cap.max_num_cqs);
	if (err)
		goto out;

	num_current_cqs = sx_iterate_eq(dev, eq, &cq_bitmap, &set_ci);

	while (num_current_cqs) {
		weight = 1 << dev->dev_cap.log_max_sdq_sz;
		for (i = 0; i < dev->dev_cap.max_num_cqs; i++) {
			if (test_bit(i, cq_bitmap.table)) {
				if (i >= NUMBER_OF_SDQS)
					weight = dev->profile.rdq_properties[i - NUMBER_OF_SDQS].rdq_weight;
				err = sx_cq_completion(dev, i, weight);
				if (err < 0)
					printk(KERN_WARNING PFX "sx_eq_int: "
						"cq_completion failed\n");
				else if (err < weight) {
					sx_bitmap_free(&cq_bitmap, i);
					--num_current_cqs;
				}
			}
		}

		num_current_cqs += sx_iterate_eq(dev, eq, &cq_bitmap, &set_ci);
	}

out:
    sx_eq_set_ci(eq, 1);

    /* Backup poll */
    if (priv && priv->cq_table.cq_rl_params &&
        atomic_read(&cq_backup_polling_enabled) &&
        atomic_read(&priv->cq_backup_polling_refcnt)) {
        int found=0;
        for (i = NUMBER_OF_SDQS; i < dev->dev_cap.max_num_cqs; i++) {
            if (!priv->cq_table.cq[i])
                continue;
            cq_bkp_poll_mode=atomic_read(&priv->cq_table.cq[i]->bkp_poll_data.cq_bkp_poll_mode);
            if (cq_bkp_poll_mode) {
                found=1;
                atomic_set(&priv->cq_table.cq[i]->bkp_poll_data.cq_bkp_poll_mode,0);
                atomic_dec(&priv->cq_backup_polling_refcnt);
                if (net_ratelimit()) {
                printk(KERN_INFO PFX "sx_eq_int: doing backup poll for QN:%d\n",i);
                }
                weight = dev->profile.rdq_properties[i - NUMBER_OF_SDQS].rdq_weight;
                err = sx_cq_completion(dev, i, weight);
                if (err < 0)
                    printk(KERN_WARNING PFX "sx_eq_int: "
                        "backup poll cq_completion failed\n");
            }
        }
        if (!found && atomic_read(&priv->cq_backup_polling_refcnt)) {
            if (net_ratelimit()) {
                printk(KERN_INFO PFX "sx_eq_int: vain backup polling\n");
            }
            atomic_set(&priv->cq_backup_polling_refcnt,0);
        }
    }    

}

static void sx_intr_tasklet_handler(unsigned long data)
{
	struct sx_dev *dev = (struct sx_dev *)data;
	struct sx_priv *priv = sx_priv(dev);
	int i;

	for (i = 0; i < SX_NUM_EQ; ++i) {
		if (!sx_bitmap_test(&priv->eq_table.bitmap, priv->eq_table.eq[i].eqn)) {
			/* This is for avoiding cases of receiving interrupts during
			 * deinit process, which happened on multi-core CPUs */
			printk(KERN_DEBUG PFX "sx_intr_tasklet_handler: Skipping EQ %d "
					"which was already freed\n",priv->eq_table.eq[i].eqn);
			continue;
		}

		sx_eq_int(dev, &priv->eq_table.eq[i]);
	}
}

static irqreturn_t sx_interrupt(int irq, void *dev_ptr)
{
	struct sx_dev *dev = dev_ptr;
	struct sx_priv *priv = sx_priv(dev);

	writel(priv->eq_table.clr_mask, priv->eq_table.clr_int);

	tasklet_schedule(&priv->intr_tasklet);

	return IRQ_HANDLED;
}

static irqreturn_t sx_msi_x_interrupt(int irq, void *dev_ptr)
{
    struct sx_dev  *dev = dev_ptr;
    struct sx_priv *priv = sx_priv(dev);
    
	tasklet_schedule(&priv->intr_tasklet);

	return IRQ_HANDLED;
}


static int sx_SW2HW_EQ(struct sx_dev *dev, struct sx_cmd_mailbox *mailbox,
			 int eq_num)
{
	return sx_cmd(dev, dev->device_id, mailbox, eq_num, 0, SX_CMD_SW2HW_EQ,
			SX_CMD_TIME_CLASS_A, sx_priv(dev)->fw.local_in_mb_size);
}

static int sx_HW2SW_EQ(struct sx_dev *dev, int eq_num)
{
	return sx_cmd(dev, dev->device_id, 0, eq_num, 0, SX_CMD_HW2SW_EQ,
			SX_CMD_TIME_CLASS_A,
			sx_priv(dev)->fw.local_in_mb_size);
}


static void sx_free_eq(struct sx_dev *dev, struct sx_eq *eq)
{
	struct sx_priv *priv = sx_priv(dev);
	int err;
	int npages = PAGE_ALIGN(sizeof(struct sx_eqe) * eq->nent) / PAGE_SIZE;
	int i;

	sx_bitmap_free(&priv->eq_table.bitmap, eq->eqn);
	err = sx_HW2SW_EQ(dev, eq->eqn);
	if (err) {
		sx_warn(dev, "HW2SW_EQ failed (%d)\n", err);
	}

	for (i = 0; i < npages; ++i)
		pci_free_consistent(dev->pdev, PAGE_SIZE,
					eq->page_list[i].buf,
					eq->page_list[i].map);

	kfree(eq->page_list);
}

static int sx_create_eq(struct sx_dev *dev, int nent,
		struct sx_eq *eq)
{
	struct sx_priv *priv = sx_priv(dev);
	struct sx_cmd_mailbox *mailbox;
	struct sx_eq_context *eq_context;
	int npages;
	int err = -ENOMEM;
	int i;

	eq->dev   = dev;
	eq->nent  = roundup_pow_of_two(max(nent, 2));
	npages = PAGE_ALIGN(eq->nent * sizeof(struct sx_eqe)) / PAGE_SIZE;

	eq->page_list = kmalloc(npages * sizeof(*eq->page_list), GFP_KERNEL);
	if (!eq->page_list)
		goto err_out;

	for (i = 0; i < npages; ++i)
		eq->page_list[i].buf = NULL;

	mailbox = sx_alloc_cmd_mailbox(dev, dev->device_id);
	if (IS_ERR(mailbox))
		goto err_out_free;

	eq_context = mailbox->buf;

	for (i = 0; i < npages; ++i) {
		eq->page_list[i].buf = dma_alloc_coherent(&dev->pdev->dev,
							  PAGE_SIZE,
							  &eq->page_list[i].map,
							  GFP_KERNEL);
		if (!eq->page_list[i].buf)
			goto err_out_free_pages;
		memset(eq->page_list[i].buf, 0x1, PAGE_SIZE);
	}

	eq->eqn = sx_bitmap_alloc(&priv->eq_table.bitmap);
	if (eq->eqn == -1)
		goto err_out_free_pages;

	eq->ci_db = dev->db_base + SX_DBELL_EQ_CI_OFFSET + 4 * eq->eqn;
	eq->arm_db = dev->db_base + SX_DBELL_EQ_ARM_OFFSET + 4 * eq->eqn;
	memset(eq_context, 0, sizeof *eq_context);
	eq_context->flags	  = SX_EQ_STATE_ARMED;
	eq_context->log_eq_size	  = ilog2(eq->nent) & 0xf;
	eq_context->int_msi_x	  = !!(dev->flags & SX_FLAG_MSI_X);

	for (i = 0; i < npages; ++i)
		eq_context->dma_addr[i] = cpu_to_be64(eq->page_list[i].map);

	err = sx_SW2HW_EQ(dev, mailbox, eq->eqn);
	if (err) {
		sx_warn(dev, "SW2HW_EQ failed (%d)\n", err);
		goto err_out_free_eq;
	}

	sx_free_cmd_mailbox(dev, mailbox);
	eq->cons_index = 0;

	return err;

err_out_free_eq:
	sx_bitmap_free(&priv->eq_table.bitmap, eq->eqn);

err_out_free_pages:
	for (i = 0; i < npages; ++i) {
		if (eq->page_list[i].buf) {
			dma_free_coherent(&dev->pdev->dev, PAGE_SIZE,
					  eq->page_list[i].buf,
					  eq->page_list[i].map);
		}
	}

	sx_free_cmd_mailbox(dev, mailbox);

err_out_free:
	kfree(eq->page_list);

err_out:
	return err;
}

static void sx_free_irqs(struct sx_dev *dev)
{
	struct sx_eq_table *eq_table = &sx_priv(dev)->eq_table;

	if (eq_table->have_irq)
		free_irq(dev->pdev->irq, dev);
	if (eq_table->eq[0].have_irq) {
		free_irq(eq_table->eq[0].irq, dev);
		eq_table->eq[0].have_irq = 0;
	}
}

static int sx_map_clr_int(struct sx_dev *dev)
{
	struct sx_priv *priv = sx_priv(dev);

	priv->clr_base = ioremap(pci_resource_start(dev->pdev,
	priv->fw.clr_int_bar) + priv->fw.clr_int_base, SX_CLR_INT_SIZE);
	if (!priv->clr_base) {
		sx_err(dev, "Couldn't map interrupt clear" \
		"register, aborting.\n");
		return -ENOMEM;
	}

	return 0;
}

static void sx_unmap_clr_int(struct sx_dev *dev)
{
	struct sx_priv *priv = sx_priv(dev);

	iounmap(priv->clr_base);
}

int sx_init_eq_table(struct sx_dev *dev)
{
	struct sx_priv *priv = sx_priv(dev);
	int i, err;

#ifdef NO_PCI
	/* No need to create EQs in simulator mode */
	return 0;
#endif
	
	tasklet_init(&priv->intr_tasklet, sx_intr_tasklet_handler,
			(unsigned long)dev);
	err = sx_bitmap_init(&priv->eq_table.bitmap, SX_NUM_EQ);
	if (err)
		return err;

	err = sx_map_clr_int(dev);
	if (err)
		goto err_out;

	priv->eq_table.clr_mask = swab32(1 << (priv->eq_table.inta_pin & 31));
	priv->eq_table.clr_int  = priv->clr_base;
	err = sx_create_eq(dev, SX_NUM_ASYNC_EQE, &priv->eq_table.eq[SX_EQ_ASYNC]);
	if (err)
		goto err_out_unmap;

	err = sx_create_eq(dev, SX_NUM_COMP_EQE, &priv->eq_table.eq[SX_EQ_COMP]);
	if (err)
		goto err_out_async;

	if (dev->flags & SX_FLAG_MSI_X) {
		err = request_irq(priv->eq_table.eq[0].irq,
				  sx_msi_x_interrupt,
				  0, DRV_NAME "_msix", dev);
		if (err)
			goto err_out_async;

		priv->eq_table.eq[SX_EQ_COMP].have_irq = 1;
		priv->eq_table.eq[SX_EQ_ASYNC].have_irq = 1;

	} else {
		err = request_irq(dev->pdev->irq, sx_interrupt,
				  IRQF_SHARED, DRV_NAME, dev);
		if (err)
			goto err_out_comp;

		priv->eq_table.have_irq = 1;
	}

	for (i = 0; i < SX_NUM_EQ; ++i)
		sx_eq_set_ci(&priv->eq_table.eq[i], 1);

	return 0;

err_out_comp:
	sx_free_eq(dev, &priv->eq_table.eq[SX_EQ_COMP]);

err_out_async:
	sx_free_eq(dev, &priv->eq_table.eq[SX_EQ_ASYNC]);

err_out_unmap:
	sx_unmap_clr_int(dev);
	sx_free_irqs(dev);

err_out:

	return err;
}


void sx_cleanup_eq_table(struct sx_dev *dev)
{
	struct sx_priv *priv = sx_priv(dev);
	int i;

#ifdef NO_PCI
	return;
#endif

	sx_free_irqs(dev);
	for (i = 0; i < SX_NUM_EQ; ++i)
		sx_free_eq(dev, &priv->eq_table.eq[i]);

	sx_unmap_clr_int(dev);

	tasklet_kill(&priv->intr_tasklet);
}


/************************************************
 *                  EOF                         *
 ***********************************************/

