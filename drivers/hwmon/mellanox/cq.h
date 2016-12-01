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

#ifndef SX_CQ_H
#define SX_CQ_H

/************************************************
 * Includes
 ***********************************************/

#include <linux/types.h>
#include "sx.h"

#define CQN_INVALID 255

/************************************************
 * Enums
 ***********************************************/

enum {
	SX_CQE_OWNER_MASK		= 0x01,
	SX_CQE_IS_SEND_MASK		= 0x40,
	SX_CQE_IS_ERR_MASK		= 0x80,
	SX_CQE_DQN_MASK			= 0x1f,
	SX_CQE_DQN_MSB_MASK		= 0x4000,
	SX_CQE_DQN_MSB_SHIFT	= 5,
};

/************************************************
 * Structures
 ***********************************************/

struct sx_cq_context {
	u8		eq_num;  /* 0 or 1 */
	u8		reserved1;
	u8		flags;
	u8		log_cq_size;
	u16		reserved2;
	__be16	producer_counter;
	u16		reserved3;
	u16 	reserved4;
	u32		reserved5;
	__be64	dma_addr[8];  /* CQE buffer dma addresses */
};

struct sx_cqe {
	u8		vlan2_lag_subport;
	u8		lag;
	__be16	system_port_lag_id;
	__be16	wqe_counter;
	__be16	dqn5_byte_count;
	__be16  ulp_crc_vlan_flow;
	__be16	trap_id;
	__be16	reserved1;
	u8		type_swid;
	u8		e_sr_dqn_owner;
};

/************************************************
 * Inline Functions
 ***********************************************/

static inline void sx_cq_arm(struct sx_cq *cq)
{
#ifndef NO_PCI
	unsigned long flags;

   	spin_lock_irqsave(&cq->rearm_lock, flags);

	/*
	 * Make sure that descriptors are written before
	 * doorbell.
	 */
	wmb();

	__raw_writel((__force u32) cpu_to_be32(cq->cons_index & 0xffff),
		     cq->arm_db);

	mmiowb();
	spin_unlock_irqrestore(&cq->rearm_lock, flags);
#endif
}

/* Update the Consumer Index */
static inline void sx_cq_set_ci(struct sx_cq *cq)
{
#ifndef NO_PCI
	/*
	 * Make sure that descriptors are written before
	 * doorbell.
	 */
	wmb();

	__raw_writel((__force u32) cpu_to_be32((cq->cons_index + cq->nent) &
			0xffff), cq->set_ci_db);

	mmiowb();
#endif
}

/************************************************
 * Functions
 ***********************************************/
void dispatch_pkt(struct sx_dev *dev, struct completion_info *ci, u16 hw_synd, int dispatch_default);
int sx_core_init_cq_table(struct sx_dev *dev);
void sx_core_destroy_cq_table(struct sx_dev *dev);
int sx_core_create_cq(struct sx_dev *dev, int nent, struct sx_cq **cq, u8 cqn);
void sx_core_destroy_cq(struct sx_dev *dev, struct sx_cq *cq);
int sx_cq_completion(struct sx_dev *dev, u32 cqn, u16 weight);
void sx_core_dump_synd_tbl(struct sx_dev *dev);
int rx_skb(void *context, struct sk_buff *skb, struct sx_cqe *cqe);
int sx_cq_credit_thread_handler(void *cq_ctx);
void wqe_sync_for_cpu(struct sx_dq *dq, int idx);
void sx_cq_show_cq(struct sx_dev *dev, int cqn);
void sx_cq_flush_rdq(struct sx_dev *my_dev, int idx);


#endif /* SX_CQ_H */

/************************************************
 *                  EOF                         *
 ***********************************************/

