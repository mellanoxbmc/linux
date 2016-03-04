/*
 * Copyright (C) Mellanox Technologies, Ltd. 2010-2016 ALL RIGHTS RESERVED.
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
 * Includes
 ***********************************************/

#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/hardirq.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/mlx_sx/device.h>
#include <linux/mlx_sx/driver.h>
#include <linux/mlx_sx/cmd.h>
#include <linux/mlx_sx/kernel_user.h>
#include "sx.h"
#include "cq.h"
#include "dq.h"
#include "ib.h"
#include "alloc.h"
#include "sx_proc.h"

/************************************************
 *  Definitions
 ***********************************************/

#define MAX_MATCHING_LISTENERS 100
#define ETH_CRC_LENGTH 4
#define IB_CRC_LENGTH 6

/************************************************
 * Globals
 ***********************************************/

extern struct sx_globals sx_glb;
extern int rx_debug;
extern int rx_debug_pkt_type;
extern int rx_debug_emad_type;
extern int rx_dump;
extern int rx_dump_cnt;
unsigned int credit_thread_vals[1001] = {0};
unsigned int arr_count = 0;
atomic_t cq_backup_polling_enabled=ATOMIC_INIT(1);
int debug_cq_backup_poll_cqn=CQN_INVALID;
struct handler_entry {
	cq_handler handler;
	void *context;
};

/************************************************
 *  Functions
 ***********************************************/

static u16 get_truncate_size_from_db(struct sx_dev *dev, int dqn)
{
	u16 ret = 0;
    unsigned long flags;

	spin_lock_irqsave(&sx_priv(dev)->db_lock,flags);
	ret = sx_priv(dev)->truncate_size_db[dqn];
	spin_unlock_irqrestore(&sx_priv(dev)->db_lock,flags);

	return ret;
}

/* Returns 1 if the port/lag id is found in the trap filter DB and the packet should be dropped */
static u8 check_trap_port_in_filter_db(struct sx_dev *dev, u16 hw_synd,
		u8 is_lag, u16 sysport_lag_id)
{
	int i;
	u8 ret = 0;
    unsigned long flags;

	if (is_lag) {
		u16 lag_id = (sysport_lag_id >> 4) & 0xfff;
		spin_lock_irqsave(&sx_priv(dev)->db_lock,flags);
		for (i = 0; i < MAX_LAG_PORTS_IN_FILTER; i++) {
			if (sx_priv(dev)->lag_filter_db[hw_synd][i] == lag_id) {
				inc_filtered_lag_packets_counter(dev);
				ret = 1;
				break;
			}
		}

		spin_unlock_irqrestore(&sx_priv(dev)->db_lock,flags);
		return ret;
	}

	/* EMADs can be received with sysport==0 */
	if (sysport_lag_id == 0)
		return 0;

	spin_lock_irqsave(&sx_priv(dev)->db_lock,flags);
	for (i = 0; i < MAX_SYSTEM_PORTS_IN_FILTER; i++) {
		if (sx_priv(dev)->sysport_filter_db[hw_synd][i] == sysport_lag_id) {
			inc_filtered_port_packets_counter(dev);
			ret = 1;
			break;
		}
	}

	spin_unlock_irqrestore(&sx_priv(dev)->db_lock,flags);
	return ret;
}

static u16 get_vid_from_db(struct sx_dev *dev, u8 is_lag, u16 sysport_lag_id)
{
	u16 ret = 1;
    unsigned long flags;

	spin_lock_irqsave(&sx_priv(dev)->db_lock,flags);
	if (is_lag) {
		u16 lag_id = (sysport_lag_id >> 4) & 0xfff;
		ret = sx_priv(dev)->pvid_lag_db[lag_id];
	} else
		ret = sx_priv(dev)->pvid_sysport_db[sysport_lag_id];

	spin_unlock_irqrestore(&sx_priv(dev)->db_lock,flags);

	return ret;
}

static u32 get_qpn(u16 hw_synd, struct sk_buff *skb)
{
	u8 lnh = 0;
	u32 qpn;

	lnh = ((struct ib_header_lrh *)skb->data)->sl_lnh & 0x3;
	if (lnh == 3)
		qpn = be32_to_cpu(((struct ib_header_multicast *)
				skb->data)->bth.dest_qp) & 0xffffff;
	else if (lnh == 2)
		qpn = be32_to_cpu(((struct ib_header_unicast *)
				skb->data)->bth.dest_qp) & 0xffffff;
	else
		qpn = 0xffffffff;

	return qpn;
}

static int is_matching(struct completion_info *ci,
		struct listener_entry *listener)
{
	if (listener->swid != ci->swid &&
			listener->swid != SWID_NUM_DONT_CARE) {
		return 0;
	}

	/* If the packet came from a user (loopback), don't return it to the same user */
	if (ci->context != NULL && listener->context == ci->context) {
		return 0;
	}

	switch (listener->listener_type) {
	case L2_TYPE_DONT_CARE:
		if (listener->critireas.dont_care.sysport ==
				SYSPORT_DONT_CARE_VALUE) {
			return 1;
		}

		/* LAGs will also work in the same way */
		if (ci->sysport != listener->critireas.dont_care.sysport) {
			break;
		}

		return 1;
	case L2_TYPE_ETH:
		if ((ci->pkt_type != PKT_TYPE_ETH) &&
				(ci->pkt_type != PKT_TYPE_FCoETH)) {
		    break;
		}

		if (ci->info.eth.ethtype != listener->critireas.eth.ethtype &&
			listener->critireas.eth.ethtype !=
					ETHTYPE_DONT_CARE_VALUE) {
			break;
		}

		if (ci->info.eth.dmac != listener->critireas.eth.dmac
			&& listener->critireas.eth.dmac !=
					DMAC_DONT_CARE_VALUE) {
			break;
		}

		if (listener->critireas.eth.emad_tid != TID_DONT_CARE_VALUE &&
				ci->info.eth.emad_tid !=
						listener->critireas.eth.emad_tid) {
			break;
		}

        if (listener->critireas.eth.from_rp != IS_RP_DONT_CARE_E &&
                ci->info.eth.from_rp != listener->critireas.eth.from_rp) {
            break;
        }

        if (listener->critireas.eth.from_bridge != IS_BRIDGE_DONT_CARE_E &&
                ci->info.eth.from_bridge != listener->critireas.eth.from_bridge) {
            break;
        }

		return 1;
	case L2_TYPE_IB:
		/* TODO: IB Raw packets have no IB header at all so they
		 * need a special handling */
		if (ci->pkt_type == PKT_TYPE_IB_Raw ||
				ci->pkt_type == PKT_TYPE_IB_non_Raw ||
				ci->pkt_type == PKT_TYPE_FCoIB ||
				ci->pkt_type == PKT_TYPE_ETHoIB) {
			if (ci->info.ib.qpn == listener->critireas.ib.qpn ||
				listener->critireas.ib.qpn ==
						QPN_DONT_CARE_VALUE) {
				return 1;
			}
		}

		break;
	default:
		break;
	}

	return 0;
}

/*
 * filter the listeners table, and call all relevant handlers
 */
void dispatch_pkt(struct sx_dev *dev, struct completion_info *ci, u16 entry, int dispatch_default) 
{
	struct listener_entry *listener;
	struct list_head *pos;
	unsigned long flags;
	static struct handler_entry callbacks[MAX_MATCHING_LISTENERS];
	int num_found = 0;
	int i = 0;

	/* validate the syndrome range */
	if (entry > NUM_HW_SYNDROMES){
		printk(KERN_ERR PFX "Error: arrived synd %d is out of range (1..%d) \n",
				entry, NUM_HW_SYNDROMES);
		return;
	}

	spin_lock_irqsave(&sx_glb.listeners_lock, flags);
	/* Checking syndrome registration and NUM_HW_SYNDROMES callback iff dispatch_default set */
	/* I don't like the syndrome dispatchers at all, but it's too late to change */
	while (1) {
		if (!list_empty(&sx_glb.listeners_db[entry].list)) {
			list_for_each(pos, &sx_glb.listeners_db[entry].list) {
				listener = list_entry(pos, struct listener_entry, list);

				if ((listener->is_default && num_found == 0) || is_matching(ci, listener)) {
					callbacks[num_found].handler =
							listener->handler;
					callbacks[num_found].context =
							listener->context;
					listener->rx_pkts++;
					++num_found;
				}
				if (num_found == MAX_MATCHING_LISTENERS)
					break;
			}
		}
		if (!dispatch_default || (entry ==  NUM_HW_SYNDROMES) || (num_found == MAX_MATCHING_LISTENERS))
			break;
		entry = NUM_HW_SYNDROMES;
	}
	spin_unlock_irqrestore(&sx_glb.listeners_lock, flags);

	for (i = 0; i < num_found; i++)
		callbacks[i].handler(ci, callbacks[i].context);

	if (num_found == 0){
		inc_unconsumed_packets_counter(dev, ci->hw_synd, ci->pkt_type);
	}
}

static int chk_completion_info(struct completion_info *ci)
{
	int err = 0;

	if (ci->swid >= NUMBER_OF_SWIDS && ci->swid != SWID_NUM_DONT_CARE) {
		err = -EINVAL;
#ifdef SX_DEBUG
		if (printk_ratelimit())
			printk(KERN_DEBUG PFX "The given cqe is not valid: " \
				" swid=[%d]\n", ci->swid);
#endif
	}

	if (ci->hw_synd > NUM_HW_SYNDROMES) {
		err = -EINVAL;
#ifdef SX_DEBUG
		if (printk_ratelimit())
			printk(KERN_DEBUG PFX "The given cqe is not valid: " \
				"hw_synd=[%d]\n", ci->hw_synd);
#endif
	}

	return err;
}

/*
 * extracts the needed data from the cqe and from the packet, calls
 * the filter listeners with that info
 */
int rx_skb(void *context, struct sk_buff *skb, struct sx_cqe *cqe)
{
	struct completion_info *ci = NULL;
	u16 hw_synd;
	int err = 0;
	int dqn;
	u16 truncate_size;
	u8 crc_present = 0;
	struct sx_dev * sx_device = (struct sx_dev *)context;
	u8 swid = 0;
	u8 is_from_rp = IS_RP_DONT_CARE_E;
	u16 fid = 0;

#ifdef SX_DEBUG
	printk(KERN_DEBUG PFX "rx_skb: Entered function\n");

{
	__be32 *buf = (void *)cqe;
/*	int i; */

	if (printk_ratelimit())
		printk(KERN_DEBUG "CQE %p contents:\n%08x\n%08x\n%08x\n%08x\n",
				cqe, be32_to_cpu(buf[0]), be32_to_cpu(buf[1]),
				be32_to_cpu(buf[2]), be32_to_cpu(buf[3]));

/*	buf = (void *)skb->data;
	printk(KERN_DEBUG "packet contents:\n");
	for (i = 0; i < be16_to_cpu(cqe->byte_count)/4; i++)
		printk(KERN_DEBUG "%08x\n", be32_to_cpu(buf[i])); */
}

#endif

	ci = kzalloc(sizeof(*ci), GFP_ATOMIC);
	if (!ci) {
			err = -ENOMEM;
			goto out_free_skb;
	}

	hw_synd  = be16_to_cpu(cqe->trap_id) & 0x01FF;
	/* TODO: WA because LP packets return with hw_synd = 0 */
	if (!hw_synd) {
		hw_synd = 1;
#ifdef SX_DEBUG
		printk(KERN_DEBUG PFX "Got a packet with trap_id==0, "
				"probably a LP response\n");
#endif
	}

	/* update skb->len to the real len,
	 * instead of the max len we allocated */
	skb->len = be16_to_cpu(cqe->dqn5_byte_count) & 0x3FFF;
	/* TODO: remove this check in the future. ISX bit means ISX header is present */
	if (be16_to_cpu(cqe->dqn5_byte_count) >> 15) {
		skb->data += ISX_HDR_SIZE;
		skb->len -= ISX_HDR_SIZE;
#ifdef SX_DEBUG
		printk(KERN_DEBUG PFX "Got a packet with ISX header\n");
#endif
	}

	ci->skb = skb;
	ci->swid = (cqe->type_swid >> 1) & 0x7;
	ci->context = NULL;
	ci->dev = sx_device;
	/* We need to remove the CRC if present, or otherwise MTU packets
	 * will be dropped in higher levels. */
	crc_present = cqe->type_swid & 0x1;
	ci->sysport = be16_to_cpu(cqe->system_port_lag_id);
	ci->hw_synd = hw_synd;
	ci->pkt_type = (cqe->type_swid >> 5) & 0x7;
	ci->is_send = (cqe->e_sr_dqn_owner >> 6) & 0x1;
	ci->is_lag = cqe->lag & 0x80 ? 1 : 0;
	if (ci->is_lag) {
		ci->lag_subport = cqe->vlan2_lag_subport & 0x1f;
	}
	else {
		ci->lag_subport = 0;
    }

	/* If packet arrived from external port then ci->sysport != 0 or is_lag != 0 */
	if (ci->sysport != 0 || ci->is_lag != 0) {
		err = sx_core_get_swid(sx_device, ci, &swid);
	    if (err) {
	        err = -EINVAL ;
	        goto out;
	    }
#ifdef SX_DEBUG
        printk(KERN_DEBUG PFX "rx_skb() pkt_type:%d, hw_synd:%d is_lag:%d, sysport:0x%x, "
                "old_swid:%d, new_swid: %d\n", ci->pkt_type, ci->hw_synd, ci->is_lag,
                ci->sysport, ci->swid, swid);
#endif
        ci->swid = swid;
	}
	else if (ci->swid < NUMBER_OF_SWIDS &&
	    sx_device->profile.swid_type[ci->swid] == SX_KU_L2_TYPE_ROUTER_PORT){
		/* if event arrived from Router Port Swid forward it to swid 0 (Default ETH swid) */
	    ci->swid = 0;
	}

	if (rx_debug &&
		(rx_debug_pkt_type == SX_DBG_PACKET_TYPE_ANY ||
		 rx_debug_pkt_type == ci->hw_synd) &&
		(rx_debug_emad_type == SX_DBG_EMAD_TYPE_ANY ||
		rx_debug_emad_type ==
				be16_to_cpu(((struct sx_emad *)skb->data)->emad_op.register_id)) ) {

		__be32 *cqebuf = (void *)cqe;

		printk(KERN_DEBUG "CQE %p contents:\n%08x\n%08x\n%08x\n%08x\n",
				cqe, be32_to_cpu(cqebuf[0]),
				be32_to_cpu(cqebuf[1]),
				be32_to_cpu(cqebuf[2]),
				be32_to_cpu(cqebuf[3]));

		printk(KERN_DEBUG PFX "rx_skb: swid = %d, "
			"sysport = %d, hw_synd = %d (reg_id: 0x%x),"
			"pkt_type = %d, byte_count = %d, is_lag: %d\n",
			ci->swid, ci->sysport, ci->hw_synd,
			be16_to_cpu(((struct sx_emad *)skb->data)->emad_op.register_id),
			ci->pkt_type, skb->len, ci->is_lag);


		if (rx_dump) {
			int i;
			u8 *buf = (void *)skb->data;
			int cnt = skb->len;

			for (i = 0; i < cnt; i++) {
				if (i == 0 || (i%4 == 0))
					printk("\n0x%04x : ", i);

				printk(" 0x%02x", buf[i]);
			}

			printk("\n");
		}

		if (rx_dump_cnt != SX_DBG_COUNT_UNLIMITED && rx_dump_cnt>0)
			rx_dump_cnt--;

		if (rx_dump_cnt == 0){
			rx_dump = 0;
			rx_debug = 0;
			rx_debug_pkt_type = 0xFF;
		}
	}

	if (ci->swid < NUMBER_OF_SWIDS) {
		sx_device->stats.rx_by_pkt_type[ci->swid][ci->pkt_type]++;
		sx_device->stats.rx_by_synd[ci->swid][ci->hw_synd]++;
	} else {
		sx_device->stats.rx_by_pkt_type[NUMBER_OF_SWIDS][ci->pkt_type]++;
		sx_device->stats.rx_by_synd[NUMBER_OF_SWIDS][ci->hw_synd]++;
	}

	/* Check if the port/lag is in the traps filter DB. If so we silently drop the packet */
	if (check_trap_port_in_filter_db((struct sx_dev *)context, ci->hw_synd,
			ci->is_lag, ci->sysport))
		goto out;

	dqn = (cqe->e_sr_dqn_owner >> 1) & SX_CQE_DQN_MASK;
	if (be16_to_cpu(cqe->dqn5_byte_count) & SX_CQE_DQN_MSB_MASK)
		dqn |= (1 << SX_CQE_DQN_MSB_SHIFT);

	/*put the original packet size befor truncation in the complition info*/
	ci->original_packet_size = skb->len;

	truncate_size = get_truncate_size_from_db((struct sx_dev *)context, dqn);
	if (truncate_size > 0 && truncate_size < skb->len)
			skb->len = truncate_size;

	switch (ci->pkt_type) {
	case PKT_TYPE_ETH:
	case PKT_TYPE_FCoETH:
		ci->info.eth.ethtype = be16_to_cpu(((struct sx_eth_hdr *)
				skb->data)->ethertype);
		ci->info.eth.dmac = be64_to_cpu(((struct sx_eth_hdr *)
				skb->data)->dmac_smac1) >> 16;
		if (ci->info.eth.ethtype == ETHTYPE_EMAD)
			ci->info.eth.emad_tid = be64_to_cpu(((struct sx_emad *)
				skb->data)->emad_op.tid) >> 32;
		else if (ci->info.eth.ethtype == ETHTYPE_VLAN) {
			ci->is_tagged = 1;
			/* Router Port is not supported in SwitchX A1.
			 * Get vlan from cqe: vlan [0:9] bits from 16 bit field (ulp_crc_vlan_flow)
			 * and vlan [10:11] bits from 8 bit field (vlan2_lag_subport) */
			ci->vid = ((be16_to_cpu(cqe->ulp_crc_vlan_flow) & 0x3ff0) >> 4) |
			        ((cqe->vlan2_lag_subport & 0x60) << 5);
		}
		else {
			ci->is_tagged = 0;
			ci->vid = get_vid_from_db((struct sx_dev *)context,
					ci->is_lag, ci->sysport);
		}
		if(crc_present){
		    ci->original_packet_size -= ETH_CRC_LENGTH;
		}
		if (crc_present && !truncate_size){
		    skb->len -= ETH_CRC_LENGTH;
		}

        /* if sysport is 0 and is_lag is 0 that the packet is FW event,
            and we don't need to check if from RP / bridge */
		if (ci->sysport != 0 || ci->is_lag != 0) {
            err = sx_core_get_rp_mode((struct sx_dev *)context,
                                      ci->is_lag, ci->sysport, ci->vid,
                                      &is_from_rp);
            if (err) {
                printk(KERN_ERR PFX "Failed sx_core_get_rp_mode(). err: %d \n",err);
            }
            ci->info.eth.from_rp = (is_from_rp) ? IS_RP_FROM_RP_E : IS_RP_NOT_FROM_RP_E;
        
            err = sx_core_get_fid_by_port_vid((struct sx_dev *)context, ci, &fid);
            if (err) {
                printk(KERN_ERR PFX "Failed sx_core_get_bridge(). err: %d \n",err);
            }
            ci->bridge_id = fid;
            ci->info.eth.from_bridge = (fid) ?
                    IS_BRIDGE_FROM_BRIDGE_E : IS_BRIDGE_NOT_FROM_BRIDGE_E;
		}
		break;
	case PKT_TYPE_IB_Raw: /* TODO: Extract qpn from IB Raw pkts */
	case PKT_TYPE_IB_non_Raw:
	case PKT_TYPE_FCoIB:
	case PKT_TYPE_ETHoIB:
		ci->info.ib.qpn = get_qpn(hw_synd, skb);
		if (ci->info.ib.qpn == 0xffffffff) {
			if (printk_ratelimit())
				printk(KERN_WARNING PFX "Received IB packet "
					"is not valid. Dropping the packet\n");
			err = -EINVAL;
			goto out;
		}

		/* Extract the IB port from the sysport */
		ci->sysport = (ci->sysport >> 4) & 0x7f;
		if (crc_present && !truncate_size)
			skb->len -= IB_CRC_LENGTH;
		break;
	default:
		if (printk_ratelimit())
			printk(KERN_WARNING PFX "Received packet type is FC, "
				"and therefore unsupported right now\n");
		err = -EINVAL;
		goto out;
	}

	err = chk_completion_info(ci);
	if (err) {
		err = -EINVAL ;
		goto out;
	}

#ifdef SX_DEBUG
	if (printk_ratelimit())
		printk(KERN_DEBUG PFX " rx_skb() received packet data: "
			"skb->len=[%d] sysport=[%d] hw_synd(trap_id)=[%d] "
			"swid=[%d] pkt_type=[%d]\n",
			ci->skb->len, ci->sysport, ci->hw_synd,
			ci->swid, ci->pkt_type);
#endif

	dispatch_pkt((struct sx_dev *)context, ci, hw_synd, 1);

out:
	kfree(ci);
out_free_skb:
	sx_skb_free(skb);

	return err;
}
EXPORT_SYMBOL(rx_skb);

static void *sx_get_cqe_from_buf(struct sx_buf *buf, int n)
{
	int offset = n * sizeof(struct sx_cqe);

	if (buf->nbufs == 1)
		return buf->u.direct.buf + offset;
	else
		return buf->u.page_list[offset >> PAGE_SHIFT].buf +
			(offset & (PAGE_SIZE - 1));
}

static void *sx_get_cqe(struct sx_cq *cq, int n)
{
	return sx_get_cqe_from_buf(&cq->buf, n);
}

static void *sx_get_sw_cqe(struct sx_cq *cq, int n)
{
	struct sx_cqe *cqe = sx_get_cqe(cq, n & (cq->nent - 1));

	return (cqe->e_sr_dqn_owner & 0x1) ^ !!(n & cq->nent) ? NULL : cqe;
}

static struct sx_cqe *sx_next_cqe_sw(struct sx_cq *cq)
{
	return sx_get_sw_cqe(cq, cq->cons_index);
}

void wqe_sync_for_cpu(struct sx_dq *dq, int idx)
{
	int dir = dq->is_send ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

	pci_dma_sync_single_for_cpu(dq->dev->pdev,
			dq->sge[idx].hdr_pld_sg.dma_addr,
			dq->sge[idx].hdr_pld_sg.len, dir);
	pci_unmap_single(dq->dev->pdev,
			dq->sge[idx].hdr_pld_sg.dma_addr,
			dq->sge[idx].hdr_pld_sg.len, dir);
	dq->sge[idx].hdr_pld_sg.vaddr = NULL;
	dq->sge[idx].hdr_pld_sg.len = 0;
	if (dq->is_send) {
		if (dq->sge[idx].pld_sg_1.len) {
			pci_dma_sync_single_for_cpu(dq->dev->pdev,
					dq->sge[idx].pld_sg_1.dma_addr,
					dq->sge[idx].pld_sg_1.len, dir);
			pci_unmap_single(dq->dev->pdev,
					dq->sge[idx].pld_sg_1.dma_addr,
					dq->sge[idx].pld_sg_1.len, dir);
			dq->sge[idx].pld_sg_1.vaddr = NULL;
			dq->sge[idx].pld_sg_1.len = 0;
		}

		if (dq->sge[idx].pld_sg_2.len) {
			pci_dma_sync_single_for_cpu(dq->dev->pdev,
					dq->sge[idx].pld_sg_2.dma_addr,
					dq->sge[idx].pld_sg_2.len, dir);
			pci_unmap_single(dq->dev->pdev,
					dq->sge[idx].pld_sg_2.dma_addr,
					dq->sge[idx].pld_sg_2.len, dir);
			dq->sge[idx].pld_sg_2.vaddr = NULL;
			dq->sge[idx].pld_sg_2.len = 0;
		}
	}
}

static int post_skb(struct sx_dq *dq)
{
	u16 size = dq->dev->profile.rdq_properties[dq->dqn].entry_size;
	int err = 0;
	struct sk_buff *new_skb;

	new_skb = alloc_skb(size, GFP_ATOMIC);
	if (!new_skb) {
		err = -ENOMEM;
		goto out;
	}

	if (skb_put(new_skb, size) == NULL) {
		err = -ENOMEM;
		goto out;
	}

	sx_core_post_recv(dq, new_skb);

out:
	return err;
}


static int sx_poll_one(struct sx_cq *cq)
{
	struct sx_cqe *cqe;
	int dqn;
	int is_send;
	int is_err;
	struct sx_dq *dq;
	int err = 0;
	struct sx_priv *priv;
	u16 wqe_ctr;
	u16 idx;
	unsigned long flags;

	spin_lock_irqsave(&cq->lock, flags);
	cqe = sx_next_cqe_sw(cq);
	if (!cqe) {
		spin_unlock_irqrestore(&cq->lock, flags);
		return -EAGAIN;
	}

	++cq->cons_index;
	spin_unlock_irqrestore(&cq->lock, flags);

	/*
	 * Make sure we read CQ entry contents after we've checked the
	 * ownership bit.
	 */
	rmb();

	dqn = (cqe->e_sr_dqn_owner >> 1) & SX_CQE_DQN_MASK;
	if (be16_to_cpu(cqe->dqn5_byte_count) & SX_CQE_DQN_MSB_MASK)
		dqn |= (1 << SX_CQE_DQN_MSB_SHIFT);

	is_err = !!(cqe->e_sr_dqn_owner & SX_CQE_IS_ERR_MASK);
	is_send  = !!(cqe->e_sr_dqn_owner & SX_CQE_IS_SEND_MASK);
	priv  = sx_priv(cq->sx_dev);
	if (is_send) {
	    if (dqn >= NUMBER_OF_SDQS) {
            sx_warn(cq->sx_dev, "dqn %d is larger than max SDQ %d.\n",
                    dqn, NUMBER_OF_SDQS);
            return 0;
	    }
	}
	else {
        if (dqn >= NUMBER_OF_RDQS) {
            sx_warn(cq->sx_dev, "dqn %d is larger than max RDQ %d.\n",
                    dqn, NUMBER_OF_RDQS);
            return 0;
        }
	}
	dq = is_send ? priv->sdq_table.dq[dqn] : priv->rdq_table.dq[dqn];

#ifdef SX_DEBUG
	printk(KERN_DEBUG PFX "sx_poll_one: dqn = %d, is_err = %d, "
			"is_send = %d\n", dqn, is_err, is_send);
#endif

	if (!dq) {
		if (printk_ratelimit()) {
			sx_warn(cq->sx_dev, "could not find dq context for %s "
					"dqn = %d\n",
					is_send ? "send" : "recv", dqn);
		}

		return 0;
	}

	wqe_ctr = be16_to_cpu(cqe->wqe_counter) & (dq->wqe_cnt - 1);
	if (is_err && !dq->is_flushing) {
		sx_warn(cq->sx_dev, "got %s completion with error, "
			"syndrom=0x%x, vendor=0x%x\n",
			is_send ? "send" : "recv", cqe->trap_id & 0xFF,
			(cqe->trap_id >> 8) & 0xFF);

		if (!is_send) {
			idx = dq->tail++ & (dq->wqe_cnt - 1);
			wqe_sync_for_cpu(dq, idx);
			sx_skb_free(dq->sge[idx].skb);
			dq->sge[idx].skb = NULL;
		}

		goto skip;
	}

	if (is_send) {
		/* find the wqe and unmap the DMA buffer.
		 * sx_skb_free to call  the destructor which will signal
		 * the write-semaphore for ETH packets, or call gen_completion
		 * for IB packets. */
		do {
			idx = dq->tail++ & (dq->wqe_cnt - 1);
			wqe_sync_for_cpu(dq, idx);
			sx_skb_free(dq->sge[idx].skb);
			dq->sge[idx].skb = NULL;
		} while (idx != wqe_ctr);

		spin_lock_irqsave(&dq->lock, flags);
		sx_add_pkts_to_sdq(dq);
		spin_unlock_irqrestore(&dq->lock, flags);
	} else {
		/* get the skb from the right rdq entry, unmap the buffers
		 * and call rx_skb with the cqe and skb */
		/* this while is temporary, we need it because FW doesn't
		 * send us error CQes for wqe too short. */
		idx = dq->tail & (dq->wqe_cnt - 1);
		while (idx != wqe_ctr) {
			if (printk_ratelimit())
				printk(KERN_DEBUG PFX "sx_poll_one: Err "
						"wqe_ctr=[%u] "
						"!= dq->tail=[%u]\n",
						wqe_ctr, dq->tail);
/*			idx = dq->tail & (dq->wqe_cnt - 1); */
			wqe_sync_for_cpu(dq, idx);
			sx_skb_free(dq->sge[idx].skb);
			dq->sge[idx].skb = NULL;
			dq->tail++;
			idx = dq->tail & (dq->wqe_cnt - 1);
			post_skb(dq);
		}

		++dq->tail;
		wqe_sync_for_cpu(dq, idx);
#ifdef SX_DEBUG
		printk(KERN_DEBUG PFX "sx_poll_one: This is a RDQ, idx = %d, "
				"wqe_ctr = %d, dq->tail = %d. "
				"Calling rx_skb\n",
			idx, wqe_ctr, dq->tail - 1);
#endif
		if (!is_err)
			rx_skb(cq->sx_dev, dq->sge[idx].skb, cqe);
		else
			sx_skb_free(dq->sge[idx].skb);

		dq->sge[idx].skb = NULL;
	}
skip:
	sx_cq_set_ci(cq);
	if (!is_send && !dq->is_flushing)
		err = post_skb(dq);

	if (is_send)
		wake_up_interruptible(&dq->tx_full_wait);

	return err;
}

/* return errno on error, otherwise num of handled cqes */
int sx_cq_completion(struct sx_dev *dev, u32 cqn, u16 weight)
{
	struct sx_cq *cq;
	struct cq_rate_limiter_params *rl_params = &sx_priv(dev)->cq_table.cq_rl_params[cqn];
	unsigned long flags;
	int num_of_cqes = 0;
	int err = 0;

	spin_lock_irqsave(&sx_priv(dev)->cq_table.lock, flags);
	cq = (sx_priv(dev)->cq_table.cq[cqn]);
	spin_unlock_irqrestore(&sx_priv(dev)->cq_table.lock, flags);
	if (!cq) {
		if (printk_ratelimit()) {
			sx_warn(dev, "Completion event for bogus CQ %08x\n", cqn);
		}

		return -EAGAIN;
	}

	do {
		if (rl_params->use_limiter && rl_params->curr_cq_credit == 0) {
			/* Print a message on the first time we drop a packet on each RDQ */
			if (!rl_params->num_cq_stops) {
				sx_warn(dev, "RDQ rate limiter was activated for RDQ %u\n",
						cqn - NUMBER_OF_SDQS);
			}

			rl_params->num_cq_stops++;
			goto out;
		}

		err = sx_poll_one(cq);
		/* -EAGAIN is the only error where the consumer index is not increased */
		if (rl_params->use_limiter && err != -EAGAIN) {
			rl_params->curr_cq_credit--;
		} else {
	        atomic_inc(&cq->bkp_poll_data.curr_num_cq_polls);
		}
	} while (!err && ++num_of_cqes < weight);

	if (num_of_cqes < weight) {
	    if (rl_params->use_limiter && (rl_params->curr_cq_credit == 0)) {
	        sx_info(dev,"CQ:%d tried to ARM from tasklet with zero credits\n",cqn);
	        goto out;
	    }
		sx_cq_arm(cq);
	}

out:
	if (!err || err == -EAGAIN)
		return num_of_cqes;

	return err;
}


static int sx_SW2HW_CQ(struct sx_dev *dev, struct sx_cmd_mailbox *mailbox,
			 int cq_num)
{
	return sx_cmd(dev, dev->device_id, mailbox, cq_num, 0, SX_CMD_SW2HW_CQ,
			SX_CMD_TIME_CLASS_A, sx_priv(dev)->fw.local_in_mb_size);
}

static int sx_HW2SW_CQ(struct sx_dev *dev, int cq_num)
{
	return sx_cmd_box(dev, dev->device_id, 0, 0, cq_num, 0,
			SX_CMD_HW2SW_CQ, SX_CMD_TIME_CLASS_A,
			sx_priv(dev)->fw.local_in_mb_size);
}

int sx_core_create_cq(struct sx_dev *dev, int nent, struct sx_cq **cq, u8 cqn)
{
	struct sx_priv *priv = sx_priv(dev);
	struct sx_cq_table *cq_table = &priv->cq_table;
	struct sx_cmd_mailbox *mailbox;
	struct sx_cq_context *cq_context;
	int err;
	unsigned long flags;
	struct sx_cq *tcq;

	if (nent <= 0 || dev == NULL)
		return -EINVAL;

	tcq = kzalloc(sizeof *tcq, GFP_KERNEL);
	if (!tcq)
		return -ENOMEM;

	tcq->cqn = sx_bitmap_set(&cq_table->bitmap, cqn);
	if (tcq->cqn == -1) {
		err = -ENOMEM;
		goto out_free_cq;
	}

	spin_lock_irqsave(&cq_table->lock, flags);
	cq_table->cq[tcq->cqn] = tcq;
	spin_unlock_irqrestore(&cq_table->lock, flags);
	mailbox = sx_alloc_cmd_mailbox(dev, dev->device_id);
	if (IS_ERR(mailbox)) {
		err = PTR_ERR(mailbox);
		goto free_from_cq_table;
	}

	cq_context = mailbox->buf;
	memset(cq_context, 0, sizeof *cq_context);
	err = sx_buf_alloc(dev, nent * sizeof(struct sx_cqe), PAGE_SIZE,
			   &tcq->buf, 1);
	if (err)
		goto err_mbox;

	tcq->nent = nent;
	cq_context->eq_num = priv->eq_table.eq[SX_EQ_COMP].eqn;
	cq_context->log_cq_size = (u8)ilog2(nent);
	if (dev->pdev)
		sx_fill_page_list(cq_context->dma_addr, &tcq->buf);

	err = sx_SW2HW_CQ(dev, mailbox, tcq->cqn);
	if (err)
		goto err_buf;

	sx_free_cmd_mailbox(dev, mailbox);
	tcq->set_ci_db  = dev->db_base + SX_DBELL_CQ_CI_OFFSET + 4 * tcq->cqn;
	tcq->arm_db     = dev->db_base + SX_DBELL_CQ_ARM_OFFSET + 4 * tcq->cqn;
	tcq->sx_dev = dev;
	tcq->cons_index = 0;
	atomic_set(&tcq->refcount, 1);
    atomic_set(&tcq->bkp_poll_data.curr_num_cq_polls,0);
    atomic_set(&tcq->bkp_poll_data.cq_bkp_poll_mode,0);
	init_completion(&tcq->free);
	spin_lock_init(&tcq->lock);
	spin_lock_init(&tcq->rearm_lock);
	sx_cq_set_ci(tcq);
	sx_cq_arm(tcq);
	*cq = tcq;

	return 0;

err_buf:
	sx_buf_free(dev, tcq->nent * sizeof(struct sx_cqe), &tcq->buf);

err_mbox:
	sx_free_cmd_mailbox(dev, mailbox);

free_from_cq_table:
	spin_lock_irqsave(&cq_table->lock, flags);
	cq_table->cq[tcq->cqn] = NULL;
	spin_unlock_irqrestore(&cq_table->lock, flags);
	sx_bitmap_free(&cq_table->bitmap, tcq->cqn);

out_free_cq:
	kfree(tcq);

	return err;
}


void sx_core_destroy_cq(struct sx_dev *dev, struct sx_cq *cq)
{
	struct sx_priv *priv = sx_priv(dev);
	struct sx_cq_table *cq_table = &priv->cq_table;
	unsigned long flags;
	int err;

	if (!cq) {
		sx_err(dev, "sx_core_destroy_cq  : cq(%p) == NULL\n", cq);
		return;
	}

	spin_lock_irqsave(&cq_table->lock, flags);
	cq_table->cq[cq->cqn] = NULL;
	spin_unlock_irqrestore(&cq_table->lock, flags);
	err = sx_HW2SW_CQ(dev, cq->cqn);
	if (err) {
		sx_warn(dev, "HW2SW_CQ failed (%d) "
				"for CQN %06x\n", err, cq->cqn);
	}

	synchronize_irq(priv->eq_table.eq[SX_EQ_COMP].irq);
	if (atomic_dec_and_test(&cq->refcount))
		complete(&cq->free);
	wait_for_completion(&cq->free);

	sx_bitmap_free(&cq_table->bitmap, cq->cqn);
	sx_buf_free(dev, cq->nent * sizeof(struct sx_cqe), &cq->buf);
	kfree(cq);
}

int sx_core_init_cq_table(struct sx_dev *dev)
{
	struct sx_cq_table *cq_table = &sx_priv(dev)->cq_table;
	struct sx_cq      **cq_array = NULL;
	int		    err	     = 0;
	int		    i	     = 0;
	unsigned long flags;

	spin_lock_init(&cq_table->lock);

	cq_array = kmalloc(dev->dev_cap.max_num_cqs * sizeof(*cq_array),
			   GFP_KERNEL);
	if (!cq_array)
		return -ENOMEM;

	for (i = 0; i < dev->dev_cap.max_num_cqs; i++)
		cq_array[i] = NULL;

	spin_lock_irqsave(&cq_table->lock, flags);
	cq_table->cq = cq_array;
	spin_unlock_irqrestore(&cq_table->lock, flags);
	err = sx_bitmap_init(&cq_table->bitmap, dev->dev_cap.max_num_cqs);
	if (err)
		return err;

	cq_table->cq_credit_thread = NULL;
	cq_table->credit_thread_active = 0;
	cq_table->rl_time_interval = 50; /* This is the default value */
	cq_table->cq_rl_params = kzalloc(dev->dev_cap.max_num_cqs * sizeof(*cq_table->cq_rl_params),
			   GFP_KERNEL);
	if (!cq_table->cq_rl_params) {
		printk(KERN_ERR PFX "Not enough memory for the rate limiter parameters\n");
		return -ENOMEM;
	}

	return 0;
}

void sx_core_destroy_cq_table(struct sx_dev *dev)
{
	struct sx_cq_table *cq_table = &sx_priv(dev)->cq_table;
	unsigned long flags;
	int i;

	if (cq_table->cq_credit_thread) {
		kthread_stop(cq_table->cq_credit_thread);
		cq_table->cq_credit_thread = NULL;
		for (i = 0; i < dev->dev_cap.max_num_cqs; i++) {
			cq_table->cq_rl_params[i].use_limiter = 0;
		}
	}

	kfree(cq_table->cq_rl_params);
	cq_table->cq_rl_params = NULL;
	spin_lock_irqsave(&cq_table->lock, flags);
	kfree(cq_table->cq);
	cq_table->cq = NULL;
	spin_unlock_irqrestore(&cq_table->lock, flags);
}

void sx_core_dump_synd_tbl(struct sx_dev *dev)
{
	struct listener_entry *listener;
	struct list_head *pos;
	unsigned long flags;
	u16 entry = 0;

	spin_lock_irqsave(&sx_glb.listeners_lock, flags);
	for (entry = 0; entry < NUM_HW_SYNDROMES + 1; entry++) {
		if (!list_empty(&sx_glb.listeners_db[entry].list)) {
			list_for_each(pos, &sx_glb.listeners_db[entry].list) {
				listener = list_entry(pos,
						struct listener_entry, list);
				printk(KERN_DEBUG
					"=============================\n");
				printk(KERN_DEBUG
						"synd=%d, swid=%d, is_def:%d, "
						"handler:%p, rx_pkt:%llu \n",
						entry,
						listener->swid,
						listener->is_default,
						listener->handler,
						listener->rx_pkts);

				switch (listener->listener_type) {
				case L2_TYPE_DONT_CARE:
					printk(KERN_DEBUG "list_type: "
							"DONT_CARE, crit [port:0x%x] \n",
							listener->critireas.dont_care.sysport);
					break;
				case L2_TYPE_IB:
					printk(KERN_DEBUG "list_type: IB, crit"
						" [qpn:0x%x (%d)] \n",
						listener->critireas.ib.qpn,
						listener->critireas.ib.qpn);
					break;
				case L2_TYPE_ETH:
					printk(KERN_DEBUG "list_type: ETH, crit "
						"[ethtype:0x%x, dmac:%llx, "
						"emad_tid:0x%x, from_rp:%u, from_bridge:%u ] \n",
					listener->critireas.eth.ethtype,
					listener->critireas.eth.dmac,
					listener->critireas.eth.emad_tid,
					listener->critireas.eth.from_rp,
					listener->critireas.eth.from_bridge);
					break;
				case L2_TYPE_FC:
					printk(KERN_DEBUG "list_type: FC \n");
					break;
				default:
					printk(KERN_DEBUG "list_type: UNKNOWN \n");
					break;
				}
			}
		}
	}

	spin_unlock_irqrestore(&sx_glb.listeners_lock, flags);
}

int sx_cq_credit_thread_handler(void *cq_ctx)
{
	struct sx_priv *sx_priv = (struct sx_priv *)cq_ctx;
	struct sx_cq_table *cq_table = &sx_priv->cq_table;
	struct sx_dq_table *rdq_table = &sx_priv->rdq_table;
	int i;
	struct sx_cqe *cqe;

	while (!kthread_should_stop()) {
		unsigned int actual_sleep_time_msecs;
		init_completion(&cq_table->done);
		actual_sleep_time_msecs = jiffies_to_msecs(jiffies);

		/* Waiting for a completion that will never happen,
		 * just so we wake up after the credit interval */
		if (!wait_for_completion_timeout(&cq_table->done,
				msecs_to_jiffies(cq_table->rl_time_interval))) {
			actual_sleep_time_msecs = jiffies_to_msecs(jiffies) - actual_sleep_time_msecs;

			credit_thread_vals[arr_count % 1001] = jiffies_to_msecs(jiffies);
			for (i = NUMBER_OF_SDQS; i < sx_priv->dev.dev_cap.max_num_cqs; i++) {
				/* If the array is not allocated,
				 * it means that we are in deinit phase */
				if (!cq_table->cq_rl_params)
					break;

				if (sx_bitmap_test(&rdq_table->bitmap, (i - NUMBER_OF_SDQS)) &&
						cq_table->cq_rl_params[i].use_limiter) {
					struct cq_rate_limiter_params *rl_params =
							&cq_table->cq_rl_params[i];
					int prev_credit = rl_params->curr_cq_credit;
					int credits = rl_params->interval_credit;

					while (actual_sleep_time_msecs >= cq_table->rl_time_interval) {
						credits += (rl_params->interval_credit / 10) + 1;
						actual_sleep_time_msecs -= cq_table->rl_time_interval / 10;
					}

					rl_params->curr_cq_credit += credits;
					if (rl_params->curr_cq_credit > rl_params->max_cq_credit)
						rl_params->curr_cq_credit = rl_params->max_cq_credit;

					/* ARM the CQ if not armed */
					if (prev_credit < rl_params->interval_credit)
						sx_cq_arm(cq_table->cq[i]);

	                if (atomic_read(&cq_backup_polling_enabled)) {
	                    if ((debug_cq_backup_poll_cqn == i) ||
	                        ((prev_credit > rl_params->interval_credit) &&
                            (cq_table->cq[i]->bkp_poll_data.last_interval_num_cq_polls ==
                             atomic_read(&cq_table->cq[i]->bkp_poll_data.curr_num_cq_polls)) &&
                             (cq_table->cq[i]->bkp_poll_data.last_interval_cons_index
                                     == cq_table->cq[i]->cons_index))) {

                            cqe=sx_next_cqe_sw(cq_table->cq[i]);
                            if ((debug_cq_backup_poll_cqn == i) ||
                                 (cqe &&
                                 !atomic_read(&cq_table->cq[i]->bkp_poll_data.cq_bkp_poll_mode))) {
                                /* Trigger backup poll */
                                int dqn;
                                dqn = (cqe->e_sr_dqn_owner >> 1) & SX_CQE_DQN_MASK;
                                if (be16_to_cpu(cqe->dqn5_byte_count) & SX_CQE_DQN_MSB_MASK)
                                    dqn |= (1 << SX_CQE_DQN_MSB_SHIFT);

                                if (net_ratelimit()) {
                                    sx_info(cq_table->cq[i]->sx_dev,"triggering backup poll for CQ:%d "
                                            "RDQ:%d cons_index:%d\n",
                                        i,dqn,cq_table->cq[i]->cons_index);
                                }
                                debug_cq_backup_poll_cqn=CQN_INVALID;
                                atomic_set(&cq_table->cq[i]->bkp_poll_data.cq_bkp_poll_mode,1);
                                atomic_inc(&sx_priv->cq_backup_polling_refcnt);
                                tasklet_schedule(&sx_priv->intr_tasklet);
                            }
	                    }
	                }

	                cq_table->cq[i]->bkp_poll_data.last_interval_num_cq_polls=
	                        atomic_read(&cq_table->cq[i]->bkp_poll_data.curr_num_cq_polls);
	                cq_table->cq[i]->bkp_poll_data.last_interval_cons_index = cq_table->cq[i]->cons_index;
				}
			}

			arr_count++;
		}
	}

	return 0;
}


void sx_cq_show_cq(struct sx_dev *dev, int cqn)
{
    struct sx_priv *priv = sx_priv(dev);
    struct sx_cq_table * __maybe_unused cq_table=&priv->cq_table;
    struct sx_cq *cq = priv->cq_table.cq[cqn];
    int iii,jjj;
    struct sx_cqe *cqe;
    u8 cqe_owner[16];

    if (!cq) {
        printk("cq %d doesn't exist \n", cqn);
        return;
    }

    printk("[cq %d]: cqn:%d, cons_index:%d, nent:%d cons_index&(nent-1):%d"
            " ref_cnt:%d \n",
            cqn,
            cq->cqn,
            cq->cons_index,
            cq->nent,
            (cq->cons_index & (cq->nent - 1)),
            atomic_read(&cq->refcount)
            );

    printk("CQ %d owner:\n", cqn);
    for (iii = 0; iii < cq->nent / 16; iii++) {
        for (jjj=0; (jjj<16) && ((iii*16+jjj) < cq->nent) ; jjj++) {
            cqe = sx_get_cqe(cq, (iii*16+jjj) & (cq->nent - 1));
            if (cqe) {
                cqe_owner[jjj]=cqe->e_sr_dqn_owner;
            } else {
                cqe_owner[jjj]=255;
            }
        }

        printk("[%5.5d]: %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x "
                        "%2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n",
               iii*16,
               cqe_owner[0],cqe_owner[1],cqe_owner[2],cqe_owner[3],
               cqe_owner[4],cqe_owner[5],cqe_owner[6],cqe_owner[7],
               cqe_owner[8],cqe_owner[9],cqe_owner[10],cqe_owner[11],
               cqe_owner[12],cqe_owner[13],cqe_owner[14],cqe_owner[15]);
    }
    printk("\n");
}

void sx_cq_flush_rdq(struct sx_dev *dev, int dqn)
{
    int iii;
    int idx;
    struct sx_priv *priv = sx_priv(dev);
    struct sx_dq_table *rdq_table = &priv->rdq_table;
    struct sx_dq *dq = rdq_table->dq[dqn];

    idx = dq->tail & (dq->wqe_cnt - 1);
    for (iii=0; iii<dq->wqe_cnt; iii++) {
        printk("%s:%d flushing rdq %d sge %d \n",__func__,__LINE__,dqn,idx);
        wqe_sync_for_cpu(dq, idx);
        sx_skb_free(dq->sge[idx].skb);
        dq->sge[idx].skb = NULL;
        dq->tail++;
        idx = dq->tail & (dq->wqe_cnt - 1);
        post_skb(dq);
    }

}

/************************************************
 *                  EOF                         *
 ***********************************************/
