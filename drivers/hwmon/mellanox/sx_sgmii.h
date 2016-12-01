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

#ifndef SX_SGMII_H
#define SX_SGMII_H

/************************************************
 * Includes
 ***********************************************/

#include <linux/types.h>

/************************************************
 *  Enums
 ***********************************************/

enum sx_sgmii_pkt_type_t {
	SX_SGMII_PKT_TYPE_IB_RAW	= 0x00,
	SX_SGMII_PKT_TYPE_IB_TRANSPORT	= 0x01,
	SX_SGMII_PKT_TYPE_ETH		= 0x02
};

enum sgmii_oob_ctrl_hdr_lp_t {
	SX_SGMII_OOB_PKT_CTRL_HDR_LP_FWD_TO_SWITCH 	= 0,
	SX_SGMII_OOB_PKT_CTRL_HDR_LP_CR_SPACE 		= 2
};

/************************************************
 * Structures
 ***********************************************/
struct sx_sgmii_ifc {
	u8		initialized;
	u64		base_smac;
	u8		number_of_macs; /* number of MACs is the actual number of flows */
	int		(*init)(void);
	void	(*deinit)(void);
	int		(*send)(struct sk_buff *skb);
	int		(*build_encapsulation_header)(struct sk_buff *skb, u64 dmac,
			u64 smac, u16 ethertype, u8 proto, u8 ver);
	int		(*build_ctrl_segment)(struct sk_buff *skb, u8 lp, u8 sdqn, 
			enum sx_sgmii_pkt_type_t pkt_type);
	u32		(*cr_space_readl)(int dev_id, u32 address, int *err);
	int		(*cr_space_writel)(int dev_id, u32 address, u32 value);
	int		(*cr_space_read_buf)(int dev_id, u32 address,
			unsigned char *buf, int size);
	int		(*cr_space_write_buf)(int dev_id, u32 address,
			unsigned char *buf, int size);
};


/************************************************
 * Functions
 ***********************************************/
#ifdef CONFIG_SX_SGMII_PRESENT
void sx_sgmii_init_cb(struct sx_sgmii_ifc *sgmii_ifc);
#else
static inline void sx_sgmii_init_cb(struct sx_sgmii_ifc *sgmii_ifc)
{
	sgmii_ifc->init = NULL;
	sgmii_ifc->deinit = NULL;
	sgmii_ifc->send = NULL;
	sgmii_ifc->build_encapsulation_header = NULL;
	sgmii_ifc->build_ctrl_segment = NULL;
	sgmii_ifc->cr_space_readl = NULL;
	sgmii_ifc->cr_space_writel = NULL;
	sgmii_ifc->cr_space_read_buf = NULL;
	sgmii_ifc->cr_space_write_buf = NULL;
}
#endif

int sx_sgmii_cr_space_write_buf(int dev_id, u32 address, unsigned char *buf, int size);
int sx_sgmii_cr_space_read_buf(int dev_id, u32 address, unsigned char *buf, int size);

int sx_sgmii_build_cr_space_header_switchx(struct sk_buff *skb, u8 token, u8 rw, u32 address, u8 size);
int sx_sgmii_build_cr_space_header_spectrum(struct sk_buff *skb, u8 token, u8 rw, u32 address, u8 size);

#endif /* SX_SGMII_H */
