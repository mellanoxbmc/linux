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

#ifndef SX_DRIVER_H
#define SX_DRIVER_H

#include <linux/device.h>
#include <linux/mlx_sx/device.h>
#include <linux/mlx_sx/kernel_user.h>

struct sx_dev;

enum sx_dev_event {
	SX_DEV_EVENT_CATASTROPHIC_ERROR,
	SX_DEV_EVENT_IB_SWID_UP,
	SX_DEV_EVENT_ETH_SWID_UP,
	SX_DEV_EVENT_IB_SWID_DOWN,
	SX_DEV_EVENT_ETH_SWID_DOWN,
	SX_DEV_EVENT_OPEN_PORT_NETDEV,
	SX_DEV_EVENT_CLOSE_PORT_NETDEV,
	SX_DEV_EVENT_PORT_UP,
	SX_DEV_EVENT_PORT_DOWN,
	SX_DEV_EVENT_PORT_REINIT,
	SX_DEV_EVENT_TYPE_INTERNAL_ERROR,
	SX_DEV_EVENT_TYPE_TCA_INIT,
	SX_DEV_EVENT_MAD_IFC_ENABLE,
	SX_DEV_EVENT_ADD_SYND_NETDEV,
	SX_DEV_EVENT_REMOVE_SYND_NETDEV,
	SX_DEV_EVENT_ADD_SYND_IPOIB,
	SX_DEV_EVENT_REMOVE_SYND_IPOIB,
	SX_DEV_EVENT_DEBUG_NETDEV,
	SX_DEV_EVENT_NODE_DESC_UPDATE,
	SX_DEV_EVENT_ADD_SYND_L2_NETDEV,
	SX_DEV_EVENT_REMOVE_SYND_L2_NETDEV
};

#define SX_PAGE_SIZE		4096
#define SX_PAGE_SHIFT		12

#define ETHTYPE_ARP		0x0806
#define ETHTYPE_VLAN		0x8100
#define ETHTYPE_EMAD		0x8932
#define ETHTYPE_DONT_CARE_VALUE 0
#define QPN_DONT_CARE_VALUE 	0xffffffff
#define QPN_MULTICAST_VALUE 	0xffffff
#define DMAC_DONT_CARE_VALUE 	0
#define TID_DONT_CARE_VALUE 	0
#define SYSPORT_DONT_CARE_VALUE 0
#define FWD_BY_FDB_TRAP_ID	0x01
#define SWITCHIB_QP0_TRAP_ID		0xf0
#define SWITCHIB_QP1_TRAP_ID		0xf1
#define SWITCHIB_OTHER_QP_TRAP_ID	0xf2
#define PACKET_SAMPLE_TRAP_ID	0x38
#define ROUTER_QP0_TRAP_ID	0x5e
#define FDB_TRAP_ID		0x06
#define ARP_REQUEST_TRAP_ID	0x50
#define ARP_RESPONSE_TRAP_ID	0x51
#define ETH_L3_MTUERROR_TRAP_ID	0x52
#define ETH_L3_TTLERROR_TRAP_ID	0x53
#define ETH_L3_LBERROR_TRAP_ID  0x54
#define MIN_IPTRAP_TRAP_ID	0x1C0 /* TODO define which one will be used */

union sx_event_data {
	struct {
		int swid;
		u16 dev_id;
	} ib_swid_change;
	struct {
		int swid;
		int synd;
		u64 mac;
	} eth_swid_up;
	struct {
		int swid;
		int hw_synd;
	} eth_l3_synd;
	struct {
		int swid;
		int hw_synd;
	} ipoib_synd;
	struct {
		int swid;
	} eth_swid_down;
	struct {
		int swid;
		u16 sysport;
		u8 	is_lag;
		u16 mid;
		char *name;
		u8  send_to_rp_as_data_supported;
	} port_netdev_set;
	struct {
		int num_of_ib_swids;
		u8  swid[NUMBER_OF_SWIDS];
		u16 max_pkey;
	} tca_init;
	struct {
		uint8_t swid;
		uint8_t NodeDescription[64];
	} node_desc_update;
};

struct sx_interface {
	void *			(*add)	 (struct sx_dev *dev);
	void			(*remove)(struct sx_dev *dev, void *context);
	void			(*event) (struct sx_dev *dev, void *context,
					enum sx_dev_event event,
					union sx_event_data *event_data);
	struct list_head	list;
};

struct sx_sgmii_ctrl_segment {
	u8	reserved1;
	u8	one;
	__be16	type_sdq_lp;
	__be32 reserved2[3];
} __attribute__((packed));

struct sx_ethernet_header {
	uint8_t dmac[6];
	uint8_t smac[6];
	__be16 et;
	uint8_t mlx_proto;
	uint8_t ver;
};

typedef enum check_dup{
    CHECK_DUP_DISABLED_E = 0,
    CHECK_DUP_ENABLED_E = 1
}check_dup_e;

typedef enum is_rp {
    IS_RP_DONT_CARE_E = 0,
    IS_RP_FROM_RP_E = 1,
    IS_RP_NOT_FROM_RP_E = 2,
} is_rp_e;

typedef enum is_bridge {
    IS_BRIDGE_DONT_CARE_E = 0,
    IS_BRIDGE_FROM_BRIDGE_E = 1,
    IS_BRIDGE_NOT_FROM_BRIDGE_E = 2,
} is_bridge_e;

int sx_core_flush_synd_by_context(void * context);
int sx_core_flush_synd_by_handler(cq_handler handler);
int sx_register_interface(struct sx_interface *intf);
void sx_unregister_interface(struct sx_interface *intf);
int sx_core_add_synd(u8 swid, u16 hw_synd, enum l2_type type, u8 is_default,
	union ku_filter_critireas crit, cq_handler handler, void *context, 
	check_dup_e check_dup, struct sx_dev* sx_dev);
int sx_core_remove_synd(u8 swid, u16 hw_synd, enum l2_type type, u8 is_default,
		union ku_filter_critireas critireas, 
		void *context, struct sx_dev* sx_dev);
int sx_core_post_send(struct sx_dev *dev, struct sk_buff *skb,
			struct isx_meta *meta);
int __sx_core_post_send(struct sx_dev *dev, struct sk_buff *skb,
			struct isx_meta *meta);
void sx_skb_free(struct sk_buff *skb);

int sx_core_get_prio2tc(struct sx_dev *dev,
               uint16_t port_lag_id, uint8_t is_lag,
               uint8_t pcp, uint8_t *tc);
int sx_core_get_pvid(struct sx_dev *dev,
                     uint16_t       port_lag_id,
                     uint8_t        is_lag,
                     uint16_t       *pvid);
int sx_core_get_vlan_tagging(struct sx_dev *dev,
               uint16_t port_lag_id, uint8_t is_lag,
               uint16_t vlan, uint8_t *is_vlan_tagged);
int sx_core_get_prio_tagging(struct sx_dev *dev,
               uint16_t port_lag_id, uint8_t is_lag,
               uint8_t *is_port_prio_tagged);
int sx_core_get_rp_vlan(struct sx_dev *dev,
                        struct completion_info *comp_info,
                        uint16_t *vlan_id);
int sx_core_get_swid(struct sx_dev *dev,
                     struct completion_info *comp_info,
                     uint8_t *swid);
int sx_core_get_vlan2ip(struct sx_dev *dev,
               uint16_t vid, uint32_t *ip_addr);
int sx_core_get_rp_rif_id(struct sx_dev *dev, uint16_t port_lag_id,
                          uint8_t is_lag, uint16_t vlan_id, uint16_t *rif_id);
int sx_core_get_rp_mode(struct sx_dev *dev, u8 is_lag, u16 sysport_lag_id,
                        u16 vlan_id, u8 *is_rp);

int sx_core_get_fid_by_port_vid(struct sx_dev *dev, 
                            struct completion_info *comp_info, uint16_t *fid);
int sx_core_get_lag_mid(struct sx_dev *dev, u16 lag_id, u16 *mid);

#endif /* SX_DRIVER_H */
