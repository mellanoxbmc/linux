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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/uio.h>
#include <linux/skbuff.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/kthread.h>
#include <linux/vmalloc.h>
#include <linux/mlx_sx/kernel_user.h>
#include <linux/mlx_sx/device.h>
#include <linux/mlx_sx/cmd.h>
#include "sx.h"
#include "cq.h"
#include "dq.h"
#include "eq.h"
#include "alloc.h"
#include "icm.h"
#include "fw.h"
#include "sx_dpt.h"
#include "sx_proc.h"

#ifdef CONFIG_44x
#include <asm/dcr.h>
#include <asm/dcr-regs.h>
#include <asm/reg.h>
#endif

/************************************************
 *  Global
 ***********************************************/

/************************************************
 *  Define
 ***********************************************/

#ifndef BUILD_VERSION
#define BUILD_VERSION ""
#endif

#define SX_CORE_CHAR_DEVICE_NAME "sxcdev"
#define SX_CORE_DRV_VERSION      "1.00 " BUILD_VERSION
static const char sx_version[] =
    DRV_NAME ": Mellanox SwitchX Core Driver "
    SX_CORE_DRV_VERSION " (" DRV_RELDATE ")\n";

#define RDQ_NUMBER_OF_ENTRIES 128
/************************************************
 *  Enum
 ***********************************************/

enum SX_CHAR_DEVICE {
    SX_MAJOR = 231,
    SX_BASE_MINOR = 193,
};
dev_t             char_dev;
struct sx_globals sx_glb;
//static int first_ib_swid = 1;

/************************************************
 *  MODULE settings
 ***********************************************/
MODULE_AUTHOR("Amos Hersch, Anatoly Lisenko");
MODULE_DESCRIPTION("Mellanox SwitchX driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(SX_CORE_DRV_VERSION);

int cq_thread_sched_priority = 0;
module_param_named(cq_thread_sched_priority, cq_thread_sched_priority, int, 0644);
MODULE_PARM_DESC(cq_thread_sched_priority, "CQ credit thread real time priority");

int rx_debug_sgmii;
module_param_named(rx_debug_sgmii,
                   rx_debug_sgmii, int, 0644);
MODULE_PARM_DESC(rx_debug_sgmii, "en/dis dump of SGMII pkts");

int rx_debug;
module_param_named(rx_debug,
                   rx_debug, int, 0644);
MODULE_PARM_DESC(rx_debug, "en/dis dump of pkts");

int rx_debug_pkt_type = SX_DBG_PACKET_TYPE_ANY;
module_param_named(rx_debug_pkt_type,
                   rx_debug_pkt_type, int, 0644);
MODULE_PARM_DESC(rx_debug_pkt_type, "trap/synd number to dump, 0xff dump all pkts");

int rx_debug_emad_type = SX_DBG_EMAD_TYPE_ANY;
module_param_named(rx_debug_emad_type,
                   rx_debug_emad_type, int, 0644);
MODULE_PARM_DESC(rx_debug_emad_type, "emad number to dump, 0xffff dump all pkts");

int tx_debug_sgmii;
module_param_named(tx_debug_sgmii,
                   tx_debug_sgmii, int, 0644);
MODULE_PARM_DESC(tx_debug_sgmii, "en/dis dump of SGMII pkts");

char *sgmii_dev_name = "mlx4_1";
module_param(sgmii_dev_name, charp, 0000);
MODULE_PARM_DESC(sgmii_dev_name, "sgmii device name (string)");

int sgmii_port_number = -1;
module_param_named(sgmii_port_number,
		sgmii_port_number, int, 0644);
MODULE_PARM_DESC(sgmii_port_number, "out port of sgmii interface");

int tx_debug;
module_param_named(tx_debug,
                   tx_debug, int, 0644);
MODULE_PARM_DESC(tx_debug, "en/dis dump of pkts");

int tx_debug_pkt_type = SX_DBG_PACKET_TYPE_ANY;
module_param_named(tx_debug_pkt_type,
                   tx_debug_pkt_type, int, 0644);
MODULE_PARM_DESC(tx_debug_pkt_type, "trap/synd number to dump, 0xff dump all pkts");

int tx_debug_emad_type = SX_DBG_EMAD_TYPE_ANY;
module_param_named(tx_debug_emad_type,
                   tx_debug_emad_type, int, 0644);
MODULE_PARM_DESC(tx_debug_emad_type, "emad type to dump, 0xff dump all emads");

int rx_dump_sgmii;
module_param_named(rx_dump_sgmii,
                   rx_dump_sgmii, int, 0644);
MODULE_PARM_DESC(rx_dump_sgmii, " 0- don't dump, 1-dump the SGMII RX packet data");

int rx_dump;
module_param_named(rx_dump,
                   rx_dump, int, 0644);
MODULE_PARM_DESC(rx_dump, " 0- don't dump, 1-dump the RX packet data");

int rx_dump_cnt = SX_DBG_COUNT_UNLIMITED;
module_param_named(rx_dump_cnt,
                   rx_dump_cnt, int, 0644);
MODULE_PARM_DESC(rx_dump_cnt, " 0xFFFF - unlimited, dump CNT packets  only");

int tx_dump_sgmii;
module_param_named(tx_dump_sgmii,
                   tx_dump_sgmii, int, 0644);
MODULE_PARM_DESC(tx_dump_sgmii, " 0- don't dump, 1-dump the SGMII TX packet data");

int tx_dump;
module_param_named(tx_dump,
                   tx_dump, int, 0644);
MODULE_PARM_DESC(tx_dump, " 0- don't dump, 1-dump the TX packet data");

int tx_dump_cnt = SX_DBG_COUNT_UNLIMITED;
module_param_named(tx_dump_cnt,
                   tx_dump_cnt, int, 0644);
MODULE_PARM_DESC(tx_dump_cnt, " 0xFFFF - unlimited, dump CNT packets  only");

int i2c_cmd_dump;
module_param_named(i2c_cmd_dump,
                   i2c_cmd_dump, int, 0644);
MODULE_PARM_DESC(i2c_cmd_dump, " 0- don't dump, 1-dump i2c data");

int i2c_cmd_op = SX_DBG_CMD_OP_TYPE_ANY;
module_param_named(i2c_cmd_op,
                   i2c_cmd_op, int, 0644);
MODULE_PARM_DESC(i2c_cmd_op, " cmd op to dump, 0xFFFF - dump all cmds");

int i2c_cmd_reg_id = SX_DBG_REG_TYPE_ANY;
module_param_named(i2c_cmd_reg_id,
                   i2c_cmd_reg_id, int, 0644);
MODULE_PARM_DESC(i2c_cmd_reg_id, " cmd reg_id to dump, 0xFFFF - dump all cmds");

int i2c_cmd_dump_cnt = SX_DBG_COUNT_UNLIMITED;
module_param_named(i2c_cmd_dump_cnt,
                   i2c_cmd_dump_cnt, int, 0644);
MODULE_PARM_DESC(i2c_cmd_dump_cnt, " print CNT commands and stop, 0xFFFF - don't stop");

int dis_vid2ip = 0;
module_param_named(dis_vid2ip,
                   dis_vid2ip, int, 0644);
MODULE_PARM_DESC(dis_vid2ip, " disable ip override");

int g_chip_type = 6;
module_param_named(g_chip_type,
                   g_chip_type, int, 0644);
MODULE_PARM_DESC(g_chip_type, " set chip type for NO PCI and SGMII");

int eventlist_drops_counter = 0;
module_param_named(eventlist_drops_counter, eventlist_drops_counter, int, 0644);
MODULE_PARM_DESC(eventlist_drops_counter, "Event list drops counter");

int unconsumed_packets_counter = 0;
module_param_named(unconsumed_packets_counter,
		unconsumed_packets_counter, int, 0644);
MODULE_PARM_DESC(unconsumed_packets_counter, "Unconsumed packets counter");

int filtered_lag_packets_counter = 0;
module_param_named(filtered_lag_packets_counter,
		filtered_lag_packets_counter, int, 0644);
MODULE_PARM_DESC(filtered_lag_packets_counter, "Filtered LAG packets counter");

int filtered_port_packets_counter = 0;
module_param_named(filtered_port_packets_counter,
		filtered_port_packets_counter, int, 0644);
MODULE_PARM_DESC(filtered_port_packets_counter, "Filtered port packets counter");

int loopback_packets_counter = 0;
module_param_named(loopback_packets_counter,
		loopback_packets_counter, int, 0644);
MODULE_PARM_DESC(loopback_packets_counter, "Loopback packets counter");

#ifdef CONFIG_PCI_MSI

static int msi_x = 1;
module_param(msi_x, int, 0444);
MODULE_PARM_DESC(msi_x, "attempt to use MSI-X if nonzero");

#else /* CONFIG_PCI_MSI */

static int msi_x = 0;

#endif /* CONFIG_PCI_MSI */

static int sx_core_dev_init_switchx_cb(struct sx_dev *dev, enum sxd_chip_types chip_type);
static int sx_core_init_one(struct sx_priv **sx_priv);
static void sx_core_remove_one(struct sx_priv *priv);
static void inc_eventlist_drops_global_counter(u16 hw_synd);
int sx_init_char_dev(struct cdev *cdev_p);
void sx_deinit_char_dev(struct cdev *cdev_p);


/************************************************
 *  Functions
 ***********************************************/

u16 translate_user_port_to_sysport(struct sx_dev *dev, u32 log_port, int* is_lag)
{
	unsigned int port_type = SX_PORT_TYPE_ID_GET(log_port);
	u16 ret = 0;
	*is_lag = 0;
	if (port_type == SX_PORT_TYPE_LAG) {
		*is_lag = 1;
		return SX_PORT_LAG_ID_GET(log_port);
	}
	else {
		if (SX_PORT_PHY_ID_GET(log_port) == CPU_PORT_PHY_ID) {
			/* Build CPU port route*/
			ret = UCROUTE_CPU_PORT_PREFIX;
			ret |= SX_PORT_DEV_ID_GET(log_port) << UCROUTE_CPU_DEV_BIT_OFFSET;
		}
		else {
			ret = sx_priv(dev)->local_to_system_db[SX_PORT_PHY_ID_GET(log_port)];
			/* For Switchx-2 it's equals to the following translation:
			ret = SX_PORT_DEV_ID_GET(log_port) << UCROUTE_DEV_ID_BIT_OFFSET;
			ret |= (SX_PORT_PHY_ID_GET(log_port) - 1) << UCROUTE_PHY_PORT_BITS_OFFSET;
			*/
		}
	}
	return ret;
}
EXPORT_SYMBOL(translate_user_port_to_sysport);

u32 translate_sysport_to_user_port(struct sx_dev *dev, u16 port, u8 is_lag)
{
    u32 lag_id = 0;

	if (is_lag) {
	    lag_id = port << SX_PORT_LAG_ID_OFFS;
	    lag_id |= SX_PORT_TYPE_LAG << SX_PORT_TYPE_ID_OFFS;
		return lag_id;
	}
	else {
		if ((port & UCROUTE_CPU_PORT_PREFIX) == UCROUTE_CPU_PORT_PREFIX) {
			return ((port & ~UCROUTE_CPU_PORT_PREFIX) >> UCROUTE_CPU_DEV_BIT_OFFSET)
				<< SX_PORT_DEV_ID_OFFS;
		}
		else {
			return ((port >> UCROUTE_DEV_ID_BIT_OFFSET) << SX_PORT_DEV_ID_OFFS) |
				(sx_priv(dev)->system_to_local_db[port]) << SX_PORT_PHY_ID_OFFS;
			/* For Switchx-2 it's equals to the following translation:
			return ((port >> UCROUTE_DEV_ID_BIT_OFFSET) << SX_PORT_DEV_ID_OFFS) |
				(((port >> UCROUTE_PHY_PORT_BITS_OFFSET) & 0xFF) + 1) << SX_PORT_PHY_ID_OFFS;
			*/
		}
	}
}
EXPORT_SYMBOL(translate_sysport_to_user_port);

#define SNOOP_MISS_WA
#if defined(CONFIG_MLNX460EX) && defined(SNOOP_MISS_WA)
int config_l2_force_snoop(void)
{
    struct device_node *np;
    u32                 r;
    const u32          *dcrreg;
    int                 len;
    const u32          *prop;
    u32                 l2_size;
    static u32          dcrbase_l2c = 0;

    if (0 == dcrbase_l2c) {
        np = of_find_compatible_node(NULL, NULL, "ibm,l2-cache");
        if (!np) {
            return 0;
        }

        /* Get l2 cache size */
        prop = of_get_property(np, "cache-size", NULL);
        if (prop == NULL) {
            printk(KERN_ERR "%s: Can't get cache-size!\n", np->full_name);
            of_node_put(np);
            return -ENODEV;
        }
        l2_size = prop[0];

        /* Map DCRs */
        dcrreg = of_get_property(np, "dcr-reg", &len);
        if (!dcrreg || (len != 4 * sizeof(u32))) {
            printk(KERN_ERR "%s: Can't get DCR register base !",
                   np->full_name);
            of_node_put(np);
            return -ENODEV;
        }
        dcrbase_l2c = dcrreg[2];
    }

    /* Force snoop */
    r = mfdcr(dcrbase_l2c + DCRN_L2C0_CFG);
    r |= 0x0000010;
    mtdcr(dcrbase_l2c + DCRN_L2C0_CFG, r);

    return 0;
}
#endif /* SNOOP_MISS_WA */

int sx_core_get_prio2tc(struct sx_dev *dev, uint16_t port_lag_id, uint8_t is_lag, uint8_t pcp, uint8_t *tc)
{
    struct sx_priv *dev_priv = sx_priv(dev);
    uint16_t        local = 0;
    unsigned long   flags;

    if (pcp > MAX_PRIO_NUM) {
        printk(KERN_ERR PFX "PCP %d is invalid. (MAX %d).\n",
               pcp, MAX_PRIO_NUM);
        return -EINVAL;
    }

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
    if (is_lag) {
        *tc = dev_priv->lag_prio2tc[port_lag_id][pcp];
    } else {
        local = dev_priv->system_to_local_db[port_lag_id];
        if (local > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Local %d is invalid. (MAX %d).\n",
                   local, MAX_PHYPORT_NUM);
            spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
            return -EINVAL;
        }
        *tc = dev_priv->port_prio2tc[local][pcp];
    }
    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);

    return 0;
}
EXPORT_SYMBOL(sx_core_get_prio2tc);

int sx_core_get_pvid(struct sx_dev *dev,
                     uint16_t       port_lag_id,
                     uint8_t        is_lag,
                     uint16_t       *pvid)
{
    struct sx_priv *dev_priv = sx_priv(dev);
    uint16_t        local = 0;
    unsigned long   flags;

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
    if (is_lag) {
        *pvid = dev_priv->pvid_lag_db[port_lag_id];
    } else {
        local = dev_priv->system_to_local_db[port_lag_id];
        if (local > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Local %d is invalid. (MAX %d).\n",
                   local, MAX_PHYPORT_NUM);
            spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
            return -EINVAL;
        }
        *pvid = dev_priv->pvid_sysport_db[local];
    }
    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);

    return 0;
}
EXPORT_SYMBOL(sx_core_get_pvid);

int sx_core_get_vlan_tagging(struct sx_dev *dev,
                             uint16_t       port_lag_id,
                             uint8_t        is_lag,
                             uint16_t       vlan,
                             uint8_t       *is_vlan_tagged)
{
    struct sx_priv *dev_priv = sx_priv(dev);
    uint16_t        local = 0;
    unsigned long   flags;

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
    if (is_lag) {
        *is_vlan_tagged = dev_priv->lag_vtag_mode[port_lag_id][vlan];
    } else {
        local = dev_priv->system_to_local_db[port_lag_id];
        if (local > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Local %d is invalid. (MAX %d).\n",
                   local, MAX_PHYPORT_NUM);
            spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
            return -EINVAL;
        }
        *is_vlan_tagged = dev_priv->port_vtag_mode[local][vlan];
    }
    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);

    return 0;
}
EXPORT_SYMBOL(sx_core_get_vlan_tagging);

int sx_core_get_prio_tagging(struct sx_dev *dev, uint16_t port_lag_id, uint8_t is_lag, uint8_t *is_port_prio_tagged)
{
    struct sx_priv *dev_priv = sx_priv(dev);
    uint16_t        local = 0;
    unsigned long   flags;

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
    if (is_lag) {
        *is_port_prio_tagged = dev_priv->lag_prio_tagging_mode[port_lag_id];
    } else {
        local = dev_priv->system_to_local_db[port_lag_id];
        if (local > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Local %d is invalid. (MAX %d).\n",
                   local, MAX_PHYPORT_NUM);
            spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
            return -EINVAL;
        }
        *is_port_prio_tagged = dev_priv->port_prio_tagging_mode[local];
    }
    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);

    return 0;
}
EXPORT_SYMBOL(sx_core_get_prio_tagging);

int sx_core_get_swid(struct sx_dev *dev, struct completion_info *comp_info, uint8_t *swid)
{
    unsigned long   flags;

    *swid = 0;

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);

    if (dev && (sx_priv(dev)->dev_specific_cb.get_swid_cb != NULL)) {
        sx_priv(dev)->dev_specific_cb.get_swid_cb(dev, comp_info, swid);
    } else {
        printk(KERN_ERR PFX "Error retrieving get_swid_cb callback\n");
        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        return -EINVAL;
    }

    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
    return 0;
}
EXPORT_SYMBOL(sx_core_get_swid);

int sx_core_get_lag_mid(struct sx_dev *dev, u16 lag_id, u16 *mid)
{
    unsigned long   flags;
    
    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
    
    if (dev && (sx_priv(dev)->dev_specific_cb.get_lag_mid_cb != NULL)) {
        sx_priv(dev)->dev_specific_cb.get_lag_mid_cb(lag_id, mid);
    } else {
        printk(KERN_ERR PFX "Error retrieving get_lag_mid_cb callback\n");
        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        return -EINVAL;
    }
    
    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
    return 0;
}
EXPORT_SYMBOL(sx_core_get_lag_mid);

int sx_core_get_rp_rif_id(struct sx_dev *dev, uint16_t port_lag_id,
                          uint8_t is_lag, uint16_t vlan_id, uint16_t *rif_id)
{
    struct sx_priv *dev_priv = sx_priv(dev);
    uint16_t        local = 0;
    unsigned long   flags;

    if (vlan_id >= MAX_VLAN_NUM) {
        printk(KERN_ERR PFX "vlan_id %d is invalid. (MAX %d).\n",
               vlan_id, MAX_VLAN_NUM);
        return -EINVAL;
    }

    *rif_id = 0;

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
    if (is_lag) {
        if (port_lag_id > MAX_LAG_NUM) {
            printk(KERN_ERR PFX "port_lag_id %d is invalid. (MAX %d).\n",
                   port_lag_id, MAX_LAG_NUM);
            spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
            return -EINVAL;
        }
        if (!dev_priv->lag_rp_rif_valid[port_lag_id][vlan_id]) {
            printk(KERN_ERR PFX "No RP on LAG ID %d and vlan %d.\n",
                   port_lag_id, vlan_id);
            spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
            return -EINVAL;
        }

        *rif_id = dev_priv->lag_rp_rif[port_lag_id][vlan_id];
    } else {
        local = dev_priv->system_to_local_db[port_lag_id];
        if (local > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Local %d is invalid. (MAX %d).\n",
                   local, MAX_PHYPORT_NUM);
            spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
            return -EINVAL;
        }
        if (!dev_priv->port_rp_rif_valid[local][vlan_id]) {
            printk(KERN_ERR PFX "No RP on port %d and vlan %d.\n",
                   local, vlan_id);
            spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
            return -EINVAL;
        }

        *rif_id = dev_priv->port_rp_rif[local][vlan_id];
    }
    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);

    return 0;
}
EXPORT_SYMBOL(sx_core_get_rp_rif_id);

int sx_core_get_rp_vlan(struct sx_dev *dev, struct completion_info *comp_info, uint16_t *vlan_id)
{
    unsigned long   flags;

    *vlan_id = 0;

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);

    if (dev && (sx_priv(dev)->dev_specific_cb.get_rp_vid_cb != NULL)) {
        sx_priv(dev)->dev_specific_cb.get_rp_vid_cb(dev, comp_info, vlan_id);
    } else {
        printk(KERN_ERR PFX "Error retrieving get_rp_vid_cb callback\n");
        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        return -EINVAL;
    }

    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
    return 0;
}
EXPORT_SYMBOL(sx_core_get_rp_vlan);

int sx_core_get_rp_mode(struct sx_dev *dev, u8 is_lag, u16 sysport_lag_id, u16 vlan_id, u8 *is_rp)
{
    unsigned long flags;
    u16 lag_id = 0;
    uint16_t local = 0;

    if (vlan_id >= MAX_VLAN_NUM) {
        printk(KERN_ERR PFX "vlan_id %d is invalid. (MAX %d).\n",
               vlan_id, MAX_VLAN_NUM);
        return -EINVAL;
    }

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
    if (is_lag) {
        lag_id = (sysport_lag_id >> 4) & 0xfff;
        if (lag_id > MAX_LAG_NUM) {
            printk(KERN_ERR PFX "LAG ID %d is invalid. (MAX %d).\n",
                   lag_id, MAX_LAG_NUM);
            spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
            return -EINVAL;
        }
        *is_rp = sx_priv(dev)->lag_rp_rif_valid[lag_id][vlan_id];
    } else {
        local = sx_priv(dev)->system_to_local_db[sysport_lag_id];
        if (local > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Local %d is invalid. (MAX %d).\n",
                   local, MAX_PHYPORT_NUM);
            spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
            return -EINVAL;
        }
        *is_rp = sx_priv(dev)->port_rp_rif_valid[local][vlan_id];
    }
    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);

    return 0;
}
EXPORT_SYMBOL(sx_core_get_rp_mode);


int sx_core_get_vlan2ip(struct sx_dev *dev, uint16_t vid, uint32_t *ip_addr)
{
    struct sx_priv *dev_priv = sx_priv(dev);
    unsigned long   flags;

    if (dis_vid2ip) {
        return 0;
    }

    if (vid >= MAX_VLAN_NUM) {
        printk(KERN_ERR PFX "vid %d is invalid. (MAX %d).\n",
               vid, MAX_VLAN_NUM);
        return -EINVAL;
    }

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);

    *ip_addr = dev_priv->icmp_vlan2ip_db[vid];

    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);

    return 0;
}
EXPORT_SYMBOL(sx_core_get_vlan2ip);

int sx_core_get_fid_by_port_vid(struct sx_dev *dev, struct completion_info *comp_info, uint16_t *fid)
{    
    struct sx_priv *dev_priv = sx_priv(dev);
    uint16_t        local = 0;
    unsigned long   flags;
    uint8_t         is_lag = comp_info->is_lag;
    uint16_t        sysport_lag_id = comp_info->sysport;
    uint16_t        lag_port_id = comp_info->lag_subport;
    uint16_t        vid = comp_info->vid;    

    if (vid >= MAX_VLAN_NUM) {
        printk(KERN_ERR PFX "vid %d is invalid. (MAX %d).\n",
               vid, MAX_VLAN_NUM);
        return -EINVAL;
    }

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
    if (is_lag) {  
        u16 lag_id = (sysport_lag_id >> 4) & 0xfff;
        local = sx_priv(dev)->lag_member_to_local_db[lag_id][lag_port_id];        
    } else {
        local = dev_priv->system_to_local_db[sysport_lag_id];
        if (local > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Local %d is invalid. (MAX %d).\n",
                   local, MAX_PHYPORT_NUM);
            spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
            return -EINVAL;
        }        
    }
    *fid = dev_priv->port_vid_to_fid[local][vid];
    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);

    return 0;
}
EXPORT_SYMBOL(sx_core_get_fid_by_port_vid);


static int sx_raise_event(struct sx_dev *dev, void *data, void *context)
{
    int                    err = 0;
    struct sk_buff        *skb = NULL;
    struct ku_raise_trap   event_data;
    struct completion_info ci;
    void                  *buff;

    err = copy_from_user((void*)(&event_data), data, sizeof(event_data));
    if (err) {
        goto out_err;
    }

    if ((event_data.buffer_size == 0) ||
        (event_data.trap_id > NUM_HW_SYNDROMES)) {
        printk(KERN_WARNING "sx_raise_event: Bad parameters\n");
        err = -EINVAL;
        goto out_err;
    }

    skb = alloc_skb(event_data.buffer_size, GFP_KERNEL);
    if (!skb) {
        err = -ENOMEM;
        goto out_err;
    }

    buff = skb_put(skb, event_data.buffer_size);
    if (buff == NULL) {
        err = -ENOMEM;
        goto out;
    }
    err = copy_from_user(buff, event_data.buffer_p,
                         event_data.buffer_size);
    if (err) {
        goto out;
    }

#ifdef SX_DEBUG
    printk(KERN_DEBUG PFX "sx_raise_event: got from user:\n");
    printk(KERN_DEBUG PFX "sx_raise_event: trap_id = %u\n",
           event_data.trap_id);
    printk(KERN_DEBUG PFX "sx_raise_event: buffer_size = %u\n",
           event_data.buffer_size);
    SX_CORE_HEXDUMP16(event_data.buffer_p, event_data.buffer_size);
#endif

    memset(&ci, 0, sizeof(ci));
    ci.hw_synd = event_data.trap_id;
    ci.sysport = event_data.sysport;
    ci.is_lag = event_data.is_lag;
    ci.lag_subport = event_data.lag_subport;
    ci.swid = event_data.swid;
    ci.skb = skb;
    ci.pkt_type = PKT_TYPE_ETH;
    ci.info.eth.dmac = DMAC_DONT_CARE_VALUE;
    ci.info.eth.ethtype = ETHTYPE_DONT_CARE_VALUE;
    ci.info.eth.emad_tid = TID_DONT_CARE_VALUE;
    ci.context = context;
    dispatch_pkt(dev, &ci, event_data.trap_id, 0);
out:
    kfree_skb(skb);
out_err:
    return err;
}

/**
 * Update the device's cap struct with the default capabilities of the HW
 * (number of RDQs, SDQs, CQs Etc.)
 */
static void set_default_capabilities(struct sx_dev *dev)
{
    dev->dev_cap.log_max_rdq_sz = 7;
    dev->dev_cap.log_max_sdq_sz = 7;
    dev->dev_cap.log_max_cq_sz = 7;

    dev->dev_cap.max_num_rdqs = NUMBER_OF_RDQS;
    dev->dev_cap.max_num_sdqs = NUMBER_OF_SDQS;
    dev->dev_cap.max_num_cqs = NUMBER_OF_RDQS + NUMBER_OF_SDQS;

    dev->dev_cap.max_num_cpu_egress_tcs = 12;
    dev->dev_cap.max_num_cpu_ingress_tcs = 16;
}

int ver_set_capabilities(struct sx_dev_cap *cap)
{
    struct sx_dev *dev = sx_glb.tmp_dev_ptr;

    printk(KERN_DEBUG PFX "ver_set_capabilities: Entered function\n");
    printk(KERN_DEBUG PFX "ver_set_capabilities: "
           "cap->log_max_rdq_sz = %d\n", cap->log_max_rdq_sz);

    dev->dev_cap.log_max_rdq_sz = cap->log_max_rdq_sz;
    dev->dev_cap.log_max_sdq_sz = cap->log_max_sdq_sz;
    dev->dev_cap.log_max_cq_sz = cap->log_max_cq_sz;

    dev->dev_cap.max_num_rdqs = cap->max_num_rdqs;
    dev->dev_cap.max_num_sdqs = cap->max_num_sdqs;
    dev->dev_cap.max_num_cqs = cap->max_num_cqs;

    dev->dev_cap.max_num_cpu_egress_tcs = cap->max_num_cpu_egress_tcs;
    dev->dev_cap.max_num_cpu_ingress_tcs = cap->max_num_cpu_ingress_tcs;

    return 0;
}
EXPORT_SYMBOL(ver_set_capabilities);

/**
 * Return a pointer to the sx device
 *
 * returns: Pointer to the sx device - success
 *	    NULL                     - error
 */
void * sx_get_dev_context(void)
{
    return sx_glb.sx_dpt.dpt_info[DEFAULT_DEVICE_ID].
           sx_pcie_info.sx_dev;
}
EXPORT_SYMBOL(sx_get_dev_context);

/************************************************
 *  Helper Functions
 ***********************************************/
void inc_unconsumed_packets_counter(struct sx_dev *dev, u16 hw_synd, enum sx_packet_type pkt_type)
{
    inc_unconsumed_packets_global_counter( hw_synd, pkt_type);
	if (dev) {
    	dev->stats.rx_unconsumed_by_synd[hw_synd][pkt_type]++;
    	dev->unconsumed_packets_counter++;
	}
#ifdef SX_DEBUG
    printk(KERN_ERR PFX "A packet with trap ID 0x%x and type %s "
           "was not consumed\n", hw_synd, sx_cqe_packet_type_str[pkt_type]);
#endif
}

void inc_eventlist_drops_counter(struct sx_dev* sx_dev, u16 hw_synd)
{
	inc_eventlist_drops_global_counter(hw_synd);

    if (sx_dev == NULL) {
        printk(KERN_ERR PFX "sx_dev is NULL\n");
        return;
    }

    sx_dev->eventlist_drops_counter++;
    sx_dev->stats.rx_eventlist_drops_by_synd[hw_synd]++;
#ifdef SX_DEBUG
    printk(KERN_ERR PFX "A packet with trap ID 0x%x "
           "was dropped from the event list\n", hw_synd);
#endif
}




void inc_filtered_lag_packets_counter(struct sx_dev *dev)
{
	inc_filtered_lag_packets_global_counter();
    dev->filtered_lag_packets_counter++;
}

void inc_filtered_port_packets_counter(struct sx_dev *dev)
{
    inc_filtered_port_packets_global_counter();
    dev->filtered_port_packets_counter++;
}

void inc_unconsumed_packets_global_counter(u16 hw_synd, enum sx_packet_type pkt_type)
{
	unconsumed_packets_counter++;
	sx_glb.stats.rx_unconsumed_by_synd[hw_synd][pkt_type]++;
 #ifdef SX_DEBUG
 	printk(KERN_ERR PFX "A packet with trap ID 0x%x and type %s "
 			"was not consumed\n", hw_synd, sx_cqe_packet_type_str[pkt_type]);
 #endif
}

static void inc_eventlist_drops_global_counter( u16 hw_synd)
{
	eventlist_drops_counter++;
	sx_glb.stats.rx_eventlist_drops_by_synd[hw_synd]++;
 #ifdef SX_DEBUG
 	printk(KERN_ERR PFX "A packet with trap ID 0x%x "
 			"was dropped from the event list\n", hw_synd);
 #endif
}

void inc_filtered_lag_packets_global_counter(void)
{
	filtered_lag_packets_counter++;
}

void inc_filtered_port_packets_global_counter(void)
{
  filtered_port_packets_counter++;
}

static int check_valid_meta(struct sx_dev *dev, struct isx_meta *meta)
{
    if (meta->etclass >= NUMBER_OF_ETCLASSES) {
        if (printk_ratelimit()) {
            printk(KERN_WARNING PFX "Error: etclass (%u) "
                   "is invalid\n",
                   meta->etclass);
        }
        return -ERANGE;
    }

    if ((meta->rdq >= NUMBER_OF_RDQS) &&
        (meta->rdq != 0x1F)) {
        if (printk_ratelimit()) {
            printk(KERN_WARNING PFX "Error: rdq (%u) is invalid\n",
                   meta->rdq);
        }
        return -ERANGE;
    }

    if ((meta->swid >= NUMBER_OF_SWIDS) &&
        (meta->swid != SWID_NUM_DONT_CARE)) {
        if (printk_ratelimit()) {
            printk(KERN_WARNING PFX "Error: swid (%u) is invalid\n",
                   meta->swid);
        }
        return -ERANGE;
    }

    /* Validate the swid really exists */
    if (dev != NULL) {
        if ((meta->swid != SWID_NUM_DONT_CARE) &&
            (0 == sx_bitmap_test(&sx_priv(dev)->swid_bitmap, meta->swid))) {
            printk(KERN_WARNING PFX "Error: swid (%u) does not exists\n", meta->swid);
            return -EINVAL;
        }
    }

    if ((meta->to_cpu != false) && (meta->to_cpu != true)) {
        if (printk_ratelimit()) {
            printk(KERN_WARNING PFX "Error: to_cpu (%u) "
                   "is invalid\n",
                   meta->to_cpu);
        }
        return -ERANGE;
    }

    if ((meta->type < SX_PKT_TYPE_MIN) ||
        (meta->type > SX_PKT_TYPE_MAX)) {
        if (printk_ratelimit()) {
            printk(KERN_WARNING PFX "Error: type (%d) is invalid\n",
                   meta->type);
        }
        return -ERANGE;
    }

    return 0;
}

/**
 * Get a packet capsulated in ku struct
 * (scattered to small buffers pointed by internal iovector)
 * and gather them together to one buffer capslated in the given skb data.
 * (NOTE: allocate skb)
 *
 * @param ku[in]   -  The given ku that capsulate the scattered packet
 * @param reserve_hdrs[in]   -  Should the function reserve place for headers
 * @param skb[out] -  A pointer to the pointer of allocated skb
 *                    to be updated by the function with the gathered packet
 *
 * returns: 0 success
 *	    !0 error
 */
static int copy_buff_to_skb(struct sk_buff **skb, struct ku_write *write_data, u8 reserve_hdrs)
{
    int            err = 0;
    int            index = 0;
    int            packet_size = 0;
    unsigned char *p_skb_data = NULL;
    struct iovec  *iov;
    int            max_headers_size = 0;

    if ((!skb) || (!write_data)) {
        err = -EINVAL;
        goto out_err;
    }

    iov = kmalloc(sizeof(*iov) * (write_data->vec_entries), GFP_KERNEL);
    if (!iov) {
        err = -ENOMEM;
        goto out_err;
    }

    /*1. copy iovector */
    err = copy_from_user((void*)iov, (void*)(write_data->iov),
                         sizeof(*iov) * (write_data->vec_entries));
    if (err) {
        goto out_free;
    }

    /* 2. calc the packet size */
    for (index = 0; index < write_data->vec_entries; index++) {
        /*valid param check*/
        if ((iov[index].iov_len == 0) || (iov[index].iov_base == NULL)) {
            err = -EINVAL;
            goto out_free;
        }

        packet_size += iov[index].iov_len;
    }

    packet_size += 2; /* ETH FCS when using SGMII */
    if (reserve_hdrs == true) {
        max_headers_size = ISX_HDR_SIZE +
                           sizeof(struct sx_sgmii_ctrl_segment) +
                           sizeof(struct sx_ethernet_header);
    }

    /* 3. allocate skb according to packet size */
    *skb = alloc_skb(packet_size + max_headers_size, GFP_KERNEL);
    if (!*skb) {
        err = -ENOMEM;
        goto out_free;
    }

    if (max_headers_size) {
        skb_reserve(*skb, max_headers_size);
    }

    /*4. copy the scattered buffers of the           */
    /*   packet to be gathered inside the skb buffer */
    for (index = 0; index < write_data->vec_entries; index++) {
        p_skb_data = skb_put(*skb, iov[index].iov_len);
        if (p_skb_data == NULL) {
            printk(KERN_WARNING PFX "copy_ku_to_skb: "
                   "skb_put failed\n");
            err = -EFAULT;
            goto out_free_skb;
        }

        memset(p_skb_data, 0, iov[index].iov_len);
        err = copy_from_user(p_skb_data, iov[index].iov_base,
                             iov[index].iov_len);
        if (err) {
            goto out_free_skb;
        }
    }

    goto out_free;

out_free_skb:
    kfree_skb(*skb);
out_free:
    kfree(iov);
out_err:
    return err;
}

static int sx_send_loopback(struct sx_dev *dev, struct ku_write *write_data, void *context)
{
    int                    err = 0;
    struct completion_info ci;

    memset(&ci, 0, sizeof(ci));
    err = copy_buff_to_skb(&ci.skb, write_data, false);
    if (err) {
        printk(KERN_WARNING "sx_send_loopback: failed copying buffer to SKB\n");
        goto out;
    }

#ifdef SX_DEBUG
    printk(KERN_DEBUG PFX "sx_send_loopback: got from user:\n");
    printk(KERN_DEBUG PFX "sx_send_loopback: trap_id = %u\n",
           write_data->meta.loopback_data.trap_id);
    printk(KERN_DEBUG PFX "sx_send_loopback: buffer_size = %u\n",
           ci.skb->len);
    SX_CORE_HEXDUMP16(ci.skb->data, ci.skb->len);
#endif

    ci.swid = write_data->meta.swid;
    ci.sysport = write_data->meta.system_port_mid;
    ci.hw_synd = write_data->meta.loopback_data.trap_id;
    ci.is_send = 0;
    ci.pkt_type = PKT_TYPE_ETH;
    ci.info.eth.dmac = DMAC_DONT_CARE_VALUE;
    ci.info.eth.ethtype = ETHTYPE_DONT_CARE_VALUE;
    ci.info.eth.emad_tid = TID_DONT_CARE_VALUE;
    ci.is_lag = write_data->meta.loopback_data.is_lag;
    ci.lag_subport = write_data->meta.loopback_data.lag_subport;
    ci.is_tagged = 0;
    ci.vid = 0;
    ci.context = context;
    dispatch_pkt(dev, &ci, ci.hw_synd, 0);
out:
    kfree_skb(ci.skb);
    return err;
}


/**
 * Get edata which is the packet parameters and a place to copy it (buf).
 *
 * Create a meta-data struct from the edata and copy it to the user
 * After it in sequential way copy the packet
 * (NOTE: the function assuming there is enough place in the given buf )
 *
 * param[in] buf   - The place the meta-data + packet should be copied to
 * param[in] edata - The packet and meta-data parameters
 *
 * returns: positive number on success (the copied size of meta-data + packet)
 *	    !0 error
 */
static int copy_pkt_to_user(char __user *buf, struct event_data *edata)
{
    struct ku_read metadata;
    int            copied_size = 0;

#ifdef SX_DEBUG
    printk(KERN_DEBUG PFX "copy_pkt_to_user()\n");
#endif
    /* copy the packet  */
    metadata.length = edata->skb->len;
    metadata.system_port = edata->system_port;
    metadata.trap_id = edata->trap_id;
    metadata.is_lag = edata->is_lag;
    metadata.swid = edata->swid;
    metadata.original_packet_size = edata->original_packet_size;
    if (edata->is_lag) {
        metadata.lag_subport = edata->lag_sub_port;
    }

    if (copy_to_user(buf, &metadata, sizeof(metadata))) {
        return -EFAULT;
    }

    copied_size += sizeof(metadata);
    if (copy_to_user(buf + copied_size, edata->skb->data, edata->skb->len)) {
        return -EFAULT;
    }

    copied_size += edata->skb->len;
#ifdef SX_DEBUG
    printk(KERN_DEBUG PFX " copy_pkt_to_user() metadata.length=[%llu] " \
           "metadata.system_port=[%d] metadata.trap_id=[%d]\n",
           metadata.length, metadata.system_port,
           metadata.trap_id);
    SX_CORE_HEXDUMP16((void*)&metadata, sizeof(metadata));
    SX_CORE_HEXDUMP16(edata->skb->data, edata->skb->len);
#endif

    return copied_size;
}

/**
 *
 * Copy the edata that holds packet/s to the given buf
 * NOTE: The function assumes that there is enough free space in
 * the buffer (which was already checked before calling here)
 *
 * param[in] edata - The packet and meta-data parameters
 * param[in] buf   - the data buffer, ku_read struct
 *
 * returns: The size of data which copied to the user buffer
 *	   !0 error
 */
static int copy_edata_to_user(struct event_data *edata, char __user *buf)
{
    struct event_data *tmp = NULL;
    int                so_far_copied = 0;
    int                copied_size = 0;
    int                err = 0;
    struct list_head  *pos, *q;
    struct sx_dev    * sx_dev = NULL;

#ifdef SX_DEBUG
    printk(KERN_DEBUG PFX " copy_edata_to_user()\n");
#endif
    list_for_each_safe(pos, q, &edata->list) {
        tmp = list_entry(pos, struct event_data, list);
        copied_size = copy_pkt_to_user(buf + so_far_copied, tmp);
        if (copied_size < 0) {
            err = copied_size;
            goto out_free;
        }

        so_far_copied += copied_size;
        list_del(pos);
        kfree_skb(tmp->skb);
        kfree(tmp);
    }

    return so_far_copied;

out_free:
    list_for_each_safe(pos, q, &edata->list) {
        tmp = list_entry(pos, struct event_data, list);
        sx_dev = tmp->dev;
        if (sx_dev && !sx_dev->eventlist_drops_counter) {
            printk(KERN_WARNING PFX
                   "copy_pkt_to_user failed, " \
                   "dropping RX packet\n");
        }

        inc_eventlist_drops_counter(sx_dev, tmp->trap_id);
        list_del(pos);
        kfree_skb(tmp->skb);
        kfree(tmp);
    }

    return so_far_copied ? so_far_copied : err;
}


/**
 *
 * Create edata linked-list and return it
 *
 * while the event list is not empty, pop event-data ,
 * if the packet size in the event-data is not to big then add it to
 * the buffer counter and continue while the buffer counter is
 * less then the user buffer
 *
 * param[in] evlist - The given event list
 * param[in] user_counter  - The number of bites to send(the user buffer size)
 *
 *
 * returns:
 *	edata list on - success 
 *      NULL          - error
 * retval:
 * 	0 on success, otherwise the size of the buffer needed for the 
 * 	first packet if there is enough space in the buffer for 
 * 	metadata, -ENOMEM if not 
 */
static int get_edata_from_elist(int               *evlist_size,
                                struct event_data *edata_list,
                                struct list_head  *evlist,
                                size_t             user_counter,
                                int                multi_packet_read_enable)
{
    struct  list_head  *pos, *q;
    struct  event_data *edata = NULL;
    size_t              buf_counter = 0;
    int                 pkt_size = 0;

    list_for_each_safe(pos, q, evlist) {
        edata = list_entry(pos, struct event_data, list);
        pkt_size = sizeof(struct ku_read) + edata->skb->len;
        if (buf_counter + pkt_size > user_counter) {
            break;
        }

        buf_counter += pkt_size;
        list_del(pos);
        list_add_tail(&edata->list, &edata_list->list);
        (*evlist_size)--;
        if (multi_packet_read_enable == false) {
            break;
        }
    }

    /* Not enough place for even a single packet */
    if (0 == buf_counter) {
        if (sizeof(struct ku_read) > user_counter) {
            return -ENOMEM;
        }
        return pkt_size;
    }
    return 0;
}

/**
 * Create new listener with the given swid,type,critireas and add it to an entry
 * in sx device listeners data-base when the entry is according to the hw_synd
 *
 * Note: A default listener is a listener which receives all the packets
 *       that haven't been consumed by another non-default listener.
 *
 * param swid            [in] - The listener swid
 * param hw_synd         [in] - The listener syndrome number
 * param type            [in] - The listener type - ETH,IB or FC, or Don't care
 * param is_default      [in] - If the listener is a default listener
 * param filter_critireas[in] - The listener additional filter critireas
 * param handler         [in] -
 * param context         [in] -
 * param check_dup       [in] - block the listener from registering twice to same trap.
 *
 * returns: 0 success
 *	   !0 error
 *
 */
int sx_core_add_synd(u8                        swid,
                     u16                       hw_synd,
                     enum l2_type              type,
                     u8                        is_default,
                     union ku_filter_critireas crit,
                     cq_handler                handler,
                     void                     *context,
                     check_dup_e               check_dup,
                     struct sx_dev           * sx_dev)
{
    unsigned long          flags;
    struct listener_entry *new_listener = NULL;
    struct list_head      *pos;
    struct listener_entry *listener;
    unsigned int           found_same_listener;

    /* if NULL use default sx_dev */
    if (sx_dev == NULL) {
        sx_dev = sx_glb.tmp_dev_ptr;
    }

#if 0
    if (!context) {
        printk(KERN_WARNING PFX "sx_core_add_synd: Cannot add listener, context is NULL\n");
        return -EINVAL;
    }
#endif

    if (!handler) {
        printk(KERN_WARNING PFX "sx_core_add_synd: Cannot add listener, handler is NULL\n");
        return -EINVAL;
    }

    new_listener = kmalloc(sizeof(*new_listener), GFP_ATOMIC);
    if (!new_listener) {
        printk(KERN_WARNING PFX "sx_core_add_synd: Failed allocating memory for the new listener\n");
        return -ENOMEM;
    }

    new_listener->swid = swid;
    new_listener->critireas = crit;
    new_listener->handler = handler;
    new_listener->context = context;
    new_listener->listener_type = type;
    new_listener->is_default = is_default;
    new_listener->rx_pkts = 0;
    spin_lock_irqsave(&sx_glb.listeners_lock, flags);
    /* default listeners are stored at Don't care */
    /* entry, at the end of the list              */

    if (is_default) {
        list_add_tail(&(new_listener->list),
                      &(sx_glb.listeners_db[hw_synd].list));
    } else {
        found_same_listener = 0;
        if ((check_dup == CHECK_DUP_ENABLED_E) &&
            !list_empty(&sx_glb.listeners_db[hw_synd].list)) {
            list_for_each(pos, &sx_glb.listeners_db[hw_synd].list) {
                listener = list_entry(pos, struct listener_entry, list);
                if (listener->context == context) {
                    found_same_listener = 1;
                    printk(KERN_WARNING PFX "this listener is already " \
                           "listening for that trap \n");
                }
            }
        }

        if (found_same_listener == 0) {
            list_add(&(new_listener->list),
                     &(sx_glb.listeners_db[hw_synd].list));
        } else {
            kfree(new_listener);
        }
    }
    spin_unlock_irqrestore(&sx_glb.listeners_lock, flags);

    return 0;
}
EXPORT_SYMBOL(sx_core_add_synd);

static int sx_core_add_synd_l3(u8 swid, u16 hw_synd, struct sx_dev *dev)
{
	union sx_event_data event_data;

	event_data.eth_l3_synd.swid = swid;
	event_data.eth_l3_synd.hw_synd = hw_synd;
	sx_core_dispatch_event(dev, SX_DEV_EVENT_ADD_SYND_NETDEV, &event_data);
	return 0;
}

static int sx_core_remove_synd_l3(u8 swid, u16 hw_synd, struct sx_dev *dev)
{
	union sx_event_data event_data;

	event_data.eth_l3_synd.swid = swid;
	event_data.eth_l3_synd.hw_synd = hw_synd;
	sx_core_dispatch_event(dev, SX_DEV_EVENT_REMOVE_SYND_NETDEV, &event_data);
	return 0;
}

static int sx_core_add_synd_l2(u8 swid, u16 hw_synd, struct sx_dev *dev)
{
	union sx_event_data event_data;

	event_data.eth_l3_synd.swid = swid;
	event_data.eth_l3_synd.hw_synd = hw_synd;
	sx_core_dispatch_event(dev, SX_DEV_EVENT_ADD_SYND_L2_NETDEV, &event_data);
	return 0;
}

static int sx_core_remove_synd_l2(u8 swid, u16 hw_synd, struct sx_dev *dev)
{
	union sx_event_data event_data;

	event_data.eth_l3_synd.swid = swid;
	event_data.eth_l3_synd.hw_synd = hw_synd;
	sx_core_dispatch_event(dev, SX_DEV_EVENT_REMOVE_SYND_L2_NETDEV, &event_data);
	return 0;
}

static int sx_core_add_synd_ipoib(u8 swid, u16 hw_synd, struct sx_dev *dev)
{
    union sx_event_data event_data;

    event_data.ipoib_synd.swid = swid;
    event_data.ipoib_synd.hw_synd = hw_synd;
    sx_core_dispatch_event(dev, SX_DEV_EVENT_ADD_SYND_IPOIB, &event_data);

    return 0;
}

static int sx_core_remove_synd_ipoib(u8 swid, u16 hw_synd, struct sx_dev *dev)
{
    union sx_event_data event_data;

    event_data.ipoib_synd.swid = swid;
    event_data.ipoib_synd.hw_synd = hw_synd;
    sx_core_dispatch_event(dev, SX_DEV_EVENT_REMOVE_SYND_IPOIB, &event_data);

    return 0;
}

static int is_to_remove_listener(u8                        swid,
                                 enum l2_type              type,
                                 u8                        is_default,
                                 union ku_filter_critireas critireas,
                                 void                     *context,
                                 struct listener_entry    *listener)
{
    if (listener->context != context) {
        return false;
    }

    if (listener->is_default != is_default) {
        return false;
    }

    if (is_default) {
        return true;
    }

    if (listener->swid != swid) {
        return false;
    }

    if (listener->listener_type != type) {
        return false;
    }

    switch (type) {
    case L2_TYPE_DONT_CARE:
        if (listener->critireas.dont_care.sysport != critireas.dont_care.sysport) {
            return false;
        }

        break;

    case L2_TYPE_ETH:
        if (listener->critireas.eth.ethtype != critireas.eth.ethtype) {
            return false;
        }

        if (listener->critireas.eth.dmac != critireas.eth.dmac) {
            return false;
        }

        if (listener->critireas.eth.emad_tid != critireas.eth.emad_tid) {
            return false;
        }

        if (listener->critireas.eth.from_rp != critireas.eth.from_rp) {
            return false;
        }

        if (listener->critireas.eth.from_bridge != critireas.eth.from_bridge) {
            return false;
        }

        break;

    case L2_TYPE_IB:
        if (listener->critireas.ib.qpn != critireas.ib.qpn) {
            return false;
        }

        break;

    default:
        break;
    }

    return true;
}

int sx_core_remove_synd(u8                        swid,
                        u16                       hw_synd,
                        enum l2_type              type,
                        u8                        is_default,
                        union ku_filter_critireas critireas,
                        void                     *context,
                        struct sx_dev           * sx_dev)
{
    unsigned long          flags;
    struct listener_entry *listener = NULL;
    struct list_head      *pos = NULL;
    struct list_head      *q = NULL;
    int                    listener_removed = 0;
    int                    entry = 0;

    /*
     * if NULL use default sx_dev
     * TODO: add instead logic for all_devs
     */
    if (sx_dev == NULL) {
        sx_dev = sx_glb.tmp_dev_ptr;
    }

#if 0
    if (context == NULL) {
        printk(KERN_DEBUG PFX "sx_core_remove_synd: Err: context = "
               "NULL, exiting...\n");
        return -EINVAL;
    }
#endif

    spin_lock_irqsave(&sx_glb.listeners_lock, flags);
    /* Used to be "is_default ? NUM_HW_SYNDROMES : hw_synd;" removed, since we 
       would like to use is_default for PUDE */
    entry = hw_synd; 
    if (!list_empty(&(sx_glb.listeners_db[entry].list))) {
        list_for_each_safe(pos, q, &sx_glb.listeners_db[entry].list) {
            listener = list_entry(pos, struct listener_entry, list);
            if (is_to_remove_listener(swid, type, is_default,
                                      critireas, context, listener)) {
                list_del(pos);
                /* Listener was allocated as atomic memory
                 * so it's OK to free it under spinlock */
                kfree(listener);
                listener_removed = 1;
                break;
            }
        }
    }

    spin_unlock_irqrestore(&sx_glb.listeners_lock, flags);
    if (listener_removed == 0) {
        printk(KERN_WARNING PFX "sx_core_remove_synd: no matching "
               "listener was found\n");
        return -EAGAIN;
    }

    return 0;
}
EXPORT_SYMBOL(sx_core_remove_synd);
static void sx_cq_handler(struct completion_info *comp_info, void *context)
{
    unsigned long      flags;
    struct event_data *edata = NULL;
    struct sx_rsc     *file = ((struct file *)(context))->private_data;
    struct sx_dev     *sx_dev = comp_info->dev;
    struct sk_buff    *skb = comp_info->skb;

    skb_get(skb);
    edata = kmalloc(sizeof(*edata), GFP_ATOMIC);
    if (edata == NULL) {
        if (sx_dev && !sx_dev->eventlist_drops_counter) {
            printk(KERN_WARNING PFX "Memory allocation "
                   "for event data failed, "
                   "dropping RX packet\n");
        }
        inc_eventlist_drops_counter(sx_dev, comp_info->hw_synd);
        goto out_free;
    }

    /* update edata params */
    INIT_LIST_HEAD(&edata->list);
    edata->skb = skb;
    edata->system_port = comp_info->sysport;
    edata->trap_id = comp_info->hw_synd;
    edata->dev_id = 0;
    edata->dev = sx_dev;
    edata->is_lag = comp_info->is_lag;
    edata->lag_sub_port = comp_info->lag_subport;
    edata->swid = comp_info->swid;
    edata->original_packet_size = comp_info->original_packet_size;
#ifdef SX_DEBUG
    printk(KERN_DEBUG PFX " sx_cq_handler(): skb->len=[%d]  sysport=[%d]"
           " hw_synd(trap_id)=[%d]\n",
           skb->len, edata->system_port, edata->trap_id);
#endif

    spin_lock_irqsave(&file->lock, flags);

    if (file->evlist_size < SX_EVENT_LIST_SIZE) {
        list_add_tail(&edata->list, &file->evlist.list);
        file->evlist_size++;
        wake_up_interruptible(&file->poll_wait);
        spin_unlock_irqrestore(&file->lock, flags);
        goto out_ok;
    }

    spin_unlock_irqrestore(&file->lock, flags);

    if ((sx_dev != NULL) && (!sx_dev->eventlist_drops_counter)) {
        printk(KERN_WARNING PFX "Event list is full, "
               "dropping RX packet\n");
    }
    inc_eventlist_drops_counter(sx_dev, comp_info->hw_synd);

out_free:
    kfree_skb(skb);
    kfree(edata);
out_ok:
    return;
}

static void sx_l2_tunnel_handler(struct completion_info *comp_info, void *context)
{
    struct sx_dev  *dev = (struct sx_dev *)context;
    struct sk_buff *skb = comp_info->skb;

    /* sx_priv(dev) has the relevant info for building the headers and sending */
    dev = NULL;
    skb_get(skb);
    kfree_skb(skb);

    return;
}

/**
 * This function is used to check the validity of the given ku
 *
 * param[in] ku_synd_ioctl - The given ku
 *
 * returns: 0 success
 *	   !0 error
 */
static int check_valid_ku_synd(struct ku_synd_ioctl *ku)
{
    int err = 0;

    if ((ku->is_default != false) && (ku->is_default != true)) {
        err = -EINVAL;
#ifdef SX_DEBUG
        printk(KERN_DEBUG PFX "The given ku_synd_ioctl not valid: "
               " ku->is_default=[%d]\n", ku->is_default);
#endif
    }

    if ((ku->swid >= NUMBER_OF_SWIDS) && (ku->swid != SWID_NUM_DONT_CARE)) {
        err = -EINVAL;
#ifdef SX_DEBUG
        printk(KERN_DEBUG PFX "The given ku_synd_ioctl not valid: "
               " ku->swid=[%d]\n", ku->swid);
#endif
    }

    if (ku->syndrome_num > NUM_HW_SYNDROMES) {
        err = -EINVAL;
#ifdef SX_DEBUG
        printk(KERN_DEBUG PFX "The given ku_synd_ioctl not valid: "
               " ku->syndrome_num=[%d]\n", ku->syndrome_num);
#endif
    }

    return err;
}

static int check_valid_profile(struct sx_dev *dev, struct sx_pci_profile *profile)
{
    int i, j;

    for (i = 0; i < NUMBER_OF_SWIDS; i++) {
        for (j = 0; j < NUMBER_OF_ETCLASSES; j++) {
            if (profile->tx_prof[i][j].sdq >=
                dev->dev_cap.max_num_sdqs) {
                printk(KERN_WARNING PFX "sdq num is > max\n");
                return -EINVAL;
            }
            if (profile->tx_prof[i][j].stclass >= 8) {
                printk(KERN_WARNING PFX "stclass num is > "
                       "max\n");
                return -EINVAL;
            }
        }
    }
    if (profile->emad_tx_prof.sdq >= dev->dev_cap.max_num_sdqs) {
        printk(KERN_WARNING PFX "emad sdq num is > max\n");
        return -EINVAL;
    }
    if (profile->emad_tx_prof.stclass >= 8) {
        printk(KERN_WARNING PFX "emad stclass num is > max\n");
        return -EINVAL;
    }
    for (i = 0; i < NUMBER_OF_SWIDS; i++) {
        if (profile->rdq_count[i] > dev->dev_cap.max_num_rdqs) {
            printk(KERN_WARNING PFX "sdq num is > max\n");
            return -EINVAL;
        }
        for (j = 0; j < profile->rdq_count[i]; j++) {
            if (profile->rdq[i][j] >= dev->dev_cap.max_num_rdqs) {
                printk(KERN_WARNING PFX "rdq[%d][%d] = %d "
                       "> max\n", i, j, profile->rdq[i][j]);
                return -EINVAL;
            }
        }
    }
    if ((profile->pci_profile >= PCI_PROFILE_EN_SINGLE_SWID) &&
        (profile->emad_rdq >= dev->dev_cap.max_num_rdqs)) {
        printk(KERN_WARNING PFX "emad_rdq = %d > max\n",
               profile->emad_rdq);
        return -EINVAL;
    }

    for (i = 0; i < NUMBER_OF_SWIDS; i++) {
        if ((profile->swid_type[i] != SX_KU_L2_TYPE_IB) &&
            (profile->swid_type[i] != SX_KU_L2_TYPE_ETH) &&
            (profile->swid_type[i] != SX_KU_L2_TYPE_ROUTER_PORT) &&
            (profile->swid_type[i] != SX_KU_L2_TYPE_DONT_CARE)) {
            printk(KERN_WARNING PFX "for swid %d type = "
                   "%d is wrong!\n",
                   i, profile->swid_type[i]);
            return -EINVAL;
        }
    }

    for (i = 0; i < NUMBER_OF_RDQS; i++) {
        if ((profile->rdq_properties[i].number_of_entries >
             1 << dev->dev_cap.log_max_rdq_sz) ||
            (profile->rdq_properties[i].entry_size >
             SX_MAX_MSG_SIZE)) {
            printk(KERN_WARNING PFX "%s:%d\n", __FILE__, __LINE__);
            printk(KERN_WARNING PFX "%d: n_enties %d , max_rdq_sz:"
                   " %d , entry_sz:%d, SX_MAX_MSG_SZ:%d\n",
                   i,
                   profile->rdq_properties[i].number_of_entries,
                   1 << dev->dev_cap.log_max_rdq_sz,
                   profile->rdq_properties[i].entry_size,
                   SX_MAX_MSG_SIZE);
            return -EINVAL;
        }
    }

    for (i = 0; i < NUMBER_OF_SDQS; i++) {
        if (profile->cpu_egress_tclass[i] >= 64) {
            printk(KERN_WARNING PFX "cpu_egress_tclass[%d] = "
                   "%d > max\n",
                   i, profile->cpu_egress_tclass[i]);
            return -EINVAL;
        }
    }

    return 0;
}

int sx_send_enable_ib_swid_events(struct sx_dev *dev, u8 swid)
{
    int                 i;
    int                 err = 0;
    union sx_event_data event_data;
    u8                  first_ib_swid;
        
    if (dev->profile.swid_type[swid] == SX_KU_L2_TYPE_IB ) {
        memset(&event_data, 0, sizeof(event_data));
        event_data.ib_swid_change.swid = swid;
        event_data.ib_swid_change.dev_id = dev->device_id;
        sx_core_dispatch_event(dev, SX_DEV_EVENT_IB_SWID_UP, &event_data);

        spin_lock(&dev->profile_lock);        
        first_ib_swid = dev->first_ib_swid;
        dev->first_ib_swid = 0;
        spin_unlock(&dev->profile_lock);
        if (first_ib_swid) {
            union sx_event_data tca_init_event_data;

            memset(&tca_init_event_data, 0, sizeof(tca_init_event_data));
            for (i = 0; i < NUMBER_OF_SWIDS; i++) {
                if (dev->profile.ipoib_router_port_enable[i]) {
                    tca_init_event_data.tca_init.swid[tca_init_event_data.tca_init.num_of_ib_swids++] = i;
                }
            }

            if (tca_init_event_data.tca_init.num_of_ib_swids != 0) {
                tca_init_event_data.tca_init.max_pkey = dev->profile.max_pkey;
                sx_core_dispatch_event(dev, SX_DEV_EVENT_TYPE_TCA_INIT, &tca_init_event_data);
            }
        }
    }
    else{
        printk(KERN_INFO PFX "Error: try to send IB_SWID_UP event on swid %d from non-IB type %d, ",
               swid, dev->profile.swid_type[swid]);
        err = -EINVAL;
    }
    
    return err;   
}


int sx_enable_swid(struct sx_dev *dev, int sx_dev_id, u8 swid, int synd, u64 mac)
{
    int                 i;
    int                 err = 0;
    u8                  dqn;
    u32                 dq_bitmap = 0;
    struct sx_dq       *dq;
    union sx_event_data event_data;
    unsigned long       flags;
    u8                  dev_profile_set;
    u8                  first_ib_swid;

    /* IF PCI-E path is not valid, no need to open DQs */
    if (!sx_dpt_is_path_valid(sx_dev_id, DPT_PATH_PCI_E)) {
        printk(KERN_INFO PFX "PCIe path is not valid for device %u, "
               "will not open DQs\n",
               sx_dev_id);
        goto send_events;
    }

    /* TODO: handle errors */
    for (i = 0; i < NUMBER_OF_ETCLASSES; i++) {
        dqn = dev->profile.tx_prof[swid][i].sdq;
        spin_lock_irqsave(&sx_priv(dev)->sdq_table.lock, flags);
        dq = sx_priv(dev)->sdq_table.dq[dqn];
        spin_unlock_irqrestore(&sx_priv(dev)->sdq_table.lock, flags);
        if (!dq) {
            err = sx_core_create_sdq(dev,
                                     1 << dev->dev_cap.log_max_sdq_sz, dqn, &dq);
            if (err) {
                goto out;
            }

            dq_bitmap |= (1 << dqn);
            /* We only want to increase the refcount if the dq is in use in another swid */
        } else if (!(dq_bitmap & (1 << dqn))) {
            atomic_inc(&dq->refcount);
        }
    }

    dq_bitmap = 0;
    for (i = 0; i < dev->profile.rdq_count[swid]; i++) {
        dqn = dev->profile.rdq[swid][i];
        spin_lock_irqsave(&sx_priv(dev)->rdq_table.lock, flags);
        dq = sx_priv(dev)->rdq_table.dq[dqn];
        spin_unlock_irqrestore(&sx_priv(dev)->rdq_table.lock, flags);
        if (!dq) {
            err = sx_core_create_rdq(dev, RDQ_NUMBER_OF_ENTRIES, dqn, &dq);
            if (err) {
                goto out;
            }

            dq_bitmap |= (1 << dqn);
        } else if (!(dq_bitmap & (1 << dqn))) {
            atomic_inc(&dq->refcount);
        }
    }

send_events:
    memset(&event_data, 0, sizeof(event_data));   
    spin_lock(&dev->profile_lock);
    dev_profile_set = dev->dev_profile_set;
    spin_unlock(&dev->profile_lock);
    if ( (dev->profile.swid_type[swid] == SX_KU_L2_TYPE_IB) &&
         dev_profile_set ) {        
        event_data.ib_swid_change.swid = swid;
        event_data.ib_swid_change.dev_id = sx_dev_id;
        sx_core_dispatch_event(dev, SX_DEV_EVENT_IB_SWID_UP, &event_data);

        spin_lock(&dev->profile_lock);
        first_ib_swid = dev->first_ib_swid;
        dev->first_ib_swid = 0;
        spin_unlock(&dev->profile_lock);
        if (first_ib_swid) {
            union sx_event_data tca_init_event_data;

            memset(&tca_init_event_data, 0, sizeof(tca_init_event_data));
            for (i = 0; i < NUMBER_OF_SWIDS; i++) {
                if (dev->profile.ipoib_router_port_enable[i]) {
                    tca_init_event_data.tca_init.swid[tca_init_event_data.tca_init.num_of_ib_swids++] = i;
                }
            }

            if (tca_init_event_data.tca_init.num_of_ib_swids != 0) {
                tca_init_event_data.tca_init.max_pkey = dev->profile.max_pkey;
                sx_core_dispatch_event(dev, SX_DEV_EVENT_TYPE_TCA_INIT, &tca_init_event_data);
            }
        }
    } else if (dev->profile.swid_type[swid] == SX_KU_L2_TYPE_ETH) {
        sx_priv(dev)->swid_data[swid].eth_swid_data.synd = synd;
        sx_priv(dev)->swid_data[swid].eth_swid_data.mac = mac;
        event_data.eth_swid_up.swid = swid;
        event_data.eth_swid_up.synd = synd;
        event_data.eth_swid_up.mac = mac;
        sx_core_dispatch_event(dev, SX_DEV_EVENT_ETH_SWID_UP, &event_data);
    }    

    sx_bitmap_set(&sx_priv(dev)->swid_bitmap, swid);
    if (sx_priv(dev)->cq_table.cq_credit_thread &&
        !sx_priv(dev)->cq_table.credit_thread_active) {
        wake_up_process(sx_priv(dev)->cq_table.cq_credit_thread);
        if (cq_thread_sched_priority != 0) {
            struct sched_param param = { .sched_priority = cq_thread_sched_priority };

            err = sched_setscheduler(sx_priv(dev)->cq_table.cq_credit_thread,
                                     SCHED_FIFO, &param);
            if (err) {
                printk(KERN_INFO PFX "Failed setting RT prio %d to the "
                       "cq_credit_thread, err = %d\n",
                       cq_thread_sched_priority, err);
            } else {
                printk(KERN_INFO PFX "Successfully set the real time priority of the "
                       "cq_credit_thread to %d\n", cq_thread_sched_priority);
            }
        }

        sx_priv(dev)->cq_table.credit_thread_active = 1;
    }

out:
    return err;
}

void sx_disable_swid(struct sx_dev *dev, u8 swid)
{
    int                 i;
    u8                  dqn;
    struct sx_dq       *dq;
    union sx_event_data event_data;
    u8                  dev_profile_set;

    for (i = 0; i < NUMBER_OF_ETCLASSES; i++) {
        dqn = dev->profile.tx_prof[swid][i].sdq;
        dq = sx_priv(dev)->sdq_table.dq[dqn];
        if (dq && (atomic_read(&dq->refcount) == 1)) {
            sx_core_destroy_sdq(dev, dq);
            sx_priv(dev)->sdq_table.dq[dqn] = NULL;
        } else if (dq) {
            atomic_dec(&dq->refcount);
        }
    }

    for (i = 0; i < dev->profile.rdq_count[swid]; i++) {
        dqn = dev->profile.rdq[swid][i];
        dq = sx_priv(dev)->rdq_table.dq[dqn];
        if (dq && (atomic_read(&dq->refcount) == 1)) {
            sx_core_destroy_rdq(dev, dq);
            sx_priv(dev)->rdq_table.dq[dqn] = NULL;
        } else if (dq) {
            atomic_dec(&dq->refcount);
        }
    }

    spin_lock(&dev->profile_lock);
    dev_profile_set = dev->dev_profile_set;
    spin_unlock(&dev->profile_lock);
    event_data.ib_swid_change.swid = swid;
    event_data.ib_swid_change.dev_id = dev->profile.dev_id;
    if (dev->profile.swid_type[swid] == SX_KU_L2_TYPE_IB && dev_profile_set ) {
        sx_core_dispatch_event(dev, SX_DEV_EVENT_IB_SWID_DOWN, &event_data);
    } else if (dev->profile.swid_type[swid] == SX_KU_L2_TYPE_ETH) {
        sx_core_dispatch_event(dev, SX_DEV_EVENT_ETH_SWID_DOWN, &event_data);
    }

    sx_bitmap_free(&sx_priv(dev)->swid_bitmap, swid);
}

/*
 *  Workaround for rdq stucked because of wqe_too_short error.
 *  Force rdq size to be SX_MAX_MSG_SIZE on SX
 */
void __sx_adjust_rdq_size(struct sx_dev *dev)
{
    int i;

    if ((sx_glb.profile.chip_type == SXD_CHIP_TYPE_UNKNOWN) ||
        (sx_glb.profile.chip_type == SXD_CHIP_TYPE_SWITCHX_A1) ||
        (sx_glb.profile.chip_type == SXD_CHIP_TYPE_SWITCHX_A2)) {
        for (i = 0; i < NUMBER_OF_RDQS; i++) {
            if (dev->profile.rdq_properties[i].entry_size != 0) {
                dev->profile.rdq_properties[i].entry_size = SX_MAX_MSG_SIZE;
            }
        }
    }
}

int sx_handle_set_profile(struct sx_dev *dev)
{
    int           err = 0;
    struct sx_dq *sdq, *rdq;
    u8            dqn;

    __sx_adjust_rdq_size(dev);

    err = check_valid_profile(dev, &dev->profile);
    if (err) {
        printk(KERN_ERR PFX "input profile is not valid\n");
        goto out;
    }

    if (!sx_dpt_is_path_valid(dev->profile.dev_id, DPT_PATH_PCI_E)) {
        printk(KERN_INFO PFX "PCIe path is not valid for device %u, "
               "will not open EMAD DQs\n", dev->profile.dev_id);
        goto out;
    }

    /* no need to open EMAD RDQ on IB only systems */
    if ((dev->profile.pci_profile >= PCI_PROFILE_EN_SINGLE_SWID) &&
        (sx_dpt_get_emad_path(dev->device_id) == DPT_PATH_PCI_E)) {
        dqn = dev->profile.emad_tx_prof.sdq;
        err = sx_core_create_sdq(dev, 1 << dev->dev_cap.log_max_sdq_sz, dqn, &sdq);
        if (err) {
            printk(KERN_ERR PFX "create EMAD sdq %d failed. err: %d\n",
                   dqn, err);
            goto out;
        }

        dqn = dev->profile.emad_rdq;
        err = sx_core_create_rdq(dev, RDQ_NUMBER_OF_ENTRIES, dqn, &rdq);
        if (err) {
            printk(KERN_ERR PFX "create EMAD rdq %d failed. err: %d\n",
                   dqn, err);
            goto out;
        }
    }

out: return err;
}

static int sx_core_get_hw_etclass_impl(struct isx_meta *meta, u8* hw_etclass)
{
    /* According to the SX PRM
     * etclass should be set to the following value: (7 - Egress Tclass) */

    *hw_etclass = 7 - meta->etclass;

    return 0;
}

static int sx_core_get_hw_etclass_impl_spectrum(struct isx_meta *meta, u8* hw_etclass)
{
    /* According to the SX PRM
     * etclass should be set to the following value: (7 - Egress Tclass) */

    *hw_etclass = meta->etclass;

    return 0;
}

static int sx_core_get_send_to_rp_as_data_supported(u8* send_to_rp_as_data_supported)
{
    *send_to_rp_as_data_supported = false;

    return 0;
}

static int sx_core_get_send_to_rp_as_data_supported_spectrum(u8* send_to_rp_as_data_supported)
{
    *send_to_rp_as_data_supported = true;

    return 0;
}

static int get_rp_vid_from_db(struct sx_dev *dev, struct completion_info *comp_info, u16 *vlan_id)
{
    struct sx_priv *dev_priv = sx_priv(dev);
    uint16_t        local = 0;
    uint16_t        port_lag_id = comp_info->sysport;
    uint8_t         is_lag = comp_info->is_lag;

    *vlan_id = 0;

    if (is_lag) {
        *vlan_id = dev_priv->lag_rp_vid[port_lag_id];
    } else {
        local = dev_priv->system_to_local_db[port_lag_id];
        if (local > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Local %d is invalid. (MAX %d).\n",
                   local, MAX_PHYPORT_NUM);
            return -EINVAL;
        }
        *vlan_id = dev_priv->local_rp_vid[local];
    }

    return 0;
}

static int get_rp_vid_from_ci(struct sx_dev *dev, struct completion_info *comp_info, u16 *vlan_id)
{
    if (comp_info->is_tagged) {
        *vlan_id = comp_info->vid;
    }
    else {
        *vlan_id = 0;
    }

    return 0;
}

static int get_swid_from_db(struct sx_dev *dev, struct completion_info *comp_info, u8 *swid)
{
    enum sx_packet_type pkt_type = comp_info->pkt_type;
    u8 is_lag = comp_info->is_lag;
    u16 sysport_lag_id = comp_info->sysport;
    u16 lag_port_id = comp_info->lag_subport;

    u16 system_port, local_port, ib_port;

    switch (pkt_type) {
    case PKT_TYPE_ETH:
    case PKT_TYPE_FCoETH:
        if (is_lag) {
            u16 lag_id = (sysport_lag_id >> 4) & 0xfff;
            local_port = sx_priv(dev)->lag_member_to_local_db[lag_id][lag_port_id];
        }
        else{
            system_port = sysport_lag_id;
            local_port = sx_priv(dev)->system_to_local_db[system_port];
        }

        break;

    case PKT_TYPE_IB_Raw: /* TODO: Extract qpn from IB Raw pkts */
    case PKT_TYPE_IB_non_Raw:
    case PKT_TYPE_FCoIB:
    case PKT_TYPE_ETHoIB:
        ib_port = (sysport_lag_id >> 4) & 0x7f;
        local_port = sx_priv(dev)->ib_to_local_db[ib_port];
        break;

    default:
        if (printk_ratelimit())
            printk(KERN_WARNING PFX "Received packet type is FC, "
                "and therefore unsupported right now\n");
        return 0;
    }

    *swid = sx_priv(dev)->local_to_swid_db[local_port];

    return 0;
}

static int get_swid_from_ci(struct sx_dev *dev, struct completion_info *comp_info, u8 *swid)
{
    *swid = comp_info->swid;

    return 0;
}

/* Used for SX */
static int sx_get_lag_mid(u16 lag_id, u16 *mid)
{
    if (mid)
        *mid = lag_id + 0xC000 + sx_glb.profile.max_mid;
    return 0;
}

/* Used for Spectrum */
static int sdk_get_lag_mid(u16 lag_id, u16 *mid)
{
    if (mid)
        *mid = lag_id + 0x100;
    return 0;
}

struct dev_specific_cb spec_cb_sx_a1 = {
    sx_core_get_hw_etclass_impl,      /* get_hw_etclass_cb   */
    sx_build_isx_header_v0,           /* sx_build_isx_header_cb */
    sx_get_sdq_from_profile,           /* sx_get_sdq_cb */
    sx_core_get_send_to_rp_as_data_supported,
    get_rp_vid_from_db,
    get_swid_from_db,
    sx_get_lag_mid,                /* get_lag_mid_cb */
#ifdef CONFIG_SX_SGMII_PRESENT
	sx_sgmii_build_cr_space_header_switchx
#endif
};
struct dev_specific_cb spec_cb_sx_a2 = {
    sx_core_get_hw_etclass_impl,      /* get_hw_etclass_cb   */
    sx_build_isx_header_v0,           /* sx_build_isx_header_cb */
    sx_get_sdq_from_profile,           /* sx_get_sdq_cb */
    sx_core_get_send_to_rp_as_data_supported,
    get_rp_vid_from_ci,
    get_swid_from_db,
    sx_get_lag_mid,                   /* get_lag_mid_cb */
#ifdef CONFIG_SX_SGMII_PRESENT
	sx_sgmii_build_cr_space_header_switchx
#endif
};
struct dev_specific_cb spec_cb_pelican = {
    sx_core_get_hw_etclass_impl_spectrum,      /* get_hw_etclass_cb   */
    sx_build_isx_header_v0,                    /* sx_build_isx_header_cb */
    sx_get_sdq_from_profile,                     /* sx_get_sdq_cb */
    sx_core_get_send_to_rp_as_data_supported,
    get_rp_vid_from_db,
    get_swid_from_db,
    NULL,                                      /* get_lag_mid_cb */
#ifdef CONFIG_SX_SGMII_PRESENT
	sx_sgmii_build_cr_space_header_spectrum
#endif
};
struct dev_specific_cb spec_cb_spectrum = {
    sx_core_get_hw_etclass_impl_spectrum,      /* get_hw_etclass_cb   */
    sx_build_isx_header_v1,                     /* sx_build_isx_header_cb */
    sx_get_sdq_per_traffic_type,                /* sx_get_sdq_cb */
    sx_core_get_send_to_rp_as_data_supported_spectrum,
    get_rp_vid_from_ci,
    get_swid_from_ci,
    sdk_get_lag_mid,                            /* get_lag_mid_cb */
#ifdef CONFIG_SX_SGMII_PRESENT
	sx_sgmii_build_cr_space_header_spectrum
#endif
};

static int sx_core_dev_init_switchx_cb(struct sx_dev *dev, enum sxd_chip_types chip_type)
{
    int           err = 0;
    unsigned long flags;

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);

    memset(&(sx_priv(dev)->dev_specific_cb), 0, sizeof(sx_priv(dev)->dev_specific_cb));

    /* init specific and common callbacks per device revision */
    switch (chip_type) {
    case SXD_CHIP_TYPE_SWITCHX_A0:
        sx_err(dev, "Cannot add device , the SDK does not support "
               "SwitchX with revision A0\n");
        err = -EINVAL;
        break;

    case SXD_CHIP_TYPE_SWITCHX_A1:
        /* for A1 revision add specific cb */
        sx_priv(dev)->dev_specific_cb = spec_cb_sx_a1;
        break;

    case SXD_CHIP_TYPE_SWITCHX_A2:
        sx_priv(dev)->dev_specific_cb = spec_cb_sx_a2;
        break;

    case SXD_CHIP_TYPE_SWITCH_IB:
    case SXD_CHIP_TYPE_SWITCH_IB2:
        /* for pelican/eagle add specific cb */
        sx_priv(dev)->dev_specific_cb = spec_cb_pelican;
        break;

    case SXD_CHIP_TYPE_SPECTRUM:
        /* for condor add specific cb */
        sx_priv(dev)->dev_specific_cb = spec_cb_spectrum;
        break;

    default:
        err = -EINVAL;
        sx_err(dev, "ERROR:hw_ver: 0x%x unsupported. \n",
               chip_type);
        break;
    }

    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);

    printk(KERN_DEBUG PFX "sx_core_dev_init_switchx_cb chip_type [%d]\n", chip_type);

    return err;
}

static void sx_skb_destructor(struct sk_buff *skb)
{
#ifndef NO_PCI
    struct sx_rsc *rsc;

    memcpy(&rsc, skb->cb, sizeof(rsc));
    up(&rsc->write_sem);
#endif
}

static int sx_flush_dq(struct sx_dev *dev, struct sx_dq *dq)
{
    int           err;
    unsigned long flags;
    unsigned long end;

    dq->is_flushing = 1;
    err = sx_dq_modify_2err(dev, dq);
    if (err) {
        sx_warn(dev, "Failed to modify dq to error. "
                "May cause resource leak\n");
        goto out_mod;
    }

    end = jiffies + 5 * HZ;
    spin_lock_irqsave(&dq->lock, flags);
    while ((int)(dq->head - dq->tail) > 0) {
        spin_unlock_irqrestore(&dq->lock, flags);
        msleep(1000 / HZ);
        if (time_after(jiffies, end)) {
            spin_lock_irqsave(&dq->lock, flags);
            break;
        }

        spin_lock_irqsave(&dq->lock, flags);
    }
    if ((int)(dq->head - dq->tail) > 0) {
        err = -ETIMEDOUT;
    } else {
        err = 0;
    }

    dq->is_flushing = 0;
    spin_unlock_irqrestore(&dq->lock, flags);

out_mod:
    return err;
}

static void sx_flush_dqs(struct sx_dev *dev, u8 send)
{
    struct sx_priv     *priv = sx_priv(dev);
    int                 err;
    int                 i;
    int                 max = send ? dev->dev_cap.max_num_sdqs : dev->dev_cap.max_num_rdqs;
    struct sx_dq_table *dq_table = send ?
                                   &priv->sdq_table : &priv->rdq_table;

    for (i = 0; i < max; i++) {
        if (dq_table->dq[i]) {
            err = sx_flush_dq(dev, dq_table->dq[i]);
            if (err && dq_table->dq[i]->is_send) {
                sx_warn(dev, "failed to flush dq %d. err %d\n", i, err);
            }

            sx_hw2sw_dq(dev, dq_table->dq[i]);
        }
    }
}

int sx_change_configuration(struct sx_dev *dev)
{
    int i;

    spin_lock(&dev->profile_lock);
    if (dev->profile_set == 1) {
        dev->profile_set = 0;
        spin_unlock(&dev->profile_lock);
        /* we unregister the device first, so sx_ib resources will be
         * cleaned (if there are such) because they might try to send
         * packets during the unregister process */
        sx_core_unregister_device(dev);
        if (dev->pdev) {
            sx_flush_dqs(dev, true);
            sx_core_destroy_sdq_table(dev, false);
            sx_flush_dqs(dev, false);
            sx_core_destroy_rdq_table(dev, false);
        }

        for (i = 0; i < NUMBER_OF_SWIDS; i++) {
            sx_bitmap_free(&sx_priv(dev)->swid_bitmap, i);
        }

        sx_core_register_device(dev);
        goto out;
    }

    spin_unlock(&dev->profile_lock);
out:
    return 0;
}

/************************************************
 *  Char device Functions
 ***********************************************/

/**
 * This function opens the device for file operations.
 * We support multiple file opens for the same device.
 * Initialize an event list per file and keep it in the file context.
 * The maximum size of the event list is constant.
 * When the file is opened, we can't read or write to it until the
 * profile is set with ioctl
 *
 * param[in] inode - the associated inode.
 * param[in] filp - a pointer to the associated file.
 *
 * returns: 0 success
 *        !0 error
 */
static int sx_core_open(struct inode *inode, struct file *filp)
{
    struct sx_rsc *file = NULL;

#ifdef SX_DEBUG
    printk(KERN_DEBUG PFX " sx_core_open() \n");
#endif
    SX_CORE_UNUSED_PARAM(inode);
    file = kzalloc(sizeof(*file), GFP_KERNEL);
    if (!file) {
#ifdef SX_DEBUG
        printk(KERN_DEBUG PFX " sx_core_open() \n");
#endif
        return -ENOMEM;
    }

    INIT_LIST_HEAD(&file->evlist.list);
    file->evlist_size = 0;
    spin_lock_init(&file->lock);
    init_waitqueue_head(&file->poll_wait);
    atomic_set(&file->multi_packet_read_enable, false);
    atomic_set(&file->read_blocking_state, true);
    sema_init(&file->write_sem, SX_WRITE_LIMIT);
    filp->private_data = file; /* connect the fd with its resources */

    return 0;
}


/**
 * Send packets - EMADs, Ethernet packets. We copy the packets
 * from user space, as is, and post them to the HW SDQ. The
 * header is built by user space. SDQ number is calculated
 * according to the profile. Buf is formatted according to
 * ku_write struct. Count is the size of ku_write buffer
 * (without the packets data). We support sending multiple
 * packets in a single operation.
 *
 * param[in] filp  - a pointer to the associated file
 * param[in] buf   - ku_write struct/s
 * param[in] count - the number of bytes to send
 * param[in] pos   - not in use
 *
 *
 * returns: 0<=res - The size of the given buffer that was written
 *          res<0  - Error
 */
static ssize_t sx_core_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos)
{
    struct ku_write write_data;
    int             err = 0;
    int             user_buffer_copied_size = 0;
    struct sk_buff *skb = NULL;
    struct sx_dev  *dev = NULL;
    struct sx_rsc  *rsc = filp->private_data;
    int             i;

    if ((count == 0) || (buf == NULL)) {
        err = -EINVAL;
        goto out;
    }

    while (user_buffer_copied_size + sizeof(write_data) <= count) {
        err = copy_from_user((void*)&write_data,
                             ((void*)buf) + user_buffer_copied_size,
                             sizeof(write_data));
        if (err) {
            goto out;
        }

        if (((write_data.vec_entries != 0) && (write_data.iov == NULL)) ||
            ((write_data.vec_entries == 0) && (write_data.iov != NULL))) {
            err = -EINVAL;
            goto out;
        }

        if (write_data.vec_entries == 0) {
            break;
        }

        err = sx_dpt_get_sx_dev_by_id(write_data.meta.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_write: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }
#if 0
        if (dev && !dev->profile_set) {
            printk(KERN_WARNING PFX "sx_core_write() cannot "
                   "execute because the profile is not "
                   "set\n");
            err = -ENOEXEC;
            goto out;
        }
#endif
        err = check_valid_meta(dev, &write_data.meta);
        if (err) {
            printk(KERN_WARNING PFX "Cannot execute because meta "
                   "is invalid\n");
            goto out;
        }

        if (write_data.meta.type == SX_PKT_TYPE_LOOPBACK_CTL) {
            err = sx_send_loopback(dev, &write_data, filp);
            if (err) {
                printk(KERN_WARNING PFX "sx_core_write: "
                       "Failed seding loopback packet\n");
                goto out;
            }

			if (dev) {
				loopback_packets_counter++;
            	dev->loopback_packets_counter++;
			}
			else if (sx_glb.tmp_dev_ptr) {
                sx_glb.tmp_dev_ptr->loopback_packets_counter++;
			}
            user_buffer_copied_size += sizeof(write_data);
            continue;
        }

        /* according to the PRM, emads should get "any ethernet swid" */
        if ((write_data.meta.type == SX_PKT_TYPE_DROUTE_EMAD_CTL) ||
            (write_data.meta.type == SX_PKT_TYPE_EMAD_CTL)) {
            if (!dev || !dev->profile_set) {
                write_data.meta.swid = 0;
            } else {
                for (i = 0; i < NUMBER_OF_SWIDS; i++) {
                    if (dev->profile.swid_type[i] ==
                        SX_KU_L2_TYPE_ETH) {
                        write_data.meta.swid = i;
                        break;
                    }
                }

                if (i == NUMBER_OF_SWIDS) { /* no ETH swids found */
                    printk(KERN_WARNING PFX "sx_core_write: Err: "
                           "trying to send an emad from "
                           "an IB only system\n");
                    err = -EFAULT;
                    write_data.meta.swid = 0;
                    goto out;
                }
            }
        }

        user_buffer_copied_size += sizeof(write_data);
        err = copy_buff_to_skb(&skb, &write_data, true);
        if (err) {
            goto out;
        }

        memcpy(skb->cb, &rsc, sizeof(rsc));
        skb->destructor = sx_skb_destructor;
#ifndef NO_PCI
        down(&rsc->write_sem);
#endif
        err = sx_core_post_send(dev, skb, &write_data.meta);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_write: got error"
                   " from sx_core_post_send\n");
            /* we don't free the packet because sx_core_post_send free
             * the packet in case of an error */
            goto out;
        }
    }

    SX_CORE_UNUSED_PARAM(pos);
    return user_buffer_copied_size;

out:
#ifdef SX_DEBUG
    printk(KERN_DEBUG PFX "sx_core_write: return "
           "value is %d (error)\n", err);
#endif
    return err;
}


/**
 * Read a bulk of packets up to count size (count includes packet and metadata)
 * Format to buf according to ku_en_read struct.
 * Read packets from the file event list.
 * The event list holds pointers to cloned skbs.
 * On each read, we free the skb.
 * If multi-packets mode (see ioctl) is enabled,
 * we read multiple packets up to count.
 * If multi-packets mode is disabled, we read a single packet up to count.
 *
 * param[in] filp  - a pointer to the associated file
 * param[in] buf   - the data buffer, ku_read struct
 * param[in] count - the number of bites to send (the size of the buffer)
 * param[in] pos   - not in use
 *
 *
 * returns: The size of data which copied to the user buffer
 *	   !0 error
 */
static ssize_t sx_core_read(struct file *filp, char __user *buf, size_t count, loff_t *pos)
{
    unsigned long     flags;
    struct event_data edata_list;
    struct sx_rsc    *file = filp->private_data;
    int               multi_packet_read_enable = false;
    int               read_blocking_state = true;
    int               err;

    SX_CORE_UNUSED_PARAM(pos);
#ifdef SX_DEBUG
    printk(KERN_DEBUG PFX " sx_core_read()\n");
#endif
    if ((count == 0) || (buf == NULL)) {
        return -EINVAL;
    }

    spin_lock_irqsave(&file->lock, flags);
    while (list_empty(&file->evlist.list)) {
        spin_unlock_irqrestore(&file->lock, flags);
        read_blocking_state = atomic_read(&file->read_blocking_state);
        if (read_blocking_state == false) { /* non-blocking */
            return -EAGAIN;
        }

        if (wait_event_interruptible(file->poll_wait,
                                     !list_empty(&file->evlist.list))) {
            return -ERESTARTSYS;
        }

        spin_lock_irqsave(&file->lock, flags);
    }

    multi_packet_read_enable = atomic_read(&file->multi_packet_read_enable);
    INIT_LIST_HEAD(&edata_list.list);
    err = get_edata_from_elist(&file->evlist_size, &edata_list,
                         &file->evlist.list, count, multi_packet_read_enable);
    spin_unlock_irqrestore(&file->lock, flags);

    /* 	
        not enough room for single packet (meta + packet)
	but enough room for meta
    */
    if (err > 0) {
        struct ku_read 	metadata;

        memset(&metadata, 0, sizeof(metadata));
        metadata.length = err;
        if (copy_to_user(buf, &metadata, sizeof(metadata)))
            return -EFAULT;
        return 0;
    }
    /* not enough room for meta */
    else if (err < 0) {	
        return err;
    }

    return copy_edata_to_user(&edata_list, buf);
}

static int sx_core_handle_access_reg_ioctl(unsigned int cmd, unsigned long data)
{
    int            err = 0;
    struct sx_dev *dev = NULL;

    switch (cmd) {
    case CTRL_CMD_ACCESS_REG_PSPA:
    {
        struct ku_access_pspa_reg pspa_reg_data;

        err = copy_from_user(&pspa_reg_data, (void*)data,
                             sizeof(pspa_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(pspa_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg PSPA: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_PSPA(dev, &pspa_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &pspa_reg_data,
                           sizeof(pspa_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_QSPTC:
    {
        struct ku_access_qsptc_reg qsptc_reg_data;

        err = copy_from_user(&qsptc_reg_data, (void*)data,
                             sizeof(qsptc_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(qsptc_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg QSPTC: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_QSPTC(dev, &qsptc_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &qsptc_reg_data,
                           sizeof(qsptc_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_QSTCT:
    {
        struct ku_access_qstct_reg qstct_reg_data;

        err = copy_from_user(&qstct_reg_data, (void*)data,
                             sizeof(qstct_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(qstct_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg QSTCT: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_QSTCT(dev, &qstct_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &qstct_reg_data,
                           sizeof(qstct_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PTYS:
    {
        struct ku_access_ptys_reg ptys_reg_data;

        err = copy_from_user(&ptys_reg_data, (void*)data,
                             sizeof(ptys_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(ptys_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg PTYS: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_PTYS(dev, &ptys_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &ptys_reg_data,
                           sizeof(ptys_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MHSR:
    {
        struct ku_access_mhsr_reg mhsr_reg_data;

        err = copy_from_user(&mhsr_reg_data, (void*)data,
                             sizeof(mhsr_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mhsr_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg MHSR: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_MHSR(dev, &mhsr_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mhsr_reg_data,
                           sizeof(mhsr_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PMLP:
    {
        struct ku_access_pmlp_reg pmlp_reg_data;

        err = copy_from_user(&pmlp_reg_data, (void*)data,
                             sizeof(pmlp_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(pmlp_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg PMLP: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_PMLP(dev, &pmlp_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &pmlp_reg_data,
                           sizeof(pmlp_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PLIB:
    {
        struct ku_access_plib_reg plib_reg_data;

        err = copy_from_user(&plib_reg_data, (void*)data,
                             sizeof(plib_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(plib_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg PLIB: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_PLIB(dev, &plib_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &plib_reg_data,
                           sizeof(plib_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_SPZR:
    {
        struct ku_access_spzr_reg spzr_reg_data;
        union sx_event_data       event_data;

        err = copy_from_user(&spzr_reg_data, (void*)data,
                             sizeof(spzr_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(spzr_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg SPZR: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_SPZR(dev, &spzr_reg_data);
        if (err) {
            goto out;
        }

        /* If the user updates the node description of the local device
        * through this path we should notify sx_ib about this change */
        if (spzr_reg_data.spzr_reg.ndm && (spzr_reg_data.op_tlv.method == 2) &&
            (sx_glb.sx_dpt.dpt_info[spzr_reg_data.dev_id].cmd_path == DPT_PATH_PCI_E)) {
            event_data.node_desc_update.swid = spzr_reg_data.spzr_reg.swid;
            memcpy(event_data.node_desc_update.NodeDescription, spzr_reg_data.spzr_reg.NodeDescription, 64);
            sx_core_dispatch_event(dev, SX_DEV_EVENT_NODE_DESC_UPDATE, &event_data);
        }

        err = copy_to_user((void*)data, &spzr_reg_data,
                           sizeof(spzr_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PAOS:
    {
        struct ku_access_paos_reg paos_reg_data;

        err = copy_from_user(&paos_reg_data, (void*)data,
                             sizeof(paos_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(paos_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg PAOS: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_PAOS(dev, &paos_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &paos_reg_data,
                           sizeof(paos_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PLPC:
     {
         struct ku_access_plpc_reg plpc_reg_data;

         err = copy_from_user(&plpc_reg_data, (void*)data,
                              sizeof(plpc_reg_data));
         if (err) {
             goto out;
         }

         err = sx_dpt_get_cmd_sx_dev_by_id(plpc_reg_data.dev_id, &dev);
         if (err) {
             printk(KERN_WARNING PFX "sx_core_access_reg PLPC: "
                    "Device doesn't exist. Aborting\n");
             goto out;
         }

         err = sx_ACCESS_REG_PLPC(dev, &plpc_reg_data);
         if (err) {
             goto out;
         }

         err = copy_to_user((void*)data, &plpc_reg_data,
                            sizeof(plpc_reg_data));
         if (err) {
             goto out;
         }
         break;
     }

    case CTRL_CMD_ACCESS_REG_PPLM:
     {
         struct ku_access_pplm_reg pplm_reg_data;

         err = copy_from_user(&pplm_reg_data, (void*)data,
                              sizeof(pplm_reg_data));
         if (err) {
             goto out;
         }

         err = sx_dpt_get_cmd_sx_dev_by_id(pplm_reg_data.dev_id, &dev);
         if (err) {
             printk(KERN_WARNING PFX "sx_core_access_reg PPLM: "
                    "Device doesn't exist. Aborting\n");
             goto out;
         }

         err = sx_ACCESS_REG_PPLM(dev, &pplm_reg_data);
         if (err) {
             goto out;
         }

         err = copy_to_user((void*)data, &pplm_reg_data,
                            sizeof(pplm_reg_data));
         if (err) {
             goto out;
         }
         break;
     }

    case CTRL_CMD_ACCESS_REG_PMPC:
    {
        struct ku_access_pmpc_reg pmpc_reg_data;

        err = copy_from_user(&pmpc_reg_data, (void*)data,
                             sizeof(pmpc_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(pmpc_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg PMPC: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_PMPC(dev, &pmpc_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &pmpc_reg_data,
                           sizeof(pmpc_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PPSC:
    {
        struct ku_access_ppsc_reg ppsc_reg_data;

        err = copy_from_user(&ppsc_reg_data, (void*)data,
                             sizeof(ppsc_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(ppsc_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg PPSC: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_PPSC(dev, &ppsc_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &ppsc_reg_data,
                           sizeof(ppsc_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PMPR:
    {
        struct ku_access_pmpr_reg pmpr_reg_data;

        err = copy_from_user(&pmpr_reg_data, (void*)data,
                             sizeof(pmpr_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(pmpr_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg PMPR: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_PMPR(dev, &pmpr_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &pmpr_reg_data,
                           sizeof(pmpr_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PMTU:
    {
        struct ku_access_pmtu_reg pmtu_reg_data;

        err = copy_from_user(&pmtu_reg_data, (void*)data,
                             sizeof(pmtu_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(pmtu_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg PMTU: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_PMTU(dev, &pmtu_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &pmtu_reg_data,
                           sizeof(pmtu_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PELC:
    {
        struct ku_access_pelc_reg pelc_reg_data;

        err = copy_from_user(&pelc_reg_data, (void*)data,
                             sizeof(pelc_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(pelc_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg PELC: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_PELC(dev, &pelc_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &pelc_reg_data,
                           sizeof(pelc_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PLBF:
    {
        struct ku_access_plbf_reg plbf_reg_data;

        err = copy_from_user(&plbf_reg_data, (void*)data,
                             sizeof(plbf_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(plbf_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg PLBF: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_PLBF(dev, &plbf_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &plbf_reg_data,
                           sizeof(plbf_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_SGCR:
    {
        struct ku_access_sgcr_reg sgcr_reg_data;

        err = copy_from_user(&sgcr_reg_data, (void*)data,
                             sizeof(sgcr_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(sgcr_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg SGCR: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_SGCR(dev, &sgcr_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &sgcr_reg_data,
                           sizeof(sgcr_reg_data));
        if (err) {
            goto out;
        }

        break;
    }

    case CTRL_CMD_ACCESS_REG_MSCI:
    {
        struct ku_access_msci_reg msci_reg_data;

        err = copy_from_user(&msci_reg_data, (void*)data,
                             sizeof(msci_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(msci_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg MSCI: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_MSCI(dev, &msci_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &msci_reg_data,
                           sizeof(msci_reg_data));
        if (err) {
            goto out;
        }

        break;
    }

    case CTRL_CMD_ACCESS_REG_SPAD:
    {
        struct ku_access_spad_reg spad_reg_data;

        err = copy_from_user(&spad_reg_data, (void*)data,
                             sizeof(spad_reg_data));
        if (err) {
            sx_err(dev, "CTRL_CMD_ACCESS_REG_SPAD: copy_from_user failed");
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(spad_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg SPAD: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_SPAD(dev, &spad_reg_data);
        if (err) {
            sx_err(dev, "CTRL_CMD_ACCESS_REG_SPAD: sx_ACCESS_REG_SPAD failed");
            goto out;
        }

        err = copy_to_user((void*)data, &spad_reg_data,
                           sizeof(spad_reg_data));
        if (err) {
            sx_err(dev, "CTRL_CMD_ACCESS_REG_SPAD: copy_to_user failed");
            goto out;
        }

        break;
    }

    case CTRL_CMD_ACCESS_REG_HTGT:
    {
        struct ku_access_htgt_reg htgt_reg_data;

        err = copy_from_user(&htgt_reg_data, (void*)data,
                             sizeof(htgt_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(htgt_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg HTGT: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_HTGT(dev, &htgt_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &htgt_reg_data,
                           sizeof(htgt_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MFSC:
    {
        struct ku_access_mfsc_reg mfsc_reg_data;

        err = copy_from_user(&mfsc_reg_data, (void*)data,
                             sizeof(mfsc_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mfsc_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg MFSC: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_MFSC(dev, &mfsc_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mfsc_reg_data,
                           sizeof(mfsc_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MFSM:
    {
        struct ku_access_mfsm_reg mfsm_reg_data;

        err = copy_from_user(&mfsm_reg_data, (void*)data,
                             sizeof(mfsm_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mfsm_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg MSFS: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_MFSM(dev, &mfsm_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mfsm_reg_data,
                           sizeof(mfsm_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MFSL:
    {
        struct ku_access_mfsl_reg mfsl_reg_data;

        err = copy_from_user(&mfsl_reg_data, (void*)data,
                             sizeof(mfsl_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mfsl_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg MFSL: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_MFSL(dev, &mfsl_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mfsl_reg_data,
                           sizeof(mfsl_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PVLC:
    {
        struct ku_access_pvlc_reg pvlc_reg_data;

        err = copy_from_user(&pvlc_reg_data, (void*)data,
                             sizeof(pvlc_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(pvlc_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg PVLC: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_PVLC(dev, &pvlc_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &pvlc_reg_data,
                           sizeof(pvlc_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MCIA:
    {
        struct ku_access_mcia_reg mcia_reg_data;

        err = copy_from_user(&mcia_reg_data, (void*)data,
                             sizeof(mcia_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mcia_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg MCIA: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_MCIA(dev, &mcia_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mcia_reg_data,
                           sizeof(mcia_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_HPKT:
    {
        struct ku_access_hpkt_reg hpkt_reg_data;

        err = copy_from_user(&hpkt_reg_data, (void*)data,
                             sizeof(hpkt_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(hpkt_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg HPKT: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_HPKT(dev, &hpkt_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &hpkt_reg_data,
                           sizeof(hpkt_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_HCAP:
    {
        struct ku_access_hcap_reg hcap_reg_data;

        err = copy_from_user(&hcap_reg_data, (void*)data,
                             sizeof(hcap_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(hcap_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg HCAP: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_HCAP(dev, &hcap_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &hcap_reg_data,
                           sizeof(hcap_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_HDRT:
    {
        struct ku_access_hdrt_reg hdrt_reg_data;

        err = copy_from_user(&hdrt_reg_data, (void*)data,
                             sizeof(hdrt_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(hdrt_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg HDRT: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_HDRT(dev, &hdrt_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &hdrt_reg_data,
                           sizeof(hdrt_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_QPRT:
    {
        struct ku_access_qprt_reg qprt_reg_data;

        err = copy_from_user(&qprt_reg_data, (void*)data,
                             sizeof(qprt_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(qprt_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg QPRT: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_QPRT(dev, &qprt_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &qprt_reg_data,
                           sizeof(qprt_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MFCR:
    {
        struct ku_access_mfcr_reg mfcr_reg_data;

        err = copy_from_user(&mfcr_reg_data, (void*)data,
                             sizeof(mfcr_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mfcr_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg MFCR: "
                   "Device doesn't exist. Aborting\n");
            goto out;
        }

        err = sx_ACCESS_REG_MFCR(dev, &mfcr_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mfcr_reg_data,
                           sizeof(mfcr_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_FORE:
    {
        struct ku_access_fore_reg fore_reg_data;

        err = copy_from_user(&fore_reg_data, (void*)data,
                             sizeof(fore_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(fore_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_FORE(dev, &fore_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &fore_reg_data,
                           sizeof(fore_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MTCAP:
    {
        struct ku_access_mtcap_reg mtcap_reg_data;

        err = copy_from_user(&mtcap_reg_data, (void*)data,
                             sizeof(mtcap_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mtcap_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }
        err = sx_ACCESS_REG_MTCAP(dev, &mtcap_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mtcap_reg_data,
                           sizeof(mtcap_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MTMP:
    {
        struct ku_access_mtmp_reg mtmp_reg_data;

        err = copy_from_user(&mtmp_reg_data, (void*)data,
                             sizeof(mtmp_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mtmp_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_MTMP(dev, &mtmp_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mtmp_reg_data,
                           sizeof(mtmp_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MTWE:
    {
        struct ku_access_mtwe_reg mtwe_reg_data;

        err = copy_from_user(&mtwe_reg_data, (void*)data,
                             sizeof(mtwe_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mtwe_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_MTWE(dev, &mtwe_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mtwe_reg_data,
                           sizeof(mtwe_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MMDIO:
    {
        struct ku_access_mmdio_reg mmdio_reg_data;

        err = copy_from_user(&mmdio_reg_data, (void*)data,
                             sizeof(mmdio_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mmdio_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_MMDIO(dev, &mmdio_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mmdio_reg_data,
                           sizeof(mmdio_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MMIA:
    {
        struct ku_access_mmia_reg mmia_reg_data;

        err = copy_from_user(&mmia_reg_data, (void*)data,
                             sizeof(mmia_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mmia_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_MMIA(dev, &mmia_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mmia_reg_data,
                           sizeof(mmia_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MFPA:
    {
        struct ku_access_mfpa_reg mfpa_reg_data;

        err = copy_from_user(&mfpa_reg_data, (void*)data,
                             sizeof(mfpa_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mfpa_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_MFPA(dev, &mfpa_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mfpa_reg_data,
                           sizeof(mfpa_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MFBE:
    {
        struct ku_access_mfbe_reg mfbe_reg_data;

        err = copy_from_user(&mfbe_reg_data, (void*)data,
                             sizeof(mfbe_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mfbe_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_MFBE(dev, &mfbe_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mfbe_reg_data,
                           sizeof(mfbe_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MFBA:
    {
        struct ku_access_mfba_reg mfba_reg_data;

        err = copy_from_user(&mfba_reg_data, (void*)data,
                             sizeof(mfba_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mfba_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_MFBA(dev, &mfba_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mfba_reg_data,
                           sizeof(mfba_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_QCAP:
    {
        struct ku_access_qcap_reg qcap_reg_data;

        err = copy_from_user(&qcap_reg_data, (void*)data,
                             sizeof(qcap_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(qcap_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_QCAP(dev, &qcap_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &qcap_reg_data,
                           sizeof(qcap_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_RAW:
    {
        struct ku_access_raw_reg raw_reg_data;

        err = copy_from_user(&raw_reg_data, (void*)data,
                             sizeof(raw_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(raw_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_RAW(dev, &raw_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &raw_reg_data,
                           sizeof(raw_reg_data));
        if (err) {
            goto out;
        }

        break;
    }

    case CTRL_CMD_ACCESS_REG_RAW_BUFF:
    {
        struct ku_access_reg_raw_buff raw_buff_data;

        err = copy_from_user(&raw_buff_data, (void*)data,
                             sizeof(raw_buff_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(raw_buff_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_RAW_BUFF(dev, &raw_buff_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &raw_buff_data,
                           sizeof(raw_buff_data));
        if (err) {
            goto out;
        }

        break;
    }

    case CTRL_CMD_ACCESS_REG_PMAOS:
    {
        struct ku_access_pmaos_reg pmaos_reg_data;

        err = copy_from_user(&pmaos_reg_data, (void*)data,
                             sizeof(pmaos_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(pmaos_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_PMAOS(dev, &pmaos_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &pmaos_reg_data,
                           sizeof(pmaos_reg_data));
        if (err) {
            goto out;
        }

        break;
    }

    case CTRL_CMD_ACCESS_REG_MFM:
    {
        struct ku_access_mfm_reg mfm_reg_data;

        err = copy_from_user(&mfm_reg_data, (void*)data,
                             sizeof(mfm_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mfm_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_MFM(dev, &mfm_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mfm_reg_data,
                           sizeof(mfm_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MGIR:
    {
        struct ku_access_mgir_reg mgir_reg_data;

        err = copy_from_user(&mgir_reg_data, (void*)data,
                             sizeof(mgir_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mgir_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_MGIR(dev, &mgir_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mgir_reg_data,
                           sizeof(mgir_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_SSPR:
    {
        struct ku_access_sspr_reg sspr_reg_data;

        err = copy_from_user(&sspr_reg_data, (void*)data,
                             sizeof(sspr_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(sspr_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_SSPR(dev, &sspr_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &sspr_reg_data,
                           sizeof(sspr_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PPAD:
    {
        struct ku_access_ppad_reg ppad_reg_data;

        err = copy_from_user(&ppad_reg_data, (void*)data,
                             sizeof(ppad_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(ppad_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_PPAD(dev, &ppad_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &ppad_reg_data,
                           sizeof(ppad_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_SPMCR:
    {
        struct ku_access_spmcr_reg spmcr_reg_data;

        err = copy_from_user(&spmcr_reg_data, (void*)data,
                             sizeof(spmcr_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(spmcr_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_SPMCR(dev, &spmcr_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &spmcr_reg_data,
                           sizeof(spmcr_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PBMC:
    {
        struct ku_access_pbmc_reg pbmc_reg_data;

        err = copy_from_user(&pbmc_reg_data, (void*)data,
                             sizeof(pbmc_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(pbmc_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_PBMC(dev, &pbmc_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &pbmc_reg_data,
                           sizeof(pbmc_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_PPTB:
    {
        struct ku_access_pptb_reg pptb_reg_data;

        err = copy_from_user(&pptb_reg_data, (void*)data,
                             sizeof(pptb_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(pptb_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_PPTB(dev, &pptb_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &pptb_reg_data,
                           sizeof(pptb_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_SPVID:
    {
        struct ku_access_spvid_reg spvid_reg_data;

        err = copy_from_user(&spvid_reg_data, (void*)data,
                             sizeof(spvid_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(spvid_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_SPVID(dev, &spvid_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &spvid_reg_data,
                           sizeof(spvid_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_SFGC:
    {
        struct ku_access_sfgc_reg sfgc_reg_data;

        err = copy_from_user(&sfgc_reg_data, (void*)data,
                             sizeof(sfgc_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(sfgc_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_SFGC(dev, &sfgc_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &sfgc_reg_data,
                           sizeof(sfgc_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_OEPFT:
    {
        struct ku_access_oepft_reg oepft_reg_data;

        err = copy_from_user(&oepft_reg_data, (void*)data,
                             sizeof(oepft_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(oepft_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_OEPFT(dev, &oepft_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &oepft_reg_data,
                           sizeof(oepft_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MRSR:
    {
        struct ku_access_mrsr_reg mrsr_reg_data;

        err = copy_from_user(&mrsr_reg_data, (void*)data,
                             sizeof(mrsr_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mrsr_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_MRSR(dev, &mrsr_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mrsr_reg_data,
                           sizeof(mrsr_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    case CTRL_CMD_ACCESS_REG_MPSC:
    {
        struct ku_access_mpsc_reg mpsc_reg_data;

        err = copy_from_user(&mpsc_reg_data, (void*)data,
                             sizeof(mpsc_reg_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_get_cmd_sx_dev_by_id(mpsc_reg_data.dev_id, &dev);
        if (err) {
            printk(KERN_WARNING PFX "sx_core_access_reg , line %d "
                   "Device doesn't exist. Aborting\n", __LINE__);
            goto out;
        }

        err = sx_ACCESS_REG_MPSC(dev, &mpsc_reg_data);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &mpsc_reg_data,
                           sizeof(mpsc_reg_data));
        if (err) {
            goto out;
        }
        break;
    }

    default:
        return -EINVAL;
    }

out:

    return err;
}

/**
 * This function is used to perform some configuration commands
 * on the local device.
 *
 * param[in] inode - the associated inode
 * param[in] filp  - a pointer to the associated file
 * param[in] cmd   - the ioctl command to be performed
 * param[in] data  - a data to be passed to the invoked function
 *
 * returns: 0 success
 *	   !0 error
 */
static long sx_core_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
    struct sx_rsc            *file = filp->private_data;
    int                       err = 0;
    unsigned long             flags;
    struct sx_dev            *dev = NULL;
    struct ku_dpt_path_add    dpt_path_add_data;
    struct ku_dpt_path_modify dpt_path_modify_data;

    SX_CORE_UNUSED_PARAM(filp);

#ifdef SX_DEBUG
    printk(KERN_DEBUG PFX " sx_core_ioctl() cmd=[%d]\n", cmd);
#endif

#if 0 /* Not sure we need this check */
    spin_lock(&sx_glb.pci_devs_lock);
    if (list_empty(&sx_glb.pci_devs_list)) {
        spin_unlock(&sx_glb.pci_devs_lock);
        printk(KERN_WARNING PFX "sx_core_ioctl: "
               "NO PCI Device is present. Aborting\n");
        return -ENXIO;
    }

    spin_unlock(&sx_glb.pci_devs_lock);
#endif

    if ((cmd >= CTRL_CMD_ACCESS_REG_MIN) &&
        (cmd <= CTRL_CMD_ACCESS_REG_MAX)) {
        return sx_core_handle_access_reg_ioctl(cmd, data);
    }

    dev = sx_glb.tmp_dev_ptr; /* TODO: temporary, should use device id instead */
    if (!dev) {
        printk(KERN_WARNING PFX "sx_core_ioctl: "
               "Device doesn't exist. Aborting\n");
        return -ENXIO;
    }

    switch (cmd) {
    case CTRL_CMD_GET_CAPABILITIES:
        err = copy_to_user((void*)data, (void*)&dev->dev_cap,
                           sizeof(dev->dev_cap));
        if (err) {
            goto out;
        }
        break;

    case CTRL_CMD_RESET:
        err = sx_change_configuration(dev);
        if (err) {
            goto out;
        }
        break;

    case CTRL_CMD_PCI_DEVICE_RESTART:
    {
        struct sx_dev *tmp_dev = NULL, *curr_dev = NULL;
        u8             found = 0;

        printk(KERN_DEBUG PFX "ioctl device restart called "
               "for device %lu\n", data);
        spin_lock(&sx_glb.pci_devs_lock);
        list_for_each_entry_safe(curr_dev, tmp_dev, &sx_glb.pci_devs_list, list) {
            if (curr_dev->device_id == data) {
                found = 1;
                break;
            }
        }

        spin_unlock(&sx_glb.pci_devs_lock);
        if (!found) {
            sx_warn(dev, "ioctl device restart: the device "
                    "wasn't found\n");
            err = -ENODEV;
            goto out;
        }

        err = sx_restart_one_pci(curr_dev->pdev);
        if (err) {
            goto out;
        }

        break;
    }

    case CTRL_CMD_SET_PCI_PROFILE:
        printk(KERN_DEBUG PFX "ioctl set pci profile called\n");
        spin_lock(&dev->profile_lock);
        if (dev->profile_set == 1) {
            printk(KERN_WARNING PFX "Err: cannot set "
                   "profile twice\n");
            spin_unlock(&dev->profile_lock);
            err = -EINVAL;
            goto out;
        }

        dev->profile_set = 1;
        spin_unlock(&dev->profile_lock);

        err = copy_from_user((void*)&dev->profile,
                             (void*)data, sizeof(dev->profile));
        if (err) {
            err = -ENOMEM;
            goto out;
        }

        err = sx_handle_set_profile(dev);
        if (err) {
            spin_lock(&dev->profile_lock);
            dev->profile_set = 0;
            spin_unlock(&dev->profile_lock);
            goto out;
        }

        break;

    case CTRL_CMD_GET_PCI_PROFILE:
    {
        struct ku_get_pci_profile pci_prof;
        spin_lock(&dev->profile_lock);
        if (!dev->profile_set) {
            printk(KERN_WARNING PFX "Err: profile is not set\n");
            spin_unlock(&dev->profile_lock);
            err = -EINVAL;
            goto out;
        }

        pci_prof.pci_profile = dev->profile.pci_profile;
        spin_unlock(&dev->profile_lock);
        err = copy_to_user((struct ku_get_pci_profile *)data,
                           &pci_prof, sizeof(pci_prof));
        break;
    }

    case CTRL_CMD_ADD_SYND:
    case CTRL_CMD_REMOVE_SYND:
    {
        struct ku_synd_ioctl      ku;
        union ku_filter_critireas critireas;
        enum l2_type              listener_type = L2_TYPE_DONT_CARE;

        err = copy_from_user((void*)&ku, (void*)data, sizeof(ku));
        if (err) {
            goto out;
        }

        err = check_valid_ku_synd(&ku);
        if (err) {
            goto out;
        }

        memset(&critireas, 0, sizeof(critireas));

        switch (ku.type) {
        case SX_KU_L2_TYPE_DONT_CARE:
            listener_type = L2_TYPE_DONT_CARE;
            critireas.dont_care.sysport = ku.critireas.dont_care.sysport;
            break;

        case SX_KU_L2_TYPE_IB:
            listener_type = L2_TYPE_IB;
            critireas.ib.qpn = ku.critireas.ib.qpn;
            break;

        case SX_KU_L2_TYPE_ETH:
            listener_type = L2_TYPE_ETH;
            critireas.eth.dmac = ku.critireas.eth.dmac;
            critireas.eth.ethtype = ku.critireas.eth.ethtype;
            critireas.eth.emad_tid = ku.critireas.eth.emad_tid;
            critireas.eth.from_rp = ku.critireas.eth.from_rp;
            critireas.eth.from_bridge = ku.critireas.eth.from_bridge;
            break;

        case SX_KU_L2_TYPE_FC:
            listener_type = L2_TYPE_FC;
            critireas.fc.TBD = ku.critireas.fc.TBD;
            break;

        default:
            printk(KERN_ERR PFX "Err: invalid listener type : %d \n",
                   ku.type);
            err = -EINVAL;
            goto out;
        }
    if (ku.channel_type == SX_KU_USER_CHANNEL_TYPE_L2_NETDEV) {
		printk("%s: Adding SX_KU_USER_CHANNEL_TYPE_L2_NETDEV\n", __func__);
		if (dev->profile.swid_type[ku.swid] != SX_KU_L2_TYPE_ETH) {
			printk("%s: Adding L2 netdev on non ethernet device is not implemented\n", __func__);
			err = -EFAULT;
		}
		
		if (cmd == CTRL_CMD_ADD_SYND) 
			err = sx_core_add_synd_l2(ku.swid, ku.syndrome_num, dev);
		else 
			err = sx_core_remove_synd_l2(ku.swid, ku.syndrome_num, dev);

		if (err)
			goto out;
	} else if (ku.channel_type == SX_KU_USER_CHANNEL_TYPE_L3_NETDEV) {
            if (dev->profile.swid_type[ku.swid] == SX_KU_L2_TYPE_ETH) {
                /* L3 traps registration */
                if (cmd == CTRL_CMD_ADD_SYND) {
                    err = sx_core_add_synd_l3(ku.swid, ku.syndrome_num, dev);
                } else {
                    err = sx_core_remove_synd_l3(ku.swid, ku.syndrome_num, dev);
                }

                if (err) {
                    goto out;
                }
            } else {
                /* IPoIB traps registration */
                if (cmd == CTRL_CMD_ADD_SYND) {
                    err = sx_core_add_synd_ipoib(ku.swid, ku.syndrome_num, dev);
                } else {
                    err = sx_core_remove_synd_ipoib(ku.swid, ku.syndrome_num, dev);
                }

                if (err) {
                    goto out;
                }
            }
        } else if (ku.channel_type == SX_KU_USER_CHANNEL_TYPE_FD) {
            if ((ku.type == SX_KU_L2_TYPE_ETH) &&
                ((sx_glb.tmp_dev_ptr->profile.swid_type[ku.swid] == SX_KU_L2_TYPE_ETH) ||
                 (sx_glb.tmp_dev_ptr->profile.swid_type[ku.swid] == SX_KU_L2_TYPE_ROUTER_PORT))) {
                ku.swid = SWID_NUM_DONT_CARE;
            } else if ((ku.type == SX_KU_L2_TYPE_DONT_CARE) &&
                       (ku.critireas.dont_care.sysport == SYSPORT_DONT_CARE_VALUE)) {
                if ((sx_glb.tmp_dev_ptr->profile.swid_type[ku.swid] == SX_KU_L2_TYPE_ETH) ||
                    (sx_glb.tmp_dev_ptr->profile.swid_type[ku.swid] == SX_KU_L2_TYPE_ROUTER_PORT)) {
                    ku.swid = SWID_NUM_DONT_CARE;
                    ku.type = SX_KU_L2_TYPE_ETH;
                } else if (sx_glb.tmp_dev_ptr->profile.swid_type[ku.swid] == SX_KU_L2_TYPE_IB) {
                    ku.type = SX_KU_L2_TYPE_IB;
                }
            }

            if (cmd == CTRL_CMD_ADD_SYND) {
                err = sx_core_add_synd(ku.swid, ku.syndrome_num,
                                       listener_type, ku.is_default,
                                       critireas, sx_cq_handler, filp,
                                       CHECK_DUP_ENABLED_E, dev);
            } else {
                err = sx_core_remove_synd(ku.swid, ku.syndrome_num,
                                          listener_type, ku.is_default,
                                          critireas, filp, dev);
            }

            if (err) {
                goto out;
            }
        } else if (ku.channel_type == SX_KU_USER_CHANNEL_TYPE_L2_TUNNEL) {
            if (cmd == CTRL_CMD_ADD_SYND) {
                sx_priv(dev)->l2_tunnel_params = ku.l2_tunnel_params;
                err = sx_core_add_synd(ku.swid, ku.syndrome_num,
                                       listener_type, ku.is_default,
                                       critireas, sx_l2_tunnel_handler, dev,
                                       CHECK_DUP_DISABLED_E, dev);
            } else {
                err = sx_core_remove_synd(ku.swid, ku.syndrome_num,
                                          listener_type, ku.is_default,
                                          critireas, dev, dev);
            }

            if (err) {
                goto out;
            }
        } else {
            return -EINVAL;
        }

        break;
    }

    case CTRL_CMD_MULTI_PACKET_ENABLE:
        if ((data != false) && (data != true)) {
            printk(KERN_WARNING PFX "ioctl MULTI_PACKET_ENABLE: " \
                   "error data = %lu > 1\n", data);
            return -EINVAL;
        }

        atomic_set(&file->multi_packet_read_enable, data);
        break;

    case CTRL_CMD_BLOCKING_ENABLE:
        if ((data != false) && (data != true)) {
            return -EINVAL;
        }

        atomic_set(&file->read_blocking_state, data);
        break;

    case CTRL_CMD_RAISE_EVENT:
        /* filp is the listener context when registering through the char device */
        err = sx_raise_event(dev, (void*)data, filp);
        if (err) {
            goto out;
        }

        break;

    case CTRL_CMD_ENABLE_SWID:
    {
        struct ku_swid_details swid_data;

        err = copy_from_user(&swid_data, (void*)data,
                             sizeof(swid_data));
        if (err) {
            goto out;
        }

        printk(KERN_DEBUG PFX "ioctl enable_swid called with swid %d\n", swid_data.swid);
        spin_lock(&dev->profile_lock);
        if (!dev->profile_set || (swid_data.swid >= NUMBER_OF_SWIDS)
            || (dev->profile.swid_type[swid_data.swid] ==
                SX_KU_L2_TYPE_DONT_CARE) ||
            sx_bitmap_test(&sx_priv(dev)->swid_bitmap,
                           swid_data.swid)) {
            spin_unlock(&dev->profile_lock);
            return -EINVAL;
        }

        spin_unlock(&dev->profile_lock);
        err = sx_enable_swid(dev, swid_data.dev_id, swid_data.swid, swid_data.iptrap_synd, swid_data.mac);
        if (err) {
            goto out;
        }

        break;
    }

    case CTRL_CMD_DISABLE_SWID:
    {
        struct ku_swid_details swid_data;

        printk(KERN_DEBUG PFX "ioctl disable_swid called\n");
        err = copy_from_user(&swid_data, (void*)data,
                             sizeof(swid_data));
        if (err) {
            goto out;
        }

        spin_lock(&dev->profile_lock);
        if (!dev->profile_set || (swid_data.swid >= NUMBER_OF_SWIDS) ||
            !sx_bitmap_test(&sx_priv(dev)->swid_bitmap,
                            swid_data.swid)) {
            spin_unlock(&dev->profile_lock);
            return -EINVAL;
        }
        spin_unlock(&dev->profile_lock);
        sx_disable_swid(dev, swid_data.swid);
        break;
    }

    case CTRL_CMD_GET_SYNDROME_STATUS:
    {
        struct ku_synd_query_ioctl *synd_query = NULL;
        struct ku_synd_query_ioctl  tmp_synd_query;
        u8                          is_registered = 0;

        err = copy_from_user(&tmp_synd_query, (void*)data,
                             sizeof(tmp_synd_query));
        if (err) {
            goto out;
        }

        if (tmp_synd_query.syndrome_num > NUM_HW_SYNDROMES) {
            err = -EINVAL;
            goto out;
        }

        spin_lock_irqsave(&sx_glb.listeners_lock, flags);
        if (!list_empty(&sx_glb.listeners_db[tmp_synd_query.syndrome_num].list)) {
            is_registered = 1;
        }
        spin_unlock_irqrestore(&sx_glb.listeners_lock, flags);

        synd_query = (struct ku_synd_query_ioctl *)data;
        err = copy_to_user(&synd_query->is_registered, &is_registered,
                           sizeof(is_registered));
        if (err) {
            goto out;
        }

        break;
    }

    case CTRL_CMD_QUERY_FW:
    {
        struct ku_query_fw query_fw;
        err = copy_from_user(&query_fw, (struct ku_query_fw *)data,
                             sizeof(query_fw));
        if (err) {
            goto out;
        }

        err = sx_QUERY_FW(dev, &query_fw);
        if (err) {
            goto out;
        }
        err = copy_to_user((void*)data, &query_fw, sizeof(query_fw));
        if (err) {
            goto out;
        }

        break;
    }

    case CTRL_CMD_QUERY_BOARD_INFO:
    {
        struct ku_query_board_info board_info;

        board_info.vsd_vendor_id = dev->vsd_vendor_id;
        memcpy(board_info.board_id, dev->board_id,
               sizeof(dev->board_id));
        break;
    }

    case CTRL_CMD_SET_DEVICE_PROFILE:
    {
        struct ku_profile profile;
        struct ku_swid_config *swid_config_type_arr = 
                &profile.swid0_config_type;
        u8  swid;
        
        err = copy_from_user(&profile, (void*)data, sizeof(profile));
        if (err) {
            goto out;
        }

        if (profile.dev_id != DEFAULT_DEVICE_ID) {
            err = sx_SET_PROFILE(dev, &profile);
            if (err) {
                goto out;
            }
        }

        /*
         * if chip is PCI attached set the global profile and
         * create ib devices if needed
         */
        if (sx_dpt_is_dev_pci_attached(profile.dev_id)){
            sx_glb.profile = profile;
            
            /*
             * set events for all IB enabled swids if
             * swid is enabled
             * */
            for (swid=0; swid<NUMBER_OF_SWIDS; swid++){
                /* if swid isnt enable skip it */
                if (!sx_bitmap_test(&sx_priv(dev)->swid_bitmap, swid)) {
                    continue;
                }

                /* check the swid is IB in device profile */
                if (swid_config_type_arr[swid].type != KU_SWID_TYPE_INFINIBAND){
                    continue;
                }

                spin_lock(&dev->profile_lock);
                if (dev->dev_profile_set != 0) {
                    spin_unlock(&dev->profile_lock);
                    break;
                }
                dev->dev_profile_set = 1;
                spin_unlock(&dev->profile_lock);

                err = sx_send_enable_ib_swid_events(dev, swid);
                if (err) {
                    spin_lock(&dev->profile_lock);
                    dev->dev_profile_set = 0;
                    spin_unlock(&dev->profile_lock);
                    goto out;
                }
            }
        }

        break;
    }

    case CTRL_CMD_GET_DEVICE_PROFILE:
    {
        struct ku_profile profile;

        err = sx_GET_PROFILE(dev, &profile);
        if (err) {
            goto out;
        }

        err = copy_to_user((void*)data, &profile, sizeof(profile));
        if (err) {
            goto out;
        }

        break;
    }

    case CTRL_CMD_ADD_DEV_PATH:

        err = copy_from_user(&dpt_path_add_data, (void*)data,
                             sizeof(dpt_path_add_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_add_dev_path(dpt_path_add_data.dev_id,
                                  dpt_path_add_data.path_type,
                                  dpt_path_add_data.path_info,
                                  dpt_path_add_data.is_local);
        if (err) {
            goto out;
        }

        break;

    case CTRL_CMD_REMOVE_DEV_PATH:

        err = copy_from_user(&dpt_path_add_data, (void*)data,
                             sizeof(dpt_path_add_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_remove_dev_path(dpt_path_add_data.dev_id,
                                     dpt_path_add_data.path_type);
        if (err) {
            goto out;
        }
        break;

    case CTRL_CMD_REMOVE_DEV:

        err = copy_from_user(&dpt_path_add_data, (void*)data,
                             sizeof(dpt_path_add_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_remove_dev(dpt_path_add_data.dev_id, 0 );
        if (err) {
            goto out;
        }
        break;

    case CTRL_CMD_SET_CMD_PATH:

        err = copy_from_user(&dpt_path_modify_data, (void*)data,
                             sizeof(dpt_path_modify_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_set_cmd_path(dpt_path_modify_data.dev_id,
                                  dpt_path_modify_data.path_type);
        if (err) {
            goto out;
        }
        break;

    case CTRL_CMD_SET_EMAD_PATH:
        err = copy_from_user(&dpt_path_modify_data, (void*)data,
                             sizeof(dpt_path_modify_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_set_emad_path(dpt_path_modify_data.dev_id,
                                   dpt_path_modify_data.path_type);
        if (err) {
            goto out;
        }

        break;

    case CTRL_CMD_SET_MAD_PATH:
        err = copy_from_user(&dpt_path_modify_data, (void*)data,
                             sizeof(dpt_path_modify_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_set_mad_path(dpt_path_modify_data.dev_id,
                                  dpt_path_modify_data.path_type);
        if (err) {
            goto out;
        }

        break;

    case CTRL_CMD_SET_CR_ACCESS_PATH:
        err = copy_from_user(&dpt_path_modify_data, (void*)data,
                             sizeof(dpt_path_modify_data));
        if (err) {
            goto out;
        }

        err = sx_dpt_set_cr_access_path(dpt_path_modify_data.dev_id,
                                        dpt_path_modify_data.path_type);
        if (err) {
            goto out;
        }

        break;

    case CTRL_CMD_GET_SWID_2_RDQ:
    {
        struct ku_swid_2_rdq_query swid_2_rdq;

        if (!dev->profile_set) {
            return -EFAULT;
        }

        err = copy_from_user(&swid_2_rdq, (void*)data,
                             sizeof(swid_2_rdq));
        if (err) {
            goto out;
        }

        if ((dev->profile.swid_type[swid_2_rdq.swid] ==
             SX_KU_L2_TYPE_DONT_CARE) ||
            !sx_bitmap_test(&sx_priv(dev)->swid_bitmap,
                            swid_2_rdq.swid)) {
            err = -EINVAL;
            goto out;
        }

        swid_2_rdq.rdq = dev->profile.rdq[swid_2_rdq.swid][0];
        err = copy_to_user((void*)data, &swid_2_rdq,
                           sizeof(swid_2_rdq));
        if (err) {
            goto out;
        }

        break;
    }

    case CTRL_CMD_TRAP_FILTER_ADD:
    {
        struct ku_trap_filter_data filter_data;
        int                        i, idx = -1;

        err = copy_from_user(&filter_data, (void*)data,
                             sizeof(filter_data));
        if (err) {
            goto out;
        }

        if (filter_data.trap_id >= NUM_HW_SYNDROMES) {
            printk(KERN_ERR PFX "Received TRAP ID %d "
                   "is invalid\n", filter_data.trap_id);
            err = -EINVAL;
            goto out;
        }

        spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
        if (filter_data.is_lag) {
            if (filter_data.lag_id >= MAX_LAG_NUM) {
                printk(KERN_ERR PFX "Received LAG ID 0x%x "
                       "is invalid\n",
                       filter_data.lag_id);
                spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                err = -EINVAL;
                goto out;
            }

            for (i = 0; i < MAX_LAG_PORTS_IN_FILTER; i++) {
                if (sx_priv(dev)->lag_filter_db[filter_data.trap_id][i] ==
                    filter_data.lag_id) {
                    printk(KERN_ERR PFX "Received LAG ID %d "
                           "is already in the filter list "
                           "of trap ID 0x%x\n",
                           filter_data.lag_id,
                           filter_data.trap_id);
                    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                    err = -EEXIST;
                    goto out;
                }

                if (sx_priv(dev)->lag_filter_db[filter_data.trap_id][i] ==
                    LAG_ID_INVALID) {
                    idx = i;
                }
            }

            if (idx == -1) {
                printk(KERN_ERR PFX "Cannot add LAG ID 0x%x to trap %d filter "
                       "because DB filter DB for this trap is full\n",
                       filter_data.lag_id, filter_data.trap_id);
                err = -EFAULT;
            } else {
                sx_priv(dev)->lag_filter_db[filter_data.trap_id][idx] =
                    filter_data.lag_id;
                printk(KERN_INFO PFX "LAG ID %u was added to filter list "
                       "of trap ID 0x%x\n", filter_data.lag_id,
                       filter_data.trap_id);
            }
        } else {
            for (i = 0; i < MAX_SYSTEM_PORTS_IN_FILTER; i++) {
                if (sx_priv(dev)->sysport_filter_db[filter_data.trap_id][i] ==
                    filter_data.sysport) {
                    printk(KERN_ERR PFX "Received system port 0x%x "
                           "is already in the filter list "
                           "of trap ID 0x%x\n",
                           filter_data.sysport,
                           filter_data.trap_id);
                    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                    err = -EEXIST;
                    goto out;
                }

                if (sx_priv(dev)->sysport_filter_db[filter_data.trap_id][i] == 0) {
                    idx = i;
                }
            }

            if (idx == -1) {
                printk(KERN_ERR PFX "Cannot add system port 0x%x to trap %d filter "
                       "because DB filter DB for this trap is full\n",
                       filter_data.sysport, filter_data.trap_id);
                err = -EFAULT;
            } else {
                sx_priv(dev)->sysport_filter_db[filter_data.trap_id][idx] =
                    filter_data.sysport;
                printk(KERN_INFO PFX "system port 0x%x was added to filter "
                       "list of trap ID 0x%x\n", filter_data.sysport,
                       filter_data.trap_id);
            }
        }

        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        break;
    }

    case CTRL_CMD_TRAP_FILTER_REMOVE:
    {
        struct ku_trap_filter_data filter_data;
        int                        i;

        err = copy_from_user(&filter_data, (void*)data,
                             sizeof(filter_data));
        if (err) {
            goto out;
        }

        if (filter_data.trap_id >= NUM_HW_SYNDROMES) {
            printk(KERN_ERR PFX "Received TRAP ID %d "
                   "is invalid\n", filter_data.trap_id);
            err = -EINVAL;
            goto out;
        }

        spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
        if (filter_data.is_lag) {
            if (filter_data.lag_id >= MAX_LAG_NUM) {
                printk(KERN_ERR PFX "Received LAG ID 0x%x "
                       "is invalid\n",
                       filter_data.lag_id);
                spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                err = -EINVAL;
                goto out;
            }

            for (i = 0; i < MAX_LAG_PORTS_IN_FILTER; i++) {
                if (sx_priv(dev)->lag_filter_db[filter_data.trap_id][i] ==
                    filter_data.lag_id) {
                    sx_priv(dev)->lag_filter_db[filter_data.trap_id][i] =
                        LAG_ID_INVALID;
                    printk(KERN_INFO PFX "LAG ID %u was removed from filter list "
                           "of trap ID 0x%x\n", filter_data.lag_id,
                           filter_data.trap_id);
                    break;
                }
            }

            if (i == MAX_LAG_PORTS_IN_FILTER) {
                printk(KERN_ERR PFX "Cannot find LAG ID 0x%x in trap ID %d filter DB\n",
                       filter_data.lag_id, filter_data.trap_id);
                err = -EINVAL;
            }
        } else {
            for (i = 0; i < MAX_SYSTEM_PORTS_IN_FILTER; i++) {
                if (sx_priv(dev)->sysport_filter_db[filter_data.trap_id][i] ==
                    filter_data.sysport) {
                    sx_priv(dev)->sysport_filter_db[filter_data.trap_id][i] = 0;
                    printk(KERN_INFO PFX "system port 0x%x was removed from filter "
                           "list of trap ID 0x%x\n", filter_data.sysport,
                           filter_data.trap_id);
                    break;
                }
            }

            if (i == MAX_SYSTEM_PORTS_IN_FILTER) {
                printk(KERN_ERR PFX "Cannot find system port 0x%x in trap ID %d "
                       "filter DB\n", filter_data.sysport, filter_data.trap_id);
                err = -EINVAL;
            }
        }

        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        break;
    }

    case CTRL_CMD_TRAP_FILTER_REMOVE_ALL:
    {
        struct ku_trap_filter_data filter_data;
        int                        i;

        err = copy_from_user(&filter_data, (void*)data,
                             sizeof(filter_data));
        if (err) {
            goto out;
        }

        if (filter_data.trap_id >= NUM_HW_SYNDROMES) {
            printk(KERN_ERR PFX "Received TRAP ID %d "
                   "is invalid\n", filter_data.trap_id);
            err = -EINVAL;
            goto out;
        }

        spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);

        for (i = 0; i < MAX_LAG_PORTS_IN_FILTER; i++) {
            sx_priv(dev)->lag_filter_db[filter_data.trap_id][i] = LAG_ID_INVALID;
        }

        for (i = 0; i < MAX_SYSTEM_PORTS_IN_FILTER; i++) {
            sx_priv(dev)->sysport_filter_db[filter_data.trap_id][i] = 0;
        }

        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        printk(KERN_INFO PFX "Removed all ports and LAGs from the filter list "
               "of trap ID %d\n", filter_data.trap_id);
        break;
    }

    case CTRL_CMD_SET_DEFAULT_VID:
    {
        struct ku_default_vid_data default_vid_data;

        err = copy_from_user(&default_vid_data, (void*)data,
                             sizeof(default_vid_data));
        if (err) {
            goto out;
        }

        spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
        if (default_vid_data.is_lag) {
            if (default_vid_data.lag_id >= MAX_LAG_NUM) {
                printk(KERN_ERR PFX "Received LAG ID 0x%x "
                       "is invalid\n",
                       default_vid_data.lag_id);
                err = -EINVAL;
                spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                goto out;
            }

            sx_priv(dev)->pvid_lag_db[default_vid_data.lag_id] =
                default_vid_data.default_vid;
        } else {
            sx_priv(dev)->pvid_sysport_db[default_vid_data.sysport] =
                default_vid_data.default_vid;
        }

        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        break;
    }

    case CTRL_CMD_SET_VID_MEMBERSHIP:
    {
        struct ku_vid_membership_data vid_data;

        err = copy_from_user(&vid_data, (void*)data,
                             sizeof(vid_data));
        if (err) {
            goto out;
        }

        if (vid_data.vid >= MAX_VLAN_NUM) {
            printk(KERN_ERR PFX "Received VID %d "
                   "is invalid\n", vid_data.vid);
            err = -EINVAL;
            goto out;
        }

        spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
        if (vid_data.is_lag) {
            if (vid_data.lag_id >= MAX_LAG_NUM) {
                printk(KERN_ERR PFX "Received LAG ID 0x%x "
                       "is invalid\n",
                       vid_data.lag_id);
                err = -EINVAL;
                spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                goto out;
            }

            sx_priv(dev)->lag_vtag_mode[vid_data.lag_id][vid_data.vid] =
                vid_data.is_tagged;
        } else {
            if (vid_data.phy_port > MAX_PHYPORT_NUM) {
                printk(KERN_ERR PFX "Phy_port %d isn invalid. (MAX %d)\n",
                       vid_data.phy_port, MAX_PHYPORT_NUM);
                err = -EINVAL;
                spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                goto out;
            }
            sx_priv(dev)->port_vtag_mode[vid_data.phy_port][vid_data.vid] =
                vid_data.is_tagged;
        }
        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        break;
    }

    case CTRL_CMD_SET_PRIO_TAGGING:
    {
        struct ku_prio_tagging_data prio_tag_data;

        err = copy_from_user(&prio_tag_data, (void*)data,
                             sizeof(prio_tag_data));
        if (err) {
            goto out;
        }

        spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
        if (prio_tag_data.is_lag) {
            if (prio_tag_data.lag_id >= MAX_LAG_NUM) {
                printk(KERN_ERR PFX "Received LAG ID 0x%x "
                       "is invalid\n",
                       prio_tag_data.lag_id);
                err = -EINVAL;
                spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                goto out;
            }

            sx_priv(dev)->port_prio_tagging_mode[prio_tag_data.lag_id] =
                prio_tag_data.is_prio_tagged;
        } else {
            if (prio_tag_data.phy_port > MAX_PHYPORT_NUM) {
                printk(KERN_ERR PFX "Phy_port %d is invalid. (MAX %d)\n",
                       prio_tag_data.phy_port, MAX_PHYPORT_NUM);
                err = -EINVAL;
                spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                goto out;
            }
            sx_priv(dev)->port_prio_tagging_mode[prio_tag_data.phy_port] =
                prio_tag_data.is_prio_tagged;
        }

        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        break;
    }

    case CTRL_CMD_SET_PRIO_TO_TC:
    {
        struct ku_prio_to_tc_data prio_to_tc_data;

        err = copy_from_user(&prio_to_tc_data, (void*)data,
                             sizeof(prio_to_tc_data));
        if (err) {
            goto out;
        }

        if (prio_to_tc_data.priority > MAX_PRIO_NUM) {
            printk(KERN_ERR PFX "Received PRIO %d "
                   "is invalid (MAX %d). \n",
                   prio_to_tc_data.priority, MAX_PRIO_NUM);
            err = -EINVAL;
            goto out;
        }

        if (prio_to_tc_data.traffic_class > (NUMBER_OF_ETCLASSES - 1)) {
            printk(KERN_ERR PFX "Received TC %d "
                   "is invalid (MAX %d).\n",
                   prio_to_tc_data.traffic_class, (NUMBER_OF_ETCLASSES - 1));
            err = -EINVAL;
            goto out;
        }

        spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);

        if (prio_to_tc_data.is_lag) {
            if (prio_to_tc_data.lag_id >= MAX_LAG_NUM) {
                printk(KERN_ERR PFX "Received LAG ID 0x%x "
                       "is invalid\n",
                       prio_to_tc_data.lag_id);
                err = -EINVAL;
                spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                goto out;
            }

            sx_priv(dev)->lag_prio2tc[prio_to_tc_data.lag_id][prio_to_tc_data.priority] =
                prio_to_tc_data.traffic_class;
        } else {
            if (prio_to_tc_data.phy_port > MAX_PHYPORT_NUM) {
                printk(KERN_ERR PFX "Received Local %d "
                       "is invalid. (MAX %d).\n",
                       prio_to_tc_data.phy_port, MAX_PHYPORT_NUM);
                err = -EINVAL;
                spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                goto out;
            }

            sx_priv(dev)->port_prio2tc[prio_to_tc_data.phy_port][prio_to_tc_data.priority] =
                prio_to_tc_data.traffic_class;
        }

        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        break;
    }

    case CTRL_CMD_SET_LOCAL_PORT_TO_SWID:
    {
        struct ku_local_port_swid_data local_port_swid_data;

        err = copy_from_user(&local_port_swid_data, (void*)data,
                             sizeof(local_port_swid_data));
        if (err) {
            goto out;
        }

        if (local_port_swid_data.local_port > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Received Local port 0x%x "
                   "is invalid (max. %d) \n",
                   local_port_swid_data.local_port, MAX_PHYPORT_NUM);
            err = -EINVAL;
            goto out;
        }

        sx_priv(dev)->local_to_swid_db[local_port_swid_data.local_port] =
            local_port_swid_data.swid;

        #ifdef SX_DEBUG
        printk(KERN_DEBUG PFX " sx_ioctl() (PSPA) LOC_PORT_TO_SWID lp:%d, swid: %d \n",
               local_port_swid_data.local_port, local_port_swid_data.swid);
        #endif

        break;
    }

    case CTRL_CMD_SET_IB_TO_LOCAL_PORT:
    {
        struct ku_ib_local_port_data ib_local_port_data;

        err = copy_from_user(&ib_local_port_data, (void*)data,
                             sizeof(ib_local_port_data));
        if (err) {
            goto out;
        }

        if (ib_local_port_data.local_port > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Received Local port 0x%x "
                   "is invalid (max. %d) \n",
                   ib_local_port_data.local_port, MAX_PHYPORT_NUM);
            err = -EINVAL;
            goto out;
        }

        if (ib_local_port_data.ib_port > MAX_IBPORT_NUM) {
            printk(KERN_ERR PFX "Received IB port 0x%x "
                   "is invalid (max. %d) \n",
                   ib_local_port_data.ib_port, MAX_IBPORT_NUM);
            err = -EINVAL;
            goto out;
        }

        sx_priv(dev)->ib_to_local_db[ib_local_port_data.ib_port] =
            ib_local_port_data.local_port;

        #ifdef SX_DEBUG
        printk(KERN_DEBUG PFX " sx_ioctl() (PLIB) IB_TO_LOCAL_PORT ib_p:%d, lc_p:%d \n",
               ib_local_port_data.ib_port, ib_local_port_data.local_port);
        #endif

        break;
    }

    case CTRL_CMD_SET_SYSTEM_TO_LOCAL_PORT:
    {
        struct ku_system_local_port_data system_local_port_data;

        err = copy_from_user(&system_local_port_data, (void*)data,
                             sizeof(system_local_port_data));
        if (err) {
            goto out;
        }

        if (system_local_port_data.local_port > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Received Local port 0x%x "
                   "is invalid (max. %d) \n",
                   system_local_port_data.local_port, MAX_PHYPORT_NUM);
            err = -EINVAL;
            goto out;
        }

	spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
        sx_priv(dev)->system_to_local_db[system_local_port_data.system_port] =
		system_local_port_data.local_port;
	sx_priv(dev)->local_to_system_db[system_local_port_data.local_port] = 
		system_local_port_data.system_port;
	spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);

#ifdef SX_DEBUG
        printk(KERN_DEBUG PFX " sx_ioctl() (SSPR) SYSTEM_TO_LOCAL_PORT sys_p:0x%x, lc_p:%d \n",
               system_local_port_data.system_port, system_local_port_data.local_port);
#endif

        break;
    }

    case CTRL_CMD_SET_PORT_RP_MODE:
    {
        struct ku_port_rp_mode_data port_rp_mode_data;
        uint16_t                    local_port;
        uint8_t                     lag_port_index;

        err = copy_from_user(&port_rp_mode_data, (void*)data,
                             sizeof(port_rp_mode_data));
        if (err) {
            goto out;
        }

        spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);

        if (port_rp_mode_data.is_lag) {
            if (port_rp_mode_data.lag_id >= MAX_LAG_NUM) {
                printk(KERN_ERR PFX "Received LAG ID 0x%x "
                       "is invalid\n",
                       port_rp_mode_data.lag_id);
                err = -EINVAL;
                spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                goto out;
            }

            sx_priv(dev)->lag_is_rp[port_rp_mode_data.lag_id] =
                port_rp_mode_data.is_rp;
            sx_priv(dev)->lag_rp_vid[port_rp_mode_data.lag_id] =
                port_rp_mode_data.vlan_id;
            sx_priv(dev)->lag_rp_rif[port_rp_mode_data.lag_id][port_rp_mode_data.vlan_id] =
                port_rp_mode_data.rif_id;
            /* If opcode = create = 0, set IS_RP value,
             * else opcode = delete = 1, set DONT_CARE value */
            sx_priv(dev)->lag_rp_rif_valid[port_rp_mode_data.lag_id][port_rp_mode_data.vlan_id] =
                    port_rp_mode_data.opcode ? IS_RP_DONT_CARE_E : IS_RP_FROM_RP_E;

            for (lag_port_index = 0; lag_port_index < MAX_LAG_MEMBERS_NUM; lag_port_index++) {
                local_port =
                    sx_priv(dev)->lag_member_to_local_db[port_rp_mode_data.lag_id][lag_port_index];
                /* if the LAG is RP than assign swid 0 to this local port ,
                 * because we want all RP traffic to arrive with swid 0 */
                if (local_port != 0) {
                    if (port_rp_mode_data.is_rp) {
                        sx_priv(dev)->local_to_swid_db[local_port] = ROUTER_PORT_SWID;
                    } else {
                        sx_priv(dev)->local_to_swid_db[local_port] = 255;
                    }
                }
            }
        } else {
            local_port = sx_priv(dev)->system_to_local_db[port_rp_mode_data.sysport];
            if (local_port > MAX_PHYPORT_NUM) {
                printk(KERN_ERR PFX "Received Local %d "
                       "is invalid. (MAX %d).\n",
                       local_port, MAX_PHYPORT_NUM);
                err = -EINVAL;
                spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                goto out;
            }
            sx_priv(dev)->local_is_rp[local_port] = port_rp_mode_data.is_rp;
            sx_priv(dev)->local_rp_vid[local_port] = port_rp_mode_data.vlan_id;
            sx_priv(dev)->port_rp_rif[local_port][port_rp_mode_data.vlan_id] =
                    port_rp_mode_data.rif_id;
            /* If opcode = create = 0, set IS_RP value,
             * else opcode = delete = 1, set DONT_CARE value */
            sx_priv(dev)->port_rp_rif_valid[local_port][port_rp_mode_data.vlan_id] =
                    port_rp_mode_data.opcode ? IS_RP_DONT_CARE_E : IS_RP_FROM_RP_E;
            if (port_rp_mode_data.is_rp) {
                sx_priv(dev)->local_to_swid_db[local_port] = ROUTER_PORT_SWID;
            } else {
                sx_priv(dev)->local_to_swid_db[local_port] = 255;
            }
        }

        #ifdef SX_DEBUG
        printk(KERN_DEBUG PFX " sx_ioctl() (RITR) RP_MODE is_lag:%d,lid:%d,sys_p:0x%x,is_rp:%d \n",
               port_rp_mode_data.is_lag,
               port_rp_mode_data.lag_id,
               port_rp_mode_data.sysport,
               port_rp_mode_data.is_rp);
        #endif

        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        break;
    }

    case CTRL_CMD_SET_LOCAL_PORT_TO_LAG:
    {
        struct ku_local_port_to_lag_data local_to_lag_data;
        uint16_t                         lag_id;
        uint16_t                         lag_port_index;

        err = copy_from_user(&local_to_lag_data, (void*)data,
                             sizeof(local_to_lag_data));
        if (err) {
            goto out;
        }

        if (local_to_lag_data.lag_id >= MAX_LAG_NUM) {
            printk(KERN_ERR PFX "Received LAG ID 0x%x "
                   "is invalid\n", local_to_lag_data.lag_id);
            err = -EINVAL;
            goto out;
        }

        if (local_to_lag_data.lag_port_index >= MAX_LAG_MEMBERS_NUM) {
            printk(KERN_ERR PFX "Received LAG port index 0x%x "
                   "is invalid\n", local_to_lag_data.lag_port_index);
            err = -EINVAL;
            goto out;
        }

        if (local_to_lag_data.local_port > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Received Local port 0x%x "
                   "is invalid (max. %d) \n",
                   local_to_lag_data.local_port, MAX_PHYPORT_NUM);
            err = -EINVAL;
            goto out;
        }

        lag_id = local_to_lag_data.lag_id;
        lag_port_index = local_to_lag_data.lag_port_index;
        if ((lag_id >= MAX_LAG_NUM) ||
            (lag_port_index >= MAX_LAG_MEMBERS_NUM)) {
            printk(KERN_ERR PFX "Received LAG ID 0x%x or LAG_INDEX %d "
                   "is invalid\n", lag_id, lag_port_index);
            err = -EINVAL;
            goto out;
        }

        if (local_to_lag_data.is_lag) {
            /* Adding the port to LAG */
            sx_priv(dev)->lag_member_to_local_db[lag_id][lag_port_index] =
                local_to_lag_data.local_port;
            /* if the LAG is RP than assign swid 0 to this local port ,
             * because we want all RP traffic to arrive with swid 0 */
            if (sx_priv(dev)->lag_is_rp[lag_id]) {
                sx_priv(dev)->local_to_swid_db[local_to_lag_data.local_port] = ROUTER_PORT_SWID;
            }
        } else {
            /* Removing the port from LAG */
            sx_priv(dev)->lag_member_to_local_db[lag_id][lag_port_index] = 0;

            /* if the LAG is RP than assign swid 255 to this local port ,
             * because we want all RP traffic to arrive with swid 0 */
            if (sx_priv(dev)->lag_is_rp[lag_id]) {
                sx_priv(dev)->local_to_swid_db[local_to_lag_data.local_port] = 255;
            }
        }

#ifdef SX_DEBUG
        printk(KERN_DEBUG PFX " sx_ioctl() (SLCOR) LOCAL_PORT_TO_LAG is_lag:%d,lid:%d,port_id:%x,loc_port:%d \n",
               local_to_lag_data.is_lag,
               lag_id,
               lag_port_index,
               local_to_lag_data.local_port);
#endif

        break;
    }

    case CTRL_CMD_SET_RDQ_RATE_LIMITER:
    {
        struct ku_set_rdq_rate_limiter rate_limiter_params;
        int                            cqn;

        printk(KERN_DEBUG PFX "ioctl CTRL_CMD_SET_RDQ_RATE_LIMITER called\n");
        if (!dev->pdev) {
            printk(KERN_DEBUG PFX "will not set rate limiter since there's no  PCI device\n");
            goto out;
        }

        err = copy_from_user(&rate_limiter_params, (void*)data,
                             sizeof(rate_limiter_params));
        if (err) {
            goto out;
        }

        if (rate_limiter_params.rdq >= NUMBER_OF_RDQS) {
            printk(KERN_WARNING PFX "Cannot set the rate limiter, RDQ value (%u) is not valid\n",
                   rate_limiter_params.rdq);
            err = -EINVAL;
            goto out;
        }

        cqn = rate_limiter_params.rdq + NUMBER_OF_SDQS;
        if (rate_limiter_params.use_limiter) {
            sx_priv(dev)->cq_table.rl_time_interval =
                max((int)rate_limiter_params.time_interval, 50);
            sx_priv(dev)->cq_table.cq_rl_params[cqn].interval_credit =
                rate_limiter_params.interval_credit;
            sx_priv(dev)->cq_table.cq_rl_params[cqn].max_cq_credit =
                rate_limiter_params.max_credit;
        }

        sx_priv(dev)->cq_table.cq_rl_params[cqn].use_limiter =
            rate_limiter_params.use_limiter;
        if (!rate_limiter_params.use_limiter) {
            /* If the RDQ is not allocated we can finish here */
            if (!sx_bitmap_test(&sx_priv(dev)->rdq_table.bitmap,
                                rate_limiter_params.rdq)) {
                goto out;
            }

            sx_priv(dev)->cq_table.cq_rl_params[cqn].curr_cq_credit = 0;
            /* The CQ might not be armed if it ran out of credits */
            sx_cq_arm(sx_priv(dev)->cq_table.cq[cqn]);
        }

        if (!sx_priv(dev)->cq_table.cq_credit_thread &&
            rate_limiter_params.use_limiter) {
            char kth_name[20];

            sprintf(kth_name, "cq_credit_thread");
            sx_priv(dev)->cq_table.cq_credit_thread =
                kthread_create(sx_cq_credit_thread_handler,
                               (void*)sx_priv(dev), kth_name);
            if (IS_ERR(sx_priv(dev)->cq_table.cq_credit_thread)) {
                printk(KERN_ERR PFX "Failed creating the CQ credit thread\n");
                return -ENOMEM;
            } else {
                printk(KERN_INFO PFX "cq_credit_thread has been created\n");
            }
        }

        /* start the CQ credit task */
        if (rate_limiter_params.use_limiter &&
            sx_bitmap_test(&sx_priv(dev)->cq_table.bitmap, cqn) &&
            !sx_priv(dev)->cq_table.credit_thread_active) {
            wake_up_process(sx_priv(dev)->cq_table.cq_credit_thread);
            if (cq_thread_sched_priority != 0) {
                struct sched_param param = { .sched_priority = cq_thread_sched_priority };

                err = sched_setscheduler(sx_priv(dev)->cq_table.cq_credit_thread,
                                         SCHED_FIFO, &param);
                if (err) {
                    printk(KERN_INFO PFX "Failed setting RT prio %d to the "
                           "cq_credit_thread, err = %d\n",
                           cq_thread_sched_priority, err);
                } else {
                    printk(KERN_INFO PFX "Successfully set the real time priority of the "
                           "cq_credit_thread to %d\n", cq_thread_sched_priority);
                }
            }


            sx_priv(dev)->cq_table.credit_thread_active = 1;
        }

        if (rate_limiter_params.use_limiter) {
            printk(KERN_DEBUG PFX "Added a rate limiter to RDQ %d: "
                   "interval_credit=%u, "
                   "max_credit=%u,"
                   "time_interval=%u\n",
                   rate_limiter_params.rdq,
                   sx_priv(dev)->cq_table.cq_rl_params[cqn].interval_credit,
                   sx_priv(dev)->cq_table.cq_rl_params[cqn].max_cq_credit,
                   sx_priv(dev)->cq_table.rl_time_interval);
        } else {
            printk(KERN_DEBUG PFX "Removed the rate limiter from RDQ %d\n",
                   rate_limiter_params.rdq);
        }

        break;
    }

    case CTRL_CMD_SET_TRUNCATE_PARAMS:
    {
        struct ku_set_truncate_params truncate_params;

        if (!dev->pdev) {
            printk(KERN_DEBUG PFX "will not set truncate params since there's no "
                   "PCI device\n");
            goto out;
        }

        err = copy_from_user(&truncate_params, (void*)data,
                             sizeof(truncate_params));
        if (err) {
            goto out;
        }

        if (truncate_params.rdq >= dev->dev_cap.max_num_rdqs) {
            printk(KERN_ERR PFX "CTRL_CMD_SET_TRUNCATE_PARAMS: RDQ %d is not valid\n",
                   truncate_params.rdq);
            return -EINVAL;
        }

        spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
        if (truncate_params.truncate_enable) {
            if (truncate_params.truncate_size < SX_TRUNCATE_SIZE_MIN) {
                printk(KERN_ERR PFX "CTRL_CMD_SET_TRUNCATE_PARAMS: Truncate size %u is not valid\n",
                       truncate_params.truncate_size);
                spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
                err = -EINVAL;
                goto out;
            }

            sx_priv(dev)->truncate_size_db[truncate_params.rdq] = truncate_params.truncate_size;
        } else {
            sx_priv(dev)->truncate_size_db[truncate_params.rdq] = 0;
        }

        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        break;
    }

    case CTRL_CMD_CR_SPACE_READ:
    {
        struct ku_cr_space_read read_data;
        unsigned char          *buf = NULL;

        err = copy_from_user(&read_data, (void*)data,
                             sizeof(read_data));
        if (err) {
            goto out;
        }

        buf = kmalloc(read_data.size, GFP_KERNEL);
        if (!buf) {
            err = -ENOMEM;
            goto out;
        }

        err = sx_dpt_cr_space_read(read_data.dev_id, read_data.address,
                                   buf, read_data.size);
        if (err) {
            kfree(buf);
            goto out;
        }

        err = copy_to_user(read_data.data, buf, read_data.size);
        kfree(buf);
        break;
    }

    case CTRL_CMD_CR_SPACE_WRITE:
    {
        struct ku_cr_space_write write_data;
        unsigned char           *buf = NULL;

        err = copy_from_user(&write_data, (void*)data,
                             sizeof(write_data));
        if (err) {
            goto out;
        }

        buf = kmalloc(write_data.size, GFP_KERNEL);
        if (!buf) {
            err = -ENOMEM;
            goto out;
        }

        err = copy_from_user(buf, write_data.data, write_data.size);
        if (err) {
            kfree(buf);
            goto out;
        }

        err = sx_dpt_cr_space_write(write_data.dev_id, write_data.address,
                                    buf, write_data.size);
        kfree(buf);
        break;
    }

    case CTRL_CMD_CREATE_PORT_NETDEV:
    {
        struct ku_port_netdev netdev_data;
        union sx_event_data   netdev_event_data;

        printk(KERN_DEBUG PFX "ioctl CTRL_CMD_CREATE_PORT_NETDEV called\n");
        err = copy_from_user(&netdev_data, (void*)data,
                             sizeof(netdev_data));
        if (err) {
            goto out;
        }

        spin_lock(&dev->profile_lock);
        if (!dev->profile_set) {
            spin_unlock(&dev->profile_lock);
            err = -EINVAL;
            goto out;
        }

        spin_unlock(&dev->profile_lock);
        netdev_event_data.port_netdev_set.sysport = netdev_data.sysport;
        netdev_event_data.port_netdev_set.is_lag = netdev_data.is_lag;
        if (netdev_data.is_lag) {
            netdev_event_data.port_netdev_set.mid = netdev_data.sysport +
                                                    0xC000 + sx_glb.profile.max_mid;
            printk(KERN_DEBUG "port_netdev_set.mid : 0x%x, is_lag: %d"
                   "sys_port:0x%x, max_mid: 0x%x \n",
                   netdev_event_data.port_netdev_set.mid,
                   netdev_data.is_lag,
                   netdev_data.sysport,
                   sx_glb.profile.max_mid);
        } else {
            printk(KERN_DEBUG "NOT LAG : port_netdev_set.mid : 0x%x \n", netdev_data.is_lag);
        }

        netdev_event_data.port_netdev_set.name = netdev_data.name;
        netdev_event_data.port_netdev_set.swid = netdev_data.swid;

        spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
        if (sx_priv(dev)->dev_specific_cb.get_send_to_rp_as_data_supported_cb != NULL) {
            sx_priv(dev)->dev_specific_cb.get_send_to_rp_as_data_supported_cb(&netdev_event_data.port_netdev_set.send_to_rp_as_data_supported);
        } else {
            netdev_event_data.port_netdev_set.send_to_rp_as_data_supported = false;
        }
        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);

        sx_core_dispatch_event(dev, SX_DEV_EVENT_OPEN_PORT_NETDEV, &netdev_event_data);
        break;
    }

    case CTRL_CMD_REMOVE_PORT_NETDEV:
    {
        struct ku_port_netdev netdev_data;
        union sx_event_data   netdev_event_data;

        printk(KERN_DEBUG PFX "ioctl CTRL_CMD_REMOVE_PORT_NETDEV called\n");
        err = copy_from_user(&netdev_data, (void*)data,
                             sizeof(netdev_data));
        if (err) {
            goto out;
        }

        spin_lock(&dev->profile_lock);
        if (!dev->profile_set) {
            spin_unlock(&dev->profile_lock);
            err = -EINVAL;
            goto out;
        }

        spin_unlock(&dev->profile_lock);
        netdev_event_data.port_netdev_set.sysport = netdev_data.sysport;
        sx_core_dispatch_event(dev, SX_DEV_EVENT_CLOSE_PORT_NETDEV, &netdev_event_data);
        break;
    }

    case CTRL_CMD_SET_SGMII_BASE_SMAC:
    {
        struct ku_sgmii_smac smac_data;

        if (sx_glb.sx_sgmii.initialized == 1) {
            printk(KERN_WARNING PFX "ioctl CTRL_CMD_SET_SGMII_BASE_SMAC: "
                   "Cannot set SGMII base SMAC after the SGMII was initialized\n");
            err = -EPERM;
            goto out;
        }

        err = copy_from_user(&smac_data, (void*)data,
                             sizeof(smac_data));
        if (err) {
            goto out;
        }

        if ((smac_data.number_of_macs == 0) ||
            (smac_data.number_of_macs > MAX_SGMII_FLOWS)) {
            printk(KERN_WARNING PFX "ioctl CTRL_CMD_SET_SGMII_BASE_SMAC: "
                   "number_of_macs %d is not valid\n", smac_data.number_of_macs);
            err = -EINVAL;
            goto out;
        }

        sx_glb.sx_sgmii.base_smac = smac_data.base_smac;
        sx_glb.sx_sgmii.number_of_macs = smac_data.number_of_macs;
        printk(KERN_INFO PFX "SGMII SMAC set to 0x%llx, number of "
               "MACs (%u)\n", sx_glb.sx_sgmii.base_smac,
               sx_glb.sx_sgmii.number_of_macs);
        break;
    }

    case CTRL_CMD_SET_VID_2_IP:
    {
        struct ku_vid2ip_data vid2ip_data;

        err = copy_from_user(&vid2ip_data, (void*)data,
                             sizeof(vid2ip_data));
        if (err) {
            goto out;
        }

        if (vid2ip_data.vid >= MAX_VLAN_NUM) {
            printk(KERN_ERR PFX "Received VID %d "
                   "is invalid\n", vid2ip_data.vid);
            err = -EINVAL;
            goto out;
        }

        spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);

        if (vid2ip_data.valid) {
            sx_priv(dev)->icmp_vlan2ip_db[vid2ip_data.vid] =
                vid2ip_data.ip_addr;
        } else {
            sx_priv(dev)->icmp_vlan2ip_db[vid2ip_data.vid] = 0;
        }

        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        break;
    }
    
    case CTRL_CMD_SET_PORT_VID_TO_FID_MAP:
    {        
        struct ku_port_vlan_to_fid_map_data port_vlan_to_fid_map_data;

        err = copy_from_user(&port_vlan_to_fid_map_data, (void*)data,
                             sizeof(port_vlan_to_fid_map_data));
        if (err) {
            goto out;
        }
        
        if (port_vlan_to_fid_map_data.local_port > MAX_PHYPORT_NUM) {
            printk(KERN_ERR PFX "Received Local port 0x%x "
                   "is invalid (max. %d) \n",
                   port_vlan_to_fid_map_data.local_port, MAX_PHYPORT_NUM);
            err = -EINVAL;
            goto out;
        }

        if (port_vlan_to_fid_map_data.vid >= MAX_VLAN_NUM) {
            printk(KERN_ERR PFX "Received VID %d "
                   "is invalid\n", port_vlan_to_fid_map_data.vid);
            err = -EINVAL;
            goto out;
        }

        spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);

        if (port_vlan_to_fid_map_data.is_mapped_to_fid) {
            sx_priv(dev)->port_vid_to_fid[port_vlan_to_fid_map_data.local_port][port_vlan_to_fid_map_data.vid] =
                port_vlan_to_fid_map_data.fid;
        } else {
            sx_priv(dev)->port_vid_to_fid[port_vlan_to_fid_map_data.local_port][port_vlan_to_fid_map_data.vid] = 0;
        }

        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        break;
    }

    default:
        return -EINVAL;
    }

out:
    return err;
}

static unsigned int sx_core_poll(struct file *filp, poll_table *wait)
{
    unsigned int   mask = 0;
    struct sx_rsc *file = NULL;
    unsigned long  flags;

    file = filp->private_data;
    poll_wait(filp, &file->poll_wait, wait);
    spin_lock_irqsave(&file->lock, flags);
    if (!list_empty(&file->evlist.list)) {
        mask |= POLLIN | POLLRDNORM;  /* readable */
    }
    if (file->evlist_size < SX_EVENT_LIST_SIZE) {
        mask |= POLLOUT | POLLWRNORM; /* writeable */
    }
    spin_unlock_irqrestore(&file->lock, flags);

    return mask;
}

static int sx_core_close(struct inode *inode, struct file *filp)
{
    struct event_data     *edata;
    unsigned long          flags;
    struct listener_entry *listener;
    struct list_head      *pos, *q;
    int                    entry;
    struct sx_rsc         *file = filp->private_data;

#ifdef SX_DEBUG
    printk(KERN_DEBUG PFX " sx_core_close() \n");
#endif

    /* delete all listener entries belong to this fd */
    spin_lock_irqsave(&sx_glb.listeners_lock, flags);
    for (entry = 0; entry < NUM_HW_SYNDROMES + 1; entry++) {
        if (!list_empty(&sx_glb.listeners_db[entry].list)) {
            list_for_each_safe(pos, q,
                               &sx_glb.listeners_db[entry].list) {
                listener = list_entry(pos,
                                      struct listener_entry, list);

                if ((struct file *)listener->context == filp) {
                    list_del(pos);
                    kfree(listener);
                }
            }
        }
    }

    spin_unlock_irqrestore(&sx_glb.listeners_lock, flags);
    spin_lock_irqsave(&file->lock, flags);
    list_for_each_safe(pos, q, &file->evlist.list) {
        edata = list_entry(pos, struct event_data, list);
        list_del(pos);
        kfree_skb(edata->skb);
        kfree(edata);
    }

    spin_unlock_irqrestore(&file->lock, flags);
    kfree(file);
    SX_CORE_UNUSED_PARAM(inode);

    return 0;
}

int sx_core_flush_synd_by_context(void * context)
{
    unsigned long          flags;
    struct listener_entry *listener;
    struct list_head      *pos, *q;
    int                    entry;

    /* delete all listener entries belong to this context */
    spin_lock_irqsave(&sx_glb.listeners_lock, flags);
    for (entry = 0; entry < NUM_HW_SYNDROMES + 1; entry++) {
        if (!list_empty(&sx_glb.listeners_db[entry].list)) {
            list_for_each_safe(pos, q,
                               &sx_glb.listeners_db[entry].list) {
                listener = list_entry(pos,
                                      struct listener_entry, list);

                if (listener->context == context) {
                    list_del(pos);
                    kfree(listener);
                }
            }
        }
    }

    spin_unlock_irqrestore(&sx_glb.listeners_lock, flags);

    return 0;
}
EXPORT_SYMBOL(sx_core_flush_synd_by_context);

int sx_core_flush_synd_by_handler(cq_handler handler)
{
    unsigned long          flags;
    struct listener_entry *listener;
    struct list_head      *pos, *q;
    int                    entry;

    /* delete all listener entries belong to this context */
    spin_lock_irqsave(&sx_glb.listeners_lock, flags);
    for (entry = 0; entry < NUM_HW_SYNDROMES + 1; entry++) {
        if (!list_empty(&sx_glb.listeners_db[entry].list)) {
            list_for_each_safe(pos, q,
                               &sx_glb.listeners_db[entry].list) {
                listener = list_entry(pos,
                                      struct listener_entry, list);

                if (listener->handler == handler) {
                    list_del(pos);
                    kfree(listener);
                }
            }
        }
    }

    spin_unlock_irqrestore(&sx_glb.listeners_lock, flags);

    return 0;
}
EXPORT_SYMBOL(sx_core_flush_synd_by_handler);

/************************************************
 *  Data-Structures
 ***********************************************/

static const struct file_operations sx_core_fops = {
    .owner = THIS_MODULE,
    .open = sx_core_open,
    .read = sx_core_read,
    .write = sx_core_write,
	.unlocked_ioctl	 =	sx_core_ioctl,
    .poll = sx_core_poll,
    .release = sx_core_close
};

/************************************************
 *  Module Functions
 ***********************************************/
static int sx_load_fw(struct sx_dev *dev)
{
    struct sx_priv *priv = sx_priv(dev);
    int             err;

    priv->fw.fw_icm = sx_alloc_icm(dev, priv->fw.fw_pages,
                                   GFP_HIGHUSER | __GFP_NOWARN, 0);
    if (!priv->fw.fw_icm) {
        sx_err(dev, "Couldn't allocate FW area, aborting.\n");
        return -ENOMEM;
    }

    err = sx_MAP_FA(dev, priv->fw.fw_icm);
    if (err) {
        sx_err(dev, "MAP_FA command failed, aborting.\n");
        goto err_free;
    }

    return 0;

err_free:
    sx_free_icm(dev, priv->fw.fw_icm, 0);
    return err;
}

static int sx_core_init_cb(struct sx_dev *dev,
                           uint16_t device_id,
                           uint16_t device_hw_revision)
{
    int err = 0;
    enum sxd_chip_types chip_type;

    switch (device_id) {
    case SXD_MGIR_HW_DEV_ID_SX:
        if (device_hw_revision == 0xA1) {
            chip_type = SXD_CHIP_TYPE_SWITCHX_A1;
        } else if (device_hw_revision == 0xA2) {
            chip_type = SXD_CHIP_TYPE_SWITCHX_A2;
        } else if (device_hw_revision == 0xA0) {
            printk(KERN_ERR PFX "The SwitchX device revision is A0, "
                   "and therefore it is not supported by SX driver\n");
            return -EFAULT;
        } else {
            printk(KERN_ERR PFX "The SwitchX device revision (0x%x) "
                   "is not supported by SX driver\n", device_hw_revision);
            return -EFAULT;
        }
        break;

    case SXD_MGIR_HW_DEV_ID_SWITCH_IB:
        chip_type = SXD_CHIP_TYPE_SWITCH_IB;
        break;

    case SXD_MGIR_HW_DEV_ID_SPECTRUM:
        chip_type = SXD_CHIP_TYPE_SPECTRUM;
        break;

    case SXD_MGIR_HW_DEV_ID_SWITCH_IB2:
		chip_type = SXD_CHIP_TYPE_SWITCH_IB2;
		break;

    default:
        printk(KERN_ERR PFX "ERROR: Unresolved chip type. device_id (%u)\n", device_id);
        return -EFAULT;
    }

    err = sx_core_dev_init_switchx_cb(dev, chip_type);
    if (err) {
        printk(KERN_ERR PFX "callback device init failed for device (%u)\n",
               dev->profile.dev_id);
        return err;
    }

    return err;
}

static int sx_init_board(struct sx_dev *dev)
{
    struct sx_board           board;
    int                       err;
    struct ku_access_mgir_reg reg_data;
    int                       retry_num = 0;

    /*
        This is a workaround to race condition occured when FW 
        boot isn't finished and we start to read MGIR. 
        We post the in_mailbox but FW zero GO bit. So we think 
        that command is done.
        After this race we get 0 in all MGIR fields.
        The temporary solution is to reread again.
        The real solution should provide interface to read HEALTH 
        bits which will indicate that FW boot is finished.
    */
    while (retry_num < 3) {
        memset(&reg_data, 0, sizeof(reg_data));
        reg_data.dev_id = dev->device_id;
        reg_data.op_tlv.type = 1;
        reg_data.op_tlv.length = 4;
        reg_data.op_tlv.dr = 0;
        reg_data.op_tlv.status = 0;
        reg_data.op_tlv.register_id = 0x9020; /* MGIR register ID */
        reg_data.op_tlv.r = 0;
        reg_data.op_tlv.method = 1; /* Query */
        reg_data.op_tlv.op_class = 1;
        reg_data.op_tlv.tid = 0;

        err = sx_ACCESS_REG_MGIR(dev, &reg_data);
        /* Only if we managed to read MGIR successfully we check the HW revision
         * to see it's not A0 */
        if (!err && !reg_data.op_tlv.status &&
            (reg_data.mgir_reg.hw_info.device_id == SXD_MGIR_HW_DEV_ID_SX) &&
            (reg_data.mgir_reg.hw_info.device_hw_revision == SXD_MGIR_HW_REV_ID_SX_A0)) {
            printk(KERN_ERR PFX "The SwitchX device revision is A0, "
                   "and therefore it is not supported by SX driver\n");
            return -EFAULT;
        }

        if (reg_data.mgir_reg.hw_info.device_id != 0) {
            break;
        }

        msleep(500*retry_num);
        retry_num++;
    } 
 
    err = sx_core_init_cb(dev, reg_data.mgir_reg.hw_info.device_id,
                          reg_data.mgir_reg.hw_info.device_hw_revision);
    if (err) {
        printk(KERN_ERR PFX "callback dev init failed for device (%u)\n",
               dev->profile.dev_id);
        return err;
    }

    err = sx_QUERY_FW(dev, NULL);
    if (err) {
        sx_err(dev, "QUERY_FW command failed, aborting.\n");
        return err;
    }

    /* init local mailboxes */
    err = sx_QUERY_FW_2(dev, dev->device_id);
    if (err) {
        sx_err(dev, "QUERY_FW_2 command failed, aborting.\n");
        return err;
    }

    dev->bar0_dbregs_offset = sx_priv(dev)->fw.doorbell_page_offset;
    dev->bar0_dbregs_bar = sx_priv(dev)->fw.doorbell_page_bar;

    err = sx_load_fw(dev);
    if (err) {
        sx_err(dev, "Failed to start FW, aborting.\n");
        return err;
    }

    err = sx_QUERY_AQ_CAP(dev);
    if (err) {
        sx_err(dev, "QUERY_AQ_CAP command failed, aborting.\n");
        goto err_stop_fw;
    }
    dev->dev_cap.max_num_cpu_egress_tcs = 12;
    dev->dev_cap.max_num_cpu_ingress_tcs = 16;

    err = sx_QUERY_BOARDINFO(dev, &board);
    if (err) {
        sx_err(dev, "QUERY_BOARDINFO command failed, aborting.\n");
        goto err_stop_fw;
    }

    sx_priv(dev)->eq_table.inta_pin = board.inta_pin;
    memcpy(dev->board_id, board.board_id, sizeof(dev->board_id));
    dev->vsd_vendor_id = board.vsd_vendor_id;
    return 0;

err_stop_fw:
    sx_UNMAP_FA(dev);
    sx_free_icm(dev, sx_priv(dev)->fw.fw_icm, 0);

    return err;
}

static void sx_enable_msi_x(struct sx_dev *dev)
{
#if 0
    struct sx_priv   *priv = sx_priv(dev);
    struct msix_entry entry;
    int               err;
    int               i;

    if (msi_x) {
        entry.entry = 0;
        err = pci_enable_msix(dev->pdev, &entry, 1);
        if (err) {
            if (err > 0) {
                printk(KERN_INFO PFX "Only %d MSI-X vectors available, "
                       "not using MSI-X\n", err);
            } else {
                printk(KERN_DEBUG PFX "Failed enabling MSI-X interrupts. "
                       "Going to use standard interrupts instead\n");
            }

            goto no_msi;
        }

        sx_info(dev, "MSI-X interrupts were enabled successfully\n");
        for (i = 0; i < SX_NUM_EQ; ++i) {
            priv->eq_table.eq[i].irq = entry.vector;
        }

        dev->flags |= SX_FLAG_MSI_X;
        return;
    }

no_msi:
    msi_x = 0;
    for (i = 0; i < SX_NUM_EQ; ++i) {
        priv->eq_table.eq[i].irq = dev->pdev->irq;
    }
#endif
}

static int sx_map_doorbell_area(struct sx_dev *dev)
{
    dev->db_base =
        ioremap(pci_resource_start(dev->pdev, dev->bar0_dbregs_bar)
                + dev->bar0_dbregs_offset,
                SX_DBELL_REGION_SIZE);
    if (!dev->db_base) {
        printk(KERN_ERR "%s(): bar: %d virt: is NULL \n",
               __func__, dev->bar0_dbregs_bar);

        return -EINVAL;
    }

    printk(KERN_DEBUG "%s(): bar: %d dev->db_base phys: 0x%llx , virt: 0x%p \n",
           __func__,
           dev->bar0_dbregs_bar,
           pci_resource_start(dev->pdev, dev->bar0_dbregs_bar)
           + dev->bar0_dbregs_offset,
           dev->db_base);

    return 0;
}

static void sx_doorbell_cleanup(struct sx_dev *dev)
{
    iounmap(dev->db_base);
}

static void sx_close_board(struct sx_dev *dev)
{
    sx_UNMAP_FA(dev);
    sx_free_icm(dev, sx_priv(dev)->fw.fw_icm, 0);
}

#ifdef NO_PCI
static void sx_destroy_sx(struct sx_dev *dev)
{
    /**Clearing the rdq tables**/
    sx_core_destroy_rdq_table(dev, true);

    /**Clearing the sdq tables**/
    sx_core_destroy_sdq_table(dev, true);

    /**Clearing the cq tables**/
    sx_core_destroy_cq_table(dev);

    /**Returning to polling (not command)**/
    sx_cmd_use_polling(dev);

    /**Clearing the eq tables**/
    sx_cleanup_eq_table(dev);

    return;
}
#endif

static int sx_setup_sx(struct sx_dev *dev)
{
    int err = 0;

    err = sx_init_eq_table(dev);
    if (err) {
        sx_err(dev, "Failed to initialize "
               "event queue table, aborting.\n");
        goto out_ret;
    }

    err = sx_cmd_use_events(dev);
    if (err) {
        sx_err(dev, "Failed to switch to event-driven "
               "firmware commands, aborting.\n");
        goto err_eq_table_free;
    }

    err = sx_core_init_cq_table(dev);
    if (err) {
        sx_err(dev, "Failed to initialize CQ table, aborting.\n");
        goto err_cmd_poll;
    }

    err = sx_core_init_sdq_table(dev);
    if (err) {
        sx_err(dev, "Failed to initialize SDQ table, aborting.\n");
        goto err_cq_table_free;
    }

    err = sx_core_init_rdq_table(dev);
    if (err) {
        sx_err(dev, "Failed to initialize RDQ table, aborting.\n");
        goto err_sdq_table_free;
    }

    return 0;

err_sdq_table_free:
    sx_core_destroy_sdq_table(dev, true);

err_cq_table_free:
    sx_core_destroy_cq_table(dev);

err_cmd_poll:
    sx_cmd_use_polling(dev);

err_eq_table_free:
    sx_cleanup_eq_table(dev);

out_ret:
    return err;
}

static int sx_core_init_one_pci(struct pci_dev *pdev, const struct pci_device_id *id)
{
#if 0
    struct sx_priv *priv = NULL;
    struct sx_dev  *dev = NULL;
    int             err = 0;

    sx_glb.pci_devs_cnt++;
    
    printk(KERN_INFO PFX "Probe %s(%d) device %u\n", __FUNCTION__, __LINE__, \
           pdev->device);
    err = sx_core_init_one(&priv);
    if (err) {
        dev_err(&pdev->dev, "sx_core_init_one failed with err: %d , aborting.\n",
                err);
        goto out;
    }

    dev = &priv->dev;

    err = pci_enable_device(pdev);
    if (err) {
        dev_err(&pdev->dev, "Cannot enable PCI device, aborting.\n");
        goto err_enable_pdev;
    }

    /* Check for BARs.  We expect 0: 1MB in Baz and 4MB in Pelican */
    if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM) ||
        ((pci_resource_len(pdev, 0) != 1 << 20) &&
         (pci_resource_len(pdev, 0) != 1 << 22))) {
        dev_err(&pdev->dev, "Missing BAR0, aborting.\n");
        err = -ENODEV;
        goto err_disable_pdev;
    }

    err = pci_request_region(pdev, 0, DRV_NAME);
    if (err) {
        dev_err(&pdev->dev, "Cannot request control region, "
                "aborting.\n");
        goto err_disable_pdev;
    }

    pci_set_master(pdev);
    err = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
    if (err) {
        dev_warn(&pdev->dev, "Warning: couldn't set 64-bit PCI "
                 "DMA mask.\n");
        err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
        if (err) {
            dev_err(&pdev->dev, "Can't set PCI DMA mask, aborting.\n");
            goto err_release_bar0;
        }
    }

    err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
    if (err) {
        dev_warn(&pdev->dev, "Warning: couldn't set 64-bit "
                 "consistent PCI DMA mask.\n");
        err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
        if (err) {
            dev_err(&pdev->dev, "Can't set consistent PCI DMA "
                    "mask, aborting.\n");
            goto err_release_bar0;
        }
    }

    dev->pdev = pdev;
#ifndef PD_BU
    err = sx_reset(dev);
    if (err) {
        sx_err(dev, "Failed to reset HW, aborting.\n");
        goto err_release_bar0;
    }
#else
    printk(KERN_INFO PFX "Performing SW reset is SKIPPED in PD mode.\n");
#endif

    if (sx_cmd_pool_create(dev)) {
        sx_err(dev, "Failed to create command buffer pool, aborting.\n");
        goto err_release_bar0;
    }

    if (sx_cmd_init_pci(dev)) {
        sx_err(dev, "Failed to initialize command interface, aborting.\n");
        goto err_free_pool;
    }

    pci_set_drvdata(pdev, dev);
    spin_lock(&sx_glb.pci_devs_lock);
    list_add(&dev->list, &sx_glb.pci_devs_list);
    spin_unlock(&sx_glb.pci_devs_lock);
    err = sx_dpt_init_dev_pci(dev);
    if (err) {
        sx_err(dev, "Failed initializing default PCI device "
               "attibutes in the DPT, aborting.\n");
        goto err_free_pool;
    }

    err = sx_init_board(dev);
    if (err) {
        goto err_cmd;
    }

    sx_enable_msi_x(dev);
    err = sx_map_doorbell_area(dev);
    if (err) {
        goto err_dbell;
    }

    /* Only if the device is not registered */
    if (priv->unregistered) {
        err = sx_core_register_device(dev);
        if (err) {
            sx_err(dev, "Failed to register the device, aborting.\n");
            goto err_dbell_clean;
        }

        priv->unregistered = 0;
    }

    err = sx_setup_sx(dev);
    if ((err == -EBUSY) && (dev->flags & SX_FLAG_MSI_X)) {
        dev->flags &= ~SX_FLAG_MSI_X;
        pci_disable_msix(dev->pdev);
        err = sx_setup_sx(dev);
    }

    if (err) {
        goto out_unregister;
    }

    dev->global_flushing = 0;
    dev->dev_stuck = 0;
    sx_core_start_catas_poll(dev);
#endif
    return 0;
#if 0
out_unregister:
    if (!priv->unregistered) {
        sx_core_unregister_device(dev);
        priv->unregistered = 1;
    }

err_dbell_clean:
    sx_doorbell_cleanup(dev);

err_dbell:
    if (dev->flags & SX_FLAG_MSI_X) {
        pci_disable_msix(pdev);
    }

    sx_close_board(dev);

err_free_pool:
    sx_cmd_pool_destroy(dev);
    list_del(&dev->list);

err_cmd:
    sx_cmd_unmap(dev);

err_release_bar0:
    pci_release_region(pdev, 0);

err_disable_pdev:
    pci_disable_device(pdev);
    pci_set_drvdata(pdev, NULL);

err_enable_pdev:
    sx_core_remove_one(priv);

out:
    return err;
#endif
}

static int sx_core_init_one(struct sx_priv **sx_priv)
{
    struct sx_priv *priv;
    struct sx_dev  *dev;
    int             i, j, err;

#ifdef NO_PCI
    printk(KERN_INFO PFX "Initializing in NO_PCI mode\n");
#endif

    if (!sx_priv) {
        printk(KERN_ERR PFX "Invalid param %s\n", __func__);
        return -EINVAL;
    }

    priv = vmalloc(sizeof(struct sx_priv));
    if (!priv) {
        printk(KERN_ERR PFX "Device struct alloc failed, aborting.\n");
        err = -ENOMEM;
        goto out;
    }
    memset(priv, 0, sizeof *priv);
    dev = &priv->dev;

    /* default pvid for all ports is 1 */
    for (i = 0; i < MAX_SYSPORT_NUM; i++) {
        if (i < MAX_LAG_NUM) {
            priv->pvid_lag_db[i] = 1;
        }

        priv->pvid_sysport_db[i] = 1;
    }

    /* initialize lag_filter_db with invalid value */
    for (i = 0; i < NUM_HW_SYNDROMES; i++) {
        for (j = 0; j < MAX_LAG_PORTS_IN_FILTER; j++) {
            priv->lag_filter_db[i][j] = LAG_ID_INVALID;
        }
    }

    err = sx_dpt_init_default_dev(dev);
    if (err) {
        sx_err(dev, "Failed initializing default device "
               "attibutes in the DPT, aborting.\n");
        goto out_free_priv;
    }

    err = sx_cmd_init(dev);
    if (err) {
        sx_err(dev, "Failed initializing command interface, aborting.\n");
        goto out_free_priv;
    }

    spin_lock_init(&dev->profile_lock);
    dev->profile_set = 0;
    dev->dev_profile_set = 0;
    dev->first_ib_swid = 1;
    spin_lock_init(&priv->ctx_lock);
    spin_lock_init(&priv->db_lock);
    INIT_LIST_HEAD(&priv->ctx_list);
    INIT_LIST_HEAD(&priv->dev_list);
    atomic_set(&priv->cq_backup_polling_refcnt, 0);

#if 0
    err = sx_core_catas_init(dev);
    if (err) {
        printk(KERN_ERR PFX "Couldn't start catas. Aborting...\n");
        goto out_free_priv;
    }
#endif

    err = sx_bitmap_init(&priv->swid_bitmap, NUMBER_OF_SWIDS);
    if (err) {
        sx_err(dev, "Failed to initialize SWIDs bitmap, aborting.\n");
        goto catas_stop;
    }
        
    set_default_capabilities(dev);
    err = sx_core_register_device(dev);
    if (err) {
        sx_err(dev, "Failed to register the device, aborting.\n");
        goto catas_stop;
    }
    memset(&dev->stats, 0, sizeof(dev->stats));    

#ifdef NO_PCI
    err = sx_setup_sx(dev);
    if (err) {
        sx_err(dev, "Failed to in sx_setup_sx, aborting.\n");
        goto out_unregister;
    }

    spin_lock(&sx_glb.pci_devs_lock);
    list_add(&dev->list, &sx_glb.pci_devs_list);
    spin_unlock(&sx_glb.pci_devs_lock);
#endif

    if (sx_priv != NULL) {
        *sx_priv = priv;
    }

    return 0;

#ifdef NO_PCI
out_unregister:
    sx_core_unregister_device(dev);
    priv->unregistered = 1;
#endif

catas_stop:
#if 0
    sx_core_catas_cleanup(dev);
#endif

out_free_priv:
    vfree(priv);

out:
    return err;
}

static void sx_core_remove_one(struct sx_priv *priv)
{
    struct sx_dev *dev;

    if (priv == NULL) {
        dev = sx_glb.tmp_dev_ptr;
        sx_glb.tmp_dev_ptr = NULL;
        priv = sx_priv(dev);
    } else {
        dev = &priv->dev;
    }

    if (!dev) {
        return;
    }

    if (!priv->unregistered) {
        sx_core_unregister_device(dev);
        priv->unregistered = 1;
    }
#if 0
    sx_core_catas_cleanup(dev);
#endif
#ifdef NO_PCI
    spin_lock(&sx_glb.pci_devs_lock);
    list_del(&dev->list);
    spin_unlock(&sx_glb.pci_devs_lock);

    /*Freeing the memmory for the device tables*/
    sx_destroy_sx(dev);
#endif

    sx_dpt_remove_dev(dev->device_id, 1);

    vfree(priv);
}

static void sx_core_remove_one_pci(struct pci_dev *pdev)
{
    struct sx_priv *priv;
    struct sx_dev  *dev;
    int             i;

    dev = pci_get_drvdata(pdev);

    if (!dev) {
        return;
    }

    spin_lock(&sx_glb.pci_devs_lock);
    list_del(&dev->list);
    spin_unlock(&sx_glb.pci_devs_lock);
    sx_glb.pci_devs_cnt--;

    priv = sx_priv(dev);
    if (!priv->unregistered) {
        sx_core_unregister_device(dev);
        priv->unregistered = 1;
    }

    /* Destroy the cq_credit_thread before we flush the DQs
     * otherwise it can stop pulling completion during the flush process */
    if (priv->cq_table.cq_credit_thread) {
        kthread_stop(priv->cq_table.cq_credit_thread);
        priv->cq_table.cq_credit_thread = NULL;
        for (i = 0; i < dev->dev_cap.max_num_cqs; i++) {
            priv->cq_table.cq_rl_params[i].use_limiter = 0;
        }

        printk(KERN_DEBUG PFX "sx_core_remove_one_pci: cq_credit_thread was killed\n");
    }

    dev->global_flushing = 1;
    sx_flush_dqs(dev, false);
    sx_flush_dqs(dev, true);

    for (i = 0; i < NUMBER_OF_SWIDS; i++) {
        if (sx_bitmap_test(&sx_priv(dev)->swid_bitmap, i)) {
            sx_disable_swid(dev, i);
        }
    }

    sx_core_destroy_rdq_table(dev, true);
    sx_core_destroy_sdq_table(dev, true);
    sx_cmd_use_polling(dev);
    sx_cleanup_eq_table(dev);
    sx_core_destroy_cq_table(dev);
    sx_doorbell_cleanup(dev);
#if 0
    if (dev->flags & SX_FLAG_MSI_X) {
        pci_disable_msix(dev->pdev);
    }
#endif

    sx_UNMAP_FA(dev);
    sx_free_icm(dev, sx_priv(dev)->fw.fw_icm, 0);
    sx_cmd_pool_destroy(dev);
    sx_cmd_unmap(dev);
#if 0
    pci_release_region(pdev, 0);
    pci_disable_device(pdev);
    pci_set_drvdata(pdev, NULL);
#endif
    sx_core_remove_one(priv);
}

int sx_restart_one_pci(struct pci_dev *pdev)
{
    if (pdev == NULL){
        printk(KERN_ERR PFX "sx_restart_one_pci error: pdev == NULL, exit \n");
        return -ENODEV;        
    }
    sx_core_remove_one_pci(pdev);
    return sx_core_init_one_pci(pdev, NULL);
}

struct pci_device_id sx_pci_table[] = {
#ifndef NO_PCI
    /* SwitchX PCI device ID */
    { PCI_VDEVICE(MELLANOX, SWITCHX_PCI_DEV_ID) },

    /* SwitchIB PCI device ID */
    { PCI_VDEVICE(MELLANOX, SWITCH_IB_PCI_DEV_ID) },

    /* Spectrum PCI device ID */
    { PCI_VDEVICE(MELLANOX, SPECTRUM_PCI_DEV_ID) },

    /* SwitchIB2 PCI device ID */
    { PCI_VDEVICE(MELLANOX, SWITCH_IB2_PCI_DEV_ID) },
#endif
    { 0, }
};

MODULE_DEVICE_TABLE(pci, sx_pci_table);

static struct pci_driver sx_driver = {
    .name = DRV_NAME,
    .id_table = sx_pci_table,
    .probe = sx_core_init_one_pci,
	.remove		= sx_core_remove_one_pci
};

int sx_init_char_dev(struct cdev *cdev_p)
{
    int ret = 0;
    int devno, major, minor;

    major = MAJOR(char_dev);
    minor = MINOR(char_dev);
    devno = MKDEV(major, minor);
    printk("%s: Create char dev with major:%d minor:%d \n",
           __func__, major, minor);

    cdev_init(cdev_p, &sx_core_fops);
    cdev_p->owner = THIS_MODULE;

    ret = cdev_add(cdev_p, devno, 1);
    if (ret) {
        printk(KERN_ERR PFX "Couldn't add char device. Aborting... err: %d\n",
               ret);
        goto out;
    }

out:
    return ret;
}

void sx_deinit_char_dev(struct cdev *cdev_p)
{
    printk("Deinit char dev: %p , usage:%d\n",
           cdev_p, cdev_p->count);
    cdev_del(cdev_p);
}

static int __init sx_core_init(void)
{
    int ret = 0;
    int i = 0;

    printk(KERN_INFO "%s", sx_version);

    memset(&sx_glb, 0, sizeof(sx_glb));

#ifndef NO_PCI
#if defined(CONFIG_MLNX460EX) && defined(SNOOP_MISS_WA)
    config_l2_force_snoop();
#endif
#endif

    sx_core_init_proc_fs();
    sx_dpt_init();

    spin_lock_init(&sx_glb.pci_devs_lock);
    INIT_LIST_HEAD(&sx_glb.pci_devs_list);

    spin_lock_init(&sx_glb.listeners_lock);
    for (i = 0; i < NUM_HW_SYNDROMES + 1; i++) {
        INIT_LIST_HEAD(&sx_glb.listeners_db[i].list);
    }

    char_dev = MKDEV(SX_MAJOR, SX_BASE_MINOR);
    ret = register_chrdev_region(char_dev, SX_MAX_DEVICES,
                                 SX_CORE_CHAR_DEVICE_NAME);
    if (ret) {
        printk(KERN_INFO PFX "Couldn't register the default device number. "
               "Trying to allocate one dynamically\n");
        ret = alloc_chrdev_region(&char_dev, SX_BASE_MINOR, SX_MAX_DEVICES,
                                  SX_CORE_CHAR_DEVICE_NAME);
        if (ret) {
            printk(KERN_ERR PFX "Couldn't register device number. "
                   "Aborting...\n");
            goto out_close_proc;
        }
    }

    sx_glb.pci_devs_cnt = 0;

#if defined(NO_PCI) || defined(CONFIG_SX_SGMII_PRESENT)
    ret = sx_core_init_one(&sx_glb.priv);
    if (ret) {
        printk(KERN_ERR PFX "Couldn't initialize the device. "
               "Aborting...\n");
        goto out_cdev;
    }

    if (g_chip_type == 0) {
        printk(KERN_ERR PFX "Chip type is not defined for device.\n");
        goto out_remove_one;
    }

    ret = sx_core_dev_init_switchx_cb(&sx_glb.priv->dev, g_chip_type);
    if (ret) {
        printk(KERN_ERR PFX "callback dev init failed for device (%u)\n",
               sx_glb.priv->dev.profile.dev_id);
        goto out_remove_one;
    }
#endif

    ret = pci_register_driver(&sx_driver);
    if (ret < 0) {
        goto out_remove_one;
    }

    ret = sx_init_char_dev(&sx_glb.cdev);
    if (ret < 0) {
        goto out_unreg_pci;
    }

    return 0;

out_unreg_pci:
    pci_unregister_driver(&sx_driver);

out_remove_one:
#if defined(NO_PCI) || defined(CONFIG_SX_SGMII_PRESENT)
    sx_core_remove_one(sx_glb.priv);

out_cdev:
#endif
    sx_deinit_char_dev(&sx_glb.cdev);
    unregister_chrdev_region(char_dev, SX_MAX_DEVICES);

out_close_proc:
    sx_core_close_proc_fs();

    return ret;
}

static void __exit sx_core_cleanup(void)
{
    unsigned long          flags;
    struct listener_entry *listener;
    struct list_head      *pos, *q;
    int                    entry;

    printk(KERN_INFO PFX "sx_core_cleanup_module \n");

    sx_core_close_proc_fs();
    pci_unregister_driver(&sx_driver);
    sx_deinit_char_dev(&sx_glb.cdev);

#if defined(NO_PCI) || defined(CONFIG_SX_SGMII_PRESENT)
    sx_core_remove_one(sx_glb.priv);
#endif /* #ifdef NO_PCI */

    unregister_chrdev_region(char_dev, SX_MAX_DEVICES);
    sx_dpt_dereg_i2c_ifc();
    if (sx_glb.sx_sgmii.deinit) {
        sx_glb.sx_sgmii.deinit();
    }

    /* delete all remaining listener entries */
    spin_lock_irqsave(&sx_glb.listeners_lock, flags);
    for (entry = 0; entry < NUM_HW_SYNDROMES + 1; entry++) {
        if (!list_empty(&sx_glb.listeners_db[entry].list)) {
            list_for_each_safe(pos, q, &sx_glb.listeners_db[entry].list) {
                listener = list_entry(pos, struct listener_entry, list);
                list_del(pos);
                kfree(listener);
            }
        }
    }
    spin_unlock_irqrestore(&sx_glb.listeners_lock, flags);
}

/************************************************
 *  MODULE init/exit
 ***********************************************/
module_init(sx_core_init);
module_exit(sx_core_cleanup);


/************************************************
 *                  EOF                         *
 ***********************************************/
