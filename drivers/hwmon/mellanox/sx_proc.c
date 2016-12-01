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

#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/wait.h>
#include <net/sock.h>
#include <asm/pgtable.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/mlx_sx/cmd.h>
#include <linux/mlx_sx/device.h>
#include <linux/mlx_sx/kernel_user.h>
#include <linux/netdevice.h>
#include <linux/vmalloc.h>
#include "sx_proc.h"
#include "cq.h"
#include "fw.h"
#include "sx.h"
#include "alloc.h"
#include "sx_dpt.h"

#define DEF_SX_CORE_PROC_DIR						"mlx_sx"
#define DEF_SX_CORE_PROC_CORE_FILE		 	"sx_core"

#define PROC_DUMP(fmt, args...)   printk(KERN_ERR fmt, ## args)

void __sx_proc_dump_swids(struct sx_dev *dev);
void __sx_proc_dump_sdq(struct sx_dev *dev);
void __sx_proc_dump_rdq(struct sx_dev *dev);
void __sx_proc_dump_eq(struct sx_dev *dev);
void __sx_proc_dump_rdq_single(struct sx_dev *dev, int rdq);
void __sx_proc_set_dev_profile(struct sx_dev *dev);
extern void sx_core_dump_synd_tbl(struct sx_dev *dev) ;
//void __dump_stats(struct sx_dev* sx_dev);

void __sx_proc_dump_reg(struct sx_dev *my_dev, unsigned long reg_id, unsigned long max_cnt);
void __sx_proc_dump_emad(struct sx_dev *my_dev, unsigned long emad_type, unsigned long max_cnt, unsigned long direction);
void __sx_proc_show_cq(struct sx_dev *dev, int cq);
void __sx_proc_flush_rdq(struct sx_dev *my_dev, int idx);
void __debug_dump_netdev(struct sx_dev *dev);
void __debug_netdev_traps(struct sx_dev *dev, unsigned long trap_id, unsigned long is_add);
void __set_net_dev_oper_state(char	*dev_name ,	char *oper_state);
void __dump_net_dev_oper_state(void);
static  ssize_t sx_proc_write(struct file *file, const char __user *buffer,
                                size_t count, loff_t *pos);


extern struct sx_globals sx_glb;

extern int rx_debug;
extern int rx_debug_pkt_type;
extern int rx_debug_emad_type;
extern int rx_dump;
extern int rx_dump_cnt;
extern int tx_debug;
extern int tx_debug_pkt_type;
extern int tx_debug_emad_type;
extern int tx_dump;
extern int tx_dump_cnt;

extern int i2c_cmd_dump;
extern int i2c_cmd_op;
extern int i2c_cmd_reg_id;
extern int i2c_cmd_dump_cnt;

int sx_proc_registered;
u8 dbg_buf[100];

extern unsigned int credit_thread_vals[100];
extern unsigned long long arr_count;
extern atomic_t cq_backup_polling_enabled;
extern int debug_cq_backup_poll_cqn;

static const char *ku_pkt_type_str[] = {
	"SX_PKT_TYPE_ETH_CTL_UC", /**< Eth control unicast */
	"SX_PKT_TYPE_ETH_CTL_MC", /**< Eth control multicast */
	"SX_PKT_TYPE_ETH_DATA", /**< Eth data */
	"SX_PKT_TYPE_DROUTE_EMAD_CTL", /**< Directed route emad */
	"SX_PKT_TYPE_EMAD_CTL", /**< Emad */
	"SX_PKT_TYPE_FC_CTL_UC", /**< FC control unicast */
	"SX_PKT_TYPE_FC_CTL_MC", /**< FC control multicast */
	"SX_PKT_TYPE_FCOE_CTL_UC", /**< FC over Eth control unicast */
	"SX_PKT_TYPE_FCOE_CTL_MC", /**< FC over Eth control multicast */
	"SX_PKT_TYPE_IB_RAW_CTL", /**< IB raw control */
	"SX_PKT_TYPE_IB_TRANSPORT_CTL", /**< IB transport control */
	"SX_PKT_TYPE_IB_RAW_DATA", /**< IB raw data */
	"SX_PKT_TYPE_IB_TRANSPORT_DATA", /**< IB transport data */
	"SX_PKT_TYPE_EOIB_CTL", /**< Eth over IB control */
	"SX_PKT_TYPE_FCOIB_CTL" , /**< FC over IB control */
};

void __dump_kdbs(void);

char *sx_proc_str_get_ulong(char *buffer, unsigned long *val)
{
	const char delimiters[] = " .,;:!-";
	char *running;
	char *token;
	running = buffer;

	token = strsep(&running, delimiters);
	if (token == NULL) {
		*val = 0;
		return NULL;
	}

	if (strstr(token, "0x"))
		*val = simple_strtoul(token, NULL, 16);
	else
		*val = simple_strtoul(token, NULL, 10);

	return running;
}


char *sx_proc_str_get_u32(char *buffer, u32 *val32)
{
	const char delimiters[] = " .,;:!-";
	char *running;
	char *token;

	running = (char *)buffer;
	token = strsep(&running, delimiters);
	if (token == NULL) {
		*val32 = 0;
		return NULL;
	}

	if (strstr(token, "0x"))
		*val32 = simple_strtol(token, NULL, 16);
	else
		*val32 = simple_strtol(token, NULL, 10);

	return running;
}


char *sx_proc_str_get_str(char *buffer, char **str)
{
	const char delimiters[] = " ,;:!-";
	char *running;
	char *token;

	running = buffer;
	token = strsep(&running, delimiters);
	if (token == NULL) {
		*str = 0;
		return NULL;
	}

	*str = token;
	return running;
}

static int sx_proc_show(struct seq_file *m, void *v)
{
	struct sx_dev *my_dev = sx_glb.sx_dpt.dpt_info[DEFAULT_DEVICE_ID].sx_pcie_info.sx_dev;
    struct sx_fw  *fw  = &sx_priv(my_dev)->fw;

	seq_printf(m,"\nPROC Version: %08x\n", 0x2);

	seq_printf(m,"fw_ver: %llu, fw_date: %02x:%02x %02x.%02x.%04x\n",
                                  fw->fw_ver & 0xffffUL, fw->fw_hour,fw->fw_minutes, fw->fw_day,fw->fw_month,fw->fw_year);

	seq_printf(m,"out_mb: offset:0x%x, size:0x%x ; in_mb offset:0x%x, size:0x%x\n",
				  fw->local_out_mb_offset, fw->local_out_mb_size,
				  fw->local_in_mb_offset, fw->local_in_mb_size);

	return 0;
}

static int sx_proc_open(struct inode *inode, struct file *file)
{
        return single_open(file, sx_proc_show, NULL);
}

static void sx_proc_handle_access_reg(struct sx_dev *dev, char *p,
		u32 dev_id, u32 reg_id, char *running) {
	int err = 0;
	u32 local_port;

	switch (reg_id) {
	case MGIR_REG_ID: {
		struct ku_access_mgir_reg reg_data;

		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 1);
		PROC_DUMP("Executing ACCESS_REG_MGIR Command\n");
		err = sx_ACCESS_REG_MGIR(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing Query ACCESS_REG_MGIR Command, PSID = %s\n",
				reg_data.mgir_reg.fw_info.psid);
		break;
	}
	case PSPA_REG_ID: {
		struct ku_access_pspa_reg reg_data;
		u32 swid_id;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 2);
		/* local port + sub port are inputs */
		running = sx_proc_str_get_u32(running, &swid_id);
		reg_data.pspa_reg.swid = swid_id;
		running = sx_proc_str_get_u32(running, &local_port);
		reg_data.pspa_reg.local_port = local_port;
		/*
		reg_data.pspa_reg.swid = 0;
		reg_data.pspa_reg.local_port = 1;
		reg_data.pspa_reg.sub_port = 0;
		*/
		PROC_DUMP("Executing ACCESS_REG_PSPA Command, "
				"swid_id:%d, lport:%d\n",
				reg_data.pspa_reg.swid,
				reg_data.pspa_reg.local_port);
		err = sx_ACCESS_REG_PSPA(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing Write ACCESS_REG_PSPA Command."
			"Query not implemented yet\n");
		break;
	}
	case OEPFT_REG_ID: {
		struct ku_access_oepft_reg reg_data;
		int i;

		for (i = 0; i < 2; i++) {
			PROC_DUMP("Executing Query ACCESS_REG_OEPFT Command, "
					"sr = %d\n", i);
			reg_data.oepft_reg.sr = 0;
			memset(&reg_data, 0, sizeof(reg_data));
			reg_data.dev_id = dev_id;
			sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 1);
			err = sx_ACCESS_REG_OEPFT(dev, &reg_data);
			if (err)
				goto print_err;

			PROC_DUMP("Finished Executing Query ACCESS_REG_OEPFT "
					"Command.\n");
			PROC_DUMP("cpu_tclass = 0x%x\n",
					reg_data.oepft_reg.cpu_tclass);
			PROC_DUMP("flow_number = 0x%x\n",
					reg_data.oepft_reg.flow_number);
			PROC_DUMP("interface = 0x%x\n",
					reg_data.oepft_reg.interface);
			PROC_DUMP("mac = 0x%llx\n",
					reg_data.oepft_reg.mac);
		}
		break;
	}
	case PTYS_REG_ID: {
		struct ku_access_ptys_reg reg_data, reg_data2;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 2);
		/* local port is the input */
		running = sx_proc_str_get_u32(running, &local_port);
		reg_data.ptys_reg.local_port = local_port;
		running = sx_proc_str_get_u32(running,
				&reg_data.ptys_reg.eth_proto_admin);
		running = sx_proc_str_get_u32(running,
				&reg_data.ptys_reg.eth_proto_oper);
		reg_data.ptys_reg.proto_mask = (1<<2);
		PROC_DUMP("Executing ACCESS_REG_PTYS Command: loc port:%d, "
				"eth_proto_adm:0x%x, proto_oper:0x%x, "
				"proto_mask:%d\n",
				reg_data.ptys_reg.local_port,
				reg_data.ptys_reg.eth_proto_admin,
				reg_data.ptys_reg.eth_proto_oper,
				reg_data.ptys_reg.proto_mask);
		err = sx_ACCESS_REG_PTYS(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing Write ACCESS_REG_PTYS Command. "
			"Now executing Query:\n");
		memset(&reg_data, 0, sizeof(reg_data));
		memset(&reg_data2, 0, sizeof(reg_data2));
		reg_data2.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data2.op_tlv, reg_id, 1);
		reg_data2.ptys_reg.local_port = 1;
		reg_data2.ptys_reg.proto_mask = (1<<2);
		err = sx_ACCESS_REG_PTYS(dev, &reg_data2);
		if (err)
			goto print_err;
		PROC_DUMP("fc_proto_capability = %x\n",
				reg_data2.ptys_reg.fc_proto_capability);
		PROC_DUMP("eth_proto_capability = %x\n",
				reg_data2.ptys_reg.eth_proto_capability);
		PROC_DUMP("ib_proto_capability = %x\n",
				reg_data2.ptys_reg.ib_proto_capability);
		PROC_DUMP("fc_proto_admin = %x\n",
				reg_data2.ptys_reg.fc_proto_admin);
		PROC_DUMP("eth_proto_admin = %x\n",
				reg_data2.ptys_reg.eth_proto_admin);
		PROC_DUMP("ib_proto_admin = %x\n",
				reg_data2.ptys_reg.ib_proto_admin);
		PROC_DUMP("fc_proto_oper = %x\n",
				reg_data2.ptys_reg.fc_proto_oper);
		PROC_DUMP("eth_proto_oper = %x\n",
				reg_data2.ptys_reg.eth_proto_oper);
		PROC_DUMP("ib_proto_oper = %x\n",
				reg_data2.ptys_reg.ib_proto_oper);
		break;
	}
	case PMLP_REG_ID: {
		struct ku_access_pmlp_reg reg_data, reg_data2;
		int i;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 2);
		/* local port is the input */
		reg_data.pmlp_reg.local_port = 1;
		reg_data.pmlp_reg.width = 4;
		for (i = 0; i < 4; i++) {
			reg_data.pmlp_reg.lane[i] = i;
			reg_data.pmlp_reg.module[i] = 3;
		}
		PROC_DUMP("Executing ACCESS_REG_PMLP Command\n");
		err = sx_ACCESS_REG_PMLP(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing Write ACCESS_REG_PMLP Command. "
			"Now executing Query:\n");
		memset(&reg_data2, 0, sizeof(reg_data2));
		reg_data2.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data2.op_tlv, reg_id, 1);
		reg_data2.pmlp_reg.local_port = 1;
		err = sx_ACCESS_REG_PMLP(dev, &reg_data2);
		if (err)
			goto print_err;

		PROC_DUMP("width = %u\n", reg_data2.pmlp_reg.width);
		PROC_DUMP("lane0 = %u\n", reg_data2.pmlp_reg.lane[0]);
		PROC_DUMP("module0 = %u\n", reg_data2.pmlp_reg.module[0]);
		PROC_DUMP("lane1 = %u\n", reg_data2.pmlp_reg.lane[1]);
		PROC_DUMP("module1 = %u\n", reg_data2.pmlp_reg.module[1]);
		PROC_DUMP("lane2 = %u\n", reg_data2.pmlp_reg.lane[2]);
		PROC_DUMP("module2 = %u\n", reg_data2.pmlp_reg.module[2]);
		PROC_DUMP("lane3 = %u\n", reg_data2.pmlp_reg.lane[3]);
		PROC_DUMP("module3 = %u\n", reg_data2.pmlp_reg.module[3]);
		break;
	}
	case PLIB_REG_ID: {
		struct ku_access_plib_reg reg_data;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 2);
		/* local port is the input */
		running = sx_proc_str_get_u32(running, &local_port);
                reg_data.plib_reg.local_port = local_port;
                running = sx_proc_str_get_u32(running, &local_port);
		reg_data.plib_reg.ib_port = local_port;
		PROC_DUMP("Executing ACCESS_REG_PLIB SET Command, local_port = %u, ib_port = %u\n",
				reg_data.plib_reg.local_port, reg_data.plib_reg.ib_port);
		err = sx_ACCESS_REG_PLIB(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing ACCESS_REG_PLIB SET Command:\n");
                sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 1);
		reg_data.plib_reg.ib_port = 0;
                PROC_DUMP("Executing ACCESS_REG_PLIB GET Command, local_port = %u\n", reg_data.plib_reg.local_port);
                err = sx_ACCESS_REG_PLIB(dev, &reg_data);
		PROC_DUMP("ib_port = %u\n", reg_data.plib_reg.ib_port);
		break;
	}
	case PAOS_REG_ID: {
		struct ku_access_paos_reg reg_data, reg_data2;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 2);
		reg_data.paos_reg.local_port = 1;
		reg_data.paos_reg.admin_status = 2;
		reg_data.paos_reg.oper_status = 0; /* read only field */
		reg_data.paos_reg.ase = 1;
		reg_data.paos_reg.ee = 0;
		reg_data.paos_reg.e = 0;
		PROC_DUMP("Executing ACCESS_REG_PAOS Command\n");
		err = sx_ACCESS_REG_PAOS(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing Write ACCESS_REG_PAOS Command. "
			"Now executing query:\n");
		memset(&reg_data2, 0, sizeof(reg_data2));
		reg_data2.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data2.op_tlv, reg_id, 1);
		reg_data2.paos_reg.local_port = 1;
		err = sx_ACCESS_REG_PAOS(dev, &reg_data2);
		if (err)
			goto print_err;

		PROC_DUMP("admin_status = %u\n",
			reg_data2.paos_reg.admin_status);
		PROC_DUMP("oper_status = %u\n",
				reg_data2.paos_reg.oper_status);
		PROC_DUMP("ase = %u\n", reg_data2.paos_reg.ase);
		PROC_DUMP("ee = %u\n", reg_data2.paos_reg.ee);
		PROC_DUMP("e = %u\n", reg_data2.paos_reg.e);
		break;
	}
	case PMPR_REG_ID: {
		struct ku_access_pmpr_reg reg_data, reg_data2;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 2);
		reg_data.pmpr_reg.module = 1;
		reg_data.pmpr_reg.attenuation5g = 2;
		PROC_DUMP("Executing ACCESS_REG_PMPR Command\n");
		err = sx_ACCESS_REG_PMPR(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing Write ACCESS_REG_PMPR Command. "
			"Now executing query:\n");
		memset(&reg_data2, 0, sizeof(reg_data2));
		reg_data2.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data2.op_tlv, reg_id, 1);
		reg_data2.pmpr_reg.module = 1;
		err = sx_ACCESS_REG_PMPR(dev, &reg_data2);
		if (err)
			goto print_err;

		PROC_DUMP("module = %u\n",
			reg_data2.pmpr_reg.module);
		PROC_DUMP("Attenuation5G = %u\n",
				reg_data2.pmpr_reg.attenuation5g);
		break;
	}
	case PMTU_REG_ID: {
		struct ku_access_pmtu_reg reg_data, reg_data2;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 2);
		reg_data.pmtu_reg.local_port = 1;
		reg_data.pmtu_reg.admin_mtu = 638;
		PROC_DUMP("Executing ACCESS_REG_PMTU Command\n");
		err = sx_ACCESS_REG_PMTU(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing Write ACCESS_REG_PMTU Command. "
			"Now executing Query:\n");
		memset(&reg_data2, 0, sizeof(reg_data2));
		reg_data2.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data2.op_tlv, reg_id, 1);
		reg_data2.pmtu_reg.local_port = 1;
		err = sx_ACCESS_REG_PMTU(dev, &reg_data2);
		if (err)
			goto print_err;

		PROC_DUMP("max_mtu = %u\n", reg_data2.pmtu_reg.max_mtu);
		PROC_DUMP("admin_mtu = %u\n", reg_data2.pmtu_reg.admin_mtu);
		PROC_DUMP("oper_mtu = %u\n", reg_data2.pmtu_reg.oper_mtu);
		break;
	}
	case PELC_REG_ID: {
		u32 operation; /* 1 = QUERY 2 = WRITE*/
		u32 op = 0;
		unsigned long admin, request;
		struct ku_access_pelc_reg reg_data;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		running = sx_proc_str_get_u32(running, &operation);
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, operation);
		/* local port and op are the input */
		running = sx_proc_str_get_u32(running, &local_port);
		running = sx_proc_str_get_u32(running, &op);
		reg_data.pelc_reg.local_port = local_port;
		reg_data.pelc_reg.op = op;
		if (operation == 2) {
			running = sx_proc_str_get_ulong(running, &admin);
			running = sx_proc_str_get_ulong(running, &request);
			reg_data.pelc_reg.admin = 0x10ULL;
			reg_data.pelc_reg.request = 0x10ULL;
			PROC_DUMP("Read admin = 0x%llx, request = 0x%llx\n", reg_data.pelc_reg.admin, reg_data.pelc_reg.request);
			PROC_DUMP("Executing ACCESS_REG_PELC Set Command\n");
	                err = sx_ACCESS_REG_PELC(dev, &reg_data);
        	        if (err)
	                        goto print_err;

/*			sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 1); */
		}
/*		reg_data.pelc_reg.admin = 0;
		reg_data.pelc_reg.request = 0; */
		else {
			PROC_DUMP("Executing ACCESS_REG_PELC Get Command\n");
			err = sx_ACCESS_REG_PELC(dev, &reg_data);
			if (err)
				goto print_err;

			PROC_DUMP("Finished Executing ACCESS_REG_PELC Get Command:\n");
			PROC_DUMP("admin = %llx\n", reg_data.pelc_reg.admin);
			PROC_DUMP("capability = %llx\n", reg_data.pelc_reg.capability);
			PROC_DUMP("request = %llx\n", reg_data.pelc_reg.request);
			PROC_DUMP("active = %llx\n", reg_data.pelc_reg.active);
		}
		break;
	}
	case PVLC_REG_ID: {
		struct ku_access_pvlc_reg reg_data, reg_data2;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 2);
		/* local port is the input */
		reg_data.pvlc_reg.local_port = 1;
		reg_data.pvlc_reg.vl_admin = 3;
		PROC_DUMP("Executing ACCESS_REG_PVLC Command\n");
		err = sx_ACCESS_REG_PVLC(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing Write ACCESS_REG_PVLC Command. "
				"Now executing Query:\n");
		memset(&reg_data2, 0, sizeof(reg_data2));
		reg_data2.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data2.op_tlv, reg_id, 1);
		reg_data2.pvlc_reg.local_port = 1;
		err = sx_ACCESS_REG_PVLC(dev, &reg_data2);
		if (err)
			goto print_err;

		PROC_DUMP("vl_cap = %u\n", reg_data2.pvlc_reg.vl_cap);
		PROC_DUMP("vl_admin = %u\n", reg_data2.pvlc_reg.vl_admin);
		PROC_DUMP("vl_oper = %u\n", reg_data2.pvlc_reg.vl_operational);
		break;
	}
	case HPKT_REG_ID: {
		struct ku_access_hpkt_reg reg_data, reg_data2;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 2);
		/* trap_id is the input */
		reg_data.hpkt_reg.action = 1;
		reg_data.hpkt_reg.trap_group = 2;
		reg_data.hpkt_reg.trap_id = 5;
		PROC_DUMP("Executing ACCESS_REG_HPKT Command\n");
		err = sx_ACCESS_REG_HPKT(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing Write ACCESS_REG_HPKT Command. "
			"Now executing Query:\n");
		memset(&reg_data2, 0, sizeof(reg_data2));
		reg_data2.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data2.op_tlv, reg_id, 1);
		reg_data2.hpkt_reg.trap_id = 0x10;
		err = sx_ACCESS_REG_HPKT(dev, &reg_data2);
		if (err)
			goto print_err;

		PROC_DUMP("action = %u\n", reg_data2.hpkt_reg.action);
		PROC_DUMP("trap_group = %u\n", reg_data2.hpkt_reg.trap_group);
		break;
	}
	case HCAP_REG_ID: {
		struct ku_access_hcap_reg reg_data;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 1);
		PROC_DUMP("Executing ACCESS_REG_HCAP Command\n");
		err = sx_ACCESS_REG_HCAP(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing ACCESS_REG_HCAP Command:\n");
		PROC_DUMP("max_num_cpu_tclass = %u\n",
				reg_data.hcap_reg.max_cpu_ingress_tclass);
		PROC_DUMP("max_num_trap_groups = %u\n",
				reg_data.hcap_reg.max_num_trap_groups);
		PROC_DUMP("max_num_dr_paths = %u\n",
				reg_data.hcap_reg.max_num_dr_paths);
		break;
	}
	case HDRT_REG_ID: {
		struct ku_access_hdrt_reg reg_data;
		u32 dr_index;
		int i;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 1);
		/* dr_index port is the input */
		sx_proc_str_get_u32(running, &dr_index);
		reg_data.hdrt_reg.dr_index = (u8)dr_index;
		PROC_DUMP("Executing ACCESS_REG_HDRT Command\n");
		err = sx_ACCESS_REG_HDRT(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing ACCESS_REG_HDRT Command:\n");
		PROC_DUMP("hop_cnt = %u\n", reg_data.hdrt_reg.hop_cnt);
		for (i = 0; i < 64; i++)
			PROC_DUMP("path %d = %u\n", i,
					reg_data.hdrt_reg.path[i]);
		for (i = 0; i < 64; i++)
			PROC_DUMP("rpath %d = %u\n", i,
					reg_data.hdrt_reg.rpath[i]);
		break;
	}
	case MFSC_REG_ID: {
		struct ku_access_mfsc_reg reg_data;
		u32 fan;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 1);
		/* fan is the input */
		running = sx_proc_str_get_u32(running, &fan);
		reg_data.mfsc_reg.pwm = (u8)(fan & 0x7);
		PROC_DUMP("Executing ACCESS_REG_MFSC Command , dev_id: %d \n", reg_data.dev_id);
		err = sx_ACCESS_REG_MFSC(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing ACCESS_REG_MFSC Command:\n");
		PROC_DUMP("pwm_duty_cycle = %u\n",
				reg_data.mfsc_reg.pwm_duty_cycle);
		break;
	}
	case MFSM_REG_ID: {
		struct ku_access_mfsm_reg reg_data;
		u32 tacho;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 1);
		/* tacho is the input */
		running = sx_proc_str_get_u32(running, &tacho);
		reg_data.mfsm_reg.tacho = (u8)(tacho & 0x7);
		PROC_DUMP("Executing ACCESS_REG_MFSM Command, dev_id: %d\n", dev_id);
		err = sx_ACCESS_REG_MFSM(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing ACCESS_REG_MFSM Command:\n");
		PROC_DUMP("n = %u\n", reg_data.mfsm_reg.n);
		PROC_DUMP("rpm = %u\n", reg_data.mfsm_reg.rpm);
		break;
	}
	case MFSL_REG_ID: {
		struct ku_access_mfsl_reg reg_data;
		u32 tacho;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 1);
		/* tacho is the input */
		running = sx_proc_str_get_u32(running, &tacho);
		reg_data.mfsl_reg.fan = (u8)(tacho & 0x7);
		PROC_DUMP("Executing ACCESS_REG_MFSL Command\n");
		err = sx_ACCESS_REG_MFSL(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing ACCESS_REG_MFSL Command:\n");
		PROC_DUMP("ee = %u\n", reg_data.mfsl_reg.ee);
		PROC_DUMP("ie = %u\n", reg_data.mfsl_reg.ie);
		PROC_DUMP("tach_min = %u\n", reg_data.mfsl_reg.tach_min);
		PROC_DUMP("tach_max = %u\n", reg_data.mfsl_reg.tach_max);
		break;
	}
	case SPZR_REG_ID: {
		struct ku_access_spzr_reg reg_data;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 1);
		reg_data.spzr_reg.EnhSwP0 = 1;
		reg_data.spzr_reg.EnhSwP0_mask = 1;
		PROC_DUMP("Executing ACCESS_REG_SPZR Command\n");
		err = sx_ACCESS_REG_SPZR(dev, &reg_data);
		break;
	}
	case HTGT_REG_ID: {
		struct ku_access_htgt_reg reg_data, reg_data2;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 2);
		PROC_DUMP("Executing ACCESS_REG_HTGT Command\n");
		reg_data.htgt_reg.swid = 254;
		reg_data.htgt_reg.type = 0;
		reg_data.htgt_reg.trap_group = 2;
		reg_data.htgt_reg.path.local_path.rdq = 21;
		reg_data.htgt_reg.path.local_path.cpu_tclass = 1;
		err = sx_ACCESS_REG_HTGT(dev, &reg_data);
		if (err)
			goto print_err;

		PROC_DUMP("Finished Executing Write ACCESS_REG_HTGT Command. "
				"Now executing Query:\n");
		memset(&reg_data2, 0, sizeof(reg_data2));
		reg_data2.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data2.op_tlv, reg_id, 1);
		reg_data2.htgt_reg.swid = 0;
		reg_data2.htgt_reg.trap_group = 2;
		err = sx_ACCESS_REG_HTGT(dev, &reg_data2);
		if (err)
			goto print_err;

		PROC_DUMP("swid = %u\n", reg_data2.htgt_reg.swid);
		PROC_DUMP("type = %u\n", reg_data2.htgt_reg.type);
		PROC_DUMP("grp = %u\n", reg_data2.htgt_reg.trap_group);
		PROC_DUMP("rdq = %u\n",
				reg_data2.htgt_reg.path.local_path.rdq);
		PROC_DUMP("cpu_tclass = %u\n",
				reg_data2.htgt_reg.path.local_path.cpu_tclass);
		break;
	}
	case MCIA_REG_ID:
	case QSPTC_REG_ID:
	case QSTCT_REG_ID:
		printk(KERN_WARNING "%s() reg_id %u not implemented yet\n",
			   __func__, reg_id);
		break;

	case MPSC_REG_ID: {
		struct ku_access_mpsc_reg reg_data, reg_data2;
		u32 rate;
		u32 enable;
		memset(&reg_data, 0, sizeof(reg_data));
		reg_data.dev_id = dev_id;

		running = sx_proc_str_get_u32(running, &local_port);
		reg_data.mpsc_reg.local_port = local_port;
		running = sx_proc_str_get_u32(running, &rate);
		reg_data.mpsc_reg.rate = rate;
		running = sx_proc_str_get_u32(running, &enable);
		reg_data.mpsc_reg.enable = (u8)enable;

		sx_cmd_set_op_tlv(&reg_data.op_tlv, reg_id, 2);
		PROC_DUMP("Executing ACCESS_REG_MPSC Write Command\n");

		err = sx_ACCESS_REG_MPSC(dev, &reg_data);
		PROC_DUMP();

		PROC_DUMP("Finished Executing Write ACCESS_REG_MPSC Command. "
				"Now executing Query:\n");
		memset(&reg_data2, 0, sizeof(reg_data2));
		reg_data2.dev_id = dev_id;
		sx_cmd_set_op_tlv(&reg_data2.op_tlv, reg_id, 1);

		err = sx_ACCESS_REG_MPSC(dev, &reg_data2);
		if (err)
			goto print_err;
		PROC_DUMP("Read rate = 0x%08u, count_drops = %lld\n", reg_data2.mpsc_reg.rate, reg_data2.mpsc_reg.count_drops);
		break;

	}

	default:
		printk(KERN_WARNING "%s() reg_id %u doesn't exist\n",
			   __func__, reg_id);

	}

	return;

print_err:
	printk(KERN_WARNING "%s() execution of ACCESS_REG on reg_id %x Failed."
			"error code = %d \n", __func__, reg_id, err);

}

static void check_netdev_name(char *name) {
	/* whitespaces are not allowed in netdevice names */
	while (*name) {
		if (*name == '/' || isspace(*name)) {
			*name = '\0';
			break;
		}

		name++;
	}

}
void __rem_port_netdev(struct sx_dev *my_dev, char *running)
{
    /* We only need the sysport */
    int sysport;
    union sx_event_data event_data;
    running = sx_proc_str_get_u32(running, &sysport);
    event_data.port_netdev_set.sysport = sysport;
    PROC_DUMP("Remove port netdev, sysport: %d \n", sysport);
    sx_core_dispatch_event(my_dev, SX_DEV_EVENT_CLOSE_PORT_NETDEV, &event_data);
}

static ssize_t sx_proc_write(struct file *file, const char __user *buffer,
				size_t count, loff_t *pos)
{
	int cmd = 0;
	char *p;
	char *running;
	char *cmd_str;
	struct sx_dev *my_dev = sx_glb.
			sx_dpt.dpt_info[DEFAULT_DEVICE_ID].
			sx_pcie_info.sx_dev;

	running = (char *)buffer;
	running = sx_proc_str_get_str(running, &cmd_str);
	p = strstr(cmd_str, "dump_dpt");
	if (p != NULL) {
		/* PROC_DUMP("  call to dump DPT table !!!\n"); */
		sx_dpt_dump();
		cmd++;
	}

	p = strstr(cmd_str, "add_port_netdev");
	if (p != NULL) {
		int sysport;
		char *name;
		union sx_event_data event_data;
		running = sx_proc_str_get_u32(running, &sysport);
		running = sx_proc_str_get_str(running, &name);
		check_netdev_name(name);
		event_data.port_netdev_set.sysport = sysport;
		event_data.port_netdev_set.name = name;
		event_data.port_netdev_set.swid = 1;
		PROC_DUMP("Add port netdev, sysport: 0x%x name: %s \n", sysport, name);
		sx_core_dispatch_event(my_dev, SX_DEV_EVENT_OPEN_PORT_NETDEV, &event_data);
		cmd++;
	}

	p = strstr(cmd_str, "rem_port_netdev");
	if (p != NULL) {
        __rem_port_netdev(my_dev, running);
		cmd++;
	}

	p = strstr(cmd_str, "dpt_setp");
	if (p != NULL) {
		int dev_id, path_type;
		running = sx_proc_str_get_u32(running, &dev_id);
		running = sx_proc_str_get_u32(running, &path_type);
		PROC_DUMP("Change dpt dev_id:%d , path_type: %d  \n",
				dev_id, path_type);
		sx_dpt_set_cmd_dbg(dev_id, path_type);
		cmd++;
	}

	p = strstr(cmd_str, "dpt_set_cr");
        if (p != NULL) {
                int dev_id, path_type;
                running = sx_proc_str_get_u32(running, &dev_id);
                running = sx_proc_str_get_u32(running, &path_type);
                PROC_DUMP("Change dpt dev_id:%d , CR path_type: %d  \n",
                                dev_id, path_type);
                sx_dpt_set_cr_access_path(dev_id, path_type);
                cmd++;
        }

	p = strstr(cmd_str, "cr_space_write");
        if (p != NULL) {
                int dev_id;
		unsigned int address;
		unsigned int value;
                running = sx_proc_str_get_u32(running, &dev_id);
                running = sx_proc_str_get_u32(running, &address);
		running = sx_proc_str_get_u32(running, &value);
                PROC_DUMP("write dword to cr_space dev_id:%d , address: 0x%x , value: %d \n",
                                dev_id, address, value);
                sx_dpt_cr_space_write(dev_id, address, (unsigned char *)&value, 4);
                cmd++;
        }

	p = strstr(cmd_str, "cr_space_read");
        if (p != NULL) {
                int dev_id;
		unsigned int address;
		unsigned int value = 0;
                running = sx_proc_str_get_u32(running, &dev_id);
                running = sx_proc_str_get_u32(running, &address);
                PROC_DUMP("read dword from cr_space dev_id:%d , address: 0x%x\n",
                                dev_id, address);
                sx_dpt_cr_space_read(dev_id, address, (unsigned char *)&value, 4);
		PROC_DUMP("cr_space read: dev_id:%d , address: 0x%x , value: %u \n",
                                dev_id, address, value);
                cmd++;
        }

	p = strstr(cmd_str, "q_fw");
	if (p != NULL) {
		struct ku_query_fw fw;
		int dev_id;
		running = sx_proc_str_get_u32(running, &dev_id);
		if (dev_id == 0)
			dev_id = my_dev->device_id;

		if (sx_glb.sx_dpt.dpt_info[dev_id].sx_pcie_info.sx_dev != NULL)
		    my_dev = sx_glb.sx_dpt.dpt_info[dev_id].sx_pcie_info.sx_dev;

		fw.dev_id = dev_id;

		PROC_DUMP("Executing QUERY_FW Command to dev %d !\n", dev_id);
		sx_QUERY_FW(my_dev, &fw);
		PROC_DUMP("fw_rev = 0x%llx\n", fw.fw_rev);
		PROC_DUMP("debug_trace = 0x%x\n", fw.dt);
		PROC_DUMP("core_clock = 0x%x\n", fw.core_clk);
		PROC_DUMP("fw_hour = 0x%x\n", fw.fw_hour);
		PROC_DUMP("fw_minutes = 0x%x\n", fw.fw_minutes);
		PROC_DUMP("fw_seconds = 0x%x\n", fw.fw_seconds);
		PROC_DUMP("fw_year = 0x%x\n", fw.fw_year);
		PROC_DUMP("fw_month = 0x%x\n", fw.fw_month);
		PROC_DUMP("fw_day = 0x%x\n", fw.fw_day);

		PROC_DUMP("Executing QUERY_FW_2 Command\n");
		sx_QUERY_FW_2(my_dev, dev_id);
		PROC_DUMP("Finished Executing QUERY_FW_2 Command:\n");
		PROC_DUMP("local_out_mb_offset = 0x%x\n",
				sx_glb.sx_dpt.dpt_info[dev_id].out_mb_offset);
		PROC_DUMP("local_out_mb_size = 0x%x\n",
				sx_glb.sx_dpt.dpt_info[dev_id].out_mb_size);
		PROC_DUMP("local_in_mb_offset = 0x%x\n",
				sx_glb.sx_dpt.dpt_info[dev_id].in_mb_offset);
		PROC_DUMP("local_in_mb_size = 0x%x\n",
				sx_glb.sx_dpt.dpt_info[dev_id].in_mb_size);
		cmd++;
	}

	p = strstr(cmd_str, "q_board_info");
	if (p != NULL) {
		struct sx_board board;

		memset(&board, 0, sizeof(board));
		PROC_DUMP("Executing QUERY_BOARDINFO Command\n");
		sx_QUERY_BOARDINFO(my_dev, &board);
		PROC_DUMP("Finished Executing QUERY_BOARDINFO Command:\n");
		PROC_DUMP("vsd_vendor_id = 0x%x\n", board.vsd_vendor_id);
		PROC_DUMP("board_id = %s\n", board.board_id);
		PROC_DUMP("inta_pin = 0x%x\n", board.inta_pin);
		cmd++;
	}

	p = strstr(cmd_str, "intr");
	if (p != NULL) {
		struct sx_board board;

		memset(&board, 0, sizeof(board));
		PROC_DUMP("Executing QUERY_BOARDINFO Command\n");
		sx_cmd_use_events(my_dev);
		sx_QUERY_BOARDINFO(my_dev, &board);
		sx_cmd_use_polling(my_dev);
		PROC_DUMP("Finished Executing QUERY_BOARDINFO Command:\n");
		PROC_DUMP("vsd_vendor_id = 0x%x\n", board.vsd_vendor_id);
		PROC_DUMP("board_id = %s\n", board.board_id);
		PROC_DUMP("inta_pin = 0x%x\n", board.inta_pin);
		cmd++;
	}


	p = strstr(cmd_str, "q_aq_cap");
	if (p != NULL) {
		struct sx_dev_cap *dev_cap = &my_dev->dev_cap;

		PROC_DUMP("Executing QUERY_AQ_CAP Command\n");
		sx_QUERY_AQ_CAP(my_dev);
		PROC_DUMP("Finished Executing QUERY_AQ_CAP Command:\n");
		PROC_DUMP("log_max_rdq_sz = %d\n", dev_cap->log_max_rdq_sz);
		PROC_DUMP("log_max_sdq_sz = %d\n", dev_cap->log_max_sdq_sz);
		PROC_DUMP("log_max_cq_sz = %d\n", dev_cap->log_max_cq_sz);
		PROC_DUMP("log_max_eq_sz = %d\n", dev_cap->log_max_eq_sz);
		PROC_DUMP("max_num_rdqs = %u\n", dev_cap->max_num_rdqs);
		PROC_DUMP("max_num_sdqs = %u\n", dev_cap->max_num_sdqs);
		PROC_DUMP("max_num_cqs = %u\n", dev_cap->max_num_cqs);
		PROC_DUMP("max_num_eqs = %u\n", dev_cap->max_num_eqs);
		PROC_DUMP("max_sg_sq = %u\n", dev_cap->max_sg_sq);
		PROC_DUMP("max_sg_rq = %u\n", dev_cap->max_sg_rq);
		cmd++;
	}

	p = strstr(cmd_str, "get_profile");
	if (p != NULL) {
		struct ku_profile *profile;

		profile = vmalloc(sizeof(*profile));
		if(profile == NULL){
			printk("Error allocate mem for profile\n");
			goto out;
		}
		memset(profile, 0, sizeof(*profile));
		PROC_DUMP("Executing GET_PROFILE Command\n");
		sx_GET_PROFILE(my_dev, profile);
		PROC_DUMP("Finished Executing GET_PROFILE Command:\n");
		PROC_DUMP("max_vepa_channels = %u\n",
				profile->max_vepa_channels);
		PROC_DUMP("max_lag = %u\n", profile->max_lag);
		PROC_DUMP("max_port_per_lag = %u\n", profile->max_port_per_lag);
		PROC_DUMP("max_mid = %u\n", profile->max_mid);
		PROC_DUMP("max_pgt = %u\n", profile->max_pgt);
		PROC_DUMP("max_system_port = %u\n", profile->max_system_port);
		PROC_DUMP("max_active_vlans = %u\n", profile->max_active_vlans);
		PROC_DUMP("max_regions = %u\n", profile->max_regions);
		PROC_DUMP("max_flood_tables = %u\n", profile->max_flood_tables);
		PROC_DUMP("max_per_vid_flood_tables = %u\n", profile->max_per_vid_flood_tables);
		PROC_DUMP("flood mode = %u\n", profile->flood_mode);
		PROC_DUMP("max_fid_offset_flood_tables = %u\n", profile->max_fid_offset_flood_tables);
		PROC_DUMP("fid_offset_table_size = %u\n", profile->fid_offset_table_size);
		PROC_DUMP("max_per_fid_flood_table = %u\n", profile->max_per_fid_flood_table);
		PROC_DUMP("per_fid_table_size = %u\n", profile->per_fid_table_size);
		PROC_DUMP("max_ib_mc = %u\n", profile->max_ib_mc);
		PROC_DUMP("max_pkey = %u\n", profile->max_pkey);
		PROC_DUMP("ar_sec = %u\n", profile->ar_sec);
		PROC_DUMP("adaptive_routing_group_cap = %u\n", profile->adaptive_routing_group_cap);
		PROC_DUMP("arn = %u\n", profile->arn);
		PROC_DUMP("kvd_linear_size = %u\n", profile->kvd_linear_size);
		PROC_DUMP("kvd_hash_single_size = %u\n", profile->kvd_hash_single_size);
		PROC_DUMP("kvd_hash_double_size = %u\n", profile->kvd_hash_double_size);
		PROC_DUMP("switch_0_type = %u\n",
				profile->swid0_config_type.type);
		PROC_DUMP("switch_1_type = %u\n",
				profile->swid1_config_type.type);
		PROC_DUMP("switch_2_type = %u\n",
				profile->swid2_config_type.type);
		PROC_DUMP("switch_3_type = %u\n",
				profile->swid3_config_type.type);
		PROC_DUMP("switch_4_type = %u\n",
				profile->swid4_config_type.type);
		PROC_DUMP("switch_5_type = %u\n",
				profile->swid5_config_type.type);
		PROC_DUMP("switch_6_type = %u\n",
				profile->swid6_config_type.type);
		PROC_DUMP("switch_7_type = %u\n",
				profile->swid7_config_type.type);
		vfree(profile);
		cmd++;
	}

	p = strstr(cmd_str, "access_reg");
	if (p != NULL) {
		u32 reg_id = 0;
		u32 dev_id = 0;
		sx_cmd_use_polling(my_dev);
		running = sx_proc_str_get_u32(running, &dev_id);
		running = sx_proc_str_get_u32(running, &reg_id);
		sx_proc_handle_access_reg(my_dev, p, dev_id, reg_id, running);
		cmd++;
	}

	p = strstr(cmd_str, "get_pci");
	if (p != NULL) {
		unsigned int sx_pci_dev_id, vendor, device;
		struct pci_dev *sx_pci_dev;
		running = sx_proc_str_get_u32(running, &sx_pci_dev_id);
		running = sx_proc_str_get_u32(running, &vendor);
		running = sx_proc_str_get_u32(running, &device);
		sx_dpt_find_pci_dev(sx_pci_dev_id, vendor, device, &sx_pci_dev);
		cmd++;
	}

	p = strstr(cmd_str, "mem_wr");
	if (p != NULL) {
		unsigned long  width;
		unsigned long  addr, val;
		running = sx_proc_str_get_ulong(running, &width);
		running = sx_proc_str_get_ulong(running, &addr);
		running = sx_proc_str_get_ulong(running, &val);
		printk("Write value %lx to addr: 0x%lx  \n", val, addr);
		sx_proc_dbg_wr(width, addr, val);
		cmd++;
	}

	p = strstr(cmd_str, "mem_rd ");
	if (p != NULL) {
		unsigned long width;
		unsigned long val, addr;
		running = sx_proc_str_get_ulong(running, &width);
		running = sx_proc_str_get_ulong(running, &addr);
		sx_proc_dbg_rd(width, addr, &val);
		printk(KERN_INFO "Read value addr: 0x%lx  val: 0x%lx \n",
			addr, val);
		cmd++;
	}

	p = strstr(cmd_str, "sw_reset");
	if (p != NULL) {
		int err = 0;

		if (!my_dev->pdev) {
			PROC_DUMP("Executing sw_reset is only possible "
					"when PCI is available\n");
			goto out;
		}

		PROC_DUMP("Executing sw_reset\n");
		err = sx_reset(my_dev);
		PROC_DUMP("Finished Executing sw_reset. err = %d\n", err);
		cmd++;
	}

	p = strstr(cmd_str, "enable_swid");
	if (p != NULL) {
		int err = 0;
		u32 swid;
		running = sx_proc_str_get_u32(running, &swid);
		spin_lock(&my_dev->profile_lock);
		if (!my_dev->profile_set || swid >= NUMBER_OF_SWIDS
				|| my_dev->profile.swid_type[swid] ==
				SX_KU_L2_TYPE_DONT_CARE ||
				sx_bitmap_test(&sx_priv(my_dev)->swid_bitmap,
						swid)) {
			PROC_DUMP("Err in param, not executing enable_swid\n");
			spin_unlock(&my_dev->profile_lock);
			goto out;
		}

		spin_unlock(&my_dev->profile_lock);
		PROC_DUMP("Executing enable_swid. dev_id = %u, "
				"swid num = %u\n",
				my_dev->device_id, swid);
		err = sx_enable_swid(my_dev, my_dev->device_id, swid, 0x1c0 + swid, 0);
		PROC_DUMP("Finished Executing enable_swid. err = %d\n", err);
		cmd++;
	}

	p = strstr(cmd_str, "disable_swid");
	if (p != NULL) {
		u32 swid;
		running = sx_proc_str_get_u32(running, &swid);
		spin_lock(&my_dev->profile_lock);
		if (!my_dev->profile_set || swid >= NUMBER_OF_SWIDS
				|| my_dev->profile.swid_type[swid] ==
				SX_KU_L2_TYPE_DONT_CARE ||
				!sx_bitmap_test(&sx_priv(my_dev)->swid_bitmap,
						swid)) {
			PROC_DUMP("Err in param not executing disable_swid\n");
			spin_unlock(&my_dev->profile_lock);
			goto out;
		}
		spin_unlock(&my_dev->profile_lock);
		PROC_DUMP("Executing disable_swid. swid num = %u\n", swid);
		sx_disable_swid(my_dev, swid);
		PROC_DUMP("Finished Executing disable_swid\n");
		cmd++;
	}

	p = strstr(cmd_str, "change_conf");
	if (p != NULL) {
		int err = 0;
		PROC_DUMP("Executing change_configuration. \n");
		err = sx_change_configuration(my_dev);
		PROC_DUMP("Finished Executing change_configuration. "
				"err = %d\n", err);
		cmd++;
	}	
	
	p = strstr(cmd_str, "pcidrv_restart");
    if (p != NULL) {
        int err = 0;
        PROC_DUMP("Executing restart pci driver. \n");        
        err = sx_restart_one_pci(my_dev->pdev);
        PROC_DUMP("Finished restart pci driver. "
                "err = %d\n", err);
        cmd++;
    }

	p = strstr(cmd_str, "set_pci_prof");
	if (p != NULL) {
		struct sx_pci_profile *profile = &my_dev->profile;
		int err = 0;
		memset(profile, 0, sizeof(*profile));
		profile->swid_type[0] = SX_KU_L2_TYPE_IB;
		profile->tx_prof[0][0].stclass = 0;
		profile->tx_prof[0][1].stclass = 1;
		profile->tx_prof[0][2].stclass = 2;
		profile->tx_prof[0][0].sdq = 0;
		profile->tx_prof[0][1].sdq = 1;
		profile->tx_prof[0][2].sdq = 2;
		profile->rdq_count[0] = 3;
		profile->rdq[0][0] = 0;
		profile->rdq[0][1] = 1;
		profile->rdq[0][2] = 2;
		profile->rdq_properties[0].number_of_entries = 128;
		profile->rdq_properties[0].entry_size = 2048;
		profile->rdq_properties[1].number_of_entries = 128;
		profile->rdq_properties[1].entry_size = 2048;
		profile->rdq_properties[2].number_of_entries = 128;
		profile->rdq_properties[2].entry_size = 2048;
		profile->cpu_egress_tclass[0] = 0;
		profile->cpu_egress_tclass[1] = 1;
		profile->cpu_egress_tclass[2] = 2;
		PROC_DUMP("Executing set profile. dev_id = %u \n",
				my_dev->device_id);
		err = sx_handle_set_profile(my_dev);
		PROC_DUMP("Finished Executing set profile. "
				"err = %d\n", err);
		if (!err)
			my_dev->profile_set = 1;
		cmd++;
	}

	p = strstr(cmd_str, "set_eth_prof");
	if (p != NULL) {
		struct sx_pci_profile profile;
		int err = 0, j;
		memset(&profile, 0, sizeof(profile));
		profile.swid_type[0] = SX_KU_L2_TYPE_ETH;

		for (j = 0; j <= 7; j++) {
			profile.tx_prof[0][j].stclass = j;
			profile.tx_prof[0][j].sdq = j;
		}

		profile.rdq_count[0] = 3;
		profile.rdq[0][0] = 0;
		profile.rdq[0][1] = 1;
		profile.rdq[0][2] = 2;

		profile.emad_tx_prof.stclass = 7;
		profile.emad_tx_prof.sdq = 7;
		profile.emad_rdq = 21;

		profile.rdq_properties[0].number_of_entries = 128;
		profile.rdq_properties[0].entry_size = 2048;
		profile.rdq_properties[0].rdq_weight = 10;
		profile.rdq_properties[1].number_of_entries = 128;
		profile.rdq_properties[1].entry_size = 2048;
		profile.rdq_properties[1].rdq_weight = 10;
		profile.rdq_properties[2].number_of_entries = 128;
		profile.rdq_properties[2].entry_size = 2048;
		profile.rdq_properties[2].rdq_weight = 10;
		profile.rdq_properties[21].number_of_entries = 128;
		profile.rdq_properties[21].entry_size = 2048;
		profile.rdq_properties[21].rdq_weight = 10;

		profile.cpu_egress_tclass[0] = 0;
		profile.cpu_egress_tclass[1] = 1;
		profile.cpu_egress_tclass[2] = 1;
		profile.cpu_egress_tclass[3] = 0;
		profile.cpu_egress_tclass[4] = 2;
		profile.cpu_egress_tclass[5] = 2;
		profile.cpu_egress_tclass[6] = 0;
		profile.cpu_egress_tclass[7] = 3;
		profile.cpu_egress_tclass[8] = 3;
		profile.cpu_egress_tclass[9] = 0;
		profile.cpu_egress_tclass[10] = 4;
		profile.cpu_egress_tclass[11] = 4;
		profile.cpu_egress_tclass[12] = 0;
		profile.cpu_egress_tclass[13] = 5;
		profile.cpu_egress_tclass[14] = 5;
		profile.cpu_egress_tclass[15] = 0;
		profile.cpu_egress_tclass[16] = 6;
		profile.cpu_egress_tclass[17] = 6;
		profile.cpu_egress_tclass[18] = 0;
		profile.cpu_egress_tclass[19] = 7;
		profile.cpu_egress_tclass[20] = 7;
		profile.cpu_egress_tclass[21] = 0;
		profile.cpu_egress_tclass[22] = 8;
		profile.cpu_egress_tclass[23] = 8;

		profile.pci_profile = PCI_PROFILE_EN_SINGLE_SWID;
		my_dev->profile.pci_profile = profile.pci_profile;

		memcpy(&my_dev->profile, &profile, sizeof(profile));
		PROC_DUMP("Executing set profile. dev_id = %u \n",
				my_dev->device_id);
		err = sx_handle_set_profile(my_dev);
		PROC_DUMP("Finished Executing set profile. "
				"err = %d\n", err);
		if (!err)
			my_dev->profile_set = 1;
		cmd++;
	}

	p = strstr(cmd_str, "dump_buf");
	if (p != NULL) {
		unsigned long val, addr, cnt;
		int i;
		running = sx_proc_str_get_ulong(running, &cnt);
		sx_proc_str_get_ulong(running, &addr);
		for (i = 0; i < cnt; i++) {
			if (i == 0 || (i%4 == 0))
				printk(KERN_INFO "\n0x%04x : ", i);

			sx_proc_dbg_rd(1, addr+i, &val);
			printk(KERN_INFO " 0x%02x", (u8)val);
		}
		printk("\n");
		cmd++;
	}

	p = strstr(cmd_str, "dump_swid");
	if (p != NULL) {
		PROC_DUMP("Dump active swids: \n");
		__sx_proc_dump_swids(my_dev);
		cmd++;
	}

	p = strstr(cmd_str, "dump_sdq");
	if (p != NULL) {
		PROC_DUMP("Dump active sdqs: \n");
		__sx_proc_dump_sdq(my_dev);
		cmd++;
	}

	p = strstr(cmd_str, "dump_rdq");
	if (p != NULL) {
		PROC_DUMP("Dump active rdqs: \n");
		__sx_proc_dump_rdq(my_dev);
		cmd++;
	}

	p = strstr(cmd_str, "dump_eq");
    if (p != NULL) {
        PROC_DUMP("Dump active eqs: \n");
        __sx_proc_dump_eq(my_dev);
        cmd++;
    }

	p = strstr(cmd_str, "show_rdq_post");
	if (p != NULL) {
		unsigned long rdq_idx;
		running = sx_proc_str_get_ulong(running, &rdq_idx);

		PROC_DUMP("Show rdqs data: \n");
		__sx_proc_dump_rdq_single(my_dev,rdq_idx);
		cmd++;
	}

	p = strstr(cmd_str, "dev_prof");
	if (p != NULL) {
		PROC_DUMP("Set dev profile: \n");
		__sx_proc_set_dev_profile(my_dev);
		cmd++;
	}

	p = strstr(cmd_str, "dump_synd");
	if (p != NULL) {
		PROC_DUMP("Dump syndromes: \n");
		sx_core_dump_synd_tbl(my_dev) ;
		cmd++;
	}

	p = strstr(cmd_str, "dump_stats");
	if (p != NULL) {
		PROC_DUMP("Dump statistics: \n");
		__dump_stats(my_dev);
		cmd++;
	}
	p = strstr(cmd_str, "trace_reg");
	if (p != NULL) {
		unsigned long reg_id;
		unsigned long max_cnt;
		running = sx_proc_str_get_ulong(running, &reg_id);
		running = sx_proc_str_get_ulong(running, &max_cnt);

		PROC_DUMP("trace_reg id:0x%lx max_cnt:0x%lx :\n",
				  reg_id, max_cnt);
		__sx_proc_dump_reg(my_dev, reg_id, max_cnt);
		cmd++;
	}

	p = strstr(cmd_str, "trace_pkt");
	if (p != NULL) {
		unsigned long emad_type;
		unsigned long max_cnt;
		unsigned long direction;
		running = sx_proc_str_get_ulong(running, &emad_type);
		running = sx_proc_str_get_ulong(running, &max_cnt);
		running = sx_proc_str_get_ulong(running, &direction);

		PROC_DUMP("trace_emad type:0x%lx max_cnt:0x%lx direction:%ld\n",
				  emad_type, max_cnt, direction);
		__sx_proc_dump_emad(my_dev, emad_type, max_cnt, direction);
		cmd++;
	}

	p = strstr(cmd_str, "dump_netdev");
	if (p != NULL) {
		PROC_DUMP("dump netdev info.\n");
		__debug_dump_netdev(my_dev);
		cmd++;
	}

	p = strstr(cmd_str, "dbg_trap");
	if (p != NULL) {
		unsigned long trap_id;
		unsigned long is_add;
		running = sx_proc_str_get_ulong(running, &trap_id);
		running = sx_proc_str_get_ulong(running, &is_add);

		PROC_DUMP("debug netdev trap: trap_id: %ld (0-all 5 traps), is_add; %ld \n",
								trap_id, is_add);
		__debug_netdev_traps(my_dev, trap_id, is_add);
		cmd++;
	}

	p = strstr(cmd_str, "oper_dev");
	if (p != NULL) {
		char	*dev_name;
		char	*oper_state;
		running = sx_proc_str_get_str(running, &dev_name);
		running = sx_proc_str_get_str(running, &oper_state);

		PROC_DUMP("debug set dev %s oper state: %s \n",
								dev_name, oper_state);
		__set_net_dev_oper_state(dev_name, oper_state);
		cmd++;
	}

	p = strstr(cmd_str, "dump_dev");
	if (p != NULL) {
		PROC_DUMP("Dump netdev :\n");
		__dump_net_dev_oper_state();
		cmd++;
	}

	p = strstr(cmd_str, "dump_kdb");
    if (p != NULL) {
        PROC_DUMP("Dump kernel DBs:\n");
        __dump_kdbs();
        cmd++;
    }



	p = strstr(cmd_str, "set_rdq_rl");
	if (p != NULL) {
		unsigned long rl_time_interval;
		unsigned long rdq;
		unsigned long max_credit;
		unsigned long use_limiter;
		unsigned long interval_credit;
		int cqn;
		running = sx_proc_str_get_ulong(running, &rl_time_interval);
		running = sx_proc_str_get_ulong(running, &rdq);
		running = sx_proc_str_get_ulong(running, &use_limiter);
		running = sx_proc_str_get_ulong(running, &max_credit);
		running = sx_proc_str_get_ulong(running, &interval_credit);

		PROC_DUMP("Set RDQ rate limiter: "
				"rl_time_interval = %lu, rdq = %lu, "
				"max_credit = %lu, use_limiter = %lu, "
				"interval_credit = %lu\n",
				rl_time_interval, rdq, max_credit,
				use_limiter, interval_credit);
		cqn = rdq + NUMBER_OF_SDQS;
		if (!sx_bitmap_test(&sx_priv(my_dev)->cq_table.bitmap, cqn)) {
			printk(KERN_WARNING PFX "Cannot set the rate limiter, RDQ %lu does not exist\n", rdq);
			cmd++;
			goto end;
		}

		if (use_limiter) {
			sx_priv(my_dev)->cq_table.rl_time_interval = max((int)rl_time_interval, 50);
			sx_priv(my_dev)->cq_table.cq_rl_params[cqn].interval_credit = interval_credit;
			sx_priv(my_dev)->cq_table.cq_rl_params[cqn].max_cq_credit = max_credit;
		}

		sx_priv(my_dev)->cq_table.cq_rl_params[cqn].use_limiter = use_limiter;
		if (!use_limiter) {
			sx_priv(my_dev)->cq_table.cq_rl_params[cqn].curr_cq_credit = 0;
			/* The CQ might not be armed if it ran out of credits */
			sx_cq_arm(sx_priv(my_dev)->cq_table.cq[cqn]);
		}


		if (!sx_priv(my_dev)->cq_table.cq_credit_thread && use_limiter) {
			char kth_name[20];

			sprintf(kth_name,"cq_credit_thread");
			sx_priv(my_dev)->cq_table.cq_credit_thread = kthread_create(sx_cq_credit_thread_handler,
					(void*)sx_priv(my_dev), kth_name);
			if (IS_ERR(sx_priv(my_dev)->cq_table.cq_credit_thread)) {
				printk(KERN_ERR PFX "Failed creating the CQ credit thread\n");
				return -ENOMEM;
			} else
				printk(KERN_INFO PFX "cq_credit_thread has been created\n");

			/* start the CQ credit task */
			wake_up_process(sx_priv(my_dev)->cq_table.cq_credit_thread);
		}
#if 0
		cqn = rdq + NUMBER_OF_SDQS;
		sx_priv(my_dev)->cq_table.rl_time_interval = max((int)rl_time_interval, 50);
		sx_priv(my_dev)->cq_table.cq_rl_params[cqn].use_limiter = use_limiter;
		sx_priv(my_dev)->cq_table.cq_rl_params[cqn].interval_credit = interval_credit;
		sx_priv(my_dev)->cq_table.cq_rl_params[cqn].max_cq_credit = max_credit;
		if (!use_limiter)
			sx_cq_arm(sx_priv(my_dev)->cq_table.cq[cqn]);
#endif
		cmd++;
	}

	p = strstr(cmd_str, "get_rdq_rl");
	if (p != NULL) {
		int i, max_idx = -1, cnt = 0, min_idx = -1;
		unsigned int sum = 0;
		unsigned int max_time_interval = 0;
		unsigned int min_time_interval = 1000;
		unsigned int limit = 1001;

		if (arr_count < 1001)
			limit = arr_count;

		printk(KERN_INFO PFX "credit_thread_vals:");
		for (i = 1; i < limit; i++) {
			unsigned int interval = credit_thread_vals[i] - credit_thread_vals[i-1];
			if (interval > 200)
				continue;

			if (i < 30)
				printk(KERN_INFO PFX "%u ", interval);

			sum += interval;
			cnt++;
			if (interval > max_time_interval) {
				max_time_interval = interval;
				max_idx = i;
			} else if (interval < min_time_interval) {
				min_time_interval = interval;
				min_idx = i;
			}
		}

		if (cnt)
			printk(KERN_INFO PFX "sum = %u, average = %u, max_time = "
					"%u, max_idx = %d, min_time = %u, min_idx = %d\n",
					sum, sum/cnt, max_time_interval, max_idx,
					min_time_interval, min_idx);
		else
			printk(KERN_INFO PFX "No statistics are available\n");

		cmd++;
	}

	p = strstr(cmd_str, "show_cq");
	if (p != NULL) {
	    unsigned long cq_idx;
	    running = sx_proc_str_get_ulong(running, &cq_idx);

	    PROC_DUMP("Show CQ data: \n");
	    __sx_proc_show_cq(my_dev,cq_idx);
	    cmd++;
	}

    p = strstr(cmd_str, "flush_rdq");
    if (p != NULL) {
        unsigned long idx;
        running = sx_proc_str_get_ulong(running, &idx);

        PROC_DUMP("Flush RDQ %ld: \n",idx);
        __sx_proc_flush_rdq(my_dev,idx);
        cmd++;
    }

    p = strstr(cmd_str, "backup_poll");
    if (p != NULL) {
        unsigned long enable;
        running = sx_proc_str_get_ulong(running, &enable);

        if (enable == 1) {
            PROC_DUMP("Enable CQ backup polling \n");
            atomic_set(&cq_backup_polling_enabled,1);
        } else if (enable == 0){
            PROC_DUMP("Disable CQ backup polling \n");
            atomic_set(&cq_backup_polling_enabled,0);
        } else {
            PROC_DUMP("CQ backup polling: %s\n",
                      atomic_read(&cq_backup_polling_enabled)?"enabled":"disabled");
        }
        cmd++;
    }

    p = strstr(cmd_str, "debug_cq_bckp_poll_cqn");
    if (p != NULL) {
        unsigned long param1;
        running = sx_proc_str_get_ulong(running, &param1);

        PROC_DUMP("debug_cq_backup_poll_cqn set to %ld \n",param1);
        debug_cq_backup_poll_cqn=param1;
		cmd++;
	}

end:
	if (cmd == 0) {
		PROC_DUMP("Available Commands:\n");
		PROC_DUMP("  add_port_netdev - Create a port netdevice");
		PROC_DUMP("  rem_port_netdev - Destroy a port netdevice");
		PROC_DUMP("  dump_dpt - dump DPT table\n");
		PROC_DUMP("  dpt_setp - set DPT table entry path\n");
		PROC_DUMP("  dpt_set_cr [dev_id] [path type] - set DPT path for cr_space\n");
		PROC_DUMP("  cr_space_write [dev_id] [address] [value] - write dword to the cr_space\n");
		PROC_DUMP("  cr_space_read [dev_id] [address] - read dword from the cr_space\n");
		PROC_DUMP("  q_fw - query fw\n");
		PROC_DUMP("  q_board_info - query board info\n");
		PROC_DUMP("  q_aq_cap - query async queues capabilities\n");
		PROC_DUMP("  get_profile - "
				"runs CONFIG_PROFILE with get modifier\n");
		PROC_DUMP("  access_reg [dev_id] [reg_id] [additional info]- runs "
				"ACCESS_REG (get) for the register with the "
				"given reg_id\n");
		PROC_DUMP("  get_pci -  walk pci devices\n");
		PROC_DUMP("  mem_wr [width 1 2 4] [address] [value] - "
				"write memory\n");
		PROC_DUMP("  mem_rd [width 1 2 4] [address] - read memory\n");
		PROC_DUMP("  sw_reset - run SW reset\n");
		PROC_DUMP("  enable_swid [swid_num] - enable the swid\n");
		PROC_DUMP("  disable_swid [swid_num] - disable the swid\n");
		PROC_DUMP("  change_conf [swid_num] - execute "
				"change_configuration\n");
		PROC_DUMP("  set_pci_prof - execute set_pci_profile\n");
		PROC_DUMP("  dump_buf [size] [address] - dumps a buffer\n");
		PROC_DUMP("  dump_swid - dumps active swids\n");
		PROC_DUMP("  dump_sdq - dumps opened sdqs\n");
		PROC_DUMP("  dump_rdq - dumps opened rdqs\n");
		PROC_DUMP("  dump_eq - dumps opened eqs\n");
		PROC_DUMP("  show_rdq_post <rdq> - dumps one rdq\n");
		PROC_DUMP("  dev_prof - set dev_profile\n");
		PROC_DUMP("  dump_synd - dump syndromes\n");
		PROC_DUMP("  dump_stats - dump statistics\n");
		PROC_DUMP("  trace_reg [reg_id] [max_cnt]\n");
		PROC_DUMP("  trace_pkt [type] [max_cnt] [direction rx:1, tx:2, both:3]\n");
		PROC_DUMP("  dump_netdev - dump netdev info\n");
		PROC_DUMP("  dbg_trap [trap_id] [is_add] - debug trap add/rem (trap_id = 0 add/rem all traps).\n");
		PROC_DUMP("  oper_dev - set oper state of netdev \n");
		PROC_DUMP("  dump_dev - set oper state of netdev \n");
		PROC_DUMP("  dump_kdb - dump kernel dbs \n");
		PROC_DUMP("  set_rdq_rl - set RDQ rate limiter <time interval> <rdq> <use_limiter> <max_credits> <interval credit>\n");
		PROC_DUMP("  get_rdq_rl - get RDQ rate limiter \n");
        PROC_DUMP("  show_cq <CQ> - dump CQ \n");
        PROC_DUMP("  flush_rdq <RDQ> - flush RDQ\n");
        PROC_DUMP("  backup_poll <disable(0)/enable(1)/query> - Enable CQ backup polling mechanism\n");
        PROC_DUMP("  debug_cq_bckp_poll_cqn <cqn> - trigger backup polling for CQN\n");

	}

out:
	return count;
}

static const struct file_operations sx_proc_fops = {
       .owner          = THIS_MODULE,
       .open           = sx_proc_open,
       .read           = seq_read,
       .llseek         = seq_lseek,
       .release        = single_release,
	.write          = sx_proc_write,
};

int sx_core_init_proc_fs(void)
{
	struct proc_dir_entry *dir_proc_entry;
	struct proc_dir_entry *proc_entry;
	char *proc_dir_name = DEF_SX_CORE_PROC_DIR;
	char *proc_file_name = DEF_SX_CORE_PROC_CORE_FILE;

	sx_proc_registered = 0;

	dir_proc_entry = proc_mkdir(proc_dir_name, NULL);
	if (dir_proc_entry == NULL) {
		printk(KERN_WARNING "create proc dir %s failed\n",
			   proc_dir_name);
		return -EINVAL;
	}

	proc_entry = proc_create(proc_file_name,
                        S_IFREG | S_IRUGO | S_IWUGO , dir_proc_entry,&sx_proc_fops);
	if (proc_entry == NULL) {
		printk(KERN_WARNING "create proc %s failed\n",
			   proc_file_name);
		return -EINVAL;
	}

	/* So we can figure out which chip */
	/* proc_entry->data = (void *) index; */
	sx_proc_registered = 1;

	printk(KERN_INFO "create proc %s successed\n", proc_file_name);

	return 0;
}

void sx_core_close_proc_fs(void)
{
	char *proc_dir_name = DEF_SX_CORE_PROC_DIR;
	char *proc_file_name = DEF_SX_CORE_PROC_CORE_FILE;
	char buf[100];

	if (sx_proc_registered) {
		(void) snprintf(buf, sizeof(buf), "%s/%s",
				proc_dir_name, proc_file_name);
		remove_proc_entry(buf, NULL);
		remove_proc_entry(proc_dir_name, NULL);
		sx_proc_registered = 0;
	}
}

void sx_proc_dbg_wr(int width, unsigned long  addr, unsigned long val)
{
	if (addr == 0)
		printk(KERN_WARNING "%s() unsupported addr NULL \n", __func__);

	switch (width) {
	case 1:
		*(u8 *)addr = (u8)val;
		break;
	case 2:
		*(u16 *)addr = (u16)val;
		break;
	case 4:
		*(u32 *)addr = (u32)val;
		break;
	case 8:
		*(u64 *)addr = val;
		break;
	default:
		printk(KERN_WARNING "%s() unsupported width %d \n",
			   __func__, width);
	}
}

void sx_proc_dbg_rd(int width, unsigned long addr, unsigned long *val)
{
	if (addr == 0)
		printk(KERN_WARNING "%s() unsupported addr NULL \n", __func__);

	switch (width) {
	case 1:
		*val = *(u8 *)addr;
		break;
	case 2:
		*val = *(u16 *)addr;
		break;
	case 4:
		*val = *(u32 *)addr;
		break;
	case 8:
		*val = *(u64 *)addr;
		break;
	default:
		printk(KERN_WARNING "%s() unsupported width %d \n",
			   __func__, width);
	}
}


void __sx_proc_dump_swids(struct sx_dev *dev)
{
	int i;

	for (i = 0; i < NUMBER_OF_SWIDS; i++)
		if (sx_bitmap_test(&sx_priv(dev)->swid_bitmap, i))
			printk(KERN_INFO "Swid %i is ACTIVE.\n", i);
		else
			printk(KERN_INFO "Swid %i is DISABLED.\n", i);
}

void __sx_proc_dump_sdq(struct sx_dev *dev)
{
	struct sx_priv *priv = sx_priv(dev);
	struct sx_dq_table *sdq_table = &priv->sdq_table;
	int i;

	if (sdq_table == NULL) {
	    printk(KERN_INFO "sdq table is empty \n");
	    return;
	}

	for (i = 0; i < dev->dev_cap.max_num_sdqs; i++) {
		if (sdq_table->dq[i]) {
			printk(KERN_INFO "[sdq %d]: dqn:%d, is_send:%d, "
					"head:%d, tail:%d,"
					" wqe_cnt:%d, wqe_shift:%d,is_flush:%d,"
					" state:%d, ref_cnt: %d\n cqn:%d, "
					"cq_con_idx:%d, cq_nent:%d, "
					"cq_ref_cnt: %d \n",
					i,
					sdq_table->dq[i]->dqn,
					sdq_table->dq[i]->is_send,
					sdq_table->dq[i]->head,
					sdq_table->dq[i]->tail,
					sdq_table->dq[i]->wqe_cnt,
					sdq_table->dq[i]->wqe_shift,
				   sdq_table->dq[i]->is_flushing,
					sdq_table->dq[i]->state,
					atomic_read(
						&sdq_table->dq[i]->refcount),

				    sdq_table->dq[i]->cq->cqn,
					sdq_table->dq[i]->cq->cons_index,
					sdq_table->dq[i]->cq->nent,
					atomic_read(
					&sdq_table->dq[i]->cq->refcount)
					);
		}
	}
}

void __sx_proc_dump_rdq(struct sx_dev *dev)
{
	struct sx_priv *priv = sx_priv(dev);
	struct sx_dq_table *rdq_table = &priv->rdq_table;
	struct sx_cq_table *cq_table = &priv->cq_table;
	int i;
	int cqn;

	if (rdq_table == NULL) {
        printk(KERN_INFO "rdq table is empty \n");
        return;
    }

	if (cq_table == NULL) {
        printk(KERN_INFO "cq table is empty \n");
        return;
    }

	for (i = 0; i < dev->dev_cap.max_num_rdqs; i++) {
		if (rdq_table->dq[i]) {
			cqn = i + NUMBER_OF_SDQS;
			printk(KERN_INFO "[rdq %d]: dqn:%d, is_send:%d, "
					"head:%d, tail:%d, wqe_cnt:%d, "
					"wqe_shift:%d,is_flush:%d, state:%d, "
					"ref_cnt: %d e_sz:%d \n cqn:%d, cq_con_idx:%d, "
					"cq_nent:%d, cq_ref_cnt: %d, use_limiter: %u,"
					"interval_credit: %d, max_cq_credit: %d, "
					"curr_cq_credit: %d, num_cq_stops = %d\n",
					i,
					rdq_table->dq[i]->dqn,
					rdq_table->dq[i]->is_send,
					rdq_table->dq[i]->head,
					rdq_table->dq[i]->tail,
					rdq_table->dq[i]->wqe_cnt,
					rdq_table->dq[i]->wqe_shift,
					rdq_table->dq[i]->is_flushing,
					rdq_table->dq[i]->state,
					atomic_read(&rdq_table->dq[i]->refcount),
					rdq_table->dq[i]->dev->profile.rdq_properties[rdq_table->dq[i]->dqn].entry_size,
					rdq_table->dq[i]->cq->cqn,
					rdq_table->dq[i]->cq->cons_index,
					rdq_table->dq[i]->cq->nent,
					atomic_read(
					&rdq_table->dq[i]->cq->refcount),
					cq_table->cq_rl_params[cqn].use_limiter,
					cq_table->cq_rl_params[cqn].interval_credit,
					cq_table->cq_rl_params[cqn].max_cq_credit,
					cq_table->cq_rl_params[cqn].curr_cq_credit,
					cq_table->cq_rl_params[cqn].num_cq_stops);
		}
	}
}

void __sx_proc_dump_eq(struct sx_dev *dev)
{
    struct sx_priv *priv = sx_priv(dev);
    struct sx_eq_table *eq_table = &priv->eq_table;
    int i;

    if (eq_table == NULL) {
        printk(KERN_INFO "eq_table is empty \n");
        return;
    }

    for (i = 0; i < SX_NUM_EQ; i++) {
        printk("[eq %d]: eqn:%d, nent:%d, cons_index:%d \n",
             i, eq_table->eq[i].eqn, eq_table->eq[i].nent,
             eq_table->eq[i].cons_index );
    }
}

void __sx_proc_dump_rdq_single(struct sx_dev *dev, int rdq)
{
	struct sx_priv *priv = sx_priv(dev);
	struct sx_dq_table *rdq_table = &priv->rdq_table;
	int i = rdq, j,k;

	/* for (i = 0; i < dev->dev_cap.max_num_rdqs; i++) */
	printk("Dump rdq index %d \n", rdq);

	if (rdq_table == NULL) {
        printk(KERN_INFO "rdq table is empty \n");
        return;
    }

	if (rdq_table->dq[i]) {
		printk("[rdq %d]: dqn:%d, is_send:%d, "
				"head:%d, tail:%d, wqe_cnt:%d, "
				"wqe_shift:%d,is_flush:%d, state:%d, "
				"ref_cnt: %d e_sz:%d \n cqn:%d, cq_con_idx:%d, "
				"cq_nent:%d, cq_ref_cnt: %d \n",
				i,
				rdq_table->dq[i]->dqn,
				rdq_table->dq[i]->is_send,
				rdq_table->dq[i]->head,
				rdq_table->dq[i]->tail,
				rdq_table->dq[i]->wqe_cnt,
				rdq_table->dq[i]->wqe_shift,
				rdq_table->dq[i]->is_flushing,
				rdq_table->dq[i]->state,
				atomic_read(
					&rdq_table->dq[i]->refcount),
				rdq_table->dq[i]->dev->profile.rdq_properties[rdq_table->dq[i]->dqn].entry_size,

				rdq_table->dq[i]->cq->cqn,
				rdq_table->dq[i]->cq->cons_index,
				rdq_table->dq[i]->cq->nent,
				atomic_read(
				&rdq_table->dq[i]->cq->refcount)
				);
	}
	else{
		printk("rdq %d doesn't exist \n", i);
	}

#define DUMP_BYTES_NUM		16

	for (j=0; j<rdq_table->dq[i]->wqe_cnt; j++) {
		char* buf = rdq_table->dq[i]->sge[j].skb->data;

		printk("==================\n");
		printk("rdq %d , wqe %d , buf: %p, first %d bytes :\n",
										i,j,buf, DUMP_BYTES_NUM);
		for (k = 0; k < DUMP_BYTES_NUM; k++) {
			if (k == 0 || (k%4 == 0))
				printk("\n0x%04x : ", k);
			printk(" 0x%02x", buf[k]);
		}
		printk("\n");
	}
}

/* Current values are for IB single swid with AR enabled */
void __sx_proc_set_dev_profile(struct sx_dev *dev)
{
	struct ku_profile single_part_eth_device_profile = {
	.dev_id		   = 255,
	.set_mask_0_63     = 0xf3ff,
	.set_mask_64_127   = 0,
	.max_vepa_channels = 0,
	.max_lag           = 0,
	.max_port_per_lag  = 0,
	.max_mid           = 0,
	.max_pgt           = 0,
	.max_system_port   = 0,
	.max_active_vlans  = 0,
	.max_regions       = 0,
	.max_flood_tables  = 0,
	.max_per_vid_flood_tables = 0,
	.flood_mode        = 0,
	.max_fid_offset_flood_tables = 0,
	.fid_offset_table_size = 0,
	.max_per_fid_flood_table = 0,
	.per_fid_table_size = 0,
	.max_ib_mc         = 27,
	.max_pkey          = 126,
	.ar_sec		   = 1,
	.adaptive_routing_group_cap = 2048,
	.arn		   = 0,
	.kvd_linear_size = 0,
    .kvd_hash_single_size = 0,
    .kvd_hash_double_size = 0,
	.swid0_config_type = {
			.mask = 1,
			.type = KU_SWID_TYPE_INFINIBAND
	},
	.swid1_config_type = {
			.mask = 1,
			.type = KU_SWID_TYPE_DISABLED
	},
	.swid2_config_type = {
			.mask = 1,
			.type = KU_SWID_TYPE_DISABLED
	},
	.swid3_config_type = {
			.mask = 1,
			.type = KU_SWID_TYPE_DISABLED
	},
	.swid4_config_type = {
			.mask = 1,
			.type = KU_SWID_TYPE_DISABLED
	},
	.swid5_config_type = {
			.mask = 1,
			.type = KU_SWID_TYPE_DISABLED
	},
	.swid6_config_type = {
			.mask = 1,
			.type = KU_SWID_TYPE_DISABLED
	},
	.swid7_config_type = {
			.mask = 1,
			.type = KU_SWID_TYPE_DISABLED
	}
	};

	sx_SET_PROFILE(dev, &single_part_eth_device_profile);
}


void __dump_stats(struct sx_dev* sx_dev)
{
	int swid, pkt_type, synd;

	for (swid=0; swid<NUMBER_OF_SWIDS+1; swid++) {
			printk("=========================\n");

			if (NUMBER_OF_SWIDS == swid) {
				printk("Packets stats for swid 254 \n");
			}
			else{
				printk("Packets stats for swid %d \n",swid);
			}

			for (pkt_type=0; pkt_type<PKT_TYPE_NUM; pkt_type++) {
				if (0 != sx_dev->stats.rx_by_pkt_type[swid][pkt_type]) {
					printk("rx pkt of type [%s (%d)]: %llu \n",
								sx_cqe_packet_type_str[pkt_type], pkt_type,
								sx_dev->stats.rx_by_pkt_type[swid][pkt_type]);
				}

				if (0 != sx_dev->stats.tx_by_pkt_type[swid][pkt_type]) {
					printk("tx pkt of type [%s (%d)]: %llu \n",
						   ku_pkt_type_str[pkt_type], pkt_type,
						   sx_dev->stats.tx_by_pkt_type[swid][pkt_type]);
				}
			} /* for (pkt_type=0; pkt_type<PKT_TYPE_NUM; pkt_type++) { */

			for (synd=0; synd<NUM_HW_SYNDROMES; synd++) {
				if (0 != sx_dev->stats.rx_by_synd[swid][synd]) {
					printk("rx pkt on synd [%d]: %llu \n", synd, sx_dev->stats.rx_by_synd[swid][synd]);
				}
			} /* for (synd=0; synd<PKT_TYPE_NUM; synd++) { */

	}

	printk("=========================\n");
	for (synd = 0; synd < NUM_HW_SYNDROMES; synd++) {
		for (pkt_type=0; pkt_type<PKT_TYPE_NUM; pkt_type++) {
			if (sx_dev->stats.rx_unconsumed_by_synd[synd][pkt_type] != 0) {
				printk("rx unconsumed on synd [%d] type [%s]: %llu \n",
						synd, sx_cqe_packet_type_str[pkt_type],
						sx_dev->stats.rx_unconsumed_by_synd[synd][pkt_type]);
			}
		}
	}
	//rx_eventlist_drops_by_synd
	printk("=========================\n");
	for (synd = 0; synd < NUM_HW_SYNDROMES; synd++) {
		if (sx_dev->stats.rx_eventlist_drops_by_synd[synd] != 0) {
			printk("rx eventlist drops on synd [%d]: %llu \n",
					synd, sx_dev->stats.rx_eventlist_drops_by_synd[synd]);
		}
	}

	printk("=========================\n");
	printk("eventlist_drops_counter: %llu \n"
				"unconsumed_packets_counter: %llu \n"
				"filtered_lag_packets_counter: %llu \n"
				"filtered_port_packets_counter: %llu \n"
				"loopback_packets_counter: %llu \n",
					sx_dev->eventlist_drops_counter,
					sx_dev->unconsumed_packets_counter,
					sx_dev->filtered_lag_packets_counter,
					sx_dev->filtered_port_packets_counter,
					sx_dev->loopback_packets_counter);
}

void __sx_proc_dump_reg(struct sx_dev *my_dev, unsigned long reg_id, unsigned long max_cnt)
{
	if (0 != max_cnt) {
		i2c_cmd_dump = SX_DBG_ENABLE;
		i2c_cmd_op = 0x40;
		i2c_cmd_reg_id = reg_id;
		i2c_cmd_dump_cnt = max_cnt;
	}
	else{
		i2c_cmd_dump = SX_DBG_DISABLE;
		i2c_cmd_op = SX_DBG_CMD_OP_TYPE_ANY;
		i2c_cmd_reg_id = SX_DBG_REG_TYPE_ANY;
		i2c_cmd_dump_cnt = SX_DBG_COUNT_UNLIMITED;
	}
}

void __sx_proc_dump_emad(struct sx_dev *my_dev, unsigned long emad_type, unsigned long max_cnt, unsigned long direction)
{
	if (SX_PROC_DUMP_PKT_DISABLE == direction ||
		0 == max_cnt){
		rx_debug = SX_DBG_DISABLE;
		rx_debug_pkt_type = SX_DBG_PACKET_TYPE_ANY;
		rx_debug_emad_type = SX_DBG_EMAD_TYPE_ANY;
		rx_dump = SX_DBG_DISABLE;
		rx_dump_cnt = SX_DBG_COUNT_UNLIMITED;

		tx_debug = SX_DBG_DISABLE;
		tx_debug_pkt_type = SX_DBG_PACKET_TYPE_ANY;
		tx_debug_emad_type = SX_DBG_EMAD_TYPE_ANY;
		tx_dump = SX_DBG_DISABLE;
		tx_dump_cnt = SX_DBG_COUNT_UNLIMITED;
		return;
	}

	if (direction & SX_PROC_DUMP_PKT_RX) {
		rx_debug = SX_DBG_ENABLE;
		rx_debug_pkt_type = 5; /* emad trap */
		rx_debug_emad_type = emad_type;
		rx_dump = SX_DBG_ENABLE;
		rx_dump_cnt = max_cnt;
	}

	if (direction & SX_PROC_DUMP_PKT_TX) {
		tx_debug = SX_DBG_ENABLE;
		tx_debug_pkt_type = SX_PKT_TYPE_EMAD_CTL;
		tx_debug_emad_type = emad_type;
		tx_dump = SX_DBG_ENABLE;
		tx_dump_cnt = max_cnt;
	}
}


void __debug_dump_netdev(struct sx_dev *dev)
{
	union sx_event_data event_data;
	sx_core_dispatch_event(dev, SX_DEV_EVENT_DEBUG_NETDEV, &event_data);
}

void __debug_netdev_traps(struct sx_dev *dev, unsigned long trap_id, unsigned long is_add)
{
	int cmd;
	union sx_event_data event_data;

	if (is_add == 1) {
		cmd = SX_DEV_EVENT_ADD_SYND_NETDEV;
	}
	else{
		cmd = SX_DEV_EVENT_REMOVE_SYND_NETDEV;
	}

	event_data.eth_l3_synd.swid = 0;
	event_data.eth_l3_synd.hw_synd = trap_id;

	sx_core_dispatch_event(dev, cmd, &event_data);
}


void __set_net_dev_oper_state(char	*dev_name ,	char *oper_state)
{
	struct net_device *dev;

	dev = first_net_device(&init_net);
	while (dev) {
		/* printk(KERN_DEBUG "found [%s]\n", dev->name); */

		if (dev_name != NULL &&
			(!strcmp(dev->name, dev_name)) ){
			printk(KERN_INFO "Device [%s] set oper state to %s \n",
				   dev->name, oper_state);

			if (strstr(oper_state, "up")) {
				netif_dormant_off(dev);
				netif_carrier_on(dev);
				netif_start_queue(dev);
			}
			else if (strstr(oper_state, "down")) {
				netif_dormant_on(dev);
				netif_carrier_off(dev);
				netif_stop_queue(dev);
			}
			else{
				printk("Unsupported oper_state: %s \n", oper_state);
			}
		}

		dev = next_net_device(dev);
	}
}

void __dump_net_dev_oper_state(void)
{
	struct net_device *dev;

	dev = first_net_device(&init_net);
	while (dev) {
		printk(KERN_DEBUG "Found dev [%s]: dormant:%d, carrier:%d  \n",
				   dev->name, netif_dormant(dev), netif_carrier_ok(dev));

		dev = next_net_device(dev);
	}
}

void __dump_kdbs(void)
{
    struct sx_dev *my_dev = sx_glb.
                sx_dpt.dpt_info[DEFAULT_DEVICE_ID].
                sx_pcie_info.sx_dev;
    struct sx_priv *priv = sx_priv(my_dev);
    int i,j,is_valid = 0, is_printed=0;

    /* IB only */
    printk("============================\n");
    printk("IB to LOCAL :\n");
    for (i=0; i<MAX_IBPORT_NUM + 1; i++){
        if (priv->ib_to_local_db[i] != 0){
            printk("[ib%d : local %d] \n",
                   i, priv->ib_to_local_db[i]);
        }
    }

    /* ETH only */
    printk("============================\n");
    printk("ETH SYS to LOCAL :\n");
    for (i=0; i<MAX_SYSPORT_NUM; i++){
        if (priv->system_to_local_db[i] != 0){
            printk("[system 0x%x : local %d : pvid %d] \n",
                   i, priv->system_to_local_db[i], priv->pvid_sysport_db[i]);
        }
    }

    printk("============================\n");
    printk("ETH LAG MEMBER to LOCAL :\n");
    for (i=0; i<MAX_LAG_NUM; i++){  /* lag */
        for (j=0; j<MAX_LAG_MEMBERS_NUM; j++){  /* lag member id */
            if (priv->lag_member_to_local_db[i][j] != 0){
                if (is_printed == 0){
                    printk("LAG 0x%x: ", i);
                    is_printed = 1;
                }

                printk("[%d:local %d], ",
                       j, priv->lag_member_to_local_db[i][j]);
            }
        }

        if (is_printed == 1)
            printk("\n");

        is_printed = 0;
    }

    printk("============================\n");
    printk("ETH SYSTEM_PORT_RP :\n");
    for (i=0; i<MAX_PHYPORT_NUM; i++){  /* system port */
        if (priv->local_is_rp[i] != 0){
            printk("(local %d), ",i);
        }
    }
    printk("\n");

    printk("============================\n");
    printk("ETH LAG_RP :\n");
    for (i=0; i<MAX_LAG_NUM; i++){  /* system port */
        if (priv->lag_is_rp[i] != 0){
            printk("%d, ",i);
        }
    }
    printk("\n");

    /* common */
    printk("============================\n");
    printk("COMMON LOCAL TO SWID :\n");
    printk("LOCAL\t| SWID\t|\n");
    printk("------------------------------------------\n");
    for (i=0; i<(MAX_PHYPORT_NUM+1); i++){  /* system port */
        printk("%02d \t| %d\t|  \n",
               i, priv->local_to_swid_db[i]);
    }

#if 0
    printk("============================\n");
    printk("COMMON SYSPORT TO PVID:\n");
    for (i=0; i<(MAX_SYSPORT_NUM); i++){  /* system port */
        printk("sysport 0x%x pvid %d\n",
               i, priv->pvid_sysport_db[i]);
    }

    printk("============================\n");
    printk("COMMON LAG TO PVID:\n");
    for (i=0; i<(MAX_LAG_NUM); i++){  /* system port */
        printk("LAG %d pvid %d\n",
               i, priv->pvid_lag_db[i]);
    }
#endif

#if 0
    printk("============================\n");
    printk("COMMON TRUNCATE_SZ:\n");
    for (i=0; i<(NUMBER_OF_RDQS); i++){  /* system port */
        printk("RDQ%d  tr_sz:%d\n",
               i, priv->truncate_size_db[i]);
    }

    printk("============================\n");
    printk("SYSPORT FILTER DB:\n");
    for (i=0; i<(NUM_HW_SYNDROMES); i++){  /* system port */
        for (j=0; j<(MAX_SYSTEM_PORTS_IN_FILTER); j++){  /* system port */
            if (priv->sysport_filter_db[i][j] != 0){
                printk("HW_SYND 0x%x SYSPORT 0x%x filter: %d \n",
                               i, j, priv->sysport_filter_db[i][j]);
            }
        }
    }

    printk("============================\n");
    printk("LAG FILTER DB:\n");
    for (i=0; i<(NUM_HW_SYNDROMES); i++){  /* system port */
        for (j=0; j<(MAX_LAG_PORTS_IN_FILTER); j++){  /* system port */
            if (priv->lag_filter_db[i][j] != 0){
                printk("HW_SYND 0x%x LAG_ID %d filter: %d \n",
                       i, j, priv->lag_filter_db[i][j]);
            }
        }
    }
#endif
    printk("============================\n");
    printk("port_prio2tc:\n");
    for (i=0; i<(MAX_PHYPORT_NUM + 1); i++){  /* system port */
        for (j=0; j<(MAX_PRIO_NUM + 1); j++){  /* system port */
           is_valid += priv->port_prio2tc[i][j];
        }

        if (is_valid){
            printk("sysport 0x%x : ", i);
            for (j=0; j<(MAX_PRIO_NUM + 1); j++){  /* system port */
                 printk("p%d:tc%d, ",
                      j, priv->port_prio2tc[i][j]);
            }
            printk("\n");
        }
        is_valid = 0;
    }

    printk("============================\n");
    printk("lag_prio2tc:\n");
    for (i=0; i<(MAX_LAG_NUM + 1); i++){  /* system port */
        for (j=0; j<(MAX_PRIO_NUM + 1); j++){  /* system port */
           is_valid += priv->lag_prio2tc[i][j];
        }

        if (is_valid){
            printk("lag_id %d : ", i);
            for (j=0; j<(MAX_PRIO_NUM + 1); j++){  /* system port */
                 printk("p%d:tc%d, ",
                      j, priv->port_prio2tc[i][j]);
            }
            printk("\n");
        }
        is_valid = 0;
    }

    printk("============================\n");
    printk("port_vtag_mode:\n");
    for (i=0; i<(MAX_PHYPORT_NUM + 1); i++){  /* system port */
        for (j=0; j<(MAX_VLAN_NUM); j++){  /* system port */
           is_valid += priv->port_vtag_mode[i][j];
        }

        if (is_valid){
            printk("sys_port 0x%x tagged on vlans: ", i);
            for (j=0; j<(MAX_VLAN_NUM); j++){  /* system port */
                if (priv->port_vtag_mode[i][j])
                    printk("%d, ",j);
            }
            printk("\n");
        }
        is_valid = 0;
    }

    printk("============================\n");
    printk("lag_vtag_mode:\n");
    for (i=0; i<(MAX_LAG_NUM + 1); i++){  /* system port */
        for (j=0; j<(MAX_VLAN_NUM); j++){  /* system port */
           is_valid += priv->lag_vtag_mode[i][j];
        }

        if (is_valid){
            printk("lag 0x%x tagged on vlans: ", i);
            for (j=0; j<(MAX_VLAN_NUM); j++){  /* system port */
                if (priv->lag_vtag_mode[i][j])
                    printk("%d, ",j);
            }
            printk("\n");
        }
        is_valid = 0;
    }

    printk("============================\n");
    printk("prio_tag_sysports:\n");
    for (i=0; i<(MAX_PHYPORT_NUM + 1); i++){  /* system port */
        if (priv->port_prio_tagging_mode[i])
            printk("0x%x, ", i);
    }
    printk("\n");


    printk("============================\n");
    printk("prio_tag_lags:\n");
    for (i=0; i<(MAX_LAG_NUM + 1); i++){  /* system port */
        if (priv->lag_prio_tagging_mode[i])
            printk("lag%d, ", i);
    }
    printk("\n");

    printk("============================\n");
    printk("vid2ip:\n");
    for (i=0; i<(MAX_VLAN_NUM - 1); i++){  /* vlan */
        if (priv->icmp_vlan2ip_db[i])
            printk("vlan%d: 0x%x, ", i, priv->icmp_vlan2ip_db[i]);
    }
    printk("\n");

    printk("============================\n");
    printk("port_rp_rif:\n");
    printk("LOCAL\t| VLAN\t| RIF\t|\n");
    printk("-------------------------\n");
    for (i=0; i<(MAX_PHYPORT_NUM + 1); i++) {
        for (j=0; j<MAX_VLAN_NUM; j++) {
            if (priv->port_rp_rif_valid[i][j]) {
                printk("%u\t| %u\t| %u\t|\n", i, j, priv->port_rp_rif[i][j]);
            }
        }
    }

    printk("============================\n");
    printk("lag_rp_rif:\n");
    printk("LID\t| VLAN\t| RIF\t|\n");
    printk("-------------------------\n");
    for (i=0; i<MAX_LAG_NUM; i++) {
        for (j=0; j<MAX_VLAN_NUM; j++) {
            if (priv->lag_rp_rif_valid[i][j]) {
                printk("%u\t| %u\t| %u\t|\n", i, j, priv->lag_rp_rif[i][j]);
            }
        }
    }
    
    printk("============================\n");
    printk("port_vid_to_fid:\n");
    printk("PORT\t| VLAN\t| FID\t|\n");
    printk("-------------------------\n");
    for (i=0; i<MAX_PHYPORT_NUM; i++) {
        for (j=0; j<MAX_VLAN_NUM; j++) {
            if (priv->port_vid_to_fid[i][j]) {
                printk("%u\t| %u\t| %u\t|\n", i, j, priv->port_vid_to_fid[i][j]);
            }
        }
    }    
}


void __sx_proc_show_cq(struct sx_dev *dev, int cqn)
{
    sx_cq_show_cq(dev,cqn);
}

void __sx_proc_flush_rdq(struct sx_dev *my_dev, int idx)
{
    sx_cq_flush_rdq(my_dev,idx);
}
