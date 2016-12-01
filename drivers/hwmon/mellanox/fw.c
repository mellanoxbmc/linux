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

#include "fw.h"
#include "sx.h"
#include <linux/mlx_sx/cmd.h>
#include <linux/mlx_sx/driver.h>

extern struct sx_globals sx_glb;
#define MCIA_REG_ID 0x9014

enum {
	MCIA_INVALID_PORT 		= 0x17,
	MCIA_PORT_NOT_SUPP 		= 0x27,
	MCIA_NOT_CONNECTED 		= 0x37,
	MCIA_NO_EEPROM 			= 0x47,
	MCIA_INVALID_PAGE 		= 0x57,
	MCIA_INVALID_DEVICE_ADDR 	= 0x67,
	MCIA_INVALID_I2C_DEV_ADDR	= 0x77,
	MCIA_CABLE_NOT_SUPP 		= 0x87,
	MCIA_I2C_ERROR 			= 0x97,
};

static const char *mcia_err_str(u8 status)
{
	switch (status) {
	case MCIA_INVALID_PORT:
		return "Invalid port";
	case MCIA_PORT_NOT_SUPP:
		return "Port not supported";
	case MCIA_NOT_CONNECTED:
		return "Not connected";
	case MCIA_NO_EEPROM:
		return "No EEPROM";
	case MCIA_INVALID_PAGE:
		return "Invalid page";
	case MCIA_INVALID_DEVICE_ADDR:
		return "Invalid device address";
	case MCIA_INVALID_I2C_DEV_ADDR:
		return "Invalid I2C device address";
	case MCIA_CABLE_NOT_SUPP:
		return "Cable not supported";
	case MCIA_I2C_ERROR:
		return "I2C";
	default:
		return "Unknown";
	}
}

#define SX_GET(dest, source, offset)					\
	do {								\
		void *__p = (char *) (source) + (offset);		\
		switch (sizeof(dest)) {				\
		case 1:							\
			(dest) = *(u8 *) __p;				\
			break;	      					\
		case 2: 						\
			(dest) = be16_to_cpup(__p);			\
			break;	      					\
		case 4: 						\
			(dest) = be32_to_cpup(__p); 			\
			break;	      					\
		case 8: 						\
			(dest) = be64_to_cpup(__p); 			\
			break;	      					\
		default: 						\
			break;						\
		}							\
	} while (0)

#define SX_PUT(dest, source, offset)					\
	do {								\
		void *__d = ((char *) (dest) + (offset));		\
		switch (sizeof(source)) {				\
		case 1:							\
			*(u8 *) __d = (source);				\
			break; 						\
		case 2:							\
			*(__be16 *) __d = cpu_to_be16(source);		\
			break; 						\
		case 4:							\
			*(__be32 *) __d = cpu_to_be32(source);		\
			break; 						\
		case 8:							\
			*(__be64 *) __d = cpu_to_be64(source);		\
			break; 						\
		default: 						\
			break;						\
		}							\
	} while (0)

#ifdef SX_DEBUG
void dump_mailbox(void *mbox, int size, char *title)
{
	u8 *buf = mbox;
	int i;

	printk(KERN_DEBUG "%s: MBOX contents (%p)\n", title, mbox);
	for (i = 0; i < size ; i += 32)
		printk(KERN_DEBUG "%02x%02x%02x%02x\n%02x%02x%02x%02x\n"
		       "%02x%02x%02x%02x\n%02x%02x%02x%02x\n"
		       "%02x%02x%02x%02x\n%02x%02x%02x%02x\n"
		       "%02x%02x%02x%02x\n%02x%02x%02x%02x\n",
		       buf[i+0], buf[i+1], buf[i+2], buf[i+3],
		       buf[i+4], buf[i+5], buf[i+6], buf[i+7],
		       buf[i+8], buf[i+9], buf[i+10], buf[i+11],
		       buf[i+12], buf[i+13], buf[i+14], buf[i+15],
		       buf[i+16], buf[i+17], buf[i+18], buf[i+19],
		       buf[i+20], buf[i+21], buf[i+22], buf[i+23],
		       buf[i+24], buf[i+25], buf[i+26], buf[i+27],
		       buf[i+28], buf[i+29], buf[i+30], buf[i+31]);
}
#endif

static void get_board_id(void *vsd, char *board_id)
{
#define VSD_OFFSET_SX_BOARD_ID	0xd0
#define SX_PSID_SIZE		16

	memset(board_id, 0, SX_BOARD_ID_LEN);
	memcpy(board_id, vsd + VSD_OFFSET_SX_BOARD_ID, SX_PSID_SIZE);
}

#define QUERY_FW_IN_MB_SIZE	0x4c
int sx_QUERY_FW(struct sx_dev *dev, struct ku_query_fw* query_fw)
{
	struct sx_fw  fw;
	struct sx_cmd *cmd = &sx_priv(dev)->cmd;
	struct sx_cmd_mailbox *mailbox;
	u32 *outbox;
	int err = 0;
	int	 target_dev_id;

	fw.local_in_mb_size = 0;
	fw.local_out_mb_offset = 0;
	fw.fw_icm = 0;
	fw.local_in_mb_offset = 0;
	fw.local_out_mb_size = 0;

	if (NULL == query_fw) {
		target_dev_id = DEFAULT_DEVICE_ID;
	} else {
		target_dev_id = query_fw->dev_id;
	}

	if (DEFAULT_DEVICE_ID < target_dev_id) {
		printk(KERN_NOTICE "dev_id %d exceeded range : 1 - %d",
					target_dev_id, target_dev_id);
		return -EINVAL;
	}

#define QUERY_FW_VER_OFFSET		0x00
#define QUERY_FW_CORE_CLOCK_OFFSET	0x08
#define QUERY_FW_DEBUG_TRACE_OFFSET	0x0c
#define QUERY_FW_FW_HOUR_OFFSET		0x10
#define QUERY_FW_FW_MINUTES_OFFSET	0x11
#define QUERY_FW_FW_SECONDS_OFFSET	0x12
#define QUERY_FW_FW_YEAR_OFFSET		0x14
#define QUERY_FW_FW_MONTH_OFFSET	0x16
#define QUERY_FW_FW_DAY_OFFSET		0x17
#define QUERY_FW_ERR_START_OFFSET	0x30
#define QUERY_FW_ERR_SIZE_OFFSET	0x38
#define QUERY_FW_ERR_BAR_OFFSET		0x3c
#define QUERY_FW_CLR_INT_BASE_OFFSET	0x20
#define QUERY_FW_CLR_INT_BAR_OFFSET	0x28
#define QUERY_FW_DB_PAGE_OFFSET_OFFSET	0x40
#define QUERY_FW_DB_PAGE_BAR_OFFSET	0x48
#define QUERY_FW_SIZE_OFFSET           	0x00

	mailbox = sx_alloc_cmd_mailbox(dev, target_dev_id);
	if (IS_ERR(mailbox))
		return PTR_ERR(mailbox);
	outbox = mailbox->buf;
	err = sx_cmd_box(dev, target_dev_id, 0, mailbox, 0, 0,
			SX_CMD_QUERY_FW, SX_CMD_TIME_CLASS_A,
			QUERY_FW_IN_MB_SIZE);
	if (err)
		goto out;

	SX_GET(fw.fw_pages, outbox, QUERY_FW_SIZE_OFFSET);
	SX_GET(fw.fw_ver,   outbox, QUERY_FW_VER_OFFSET);
	/*
	 * FW subminor version is at more significant bits than minor
	 * version, so swap here.
	 */
	fw.fw_ver = (fw.fw_ver & 0xffff00000000ull) |
		((fw.fw_ver & 0xffff0000ull) >> 16) |
		((fw.fw_ver & 0x0000ffffull) << 16);
	dev->fw_ver = fw.fw_ver;
	SX_GET(fw.core_clock,  outbox, QUERY_FW_CORE_CLOCK_OFFSET);
	SX_GET(fw.debug_trace, outbox, QUERY_FW_DEBUG_TRACE_OFFSET);
	fw.debug_trace = fw.debug_trace >> 7;
	cmd->max_cmds = 1;
	sx_dbg(dev, "FW version %012llx, max commands %d\n",
		  (unsigned long long) fw.fw_ver, cmd->max_cmds);
	SX_GET(fw.catas_offset, outbox, QUERY_FW_ERR_START_OFFSET);
	SX_GET(fw.catas_size,   outbox, QUERY_FW_ERR_SIZE_OFFSET);
	SX_GET(fw.catas_bar,    outbox, QUERY_FW_ERR_BAR_OFFSET);
	fw.catas_bar = (fw.catas_bar >> 6) * 2;
	sx_dbg(dev, "Error buffer offset at 0x%llx, size 0x%x in BAR %u\n",
		 (unsigned long long) fw.catas_offset, fw.catas_size,
		 fw.catas_bar);
	SX_GET(fw.clr_int_base, outbox, QUERY_FW_CLR_INT_BASE_OFFSET);
	SX_GET(fw.clr_int_bar,  outbox, QUERY_FW_CLR_INT_BAR_OFFSET);
	fw.clr_int_bar = (fw.clr_int_bar >> 6) * 2;
	sx_dbg(dev, "FW size %d KB\n", fw.fw_pages << 2);
	sx_dbg(dev, "Clear int base at 0x%llx, in BAR %u\n",
		 (unsigned long long) fw.clr_int_base, fw.clr_int_bar);
	SX_GET(fw.doorbell_page_offset, outbox,
			QUERY_FW_DB_PAGE_OFFSET_OFFSET);
	SX_GET(fw.doorbell_page_bar,    outbox, QUERY_FW_DB_PAGE_BAR_OFFSET);
	SX_GET(fw.fw_hour,    outbox, QUERY_FW_FW_HOUR_OFFSET);
	SX_GET(fw.fw_minutes, outbox, QUERY_FW_FW_MINUTES_OFFSET);
	SX_GET(fw.fw_seconds, outbox, QUERY_FW_FW_SECONDS_OFFSET);
	SX_GET(fw.fw_year,    outbox, QUERY_FW_FW_YEAR_OFFSET);
	SX_GET(fw.fw_month,   outbox, QUERY_FW_FW_MONTH_OFFSET);
	SX_GET(fw.fw_day,     outbox, QUERY_FW_FW_DAY_OFFSET);

	if (NULL != query_fw) {
		query_fw->core_clk = fw.core_clock;
		query_fw->dt = fw.debug_trace;
		query_fw->fw_year = fw.fw_year;
		query_fw->fw_month = fw.fw_month;
		query_fw->fw_day = fw.fw_day;
		query_fw->fw_hour = fw.fw_hour;
		query_fw->fw_minutes = fw.fw_minutes;
		query_fw->fw_seconds = fw.fw_seconds;
		query_fw->fw_rev = fw.fw_ver;
	}

	if (0 == sx_priv(dev)->is_fw_initialized &&
		DPT_PATH_PCI_E == sx_glb.sx_dpt.dpt_info[target_dev_id].cmd_path) {
		memcpy(&sx_priv(dev)->fw, &fw, sizeof(fw));
		sx_priv(dev)->is_fw_initialized = 1;
	}

out:
	sx_free_cmd_mailbox(dev, mailbox);
	return err;
}
EXPORT_SYMBOL(sx_QUERY_FW);

int sx_QUERY_FW_2(struct sx_dev *dev, int sx_dev_id)
{
	struct sx_fw  *fw  = &sx_priv(dev)->fw;
	struct sx_cmd_mailbox *mailbox;
	int err = 0;
	u32 out_mb_info, in_mb_info;

#define QUERY_FW_OUT_MB_INFO_OFFSET	0x00
#define QUERY_FW_IN_MB_INFO_OFFSET	0x04
	mailbox = sx_alloc_cmd_mailbox(dev, dev->device_id);
	if (IS_ERR(mailbox))
		return PTR_ERR(mailbox);
	mailbox->imm_data = 0ULL;
	err = sx_cmd_imm(dev, sx_dev_id, 0, mailbox, 0, 1, SX_CMD_QUERY_FW,
			    SX_CMD_TIME_CLASS_A, QUERY_FW_IN_MB_SIZE);
	if (err)
		goto out;

	in_mb_info = mailbox->imm_data >> 32;
	out_mb_info = mailbox->imm_data & 0xFFFFFFFFUL;

	/* TODO: what about endianess?? */
	if (dev->device_id == sx_dev_id) {
		fw->local_in_mb_offset	= (in_mb_info & 0x000FFFFF);
		fw->local_in_mb_size	= in_mb_info >> 20;
		fw->local_out_mb_offset	= (out_mb_info & 0x000FFFFF);
		fw->local_out_mb_size	= out_mb_info >> 20;
	}
#ifdef SX_DEBUG
	printk(KERN_INFO PFX "sx_QUERY_FW_2 for dev_id %u before:\n"
			"in_mb_offset=0x%x\n"
			"in_mb_size=%u\n"
			"out_mb_offset=0x%x\n"
			"out_mb_size=%u\n",
			sx_dev_id,
			sx_glb.sx_dpt.dpt_info[sx_dev_id].in_mb_offset,
			sx_glb.sx_dpt.dpt_info[sx_dev_id].in_mb_size,
			sx_glb.sx_dpt.dpt_info[sx_dev_id].out_mb_offset,
			sx_glb.sx_dpt.dpt_info[sx_dev_id].out_mb_size);
#endif
	sx_glb.sx_dpt.dpt_info[sx_dev_id].in_mb_offset = (in_mb_info & 0x000FFFFF);
	sx_glb.sx_dpt.dpt_info[sx_dev_id].in_mb_size = in_mb_info >> 20;
	sx_glb.sx_dpt.dpt_info[sx_dev_id].out_mb_offset = (out_mb_info & 0x000FFFFF);
	sx_glb.sx_dpt.dpt_info[sx_dev_id].out_mb_size = out_mb_info >> 20;
#ifdef SX_DEBUG
	printk(KERN_INFO PFX "sx_QUERY_FW_2 for dev_id %u results:\n"
			"in_mb_offset=0x%x\n"
			"in_mb_size=%u\n"
			"out_mb_offset=0x%x\n"
			"out_mb_size=%u\n",
			sx_dev_id,
			sx_glb.sx_dpt.dpt_info[sx_dev_id].in_mb_offset,
			sx_glb.sx_dpt.dpt_info[sx_dev_id].in_mb_size,
			sx_glb.sx_dpt.dpt_info[sx_dev_id].out_mb_offset,
			sx_glb.sx_dpt.dpt_info[sx_dev_id].out_mb_size);
#endif
out:
	sx_free_cmd_mailbox(dev, mailbox);
	return err;
}

int sx_map_cmd(struct sx_dev *dev, u16 op, struct sx_icm *icm)
{
	struct sx_cmd_mailbox *mailbox;
	struct sx_icm_iter iter;
	__be64 *pages;
	int lg;
	int nent = 0;
	int i;
	int err = 0;
	int ts = 0, tc = 0;

	mailbox = sx_alloc_cmd_mailbox(dev, dev->device_id);
	if (IS_ERR(mailbox))
		return PTR_ERR(mailbox);
	memset(mailbox->buf, 0, SX_MAILBOX_SIZE);
	pages = mailbox->buf;

	for (sx_icm_first(icm, &iter);
	     !sx_icm_last(&iter);
	     sx_icm_next(&iter)) {
		/*
		 * We have to pass pages that are aligned to their
		 * size, so find the least significant 1 in the
		 * address or size and use that as our log2 size.
		 */
		lg = ffs(sx_icm_addr(&iter) | sx_icm_size(&iter)) - 1;
		if (lg < SX_ICM_PAGE_SHIFT) {
			sx_warn(dev, "Got FW area not aligned to "
					"%d (%llx/%lx).\n", SX_ICM_PAGE_SIZE,
				   (unsigned long long) sx_icm_addr(&iter),
				   sx_icm_size(&iter));
			err = -EINVAL;
			goto out;
		}

		for (i = 0; i < sx_icm_size(&iter) >> lg; ++i) {
			pages[nent] =
				cpu_to_be64((sx_icm_addr(&iter) + (i << lg)) |
					    (lg - SX_ICM_PAGE_SHIFT));
			ts += 1 << (lg - 10);
			++tc;

			if (++nent == SX_MAILBOX_SIZE / 16) {
				err = sx_cmd(dev, dev->device_id, mailbox,
					nent, 0, op,
					SX_CMD_TIME_CLASS_B,
					sx_priv(dev)->fw.local_in_mb_size);
				if (err)
					goto out;
				nent = 0;
			}
		}
	}

	if (nent)
		err = sx_cmd(dev, dev->device_id, mailbox, nent, 0, op,
				SX_CMD_TIME_CLASS_B, 0);
	if (err)
		goto out;

	switch (op) {
	case SX_CMD_MAP_FA:
		sx_dbg(dev, "Mapped %d chunks/%d KB for FW.\n", tc, ts);
		break;
	}

out:
	sx_free_cmd_mailbox(dev, mailbox);
	return err;
}


int sx_MAP_FA(struct sx_dev *dev, struct sx_icm *icm)
{
	return sx_map_cmd(dev, SX_CMD_MAP_FA, icm);
}

int sx_UNMAP_FA(struct sx_dev *dev)
{
	return sx_cmd(dev, dev->device_id, 0, 0, 0, SX_CMD_UNMAP_FA,
			SX_CMD_TIME_CLASS_B,
			sx_priv(dev)->fw.local_in_mb_size);
}

int sx_QUERY_AQ_CAP(struct sx_dev *dev)
{
	struct sx_cmd_mailbox *mailbox;
	u32 *outbox;
	u8 field;
	int err;
	struct sx_dev_cap *dev_cap = &dev->dev_cap;

#define QUERY_DEV_CAP_MAX_SDQ_SZ_OFFSET		0x0
#define QUERY_DEV_CAP_MAX_SDQ_OFFSET		0x3
#define QUERY_DEV_CAP_MAX_RDQ_SZ_OFFSET		0x4
#define QUERY_DEV_CAP_MAX_RDQ_OFFSET		0x7
#define QUERY_DEV_CAP_MAX_CQ_SZ_OFFSET		0x8
#define QUERY_DEV_CAP_MAX_CQ_OFFSET		0xb
#define QUERY_DEV_CAP_MAX_EQ_SZ_OFFSET		0xc
#define QUERY_DEV_CAP_MAX_EQ_OFFSET		0xf
#define QUERY_DEV_CAP_MAX_SG_SQ_OFFSET		0x12
#define QUERY_DEV_CAP_MAX_SG_RQ_OFFSET		0x13

	mailbox = sx_alloc_cmd_mailbox(dev, dev->device_id);
	if (IS_ERR(mailbox))
		return PTR_ERR(mailbox);
	outbox = mailbox->buf;

	err = sx_cmd_box(dev, dev->device_id, 0, mailbox, 0, 0,
			SX_CMD_QUERY_AQ_CAP, SX_CMD_TIME_CLASS_A, 0);

	if (err)
		goto out;

	SX_GET(field, outbox, QUERY_DEV_CAP_MAX_SDQ_SZ_OFFSET);
	dev_cap->log_max_sdq_sz = min((int)field, SX_MAX_LOG_DQ_SIZE);
	SX_GET(field, outbox, QUERY_DEV_CAP_MAX_SDQ_OFFSET);
	dev_cap->max_num_sdqs = field;
	SX_GET(field, outbox, QUERY_DEV_CAP_MAX_RDQ_SZ_OFFSET);
	dev_cap->log_max_rdq_sz = min((int)field, SX_MAX_LOG_DQ_SIZE);
	SX_GET(field, outbox, QUERY_DEV_CAP_MAX_RDQ_OFFSET);
	dev_cap->max_num_rdqs = field;
	SX_GET(field, outbox, QUERY_DEV_CAP_MAX_CQ_SZ_OFFSET);
	dev_cap->log_max_cq_sz = field;
	SX_GET(field, outbox, QUERY_DEV_CAP_MAX_CQ_OFFSET);
	dev_cap->max_num_cqs = field;
	SX_GET(field, outbox, QUERY_DEV_CAP_MAX_EQ_SZ_OFFSET);
	dev_cap->log_max_eq_sz = field;
	SX_GET(field, outbox, QUERY_DEV_CAP_MAX_EQ_OFFSET);
	dev_cap->max_num_eqs = field;
	SX_GET(field, outbox, QUERY_DEV_CAP_MAX_SG_SQ_OFFSET);
	dev_cap->max_sg_sq = field;
	SX_GET(field, outbox, QUERY_DEV_CAP_MAX_SG_RQ_OFFSET);
	dev_cap->max_sg_rq = field;

	sx_dbg(dev, "Log Max SDQ sz: %d, num SDQs: %d\n",
		  dev_cap->log_max_sdq_sz, dev_cap->max_num_sdqs);
	sx_dbg(dev, "Log Max RDQ sz: %d, num RDQs: %d\n",
		  dev_cap->log_max_rdq_sz, dev_cap->max_num_rdqs);
	sx_dbg(dev, "Log Max CQ sz: %d, num CQs: %d\n",
		  dev_cap->log_max_cq_sz, dev_cap->max_num_cqs);
	sx_dbg(dev, "Log Max EQ sz: %d, num EQs: %d\n",
		  dev_cap->log_max_eq_sz, dev_cap->max_num_eqs);

out:
	sx_free_cmd_mailbox(dev, mailbox);
	return err;
}
EXPORT_SYMBOL(sx_QUERY_AQ_CAP);

int sx_QUERY_BOARDINFO(struct sx_dev *dev, struct sx_board *board)
{
	struct sx_cmd_mailbox *mailbox;
	u32 *outbox;
	int err;

#define QUERY_ADAPTER_INTA_PIN_OFFSET      0x10
#define QUERY_ADAPTER_VSD_VENDOR_ID_OFFSET 0x1e
#define QUERY_ADAPTER_VSD_OFFSET           0x20

	memset(board, 0, sizeof(*board));
	mailbox = sx_alloc_cmd_mailbox(dev, dev->device_id);
	if (IS_ERR(mailbox))
		return PTR_ERR(mailbox);

	outbox = mailbox->buf;
	err = sx_cmd_box(dev, dev->device_id, 0, mailbox, 0, 0,
			SX_CMD_QUERY_BOARDINFO, SX_CMD_TIME_CLASS_A, 0);
	if (err)
		goto out;

	SX_GET(board->vsd_vendor_id, outbox,
			QUERY_ADAPTER_VSD_VENDOR_ID_OFFSET);
	SX_GET(board->inta_pin, outbox, QUERY_ADAPTER_INTA_PIN_OFFSET);
	sx_dbg(dev, "sx_QUERY_ADAPTER: inta_pin = 0x%x\n", board->inta_pin);

	if (board->vsd_vendor_id == PCI_VENDOR_ID_MELLANOX)
		get_board_id(outbox + QUERY_ADAPTER_VSD_OFFSET / 4,
				board->board_id);

out:
	sx_free_cmd_mailbox(dev, mailbox);
	return err;
}
EXPORT_SYMBOL(sx_QUERY_BOARDINFO);

static void set_opoeration_tlv(void *inbox, struct ku_operation_tlv *op_tlv)
{
	u16 type_len = 0;
	u8 dr_status = 0;
	u8 r_method = 0;

#define TYPE_LEN_OFFSET		0x00
#define DR_STATUS_OFFSET	0x02
#define REGISTER_ID_OFFSET	0x04
#define R_METHOD_OFFSET		0x06
#define CLASS_OFFSET		0x07
#define TID_OFFSET		0x08

	type_len = op_tlv->length | (op_tlv->type << 11);
	SX_PUT(inbox, type_len, TYPE_LEN_OFFSET);
	dr_status = op_tlv->status | (op_tlv->dr << 7);
	SX_PUT(inbox, dr_status, DR_STATUS_OFFSET);
	SX_PUT(inbox, op_tlv->register_id, REGISTER_ID_OFFSET);
	r_method = op_tlv->method | (op_tlv->r << 7);
	SX_PUT(inbox, r_method, R_METHOD_OFFSET);
	SX_PUT(inbox, op_tlv->op_class, CLASS_OFFSET);
	SX_PUT(inbox, op_tlv->tid, TID_OFFSET);
}

#define OPERATION_TLV_SIZE	0x10
#define IN_MB_SIZE(reg_dword_size) \
	(((reg_dword_size) * 4) + OPERATION_TLV_SIZE)
static void get_operation_tlv(void *outbox, struct ku_operation_tlv *op_tlv)
{
	u16 type_len = 0;
	u8 dr_status = 0;
	u8 r_method = 0;

#define TYPE_LEN_OFFSET		0x00
#define DR_STATUS_OFFSET	0x02
#define REGISTER_ID_OFFSET	0x04
#define R_METHOD_OFFSET		0x06
#define CLASS_OFFSET		0x07
#define TID_OFFSET		0x08

	SX_GET(type_len, outbox, TYPE_LEN_OFFSET);
	op_tlv->length = type_len & 0x7ff;
	op_tlv->type = type_len >> 11;
	SX_GET(dr_status, outbox, DR_STATUS_OFFSET);
	op_tlv->status = dr_status & 0x7f;
	op_tlv->dr = dr_status >> 7;
	SX_GET(op_tlv->register_id, outbox, REGISTER_ID_OFFSET);
	SX_GET(r_method, outbox, R_METHOD_OFFSET);
	op_tlv->method = r_method & 0x7f;
	op_tlv->r = r_method >> 7;
	SX_GET(op_tlv->op_class, outbox, CLASS_OFFSET);
	SX_GET(op_tlv->tid, outbox, TID_OFFSET);
	if (op_tlv->status) {
		if (op_tlv->register_id != MCIA_REG_ID)
			printk(KERN_WARNING PFX "get_operation_tlv: Err: Got status "
				"0x%x for register 0x%x\n",
				op_tlv->status, op_tlv->register_id);
		else
			printk(KERN_WARNING PFX "MCIA register reported "
					"%s error\n", mcia_err_str(op_tlv->status));
	}
}

#define REG_TLV_OFFSET		0x10
#define REG_TLV_TYPE		0x03
int sx_ACCESS_REG_MGIR(struct sx_dev *dev, struct ku_access_mgir_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_HW_INFO_OFFSET			0x14
#define REG_HW_INFO_DEVICE_HW_REVISION_OFFSET	0x0
#define REG_HW_INFO_DEVICE_ID_OFFSET		0x2
#define REG_HW_INFO_DVFS_OFFSET			0x7
#define REG_HW_INFO_UPTIME_OFFSET		0x1c

#define REG_FW_INFO_OFFSET			0x34
#define REG_FW_INFO_MAJOR_OFFSET		0x01
#define REG_FW_INFO_MINOR_OFFSET		0x02
#define REG_FW_INFO_SUB_MINOR_OFFSET		0x03
#define REG_FW_INFO_BUILD_ID_OFFSET		0x04
#define REG_FW_INFO_MONTH_OFFSET		0x08
#define REG_FW_INFO_DAY_OFFSET			0x09
#define REG_FW_INFO_YEAR_OFFSET			0x0a
#define REG_FW_INFO_HOUR_OFFSET			0x0e
#define REG_FW_INFO_PSID_OFFSET			0x10
#define REG_FW_INFO_INI_FILE_VERSION_OFFSET	0x20
#define REG_FW_INFO_EXTENDED_MAJOR_OFFSET	0x24
#define REG_FW_INFO_EXTENDED_MINOR_OFFSET	0x28
#define REG_FW_INFO_EXTENDED_SUB_MINOR_OFFSET	0x2c

#define REG_SW_INFO_OFFSET			0x74
#define REG_SW_INFO_MAJOR_OFFSET		0x01
#define REG_SW_INFO_MINOR_OFFSET		0x02
#define REG_SW_INFO_SUB_MINOR_OFFSET		0x03

#define MGIR_REG_LEN				0x21


	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MGIR_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
#if 0 /* This is a RO register */
	SX_PUT(inbox, reg_data->mgir_reg.hw_info.device_hw_revision,
			REG_HW_INFO_OFFSET + REG_HW_INFO_DEVICE_HW_REVISION_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.hw_info.device_id,
			REG_HW_INFO_OFFSET + REG_HW_INFO_DEVICE_ID_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.hw_info.dvfs,
			REG_HW_INFO_OFFSET + REG_HW_INFO_DVFS_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.hw_info.uptime,
			REG_HW_INFO_OFFSET + REG_HW_INFO_UPTIME_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.fw_info.major,
			REG_FW_INFO_OFFSET + REG_FW_INFO_MAJOR_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.fw_info.minor,
			REG_FW_INFO_OFFSET + REG_FW_INFO_MINOR_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.fw_info.sub_minor,
			REG_FW_INFO_OFFSET + REG_FW_INFO_SUB_MINOR_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.fw_info.build_id,
			REG_FW_INFO_OFFSET + REG_FW_INFO_BUILD_ID_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.fw_info.month,
			REG_FW_INFO_OFFSET + REG_FW_INFO_MONTH_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.fw_info.day,
			REG_FW_INFO_OFFSET + REG_FW_INFO_DAY_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.fw_info.year,
			REG_FW_INFO_OFFSET + REG_FW_INFO_YEAR_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.fw_info.hour,
			REG_FW_INFO_OFFSET + REG_FW_INFO_HOUR_OFFSET);
	memcpy(inbox + REG_FW_INFO_OFFSET + REG_FW_INFO_PSID_OFFSET,
			reg_data->mgir_reg.fw_info.psid, SX_PSID_SIZE);
	SX_PUT(inbox, reg_data->mgir_reg.fw_info.ini_file_version,
			REG_FW_INFO_OFFSET + REG_FW_INFO_INI_FILE_VERSION_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.fw_info.extended_major,
			REG_FW_INFO_OFFSET + REG_FW_INFO_EXTENDED_MAJOR_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.fw_info.extended_minor,
			REG_FW_INFO_OFFSET + REG_FW_INFO_EXTENDED_MINOR_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.fw_info.extended_sub_minor,
			REG_FW_INFO_OFFSET + REG_FW_INFO_EXTENDED_SUB_MINOR_OFFSET);

	SX_PUT(inbox, reg_data->mgir_reg.sw_info.major,
			REG_SW_INFO_OFFSET + REG_SW_INFO_MAJOR_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.sw_info.minor,
			REG_FW_INFO_OFFSET + REG_SW_INFO_MINOR_OFFSET);
	SX_PUT(inbox, reg_data->mgir_reg.sw_info.sub_minor,
			REG_SW_INFO_OFFSET + REG_SW_INFO_SUB_MINOR_OFFSET);
#endif
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MGIR_REG_LEN));
printk("%s err=%d\n", __func__, err);
	if (err)
		goto out;
printk("%s err=%d oper=%d\n", __func__, err, reg_data->op_tlv.method);
	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mgir_reg.hw_info.device_hw_revision, outbox,
				REG_HW_INFO_OFFSET + REG_HW_INFO_DEVICE_HW_REVISION_OFFSET);
		SX_GET(reg_data->mgir_reg.hw_info.device_id, outbox,
				REG_HW_INFO_OFFSET + REG_HW_INFO_DEVICE_ID_OFFSET);
		SX_GET(reg_data->mgir_reg.hw_info.dvfs, outbox,
				REG_HW_INFO_OFFSET + REG_HW_INFO_DVFS_OFFSET);
		reg_data->mgir_reg.hw_info.dvfs &= 0x1f;
		SX_GET(reg_data->mgir_reg.hw_info.uptime, outbox,
				REG_HW_INFO_OFFSET + REG_HW_INFO_UPTIME_OFFSET);
		SX_GET(reg_data->mgir_reg.fw_info.major, outbox,
				REG_FW_INFO_OFFSET + REG_FW_INFO_MAJOR_OFFSET);
		SX_GET(reg_data->mgir_reg.fw_info.minor, outbox,
				REG_FW_INFO_OFFSET + REG_FW_INFO_MINOR_OFFSET);
		SX_GET(reg_data->mgir_reg.fw_info.sub_minor, outbox,
				REG_FW_INFO_OFFSET + REG_FW_INFO_SUB_MINOR_OFFSET);
		SX_GET(reg_data->mgir_reg.fw_info.build_id, outbox,
				REG_FW_INFO_OFFSET + REG_FW_INFO_BUILD_ID_OFFSET);
		SX_GET(reg_data->mgir_reg.fw_info.month, outbox,
				REG_FW_INFO_OFFSET + REG_FW_INFO_MONTH_OFFSET);
		SX_GET(reg_data->mgir_reg.fw_info.day, outbox,
				REG_FW_INFO_OFFSET + REG_FW_INFO_DAY_OFFSET);
		SX_GET(reg_data->mgir_reg.fw_info.year, outbox,
				REG_FW_INFO_OFFSET + REG_FW_INFO_YEAR_OFFSET);
		SX_GET(reg_data->mgir_reg.fw_info.hour, outbox,
				REG_FW_INFO_OFFSET + REG_FW_INFO_HOUR_OFFSET);
		memcpy(reg_data->mgir_reg.fw_info.psid,
			(u8 *)outbox + REG_FW_INFO_OFFSET + REG_FW_INFO_PSID_OFFSET,
			SX_PSID_SIZE);
		SX_GET(reg_data->mgir_reg.fw_info.ini_file_version, outbox,
				REG_FW_INFO_OFFSET + REG_FW_INFO_INI_FILE_VERSION_OFFSET);
		SX_GET(reg_data->mgir_reg.fw_info.extended_major, outbox,
				REG_FW_INFO_OFFSET + REG_FW_INFO_EXTENDED_MAJOR_OFFSET);
		SX_GET(reg_data->mgir_reg.fw_info.extended_minor, outbox,
				REG_FW_INFO_OFFSET + REG_FW_INFO_EXTENDED_MINOR_OFFSET);
		SX_GET(reg_data->mgir_reg.fw_info.extended_sub_minor, outbox,
				REG_FW_INFO_OFFSET + REG_FW_INFO_EXTENDED_SUB_MINOR_OFFSET);
		SX_GET(reg_data->mgir_reg.sw_info.major, outbox,
				REG_SW_INFO_OFFSET + REG_SW_INFO_MAJOR_OFFSET);
		SX_GET(reg_data->mgir_reg.sw_info.minor, outbox,
				REG_FW_INFO_OFFSET + REG_SW_INFO_MINOR_OFFSET);
		SX_GET(reg_data->mgir_reg.sw_info.sub_minor, outbox,
				REG_SW_INFO_OFFSET + REG_SW_INFO_SUB_MINOR_OFFSET);
	}

#ifdef NO_PCI_XX
	/* simulate for VM systems */
	reg_data->mgir_reg.hw_info.device_id = SXD_MGIR_HW_DEV_ID_SX;
	reg_data->mgir_reg.hw_info.device_hw_revision =	SXD_MGIR_HW_REV_ID_SX_A2;
#endif

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_MGIR);

int sx_ACCESS_REG_PLIB(struct sx_dev *dev, struct ku_access_plib_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_LOCAL_PORT_OFFSET	0x15
#define REG_IB_PORT_OFFSET	0x17
#define PLIB_REG_LEN		0x05

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PLIB_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->plib_reg.local_port, REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->plib_reg.ib_port, REG_IB_PORT_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PLIB_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->plib_reg.local_port, outbox,
				REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->plib_reg.ib_port, outbox,
				REG_IB_PORT_OFFSET);

	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PLIB);

int sx_ACCESS_REG_PMLP(struct sx_dev *dev, struct ku_access_pmlp_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	int i;
	u8 tmp_u8;

#define REG_LOCAL_PORT_OFFSET	0x15
#define REG_DIFF_RX_TX_OFFSET	0x14
#define REG_WIDTH_OFFSET	0x17
#define REG_RX_LANE_0_OFFSET	0x18
#define REG_LANE_0_OFFSET	0x19
#define REG_MODULE_0_OFFSET	0x1b
#define PMLP_REG_LEN		0x11

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PMLP_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	tmp_u8 = reg_data->pmlp_reg.use_different_rx_tx << 7;
	SX_PUT(inbox, tmp_u8, REG_DIFF_RX_TX_OFFSET);
	SX_PUT(inbox, reg_data->pmlp_reg.local_port, REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->pmlp_reg.width, REG_WIDTH_OFFSET);
	for (i = 0; i < NUMBER_OF_SERDESES; i++) {
		SX_PUT(inbox, reg_data->pmlp_reg.rx_lane[i],
				REG_RX_LANE_0_OFFSET + (4 * i));
		SX_PUT(inbox, reg_data->pmlp_reg.lane[i],
				REG_LANE_0_OFFSET + (4 * i));
		SX_PUT(inbox, reg_data->pmlp_reg.module[i],
				REG_MODULE_0_OFFSET + (4 * i));
	}

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PMLP_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(tmp_u8, outbox, REG_DIFF_RX_TX_OFFSET);
		reg_data->pmlp_reg.use_different_rx_tx = tmp_u8 >> 7;
		SX_GET(reg_data->pmlp_reg.local_port, outbox,
				REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->pmlp_reg.width, outbox,
				REG_WIDTH_OFFSET);

		for (i = 0; i < NUMBER_OF_SERDESES; i++) {
			SX_GET(reg_data->pmlp_reg.rx_lane[i], outbox,
					REG_RX_LANE_0_OFFSET + (4 * i));
			SX_GET(reg_data->pmlp_reg.lane[i], outbox,
					REG_LANE_0_OFFSET + (4 * i));
			SX_GET(reg_data->pmlp_reg.module[i], outbox,
					REG_MODULE_0_OFFSET + (4 * i));
		}
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PMLP);

int sx_ACCESS_REG_MHSR(struct sx_dev *dev,struct ku_access_mhsr_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_HEALTH_OFFSET	0x17
#define MHSR_REG_LEN		0x2

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MHSR_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mhsr_reg.health, REG_HEALTH_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MHSR_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mhsr_reg.health, outbox,
				REG_HEALTH_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_MHSR);

int sx_ACCESS_REG_PTYS(struct sx_dev *dev, struct ku_access_ptys_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_LOCAL_PORT_OFFSET	0x15
#define REG_PROTO_MASK_OFFSET	0x17
#define REG_FC_CAP_OFFSET	0x1c
#define REG_ETH_CAP_OFFSET	0x20
#define REG_IB_CAP_OFFSET	0x24
#define REG_FC_ADMIN_OFFSET	0x28
#define REG_ETH_ADMIN_OFFSET	0x2c
#define REG_IB_ADMIN_OFFSET	0x30
#define REG_FC_OPER_OFFSET	0x34
#define REG_ETH_OPER_OFFSET	0x38
#define REG_IB_OPER_OFFSET	0x3c
#define PTYS_REG_LEN		0x11

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PTYS_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->ptys_reg.local_port, REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->ptys_reg.proto_mask, REG_PROTO_MASK_OFFSET);
	SX_PUT(inbox, reg_data->ptys_reg.fc_proto_capability,
			REG_FC_CAP_OFFSET);
	SX_PUT(inbox, reg_data->ptys_reg.eth_proto_capability,
			REG_ETH_CAP_OFFSET);
	SX_PUT(inbox, reg_data->ptys_reg.ib_proto_capability,
			REG_IB_CAP_OFFSET);
	SX_PUT(inbox, reg_data->ptys_reg.fc_proto_admin, REG_FC_ADMIN_OFFSET);
	SX_PUT(inbox, reg_data->ptys_reg.eth_proto_admin,
			REG_ETH_ADMIN_OFFSET);
	SX_PUT(inbox, reg_data->ptys_reg.ib_proto_admin, REG_IB_ADMIN_OFFSET);
	SX_PUT(inbox, reg_data->ptys_reg.fc_proto_oper, REG_FC_OPER_OFFSET);
	SX_PUT(inbox, reg_data->ptys_reg.eth_proto_oper, REG_ETH_OPER_OFFSET);
	SX_PUT(inbox, reg_data->ptys_reg.ib_proto_oper, REG_IB_OPER_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PTYS_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->ptys_reg.local_port, outbox,
				REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->ptys_reg.proto_mask, outbox,
				REG_PROTO_MASK_OFFSET);
		SX_GET(reg_data->ptys_reg.fc_proto_capability, outbox,
				REG_FC_CAP_OFFSET);
		SX_GET(reg_data->ptys_reg.eth_proto_capability, outbox,
				REG_ETH_CAP_OFFSET);
		SX_GET(reg_data->ptys_reg.ib_proto_capability, outbox,
				REG_IB_CAP_OFFSET);
		SX_GET(reg_data->ptys_reg.fc_proto_admin, outbox,
				REG_FC_ADMIN_OFFSET);
		SX_GET(reg_data->ptys_reg.eth_proto_admin, outbox,
				REG_ETH_ADMIN_OFFSET);
		SX_GET(reg_data->ptys_reg.ib_proto_admin, outbox,
				REG_IB_ADMIN_OFFSET);
		SX_GET(reg_data->ptys_reg.fc_proto_oper, outbox,
				REG_FC_OPER_OFFSET);
		SX_GET(reg_data->ptys_reg.eth_proto_oper, outbox,
				REG_ETH_OPER_OFFSET);
		SX_GET(reg_data->ptys_reg.ib_proto_oper, outbox,
				REG_IB_OPER_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PTYS);

int sx_ACCESS_REG_QSTCT(struct sx_dev *dev,
		struct ku_access_qstct_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_SWID_OFFSET		0x14
#define REG_PRIO_OFFSET		0x16
#define REG_UTCLASS_OFFSET	0x1b
#define REG_MTCLASS_OFFSET	0x1f
#define QSTCT_REG_LEN		0x05

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= QSTCT_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->qstct_reg.swid, REG_SWID_OFFSET);
	SX_PUT(inbox, reg_data->qstct_reg.prio, REG_PRIO_OFFSET);
	SX_PUT(inbox, reg_data->qstct_reg.utclass, REG_UTCLASS_OFFSET);
	SX_PUT(inbox, reg_data->qstct_reg.mtclass, REG_MTCLASS_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(QSTCT_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->qstct_reg.swid, outbox,
				REG_SWID_OFFSET);
		SX_GET(reg_data->qstct_reg.prio, outbox, REG_PRIO_OFFSET);
		SX_GET(reg_data->qstct_reg.utclass, outbox,
				REG_UTCLASS_OFFSET);
		SX_GET(reg_data->qstct_reg.mtclass, outbox,
				REG_MTCLASS_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_QSTCT);

int sx_ACCESS_REG_QSPTC(struct sx_dev *dev,
		struct ku_access_qsptc_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_LOCAL_IPORT_OFFSET	0x14
#define REG_LOCAL_EPORT_OFFSET	0x15
#define REG_ITCLASS_OFFSET	0x16
#define REG_TCLASS_OFFSET	0x1b
#define QSPTC_REG_LEN		0x03

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= QSPTC_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->qsptc_reg.local_iport, REG_LOCAL_IPORT_OFFSET);
	SX_PUT(inbox, reg_data->qsptc_reg.local_eport, REG_LOCAL_EPORT_OFFSET);
	SX_PUT(inbox, reg_data->qsptc_reg.itclass, REG_ITCLASS_OFFSET);
	SX_PUT(inbox, reg_data->qsptc_reg.tclass, REG_TCLASS_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(QSPTC_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->qsptc_reg.local_iport, outbox,
				REG_LOCAL_IPORT_OFFSET);
		SX_GET(reg_data->qsptc_reg.local_eport, outbox,
				REG_LOCAL_EPORT_OFFSET);
		SX_GET(reg_data->qsptc_reg.itclass, outbox,
				REG_ITCLASS_OFFSET);
		SX_GET(reg_data->qsptc_reg.tclass, outbox, REG_TCLASS_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_QSPTC);

int sx_ACCESS_REG_PSPA(struct sx_dev *dev, struct ku_access_pspa_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_SWID_OFFSET		0x14
#define REG_LOCAL_PORT_OFFSET	0x15
#define REG_SUB_PORT_OFFSET	0x16
#define PSPA_REG_LEN		0x03

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PSPA_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->pspa_reg.swid, REG_SWID_OFFSET);
	SX_PUT(inbox, reg_data->pspa_reg.local_port, REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->pspa_reg.sub_port, REG_SUB_PORT_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PSPA_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->pspa_reg.swid, outbox, REG_SWID_OFFSET);
		SX_GET(reg_data->pspa_reg.local_port, outbox,
				REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->pspa_reg.sub_port, outbox,
				REG_SUB_PORT_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PSPA);

int sx_ACCESS_REG_SPZR(struct sx_dev *dev, struct ku_access_spzr_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u8 tmp_val_u8 = 0;
	int i;

#define REG_SWID_OFFSET			0x14
#define REG_NDM_OFFSET			0x16
#define REG_NDM_BIT_N			7
#define REG_ENH_SW_P0_MASK_OFFSET	0x16
#define REG_ENH_SW_P0_MASK_BIT_N	6
#define REG_CM_OFFSET			0x16
#define REG_CM_BIT_N			5
#define REG_VK_OFFSET			0x16
#define REG_VK_BIT_N			4
#define REG_MP_OFFSET			0x16
#define REG_MP_BIT_N			3
#define REG_SIG_OFFSET			0x16
#define REG_SIG_BIT_N			2
#define REG_NG_OFFSET			0x16
#define REG_NG_BIT_N			1
#define REG_G0_OFFSET			0x16
#define REG_G0_BIT_N			0
#define REG_ENH_SW_P0_OFFSET		0x17
#define REG_ENH_SW_P0_BIT_N		1
#define REG_CAP_MASK_OFFSET		0x18
#define REG_SYS_IMG_GUID_H_OFFSET	0x1c
#define REG_SYS_IMG_GUID_L_OFFSET	0x20
#define REG_GUID0_H_OFFSET		0x24
#define REG_GUID0_L_OFFSET		0x28
#define REG_NODE_GUID_H_OFFSET		0x2c
#define REG_NODE_GUID_L_OFFSET		0x30
#define REG_V_KEY_H_OFFSET		0x34
#define REG_V_KEY_L_OFFSET		0x38
#define REG_MAX_PKEY_OFFSET		0x3c
#define REG_NODE_DESC_OFFSET		0x44
#define SPZR_REG_LEN			0x41

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;
	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= SPZR_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	/* SWitch partition ID */
	SX_PUT(inbox, reg_data->spzr_reg.swid, REG_SWID_OFFSET);
	tmp_val_u8 = 0;
	/* Node description mask.Set to 1 to write the NodeDescription field */
	tmp_val_u8 |= (reg_data->spzr_reg.ndm ? (1<<REG_NDM_BIT_N) : 0);
	/* Enhanced Switch Port 0 mask. When 1, the EnhSwP0 field is valid
	 * (see below). When 0, the EnhSwP0 field is ignored */
	tmp_val_u8 |= (reg_data->spzr_reg.EnhSwP0_mask ?
			(1<<REG_ENH_SW_P0_MASK_BIT_N) : 0);
	/* Set PortInfo:CapabilityMask to PortInfo:CapabilityMask specified */
	tmp_val_u8 |= (reg_data->spzr_reg.cm ? (1<<REG_CM_BIT_N) : 0);
	/* Set the internal GSA V_Key. Incoming VendorSpecific MADs must have
	 * matching V_Key to the one set by this command */
	tmp_val_u8 |= (reg_data->spzr_reg.vk ? (1<<REG_VK_BIT_N) : 0);
	/* Change PKey table size to max_pkey */
	tmp_val_u8 |= (reg_data->spzr_reg.mp ? (1<<REG_MP_BIT_N) : 0);
	/* Set System Image GUID to system_image_guid specified.
	System_image_guid and sig must be the same for all ports.*/
	tmp_val_u8 |= (reg_data->spzr_reg.sig ? (1<<REG_SIG_BIT_N) : 0);
	/* Set node GUID to node_guid specified.
	node_guid and ng must be the same for all ports.*/
	tmp_val_u8 |= (reg_data->spzr_reg.ng ? (1<<REG_NG_BIT_N) : 0);
	/* Set port GUID0 to GUID0 specified */
	tmp_val_u8 |= (reg_data->spzr_reg.g0 ? (1<<REG_G0_BIT_N) : 0);
	SX_PUT(inbox, tmp_val_u8, REG_NDM_OFFSET);
	tmp_val_u8 = 0;
	/* When set, it enables Enhanced Switch
	 * Port 0. Reported in NodeInfo. Otherwise,
	 * Enhanced Switch Port 0 is disabled. */
	tmp_val_u8 |= (reg_data->spzr_reg.EnhSwP0 ?
			(1<<REG_ENH_SW_P0_BIT_N) : 0);
	SX_PUT(inbox, tmp_val_u8, REG_ENH_SW_P0_OFFSET);
	if (reg_data->spzr_reg.cm)
		/* Sets the PortInfoCapabilityMask:
		Specifies the supported capabilities of this node.
		A bit set to 1 for affirmation of supported capability.*/
		SX_PUT(inbox, reg_data->spzr_reg.capability_mask,
				REG_CAP_MASK_OFFSET);
	if (reg_data->spzr_reg.sig)
		/* System Image GUID, takes effect only if
		 * the sig bit is set. Must be the same for
		 * both ports.*/
		SX_PUT(inbox, reg_data->spzr_reg.system_image_guid_h_l,
				REG_SYS_IMG_GUID_H_OFFSET);
	if (reg_data->spzr_reg.g0)
		/*EUI-64 GUID assigned by the manufacturer,
		 * takes effect only if the g0 bit is set */
		SX_PUT(inbox, reg_data->spzr_reg.guid0_h_l, REG_GUID0_H_OFFSET);
	if (reg_data->spzr_reg.ng)
		/* Node GUID, takes effect only if the ng bit is set
		 * Must be the same for both ports.*/
		SX_PUT(inbox, reg_data->spzr_reg.node_guid_h_l,
				REG_NODE_GUID_H_OFFSET);
	if (reg_data->spzr_reg.vk) {
		/* The internal GSA V_Key. Incoming VendorSpecific MADs must
		 * have a V_Key matching the one set by this command. If not
		 * specified, then must be set to 0. 32 V_Key_l Takes effect
		 * only if the VK bit is set
		 */
		SX_PUT(inbox, reg_data->spzr_reg.v_key_h, REG_V_KEY_H_OFFSET);
		SX_PUT(inbox, reg_data->spzr_reg.v_key_l, REG_V_KEY_L_OFFSET);
	}
	if (reg_data->spzr_reg.mp)
		/* max_pkey is derived from the profile - no set.
		 * Maximum pkeys for the port.
		 * Must be the same for both ports.
		 * Takes effect if the mp bit is set */
		SX_PUT(inbox, reg_data->spzr_reg.max_pkey,
				REG_MAX_PKEY_OFFSET);
	if (reg_data->spzr_reg.ndm)
		/* Text string that describes the node */
		for (i = 0; i < 64; i++)
			SX_PUT(inbox, reg_data->spzr_reg.NodeDescription[i],
					REG_NODE_DESC_OFFSET + i);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(SPZR_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->spzr_reg.swid, outbox, REG_SWID_OFFSET);

		SX_GET(tmp_val_u8, outbox, REG_NDM_OFFSET);
		reg_data->spzr_reg.ndm =
				tmp_val_u8 & (1<<REG_NDM_BIT_N) ? 1 : 0 ;
		reg_data->spzr_reg.EnhSwP0_mask =
			tmp_val_u8 & (1<<REG_ENH_SW_P0_MASK_BIT_N) ? 1 : 0 ;
		reg_data->spzr_reg.cm =
				tmp_val_u8 & (1<<REG_CM_BIT_N)  ? 1 : 0 ;
		reg_data->spzr_reg.vk =
				tmp_val_u8 & (1<<REG_VK_BIT_N)  ? 1 : 0 ;
		reg_data->spzr_reg.mp =
				tmp_val_u8 & (1<<REG_MP_BIT_N)  ? 1 : 0 ;
		reg_data->spzr_reg.sig =
				tmp_val_u8 & (1<<REG_SIG_BIT_N) ? 1 : 0 ;
		reg_data->spzr_reg.ng =
				tmp_val_u8 & (1<<REG_NG_BIT_N)  ? 1 : 0 ;
		reg_data->spzr_reg.g0 =
				tmp_val_u8 & (1<<REG_G0_BIT_N)  ? 1 : 0 ;

		SX_GET(tmp_val_u8, outbox, REG_ENH_SW_P0_OFFSET);
		reg_data->spzr_reg.EnhSwP0 =
				tmp_val_u8 & (1<<REG_ENH_SW_P0_BIT_N) ? 1 : 0 ;

		SX_GET(reg_data->spzr_reg.capability_mask, outbox,
				REG_CAP_MASK_OFFSET);
		SX_GET(reg_data->spzr_reg.system_image_guid_h_l, outbox,
				REG_SYS_IMG_GUID_H_OFFSET);
		SX_GET(reg_data->spzr_reg.guid0_h_l, outbox,
				REG_GUID0_H_OFFSET);
		SX_GET(reg_data->spzr_reg.node_guid_h_l, outbox,
				REG_NODE_GUID_H_OFFSET);
		SX_GET(reg_data->spzr_reg.v_key_h, outbox,
				REG_V_KEY_H_OFFSET);
		SX_GET(reg_data->spzr_reg.v_key_l, outbox,
				REG_V_KEY_L_OFFSET);
		SX_GET(reg_data->spzr_reg.max_pkey, outbox,
				REG_MAX_PKEY_OFFSET);
		for (i = 0; i < 64; i++)
			SX_GET(reg_data->spzr_reg.NodeDescription[i], outbox,
					REG_NODE_DESC_OFFSET + i);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_SPZR);

int sx_ACCESS_REG_MJTAG(struct sx_dev *dev, struct ku_access_mjtag_reg *reg_data)
{
    struct sx_cmd_mailbox *in_mailbox;
    struct sx_cmd_mailbox *out_mailbox;
    u32 *inbox;
    u32 *outbox;
    int err, counter=0;
    u16 type_len = 0;
    u8 tmp_val_u8;

#define MJTAG_REG_CMD_OFFSET                  0x14
#define MJTAG_REG_CMD_BIT_N                   6
#define MJTAG_REG_SEQ_NUM_BIT_N               0x0f
#define MJTAG_REG_SEQ_NUM_OFFSET              0x14
#define MJTAG_REG_TRANSACTIONS_SIZE_OFFSET    0x17
#define MJTAG_REG_JTAG_TRANSACTION_OFFSET     0x18
#define MJTAG_REG_TRANSACTION_TDO_BIT_N       0x03
#define MJTAG_REG_TRANSACTION_TDI_BIT_N       0x01
#define MJTAG_REG_LEN                   0x0c

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}


	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MJTAG_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	tmp_val_u8 = 0;
	tmp_val_u8 |= reg_data->mjtag_reg.cmd << MJTAG_REG_CMD_BIT_N;
	tmp_val_u8 |=reg_data->mjtag_reg.seq_num;
	SX_PUT(inbox, tmp_val_u8, MJTAG_REG_CMD_OFFSET);
	SX_PUT(inbox, reg_data->mjtag_reg.size, MJTAG_REG_TRANSACTIONS_SIZE_OFFSET);
	for(;counter<reg_data->mjtag_reg.size;counter++) {
		tmp_val_u8=0;
		tmp_val_u8 |= (reg_data->mjtag_reg.jtag_transaction_sets[counter].tdo & 0x01) << MJTAG_REG_TRANSACTION_TDO_BIT_N;
		tmp_val_u8 |= (reg_data->mjtag_reg.jtag_transaction_sets[counter].tdi & 0x01) << MJTAG_REG_TRANSACTION_TDI_BIT_N;
		tmp_val_u8 |= (reg_data->mjtag_reg.jtag_transaction_sets[counter].tms & 0x01);
		SX_PUT(inbox,tmp_val_u8, MJTAG_REG_JTAG_TRANSACTION_OFFSET+counter);
	}

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MJTAG_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	SX_GET(tmp_val_u8, outbox, MJTAG_REG_CMD_OFFSET);
	reg_data->mjtag_reg.cmd = tmp_val_u8 & (1 << MJTAG_REG_CMD_BIT_N) ? 1 : 0;
	SX_GET(tmp_val_u8, outbox, MJTAG_REG_SEQ_NUM_OFFSET);
	reg_data->mjtag_reg.seq_num = tmp_val_u8 & MJTAG_REG_SEQ_NUM_BIT_N ? 1 : 0;
	SX_GET(reg_data->mjtag_reg.size, outbox,
	        MJTAG_REG_TRANSACTIONS_SIZE_OFFSET);

	for (counter = 0; counter < reg_data->mjtag_reg.size; counter++) {
		SX_GET(tmp_val_u8, outbox, MJTAG_REG_JTAG_TRANSACTION_OFFSET);
		reg_data->mjtag_reg.jtag_transaction_sets[counter].tdi =
		        tmp_val_u8 & (1 << MJTAG_REG_TRANSACTION_TDI_BIT_N) ? 1 : 0;
		reg_data->mjtag_reg.jtag_transaction_sets[counter].tdo =
		        tmp_val_u8 & (1 << MJTAG_REG_TRANSACTION_TDO_BIT_N) ? 1 : 0;
		reg_data->mjtag_reg.jtag_transaction_sets[counter].tms =
		        tmp_val_u8 & 1 ? 1 : 0;
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_MJTAG);

int sx_ACCESS_REG_PPSC(struct sx_dev *dev, struct ku_access_ppsc_reg *reg_data)
{
    struct sx_cmd_mailbox *in_mailbox;
    struct sx_cmd_mailbox *out_mailbox;
    u32 *inbox;
    u32 *outbox;
    int err;
    u16 type_len = 0;

#define PPSC_REG_LOCAL_PORT_OFFSET                      0x15
#define PPSC_REG_LOCAL_PORT_N                           0x08
#define PPSC_REG_WRPS_ADMIN_OFFSET                      0x27
#define PPSC_REG_WRPS_ADMIN_N                           0x04
#define PPSC_REG_WRPS_STATUS_OFFSET                     0x2B
#define PPSC_REG_WRPS_STATUS_N                          0x04
#define PPSC_REG_UP_THRESHOLD_OFFSET                    0x2D
#define PPSC_REG_UP_THRESHOLD_N                         0x08
#define PPSC_REG_DOWN_THRESHOLD_OFFSET                  0x2D
#define PPSC_REG_DOWN_THRESHOLD_N                       0x08
#define PPSC_REG_SRPS_ADMIN_OFFSET                      0x35
#define PPSC_REG_SRPS_ADMIN_N                           0x04
#define PPSC_REG_SRPS_STATUS_OFFSET                     0x39
#define PPSC_REG_SRPS_STATUS_N                          0x04

#define PPSC_REG_LEN                                    0x0D

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}


	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PPSC_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
    SX_PUT(inbox, reg_data->ppsc_reg.local_port, PPSC_REG_LOCAL_PORT_OFFSET);
    SX_PUT(inbox, reg_data->ppsc_reg.wrps_admin, PPSC_REG_WRPS_ADMIN_OFFSET);
    SX_PUT(inbox, reg_data->ppsc_reg.wrps_status, PPSC_REG_WRPS_STATUS_OFFSET);
    SX_PUT(inbox, reg_data->ppsc_reg.up_threshold, PPSC_REG_UP_THRESHOLD_OFFSET);
    SX_PUT(inbox, reg_data->ppsc_reg.down_threshold, PPSC_REG_DOWN_THRESHOLD_OFFSET);
    SX_PUT(inbox, reg_data->ppsc_reg.srps_admin, PPSC_REG_SRPS_ADMIN_OFFSET);
    SX_PUT(inbox, reg_data->ppsc_reg.srps_status, PPSC_REG_SRPS_STATUS_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PPSC_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
        SX_GET(reg_data->ppsc_reg.local_port, outbox, PPSC_REG_LOCAL_PORT_OFFSET);
        SX_GET(reg_data->ppsc_reg.wrps_admin, outbox, PPSC_REG_WRPS_ADMIN_OFFSET);
        SX_GET(reg_data->ppsc_reg.wrps_status, outbox, PPSC_REG_WRPS_STATUS_OFFSET);
        SX_GET(reg_data->ppsc_reg.up_threshold, outbox,PPSC_REG_UP_THRESHOLD_OFFSET);
        SX_GET(reg_data->ppsc_reg.down_threshold, outbox,PPSC_REG_DOWN_THRESHOLD_OFFSET);
        SX_GET(reg_data->ppsc_reg.srps_admin, outbox, PPSC_REG_SRPS_ADMIN_OFFSET);
        SX_GET(reg_data->ppsc_reg.srps_status, outbox, PPSC_REG_SRPS_STATUS_OFFSET);


	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PPSC);

int sx_ACCESS_REG_PAOS(struct sx_dev *dev, struct ku_access_paos_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u8 tmp_val_u8;

#define REG_SWID_OFFSET			0x14
#define REG_LOCAL_PORT_OFFSET		0x15
#define REG_ADMIN_STATUS_OFFSET		0x16
#define REG_OPER_STATUS_OFFSET		0x17
#define REG_ASE_OFFSET			0x18
#define REG_ASE_BIT_N			7
#define REG_EE_BIT_N			6
#define REG_E_OFFSET			0x1B
#define PAOS_REG_LEN			0x05

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PAOS_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->paos_reg.swid, REG_SWID_OFFSET);
	SX_PUT(inbox, reg_data->paos_reg.local_port, REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->paos_reg.admin_status,
			REG_ADMIN_STATUS_OFFSET);
	tmp_val_u8 = 0;
	tmp_val_u8 |= reg_data->paos_reg.ase ? (1 <<  REG_ASE_BIT_N) : 0;
	tmp_val_u8 |= reg_data->paos_reg.ee ? (1 <<  REG_EE_BIT_N) : 0;
	SX_PUT(inbox, tmp_val_u8, REG_ASE_OFFSET);
	SX_PUT(inbox, reg_data->paos_reg.e, REG_E_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PAOS_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->paos_reg.swid, outbox,
				REG_SWID_OFFSET);
		SX_GET(reg_data->paos_reg.local_port, outbox,
				REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->paos_reg.admin_status, outbox,
			   REG_ADMIN_STATUS_OFFSET);
		SX_GET(reg_data->paos_reg.oper_status, outbox,
				REG_OPER_STATUS_OFFSET);
		SX_GET(tmp_val_u8, outbox, REG_ASE_OFFSET);
		reg_data->paos_reg.ase =
				tmp_val_u8 & (1 <<  REG_ASE_BIT_N) ? 1 : 0;
		reg_data->paos_reg.ee =
				tmp_val_u8 & (1 <<  REG_EE_BIT_N) ? 1 : 0;
		SX_GET(reg_data->paos_reg.e, outbox, REG_E_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PAOS);

int sx_ACCESS_REG_PPLM(struct sx_dev *dev, struct ku_access_pplm_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u32 retransmission_active_and_fec_mode_active;

#define REG_LOCAL_PORT_OFFSET				0x15
#define REG_PORT_PROFILE_MODE_OFFSET		0x1C
#define REG_STATIC_PORT_PROFILE_OFFSET		0x1D
#define REG_ACTIVE_PORT_PROFILE_OFFSET		0x1E
#define REG_RETRANSMISSION_ACTIVE_OFFSET	0x20
#define PPLM_REG_LEN						0x06

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PPLM_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->pplm_reg.local_port, REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->pplm_reg.port_profile_mode, REG_PORT_PROFILE_MODE_OFFSET);
	SX_PUT(inbox, reg_data->pplm_reg.static_port_profile,REG_STATIC_PORT_PROFILE_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PPLM_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->pplm_reg.local_port, outbox,
				REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->pplm_reg.port_profile_mode, outbox,
				REG_PORT_PROFILE_MODE_OFFSET);
		SX_GET(reg_data->pplm_reg.static_port_profile, outbox,
				REG_STATIC_PORT_PROFILE_OFFSET);
		SX_GET(reg_data->pplm_reg.active_port_profile, outbox,
				REG_ACTIVE_PORT_PROFILE_OFFSET);

		SX_GET(retransmission_active_and_fec_mode_active, outbox,
						REG_RETRANSMISSION_ACTIVE_OFFSET);
		reg_data->pplm_reg.retransmission_active =
				(retransmission_active_and_fec_mode_active >> 24) & 0xFF;
		reg_data->pplm_reg.fec_mode_active =
						retransmission_active_and_fec_mode_active & 0xFFFFFF;

	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PPLM);

int sx_ACCESS_REG_PLPC(struct sx_dev *dev, struct ku_access_plpc_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_PROFILE_ID_OFFSET						0x14
#define REG_PLPC_PROTO_MASK_OFFSET					0x16
#define REG_LANE_SPEED_OFFSET						0x1A
#define REG_LPBF_OFFSET								0x1E
#define REG_FEC_MODE_POLICY_OFFSET					0x1F
#define REG_RETRANSMISSION_CAPABILITY_OFFSET		0x20
#define REG_FEC_MODE_CAPABILITY_OFFSET				0x21
#define REG_RETRANSMISSION_SUPPORT_ADMIN_OFFSET		0x24
#define REG_FEC_MODE_SUPPORT_ADMIN_OFFSET			0x25
#define REG_RETRANSMISSION_REQUEST_ADMIN_OFFSET		0x28
#define REG_FEC_MODE_REQUEST_ADMIN_OFFSET			0x29
#define PLPC_REG_LEN								0xB

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PLPC_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->plpc_reg.profile_id, REG_PROFILE_ID_OFFSET);
	SX_PUT(inbox, reg_data->plpc_reg.proto_mask, REG_PLPC_PROTO_MASK_OFFSET);
	SX_PUT(inbox, reg_data->plpc_reg.lane_speed,REG_LANE_SPEED_OFFSET);
	SX_PUT(inbox, reg_data->plpc_reg.lpbf,REG_LPBF_OFFSET);
	SX_PUT(inbox, reg_data->plpc_reg.fec_mode_policy,REG_FEC_MODE_POLICY_OFFSET);
	SX_PUT(inbox, reg_data->plpc_reg.retransmission_capability,REG_RETRANSMISSION_CAPABILITY_OFFSET);
	SX_PUT(inbox, reg_data->plpc_reg.fec_mode_capability,REG_FEC_MODE_CAPABILITY_OFFSET);
	SX_PUT(inbox, reg_data->plpc_reg.retransmission_support_admin,REG_RETRANSMISSION_SUPPORT_ADMIN_OFFSET);
	SX_PUT(inbox, reg_data->plpc_reg.fec_mode_support_admin,REG_FEC_MODE_SUPPORT_ADMIN_OFFSET);
	SX_PUT(inbox, reg_data->plpc_reg.retransmission_request_admin,REG_RETRANSMISSION_REQUEST_ADMIN_OFFSET);
	SX_PUT(inbox, reg_data->plpc_reg.fec_mode_request_admin,REG_FEC_MODE_REQUEST_ADMIN_OFFSET);




	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PLPC_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->plpc_reg.profile_id, outbox,
				REG_PROFILE_ID_OFFSET);
		SX_GET(reg_data->plpc_reg.proto_mask, outbox,
				REG_PROTO_MASK_OFFSET);
		SX_GET(reg_data->plpc_reg.lane_speed, outbox,
				REG_LANE_SPEED_OFFSET);
		SX_GET(reg_data->plpc_reg.lpbf, outbox,
				REG_LPBF_OFFSET);
		SX_GET(reg_data->plpc_reg.fec_mode_policy, outbox,
				REG_FEC_MODE_POLICY_OFFSET);
		SX_GET(reg_data->plpc_reg.retransmission_capability, outbox,
				REG_RETRANSMISSION_CAPABILITY_OFFSET);
		SX_GET(reg_data->plpc_reg.fec_mode_capability, outbox,
				REG_FEC_MODE_CAPABILITY_OFFSET);
		SX_GET(reg_data->plpc_reg.retransmission_support_admin, outbox,
				REG_RETRANSMISSION_SUPPORT_ADMIN_OFFSET);
		SX_GET(reg_data->plpc_reg.fec_mode_support_admin, outbox,
				REG_FEC_MODE_SUPPORT_ADMIN_OFFSET);
		SX_GET(reg_data->plpc_reg.retransmission_request_admin, outbox,
				REG_RETRANSMISSION_REQUEST_ADMIN_OFFSET);
		SX_GET(reg_data->plpc_reg.fec_mode_request_admin, outbox,
				REG_FEC_MODE_REQUEST_ADMIN_OFFSET);

	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PLPC);

int sx_ACCESS_REG_PMPC(struct sx_dev *dev, struct ku_access_pmpc_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u32 i = 0;

#define REG_MODULE_STATE_UPDATED_OFFSET			0x14
#define PMPC_REG_LEN			0x09

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PMPC_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	for(i = 0 ; i < 8; i++){
		SX_PUT(inbox, reg_data->pmpc_reg.module_state_updated_bitmap[i], REG_MODULE_STATE_UPDATED_OFFSET+(i*4));
	}

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PMPC_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		for(i = 0 ; i < 8; i++){
			SX_GET(reg_data->pmpc_reg.module_state_updated_bitmap[i], outbox,
					REG_MODULE_STATE_UPDATED_OFFSET + (i*4));
		}
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_PMPC);

int sx_ACCESS_REG_PMPR(struct sx_dev *dev, struct ku_access_pmpr_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define PMPR_REG_MODULE_OFFSET		0x15
#define PMPR_REG_ATTENUATION_5G_OFFSET		0x1B
#define PMPR_REG_ATTENUATION_7G_OFFSET      0x1F
#define PMPR_REG_ATTENUATION_12G_OFFSET     0x23
#define PMPR_REG_LEN			0x05

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PMPR_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->pmpr_reg.module, PMPR_REG_MODULE_OFFSET);
	SX_PUT(inbox, reg_data->pmpr_reg.attenuation5g,	PMPR_REG_ATTENUATION_5G_OFFSET);
	SX_PUT(inbox, reg_data->pmpr_reg.attenuation7g, PMPR_REG_ATTENUATION_7G_OFFSET);
	SX_PUT(inbox, reg_data->pmpr_reg.attenuation12g, PMPR_REG_ATTENUATION_12G_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PMPR_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->pmpr_reg.module, outbox,
				PMPR_REG_MODULE_OFFSET);
		SX_GET(reg_data->pmpr_reg.attenuation5g, outbox,
			   PMPR_REG_ATTENUATION_5G_OFFSET);
		SX_GET(reg_data->pmpr_reg.attenuation7g, outbox,
		       PMPR_REG_ATTENUATION_7G_OFFSET);
		SX_GET(reg_data->pmpr_reg.attenuation12g, outbox,
		       PMPR_REG_ATTENUATION_12G_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PMPR);

int sx_ACCESS_REG_PMAOS(struct sx_dev *dev,
		struct ku_access_pmaos_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u8 tmp_val_u8;

#define REG_MODULE_OFFSET		0x15
#define PMAOS_REG_LEN			0x05

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PMAOS_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->pmaos_reg.module, REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->pmaos_reg.admin_status,
			REG_ADMIN_STATUS_OFFSET);
	tmp_val_u8 = 0;
	tmp_val_u8 |= reg_data->pmaos_reg.ase ? (1 <<  REG_ASE_BIT_N) : 0;
	tmp_val_u8 |= reg_data->pmaos_reg.ee ? (1 <<  REG_EE_BIT_N) : 0;
	SX_PUT(inbox, tmp_val_u8, REG_ASE_OFFSET);
	SX_PUT(inbox, reg_data->pmaos_reg.e, REG_E_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PMAOS_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->pmaos_reg.module, outbox,
				REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->pmaos_reg.admin_status, outbox,
			   REG_ADMIN_STATUS_OFFSET);
		SX_GET(reg_data->pmaos_reg.oper_status, outbox,
				REG_OPER_STATUS_OFFSET);
		SX_GET(tmp_val_u8, outbox, REG_ASE_OFFSET);
		reg_data->pmaos_reg.ase =
				tmp_val_u8 & (1 <<  REG_ASE_BIT_N) ? 1 : 0;
		reg_data->pmaos_reg.ee =
				tmp_val_u8 & (1 <<  REG_EE_BIT_N) ? 1 : 0;
		SX_GET(reg_data->pmaos_reg.e, outbox, REG_E_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PMAOS);

int sx_ACCESS_REG_PMTU(struct sx_dev *dev, struct ku_access_pmtu_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_LOCAL_PORT_OFFSET	0x15
#define REG_MAX_MTU_OFFSET	0x18
#define REG_ADMIN_MTU_OFFSET	0x1c
#define REG_OPER_MTU_OFFSET	0x20
#define PMTU_REG_LEN		0x05

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PMTU_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->pmtu_reg.local_port, REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->pmtu_reg.max_mtu, REG_MAX_MTU_OFFSET);
	SX_PUT(inbox, reg_data->pmtu_reg.admin_mtu, REG_ADMIN_MTU_OFFSET);
	SX_PUT(inbox, reg_data->pmtu_reg.oper_mtu, REG_OPER_MTU_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PMTU_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->pmtu_reg.local_port, outbox,
				REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->pmtu_reg.max_mtu, outbox,
				REG_MAX_MTU_OFFSET);
		SX_GET(reg_data->pmtu_reg.admin_mtu, outbox,
				REG_ADMIN_MTU_OFFSET);
		SX_GET(reg_data->pmtu_reg.oper_mtu, outbox,
				REG_OPER_MTU_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PMTU);

int sx_ACCESS_REG_PELC(struct sx_dev *dev, struct ku_access_pelc_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_PELC_OP_OFFSET   0x14
#define REG_PELC_LOCAL_PORT_OFFSET   0x15

// 0x16, 0x17 reserved not in use

#define REG_PELC_OP_ADMIN_OFFSET    0x18
#define REG_PELC_OP_CAPABILITY_OFFSET   0x19
#define REG_PELC_OP_REQUEST_OFFSET  0x1a
#define REG_PELC_OP_ACTIVE_OFFSET  0x1b

#define REG_PELC_ADMIN_OFFSET    0x1c
#define REG_PELC_CAPABILITY_OFFSET   0x24
#define REG_PELC_REQUEST_OFFSET  0x2c
#define REG_PELC_ACTIVE_OFFSET	0x34
#define PELC_REG_LEN    0x0b

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PELC_REG_LEN;

	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
    reg_data->pelc_reg.op = reg_data->pelc_reg.op << 4;
	SX_PUT(inbox, reg_data->pelc_reg.op, REG_PELC_OP_OFFSET);
    SX_PUT(inbox, reg_data->pelc_reg.local_port, REG_PELC_LOCAL_PORT_OFFSET);
    SX_PUT(inbox, reg_data->pelc_reg.op_admin, REG_PELC_OP_ADMIN_OFFSET);
    SX_PUT(inbox, reg_data->pelc_reg.op_capability, REG_PELC_OP_CAPABILITY_OFFSET);
    SX_PUT(inbox, reg_data->pelc_reg.op_request, REG_PELC_OP_REQUEST_OFFSET);
    SX_PUT(inbox, reg_data->pelc_reg.op_active, REG_PELC_OP_ACTIVE_OFFSET);
    SX_PUT(inbox, reg_data->pelc_reg.admin, REG_PELC_ADMIN_OFFSET);
	SX_PUT(inbox, reg_data->pelc_reg.capability, REG_PELC_CAPABILITY_OFFSET);
    SX_PUT(inbox, reg_data->pelc_reg.request, REG_PELC_REQUEST_OFFSET);
	SX_PUT(inbox, reg_data->pelc_reg.active, REG_PELC_ACTIVE_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PELC_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->pelc_reg.op, outbox,
				REG_PELC_OP_OFFSET);
        reg_data->pelc_reg.op = reg_data->pelc_reg.op >> 4;
        SX_GET(reg_data->pelc_reg.local_port, outbox,
				REG_PELC_LOCAL_PORT_OFFSET);
        SX_GET(reg_data->pelc_reg.op_admin, outbox,
                REG_PELC_OP_ADMIN_OFFSET);
        SX_GET(reg_data->pelc_reg.op_capability, outbox,
                REG_PELC_OP_CAPABILITY_OFFSET);
        SX_GET(reg_data->pelc_reg.op_request, outbox,
                REG_PELC_OP_REQUEST_OFFSET);
        SX_GET(reg_data->pelc_reg.op_active, outbox,
                REG_PELC_OP_ACTIVE_OFFSET);
        SX_GET(reg_data->pelc_reg.admin, outbox,
				REG_PELC_ADMIN_OFFSET);
		SX_GET(reg_data->pelc_reg.capability, outbox,
				REG_PELC_CAPABILITY_OFFSET);
		SX_GET(reg_data->pelc_reg.request, outbox,
				REG_PELC_REQUEST_OFFSET);
        SX_GET(reg_data->pelc_reg.active, outbox,
				REG_PELC_ACTIVE_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PELC);

int sx_ACCESS_REG_PVLC(struct sx_dev *dev, struct ku_access_pvlc_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_LOCAL_PORT_OFFSET		0x15
#define REG_VL_CAP_OFFSET		0x1b
#define REG_VL_ADMIN_OFFSET		0x1f
#define REG_VL_OPERATIONAL_OFFSET	0x23
#define PVLC_REG_LEN			0x05

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PVLC_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->pvlc_reg.local_port, REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->pvlc_reg.vl_cap, REG_VL_CAP_OFFSET);
	SX_PUT(inbox, reg_data->pvlc_reg.vl_admin, REG_VL_ADMIN_OFFSET);
	SX_PUT(inbox, reg_data->pvlc_reg.vl_operational,
			REG_VL_OPERATIONAL_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PVLC_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->pvlc_reg.local_port, outbox,
				REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->pvlc_reg.vl_cap, outbox,
				REG_VL_CAP_OFFSET);
		SX_GET(reg_data->pvlc_reg.vl_admin, outbox,
				REG_VL_ADMIN_OFFSET);
		SX_GET(reg_data->pvlc_reg.vl_operational, outbox,
				REG_VL_OPERATIONAL_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PVLC);

int sx_ACCESS_REG_MCIA(struct sx_dev *dev, struct ku_access_mcia_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_L_OFFSET			0x14
#define REG_MODULE_OFFSET		0x15
#define REG_STATUS_OFFSET		0x17
#define REG_I2C_DEVICE_ADDRESS_OFFSET	0x18
#define REG_PAGE_NUMBER_OFFSET		0x19
#define REG_DEVICE_ADDRESS_OFFSET	0x1a
#define REG_SIZE_OFFSET			0x1e
#define REG_DWORD_0_OFFSET		0x24
#define REG_DWORD_1_OFFSET		0x28
#define REG_DWORD_2_OFFSET		0x2c
#define REG_DWORD_3_OFFSET		0x30
#define REG_DWORD_4_OFFSET		0x34
#define REG_DWORD_5_OFFSET		0x38
#define REG_DWORD_6_OFFSET		0x3c
#define REG_DWORD_7_OFFSET		0x40
#define REG_DWORD_8_OFFSET		0x44
#define REG_DWORD_9_OFFSET		0x48
#define REG_DWORD_10_OFFSET		0x4c
#define REG_DWORD_11_OFFSET		0x50
#define MCIA_REG_LEN			0x11

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MCIA_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.l, REG_L_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.module, REG_MODULE_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.i2c_device_address,
			REG_I2C_DEVICE_ADDRESS_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.page_number, REG_PAGE_NUMBER_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.device_address,
			REG_DEVICE_ADDRESS_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.size,
			REG_SIZE_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.dword_0,
			REG_DWORD_0_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.dword_1,
			REG_DWORD_1_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.dword_2,
			REG_DWORD_2_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.dword_3,
			REG_DWORD_3_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.dword_4,
			REG_DWORD_4_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.dword_5,
			REG_DWORD_5_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.dword_6,
			REG_DWORD_6_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.dword_7,
			REG_DWORD_7_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.dword_8,
			REG_DWORD_8_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.dword_9,
			REG_DWORD_9_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.dword_10,
			REG_DWORD_10_OFFSET);
	SX_PUT(inbox, reg_data->mcia_reg.dword_11,
			REG_DWORD_11_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MCIA_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mcia_reg.l, outbox,
				REG_L_OFFSET);
		SX_GET(reg_data->mcia_reg.module, outbox,
				REG_MODULE_OFFSET);
		SX_GET(reg_data->mcia_reg.status, outbox,
				REG_STATUS_OFFSET);
		SX_GET(reg_data->mcia_reg.i2c_device_address, outbox,
				REG_I2C_DEVICE_ADDRESS_OFFSET);
		SX_GET(reg_data->mcia_reg.page_number, outbox,
				REG_PAGE_NUMBER_OFFSET);
		SX_GET(reg_data->mcia_reg.device_address, outbox,
				REG_DEVICE_ADDRESS_OFFSET);
		SX_GET(reg_data->mcia_reg.size, outbox, REG_SIZE_OFFSET);
		SX_GET(reg_data->mcia_reg.dword_0, outbox,
				REG_DWORD_0_OFFSET);
		SX_GET(reg_data->mcia_reg.dword_1, outbox,
				REG_DWORD_1_OFFSET);
		SX_GET(reg_data->mcia_reg.dword_2, outbox,
				REG_DWORD_2_OFFSET);
		SX_GET(reg_data->mcia_reg.dword_3, outbox,
				REG_DWORD_3_OFFSET);
		SX_GET(reg_data->mcia_reg.dword_4, outbox,
				REG_DWORD_4_OFFSET);
		SX_GET(reg_data->mcia_reg.dword_5, outbox,
				REG_DWORD_5_OFFSET);
		SX_GET(reg_data->mcia_reg.dword_6, outbox,
				REG_DWORD_6_OFFSET);
		SX_GET(reg_data->mcia_reg.dword_7, outbox,
				REG_DWORD_7_OFFSET);
		SX_GET(reg_data->mcia_reg.dword_8, outbox,
				REG_DWORD_8_OFFSET);
		SX_GET(reg_data->mcia_reg.dword_9, outbox,
				REG_DWORD_9_OFFSET);
		SX_GET(reg_data->mcia_reg.dword_10, outbox,
				REG_DWORD_10_OFFSET);
		SX_GET(reg_data->mcia_reg.dword_11, outbox,
				REG_DWORD_11_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_MCIA);

int sx_ACCESS_REG_HPKT(struct sx_dev *dev, struct ku_access_hpkt_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u32 trap_info = 0;

#define REG_ACK_BITN			24
#define REG_ACTION_BITN			20
#define REG_TRAP_GROUP_BITN		12
#define REG_TRAP_INFO_OFFSET	0x14
#define REG_HPKT_CTRL_OFFSET    0x19
#define HPKT_REG_LEN			0x05

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= HPKT_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);

	trap_info = ((u32)(reg_data->hpkt_reg.ack) & 0x1) << REG_ACK_BITN;
	trap_info |= ((u32)(reg_data->hpkt_reg.action) & 0x7 )<< REG_ACTION_BITN;
	trap_info |= ((u32)(reg_data->hpkt_reg.trap_group) & 0x3f ) << REG_TRAP_GROUP_BITN;
	trap_info |= (u32)(reg_data->hpkt_reg.trap_id) & 0x1FF;
	SX_PUT(inbox, trap_info, REG_TRAP_INFO_OFFSET);
	SX_PUT(inbox, reg_data->hpkt_reg.control, REG_HPKT_CTRL_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(HPKT_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(trap_info, outbox, REG_TRAP_INFO_OFFSET);
		reg_data->hpkt_reg.ack = (trap_info >> REG_ACK_BITN) & 0x1;
		reg_data->hpkt_reg.action = (trap_info >> REG_ACTION_BITN) & 0x7;
		reg_data->hpkt_reg.trap_group = (trap_info >> REG_TRAP_GROUP_BITN) & 0x3F;
		reg_data->hpkt_reg.trap_id = trap_info & 0x1FF;
		SX_GET(reg_data->hpkt_reg.control, outbox, REG_HPKT_CTRL_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_HPKT);

int sx_ACCESS_REG_HCAP(struct sx_dev *dev, struct ku_access_hcap_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_MAX_CPU_EGRESS_TCS_OFFSET		0x1b
#define REG_MAX_CPU_INGRESS_TCS_OFFSET		0x1f
#define REG_MAX_TRAP_GROUPS_OFFSET			0x23
#define REG_DR_PATHS_OFFSET					0x27
#define HCAP_REG_LEN						0x09

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= HCAP_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->hcap_reg.max_cpu_egress_tclass,
			REG_MAX_CPU_EGRESS_TCS_OFFSET);
	SX_PUT(inbox, reg_data->hcap_reg.max_cpu_ingress_tclass,
			REG_MAX_CPU_INGRESS_TCS_OFFSET);
	SX_PUT(inbox, reg_data->hcap_reg.max_num_trap_groups,
			REG_MAX_TRAP_GROUPS_OFFSET);
	SX_PUT(inbox, reg_data->hcap_reg.max_num_dr_paths,
			REG_DR_PATHS_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(HCAP_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->hcap_reg.max_cpu_egress_tclass, outbox,
				REG_MAX_CPU_EGRESS_TCS_OFFSET);
		SX_GET(reg_data->hcap_reg.max_cpu_ingress_tclass, outbox,
				REG_MAX_CPU_INGRESS_TCS_OFFSET);
		SX_GET(reg_data->hcap_reg.max_num_trap_groups, outbox,
				REG_MAX_TRAP_GROUPS_OFFSET);
		SX_GET(reg_data->hcap_reg.max_num_dr_paths, outbox,
				REG_DR_PATHS_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_HCAP);

int sx_ACCESS_REG_MFSC(struct sx_dev *dev, struct ku_access_mfsc_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_FAN_OFFSET	0x14
#define REG_PWM_OFFSET	0x1b
#define MFSC_REG_LEN	0x3

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MFSC_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mfsc_reg.pwm, REG_FAN_OFFSET);
	SX_PUT(inbox, reg_data->mfsc_reg.pwm_duty_cycle, REG_PWM_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MFSC_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mfsc_reg.pwm, outbox,
				REG_FAN_OFFSET);
		SX_GET(reg_data->mfsc_reg.pwm_duty_cycle, outbox,
				REG_PWM_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MFSC);

int sx_ACCESS_REG_MFSM(struct sx_dev *dev, struct ku_access_mfsm_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_TACHO_OFFSET	0x14
#define REG_N_OFFSET		0x17
#define REG_RPM_OFFSET		0x1a
#define MFSM_REG_LEN		0x3

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MFSM_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mfsm_reg.tacho, REG_TACHO_OFFSET);
	SX_PUT(inbox, reg_data->mfsm_reg.n, REG_N_OFFSET);
	SX_PUT(inbox, reg_data->mfsm_reg.rpm, REG_RPM_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MFSM_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mfsm_reg.tacho, outbox,
				REG_TACHO_OFFSET);
		SX_GET(reg_data->mfsm_reg.n, outbox, REG_N_OFFSET);
		SX_GET(reg_data->mfsm_reg.rpm, outbox, REG_RPM_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MFSM);

int sx_ACCESS_REG_MFSL(struct sx_dev *dev, struct ku_access_mfsl_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u8 flags;

#define REG_FAN_OFFSET		0x14
#define REG_EE_SHIFT		2
#define REG_FLAGS_OFFSET	0x17
#define REG_TACH_MIN_OFFSET	0x1a
#define REG_TACH_MAX_OFFSET	0x1e
#define MFSL_REG_LEN		0x3

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MFSL_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mfsl_reg.fan, REG_FAN_OFFSET);
	flags = (reg_data->mfsl_reg.ee << REG_EE_SHIFT);
	flags |= reg_data->mfsl_reg.ie;
	SX_PUT(inbox, flags, REG_FLAGS_OFFSET);
	SX_PUT(inbox, reg_data->mfsl_reg.tach_min, REG_TACH_MIN_OFFSET);
	SX_PUT(inbox, reg_data->mfsl_reg.tach_max, REG_TACH_MAX_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MFSL_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mfsl_reg.fan, outbox, REG_FAN_OFFSET);
		SX_GET(flags, outbox, REG_FLAGS_OFFSET);
		reg_data->mfsl_reg.ee = (flags >> REG_EE_SHIFT) & 0x3;
		reg_data->mfsl_reg.ie = flags & 0x1;
		SX_GET(reg_data->mfsl_reg.tach_min, outbox,
				REG_TACH_MIN_OFFSET);
		SX_GET(reg_data->mfsl_reg.tach_max, outbox,
				REG_TACH_MAX_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MFSL);

int sx_ACCESS_REG_HDRT(struct sx_dev *dev, struct ku_access_hdrt_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	int i;
	u16 type_len = 0;

#define REG_DR_INDEX_OFFSET	0x14
#define REG_HOP_CNT_OFFSET	0x16
#define REG_FIRST_PATH_OFFSET	0x18
#define REG_NUM_OF_PATHS	0x40
#define REG_FIRST_RPATH_OFFSET	(REG_FIRST_PATH_OFFSET + REG_NUM_OF_PATHS)
#define HDRT_REG_LEN		0x17

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= HDRT_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->hdrt_reg.dr_index, REG_DR_INDEX_OFFSET);
	SX_PUT(inbox, reg_data->hdrt_reg.hop_cnt, REG_HOP_CNT_OFFSET);
	for (i = 0; i < REG_NUM_OF_PATHS; i++)
		SX_PUT(inbox, reg_data->hdrt_reg.path[i],
				REG_FIRST_PATH_OFFSET + i);

	for (i = 0; i < REG_NUM_OF_PATHS; i++)
		SX_PUT(inbox, reg_data->hdrt_reg.rpath[i],
				REG_FIRST_RPATH_OFFSET + i);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(HDRT_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->hdrt_reg.dr_index, outbox,
				REG_DR_INDEX_OFFSET);
		SX_GET(reg_data->hdrt_reg.hop_cnt, outbox, REG_HOP_CNT_OFFSET);
		for (i = 0; i < REG_NUM_OF_PATHS; i++)
			SX_GET(reg_data->hdrt_reg.path[i], outbox,
					REG_FIRST_PATH_OFFSET + i);

		for (i = 0; i < REG_NUM_OF_PATHS; i++)
			SX_GET(reg_data->hdrt_reg.rpath[i], outbox,
					REG_FIRST_RPATH_OFFSET + i);

	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_HDRT);

int sx_ACCESS_REG_MTMP(struct sx_dev *dev, struct ku_access_mtmp_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u8 flags = 0;

#define REG_SENSOR_INDEX_OFFSET	0x17
#define REG_TEMP_OFFSET		0x1a
#define REG_MTE_SHIFT		0x7
#define REG_MTR_SHIFT		0x6
#define REG_MTE_MTR_OFFSET	0x1c
#define REG_MAX_TEMP_OFFSET	0x1e
#define REG_TEE_OFFSET		0x20
#define REG_TEE_SHIFT		0x6
#define REG_TEMP_THRSHLD_OFFSET	0x22
#define MTMP_REG_LEN		0x6

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MTMP_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mtmp_reg.sensor_index,
			REG_SENSOR_INDEX_OFFSET);
	SX_PUT(inbox, reg_data->mtmp_reg.temperature, REG_TEMP_OFFSET);
	flags |= reg_data->mtmp_reg.mte << REG_MTE_SHIFT;
	flags |= reg_data->mtmp_reg.mtr << REG_MTR_SHIFT;
	SX_PUT(inbox, flags, REG_MTE_MTR_OFFSET);
	SX_PUT(inbox, reg_data->mtmp_reg.max_temperature, REG_MAX_TEMP_OFFSET);
	flags = reg_data->mtmp_reg.tee << REG_TEE_SHIFT;
	SX_PUT(inbox, flags, REG_TEE_OFFSET);
	SX_PUT(inbox, reg_data->mtmp_reg.temperature_threshold,
			REG_TEMP_THRSHLD_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MTMP_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mtmp_reg.sensor_index, outbox,
				REG_SENSOR_INDEX_OFFSET);
		SX_GET(reg_data->mtmp_reg.temperature, outbox,
				REG_TEMP_OFFSET);
		SX_GET(flags, outbox, REG_MTE_MTR_OFFSET);
		reg_data->mtmp_reg.mte = (flags >> REG_MTE_SHIFT) & 0x1;
		reg_data->mtmp_reg.mtr = (flags >> REG_MTR_SHIFT) & 0x1;
		SX_GET(reg_data->mtmp_reg.max_temperature, outbox,
				REG_MAX_TEMP_OFFSET);
		SX_GET(flags, outbox, REG_TEE_OFFSET);
		reg_data->mtmp_reg.tee = (flags >> REG_TEE_SHIFT) & 0x3;
		SX_GET(reg_data->mtmp_reg.temperature_threshold, outbox,
				REG_TEMP_THRSHLD_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MTMP);

int sx_ACCESS_REG_MMDIO(struct sx_dev *dev,
		struct ku_access_mmdio_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_MDIO_INDEX_OFFSET	0x15
#define REG_OPERATION_OFFSET	0x17
#define REG_ADDRESS_OFFSET	0x18
#define REG_DATA_OFFSET		0x1c
#define MMDIO_REG_LEN		0x4

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MMDIO_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mmdio_reg.mdio_index, REG_MDIO_INDEX_OFFSET);
	SX_PUT(inbox, reg_data->mmdio_reg.operation, REG_OPERATION_OFFSET);
	SX_PUT(inbox, reg_data->mmdio_reg.address, REG_ADDRESS_OFFSET);
	SX_PUT(inbox, reg_data->mmdio_reg.data, REG_DATA_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MMDIO_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mmdio_reg.mdio_index, outbox,
				REG_MDIO_INDEX_OFFSET);
		SX_GET(reg_data->mmdio_reg.operation, outbox,
				REG_OPERATION_OFFSET);
		SX_GET(reg_data->mmdio_reg.address, outbox, REG_ADDRESS_OFFSET);
		SX_GET(reg_data->mmdio_reg.data, outbox, REG_DATA_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MMDIO);

int sx_ACCESS_REG_MMIA(struct sx_dev *dev, struct ku_access_mmia_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_OPERATION_OFFSET	0x17
#define REG_DATA_OFFSET		0x1c
#define MMIA_REG_LEN		0x4

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MMIA_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mmia_reg.operation, REG_OPERATION_OFFSET);
	SX_PUT(inbox, reg_data->mmia_reg.data, REG_DATA_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MMIA_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mmia_reg.operation, outbox,
				REG_OPERATION_OFFSET);
		SX_GET(reg_data->mmia_reg.data, outbox, REG_DATA_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MMIA);

int sx_ACCESS_REG_MFPA(struct sx_dev *dev, struct ku_access_mfpa_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u8 flag;

#define REG_P_OFFSET			0x16
#define REG_FS_OFFSET			0x17
#define REG_FS_BITS_SHIFT		4
#define REG_BOOT_ADDRESS_OFFSET		0x18
#define REG_FLASH_NUM_OFFSET		0x27
#define REG_JEDEC_ID_OFFSET		0x28
#define REG_BLOCK_ALLIGNMENT_OFFSET	0x2d
#define REG_SECTOR_SIZE_OFFSET		0x2f
#define REG_CAPABILITY_MASK_OFFSET	0x30
#define REG_CAPABILITY_MASK_BIT_N	7
#define MFPA_REG_LEN			0x40

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MFPA_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mfpa_reg.p, REG_P_OFFSET);
	flag = (u8)(reg_data->mfpa_reg.fs << REG_FS_BITS_SHIFT);
	SX_PUT(inbox, flag, REG_FS_OFFSET);
	SX_PUT(inbox, reg_data->mfpa_reg.boot_address, REG_BOOT_ADDRESS_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MFPA_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mfpa_reg.p, outbox, REG_P_OFFSET);
		SX_GET(flag, outbox, REG_FS_OFFSET);
		reg_data->mfpa_reg.fs = (u8)(flag >> REG_FS_BITS_SHIFT);
		SX_GET(reg_data->mfpa_reg.boot_address, outbox,
				REG_BOOT_ADDRESS_OFFSET);
		SX_GET(reg_data->mfpa_reg.flash_num, outbox,
				REG_FLASH_NUM_OFFSET);
		SX_GET(reg_data->mfpa_reg.jedec_id, outbox,
				REG_JEDEC_ID_OFFSET);
		SX_GET(reg_data->mfpa_reg.block_allignment, outbox,
				REG_BLOCK_ALLIGNMENT_OFFSET);
		SX_GET(reg_data->mfpa_reg.sector_size, outbox,
				REG_SECTOR_SIZE_OFFSET);
		SX_GET(flag, outbox, REG_CAPABILITY_MASK_OFFSET);
		reg_data->mfpa_reg.capability_mask =
				(u8)(flag >> REG_CAPABILITY_MASK_BIT_N);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MFPA);

int sx_ACCESS_REG_MFBE(struct sx_dev *dev, struct ku_access_mfbe_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u8 flag;

#define REG_P_OFFSET		0x16
#define REG_FS_OFFSET		0x17
#define REG_FS_BITS_SHIFT	4
#define REG_MFBE_ADDRESS_OFFSET	0x1c
#define MFBE_REG_LEN		0x4

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MFBE_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mfbe_reg.p, REG_P_OFFSET);
	flag = (u8)(reg_data->mfbe_reg.fs << REG_FS_BITS_SHIFT);
	SX_PUT(inbox, flag, REG_FS_OFFSET);
	SX_PUT(inbox, reg_data->mfbe_reg.address, REG_MFBE_ADDRESS_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MFBE_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mfbe_reg.p, outbox, REG_P_OFFSET);
		SX_GET(flag, outbox, REG_FS_OFFSET);
		reg_data->mfbe_reg.fs = (u8)(flag >> REG_FS_BITS_SHIFT);
		SX_GET(reg_data->mfbe_reg.address, outbox,
				REG_MFBE_ADDRESS_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MFBE);

int sx_ACCESS_REG_MFBA(struct sx_dev *dev, struct ku_access_mfba_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u8 flag;
	int i;

#define REG_P_OFFSET		0x16
#define REG_FS_OFFSET		0x17
#define REG_FS_BITS_SHIFT	4
#define REG_MFBA_SIZE_OFFSET	0x1a
#define REG_MFBA_ADDRESS_OFFSET	    0x1c
#define REG_MFBA_DATA_OFFSET	    0x20
#define MFBA_REG_LEN		0x40

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MFBA_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mfba_reg.p, REG_P_OFFSET);
	flag = (u8)(reg_data->mfba_reg.fs << REG_FS_BITS_SHIFT);
	SX_PUT(inbox, flag, REG_FS_OFFSET);
	SX_PUT(inbox, reg_data->mfba_reg.size, REG_MFBA_SIZE_OFFSET);
	SX_PUT(inbox, reg_data->mfba_reg.address, REG_MFBA_ADDRESS_OFFSET);
	for (i = 0; i < 192; i++)
		SX_PUT(inbox, reg_data->mfba_reg.data[i],
				REG_MFBA_DATA_OFFSET + i);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MFBA_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mfba_reg.p, outbox, REG_P_OFFSET);
		SX_GET(flag, outbox, REG_FS_OFFSET);
		reg_data->mfba_reg.fs = (u8)(flag >> REG_FS_BITS_SHIFT);
		SX_GET(reg_data->mfba_reg.size, outbox, REG_MFBA_SIZE_OFFSET);
		SX_GET(reg_data->mfba_reg.address, outbox,
				REG_MFBA_ADDRESS_OFFSET);
		for (i = 0; i < 192; i++)
			SX_GET(reg_data->mfba_reg.data[i], outbox,
					REG_MFBA_DATA_OFFSET + i);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MFBA);

int sx_ACCESS_REG_QCAP(struct sx_dev *dev, struct ku_access_qcap_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_MAX_POLICER_PER_PORT_OFFSET		0x1f
#define REG_MAX_POLICER_GLOBAL_OFFSET		0x23
#define QCAP_REG_LEN				0x09

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= QCAP_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->qcap_reg.max_policers_per_port,
			REG_MAX_POLICER_PER_PORT_OFFSET);
	SX_PUT(inbox, reg_data->qcap_reg.max_policers_global,
			REG_MAX_POLICER_GLOBAL_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(QCAP_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->qcap_reg.max_policers_per_port, outbox,
				REG_MAX_POLICER_PER_PORT_OFFSET);
		SX_GET(reg_data->qcap_reg.max_policers_global, outbox,
				REG_MAX_POLICER_GLOBAL_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_QCAP);

int sx_ACCESS_REG_RAW(struct sx_dev *dev, struct ku_access_raw_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define OP_TLV_SIZE	0x10

	if (SX_MAILBOX_SIZE < OP_TLV_SIZE + reg_data->raw_reg.size) {
		sx_warn(dev, "Cannot send the raw register access request "
				"since the mailbox size is too small\n");
		return -ENOMEM;
	}

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= (reg_data->raw_reg.size / 4) + 1;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	err = copy_from_user(((u8 *)inbox) + OP_TLV_SIZE + 4,
			reg_data->raw_reg.buff, reg_data->raw_reg.size);
	if (err)
		goto out;

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			20 + reg_data->raw_reg.size);
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	err = copy_to_user(reg_data->raw_reg.buff,
			((u8 *)outbox) + OP_TLV_SIZE + 4,
			reg_data->raw_reg.size);

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}

int sx_ACCESS_REG_RAW_BUFF(struct sx_dev *dev,
		struct ku_access_reg_raw_buff *raw_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;

	if (SX_MAILBOX_SIZE < raw_data->raw_buff.size) {
		sx_warn(dev, "Cannot send the raw register access request "
				"since the mailbox size is too small\n");
		return -ENOMEM;
	}

	in_mailbox = sx_alloc_cmd_mailbox(dev, raw_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, raw_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	err = copy_from_user(((u8 *)inbox), raw_data->raw_buff.buff,
			raw_data->raw_buff.size);
	if (err)
		goto out;

	err = sx_cmd_box(dev, raw_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			raw_data->raw_buff.size);
	if (err)
		goto out;

	err = copy_to_user(raw_data->raw_buff.buff, outbox,
			raw_data->raw_buff.size);

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}

int sx_ACCESS_REG_MTWE(struct sx_dev *dev, struct ku_access_mtwe_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_SENSOR_WARNING_OFFSET	0x17
#define MTWE_REG_LEN			0x3

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MTWE_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mtwe_reg.sensor_warning,
			REG_SENSOR_WARNING_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MTWE_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mtwe_reg.sensor_warning, outbox,
				REG_SENSOR_WARNING_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MTWE);

int sx_ACCESS_REG_MTCAP(struct sx_dev *dev,
		struct ku_access_mtcap_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_SENSOR_COUNT_OFFSET	0x17
#define MTCAP_REG_LEN		0x3

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MTCAP_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mtcap_reg.sensor_count,
			REG_SENSOR_COUNT_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MTCAP_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mtcap_reg.sensor_count, outbox,
				REG_SENSOR_COUNT_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MTCAP);

int sx_ACCESS_REG_FORE(struct sx_dev *dev, struct ku_access_fore_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_FAN_UNDER_LIMIT_OFFSET	0x14
#define REG_FAN_OVER_LIMIT_OFFSET	0x18
#define FORE_REG_LEN			0x4

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= FORE_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->fore_reg.fan_under_limit,
			REG_FAN_UNDER_LIMIT_OFFSET);
	SX_PUT(inbox, reg_data->fore_reg.fan_over_limit,
			REG_FAN_OVER_LIMIT_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(FORE_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->fore_reg.fan_under_limit, outbox,
				REG_FAN_UNDER_LIMIT_OFFSET);
		SX_GET(reg_data->fore_reg.fan_over_limit, outbox,
				REG_FAN_OVER_LIMIT_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_FORE);

int sx_ACCESS_REG_MFCR(struct sx_dev *dev, struct ku_access_mfcr_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_PWM_FREQ_OFFSET	0x17
#define REG_TACHO_ACTIVE_OFFSET	0x18
#define REG_PWM_ACTIVE_OFFSET	0x1b
#define MFCR_REG_LEN		0x3

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MFCR_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mfcr_reg.pwm_frequency, REG_PWM_FREQ_OFFSET);
	SX_PUT(inbox, reg_data->mfcr_reg.tacho_active,
			REG_TACHO_ACTIVE_OFFSET);
	SX_PUT(inbox, reg_data->mfcr_reg.pwm_active, REG_PWM_ACTIVE_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MFCR_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mfcr_reg.pwm_frequency, outbox,
				REG_PWM_FREQ_OFFSET);
		SX_GET(reg_data->mfcr_reg.tacho_active, outbox,
				REG_TACHO_ACTIVE_OFFSET);
		SX_GET(reg_data->mfcr_reg.pwm_active, outbox,
				REG_PWM_ACTIVE_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_MFCR);

int sx_ACCESS_REG_MFM(struct sx_dev *dev, struct ku_access_mfm_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_MFM_INDEX_OFFSET	0x17
#define REG_MEMORY_OFFSET	0x1c
#define REG_MEMORY_MASK_OFFSET	0x24
#define MFM_REG_LEN		0x7

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MFM_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mfm_reg.index, REG_MFM_INDEX_OFFSET);
	SX_PUT(inbox, reg_data->mfm_reg.memory, REG_MEMORY_OFFSET);
	SX_PUT(inbox, reg_data->mfm_reg.memory_mask, REG_MEMORY_MASK_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MFM_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mfm_reg.index, outbox,
				REG_MFM_INDEX_OFFSET);
		SX_GET(reg_data->mfm_reg.memory, outbox,
				REG_MEMORY_OFFSET);
		SX_GET(reg_data->mfm_reg.memory_mask, outbox,
				REG_MEMORY_MASK_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_MFM);

int sx_ACCESS_REG_QPRT(struct sx_dev *dev, struct ku_access_qprt_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_LOCAL_PORT_OFFSET	0x15
#define REG_PRIO_OFFSET		0x16
#define REG_RPRIO_OFFSET	0x1b
#define QPRT_REG_LEN		0x3

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= QPRT_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->qprt_reg.local_port, REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->qprt_reg.prio, REG_PRIO_OFFSET);
	SX_PUT(inbox, reg_data->qprt_reg.rprio, REG_RPRIO_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(QPRT_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->qprt_reg.local_port, outbox,
				REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->qprt_reg.prio, outbox, REG_PRIO_OFFSET);
		SX_GET(reg_data->qprt_reg.rprio, outbox, REG_RPRIO_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_QPRT);

int sx_ACCESS_REG_HTGT(struct sx_dev *dev, struct ku_access_htgt_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u16 mac_47_32;
	u32 mac_31_0;
	u8 tmp;

#define REG_HTGT_SWID_OFFSET		0x14
#define REG_HTGT_TYPE_OFFSET		0x16
#define REG_HTGT_GRP_OFFSET			0x17
#define REG_HTGT_PIDE_OFFSET		0x1A
#define REG_HTGT_PID_OFFSET			0x1B
#define REG_HTGT_MRR_ACTION_OFFSET  0x1E
#define REG_HTGT_MRR_AGENT_OFFSET   0x1F
#define REG_HTGT_PRIO_OFFSET        0x23
#define REG_HTGT_PATH_OFFSET		0x24
#define REG_HTGT_LP_CPU_TC_OFFSET	(REG_HTGT_PATH_OFFSET + 1)
#define REG_HTGT_LP_RDQ_OFFSET		(REG_HTGT_PATH_OFFSET + 3)
#define REG_HTGT_SP_STK_TC_OFFSET	(REG_HTGT_PATH_OFFSET + 0)
#define REG_HTGT_SP_CPU_TC_OFFSET	(REG_HTGT_PATH_OFFSET + 1)
#define REG_HTGT_SP_RDQ_OFFSET		(REG_HTGT_PATH_OFFSET + 3)
#define REG_HTGT_SP_SYS_PORT_OFFSET	(REG_HTGT_PATH_OFFSET + 7)
#define REG_HTGT_DRP_DR_PTR_OFFSET	(REG_HTGT_PATH_OFFSET + 3)
#define REG_HTGT_EP_MAC_47_32_OFFSET	(REG_HTGT_PATH_OFFSET + 3)
#define REG_HTGT_EP_MAC_31_0_OFFSET	(REG_HTGT_PATH_OFFSET + 7)
#define REG_HTGT_EP_VID_OFFSET		(REG_HTGT_PATH_OFFSET + 0xb)
#define HTGT_REG_LEN				0x41

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= HTGT_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->htgt_reg.swid, REG_HTGT_SWID_OFFSET);
	SX_PUT(inbox, reg_data->htgt_reg.type, REG_HTGT_TYPE_OFFSET);
	SX_PUT(inbox, reg_data->htgt_reg.trap_group, REG_HTGT_GRP_OFFSET);
	tmp = (reg_data->htgt_reg.pide & 0x1) << 7;
	SX_PUT(inbox, tmp, REG_HTGT_PIDE_OFFSET);
	SX_PUT(inbox, reg_data->htgt_reg.pid, REG_HTGT_PID_OFFSET);
	SX_PUT(inbox, reg_data->htgt_reg.mirror_action, REG_HTGT_MRR_ACTION_OFFSET);
	SX_PUT(inbox, reg_data->htgt_reg.mirror_agent, REG_HTGT_MRR_AGENT_OFFSET);
	SX_PUT(inbox, reg_data->htgt_reg.priority, REG_HTGT_PRIO_OFFSET);

	switch (reg_data->htgt_reg.type) {
	case HTGT_LOCAL_PATH:
		SX_PUT(inbox, reg_data->htgt_reg.path.local_path.cpu_tclass,
				REG_HTGT_LP_CPU_TC_OFFSET);
		SX_PUT(inbox, reg_data->htgt_reg.path.local_path.rdq,
				REG_HTGT_LP_RDQ_OFFSET);
		break;

	case HTGT_STACKING_PATH:
		SX_PUT(inbox, reg_data->
				htgt_reg.path.stacking_path.stacking_tclass,
			   REG_HTGT_SP_STK_TC_OFFSET);
		SX_PUT(inbox, reg_data->htgt_reg.path.stacking_path.cpu_tclass,
			   REG_HTGT_SP_CPU_TC_OFFSET);
		SX_PUT(inbox, reg_data->htgt_reg.path.stacking_path.rdq,
			   REG_HTGT_SP_RDQ_OFFSET);
		SX_PUT(inbox,
			reg_data->htgt_reg.path.stacking_path.cpu_sys_port,
			   REG_HTGT_SP_SYS_PORT_OFFSET);
		break;

	case HTGT_DR_PATH:
		SX_PUT(inbox, reg_data->htgt_reg.path.dr_path.dr_ptr,
				REG_HTGT_DRP_DR_PTR_OFFSET);
		break;

	case HTGT_ETH_PATH:
		mac_47_32 = (reg_data->htgt_reg.path.eth_path.mac >> 32) &
				0xFFFF;
		mac_31_0 = reg_data->htgt_reg.path.eth_path.mac & 0xFFFFFFFF;
		SX_PUT(inbox, mac_47_32, REG_HTGT_EP_MAC_47_32_OFFSET);
		SX_PUT(inbox, mac_31_0, REG_HTGT_EP_MAC_31_0_OFFSET);
		SX_PUT(inbox, reg_data->htgt_reg.path.eth_path.vid,
				REG_HTGT_EP_VID_OFFSET);
		break;

	default:
		printk(KERN_ERR "%s(): Incorrect HTGT path type: %d \n",
					__func__, reg_data->htgt_reg.type);
		return -EINVAL;
	}

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(HTGT_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->htgt_reg.swid, outbox,
				REG_HTGT_SWID_OFFSET);
		SX_GET(reg_data->htgt_reg.type, outbox,
				REG_HTGT_TYPE_OFFSET);
		SX_GET(reg_data->htgt_reg.trap_group, outbox,
				REG_HTGT_GRP_OFFSET);
		SX_GET(tmp, outbox, REG_HTGT_PIDE_OFFSET);
		reg_data->htgt_reg.pide = (tmp >> 0x7) & 0x1;
		SX_GET(reg_data->htgt_reg.pid, outbox,
				REG_HTGT_PID_OFFSET);
		SX_GET(reg_data->htgt_reg.mirror_action, outbox,
		       REG_HTGT_MRR_ACTION_OFFSET);
		SX_GET(reg_data->htgt_reg.mirror_agent, outbox,
		       REG_HTGT_MRR_AGENT_OFFSET);
		SX_GET(reg_data->htgt_reg.priority, outbox,
		       REG_HTGT_PRIO_OFFSET);

		switch (reg_data->htgt_reg.type) {
		case HTGT_LOCAL_PATH:
			SX_GET(reg_data->htgt_reg.path.local_path.cpu_tclass,
					outbox, REG_HTGT_LP_CPU_TC_OFFSET);
			SX_GET(reg_data->htgt_reg.path.local_path.rdq, outbox,
				REG_HTGT_LP_RDQ_OFFSET);
			break;

		case HTGT_STACKING_PATH:
			SX_GET(reg_data->
				htgt_reg.path.stacking_path.stacking_tclass,
				    outbox, REG_HTGT_SP_STK_TC_OFFSET);
			SX_GET(reg_data->
				htgt_reg.path.stacking_path.cpu_tclass,
				    outbox, REG_HTGT_SP_CPU_TC_OFFSET);
			SX_GET(reg_data->htgt_reg.path.stacking_path.rdq,
				    outbox, REG_HTGT_SP_RDQ_OFFSET);
			SX_GET(reg_data->
				htgt_reg.path.stacking_path.cpu_sys_port,
				    outbox, REG_HTGT_SP_SYS_PORT_OFFSET);
			break;

		case HTGT_DR_PATH:
			SX_GET(reg_data->htgt_reg.path.dr_path.dr_ptr,
				    outbox, REG_HTGT_DRP_DR_PTR_OFFSET);
			break;

		case HTGT_ETH_PATH:
			SX_GET(mac_47_32, outbox,
					REG_HTGT_EP_MAC_47_32_OFFSET);
			SX_GET(mac_31_0, outbox, REG_HTGT_EP_MAC_31_0_OFFSET);
			reg_data->htgt_reg.path.eth_path.mac = mac_47_32;
			reg_data->htgt_reg.path.eth_path.mac =
				(reg_data->htgt_reg.path.eth_path.mac << 32) |
				mac_31_0;
			SX_GET(reg_data->htgt_reg.path.eth_path.vid, outbox,
				   REG_HTGT_EP_VID_OFFSET);
			break;

		default:
			printk(KERN_ERR "%s(): Incorrect HTGT path type: "
					"%d on query\n", __func__,
					reg_data->htgt_reg.type);
			return -EINVAL;
		}
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_HTGT);

int sx_ACCESS_REG_SPAD(struct sx_dev *dev, struct ku_access_spad_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define REG_BASE_MAC_OFFSET	0x14
#define SPAD_REG_LEN		0x05

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= SPAD_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->spad_reg.base_mac, REG_BASE_MAC_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(SPAD_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->spad_reg.base_mac, outbox,
				REG_BASE_MAC_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_SPAD);

#define PROFILE_SET_MASK_H_OFFSET		0x00
#define PROFILE_SET_MASK_L_OFFSET		0x08
#define PROFILE_MAX_VEPA_OFFSET			0x13
#define PROFILE_MAX_LAG_OFFSET			0x16
#define PROFILE_MAX_PORT_OFFSET			0x1a
#define PROFILE_MAX_MID_OFFSET			0x1e
#define PROFILE_MAX_PGT_OFFSET			0x22
#define PROFILE_MAX_SYSPORT_OFFSET		0x26
#define PROFILE_MAX_VLANS_OFFSET		0x2a
#define PROFILE_MAX_REGIONS_OFFSET		0x2e
#define PROFILE_MAX_FLOOD_TABLES_OFFSET		0x31
#define PROFILE_MAX_PER_VID_FLOOD_TABLES_OFFSET	0x32
#define PROFILE_FLOOD_MODE_OFFSET		0x33
#define PROFILE_MAX_FID_OFFSET_FLOOD_TABLES_OFFSET 0x34
#define PROFILE_FID_OFFSET_TABLE_SIZE_OFFSET	   0x36
#define PROFILE_MAX_PER_FID_FLOOD_TABLE_OFFSET	 0x38
#define PROFILE_PER_FID_TABLE_SIZE_OFFSET		  0x3A
#define PROFILE_MAX_IB_MC_OFFSET		0x42
#define PROFILE_MAX_PKEY_OFFSET			0x46
#define PROFILE_AR_SEC_OFFSET			0x4c
#define PROFILE_AR_GRP_CAP_OFFSET		0x4e
#define PROFILE_ARN_OFFSET			0x50
#define PROFILE_ARN_BIT_N			7
#define PROFILE_KVD_LINEAR_SIZE_OFFSET  0x54
#define PROFILE_KVD_HASH_SINGLE_SIZE_OFFSET   0x58
#define PROFILE_KVD_HASH_DOUBLE_SIZE_OFFSET   0x5C
#define PROFILE_SWID_0_CONF_OFFSET		0x60
#define PROFILE_SWID_1_CONF_OFFSET		0x68
#define PROFILE_SWID_2_CONF_OFFSET		0x70
#define PROFILE_SWID_3_CONF_OFFSET		0x78
#define PROFILE_SWID_4_CONF_OFFSET		0x80
#define PROFILE_SWID_5_CONF_OFFSET		0x88
#define PROFILE_SWID_6_CONF_OFFSET		0x90
#define PROFILE_SWID_7_CONF_OFFSET		0x98
#define PROFILE_RESERVED1_OFFSET		0xb4
#define CONFIG_PROFILE_MB_SIZE			0xb8
int sx_GET_PROFILE(struct sx_dev *dev, struct ku_profile *profile)
{
	struct sx_cmd_mailbox *mailbox;
	u32 *outbox;
	int err;
	u8 tmp;

	memset(profile, 0, sizeof(*profile));
	mailbox = sx_alloc_cmd_mailbox(dev, profile->dev_id);
	if (IS_ERR(mailbox))
		return PTR_ERR(mailbox);

	outbox = mailbox->buf;
	err = sx_cmd_box(dev, dev->device_id, 0, mailbox, 0, 0x2,
			SX_CMD_CONFIG_PROFILE, SX_CMD_TIME_CLASS_A,
			CONFIG_PROFILE_MB_SIZE);
	if (err)
		goto out;

	SX_GET(profile->max_vepa_channels, outbox, PROFILE_MAX_VEPA_OFFSET);
	SX_GET(profile->max_lag, outbox, PROFILE_MAX_LAG_OFFSET);
	SX_GET(profile->max_port_per_lag, outbox, PROFILE_MAX_PORT_OFFSET);
	SX_GET(profile->max_mid, outbox, PROFILE_MAX_MID_OFFSET);
	SX_GET(profile->max_pgt, outbox, PROFILE_MAX_PGT_OFFSET);
	SX_GET(profile->max_system_port, outbox, PROFILE_MAX_SYSPORT_OFFSET);
	SX_GET(profile->max_active_vlans, outbox, PROFILE_MAX_VLANS_OFFSET);
	SX_GET(profile->max_regions, outbox, PROFILE_MAX_REGIONS_OFFSET);
	SX_GET(profile->max_flood_tables, outbox, PROFILE_MAX_FLOOD_TABLES_OFFSET);
	SX_GET(profile->max_per_vid_flood_tables, outbox,
			PROFILE_MAX_PER_VID_FLOOD_TABLES_OFFSET);
	SX_GET(profile->flood_mode, outbox, PROFILE_FLOOD_MODE_OFFSET);
	SX_GET(profile->max_fid_offset_flood_tables, outbox, PROFILE_MAX_FID_OFFSET_FLOOD_TABLES_OFFSET);
	SX_GET(profile->fid_offset_table_size, outbox, PROFILE_FID_OFFSET_TABLE_SIZE_OFFSET);
	SX_GET(profile->max_per_fid_flood_table, outbox, PROFILE_MAX_PER_FID_FLOOD_TABLE_OFFSET);
	SX_GET(profile->per_fid_table_size, outbox, PROFILE_PER_FID_TABLE_SIZE_OFFSET);
	SX_GET(profile->max_ib_mc, outbox, PROFILE_MAX_IB_MC_OFFSET);
	SX_GET(profile->max_pkey, outbox, PROFILE_MAX_PKEY_OFFSET);
	SX_GET(profile->ar_sec, outbox, PROFILE_AR_SEC_OFFSET);
	SX_GET(profile->adaptive_routing_group_cap, outbox, PROFILE_AR_GRP_CAP_OFFSET);
	SX_GET(profile->arn, outbox, PROFILE_ARN_OFFSET);
	profile->arn = profile->arn >> PROFILE_ARN_BIT_N;
	SX_GET(profile->kvd_linear_size, outbox, PROFILE_KVD_LINEAR_SIZE_OFFSET);
	SX_GET(profile->kvd_hash_single_size, outbox, PROFILE_KVD_HASH_SINGLE_SIZE_OFFSET);
	SX_GET(profile->kvd_hash_double_size, outbox, PROFILE_KVD_HASH_DOUBLE_SIZE_OFFSET);
	SX_GET(tmp, outbox, PROFILE_SWID_0_CONF_OFFSET + 1);
	profile->swid0_config_type.type = tmp >> 4;
	SX_GET(tmp, outbox, PROFILE_SWID_1_CONF_OFFSET + 1);
	profile->swid1_config_type.type = tmp >> 4;
	SX_GET(tmp, outbox, PROFILE_SWID_2_CONF_OFFSET + 1);
	profile->swid2_config_type.type = tmp >> 4;
	SX_GET(tmp, outbox, PROFILE_SWID_3_CONF_OFFSET + 1);
	profile->swid3_config_type.type =  tmp >> 4;
	SX_GET(tmp, outbox, PROFILE_SWID_4_CONF_OFFSET + 1);
	profile->swid4_config_type.type = tmp >> 4;
	SX_GET(tmp, outbox, PROFILE_SWID_5_CONF_OFFSET + 1);
	profile->swid5_config_type.type = tmp >> 4;
	SX_GET(tmp, outbox, PROFILE_SWID_6_CONF_OFFSET + 1);
	profile->swid6_config_type.type = tmp >> 4;
	SX_GET(tmp, outbox, PROFILE_SWID_7_CONF_OFFSET + 1);
	profile->swid7_config_type.type = tmp >> 4;
	SX_GET(profile->swid0_config_type.properties, outbox,
			PROFILE_SWID_0_CONF_OFFSET + 3);
	SX_GET(profile->swid1_config_type.properties, outbox,
			PROFILE_SWID_1_CONF_OFFSET + 3);
	SX_GET(profile->swid2_config_type.properties, outbox,
			PROFILE_SWID_2_CONF_OFFSET + 3);
	SX_GET(profile->swid3_config_type.properties, outbox,
			PROFILE_SWID_3_CONF_OFFSET + 3);
	SX_GET(profile->swid4_config_type.properties, outbox,
			PROFILE_SWID_4_CONF_OFFSET + 3);
	SX_GET(profile->swid5_config_type.properties, outbox,
			PROFILE_SWID_5_CONF_OFFSET + 3);
	SX_GET(profile->swid6_config_type.properties, outbox,
			PROFILE_SWID_6_CONF_OFFSET + 3);
	SX_GET(profile->swid7_config_type.properties, outbox,
			PROFILE_SWID_7_CONF_OFFSET + 3);
	SX_GET(profile->reserved1, outbox, PROFILE_RESERVED1_OFFSET);

out:
	sx_free_cmd_mailbox(dev, mailbox);
	return err;
}
EXPORT_SYMBOL(sx_GET_PROFILE);

int sx_SET_PROFILE(struct sx_dev *dev, struct ku_profile *profile)
{
	struct sx_cmd_mailbox *mailbox;
	u32 *inbox;
	int err;
	u8 temp_u8 = 0;

	mailbox = sx_alloc_cmd_mailbox(dev, profile->dev_id);
	if (IS_ERR(mailbox))
		return PTR_ERR(mailbox);

	inbox = mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	SX_PUT(inbox, profile->set_mask_0_63, PROFILE_SET_MASK_L_OFFSET);
	SX_PUT(inbox, profile->set_mask_64_127, PROFILE_SET_MASK_H_OFFSET);
	SX_PUT(inbox, profile->max_vepa_channels, PROFILE_MAX_VEPA_OFFSET);
	SX_PUT(inbox, profile->max_lag, PROFILE_MAX_LAG_OFFSET);
	SX_PUT(inbox, profile->max_port_per_lag, PROFILE_MAX_PORT_OFFSET);
	SX_PUT(inbox, profile->max_mid, PROFILE_MAX_MID_OFFSET);
	SX_PUT(inbox, profile->max_pgt, PROFILE_MAX_PGT_OFFSET);
	SX_PUT(inbox, profile->max_system_port,
			PROFILE_MAX_SYSPORT_OFFSET);
	SX_PUT(inbox, profile->max_active_vlans, PROFILE_MAX_VLANS_OFFSET);
	SX_PUT(inbox, profile->max_regions, PROFILE_MAX_REGIONS_OFFSET);
	SX_PUT(inbox, profile->max_flood_tables, PROFILE_MAX_FLOOD_TABLES_OFFSET);
	SX_PUT(inbox, profile->max_per_vid_flood_tables,
			PROFILE_MAX_PER_VID_FLOOD_TABLES_OFFSET);
	SX_PUT(inbox, profile->flood_mode, PROFILE_FLOOD_MODE_OFFSET);
	SX_PUT(inbox, profile->max_fid_offset_flood_tables, PROFILE_MAX_FID_OFFSET_FLOOD_TABLES_OFFSET);
	SX_PUT(inbox, profile->fid_offset_table_size, PROFILE_FID_OFFSET_TABLE_SIZE_OFFSET);
	SX_PUT(inbox, profile->max_per_fid_flood_table, PROFILE_MAX_PER_FID_FLOOD_TABLE_OFFSET);
	SX_PUT(inbox, profile->per_fid_table_size, PROFILE_PER_FID_TABLE_SIZE_OFFSET);
	SX_PUT(inbox, profile->max_ib_mc, PROFILE_MAX_IB_MC_OFFSET);
	SX_PUT(inbox, profile->max_pkey, PROFILE_MAX_PKEY_OFFSET);
	SX_PUT(inbox, profile->ar_sec, PROFILE_AR_SEC_OFFSET);
	SX_PUT(inbox, profile->adaptive_routing_group_cap, PROFILE_AR_GRP_CAP_OFFSET);
	SX_PUT(inbox, (u8)(profile->arn << PROFILE_ARN_BIT_N), PROFILE_ARN_OFFSET);
	SX_PUT(inbox, profile->kvd_linear_size, PROFILE_KVD_LINEAR_SIZE_OFFSET);
	SX_PUT(inbox, profile->kvd_hash_single_size, PROFILE_KVD_HASH_SINGLE_SIZE_OFFSET);
	SX_PUT(inbox, profile->kvd_hash_double_size, PROFILE_KVD_HASH_DOUBLE_SIZE_OFFSET);
	SX_PUT(inbox, profile->swid0_config_type.mask,
			PROFILE_SWID_0_CONF_OFFSET);
	SX_PUT(inbox, profile->swid0_config_type.properties,
			PROFILE_SWID_0_CONF_OFFSET + 3);
	temp_u8 = (u8)(profile->swid0_config_type.type << 4);
	SX_PUT(inbox, temp_u8, PROFILE_SWID_0_CONF_OFFSET + 1);
	SX_PUT(inbox, profile->swid1_config_type.mask,
			PROFILE_SWID_1_CONF_OFFSET);
	SX_PUT(inbox, profile->swid1_config_type.properties,
			PROFILE_SWID_1_CONF_OFFSET + 3);
	temp_u8 = (u8)(profile->swid1_config_type.type << 4);
	SX_PUT(inbox, temp_u8, PROFILE_SWID_1_CONF_OFFSET + 1);
	SX_PUT(inbox, profile->swid2_config_type.mask,
			PROFILE_SWID_2_CONF_OFFSET);
	SX_PUT(inbox, profile->swid2_config_type.properties,
			PROFILE_SWID_2_CONF_OFFSET + 3);
	temp_u8 = (u8)(profile->swid2_config_type.type << 4);
	SX_PUT(inbox, temp_u8, PROFILE_SWID_2_CONF_OFFSET + 1);
	SX_PUT(inbox, profile->swid3_config_type.mask,
			PROFILE_SWID_3_CONF_OFFSET);
	SX_PUT(inbox, profile->swid3_config_type.properties,
			PROFILE_SWID_3_CONF_OFFSET + 3);
	temp_u8 = (u8)(profile->swid3_config_type.type << 4);
	SX_PUT(inbox, temp_u8, PROFILE_SWID_3_CONF_OFFSET + 1);
	SX_PUT(inbox, profile->swid4_config_type.mask,
			PROFILE_SWID_4_CONF_OFFSET);
	SX_PUT(inbox, profile->swid4_config_type.properties,
			PROFILE_SWID_4_CONF_OFFSET + 3);
	temp_u8 = (u8)(profile->swid4_config_type.type << 4);
	SX_PUT(inbox, temp_u8, PROFILE_SWID_4_CONF_OFFSET + 1);
	SX_PUT(inbox, profile->swid5_config_type.mask,
			PROFILE_SWID_5_CONF_OFFSET);
	SX_PUT(inbox, profile->swid5_config_type.properties,
			PROFILE_SWID_5_CONF_OFFSET + 3);
	temp_u8 = (u8)(profile->swid5_config_type.type << 4);
	SX_PUT(inbox, temp_u8, PROFILE_SWID_5_CONF_OFFSET + 1);
	SX_PUT(inbox, profile->swid6_config_type.mask,
			PROFILE_SWID_6_CONF_OFFSET);
	SX_PUT(inbox, profile->swid6_config_type.properties,
			PROFILE_SWID_6_CONF_OFFSET + 3);
	temp_u8 = (u8)(profile->swid6_config_type.type << 4);
	SX_PUT(inbox, temp_u8, PROFILE_SWID_6_CONF_OFFSET + 1);
	SX_PUT(inbox, profile->swid7_config_type.mask,
			PROFILE_SWID_7_CONF_OFFSET);
	SX_PUT(inbox, profile->swid7_config_type.properties,
			PROFILE_SWID_7_CONF_OFFSET + 3);
	temp_u8 = (u8)(profile->swid7_config_type.type << 4);
	SX_PUT(inbox, temp_u8, PROFILE_SWID_7_CONF_OFFSET + 1);
	SX_PUT(inbox, profile->reserved1, PROFILE_RESERVED1_OFFSET);

	err = sx_cmd(dev, profile->dev_id, mailbox, 0, 1, SX_CMD_CONFIG_PROFILE,
			SX_CMD_TIME_CLASS_A, CONFIG_PROFILE_MB_SIZE);

	sx_free_cmd_mailbox(dev, mailbox);
	return err;
}

int sx_ACCESS_REG_SSPR(struct sx_dev *dev, struct ku_access_sspr_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u8 tmp_val_u8 = 0;

#define SSPR_REG_MASTER_BIT_OFFSET	0x14
#define SSPR_REG_MASTER_BIT_N	7
#define SSPR_REG_LOCAL_PORT_OFFSET	0x15
#define SSPR_REG_SUB_PORT	0x16
#define SSPR_REG_SYSTEM_PORT	0x1a
#define SSPR_REG_LEN		0x03

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= SSPR_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);

	tmp_val_u8 = 0;
	tmp_val_u8 |= reg_data->sspr_reg.is_master ? (1 <<  SSPR_REG_MASTER_BIT_N) : 0;
	SX_PUT(inbox, tmp_val_u8, SSPR_REG_MASTER_BIT_OFFSET);
	SX_PUT(inbox, reg_data->sspr_reg.local_port, SSPR_REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->sspr_reg.sub_port, SSPR_REG_SUB_PORT);
	SX_PUT(inbox, reg_data->sspr_reg.system_port, SSPR_REG_SYSTEM_PORT);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(SSPR_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(tmp_val_u8, outbox,
			   SSPR_REG_MASTER_BIT_OFFSET);
		if (tmp_val_u8 & (1 <<  SSPR_REG_MASTER_BIT_N)) {
			reg_data->sspr_reg.is_master = 1;
		}
		SX_GET(reg_data->sspr_reg.local_port, outbox,
				SSPR_REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->sspr_reg.sub_port, outbox,
				SSPR_REG_SUB_PORT);
		SX_GET(reg_data->sspr_reg.system_port, outbox,
				SSPR_REG_SYSTEM_PORT);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_SSPR);

int sx_ACCESS_REG_PPAD(struct sx_dev *dev, struct ku_access_ppad_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u16 mac_47_32 = 0;
	u32	mac_31_8 = 0;

#define PPAD_REG_BASE_MAC_47_32	0x16
#define PPAD_REG_BASE_MAC_31_8	0x18
#define PPAD_REG_LEN		0x05

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PPAD_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);

	mac_47_32 = (reg_data->ppad_reg.mac[0] << 8) | reg_data->ppad_reg.mac[1];
	mac_31_8 = (reg_data->ppad_reg.mac[2] << 24) |
		   (reg_data->ppad_reg.mac[3] << 16) |
		   (reg_data->ppad_reg.mac[4] << 8); /* Last byte resevred (0) */
	SX_PUT(inbox, mac_47_32, PPAD_REG_BASE_MAC_47_32);
	SX_PUT(inbox, mac_47_32, PPAD_REG_BASE_MAC_31_8);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PPAD_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(mac_47_32, outbox,
				PPAD_REG_BASE_MAC_47_32);
		SX_GET(mac_31_8, outbox,
				PPAD_REG_BASE_MAC_31_8);
		reg_data->ppad_reg.mac[5] = (mac_31_8) & 0xff;
		reg_data->ppad_reg.mac[4] = (mac_31_8 >> 8) & 0xff;
		reg_data->ppad_reg.mac[3] = (mac_31_8 >> 16) & 0xff;
		reg_data->ppad_reg.mac[2] = (mac_31_8 >> 24)& 0xff;
		reg_data->ppad_reg.mac[1] = mac_47_32 & 0xff;
		reg_data->ppad_reg.mac[0] = (mac_47_32 >> 8) & 0xff;
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PPAD);


int sx_ACCESS_REG_SPMCR(struct sx_dev *dev, struct ku_access_spmcr_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define SPMCR_REG_SWID_OFFSET	0x14
#define SPMCR_REG_LOCAL_PORT_OFFSET	0x15
#define SPMCR_REG_MAX_SUB_PORT_OFFSET	0x16
#define SPMCR_REG_BASE_STAG_VID_OFFSET	0x1a
#define SPMCR_REG_LEN		0x03

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= SPMCR_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->spmcr_reg.swid, SPMCR_REG_SWID_OFFSET);
	SX_PUT(inbox, reg_data->spmcr_reg.local_port, SPMCR_REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->spmcr_reg.max_sub_port, SPMCR_REG_MAX_SUB_PORT_OFFSET);
	SX_PUT(inbox, reg_data->spmcr_reg.base_stag_vid, SPMCR_REG_BASE_STAG_VID_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(SPMCR_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->spmcr_reg.swid, outbox,
				SPMCR_REG_SWID_OFFSET);
		SX_GET(reg_data->spmcr_reg.local_port, outbox,
				SPMCR_REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->spmcr_reg.max_sub_port, outbox,
				SPMCR_REG_MAX_SUB_PORT_OFFSET);
		SX_GET(reg_data->spmcr_reg.swid, outbox,
				SPMCR_REG_BASE_STAG_VID_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_SPMCR);

int sx_ACCESS_REG_PBMC(struct sx_dev *dev, struct ku_access_pbmc_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err, i;
	u16 type_len = 0;

#define PBMC_REG_LOCAL_PORT_OFFSET	0x15
#define PBMC_REG_XOF_TIMER_VAL_OFFSET	0x19
#define PBMC_REG_XOF_REFRESH_OFFSET	0x1b
#define PBMC_REG_PORT_BUFF_SIZE_OFFSET	0x1e
#define PBMC_REG_BUFF_0_OFFSET	0x20
#define PBMC_REG_LEN		24

#define BUFF_SIZE_OFFSET	2
#define BUFF_XOFF_OFFSET	4
#define BUFF_XON_OFFSET		6


	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PBMC_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->pbmc_reg.local_port, PBMC_REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->pbmc_reg.xof_timer_value, PBMC_REG_XOF_TIMER_VAL_OFFSET);
	SX_PUT(inbox, reg_data->pbmc_reg.xof_refresh, PBMC_REG_XOF_REFRESH_OFFSET);
	for (i=0;i<10;i++) {
		int buff_offset = PBMC_REG_BUFF_0_OFFSET + 8*i;
		SX_PUT(inbox, reg_data->pbmc_reg.buffer[i].size, buff_offset + BUFF_SIZE_OFFSET);
		SX_PUT(inbox, reg_data->pbmc_reg.buffer[i].xof_threshold, buff_offset + BUFF_XOFF_OFFSET);
		SX_PUT(inbox, reg_data->pbmc_reg.buffer[i].xon_threshold, buff_offset + BUFF_XON_OFFSET);
	}
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PBMC_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->pbmc_reg.local_port, outbox,
				PBMC_REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->pbmc_reg.xof_timer_value, outbox,
				PBMC_REG_XOF_TIMER_VAL_OFFSET);
		SX_GET(reg_data->pbmc_reg.xof_refresh, outbox,
				PBMC_REG_XOF_REFRESH_OFFSET);
			for (i=0;i<10;i++) {
				int buff_offset = PBMC_REG_BUFF_0_OFFSET + 8*i;
				SX_GET(reg_data->pbmc_reg.buffer[i].size, outbox,buff_offset + BUFF_SIZE_OFFSET);
				SX_GET(reg_data->pbmc_reg.buffer[i].xof_threshold, outbox,buff_offset + BUFF_XOFF_OFFSET);
				SX_GET(reg_data->pbmc_reg.buffer[i].xon_threshold, outbox,buff_offset + BUFF_XON_OFFSET);
			}
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PBMC);

int sx_ACCESS_REG_PPTB(struct sx_dev *dev, struct ku_access_pptb_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u8 tmp_val_u8 = 0;

#define PPTB_REG_LOCAL_PORT_OFFSET	0x15
#define PPTB_REG_CM_UM_BITS_OFFSET	0x16
#define PPTB_REG_UM_BIT_N	0
#define PPTB_REG_CM_BIT_N	1
#define PPTB_REG_PM_OFFSET	0x17
#define PPTB_REG_PRIO_7_6_BUFF_OFFSET	0x18
#define PPTB_REG_PRIO_5_4_BUFF_OFFSET	0x19
#define PPTB_REG_PRIO_3_2_BUFF_OFFSET	0x1a
#define PPTB_REG_PRIO_1_0_BUFF_OFFSET	0x1b
#define PPTB_REG_CTRL_UNTAG_BUFF_OFFSET	0x1f
#define PPTB_REG_LEN		0x05

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PPTB_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->pptb_reg.local_port, PPTB_REG_LOCAL_PORT_OFFSET);
	tmp_val_u8 = (reg_data->pptb_reg.cm << PPTB_REG_CM_BIT_N) | reg_data->pptb_reg.um;
	SX_PUT(inbox, tmp_val_u8, PPTB_REG_CM_UM_BITS_OFFSET);
	SX_PUT(inbox, reg_data->pptb_reg.pm, PPTB_REG_PM_OFFSET);
	tmp_val_u8 = (reg_data->pptb_reg.prio_7_buff << 4) | (reg_data->pptb_reg.prio_6_buff & 0xf);
	SX_PUT(inbox, tmp_val_u8, PPTB_REG_PRIO_7_6_BUFF_OFFSET);
	tmp_val_u8 = (reg_data->pptb_reg.prio_5_buff << 4) | (reg_data->pptb_reg.prio_4_buff & 0xf);
	SX_PUT(inbox, tmp_val_u8, PPTB_REG_PRIO_5_4_BUFF_OFFSET);
	tmp_val_u8 = (reg_data->pptb_reg.prio_3_buff << 4) | (reg_data->pptb_reg.prio_2_buff & 0xf);
	SX_PUT(inbox, tmp_val_u8, PPTB_REG_PRIO_3_2_BUFF_OFFSET);
	tmp_val_u8 = (reg_data->pptb_reg.prio_1_buff << 4) | (reg_data->pptb_reg.prio_0_buff & 0xf);
	SX_PUT(inbox, tmp_val_u8, PPTB_REG_PRIO_1_0_BUFF_OFFSET);
	tmp_val_u8 = (reg_data->pptb_reg.ctrl_buff << 4) | (reg_data->pptb_reg.untagged_buff & 0xf);
	SX_PUT(inbox, tmp_val_u8, PPTB_REG_CTRL_UNTAG_BUFF_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PPTB_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->pptb_reg.local_port, outbox,
				PPTB_REG_LOCAL_PORT_OFFSET);
		SX_GET(tmp_val_u8, outbox,
				PPTB_REG_CM_UM_BITS_OFFSET);
		reg_data->pptb_reg.um = (tmp_val_u8 >> PPTB_REG_UM_BIT_N) & 1;
		reg_data->pptb_reg.cm =	(tmp_val_u8 >> PPTB_REG_CM_BIT_N) & 1;
		SX_GET(reg_data->pptb_reg.pm, outbox,
				PPTB_REG_PM_OFFSET);
		SX_GET(tmp_val_u8, outbox,
				PPTB_REG_PRIO_7_6_BUFF_OFFSET);
		reg_data->pptb_reg.prio_6_buff = tmp_val_u8 & 0xf;
		reg_data->pptb_reg.prio_7_buff = (tmp_val_u8 >> 4)& 0xf;
		SX_GET(tmp_val_u8, outbox,
				PPTB_REG_PRIO_5_4_BUFF_OFFSET);
		reg_data->pptb_reg.prio_4_buff = tmp_val_u8 & 0xf;
		reg_data->pptb_reg.prio_5_buff = (tmp_val_u8 >> 4)& 0xf;
		SX_GET(tmp_val_u8, outbox,
				PPTB_REG_PRIO_3_2_BUFF_OFFSET);
		reg_data->pptb_reg.prio_2_buff = tmp_val_u8 & 0xf;
		reg_data->pptb_reg.prio_3_buff = (tmp_val_u8 >> 4)& 0xf;
		SX_GET(tmp_val_u8, outbox,
				PPTB_REG_PRIO_1_0_BUFF_OFFSET);
		reg_data->pptb_reg.prio_0_buff = tmp_val_u8 & 0xf;
		reg_data->pptb_reg.prio_1_buff = (tmp_val_u8 >> 4)& 0xf;
		SX_GET(tmp_val_u8, outbox,
				PPTB_REG_CTRL_UNTAG_BUFF_OFFSET);
		reg_data->pptb_reg.untagged_buff = tmp_val_u8 & 0xf;
		reg_data->pptb_reg.ctrl_buff = (tmp_val_u8 >> 4)& 0xf;
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_PPTB);

int sx_ACCESS_REG_SMID(struct sx_dev *dev, struct ku_access_smid_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err,i,j;
	u16 type_len = 0;
	u32 tmp_val_u32;
	u32 tmp_mask_u32;

#define SMID_REG_SWID_OFFSET	0x14
#define SMID_REG_MID_OFFSET		0x16

#define SMID_REG_PORTS_255_224_OFFSET		0x34
#define SMID_REG_PORTS_223_192_OFFSET		0x38
#define SMID_REG_PORTS_191_160_OFFSET		0x3c
#define SMID_REG_PORTS_159_128_OFFSET		0x40
#define SMID_REG_PORTS_127_96_OFFSET		0x44
#define SMID_REG_PORTS_95_64_OFFSET			0x48
#define SMID_REG_PORTS_63_32_OFFSET			0x4c
#define SMID_REG_PORTS_31_0_OFFSET			 0x50

#define SMID_REG_PORTS_255_224_MASK_OFFSET		0x54
#define SMID_REG_PORTS_223_192_MASK_OFFSET		0x58
#define SMID_REG_PORTS_191_160_MASK_OFFSET		0x5c
#define SMID_REG_PORTS_159_128_MASK_OFFSET		0x60
#define SMID_REG_PORTS_127_96_MASK_OFFSET		0x64
#define SMID_REG_PORTS_95_64_MASK_OFFSET			0x68
#define SMID_REG_PORTS_63_32_MASK_OFFSET			0x6c
#define SMID_REG_PORTS_31_0_MASK_OFFSET			 0x70

#define SMID_REG_LEN		0x19

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= SMID_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->smid_reg.swid, SMID_REG_SWID_OFFSET);
	SX_PUT(inbox, reg_data->smid_reg.mid, SMID_REG_MID_OFFSET);

	for (j=0;j<256;) {
		tmp_val_u32 = 0;
		tmp_mask_u32 = 0;
		for (i=0;i<32;i++) {
			tmp_val_u32 |= (reg_data->smid_reg.ports_bitmap[j+i]? (1<<i):0);
			tmp_mask_u32 |= (reg_data->smid_reg.mask_bitmap[j+i]? (1<<i):0);
		}

		if (j==0) {
			SX_PUT(inbox, tmp_val_u32, SMID_REG_PORTS_31_0_OFFSET);
			SX_PUT(inbox, tmp_mask_u32, SMID_REG_PORTS_31_0_MASK_OFFSET);
		}
		else if (j==32) {
			SX_PUT(inbox, tmp_val_u32, SMID_REG_PORTS_63_32_OFFSET);
			SX_PUT(inbox, tmp_mask_u32, SMID_REG_PORTS_63_32_MASK_OFFSET);
		}
		else if (j==64) {
			SX_PUT(inbox, tmp_val_u32, SMID_REG_PORTS_95_64_OFFSET);
			SX_PUT(inbox, tmp_mask_u32, SMID_REG_PORTS_95_64_MASK_OFFSET);
		}
		else if (j==96) {
			SX_PUT(inbox, tmp_val_u32, SMID_REG_PORTS_127_96_OFFSET);
			SX_PUT(inbox, tmp_mask_u32, SMID_REG_PORTS_127_96_MASK_OFFSET);
		}
		else if (j==128) {
			SX_PUT(inbox, tmp_val_u32, SMID_REG_PORTS_159_128_OFFSET);
			SX_PUT(inbox, tmp_mask_u32, SMID_REG_PORTS_159_128_MASK_OFFSET);
		}
		else if (j==160) {
			SX_PUT(inbox, tmp_val_u32, SMID_REG_PORTS_191_160_OFFSET);
			SX_PUT(inbox, tmp_mask_u32, SMID_REG_PORTS_191_160_MASK_OFFSET);
		}
		else if (j==192) {
			SX_PUT(inbox, tmp_val_u32, SMID_REG_PORTS_223_192_OFFSET);
			SX_PUT(inbox, tmp_mask_u32, SMID_REG_PORTS_223_192_MASK_OFFSET);
		}
		else if (j==224) {
			SX_PUT(inbox, tmp_val_u32, SMID_REG_PORTS_255_224_OFFSET);
			SX_PUT(inbox, tmp_mask_u32, SMID_REG_PORTS_255_224_MASK_OFFSET);
		}

		j+=32;
	}

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(SMID_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->smid_reg.swid, outbox,
				SMID_REG_SWID_OFFSET);
		SX_GET(reg_data->smid_reg.mid, outbox,
				SMID_REG_MID_OFFSET);
		/*
		SX_GET(tmp_val_u32, outbox,SMID_REG_PORTS_255_224_OFFSET);
		SX_GET(tmp_val_u32, outbox,SMID_REG_PORTS_223_192_OFFSET);
		SX_GET(tmp_val_u32, outbox,SMID_REG_PORTS_191_160_OFFSET);
		SX_GET(tmp_val_u32, outbox,SMID_REG_PORTS_159_128_OFFSET);
		SX_GET(tmp_val_u32, outbox,SMID_REG_PORTS_127_96_OFFSET);
		SX_GET(tmp_val_u32, outbox,SMID_REG_PORTS_95_64_OFFSET);
		*/
		SX_GET(tmp_val_u32, outbox,SMID_REG_PORTS_63_32_OFFSET);
		for (i=0;i<32;i++) {
			if (tmp_val_u32 & (1<<i)) {
				reg_data->smid_reg.ports_bitmap[32 + i] = 1;
			}
		}
		SX_GET(tmp_val_u32, outbox,SMID_REG_PORTS_31_0_OFFSET);
		for (i=0;i<32;i++) {
			if (tmp_val_u32 & (1<<i)) {
				reg_data->smid_reg.ports_bitmap[0 + i] = 1;
			}
		}
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_SMID);

int sx_ACCESS_REG_SPMS(struct sx_dev *dev, struct ku_access_spms_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u32	tmp_val_u32;

	printk("%s() ENTER : dev_id: %d, 	tlv: op_class: %d, r: %d, reg_id: %d, method: %d, len: %d \n"
				"			spms_reg: lport: 0x%x, state[0: 0x%x,1: 0x%x,14: 0x%x,15: 0x%x]\n",
		   __func__,
		   reg_data->dev_id,
		   reg_data->op_tlv.op_class,
		   reg_data->op_tlv.r,
		   reg_data->op_tlv.register_id,
		   reg_data->op_tlv.method,
		   reg_data->op_tlv.length,

		   reg_data->spms_reg.local_port,
		   reg_data->spms_reg.state[0],
		   reg_data->spms_reg.state[1],
		   reg_data->spms_reg.state[14],
		   reg_data->spms_reg.state[15]
		   );

#define SPMS_REG_LOCAL_PORT_OFFSET	0x15
#define SPMS_REG_VLAN_63_48_STP_STATE_OFFSET	0x18
#define SPMS_REG_VLAN_47_32_STP_STATE_OFFSET	0x1c
#define SPMS_REG_VLAN_31_16_STP_STATE_OFFSET	0x20
#define SPMS_REG_VLAN_15_0_STP_STATE_OFFSET		0x24
#define SPMS_REG_LEN		0x06

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= SPMS_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->spms_reg.local_port, SPMS_REG_LOCAL_PORT_OFFSET);
		tmp_val_u32 = 0x0C;
		SX_PUT(inbox, tmp_val_u32, SPMS_REG_VLAN_15_0_STP_STATE_OFFSET);
		tmp_val_u32 = 0;
		SX_PUT(inbox, tmp_val_u32, SPMS_REG_VLAN_31_16_STP_STATE_OFFSET);
		tmp_val_u32 = 0;
		SX_PUT(inbox, tmp_val_u32, SPMS_REG_VLAN_47_32_STP_STATE_OFFSET);
		tmp_val_u32 = 0;
		SX_PUT(inbox, tmp_val_u32, SPMS_REG_VLAN_63_48_STP_STATE_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(SPMS_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->spms_reg.local_port, outbox,
				SPMS_REG_LOCAL_PORT_OFFSET);
		SX_GET(tmp_val_u32, outbox, SPMS_REG_VLAN_15_0_STP_STATE_OFFSET);
		if (tmp_val_u32 & 0xc) {
			reg_data->spms_reg.state[1]  = tmp_val_u32 & 0x3;
		}
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_SPMS);

int sx_ACCESS_REG_SPVID(struct sx_dev *dev, struct ku_access_spvid_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define SPVID_REG_LOCAL_PORT_OFFSET		0x15
#define SPVID_REG_SUB_PORT_OFFSET			0x16
#define SPVID_REG_PORT_VID_OFFSET			0x1a
#define SPVID_REG_LEN		0x03

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= SPVID_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->spvid_reg.local_port, SPVID_REG_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->spvid_reg.sub_port, SPVID_REG_SUB_PORT_OFFSET);
	SX_PUT(inbox, reg_data->spvid_reg.port_default_vid, SPVID_REG_PORT_VID_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(SPVID_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->spvid_reg.local_port, outbox,
				SPVID_REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->spvid_reg.sub_port, outbox,
				SPVID_REG_SUB_PORT_OFFSET);
		SX_GET(reg_data->spvid_reg.port_default_vid, outbox,
				SPVID_REG_PORT_VID_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_SPVID);

int sx_ACCESS_REG_SFGC(struct sx_dev *dev, struct ku_access_sfgc_reg *reg_data)
{
	return -EINVAL; /* The register had changed and not sure we need CMD IFC for it */
}
EXPORT_SYMBOL(sx_ACCESS_REG_SFGC);

int sx_ACCESS_REG_SFD(struct sx_dev *dev, struct ku_access_sfd_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err,i;
	u16 type_len = 0;
	u8	tmp_val_u8;
	u16	tmp_val_u16;
	u32	tmp_val_u32;
	u8 sfd_reg_len;


#define SFD_REG_SWID_OFFSET							  0x14
#define SFD_REG_OP_NEXT_LOCATOR_OFFSET	0x18
#define SFD_REG_NUM_RECORDS_OFFSET			0x1f
#define SFD_REG_FDB_RECORD_0_OFFSET			 0x24
#define SFD_REG_WITH_SINGLE_RECORD_LEN	  0x9

	sfd_reg_len = SFD_REG_WITH_SINGLE_RECORD_LEN +
		(reg_data->sfd_reg.num_records - 1)*4;

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= sfd_reg_len;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->sfd_reg.swid, SFD_REG_SWID_OFFSET);
	tmp_val_u32 = ((reg_data->sfd_reg.operation & 0x3) << 30) |
							 (reg_data->sfd_reg.record_locator & ~(0x3 << 30));
	SX_PUT(inbox, tmp_val_u32, SFD_REG_OP_NEXT_LOCATOR_OFFSET);
	SX_PUT(inbox, reg_data->sfd_reg.num_records, SFD_REG_NUM_RECORDS_OFFSET);

	for (i=0;i<reg_data->sfd_reg.num_records;i++) {
		switch (reg_data->sfd_reg.sfd_type[i]) {
		case SFD_TYPE_UNICAST:
			{
	#define SFD_REG_FDB_RECORD_UC_SWID_OFF	0
	#define SFD_REG_FDB_RECORD_UC_TYPE_POLICY_OFF	1
	#define SFD_REG_FDB_RECORD_UC_MAC_47_32_OFF	2
	#define SFD_REG_FDB_RECORD_UC_MAC_31_0_OFF	4
	#define SFD_REG_FDB_RECORD_UC_SUB_PORT_OFF	9
	#define SFD_REG_FDB_RECORD_UC_FID_VID_OFF	10
	#define SFD_REG_FDB_RECORD_UC_ACTION_OFF	12
	#define SFD_REG_FDB_RECORD_UC_SYSTEM_PORT_OFF	14
				int record_offset = SFD_REG_FDB_RECORD_0_OFFSET + 16*i;
				SX_PUT(inbox, reg_data->sfd_reg.swid, record_offset +SFD_REG_FDB_RECORD_UC_SWID_OFF );
				tmp_val_u8	= ((reg_data->sfd_reg.sfd_type[i] & 0xF)<<4) |
										(reg_data->sfd_reg.sfd_data_type[i].uc.policy << 2);
				SX_PUT(inbox, tmp_val_u8, record_offset +SFD_REG_FDB_RECORD_UC_TYPE_POLICY_OFF );
				tmp_val_u16	= (reg_data->sfd_reg.sfd_data_type[i].uc.mac.ether_addr_octet[5] << 8) |
										 reg_data->sfd_reg.sfd_data_type[i].uc.mac.ether_addr_octet[4];
				SX_PUT(inbox, tmp_val_u16	, record_offset +SFD_REG_FDB_RECORD_UC_MAC_47_32_OFF );
				tmp_val_u32	= (reg_data->sfd_reg.sfd_data_type[i].uc.mac.ether_addr_octet[3] << 24) |
										 (reg_data->sfd_reg.sfd_data_type[i].uc.mac.ether_addr_octet[2] << 16) |
										(reg_data->sfd_reg.sfd_data_type[i].uc.mac.ether_addr_octet[1] << 8) |
										reg_data->sfd_reg.sfd_data_type[i].uc.mac.ether_addr_octet[0];
				SX_PUT(inbox, tmp_val_u32, record_offset +SFD_REG_FDB_RECORD_UC_MAC_31_0_OFF );
				SX_PUT(inbox, reg_data->sfd_reg.sfd_data_type[i].uc.sub_port, record_offset +SFD_REG_FDB_RECORD_UC_SUB_PORT_OFF );
				SX_PUT(inbox, reg_data->sfd_reg.sfd_data_type[i].uc.fid_vid_type.vid, record_offset +SFD_REG_FDB_RECORD_UC_FID_VID_OFF );
				tmp_val_u8	= reg_data->sfd_reg.sfd_data_type[i].uc.action << 4;
				SX_PUT(inbox, tmp_val_u8, record_offset +SFD_REG_FDB_RECORD_UC_ACTION_OFF );
				SX_PUT(inbox, reg_data->sfd_reg.sfd_data_type[i].uc.system_port, record_offset +SFD_REG_FDB_RECORD_UC_SYSTEM_PORT_OFF );
				break;
			}
		case SFD_TYPE_UNICAST_LAG:
			printk("DEMO func: skipping FDB LAG record add over i2c\n");
			break;
		case SFD_TYPE_MULTICAST:
			printk("DEMO func: skipping FDB MC record add over i2c\n");
			break;
		}
	}

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(sfd_reg_len /*SFD_REG_LEN*/));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		int record_offset = SFD_REG_FDB_RECORD_0_OFFSET;

		printk("DEMO func: FDB record get over i2c will get only 1 record \n");

		SX_GET(reg_data->sfd_reg.swid, outbox, SFD_REG_SWID_OFFSET);
		SX_GET(tmp_val_u32, outbox,SFD_REG_OP_NEXT_LOCATOR_OFFSET);
		reg_data->sfd_reg.operation = (tmp_val_u32 >> 30) & 0x3;
		reg_data->sfd_reg.record_locator = tmp_val_u32 & ~(0x3 << 30);
		SX_GET(reg_data->sfd_reg.num_records, outbox, SFD_REG_NUM_RECORDS_OFFSET);

		SX_GET(reg_data->sfd_reg.swid, outbox, record_offset +SFD_REG_FDB_RECORD_UC_SWID_OFF );

		SX_GET(tmp_val_u8, outbox, record_offset +SFD_REG_FDB_RECORD_UC_TYPE_POLICY_OFF );
		reg_data->sfd_reg.sfd_type[0] = (tmp_val_u8	>> 4) & 0xF;
		reg_data->sfd_reg.sfd_data_type[0].uc.policy = (tmp_val_u8 >> 2) & 0x3;

		if (reg_data->sfd_reg.sfd_type[0] != SFD_TYPE_UNICAST) {
			printk("DEMO func: FDB record get for sfd_type = %d ( != SFD_TYPE_UNICAST) not supported !!! \n",
				   reg_data->sfd_reg.sfd_type[0] );
			goto out;
		}

		SX_GET(tmp_val_u16, outbox, record_offset +SFD_REG_FDB_RECORD_UC_MAC_47_32_OFF );
		reg_data->sfd_reg.sfd_data_type[i].uc.mac.ether_addr_octet[5] = (tmp_val_u16 >> 8) & 0xff;
		reg_data->sfd_reg.sfd_data_type[i].uc.mac.ether_addr_octet[4] = tmp_val_u16 & 0xff;

		SX_GET(tmp_val_u32, outbox, record_offset +SFD_REG_FDB_RECORD_UC_MAC_31_0_OFF );
		reg_data->sfd_reg.sfd_data_type[i].uc.mac.ether_addr_octet[3] = (tmp_val_u32>> 24) & 0xff;
		reg_data->sfd_reg.sfd_data_type[i].uc.mac.ether_addr_octet[2] = (tmp_val_u32>> 16) & 0xff;
		reg_data->sfd_reg.sfd_data_type[i].uc.mac.ether_addr_octet[1] = (tmp_val_u32>> 8) & 0xff;
		reg_data->sfd_reg.sfd_data_type[i].uc.mac.ether_addr_octet[0] = tmp_val_u32 & 0xff;

		SX_GET(reg_data->sfd_reg.sfd_data_type[i].uc.sub_port, outbox, record_offset +SFD_REG_FDB_RECORD_UC_SUB_PORT_OFF );
		SX_GET(reg_data->sfd_reg.sfd_data_type[i].uc.fid_vid_type.vid, outbox, record_offset +SFD_REG_FDB_RECORD_UC_FID_VID_OFF );

		SX_GET(tmp_val_u8, outbox, record_offset +SFD_REG_FDB_RECORD_UC_ACTION_OFF );
		reg_data->sfd_reg.sfd_data_type[i].uc.action = (tmp_val_u8 >> 4) & 0xf;

		SX_GET(reg_data->sfd_reg.sfd_data_type[i].uc.system_port, outbox, record_offset +SFD_REG_FDB_RECORD_UC_SYSTEM_PORT_OFF );
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_SFD);

int sx_ACCESS_REG_OEPFT(struct sx_dev *dev, struct ku_access_oepft_reg *reg_data)
{
	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u32 sr_flow_number = 0;

#define OEPFT_REG_SR_FLOW_NUMBER_OFFSET	0x14
#define OEPFT_REG_CPU_TCLASS_OFFSET	0x19
#define OEPFT_REG_IF_OFFSET		0x1b
#define OEPFT_REG_MAC_OFFSET		0x1c
#define OEPFT_REG_LEN			0x05

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= OEPFT_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	sr_flow_number = reg_data->oepft_reg.flow_number & 0xffffff;
	if (reg_data->oepft_reg.sr)
		sr_flow_number |= 0x80000000;
	SX_PUT(inbox, sr_flow_number, OEPFT_REG_SR_FLOW_NUMBER_OFFSET);
	SX_PUT(inbox, reg_data->oepft_reg.cpu_tclass,
			OEPFT_REG_CPU_TCLASS_OFFSET);
	SX_PUT(inbox, reg_data->oepft_reg.interface, OEPFT_REG_IF_OFFSET);
	SX_PUT(inbox, reg_data->oepft_reg.mac, OEPFT_REG_MAC_OFFSET);
	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(OEPFT_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(sr_flow_number, outbox, OEPFT_REG_SR_FLOW_NUMBER_OFFSET);
		reg_data->oepft_reg.flow_number = sr_flow_number & 0xffffff;
		reg_data->oepft_reg.sr = ((sr_flow_number >> 31) & 0x1);
		SX_GET(reg_data->oepft_reg.cpu_tclass, outbox,
				OEPFT_REG_CPU_TCLASS_OFFSET);
		SX_GET(reg_data->oepft_reg.interface, outbox,
				OEPFT_REG_IF_OFFSET);
		SX_GET(reg_data->oepft_reg.mac, outbox,
				OEPFT_REG_MAC_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

}
EXPORT_SYMBOL(sx_ACCESS_REG_OEPFT);

int sx_ACCESS_REG_PLBF(struct sx_dev *dev, struct ku_access_plbf_reg *reg_data)
{

	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define PLBF_LOCAL_PORT_OFFSET		0x15
#define PLBF_LBF_MODE_OFFSET		0x17
#define PLBF_REG_LEN			0x03

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= PLBF_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->plbf_reg.port, PLBF_LOCAL_PORT_OFFSET);
	SX_PUT(inbox, reg_data->plbf_reg.lbf_mode, PLBF_LBF_MODE_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(PAOS_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { // 0x01 = Query
		SX_GET(reg_data->plbf_reg.port, outbox,PLBF_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->plbf_reg.lbf_mode, outbox, PLBF_LBF_MODE_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

        return 0;
}
EXPORT_SYMBOL(sx_ACCESS_REG_PLBF);

int sx_ACCESS_REG_SGCR(struct sx_dev *dev, struct ku_access_sgcr_reg *reg_data)
{

	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define SGCR_LLB_OFFSET		0x1b
#define SGCR_REG_LEN			0x15

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= SGCR_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->sgcr_reg.llb, SGCR_LLB_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(SGCR_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { // 0x01 = Query
		SX_GET(reg_data->sgcr_reg.llb, outbox,SGCR_LLB_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

        return 0;
}
EXPORT_SYMBOL(sx_ACCESS_REG_SGCR);

int sx_ACCESS_REG_MSCI(struct sx_dev *dev, struct ku_access_msci_reg *reg_data)
{

	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;

#define MSCI_INDEX_OFFSET		0x17
#define MSCI_VERSION_OFFSET		0x18
#define MSCI_REG_LEN			0x5

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MSCI_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->msci_reg.index, MSCI_INDEX_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MSCI_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { // 0x01 = Query
		SX_GET(reg_data->msci_reg.index, outbox, MSCI_INDEX_OFFSET);
		SX_GET(reg_data->msci_reg.version, outbox, MSCI_VERSION_OFFSET);
	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

        return 0;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MSCI);

int sx_ACCESS_REG_MRSR(struct sx_dev *dev, struct ku_access_mrsr_reg *reg_data)
{

	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err;
	u16 type_len = 0;
	u8 tmp = 0;

#define MRSR_COMMAND_OFFSET		0x17
#define MRSR_REG_LEN			0x3

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MRSR_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	tmp = (u8)reg_data->mrsr_reg.command;
	SX_PUT(inbox, tmp, MRSR_COMMAND_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MRSR_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;

        return 0;
}

EXPORT_SYMBOL(sx_ACCESS_REG_MRSR);

int sx_ACCESS_REG_MPSC(struct sx_dev *dev, struct ku_access_mpsc_reg *reg_data)
{

	struct sx_cmd_mailbox *in_mailbox;
	struct sx_cmd_mailbox *out_mailbox;
	u32 *inbox;
	u32 *outbox;
	int err = 0;
	u16 type_len = 0;
	u8 tmp = 0;

#define REG_LOCAL_PORT_OFFSET		0x15
#define REG_C_E_OFFSET			REG_LOCAL_PORT_OFFSET + 3
#define REG_C_BIT_N			7
#define REG_E_BIT_N			6
#define REG_RATE_OFFSET                 REG_C_E_OFFSET + 4
#define REG_COUNT_DROP_OFFSET           REG_RATE_OFFSET + 4
#define MPSC_REG_LEN			0x6

	in_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(in_mailbox))
		return PTR_ERR(in_mailbox);

	out_mailbox = sx_alloc_cmd_mailbox(dev, reg_data->dev_id);
	if (IS_ERR(out_mailbox)) {
		err = PTR_ERR(out_mailbox);
		goto out_free;
	}

	inbox = in_mailbox->buf;
	memset(inbox, 0, SX_MAILBOX_SIZE);
	outbox = out_mailbox->buf;

	set_opoeration_tlv(inbox, &reg_data->op_tlv);
	type_len = REG_TLV_TYPE << 11;
	type_len |= MPSC_REG_LEN;
	SX_PUT(inbox, type_len, REG_TLV_OFFSET);
	SX_PUT(inbox, reg_data->mpsc_reg.local_port, REG_LOCAL_PORT_OFFSET);
	tmp = 0;
	tmp |= reg_data->mpsc_reg.clear_count ? (1 << REG_C_BIT_N) : 0;
	tmp |= reg_data->mpsc_reg.enable ? (1 << REG_E_BIT_N) : 0;
	SX_PUT(inbox, tmp, REG_C_E_OFFSET);
	SX_PUT(inbox, reg_data->mpsc_reg.rate, REG_RATE_OFFSET);

	err = sx_cmd_box(dev, reg_data->dev_id, in_mailbox, out_mailbox, 0, 0,
			SX_CMD_ACCESS_REG, SX_CMD_TIME_CLASS_A,
			IN_MB_SIZE(MPSC_REG_LEN));
	if (err)
		goto out;

	get_operation_tlv(outbox, &reg_data->op_tlv);
	if (reg_data->op_tlv.method == 0x01) { /* 0x01 = Query */
		SX_GET(reg_data->mpsc_reg.local_port, outbox,
				REG_LOCAL_PORT_OFFSET);
		SX_GET(reg_data->mpsc_reg.rate, outbox,
				REG_RATE_OFFSET);
		SX_GET(reg_data->mpsc_reg.count_drops, outbox,
				REG_COUNT_DROP_OFFSET);

	}

out:
	sx_free_cmd_mailbox(dev, out_mailbox);
out_free:
	sx_free_cmd_mailbox(dev, in_mailbox);
	return err;
}
EXPORT_SYMBOL(sx_ACCESS_REG_MPSC);
