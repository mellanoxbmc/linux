/**
 *
 * Copyright (C) 2012-2014, Mellanox Technologies Ltd.  ALL RIGHTS RESERVED.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/delay.h>

static unsigned short cir_reg_num = 1;
static unsigned short access_rtry = 0;

#define MAX_EMAD_FRAME_SIZE 1518
#define DIR_ROUTE_TLV_SIZE   132
#define END_TLV_SIZE           4
#define SX_MGIR_REG  0x9020 // Misc General Information Register

/* Command Interface Register 
   Read and write accesses to the CIR must be DWORD-sized and DWORD aligned 
   (i.e., the access address and access size must both be multiplications of 4 bytes)
*/
#define  CIR_GO_SW           0x0 // Software ownership
#define  CIR_GO_HW           0x1 // Hardware ownership
#define  CIR_EVENT_NO_REPORT 0x0
#define  CIR_EVENT_REPORT    0x1
#define  CIR_STATUS_OFF      0x18
#define  IN_PARAM_OFFSET     0x00
#define  IN_MODIFIER_OFFSET  0x08
#define  OUT_PARAM_OFFSET    0x0c
#define  TOKEN_OFFSET        0x14
#define  STATUS_OFFSET       0x18
#define  OPMOD_SHIFT         12
#define  EVENT_BIT           22
#define  GO_BIT              23
struct reg_cir_layout {
        u32 in_param_h;        // Input Parameter: parameter or pointer to input mailbox 
        u32 in_param_l;        // Input Parameter: parameter or pointer to input mailbox 
        u32 input_modifier;    // Input Parameter Modifier
        u32 out_param_h;       // Output Parameter: parameter or pointer to output mailbox 
        u32 out_param_l;       // Output Parameter: parameter or pointer to output mailbox 
        u16 token;
        u8 status;
        u8 event:1;
        u8 go;
        u16 opcode;
        u8 opcode_modifier;
};

/* Software Reset Register */
#define SW_RESET 0x1
struct reg_sw_reset_layout {
        u32 sw_reset:1;
        u32 reserved1:31;
};

/* DoorBell Registers 
   Located on the device BAR. The location of the doorbell registers can
   be queried through the QUERY_FW
*/
struct reg_dorbell_layout {
        u8 reserved[100];       // ??? Dorbell size 
};

/* Message Signalled Interrupt Register */
struct reg_msix_layout {
        u8 reserved[100];       // ??? Message Signalled Interrupt Register size
};

/* Clear Interrupt Register */
struct reg_clr_irq_layout {
        u32 clr_int;            // Clear Interrupt. Write transactions to this register will clear
                                // (de-assert) the virtual interrupt output pins. The value
                                // to be written in this register is obtained through the
                                // QUERY_BOARDINFO command.
                                // This register is write-only. Reading from this register will cause
                                // undefined result
};

/* SwitchX BAR0 Layout
   - Command Interface Register (CIR) is used to submit commands mainly for
     initial configuration of the device. Two commands registers are supported
     - Primary CIR - for command submission through PCIe
     - Secondary CIR - for command submission through i2c. The secondary CIR supports 
       restricted set of commands.
   - Doorbell registers are used to update the HW with new descriptors posted to send/receive descriptor
     queues or management of completion and event queues.
   - Clear Interrupt Register
   - MSIX Registers - SwitchX can generate interrupts in the form of PCI interrupt or Message Signalled Interrupt (MSIX).
   - SW Reset Register
*/
struct sx_bar0_layout {
#define SX_BAR0_OFFSET_PRIM    0x71000
        struct reg_cir_layout prim_cmd_if;   // 71000h-71018h Primary Command Interface Register
#define SX_BAR0_OFFSET_SECOND  0x72000
        struct reg_cir_layout second_cmd_if; // 72000h-72018h Secondary Command Interface Register
#define SX_SW_RESET_OFFSET     0xF0010
        struct reg_sw_reset_layout sw_reset; // F00010h SW Reset
        struct reg_dorbell_layout door_bell; // Doorbell Registers
        struct reg_clr_irq_layout irq;       // Clear Interrupt Register
        struct reg_msix_layout msix;         // MSIX Register
};

#define SX_MAX_REG_SIZE 128
 
/* MGIR Misc General Information Register Layout */
struct hw_info_layout { // 00h - 1Ch
        u16 device_hw_revision;
        u16 device_id;
        u16 resrved0;
        u8 dvfs; /* 5 bits */
        u32 resrved2[5];
        u32 uptime;
};

struct fw_info_layout { // 20h - 5Ch
        u8 resrved0;
        u8 major;
        u8 minor;
        u8 sub_minor;
        u32 build_id;
        u8 month;
        u8 day;
        u16 year;
        u16 resrved1;
        u16 hour;
        u32 psid[4];
        u32 ini_file_version;
        u32 extended_major;
        u32 extended_minor;
        u32 extended_sub_minor;
        u32 resrved4[4];
};

struct sw_info_layout { // 60h - 7Ch
        u8 resrved0;
        u8 major;
        u8 minor;
        u8 sub_minor;
        u32 resrved4[7];
};

struct reg_mgir_layout {
        struct hw_info_layout hw_info; // 00h - 1Ch
        struct fw_info_layout fw_info; // 20h - 5Ch
        struct sw_info_layout sw_info; // 60h - 7Ch
};
 
/* SwitchX commands opcodes */
enum switchx_cmd_opcodes {
        SX_QUERY_FW_OP        = 0x04, // QUERY_FW command opcode
        SX_QUERY_BOARDINFO_OP = 0x06, // QUERY_BOARDINFO command opcode
        SX_CONFIG_PROFILE_OP  = 0x100,// CONFIG_PROFILE command opcode TBD
        SX_ACCESS_REG_OP      = 0x40, // ACCESS_REG command opcode
        /* Not supported by I2c */
        SX_QUERY_AQ_CAP_OP    = 0x03,    
        SX_SW2HW_EQ_OP        = 0x13, 
        SX_HW2SW_EQ_OP        = 0x14,
        SX_QUERY_EQ_OP        = 0x15,
        SX_SW2HW_CQ_OP        = 0x16,  
        SX_HW2SW_CQ_OP        = 0x17,  
        SX_QUERY_CQ_OP        = 0x18,  
        SX_2ERR_DQ_OP         = 0x1E, 
        SX_QUERY_DQ_OP        = 0x22,
        SX_MAD_IFC_OP         = 0x24,
        SX_SW2HW_DQ_OP        = 0x201,  
        SX_HW2SW_DQ_OP        = 0x202,
        SX_INIT_MAD_DEMUX_OP  = 0x203,
        SX_MAP_FA_OP_OP       = 0xFFF, /// Should be some other constant
        SX_UNMAP_FA_OP_OP     = 0xFFE,       
};

enum switchx_ret_status {
        SX_STAT_OK            = 0x00,   // Command execution succeeded.
        SX_STAT_INTERNAL_ERR  = 0x01,   // Internal error (e.g. bus error) occurred while processing command.
        SX_STAT_BAD_OP        = 0x02,   // Operation/command not supported or opcode modifier not supported.
        SX_STAT_BAD_PARAM     = 0x03,   // Parameter not supported, parameter out of range.
        SX_STAT_BAD_SYS_STATE = 0x04,   // System was not enabled or bad system state.
        SX_STAT_BAD_RESOURCE  = 0x05,   // Attempt to access reserved or unallocated resource, or resource in
                                        // inappropriate ownership.
        SX_STAT_RESOURCE_BUSY = 0x06,   // Requested resource is currently executing a command.
        SX_STAT_EXCEED_LIM    = 0x08,   // Required capability exceeds device limits.
        SX_STAT_BAD_RES_STATE = 0x09,   // Resource is not in the appropriate state or ownership.
        SX_STAT_BAD_INDEX     = 0x0A,   // Index out of range (might be beyond table size or attempt to 
                                        // access a reserved resource).
        SX_STAT_BAD_NVMEM     = 0x0B,   // checksum/CRC failed.
        SX_STAT_BAD_PKT       = 0x30,   // Bad management packet (silently discarded).
};

/* QUERY_FW
 Description
 *------*------*-----*--------*------*---------*
 |Opcode|Op Mod|Event|IN PARAM|IN Mod|OUT PARAM|
 *------*------*-----*--------*------*---------*
 |0x4   |Yes   |Yes  |N/A     |Yes   |Mail box |
 *------*------*-----*--------*------*---------*
 The QUERY_FW command retrieves information related to the firmware, command interface version and 
 the amount of resources that should be   allocated to the firmware. The returned output parameter 
 depends on the OpMod value. When executing QUERY_FW through the i2c, it is required to work with
 local mailboxes. The location of the mailboxes on the i2c address space can be retrieved by 
 issuing QUERY_FW with opcode modifier 0x1.
*/

/* Opcode Modifier */
enum query_fw_opcode_mod {
        SX_MB_OUTPUT      = 0x00, // An output mailbox is returned, struct query_fw_output_mbox_layout
        SX_IMM_OUTPUT_1   = 0x01, // The output parameter is an immediate value, struct output_param_opmode1
        SX_IMM_OUTPUT_2   = 0x02, // The output parameter is an immediate values, struct output_param_opmode2
};

/*  Input Modifier
    Input Modifier for this command is relevant only if Opcode Modifier (OpMod) is equal to 0x2. 
    Otherwise reserved.
*/
enum query_fw_input_mod {
        SX_BAR0_OFF_NONE  = 0x00, // Input modifier parameter is not relevant
        SX_BAR0_OFF_BSR   = 0x01, // Output parameter returns the offset in BAR0 where the 
                                  // Boot Syndrome Register should be read from
        SX_BAR0_OFF_FER   = 0x03  // Output parameter returns the offset in BAR0 where the
                                  // Fatal Error Register should be read from
};

struct query_fw_output_mbox_layout {
	u16 fw_rev_major;
	u16 fw_pages;
	u16 fw_rev_minor;
	u16 fw_rev_subminor;
	u16 cmd_interface_rev;
	u16 core_clk;
	u32 reserved1:31;
	u32 dt:1;
	u8 reserved2;
	u8 fw_seconds;
	u8 fw_minutes;
	u8 fw_hour;
	u16 fw_year;                // Firmware timestamp - year (displayed as a hexadecimal number; e.g. 0x2005)
	u8 fw_month;                // Firmware timestamp - month (displayed as a hexadecimal number)
	u8 fw_day;                  // Firmware timestamp - day (displayed as a hexadecimal number)

	u32 reserved3;
 	u32 reserved4;
 	u32 clr_int_base_offset_h;
 	u32 clr_int_base_offset_l;
 	u32 reserved5:30; 
 	u32 clr_int_bar:2;
 	u32 reserved6;
 	u32 error_buf_offset_h;
 	u32 error_buf_offset_l;
 	u32 error_buf_size;
 	u32 reserved7:30;
 	u32 error_buf_bar:2;
 	u32 doorbell_page_offset_h;
 	u32 doorbell_page_offset_l;
 	u32 reserved8:30;
 	u32 doorbell_page_bar:2;
 	u8 reserved9[180];
};


struct param_h_opmode1 {
 	u32 local_mb_bar0_offset:20;
 	u32 local_mb_size:12;
};

// Format for Immediate Output Parameter With OpMod=0x1
struct output_param_opmode1 {
 	struct param_h_opmode1 out_param_h; // bits 63:32 of local output mailbox - cir_reg out_param_l 
 	struct param_h_opmode1 out_param_l; // bits 31: 0 of local output mailbox - cir_reg out_param_h
};

struct param_h_opmode2 {
 	u32 local_mb_bar0_offset;
};

// Format for Immediate Output Parameter With OpMod=0x2
struct output_param_opmode2 {
 	u32 out_param_h;                    // bits 63:32 of output parameter (reserved)
 	struct param_h_opmode2 out_param_l; // bits 31: 0 of output parameter
};

/* ACCESS_REG
 Description
 *------*------*-----*--------*------*---------*
 |Opcode|Op Mod|Event|IN PARAM|IN Mod|OUT PARAM|
 *------*------*-----*--------*------*---------*
 |      |N/A   |Y    |Mail box|N/A   |Mail box |
 *------*------*-----*--------*-------*--------*
 The ACCESS_REG command supports accessing device registers through PCIe and i2c. This
 access is mainly used for bootstrapping. Note that only a subset of registers is supported
 as listed below
*/
#define MGIR_REG_ID  SX_MGIR_REG  // 0x9020 MGIR Misc General Information Register


/* EMAD TLV Types */
enum emad_tlv_types {
        SX_END_OF_TLV        = 0x00, // "END"       len=1   End of TLV list. Must be present on all packets as the last TLV.
        SX_OPERATION_TLV     = 0x01, // "OPERATION" len=4   Operation TLV, describes the method/class and response status.
        SX_DIRECT_ROUTE_TLV  = 0x02, // "DR"        len=33  Direct Route TLV. Describes the direct route path. Must only be present
                                     //                     in direct route EMADs. Must follow OPERATION TLV if present.
        SX_REG_ACCESS_TLV    = 0x03, // "REG"       len=VAR Register value. Must be present for REG_ACCESS class.
        SX_REG_USER_DATA_TLV = 0x04, // "USERDATA"  len=VAR User data for IPC class.
};

/* EMAD Classes */
enum emad_classes {
        SX_EMAD_RESERVED_CLASS   = 0x00, // Reserved Reserved for future use N/A
        SX_EMAD_REG_ACCESS_CLASS = 0x01, // Access to SwitchX registers. register_id specifies the register to be accessed.
                                         // OPERATION, (DR), REG, END
        SX_EMAD_CPU_OPER_CLASS   = 0x02, // IPC Access to CPU OPERATION, (DR), USERDATA, END
};

struct emad_eth_header {
        u8 dmac3;
        u8 dmac2;
        u8 dmac1;
        u8 dmac0;
        u8 smac1; 
        u8 smac0;
        u8 dmac5;
        u8 dmac4;
        u8 smac5;
        u8 smac4;
        u8 smac3;
        u8 smac2;
        u8 reserved1:4;
        u8 ver:4;
        u8 mlx_proto;
        u16 et; 
};

enum oper_tlv_status {
        OPER_TLV_STATUS_OK           = 0x00, // Good. Operation Performed.
        OPER_TLV_STATUS_BUSY         = 0x01, // Device is busy. Can not perform the operation at the moment, requester
                                             // should retry the operation later.
        OPER_TLV_STATUS_VER_NSUPP    = 0x02, // Version not supported.
        OPER_TLV_STATUS_UNKNWN       = 0x03, // Unknown TLV.
        OPER_TLV_STATUS_REG_NSUPP    = 0x04, // Register not supported.
        OPER_TLV_STATUS_CLASS_NSUPP  = 0x05, // Class not supported.
        OPER_TLV_STATUS_METHOD_NSUPP = 0x06, // Method not supported
        OPER_TLV_STATUS_BAD_PARAM    = 0x07, // Bad parameter (e.g. port out of range, non stacking port)
        OPER_TLV_STATUS_RSRC_NAVAIL  = 0x08, // Resource not available (e.g. attempt to write to a full FDB,
                                             // allocation failed)
        OPER_TLV_STATUS_MSG_ACK      = 0x09, // Message Receipt Acknowledgement. Will return answer later.
                                             // Requester should rearm retransmission timer.
};

enum oper_tlv_method {
        OPER_TLV_METHOD_QUERY = 0x01, // Query
        OPER_TLV_METHOD_WRITE = 0x02, // Write
        OPER_TLV_METHOD_SEND  = 0x03, // Send (response not supported)
        OPER_TLV_METHOD_EVENT = 0x05, // Event (optional response)
};

#define DIRECT_ROUTE_NOT_SET 0x0
#define DIRECT_ROUTE_SET     0x1
#define OPER_TLV_REQ         0x0
#define OPER_TLV_RESP        0x1
struct operation_tlv {
        u8 type;
        u16 len;
        u8 dr;
        u8 status;
        u16 register_id;
        u8 res_req;
        u8 method;
        u64 tid;                // Transaction ID - 64 bit opaque value returned as is in response message
                                // This field simplifies software tracking of transactions.
};

struct end_tlv {
        u8 type;
        u16 len;
        u8 reserved1;
};

/* Maximum size of swicth_reg filed within REG_ACCESS TLV */
#define SX_REG_MAX_SIZE ( MAX_EMAD_FRAME_SIZE - \
                          DIR_ROUTE_TLV_SIZE - \
                          sizeof(struct emad_eth_header) - \
                          sizeof(struct operation_tlv) - \
                          struct(struct end_tlv) \
                         ) // size in bytes
/* Maximum size of emad frame for access register EMAD frame */
#define SX_ACCESS_REG_EF ( SX_REG_MAX_SIZE + \
                           sizeof(struct operation_tlv) - \
                           sizeof(struct end_tlv) \
                         ) // size in bytes
struct reg_access_tlv {
#ifdef __BIG_ENDIAN
        u16 type:5;             // Operation - from enum emad_tlv_types
        u16 len:11;             // Length of TLV in DWORDs
        u16 reserved1;          //
#else
        u32 b1;
#endif
        void *switchx_reg;      // switchx register, referenced in register_id filed of operation TLV,
                                // variable size, maximium size iz SX_REG_MAX_SIZE
};

/* Access Register Layout */
struct access_reg_layout {
        struct operation_tlv oper_tlv;      // Operation to perform including all fields
        struct reg_access_tlv reg_accs_tlv; // Register access information
};

/* EMAD Frame Format Layout
        eth_hdr;          // Ethernet Header
        oper_tlv;         // OPERATION TLV (Mandatory)
        dr_tlv;           // DR TLV (Present in DR packets only)
        reg_access;       // REG TLV (present in REG_ACCESS class)  
        user_data;        // USERDATA TLV (present in IPC class)
        end_tlv;          // END TLV (Mandatory)
*/
struct emad_frame_access_reg_template {
        struct emad_eth_header eth_hdr;
        struct operation_tlv oper_tlv;
        struct reg_access_tlv reg_accs_tlv;
        struct end_tlv e_tlv;
};

#define SX_I2C_DEV_SIGNATURE 0xf0e1c2b3
#define SXDBGMODE 1
#define dprintk(...) do { } while(0)

#define SWITCHX_ADDR32                4
#define MAX_I2C_BUFF_SIZE           256
#define MAX_ASIC_ID_FOR_HEALTH_TEST 250

#define BUS_ID_FROM_ID(i2c_dev_id)          ( (i2c_dev_id >> 8) & 0x0000ffff)
#define GET_PARENT_BUS_ID(bus_id)           ( (bus_id >> 8) & 0x0000ffff)
#define GET_LOCAL_BUS_ID(asic_id)           ( ((asic_id % 2) == 1) ? 1 : 2)
#define GET_SHIFT_FROM_LOCAL_BUS(local_bus) ( (local_bus == 1) ? 6 : 4)
#define ADDR_FROM_DEV_ID(i2c_dev_id)        ( i2c_dev_id & 0x7f )
#define GET_WIDTH(off)                      ( (off > 0xffff) ? 4 : (off > 0xff) ? 2 : 1 )

#define _FL_ struct file *fl,

#define LPCI2C_BLOCK_MAX 32

#define REG_ACCESS_TLV_LEN		                0x01
#define REG_TLV_LEN		                        0x04
#define REG_TLV_END_LEN		                        0x01
#define REG_TLV_TYPE		                        0x03
#define REG_TLV_OFFSET		                        0x10
#define ASIC_PSID_SZIE		                          16
#define MGIR_REG_LEN					0x20
#define MGIR_REG_HW_INFO_OFFSET				0x14
#define MGIR_REG_HW_INFO_DEVICE_HW_REVISION_OFFSET	0x0
#define MGIR_REG_HW_INFO_DEVICE_ID_OFFSET		0x2
#define MGIR_REG_HW_INFO_DVFS_OFFSET			0x7
#define MGIR_REG_HW_INFO_UPTIME_OFFSET			0x1c
#define MGIR_REG_FW_INFO_OFFSET				0x34
#define MGIR_REG_FW_INFO_MAJOR_OFFSET			0x01
#define MGIR_REG_FW_INFO_MINOR_OFFSET			0x02
#define MGIR_REG_FW_INFO_SUB_MINOR_OFFSET		0x03
#define MGIR_REG_FW_INFO_BUILD_ID_OFFSET		0x04
#define MGIR_REG_FW_INFO_MONTH_OFFSET			0x08
#define MGIR_REG_FW_INFO_DAY_OFFSET			0x09
#define MGIR_REG_FW_INFO_YEAR_OFFSET			0x0a
#define MGIR_REG_FW_INFO_HOUR_OFFSET			0x0e
#define MGIR_REG_FW_INFO_PSID_OFFSET			0x10
#define MGIR_REG_FW_INFO_INI_FILE_VERSION_OFFSET	0x20
#define MGIR_REG_FW_INFO_EXTENDED_MAJOR_OFFSET		0x24
#define MGIR_REG_FW_INFO_EXTENDED_MINOR_OFFSET		0x28
#define MGIR_REG_FW_INFO_EXTENDED_SUB_MINOR_OFFSET	0x2c
#define MGIR_REG_SW_INFO_OFFSET				0x74
#define MGIR_REG_SW_INFO_MAJOR_OFFSET			0x01
#define MGIR_REG_SW_INFO_MINOR_OFFSET			0x02
#define MGIR_REG_SW_INFO_SUB_MINOR_OFFSET		0x03

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x48, I2C_CLIENT_END };


/* Each client has this additional data */
struct switchx_data {
	struct list_head list;
	int signature;
	struct device *i2c_dev;
	struct i2c_client *client;
	char name[I2C_NAME_SIZE];
	struct reg_cir_layout reg_cir;
	u32 mb_size_in;
	u32 mb_offset_in;
	u32 mb_size_out;
	u32 mb_offset_out;
	int retries;
	int timeout;
	int retry_cntr;
	u64 transact_id;
	int bus_id;
	int i2c_dev_addr;
	u8 cache[SX_MAX_REG_SIZE];
	struct mutex cmd_lock;
#define SX_ROUTE_DEPTH 4
	struct i2c_adapter *parent[SX_ROUTE_DEPTH];
	int (*deselect[SX_ROUTE_DEPTH - 1])(struct i2c_adapter *, void *mux_dev, u32 chan_id);
	u8 route_enforced;
#define GO_BIT_STAT_OK    0
#define GO_BIT_STAT_STUCK 1
	u8 go_bit_status;
	u16 asic_id;
};

/* Container structure */
struct switchx_probe_failed {
	struct list_head list;
	int bus_id;
	int probe_cntr;
};

/* Container structure */
struct switchx_config {
	struct list_head switchx_list;
	struct list_head probe_failed_list;
	u32 reg_cir_offset;
	u32 reg_sw_reset_offset;
	u32 last_access_dev_id;
	int last_accessed_bus_id;
};
static struct switchx_config switchx_cfg;

static struct kmem_cache *switchx_probe_failed_cache __read_mostly;

enum chips {
	any_chip, switchx, connectx, switchib, switchspc,
};

static inline int translate_offset_addr(u32 addr, u32 offset, u32 len, u8 width, char *msgbuf,
				  	struct i2c_msg *msg, u8 *in_out_buf, u8 flag);

#define ICR_REG_WRITE_GO_BIT(event, op_modifier, op, icr_reg)                          \
        cpu_to_be32(((1 << GO_BIT) | (event ? (1 << EVENT_BIT) : 0) |                  \
                        (op_modifier << OPMOD_SHIFT) | op), icr_reg);

#define ASIC_GET_BUF(dest, source, offset)				\
	do {								\
		void *__p = (char *) (source) + (offset);		\
		switch (sizeof(dest)) {				        \
		case 1: (dest) = *(u8 *) __p; break;	      	        \
		case 2: (dest) = be16_to_cpup(__p); break;	 	\
		case 4: (dest) = be32_to_cpup(__p); break;	       	\
		case 8: (dest) = be64_to_cpup(__p); break;	      	\
		default: break;						\
		}							\
	} while (0)

#define ASIC_SET_BUF(dest, source, offset)				\
	do {								\
		void *__d = ((char *) (dest) + (offset));		\
		switch (sizeof(source)) {				\
		case 1: *(u8 *) __d = (source);	 break; 		\
		case 2: *(__be16 *) __d = cpu_to_be16(source); break; 	\
		case 4: *(__be32 *) __d = cpu_to_be32(source); break; 	\
		case 8: *(__be64 *) __d = cpu_to_be64(source); break; 	\
		default:  break;					\
		}							\
	} while (0)

#define TYPE_LEN_OFFSET         0x00
#define DR_STATUS_OFFSET        0x02
#define REGISTER_ID_OFFSET      0x04
#define R_METHOD_OFFSET         0x06
#define CLASS_OFFSET            0x07
#define TID_OFFSET              0x08
#define SET_OPER_TLV(buf,type,len,dr,status,res_req,reg_id,method,class,tid)	\
{										\
        u16 type_len = 0;						        \
        u8 dr_status = 0;						        \
        u8 r_method = 0;						        \
        type_len = len | (type << 11);						\
        ASIC_SET_BUF(buf, type_len, TYPE_LEN_OFFSET);				\
        dr_status = status | (dr << 7);						\
        ASIC_SET_BUF(buf, dr_status, DR_STATUS_OFFSET);				\
        ASIC_SET_BUF(buf, reg_id, REGISTER_ID_OFFSET);		                \
        r_method = method | (res_req << 7);					\
        ASIC_SET_BUF(buf, r_method, R_METHOD_OFFSET);				\
        ASIC_SET_BUF(buf, class, CLASS_OFFSET);					\
        ASIC_SET_BUF(buf, tid, TID_OFFSET);					\
}

#define OPERATION_TLV_SIZE	0x10
#define IN_MB_SIZE(reg_dword_size) (((reg_dword_size) * 4) + OPERATION_TLV_SIZE)
#define TYPE_LEN_OFFSET		0x00
#define DR_STATUS_OFFSET	0x02
#define REGISTER_ID_OFFSET	0x04
#define R_METHOD_OFFSET		0x06
#define CLASS_OFFSET		0x07
#define TID_OFFSET		0x08
#define GET_OPER_TLV(buf,type,len,dr,status,res_req,reg_id,method,class,tid)	\
{						        			\
	u16 type_len = 0;						        \
	u8 dr_status = 0;						        \
	u8 r_method = 0;						        \
	ASIC_GET_BUF(type_len, buf, TYPE_LEN_OFFSET);				\
	len = type_len & 0x7ff;						        \
	type = type_len >> 11;						        \
	ASIC_GET_BUF(dr_status, buf, DR_STATUS_OFFSET);				\
	status = dr_status & 0x7f;						\
	dr = dr_status >> 7;						        \
	ASIC_GET_BUF(reg_id, buf, REGISTER_ID_OFFSET);				\
	ASIC_GET_BUF(r_method, buf, R_METHOD_OFFSET);				\
	method = r_method & 0x7f;						\
	res_req = r_method >> 7;						\
	ASIC_GET_BUF(class, buf, CLASS_OFFSET);					\
	ASIC_GET_BUF(tid, buf, TID_OFFSET);					\										\
}

static inline void get_mgir_reg(u8 *buf, struct reg_mgir_layout *mgir)
{
	ASIC_GET_BUF(mgir->hw_info.device_hw_revision, buf, MGIR_REG_HW_INFO_OFFSET + MGIR_REG_HW_INFO_DEVICE_HW_REVISION_OFFSET);
	ASIC_GET_BUF(mgir->hw_info.device_id, buf, MGIR_REG_HW_INFO_OFFSET + MGIR_REG_HW_INFO_DEVICE_ID_OFFSET);
	ASIC_GET_BUF(mgir->hw_info.dvfs, buf, MGIR_REG_HW_INFO_OFFSET + MGIR_REG_HW_INFO_DVFS_OFFSET);
	mgir->hw_info.dvfs &= 0x1f;
	ASIC_GET_BUF(mgir->hw_info.uptime, buf, MGIR_REG_HW_INFO_OFFSET + MGIR_REG_HW_INFO_UPTIME_OFFSET);
	ASIC_GET_BUF(mgir->fw_info.major, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_MAJOR_OFFSET);
	ASIC_GET_BUF(mgir->fw_info.minor, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_MINOR_OFFSET);
	ASIC_GET_BUF(mgir->fw_info.sub_minor, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_SUB_MINOR_OFFSET);
	ASIC_GET_BUF(mgir->fw_info.build_id, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_BUILD_ID_OFFSET);
	ASIC_GET_BUF(mgir->fw_info.month, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_MONTH_OFFSET);
	ASIC_GET_BUF(mgir->fw_info.day, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_DAY_OFFSET);
	ASIC_GET_BUF(mgir->fw_info.year, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_YEAR_OFFSET);
	ASIC_GET_BUF(mgir->fw_info.hour, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_HOUR_OFFSET);
	memcpy(mgir->fw_info.psid, buf + MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_PSID_OFFSET, ASIC_PSID_SZIE);
	ASIC_GET_BUF(mgir->fw_info.ini_file_version, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_INI_FILE_VERSION_OFFSET);
	ASIC_GET_BUF(mgir->fw_info.extended_major, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_EXTENDED_MAJOR_OFFSET);
	ASIC_GET_BUF(mgir->fw_info.extended_minor, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_EXTENDED_MINOR_OFFSET);
	ASIC_GET_BUF(mgir->fw_info.extended_sub_minor, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_FW_INFO_EXTENDED_SUB_MINOR_OFFSET);
	ASIC_GET_BUF(mgir->sw_info.major, buf, MGIR_REG_SW_INFO_OFFSET + MGIR_REG_SW_INFO_MAJOR_OFFSET);
	ASIC_GET_BUF(mgir->sw_info.minor, buf, MGIR_REG_FW_INFO_OFFSET + MGIR_REG_SW_INFO_MINOR_OFFSET);
	ASIC_GET_BUF(mgir->sw_info.sub_minor, buf, MGIR_REG_SW_INFO_OFFSET + MGIR_REG_SW_INFO_SUB_MINOR_OFFSET);
}

#define I2C_XFER(client, data, msgs, try, orig_jiffies, read_write)            \
	orig_jiffies = jiffies;                                                \
	for (try = 0; try <= data->retries; try++) {                           \
		rc = i2c_transfer(client->adapter, msgs, read_write);          \
		if (rc > 0) {		                                       \
			break;                                                 \
		}                                                              \
		if (time_after(jiffies, orig_jiffies + data->timeout)) {       \
			rc = -EIO;                                             \
			printk( "%s try=%d rc=%d\n", __FUNCTION__, try, rc);    \
			return rc;                                             \
		}                                                              \
	}                                                                      \
	printk( "%s try=%d rc=%d\n", __FUNCTION__, try, rc);

#define I2C_SET_BUF_XFER(client, data, try, orig_jiffies, tbuf, msgs, msgbuf, offset,         \
				size, width, rw_flag)                                         \
	translate_offset_addr(client->addr, offset, size,                                     \
				width, msgbuf, msgs, tbuf, rw_flag);                          \
	I2C_XFER(client, data, msgs, try, orig_jiffies, rw_flag);                             \
	if (rc < 0) {                                                                         \
		printk(KERN_INFO "%s(%d) rc=%d\n", __FUNCTION__, __LINE__, rc);               \
		return rc;                                                                    \
	}

#define BUS_XFER(c,d,rw,p,q,s) I2C_XFER(client, data, msgs, try, orig_jiffies, read_write)

static inline int icr_reg_write(struct i2c_client *client, struct switchx_data *data,
				u16 token, u32 in_modifier,
				u8 op_modifier, u16 op)
{
	unsigned long orig_jiffies;
	int try;
	struct i2c_msg msgs[1];
	u8 msgbuf[SWITCHX_ADDR32];
	u32 icr_reg_buf[7];
	int rc = -EIO;
	memset(icr_reg_buf, 0, 28);
	icr_reg_buf[2] = cpu_to_be32(in_modifier);
	icr_reg_buf[5] = cpu_to_be32(token << 16);
	icr_reg_buf[6] = cpu_to_be32((op_modifier << OPMOD_SHIFT) | op);

	dprintk(		 "%s(%d) icr_reg_buf[2]=0x%08x icr_reg_buf[5]=0x%08x icr_reg_buf[6]=0x%08x\n",
		  __FUNCTION__, __LINE__, icr_reg_buf[2], icr_reg_buf[5], icr_reg_buf[6]);

	I2C_SET_BUF_XFER(client, data, try, orig_jiffies,
				(u8 *)icr_reg_buf, msgs, msgbuf,
				SX_BAR0_OFFSET_SECOND, 28, SWITCHX_ADDR32, 0);

	/* Write go bit as ceparate */
	icr_reg_buf[6] |= cpu_to_be32(1 << GO_BIT);
	dprintk(		  "%s(%d) icr_reg_buf[6]=0x%08x\n",
		  __FUNCTION__, __LINE__, icr_reg_buf[6]);
	I2C_SET_BUF_XFER(client, data, try, orig_jiffies,
				(u8 *)&icr_reg_buf[6], msgs, msgbuf,
				SX_BAR0_OFFSET_SECOND + STATUS_OFFSET, 4, SWITCHX_ADDR32, 0);

	return rc;
}

static inline int icr_reg_read(struct i2c_client *client, u32 icr_reg_buf, u8 *status, u8 *go)
{
	int rc = 0;
	*status = icr_reg_buf & 0x000000ff; //>> 24;
	*go = icr_reg_buf & (1 << GO_BIT);

	dprintk(		 "%s(%d) icr_reg_buf=0x%02x *status=0x%08x *go=0x%02x\n",
		  __FUNCTION__, __LINE__, icr_reg_buf, *status, *go);

	return rc;
}

static inline void convert_mbox(u32 *p_mbox)
{
	*p_mbox = cpu_to_be32(*p_mbox);
	*(p_mbox + 1) = cpu_to_be32(*(p_mbox + 1));
}

static inline void convert_query_fw_mbox(u32 *p_query_fw_mbox)
{
	int i;
	u32 *p_mbox = p_query_fw_mbox;
	int dword_size = sizeof(struct query_fw_output_mbox_layout) / 4;
	if (sizeof(struct query_fw_output_mbox_layout) % 4)
		dword_size += 1;
	for (i = 0; i < dword_size; i++, p_mbox++)
		*p_mbox = cpu_to_be32(*p_mbox);
}

static inline void convert_oper_tlv(u32 *p_oper_tlv)
{
	int i;
	u32 *p_tlv = p_oper_tlv;
	int dword_size = sizeof(struct operation_tlv) / 4;
	if (sizeof(struct operation_tlv) % 4)
		dword_size += 1;
	for (i = 0; i < dword_size; i++, p_tlv++)
		*p_tlv = cpu_to_be32(*p_tlv);
}

static inline void convert_reg_accs_tlv(u32 *p_reg_tlv)
{
	int i;
	u32 *p_tlv = p_reg_tlv;
	int dword_size = sizeof(struct reg_access_tlv) / 4;
	if (sizeof(struct reg_access_tlv) % 4)
		dword_size += 1;
	for (i = 0; i < dword_size; i++, p_tlv++)
		*p_tlv = cpu_to_be32(*p_tlv);
}

static inline void convert_end_tlv(u32 *p_end_tlv)
{
	int i;
	u32 *p_tlv = p_end_tlv;
	int dword_size = sizeof(struct end_tlv) / 4;
	if (sizeof(struct end_tlv) % 4)
		dword_size += 1;
	for (i = 0; i < dword_size; i++, p_tlv++)
		*p_tlv = cpu_to_be32(*p_tlv);
}

static inline int translate_offset_addr(u32 addr, u32 offset, u32 len, u8 width, char *msgbuf,
				  struct i2c_msg *msg, u8 *in_out_buf, u8 flag)
{
	char *msg0;
	struct i2c_msg *msg_i2c = msg;

	if (flag == I2C_M_RD) {
		msg0 = msgbuf;
		switch (width) {
			case 4:
				*msg0++ = offset >> 24;
				*msg0++ = offset >> 16;
				*msg0++ = offset >> 8;
				*msg0 = offset;
				break;
			case 2:
				offset &= 0xffff;
				*msg0++ = offset >> 8;
				*msg0 = offset;
				break;
			case 1:
				offset &= 0xff;
				*msg0 = offset;
				break;
		}
		msg_i2c->addr = addr;
		msg_i2c->buf = msgbuf;
		msg_i2c->len = width;
		msg_i2c++;
		msg_i2c->addr = addr;
		msg_i2c->flags = I2C_M_RD;
		msg_i2c->buf = in_out_buf;
		msg_i2c->len = len;
	}
	else {
		msg0 = in_out_buf;
		switch (width) {
			case 4:
				*msg0++ = offset >> 24;
				*msg0++ = offset >> 16;
				*msg0++ = offset >> 8;
				*msg0 = offset;
				break;
			case 2:
				offset &= 0xffff;
				*msg0++ = offset >> 8;
				*msg0 = offset;
				break;
			case 1:
				offset &= 0xff;
				*msg0 = offset;
				break;
		}
		msg_i2c->addr = addr;
		msg_i2c->flags = 0;
		msg_i2c->len = width + len;
		msg_i2c->buf = in_out_buf;
	}
	return 1;
}

static int i2c_software_reset(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct switchx_data *data = i2c_get_clientdata(client);
	unsigned long orig_jiffies;
	int try;
	u8 tbuf[sizeof(struct reg_sw_reset_layout) + SWITCHX_ADDR32];
	struct reg_sw_reset_layout sw_res;
	int rc = -EIO;
	struct i2c_msg msgs[1];
	u8 msgbuf[SWITCHX_ADDR32];

        memset(&sw_res, 0, sizeof(struct reg_sw_reset_layout));
        sw_res.sw_reset = SW_RESET;
	memset(tbuf, 0, sizeof(struct reg_sw_reset_layout) + SWITCHX_ADDR32);
	memcpy(tbuf + SWITCHX_ADDR32, (u8 *)&sw_res, sizeof(struct reg_sw_reset_layout));

	translate_offset_addr(client->addr, switchx_cfg.reg_sw_reset_offset, sizeof(struct reg_sw_reset_layout),
				SWITCHX_ADDR32, msgbuf, msgs, tbuf, 0);

	orig_jiffies = jiffies;
	for (try = 0; try <= data->retries; try++) {
		rc = i2c_transfer(client->adapter, msgs, 1);
		if (rc > 0) {
			break;
		}
		if (time_after(jiffies, orig_jiffies + data->timeout)) {
			rc = -EIO;
			break;
		}
	}

	return rc;
}

#define LEN_ACQ_GO 4
#define OFF_ACQ_GO ( switchx_cfg.reg_cir_offset + 24 )
static int acquire_go_bit_complete(struct i2c_client *client,
			           struct switchx_data *data,
			           struct reg_cir_layout *reg_cir,
			           int len,
			           int offset,
			           u8 *in_out_buf,
			           int (*f_finish)(struct i2c_client *client,
					           struct switchx_data *data,
					           struct reg_cir_layout *reg_cir,
					           int len,
					           int offset,
					           u8 *in_out_buf))
{
	struct reg_cir_layout _reg_cir;
	int rc = -EIO;
	int try = 0;
	unsigned long orig_jiffies;
	struct i2c_msg msgs[2];
	u8 msgbuf[SWITCHX_ADDR32];
	u8 status = 1;
	u8 go = 1;

	memset(&_reg_cir, 0, sizeof(struct reg_cir_layout));
	memset(msgs, 0, 2*sizeof(struct i2c_msg));

	dprintk(		"%s(%d) before getting GO len=%d offset=%d LEN_ACQ_GO=%d OFF_ACQ_GO=0x%08x\n",
		__FUNCTION__, __LINE__, len, offset, LEN_ACQ_GO, OFF_ACQ_GO);

	translate_offset_addr(client->addr, OFF_ACQ_GO, LEN_ACQ_GO/* + 4*/, SWITCHX_ADDR32, msgbuf, msgs,
				((u8 *)(&_reg_cir) + CIR_STATUS_OFF), I2C_M_RD);

	orig_jiffies = jiffies;
	for (try = 0; try <= data->retries; try++) {
		rc = i2c_transfer(client->adapter, msgs, 2);
		rc = (rc >= 0) ? icr_reg_read(client, *((u32 *)((u8 *)(&_reg_cir) + CIR_STATUS_OFF)), &status, &go) : rc;
		if ((rc >= 0) && (go == CIR_GO_SW) /*&& (status == 0)*/) {
			dprintk(				"%s(%d) go=%x status=%x rc=%d\n",
				__FUNCTION__, __LINE__, go, status, rc);
			_reg_cir.status = status;
			_reg_cir.go = go;
			if(f_finish)
				rc = f_finish(client, data, &_reg_cir, len, offset, in_out_buf);
			break;
		}
		if (time_after(jiffies, orig_jiffies + data->timeout)) {
			rc = -EIO;
			break;
		}
		cond_resched();
	}
	data->retry_cntr += try;

	return (try <= data->retries) ? rc : -EIO;
}

static int acquire_go_bit(struct i2c_client *client,
		     	  struct switchx_data *data,
		     	  struct reg_cir_layout *reg_cir,
			  int len,
			  int offset,
		     	  u8 *in_out_buf,
		     	  int (*f_start)(struct i2c_client *client,
				         struct switchx_data *data,
				         struct reg_cir_layout *reg_cir,
				         int len,
					 int offset,
				         u8 *in_out_buf),
		     	  int (*f_finish)(struct i2c_client *client,
				          struct switchx_data *data,
				          struct reg_cir_layout *reg_cir,
				          int len,
					  int offset,
				          u8 *in_out_buf))
{
	struct reg_cir_layout _reg_cir;
	int try = 0;
	int rc = -EIO;
	struct i2c_msg msgs[2];
	u8 msgbuf[SWITCHX_ADDR32];
	unsigned long orig_jiffies;
	u8 status = 0;
	u8 go = 1;

	memset(msgs, 0, 2*sizeof(struct i2c_msg));
	memset(&_reg_cir, 0, sizeof(struct reg_cir_layout));
	_reg_cir.go = 1; // Just for cleanup
	mutex_lock(&data->cmd_lock);

	dprintk(		"%s(%d) before getting GO len=%d offset=%d LEN_ACQ_GO=%d OFF_ACQ_GO=0x%08x 0x%08x 0x%08x\n",
		__FUNCTION__, __LINE__, len, offset, LEN_ACQ_GO, OFF_ACQ_GO,
		(int)offsetof(struct reg_cir_layout, status), CIR_STATUS_OFF);

	translate_offset_addr(client->addr, OFF_ACQ_GO, LEN_ACQ_GO, SWITCHX_ADDR32, msgbuf, msgs,
				((u8 *)(&_reg_cir) + CIR_STATUS_OFF), I2C_M_RD);


	orig_jiffies = jiffies;
	for (try = 0; try <= data->retries; try++) {
		rc = i2c_transfer(client->adapter, msgs, 2);
		rc = (rc >= 0) ? icr_reg_read(client, *((u32 *)((u8 *)(&_reg_cir) + CIR_STATUS_OFF)), &status, &go) : rc;
		if ((rc >= 0) && (go == CIR_GO_SW) /*&& (!status)*/) {
			dprintk(				"%s(%d) got GO go=%x status=%x go=%x opcode=%x opcode_mod=%x event=%x status=%x buf %x %x %x %x\n",
				__FUNCTION__, __LINE__, go, status, _reg_cir.go,
				_reg_cir.opcode, _reg_cir.opcode_modifier,
        			_reg_cir.event, _reg_cir.status,
        			*((u8 *)(&_reg_cir) + CIR_STATUS_OFF), *((u8 *)(&_reg_cir) + CIR_STATUS_OFF + 1),
        			*((u8 *)(&_reg_cir) + CIR_STATUS_OFF + 2), *((u8 *)(&_reg_cir) + CIR_STATUS_OFF + 3));
			if (f_start) {
				rc = f_start(client, data, reg_cir, len, offset, in_out_buf);
				if (!rc)
					break;

				if(f_finish)
					rc = acquire_go_bit_complete(client, data, reg_cir, len,
									offset, in_out_buf, f_finish);
			}
			else {
				rc = 0;
			}
			break;
		}
		if (time_after(jiffies, orig_jiffies + data->timeout)) {
			rc = -EIO;
			break;
		}
		cond_resched();
	}

	mutex_unlock(&data->cmd_lock);
	data->retry_cntr += try;

	return  (try <= data->retries) ? rc : -EIO;
}

static int _cmd_start(struct i2c_client *client,
		      struct switchx_data *data,
		      struct reg_cir_layout *reg_cir,
		      int len, int offset, u8 *in_out_buf)
{
	struct i2c_msg msgs[1];
	u8 msgbuf[SWITCHX_ADDR32];
	u8 tbuf[MAX_I2C_BUFF_SIZE];
	int size = sizeof(struct reg_cir_layout);
        int j, out_len = 0;
	int rc;
	u32 icr_reg = 0;
	u32 icr_reg_buf[7];
        int num = len/LPCI2C_BLOCK_MAX;
        u8 *buf;
        int off;
        int chunk_size = (len > LPCI2C_BLOCK_MAX) ? LPCI2C_BLOCK_MAX : len;
        int i;


        if (len%LPCI2C_BLOCK_MAX)
                num++;

	dprintk(		"%s(%d) cir 0x%08x go=%x stat=%x tok=%d opcode=%d opmod=%d event=%d inmod %x par %x %x %x %x len %d offset %d\n",
		__FUNCTION__, __LINE__, switchx_cfg.reg_cir_offset, reg_cir->go, reg_cir->status,
		reg_cir->token, reg_cir->opcode, reg_cir->opcode_modifier, reg_cir->event, reg_cir->input_modifier,
		reg_cir->in_param_h, reg_cir->in_param_l, reg_cir->out_param_h, reg_cir->out_param_l, len, offset);
 
	if (reg_cir->opcode == SX_ACCESS_REG_OP) {
		dprintk(			"%s write input at 0x%08x (len=%d)\n",
			__FUNCTION__, data->mb_offset_in, len);
		memset(msgs, 0, sizeof(struct i2c_msg));
		memset(tbuf, 0, len + SWITCHX_ADDR32);
		memcpy(tbuf + SWITCHX_ADDR32, in_out_buf, len);

		j = 0;
		while(j < len) {
			if ((len - j) >= 32) {
				dprintk(					"%s(%d) 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
					"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
					"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
					"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		 	 		__FUNCTION__, __LINE__,
		  			*(tbuf+j), *(tbuf+j+1), *(tbuf+j+2), *(tbuf+j+3),
		  			*(tbuf+j+4), *(tbuf+j+5), *(tbuf+j+6), *(tbuf+j+7),
		  			*(tbuf+j+8), *(tbuf+j+9), *(tbuf+j+10), *(tbuf+j+11),
		  			*(tbuf+j+12), *(tbuf+j+13), *(tbuf+j+14), *(tbuf+j+15),
		  			*(tbuf+j+16), *(tbuf+j+17), *(tbuf+j+18), *(tbuf+j+19),
		  			*(tbuf+j+20), *(tbuf+j+21), *(tbuf+j+22), *(tbuf+j+23),
		  			*(tbuf+j+24), *(tbuf+j+25), *(tbuf+j+26), *(tbuf+j+27),
		  			*(tbuf+j+28), *(tbuf+j+29), *(tbuf+j+30), *(tbuf+j+31));
			}
			else {
				dprintk(					"%s(%d) 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
					"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
					"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		 	 		__FUNCTION__, __LINE__,
		  			*(tbuf+j), *(tbuf+j+1), *(tbuf+j+2), *(tbuf+j+3),
		  			*(tbuf+j+4), *(tbuf+j+5), *(tbuf+j+6), *(tbuf+j+7),
		  			*(tbuf+j+8), *(tbuf+j+9), *(tbuf+j+10), *(tbuf+j+11),
		  			*(tbuf+j+12), *(tbuf+j+13), *(tbuf+j+14), *(tbuf+j+15),
		  			*(tbuf+j+16), *(tbuf+j+17), *(tbuf+j+18), *(tbuf+j+19),
		  			*(tbuf+j+20), *(tbuf+j+21), *(tbuf+j+22), *(tbuf+j+23),
		  			*(tbuf+j+24));
			}
			j += 32;
		}
		dprintk(				"%s(%d) 0x%02x bytes to be written\n",
				__FUNCTION__, __LINE__, len);


        	buf = tbuf;
		off = data->mb_offset_in;
        	for (i = 0; i < num; i++) {
			chunk_size = (len > LPCI2C_BLOCK_MAX) ? LPCI2C_BLOCK_MAX : len;

			rc = translate_offset_addr(client->addr, off, chunk_size, SWITCHX_ADDR32,
							msgbuf, msgs, buf, 0);
			rc = i2c_transfer(client->adapter, msgs, 1);

			if (rc < 0)
				return -EIO;
			buf += chunk_size;
			off += chunk_size;
			len -= chunk_size;
			memset(msgs, 0, 2*sizeof(struct i2c_msg));
        	}
	}

	/* Write out Command Interface Register */
        reg_cir->token = (u16)client->adapter->nr; /* Put bus id as token */
	dprintk(		"%s(%d) write cir at 0x%08x go=%x stat=%x tok=%d opcode=%d opmod=%d event=%d inmod %x par %x %x %x %x\n",
		__FUNCTION__, __LINE__, switchx_cfg.reg_cir_offset,
		reg_cir->go, reg_cir->status, reg_cir->token,
		reg_cir->opcode, reg_cir->opcode_modifier, reg_cir->event, reg_cir->input_modifier,
		reg_cir->in_param_h, reg_cir->in_param_l, reg_cir->out_param_h, reg_cir->out_param_l);
	memset(msgs, 0, sizeof(struct i2c_msg));
	memset(tbuf, 0, size + SWITCHX_ADDR32);

	memset(icr_reg_buf, 0, 28);
	icr_reg_buf[2] = cpu_to_be32(reg_cir->input_modifier);
	icr_reg_buf[5] = cpu_to_be32(reg_cir->token << 16);
	icr_reg_buf[6] = cpu_to_be32((reg_cir->opcode_modifier << OPMOD_SHIFT) | reg_cir->opcode |
					(reg_cir->event << EVENT_BIT));

	dprintk(		  "%s(%d) icr_reg_buf[2]=0x%08x icr_reg_buf[5]=0x%08x icr_reg_buf[6]=0x%08x\n",
		  __FUNCTION__, __LINE__, icr_reg_buf[2], icr_reg_buf[5], icr_reg_buf[6]);

	memcpy(tbuf + SWITCHX_ADDR32, (u8 *)&icr_reg_buf, 28);
	translate_offset_addr(client->addr, SX_BAR0_OFFSET_SECOND, 28, SWITCHX_ADDR32,
				msgbuf, msgs, tbuf, 0);
	out_len = 28;

	rc = i2c_transfer(client->adapter, msgs, 1);

	dprintk(			"%s(%d) 0x%02x bytes has been written at 0x%08x to HCR\n",
			__FUNCTION__, __LINE__, out_len, SX_BAR0_OFFSET_SECOND);
	j = 0;
	while(j < out_len) {
		if ((out_len - j) >= 32) {
			dprintk(				"%s(%d) 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		 	 	__FUNCTION__, __LINE__,
		  		*(tbuf+j), *(tbuf+j+1), *(tbuf+j+2), *(tbuf+j+3),
		  		*(tbuf+j+4), *(tbuf+j+5), *(tbuf+j+6), *(tbuf+j+7),
		  		*(tbuf+j+8), *(tbuf+j+9), *(tbuf+j+10), *(tbuf+j+11),
		  		*(tbuf+j+12), *(tbuf+j+13), *(tbuf+j+14), *(tbuf+j+15),
		  		*(tbuf+j+16), *(tbuf+j+17), *(tbuf+j+18), *(tbuf+j+19),
		  		*(tbuf+j+20), *(tbuf+j+21), *(tbuf+j+22), *(tbuf+j+23),
		  		*(tbuf+j+24), *(tbuf+j+25), *(tbuf+j+26), *(tbuf+j+27),
		  		*(tbuf+j+28), *(tbuf+j+29), *(tbuf+j+30), *(tbuf+j+31));
		}
		else {
			dprintk(				"%s(%d) 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		 	 	__FUNCTION__, __LINE__,
		  		*(tbuf+j), *(tbuf+j+1), *(tbuf+j+2), *(tbuf+j+3),
		  		*(tbuf+j+4), *(tbuf+j+5), *(tbuf+j+6), *(tbuf+j+7),
		  		*(tbuf+j+8), *(tbuf+j+9), *(tbuf+j+10), *(tbuf+j+11),
		  		*(tbuf+j+12), *(tbuf+j+13), *(tbuf+j+14), *(tbuf+j+15),
		  		*(tbuf+j+16), *(tbuf+j+17), *(tbuf+j+18), *(tbuf+j+19),
		  		*(tbuf+j+20), *(tbuf+j+21), *(tbuf+j+22), *(tbuf+j+23),
		  		*(tbuf+j+24));

		}
		j += 32;
	}

	if (rc < 0)
		return -EIO;

	/* Write out Command Interface Register GO bit - do it by separate transaction */
        reg_cir->go = CIR_GO_HW;                   /* Go (Software ownership for the CIR) */
	dprintk(		"%s(%d) write cir at 0x%08x go=%x stat=%x tok=%d opcode=%d opmod=%d event=%d param %x %x %x %x\n",
		__FUNCTION__, __LINE__, switchx_cfg.reg_cir_offset + STATUS_OFFSET,
		reg_cir->go, reg_cir->status, reg_cir->token, reg_cir->opcode,
		reg_cir->opcode_modifier, reg_cir->event, reg_cir->in_param_h, reg_cir->in_param_l,
		reg_cir->out_param_h, reg_cir->out_param_l);

	memset(msgs, 0, sizeof(struct i2c_msg));
	memset(tbuf, 0, size + SWITCHX_ADDR32);

        icr_reg = cpu_to_be32((1 << GO_BIT) | (reg_cir->event?(1 << EVENT_BIT):0) |
                                (reg_cir->opcode_modifier << OPMOD_SHIFT) | reg_cir->opcode);
	memcpy(tbuf + SWITCHX_ADDR32, (u8 *)&icr_reg, 4);
	translate_offset_addr(client->addr, switchx_cfg.reg_cir_offset + STATUS_OFFSET, 4, SWITCHX_ADDR32,
				msgbuf, msgs, tbuf, 0);

	dprintk(		  "%s(%d) write GO bit at offset=0x%08x size=4 (0x%08x) buf=0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		  __FUNCTION__, __LINE__, switchx_cfg.reg_cir_offset + STATUS_OFFSET, icr_reg,
		  *(tbuf), *(tbuf+1), *(tbuf+2), *(tbuf+3), *(tbuf+4), *(tbuf+5), *(tbuf+6), *(tbuf+7));

	rc = i2c_transfer(client->adapter, msgs, 1);
	if (rc < 0)
		return -EIO;

	return rc;
}

static int query_fw_cmd_finish_imm(struct i2c_client *client,
			           struct switchx_data *data,
			           struct reg_cir_layout *reg_cir,
			           int len, int offset, u8 *in_out_buf)
{
	struct i2c_msg msgs[2];
	u8 msgbuf[SWITCHX_ADDR32];
	struct reg_cir_layout _reg_cir;
	int rc;

	switch (reg_cir->status) {
		case SX_STAT_OK:
			break;
		case SX_STAT_INTERNAL_ERR:
			printk(KERN_INFO "%s: Internal bus error occurred while processing command, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case SX_STAT_BAD_OP:
			printk(KERN_INFO "%s: Operation not supported, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case SX_STAT_BAD_SYS_STATE:
			printk(KERN_INFO "%s: Switch is not initialized, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case SX_STAT_EXCEED_LIM:
			printk(KERN_INFO "%s: Switch is not initialized, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	default:
			printk(KERN_INFO "%s: Unexpected status %x is reported, client %s addr 0x%02x\n",
				__FUNCTION__, reg_cir->status, client->name, client->addr);
			// return -EIO;
	}

        /* Read all register, in spite only out_param_h and out_param_l are necessary anyway it happens once */
	memset(&_reg_cir, 0, sizeof(struct reg_cir_layout));
	memset(msgs, 0, 2*sizeof(struct i2c_msg));

	rc = translate_offset_addr(client->addr, switchx_cfg.reg_cir_offset,
				sizeof(struct reg_cir_layout),
				SWITCHX_ADDR32, msgbuf, msgs, (u8 *)&_reg_cir, I2C_M_RD);

	dprintk(		"%s len=%d(%d) 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x " \
		"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		__FUNCTION__, len, (int)sizeof(struct reg_cir_layout),
		*in_out_buf, *(in_out_buf+1), *(in_out_buf+2), *(in_out_buf+3),
		*(in_out_buf+4), *(in_out_buf+5), *(in_out_buf+6), *(in_out_buf+7),
		*(in_out_buf+8), *(in_out_buf+9), *(in_out_buf+10), *(in_out_buf+11),
		*(in_out_buf+12), *(in_out_buf+13), *(in_out_buf+14), *(in_out_buf+15));

	rc = i2c_transfer(client->adapter, msgs, 2);

	dprintk(		"%s  cir  inmod=%x o=%x stat=%x tok=%d opcode=%d opmod=%d event=%d param %x %x %x %x\n",
		__FUNCTION__, _reg_cir.input_modifier, _reg_cir.go, _reg_cir.status, _reg_cir.token,
		_reg_cir.opcode, _reg_cir.opcode_modifier, _reg_cir.event, _reg_cir.in_param_h,
		_reg_cir.in_param_l, _reg_cir.out_param_h, _reg_cir.out_param_l);

	memcpy(in_out_buf, &_reg_cir.out_param_h, len);

	dprintk(		"%s len=%d(%d) 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x " \
		"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		__FUNCTION__, len, (int)sizeof(struct reg_cir_layout),
		*in_out_buf, *(in_out_buf+1), *(in_out_buf+2), *(in_out_buf+3),
		*(in_out_buf+4), *(in_out_buf+5), *(in_out_buf+6), *(in_out_buf+7),
		*(in_out_buf+8), *(in_out_buf+9), *(in_out_buf+10), *(in_out_buf+11),
		*(in_out_buf+12), *(in_out_buf+13), *(in_out_buf+14), *(in_out_buf+15));

	return rc;
}

static int _cmd_finish_mbox(struct i2c_client *client,
			    struct switchx_data *data,
			    struct reg_cir_layout *reg_cir,
			    int len, int offset, u8 *in_out_buf)
{
	int rc = -EIO;
	struct i2c_msg msgs[2];
	u8 msgbuf[SWITCHX_ADDR32];

	switch (reg_cir->status) {
		case SX_STAT_OK:
			break;
		case SX_STAT_INTERNAL_ERR:
			printk(KERN_INFO "%s: Internal bus error occurred while processing command, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
		case SX_STAT_BAD_PARAM:
			printk(KERN_INFO "%s: Bad parameters, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case SX_STAT_BAD_OP:
			printk(KERN_INFO "%s: Operation not supported, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case SX_STAT_BAD_SYS_STATE:
			printk(KERN_INFO "%s: Bad system state, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case SX_STAT_EXCEED_LIM:
			printk(KERN_INFO "%s: Exceeds device limits, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	default:
			printk(KERN_INFO "%s: Unexpected status is reported, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
	}

	memset(msgs, 0, 2*sizeof(struct i2c_msg));
	dprintk(		"%s  cir read data at offset = 0x%08x width = %d len =%d param %x %x %x %x\n",
		__FUNCTION__, data->mb_offset_out + offset, SWITCHX_ADDR32, len, reg_cir->in_param_h,
		reg_cir->in_param_l, reg_cir->out_param_h, reg_cir->out_param_l);

	if (len <= LPCI2C_BLOCK_MAX) {
		rc = translate_offset_addr(client->addr, data->mb_offset_out + offset, len, SWITCHX_ADDR32,
					msgbuf, msgs, in_out_buf, I2C_M_RD);
		rc = i2c_transfer(client->adapter, msgs, 2);
	}
	else {
        	int num = len/LPCI2C_BLOCK_MAX;
        	u8 *tbuf = in_out_buf;
        	int off = data->mb_offset_out + offset;
        	int chunk_size = (len > LPCI2C_BLOCK_MAX) ? LPCI2C_BLOCK_MAX : len;
        	int i;

        	if (len%LPCI2C_BLOCK_MAX)
                	num++;
        	for (i = 0; i < num; i++) {
			chunk_size = (len > LPCI2C_BLOCK_MAX) ? LPCI2C_BLOCK_MAX : len;

			dprintk(				"%s i=%d (num=%d) chunk_size=%d off=0x%08x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x " \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n " \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x " \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
				__FUNCTION__, i, num, chunk_size, off,
				*(in_out_buf+16*i), *(in_out_buf+1+16*i), *(in_out_buf+2+16*i), *(in_out_buf+3+16*i),
				*(in_out_buf+4+16*i), *(in_out_buf+5+16*i), *(in_out_buf+6+16*i), *(in_out_buf+7+16*i),
				*(in_out_buf+8+16*i), *(in_out_buf+9+16*i), *(in_out_buf+10+16*i), *(in_out_buf+11+16*i),
				*(in_out_buf+12+16*i), *(in_out_buf+13+16*i), *(in_out_buf+14+16*i), *(in_out_buf+15+16*i),
				*(in_out_buf+16+16*i), *(in_out_buf+17+16*i), *(in_out_buf+18+16*i), *(in_out_buf+19+16*i),
				*(in_out_buf+20+16*i), *(in_out_buf+21+16*i), *(in_out_buf+22+16*i), *(in_out_buf+23+16*i),
				*(in_out_buf+24+16*i), *(in_out_buf+25+16*i), *(in_out_buf+26+16*i), *(in_out_buf+27+16*i),
				*(in_out_buf+28+16*i), *(in_out_buf+29+16*i), *(in_out_buf+30+16*i), *(in_out_buf+31+16*i));

			rc = translate_offset_addr(client->addr, off, chunk_size, SWITCHX_ADDR32,
							msgbuf, msgs, tbuf, I2C_M_RD);
			rc = i2c_transfer(client->adapter, msgs, 2);

			dprintk(				"%s i=%d (num=%d) chunk_size=%d off=0x%08x rc=%d msgbuf 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x " \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n " \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x " \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
				__FUNCTION__, i, num, chunk_size, off, rc,
				*(msgbuf), *(msgbuf+1), *(msgbuf+2), *(msgbuf+3),
				*(in_out_buf+16*i), *(in_out_buf+1+16*i), *(in_out_buf+2+16*i), *(in_out_buf+3+16*i),
				*(in_out_buf+4+16*i), *(in_out_buf+5+16*i), *(in_out_buf+6+16*i), *(in_out_buf+7+16*i),
				*(in_out_buf+8+16*i), *(in_out_buf+9+16*i), *(in_out_buf+10+16*i), *(in_out_buf+11+16*i),
				*(in_out_buf+12+16*i), *(in_out_buf+13+16*i), *(in_out_buf+14+16*i), *(in_out_buf+15+16*i),
				*(in_out_buf+16+16*i), *(in_out_buf+17+16*i), *(in_out_buf+18+16*i), *(in_out_buf+19+16*i),
				*(in_out_buf+20+16*i), *(in_out_buf+21+16*i), *(in_out_buf+22+16*i), *(in_out_buf+23+16*i),
				*(in_out_buf+24+16*i), *(in_out_buf+25+16*i), *(in_out_buf+26+16*i), *(in_out_buf+27+16*i),
				*(in_out_buf+28+16*i), *(in_out_buf+29+16*i), *(in_out_buf+30+16*i), *(in_out_buf+31+16*i));

			if (rc < 0)
				return -EIO;
			tbuf += chunk_size;
			off += chunk_size;
			len -= chunk_size;
			memset(msgs, 0, 2*sizeof(struct i2c_msg));
        	}
	}

	if (rc < 0)
		return -EIO;
	return 0;
}

static int query_fw_cmd(struct device *dev, enum query_fw_opcode_mod opmod,
			enum query_fw_input_mod inmod, u8 event,
                        int len, int offset, u8 *in_out_buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct switchx_data *data = i2c_get_clientdata(client);
	struct reg_cir_layout *reg_cir = &data->reg_cir;
	int rc = 0;

        memset(reg_cir, 0, sizeof(struct reg_cir_layout));
        reg_cir->opcode = SX_QUERY_FW_OP;
        reg_cir->opcode_modifier = opmod;
        reg_cir->event = event;
        reg_cir->input_modifier = inmod;

	dprintk(         	"%s(%d) offset=0x%08x op=%x event=%d opmod=%x inmod=%x stat=%x tok=%x data=%p reg_cir=%p len=%d\n",
         	__FUNCTION__, __LINE__, offset, reg_cir->opcode, reg_cir->opcode_modifier, reg_cir->event,
        	reg_cir->input_modifier, reg_cir->status, reg_cir->token, data, reg_cir, len);

        switch (opmod) {
        case SX_MB_OUTPUT: /* An output mailbox is returned, struct query_fw_output_mbox_layout */
        	if ((rc = acquire_go_bit(client, data, (void *)reg_cir, len, offset, in_out_buf,
                                         _cmd_start, _cmd_finish_mbox)) < 0)
			return rc;
        	break;
        case SX_IMM_OUTPUT_1: /* The output parameter is an immediate value, struct output_param_opmode1 */
        	if ((rc = acquire_go_bit(client, data, (void *)reg_cir, len, offset, in_out_buf,
                                         _cmd_start, query_fw_cmd_finish_imm)) < 0)
			return rc;
        	break;
        case SX_IMM_OUTPUT_2: /* The output parameter is an immediate values, struct output_param_opmode2 */
		rc = -ENOTSUPP;
        	break;
        }

        return (rc > 0) ? 0 : -1;
}

static int access_reg_cmd_finish_get(struct i2c_client *client,
				     struct switchx_data *data,
				     struct reg_cir_layout *reg_cir,
				     int len, int offset, u8 *in_out_buf)
{
	struct access_reg_layout *acc_reg = (struct access_reg_layout *)in_out_buf;
	struct i2c_msg msgs[2];
	u8 msgbuf[SWITCHX_ADDR32];
	int j, out_len = len;
	int rc;
	unsigned long orig_jiffies;
	int try = 0;
        int num = len/LPCI2C_BLOCK_MAX;
        int chunk_size = (len > LPCI2C_BLOCK_MAX) ? LPCI2C_BLOCK_MAX : len;
	u8 *pbuf = in_out_buf;
        int off = data->mb_offset_out + offset;
        int i;
	u8 status;
	u8 go;

        if (len%LPCI2C_BLOCK_MAX)
                num++;
        icr_reg_read(client, *((u32 *)((u8 *)(reg_cir) + CIR_STATUS_OFF)), &status, &go);


	switch (status) {
		case SX_STAT_OK:
			break;
		case SX_STAT_INTERNAL_ERR:
			printk(KERN_INFO "%s: Internal bus error occurred while processing command, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case SX_STAT_BAD_SYS_STATE:
			printk(KERN_INFO "%s: Switch is not initialized, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case SX_STAT_BAD_OP:
			printk(KERN_INFO "%s: Operation not supported, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	default:
			printk(KERN_INFO "%s: Unexpected status 0x%02x is reported, client %s addr 0x%02x\n",
				__FUNCTION__, status, client->name, client->addr);
			return -EIO;
	}

	/* ACCESS_REG commands uses mailboxes. In order to use mailboxes through the i2c, special area is
	   reserved on the i2c address space that can be used for input and output mailboxes. Such mailboxes
	   are called Local Mailboxes. When using a local mailbox, software should specify 0 as the
	   Input/Output parameters. The location of the Local Mailbox addresses on the i2c space can be retrieved
	   through the QUERY_FW command*/
	memset(msgs, 0, 2*sizeof(struct i2c_msg));


        for (i = 0; i < num; i++) {
		chunk_size = (len > LPCI2C_BLOCK_MAX) ? LPCI2C_BLOCK_MAX : len;
		rc = translate_offset_addr(client->addr, off, chunk_size, SWITCHX_ADDR32,
						msgbuf, msgs, pbuf, I2C_M_RD);

		orig_jiffies = jiffies;
		for (try = 0; try <= access_rtry; try++) {
			rc = i2c_transfer(client->adapter, msgs, 2);
			if (rc > 0)
				break;
		}
		data->retry_cntr += try;

		pbuf += chunk_size;
		off += chunk_size;
		len -= chunk_size;
		memset(msgs, 0, 2*sizeof(struct i2c_msg));
        }

	ASIC_GET_BUF(acc_reg->oper_tlv.status, in_out_buf, DR_STATUS_OFFSET);
	acc_reg->oper_tlv.status &= 0x7f;

	dprintk(			"%s(%d) 0x%02x bytes has been read\n",
			__FUNCTION__, __LINE__, out_len);
	j = 0;
	while(j < out_len) {
		if ((out_len - j) >= 32) {
			dprintk(				"%s(%d) 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		 	 	__FUNCTION__, __LINE__,
		  		*(in_out_buf+j), *(in_out_buf+j+1), *(in_out_buf+j+2), *(in_out_buf+j+3),
		  		*(in_out_buf+j+4), *(in_out_buf+j+5), *(in_out_buf+j+6), *(in_out_buf+j+7),
		  		*(in_out_buf+j+8), *(in_out_buf+j+9), *(in_out_buf+j+10), *(in_out_buf+j+11),
		  		*(in_out_buf+j+12), *(in_out_buf+j+13), *(in_out_buf+j+14), *(in_out_buf+j+15),
		  		*(in_out_buf+j+16), *(in_out_buf+j+17), *(in_out_buf+j+18), *(in_out_buf+j+19),
		  		*(in_out_buf+j+20), *(in_out_buf+j+21), *(in_out_buf+j+22), *(in_out_buf+j+23),
		  		*(in_out_buf+j+24), *(in_out_buf+j+25), *(in_out_buf+j+26), *(in_out_buf+j+27),
		  		*(in_out_buf+j+28), *(in_out_buf+j+29), *(in_out_buf+j+30), *(in_out_buf+j+31));
		}
		else {
			dprintk(				"%s(%d) 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		 	 	__FUNCTION__, __LINE__,
		  		*(in_out_buf+j), *(in_out_buf+j+1), *(in_out_buf+j+2), *(in_out_buf+j+3),
		  		*(in_out_buf+j+4), *(in_out_buf+j+5), *(in_out_buf+j+6), *(in_out_buf+j+7),
		  		*(in_out_buf+j+8), *(in_out_buf+j+9), *(in_out_buf+j+10), *(in_out_buf+j+11),
		  		*(in_out_buf+j+12), *(in_out_buf+j+13), *(in_out_buf+j+14), *(in_out_buf+j+15),
		  		*(in_out_buf+j+16), *(in_out_buf+j+17), *(in_out_buf+j+18), *(in_out_buf+j+19),
		  		*(in_out_buf+j+20), *(in_out_buf+j+21), *(in_out_buf+j+22), *(in_out_buf+j+23),
		  		*(in_out_buf+j+24));
		}
		j += 32;
	}

	switch (acc_reg->oper_tlv.status) {
        	case OPER_TLV_STATUS_OK:
			break;
        	case OPER_TLV_STATUS_BUSY:
			printk(KERN_INFO "%s: Device is busy, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case OPER_TLV_STATUS_VER_NSUPP:
			printk(KERN_INFO "%s: Version not supported, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case OPER_TLV_STATUS_UNKNWN:
			printk(KERN_INFO "%s: Unknown TLV, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case OPER_TLV_STATUS_REG_NSUPP:
			printk(KERN_INFO "%s: Register not supported, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case OPER_TLV_STATUS_CLASS_NSUPP:
			printk(KERN_INFO "%s: Class not supported, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case OPER_TLV_STATUS_METHOD_NSUPP:
			printk(KERN_INFO "%s: Method not supported, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case OPER_TLV_STATUS_BAD_PARAM:
			printk(KERN_INFO "%s: Bad parameter, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case OPER_TLV_STATUS_RSRC_NAVAIL:
			printk(KERN_INFO "%s: Resource not available, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	case OPER_TLV_STATUS_MSG_ACK:
			printk(KERN_INFO "%s: Message Receipt Acknowledgement, client %s addr 0x%02x\n",
				__FUNCTION__, client->name, client->addr);
			return -EIO;
        	default:
			printk(KERN_INFO "%s: Unexpected TLV status 0x%02x is reported, client %s addr 0x%02x\n",
				__FUNCTION__, acc_reg->oper_tlv.status, client->name, client->addr);
			return -EIO;
	}

	dprintk(		"%s read input at 0x%08x status=%x\n",
		__FUNCTION__, data->mb_offset_out, acc_reg->oper_tlv.status);

	return 0;
}

static int access_reg_cmd(struct device *dev, u32 reg_id, int reg_len, enum oper_tlv_method method,
			  int len, int offset, u8 *in_out_buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct switchx_data *data = i2c_get_clientdata(client);
        struct operation_tlv oper_tlv;
        struct reg_access_tlv reg_accs_tlv;
        struct end_tlv e_tlv;
	u8 tlv_buf[sizeof(struct operation_tlv) + offsetof(struct reg_access_tlv, switchx_reg) +
		   SX_MAX_REG_SIZE + sizeof(struct end_tlv)];
	struct reg_cir_layout *reg_cir = &data->reg_cir;
	int off = 0;
	int offreg;
	int j;
	int rc = 0;
	u16 regtlv = 0;

	dprintk(               "%s input buf (len=%d) reg_len=%d reg_id=%x method=%d\n",
               __FUNCTION__, len, reg_len, reg_id, method);

	offreg = offsetof(struct reg_access_tlv, switchx_reg);
	memset(tlv_buf, 0, sizeof(struct operation_tlv) + offreg +
		   SX_MAX_REG_SIZE + sizeof(struct end_tlv));
	memset(&oper_tlv, 0, sizeof(struct operation_tlv));
	memset(&reg_accs_tlv, 0, offreg);
	memset(&e_tlv, 0, sizeof(struct end_tlv));
	memset(reg_cir, 0, sizeof(struct reg_cir_layout));

	/* Put Oper TLV  */
	SET_OPER_TLV(tlv_buf,
			SX_OPERATION_TLV,
			OPERATION_TLV_SIZE/4,
			DIRECT_ROUTE_NOT_SET,
			OPER_TLV_STATUS_OK,
			OPER_TLV_REQ,
			(u16)reg_id,
			(u8)method,
			(u8)SX_EMAD_REG_ACCESS_CLASS,
			data->transact_id++);
	/* Put Reg TLV constant part */
        regtlv = ((REG_TLV_TYPE << 11) | (reg_len/4 + REG_ACCESS_TLV_LEN));
	ASIC_SET_BUF(tlv_buf, regtlv, REG_TLV_OFFSET);
	/* Put Reg TLV variable part */

	/* Put End TLV */
        regtlv = (SX_END_OF_TLV << 11) | REG_TLV_END_LEN;
	ASIC_SET_BUF(tlv_buf, regtlv, (REG_TLV_LEN + REG_ACCESS_TLV_LEN) * 4 + reg_len);
	off = reg_len + (REG_TLV_LEN + REG_ACCESS_TLV_LEN) * 4;

	dprintk(               "%s input buf (len=%d) reg_len=%d reg_id=%x method=%d\n",
               __FUNCTION__, len, reg_len, reg_id, method);

	j = 0;
	while(j < len) {
		if ((len - j) >= 32) {
			dprintk(				"%s(%d) 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		 	 	__FUNCTION__, __LINE__,
		  		*(tlv_buf+j), *(tlv_buf+j+1), *(tlv_buf+j+2), *(tlv_buf+j+3),
		  		*(tlv_buf+j+4), *(tlv_buf+j+5), *(tlv_buf+j+6), *(tlv_buf+j+7),
		  		*(tlv_buf+j+8), *(tlv_buf+j+9), *(tlv_buf+j+10), *(tlv_buf+j+11),
		  		*(tlv_buf+j+12), *(tlv_buf+j+13), *(tlv_buf+j+14), *(tlv_buf+j+15),
		  		*(tlv_buf+j+16), *(tlv_buf+j+17), *(tlv_buf+j+18), *(tlv_buf+j+19),
		  		*(tlv_buf+j+20), *(tlv_buf+j+21), *(tlv_buf+j+22), *(tlv_buf+j+23),
		  		*(tlv_buf+j+24), *(tlv_buf+j+25), *(tlv_buf+j+26), *(tlv_buf+j+27),
		  		*(tlv_buf+j+28), *(tlv_buf+j+29), *(tlv_buf+j+30), *(tlv_buf+j+31));
		}
		else {
			dprintk(				"%s(%d) 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n" \
				"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		 	 	__FUNCTION__, __LINE__,
		  		*(tlv_buf+j), *(tlv_buf+j+1), *(tlv_buf+j+2), *(tlv_buf+j+3),
		  		*(tlv_buf+j+4), *(tlv_buf+j+5), *(tlv_buf+j+6), *(tlv_buf+j+7),
		  		*(tlv_buf+j+8), *(tlv_buf+j+9), *(tlv_buf+j+10), *(tlv_buf+j+11),
		  		*(tlv_buf+j+12), *(tlv_buf+j+13), *(tlv_buf+j+14), *(tlv_buf+j+15),
		  		*(tlv_buf+j+16), *(tlv_buf+j+17), *(tlv_buf+j+18), *(tlv_buf+j+19),
		  		*(tlv_buf+j+20), *(tlv_buf+j+21), *(tlv_buf+j+22), *(tlv_buf+j+23),
		  		*(tlv_buf+j+24));
		}
		j += 32;
	}

        reg_cir->opcode = SX_ACCESS_REG_OP;                           /* ACCESS_REG command opcode 0x40   */
        reg_cir->event = CIR_EVENT_NO_REPORT;

        switch (method) {
        case OPER_TLV_METHOD_QUERY:
                if ((rc = acquire_go_bit(client, data, (void *)reg_cir, off + REG_TLV_END_LEN * 4,
                                         offset, tlv_buf, _cmd_start, access_reg_cmd_finish_get)) != 0)
                        return rc;
                memcpy(in_out_buf, &tlv_buf[(REG_TLV_LEN + REG_ACCESS_TLV_LEN) * 4], len);
                break;
        case OPER_TLV_METHOD_WRITE:
                if ((rc = acquire_go_bit(client, data, (void *)reg_cir, off + REG_TLV_END_LEN * 4,
                                         offset, tlv_buf, _cmd_start, NULL)) != 0)
                        return rc;
                break;
        case OPER_TLV_METHOD_SEND:
        case OPER_TLV_METHOD_EVENT:
                rc = -ENOTSUPP;
                break;
        }

        return rc;
}

static ssize_t switchx_sw_reset(_FL_  struct kobject *kobj, struct bin_attribute *attr,
				  char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);

	return i2c_software_reset(&client->dev);
}

static ssize_t query_fw_read(_FL_  struct kobject *kobj, struct bin_attribute *attr,
				  char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	int rc;

	dev_dbg(&client->dev, "query_fw_read(p=%p, off=%lli, c=%zi)\n",
		buf, off, count);
	if (off % 4)
		return 0;
	if (off >= sizeof(struct query_fw_output_mbox_layout))
		return 0;
	if (off + count > sizeof(struct query_fw_output_mbox_layout))
		count = sizeof(struct query_fw_output_mbox_layout) - off;

	/* Read from mailbox */
	rc = query_fw_cmd(&client->dev, SX_MB_OUTPUT, SX_BAR0_OFF_NONE,
			  CIR_EVENT_REPORT, count, off, buf);

	return count;
}

static ssize_t show_mgir_reg(_FL_  struct kobject *kobj, struct bin_attribute *attr,
				  char *buf, loff_t off, size_t count)
{
#ifdef SXDBGMODE
	struct reg_mgir_layout *p_mgir_reg = (struct reg_mgir_layout *)buf;
#endif
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	struct reg_mgir_layout mgir_reg;
	int rc;

	memset(&mgir_reg, 0, sizeof(struct reg_mgir_layout));
	dev_dbg(&client->dev, "%s(p=%p, off=%lli, c=%zi)\n",
		__FUNCTION__, buf, off, count);
	if (off % 4)
		return -1;
	if (off >= sizeof(struct reg_mgir_layout))
		return -1;
	if (off + count > sizeof(struct reg_mgir_layout))
		count = sizeof(struct reg_mgir_layout) - off;

	rc = access_reg_cmd(&client->dev, MGIR_REG_ID,
				MGIR_REG_LEN * 4/* sizeof(struct reg_mgir_layout)*/,
				OPER_TLV_METHOD_QUERY, /*count*/
				(REG_TLV_LEN + REG_ACCESS_TLV_LEN + MGIR_REG_LEN + REG_TLV_END_LEN) * 4,
				off, buf);

	get_mgir_reg((u8 *)p_mgir_reg, &mgir_reg);

	dprintk(		"%s (%p) 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x " \
		"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		__FUNCTION__, p_mgir_reg, *buf, *(buf+1), *(buf+2), *(buf+3), *(buf+4), *(buf+5),
		*(buf+6), *(buf+7), *(buf+8), *(buf+9), *(buf+10), *(buf+11), *(buf+12), *(buf+13),
		*(buf+14), *(buf+15));
	dprintk(		"%s sw_major=0x%02x sw_minor=%04x sw_sub_minor=%04x dvfs=%04x rev=%04x dev_id=%04x uptime=%04x\n"
		"major=%04x minor=%04x sub_minor=%04x build_id=%04x month=%04x day=%04x year=%04x hour=%04x\n"
		"ini_file_version=%08x extended_major=%08x extended_minor=%08x extended_sub_minor=%08x\n",
		__FUNCTION__, mgir_reg.sw_info.major,
		mgir_reg.sw_info.minor, mgir_reg.sw_info.sub_minor,
		mgir_reg.hw_info.dvfs, mgir_reg.hw_info.device_hw_revision,
		mgir_reg.hw_info.device_id, mgir_reg.hw_info.uptime,
		mgir_reg.fw_info.major, mgir_reg.fw_info.minor, mgir_reg.fw_info.sub_minor,
		mgir_reg.fw_info.build_id, mgir_reg.fw_info.month, mgir_reg.fw_info.day,
		mgir_reg.fw_info.year, mgir_reg.fw_info.hour, mgir_reg.fw_info.ini_file_version,
		mgir_reg.fw_info.extended_major, mgir_reg.fw_info.extended_minor,
		mgir_reg.fw_info.extended_sub_minor);

	return count;
}

static int __i2c_match_addr(struct device *dev, void *addrp)
{
	struct i2c_client *client = i2c_verify_client(dev);
	int addr = *(int *)addrp;

        dprintk(
                "%s client=%p addr=%x\n", __FUNCTION__, client, addr);

	if (client && client->addr == addr)
		return 1;
	return 0;
}

static int get_dev_by_bus_id_and_addr(int bus_id, int addr, struct device *dev)
{
	struct device *adap_dev;
        struct device *client_dev;
        struct i2c_adapter *adap;

        dprintk(
                "%s bus=%d addr=%x\n", __FUNCTION__, bus_id, addr);
        adap = i2c_get_adapter(bus_id);
	if (adap == NULL) {
                printk(
			"%s adapter not found bus=%d addr=%x\n", __FUNCTION__, bus_id, addr);
		return -EINVAL;
        }
	adap_dev = &adap->dev;
	client_dev = device_find_child(adap_dev, &addr, __i2c_match_addr);
	if(!client_dev) {
                printk(
			"%s client not found bus=%d addr=%x\n", __FUNCTION__, bus_id, addr);
		return -EINVAL;
        }
	dev = client_dev;

	return 0;
}

struct i2c_client *sx_get_client_by_i2c_bus_and_addr(int bus_id, int addr)
{
	struct switchx_data *swd;

	list_for_each_entry(swd, &switchx_cfg.switchx_list, list) {
		if ((bus_id == swd->bus_id) && (addr == swd->i2c_dev_addr))
			return swd->client;
	}
	return NULL;
}

struct switchx_probe_failed * find_dev_probe_context(int bus_id)
{
	struct switchx_probe_failed *swpf, *next;

	list_for_each_entry_safe(swpf, next, &switchx_cfg.probe_failed_list, list) {
		if (bus_id == swpf->bus_id)
			return swpf; /* device on this bus failed probing */
	}
	return NULL;
}

int sx_software_reset(int i2c_dev_id)
{
	int bus_id = BUS_ID_FROM_ID(i2c_dev_id);
	u8 addr = ADDR_FROM_DEV_ID(i2c_dev_id);
	struct device *dev = NULL;

	if (!(get_dev_by_bus_id_and_addr(bus_id, addr, dev)))
		return -EINVAL;
	return i2c_software_reset(dev);
}
EXPORT_SYMBOL(sx_software_reset);

int i2c_get_fw_rev(int i2c_dev_id, u64* fw_rev)
{
	struct i2c_client *client;
	int bus_id = BUS_ID_FROM_ID(i2c_dev_id);
	u8 addr = ADDR_FROM_DEV_ID(i2c_dev_id);
	struct query_fw_output_mbox_layout query_fw;

	client = sx_get_client_by_i2c_bus_and_addr(bus_id, addr);
	if(!client)
		return -EUNATCH;

	if (!query_fw_cmd(&client->dev, SX_MB_OUTPUT, SX_BAR0_OFF_NONE, \
					  CIR_EVENT_REPORT, sizeof(struct query_fw_output_mbox_layout), 0, (u8*)&query_fw))
		return -EIO;

	*fw_rev = (u64)query_fw.fw_rev_major << 32 | (u64)query_fw.fw_rev_minor << 16 | query_fw.fw_rev_subminor;

	return 0;
}
EXPORT_SYMBOL(i2c_get_fw_rev);

int i2c_set_go_bit_stuck(int i2c_dev_id)
{
	struct switchx_data *data = NULL;
	struct i2c_client *client;
	int bus_id = BUS_ID_FROM_ID(i2c_dev_id);
	u8 addr = ADDR_FROM_DEV_ID(i2c_dev_id);

	client = sx_get_client_by_i2c_bus_and_addr(bus_id, addr);
	if(!client)
		return -EUNATCH;
	data = i2c_get_clientdata(client);
	if(!data)
		return -EINVAL;
	if(data->signature != SX_I2C_DEV_SIGNATURE)
		return -EINVAL;
	data->go_bit_status = GO_BIT_STAT_STUCK;
	return 0;
}

int i2c_get_local_mbox(int i2c_dev_id, u32 *mb_size_in, u32 *mb_offset_in, u32 *mb_size_out, u32 *mb_offset_out)
{
	struct switchx_data *data;
	struct i2c_client *client;
	int bus_id = BUS_ID_FROM_ID(i2c_dev_id);
	u8 addr = ADDR_FROM_DEV_ID(i2c_dev_id);

	dprintk(		"%s i2c_dev_id=%x\n", __FUNCTION__, i2c_dev_id);

        if (!mb_size_in || !mb_offset_in || !mb_size_out || !mb_offset_out)
		return -EINVAL;
	client = sx_get_client_by_i2c_bus_and_addr(bus_id, addr);
	if(!client)
		return -EUNATCH;

	data = i2c_get_clientdata(client);
	if(!data)
		return -EINVAL;

	*mb_size_in = data->mb_size_in;
	*mb_offset_in = data->mb_offset_in;
	*mb_size_out = data->mb_size_out;
	*mb_offset_out = data->mb_offset_out;

	dprintk("%s mb size=%x off=0x%08x out mb size=%x off=0x%08x\n",
		__func__, data->mb_size_in, data->mb_offset_in, data->mb_size_out, data->mb_offset_out);

	return 0;
}
EXPORT_SYMBOL(i2c_get_local_mbox);

int i2c_enforce(int i2c_dev_id)
{
	return 0;
}
EXPORT_SYMBOL(i2c_enforce);

int i2c_release(int i2c_dev_id)
{
	return 0;
}
EXPORT_SYMBOL(i2c_release);

int i2c_write(int i2c_dev_id, int offset, int len, u8 *in_out_buf)
{
	struct switchx_data *data;
	struct i2c_client *client;
	int bus_id = BUS_ID_FROM_ID(i2c_dev_id);
	u8 addr = ADDR_FROM_DEV_ID(i2c_dev_id);
	struct i2c_msg msgs[1];
	u8 msgbuf[SWITCHX_ADDR32];
	u8 tbuf[MAX_I2C_BUFF_SIZE + 64];
	unsigned long orig_jiffies;
	int try = 0;
	int rc = 0;
        int num = len/LPCI2C_BLOCK_MAX;
        int off = offset;
        int chunk_size = (len > LPCI2C_BLOCK_MAX) ? LPCI2C_BLOCK_MAX : len;
	u8 *pbuf = tbuf;
        int i;

        if (len%LPCI2C_BLOCK_MAX)
                num++;

	client = sx_get_client_by_i2c_bus_and_addr(bus_id, addr);
	dprintk(		"%s i2c_dev_id=%x bus=%d addr=%x client=%p\n", __FUNCTION__, i2c_dev_id, bus_id, addr, client);
	if(!client)
		return -EUNATCH;

	data = i2c_get_clientdata(client);
	if(!data)
		return -EINVAL;

	memset(msgs, 0, sizeof(struct i2c_msg));
	memset(tbuf, 0, len + SWITCHX_ADDR32);
	memcpy(tbuf + SWITCHX_ADDR32, in_out_buf, len);

        for (i = 0; i < num; i++) {
		chunk_size = (len > LPCI2C_BLOCK_MAX) ? LPCI2C_BLOCK_MAX : len;
		rc = translate_offset_addr(client->addr, off, chunk_size, SWITCHX_ADDR32,
						msgbuf, msgs, pbuf, 0);

	orig_jiffies = jiffies;
	for (try = 0; try <= access_rtry; try++) {
		rc = i2c_transfer(client->adapter, msgs, 1);
		if (rc > 0)
			break;
		}
		data->retry_cntr += try;
		pbuf += chunk_size;
		off += chunk_size;
		len -= chunk_size;
		memset(msgs, 0, 2*sizeof(struct i2c_msg));
        }
	dprintk(		"%s i2c_dev_id=%x bus=%d addr=%x client=%p\n", __FUNCTION__, i2c_dev_id, bus_id, addr, client);

	return (rc > 0) ? 0 : rc;
}
EXPORT_SYMBOL(i2c_write);

int i2c_read(int i2c_dev_id, int offset, int len, u8 *in_out_buf)
{
	struct switchx_data *data;
	struct i2c_client *client;
	int bus_id = BUS_ID_FROM_ID(i2c_dev_id);
	u8 addr = ADDR_FROM_DEV_ID(i2c_dev_id);
	struct i2c_msg msgs[2];
	u8 msgbuf[SWITCHX_ADDR32];
	unsigned long orig_jiffies;
	int try = 0;
	int rc = 0;
        int num = len/LPCI2C_BLOCK_MAX;
        int off = offset;
        int chunk_size = (len > LPCI2C_BLOCK_MAX) ? LPCI2C_BLOCK_MAX : len;
	u8 *pbuf = in_out_buf;
        int i;

        if (len%LPCI2C_BLOCK_MAX)
                num++;

	client = sx_get_client_by_i2c_bus_and_addr(bus_id, addr);
	dprintk(		"%s i2c_dev_id=%x bus=%d addr=%x client=%p\n", __FUNCTION__, i2c_dev_id, bus_id, addr, client);
	if(!client)
		return -EUNATCH;

	data = i2c_get_clientdata(client);
	if(!data)
		return -EINVAL;

	memset(msgs, 0, 2*sizeof(struct i2c_msg));

        for (i = 0; i < num; i++) {
		chunk_size = (len > LPCI2C_BLOCK_MAX) ? LPCI2C_BLOCK_MAX : len;
		rc = translate_offset_addr(client->addr, off, chunk_size, SWITCHX_ADDR32,
						msgbuf, msgs, pbuf, I2C_M_RD);

		orig_jiffies = jiffies;
		for (try = 0; try <= access_rtry; try++) {
			rc = i2c_transfer(client->adapter, msgs, 2);
			if (rc > 0)
				break;
		}
		data->retry_cntr += try;
		pbuf += chunk_size;
		off += chunk_size;
		len -= chunk_size;
		memset(msgs, 0, 2*sizeof(struct i2c_msg));
        }

	return (rc > 0) ? 0 : rc;
}
EXPORT_SYMBOL(i2c_read);

int i2c_write_dword(int i2c_dev_id, int offset, u32 val)
{
	struct switchx_data *data;
	struct i2c_client *client;
	int bus_id = BUS_ID_FROM_ID(i2c_dev_id);
	u8 addr = ADDR_FROM_DEV_ID(i2c_dev_id);
	struct i2c_msg msgs[1];
	u8 msgbuf[SWITCHX_ADDR32];
	u8 tbuf[SWITCHX_ADDR32 + 4];
	unsigned long orig_jiffies;
	int try;
	int rc = 0;

	client = sx_get_client_by_i2c_bus_and_addr(bus_id, addr);
	dprintk( "%s i2c_dev_id=%x bus=%d addr=%x client=%p offset=0x%08x\n",
		    __FUNCTION__, i2c_dev_id, bus_id, addr, client, offset);
	if(!client)
		return -EUNATCH;

	data = i2c_get_clientdata(client);
	if(!data)
		return -EINVAL;

	memset(msgs, 0, sizeof(struct i2c_msg));
	memset(tbuf, 0, SWITCHX_ADDR32 + 4);
	memcpy(tbuf + SWITCHX_ADDR32, (u8 *)&val, 4);

	translate_offset_addr(client->addr, offset, 4, SWITCHX_ADDR32,
				msgbuf, msgs, tbuf, 0);

	orig_jiffies = jiffies;
	for (try = 0; try <= access_rtry; try++) {
		rc = i2c_transfer(client->adapter, msgs, 1);
		if (rc > 0)
			break;
	}
	data->retry_cntr += try;
	dprintk(		"%s rc=%d try=%d\n", __FUNCTION__, rc, try);

	return (rc > 0) ? 0 : rc;
}
EXPORT_SYMBOL(i2c_write_dword);

int i2c_read_dword(int i2c_dev_id, int offset, u32 *val)
{
	struct switchx_data *data;
	struct i2c_client *client;
	int bus_id = BUS_ID_FROM_ID(i2c_dev_id);
	u8 addr = ADDR_FROM_DEV_ID(i2c_dev_id);
	struct i2c_msg msgs[2];
	u8 msgbuf[SWITCHX_ADDR32];
	unsigned long orig_jiffies;
	int try;
	int rc = 0;

	client = sx_get_client_by_i2c_bus_and_addr(bus_id, addr);
	dprintk( "%s i2c_dev_id=%x bus=%d addr=%x client=%p offset=0x%08x\n",
		  __FUNCTION__, i2c_dev_id, bus_id, addr, client, offset);
	if(!client)
		return -EUNATCH;

	data = i2c_get_clientdata(client);
	if(!data)
		return -EINVAL;

	memset(msgs, 0, 2*sizeof(struct i2c_msg));
	translate_offset_addr(client->addr, offset, 4, SWITCHX_ADDR32,
				msgbuf, msgs, (u8 *)val, I2C_M_RD);

	orig_jiffies = jiffies;
	for (try = 0; try <= access_rtry; try++) {
		rc = i2c_transfer(client->adapter, msgs, 2);



memcpy(msgbuf, val, 4);
dprintk("%s(%d) 0x%02x 0x%02x 0x%02x  0x%02x\n",
		 	 __FUNCTION__, __LINE__,
		  	*(msgbuf), *(msgbuf+1), *(msgbuf+2), *(msgbuf+3));
if ( *(msgbuf) == 0x30 ) {
*(msgbuf) = 0;
memcpy(val, msgbuf, 4);
}



		if (rc > 0) {
			break;
		}
		/* This API is used for access to HCR register. In case this
                   transaction failed - status is to be provided, since such
                   failure in certain scenario can cause i2c bus stuck */
	}
	data->retry_cntr += try;
	dprintk(		"%s rc=%d try=%d\n", __FUNCTION__, rc, try);

	return (rc > 0) ? 0 : rc;
}
EXPORT_SYMBOL(i2c_read_dword);

static struct bin_attribute query_fm_attr = {
	.attr = {
		.name = "queryfw",
		.mode = S_IRUGO,
	},
	.size = sizeof(struct query_fw_output_mbox_layout),
	.read = query_fw_read,
};

static struct bin_attribute mgir_reg_attr = {
	.attr = {
		.name = "mgir_reg",
		.mode = S_IRUGO,
	},
	.size = sizeof(struct reg_mgir_layout),
	.read = show_mgir_reg,
};

static struct bin_attribute sw_reset_attr = {
	.attr = {
		.name = "sw_reset",
		.mode = S_IWUSR,
	},
	.size = sizeof(struct reg_sw_reset_layout),
	.write = switchx_sw_reset,
};

/* Return 0 if detection is successful, -ENODEV otherwise */
static int switchx_detect(struct i2c_client *client,
			 struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA
				     | I2C_FUNC_SMBUS_WORD_DATA
				     | I2C_FUNC_SMBUS_WRITE_BYTE))
		return -ENODEV;

	strlcpy(info->type, "switchx", I2C_NAME_SIZE);

	return 0;
}

static int switchx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct switchx_data *data = NULL;
	struct output_param_opmode1 output_param;
	struct i2c_adapter *adap = NULL;
	struct query_fw_output_mbox_layout query_fw;
	int rc = -1;

	data = kzalloc(sizeof(struct switchx_data), GFP_KERNEL);
	if (!data) {
		rc = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct switchx_data));
	data->signature = SX_I2C_DEV_SIGNATURE;

	i2c_set_clientdata(client, data);
	mutex_init(&data->cmd_lock);

	if (sysfs_create_bin_file(&client->dev.kobj, &query_fm_attr))
		goto exit_free;
	if (sysfs_create_bin_file(&client->dev.kobj, &mgir_reg_attr))
		goto exit_query_fw_cmd;
	if (sysfs_create_bin_file(&client->dev.kobj, &sw_reset_attr))
		goto exit_mgir;

	data->retries = 10;    // use 10 retries only during probing, then callibrate it accrding
			       // to number of retries of query_fm at it first probing call
	data->timeout = 500;
	data->transact_id = 0;
	data->bus_id = client->adapter->nr;
	data->i2c_dev_addr = client->addr;
	data->i2c_dev = &client->dev;
	data->client = client;
	data->route_enforced = 0;
	adap = client->adapter;

	/* ACCESS_REG and CONFIG_PROFILE commands use mailboxes for input and for output parameters.
	   QUERY_FW and QUERY_BOARDINFO commands uses mailboxes for output parameters.
	   In order to use mailboxes through the i2c, special area is reserved on the i2c address space that
	   can be used for input and output mailboxes.
	   Such mailboxes are called Local Mailboxes. When using a local mailbox, software should specify 0 as the
	   Input/Output parameters. The location of the Local Mailbox addresses on the i2c space can be retrieved
	   through the QUERY_FW command.
	   For this purpose QUERY_FW is to be issued with opcode modifier equal SX_IMM_OUTPUT_1 (0x01).
	   For such command the output parameter is an immediate value, struct output_param_opmode1.
	   Invoke QUERY_FW command for swicthx probing and for getting local mailboxes addresses.
	   Read from immedate output parameters.
	*/
	memset(&output_param, 0, sizeof(struct output_param_opmode1));
	if((rc = query_fw_cmd(&client->dev,
				SX_IMM_OUTPUT_1,
				SX_BAR0_OFF_NONE,
				CIR_EVENT_NO_REPORT,
				sizeof(struct output_param_opmode1),
				0,
				(u8 *)&output_param)) < 0) {
		goto exit_query_fw_cmd;
	}
	list_add(&data->list, &switchx_cfg.switchx_list);

	convert_mbox((u32 *)&output_param);
	dprintk(		"%s(%d) 0x%08x 0x%08x 0x%08x 0x%08x\n",
		__FUNCTION__, __LINE__,
		output_param.out_param_h.local_mb_size, output_param.out_param_h.local_mb_bar0_offset,
		output_param.out_param_l.local_mb_size, output_param.out_param_l.local_mb_bar0_offset);

	data->mb_size_in = output_param.out_param_h.local_mb_size;
	data->mb_offset_in = output_param.out_param_h.local_mb_bar0_offset;
	data->mb_size_out = output_param.out_param_l.local_mb_size;
	data->mb_offset_out = output_param.out_param_l.local_mb_bar0_offset;
	data->retries = data->retry_cntr + 3;
	data->go_bit_status = GO_BIT_STAT_OK;
	memset(data->cache, 0, SX_MAX_REG_SIZE);
	memset(&query_fw, 0, sizeof(struct query_fw_output_mbox_layout));
	query_fw_cmd(&client->dev,
			SX_MB_OUTPUT,
			SX_BAR0_OFF_NONE,
			CIR_EVENT_REPORT,
			sizeof(struct query_fw_output_mbox_layout),
			0,
			(u8*)&query_fw);

	convert_query_fw_mbox((u32 *)&query_fw);
	memcpy(data->name, id->name, sizeof(id->name));
	printk(KERN_INFO "ASIC probe device %s (rtry=%d) fw_rev: major=%d minor=%d submin=%d in mb size=%x off=0x%08x out mb size=%x off=0x%08x\n",
		id->name, data->retry_cntr,
		query_fw.fw_rev_major, query_fw.fw_rev_minor, query_fw.fw_rev_subminor,
		data->mb_size_in, data->mb_offset_in, data->mb_size_out, data->mb_offset_out);

	switch (id->driver_data) {
	case any_chip:
	case switchx:
	case connectx:
	case switchib:
	case switchspc:
		break;
	}
 
	return rc;

      exit_mgir:
	sysfs_remove_bin_file(&client->dev.kobj, &mgir_reg_attr);
      exit_query_fw_cmd:
	sysfs_remove_bin_file(&client->dev.kobj, &query_fm_attr);
      exit_free:
	i2c_set_clientdata(client, NULL);
	kfree(data);
      exit:
	printk(KERN_INFO "ASIC devic %sn", id->name);

	return rc;
}

static int switchx_remove(struct i2c_client *client)
{
	struct switchx_data *data = i2c_get_clientdata(client);

	sysfs_remove_bin_file(&client->dev.kobj, &mgir_reg_attr);
	sysfs_remove_bin_file(&client->dev.kobj, &query_fm_attr);
	if (!list_empty(&switchx_cfg.switchx_list))
		list_del_rcu(&data->list);
	if(data)
		kfree(data);

	return 0;
}

static const struct i2c_device_id switchx_id[] = {
	{ "switchx", switchx },
	{ "connectx", connectx },
	{ "switchib", switchib },
	{ "switchspc", switchspc },
	{ }
};

MODULE_DEVICE_TABLE(i2c, switchx_id);

/* This is the driver that will be inserted */
static struct i2c_driver switchx_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "switchx",
	},
	.probe		= switchx_probe,
	.remove		= switchx_remove,
	.id_table	= switchx_id,
	.detect		= switchx_detect,
	.address_list	= normal_i2c,
};

static int __init switchx_init(void)
{
	switchx_probe_failed_cache = kmem_cache_create("switchx_probe_failed_cache",
					 sizeof(struct switchx_probe_failed),
					 0,
					 SLAB_HWCACHE_ALIGN, NULL);
	if (!switchx_probe_failed_cache)
		return -ENOMEM;

	INIT_LIST_HEAD(&switchx_cfg.switchx_list);
	INIT_LIST_HEAD(&switchx_cfg.probe_failed_list);
	if (cir_reg_num == 0)
		switchx_cfg.reg_cir_offset = SX_BAR0_OFFSET_PRIM;
	else
		switchx_cfg.reg_cir_offset = SX_BAR0_OFFSET_SECOND;
	switchx_cfg.reg_sw_reset_offset = SX_SW_RESET_OFFSET;
	if (i2c_add_driver(&switchx_driver) != 0)
		return -1;
	switchx_cfg.last_access_dev_id = 0;

	printk(KERN_INFO "%s: reg_cir_offset=0x%08x reg_sw_reset_offset=0x%08x\n",
		__FUNCTION__, switchx_cfg.reg_cir_offset, switchx_cfg.reg_sw_reset_offset);

{
	struct i2c_board_info info;
	struct i2c_adapter *adap;
	struct i2c_client *client;
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = 0x48;
	strlcpy(info.type, "switchspc", I2C_NAME_SIZE);
	adap = i2c_get_adapter(4);
	if (adap)
		client = i2c_new_device(adap, &info);
}

	return 0;
}

static void __exit switchx_exit(void)
{
	struct switchx_probe_failed *swpf, *next;

	i2c_del_driver(&switchx_driver);

	if (switchx_probe_failed_cache) {
                list_for_each_entry_safe(swpf, next, &switchx_cfg.probe_failed_list, list) {
                        list_del(&swpf->list);
                        kmem_cache_free(switchx_probe_failed_cache, swpf);
                }
		kmem_cache_destroy(switchx_probe_failed_cache);
	}
}


MODULE_AUTHOR("Vadim Pasternak <vadimp@mellanox.com>");
MODULE_DESCRIPTION("SWITCHX I2C/SMbus driver");
MODULE_LICENSE("GPL v2");

module_init(switchx_init);
module_exit(switchx_exit);
