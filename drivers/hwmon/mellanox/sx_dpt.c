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

#include <linux/skbuff.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/if_ether.h>
#include <linux/mlx_sx/kernel_user.h>
#include <linux/mlx_sx/device.h>
#include <linux/mlx_sx/driver.h>
#include "sx_dpt.h"
#include "sx.h"
#include "fw.h"
#include "sx_sgmii.h"

#define MAX_I2C_RETRIES 1

extern struct sx_globals    sx_glb;
extern struct pci_device_id sx_pci_table[];
extern int                  tx_debug;

#include <linux/module.h>
#include <linux/mlx_sx/sx_i2c_if.h>

#ifdef SX_DEBUG
#define DPRINTK(fmt, args ...)                                     \
    do {                                                           \
        printk(KERN_ERR DRV_NAME " [%s: %s() line %d]: " fmt "\n", \
               __FILE__, __func__, __LINE__, ## args);             \
    } while (0)
#else
#define DPRINTK(fmt, args ...) \
    do {} while (0)
#endif

#define ENTER_FUNC() DPRINTK("ENTER {");
#define EXIT_FUNC()  DPRINTK("EXIT }");

#define PRINTK_ERR(fmt, args ...)                                  \
    do {                                                           \
        printk(KERN_ERR DRV_NAME " [%s: %s() line %d]: " fmt "\n", \
               __FILE__, __func__, __LINE__, ## args);             \
    } while (0)

static char *dpt_type2str[] = {
    /* DPT_PATH_INVALID = 0*/
    "NONE",
    /* DPT_PATH_I2C = 1*/
    " I2C",
    /* DPT_PATH_SGMII = 2*/
    "SGMII",
    /* DPT_PATH_PCI_E = 3*/
    "PCIE",
};

/*
 * Init default DPT device 255.
 * On systems without PCI adding virtual PCI device.
 * On systems with PCI , PCI devices will be added to
 * DPT from pci_probe callbacks.
 */
int sx_dpt_init_default_dev(struct sx_dev *sx_dev)
{
    union ku_dpt_path_info path_data;
    int                    dev_id = DEFAULT_DEVICE_ID;
    int                    err;

    sx_dev->device_id = dev_id;
    path_data.sx_i2c_info.sx_i2c_dev = 0x448;
    err = sx_dpt_add_dev_path(dev_id, DPT_PATH_I2C, path_data, 1);
    if (err) {
        sx_err(sx_dev, "Failed adding I2C path to the default device\n");
        return err;
    }

    err = sx_dpt_set_cr_access_path(dev_id, DPT_PATH_I2C);
    if (err) {
        sx_err(sx_dev, "Failed setting CR access path to I2C\n");
        return err;
    }

#ifdef NO_PCI
    /* In NO_PCI mode we act as if we had PCI path */
    sx_glb.tmp_dev_ptr = sx_dev;
    return sx_dpt_init_dev_pci(sx_dev);
#endif

#ifdef CONFIG_SX_SGMII_PRESENT
    /* In NO_PCI mode we act as if we had PCI path */
    sx_glb.tmp_dev_ptr = sx_dev;
#endif

    return err;
}

/*
 * In Barracuda system SwitchIB should be
 * the default (255) device
 */
int sx_dpt_alloc_pci_dev_id(struct sx_dev *sx_dev, int* dpt_dev_id)
{
    int err;
    int is_path_valid;

#ifndef NO_PCI
    struct sx_dev *tmp_sx_dev = NULL;
#endif
    int i, alloc_dev_id, free_id_found = 0;

    is_path_valid = sx_dpt_is_path_valid(DEFAULT_DEVICE_ID, DPT_PATH_PCI_E);
    if (!is_path_valid) {
        *dpt_dev_id = DEFAULT_DEVICE_ID;
        goto out;
    }

    /* lookup for next pci dev. */
    for (i = DEFAULT_DEVICE_ID; i > 0; i--) {
        is_path_valid = sx_dpt_is_path_valid(i, DPT_PATH_PCI_E);
        if (!is_path_valid) {
            free_id_found = 1;
            alloc_dev_id = i;
            break;
        }
    }

    if (!free_id_found) {
        sx_err(sx_dev, "No free dev_id found.\n");
        err = -EINVAL;
        goto out;
    }

#ifndef NO_PCI
    /*
     * In Barracude systems SwitchIB always should be default device
     */
    err = sx_dpt_get_sx_dev_by_id(DEFAULT_DEVICE_ID, &tmp_sx_dev);

    if (tmp_sx_dev == NULL || sx_dev == NULL) {
		printk("Error null ptr: tmp_sx_dev: %p,sx_dev: %p \n",
		       tmp_sx_dev, sx_dev);
		err = -EINVAL;
	    goto out;
	}
    
    if ( tmp_sx_dev->pdev == NULL ) {
        printk("Error null ptr: tmp_sx_dev->pdev: 0x%p \n",
                   tmp_sx_dev->pdev );
        err = -EINVAL;
        goto out;
    }

	if ( sx_dev->pdev == NULL ) {
		printk("Error null ptr: sx_dev->pdev: 0x%p \n",
				sx_dev->pdev );
		err = -EINVAL;
		goto out;
    }

    if ((tmp_sx_dev->pdev->device == SWITCHX_PCI_DEV_ID) &&
        (sx_dev->pdev->device == SWITCH_IB_PCI_DEV_ID ||
         sx_dev->pdev->device == SWITCH_IB2_PCI_DEV_ID)) {
        err = sx_dpt_move(alloc_dev_id, DEFAULT_DEVICE_ID);
        if (err) {
            goto out;
        }

        *dpt_dev_id = DEFAULT_DEVICE_ID;
        goto out;
    }
#endif

    *dpt_dev_id = alloc_dev_id;
out:
    return 0;
}

/*
 * Build pci dev id [8 bit busno|8 bit slot|8 bit devfn]
 * for example on piranha ppc : 0x18100
 */
unsigned int sx_dpt_build_pci_dev_id(struct sx_dev *sx_dev)
{
    unsigned int dpt_pci_dev_id = 0;

#ifdef NO_PCI
    return 0x18100;
#endif

    dpt_pci_dev_id |= sx_dev->pdev->bus->number << 16;
    dpt_pci_dev_id |= PCI_SLOT(sx_dev->pdev->devfn) << 8;
    dpt_pci_dev_id |= PCI_FUNC(sx_dev->pdev->devfn);
    printk("%s: Build pci_dev_id: 0x%x , bus:%d , slot(dev):%d, fn:%d \n",
           __func__, dpt_pci_dev_id, sx_dev->pdev->bus->number,
           PCI_SLOT(sx_dev->pdev->devfn), PCI_FUNC(sx_dev->pdev->devfn));
    return dpt_pci_dev_id;
}

int sx_dpt_init_dev_pci(struct sx_dev *sx_dev)
{
    union ku_dpt_path_info path_data;
    int                    dev_id;
    int                    err;

    err = sx_dpt_alloc_pci_dev_id(sx_dev, &dev_id);
    if (err) {
        sx_err(sx_dev, "Failed adding PCI path to the default device\n");
        return err;
    }

    sx_dev->device_id = dev_id;
    printk("Called %s with device_id: %d \n", __func__, sx_dev->device_id);
    path_data.sx_pcie_info.pci_id = sx_dpt_build_pci_dev_id(sx_dev);
    path_data.sx_pcie_info.sx_dev = sx_dev;
    err = sx_dpt_add_dev_path(dev_id, DPT_PATH_PCI_E, path_data, 1);
    if (err) {
        sx_err(sx_dev, "Failed adding PCI path to the default device\n");
        return err;
    }

#ifndef NO_PCI
    sx_dpt_set_cmd_path(dev_id, DPT_PATH_PCI_E);
    sx_dpt_set_emad_path(dev_id, DPT_PATH_PCI_E);
    sx_dpt_set_mad_path(dev_id, DPT_PATH_PCI_E);
#else
    sx_dpt_set_cmd_path(dev_id, DPT_PATH_I2C);
    sx_dpt_set_emad_path(dev_id, DPT_PATH_I2C);
    sx_dpt_set_mad_path(dev_id, DPT_PATH_I2C);
#endif

    if (dev_id == DEFAULT_DEVICE_ID) {
        sx_glb.tmp_dev_ptr = sx_dev;
    }


    return err;
}

int sx_dpt_init()
{
    int err = 0;
    int i;

    ENTER_FUNC();

    for (i = 0; i < MAX_NUM_OF_REMOTE_SWITCHES; i++) {
        sx_glb.sx_dpt.dpt_info[i].cmd_path = DPT_PATH_INVALID;
        sx_glb.sx_dpt.dpt_info[i].emad_path = DPT_PATH_INVALID;
        sx_glb.sx_dpt.dpt_info[i].mad_path = DPT_PATH_INVALID;
        sx_glb.sx_dpt.dpt_info[i].cr_access_path = DPT_PATH_INVALID;

        memset(sx_glb.sx_dpt.dpt_info[i].is_ifc_valid, 0,
               sizeof(sx_glb.sx_dpt.dpt_info[i].is_ifc_valid));
    }
#ifdef NO_PCI
    sx_glb.sx_i2c.read = NULL;
    sx_glb.sx_i2c.write = NULL;
    sx_glb.sx_i2c.read_dword = NULL;
    sx_glb.sx_i2c.write_dword = NULL;
    sx_glb.sx_i2c.enforce = NULL;
    sx_glb.sx_i2c.release = NULL;
#else
    sx_glb.sx_i2c.read = sx_dpt_stub_i2c_read;
    sx_glb.sx_i2c.write = sx_dpt_stub_i2c_write;
    sx_glb.sx_i2c.read_dword = sx_dpt_stub_i2c_read_dword;
    sx_glb.sx_i2c.write_dword = sx_dpt_stub_i2c_write_dword;
    sx_glb.sx_i2c.enforce = sx_dpt_stub_i2c_enforce;
    sx_glb.sx_i2c.release = sx_dpt_stub_i2c_release;
#endif
    sx_glb.sx_i2c.get_fw_rev = NULL;
    sx_glb.sx_i2c.set_go_bit_stuck = NULL;
    sx_glb.sx_sgmii.initialized = 0;
    sx_glb.sx_sgmii.base_smac = 0;
    sx_sgmii_init_cb(&sx_glb.sx_sgmii);

    EXIT_FUNC();
    return err;
}

#if 0
int sx_dpt_find_pci_dev_old(unsigned int sx_pci_dev_id, int vendor, int device, struct pci_dev **sx_pci_dev)
{
    int             pci_bus_id;
    int             pci_dev_id;
    int             pci_func_id;
    struct pci_dev *tmp_dev = NULL;
    int             err = 1;

    *sx_pci_dev = NULL;

    pci_bus_id = SX_GET_PCI_BUS_ID(sx_pci_dev_id);
    pci_dev_id = SX_GET_PCI_DEV_ID(sx_pci_dev_id);
    pci_func_id = SX_GET_PCI_FUNC_ID(sx_pci_dev_id);
    printk(KERN_DEBUG "%s(): find pci ven:0x%x, dev: %x "
           "pci addr %x:%x:%x\n", __func__, vendor, device,
           pci_bus_id, pci_dev_id, pci_func_id);

    do {
        tmp_dev = pci_get_device(vendor, device, tmp_dev);
        *sx_pci_dev = tmp_dev;
        if ((tmp_dev != NULL) && tmp_dev->bus) {
            printk(KERN_DEBUG "bus_num: %d , %s , dev: %d , "
                   "fn: %d, tmp_dev->devfn %d \n",
                   tmp_dev->bus->number,
                   tmp_dev->bus->name,
                   PCI_SLOT(tmp_dev->devfn),
                   PCI_FUNC(tmp_dev->devfn),
                   tmp_dev->devfn);
            printk(KERN_DEBUG "PCI Device found successfully\n");
            err = 0;
            break;
        }
    } while (tmp_dev != NULL);

    return err;
}
#endif

int sx_dpt_find_pci_dev(unsigned int sx_pci_dev_id, int vendor, int device, struct pci_dev **sx_pci_dev)
{
    int            pci_bus_id;
    int            pci_dev_id;
    int            pci_func_id;
    int            pci_devfn;
    int            err = 1;
    struct sx_dev *tmp_dev = NULL, *curr_dev = NULL;
    int            is_found = 0;

    *sx_pci_dev = NULL;

    pci_bus_id = SX_GET_PCI_BUS_ID(sx_pci_dev_id);
    pci_dev_id = SX_GET_PCI_DEV_ID(sx_pci_dev_id);
    pci_func_id = SX_GET_PCI_FUNC_ID(sx_pci_dev_id);
    pci_devfn = PCI_DEVFN(pci_dev_id, pci_func_id);

    printk(KERN_DEBUG "%s(): lookup for sx_pci_dev_id: 0x%x "
           "pci addr %x:%x:%x\n", __func__, sx_pci_dev_id,
           pci_bus_id, pci_dev_id, pci_func_id);

    /* The user gives us dev_id in the data param (immediate) */
    list_for_each_entry_safe(curr_dev, tmp_dev, &sx_glb.pci_devs_list, list) {
        printk(KERN_DEBUG "%s(): CHECK vendor: 0x%x , device : 0x%x ,"
               "pci addr %x:%x:%x\n", __func__,
               curr_dev->pdev->vendor, curr_dev->pdev->device,
               curr_dev->pdev->bus->number,
               PCI_SLOT(curr_dev->pdev->devfn),
               PCI_FUNC(curr_dev->pdev->devfn));
        if ((curr_dev->pdev->vendor == PCI_VENDOR_ID_MELLANOX) &&
            (curr_dev->pdev->bus->number == pci_bus_id) &&
            (curr_dev->pdev->devfn == pci_devfn)) {
            is_found = 1;
            err = 0;
            *sx_pci_dev = curr_dev->pdev;
            break;
        }
    }

    if (!is_found &&
        (sx_glb.sx_dpt.dpt_info[DEFAULT_DEVICE_ID].sx_pcie_info.sx_dev != NULL)) {
        curr_dev = sx_glb.sx_dpt.dpt_info[DEFAULT_DEVICE_ID].sx_pcie_info.sx_dev;
        *sx_pci_dev = curr_dev->pdev;
        err = 0;
    }

    return err;
}

bool sx_dpt_is_path_valid(int sx_dev_id, enum  ku_dpt_path_type path)
{
    return sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[path];
}

/*
 * This function will add path to entry dev_id in DPT. The path
 * should be provided with remote device description. For
 * example , if added path is I2C than path_data should be
 * i2c_dev_id.
 */
int sx_dpt_add_dev_path(int sx_dev_id, enum ku_dpt_path_type path, union ku_dpt_path_info path_data, uint8_t is_local)
{
    int err = 0;

#ifndef NO_PCI
    struct pci_dev *sx_pci_dev = NULL;
#endif
    unsigned int sx_pci_dev_id;

    ENTER_FUNC();

    switch (path) {
    case DPT_PATH_I2C:
        if (sx_glb.sx_i2c.is_registered == 0) {
            sx_dpt_reg_i2c_ifc(&sx_glb.sx_i2c);
            sx_glb.sx_i2c.is_registered = 1;
        }

        sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_i2c_info = path_data.sx_i2c_info;
        sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_I2C] = true;
        if (sx_glb.sx_i2c.get_local_mbox) {
            sx_glb.sx_i2c.get_local_mbox(
                path_data.sx_i2c_info.sx_i2c_dev,
                &sx_glb.sx_dpt.dpt_info[sx_dev_id].in_mb_size,
                &sx_glb.sx_dpt.dpt_info[sx_dev_id].in_mb_offset,
                &sx_glb.sx_dpt.dpt_info[sx_dev_id].out_mb_size,
                &sx_glb.sx_dpt.dpt_info[sx_dev_id].out_mb_offset);
        }
        if (!is_local && sx_glb.sx_i2c.get_fw_rev) {
            err = sx_glb.sx_i2c.enforce(path_data.sx_i2c_info.sx_i2c_dev);
            if (err) {
                PRINTK_ERR("I2C bus %d of device %d is not ready. "
                           "query_fw will not be performed. err=%d\n",
                           path_data.sx_i2c_info.sx_i2c_dev, sx_dev_id, err);
                err = -EBUSY;
                break;
            }

            sx_glb.sx_i2c.get_fw_rev(path_data.sx_i2c_info.sx_i2c_dev,
                                     &sx_glb.sx_dpt.dpt_info[sx_dev_id].fw_rev);
            err = sx_glb.sx_i2c.release(path_data.sx_i2c_info.sx_i2c_dev);
            if (err) {
                PRINTK_ERR("I2C bus %d of device %d is not ready. "
                           "i2c release failed. err=%d\n",
                           path_data.sx_i2c_info.sx_i2c_dev, sx_dev_id, err);
                err = -EBUSY;
                break;
            }
        }

        break;

    case DPT_PATH_SGMII:
        if (!sx_glb.sx_sgmii.init) {
            PRINTK_ERR("Cannot add SGMII path. SGMII is not supported\n");
            err = -EINVAL;
            break;
        }

        if (!sx_glb.sx_sgmii.initialized) {
            if (sx_glb.sx_sgmii.init()) {
                err = -EINVAL;
                break;
            }
        }

        sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_sgmii_info = path_data.sx_sgmii_info;
        sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_SGMII] = true;
        break;

    case DPT_PATH_PCI_E:
        sx_pci_dev_id = path_data.sx_pcie_info.pci_id;
#ifndef NO_PCI
        err = sx_dpt_find_pci_dev(sx_pci_dev_id, 0, 0, &sx_pci_dev);
        if (err) {
            PRINTK_ERR("sx_dpt_find_pci_dev failed, sx_pci_dev_id: 0x%x \n",
                       sx_pci_dev_id);
            err = -EINVAL;
            goto out;
        }

        sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info.pci_id = sx_pci_dev_id;
        sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info.sx_dev = pci_get_drvdata(sx_pci_dev);
        if (is_local) {
            ((struct sx_dev *)(sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info.sx_dev))->device_id = sx_dev_id;
        }
#else
        sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info.pci_id = sx_pci_dev_id;
        sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info.sx_dev = sx_glb.tmp_dev_ptr;
        sx_glb.tmp_dev_ptr->device_id = sx_dev_id;
#endif
        sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_PCI_E] = true;
        printk(KERN_INFO PFX "Successfully added path %s to device %d pci_dev_id %u\n",
               dpt_type2str[path], sx_dev_id, sx_pci_dev_id);
        break;

    default:
        PRINTK_ERR("Try to set invalid path to sx_dev_id: %d, dpt_path: %s\n",
                   sx_dev_id, dpt_type2str[path]);
        err = -EINVAL;
        break;
    }

#ifndef NO_PCI
out:
#endif

    if (!err) {
        printk(KERN_INFO PFX "Successfully added path %s to device %d\n",
               dpt_type2str[path], sx_dev_id);
    }

    EXIT_FUNC();
    return err;
}

/**
 * This function with params dev_id and path will invalidate the
 * given "path" at index DEV_ID in DPT. All other pathes will
 * remains without changes.
 */
int sx_dpt_remove_dev_path(int sx_dev_id, enum ku_dpt_path_type path)
{
    int err = 0;

    ENTER_FUNC();

    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].cmd_path == path) {
        PRINTK_ERR("Can't remove path because %d "
                   "is current active cmd_path.\n", path);
        return -EINVAL;
    }

    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].emad_path == path) {
        PRINTK_ERR("Can't remove path because %d "
                   "is current active emad_path.\n", path);
        return -EINVAL;
    }

    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].mad_path == path) {
        PRINTK_ERR("Can't remove path because %d "
                   "is current active mad_path.\n", path);
        return -EINVAL;
    }

    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].cr_access_path == path) {
        PRINTK_ERR("Can't remove path because %d "
                   "is current active cr_access_path.\n", path);
        return -EINVAL;
    }

    switch (path) {
    case DPT_PATH_I2C:
        sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_I2C] =
            false;
        memset(&sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_i2c_info, 0,
               sizeof(sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_i2c_info));
        break;

    case DPT_PATH_SGMII:
        sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_SGMII]
            = false;
        memset(&sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_sgmii_info, 0,
               sizeof(sx_glb.sx_dpt.dpt_info[sx_dev_id].
                      sx_sgmii_info));
        break;

    case DPT_PATH_PCI_E:
        sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_PCI_E]
            = false;
        memset(&sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info, 0,
               sizeof(sx_glb.sx_dpt.dpt_info[sx_dev_id].
                      sx_pcie_info));
        break;

    default:
        PRINTK_ERR("Invalid path.\n");
        err = -EINVAL;
    }

    if (!err) {
        printk(KERN_INFO PFX "Successfully removed path %s from device %d\n",
               dpt_type2str[path], sx_dev_id);
    }

    EXIT_FUNC();
    return err;
}

/**
 * This function with params dev_id and path will invalidate all pathes
 * at index DEV_ID in DPT.
 */
void __sx_dpt_remove_dev(int sx_dev_id)
{
    ENTER_FUNC();
    printk(KERN_DEBUG "%s() Remove dev %d \n", __func__, sx_dev_id);

    sx_glb.sx_dpt.dpt_info[sx_dev_id].cmd_path = DPT_PATH_INVALID;
    sx_glb.sx_dpt.dpt_info[sx_dev_id].emad_path = DPT_PATH_INVALID;
    sx_glb.sx_dpt.dpt_info[sx_dev_id].mad_path = DPT_PATH_INVALID;
    sx_glb.sx_dpt.dpt_info[sx_dev_id].cr_access_path = DPT_PATH_INVALID;

    sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_I2C] = false;
    sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_SGMII] = false;
    sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_PCI_E] = false;

    memset(&sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_i2c_info, 0,
           sizeof(sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_i2c_info));
    memset(&sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_sgmii_info, 0,
           sizeof(sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_sgmii_info));
    memset(&sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info, 0,
           sizeof(sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info));

    printk(KERN_INFO PFX "Successfully removed device %d from the DPT\n", sx_dev_id);
    EXIT_FUNC();
}

int sx_dpt_is_dev_pci_attached(int sx_dev_id)
{
    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_PCI_E] &&
            (sx_glb.sx_dpt.dpt_info[sx_dev_id].cmd_path == DPT_PATH_PCI_E) ){
        return 1;
    }

    return 0;
}

int sx_dpt_remove_dev(int sx_dev_id, int restart_flow)
{
    int err = 0;
    void  *sx_dev = NULL; 
    void  *def_sx_dev = NULL;
    
    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_PCI_E]){
        sx_dev = sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info.sx_dev;
    }
    
    if (sx_glb.sx_dpt.dpt_info[DEFAULT_DEVICE_ID].is_ifc_valid[DPT_PATH_PCI_E]){
        def_sx_dev = sx_glb.sx_dpt.dpt_info[DEFAULT_DEVICE_ID].sx_pcie_info.sx_dev;
    }        
    
    /* 
     * if the removed device is PCI (CMD_PATH=PCI) and 
     * the it is the same as default device then remove default device too 
     */ 
    if ( (sx_glb.sx_dpt.dpt_info[sx_dev_id].cmd_path == DPT_PATH_PCI_E) && 
         (sx_dev == def_sx_dev) && restart_flow) {
        __sx_dpt_remove_dev(DEFAULT_DEVICE_ID);
    }
    
    __sx_dpt_remove_dev(sx_dev_id);

    return err;
}

/**
 * This function is used to change dpt path : i2c , pci-e , sgmii
 * by changing the callbacks to approprite interface functions
 * To validate if the change is suceessed need to make some
 * wr/rd to check if the new interface is working
 */
int sx_dpt_set_cmd_path(int sx_dev_id, enum  ku_dpt_path_type cmd_path)
{
    int                   err = 0;
    enum ku_dpt_path_type old_cmd_path;

    ENTER_FUNC();

    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[cmd_path] == false) {
        PRINTK_ERR("sx_dev_id:%d , path %d is not valid !\n", sx_dev_id, cmd_path);
        return -EINVAL;
    }

    if ((cmd_path != DPT_PATH_I2C) && (cmd_path != DPT_PATH_PCI_E) && (cmd_path != DPT_PATH_SGMII)) {
        PRINTK_ERR("path %d is not valid ! Supported CMD pathes: "
                   "I2C and PCI-E\n", cmd_path);
        return -EINVAL;
    }

    /* save old path */
    old_cmd_path = sx_glb.sx_dpt.dpt_info[sx_dev_id].cmd_path;
    /* change the pathes */
    sx_glb.sx_dpt.dpt_info[sx_dev_id].cmd_path = cmd_path;
    /* validate new cmd path */
    if (old_cmd_path != sx_glb.sx_dpt.dpt_info[sx_dev_id].cmd_path) {
        err = sx_dpt_validate_new_cmd_path(sx_dev_id, cmd_path);
        if (err) {
            PRINTK_ERR("sx_dpt_validate_new_cmd_path() "
                       "for path %d failed. Rollback cmd_path to %d\n",
                       cmd_path, old_cmd_path);
            sx_glb.sx_dpt.dpt_info[sx_dev_id].cmd_path =
                old_cmd_path;
        }
    }

    if (!err) {
        printk(KERN_INFO PFX "Successfully set CMD path for device %d to %s\n",
               sx_dev_id, dpt_type2str[cmd_path]);
    }

    EXIT_FUNC();
    return err;
}

int sx_dpt_get_i2c_info(int sx_dev_id, struct ku_dpt_i2c_info** i2c_info)
{
    if ((i2c_info == NULL) || (sx_dev_id > MAX_NUM_OF_REMOTE_SWITCHES)) {
        return -EINVAL;
    }

    *i2c_info = &(sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_i2c_info);
    return 0;
}

int sx_dpt_get_cmd_path(int sx_dev_id)
{
    /* DEFAULT_DEVICE_ID is a WA for FW burn on local device
     * before device ID is known. Burning FW will be done through CMD IFC
     * over the PCI */
#ifndef NO_PCI
    if (sx_dev_id == DEFAULT_DEVICE_ID) {
        return DPT_PATH_PCI_E;
    }
#endif

    return sx_glb.sx_dpt.dpt_info[sx_dev_id].cmd_path;
}

/**
 * This function is used to set the emad path in the dpt
 */
int sx_dpt_set_emad_path(int sx_dev_id, enum  ku_dpt_path_type emad_path)
{
    int                   err = 0;
    enum ku_dpt_path_type old_emad_path;

    ENTER_FUNC();

    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[emad_path] ==
        false) {
        PRINTK_ERR("path %d is not valid !\n", emad_path);
        return -EINVAL;
    }

    /* save old path */
    old_emad_path = sx_glb.sx_dpt.dpt_info[sx_dev_id].emad_path;
    /* change the pathes */
    sx_glb.sx_dpt.dpt_info[sx_dev_id].emad_path = emad_path;
    /* validate new emad path */
    if (old_emad_path != sx_glb.sx_dpt.dpt_info[sx_dev_id].emad_path) {
        err = sx_dpt_validate_new_emad_path(sx_dev_id);
        if (err) {
            PRINTK_ERR("sx_dpt_validate_new_emad_path() for "
                       "path %d failed. Rollback emad_path to %d\n",
                       emad_path, old_emad_path);
            sx_glb.sx_dpt.dpt_info[sx_dev_id].emad_path =
                old_emad_path;
        }
    }

    if (!err) {
        printk(KERN_INFO PFX "Successfully set EMAD path for device %d to %s\n",
               sx_dev_id, dpt_type2str[emad_path]);
    }

    EXIT_FUNC();
    return err;
}

int sx_dpt_get_emad_path(int sx_dev_id)
{
    return sx_glb.sx_dpt.dpt_info[sx_dev_id].emad_path;
}

int sx_dpt_set_mad_path(int sx_dev_id, enum  ku_dpt_path_type mad_path)
{
    int                   err = 0;
    enum ku_dpt_path_type old_mad_path;

    ENTER_FUNC();

    if ((sx_dev_id < 0) ||
        (sx_dev_id > MAX_NUM_OF_REMOTE_SWITCHES)) {
        PRINTK_ERR("Device id is not valid");
        return -EINVAL;
    }

    if ((mad_path < DPT_PATH_MIN) ||
        (mad_path > DPT_PATH_MAX)) {
        PRINTK_ERR("MAD path is not valid");
        return -EINVAL;
    }

    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[mad_path] == false) {
        PRINTK_ERR("Path %d is not valid !\n", mad_path);
        return -EINVAL;
    }

    /* save old path */
    old_mad_path = sx_glb.sx_dpt.dpt_info[sx_dev_id].mad_path;
    /* change the pathes */
    sx_glb.sx_dpt.dpt_info[sx_dev_id].mad_path = mad_path;
    /* validate new emad path */
    if (old_mad_path != sx_glb.sx_dpt.dpt_info[sx_dev_id].mad_path) {
        err = sx_dpt_validate_new_emad_path(sx_dev_id);
        if (err) {
            PRINTK_ERR("sx_dpt_validate_new_emad_path() for path "
                       "%d failed. Rollback emad_path to %d\n",
                       mad_path, old_mad_path);
            sx_glb.sx_dpt.dpt_info[sx_dev_id].mad_path =
                old_mad_path;
        }
    }

    if (!err) {
        printk(KERN_INFO PFX "Successfully set MAD path for device %d to %s\n",
               sx_dev_id, dpt_type2str[mad_path]);
    }

    EXIT_FUNC();
    return err;
}

int sx_dpt_set_cr_access_path(int sx_dev_id, enum ku_dpt_path_type cr_access_path)
{
    int err = 0;

    ENTER_FUNC();

    if ((sx_dev_id < 0) ||
        (sx_dev_id > MAX_NUM_OF_REMOTE_SWITCHES)) {
        PRINTK_ERR("Device id is not valid.\n");
        return -EINVAL;
    }

    if ((cr_access_path < DPT_PATH_MIN) ||
        (cr_access_path > DPT_PATH_MAX)) {
        PRINTK_ERR("CR_ACCESS path is not valid.\n");
        return -EINVAL;
    }

    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[cr_access_path] == false) {
        PRINTK_ERR("path %d is not valid !\n", cr_access_path);
        return -EINVAL;
    }

    /* change the path */
    sx_glb.sx_dpt.dpt_info[sx_dev_id].cr_access_path = cr_access_path;

    printk(KERN_INFO PFX "Successfully set CR_ACCESS path for device %d to %s\n",
               sx_dev_id, dpt_type2str[cr_access_path]);

    EXIT_FUNC();
    return err;
}

enum ku_dpt_path_type sx_dpt_get_cr_access_path(int sx_dev_id)
{
    return sx_glb.sx_dpt.dpt_info[sx_dev_id].cr_access_path;
}

int sx_get_sdq_from_profile(struct sx_dev *dev, enum ku_pkt_type type, u8 swid, u8 etclass, u8 *stclass, u8 *sdq)
{
    if ((type == SX_PKT_TYPE_DROUTE_EMAD_CTL) ||
        (type == SX_PKT_TYPE_EMAD_CTL)) {
        if (stclass) {
            *stclass = dev->profile.emad_tx_prof.stclass;
        }
        *sdq = dev->profile.emad_tx_prof.sdq;
        goto out;
    }

    if (swid >= NUMBER_OF_SWIDS) {
        printk("swid %d is out of range [0..%d], force to 0\n",
               swid, NUMBER_OF_SWIDS);
        swid = 0;
    }

    if (etclass >= NUMBER_OF_ETCLASSES) {
        printk("etclass %d is out of range [0..%d], force to 0\n",
               etclass, NUMBER_OF_ETCLASSES);
        etclass = 0;
    }

    if (stclass) {
        *stclass = dev->profile.tx_prof[swid][etclass].stclass;
    }
    *sdq = dev->profile.tx_prof[swid][etclass].sdq;

out:
    return 0;
}

int sx_get_sdq_per_traffic_type(struct sx_dev *dev, enum ku_pkt_type type, u8 swid, u8 etclass, u8 *stclass, u8 *sdq)
{
    if ((type == SX_PKT_TYPE_DROUTE_EMAD_CTL) ||
        (type == SX_PKT_TYPE_EMAD_CTL)) {
        *sdq = 0;
    } else if ((type == SX_PKT_TYPE_ETH_CTL_UC) ||
               (type == SX_PKT_TYPE_ETH_CTL_MC)) {
        *sdq = 1;
    } else {
        *sdq = 2;
    }
    return 0;
}
static void get_specific_data(enum ku_pkt_type type, u8 to_cpu, u8 etclass, u8 lp, struct isx_specific_data *data)
{
    if ((type == SX_PKT_TYPE_ETH_DATA) || (type == SX_PKT_TYPE_IB_RAW_DATA)
        || (type == SX_PKT_TYPE_IB_TRANSPORT_DATA /*&& !lp*/)) {
        data->type = 0;
    } else {
        data->type = 6;
    }
    if ((type == SX_PKT_TYPE_DROUTE_EMAD_CTL) || (type == SX_PKT_TYPE_EMAD_CTL)) {
        data->emad = 1;
    } else {
        data->emad = 0;
    }
    if ((type == SX_PKT_TYPE_ETH_CTL_MC) || (type == SX_PKT_TYPE_FC_CTL_MC)
        || (type == SX_PKT_TYPE_FCOE_CTL_MC)) {
        data->mc = 1;
    } else {
        data->mc = 0;
    }
    if (type >= SX_PKT_TYPE_IB_RAW_CTL) {
        data->protocol = 0;
    } else {
        data->protocol = 1;
    }

    switch (type) {
    case SX_PKT_TYPE_ETH_CTL_UC:
    case SX_PKT_TYPE_ETH_CTL_MC:
    case SX_PKT_TYPE_EMAD_CTL:
    case SX_PKT_TYPE_FC_CTL_UC:
    case SX_PKT_TYPE_FC_CTL_MC:
    case SX_PKT_TYPE_FCOE_CTL_UC:
    case SX_PKT_TYPE_FCOE_CTL_MC:
        data->ctl = 0;
        break;

    case SX_PKT_TYPE_ETH_DATA:
    case SX_PKT_TYPE_IB_RAW_DATA:
    case SX_PKT_TYPE_IB_TRANSPORT_DATA:
        data->ctl = 1;
        break;

    case SX_PKT_TYPE_DROUTE_EMAD_CTL:
        data->ctl = 2;
        break;

    case SX_PKT_TYPE_IB_RAW_CTL:
    case SX_PKT_TYPE_IB_TRANSPORT_CTL:
    case SX_PKT_TYPE_EOIB_CTL:
    case SX_PKT_TYPE_FCOIB_CTL:
        data->ctl = 3;
        break;

    default:
        break;
    }

    if (to_cpu) {
        data->ctclass = (etclass & 0x8) >> 3;
    } else {
        data->ctclass = 0;
    }

    data->cpu_signature = 0;
    data->signature = 0xE0E0;
}

static void get_specific_data_version_1(enum ku_pkt_type          type,
                                        u8                        to_cpu,
                                        u8                        etclass,
                                        u8                        lp,
                                        u8                        rx_is_router,
                                        u8                        fid_valid,
                                        u16                       fid,
                                        struct isx_specific_data *data)
{
    switch (type) {
    case SX_PKT_TYPE_ETH_CTL_UC:
    case SX_PKT_TYPE_ETH_CTL_MC:
        data->type = 6;
        data->protocol = 1;
        data->ctl = 0;
        data->use_control_tclass = 1;
        data->etclass = 0; /* When control_tclass = 1, this field is reserved */
        data->fid = 0;
        data->fid_valid = 0;
        data->rx_is_router = 0;
        break;

    case SX_PKT_TYPE_ETH_DATA:
        data->type = 0;
        data->protocol = 1;
        data->ctl = 1;
        data->use_control_tclass = 0;
        data->etclass = 0;
        data->rx_is_router = rx_is_router;
        data->fid_valid = fid_valid;
        data->fid = fid;
        break;

    case SX_PKT_TYPE_DROUTE_EMAD_CTL:
    case SX_PKT_TYPE_EMAD_CTL:
        data->type = 6;
        data->protocol = 1;
        data->ctl = 2;
        data->use_control_tclass = 0;
        data->etclass = 0;
        data->fid = 0;
        data->fid_valid = 0;
        data->rx_is_router = 0;
        break;

    default:
        break;
    }

    data->version = 1;
}


/**
 * get HW etclass from device callback structure
 */
int sx_get_hw_etclass(struct isx_meta *meta, u8* hw_etclass)
{
    struct sx_dev *dev;
    unsigned long  flags;
    int            err;

    err = sx_dpt_get_sx_dev_by_id(meta->dev_id, &dev);
    if (err) {
        PRINTK_ERR("Invalid device id %d\n", meta->dev_id);
        return -EINVAL;
    }
    
    /* case of SGMII dev */
    if (dev == NULL){
        *hw_etclass = meta->etclass;
        return 0;
    }

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
    if (sx_priv(dev)->dev_specific_cb.get_hw_etclass_cb != NULL) {
        sx_priv(dev)->dev_specific_cb.get_hw_etclass_cb(meta, hw_etclass);
    } else {
        *hw_etclass = meta->etclass;
    }
    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);

    return 0;
}

/**
 * callback structure for selecting SDQ based on condor and BAZ
 */
int sx_get_sdq(struct isx_meta *meta,
               struct sx_dev   *dev,
               enum ku_pkt_type type,
               u8               swid,
               u8               etclass,
               u8              *stclass,
               u8              *sdq)
{
    unsigned long flags;
    u8            hw_etclass = 0;


    /* this func also take priv->db_lock */
    if (sx_get_hw_etclass(meta, &hw_etclass) != 0) {
        PRINTK_ERR("Error retrieving HW Etclass!\n");
        return -EINVAL;
    }

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);
    if (dev && (sx_priv(dev)->dev_specific_cb.sx_get_sdq_cb != NULL)) {
        sx_priv(dev)->dev_specific_cb.sx_get_sdq_cb(dev, type, swid,
                                                    etclass, stclass, sdq);
    } else {
        PRINTK_ERR("Error retrieving sx_get_sdq_cb callback structure!\n");
        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        return -EINVAL;
    }

    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
    return 0;
}

/**
 * callback structure for filling meta based on condor and BAZ
 */
int sx_build_isx_header(struct isx_meta *meta, struct sk_buff *skb, u8 stclass)
{
    struct sx_dev *dev = sx_glb.tmp_dev_ptr;
    unsigned long  flags;
    u8             hw_etclass = 0;

    /* this func also take priv->db_lock */
    if (sx_get_hw_etclass(meta, &hw_etclass) != 0) {
        PRINTK_ERR("Error retrieving HW Etclass!\n");
        return -EINVAL;
    }

    spin_lock_irqsave(&sx_priv(dev)->db_lock, flags);

    if (dev && (sx_priv(dev)->dev_specific_cb.sx_build_isx_header_cb != NULL)) {
        sx_priv(dev)->dev_specific_cb.sx_build_isx_header_cb(meta, skb, stclass, hw_etclass);
    } else {
        PRINTK_ERR("Error retrieving sx_build_isx_header callback structure!\n");
        spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
        return -EINVAL;
    }

    spin_unlock_irqrestore(&sx_priv(dev)->db_lock, flags);
    return 0;
}

/**
 * Build isx_header from the given meta (meta-data) and push the isx_headr
 * to the beginning of the skb (before the pkt-buffer part) for Version 0
 */
int sx_build_isx_header_v0(struct isx_meta *meta, struct sk_buff *skb, u8 stclass, u8 hw_etclass)
{
    struct tx_base_header_v0 *p_hdr;
    struct isx_specific_data  specific_meta;

    p_hdr = (struct tx_base_header_v0 *)skb_push(skb, ISX_HDR_SIZE);
    if (p_hdr == NULL) {
        if (printk_ratelimit()) {
            printk(KERN_WARNING PFX "sx_build_isx_header: "
                   "Error in skb_push\n");
        }
        return -ENOMEM;
    }

    memset(p_hdr, 0, sizeof(*p_hdr));
    memset(&specific_meta, 0, sizeof(specific_meta));
    get_specific_data(meta->type, meta->to_cpu,
                      meta->etclass, meta->lp, &specific_meta);
    p_hdr->ctl_mc |=
        TO_FIELD(TX_HDR_CTL_MASK,
                 TX_HDR_CTL_SHIFT, specific_meta.ctl);
    p_hdr->ctl_mc |=
        TO_FIELD(TX_HDR_MC_MASK,
                 TX_HDR_MC_SHIFT, specific_meta.mc);
    p_hdr->protocol_etclass |=
        TO_FIELD(TX_HDR_PROTOCOL_MASK,
                 TX_HDR_PROTOCOL_SHIFT,
                 specific_meta.protocol);
    p_hdr->protocol_etclass |=
        TO_FIELD(TX_HDR_ETCLASS_MASK,
                 TX_HDR_ETCLASS_SHIFT, hw_etclass);
    p_hdr->swid |= (u16)TO_FIELD(TX_HDR_SWID_MASK,
                                 TX_HDR_SWID_SHIFT, meta->swid);
    p_hdr->swid = cpu_to_be16(p_hdr->swid);
    p_hdr->system_port_mid |=
        (u16)TO_FIELD(TX_HDR_SYSTEM_PORT_MID_MASK,
                      TX_HDR_SYSTEM_PORT_MID_SHIFT,
                      meta->system_port_mid);
    p_hdr->system_port_mid = cpu_to_be16(p_hdr->system_port_mid);
    p_hdr->ctclass3_rdq_cpu_signature |=
        (u16)TO_FIELD(TX_HDR_CTCLASS3_MASK,
                      TX_HDR_CTCLASS3_SHIFT,
                      specific_meta.ctclass);
    p_hdr->ctclass3_rdq_cpu_signature |=
        (u16)TO_FIELD(TX_HDR_RDQ_MASK,
                      TX_HDR_RDQ_SHIFT, meta->rdq);
    p_hdr->ctclass3_rdq_cpu_signature |=
        (u16)TO_FIELD(TX_HDR_CPU_SIGNATURE_MASK,
                      TX_HDR_CPU_SIGNATURE_SHIFT,
                      specific_meta.cpu_signature);
    p_hdr->ctclass3_rdq_cpu_signature =
        cpu_to_be16(p_hdr->ctclass3_rdq_cpu_signature);
    p_hdr->signature |=
        (u16)TO_FIELD(TX_HDR_SIGNATURE_MASK,
                      TX_HDR_SIGNATURE_SHIFT,
                      specific_meta.signature);
    p_hdr->signature = cpu_to_be16(p_hdr->signature);
    p_hdr->stclass_emad_type |=
        (u16)TO_FIELD(TX_HDR_STCLASS_MASK,
                      TX_HDR_STCLASS_SHIFT, stclass);
    p_hdr->stclass_emad_type |=
        (u16)TO_FIELD(TX_HDR_EMAD_MASK,
                      TX_HDR_EMAD_SHIFT, specific_meta.emad);
    p_hdr->stclass_emad_type |=
        (u16)TO_FIELD(TX_HDR_TYPE_MASK,
                      TX_HDR_TYPE_SHIFT, specific_meta.type);
    p_hdr->stclass_emad_type = cpu_to_be16(p_hdr->stclass_emad_type);
    if (tx_debug) {
        printk(KERN_DEBUG PFX "sx_core_post_send: Sending for version 0 "
               "pkt with specific meta: "
               "sysport:0x%x, specific_meta.ctl: %d,"
               "specific_meta.protocol: %d\n",
               meta->system_port_mid,
               specific_meta.ctl, specific_meta.protocol);
    }
    return 0;
}
/**
 * Build isx_header from the given meta (meta-data) and push the isx_headr
 * to the beginning of the skb (before the pkt-buffer part) for condor (Version 1)
 */
int sx_build_isx_header_v1(struct isx_meta *meta, struct sk_buff *skb, u8 stclass,  u8 hw_etclass)
{
    struct tx_base_header_v1 *p_hdr;
    struct isx_specific_data  specific_meta;

    p_hdr = (struct tx_base_header_v1 *)skb_push(skb, ISX_HDR_SIZE);
    if (p_hdr == NULL) {
        if (printk_ratelimit()) {
            printk(KERN_WARNING PFX "sx_build_isx_header: "
                   "Error in skb_push\n");
        }
        return -ENOMEM;
    }

    memset(p_hdr, 0, sizeof(*p_hdr));
    memset(&specific_meta, 0, sizeof(specific_meta));
    get_specific_data_version_1(meta->type, meta->to_cpu,
                                meta->etclass, meta->lp, meta->rx_is_router,
                                meta->fid_valid, meta->fid, &specific_meta);

    p_hdr->version_ctl |=
        TO_FIELD(TX_HDR_VER_MASK_V1,
                 TX_HDR_VER_SHIFT_V1, specific_meta.version);

    p_hdr->version_ctl |=
        TO_FIELD(TX_HDR_CTL_MASK,
                 TX_HDR_CTL_SHIFT, specific_meta.ctl);

    p_hdr->protocol_rx_is_router_fid_valid |=
        TO_FIELD(TX_HDR_PROTOCOL_MASK,
                 TX_HDR_PROTOCOL_SHIFT,
                 specific_meta.protocol);

    p_hdr->protocol_rx_is_router_fid_valid |=
        TO_FIELD(TX_HDR_RX_IS_ROUTER_MASK_V1,
                 TX_HDR_RX_IS_ROUTER_SHIFT_V1,
                 specific_meta.rx_is_router);

    p_hdr->protocol_rx_is_router_fid_valid |=
        TO_FIELD(TX_HDR_FID_VALID_MASK_V1,
                 TX_HDR_FID_VALID_SHIFT_V1,
                 specific_meta.fid_valid);

    p_hdr->swid_control_etclass |= (u16)TO_FIELD(TX_HDR_SWID_MASK,
                                                 TX_HDR_SWID_SHIFT, meta->swid);

    p_hdr->swid_control_etclass |=
        (u16)TO_FIELD(TX_HDR_CONTROL_MASK_V1,
                      TX_HDR_CONTROL_SHIFT_V1,
                      specific_meta.use_control_tclass);
    p_hdr->swid_control_etclass |=
        (u16)TO_FIELD(TX_HDR_ETCLASS_MASK_V1,
                      TX_HDR_ETCLASS_SHIFT_V1, specific_meta.etclass);

    p_hdr->swid_control_etclass = cpu_to_be16(p_hdr->swid_control_etclass);

    p_hdr->system_port_mid |=
        (u16)TO_FIELD(TX_HDR_SYSTEM_PORT_MID_MASK,
                      TX_HDR_SYSTEM_PORT_MID_SHIFT,
                      meta->system_port_mid);

    p_hdr->system_port_mid = cpu_to_be16(p_hdr->system_port_mid);

    p_hdr->fid |=
        (u16)TO_FIELD(TX_HDR_FID_MASK_V1,
                      TX_HDR_FID_SHIFT_V1,
                      specific_meta.fid);
    p_hdr->fid = cpu_to_be16(p_hdr->fid);

    p_hdr->type |=
        TO_FIELD(TX_HDR_TYPE_MASK,
                 TX_HDR_TYPE_SHIFT, specific_meta.type);

    if (tx_debug) {
        printk(KERN_DEBUG PFX "sx_core_post_send: Sending "
               "pkt with specific meta: "
               "specific_meta.version: %d , swid: %d , sysport:0x%x, specific_meta.ctl: %d,"
               "specific_meta.protocol: %d, specific_meta.rx_is_router:%d, type: %d, "
               "specific_meta.fid_valid: %d,  specific_meta.use_control_tclass: %d, specific_meta.etclass :%d,"
               "specific_meta.fid: %d\n",
               specific_meta.version, meta->swid,
               meta->system_port_mid,
               specific_meta.ctl, specific_meta.protocol, specific_meta.rx_is_router, specific_meta.type,
               specific_meta.fid_valid, specific_meta.use_control_tclass, specific_meta.etclass, specific_meta.fid);
    }

    return 0;
}

/**
 * This functions will send emads according emad_path defined in DPT : I2C,PCI,SGMII.
 */
int sx_dpt_send_emad(int sx_dev_id, struct sk_buff *skb, struct isx_meta *meta)
{
    int            err = 0;
    struct sx_dev *dev = sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info.sx_dev;

    ENTER_FUNC();

    DPRINTK(KERN_DEBUG "send emad to dev_id %d, skb: %p\n", sx_dev_id, skb);
    switch (sx_glb.sx_dpt.dpt_info[sx_dev_id].emad_path) {
    case DPT_PATH_I2C:
        PRINTK_ERR("Send emad over I2C isn't supported.\n");
        err = -EINVAL;
        break;

    case DPT_PATH_SGMII:
        err = sx_build_isx_header(meta, skb, 0);
        if (err) {
            if (printk_ratelimit()) {
                printk(KERN_WARNING PFX "sx_dpt_send_emad: "
                       "error in build header/stub\n");
            }
            sx_skb_free(skb);
            break;
        }

        err = sx_glb.sx_sgmii.build_ctrl_segment(skb, meta->lp, 0, SX_SGMII_PKT_TYPE_ETH);
        if (err) {
            sx_skb_free(skb);
            break;
        }

        err = sx_glb.sx_sgmii.build_encapsulation_header(skb,
                                                         sx_glb.sx_dpt.dpt_info[sx_dev_id] \
                                                         .sx_sgmii_info.dmac,
                                                         sx_glb.sx_sgmii.base_smac,
                                                         ETHTYPE_EMAD, 0, 0);
        if (err) {
            sx_skb_free(skb);
            break;
        }
#ifdef SX_DEBUG
        printk(KERN_DEBUG PFX "Going to send an EMAD through SGMII "
               "to device %d\n", sx_dev_id);
#endif
        err = sx_glb.sx_sgmii.send(skb);
        break;

    case DPT_PATH_PCI_E:
        DPRINTK("Send emad over PCI-E.\n");
        err = __sx_core_post_send(dev, skb, meta);
        break;

    default:
        PRINTK_ERR("dpt_info[%d].emad_path: %d is invalid\n",
                   sx_dev_id,
                   sx_glb.sx_dpt.dpt_info[sx_dev_id].emad_path);
        err = -EINVAL;
        break;
    }

    EXIT_FUNC();
    return err;
}

/**
 * This functions will send mads according mad_path defined in DPT :
 * I2C,PCI,SGMII.
 */
int sx_dpt_send_mad(int sx_dev_id, struct sk_buff *skb, struct isx_meta *meta)
{
    int            err = 0;
    struct sx_dev *dev = sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info.sx_dev;

    ENTER_FUNC();

    DPRINTK("send mad to dev_id %d, skb: %p\n", sx_dev_id, skb);
    switch (sx_glb.sx_dpt.dpt_info[sx_dev_id].mad_path) {
    case DPT_PATH_I2C:
        PRINTK_ERR("Send mad over I2C isn't supported.\n");
        err = -EINVAL;
        break;

    case DPT_PATH_SGMII:
        err = sx_build_isx_header(meta, skb, 0);
        if (err) {
            if (printk_ratelimit()) {
                printk(KERN_WARNING PFX "sx_dpt_send_emad: "
                       "error in build header/stub\n");
            }
            sx_skb_free(skb);
            break;
        }

        err = sx_glb.sx_sgmii.build_ctrl_segment(skb, meta->lp, 0,
                                                 SX_SGMII_PKT_TYPE_IB_TRANSPORT);
        if (err) {
            sx_skb_free(skb);
            break;
        }

        err = sx_glb.sx_sgmii.build_encapsulation_header(skb,
                                                         sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_sgmii_info.dmac,
                                                         sx_glb.sx_sgmii.base_smac,
                                                         ETHTYPE_EMAD, 0, 0);
        if (err) {
            sx_skb_free(skb);
            break;
        }

        printk(KERN_DEBUG PFX "Going to send a MAD through SGMII to "
               "device %d\n", sx_dev_id);
        err = sx_glb.sx_sgmii.send(skb);
        break;

    case DPT_PATH_PCI_E:
        DPRINTK("Send mad over PCI-E\n");
        err = __sx_core_post_send(dev, skb, meta);
        break;

    default:
        PRINTK_ERR("dpt_info[%d].mad_path: %d is INVALID !\n",
                   sx_dev_id,
                   sx_glb.sx_dpt.dpt_info[sx_dev_id].mad_path);
        err = -EINVAL;
        break;
    }

    EXIT_FUNC();
    return err;
}

int sx_dpt_get_sx_dev_by_id(int sx_dev_id, struct sx_dev **dev)
{
    int ret = 0;

    ENTER_FUNC();    

#ifdef NO_PCI
    if (sx_glb.tmp_dev_ptr == NULL) {
        *dev = NULL;
        ret = -EINVAL;
		PRINTK_ERR("There is no device to configure for DPT\n");
        goto out;
    }

    *dev = sx_glb.tmp_dev_ptr;
    goto out;
#endif

#ifdef CONFIG_SX_SGMII_PRESENT
    /* for SXD_SIGNAL sw event handling on system close */
    if (sx_dev_id == DEFAULT_DEVICE_ID) {
    	*dev = NULL;
		goto out;
	}
#endif

    *dev = NULL;
    if (sx_dev_id > MAX_NUM_OF_REMOTE_SWITCHES) {
        PRINTK_ERR("Device id is not valid\n");
        ret = -EINVAL;
        goto out;
    }
    

    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_PCI_E] == true) {
        *dev = (struct sx_dev *)sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info.sx_dev;
    } else if (sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_SGMII] == true) {
        *dev = NULL;
    } else {
        PRINTK_ERR("no path configured for dev_id %d\n", sx_dev_id);
        ret = -ENXIO;
        goto out;
    }
    
out:
    EXIT_FUNC();
    return ret;
}

int sx_dpt_get_cmd_sx_dev_by_id(int sx_dev_id, struct sx_dev **dev)
{
    int ret = 0;

    ENTER_FUNC();

#ifdef NO_PCI
    if (sx_glb.tmp_dev_ptr == NULL) {
        *dev = NULL;
        ret = -EINVAL;
        goto out;
    }

    *dev = sx_glb.tmp_dev_ptr;
    goto out;
#endif

    *dev = NULL;
    if (sx_dev_id > MAX_NUM_OF_REMOTE_SWITCHES) {
        PRINTK_ERR("Device id is not valid\n");
        ret = -EINVAL;
        goto out;
    }

    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].cmd_path == DPT_PATH_PCI_E) {
        if (sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_PCI_E] == true) {
            *dev = (struct sx_dev *)sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_pcie_info.sx_dev;
        } else {
            PRINTK_ERR("no PCIE path configured for dev_id %d\n", sx_dev_id);
            ret = -ENXIO;
            goto out;
        }
    } else {
        *dev = sx_glb.tmp_dev_ptr;
    }

out:
    EXIT_FUNC();
    return ret;
}

int sx_dpt_get_i2c_dev_by_id(int sx_dev_id, int *i2c_dev)
{
    ENTER_FUNC();

    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_I2C] == false) {
        PRINTK_ERR("Path DPT_PATH_I2C is not valid.\n");
        *i2c_dev = 0;
        return -EINVAL;
    }

    *i2c_dev = sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_i2c_info.sx_i2c_dev;

    EXIT_FUNC();

    return 0;
}

int sx_dpt_get_sgmii_dev_mac(int sx_dev_id, u64 *mac)
{
    ENTER_FUNC();
    if (sx_glb.sx_dpt.dpt_info[sx_dev_id].is_ifc_valid[DPT_PATH_SGMII] == false) {
        PRINTK_ERR("Path DPT_PATH_SGMII is not valid.\n");
        *mac = 0;
        return -EINVAL;
    }

    *mac = sx_glb.sx_dpt.dpt_info[sx_dev_id].sx_sgmii_info.dmac;

    EXIT_FUNC();

    return 0;
}

int sx_dpt_validate_new_cmd_path(int sx_dev_id, enum  ku_dpt_path_type cmd_path)
{
    if (cmd_path == DPT_PATH_SGMII) {
        return sx_QUERY_FW_2(sx_glb.tmp_dev_ptr, sx_dev_id);
    }

    return 0;
}

int sx_dpt_validate_new_emad_path(int sx_dev_id)
{
    int err = 0;

    ENTER_FUNC();
    EXIT_FUNC();

    return err;
}

int sx_dpt_validate_new_mad_path(int sx_dev_id)
{
    int err = 0;

    ENTER_FUNC();
    EXIT_FUNC();

    return err;
}

/**
 * reg - CR space adress to write to
 * value - the value we want to write
 */
int sx_dpt_i2c_writel(int dev_id, u32 reg, u32 value)
{
    int err = 0;
    int i2c_dev_id;
    int i;

    ENTER_FUNC();
    err = sx_dpt_get_i2c_dev_by_id(dev_id, &i2c_dev_id);
    if (err) {
        PRINTK_ERR("sx_dpt_get_i2c_dev_by_id for dev_id: %d "
                   "failed !\n", dev_id);
        return -EINVAL;
    }

    DPRINTK("dev_id: 0x%x, i2c_dev: 0x%x, reg:0x%x, val: 0x%x\n",
            dev_id, i2c_dev_id, reg, value);

    for (i = 0; i < MAX_I2C_RETRIES; i++) {
        err = sx_glb.sx_i2c.write_dword(i2c_dev_id, reg, value);
        if (!err) {
            break;
        }
    }

    if (err) {
        printk(KERN_WARNING "sx_dpt_i2c_writel for dev_id: %d failed "
               "after %d tries!\n", dev_id, i);
        return -EINVAL;
    }

    if (i > 0) {
        printk(KERN_INFO "sx_dpt_i2c_writel for dev_id: %d succeeded "
               "after %d tries!\n", dev_id, i);
    }

    EXIT_FUNC();
    return err;
}

int sx_dpt_i2c_write_buf(int dev_id, unsigned int i2c_offset, unsigned char *buf, int size)
{
    int err = 0;
    int i2c_dev_id;
    int i;

    ENTER_FUNC();
    err = sx_dpt_get_i2c_dev_by_id(dev_id, &i2c_dev_id);
    if (err) {
        PRINTK_ERR("sx_dpt_get_i2c_dev_by_id for dev_id: "
                   "%d failed !\n", dev_id);
        return -EINVAL;
    }

    DPRINTK("dev_id: 0x%x, i2c_dev_id: 0x%x, i2c_offset:0x%x, buf: %p, "
            "size: %d\n", dev_id, i2c_dev_id,
            i2c_offset, buf, size);
    for (i = 0; i < MAX_I2C_RETRIES; i++) {
        err = sx_glb.sx_i2c.write(i2c_dev_id, i2c_offset, size, buf);
        if (!err) {
            break;
        }
    }

    if (err) {
        printk(KERN_ERR "sx_dpt_i2c_write_buf for dev_id: %d "
               "failed after %d tries! err: %d\n", dev_id, i, err);
        return -EINVAL;
    }

    if (i > 0) {
        printk(KERN_INFO "sx_dpt_i2c_write_buf for dev_id: %d "
               "succeeded after %d tries!\n", dev_id, i);
    }

    EXIT_FUNC();
    return err;
}

int sx_dpt_i2c_read_buf(int dev_id, unsigned int i2c_offset, unsigned char *buf, int size)
{
    int err = 0;
    int i2c_dev_id;
    int i;

    ENTER_FUNC();
    err = sx_dpt_get_i2c_dev_by_id(dev_id, &i2c_dev_id);
    if (err) {
        PRINTK_ERR("sx_dpt_get_i2c_dev_by_id for dev_id: "
                   "%d failed !\n", dev_id);
        return -EINVAL;
    }

    DPRINTK("dev_id: 0x%x, i2c_dev_id: 0x%x, i2c_offset:0x%x, buf: %p, "
            "size: %d\n", dev_id, i2c_dev_id,
            i2c_offset, buf, size);

    for (i = 0; i < MAX_I2C_RETRIES; i++) {
        err = sx_glb.sx_i2c.read(i2c_dev_id, i2c_offset, size, buf);
        if (!err) {
            break;
        }
    }

    if (err) {
        PRINTK_ERR("sx_dpt_i2c_read_buf for dev_id: %d failed "
                   "after %d tries! err: %d \n", dev_id, i, err);
        return -EINVAL;
    }

    if (i > 0) {
        printk(KERN_INFO "sx_dpt_i2c_read_buf for dev_id: %d "
               "succeeded after %d tries!\n", dev_id, i);
    }

    EXIT_FUNC();
    return err;
}

u32 sx_dpt_i2c_readl(int dev_id, u32 address, int *err)
{
    int value = 0;
    int i2c_dev_id;
    int i;

    ENTER_FUNC();
    *err = sx_dpt_get_i2c_dev_by_id(dev_id, &i2c_dev_id);
    if (*err) {
        PRINTK_ERR("sx_dpt_get_i2c_dev_by_id for dev_id: %d "
                   "failed !\n", dev_id);
        return 0;
    }

    for (i = 0; i < MAX_I2C_RETRIES; i++) {
        *err = sx_glb.sx_i2c.read_dword(i2c_dev_id, address, &value);
        if (!(*err)) {
            break;
        }
    }

    if (*err) {
        printk(KERN_DEBUG "sx_dpt_i2c_readl for dev_id: %d failed "
               "after %d tries!\n", dev_id, i);
        return 0;
    }

    if (i > 0) {
        printk(KERN_INFO "sx_dpt_i2c_read_buf for dev_id: %d "
               "succeeded after %d tries!\n", dev_id, i);
    }

    EXIT_FUNC();
    return value;
}

void sx_dump_dpt_entry(int i)
{
    struct sx_dpt_info dpt_info = sx_glb.sx_dpt.dpt_info[i];

    printk(KERN_INFO "%d:   c:%s e:%s m:%s cr:%s ; ",
           i,
           dpt_type2str[dpt_info.cmd_path],
           dpt_type2str[dpt_info.emad_path],
           dpt_type2str[dpt_info.mad_path],
           dpt_type2str[dpt_info.cr_access_path]);

    if (sx_glb.sx_dpt.dpt_info[i].is_ifc_valid[DPT_PATH_I2C]) {
        printk(KERN_INFO "i2c:0x%x mbox in 0x%x out 0x%x",
               dpt_info.sx_i2c_info.sx_i2c_dev,
               dpt_info.in_mb_offset,
               dpt_info.out_mb_offset);
    }

    if (sx_glb.sx_dpt.dpt_info[i].is_ifc_valid[DPT_PATH_PCI_E]) {
        printk(KERN_INFO "pcie: %p ", dpt_info.sx_pcie_info.sx_dev);
    }

    if (sx_glb.sx_dpt.dpt_info[i].is_ifc_valid[DPT_PATH_SGMII]) {
        printk(KERN_INFO "sgmii: dmac 0x%llx smac 0x%llx mbox in 0x%x out 0x%x",
               dpt_info.sx_sgmii_info.dmac,
               sx_glb.sx_sgmii.base_smac,
               dpt_info.in_mb_offset,
               dpt_info.out_mb_offset);
    }

    printk(KERN_INFO "\n");
}

void sx_dpt_dump(void)
{
    int cnt = 0;
    int i;

    printk(KERN_INFO "DUMP DPT:\n");
    for (i = 0; i < MAX_NUM_OF_REMOTE_SWITCHES + 1; i++) {
        if (sx_glb.sx_dpt.dpt_info[i]. \
            is_ifc_valid[DPT_PATH_I2C] ||
            sx_glb.sx_dpt.dpt_info[i]. \
            is_ifc_valid[DPT_PATH_SGMII] ||
            sx_glb.sx_dpt.dpt_info[i]. \
            is_ifc_valid[DPT_PATH_PCI_E]) {
            sx_dump_dpt_entry(i);
            cnt++;
        }
    }

    printk(KERN_INFO "Active DPT entries : %d \n", cnt);
}

int sx_dpt_move(int dst_dev_id, int src_dev_id)
{
    struct sx_dev *tmp_sx_dev = NULL;

    if (dst_dev_id > (MAX_NUM_OF_REMOTE_SWITCHES - 1)) {
        printk(KERN_INFO "%s: dst_dev_id %d is out of range [0 .. %d] \n",
               __func__, dst_dev_id, (MAX_NUM_OF_REMOTE_SWITCHES - 1));
        return -EINVAL;
    }

    if (src_dev_id > (MAX_NUM_OF_REMOTE_SWITCHES - 1)) {
        printk(KERN_INFO "%s: src_dev_id %d is out of range [0 .. %d] \n",
               __func__, src_dev_id, (MAX_NUM_OF_REMOTE_SWITCHES - 1));
        return -EINVAL;
    }


    sx_glb.sx_dpt.dpt_info[dst_dev_id] = sx_glb.sx_dpt.dpt_info[src_dev_id];
    memset(&sx_glb.sx_dpt.dpt_info[src_dev_id], 0, sizeof(sx_glb.sx_dpt.dpt_info[src_dev_id]));
    sx_dpt_get_sx_dev_by_id(dst_dev_id, &tmp_sx_dev);
    tmp_sx_dev->device_id = dst_dev_id;

    printk(KERN_INFO "Move DPT entry %d to %d .\n", src_dev_id, dst_dev_id);
    return 0;
}

void sx_dpt_set_cmd_dbg(int dev_id, int path)
{
    sx_dpt_set_cmd_path(dev_id, path);
}

void sx_dpt_reg_i2c_ifc(struct sx_i2c_ifc *i2c_ifc)
{
    sx_glb.sx_i2c.read = __symbol_get("i2c_read");
    sx_glb.sx_i2c.write = __symbol_get("i2c_write");
    sx_glb.sx_i2c.read_dword = __symbol_get("i2c_read_dword");
    sx_glb.sx_i2c.write_dword = __symbol_get("i2c_write_dword");
    sx_glb.sx_i2c.enforce = __symbol_get("i2c_enforce");
    sx_glb.sx_i2c.release = __symbol_get("i2c_release");
    sx_glb.sx_i2c.get_local_mbox = __symbol_get("i2c_get_local_mbox");
    sx_glb.sx_i2c.get_fw_rev = __symbol_get("i2c_get_fw_rev");
    sx_glb.sx_i2c.set_go_bit_stuck = __symbol_get("i2c_set_go_bit_stuck");
}
EXPORT_SYMBOL(sx_dpt_reg_i2c_ifc);

void sx_dpt_dereg_i2c_ifc(void)
{
    if (!sx_glb.sx_i2c.read || (sx_glb.sx_i2c.read == sx_dpt_stub_i2c_read)) {
        return;
    }

    if (sx_glb.sx_i2c.read) {
        __symbol_put("i2c_read");
    }
    if (sx_glb.sx_i2c.write) {
        __symbol_put("i2c_write");
    }
    if (sx_glb.sx_i2c.read_dword) {
        __symbol_put("i2c_read_dword");
    }
    if (sx_glb.sx_i2c.write_dword) {
        __symbol_put("i2c_write_dword");
    }
    if (sx_glb.sx_i2c.enforce) {
        __symbol_put("i2c_enforce");
    }
    if (sx_glb.sx_i2c.release) {
        __symbol_put("i2c_release");
    }
    if (sx_glb.sx_i2c.get_local_mbox) {
        __symbol_put("i2c_get_local_mbox");
    }
    if (sx_glb.sx_i2c.get_fw_rev) {
        __symbol_put("i2c_get_fw_rev");
    }
    if (sx_glb.sx_i2c.set_go_bit_stuck) {
        __symbol_put("i2c_set_go_bit_stuck");
    }
}
EXPORT_SYMBOL(sx_dpt_dereg_i2c_ifc);

int sx_dpt_stub_i2c_enforce(int i2c_dev_id)
{
    PRINTK_ERR("i2c module non registered !!!\n");

    return 0;
}

int sx_dpt_stub_i2c_release(int i2c_dev_id)
{
    PRINTK_ERR("i2c module non registered !!!\n");

    return 0;
}

int sx_dpt_stub_i2c_write(int i2c_dev_id, int offset, int len, u8 *in_out_buf)
{
    PRINTK_ERR("i2c module non registered !!!\n");

    return 0;
}

int sx_dpt_stub_i2c_read(int i2c_dev_id, int offset, int len, u8 *in_out_buf)
{
    PRINTK_ERR("i2c module not registered !\n");

    return 0;
}

int sx_dpt_stub_i2c_write_dword(int i2c_dev_id, int offset, u32 val)
{
    PRINTK_ERR("i2c module not registered!\n");

    return 0;
}

int sx_dpt_stub_i2c_read_dword(int i2c_dev_id, int offset, u32 *val)
{
    PRINTK_ERR("i2c module not registered !\n");

    return 0;
}

int sx_dpt_cr_space_read(int dev_id, unsigned int address, unsigned char *buf, int size)
{
    enum ku_dpt_path_type   cr_access_path = sx_dpt_get_cr_access_path(dev_id);
    int                   err = 0;
    struct ku_dpt_i2c_info* sx_i2c_info;

    ENTER_FUNC();

    switch (cr_access_path){
    case (DPT_PATH_I2C):
		if (!sx_glb.sx_i2c.enforce) {
		    printk(KERN_ERR PFX "enforce is NULL!!!\n");
		    err = -EINVAL;
		    goto out;
		}

		err = sx_dpt_get_i2c_info(dev_id, &sx_i2c_info);
		if (err) {
		    printk(KERN_WARNING PFX "Can't get I2C info of device %d. "
		           "cr space read will not be performed. err = %d\n",
		           dev_id, err);
		    goto out;
		}
		err = sx_glb.sx_i2c.enforce(sx_i2c_info->sx_i2c_dev);
		if (err) {
		    printk(KERN_WARNING PFX "I2C bus of device %d is not ready. "
		           "cr space read will not be performed. err = %d\n",
		           dev_id, err);
		    goto out;
		}

		err = sx_dpt_i2c_read_buf(dev_id, address, buf, size);
		sx_glb.sx_i2c.release(sx_i2c_info->sx_i2c_dev);
		if (err) {
		        printk(KERN_WARNING PFX "I2C bus of device %d is not ready. "
		           "cr space read will not be performed. err = %d\n",
		           dev_id, err);
			goto out;
		}

		break;
#ifdef CONFIG_SX_SGMII_PRESENT
	case (DPT_PATH_SGMII):
        err = sx_sgmii_cr_space_read_buf(dev_id, address, buf, size);
        if (err) {
            printk(KERN_WARNING PFX "Can't read CR space value from device %d  "
                   "cr space read will not be performed. err = %d\n",
                   dev_id, err);
        }
        break;
#endif
    case (DPT_PATH_PCI_E):
        printk(KERN_WARNING PFX "sx_dpt_cr_space_read: reading from CR space "
               "of device %u over PCI isn't supported\n", dev_id);
        err = -EINVAL;
        break;
    
    default:
        printk(KERN_ERR PFX "sx_dpt_cr_space_read: Can't read from CR space "
               "of device %u because the DPT path isn't valid\n", dev_id);
        err = -EINVAL;
        break;
    }

out:
    EXIT_FUNC();
    return err;
}

int sx_dpt_cr_space_write(int dev_id, unsigned int address, unsigned char *buf, int size)
{
    enum ku_dpt_path_type   cr_access_path = sx_dpt_get_cr_access_path(dev_id);
    int                   err = 0;
    struct ku_dpt_i2c_info* sx_i2c_info;

    ENTER_FUNC();

    switch (cr_access_path) {
    case (DPT_PATH_I2C):
		if (!sx_glb.sx_i2c.enforce) {
		    printk(KERN_ERR PFX "enforce is NULL!!!\n");
		    err = -EINVAL;
		    goto out;
		}

		err = sx_dpt_get_i2c_info(dev_id, &sx_i2c_info);
		if (err) {
		    printk(KERN_WARNING PFX "Can't get I2C info of device %d. "
		           "cr space read will not be performed. err = %d\n",
		           dev_id, err);
		    goto out;
		}
		err = sx_glb.sx_i2c.enforce(sx_i2c_info->sx_i2c_dev);
		if (err) {
		    printk(KERN_WARNING PFX "I2C bus of device %d is not ready. "
		           "cr space write will not be performed. err = %d\n",
		           dev_id, err);
		    goto out;
		}

		err = sx_dpt_i2c_write_buf(dev_id, address, buf, size);
		sx_glb.sx_i2c.release(sx_i2c_info->sx_i2c_dev);
		if (err) {
		    printk(KERN_WARNING PFX "I2C bus of device %d is not ready. "
		           "cr space write will not be performed. err = %d\n",
		           dev_id, err);
		    goto out;
		}
        break;
#ifdef CONFIG_SX_SGMII_PRESENT
	case (DPT_PATH_SGMII):
        err = sx_sgmii_cr_space_write_buf(dev_id, address, buf, size);
        if (err) {
            printk(KERN_WARNING PFX "Can't write CR space value to device %d  "
                   "cr space write will not be performed. err = %d\n",
                   dev_id, err);
        }
        break;
#endif
    case (DPT_PATH_PCI_E):
        printk(KERN_WARNING PFX "sx_dpt_cr_space_write: reading from CR space "
               "of device %u over PCI isn't supported\n", dev_id);
        err = -EINVAL;
        break;

    default:
        printk(KERN_ERR PFX "sx_dpt_cr_space_write: Can't write to CR space "
               "of device %u because the DPT path isn't valid\n", dev_id);
        err = -EINVAL;
        break;
    }

out:
    EXIT_FUNC();
    return err;
}
