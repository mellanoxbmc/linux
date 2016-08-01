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

#include <linux/delay.h>
#include "sx.h"
#include "dq.h"
#include "alloc.h"

/* taken as is from the IS4 driver, we might need to remove the part where we
 * save the pci config headers before the reset, and restore them afterwards.
 * Depends on the decision of the FW guys. */
int sx_reset(struct sx_dev *dev)
{
	void __iomem *reset;
	u32 *hca_header = NULL;
	int pcie_cap;
	u16 devctl;
	u16 linkctl;
	u16 vendor = 0xffff;
	unsigned long end;
	u32 sem;
	int i;
	int err = 0;

	if (!dev->pdev) {
		sx_err(dev, "SW reset will not be executed since PCI device is not present");
	}

#define SX_RESET_BASE	0xf0000
#define SX_RESET_SIZE	0x404
#define SX_SEM_OFFSET	0x400
#define SX_SEM_BIT	(1 << 31)
#define SX_RESET_OFFSET	0x10
#define SX_RESET_VALUE	swab32(1)

#ifndef INCREASED_TIMEOUT
#define SX_SEM_TIMEOUT_JIFFIES		(10 * HZ)
#else
#define SX_SEM_TIMEOUT_JIFFIES		(100 * HZ)
#endif
#define SX_RESET_TIMEOUT_JIFFIES	(2 * HZ)

	/*
	 * Reset the chip.  This is somewhat ugly because we have to
	 * save off the PCI header before reset and then restore it
	 * after the chip reboots.  We skip config space offsets 22
	 * and 23 since those have a special meaning.
	 */

	/* Do we need to save off the full 4K PCI Express header?? */
	hca_header = kmalloc(256, GFP_KERNEL);
	if (!hca_header) {
		err = -ENOMEM;
		sx_err(dev, "Couldn't allocate memory to save HCA "
			  "PCI header, aborting.\n");
		goto out;
	}

	pcie_cap = pci_find_capability(dev->pdev, PCI_CAP_ID_EXP);

	for (i = 0; i < 64; ++i) {
		if (i == 22 || i == 23)
			continue;
		if (pci_read_config_dword(dev->pdev, i * 4, hca_header + i)) {
			err = -ENODEV;
			sx_err(dev, "Couldn't save HCA PCI header, aborting.\n");
			goto out;
		}
	}

	reset = ioremap(pci_resource_start(dev->pdev, 0) + SX_RESET_BASE,
			SX_RESET_SIZE);
	if (!reset) {
		err = -ENOMEM;
		sx_err(dev, "Couldn't map HCA reset register, aborting.\n");
		goto out;
	}

	/* grab HW semaphore to lock out flash updates */
	end = jiffies + SX_SEM_TIMEOUT_JIFFIES;
	do {
		sem = be32_to_cpu(readl(reset + SX_SEM_OFFSET)) & SX_SEM_BIT;
		if (!sem)
			break;

		msleep(1);
	} while (time_before(jiffies, end));

	if (sem) {
		sx_err(dev, "Failed to obtain HW semaphore, aborting\n");
		err = -EAGAIN;
		iounmap(reset);
		goto out;
	}

	/* actually hit reset */
	writel(SX_RESET_VALUE, reset + SX_RESET_OFFSET);
	iounmap(reset);

	/* Wait three seconds before accessing device */
#ifndef INCREASED_TIMEOUT
	msleep(3000);
#else
	msleep(180000);
#endif

	end = jiffies + SX_RESET_TIMEOUT_JIFFIES;
	do {
		if (!pci_read_config_word(dev->pdev, PCI_VENDOR_ID, &vendor) &&
		    vendor != 0xffff)
			break;

		msleep(1);
	} while (time_before(jiffies, end));

	if (vendor == 0xffff) {
		err = -ENODEV;
		sx_err(dev, "PCI device did not come back after reset, aborting.\n");
		goto out;
	}

	/* Now restore the PCI headers */
	if (pcie_cap) {
		devctl = hca_header[(pcie_cap + PCI_EXP_DEVCTL) / 4];
		if (pci_write_config_word(dev->pdev, pcie_cap + PCI_EXP_DEVCTL,
					   devctl)) {
			err = -ENODEV;
			sx_err(dev, "Couldn't restore HCA PCI Express "
				 "Device Control register, aborting.\n");
			goto out;
		}

		linkctl = hca_header[(pcie_cap + PCI_EXP_LNKCTL) / 4];
		if (pci_write_config_word(dev->pdev, pcie_cap + PCI_EXP_LNKCTL,
					   linkctl)) {
			err = -ENODEV;
			sx_err(dev, "Couldn't restore HCA PCI Express "
				 "Link control register, aborting.\n");
			goto out;
		}
	}

	for (i = 0; i < 16; ++i) {
		if (i * 4 == PCI_COMMAND)
			continue;

		if (pci_write_config_dword(dev->pdev, i * 4, hca_header[i])) {
			err = -ENODEV;
			sx_err(dev, "Couldn't restore HCA reg %x, aborting.\n", i);
			goto out;
		}
	}

	if (pci_write_config_dword(dev->pdev, PCI_COMMAND,
				   hca_header[PCI_COMMAND / 4])) {
		err = -ENODEV;
		sx_err(dev, "Couldn't restore HCA COMMAND, aborting.\n");
		goto out;
	}

out:
	kfree(hca_header);

	return err;
}
