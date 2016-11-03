/********************************************************************************
* File Name     : drivers/usb/host/ehci-aspeed.c 
* Author         : Ryan Chen
* Description   : EHCI HCD (Host Controller Driver) for USB
* 
* Copyright (C) ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
*
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
*
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
    
*   History      : 
*    1. 2012/08/17 Ryan Chen created this file 
* 
********************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include "ehci.h"

/* ASPEED EHCI USB Host Controller */

#define DRIVER_DESC "ASpeed On-Chip EHCI Host Controller"

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

static const struct of_device_id of_ast_ehci_ids[] = {
	{ .compatible = "aspeed,ast2500-ehci", },
	{}
};
MODULE_DEVICE_TABLE(of, of_ast_ehci_ids);

static struct hc_driver __read_mostly ehci_ast_hc_driver;

static int ehci_ast_setup(struct usb_hcd *hcd);

static const struct ehci_driver_overrides ehci_ast_driver_orverrides __initconst = {
	.reset = ehci_ast_setup,
};

static int ehci_ast_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	ehci->caps = hcd->regs;

#ifdef CONFIG_USB_EHCI_ROOT_HUB_TT
	hcd->has_tt = 1;
#else
	hcd->has_tt = 0;
#endif

	retval = ehci_setup(hcd);
	if (retval)
		return retval;

	return retval;
}

static int ehci_ast_drv_probe(struct platform_device *pdev)
{
		struct resource *res;
		struct usb_hcd *hcd;
		void __iomem *regs;
		int irq, err;

		if (usb_disabled())
			return -ENODEV;

		dev_info(&pdev->dev, "Initializing ASPEED-SoC USB Host Controller\n");
	
		irq = platform_get_irq(pdev, 0);
		if (irq <= 0) {
			dev_err(&pdev->dev,
				"Found HC with no IRQ. Check %s setup!\n",
				dev_name(&pdev->dev));
			err = -ENODEV;
			goto err1;
		}

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			dev_err(&pdev->dev,
				"Found HC with no register addr. Check %s setup!\n",
				dev_name(&pdev->dev));
			err = -ENODEV;
			goto err1;
		}

		if (!request_mem_region(res->start, resource_size(res), res->name)) {
			dev_dbg(&pdev->dev, "controller already in use\n");
			err = -EBUSY;
			goto err1;
		}

		regs = ioremap_nocache(res->start, resource_size(res));
		if (regs == NULL) {
			dev_dbg(&pdev->dev, "error mapping memory\n");
			err = -EFAULT;
			goto err2;
		}

		hcd = usb_create_hcd(&ehci_ast_hc_driver, &pdev->dev, dev_name(&pdev->dev));
		if (!hcd) {
			err = -ENOMEM;
			goto err3;
		}

		hcd->rsrc_start = res->start;
		hcd->rsrc_len = resource_size(res);
		hcd->regs = regs;

		err = usb_add_hcd(hcd, irq, 0);
		if (err)
			goto err4;

		return 0;
	
err4:
		usb_put_hcd(hcd);
err3:
		iounmap(regs);
err2:
		release_mem_region(res->start, resource_size(res));
err1:
		dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), err);
	
		return err;
}

static int ehci_ast_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

	return 0;	
}

 /*TBD*/
#if 0 /*CONFIG_PM*/
static int ehci_hcd_ast_drv_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	unsigned long		flags;
	int			rc = 0;

	if (time_before(jiffies, ehci->next_statechange))
		msleep(10);

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave (&ehci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		rc = -EINVAL;
		goto bail;
	}
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void)ehci_readl(ehci, &ehci->regs->intr_enable);

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
 bail:
	spin_unlock_irqrestore (&ehci->lock, flags);

	// could save FLADJ in case of Vaux power loss
	// ... we'd only use it to handle clock skew

	return rc;
}
static int ehci_hcd_ast_drv_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);

	// maybe restore FLADJ

	if (time_before(jiffies, ehci->next_statechange))
		msleep(100);

	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	usb_root_hub_lost_power(hcd->self.root_hub);

	/* Else reset, to cope with power loss or flush-to-storage
	 * style "resume" having let BIOS kick in during reboot.
	 */
	(void) ehci_halt(ehci);
	(void) ehci_reset(ehci);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
//	if (ehci->reclaim)
//		end_unlink_async(ehci);
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */

	/* here we "know" root ports should always stay powered */
	ehci_port_power(ehci, 1, true);

	hcd->state = HC_STATE_SUSPENDED;
	return 0;
}
#endif /* TBD */

MODULE_ALIAS("platform:ehci_ast");

static struct platform_driver ehci_hcd_ast_driver = {
	.probe		= ehci_ast_drv_probe,
	.remove		= ehci_ast_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
#if 0 /*CONFIG_PM*/
	.suspend		= ehci_hcd_ast_drv_suspend,
	.resume		= ehci_hcd_ast_drv_resume,
#endif	
	.driver = {
		.name					= "ehci-ast",
		.of_match_table	= of_ast_ehci_ids,
	},
};

static int __init ehci_ast_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " DRIVER_DESC "\n", ehci_hcd_ast_driver.driver.name);

	ehci_init_driver(&ehci_ast_hc_driver, &ehci_ast_driver_orverrides);
	return platform_driver_register(&ehci_hcd_ast_driver);
}
module_init(ehci_ast_init);

static void __exit ehci_ast_cleanup(void)
{
	platform_driver_unregister(&ehci_hcd_ast_driver);
}
module_exit(ehci_ast_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("ASpeed");
MODULE_LICENSE("GPL");
