/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2019 MediaTek Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// file    mdrv_gpio_io.c
/// @brief  GPIO Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////


//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/gpio.h>
#include <linux/vmalloc.h>
#include <linux/version.h>

#include "mst_devid.h"
#include "mdrv_gpio_io.h"
#include "mhal_gpio_reg.h"
#include "mhal_gpio.h"
#include "mdrv_gpio.h"


//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define GPIO_DBG_ENABLE		1
#if GPIO_DBG_ENABLE
#define GPIO_PRINT(fmt, args...)        printk("[GPIO][%05d] " fmt, __LINE__, ## args)
#else
#define GPIO_PRINT(fmt, args...)
#endif
#define GPIO_MAX_PIN_NUMBER 200
#define GPIO_MAX_RESERVED_PIN_NUMBER 20 /* reserved pins that should not be exposed to sysfs. E.g. swi2c pads */
struct mstar_gpio_dev
{
	struct platform_device* pdev;
	struct irq_domain *irq_domain;
	u32 eint_num;
	spinlock_t lock;
	u32 *pin_status;
	u32 ngpio;
};

u32 * pin_disable;
//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------

static int mstar_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	int value = MHal_GPIO_Pad_Level(offset);
	//GPIO_PRINT(KERN_EMERG "[%s] gpio_%d: value=%d\n", __FUNCTION__, offset, value);
	return value;
}

static void mstar_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
    //GPIO_PRINT(KERN_EMERG "[%s] gpio_%d: value=%d\n", __FUNCTION__, offset, value);
    if (value) //high
    {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,10,86)
        if (value == 2 && pin_disable)
        {
            pin_disable[offset] = 1;
        }
        else
#endif
        {
            MHal_GPIO_Set_High(offset);
        }
    }
    else
    {
        MHal_GPIO_Set_Low(offset);
    }
}

static int mstar_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	//MHal_GPIO_Pad_Odn(offset);
	MHal_GPIO_Set_Input(offset);
	return 0;
}

static int mstar_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	//mstar_gpio_set(chip, offset, value);
	MHal_GPIO_Pad_Oen(offset);
	return 0;
}

static int mstar_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return MHal_GPIO_Get_Interrupt_Num(offset);
}

static int mstar_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
        return MHal_GPIO_Pad_InOut(offset);
}

static void mstar_gpio_free(struct gpio_chip *chip, unsigned offset)
{
        //pin_disable[offset] = 1;
        return;
}

static struct gpio_chip mstar_gpio_chip = {
	.label			= "gpio",
	.to_irq 			= mstar_gpio_to_irq,
        .free                   = mstar_gpio_free,
	.get_direction          = mstar_gpio_get_direction,
        .direction_input	= mstar_gpio_direction_input,
	.direction_output	= mstar_gpio_direction_output,
	.set				= mstar_gpio_set,
	.get				= mstar_gpio_get,
	.base			= 0,
};


static int mstar_gpio_dt_parse_reserved_pads(int *reserved_pads)
{
	u32 sda_pad;
	u32 scl_pad;
	u32 gpio_tmp;
	u32 wifi_ctl_gpio;
	int i = 0;
	int rsv = 0;
	struct device_node *dn;

	if (!reserved_pads)
		return -EINVAL;

	for (i = 0 ; i < GPIO_MAX_RESERVED_PIN_NUMBER; i++)
		reserved_pads[i] = -1;

	/* obtain pads that are used by swi2c */
	for_each_compatible_node(dn, NULL, "mstar,swi2c")
	{
		if (rsv >= GPIO_MAX_RESERVED_PIN_NUMBER)
			break;

		if (0 != of_property_read_u32(dn, "sda-gpio", &sda_pad) ||
			0 != of_property_read_u32(dn, "scl-gpio", &scl_pad) )
		{
			GPIO_PRINT("failed to obtain pads reserved for swi2c\n");
			continue;
		}

		reserved_pads[rsv] = sda_pad;
		reserved_pads[rsv+1] = scl_pad;
		rsv += 2;
	}
#if defined (CONFIG_SWITCH_GPIO)
        /* obtain pads that are used by switch */
	for_each_compatible_node(dn, NULL, "mstar,switch-gpio")
	{
		if (rsv >= GPIO_MAX_RESERVED_PIN_NUMBER)
			break;

		if (0 != of_property_read_u32(dn, "switch-gpio", &gpio_tmp))
		{
			GPIO_PRINT("failed to obtain pads reserved for switch-gpio\n");
			continue;
		}

		reserved_pads[rsv] = gpio_tmp;
		rsv += 1;
	}
#endif

	/* obtain pads that are used by wifi */
	for_each_compatible_node(dn, NULL, "mstar,gpio-wifi-ctl")
	{
		if (rsv >= GPIO_MAX_RESERVED_PIN_NUMBER)
			break;

		if (0 != of_property_read_u32(dn, "wifi-ctl-gpio", &wifi_ctl_gpio))
		{
			GPIO_PRINT("failed to obtain pads reserved for wifi control gpio\n");
			continue;
		}

		reserved_pads[rsv] = wifi_ctl_gpio;
		rsv += 1;
	}

	return 0;

}

#if defined(CONFIG_OF)
static struct of_device_id mstar_gpio_of_device_ids[] = {
		{.compatible = "mstar,gpio"},
		{},
};
#endif

static int mstar_gpio_drv_probe(struct platform_device *pdev)
{
	int retval = 0;
	int pad;
	int ngpio;
	int reserved_pads[GPIO_MAX_RESERVED_PIN_NUMBER];
	struct mstar_gpio_dev *gpio_dev;
        struct device_node *dn;
        struct property *prop;
        u32 *skipsr_pad;
        int length;
        #if defined(CONFIG_OF)
	const struct of_device_id *match;
	match = of_match_device(mstar_gpio_of_device_ids, &pdev->dev);
	if (!match)
	{
		dev_err(&pdev->dev, "Failed to find gpio controller\n");
		return -ENODEV;
	}
        #endif

	gpio_dev = devm_kzalloc(&pdev->dev, sizeof(struct mstar_gpio_dev), GFP_KERNEL);
	if (!gpio_dev)
	{
		dev_err(&pdev->dev, "failed to alloc memory for gpio!\n");
		return -ENOMEM;
	}

	/* get gpio allocated irq count, INT_COUNT is statically allocated by hal layer */
	gpio_dev->eint_num = INT_COUNT;
	if (gpio_dev->eint_num == 0)
	{
		dev_err(&pdev->dev, "Couldn't determine # GPIO Pins uses IRQ\n");
		//return -ENOENT;
	}

	ngpio = MHal_GPIO_Get_Pins_Count();
	BUG_ON(ngpio <= 0);

	gpio_dev->pdev = pdev;
	gpio_dev->ngpio = ngpio;
	gpio_dev->pin_status = vmalloc(sizeof(u32) * ngpio * 2); // OEN, OUT
        pin_disable = vmalloc(sizeof(u32) * ngpio);
	memset(pin_disable, 0, sizeof(u32) * ngpio);
	if (!gpio_dev->pin_status)
	{
		dev_err(&pdev->dev, "failed to alloc memory for gpio!\n");
		return -ENOMEM;
	}
#if defined(CONFIG_OF)
	retval = mstar_gpio_dt_parse_reserved_pads(reserved_pads);
	if(retval < 0)
	{
		dev_err(&pdev->dev, "failed to obtain GPIO reserved pads\n");
	}
    dn = pdev->dev.of_node;
    if (dn)
    {
        prop = of_find_property(dn, "skip-save-status", &length);
        if (!prop)
        {
            printk(KERN_ERR "no skip-save-status gpios\n");
            goto load_skip_save_status_gpio_end;
        }
        //printk("skip-save_status gpio numbers: %d", __FUNCTION__, __LINE__, length/sizeof(u32));
        if (length > 0)
        {
            length = length / sizeof(u32);
            skipsr_pad = devm_kzalloc(&pdev->dev, sizeof(u32) * length, GFP_KERNEL);
            if (!skipsr_pad)
            {
                printk("skipsr_pad is null\n");
                goto load_skip_save_status_gpio_end;
            }

            retval = of_property_read_u32_array(dn, "skip-save-status", skipsr_pad, length);
            if (retval < 0)
            {
                printk("get skip-save-status gpio failed\n");
                devm_kfree(&pdev->dev, skipsr_pad);
                goto load_skip_save_status_gpio_end;
            }

            for (pad=0; pad<length; pad++)
            {
                //printk("skip-save-status gpio[%d]: %d\n", i, skipsr_pad[i]);
                if (skipsr_pad[pad] < ngpio)
                {
                    pin_disable[skipsr_pad[pad]] = 1;
                }
            }
#if 0
            for (pad=0; pad < ngpio; pad++)
            {
                if (pin_disable[pad] == 1)
                    printk("[GPIO] skip-save-status gpio idx : %d\n", pad);
            }
#endif
            devm_kfree(&pdev->dev, skipsr_pad);
        }
    }
load_skip_save_status_gpio_end:
#else
        int *data = dev_get_platdata(&pdev->dev);
        if (!data){
                dev_err(&pdev->dev, "could not get resource\n");
                return -EINVAL;
        }
        memcpy(reserved_pads,data,sizeof(reserved_pads));
#endif
	platform_set_drvdata(pdev, gpio_dev);

#if defined(CONFIG_OF_GPIO)
        mstar_gpio_chip.of_node = pdev->dev.of_node;
#endif

	mstar_gpio_chip.ngpio = ngpio;
	gpio_dev->irq_domain = irq_domain_add_tree(pdev->dev.of_node,
							&irq_domain_simple_ops,
							gpio_dev);
	if (!gpio_dev->irq_domain)
	{
		dev_err(&pdev->dev, "Couldn't allocate IRQ domain\n");
		retval = -ENXIO;
		goto err_irq_domain;
	}
	/* create irq mapping */
	for (pad= 0; pad< gpio_dev->eint_num; pad++)
	{
		int irq_pad = (int)gpio_IntPad[pad];
		irq_create_mapping(gpio_dev->irq_domain, irq_pad);
		/*let arm gic handle the polarity and mask*/
	}
	gpiochip_add(&mstar_gpio_chip);

	spin_lock_init(&gpio_dev->lock);
	/* export all pins except reserved to sysfs */
	for (pad = 0; pad < ngpio; pad++)
	{
		int rsv;
		/* linear search takes long time if GPIO_MAX_RESERVED_PIN_NUMBER is very large,
		consider refine the implementation if the number is larger than 50 */
		for (rsv = 0; rsv < GPIO_MAX_RESERVED_PIN_NUMBER; rsv++)
		{
			if (reserved_pads[rsv] == pad)
				break;
		}
		if(rsv < GPIO_MAX_RESERVED_PIN_NUMBER)
			continue;

		retval = gpio_request(pad, "sysfs");
		if (retval < 0)
			continue;
		gpio_export(pad, 1);
	}

#if 0
	mstar_gpio_test_irq();
#endif
	return 0;

err_irq_domain:
	irq_domain_remove(gpio_dev->irq_domain);

	return retval;
}

static int mstar_gpio_drv_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mstar_gpio_dev *gpio_dev;
	u32 pin_cnt = 0;
	int ret = 0;

	GPIO_PRINT("%s is invoked\n", __FUNCTION__);

	gpio_dev = (struct mstar_gpio_dev *)platform_get_drvdata(pdev);

        #if defined(CONFIG_MSTAR_GPIO_SUSPEND)
        /* store gpio pins status */
	BUG_ON(gpio_dev->pin_status == NULL);

	ret = MHal_GPIO_Get_Pin_Status_Array(gpio_dev->pin_status, gpio_dev->ngpio, &pin_cnt );
	//BUG_ON(gpio_dev->pin_status == 0);
        #endif

	return ret;
}

static int mstar_gpio_drv_resume(struct platform_device *pdev)
{
	struct mstar_gpio_dev *gpio_dev;
	u32 pin_cnt = 0;
	int ret;

	GPIO_PRINT("%s is invoked\n", __FUNCTION__);

	gpio_dev = (struct mstar_gpio_dev *)platform_get_drvdata(pdev);

        #if defined(CONFIG_MSTAR_GPIO_SUSPEND)
	BUG_ON(gpio_dev->pin_status == NULL);
	/* restore gpio pins status */
	ret = MHal_GPIO_Set_Pin_Status_Array(gpio_dev->pin_status, gpio_dev->ngpio, &pin_cnt, pin_disable);
	//BUG_ON(gpio_dev->pin_status == 0);
        #endif
	/* TODO: re-expose pins to sysfs? */
	return 0;
}

static int mstar_gpio_drv_remove(struct platform_device *pdev)
{
	if( !(pdev->name) || strcmp(pdev->name,"Mstar-gpio")
		|| pdev->id!=0)
	{
		return -1;
	}
	pdev->dev.platform_data = NULL;
	return 0;
}

static struct platform_driver Mstar_gpio_driver = {
	.probe			= mstar_gpio_drv_probe,
	.remove 		= mstar_gpio_drv_remove,
	.suspend		= mstar_gpio_drv_suspend,
	.resume 		= mstar_gpio_drv_resume,

	.driver = {
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(mstar_gpio_of_device_ids),
#endif
		.name 	= "Mstar-gpio",
		.owner 	= THIS_MODULE,
	}
};

static int __init mstar_gpio_drv_init_module(void)
{
	return platform_driver_register(&Mstar_gpio_driver);
}

static void __exit mstar_gpio_drv_exit_module(void)
{
	platform_driver_unregister(&Mstar_gpio_driver);
}

postcore_initcall(mstar_gpio_drv_init_module);
module_exit(mstar_gpio_drv_exit_module);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("GPIO driver");
MODULE_LICENSE("GPL");

