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

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include "mdrv_wifi_ctl.h"
#include "mdrv_gpio.h"

//-------------------------------------------------------------------------------------------------
// Define & data type
//-------------------------------------------------------------------------------------------------
#define MTK_WIFI_CTL_NAME           "mstar,gpio-wifi-ctl"
#define MTK_WIFI_RESET_DELAY_MS     5
#define MTK_WIFI_GPIO_INVALID       0xFFFF

static MTK_Wifi_Ctl_Parm wifi_ctl;
static MTK_Wifi_Ctl_Parm wifi_power_ctl;

static int wifi_ctl_notify_handler(struct notifier_block* nb, unsigned long event, void *unused)
{
    MTK_WIFI_CTL_INFO("%s, event = %lu\n", __FUNCTION__, event);

    switch (event)
    {
        case SYS_DOWN:
        case SYS_POWER_OFF:
            if (wifi_ctl.invert)
                MDrv_GPIO_Set_Low(wifi_ctl.gpio);
            else
                MDrv_GPIO_Set_High(wifi_ctl.gpio);
            break;

        default:
            break;

    }

    return NOTIFY_OK;
}

static struct notifier_block wifi_ctl_notifier = {
    .notifier_call = wifi_ctl_notify_handler,
};

static void mtk_wifi_power_on(bool flag)
{
    MTK_WIFI_CTL_INFO("%s, power control gpio = %lu, invert = %u\n", __FUNCTION__, wifi_power_ctl.gpio, wifi_power_ctl.invert);

    if (flag)
    {
        //power on
        if(wifi_power_ctl.invert)
            MDrv_GPIO_Set_Low(wifi_power_ctl.gpio);
        else
            MDrv_GPIO_Set_High(wifi_power_ctl.gpio);
    }
    else
    {
         //power off
        if(wifi_power_ctl.invert)
            MDrv_GPIO_Set_High(wifi_power_ctl.gpio);
        else
            MDrv_GPIO_Set_Low(wifi_power_ctl.gpio);
    }
    msleep(wifi_power_ctl.delay_ms);
}

static void mtk_wifi_reset(void)
{
    MTK_WIFI_CTL_INFO("%s, gpio = %lu, invert = %u\n", __FUNCTION__, wifi_ctl.gpio, wifi_ctl.invert);

    if (wifi_ctl.invert)
    {
        MDrv_GPIO_Set_Low(wifi_ctl.gpio);
        msleep(wifi_ctl.delay_ms);
        MDrv_GPIO_Set_High(wifi_ctl.gpio);
    }
    else
    {
        MDrv_GPIO_Set_High(wifi_ctl.gpio);
        msleep(wifi_ctl.delay_ms);
        MDrv_GPIO_Set_Low(wifi_ctl.gpio);
    }
}

static int mtk_wifi_ctl_probe(struct platform_device *pdev)
{
    MTK_WIFI_CTL_DBG("%s\n", __FUNCTION__);
    u32 value = 0;

    wifi_ctl.gpio=MTK_WIFI_GPIO_INVALID;
    wifi_power_ctl.gpio=MTK_WIFI_GPIO_INVALID;
#if defined (CONFIG_OF)
    if (of_property_read_u32(pdev->dev.of_node, "wifi-ctl-gpio", &wifi_ctl.gpio))
    {
        MTK_WIFI_CTL_ERR("Fail to get gpio from DTS\n");
        return -EINVAL;
    }

    if (of_property_read_u32(pdev->dev.of_node, "wifi-ctl-delayms", &wifi_ctl.delay_ms))
    {
        MTK_WIFI_CTL_WARN("Use default delay time\n");
        wifi_ctl.delay_ms = MTK_WIFI_RESET_DELAY_MS;
    }

    if (of_property_read_u32(pdev->dev.of_node, "wifi-ctl-invert", &value))
    {
        MTK_WIFI_CTL_WARN("Use default invert setting\n");
        wifi_ctl.invert = FALSE;
    }
    else
    {
        if (value)
            wifi_ctl.invert = TRUE;
        else
            wifi_ctl.invert = FALSE;
    }

    if (of_property_read_u32(pdev->dev.of_node, "wifi-ctl-powerdown", &value))
    {
        MTK_WIFI_CTL_WARN("Default disable wifi power down\n");
        wifi_ctl.PowerDown = FALSE;

    }
    else
    {
        if (value)
            wifi_ctl.PowerDown = TRUE;
        else
            wifi_ctl.PowerDown = FALSE;
    }

    if (wifi_ctl.PowerDown)
    {
        MTK_WIFI_CTL_DBG("Register reboot notifier\n");
        register_reboot_notifier(&wifi_ctl_notifier);
    }

    MTK_WIFI_CTL_DBG("gpio = %lu\n", wifi_ctl.gpio);
    MTK_WIFI_CTL_DBG("delay ms = %lu\n", wifi_ctl.delay_ms);
    MTK_WIFI_CTL_DBG("invert = %u\n", wifi_ctl.invert);
    MTK_WIFI_CTL_DBG("PowerDown = %u\n", wifi_ctl.PowerDown);

    if (of_property_read_u32(pdev->dev.of_node, "wifi-power-ctl-gpio", &wifi_power_ctl.gpio))
    {
        MTK_WIFI_CTL_ERR("Fail to get power control gpio from DTS\n");
        wifi_power_ctl.gpio=MTK_WIFI_GPIO_INVALID;
    }
    else
    {
        if (of_property_read_u32(pdev->dev.of_node, "wifi-power-ctl-delayms", &wifi_power_ctl.delay_ms))
        {
            MTK_WIFI_CTL_WARN("Use power default delay time\n");
            wifi_power_ctl.delay_ms = MTK_WIFI_RESET_DELAY_MS;
        }

        if (of_property_read_u32(pdev->dev.of_node, "wifi-power-ctl-invert", &value))
        {
            MTK_WIFI_CTL_WARN("Use power default invert setting\n");
            wifi_power_ctl.invert = FALSE;
        }
        else
        {
            if (value)
                wifi_power_ctl.invert = TRUE;
            else
                wifi_power_ctl.invert = FALSE;
        }
        MTK_WIFI_CTL_DBG("power control gpio = %lu\n", wifi_power_ctl.gpio);
        MTK_WIFI_CTL_DBG("delay ms = %lu\n", wifi_power_ctl.delay_ms);
        MTK_WIFI_CTL_DBG("invert = %u\n", wifi_power_ctl.invert);
    }
#endif

    if(wifi_power_ctl.gpio!=MTK_WIFI_GPIO_INVALID)
    {
       mtk_wifi_power_on(TRUE);
    }
    /* Do wifi reset */
    if(wifi_ctl.gpio!=MTK_WIFI_GPIO_INVALID)
    {
        mtk_wifi_reset();
    }

    return 0;
}

static int mtk_wifi_ctl_remove(struct platform_device *pdev)
{
    MTK_WIFI_CTL_DBG("%s\n", __FUNCTION__);

    return 0;
}

static int mtk_wifi_ctl_suspend(struct platform_device *dev, pm_message_t state)
{
    MTK_WIFI_CTL_DBG("%s\n", __FUNCTION__);

    if(wifi_power_ctl.gpio!=MTK_WIFI_GPIO_INVALID)
    {
       mtk_wifi_power_on(FALSE);
    }
    return 0;
}

static int mtk_wifi_ctl_resume(struct platform_device *dev)
{
    MTK_WIFI_CTL_DBG("%s\n", __FUNCTION__);
    if(wifi_power_ctl.gpio!=MTK_WIFI_GPIO_INVALID)
    {
       mtk_wifi_power_on(TRUE);
    }
    return 0;
}

#if defined (CONFIG_OF)
static struct of_device_id mtk_wifi_ctl_of_device_ids[] = {
     {.compatible = MTK_WIFI_CTL_NAME},
     {},
};
#endif
static struct platform_driver mtk_wifi_ctl_driver = {
    .probe      = mtk_wifi_ctl_probe,
    .remove     = mtk_wifi_ctl_remove,
    .suspend    = mtk_wifi_ctl_suspend,
    .resume     = mtk_wifi_ctl_resume,

    .driver = {
#if defined(CONFIG_OF)
        .of_match_table = mtk_wifi_ctl_of_device_ids,
#endif
        .name   = MTK_WIFI_CTL_NAME,
        .owner  = THIS_MODULE,
    }
};

static int __init mtk_wifi_ctl_init(void)
{
    platform_driver_register(&mtk_wifi_ctl_driver);

    return 0;
}

static void __exit mtk_wifi_ctl_exit(void)
{
    platform_driver_unregister(&mtk_wifi_ctl_driver);
}

MODULE_AUTHOR("MediaTek");
module_init(mtk_wifi_ctl_init);
module_exit(mtk_wifi_ctl_exit);
