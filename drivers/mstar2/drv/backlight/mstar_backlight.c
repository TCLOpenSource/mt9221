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

/*
base on linux/drivers/video/backlight/pwm_bl.c
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include "mhal_gpio_reg.h"
#ifdef CONFIG_MSTAR_PWM
#include "mdrv_pwm.h"
#endif /* CONFIG_MSTAR_PWM */
#include "mdrv_pm.h"

struct pwm_bl_data {
    struct pwm_device   *pwm;
    struct device       *dev;
    unsigned int        period;
    unsigned int        lth_brightness;
    unsigned int        *levels;
    unsigned int        enable_gpio;
    unsigned int        bl_gpio_invert;

    int         (*notify)(struct device *,
                      int brightness);
    void            (*notify_after)(struct device *,
                    int brightness);
    int         (*check_fb)(struct device *, struct fb_info *);
    void            (*exit)(struct device *);

    unsigned int        led_gpio;
    unsigned int        led_gpio_invert;
    bool                enable_led_gpio_ctrl;

    unsigned int        vcc_gpio;
    unsigned int        vcc_gpio_invert;
    bool                enable_vcc_gpio_ctrl;

    bool is_led_pwm_ctrl;
    unsigned int led_pwm_ctrl_channel;
    unsigned short led_pwm_ctrl_resume_period;
    unsigned short led_pwm_ctrl_resume_duty;
    unsigned short led_pwm_ctrl_resume_div;
    unsigned char led_pwm_ctrl_resume_polarity;
    unsigned short led_pwm_ctrl_suspend_period;
    unsigned short led_pwm_ctrl_suspend_duty;
    unsigned short led_pwm_ctrl_suspend_div;
    unsigned char led_pwm_ctrl_suspend_polarity;
};

//static struct platform_device mstar_backlight = {
//        .name = "backlight",
//};

//MODULE_DEVICE_TABLE(platform, mstar_backlight);

static int pwm_backlight_update_status(struct backlight_device *bl)
{

    struct pwm_bl_data *pb = bl_get_data(bl);
    int brightness = bl->props.brightness;
    //int max = bl->props.max_brightness;

    printk(KERN_INFO "%s\n", __func__);
    if (bl->props.power != FB_BLANK_UNBLANK ||
        bl->props.fb_blank != FB_BLANK_UNBLANK ||
        bl->props.state & BL_CORE_FBBLANK)
        brightness = 0;

    if (pb->notify)
        brightness = pb->notify(pb->dev, brightness);

    if (brightness == 0) {
#if 0
        pwm_config(pb->pwm, 0, pb->period);
        pwm_disable(pb->pwm);
#else
        printk(KERN_INFO "turn OFF backlight: %d, inverse: %d\n", pb->enable_gpio, pb->bl_gpio_invert);

        if (pb->bl_gpio_invert != 0)
            gpio_set_value(pb->enable_gpio, 1);
        else
            gpio_set_value(pb->enable_gpio, 0);

        if(pb->enable_vcc_gpio_ctrl) {
            printk(KERN_INFO "turn OFF vcc: %d, inverse: %d\n", pb->vcc_gpio, pb->vcc_gpio_invert);
            if (pb->vcc_gpio_invert)
                gpio_set_value(pb->vcc_gpio, 1);
            else
                gpio_set_value(pb->vcc_gpio, 0);
        }

        if (pb->enable_led_gpio_ctrl) {
            if (pb->led_gpio_invert)
                gpio_set_value(pb->led_gpio, 1);
            else
                gpio_set_value(pb->led_gpio, 0);
        }
#endif
    } else {
        if(pb->enable_vcc_gpio_ctrl) {
            printk(KERN_INFO "turn ON vcc: %d, inverse: %d\n", pb->vcc_gpio, pb->vcc_gpio_invert);
            if (pb->vcc_gpio_invert)
                gpio_set_value(pb->vcc_gpio, 0);
            else
                gpio_set_value(pb->vcc_gpio, 1);
        }
        printk(KERN_INFO "turn ON backlight: %d, inverse: %d\n", pb->enable_gpio, pb->bl_gpio_invert);

        if (pb->bl_gpio_invert != 0)
            gpio_set_value(pb->enable_gpio, 0);
        else
            gpio_set_value(pb->enable_gpio, 1);
#if 0
        int duty_cycle;

        if (pb->levels) {
            duty_cycle = pb->levels[brightness];
            max = pb->levels[max];
        } else {
            duty_cycle = brightness;
        }

        duty_cycle = pb->lth_brightness +
             (duty_cycle * (pb->period - pb->lth_brightness) / max);
        pwm_config(pb->pwm, duty_cycle, pb->period);
        pwm_enable(pb->pwm);
#endif
    }

    if (pb->notify_after)
        pb->notify_after(pb->dev, brightness);

    printk(KERN_INFO "backlight adjust done\n");
    return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
    return bl->props.brightness;
}

static const struct backlight_ops pwm_backlight_ops = {
    .update_status  = pwm_backlight_update_status,
    .get_brightness = pwm_backlight_get_brightness,
//    .check_fb   = pwm_backlight_check_fb,
};

#ifdef CONFIG_OF
static int pwm_backlight_parse_dt(struct device *dev,
                  struct platform_pwm_backlight_data *data)
{
    struct device_node *node = dev->of_node;
    struct property *prop;
    int length;
    u32 value;
    int ret;

    if (!node) {
        printk(KERN_ERR "No device tree entry for backlight device\n");
        return -ENODEV;
    }

    memset(data, 0, sizeof(*data));

    /* determine the number of brightness levels */
    prop = of_find_property(node, "brightness-levels", &length);
    if (!prop) {
        printk(KERN_ERR "failed to find brightness-levels property\n");
        return -EINVAL;
    }

//    printk("backlight length = %d\n", length);
    data->max_brightness = length / sizeof(u32);

    // get gpio pin that controls backlight
    ret = of_property_read_u32(node, "bl_ctrl", &data->enable_gpio);
    if (ret != 0) {
        printk(KERN_ERR "failed to find bl_ctrl property\n");
        return -EINVAL;
    }
    printk(KERN_INFO "backlight control using gpio pin #%d\n", data->enable_gpio);

    /* read brightness levels from DT property */
    if (data->max_brightness > 0) {
        size_t size = sizeof(*data->levels) * data->max_brightness;

        data->levels = devm_kzalloc(dev, size, GFP_KERNEL);
        if (!data->levels)
            return -ENOMEM;

        ret = of_property_read_u32_array(node, "brightness-levels",
                         data->levels,
                         data->max_brightness);
        if (ret < 0)
            return ret;

        ret = of_property_read_u32(node, "default-brightness-level",
                       &value);
        if (ret < 0)
            return ret;

        data->dft_brightness = value;
        data->max_brightness--;
    }

    /*
     * TODO: Most users of this driver use a number of GPIOs to control
     *       backlight power. Support for specifying these needs to be
     *       added.
     */

    return 0;
}

static struct of_device_id pwm_backlight_of_match[] = {
    { .compatible = "backlight" },
};

MODULE_DEVICE_TABLE(of, pwm_backlight_of_match);
#else
static int pwm_backlight_parse_dt(struct device *dev,
                  struct platform_pwm_backlight_data *data)
{
    return -ENODEV;
}
#endif

static int pwm_backlight_probe(struct platform_device *pdev)
{
    struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
    struct platform_pwm_backlight_data defdata;
    struct backlight_properties props;
    struct backlight_device *bl;
    struct pwm_bl_data *pb;
    struct device_node *node = pdev->dev.of_node;
    unsigned int max;
    int ret;
    bool b_update_status = true;
    u32 of_val;

    printk(KERN_INFO "%s\n", __func__);
    if (!data) {
        ret = pwm_backlight_parse_dt(&pdev->dev, &defdata);
        if (ret < 0) {
            dev_err(&pdev->dev, "failed to find platform data..\n");
            return ret;
        }

        data = &defdata;
    }

    if (data->init) {
        ret = data->init(&pdev->dev);
        if (ret < 0)
            return ret;
    }

    pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
    if (!pb) {
        dev_err(&pdev->dev, "no memory for state\n");
        ret = -ENOMEM;
        goto err_alloc;
    }

    //parse dts besides pwm_backlight setting
    if (node) {
        ret = (of_property_read_u32(node, "vcc_gpio", &pb->vcc_gpio) | of_property_read_u32(node, "vcc_gpio_invert", &pb->vcc_gpio_invert));
        if (ret != 0) {
            printk(KERN_ERR "failed to find vcc_gpio property\n");
            pb->enable_vcc_gpio_ctrl = false;
        } else {
            if (pb->vcc_gpio == 999) {
                printk(KERN_ERR "invalid vcc gpio num: %d\n", pb->vcc_gpio);
                pb->enable_vcc_gpio_ctrl = false;
            } else {
                printk(KERN_INFO "vcc control using gpio pin #%d, invert: %d\n", pb->vcc_gpio, pb->vcc_gpio_invert);
                pb->enable_vcc_gpio_ctrl = true;
            }
        }

        ret = (of_property_read_u32(node, "led_gpio", &pb->led_gpio) | of_property_read_u32(node, "led_gpio_invert", &pb->led_gpio_invert));
        if (ret != 0) {
            printk(KERN_ERR "failed to find led_ctrl property\n");
            pb->enable_led_gpio_ctrl = false;
        } else {
            if (pb->led_gpio == 999) {
                printk(KERN_ERR "invalid led gpio num: %d\n", pb->led_gpio);
                pb->enable_led_gpio_ctrl = false;
            } else {
                printk(KERN_INFO "led control using gpio pin #%d, invert: %d\n", pb->led_gpio, pb->led_gpio_invert);
                pb->enable_led_gpio_ctrl = true;
            }
        }

        ret = of_property_read_u32(node, "bl_ctrl_inverse", &pb->bl_gpio_invert);
        if (ret != 0)
            pb->bl_gpio_invert = 0;

        printk(KERN_INFO "backlight control using gpio pin as invert %d\n", pb->bl_gpio_invert);

        /* parsing led pwm control parameters */
        if (of_property_read_bool(node, "led-pwm-control-support") == true) {
            printk(KERN_INFO "backlight: support led pwm control\n");
            pb->is_led_pwm_ctrl = true;

            of_val = 0;
            if (of_property_read_u32(node, "led-pwm-control-channel", &of_val)) {
                pb->led_pwm_ctrl_channel = 0;
                printk(KERN_WARNING "backlight: get channel failed\n");
            } else {
                pb->led_pwm_ctrl_channel = of_val;
            }

            of_val = 0;
            if (of_property_read_u32(node, "led-pwm-control-resume-period", &of_val)) {
                pb->led_pwm_ctrl_resume_period = 0;
                printk(KERN_WARNING "backlight: get resume-period failed\n");
            } else {
                pb->led_pwm_ctrl_resume_period = (unsigned short)of_val;
            }

            of_val = 0;
            if (of_property_read_u32(node, "led-pwm-control-resume-duty", &of_val)) {
                pb->led_pwm_ctrl_resume_duty = 0;
                printk(KERN_WARNING "backlight: get resume-duty failed\n");
            } else {
               pb->led_pwm_ctrl_resume_duty = (unsigned short)of_val;
            }

            of_val = 0;
            if (of_property_read_u32(node, "led-pwm-control-resume-div", &of_val)) {
                pb->led_pwm_ctrl_resume_div = 0;
                printk(KERN_WARNING "backlight: get resume-div failed\n");
            } else {
                pb->led_pwm_ctrl_resume_div = (unsigned short)of_val;
            }

            of_val = 0;
            if (of_property_read_u32(node, "led-pwm-control-resume-polarity", &of_val)) {
                pb->led_pwm_ctrl_resume_polarity = 0;
                printk(KERN_WARNING "backlight: get resume-polarity failed\n");
            } else {
                pb->led_pwm_ctrl_resume_polarity = (unsigned char)of_val;
            }

            of_val = 0;
            if (of_property_read_u32(node, "led-pwm-control-suspend-period", &of_val)) {
                pb->led_pwm_ctrl_suspend_period = 0;
                printk(KERN_WARNING "backlight: get suspend-period failed\n");
            } else {
                pb->led_pwm_ctrl_suspend_period = (unsigned short)of_val;
            }

            of_val = 0;
            if (of_property_read_u32(node, "led-pwm-control-suspend-duty", &of_val)) {
                pb->led_pwm_ctrl_suspend_duty = 0;
                printk(KERN_WARNING "backlight: get suspend-duty failed\n");
            } else {
                pb->led_pwm_ctrl_suspend_duty = (unsigned short)of_val;
            }

            of_val = 0;
            if (of_property_read_u32(node, "led-pwm-control-suspend-div", &of_val)) {
                pb->led_pwm_ctrl_suspend_div = 0;
                printk(KERN_WARNING "backlight: get suspend-div failed\n");
            } else {
                pb->led_pwm_ctrl_suspend_div = (unsigned short)of_val;
            }

            of_val = 0;
            if (of_property_read_u32(node, "led-pwm-control-suspend-polarity", &of_val)) {
                pb->led_pwm_ctrl_suspend_polarity = 0;
                printk(KERN_WARNING "backlight: get suspend-polarity failed\n");
            } else {
                pb->led_pwm_ctrl_suspend_polarity = (unsigned char)of_val;
            }
            printk(KERN_INFO "backlight: ch(0x%x), resume period(0x%x), duty(0x%x), div(0x%x), polarity(0x%x)\n",
                   pb->led_pwm_ctrl_channel,
                   pb->led_pwm_ctrl_resume_period,
                   pb->led_pwm_ctrl_resume_duty,
                   pb->led_pwm_ctrl_resume_div,
                   pb->led_pwm_ctrl_resume_polarity);
            printk(KERN_INFO "backlight: suspend period(0x%x), duty(0x%x), div(0x%x), polarity(0x%x)\n",
                   pb->led_pwm_ctrl_suspend_period,
                   pb->led_pwm_ctrl_suspend_duty,
                   pb->led_pwm_ctrl_suspend_div,
                   pb->led_pwm_ctrl_suspend_polarity);
        } else {
            printk(KERN_INFO "backlight: not support led pwm control\n");
            pb->is_led_pwm_ctrl = false;
        }
    } else {
        pb->enable_led_gpio_ctrl = false;
        pb->enable_vcc_gpio_ctrl = false;
        pb->is_led_pwm_ctrl = false;
    }

    if (data->levels) {
        max = data->levels[data->max_brightness];
        pb->levels = data->levels;
    } else
        max = data->max_brightness;

    if (data->enable_gpio) {
        pb->enable_gpio = data->enable_gpio;
    }
    else {
        // there is no way to determine the correctness of enable_gpio, won't do anything here
        dev_err(&pdev->dev, "invalid bl control gpio = %d\n", data->enable_gpio);
        return -EINVAL;
    }
#if 0 // gpio_request will fail since M* gpio driver already requested for sysfs
    ret = gpio_request(pb->enable_gpio, "backlight_ctrl");
    if (ret) {
        dev_err(&pdev->dev, "gpio request filaed, %d\n", PAD_GPIO0_PM);
        goto err_alloc;
    }
#endif

    pb->notify = data->notify;
    pb->notify_after = data->notify_after;
    pb->check_fb = data->check_fb;
    pb->exit = data->exit;
    pb->dev = &pdev->dev;

#if 0 // probably for future use?
    pb->pwm = devm_pwm_get(&pdev->dev, NULL);
    if (IS_ERR(pb->pwm)) {
        dev_err(&pdev->dev, "unable to request PWM, trying legacy API\n");

        pb->pwm = pwm_request(data->pwm_id, "pwm-backlight");
        if (IS_ERR(pb->pwm)) {
            dev_err(&pdev->dev, "unable to request legacy PWM\n");
            ret = PTR_ERR(pb->pwm);
            goto err_alloc;
        }
    }
    dev_dbg(&pdev->dev, "got pwm for backlight\n");

    /*
     * The DT case will set the pwm_period_ns field to 0 and store the
     * period, parsed from the DT, in the PWM device. For the non-DT case,
     * set the period from platform data.
     */
    if (data->pwm_period_ns > 0)
        pwm_set_period(pb->pwm, data->pwm_period_ns);

    pb->period = pwm_get_period(pb->pwm);
    pb->lth_brightness = data->lth_brightness * (pb->period / max);

#endif

    memset(&props, 0, sizeof(struct backlight_properties));
    props.type = BACKLIGHT_RAW;
    props.max_brightness = data->max_brightness;
    bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
                       &pwm_backlight_ops, &props);
    if (IS_ERR(bl)) {
        dev_err(&pdev->dev, "failed to register backlight\n");
        ret = PTR_ERR(bl);
        goto err_alloc;
    }

    if (data->dft_brightness > data->max_brightness) {
        dev_warn(&pdev->dev,
             "invalid default brightness level: %u, using %u\n",
             data->dft_brightness, data->max_brightness);
        data->dft_brightness = data->max_brightness;
        //the default brightness level is invalid, don't update backlight status.
        b_update_status = false;
    }

    bl->props.brightness = data->dft_brightness;
    if(b_update_status)
        backlight_update_status(bl);

    platform_set_drvdata(pdev, bl);
    return 0;

err_alloc:
    if (data->exit)
        data->exit(&pdev->dev);
    return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
    struct backlight_device *bl = platform_get_drvdata(pdev);
    struct pwm_bl_data *pb = bl_get_data(bl);

    backlight_device_unregister(bl);
    pwm_config(pb->pwm, 0, pb->period);
    pwm_disable(pb->pwm);
    if (pb->exit)
        pb->exit(&pdev->dev);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_backlight_suspend(struct device *dev)
{
    struct backlight_device *bl = dev_get_drvdata(dev);
    struct pwm_bl_data *pb = bl_get_data(bl);

    if (pb->is_led_pwm_ctrl == true) {
#ifdef CONFIG_MSTAR_PWM
        printk(KERN_INFO "backlight: set led pwm in suspend\n");

        MDrv_PWM_Init(0);

        //standby enter/stop
        MDrv_PM_PWM_Enable_Ex(pb->led_pwm_ctrl_channel, 1);
        MDrv_PM_PWM_Dben_Ex(pb->led_pwm_ctrl_channel, 0);
        MDrv_PM_PWM_Div_Ex(pb->led_pwm_ctrl_channel, pb->led_pwm_ctrl_suspend_div);
        MDrv_PM_PWM_Period_Ex(pb->led_pwm_ctrl_channel, pb->led_pwm_ctrl_suspend_period);
        MDrv_PM_PWM_DutyCycle_Ex(pb->led_pwm_ctrl_channel, pb->led_pwm_ctrl_suspend_duty);
        MDrv_PM_PWM_Polarity_Ex(pb->led_pwm_ctrl_channel, pb->led_pwm_ctrl_suspend_polarity);
        MDrv_PM_PWM_Enable_Ex(pb->led_pwm_ctrl_channel, 1);
#endif /* CONFIG_MSTAR_PWM */
    }

#if 0 /* no used before*/
    if (pb->notify)
        pb->notify(pb->dev, 0);
    pwm_config(pb->pwm, 0, pb->period);
    pwm_disable(pb->pwm);
    if (pb->notify_after)
        pb->notify_after(pb->dev, 0);
#endif

    return 0;
}

static int pwm_backlight_resume(struct device *dev)
{
    struct backlight_device *bl = dev_get_drvdata(dev);
    struct pwm_bl_data *pb = bl_get_data(bl);

    if (pb->is_led_pwm_ctrl == true) {
        if ((MDrv_PM_GetWakeupSource() != PM_WAKEUPSRC_RTC) &&
            (MDrv_PM_GetWakeupSource() != PM_WAKEUPSRC_WOC) &&
            (MDrv_PM_GetWakeupSource() != PM_WAKEUPSRC_GPIO_WOWLAN)) {
#ifdef CONFIG_MSTAR_PWM
            printk(KERN_INFO "backlight: set led pwm in resume\n");
            MDrv_PWM_Init(0);
            //standby enter/stop
            MDrv_PM_PWM_Enable_Ex(pb->led_pwm_ctrl_channel, 1);
            MDrv_PM_PWM_Dben_Ex(pb->led_pwm_ctrl_channel, 0);
            MDrv_PM_PWM_Div_Ex(pb->led_pwm_ctrl_channel, pb->led_pwm_ctrl_resume_div);
            MDrv_PM_PWM_Period_Ex(pb->led_pwm_ctrl_channel, pb->led_pwm_ctrl_resume_period);
            MDrv_PM_PWM_DutyCycle_Ex(pb->led_pwm_ctrl_channel, pb->led_pwm_ctrl_resume_duty);
            MDrv_PM_PWM_Polarity_Ex(pb->led_pwm_ctrl_channel, pb->led_pwm_ctrl_resume_polarity);
            MDrv_PM_PWM_Enable_Ex(pb->led_pwm_ctrl_channel, 1);
#endif /* CONFIG_MSTAR_PWM */
        }
    }

#if 0 /* no used before*/
    backlight_update_status(bl);
#endif
    return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_backlight_pm_ops, pwm_backlight_suspend,
             pwm_backlight_resume);

static struct platform_driver pwm_backlight_driver = {
    .driver     = {
        .name       = "backlight",
        .owner      = THIS_MODULE,
        .pm     = &pwm_backlight_pm_ops,
        .of_match_table = of_match_ptr(pwm_backlight_of_match),
    },
    .probe      = pwm_backlight_probe,
    .remove     = pwm_backlight_remove,
};

#if 0
static int __init mstar_backlight_init(void)
{
    if (platform_device_register(&mstar_backlight)) {
        printk("backlight register platform device fail = %d\n");
        return  -1;
    }
    if (platform_driver_register(&pwm_backlight_driver)) {
        printk("backlight register platform device fail = %d\n");
        return -1;
    }
    return 0;
}
#endif

//module_init(mstar_backlight_init);
module_platform_driver(pwm_backlight_driver);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");
