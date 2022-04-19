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
/// file    mdrv_pwm.c
/// @brief  TEMP Driver Interface for Export
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/pwm.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
//drver header files
#include "mst_devid.h"
//#include "mdrv_mstypes.h"
#include "MsTypes.h"
#include "mdrv_pwm.h"
#include "mhal_pwm.h"
#include "mreg_pwm.h"

//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
typedef enum
{
  e_pwm_attr_oen = 0,
  e_pwm_attr_period,
  e_pwm_attr_duty,
  e_pwm_attr_div,
  e_pwm_attr_shift,
  e_pwm_attr_pol,
  e_pwm_attr_dben,
  e_pwm_attr_vdben,
  e_pwm_attr_vrst,
  e_pwm_attr_rstmux,
  e_pwm_attr_rstcnt,
  e_pwm_attr_max,
}pwm_attr_e;


//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------

struct mtk_pwm_dat
{
    char pwm_ch;
    unsigned int period;
    unsigned int shift;
    unsigned int div;
    bool pol;
    bool rst_mux;
    bool rst_vsync;
    unsigned int rstcnt;
};

struct mtk_pwm
{
    struct pwm_chip chip;
    pwm_attr_e e_pwm;
    const struct mtk_pwm_dat *data;
};

static const struct mtk_pwm_dat mtk7322_pwm_data =
{
    .pwm_ch = 0,
    .period = 0xFFFF,
    .shift = 0,
    .div = 0,
    .pol = 0,
    .rst_mux = 0,
    .rst_vsync = 0,
    .rstcnt = 0,
};
//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
#define PWM_HW_SUPPORT_CH   6

static inline struct mtk_pwm *to_mtk_pwm(struct pwm_chip *chip)
{
    return container_of(chip, struct mtk_pwm, chip);
}

static void mtk_pwm_update_ctrl(struct mtk_pwm *mdp, unsigned int data)
{
    switch(mdp->e_pwm)
    {
        case e_pwm_attr_oen:
            MDrv_PWM_Oen((mdp->data->pwm_ch),data);
        break;
        case e_pwm_attr_period:
            MDrv_PWM_Period((mdp->data->pwm_ch),data);
        break;
        case e_pwm_attr_duty:
            MDrv_PWM_DutyCycle((mdp->data->pwm_ch),data);
        break;
        case e_pwm_attr_div:
            MDrv_PWM_Div((mdp->data->pwm_ch),data);
        break;
        case e_pwm_attr_shift:
            MDrv_PWM_Shift((mdp->data->pwm_ch),data);
        break;
        case e_pwm_attr_pol:
            MDrv_PWM_Polarity((mdp->data->pwm_ch),data);
        break;
        case e_pwm_attr_dben:
            MDrv_PWM_Dben((mdp->data->pwm_ch),data);
        break;
        case e_pwm_attr_vdben:
            MDrv_PWM_Vdben((mdp->data->pwm_ch),data);
        break;
        case e_pwm_attr_vrst:
            MDrv_PWM_ResetEn((mdp->data->pwm_ch),data);
        break;
        case e_pwm_attr_rstmux:
            MDrv_PWM_RstMux((mdp->data->pwm_ch),data);
        break;
        case e_pwm_attr_rstcnt:
            MDrv_PWM_RstCnt((mdp->data->pwm_ch),data);
        break;
    }
}

static int mtk_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
                   int duty_t, int period_t)
{
    struct mtk_pwm *mdp = to_mtk_pwm(chip);
    int err;
    int duty_set,period_set;

    if((duty_t==0)||(duty_t>period_t))
    {
        mdp->e_pwm = e_pwm_attr_shift;
        /*solve glitch when duty is 0*/
        mtk_pwm_update_ctrl(mdp,1);
        return 0 ;
    }

    /*24Mhz XTAL timing is nsec*/
    period_set = (24*period_t)/1000;
    duty_set = (24*duty_t)/1000;

    mdp->e_pwm = e_pwm_attr_period;
    mtk_pwm_update_ctrl(mdp,period_set);
    mdp->e_pwm = e_pwm_attr_duty;
    mtk_pwm_update_ctrl(mdp,duty_set);

    if (mdp->data->shift)
    {
        mdp->e_pwm = e_pwm_attr_shift;
        mtk_pwm_update_ctrl(mdp,mdp->data->shift);
    }

    if (mdp->data->div)
    {
        mdp->e_pwm = e_pwm_attr_div;
        mtk_pwm_update_ctrl(mdp,mdp->data->div);
    }

    if (mdp->data->pol)
    {
        mdp->e_pwm = e_pwm_attr_pol;
        mtk_pwm_update_ctrl(mdp,1);
    }

    if (mdp->data->rst_vsync)
    {
        mdp->e_pwm = e_pwm_attr_vrst;
        mtk_pwm_update_ctrl(mdp,1);
        if(mdp->data->rstcnt)
        {
           mdp->e_pwm = e_pwm_attr_rstcnt;
           mtk_pwm_update_ctrl(mdp,mdp->data->rstcnt);
        }
        if(mdp->data->rst_mux)
        {
           mdp->e_pwm = e_pwm_attr_rstmux;
           mtk_pwm_update_ctrl(mdp,mdp->data->rst_mux);
        }
    }

    return 0;
}

static int mtk_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
    struct mtk_pwm *mdp = to_mtk_pwm(chip);
    int err;

    mdp->e_pwm = e_pwm_attr_oen ;
    mtk_pwm_update_ctrl(mdp,0);
    return 0;
}

static void mtk_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
    struct mtk_pwm *mdp = to_mtk_pwm(chip);
    int err;

    mdp->e_pwm = e_pwm_attr_oen ;
    mtk_pwm_update_ctrl(mdp,1);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
static void mtk_pwm_get_state(struct pwm_chip *chip,struct pwm_device *pwm,struct pwm_state *state)
{
    struct mtk_pwm *mdp = to_mtk_pwm(chip);
    int    pwm_attr_get;

    MDrv_PWM_GetProperty(E_PWM_GetPeriod,mdp->data->pwm_ch,&pwm_attr_get);
    state->period = pwm_attr_get;
    MDrv_PWM_GetProperty(E_PWM_GetDutyCycle,mdp->data->pwm_ch,&pwm_attr_get);
    state->duty_cycle = pwm_attr_get;
    MDrv_PWM_GetProperty(E_PWM_GetPolarity,mdp->data->pwm_ch,&pwm_attr_get);
    state->polarity = pwm_attr_get;
    MDrv_PWM_GetProperty(E_PWM_GetOen,mdp->data->pwm_ch,&pwm_attr_get);
    state->enabled = pwm_attr_get;
}
#endif

static const struct pwm_ops mtk_pwm_ops = {
    .config = mtk_pwm_config,
    .enable = mtk_pwm_enable,
    .disable = mtk_pwm_disable,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    .get_state = mtk_pwm_get_state,
#endif
    .owner = THIS_MODULE,
};

static int __init mtk_pwm_probe(struct platform_device *pdev)
{
    struct mtk_pwm *mdp;
    int ret;

    mdp = devm_kzalloc(&pdev->dev, sizeof(*mdp), GFP_KERNEL);

    if (!mdp)
        return -ENOMEM;

    //mdp->data = of_device_get_match_data(&pdev->dev);
    mdp->data = &mtk7322_pwm_data ;
    mdp->chip.dev = &pdev->dev;
    mdp->chip.ops = &mtk_pwm_ops;
    mdp->chip.npwm = PWM_HW_SUPPORT_CH;/*It will return error without this setting*/

    MDrv_PWM_Init(E_PWM_DBGLV_ERR_ONLY);
    mdp->e_pwm = e_pwm_attr_oen ;

    if ((mdp->data->pwm_ch)!=E_PWM_MAX)
        mtk_pwm_update_ctrl(mdp,0);
    else
        return -1;

    mdp->e_pwm = e_pwm_attr_period ;
    mtk_pwm_update_ctrl(mdp,mdp->data->period);

    /*
    mtk_pwm_config(&mdp->chip,pwm_t,400000,800000);
    */

    ret = pwmchip_add(&mdp->chip);
    if (ret < 0) {
        dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
    }

    platform_set_drvdata(pdev, mdp);

    return ret;
    }

static int mtk_pwm_remove(struct platform_device *pdev)
{
    if( !(pdev->name) || strcmp(pdev->name,"mtk,pwm")
        || pdev->id!=0)
    {
        return -1;
    }
    pdev->dev.platform_data = NULL;
    return 0;
}



#if defined (CONFIG_OF)
static struct of_device_id mtkpwm_of_device_ids[] = {
        {.compatible = "mtk,pwm", .data = &mtk7322_pwm_data},
        {},
};
#endif

static struct platform_driver Mtk_pwm_driver = {
    .probe      = mtk_pwm_probe,
    .remove     = mtk_pwm_remove,

    .driver = {
#if defined(CONFIG_OF)
    .of_match_table = mtkpwm_of_device_ids,
#endif
    .name   = "mtk,pwm",
    .owner  = THIS_MODULE,
}
};


static int __init mtk_pwm_init_module(void)
{
    int retval=0;
    printk("[kpwm] %s\n",__FUNCTION__);
    retval = platform_driver_register(&Mtk_pwm_driver);
    return retval;
}

static void __exit mtk_pwm_exit_module(void)
{
    platform_driver_unregister(&Mtk_pwm_driver);
}

module_init(mtk_pwm_init_module);
module_exit(mtk_pwm_exit_module);


MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("pwm driver");
MODULE_LICENSE("GPL");

