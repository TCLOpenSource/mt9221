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

/* From linux. */
#include <linux/version.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <trace/events/power.h>

/* From mstar. */
#include "mdrv_r2.h"

//-------------------------------------------------------------------------------------------------
// DEFINE
//-------------------------------------------------------------------------------------------------
#define MDRV_R2_FRC_NAME    "FRC-R2"

//-------------------------------------------------------------------------------------------------
// PM_OPS
//-------------------------------------------------------------------------------------------------
#if defined(CONFIG_MSTAR_FRC_R2)
static int _MDrv_FRC_R2_Suspend_Prepare(struct device *dev)
{
    return MDrv_FRC_R2_Suspend(E_R2_STATE_SUSPEND_PRE);
}

static int _MDrv_FRC_R2_Suspend_Noirq(struct device *dev)
{
    return MDrv_FRC_R2_Suspend(E_R2_STATE_SUSPEND_NOIRQ);
}

static int _MDrv_FRC_R2_Resume_Noirq(struct device *dev)
{
    return MDrv_FRC_R2_Resume(E_R2_STATE_RESUME_NOIRQ);
}

static const struct dev_pm_ops _MDrv_FRC_R2_PM_OPS =
{
    .prepare        = _MDrv_FRC_R2_Suspend_Prepare,
    .suspend_noirq  = _MDrv_FRC_R2_Suspend_Noirq,
    .resume_noirq   = _MDrv_FRC_R2_Resume_Noirq,
};

//-------------------------------------------------------------------------------------------------
// DRIVER
//-------------------------------------------------------------------------------------------------
static int _MDrv_FRC_R2_Probe(struct platform_device *pdev)
{
    /* Maybe register debug proc here. */
    return 0;
}

static int _MDrv_FRC_R2_Remove(struct platform_device *pdev)
{
    /* Sure things, remove the debug proc here. */
    return 0;
}

#ifdef CONFIG_OF
static struct of_device_id _MDrv_FRC_R2_OF_DEV_IDS[] = {
    {.compatible = MDRV_R2_FRC_NAME},
    {},
};
#endif

static struct platform_driver _MDrv_FRC_R2_Driver = {
    .probe          = _MDrv_FRC_R2_Probe,
    .remove         = _MDrv_FRC_R2_Remove,
    .driver         = {
#ifdef CONFIG_OF
    .of_match_table = _MDrv_FRC_R2_OF_DEV_IDS,
#endif
        .pm             = &_MDrv_FRC_R2_PM_OPS,
        .name           = MDRV_R2_FRC_NAME,
        .owner          = THIS_MODULE,
    },
};

//-------------------------------------------------------------------------------------------------
// MODULE
//-------------------------------------------------------------------------------------------------
static int __init _MDrv_FRC_R2_Init(void)
{
    return platform_driver_register(&_MDrv_FRC_R2_Driver);
}

static void __exit _MDrv_FRC_R2_Exit(void)
{
    platform_driver_unregister(&_MDrv_FRC_R2_Driver);
}

module_init(_MDrv_FRC_R2_Init);
module_exit(_MDrv_FRC_R2_Exit);
MODULE_DESCRIPTION("Mstar FRC_R2 driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("<Weiting.Tsai@mediatek.com>");
#endif
