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
//  Include Files
//-------------------------------------------------------------------------------------------------
/* From linux. */
#include <asm/io.h>
#include <asm/uaccess.h>
#include <crypto/internal/hash.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>

/* From mstar. */
#include "mdrv_pm.h"
#include "mdrv_mpm.h"
#include "mdrv_types.h"

//--------------------------------------------------------------------------------------------------
//  Extern
//--------------------------------------------------------------------------------------------------
extern int is_mstar_str(void);

//--------------------------------------------------------------------------------------------------
//  Debug
//--------------------------------------------------------------------------------------------------
#define MDRV_MPM_INFO(fmt, args...)     printk(KERN_INFO    "[%s][%d] " fmt, __func__, __LINE__, ## args)
#define MDRV_MPM_WARN(fmt, args...)     printk(KERN_WARNING "[%s][%d] " fmt, __func__, __LINE__, ## args)
#define MDRV_MPM_ERROR(fmt, args...)    printk(KERN_ERR     "[%s][%d] " fmt, __func__, __LINE__, ## args)
#define MDRV_MPM_DEBUG(fmt, args...)    printk(KERN_DEBUG   "[%s][%d] " fmt, __func__, __LINE__, ## args)

//--------------------------------------------------------------------------------------------------
//  Forward declaration
//--------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//  Local variable
//--------------------------------------------------------------------------------------------------
static int _MDrv_MPM_Max_Cnt = 0;
static int _MDrv_MPM_STR_Cnt = 0;
static int _MDrv_MPM_Ignore_Cnt = 0;

//-------------------------------------------------------------------------------------------------
//  Golbal variable
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local function
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Golbal function
//-------------------------------------------------------------------------------------------------
int MDrv_MPM_Get_Cnt(int key)
{
    int val = -1;

    switch (key)
    {
        case MDRV_MPM_KEY_MAX_CNT:
            val = _MDrv_MPM_Max_Cnt;
            break;

        case MDRV_MPM_KEY_STR_CNT:
            val = _MDrv_MPM_STR_Cnt;
            break;

        case MDRV_MPM_KEY_IGNORE_CNT:
            val = _MDrv_MPM_Ignore_Cnt;
            break;
    }

    return val;
}

void MDrv_MPM_Set_Cnt(int key, int cnt)
{
    switch (key)
    {
        case MDRV_MPM_KEY_MAX_CNT:
            _MDrv_MPM_Max_Cnt = cnt;
            break;

        case MDRV_MPM_KEY_STR_CNT:
        case MDRV_MPM_KEY_IGNORE_CNT:
            if ((MDrv_PM_GetWakeupSource() != PM_WAKEUPSRC_RTC) &&
                (MDrv_PM_GetWakeupSource() != PM_WAKEUPSRC_WOC) &&
                (MDrv_PM_GetWakeupSource() != PM_WAKEUPSRC_VOICE) &&
                (MDrv_PM_GetWakeupSource() != PM_WAKEUPSRC_GPIO_WOWLAN))
                _MDrv_MPM_STR_Cnt++;
            else
                _MDrv_MPM_Ignore_Cnt++;

            break;
    }

    MDRV_MPM_DEBUG("Max_Cnt=%d, STR_Cnt=%d, Ignore_Cnt=%d\n",
                 _MDrv_MPM_Max_Cnt,
                 _MDrv_MPM_STR_Cnt,
                 _MDrv_MPM_Ignore_Cnt);
}

bool MDrv_MPM_Check_DC(void)
{
    BOOL            bRet = FALSE;
    PM_WakeCfg_t    stCfgDef = {0};

    bRet = (_MDrv_MPM_Max_Cnt > 0 && _MDrv_MPM_Max_Cnt <= _MDrv_MPM_STR_Cnt);

    if (!is_mstar_str())
    {
        MDrv_PM_Read_Key(PM_KEY_DEFAULT, (void *)&stCfgDef);
        bRet |= (stCfgDef.u8PmStrMode == PM_DC_MODE);
    }

    return bRet;
}

//-------------------------------------------------------------------------------------------------
//  Early
//-------------------------------------------------------------------------------------------------
