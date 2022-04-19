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



#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <asm/delay.h>

#include "mhal_rtc.h"
#include "mdrv_rtc.h"


void MDrv_RTC_Init(PM_RtcParam *pPmRtcParam)
{
   printk(KERN_EMERG "==RTC== %s, %d\n" , __FUNCTION__, __LINE__);
   MHAL_RTC_Init(pPmRtcParam->u8PmRtcIndex, pPmRtcParam->u32RtcCtrlWord);
}

void MDrv_RTC_SetCount(PM_RtcParam *pPmRtcParam)
{
   //printk(KERN_EMERG "==RTC== %s, %d\n" , __FUNCTION__, __LINE__);
   MHAL_RTC_SetCounter(pPmRtcParam->u8PmRtcIndex, pPmRtcParam->u32RtcSetCounter);
}

U32 MDrv_RTC_GetCount(PM_RtcParam *pPmRtcParam)
{
    //printk(KERN_EMERG "==RTC== %s, %d\n" , __FUNCTION__, __LINE__);
    return MHAL_RTC_GetCounter(pPmRtcParam->u8PmRtcIndex);
}

void MDrv_RTC_SetMatchCount(PM_RtcParam *pPmRtcParam)
{
   //printk(KERN_EMERG "==RTC== %s, %d\n" , __FUNCTION__, __LINE__);
   MHAL_RTC_SetMatchCounter(pPmRtcParam->u8PmRtcIndex, pPmRtcParam->u32RtcSetMatchCounter);
}

U32 MDrv_RTC_GetMatchCount(PM_RtcParam *pPmRtcParam)
{
    //printk(KERN_EMERG "==RTC== %s, %d\n" , __FUNCTION__, __LINE__);
    return MHAL_RTC_GetMatchCounter(pPmRtcParam->u8PmRtcIndex);
}

U16 MDrv_SW_Dummy_Read(void)
{
    return MHAL_SLEEP_SW_Dummy_REG_Read();
}

int Request_RTC_IRQ(irq_handler_t handler, void *dev_id)
{
    return (MHal_RTC_Request_IRQ(handler, dev_id));
}
EXPORT_SYMBOL(Request_RTC_IRQ);

int Free_RTC_IRQ(void *dev_id)
{
    return 0;
}

void MDrv_RTC_Enable_Interrupt(void)
{
    MHal_RTC_Enable_Interrupt();
}

void MDrv_RTC_Disable_Interrupt(void)
{
    MHal_RTC_Disable_Interrupt();
}

EXPORT_SYMBOL(Free_RTC_IRQ);

bool MDrv_RTC_SetDummy(PM_RtcParam *pPmRtcParam, u16 val)
{
    if (pPmRtcParam->u8PmRtcDummyIndex == 0) {
        MHAL_RTC_Set_Dummy0(pPmRtcParam->u8PmRtcIndex, val);
    } else if (pPmRtcParam->u8PmRtcDummyIndex == 1) {
        MHAL_RTC_Set_Dummy1(pPmRtcParam->u8PmRtcIndex, val);
    } else {
        printk(KERN_ERR "[%s %d] rtc=%d, wrong dummy=%d, val=%d\n", __FUNCTION__, __LINE__, pPmRtcParam->u8PmRtcIndex, pPmRtcParam->u8PmRtcDummyIndex, val);
        return false;
    }

    return true;
}

bool MDrv_RTC_GetDummy(PM_RtcParam *pPmRtcParam, u16 *pval)
{
    if (pPmRtcParam->u8PmRtcDummyIndex == 0) {
        *pval = MHAL_RTC_Get_Dummy0(pPmRtcParam->u8PmRtcIndex);
    } else if (pPmRtcParam->u8PmRtcDummyIndex == 1) {
        *pval = MHAL_RTC_Get_Dummy1(pPmRtcParam->u8PmRtcIndex);
    } else {
        printk(KERN_ERR "[%s %d] rtc=%d, wrong dummy=%d\n", __FUNCTION__, __LINE__, pPmRtcParam->u8PmRtcIndex, pPmRtcParam->u8PmRtcDummyIndex);
        return false;
    }

    return true;
}
