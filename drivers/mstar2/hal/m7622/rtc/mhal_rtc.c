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

#include "mhal_rtc.h"
#include "mhal_rtc_reg.h"
#include "chip_int.h"
//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
#define REG16(_u32RegBase)     *((volatile U16*)(RIU_MAP+((_u32RegBase)<<1)))

static E_MS_RTC  g_rtc_num = E_RTC_0;
irq_handler_t pm_rtc_irq_pCallback = NULL;

static void MHAL_RTC_Write2Byte(U32 u32RegBase, U16 data)
{
	REG16(u32RegBase) = data;
}

static U16 MHAL_RTC_Read2Byte(U32 u32RegBase)
{
	return REG16(u32RegBase);
}

static void MHAL_RTC_WriteBit(U32 u32RegBase, U16 u16Val, bool bEnable)
{
    if(bEnable)
		REG16(u32RegBase)|=u16Val;
    else
		REG16(u32RegBase)&=(~u16Val);
}

static void MHAL_RTC_Write4Byte(U32 _u32RegBase, U32 data)
{
	REG16(_u32RegBase) = data & 0xFFFFUL;
	REG16((_u32RegBase+0x2UL)) = (data >>16) & 0xFFFFUL;
}

static U32 MHAL_RTC_Read4Byte(U32 _u32RegBase)
{
	U32 data = 0;
	data = REG16(_u32RegBase+0x2UL) ;
	data = data <<16 | REG16(_u32RegBase);
	return data;
}

static U32 MHAL_RTC_GET_BASE(E_MS_RTC eRtc)
{
    U32 u32RegAddr=0;
    switch(eRtc)
    {
        case E_RTC_0:
            u32RegAddr=REG_RTC_BASE_0;
            break;
        case E_RTC_2:
            u32RegAddr=REG_RTC_BASE_2;
            break;
    }
    return u32RegAddr;

}

irqreturn_t _MHAL_RTCINT_INTHandler(int irq, void *dev_id)
{
    U8 i;
    U16 u16sts;
    unsigned long flags;
    irq_handler_t  pCB;

    if(MHAL_RTC_GetInterrupt_Status(g_rtc_num))
    {
        pCB = pm_rtc_irq_pCallback;
        if(pCB != 0)
        {
            pCB(0, dev_id);
        }
        MHAL_RTC_ClearInterrupt_Status(g_rtc_num);
        return IRQ_HANDLED;
    }
    return IRQ_NONE;
}



int MHal_RTC_Request_IRQ(irq_handler_t pCallback, void *dev_id)
{
    pm_rtc_irq_pCallback = NULL;

    if (!pCallback)
        return -EBUSY;

    pm_rtc_irq_pCallback = pCallback;

#ifdef CONFIG_KEYBOARD_MTK
    if (request_irq(E_IRQ_PM_SLEEP, (irq_handler_t)_MHAL_RTCINT_INTHandler, IRQF_TRIGGER_HIGH | IRQF_SHARED, "RTC_PM", dev_id))
#else
    if (request_irq(E_IRQ_PM_SLEEP, (irq_handler_t)_MHAL_RTCINT_INTHandler, IRQF_TRIGGER_HIGH, "RTC_PM", dev_id))
#endif
    {
        printk("request_irq fail\n");
        return -EBUSY;
    }

    MHAL_RTC_WriteBit(REG_WK_IRQ_MASK ,RTC_WK_SRC, 0);

    return 0;
}

void MHal_RTC_Enable_Interrupt(void)
{
#if 1
    printk("rtc %s\n", __func__);
    //ret = MHal_PM_RTC_Interrupt_Init();
    //pm_rtc_irq_pCallback= pCallback;
    //rtc_irq_dev_id= dev_id;
    MHAL_RTC_ClearInterrupt_Status(g_rtc_num);
    //MHAL_RTC_WriteBit(REG_HST0_IRQ_MASK_15_0_, PM_SLEEP_INT_MASK, 0);
    MHAL_RTC_WriteBit(REG_WK_IRQ_MASK, RTC_WK_SRC, 0);
    MHAL_RTC_WriteBit(MHAL_RTC_GET_BASE(g_rtc_num) + REG_RTC_CTRL_REG, RTC_INT_MASK_BIT, 0);
#endif
}

void MHal_RTC_Disable_Interrupt(void)
{
#if 1
    printk("rtc %s\n", __func__);
    //MHAL_RTC_WriteBit(REG_HST0_IRQ_MASK_15_0_,PM_SLEEP_INT_MASK,1);
    MHAL_RTC_WriteBit(MHAL_RTC_GET_BASE(g_rtc_num) + REG_RTC_CTRL_REG, RTC_INT_MASK_BIT, 1);
    MHAL_RTC_WriteBit(REG_WK_IRQ_MASK, RTC_WK_SRC, 1);
    //pm_rtc_irq_pCallback= NULL;
    //rtc_irq_dev_id= NULL;
#endif
}
//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------

void MHAL_RTC_Reading(E_MS_RTC eRtc, bool bEnable)
{
    MHAL_RTC_WriteBit(MHAL_RTC_GET_BASE(eRtc)+REG_RTC_CTRL_REG,RTC_READ_EN_BIT,bEnable);
}

void MHAL_RTC_Loading (E_MS_RTC eRtc, bool bEnable)
{
    MHAL_RTC_WriteBit(MHAL_RTC_GET_BASE(eRtc)+REG_RTC_CTRL_REG,RTC_LOAD_EN_BIT,bEnable);
}

void MHAL_RTC_RESET(E_MS_RTC eRtc, bool bEnable)
{
    MHAL_RTC_WriteBit(MHAL_RTC_GET_BASE(eRtc)+REG_RTC_CTRL_REG,RTC_SOFT_RSTZ_BIT,bEnable);
}

void MHAL_RTC_Counter(E_MS_RTC eRtc, bool bEnable)
{
    MHAL_RTC_WriteBit(MHAL_RTC_GET_BASE(eRtc)+REG_RTC_CTRL_REG,RTC_CNT_EN_BIT,bEnable);
}

void MHAL_RTC_Init(E_MS_RTC eRtc,U32 u32Xtal)
{
    MHAL_RTC_WriteBit(REG_PM_CKG_RTC,BIT2,ENABLE); //RTC clock switch to 12MHz
    MHAL_RTC_RESET(eRtc, ENABLE);
    MHAL_RTC_Write4Byte(MHAL_RTC_GET_BASE(eRtc)+REG_RTC_FREQ_CW, u32Xtal);
    MHAL_RTC_Counter(eRtc,ENABLE);
}

void MHAL_RTC_SetCounter(E_MS_RTC eRtc,U32 u32RtcSetCounter)
{
    MHAL_RTC_Write4Byte(MHAL_RTC_GET_BASE(eRtc)+REG_RTC_LOAD_VAL , u32RtcSetCounter);
    MHAL_RTC_Loading(eRtc, ENABLE);
}

U32 MHAL_RTC_GetCounter(E_MS_RTC eRtc)
{
    U32 u32Reg;
    U16 val;
    U32 cnt = 0;

    MHAL_RTC_Reading(eRtc, ENABLE);
    //wait for HW latch bits okay, otherwise sometimes it read wrong value
    do {
        val = MHAL_RTC_Read2Byte(MHAL_RTC_GET_BASE(eRtc) + REG_RTC_CTRL_REG);
        cnt++;
    } while ((val & RTC_READ_EN_BIT) && (cnt < 1000));

    u32Reg = MHAL_RTC_Read4Byte(MHAL_RTC_GET_BASE(eRtc)+REG_RTC_CNT);
    printk(KERN_EMERG "==RTC== %s, %d, time:0x%x, val:0x%x, cnt:%d\n" , __FUNCTION__, __LINE__, u32Reg, val, cnt);
    return u32Reg;
}

void MHAL_RTC_SetMatchCounter(E_MS_RTC eRtc,U32 u32RtcSetCounter)
{
    MHAL_RTC_Write4Byte(MHAL_RTC_GET_BASE(eRtc) + REG_RTC_MATCH_VAL_L , u32RtcSetCounter);
    MHAL_RTC_Write4Byte(MHAL_RTC_GET_BASE(eRtc) + REG_RTC_MATCH_VAL_H , 0);
}

U32 MHAL_RTC_GetMatchCounter(E_MS_RTC eRtc)
{
    U32 u32Reg;
    u32Reg = MHAL_RTC_Read4Byte(MHAL_RTC_GET_BASE(eRtc) + REG_RTC_MATCH_VAL_L);
    //printk(KERN_EMERG "==RTC== %s, %d, %x\n" , __FUNCTION__, __LINE__, u32Reg);
    return u32Reg;
}

U16 MHAL_RTC_GetInterrupt_Status(E_MS_RTC eRtc)
{
    u16 val;
    val = MHAL_RTC_Read2Byte(MHAL_RTC_GET_BASE(eRtc) + REG_RTC_CTRL_REG);
    return (val & RTC_INT_STATUS_BIT);
}

void MHAL_RTC_ClearInterrupt_Status(E_MS_RTC eRtc)
{
    MHAL_RTC_WriteBit(MHAL_RTC_GET_BASE(eRtc) + REG_RTC_CTRL_REG, RTC_INT_CLEAR_BIT, 1);
}

U16 MHAL_SLEEP_SW_Dummy_REG_Read(void)
{
    return(MHAL_RTC_Read2Byte(REG_SLEEP_BASE+SLEEP_DUMMY_REG));
}

void MHAL_RTC_Set_Dummy0(E_MS_RTC eRtc, u16 u16Val)
{
    MHAL_RTC_Write2Byte(MHAL_RTC_GET_BASE(eRtc) + REG_RTC_DUMMY0, u16Val);
}

u16 MHAL_RTC_Get_Dummy0(E_MS_RTC eRtc)
{
    return MHAL_RTC_Read2Byte(MHAL_RTC_GET_BASE(eRtc) + REG_RTC_DUMMY0);
}

void MHAL_RTC_Set_Dummy1(E_MS_RTC eRtc, u16 u16Val)
{
    MHAL_RTC_Write2Byte(MHAL_RTC_GET_BASE(eRtc) + REG_RTC_DUMMY1, u16Val);
}

u16 MHAL_RTC_Get_Dummy1(E_MS_RTC eRtc)
{
    return MHAL_RTC_Read2Byte(MHAL_RTC_GET_BASE(eRtc) + REG_RTC_DUMMY1);
}
