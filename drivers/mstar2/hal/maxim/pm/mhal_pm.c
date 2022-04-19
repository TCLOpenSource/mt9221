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
#include "uapi/linux/psci.h"
#include "mdrv_types.h"
#include "mdrv_pm.h"
#include "mhal_pm.h"
#include "mhal_pm_reg.h"
//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
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
/* 8-bit RIU access. */
u8 MHal_PM_ReadReg8(u32 reg)
{
    return  *((volatile u8 *)(PM_RIU_REG_BASE + (reg << 1)) - (reg & 1));
}
void MHal_PM_WriteReg8(u32 reg, u8 val)
{
    *((volatile u8 *)(PM_RIU_REG_BASE + (reg << 1) - (reg & 1))) = val;
}

/*16-bit RIU address*/
u16 MHal_PM_ReadReg16(u32 reg)
{
    u16 val;
    val = *((volatile u16 *) (PM_RIU_REG_BASE + (reg << 1)));
    return val;
}
void MHal_PM_WriteReg16(u32 reg, u16 val)
{
    *((volatile u16 *) (PM_RIU_REG_BASE + (reg << 1))) = val;
}

/* 8-bit RIU mask access. */
void MHal_PM_WriteReg8Mask(u32 reg, u8 mask, u8 enable)
{
    u8 val = 0;

    val = MHal_PM_ReadReg8(reg);
    /* Set. */
    val |= (mask & enable);
    /* Clear. */
    val &= ~(mask & ~enable);
    MHal_PM_WriteReg8(reg, val);
}

void MHal_PM_WakeIrqMask(u8 mask)
{
    u8 valtmp = 0;
    valtmp = MHal_PM_ReadReg8(REG_PM_WK_MASK);
    //printk("\033[45;37m  ""%s[%x] ::   \033[0m\n",__FUNCTION__ ,valtmp);
    valtmp &= ~mask;
    MHal_PM_WriteReg8(REG_PM_WK_MASK, valtmp);
}

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
u8 MHal_PM_GetPowerOnKey(void)
{
    return ((u8)(MHal_PM_ReadReg16(REG_MCU_IR_POWERON_KEY) >> 8));
}

void MHal_PM_CleanPowerOnKey(void)
{
    *((volatile u16 *) (PM_RIU_REG_BASE + (REG_MCU_IR_POWERON_KEY << 1))) &= 0x00FF;
}
//-------------------------------------------------------------------------------------------------
void MHal_PM_Disable_8051(void)
{
    MHal_PM_WriteReg16(REG_PM_CPU_SW_RST, MHal_PM_ReadReg16(REG_PM_CPU_SW_RST) & ~(BIT12 | BIT8));
}

PM_Result MHal_PM_SetSRAMOffsetForMCU(void)
{
    u32 start_addr = 0x0000UL;
    u32 end_addr   = 0x5FFFUL; // Size: 24K
    u32 u32Cnt;
    //Disable 8051
    MHal_PM_WriteReg16(REG_PM_CPU_SW_RST, MHal_PM_ReadReg16(REG_PM_CPU_SW_RST) & ~(BIT12|BIT8));
    //MCK CLK=XTAL
    MHal_PM_WriteReg16(REG_PM_MCU_CLK, MHal_PM_ReadReg16(REG_PM_MCU_CLK) & ~BIT7);
    //i_cache rstz
    MHal_PM_WriteReg16(REG_MCU_CONFIG, MHal_PM_ReadReg16(REG_MCU_CONFIG) & ~BIT3);
    //disable i cache (enable icache bypass)
    MHal_PM_WriteReg16(REG_MCU_CACHE_CONFIG, MHal_PM_ReadReg16(REG_MCU_CACHE_CONFIG) | BIT0);
    //DISALBE SPI/DRAM
    MHal_PM_WriteReg16(REG_MCU_CONFIG, MHal_PM_ReadReg16(REG_MCU_CONFIG) & ~(BIT1|BIT2|BIT3));
    //ENABLE SRAM
    MHal_PM_WriteReg16(REG_MCU_CONFIG, MHal_PM_ReadReg16(REG_MCU_CONFIG) | BIT0);
    //Set Sram address
    MHal_PM_WriteReg16(REG_MCU_SRAM_START_ADDR_L, (u16)  start_addr & 0xFFFFUL);
    MHal_PM_WriteReg16(REG_MCU_SRAM_START_ADDR_H, (u16) (start_addr >> 16) & 0xFFUL);
    MHal_PM_WriteReg16(REG_MCU_SRAM_END_ADDR_L, (u16)  end_addr & 0xFFFFUL);
    MHal_PM_WriteReg16(REG_MCU_SRAM_END_ADDR_H, (u16) (end_addr >> 16) & 0xFFUL);
    MHal_PM_WriteReg16(REG_PM_RST_CPU0_PASSWORD, 0x829fUL);
    extern uint32_t isPSCI;
    if(    TEEINFO_TYPTE==SECURITY_TEEINFO_OSTYPE_OPTEE && isPSCI == PSCI_RET_SUCCESS)
    {
        printk("PM_SetSRAMOffsetForMCU: OPTEE flow\n");
    }
    else
    {
        printk("PM_SetSRAMOffsetForMCU: Not OPTEE flow(TYPE:%d)\n", TEEINFO_TYPTE);
    MHal_PM_WriteReg16(REG_PM_CPU_SW_RST, MHal_PM_ReadReg16(REG_PM_CPU_SW_RST) | BIT8);
    udelay(1200);
    MHal_PM_WriteReg16(REG_PM_CPU_SW_RST, MHal_PM_ReadReg16(REG_PM_CPU_SW_RST) | BIT12);
    printk("PM Wait for PM51 standby...........\n");
    u32Cnt = 0;
    while(MHal_PM_ReadReg16(REG_PM_LOCK) != 0x02UL)
    {
        u32Cnt++;
        if (u32Cnt == 0x10000)
        {
            printk("PM51 run fail...........\n");
            return E_PM_TIMEOUT;
        }
        udelay(120);
    }
    }
    printk("PM51 run ok...........\n");
    return E_PM_OK;
}

//-------------------------------------------------------------------------------------------------
PM_Result MHal_PM_CopyBin2Sram(u32 u32SrcAddr)
{
    u32 u32Cnt;
    //Disable 8051
    MHal_PM_WriteReg16(REG_PM_CPU_SW_RST, MHal_PM_ReadReg16(REG_PM_CPU_SW_RST) & ~(BIT12|BIT8));
    MHal_PM_WriteReg16(0x100904UL, 0x0a40);
    //src address
    MHal_PM_WriteReg16(0x100908UL, u32SrcAddr & 0xffff);
    MHal_PM_WriteReg16(0x10090aUL, (u32SrcAddr >> 16) & 0xffff);
    //dst address
    MHal_PM_WriteReg16(0x10090cUL, 0x0000);
    MHal_PM_WriteReg16(0x10090eUL, 0x0000);
    //set size
    MHal_PM_WriteReg16(0x100910UL, 0x6000);
    MHal_PM_WriteReg16(0x100912UL, 0x0000);
    MHal_PM_WriteReg16(0x100900UL, 0x0001);
    u32Cnt=0;
    while(MHal_PM_ReadReg16(0x100900UL) & 0x1)
    {
        u32Cnt++;
        if (u32Cnt == 0x10000)
        {
            return E_PM_TIMEOUT;
        }
        udelay(120);
    }
    return E_PM_OK;
}
//-------------------------------------------------------------------------------------------------
void MHal_PM_SetDram2Register(u32 u32DramAddr)
{
	u8 loop;
	u16 u16temp;
	for (loop=0; loop<0x08; loop++)
	{
		MHal_PM_WriteReg16(REG_MBX_0+(loop<<1), 0x0000);
	}
	MHal_PM_WriteReg16(REG_MBX_0, 0x2015);
	u16temp = (u16) (u32DramAddr >> 16);
	MHal_PM_WriteReg16(REG_MBX_1, (u16temp << 8) + (u16temp >> 8) );
	u16temp = (u16)u32DramAddr;
	MHal_PM_WriteReg16(REG_MBX_2, (u16temp << 8) + (u16temp >> 8) );
	MHal_PM_WriteReg16(REG_MBX_3, 0x55aa);
}


//-------------------------------------------------------------------------------------------------
void MHal_PM_RunTimePM_Disable_PassWord(void)
{
    if (MHal_PM_ReadReg16(0xeaa) == 0xbbbb)
    {
        MHal_PM_WriteReg16(0xeaa, 0xbabe);
        while(MHal_PM_ReadReg16(0xeaa) != 0xaaaa);
        MHal_PM_WriteReg16(0xeaa, 0x1234);
        //Disable 8051
        MHal_PM_WriteReg16(REG_PM_CPU_SW_RST, MHal_PM_ReadReg16(REG_PM_CPU_SW_RST) & ~(BIT12|BIT8));
    }
}

void MHal_PM_SetDRAMOffsetForMCU(u32 u32Offset)
{
    u32 start_addr = 0x0000000UL;
    u32 end_addr = 0xFFFFFFFFUL;
    u8 times = 0;

    MHal_PM_WriteReg8(REG_PM_LOCK, 0x00UL); //Clear bit

    MHal_PM_WriteReg8Mask(REG_PM_MCU_CLK, BIT7, ~BIT7);          //mcu51 clk=xtal
    MHal_PM_WriteReg8Mask(REG_MCU_CONFIG, BIT3, ~BIT3);          //i_cache rstz
    MHal_PM_WriteReg8Mask(REG_MCU_CACHE_CONFIG, BIT0 , BIT0);    //disable i cache (enable icache bypass)
    MHal_PM_WriteReg8Mask(REG_MCU_CONFIG, BIT1, ~BIT1);          //SPI  disable
    MHal_PM_WriteReg8Mask(REG_MCU_CONFIG, BIT2, BIT2);           //DRAM enable
    MHal_PM_WriteReg8Mask(REG_MCU_CONFIG, BIT0, ~BIT0);          //SRAM disable

    MHal_PM_WriteReg16(REG_MCU_DRAM_START_ADDR_0, (u16)start_addr & 0x0000FFFFUL);        //reg_dram_start_addr_0
    MHal_PM_WriteReg16(REG_MCU_DRAM_START_ADDR_1, (u16)(start_addr >> 16) & 0x0000FFFFUL);
    MHal_PM_WriteReg16(REG_MCU_DRAM_END_ADDR_0, (u16)end_addr & 0x0000FFFFUL);            //reg_dram_end_addr_0
    MHal_PM_WriteReg16(REG_MCU_DRAM_END_ADDR_1, (u16)(end_addr >> 16) & 0x00000FFFFUL);

    //set reset password 0x0e54=0x829f
    MHal_PM_WriteReg16(REG_PM_RST_CPU0_PASSWORD, 0x829fUL);

    MHal_PM_WriteReg16(REG_ICACHE_SDRAM_CODE_MAP, (u32Offset >> 16));

    //reg_fire[3:0]                   0x000e53 = 0x01
    //reg_fire[3:0]                   0x000e53 = 0x00
    MHal_PM_WriteReg8(REG_PM_CPU_SW_RST_P, MHal_PM_ReadReg8(REG_PM_CPU_SW_RST_P) | BIT0);
    udelay(120); // delay 120us
    MHal_PM_WriteReg8(REG_PM_CPU_SW_RST_P, MHal_PM_ReadReg8(REG_PM_CPU_SW_RST_P) | BIT4); //release 8051  reset

    printk(KERN_ERR "[%s][%d] Wait for PM51 standby DRAM ...........\n", __func__, __LINE__);
    while ((MHal_PM_ReadReg16(REG_PM_LOCK) != 0x02UL))
    {
        if (times > 20)
        {
            // it will try to run RT_PM in 400ms, when time's up, the function will return directly
            printk(KERN_ERR "[%s][%d] PM51 run failed...........\n", __func__, __LINE__);
            return;
        }
        // delay 20ms
        mdelay(20);
        times++;
    }
    printk(KERN_ERR "[%s][%d] PM51 run ok...........\n", __func__, __LINE__);
}

u8 MHal_PM_GetWakeupSource(void)
{
    return MHal_PM_ReadReg8(REG_PM_DUMMY_WAKEUP_SOURCE);
}
