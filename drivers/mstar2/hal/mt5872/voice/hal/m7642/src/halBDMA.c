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
/// file    halBDMA.c
/// @brief  MStar BDMA hal DDI
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#define _MHAL_BDMA_C

//=============================================================================
// Include Files
//=============================================================================
#include "halBDMA.h"

#include "mosWrapper.h"

#define BDMA_MSG(fmt, args...) MOS_DBG_PRINT(MOS_DBG_LEVEL_BDMA, "[BDMA] " fmt, ##args)
#define BDMA_ERR(fmt, args...) MOS_DBG_ERROR("[BDMA ERR] " fmt, ##args);


#define BDMA_SET_CH_REG(x)         (GET_REG16_ADDR(REG_ADDR_BASE_VBDMA,x))

#define BDMA_REG_CTRL(ch)           BDMA_SET_CH_REG(0x00+ch*0x10)
#define BDMA_REG_STATUS(ch)         BDMA_SET_CH_REG(0x01+ch*0x10)
#define BDMA_REG_DEV_SEL(ch)        BDMA_SET_CH_REG(0x02+ch*0x10)
#define BDMA_REG_MISC(ch)           BDMA_SET_CH_REG(0x03+ch*0x10)
#define BDMA_REG_SRC_ADDR_L(ch)     BDMA_SET_CH_REG(0x04+ch*0x10)
#define BDMA_REG_SRC_ADDR_H(ch)     BDMA_SET_CH_REG(0x05+ch*0x10)
#define BDMA_REG_DST_ADDR_L(ch)     BDMA_SET_CH_REG(0x06+ch*0x10)
#define BDMA_REG_DST_ADDR_H(ch)     BDMA_SET_CH_REG(0x07+ch*0x10)
#define BDMA_REG_SIZE_L(ch)         BDMA_SET_CH_REG(0x08+ch*0x10)
#define BDMA_REG_SIZE_H(ch)         BDMA_SET_CH_REG(0x09+ch*0x10)
#define BDMA_REG_CMD0_L(ch)         BDMA_SET_CH_REG(0x0A+ch*0x10)
#define BDMA_REG_CMD0_H(ch)         BDMA_SET_CH_REG(0x0B+ch*0x10)
#define BDMA_REG_CMD1_L(ch)         BDMA_SET_CH_REG(0x0C+ch*0x10)
#define BDMA_REG_CMD1_H(ch)         BDMA_SET_CH_REG(0x0D+ch*0x10)
#define BDMA_REG_CMD2_L(ch)         BDMA_SET_CH_REG(0x0E+ch*0x10)
#define BDMA_REG_CMD2_H(ch)         BDMA_SET_CH_REG(0x0F+ch*0x10)



//---------------------------------------------
// definition for BDMA_REG_CH0_CTRL/BDMA_REG_CH1_CTRL
//---------------------------------------------
#define BDMA_CH_TRIGGER             BIT(0)
#define BDMA_CH_STOP                BIT(4)


//---------------------------------------------
// definition for REG_BDMA_CH0_STATUS/REG_BDMA_CH1_STATUS
//---------------------------------------------
#define BDMA_CH_QUEUED              BIT(0)
#define BDMA_CH_BUSY                BIT(1)
#define BDMA_CH_INT                 BIT(2)
#define BDMA_CH_DONE                BIT(3)
#define BDMA_CH_RESULT              BIT(4)
#define BDMA_CH_CLEAR_STATUS        (BDMA_CH_INT|BDMA_CH_DONE|BDMA_CH_RESULT)

//---------------------------------------------
// definition for REG_BDMA_CH0_MISC/REG_BDMA_CH1_MISC
//---------------------------------------------

#define BDMA_CH_ADDR_DECDIR         BIT(0)
#define BDMA_CH_DONE_INT_EN         BIT(1)
#define BDMA_CH_CRC_REFLECTION      BIT(4)
#define BDMA_CH_MOBF_EN             BIT(5)

BOOL MHal_BDMA_WaitDone(BDMA_Ch channel,U32 u32Timeoutmsec)
{
    U32 count=0;
    for(count=0;count<u32Timeoutmsec;count++)
    {
        if(INREG16(BDMA_REG_STATUS(channel)) & BDMA_CH_DONE)
        {
            return TRUE;
        }
        MOS_mDelay(1);
    }

    return FALSE;
}

BOOL MHal_BDMA_PollingDone(BDMA_Ch channel)
{
    while(1)
    {
        if(INREG16(BDMA_REG_STATUS(channel)) & BDMA_CH_DONE)
        {
            return TRUE;
        }
    }
}

BOOL MHal_BDMA_IsBusy(BDMA_Ch channel)
{
    return (BDMA_CH_BUSY==(INREG16(BDMA_REG_STATUS(channel)) & BDMA_CH_BUSY));
}

BOOL MHal_BDMA_FlashToMIU(BDMA_Ch channel, U32 u32FlashAddr, U32 u32DramAddr, U32 u32Len)
{
    int i=0;
    //u32DramAddr &= (~MIU0_START_ADDR);
    if((u32Len & 0x0F) || (u32FlashAddr& 0x0F) || (u32DramAddr & 0x0F))
    {
    BDMA_ERR("MHal_BDMA_FlashToMIU: address or length should be 16 bytes aligned!!\n");
    return FALSE;
    }

    for(i=0;i<5000;i++)
    {
    if(0==(INREG16(BDMA_REG_STATUS(channel)) & BDMA_CH_BUSY))
    {
    break;
    }
      MOS_mDelay(1);
    }

    if(5000==i)
    {
    BDMA_ERR("MHal_BDMA_FLASH_TO_MEM error!! device is busy!!\n");
    return FALSE;
    }

    OUTREG16(BDMA_REG_STATUS(channel),BDMA_CH_CLEAR_STATUS);

    OUTREG16(BDMA_REG_DEV_SEL(channel),0x4035);
    OUTREG16(BDMA_REG_SRC_ADDR_L(channel),(U16)(u32FlashAddr & 0xFFFF));
    OUTREG16(BDMA_REG_SRC_ADDR_H(channel),(U16)((u32FlashAddr>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_DST_ADDR_L(channel),(U16)(u32DramAddr & 0xFFFF));
    OUTREG16(BDMA_REG_DST_ADDR_H (channel),(U16)((u32DramAddr>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_SIZE_L(channel),(U16)(u32Len & 0xFFFF));
    OUTREG16(BDMA_REG_SIZE_H(channel),(U16)((u32Len>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_CTRL(channel),BDMA_CH_TRIGGER);

    return TRUE;
}

BOOL MHal_BDMA_CalculateCRC32FromMIU(BDMA_Ch channel, U32 u32DramAddr, U32 u32Len)
{
    //u32DramAddr &= (~MIU0_START_ADDR);

    OUTREG16(BDMA_REG_STATUS(channel),BDMA_CH_CLEAR_STATUS);
    OUTREG16(BDMA_REG_MISC(channel), 0x0010); //reflection 1,checked with VAL

    OUTREG16(BDMA_REG_DEV_SEL(channel),0x0340);
    OUTREG16(BDMA_REG_SRC_ADDR_L(channel),(U16)(u32DramAddr & 0xFFFF));
    OUTREG16(BDMA_REG_SRC_ADDR_H(channel),(U16)((u32DramAddr>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_DST_ADDR_L(channel),(U16)(u32DramAddr & 0xFFFF));
    OUTREG16(BDMA_REG_DST_ADDR_H(channel),(U16)((u32DramAddr>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_SIZE_L(channel),(U16)(u32Len & 0xFFFF));
    OUTREG16(BDMA_REG_SIZE_H(channel),(U16)((u32Len>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_CMD0_L(channel), 0x1db7);
    OUTREG16(BDMA_REG_CMD0_H(channel), 0x04c1);

    OUTREG16(BDMA_REG_CMD1_L(channel), 0xFFFF);
    OUTREG16(BDMA_REG_CMD1_H(channel), 0xFFFF);

    OUTREG16(BDMA_REG_CTRL(channel),BDMA_CH_TRIGGER);

    return TRUE;
}
// ming ?
BOOL MHal_BDMA_CalculateCRC32FromIMI(BDMA_Ch channel, U32 u32SramAddr, U32 u32Len)
{
    //u32DramAddr &= (~MIU0_START_ADDR);

    OUTREG16(BDMA_REG_STATUS(channel),BDMA_CH_CLEAR_STATUS);

    OUTREG16(BDMA_REG_MISC(channel), 0x2010); //reflection 1,checked with VAL

    OUTREG16(BDMA_REG_DEV_SEL(channel),0x0340);
    OUTREG16(BDMA_REG_SRC_ADDR_L(channel),(U16)(u32SramAddr & 0xFFFF));
    OUTREG16(BDMA_REG_SRC_ADDR_H(channel),(U16)((u32SramAddr>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_DST_ADDR_L(channel),(U16)(u32SramAddr & 0xFFFF));
    OUTREG16(BDMA_REG_DST_ADDR_H(channel),(U16)((u32SramAddr>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_SIZE_L(channel),(U16)(u32Len & 0xFFFF));
    OUTREG16(BDMA_REG_SIZE_H(channel),(U16)((u32Len>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_CMD0_L(channel), 0x1db7);
    OUTREG16(BDMA_REG_CMD0_H(channel), 0x04c1);

    OUTREG16(BDMA_REG_CMD1_L(channel), 0xFFFF);
    OUTREG16(BDMA_REG_CMD1_H(channel), 0xFFFF);

    OUTREG16(BDMA_REG_CTRL(channel),BDMA_CH_TRIGGER);

    return TRUE;
}

// ming ?
BOOL MHal_BDMA_PatternSearchFromIMI(BDMA_Ch channel, U32 u32SramAddr, U32 u32Len, U32 u32Pattern)
{
    U16 u16RegVal;

    OUTREG16(BDMA_REG_STATUS(channel),BDMA_CH_CLEAR_STATUS);

    OUTREG16(BDMA_REG_MISC(channel), 0x2000); //reflection 1,checked with VAL

    OUTREG16(BDMA_REG_DEV_SEL(channel),0x0240);
    OUTREG16(BDMA_REG_SRC_ADDR_L(channel),(U16)(u32SramAddr & 0xFFFF));
    OUTREG16(BDMA_REG_SRC_ADDR_H(channel),(U16)((u32SramAddr>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_DST_ADDR_L(channel),(U16)(u32SramAddr & 0xFFFF));
    OUTREG16(BDMA_REG_DST_ADDR_H(channel),(U16)((u32SramAddr>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_SIZE_L(channel),(U16)(u32Len & 0xFFFF));
    OUTREG16(BDMA_REG_SIZE_H(channel),(U16)((u32Len>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_CMD0_L(channel), (U16)(u32Pattern & 0xFFFF));
    OUTREG16(BDMA_REG_CMD0_H(channel), (U16)((u32Pattern>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_CMD1_L(channel), 0);
    OUTREG16(BDMA_REG_CMD1_H(channel), 0);

    OUTREG16(BDMA_REG_CTRL(channel),BDMA_CH_TRIGGER);

    while(1)
    {
        u16RegVal = INREG16(BDMA_REG_STATUS(channel));
        if((u16RegVal & BDMA_CH_DONE) || (u16RegVal & BDMA_CH_RESULT) )
        {
            break;
        }
    }

    if(u16RegVal & BDMA_CH_RESULT)
        return TRUE;
    else
        return FALSE;
}

U32 MHal_BDMA_GetCRC32(BDMA_Ch channel)
{
    return (INREG16(BDMA_REG_CMD1_L(channel)) | (INREG16(BDMA_REG_CMD1_H(channel)) <<16));
}

BOOL MHal_BDMA_IMItoMIU(BDMA_Ch channel, U32 u32Src, U32 u32Dst, U32 u32Len)
{

    //u32Src &= (~MIU_BASE_ADDR);
    //u32Dst &= (~MIU_BASE_ADDR);

    OUTREG16(BDMA_REG_STATUS(channel),BDMA_CH_CLEAR_STATUS);

    OUTREG16(BDMA_REG_MISC(channel), 0x2000);// replace 00:MIU0, 01:MIU1, 10:IMI; [15:14] for CHN1, [13:12] for CHN0

    OUTREG16(BDMA_REG_DEV_SEL(channel),0x4140); //replace  channel0 to IMI, channel1 to MIU
    OUTREG16(BDMA_REG_SRC_ADDR_L(channel),(U16)(u32Src & 0xFFFF));
    OUTREG16(BDMA_REG_SRC_ADDR_H(channel),(U16)((u32Src>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_DST_ADDR_L(channel), (U16)(u32Dst & 0xFFFF));
    OUTREG16(BDMA_REG_DST_ADDR_H(channel), (U16)((u32Dst>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_SIZE_L(channel),(U16)(u32Len & 0xFFFF));
    OUTREG16(BDMA_REG_SIZE_H(channel),(U16)((u32Len>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_CTRL(channel),BDMA_CH_TRIGGER);

    return TRUE;
}

BOOL MHal_BDMA_MIUtoIMI(BDMA_Ch channel, U32 u32Src, U32 u32Dst, U32 u32Len)
{

    //u32Src &= (~MIU_BASE_ADDR);
    //u32Dst &= (~MIU_BASE_ADDR);

    OUTREG16(BDMA_REG_STATUS(channel),BDMA_CH_CLEAR_STATUS);

    //OUTREG16(BDMA_REG_MISC(channel),BDMA_CH_DONE_INT_EN | 0x2000);
    OUTREG16(BDMA_REG_MISC(channel), 0x8000);

    //OUTREG16(BDMA_REG_DEV_SEL(channel),0x4041);
    OUTREG16(BDMA_REG_DEV_SEL(channel),0x4140); //replace MIU channel0
    OUTREG16(BDMA_REG_SRC_ADDR_L(channel),(U16)(u32Src & 0xFFFF));
    OUTREG16(BDMA_REG_SRC_ADDR_H(channel),(U16)((u32Src>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_DST_ADDR_L(channel), (U16)(u32Dst & 0xFFFF));
    OUTREG16(BDMA_REG_DST_ADDR_H(channel), (U16)((u32Dst>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_SIZE_L(channel),(U16)(u32Len & 0xFFFF));
    OUTREG16(BDMA_REG_SIZE_H(channel),(U16)((u32Len>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_CTRL(channel),BDMA_CH_TRIGGER);

    return TRUE;
}


BOOL MHal_BDMA_IMItoIMI(BDMA_Ch channel, U32 u32Src, U32 u32Dst, U32 u32Len)
{

    //u32Src &= (~IMI_BASE_ADDR);
    //u32Dst &= (~IMI_BASE_ADDR);

    BDMA_MSG("MHal_BDMA_IMItoIMI = 0x%x -> 0x%x (0x%x)\n", u32Src, u32Dst, u32Len);

    OUTREG16(BDMA_REG_STATUS(channel),BDMA_CH_CLEAR_STATUS);

    OUTREG16(BDMA_REG_MISC(channel),0x8000);

    OUTREG16(BDMA_REG_DEV_SEL(channel),0x4141);

    OUTREG16(BDMA_REG_SRC_ADDR_L(channel),(U16)(u32Src & 0xFFFF));
    OUTREG16(BDMA_REG_SRC_ADDR_H(channel),(U16)((u32Src>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_DST_ADDR_L(channel), (U16)(u32Dst & 0xFFFF));
    OUTREG16(BDMA_REG_DST_ADDR_H(channel), (U16)((u32Dst>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_SIZE_L(channel),(U16)(u32Len & 0xFFFF));
    OUTREG16(BDMA_REG_SIZE_H(channel),(U16)((u32Len>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_CTRL(channel),BDMA_CH_TRIGGER);

    return TRUE;
}

BOOL MHal_BDMA_MIUtoMIU(BDMA_Ch channel, U32 u32Src, U32 u32Dst, U32 u32Len)
{

    //u32Src &= (~IMI_BASE_ADDR);
    //u32Dst &= (~IMI_BASE_ADDR);

    //BDMA_MSG("MHal_BDMA_IMItoIMI = 0x%x -> 0x%x (0x%x)\n", u32Src, u32Dst, u32Len);

    OUTREG16(BDMA_REG_STATUS(channel),BDMA_CH_CLEAR_STATUS);

    OUTREG16(BDMA_REG_MISC(channel),0x8000);

    OUTREG16(BDMA_REG_DEV_SEL(channel),0x4040);

    OUTREG16(BDMA_REG_SRC_ADDR_L(channel),(U16)(u32Src & 0xFFFF));
    OUTREG16(BDMA_REG_SRC_ADDR_H(channel),(U16)((u32Src>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_DST_ADDR_L(channel), (U16)(u32Dst & 0xFFFF));
    OUTREG16(BDMA_REG_DST_ADDR_H(channel), (U16)((u32Dst>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_SIZE_L(channel),(U16)(u32Len & 0xFFFF));
    OUTREG16(BDMA_REG_SIZE_H(channel),(U16)((u32Len>>16) & 0xFFFF));

    OUTREG16(BDMA_REG_CTRL(channel),BDMA_CH_TRIGGER);

    return TRUE;
}

BOOL MHal_BDMA_ClearStatus(BDMA_Ch channel)
{
    OUTREG16(BDMA_REG_STATUS(channel),BDMA_CH_CLEAR_STATUS);
    return TRUE;
}

void MHal_BDMA_Init(void)
{
	SETREG16(GET_REG8_ADDR(RIU_BASE_ADDR, 0x112ca4 ), 0x2); // bdma XTAIL Clock
	MOS_mDelay(1);
}
