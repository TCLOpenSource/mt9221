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
/// @file   mhal_graphic.c
/// @brief  MStar graphic Interface
/// @author MStar Semiconductor Inc. by Kyle.Ou
/// @attention
/// <b><em></em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////

//=============================================================================
// Include Files
//=============================================================================
#include <linux/kernel.h>   /* printk() */

#include "mdrv_mstypes.h"
#include "mdrv_graphic.h"
#include "mhal_graphic.h"
#include "mhal_graphic_reg.h"

//=============================================================================
// Macros
//=============================================================================
#define ALIGN_CHECK(value,factor) ((value + factor-1) & (~(factor-1)))

//=============================================================================
// Global Variables
//=============================================================================
MS_BOOL bForceWriteIn = FALSE;

MS_U16 _GOP_GetBPP(EN_DRV_GOPColorType fbFmt)
{
    MS_U16 u16bpp=0;

    switch (fbFmt)
    {
        case E_DRV_GOP_COLOR_RGB555_BLINK:
        case E_DRV_GOP_COLOR_RGB565:
        case E_DRV_GOP_COLOR_ARGB1555:
        case E_DRV_GOP_COLOR_RGBA5551:
        case E_DRV_GOP_COLOR_ARGB4444:
        case E_DRV_GOP_COLOR_RGBA4444:
        case E_DRV_GOP_COLOR_YUV422:
        case E_DRV_GOP_COLOR_RGB555YUV422:
        case E_DRV_GOP_COLOR_2266:
            u16bpp = 2;
            break;
        case E_DRV_GOP_COLOR_ARGB8888:
        case E_DRV_GOP_COLOR_ABGR8888:
            u16bpp = 4;
            break;
        case E_DRV_GOP_COLOR_I8:
            u16bpp = 1;
            break;
        default:
            u16bpp = 0xFFFF;
            break;
    }
    return u16bpp;
}

void _HAL_GOP_Init(MS_U8 u8GopIdx)
{
    MS_U8 u8Idx;
    MS_U32 bankoffset=0;

    _HAL_GOP_GetBnkOfstByGop(u8GopIdx,&bankoffset);
    _HAL_GOP_Write16Reg(bankoffset+GOP_4G_CTRL0, 0x0, GOP_BIT0);                      // GOP rst
    _HAL_GOP_Write16Reg(bankoffset+GOP_4G_CTRL0, 0x0,(GOP_BIT1|GOP_BIT2));   // clear H/V sync reserve
    _HAL_GOP_Write16Reg(bankoffset+GOP_4G_CTRL1, 0x4100, GOP_REG_HW_MASK);            // Set REGDMA interval

    //_HAL_GOP_Write16Reg(bankoffset+GOP_4G_HSTRCH, 0x1000, GOP_REG_WORD_MASK);
    //_HAL_GOP_Write16Reg(bankoffset+GOP_4G_VSTRCH, 0x1000, GOP_REG_WORD_MASK);
    _HAL_GOP_Write16Reg(bankoffset+GOP_4G_CTRL0, GOP_BIT14, GOP_BIT14);               // Set mask Hsync when VFDE is low

    _HAL_GOP_Write16Reg(bankoffset+GOP_4G_RATE, 0x0018, GOP_REG_WORD_MASK);           // enable blink capability - for ttx usage
    _HAL_GOP_Write16Reg(bankoffset+GOP_4G_CTRL0, 0x0000, GOP_BIT10);                  // Enable RGB output

    _HAL_GOP_Write16Reg(bankoffset+GOP_4G_BW, GOP_FIFO_BURST_ALL, GOP_FIFO_BURST_MASK);  //set GOP DMA Burst length to "32"
    _HAL_GOP_Write16Reg(bankoffset+GOP_4G_BW, GOP_FIFO_THRESHOLD, GOP_REG_LW_MASK);  //set DMA FIFO threshold to 3/4 FIFO length

    _HAL_GOP_Write16Reg(bankoffset+GOP_4G_OLDADDR, 0x0, (GOP_BIT1 | GOP_BIT0)); //temporally disable GOP clk dynamical gated - to avoid update palette problem

    if (u8GopIdx==0)
    {
        for (u8Idx=0; u8Idx<MAX_GOP0_GWIN; u8Idx++)
        {
                _HAL_GOP_Write16Reg(GOP_4G_DRAM_HVSTOP_L(u8Idx),0, GOP_REG_WORD_MASK);
                _HAL_GOP_Write16Reg(GOP_4G_DRAM_HVSTOP_H(u8Idx),0, GOP_REG_WORD_MASK);
        }
    }
}

void _HAL_GOP_SetHMirror(MS_U8 u8GOP,MS_BOOL bEnable)
{
    MS_U32 u32bankoffset=0;

    _HAL_GOP_GetBnkOfstByGop(u8GOP, &u32bankoffset);
    _HAL_GOP_Write16Reg(u32bankoffset+GOP_4G_CTRL0, bEnable?GOP_BIT12:0, GOP_BIT12);
}

void _HAL_GOP_SetVMirror(MS_U8 u8GOP,MS_BOOL bEnable)
{
    MS_U32 u32bankoffset=0;

    _HAL_GOP_GetBnkOfstByGop(u8GOP, &u32bankoffset);
    _HAL_GOP_Write16Reg(u32bankoffset+GOP_4G_CTRL0, bEnable?GOP_BIT13:0, GOP_BIT13);
}

void _HAL_GOP_OutputColor_EX(MS_U8 u8GOP,EN_DRV_GOP_OUTPUT_COLOR type)
{
    MS_U32 u32bankoffset=0;
    _HAL_GOP_GetBnkOfstByGop(u8GOP, &u32bankoffset);

    if(type == DRV_GOPOUT_RGB)
        _HAL_GOP_Write16Reg(u32bankoffset+GOP_4G_CTRL0, 0, GOP_BIT10);
    else
        _HAL_GOP_Write16Reg(u32bankoffset+GOP_4G_CTRL0, GOP_BIT10, GOP_BIT10);
}

void _HAL_GOP_MIUSel(MS_U8 u8GOP, E_DRV_GOP_SEL_TYPE MiuSel)
{
    MS_U32 u32bankoffset=0;

    _HAL_GOP_GetBnkOfstByGop(u8GOP, &u32bankoffset);
    _HAL_GOP_Write16Reg(u32bankoffset+GOP_4G_MIU_SEL, MiuSel, GOP_BIT0|GOP_BIT1);
}

void _HAL_GOP_Write16Reg(MS_U32 u32addr, MS_U16 u16val, MS_U16 mask)
{
    MS_U16 u16tmp = 0;
    u16tmp = GOP_READ2BYTE(u32addr);
    u16tmp &= ~mask;
    u16val &=  mask;
    u16val |=  u16tmp;
    GOP_WRITE2BYTE(u32addr,u16val);
}

void _HAL_GOP_Read16Reg(MS_U32 u32addr, MS_U16* pu16ret)
{
    *pu16ret = GOP_READ2BYTE(u32addr);
}

void _HAL_GOP_Write32Reg(MS_U32 u32addr, MS_U32 u32val)
{
    GOP_WRITE2BYTE(u32addr, (u32val&0xFFFF));
    GOP_WRITE2BYTE(u32addr+2, (u32val&0xFFFF0000)>>16);
}

MS_U8 _HAL_GOP_GetBnkOfstByGop(MS_U8 gop, MS_U32 *pBnkOfst)
{
    if (gop==0)
        *pBnkOfst = GOP_4G_OFST<<8;
    else if (gop==1)
        *pBnkOfst = GOP_2G_OFST<<8;
    else if (gop==2)
        *pBnkOfst = GOP_1G_OFST<<8;
    else if (gop==3)
        *pBnkOfst = GOP_1GX_OFST<<8;
    else if (gop==4)
        *pBnkOfst = GOP_DW_OFST<<8;
    else
        return FALSE;
    return TRUE;
}

void _HAL_GOP_GWIN_SetDstPlane(MS_U8 u8GOP, EN_DRV_GOP_DST_TYPE eDsttype, MS_BOOL bOnlyCheck)
{
    MS_BOOL b1pEnable;
    MS_U16 u16RegVal = 0, u16HSyncMask=GOP_BIT14;
    MS_U32 u32Bankoffset=0;

    switch (eDsttype)
    {
        case E_DRV_GOP_DST_IP0:
            u16RegVal = 0x0;
            b1pEnable = FALSE;
            u16HSyncMask=0;
            break;

        case E_DRV_GOP_DST_IP0_SUB:
            u16RegVal = 0x1;
            b1pEnable = FALSE;
            u16HSyncMask=0;
            break;

        case E_DRV_GOP_DST_OP0:
            u16RegVal = 0x2;
            b1pEnable = FALSE;
            u16HSyncMask=GOP_BIT14;
            break;

        case E_DRV_GOP_DST_VOP:
            u16RegVal = 0x3;
            b1pEnable = FALSE;
            u16HSyncMask=GOP_BIT14;
            break;

        case E_DRV_GOP_DST_VOP_SUB:
            u16RegVal = 0x4;
            b1pEnable = FALSE;
            u16HSyncMask=GOP_BIT14;
            break;

        case E_DRV_GOP_DST_FRC:
            u16RegVal = 0x6;
            b1pEnable = TRUE;
            u16HSyncMask=GOP_BIT14;
            break;

        case E_DRV_GOP_DST_VE:
            u16RegVal = 0x7;
            b1pEnable = TRUE;
            u16HSyncMask=GOP_BIT14;
            break;
        case E_DRV_GOP_DST_DIP:
            u16RegVal = 0x8;
            b1pEnable = TRUE;
            u16HSyncMask=GOP_BIT14;
            break;
        case E_DRV_GOP_DST_OP_DUAL_RATE:
            u16RegVal = 0xB;
            b1pEnable = FALSE;
            u16HSyncMask=0x0;
            break;
        default:
            printk("[%s] invalid DST value\n", __FUNCTION__);
            return;
    }

    if(bOnlyCheck == FALSE)
    {
        _HAL_GOP_GetBnkOfstByGop(u8GOP, &u32Bankoffset);
        _HAL_GOP_Write16Reg(u32Bankoffset + GOP_4G_SRAM_BORROW, b1pEnable?GOP_BIT14:0,GOP_BIT14);
        _HAL_GOP_Write16Reg(u32Bankoffset+GOP_4G_CTRL1, u16RegVal, BMASK(GOP_BIT3:0));
        _HAL_GOP_Write16Reg(u32Bankoffset+GOP_4G_CTRL0, u16HSyncMask, GOP_BIT14);               // Set mask Hsync when VFDE is low
    }
}

void _HAL_GOP_EnableTransClr_EX(MS_U8 u8GOP,EN_DRV_GOP_TRANSCLR_FMT eFmt, MS_BOOL bEnable)
{
    MS_U16 u16Regval = 0;
    MS_U32 u32BankOffSet = 0;

    _HAL_GOP_GetBnkOfstByGop(u8GOP, &u32BankOffSet);
    _HAL_GOP_Read16Reg(u32BankOffSet+GOP_4G_CTRL0, &u16Regval);

    if(bEnable)
    {
        if(eFmt == GOPTRANSCLR_FMT0)
            u16Regval |= GOP_RGB_TRANSPARENT_ENABLE;
        else
            u16Regval |= GOP_YUV_TRANSPARENT_ENABLE;
    }
    else
    {
        if(eFmt == GOPTRANSCLR_FMT0)
            u16Regval &= ~(GOP_RGB_TRANSPARENT_ENABLE);
        else
            u16Regval &= ~(GOP_YUV_TRANSPARENT_ENABLE);
    }

    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_CTRL0, u16Regval, (GOP_YUV_TRANSPARENT_ENABLE|GOP_RGB_TRANSPARENT_ENABLE));
}

MS_BOOL _HAL_GOP_IsVMirrorOn(MS_U8 u8GOPnum)
{
    MS_BOOL bRet = FALSE;
    MS_U16 u16RegVal = 0;
    MS_U32 u32BankOffSet = 0;

    _HAL_GOP_GetBnkOfstByGop(u8GOPnum, &u32BankOffSet);
    _HAL_GOP_Read16Reg(u32BankOffSet + GOP_4G_CTRL0, &u16RegVal);

    if(u16RegVal & GOP_VMIRROR_EN)
    {
        bRet = TRUE;
    }

    return bRet;
}

void _HAL_GOP_SetGwinInfo(MS_U8 u8GOP, MS_U8 u8win,DRV_GWIN_INFO WinInfo)
{
    MS_U16 u16Bpp = 0;
    MS_U32 u32BankOffSet = 0, u32RingBuffer = 0, u32Voffset = 0;
    MS_BOOL bPixelModeSupport = FALSE;

    _HAL_GOP_GetBnkOfstByGop(u8GOP, &u32BankOffSet);

    u16Bpp = _GOP_GetBPP(WinInfo.clrType);
    if(u16Bpp == 0xFFFF)
    {
        printk("[%s] invalud color format\n",__FUNCTION__);
        return;
    }

    WinInfo.u32Addr = ALIGN_CHECK(WinInfo.u32Addr,GOP_WordUnit);
    bPixelModeSupport = MHal_GRAPHIC_GetPixelModeSupport();
    if(bPixelModeSupport == TRUE)
    {
        WinInfo.u16Pitch = ALIGN_CHECK(WinInfo.u16Pitch,GOP_GWIN_SIZE_PIXELBASE_ALIGN);
        WinInfo.u16HStart = ALIGN_CHECK(WinInfo.u16HStart, GOP_GWIN_SIZE_PIXELBASE_ALIGN);
        WinInfo.u16HEnd = ALIGN_CHECK(WinInfo.u16HEnd, GOP_GWIN_SIZE_PIXELBASE_ALIGN);
    }
    else
    {
        WinInfo.u16Pitch = ALIGN_CHECK(WinInfo.u16Pitch,(GOP_WordUnit/u16Bpp));
        WinInfo.u16HStart = ALIGN_CHECK(WinInfo.u16HStart,(GOP_WordUnit/u16Bpp));
        WinInfo.u16HEnd = ALIGN_CHECK(WinInfo.u16HEnd,(GOP_WordUnit/u16Bpp));
    }

    //Color Fmt
    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_GWIN0_CTRL(u8win), WinInfo.clrType <<4,0x00f0);
    //Address
    _HAL_GOP_Write32Reg(u32BankOffSet+GOP_4G_DRAM_RBLK_L(u8win), WinInfo.u32Addr);
    //Pitch
    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_DRAM_RBLK_HSIZE(u8win), WinInfo.u16Pitch , GOP_REG_WORD_MASK);
    //HStart
    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_HSTR(u8win), (WinInfo.u16HStart*u16Bpp) , GOP_REG_WORD_MASK);
    //HEnd
    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_HEND(u8win), WinInfo.u16HEnd , GOP_REG_WORD_MASK);
    //VStart
    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_VSTR(u8win), WinInfo.u16VStart , GOP_REG_WORD_MASK);
    //VEnd
    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_VEND(u8win), WinInfo.u16VEnd, GOP_REG_WORD_MASK);
    //Ring Buffer
    u32RingBuffer = WinInfo.u16Pitch*(WinInfo.u16VEnd - WinInfo.u16VStart)*u16Bpp;
    _HAL_GOP_Write32Reg(u32BankOffSet+GOP_4G_DRAM_RBLK_SIZE_L(u8win), u32RingBuffer);

    switch(WinInfo.clrType)
    {
        case E_DRV_GOP_COLOR_RGB555_BLINK:
        case E_DRV_GOP_COLOR_RGB565:
        case E_DRV_GOP_COLOR_ARGB4444:
        case E_DRV_GOP_COLOR_2266:
        case E_DRV_GOP_COLOR_I8:
        case E_DRV_GOP_COLOR_ARGB8888:
        case E_DRV_GOP_COLOR_ARGB1555:
        case E_DRV_GOP_COLOR_ABGR8888:
        case E_DRV_GOP_COLOR_RGBA5551:
        case E_DRV_GOP_COLOR_RGBA4444:
            _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_CTRL_CSC, 0,GOP_BIT4);
            break;
        case E_DRV_GOP_COLOR_YUV422:
            _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_CTRL_CSC, GOP_BIT4,GOP_BIT4);
            break;
        default:
            printk("[%s][fbdev] not support color fmt:%d!\n",__FUNCTION__,WinInfo.clrType);
            break;
    }

    if(_HAL_GOP_IsVMirrorOn(u8GOP))
    {
       u32Voffset = WinInfo.u16HEnd * (WinInfo.u16VEnd - 1) * u16Bpp;
       _HAL_GOP_Write32Reg(u32BankOffSet+GOP_4G_DRAM_VSTR_L(u8win), u32Voffset);
    }
    else
    {
        _HAL_GOP_Write32Reg(u32BankOffSet+GOP_4G_DRAM_VSTR_L(u8win), u32Voffset);
    }
}

void _HAL_GOP_SetStretchWin(MS_U8 u8GOP,MS_U16 u16x, MS_U16 u16y, MS_U16 u16width, MS_U16 u16height)
{
    MS_U32 u32BankOffSet = 0;

    _HAL_GOP_GetBnkOfstByGop(u8GOP, &u32BankOffSet);
    _HAL_GOP_Write16Reg(u32BankOffSet + GOP_4G_STRCH_HSTR, u16x, GOP_REG_WORD_MASK);
    _HAL_GOP_Write16Reg(u32BankOffSet + GOP_4G_STRCH_VSTR, u16y , GOP_REG_WORD_MASK);
    _HAL_GOP_Write16Reg(u32BankOffSet + GOP_4G_STRCH_HSZ, (u16width/GOP_STRETCH_WIDTH_UNIT) , GOP_REG_WORD_MASK);
    _HAL_GOP_Write16Reg(u32BankOffSet + GOP_4G_STRCH_VSZ, u16height , GOP_REG_WORD_MASK);
    _HAL_GOP_Write16Reg(u32BankOffSet + GOP_4G_RDMA_HT, (u16width+3)/2, 0x07ff);
}

void _HAL_GOP_SetHScale(MS_U8 u8GOP,MS_BOOL bEnable, MS_U16 u16src, MS_U16 u16dst)
{
    MS_U32 u32hratio =0x1000,u32BankOffSet=0;

    _HAL_GOP_GetBnkOfstByGop(u8GOP, &u32BankOffSet);
    if (bEnable)
    {
        u32hratio = (MS_U32)(u16src) * SCALING_MULITPLIER;
        if(u32hratio %(MS_U32)u16dst !=0)
            u32hratio = (u32hratio/(MS_U32)u16dst)+1;
        else
            u32hratio /= (MS_U32)u16dst;
    }
    else
        u32hratio = SCALING_MULITPLIER;

    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_HSTRCH, u32hratio , GOP_REG_WORD_MASK);
    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_HSTRCH_INI, 0 , GOP_REG_WORD_MASK);
}

void _HAL_GOP_SetVScale(MS_U8 u8GOP,MS_BOOL bEnable, MS_U16 u16src, MS_U16 u16dst)
{
    MS_U32 u32vratio =0x1000,u32BankOffSet=0;

    _HAL_GOP_GetBnkOfstByGop(u8GOP, &u32BankOffSet);

    if (bEnable)
    {
        u32vratio = (MS_U32)(u16src) * SCALING_MULITPLIER;
        if(u32vratio %(MS_U32)u16dst !=0)
            u32vratio = (u32vratio/(MS_U32)u16dst)+1;
        else
            u32vratio /= (MS_U32)u16dst;
    }
    else
        u32vratio = SCALING_MULITPLIER;

    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_VSTRCH, u32vratio , GOP_REG_WORD_MASK);
    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_VSTRCH_INI, 0 , GOP_REG_WORD_MASK);
}

void _HAL_GOP_GettXCTiming(MS_U16* pu16Width, MS_U16* pu16Height)
{
    MS_U16 u16Reg1 = 0;
    MS_U16 u16Reg2 = 0;
    _HAL_GOP_Read16Reg(XC_BK10_04_L, &u16Reg1);
    _HAL_GOP_Read16Reg(XC_BK10_05_L, &u16Reg2);
    *pu16Width = u16Reg2 - u16Reg1 + 1;

    _HAL_GOP_Read16Reg(XC_BK10_06_L, &u16Reg1);
    _HAL_GOP_Read16Reg(XC_BK10_07_L, &u16Reg2);
    *pu16Height = u16Reg2 - u16Reg1 + 1;
}

void _HAL_GOP_GWIN_Enable(MS_U8 u8GOP, MS_U8 u8win,MS_BOOL bEnable)
{
    MS_U32 u32BankOffSet=0;

    _HAL_GOP_GetBnkOfstByGop(u8GOP, &u32BankOffSet);
    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_CTRL0, 0x0, GOP_BIT11);       // Enable Transparent Color
    _HAL_GOP_Write16Reg(u32BankOffSet+GOP_4G_GWIN0_CTRL(u8win), bEnable, 0x1);
}

void _HAL_GOP_UpdateReg(MS_U8 u8Gop)
{
    MS_U32 u32BankOffSet=0;

    if(bForceWriteIn)
    {
        _HAL_GOP_Write16Reg(GOP_BAK_SEL, GOP_VAL_FWR, GOP_REG_HW_MASK);
        _HAL_GOP_Write16Reg(GOP_BAK_SEL, 0x0000, GOP_REG_HW_MASK);
    }
    else
    {
        _HAL_GOP_GetBnkOfstByGop(u8Gop,&u32BankOffSet);

        if(u8Gop == 0)
            _HAL_GOP_Write16Reg(GOP_BAK_SEL, GOP_BIT12, GOP_BIT12);
        else if(u8Gop == 1)
            _HAL_GOP_Write16Reg(GOP_BAK_SEL, GOP_BIT13, GOP_BIT13);
        else if(u8Gop == 2)
            _HAL_GOP_Write16Reg(GOP_BAK_SEL, GOP_BIT14, GOP_BIT14);
        else if(u8Gop == 3)
            _HAL_GOP_Write16Reg(GOP_BAK_SEL, GOP_BIT15, GOP_BIT15);
        else if(u8Gop == 4)
            _HAL_GOP_Write16Reg(GOP_BAK_SEL, GOP_BIT4, GOP_BIT4);
    }
}

void _HAL_GOP_SetForceWrite(MS_BOOL bEnable)
{
    bForceWriteIn = bEnable;
}

void _HAL_GOP_SetBlending(MS_U8 u8GOP, MS_U8 u8win, MS_BOOL bEnable, MS_U8 u8coef)
{
    MS_U32 u32bankoffset = 0;

    _HAL_GOP_GetBnkOfstByGop(u8GOP, &u32bankoffset);
    _HAL_GOP_Write16Reg(u32bankoffset+GOP_4G_GWIN0_CTRL(u8win), bEnable<<14, 0x4000);
    _HAL_GOP_Write16Reg(u32bankoffset+GOP_4G_GWIN_ALPHA01(u8win), u8coef, 0xff);
}

MS_BOOL MHal_GRAPHIC_GetPixelModeSupport(void)
{
    return TRUE;
}

MS_BOOL MHal_GRAPHIC_IsInit(MS_U8 u8GOPNum)
{
    MS_U16 u16RegVal = 0;
    MS_U32 u32Bankoffset = 0;

    _HAL_GOP_GetBnkOfstByGop(u8GOPNum, &u32Bankoffset);

    _HAL_GOP_Read16Reg(u32Bankoffset + GOP_4G_HS_PIPE, &u16RegVal);
    if(u16RegVal == 0)
    {
        return FALSE;
    }

    _HAL_GOP_Read16Reg(XC_BK00_06_L, &u16RegVal);
    if(u16RegVal == 0)
    {
        return FALSE;
    }

    return TRUE;
}

MS_BOOL MHal_GRAPHIC_InitGopPart(MS_U8 u8GOPNum)
{
    MS_U32 u32Bankoffset = 0;
    MS_U8 u8MuxIdx = 0, u8GOPNumInMux = 0;

    _HAL_GOP_GetBnkOfstByGop(u8GOPNum, &u32Bankoffset);

    _HAL_GOP_Write16Reg(u32Bankoffset+GOP_4G_CTRL0, GOP_BIT15, GOP_BIT15);
    // set VIP/VOP timing select, always select VOP should be OK.
    _HAL_GOP_Write16Reg(u32Bankoffset+GOP_4G_MULTI_ALPHA, GOP_OP_VDE_SEL, GOP_OP_VDE_SEL);
    // enable pixel mode
    _HAL_GOP_Write16Reg(u32Bankoffset+GOP_4G_BANK_FWR, GOP_PIXEL_BASE_EN, GOP_PIXEL_BASE_EN);

    switch(u8GOPNum)
    {
        case E_GOP0:
            _HAL_GOP_Write16Reg(GOP_GOP0CLK, ~(GOP_G0_DISABLE_CLK), GOP_G0_DISABLE_CLK);
            break;
        case E_GOP1:
            _HAL_GOP_Write16Reg(GOP_GOP0CLK, ~(GOP_G1_DISABLE_CLK), GOP_G1_DISABLE_CLK);
            break;
        case E_GOP2:
            _HAL_GOP_Write16Reg(GOP_GOP2CLK, ~(GOP_G2_DISABLE_CLK), GOP_G2_DISABLE_CLK);
            break;
        default:
            printk("[%s][fbdev] not support GOP:%d!\n",__FUNCTION__,u8GOPNum);
            break;
    }

    if(MHal_GOP_SetPipeDelay(u8GOPNum) == FALSE)
    {
        return FALSE;
    }

    for(u8MuxIdx = 0; u8MuxIdx < MAX_GOP_MUX; u8MuxIdx++)
    {
        u8GOPNumInMux = MHal_GOP_GetMux((EN_GOP_MUXSEL)u8MuxIdx);
        if(u8GOPNum == u8GOPNumInMux)
        {
            MHal_GOP_SetOSDBEnable((EN_GOP_MUXSEL)u8MuxIdx);
        }
    }

    return TRUE;
}

MS_BOOL MHal_GOP_SetPipeDelay(MS_U8 u8GopIdx)
{
    MS_U32 u32Bankoffset = 0;
    MS_U16 u16PipeDelay = 0, u16PanelHstart = 0;
    MS_U8 u8MuxIdx = 0,u8GOPNumInMux = 0;

    _HAL_GOP_GetBnkOfstByGop(u8GopIdx, &u32Bankoffset);
    _HAL_GOP_Read16Reg(XC_BK10_04_L, &u16PanelHstart);

    for(u8MuxIdx = 0; u8MuxIdx < MAX_GOP_MUX; u8MuxIdx++)
    {
        u8GOPNumInMux = MHal_GOP_GetMux((EN_GOP_MUXSEL)u8MuxIdx);

        if(u8GopIdx == u8GOPNumInMux)
        {
            switch(u8MuxIdx)
            {
                case E_GOP_MUX0:
                    u16PipeDelay = u16PanelHstart + (GOP_MUX0_Offset * GOP_MUX_DELTA) + GOP_PD;
                    break;
                case E_GOP_MUX1:
                    u16PipeDelay = u16PanelHstart + (GOP_MUX1_Offset * GOP_MUX_DELTA) + GOP_PD;
                    break;
                case E_GOP_MUX2:
                    u16PipeDelay = u16PanelHstart + (GOP_MUX2_Offset * GOP_MUX_DELTA) + GOP_PD;
                    break;
                case E_GOP_MUX3:
                    u16PipeDelay = u16PanelHstart + (GOP_MUX3_Offset * GOP_MUX_DELTA) + GOP_PD;
                    break;
                case E_GOP_MUX4:
                    u16PipeDelay = u16PanelHstart + (GOP_MUX4_Offset * GOP_MUX_DELTA) + GOP_PD;
                    break;
                default:
                    printk("[%s][fbdev] not support GOP:%d!\n",__FUNCTION__,u8GopIdx);
                    return FALSE;
            }
            _HAL_GOP_Write16Reg(u32Bankoffset+GOP_4G_HS_PIPE, u16PipeDelay, GOP_REG_WORD_MASK);
        }
    }

    return TRUE;
}

MS_U8 MHal_GOP_GetMux(EN_GOP_MUXSEL enGopMux)
{
    MS_U16 u16GopMux;

    _HAL_GOP_Read16Reg(GOP_MUX, &u16GopMux);
    return (MS_U8)(u16GopMux >> (enGopMux*GOP_MUX_SHIFT))& GOP_REGMUX_MASK;
}

MS_BOOL MHal_GOP_SetOSDBEnable(EN_GOP_MUXSEL enGopMux)
{
    switch(enGopMux)
    {
        case E_GOP_MUX0:
            _HAL_GOP_Write16Reg(XC_BK00_06_L, GOP_BIT15,GOP_BIT15);
            break;
        case E_GOP_MUX1:
            _HAL_GOP_Write16Reg(XC_BK00_06_L, GOP_BIT14,GOP_BIT14);
            break;
        case E_GOP_MUX2:
            _HAL_GOP_Write16Reg(XC_BK00_06_L, GOP_BIT13,GOP_BIT13);
            break;
        case E_GOP_MUX3:
            _HAL_GOP_Write16Reg(XC_BK00_06_L, GOP_BIT12,GOP_BIT12);
            break;
        case E_GOP_MUX4:
            _HAL_GOP_Write16Reg(XC_BK00_06_L, GOP_BIT11,GOP_BIT11);
            break;
        default:
            printk("[%s][fbdev] not support OpMux:%d!!\n",__FUNCTION__,enGopMux);
            return FALSE;
    }

    return TRUE;
}
