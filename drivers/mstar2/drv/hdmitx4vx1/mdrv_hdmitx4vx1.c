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
/// file    mdrv_hdmitx4vx1.c
/// @brief  HDMITX4VX1 Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/kthread.h>

//driver include
#include "mdrv_hdmitx4vx1.h"
#include "mdrv_iic_io.h"
#include "haltx_timing_atop_tbl.h"
#include "haltx_timing_atop_tbl.c"

#include "haltx_timing_lpll_tbl.h"
#include "haltx_timing_lpll_tbl.c"

#include "haltx_timing_vx1_tbl.h"
#include "haltx_timing_vx1_tbl.c"

#include "haltx_timing_tbl.h"
#include "haltx_timing_tbl.c"

#include "haltx_timing_3x3matrix_tbl.h"
#include "haltx_timing_3x3matrix_tbl.c"



//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define HDMITX4VX1_I2C_SLAVE_ID        0xA2
#define HDMITX4VX1_I2C_BUS_NUM         2
#define HDMITX4VX1_STATE_IS_READY      0xB430
#define HDMITX4VX1_CHIP_STATE          0x1EFE
#define HDMITX4VX1_MAILBOX_REG_BASE    0x1200
#define HDMITX4VX1_STATUS              0x1252

#define FLAGS_SIZE              2       //unit: byte
#define DEVICENUM_SIZE          1       //unit: byte
#define INDEX_SIZE              1       //unit: byte
#define BSTATUS_SIZE            2       //unit: byte
#define MAX_DEVICE_NUM          12      //Maximum support device number
#define DEVICE_KSV_LIST_SIZE    5       //unit: byte. Each device has 5Bytes data for its KSV
#define MAX_KSV_LIST_SIZE       (MAX_DEVICE_NUM*DEVICE_KSV_LIST_SIZE)      //unit: byte
#define MAX_KSV_BSTATUS_SIZE    (FLAGS_SIZE + DEVICENUM_SIZE + INDEX_SIZE + BSTATUS_SIZE + MAX_KSV_LIST_SIZE)
#define INVALID_DEVIE_NUM       (0xFF)




//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
EN_HDMITX4VX1_VIDEO_OUTPUT_TYPE genHDMITX_Mode = E_HDMITX4VX1_VIDEO_OUTPUT_HDMI;
EN_HDMITX4VX1_INPUT_COLOR_TYPE  genHDMITx_InColor = E_HDMITX4VX1_COLORS_INPUT_RGB;
EN_HDMITX4VX1_OUTPUT_COLOR_TYPE genHDMITx_OutColor = E_HDMITX4VX1_COLORS_OUTPUT_YUV_422;
EN_HDMITX4VX1_COLOR_DEPTH       genHDMITx_ColorDepth = E_HDMITX4VX1_COLORS_NOID;

EN_HDMITX4VX1_AUDIO_FREQUENCY_TYPE    genAudio_Freq = E_HDMITX4VX1_AUDIO_48KHz;
EN_HDMITX4VX1_AUDIO_CHANNEL_COUNT     genAudio_Ch_Cnt = E_HDMITX4VX1_AUDIO_CH_2;
EN_HDMITX4VX1_AUDIO_CODING_TYPE       genAudio_CodeType = E_HDMITX4VX1_AUDIO_PCM;
EN_HDMITX4VX1_AUDIO_SOURCE_FORMAT     genAudio_SrcFmt = E_HDMITX4VX1_AUDIO_FORMAT_PCM;
EN_HDMITX4VX1_AUDIO_SOURCE_TYPE       genAudio_SrcType = E_HDMITX4VX1_AUDIO_I2S;
EN_HDMITX4VX1_VIDEO_TIMING            genHDMITx_VideoTiming = E_HDMITX4VX1_TIMING_1920x1080p_60Hz;
EN_HDMITX4VX1_VIDEO_3D_STRUCTURE_TYPE genHDMITx_3D = E_HDMITX4VX1_VIDEO_3D_Not_in_Use;
EN_HDMITX4VX1_VIDEO_4K2K_VIC          genHDMITx_4K2K_VIC = E_HDMITX4VX1_VIDEO_4K2K_Reserved;



//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------

static MS_BOOL HDMITX4VX1_IIC_WriteRegister(MS_U16 u16SlaveCfg, MS_U32 uAddrCnt, MS_U8 *RegAddr, MS_U32 uSize, MS_U8 *RegData)
{
    if(MDrv_SW_IIC_WriteBytes(HDMITX4VX1_I2C_BUS_NUM, u16SlaveCfg, uAddrCnt, RegAddr,uSize,RegData)<0)
    {
        HDMITX4VX1_KDBG("%s, fail\n", __func__);
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

static MS_BOOL HDMITX4VX1_IIC_ReadRegister(MS_U16 u16SlaveCfg, MS_U32 uAddrCnt, MS_U8 *pRegAddr, MS_U32 uSize, MS_U8 *pData)
{
    if(MDrv_SW_IIC_ReadBytes(HDMITX4VX1_I2C_BUS_NUM, u16SlaveCfg, uAddrCnt, pRegAddr, uSize, pData)<0)
    {
        HDMITX4VX1_KDBG("%s, fail\n", __func__);
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

MS_BOOL HDMITX4VX1_WriteBytes(MS_U16 u16Addr, MS_U8 u8Buf)
{
    MS_U8 u8Databuf[5] = {0, 0, 0, 0, 0};
    MS_BOOL bResult = TRUE;
    u8Databuf[0] = 0x10;
    u8Databuf[3] = u8Buf;

    u8Databuf[1] = (MS_U8)(u16Addr >> 8);          //the high byte need not move left 1 bit
    u8Databuf[2] = (MS_U8)((u16Addr & 0xFF));        //the low byte moves left 1 bit, reset bit0 means data low byte

    if(HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 4, u8Databuf) == FALSE)
    {
        HDMITX4VX1_KDBG("IIC Write fail\n");
        bResult = HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 4, u8Databuf);
    }
    return bResult;

}

MS_BOOL HDMITX4VX1_ReadBytes(MS_U16 u16Addr, MS_U8* pBuf)
{

    MS_BOOL bResult = TRUE;
    MS_U8 u8Databuf[6]={0, 0, 0, 0, 0, 0};

    u8Databuf[0] = 0x10;

    u8Databuf[1] = (MS_U8)(u16Addr >> 8);          //the high byte need not move left 1 bit
    u8Databuf[2] = (MS_U8)((u16Addr & 0xFF));        //the low byte moves left 1 bit, reset bit0 means data low byte

    if(HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 3, u8Databuf) == FALSE)
    {
        HDMITX4VX1_KDBG("IIC Read fail 1\n");
        bResult &= (HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 3, u8Databuf));
    }

    if(HDMITX4VX1_IIC_ReadRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, pBuf) == FALSE)
    {
        HDMITX4VX1_KDBG("IIC Read fail 2\n");
        bResult &= (HDMITX4VX1_IIC_ReadRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, pBuf));
    }
    return bResult;

}

MS_BOOL HDMITX4VX1_WriteBytesMask(MS_U16 u16Addr, MS_U8 u8Mask, MS_U8 u8Buf)
{
    MS_U8 u8Databuf[5] = {0, 0, 0, 0, 0};
    MS_BOOL bResult = TRUE;
    MS_U8 u8regval = 0x0;

    u8Databuf[0] = 0x10;

    if( HDMITX4VX1_ReadBytes(u16Addr,&u8regval) == FALSE)
    {
        HDMITX4VX1_KDBG("IIC Read fail 1\n");
    }
    u8Databuf[3]  = ((u8regval & (~(u8Mask))) | u8Buf);

    u8Databuf[1] = (MS_U8)(u16Addr >> 8);          //the high byte need not move left 1 bit
    u8Databuf[2] = (MS_U8)((u16Addr & 0xFF));        //the low byte moves left 1 bit, reset bit0 means data low byte

    if(HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 4, u8Databuf) == FALSE)
    {
        HDMITX4VX1_KDBG("IIC Write fail\n");
        bResult = HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 4, u8Databuf);
    }
    return bResult;
}

void HDMITX4VX1_w2byte(MS_U16 u16Addr, MS_U16 u16val)
{
    MS_U8 u8regval = 0;

    u8regval = (MS_U8)(u16val & 0xFF);
    HDMITX4VX1_WriteBytes(u16Addr, u8regval);

    u8regval = (MS_U8)(u16val >> 8);
    HDMITX4VX1_WriteBytes(u16Addr+1, u8regval);
}


MS_U16 HDMITX4VX1_r2byte(MS_U16 u16Addr)
{
    MS_U8 u8regval = 0;
    MS_U16 u16regval = 0;

    HDMITX4VX1_ReadBytes(u16Addr, &u8regval);
    u16regval = u8regval;

    HDMITX4VX1_ReadBytes(u16Addr+1, &u8regval);
    u16regval |= (((MS_U16)u8regval)<<8);

    return u16regval;
}

MS_BOOL Mailbox_WriteCommand(EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx, MS_U8 data_len, MS_U8 *pBuf)
{
    int i=0;
    HDMITX4VX1_WriteBytes(HDMITX4VX1_MAILBOX_REG_BASE | 0x00,command_idx);
    HDMITX4VX1_WriteBytes(HDMITX4VX1_MAILBOX_REG_BASE | 0x01,data_len);
    for(i=0;i<data_len;i++)
    {
         HDMITX4VX1_WriteBytes(HDMITX4VX1_MAILBOX_REG_BASE + 0x02 + i,pBuf[i]);
    }
    return TRUE;
}

MS_BOOL Mailbox_ReadCommand(EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE *command_idx, MS_U8 *data_len, MS_U8 *pBuf)
{
    MS_U8 read_command = 0;
    MS_U8 read_datalen=0;
    int i=0;

    HDMITX4VX1_ReadBytes(HDMITX4VX1_MAILBOX_REG_BASE | 0x00,&read_command);
    switch(read_command)
    {
        case E_MAILBOX_CONFIQ_VIDEO:
        case E_MAILBOX_CONFIQ_AUDIO:
        case E_MAILBOX_AVMUTE_TIMING_RESET:
        case E_MAILBOX_QUERY_EDID_INFO:
        case E_MAILBOX_REPORT_EDID_INFO:
        case E_MAILBOX_ACK_STATUS_RESPOND:
        case E_MAILBOX_DEBUG:
        case E_MAILBOX_COMBINE_COMMAND:
        case E_MAILBOX_HDCP_COMMAND:
            *command_idx=(EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE)read_command;
            break;
        default:
            HDMITX4VX1_KDBG("Mailbox_ReadCommand Fail!!! Wrong Command Type!!!\n");
            return FALSE;
    }
    HDMITX4VX1_ReadBytes(HDMITX4VX1_MAILBOX_REG_BASE | 0x01,&read_datalen);
    *data_len=read_datalen;

    for(i=0;i<(*data_len);i++)
    {
         HDMITX4VX1_ReadBytes(HDMITX4VX1_MAILBOX_REG_BASE + 0x02 + i,&pBuf[i]);
    }
    return TRUE;

}

MS_BOOL Mailbox_WriteCommand_Config_Video(ST_MAILBOX_COMMAND_CONFIQ_VIDEO *pBuf)
{
    MS_U8 pData[7];
    int dataIdx=1;
    memset(pData, 0, sizeof(MS_U8)*7);

    HDMITX4VX1_KDBG("%s, %d\n",__func__,__LINE__);
    HDMITX4VX1_KDBG("CommandType(%x), DataLen(%x), TimingPre(%x), ColorPre(%x), VS3DPre(%x), AnalogPre(%x), TimingIdx(%x), ColorDepth(%x), OutputMode(%x), InColor(%x), OutColor(%x), VS3D(%x), Current(%x), Pren2(%x), Precon(%x), Tenpre(%x), Ten(%x)\n",
    (int)pBuf->command_idx,(int)pBuf->data_len,
    (int)pBuf->timing_present,(int)pBuf->color_present,(int)pBuf->VSinfo_3D_present,(int)pBuf->analog_present,
    (int)pBuf->timing_idx,(int)pBuf->color_depth,(int)pBuf->output_mode,(int)pBuf->in_color,(int)pBuf->out_color,(int)pBuf->vs_3d,
    (int)pBuf->currentforvideo,(int)pBuf->pren2,(int)pBuf->precon,(int)pBuf->tenpre,(int)pBuf->ten);

    if (pBuf->command_idx!=E_MAILBOX_CONFIQ_VIDEO)
    {
        HDMITX4VX1_KDBG("Wrong Command Type in %s !!!\n", __func__);
        return FALSE;
    }
    if (pBuf->data_len>7)
    {
        HDMITX4VX1_KDBG("Wrong Data Length for %s, Maximum Data Length is 7 !!!\n", __func__);
        return FALSE;
    }

    pData[0]=(pBuf->analog_present<<4)|(pBuf->hdcp_present<<3)|(pBuf->VSinfo_3D_present<<2)|(pBuf->color_present<<1)|pBuf->timing_present;

    if(pBuf->timing_present && (pBuf->data_len>dataIdx))
    {
        pData[dataIdx]=pBuf->timing_idx;
        dataIdx++;
        pData[dataIdx]=(pBuf->output_mode<<3) | pBuf->color_depth;
        if(!pBuf->color_present)
            dataIdx++;
    }
    if(pBuf->color_present && (pBuf->data_len>dataIdx))
    {
        if(pBuf->timing_present)
        {
            pData[dataIdx]=(pBuf->out_color<<6) | (pBuf->in_color<<5) | pData[dataIdx];
        }
        else
        {
            pData[dataIdx]=(pBuf->out_color<<6) | (pBuf->in_color<<5);
        }
        dataIdx++;
    }
    if((pBuf->VSinfo_3D_present || pBuf->hdcp_present) && (pBuf->data_len>dataIdx))
    {
        pData[dataIdx]=(pBuf->hdcp<<4)|(pBuf->vs_3d);
        dataIdx++;
    }
    if(pBuf->analog_present && (pBuf->data_len>dataIdx))
    {
        pData[dataIdx]=(pBuf->pren2<<4) | pBuf->currentforvideo;
        pData[dataIdx+1]=(pBuf->tenpre<<4) | pBuf->precon;
    }

    return Mailbox_WriteCommand(pBuf->command_idx,pBuf->data_len,pData);

}

MS_BOOL Mailbox_WriteCommand_Config_Audio(ST_MAILBOX_COMMAND_CONFIQ_AUDIO *pBuf)
{
    MS_U8 pData[5];
    int dataIdx=1;
    memset(pData, 0, sizeof(MS_U8)*5);

    HDMITX4VX1_KDBG("%s, %d\n",__func__,__LINE__);
    HDMITX4VX1_KDBG("CommandType(%x), DataLen(%x), InformPre(%x), SourcePre(%x), FormatPre(%x), Frequency(%x), ChannelNum(%x), CodeType(%x), SourceType(%x), SourceFormat(%x)\n",
    (int)pBuf->command_idx,(int)pBuf->data_len,
    (int)pBuf->inform_present,(int)pBuf->source_present,(int)pBuf->fmt_present,
    (int)pBuf->frequency,(int)pBuf->channel_num,(int)pBuf->code_type,(int)pBuf->source_type,(int)pBuf->source_fmt);

    if(pBuf->command_idx!=E_MAILBOX_CONFIQ_AUDIO)
    {
        HDMITX4VX1_KDBG("Wrong Command Type in %s !!!\n",__func__);
        return FALSE;
    }
    if(pBuf->data_len>5)
    {
        HDMITX4VX1_KDBG(" Wrong Data Length for %s, Maximum Data Length is 5 !!!\n",__func__);
        return FALSE;
    }

    pData[0]=(pBuf->fmt_present<<2)|(pBuf->source_present<<1)|pBuf->inform_present;

    if(pBuf->inform_present && (pBuf->data_len>dataIdx))
    {
        pData[dataIdx]=(pBuf->channel_num<<4)|pBuf->frequency;
        pData[dataIdx+1]=pBuf->code_type;
        dataIdx=dataIdx+2;
    }
    if(pBuf->source_present && (pBuf->data_len>dataIdx))
    {
        pData[dataIdx]=pBuf->source_type;
        dataIdx++;
    }
    if(pBuf->fmt_present && (pBuf->data_len>dataIdx))
    {
        pData[dataIdx]=pBuf->source_fmt;
    }

    return Mailbox_WriteCommand(pBuf->command_idx,pBuf->data_len,pData);

}

MS_BOOL Mailbox_WriteCommand_TimingChange_AVmute(ST_MAILBOX_COMMAND_TIMING_CHANGE_AVMUTE *pBuf)
{
    MS_U8 pData[2];
    memset(pData, 0, sizeof(MS_U8)*2);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);
    HDMITX4VX1_KDBG(" CommandType(%x), DataLen(%x), VideoFlag(%x), AudioFlag(%x), AVMuteFlag(%x), TimingChangeFlag(%x), EnVideo(%x), EnAudio(%x), EnAVMute(%x), EnTimingChange(%x)\n",
    (int)pBuf->command_idx,(int)pBuf->data_len,
    (int)pBuf->video_flag,(int)pBuf->audio_flag,(int)pBuf->avmute_flag,(int)pBuf->timing_flag,
    (int)pBuf->enVideo_mute,(int)pBuf->enAudio_mute,(int)pBuf->enAV_mute,(int)pBuf->enTiming);

    HDMITX4VX1_KDBG(" hdmitxpower_flag(%x), enHdmiPower(%x)\n",(int)pBuf->hdmitxpower_flag,(int)pBuf->enHdmiPower);

    if (pBuf->command_idx!=E_MAILBOX_AVMUTE_TIMING_RESET)
    {
        HDMITX4VX1_KDBG(" Wrong Command Type in %s !!!\n",__func__);
        return FALSE;
    }
    if (pBuf->data_len>2)
    {
        HDMITX4VX1_KDBG(" Wrong Data Wrong for %s, Maximum Data Length is 2 !!!\n",__func__);
        return FALSE;
    }

    pData[0]=(pBuf->hdmitxpower_flag<<4)|(pBuf->timing_flag<<3)|(pBuf->avmute_flag<<2)|(pBuf->audio_flag<<1)|pBuf->video_flag;
    pData[1]=(pBuf->enHdmiPower<<4)|(pBuf->enTiming<<3)|(pBuf->enAV_mute<<2)|(pBuf->enAudio_mute<<1)|pBuf->enVideo_mute;

    return Mailbox_WriteCommand(pBuf->command_idx,pBuf->data_len,pData);

}

MS_BOOL Mailbox_WriteCommand_Query_EDID(ST_MAILBOX_COMMAND_QUERY_EDID_INFO *pBuf)
{
    MS_U8 pData[3];
    memset(pData, 0, sizeof(MS_U8)*3);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);
    HDMITX4VX1_KDBG(" CommandType(%x), DataLen(%x), QueryType(%x), 3DTiming(%x), DescriptorStrIdx(%x), DescriptorEndIdx(%x), 64BytesIdx(%x), EdidIdx(%x)\n",
    (int)pBuf->command_idx,(int)pBuf->data_len,(int)pBuf->query_type,
    (int)pBuf->timing_3d,(int)pBuf->des_startIdx,(int)pBuf->des_endIdx,(int)pBuf->bytes64_Idx,(int)pBuf->edid_Idx);

    if (pBuf->command_idx!=E_MAILBOX_QUERY_EDID_INFO)
    {
        HDMITX4VX1_KDBG(" Wrong Command Type in %s !!!\n",__func__);
        return FALSE;
    }
    if (pBuf->data_len>3)
    {
        HDMITX4VX1_KDBG(" Wrong Data Length for %s, Maximum Data Length is 3 !!!\n",__func__);
        return FALSE;
    }

    pData[0]=pBuf->query_type;
    if(pBuf->data_len>1)
    {
        switch(pBuf->query_type)
        {
                case E_HDMITX4VX1_QUERY_3D_STRUCTURE:
                    pData[1]=pBuf->timing_3d;
                    pData[2]=0;
                    break;
                case E_HDMITX4VX1_QUERY_AUDIO_DESCRIPTOR:
                    pData[1]=pBuf->des_startIdx;
                    pData[2]=pBuf->des_endIdx;
                    break;
                case E_HDMITX4VX1_QUERY_VIDEO_DESCRIPTOR:
                    pData[1]=pBuf->des_startIdx;
                    pData[2]=pBuf->des_endIdx;
                    break;
                case E_HDMITX4VX1_QUERY_EDID_DATA:
                    pData[1]=pBuf->edid_Idx;
                    pData[2]=pBuf->bytes64_Idx;
                    break;
                case E_HDMITX4VX1_QUERY_COLOR_FORMAT:
                    pData[1]=pBuf->timing_colorfmt;
                    pData[2]=0;
                    break;
                case E_HDMITX4VX1_QUERY_KSV_BSTATUS:
                    pData[1]=pBuf->KSV_B_Idx;
                    pData[2]=0;
                    break;
                default:
                    HDMITX4VX1_KDBG(" Wrong Data Length for this Query Type in Mailbox Write Command EDID Information !!!\n");
                    return FALSE;
        }
    }

    return Mailbox_WriteCommand(pBuf->command_idx,pBuf->data_len,pData);

}

MS_BOOL Mailbox_WriteCommand_HDCP_COMD(ST_MAILBOX_COMMAND_HDCP_COMD *pBuf)
{
    MS_U8 pData[22];
    MS_U8 pDataKSVB[MAX_KSV_BSTATUS_SIZE];
    MS_U8 Vpos_idx=0;
    MS_U8 Data_info_len = FLAGS_SIZE + DEVICENUM_SIZE + INDEX_SIZE + BSTATUS_SIZE;
    int i=0;
    memset(pData, 0, sizeof(MS_U8)*22);
    memset(pDataKSVB, 0, sizeof(MS_U8)*MAX_KSV_BSTATUS_SIZE);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);
    HDMITX4VX1_KDBG(" CommandType(%x), DataLen(%x), DDC_pre(%x), RepeaterV_pre(%x), RepeaterReset_pre(%x), DDC_port(%x), DDC_en(%x), ResetRepeater_en(%x)\n",
    (int)pBuf->command_idx,(int)pBuf->data_len,
    (int)pBuf->ddcport_present,(int)pBuf->repeaterV_present,(int)pBuf->repeaterReset_present,
    (int)pBuf->ddc_port,(int)pBuf->enddc_port,(int)pBuf->enRepeaterReset);
    HDMITX4VX1_KDBG(" user_hpd_present(%x), hdcp14repeater_present(%x), enUserHPD(%x), enHDCP14Repeater(%x)\n",
    (int)pBuf->user_hpd_present,(int)pBuf->hdcp14repeater_present,(int)pBuf->enUserHPD,(int)pBuf->enHDCP14Repeater);
    HDMITX4VX1_KDBG(" repeatersourcehdmi_present(%x), enSourceHDMI(%x)\n",
    (int)pBuf->repeatersourcehdmi_present,(int)pBuf->enSourceHDMI);
    HDMITX4VX1_KDBG(" repeate_en_present(%x), enOpenRepeater(%x)\n",
    (int)pBuf->repeate_en_present,(int)pBuf->enOpenRepeater);
    HDMITX4VX1_KDBG(" ext_present(%x), KSVbstatus_present(%x), device_num(%x), Idx_forKSVBStatus(%x)\n",
    (int)pBuf->ext_present,(int)pBuf->KSVbstatus_present,(int)pBuf->device_num,(int)pBuf->Idx_forKSVBStatus);
    HDMITX4VX1_KDBG(" BStatus[0](%x), BStatus[1](%x)\n",
    (int)pBuf->BStatus[0],(int)pBuf->BStatus[1]);
    HDMITX4VX1_KDBG(" SOC_hpd_present(%x), enSOCHPD(%x)\n",
    (int)pBuf->SOC_hpd_present,(int)pBuf->enSOCHPD);

    if (pBuf->command_idx!=E_MAILBOX_HDCP_COMMAND)
    {
        HDMITX4VX1_KDBG(" Wrong Command Type in %s !!!\n",__func__);
        return FALSE;
    }

    //special data form for setting KSV & B status, if KSV_Bstatus_present rises, other present_flags are useless
    if(pBuf->ext_present && pBuf->KSVbstatus_present)
    {
        HDMITX4VX1_KDBG(" Set KSV and BStatus in %s !!!\n",__func__);

        pDataKSVB[0]=pBuf->ext_present<<7;
        pDataKSVB[1]=pBuf->KSVbstatus_present;
        pDataKSVB[2]=pBuf->device_num;
        pDataKSVB[3]=pBuf->Idx_forKSVBStatus;
        pDataKSVB[4]=pBuf->BStatus[0];
        pDataKSVB[5]=pBuf->BStatus[1];
        if(pBuf->device_num!=0)
        {
            for(i=0;i<MAX_KSV_LIST_SIZE;i++)
            {
                pDataKSVB[i+Data_info_len]=pBuf->KSVlist[i];
            }
        }

        return Mailbox_WriteCommand(pBuf->command_idx,pBuf->data_len,pDataKSVB);
    }

    //special data form for SOC HPD, if SOC_hpd_present rises, other present_flags are useless
    if(pBuf->ext_present && pBuf->SOC_hpd_present)
    {
        HDMITX4VX1_KDBG(" Set SOC HPD %s !!!\n",__func__);
        pData[0]=pBuf->enSOCHPD;

        return Mailbox_WriteCommand(pBuf->command_idx,pBuf->data_len,pData);
    }

    //normal data form
    pData[0]=(pBuf->repeate_en_present<<6)|(pBuf->repeatersourcehdmi_present<<5)|(pBuf->hdcp14repeater_present<<4)|(pBuf->user_hpd_present<<3)|(pBuf->repeaterReset_present<<2)|(pBuf->repeaterV_present<<1)|pBuf->ddcport_present;

    if((pBuf->repeaterV_present)&&(!pBuf->repeaterReset_present)&&(!pBuf->ddcport_present)&&(!pBuf->user_hpd_present)&&(!pBuf->hdcp14repeater_present)&&(!pBuf->repeatersourcehdmi_present)&&(!pBuf->repeate_en_present))
        Vpos_idx=1;
    else
        Vpos_idx=2;

    if(pBuf->ddcport_present)
        pData[1]=(pBuf->enddc_port<<2)|pBuf->ddc_port|pData[1];

    if(pBuf->repeaterReset_present)
        pData[1]=(pBuf->enRepeaterReset<<3)|pData[1];

    if(pBuf->user_hpd_present)
        pData[1]=(pBuf->enUserHPD<<4)|pData[1];

    if(pBuf->hdcp14repeater_present)
        pData[1]=(pBuf->enHDCP14Repeater<<5)|pData[1];

    if(pBuf->repeatersourcehdmi_present)
        pData[1]=(pBuf->enSourceHDMI<<6)|pData[1];

    if(pBuf->repeate_en_present)
        pData[1]=(pBuf->enOpenRepeater<<7)|pData[1];

    if(pBuf->repeaterV_present)
    {
        for(i=0;i<(pBuf->data_len - Vpos_idx);i++)
            pData[Vpos_idx+i]=pBuf->repeaterV_val[i];
    }

    return Mailbox_WriteCommand(pBuf->command_idx,pBuf->data_len,pData);

}


MS_BOOL Mailbox_ReadCommand_Report_4K2KVic (ST_MAILBOX_COMMAND_REPORT_EDID_INFO_4K2KVIC *pBuf)
{
    MS_U8 pData[10];
    int i=0;
    memset(pData, 0, sizeof(MS_U8)*10);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->command_idx!=E_MAILBOX_REPORT_EDID_INFO)
        {
            HDMITX4VX1_KDBG(" Wrong Command Type in %s !!!\n",__func__);
            return FALSE;
        }
        if(pData[0]!=0x01)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Query Type!!!\n",__func__);
            return FALSE;
        }
        pBuf->query_type=(EN_HDMITX4VX1_QUERY_TYPE)pData[0];
        pBuf->vic_num=pData[1]&0x0F;
        for(i=0;i<(pBuf->data_len - 2);i++)
        {
            switch(pData[i+2])
            {
                    case E_HDMITX4VX1_VIDEO_4K2K_Reserved:
                    case E_HDMITX4VX1_VIDEO_4K2K_30Hz:
                    case E_HDMITX4VX1_VIDEO_4K2K_25Hz:
                    case E_HDMITX4VX1_VIDEO_4K2K_24Hz:
                    case E_HDMITX4VX1_VIDEO_4K2K_24Hz_SMPTE:
                        pBuf->vic4k2k[i]=(EN_HDMITX4VX1_VIDEO_4K2K_VIC)pData[i+2];
                        break;
                    default:
                        HDMITX4VX1_KDBG(" %s Fail !!! Wrong Type of 4K2KVic !!!\n",__func__);
                        pBuf->vic4k2k[i]=E_HDMITX4VX1_VIDEO_4K2K_Reserved;
            }
        }
        return TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}


MS_BOOL Mailbox_ReadCommand_Report_3D_Structure(ST_MAILBOX_COMMAND_REPORT_EDID_INFO_3DSTRUCT *pBuf)
{
    MS_U8 data_len=0x04;
    MS_U8 pData[4];
    MS_U16 Data3D=(0);
    memset(pData, 0, sizeof(MS_U8)*4);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->data_len!=data_len)
        {
            HDMITX4VX1_KDBG(" %s Fail !!! Wrong Data Length!!! Length should be 4!!!\n",__func__);
            return FALSE;
        }
        if(pData[0]==0x02)
        {
            pBuf->query_type=(EN_HDMITX4VX1_QUERY_TYPE)pData[0];
            switch(pData[1])
            {
                    case E_HDMITX4VX1_QUERY_3DTIMING_1280x720p_50Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_1280x720p_60Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_1920x1080i_50Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_1920x1080i_60Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_1920x1080p_24Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_1920x1080p_25Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_1920x1080p_50Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_1920x1080p_60Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_4K2Kp_25Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_4K2Kp_30Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_4K2Kp_50Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_4K2Kp_60Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_4096x2160p_25Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_4096x2160p_30Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_4096x2160p_50Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_4096x2160p_60Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_1920x1080p_30Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_4K2Kp_24Hz:
                    case E_HDMITX4VX1_QUERY_3DTIMING_4096x2160p_24Hz:
                        pBuf->repTiming_3d=(EN_HDMITX4VX1_QUERY_3DTIMING)pData[1];
                        break;
                    default:
                        HDMITX4VX1_KDBG(" %s Fail !!! Wrong Type of 3D Timing !!!\n",__func__);
                        return FALSE;
            }

            Data3D = (pData[3]<<8) | pData[2];
            pBuf->edid_3d=(EN_HDMITX4VX1_VIDEO_REPORT_3D_STRUCTURE_TYPE)Data3D;
            HDMITX4VX1_KDBG(" Data3D Struct = %x !!!\n",(int)pBuf->edid_3d);

            return TRUE;
        }
        else
        {
            HDMITX4VX1_KDBG(" %s Fail !!! Wrong Query Type!!!\n",__func__);
            return FALSE;
        }
    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}

MS_BOOL Mailbox_ReadCommand_Report_Num_AudioDescriptor(ST_MAILBOX_COMMAND_REPORT_EDID_INFO_AUDIODES_NUM *pBuf)
{
    MS_U8 data_len=0x02;
    MS_U8 pData[2];
    memset(pData, 0, sizeof(MS_U8)*2);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->command_idx!=E_MAILBOX_REPORT_EDID_INFO)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Command Type!!!\n",__func__);
            return FALSE;
        }
        if(pBuf->data_len!=data_len)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Data Length!!!\n",__func__);
            return FALSE;
        }
        if(pData[0]!=0x03)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Query Type!!!\n",__func__);
            return FALSE;
        }
        pBuf->query_type=(EN_HDMITX4VX1_QUERY_TYPE)pData[0];
        pBuf->auddes_num=pData[1];
        return TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}

MS_BOOL Mailbox_ReadCommand_Report_AudioDescriptor (ST_MAILBOX_COMMAND_REPORT_EDID_INFO_AUDIODES *pBuf)
{
    MS_U8 pData[68];
    int i=0;
    memset(pData, 0, sizeof(MS_U8)*68);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->command_idx!=E_MAILBOX_REPORT_EDID_INFO)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Command Type!!!\n",__func__);
            return FALSE;
        }
        if(pData[0]!=0x04)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Query Type!!!\n",__func__);
            return FALSE;
        }
        pBuf->query_type=(EN_HDMITX4VX1_QUERY_TYPE)pData[0];
        pBuf->audstr_Idx=pData[1];
        pBuf->audend_Idx=pData[2];
        for(i=0;i<(pBuf->data_len - 3);i++)
        {
            pBuf->aud_des[i]=pData[i+3];
        }
        return TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}

MS_BOOL Mailbox_ReadCommand_Report_Num_VideoDescriptor(ST_MAILBOX_COMMAND_REPORT_EDID_INFO_VIDEODES_NUM *pBuf)
{
    MS_U8 data_len=0x02;
    MS_U8 pData[2];
    memset(pData, 0, sizeof(MS_U8)*2);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->command_idx!=E_MAILBOX_REPORT_EDID_INFO)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Command Type!!!\n",__func__);
            return FALSE;
        }
        if(pBuf->data_len!=data_len)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Data Length!!!\n",__func__);
            return FALSE;
        }
        if(pData[0]!=0x05)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Query Type!!!\n",__func__);
            return FALSE;
        }
        pBuf->query_type=(EN_HDMITX4VX1_QUERY_TYPE)pData[0];
        pBuf->viddes_num=pData[1];
        return TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}

MS_BOOL Mailbox_ReadCommand_Report_VideoDescriptor (ST_MAILBOX_COMMAND_REPORT_EDID_INFO_VIDEODES *pBuf)
{
    MS_U8 pData[68];
    int i=0;
    memset(pData, 0, sizeof(MS_U8)*68);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->command_idx!=E_MAILBOX_REPORT_EDID_INFO)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Command Type!!!\n",__func__);
            return FALSE;
        }
        if(pData[0]!=0x06)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Query Type!!!\n",__func__);
            return FALSE;
        }
        pBuf->query_type=(EN_HDMITX4VX1_QUERY_TYPE)pData[0];
        pBuf->vidstr_Idx=pData[1];
        pBuf->vidend_Idx=pData[2];
        for(i=0;i<(pBuf->data_len - 3);i++)
        {
            pBuf->vid_des[i]=pData[i+3];
        }
        return TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}

MS_BOOL Mailbox_ReadCommand_Report_EDID_RawData(ST_MAILBOX_COMMAND_REPORT_EDID_INFO_EDIDRAWDATA *pBuf)
{
    MS_U8 pData[68];
    int i=0;
    memset(pData, 0, sizeof(MS_U8)*68);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->command_idx!=E_MAILBOX_REPORT_EDID_INFO)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Command Type!!!\n",__func__);
            return FALSE;
        }
        if(pData[0]!=0x07)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Query Type!!!\n",__func__);
            return FALSE;
        }
        pBuf->query_type=(EN_HDMITX4VX1_QUERY_TYPE)pData[0];
        pBuf->Idx_edid=pData[1]&0x0F;
        pBuf->Idx_64bytes=pData[1]&0xF0;
        for(i=0;i<(pBuf->data_len - 2);i++)
        {
            pBuf->raw_data[i]=pData[i+2];
        }
        return TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}

MS_BOOL Mailbox_ReadCommand_Report_RxEdidInfo (ST_MAILBOX_COMMAND_REPORT_EDID_INFO_RX_EDID_INFO *pBuf)
{

    MS_U8 pData[15];
    memset(pData, 0, sizeof(MS_U8)*2);
    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->command_idx!=E_MAILBOX_REPORT_EDID_INFO)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Command Type!!!\n",__func__);
            return FALSE;
        }
        if(pData[0]!=E_HDMITX4VX1_QUERY_RX_EDID_INFO)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Query Type!!!\n",__func__);
            return FALSE;
        }
        pBuf->query_type=(EN_HDMITX4VX1_QUERY_TYPE)pData[0];
        pBuf->bSupportHdmi=(MS_BOOL)pData[1];
        pBuf->enColorDepth=(EN_HDMITX4VX1_COLOR_DEPTH_INFO)pData[2];
        pBuf->u8ManufacturerID[0]=pData[3];
        pBuf->u8ManufacturerID[1]=pData[4];
        pBuf->u8ManufacturerID[2]=pData[5];
        pBuf->u8Colorimetry=pData[6];
        pBuf->u8ManufacturerProductCode[0]=pData[7];
        pBuf->u8ManufacturerProductCode[1]=pData[8];
        pBuf->u8SerialNum[0]=pData[9];
        pBuf->u8SerialNum[1]=pData[10];
        pBuf->u8SerialNum[2]=pData[11];
        pBuf->u8SerialNum[3]=pData[12];
        pBuf->u8PhyAddr0=pData[13];
        pBuf->u8PhyAddr1=pData[14];
        return TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}

MS_BOOL Mailbox_ReadCommand_Report_ColorFormat(ST_MAILBOX_COMMAND_REPORT_EDID_INFO_COLORFMT *pBuf)
{
    MS_U8 pData[3];
    memset(pData, 0, sizeof(MS_U8)*3);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->command_idx!=E_MAILBOX_REPORT_EDID_INFO)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Command Type!!!\n",__func__);
            return FALSE;
        }
        if(pData[0]!=0x09)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Query Type!!!\n",__func__);
            return FALSE;
        }
        pBuf->query_type=(EN_HDMITX4VX1_QUERY_TYPE)pData[0];
        pBuf->colfmt_timing=(EN_HDMITX4VX1_QUERY_COLORFMTTIMING)pData[1];
        pBuf->supColorFmt=(MS_U8)pData[2];

        return TRUE;

    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}

MS_BOOL Mailbox_ReadCommand_Report_HWInfo(ST_MAILBOX_COMMAND_REPORT_EDID_INFO_HWINFO *pBuf)
{
    MS_U8 pData[3];
    memset(pData, 0, sizeof(MS_U8)*3);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->command_idx!=E_MAILBOX_REPORT_EDID_INFO)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Command Type!!!\n",__func__);
            return FALSE;
        }
        if(pData[0]!=0x0A)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Query Type!!!\n",__func__);
            return FALSE;
        }
        pBuf->query_type=(EN_HDMITX4VX1_QUERY_TYPE)pData[0];
        pBuf->timingID    = (EN_HDMITX4VX1_VIDEO_TIMING) pData[1];
        pBuf->color_depth = (EN_HDMITX4VX1_COLOR_DEPTH) (pData[2]&0x07);
        pBuf->output_mode = (EN_HDMITX4VX1_VIDEO_OUTPUT_TYPE) ((pData[2]&0x18)>>3);
        pBuf->in_color    = (EN_HDMITX4VX1_INPUT_COLOR_TYPE)  ((pData[2]&0x20)>>5);
        pBuf->out_color   = (EN_HDMITX4VX1_OUTPUT_COLOR_TYPE) ((pData[2]&0xC0)>>6);

        return TRUE;

    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}


MS_BOOL Mailbox_ReadCommand_Report_KSV_BStatus(ST_MAILBOX_COMMAND_REPORT_EDID_INFO_KSV_BSTATUS *pBuf)
{
    MS_U8 pData[67];
    int i=0;
    memset(pData, 0, sizeof(MS_U8)*67);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->command_idx!=E_MAILBOX_REPORT_EDID_INFO)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Command Type!!!\n",__func__);
            return FALSE;
        }
        if(pData[0]!=0x0B)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Query Type!!!\n",__func__);
            return FALSE;
        }
        pBuf->query_type=(EN_HDMITX4VX1_QUERY_TYPE)pData[0];
        pBuf->device_num=(MS_U8)pData[1];
        pBuf->KSVB_Idx=(MS_U8)pData[2];

        if(pBuf->device_num == INVALID_DEVIE_NUM)
        {
            HDMITX4VX1_KDBG(" %s KSV List invaild!!! device_num=%d \n",__func__, pBuf->device_num);
            return FALSE;
        }
        //HDMITX4VX1_DBG_MORE(HDMITX4VX1_KDBG(" KSV_Data:---\n"));
        for(i=0;i<(pBuf->data_len - 3);i++)
        {
            pBuf->KSVB_data[i]=(MS_U8)pData[i+3];
            //HDMITX4VX1_DBG_MORE(HDMITX4VX1_KDBG("%x ",pBuf->KSVB_data[i]));
        }
        //HDMITX4VX1_DBG_MORE(HDMITX4VX1_KDBG("----------------------------\n"));
        return TRUE;

    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}

MS_BOOL Mailbox_ReadCommand_Report_HDCPKey_Status(ST_MAILBOX_COMMAND_REPORT_EDID_INFO_HDCPKEY_STATUS *pBuf)
{
    MS_U8 data_len=0x02;
    MS_U8 pData[2];
    memset(pData, 0, sizeof(MS_U8)*2);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->command_idx!=E_MAILBOX_REPORT_EDID_INFO)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Command Type!!!\n",__func__);
            return FALSE;
        }
        if(pBuf->data_len!=data_len)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Data Length!!!\n",__func__);
            return FALSE;
        }
        if(pData[0]!=0x0C)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Query Type!!!\n",__func__);
            return FALSE;
        }
        pBuf->query_type=(EN_HDMITX4VX1_QUERY_TYPE)pData[0];
        pBuf->HDCP14Key_Status=(EN_HDMITX4VX1_HDCPKEY_STATUS_TYPE)(pData[1]&0x3);
        pBuf->HDCP22Key_Status=(EN_HDMITX4VX1_HDCPKEY_STATUS_TYPE)((pData[1]>>2)&0x3);
        return TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}


MS_BOOL Mailbox_ReadCommand_ACK_Status(ST_MAILBOX_COMMAND_ACK_STATUS_RESPONSE *pBuf)
{
    MS_U8 data_len=0x02;
    MS_U8 pData[2];
    memset(pData, 0, sizeof(MS_U8)*2);

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    if(Mailbox_ReadCommand(&pBuf->command_idx,&pBuf->data_len,pData))
    {
        if(pBuf->data_len!=data_len)
        {
            HDMITX4VX1_KDBG(" %s Fail!!! Wrong Data Length!!!\n",__func__);
            return FALSE;
        }
        if(pBuf->command_idx==E_MAILBOX_ACK_STATUS_RESPOND)
        {
            switch(pData[0])
            {
                    case E_MAILBOX_REPORT_CONFIQ_VIDEO:
                    case E_MAILBOX_REPORT_CONFIQ_AUDIO:
                    case E_MAILBOX_REPORT_AVMUTE_TIMING_RESET:
                    case E_MAILBOX_REPORT_REPORT_EDID_INFO:
                        pBuf->mbx_reccom=(EN_HDMITX4VX1_MAILBOX_RECIEVED_COMMAND_TYPE)pData[0];
                        break;
                    default:
                        HDMITX4VX1_KDBG(" %s Receive Wrong Command Type !!!\n",__func__);
                        return FALSE;
            }

            switch(pData[1])
            {
                    case E_MAILBOX_REPORT_SUCCESS:
                    case E_MAILBOX_REPORT_FAIL:
                    case E_MAILBOX_REPORT_PARAMETER_INVALID:
                        pBuf->mbx_status=(EN_HDMITX4VX1_MAILBOX_STATUS_RESPONSE_TYPE)pData[1];
                        break;
                    default:
                        HDMITX4VX1_KDBG(" %s Receive Wrong Status Type !!!\n",__func__);
                        return FALSE;
            }
            return TRUE;
        }
        else
        {
            HDMITX4VX1_KDBG(" %s Fail !!! Wrong Command Type!!!\n",__func__);
            return FALSE;
        }
    }
    else
    {
        HDMITX4VX1_KDBG(" %s Read Fail !!!\n",__func__);
        return FALSE;
    }
}

MS_BOOL EnterSerialDebugMode(MS_BOOL bEnable)
{
    MS_U8 u8DataBuf[5] = {0x53, 0x45, 0x52, 0x44, 0x42};
    MS_BOOL bResult = TRUE;
    MS_U8 u8cnt = 0;

    HDMITX4VX1_KDBG(" EnterSerialDebugMode:%d ++++++++++++++++++++++++++\n", bEnable);
    if (bEnable)
    {
        u8DataBuf[0] = 0x53;

        while(HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 5, u8DataBuf) == FALSE)
        {
            u8cnt++;
            bResult = FALSE;
            //HDMITX4VX1_KDBG("FAIL %d\n", u8cnt);
            if(u8cnt == 0x20)
                break;
        }
    }
    else
    {
        u8DataBuf[0] = 0x45;

        while(HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 5, u8DataBuf) == FALSE)
        {
            bResult = FALSE;
            if(u8cnt == 0x20)
                break;
        }
    }
    return bResult;

}

MS_BOOL IIC_Config(void)
{
    MS_U8 u8DataBuf = 0;
    MS_BOOL bResult = TRUE;

    HDMITX4VX1_KDBG(" IIC_Config ++++++++++++++++++++++++++++\n");

    // change to CH3
    u8DataBuf = 0x81;
    if(HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf) == FALSE)
    {
        HDMITX4VX1_KDBG(" Ch3  0x81 fail\n");
        bResult &= HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf);
    }

    u8DataBuf = 0x83;
    if(HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf) == FALSE)
    {
        HDMITX4VX1_KDBG(" Ch3  0x83 fail\n");
        bResult &= HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf);
    }

    u8DataBuf = 0x84;
    if(HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf) == FALSE)
    {
        HDMITX4VX1_KDBG(" Ch3  0x84 fail\n");
        bResult &= HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf);
    }

    // address 2 byte
    u8DataBuf = 0x51;
    if(HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf) == FALSE)
    {
        HDMITX4VX1_KDBG(" Ch3  0x51 fail\n");
        bResult &= HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf);
    }
    // config
    u8DataBuf = 0x7F;
    if(HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf) == FALSE)
    {
        HDMITX4VX1_KDBG(" Ch3  0x7F fail\n");
        bResult &= HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf);
    }

    return bResult;

}

MS_BOOL IICUseBus(MS_BOOL bEnable)
{

    MS_U8 u8DataBuf = 0;
    MS_BOOL bResult = FALSE;

    if(bEnable)
        u8DataBuf = 0x35;
    else
        u8DataBuf = 0x34;

    HDMITX4VX1_KDBG(" IICUseBus:%d ++++++++++++++++++++++++++++\n", bEnable);

    if(HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf) == FALSE)
    {
        HDMITX4VX1_KDBG(" IICUseBus read write fail\n");
        bResult = HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf);
    }

    // shape IIC Plus
    u8DataBuf = 0x71;
    if(HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf) == FALSE)
    {
        HDMITX4VX1_KDBG(" IICUseBus shap IIC fail\n");
        bResult = HDMITX4VX1_IIC_WriteRegister(HDMITX4VX1_I2C_SLAVE_ID, 0, NULL, 1, &u8DataBuf);
    }

    return bResult;

}

void HDMITX4VX1_IIC_Init(void)
{
    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);
    IICUseBus(FALSE);
    EnterSerialDebugMode(FALSE);

    EnterSerialDebugMode(TRUE);
    IIC_Config();
    IICUseBus(TRUE);
}

MS_BOOL Fn_WaitReport(void)
{
    int loopnum=50;
    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    while((HDMITX4VX1_r2byte(0x1200) & 0xFF)!=0x05)
    {
        loopnum--;
        mdelay(100);
        if(loopnum<0)
        {
            HDMITX4VX1_KDBG(" Wait MBX Report Fail! Time out!\n");
            return FALSE;
        }
    }

    return TRUE;
}


MS_BOOL Fn_WaitStatus(void)
{
    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);
    HDMITX4VX1_KDBG(" %s Received Command(%x)\n",__func__,(int)(HDMITX4VX1_r2byte(0x1202) & 0xFF));
    HDMITX4VX1_KDBG(" Status(%x)\n",(int)(HDMITX4VX1_r2byte(0x1203) & 0xFF));
    return TRUE;
}


void HDMITX4VX1_SPI_Init(EN_HDMITX4VX1_LOCKN_TYPE enLOCKNType)
{
    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);
    HDMITX4VX1_w2byte(0x1304, 0x0001);
    HDMITX4VX1_w2byte(0x1E40, 0x0000);
    HDMITX4VX1_w2byte(0x1E42, 0x0000);
    HDMITX4VX1_w2byte(0x1E44, 0x0000);
    HDMITX4VX1_w2byte(0x1E46, 0x0000);
    HDMITX4VX1_w2byte(0x1E48, 0x0000);
    HDMITX4VX1_w2byte(0x1E4A, 0x0000);
    HDMITX4VX1_w2byte(0x1E4C, 0x0000);
    HDMITX4VX1_w2byte(0x1E4E, 0x0000);
    HDMITX4VX1_w2byte(0x1E60, 0x0000);
    HDMITX4VX1_w2byte(0x1E62, 0x0000);
    HDMITX4VX1_w2byte(0x1E64, 0x0000);
    HDMITX4VX1_w2byte(0x1E66, 0x0000);
    HDMITX4VX1_w2byte(0x1E68, 0x0000);
    HDMITX4VX1_w2byte(0x1E74, 0x0000);
    HDMITX4VX1_w2byte(0x1E76, 0x0020);
    HDMITX4VX1_w2byte(0x1E78, 0x0000);
    HDMITX4VX1_w2byte(0x1E7A, 0x0000);
    HDMITX4VX1_w2byte(0x0B00, 0x0000); //all_pad_in = 0

    if(enLOCKNType==E_HDMITX4VX1_HW_LOCKN)//hw lockn
    {
        HDMITX4VX1_w2byte(0x0B80, 0x1000); //[10] reg_spi_mode = 1 [12] reg_uart_mode1 = 1
    }
    else//sw lockn
    {
        HDMITX4VX1_w2byte(0x0B80, 0x2000); //[10] reg_spi_mode = 1 [12] reg_uart_mode1 = 1
    }


    HDMITX4VX1_w2byte(0x000FE4, 0x0001);

    HDMITX4VX1_w2byte(0x000EC0, 0x000B); //reg_fsp_wd1[7:0], reg_fsp_wd0[7:0]
    HDMITX4VX1_w2byte(0x000EC2, 0x0000); //reg_fsp_wd3[7:0], reg_fsp_wd2[7:0]
    HDMITX4VX1_w2byte(0x000EC4, 0x0000); //reg_fsp_wd5[7:0], reg_fsp_wd4[7:0]
    HDMITX4VX1_w2byte(0x000EC6, 0x0000); //reg_fsp_wd7[7:0], reg_fsp_wd6[7:0]
    HDMITX4VX1_w2byte(0x000EC8, 0x0000); //reg_fsp_wd9[7:0], reg_fsp_wd8[7:0]
    HDMITX4VX1_w2byte(0x000ED4, 0x0005); //reg_fsp_wbf_size2[3:0], reg_fsp_wbf_size1[3:0], reg_fsp_wbf_size0[3:0]
    HDMITX4VX1_w2byte(0x000ED6, 0x0001); //reg_fsp_rbf_size2[3:0], reg_fsp_rbf_size1[3:0], reg_fsp_rbf_size0[3:0]

    HDMITX4VX1_w2byte(0x000ED8, 0x0007); //reg_fsp_ctrl1[7:0], reg_fsp_ctrl0[7:0]

    HDMITX4VX1_w2byte(0x000EDA, 0x0001); //reg_fsp_trigger_le_w
    udelay(10*1000);
    HDMITX4VX1_w2byte(0x000EDE, 0x0001); //reg_fsp_clear_done_flag_le_w

    HDMITX4VX1_w2byte(0x001E72, 0x0800);  //reg_sw_spi_clk, reg_ckg_spi[3:0]
    HDMITX4VX1_w2byte(0x001E72, 0x1800);  //reg_sw_spi_clk, reg_ckg_spi[3:0]


    HDMITX4VX1_w2byte(0x1E20, 0x0003);
    HDMITX4VX1_w2byte(0x1C80, 0x0001);

    udelay(1*1000);
    HDMITX4VX1_w2byte(0x1C80, 0x000F);
}

MS_BOOL HDMITX4VX1_checkChipID(void)
{
    MS_U8 chipID = 0;

    //read ChipID
    HDMITX4VX1_ReadBytes(0x1E00,&chipID);
    HDMITX4VX1_KDBG(" show ChipID 1E 00 =%x\n",chipID);
    if(chipID!=0x97)
    {
        HDMITX4VX1_KDBG(" Read Chip ID FAIL!!!\n");
        return FALSE;
    }
    return TRUE;
}

MS_BOOL checkCpuIsRuning(void)
{
    MS_U16 chipCPU = 0;
    int num_looptime=50;
    //MS_U32 bFirstTimer = 0;
    //MS_U32 bTimer = 0;
    //read HDMITX4VX1 CPU Status
    chipCPU=HDMITX4VX1_r2byte(HDMITX4VX1_CHIP_STATE);
    //bFirstTimer=MsOS_GetSystemTime();
    while(chipCPU != HDMITX4VX1_STATE_IS_READY)
    {
        num_looptime--;
        HDMITX4VX1_KDBG(" Show ChipCPU Status 1E FE =%x\n",chipCPU);
        //bTimer=MsOS_GetSystemTime();
        if(num_looptime<0)
        {
            HDMITX4VX1_KDBG(" Init Fail! CPU not ready! Time out!\n");
            return FALSE;
        }
        mdelay(100);
        chipCPU=HDMITX4VX1_r2byte(HDMITX4VX1_CHIP_STATE);
    }
    chipCPU=HDMITX4VX1_r2byte(HDMITX4VX1_CHIP_STATE);
    HDMITX4VX1_KDBG(" Show ChipCPU Status 1E FE =%x\n",chipCPU);
    return TRUE;
}

void Fn_Send_Interrupt_HDMITX4VX1(void)
{
    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);
    HDMITX4VX1_w2byte(0x2B0C, 0x0000);
    HDMITX4VX1_w2byte(0x2B0C, 0x0001);
}

MS_BOOL Fn_ReadACK(void)
{
    MS_BOOL pBuf=FALSE;

    //read bank
    if((( HDMITX4VX1_r2byte(0x1250) &0x2)>>1))
    {
        pBuf=TRUE;
    }
    else
    {
        pBuf=FALSE;
    }

    if(pBuf)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void Fn_SetACK_Low(void)
{
    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);
    HDMITX4VX1_w2byte(0x1250,0x0000);

    HDMITX4VX1_w2byte(0x0A02, (HDMITX4VX1_r2byte(0x0A02) & 0xFFDF));
    HDMITX4VX1_w2byte(0x0A0A, (HDMITX4VX1_r2byte(0x0A0A) | 0x0020));
}

MS_BOOL Fn_WaitACK(void)
{
    //read Raptors ACK
    int loopnum=1000;

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    while(!Fn_ReadACK())
    {
        loopnum--;
        mdelay(5);
        if(loopnum<0)
        {
            HDMITX4VX1_KDBG(" Read MBX ACK Fail! Time out!\n");
            return FALSE;
        }
    }

    return TRUE;
}

void HDMITX4VX1_GPIO_Init(void)
{
    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);
    HDMITX4VX1_WriteBytesMask(0x0B00,0xFF,0x00);
    HDMITX4VX1_WriteBytesMask(0x0B82,0xFF,0x00);
    HDMITX4VX1_WriteBytesMask(0x0B17,0xFF,0x03);
}

MS_U8 haltx_timing_get_uart_mode(void)
{
/*
    MS_U8 u8regval = 0;
    MS_U8 u8returnval = 0;

    RAPTORS_DBG_MORE(printf("[Device Raptors] %s, %d\n",__func__,__LINE__));

    Device_Hdmitx_IIC_ReadBytes(REG_PAD_TOP_40_H, &u8regval);
    u8returnval = (u8regval& 0x70) >> 4;
    RAPTORS_DBG_MORE(printf("[Device Raptors] uart mode=%d, %s, %d\n",u8returnval,__func__,__LINE__));*/
    return E_TX_TIMING_UART_MODE_2;//u8returnval;
}

void haltx_timing_set_vby1_sw_lockn(MS_BOOL ben)
{
    HDMITX4VX1_WriteBytesMask(REG_PAD_TOP_0D_H, BIT2, ben ? BIT2 : 0);
}

void haltx_timing_set_vby1_pw_on_off(MS_BOOL ben)
{
    HDMITX4VX1_w2byte(REG_VBY1_RX_01_L, ben ? 0xFFFF : 0x0000);
}

void haltx_timing_set_lpll_on_off(MS_BOOL ben)
{
    //RAPTORS_DBG_MORE(printf("[Device Raptors] ben:%d\n",ben));
    HDMITX4VX1_WriteBytesMask(REG_LPLL_0C_L, 0x08, ben ? 0x08 : 0x00);
}

void haltx_timing_set_vby1_fifo_dis(MS_BOOL ben)
{
    HDMITX4VX1_WriteBytesMask(REG_VBY1_RX_1D_L, BIT6|BIT4|BIT2|BIT0, ben ? BIT6|BIT4|BIT2|BIT0 : 0);
}

void haltx_timing_set_vby1_lane_timeout(MS_BOOL ben)
{
    HDMITX4VX1_WriteBytesMask(REG_VBY1_RX_A_24_H, BIT7, ben ? BIT7: 0);
}

MS_BOOL haltx_timing_check_vby1_lock_staus(MS_U8 u8pattern)
{
    #define VX1_LOCK_4_SUCCESS  0x10
    #define VX1_LOCK_3_SUCCESS  0x01
    #define VX1_LOCK_4_3_FAIL   0x22
    #define VX1_LOCK_CDR_STATE  0x03
    #define VX1_LOCK_DATA_STATE 0x04

    static MS_BOOL bPreStatus = VX1_LOCK_4_3_FAIL;
    MS_BOOL bret = 1;
    MS_U16 u16state3210, u16state7654;
    MS_U16 u16mask3210, u16mask7654;
    MS_U16 u16lane_sel3210, u16lane_sel7654;
    MS_U16 u16reg3210, u16reg7654;
    MS_BOOL bDbgPrint = 0;

    u16lane_sel3210 = HDMITX4VX1_r2byte(REG_VBY1_RX_15_L);
    u16lane_sel7654 = HDMITX4VX1_r2byte(REG_VBY1_RX_16_L);

    if(u16lane_sel3210 == 0x6240 &&  u16lane_sel7654 == 0x7351)
    {  // 8lanes
        u16state3210 = u8pattern == VX1_LOCK_CDR_STATE ? 0x3333 : 0x4444;
        u16state7654 = u8pattern == VX1_LOCK_CDR_STATE ? 0x3333 : 0x4444;
        u16mask3210 = 0xFFFF;
        u16mask7654 = 0xFFFF;
    }
    else if(u16lane_sel3210 == 0x0604 &&  u16lane_sel7654 == 0x0705)
    { // 4 lanes
        u16state3210 = 0x0000;
        u16state7654 = u8pattern == VX1_LOCK_CDR_STATE ? 0x3333 : 0x4444;
        u16mask3210 = 0x0000;
        u16mask7654 = 0xFFFF;
    }
    else if(u16lane_sel3210 == 0x0004 &&  u16lane_sel7654 == 0x0005)
    { // 2 lane
        u16state3210 = 0x0000;
        u16state7654 = u8pattern == VX1_LOCK_CDR_STATE ? 0x0033 : 0x0044;
        u16mask3210 = 0x0000;
        u16mask7654 = 0x00FF;
    }
    else
    {// 1 lane
        u16state3210 = 0x0000;
        u16state7654 = u8pattern == VX1_LOCK_CDR_STATE ? 0x0003 : 0x0004;
        u16mask3210 = 0x0000;
        u16mask7654 = 0x000F;
    }

    u16reg3210 = HDMITX4VX1_r2byte(REG_VBY1_RX_A_20_L);
    u16reg7654 = HDMITX4VX1_r2byte(REG_VBY1_RX_A_21_L);

    bret &= ((u16reg3210 & u16mask3210) == u16state3210) ? 1 : 0;
    bret &= ((u16reg7654 & u16mask7654) == u16state7654) ? 1 : 0;


    if(((u16reg3210 & u16mask3210) != u16state3210)  ||
       ((u16reg7654 & u16mask7654) != u16state7654))
    {
        bDbgPrint = (u8pattern == VX1_LOCK_CDR_STATE) ? (bPreStatus & VX1_LOCK_3_SUCCESS) : (bPreStatus & VX1_LOCK_4_SUCCESS);
        bPreStatus = (u8pattern == VX1_LOCK_CDR_STATE) ? bPreStatus & ~VX1_LOCK_3_SUCCESS : (bPreStatus & ~VX1_LOCK_4_SUCCESS);
    }
    else
    {
        if(u8pattern == VX1_LOCK_CDR_STATE)
        {
            bDbgPrint = (bPreStatus & VX1_LOCK_3_SUCCESS) ? 0 : 1;
        }
        else
        {
            bDbgPrint = (bPreStatus & VX1_LOCK_4_SUCCESS) ? 0 : 1;
        }
        bPreStatus = (u8pattern == VX1_LOCK_CDR_STATE) ?
                     (bPreStatus & ~VX1_LOCK_3_SUCCESS) | VX1_LOCK_3_SUCCESS :
                     (bPreStatus & ~VX1_LOCK_4_SUCCESS) | VX1_LOCK_4_SUCCESS;
    }
    if(bDbgPrint)
    {
        HDMITX4VX1_KDBG(" M(%04x%04x)R(%04x%04x)P(%04x%04x)\n",
            u16mask3210, u16mask7654, (u16reg3210), (u16reg7654), u16state3210, u16state7654);
    }

    return bret;
}


MS_BOOL haltx_timing_process_vby1_sw_lockn(MS_U32 u32timeout)
{
    //MS_U32 u32time;
    MS_BOOL bRet = FALSE;
    MS_U32 loop_delaytime=1;
    MS_U32 loop_num=0;

    loop_num=u32timeout;

    // pull high LOCKN
    haltx_timing_set_vby1_sw_lockn(TRUE);

    // ALN timeout sw enable
    haltx_timing_set_vby1_lane_timeout(TRUE);

    // trun off vby1
    haltx_timing_set_vby1_pw_on_off(FALSE);

    // turn on vby1
    haltx_timing_set_vby1_pw_on_off(TRUE);

    // delay 10ms
    mdelay(10);

    // check 333
    //u32time = MsOS_GetSystemTime();

    while(1)
    {
        loop_num--;

        if( haltx_timing_check_vby1_lock_staus(3))
        {
            bRet = TRUE;
            break;
        }

        if(loop_num<0)
        {
            HDMITX4VX1_KDBG(" %s(),%d Error, Check vby1 3333 Fail\n",__func__,__LINE__);
            break;
        }
        mdelay(loop_delaytime);
    }

    if(bRet)
    {
        // pull LOCKN to low
        haltx_timing_set_vby1_sw_lockn(FALSE);
    }

    // ALN timeout sw disable
    haltx_timing_set_vby1_lane_timeout(FALSE);

    return bRet;
}

void haltx_timing_set_vx1_fifo_rest_by_vsync(void)
{
    MS_BOOL bVsCnt = 0;
    MS_U8   u8BK27_1D;
    //MS_U32 u32Time;
    MS_U32 loop_num=500;
    HDMITX4VX1_ReadBytes(REG_VBY1_RX_1D_L, &u8BK27_1D);
    HDMITX4VX1_WriteBytesMask(REG_VBY1_RX_1D_L, BIT2|BIT0, 0);
    HDMITX4VX1_WriteBytesMask(REG_SC_TOP_09_L, BIT3, BIT3); // clear vs
    //u32Time = MsOS_GetSystemTime();
    while(1)
    {
        loop_num--;

        HDMITX4VX1_WriteBytesMask(REG_SC_TOP_09_L, BIT3, 0);
        if( HDMITX4VX1_r2byte(REG_SC_TOP_0A_L) & BIT3) // get vs flag
        {
            bVsCnt++;
            if(bVsCnt == 2)
            {
                HDMITX4VX1_WriteBytesMask(REG_VBY1_RX_1C_L, BIT2, BIT2);
            }
            if(bVsCnt == 3)
            {
                HDMITX4VX1_WriteBytesMask(REG_VBY1_RX_1C_L, BIT2, 0);
                break;
            }
            HDMITX4VX1_WriteBytesMask(REG_SC_TOP_09_L, BIT3, BIT3); // clear vs
        }

        if(loop_num<0)
        {
            break;
        }
        mdelay(1);
    }
    HDMITX4VX1_WriteBytesMask(REG_VBY1_RX_1D_L, BIT2|BIT0, u8BK27_1D);
}

void haltx_timing_load_scdc(MS_BOOL bEn)
{

    HDMITX4VX1_WriteBytesMask(REG_PAD_TOP_00_L, 0xFF, 0x00);
    HDMITX4VX1_WriteBytesMask(REG_PAD_TOP_40_L, 0x80, 0x80);
    HDMITX4VX1_WriteBytesMask(REG_PAD_TOP_10_L, 0x80, 0x80);
    HDMITX4VX1_WriteBytesMask(REG_HDMITX_MISC_01_H, 0xC0, 0xC0);

    // master i2c init
    HDMITX4VX1_WriteBytesMask(0x1010, 0xFF, 0x78);
    HDMITX4VX1_WriteBytesMask(0x1012, 0xFF, 0x78);
    HDMITX4VX1_WriteBytesMask(0x1014, 0xFF, 0x78);
    HDMITX4VX1_WriteBytesMask(0x1016, 0xFF, 0x10);
    HDMITX4VX1_WriteBytesMask(0x1018, 0xFF, 0x78);
    HDMITX4VX1_WriteBytesMask(0x101A, 0xFF, 0x02);

    HDMITX4VX1_WriteBytesMask(0x1000, 0xFF, 0x60);

    // Slave 0xA8 , address 0x20, value 0x03 to enable scdc

    //start
    HDMITX4VX1_WriteBytes(0x1002, 0x01);
    HDMITX4VX1_WriteBytes(0x1008, 0x01);

    //write ID: 0xA8
    HDMITX4VX1_WriteBytes(0x1004, 0xA8);
    HDMITX4VX1_WriteBytes(0x1008, 0x01);

    // write address: 0x20
    HDMITX4VX1_WriteBytes(0x1004, 0x20);
    HDMITX4VX1_WriteBytes(0x1008, 0x01);

    // write data: 0x03
    HDMITX4VX1_WriteBytes(0x1004, bEn ? 0x03 : 0x00 );
    HDMITX4VX1_WriteBytes(0x1008, 0x01);

     //stop
    HDMITX4VX1_WriteBytes(0x1003, 0x01);
    HDMITX4VX1_WriteBytes(0x1008, 0x01);
}

void haltx_timing_load_register(MS_U8 *pdata, MS_U16 u16regnum, MS_U16 u16datasize, MS_U8 u8dataoffset)
{
    MS_U16 i, j;
    MS_U32 u32addr;
    MS_U8 u8mask, u8value;

    for(i=0; i< u16regnum; i++)
    {
        j = i *  (REG_ADDR_SIZE+REG_MASK_SIZE+u16datasize);

        u32addr = (((MS_U32)pdata[j ]) << 8)  | pdata[j +1];
        u8mask = pdata[j + 2];
        u8value = pdata[j + 3 + u8dataoffset];

        HDMITX4VX1_WriteBytesMask(u32addr, u8mask, u8value);

        //RAPTORS_DBG_MORE(printf("[Device Raptors] addr:%04x, msk=%02x, val=%02x\n", u32addr, u8mask, u8value));
    }

}


MS_U8 hal_HDMITx_CalcAVIInfoFrameCheckSum(void)
{
    MS_U8 ucSumVal = 0;
    MS_U8 u8TempRegVal = 0;
    MS_U8 i = 0;

    ucSumVal += (E_HDMITX_AVI_INFOFRAME + HDMITX_AVI_INFO_PKT_VER + HDMITX_AVI_INFO_PKT_LEN);

    for (i=0;i<HDMITX_AVI_INFO_PKT_LEN;i++)
    {
        if(i==3)
        {
            HDMITX4VX1_ReadBytes(REG_HDMITX_09_L+i,&u8TempRegVal);
            ucSumVal += ( u8TempRegVal & 0x7F); //mask out reserved bit
        }
        else if (i==4)
        {
            HDMITX4VX1_ReadBytes(REG_HDMITX_09_L+i,&u8TempRegVal);
            ucSumVal += (u8TempRegVal & 0x3F); //mask out reserved bit
        }
        else
        {
            HDMITX4VX1_ReadBytes(REG_HDMITX_09_L+i,&u8TempRegVal);
            ucSumVal += u8TempRegVal;
        }
    }

    return (MS_U8)((~ucSumVal) + 0x01);
}


//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
void HDMITX4VX1_Probe(void)
{
    MS_BOOL ret = FALSE;

    //if i2c not work, do i2c init at probe
    ret = HDMITX4VX1_checkChipID();
    if(ret == FALSE)
    {
        HDMITX4VX1_IIC_Init();
    }
}


MS_BOOL HDMITX4VX1_Init(EN_HDMITX4VX1_LOCKN_TYPE enLOCKNType)
{
    MS_BOOL ret = FALSE;
    //MS_U16 testval=0;

    //if i2c not work, do i2c init
    ret = HDMITX4VX1_checkChipID();
    if(ret == FALSE)
    {
        HDMITX4VX1_IIC_Init();
    }

    //check i2c is work or not
    ret = HDMITX4VX1_checkChipID();
    if(ret == FALSE)
    {
        HDMITX4VX1_KDBG(" Read Chip ID FAIL!!!\n");
        return ret;
    }

    //sw reset, add for test
    //icebox test
    //HDMITX4VX1_KDBG(" do sw reset\n");
    //HDMITX4VX1_w2byte(0x1F60,0x0079);
    //HDMITX4VX1_KDBG(" IIC init \n");
    //HDMITX4VX1_IIC_Init();
    //HDMITX4VX1_KDBG(" icebox test IIC value \n");
    //HDMITX4VX1_w2byte(0x1200,0x3412);
    //testval=HDMITX4VX1_r2byte(0x1200);
    //HDMITX4VX1_KDBG(" icebox test testval =0x%x\n",testval);

    //if CPU is not ready, do SPI init
    if(HDMITX4VX1_r2byte(HDMITX4VX1_CHIP_STATE) != HDMITX4VX1_STATE_IS_READY)
    {
        // test for force do SPI init
        //HDMITX4VX1_KDBG(" do sw reset\n");
        //HDMITX4VX1_w2byte(0x1F60,0x0079);
        //HDMITX4VX1_IIC_Init();

        HDMITX4VX1_SPI_Init(enLOCKNType);
    }

    ret = checkCpuIsRuning();
    if(ret == FALSE)
        {
            HDMITX4VX1_KDBG(" Init Fail! CPU not ready! Time out!\n");
        return ret;
    }

    return TRUE;
}

MS_BOOL HDMITX4VX1_Video_config(ST_MAILBOX_COMMAND_CONFIQ_VIDEO stVideo)
{
    Mailbox_WriteCommand_Config_Video(&stVideo);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }
    Fn_WaitStatus();
    Fn_SetACK_Low();//set ACK low after read it high
    return TRUE;
}

MS_BOOL HDMITX4VX1_Audio_config(ST_MAILBOX_COMMAND_CONFIQ_AUDIO stAudio)
{
    Mailbox_WriteCommand_Config_Audio(&stAudio);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }
    Fn_WaitStatus();
    Fn_SetACK_Low();//set ACK low after read it high
    return TRUE;
}

MS_BOOL HDMITX4VX1_TimingChange_AVmute(ST_MAILBOX_COMMAND_TIMING_CHANGE_AVMUTE stChange)
{
    Mailbox_WriteCommand_TimingChange_AVmute(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }
    Fn_WaitStatus();
    Fn_SetACK_Low();//set ACK low after read it high
    return TRUE;
}

MS_BOOL HDMITX4VX1_QUERY_4K2KVIC(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_4K2KVIC *get4k2k)
{
    MS_BOOL bRet = FALSE;
    Mailbox_WriteCommand_Query_EDID(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }

    Fn_SetACK_Low();//set ACK low after read it high

    if(Fn_WaitReport())
    {
        Mailbox_ReadCommand_Report_4K2KVic(get4k2k);
        bRet = TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s, %d Fail!!\n",__func__,__LINE__);
        return FALSE;
    }
        return bRet;
}


MS_BOOL HDMITX4VX1_QUERY_3D_Structure(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_3DSTRUCT *get3D)
{
    MS_BOOL bRet = FALSE;
    Mailbox_WriteCommand_Query_EDID(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }

    Fn_SetACK_Low();//set ACK low after read it high

    if(Fn_WaitReport())
    {
        Mailbox_ReadCommand_Report_3D_Structure(get3D);
        bRet = TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s, %d Fail!!\n",__func__,__LINE__);
        return FALSE;
    }
        return bRet;
}

MS_BOOL HDMITX4VX1_QUERY_Num_AudioDescriptor(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_AUDIODES_NUM *getNUM)
{
    MS_BOOL bRet = FALSE;
    Mailbox_WriteCommand_Query_EDID(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }

    Fn_SetACK_Low();//set ACK low after read it high

    if(Fn_WaitReport())
    {
        Mailbox_ReadCommand_Report_Num_AudioDescriptor(getNUM);
        bRet = TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s, %d Fail!!\n",__func__,__LINE__);
        return FALSE;
    }
        return bRet;
}

MS_BOOL HDMITX4VX1_QUERY_AudioDescriptor(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_AUDIODES *getAudio)
{
    MS_BOOL bRet = FALSE;
    Mailbox_WriteCommand_Query_EDID(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }

    Fn_SetACK_Low();//set ACK low after read it high

    if(Fn_WaitReport())
    {
        Mailbox_ReadCommand_Report_AudioDescriptor(getAudio);
        bRet = TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s, %d Fail!!\n",__func__,__LINE__);
        return FALSE;
    }
        return bRet;
}

MS_BOOL HDMITX4VX1_QUERY_Num_VideoDescriptor(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_VIDEODES_NUM *getNUM)
{
    MS_BOOL bRet = FALSE;
    Mailbox_WriteCommand_Query_EDID(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }

    Fn_SetACK_Low();//set ACK low after read it high

    if(Fn_WaitReport())
    {
        Mailbox_ReadCommand_Report_Num_VideoDescriptor(getNUM);
        bRet = TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s, %d Fail!!\n",__func__,__LINE__);
        return FALSE;
    }
        return bRet;
}

MS_BOOL HDMITX4VX1_QUERY_VideoDescriptor(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_VIDEODES *getVideo)
{
    MS_BOOL bRet = FALSE;
    Mailbox_WriteCommand_Query_EDID(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }

    Fn_SetACK_Low();//set ACK low after read it high

    if(Fn_WaitReport())
    {
        Mailbox_ReadCommand_Report_VideoDescriptor(getVideo);
        bRet = TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s, %d Fail!!\n",__func__,__LINE__);
        return FALSE;
    }
        return bRet;
}

MS_BOOL HDMITX4VX1_QUERY_EDID_RawData(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_EDIDRAWDATA *getRaw)
{
    MS_BOOL bRet = FALSE;
    Mailbox_WriteCommand_Query_EDID(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }

    Fn_SetACK_Low();//set ACK low after read it high

    if(Fn_WaitReport())
    {
        Mailbox_ReadCommand_Report_EDID_RawData(getRaw);
        bRet = TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s, %d Fail!!\n",__func__,__LINE__);
        return FALSE;
    }
        return bRet;
}

MS_BOOL HDMITX4VX1_QUERY_RxEdidInfo(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_RX_EDID_INFO *getEDID)
{
    MS_BOOL bRet = FALSE;
    Mailbox_WriteCommand_Query_EDID(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }

    Fn_SetACK_Low();//set ACK low after read it high

    if(Fn_WaitReport())
    {
        Mailbox_ReadCommand_Report_RxEdidInfo(getEDID);
        bRet = TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s, %d Fail!!\n",__func__,__LINE__);
        return FALSE;
    }
        return bRet;
}

MS_BOOL HDMITX4VX1_QUERY_ColorFormat(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_COLORFMT *getcolor)
{
    MS_BOOL bRet = FALSE;
    Mailbox_WriteCommand_Query_EDID(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }

    Fn_SetACK_Low();//set ACK low after read it high

    if(Fn_WaitReport())
    {
        Mailbox_ReadCommand_Report_ColorFormat(getcolor);
        bRet = TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s, %d Fail!!\n",__func__,__LINE__);
        return FALSE;
    }
        return bRet;
}

MS_BOOL HDMITX4VX1_QUERY_HWInfo(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_HWINFO *getHW)
{
    MS_BOOL bRet = FALSE;
    Mailbox_WriteCommand_Query_EDID(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }

    Fn_SetACK_Low();//set ACK low after read it high

    if(Fn_WaitReport())
    {
        Mailbox_ReadCommand_Report_HWInfo(getHW);
        bRet = TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s, %d Fail!!\n",__func__,__LINE__);
        return FALSE;
    }
        return bRet;
}

MS_BOOL HDMITX4VX1_QUERY_KSV_BStatus(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_KSV_BSTATUS *getKSVB)
{
    MS_BOOL bRet = FALSE;
    Mailbox_WriteCommand_Query_EDID(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }

    Fn_SetACK_Low();//set ACK low after read it high

    if(Fn_WaitReport())
    {
        Mailbox_ReadCommand_Report_KSV_BStatus(getKSVB);
        bRet = TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s, %d Fail!!\n",__func__,__LINE__);
        return FALSE;
    }
        return bRet;
}

MS_BOOL HDMITX4VX1_QUERY_HDCPKey_Status(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_HDCPKEY_STATUS *getKSVB)
{
    MS_BOOL bRet = FALSE;
    Mailbox_WriteCommand_Query_EDID(&stChange);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }

    Fn_SetACK_Low();//set ACK low after read it high

    if(Fn_WaitReport())
    {
        Mailbox_ReadCommand_Report_HDCPKey_Status(getKSVB);
        bRet = TRUE;
    }
    else
    {
        HDMITX4VX1_KDBG(" %s, %d Fail!!\n",__func__,__LINE__);
        return FALSE;
    }
        return bRet;
}

MS_BOOL HDMITX4VX1_HDCP_CMD(ST_MAILBOX_COMMAND_HDCP_COMD stHDCP)
{
    Mailbox_WriteCommand_HDCP_COMD(&stHDCP);

    Fn_Send_Interrupt_HDMITX4VX1();

    if(!Fn_WaitACK())//read Raptors ACK
    {
        HDMITX4VX1_KDBG(" %s, %d , Setting Wrong!!! Read ACK Fail!!!\n",__func__,__LINE__);
        return FALSE;
    }
    Fn_WaitStatus();
    Fn_SetACK_Low();//set ACK low after read it high
    return TRUE;
}

int HDMITX4VX1_GetRxStatus(void)
{
    MS_U32 u8regval=0;

    HDMITX4VX1_KDBG(" %s, %d\n",__func__,__LINE__);

    u8regval=(MS_U32)( HDMITX4VX1_r2byte(HDMITX4VX1_STATUS) & 0x1D );

    return u8regval;
}

void HDMITX4VX1_loadKeepOutputScript(EN_HDMITX4VX1_VIDEO_TIMING loadtiming,
EN_HDMITX4VX1_COLOR_DEPTH loadcolordepth,EN_HDMITX4VX1_OUTPUT_COLOR_TYPE loadoutcolorfmt)
{
    MS_U8 u8tx_timing_id = MS_TX_TIMING_ID_NUM;
    MS_U8 u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
    MS_U8 u8tx_lpll_id = MS_TX_LPLL_ID_NUM;
    MS_U8 u8tx_vx1_id = MS_TX_VX1_ID_NUM;
    MS_U8 u8tx_timing_3x3matrix_id = MS_TX_3X3_MATRIX_ID_NUM;
    MS_U16 i;
    MS_U8 ucChkSum = 0;
    MS_BOOL bWrite =0;
    //MS_U32 u32FirstTimer = 0;
    //MS_U32 u32TimerCountTimeout = 0;
    int loopnum=100;
    EN_HDMITX4VX1_VIDEO_TIMING entimingid=loadtiming;//genHDMITx_VideoTiming;
    EN_HDMITX4VX1_COLOR_DEPTH encolordepth=loadcolordepth;//genHDMITx_ColorDepth;
    //E_TX_TIMING_LOAD_TYPE enloadtype;
    EN_HDMITX4VX1_OUTPUT_COLOR_TYPE encolorfmt=loadoutcolorfmt;//genHDMITx_OutColor;
    E_TX_TIMING_3X3_MATRIX_TYPE en3x3matrix=E_TX_TIMING_3X3_MATRIX_OFF;

    MS_TX_3X3_MATRIX_INFO *ptx3x3matrixinfo = NULL;
    MS_TX_TIMING_INFO *ptxtiminginfo = NULL;
    MS_TX_ATOP_INFO   *ptxatopinfo = NULL;
    MS_TX_LPLL_INFO   *ptxlpllinfo = NULL;
    MS_TX_VX1_INFO    *ptxvx1info = NULL;

    if(encolordepth==E_HDMITX4VX1_COLORS_NOID)
    {
        encolordepth=E_HDMITX4VX1_COLORS_8bits;
    }

    if((genHDMITx_InColor==E_HDMITX4VX1_COLORS_INPUT_RGB)&&(genHDMITx_OutColor!=E_HDMITX4VX1_COLORS_OUTPUT_RGB_444))
    {
        en3x3matrix=E_TX_TIMING_3X3_MATRIX_R2Y_HD_FULL;
    }

    HDMITX4VX1_KDBG(" %s: %d!!!\n",__func__,__LINE__);

    HDMITX4VX1_GPIO_Init();

    switch(en3x3matrix)
    {
        case E_TX_TIMING_3X3_MATRIX_OFF:
            u8tx_timing_3x3matrix_id = MS_TX_3X3_MATRIX_ID_OFF;
            break;
        case E_TX_TIMING_3X3_MATRIX_R2Y_SD_LIMIT:
            u8tx_timing_3x3matrix_id = MS_TX_3X3_MATRIX_ID_R2Y_SD_Limit;
            break;
        case E_TX_TIMING_3X3_MATRIX_R2Y_SD_FULL:
            u8tx_timing_3x3matrix_id = MS_TX_3X3_MATRIX_ID_R2Y_SD_Full;
            break;
        case E_TX_TIMING_3X3_MATRIX_R2Y_HD_LIMIT:
            u8tx_timing_3x3matrix_id = MS_TX_3X3_MATRIX_ID_R2Y_HD_Limit;
            break;
        case E_TX_TIMING_3X3_MATRIX_R2Y_HD_FULL:
            u8tx_timing_3x3matrix_id = MS_TX_3X3_MATRIX_ID_R2Y_HD_Full;
            break;
        default:
            u8tx_timing_3x3matrix_id = MS_TX_3X3_MATRIX_ID_NUM;
            break;
        }

    switch(entimingid)
    {
        // new timing added in Raptors, open when need it
        /*
        case E_TX_TIMING_640x480_60P:
            u8tx_vx1_id = MS_TX_VX1_ID_1_LANE;
            u8tx_timing_id = MS_TX_TIMING_ID_640x480_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_480_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_640x480_60P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_10bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_10bits_480_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_10bits_640x480_60P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_12bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_12bits_480_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_12bits_640x480_60P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_16bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_16bits_480_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_16bits_640x480_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;
            */

        case E_HDMITX4VX1_TIMING_720x480p_60Hz:
        case E_HDMITX4VX1_TIMING_720x576p_50Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_1_LANE;
            u8tx_timing_id = entimingid == E_HDMITX4VX1_TIMING_720x480p_60Hz ? MS_TX_TIMING_ID_720x480_60P:
                                                                                        MS_TX_TIMING_ID_720x576_50P;

            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_480_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_480_60P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_10bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_10bits_480_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_10bits_480_60P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_12bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_12bits_480_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_12bits_480_60P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_16bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_16bits_480_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_16bits_480_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1280x720p_50Hz:
        case E_HDMITX4VX1_TIMING_1280x720p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id = entimingid == E_HDMITX4VX1_TIMING_1280x720p_60Hz ? MS_TX_TIMING_ID_1280x720_60P :
                                                                                         MS_TX_TIMING_ID_1280x720_50P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_720_60P;
              u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_720_60P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_10bits)
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_10bits_720_60P;
              u8tx_lpll_id     = MS_TX_LPLL_ID_10bits_720_60P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_12bits)
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_12bits_720_60P;
              u8tx_lpll_id     = MS_TX_LPLL_ID_12bits_720_60P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_16bits)
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_16bits_720_60P;
              u8tx_lpll_id     = MS_TX_LPLL_ID_16bits_720_60P;
            }
            else
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
              u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;


        case E_HDMITX4VX1_TIMING_1920x1080p_24Hz:
        // new timing added in HDMITX4VX1, open when need it
        //case E_HDMITX4VX1_TIMING_1920x1080p_30Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;

            u8tx_timing_id =  entimingid == E_HDMITX4VX1_TIMING_1920x1080p_24Hz ? MS_TX_TIMING_ID_1920x1080_24P :
                                                                                           MS_TX_TIMING_ID_1920x1080_30P ;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_720_60P;
              u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_720_60P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_10bits)
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_10bits_720_60P;
              u8tx_lpll_id     = MS_TX_LPLL_ID_10bits_720_60P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_12bits)
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_12bits_720_60P;
              u8tx_lpll_id     = MS_TX_LPLL_ID_12bits_720_60P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_16bits)
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_16bits_720_60P;
              u8tx_lpll_id     = MS_TX_LPLL_ID_16bits_720_60P;
            }
            else
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
              u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1920x1080i_50Hz:
        case E_HDMITX4VX1_TIMING_1920x1080i_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id = entimingid == E_HDMITX4VX1_TIMING_1920x1080i_60Hz ? MS_TX_TIMING_ID_1920x1080_60I :
                                                                                          MS_TX_TIMING_ID_1920x1080_50I;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_1080_60I;
              u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1080_60I;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_10bits)
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_10bits_1080_60I;
              u8tx_lpll_id     = MS_TX_LPLL_ID_10bits_1080_60I;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_12bits)
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_12bits_1080_60I;
              u8tx_lpll_id     = MS_TX_LPLL_ID_12bits_1080_60I;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_16bits)
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_16bits_1080_60I;
              u8tx_lpll_id     = MS_TX_LPLL_ID_16bits_1080_60I;
            }
            else
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
              u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1920x1080p_50Hz:
        case E_HDMITX4VX1_TIMING_1920x1080p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            if(encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420)
            {
                u8tx_timing_id = entimingid == E_HDMITX4VX1_TIMING_1920x1080p_60Hz ? MS_TX_TIMING_ID_1920x1080_420_60P :
                                                                                              MS_TX_TIMING_ID_1920x1080_420_50P;

                if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_720_60P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_720_60P;
                }
                else if(encolordepth == E_HDMITX4VX1_COLORS_10bits)
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_10bits_720_60P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_10bits_720_60P;
                }
                else if(encolordepth == E_HDMITX4VX1_COLORS_12bits)
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_12bits_720_60P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_12bits_720_60P;
                }
                else if(encolordepth == E_HDMITX4VX1_COLORS_16bits)
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_16bits_720_60P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_16bits_720_60P;
                }
                else
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
                }
            }
            else
            {
                u8tx_timing_id = entimingid == E_HDMITX4VX1_TIMING_1920x1080p_60Hz ? MS_TX_TIMING_ID_1920x1080_60P :
                                                                                              MS_TX_TIMING_ID_1920x1080_50P;

                u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
                if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_1080_60P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1080_60P;
                }
                else if(encolordepth == E_HDMITX4VX1_COLORS_10bits)
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_10bits_1080_60P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_10bits_1080_60P;
                }
                else if(encolordepth == E_HDMITX4VX1_COLORS_12bits)
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_12bits_1080_60P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_12bits_1080_60P;
                }
                else if(encolordepth == E_HDMITX4VX1_COLORS_16bits)
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_16bits_1080_60P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_16bits_1080_60P;
                }
                else
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
                }
            }
            break;

        case E_HDMITX4VX1_TIMING_4K2Kp_24Hz:
        case E_HDMITX4VX1_TIMING_4K2Kp_25Hz:
        case E_HDMITX4VX1_TIMING_4K2Kp_30Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_4_LANE;
            if(entimingid ==  E_HDMITX4VX1_TIMING_4K2Kp_24Hz)
            {
                u8tx_timing_id = (encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420) ? MS_TX_TIMING_ID_3840x2160_420_24P : MS_TX_TIMING_ID_3840x2160_24P;
            }
            else if(entimingid ==  E_HDMITX4VX1_TIMING_4K2Kp_25Hz)
            {
                u8tx_timing_id = (encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420) ? MS_TX_TIMING_ID_3840x2160_420_25P : MS_TX_TIMING_ID_3840x2160_25P;
            }
            else
            {
                u8tx_timing_id = (encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420) ? MS_TX_TIMING_ID_3840x2160_420_30P : MS_TX_TIMING_ID_3840x2160_30P;
            }


            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
              u8hdmitx_atop_id = (encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420) ? MS_TX_ATOP_ID_8bits_4K2K_420_30P : MS_TX_ATOP_ID_8bits_4K2K_3D_30P;
              u8tx_lpll_id     = (encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420) ? MS_TX_LPLL_ID_8bits_4K2K_420_30P : MS_TX_LPLL_ID_8bits_4K2K_3D_30P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_10bits)
            {
              u8hdmitx_atop_id = (encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420) ? MS_TX_ATOP_ID_10bits_4K2K_420_30P : MS_TX_ATOP_ID_10bits_4K2K_3D_30P;
              u8tx_lpll_id     = (encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420) ? MS_TX_LPLL_ID_10bits_4K2K_420_30P : MS_TX_LPLL_ID_10bits_4K2K_3D_30P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_12bits)
            {
              u8hdmitx_atop_id = (encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420) ? MS_TX_ATOP_ID_12bits_4K2K_420_30P : MS_TX_ATOP_ID_12bits_4K2K_3D_30P;
              u8tx_lpll_id     = (encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420) ? MS_TX_LPLL_ID_12bits_4K2K_3D_30P  : MS_TX_LPLL_ID_12bits_4K2K_3D_30P;
            }
            else if(encolordepth == E_HDMITX4VX1_COLORS_16bits)
            {
              u8hdmitx_atop_id = (encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420) ? MS_TX_ATOP_ID_16bits_4K2K_420_30P : MS_TX_ATOP_ID_16bits_4K2K_3D_30P;
              u8tx_lpll_id     = (encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420) ? MS_TX_LPLL_ID_16bits_4K2K_420_30P : MS_TX_LPLL_ID_16bits_4K2K_3D_30P;
            }
            else
            {
              u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
              u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }

            break;

        case E_HDMITX4VX1_TIMING_4K2Kp_50Hz:
        case E_HDMITX4VX1_TIMING_4K2Kp_60Hz:
        case E_HDMITX4VX1_TIMING_4096x2160_50Hz:
        case E_HDMITX4VX1_TIMING_4096x2160_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_8_LANE;
            if(encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420)
            {
                u8tx_timing_id = entimingid == E_HDMITX4VX1_TIMING_4K2Kp_50Hz     ? MS_TX_TIMING_ID_3840x2160_420_50P :
                                 entimingid == E_HDMITX4VX1_TIMING_4K2Kp_60Hz     ? MS_TX_TIMING_ID_3840x2160_420_60P :
                                 entimingid == E_HDMITX4VX1_TIMING_4096x2160_50Hz ? MS_TX_TIMING_ID_4096x2160_420_50P :
                                                                                             MS_TX_TIMING_ID_4096x2160_420_60P;

                if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
                {
                    u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_4K2K_420_60P;
                    u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_4K2K_420_60P;
                }
                else if(encolordepth == E_HDMITX4VX1_COLORS_10bits)
                {
                    u8hdmitx_atop_id = MS_TX_ATOP_ID_10bits_4K2K_420_60P;
                    u8tx_lpll_id     = MS_TX_LPLL_ID_10bits_4K2K_420_60P;
                }
                else if(encolordepth == E_HDMITX4VX1_COLORS_12bits)
                {
                    u8hdmitx_atop_id = MS_TX_ATOP_ID_12bits_4K2K_420_60P;
                    u8tx_lpll_id     = MS_TX_LPLL_ID_12bits_4K2K_420_60P;
                }
                else if(encolordepth == E_HDMITX4VX1_COLORS_16bits)
                {
                    u8hdmitx_atop_id = MS_TX_ATOP_ID_16bits_4K2K_420_60P;
                    u8tx_lpll_id     = MS_TX_LPLL_ID_16bits_4K2K_420_60P;
                }
                else
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
                }
            }
            else
            {
                u8tx_timing_id = entimingid == E_HDMITX4VX1_TIMING_4K2Kp_50Hz     ? MS_TX_TIMING_ID_3840x2160_50P :
                                 entimingid == E_HDMITX4VX1_TIMING_4K2Kp_60Hz     ? MS_TX_TIMING_ID_3840x2160_60P :
                                 entimingid == E_HDMITX4VX1_TIMING_4096x2160_50Hz ? MS_TX_TIMING_ID_4096x2160_50P :
                                                                                             MS_TX_TIMING_ID_4096x2160_60P ;
                if(encolordepth == E_HDMITX4VX1_COLORS_8bits ||
                  (encolordepth == E_HDMITX4VX1_COLORS_10bits && encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_422))
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_4K2K_60P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_4K2K_60P;
                }
                else
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
                }
            }

            break;

        case E_HDMITX4VX1_TIMING_4096x2160p_24Hz:
        case E_HDMITX4VX1_TIMING_4096x2160p_25Hz:
        case E_HDMITX4VX1_TIMING_4096x2160p_30Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_4_LANE;
            if(encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420)
            {
                u8tx_timing_id   = MS_TX_TIMING_ID_NUM;
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            else
            {
                u8tx_timing_id = entimingid == E_HDMITX4VX1_TIMING_4096x2160p_24Hz ? MS_TX_TIMING_ID_4096x2160_24P :
                                 entimingid == E_HDMITX4VX1_TIMING_4096x2160p_25Hz ? MS_TX_TIMING_ID_4096x2160_25P :
                                                                                              MS_TX_TIMING_ID_4096x2160_30P;

                if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_4K2K_3D_30P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_4K2K_3D_30P;
                }
                else if(encolordepth == E_HDMITX4VX1_COLORS_10bits)
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_10bits_4K2K_3D_30P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_10bits_4K2K_3D_30P;
                }
                else if(encolordepth == E_HDMITX4VX1_COLORS_12bits)
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_12bits_4K2K_3D_30P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_12bits_4K2K_3D_30P;
                }
                else if(encolordepth == E_HDMITX4VX1_COLORS_16bits)
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_16bits_4K2K_3D_30P;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_16bits_4K2K_3D_30P;
                }
                else
                {
                  u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                  u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
                }
            }
            break;

        case E_HDMITX4VX1_TIMING_800x600p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_800x600_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_480_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_800x600_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_848x480p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_848x480_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_480_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_848x480_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1024x768p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1024x768_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_720_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1024x768_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1280x768p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1280x768_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_720_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1280x768_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1280x800p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1280x800_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_720_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1280x800_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1280x960p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1280x960_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_1080_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1280x960_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1280x1024p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1280x1024_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_1080_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1280x960_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1360x768p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1360x768_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_1080_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1360x768_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1366x768p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1366x768_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_1080_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1360x768_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1400x1050p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1400x1050_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_1080_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1400x1050_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1440x900p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1440x900_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_1080_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1440x900_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1600x900p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_2_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1600x900_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_1080_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1600x900_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1600x1200p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_4_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1600x1200_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_1080_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1600x1200_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1680x1050p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_4_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1680x1050_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_1080_60P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1680x1050_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_1920x1200p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_4_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_1920x1200_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_4K2K_3D_30P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_1920x1200_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        case E_HDMITX4VX1_TIMING_2048x1152p_60Hz:
            u8tx_vx1_id = MS_TX_VX1_ID_4_LANE;
            u8tx_timing_id   = MS_TX_TIMING_ID_2048x1152_60P;
            if(encolordepth == E_HDMITX4VX1_COLORS_8bits)
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_8bits_4K2K_3D_30P;
                u8tx_lpll_id     = MS_TX_LPLL_ID_8bits_2048x1152_60P;
            }
            else
            {
                u8hdmitx_atop_id = MS_TX_ATOP_ID_NUM;
                u8tx_lpll_id     = MS_TX_LPLL_ID_NUM;
            }
            break;

        default:
            HDMITX4VX1_KDBG(" NG ID:%d, %s, %d\n",entimingid,__func__,__LINE__);
        }


    if(u8tx_timing_id   == MS_TX_TIMING_ID_NUM || u8tx_timing_id   == 0xFF ||
           u8hdmitx_atop_id == MS_TX_ATOP_ID_NUM   || u8hdmitx_atop_id == 0xFF ||
           u8tx_lpll_id     == MS_TX_LPLL_ID_NUM   || u8tx_lpll_id     == 0xFF ||
           u8tx_vx1_id      == MS_TX_VX1_ID_NUM    || u8tx_vx1_id      == 0xFF)
    {
        HDMITX4VX1_KDBG(" NG T:%d,A:%d,L:%d,V:%d\n", u8tx_timing_id, u8hdmitx_atop_id,u8tx_lpll_id, u8tx_vx1_id);
    }
    else
    {

        if( haltx_timing_get_uart_mode() == E_TX_TIMING_UART_MODE_2)
        {
            haltx_timing_set_vby1_sw_lockn(TRUE); // pull high LOCKN
        }

        haltx_timing_set_vby1_pw_on_off(FALSE);
        haltx_timing_set_lpll_on_off(FALSE);

        HDMITX4VX1_KDBG(" TIME=(%d),ATOP=(%d),LPLL=(%d),Vx1=(%d),3x3=(%d)\n",
            u8tx_timing_id,u8hdmitx_atop_id,u8tx_lpll_id,u8tx_vx1_id, u8tx_timing_3x3matrix_id);

        // Load Tx Vx1
        for(i=0; i<MS_TX_VX1_TAB_NUM; i++)
        {
            ptxvx1info = &stHALTX_TIMING_VX1_TBL[i];

            bWrite = TRUE;

            //HDMITX4VX1_DBG_MORE(printf("[Device HDMITX4VX1] TX_Vx1: TAB:%d, write:%d, loadtype=load_all\n", i, bWrite));
            if(bWrite)
            {
                if(ptxvx1info->enIPType == MS_TX_VX1_IP_COMMON)
                {
                    haltx_timing_load_register(ptxvx1info->pData, ptxvx1info->u16RegNum, REG_DATA_SIZE, 0);
                }
                else
                {
                    haltx_timing_load_register(ptxvx1info->pData, ptxvx1info->u16RegNum, MS_TX_VX1_ID_NUM, u8tx_vx1_id);
                }
            }
        }


        // Load TX_ATOP
        for(i=0; i<MS_TX_ATOP_TAB_NUM; i++)
        {
            ptxatopinfo = &stHALTX_TIMING_ATOP_TBL[i];

            bWrite = TRUE;
            //HDMITX4VX1_DBG_MORE(printf(" TX_ATOP: TAB:%d, write:%d, loadtype=load_all\n", i, bWrite));
            if(bWrite)
            {
                if(ptxatopinfo->enIPType == MS_TX_ATOP_IP_COMMON)
                {
                    haltx_timing_load_register(ptxatopinfo->pData, ptxatopinfo->u16RegNum, REG_DATA_SIZE, 0);
                }
                else
                {
                    haltx_timing_load_register(ptxatopinfo->pData, ptxatopinfo->u16RegNum, MS_TX_ATOP_ID_NUM, u8hdmitx_atop_id);
                }
            }
        }

        // Load TX_LPLL
        for(i=0; i<MS_TX_LPLL_TAB_NUM; i++)
        {
            ptxlpllinfo = &stHALTX_TIMING_LPLL_TBL[i];

            bWrite = TRUE;
            //HDMITX4VX1_DBG_MORE(printf(" TX_LPLL: TAB:%d, write:%d, loadtype=load_all\n", i, bWrite));
            if(bWrite)
            {
                if(ptxlpllinfo->enIPType == MS_TX_LPLL_IP_COMMON)
                {
                    haltx_timing_load_register(ptxlpllinfo->pData, ptxlpllinfo->u16RegNum, REG_DATA_SIZE, 0);
                }
                else
                {
                    haltx_timing_load_register(ptxlpllinfo->pData, ptxlpllinfo->u16RegNum, MS_TX_LPLL_ID_NUM, u8tx_lpll_id);
                }
            }
        }

        // Load TX_TIMING
        for(i=0; i<MS_TX_TIMING_TAB_NUM; i++)
        {
            ptxtiminginfo = &stHALTX_TIMING_TBL[i];

            bWrite = TRUE;
            //HDMITX4VX1_DBG_MORE(printf(" TX_TIMING: TAB:%d, write:%d, loadtype=load_all\n", i, bWrite));

            if(bWrite)
            {
                if(ptxtiminginfo->enIPType == MS_TX_TIMING_IP_COMMON)
                {
                    haltx_timing_load_register(ptxtiminginfo->pData, ptxtiminginfo->u16RegNum, REG_DATA_SIZE, 0);
                }
                else
                {
                    haltx_timing_load_register(ptxtiminginfo->pData, ptxtiminginfo->u16RegNum, MS_TX_TIMING_ID_NUM, u8tx_timing_id);
                }
            }
        }

        if(encolorfmt == E_HDMITX4VX1_COLORS_OUTPUT_YUV_420)
        {
            HDMITX4VX1_WriteBytesMask(REG_SC_TOP_0F_L, 0x30, 0x10);
        }
        else if(en3x3matrix == E_TX_TIMING_3X3_MATRIX_OFF) //reg_sc_in_sel: 1: 420
        {
            HDMITX4VX1_WriteBytesMask(REG_SC_TOP_0F_L, 0x30, 0x00); //reg_sc_in_sel: 0: 444
        }
        else
        {
            HDMITX4VX1_WriteBytesMask(REG_SC_TOP_0F_L, 0x30, 0x20); //reg_sc_in_sel: 2 444 csc in
        }

        // set 3x3 matrix

        for(i=0; i<MS_TX_3X3_MATRIX_TAB_NUM; i++)
        {
            ptx3x3matrixinfo = &stHALTX_TIMING_3X3MATRIX_TBL[i];

            bWrite = TRUE;
            if(bWrite)
            {
                if(ptx3x3matrixinfo->enIPType == MS_TX_3X3_MATRIX_IP_COMMON)
                {
                    haltx_timing_load_register(ptx3x3matrixinfo->pData, ptx3x3matrixinfo->u16RegNum, REG_DATA_SIZE, 0);
                }
                else
                {
                    haltx_timing_load_register(ptx3x3matrixinfo->pData, ptx3x3matrixinfo->u16RegNum, MS_TX_3X3_MATRIX_ID_NUM, u8tx_timing_3x3matrix_id);
                }
            }
        }

        //rewrite avi output info settings
        //Y2, Y1, Y0: RGB, YCbCr 422, 444, 420
        HDMITX4VX1_WriteBytesMask(REG_HDMITX_09_L,0xE0,(genHDMITx_OutColor<< 5));
        //A0 field =0
        HDMITX4VX1_WriteBytesMask(REG_HDMITX_09_L,0x18,((0 & 0x03) << 4));
        //S1, S0 field =0
        HDMITX4VX1_WriteBytesMask(REG_HDMITX_09_L,0x0F,0);

        //C1, C0, M1, M0
        HDMITX4VX1_WriteBytesMask(REG_HDMITX_09_H,0x3F,0x28);

        //C0, C1 =>  default colormetry=0
        HDMITX4VX1_WriteBytesMask(REG_HDMITX_09_H,0xC0,0x00);

        //Q0, Q1 => default enRgbQuantization=2, E_HDMITX_VIDEO_RGB_QUANTIZATION_FULL
        HDMITX4VX1_WriteBytesMask(REG_HDMITX_0A_L,0x0C,(0x02<<2));

        //ITC => default = 0
        HDMITX4VX1_WriteBytesMask(REG_HDMITX_0A_L,0x80,0);

        //IT content => default=2, E_HDMITX_IT_CONTENT_CINEMA
        HDMITX4VX1_WriteBytesMask(REG_HDMITX_0B_L,0x30,(0x02<<4));

        //AVI version
        HDMITX4VX1_WriteBytesMask(REG_HDMITX_0F_H, 0x03, HDMITX_AVI_INFO_PKT_VER);

        //rewrite avi packet check sum
        ucChkSum = hal_HDMITx_CalcAVIInfoFrameCheckSum();

        HDMITX4VX1_KDBG(" AVI check sum=%x\n",ucChkSum);
        HDMITX4VX1_w2byte(REG_HDMITX_10_L, 0x01 | (ucChkSum << 8));
        HDMITX4VX1_WriteBytes(REG_HDMITX_01_L, 0x00);
        HDMITX4VX1_WriteBytesMask(REG_HDMITX_01_L, 0x04, 0x04);

        //haltx_timing_set_vx1_fifo_rst_flag(TRUE);
        haltx_timing_set_vby1_fifo_dis(TRUE);

        if( haltx_timing_get_uart_mode() == E_TX_TIMING_UART_MODE_2)
        {
            if(haltx_timing_process_vby1_sw_lockn(VX1_LOCKN_STATE3_TIMEOUT))
            {
                HDMITX4VX1_KDBG(" Vx1LKN3 OK\n");
            }
            else
            {
                HDMITX4VX1_KDBG(" Vx1LKN3 NG\n");
            }
        }
        else
        {
            haltx_timing_set_vby1_pw_on_off(TRUE);
        }

            haltx_timing_set_lpll_on_off(TRUE);

        //fifo reset
        haltx_timing_set_vx1_fifo_rest_by_vsync();

        //load scdc setting
        if((entimingid == E_HDMITX4VX1_TIMING_4K2Kp_50Hz)||(entimingid == E_HDMITX4VX1_TIMING_4K2Kp_60Hz))
        {
            haltx_timing_load_scdc(TRUE);
        }
        else
        {
            haltx_timing_load_scdc(FALSE);
        }

        //vby1 check status 4444
        //u32FirstTimer=MsOS_GetSystemTime();
        while(!haltx_timing_check_vby1_lock_staus(4))
        {
            loopnum--;
            haltx_timing_process_vby1_sw_lockn(VX1_LOCKN_STATE3_TIMEOUT);

            mdelay(100);
            //u32TimerCountTimeout=MsOS_GetSystemTime();

            if(loopnum < 0)
            {
                HDMITX4VX1_KDBG(" Vby lock fail! Time out!, %s, %d\n",__func__,__LINE__);
                break;
            }

        }

    }

}

void HDMITX4VX1_ReadRegisterForce(int iSize, ST_HDMITX4VX1_REGISTER *stBuf)
{
    int i=0;
    for(i=0;i<iSize;i++)
    {
        HDMITX4VX1_ReadBytes(stBuf[i].reg_addr,&stBuf[i].reg_val);
    }
}

void HDMITX4VX1_WriteRegisterMaskForce(int iSize, ST_HDMITX4VX1_REGISTER *stBuf)
{
    int i=0;
    //MS_U8 tempval=0;
    for(i=0;i<iSize;i++)
    {
        HDMITX4VX1_WriteBytesMask(stBuf[i].reg_addr,stBuf[i].mask,stBuf[i].reg_val);

        //test write in val
        //HDMITX4VX1_ReadBytes(stBuf[i].reg_addr,&tempval);
        //printk("[HDMITX4VX1] icebox test write addr=0x%x before_val=%x after_val=%x\n",stBuf[i].reg_addr,stBuf[i].reg_val,tempval);
    }
}
