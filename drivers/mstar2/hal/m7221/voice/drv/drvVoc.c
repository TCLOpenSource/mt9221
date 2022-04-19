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

#include "mosWrapper.h"

#include "drvMBX.h"
#include "drvBDMA.h"
#include "drvVoc.h"
#include "halCHIPTOP.h"
#include "halBDMA.h"
#include <linux/slab.h>
#include <asm/uaccess.h>
#include "halMBX.h"
#include <linux/vmalloc.h>

//#include "ms_types.h"
#define VOC_MSG(fmt, args...) MOS_DBG_PRINT(MOS_DBG_LEVEL_VOC, "[VOC]" fmt, ##args)
#define VOC_ERROR(fmt, args...) MOS_DBG_ERROR("[VOC ERROR]" fmt, ##args);

//#define CM4_PERIOD (10*16*2*2*2)
#define FULL_THRES 3 //CM4_PERIOD
#define MAX_CM4_PERIOD (1ULL<<32)

#define REG_PM_UART_PAD           GET_REG16_ADDR(REG_ADDR_BASE_PM_SLEEP, 0x76)
#define CM4_BIN_SIZE 0x80000
#define CM4_PATH "/dev/block/platform/mstar_mci.0/by-name/cm4"

static U32 g_nCm4Period;
static U32 g_nLevelCount;
static U32 g_nLastPeriodNum;
static U32 g_nPeriodSize;
static U32 g_nBufferSize;
static U32 g_nStatus;
//static void __iomem *g_pImiBase;
//static void * g_pFlag;
static U32 gInit = 0;
static bool g_bDisabledCM4Flag = false;
static unsigned char u8CM4_FW_Binary[CM4_BIN_SIZE];

//static U8 u8CM4_FW_Binary[512*1024] = {
//    #include "fwCM4pack.dat"
//};


#define PCM_NORMAL 0
#define PCM_XRUN 1

void VocDmaInit(U32 nDmaBufSize, U32 nPeriodSize)
{
    VOC_MSG("in %s buf=%ld, period=%ld\n",__FUNCTION__,nDmaBufSize,nPeriodSize);
    //struct snd_pcm_runtime *runtime = substream->runtime;
    g_nLevelCount = 0;
    g_nLastPeriodNum = 0;
    g_nPeriodSize = nPeriodSize;//frames_to_bytes(runtime, runtime->period_size);
    g_nBufferSize = nDmaBufSize;//runtime->dma_bytes;
    g_nStatus = PCM_NORMAL;
    g_nCm4Period = 0;

    //g_pFlag = ioremap(0x1F006600,2);
  //  memset(g_pFlag,0,2);
}

U32 VocDmaBufferSize(void)
{
    return g_nBufferSize;
}

void VocDmaReset(void)
{
    g_nLevelCount = 0;
    g_nLastPeriodNum = 0;
    g_nStatus = PCM_NORMAL;

}

U32 VocDmaGetLevelCnt(void)
{
    return g_nLevelCount;
}

U32 VocDmaTrigLevelCnt(U32 nDataSize)
{
    if(nDataSize <= g_nLevelCount)
    {
        g_nLevelCount -= nDataSize;
        return nDataSize;
    }
    else
    {
        VOC_MSG("VocDmaTrigLevelCnt size %ld too large %ld\n",nDataSize,g_nLevelCount);
        return 0;
    }
}

/* called in ISR */
U32 VocDmaUpdateLevelCnt( U32 nPeriodNum, U32 nPeriodSize)
{
    U64 nInterval;
    if(g_nCm4Period==0)
    {
        g_nCm4Period = nPeriodSize;
        //g_nLevelCount = nPeriodSize; //1st frame, period num = 0
        VOC_MSG("CM4 period size %ld\n",g_nCm4Period);
    }

    if(nPeriodNum<=g_nLastPeriodNum)
    {
        VOC_MSG("wrap around last= %ld, new %ld\n",g_nLastPeriodNum,nPeriodNum);
        nInterval = (U64)nPeriodNum + MAX_CM4_PERIOD - g_nLastPeriodNum;
    }
    else
    {
        nInterval = (nPeriodNum - g_nLastPeriodNum);
        //if(nInterval>1)
        //    VOC_MSG("CM4 period %d \n",(int)nInterval);

    }

    g_nLevelCount += nInterval*g_nCm4Period;

    /* xrun threshold */
    if(g_nLevelCount > g_nBufferSize-g_nCm4Period*FULL_THRES)
    {
        g_nStatus = PCM_XRUN;
    }


    g_nLastPeriodNum = nPeriodNum;
    return g_nLevelCount;
}

BOOL VocDmaIsDataReady(void)
{
    return (g_nLevelCount >= g_nPeriodSize?true:false);
}

BOOL VocDmaIsXrun(void)
{
    return (g_nStatus==PCM_XRUN?true:false);
}

void VocDmaClrStatus(void)
{
    g_nStatus = PCM_NORMAL;
}

#ifdef __SELF_TEST__
S32 VocDmaInitChannel(U32 nPhysDmaAddr,
                             U32 nBufferSize,
                             U32 nChannels,
                             U32 nSampleWidth,
                             U32 nSampleRate)
{
    MBX_Result tResult;
    MBX_Msg tMsg;

    tMsg.eRoleID = E_MBX_ROLE_HK;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_VD_CONFIG;

    MBX_MSG_PUT_2U32(tMsg, nPhysDmaAddr, nBufferSize);

    tResult=MDrv_MBX_SendMsgExt(&tMsg,E_MBX_ROLE_CP);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_PRINTF(ERROR_LEVEL,"init channel error : %d\n",tResult);
    }
    return tResult;
}


S32 VocDmaStartChannel(void)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    VOC_PRINTF(DEBUG_LEVEL,"in %s\n",__FUNCTION__);

    tMsg.eRoleID = E_MBX_ROLE_HK;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_VD_ENABLE;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = TRUE;

    tResult=MDrv_MBX_SendMsgExt(&tMsg,E_MBX_ROLE_CP);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_PRINTF(ERROR_LEVEL,"start channel error : %d\n",tResult);
    }
    return tResult;
}

S32 VocDmaStopChannel(void)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_HK;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_VD_ENABLE;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = FALSE;

    VOC_PRINTF(DEBUG_LEVEL,"in %s\n",__FUNCTION__);

    tResult=MDrv_MBX_SendMsgExt(&tMsg,E_MBX_ROLE_CP);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_PRINTF(ERROR_LEVEL,"stop channel error : %d\n",tResult);
    }
    return tResult;
}
#else
S32 VocDmaInitChannel(U32 nPhysDmaAddr,
                             U32 nBufferSize,
                             U32 nChannels,
                             U32 nSampleWidth,
                             U32 nSampleRate)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_VD_CONFIG;

    MBX_MSG_PUT_2U32(tMsg, nPhysDmaAddr, g_nBufferSize);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("init channel error : %d\n",tResult);
    }
    return tResult;
}


S32 VocDmaStartChannel(void)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    VOC_MSG("in %s\n",__FUNCTION__);
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_VD_ENABLE;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = TRUE;

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("start channel error : %d\n",tResult);
    }
    return tResult;
}

S32 VocDmaStopChannel(void)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_VD_ENABLE;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = FALSE;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("stop channel error : %d\n",tResult);
    }

    VOC_MSG("VocDmaStopChannel,last Period %ld\n",g_nLastPeriodNum);
    return tResult;
}

/* called when audio close */
S32 VocDmaResetAudio(void)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_RESET;

    tMsg.u8ParameterCount = 0;
    //tMsg.u8Parameters[0] = false;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("reset audio error : %d\n",tResult);
    }
    return tResult;
}

S32 VocDmaEnableSinegen(BOOL bEn)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_SIGEN_ENABLE;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = bEn;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("enable sinegen error : %d\n",tResult);
    }
    return tResult;
}

S32 VocEnableVq(BOOL bEn)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_VQ_ENABLE;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = bEn;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("enable vq error : %d\n",tResult);
    }
    return tResult;
}


S32 VocConfigVq(VQ_CONFIG_S sConfig)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_VQ_CONFIG;


    if(sConfig.nMode>=E_MBX_VQ_MODE_MAX)
        sConfig.nMode = 0;
    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = sConfig.nMode;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("config vq error : %d\n",tResult);
    }
    return tResult;
}

S32 VocEnableDa(BOOL bEn)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_CUS0_ENABLE;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = bEn;

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("enable da error : %d\n",tResult);
    }
    return tResult;
}

S32 VocEnableHpf(U8 nStage)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_HPF_ENABLE;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = nStage;

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("enable Hpf error : %d\n",tResult);
    }
    return tResult;
}


S32 VocConfigHpf(S8 nConfig)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_HPF_CONFIG;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = nConfig;

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("Config Hpf error : %d\n",tResult);
    }
    return tResult;
}

void VocEnableUart(int nEn)
{
    U16 u16Val;
    if (nEn)
    {
    VOC_MSG("in %s Uart enable \n",__FUNCTION__);
    u16Val = INREG16(REG_PM_UART_PAD);
    u16Val &= ~(7<<6);
    OUTREG16(REG_PM_UART_PAD, (u16Val |(0x1<<6)));
    }
    else
    {
    VOC_MSG("in %s Uart disable \n",__FUNCTION__);
    u16Val = INREG16(REG_PM_UART_PAD);
    u16Val &= ~(7<<6);
    OUTREG16(REG_PM_UART_PAD, (u16Val |(0x0<<6)));
    }
}

S32 VocEnableVp(BOOL bEn)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_VP_ENABLE;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = bEn;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("enable vp error : %d\n",tResult);
    }
    return tResult;
}


S32 VocConfigVp(VP_CONFIG_S sConfig)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_VP_CONFIG;


    if(sConfig.nScale>16)
        sConfig.nScale = 16;
    if(sConfig.nScale<1)
        sConfig.nScale = 1;
    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = sConfig.nScale;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("config vq error : %d\n",tResult);
    }
    return tResult;
}

S32 VocSleepCm4(BOOL bEn)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_SLEEP;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = bEn;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("enable sleep error : %d\n",tResult);
    }
    return tResult;
}

S32 VocDmicNumber(U8 nMic)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_MIC_CONFIG;

    tMsg.u8ParameterCount = 3;
    tMsg.u8Parameters[0] = nMic;
    tMsg.u8Parameters[1] = E_MBX_AUD_RATE_16K;
    tMsg.u8Parameters[2] = E_MBX_AUD_BITWIDTH_MAX;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("config dmic number : %d\n",tResult);
    }
    return tResult;
}


S32 VocDmicBitwidth(U8 nBitwidth)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_MIC_CONFIG;

    tMsg.u8ParameterCount = 3;
    tMsg.u8Parameters[0] = E_MBX_AUD_MIC_MAX;
    tMsg.u8Parameters[1] = E_MBX_AUD_RATE_16K;
    tMsg.u8Parameters[2] = nBitwidth;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("config dmic bitwidth : %d\n",tResult);
    }
    return tResult;
}

S32 VocDmicGain(U8 nGain)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_MIC_GAIN;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = nGain;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("config dmic gain : %d\n",tResult);
    }
    return tResult;
}

S32 VocI2sEnable(BOOL bEn)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_I2S_ENABLE;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = bEn;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("enable i2s error : %d\n",tResult);
    }
    return tResult;
}

S32 VocHwAecEnable(BOOL bEn)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_AEC_ENABLE;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = bEn;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("enable hw aec error : %d\n",tResult);
    }
    return tResult;
}

S32 VocSwGain(U8 nGain)
{
    MBX_Result tResult;
    MBX_Msg tMsg;
    tMsg.eRoleID = E_MBX_ROLE_CP;
    tMsg.eMsgType = E_MBX_MSG_TYPE_NORMAL;
    tMsg.u8MsgClass = E_MBX_CLASS_VOC;
    tMsg.u8Index = E_MBX_MSG_SW_GAIN;

    tMsg.u8ParameterCount = 1;
    tMsg.u8Parameters[0] = nGain;

    VOC_MSG("in %s\n",__FUNCTION__);

    tResult=MDrv_VOC_MBX_SendMsg(&tMsg);
    if(tResult!=E_MBX_SUCCESS)
    {
        VOC_ERROR("config sw gain : %d\n",tResult);
    }
    return tResult;
}

#endif

static int VocCopyBin2DramFinish = 0;
bool VocCopyBin2Dram(void)
{
    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos;

    if(VocCopyBin2DramFinish)
        return TRUE;

    fp = filp_open(CM4_PATH, O_RDONLY, 0);

    if (IS_ERR(fp)){
        VOC_ERROR("%s, filp_open failed\n", __FUNCTION__);
        return FALSE;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_read(fp, u8CM4_FW_Binary, CM4_BIN_SIZE, &pos);

    filp_close(fp, NULL);
    set_fs(fs);
    VocCopyBin2DramFinish = 1;

    return TRUE;
}

void VocCm4Init(void)
{
  MHal_CM4_Init();
  MHal_CM4_DcOn();
  MHal_BDMA_Init();
  gInit = 1;
}
#define KEYWORD                 "[BR:"
#define KEYWORD_END             ']'
bool verifyHeader(char* pHeader)
{
    unsigned int nTagSize = *(unsigned int*)(pHeader+0);
    unsigned int nDataSize = *(unsigned int*)(pHeader+nTagSize);
    char Pattern_start[5]={0};
    char *branch = NULL;
    int strIndex = 0;
    bool ret = false;
    int i = 0;
    if((nDataSize == 0) || (nTagSize > 0x100))
    {
        return ret;
    }
    strncpy(Pattern_start,KEYWORD,sizeof(KEYWORD));
    for(i = 0;i < nTagSize; i++)
    {
        if((strIndex==0) && (strncmp(pHeader+i,Pattern_start,strlen(KEYWORD)) == 0))
        {
            VOC_MSG("[%x][%c][%c][%c]\n",i,*(pHeader+i),*(pHeader+i+1),*(pHeader+i+2));
            i+=strlen(KEYWORD);
            strIndex = i;
        }
        else if((strIndex!=0) && ((*(pHeader+i))==KEYWORD_END))
        {
            branch = vmalloc(sizeof(char)*(i-strIndex+1));
            memset(branch,'\0',sizeof(char)*(i-strIndex+1));
            memcpy(branch,pHeader+strIndex,i-strIndex);
            VOC_MSG(" verify header is pass, branch Name:[%s]\n",branch);
            ret = true;
            vfree(branch);
            break;
        }
    }
    return ret;
}

#define CHECK_PATTERN 0x87878787  //check pattern located at last 4 bytes in CM4 bin
S32 VocUpdateCm4Fw(void)
{
    unsigned int sramCRC = 0;
    U8 checkHeader;
    U32 nRetry=0;
    BOOL ret = FALSE;
    unsigned int nTagSize = *(unsigned int*)(u8CM4_FW_Binary+0);
    unsigned int nDataSize = *(unsigned int*)(u8CM4_FW_Binary+nTagSize);
    unsigned int nDataOffset = nTagSize+4;
    unsigned int nCRC  = *(unsigned int*)(u8CM4_FW_Binary+nTagSize-4); //last 4 bytes will be CRC

    //check header
    VOC_MSG("in %s, nCRC: %x \n",__FUNCTION__,nCRC);
    if(gInit != 1)
    {
        VOC_MSG("gInit != 1 in %s[%d]\n",__FUNCTION__,__LINE__);
        VocCm4Init();
    }else{
        VOC_MSG("gInit == 1 in %s,[%d]\n",__FUNCTION__,__LINE__);
    }
    MHal_CM4_Halt();
    if (verifyHeader(u8CM4_FW_Binary))
    {
        while(nRetry++<5)
        {
            ret = MDrv_BDMA_Copy((virt_to_phys(u8CM4_FW_Binary)-MIU_BASE)+nDataOffset, 0x0, nDataSize, E_BDMA_HKtoCP);
            sramCRC = (unsigned int) MDrv_BDMA_Crc32(0x0, nDataSize, E_BDMA_DEV_IMI);
            VOC_MSG("in %s, sramCRC: [%x] ,nCRC: [%x]\n",__FUNCTION__,sramCRC,nCRC);
            if(sramCRC == nCRC)
            {
               VOC_MSG("CRC pass , break\n");
                break;
            }
        }

        if(nRetry>5)
        {
            VOC_ERROR("in %s, load code failed!!\n",__FUNCTION__);
            return -1;
        }

        MHal_CM4_Run();

        /* wait CM4 ready */
        nRetry = 0;
        while(nRetry++<100)
        {
            ret = MDrv_BDMA_PatternSearch(0+nDataSize-4, 4, CHECK_PATTERN, E_BDMA_DEV_IMI);
            if(ret == TRUE)
            {
                 break;
            }
            mdelay(1);
        }
        if(nRetry >= 101)
        {
            VOC_ERROR("out %s, CM4 not work!!\n",__FUNCTION__);
            return -1;
        }
        VOC_MSG("wait %lu ms, out %s\n",nRetry-1,__FUNCTION__);
    }
    else
    {
        VOC_MSG("%s CM4 no header \n",__FUNCTION__);
        //memcpy_toio(g_pImiBase, u8CM4_FW_Binary, sizeof(u8CM4_FW_Binary));
        MDrv_BDMA_Copy(virt_to_phys(u8CM4_FW_Binary)-MIU_BASE, 0x0, sizeof(u8CM4_FW_Binary), E_BDMA_HKtoCP);
        MHal_CM4_Run();
        mdelay(100); // 100ms
        VOC_MSG("out %s\n",__FUNCTION__);
    }
    return 0;
}


void VocDcOn(void)
{
    VOC_MSG("in %s\n",__FUNCTION__);
    MHal_CM4_DcOn();
    MHal_BDMA_Init();
    MHAL_VOC_MBX_Init(E_MBX_ROLE_HK);
    VOC_MSG("out %s\n",__FUNCTION__);
}

void VocDcOff(void)
{
    VOC_MSG("in %s\n",__FUNCTION__);
    MHal_CM4_DcOff();
    g_bDisabledCM4Flag = false;  // clean cm4 has been shut down flag
    VOC_MSG("out %s\n",__FUNCTION__);
}
void VocInitOff(void)
{
    VOC_MSG("in %s\n",__FUNCTION__);
    MHal_CM4_InitOff();
    gInit = 0;
    g_bDisabledCM4Flag = true;  // set cm4 shut down flag
    VOC_MSG("out %s\n",__FUNCTION__);
}

/**
 * VocIsCm4ShutDown - check if cm4 has been shut down or not.
 *
 * TV Soc with CM4 have a case: Totally shut down cm4 for power consumption lower than 0.5W.
 * If the VQ off, it will totally shut down cm4 after str off.
 * But at the time of str on, it need to check cm4 has been shut down or not when reload some cm4 config
 *
 * Return: true if successful, or a false on failure.
 */
bool VocIsCm4ShutDown(void)
{
    VOC_MSG("in %s\n",__FUNCTION__);
    if(true == g_bDisabledCM4Flag)
    {
        g_bDisabledCM4Flag = false;  // clean cm4 has been shut down flag
        return true;
    }
    else
    {
        return false;
    }
}
