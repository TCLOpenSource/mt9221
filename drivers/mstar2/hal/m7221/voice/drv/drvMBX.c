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
/// file    drvMBX.c
/// @brief  MStar MailBox DDI
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#define _DRV_MBX_C
/* Standard includes. */

/* Kernel includes. */
#include "mosWrapper.h"

/* driver includes. */
#include "drvMBX.h"
#include "halMBX.h"
#include "halCPUINT.h"
#include <linux/delay.h>

#define MBX_MSG(fmt, args...) MOS_DBG_PRINT(MOS_DBG_LEVEL_MBX, "[MBX] " fmt, ##args)
#define MBX_ERROR(fmt, args...) MOS_DBG_ERROR("[MBX ERR] " fmt, ##args);

//=============================================================================
// Local Variables
//=============================================================================
typedef struct
{
  MBX_ROLE_ID eMbxHostRole;
  CPUINT_ID_e eMbxRole2Cpu[E_MBX_ROLE_MAX];
  //struct mutex tLock;
  spinlock_t            tMbxLock;
} MBX_Mgr;


static MBX_Mgr _mgrMBX;
static U32 _u32TimeoutMillSecs;

/*-----------------------------------------------------------*/

/* jiffies can't be used when local_irq_disabled*/
U32 _MDrv_VOC_MBX_GetSystemTime(void)
{
//    U64 CurTimeUs = Chip_Get_US_Ticks();
    struct timespec  tv;
    ktime_get_ts(&tv);
    U64 CurTimeUs = tv.tv_sec * 1000000LL + (u64) (tv.tv_nsec / 1000LL);

    do_div(CurTimeUs,1000);
    return (U32)CurTimeUs;
    //return jiffies_to_msecs(jiffies);
}

void MDrv_VOC_MBX_Init(U32 nTimeOutMs)
{

  MHAL_VOC_MBX_Init(E_MBX_ROLE_HK);

  _mgrMBX.eMbxHostRole = E_MBX_ROLE_HK;
  _mgrMBX.eMbxRole2Cpu[E_MBX_ROLE_CP] = E_CPUINT_CP;
  _mgrMBX.eMbxRole2Cpu[E_MBX_ROLE_HK] = E_CPUINT_HK0;

  _u32TimeoutMillSecs = nTimeOutMs;

  //mutex_init(&_mgrMBX.tLock);
  spin_lock_init(&_mgrMBX.tMbxLock);

}

#if 0
MBX_Result MDrv_VOC_MBX_SendMsg(MBX_Msg *pMsg)
{
  MBX_Result mbxResult = E_MBX_SUCCESS;
  MBXHAL_Fire_Status mbxHalFireStatus;
  U32 u32WaitCnt = 0;
  //parameter check:
  if(NULL == pMsg)
  {
    MBX_ERROR("MDrv_VOC_MBX_SendMsg: E_MBX_ERR_INVALID_PARAM! \n");
    return E_MBX_ERR_INVALID_PARAM;
  }
  mutex_lock(&_mgrMBX.tLock);
  do
  {
    mbxResult = MHAL_VOC_MBX_GetFireStatus(_mgrMBX.eMbxHostRole, pMsg->eRoleID, &mbxHalFireStatus);
    if(E_MBX_SUCCESS != mbxResult)
    {
      MBX_ERROR("MDrv_VOC_MBX_SendMsg: MHAL_MBX_GetFireStatus failed = %d \n ", mbxResult);
      return E_MBX_ERR_INVALID_PARAM;
    }

    if(E_MBXHAL_FIRE_SUCCESS == mbxHalFireStatus)
      break;

    //check If Timeout:
    u32WaitCnt++;
    if(u32WaitCnt >= 0x10000)
      break;
  }
  while(1);

  //mbxResult = _MDrv_VOC_MBX_SendMsg(pMsg, _mgrMBX.eMbxHostRole);
  MHAL_VOC_MBX_ClearAll(pMsg, _mgrMBX.eMbxHostRole);
  mbxResult = MHAL_VOC_MBX_Fire(pMsg, _mgrMBX.eMbxHostRole);
  if(E_MBX_SUCCESS == mbxResult)
  {
    MHal_CPUINT_Fire(_mgrMBX.eMbxRole2Cpu[pMsg->eRoleID], _mgrMBX.eMbxRole2Cpu[_mgrMBX.eMbxHostRole]);
  }
  else
  {
    MBX_ERROR("MDrv_VOC_MBX_SendMsg: MHAL_MBX_Fire Failed! Result=%d \n", mbxResult);
  }
  mutex_unlock(&_mgrMBX.tLock);
  return mbxResult;
}
#else
MBX_Result MDrv_VOC_MBX_SendMsg(MBX_Msg *pMsg)
{
  MBX_Result mbxResult = E_MBX_SUCCESS;
  MBXHAL_Fire_Status mbxHalFireStatus;
  MBXHAL_Cmd_Status mbxHalCmdStatus;
  U32 u32WaitCnt = 0;
  U32 u32TimeTicket;
  //parameter check:
  if(NULL == pMsg)
  {
    MBX_ERROR("MDrv_VOC_MBX_SendMsg: E_MBX_ERR_INVALID_PARAM! \n");
    return E_MBX_ERR_INVALID_PARAM;
  }
  //mutex_lock(&_mgrMBX.tLock);
  spin_lock(&_mgrMBX.tMbxLock);
  #if 0
  do
  {
    mbxResult = MHAL_VOC_MBX_GetFireStatus(_mgrMBX.eMbxHostRole, pMsg->eRoleID, &mbxHalFireStatus);
    if(E_MBX_SUCCESS != mbxResult)
    {
      MBX_ERROR("MDrv_VOC_MBX_SendMsg: MHAL_MBX_GetFireStatus failed = %d \n ", mbxResult);
      return E_MBX_ERR_INVALID_PARAM;
    }

    if(E_MBXHAL_FIRE_SUCCESS == mbxHalFireStatus)
      break;

    //check If Timeout:
    u32WaitCnt++;
    if(u32WaitCnt >= 0x10000)
    {
      //break;
      MBX_ERROR("MDrv_VOC_MBX_SendMsg: timeout failed = %d \n ", mbxHalFireStatus);
      return E_MBX_ERR_TIME_OUT;
    }
    //usleep_range(100, 200);
    mdelay(1);
  }
  while(1);

  //mbxResult = _MDrv_VOC_MBX_SendMsg(pMsg, _mgrMBX.eMbxHostRole);
  MHAL_MBX_ClearAll(pMsg, _mgrMBX.eMbxHostRole);
  #endif
  mbxResult = MHAL_VOC_MBX_Fire(pMsg, _mgrMBX.eMbxHostRole);
  if(E_MBX_SUCCESS == mbxResult)
  {
    MHal_CPUINT_Fire(_mgrMBX.eMbxRole2Cpu[pMsg->eRoleID], _mgrMBX.eMbxRole2Cpu[_mgrMBX.eMbxHostRole]);

    u32TimeTicket = _MDrv_VOC_MBX_GetSystemTime();
    do
    {
      mbxResult = MHAL_VOC_MBX_GetFireStatus(_mgrMBX.eMbxHostRole , pMsg->eRoleID, &mbxHalFireStatus);

      if(E_MBX_SUCCESS != mbxResult)
      {
        break;
      }

      if(E_MBXHAL_FIRE_OVERFLOW == mbxHalFireStatus)
      {
         mbxResult = E_MBX_ERR_PEER_CPU_OVERFLOW;
         break;
      }

      if(E_MBXHAL_FIRE_DISABLED == mbxHalFireStatus)
      {
        mbxResult = E_MBX_ERR_PEER_CPU_NOTREADY;
        break;
      }

      if(E_MBXHAL_FIRE_SUCCESS == mbxHalFireStatus)
      {
        mbxResult = E_MBX_SUCCESS;
        break;
      }

      //check If Timeout:
      u32WaitCnt++;
      if(u32WaitCnt >= 0x10000)
      {
        U32 u32CurTime=_MDrv_VOC_MBX_GetSystemTime();
        if((u32CurTime-u32TimeTicket) >= _u32TimeoutMillSecs)
        {

           MHAL_VOC_MBX_SetConfig(_mgrMBX.eMbxHostRole );
           mbxResult = E_MBX_ERR_PEER_CPU_NOT_ALIVE;
           break;
        }
        u32WaitCnt = 0;
      }
    }while(TRUE);
  }
  else
  {
    MBX_ERROR("MDrv_VOC_MBX_SendMsg: MHAL_MBX_Fire Failed! Result=%d \n", mbxResult);
  }

  if (mbxResult != E_MBX_SUCCESS)
  {
    MBX_ERROR("MDrv_VOC_MBX_SendMsg: Failed! Result=%d \n", mbxResult);
    MHAL_VOC_MBX_ClearStatus(pMsg, _mgrMBX.eMbxHostRole);
  }
  else
  {
    mbxResult = MHAL_VOC_MBX_GetCmdStatus(_mgrMBX.eMbxHostRole , pMsg->eRoleID, &mbxHalCmdStatus);
    if(E_MBX_SUCCESS != mbxResult)
    {
      MBX_ERROR("MHAL_VOC_MBX_GetCmdStatus: Failed! Result=%d \n", mbxResult);
    }
    else
    {
      if(E_MBXHAL_CMD_ERROR== mbxHalCmdStatus)
      {
        MBX_ERROR("MDrv_VOC_MBX_SendMsg: error parameter! \n");
        mbxResult = E_MBX_ERR_INVALID_PARAM;
        MHAL_VOC_MBX_ClearStatus(pMsg, _mgrMBX.eMbxHostRole);

      }
    }

  }

  //mutex_unlock(&_mgrMBX.tLock);
  spin_unlock(&_mgrMBX.tMbxLock);
  return mbxResult;
}
#endif
MBX_Result MDrv_MBX_SendMsgExt(MBX_Msg *pMsg, MBX_ROLE_ID eSrcRole)
{
  MBX_Result mbxResult = E_MBX_SUCCESS;
  MBXHAL_Fire_Status mbxHalFireStatus;
  U32 u32WaitCnt = 0;
  //parameter check:
  if(NULL == pMsg)
  {
    MBX_ERROR("MDrv_MBX_SendMsgExt: E_MBX_ERR_INVALID_PARAM! \n");
    return E_MBX_ERR_INVALID_PARAM;
  }
  //mutex_lock(&_mgrMBX.tLock);
  spin_lock(&_mgrMBX.tMbxLock);
  do
  {
    mbxResult = MHAL_VOC_MBX_GetFireStatus(eSrcRole, pMsg->eRoleID, &mbxHalFireStatus);
    if(E_MBX_SUCCESS != mbxResult)
    {
      MBX_ERROR("MDrv_MBX_SendMsgExt: MHAL_MBX_GetFireStatus failed = %d \n ", mbxResult);
      return E_MBX_ERR_INVALID_PARAM;
    }

    if(E_MBXHAL_FIRE_SUCCESS == mbxHalFireStatus)
      break;

    //check If Timeout:
    u32WaitCnt++;
    if(u32WaitCnt >= 0x10000)
      break;
  }
  while(1);

  //mbxResult = _MDrv_MBX_SendMsg(pMsg, _mgrMBX.eMbxHostRole);
  MHAL_MBX_ClearAll(pMsg, eSrcRole);
  mbxResult = MHAL_VOC_MBX_Fire(pMsg, eSrcRole);
  if(E_MBX_SUCCESS == mbxResult)
  {
    MHal_CPUINT_Fire(_mgrMBX.eMbxRole2Cpu[pMsg->eRoleID], _mgrMBX.eMbxRole2Cpu[eSrcRole]);
  }
  else
  {
    MBX_ERROR("MDrv_MBX_SendMsg: MDrv_MBX_SendMsgExt Failed! Result=%d \n", mbxResult);
  }
  //mutex_unlock(&_mgrMBX.tLock);
  spin_unlock(&_mgrMBX.tMbxLock);
  return mbxResult;
}


/*MBX_Result MDrv_MBX_RecvMsg(MBX_Msg *pMsg, BOOL bBlock)
{
  TickType_t xTicksToWait;
  MBX_Result mbxResult = E_MBX_SUCCESS;

  //parameter check:
  if((pMsg == NULL))
  {
    MBX_ERROR("MDrv_MBX_RecvMsg: E_MBX_ERR_INVALID_PARAM! \n");
    return E_MBX_ERR_INVALID_PARAM;
  }

  if(bBlock)
  {
    xTicksToWait = portMAX_DELAY;
    MOS_IRQn_Unmask(_mgrMBX.nMbxIntNum);
  }
  else
  {
    xTicksToWait = 0;
    MOS_IRQn_Mask(_mgrMBX.nMbxIntNum);
  }

  if(xSemaphoreTake(_mgrMBX.tMbxIntSem, portMAX_DELAY) == pdPASS)
    mbxResult = MHAL_MBX_Recv(pMsg,  _mgrMBX.eMbxHostRole);

  if((E_MBX_SUCCESS != mbxResult) && (E_MBX_ERR_NO_MORE_MSG != mbxResult))
  {
    MBX_ERROR("MDrv_MBX_SendMsg: MDrv_MBX_RecvMsg Failed! Result=%d \n", mbxResult);
  }

  return mbxResult;
}*/


//called on ISR
MBX_Result MDrv_MBX_RecvMsgExt(MBX_Msg *pMsg)
{
  MBX_Result mbxResult = E_MBX_SUCCESS;

  //parameter check:
  if((pMsg == NULL))
  {
    MBX_ERROR("MDrv_MBX_RecvMsgExt: E_MBX_ERR_INVALID_PARAM! \n");
    return E_MBX_ERR_INVALID_PARAM;
  }

  mbxResult = MHAL_VOC_MBX_Recv(pMsg,  _mgrMBX.eMbxHostRole);

  if((E_MBX_SUCCESS != mbxResult) && (E_MBX_ERR_NO_MORE_MSG != mbxResult))
  {
    MBX_ERROR("MDrv_MBX_RecvMsgExt: MDrv_MBX_RecvMsg Failed! Result=%d \n", mbxResult);
  }

  MHAL_VOC_MBX_RecvEnd(pMsg->eRoleID, _mgrMBX.eMbxHostRole, E_MBXHAL_RECV_SUCCESS);

  return mbxResult;
}

