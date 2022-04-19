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
/// @file   drvMBX.h
/// @brief  MStar Mailbox Driver DDI
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

/*! \defgroup G_MBX MBX interface
    \ingroup  G_PERIPHERAL

    \brief

    This drvMBX.h provided function for one processor to send a message to another processor, also
	receive messages from others processor.

    <b>Features</b>
    - Mailbox Initialize & De-initialize function
    - Mailbox Class Register function
    - Mailbox Send & Receive function

    <b> Operation Code Flow: </b> \n
    -# MBX initialize with configuration.
    -# Correct MBX class for specific thread.
    -# Send MBX message.
	-# Receive MBX message.

    \defgroup G_MBX_INIT Initialization Task relative
    \ingroup  G_MBX
    \defgroup G_MBX_COMMON Common Task relative
    \ingroup  G_MBX
    \defgroup G_MBX_CONTROL Control relative
    \ingroup  G_MBX
    \defgroup G_MBX_OTHER  other relative
    \ingroup  G_MBX
    \defgroup G_MBX_ToBeModified MBX api to be modified
    \ingroup  G_MBX
    \defgroup G_MBX_ToBeRemove MBX api to be removed
    \ingroup  G_MBX
*/

#ifndef _DRV_MBX_H
#define _DRV_MBX_H

#ifdef __cplusplus
extern "C"
{
#endif

//=============================================================================
// Includs
//=============================================================================
#include "mosWrapper.h"


//=============================================================================
// Defines & Macros
//=============================================================================

/// Max Mailbox Queque Size for one Mail Class
#define MAX_MBX_QUEUE_SIZE      256
/// Max Parameter Size ( in bytes ) for one Mail Message
#define MAX_MBX_PARAM_SIZE      10
/// Max Size ( in bytes ) for SetInformation
#define MAX_MBX_INFORMATION_SIZE    8

/// Define The Queue Status of The Mail Class queue
/// Queue overflow: new incoming mail will be skip
#define MBX_STATUS_QUEUE_OVER_FLOW       0x00000001
/// Queque Has Instant Msg: there has new instant message in queue
#define MBX_STATUS_QUEUE_HAS_INSTANT_MSG 0x00000002
/// Queue Has Normal Msg: there has new normal message in queue
#define MBX_STATUS_QUEUE_HAS_NORMAL_MSG  0x00000004

/// Define The Flags for Receive Message from Message Queue
/// Recv any message class, this means, the targetClass is useless if this bit is set
//#define MBX_CHECK_ALL_MSG_CLASS 0x00000001
/// Check Instant Message
#define MBX_CHECK_INSTANT_MSG   0x00000002
/// Check Normal Message
#define MBX_CHECK_NORMAL_MSG    0x00000004
/// Recv any message class, this means, the targetClass is useless if this bit is set
#define MBX_CHECK_ALL_MSG_CLASS (MBX_CHECK_INSTANT_MSG|MBX_CHECK_NORMAL_MSG)
/// Block Receive function call until time out if no message available
#define MBX_CHECK_BLOCK_RECV    0x80000000

/// Define debug level:
/// Default Debug Level: No any Debug Message Print out
#define MBX_DBG_LEVEL_NONE       0x00
/// HAL Debug Level: Print out MBX Hal Error/Warning
#define MBX_DBG_LEVEL_MBXHAL     0x01
/// Message Queue Debug Level: Print out Message Queue Error/Warning
#define MBX_DBG_LEVEL_MSGQ       0x02
/// Interrupt HAL Debug Level: Print out Inerrut HAL Error/Warning
#define MBX_DBG_LEVEL_MBXINT     0x04
/// Critical Debug Level: Print out any Critical Error/Warning
#define MBX_DBG_LEVEL_CRITICAL   0x08
/// All Debug Level: Print out any Error/Wanring, That is, All above Debug Level Errors/Warnings
#define MBX_DBG_LEVEL_ALL        0x0F


#define MBX_MSG_PUT_U32(msg, val) { \
                                   msg.u8ParameterCount = 4;\
                                   msg.u8Parameters[0] = val & 0xff;\
                                   msg.u8Parameters[1] = val>>8 & 0xff;\
                                   msg.u8Parameters[2] = val>>16 & 0xff;\
                                   msg.u8Parameters[3] = val>>24 & 0xff;\
                                 }

#define MBX_MSG_PUT_2U32(msg, val1, val2) { \
                                   msg.u8ParameterCount = 8;\
                                   msg.u8Parameters[0] = val1 & 0xff;\
                                   msg.u8Parameters[1] = val1>>8 & 0xff;\
                                   msg.u8Parameters[2] = val1>>16 & 0xff;\
                                   msg.u8Parameters[3] = val1>>24 & 0xff;\
                                   msg.u8Parameters[4] = val2 & 0xff;\
                                   msg.u8Parameters[5] = val2>>8 & 0xff;\
                                   msg.u8Parameters[6] = val2>>16 & 0xff;\
                                   msg.u8Parameters[7] = val2>>24 & 0xff;\
                                 }

#define MBX_MSG_PUT_2U32plusU16(msg, val1, val2, val3) { \
                                   msg.u8ParameterCount = 10;\
                                   msg.u8Parameters[0] = val1 & 0xff;\
                                   msg.u8Parameters[1] = val1>>8 & 0xff;\
                                   msg.u8Parameters[2] = val1>>16 & 0xff;\
                                   msg.u8Parameters[3] = val1>>24 & 0xff;\
                                   msg.u8Parameters[4] = val2 & 0xff;\
                                   msg.u8Parameters[5] = val2>>8 & 0xff;\
                                   msg.u8Parameters[6] = val2>>16 & 0xff;\
                                   msg.u8Parameters[7] = val2>>24 & 0xff;\
                                   msg.u8Parameters[8] = val3 & 0xff;\
                                   msg.u8Parameters[9] = val3>>8 & 0xff;\
                                 }


#define MBX_MSG_GET_U32(msg, val) { \
                                  val = (U32)(msg.u8Parameters[0] | msg.u8Parameters[1]<<8 | msg.u8Parameters[2]<<16 | msg.u8Parameters[3]<<24);\
                                 }

#define MBX_MSG_GET_2U32(msg, val1, val2) { \
                                  val1 = (U32)(msg.u8Parameters[0] | msg.u8Parameters[1]<<8 | msg.u8Parameters[2]<<16 | msg.u8Parameters[3]<<24);\
                                  val2 = (U32)(msg.u8Parameters[4] | msg.u8Parameters[5]<<8 | msg.u8Parameters[6]<<16 | msg.u8Parameters[7]<<24);\
                                 }

#define MBX_MSG_GET_2U32plusU16(msg, val1, val2, val3) { \
                                  val1 = (U32)(msg.u8Parameters[0] | msg.u8Parameters[1]<<8 | msg.u8Parameters[2]<<16 | msg.u8Parameters[3]<<24);\
                                  val2 = (U32)(msg.u8Parameters[4] | msg.u8Parameters[5]<<8 | msg.u8Parameters[6]<<16 | msg.u8Parameters[7]<<24);\
                                  val3 = (U16)(msg.u8Parameters[8] | msg.u8Parameters[9]<<8);\
                                 }

//=============================================================================
// Type and Structure Declaration
//=============================================================================

//=============================================================================
// Enums
/// Mail Message Class Defines
typedef enum
{
    /// MBX_CLASS_SYSTEM
    E_MBX_CLASS_SYSTEM         = 0,
    E_MBX_CLASS_VOC            = 0,
    E_MBX_CLASS_NUM,
    /// The End NUmber of Max Class ID
    E_MBX_CLASS_MAX = 0xFF
}MBX_Class;


/// Mail Box Supported CPU ID Defins
typedef enum
{
    /// House Keeping Identifier
    E_MBX_ROLE_HK = 0,
    /// Co-Processer Identifier
    E_MBX_ROLE_CP = 1,

    E_MBX_ROLE_MAX
}MBX_ROLE_ID;


/// Mail Message Class Type Defines
typedef enum
{
    /// Normal Message Class Type
    E_MBX_MSG_TYPE_NORMAL  = 0,
    /// Instant Message Class Type
    E_MBX_MSG_TYPE_INSTANT = 1
}MBX_MSG_Type;

/// MB DDI return value
typedef enum
{
    /// Success Call
    E_MBX_SUCCESS                = 0,
    /// Error: Not Initialized
    E_MBX_ERR_NOT_INITIALIZED        = 1,
    /// Error: No more Memory, Queue Memory Issue
    E_MBX_ERR_NO_MORE_MEMORY         = 2,
    /// Error: class has been used by other APP
    E_MBX_ERR_SLOT_BUSY              = 3,
    /// Error: class has been openned by this APP, you do not need to open it again
    E_MBX_ERR_SLOT_AREADY_OPENNED    = 4,
    /// Error: class not registered yet
    E_MBX_ERR_SLOT_NOT_OPENNED       = 5,
    /// Error: unknow cpu id
    E_MBX_ERR_INVALID_CPU_ID         = 6,
    /// Error: invalid parameter
    E_MBX_ERR_INVALID_PARAM          = 7,
    /// Error: peer cpu is peek Mail from Hardware, you can not send mail now
    E_MBX_ERR_PEER_CPU_BUSY          = 8,
    /// Error: peer cpu do not alive, Mail never peek out, Need check peer cpu is alive or not
    E_MBX_ERR_PEER_CPU_NOT_ALIVE     = 9,
    /// Error: peer cpu not initialized yet, not ready to receive mail message
    E_MBX_ERR_PEER_CPU_NOTREADY = 10,
    /// Error: peer cpu the dedicated class Overflow!
    E_MBX_ERR_PEER_CPU_OVERFLOW = 11,
    /// Error: the mail message has been fetched yet, there has no message in hardware
    E_MBX_ERR_MSG_ALREADY_FETCHED = 12,
    /// Error: time out with dedicated request
    E_MBX_ERR_TIME_OUT               = 13,
    /// Error: no mail message in message queue
    E_MBX_ERR_NO_MORE_MSG            = 14,
    /// Error: has mail message in queue when un-register mail class or DeInit MailBox Driver
    E_MBX_ERR_HAS_MSG_PENDING        = 15,
    /// Error: not implemente yet for request
    E_MBX_ERR_NOT_IMPLEMENTED        = 16,
    /// Error: unknow error, like system error
    E_MBX_UNKNOW_ERROR           = 0xFFFF
} MBX_Result;

//=============================================================================
// Structurs

/// Mail Message Define
typedef struct
{
    /// Role ID, for send, this is the Mail Target, for receive, this is the Source which sent this message
    MBX_ROLE_ID      eRoleID;
    /// Mail Message Type, Normal or Instant
    MBX_MSG_Type     eMsgType;

    /// Ctrl Byte in Mail Message
    U8              u8Ctrl;
    /// @ref MBX_Class
    U8              u8MsgClass; //should be MBX_Class, for alignment let it be S8, gcc default sizeof(enum) is 4 bytes.
    /// Command Index defined by Apps
    U8              u8Index;
    /// Parameter Count
    U8              u8ParameterCount;
    /// Parameters, Max Number @ref MAX_MBX_PARAM_SIZE
    U8              u8Parameters[MAX_MBX_PARAM_SIZE];
    /// Status 0 byte of Mail Message
    U8              u8S0;
    /// Status 1 byte of Mail Message
    U8              u8S1;
}MBX_Msg;

typedef enum
{
  E_MBX_VQ_MODE_NORMAL = 0,
  E_MBX_VQ_MODE_LOOP,
  E_MBX_VQ_MODE_PM,
  E_MBX_VQ_MODE_VAD,
  E_MBX_VQ_MODE_DWW,
  E_MBX_VQ_MODE_TEST,
  E_MBX_VQ_MODE_SBOOT,
  E_MBX_VQ_MODE_MAX,
} MBX_VQ_Mode_e;

typedef enum
{
  E_MBX_AUD_I2S_FMT_I2S,
  E_MBX_AUD_I2S_FMT_LEFT_JUSTIFY
} MBX_AudI2sFmt_e;

#if 0
typedef enum
{
  E_MBX_AUD_MIC_NO2,
  E_MBX_AUD_MIC_NO4,
  E_MBX_AUD_MIC_NO6,
  E_MBX_AUD_MIC_NO8,
  E_MBX_AUD_MIC_MAX
} MBX_AudMic_e;

#else

typedef enum
{
  E_MBX_AUD_MIC_NO2,
  E_MBX_AUD_MIC_NO4,
  E_MBX_AUD_MIC_NO6,
  E_MBX_AUD_MIC_NO8,
  E_MBX_AUD_MIC_NO1L,
  E_MBX_AUD_MIC_NO1R,
  E_MBX_AUD_MIC_MAX
} MBX_AudMic_e;
#endif

typedef enum
{
  E_MBX_AUD_REF_NO0,
  E_MBX_AUD_REF_NO2,
  E_MBX_AUD_REF_NO4,
  E_MBX_AUD_REF_NO6,
  E_MBX_AUD_REF_NO8,
  E_MBX_AUD_REF_MAX
} MBX_AudRef_e;


typedef enum
{
  E_MBX_AUD_BITWIDTH_16,
  E_MBX_AUD_BITWIDTH_24,
  E_MBX_AUD_BITWIDTH_32,
  E_MBX_AUD_BITWIDTH_MAX,
} MBX_AudBitWidth_e;

typedef enum
{
  E_MBX_AUD_I2S_MSMODE_SLAVE,
  E_MBX_AUD_I2S_MSMODE_MASTER,
} MBX_AudI2sMsMode_e;

typedef enum
{
  E_MBX_AUD_I2S_MODE_I2S,
  E_MBX_AUD_I2S_MODE_TDM,
} MBX_AudI2sMode_e;

typedef enum
{
  E_MBX_AUD_RATE_SLAVE,
  E_MBX_AUD_RATE_8K,
  E_MBX_AUD_RATE_16K,
  E_MBX_AUD_RATE_32K,
  E_MBX_AUD_RATE_48K,
  E_MBX_AUD_RATE_96K,
  E_MBX_AUD_RATE_MAX
} MBX_AudRate_e;

typedef enum
{
  E_MBX_MSG_NONE = 0,
  E_MBX_MSG_VD_ENABLE, /* u8ParameterCount:1, u8Parameters[0]:TRUE/FALSE */
  E_MBX_MSG_VD_CONFIG, /* u8ParameterCount:8, u8Parameters[0:3]:ACPU buffer addr, u8Parameters[4:7]:ACPU buffer size */
  E_MBX_MSG_VQ_ENABLE, /* u8ParameterCount:1, u8Parameters[0]:TRUE/FALSE */
  E_MBX_MSG_VQ_CONFIG, /* u8ParameterCount:1, u8Parameters[0]:VQ Mode, refer MBX_VQ_Mode_e */
  E_MBX_MSG_VP_ENABLE, /* u8ParameterCount:1, u8Parameters[0]:TRUE/FALSE */
  E_MBX_MSG_VP_CONFIG, /* u8ParameterCount:1, u8Parameters[0]:Scale(1-16) */
  E_MBX_MSG_RESET,     /* u8ParameterCount:0 */
  E_MBX_MSG_SIGEN_ENABLE, /* u8ParameterCount:1, u8Parameters[0]:TRUE/FALSE */
  E_MBX_MSG_MIC_CONFIG,   /* u8ParameterCount:3, u8Parameters[0]:mic number (MBX_AudMic_e), u8Parameters[1]:sample rate (MBX_AudRate_e), u8Parameters[2]:bit width(MBX_AudBitWidth_e)*/
  E_MBX_MSG_MIC_GAIN,     /* u8ParameterCount:2, u8Parameters[1:0]: Gain step(0.125dB per step)*/
  E_MBX_MSG_I2S_ENABLE,   /* u8ParameterCount:1, u8Parameters[0]:TRUE/FALSE */
  E_MBX_MSG_I2S_CONFIG,   /* u8ParameterCount:1, u8Parameters[0]: */
  E_MBX_MSG_AEC_ENABLE,   /* u8ParameterCount:1, u8Parameters[0]:ref channel number */
  E_MBX_MSG_TEST_ENABLE,   /* u8ParameterCount:1, u8Parameters[0]:TRUE/FALSE */
  E_MBX_MSG_TEST_REG,      /* u8ParameterCount:4, u8Parameters[0:3]:customer function pointer for internal test */
  E_MBX_MSG_PERIOD_NOTIFY, /* u8ParameterCount:8, u8Parameters[0:3]: period count, u8Parameters[4:7]: period size */
  E_MBX_MSG_STARTTIME_NOTIFY, /* u8ParameterCount:8, u8Parameters[0:3]: low part of start time, u8Parameters[4:7]: high part of start time */
  E_MBX_MSG_STATUS_NOTIFY, /*  */
  E_MBX_MSG_SLEEP,         /* u8ParameterCount:1, u8Parameters[0]: suspend/resume(1/0) for test */
  E_MBX_MSG_CUS0_ENABLE,   /* u8ParameterCount:1, u8Parameters[0]:TRUE/FALSE */
  E_MBX_MSG_CUS1_ENABLE,   /* u8ParameterCount:1, u8Parameters[0]:TRUE/FALSE */
  E_MBX_MSG_CUS2_ENABLE,   /* u8ParameterCount:1, u8Parameters[0]:TRUE/FALSE */
  E_MBX_MSG_HPF_ENABLE,    /* u8ParameterCount:1, u8Parameters[0]:off/1-stage/2-stage */
  E_MBX_MSG_HPF_CONFIG,    /* u8ParameterCount:1, u8Parameters[0]:PARAMETER(-2 ~ 9) */
  E_MBX_MSG_AMIC_ENABLE,   /* u8ParameterCount:1, u8Parameters[0]:TRUE/FALSE */
  E_MBX_MSG_AMIC_CONFIG,   /* u8ParameterCount:2, u8Parameters[0]:MICAMP GAIN, u8Parameters[1]:ADC GAIN  */
  E_MBX_MSG_SW_GAIN,       /* u8ParameterCount:1, u8Parameters[0]: Gain level(1dB per step, 0~5)*/
  E_MBX_MSG_VAD_GAIN,       /* u8ParameterCount:1, u8Parameters[0]: Gain level(1dB per step, 0~5)*/
  E_MBX_MSG_MIC_MAP,  /* u8ParameterCount:2, u8Parameters[0]:destination mic number, u8Parameters[1]:source mic number*/
  E_MBX_MSG_PM_GPIO,  /* u8ParameterCount:3, u8Parameters[0]:in/out (0/1), u8Parameters[1]:gpio number (3-12), u8Parameters[2]:init status (0-1)*/
  E_MBX_MSG_REF_CONFIG,   /* u8ParameterCount:3, u8Parameters[0]:ref number (MBX_AudRef_e), u8Parameters[1]:sample rate (MBX_AudRate_e), u8Parameters[2]:bit width(MBX_AudBitWidth_e)*/
  E_MSG_NUM,
} MBX_MSG_e;

//void NVIC_Handler_MBX(void);
void MDrv_VOC_MBX_Init(U32 nTimeOutMs);
MBX_Result MDrv_VOC_MBX_SendMsg(MBX_Msg *pMsg);
//MBX_Result MDrv_MBX_RecvMsg(MBX_Msg *pMsg);
MBX_Result MDrv_MBX_RecvMsgExt(MBX_Msg *pMsg);
MBX_Result MDrv_MBX_SendMsgExt(MBX_Msg *pMsg, MBX_ROLE_ID eSrcRole);

#ifdef __cplusplus
}
#endif

#endif //_DRV_MBX_H

