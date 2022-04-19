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
/// @file   mdrv_mbx.h
/// @brief  MStar Mailbox Driver DDI
/// @author MStar Semiconductor Inc.
/// @attention
/// <b>(OBSOLETED) <em>legacy interface is only used by MStar proprietary Mail Message communication\n
/// It's API level for backward compatible and will be remove in the next version.\n
/// Please refer @ref drvGE.h for future compatibility.</em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _DRV_MBX_H
#define _DRV_MBX_H

#ifdef _DRV_MBX_C
#define INTERFACE
#else
#define INTERFACE extern
#endif

//=============================================================================
// Includs
//=============================================================================

//=============================================================================
// Defines & Macros
//=============================================================================
#define INVALID_PTR  (-1)
#define IS_VALID_PTR(ptr)  (ptr != INVALID_PTR)
#define INVALID_MBXCLASS(mbxClass)  ((mbxClass < E_MBX_CLASS_MIN) || (mbxClass >= E_MBX_CLASS_MAX))
#define INVALID_CPUID(cpuId)  ((cpuId < E_MBX_CPU_MIN) || (cpuId >= E_MBX_CPU_MAX))
#define INVALID_ROLEID(roleId)  ((roleId < E_MBX_ROLE_MIN) || (roleId >= E_MBX_ROLE_MAX))

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

/// the Index of Mailbox sent to 51
#define PM_CMDIDX_GLOBAL_CHIP_RESET 0x021

///the flag of Mailbox of reset in 51
#define GLOBAL_CHIPS_RESET_IN_51 0x01

/// Define FD ID for kernel mode usage
#define MBX_KERNEL_FD 99
/// Define trigger value for kernel mode usage (FALSE: will not trigger kill_fasync)
#define MBX_KERNEL_TRIGGER FALSE

#define MBX_ASYNCNOTIFIER_MAX CONFIG_MSTAR_MBX_ASYNC_NOTIFIER_SIZE
//=============================================================================
// Type and Structure Declaration
//=============================================================================
//=============================================================================
// Enums

/// Mail Message Class Defines
typedef enum
{
    E_MBX_CLASS_MIN            = 0,
    /// MBX_CLASS_SYSTEM
    E_MBX_CLASS_SYSTEM         = E_MBX_CLASS_MIN,
    /// MBX_CLASS_INPUT
    E_MBX_CLASS_INPUT          = 1,
    /// MBX_CLASS_TSP
    E_MBX_CLASS_TSP            = 2,
    /// MBX_CLASS_CHANNEL
    E_MBX_CLASS_CHANNEL        = 3,
    /// MBX_CLASS_MEMORY
    E_MBX_CLASS_MEMORY         = 4,
    /// MBX_CLASS_MPEG
    E_MBX_CLASS_MPEG           = 5,
    /// MBX_CLASS_VIDEO
    E_MBX_CLASS_VIDEO          = 6,
    /// MBX_CLASS_AUDIO
    E_MBX_CLASS_AUDIO          = 7,
    /// MBX_CLASS_MHEG5_STATE
    E_MBX_CLASS_MHEG5_STATE    = 8,
    /// MBX_CLASS_MVF
    E_MBX_CLASS_MVF            = 9,
    /// MBX_CLASS_GE
    E_MBX_CLASS_GE             = 10,
    /// MBX_CLASS_CI
    E_MBX_CLASS_CI            = 11,  //rename unused "MBX_CLASS_NUM" as "MBX_CLASS_CI" for CIMHEG usage
    /// MBX_CLASS_TEST
    E_MBX_CLASS_TEST           = 12,
    /// MBX_CLASS_MAD
    E_MBX_CLASS_MAD            = 13,

    /// MBX_CLASS_MPEG2_FS
    E_MBX_CLASS_MPEG2_FS       = 14,
    /// MBX_CLASS_MPEG2
    E_MBX_CLASS_MPEG2          = 15,
    /// MBX_CLASS_BMP
    E_MBX_CLASS_BMP            = 16,
    /// MBX_CLASS_PNG
    E_MBX_CLASS_PNG            = 17,
    /// MBX_CLASS_JPEG
    E_MBX_CLASS_JPEG           = 18,
    /// MBX_CLASS_MJPEG
    E_MBX_CLASS_MJPEG          = 19,
    /// MBX_CLASS_JPEG_ENCODER
    E_MBX_CLASS_JPEG_ENCODER   = 20,
    ///
    E_MBX_CLASS_VDPLAYER_FS    = 21,
    /// MBX_CLASS_VDPLAYER_FS
    E_MBX_CLASS_VDPLAYER       = 22,
    /// MBX_CLASS_RMPLAYER_FS
    E_MBX_CLASS_RMPLAYER_FS    = 23,
    /// MBX_CLASS_RMPLAYER
    E_MBX_CLASS_RMPLAYER       = 24,
    /// MBX_CLASS_TSPLAYER_FS
    E_MBX_CLASS_TSPLAYER_FS    = 25,
    /// MBX_CLASS_TSPLAYER
    E_MBX_CLASS_TSPLAYER       = 26,
    /// MBX_CLASS_LZSS
    E_MBX_CLASS_LZSS           = 27,
    /// MBX_CLASS_CAPE
    E_MBX_CLASS_CAPE           = 28,
    /// MBX_CLASS_CC
    E_MBX_CLASS_CC             = 29,
    /// MBX_CLASS_DLNA
    E_MBX_CLASS_DLNA           = 30,
    /// MBX_CLASS_DUMMY_LOOP
    E_MBX_CLASS_DUMMY_LOOP     = 31,
    /// MBX_CLASS_CHAKRA_SUBSYS
    E_MBX_CLASS_CHAKRA_SUBSYS  = 32,
    /// MBX_CLASS_FCNTL
    E_MBX_CLASS_FCNTL          = 33,
    /// MBX_CLASS_IRKEY
    E_MBX_CLASS_IRKEY          = 34,
    /// MBX_CLASS_BTPD
    E_MBX_CLASS_BTPD           = 35,
    /// MBX_CLASS_OBAMA_CMD
    E_MBX_CLASS_OBAMA_CMD      = 36,
    /// MBX_CLASS_OBAMA_APP
    E_MBX_CLASS_OBAMA_APP      = 37,
    /// MBX_CLASS_KTV
    E_MBX_CLASS_KTV             = 38,
    /// MBX_CLASS_CIPLUS
    E_MBX_CLASS_CIPLUS         = 39,
    /// MBX_CLASS_PRINT_MESSAGE
    E_MBX_CLASS_PRINT_MESSAGE = 40,
    /// MBX_CLASS_SHWFS
    E_MBX_CLASS_SHWFS          = 41,
    /// MBX_CLASS_AUTO_TEST
    E_MBX_CLASS_AUTO_TEST = 42,
    /// MBX_CLASS_STILLIMAGE_CMD_IO
    E_MBX_CLASS_STILLIMAGE_CMD_IO = 43,
    /// MBX_CLASS_STILLIMAGE_DATA_IO_FAST
    E_MBX_CLASS_STILLIMAGE_DATA_IO_FAST = 44,
    /// MBX_CLASS_MSTILLIMAGE_DATA_IO
    E_MBX_CLASS_STILLIMAGE_DATA_IO = 45,
    /// MBX_CLASS_NET_DEBUG
    E_MBX_CLASS_NET_DEBUG = 46,
    /// MBX_CLASS_PM_WAIT
    E_MBX_CLASS_PM_WAIT        = 47,
    /// MBX_CLASS_PM_NOWAIT
    E_MBX_CLASS_PM_NOWAIT      = 48,
    /// MBX_CLASS_IRKEY_NOWAIT
    E_MBX_CLASS_IRKEY_NOWAIT   = 49,
    /// MBX_CLASS_SAR
    E_MBX_CLASS_SAR            = 50,
    /// MBX_CLASS_SAR_NOWAIT
    E_MBX_CLASS_SAR_NOWAIT     = 51,
    /// MBX_CLASS_SECUREBOOT_WAIT
    E_MBX_CLASS_SECUREBOOT_WAIT     = 52,
    /// MBX_CLASS_SECUREBOOT_NOWAIT
    E_MBX_CLASS_SECUREBOOT_NOWAIT     = 53,
    /// MBX_CLASS_FRC
    E_MBX_CLASS_FRC = 54,
    //MBX_CLASS_PM_PWM_WAIT
    E_MBX_CLASS_PM_PWM_WAIT= 55,
    //MBX_CLASS_PM_PWM_NOWAIT
    E_MBX_CLASS_PM_PWM_NOWAIT= 56,
    //E_MBX_CLASS_DYNAMIC_ID
    E_MBX_CLASS_DYNAMIC_ID     = 57,
    //MBX_CLASS_QUERY_CLASS
    E_MBX_CLASS_QUERY_CLASS = 58,
    E_MBX_CLASS_CB_SECURE_WAIT =59,
    E_MBX_CLASS_CB_SECURE_NOWAIT = 60,
    //MBX_CLASS_CLEANBOOT_PNL_WAIT
    E_MBX_CLASS_CB_PNL_WAIT= 61,
    //MBX_CLASS_CLEANBOOT_PNL_NOWAIT
    E_MBX_CLASS_CB_PNL_NOWAIT= 62,
    E_MBX_CLASS_CB_LOGO_NOWAIT= 63,
    E_MBX_CLASS_CB_MUSIC_NOWAIT= 64,
    E_MBX_CLASS_CB_OSD_NOWAIT= 65,
    E_MBX_CLASS_CB_HEARTBEAT_NOWAIT= 66,
    E_MBX_CLASS_CB_CHECK_NOWAIT= 67,
    /// Define the Number of Mail Class, It is very Import, must be at the end of the list!
    //put the following two enum at the end of the list
    E_MBX_CLASS_NUM,
    /// The End NUmber of Max Class ID
    E_MBX_CLASS_MAX = 0xFF
}MBX_Class;

/// Mail Box Supported CPU ID Defins
typedef enum
{
   E_MBX_CPU_MIN = 0,
   /// CPU PM ID
   E_MBX_CPU_PM = E_MBX_CPU_MIN,
   /// CPU AEON ID
   E_MBX_CPU_AEON = 1,
   /// CPU MIPS ID
   E_MBX_CPU_MIPS = 2,
   //  CPU  R2M (same as MIPS)
   E_MBX_CPU_R2M = 2,
   /// CPU MIPS VPE0 ID, T3 Only
   E_MBX_CPU_MIPS_VPE0 = 2,
   /// CPU MIPS VPE1 ID, T3 Only
   E_MBX_CPU_MIPS_VPE1 = 3,
   #if 1//frcr2_integration###
   /// CPU  FRC-R2
   E_MBX_CPU_R2FRC = 4,
   #endif
   E_MBX_CPU_MAX,
}MBX_CPU_ID;

/// Mail Box Supported CPU ID Defins
typedef enum
{
    E_MBX_ROLE_MIN = 0,
#if defined(MSTAR_MBX_GROUP_PATH)
    /// House Keeping Identifier
    E_MBX_ROLE_HK = E_MBX_ROLE_MIN,
    /// Co-Processer Identifier
    E_MBX_ROLE_CP = 1,
    /// PM Identifier
    E_MBX_ROLE_AEON = 3,
    /// FRC
    E_MBX_ROLE_FRC = 4,
    E_MBX_ROLE_PM = 5,
    E_MBX_ROLE_MAX
#else
    /// House Keeping Identifier
    E_MBX_ROLE_HK = E_MBX_ROLE_MIN,
    /// Co-Processer Identifier
    E_MBX_ROLE_CP = 1,
    /// PM Identifier
    E_MBX_ROLE_PM = 2,
    E_MBX_ROLE_AEON = 3,
    /// FRC
    E_MBX_ROLE_FRC = 4,
    E_MBX_ROLE_MAX
#endif
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

#if defined(CONFIG_COMPAT)
typedef MS_U64  TYPE_MBX_C_U64;
#else
typedef MS_U32  TYPE_MBX_C_U64;
#endif

/// Mail Message Define
typedef struct  __attribute__((packed))
{
    /// Role ID, for send, this is the Mail Target, for receive, this is the Source which sent this message
    MBX_ROLE_ID      eRoleID;
    /// Mail Message Type, Normal or Instant
    MBX_MSG_Type     eMsgType;

    /// Ctrl Byte in Mail Message
    MS_U8              u8Ctrl;
    /// @ref MBX_Class
    MS_U8              u8MsgClass; //should be MBX_Class, for alignment let it be S8, gcc default sizeof(enum) is 4 bytes.
    /// Command Index defined by Apps
    MS_U8              u8Index;
    /// Parameter Count
    MS_U8              u8ParameterCount;
    /// Parameters, Max Number @ref MAX_MBX_PARAM_SIZE
    MS_U8              u8Parameters[MAX_MBX_PARAM_SIZE];
    /// Status 0 byte of Mail Message
    MS_U8              u8S0;
    /// Status 1 byte of Mail Message
    MS_U8              u8S1;
}MBX_Msg;

/// Mail Message Queue status
typedef struct  __attribute__((packed))
{
    /// Mail message Queue status,
    /// @ref MBX_STATUS_QUEUE_OVER_FLOW
    /// @ref MBX_STATUS_QUEUE_HAS_INSTANT_MSG
    /// @ref MBX_STATUS_QUEUE_HAS_NORMAL_MSG
    MS_U32 status;
    /// pended normal message count in class message queue
    MS_U32 u32NormalMsgCount;
    /// pended Instant message count in class message queue
    MS_U32 u32InstantMsgCount;
}MBX_MSGQ_Status;

typedef struct __attribute__((packed))
{
    MS_S16 s16MsgFirst; //the first MsgPoolItem slot idx in MsgPool
    MS_S16 s16MsgEnd; //the end MsgPoolItem slot idx in MsgPool

    MS_S16 s16InstantMsgFirst; //the first instant MsgPoolItem slot idx in MsgPool
    MS_S16 s16InstantMsgEnd; //the end instant MsgPoolItem slot idx in MsgPool

    MS_U16 u16MsgNum; //the number of MsgPoolItem in MsgQ;
    MS_U16 u16InstantMsgNum; //the number of instant MsgPoolItem in MsgQ;

    MS_U16 u16MsgQStatus; //the MsgQ Status: invalid; normal, empty; voerflow;
    MS_U16 u16MsgQSize; //the MsgQ size when the msg class is registed; should be no larger than MAX_MBX_QUEUE_SIZE

    MS_S16 s16MsgQNotifierID; //the MsgQ Register in MBX Driver
    MS_S16 s16NextMsgQ; //the Next MsgQ which belongs to the same Register;
}MSGPOOL_MsgQMgr; //message queue manager, per class per queue.

typedef struct  __attribute__((packed))
{
    TYPE_MBX_C_U64 u32AsyncID;
    struct fasync_struct  *async_queue; /* asynchronous readers */
    MS_S16 s16MsgQFirst; /* the first MsgQ Idx */
    MS_U16 u16Usage; /* the slot usage of notifier*/
    MS_BOOL bEnable; /* the register message queue is enabled or not */
    MSGPOOL_MsgQMgr msgQmgr[E_MBX_CLASS_MAX]; /* message queue manager per process */
    MS_BOOL bTrigger[E_MBX_CLASS_MAX];
} MBX_ASYNC_NOTIFIER;

#define MBX_DEV_FLAG_PM2HKINT 0x01
#define MBX_DEV_FLAG_HK2PMINT 0x02
typedef struct _MSTAR_MBX_DEV{
    unsigned long flag;
}MSTAR_MBX_DEV;
int MDrv_MBX_Suspend(MSTAR_MBX_DEV *pmbx_dev);
int MDrv_MBX_Resume(MSTAR_MBX_DEV *pmbx_dev);

typedef void (*MBX_QUEUE_STATUS_NOTIFY_FUNC)(MBX_Class eMsgClass, MBX_MSGQ_Status *pMsgQueueStatus);

INTERFACE MBX_Result  MDrv_MBX_Startup(void);
INTERFACE MBX_Result  MDrv_MBX_Exit(void);

INTERFACE MBX_Result  MDrv_MBX_Init(MBX_CPU_ID eHKCPU, MBX_ROLE_ID eHostRole, MS_U32 u32TimeoutMillSecs);
INTERFACE MBX_Result  MDrv_MBX_Init_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_CPU_ID eHKCPU, MBX_ROLE_ID eHostRole, MS_U32 u32TimeoutMillSecs);
INTERFACE MBX_Result  MDrv_MBX_DeInit(MS_BOOL bForceDiscardPendingMsg);
INTERFACE MBX_Result  MDrv_MBX_DeInit_Async(TYPE_MBX_C_U64 u32AsyncID, MS_BOOL bForceDiscardPendingMsg);
INTERFACE MBX_Result  MDrv_MBX_FASYNC(MS_S32 s32Fd, TYPE_MBX_C_U64 u32AsyncID, MS_S32 s32Mode);
INTERFACE MBX_Result  MDrv_MBX_ReleaseFASYNC(TYPE_MBX_C_U64 u32AsyncID);

INTERFACE MBX_Result  MDrv_MBX_RegisterMSG(MBX_Class eMsgClass, MS_U16 u16MsgQueueSize);
INTERFACE MBX_Result  MDrv_MBX_RegisterMSG_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Class eMsgClass, MS_U16 u16MsgQueueSize);
INTERFACE MBX_Result  MDrv_MBX_UnRegisterMSG(MBX_Class eMsgClass, MS_BOOL bForceDiscardMsgQueue);
INTERFACE MBX_Result  MDrv_MBX_UnRegisterMSG_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Class eMsgClass, MS_BOOL bForceDiscardMsgQueue);
INTERFACE MBX_Result  MDrv_MBX_ClearMSG(MBX_Class eMsgClass);
INTERFACE MBX_Result  MDrv_MBX_ClearMSG_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Class eMsgClass);
INTERFACE MBX_Result  MDrv_MBX_SendMsg(MBX_Msg *pMsg);
INTERFACE MBX_Result  MDrv_MBX_SendMsg_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Msg *pMsg, MS_U8 u8Trigger);
INTERFACE MBX_Result  MDrv_MBX_SendMsgLoopback(MBX_Msg *pMsg, MBX_ROLE_ID eSrcRoleId);
INTERFACE MBX_Result  MDrv_MBX_SendMsgLoopback_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Msg *pMsg, MBX_ROLE_ID eSrcRoleId);
INTERFACE MBX_Result  MDrv_MBX_RecvMsg(MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32WaitMillSecs, MS_U32 u32Flag);
INTERFACE MBX_Result  MDrv_MBX_RecvMsg_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32WaitMillSecs, MS_U32 u32Flag ,MS_U8 u8Trigger);
INTERFACE MBX_Result  MDrv_MBX_CheckMsg(MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32WaitMillSecs, MS_U32 u32Flag);
INTERFACE MBX_Result  MDrv_MBX_CheckMsg_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32WaitMillSecs, MS_U32 u32Flag);
INTERFACE MBX_Result  MDrv_MBX_GetMsgQueueStatus(MBX_Class eTargetClass, MBX_MSGQ_Status *pMsgQueueStatus);
INTERFACE MBX_Result  MDrv_MBX_GetMsgQueueStatus_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Class eTargetClass, MBX_MSGQ_Status *pMsgQueueStatus);
INTERFACE MBX_Result  MDrv_MBX_RemoveLatestMsg(TYPE_MBX_C_U64 u32AsyncID);

// For Application Set/Get Information API:
INTERFACE MBX_Result  MDrv_MBX_SetInformation(MBX_ROLE_ID eTargetRole, MS_U8 *pU8Info, MS_U8 u8Size);
INTERFACE MBX_Result  MDrv_MBX_SetInformation_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_ROLE_ID eTargetRole, MS_U8 *pU8Info, MS_U8 u8Size);
INTERFACE MBX_Result  MDrv_MBX_GetInformation(MBX_ROLE_ID eTargetRole, MS_U8 *pU8Info, MS_U8 u8Size);
INTERFACE MBX_Result  MDrv_MBX_GetInformation_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_ROLE_ID eTargetRole, MS_U8 *pU8Info, MS_U8 u8Size);

INTERFACE MBX_Result  MDrv_MBX_Enable(MS_BOOL bEnable);//Enable receiving regisited messages in kernel
INTERFACE MBX_Result  MDrv_MBX_Enable_Async(TYPE_MBX_C_U64 u32AsyncID, MS_BOOL bEnable);
INTERFACE MS_BOOL     MDrv_MBX_GetEnableStatus(TYPE_MBX_C_U64 u32AsyncID);

INTERFACE void  MDrv_MBX_NotifyPMtoSetPowerOff(void);
INTERFACE void  MDrv_MBX_NotifyPMtoSetPowerdown(void);

INTERFACE MS_U8 MDrv_PM_Get_BrickTerminator_Info(void);
INTERFACE void MDrv_PM_Set_BrickTerminator_Info(MS_U8 u8Value);

/* MDrv API sync with utpa */
typedef void (*MBX_MAIL_ARRIVE_NOTIFY_FUNC)( MBX_Msg *pMsg, MS_BOOL *pbAddToQueue);
INTERFACE MBX_Result  MDrv_MBX_RegisterMSGWithCallBack(MBX_Class eMsgClass, MS_U16 u16MsgQueueSize, MBX_MAIL_ARRIVE_NOTIFY_FUNC notifier);
INTERFACE void MDrv_MBX_SetDbgLevel(MS_U8 u8DbgLevel);
INTERFACE void MDrv_MBX_SetCallDrvFlag(MS_BOOL bEnable);
INTERFACE MS_BOOL MDrv_MBX_GetCallDrvFlag(void);

#undef INTERFACE
#endif //_DRV_MBX_H

