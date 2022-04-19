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
/// @file   mdrv_temp.h
/// @brief  TEMP Driver Interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_SC_H_
#define _MDRV_SC_H_

#include <linux/fs.h>
#include <linux/cdev.h>
#include "mdrv_types.h"
#include <linux/version.h>

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#include <linux/spinlock.h>
#endif

//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define SC_FIFO_SIZE                512                                 // Rx fifo size

// #define SC_DEBUG
#ifdef SC_DEBUG
#define SC_PRINT(_fmt, _args...)    printk(KERN_WARNING "[%s][%d] " _fmt, __FUNCTION__, __LINE__, ## _args)
#define SC_ASSERT(_con) \
    do { \
        if (!(_con)) { \
            printk(KERN_CRIT "BUG at %s:%d assert(%s)\n", \
                    __FILE__, __LINE__, #_con); \
            BUG(); \
        } \
    } while (0)
#else
#define SC_PRINT(fmt, args...)
#define SC_ASSERT(arg)
#endif

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#define SC_ATR_LEN_MAX              33                                  ///< Maximum length of ATR
#define SC_MDB_CMD_DATA_LEN         10
#endif

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

typedef struct
{
    int                         s32Major;
    int                         s32Minor;
    struct cdev                 stCDev;
    struct file_operations      fops;
    struct fasync_struct        *async_queue; /* asynchronous readers */
} SC_DEV;

/// SmartCard Info
typedef struct
{
    BOOL                        bCardIn;                            ///Status care in
    BOOL                        bLastCardIn;
    U32                         u32CardStatus;
    wait_queue_head_t           stWaitQue;

    U8                          u8FifoRx[SC_FIFO_SIZE];
    U16                         u16FifoRxRead;
    U16                         u16FifoRxWrite;

    U8                          u8FifoTx[SC_FIFO_SIZE];
    U16                         u16FifoTxRead;
    U16                         u16FifoTxWrite;
    U32                         u32CardAttr;
    U32                         u32CwtRxErrorIndex;
    U32                         u32StartTimeRstToATR;

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
    spinlock_t                  lock;
#endif
} SC_Info;

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
/// SmartCard VCC control mode
typedef enum
{
    E_SC_VCC_CTRL_8024_ON,                                              ///< by external 8024 on
    E_SC_VCC_CTRL_LOW,                                                  ///< by direct Vcc (low active)
    E_SC_VCC_CTRL_HIGH,                                                 ///< by direct Vcc (high active)
    E_SC_OCP_VCC_HIGH,
    E_SC_VCC_VCC_ONCHIP_8024,
} SC_VccCtrl;

/// SmartCard CLK setting
typedef enum
{
    E_SC_CLK_3M,                                                        ///< 3 MHz
    E_SC_CLK_4P5M,                                                      ///< 4.5 MHz
    E_SC_CLK_6M,                                                        ///< 6 MHz
    E_SC_CLK_13M,                                                        ///< 6 MHz
    E_SC_CLK_4M,                                                        ///< 4 MHz
} SC_ClkCtrl;

// SmartCard mdebug info
#define SC_MDB_PAD_CONFIG_LEN         100
typedef struct __attribute__((packed))
{
    MS_U8                       u8Protocol;                         ///T= Protocol
    MS_U8                       pu8Atr[SC_ATR_LEN_MAX];             ///Atr buffer
    MS_U16                      u16Fi;                               ///Fi
    MS_U16                      u16Di;                               ///Di
    MS_U16                      u16AtrLen;                          ///Atr length
    MS_U16                      u16HistLen;                         ///History length
    MS_BOOL                     bOpened;                            ///Open
    MS_BOOL                     bCardIn;                            ///Status care in
    SC_VccCtrl                  eVccCtrl;
    SC_ClkCtrl                  eCardClk;                           ///< Clock
    MS_U8                       au8PadConf[SC_MDB_PAD_CONFIG_LEN];  ///Pad Config Setting
} SC_MdbgInfo;

typedef struct __attribute__((packed))
{
    MS_U32                      u32MdbCmd;
    MS_U8                       au8CmdData[SC_MDB_CMD_DATA_LEN];
} SC_MdbgCmdData;
#endif

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
int MDrv_SC_Open(struct inode *inode, struct file *filp);
ssize_t MDrv_SC_Read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t MDrv_SC_Write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
unsigned int MDrv_SC_Poll(struct file *filp, poll_table *wait);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int MDrv_SC_AttachInterrupt(struct file *filp, unsigned long arg);
int MDrv_SC_DetachInterrupt(struct file *filp, unsigned long arg);
int MDrv_SC_ResetFIFO(struct file *filp, unsigned long arg);
int MDrv_SC_GetEvent(struct file *filp, unsigned long arg);
int MDrv_SC_SetEvent(struct file *filp, unsigned long arg);
int MDrv_SC_GetAttribute(struct file *filp, unsigned long arg);
int MDrv_SC_SetAttribute(struct file *filp, unsigned long arg);
int MDrv_SC_CheckRstToATR(struct file *filp, unsigned long arg);
int MDrv_SC_GetCwtRxErrorIndex(struct file *filp, unsigned long arg);
int MDrv_SC_ResetRstToATR(struct file *filp, unsigned long arg);
int MDrv_SC_EnableIRQ(struct file *filp, unsigned long arg);
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
int mdb_sc_write_info(struct file *filp, unsigned long arg);
int mdb_sc_get_cmd_data(struct file *filp, unsigned long arg);
#endif
#else
int MDrv_SC_AttachInterrupt(struct inode *inode, struct file *filp, unsigned long arg);
int MDrv_SC_DetachInterrupt(struct inode *inode, struct file *filp, unsigned long arg);
int MDrv_SC_ResetFIFO(struct inode *inode, struct file *filp, unsigned long arg);
int MDrv_SC_GetEvent(struct inode *inode, struct file *filp, unsigned long arg);
int MDrv_SC_SetEvent(struct inode *inode, struct file *filp, unsigned long arg);
int MDrv_SC_GetAttribute(struct inode *inode, struct file *filp, unsigned long arg);
int MDrv_SC_SetAttribute(struct inode *inode, struct file *filp, unsigned long arg);
int MDrv_SC_CheckRstToATR(struct inode *inode, struct file *filp, unsigned long arg);
int MDrv_SC_GetCwtRxErrorIndex(struct inode *inode, struct file *filp, unsigned long arg);
int MDrv_SC_ResetRstToATR(struct inode *inode, struct file *filp, unsigned long arg);
int MDrv_SC_EnableIRQ(struct inode *inode, struct file *filp, unsigned long arg);
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
int mdb_sc_write_info(struct inode *inode, struct file *filp, unsigned long arg);
int mdb_sc_get_cmd_data(struct inode *inode, struct file *filp, unsigned long arg);
#endif
#endif

int KDrv_SC_Open(MS_U8 u8SCID);
ssize_t KDrv_SC_Read(MS_U8 u8SCID, char __user *buf, size_t count);
int KDrv_SC_AttachInterrupt(MS_U8 u8SCID);
int KDrv_SC_DetachInterrupt(MS_U8 u8SCID);
ssize_t KDrv_SC_Write(MS_U8 u8SCID, const char *buf, size_t count);
int KDrv_SC_Poll(MS_U8 u8SCID, MS_U32 u32TimeoutMs);
void KDrv_SC_ResetFIFO(MS_U8 u8SCID);
int KDrv_SC_GetEvent(MS_U8 u8SCID, MS_U32 *pu32Events);
int KDrv_SC_SetEvent(MS_U8 u8SCID, MS_U32 *pu32Events);
int KDrv_SC_GetAttribute(MS_U8 u8SCID, MS_U32 *pu32Attrs);
int KDrv_SC_SetAttribute(MS_U8 u8SCID, MS_U32 *pu32Attrs);
int KDrv_SC_CheckRstToATR(MS_U8 u8SCID, MS_U32 u32RstToAtrPeriod);
int KDrv_SC_GetCwtRxErrorIndex(MS_U8 u8SCID, MS_U32 *pu32CwtRxErrorIndex);
int KDrv_SC_ResetRstToATR(MS_U8 u8SCID);
int KDrv_SC_EnableIRQ(MS_U8 u8SCID);
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
int KDrv_SC_MdbWriteInfo(MS_U8 u8SCID, SC_MdbgInfo *pstInfo);
int KDrv_SC_GetCmdData(MS_U8 u8SCID, SC_MdbgCmdData *pstCmdData);
#endif

#endif // _MDRV_TEMP_H_

