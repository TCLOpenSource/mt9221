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
/// file    mdrv_temp.c
/// @brief  TEMP Driver Interface
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
#include <linux/poll.h>
#include <linux/wait.h>
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/time.h>
#include <linux/delay.h>

//drver header files
#include "chip_int.h"
#include "mdrv_mstypes.h"
#include "reg_sc.h"
#include "mhal_sc.h"
#include "mdrv_sc.h"

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#endif

// For OPTEE
#include "chip_setup.h"
#include "mdrv_types.h"
#include "mdrv_tee_general.h"

//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define SC_MAX_CONT_SEND_LEN        (24)

//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------
typedef enum
{
    E_SC_INT_TX_LEVEL = 0x00000001,
    E_SC_INT_CARD_IN = 0x00000002, //UART_SCSR_INT_CARDIN
    E_SC_INT_CARD_OUT = 0x00000004, //UART_SCSR_INT_CARDOUT
    E_SC_INT_CGT_TX_FAIL = 0x00000008,
    E_SC_INT_CGT_RX_FAIL = 0x00000010,
    E_SC_INT_CWT_TX_FAIL = 0x00000020,
    E_SC_INT_CWT_RX_FAIL = 0x00000040,
    E_SC_INT_BGT_FAIL = 0x00000080,
    E_SC_INT_BWT_FAIL = 0x00000100,
    E_SC_INT_PE_FAIL = 0x00000200,
    E_SC_INT_RST_TO_ATR_FAIL = 0x00000400,
    E_SC_INT_MDB_CMD_DATA = 0x00000800,
    E_SC_INT_GET_FROM_OPTEE = 0x00001000,
    E_SC_INT_INVALID = 0xFFFFFFFF
}SC_INT_BIT_MAP;

typedef enum
{
    E_SC_ATTR_INVALID = 0x00000000,
    E_SC_ATTR_TX_LEVEL = 0x00000001,
    E_SC_ATTR_NOT_RCV_CWT_FAIL = 0x00000002,
    E_SC_ATTR_T0_PE_KEEP_RCV = 0x00000004,
    E_SC_ATTR_FAIL_TO_RST_LOW = 0x00000008,
    E_SC_ATTR_ENABLE_OPTEE = 0x00000010
}SC_ATTR_TYPE;

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
static SC_Info  _scInfo[SC_DEV_NUM] = {
    {
        .bCardIn        = FALSE,
        .bLastCardIn    = FALSE,
        .u32CardStatus  = 0,
        .u16FifoRxRead  = 0,
        .u16FifoRxWrite = 0,
        .u16FifoTxRead  = 0,
        .u16FifoTxWrite = 0,
        .u32CardAttr    = E_SC_ATTR_INVALID,
        .u32CwtRxErrorIndex = 0xFFFFFFFF,
        .u32StartTimeRstToATR = 0,
    },
#if (SC_DEV_NUM > 1) // no more than 2
    {
        .bCardIn        = FALSE,
        .bLastCardIn    = FALSE,
        .u32CardStatus  = 0,
        .u16FifoRxRead  = 0,
        .u16FifoRxWrite = 0,
        .u16FifoTxRead  = 0,
        .u16FifoTxWrite = 0,
        .u32CardAttr    = E_SC_ATTR_INVALID,
        .u32CwtRxErrorIndex = 0xFFFFFFFF,
        .u32StartTimeRstToATR = 0,
    }
#endif
};

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
static SC_MdbgInfo _scMdbInfo[SC_DEV_NUM] = {
    {
        .bOpened    = FALSE,
        .bCardIn    = FALSE,
        .u8Protocol = 0,
    },
#if (SC_DEV_NUM > 1) // no more than 2
    {
        .bOpened    = FALSE,
        .bCardIn    = FALSE,
        .u8Protocol = 0,
    },
#endif
};

static int mdb_sc_node_open(struct inode *inode, struct file *file);
static ssize_t mdb_sc_node_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos);

const struct file_operations mdb_sc_node_operations = {
    .owner      = THIS_MODULE,
    .open       = mdb_sc_node_open,
    .read       = seq_read,
    .write      = mdb_sc_node_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};

typedef enum {
    MDB_SC_CMD_UNKNOW = 0x00,
    MDB_SC_CMD_HELP,
    MDB_SC_CMD_SET_DBG_LV,
    MDB_SC_CMD_MAX
} MDB_SC_CMD;

#define MDB_CMD_NUM     (MDB_SC_CMD_MAX-1)
#define MBD_CMD_SIZE    16

struct mdb_sc_cmd {
    char command[MBD_CMD_SIZE];
    MDB_SC_CMD index;
};

static struct mdb_sc_cmd mdb_sc_cmd_table[MDB_CMD_NUM] = {
    {
        .command = "help",
        .index = MDB_SC_CMD_HELP,
    },
    {
        .command = "dbg_info",
        .index = MDB_SC_CMD_SET_DBG_LV,
    },
};

static SC_MdbgCmdData _scMdbCmdata[SC_DEV_NUM] = {
    {
        .u32MdbCmd  = 0,
    },
#if (SC_DEV_NUM > 1) // no more than 2
    {
        .u32MdbCmd  = 0,
    },
#endif
};
#endif

//-------------------------------------------------------------------------------------------------
//  Export
//-------------------------------------------------------------------------------------------------
EXPORT_SYMBOL(KDrv_SC_Open);
EXPORT_SYMBOL(KDrv_SC_Read);
EXPORT_SYMBOL(KDrv_SC_AttachInterrupt);
EXPORT_SYMBOL(KDrv_SC_DetachInterrupt);
EXPORT_SYMBOL(KDrv_SC_Write);
EXPORT_SYMBOL(KDrv_SC_Poll);
EXPORT_SYMBOL(KDrv_SC_ResetFIFO);
EXPORT_SYMBOL(KDrv_SC_GetEvent);
EXPORT_SYMBOL(KDrv_SC_SetEvent);
EXPORT_SYMBOL(KDrv_SC_GetAttribute);
EXPORT_SYMBOL(KDrv_SC_SetAttribute);
EXPORT_SYMBOL(KDrv_SC_CheckRstToATR);
EXPORT_SYMBOL(KDrv_SC_GetCwtRxErrorIndex);
EXPORT_SYMBOL(KDrv_SC_ResetRstToATR);
EXPORT_SYMBOL(KDrv_SC_EnableIRQ);
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
EXPORT_SYMBOL(KDrv_SC_MdbWriteInfo);
EXPORT_SYMBOL(KDrv_SC_GetCmdData);
#endif

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
irqreturn_t MDrv_SC_ISR1(int irq, void *devid);
irqreturn_t MDrv_SC_ISR2(int irq, void *devid);

static U32 _MDrv_SC_GetTimeUs(void)
{
    struct timeval stTime;
    U32 u32Val;

    do_gettimeofday(&stTime);

    u32Val = (U32)(stTime.tv_sec * 1000000 + stTime.tv_usec);

    return u32Val;
}

static int _MDrv_SC_Open(U8 u8SCID)
{
    SC_PRINT("%s is invoked\n", __FUNCTION__);

    init_waitqueue_head(&_scInfo[u8SCID].stWaitQue);

    return 0;
}

static ssize_t _MDrv_SC_Read(U8 u8SCID, char *buf, size_t count)
{
    ssize_t idx = 0;
    int IsUserSpace;

    if (_scInfo[u8SCID].u16FifoRxWrite == _scInfo[u8SCID].u16FifoRxRead)
        return idx;

    IsUserSpace = access_ok(VERIFY_WRITE, buf, count);

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
    spin_lock(&_scInfo[u8SCID].lock);
#endif

    for (idx = 0; idx < count; idx++)
    {
        if (_scInfo[u8SCID].u16FifoRxWrite == _scInfo[u8SCID].u16FifoRxRead)
            break;

        if (IsUserSpace)
            put_user(_scInfo[u8SCID].u8FifoRx[_scInfo[u8SCID].u16FifoRxRead++], (char __user *)&buf[idx]);
        else
            buf[idx] = _scInfo[u8SCID].u8FifoRx[_scInfo[u8SCID].u16FifoRxRead++];

        if (_scInfo[u8SCID].u16FifoRxRead == SC_FIFO_SIZE)
        {
            _scInfo[u8SCID].u16FifoRxRead = 0;
        }
    }

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
    _scMdbInfo[u8SCID].u16HistLen = count;
    spin_unlock(&_scInfo[u8SCID].lock);
#endif

    return idx;
}

static int _MDrv_SC_AttachInterrupt(U8 u8SCID)
{
    if (u8SCID == 0)
    {
        if (request_irq(SC_IRQ, MDrv_SC_ISR1, SA_INTERRUPT, "SC", NULL))
        {
            printk("[%s][%d] request_irq SC_IRQ error\n", __FUNCTION__, __LINE__);
            return 1;
        }
        else
        {
            printk("[%s][%d] request_irq SC_IRQ ok\n", __FUNCTION__, __LINE__);
            return 0;
        }
    }
#if (SC_DEV_NUM > 1) // no more than 2
    else if (u8SCID == 1)
    {
        if (request_irq(SC_IRQ2, MDrv_SC_ISR2, SA_INTERRUPT, "SC", NULL))
        {
            printk("[%s][%d] request_irq SC_IRQ2 error\n", __FUNCTION__, __LINE__);
            return 1;
        }
        else
        {
            printk("[%s][%d] request_irq SC_IRQ2 ok\n", __FUNCTION__, __LINE__);
            return 0;
        }
    }
#endif

    return 1;
}

static int _MDrv_SC_DetachInterrupt(U8 u8SCID)
{
    if (u8SCID == 0)
    {
        free_irq(SC_IRQ, NULL);
    }
#if (SC_DEV_NUM > 1) // no more than 2
    else if (u8SCID == 1)
    {
        free_irq(SC_IRQ2, NULL);
    }
#endif

    return 0;
}

static BOOL _MDrv_SC_CheckWqStatus(U8 u8SCID)
{
    BOOL bRetVal = FALSE;

    /*if (_scInfo[u8SCID].u16FifoRxRead != _scInfo[u8SCID].u16FifoRxWrite)
    {
        bRetVal = TRUE;
    }*/
    if (_scInfo[u8SCID].u32CardStatus)
    {
        bRetVal = TRUE;
    }

    return bRetVal;
}

static ssize_t _MDrv_SC_Write(U8 u8SCID, const char *buf, size_t count)
{
    ssize_t idx = 0;
    U32 tmp;
    U32 u32SendLen, u2Index;
    int IsUserSpace;

    if(u8SCID >= SC_DEV_NUM)
        return -EFAULT;

    IsUserSpace = access_ok(VERIFY_READ, buf, count);

    //Fill up SW TX FIFO
    for (idx = 0; idx < count; idx++)
    {
        if (IsUserSpace)
            get_user(_scInfo[u8SCID].u8FifoTx[_scInfo[u8SCID].u16FifoTxWrite], (char __user *)&buf[idx]);
        else
            _scInfo[u8SCID].u8FifoTx[_scInfo[u8SCID].u16FifoTxWrite] = buf[idx];
        //Fix u16FifoTxWrite out off size when u16FifoTxWrite reach SC_FIFO_SIZE and u16FifoTxRead is zero
        tmp = (_scInfo[u8SCID].u16FifoTxWrite + 1) % SC_FIFO_SIZE;
        if (tmp != _scInfo[u8SCID].u16FifoTxRead)
        {
            // Not overflow
            _scInfo[u8SCID].u16FifoTxWrite = tmp;
        }
        else
        {
            // overflow
            printk("[%s][%d] TX buffer Overflow\n", __FUNCTION__, __LINE__);
            break;
        }
    }

    //
    // If TX level attribute is set, then transmit TX data by TX level driven instead of TX buffer empty driven
    // This can avoid too large interval between 1st data byte and 2nd's
    //
    if (_scInfo[u8SCID].u32CardAttr & E_SC_ATTR_TX_LEVEL)
    {
        // To use tx level int in tx pkts send
        if (count > SC_MAX_CONT_SEND_LEN)
        {
            u32SendLen = SC_MAX_CONT_SEND_LEN;
        }
        else
        {
            u32SendLen = count;
        }
        for (u2Index = 0; u2Index < u32SendLen; u2Index++)
        {
            if (_scInfo[u8SCID].u16FifoTxRead == _scInfo[u8SCID].u16FifoTxWrite)
                break;

            SC_WRITE(u8SCID, UART_TX, _scInfo[u8SCID].u8FifoTx[_scInfo[u8SCID].u16FifoTxRead++]);
            if (_scInfo[u8SCID].u16FifoTxRead == SC_FIFO_SIZE)
            {
                _scInfo[u8SCID].u16FifoTxRead = 0;
            }
            else if (_scInfo[u8SCID].u16FifoTxRead == _scInfo[u8SCID].u16FifoTxWrite)
            {
                break;
            }
        }
    }
    else
    {
        if ((SC_READ(u8SCID, UART_LSR) & UART_LSR_THRE) &&
                (_scInfo[u8SCID].u16FifoTxRead != _scInfo[u8SCID].u16FifoTxWrite))
        {
            SC_WRITE(u8SCID, UART_TX, _scInfo[u8SCID].u8FifoTx[_scInfo[u8SCID].u16FifoTxRead++]);
            if (_scInfo[u8SCID].u16FifoTxRead == SC_FIFO_SIZE)
            {
                _scInfo[u8SCID].u16FifoTxRead = 0;
            }

        }
    }

    return idx;
}

static void _MDrv_SC_ResetFIFO(U8 u8SCID)
{
    _scInfo[u8SCID].u16FifoRxRead   = 0;
    _scInfo[u8SCID].u16FifoRxWrite  = 0;
    _scInfo[u8SCID].u16FifoTxRead   = 0;
    _scInfo[u8SCID].u16FifoTxWrite  = 0;
}

static int _MDrv_SC_CheckRstToATR(U8 u8SCID, U32 u32RstToAtrPeriod)
{
    U32 u32DiffTime;
    U32 u32StartTime = _scInfo[u8SCID].u32StartTimeRstToATR;

    u32DiffTime = _MDrv_SC_GetTimeUs();
    if (u32DiffTime < u32StartTime)
    {
        u32DiffTime = (0xFFFFFFFFUL - u32StartTime) + u32DiffTime;
    }
    else
    {
        u32DiffTime = u32DiffTime - u32StartTime;
    }

    while (u32DiffTime <= u32RstToAtrPeriod)
    {
        udelay(10);

        u32DiffTime = _MDrv_SC_GetTimeUs();
        if (u32DiffTime < u32StartTime)
        {
            u32DiffTime = (0xFFFFFFFFUL - u32StartTime) + u32DiffTime;
        }
        else
        {
            u32DiffTime = u32DiffTime - u32StartTime;
        }

        if (_scInfo[u8SCID].u16FifoRxRead != _scInfo[u8SCID].u16FifoRxWrite)
            break;
    }

    if (u32DiffTime > u32RstToAtrPeriod)
    {
        if (_scInfo[u8SCID].u32CardAttr & E_SC_ATTR_FAIL_TO_RST_LOW)
            HAL_SC_ResetPadCtrl(u8SCID, FALSE);
        _scInfo[u8SCID].u32CardStatus |= E_SC_INT_RST_TO_ATR_FAIL;
    }

    return 0;
}

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
static void _MDrv_SC_MdbWriteInfo(MS_U8 u8SCID, SC_MdbgInfo *pstInfo)
{
    _scMdbInfo[u8SCID].bOpened = pstInfo->bOpened;
    _scMdbInfo[u8SCID].bCardIn = pstInfo->bCardIn;
    _scMdbInfo[u8SCID].u16Fi = pstInfo->u16Fi;
    _scMdbInfo[u8SCID].u16Di = pstInfo->u16Di;
    _scMdbInfo[u8SCID].u8Protocol = pstInfo->u8Protocol;
    _scMdbInfo[u8SCID].eVccCtrl = pstInfo->eVccCtrl;
    _scMdbInfo[u8SCID].eCardClk = pstInfo->eCardClk;
    _scMdbInfo[u8SCID].u16AtrLen = pstInfo->u16AtrLen;
    memcpy(_scMdbInfo[u8SCID].pu8Atr, pstInfo->pu8Atr, _scMdbInfo[u8SCID].u16AtrLen);
    memcpy(_scMdbInfo[u8SCID].au8PadConf, pstInfo->au8PadConf, sizeof(_scMdbInfo[u8SCID].au8PadConf));
}
#endif

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
int KDrv_SC_Open(MS_U8 u8SCID)
{
    return _MDrv_SC_Open((U8)u8SCID);
}

ssize_t KDrv_SC_Read(MS_U8 u8SCID, char *buf, size_t count)
{
    return _MDrv_SC_Read((U8)u8SCID, buf, count);
}

int KDrv_SC_AttachInterrupt(MS_U8 u8SCID)
{
    return _MDrv_SC_AttachInterrupt((U8)u8SCID);
}

int KDrv_SC_DetachInterrupt(MS_U8 u8SCID)
{
    return _MDrv_SC_DetachInterrupt((U8)u8SCID);
}

ssize_t KDrv_SC_Write(MS_U8 u8SCID, const char *buf, size_t count)
{
    return _MDrv_SC_Write((U8)u8SCID, buf, count);
}

int KDrv_SC_Poll(MS_U8 u8SCID, MS_U32 u32TimeoutMs)
{
    int err = 0;
    unsigned long timeout = msecs_to_jiffies(u32TimeoutMs);

    err = wait_event_interruptible_timeout(_scInfo[u8SCID].stWaitQue, _MDrv_SC_CheckWqStatus((U8)u8SCID), timeout);

    if (err < 0)
        return err;

    if (err > 0)
        return 0;
    else
        return -ETIMEDOUT;
}

void KDrv_SC_ResetFIFO(MS_U8 u8SCID)
{
    _MDrv_SC_ResetFIFO((U8)u8SCID);
}

int KDrv_SC_GetEvent(MS_U8 u8SCID, MS_U32 *pu32Events)
{
    *pu32Events = (MS_U32)_scInfo[u8SCID].u32CardStatus;
    _scInfo[u8SCID].u32CardStatus = 0;

    return 0;
}

int KDrv_SC_SetEvent(MS_U8 u8SCID, MS_U32 *pu32Events)
{
    _scInfo[u8SCID].u32CardStatus = *pu32Events;

    return 0;
}

int KDrv_SC_GetAttribute(MS_U8 u8SCID, MS_U32 *pu32Attrs)
{
    *pu32Attrs = (MS_U32)_scInfo[u8SCID].u32CardAttr;

    return 0;
}


int KDrv_SC_SetAttribute(MS_U8 u8SCID, MS_U32 *pu32Attrs)
{
    _scInfo[u8SCID].u32CardAttr = *pu32Attrs;

    return 0;
}

int KDrv_SC_CheckRstToATR(MS_U8 u8SCID, MS_U32 u32RstToAtrPeriod)
{
    return _MDrv_SC_CheckRstToATR((U8)u8SCID, (U32)u32RstToAtrPeriod);
}

int KDrv_SC_GetCwtRxErrorIndex(MS_U8 u8SCID, MS_U32 *pu32CwtRxErrorIndex)
{
    *pu32CwtRxErrorIndex = _scInfo[u8SCID].u32CwtRxErrorIndex;

    return 0;
}

int KDrv_SC_ResetRstToATR(MS_U8 u8SCID)
{
    _scInfo[u8SCID].u32StartTimeRstToATR = _MDrv_SC_GetTimeUs();

    return 0;
}

int KDrv_SC_EnableIRQ(MS_U8 u8SCID)
{
    if (u8SCID == 0)
    {
        enable_irq(SC_IRQ);
    }
    #if (SC_DEV_NUM > 1) // no more than 2
    if (u8SCID == 1)
    {
        enable_irq(SC_IRQ2);
    }
    #endif

    return 0;
}

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
int KDrv_SC_MdbWriteInfo(MS_U8 u8SCID, SC_MdbgInfo *pstInfo)
{
    if(u8SCID >= SC_DEV_NUM)
        return -EFAULT;

    _MDrv_SC_MdbWriteInfo(u8SCID, pstInfo);

    return 0;
}

int KDrv_SC_GetCmdData(MS_U8 u8SCID, SC_MdbgCmdData *pstCmdData)
{
    if(u8SCID >= SC_DEV_NUM)
        return -EFAULT;

    memcpy(pstCmdData, &_scMdbCmdata[u8SCID], sizeof(SC_MdbgCmdData));

    return 0;
}
#endif

BOOL MDrv_SC_ISR_Proc(U8 u8SCID)
{
    HAL_SC_TX_LEVEL_GWT_INT stTxLevelGWT_Int;
    U8  u8Reg, u8TxLevel = 0;
    U32 cnt;
    U32 idx;
    BOOL bWakeUp = FALSE;
    BOOL bTxLvChk = FALSE;

    if(u8SCID >= SC_DEV_NUM)
        return FALSE;

    if((TEEINFO_TYPTE == SECURITY_TEEINFO_OSTYPE_OPTEE) && (_scInfo[u8SCID].u32CardAttr & E_SC_ATTR_ENABLE_OPTEE))
    {
        if (u8SCID == 0)
        {
            disable_irq_nosync(SC_IRQ);
        }
        #if (SC_DEV_NUM > 1) // no more than 2
        if (u8SCID == 1)
        {
            disable_irq_nosync(SC_IRQ2);
        }
        #endif
        _scInfo[u8SCID].u32CardStatus |= E_SC_INT_GET_FROM_OPTEE;
        bWakeUp = TRUE;
    }
    else
    {
        // Try to get timing fail flag
        if (HAL_SC_GetIntTxLevelAndGWT(u8SCID, &stTxLevelGWT_Int))
        {
            if (stTxLevelGWT_Int.bTxLevelInt || stTxLevelGWT_Int.bCGT_TxFail ||
                stTxLevelGWT_Int.bCGT_RxFail || stTxLevelGWT_Int.bCWT_TxFail ||
                stTxLevelGWT_Int.bCWT_RxFail || stTxLevelGWT_Int.bBGT_Fail || stTxLevelGWT_Int.bBWT_Fail)
            {
                if (stTxLevelGWT_Int.bTxLevelInt)
                {
                    _scInfo[u8SCID].u32CardStatus |= E_SC_INT_TX_LEVEL;
                }
                if (stTxLevelGWT_Int.bCWT_RxFail)  //CWT RX INT
                {
                    _scInfo[u8SCID].u32CardStatus |= E_SC_INT_CWT_RX_FAIL;
                    _scInfo[u8SCID].u32CwtRxErrorIndex = (U32)_scInfo[u8SCID].u16FifoRxWrite;
                }
                if (stTxLevelGWT_Int.bCWT_TxFail)  //CWT TX INT
                {
                    _scInfo[u8SCID].u32CardStatus |= E_SC_INT_CWT_TX_FAIL;
                }
                if (stTxLevelGWT_Int.bCGT_RxFail)  //CGT RX INT
                {
                    _scInfo[u8SCID].u32CardStatus |= E_SC_INT_CGT_RX_FAIL;
                }
                if (stTxLevelGWT_Int.bCGT_TxFail)  //CGT TX INT
                {
                    _scInfo[u8SCID].u32CardStatus |= E_SC_INT_CGT_TX_FAIL;
                }
                if (stTxLevelGWT_Int.bBGT_Fail)  //BGT INT
                {
                    _scInfo[u8SCID].u32CardStatus |= E_SC_INT_BGT_FAIL;
                }
                if (stTxLevelGWT_Int.bBWT_Fail)  //BWT INT
                {
                    HAL_SC_RstToIoEdgeDetCtrl(u8SCID, FALSE);
                    if (_scInfo[u8SCID].u32CardAttr & E_SC_ATTR_FAIL_TO_RST_LOW)
                        HAL_SC_ResetPadCtrl(u8SCID, FALSE);
                    _scInfo[u8SCID].u32CardStatus |= E_SC_INT_BWT_FAIL;
                }

                // Clear int flag
                HAL_SC_ClearIntTxLevelAndGWT(u8SCID);

                //
                // To prevent tx level int be cleared due to CGWT/BGWT, we fill up tx FIFO for below case by enable bTxLvChk
                // 1. Tx FIFO count is less than Tx Level
                // 2. SW Tx FIFO R/W index is not the same, that means there is data in SW Tx FIFO need to be sent
                //
                u8TxLevel = HAL_SC_GetTxLevel(u8SCID);
                u8Reg = HAL_SC_GetTxFifoCnt(u8SCID);
                if (u8Reg <= u8TxLevel)
                {
                    if (_scInfo[u8SCID].u16FifoTxRead != _scInfo[u8SCID].u16FifoTxWrite)
                    {
                        bTxLvChk = TRUE;
                        SC_PRINT("To enable bTxLvChk, 0x%X\n", u8Reg);
                    }
                }

                bWakeUp = TRUE;

                if (stTxLevelGWT_Int.bTxLevelInt || bTxLvChk)
                {
                    cnt = 0;
                    while(1)
                    {
                        if (_scInfo[u8SCID].u16FifoTxRead == _scInfo[u8SCID].u16FifoTxWrite)
                            break;

                        // To avoid Tx FIFO overflow
                        u8TxLevel = HAL_SC_GetTxLevel(u8SCID);
                        if (HAL_SC_GetTxFifoCnt(u8SCID) >= (u8TxLevel+3))
                        {
                            SC_PRINT("To ignore TX this time\n");
                            break;
                        }

                        SC_WRITE(u8SCID, UART_TX, _scInfo[u8SCID].u8FifoTx[_scInfo[u8SCID].u16FifoTxRead++]);
                        cnt++;
                        if (_scInfo[u8SCID].u16FifoTxRead == SC_FIFO_SIZE)
                        {
                            _scInfo[u8SCID].u16FifoTxRead = 0;
                        }
                        else if (_scInfo[u8SCID].u16FifoTxRead == _scInfo[u8SCID].u16FifoTxWrite)
                        {
                            break;
                        }
                        else
                        {
                            if (cnt >= 16)
                                break;
                        }
                    }
                }
            }
        }

        u8Reg = SC_READ(u8SCID, UART_IIR);
        if (!(u8Reg & UART_IIR_NO_INT))
        {
            u8Reg = HAL_SC_GetLsr(u8SCID);
            while (u8Reg & (UART_LSR_DR | UART_LSR_BI))
            {
                bWakeUp = TRUE;
                _scInfo[u8SCID].u8FifoRx[_scInfo[u8SCID].u16FifoRxWrite] = SC_READ(u8SCID, UART_RX);

                if ((_scInfo[u8SCID].u32CardStatus & E_SC_INT_CWT_RX_FAIL) &&
                    (_scInfo[u8SCID].u32CardAttr & E_SC_ATTR_NOT_RCV_CWT_FAIL))
                {
                    // Do nothing for CWT fail
                }
                else
                {
                    if (u8Reg & UART_LSR_PE)
                    {
                        _scInfo[u8SCID].u32CardStatus |= E_SC_INT_PE_FAIL;
                        if (_scInfo[u8SCID].u32CardAttr & E_SC_ATTR_T0_PE_KEEP_RCV)//for conax
                        {
                            u8Reg = HAL_SC_GetLsr(u8SCID);
                            continue;
                        }
                        else
                        {
                            break;
                        }
                    }
                    //Fix u16FifoRxWrite out off size when u16FifoRxWrite reach SC_FIFO_SIZE and u16FifoRxRead is zero
                    idx = (_scInfo[u8SCID].u16FifoRxWrite + 1) % SC_FIFO_SIZE;
                    if (idx != _scInfo[u8SCID].u16FifoRxRead)
                    {
                        // Not overflow
                        _scInfo[u8SCID].u16FifoRxWrite = idx;
                    }
                    else
                    {
                        // overflow
                        printk("[%s][%d] RX buffer Overflow, maybe ATR Failed or not call _SC_Read\n", __FUNCTION__, __LINE__);
                        break;
                    }
                }

                u8Reg = HAL_SC_GetLsr(u8SCID);
            }

            if (u8Reg & UART_LSR_THRE)
            {
                cnt = 16;
                do
                {
                    if (_scInfo[u8SCID].u16FifoTxRead == _scInfo[u8SCID].u16FifoTxWrite)
                        break;

                    bWakeUp = TRUE;
                    SC_WRITE(u8SCID, UART_TX, _scInfo[u8SCID].u8FifoTx[_scInfo[u8SCID].u16FifoTxRead++]);
                    if (_scInfo[u8SCID].u16FifoTxRead == SC_FIFO_SIZE)
                    {
                        _scInfo[u8SCID].u16FifoTxRead = 0;
                    }

                } while (--cnt > 0);
            }
        }

        // Check special event from SMART
        u8Reg = SC_READ(u8SCID, UART_SCSR);
        if (u8Reg & (UART_SCSR_INT_CARDIN | UART_SCSR_INT_CARDOUT))
        {
            SC_WRITE(u8SCID, UART_SCSR, u8Reg); // clear interrupt
            _scInfo[u8SCID].u32CardStatus |= u8Reg & (UART_SCSR_INT_CARDIN | UART_SCSR_INT_CARDOUT);
            bWakeUp = TRUE;
        }

        //Check HW Rst to IO fail
        if (HAL_SC_CheckIntRstToIoEdgeFail(u8SCID))
        {
            HAL_SC_MaskIntRstToIoEdgeFail(u8SCID); //Mask int
            _scInfo[u8SCID].u32CardStatus |= E_SC_INT_RST_TO_ATR_FAIL;
            bWakeUp = TRUE;
        }
    }


    if (bWakeUp)
    {
        wake_up_interruptible(&_scInfo[u8SCID].stWaitQue);
    }

    return TRUE; // handled
}

//-------------------------------------------------------------------------------------------------
/// Handle smart card Interrupt notification handler
/// @param  irq             \b IN: interrupt number
/// @param  devid           \b IN: device id
/// @return IRQ_HANDLED
/// @attention
//-------------------------------------------------------------------------------------------------
irqreturn_t MDrv_SC_ISR1(int irq, void *devid)
{
    if (!MDrv_SC_ISR_Proc(0))
    {
        SC_PRINT("ISR proc is failed\n");
    }

    return IRQ_HANDLED;
}

//-------------------------------------------------------------------------------------------------
/// Handle smart card Interrupt notification handler
/// @param  irq             \b IN: interrupt number
/// @param  devid           \b IN: device id
/// @return IRQ_HANDLED
/// @attention
//-------------------------------------------------------------------------------------------------
irqreturn_t MDrv_SC_ISR2(int irq, void *devid)
{
    if (!MDrv_SC_ISR_Proc(1))
    {
        SC_PRINT("ISR proc is failed\n");
    }

    return IRQ_HANDLED;
}

int MDrv_SC_Open(struct inode *inode, struct file *filp)
{
    U8 u8SCID = (U8)(int)filp->private_data;

    return _MDrv_SC_Open(u8SCID);
}

ssize_t MDrv_SC_Read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    U8 u8SCID = (U8)(int)filp->private_data;

    return _MDrv_SC_Read(u8SCID, buf, count);
}

ssize_t MDrv_SC_Write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    U8 u8SCID = (U8)(int)filp->private_data;

    return _MDrv_SC_Write(u8SCID, buf, count);
}

unsigned int MDrv_SC_Poll(struct file *filp, poll_table *wait)
{
    U8 u8SCID = (U8)(int)filp->private_data;
    unsigned int mask = 0;

    poll_wait(filp, &_scInfo[u8SCID].stWaitQue, wait);
    if (_scInfo[u8SCID].u16FifoRxRead != _scInfo[u8SCID].u16FifoRxWrite)
    {
        mask |= POLLIN;
    }
    if (_scInfo[u8SCID].u32CardStatus)
    {
        mask |= POLLPRI;
    }

    return mask;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int MDrv_SC_AttachInterrupt(struct file *filp, unsigned long arg)
#else
int MDrv_SC_AttachInterrupt(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    U8 u8SCID = (U8)(int)filp->private_data;

    SC_PRINT("%s is invoked\n", __FUNCTION__);

    return _MDrv_SC_AttachInterrupt(u8SCID);
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int MDrv_SC_DetachInterrupt(struct file *filp, unsigned long arg)
#else
int MDrv_SC_DetachInterrupt(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    U8 u8SCID = (U8)(int)filp->private_data;

    SC_PRINT("%s is invoked\n", __FUNCTION__);

    return _MDrv_SC_DetachInterrupt(u8SCID);
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int MDrv_SC_ResetFIFO(struct file *filp, unsigned long arg)
#else
int MDrv_SC_ResetFIFO(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    U8 u8SCID = (U8)(int)filp->private_data;

    SC_PRINT("%s is invoked\n", __FUNCTION__);
    _MDrv_SC_ResetFIFO(u8SCID);

    return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int MDrv_SC_GetEvent(struct file *filp, unsigned long arg)
#else
int MDrv_SC_GetEvent(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    U8 u8SCID = (U8)(int)filp->private_data;

    put_user(_scInfo[u8SCID].u32CardStatus, (int __user *)arg);
    _scInfo[u8SCID].u32CardStatus = 0;

    return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int MDrv_SC_SetEvent(struct file *filp, unsigned long arg)
#else
int MDrv_SC_SetEvent(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    U8 u8SCID = (U8)(int)filp->private_data;

    get_user(_scInfo[u8SCID].u32CardStatus, (int __user *)arg);

    return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int MDrv_SC_GetAttribute(struct file *filp, unsigned long arg)
#else
int MDrv_SC_GetAttribute(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    U8 u8SCID = (U8)(int)filp->private_data;

    put_user(_scInfo[u8SCID].u32CardAttr, (int __user *)arg);

    return 0;
}


#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int MDrv_SC_SetAttribute(struct file *filp, unsigned long arg)
#else
int MDrv_SC_SetAttribute(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    U8 u8SCID = (U8)(int)filp->private_data;

    get_user(_scInfo[u8SCID].u32CardAttr, (int __user *)arg);

    return 0;
}

////////////////////////////
//MDrv_SC_CheckRstToATR is a SW patch function for rst_to_io detect
////////////////////////////
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int MDrv_SC_CheckRstToATR(struct file *filp, unsigned long arg)
#else
int MDrv_SC_CheckRstToATR(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    U8 u8SCID = (U8)(int)filp->private_data;
    U32 u32RstToAtrPeriod;

    get_user(u32RstToAtrPeriod, (int __user *)arg);

    return _MDrv_SC_CheckRstToATR(u8SCID, u32RstToAtrPeriod);
}

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int mdb_sc_write_info(struct file *filp, unsigned long arg)
#else
int mdb_sc_write_info(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    int err = 0;
    size_t u8SCID = (size_t)filp->private_data;
    SC_MdbgInfo info;

    if (copy_from_user(&info, (int __user *)arg, sizeof(SC_MdbgInfo)))
    {
        printk("[SC] %s: Failed to get info data from user\n", __FUNCTION__);
        err = -EFAULT;
        goto exit;
    }

    if(info.u16AtrLen > SC_ATR_LEN_MAX || info.u16AtrLen == 0)
    {
        printk("[SC] %s: Failed to get ATR lens from user\n", __FUNCTION__);
        err = -EFAULT;
        goto exit;
    }

    _MDrv_SC_MdbWriteInfo(u8SCID, &info);

exit:
    return err;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int mdb_sc_get_cmd_data(struct file *filp, unsigned long arg)
#else
int mdb_sc_get_cmd_data(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    U8 u8SCID = (U8)(int)filp->private_data;

    if(u8SCID >= SC_DEV_NUM)
        return -EFAULT;

    if (copy_to_user((int __user *)arg, &_scMdbCmdata[u8SCID], sizeof(SC_MdbgCmdData)))
    {
        printk("[SC] %s: Failed to copy cmd data to user\n", __FUNCTION__);
        return -EFAULT;
    }

    return 0;
}

static void mdb_sc_print_ctrl_type(struct seq_file *m, MS_U8 u8SCID)
{
    seq_printf(m, "%15s : ", "Control type");
    switch(_scMdbInfo[u8SCID].eVccCtrl)
    {
        case E_SC_VCC_CTRL_8024_ON:
            seq_printf(m, "%s", "8024\n");
        break;
        case E_SC_VCC_CTRL_LOW:
            seq_printf(m, "%s", "Active low\n");
        break;
        case E_SC_VCC_CTRL_HIGH:
            seq_printf(m, "%s", "Active high\n");
        break;
        case E_SC_OCP_VCC_HIGH:
            seq_printf(m, "%s", "ocp high\n");
        break;
        case E_SC_VCC_VCC_ONCHIP_8024:
            seq_printf(m, "%s", "OnChip 8024\n");
        break;
        default:
            seq_printf(m, "%s", "Unknow\n");
    }
}

static void mdb_sc_print_buffer(struct seq_file *m, char *name, MS_U8 *buf, MS_U32 len)
{
    MS_U8 i;

    seq_printf(m, "%15s :", name);
    for (i = 0; i < len; i++)
    {
        seq_printf(m, " 0x%02x", buf[i]);
        if (((i+1)%16) == 0)
        {
            seq_printf(m, "\n%16s ", " ");
        }
    }
    seq_printf(m, "\n");
}

static void mdb_sc_print_rxfifo(struct seq_file *m, MS_U8 u8SCID, MS_U32 len)
{
    MS_U32 i;
    MS_U32 index = 0;

    spin_lock(&_scInfo[u8SCID].lock);

    seq_printf(m, "%15s :", "History buffer");
    if (len < _scInfo[u8SCID].u16FifoRxRead)
        index = _scInfo[u8SCID].u16FifoRxRead - len;
    else
        index = SC_FIFO_SIZE - (len - _scInfo[u8SCID].u16FifoRxRead);

    for (i = 0; i < len; i++)
    {
        if (index == SC_FIFO_SIZE)
            index = 0;

        seq_printf(m, " 0x%02x", _scInfo[u8SCID].u8FifoRx[index]);
        if (((i+1)%16) == 0)
        {
            seq_printf(m, "\n%16s ", " ");
        }
        index++;
    }
    seq_printf(m, "\n");

    spin_unlock(&_scInfo[u8SCID].lock);
}

static void mdb_sc_print_clk_type(struct seq_file *m, MS_U8 u8SCID)
{
    seq_printf(m, "%15s : ", "Clock");
    switch(_scMdbInfo[u8SCID].eCardClk)
    {
        case E_SC_CLK_3M:
            seq_printf(m, "%s", "3M\n");
            break;
        case E_SC_CLK_4P5M:
            seq_printf(m, "%s", "4P5M\n");
            break;
        case E_SC_CLK_6M:
            seq_printf(m, "%s", "6M\n");
            break;
        case E_SC_CLK_13M:
            seq_printf(m, "%s", "13M\n");
            break;
        case E_SC_CLK_4M:
            seq_printf(m, "%s", "4M\n");
            break;
        default:
            seq_printf(m, "%s", "Unknow\n");
    }
}

static int mdb_sc_node_show(struct seq_file *m, void *v)
{
    MS_U8 i;

    seq_printf(m, "---------MStar SmartCard Info---------\n");

    for (i = 0; i < SC_DEV_NUM; i++)
    {
        seq_printf(m, "SMC %d:\n", i);

        if (_scMdbInfo[i].bOpened == TRUE)
            seq_printf(m, "%15s : %s", "Enable", "True\n");
        else
            seq_printf(m, "%15s : %s", "Enable", "False\n");

        if (_scMdbInfo[i].bCardIn == TRUE)
            seq_printf(m, "%15s : %s", "Status", "Card in\n");
        else
            seq_printf(m, "%15s : %s", "Status", "Card out\n");

         mdb_sc_print_ctrl_type(m, i);

         seq_printf(m, "%15s : %s\n", "Pad config", _scMdbInfo[i].au8PadConf);
         seq_printf(m, "%15s : T=%u\n", "Protocol", _scMdbInfo[i].u8Protocol);

        if (_scMdbInfo[i].bCardIn == TRUE)
         mdb_sc_print_buffer(m, "ATR", _scMdbInfo[i].pu8Atr,  _scMdbInfo[i].u16AtrLen);
        else
            mdb_sc_print_buffer(m, "ATR", _scMdbInfo[i].pu8Atr,  0);

        if (_scMdbInfo[i].bCardIn == TRUE)
            mdb_sc_print_rxfifo(m, i, _scMdbInfo[i].u16HistLen);
        else
            mdb_sc_print_rxfifo(m, i, 0);

         mdb_sc_print_clk_type(m, i);

         seq_printf(m, "%15s : %u\n", "Fi", _scMdbInfo[i].u16Fi);
         seq_printf(m, "%15s : %u\n", "Di", _scMdbInfo[i].u16Di);
         seq_printf(m, "\n");
    }

    return 0;
}

static int mbd_sc_cmd_handler(char *cmd)
{
    MS_U8 i;
    char acTmpCmd[MBD_CMD_SIZE];
    char *pCur = acTmpCmd;
    char *pTmp = NULL;
    char *pNext = NULL;

    strcpy(pCur, cmd);
    pTmp = strpbrk (pCur, " ");

    if (pTmp != NULL)
    {
        *pTmp = '\0';
        pNext = pTmp + 1;
    }

    for (i = 0; i < MDB_CMD_NUM; i++)
    {
        if (strcmp(pCur, mdb_sc_cmd_table[i].command) == 0)
            break;
    }

    if (i == MDB_CMD_NUM)
        return -EINVAL;

    switch(mdb_sc_cmd_table[i].index)
    {
        case MDB_SC_CMD_HELP:
            printk("*************** echo help ***************\n");
            printk("1. Control debug message level\n");
            printk("   Command: dbg_info [dbg_level]\n");
            printk("            dbg_level : ERR, INFO, ALL\n");
            printk("*****************************************\n");
            break;

        case MDB_SC_CMD_SET_DBG_LV:
            pCur = pNext;
            for (i = 0; i < SC_DEV_NUM; i++)
            {
                _scMdbCmdata[i].u32MdbCmd = (MS_U32)MDB_SC_CMD_SET_DBG_LV;
                if(pCur != NULL)
                    strncpy(_scMdbCmdata[i].au8CmdData, pCur, SC_MDB_CMD_DATA_LEN);
                _scInfo[i].u32CardStatus |= E_SC_INT_MDB_CMD_DATA;
                if (_scMdbInfo[i].bOpened == TRUE)
                    wake_up_interruptible(&_scInfo[i].stWaitQue);
            }
            break;

        case MDB_SC_CMD_UNKNOW:
            default:
            return -EINVAL;
    }

    return 0;
}


static ssize_t mdb_sc_node_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
    char cmd[16];
    int ret = 0;

    if (count > (MBD_CMD_SIZE - 1) || count < 1)
    {
        printk("Invalid command\n");
        return -EINVAL;
    }

    if (copy_from_user(cmd, buffer, count))
    {
        return -EFAULT;
    }

    cmd[count - 1] = '\0';

    ret = mbd_sc_cmd_handler(cmd);

    if (ret)
        return ret;
    else
        return count;
}

static int mdb_sc_node_open(struct inode *inode, struct file *file)
{
    return single_open(file, mdb_sc_node_show, NULL);
}
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int MDrv_SC_GetCwtRxErrorIndex(struct file *filp, unsigned long arg)
#else
int MDrv_SC_GetCwtRxErrorIndex(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    U8 u8SCID = (U8)(int)filp->private_data;

    put_user(_scInfo[u8SCID].u32CwtRxErrorIndex, (int __user *)arg);

    return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int MDrv_SC_ResetRstToATR(struct file *filp, unsigned long arg)
#else
int MDrv_SC_ResetRstToATR(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    U8 u8SCID = (U8)(int)filp->private_data;

    _scInfo[u8SCID].u32StartTimeRstToATR = _MDrv_SC_GetTimeUs();

    return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
int MDrv_SC_EnableIRQ(struct file *filp, unsigned long arg)
#else
int MDrv_SC_EnableIRQ(struct inode *inode, struct file *filp, unsigned long arg)
#endif
{
    U8 u8SCID = (U8)(int)filp->private_data;

    if (u8SCID == 0)
    {
        enable_irq(SC_IRQ);
    }
    #if (SC_DEV_NUM > 1) // no more than 2
    if (u8SCID == 1)
    {
        enable_irq(SC_IRQ2);
    }
    #endif

    return 0;
}
