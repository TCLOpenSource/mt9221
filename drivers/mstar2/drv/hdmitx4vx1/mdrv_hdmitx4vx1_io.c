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
/// file    mdrv_hdmitx4vx1_io.c
/// @brief  hdmitx4vx1 Driver Interface for Export
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <linux/platform_device.h>

#include "mst_devid.h"
#include "mdrv_mstypes.h"
#include "mdrv_hdmitx4vx1_io.h"
#include "mdrv_hdmitx4vx1.h"


//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define     MDRV_HDMITX4VX1_DEVICE_COUNT           1
#define     MDRV_HDMITX4VX1_NAME                           "hdmitx4vx1"



//-------------------------------------------------------------------------------------------------
//  Macros
//-------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------
typedef struct
{
    int s32Major;
    int s32Minor;
    int refCnt;
    struct cdev cdev;
    struct file_operations fops;
} HDMITX4VX1_DEV;



//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------
//  Forward declaration
//--------------------------------------------------------------------------------------------------
static int _MDrv_HDMITX4VX1_Open (struct inode *inode, struct file *filp);
static ssize_t _MDrv_HDMITX4VX1_Write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static ssize_t _MDrv_HDMITX4VX1_Read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static int _MDrv_HDMITX4VX1_Release(struct inode *inode, struct file *filp);
static long _MDrv_HDMITX4VX1_IOCtl(struct file *filp, unsigned int u32Cmd, unsigned long u32Arg);

static int _MDrv_HDMITX4VX1_Probe(struct platform_device *pdev);
static int _MDrv_HDMITX4VX1_Remove(struct platform_device *pdev);
static int _MDrv_HDMITX4VX1_Suspend(struct platform_device *dev, pm_message_t state);
static int _MDrv_HDMITX4VX1_Resume(struct platform_device *dev);


//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
static struct class *mHDMITX4VX1_class;


static HDMITX4VX1_DEV devHDMITX4VX1 =
{
    .s32Major = MDRV_MAJOR_HDMITX4VX1,
    .s32Minor = MDRV_MINOR_HDMITX4VX1,
    .refCnt = 0,

    .cdev =
    {
        .kobj = {.name= MDRV_NAME_HDMITX4VX1, },
        .owner = THIS_MODULE,
    },
    .fops =
    {
        .open =                     _MDrv_HDMITX4VX1_Open,
        .write =                    _MDrv_HDMITX4VX1_Write,
        .read =                     _MDrv_HDMITX4VX1_Read,
        .release =                  _MDrv_HDMITX4VX1_Release,
        .unlocked_ioctl =           _MDrv_HDMITX4VX1_IOCtl,
    }

};

static struct of_device_id mstarHDMITX4VX1_of_device_ids[] = {
         {.compatible = "mstar-hdmitx4vx1"},
         {},
};

static struct platform_driver Mstar_HDMITX4VX1_driver = {
    .probe      = _MDrv_HDMITX4VX1_Probe,
    .remove     = _MDrv_HDMITX4VX1_Remove,
    .suspend    = _MDrv_HDMITX4VX1_Suspend,
    .resume     = _MDrv_HDMITX4VX1_Resume,

    .driver = {

        .name   = "mstar-hdmitx4vx1",
        .owner  = THIS_MODULE,
        .of_match_table = mstarHDMITX4VX1_of_device_ids,

    }
};

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
static int _MDrv_HDMITX4VX1_Probe(struct platform_device *pdev)
{
    HDMITX4VX1_KDBG("Probe is called\n");
    HDMITX4VX1_Probe();
    return 0;
}

static int _MDrv_HDMITX4VX1_Remove(struct platform_device *pdev)
{
    return 0;
}
static int _MDrv_HDMITX4VX1_Suspend(struct platform_device *dev, pm_message_t state)
{
    return 0;
}

static int _MDrv_HDMITX4VX1_Resume(struct platform_device *dev)
{
    return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------
// IOCtrl Driver interface functions
//-------------------------------------------------------------------------------------------------
int _MDrv_HDMITX4VX1_Open (struct inode *inode, struct file *filp)
{
    devHDMITX4VX1.refCnt++;
    return 0;
}

static ssize_t _MDrv_HDMITX4VX1_Write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    return 0;
}
static ssize_t _MDrv_HDMITX4VX1_Read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    return 0;
}

int _MDrv_HDMITX4VX1_Release(struct inode *inode, struct file *filp)
{
    devHDMITX4VX1.refCnt--;
    return 0;
}
int _MDrv_HDMITX4VX1_IOC_Init(struct file *filp, unsigned long arg)
{
    EN_HDMITX4VX1_LOCKN_TYPE enLOCKNType;

    if(copy_from_user(&enLOCKNType, (EN_HDMITX4VX1_LOCKN_TYPE __user *)arg, sizeof(EN_HDMITX4VX1_LOCKN_TYPE)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_Init(enLOCKNType)==FALSE)
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_VIDEO_CONFIG(struct file *filp, unsigned long arg)
{
    ST_MAILBOX_COMMAND_CONFIQ_VIDEO stVideoConfig;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stVideoConfig, (ST_MAILBOX_COMMAND_CONFIQ_VIDEO __user *)arg, sizeof(ST_MAILBOX_COMMAND_CONFIQ_VIDEO)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_Video_config(stVideoConfig)==FALSE)
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_AUDIO_CONFIG(struct file *filp, unsigned long arg)
{
    ST_MAILBOX_COMMAND_CONFIQ_AUDIO stAudioConfig;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stAudioConfig, (ST_MAILBOX_COMMAND_CONFIQ_AUDIO __user *)arg, sizeof(ST_MAILBOX_COMMAND_CONFIQ_AUDIO)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_Audio_config(stAudioConfig)==FALSE)
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_TIMINGCHANGEAVMUTE_CONFIG(struct file *filp, unsigned long arg)
{
    ST_MAILBOX_COMMAND_TIMING_CHANGE_AVMUTE stTimingChangeConfig;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stTimingChangeConfig, (ST_MAILBOX_COMMAND_TIMING_CHANGE_AVMUTE __user *)arg, sizeof(ST_MAILBOX_COMMAND_TIMING_CHANGE_AVMUTE)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_TimingChange_AVmute(stTimingChangeConfig)==FALSE)
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_QUERY_4K2KVIC(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_QUERY_4K2K_VIC stQuery;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stQuery, (ST_HDMITX4VX1_IOC_QUERY_4K2K_VIC __user *)arg, sizeof(ST_HDMITX4VX1_IOC_QUERY_4K2K_VIC)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_QUERY_4K2KVIC(stQuery.query_cmd,&stQuery.report_cmd)==FALSE)
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_HDMITX4VX1_IOC_QUERY_4K2K_VIC __user *)arg, &stQuery, sizeof(ST_HDMITX4VX1_IOC_QUERY_4K2K_VIC)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_QUERY_3DSTRUCTURE(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_QUERY_3D_STRUCTURE stQuery;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stQuery, (ST_HDMITX4VX1_IOC_QUERY_3D_STRUCTURE __user *)arg, sizeof(ST_HDMITX4VX1_IOC_QUERY_3D_STRUCTURE)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_QUERY_3D_Structure(stQuery.query_cmd,&stQuery.report_cmd)==FALSE)
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_HDMITX4VX1_IOC_QUERY_3D_STRUCTURE __user *)arg, &stQuery, sizeof(ST_HDMITX4VX1_IOC_QUERY_3D_STRUCTURE)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_QUERY_AUDIODESCRIPTOR_NUM(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_QUERY_NUM_AUDIODESCRIPTOR stQuery;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stQuery, (ST_HDMITX4VX1_IOC_QUERY_NUM_AUDIODESCRIPTOR __user *)arg, sizeof(ST_HDMITX4VX1_IOC_QUERY_NUM_AUDIODESCRIPTOR)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_QUERY_Num_AudioDescriptor(stQuery.query_cmd,&stQuery.report_cmd)==FALSE)
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_HDMITX4VX1_IOC_QUERY_NUM_AUDIODESCRIPTOR __user *)arg, &stQuery, sizeof(ST_HDMITX4VX1_IOC_QUERY_NUM_AUDIODESCRIPTOR)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_QUERY_AUDIODESCRIPTOR(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_QUERY_AUDIODESCRIPTOR stQuery;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stQuery, (ST_HDMITX4VX1_IOC_QUERY_AUDIODESCRIPTOR __user *)arg, sizeof(ST_HDMITX4VX1_IOC_QUERY_AUDIODESCRIPTOR)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_QUERY_AudioDescriptor(stQuery.query_cmd,&stQuery.report_cmd)==FALSE)
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_HDMITX4VX1_IOC_QUERY_AUDIODESCRIPTOR __user *)arg, &stQuery, sizeof(ST_HDMITX4VX1_IOC_QUERY_AUDIODESCRIPTOR)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_QUERY_VIDEODESCRIPTOR_NUM(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_QUERY_NUM_VIDEODESCRIPTOR stQuery;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stQuery, (ST_HDMITX4VX1_IOC_QUERY_NUM_VIDEODESCRIPTOR __user *)arg, sizeof(ST_HDMITX4VX1_IOC_QUERY_NUM_VIDEODESCRIPTOR)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_QUERY_Num_VideoDescriptor(stQuery.query_cmd,&stQuery.report_cmd)==FALSE)
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_HDMITX4VX1_IOC_QUERY_NUM_VIDEODESCRIPTOR __user *)arg, &stQuery, sizeof(ST_HDMITX4VX1_IOC_QUERY_NUM_VIDEODESCRIPTOR)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_QUERY_VIDEODESCRIPTOR(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_QUERY_VIDEODESCRIPTOR stQuery;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stQuery, (ST_HDMITX4VX1_IOC_QUERY_VIDEODESCRIPTOR __user *)arg, sizeof(ST_HDMITX4VX1_IOC_QUERY_VIDEODESCRIPTOR)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_QUERY_VideoDescriptor(stQuery.query_cmd,&stQuery.report_cmd)==FALSE)
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_HDMITX4VX1_IOC_QUERY_VIDEODESCRIPTOR __user *)arg, &stQuery, sizeof(ST_HDMITX4VX1_IOC_QUERY_VIDEODESCRIPTOR)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_QUERY_EDIDRAWDATA(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_QUERY_EDIDRAWDATA stQuery;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stQuery, (ST_HDMITX4VX1_IOC_QUERY_EDIDRAWDATA __user *)arg, sizeof(ST_HDMITX4VX1_IOC_QUERY_EDIDRAWDATA)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_QUERY_EDID_RawData(stQuery.query_cmd,&stQuery.report_cmd)==FALSE)
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_HDMITX4VX1_IOC_QUERY_EDIDRAWDATA __user *)arg, &stQuery, sizeof(ST_HDMITX4VX1_IOC_QUERY_EDIDRAWDATA)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_QUERY_RXEDIDINFO(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_QUERY_RXEDIDINFO stQuery;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stQuery, (ST_HDMITX4VX1_IOC_QUERY_RXEDIDINFO __user *)arg, sizeof(ST_HDMITX4VX1_IOC_QUERY_RXEDIDINFO)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_QUERY_RxEdidInfo(stQuery.query_cmd,&stQuery.report_cmd)==FALSE)
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_HDMITX4VX1_IOC_QUERY_RXEDIDINFO __user *)arg, &stQuery, sizeof(ST_HDMITX4VX1_IOC_QUERY_RXEDIDINFO)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_QUERY_COLORFORMAT(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_QUERY_COLORFORMAT stQuery;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stQuery, (ST_HDMITX4VX1_IOC_QUERY_COLORFORMAT __user *)arg, sizeof(ST_HDMITX4VX1_IOC_QUERY_COLORFORMAT)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_QUERY_ColorFormat(stQuery.query_cmd,&stQuery.report_cmd)==FALSE)
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_HDMITX4VX1_IOC_QUERY_COLORFORMAT __user *)arg, &stQuery, sizeof(ST_HDMITX4VX1_IOC_QUERY_COLORFORMAT)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_QUERY_HWINFO(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_QUERY_HWINFO stQuery;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stQuery, (ST_HDMITX4VX1_IOC_QUERY_HWINFO __user *)arg, sizeof(ST_HDMITX4VX1_IOC_QUERY_HWINFO)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_QUERY_HWInfo(stQuery.query_cmd,&stQuery.report_cmd)==FALSE)
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_HDMITX4VX1_IOC_QUERY_HWINFO __user *)arg, &stQuery, sizeof(ST_HDMITX4VX1_IOC_QUERY_HWINFO)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_QUERY_KSVBSTATUS(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_QUERY_KSVBSTATUS stQuery;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stQuery, (ST_HDMITX4VX1_IOC_QUERY_KSVBSTATUS __user *)arg, sizeof(ST_HDMITX4VX1_IOC_QUERY_KSVBSTATUS)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_QUERY_KSV_BStatus(stQuery.query_cmd,&stQuery.report_cmd)==FALSE)
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_HDMITX4VX1_IOC_QUERY_KSVBSTATUS __user *)arg, &stQuery, sizeof(ST_HDMITX4VX1_IOC_QUERY_KSVBSTATUS)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_QUERY_HDCPKEYSTATUS(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_QUERY_HDCPKEYSTATUS stQuery;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stQuery, (ST_HDMITX4VX1_IOC_QUERY_HDCPKEYSTATUS __user *)arg, sizeof(ST_HDMITX4VX1_IOC_QUERY_HDCPKEYSTATUS)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_QUERY_HDCPKey_Status(stQuery.query_cmd,&stQuery.report_cmd)==FALSE)
    {
        return -EFAULT;
    }

    if(copy_to_user((ST_HDMITX4VX1_IOC_QUERY_HDCPKEYSTATUS __user *)arg, &stQuery, sizeof(ST_HDMITX4VX1_IOC_QUERY_HDCPKEYSTATUS)))
    {
        return -EFAULT;
    }

    return 0;
}


int _MDrv_HDMITX4VX1_IOC_HDCP_COMMAND(struct file *filp, unsigned long arg)
{
    ST_MAILBOX_COMMAND_HDCP_COMD stHDCP;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stHDCP, (ST_MAILBOX_COMMAND_HDCP_COMD __user *)arg, sizeof(ST_MAILBOX_COMMAND_HDCP_COMD)))
    {
        return -EFAULT;
    }

    if(HDMITX4VX1_HDCP_CMD(stHDCP)==FALSE)
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_GET_RXSTATUS(struct file *filp, unsigned long arg)
{
    int iRxStatus=0;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    iRxStatus=HDMITX4VX1_GetRxStatus();

    if(copy_to_user((int __user *)arg, &iRxStatus, sizeof(int)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_LOADOUTPUTSCRIPT(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_LOADOUTPUTSCRIPT_SETTINGS stScriptSettings;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stScriptSettings, (ST_HDMITX4VX1_LOADOUTPUTSCRIPT_SETTINGS __user *)arg, sizeof(ST_HDMITX4VX1_LOADOUTPUTSCRIPT_SETTINGS)))
    {
        return -EFAULT;
    }

    HDMITX4VX1_loadKeepOutputScript(stScriptSettings.loadtiming,stScriptSettings.loadcolordepth,stScriptSettings.loadoutcolorfmt);

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_READREGISTER(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_READREGISTER stReg;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stReg, (ST_HDMITX4VX1_IOC_READREGISTER __user *)arg, sizeof(ST_HDMITX4VX1_IOC_READREGISTER)))
    {
        return -EFAULT;
    }

    HDMITX4VX1_ReadRegisterForce(stReg.register_num,stReg.read_register);

    if(copy_to_user((ST_HDMITX4VX1_IOC_READREGISTER __user *)arg, &stReg, sizeof(ST_HDMITX4VX1_IOC_READREGISTER)))
    {
        return -EFAULT;
    }

    return 0;
}

int _MDrv_HDMITX4VX1_IOC_WRITEREGISTER_MASK(struct file *filp, unsigned long arg)
{
    ST_HDMITX4VX1_IOC_WRITEREGISTER stReg;
    HDMITX4VX1_KDBG("[HDMITX4VX1] %s, %d\n",__func__,__LINE__);

    if(copy_from_user(&stReg, (ST_HDMITX4VX1_IOC_WRITEREGISTER __user *)arg, sizeof(ST_HDMITX4VX1_IOC_WRITEREGISTER)))
    {
        return -EFAULT;
    }

    HDMITX4VX1_WriteRegisterMaskForce(stReg.register_num,stReg.write_register);

    return 0;
}


long _MDrv_HDMITX4VX1_IOCtl(struct file *filp, unsigned int u32Cmd, unsigned long u32Arg)
{
    int err = 0;
    int retval = 0;

    if(devHDMITX4VX1.refCnt <= 0)
    {
        HDMITX4VX1_KDBG("\n[HDMITX4VX1] refCnt <= 0 \n ");
        return -EFAULT;
    }
    /* check u32Cmd valid */
    if(IOCTL_HDMITX4VX1_MAGIC == _IOC_TYPE(u32Cmd))
    {
        if(_IOC_NR(u32Cmd) >= HDMITX4VX1_MAX_IOC_NR)
        {
            HDMITX4VX1_KDBG("[HDMITX4VX1] IOCtl NR Error!!! (Cmd=%x)\n",u32Cmd);
            return -ENOTTY;
        }
    }
    else
    {
        HDMITX4VX1_KDBG("[HDMITX4VX1] IOCtl MAGIC Error!!! (Cmd=%x)\n",u32Cmd);
        return -ENOTTY;
    }

    /* verify Access */
    if (_IOC_DIR(u32Cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)u32Arg, _IOC_SIZE(u32Cmd));
    }
    else if (_IOC_DIR(u32Cmd) & _IOC_WRITE)
    {
        err =  !access_ok(VERIFY_READ, (void __user *)u32Arg, _IOC_SIZE(u32Cmd));
    }
    if (err)
    {
        HDMITX4VX1_KDBG("\n[HDMITX4VX1] verify fail \n ");
        return -EFAULT;
    }

    HDMITX4VX1_KDBG("[HDMITX4VX1] IOCtl number %x \n ", _IOC_NR(u32Cmd));
    /* handle u32Cmd */
    switch(u32Cmd)
    {
        case IOCTL_HDMITX4VX1_INIT:
            retval = _MDrv_HDMITX4VX1_IOC_Init(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_VIDEO_CONFIG:
            retval = _MDrv_HDMITX4VX1_IOC_VIDEO_CONFIG(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_AUDIO_CONFIG:
            retval = _MDrv_HDMITX4VX1_IOC_AUDIO_CONFIG(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_TIMINGCHANGEAVMUTE_CONFIG:
            retval = _MDrv_HDMITX4VX1_IOC_TIMINGCHANGEAVMUTE_CONFIG(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_QUERY_4K2KVIC:
            retval = _MDrv_HDMITX4VX1_IOC_QUERY_4K2KVIC(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_QUERY_3DSTRUCTURE:
            retval = _MDrv_HDMITX4VX1_IOC_QUERY_3DSTRUCTURE(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_QUERY_AUDIODESCRIPTOR_NUM:
            retval = _MDrv_HDMITX4VX1_IOC_QUERY_AUDIODESCRIPTOR_NUM(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_QUERY_AUDIODESCRIPTOR:
            retval = _MDrv_HDMITX4VX1_IOC_QUERY_AUDIODESCRIPTOR(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_QUERY_VIDEODESCRIPTOR_NUM:
            retval = _MDrv_HDMITX4VX1_IOC_QUERY_VIDEODESCRIPTOR_NUM(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_QUERY_VIDEODESCRIPTOR:
            retval = _MDrv_HDMITX4VX1_IOC_QUERY_VIDEODESCRIPTOR(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_QUERY_EDIDRAWDATA:
            retval = _MDrv_HDMITX4VX1_IOC_QUERY_EDIDRAWDATA(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_QUERY_RXEDIDINFO:
            retval = _MDrv_HDMITX4VX1_IOC_QUERY_RXEDIDINFO(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_QUERY_COLORFORMAT:
            retval = _MDrv_HDMITX4VX1_IOC_QUERY_COLORFORMAT(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_QUERY_HWINFO:
            retval = _MDrv_HDMITX4VX1_IOC_QUERY_HWINFO(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_QUERY_KSVBSTATUS:
            retval = _MDrv_HDMITX4VX1_IOC_QUERY_KSVBSTATUS(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_QUERY_HDCPKEYSTATUS:
            retval = _MDrv_HDMITX4VX1_IOC_QUERY_HDCPKEYSTATUS(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_HDCP_COMMAND:
            retval = _MDrv_HDMITX4VX1_IOC_HDCP_COMMAND(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_GET_RXSTATUS:
            retval = _MDrv_HDMITX4VX1_IOC_GET_RXSTATUS(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_LOADOUTPUTSCRIPT:
            retval = _MDrv_HDMITX4VX1_IOC_LOADOUTPUTSCRIPT(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_READREGISTER:
            retval = _MDrv_HDMITX4VX1_IOC_READREGISTER(filp, u32Arg);
            break;

        case IOCTL_HDMITX4VX1_WRITEREGISTER_MASK:
            retval = _MDrv_HDMITX4VX1_IOC_WRITEREGISTER_MASK(filp, u32Arg);
            break;


        default:  /* redundant, as cmd was checked against MAXNR */
            HDMITX4VX1_KDBG("[HDMITX4VX1] ERROR IOCtl number %x\n ",u32Cmd);
            return -ENOTTY;
    }

    return (long)retval;
}


int _MDrv_HDMITX4VX1_ModuleInit(void)
{
    int         s32Ret;
    dev_t       dev;
    int         Retfordriverreg;

    mHDMITX4VX1_class = class_create(THIS_MODULE, MDRV_NAME_HDMITX4VX1);
    if (IS_ERR(mHDMITX4VX1_class))
    {
        return PTR_ERR(mHDMITX4VX1_class);
    }

    if (devHDMITX4VX1.s32Major)
    {
        dev = MKDEV(devHDMITX4VX1.s32Major, devHDMITX4VX1.s32Minor);
        s32Ret = register_chrdev_region(dev, MDRV_HDMITX4VX1_DEVICE_COUNT, MDRV_HDMITX4VX1_NAME);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, devHDMITX4VX1.s32Minor, MDRV_HDMITX4VX1_DEVICE_COUNT, MDRV_HDMITX4VX1_NAME);
        devHDMITX4VX1.s32Major = MAJOR(dev);
    }

    if (0 > s32Ret)
    {
        HDMITX4VX1_KDBG("[HDMITX4VX1] Unable to get major= %d  ;s32Ret=%d  ;\n", devHDMITX4VX1.s32Major,s32Ret);
        PTR_ERR(mHDMITX4VX1_class);
        return s32Ret;
    }

    cdev_init(&devHDMITX4VX1.cdev ,&devHDMITX4VX1.fops);
    if (0 != (s32Ret = cdev_add(&devHDMITX4VX1.cdev, dev, MDRV_HDMITX4VX1_DEVICE_COUNT)))
    {
        HDMITX4VX1_KDBG("[HDMITX4VX1] Unable add a character device\n");
        unregister_chrdev_region(dev, MDRV_HDMITX4VX1_DEVICE_COUNT);
        PTR_ERR(mHDMITX4VX1_class);
        return s32Ret;
    }

    /* initial the whole HDMITX4VX1 Driver */
    /*
    if(EFAULT == _MDrv_XC_RegisterInterrupt())
    {
        HDMITX4VX1_KDBG("[HDMITX4VX1] Startup HDMITX4VX1 Driver Failed! %d\n", devHDMITX4VX1.s32Major);
        cdev_del(&devHDMITX4VX1.cdev);
        unregister_chrdev_region(dev, MDRV_HDMITX4VX1_DEVICE_COUNT);
        PTR_ERR(mHDMITX4VX1_class);
        return -ENOMEM;
    }
    */
    device_create(mHDMITX4VX1_class, NULL, dev, NULL, MDRV_NAME_HDMITX4VX1);
    Retfordriverreg=platform_driver_register(&Mstar_HDMITX4VX1_driver);

    //MDrv_XC_Init();
    //HDMITX4VX1_KDBG("[HDMITX4VX1] module init driver register=%d\n",Retfordriverreg);
    HDMITX4VX1_KDBG("[HDMITX4VX1] module init success\n");
    //testforI2C();

    return 0;
}


void _MDrv_HDMITX4VX1_ModuleExit(void)
{
    /*de-initial the who Driver */
    //_MDrv_XC_DeRegisterInterrupt();

    cdev_del(&devHDMITX4VX1.cdev);
    unregister_chrdev_region(MKDEV(devHDMITX4VX1.s32Major, devHDMITX4VX1.s32Minor), MDRV_HDMITX4VX1_DEVICE_COUNT);
    device_destroy(mHDMITX4VX1_class, MKDEV(devHDMITX4VX1.s32Major, devHDMITX4VX1.s32Minor));
    class_destroy(mHDMITX4VX1_class);
    platform_driver_unregister(&Mstar_HDMITX4VX1_driver);

    //MDrv_XC_Exit();
}


module_init(_MDrv_HDMITX4VX1_ModuleInit);
module_exit(_MDrv_HDMITX4VX1_ModuleExit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("HDMITX4VX1 ioctrl driver");
MODULE_LICENSE("GPL");
