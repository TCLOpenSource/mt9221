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
/// file    mdrv_temp_io.c
/// @brief  TEMP Driver Interface for Export
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
#include <linux/string.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif
#include <linux/device.h>

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#include <linux/namei.h>
#include <linux/proc_fs.h>
#include "mdrv_types.h"
#include "mdrv_system.h"
#endif

//drver header files
#include "mst_devid.h"
#include "mdrv_mstypes.h"
#include "mdrv_sc_io.h"
#include "mhal_sc.h"
#include "mdrv_sc.h"

//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------
static int                  _mod_sc_open (struct inode *inode, struct file *filp);
static int                  _mod_sc_release(struct inode *inode, struct file *filp);
static ssize_t              _mod_sc_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t              _mod_sc_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static unsigned int         _mod_sc_poll(struct file *filp, poll_table *wait);

#if defined(CONFIG_COMPAT)
static long Compat_mod_sc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
static long                 _mod_sc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int                  _mod_sc_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif
//static int                  _mod_sc_fasync(int fd, struct file *filp, int mode);

SC_DEV devSC =
{
    .s32Major   =           MDRV_MAJOR_SMART,
    .s32Minor   =           MDRV_MINOR_SMART,
    .stCDev =
    {
        .kobj   =           {.name = MDRV_NAME_SMART, },
        .owner  =           THIS_MODULE,
    },
    .fops =
    {
        .open   =           _mod_sc_open,
        .release=           _mod_sc_release,
        .read   =           _mod_sc_read,
        .write  =           _mod_sc_write,
        .poll   =           _mod_sc_poll,
        #if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,20)
        .unlocked_ioctl  =  _mod_sc_ioctl,
        #else
        .ioctl  =           _mod_sc_ioctl,
        #endif
        #if defined(CONFIG_COMPAT)
		.compat_ioctl    = Compat_mod_sc_ioctl,
		#endif
        //.fasync =	        _mod_sc_fasync,
    },
};


//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------

static struct class *sc_class;

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
struct proc_dir_entry *mdb_proc_entry = NULL;
extern const struct file_operations mdb_sc_node_operations;
#endif
//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
static int _mod_sc_open(struct inode *inode, struct file *filp)
{
    SC_PRINT("%s is invoked\n", __FUNCTION__);
    filp->private_data = (void*)(iminor(inode) - devSC.s32Minor);

    return MDrv_SC_Open(inode, filp);
}

static int _mod_sc_release(struct inode *inode, struct file *filp)
{
    SC_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

static ssize_t _mod_sc_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{

    SC_PRINT("%s is invoked count=%d\n", __FUNCTION__, count);
    return MDrv_SC_Read(filp, buf, count, f_pos);
}

static ssize_t _mod_sc_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    SC_PRINT("%s is invoked count=%d\n", __FUNCTION__, count);
    return MDrv_SC_Write(filp, buf, count, f_pos);
}

static unsigned int _mod_sc_poll(struct file *filp, poll_table *wait)
{
    // SC_PRINT("%s is invoked\n", __FUNCTION__);
    return MDrv_SC_Poll(filp, wait);
}

#if 0
static int _mod_sc_fasync(int fd, struct file *filp, int mode)
{
    SC_PRINT("%s is invoked\n", __FUNCTION__);
	return fasync_helper(fd, filp, mode, &devSC.async_queue);
}
#endif

#if defined(CONFIG_COMPAT)
static long Compat_mod_sc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int err= 0;
    switch(cmd)
    {
        case MDRV_SC_ATTACH_INTERRUPT:
        case MDRV_SC_DETACH_INTERRUPT:
        case MDRV_SC_RESET_FIFO:
        case MDRV_SC_GET_EVENTS:
        case MDRV_SC_SET_EVENTS:
        case MDRV_SC_GET_ATTRIBUTE:
        case MDRV_SC_SET_ATTRIBUTE:
        case MDRV_SC_CHECK_RST_TO_ATR:
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
        case MDRV_SC_MDB_WRITE_INFO:
        case MDRV_SC_MDB_GET_CMD_DATA:
#endif
        case MDRV_SC_GET_CWT_RX_ERROR_INDEX:
        case MDRV_SC_RESET_RST_TO_ATR:
        case MDRV_SC_ENABLE_IRQ:
            return filp->f_op->unlocked_ioctl(filp, cmd,(unsigned long)compat_ptr(arg));

        default:
            return -ENOIOCTLCMD;
    }
    return -ENOIOCTLCMD;
}
#endif


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
static long _mod_sc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int _mod_sc_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
    int err= 0;

    SC_PRINT("%s is invoked\n", __FUNCTION__);

    /*
     * extract the type and number bitfields, and don't decode
     * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
     */
    if ((SC_IOC_MAGIC != _IOC_TYPE(cmd)) || (_IOC_NR(cmd) > SC_IOC_MAXNR))
    {
        return -ENOTTY;
    }

    /*
     * the direction is a bitmask, and VERIFY_WRITE catches R/W
     * transfers. `Type' is user-oriented, while
     * access_ok is kernel-oriented, so the concept of "read" and
     * "write" is reversed
     */

    if (_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }
    if (err)
    {
        return -EFAULT;
    }

    #if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,36)
    switch(cmd)
    {
        case MDRV_SC_ATTACH_INTERRUPT:
            MDrv_SC_AttachInterrupt(filp, arg);
            break;
        case MDRV_SC_DETACH_INTERRUPT:
            MDrv_SC_DetachInterrupt(filp, arg);
            break;
        case MDRV_SC_RESET_FIFO:
            MDrv_SC_ResetFIFO(filp, arg);
            break;
        case MDRV_SC_GET_EVENTS:
            MDrv_SC_GetEvent(filp, arg);
            break;
        case MDRV_SC_SET_EVENTS:
            MDrv_SC_SetEvent(filp, arg);
            break;
        case MDRV_SC_GET_ATTRIBUTE:
            MDrv_SC_GetAttribute(filp, arg);
            break;
        case MDRV_SC_SET_ATTRIBUTE:
            MDrv_SC_SetAttribute(filp, arg);
            break;
        case MDRV_SC_CHECK_RST_TO_ATR:
            MDrv_SC_CheckRstToATR(filp, arg);
            break;
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
        case MDRV_SC_MDB_WRITE_INFO:
            mdb_sc_write_info(filp, arg);
            break;
        case MDRV_SC_MDB_GET_CMD_DATA:
            mdb_sc_get_cmd_data(filp, arg);
            break;
#endif
        case MDRV_SC_GET_CWT_RX_ERROR_INDEX:
            MDrv_SC_GetCwtRxErrorIndex(filp, arg);
            break;
        case MDRV_SC_RESET_RST_TO_ATR:
            MDrv_SC_ResetRstToATR(filp, arg);
            break;
        case MDRV_SC_ENABLE_IRQ:
            MDrv_SC_EnableIRQ(filp, arg);
            break;
        default:
            SC_PRINT("ioctl: unknown command\n");
            return -ENOTTY;
    }
    #else
    switch(cmd)
    {
        case MDRV_SC_ATTACH_INTERRUPT:
            MDrv_SC_AttachInterrupt(inode, filp, arg);
            break;
        case MDRV_SC_DETACH_INTERRUPT:
            MDrv_SC_DetachInterrupt(inode, filp, arg);
            break;
        case MDRV_SC_RESET_FIFO:
            MDrv_SC_ResetFIFO(inode, filp, arg);
            break;
        case MDRV_SC_GET_EVENTS:
            MDrv_SC_GetEvent(inode, filp, arg);
            break;
        case MDRV_SC_SET_EVENTS:
            MDrv_SC_SetEvent(inode, filp, arg);
            break;
        case MDRV_SC_GET_ATTRIBUTE:
            MDrv_SC_GetAttribute(inode, filp, arg);
            break;
        case MDRV_SC_SET_ATTRIBUTE:
            MDrv_SC_SetAttribute(inode, filp, arg);
            break;
        case MDRV_SC_CHECK_RST_TO_ATR:
            MDrv_SC_CheckRstToATR(inode, filp, arg);
            break;
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
        case MDRV_SC_MDB_WRITE_INFO:
            mdb_sc_write_info(inode, filp, arg);
            break;
        case MDRV_SC_MDB_GET_CMD_DATA:
            mdb_sc_get_cmd_data(inode, filp, arg);
            break;
#endif
        case MDRV_SC_GET_CWT_RX_ERROR_INDEX:
            MDrv_SC_GetCwtRxErrorIndex(inode, filp, arg);
            break;
        case MDRV_SC_RESET_RST_TO_ATR:
            MDrv_SC_ResetRstToATR(inode, filp, arg);
            break;
        case MDRV_SC_ENABLE_IRQ:
            MDrv_SC_EnableIRQ(inode, filp, arg);
            break;
        default:
            SC_PRINT("ioctl: unknown command\n");
            return -ENOTTY;
    }
    #endif

    return 0;
}


//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
MSYSTEM_STATIC int __init mod_sc_init(void)
{
    int         s32Ret;
    dev_t       dev;

#ifdef CONFIG_MSTAR_UDEV_NODE
    sc_class = class_create(THIS_MODULE, MDRV_NAME_SMART);
    if (IS_ERR(sc_class))
    {
        return PTR_ERR(sc_class);
    }
#endif
    SC_PRINT("%s is invoked\n", __FUNCTION__);

    if (devSC.s32Major)
    {
        dev = MKDEV(devSC.s32Major, devSC.s32Minor);
        s32Ret = register_chrdev_region(dev, SC_DEV_NUM, MDRV_NAME_SMART);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, devSC.s32Minor, SC_DEV_NUM, MDRV_NAME_SMART);
        devSC.s32Major = MAJOR(dev);
    }

    if (0 > s32Ret)
    {
        SC_PRINT("Unable to get major %d\n", devSC.s32Major);
        return s32Ret;
    }

    cdev_init(&devSC.stCDev, &devSC.fops);
    if (0 != (s32Ret = cdev_add(&devSC.stCDev, dev, SC_DEV_NUM)))
    {
        SC_PRINT("Unable add a character device\n");
        unregister_chrdev_region(dev, SC_DEV_NUM);
        return s32Ret;
    }

#ifdef CONFIG_MSTAR_UDEV_NODE
    device_create(sc_class, NULL, dev, NULL, MDRV_NAME_SMART);
#endif

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
    MDrv_SYS_UtopiaMdbMkdir();

    mdb_proc_entry = proc_create("utopia_mdb/smc", S_IRUSR, NULL, &mdb_sc_node_operations);
    if (mdb_proc_entry == NULL)
    {
        printk("[SC] %s: unable to create proc node\n", __FUNCTION__);
    }
#endif

    return 0;
}

MSYSTEM_STATIC void __exit mod_sc_exit(void)
{
    SC_PRINT("%s is invoked\n", __FUNCTION__);

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
    if (mdb_proc_entry != NULL)
        proc_remove(mdb_proc_entry);
#endif

    cdev_del(&devSC.stCDev);
    unregister_chrdev_region(MKDEV(devSC.s32Major, devSC.s32Minor), SC_DEV_NUM);
}

module_init(mod_sc_init);
module_exit(mod_sc_exit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("Smart card driver");
MODULE_LICENSE("GPL");
