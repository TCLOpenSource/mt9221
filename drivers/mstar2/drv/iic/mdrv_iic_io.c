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
/// file    mdrv_iic.c
/// @brief  IIC Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////


//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
//#include "MsCommon.h"
#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,18,31)
#include <linux/of_i2c.h>
#else
extern void of_i2c_register_devices(struct i2c_adapter *adap);
#endif
#include <linux/of_gpio.h>

#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

#include "mst_devid.h"
#include "mdrv_iic_io.h"
#include "mhal_iic_reg.h"
#include "mdrv_iic.h"
#include "mhal_iic.h"


//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------

#define IIC_MUTEX_WAIT_TIME         3000

#define IIC_DBG_ENABLE              0

#if IIC_DBG_ENABLE
#define IIC_DBG(_f)                 (_f)
#else
#define IIC_DBG(_f)
#endif

#if 0
#define LINE_DBG()                  printf("IIC %d\n", __LINE__)
#else
#define LINE_DBG()
#endif

#define IIC_PRINT(fmt, args...)		printk("[IIC][%05d] " fmt, __LINE__, ## args)

typedef struct
{
    S32                         s32MajorIIC;
    S32                         s32MinorIIC;
    struct cdev                 cDevice;
    struct file_operations      IICFop;
    struct fasync_struct        *async_queue; /* asynchronous readers */
} IIC_ModHandle_t;


#define MOD_IIC_DEVICE_COUNT         1
#define MOD_IIC_NAME                 "ModIIC"

static struct mstar_i2c_attr
{
    u16 iic_mode;
    u16 bus_index;
    u16 sda_pad;
    u16 scl_pad;
    u8  hw_port;
    u16 pad_mux;
    u16 retries;
    u16 def_delay;
    u16 speed_khz;
};

struct mstar_i2c_dev {
    struct mstar_i2c_attr I2cAttr;
	struct platform_device* pdev;
	struct i2c_adapter      adapter;
};

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
static u8 bModIICisInit = 0;

#if defined(CONFIG_I2C_CHARDEV)
static struct class *chrdev_modiic_class = NULL;
#endif
//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------
static int                      _MDrv_IIC_Open (struct inode *inode, struct file *filp);
static int                      _MDrv_IIC_Release(struct inode *inode, struct file *filp);
static ssize_t                  _MDrv_IIC_Read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t                  _MDrv_IIC_Write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static unsigned int             _MDrv_IIC_Poll(struct file *filp, poll_table *wait);
#ifdef HAVE_UNLOCKED_IOCTL
static long _MDrv_IIC_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int _MDrv_IIC_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif
#if defined(CONFIG_COMPAT)
static long Compat_MDrv_IIC_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif

static int                      _MDrv_IIC_Fasync(int fd, struct file *filp, int mode);

//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
static IIC_ModHandle_t IICDev=
{
    .s32MajorIIC = MDRV_MAJOR_IIC,
    .s32MinorIIC = MDRV_MINOR_IIC,
    .cDevice =
    {
        .kobj = {.name= MOD_IIC_NAME, },
        .owner = THIS_MODULE,
    },
    .IICFop =
    {
        .open =     _MDrv_IIC_Open,
        .release =  _MDrv_IIC_Release,
        .read =     _MDrv_IIC_Read,
        .write =    _MDrv_IIC_Write,
        .poll =     _MDrv_IIC_Poll,
        #ifdef HAVE_UNLOCKED_IOCTL
        .unlocked_ioctl = _MDrv_IIC_Ioctl,
        #else
        .ioctl = _MDrv_IIC_Ioctl,
        #endif
		#if defined(CONFIG_COMPAT)
		.compat_ioctl = Compat_MDrv_IIC_Ioctl,
		#endif
        .fasync =   _MDrv_IIC_Fasync,
    },
};
//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
#if defined(CONFIG_COMPAT)
int mdrv_iic_compat_get_ion_allaction_data(IIC_Param_Compat __user *data32, IIC_Param __user *data)
{
	IIC_Param_Compat temp;
	U8 *ptr_addr;
	U8 *ptr_buf;
	int err = 0;

	err = get_user(temp.u8IdIIC, &data32->u8IdIIC);
	err = get_user(temp.u8ClockIIC, &data32->u8ClockIIC);
	err = get_user(temp.u8SlaveIdIIC, &data32->u8SlaveIdIIC);
	err = get_user(temp.u8AddrSizeIIC, &data32->u8AddrSizeIIC);

	err = get_user(ptr_addr, &data32->u8AddrIIC);
	err = get_user(ptr_buf, &data32->u8pbufIIC);
	temp.u8AddrIIC = compat_ptr(ptr_addr);   //(1) pointer transmit for 32bits to 64bits
	temp.u8pbufIIC = compat_ptr(ptr_buf);    //(1) pointer transmit for 32bits to 64bits
	err = get_user(temp.u32DataSizeIIC, &data32->u32DataSizeIIC);

	err = put_user(temp.u8IdIIC, &data->u8IdIIC);
	err = put_user(temp.u8ClockIIC, &data->u8ClockIIC);
	err = put_user(temp.u8SlaveIdIIC, &data->u8SlaveIdIIC);
	err = put_user(temp.u8AddrSizeIIC, &data->u8AddrSizeIIC); //(2) lost : u8AddrSizeIIC convert

	err = put_user(temp.u8AddrIIC, &data->u8AddrIIC);
	err = put_user(temp.u8pbufIIC, &data->u8pbufIIC);
	err = put_user(temp.u32DataSizeIIC, &data->u32DataSizeIIC);

	return err;
}

int mdrv_iic_compat_put_ion_allaction_data(IIC_Param_Compat __user *data32, IIC_Param  __user *data, int rw_flag)
{
	U8 *ptr_addr;
	U8 *ptr_buf;

	int err = 0;

	if (rw_flag > 0) // read iic
	{
		err = get_user(ptr_addr, &data->u8AddrIIC);
		err = put_user(ptr_addr, &data32->u8AddrIIC);
		err = get_user(ptr_buf, &data->u8pbufIIC);
		err = put_user(ptr_buf, &data32->u8pbufIIC);
	}
	return err;
}
#endif

static int _MDrv_IIC_Open (struct inode *inode, struct file *filp)
{
	IIC_ModHandle_t *dev;

    IIC_PRINT("%s is invoked\n", __FUNCTION__);

    dev = container_of(inode->i_cdev, IIC_ModHandle_t, cDevice);
	filp->private_data = dev;

    return 0;
}

static int _MDrv_IIC_Release(struct inode *inode, struct file *filp)
{
    IIC_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

static ssize_t _MDrv_IIC_Read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	S32 u32RetCountIIC = 0;
	IIC_Param_t IIC_ReadParam;
	U8 pRdBuffer[IIC_RD_BUF_SIZE];
	U8 pRdAddr[IIC_WR_ADDR_SIZE];

#if defined(CONFIG_COMPAT)
	IIC_Param_Compat __user *data32;
#endif
	IIC_Param __user *data;

#if defined(CONFIG_COMPAT)
	if(is_compat_task())
	{ //32bit

		data32 = compat_ptr(buf);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		mdrv_iic_compat_get_ion_allaction_data(data32, data);
		if( copy_from_user(&IIC_ReadParam, (IIC_Param_t *)data, sizeof(IIC_Param_t)) ) //(3) "'count'" is old size with 32bits stuct , need using 64bits struct size
		{
			printk("\033[0;32;31m %s %d :ERROR: Copy from User Error! \033[m\n", __func__, __LINE__);
			return -1;
		}
#if 0
#if 0
printk("\033[0;32;31m [Ian] IIC_ReadParam %s %d %d %d %d %d %d %d %d %d %p %d\033[m\n",__func__,__LINE__,
			IIC_ReadParam.u8IdIIC,
			IIC_ReadParam.u8ClockIIC,
			IIC_ReadParam.u8SlaveIdIIC,
			IIC_ReadParam.u8AddrSizeIIC,
			IIC_ReadParam.u8AddrIIC[0],
			IIC_ReadParam.u8AddrIIC[1],
			IIC_ReadParam.u8AddrIIC[2],
			IIC_ReadParam.u8AddrIIC[3],
			IIC_ReadParam.u8pbufIIC,
			IIC_ReadParam.u32DataSizeIIC);
#else
                 udelay(1000); //delay ok
                //no delay may read fail randomly and read data is always 0
#endif
#endif
		if ((IIC_ReadParam.u8AddrSizeIIC > IIC_MAX_ADDR_SIZE) || (IIC_ReadParam.u32DataSizeIIC > IIC_MAX_BUF_SIZE))
		{
			printk("\033[0;32;31m %s %d :ERROR: Address size or data size is over! \033[m\n", __func__, __LINE__);
			return -1;
		}

		if( IIC_ReadParam.u32DataSizeIIC > IIC_RD_BUF_SIZE )
		{
			IIC_ReadParam.u8pbufIIC = kmalloc(IIC_ReadParam.u32DataSizeIIC, GFP_KERNEL);
		}
		else
		{
			IIC_ReadParam.u8pbufIIC = pRdBuffer;
		}

		if (IIC_ReadParam.u8AddrSizeIIC > IIC_WR_ADDR_SIZE)
		{
			IIC_ReadParam.u8AddrIIC = kmalloc(IIC_ReadParam.u8AddrSizeIIC, GFP_KERNEL);
		}
		else
		{
			IIC_ReadParam.u8AddrIIC = pRdAddr;
		}

		if ((IIC_ReadParam.u8pbufIIC == NULL) || (IIC_ReadParam.u8AddrIIC == NULL))
		{
			printk("\033[0;32;31m %s %d u8pbufIIC or u8AddrIIC is null \033[m\n", __func__, __LINE__);
			u32RetCountIIC = -1;
			goto end;
		}

		if (copy_from_user(IIC_ReadParam.u8AddrIIC, ((IIC_Param_Compat *)buf)->u8AddrIIC, IIC_ReadParam.u8AddrSizeIIC))
		{
			printk("\033[0;32;31m %s %d :ERROR: Copy from User Error! \033[m\n", __func__, __LINE__);
			u32RetCountIIC = -1;
			goto end;
		}

		if (IIC_ReadParam.u8IdIIC >= IIC_NUM_OF_MAX)
		{
			printk("\033[0;32;31m %s %d ERROR: invalid IIC BUS ID \033[m\n", __func__, __LINE__);
			u32RetCountIIC = -1;
			goto end;
		}
#if 0   //Not support HWI2C yet.
		if (IIC_ReadParam.u8IdIIC <= IIC_NUM_OF_HW)
		{
			u32RetCountIIC = MDrv_HW_IIC_ReadBytes(IIC_ReadParam.u8IdIIC,IIC_ReadParam.u8SlaveIdIIC,IIC_ReadParam.u8AddrSizeIIC, IIC_ReadParam.u8AddrIIC, IIC_ReadParam.u32DataSizeIIC,IIC_ReadParam.u8pbufIIC);
		}
		else
#endif
		{
			u32RetCountIIC = MDrv_SW_IIC_ReadBytes(IIC_ReadParam.u8IdIIC,IIC_ReadParam.u8SlaveIdIIC,IIC_ReadParam.u8AddrSizeIIC, IIC_ReadParam.u8AddrIIC, IIC_ReadParam.u32DataSizeIIC,IIC_ReadParam.u8pbufIIC);
		}

		if ((0 == u32RetCountIIC) && (IIC_ReadParam.u32DataSizeIIC > 0))
		{
			u32RetCountIIC = IIC_ReadParam.u32DataSizeIIC;
			if (copy_to_user(data, &IIC_ReadParam, sizeof(IIC_Param_t)))
			{
				printk("\033[0;32;31m %s %d :ERROR: Copy to User Error! \033[m\n", __func__, __LINE__);
				u32RetCountIIC = -1;
				goto end;
			}

			if (copy_to_user(data32->u8pbufIIC, data->u8pbufIIC, sizeof(U8)*u32RetCountIIC))
			{
				printk("\033[0;32;31m %s %d :ERROR: Copy to User Error! \033[m\n", __func__, __LINE__);
				u32RetCountIIC = -1;
				goto end;

			}
			IIC_DBG(printk("\033[0;32;31m %s %d \033[m\n",__func__,__LINE__));
		}
		IIC_DBG(printk("_MDrv_IIC_Read() -> u32RetCountIIC=%d\n", u32RetCountIIC));
	}
	else
#endif
        {
		if( copy_from_user(&IIC_ReadParam, (IIC_Param_t *)buf, sizeof(IIC_Param_t)) ) //(3) "'count'" is old size with 32bits stuct , need using 64bits struct size
		{
			printk("\033[0;32;31m %s %d :ERROR: Copy from User Error! \033[m\n", __func__, __LINE__);
			return -1;
		}
#if 0
#if 0
		printk("\033[0;32;31m [Ian] IIC_ReadParam %s %d %d %d %d %d %d %d %d %d %p %d\033[m\n",__func__,__LINE__,
			IIC_ReadParam.u8IdIIC,
			IIC_ReadParam.u8ClockIIC,
			IIC_ReadParam.u8SlaveIdIIC,
			IIC_ReadParam.u8AddrSizeIIC,
			IIC_ReadParam.u8AddrIIC[0],
			IIC_ReadParam.u8AddrIIC[1],
			IIC_ReadParam.u8AddrIIC[2],
			IIC_ReadParam.u8AddrIIC[3],
			IIC_ReadParam.u8pbufIIC,
			IIC_ReadParam.u32DataSizeIIC);
#else
		udelay(1000); //delay ok
		//no delay may read fail randomly and read data is always 0
#endif
#endif

		if ((IIC_ReadParam.u8AddrSizeIIC > IIC_MAX_ADDR_SIZE) || (IIC_ReadParam.u32DataSizeIIC > IIC_MAX_BUF_SIZE))
		{
			printk("\033[0;32;31m %s %d :ERROR: Address size or data size is over! \033[m\n", __func__, __LINE__);
			return -1;
		}

		if( IIC_ReadParam.u32DataSizeIIC > IIC_RD_BUF_SIZE )
		{
			IIC_ReadParam.u8pbufIIC = kmalloc(IIC_ReadParam.u32DataSizeIIC, GFP_KERNEL);
		}
		else
		{
			IIC_ReadParam.u8pbufIIC = pRdBuffer;
		}

		if (IIC_ReadParam.u8AddrSizeIIC > IIC_WR_ADDR_SIZE)
		{
			IIC_ReadParam.u8AddrIIC = kmalloc(IIC_ReadParam.u8AddrSizeIIC, GFP_KERNEL);
		}
		else
		{
			IIC_ReadParam.u8AddrIIC = pRdAddr;
		}

		if ((IIC_ReadParam.u8pbufIIC == NULL) || (IIC_ReadParam.u8AddrIIC == NULL))
		{
			printk("\033[0;32;31m %s %d u8pbufIIC or u8AddrIIC is null \033[m\n", __func__, __LINE__);
			u32RetCountIIC = -1;
			goto end;
		}

		if( IIC_ReadParam.u8IdIIC >= IIC_NUM_OF_MAX )
		{
			printk("\033[0;32;31m %s %d ERROR: invalid IIC BUS ID \033[m\n", __func__, __LINE__);
			u32RetCountIIC = -1;
			goto end;
		}
#if 0   //Not support hwi2c yet
		if(IIC_ReadParam.u8IdIIC <= IIC_NUM_OF_HW)
		{
			u32RetCountIIC = MDrv_HW_IIC_ReadBytes(IIC_ReadParam.u8IdIIC,IIC_ReadParam.u8SlaveIdIIC,IIC_ReadParam.u8AddrSizeIIC, IIC_ReadParam.u8AddrIIC, IIC_ReadParam.u32DataSizeIIC,IIC_ReadParam.u8pbufIIC);
			printk("[kernel-HWI2C][%s][%d]\n", __FUNCTION__, __LINE__);
		}
		else
#endif
		{
			u32RetCountIIC = MDrv_SW_IIC_ReadBytes(IIC_ReadParam.u8IdIIC,IIC_ReadParam.u8SlaveIdIIC,IIC_ReadParam.u8AddrSizeIIC, IIC_ReadParam.u8AddrIIC, IIC_ReadParam.u32DataSizeIIC,IIC_ReadParam.u8pbufIIC);
		}

		if ((0 == u32RetCountIIC) && (IIC_ReadParam.u32DataSizeIIC > 0))
		{
			u32RetCountIIC = IIC_ReadParam.u32DataSizeIIC;
			if (copy_to_user(((IIC_Param_t *)buf)->u8pbufIIC, IIC_ReadParam.u8pbufIIC, sizeof(U8)*IIC_ReadParam.u32DataSizeIIC))
			{
				printk("\033[0;32;31m %s %d :ERROR: Copy to User Error! \033[m\n", __func__, __LINE__);
				u32RetCountIIC = -1;
				goto end;
			}
			IIC_DBG(printk("\033[0;32;31m %s %d \033[m\n",__func__,__LINE__));
		}
	}

end:
	if ((IIC_ReadParam.u8pbufIIC != pRdBuffer) && (IIC_ReadParam.u8pbufIIC != NULL))
	{
		kfree(IIC_ReadParam.u8pbufIIC);
	}
	if ((IIC_ReadParam.u8AddrIIC != pRdAddr) && (IIC_ReadParam.u8AddrIIC != NULL))
	{
		kfree(IIC_ReadParam.u8AddrIIC);
	}
	return u32RetCountIIC;
}

static ssize_t _MDrv_IIC_Write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	U32 u32RetCountIIC = 0;
	IIC_Param_t IIC_WriteParam;
	U8 pWrAddr[IIC_WR_ADDR_SIZE];
	U8 pWrBuffer[IIC_WR_BUF_SIZE];

#if defined(CONFIG_COMPAT)
	if(is_compat_task())
	{
		IIC_Param_Compat __user *data32;
		IIC_Param __user *data;

		data32 = compat_ptr(buf);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		mdrv_iic_compat_get_ion_allaction_data(data32, data);

		if( copy_from_user(&IIC_WriteParam, (IIC_Param_t *)data, sizeof(IIC_Param_t)) )
		{
			printk("\033[0;32;31m %s %d :ERROR: Copy from User Error! \033[m\n", __func__, __LINE__);
			return -1;
		}

		if ((IIC_WriteParam.u8AddrSizeIIC > IIC_MAX_ADDR_SIZE) || (IIC_WriteParam.u32DataSizeIIC > IIC_MAX_BUF_SIZE))
		{
			printk("\033[0;32;31m %s %d :ERROR: Address size or data size is over! \033[m\n", __func__, __LINE__);
			return -1;
		}

		if (IIC_WriteParam.u8AddrSizeIIC > IIC_WR_ADDR_SIZE)
		{
			IIC_WriteParam.u8AddrIIC = kmalloc(IIC_WriteParam.u8AddrSizeIIC, GFP_KERNEL);
		}
		else
		{
			IIC_WriteParam.u8AddrIIC = pWrAddr;
		}

		if (IIC_WriteParam.u32DataSizeIIC > IIC_WR_BUF_SIZE)
		{
			IIC_WriteParam.u8pbufIIC = kmalloc(IIC_WriteParam.u32DataSizeIIC, GFP_KERNEL);
		}
		else
		{
			IIC_WriteParam.u8pbufIIC = pWrBuffer;
		}

		if ((IIC_WriteParam.u8pbufIIC == NULL) || (IIC_WriteParam.u8AddrIIC == NULL))
		{
			printk("\033[0;32;31m %s %d u8pbufIIC or u8AddrIIC is null \033[m\n", __func__, __LINE__);
			u32RetCountIIC = -1;
			goto end;
		}

		if (copy_from_user(IIC_WriteParam.u8pbufIIC, ((IIC_Param_Compat *)buf)->u8pbufIIC, IIC_WriteParam.u32DataSizeIIC))
		{
			printk("\033[0;32;31m %s %d :ERROR: Copy from User Error! \033[m\n", __func__, __LINE__);
			u32RetCountIIC = -1;
			goto end;
		}

		if (copy_from_user(IIC_WriteParam.u8AddrIIC, ((IIC_Param_Compat *)buf)->u8AddrIIC, IIC_WriteParam.u8AddrSizeIIC))
		{
			printk("\033[0;32;31m %s %d :ERROR: Copy from User Error! \033[m\n", __func__, __LINE__);
			u32RetCountIIC = -1;
			goto end;
		}

		if (IIC_WriteParam.u8IdIIC >= IIC_NUM_OF_MAX)
		{
			printk("\033[0;32;31m %s %d ERROR: invalid IIC BUS ID \033[m\n", __func__, __LINE__);
			u32RetCountIIC = -1;
			goto end;
		}
#if 0   //Not support HWI2C
		if (IIC_WriteParam.u8IdIIC <= IIC_NUM_OF_HW)
		{
			u32RetCountIIC = MDrv_HW_IIC_WriteBytes(IIC_WriteParam.u8IdIIC, IIC_WriteParam.u8SlaveIdIIC, IIC_WriteParam.u8AddrSizeIIC, IIC_WriteParam.u8AddrIIC, IIC_WriteParam.u32DataSizeIIC, IIC_WriteParam.u8pbufIIC);
			printk("[kernel-HWI2C][%s][%d]\n", __FUNCTION__, __LINE__);
		}
		else
#endif
		{
			u32RetCountIIC = MDrv_SW_IIC_WriteBytes(IIC_WriteParam.u8IdIIC, IIC_WriteParam.u8SlaveIdIIC, IIC_WriteParam.u8AddrSizeIIC, IIC_WriteParam.u8AddrIIC, IIC_WriteParam.u32DataSizeIIC, IIC_WriteParam.u8pbufIIC);
		}

		if (0 == u32RetCountIIC)
		{
			u32RetCountIIC = IIC_WriteParam.u32DataSizeIIC;
		}
	}
	else
#endif
	{
		if( copy_from_user(&IIC_WriteParam, (IIC_Param_t *)buf, count) )
		{
			printk("\033[0;32;31m %s %d :ERROR: Copy from User Error! \033[m\n", __func__, __LINE__);
			return -1;
		}
#if 0
		printk("u8IdIIC[%d] u8SlaveIdIIC[%x] u8ClockIIC[%d] u8AddrSizeIIC[%d] u32DataSizeIIC[%d]\n",
			IIC_WriteParam.u8IdIIC,\
			IIC_WriteParam.u8SlaveIdIIC,\
			IIC_WriteParam.u8ClockIIC,\
			IIC_WriteParam.u8AddrSizeIIC,\
			IIC_WriteParam.u32DataSizeIIC);
#endif

		if ((IIC_WriteParam.u8AddrSizeIIC > IIC_MAX_ADDR_SIZE) || (IIC_WriteParam.u32DataSizeIIC > IIC_MAX_BUF_SIZE))
		{
			printk("\033[0;32;31m %s %d :ERROR: Address size or data size is over! \033[m\n", __func__, __LINE__);
			return -1;
		}

		if (IIC_WriteParam.u8AddrSizeIIC > IIC_WR_ADDR_SIZE)
		{
			IIC_WriteParam.u8AddrIIC = kmalloc(IIC_WriteParam.u8AddrSizeIIC, GFP_KERNEL);
		}
		else
		{
			IIC_WriteParam.u8AddrIIC = pWrAddr;
		}

		if (IIC_WriteParam.u32DataSizeIIC > IIC_WR_BUF_SIZE)
		{
			IIC_WriteParam.u8pbufIIC = kmalloc(IIC_WriteParam.u32DataSizeIIC, GFP_KERNEL);
		}
		else
		{
			IIC_WriteParam.u8pbufIIC = pWrBuffer;
		}

		if ((IIC_WriteParam.u8pbufIIC == NULL) || (IIC_WriteParam.u8AddrIIC == NULL))
		{
			printk("\033[0;32;31m %s %d u8pbufIIC or u8AddrIIC is null \033[m\n", __func__, __LINE__);
			u32RetCountIIC = -1;
			goto end;
		}

		if (copy_from_user(IIC_WriteParam.u8pbufIIC, ((IIC_Param_t *)buf)->u8pbufIIC, IIC_WriteParam.u32DataSizeIIC))
		{
			printk("\033[0;32;31m %s %d :ERROR: Copy from User Error! \033[m\n", __func__, __LINE__);
			u32RetCountIIC = -1;
			goto end;
		}

		if (copy_from_user(IIC_WriteParam.u8AddrIIC, ((IIC_Param_t *)buf)->u8AddrIIC, IIC_WriteParam.u8AddrSizeIIC))
		{
			printk("\033[0;32;31m %s %d :ERROR: Copy from User Error! \033[m\n", __func__, __LINE__);
			u32RetCountIIC = -1;
			goto end;
		}

		if (IIC_WriteParam.u8IdIIC >= IIC_NUM_OF_MAX)
		{
			printk("_MDrv_IIC_Write():ERROR: invalid IIC ID\n");
			u32RetCountIIC = -1;
			goto end;
		}
#if 0
		if (IIC_WriteParam.u8IdIIC <= IIC_NUM_OF_HW)
		{
			u32RetCountIIC = MDrv_HW_IIC_WriteBytes(IIC_WriteParam.u8IdIIC, IIC_WriteParam.u8SlaveIdIIC, IIC_WriteParam.u8AddrSizeIIC, IIC_WriteParam.u8AddrIIC, IIC_WriteParam.u32DataSizeIIC, IIC_WriteParam.u8pbufIIC);
			printk("[kernel-HWI2C][%s][%d]\n", __FUNCTION__, __LINE__);
		}
		else
#endif
		{
			u32RetCountIIC = MDrv_SW_IIC_WriteBytes(IIC_WriteParam.u8IdIIC, IIC_WriteParam.u8SlaveIdIIC, IIC_WriteParam.u8AddrSizeIIC, IIC_WriteParam.u8AddrIIC, IIC_WriteParam.u32DataSizeIIC, IIC_WriteParam.u8pbufIIC);
		}

		if (0 == u32RetCountIIC)
		{
			u32RetCountIIC = IIC_WriteParam.u32DataSizeIIC;
		}
	}

end:
	if ((IIC_WriteParam.u8pbufIIC != pWrBuffer) && (IIC_WriteParam.u8pbufIIC != NULL))
	{
		kfree(IIC_WriteParam.u8pbufIIC);
	}
	if ((IIC_WriteParam.u8AddrIIC != pWrAddr) && (IIC_WriteParam.u8AddrIIC != NULL))
	{
		kfree(IIC_WriteParam.u8AddrIIC);
	}

	return u32RetCountIIC;
}

static unsigned int _MDrv_IIC_Poll(struct file *filp, poll_table *wait)
{
    IIC_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}


static int _MDrv_IIC_Fasync(int fd, struct file *filp, int mode)
{
    IIC_PRINT("%s is invoked\n", __FUNCTION__);

	return 0;//fasync_helper(fd, filp, mode, &IICDev.async_queue);
}

#if defined(CONFIG_COMPAT)
static long Compat_MDrv_IIC_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
	switch(cmd)
	{
        case Compat_MDRV_IIC_INIT:
        case Compat_MDRV_IIC_BUSCFG:
		{
			return filp->f_op->unlocked_ioctl(filp, cmd,(unsigned long)compat_ptr(arg));
		}
        case Compat_MDRV_IIC_CLOCK:
		{
			IIC_Param_Compat __user *data32;
			IIC_Param __user *data;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;
			mdrv_iic_compat_get_ion_allaction_data(data32, data);
			return filp->f_op->unlocked_ioctl(filp, MDRV_IIC_CLOCK,(unsigned long)data);
		}
		case Compat_MDRV_IIC_ENABLE:
		{
			IIC_Param_Compat __user *data32;
			IIC_Param __user *data;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;
			mdrv_iic_compat_get_ion_allaction_data(data32, data);


			return filp->f_op->unlocked_ioctl(filp, MDRV_IIC_ENABLE,(unsigned long)data);
		}
		default:
			printk("\033[0;32;31m %s %d Ioctl cmd %d undefined \033[m\n",__func__,__LINE__,cmd);
			return -ENOIOCTLCMD;
	}

}
#endif

#ifdef HAVE_UNLOCKED_IOCTL
static long _MDrv_IIC_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int _MDrv_IIC_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
    S32 s32Err= 0;
	IIC_Param_t IIC_param;
    #if (defined(CONFIG_MSTAR_TITANIA)||defined(CONFIG_MSTAR_TITANIA2))
    #else
	IIC_BusCfg_t IIC_BusObj;
    #endif

    IIC_PRINT("%s is invoked\n", __FUNCTION__);

    /*
     * extract the type and number bitfields, and don't decode
     * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
     */
    if ((IIC_IOC_MAGIC!= _IOC_TYPE(cmd)) || (_IOC_NR(cmd)> IIC_IOC_MAXNR))
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
        s32Err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        s32Err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }
    if (s32Err)
    {
        return -EFAULT;
    }


    //PROBE_IO_ENTRY(MDRV_MAJOR_IIC, _IOC_NR(cmd));

    switch(cmd)
    {
        case MDRV_IIC_INIT:
            IIC_PRINT("ioctl: MDRV_IIC_INIT\n");
            MDrv_IIC_Init();
            break;
        case MDRV_IIC_CLOCK:
            IIC_PRINT("ioctl: MDRV_IIC_CLOCK\n");
            if (copy_from_user(&IIC_param, (IIC_Param_t __user *)arg, sizeof(IIC_Param_t)))
            {
                //PROBE_IO_EXIT(MDRV_MAJOR_IIC, _IOC_NR(cmd));
 	            return -EFAULT;
            }

			if( IIC_param.u8IdIIC >= IIC_NUM_OF_MAX )
			{
			    //PROBE_IO_EXIT(MDRV_MAJOR_IIC, _IOC_NR(cmd));
				return -EFAULT;
			}
#if 0       //Not support HWI2C yet
            if( IIC_param.u8IdIIC <= IIC_NUM_OF_HW )
			{
				MDrv_HW_IIC_Clock_Select(IIC_param.u8ClockIIC);
			}
			else
#endif
			{
				MDrv_SW_IIC_SetSpeed(IIC_param.u8IdIIC, IIC_param.u8ClockIIC);
			}
            break;

    #if (defined(CONFIG_MSTAR_TITANIA)||defined(CONFIG_MSTAR_TITANIA2))
    #else
		case MDRV_IIC_ENABLE:
			IIC_PRINT("ioctl: MDRV_IIC_ENABLE\n");
			if (copy_from_user(&IIC_param, (IIC_Param_t __user *)arg, sizeof(IIC_Param_t)))
			{
			    //PROBE_IO_EXIT(MDRV_MAJOR_IIC, _IOC_NR(cmd));
				return -EFAULT;
            }
			if(( IIC_param.u8IdIIC <= IIC_NUM_OF_HW )||( IIC_param.u8IdIIC >= IIC_NUM_OF_MAX ))
			{
                IIC_PRINT("SW I2C Channel is BTW (%d) ~ %d\n",IIC_NUM_OF_HW+1,IIC_NUM_OF_MAX-1);
			    //PROBE_IO_EXIT(MDRV_MAJOR_IIC, _IOC_NR(cmd));
				return -EFAULT;
			}
            IIC_PRINT("MDRV_IIC_ENABLE: SW I2C Channel = %d\n",IIC_param.u8IdIIC);
            IIC_PRINT("MDRV_IIC_ENABLE: enable = %d\n",IIC_param.u8ClockIIC);
            MDrv_SW_IIC_Enable( IIC_param.u8IdIIC, ((IIC_param.u8ClockIIC) ? TRUE : FALSE) );
			break;

            case MDRV_IIC_BUSCFG:
            IIC_PRINT("ioctl: MDRV_IIC_BUSCFG\n");
            if (copy_from_user(&IIC_BusObj, (IIC_BusCfg_t __user *)arg, sizeof(IIC_BusCfg_t)))
            {
                //PROBE_IO_EXIT(MDRV_MAJOR_IIC, _IOC_NR(cmd));
 	            return -EFAULT;
            }
            IIC_PRINT("SW I2C u8ChIdx --> is %d\n",IIC_BusObj.u8ChIdx);
			if(( IIC_BusObj.u8ChIdx <= IIC_NUM_OF_HW )||( IIC_BusObj.u8ChIdx >= IIC_NUM_OF_MAX ))
			{
                IIC_PRINT("SW I2C Channel is BTW (%d) ~ %d\n",IIC_NUM_OF_HW+1,IIC_NUM_OF_MAX-1);
			    //PROBE_IO_EXIT(MDRV_MAJOR_IIC, _IOC_NR(cmd));
				return -EFAULT;
			}
            MDrv_SW_IIC_Init_Bus((IIC_BusCfg_t*)&IIC_BusObj, IIC_BusObj.u8ChIdx);
            break;
#endif
        default:
            IIC_PRINT("ioctl: unknown command\n");
            //PROBE_IO_EXIT(MDRV_MAJOR_IIC, _IOC_NR(cmd));
            return -ENOTTY;
    }
    //PROBE_IO_EXIT(MDRV_MAJOR_IIC, _IOC_NR(cmd));
    return 0;
}

static int mod_iic_init(void)
{
	S32 s32Ret;
	dev_t dev;

	if (bModIICisInit == 0)
	{
		printk("%s is invoked\n", __FUNCTION__);

		if (IICDev.s32MajorIIC)
		{
			dev = MKDEV(IICDev.s32MajorIIC, IICDev.s32MinorIIC);
			s32Ret = register_chrdev_region(dev, MOD_IIC_DEVICE_COUNT, MOD_IIC_NAME);
		}
		else
		{
			s32Ret = alloc_chrdev_region(&dev, IICDev.s32MinorIIC, MOD_IIC_DEVICE_COUNT, MOD_IIC_NAME);
			IICDev.s32MajorIIC = MAJOR(dev);
		}

		if ( 0 > s32Ret)
		{
			IIC_PRINT("Unable to get major %d\n", IICDev.s32MajorIIC);
			return s32Ret;
		}
		cdev_init(&IICDev.cDevice, &IICDev.IICFop);
		if (0 != (s32Ret= cdev_add(&IICDev.cDevice, dev, MOD_IIC_DEVICE_COUNT)))
		{
			IIC_PRINT("Unable add a character device\n");
			unregister_chrdev_region(dev, MOD_IIC_DEVICE_COUNT);
			return s32Ret;
		}
#if defined(CONFIG_I2C_CHARDEV)
		chrdev_modiic_class = class_create(THIS_MODULE, "ModIIC-dev");
		if (IS_ERR(chrdev_modiic_class))
		{
			s32Ret = PTR_ERR(chrdev_modiic_class);
			return s32Ret;
		}

		device_create(chrdev_modiic_class, NULL, MKDEV(IICDev.s32MajorIIC, IICDev.s32MinorIIC), NULL, "ModIIC");
#endif
		bModIICisInit = 1;
	}

	return 0;
}

static void mod_iic_exit(void)
{
	IIC_PRINT("%s is invoked\n", __FUNCTION__);

#if defined(CONFIG_I2C_CHARDEV)
	device_destroy(chrdev_modiic_class, MKDEV(IICDev.s32MajorIIC, IICDev.s32MinorIIC));
	class_destroy(chrdev_modiic_class);
#endif
	cdev_del(&IICDev.cDevice);
	unregister_chrdev_region(MKDEV(IICDev.s32MajorIIC, IICDev.s32MinorIIC), MOD_IIC_DEVICE_COUNT);
}

/* linux i2c driver framework */

static int mstar_i2c_xfer_msg(struct i2c_adapter *adap, struct i2c_msg *msg, int stop)
{
    struct mstar_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
    int ret = 0;
    u8 *pu8Buf = msg->buf;
    u16 u16BufLen = msg->len;

    msg->addr = msg->addr << 1;

    dev_dbg(&i2c_dev->pdev->dev, "I2C message to: 0x%04x, len: %d,flags: 0x%04x, stop: %d\n",
            msg->addr, msg->len, msg->flags, stop);

    if (msg->len == 0)
    {
        dev_dbg(&i2c_dev->pdev->dev, "I2C message size is 0\n");
        return -EINVAL;
    }

    if (msg->flags & I2C_M_RD)
    {
        /* send read message */
        if (i2c_dev->I2cAttr.iic_mode == MSTAR_SWIIC)
        {
            ret = MDrv_SW_IIC_ReadByteArrayDirectly(i2c_dev->I2cAttr.bus_index, msg->addr, u16BufLen, pu8Buf, stop);
        }
        else
        {
            ret = MDrv_HW_IIC_ReadByteArrayDirectly(i2c_dev->I2cAttr.hw_port, msg->addr, u16BufLen, pu8Buf, stop);
        }
        /* stop at every memssage */
        IIC_DBG(printk("swi2c recieved %u bytes from slave %u\n", ret, (u32)msg->addr));
    }
    else
    {
        /* send write message */
        //TODO: setup mutex or embedded into MDrv
        if (i2c_dev->I2cAttr.iic_mode == MSTAR_SWIIC)
        {
            ret = MDrv_SW_IIC_WriteByteArrayDirectly(i2c_dev->I2cAttr.bus_index, msg->addr, u16BufLen, pu8Buf, stop);
        }
        else
        {
            ret = MDrv_HW_IIC_WriteByteArrayDirectly(i2c_dev->I2cAttr.hw_port, msg->addr, u16BufLen, pu8Buf, stop);
        }
        IIC_DBG(printk("swi2c sent %u bytes to slave %u\n", ret, (u32)msg->addr));
    }

    return ret;
}


static int mstar_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
    struct mstar_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
    int ret = -1;
    int i;

    if (i2c_dev->I2cAttr.iic_mode == MSTAR_HWIIC)
        HWIIC_MUTEX_LOCK(i2c_dev->I2cAttr.hw_port);
    else if (i2c_dev->I2cAttr.iic_mode == MSTAR_SWIIC)
    {
        LOCK_HW_SEM();
        IIC_MUTEX_S_LOCK();
        IIC_MUTEX_LOCK(i2c_dev->I2cAttr.bus_index);
    }

    for (i = 0; i < num; i++)
    {
        /* NOTICE: place stop at the end of the last msg */
        ret = mstar_i2c_xfer_msg(adap, &msgs[i], (i == (num - 1)));
        if (ret < 0)
        {
            break;
        }
    }

    if (i2c_dev->I2cAttr.iic_mode == MSTAR_HWIIC)
        HWIIC_MUTEX_UNLOCK(i2c_dev->I2cAttr.hw_port);
    else if (i2c_dev->I2cAttr.iic_mode == MSTAR_SWIIC)
    {
        IIC_MUTEX_UNLOCK(i2c_dev->I2cAttr.bus_index);
        IIC_MUTEX_S_UNLOCK();
        UNLOCK_HW_SEM();
    }

    return (ret > 0)? i : ret;
}

static u32 mstar_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm mstar_i2c_algo = {
	.master_xfer    = mstar_i2c_xfer,
	.functionality  = mstar_i2c_func,
};

#if 0
static int test_swi2c_xfer(struct i2c_adapter *adap)
{
	int ret;

	struct i2c_msg write_msg[2];
	struct i2c_msg read_msg[2];

	u8 data[6] = {0x53, 0x45, 0x52, 0x44, 0x42};
	u8 id_buf[2] = {0xFF, 0xFF};
	// PRADO I2C debug mode
	write_msg[0].addr = 0x80;   // slave address, in this case PMIC
	write_msg[0].flags = 0x0;   // i2c bus write
	write_msg[0].len = 5;       // commands length = 5
	write_msg[0].buf = data;    // commands

	//send command to PRADO
	ret = mstar_swi2c_xfer(adap, &write_msg[0], 1);

	//PRADO use I2C_USE_CFG
	data[0] = 0x7f;
	write_msg[0].len = 1;       // data length = 1
	write_msg[0].buf = data;    // data
	ret = mstar_swi2c_xfer(adap, &write_msg[0], 1);

	//PRADO use I2C_USE_CFG
	data[0] = 0x7d;
	write_msg[0].len = 1;       // data length = 1
	write_msg[0].buf = data;    // data
	ret = mstar_swi2c_xfer(adap, &write_msg[0], 1);

	//PRADO use I2C_USE_CFG
	data[0] = 0x50;
	write_msg[0].len = 1;       // data length = 1
	write_msg[0].buf = data;    // data
	ret = mstar_swi2c_xfer(adap, &write_msg[0], 1);

	//PRADO use I2C_USE_CFG
	data[0] = 0x55;
	write_msg[0].len = 1;       // data length = 1
	write_msg[0].buf = data;    // data
	ret = mstar_swi2c_xfer(adap, &write_msg[0], 1);

	//PRADO use I2C_USE_CFG
	data[0] = 0x35;
	write_msg[0].len = 1;       // data length = 1
	write_msg[0].buf = data;    // data
	ret = mstar_swi2c_xfer(adap, &write_msg[0], 1);

	// PRADO set target read address
	data[0] = 0x10;                 // command IC to read
	data[1] = 0xC0;                 // target register 0xC0
	read_msg[0].addr = 0x80;        // slave address, in this case PRADO 0x80
	read_msg[0].flags = 0x0;        // i2c bus write
	read_msg[0].len = 2;            // address length = 2
	read_msg[0].buf = data;         // slave register address

	// PRADO read
	read_msg[1].addr = 0x80;        // slave address, in this case PRADO 0x80
	read_msg[1].flags = I2C_M_RD;   // i2c bus read
	read_msg[1].len = 2;            // length of data to read = 2
	read_msg[1].buf = id_buf;       // read to id_buf

	ret = mstar_swi2c_xfer(adap, &read_msg[0], 1);
	ret = mstar_swi2c_xfer(adap, &read_msg[1], 1);

	if(ret < 0) {
	    IIC_PRINT("test FAILED\n");
	    return 0;
	}
	if(id_buf[1] == 0x9A) {
	    IIC_PRINT("test SUCCESS!\n");
	    return ret;
	} else {
	    IIC_PRINT("test FAILED! recieved data: %x %x\n", id_buf[0], id_buf[1]);
	}

	return ret;
}
#endif

static void mstar_swi2c_init(struct mstar_i2c_dev *i2c_dev)
{

	/* init MDrv IIC backend */
	IIC_BusCfg_t bus_cfg;
	bus_cfg.u8Enable = FALSE;
	bus_cfg.u16PadSCL = i2c_dev->I2cAttr.scl_pad;
	bus_cfg.u16PadSDA = i2c_dev->I2cAttr.sda_pad;
	bus_cfg.u16DefDelay = i2c_dev->I2cAttr.def_delay;
	bus_cfg.u16SpeedKHz = i2c_dev->I2cAttr.speed_khz;
	bus_cfg.u16Retries = i2c_dev->I2cAttr.retries;
	bus_cfg.u8ChIdx = i2c_dev->I2cAttr.bus_index;

	bus_cfg.u8Hw_Port = 0;
	bus_cfg.u32PadMux = 0;

	IIC_PRINT("[%s]i2c_dev->retries = %u i2c_dev->bus_index= %u\n", __FUNCTION__, i2c_dev->I2cAttr.retries,
			i2c_dev->I2cAttr.bus_index);
	MDrv_SW_IIC_Init_Setup();
	MDrv_SW_IIC_Init_Bus(&bus_cfg, i2c_dev->I2cAttr.bus_index);
}

static void mstar_hwi2c_init(struct mstar_i2c_dev *i2c_dev)
{
	/* init MDrv IIC backend */
	IIC_BusCfg_t bus_cfg;
	bus_cfg.u8Hw_Port = i2c_dev->I2cAttr.hw_port;
	bus_cfg.u32PadMux = i2c_dev->I2cAttr.pad_mux;
	bus_cfg.u16DefDelay = i2c_dev->I2cAttr.def_delay;
	bus_cfg.u16SpeedKHz = i2c_dev->I2cAttr.speed_khz;
	bus_cfg.u16Retries = i2c_dev->I2cAttr.retries;
	IIC_PRINT("[%s]i2c_dev->retries = %u i2c_dev->bus_index= %u\n", __FUNCTION__, i2c_dev->I2cAttr.retries,
			i2c_dev->I2cAttr.bus_index);
	MDrv_HW_IIC_Init(&bus_cfg, i2c_dev->I2cAttr.bus_index);
}

static int mstar_swi2c_parse_dt(struct device_node *dn, struct mstar_i2c_dev *i2c_dev)
{
    u32 iic_mode = 0;
    u32 sda_pad = 0;
    u32 scl_pad = 0;
    u32 hw_port = 0;
    u32 pad_mux = 0;
    u32 def_delay = 0;
    u32 speed_khz = 0;
    u32 retries = 0;
    u32 bus_index = 0;

    /*
    (1)	Parse Device Tree
    */
    if (0 != of_property_read_u32(dn, "iic-mode", &iic_mode) ||
        0 != of_property_read_u32(dn, "sda-gpio", &sda_pad) ||
        0 != of_property_read_u32(dn, "scl-gpio", &scl_pad) ||
        0 != of_property_read_u32(dn, "hw-port", &hw_port) ||
        0 != of_property_read_u32(dn, "pad-mux", &pad_mux) ||
        0 != of_property_read_u32(dn, "def-delay", &def_delay) ||
        0 != of_property_read_u32(dn, "retries", &retries) ||
        0 != of_property_read_u32(dn, "bus-index", &bus_index) ||
        0 != of_property_read_u32(dn, "speed-khz", &speed_khz))
	{
		printk(KERN_ERR "[mstar][kswi2c]Parse dts error\n");
		return -ENXIO;
	}

    /*
    (2)	Judge Soft-Ware IIC GPIO pin status
    */
    if (iic_mode == MSTAR_SWIIC && (!gpio_is_valid(sda_pad) || !gpio_is_valid(scl_pad)))
	{
        printk(KERN_ERR "%s: invalid GPIO pins, sda=%d/scl=%d\n",__FUNCTION__, sda_pad, scl_pad);
		return -ENODEV;
	}

	IIC_PRINT("[SWI2C]index: %u, retries: %u, sda:%u, scl:%u, speed_khz:%u def_delay:%u\n",
				bus_index, retries, sda_pad, scl_pad, speed_khz, def_delay);

    /*
    (3)	Assign value
    */
    i2c_dev->I2cAttr.iic_mode = (u16)iic_mode;
    i2c_dev->I2cAttr.sda_pad = (u16)sda_pad;
    i2c_dev->I2cAttr.scl_pad = (u16)scl_pad;
    i2c_dev->I2cAttr.pad_mux = (u16)pad_mux;
    i2c_dev->I2cAttr.hw_port = (u8)hw_port;
    i2c_dev->I2cAttr.def_delay = (u16)def_delay;
    i2c_dev->I2cAttr.speed_khz = (u16)speed_khz;
    i2c_dev->I2cAttr.retries = (u16)retries;
    i2c_dev->I2cAttr.bus_index = (u16)bus_index;
    return 0;

}
static int mstar_iic_drv_suspend(struct platform_device *dev, pm_message_t state)
{
    return 0;
}
static int mstar_iic_drv_resume(struct platform_device *dev)
{
    MDrv_HW_IIC_Resume();
    return 0;
}

static u8 first_probe = 0;

static int mstar_iic_drv_probe(struct platform_device *pdev)
{
	int retval = 0, i = 0;
	struct mstar_i2c_dev *i2c_dev;
	struct i2c_adapter *adap;
	struct device_node *dn;

	IIC_PRINT("[ki2c] %s\n",__FUNCTION__);

	/* init swi2c device struct */
	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(struct mstar_i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
	{
		dev_err(&pdev->dev, "Unable to allocate memory for mstar-i2c\n");
		retval = -ENOMEM;
		goto err_init_swi2c;
	}

	/* parse OF i2c info */
    #if defined (CONFIG_OF)
	dn = pdev->dev.of_node;
	if (dn)
	{
		retval = mstar_swi2c_parse_dt(dn, i2c_dev);
		if (retval < 0)
		{
			IIC_PRINT("[ki2c] unable to parse device tree\n");
			goto err_init_swi2c;
		}
	}
    #else
    struct mstar_i2c_attr *data = dev_get_platdata(&pdev->dev);
            if (!data){
                dev_err(&pdev->dev, "could not get resource\n");
                return -EINVAL;
            }

            i2c_dev->I2cAttr.iic_mode = data->iic_mode;
            i2c_dev->I2cAttr.sda_pad = data->sda_pad;
            i2c_dev->I2cAttr.scl_pad = data->scl_pad;
            i2c_dev->I2cAttr.hw_port = data->hw_port;
            i2c_dev->I2cAttr.pad_mux = data->pad_mux;
            i2c_dev->I2cAttr.def_delay = data->def_delay;
            i2c_dev->I2cAttr.speed_khz = data->speed_khz;
            i2c_dev->I2cAttr.retries = data->retries;
            i2c_dev->I2cAttr.bus_index = data->bus_index;
#endif

    if (i2c_dev->I2cAttr.iic_mode)
    {
        IIC_PRINT("[HWI2C Mode]\n");
        IIC_PRINT("[HWI2C]index: %u, retries: %u, reg:%u, padmux:%u, speed_khz:%u def_delay:%u\n",
        i2c_dev->I2cAttr.bus_index, i2c_dev->I2cAttr.retries, i2c_dev->I2cAttr.hw_port, i2c_dev->I2cAttr.pad_mux, i2c_dev->I2cAttr.speed_khz, i2c_dev->I2cAttr.def_delay);
    }
    else
    {
        IIC_PRINT("[SWI2C Mode]\n");
        IIC_PRINT("[SWI2C]index: %u, retries: %u, sda:%u, clk:%u, speed_khz:%u def_delay:%u\n",
				i2c_dev->I2cAttr.bus_index, i2c_dev->I2cAttr.retries, i2c_dev->I2cAttr.sda_pad, i2c_dev->I2cAttr.scl_pad, i2c_dev->I2cAttr.speed_khz, i2c_dev->I2cAttr.def_delay);
    }


    /*
    Reserve GPIO pin for SWIIC.
    Thus, User space could not control gpios of SWIIC vie SYSFS
    */
    if (i2c_dev->I2cAttr.iic_mode == MSTAR_SWIIC)
    {
	retval = devm_gpio_request(&pdev->dev, i2c_dev->I2cAttr.sda_pad, "sda");
        if (retval < 0)
        {
		dev_err(&pdev->dev, "request gpio pad %u failed\n", i2c_dev->I2cAttr.sda_pad);
		goto err_init_swi2c;
	}
	retval = devm_gpio_request(&pdev->dev, i2c_dev->I2cAttr.scl_pad, "scl");
        if (retval < 0)
        {
		dev_err(&pdev->dev, "request gpio pad %u failed\n", i2c_dev->I2cAttr.scl_pad);
		goto err_gpio;
	}
    }

	i2c_dev->pdev = pdev;
	platform_set_drvdata(pdev, i2c_dev);

	/* init swi2c MDrv backend */
    if (i2c_dev->I2cAttr.iic_mode == MSTAR_SWIIC)
    {
	mstar_swi2c_init(i2c_dev);
    }
    else
    {
        mstar_hwi2c_init(i2c_dev);
    }

	/* register i2c adapter and bus algorithm */
	adap = &i2c_dev->adapter;
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strlcpy(adap->name, "Mstar I2C adapter", sizeof(adap->name));
	adap->dev.parent = &pdev->dev;
    adap->algo = &mstar_i2c_algo;
	adap->dev.of_node = pdev->dev.of_node;
	adap->retries = i2c_dev->I2cAttr.retries;
	adap->timeout = 2 * HZ;
	adap->nr = i2c_dev->I2cAttr.bus_index;

	i2c_set_adapdata(adap, i2c_dev);

	retval = i2c_add_adapter(adap);
    if (retval)
    {
	    dev_err(&pdev->dev, "failed to add i2c adapter");
	    goto err_gpio;
	}

	/* register i2c bus to dt */
 #if defined (CONFIG_OF)
	of_i2c_register_devices(adap);
#endif

	dev_info(&pdev->dev, "Init mstar i2c is done\n");
#if 0
	test_swi2c_xfer(adap);
#endif
	/* all set, now init the ioctl interface */
	mod_iic_init();

	if (first_probe == 0)
	{
		first_probe = 1;
		for(i=0;i<HWI2C_PORTS;i++)
			HWIIC_MUTEX_CREATE(i);
	}
	return 0;

err_gpio:
	gpio_free(i2c_dev->I2cAttr.scl_pad);
	gpio_free(i2c_dev->I2cAttr.sda_pad);
err_init_swi2c:
	return retval;
}

static int mstar_iic_drv_remove(struct platform_device *pdev)
{
	struct mstar_i2c_dev *dev = platform_get_drvdata(pdev);

	if( !(pdev->name) || strcmp(pdev->name,"Mstar-swi2c")
		|| pdev->id!=0)
	{
		return -1;
	}

	mod_iic_exit();
	i2c_del_adapter(&dev->adapter);
	gpio_free(dev->I2cAttr.scl_pad);
	gpio_free(dev->I2cAttr.sda_pad);
	pdev->dev.platform_data = NULL;
	return 0;
}


#if defined (CONFIG_OF)
static struct of_device_id mstariic_of_device_ids[] = {
		{.compatible = "mstar,swi2c"},
		{},
};
#endif

static struct platform_driver Mstar_iic_driver = {
	.probe 		= mstar_iic_drv_probe,
	.remove		= mstar_iic_drv_remove,
	.suspend		= mstar_iic_drv_suspend,
	.resume		= mstar_iic_drv_resume,

	.driver = {
#if defined(CONFIG_OF)
		.of_match_table = mstariic_of_device_ids,
#endif
		.name	= "Mstar-iic",
		.owner	= THIS_MODULE,
	}
};


static int __init mstar_iic_drv_init_module(void)
{
    int retval=0;
    printk("[kswi2c] %s\n",__FUNCTION__);
    retval = platform_driver_register(&Mstar_iic_driver);
    return retval;
}

static void __exit mstar_iic_drv_exit_module(void)
{
	platform_driver_unregister(&Mstar_iic_driver);
}


module_init(mstar_iic_drv_init_module);
module_exit(mstar_iic_drv_exit_module);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("IIC driver");
MODULE_LICENSE("GPL");
