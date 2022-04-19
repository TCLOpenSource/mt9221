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
/// file    mdrv_gpio.c
/// @brief  GPIO Driver Interface
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
#include <linux/spinlock.h>
#include <asm/io.h>

#include "mst_devid.h"

#include "mdrv_gpio.h"
#include "mhal_gpio_reg.h"
#include "mhal_gpio.h"

//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
#if 0//reserved
static   struct semaphore        pm_gpio_mutex ;
#define  _MUTEX_INIT()           init_MUTEX( &pm_gpio_mutex )
#define _MUTEX_LOCK()            down( &pm_gpio_mutex )
#define _MUTEX_UNLOCK()          up( &pm_gpio_mutex )
#endif

spinlock_t mstar_gpio_lock;
/* Is it required to disable interrupt? */
#define LOCK_GPIO() \
        unsigned long flags;\
        spin_lock_irqsave(&mstar_gpio_lock, flags);

#define UNLOCK_GPIO()\
        spin_unlock_irqrestore(&mstar_gpio_lock, flags);

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
/// GPIO chiptop initialization
/// @return None
/// @note   Called only once at system initialization
//-------------------------------------------------------------------------------------------------
void MDrv_GPIO_Init(void)
{
  MHal_GPIO_Init();
  spin_lock_init(&mstar_gpio_lock);
}
EXPORT_SYMBOL(MDrv_GPIO_Init);


//-------------------------------------------------------------------------------------------------
/// select one pad to set
/// @param  u8IndexGPIO              \b IN:  pad index
/// @return None
/// @note
//-------------------------------------------------------------------------------------------------
void MDrv_GPIO_WriteRegBit(U32 u32Reg, U8 u8Enable, U8 u8BitMsk)
{
    MHal_GPIO_WriteRegBit(u32Reg,u8Enable,u8BitMsk);
}
EXPORT_SYMBOL(MDrv_GPIO_WriteRegBit);

//-------------------------------------------------------------------------------------------------
/// select one pad to set
/// @param  u8IndexGPIO              \b IN:  pad index
/// @return None
/// @note
//-------------------------------------------------------------------------------------------------
void MDrv_GPIO_Pad_Set(U32 u32IndexGPIO)
{
    MHal_GPIO_Pad_Set(u32IndexGPIO);
}
EXPORT_SYMBOL(MDrv_GPIO_Pad_Set);

//-------------------------------------------------------------------------------------------------
/// enable output for selected one pad
/// @param  u8IndexGPIO              \b IN:  pad index
/// @return None
/// @note
//-------------------------------------------------------------------------------------------------
void MDrv_GPIO_Pad_Oen(U32 u32IndexGPIO)
{
   LOCK_GPIO();
   MHal_GPIO_Pad_Oen(u32IndexGPIO);
   UNLOCK_GPIO();
}
EXPORT_SYMBOL(MDrv_GPIO_Pad_Oen);

//-------------------------------------------------------------------------------------------------
/// enable input for selected one pad
/// @param  u8IndexGPIO              \b IN:  pad index
/// @return None
/// @note
//-------------------------------------------------------------------------------------------------
void MDrv_GPIO_Pad_Odn(U32 u32IndexGPIO)
{
   LOCK_GPIO();
   MHal_GPIO_Pad_Odn(u32IndexGPIO);
   UNLOCK_GPIO();
}
EXPORT_SYMBOL(MDrv_GPIO_Pad_Odn);

//-------------------------------------------------------------------------------------------------
/// read data from selected one pad
/// @param  u8IndexGPIO              \b IN:  pad index
/// @return None
/// @note
//-------------------------------------------------------------------------------------------------
U8 MDrv_GPIO_Pad_Read(U32 u32IndexGPIO)
{
   U8 ret;
   LOCK_GPIO();
   ret = MHal_GPIO_Pad_Level(u32IndexGPIO);
   UNLOCK_GPIO();
   return ret;
}
EXPORT_SYMBOL(MDrv_GPIO_Pad_Read);

//-------------------------------------------------------------------------------------------------
/// read pad direction for selected one pad
/// @param  u8IndexGPIO              \b IN:  pad index
/// @return None
/// @note
//-------------------------------------------------------------------------------------------------
U8 MDrv_GPIO_Pad_InOut(U32 u32IndexGPIO)
{
   U8 ret;
   LOCK_GPIO();
   ret = MHal_GPIO_Pad_InOut(u32IndexGPIO);
   UNLOCK_GPIO();
   return ret;
}
EXPORT_SYMBOL(MDrv_GPIO_Pad_InOut);

//-------------------------------------------------------------------------------------------------
/// output pull high for selected one pad
/// @param  u8IndexGPIO              \b IN:  pad index
/// @return None
/// @note
//-------------------------------------------------------------------------------------------------
void MDrv_GPIO_Pull_High(U32 u32IndexGPIO)
{
   LOCK_GPIO();
   MHal_GPIO_Pull_High(u32IndexGPIO);
   UNLOCK_GPIO();
}
EXPORT_SYMBOL(MDrv_GPIO_Pull_High);

//-------------------------------------------------------------------------------------------------
/// output pull low for selected one pad
/// @param  u8IndexGPIO              \b IN:  pad index
/// @return None
/// @note
//-------------------------------------------------------------------------------------------------
void MDrv_GPIO_Pull_Low(U32 u32IndexGPIO)
{
   LOCK_GPIO();
   MHal_GPIO_Pull_Low(u32IndexGPIO);
   UNLOCK_GPIO();
}
EXPORT_SYMBOL(MDrv_GPIO_Pull_Low);

//-------------------------------------------------------------------------------------------------
/// output set high for selected one pad
/// @param  u8IndexGPIO              \b IN:  pad index
/// @return None
/// @note
//-------------------------------------------------------------------------------------------------
void MDrv_GPIO_Set_High(U32 u32IndexGPIO)
{
   LOCK_GPIO();
   MHal_GPIO_Set_High(u32IndexGPIO);
   UNLOCK_GPIO();
}
EXPORT_SYMBOL(MDrv_GPIO_Set_High);

//-------------------------------------------------------------------------------------------------
/// output set low for selected one pad
/// @param  u8IndexGPIO              \b IN:  pad index
/// @return None
/// @note
//-------------------------------------------------------------------------------------------------
void MDrv_GPIO_Set_Low(U32 u32IndexGPIO)
{
   LOCK_GPIO();
   MHal_GPIO_Set_Low(u32IndexGPIO);
   UNLOCK_GPIO();
}
EXPORT_SYMBOL(MDrv_GPIO_Set_Low);

//-------------------------------------------------------------------------------------------------
/// output set low for selected one pad
/// @param  u8IndexGPIO              \b IN:  pad index
/// @return None
/// @note
//-------------------------------------------------------------------------------------------------
void MDrv_GPIO_Set_Input(U32 u32IndexGPIO)
{
   LOCK_GPIO();
   MHal_GPIO_Set_Input(u32IndexGPIO);
   UNLOCK_GPIO();
}
EXPORT_SYMBOL(MDrv_GPIO_Set_Input);

#ifdef CONFIG_EXT_INTERRUPT_SUPPORT
static void (*_GPIOCallback)(void);
static irqreturn_t gpio_irq_handler(int irq, void *data)
{
   _GPIOCallback();
   return IRQ_HANDLED;//dead code
}

int request_gpio_irq(int gpio_num, irq_handler_t handler, unsigned long irqflags, void *dev_id)
{
   return (MHal_GPIO_Enable_Interrupt(gpio_num, irqflags, handler, dev_id));
}
EXPORT_SYMBOL(request_gpio_irq);

int free_gpio_irq(int gpio_num, void *dev_id)
{
   return (MHal_GPIO_Disable_Interrupt(gpio_num, dev_id));
}
EXPORT_SYMBOL(free_gpio_irq);

void Mstar_Gpio_Irq_Attach(int gpio_num, unsigned long irqflags,void *ptr)
{
  /*Export to Utpa2k.ko usage;Utopia MsOS_SetEvent register here*/
  _GPIOCallback=ptr;
  /*Utpa rising enum is 0,but 1 for kernel;Utpa falling enum is 1,but 2 for kernel*/
  request_gpio_irq(gpio_num,gpio_irq_handler,irqflags+1,NULL);
}
EXPORT_SYMBOL(Mstar_Gpio_Irq_Attach);
#endif
