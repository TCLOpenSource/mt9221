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

#ifndef _HAL_LP_STANDBY_H_
#define _HAL_LP_STANDBY_H_
//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/slab.h>
#include "MsTypes.h"


//-------------------------------------------------------------------------------------------------
// Define & data type
//-------------------------------------------------------------------------------------------------

#define MSTAR_STANDBY_INFO(fmt, args...)    printk(KERN_INFO "[LP STANDBY INFO] %s:%d " fmt,__FUNCTION__,__LINE__, ##args)
#define MSTAR_STANDBY_WARN(fmt, args...)    printk(KERN_WARNING "[LP STANDBY WARNING] %s:%d " fmt,__FUNCTION__,__LINE__, ##args)
#define MSTAR_STANDBY_ERR(fmt, args...)     printk(KERN_ERR "[LP STANDBY ERROR] %s:%d " fmt,__FUNCTION__,__LINE__, ##args)
#if defined(MSTAR_STANDBY_DEBUG)
#define MSTAR_STANDBY_DBG(fmt, args...)     printk(KERN_INFO "[LP STANDBY DEBUG] %s:%d " fmt,__FUNCTION__,__LINE__, ##args)
#else
#define MSTAR_STANDBY_DBG(fmt, args...)
#endif

#if defined(MSTAR_STANDBY_DEBUG)
#define MSTAR_STANDBY_DEBUG_REG             0x000604
#endif

#define MSTAR_STANDBY_BACKLIGHT_GPIO        5

void HAL_Standby_Setup(void);
void HAL_Standby_Restore(void);
int HAL_Standby_Suspend(void);
void HAL_Standby_Resume(void);

#endif
