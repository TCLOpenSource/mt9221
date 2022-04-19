////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2018 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (MStar Confidential Information) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

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

