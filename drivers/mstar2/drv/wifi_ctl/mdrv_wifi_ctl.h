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

#ifndef _DRV_WIFI_CTL_H_
#define _DRV_WIFI_CTL_H_
//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/types.h>
#include "MsTypes.h"

//-------------------------------------------------------------------------------------------------
// Define & data type
//-------------------------------------------------------------------------------------------------

//#define MTK_WIFI_CTL_DEBUG

#define MTK_WIFI_CTL_INFO(fmt, args...)    printk(KERN_INFO "[WIFI CONTROL INFO] %s:%d " fmt,__FUNCTION__,__LINE__, ##args)
#define MTK_WIFI_CTL_WARN(fmt, args...)    printk(KERN_WARNING "[WIFI CONTROL WARNING] %s:%d " fmt,__FUNCTION__,__LINE__, ##args)
#define MTK_WIFI_CTL_ERR(fmt, args...)     printk(KERN_ERR "[WIFI CONTROL ERROR] %s:%d " fmt,__FUNCTION__,__LINE__, ##args)
#if defined(MTK_WIFI_CTL_DEBUG)
#define MTK_WIFI_CTL_DBG(fmt, args...)     printk(KERN_INFO "[WIFI CONTROL DEBUG] %s:%d " fmt,__FUNCTION__,__LINE__, ##args)
#else
#define MTK_WIFI_CTL_DBG(fmt, args...)
#endif

typedef struct
{
    u32 gpio;
    u32 delay_ms;
    u8 invert;
    u8 PowerDown;
} MTK_Wifi_Ctl_Parm;

#endif

