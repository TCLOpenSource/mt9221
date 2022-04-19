////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2007 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (¡§MStar Confidential Information¡¨) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @file   mdrv_dish.h
/// @brief  DISH Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _DRV_DISH_H_
#define _DRV_DISH_H_

#include <asm/types.h>
#include "mdrv_types.h"
#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------
typedef struct __attribute__((__packed__))
{
    U32 u32param;
    U32 retval;
} MS_DISH_Common_Info, *PMS_DISH_Common_Info;

typedef struct __attribute__((__packed__))
{
    U8 u8ChIdx;
    U16 u16PadSCL;
    U16 u16PadSDA;
    U8 u8SlaveId;
    U16 u16SpeedKHz;
    U16 u16LNBEnablePinNum;
    U8 u8LNBEnablePinInv;
    U16 u16LNBSelectPin1Num;
    U8 u8LNBSelectPin1Inv;
    U16 u16LNBSelectPin2Num;
    U8 u8LNBSelectPin2Inv;
    U32 retval;
} MS_DISH_DishInit_Info, *PMS_DISH_DishInit_Info;

typedef struct __attribute__((__packed__))
{
    U32 u32LNBPower;
    U32 retval;
} MS_DISH_SetLNBPower_Info, *PMS_DISH_SetLNBPower_Info;

typedef struct __attribute__((__packed__))
{
    U32* pu32LNBPower_bOn;
    U32 retval;
} MS_DISH_GetLNBPower_Info, *PMS_DISH_GetLNBPower_Info;

typedef struct __attribute__((__packed__))
{
    U32 u32Set22KOnOff;
    U32 retval;
} MS_DISH_Set22KOnOff_Info, *PMS_DISH_Set22KOnOff_Info;

typedef struct __attribute__((__packed__))
{
    U32* pu32Get22KStatus;
    U32 retval;
} MS_DISH_Get22KStatus_Info, *PMS_DISH_Get22KStatus_Info;

#if defined(CONFIG_COMPAT)
typedef struct __attribute__((__packed__))
{
    compat_uptr_t pu32LNBPower_bOn;
    U32 retval;
} COMPAT_MS_DISH_GetLNBPower_Info, *COMPAT_PMS_DISH_GetLNBPower_Info;

typedef struct __attribute__((__packed__))
{
    compat_uptr_t pu32Get22KStatus;
    U32 retval;
} COMPAT_MS_DISH_Get22KStatus_Info, *COMPAT_PMS_DISH_Get22KStatus_Info;

#endif

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
//#define TUNER_IOC_MAGIC    't'
#define DISH_IOC_MAGIC     'd'//alex

#define MDRV_DISH_Init                  _IOR(DISH_IOC_MAGIC, 0, MS_DISH_DishInit_Info)
#define MDRV_DISH_SetLNBPower    _IOR(DISH_IOC_MAGIC, 1, MS_DISH_SetLNBPower_Info)
#if defined(CONFIG_COMPAT)
#define MDRV_DISH_GetLNBPower    _IOWR(DISH_IOC_MAGIC, 2, COMPAT_MS_DISH_GetLNBPower_Info)
#else
#define MDRV_DISH_GetLNBPower    _IOWR(DISH_IOC_MAGIC, 2, MS_DISH_GetLNBPower_Info)
#endif
#define MDRV_DISH_Set22KOnOff    _IOWR(DISH_IOC_MAGIC, 3, MS_DISH_Set22KOnOff_Info)
#if defined(CONFIG_COMPAT)
#define MDRV_DISH_Get22KStatus    _IOWR(DISH_IOC_MAGIC, 4, COMPAT_MS_DISH_Get22KStatus_Info)
#else
#define MDRV_DISH_Get22KStatus    _IOWR(DISH_IOC_MAGIC, 4, MS_DISH_Get22KStatus_Info)
#endif
#define MDRV_DISH_IsOverCurrent    _IOWR(DISH_IOC_MAGIC, 5, MS_DISH_Common_Info)

#define DISH_IOC_MAXNR    6
#define GPIOPinNumNotSupport 10000 //  if GPIO pin is un-defined use this to protect that kernel driver will not control this pin  
#define GPIOPinKernelOffSet 1 // the GPIO pin number defined in SN and kernel have offset 1

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------


#endif // _DRV_TUNER_H_

