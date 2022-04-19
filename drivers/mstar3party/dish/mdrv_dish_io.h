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
/// @file   mdrv_dish_io.h
/// @brief  DISH_LNB Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _DRV_DISH_IO_H_
#define _DRV_DISH_IO_H_

#include <asm/types.h>
#include "mdrv_types.h"
#include "mdrv_dish_datatype.h"

//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

#define UNIT_DISH_MODULE_CHAR 20
//Dish module group
const char DISH_MODULE_NAME[][UNIT_DISH_MODULE_CHAR] =
{
    "Mstar-RT5047","Mstar-TPS65235"
};
//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
int MDrv_DISH_Init(int minor, s_Dish_dev_info *devDishInit);
int MDrv_DISH_SetLNBPower(int minor, EN_SAT_LNBPOWER_TYPE eLNBPower);
int MDrv_DISH_GetLNBPower(int minor, EN_SAT_LNBPOWER_TYPE *eLNBPower);
int MDrv_DISH_Set22KOnOff(int minor, int bOn);
int MDrv_DISH_Get22KStatus(int minor, int *bOn);
int MDrv_DISH_IsOverCurrent(int minor);
int MDrv_DISH_Suspend(void);
int MDrv_DISH_Resume(void);
int MDrv_DISH_GetDISHMdbInfo(void);
int MDrv_DISH_WriteDISHMdbInfo(void);


#endif // _DRV_DISH_IO_H_

