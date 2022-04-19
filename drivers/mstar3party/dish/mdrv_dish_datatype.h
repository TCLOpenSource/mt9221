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
/// @file   mdrv_dish_datatype.h
/// @brief  DISH_LNB Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_DISH_DATATYPE_H_
#define _MDRV_DISH_DATATYPE_H_

//------------------------------------------------------------
//enum
//------------------------------------------------------------
///TONE BUREST TYPE
typedef enum
{
    E_TONE0,                            ///  TYPE: TONE0
    E_TONE1,                            ///  TYPE: TONE1
} EN_SAT_TONEBUREST_TYPE;

///LNB POWER TYPE
typedef enum
{
    E_LNB_POWER_OFF,                    ///  POWER OFF
    E_LNB_POWER_13V,                    ///  POWER 13V
    E_LNB_POWER_14V,                    ///  POWER 14V
    E_LNB_POWER_18V,                    ///  POWER 18V
    E_LNB_POWER_19V,                    ///  POWER 19V
} EN_SAT_LNBPOWER_TYPE;

typedef enum
{
    // Before init success.
    E_CLOSE,
    // Init success.
    E_WORKING,
    // Under suspend mode.
    E_SUSPEND,
    // Number of dish status.
    E_DISH_STATUS_NUM
} E_DISH_STATUS;

typedef struct
{
    E_DISH_STATUS                 e_status;
    EN_SAT_LNBPOWER_TYPE          e_lnbpower;
    EN_SAT_TONEBUREST_TYPE        e_toneburest;
    U8                            u8_slaveID;
    U8                            u8_i2c_bus;
    U16                           u16LNBEnablePinNum;
    U8                            u8LNBEnablePinInv;
    U16                           u16LNBSelectPin1Num;
    U8                            u8LNBSelectPin1Inv;
    U16                           u16LNBSelectPin2Num;
    U8                            u8LNBSelectPin2Inv;
}s_Dish_dev_info;

#endif






