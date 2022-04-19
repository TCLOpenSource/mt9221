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
/// @file   mdrv_tuner.h
/// @brief  TUNER Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _DRV_TUNER_IO_H_
#define _DRV_TUNER_IO_H_

#include <asm/types.h>
#include "mdrv_types.h"
#include "mdrv_tuner_datatype.h"

//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

#define UNIT_TUNER_MODULE_CHAR 20
//Tuner module group for DVB-T/T2,ATSC,DTMB,ISDB-T
const char TUNER_MODULE_NAME[][UNIT_TUNER_MODULE_CHAR] =
{
    "Mstar-MXL661","Mstar-Si2151","Mstar-R842"
};

//Tuner module group for DVB-S/S2
const char TUNER_MODULE_NAME_DVBS[][UNIT_TUNER_MODULE_CHAR] =
{
    "Mstar-AV2012","Mstar-AV2017","Mstar-AV2018","Mstar-RT710"
};

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
int MDrv_TUNER_Connect(int minor);
int MDrv_TUNER_Disconnect(int minor);
int MDrv_TUNER_ATV_SetTune(int minor, U32 u32FreqKHz, U32 eBand, U32 eMode, U8 otherMode);
int MDrv_TUNER_DVBS_SetTune(int minor, U16 u16CenterFreqMHz, U32 u32SymbolRateKs);
int MDrv_TUNER_DTV_SetTune(int minor, U32 freq, U32 eBandWidth, U32 eMode);
int MDrv_TUNER_ExtendCommand(int minor, U8 u8SubCmd, U32 u32Param1, U32 u32Param2, void* pvoidParam3);
int MDrv_TUNER_TunerInit(int minor, s_Tuner_dev_info *devTunerInit);
int MDrv_TUNER_ConfigAGCMode(int minor, U32 eMode);
int MDrv_TUNER_SetTunerInScanMode(int minor, U32 bScan);
int MDrv_TUNER_SetTunerInFinetuneMode(int minor, U32 bFinetune);
int MDrv_TUNER_GetCableStatus(int minor, U32 eStatus);
int MDrv_TUNER_TunerReset(int minor);
int MDrv_TUNER_IsLocked(int minor);
int MDrv_TUNER_GetRSSI(int minor, U16 u16Gain, U8 u8DType);
int MDrv_TUNER_Suspend(void);
int MDrv_TUNER_Resume(void);
ssize_t MDrv_TUNER_GetTUNERMdbInfo(struct file *file, char __user *buf, size_t count, loff_t *ppos);
ssize_t MDrv_TUNER_WriteTUNERMdbInfo(struct file *file, const char __user *buf, size_t count, loff_t *ppos);


#endif // _DRV_TUNER_IO_H_

