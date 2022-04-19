/*******************************************************************************
 *
 * FILE NAME          : MxL661_OEM_Drv.h
 * 
 * AUTHOR             : Dong Liu 
 *
 * DATE CREATED       : 11/23/2011
 *
 * DESCRIPTION        : Header file for MxL661_OEM_Drv.c
 *
 *******************************************************************************
 *                Copyright (c) 2010, MaxLinear, Inc.
 ******************************************************************************/

#ifndef __MxL661_OEM_DRV_H__
#define __MxL661_OEM_DRV_H__

/******************************************************************************
    Include Header Files
    (No absolute paths - paths handled by make file)
******************************************************************************/

#include "MaxLinearDataTypes.h"
#include "MxL_Debug.h"

/******************************************************************************
    Macros
******************************************************************************/

/******************************************************************************
    User-Defined Types (Typedefs)
******************************************************************************/

/******************************************************************************
    Global Variable Declarations
******************************************************************************/

/******************************************************************************
    Prototypes
******************************************************************************/
#if 1 //kdrv
#include "mdrv_types.h"
#endif
void MxLWare661_OEM_I2C_channel_setting(UINT8 bus_num);
MXL_STATUS MxLWare661_OEM_WriteRegister(UINT8 I2cSlaveAddr, UINT8 RegAddr, UINT8 RegData);
MXL_STATUS MxLWare661_OEM_ReadRegister(UINT8 I2cSlaveAddr, UINT8 RegAddr, UINT8 *DataPtr);
void MxLWare661_OEM_Sleep(UINT16 DelayTimeInMs);  

#endif /* __MxL661_OEM_DRV_H__*/




