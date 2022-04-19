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

/*! \defgroup G_BDMA BDMA interface
    \ingroup  G_PERIPHERAL

    \brief

    BDMA is Byte-aligned data transfer DMA engine. It can execute data transmission between MIU to MIU,
    flash to MIU, SRAM to MIU?圯tc.

    <b>Features</b>

    - Byte-aligned data transfer DMA engine
    - Address increase or decrease while DMA
    - Two command channels (optional). Note that when 2 command channels exist, they are processed as FIFO (First trigger, first service)
    - Several data width for device (1/2/4/8/16 bytes)
    - Bundled source device: MIU/Flash
    - Bundled destination device: MIU/VDMCU/DSP/TSP/HK51 1K SRAM

    <b> Address decreasing mode:</b> \n
    Avoid data overlapped on the same storage. Ex: Copy data A to B.
    \image html drvBDMA_pic.png

    <b> BDMA Block Diagram: </b> \n
    \image html drvBDMA_pic2.png

    <b> Operation Code Flow: </b> \n
    -# Prepare BDMA setting for each operation
    -# Set and start BDMA in command handle
    -# Get BDMA free channel
    -# Set setting
    -# Trigger BDMA
    -# BDMA done by polling
    \image html drvBDMA_pic3.png


    \defgroup G_BDMA_INIT Initialization Task relative
    \ingroup  G_BDMA
    \defgroup G_BDMA_COMMON Common Task relative
    \ingroup  G_BDMA
    \defgroup G_BDMA_PS Pattern search relative
    \ingroup  G_BDMA
    \defgroup G_BDMA_MOBF  MOBF relative
    \ingroup  G_BDMA
    \defgroup G_BDMA_ToBeModified BDMA api to be modified
    \ingroup  G_BDMA
    \defgroup G_BDMA_ToBeRemove BDMA api to be removed
    \ingroup  G_BDMA
*/

#ifndef _DRVBDMA_H_
#define _DRVBDMA_H_

////////////////////////////////////////////////////////////////////////////////
/// @file drvBDMA.h
/// @author MStar Semiconductor Inc.
/// @brief Byte DMA control driver
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Header Files
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C"
{
#endif


#include <stdbool.h>

////////////////////////////////////////////////////////////////////////////////
// Define & data type
////////////////////////////////////////////////////////////////////////////////
#define MSIF_BDMA_LIB_CODE	{'B','D','M','A'}    //Lib code
#define MSIF_BDMA_LIBVER		{'0','3'}            //LIB version
#define MSIF_BDMA_BUILDNUM      {'0','1'}            //Build Number
#define MSIF_BDMA_CHANGELIST   {'0','0','3','4','8','0','3','3'} //P4 ChangeList Number

#define BDMA_DRV_VERSION                /* Character String for DRV/API version             */  \
    MSIF_TAG,                           /* 'MSIF'                                           */  \
    MSIF_CLASS,                         /* '00'                                             */  \
    MSIF_CUS,                           /* 0x0000                                           */  \
    MSIF_MOD,                           /* 0x0000                                           */  \
    MSIF_CHIP,                                                                                  \
    MSIF_CPU,                                                                                   \
    MSIF_BDMA_LIB_CODE,                  /* IP__                                             */  \
    MSIF_BDMA_LIBVER,                    /* 0.0 ~ Z.Z                                        */  \
    MSIF_BDMA_BUILDNUM,                  /* 00 ~ 99                                          */  \
    MSIF_BDMA_CHANGELIST,                /* CL#                                              */  \
    MSIF_OS

//v: value n: shift n bits
#define _LShift(v, n)           ((v) << (n))
#define _RShift(v, n)           ((v) >> (n))

#define BDMA_SEARCH_ALL_MATCHED (0)
#define BDMA_CRC32_POLY         (0x04C11DB7)
#define BDMA_CRC16_POLY         (0x8005)
#define BDMA_CRC_SEED_0         (0)
#define BDMA_CRC_SEED_F         (0xFFFFFFFF)

/// Operation cfg
#define BDMA_OPCFG_DEF          		(0)
#define BDMA_OPCFG_INV_COPY     	(0x01)
#define BDMA_OPCFG_CRC_REFLECT  	(0x02)      //bit reflection of each input byte
#define BDMA_OPCFG_CRC_COPY     	(0x04)      //copy then crc check
#define BDMA_OPCFG_NOWAIT_COPY  	(0x08)      //copy then quit
#define BDMA_OPCFG_MOBF_PS  		(0x10)      //copy then quit

typedef enum _BDMA_DbgLv
{
    E_BDMA_DBGLV_NONE           //no debug message
    ,E_BDMA_DBGLV_PERFORMANCE   //show performance only
    ,E_BDMA_DBGLV_ERR_ONLY      //show error only
    ,E_BDMA_DBGLV_REG_DUMP      //show error & reg dump
    ,E_BDMA_DBGLV_INFO          //show error & informaiton
    ,E_BDMA_DBGLV_ALL           //show error, information & funciton name
}BDMA_DbgLv;

typedef enum _BDMA_Dev
{
    E_BDMA_DEV_MIU0
    ,E_BDMA_DEV_MIU1
    ,E_BDMA_DEV_SEARCH
    ,E_BDMA_DEV_CRC32
    ,E_BDMA_DEV_MEM_FILL
    ,E_BDMA_DEV_FLASH
    ,E_BDMA_DEV_DMDMCU
    ,E_BDMA_DEV_VDMCU
    ,E_BDMA_DEV_DSP
    ,E_BDMA_DEV_TSP
    ,E_BDMA_DEV_1KSRAM_HK51
    ,E_BDMA_DEV_MIU2
    ,E_BDMA_DEV_MIU3
    ,E_BDMA_DEV_NOT_SUPPORT
}BDMA_Dev;

typedef enum _BDMA_SrcDev
{
    E_BDMA_SRCDEV_MIU0          = E_BDMA_DEV_MIU0
    ,E_BDMA_SRCDEV_MIU1         = E_BDMA_DEV_MIU1
    ,E_BDMA_SRCDEV_MEM_FILL     = E_BDMA_DEV_MEM_FILL
    ,E_BDMA_SRCDEV_FLASH        = E_BDMA_DEV_FLASH
    ,E_BDMA_SRCDEV_MIU2         = E_BDMA_DEV_MIU2
    ,E_BDMA_SRCDEV_MIU3         = E_BDMA_DEV_MIU3
    ,E_BDMA_SRCDEV_NOT_SUPPORT  = E_BDMA_DEV_NOT_SUPPORT
}BDMA_SrcDev;

typedef enum _BDMA_DstDev
{
    E_BDMA_DSTDEV_MIU0          = E_BDMA_DEV_MIU0
    ,E_BDMA_DSTDEV_MIU1         = E_BDMA_DEV_MIU1
    ,E_BDMA_DSTDEV_SEARCH       = E_BDMA_DEV_SEARCH
    ,E_BDMA_DSTDEV_CRC32        = E_BDMA_DEV_CRC32
    ,E_BDMA_DSTDEV_DMDMCU       = E_BDMA_DEV_DMDMCU         //Demod
    ,E_BDMA_DSTDEV_VDMCU        = E_BDMA_DEV_VDMCU          //VD
    ,E_BDMA_DSTDEV_DSP          = E_BDMA_DEV_DSP
    ,E_BDMA_DSTDEV_TSP          = E_BDMA_DEV_TSP
    ,E_BDMA_DSTDEV_HK51_1KSRAM  = E_BDMA_DEV_1KSRAM_HK51
    ,E_BDMA_DSTDEV_MIU2         = E_BDMA_DEV_MIU2
    ,E_BDMA_DSTDEV_MIU3         = E_BDMA_DEV_MIU3
    ,E_BDMA_DSTDEV_NOT_SUPPORT  = E_BDMA_DEV_NOT_SUPPORT
}BDMA_DstDev;

#define BDMA_SET_CPYTYPE(src, dst) ((src & 0x0F) | _LShift((dst &0x0F), 8))

typedef enum _BDMA_CpyType
{
    E_BDMA_SDRAM2SDRAM          = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU0, E_BDMA_DEV_MIU0)
    ,E_BDMA_SDRAM2SDRAM1        = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU0, E_BDMA_DEV_MIU1)
    ,E_BDMA_SDRAM2SDRAM2        = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU0, E_BDMA_DEV_MIU2)
    ,E_BDMA_SDRAM2SDRAM3        = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU0, E_BDMA_DEV_MIU3)
    ,E_BDMA_SDRAM2DMDMCU        = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU0, E_BDMA_DEV_DMDMCU)
    ,E_BDMA_SDRAM2VDMCU         = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU0, E_BDMA_DEV_VDMCU)
    ,E_BDMA_SDRAM2DSP           = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU0, E_BDMA_DEV_DSP)
    ,E_BDMA_SDRAM2TSP           = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU0, E_BDMA_DEV_TSP)
    ,E_BDMA_SDRAM2SRAM1K_HK51   = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU0, E_BDMA_DEV_1KSRAM_HK51)
    ,E_BDMA_SDRAM12SDRAM        = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU1, E_BDMA_DEV_MIU0)
    ,E_BDMA_SDRAM12SDRAM1       = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU1, E_BDMA_DEV_MIU1)
    ,E_BDMA_SDRAM12SDRAM2       = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU1, E_BDMA_DEV_MIU2)
    ,E_BDMA_SDRAM12SDRAM3       = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU1, E_BDMA_DEV_MIU3)
    ,E_BDMA_SDRAM12DMDMCU       = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU1, E_BDMA_DEV_DMDMCU)
    ,E_BDMA_SDRAM12VDMCU        = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU1, E_BDMA_DEV_VDMCU)
    ,E_BDMA_SDRAM12DSP          = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU1, E_BDMA_DEV_DSP)
    ,E_BDMA_SDRAM12TSP          = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU1, E_BDMA_DEV_TSP)
    ,E_BDMA_SDRAM12SRAM1K_HK51  = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU1, E_BDMA_DEV_1KSRAM_HK51)
    ,E_BDMA_SDRAM22SDRAM        = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU2, E_BDMA_DEV_MIU0)
    ,E_BDMA_SDRAM22SDRAM1       = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU2, E_BDMA_DEV_MIU1)
    ,E_BDMA_SDRAM22SDRAM2       = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU2, E_BDMA_DEV_MIU2)
    ,E_BDMA_SDRAM22SDRAM3       = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU2, E_BDMA_DEV_MIU3)
    ,E_BDMA_SDRAM22DMDMCU       = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU2, E_BDMA_DEV_DMDMCU)
    ,E_BDMA_SDRAM22VDMCU        = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU2, E_BDMA_DEV_VDMCU)
    ,E_BDMA_SDRAM22DSP          = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU2, E_BDMA_DEV_DSP)
    ,E_BDMA_SDRAM22TSP          = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU2, E_BDMA_DEV_TSP)
    ,E_BDMA_SDRAM22SRAM1K_HK51  = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU2, E_BDMA_DEV_1KSRAM_HK51)
    ,E_BDMA_SDRAM32SDRAM        = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU3, E_BDMA_DEV_MIU0)
    ,E_BDMA_SDRAM32SDRAM1       = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU3, E_BDMA_DEV_MIU1)
    ,E_BDMA_SDRAM32SDRAM2       = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU3, E_BDMA_DEV_MIU2)
    ,E_BDMA_SDRAM32SDRAM3       = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU3, E_BDMA_DEV_MIU3)
    ,E_BDMA_SDRAM32DMDMCU       = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU3, E_BDMA_DEV_DMDMCU)
    ,E_BDMA_SDRAM32VDMCU        = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU3, E_BDMA_DEV_VDMCU)
    ,E_BDMA_SDRAM32DSP          = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU3, E_BDMA_DEV_DSP)
    ,E_BDMA_SDRAM32TSP          = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU3, E_BDMA_DEV_TSP)
    ,E_BDMA_SDRAM32SRAM1K_HK51  = BDMA_SET_CPYTYPE(E_BDMA_DEV_MIU3, E_BDMA_DEV_1KSRAM_HK51)
    ,E_BDMA_FLASH2SDRAM         = BDMA_SET_CPYTYPE(E_BDMA_DEV_FLASH, E_BDMA_DEV_MIU0)
    ,E_BDMA_FLASH2SDRAM1        = BDMA_SET_CPYTYPE(E_BDMA_DEV_FLASH, E_BDMA_DEV_MIU1)
    ,E_BDMA_FLASH2DMDMCU        = BDMA_SET_CPYTYPE(E_BDMA_DEV_FLASH, E_BDMA_DEV_DMDMCU)
    ,E_BDMA_FLASH2VDMCU         = BDMA_SET_CPYTYPE(E_BDMA_DEV_FLASH, E_BDMA_DEV_VDMCU)
    ,E_BDMA_FLASH2DSP           = BDMA_SET_CPYTYPE(E_BDMA_DEV_FLASH, E_BDMA_DEV_DSP)
    ,E_BDMA_FLASH2TSP           = BDMA_SET_CPYTYPE(E_BDMA_DEV_FLASH, E_BDMA_DEV_TSP)
    ,E_BDMA_FLASH2SRAMHK51      = BDMA_SET_CPYTYPE(E_BDMA_DEV_FLASH, E_BDMA_DEV_1KSRAM_HK51)
    ,E_BDMA_CPYTYPE_MAX
}BDMA_CpyType;

typedef enum _BDMA_Result
{
    E_BDMA_NOT_SUPPORT = -1
    ,E_BDMA_FAIL = 0
    ,E_BDMA_OK = 1
    ,E_BDMA_TIMEOUT
    ,E_BDMA_QUEUE_FULL
    ,E_BDMA_BUSY
}BDMA_Result;

typedef struct _BDMA_HwInfo
{
    bool bEnMIU1;        //MIU1
    bool bEnHost;        //bdma host
    bool bEnMemFill;     //memory fill
    bool bEnFlsCpy;      //flash copy
    bool bEnDevDw;       //bdma device data width
    bool bEnDmyWrCnt;    //bdma dummy wr count
    bool bEnDMDMCU;      //bdma to DeMod MCU
    bool bEnTSP;         //bdma to TSP
    bool bEnDSP;         //bdma to DSP
    bool bEnHK51_1KSRAM; //bdma to HK51_1KSRAM
}BDMA_HwInfo;

typedef struct _BDMA_Info
{
    U8          u8ChNum;
    U16         u16ChipVer;
    long         u32IOMap;
    long         phy64MIU1Base;
    U32      s32Mutex;
    bool        bInit;
    BDMA_DbgLv  eDbgLv;
    BDMA_HwInfo sHwCap;
}BDMA_Info;

typedef struct _BDMA_ChStatus
{
    bool bIsBusy;
    bool bIsInt;
    bool bIsFound;
}BDMA_ChStatus;

typedef struct _BDMA_Status
{
    bool            bInit;
    BDMA_DbgLv      eDbgLv;
    BDMA_ChStatus   sChSta[2];
}BDMA_Status;

typedef void (*BDMA_ISR_CBF)(BDMA_Result eRet);

BDMA_Result MDrv_BDMA_Init(long phy64Miu1Base);
U32 MDrv_BDMA_Search(long phy64Addr, U32 u32Len, U32 u32Pattern, U32 u32ExcluBit, BDMA_SrcDev eDev);
U32 MDrv_BDMA_CRC32(long phy64Addr, U32 u32Len, U32 u32Poly, U32 u32Seed, BDMA_SrcDev eDev, bool bReflect);

BDMA_Result MDrv_BDMA_MemCopy(long phy64SrcAddr, long phy64DstAddr, U32 u32Len);
BDMA_Result MDrv_BDMA_FlashCopy2Dram(long phy64FlashAddr, long phy64DramAddr, U32 u32Len);
//BDMA_Result MDrv_BDMA_CopyHnd(long phy64SrcAddr, long phy64DstAddr, U32 u32Len, BDMA_CpyType eCpyType, U8 u8OpCfg);
BDMA_Result MDrv_BDMA_CopyHnd_internal(long phy64SrcAddr, long phy64DstAddr, U32 u32Len, BDMA_CpyType eCpyType, U8 u8OpCfg, U8 Channel);

BDMA_Result MDrv_BDMA_WaitFlashDone(void);
void MDrv_BDMA_SetSPIOffsetForMCU(void);
void MDrv_BDMA_DumpCB(void *pvOpCB);
BDMA_Result MDrv_BDMA_Stop_All(void);
BDMA_Result MDrv_BDMA_Stop(U8 u8Ch);
BDMA_Result MDrv_BDMA_Exit(void);
static U8 _BDMA_GetDevCfg(BDMA_Dev eDev);
static BDMA_Result BDMA_Start(U8 u8SrcDev, U8 u8DstDev, U8 u8Ch);
static BDMA_Result BDMA_Check_Device(BDMA_SrcDev eSrc, BDMA_DstDev eDst);

BDMA_Result MDrv_BDMA_CH0_MemCopy_MIU0toMIU0(long phy64SrcAddr, long phy64DstAddr, U32 u32Len);
BDMA_Result MDrv_BDMA_CH0_MemCopy_MIU0toMIU1(long phy64SrcAddr, long phy64DstAddr, U32 u32Len);
BDMA_Result MDrv_BDMA_CH0_MemCopy_MIU1toMIU0(long phy64SrcAddr, long phy64DstAddr, U32 u32Len);
BDMA_Result MDrv_BDMA_CH0_MemCopy_MIU1toMIU1(long phy64SrcAddr, long phy64DstAddr, U32 u32Len);

BDMA_Result MDrv_BDMA_CH1_MemCopy_MIU0toMIU0(long phy64SrcAddr, long phy64DstAddr, U32 u32Len);
BDMA_Result MDrv_BDMA_CH1_MemCopy_MIU0toMIU1(long phy64SrcAddr, long phy64DstAddr, U32 u32Len);
BDMA_Result MDrv_BDMA_CH1_MemCopy_MIU1toMIU0(long phy64SrcAddr, long phy64DstAddr, U32 u32Len);
BDMA_Result MDrv_BDMA_CH1_MemCopy_MIU1toMIU1(long phy64SrcAddr, long phy64DstAddr, U32 u32Len);

BDMA_Result MDrv_BDMA_CH0_PatternFill(long u32Addr, U32 u32Len, U32 u32Pattern, BDMA_DstDev eDev);
BDMA_Result MDrv_BDMA_CH1_PatternFill(long u32Addr, U32 u32Len, U32 u32Pattern, BDMA_DstDev eDev);
#ifdef __cplusplus
}
#endif
#endif
