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
/// file    mdrv_tuner.c
/// @brief  TUNER Driver Interface
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
#include <asm/io.h>

#include "mst_devid.h"
#include "mdrv_types.h"

#include "mdrv_tuner_datatype.h"
#include "mdrv_iic_io.h"
#include "mdrv_tuner_AV2017.h"
#if 0 //kdrv
extern int MDrv_SW_IIC_WriteBytes(U8 u8BusNum, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U16 u16BufLen, U8* pu8Buf);
extern int MDrv_SW_IIC_ReadBytes(U8 u8BusNum, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U16 u16BufLen, U8* pu8Buf);
#endif
const char match_name[16]="Mstar-AV2017";
//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define malloc(x) kmalloc(x, GFP_KERNEL)
#define free kfree
//#define printf TUNER_PRINT
#define TUNER_PRINT(fmt, args...)        printk("[%s][%05d]" fmt, match_name, __LINE__, ## args)
#ifdef TUNER_DEBUG
    #define DBG_TUNER(x) x
#else
    #define DBG_TUNER(x) //x
#endif
#ifndef UNUSED //to avoid compile warnings...
    #define UNUSED(var) (void)((var) = (var))
#endif

/// Define DVB-S Roll-Off factor
typedef enum
{
    SAT_RO_35, ///< roll-off factor = 0.35
    SAT_RO_25, ///< roll-off factor = 0.25
    SAT_RO_20, ///< roll-off factor = 0.20
    E_SAT_RO_35 = SAT_RO_35,
    E_SAT_RO_25 = SAT_RO_25,
    E_SAT_RO_20 = SAT_RO_20,
} EN_SAT_ROLL_OFF_TYPE;

//#define mcSHOW_HW_MSG(fmt, args...)

#define MAX_TUNER_DEV_NUM                     1

#define AV2017_I2C_ADDR                       ((U8)  0xC6)
#define  TUNER_CRYSTAL_FREQ         27
static U8 tuner_i2c_channel_bus = 0;  //i2c bus select
static U8 _u8SlaveID = AV2017_I2C_ADDR;
//static U8 sCurRolloff = E_SAT_RO_35;

static TunerPara AVtuner;
static s_Tuner_dev_info *p_dev = NULL;
static s_Tuner_dev_info dev[MAX_TUNER_DEV_NUM] =
{
  {E_TUNER_DTV_DVB_S_MODE, E_CLOSE, 0, 0,0,0,0,0,0,0,0},
};

typedef struct
{
    float    LvdBm;
    U16 u16_if_agc;
}S_RSSI_TABLE;

typedef enum
{
    DEMOD_EXTEND_msb131x = 0x00,
    DEMOD_EXTEND_msb1240,
    DEMOD_EXTEND2_msb1245,
} DEMOD_TYPE;
/*
static S_RSSI_TABLE rssi_av2018_131x[]=
{//19*10-4 = 186
{-92.5, 0xF253},{-92.0, 0xF1EA},{-91.5, 0xF145},{-91.0, 0xF0C8},{-90.5, 0xF05A},{-90.0, 0xEFE2},{-89.5, 0xEF2E},{-89.0, 0xEEAC},{-88.5, 0xEE34},{-88.0, 0xEDB2},
{-87.5, 0xECFE},{-87.0, 0xEC63},{-86.5, 0xEBE6},{-86.0, 0xEB28},{-85.5, 0xEA92},{-85.0, 0xE9F7},{-84.5, 0xE970},{-84.0, 0xE894},{-83.5, 0xE7E5},{-83.0, 0xE6EB},
{-82.5, 0xE62D},{-82.0, 0xE5A1},{-81.5, 0xE510},{-81.0, 0xE443},{-80.5, 0xE3A8},{-80.0, 0xE303},{-79.5, 0xE236},{-79.0, 0xE1A0},{-78.5, 0xE0EC},{-78.0, 0xE024},
{-77.5, 0xDFB6},{-77.0, 0xDF57},{-76.5, 0xDEF3},{-76.0, 0xDE67},{-75.5, 0xDDF9},{-75.0, 0xDD86},{-74.5, 0xDD2C},{-74.0, 0xDCD2},{-73.5, 0xDC73},{-73.0, 0xDC14},
{-72.5, 0xDBB5},{-72.0, 0xDB38},{-71.5, 0xDA5C},{-71.0, 0xD99E},{-70.5, 0xD8D6},{-70.0, 0xD7E6},{-69.5, 0xD728},{-69.0, 0xD660},{-68.5, 0xD584},{-68.0, 0xD4B2},
{-67.5, 0xD3F4},{-67.0, 0xD32C},{-66.5, 0xD26E},{-66.0, 0xD1C4},{-65.5, 0xD12E},{-65.0, 0xD0A7},{-64.5, 0xD03E},{-64.0, 0xCFD0},{-63.5, 0xCF71},{-63.0, 0xCF12},
{-62.5, 0xCEBD},{-62.0, 0xCE68},{-61.5, 0xCE18},{-61.0, 0xCDC8},{-60.5, 0xCD82},{-60.0, 0xCD37},{-59.5, 0xCCE2},{-59.0, 0xCC9C},{-58.5, 0xCC4C},{-58.0, 0xCBFC},
{-57.5, 0xCBAC},{-57.0, 0xCB5C},{-56.5, 0xCAEE},{-56.0, 0xCA44},{-55.5, 0xC97C},{-55.0, 0xC850},{-54.5, 0xC73D},{-54.0, 0xC670},{-53.5, 0xC5D0},{-53.0, 0xC53F},
{-52.5, 0xC4C2},{-52.0, 0xC445},{-51.5, 0xC3CD},{-51.0, 0xC350},{-50.5, 0xC2DD},{-50.0, 0xC26A},{-49.5, 0xC1DE},{-49.0, 0xC152},{-48.5, 0xC0BC},{-48.0, 0xC021},
{-47.5, 0xBF9A},{-47.0, 0xBF09},{-46.5, 0xBE73},{-46.0, 0xBE05},{-45.5, 0xBD74},{-45.0, 0xBCED},{-44.5, 0xBC70},{-44.0, 0xBBE4},{-43.5, 0xBB7B},{-43.0, 0xBAF9},
{-42.5, 0xBA6D},{-42.0, 0xB9BE},{-41.5, 0xB90A},{-41.0, 0xB838},{-40.5, 0xB77F},{-40.0, 0xB6DF},{-39.5, 0xB635},{-39.0, 0xB568},{-38.5, 0xB4C8},{-38.0, 0xB43C},
{-37.5, 0xB3C4},{-37.0, 0xB333},{-36.5, 0xB2B6},{-36.0, 0xB239},{-35.5, 0xB1A8},{-35.0, 0xB12B},{-34.5, 0xB09F},{-34.0, 0xB022},{-33.5, 0xAFA5},{-33.0, 0xAF28},
{-32.5, 0xAEB0},{-32.0, 0xAE42},{-31.5, 0xADD9},{-31.0, 0xAD75},{-30.5, 0xAD02},{-30.0, 0xAC80},{-29.5, 0xABFE},{-29.0, 0xAB77},{-28.5, 0xAB04},{-28.0, 0xAA87},
{-27.5, 0xAA1E},{-27.0, 0xA9BF},{-26.5, 0xA965},{-26.0, 0xA91A},{-25.5, 0xA8A2},{-25.0, 0xA820},{-24.5, 0xA7B2},{-24.0, 0xA753},{-23.5, 0xA6DB},{-23.0, 0xA64F},
{-22.5, 0xA5D7},{-22.0, 0xA564},{-21.5, 0xA4E7},{-21.0, 0xA82F},{-20.5, 0xA3B6},{-20.0, 0xA2F8},{-19.5, 0xA24E},{-19.0, 0xA1C7},{-18.5, 0xA145},{-18.0, 0xA096},
{-17.5, 0xA00A},{-17.0, 0x9F83},{-16.5, 0x9ED4},{-16.0, 0x9E4D},{-15.5, 0x9DD0},{-15.0, 0x9CEA},{-14.5, 0x9C72},{-14.0, 0x9BFA},{-13.5, 0x9B6E},{-13.0, 0x9ACE},
{-12.5, 0x9A60},{-12.0, 0x9A0B},{-11.5, 0x99A2},{-11.0, 0x992F},{-10.5, 0x98D0},{-10.0, 0x9871},{-9.5, 0x982B},{-9.0, 0x97D6},{-8.5, 0x9772},{-8.0, 0x9722},
{-7.5, 0x96E1},{-7.0, 0x96A5},{-6.5, 0x965A},{-6.0, 0x960A},{-5.5, 0x95B5},{-5.0, 0x9560},{-4.5, 0x951F},{-4.0, 0x94E3},{-3.5, 0x94A2},{-3.0, 0x9452},
{-2.5, 0x93F8},{-2.0, 0x939E},{-1.5, 0x934E},{-1.0, 0x9312},{-0.5, 0x92D1},{-0.0, 0x9286}
};

static S_RSSI_TABLE rssi_av2018_1240[]=
{
{-95.0,0xFF00},
{-90.0,0xFF00},
{-85.0,0x8B5C},
{-80.0,0x81DC},
{-75.0,0x7D44},
{-70.0,0x7B88},
{-65.0,0x770F},
{-60.0,0x74FB},
{-55.0,0x7256},
{-50.0,0x6C8F},
{-45.0,0x6986},
{-40.0,0x6753},
{-35.0,0x64E6},
{-30.0,0x6295},
{-25.0,0x606E},
{-20.0,0x5E5B},
{-15.0,0x5BF8},
{-10.0,0x5A1F},
{- 5.0,0x5971},
{- 0.0,0x591D},
};
*/
U8 TunerInitialSetting[2][42]=
{
    {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29},
    {0x38, 0x00, 0x00, 0x50, 0x1F, 0xA3, 0xFD, 0x58, 0x0E, 0x82, 0x88, 0xB4, 0xD6, 0x40, 0x94, 0x4A, 0x66, 0x40, 0x80, 0x2B, 0x6A, 0x50, 0x91, 0x27, 0x8F, 0xCC, 0x21, 0x10, 0x80, 0x0E, 0xF5, 0x7F, 0x4D, 0xBF, 0xE0, 0xE0, 0x2E, 0x02, 0xAB, 0x97, 0xC5, 0xA8}
};
/***Tuner Control Functions********************************************************************/
static BOOL Tuner_Parameter_Initial (pTunerPara pAVtuner);
static BOOL Tuner_Register_Initial (pTunerPara pAVtuner);
static BOOL Tuner_Set_Channel_Frequency (pTunerPara pAVtuner, U32 channel_freq_mhz);
static BOOL Tuner_Set_Filter_Bandwith (pTunerPara pAVtuner,U32 filter_BW_khz);
static BOOL Tuner_Set_RFLP_PD (pTunerPara pAVtuner);
static BOOL Tuner_Set_Fine_Tune (pTunerPara pAVtuner);
//static BOOL Tuner_Set_RXout_PGA (pTunerPara pAVtuner);
static BOOL Tuner_Get_Channel_Lock (pTunerPara pAVtuner);
static BOOL Tuner_Get_Filter_Lock (pTunerPara pAVtuner);


//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------

static void platform_ms_delay(U32 DelayTimeInMs)
{

    // OEM should implement sleep operation
   #if 0 //kdrv
   /* Variable declarations */
  usleep(1000*DelayTimeInMs);
   #else
   mdelay(DelayTimeInMs);
   #endif
    return;
}

BOOL AV2017_WriteArrayReg(U8 u8SlaveID, U8 u8Addr, U8 * buff, U8 length)
{
    BOOL bRet;
    U8 queue[16];
    U8 i;

    queue[0]=u8Addr;//reg_start

    for (i=0;i<length;i++)
    {
        queue[1+i]= buff[i];
    }


    bRet=MDrv_SW_IIC_WriteBytes(tuner_i2c_channel_bus,  u8SlaveID, 0, 0, length+1, queue);
//int MDrv_SW_IIC_WriteBytes(u8 u8BusNum, u8 u8SlaveID, u8 AddrCnt, u8* pu8addr, u16 u16size, u8* pBuf);

    if(bRet==TRUE)
    {
        TUNER_PRINT("Tuner I2C write FAIL!!!!!!\n");
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

BOOL AV2017_WriteReg(U8 u8SlaveID, U8 u8Addr, U8 u8Data)
{
    BOOL bRet;
    U8 u8Value[2];
    u8Value[0]=u8Addr;
    u8Value[1]=u8Data;

    bRet=MDrv_SW_IIC_WriteBytes(tuner_i2c_channel_bus, u8SlaveID, 0, 0, 2, u8Value);

    if(bRet==TRUE)
    {
        TUNER_PRINT("Tuner I2C write FAIL!!!!!!\n");
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}


BOOL AV2017_ReadReg(U8 u8SlaveID, U8 u8Addr, U8 *u8Data)
{
    BOOL bRet;

    bRet=MDrv_SW_IIC_WriteBytes(tuner_i2c_channel_bus,  u8SlaveID, 0, 0, 1, &u8Addr);
    bRet|=MDrv_SW_IIC_ReadBytes(tuner_i2c_channel_bus, u8SlaveID, 0, 0, 1, u8Data);
    if(bRet==TRUE)
    {
        TUNER_PRINT("Tuner I2C read FAIL!!!!\n");
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}
void AV2017_SlaveID_Check(void)
{
     U8 regValue;
     U8 u8OrignalSlaveID = _u8SlaveID;

      _u8SlaveID = 0xC0;
      do
      {
          regValue=(char) (0x38);
          if(AV2017_WriteReg(_u8SlaveID,0,regValue))
          {
               regValue = 0;
               if(AV2017_ReadReg(_u8SlaveID,0,&regValue))
               {
                     if(regValue == 0x38)
                     {
                         break;
                     }
               }
          }
          _u8SlaveID += 0x02;
      }while(_u8SlaveID <= 0xC6);
      AVtuner.I2C_ADDR = (TUNER_I2C_ADDRESS)_u8SlaveID;
      if(_u8SlaveID > 0xC6)
      {
           _u8SlaveID = AV2017_I2C_ADDR;
           AVtuner.I2C_ADDR = (TUNER_I2C_ADDRESS)AV2017_I2C_ADDR;
      }

      if(u8OrignalSlaveID != _u8SlaveID)
      {
          TUNER_PRINT("[Warning] %s, %d, AV2017 Slave ID mismatch...AV2017 Slave ID defined in Board.h=0x%x, AV2017 Slave ID found by AV2017_SlaveID_Check=0x%x\n", __FILE__, __LINE__, u8OrignalSlaveID, _u8SlaveID);
      }
}
/*********************************************************************************
* Channel lock status
* Paramter1:    pTunerPara : pointer of Tuner parameter structure
* Return value: TUNER_ErrorCode : error code of definition
* Description:
* 1. reg[11]_D0 is CHLF. Read-only bit. Lock=1. Unlock=0.
**********************************************************************************/
BOOL Tuner_Get_Channel_Lock (pTunerPara pAVtuner)
{
    BOOL result = TRUE;
    U8 reg11=0;
    U8 lock=0;

    AV2017_ReadReg (_u8SlaveID, 11, &reg11);
    lock = reg11 &0x01;
    DBG_TUNER(TUNER_PRINT("Tuner_Get_Channel_Lock  lock_reg:[0x%x]\n", lock));
    if ( lock != 1)
    {
        result = FALSE;
        TUNER_PRINT("Tuner Get CHANNEL UNLOCK!!!");
    }
    return result;
}

/*********************************************************************************
* Filter bandwidth lock status
* Paramter1:    pTunerPara : pointer of Tuner parameter structure
* Return value: TUNER_ErrorCode : error code of definition
* Description:
* 1. reg[11]_D1 is BWLF. Read-only bit. Lock=1. Unlock=0.
**********************************************************************************/
static BOOL Tuner_Get_Filter_Lock (pTunerPara pAVtuner)
{
    BOOL result = TRUE;
    U8 reg11=0;
    U8 lock = 0;

    AV2017_ReadReg (_u8SlaveID, 11, &reg11);
    lock = (reg11>>1) &0x01;
    DBG_TUNER(TUNER_PRINT("Tuner_Get_Filter_Lock  lock_reg:[0x%x]\n", lock));
    if ( lock != 1)
    {
        TUNER_PRINT("Tuner Get Filter UNLOCK!!!");
        result = FALSE;
    }
    return result;
}
/*******************************************************************
* Tuner parameter initialization:
* Paramter1:    pTunerPara : pointer of Tuner parameter structure
* Return value: TUNER_ErrorCode : error code of definition
* Description: Initialize all the parameters in the data structure of pAVtuner
********************************************************************/
static BOOL Tuner_Parameter_Initial (pTunerPara pAVtuner)
{
    BOOL result = TRUE;

    pAVtuner->crystal_khz   = TUNER_CRYSTAL_FREQ*1000;            // (Unit: KHz)Tuner_crystal

    pAVtuner->I2C_ADDR      = ADDR1_H_ADDR0_H;  // Tuner I2C address at write mode
    pAVtuner->IQmode        = Differential;     // IQ_MODE
    pAVtuner->PGA_Gain      = PGA_1_5dB ;       // PGA_GAIN
    pAVtuner->PGA_Current   = PGA_1_5mA ;       // PGA_DRIVE_CURRENT
    pAVtuner->XO_Current    = HIGH_AV2017;      // XO_DRIVE_CURRENT
    pAVtuner->XO_EN         = XO_ON;            // XO_ENABLE Setting
    pAVtuner->RFLP_EN       = RFLP_OFF;         // RFLP_ENABLE Setting
    pAVtuner->PD_EN         = Wake_Up;          // SOFTWARE_POWERDOWN
    pAVtuner->FT            = FT_ON;            // TUNER_Fine_Tune
    pAVtuner->blind_scan    = 0;                // blind_scan

    return result;
}

/*******************************************************************************
* Tuner Power-ON registers initialization
* Paramter1:    pTunerPara : pointer of Tuner parameter structure
* Return value: TUNER_ErrorCode : error code of definition
* Description: Initialize all registers of the Tuner
********************************************************************************/
static BOOL Tuner_Register_Initial (pTunerPara pAVtuner)
{
    BOOL result = TRUE;
    U8 reg[50];

    reg[0]= 0x50; //2150M
    reg[1]= 0xA1; //2150M
    reg[2]= 0x2F; //2150M
    reg[3]= 0x50; //2150M
    reg[4]= 0x1f;
    reg[5]= 0xa3;
    reg[6]= 0xfd;
    reg[7]= 0x58;
    reg[8]= (0x04)|(pAVtuner->PGA_Gain <<3)|(pAVtuner->PGA_Current);

    if( pAVtuner->crystal_khz < 25000 )
        reg[9]= (0x00)|(pAVtuner->XO_Current<<6); // ctrl_clk Divider=/1
    else
        reg[9]= (0x02)|(pAVtuner->XO_Current<<6); // ctrl_clk Divider=/2

    reg[10]= 0x88;
    reg[11]= 0xb4;
    reg[12]= (0x16)|(pAVtuner->XO_EN <<7)|(pAVtuner->RFLP_EN <<6)|(pAVtuner->PD_EN <<5);
    reg[13]= 0x40;
    reg[14]= 0x94;
    reg[15]= 0x4a;
    reg[16]= 0x66;
    reg[17]= 0x40;
    reg[18]= 0x80;
    reg[19]= 0x2b;
    reg[20]= 0x6a;
    reg[21]= 0x50;
    reg[22]= 0x91;
    reg[23]= 0x27;
    reg[24]= 0x8f;
    reg[25]= 0xcc;
    reg[26]= 0x21;
    reg[27]= 0x10;
    reg[28]= 0x80;
    reg[29]= 0xE2;
    reg[30]= 0xf5;
    reg[31]= 0x7f;
    reg[32]= 0x4a;
    reg[33]= 0x9b;
    reg[34]= 0xe0;
    reg[35]= 0xe0;
    reg[36]= 0x36;
    reg[37]= (U8)(pAVtuner->FT);
    reg[38]= 0xab;
    reg[39]= 0x97;
    reg[40]= 0xc5;
    reg[41]= 0xa8;

    /* Sequence 3*/
    /* Send reg12*/
    result &= AV2017_WriteArrayReg(_u8SlaveID, 0, &reg[0], 12);
    result &= AV2017_WriteArrayReg(_u8SlaveID, 13, &reg[13], 12);
    result &= AV2017_WriteArrayReg(_u8SlaveID, 25, &reg[25], 11);
    result &= AV2017_WriteArrayReg(_u8SlaveID, 36, &reg[36], 6);
    result &= AV2017_WriteReg(_u8SlaveID,0,reg[0]);//need to be set finally
    result &= AV2017_WriteReg(_u8SlaveID,12,reg[12]);

    /* Time delay ms*/
    platform_ms_delay(100);

    //re-initial

    /* Sequence 3*/
    /* Send reg12*/
    result &= AV2017_WriteArrayReg(_u8SlaveID, 0, &reg[0], 12);
    result &= AV2017_WriteArrayReg(_u8SlaveID, 13, &reg[13], 12);
    result &= AV2017_WriteArrayReg(_u8SlaveID, 25, &reg[25], 11);
    result &= AV2017_WriteArrayReg(_u8SlaveID, 36, &reg[36], 6);
    result &= AV2017_WriteReg(_u8SlaveID,0,reg[0]);//need to be set finally
    result &= AV2017_WriteReg(_u8SlaveID,12,reg[12]);


    /* Time delay ms*/
    platform_ms_delay(5);

    return result;
}
/*********************************************************************************
* Loopthrough and power-down setting
* Paramter1:    pTunerPara : pointer of Tuner parameter structure
* Return value: TUNER_ErrorCode : error code of definition
* Description:
* 1. reg[12]_D7 is xocore_ena. Enable/disable XO section
* 2. reg[12]_D6 is RFLP_ena.   Enable/disable Loop-through section
* 3. reg[12]_D5 is PD_soft.	   Power ON/OFF Receiver,Synthesizer,VCO section
* 4. For tuner model name =1, hardware Power-down is controled by Hardware Pin13, which turn off Receiver,Synthesizer,VCO,XO,Control section.
*	 For tuner model name =1, make sure Pin13 is at low when sending registers.
**********************************************************************************/
static BOOL Tuner_Set_RFLP_PD (pTunerPara pAVtuner)
{
    BOOL result = TRUE;
    U8 reg12;

    reg12 = (0x16)|( pAVtuner->XO_EN <<7)|(pAVtuner->RFLP_EN <<6)|(pAVtuner->PD_EN <<5);

    result = AV2017_WriteReg(_u8SlaveID,12, reg12);
    /* Time delay ms*/
    platform_ms_delay(5);
    return result;
}
/*********************************************************************************
* Fine-tune function setting
* Paramter1:    pTunerPara : pointer of Tuner parameter structure
* Return value: BOOL : error code of definition
* Description:
* 1. Fine-tune the gain for sensitivity and linearity.
*    reg[37]_D2 is FT_block,Tp=12ms. reg[37]_D1 is FT_EN. reg[37]_D0 is FT_hold.
* 2. FT_OFF      = 0x00  : FT_block=0, FT_EN=0, FT_hold=0. The Gain is at best sensitivity gain.
* 3. FT_ON       = 0x02  : FT_block=0, FT_EN=1, FT_hold=0. Turn on to fine-tuned gain continuously between sensitivity and linearity.
* 4. FT_Hold     = 0x03  : FT_block=0, FT_EN=1, FT_hold=1. Stop fine-tuning the gain and hold the current fine-tuned gain
* 5. FT_Delay_ON = 0x06	 : FT_block=1, FT_EN=1, FT_hold=0. FT_block=1 trigger the FT hardware delay.
*    After the  delay time 12ms, Fine-tune Function is turned on. And,the chip hardware resets FT_block=1 to FT_block=0 internally.
*    Then, FT function is continuously fine-tuning the gain by the voltage level of pin RFAGC
* 6. Fint-tune function take a reference to RFAGC voltage level of Hardware Pin5.
*	 When Fine-tune is enable, make sure the RFAGC do not have a sharp jump that cause a longer AGC settling time.
***********************************************************************************/
static BOOL Tuner_Set_Fine_Tune (pTunerPara pAVtuner)
{
    BOOL result = TRUE;
    U32 pre_delay;
    U32 post_delay;
    U8 reg37;

    switch ( pAVtuner->FT )
    {
        case  FT_ON:{
            pre_delay  = 1;
            post_delay = 5;
            break;}
        case  FT_Hold:{
            pre_delay  = 10;
            post_delay = 1;
            break;}
        case  FT_Delay_ON:{
            pre_delay  = 1;
            post_delay = 15;
            break;}
        default:{ //OFF
            pre_delay  = 1;
            post_delay = 1;}
    }

    reg37 = pAVtuner->FT;
    /* Time delay ms*/
    platform_ms_delay(pre_delay);
    /* Send Fine-tune Function Control*/
    result=AV2017_WriteReg(_u8SlaveID,37, reg37);
    /* Time delay ms*/
    platform_ms_delay(post_delay);
    return result;
}
/*********************************************************************************
* Channel frequency setting
* Paramter1:    pTunerPara : pointer of Tuner parameter structure
* Paramter2:    MS_U32 : channel frequency (unit: MHz)
* Return value: BOOL : error code of definition
* Description:
* 1. Send reg3,2,1,0 to set tuner at the channel frequency.
*    reg0 is int<7:0>; reg1 is frac<16:9>; reg2 is frac<8:1>; reg3_D7 is frac<0>
*	 For Tuner model name =1, reg3_D2 is IQ mode selection.
* 2. If XO is not default 27MHz, need to manually select VCO Divider.
*    The VCO Divider switching point is freqency=1167.75MHz,
*    and send Reg16 before Reg0~3
**********************************************************************************/
static BOOL Tuner_Set_Channel_Frequency (pTunerPara pAVtuner, U32 channel_freq_khz)
{

    BOOL bRet = TRUE;
    U8 reg0;
    U8 reg1;
    U8 reg2;
    U8 reg3;
    U8 reg16;
    U8 reg30;
    U32 fracN;
    U32 XO_khz;

    XO_khz = pAVtuner->crystal_khz;

    if((channel_freq_khz%XO_khz) > (XO_khz-501))
    {
        channel_freq_khz /= 1000;
        channel_freq_khz *= 1000;
        channel_freq_khz += 1000;
    }
    else if((channel_freq_khz%XO_khz) < 500)
    {
        channel_freq_khz /= 1000;
        channel_freq_khz *= 1000;
    }
    else
    {}

    if( XO_khz != 27000 )
    {
        if(channel_freq_khz > 1167750)
            reg16 = 0x56; // VCO Divider=/2
        else
            reg16 = 0x46; // VCO Divider=/4

        /* Send Reg16*/
        bRet = AV2017_WriteReg(_u8SlaveID,16,reg16);
        if(bRet!=TRUE){ return bRet; }
    }

    fracN = (channel_freq_khz + XO_khz/2)/XO_khz;
    if(fracN > 0xff)
        fracN = 0xff;
    reg0=(U8) (fracN & 0xff);
    fracN = ((channel_freq_khz/1000)<<17)/(XO_khz/1000);
    fracN = fracN & 0x1ffff;
    reg1=(U8) ((fracN>>9)&0xff);
    reg2=(U8) ((fracN>>1)&0xff);

    reg3= (0x50)|(fracN<<7)|(pAVtuner->IQmode<<2);
    /* Sequence 4*/

    /* Send Reg3*/
    bRet = AV2017_WriteReg(_u8SlaveID,3,reg3);
    if(bRet!=TRUE){ return bRet; }
    /* Send Reg2*/
    bRet = AV2017_WriteReg(_u8SlaveID,2,reg2);
    if(bRet!=TRUE){ return bRet; }
    /* Send Reg1*/
    bRet = AV2017_WriteReg(_u8SlaveID,1,reg1);
    if(bRet!=TRUE){ return bRet; }
    /* Send Reg0*/
    bRet = AV2017_WriteReg(_u8SlaveID,0,reg0);
    if(bRet!=TRUE){ return bRet; }
    /* Time delay ms*/

    platform_ms_delay(100);

    /* Send Reg3*/
    bRet = AV2017_WriteReg(_u8SlaveID,3,reg3);
    if(bRet!=TRUE){ return bRet; }
    /* Send Reg2*/
    bRet = AV2017_WriteReg(_u8SlaveID,2,reg2);
    if(bRet!=TRUE){ return bRet; }
    /* Send Reg1*/
    bRet = AV2017_WriteReg(_u8SlaveID,1,reg1);
    if(bRet!=TRUE){ return bRet; }
    /* Send Reg0*/
    bRet = AV2017_WriteReg(_u8SlaveID,0,reg0);
    if(bRet!=TRUE){ return bRet; }
    /* Time delay ms*/

    platform_ms_delay(4);

    /*
    if( (channel_freq_khz%XO_khz)==0)
    {
        reg30 = 0xe5;   //Bug Fix: If set to 0xE5, tuner only tune with 1080MHz
    }
    else
    {
        reg30 = 0xf5;
    }

    bRet = AV2017_WriteReg(_u8SlaveID,30,reg30);
    */

    //if(bRet!=TRUE){ return bRet; }

    platform_ms_delay(4);

    return bRet;
}

/*********************************************************************************
* ZIF low pass filter bandwidth setting
* Paramter1:    pTunerPara : pointer of Tuner parameter structure
* Paramter2:    MS_U32 : LPF bandwidth (unit: kHz)
* Return value: BOOL : error code of definition
* Description:
* 1. Calculate the filter reference clk from XO. Transfer Filter BW setting to register5
* 2. Add Fine-tune function after Bandwidth setting.
**********************************************************************************/
static BOOL Tuner_Set_Filter_Bandwith (pTunerPara pAVtuner, U32 filter_BW_khz)
{
    BOOL bRet=TRUE;
    U32 ctrl_clk_khz;
    U32 BW_CLK_khz;
    U32 XO_khz;
    U32 filter_khz;
    U32 BF;
    U32 BW;
    U8 reg5;

    XO_khz = pAVtuner->crystal_khz;

    // Ref from AV2012 driver for avoid SR too large than configure BW.
    // rolloff is 35%
    BW = filter_BW_khz*135/200;
    // monsen 20080726, BB request low IF when sym < 6.5MHz
    // add 6M when Rs<6.5M,
    if(filter_BW_khz<6500)
    {
        BW = BW + 6000;
    }
    // add 2M for LNB frequency shifting
    BW = BW + 2000;
    // add 8% margin since fc is not very accurate
    BW = BW*108/100;
    // Bandwidth can be tuned from 4M to 40M
    if( BW< 4000)
    {
        BW = 4000;
    }
    if( BW> 40000)
    {
        BW = 40000;
    }
    filter_khz = BW;
    // Ref from AV2012 driver for avoid SR too large than configure BW.

    if( XO_khz < 25000 )
        ctrl_clk_khz = XO_khz;
    else
        ctrl_clk_khz = XO_khz/2;
    /* Filter Bandwidth reference CLK calculation */
    BW_CLK_khz = (ctrl_clk_khz +64/2)/64;
    /* BF = filter_BW (KHz) * 1.27 / BW_CLK (KHz)*/
    BF = (filter_khz*127 + 100*BW_CLK_khz/2)/(100*BW_CLK_khz);
    if(BF > 0xff)
        BF = 0xff;
    reg5 = (U8)BF;
    /* Sequence 5*/
    /* Send Reg5*/
    bRet = AV2017_WriteReg(_u8SlaveID,5, reg5);
    if(bRet!=TRUE){ return bRet; }
    /* Time delay ms*/
    platform_ms_delay(4);

    /* Reset FT after Filter BW setting */
    if (pAVtuner->blind_scan == 1)
    {
        /* choose one for different blind scan mechanism */
        // pAVtuner->FT = FT_OFF;
        // pAVtuner->FT = FT_Hold;
        pAVtuner->FT = FT_ON;
    }
    else
    {
        pAVtuner->FT = FT_Delay_ON;
    }
    bRet = Tuner_Set_Fine_Tune (pAVtuner);
    if(bRet!=TRUE){ return bRet; }

    return bRet;
}

int MDrv_TUNER_ConfigAGCMode(int minor, U32 eMode);

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
int MDrv_TUNER_Connect(int minor)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    p_dev=&(dev[minor]);
    return TRUE;
}

int MDrv_TUNER_Disconnect(int minor)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    p_dev=&(dev[minor]);
    return TRUE;
}

int MDrv_TUNER_ATV_SetTune(int minor, U32 u32FreqKHz, U32 eBand, U32 eMode, U8 otherMode)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    p_dev=&(dev[minor]);

    return FALSE;
}

int MDrv_TUNER_DVBS_SetTune(int minor, U16 u16CenterFreqMHz, U32 u32SymbolRateKs)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    BOOL bRet=TRUE;
    p_dev=&(dev[minor]);

    if (p_dev->e_status != E_WORKING)
    {
        TUNER_PRINT("[Error]%s,%d\n",__FILE__,__LINE__);
        return FALSE;
    }
    p_dev->e_std = E_TUNER_DTV_DVB_S_MODE;
    p_dev->u32_freq = u16CenterFreqMHz;
    p_dev->u32_symbolrate = u32SymbolRateKs;
    TUNER_PRINT("u32_freq:[%d] u32_symbolrate:[%d]\n",p_dev->u32_freq,p_dev->u32_symbolrate);

    bRet = Tuner_Set_Channel_Frequency (&AVtuner, (U32)u16CenterFreqMHz*1000);
    if (bRet != TRUE)
    {
        TUNER_PRINT("Tuner_Set_Channel_Frequency fail, error_code\n");
        bRet = false;
    }
    bRet = Tuner_Set_Filter_Bandwith (&AVtuner, u32SymbolRateKs);
    if (bRet != TRUE)
    {
        TUNER_PRINT("Tuner_Set_Filter_Bandwith fail, error_code\n");
    }
    AVtuner.RFLP_EN = RFLP_ON;
    bRet = Tuner_Set_RFLP_PD (&AVtuner);
    if (bRet != TRUE)
    {
        TUNER_PRINT("Tuner_Set_RFLP_PD fail, error_code\n");
    }

    if (bRet == false)
    {
        TUNER_PRINT("AIROHA_AV2017_ConfigTuner fail\n");
    }

    return bRet;
}

int MDrv_TUNER_DTV_SetTune(int minor, U32 freq, U32 eBandWidth, U32 eMode)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    p_dev=&(dev[minor]);

    return FALSE;
}

int MDrv_TUNER_ExtendCommand(int minor, U8 u8SubCmd, U32 u32Param1, U32 u32Param2, void* pvoidParam3)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    p_dev=&(dev[minor]);

    return FALSE;
}

int MDrv_TUNER_TunerInit(int minor,s_Tuner_dev_info *devTunerInit)
{
    BOOL bRet=TRUE;
    //U8 index;

    _u8SlaveID=devTunerInit->u8_slaveID;
    tuner_i2c_channel_bus =  devTunerInit->u8_i2c_bus;
    TUNER_PRINT("%s is invoked  u8Slave:[0x%x] bus_channel_num: [%d]\n", __FUNCTION__,_u8SlaveID,tuner_i2c_channel_bus);
    if (minor < MAX_TUNER_DEV_NUM)
    {
        p_dev=&(dev[minor]);
        p_dev->u8_slaveID = _u8SlaveID;
        p_dev->u8_i2c_bus = tuner_i2c_channel_bus;
        if ( (p_dev->e_status == E_CLOSE)
        || (p_dev->e_status == E_SUSPEND) )
        {
            bRet = Tuner_Parameter_Initial (&AVtuner);
            if (bRet != TRUE)
            {
                TUNER_PRINT("Tuner_Parameter_Initial fail, error_code\n");
            }

            AV2017_SlaveID_Check();


            bRet = Tuner_Register_Initial (&AVtuner);
            if (bRet != TRUE)
            {
                TUNER_PRINT("Tuner_Register_Initial fail\n");
            }
            else
            {
                p_dev->e_status = E_WORKING;
            }
        }
        else if(p_dev->e_status == E_WORKING)
        {
            AV2017_SlaveID_Check();
        }
    }
    else
    {
        bRet = FALSE;
    }
    return bRet;
}



int MDrv_TUNER_ConfigAGCMode(int minor, U32 eMode)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    p_dev=&(dev[minor]);

    return FALSE;
}

int MDrv_TUNER_SetTunerInScanMode(int minor, U32 bScan)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    p_dev=&(dev[minor]);

    return TRUE;
}

int MDrv_TUNER_SetTunerInFinetuneMode(int minor, U32 bFinetune)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    p_dev=&(dev[minor]);

    return FALSE;
}

int MDrv_TUNER_GetCableStatus(int minor, U32 eStatus)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    return FALSE;
}

int MDrv_TUNER_TunerReset(int minor)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    return 0;
}

int MDrv_TUNER_IsLocked(int minor)
{
    BOOL bRet = TRUE;
    bRet = Tuner_Get_Channel_Lock (&AVtuner);
    if (bRet != TRUE)
    {
        TUNER_PRINT("Tuner_Get_Channel_Lock fail\n");
    }
    bRet = Tuner_Get_Filter_Lock (&AVtuner);
    if (bRet != TRUE)
    {
        TUNER_PRINT("Tuner_Get_Filter_Lock fail\n");
    }
    return bRet;

}

int MDrv_TUNER_GetRSSI(int minor, U16 u16Gain, U8 u8DType)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    return 0;
}

int MDrv_TUNER_Suspend(void)
{
    U8 i = 0;
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    for (i=0; i<MAX_TUNER_DEV_NUM; i++)
    {
        if (dev[i].e_status == E_WORKING)
        {
            dev[i].e_status = E_SUSPEND;
        }
    }
#if 0//kdrv_temp
    if (NULL != api) {
        free(api);
        api = NULL;
    }
#endif
    return 0;
}


int MDrv_TUNER_Resume(void)
{
    U8 i = 0;
    int ret_code = FALSE;
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    for (i=0; i<MAX_TUNER_DEV_NUM; i++)
    {
        if (dev[i].e_status == E_SUSPEND)
        {
            _u8SlaveID = dev[i].u8_slaveID;
            tuner_i2c_channel_bus = dev[i].u8_i2c_bus;
            if (MDrv_TUNER_TunerInit((int)i,&dev[i])==TRUE)
            {
                if (dev[i].e_std == E_TUNER_DTV_DVB_S_MODE)
                {
                    ret_code = MDrv_TUNER_DVBS_SetTune((int)i, dev[i].u32_freq, dev[i].u32_symbolrate);
                    if (ret_code == FALSE)
                    {
                        TUNER_PRINT("Error, DTV_SetTune fail after resume.%d.%d.%d.%d\n",i,dev[i].u32_freq,dev[i].u32_eband_bandwidth,dev[i].u32_eMode);
                    }
                }
                else
                {
                    TUNER_PRINT("Warnning, Undefine STD after resume...indx=%d,std=%d\n",i,dev[i].e_std);
                }
            }
            else
            {
                TUNER_PRINT("Error, Tuner resume init fail...\n");
                ret_code = FALSE;
            }
        }
    }

    return ret_code;
}

#ifdef CONFIG_MSTAR_UTOPIA2K_MDEBUG
ssize_t MDrv_TUNER_GetTUNERMdbInfo(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    return 0;
}

ssize_t MDrv_TUNER_WriteTUNERMdbInfo(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    return 0;
}
#endif


