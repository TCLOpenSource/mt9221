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
//#include <unistd.h>

#include "mst_devid.h"
#include "mdrv_types.h"

#include "mdrv_tuner_datatype.h"
#include "mdrv_iic_io.h"
#include "mdrv_tuner_RT710.h"
const char match_name[15]="Mstar-RT710";
#define Sleep(x) mdelay(x)
#define malloc(x) kmalloc(x, GFP_KERNEL)
#define free kfree
//#define printf TUNER_PRINT
#define TUNER_PRINT(fmt, args...)        printk("[%s][%05d]" fmt, match_name, __LINE__, ## args)
#ifdef TUNER_DEBUG
    #define DBG_TUNER(x) x
#else
    #define DBG_TUNER(x) x
#endif
#ifndef UNUSED //to avoid compile warnings...
    #define UNUSED(var) (void)((var) = (var))
#endif


#if 0 //kdrv
extern int MDrv_SW_IIC_WriteBytes(U8 u8BusNum, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U16 u16BufLen, U8* pu8Buf);
extern int MDrv_SW_IIC_ReadBytes(U8 u8BusNum, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U16 u16BufLen, U8* pu8Buf);
#endif

#define tuner_i2c_SlaveID                       ((U8)  0xF4)
static U8 _u8SlaveID = tuner_i2c_SlaveID;
U8 R0TOR3_Write_Arry[4]={0x00,0x00,0x00,0x00};
static U8 tuner_i2c_channel_bus = 0;  //i2c bus select
//static U8 tuner_i2c_SlaveID=RT710_ADDRESS;


I2C_TYPE_RT710 RT710_Write_Byte;
I2C_LEN_TYPE_RT710 RT710_Write_Len;
I2C_LEN_TYPE_RT710 RT710_Read_Len;

U8  R710_Initial_done_flag = FALSE;
U8  Chip_type_flag = TRUE;
U32 R710_satellite_bw;
U32 R710_pre_satellite_bw;
#define MAX_TUNER_DEV_NUM                     1
static U32 u32_current_IF_Freq = 0;
// static bool bool_standby_flag = FALSE;
static s_Tuner_dev_info *p_dev = NULL;
static s_Tuner_dev_info dev[MAX_TUNER_DEV_NUM] =
{
  {E_TUNER_DTV_DVB_S_MODE, E_CLOSE, 0, 0,0,0,0,0,0,0,0},
};

static RT710_TUNER_NUM_TYPE TUNER_NO = RT710_TUNER_1;
//===========Xtal Cap set (Range is 0~30pF) ==============
//Suggest Xtal_cap use 30pF when xtal CL value is 16pF , Default.
U8 Xtal_cap = 30;  //Unit:pF


RT710_Err_Type RT710_PLL(U32 PLL_Freq);
RT710_Err_Type RT710_Setting(RT710_INFO_Type RT710_Set_Info);
RT710_Err_Type RT710_Standby( RT710_LoopThrough_Type RT710_LTSel, RT710_ClockOut_Type RT710_CLKSel);
RT710_Err_Type RT710_GetRfGain(RT710_RF_Gain_Info *RT710_rf_gain);
RT710_Err_Type RT710_GetRfRssi(U32 RF_Freq_Khz, S32*RfLevelDbm, U8 *fgRfGainflag);
static BOOL MDrv_Tuner_SetFreq(U16 u16CenterFreq_MHz, U32 u32SymbolRate_Ks);
static BOOL MDrv_DVBS_Tuner_Initial(void);
static BOOL RT710_PLL_Lock(void);

/*
void RT710_OEM_I2C_channel_setting(U8 bus_num,U8 SlaveID)
{
    tuner_i2c_channel_bus = bus_num;
    tuner_i2c_SlaveID=SlaveID;
}
*/

#if(RT710_0DBM_SETTING == TRUE)
    //0dBm ; LT:lna output ; LT:HPF off  ; LT Current High
    U8 RT710_Reg_Arry_Init[RT710_Reg_Num] ={0x40, 0x1C, 0xA0, 0x90, 0x41, 0x50, 0xED, 0x25, 0x47, 0x58, 0x38, 0x60, 0x38, 0xE7, 0x90, 0x35};
#else
    //-10~-30dBm ; LT:lna center ; LT:HPF on  ; LT Current Low
    U8 RT710_Reg_Arry_Init[RT710_Reg_Num] ={0x40, 0x1D, 0x20, 0x10, 0x41, 0x50, 0xED, 0x25, 0x07, 0x58, 0x39, 0x64, 0x38, 0xE7, 0x90, 0x35};
#endif

    U8 RT720_Reg_Arry_Init[RT710_Reg_Num] ={0x00, 0x1C, 0x00, 0x10, 0x41, 0x48, 0xDA, 0x4B, 0x07, 0x58, 0x38, 0x40, 0x37, 0xE7, 0x4C, 0x59};

    U8 RT710_Reg_Arry[RT710_Reg_Num];


static U16 RT710_Lna_Acc_Gain[19] =
{
 0, 26, 42, 74, 103, 129, 158, 181, 188, 200,  //0~9
 220, 248, 280, 312, 341, 352, 366, 389, 409,   //10~19
};
static U16 RT710_Lna2_Acc_Gain[32] =
{
 0, 27, 53, 81, 109, 134, 156, 176, 194, 202,   //0~9
 211, 221, 232, 245, 258, 271, 285, 307, 326, 341,  //10~19
 357, 374, 393, 410, 428, 439, 445, 470, 476, 479,  //20~29
 495, 507   //30~31
};

int RT710_Convert(int InvertNum)
{
    int ReturnNum = 0;
    int AddNum    = 0x80;
    int BitNum    = 0x01;
    int CuntNum   = 0;

    for(CuntNum = 0;CuntNum < 8;CuntNum ++)
    {
        if(BitNum & InvertNum)
            ReturnNum += AddNum;

        AddNum /= 2;
        BitNum *= 2;
    }

    return ReturnNum;
}

BOOL RT710_I2C_Write_Len(RT710_TUNER_NUM_TYPE RT710_Tuner_Num, I2C_LEN_TYPE_RT710 *I2C_Info)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    U8 u8Data[50];
    int DataCunt = 0;
    BOOL rbt= TRUE;
    int status;
    for(DataCunt = 0;DataCunt < I2C_Info->Len;DataCunt++)
    {
        u8Data[DataCunt] = I2C_Info->Data[DataCunt];
    }

    status=MDrv_SW_IIC_WriteBytes(tuner_i2c_channel_bus, _u8SlaveID, 1, &(I2C_Info->RegAddr), I2C_Info->Len, u8Data);
    //TUNER_PRINT("MDrv_SW_IIC_WriteBytes :  [%d] \n",status);
    if(status==0)
        rbt= TRUE;
    else
       rbt= FALSE;

    if(rbt != TRUE)
        TUNER_PRINT("Tuner written fail !~~");
    return rbt;
}

BOOL RT710_I2C_Read_Len(RT710_TUNER_NUM_TYPE RT710_Tuner_Num, I2C_LEN_TYPE_RT710 *I2C_Info)
{
    U8 u8Data[50];
    int DataCunt = 0;
    BOOL rbt= TRUE;
    int status;
   status=MDrv_SW_IIC_ReadBytes(tuner_i2c_channel_bus ,_u8SlaveID,1,&(I2C_Info->RegAddr), I2C_Info->Len, u8Data);
   //TUNER_PRINT("MDrv_SW_IIC_ReadBytes :  [%d] \n",status);
   if(status==0)
      rbt= TRUE;
   else
      rbt= FALSE;

   if(rbt != TRUE)
      TUNER_PRINT("Tuner read fail !~~");

    for(DataCunt = 0;DataCunt < I2C_Info->Len;DataCunt++)
    {
         I2C_Info->Data[DataCunt] = RT710_Convert(u8Data[DataCunt]);
    }
   return rbt;
}

BOOL RT710_I2C_Write(RT710_TUNER_NUM_TYPE RT710_Tuner_Num, I2C_TYPE_RT710 *I2C_Info)
{
    int status;
    BOOL rbt= TRUE;
    status=MDrv_SW_IIC_WriteBytes(tuner_i2c_channel_bus, _u8SlaveID, 1, &(I2C_Info->RegAddr), 1, &(I2C_Info->Data));
    //TUNER_PRINT("MDrv_SW_IIC_WriteBytes :  [%d] \n",status);
    if(status==0)
        rbt= TRUE;
    else
       rbt= FALSE;

    if(rbt != TRUE)
        TUNER_PRINT("\033[31m Single Write Tuner IIC FAIL!!!!!\n\033[m");;
    return rbt;
}
RT710_Err_Type RT710_Setting(RT710_INFO_Type RT710_Set_Info)
{
        U8 fine_tune = 0;
        U8 Coarse = 0;
        U16 bw_offset = 20000;
        U32 offset_range = 0;
        int icount=0;
        int i=0;

        RT710_Write_Len.RegAddr=0x00;
        RT710_Write_Len.Len=0x10;

        RT710_Read_Len.RegAddr=0x00;
        RT710_Read_Len.Len=0x04;


        if(R710_Initial_done_flag == FALSE)
            return RT710_Fail;

        // LOOP_THROUGH(0=on ; 1=off)
        if(RT710_Set_Info.RT710_LoopThrough_Mode != LOOP_THROUGH)
        {
            RT710_Reg_Arry[1] |=0x04;
        }
        else
        {
            RT710_Reg_Arry[1] &=0xFB;
        }

        //Clock out(1=off ; 0=on)
        if(RT710_Set_Info.RT710_ClockOut_Mode != ClockOutOn)
        {
            RT710_Reg_Arry[3] |=0x10;
        }
        else
        {
            RT710_Reg_Arry[3] &=0xEF;
        }

        //Output Signal Mode
        if(RT710_Set_Info.RT710_OutputSignal_Mode != DifferentialOut)
        {
            RT710_Reg_Arry[11] |=0x10;
        }
        else
        {
            RT710_Reg_Arry[11] &=0xEF;
        }

        //AGC Type  //R13[4] Negative=0 ; Positive=1;
        if(RT710_Set_Info.RT710_AGC_Mode != AGC_Positive)
        {
            RT710_Reg_Arry[13] &=0xEF;
        }
        else
        {
            RT710_Reg_Arry[13] |=0x10;
        }

        //RT710_Vga_Sttenuator_Type
        if(RT710_Set_Info.RT710_AttenVga_Mode != ATTENVGAON)
        {
            RT710_Reg_Arry[11] &= 0xF7;
        }
        else
        {
            RT710_Reg_Arry[11] |= 0x08;
        }


        if(Chip_type_flag == TRUE)//TRUE:RT710 ; FALSE;RT720
        {
            RT710_Reg_Arry[14] = (RT710_Reg_Arry[14] & 0xFC) | RT710_Set_Info.R710_FineGain;
        }
        else
        {
            if( RT710_Set_Info.R710_FineGain > 1)
                RT710_Reg_Arry[14] = (RT710_Reg_Arry[14] & 0xFE) | 0x01;
            else
                RT710_Reg_Arry[14] = (RT710_Reg_Arry[14] & 0xFE) | 0x00;
        }
        RT710_Reg_Arry[3] = ((RT710_Reg_Arry[3] & 0xF0) | ((30 - Xtal_cap)>>1));

        for(i=0;i<RT710_Write_Len.Len;i++)
        {
            RT710_Write_Len.Data[i]=RT710_Reg_Arry[i];
        }

        if(RT710_I2C_Write_Len(TUNER_NO,&RT710_Write_Len) != RT710_Success)
            return RT710_Fail;


        // Check Input Frequency Range
        if(Chip_type_flag == TRUE)//TRUE:RT710 ; FALSE;RT720
        {
            if((RT710_Set_Info.RF_KHz<600000) || (RT710_Set_Info.RF_KHz>2400000))
                return RT710_Fail;
        }
        else
        {
            if((RT710_Set_Info.RF_KHz<200000) || (RT710_Set_Info.RF_KHz>2400000))
                return RT710_Fail;
        }

        if(RT710_PLL(RT710_Set_Info.RF_KHz)!=RT710_Success)
             return RT710_Fail;

        Sleep(10);

        if(Chip_type_flag == TRUE) //TRUE:RT710 ; FALSE;RT720
        {
            if((RT710_Set_Info.RF_KHz >= 1600000) && (RT710_Set_Info.RF_KHz < 1950000))
            {
                RT710_Reg_Arry[2] |= 0x40; //LNA Mode with att
                RT710_Reg_Arry[8] |= 0x80; //Mixer Buf -3dB
                RT710_Reg_Arry[10] =  (RT710_Reg_Arry_Init[10]); //AGC VTH:1.24V  ;  VTL:0.64V
                RT710_Write_Byte.RegAddr = 0x0A;
                RT710_Write_Byte.Data = RT710_Reg_Arry[10];
                if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
                    return RT710_Fail;
            }
            else
            {
                RT710_Reg_Arry[2] &= 0xBF; //LNA Mode no att
                RT710_Reg_Arry[8] &= 0x7F; //Mixer Buf off
                if(RT710_Set_Info.RF_KHz >= 1950000)
                {
                    RT710_Reg_Arry[10] = ((RT710_Reg_Arry[10] & 0xF0) | 0x08); //AGC VTH:1.04V
                    RT710_Reg_Arry[10] = ((RT710_Reg_Arry[10] & 0x0F) | 0x30); //AGC VTL:0.64V
                    RT710_Write_Byte.RegAddr = 0x0A;
                    RT710_Write_Byte.Data = RT710_Reg_Arry[10];
                    if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
                        return RT710_Fail;
                }
                else
                {
                    RT710_Reg_Arry[10] =  (RT710_Reg_Arry_Init[10]); //AGC VTH:1.14V  ;  VTL:0.64V
                    RT710_Write_Byte.RegAddr = 0x0A;
                    RT710_Write_Byte.Data = RT710_Reg_Arry[10];
                    if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
                        return RT710_Fail;
                }
            }
            RT710_Write_Byte.RegAddr = 0x02;
            RT710_Write_Byte.Data = RT710_Reg_Arry[2];
            if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
                return RT710_Fail;
            RT710_Write_Byte.RegAddr = 0x08;
            RT710_Write_Byte.Data = RT710_Reg_Arry[8];
            if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
                return RT710_Fail;

            if(RT710_Set_Info.RF_KHz >= 2000000)
            {
                RT710_Reg_Arry[14]=( RT710_Reg_Arry[14]& 0xF3)|0x08;
            }
            else
            {
                RT710_Reg_Arry[14]=( RT710_Reg_Arry[14]& 0xF3)|0x00;
            }
            RT710_Write_Byte.RegAddr = 0x0E;
            RT710_Write_Byte.Data = RT710_Reg_Arry[14];
            if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
                return RT710_Fail;
        }
        else
        {
            //Scan_Type
            if(RT710_Set_Info.RT710_Scan_Mode != AUTO_SCAN)
            {
                RT710_Reg_Arry[11] &= 0xFC;
                // sync from sn_mstar_master
                if(RT710_Set_Info.SymbolRate_Kbps >= 15000)
                    RT710_Set_Info.SymbolRate_Kbps += 6000;
            }
            else
            {
                //RT710_Reg_Arry[11] |= 0x03; //5db
                // sync from sn_mstar_master
                RT710_Reg_Arry[11] |= 0x02; ///3db
                RT710_Set_Info.SymbolRate_Kbps += 10000;
            }
            RT710_Write_Byte.RegAddr = 0x0B;
            RT710_Write_Byte.Data = RT710_Reg_Arry[11];
            if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
                return RT710_Fail;
        }


        switch (RT710_Set_Info.RT710_RollOff_Mode)
        {
        case ROLL_OFF_0_15:
            R710_satellite_bw = (U32)(RT710_Set_Info.SymbolRate_Kbps * 115);
            R710_satellite_bw =(U32)(R710_satellite_bw/10);
            break;
        case ROLL_OFF_0_20:
            R710_satellite_bw = (U32)(RT710_Set_Info.SymbolRate_Kbps * 12);
            break;
        case ROLL_OFF_0_25:
            R710_satellite_bw = (U32)(RT710_Set_Info.SymbolRate_Kbps * 125);
            R710_satellite_bw =(U32)(R710_satellite_bw/10);
            break;
        case ROLL_OFF_0_30:
            R710_satellite_bw = (U32)(RT710_Set_Info.SymbolRate_Kbps * 13);
            break;
        case ROLL_OFF_0_35:
            R710_satellite_bw = (U32)(RT710_Set_Info.SymbolRate_Kbps * 135);
            R710_satellite_bw =(U32)(R710_satellite_bw/10);
            break;
        case ROLL_OFF_0_40:
            R710_satellite_bw = (U32)(RT710_Set_Info.SymbolRate_Kbps * 14);
            break;
        }


        if(R710_satellite_bw != R710_pre_satellite_bw)
        {
            if(Chip_type_flag == TRUE)//TRUE:RT710 ; FALSE;RT720
            {
                if((R710_satellite_bw) >=380000)
                {
                    fine_tune=1;
                    Coarse = (( R710_satellite_bw -380000) /17400)+16;
                    if((U32)(( R710_satellite_bw -380000) % 17400) > 0)
                    Coarse+=1;
                }
                else if(R710_satellite_bw<=50000)
                {
                    Coarse=0;
                    fine_tune=0;
                }
                else if((R710_satellite_bw>50000) && (R710_satellite_bw<=73000))
                {
                    Coarse=0;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>73000) && (R710_satellite_bw<=96000))
                {
                    Coarse=1;
                    fine_tune=0;
                }
                else if((R710_satellite_bw>96000) && (R710_satellite_bw<=104000))
                {
                    Coarse=1;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>104000) && (R710_satellite_bw<=116000))
                {
                    Coarse=2;
                    fine_tune=0;
                }
                else if((R710_satellite_bw>116000) && (R710_satellite_bw<=126000))
                {
                    Coarse=2;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>126000) && (R710_satellite_bw<=134000))
                {
                    Coarse=3;
                    fine_tune=0;
                }
                else if((R710_satellite_bw>134000) && (R710_satellite_bw<=146000))
                {
                    Coarse=3;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>146000) && (R710_satellite_bw<=158000))
                {
                    Coarse=4;
                    fine_tune=0;
                }
                else if((R710_satellite_bw>158000) && (R710_satellite_bw<=170000))
                {
                    Coarse=4;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>170000) && (R710_satellite_bw<=178000))
                {
                    Coarse=5;
                    fine_tune=0;
                }
                else if((R710_satellite_bw>178000) && (R710_satellite_bw<=190000))
                {
                    Coarse=5;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>190000) && (R710_satellite_bw<=202000))
                {
                    Coarse=6;
                    fine_tune=0;
                }
                else if((R710_satellite_bw>202000) && (R710_satellite_bw<=212000))
                {
                    Coarse=6;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>212000) && (R710_satellite_bw<=218000))
                {
                    Coarse=7;
                    fine_tune=0;
                }
                else if((R710_satellite_bw>218000) && (R710_satellite_bw<=234000))
                {
                    Coarse=7;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>234000) && (R710_satellite_bw<=244000))
                {
                    Coarse=9;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>244000) && (R710_satellite_bw<=246000))
                {
                    Coarse=10;
                    fine_tune=0;
                }
                else if((R710_satellite_bw>246000) && (R710_satellite_bw<=262000))
                {
                    Coarse=10;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>262000) && (R710_satellite_bw<=266000))
                {
                    Coarse=11;
                    fine_tune=0;
                }
                else if((R710_satellite_bw>266000) && (R710_satellite_bw<=282000))
                {
                    Coarse=11;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>282000) && (R710_satellite_bw<=298000))
                {
                    Coarse=12;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>298000) && (R710_satellite_bw<=318000))
                {
                    Coarse=13;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>318000) && (R710_satellite_bw<=340000))
                {
                    Coarse=14;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>340000) && (R710_satellite_bw<=358000))
                {
                    Coarse=15;
                    fine_tune=1;
                }
                else if((R710_satellite_bw>358000) && (R710_satellite_bw<380000))
                {
                    Coarse=16;
                    fine_tune=1;
                }
            }
            else
            {

                if(RT710_Set_Info.RT710_RollOff_Mode>= 2 )
                fine_tune=1;
                else
                fine_tune=0;

                offset_range = fine_tune * bw_offset ;

                /*Symbol Rate <= 15MHz =>  +3MHz
                Symbol Rate 15MHz ~ 20MHz  =>  +2MHz
                Symbol Rate <= 30MHz =>  +1MHz
                */
                if(RT710_Set_Info.SymbolRate_Kbps<=15000)
                    RT710_Set_Info.SymbolRate_Kbps += 3000;
                else if(RT710_Set_Info.SymbolRate_Kbps<=20000)
                    RT710_Set_Info.SymbolRate_Kbps += 2000;
                else if(RT710_Set_Info.SymbolRate_Kbps<=30000)
                    RT710_Set_Info.SymbolRate_Kbps += 1000;
                else
                    RT710_Set_Info.SymbolRate_Kbps += 0;

                if((RT710_Set_Info.SymbolRate_Kbps*12) < (88000+offset_range) )
                {
                    Coarse=0;
                }
                else if((RT710_Set_Info.SymbolRate_Kbps*12) > (88000+offset_range) && (RT710_Set_Info.SymbolRate_Kbps*12) <= (368000+offset_range) )
                {
                    Coarse = (( (RT710_Set_Info.SymbolRate_Kbps*12) - (88000+offset_range)) / 20000);
                    if(((U16)((RT710_Set_Info.SymbolRate_Kbps*12) -(88000+offset_range)) % 20000) > 0)
                        Coarse+=1;
                    if(Coarse>=7)
                        Coarse+=1;
                }
                else if((RT710_Set_Info.SymbolRate_Kbps*12) > (368000+offset_range) && (RT710_Set_Info.SymbolRate_Kbps*12)<=(764000+offset_range))
                {
                    Coarse = (( (RT710_Set_Info.SymbolRate_Kbps*12) -(368000+offset_range)) /20000) +15;
                    if(((U16)( (RT710_Set_Info.SymbolRate_Kbps*12) -(368000+offset_range)) % 20000) > 0)
                        Coarse+=1;

                    if(Coarse>=33)
                        Coarse+=3;
                    else if(Coarse >= 29)
                        Coarse+=2;
                    else if(Coarse>=27)
                        Coarse+=3;
                    else if(Coarse>=24)
                        Coarse+=2;
                    else if(Coarse>=19)
                        Coarse+=1;
                    else
                        Coarse+=0;
                }
                else
                {
                    Coarse=42;
                }

            }

            //fine tune and coras filter write
            RT710_Write_Byte.RegAddr = 0x0F;
            RT710_Read_Len.Data[15] &= 0x00;
            RT710_Reg_Arry[15] = ((RT710_Read_Len.Data[15] | ( fine_tune  ) ) | ( Coarse<<2));
            RT710_Write_Byte.Data = RT710_Reg_Arry[15];
            if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
                return RT710_Fail;
        }

        R710_pre_satellite_bw = R710_satellite_bw;

        for(icount=0;icount<4;icount++)
        {
            R0TOR3_Write_Arry[icount]=RT710_Reg_Arry[icount];
        }
        return RT710_Success;
}

RT710_Err_Type RT710_PLL(U32 Freq)
{
    U8  MixDiv   = 2;
    // MAPI_U8  DivBuf   = 0;
    U8  Ni       = 0;
    U8  Si       = 0;
    U8  DivNum   = 0;
    U8  Nint     = 0;
    U32 VCO_Min  = 2350000;
    U32 VCO_Max  = VCO_Min*2;
    U32 VCO_Freq = 0;
    U32 PLL_Ref    = RT710_Xtal;
    U32 VCO_Fra    = 0;        //VCO contribution by SDM (kHz)
    U16 Nsdm       = 2;
    U16 SDM        = 0;
    U16 SDM16to9   = 0;
    U16 SDM8to1  = 0;
    //MAPI_U8  Judge    = 0;
    // MAPI_U8  VCO_fine_tune = 0;
    if(R710_Initial_done_flag == FALSE)
        return RT710_Fail;
    if(Chip_type_flag == FALSE)
        PLL_Ref = RT720_Xtal;
    if (Freq>=2350000)
    {
        VCO_Min  = Freq;
    }
    while(MixDiv <= 16)  // 2 -> 4 -> 8 -> 16
    {
        if(((Freq * MixDiv) >= VCO_Min) && ((Freq * MixDiv) <= VCO_Max))// Lo_Freq->Freq
        {
            if(MixDiv==2)
                DivNum = 1;
            else if(MixDiv==4)
                DivNum = 0;
            else if(MixDiv==8)
                DivNum = 2;
            else   //16
                DivNum = 3;
            break;
        }
        MixDiv = MixDiv << 1;
    }

    //Divider
    RT710_Write_Byte.RegAddr = 0x04;
    RT710_Reg_Arry[4] &= 0xFE;
    RT710_Reg_Arry[4] |= (DivNum & 0x01) ;
    RT710_Write_Byte.Data = RT710_Reg_Arry[4];
    if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
        return RT710_Fail;

    if(Chip_type_flag == FALSE)//TRUE:RT710 ; FALSE;RT720
    {
        RT710_Write_Byte.RegAddr = 0x08;
        RT710_Reg_Arry[8] &= 0xEF;
        RT710_Reg_Arry[8] |= ((DivNum & 0x02)>>1) << 4;
        RT710_Write_Byte.Data = RT710_Reg_Arry[8];
        if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
            return RT710_Fail;
        //Depend on divider setting
        //If Div /2 or /4 : Div Current 150uA(10); other : Div Current 100uA(01) R4[7:6]
        //If Div /2 or /4 : PLL IQ Buf High(1); other : PLL IQ Buf Low(0) R12[4]
        if(DivNum <= 1) // Div/2 or /4
        {
            RT710_Reg_Arry[4] &= 0x3F;
            RT710_Reg_Arry[4] |= 0x40;
            RT710_Reg_Arry[12] |= 0x10;

        }
        else    //Div /8 or /16
        {
            RT710_Reg_Arry[4] &= 0x3F;
            RT710_Reg_Arry[4] |= 0x80;
            RT710_Reg_Arry[12] &= 0xEF;

        }
        RT710_Write_Byte.RegAddr = 0x0C;
        RT710_Write_Byte.Data = RT710_Reg_Arry[12];
        if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
            return RT710_Fail;
        RT710_Write_Byte.RegAddr = 0x04;
        RT710_Write_Byte.Data = RT710_Reg_Arry[4];
        if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
            return RT710_Fail;
    }

    VCO_Freq = Freq * MixDiv;       // Lo_Freq->Freq
    Nint     = (U8) (VCO_Freq / 2 / PLL_Ref);
    VCO_Fra  = (U16) (VCO_Freq - 2 * PLL_Ref * Nint);

    //boundary spur prevention
    if (VCO_Fra < (PLL_Ref/64))             // 2*PLL_Ref/128
        VCO_Fra = 0;
    else if (VCO_Fra > (PLL_Ref*127/64))    // 2*PLL_Ref*127/128
    {
        VCO_Fra = 0;
        Nint ++;
    }
    else if((VCO_Fra >  (PLL_Ref*127/128)) && (VCO_Fra < PLL_Ref)) //> 2*PLL_Ref*127/256,  < 2*PLL_Ref*128/256
        VCO_Fra = (PLL_Ref*127/128);      // VCO_Fra = 2*PLL_Ref*127/256
    else if((VCO_Fra > PLL_Ref) && (VCO_Fra < (PLL_Ref*129/128))) //> 2*PLL_Ref*128/256,  < 2*PLL_Ref*129/256
        VCO_Fra = (PLL_Ref*129/128);      // VCO_Fra = 2*PLL_Ref*129/256
    else
        VCO_Fra = VCO_Fra;

    //N & S
    Ni       = (U8)((Nint - 13) / 4);
    Si       = (U8)(Nint - 4 *Ni - 13);
    RT710_Write_Byte.RegAddr = 0x05;
    RT710_Reg_Arry[5]  = 0x00;
    RT710_Reg_Arry[5] |= (Ni + (Si << 6));
    RT710_Write_Byte.Data  = RT710_Reg_Arry[5];
    if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
        return RT710_Fail;

    //pw_sdm
    RT710_Write_Byte.RegAddr = 0x04;
    RT710_Reg_Arry[4] &= 0xFD;
    if(VCO_Fra == 0)
        RT710_Reg_Arry[4] |= 0x02;
    RT710_Write_Byte.Data = RT710_Reg_Arry[4];
    if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
        return RT710_Fail;

    //SDM calculator
    while(VCO_Fra > 1)
    {
        if (VCO_Fra > (2*PLL_Ref / Nsdm))
        {
            SDM = SDM + 32768 / (Nsdm/2);
            VCO_Fra = VCO_Fra - 2*PLL_Ref / Nsdm;
            if (Nsdm >= 0x8000)
                break;
        }
        Nsdm = Nsdm << 1;
    }

    SDM16to9 = SDM >> 8;
    SDM8to1 =  SDM - (SDM16to9 << 8);

    RT710_Write_Byte.RegAddr = 0x07;
    RT710_Reg_Arry[7]   = (U8) SDM16to9;
    RT710_Write_Byte.Data    = RT710_Reg_Arry[7];
    if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
        return RT710_Fail;
    RT710_Write_Byte.RegAddr = 0x06;
    RT710_Reg_Arry[6]   = (U8) SDM8to1;
    RT710_Write_Byte.Data    = RT710_Reg_Arry[6];
    if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
        return RT710_Fail;
    return RT710_Success;
}

RT710_Err_Type RT710_Standby( RT710_LoopThrough_Type RT710_LTSel, RT710_ClockOut_Type RT710_CLKSel )
{
    int icount=0;
    int i=0;
    U8 RT710_Standby_Reg_Arry[RT710_Reg_Num]={0xFF, 0x5C, 0x88, 0x30, 0x41, 0xC8, 0xED, 0x25, 0x47, 0xFC, 0x48, 0xA2, 0x08, 0x0F, 0xF3, 0x59};
    //Clock out(1=off ; 0=on)
    if(R710_Initial_done_flag == FALSE)
        return RT710_Success;
    if(RT710_CLKSel != ClockOutOn)
    {
        RT710_Standby_Reg_Arry[3] |=0x10;
    }
    else
    {
        RT710_Standby_Reg_Arry[3] &=0xEF;
    }

    if(Chip_type_flag == FALSE)//TRUE:RT710 ; FALSE;RT720
    {
        RT710_Standby_Reg_Arry[1] |=0x02;//lna off ;can off together
        RT710_Standby_Reg_Arry[3] |=0x20;//Xtal ldo off
        RT710_Write_Byte.RegAddr = 0x03;
        RT710_Write_Byte.Data  = RT710_Standby_Reg_Arry[3];
        if(RT710_I2C_Write(TUNER_NO,&RT710_Write_Byte) != RT710_Success)
            return RT710_Fail;
    }

    RT710_Write_Len.RegAddr=0x00;
    RT710_Write_Len.Len=0x10;
    for(i=0;i<RT710_Write_Len.Len;i++)
    {
        RT710_Write_Len.Data[i]=RT710_Standby_Reg_Arry[i];
    }

    if(RT710_I2C_Write_Len(TUNER_NO,&RT710_Write_Len) != RT710_Success)
        return RT710_Fail;

    R710_Initial_done_flag = FALSE;

    for(icount=0;icount<4;icount++)
    {
        R0TOR3_Write_Arry[icount]=RT710_Standby_Reg_Arry[icount];
    }

    return RT710_Success;
}


//------------------------------------------------------------------//
//  RT710_PLL_Lock( ): Read PLL lock status (R2[7])        //
//  Return: 1: PLL lock                                                    //
//          0: PLL unlock                                                //
//------------------------------------------------------------------//
BOOL RT710_PLL_Lock(void)
{
    U8 fg_lock = 0;
    I2C_LEN_TYPE_RT710 Dlg_I2C_Len;

    Dlg_I2C_Len.RegAddr = 0x00;
    Dlg_I2C_Len.Len = 3;
    if(RT710_I2C_Read_Len(TUNER_NO,&Dlg_I2C_Len) != RT710_Success)
    {
        RT710_I2C_Read_Len(TUNER_NO,&Dlg_I2C_Len);
    }

    if( (Dlg_I2C_Len.Data[2] & 0x80) == 0x00 )
        fg_lock = 0;
    else
        fg_lock = 1;

    TUNER_PRINT("fg_lock=%d\n", fg_lock);
    return fg_lock;
}



RT710_Err_Type RT710_GetRfGain(RT710_RF_Gain_Info *RT710_rf_gain)
{
    I2C_LEN_TYPE_RT710 Dlg_I2C_Len;
    Dlg_I2C_Len.RegAddr = 0x00;
    Dlg_I2C_Len.Len     = 0x04;
    RT710_I2C_Read_Len(TUNER_NO,&Dlg_I2C_Len) ; // read data length
    RT710_rf_gain->RF_gain = (Dlg_I2C_Len.Data[1]>>4); //get rf gain
    RT710_rf_gain->RF_gain +=((Dlg_I2C_Len.Data[1] & 0x01)<<4);

    /*0~5: mixeramp
      6~7: mix-buf
      29~30:mix-buf
      other:lna
    */
    if(Chip_type_flag == TRUE)
    {
        if (RT710_rf_gain->RF_gain <= 2)
        {
            RT710_rf_gain->RF_gain=0;
        }
        else if(RT710_rf_gain->RF_gain > 2 && RT710_rf_gain->RF_gain <= 9)
        {
            RT710_rf_gain->RF_gain -=2;
        }
        else if(RT710_rf_gain->RF_gain >9 && RT710_rf_gain->RF_gain <=12)
        {
            RT710_rf_gain->RF_gain = 7;
        }
        else if(RT710_rf_gain->RF_gain>12 && RT710_rf_gain->RF_gain <= 22)
        {
            RT710_rf_gain->RF_gain -= 5;
        }
        else if(RT710_rf_gain->RF_gain > 22)
        {
            RT710_rf_gain->RF_gain = 18;
        }
    }

    return RT710_Success;
}

//----------------------------------------------------------------------//
//  RT710_GetRfRssi( ): Get RF RSSI                                      //
//  1st parameter: input RF Freq    (KHz)                               //
//  2rd parameter: output signal level (dBm*1000)                       //
//  3th parameter: output RF max gain indicator (0:min gain, 1:max gain, 2:active gain) //
//-----------------------------------------------------------------------//
RT710_Err_Type RT710_GetRfRssi(U32 RF_Freq_Khz, S32 *RfLevelDbm, U8 *fgRfGainflag)
{
    RT710_RF_Gain_Info rf_gain_info;
    U8  u1Gain1;
    U16  acc_lna_gain;
    U16  rf_total_gain;
    U16  u2FreqFactor;
    S32     rf_rssi;
    S32    fine_tune = 0;    //for find tune

    RT710_GetRfGain(&rf_gain_info);

    u1Gain1 = rf_gain_info.RF_gain;

   //max gain indicator
    if(((u1Gain1==18)&&(Chip_type_flag==TRUE))||((u1Gain1==31)&&(Chip_type_flag==FALSE)))
        *fgRfGainflag = 1;
    else if(u1Gain1==0)
        *fgRfGainflag = 0;
    else
        *fgRfGainflag = 2;

    u2FreqFactor = 0;

    //Select LNA freq table
    if(Chip_type_flag==FALSE)
    {
        u2FreqFactor = 70;
    }
    else if(RF_Freq_Khz<1200000)   //<1200M
    {
        u2FreqFactor = 190;
    }
    else if((RF_Freq_Khz>=1200000)&&(RF_Freq_Khz<1800000))   //1200~1800M
    {
        u2FreqFactor = 170;
    }
    else    // >=2000MHz
    {
        u2FreqFactor = 140;
    }

    //LNA Gain
    if(Chip_type_flag==FALSE)
        acc_lna_gain = RT710_Lna2_Acc_Gain[u1Gain1];
    else
        acc_lna_gain = RT710_Lna_Acc_Gain[u1Gain1];

    //Add Rf Buf and Mixer Gain
    rf_total_gain = acc_lna_gain;

    rf_rssi = fine_tune - (S32) (rf_total_gain + u2FreqFactor);
    *RfLevelDbm = (rf_rssi*100);


    return RT710_Success;
}


/////////// Global Functions////////////////////
BOOL MDrv_Tuner_SetFreq(U16 u16CenterFreq_MHz, U32 u32SymbolRate_Ks)
{
    //Set RT710_Info
    RT710_INFO_Type RT710_Info_Msg;
    RT710_Info_Msg.RF_KHz = (u16CenterFreq_MHz * 1000); // Set RF Freq. unit: KHz
    #if 0
    #ifdef DVBS_RRPLACE_BY_SLT
    if (eBW == 0)
    {
        RT710_Info_Msg.DVBSBW = u32SymbolRate_Ks * 2; //BW = Symbol Rate * (2)
        // RT710_Info_Msg.DVBSBW = dSym_KHz * 27 / 20; //BW = Symbol Rate * (1 + Roll Off), Roll Off = 0.35
    }
    else
    {
        RT710_Info_Msg.DVBSBW = u32SymbolRate_Ks;
    }
    #elif defined DVBS2_BS_WQ
    RT710_Info_Msg.DVBSBW = u32SymbolRate_Ks;
    #else
    if (g_RT710_fIsScanning)
        RT710_Info_Msg.DVBSBW = u32SymbolRate_Ks;
    else
        RT710_Info_Msg.DVBSBW = u32SymbolRate_Ks * 27 / 20; //BW = Symbol Rate * (1 + Roll Off), Roll Off = 0.35
    #endif
    #endif
    RT710_Info_Msg.SymbolRate_Kbps = (U32)(u32SymbolRate_Ks * 27 / 20);//chenhongxiang@2015/04/16 Remark: For DVBS2 8-PSK can't detect!
    RT710_Info_Msg.RT710_RollOff_Mode = ROLL_OFF_0_25;
    RT710_Info_Msg.RT710_LoopThrough_Mode = SIGLE_IN; //Set LoopThrough ON
    RT710_Info_Msg.RT710_ClockOut_Mode = ClockOutOff; //Set Clock Out
    RT710_Info_Msg.RT710_OutputSignal_Mode = DifferentialOut; //Modify for UCCP spectrum analyser
    RT710_Info_Msg.RT710_AGC_Mode = AGC_Negative;
    RT710_Info_Msg.RT710_AttenVga_Mode = ATTENVGAOFF;
    RT710_Info_Msg.R710_FineGain = FINEGAIN_3DB;
    RT710_Info_Msg.RT710_Scan_Mode = MANUAL_SCAN; // support only RT720

    //Run RT710_Setting
    if(RT710_Setting(RT710_Info_Msg) != RT710_Success)
        return RT710_Fail;

    u32_current_IF_Freq = RT710_Info_Msg.RF_KHz;
    return RT710_Success;

}

int MDrv_TUNER_DVBS_SetTune(int minor, U16 u16CenterFreqMHz, U32 u32SymbolRateKs)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    BOOL bRet;
    bRet=TRUE;
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

    bRet = MDrv_Tuner_SetFreq (u16CenterFreqMHz,u32SymbolRateKs);
    if (bRet != TRUE)
    {
        TUNER_PRINT("Tuner_Set_Channel_Frequency fail, error_code\n");
        bRet = false;
    }
    return bRet;
}



//  to do
BOOL MDrv_DVBS_Tuner_Initial(void)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
  // There is no init function of RT710
    RT710_INFO_Type RT710_Set_Info;
    int icount=0;
    int i;
    RT710_Set_Info.RT710_LoopThrough_Mode = LOOP_THROUGH; //Set LoopThrough ON
    RT710_Set_Info.RT710_ClockOut_Mode = ClockOutOn; //Set Clock Out
    RT710_Set_Info.RT710_OutputSignal_Mode = DifferentialOut; //Modify for UCCP spectrum analyser
    RT710_Set_Info.RT710_AGC_Mode = AGC_Negative;
    RT710_Set_Info.RT710_AttenVga_Mode = ATTENVGAOFF;
    RT710_Set_Info.R710_FineGain = FINEGAIN_3DB;
    RT710_Set_Info.RT710_Scan_Mode = MANUAL_SCAN; // support only RT720

    RT710_Write_Len.RegAddr=0x00;
    RT710_Write_Len.Len=0x10;

    RT710_Read_Len.RegAddr=0x00;
    RT710_Read_Len.Len=0x04;


    if(R710_Initial_done_flag == FALSE)
    {
        R710_pre_satellite_bw = 0; //Init BW Value
        R710_satellite_bw = 0;


        RT710_I2C_Read_Len(TUNER_NO,&RT710_Read_Len) ; // read data length
        if((RT710_Read_Len.Data[3]&0xF0)==0x70) //TRUE:RT710(R3[7:4]=7) ; FALSE;RT720(R3[7:4]=0)
            Chip_type_flag=TRUE;
        else
            Chip_type_flag=FALSE;



        if(Chip_type_flag == TRUE)//TRUE:RT710 ; FALSE;RT720
        {
            for(icount=0;icount<RT710_Reg_Num;icount++)
            {
                RT710_Reg_Arry[icount]=RT710_Reg_Arry_Init[icount];
            }
        }
        else
        {
            for(icount=0;icount<RT710_Reg_Num;icount++)
            {
                RT710_Reg_Arry[icount]=RT720_Reg_Arry_Init[icount];
            }
        }


        // LOOP_THROUGH(0=on ; 1=off)
        if(RT710_Set_Info.RT710_LoopThrough_Mode != LOOP_THROUGH)
        {
            RT710_Reg_Arry[1] |=0x04;
        }
        else
        {
            RT710_Reg_Arry[1] &=0xFB;
        }

        //Clock out(1=off ; 0=on)
        if(RT710_Set_Info.RT710_ClockOut_Mode != ClockOutOn)
        {
            RT710_Reg_Arry[3] |=0x10;
        }
        else
        {
            RT710_Reg_Arry[3] &=0xEF;
        }

        //Output Signal Mode
        if(RT710_Set_Info.RT710_OutputSignal_Mode != DifferentialOut)
        {
            RT710_Reg_Arry[11] |=0x10;
        }
        else
        {
            RT710_Reg_Arry[11] &=0xEF;
        }

        //AGC Type  //R13[4] Negative=0 ; Positive=1;
        if(RT710_Set_Info.RT710_AGC_Mode != AGC_Positive)
        {
            RT710_Reg_Arry[13] &=0xEF;
        }
        else
        {
            RT710_Reg_Arry[13] |=0x10;
        }

        //RT710_Vga_Sttenuator_Type
        if(RT710_Set_Info.RT710_AttenVga_Mode != ATTENVGAON)
        {
            RT710_Reg_Arry[11] &= 0xF7;
        }
        else
        {
            RT710_Reg_Arry[11] |= 0x08;
        }


        if(Chip_type_flag == TRUE)//TRUE:RT710 ; FALSE;RT720
        {
            RT710_Reg_Arry[14] = (RT710_Reg_Arry[14] & 0xFC) | RT710_Set_Info.R710_FineGain;
        }
        else
        {
            if( RT710_Set_Info.R710_FineGain > 1)
                RT710_Reg_Arry[14] = (RT710_Reg_Arry[14] & 0xFE) | 0x01;
            else
                RT710_Reg_Arry[14] = (RT710_Reg_Arry[14] & 0xFE) | 0x00;
        }
        RT710_Reg_Arry[3] = ((RT710_Reg_Arry[3] & 0xF0) | ((30 - Xtal_cap)>>1));

        for(i=0;i<RT710_Write_Len.Len;i++)
        {
            RT710_Write_Len.Data[i]=RT710_Reg_Arry[i];
        }
        if(RT710_I2C_Write_Len(TUNER_NO,&RT710_Write_Len) != RT710_Success)
            return RT710_Fail;

        R710_Initial_done_flag = TRUE;
    }
    TUNER_PRINT(" R710_Initial_done_flag==[%d]\n",R710_Initial_done_flag);
    return TRUE;
}

int MDrv_TUNER_TunerInit(int minor, s_Tuner_dev_info *devTunerInit)
{
    BOOL bRet=TRUE;
    //U8 index;
    _u8SlaveID=devTunerInit->u8_slaveID;
    tuner_i2c_channel_bus = devTunerInit->u8_i2c_bus;
    //RT710_OEM_I2C_channel_setting(bus_channel_num,SlaveID);
    TUNER_PRINT("%s is invoked  u8Slave:[0x%x] bus_channel_num: [%d]\n", __FUNCTION__,_u8SlaveID,tuner_i2c_channel_bus);
    if (minor < MAX_TUNER_DEV_NUM)
    {
        p_dev=&(dev[minor]);
        p_dev->u8_slaveID = _u8SlaveID;
        p_dev->u8_i2c_bus = tuner_i2c_channel_bus;
        if ( (p_dev->e_status == E_CLOSE)
        || (p_dev->e_status == E_SUSPEND) )
        {
            bRet = MDrv_DVBS_Tuner_Initial();
            if (bRet != TRUE)
            {
                TUNER_PRINT("Tuner_Parameter_Initial fail, error_code\n");
            }
            else
            {
                p_dev->e_status = E_WORKING;
            }
        }
    }
    else
    {
        bRet = FALSE;
    }
    return bRet;
}





BOOL MDrv_TUNER_IsLocked(int minor)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    if(R710_Initial_done_flag==FALSE)
        return RT710_Fail;

    return RT710_PLL_Lock();
}
/////////

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

U32 MDrv_TUNER_GetRSSI(U16 u16_gain, U8 Dtype)
{
    RT710_Err_Type Err_code = RT710_Success;
    S32 rssi_1000dBm = 0;
    U8  gain_flag = 0;
    S32 rssi_dbm = 0;

    u16_gain= u16_gain;
    Dtype =Dtype;

    Err_code = RT710_GetRfRssi(u32_current_IF_Freq, &rssi_1000dBm, &gain_flag);

    if (Err_code == RT710_Fail)
    {
        rssi_dbm = -199;
    }
    else
        rssi_dbm = (S32)rssi_1000dBm;

    return rssi_dbm;
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


