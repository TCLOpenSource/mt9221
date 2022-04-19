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

#include "MxL661/MaxLinearDataTypes.h"
#include "MxL661/MxL661_OEM_Drv.h"
#include "MxL661/MxL661_TunerApi.h"
#include "MxL661/MxL661_TunerCfg.h"
#include "MxL661/MxL661_TunerSpurTable.h"
#include "BypassEWBSIIC.h"
const char match_name[16]="Mstar-MXL661";
//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define malloc(x) kmalloc(x, GFP_KERNEL)
#define free kfree

#define TUNER_PRINT(fmt, args...)        printk("[%s][%05d] " fmt, match_name, __LINE__, ## args)
#ifdef TUNER_DEBUG
    #define DBG_TUNER(x) x
#else
    #define DBG_TUNER(x) //x
#endif
#ifndef UNUSED //to avoid compile warnings...
#define UNUSED(var) (void)((var) = (var))
#endif

//#define mcSHOW_HW_MSG(fmt, args...)

#define MAX_TUNER_DEV_NUM                     2
#define MXL661_DEFAULTCAP                     31
#define MXL661_I2C_ADDR                       ((U8)  0xC0)
#define MSTAR_INTERN_VIF                      1
#define FRONTEND_IF_DEMODE_TYPE               MSTAR_INTERN_VIF
#define MAX_FINE_TUNE_STEP                    32
#define FREQ_STEP                             62.5//50  //Khz
#define FREQ_STEP_10                          625//62.5//50  //Khz
#define TN_FREQ_STEP                          E_FREQ_STEP_62_5KHz
#define ATV_TUNER_IF_FREQ_KHz                 5000L
#define DTV_TUNER_IF_FREQ_KHz                 5000L
#define DTMB_TUNER_IF_FREQ_KHz                5000L
#define TUNER_L_PRIME_IF_FREQ_KHz             5000L
//#define IF_FREQ_A                       4100L
#define ATV_IF_STEP                       (ATV_TUNER_IF_FREQ_KHz/FREQ_STEP)
#define DTV_IF_STEP                       (DTV_TUNER_IF_FREQ_KHz/FREQ_STEP)
#define TUNER_PLL_STABLE_TIME             30
#define SECAM_L_PRIME_TUNER_PLL_STABLE_TIME     35

#define TUNER_DTV_IF_FREQ_KHz      DTV_TUNER_IF_FREQ_KHz
#define TUNER_DTV_IQ_SWAP           0

#define TUNER_VHF_LOWMIN_FREQ             47000L
#define TUNER_VHF_LOWMAX_FREQ             160000L
#define TUNER_VHF_HIGHMIN_FREQ            160000L
#define TUNER_VHF_HIGHMAX_FREQ            445000L
#define TUNER_UHF_MIN_FREQ_UK             445000L
#define TUNER_UHF_MIN_FREQ                434000L
#define TUNER_UHF_MAX_FREQ                894000L

//define lock range of AFT value
#define MIN_AFT_VALUE                       24
#define MAX_AFT_VALUE                         38
#define FINE_MIN_AFT_VALUE                  30
#define CENTER_AFT_VALUE                    31
#define FINE_MAX_AFT_VALUE                 2
#define MAX_ZIG_ZAG_NUM                      7
#define NTSC_TUNER_PLL_STABLE_TIME     20       //normally is smaller then 6ms
#define TUNER_HIBYTE(w)                       ((U8) (((U16) (w) >> 8) & 0xFF))
#define TUNER_LOBYTE(w)                      ((U8) (w & 0xFF))

#define VIF_TUNER_TYPE 1     // 0: RF Tuner; 1: Silicon Tuner
#define VIF_CR_RATE_B                   0x000ABDA1 // 5+2.25M   // [21:0], CR_RATE for 6.4 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_B                     0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_GH                  0x000B7B42  //5+2.75M         // [21:0], CR_RATE for 1.52 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_GH                    0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_DK                  0x000B7B42// //5+2.75M           // [21:0], CR_RATE for 1.52 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_DK                    0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_I                   0x000B7B42 //5+2.75M          // [21:0], CR_RATE for 1.52 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_I                     0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_L                   0x000B7B42 // 5+2.75M         // [21:0], CR_RATE for 1.52 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_L                     0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_LL                  0x000B7B42// //5+2.75M          // [21:0], CR_RATE for 7.02 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_LL                    0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_MN                  0x000A0000// //5+1.75M         // [21:0], CR_RATE for 1.75 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_MN                    0                   // Video carrier position; 0: high side; 1:low side


#define VIF_SOS_21_FILTER_C0            0x032D
#define VIF_SOS_21_FILTER_C1            0x069B
#define VIF_SOS_21_FILTER_C2            0x0209
#define VIF_SOS_21_FILTER_C3            0x04D3
#define VIF_SOS_21_FILTER_C4            0x015C

#define VIF_SOS_22_FILTER_C0            0x034D
#define VIF_SOS_22_FILTER_C1            0x068E
#define VIF_SOS_22_FILTER_C2            0x01F8
#define VIF_SOS_22_FILTER_C3            0x04B3
#define VIF_SOS_22_FILTER_C4            0x017A

#define VIF_SOS_31_FILTER_C0            0x0217
#define VIF_SOS_31_FILTER_C1            0x0754
#define VIF_SOS_31_FILTER_C2            0x00AC
#define VIF_SOS_31_FILTER_C3            0x05E9
#define VIF_SOS_31_FILTER_C4            0x0200

#define VIF_SOS_32_FILTER_C0            0x0000
#define VIF_SOS_32_FILTER_C1            0x0000
#define VIF_SOS_32_FILTER_C2            0x0000
#define VIF_SOS_32_FILTER_C3            0x0000
#define VIF_SOS_32_FILTER_C4            0x0200


//B_Stereo_A2 ========================
// 1.75Mhz
#define N_A1_C0_B_A2 0x03A4
#define N_A1_C1_B_A2 0x063C
#define N_A1_C2_B_A2 0x0421
// 1.51Mhz
#define N_A2_C0_B_A2 0x03AB
#define N_A2_C1_B_A2 0x063C
#define N_A2_C2_B_A2 0x0419
// 8.75Mhz
#define S_11_C0_B_A2 0x011B
#define S_11_C1_B_A2 0x063C
#define S_11_C2_B_A2 0x0200
#define S_11_C3_B_A2 0x06D3
#define S_11_C4_B_A2 0x0200
// 1.75Mhz
#define S_12_C0_B_A2 0x03A4
#define S_12_C1_B_A2 0x063C
#define S_12_C2_B_A2 0x0200
#define S_12_C3_B_A2 0x0421
#define S_12_C4_B_A2 0x0200

//B_Mono_NICAM ====================
// 1.75Mhz
#define N_A1_C0_B_NICAM 0x03A3
#define N_A1_C1_B_NICAM 0x063C
#define N_A1_C2_B_NICAM 0x0421
// 1.4Mhz
#define N_A2_C0_B_NICAM 0x03AF
#define N_A2_C1_B_NICAM 0x063C
#define N_A2_C2_B_NICAM 0x0415
// 8.75Mhz
#define S_11_C0_B_NICAM 0x011B
#define S_11_C1_B_NICAM 0x063C
#define S_11_C2_B_NICAM 0x0200
#define S_11_C3_B_NICAM 0x06D3
#define S_11_C4_B_NICAM 0x0200
// 1.75Mhz
#define S_12_C0_B_NICAM 0x03A4
#define S_12_C1_B_NICAM 0x063C
#define S_12_C2_B_NICAM 0x0200
#define S_12_C3_B_NICAM 0x0421
#define S_12_C4_B_NICAM 0x0200

//GH_Stereo_A2 =======================
// 2.25Mhz
#define N_A1_C0_GHA2 0x038F
#define N_A1_C1_GHA2 0x063C
#define N_A1_C2_GHA2 0x0436
// 2.01Mhz
#define N_A2_C0_GHA2 0x039A
#define N_A2_C1_GHA2 0x063C
#define N_A2_C2_GHA2 0x042B
// 10.25Mhz
#define S_11_C0_GHA2 0x004D
#define S_11_C1_GHA2 0x063C
#define S_11_C2_GHA2 0x0200
#define S_11_C3_GHA2 0x07AE
#define S_11_C4_GHA2 0x0200
// 2.25Mhz
#define S_12_C0_GHA2 0x038F
#define S_12_C1_GHA2 0x063C
#define S_12_C2_GHA2 0x0200
#define S_12_C3_GHA2 0x0436
#define S_12_C4_GHA2 0x0200

//GH_Mono_NICAM ===================
// 2.25Mhz
#define N_A1_C0_GHMN 0x038F
#define N_A1_C1_GHMN 0x063C
#define N_A1_C2_GHMN 0x0436
// 1.9Mhz
#define N_A2_C0_GHMN 0x039E
#define N_A2_C1_GHMN 0x063C
#define N_A2_C2_GHMN 0x0427
// 10.25Mhz
#define S_11_C0_GHMN 0x004D
#define S_11_C1_GHMN 0x063C
#define S_11_C2_GHMN 0x0200
#define S_11_C3_GHMN 0x07AE
#define S_11_C4_GHMN 0x0200
// 2.25Mhz
#define S_12_C0_GHMN 0x038F
#define S_12_C1_GHMN 0x063C
#define S_12_C2_GHMN 0x0200
#define S_12_C3_GHMN 0x0436
#define S_12_C4_GHMN 0x0200

//DK1_Stero_A2 ======================
// 1.25Mhz
#define N_A1_C0_DK1A2 0x03B3
#define N_A1_C1_DK1A2 0x063C
#define N_A1_C2_DK1A2 0x0411
// 1.49Mhz
#define N_A2_C0_DK1A2 0x03AC
#define N_A2_C1_DK1A2 0x063C
#define N_A2_C2_DK1A2 0x0418
// 9.25Mhz
#define S_11_C0_DK1A2 0x00D7
#define S_11_C1_DK1A2 0x063C
#define S_11_C2_DK1A2 0x0200
#define S_11_C3_DK1A2 0x071B
#define S_11_C4_DK1A2 0x0200
// 1.25Mhz
#define S_12_C0_DK1A2 0x03B3
#define S_12_C1_DK1A2 0x063C
#define S_12_C2_DK1A2 0x0200
#define S_12_C3_DK1A2 0x0411
#define S_12_C4_DK1A2 0x0200

//DK2_Stero_A2 ======================
// 1.25Mhz
#define N_A1_C0_DK2A2 0x03B3
#define N_A1_C1_DK2A2 0x063C
#define N_A1_C2_DK2A2 0x0411
// 1.01Mhz
#define N_A2_C0_DK2A2 0x03B3
#define N_A2_C1_DK2A2 0x063C
#define N_A2_C2_DK2A2 0x040B
// 9.25Mhz
#define S_11_C0_DK2A2 0x00D7
#define S_11_C1_DK2A2 0x063C
#define S_11_C2_DK2A2 0x0200
#define S_11_C3_DK2A2 0x071B
#define S_11_C4_DK2A2 0x0200
// 1.25Mhz
#define S_12_C0_DK2A2 0x03B3
#define S_12_C1_DK2A2 0x063C
#define S_12_C2_DK2A2 0x0200
#define S_12_C3_DK2A2 0x0411
#define S_12_C4_DK2A2 0x0200

//DK3_Stero_A2=====================
// 1.25Mhz
#define N_A1_C0_DK3A2 0x03B3
#define N_A1_C1_DK3A2 0x063C
#define N_A1_C2_DK3A2 0x0411
// 2.01Mhz
#define N_A2_C0_DK3A2 0x039A
#define N_A2_C1_DK3A2 0x063C
#define N_A2_C2_DK3A2 0x042B
// 9.25Mhz
#define S_11_C0_DK3A2 0x00D7
#define S_11_C1_DK3A2 0x063C
#define S_11_C2_DK3A2 0x0200
#define S_11_C3_DK3A2 0x071B
#define S_11_C4_DK3A2 0x0200
// 1.25Mhz
#define S_12_C0_DK3A2 0x03B3
#define S_12_C1_DK3A2 0x063C
#define S_12_C2_DK3A2 0x0200
#define S_12_C3_DK3A2 0x0411
#define S_12_C4_DK3A2 0x0200

//DK_Mono_NICAM===================
// 1.25Mhz
#define N_A1_C0_DKMN 0x03B3
#define N_A1_C1_DKMN 0x063C
#define N_A1_C2_DKMN 0x0411
// 1.9Mhz
#define N_A2_C0_DKMN 0x039E
#define N_A2_C1_DKMN 0x063C
#define N_A2_C2_DKMN 0x0427
// 9.25Mhz
#define S_11_C0_DKMN 0x00D7
#define S_11_C1_DKMN 0x063C
#define S_11_C2_DKMN 0x0200
#define S_11_C3_DKMN 0x071B
#define S_11_C4_DKMN 0x0200
// 1.25Mhz
#define S_12_C0_DKMN 0x03B3
#define S_12_C1_DKMN 0x063C
#define S_12_C2_DKMN 0x0200
#define S_12_C3_DKMN 0x0411
#define S_12_C4_DKMN 0x0200

//Sound : I ===========================
// 1.75Mhz
#define N_A1_C0_I 0x03A4
#define N_A1_C1_I 0x063C
#define N_A1_C2_I 0x0421
// 1.2Mhz
#define N_A2_C0_I 0x03B4
#define N_A2_C1_I 0x063C
#define N_A2_C2_I 0x0410
// 9.75Mhz
#define S_11_C0_I 0x0092
#define S_11_C1_I 0x063C
#define S_11_C2_I 0x0200
#define S_11_C3_I 0x0764
#define S_11_C4_I 0x0200
// 1.75Mhz
#define S_12_C0_I 0x03A4
#define S_12_C1_I 0x063C
#define S_12_C2_I 0x0200
#define S_12_C3_I 0x0421
#define S_12_C4_I 0x0200
//Sound : MN=========================
// 2.25Mhz
#define N_A1_C0_NTSC 0x038F
#define N_A1_C1_NTSC 0x063C
#define N_A1_C2_NTSC 0x0436
// 2.03Mhz
#define N_A2_C0_NTSC 0x0399
#define N_A2_C1_NTSC 0x063C
#define N_A2_C2_NTSC 0x042C
// 8.25Mhz
#define S_11_C0_NTSC 0x015D
#define S_11_C1_NTSC 0x063C
#define S_11_C2_NTSC 0x0200
#define S_11_C3_NTSC 0x068D
#define S_11_C4_NTSC 0x0200
// 2.25Mhz
#define S_12_C0_NTSC 0x038F
#define S_12_C1_NTSC 0x063C
#define S_12_C2_NTSC 0x0200
#define S_12_C3_NTSC 0x0436
#define S_12_C4_NTSC 0x0200

//Sound : L ==========================
// 1.25Mhz
#define N_A1_C0_L 0x03B3
#define N_A1_C1_L 0x063C
#define N_A1_C2_L 0x0411
// 1.9Mhz
#define N_A2_C0_L 0x039E
#define N_A2_C1_L 0x063C
#define N_A2_C2_L 0x0427
// 9.25Mhz
#define S_11_C0_L 0x00D7
#define S_11_C1_L 0x063C
#define S_11_C2_L 0x0200
#define S_11_C3_L 0x071B
#define S_11_C4_L 0x0200
// 1.25Mhz
#define S_12_C0_L 0x03B3
#define S_12_C1_L 0x063C
#define S_12_C2_L 0x0200
#define S_12_C3_L 0x0411
#define S_12_C4_L 0x0200

//End of Table===================

#define IF_OUT_CFG_GAIN_LEVEL_DVB        6 //4
#define IF_OUT_CFG_GAIN_LEVEL_DVB_T2     6
#define IF_OUT_CFG_GAIN_LEVEL_DTMB       10 // 15 //for external ATBM886x DTMB demod must use 15
#define IF_OUT_CFG_GAIN_LEVEL_ISDB       13//6 //13 update by sj.rao for RF
#define IF_OUT_CFG_GAIN_LEVEL_ATSC       13
#define IF_OUT_CFG_GAIN_LEVEL_J83B       13
#define IF_OUT_CFG_GAIN_LEVEL_ATV_NORMAL 9 //8 //7
#define IF_OUT_CFG_GAIN_LEVEL_ATV_SCAN   9
#define IF_OUT_CFG_GAIN_LEVEL_ATV_SECAM  8


//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------

#if 0//kdrv
int m_bDeviceBusy;
#endif
MXL661_COMMAND_T apiCmd;
MXL661_SIGNAL_MODE_E curSignalMode;
int m_bInATVScanMode;
int m_bLastInATVScanMode;
int m_bNeedResetAGCMode;
int m_bInATVFinetuneMode;//Add to fix fineTune garbage(Mantis 0586666)
U32 u32GlobalATVRfFreqKHz;//Add to fix fineTune garbage(Mantis 0586666)
U32 u32chantuneFreq;   // record the frequency set by ChannelTune command
EN_TUNER_MODE m_eLastTunerMode;
U8 m_u8LastOtherMode;
static U8 SlaveID=0xC0;
static S8 XtalCap=MXL661_DEFAULTCAP;
static U8 tuner_i2c_channel_bus=1;
static bool isUseEWBSdemod=FALSE;
#if 0//kdrv
pthread_mutex_t m_mutex_SetTune;    // the mutex for SetTune
#endif
MXL661_AGC_ID_E AGC_Pin_Internal = MXL661_AGC1;//default : AGC1;
MXL661_AGC_ID_E AGC_Pin_External = MXL661_AGC2;//default : AGC2;


//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------

static s_Tuner_dev_info *p_dev = NULL;
static s_Tuner_dev_info dev[MAX_TUNER_DEV_NUM] =
{
  {E_TUNER_DTV_DVB_T_MODE, E_CLOSE, E_RF_CH_BAND_8MHz, 0,0,0,0,0,0,0,0},
  {E_TUNER_DTV_DVB_T_MODE, E_CLOSE, E_RF_CH_BAND_8MHz, 0,0,0,0,0,0,0,0},
};

static MXL661_XTAL_FREQ_E Xtal_Fre = MXL661_XTAL_16MHz;

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
MXL661_IF_FREQ_E IfFreqConversion(U16 u16IfFreqKHz)
{
    MXL661_IF_FREQ_E enIfFreq = MXL661_IF_5MHz;
    switch(u16IfFreqKHz)
    {
        case 3650:
            enIfFreq = MXL661_IF_3_65MHz;
            break;

        case 4000:
            enIfFreq = MXL661_IF_4MHz;
            break;

        case 4100:
            enIfFreq = MXL661_IF_4_1MHz;
            break;

        case 4150:
            enIfFreq = MXL661_IF_4_15MHz;
            break;

        case 4500:
            enIfFreq = MXL661_IF_4_5MHz;
            break;

        case 4570:
            enIfFreq = MXL661_IF_4_57MHz;
            break;

        case 5000:
            enIfFreq = MXL661_IF_5MHz;
            break;

        case 5380:
            enIfFreq = MXL661_IF_5_38MHz;
            break;

        case 6000:
            enIfFreq = MXL661_IF_6MHz;
            break;

        case 6280:
            enIfFreq = MXL661_IF_6_28MHz;
            break;

        case 7200:
            enIfFreq = MXL661_IF_7_2MHz;
            break;

        case 8250:
            enIfFreq = MXL661_IF_8_25MHz;
            break;

        case 35250:
            enIfFreq = MXL661_IF_35_25MHz;
            break;

        case 36000:
            enIfFreq = MXL661_IF_36MHz;
            break;

        case 36150:
            enIfFreq = MXL661_IF_36_15MHz;
            break;

        case 36650:
            enIfFreq = MXL661_IF_36_65MHz;
            break;

        case 44000:
            enIfFreq = MXL661_IF_44MHz;
            break;

        default:
            enIfFreq = MXL661_IF_5MHz;
            break;
    }
    return enIfFreq;
}

static void MxL_API_GetLockStatus(U8* RfLock, U8* RefLock)
{
  MXL661_COMMAND_T apiCmd;

  // Read back Tuner lock status
  apiCmd.commandId = MXL661_TUNER_LOCK_STATUS_REQ;
  apiCmd.MxLIf.cmdTunerLockReq.I2cSlaveAddr = SlaveID;

  if (MXL_TRUE == MxLWare661_API_GetTunerStatus(&apiCmd))
  {
    *RfLock = apiCmd.MxLIf.cmdTunerLockReq.RfSynLock;
    *RefLock = apiCmd.MxLIf.cmdTunerLockReq.RefSynLock;
  }
  return;
}
/*
static void MxL_API_ATV_Finetune(MXL_BOOL ScaleUp )
{
  MXL_STATUS status;
  MXL661_COMMAND_T apiCmd;

  apiCmd.commandId = MXL661_TUNER_ENABLE_FINE_TUNE_CFG;
  apiCmd.MxLIf.cmdEnableFineTuneCfg.I2cSlaveAddr = MXL661_I2C_ADDR;
  apiCmd.MxLIf.cmdEnableFineTuneCfg.EnableFineTune  = MXL_ENABLE;
  status = MxLWare661_API_ConfigTuner(&apiCmd);
  if(status != MXL_TRUE)
    printf(" Enable Fine Tune status = %d\n",status);

  apiCmd.commandId = MXL661_TUNER_FINE_TUNE_CFG;
  apiCmd.MxLIf.cmdFineTuneCfg.I2cSlaveAddr = MXL661_I2C_ADDR;
  apiCmd.MxLIf.cmdFineTuneCfg.ScaleUp = ScaleUp;
  status = MxLWare661_API_ConfigTuner(&apiCmd);
  if(status != MXL_TRUE)
    printf("Fine Tune operation status = %d\n",status);

  return;
}*/

int CheckFineTuneRange(U32 u32FreqKHz)
{
#if 0//kdrv
    if( (u32FreqKHz > (u32chantuneFreq - MAX_FINE_TUNE_STEP*FREQ_STEP)) && (u32FreqKHz < (u32chantuneFreq + MAX_FINE_TUNE_STEP*FREQ_STEP)) )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
#else
    if( ((u32FreqKHz*10) > (u32chantuneFreq*10 - MAX_FINE_TUNE_STEP*FREQ_STEP_10)) && ((u32FreqKHz*10) < (u32chantuneFreq*10 + MAX_FINE_TUNE_STEP*FREQ_STEP_10)) )
    {
        return TRUE;
    }
    else
    {
        TUNER_PRINT("%s return False\n", __FUNCTION__);
        return FALSE;
    }
#endif
}

int Tuner_AFC_RangeSet(MXL661_AFC_RANGE_E u32FreqOffsetKHz)
{
    MXL_STATUS status = MXL_FAILED;
    apiCmd.commandId = MXL661_TUNER_AFC_CFG;
    apiCmd.MxLIf.cmdAfcCfg.I2cSlaveAddr = SlaveID;
    apiCmd.MxLIf.cmdAfcCfg.AfcRangeInKHz = u32FreqOffsetKHz;
    status = MxLWare661_API_ConfigTuner(&apiCmd);
    return status;
}

int MDrv_TUNER_ConfigAGCMode(int minor, U32 eMode);

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
int MDrv_TUNER_Connect(int minor)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    p_dev=&(dev[minor]);

#if 0//kdrv
    if (m_bDeviceBusy==1)
        return FALSE;
#endif
    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(1);

    MXL_STATUS status = MXL_FAILED;

    //Step 1 : Soft Reset MxL661
    apiCmd.commandId = MXL661_DEV_SOFT_RESET_CFG;
    apiCmd.MxLIf.cmdResetCfg.I2cSlaveAddr=SlaveID;
    status = MxLWare661_API_ConfigDevice(&apiCmd);
    if (MXL_FAILED == status)
    {
        TUNER_PRINT("Error! MXL_DEV_SOFT_RESET_CFG\n");
    }

    //Step 2 : Overwrite Default
    apiCmd.commandId = MXL661_DEV_OVERWRITE_DEFAULT_CFG;
    apiCmd.MxLIf.cmdOverwriteDefault.SingleSupply_3_3V = MXL_ENABLE;
    apiCmd.MxLIf.cmdOverwriteDefault.I2cSlaveAddr=SlaveID;
    status = MxLWare661_API_ConfigDevice(&apiCmd);
    if (MXL_FAILED == status)
    {
        TUNER_PRINT("Error! MXL_DEV_OVERWRITE_DEFAULT_CFG\n");
    }

    //Step 3 : XTAL Setting
    apiCmd.commandId = MXL661_DEV_XTAL_SET_CFG;
    apiCmd.MxLIf.cmdXtalCfg.XtalFreqSel = Xtal_Fre;

    // Force XtalCap 0. Improve accuracy of XTAL from HW side...
    // Dont modify this value...Keep it XtalCap 0.
    // XtalCap = 0, ST18CM-2-E isnt working. (scan no channel).
    // XtalCap = 31 is fine.
    apiCmd.MxLIf.cmdXtalCfg.XtalCap = XtalCap;
    apiCmd.MxLIf.cmdXtalCfg.ClkOutEnable = MXL_DISABLE;
    apiCmd.MxLIf.cmdXtalCfg.ClkOutDiv = MXL_DISABLE;
    apiCmd.MxLIf.cmdXtalCfg.SingleSupply_3_3V = MXL_ENABLE;
    // Comments by MXL LD, 05/22:  if update to v2.1.10.9, need add this parameter
    apiCmd.MxLIf.cmdXtalCfg.XtalSharingMode = MXL_DISABLE;
    apiCmd.MxLIf.cmdXtalCfg.I2cSlaveAddr=SlaveID;
    status = MxLWare661_API_ConfigDevice(&apiCmd);
    if (MXL_FAILED == status)
    {
        TUNER_PRINT("Error! MXL_DEV_XTAL_SET_CFG\n");
    }

    // Comments by MXL LD, 05/22: Move tuner power up setting API calling from Channel tune to init process
    //Step 9 : Power up setting
    apiCmd.commandId = MXL661_TUNER_POWER_UP_CFG;
    apiCmd.MxLIf.cmdTunerPoweUpCfg.Enable = MXL_ENABLE;
    apiCmd.MxLIf.cmdTunerPoweUpCfg.I2cSlaveAddr = SlaveID;
    status = MxLWare661_API_ConfigTuner(&apiCmd);
    if (MXL_FAILED == status)
    {
        TUNER_PRINT("Error! MXL_TUNER_POWER_UP_CFG\n");
    }

    //Step 4 : IF Out setting
    apiCmd.commandId = MXL661_DEV_IF_OUT_CFG;
    apiCmd.MxLIf.cmdIfOutCfg.IFOutFreq = IfFreqConversion(DTV_TUNER_IF_FREQ_KHz);
    apiCmd.MxLIf.cmdIfOutCfg.ManualFreqSet= MXL_DISABLE;  //arvin;;m10
    //apiCmd.MxLIf.cmdIfOutCfg.ManualIFOutFreqInKHz = 5380;//4100;  //arvin;;
    apiCmd.MxLIf.cmdIfOutCfg.IFInversion = MXL_DISABLE;
    apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_DVB;
    apiCmd.MxLIf.cmdIfOutCfg.PathSel = MXL661_IF_PATH1;
    apiCmd.MxLIf.cmdIfOutCfg.I2cSlaveAddr = SlaveID;
    status = MxLWare661_API_ConfigDevice(&apiCmd);
    if (MXL_FAILED == status)
    {
        TUNER_PRINT("Error! MXL_DEV_IF_OUT_CFG\n");
    }

    //Step 5 : AGC Setting
    apiCmd.commandId = MXL661_TUNER_AGC_CFG;
    apiCmd.MxLIf.cmdAgcSetCfg.AgcSel = MXL661_AGC1;
    apiCmd.MxLIf.cmdAgcSetCfg.AgcType = MXL661_AGC_EXTERNAL;
    apiCmd.MxLIf.cmdAgcSetCfg.SetPoint = 66;
    apiCmd.MxLIf.cmdAgcSetCfg.AgcPolarityInverstion = MXL_DISABLE;
    apiCmd.MxLIf.cmdAgcSetCfg.I2cSlaveAddr = SlaveID;
    status = MxLWare661_API_ConfigTuner(&apiCmd);
    if (MXL_FAILED == status)
    {
        TUNER_PRINT("Error! MXL_TUNER_AGC_CFG\n");
    }



    //Step 6 :set AFC range
    status = Tuner_AFC_RangeSet(MXL661_AFC_DISABLE);
    if (MXL_FAILED == status)
    {
        TUNER_PRINT("Error! MXL_AFC_DISABLE\n");
    }
    curSignalMode = MXL661_SIGNAL_NA;

    m_eLastTunerMode = E_TUNER_INVALID;
    m_u8LastOtherMode = E_AUDIOSTANDARD_NOTSTANDARD_;
    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(0);

#if 0//kdrv
    m_bDeviceBusy=1;
#endif
    m_bInATVFinetuneMode = MXL_DISABLE; //Add to fix fineTune garbage(Mantis 0586666)
    u32GlobalATVRfFreqKHz = 0; //Add to fix fineTune garbage(Mantis 0586666)
    u32chantuneFreq = 0;
    return TRUE;
}

int MDrv_TUNER_Disconnect(int minor)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    p_dev=&(dev[minor]);
    return 0;
}

int MDrv_TUNER_ATV_SetTune(int minor, U32 u32FreqKHz, U32 eBand, U32 eMode, U8 otherMode)
{
    int retb = TRUE;
    MXL_STATUS status = MXL_FAILED;
    MXL661_IF_FREQ_E mxl_IF_Out_Freq;
    U32 u32IFOutFreqinKHz = 5000;
    int mxl_IF_Inversion;
    int fineTuneScaleUp; //Add to fix fineTune garbage(Mantis 0586666)

    U8  u8_lock1 = 0;
    U8  u8_lock2 = 0;

    //TUNER_PRINT(" is invoked\n", __FUNCTION__);
    TUNER_PRINT("%s minor: [%d] u32FreqKHz:[%d] eBand:[%d] eMode:[%d] otherMode:[%d]\n",__FUNCTION__,minor,u32FreqKHz,eBand,eMode,otherMode);
    //UNUSED(eBand);

    p_dev=&(dev[minor]);

    if (p_dev->e_status != E_WORKING)
    {
        TUNER_PRINT("[Error]%s,%d\n",__FILE__,__LINE__);
        return FALSE;
    }
    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(1);

    p_dev->e_std = E_TUNER_ATV_PAL_MODE;
    p_dev->u32_freq = u32FreqKHz;
    p_dev->u32_eband_bandwidth = eBand;
    p_dev->u32_eMode = eMode;
    p_dev->u8_otherMode = otherMode;

    //pthread_mutex_lock(&m_mutex_SetTune);
    if( (m_bInATVFinetuneMode == MXL_ENABLE) && (CheckFineTuneRange(u32FreqKHz) == TRUE) )  // Do FineTune
    {

#if 0 //AFC is already disable in Connect()
        // when do fine tune , need stop tuner AFC function
        Tuner_AFC_RangeSet(MXL661_AFC_DISABLE);
#endif

        // Determine the fine tune direction
        if (u32FreqKHz > u32GlobalATVRfFreqKHz)
        {
            fineTuneScaleUp = MXL_FINE_TUNE_STEP_UP;
            DBG_TUNER(TUNER_PRINT("\x1b[34m[Hao] %s MXL_FINE_TUNE_STEP_UP [%d] \x1b[0m\n",__FUNCTION__,__LINE__));
        }
        else
        {
            fineTuneScaleUp = MXL_FINE_TUNE_STEP_DOWN;
            DBG_TUNER(TUNER_PRINT("\x1b[34m[Hao] %s MXL_FINE_TUNE_STEP_DOWN [%d] \x1b[0m\n",__FUNCTION__,__LINE__));
        }

        // call MXL_TUNER_ENABLE_FINE_TUNE_CFG and MXL_TUNER_FINE_TUNE_CFG API functions
        apiCmd.commandId = MXL661_TUNER_ENABLE_FINE_TUNE_CFG;
        apiCmd.MxLIf.cmdEnableFineTuneCfg.EnableFineTune = MXL_ENABLE;
        //apiCmd.MxLIf.cmdEnableFineTuneCfg.I2cSlaveAddr = SlaveID;
        status = MxLWare661_API_ConfigTuner(&apiCmd);
        if (MXL_FAILED == status)
        {
            TUNER_PRINT("Error! MXL661_TUNER_ENABLE_FINE_TUNE_CFG\n");
        }

        apiCmd.commandId = MXL661_TUNER_FINE_TUNE_CFG;
        apiCmd.MxLIf.cmdFineTuneCfg.ScaleUp = fineTuneScaleUp;
        //apiCmd.MxLIf.cmdFineTuneCfg.I2cSlaveAddr = SlaveID;
        status = MxLWare661_API_ConfigTuner(&apiCmd);
        if (MXL_FAILED == status)
        {
            TUNER_PRINT("Error! MXL661_TUNER_FINE_TUNE_CFG\n");
        }
    }
    else    // Do SetAppMode and ChannelTune
    {

        if( (m_bInATVFinetuneMode == MXL_ENABLE) && (CheckFineTuneRange(u32FreqKHz) == FALSE) )
        {
#if 0//kdrv
            // if the frequency is out of fine-tune range in fine-tune mode, do ChannelTune command instead of FineTune command.
            if( u32FreqKHz >= (u32chantuneFreq + MAX_FINE_TUNE_STEP*FREQ_STEP) )
            {
                u32FreqKHz = u32chantuneFreq + MAX_FINE_TUNE_STEP*FREQ_STEP;
            }
            else    // u32FreqKHz <= (u32chantuneFreq - MAX_FINE_TUNE_STEP*FREQ_STEP)
            {
                u32FreqKHz = u32chantuneFreq - MAX_FINE_TUNE_STEP*FREQ_STEP;
            }
#else
            // if the frequency is out of fine-tune range in fine-tune mode, do ChannelTune command instead of FineTune command.
            if( (u32FreqKHz*10) >= (u32chantuneFreq*10 + MAX_FINE_TUNE_STEP*FREQ_STEP_10) )
            {
                u32FreqKHz = u32chantuneFreq*10 + MAX_FINE_TUNE_STEP*FREQ_STEP_10;
            }
            else    // u32FreqKHz <= (u32chantuneFreq - MAX_FINE_TUNE_STEP*FREQ_STEP)
            {
                u32FreqKHz = u32chantuneFreq*10 - MAX_FINE_TUNE_STEP*FREQ_STEP_10;
            }
            u32FreqKHz = u32FreqKHz/10;
            DBG_TUNER(TUNER_PRINT("Refine u32FreqKHz: %d\n",u32FreqKHz));
#endif
        }

        if((m_eLastTunerMode != eMode) || (m_u8LastOtherMode != otherMode) || (m_bLastInATVScanMode != m_bInATVScanMode))
        {
            m_eLastTunerMode = eMode;
            m_u8LastOtherMode = otherMode;
            m_bLastInATVScanMode = m_bInATVScanMode;

            MDrv_TUNER_ConfigAGCMode(minor,eMode);

            //Step 4 : IF Out setting
            if(eMode == E_TUNER_ATV_SECAM_L_PRIME_MODE)
            {
                mxl_IF_Out_Freq = MXL661_IF_5MHz;
                u32IFOutFreqinKHz = 5000;
                mxl_IF_Inversion = MXL_DISABLE;
                curSignalMode = MXL661_ANA_SECAM_L;
            }
            else
            {
                switch(otherMode)
                {
                    case E_AUDIOSTANDARD_BG_:
                    case E_AUDIOSTANDARD_BG_A2_:
                    case E_AUDIOSTANDARD_BG_NICAM_:
                         mxl_IF_Out_Freq = MXL661_IF_5MHz;
                         u32IFOutFreqinKHz = 5000;
                         mxl_IF_Inversion = MXL_ENABLE;
                         curSignalMode = MXL661_ANA_PAL_BG;
                         break;
                    case E_AUDIOSTANDARD_I_:
                         mxl_IF_Out_Freq = MXL661_IF_5MHz;
                         u32IFOutFreqinKHz = 5000;
                         mxl_IF_Inversion = MXL_ENABLE;
                         curSignalMode = MXL661_ANA_PAL_I;
                         break;
                    case E_AUDIOSTANDARD_DK_:
                    case E_AUDIOSTANDARD_DK1_A2_:
                    case E_AUDIOSTANDARD_DK2_A2_:
                    case E_AUDIOSTANDARD_DK3_A2_:
                    case E_AUDIOSTANDARD_DK_NICAM_:
                         mxl_IF_Out_Freq = MXL661_IF_5MHz;
                         u32IFOutFreqinKHz = 5000;
                         mxl_IF_Inversion = MXL_ENABLE;
                         curSignalMode = MXL661_ANA_PAL_D;
                         break;
                    case E_AUDIOSTANDARD_M_:
                    case E_AUDIOSTANDARD_M_BTSC_:
                    case E_AUDIOSTANDARD_M_A2_:
                    case E_AUDIOSTANDARD_M_EIA_J_:
                         mxl_IF_Out_Freq = MXL661_IF_5MHz;
                         u32IFOutFreqinKHz = 5000;
                         mxl_IF_Inversion = MXL_ENABLE;
                         curSignalMode = MXL661_ANA_NTSC_MODE;
                         break;
                    case E_AUDIOSTANDARD_L_:
                         mxl_IF_Out_Freq = MXL661_IF_5MHz;
                         u32IFOutFreqinKHz = 5000;
                         mxl_IF_Inversion = MXL_ENABLE;
                         curSignalMode = MXL661_ANA_SECAM_L;
                         break;
                    case E_AUDIOSTANDARD_NOTSTANDARD_:
                    case 0xFF:
                    default:
                         mxl_IF_Out_Freq = MXL661_IF_5MHz;
                         u32IFOutFreqinKHz = 5000;
                         mxl_IF_Inversion = MXL_ENABLE;
                         curSignalMode = MXL661_ANA_PAL_D;
                         break;
                }
            }

            apiCmd.commandId = MXL661_DEV_IF_OUT_CFG;
            apiCmd.MxLIf.cmdIfOutCfg.IFOutFreq = mxl_IF_Out_Freq;
            apiCmd.MxLIf.cmdIfOutCfg.ManualFreqSet= MXL_DISABLE;
            apiCmd.MxLIf.cmdIfOutCfg.IFInversion = mxl_IF_Inversion;
            if(m_bInATVScanMode == TRUE)
            {
                apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_ATV_SCAN;
            }
            else
            {
                if(eMode == E_TUNER_ATV_SECAM_L_PRIME_MODE || otherMode == E_AUDIOSTANDARD_L_)
                {
                    apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_ATV_SECAM;
                }
                else
                {
                    apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_ATV_NORMAL;
                }
            }
            apiCmd.MxLIf.cmdIfOutCfg.PathSel = MXL661_IF_PATH1;
            //apiCmd.MxLIf.cmdIfOutCfg.I2cSlaveAddr = SlaveID;
            status = MxLWare661_API_ConfigDevice(&apiCmd);
            if (MXL_FAILED == status)
            {
                TUNER_PRINT("Error! MXL_DEV_IF_OUT_CFG\n");
            }

            //Step 6 : Application Mode setting
            apiCmd.commandId = MXL661_TUNER_MODE_CFG;

            if(eMode == E_TUNER_ATV_SECAM_L_PRIME_MODE)
            {
                if(otherMode == E_AUDIOSTANDARD_I_)
                    apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_ANA_SECAM_I;
                else
                    apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_ANA_SECAM_L;
            }
            else if (eMode == E_TUNER_ATV_NTSC_MODE)
            {
                apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_ANA_NTSC_MODE;
                apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_6MHz;
            }
            else
            {
                switch(otherMode)
                {
                    case E_AUDIOSTANDARD_BG_:
                    case E_AUDIOSTANDARD_BG_A2_:
                    case E_AUDIOSTANDARD_BG_NICAM_:
                         apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_ANA_PAL_BG;
                         if(u32FreqKHz < 300000) //for PAL B
                         {
                             apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_7MHz;
                         }
                         else //for PAL G
                         {
                             apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
                         }
                         break;
                    case E_AUDIOSTANDARD_I_:
                         apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_ANA_PAL_I;
                         apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
                         break;
                    case E_AUDIOSTANDARD_DK_:
                    case E_AUDIOSTANDARD_DK1_A2_:
                    case E_AUDIOSTANDARD_DK2_A2_:
                    case E_AUDIOSTANDARD_DK3_A2_:
                    case E_AUDIOSTANDARD_DK_NICAM_:
                         apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_ANA_PAL_D;
                         apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
                         break;
                    case E_AUDIOSTANDARD_M_:
                    case E_AUDIOSTANDARD_M_BTSC_:
                    case E_AUDIOSTANDARD_M_A2_:
                    case E_AUDIOSTANDARD_M_EIA_J_:
                    case E_AUDIOSTANDARD_NOTSTANDARD_:
                         apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_ANA_NTSC_MODE;
                         apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_6MHz;
                         break;
                    case E_AUDIOSTANDARD_L_:
                         apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_ANA_SECAM_L;
                         apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
                         break;
                    case 0xFF:
                    default:
                         apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_ANA_PAL_D;
                         apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
                         break;
                }
            }

            apiCmd.MxLIf.cmdModeCfg.IFOutFreqinKHz = u32IFOutFreqinKHz;
            apiCmd.MxLIf.cmdModeCfg.XtalFreqSel = Xtal_Fre;
            if(m_bInATVScanMode == TRUE)
            {
                apiCmd.MxLIf.cmdModeCfg.IFOutGainLevel = IF_OUT_CFG_GAIN_LEVEL_ATV_SCAN;
            }
            else
            {
                if(eMode == E_TUNER_ATV_SECAM_L_PRIME_MODE || otherMode == E_AUDIOSTANDARD_L_)
                {
                    apiCmd.MxLIf.cmdModeCfg.IFOutGainLevel = IF_OUT_CFG_GAIN_LEVEL_ATV_SECAM;
                }
                else
                {
                    apiCmd.MxLIf.cmdModeCfg.IFOutGainLevel = IF_OUT_CFG_GAIN_LEVEL_ATV_NORMAL;
                }
            }
            //apiCmd.MxLIf.cmdModeCfg.I2cSlaveAddr = SlaveID;
            status = MxLWare661_API_ConfigTuner(&apiCmd);
            if (MXL_FAILED == status)
            {
                TUNER_PRINT("Error! MXL_TUNER_MODE_CFG\n");
            }
        }

        //fine-tune must be disabled right before doing channel tune, otherwise fine-tune will not works. 20121204
        apiCmd.commandId = MXL661_TUNER_ENABLE_FINE_TUNE_CFG;
//        apiCmd.MxLIf.cmdEnableFineTuneCfg.I2cSlaveAddr = SlaveID;
        apiCmd.MxLIf.cmdEnableFineTuneCfg.EnableFineTune  = MXL_DISABLE;
        status = MxLWare661_API_ConfigTuner(&apiCmd);

        //Step 7 : Channel frequency & bandwidth setting
        apiCmd.commandId = MXL661_TUNER_CHAN_TUNE_CFG;
        apiCmd.MxLIf.cmdChanTuneCfg.TuneType = MXL661_VIEW_MODE;
        apiCmd.MxLIf.cmdChanTuneCfg.XtalFreqSel = Xtal_Fre;
        apiCmd.MxLIf.cmdChanTuneCfg.IFOutFreqinKHz = u32IFOutFreqinKHz;

        if(eMode == E_TUNER_ATV_SECAM_L_PRIME_MODE)
        {
            apiCmd.MxLIf.cmdChanTuneCfg.SignalMode = MXL661_ANA_SECAM_L;
            apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
            apiCmd.MxLIf.cmdChanTuneCfg.FreqInHz =(u32FreqKHz-2750)*1000L;//Freq*100*1000;
        }
        else
        {
            switch(otherMode)
            {
                case E_AUDIOSTANDARD_BG_:
                case E_AUDIOSTANDARD_BG_A2_:
                case E_AUDIOSTANDARD_BG_NICAM_:
                     apiCmd.MxLIf.cmdChanTuneCfg.SignalMode = MXL661_ANA_PAL_BG;
                     if(u32FreqKHz < 300000) //for PAL B
                     {
                         apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_7MHz;
                         apiCmd.MxLIf.cmdChanTuneCfg.FreqInHz =(u32FreqKHz+2250)*1000L;//Freq*100*1000;
                     }
                     else
                     {
                         apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
                         apiCmd.MxLIf.cmdChanTuneCfg.FreqInHz =(u32FreqKHz+2750)*1000L;//Freq*100*1000;
                     }
                     break;

                case E_AUDIOSTANDARD_I_:
                     apiCmd.MxLIf.cmdChanTuneCfg.SignalMode = MXL661_ANA_PAL_I;
                     apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
                     apiCmd.MxLIf.cmdChanTuneCfg.FreqInHz =(u32FreqKHz+2750)*1000L;//Freq*100*1000;
                     break;

                case E_AUDIOSTANDARD_DK_:
                case E_AUDIOSTANDARD_DK1_A2_:
                case E_AUDIOSTANDARD_DK2_A2_:
                case E_AUDIOSTANDARD_DK3_A2_:
                case E_AUDIOSTANDARD_DK_NICAM_:
                     apiCmd.MxLIf.cmdChanTuneCfg.SignalMode = MXL661_ANA_PAL_D;
                     apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
                     apiCmd.MxLIf.cmdChanTuneCfg.FreqInHz =(u32FreqKHz+2750)*1000L;//Freq*100*1000;
                     break;

                case E_AUDIOSTANDARD_M_:
                case E_AUDIOSTANDARD_M_BTSC_:
                case E_AUDIOSTANDARD_M_A2_:
                case E_AUDIOSTANDARD_M_EIA_J_:
                     apiCmd.MxLIf.cmdChanTuneCfg.SignalMode = MXL661_ANA_NTSC_MODE;
                     apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_6MHz;
                     apiCmd.MxLIf.cmdChanTuneCfg.FreqInHz =(u32FreqKHz+1750)*1000L;//Freq*100*1000;
                     break;
                case E_AUDIOSTANDARD_L_:
                     apiCmd.MxLIf.cmdChanTuneCfg.SignalMode = MXL661_ANA_SECAM_L;
                     apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
                     apiCmd.MxLIf.cmdChanTuneCfg.FreqInHz =(u32FreqKHz+2750)*1000L;//Freq*100*1000;
                     break;
                default:
                     apiCmd.MxLIf.cmdChanTuneCfg.SignalMode = MXL661_ANA_PAL_D;
                     apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
                     apiCmd.MxLIf.cmdChanTuneCfg.FreqInHz =(u32FreqKHz+2750)*1000L;//Freq*100*1000;
                     break;
            }
        }
//        apiCmd.MxLIf.cmdChanTuneCfg.I2cSlaveAddr = SlaveID;
        status = MxLWare661_API_ConfigTuner(&apiCmd);
        if (MXL_FAILED == status)
        {
            TUNER_PRINT("Error! MXL_TUNER_CHAN_TUNE_CFG\n");
        }

#if 1 //Although AFC is already disabled in Connect(), disable fine-tune command(above) will enable AFC.
        //Step 8 : Disable AFC in tuner if scan
        //Disable mxl661 AFT always due to unexpected behavior.
            Tuner_AFC_RangeSet(MXL661_AFC_DISABLE);
#endif

        // Comments by MXL LD, 05/22:  remove MXL_TUNER_POWER_UP_CFG API calling from tuner channel tune to init phase
#if 0
        //Step 9 : Power up setting
        apiCmd.commandId = MXL_TUNER_POWER_UP_CFG;
        apiCmd.MxLIf.cmdTunerPoweUpCfg.Enable = MXL_ENABLE;
        status = MxLWare661_API_ConfigTuner(&apiCmd);
        if (MXL_FAILED == status)
        {
            TUNER_PRINT("Error! MXL_TUNER_POWER_UP_CFG\n");
        }
#else
        // Comments by MXL LD, 05/22: replace MXL_TUNER_POWER_UP_CFG calling with MXL_TUNER_START_TUNE_CFG API
        //   First do disable tune operation
        //Step 10 : Sequencer setting: disable tune
        apiCmd.commandId = MXL661_TUNER_START_TUNE_CFG;
        apiCmd.MxLIf.cmdStartTuneCfg.StartTune = MXL_DISABLE;
//        apiCmd.MxLIf.cmdStartTuneCfg.I2cSlaveAddr = SlaveID;
        status = MxLWare661_API_ConfigTuner(&apiCmd);
        if (MXL_FAILED == status)
        {
            TUNER_PRINT("Error! MXL_TUNER_START_TUNE_CFG\n");
        }
#endif
        //Step 10 : Sequencer setting : enable tune
        apiCmd.commandId = MXL661_TUNER_START_TUNE_CFG;
        apiCmd.MxLIf.cmdStartTuneCfg.StartTune = MXL_ENABLE;
//        apiCmd.MxLIf.cmdStartTuneCfg.I2cSlaveAddr = SlaveID;
        status = MxLWare661_API_ConfigTuner(&apiCmd);
        if (MXL_FAILED == status)
        {
            TUNER_PRINT("Error! MXL_TUNER_START_TUNE_CFG\n");
        }

        // Save the frequency set by ChannelTune command
        u32chantuneFreq = u32FreqKHz;
    }
    // Save current RF frequency point
    u32GlobalATVRfFreqKHz = u32FreqKHz;

    MxL_API_GetLockStatus(&u8_lock1,&u8_lock2);
    if ((u8_lock1+u8_lock2) == 2)
        TUNER_PRINT("MxL Tuner Lock....\n");
    else
        TUNER_PRINT("MxL Tuner unLock....\n");

    //pthread_mutex_unlock(&m_mutex_SetTune);
    // wait for signal stable
    if(eMode == E_TUNER_ATV_SECAM_L_PRIME_MODE)
    {
        MxLWare661_OEM_Sleep(SECAM_L_PRIME_TUNER_PLL_STABLE_TIME);
    }
    else
    {
        MxLWare661_OEM_Sleep(TUNER_PLL_STABLE_TIME); // added for ATV randomly missing channel.
    }
    
    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(0);

    return retb;
}

int MDrv_TUNER_DVBS_SetTune(int minor, U16 u16CenterFreqMHz, U32 u32SymbolRateKs)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    return 0;
}

int MDrv_TUNER_DTV_SetTune(int minor, U32 freq, U32 eBandWidth, U32 eMode)
{
    //DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));

    MXL_STATUS status = MXL_FAILED;
    U8  u8_lock1 = 0;
    U8  u8_lock2 = 0;

//    if (eMode == E_TUNER_DTV_DVB_T2_MODE)
//        eMode = E_TUNER_DTV_DVB_T_MODE;
    TUNER_PRINT("%s minor: [%d] freq:[%d] eBandWidth:[%d] eMode:[%d]\n",__FUNCTION__,minor,freq,eBandWidth,eMode);
    p_dev=&(dev[minor]);

    if (p_dev->e_status != E_WORKING)
    {
        TUNER_PRINT("[Error]%s,%d\n",__FILE__,__LINE__);
        return FALSE;
    }
        
    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(1);
    p_dev->e_std = E_TUNER_DTV_DVB_T_MODE;
    p_dev->u32_freq = freq;
    p_dev->u32_eband_bandwidth = eBandWidth;
    p_dev->u32_eMode = eMode;

    if(m_eLastTunerMode != eMode)
    {
        m_eLastTunerMode = eMode;

        MDrv_TUNER_ConfigAGCMode(minor,eMode);

        //Step 6 : Application Mode setting
        apiCmd.commandId = MXL661_TUNER_MODE_CFG;
        apiCmd.MxLIf.cmdModeCfg.IFOutFreqinKHz = TUNER_DTV_IF_FREQ_KHz;
        if(eMode == E_TUNER_DTV_DVB_T_MODE)
        {
            apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_DIG_DVB_T_DTMB;
            apiCmd.MxLIf.cmdModeCfg.IFOutGainLevel = IF_OUT_CFG_GAIN_LEVEL_DVB;
        }
        else if(eMode == E_TUNER_DTV_DVB_T2_MODE)
        {
            apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_DIG_DVB_T_DTMB;
            apiCmd.MxLIf.cmdModeCfg.IFOutGainLevel = IF_OUT_CFG_GAIN_LEVEL_DVB_T2;
        }
        else if(eMode == E_TUNER_DTV_DTMB_MODE)
        {
            apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_DIG_DVB_T_DTMB;
            apiCmd.MxLIf.cmdModeCfg.IFOutGainLevel = IF_OUT_CFG_GAIN_LEVEL_DTMB;
            apiCmd.MxLIf.cmdModeCfg.IFOutFreqinKHz = DTMB_TUNER_IF_FREQ_KHz;
        }
        else if(eMode == E_TUNER_DTV_ISDB_MODE)
        {
            apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_DIG_ISDBT_ATSC;
            apiCmd.MxLIf.cmdModeCfg.IFOutGainLevel = IF_OUT_CFG_GAIN_LEVEL_ISDB;
        }
        else if(eMode == E_TUNER_DTV_ATSC_MODE)
        {
            apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_DIG_ISDBT_ATSC;
            apiCmd.MxLIf.cmdModeCfg.IFOutGainLevel = IF_OUT_CFG_GAIN_LEVEL_ATSC;
            DBG_TUNER(TUNER_PRINT("\n[Mstar Demod Debug] Curr Tuner Mode is E_TUNER_DTV_ATSC_MODE"));
        }
        else if(eMode == E_TUNER_DTV_J83B_MODE)
        {
            apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_DIG_J83B;
            apiCmd.MxLIf.cmdModeCfg.IFOutGainLevel = IF_OUT_CFG_GAIN_LEVEL_ATSC;
            DBG_TUNER(TUNER_PRINT("\n[Mstar Demod Debug] Curr Tuner Mode is J83B"));
        }
        else
        {
            apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_DIG_DVB_C;
            apiCmd.MxLIf.cmdModeCfg.IFOutGainLevel = IF_OUT_CFG_GAIN_LEVEL_DVB;
        }


        apiCmd.MxLIf.cmdModeCfg.XtalFreqSel = Xtal_Fre;
//        apiCmd.MxLIf.cmdModeCfg.I2cSlaveAddr = SlaveID;
        status = MxLWare661_API_ConfigTuner(&apiCmd);
        if (MXL_FAILED == status)
        {
            TUNER_PRINT("Error! MXL_TUNER_MODE_CFG\n");
        }

    }
    if(eMode == E_TUNER_DTV_J83B_MODE || eMode == E_TUNER_DTV_ATSC_MODE)
    {
      MxLWare661_OEM_WriteRegister(0xC0, 0x5B,0x10);
      MxLWare661_OEM_WriteRegister(0xC0, 0x5C,0xB1);
      MxLWare661_OEM_WriteRegister(0xC0, 0xB5,0x65);
      MxLWare661_OEM_WriteRegister(0xC0, 0xC2,0xA9);
      MxLWare661_OEM_WriteRegister(0xC0, 0xB4,0x1C);
    }
    //Step 7 : Channel frequency & bandwidth setting
    apiCmd.commandId = MXL661_TUNER_CHAN_TUNE_CFG;
    apiCmd.MxLIf.cmdChanTuneCfg.TuneType = MXL661_VIEW_MODE;


    if ((eMode == E_TUNER_DTV_DVB_T_MODE) ||
        (eMode == E_TUNER_DTV_DVB_T2_MODE) ||
        (eMode == E_TUNER_DTV_DTMB_MODE) ||
        (eMode == E_TUNER_DTV_ISDB_MODE) ||
        (eMode == E_TUNER_DTV_ATSC_MODE))
    {
        if(eBandWidth == E_RF_CH_BAND_6MHz)
        {
            apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_DIG_TERR_BW_6MHz;
        }
        else if(eBandWidth == E_RF_CH_BAND_7MHz)
        {
            apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_DIG_TERR_BW_7MHz;
        }
        else
        {
            apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_DIG_TERR_BW_8MHz;
        }
    }
    else
    {
        if(eBandWidth == E_RF_CH_BAND_6MHz)
        {
            apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_6MHz;
        }
        else if(eBandWidth == E_RF_CH_BAND_7MHz)
        {
            apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_7MHz;
        }
        else
        {
            apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
        }
    }

    if (eMode == E_TUNER_DTV_ATSC_MODE || eMode == E_TUNER_DTV_J83B_MODE)
        apiCmd.MxLIf.cmdChanTuneCfg.FreqInHz =  freq*1L;
    else
        apiCmd.MxLIf.cmdChanTuneCfg.FreqInHz =  freq*1000L;

    apiCmd.MxLIf.cmdChanTuneCfg.IFOutFreqinKHz = TUNER_DTV_IF_FREQ_KHz;
    if(eMode == E_TUNER_DTV_DVB_T_MODE)
    {
        apiCmd.MxLIf.cmdChanTuneCfg.SignalMode =  MXL661_DIG_DVB_T_DTMB;
    }
    else if(eMode == E_TUNER_DTV_DVB_T2_MODE)
    {
        apiCmd.MxLIf.cmdChanTuneCfg.SignalMode =  MXL661_DIG_DVB_T_DTMB;
    }
    else if(eMode == E_TUNER_DTV_DTMB_MODE)
    {
        apiCmd.MxLIf.cmdChanTuneCfg.SignalMode =  MXL661_DIG_DVB_T_DTMB;
        apiCmd.MxLIf.cmdChanTuneCfg.IFOutFreqinKHz = DTMB_TUNER_IF_FREQ_KHz;
    }
    else if(eMode == E_TUNER_DTV_ISDB_MODE)
    {
        apiCmd.MxLIf.cmdChanTuneCfg.SignalMode =  MXL661_DIG_ISDBT_ATSC;
    }
    else if(eMode == E_TUNER_DTV_ATSC_MODE)
    {
        apiCmd.MxLIf.cmdChanTuneCfg.SignalMode =  MXL661_DIG_ISDBT_ATSC;
    }
    else if(eMode == E_TUNER_DTV_J83B_MODE)
    {
        apiCmd.MxLIf.cmdChanTuneCfg.SignalMode =  MXL661_DIG_J83B;
    }
    else
    {
        apiCmd.MxLIf.cmdChanTuneCfg.SignalMode =  MXL661_DIG_DVB_C;
    }

    apiCmd.MxLIf.cmdChanTuneCfg.XtalFreqSel = Xtal_Fre;
//    apiCmd.MxLIf.cmdChanTuneCfg.I2cSlaveAddr = SlaveID;
    status = MxLWare661_API_ConfigTuner(&apiCmd);
    if (MXL_FAILED == status)
    {
        TUNER_PRINT("Error! MXL_TUNER_CHAN_TUNE_CFG\n");
    }

    if(eMode == E_TUNER_DTV_J83B_MODE || eMode == E_TUNER_DTV_ATSC_MODE)
    {
      MxLWare661_OEM_WriteRegister(0xC0, 0x5B,0x10);
      MxLWare661_OEM_WriteRegister(0xC0, 0x5C,0xB1);
      MxLWare661_OEM_WriteRegister(0xC0, 0xB5,0x65);
      MxLWare661_OEM_WriteRegister(0xC0, 0xC2,0xA9);
      MxLWare661_OEM_WriteRegister(0xC0, 0xB4,0x1C);
    }	
    // Comments by MXL LD, 05/22:  call MXL_TUNER_START_TUNE_CFG instead of
    //     call MXL_TUNER_POWER_UP_CFG
#if 0
    //Step 8 : Power up setting
    apiCmd.commandId = MXL_TUNER_POWER_UP_CFG;
    apiCmd.MxLIf.cmdTunerPoweUpCfg.Enable = MXL_ENABLE;
    status = MxLWare661_API_ConfigTuner(&apiCmd);
    if (MXL_FAILED == status)
    {
        printf("Error! MXL_TUNER_POWER_UP_CFG\n");
    }
#else
    //Step 9 : Sequencer setting
    apiCmd.commandId = MXL661_TUNER_START_TUNE_CFG;
    apiCmd.MxLIf.cmdStartTuneCfg.StartTune = MXL_DISABLE;
//    apiCmd.MxLIf.cmdStartTuneCfg.I2cSlaveAddr = SlaveID;
    status = MxLWare661_API_ConfigTuner(&apiCmd);
    if (MXL_FAILED == status)
    {
        TUNER_PRINT("Error! MXL_TUNER_START_TUNE_CFG\n");
    }
#endif

    //Step 9 : Sequencer setting
    apiCmd.commandId = MXL661_TUNER_START_TUNE_CFG;
    apiCmd.MxLIf.cmdStartTuneCfg.StartTune = MXL_ENABLE;
//    apiCmd.MxLIf.cmdStartTuneCfg.I2cSlaveAddr = SlaveID;
    status = MxLWare661_API_ConfigTuner(&apiCmd);
    if (MXL_FAILED == status)
    {
        TUNER_PRINT("Error! MXL_TUNER_START_TUNE_CFG\n");
    }

    MxL_API_GetLockStatus(&u8_lock1,&u8_lock2);
    if ((u8_lock1+u8_lock2) == 2)
        TUNER_PRINT("MxL Tuner Lock....\n");
    else
        TUNER_PRINT("MxL Tuner unLock....\n");

    // Wait 30ms
    MxLWare661_OEM_Sleep(TUNER_PLL_STABLE_TIME);

    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(0);
    return TRUE;

}

int MDrv_TUNER_ExtendCommand(int minor, U8 u8SubCmd, U32 u32Param1, U32 u32Param2, void* pvoidParam3)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked u8SubCmd [%d]\n", __FUNCTION__,u8SubCmd));

    p_dev=&(dev[minor]);

    switch(u8SubCmd)
    {
        case E_TUNER_SUBCMD_GET_FREQ_STEP:
        {
            EN_FREQ_STEP *eFreqStep = (EN_FREQ_STEP*)pvoidParam3;
            *eFreqStep = TN_FREQ_STEP;
        }
        break;

        case E_TUNER_SUBCMD_GET_IF_FREQ:
        {
            U16 *u16IFFreq = (U16 *)pvoidParam3;
            *u16IFFreq = ATV_TUNER_IF_FREQ_KHz;
        }
        break;

        case E_TUNER_SUBCMD_GET_L_PRIME_IF_FREQ:
        {
            U16 *u16IFFreq = (U16 *)pvoidParam3;
            *u16IFFreq = TUNER_L_PRIME_IF_FREQ_KHz;
        }
        break;

        case E_TUNER_SUBCMD_GET_VHF_LOWMIN_FREQ:
        {
            U32 *u32Freq = (U32 *)pvoidParam3;
            *u32Freq = TUNER_VHF_LOWMIN_FREQ;
        }
        break;

        case E_TUNER_SUBCMD_GET_VHF_LOWMAX_FREQ:
        {
            U32 *u32Freq = (U32 *)pvoidParam3;
            *u32Freq = TUNER_VHF_LOWMAX_FREQ;
        }
        break;

        case E_TUNER_SUBCMD_GET_VHF_HIGHMIN_FREQ:
        {
            U32 *u32Freq = (U32 *)pvoidParam3;
            *u32Freq = TUNER_VHF_HIGHMIN_FREQ;
        }
        break;

        case E_TUNER_SUBCMD_GET_VHF_HIGHMAX_FREQ:
        {
            U32 *u32Freq = (U32 *)pvoidParam3;
            *u32Freq = TUNER_VHF_HIGHMAX_FREQ;
        }
        break;

        case E_TUNER_SUBCMD_GET_UHF_MIN_FREQ:
        {
            U32 *u32Freq = (U32 *)pvoidParam3;
            *u32Freq = TUNER_UHF_MIN_FREQ;
        }
        break;

        case E_TUNER_SUBCMD_GET_UHF_MAX_FREQ:  //arvin;;
        {
            U32 *u32Freq = (U32 *)pvoidParam3;
            *u32Freq = TUNER_UHF_MAX_FREQ;
        }
        break;

        case E_TUNER_SUBCMD_GET_VIF_TUNER_TYPE:
        {
            U8 *u8TunerType = (U8 *)pvoidParam3;
            *u8TunerType = VIF_TUNER_TYPE;
        }
        break;

        case E_TUNER_SUBCMD_GET_VIF_PARA:
        {
            stVIFInitialIn *stVIFPara = (stVIFInitialIn *)pvoidParam3;
            stVIFPara->VifCrRate_B = VIF_CR_RATE_B;
            stVIFPara->VifCrInvert_B = VIF_CR_INVERT_B;
            stVIFPara->VifCrRate_GH = VIF_CR_RATE_GH;
            stVIFPara->VifCrInvert_GH =VIF_CR_INVERT_GH;
            stVIFPara->VifCrRate_DK = VIF_CR_RATE_DK;
            stVIFPara->VifCrInvert_DK = VIF_CR_INVERT_DK;
            stVIFPara->VifCrRate_I = VIF_CR_RATE_I;
            stVIFPara->VifCrInvert_I = VIF_CR_INVERT_I;
            stVIFPara->VifCrRate_L = VIF_CR_RATE_L;
            stVIFPara->VifCrInvert_L = VIF_CR_INVERT_L;
            stVIFPara->VifCrRate_LL = VIF_CR_RATE_LL;
            stVIFPara->VifCrInvert_LL = VIF_CR_INVERT_LL;
            stVIFPara->VifCrRate_MN = VIF_CR_RATE_MN;
            stVIFPara->VifCrInvert_MN = VIF_CR_INVERT_MN;

        }
        break;

        case E_TUNER_SUBCMD_GET_VIF_NOTCH_SOSFILTER:
        {
            U16 *u16NotchArray = (U16 *)pvoidParam3;
            U16 *u16SosArray = &u16NotchArray[u32Param2];

            switch((EN_VIF_SOUND_SYSTEM)u32Param1)
            {
                case E_VIF_SOUND_B_STEREO_A2:
                {
                     u16NotchArray[0] = N_A1_C0_B_A2;
                     u16NotchArray[1] = N_A1_C1_B_A2;
                     u16NotchArray[2] = N_A1_C2_B_A2;

                     u16NotchArray[3] = N_A2_C0_B_A2;
                     u16NotchArray[4] = N_A2_C1_B_A2;
                     u16NotchArray[5] = N_A2_C2_B_A2;

                     u16SosArray[0] = S_11_C0_B_A2;
                     u16SosArray[1] = S_11_C1_B_A2;
                     u16SosArray[2] = S_11_C2_B_A2;
                     u16SosArray[3] = S_11_C3_B_A2;
                     u16SosArray[4] = S_11_C4_B_A2;

                     u16SosArray[5] = S_12_C0_B_A2;
                     u16SosArray[6] = S_12_C1_B_A2;
                     u16SosArray[7] = S_12_C2_B_A2;
                     u16SosArray[8] = S_12_C3_B_A2;
                     u16SosArray[9] = S_12_C4_B_A2;
                }
                break;

                case E_VIF_SOUND_B_MONO_NICAM:
                {
                     u16NotchArray[0] = N_A1_C0_B_NICAM;
                     u16NotchArray[1] = N_A1_C1_B_NICAM;
                     u16NotchArray[2] = N_A1_C2_B_NICAM;

                     u16NotchArray[3] = N_A2_C0_B_NICAM;
                     u16NotchArray[4] = N_A2_C1_B_NICAM;
                     u16NotchArray[5] = N_A2_C2_B_NICAM;

                     u16SosArray[0] = S_11_C0_B_NICAM;
                     u16SosArray[1] = S_11_C1_B_NICAM;
                     u16SosArray[2] = S_11_C2_B_NICAM;
                     u16SosArray[3] = S_11_C3_B_NICAM;
                     u16SosArray[4] = S_11_C4_B_NICAM;

                     u16SosArray[5] = S_12_C0_B_NICAM;
                     u16SosArray[6] = S_12_C1_B_NICAM;
                     u16SosArray[7] = S_12_C2_B_NICAM;
                     u16SosArray[8] = S_12_C3_B_NICAM;
                     u16SosArray[9] = S_12_C4_B_NICAM;
                }
                break;

                case E_VIF_SOUND_GH_STEREO_A2:
                {
                     u16NotchArray[0] = N_A1_C0_GHA2;
                     u16NotchArray[1] = N_A1_C1_GHA2;
                     u16NotchArray[2] = N_A1_C2_GHA2;

                     u16NotchArray[3] = N_A2_C0_GHA2;
                     u16NotchArray[4] = N_A2_C1_GHA2;
                     u16NotchArray[5] = N_A2_C2_GHA2;

                     u16SosArray[0] = S_11_C0_GHA2;
                     u16SosArray[1] = S_11_C1_GHA2;
                     u16SosArray[2] = S_11_C2_GHA2;
                     u16SosArray[3] = S_11_C3_GHA2;
                     u16SosArray[4] = S_11_C4_GHA2;

                     u16SosArray[5] = S_12_C0_GHA2;
                     u16SosArray[6] = S_12_C1_GHA2;
                     u16SosArray[7] = S_12_C2_GHA2;
                     u16SosArray[8] = S_12_C3_GHA2;
                     u16SosArray[9] = S_12_C4_GHA2;
                }
                break;

                case E_VIF_SOUND_GH_MONO_NICAM:
                {
                     u16NotchArray[0] = N_A1_C0_GHMN;
                     u16NotchArray[1] = N_A1_C1_GHMN;
                     u16NotchArray[2] = N_A1_C2_GHMN;

                     u16NotchArray[3] = N_A2_C0_GHMN;
                     u16NotchArray[4] = N_A2_C1_GHMN;
                     u16NotchArray[5] = N_A2_C2_GHMN;

                     u16SosArray[0] = S_11_C0_GHMN;
                     u16SosArray[1] = S_11_C1_GHMN;
                     u16SosArray[2] = S_11_C2_GHMN;
                     u16SosArray[3] = S_11_C3_GHMN;
                     u16SosArray[4] = S_11_C4_GHMN;

                     u16SosArray[5] = S_12_C0_GHMN;
                     u16SosArray[6] = S_12_C1_GHMN;
                     u16SosArray[7] = S_12_C2_GHMN;
                     u16SosArray[8] = S_12_C3_GHMN;
                     u16SosArray[9] = S_12_C4_GHMN;
                }
                break;

                case E_VIF_SOUND_I:
                {
                     u16NotchArray[0] = N_A1_C0_I;
                     u16NotchArray[1] = N_A1_C1_I;
                     u16NotchArray[2] = N_A1_C2_I;

                     u16NotchArray[3] = N_A2_C0_I;
                     u16NotchArray[4] = N_A2_C1_I;
                     u16NotchArray[5] = N_A2_C2_I;

                     u16SosArray[0] = S_11_C0_I;
                     u16SosArray[1] = S_11_C1_I;
                     u16SosArray[2] = S_11_C2_I;
                     u16SosArray[3] = S_11_C3_I;
                     u16SosArray[4] = S_11_C4_I;

                     u16SosArray[5] = S_12_C0_I;
                     u16SosArray[6] = S_12_C1_I;
                     u16SosArray[7] = S_12_C2_I;
                     u16SosArray[8] = S_12_C3_I;
                     u16SosArray[9] = S_12_C4_I;
                }
                break;

                case E_VIF_SOUND_DK1_STEREO_A2:
                {
                     u16NotchArray[0] = N_A1_C0_DK1A2;
                     u16NotchArray[1] = N_A1_C1_DK1A2;
                     u16NotchArray[2] = N_A1_C2_DK1A2;

                     u16NotchArray[3] = N_A2_C0_DK1A2;
                     u16NotchArray[4] = N_A2_C1_DK1A2;
                     u16NotchArray[5] = N_A2_C2_DK1A2;

                     u16SosArray[0] = S_11_C0_DK1A2;
                     u16SosArray[1] = S_11_C1_DK1A2;
                     u16SosArray[2] = S_11_C2_DK1A2;
                     u16SosArray[3] = S_11_C3_DK1A2;
                     u16SosArray[4] = S_11_C4_DK1A2;

                     u16SosArray[5] = S_12_C0_DK1A2;
                     u16SosArray[6] = S_12_C1_DK1A2;
                     u16SosArray[7] = S_12_C2_DK1A2;
                     u16SosArray[8] = S_12_C3_DK1A2;
                     u16SosArray[9] = S_12_C4_DK1A2;
                }
                break;

                case E_VIF_SOUND_DK2_STEREO_A2:
                {
                     u16NotchArray[0] = N_A1_C0_DK2A2;
                     u16NotchArray[1] = N_A1_C1_DK2A2;
                     u16NotchArray[2] = N_A1_C2_DK2A2;

                     u16NotchArray[3] = N_A2_C0_DK2A2;
                     u16NotchArray[4] = N_A2_C1_DK2A2;
                     u16NotchArray[5] = N_A2_C2_DK2A2;

                     u16SosArray[0] = S_11_C0_DK2A2;
                     u16SosArray[1] = S_11_C1_DK2A2;
                     u16SosArray[2] = S_11_C2_DK2A2;
                     u16SosArray[3] = S_11_C3_DK2A2;
                     u16SosArray[4] = S_11_C4_DK2A2;

                     u16SosArray[5] = S_12_C0_DK2A2;
                     u16SosArray[6] = S_12_C1_DK2A2;
                     u16SosArray[7] = S_12_C2_DK2A2;
                     u16SosArray[8] = S_12_C3_DK2A2;
                     u16SosArray[9] = S_12_C4_DK2A2;
                }
                break;

                case E_VIF_SOUND_DK3_STEREO_A2:
                {
                     u16NotchArray[0] = N_A1_C0_DK3A2;
                     u16NotchArray[1] = N_A1_C1_DK3A2;
                     u16NotchArray[2] = N_A1_C2_DK3A2;

                     u16NotchArray[3] = N_A2_C0_DK3A2;
                     u16NotchArray[4] = N_A2_C1_DK3A2;
                     u16NotchArray[5] = N_A2_C2_DK3A2;

                     u16SosArray[0] = S_11_C0_DK3A2;
                     u16SosArray[1] = S_11_C1_DK3A2;
                     u16SosArray[2] = S_11_C2_DK3A2;
                     u16SosArray[3] = S_11_C3_DK3A2;
                     u16SosArray[4] = S_11_C4_DK3A2;

                     u16SosArray[5] = S_12_C0_DK3A2;
                     u16SosArray[6] = S_12_C1_DK3A2;
                     u16SosArray[7] = S_12_C2_DK3A2;
                     u16SosArray[8] = S_12_C3_DK3A2;
                     u16SosArray[9] = S_12_C4_DK3A2;
                }
                break;

                case E_VIF_SOUND_DK_MONO_NICAM:
                {
                     u16NotchArray[0] = N_A1_C0_DKMN;
                     u16NotchArray[1] = N_A1_C1_DKMN;
                     u16NotchArray[2] = N_A1_C2_DKMN;

                     u16NotchArray[3] = N_A2_C0_DKMN;
                     u16NotchArray[4] = N_A2_C1_DKMN;
                     u16NotchArray[5] = N_A2_C2_DKMN;

                     u16SosArray[0] = S_11_C0_DKMN;
                     u16SosArray[1] = S_11_C1_DKMN;
                     u16SosArray[2] = S_11_C2_DKMN;
                     u16SosArray[3] = S_11_C3_DKMN;
                     u16SosArray[4] = S_11_C4_DKMN;

                     u16SosArray[5] = S_12_C0_DKMN;
                     u16SosArray[6] = S_12_C1_DKMN;
                     u16SosArray[7] = S_12_C2_DKMN;
                     u16SosArray[8] = S_12_C3_DKMN;
                     u16SosArray[9] = S_12_C4_DKMN;
                }
                break;

                case E_VIF_SOUND_L:
                case E_VIF_SOUND_LL:
                {
                     u16NotchArray[0] = N_A1_C0_L;
                     u16NotchArray[1] = N_A1_C1_L;
                     u16NotchArray[2] = N_A1_C2_L;

                     u16NotchArray[3] = N_A2_C0_L;
                     u16NotchArray[4] = N_A2_C1_L;
                     u16NotchArray[5] = N_A2_C2_L;

                     u16SosArray[0] = S_11_C0_L;
                     u16SosArray[1] = S_11_C1_L;
                     u16SosArray[2] = S_11_C2_L;
                     u16SosArray[3] = S_11_C3_L;
                     u16SosArray[4] = S_11_C4_L;

                     u16SosArray[5] = S_12_C0_L;
                     u16SosArray[6] = S_12_C1_L;
                     u16SosArray[7] = S_12_C2_L;
                     u16SosArray[8] = S_12_C3_L;
                     u16SosArray[9] = S_12_C4_L;
                }
                break;

                case E_VIF_SOUND_MN:
                {
                     u16NotchArray[0] = N_A1_C0_NTSC;
                     u16NotchArray[1] = N_A1_C1_NTSC;
                     u16NotchArray[2] = N_A1_C2_NTSC;

                     u16NotchArray[3] = N_A2_C0_NTSC;
                     u16NotchArray[4] = N_A2_C1_NTSC;
                     u16NotchArray[5] = N_A2_C2_NTSC;

                     u16SosArray[0] = S_11_C0_NTSC;
                     u16SosArray[1] = S_11_C1_NTSC;
                     u16SosArray[2] = S_11_C2_NTSC;
                     u16SosArray[3] = S_11_C3_NTSC;
                     u16SosArray[4] = S_11_C4_NTSC;

                     u16SosArray[5] = S_12_C0_NTSC;
                     u16SosArray[6] = S_12_C1_NTSC;
                     u16SosArray[7] = S_12_C2_NTSC;
                     u16SosArray[8] = S_12_C3_NTSC;
                     u16SosArray[9] = S_12_C4_NTSC;
                }
                break;

                default:
                    break;
            }
        }
        break;

        case E_TUNER_SUBCMD_GET_DTV_IF_FREQ:
        {
            switch (u32Param1)
            {
                case E_TUNER_DTV_DVB_C_MODE:
                case E_TUNER_DTV_DVB_T_MODE:
                case E_TUNER_DTV_DVB_T2_MODE:
                    {
                        U16 *u16DtvIFFreq = (U16 *)pvoidParam3;
                        *u16DtvIFFreq = TUNER_DTV_IF_FREQ_KHz;
                    }
                    break;

                case E_TUNER_DTV_DTMB_MODE:
                    {
                        U16 *u16DtvIFFreq = (U16 *)pvoidParam3;
                        *u16DtvIFFreq = DTMB_TUNER_IF_FREQ_KHz;
                    }
                    break;

                case E_TUNER_DTV_ISDB_MODE:
                    {
                        U16 *u16DtvIFFreq = (U16 *)pvoidParam3;
                        *u16DtvIFFreq = TUNER_DTV_IF_FREQ_KHz;
                    }
                    break;
                case E_TUNER_DTV_ATSC_MODE:
                    {
                        U16 *u16DtvIFFreq = (U16 *)pvoidParam3;
                        *u16DtvIFFreq = TUNER_DTV_IF_FREQ_KHz;
                    }
                    break;
                default:
                    TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                    return FALSE;
            }
        }
            break;
        case E_TUNER_SUBCMD_GET_IQ_SWAP:
        {
            U8 *u8IqSwap = (U8 *)pvoidParam3;
            *u8IqSwap = TUNER_DTV_IQ_SWAP;
        }
            break;

        case E_TUNER_SUBCMD_GET_PEAKING_PARAMETER:
        {
            stVIFUserFilter *VIF_UserFilter = (stVIFUserFilter *)pvoidParam3;
            //ASSERT(VIF_UserFilter);
            switch (u32Param1)
            {
                case E_RFBAND_VHF_LOW:
                    if((u32Param2==(U32)IF_FREQ_B)||(u32Param2==(U32)IF_FREQ_G))
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else if(u32Param2==(U32)IF_FREQ_DK)
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else if(u32Param2==(U32)IF_FREQ_I)
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else if(u32Param2 == (U32)IF_FREQ_L)
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else if(u32Param2 == (U32)IF_FREQ_L_PRIME)
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else
                    {
                        TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                        return FALSE;
                    }
                    break;
                case E_RFBAND_VHF_HIGH:
                    if((u32Param2 == (U32)IF_FREQ_B) || (u32Param2 == (U32)IF_FREQ_G))
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else if(u32Param2 == (U32)IF_FREQ_DK)
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else if(u32Param2 == (U32)IF_FREQ_I)
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else if(u32Param2 == (U32)IF_FREQ_L)
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else if(u32Param2 == (U32)IF_FREQ_L_PRIME)
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else
                    {
                        TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                        return FALSE;
                    }
                    break;
                case E_RFBAND_UHF:
                    if((u32Param2 == (U32)IF_FREQ_B) || (u32Param2 == (U32)IF_FREQ_G))
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else if(u32Param2 == (U32)IF_FREQ_DK)
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else if(u32Param2 == (U32)IF_FREQ_I)
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else if(u32Param2 == (U32)IF_FREQ_L)
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else if(u32Param2 == (U32)IF_FREQ_L_PRIME)
                    {
                        VIF_UserFilter->VifSos21FilterC0  = VIF_SOS_21_FILTER_C0;
                        VIF_UserFilter->VifSos21FilterC1  = VIF_SOS_21_FILTER_C1;
                        VIF_UserFilter->VifSos21FilterC2  = VIF_SOS_21_FILTER_C2;
                        VIF_UserFilter->VifSos21FilterC3  = VIF_SOS_21_FILTER_C3;
                        VIF_UserFilter->VifSos21FilterC4  = VIF_SOS_21_FILTER_C4;

                        VIF_UserFilter->VifSos22FilterC0  = VIF_SOS_22_FILTER_C0;
                        VIF_UserFilter->VifSos22FilterC1  = VIF_SOS_22_FILTER_C1;
                        VIF_UserFilter->VifSos22FilterC2  = VIF_SOS_22_FILTER_C2;
                        VIF_UserFilter->VifSos22FilterC3  = VIF_SOS_22_FILTER_C3;
                        VIF_UserFilter->VifSos22FilterC4  = VIF_SOS_22_FILTER_C4;

                        VIF_UserFilter->VifSos31FilterC0  = VIF_SOS_31_FILTER_C0;
                        VIF_UserFilter->VifSos31FilterC1  = VIF_SOS_31_FILTER_C1;
                        VIF_UserFilter->VifSos31FilterC2  = VIF_SOS_31_FILTER_C2;
                        VIF_UserFilter->VifSos31FilterC3  = VIF_SOS_31_FILTER_C3;
                        VIF_UserFilter->VifSos31FilterC4  = VIF_SOS_31_FILTER_C4;
                        VIF_UserFilter->VifSos32FilterC0  = VIF_SOS_32_FILTER_C0;
                        VIF_UserFilter->VifSos32FilterC1  = VIF_SOS_32_FILTER_C1;
                        VIF_UserFilter->VifSos32FilterC2  = VIF_SOS_32_FILTER_C2;
                        VIF_UserFilter->VifSos32FilterC3  = VIF_SOS_32_FILTER_C3;
                        VIF_UserFilter->VifSos32FilterC4  = VIF_SOS_32_FILTER_C4;
                    }
                    else
                    {
                        TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                        return FALSE;
                    }
                    break;
                default:
                    TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                    return FALSE;
                    break;
            }
        }
            break;

        case E_TUNER_SUBCMD_GET_RF_LEVEL:
        {
            MXL661_COMMAND_T mxl661_inst;

            mxl661_inst.commandId = MXL661_TUNER_RX_PWR_REQ;
            // fake i2c slave id.
            mxl661_inst.MxLIf.cmdTunerPwrReq.I2cSlaveAddr = SlaveID;

            if (MxLWare661_API_GetTunerStatus(&mxl661_inst) != MXL_TRUE)
            {
                return FALSE;
            }
#if 0//kdrv
            *((float *)pvoidParam3) = (float)mxl661_inst.MxLIf.cmdTunerPwrReq.RxPwr/100.0;
#else
            //*((int *)pvoidParam3) = power_level*100
            *((int *)pvoidParam3) = (int)mxl661_inst.MxLIf.cmdTunerPwrReq.RxPwr;
#endif
        }
            break;
        case E_TUNER_SUBCMD_SELECT_AGC_PIN:
        {
            //UNUSED(pvoidParam3);
            switch(u32Param1)
            {
                case E_TUNER_DTV_DTMB_MODE:
                case E_TUNER_DTV_DVB_C_MODE:
                case E_TUNER_DTV_DVB_T_MODE:
                case E_TUNER_ATV_PAL_MODE:
                case E_TUNER_ATV_SECAM_L_PRIME_MODE:
                case E_TUNER_ATV_NTSC_MODE:
                {
                    if(u32Param2 == 0)
                    {
                        AGC_Pin_Internal = MXL661_AGC1;
                        DBG_TUNER(TUNER_PRINT("AGC_Pin_Internal = MXL661_AGC1 \n"));
                    }
                    else if(u32Param2 == 1)
                    {
                        AGC_Pin_Internal = MXL661_AGC2;
                        DBG_TUNER(TUNER_PRINT("AGC_Pin_Internal = MXL661_AGC2 \n"));
                    }
                }
                    break;

                case E_TUNER_DTV_ISDB_MODE:
                case E_TUNER_DTV_DVB_T2_MODE:
                {
                    if(u32Param2 == 0)
                    {
                        AGC_Pin_External = MXL661_AGC1;
                        DBG_TUNER(TUNER_PRINT("AGC_Pin_External = MXL661_AGC1 \n"));
                    }
                    else if(u32Param2 == 1)
                    {
                        AGC_Pin_External = MXL661_AGC2;
                        DBG_TUNER(TUNER_PRINT("AGC_Pin_External = MXL661_AGC2 \n"));
                    }
                }
                    break;

                default:
                    break;
            }
        }
            break;
        case E_TUNER_SUBCMD_USER_DEFINE:
        {
            if (u32Param1==1)
                Xtal_Fre=MXL661_XTAL_24MHz;
            else if(u32Param1==0)
                Xtal_Fre=MXL661_XTAL_16MHz;
        }
            break;
        default:
            //UNUSED(u8SubCmd);
            //UNUSED(u32Param1);
            //UNUSED(u32Param2);
            //UNUSED(pvoidParam3);
            return FALSE;
            break;
    }
    return TRUE;
}

int MDrv_TUNER_TunerInit(int minor, s_Tuner_dev_info *devTunerInit)
{
    int ret = TRUE;

    //TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    TUNER_PRINT("%s  _XtalCap=%d \n",__FUNCTION__,devTunerInit->s8_Xtalcap);
    SlaveID=devTunerInit->u8_slaveID;
    if ((devTunerInit->s8_Xtalcap==-1)||(devTunerInit->s8_Xtalcap<=0))  //Cap didn't have negative value
        XtalCap=MXL661_DEFAULTCAP;
    else
        XtalCap=devTunerInit->s8_Xtalcap;
    tuner_i2c_channel_bus = devTunerInit->u8_i2c_bus;
    TUNER_PRINT("%s  _u8SlaveID 0x%x\n",__FUNCTION__,SlaveID);
    TUNER_PRINT("%s  _XtalCap=%d \n",__FUNCTION__,XtalCap);
    TUNER_PRINT("%s  I2C_bus=%d \n",__FUNCTION__,tuner_i2c_channel_bus);
    MxLWare661_OEM_I2C_channel_setting(tuner_i2c_channel_bus);
    if(CHECK_EWBS_DEMOD_EXIST(tuner_i2c_channel_bus))
    isUseEWBSdemod=TRUE;
    if (minor < MAX_TUNER_DEV_NUM)
    {
        p_dev=&(dev[minor]);
        p_dev->u8_slaveID = SlaveID;
        p_dev->u8_i2c_bus = tuner_i2c_channel_bus;
        p_dev->s8_Xtalcap =XtalCap;
        if ( (p_dev->e_status == E_CLOSE)
            || (p_dev->e_status == E_SUSPEND) )
        {
            if(MDrv_TUNER_Connect(minor)== FALSE)
            {
                TUNER_PRINT("tuner init fail\n");
                ret=FALSE;
            }
            else
            {
                p_dev->e_status = E_WORKING;
            }
        }
    }
    else
    {
        ret = FALSE;
    }
    return ret;
}


int MDrv_TUNER_ConfigAGCMode(int minor, U32 eMode)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    static EN_TUNER_MODE eLastMode = E_TUNER_INVALID;
    MXL_STATUS status = MXL_FAILED;
    p_dev=&(dev[minor]);

    if((eLastMode == eMode) && (m_bNeedResetAGCMode == FALSE))//to avoid set repeatly
    {
        return TRUE;
    }
    DBG_TUNER(TUNER_PRINT("\n===>ConfigAGCMode eMode %d\n",eMode));
    eLastMode = eMode;

    if(m_bInATVScanMode == TRUE)//ATV scanning use external AGC to avoid channel missed
    {
        //IF Out setting:Done in ATV_SetTune()
        //external AGC1 Setting
        apiCmd.commandId = MXL661_TUNER_AGC_CFG;
        apiCmd.MxLIf.cmdAgcSetCfg.AgcSel = MXL661_AGC1;
        apiCmd.MxLIf.cmdAgcSetCfg.AgcType =MXL661_AGC_EXTERNAL;
        apiCmd.MxLIf.cmdAgcSetCfg.SetPoint = 66;
        apiCmd.MxLIf.cmdAgcSetCfg.AgcPolarityInverstion = MXL_DISABLE;
//        apiCmd.MxLIf.cmdAgcSetCfg.I2cSlaveAddr = SlaveID;
        status = MxLWare661_API_ConfigTuner(&apiCmd);
        if (MXL_FAILED == status)
        {
            TUNER_PRINT("ATV Scanning Error!  MXL_TUNER_AGC_CFG\n");
            return FALSE;
        }
    }
    else
    {
        switch(eMode)
        {
            case E_TUNER_DTV_DTMB_MODE:
                apiCmd.commandId = MXL661_DEV_IF_OUT_CFG;
                apiCmd.MxLIf.cmdIfOutCfg.IFOutFreq = IfFreqConversion(DTMB_TUNER_IF_FREQ_KHz);
                apiCmd.MxLIf.cmdIfOutCfg.ManualFreqSet= MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.IFInversion = MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_DTMB;

                apiCmd.MxLIf.cmdIfOutCfg.PathSel = MXL661_IF_PATH1;
//                apiCmd.MxLIf.cmdIfOutCfg.I2cSlaveAddr = SlaveID;
                status = MxLWare661_API_ConfigDevice(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("DTMB Error! MXL_DEV_IF_OUT_CFG\n");
                    return FALSE;
                }

                //AGC Setting:AGC1 external
                apiCmd.commandId = MXL661_TUNER_AGC_CFG;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcSel = AGC_Pin_Internal;//default: AGC1
                apiCmd.MxLIf.cmdAgcSetCfg.AgcType =MXL661_AGC_EXTERNAL;
                apiCmd.MxLIf.cmdAgcSetCfg.SetPoint = 66;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcPolarityInverstion = MXL_DISABLE;
//                apiCmd.MxLIf.cmdAgcSetCfg.I2cSlaveAddr = SlaveID;
                status = MxLWare661_API_ConfigTuner(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("\r\nDTMB Error! MXL_TUNER_AGC_CFG\n");
                    return FALSE;
                }
                break;

            case E_TUNER_DTV_ISDB_MODE:
                apiCmd.commandId = MXL661_DEV_IF_OUT_CFG;
                apiCmd.MxLIf.cmdIfOutCfg.IFOutFreq = IfFreqConversion(DTV_TUNER_IF_FREQ_KHz);
                apiCmd.MxLIf.cmdIfOutCfg.ManualFreqSet= MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.IFInversion = MXL_ENABLE;
                apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_ISDB;

                apiCmd.MxLIf.cmdIfOutCfg.PathSel = MXL661_IF_PATH1;
//                apiCmd.MxLIf.cmdIfOutCfg.I2cSlaveAddr = SlaveID;
                status = MxLWare661_API_ConfigDevice(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("ISDB Error! MXL_DEV_IF_OUT_CFG\n");
                    return FALSE;
                }

                //AGC Setting:AGC1 external
                apiCmd.commandId = MXL661_TUNER_AGC_CFG;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcSel = AGC_Pin_Internal;//defaut: AGC2
                apiCmd.MxLIf.cmdAgcSetCfg.AgcType =MXL661_AGC_EXTERNAL;//MXL661_AGC_SELF;
                if(isUseEWBSdemod)
                apiCmd.MxLIf.cmdAgcSetCfg.AgcType =MXL661_AGC_SELF;//MXL661_AGC_SELF;
                apiCmd.MxLIf.cmdAgcSetCfg.SetPoint = 66;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcPolarityInverstion = MXL_DISABLE;
//                apiCmd.MxLIf.cmdAgcSetCfg.I2cSlaveAddr = SlaveID;
                status = MxLWare661_API_ConfigTuner(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("\r\nISDB Error! MXL_TUNER_AGC_CFG\n");
                    return FALSE;
                }
                break;
            case E_TUNER_DTV_ATSC_MODE:

                apiCmd.commandId = MXL661_TUNER_AGC_CFG;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcSel = MXL661_AGC1;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcType =MXL661_AGC_EXTERNAL;
                apiCmd.MxLIf.cmdAgcSetCfg.SetPoint = 66;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcPolarityInverstion = MXL_DISABLE;
                //        apiCmd.MxLIf.cmdAgcSetCfg.I2cSlaveAddr = SlaveID;
                status = MxLWare661_API_ConfigTuner(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("ATV Scanning Error!  MXL_TUNER_AGC_CFG\n");
                    return FALSE;
                }
                DBG_TUNER(TUNER_PRINT("#####[Mstar Demod Debug] is m_bInATVScanMode == TRUE  mode\n"));



                apiCmd.commandId = MXL661_DEV_IF_OUT_CFG;
                apiCmd.MxLIf.cmdIfOutCfg.IFOutFreq = IfFreqConversion(DTV_TUNER_IF_FREQ_KHz);
                apiCmd.MxLIf.cmdIfOutCfg.ManualFreqSet= MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.IFInversion = MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_ATSC;

                apiCmd.MxLIf.cmdIfOutCfg.PathSel = MXL661_IF_PATH1;
                status = MxLWare661_API_ConfigDevice(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("ISDB Error! MXL_DEV_IF_OUT_CFG\n");
                    return FALSE;
                }


                DBG_TUNER(TUNER_PRINT("\n#####[Mstar Demod Debug] is E_TUNER_DTV_ATSC_MODE ATSC mode"));
                break;
            case E_TUNER_DTV_J83B_MODE:
                apiCmd.commandId = MXL661_DEV_IF_OUT_CFG;
                apiCmd.MxLIf.cmdIfOutCfg.IFOutFreq = IfFreqConversion(DTV_TUNER_IF_FREQ_KHz);
                apiCmd.MxLIf.cmdIfOutCfg.ManualFreqSet= MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.IFInversion = MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_ATSC;

                apiCmd.MxLIf.cmdIfOutCfg.PathSel = MXL661_IF_PATH1;
                status = MxLWare661_API_ConfigDevice(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("ISDB Error! MXL_DEV_IF_OUT_CFG\n");
                    return FALSE;
                }

                //AGC Setting:AGC1 external
                apiCmd.commandId = MXL661_TUNER_AGC_CFG;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcSel = AGC_Pin_Internal;//defaut: AGC2
                apiCmd.MxLIf.cmdAgcSetCfg.AgcType =MXL661_AGC_EXTERNAL;//MXL661_AGC_SELF;
                apiCmd.MxLIf.cmdAgcSetCfg.SetPoint = 66;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcPolarityInverstion = MXL_DISABLE;
                status = MxLWare661_API_ConfigTuner(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("\r\nISDB Error! MXL_TUNER_AGC_CFG\n");
                    return FALSE;
                }
                DBG_TUNER(TUNER_PRINT("\n#####[Mstar Demod Debug] is E_TUNER_DTV_J83B_MODE J83B mode"));
                break;

            case E_TUNER_DTV_DVB_C_MODE:
            case E_TUNER_DTV_DVB_T_MODE:
                apiCmd.commandId = MXL661_DEV_IF_OUT_CFG;
                apiCmd.MxLIf.cmdIfOutCfg.IFOutFreq = IfFreqConversion(DTV_TUNER_IF_FREQ_KHz);
                apiCmd.MxLIf.cmdIfOutCfg.ManualFreqSet= MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.IFInversion = MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_DVB;
                apiCmd.MxLIf.cmdIfOutCfg.PathSel = MXL661_IF_PATH1;
//                apiCmd.MxLIf.cmdIfOutCfg.I2cSlaveAddr = SlaveID;
                status = MxLWare661_API_ConfigDevice(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("DVBC Error! MXL_DEV_IF_OUT_CFG\n");
                    return FALSE;
                }

                //AGC Setting:AGC1 external
                apiCmd.commandId = MXL661_TUNER_AGC_CFG;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcSel = AGC_Pin_Internal;//default: AGC1
                apiCmd.MxLIf.cmdAgcSetCfg.AgcType =MXL661_AGC_EXTERNAL;
                apiCmd.MxLIf.cmdAgcSetCfg.SetPoint = 66;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcPolarityInverstion = MXL_DISABLE;
//                apiCmd.MxLIf.cmdAgcSetCfg.I2cSlaveAddr = SlaveID;
                status = MxLWare661_API_ConfigTuner(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("\r\nDVBC Error! MXL_TUNER_AGC_CFG\n");
                    return FALSE;
                }
                break;
            case E_TUNER_DTV_DVB_T2_MODE:
                apiCmd.commandId = MXL661_DEV_IF_OUT_CFG;
                apiCmd.MxLIf.cmdIfOutCfg.IFOutFreq = IfFreqConversion(DTV_TUNER_IF_FREQ_KHz);
                apiCmd.MxLIf.cmdIfOutCfg.ManualFreqSet= MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.IFInversion = MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_DVB_T2;
                apiCmd.MxLIf.cmdIfOutCfg.PathSel = MXL661_IF_PATH1;
//                apiCmd.MxLIf.cmdIfOutCfg.I2cSlaveAddr = SlaveID;
                status = MxLWare661_API_ConfigDevice(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("DVBT2 Error! MXL_DEV_IF_OUT_CFG\n");
                    return FALSE;
                }

                //AGC Setting:AGC1 external
                apiCmd.commandId = MXL661_TUNER_AGC_CFG;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcSel = AGC_Pin_External;//defaut: AGC2
                apiCmd.MxLIf.cmdAgcSetCfg.AgcType =MXL661_AGC_EXTERNAL;
                apiCmd.MxLIf.cmdAgcSetCfg.SetPoint = 66;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcPolarityInverstion = MXL_DISABLE;
//                apiCmd.MxLIf.cmdAgcSetCfg.I2cSlaveAddr = SlaveID;
                status = MxLWare661_API_ConfigTuner(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("\r\nDVBT2 Error! MXL_TUNER_AGC_CFG\n");
                    return FALSE;
                }
                break;

            case E_TUNER_ATV_PAL_MODE:
            case E_TUNER_ATV_SECAM_L_PRIME_MODE:
            case E_TUNER_ATV_NTSC_MODE:
                if(m_bNeedResetAGCMode == TRUE)//complete gain level set  for manual scan finished ,normal mode and autoscan finished done in atv_settune()
                {
                    apiCmd.commandId = MXL661_DEV_IF_OUT_CFG;
                    apiCmd.MxLIf.cmdIfOutCfg.IFOutFreq = MXL661_IF_5MHz;
                    apiCmd.MxLIf.cmdIfOutCfg.ManualFreqSet= MXL_DISABLE;
                    if(eMode == E_TUNER_ATV_SECAM_L_PRIME_MODE)
                    {
                        apiCmd.MxLIf.cmdIfOutCfg.IFInversion = MXL_DISABLE;
                        apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_ATV_SECAM;
                    }
                    else
                    {
                        apiCmd.MxLIf.cmdIfOutCfg.IFInversion = MXL_ENABLE;
                        apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_ATV_NORMAL;
                    }
                    apiCmd.MxLIf.cmdIfOutCfg.PathSel = MXL661_IF_PATH1;
//                    apiCmd.MxLIf.cmdIfOutCfg.I2cSlaveAddr = SlaveID;
                    status = MxLWare661_API_ConfigDevice(&apiCmd);
                    if (MXL_FAILED == status)
                    {
                        TUNER_PRINT("ATV normal Error! MXL_DEV_IF_OUT_CFG\n");
                        return FALSE;
                    }

                    //Application Mode setting
                    apiCmd.commandId = MXL661_TUNER_MODE_CFG;
                    if(eMode == E_TUNER_ATV_SECAM_L_PRIME_MODE)
                    {
                        apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_ANA_SECAM_L;
                        apiCmd.MxLIf.cmdModeCfg.IFOutGainLevel = IF_OUT_CFG_GAIN_LEVEL_ATV_SECAM;
                    }
                    else
                    {
                        apiCmd.MxLIf.cmdModeCfg.SignalMode = MXL661_ANA_PAL_D;
                        apiCmd.MxLIf.cmdModeCfg.IFOutGainLevel = IF_OUT_CFG_GAIN_LEVEL_ATV_NORMAL;
                    }
                    apiCmd.MxLIf.cmdChanTuneCfg.BandWidth = MXL661_ANA_TV_DIG_CABLE_BW_8MHz;
                    apiCmd.MxLIf.cmdModeCfg.IFOutFreqinKHz = 5000;
                    apiCmd.MxLIf.cmdModeCfg.XtalFreqSel = Xtal_Fre;
//                    apiCmd.MxLIf.cmdModeCfg.I2cSlaveAddr = SlaveID;
                    status = MxLWare661_API_ConfigTuner(&apiCmd);
                    if (MXL_FAILED == status)
                    {
                        TUNER_PRINT("ATV normal Error! MXL_TUNER_MODE_CFG\n");
                        return FALSE;
                    }
                }

                //AGC Setting:AGC1 external
                apiCmd.commandId = MXL661_TUNER_AGC_CFG;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcSel = AGC_Pin_Internal;//default: AGC1
                apiCmd.MxLIf.cmdAgcSetCfg.AgcType =MXL661_AGC_SELF;//to avoid ATV vertical line scrambled
                apiCmd.MxLIf.cmdAgcSetCfg.SetPoint = 66;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcPolarityInverstion = MXL_ENABLE;
//                apiCmd.MxLIf.cmdModeCfg.I2cSlaveAddr = SlaveID;
                status = MxLWare661_API_ConfigTuner(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("\r\nATV normal Error! MXL_TUNER_AGC_CFG\n");
                    return FALSE;
                }
                break;


            default:
                apiCmd.commandId = MXL661_DEV_IF_OUT_CFG;
                apiCmd.MxLIf.cmdIfOutCfg.IFOutFreq = IfFreqConversion(DTV_TUNER_IF_FREQ_KHz);
                apiCmd.MxLIf.cmdIfOutCfg.ManualFreqSet= MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.IFInversion = MXL_DISABLE;
                apiCmd.MxLIf.cmdIfOutCfg.GainLevel = IF_OUT_CFG_GAIN_LEVEL_DVB;
                apiCmd.MxLIf.cmdIfOutCfg.PathSel = MXL661_IF_PATH1;
//                apiCmd.MxLIf.cmdIfOutCfg.I2cSlaveAddr = SlaveID;
                status = MxLWare661_API_ConfigDevice(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("DVBC Error! MXL_DEV_IF_OUT_CFG\n");
                    return FALSE;
                }

                  //AGC Setting:AGC1 external
                apiCmd.commandId = MXL661_TUNER_AGC_CFG;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcSel = AGC_Pin_Internal;//default: AGC1
                apiCmd.MxLIf.cmdAgcSetCfg.AgcType =MXL661_AGC_EXTERNAL;
                apiCmd.MxLIf.cmdAgcSetCfg.SetPoint = 66;
                apiCmd.MxLIf.cmdAgcSetCfg.AgcPolarityInverstion = MXL_DISABLE;
//                apiCmd.MxLIf.cmdAgcSetCfg.I2cSlaveAddr = SlaveID;
                status = MxLWare661_API_ConfigTuner(&apiCmd);
                if (MXL_FAILED == status)
                {
                    TUNER_PRINT("\r\ndefault:Error! MXL_TUNER_AGC_CFG\n");
                    return FALSE;
                }
                break;
        }
    }

    m_bNeedResetAGCMode = FALSE;

    return TRUE;
}

int MDrv_TUNER_SetTunerInScanMode(int minor, U32 bScan)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    p_dev=&(dev[minor]);
    if(p_dev->m_bInATVScanMode != bScan)
    {
        p_dev->m_bInATVScanMode = bScan;
        m_bNeedResetAGCMode = TRUE;
    }
    return TRUE;
}

int MDrv_TUNER_SetTunerInFinetuneMode(int minor, U32 bFinetune)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    p_dev=&(dev[minor]);
    //m_bInATVFinetuneMode = bFinetune;
    p_dev->m_bInfinttuneMode = bFinetune;
    return TRUE;
}

int MDrv_TUNER_GetCableStatus(int minor, U32 eStatus)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    return 0;
}

int MDrv_TUNER_TunerReset(int minor)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    m_bNeedResetAGCMode = TRUE;
    return MDrv_TUNER_Connect(minor);
}

int MDrv_TUNER_IsLocked(int minor)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    return 0;
}

int MDrv_TUNER_GetRSSI(int minor, U16 u16Gain, U8 u8DType)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    return 0;
}

int MDrv_TUNER_Suspend(void)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    U8 i = 0;
    for (i=0; i<MAX_TUNER_DEV_NUM; i++)
    {
        if (dev[i].e_status == E_WORKING)
        {
            dev[i].e_status = E_SUSPEND;
            dev[i].pre_bw = E_RF_CH_BAND_INVALID;
            dev[i].s16Prev_finetune = 0;
            dev[i].m_bInATVScanMode = 0;
            dev[i].m_bInfinttuneMode = 0;
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
    int ret_code = 0;
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    for (i=0; i<MAX_TUNER_DEV_NUM; i++)
    {
        if (dev[i].e_status == E_SUSPEND)
        {
            m_bNeedResetAGCMode = TRUE; // for set AGC setting in resume stage
            SlaveID = dev[i].u8_slaveID;
            tuner_i2c_channel_bus = dev[i].u8_i2c_bus;
            XtalCap=dev[i].s8_Xtalcap;
            if (MDrv_TUNER_TunerInit((int)i,&dev[i])==TRUE)
            {
                if (dev[i].e_std == E_TUNER_DTV_DVB_T_MODE)
                {
                    ret_code = MDrv_TUNER_DTV_SetTune((int)i, dev[i].u32_freq, dev[i].u32_eband_bandwidth,dev[i].u32_eMode);
                    if (ret_code == FALSE)
                    {
                        TUNER_PRINT("Error, DTV_SetTune fail after resume.%d.%d.%d.%d\n",i,dev[i].u32_freq,dev[i].u32_eband_bandwidth,dev[i].u32_eMode);
                    }
                }
                else if (dev[i].e_std == E_TUNER_ATV_PAL_MODE)
                {
                    ret_code = MDrv_TUNER_ATV_SetTune((int)i, dev[i].u32_freq, dev[i].u32_eband_bandwidth, dev[i].u32_eMode, dev[i].u8_otherMode);
                    if (ret_code == FALSE)
                    {
                        TUNER_PRINT("Error, ATV_SetTune fail after resume.%d.%d.%d.%d.%d\n",i,dev[i].u32_freq,dev[i].u32_eband_bandwidth,dev[i].u32_eMode,dev[i].u8_otherMode);
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
    int retb = FALSE;
#if (0) // not yet support to MxL661
    //MdbPrint(u64ReqHdl,"---------MStar DEMOD Info---------\n\n");
    printf("---------MStar TUNER Info---------\n");

  // Printf the Info
    if (MDB_tuner_eNetworktype == TRUE)
    {
        TUNER_PRINT("Tuner Lock Status : DTV\n");
        TUNER_PRINT("DTV RF Freq : %d MHz\n", DTV_FREQ_MDB);
        TUNER_PRINT("DTV IF Freq : %d KHz\n", api->prop->dtv_lif_freq.offset);
        TUNER_PRINT("DTV BW : %d\n", api->prop->dtv_mode.bw);
        TUNER_PRINT("DTV Standard : %d\n", api->prop->dtv_mode.modulation);
        TUNER_PRINT("DTV invert spectrum: %d\n", api->prop->dtv_mode.invert_spectrum);
        TUNER_PRINT("DTV agc speed: %d\n", api->prop->dtv_agc_speed.if_agc_speed);
        TUNER_PRINT("DTV LIF OUT AMP: %d\n", api->prop->dtv_lif_out.amp);
        TUNER_PRINT("DTV RSSI: %s (dBm)\n", (S16)(api->rsp->tuner_status.rssi));
    }
    else
    {
	TUNER_PRINT("Tuner Lock Status : ATV\n");
	TUNER_PRINT("ATV RF Freq : %d KHz\n", ATV_FREQ_MDB);
	TUNER_PRINT("ATV IF Freq : %d KHz\n", api->prop->atv_lif_freq.offset);
	TUNER_PRINT("ATV Standard: %d\n", api->prop->atv_video_mode.video_sys);
	TUNER_PRINT("ATV invert spectrum: %d\n", api->prop->atv_video_mode.invert_spectrum);
	TUNER_PRINT("ATV agc speed: %d\n", api->prop->atv_agc_speed.if_agc_speed);
	TUNER_PRINT("ATV LIF OUT AMP: %d\n", api->prop->atv_lif_out.amp);
    }
#endif
    return retb;
}

ssize_t MDrv_TUNER_WriteTUNERMdbInfo(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    return 0;
}

#endif
