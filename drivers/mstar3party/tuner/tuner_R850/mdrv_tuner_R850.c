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
#include "mdrv_iic_io.h"

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#endif
#include <linux/slab.h>
#include "R850/R850.h"
//#include "R842/R842.c"
#include "BypassEWBSIIC.h"

#include "mdrv_tuner_datatype.h"

//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define malloc(x) kmalloc(x, GFP_KERNEL)
#define free kfree
#define printf TUNER_PRINT
#define TUNER_PRINT(fmt, args...)        printk("[%s][%05d] " fmt, match_name, __LINE__, ## args)


#ifndef UNUSED //to avoid compile warnings...
#define UNUSED(var) (void)((var) = (var))
#endif

#define MAPI_TRUE 1 // true
#define MAPI_FALSE 0 // false
#define DBG_TUNER(x)  //x
// ------------------------------------------------------------
// Other Define
// ------------------------------------------------------------
#define DIVIDER_RATIO_50KHz         0x05
#define DIVIDER_RATIO_31_25KHz      0x04
#define DIVIDER_RATIO_62_5KHz       0x03
#define DIVIDER_RATIO_125KHz        0x02
#define DIVIDER_RATIO_142_85KHz     0x01
#define DIVIDER_RATIO_166_7KHz      0x00

#define TN_FREQ_STEP                E_FREQ_STEP_62_5KHz
#define DIVIDER_RATIO               DIVIDER_RATIO_50KHz
#define TUNER_IF_FREQ_KHz           38900L
#define TUNER_L_PRIME_IF_FREQ_KHz   33900L

#define _TUNER_I2C_ID               0xC2
#define DIGITAL_TUNER_IF            (36)
#define MIN_FREQ_SUPPORT_KHz        40000
#define MAX_FREQ_SUPPORT_KHz        880000

#define TUNER_DTV_IQ_SWAP           1// iq swap
#define TUNER_VHF_LOWMIN_FREQ       46250L
#define TUNER_VHF_LOWMAX_FREQ       142000L
#define TUNER_VHF_HIGHMIN_FREQ      149000L
#define TUNER_VHF_HIGHMAX_FREQ      426000L
#define TUNER_UHF_MIN_FREQ_UK       470000L
#define TUNER_UHF_MIN_FREQ          434000L
#define TUNER_UHF_MAX_FREQ          863250L

#define DIVIDER_166K                0
#define DIVIDER_142K                1
#define DIVIDER_80K                 2
#define DIVIDER_62K                 3
#define DIVIDER_31K                 4
#define DIVIDER_50K                 5

#define TN_FREQ_SS_INVERSE_625      16      // 1/0.0625
#define TN_RSA_RSB_625              0x06
#define TN_RSA_RSB_50               0x05
#define TN_FREQ_SS_625              62.5    // kHz
#define TN_FREQ_SS_50               50      // kHz
#define TN_FREQ_SS_INVERSE_50       20      // 1/0.05
#define TN_FREQ_SS                  TN_FREQ_SS_625

#define TN_LOW_BAND                 0x01
#define TN_MID_BAND                 0x02
#define TN_HIGH_BAND                0x04

#define IF_OUT_LEVEL_114            0
#define IF_OUT_LEVEL_112            (1<<3)
#define IF_OUT_LEVEL_110            (2<<3)
#define IF_OUT_LEVEL_108            (3<<3)
#define IF_OUT_LEVEL_106            (4<<3)
#define IF_OUT_LEVEL_104            (5<<3)
#define IF_OUT_LEVEL_105            (6<<3)
#define IF_OUT_LEVEL_Disable        (7<<3)

#define DIGITAL_IF_AGC_ON           0x00
#define DIGITAL_IF_AGC_OFF          0x08

#define CHARGE_PUMP_1               (1<<6)
#define CHARGE_PUMP_2               (2<<6)

#define CHARGE_PUMP_3               (1<<1)

#define ANALOG_IF_OUT               0x04
#define DIGITAL_IF_OUT              0x00

#define FAST_LOCK                   0x20

#define BAND_SWITCH_UHF             0x04
#define BAND_SWITCH_VHF_HIGH        0x02
#define BAND_SWITCH_VHF_LOW         0x01

#define TUNER_HIBYTE(w)             ((U8) (((U16) (w) >> 8) & 0xFF))
#define TUNER_LOBYTE(w)             ((U8) (w & 0xFF))


// Analog picture carrier
#define ATV_VC_MN  6750L
#define ATV_VC_B   7250L
#define ATV_VC_GH  7750L
#define ATV_VC_I   7750L
#define ATV_VC_DK  7750L
#define ATV_VC_L   7750L
#define ATV_VC_LL  2250L

// [21:0], CR_RATE for Video_Carrier_BBB KHz, HEX((Video_Carrier_BBB/43200.0) * (2^22))
#define VIF_TUNER_TYPE                  1                   // 0: RF Tuner; 1: Silicon Tuner
#define VIF_CR_RATE_B                   (U32)(ATV_VC_B*4194304/43200+0.5)
#define VIF_CR_INVERT_B                 0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_GH                  (U32)(ATV_VC_GH*4194304/43200+0.5)
#define VIF_CR_INVERT_GH                0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_DK                  (U32)(ATV_VC_DK*4194304/43200+0.5)
#define VIF_CR_INVERT_DK                0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_I                   (U32)(ATV_VC_I*4194304/43200+0.5)
#define VIF_CR_INVERT_I                 0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_L                   (U32)(ATV_VC_L*4194304/43200+0.5)
#define VIF_CR_INVERT_L                 0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_LL                  (U32)(ATV_VC_LL*4194304/43200+0.5)
#define VIF_CR_INVERT_LL                1                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_MN                  (U32)(ATV_VC_MN*4194304/43200+0.5)
#define VIF_CR_INVERT_MN                0                   // Video carrier position; 0: high side; 1:low side


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

//Sound : L ==========================
// 8.75Mhz
#define N_A1_C0_LL 0x011B
#define N_A1_C1_LL 0x063C
#define N_A1_C2_LL 0x06D3
// 8.1Mhz
#define N_A2_C0_LL 0x0170
#define N_A2_C1_LL 0x063C
#define N_A2_C2_LL 0x0678
// 0.75Mhz
#define S_11_C0_LL 0x03BD
#define S_11_C1_LL 0x063C
#define S_11_C2_LL 0x0200
#define S_11_C3_LL 0x0406
#define S_11_C4_LL 0x0200
// 8.75Mhz
#define S_12_C0_LL 0x011B
#define S_12_C1_LL 0x063C
#define S_12_C2_LL 0x0200
#define S_12_C3_LL 0x06D3
#define S_12_C4_LL 0x0200


//End of Table===================

#define VIF_SOS_21_FILTER_C0        0x0268
#define VIF_SOS_21_FILTER_C1        0x071E
#define VIF_SOS_21_FILTER_C2        0x01D6
#define VIF_SOS_21_FILTER_C3        0x0598
#define VIF_SOS_21_FILTER_C4        0x010C
#define VIF_SOS_22_FILTER_C0        0x027C
#define VIF_SOS_22_FILTER_C1        0x0707
#define VIF_SOS_22_FILTER_C2        0x01E5
#define VIF_SOS_22_FILTER_C3        0x0584
#define VIF_SOS_22_FILTER_C4        0x0114

#if (ENABLE_CUS19 == 1)
#define VIF_SOS_31_FILTER_C0        0x01D5
#define VIF_SOS_31_FILTER_C1        0x06A8
#define VIF_SOS_31_FILTER_C2        0x0158
#define VIF_SOS_31_FILTER_C3        0x062C
#define VIF_SOS_31_FILTER_C4        0x0200
#define VIF_SOS_32_FILTER_C0        0x0322
#define VIF_SOS_32_FILTER_C1        0x06B8
#define VIF_SOS_32_FILTER_C2        0x0148
#define VIF_SOS_32_FILTER_C3        0x04DE
#define VIF_SOS_32_FILTER_C4        0x0200
#else
#define VIF_SOS_31_FILTER_C0        0x022A
#define VIF_SOS_31_FILTER_C1        0x0754
#define VIF_SOS_31_FILTER_C2        0x00AC
#define VIF_SOS_31_FILTER_C3        0x05D6
#define VIF_SOS_31_FILTER_C4        0x0200
#define VIF_SOS_32_FILTER_C0        0x01EB
#define VIF_SOS_32_FILTER_C1        0x078A
#define VIF_SOS_32_FILTER_C2        0x0076
#define VIF_SOS_32_FILTER_C3        0x0615
#define VIF_SOS_32_FILTER_C4        0x0200
#endif

/*@ </Definitions> @*/

#define ENABLE_TUNER_INIT_THREAD 0
//#define DTMB_SYSTEM_ENABLE       1

static U8 bInit_Rafael = 0;
#if ENABLE_TUNER_INIT_THREAD
static bool bInitializing = FALSE;
pthread_mutex_t m_mutexTunerInit = PTHREAD_MUTEX_INITIALIZER;
#endif

static U32 FREQ_RSSI;
typedef struct
{
    S16    LvdBm;
    U16 u16_if_agc;
}S_RSSI_TABLE;

typedef enum
{
    DEMOD_INTERN = 0x00,
    DEMOD_EXTEND,
} DEMOD_TYPE;

static S_RSSI_TABLE ssi_r850_INTERN[]=
{
    {-82,0xa334},
    {-81,0xa050},
    {-80,0x9df1},
    {-79,0x9b81},
    {-78,0x998f},
    {-77,0x97df},
    {-76,0x967e},
    {-75,0x9540},
    {-74,0x942e},
    {-73,0x9347},
    {-72,0x9257},
    {-71,0x9180},
    {-70,0x90c5},
    {-69,0x901c},
    {-68,0x8f6d},
    {-67,0x8ecc},
    {-66,0x8e3f},
    {-65,0x8daa},
    {-64,0x8d29},
    {-63,0x8ca7},
    {-62,0x8c20},
    {-61,0x8b99},
    {-60,0x8b03},
    {-59,0x8a8e},
    {-58,0x89ff},
    {-57,0x8960},
    {-56,0x88b6},
    {-55,0x8800},
    {-54,0x871a},
    {-53,0x8627},
    {-52,0x8519},
    {-51,0x8417},
};

static S_RSSI_TABLE ssi_r850_EXTERN[]=
{
    {-95,0xFF00},
    {-90,0xFF00},
    {-85,0x8B5C},
    {-80,0x81DC},
    {-75,0x7D44},
    {-70,0x7B88},
    {-65,0x770F},
    {-60,0x74FB},
    {-55,0x7256},
    {-50,0x6C8F},
    {-45,0x6986},
    {-40,0x6753},
    {-35,0x64E6},
    {-30,0x6295},
    {-25,0x606E},
    {-20,0x5E5B},
    {-15,0x5BF8},
    {-10,0x5A1F},
    {- 5,0x5971},
    {- 0,0x591D},
};

U32 sg_u32_rf_freq = 0;
R850_Standard_Type sg_cur_std = R850_STD_SIZE;

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
const char match_name[14]="Mstar-R850";
static U8 SlaveID=0;
static U8 tuner_i2c_channel_bus;
static U8 tuner_i2c_SlaveID;


static s_Tuner_dev_info *p_dev = NULL;
static s_Tuner_dev_info dev[MAX_TUNER_DEV_NUM] =
{
  {E_TUNER_DTV_DVB_T_MODE, E_CLOSE, E_RF_CH_BAND_8MHz, 0,0,0,0,0,0,0,0},
  {E_TUNER_DTV_DVB_T_MODE, E_CLOSE, E_RF_CH_BAND_8MHz, 0,0,0,0,0,0,0,0},
};


//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------
#ifdef CONFIG_MSTAR_UTOPIA2K_MDEBUG
U32 DTV_FREQ_MDB = 0;
U32 ATV_FREQ_MDB = 0;
bool MDB_tuner_eNetworktype = 0;
#endif

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
int MDrv_TUNER_Connect(int minor)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    p_dev=&(dev[minor]);
    return 0;
}

int MDrv_TUNER_Disconnect(int minor)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    p_dev=&(dev[minor]);
    return 0;
}

void R850_OEM_I2C_channel_setting(U8 bus_num,U8 SlaveID)
{
    tuner_i2c_channel_bus = bus_num;
    tuner_i2c_SlaveID=SlaveID;
    TUNER_PRINT("tuner_i2c_channel [%d] \n",tuner_i2c_channel_bus);
    TUNER_PRINT("tuner_i2c_SlaveID [%d] \n",tuner_i2c_SlaveID);
}

BOOL   UserWrittenI2CRead(U32 AddrSize, U8* pAddr,U32 ReadLen, U8* pData);
BOOL   UserWrittenI2CWrite (U32 AddrSize, U8* pAddr,U32 WriteLen, U8* pData);

//*--------------------------------------------------------------------------------------
//* Function Name       :   UserWrittenI2CRead
//* Object              :
//* Input Parameters    :   MAPI_U32 AddrSize,
//*                         MAPI_U8* pAddr,
//*                         MAPI_U32 ReadLen,
//*                         MAPI_U8* pData
//* Output Parameters   :   MAPI_BOOL.
//*--------------------------------------------------------------------------------------
BOOL UserWrittenI2CRead(U32 AddrSize, U8* pAddr,
                                U32 ReadLen, U8* pData)
{
    /* Variable declarations */
    int status;
    BOOL rbt= TRUE;

   status=MDrv_SW_IIC_ReadBytes(tuner_i2c_channel_bus ,tuner_i2c_SlaveID,AddrSize,pAddr, ReadLen, pData);
   //TUNER_PRINT("MDrv_SW_IIC_ReadBytes :  [%d] \n",status);
   if(status==0)
      rbt= TRUE;
   else
      rbt= FALSE;

   if(rbt != TRUE)
      TUNER_PRINT("Tuner read fail !~~");
   return rbt;
}

//*--------------------------------------------------------------------------------------
//* Function Name       : UserWrittenI2CWrite
//* Object              :
//* Input Parameters    :   MAPI_U32 AddrSize,
//*                         MAPI_U8* pAddr,
//*                         MAPI_U32 WriteLen,
//*                         MAPI_U8* pData
//* Output Parameters   :   MAPI_BOOL.
//*--------------------------------------------------------------------------------------
BOOL UserWrittenI2CWrite (U32 AddrSize, U8* pAddr,
                                    U32 WriteLen, U8* pData)
{
    /* Variable declarations */
    int status;
    BOOL rbt= TRUE;

    status=MDrv_SW_IIC_WriteBytes(tuner_i2c_channel_bus, tuner_i2c_SlaveID, AddrSize, pAddr, WriteLen, pData);
    //TUNER_PRINT("MDrv_SW_IIC_WriteBytes :  [%d] \n",status);
    if(status==0)
        rbt= TRUE;
    else
       rbt= FALSE;

    if(rbt != TRUE)
        TUNER_PRINT("Tuner written fail !~~");
    return rbt;

}

int MDrv_TUNER_ATV_SetTune(int minor, U32 u32FreqKHz, U32 eBand, U32 eMode, U8 otherMode)
{
#if 0 // ATV set tune haven't been ready in R850.c
    R842_Set_Info stR842_Info;
    R842_Standard_Type StandardMode;
        //UNUSED(eBand);
    TUNER_PRINT("%s minor: [%d] u32FreqKHz:[%d] eBand:[%d] eMode:[%d] otherMode:[%d]\n",__FUNCTION__,minor,u32FreqKHz,eBand,eMode,otherMode);
    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(1);
#if ENABLE_TUNER_INIT_THREAD
       if(bInitializing)
       return FALSE;
#endif

    if(!bInit_Rafael) //double check lock
    {
#if ENABLE_TUNER_INIT_THREAD
          mapi_scope_lock(scopeLock, &m_mutexTunerInit);
#endif
       if(!bInit_Rafael)
       {
          if (R842_Init((U8)XtalCap)==R842_Fail)
          {
             TUNER_PRINT("R842 init fail....\n");
          }
          else
          {
             bInit_Rafael = 1;
          }
       }
    }

    eBand = eBand;
    p_dev=&(dev[minor]);

    if (p_dev->e_status != E_WORKING)
    {
        TUNER_PRINT("[Error]%s,%d\n",__FILE__,__LINE__);
        return FALSE;
    }

    p_dev->e_std = E_TUNER_ATV_PAL_MODE;
    p_dev->u32_freq = u32FreqKHz;
    p_dev->u32_eband_bandwidth = eBand;
    p_dev->u32_eMode = eMode;
    p_dev->u8_otherMode = otherMode;


    if(eMode == E_TUNER_ATV_SECAM_L_PRIME_MODE)
    {
        StandardMode = R842_SECAM_L1_CIF_5M;
    }
    else if(eMode == E_TUNER_ATV_NTSC_MODE)
    {
        #if DTMB_SYSTEM_ENABLE
            StandardMode = R842_MN_5800;
        #else
            StandardMode = R842_MN_5800;
        #endif
    }
    else
    {
            //printf("OtherMode[%d]\r\n",u8OtherMode);
        if(otherMode == 0xff)
        {
            #if DTMB_SYSTEM_ENABLE
                StandardMode = R842_PAL_DK_CIF_5M;
            #else
                StandardMode = R842_PAL_DK_CIF_5M;
            #endif
        }
        else
        {
            switch((AUDIOSTANDARD_TYPE_)otherMode)
            {
                case E_AUDIOSTANDARD_BG_:
                case E_AUDIOSTANDARD_BG_A2_:
                case E_AUDIOSTANDARD_BG_NICAM_:
                #if DTMB_SYSTEM_ENABLE
                    if (u32FreqKHz <=  300000)
                        StandardMode = R842_PAL_B_7M_CIF_5M;
                    else
                        StandardMode = R842_PAL_BGH_8M_CIF_5M;
                #else
                    if (u32FreqKHz <=  300000)
                        StandardMode = R842_PAL_B_7M_CIF_5M;
                    else
                        StandardMode = R842_PAL_BGH_8M_CIF_5M;
                #endif
                break;

                case E_AUDIOSTANDARD_I_:
#if DTMB_SYSTEM_ENABLE
                        StandardMode = R842_PAL_I_CIF_5M;
#else
                        StandardMode = R842_PAL_I_CIF_5M;
#endif
                break;

                case E_AUDIOSTANDARD_DK_:
                case E_AUDIOSTANDARD_DK1_A2_:
                case E_AUDIOSTANDARD_DK2_A2_:
                case E_AUDIOSTANDARD_DK3_A2_:
                case E_AUDIOSTANDARD_DK_NICAM_:
#if DTMB_SYSTEM_ENABLE
                        StandardMode = R842_PAL_DK_CIF_5M;
#else
                        StandardMode = R842_PAL_DK_CIF_5M;
#endif
                break;

                case E_AUDIOSTANDARD_L_:
#if DTMB_SYSTEM_ENABLE
                        StandardMode = R842_SECAM_L_CIF_5M;
#else
                        StandardMode = R842_SECAM_L_CIF_5M;
#endif
                break;

                case E_AUDIOSTANDARD_M_:
                case E_AUDIOSTANDARD_M_BTSC_:
                case E_AUDIOSTANDARD_M_A2_:
                case E_AUDIOSTANDARD_M_EIA_J_:
#if DTMB_SYSTEM_ENABLE
                        StandardMode = R842_MN_5800;
#else
                        StandardMode = R842_MN_5800;
#endif
                break;

                default:
#if DTMB_SYSTEM_ENABLE
                        StandardMode = R842_PAL_DK_CIF_5M;
#else
                        StandardMode = R842_PAL_DK_CIF_5M;
#endif
                break;
            }
       }

    }


    memset(&stR842_Info, 0, sizeof(stR842_Info));
    stR842_Info.R842_Standard =StandardMode;
    stR842_Info.RF_KHz = u32FreqKHz;

    R842_SetPllData(stR842_Info);

    sg_u32_rf_freq = stR842_Info.RF_KHz;
    sg_cur_std = stR842_Info.R842_Standard;
    mdelay(10);
    printk("\n\r R842 wait 10");
    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(0);
#endif
    return TRUE;
}


int MDrv_TUNER_DVBS_SetTune(int minor, U16 u16CenterFreqMHz, U32 u32SymbolRateKs)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return FALSE;
}

int MDrv_TUNER_DTV_SetTune(int minor, U32 Freq, U32 eBandWidth, U32 eMode)
{
    R850_Set_Info stR850_Info;
    R850_Standard_Type StandardMode;


#if ENABLE_TUNER_INIT_THREAD
        if(bInitializing)
            return FALSE;
#endif

    if(!bInit_Rafael) //double check lock
    {
#if ENABLE_TUNER_INIT_THREAD
           mapi_scope_lock(scopeLock, &m_mutexTunerInit);
#endif
        if(!bInit_Rafael)
        {
            if (R850_Init(R850_TUNER_1,R850_DVB_T_6M)==RT_Fail)
            {
                TUNER_PRINT("R850 init fail....\n");
            }
            else
            {
                bInit_Rafael = 1;
            }
        }
    }

    switch(eMode)
    {
        case E_TUNER_DTV_ATSC_MODE:
            StandardMode = R850_ATSC_IF_5M;
            break;
        case E_TUNER_DTV_DTMB_MODE:
            StandardMode = R850_DTMB_8M_IF_5M;
            break;
        case E_TUNER_DTV_DVB_C_MODE:
            if(eBandWidth == E_RF_CH_BAND_6MHz)
            {
                StandardMode = R850_DVB_C_6M_IF_5M;
            }
            else
            {
                StandardMode = R850_DVB_C_8M_IF_5M;
            }
            break;
        case E_TUNER_DTV_DVB_T_MODE:
            if(eBandWidth == E_RF_CH_BAND_6MHz)
            {
                StandardMode = R850_DVB_T_6M_IF_5M;
            }
            else if(eBandWidth == E_RF_CH_BAND_7MHz)
            {
                StandardMode = R850_DVB_T_7M_IF_5M;
            }
            else
            {
                StandardMode = R850_DVB_T_8M_IF_5M;
            }
            break;
        case E_TUNER_DTV_DVB_T2_MODE:
            if(eBandWidth == E_RF_CH_BAND_6MHz)
            {
                StandardMode = R850_DVB_T2_6M_IF_5M;
            }
            else if(eBandWidth == E_RF_CH_BAND_7MHz)
            {
                StandardMode = R850_DVB_T2_7M_IF_5M;
            }
            else
            {
                StandardMode = R850_DVB_T2_8M_IF_5M;
            }
            break;
        case E_TUNER_DTV_ISDB_MODE:
            StandardMode = R850_ISDB_T_IF_5M;
            break;

        default:
            StandardMode = R850_DTMB_8M_IF_5M;
            /// TODO: error happened
            break;
    }

    memset(&stR850_Info, 0, sizeof(stR850_Info));
    stR850_Info.R850_Standard =StandardMode;
    stR850_Info.RF_KHz = Freq * 1000L;
    stR850_Info.R850_LT= LT_OFF;
    FREQ_RSSI = stR850_Info.RF_KHz;
    stR850_Info.R850_ClkOutMode = CLK_OUT_ON;


    R850_SetPllData(R850_TUNER_1,stR850_Info);

    return TRUE;
}




int MDrv_TUNER_ExtendCommand(int minor, U8 u8SubCmd, U32 u32Param1, U32 u32Param2, void* pvoidParam3)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    DBG_TUNER(TUNER_PRINT("%s, %d, u8SubCmd=%d, u32Param1=%d, u32Param2=%d\n",__FUNCTION__,__LINE__,u8SubCmd,u32Param1,u32Param2);)

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
                *u16IFFreq = TUNER_IF_FREQ_KHz;
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

        case E_TUNER_SUBCMD_GET_UHF_MAX_FREQ:
            {
                U32 *u32Freq = (U32 *)pvoidParam3;
                *u32Freq = TUNER_UHF_MAX_FREQ;
            }
            break;

        case E_TUNER_SUBCMD_GET_VIF_TUNER_TYPE:
        {
            U8 *u8VifTunerType = (U8 *)pvoidParam3;
            *u8VifTunerType = VIF_TUNER_TYPE;
        }
            break;

        case E_TUNER_SUBCMD_GET_VIF_PARA:
        {
            stVIFInitialIn *p_stVIFInitialIn = (stVIFInitialIn *)pvoidParam3;

            p_stVIFInitialIn->VifCrRate_B = VIF_CR_RATE_B;
            p_stVIFInitialIn->VifCrInvert_B = VIF_CR_INVERT_B;
            p_stVIFInitialIn->VifCrRate_GH = VIF_CR_RATE_GH;
            p_stVIFInitialIn->VifCrInvert_GH = VIF_CR_INVERT_GH;
            p_stVIFInitialIn->VifCrRate_DK = VIF_CR_RATE_DK;
            p_stVIFInitialIn->VifCrInvert_DK = VIF_CR_INVERT_DK;
            p_stVIFInitialIn->VifCrRate_I = VIF_CR_RATE_I;
            p_stVIFInitialIn->VifCrInvert_I = VIF_CR_INVERT_I;
            p_stVIFInitialIn->VifCrRate_L = VIF_CR_RATE_L;
            p_stVIFInitialIn->VifCrInvert_L = VIF_CR_INVERT_L;
            p_stVIFInitialIn->VifCrRate_LL = VIF_CR_RATE_LL;
            p_stVIFInitialIn->VifCrInvert_LL = VIF_CR_INVERT_LL;
            p_stVIFInitialIn->VifCrRate_MN = VIF_CR_RATE_MN;
            p_stVIFInitialIn->VifCrInvert_MN = VIF_CR_INVERT_MN;
        }
            break;

        case E_TUNER_SUBCMD_GET_VIF_NOTCH_SOSFILTER:
        {
            U32 u32NotchDataLength = u32Param2;
            U16* pu16NotchData = (U16 *)pvoidParam3;
            U16* pu16SOSFilterData = ((U16 *)pvoidParam3) + u32NotchDataLength;
            switch((EN_VIF_SOUND_SYSTEM)u32Param1)
            {
               case E_VIF_SOUND_B_STEREO_A2:
                    pu16NotchData[0] = N_A1_C0_B_A2;
                    pu16NotchData[1] = N_A1_C1_B_A2;
                    pu16NotchData[2] = N_A1_C2_B_A2;

                    pu16NotchData[3] = N_A2_C0_B_A2;
                    pu16NotchData[4] = N_A2_C1_B_A2;
                    pu16NotchData[5] = N_A2_C2_B_A2;

                    pu16SOSFilterData[0] = S_11_C0_B_A2;
                    pu16SOSFilterData[1] = S_11_C1_B_A2;
                    pu16SOSFilterData[2] = S_11_C2_B_A2;
                    pu16SOSFilterData[3] = S_11_C3_B_A2;
                    pu16SOSFilterData[4] = S_11_C4_B_A2;

                    pu16SOSFilterData[5] = S_12_C0_B_A2;
                    pu16SOSFilterData[6] = S_12_C1_B_A2;
                    pu16SOSFilterData[7] = S_12_C2_B_A2;
                    pu16SOSFilterData[8] = S_12_C3_B_A2;
                    pu16SOSFilterData[9] = S_12_C4_B_A2;
                    break;

                case E_VIF_SOUND_B_MONO_NICAM:
                    pu16NotchData[0] = N_A1_C0_B_NICAM;
                    pu16NotchData[1] = N_A1_C1_B_NICAM;
                    pu16NotchData[2] = N_A1_C2_B_NICAM;

                    pu16NotchData[3] = N_A2_C0_B_NICAM;
                    pu16NotchData[4] = N_A2_C1_B_NICAM;
                    pu16NotchData[5] = N_A2_C2_B_NICAM;

                    pu16SOSFilterData[0] = S_11_C0_B_NICAM;
                    pu16SOSFilterData[1] = S_11_C1_B_NICAM;
                    pu16SOSFilterData[2] = S_11_C2_B_NICAM;
                    pu16SOSFilterData[3] = S_11_C3_B_NICAM;
                    pu16SOSFilterData[4] = S_11_C4_B_NICAM;

                    pu16SOSFilterData[5] = S_12_C0_B_NICAM;
                    pu16SOSFilterData[6] = S_12_C1_B_NICAM;
                    pu16SOSFilterData[7] = S_12_C2_B_NICAM;
                    pu16SOSFilterData[8] = S_12_C3_B_NICAM;
                    pu16SOSFilterData[9] = S_12_C4_B_NICAM;
                    break;

                case E_VIF_SOUND_GH_STEREO_A2:
                    pu16NotchData[0] = N_A1_C0_GHA2;
                    pu16NotchData[1] = N_A1_C1_GHA2;
                    pu16NotchData[2] = N_A1_C2_GHA2;

                    pu16NotchData[3] = N_A2_C0_GHA2;
                    pu16NotchData[4] = N_A2_C1_GHA2;
                    pu16NotchData[5] = N_A2_C2_GHA2;

                    pu16SOSFilterData[0] = S_11_C0_GHA2;
                    pu16SOSFilterData[1] = S_11_C1_GHA2;
                    pu16SOSFilterData[2] = S_11_C2_GHA2;
                    pu16SOSFilterData[3] = S_11_C3_GHA2;
                    pu16SOSFilterData[4] = S_11_C4_GHA2;

                    pu16SOSFilterData[5] = S_12_C0_GHA2;
                    pu16SOSFilterData[6] = S_12_C1_GHA2;
                    pu16SOSFilterData[7] = S_12_C2_GHA2;
                    pu16SOSFilterData[8] = S_12_C3_GHA2;
                    pu16SOSFilterData[9] = S_12_C4_GHA2;
                    break;

                case E_VIF_SOUND_GH_MONO_NICAM:
                    pu16NotchData[0] = N_A1_C0_GHMN;
                    pu16NotchData[1] = N_A1_C1_GHMN;
                    pu16NotchData[2] = N_A1_C2_GHMN;

                    pu16NotchData[3] = N_A2_C0_GHMN;
                    pu16NotchData[4] = N_A2_C1_GHMN;
                    pu16NotchData[5] = N_A2_C2_GHMN;

                    pu16SOSFilterData[0] = S_11_C0_GHMN;
                    pu16SOSFilterData[1] = S_11_C1_GHMN;
                    pu16SOSFilterData[2] = S_11_C2_GHMN;
                    pu16SOSFilterData[3] = S_11_C3_GHMN;
                    pu16SOSFilterData[4] = S_11_C4_GHMN;

                    pu16SOSFilterData[5] = S_12_C0_GHMN;
                    pu16SOSFilterData[6] = S_12_C1_GHMN;
                    pu16SOSFilterData[7] = S_12_C2_GHMN;
                    pu16SOSFilterData[8] = S_12_C3_GHMN;
                    pu16SOSFilterData[9] = S_12_C4_GHMN;
                    break;

                case E_VIF_SOUND_I:
                    pu16NotchData[0] = N_A1_C0_I;
                    pu16NotchData[1] = N_A1_C1_I;
                    pu16NotchData[2] = N_A1_C2_I;

                    pu16NotchData[3] = N_A2_C0_I;
                    pu16NotchData[4] = N_A2_C1_I;
                    pu16NotchData[5] = N_A2_C2_I;

                    pu16SOSFilterData[0] = S_11_C0_I;
                    pu16SOSFilterData[1] = S_11_C1_I;
                    pu16SOSFilterData[2] = S_11_C2_I;
                    pu16SOSFilterData[3] = S_11_C3_I;
                    pu16SOSFilterData[4] = S_11_C4_I;

                    pu16SOSFilterData[5] = S_12_C0_I;
                    pu16SOSFilterData[6] = S_12_C1_I;
                    pu16SOSFilterData[7] = S_12_C2_I;
                    pu16SOSFilterData[8] = S_12_C3_I;
                    pu16SOSFilterData[9] = S_12_C4_I;
                    break;

                case E_VIF_SOUND_DK1_STEREO_A2:
                    pu16NotchData[0] = N_A1_C0_DK1A2;
                    pu16NotchData[1] = N_A1_C1_DK1A2;
                    pu16NotchData[2] = N_A1_C2_DK1A2;

                    pu16NotchData[3] = N_A2_C0_DK1A2;
                    pu16NotchData[4] = N_A2_C1_DK1A2;
                    pu16NotchData[5] = N_A2_C2_DK1A2;

                    pu16SOSFilterData[0] = S_11_C0_DK1A2;
                    pu16SOSFilterData[1] = S_11_C1_DK1A2;
                    pu16SOSFilterData[2] = S_11_C2_DK1A2;
                    pu16SOSFilterData[3] = S_11_C3_DK1A2;
                    pu16SOSFilterData[4] = S_11_C4_DK1A2;

                    pu16SOSFilterData[5] = S_12_C0_DK1A2;
                    pu16SOSFilterData[6] = S_12_C1_DK1A2;
                    pu16SOSFilterData[7] = S_12_C2_DK1A2;
                    pu16SOSFilterData[8] = S_12_C3_DK1A2;
                    pu16SOSFilterData[9] = S_12_C4_DK1A2;
                    break;

                case E_VIF_SOUND_DK2_STEREO_A2:
                    pu16NotchData[0] = N_A1_C0_DK2A2;
                    pu16NotchData[1] = N_A1_C1_DK2A2;
                    pu16NotchData[2] = N_A1_C2_DK2A2;

                    pu16NotchData[3] = N_A2_C0_DK2A2;
                    pu16NotchData[4] = N_A2_C1_DK2A2;
                    pu16NotchData[5] = N_A2_C2_DK2A2;

                    pu16SOSFilterData[0] = S_11_C0_DK2A2;
                    pu16SOSFilterData[1] = S_11_C1_DK2A2;
                    pu16SOSFilterData[2] = S_11_C2_DK2A2;
                    pu16SOSFilterData[3] = S_11_C3_DK2A2;
                    pu16SOSFilterData[4] = S_11_C4_DK2A2;

                    pu16SOSFilterData[5] = S_12_C0_DK2A2;
                    pu16SOSFilterData[6] = S_12_C1_DK2A2;
                    pu16SOSFilterData[7] = S_12_C2_DK2A2;
                    pu16SOSFilterData[8] = S_12_C3_DK2A2;
                    pu16SOSFilterData[9] = S_12_C4_DK2A2;
                    break;

                case E_VIF_SOUND_DK3_STEREO_A2:
                    pu16NotchData[0] = N_A1_C0_DK3A2;
                    pu16NotchData[1] = N_A1_C1_DK3A2;
                    pu16NotchData[2] = N_A1_C2_DK3A2;

                    pu16NotchData[3] = N_A2_C0_DK3A2;
                    pu16NotchData[4] = N_A2_C1_DK3A2;
                    pu16NotchData[5] = N_A2_C2_DK3A2;

                    pu16SOSFilterData[0] = S_11_C0_DK3A2;
                    pu16SOSFilterData[1] = S_11_C1_DK3A2;
                    pu16SOSFilterData[2] = S_11_C2_DK3A2;
                    pu16SOSFilterData[3] = S_11_C3_DK3A2;
                    pu16SOSFilterData[4] = S_11_C4_DK3A2;

                    pu16SOSFilterData[5] = S_12_C0_DK3A2;
                    pu16SOSFilterData[6] = S_12_C1_DK3A2;
                    pu16SOSFilterData[7] = S_12_C2_DK3A2;
                    pu16SOSFilterData[8] = S_12_C3_DK3A2;
                    pu16SOSFilterData[9] = S_12_C4_DK3A2;
                    break;

                case E_VIF_SOUND_DK_MONO_NICAM:
                    pu16NotchData[0] = N_A1_C0_DKMN;
                    pu16NotchData[1] = N_A1_C1_DKMN;
                    pu16NotchData[2] = N_A1_C2_DKMN;

                    pu16NotchData[3] = N_A2_C0_DKMN;
                    pu16NotchData[4] = N_A2_C1_DKMN;
                    pu16NotchData[5] = N_A2_C2_DKMN;

                    pu16SOSFilterData[0] = S_11_C0_DKMN;
                    pu16SOSFilterData[1] = S_11_C1_DKMN;
                    pu16SOSFilterData[2] = S_11_C2_DKMN;
                    pu16SOSFilterData[3] = S_11_C3_DKMN;
                    pu16SOSFilterData[4] = S_11_C4_DKMN;

                    pu16SOSFilterData[5] = S_12_C0_DKMN;
                    pu16SOSFilterData[6] = S_12_C1_DKMN;
                    pu16SOSFilterData[7] = S_12_C2_DKMN;
                    pu16SOSFilterData[8] = S_12_C3_DKMN;
                    pu16SOSFilterData[9] = S_12_C4_DKMN;
                    break;

                case E_VIF_SOUND_L:
                    pu16NotchData[0] = N_A1_C0_L;
                    pu16NotchData[1] = N_A1_C1_L;
                    pu16NotchData[2] = N_A1_C2_L;

                    pu16NotchData[3] = N_A2_C0_L;
                    pu16NotchData[4] = N_A2_C1_L;
                    pu16NotchData[5] = N_A2_C2_L;

                    pu16SOSFilterData[0] = S_11_C0_L;
                    pu16SOSFilterData[1] = S_11_C1_L;
                    pu16SOSFilterData[2] = S_11_C2_L;
                    pu16SOSFilterData[3] = S_11_C3_L;
                    pu16SOSFilterData[4] = S_11_C4_L;

                    pu16SOSFilterData[5] = S_12_C0_L;
                    pu16SOSFilterData[6] = S_12_C1_L;
                    pu16SOSFilterData[7] = S_12_C2_L;
                    pu16SOSFilterData[8] = S_12_C3_L;
                    pu16SOSFilterData[9] = S_12_C4_L;
                    break;

                case E_VIF_SOUND_LL:
                    pu16NotchData[0] = N_A1_C0_LL;
                    pu16NotchData[1] = N_A1_C1_LL;
                    pu16NotchData[2] = N_A1_C2_LL;

                    pu16NotchData[3] = N_A2_C0_LL;
                    pu16NotchData[4] = N_A2_C1_LL;
                    pu16NotchData[5] = N_A2_C2_LL;

                    pu16SOSFilterData[0] = S_11_C0_LL;
                    pu16SOSFilterData[1] = S_11_C1_LL;
                    pu16SOSFilterData[2] = S_11_C2_LL;
                    pu16SOSFilterData[3] = S_11_C3_LL;
                    pu16SOSFilterData[4] = S_11_C4_LL;

                    pu16SOSFilterData[5] = S_12_C0_LL;
                    pu16SOSFilterData[6] = S_12_C1_LL;
                    pu16SOSFilterData[7] = S_12_C2_LL;
                    pu16SOSFilterData[8] = S_12_C3_LL;
                    pu16SOSFilterData[9] = S_12_C4_LL;
                    break;

                case E_VIF_SOUND_MN:
                    pu16NotchData[0] = N_A1_C0_NTSC;
                    pu16NotchData[1] = N_A1_C1_NTSC;
                    pu16NotchData[2] = N_A1_C2_NTSC;

                    pu16NotchData[3] = N_A2_C0_NTSC;
                    pu16NotchData[4] = N_A2_C1_NTSC;
                    pu16NotchData[5] = N_A2_C2_NTSC;

                    pu16SOSFilterData[0] = S_11_C0_NTSC;
                    pu16SOSFilterData[1] = S_11_C1_NTSC;
                    pu16SOSFilterData[2] = S_11_C2_NTSC;
                    pu16SOSFilterData[3] = S_11_C3_NTSC;
                    pu16SOSFilterData[4] = S_11_C4_NTSC;

                    pu16SOSFilterData[5] = S_12_C0_NTSC;
                    pu16SOSFilterData[6] = S_12_C1_NTSC;
                    pu16SOSFilterData[7] = S_12_C2_NTSC;
                    pu16SOSFilterData[8] = S_12_C3_NTSC;
                    pu16SOSFilterData[9] = S_12_C4_NTSC;
                    break;

                default:
                    TUNER_PRINT("Warning:%s(),%d\n",__func__,__LINE__);
                    break;
            }
        }
            break;


        case E_TUNER_SUBCMD_GET_DTV_IF_FREQ:
        {
            switch (u32Param1) // demod mode
            {
                case E_TUNER_DTV_DVB_T_MODE:
                case E_TUNER_DTV_DVB_T2_MODE:
                case E_TUNER_DTV_DTMB_MODE:
                    *((U32 *)pvoidParam3)=5000L;
                    break;
                case E_TUNER_DTV_DVB_C_MODE:
                    *((U32 *)pvoidParam3)=5000L;
                    break;
                case E_TUNER_DTV_ATSC_MODE:
                    *((U32 *)pvoidParam3)=5000L;
                    break;
                case E_TUNER_DTV_ISDB_MODE:
                    *((U32 *)pvoidParam3)=5000L;
                    break;

                default:
                    TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                    return FALSE;
                    break;
            }
        }
            break;

        case E_TUNER_SUBCMD_GET_IQ_SWAP:
        {
            U8 *u8IqSwap = (U8 *)pvoidParam3;
            *u8IqSwap = TUNER_DTV_IQ_SWAP;
            if ( u32Param1 )
            {
                *u8IqSwap ^= 0x01;
            }
        }
            break;

        case E_TUNER_SUBCMD_GET_RF_TABLE:
        {
            (*(DMD_SSI_TABLE **)pvoidParam3)=NULL; // if not used, should return NULL;
            return FALSE;
        }
            break;

        case E_TUNER_SUBCMD_GET_RF_LEVEL:
        {
            R850_ErrCode err = RT_Fail;

            R850_RF_Gain_Info pRfGain;
            INT32 RfLevelDbm=0;
            R850_Standard_Type RT_Standard=R850_ISDB_T_IF_5M;
            UINT8 fgRfMaxGain;

            err = R850_GetRfGain(R850_TUNER_1,&pRfGain);
            err = R850_GetRfRssi(R850_TUNER_1,FREQ_RSSI, RT_Standard, &RfLevelDbm, &fgRfMaxGain);

            if(err != RT_Success)
            {
                *((float *)pvoidParam3)=-10.0f;
                TUNER_PRINT("\n xhw R828 RF Level Get Error\n");
            }
            else
            {
                if (pRfGain.RF_gain_comb > 592)
                {
                    U8 indx = 0;
                    U8 arry_size = 0;
                    float rf_dbm = 0.0;
                    U16 u16_gain = (U16)u32Param2;
                    S_RSSI_TABLE *ssi_r850;

                    switch(u32Param1)
                {
                        case DEMOD_INTERN:
                            arry_size = sizeof(ssi_r850_INTERN)/sizeof(S_RSSI_TABLE);
                            ssi_r850 = ssi_r850_INTERN;
                            break;
                        case DEMOD_EXTEND:
                            arry_size = sizeof(ssi_r850_EXTERN)/sizeof(S_RSSI_TABLE);
                            ssi_r850 = ssi_r850_EXTERN;
                            break;
                        default:
                            TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                            return FALSE;
                }

                   if (u16_gain == 0)
                    {
                        rf_dbm = (float)RfLevelDbm;
                    }
                   else if (u16_gain > ssi_r850[0].u16_if_agc)
                    {
                        rf_dbm = (float)(u16_gain - ssi_r850[1].u16_if_agc)/(float)(ssi_r850[0].u16_if_agc - ssi_r850[1].u16_if_agc)*(ssi_r850[0].LvdBm - ssi_r850[1].LvdBm)+ssi_r850[1].LvdBm;
                    }
                    else if (u16_gain <= ssi_r850[arry_size-1].u16_if_agc)
                    {
                        rf_dbm = (float)(u16_gain - ssi_r850[arry_size-2].u16_if_agc)/(float)(ssi_r850[arry_size-1].u16_if_agc - ssi_r850[arry_size-2].u16_if_agc)*(ssi_r850[arry_size-1].LvdBm - ssi_r850[arry_size-2].LvdBm)+ssi_r850[arry_size-2].LvdBm;
                    }
                    else
                    {
                        for(indx = 0;indx<arry_size;indx++)
                        {
                            if (u16_gain > ssi_r850[indx].u16_if_agc) break;
                        }
                        if (indx < arry_size)
                        {
                            rf_dbm = (float)(u16_gain - ssi_r850[indx].u16_if_agc)/(float)(ssi_r850[indx-1].u16_if_agc - ssi_r850[indx].u16_if_agc)*(ssi_r850[indx-1].LvdBm - ssi_r850[indx].LvdBm)+ssi_r850[indx].LvdBm;
                        }
                        else
                        {
                            TUNER_PRINT("[R850][Tuner]Warning!!!it's unreasonable at this case, indx=%d\n",indx);
                        }
                    }
                    *((float *)pvoidParam3)=rf_dbm;
                }
                else
                {
                   *((float *)pvoidParam3) = RfLevelDbm;
                }
            }
        }
            break;

        case E_TUNER_SUBCMD_GET_DEMOD_CONFIG:
        {
            switch (u32Param1) // demod mode
            {
                case E_TUNER_DTV_DVB_T_MODE:
                    *((MS_U8 **)pvoidParam3)=NULL;
                    break;
                case E_TUNER_DTV_DVB_C_MODE:
                    *((MS_U8 **)pvoidParam3)=NULL;
                    break;
                default:
                    TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                    return FALSE;
                    break;
            }
        }
            break;

        case E_TUNER_SUBCMD_GET_PEAKING_PARAMETER:
        {
            stVIFUserFilter *VIF_UserFilter = (stVIFUserFilter *)pvoidParam3;
            ASSERT(VIF_UserFilter);
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

        default:
            UNUSED(u8SubCmd);
            UNUSED(u32Param1);
            UNUSED(u32Param2);
            UNUSED(pvoidParam3);
            return FALSE;
            break;
    }

return TRUE;
}


int MDrv_TUNER_TunerInit(int minor,s_Tuner_dev_info *devTunerInit)
{
    int ret = TRUE;
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    SlaveID=devTunerInit->u8_slaveID;
    tuner_i2c_channel_bus = devTunerInit->u8_i2c_bus;
    R850_OEM_I2C_channel_setting(tuner_i2c_channel_bus,SlaveID);

    TUNER_PRINT("%s  _u8SlaveID 0x%x\n",__FUNCTION__,SlaveID);
    TUNER_PRINT("%s  I2C_bus=%d \n",__FUNCTION__,tuner_i2c_channel_bus);

    if (minor < MAX_TUNER_DEV_NUM)
    {
        p_dev=&(dev[minor]);
        p_dev->m_bInATVScanMode = FALSE;
        p_dev->u8_slaveID = SlaveID;
        p_dev->u8_i2c_bus = tuner_i2c_channel_bus;
        if ( (p_dev->e_status == E_CLOSE)
        || (p_dev->e_status == E_SUSPEND) )
        {

            if (R850_Init(R850_TUNER_1,R850_DVB_T_6M) == RT_Fail)
            {
                TUNER_PRINT("R850 init fail....\n");
                return FALSE;
            }
            else
            {
                p_dev->e_status = E_WORKING;
                bInit_Rafael=1;
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
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

int MDrv_TUNER_SetTunerInScanMode(int minor, U32 bScan)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    if(p_dev->m_bInATVScanMode != bScan)
    {
        p_dev->m_bInATVScanMode = bScan;
    }
    return TRUE;
}

int MDrv_TUNER_SetTunerInFinetuneMode(int minor, U32 bFinetune)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

int MDrv_TUNER_GetCableStatus(int minor, U32 eStatus)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

int MDrv_TUNER_TunerReset(int minor)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

int MDrv_TUNER_IsLocked(int minor)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

int MDrv_TUNER_Suspend(void)
{
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


    return 0;
}





int MDrv_TUNER_Resume(void)
{
    U8 i = 0;
    int ret_code = 0;

    for (i=0; i<MAX_TUNER_DEV_NUM; i++)
    {
        if (dev[i].e_status == E_SUSPEND)
        {
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



int MDrv_TUNER_GetRSSI(int minor, U16 u16Gain, U8 u8DType)
{
      S32 s32_rssi_dbm = 0;
       if (R850_GetTotalRssi(R850_TUNER_1,sg_u32_rf_freq, sg_cur_std, &s32_rssi_dbm) == R842_Fail)
       {
           TUNER_PRINT("[Error]R842_GetTotalRssi fail\n");
            s32_rssi_dbm = -10;
        }
        return s32_rssi_dbm;
}

ssize_t MDrv_TUNER_WriteTUNERMdbInfo(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

ssize_t MDrv_TUNER_GetTUNERMdbInfo(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int retb = FALSE;
    //MdbPrint(u64ReqHdl,"---------MStar DEMOD Info---------\n\n");
    TUNER_PRINT("---------MStar TUNER Info---------\n");

  // Printf the Info
    if (MDB_tuner_eNetworktype == TRUE)
    {
        TUNER_PRINT("Tuner Lock Status : DTV\n");

    }
    else
    {
        TUNER_PRINT("Tuner Lock Status : ATV\n");
    }
    return retb;
}

