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

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#endif

#include <linux/slab.h>
#include "Si2151/Si2151_typedefs.h"
#include "Si2151/Si2151_L1_API.h"
#include "Si2151/Si2151_L2_API.h"
#include "Si2151/si2151_i2c_api.h"
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

#ifndef UNUSED //to avoid compile warnings...
#define UNUSED(var) (void)((var) = (var))
#endif

#define TUNER_PRINT(fmt, args...)        printk("[%s][%05d] " fmt, match_name, __LINE__, ## args)
#define mcSHOW_HW_MSG(fmt, args...)  printk("[%s][%05d] " fmt, match_name, __LINE__, ## args)
#ifdef TUNER_DEBUG
    #define DBG_TUNER(x) x
#else
    #define DBG_TUNER(x) //x
#endif

#define C_Si2151_LO_ADDRESS             ((U8)  0xC0)

//PLL lock check parameters
#define C_Si2151_PF_TO_CF_SHIFT_B 2250 /*KHZ*/
#define C_Si2151_PF_TO_CF_SHIFT_G 2750 /*KHZ*/
#define C_Si2151_PF_TO_CF_SHIFT_DK 2750 /*KHZ*/
#define C_Si2151_PF_TO_CF_SHIFT_I  2750 /*KHZ*/
#define C_Si2151_PF_TO_CF_SHIFT_L  2750 /*KHZ*/
#define C_Si2151_PF_TO_CF_SHIFT_M  1750 /*KHZ*/
#define C_Si2151_PF_TO_CF_SHIFT_L1 2750 /*KHZ*/


// center frequencies
#define C_Si2151_IF_CENTER_B  4750 /*KHZ*/
#define C_Si2151_IF_CENTER_GH 4250 /*KHZ*/
#define C_Si2151_IF_CENTER_DK 5250 /*KHZ*/
#define C_Si2151_IF_CENTER_I  4750 /*KHZ*/
#define C_Si2151_IF_CENTER_L  5250 /*KHZ*/
#define C_Si2151_IF_CENTER_L1 5250 /*KHZ*/
#define C_Si2151_IF_CENTER_M  4250 /*KHZ*/
#define C_Si2151_IF_CENTER_N  4250 /*KHZ*/
#define C_Si2151_IF_FREQUENCY          ((U16)  5000)  /*kHz*/

#define TN_FREQ_STEP                    E_FREQ_STEP_62_5KHz //E_FREQ_STEP_50KHz
#define MAX_FINE_TUNE_STEP              16
#define FREQ_STEP_10                    625//62.5//50  //Khz


#define TUNER_VHF_LOWMIN_FREQ             46250L
#define TUNER_VHF_LOWMAX_FREQ             142000L
#define TUNER_VHF_HIGHMIN_FREQ            149000L
#define TUNER_VHF_HIGHMAX_FREQ            426000L
#define TUNER_UHF_MIN_FREQ_UK             470000L
#define TUNER_UHF_MIN_FREQ                434000L
#define TUNER_UHF_MAX_FREQ                863250L

#define VIF_TUNER_TYPE                  1                   // 0: RF Tuner; 1: Silicon Tuner
#define VIF_CR_RATE_B                   0x000A5ED1 // 4.75M + 2.25M   // [21:0], CR_RATE for 7.0 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_B                 0                       // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_GH                  0x000A5ED1 // 4.25M + 2.75M       // [21:0], CR_RATE for 7.0 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_GH                0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_DK                  0x000BDA13 // 5.25M + 2.75M   // [21:0], CR_RATE for 8.0 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_DK                0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_I                   0x000B1C72 // 4.75M + 2.75M          // [21:0], CR_RATE for 7.5 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_I                 0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_L                   0x000BDA13 // 5.25M + 2.75M   // [21:0], CR_RATE for 8.0 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_L                 0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_LL                  0x000BDA13 // 5.25M + 2.75M  // [21:0], CR_RATE for 8.0 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_LL                0                   // Video carrier position; 0: high side; 1:low side
#define VIF_CR_RATE_MN                  0x0008E38E // 4.25 + 1.75M  // [21:0], CR_RATE for 6.0 MHz, HEX((xxx/43.2) * (2^22))
#define VIF_CR_INVERT_MN                0                   // Video carrier position; 0: high side; 1:low side

#define TUNER_DTV_IQ_SWAP               1 // iq swap
#define TUNER_VIF_TAGC_ODMODE           0 // IF AGC OD MODE
#define TUNER_DVB_IF_AGC_MODE           0


#define MAX_TUNER_DEV_NUM 2
static S8 XtalCap=31;

// VideoIF = 7.0M
U16 VIF_A1_A2_SOS11_SOS12_B_Stereo_A2[16]=
{
 0x03C0,0x0628,0x0418,
 0x03C7,0x0628,0x0411,
 0x0143,0x0628,0x0200,0x06B0,0x0200,
 0x03C0,0x0628,0x0200,0x0418,0x0200
};

// VideoIF = 7.0M
U16 VIF_A1_A2_SOS11_SOS12_B_Mono_NICAM[16]=
{
 0x03C0,0x0628,0x0418,
 0x03C9,0x0628,0x040E,
 0x0143,0x0628,0x0200,0x06B0,0x0200,
 0x03C0,0x0628,0x0200,0x0418,0x0200
};

// VideoIF = 7.0M
U16 VIF_A1_A2_SOS11_SOS12_GH_Stereo_A2[16]=
{
 0x03C0,0x0628,0x0418,
 0x03C7,0x0628,0x0411,
 0x00B9,0x0628,0x0200,0x0740,0x0200,
 0x03C0,0x0628,0x0200,0x0418,0x0200
};

// VideoIF = 7.0M
U16 VIF_A1_A2_SOS11_SOS12_GH_Mono_NICAM[16]=
{
 0x03C0,0x0628,0x0418,
 0x03C9,0x0628,0x040E,
 0x00B9,0x0628,0x0200,0x0740,0x0200,
 0x03C0,0x0628,0x0200,0x0418,0x0200
};

// VideoIF = 8.0M
U16 VIF_A1_A2_SOS11_SOS12_DK1_Stereo_A2[16]=
{
 0x03C0,0x0628,0x0418,
 0x03B8,0x0628,0x0421,
 0x00B9,0x0628,0x0200,0x0740,0x0200,
 0x03C0,0x0628,0x0200,0x0418,0x0200
};

// VideoIF = 8.0M
U16 VIF_A1_A2_SOS11_SOS12_DK2_Stereo_A2[16]=
{
 0x03C0,0x0628,0x0418,
 0x03C7,0x0628,0x0411,
 0x00B9,0x0628,0x0200,0x0740,0x0200,
 0x03C0,0x0628,0x0200,0x0418,0x0200
};

// VideoIF = 8.0M
U16 VIF_A1_A2_SOS11_SOS12_DK3_Stereo_A2[16]=
{
 0x03C0,0x0628,0x0418,
 0x03A2,0x0628,0x0437,
 0x00B9,0x0628,0x0200,0x0740,0x0200,
 0x03C0,0x0628,0x0200,0x0418,0x0200
};

// VideoIF = 8.0M
U16 VIF_A1_A2_SOS11_SOS12_DK_Mono_NICAM[16]=
{
 0x03C0,0x0628,0x0418,
 0x03A7,0x0628,0x0432,
 0x00B9,0x0628,0x0200,0x0740,0x0200,
 0x03C0,0x0628,0x0200,0x0418,0x0200
};

// VideoIF = 7.5M
U16 VIF_A1_A2_SOS11_SOS12_I[16]=
{
 0x03C0,0x0628,0x0418,
 0x03CE,0x0628,0x040A,
 0x00B9,0x0628,0x0200,0x0740,0x0200,
 0x03C0,0x0628,0x0200,0x0418,0x0200
};

// VideoIF = 6.0M
U16 VIF_A1_A2_SOS11_SOS12_MN[16]=
{
 0x03C0,0x0628,0x0418,
 0x03C6,0x0628,0x0412,
 0x01C6,0x0628,0x0200,0x0627,0x0200,
 0x03C0,0x0628,0x0200,0x0418,0x0200
};

// VideoIF = 8.0M
U16 VIF_A1_A2_SOS11_SOS12_L[16]=
{
 0x03C0,0x0628,0x0418,
 0x03A7,0x0628,0x0432,
 0x00B9,0x0628,0x0200,0x0740,0x0200,
 0x03C0,0x0628,0x0200,0x0418,0x0200
};

// VideoIF = 8.0M
U16 VIF_A1_A2_SOS11_SOS12_LP[16]=
{
 0x03C0,0x0628,0x0418,
 0x03A7,0x0628,0x0432,
 0x00B9,0x0628,0x0200,0x0740,0x0200,
 0x03C0,0x0628,0x0200,0x0418,0x0200
};


typedef enum SI_Network
{
    SI_network_none        = 0x00,    /**< No network */
    SI_network_dvbt1       = 0x01,    /**< DVB-T (Terrestrial) */
    SI_network_dvbs1       = 0x02,    /**< DVB-S (Satellite)   */
    SI_network_dvbc        = 0x04,    /**< DVB-C (Cable) */
    SI_network_analog      = 0x08,    /**< Analog */
    SI_network_dvbt2       = 0x10,    /**< DVB-T2 (Terrestrial) */
    SI_network_dvbs2       = 0x20,    /**< DVB-S2 (Satellite)   */
    SI_network_dvbt        = 0x11,    /**< DVB-T, DVB-T2 (Terrestrial) */
    SI_network_dvbs        = 0x22,    /**< DVB-S, DVB-S2 (Satellite)   */
    SI_network_ip          = 0x40,    /**< IP network   */
    SI_network_fm          = 0x80     /**< Fm Radio    */
} SI_Network;

typedef enum SI_AnalogTVSystem
{
    si_tv_system_m,
    si_tv_system_bg,
    si_tv_system_i,
    si_tv_system_dk,
    si_tv_system_l,
    si_tv_system_lp,
    si_tv_system_invalid
}SI_AnalogTVSystem;

typedef enum
{
    E_Network_Type_DVBT =1,
    E_Network_Type_DVBC ,
    E_Network_Type_Analog_T ,
    E_Network_Type_Analog_C ,
    E_Network_Type_DVBT2,
    E_Network_Type_ATSC,
    E_Network_Type_ISDBT,
    E_Network_Type_ISDBC,
    E_Network_Type_DVBS,
    E_Network_Type_DTMB,
    E_Network_Type_QAM_US,
} Network_Type;


//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
const char match_name[16]="Mstar-Si2151";
static U8 SlaveID=0;
static U8 tuner_i2c_channel_bus=0;
static bool isUseEWBSdemod=FALSE;

U32 u32chantuneFreq=0;   // record the frequency set by ChannelTune command
//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
static L1_Si2151_Context *api = NULL;

static s_Tuner_dev_info *p_dev = NULL;
static s_Tuner_dev_info dev[MAX_TUNER_DEV_NUM] =
{
  {E_TUNER_INVALID, E_CLOSE, E_RF_CH_BAND_INVALID, 0,0,0,0,0,0,0,0},
  {E_TUNER_INVALID, E_CLOSE, E_RF_CH_BAND_INVALID, 0,0,0,0,0,0,0,0},
};

//SI2151 GPIO control pin
#define   u8gpio_control_bit_pin1         ((U8) 0x01)
#define   u8gpio_control_bit_pin2         ((U8) 0x02)

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------
#ifdef CONFIG_MSTAR_UTOPIA2K_MDEBUG
U32 DTV_FREQ_MDB = 0;
U32 ATV_FREQ_MDB = 0;
bool MDB_tuner_eNetworktype = 0;
#endif


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------

static SI_AnalogTVSystem Atv_system_enum_conversion(EN_TUNER_MODE eMode,AUDIOSTANDARD_TYPE_ OtherMode)
{

    SI_AnalogTVSystem ret_atv_sys = si_tv_system_invalid;

    switch (eMode)
    {
        case E_TUNER_ATV_PAL_MODE:
            {
                switch(OtherMode)
                {
                    case E_AUDIOSTANDARD_BG_:
                    case E_AUDIOSTANDARD_BG_A2_:
                    case E_AUDIOSTANDARD_BG_NICAM_:
                        ret_atv_sys = si_tv_system_bg;
                        break;
                    case E_AUDIOSTANDARD_I_:
                        ret_atv_sys = si_tv_system_i;
                        break;
                    case E_AUDIOSTANDARD_DK_:
                    case E_AUDIOSTANDARD_DK1_A2_:
                    case E_AUDIOSTANDARD_DK2_A2_:
                    case E_AUDIOSTANDARD_DK3_A2_:
                    case E_AUDIOSTANDARD_DK_NICAM_:
                        ret_atv_sys = si_tv_system_dk;
                        break;
                    case E_AUDIOSTANDARD_L_:
                        ret_atv_sys = si_tv_system_l;
                        break;
                    case E_AUDIOSTANDARD_M_:
                    case E_AUDIOSTANDARD_M_BTSC_:
                    case E_AUDIOSTANDARD_M_A2_:
                    case E_AUDIOSTANDARD_M_EIA_J_:
                        ret_atv_sys = si_tv_system_m;
                        break;
                    case E_AUDIOSTANDARD_NOTSTANDARD_:
                    default:
                        ret_atv_sys = si_tv_system_dk;
                        break;
                }
            }
            break;
        case E_TUNER_ATV_SECAM_L_PRIME_MODE:
            ret_atv_sys = si_tv_system_lp;
            break;
//        case E_TUNER_ATV_SECAM_L_MODE:
//            ret_atv_sys = si_tv_system_l;
//            break;
        case E_TUNER_ATV_NTSC_MODE:
            ret_atv_sys = si_tv_system_m;
            break;
        case E_TUNER_DTV_ISDB_MODE:
        case E_TUNER_DTV_DVB_T_MODE:
        case E_TUNER_DTV_DVB_C_MODE:
        case E_TUNER_DTV_DVB_S_MODE:
        case E_TUNER_DTV_DTMB_MODE:
        case E_TUNER_DTV_ATSC_MODE:
        case E_TUNER_DTV_DVB_T2_MODE:
        default:
            ret_atv_sys = si_tv_system_invalid;
            break;
    }

    return ret_atv_sys;
}

static Network_Type Dtv_system_enum_conversion(EN_TUNER_MODE eMode)
{
    Network_Type ret_dtv_sys = E_Network_Type_ISDBT;

    switch (eMode)
    {
        case E_TUNER_DTV_ISDB_MODE:
            ret_dtv_sys = E_Network_Type_ISDBT;
            break;
        case E_TUNER_DTV_DVB_T_MODE:
            ret_dtv_sys = E_Network_Type_DVBT;
            break;
        case E_TUNER_DTV_DVB_C_MODE:
            ret_dtv_sys = E_Network_Type_DVBC;
            break;
        case E_TUNER_DTV_DVB_S_MODE:
            ret_dtv_sys = E_Network_Type_DVBS;
            break;
        case E_TUNER_DTV_DTMB_MODE:
            ret_dtv_sys = E_Network_Type_DTMB;
            break;
        case E_TUNER_DTV_ATSC_MODE:
            ret_dtv_sys = E_Network_Type_ATSC;
            break;
        case E_TUNER_DTV_DVB_T2_MODE:
            ret_dtv_sys = E_Network_Type_DVBT2;
            break;
        case E_TUNER_DTV_J83B_MODE:
            ret_dtv_sys = E_Network_Type_QAM_US;
            break;
        case E_TUNER_ATV_PAL_MODE:
        case E_TUNER_ATV_SECAM_L_PRIME_MODE:
        case E_TUNER_ATV_NTSC_MODE:
        default:
            ret_dtv_sys = E_Network_Type_DVBT;
            break;
    }
    return ret_dtv_sys;
}


static int Si2151_GetRSSILevel(S16 *strength_100dBm)
{
    int return_code = 0;
    if ((return_code=Si2151_L1_TUNER_STATUS(api))!=0) {
        mcSHOW_HW_MSG(" [Silabs]: ERROR_READING_COMMAND Si2151_TUNER_STATUS\n");
        return return_code;
    }
    *strength_100dBm=100*(S16)(api->rsp->tuner_status.rssi);
    return TRUE;
}


/* Setup properties to switch standards. */
static void SetIfDemod(RF_CHANNEL_BANDWIDTH eBandWidth, Network_Type eNetworktype, SI_AnalogTVSystem TV_SYS)
{
    // set a global here to save the video standard,  because it's not always available from the caller.
    if(TV_SYS == si_tv_system_invalid)
    {
        switch (eNetworktype)
        {
#if (0) //for DVBT T2 merge FW same tuner setting is needed
           case E_Network_Type_DVBT:
                api->prop->dtv_mode.modulation = Si2151_DTV_MODE_PROP_MODULATION_DVBT;
                if(eBandWidth == E_RF_CH_BAND_6MHz)
                    api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_6MHZ;
                else if (eBandWidth == E_RF_CH_BAND_7MHz)
                    api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_7MHZ;
                else
                    api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_8MHZ;

                api->prop->dtv_mode.invert_spectrum = Si2151_DTV_MODE_PROP_INVERT_SPECTRUM_NORMAL;

                if (Si2151_L1_SetProperty2(api, Si2151_DTV_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_MODE_PROP\n");
                }
                api->prop->dtv_agc_speed.if_agc_speed = Si2151_DTV_AGC_SPEED_PROP_IF_AGC_SPEED_AUTO;
                api->prop->dtv_agc_speed.agc_decim = Si2151_DTV_AGC_SPEED_PROP_AGC_DECIM_OFF;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_AGC_SPEED_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_AGC_SPEED_PROP\n");
                }
                api->prop->dtv_initial_agc_speed_period.period = 0;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP\n");
                }
                /* AMP=17 to get IF_AGC=1.24Vrms in DVB-T when AMP is controlled by demod AGC (DTV_AGC_SOURCE=DLIF_AGC_3DB) */
                api->prop->dtv_lif_out.amp =20; //wayne
                //printf("***[SetIfDemod_DVB-T] dtv_lif_out.amp = %d ",api->prop->dtv_lif_out.amp);
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_OUT_PROP) != 0)
                {                    
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_OUT_PROP\n");
                }
                api->prop->dtv_lif_freq.offset =C_Si2151_IF_FREQUENCY;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_FREQ_PROP\n");
                }
                break;
           case E_Network_Type_DVBT2:
                api->prop->dtv_mode.modulation = Si2151_DTV_MODE_PROP_MODULATION_DVBT;
                if(eBandWidth == E_RF_CH_BAND_8MHz)
                    api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_8MHZ;
                else if (eBandWidth == E_RF_CH_BAND_7MHz)
                    api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_7MHZ;
                else
                    api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_6MHZ;

                api->prop->dtv_mode.invert_spectrum = Si2151_DTV_MODE_PROP_INVERT_SPECTRUM_NORMAL;

                if (Si2151_L1_SetProperty2(api, Si2151_DTV_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_MODE_PROP\n");
                }
                api->prop->dtv_agc_speed.if_agc_speed = Si2151_DTV_AGC_SPEED_PROP_IF_AGC_SPEED_39;
                api->prop->dtv_agc_speed.agc_decim = Si2151_DTV_AGC_SPEED_PROP_AGC_DECIM_4;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_AGC_SPEED_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_AGC_SPEED_PROP\n");
                }
                api->prop->dtv_initial_agc_speed_period.period = 210;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP\n");
                }
                /* AMP=32 to get IF_AGC=1.24Vrms in DVB-T2 when AMP is controlled by demod AGC (DTV_AGC_SOURCE=DLIF_AGC_3DB) */
                api->prop->dtv_lif_out.amp =25;
                TUNER_PRINT("***[SetIfDemod_DVB-T2] dtv_lif_out.amp = %d ",api->prop->dtv_lif_out.amp);
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_OUT_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_OUT_PROP\n");
                }
                api->prop->dtv_lif_freq.offset =C_Si2151_IF_FREQUENCY;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_FREQ_PROP\n");
                }
                break;
#endif
           case E_Network_Type_DVBT:
           case E_Network_Type_DVBT2:
                api->prop->dtv_mode.modulation = Si2151_DTV_MODE_PROP_MODULATION_DVBT;
                if(eBandWidth == E_RF_CH_BAND_6MHz)
                    api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_6MHZ;
                else if (eBandWidth == E_RF_CH_BAND_7MHz)
                    api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_7MHZ;
                else
                    api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_8MHZ;

                api->prop->dtv_mode.invert_spectrum = Si2151_DTV_MODE_PROP_INVERT_SPECTRUM_NORMAL;

                if (Si2151_L1_SetProperty2(api, Si2151_DTV_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_MODE_PROP\n");
                }
                api->prop->dtv_agc_speed.if_agc_speed = Si2151_DTV_AGC_SPEED_PROP_IF_AGC_SPEED_AUTO;
                api->prop->dtv_agc_speed.agc_decim = Si2151_DTV_AGC_SPEED_PROP_AGC_DECIM_OFF;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_AGC_SPEED_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_AGC_SPEED_PROP\n");
                }
                api->prop->dtv_initial_agc_speed_period.period = 0;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP\n");
                }
                /* AMP=17 to get IF_AGC=1.24Vrms in DVB-T when AMP is controlled by demod AGC (DTV_AGC_SOURCE=DLIF_AGC_3DB) */
                api->prop->dtv_lif_out.amp =20;	//wayne
                //printk("***[SetIfDemod_DVB-T] dtv_lif_out.amp = %d ",api->prop->dtv_lif_out.amp);
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_OUT_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_OUT_PROP\n");
                }
                //--------------Start: add for DVBT2 FEF--------------
                api->prop->dtv_initial_agc_speed.if_agc_speed = Si2151_DTV_INITIAL_AGC_SPEED_PROP_IF_AGC_SPEED_AUTO;
                api->prop->dtv_initial_agc_speed.agc_decim = Si2151_DTV_INITIAL_AGC_SPEED_PROP_AGC_DECIM_OFF;
                Si2151_L1_SetProperty2(api, Si2151_DTV_INITIAL_AGC_SPEED_PROP);

                api->prop->dtv_agc_auto_freeze.thld = 9;
                api->prop->dtv_agc_auto_freeze.timeout =63;
                Si2151_L1_SetProperty2(api, Si2151_DTV_AGC_AUTO_FREEZE_PROP);
                //--------------End: add for DVBT2 FEF--------------
                api->prop->dtv_lif_freq.offset =C_Si2151_IF_FREQUENCY;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_FREQ_PROP\n");
                }
                break;
           case E_Network_Type_DVBC:
                api->prop->dtv_mode.modulation = Si2151_DTV_MODE_PROP_MODULATION_DVBC;
                api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_8MHZ;
                api->prop->dtv_mode.invert_spectrum = Si2151_ATV_VIDEO_MODE_PROP_INVERT_SPECTRUM_INVERTED;

                if (Si2151_L1_SetProperty2(api, Si2151_DTV_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_MODE_PROP\n");
                }
                api->prop->dtv_agc_speed.if_agc_speed = Si2151_DTV_AGC_SPEED_PROP_IF_AGC_SPEED_AUTO;
                api->prop->dtv_agc_speed.agc_decim = Si2151_DTV_AGC_SPEED_PROP_AGC_DECIM_OFF;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_AGC_SPEED_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_AGC_SPEED_PROP\n");
                }
                api->prop->dtv_initial_agc_speed_period.period = 0;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP\n");
                }
                /* AMP=14 to get IF_AGC=1.20Vrms in DVB-C when AMP is controlled by demod AGC (DTV_AGC_SOURCE=DLIF_AGC_3DB) */
                api->prop->dtv_lif_out.amp =20;
                TUNER_PRINT("***[SetIfDemod_DVB-C] dtv_lif_out.amp = %d ",api->prop->dtv_lif_out.amp);

                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_OUT_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_OUT_PROP\n");
                }
                api->prop->dtv_lif_freq.offset =C_Si2151_IF_FREQUENCY;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_FREQ_PROP\n");
                }
                break;
           case E_Network_Type_ISDBT:
                api->prop->dtv_mode.modulation = Si2151_DTV_MODE_PROP_MODULATION_ISDBT;
                api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_6MHZ;
                api->prop->dtv_mode.invert_spectrum = Si2151_DTV_MODE_PROP_INVERT_SPECTRUM_NORMAL;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_MODE_PROP\n");
                }
                api->prop->dtv_agc_speed.if_agc_speed = Si2151_DTV_AGC_SPEED_PROP_IF_AGC_SPEED_AUTO;
                api->prop->dtv_agc_speed.agc_decim = Si2151_DTV_AGC_SPEED_PROP_AGC_DECIM_OFF;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_AGC_SPEED_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_AGC_SPEED_PROP\n");
                }
                api->prop->dtv_initial_agc_speed_period.period = 0;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP\n");
                }
                /* AMP=55 to get IF_AGC 1.0v in MSB1400 on 149B-C01A when AMP is controlled by demod AGC (DTV_AGC_SOURCE=DLIF_AGC_3DB) */
                api->prop->dtv_lif_out.amp =37;
                TUNER_PRINT("***[SetIfDemod_ISDBT] dtv_lif_out.amp = %d ",api->prop->dtv_lif_out.amp);
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_OUT_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_OUT_PROP\n");
                }

                api->prop->dtv_lif_freq.offset =C_Si2151_IF_FREQUENCY;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_FREQ_PROP\n");
                }
                break;
           case E_Network_Type_ATSC:
                api->prop->dtv_mode.modulation = Si2151_DTV_MODE_PROP_MODULATION_ATSC;
                api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_6MHZ;
                api->prop->dtv_mode.invert_spectrum = Si2151_DTV_MODE_PROP_INVERT_SPECTRUM_NORMAL;

                if (Si2151_L1_SetProperty2(api, Si2151_DTV_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_MODE_PROP\n");
                }
                api->prop->dtv_agc_speed.if_agc_speed = Si2151_DTV_AGC_SPEED_PROP_IF_AGC_SPEED_AUTO;
                api->prop->dtv_agc_speed.agc_decim = Si2151_DTV_AGC_SPEED_PROP_AGC_DECIM_OFF;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_AGC_SPEED_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_AGC_SPEED_PROP\n");
                }
                api->prop->dtv_initial_agc_speed_period.period = 0;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP\n");
                }
                /* AMP=17 to get IF_AGC=1.24Vrms in DVB-T when AMP is controlled by demod AGC (DTV_AGC_SOURCE=DLIF_AGC_3DB) */
                api->prop->dtv_lif_out.amp =20;
                TUNER_PRINT("***[SetIfDemod_ATSC] dtv_lif_out.amp = %d ",api->prop->dtv_lif_out.amp);

                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_OUT_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_OUT_PROP\n");
                }
                api->prop->dtv_lif_freq.offset = C_Si2151_IF_FREQUENCY;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_FREQ_PROP\n");
                }
                 api->prop->tuner_return_loss.mode = Si2151_TUNER_RETURN_LOSS_PROP_MODE_TERRESTRIAL;
                 if (Si2151_L1_SetProperty2(api, Si2151_TUNER_RETURN_LOSS_PROP_CODE) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_RETURN_LOSS_PROP_CODE\n");
                }

                api->prop->dtv_pga_limits.max = 56;
                api->prop->dtv_pga_limits.min = 24;
                if (Si2151_L1_SetProperty2(api,Si2151_DTV_PGA_LIMITS_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_PGA_LIMITS_PROP\n");
                }

                break;
           case E_Network_Type_QAM_US:
                api->prop->dtv_mode.modulation = Si2151_DTV_MODE_PROP_MODULATION_QAM_US;
                api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_6MHZ;
                api->prop->dtv_mode.invert_spectrum = Si2151_DTV_MODE_PROP_INVERT_SPECTRUM_NORMAL;

                if (Si2151_L1_SetProperty2(api, Si2151_DTV_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_MODE_PROP\n");
                }
                api->prop->dtv_agc_speed.if_agc_speed = Si2151_DTV_AGC_SPEED_PROP_IF_AGC_SPEED_AUTO;
                api->prop->dtv_agc_speed.agc_decim = Si2151_DTV_AGC_SPEED_PROP_AGC_DECIM_OFF;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_AGC_SPEED_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_AGC_SPEED_PROP\n");
                }
                api->prop->dtv_initial_agc_speed_period.period = 0;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP\n");
                }
                /* AMP=17 to get IF_AGC=1.24Vrms in DVB-T when AMP is controlled by demod AGC (DTV_AGC_SOURCE=DLIF_AGC_3DB) */
                api->prop->dtv_lif_out.amp =20;
                TUNER_PRINT("***[SetIfDemod_QAM_US] dtv_lif_out.amp = %d ",api->prop->dtv_lif_out.amp);

                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_OUT_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_OUT_PROP\n");
                }
                api->prop->dtv_lif_freq.offset = C_Si2151_IF_FREQUENCY;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_FREQ_PROP\n");
                }

                api->prop->tuner_return_loss.mode = Si2151_TUNER_RETURN_LOSS_PROP_MODE_TERRESTRIAL;
                 if (Si2151_L1_SetProperty2(api, Si2151_TUNER_RETURN_LOSS_PROP_CODE) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_RETURN_LOSS_PROP_CODE\n");
                }
                api->prop->dtv_pga_limits.max = 56;
                api->prop->dtv_pga_limits.min = 24;
                if (Si2151_L1_SetProperty2(api,Si2151_DTV_PGA_LIMITS_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_PGA_LIMITS_PROP\n");
                }
                break;
           case E_Network_Type_DTMB:
                api->prop->dtv_mode.modulation = Si2151_DTV_MODE_PROP_MODULATION_DTMB;
                api->prop->dtv_mode.bw = Si2151_DTV_MODE_PROP_BW_BW_8MHZ;
                api->prop->dtv_mode.invert_spectrum = Si2151_DTV_MODE_PROP_INVERT_SPECTRUM_NORMAL;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_MODE_PROP\n");
                }
                api->prop->dtv_agc_speed.if_agc_speed = Si2151_DTV_AGC_SPEED_PROP_IF_AGC_SPEED_AUTO;
                api->prop->dtv_agc_speed.agc_decim = Si2151_DTV_AGC_SPEED_PROP_AGC_DECIM_OFF;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_AGC_SPEED_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_AGC_SPEED_PROP\n");
                }
                api->prop->dtv_initial_agc_speed_period.period = 0;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_INITIAL_AGC_SPEED_PERIOD_PROP\n");
                }
                /* AMP=55 to get IF_AGC 1.0v in MSB1400 on 149B-C01A when AMP is controlled by demod AGC (DTV_AGC_SOURCE=DLIF_AGC_3DB) */
                api->prop->dtv_lif_out.amp =27;
                TUNER_PRINT("***[SetIfDemod_DTMB] dtv_lif_out.amp = %d ",api->prop->dtv_lif_out.amp);

                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_OUT_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_OUT_PROP\n");
                }

                api->prop->dtv_lif_freq.offset =C_Si2151_IF_FREQUENCY;
                if (Si2151_L1_SetProperty2(api, Si2151_DTV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_DTV_LIF_FREQ_PROP\n");
                }
                break;
           case E_Network_Type_ISDBC:
           case E_Network_Type_Analog_T:
           case E_Network_Type_Analog_C:
           case E_Network_Type_DVBS:
           default:
                TUNER_PRINT("[Error]%s,%s,%d\n",__FILE__,__FUNCTION__,__LINE__);
                break;
        }
    }
    else
    {
        switch (TV_SYS)
        {
            case si_tv_system_bg:
                    api->prop->atv_video_mode.invert_spectrum = Si2151_ATV_VIDEO_MODE_PROP_INVERT_SPECTRUM_INVERTED;
                if (Si2151_L1_SetProperty2(api, Si2151_ATV_VIDEO_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_VIDEO_MODE_PROP\n");
                }
                break;
            case si_tv_system_dk:
                api->prop->atv_video_mode.video_sys = Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_DK;
                api->prop->atv_video_mode.invert_spectrum = Si2151_ATV_VIDEO_MODE_PROP_INVERT_SPECTRUM_INVERTED;
                if (Si2151_L1_SetProperty2(api, Si2151_ATV_VIDEO_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_VIDEO_MODE_PROP\n");
                }
                api->prop->atv_lif_freq.offset = C_Si2151_IF_CENTER_DK;
                if (Si2151_L1_SetProperty2(api, Si2151_ATV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_LIF_FREQ_PROP\n");
                }
                break;
            case si_tv_system_i:
                api->prop->atv_video_mode.video_sys = Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_I;
                api->prop->atv_video_mode.invert_spectrum = Si2151_ATV_VIDEO_MODE_PROP_INVERT_SPECTRUM_INVERTED;
                if (Si2151_L1_SetProperty2(api, Si2151_ATV_VIDEO_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_VIDEO_MODE_PROP\n");
                }
                api->prop->atv_lif_freq.offset = C_Si2151_IF_CENTER_I;
                if (Si2151_L1_SetProperty2(api, Si2151_ATV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_LIF_FREQ_PROP\n");
                }
                break;
            case si_tv_system_m:
                api->prop->atv_video_mode.video_sys = Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_M;
                api->prop->atv_video_mode.invert_spectrum = Si2151_ATV_VIDEO_MODE_PROP_INVERT_SPECTRUM_INVERTED;
                if (Si2151_L1_SetProperty2(api, Si2151_ATV_VIDEO_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_VIDEO_MODE_PROP\n");
                }
                api->prop->atv_lif_freq.offset = C_Si2151_IF_CENTER_M;
                if (Si2151_L1_SetProperty2(api, Si2151_ATV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_LIF_FREQ_PROP\n");
                }
                break;
            case si_tv_system_l:
                api->prop->atv_video_mode.video_sys = Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_L;
                api->prop->atv_video_mode.invert_spectrum = Si2151_ATV_VIDEO_MODE_PROP_INVERT_SPECTRUM_INVERTED;
                if (Si2151_L1_SetProperty2(api, Si2151_ATV_VIDEO_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_VIDEO_MODE_PROP\n");
                }
                api->prop->atv_lif_freq.offset = C_Si2151_IF_CENTER_L;
                if (Si2151_L1_SetProperty2(api, Si2151_ATV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_LIF_FREQ_PROP\n");
                }
                api->prop->atv_lif_out.offset = Si2151_ATV_LIF_OUT_PROP_OFFSET_DEFAULT;
                break;
            case si_tv_system_lp:
                api->prop->atv_video_mode.video_sys = Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_LP;
                api->prop->atv_video_mode.invert_spectrum = Si2151_ATV_VIDEO_MODE_PROP_INVERT_SPECTRUM_INVERTED;
                if (Si2151_L1_SetProperty2(api, Si2151_ATV_VIDEO_MODE_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_VIDEO_MODE_PROP\n");
                }
                api->prop->atv_lif_freq.offset = C_Si2151_IF_CENTER_L1;
                if (Si2151_L1_SetProperty2(api, Si2151_ATV_LIF_FREQ_PROP) != 0)
                {
                    mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_LIF_FREQ_PROP\n");
                }
                break;
            //case si_tv_system_invalid:
            default:
                break;
        }
    }
}
#if 0
static U32 Si2151_GetDefaultIF(U32 Freq, SI_AnalogTVSystem TV_SYS)
{
    U32 offset=0;
    switch (TV_SYS)
    {
        case si_tv_system_bg:
            if (Freq < 300000)
            {
                offset = C_Si2151_IF_CENTER_B;
            }
            else
            {
                offset = C_Si2151_IF_CENTER_GH;
            }
            break;
        case si_tv_system_dk:
            offset = C_Si2151_IF_CENTER_DK;
            break;
        case si_tv_system_i:
            offset = C_Si2151_IF_CENTER_I;
            break;
        case si_tv_system_m:
            offset = C_Si2151_IF_CENTER_M;
            break;
        case si_tv_system_l:
            offset = C_Si2151_IF_CENTER_L;
            break;
        case si_tv_system_lp:
            offset = C_Si2151_IF_CENTER_L1;
            break;
        case si_tv_system_invalid:
        default:
            TUNER_PRINT("[Error]%s, %s, %d\n",__FILE__,__FUNCTION__,__LINE__);
            break;
    }
    return offset;
}
#endif
/*
int mdev_ATV_GetSignalStrength_Si2151(U16 *strength)
{
    float Prel = 0.0;
    float f_ssi = 0;
    float ch_power_db_a=0;
    float ch_power_db=0;

    ch_power_db_a = ch_power_db_a;
    Prel = Prel;

    ch_power_db = Si2151_GetRSSILevel(&ch_power_db);
    //the range of Si2151 strength if -5 to -75
    if ( ch_power_db < -75)
    {
        ch_power_db=-75;
    }
    else if ( ch_power_db >-5)
    {
        ch_power_db=-5;
    }
    f_ssi=(((-4)-(ch_power_db))*100)/72;
    *strength = (U16)(100-f_ssi);
    mcSHOW_HW_MSG((">>> SSI_CH_PWR(dB) = %f , Score = %f<<<\n", ch_power_db, f_ssi));
    mcSHOW_HW_MSG((">>> SSI = %d <<<\n", (int)*strength));
    return TRUE;
}
*/
static int device_tuner_si_2151_init(void)
{
    int retb=FALSE, error;

    if (NULL == api) {
        api = (L1_Si2151_Context *)malloc(sizeof(L1_Si2151_Context));
    }
    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(1);
    /* Software Init */
    Si2151_L1_API_Init(api,C_Si2151_LO_ADDRESS);
    /*** below power up setting is for successfully excuting Si2151_LoadFirmware_16 */
    api->cmd->power_up.clock_mode =  Si2151_POWER_UP_CMD_CLOCK_MODE_XTAL;
    api->cmd->power_up.en_xout    =  Si2151_POWER_UP_CMD_EN_XOUT_EN_XOUT;
    if (Si2151_Init(api) != 0 )
    {
        TUNER_PRINT("\n");
        TUNER_PRINT("ERROR ----------------------------------\n");
        TUNER_PRINT("ERROR initializing the Si2151!\n");
        TUNER_PRINT("ERROR ----------------------------------\n");
        TUNER_PRINT("\n");
        retb = FALSE;
    }
    else
    {
        /*** Enable external AGC1 control pin*/
        error=Si2151_L1_CONFIG_PINS (api,
                                                             Si2151_CONFIG_PINS_CMD_GPIO1_MODE_NO_CHANGE,
                                                             Si2151_CONFIG_PINS_CMD_GPIO1_READ_DO_NOT_READ,
                                                             Si2151_CONFIG_PINS_CMD_GPIO2_MODE_NO_CHANGE,
                                                             Si2151_CONFIG_PINS_CMD_GPIO2_READ_DO_NOT_READ,
                                                             Si2151_CONFIG_PINS_CMD_AGC1_MODE_ATV_DTV_AGC,
                                                             Si2151_CONFIG_PINS_CMD_AGC1_READ_DO_NOT_READ,
                                                             Si2151_CONFIG_PINS_CMD_AGC2_MODE_NO_CHANGE,
                                                             Si2151_CONFIG_PINS_CMD_AGC2_READ_DO_NOT_READ,
                                                             Si2151_CONFIG_PINS_CMD_XOUT_MODE_NO_CHANGE);
        TUNER_PRINT ("Si2151_L1_CONFIG_PINS, ATV_DTV_AGC1, error 0x%02x: %s\n", error, Si2151_L1_API_ERROR_TEXT(error));
        api->prop->atv_video_mode.video_sys = 0xff;
        api->prop->dtv_mode.modulation = 0xff;
        api->prop->dtv_agc_speed.if_agc_speed = 0xff;
        retb = TRUE;
    }
    u32chantuneFreq = 0;
    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(0);
    return retb;
}

static bool CheckFineTuneRange(U32 u32FreqKHz)
{
    bool retb = FALSE;
    if(((u32FreqKHz*10) > (u32chantuneFreq*10 - MAX_FINE_TUNE_STEP*FREQ_STEP_10)) && ((u32FreqKHz*10) < (u32chantuneFreq*10 + MAX_FINE_TUNE_STEP*FREQ_STEP_10)))
    {
        retb = TRUE;
    }
    else
    {
        TUNER_PRINT("%s return False\n", __FUNCTION__);
        retb = FALSE;
    }
    return retb;
}

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
int MDrv_TUNER_Connect(int minor)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    p_dev=&(dev[minor]);
    if(CHECK_EWBS_DEMOD_EXIST(tuner_i2c_channel_bus))
    isUseEWBSdemod=TRUE;
    return 0;
}

int MDrv_TUNER_Disconnect(int minor)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    p_dev=&(dev[minor]);
    return 0;
}

int MDrv_TUNER_ATV_SetTune(int minor, U32 u32FreqKHz, U32 eBand, U32 eMode, U8 otherMode)
{
    int retb = FALSE;
    U32 freqHz = 0;
    //int freq_offset=0;
    U32 timeout = 0;
    S16 s16Finetune = 0;
    S16 s16FinetuneStep = 0;
    SI_AnalogTVSystem TV_SYS = si_tv_system_invalid;
    U8 previous_video_sys = 0;

    #ifdef CONFIG_MSTAR_UTOPIA2K_MDEBUG
    ATV_FREQ_MDB = u32FreqKHz;
    MDB_tuner_eNetworktype = false;
    #endif

    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    TUNER_PRINT("%s, %d, Freq=%d Khz, eBand=%d, mode=%d, otherMode=%d\n",__FUNCTION__,__LINE__,u32FreqKHz,eBand,eMode,otherMode);

    //eBand = eBand;

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

    previous_video_sys = api->prop->atv_video_mode.video_sys;

    TV_SYS = Atv_system_enum_conversion(eMode,(AUDIOSTANDARD_TYPE_)otherMode);

    DBG_TUNER(TUNER_PRINT ("ATV_SetTune freq = %d; SYS = %d...........\n", u32FreqKHz, TV_SYS);)

    /* set the following properties back to their defaults in case it was optimized for DTMB application in DTV_SetTune()*/
    api->prop->tuner_return_loss_optimize.config               = Si2151_TUNER_RETURN_LOSS_OPTIMIZE_PROP_CONFIG_DISABLE;
    api->prop->tuner_return_loss_optimize_2.thld               =    31; /* (default    31) */
    api->prop->tuner_return_loss_optimize_2.window             =     0; /* (default     0) */
    api->prop->tuner_return_loss_optimize_2.engagement_delay   =    15; /* (default    15) */
    api->prop->tuner_tf1_boundary_offset.tf1_boundary_offset   =     0;

    api->prop->tuner_return_loss_optimize.thld                 =     0;
    api->prop->tuner_return_loss_optimize.engagement_delay     =     7;
    api->prop->tuner_return_loss_optimize.disengagement_delay  =    10;

    if (Si2151_L1_SetProperty2(api, Si2151_TUNER_RETURN_LOSS_OPTIMIZE_PROP) != 0)
    {
        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_RETURN_LOSS_OPTIMIZE_PROP\n");
    }
    if (Si2151_L1_SetProperty2(api, Si2151_TUNER_RETURN_LOSS_OPTIMIZE_2_PROP) != 0)
    {
        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_RETURN_LOSS_OPTIMIZE_2_PROP\n");
    }
    if (Si2151_L1_SetProperty2(api, Si2151_TUNER_TF1_BOUNDARY_OFFSET_PROP) != 0)
    {
        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_TF1_BOUNDARY_OFFSET_PROP\n");
    }

    if ((p_dev->m_bInATVScanMode == FALSE)&&(p_dev->m_bInfinttuneMode == TRUE) && (CheckFineTuneRange(u32FreqKHz) == TRUE))
    {
        // Fine-tune mode
        s16Finetune = u32chantuneFreq - u32FreqKHz;
        if (u32chantuneFreq != 0)
        {
            s16FinetuneStep = s16Finetune*2;//s16FinetuneStep = s16Finetune/0.5;
            if(Si2151_L1_FINE_TUNE(api, 0, s16FinetuneStep) != 0)
            {
                printk((" [Silabs]: Error Si2151_L1_FINE_TUNE\n"));
            }
            else
            {
                printk("\nSi2151_L1_FINE_TUNE u32FreqKHz: %u u32chantuneFreq: %u s16Finetune: %d s16FinetuneStep: %d\n", u32FreqKHz, u32chantuneFreq, s16Finetune, s16FinetuneStep);
            }
        }
        retb = TRUE;
    }
    else
    {
        /* do not go through SetIfdemod if same TV_SYS is used but only frequency is changed*/
        switch (TV_SYS)
        {
            case si_tv_system_bg:
                if(u32FreqKHz < 300000)
                {
                    freqHz = (u32FreqKHz * 1000) + C_Si2151_PF_TO_CF_SHIFT_B*1000;
                }
                else
                {
                    freqHz = (u32FreqKHz * 1000) + C_Si2151_PF_TO_CF_SHIFT_G*1000;
                }
                api->prop->tuner_lo_injection.band_1      = Si2151_TUNER_LO_INJECTION_PROP_BAND_1_HIGH_SIDE;
                api->prop->tuner_lo_injection.band_2      = Si2151_TUNER_LO_INJECTION_PROP_BAND_2_LOW_SIDE;
                api->prop->tuner_lo_injection.band_3      = Si2151_TUNER_LO_INJECTION_PROP_BAND_3_LOW_SIDE;
                if (Si2151_L1_SetProperty2(api,Si2151_TUNER_LO_INJECTION_PROP) != 0)
                {
                        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_LO_INJECTION_PROP\n");
                }
                if (u32FreqKHz < 300000 && previous_video_sys != Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_B )
                {
                    api->prop->atv_video_mode.video_sys=Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_B;
                    api->prop->atv_lif_freq.offset = C_Si2151_IF_CENTER_B;
                    if (Si2151_L1_SetProperty2(api, Si2151_ATV_LIF_FREQ_PROP) != 0)
                    {
                        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_LIF_FREQ_PROP\n");
                    }
                    SetIfDemod(E_RF_CH_BAND_8MHz, E_Network_Type_Analog_T, TV_SYS);
                    break;
                }
                if (u32FreqKHz >= 300000 && previous_video_sys != Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_GH)
                {
                    api->prop->atv_video_mode.video_sys=Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_GH;
                    api->prop->atv_lif_freq.offset = C_Si2151_IF_CENTER_GH;
                    if (Si2151_L1_SetProperty2(api, Si2151_ATV_LIF_FREQ_PROP) != 0)
                    {
                        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_ATV_LIF_FREQ_PROP\n");
                    }
                    SetIfDemod(E_RF_CH_BAND_8MHz, E_Network_Type_Analog_T, TV_SYS);
                    break;
                }
                break;
            case si_tv_system_dk:
                freqHz = (u32FreqKHz * 1000) + C_Si2151_PF_TO_CF_SHIFT_DK*1000;
                api->prop->tuner_lo_injection.band_1      = Si2151_TUNER_LO_INJECTION_PROP_BAND_1_HIGH_SIDE;
                api->prop->tuner_lo_injection.band_2      = Si2151_TUNER_LO_INJECTION_PROP_BAND_2_HIGH_SIDE;
                api->prop->tuner_lo_injection.band_3      = Si2151_TUNER_LO_INJECTION_PROP_BAND_3_HIGH_SIDE;
                if (Si2151_L1_SetProperty2(api,Si2151_TUNER_LO_INJECTION_PROP) != 0)
                {
                        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_LO_INJECTION_PROP\n");
                }
                if (previous_video_sys != Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_DK)
                {
                    SetIfDemod(E_RF_CH_BAND_8MHz, E_Network_Type_Analog_T, TV_SYS);
                }
                break;
            case si_tv_system_i:
                freqHz = (u32FreqKHz * 1000) + C_Si2151_PF_TO_CF_SHIFT_I*1000;
                api->prop->tuner_lo_injection.band_1      = Si2151_TUNER_LO_INJECTION_PROP_BAND_1_HIGH_SIDE;
                api->prop->tuner_lo_injection.band_2      = Si2151_TUNER_LO_INJECTION_PROP_BAND_2_HIGH_SIDE;
                api->prop->tuner_lo_injection.band_3      = Si2151_TUNER_LO_INJECTION_PROP_BAND_3_HIGH_SIDE;
                if (Si2151_L1_SetProperty2(api,Si2151_TUNER_LO_INJECTION_PROP) != 0)
                {
                        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_LO_INJECTION_PROP\n");
                }
                if (previous_video_sys != Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_I)
                {
                    SetIfDemod(E_RF_CH_BAND_8MHz, E_Network_Type_Analog_T, TV_SYS);
                }
                break;
            case si_tv_system_m:
                freqHz = (u32FreqKHz * 1000) + C_Si2151_PF_TO_CF_SHIFT_M*1000;
                api->prop->tuner_lo_injection.band_1      = Si2151_TUNER_LO_INJECTION_PROP_BAND_1_HIGH_SIDE;
                api->prop->tuner_lo_injection.band_2      = Si2151_TUNER_LO_INJECTION_PROP_BAND_2_HIGH_SIDE;
                api->prop->tuner_lo_injection.band_3      = Si2151_TUNER_LO_INJECTION_PROP_BAND_3_HIGH_SIDE;
                if (Si2151_L1_SetProperty2(api,Si2151_TUNER_LO_INJECTION_PROP) != 0)
                {
                        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_LO_INJECTION_PROP\n");
                }
                if (previous_video_sys != Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_M)
                {
                    SetIfDemod(E_RF_CH_BAND_8MHz, E_Network_Type_Analog_T, TV_SYS);
                }
                break;
            case si_tv_system_l:
                freqHz = (u32FreqKHz * 1000) + C_Si2151_PF_TO_CF_SHIFT_L*1000;
                api->prop->tuner_lo_injection.band_1      = Si2151_TUNER_LO_INJECTION_PROP_BAND_1_HIGH_SIDE;
                api->prop->tuner_lo_injection.band_2      = Si2151_TUNER_LO_INJECTION_PROP_BAND_2_LOW_SIDE;
                api->prop->tuner_lo_injection.band_3      = Si2151_TUNER_LO_INJECTION_PROP_BAND_3_LOW_SIDE;
                if (Si2151_L1_SetProperty2(api,Si2151_TUNER_LO_INJECTION_PROP) != 0)
                {
                        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_LO_INJECTION_PROP\n");
                }
                if (previous_video_sys != Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_L)
                {
                    SetIfDemod(E_RF_CH_BAND_8MHz, E_Network_Type_Analog_T, TV_SYS);
                }
                break;
            case si_tv_system_lp:
                freqHz = (u32FreqKHz * 1000) - C_Si2151_PF_TO_CF_SHIFT_L1*1000;
                api->prop->tuner_lo_injection.band_1      = Si2151_TUNER_LO_INJECTION_PROP_BAND_1_HIGH_SIDE;
                api->prop->tuner_lo_injection.band_2      = Si2151_TUNER_LO_INJECTION_PROP_BAND_2_LOW_SIDE;
                api->prop->tuner_lo_injection.band_3      = Si2151_TUNER_LO_INJECTION_PROP_BAND_3_LOW_SIDE;
                if (Si2151_L1_SetProperty2(api,Si2151_TUNER_LO_INJECTION_PROP) != 0)
                {
                        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_LO_INJECTION_PROP\n");
                }
                if (previous_video_sys != Si2151_ATV_VIDEO_MODE_PROP_VIDEO_SYS_LP)
                {
                    SetIfDemod(E_RF_CH_BAND_8MHz, E_Network_Type_Analog_T, TV_SYS);
                }
                break;
            case si_tv_system_invalid:
            default:
                mcSHOW_HW_MSG("[Error]%s, %s, %d\n", __FILE__,__FUNCTION__,__LINE__);
                break;
        }


        if(p_dev->m_bInATVScanMode == TRUE)
        {
            // Scan Mode
            p_dev->m_bInfinttuneMode = FALSE;
            if (Si2151_L1_ATV_SCAN_TUNE(api, freqHz) != 0)
            {
                mcSHOW_HW_MSG(" [Silabs]: Error Si2151_L1_ATV_SCAN_TUNE\n");
                return FALSE;
            }

            //system_wait(40); //wait 40ms
            // usleep(35000);  //wait 35ms
            mdelay(35);
            retb = TRUE;
            DBG_TUNER(TUNER_PRINT("!!!!!!!!!!!!!ATV_SCAN_TUNE complete..................................\n");)

        }
        else
        {
            if (Si2151_L1_TUNER_TUNE_FREQ(api,Si2151_TUNER_TUNE_FREQ_CMD_MODE_ATV,  freqHz) != 0)
            {
                mcSHOW_HW_MSG(" [Silabs]: Error Si2151_L1_TUNER_TUNE_FREQ\n");
            }

            /* wait for TUNINT, timeout is 36ms */
            timeout = 36;
            // start_time = MsOS_GetSystemTime();
            while( (timeout--) > 0)
            {
                if (Si2151_L1_CheckStatus(api) != 0)
                    return FALSE;
                if (api->status->tunint)
                {
                    retb = TRUE;
                    break;
                }
                mdelay(1);
            }

            /* wait for ATVINT, timeout is 110ms */
            timeout = 110;
            // start_time = MsOS_GetSystemTime();
            while( (timeout--) > 0 )
            {
                if (Si2151_L1_CheckStatus(api) != 0)
                    return FALSE;
                if (api->status->atvint)
                {
                    TUNER_PRINT("!!!!!!!!!!!!!atvint fail\n");
                    retb = TRUE;
                    break;
                }
                else if(timeout<=0)
                   DBG_TUNER(TUNER_PRINT("atvint done\n"));             
                mdelay(1);
            }
        }
        u32chantuneFreq = u32FreqKHz;
    }
    
    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(0);

    return retb;
}

int MDrv_TUNER_DVBS_SetTune(int minor, U16 u16CenterFreqMHz, U32 u32SymbolRateKs)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

int MDrv_TUNER_DTV_SetTune(int minor, U32 freq, U32 eBandWidth, U32 eMode)
{
    int retb = FALSE;
    U32 freqHz = 0;
    U32 timeout = 0;
    Network_Type eNetworktype = E_Network_Type_DVBT;
    unsigned char previous_dtv_mode = 0;
    unsigned char previous_agc_speed = 0;

    #ifdef CONFIG_MSTAR_UTOPIA2K_MDEBUG
    DTV_FREQ_MDB = freq;
    MDB_tuner_eNetworktype = TRUE;
    #endif

    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    TUNER_PRINT("%s, %d, Freq=%d Hz, bw=%d, mode=%d\n",__FUNCTION__,__LINE__,freq,eBandWidth,eMode);

    eNetworktype = Dtv_system_enum_conversion(eMode);

    p_dev=&(dev[minor]);

    if (p_dev->e_status != E_WORKING)
    {
        TUNER_PRINT("[Error]%s,%d\n",__FILE__,__LINE__);
        return FALSE;
    }

    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(1);

    previous_dtv_mode=api->prop->dtv_mode.modulation;
    previous_agc_speed=api->prop->dtv_agc_speed.if_agc_speed;

    DBG_TUNER(TUNER_PRINT ("Si2151_TunerSetFreq freq = %d; Band = %d Network = %d\n", (U16)freq,eBandWidth, eNetworktype);)

    p_dev->e_std = E_TUNER_DTV_DVB_T_MODE;
    p_dev->u32_freq = freq;
    p_dev->u32_eband_bandwidth = eBandWidth;
    p_dev->u32_eMode = eMode;

    api->prop->tuner_lo_injection.band_1      = Si2151_TUNER_LO_INJECTION_PROP_BAND_1_HIGH_SIDE;
    api->prop->tuner_lo_injection.band_2      = Si2151_TUNER_LO_INJECTION_PROP_BAND_2_LOW_SIDE;
    api->prop->tuner_lo_injection.band_3      = Si2151_TUNER_LO_INJECTION_PROP_BAND_3_LOW_SIDE;
    if (Si2151_L1_SetProperty2(api,Si2151_TUNER_LO_INJECTION_PROP) != 0)
    {
            mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_LO_INJECTION_PROP\n");
    }

    /* Set the Tuner return loss optimize and TF1 boundary if in DTMB mode, otherwise reset it to default values */
    if (eNetworktype == E_Network_Type_DTMB)
    {
        api->prop->tuner_return_loss_optimize.config               =    91;
        api->prop->tuner_return_loss_optimize_2.thld               =    15; /* (default    31) */
        api->prop->tuner_return_loss_optimize_2.window             =     5; /* (default     0) */
        api->prop->tuner_return_loss_optimize_2.engagement_delay   =     3; /* (default    15) */
        api->prop->tuner_tf1_boundary_offset.tf1_boundary_offset   =    22;
    }
    else
    {
        api->prop->tuner_return_loss_optimize.config               = Si2151_TUNER_RETURN_LOSS_OPTIMIZE_PROP_CONFIG_DISABLE;
        api->prop->tuner_return_loss_optimize_2.thld               =    31; /* (default    31) */
        api->prop->tuner_return_loss_optimize_2.window             =     0; /* (default     0) */
        api->prop->tuner_return_loss_optimize_2.engagement_delay   =    15; /* (default    15) */
        api->prop->tuner_tf1_boundary_offset.tf1_boundary_offset   =     0;
    }

    /* set the remaining optimize values to their defaults */
    api->prop->tuner_return_loss_optimize.thld                 =     0;
    api->prop->tuner_return_loss_optimize.engagement_delay     =     7;
    api->prop->tuner_return_loss_optimize.disengagement_delay  =    10;

    if (Si2151_L1_SetProperty2(api, Si2151_TUNER_RETURN_LOSS_OPTIMIZE_PROP) != 0)
    {
        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_RETURN_LOSS_OPTIMIZE_PROP\n");
    }
    if (Si2151_L1_SetProperty2(api, Si2151_TUNER_RETURN_LOSS_OPTIMIZE_2_PROP) != 0)
    {
        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_RETURN_LOSS_OPTIMIZE_2_PROP\n");
    }
    if (Si2151_L1_SetProperty2(api, Si2151_TUNER_TF1_BOUNDARY_OFFSET_PROP) != 0)
    {
        mcSHOW_HW_MSG(" [Silabs]: ERROR_SETTING_PROPERTY Si2151_TUNER_TF1_BOUNDARY_OFFSET_PROP\n");
    }

    switch (eNetworktype)
    {
#if (0) // for DVBT T2 merge FW same tuner setting is needed
       case E_Network_Type_DVBT:
            if ( (previous_dtv_mode != Si2151_DTV_MODE_PROP_MODULATION_DVBT)
                || (previous_agc_speed!=Si2151_DTV_AGC_SPEED_PROP_IF_AGC_SPEED_AUTO)
                || (eBandWidth != p_dev->pre_bw)
                )
                SetIfDemod(eBandWidth, eNetworktype, si_tv_system_invalid);
            break;
       case E_Network_Type_DVBT2:
            if ( (previous_dtv_mode != Si2151_DTV_MODE_PROP_MODULATION_DVBT)
                || (previous_agc_speed!=Si2151_DTV_AGC_SPEED_PROP_IF_AGC_SPEED_39)
                || (eBandWidth != p_dev->pre_bw)
               )
                SetIfDemod(eBandWidth, eNetworktype, si_tv_system_invalid);
            break;
#endif

       case E_Network_Type_DVBT:
       case E_Network_Type_DVBT2:
            if ( (previous_dtv_mode != Si2151_DTV_MODE_PROP_MODULATION_DVBT)
                || (previous_agc_speed!=Si2151_DTV_AGC_SPEED_PROP_IF_AGC_SPEED_AUTO)
                || (eBandWidth != p_dev->pre_bw)
                )
                SetIfDemod(eBandWidth, eNetworktype, si_tv_system_invalid);
            break;
       case E_Network_Type_DVBC:
            if (previous_dtv_mode != Si2151_DTV_MODE_PROP_MODULATION_DVBC)
                SetIfDemod(eBandWidth, eNetworktype, si_tv_system_invalid);
            break;
       case E_Network_Type_ISDBT:
            if (previous_dtv_mode != Si2151_DTV_MODE_PROP_MODULATION_ISDBT)
                SetIfDemod(eBandWidth, eNetworktype, si_tv_system_invalid);
            break;
       case E_Network_Type_ISDBC:
            if (previous_dtv_mode != Si2151_DTV_MODE_PROP_MODULATION_ISDBC)
                SetIfDemod(eBandWidth, eNetworktype, si_tv_system_invalid);
            break;
       case E_Network_Type_DTMB:
            if (previous_dtv_mode != Si2151_DTV_MODE_PROP_MODULATION_DTMB)
                SetIfDemod(eBandWidth, eNetworktype, si_tv_system_invalid);
            break;
       case E_Network_Type_ATSC:
            if (previous_dtv_mode != Si2151_DTV_MODE_PROP_MODULATION_ATSC)
                SetIfDemod(eBandWidth, eNetworktype, si_tv_system_invalid);
            break;
       case E_Network_Type_QAM_US:
            if (previous_dtv_mode != Si2151_DTV_MODE_PROP_MODULATION_QAM_US)
                SetIfDemod(eBandWidth, eNetworktype, si_tv_system_invalid);
            break;
       case E_Network_Type_Analog_T:
       case E_Network_Type_Analog_C:
       case E_Network_Type_DVBS:
       default:
            TUNER_PRINT("[Error]%s,%s,%d\n",__FILE__,__FUNCTION__,__LINE__);
            break;
    }

    if (eMode == E_TUNER_DTV_ATSC_MODE || eMode == E_TUNER_DTV_J83B_MODE)
        freqHz = (U32)(freq * 1);
    else
        freqHz = (U32)(freq * 1000);

    // According to tuner vender info 198.5MHz is a threshold for using another gear of filter. Therefore, for better sensitivity performance give it 1KHz offset.  
    if(freqHz == 198500000)
        freqHz = 198499000;

    if (freqHz < Si2151_TUNER_TUNE_FREQ_CMD_FREQ_MIN || freqHz > Si2151_TUNER_TUNE_FREQ_CMD_FREQ_MAX)
    {
        mcSHOW_HW_MSG(" [Silabs]: <= Frequency out of Range\n");
        return FALSE;
    }

    if (Si2151_L1_TUNER_TUNE_FREQ(api,Si2151_TUNER_TUNE_FREQ_CMD_MODE_DTV,  freqHz) != 0)
    {
        mcSHOW_HW_MSG(" [Silabs]: Error Si2151_L1_TUNER_TUNE_FREQ\n");
        return FALSE;
    }

    /* wait for TUNINT, timeout is 36ms */
    timeout = 36;
    // start_time = MsOS_GetSystemTime();
    while( (timeout--) > 0 )
    {
        if (Si2151_L1_CheckStatus(api) != 0)
            return FALSE;
        if (api->status->tunint)
        {
            retb = TRUE;
            DBG_TUNER(TUNER_PRINT("!!!!!!!!!!!!!LOCK...................\n");)
            break;
        }
        mdelay(1);
    }
    /* wait for DTVINT, timeout is 10ms */
    timeout = 10;
    // start_time = MsOS_GetSystemTime();
    while( (timeout--) > 0 )
    {
        if (Si2151_L1_CheckStatus(api) != 0)
            return FALSE;
        if (api->status->dtvint)
        {
            DBG_TUNER(TUNER_PRINT("!!!!!!!!!!!!!DTV ----------LOCK................\n");)
            retb = TRUE;
            break;
        }
        mdelay(1);
    }

    if (retb == TRUE)
        p_dev->pre_bw = eBandWidth;

    if(isUseEWBSdemod)
    BYPASS_EWBS_CHANNEL(0);
    return retb;
}

int MDrv_TUNER_ExtendCommand(int minor, U8 u8SubCmd, U32 u32Param1, U32 u32Param2, void* pvoidParam3)
{
    DBG_TUNER(TUNER_PRINT("%s, %d, u8SubCmd=%d, u32Param1=%d, u32Param2=%d\n",__FUNCTION__,__LINE__,u8SubCmd,u32Param1,u32Param2);)

    p_dev=&(dev[minor]);

    switch(u8SubCmd)
    {
        case E_TUNER_SUBCMD_GET_FREQ_STEP:
        {
            //fix kasan slab out bounds issue.
            //EN_FREQ_STEP *eFreqStep = (EN_FREQ_STEP*)pvoidParam3;
            U8 *eFreqStep = (U8 *)pvoidParam3;
            *eFreqStep = TN_FREQ_STEP;
        }
            break;

        case E_TUNER_SUBCMD_GET_IF_FREQ:
        {
            U16 *u16IFFreq = (U16 *)pvoidParam3;
            *u16IFFreq = C_Si2151_IF_CENTER_B;
        }
            break;

        case E_TUNER_SUBCMD_GET_L_PRIME_IF_FREQ:
        {
            U16 *u16IFFreq = (U16 *)pvoidParam3;
            *u16IFFreq = C_Si2151_IF_CENTER_L1;
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
            U16 *pu16_vif_notch_coef = NULL;
            U16 *pu16_list = (U16*)pvoidParam3;
            U8 indx = 0;

            switch((EN_VIF_SOUND_SYSTEM)u32Param1)
            {
                case E_VIF_SOUND_B_STEREO_A2:
                    pu16_vif_notch_coef = VIF_A1_A2_SOS11_SOS12_B_Stereo_A2;
                    break;

                case E_VIF_SOUND_B_MONO_NICAM:
                    pu16_vif_notch_coef = VIF_A1_A2_SOS11_SOS12_B_Mono_NICAM;
                    break;

                case E_VIF_SOUND_GH_STEREO_A2:
                    pu16_vif_notch_coef = VIF_A1_A2_SOS11_SOS12_GH_Stereo_A2;
                    break;

                case E_VIF_SOUND_GH_MONO_NICAM:
                    pu16_vif_notch_coef = VIF_A1_A2_SOS11_SOS12_GH_Mono_NICAM;
                    break;

                case E_VIF_SOUND_I:
                    pu16_vif_notch_coef = VIF_A1_A2_SOS11_SOS12_I;
                    break;
                case E_VIF_SOUND_DK1_STEREO_A2:
                    pu16_vif_notch_coef = VIF_A1_A2_SOS11_SOS12_DK1_Stereo_A2;
                    break;

                case E_VIF_SOUND_DK2_STEREO_A2:
                    pu16_vif_notch_coef = VIF_A1_A2_SOS11_SOS12_DK2_Stereo_A2;
                    break;

                case E_VIF_SOUND_DK3_STEREO_A2:
                    pu16_vif_notch_coef = VIF_A1_A2_SOS11_SOS12_DK3_Stereo_A2;
                    break;

                case E_VIF_SOUND_DK_MONO_NICAM:
                    pu16_vif_notch_coef = VIF_A1_A2_SOS11_SOS12_DK_Mono_NICAM;
                    break;

                case E_VIF_SOUND_L:
                    pu16_vif_notch_coef = VIF_A1_A2_SOS11_SOS12_L;
                    break;

                case E_VIF_SOUND_LL:
                    pu16_vif_notch_coef = VIF_A1_A2_SOS11_SOS12_LP;
                    break;

                case E_VIF_SOUND_MN:
                    pu16_vif_notch_coef = VIF_A1_A2_SOS11_SOS12_MN;
                    break;
                default:
                    TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                    return FALSE;
                    break;
            }

            for (indx = 0;indx < 16;indx++)
            {
                *(pu16_list+indx) = *(pu16_vif_notch_coef+indx);
            }

        }
            break;
        case E_TUNER_SUBCMD_GET_DTV_IF_FREQ:
        {
            switch (u32Param1) // demod mode
            {
                case E_TUNER_DTV_DVB_T2_MODE:
                    switch(u32Param2) // bandwidth
                    {
                        case E_RF_CH_BAND_6MHz:
                            *((U32 *)pvoidParam3)=C_Si2151_IF_FREQUENCY;
                            break;
                        case E_RF_CH_BAND_7MHz:
                            *((U32 *)pvoidParam3)=C_Si2151_IF_FREQUENCY;
                            break;
                        case E_RF_CH_BAND_8MHz:
                            *((U32 *)pvoidParam3)=C_Si2151_IF_FREQUENCY;
                            break;
                        default:
                            TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                            return FALSE;
                            break;
                    }
                    break;
                case E_TUNER_DTV_DVB_T_MODE:
                    switch(u32Param2) // bandwidth
                    {
                        case E_RF_CH_BAND_6MHz:
                            *((U32 *)pvoidParam3)=C_Si2151_IF_FREQUENCY;
                            break;
                        case E_RF_CH_BAND_7MHz:
                            *((U32 *)pvoidParam3)=C_Si2151_IF_FREQUENCY;
                            break;
                        case E_RF_CH_BAND_8MHz:
                            *((U32 *)pvoidParam3)=C_Si2151_IF_FREQUENCY;
                            break;
                        default:
                            TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                            return FALSE;
                            break;
                    }
                    break;
                case E_TUNER_DTV_DVB_C_MODE:
                    switch(u32Param2)
                    {
                        case E_RF_CH_BAND_6MHz:
                        case E_RF_CH_BAND_7MHz:
                        case E_RF_CH_BAND_8MHz:
                            *((U32 *)pvoidParam3)=C_Si2151_IF_FREQUENCY;
                            break;
                        default:
                            TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                            return FALSE;
                            break;
                    }
                    break;
                case E_TUNER_DTV_ATSC_MODE:
                case E_TUNER_DTV_DTMB_MODE:
                case E_TUNER_DTV_ISDB_MODE:
                    *((U32 *)pvoidParam3)=C_Si2151_IF_FREQUENCY;
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
            if ((E_TUNER_DTV_ATSC_MODE == u32Param1)||(E_TUNER_DTV_DTMB_MODE == u32Param1))
            {
                *u8IqSwap = 0;
            }
            else
            {
                *u8IqSwap = TUNER_DTV_IQ_SWAP;
                if ( u32Param1 )
                {
                    *u8IqSwap ^= 0x01;
                }
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
            S16 ssi_100dbm = 0;
            Si2151_GetRSSILevel(&ssi_100dbm);
            *((int *)pvoidParam3) = (int)ssi_100dbm;
            return TRUE;
        }
            break;

        case E_TUNER_SUBCMD_GET_DEMOD_CONFIG:
        {

            switch (u32Param1) // demod mode
            {
                case E_TUNER_DTV_DVB_T_MODE:
                    *((U8 **)pvoidParam3)=NULL;
                    break;
                case E_TUNER_DTV_DVB_C_MODE:
                    *((U8 **)pvoidParam3)=NULL;
                    break;
                case E_TUNER_ATV_PAL_MODE:
                case E_TUNER_ATV_SECAM_L_PRIME_MODE:
                case E_TUNER_ATV_NTSC_MODE:
                {
                    U32 *u32AgcOdMode = (U32 *)pvoidParam3;
                    *u32AgcOdMode = TUNER_VIF_TAGC_ODMODE;
                }
                    break;
                case E_TUNER_DTV_DVB_T2_MODE:
                {
                    U8 *u8if_AgcMode = (U8 *)pvoidParam3;
                    *u8if_AgcMode = TUNER_DVB_IF_AGC_MODE;
                }
                  break;
                default:
                    TUNER_PRINT("Error:%s(),%d\n",__func__,__LINE__);
                    return FALSE;
                    break;
            }
        }
            break;

        case E_TUNER_SUBCMD_CONTROL_GPIO_PIN:
        {
            //  u32Param1(Enable bit)    ==>    bit 0: pin1 is enable[1] or disable[0]  ; bit 1: pin2 is enable[1] or disable[0]
            //  u32Param2(Control bit)   ==>    bit 0: pin1 is high[1] or low[0]            ; bit 1: pin1 is high[1] or low[0]
            U8 u8error_code = 0;
            unsigned char gpio1_mode = Si2151_CONFIG_PINS_CMD_GPIO1_MODE_NO_CHANGE;
            unsigned char gpio2_mode = Si2151_CONFIG_PINS_CMD_GPIO2_MODE_NO_CHANGE;

            switch (u32Param1&(u8gpio_control_bit_pin1|u8gpio_control_bit_pin2))
            {
                case 0: // not control any pin
                    TUNER_PRINT(" Not control any pin!!!! \n");
                    return FALSE;
                break;
                case 1: // pin 1 is Enable ; pin2 is Disable
                {
                    gpio1_mode = ((u32Param2&u8gpio_control_bit_pin1) == u8gpio_control_bit_pin1) ? Si2151_CONFIG_PINS_CMD_GPIO1_MODE_DRIVE_1 : Si2151_CONFIG_PINS_CMD_GPIO1_MODE_DRIVE_0;
                    u8error_code = Si2151_L1_CONFIG_PINS(api,
                                          gpio1_mode,
                                          Si2151_CONFIG_PINS_CMD_GPIO1_READ_DO_NOT_READ,
                                          Si2151_CONFIG_PINS_CMD_GPIO2_MODE_NO_CHANGE,
                                          Si2151_CONFIG_PINS_CMD_GPIO2_READ_DO_NOT_READ,
                                          Si2151_CONFIG_PINS_CMD_AGC1_MODE_NO_CHANGE,
                                          Si2151_CONFIG_PINS_CMD_AGC1_READ_DO_NOT_READ,
                                          Si2151_CONFIG_PINS_CMD_AGC2_MODE_NO_CHANGE,
                                          Si2151_CONFIG_PINS_CMD_AGC2_READ_DO_NOT_READ,
                                          Si2151_CONFIG_PINS_CMD_XOUT_MODE_NO_CHANGE);
                }
                break;

                case 2: // pin 1 is Disable ; pin2 is Enable
                {
                    gpio2_mode = ((u32Param2&u8gpio_control_bit_pin2) == u8gpio_control_bit_pin2) ? Si2151_CONFIG_PINS_CMD_GPIO2_MODE_DRIVE_1 : Si2151_CONFIG_PINS_CMD_GPIO2_MODE_DRIVE_0;
                    u8error_code = Si2151_L1_CONFIG_PINS(api,
                                              Si2151_CONFIG_PINS_CMD_GPIO1_MODE_NO_CHANGE,
                                              Si2151_CONFIG_PINS_CMD_GPIO1_READ_DO_NOT_READ,
                                              gpio2_mode,
                                              Si2151_CONFIG_PINS_CMD_GPIO2_READ_DO_NOT_READ,
                                              Si2151_CONFIG_PINS_CMD_AGC1_MODE_NO_CHANGE,
                                              Si2151_CONFIG_PINS_CMD_AGC1_READ_DO_NOT_READ,
                                              Si2151_CONFIG_PINS_CMD_AGC2_MODE_NO_CHANGE,
                                              Si2151_CONFIG_PINS_CMD_AGC2_READ_DO_NOT_READ,
                                              Si2151_CONFIG_PINS_CMD_XOUT_MODE_NO_CHANGE);
                }
                break;

                case 3: // pin 1 is Enable ; pin2 is Enable
                {
                    gpio1_mode = ((u32Param2&u8gpio_control_bit_pin1) == u8gpio_control_bit_pin1) ? Si2151_CONFIG_PINS_CMD_GPIO1_MODE_DRIVE_1 : Si2151_CONFIG_PINS_CMD_GPIO1_MODE_DRIVE_0;
                    gpio2_mode = ((u32Param2&u8gpio_control_bit_pin2) == u8gpio_control_bit_pin2) ? Si2151_CONFIG_PINS_CMD_GPIO2_MODE_DRIVE_1 : Si2151_CONFIG_PINS_CMD_GPIO2_MODE_DRIVE_0;
                    u8error_code = Si2151_L1_CONFIG_PINS(api,
                                              gpio1_mode,
                                              Si2151_CONFIG_PINS_CMD_GPIO1_READ_DO_NOT_READ,
                                              gpio2_mode,
                                              Si2151_CONFIG_PINS_CMD_GPIO2_READ_DO_NOT_READ,
                                              Si2151_CONFIG_PINS_CMD_AGC1_MODE_NO_CHANGE,
                                              Si2151_CONFIG_PINS_CMD_AGC1_READ_DO_NOT_READ,
                                              Si2151_CONFIG_PINS_CMD_AGC2_MODE_NO_CHANGE,
                                              Si2151_CONFIG_PINS_CMD_AGC2_READ_DO_NOT_READ,
                                              Si2151_CONFIG_PINS_CMD_XOUT_MODE_NO_CHANGE);
                }
                break;

                default:
                    TUNER_PRINT("u32Param1 is not allow and u32Param1 =%u \n", u32Param1);
                    return FALSE;
            }
            if (u8error_code != NO_Si2151_ERROR)
            {
                TUNER_PRINT(" error code = %d \n",u8error_code);
                TUNER_PRINT(" Si2151_L1_CONFIG_PINS fail \n");
                return FALSE;
            }
            return TRUE;
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

int MDrv_TUNER_TunerInit(int minor,s_Tuner_dev_info *devTunerInit )
{
    int ret = TRUE;
    SlaveID=devTunerInit->u8_slaveID;
    tuner_i2c_channel_bus = devTunerInit->u8_i2c_bus;
    TUNER_PRINT("%s is invoked _u8SlaveID 0x%x\n", __FUNCTION__,SlaveID);
    SI2151_OEM_I2C_channel_setting(tuner_i2c_channel_bus);
    if(CHECK_EWBS_DEMOD_EXIST(tuner_i2c_channel_bus))
    isUseEWBSdemod=TRUE;
    if (minor < MAX_TUNER_DEV_NUM)
    {
        p_dev=&(dev[minor]);
        p_dev->m_bInATVScanMode = FALSE;
        p_dev->u8_slaveID = SlaveID;
        p_dev->u8_i2c_bus = tuner_i2c_channel_bus;
        if ( (p_dev->e_status == E_CLOSE)
            || (p_dev->e_status == E_SUSPEND) )
        {
            if(device_tuner_si_2151_init()==FALSE)
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
    return 0;
}

int MDrv_TUNER_SetTunerInScanMode(int minor, U32 bScan)
{	
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    p_dev=&(dev[minor]);
    if(p_dev->m_bInATVScanMode != bScan)
    {
        p_dev->m_bInATVScanMode = bScan;
    }
    return TRUE;
}

int MDrv_TUNER_SetTunerInFinetuneMode(int minor, U32 bFinetune)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    p_dev=&(dev[minor]);
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
    return 0;
}

int MDrv_TUNER_IsLocked(int minor)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    return 0;
}

int MDrv_TUNER_GetRSSI(int minor, U16 u16Gain, U8 u8DType)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    S16 strength_100dBm=0;
    Si2151_GetRSSILevel(&strength_100dBm);
    return strength_100dBm;
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

    if (NULL != api) {
        free(api);
        api = NULL;
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
    //MdbPrint(u64ReqHdl,"---------MStar DEMOD Info---------\n\n");
    DBG_TUNER(TUNER_PRINT("---------MStar TUNER Info---------\n"));

  // Printf the Info
    if (MDB_tuner_eNetworktype == TRUE)
    {
        DBG_TUNER(TUNER_PRINT("Tuner Lock Status   : DTV\n"));
        DBG_TUNER(TUNER_PRINT("DTV RF Freq         : %d MHz\n", DTV_FREQ_MDB));
        DBG_TUNER(TUNER_PRINT("DTV IF Freq         : %d KHz\n", api->prop->dtv_lif_freq.offset));
        switch((int)(api->prop->dtv_mode.bw))
        {
            case 6:
                DBG_TUNER(TUNER_PRINT("DTV BW              : %d(6MHz)\n", api->prop->dtv_mode.bw));
                break;
            case 7:
                DBG_TUNER(TUNER_PRINT("DTV BW              : %d(7MHz)\n", api->prop->dtv_mode.bw));
                break;
            case 8:
                DBG_TUNER(TUNER_PRINT("DTV BW              : %d(8MHz)\n", api->prop->dtv_mode.bw));
                break;
            case 9:
                DBG_TUNER(TUNER_PRINT("DTV BW              : %d(1.7MHz)\n", api->prop->dtv_mode.bw));
                break;
            case 10:
                DBG_TUNER(TUNER_PRINT("DTV BW              : %d(6.1MHz)\n", api->prop->dtv_mode.bw));
                break;
            default:
                DBG_TUNER(TUNER_PRINT("DTV BW              : Unknown\n"));
                break;
        }
        switch((int)(api->prop->dtv_mode.modulation))
        {
            case 0:
                DBG_TUNER(TUNER_PRINT("DTV Standard        : %d(ATSC)\n", api->prop->dtv_mode.modulation));
                break;
            case 1:
                DBG_TUNER(TUNER_PRINT("DTV Standard        : %d(QAM_US)\n", api->prop->dtv_mode.modulation));
                break;
            case 2:
                DBG_TUNER(TUNER_PRINT("DTV Standard        : %d(DVBT)\n", api->prop->dtv_mode.modulation));
                break;
            case 3:
                DBG_TUNER(TUNER_PRINT("DTV Standard        : %d(DVBC)\n", api->prop->dtv_mode.modulation));
                break;
            case 4:
                DBG_TUNER(TUNER_PRINT("DTV Standard        : %d(ISDBT)\n", api->prop->dtv_mode.modulation));
                break;
            case 5:
                DBG_TUNER(TUNER_PRINT("DTV Standard        : %d(ISDBC)\n", api->prop->dtv_mode.modulation));
                break;
            case 6:
                DBG_TUNER(TUNER_PRINT("DTV Standard        : %d(DTMB)\n", api->prop->dtv_mode.modulation));
                break;
            default:
                DBG_TUNER(TUNER_PRINT("DTV Standard        : 2(DVBT)\n"));
                break;
        }
        DBG_TUNER(TUNER_PRINT("DTV invert spectrum : %d\n", api->prop->dtv_mode.invert_spectrum));
        DBG_TUNER(TUNER_PRINT("DTV agc speed       : %d\n", api->prop->dtv_agc_speed.if_agc_speed));
        DBG_TUNER(TUNER_PRINT("DTV LIF OUT AMP     : %d\n", api->prop->dtv_lif_out.amp));
        if(api->rsp->tuner_status.rssi != NULL)
            DBG_TUNER(TUNER_PRINT("DTV RSSI            : -%d (dBm)\n", (S16)(api->rsp->tuner_status.rssi)));
        else
            DBG_TUNER(TUNER_PRINT("DTV RSSI            : %d (dBm)\n", (S16)(api->rsp->tuner_status.rssi)));
    }
    else
    {
        DBG_TUNER(TUNER_PRINT("Tuner Lock Status   : ATV\n"));
        DBG_TUNER(TUNER_PRINT("ATV RF Freq         : %d KHz\n", ATV_FREQ_MDB));
        DBG_TUNER(TUNER_PRINT("ATV IF Freq         : %d KHz\n", api->prop->atv_lif_freq.offset));
        switch((int)(api->prop->atv_video_mode.video_sys))
        {
            case 0:
                DBG_TUNER(TUNER_PRINT("ATV Standard        : %d(B)\n", api->prop->atv_video_mode.video_sys));
                break;
            case 1:
                DBG_TUNER(TUNER_PRINT("ATV Standard        : %d(GH)\n", api->prop->atv_video_mode.video_sys));
                break;
            case 2:
                DBG_TUNER(TUNER_PRINT("ATV Standard        : %d(M)\n", api->prop->atv_video_mode.video_sys));
                break;
            case 3:
                DBG_TUNER(TUNER_PRINT("ATV Standard        : %d(N)\n", api->prop->atv_video_mode.video_sys));
                break;
            case 4:
                DBG_TUNER(TUNER_PRINT("ATV Standard        : %d(I)\n", api->prop->atv_video_mode.video_sys));
                break;
            case 5:
                DBG_TUNER(TUNER_PRINT("ATV Standard        : %d(DK)\n", api->prop->atv_video_mode.video_sys));
                break;
            case 6:
                DBG_TUNER(TUNER_PRINT("ATV Standard        : %d(L)\n", api->prop->atv_video_mode.video_sys));
                break;
            case 7:
                DBG_TUNER(TUNER_PRINT("ATV Standard        : %d(LP)\n", api->prop->atv_video_mode.video_sys));
                break;
            default:
                DBG_TUNER(TUNER_PRINT("ATV Standard        : 0(B)\n"));
                break;
        }
        DBG_TUNER(TUNER_PRINT("ATV invert spectrum : %d\n", api->prop->atv_video_mode.invert_spectrum));
        DBG_TUNER(TUNER_PRINT("ATV agc speed       : %d\n", api->prop->atv_agc_speed.if_agc_speed));
        DBG_TUNER(TUNER_PRINT("ATV LIF OUT AMP     : %d\n", api->prop->atv_lif_out.amp));
    }
    return retb;
}

ssize_t MDrv_TUNER_WriteTUNERMdbInfo(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    DBG_TUNER(TUNER_PRINT("%s is invoked\n", __FUNCTION__));
    return 0;
}

#endif




