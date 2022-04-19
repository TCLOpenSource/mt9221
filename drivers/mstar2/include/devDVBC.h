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

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// file    devDVBC.h
/// @brief  DVBC deModulator device driver Interface
/// @author MStar Semiconductor,Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _DEV_DVBC_H_
#define _DEV_DVBC_H_

//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------

#ifndef MDEV_MAJOR_DVBC
#define MDEV_MAJOR_DVBC             0                                   // device major number
#endif

#define DEVDVBC_DEV_NUM             1                                   // number of device

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------

/* Use 'd' as magic number */
#define DEVDVBC_IOC_MAGIC           'd'
#define DEVDVBC_IOC_RESET           _IO(DEVDVBC_IOC_MAGIC,      0)
#define DEVDVBC_IOC_RESTART         _IOW(DEVDVBC_IOC_MAGIC,     1,  int)
#define DEVDVBC_IOC_SETMODE         _IOW(DEVDVBC_IOC_MAGIC,     2,  int)
#define DEVDVBC_IOC_SETBW           _IOW(DEVDVBC_IOC_MAGIC,     3,  int)
#define DEVDVBC_IOC_SETTSOUT        _IOW(DEVDVBC_IOC_MAGIC,     4,  int)
#define DEVDVBC_IOC_SETPOWER        _IOW(DEVDVBC_IOC_MAGIC,     5,  int)
#define DEVDVBC_IOC_GETPARAM        _IOR(DEVDVBC_IOC_MAGIC,     6,  int)
#define DEVDVBC_IOC_GETLOCK         _IOR(DEVDVBC_IOC_MAGIC,     7,  int)
#define DEVDVBC_IOC_GETNOISEPOWER   _IOR(DEVDVBC_IOC_MAGIC,     8,  int)
#define DEVDVBC_IOC_GETWINDOWLEN    _IOR(DEVDVBC_IOC_MAGIC,     9,  int)
#define DEVDVBC_IOC_GETERRORBIT     _IOR(DEVDVBC_IOC_MAGIC,     10, int)
#define DEVDVBC_IOC_GETRFGAIN       _IOR(DEVDVBC_IOC_MAGIC,     11, int)
#define DEVDVBC_IOC_GETIFGAIN       _IOR(DEVDVBC_IOC_MAGIC,     12, int)
#define DEVDVBC_IOC_GETBW           _IOR(DEVDVBC_IOC_MAGIC,     13, int)

#define DEVDVBC_IOC_MAXNR           14

#define DEVDVBC_STATE_NUM           11

#define DEVDVBC_OK                  0
#define DEVDVBC_FAIL                -EFAULT



//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

typedef enum
{
    pc_agc_k_RstMixerAGCTR,
    pc_dagc_hum_k_RstMixerAGCTR,
    pc_dagc_hum_k_RstEQFEC,
    pc_iqb_mup_mug_RstEQth,
    p_eq_fmax_32_128_RstEQth,
    p_eq_fmax_16_64_256_RstEQth,
    pc_eq_ki_kp0_32_128_RstEQth,
    pc_eq_ki_kp1_32_128_RstEQth,
    pc_eq_ki_kp2_32_128_RstEQth,
    pc_eq_ki_kp3_32_128_RstEQth,
    pc_eq_ki_kp4_32_128_RstEQth,
    pc_eq_ki_kp0_16_64_256_RstEQth,
    pc_eq_ki_kp1_16_64_256_RstEQth,
    pc_eq_ki_kp2_16_64_256_RstEQth,
    pc_eq_ki_kp3_16_64_256_RstEQth,
    pc_eq_ki_kp4_16_64_256_RstEQth,
    pc_agc_k_SetReg,
    p_agc_ref_SetReg,
    pc_AGC_cnt_cycle_SetReg,
    p_RF_gain_min_L_SetReg,
    p_RF_gain_min_H_SetReg,
    p_RF_gain_max_L_SetReg,
    p_RF_gain_max_H_SetReg,
    p_IF_gain_min_L_SetReg,
    p_IF_gain_min_H_SetReg,
    p_IF_gain_max_L_SetReg,
    p_IF_gain_max_H_SetReg,
    p_hum_err_th_SetReg,
    p_hum_det_lim_SetReg,
    pc_hum_cnt_max_SetReg,
    p_dagc_hum_ref_SetReg,
    p_dagc_hum_gain_set_value_SetReg,
    pc_dagc_hum_k_SetReg,
    p_IIS_Blank_th_SetReg,
    p_IIS_Trig_th_SetReg,
    pc_IIS_maxdel_sel_SetReg,
    pc_tr_ki_kp_SetReg,
    p_Fsync_search_sync_period_SetReg,
    p_Fsync_th_found_L_SetReg,
    p_Fsync_th_lock_L_SetReg,
    p_Fsync_max_hit_count_L_SetReg,
    p_Fsync_max_hit_count_H_SetReg,
    p_bit_err_cnt_peroid_H_SetReg,
    pc_hum_cnt_max_S0,
    pc_hum_cnt_max_S1,
    p_iqb_enable_S4,
    p_erasure_enable_S4,
    pc_agc_k_S9,
    pc_eq_cr_ki_kp4_S10_0,
    pc_eq_cr_ki_kp4_S10_1,
    pc_iqb_mup_mug_S10,
} DvbcParmType;


typedef enum _DVBC_QamType
{
    E_DVBC_QAM16  = 0,                                                  ///< QAM 16
    E_DVBC_QAM32  = 1,                                                  ///< QAM 32
    E_DVBC_QAM64  = 2,                                                  ///< QAM 64
    E_DVBC_QAM128 = 3,                                                  ///< QAM 128
    E_DVBC_QAM256 = 4,                                                  ///< QAM 256

    E_DVBC_QAM_LASTNUM,
} __attribute__((packed)) DVBC_QAMType;

/// Demodulation IQ type
typedef enum _DVBC_IQType
{
    E_DVBC_IQ_NORMAL = 0,                                               ///< IQ Normal
    E_DVBC_IQ_INVERT = 1,                                               ///< IQ Invert
} __attribute__((packed)) DVBC_IQType;

/// Demodulation tap assign
typedef enum _DVBC_TapAssign
{
    E_DVBC_TAP_PRE  = 0,                                                ///< PreTap assign
    E_DVBC_TAP_POST = 1,                                                ///< PostTap assign
} __attribute__((packed)) DVBC_TapAssign;


/// Demodulator auto tune control
typedef struct
{
    // Demodulator option
    MS_BOOL                            bX4CFE_en;                          ///< Carrier frequency estimation
    MS_BOOL                            bPPD_en;                            ///< Tap assign estimation
    MS_BOOL                            bIQAutoSwap_en;                     ///< IQ mode auto swap
    MS_BOOL                            bQAMScan_en;                        ///< QAM type auto scan
    MS_BOOL                            bFHO_en;                            ///< FHO
} DVBC_Mode;

/// Demodulator tune parameter
typedef struct
{
    // Channel setting
    MS_U32                             u32SymRate;                         ///< Symbol rate (Ksym/sec)

    // Channel characteristic
    DVBC_QAMType                    eQamType;                           ///< QAM type
    DVBC_IQType                     eIQSwap;                            ///< IQ type
    DVBC_TapAssign                  eTapAssign;                         ///< Tap assign
    MS_U32                             u32FreqOffset;                      ///< Carrier frequency offset
} DVBC_Param;


//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------


#endif // _DEV_DVBC_H_

