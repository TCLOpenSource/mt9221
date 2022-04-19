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
#ifndef _MDRV_GFLIP_CFD_H
#define _MDRV_GFLIP_CFD_H

//CFD header
#include "mdrv_video_info_if.h"

//CFD Version define
#define CFD_CONTROL_POINT_ST_VERSION (1)
#define STU_CFDAPI_GOP_PRESDRIP_VERSION (1)

typedef struct _STU_CFDAPI_GOP_PRESDRIP
{
    MS_U32 u32Version;   ///<Version of current structure. Please always set to "CFD_MAIN_CONTROL_ST_VERSION" as input
    MS_U16 u16Length;    ///<Length of this structure, u16Length=sizeof(STU_CFDAPI_Kano_SDRIP)

    //IP2 CSC
    MS_U8 u8CSC_Mode;
    MS_U8 u8CSC_Ratio1;
    MS_U8 u8CSC_Manual_Vars_en;
    MS_U8 u8CSC_MC;

} STU_CFDAPI_GOP_PRESDRIP;

typedef struct _STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP
{

    STU_CFDAPI_GOP_PRESDRIP  stu_GOP_PRESDRIP_Param;

} STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP;

typedef struct _STU_CFDAPI_HW_IPS_GOP
{

    //u8HWGroup is for GOP group ID, starts from 0
    MS_U8 u8HWGroup;

    STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP *pstu_PRESDRIP_Input;

} STU_CFDAPI_HW_IPS_GOP;

typedef struct _STU_CFDAPI_UI_CONTROL_TESTING
{

    MS_U32 u32TestCases;

} STU_CFDAPI_UI_CONTROL_TESTING;

typedef struct _STU_CFDAPI_MAIN_CONTROL_GOP_TESTING
{

    MS_U32 u32TestCases;

} STU_CFDAPI_MAIN_CONTROL_GOP_TESTING;

typedef struct _STU_CFDAPI_TOP_CONTROL_GOP
{
    //share with different HW
    MS_U32 u32Version;
    MS_U16 u16Length;
    STU_CFDAPI_MAIN_CONTROL_GOP      *pstu_Main_Control;
    //STU_CFDAPI_MM_PARSER           *pstu_MM_Param;
    //STU_CFDAPI_HDMI_EDID_PARSER    *pstu_HDMI_EDID_Param;
    STU_CFDAPI_UI_CONTROL            *pstu_UI_Param;
    STU_CFDAPI_PANEL_FORMAT          *pstu_Panel_Param;
    STU_CFDAPI_HW_IPS_GOP            *pstu_HW_IP_Param;
    STU_CFDAPI_GOP_FORMAT            *pstu_GOP_Param;

    STU_CFDAPI_DEBUG                 *pstu_Debug_Param;

} STU_CFDAPI_TOP_CONTROL_GOP;

typedef struct _STU_CFDAPI_PREPROCESS_STATUS_LITE
{
    MS_U32 u32Version;
    MS_U16 u16Length;

    STU_CFD_MS_ALG_COLOR_FORMAT_LITE stu_Cfd_color_format;

}STU_CFDAPI_PREPROCESS_STATUS_LITE;

typedef struct _STU_CFDAPI_GOP_PREPROCESS_OUT
{
    MS_U32 u32Version;
    MS_U16 u16Length;

    STU_CFDAPI_PREPROCESS_STATUS_LITE    stu_Cfd_preprocess_status;

}STU_CFDAPI_GOP_PREPROCESS_OUT;

typedef struct _STU_CFD_GENERAL_CONTROL_GOP
{
    MS_U32 u32Version;   ///<Version of current structure. Please always set to "CFD_MAIN_CONTROL_ST_VERSION" as input
    MS_U16 u16Length;    ///<Length of this structure, u16Length=sizeof(STU_CFD_MAIN_CONTROL)

    //MS_U8 u8InputSource;
    //MS_U8 u8MainSubMode;
    //MS_U8 u8IsRGBBypass;

    MS_U8 u8HWGroupMode;
    MS_U8 u8HWGroup;

    //E_CFD_CFIO_RANGE_LIMIT
    //E_CFD_CFIO_RANGE_FULL
    //MS_U8 u8DoPathFullRange;
    STU_CFDAPI_DEBUG                 stu_Debug_Param;

} STU_CFD_GENERAL_CONTROL_GOP;

typedef struct _STU_CFD_CONTROL_POINT
{
    MS_U32 u32Version;   ///<Version of current structure. Please always set to "CFD_CONTROL_POINT_ST_VERSION" as input
    MS_U16 u16Length;    ///<Length of this structure, u16Length=sizeof(STU_CFD_CONTROL_POINT)
    MS_U8 u8MainSubMode;
    MS_U8 u8Format;
    MS_U8 u8DataFormat;
    MS_U8 u8IsFullRange;
    MS_U8 u8HDRMode;
    MS_U8 u8SDRIPMode;
    MS_U8 u8HDRIPMode;
    MS_U8 u8GamutOrderIdx;
    MS_U8 u8ColorPriamries;
    MS_U8 u8TransferCharacterstics;
    MS_U8 u8MatrixCoefficients;

    MS_U16 u16BlackLevelY;
    MS_U16 u16BlackLevelC;
    MS_U16 u16WhiteLevelY;
    MS_U16 u16WhiteLevelC;

    /// HDR10plus
    MS_U32 u16PanelMaxLuminance;
    MS_U32 u32NormGain;

    // new part
    // for CFD control
    // 0: call Mapi_Cfd_Preprocessing and Mapi_Cfd_Decision
    // 1: call Mapi_Cfd_Preprocessing only
    MS_U8 u8Cfd_process_mode;

    // Process Mode
    // 0: color format driver on - auto
    // CFD process depends on the input of CFD
    // 1: shows SDR UI, UI 709, no video
    MS_U8 u8PredefinedProcess;

    // report Process
    // bit0 DoDLCflag
    // bit1 DoGamutMappingflag
    // bit2 DoTMOflag
    MS_U8 u8Process_Status;
    MS_U8 u8Process_Status2;
    MS_U8 u8Process_Status3;

    MS_U16 u16_check_status;
    STU_CFD_COLORIMETRY stu_Source_Param_Colorimetry;
    STU_CFD_COLORIMETRY stu_Panel_Param_Colorimetry;
} STU_CFD_CONTROL_POINT;

typedef struct
{
    STU_CFD_GENERAL_CONTROL_GOP          *pstu_Cfd_General_Control;
    STU_CFD_CONTROL_POINT                 *pstu_Cfd_Control_Point_Front;
    STU_CFD_CONTROL_POINT                 *pstu_Cfd_Control_Point_End;
    STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP *pstu_SDRIP_Param;
    STU_CFDAPI_UI_CONTROL                 *pstu_UI_control;

} ST_CFD_GOP_PRESDR_INPUT;

typedef struct _STU_Register_Table
{
    MS_U32 u32Depth;
    MS_U32 *pu32Address;
    MS_U16 *pu16Value;
    MS_U16 *pu16Mask;
    MS_U16 *pu16Client;
} STU_Register_Table;

typedef struct _STU_Autodownload_Table
{
    MS_U16 u16client;
    MS_U8 *pu8Data ;
    MS_U32 u32Size;
} STU_Autodownload_Table;

typedef struct _ST_STATUS
{
    MS_U32 u32Version;
    MS_U16 u16Length;
    MS_U16 u16UpdateStatus; // Check whitch IPs need to update
    MS_U8 u8Path;
}ST_STATUS;

typedef struct
{
    MS_U32 u32Version;
    MS_U16 u16Length;
    STU_Register_Table stRegTable;
    STU_Autodownload_Table stAdlTable;
    ST_STATUS stStatus;
}ST_CFD_CSC_HW_OUTPUT;

// Coloect all output
typedef struct
{
    MS_U32 u32Version;
    MS_U16 u16Length;
    STU_Register_Table stRegTable;
    STU_Autodownload_Table stAdlTable;
    ST_STATUS stStatus;
}ST_CFD_GENERAL_HW_OUTPUT;

typedef struct
{
    MS_U32 u32Version;
    MS_U16 u16Length;
    ST_CFD_CSC_HW_OUTPUT     stu_cfd_GOP_PreSDR_CSC_hw_output;
    ST_CFD_GENERAL_HW_OUTPUT stu_cfd_GOP_PreSDR_RGBOffset_hw_output;

} ST_CFD_GOP_PreSDR_HW_OUTPUT;

typedef struct _STU_CFDAPI_TOP_CONTROL_GOP_TESTING
{
    //share with different HW
    MS_U32 u32Version;
    MS_U16 u16Length;

    MS_U8  u8TestEn;

    STU_CFDAPI_MAIN_CONTROL_GOP_TESTING   stu_Main_Control;

    STU_CFDAPI_UI_CONTROL_TESTING         stu_UI_Param;
    //STU_CFDAPI_PANEL_FORMAT_TESTING       stu_Panel_Param;
    //STU_CFDAPI_HW_IPS_GOP_TESTING         stu_HW_IP_Param;
    //STU_CFDAPI_GOP_FORMAT_TESTING         stu_GOP_Param;

} STU_CFDAPI_TOP_CONTROL_GOP_TESTING;

typedef enum _E_CFD_MC_ERR//enum for u8HDR2SDR_Mode/u8SDR_Mode
{
    //Main control starts from 0x0000
    //if error happens, see Mapi_Cfd_inter_Main_Control_Param_Check()
    E_CFD_MC_ERR_NOERR      = 0x0000, //process is ok
    E_CFD_MC_ERR_INPUT_SOURCE    = 0x0001, //input source is over defined range, please check. force input_source to E_CFD_MC_SOURCE_HDMI now!!
    E_CFD_MC_ERR_INPUT_ANALOGIDX = 0x0002, //input analog idx is over defined range, please check. force input analog idx to E_CFD_INPUT_ANALOG_RF_NTSC_44 now!!
    E_CFD_MC_ERR_INPUT_FORMAT = 0x0003, //input format is over defined range, please check. force input format to E_CFD_CFIO_YUV_BT709 now!!

    E_CFD_MC_ERR_INPUT_DATAFORMAT = 0x0004, //input data format is over defined range, please check. force input data format to E_CFD_MC_FORMAT_YUV422 now!!
    E_CFD_MC_ERR_INPUT_ISFULLRANGE = 0x0005, //input data format is over defined range, please check. force MainControl u8Input_IsFullRange to E_CFD_CFIO_RANGE_LIMIT!!

    E_CFD_MC_ERR_INPUT_HDRMODE = 0x0006, //input HDR mode is over defined range, please check. force MainControl u8Input_HDRMode to E_CFIO_MODE_SDR!!
    E_CFD_MC_ERR_INPUT_ISRGBBYPASS = 0X0007,

    E_CFD_MC_ERR_INPUT_SDRIPMODE = 0x0008,
    E_CFD_MC_ERR_INPUT_HDRIPMODE = 0x0009,

    E_CFD_MC_ERR_INPUT_Mid_Format_Mode = 0x000a,
    E_CFD_MC_ERR_INPUT_Mid_Format       = 0x000b,
    E_CFD_MC_ERR_INPUT_Mid_DataFormat   = 0x000c,
    E_CFD_MC_ERR_INPUT_Mid_IsFullRange  = 0x000d,
    E_CFD_MC_ERR_INPUT_Mid_HDRMode      = 0x000e,
    E_CFD_MC_ERR_INPUT_Mid_Colour_primaries = 0x000f,
    E_CFD_MC_ERR_INPUT_Mid_Transfer_Characteristics = 0x0010,
    E_CFD_MC_ERR_INPUT_Mid_Matrix_Coeffs = 0x0011,

    E_CFD_MC_ERR_OUTPUT_SOURCE    = 0x0012,
    E_CFD_MC_ERR_OUTPUT_FORMAT    = 0x0013,

    E_CFD_MC_ERR_OUTPUT_DATAFORMAT  = 0x0014,
    E_CFD_MC_ERR_OUTPUT_ISFULLRANGE = 0x0015,

    E_CFD_MC_ERR_OUTPUT_HDRMODE = 0x0016,

    E_CFD_MC_ERR_HDMIOutput_GammutMapping_Mode = 0x0017,
    E_CFD_MC_ERR_HDMIOutput_GammutMapping_MethodMode = 0x0018,
    E_CFD_MC_ERR_MMInput_ColorimetryHandle_Mode = 0x0019,
    E_CFD_MC_ERR_PanelOutput_GammutMapping_Mode = 0x001a,
    E_CFD_MC_ERR_TMO_TargetRefer_Mode = 0x001b,
    E_CFD_MC_ERR_Target_Max_Luminance = 0x001c,
    E_CFD_MC_ERR_Target_Med_Luminance = 0x001d,
    E_CFD_MC_ERR_Target_Min_Luminance = 0x001e,
    E_CFD_MC_ERR_Source_Max_Luminance = 0x001f,
    E_CFD_MC_ERR_Source_Med_Luminance = 0x0020,
    E_CFD_MC_ERR_Source_Min_Luminance = 0x0021,

    E_CFD_MC_ERR_INPUT_HDRIPMODE_HDRMODE_RULE_VIOLATION = 0X0040,
    E_CFD_MC_ERR_INPUT_FORMAT_DATAFORMAT_NOT_MATCH,
    E_CFD_MC_ERR_INPUT_MAIN_CONTROLS,

    //MM starts from 0x0100
    E_CFD_MC_ERR_MM_Colour_primaries = 0x0100,
    E_CFD_MC_ERR_MM_Transfer_Characteristics = 0x0101,
    E_CFD_MC_ERR_MM_Matrix_Coeffs = 0x0102,
    E_CFD_MC_ERR_MM_IsFullRange = 0x0103,
    E_CFD_MC_ERR_MM_Mastering_Display = 0x0104,
    E_CFD_MC_ERR_MM_Mastering_Display_Coordinates = 0x0105,
    E_CFD_MC_ERR_MM_Handle_Undefined_Case = 0x0106,
    E_CFD_MC_ERR_MM_SEI_Defined = 0x0107,/// HDR10plus
    E_CFD_MC_ERR_MM_SEI_Setting = 0x0108,/// HDR10plus
    E_CFD_MC_ERR_MM_SEI_OverRange = 0x0109,/// HDR10plus

    //HDMI EDID starts from 0x0200
    E_CFD_MC_ERR_HDMI_EDID = 0x0200,
    E_CFD_MC_ERR_HDMI_EDID_Mastering_Display_Coordinates = 0x0201,

    //HDMI InfoFrame starts from 0x0280,
    E_CFD_MC_ERR_HDMI_INFOFRAME = 0x0280,
    E_CFD_MC_ERR_HDMI_INFOFRAME_HDR_Infor = 0x0281,
    E_CFD_MC_ERR_HDMI_INFOFRAME_Mastering_Display_Coordinates = 0x0282,
    E_CFD_MC_ERR_HDMI_INFOFRAME_VSIF_Defined = 0x0283,/// HDR10plus
    E_CFD_MC_ERR_HDMI_INFOFRAME_VSIF_Setting = 0x0284,/// HDR10plus
    E_CFD_MC_ERR_HDMI_INFOFRAME_VSIF_OverRange = 0x0285,/// HDR10plus

    //Panel starts from 0x0300
    E_CFD_MC_ERR_Panel_infor = 0x0300,
    E_CFD_MC_ERR_Panel_infor_Mastering_Display_Coordinates = 0x0301,

    //OSD starts from 0x380
    E_CFD_MC_ERR_UI_infor = 0x0380,

    //HW control starts from 0x390
    E_CFD_MC_ERR_HW_Main_Param = 0x0390,

    //controls for HW
    E_CFD_MC_ERR_HW_IPS_PARAM_OVERRANGE = 0x03A0, //parameters for HW IP control
    E_CFD_MC_ERR_HW_IPS_PARAM_HDR_OVERRANGE = 0x03A1, //parameters for HW IP control
    E_CFD_MC_ERR_HW_IPS_PARAM_SDR_OVERRANGE = 0x03A2, //parameters for HW IP control
    E_CFD_MC_ERR_HW_IPS_PARAM_TMO_OVERRANGE = 0x03A3, //parameters for HW IP control
    E_CFD_MC_ERR_HW_IPS_PARAM_DLC_OVERRANGE = 0x03A4, //parameters for HW IP control

    //interrelation of API
    E_CFD_MC_ERR_INPUT_MAIN_CONTROLS_MM_INTERRELATION,
    E_CFD_MC_ERR_INPUT_MAIN_CONTROLS_HDMI_INTERRELATION,
    E_CFD_MC_ERR_OUTPUT_PANEL_SDRIPMODE_INTERRELATION,

    //HW IP Capability
    E_CFD_MC_ERR_HW_IPS_BT2020CLtoNCL_NOTSUPPORTED = 0x0400,
    E_CFD_MC_ERR_HW_IPS_GMforXVYCC_NOTSUPPORTED = 0x0401,
    E_CFD_MC_ERR_HW_IPS_SDR2HDR_NOTSUPPORTED = 0x0402, //no SDR to HDR now
    E_CFD_MC_ERR_HW_IPS_HDRXtoHDRY_NOTSUPPORTED = 0x0403,
    E_CFD_MC_ERR_HW_IPS_GM_NOTSUPPORTED = 0x0404,
    E_CFD_MC_ERR_HW_IPS_TMO_NOTSUPPORTED = 0x0404,

    //
    //E_CFD_MC_ERR_BYPASS       , //STB/TV can not handle this case, so force all IP bypass
    //E_CFD_MC_ERR_CONTROLINPUT,//something wrong in setting for STU_CFDAPI_MAIN_CONTROL, please check settings
    //E_CFD_MC_ERR_CONTROL_MM_INPUT , ////something wrong in parameters of structures of main control and MM
    //E_CFD_MC_ERR_MMINPUT_FORCE709   , //force input_format to BT709, some information is not avaliable for MM
    //E_CFD_MC_ERR_MMINPUT0   ,
    //E_CFD_MC_ERR_HDMIINPUT  , //this function can not support current HDMI input
    //E_CFD_MC_ERR_ANALOGINPUT , //this function can not support current Analog input
    //E_CFD_MC_ERR_WRONGINPUT  , //this function can not support current input
    //E_CFD_MC_ERR_WRONGINPUTSOURCE  ,//can not support current input source, check the value of u8Input_Source
    E_CFD_MC_ERR_WRONGOUTPUTSOURCE  ,//can not support current output source, check the value of u8Output_Source
    E_CFD_MC_ERR_PROCESSOFF   , //process off
    //E_CFD_MC_ERR_NOSDR2HDRNOW , //no SDR to HDR now, check u8Input_HDRMode & u8Output_HDRMode
    //E_CFD_MC_ERR_WRONGTMOSET  , //not support such TMO , check u8Input_HDRMode & u8Output_HDRMode
    //E_CFD_MC_ERR_PARTIALWRONG , //STB/TV can not handle this case, so force all IP bypass
    //E_CFD_MC_ERR_HW_NOT_SUPPORT_THIS_INPUT, //for current HW, this input format can not supported
    //E_CFD_MC_ERR_HW_NOT_SUPPORT_THIS_OUTPUT, //for current HW, this input format can not supported

    //HW IP process function from Ali
    E_CFD_MC_ERR_HW_IP_PARAMETERS, //check the parameters from CFD to IP process functions from Ali

    //E_CFD_MC_ERR_DLCIP_PARAMETERS, //need to check the parameters for DLC IPs, setting is out of range
    //E_CFD_MC_ERR_0x1001_ERR_IN_Input_Analog_SetConfigures = 0x1001, //some errors happens in Input_Analog_SetConfigures(), please debug

    E_CFD_ERR_PARAM_OVERRANGE,
    E_CFD_ERR_NULLPOINTER,

    E_CFD_MC_ERR_EMUEND

} E_CFD_API_Status; //E_CFD_MC_ERR;

typedef struct{
    MS_U8 u8BlackLevel;
    MS_U8 u8WhiteLevel;
}ST_KDRV_XC_CFD_BLACKANDWHITE;

#endif  //_MDRV_GFLIP_CFD_H
