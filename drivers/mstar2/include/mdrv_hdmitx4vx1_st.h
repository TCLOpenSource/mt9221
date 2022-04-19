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

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// @file   mdrv_hdmitx4vx1_st.h
// @brief  hdmitx4vx1 Driver Interface
// @author MStar Semiconductor Inc.
//////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _MDRV_HDMITX4VX1_ST_H
#define _MDRV_HDMITX4VX1_ST_H

//=============================================================================
// Includs
//=============================================================================

//=============================================================================
// Type and Structure Declaration
//=============================================================================
#define BSTATUS_SIZE            2       //unit: byte
#define MAX_DEVICE_NUM          12      //Maximum support device number
#define DEVICE_KSV_LIST_SIZE    5       //unit: byte. Each device has 5Bytes data for its KSV
#define MAX_KSV_LIST_SIZE       (MAX_DEVICE_NUM*DEVICE_KSV_LIST_SIZE)      //unit: byte

/// SW/HW LOCKN
typedef enum
{
    E_HDMITX4VX1_HW_LOCKN = 0,
    E_HDMITX4VX1_SW_LOCKN = 1,
} EN_HDMITX4VX1_LOCKN_TYPE; // HDMITX4VX1 lockn type index


/// Mailbox Command Type Index
typedef enum
{
    E_MAILBOX_CONFIQ_VIDEO        = 0x01,
    E_MAILBOX_CONFIQ_AUDIO        = 0x02,
    E_MAILBOX_AVMUTE_TIMING_RESET = 0x03,
    E_MAILBOX_QUERY_EDID_INFO     = 0x04,
    E_MAILBOX_REPORT_EDID_INFO    = 0x05,
    E_MAILBOX_ACK_STATUS_RESPOND  = 0x06,
    E_MAILBOX_DEBUG               = 0x07,
    E_MAILBOX_COMBINE_COMMAND     = 0x08,
    E_MAILBOX_HDCP_COMMAND        = 0x09,
} EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE; // mailbox command type

/// Mailbox Timing Type
typedef enum
{
    E_HDMITX4VX1_TIMING_720x576p_50Hz   = 0x00,
    E_HDMITX4VX1_TIMING_720x480p_60Hz   = 0x01,
    E_HDMITX4VX1_TIMING_1280x720p_50Hz  = 0x02,
    E_HDMITX4VX1_TIMING_1280x720p_60Hz  = 0x03,
    E_HDMITX4VX1_TIMING_1920x1080p_24Hz = 0x04,
    E_HDMITX4VX1_TIMING_1920x1080i_50Hz = 0x05,
    E_HDMITX4VX1_TIMING_1920x1080i_60Hz = 0x06,
    E_HDMITX4VX1_TIMING_1920x1080p_50Hz = 0x07,
    E_HDMITX4VX1_TIMING_1920x1080p_60Hz = 0x08,
    E_HDMITX4VX1_TIMING_4K2Kp_25Hz      = 0x09,
    E_HDMITX4VX1_TIMING_4K2Kp_30Hz      = 0x0A,
    E_HDMITX4VX1_TIMING_4K2Kp_50Hz      = 0x0B,
    E_HDMITX4VX1_TIMING_4K2Kp_60Hz      = 0x0C,
    E_HDMITX4VX1_TIMING_4096x2160_50Hz  = 0x0D,
    E_HDMITX4VX1_TIMING_4096x2160_60Hz  = 0x0E,
    E_HDMITX4VX1_TIMING_4K2Kp_24Hz      = 0x0F,
    E_HDMITX4VX1_TIMING_4096x2160p_24Hz = 0x10,
    E_HDMITX4VX1_TIMING_4096x2160p_25Hz = 0x11,
    E_HDMITX4VX1_TIMING_4096x2160p_30Hz = 0x12,
    E_HDMITX4VX1_TIMING_800x600p_60Hz   = 0x80,
    E_HDMITX4VX1_TIMING_848x480p_60Hz   = 0x81,
    E_HDMITX4VX1_TIMING_1024x768p_60Hz  = 0x82,
    E_HDMITX4VX1_TIMING_1280x768p_60Hz  = 0x83,
    E_HDMITX4VX1_TIMING_1280x800p_60Hz  = 0x84,
    E_HDMITX4VX1_TIMING_1280x960p_60Hz  = 0x85,
    E_HDMITX4VX1_TIMING_1280x1024p_60Hz = 0x86,
    E_HDMITX4VX1_TIMING_1360x768p_60Hz  = 0x87,
    E_HDMITX4VX1_TIMING_1366x768p_60Hz  = 0x88,
    E_HDMITX4VX1_TIMING_1400x1050p_60Hz = 0x89,
    E_HDMITX4VX1_TIMING_1440x900p_60Hz  = 0x8A,
    E_HDMITX4VX1_TIMING_1600x900p_60Hz  = 0x8B,
    E_HDMITX4VX1_TIMING_1600x1200p_60Hz = 0x8C,
    E_HDMITX4VX1_TIMING_1680x1050p_60Hz = 0x8D,
    E_HDMITX4VX1_TIMING_1920x1200p_60Hz = 0x8E,
    E_HDMITX4VX1_TIMING_2048x1152p_60Hz = 0x8F,
    E_HDMITX4VX1_TIMING_MAX             = 0x90,
} EN_HDMITX4VX1_VIDEO_TIMING; // HDMITX4VX1 timing type index

/// Mailbox Color Depth Type
typedef enum
{
    E_HDMITX4VX1_COLORS_NOID   = 0,
    E_HDMITX4VX1_COLORS_8bits  = 1,
    E_HDMITX4VX1_COLORS_10bits = 2,
    E_HDMITX4VX1_COLORS_12bits = 3,
    E_HDMITX4VX1_COLORS_16bits = 4,
} EN_HDMITX4VX1_COLOR_DEPTH; // HDMITX4VX1 color type index

/// Mailbox Video Output Mode
typedef enum
{
    E_HDMITX4VX1_VIDEO_OUTPUT_DVI       = 0,
    E_HDMITX4VX1_VIDEO_OUTPUT_DVI_HDCP  = 1,
    E_HDMITX4VX1_VIDEO_OUTPUT_HDMI      = 2,
    E_HDMITX4VX1_VIDEO_OUTPUT_HDMI_HDCP = 3,
} EN_HDMITX4VX1_VIDEO_OUTPUT_TYPE; // HDMITX4VX1 video output type index

/// Mailbox Input Color Type
typedef enum
{
    E_HDMITX4VX1_COLORS_INPUT_RGB = 0,
    E_HDMITX4VX1_COLORS_INPUT_YUV = 1,
} EN_HDMITX4VX1_INPUT_COLOR_TYPE; // HDMITX4VX1 input color type index

/// Mailbox Output Color Type
typedef enum
{
    E_HDMITX4VX1_COLORS_OUTPUT_RGB_444 = 0,
    E_HDMITX4VX1_COLORS_OUTPUT_YUV_422 = 1,
    E_HDMITX4VX1_COLORS_OUTPUT_YUV_444 = 2,
    E_HDMITX4VX1_COLORS_OUTPUT_YUV_420 = 3,
} EN_HDMITX4VX1_OUTPUT_COLOR_TYPE; // HDMITX4VX1 output color type index

/// Mailbox 3D VS Information Type
typedef enum
{
    E_HDMITX4VX1_VIDEO_3D_FramePacking     = 0,
    E_HDMITX4VX1_VIDEO_3D_FieldAlternative = 1,
    E_HDMITX4VX1_VIDEO_3D_LineAlternative  = 2,
    E_HDMITX4VX1_VIDEO_3D_SidebySide_FULL  = 3,
    E_HDMITX4VX1_VIDEO_3D_L_Dep            = 4,
    E_HDMITX4VX1_VIDEO_3D_L_Dep_Graphic_Dep= 5,
    E_HDMITX4VX1_VIDEO_3D_TopandBottom     = 6,
    E_HDMITX4VX1_VIDEO_3D_SidebySide_Half  = 8,
    E_HDMITX4VX1_VIDEO_3D_Not_in_Use       = 15,
} EN_HDMITX4VX1_VIDEO_3D_STRUCTURE_TYPE; // HDMITX4VX1 3D VS info type index

/// Mailbox Setting Video HDCP Type
typedef enum
{
    E_HDMITX4VX1_VIDEO_HDCP_14 = 0,
    E_HDMITX4VX1_VIDEO_HDCP_22 = 1,
} EN_HDMITX4VX1_VIDEO_HDCP_TYPE; // HDMITX4VX1 hdcp type index


/// Mailbox 4K2K VS Information Type
typedef enum
{
    E_HDMITX4VX1_VIDEO_4K2K_Reserved    = 0, // 0x00
    E_HDMITX4VX1_VIDEO_4K2K_30Hz        = 1, // 0x01
    E_HDMITX4VX1_VIDEO_4K2K_25Hz        = 2, // 0x02
    E_HDMITX4VX1_VIDEO_4K2K_24Hz        = 3, // 0x03
    E_HDMITX4VX1_VIDEO_4K2K_24Hz_SMPTE  = 4, // 0x04
} EN_HDMITX4VX1_VIDEO_4K2K_VIC;// HDMITX4VX1 4K2K VS info type index


/// Mailbox Audio Frequency Type
typedef enum
{
    E_HDMITX4VX1_AUDIO_32KHz   = 0, // 0x00
    E_HDMITX4VX1_AUDIO_44KHz   = 1, // 0x01
    E_HDMITX4VX1_AUDIO_48KHz   = 2, // 0x02
    E_HDMITX4VX1_AUDIO_88KHz   = 3, // 0x03
    E_HDMITX4VX1_AUDIO_96KHz   = 4, // 0x04
    E_HDMITX4VX1_AUDIO_176KHz  = 5, // 0x05
    E_HDMITX4VX1_AUDIO_192KHz  = 6, // 0x06
} EN_HDMITX4VX1_AUDIO_FREQUENCY_TYPE;// HDMITX4VX1 audio frequency index

/// Mailbox Audio Channel Number
typedef enum
{
    E_HDMITX4VX1_AUDIO_CH_2    = 2, // 2 channels
    E_HDMITX4VX1_AUDIO_CH_8    = 8, // 8 channels
} EN_HDMITX4VX1_AUDIO_CHANNEL_COUNT;// HDMITX4VX1 audio channel number index

/// Mailbox Audio Code Type
typedef enum
{
    E_HDMITX4VX1_AUDIO_PCM     = 0, // PCM
    E_HDMITX4VX1_AUDIO_NONPCM  = 1, // non-PCM
} EN_HDMITX4VX1_AUDIO_CODING_TYPE;// HDMITX4VX1 audio code type index

/// Mailbox Audio Channel Number
typedef enum
{
    E_HDMITX4VX1_AUDIO_SPDIF   = 0, // SPDIF
    E_HDMITX4VX1_AUDIO_I2S     = 1, // I2S
} EN_HDMITX4VX1_AUDIO_SOURCE_TYPE;// HDMITX4VX1 audio source type index

/// Mailbox Audio Channel Number
typedef enum
{
    E_HDMITX4VX1_AUDIO_FORMAT_PCM   = 0, // PCM
    E_HDMITX4VX1_AUDIO_FORMAT_DSD   = 1, // DSD
    E_HDMITX4VX1_AUDIO_FORMAT_HBR   = 2, // HBR
} EN_HDMITX4VX1_AUDIO_SOURCE_FORMAT;// HDMITX4VX1 audio source format index

/// Mailbox Video Mute
typedef enum
{
    E_HDMITX4VX1_VIDEO_MUTE_DISABLE   = 0, // disable
    E_HDMITX4VX1_VIDEO_MUTE_ENABLE    = 1, // enable
} EN_HDMITX4VX1_VIDEO_MUTE;// HDMITX4VX1 video mute index

/// Mailbox Audio Mute
typedef enum
{
    E_HDMITX4VX1_AUDIO_MUTE_DISABLE   = 0, // disable
    E_HDMITX4VX1_AUDIO_MUTE_ENABLE    = 1, // enable
} EN_HDMITX4VX1_AUDIO_MUTE;// HDMITX4VX1 audio mute index

/// Mailbox Audio&Video Mute
typedef enum
{
    E_HDMITX4VX1_AVMUTE_DISABLE   = 0, // disable
    E_HDMITX4VX1_AVMUTE_ENABLE    = 1, // enable
} EN_HDMITX4VX1_AVMUTE;// HDMITX4VX1 audio and video mute index

/// Mailbox Timing Change
typedef enum
{
    E_HDMITX4VX1_TIMING_CHANGE_DISABLE   = 0, // disable
    E_HDMITX4VX1_TIMING_CHANGE_ENABLE    = 1, // enable
} EN_HDMITX4VX1_TIMING_CHANGE;// HDMITX4VX1 timing reset index

/// Mailbox HDMITX Power On/Off
typedef enum
{
    E_HDMITX4VX1_POWER_OFF   = 0, // Off
    E_HDMITX4VX1_POWER_ON    = 1, // On
} EN_HDMITX4VX1_POWER_CHANGE;// HDMITX4VX1 hdmitx power index


/// Mailbox Query Type
typedef enum
{
    E_HDMITX4VX1_QUERY_4K2K_VIC             = 0x01, // 0x01
    E_HDMITX4VX1_QUERY_3D_STRUCTURE         = 0x02, // 0x02
    E_HDMITX4VX1_QUERY_AUDIO_DESCRIPTOR_NUM = 0x03, // 0x03
    E_HDMITX4VX1_QUERY_AUDIO_DESCRIPTOR     = 0x04, // 0x04
    E_HDMITX4VX1_QUERY_VIDEO_DESCRIPTOR_NUM = 0x05, // 0x05
    E_HDMITX4VX1_QUERY_VIDEO_DESCRIPTOR     = 0x06, // 0x06
    E_HDMITX4VX1_QUERY_EDID_DATA            = 0x07, // 0x07
    E_HDMITX4VX1_QUERY_RX_EDID_INFO         = 0x08, // 0x08
    E_HDMITX4VX1_QUERY_COLOR_FORMAT         = 0x09, // 0x09
    E_HDMITX4VX1_QUERY_HW_INFO              = 0x0A, // 0x0A
    E_HDMITX4VX1_QUERY_KSV_BSTATUS          = 0x0B, // 0x0B
    E_HDMITX4VX1_QUERY_HDCPKEY_STATUS       = 0x0C, // 0x0C
} EN_HDMITX4VX1_QUERY_TYPE;// HDMITX4VX1 command 0x04 0x05 query type index


/// Mailbox 3D Timing Type for Command 0x04 Query
typedef enum
{
    E_HDMITX4VX1_QUERY_3DTIMING_1280x720p_50Hz  = 0x00,
    E_HDMITX4VX1_QUERY_3DTIMING_1280x720p_60Hz  = 0x01,
    E_HDMITX4VX1_QUERY_3DTIMING_1920x1080i_50Hz = 0x02,
    E_HDMITX4VX1_QUERY_3DTIMING_1920x1080i_60Hz = 0x03,
    E_HDMITX4VX1_QUERY_3DTIMING_1920x1080p_24Hz = 0x04,
    E_HDMITX4VX1_QUERY_3DTIMING_1920x1080p_25Hz = 0x05,
    E_HDMITX4VX1_QUERY_3DTIMING_1920x1080p_50Hz = 0x06,
    E_HDMITX4VX1_QUERY_3DTIMING_1920x1080p_60Hz = 0x07,
    E_HDMITX4VX1_QUERY_3DTIMING_4K2Kp_25Hz      = 0x08,
    E_HDMITX4VX1_QUERY_3DTIMING_4K2Kp_30Hz      = 0x09,
    E_HDMITX4VX1_QUERY_3DTIMING_4K2Kp_50Hz      = 0x0A,
    E_HDMITX4VX1_QUERY_3DTIMING_4K2Kp_60Hz      = 0x0B,
    E_HDMITX4VX1_QUERY_3DTIMING_4096x2160p_25Hz = 0x0C,
    E_HDMITX4VX1_QUERY_3DTIMING_4096x2160p_30Hz = 0x0D,
    E_HDMITX4VX1_QUERY_3DTIMING_4096x2160p_50Hz = 0x0E,
    E_HDMITX4VX1_QUERY_3DTIMING_4096x2160p_60Hz = 0x0F,
    E_HDMITX4VX1_QUERY_3DTIMING_1920x1080p_30Hz = 0x10,
    E_HDMITX4VX1_QUERY_3DTIMING_4K2Kp_24Hz      = 0x11,
    E_HDMITX4VX1_QUERY_3DTIMING_4096x2160p_24Hz = 0x12,
    E_HDMITX4VX1_QUERY_3DTIMING_MAX             = 0x13,
} EN_HDMITX4VX1_QUERY_3DTIMING; // HDMITX4VX1 3D timing type index for command 0x04 query & 0x05 report

/// Mailbox Color Format Timing Type for Command 0x04 Query
typedef enum
{
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_720x576p_50Hz   = 0x00,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_720x480p_60Hz   = 0x01,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_1280x720p_50Hz  = 0x02,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_1280x720p_60Hz  = 0x03,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_1920x1080p_24Hz = 0x04,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_1920x1080i_50Hz = 0x05,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_1920x1080i_60Hz = 0x06,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_1920x1080p_50Hz = 0x07,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_1920x1080p_60Hz = 0x08,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_4K2Kp_25Hz      = 0x09,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_4K2Kp_30Hz      = 0x0A,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_4K2Kp_50Hz      = 0x0B,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_4K2Kp_60Hz      = 0x0C,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_4096x2160p_50Hz = 0x0D,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_4096x2160p_60Hz = 0x0E,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_1920x1080p_25Hz = 0x0F,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_1920x1080p_30Hz = 0x10,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_4K2Kp_24Hz      = 0x11,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_4096x2160p_24Hz = 0x12,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_4096x2160p_25Hz = 0x13,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_4096x2160p_30Hz = 0x14,
    E_HDMITX4VX1_QUERY_COLORFMTTIMING_MAX             = 0x15,
} EN_HDMITX4VX1_QUERY_COLORFMTTIMING; // HDMITX4VX1 3D timing type index for command 0x04 query & 0x05 report


/// Mailbox 3D VS Information Type for Command 0x05 Report
typedef enum
{
    E_HDMITX4VX1_VIDEO_REPORT_3D_FramePacking               = 0x0001,
    E_HDMITX4VX1_VIDEO_REPORT_3D_FieldAlternative           = 0x0002,
    E_HDMITX4VX1_VIDEO_REPORT_3D_LineAlternative            = 0x0004,
    E_HDMITX4VX1_VIDEO_REPORT_3D_SidebySide_FULL            = 0x0008,
    E_HDMITX4VX1_VIDEO_REPORT_3D_L_Dep                      = 0x0010,
    E_HDMITX4VX1_VIDEO_REPORT_3D_L_Dep_Graphic_Dep          = 0x0020,
    E_HDMITX4VX1_VIDEO_REPORT_3D_TopandBottom               = 0x0040,
    E_HDMITX4VX1_VIDEO_REPORT_3D_SidebySide_Half_Horizontal = 0x0100,
    E_HDMITX4VX1_VIDEO_REPORT_3D_SidebySide_Half_Quincunx   = 0x8000,
} EN_HDMITX4VX1_VIDEO_REPORT_3D_STRUCTURE_TYPE; // HDMITX4VX1 3D VS info type index

/// Mailbox Report Color Depth Info
typedef enum
{
    E_HDMITX4VX1_COLOR_DEPTH_NOID   = 0,
    E_HDMITX4VX1_COLOR_DEPTH_24bits = 4,
    E_HDMITX4VX1_COLOR_DEPTH_30bits = 5,
    E_HDMITX4VX1_COLOR_DEPTH_36bits = 6,
    E_HDMITX4VX1_COLOR_DEPTH_48bits = 7
} EN_HDMITX4VX1_COLOR_DEPTH_INFO; // HDMITX4VX1 color type index

/// Mailbox Status Response
typedef enum
{
    E_MAILBOX_REPORT_CONFIQ_VIDEO        = 0x01,
    E_MAILBOX_REPORT_CONFIQ_AUDIO        = 0x02,
    E_MAILBOX_REPORT_AVMUTE_TIMING_RESET = 0x03,
    E_MAILBOX_REPORT_REPORT_EDID_INFO    = 0x04,
    E_MAILBOX_REPORT_DEBUG               = 0x07,
} EN_HDMITX4VX1_MAILBOX_RECIEVED_COMMAND_TYPE; // mailbox received command type

typedef enum
{
    E_MAILBOX_REPORT_SUCCESS           = 0x00,
    E_MAILBOX_REPORT_FAIL              = 0x01,
    E_MAILBOX_REPORT_PARAMETER_INVALID = 0x02,
    E_MAILBOX_REPORT_COMMAND_INVALID   = 0x03,
} EN_HDMITX4VX1_MAILBOX_STATUS_RESPONSE_TYPE; // mailbox received status response type

/// Mailbox Debug Type
typedef enum
{
    E_MAILBOX_DEBUG_TEST_PATTERN     = 0x00,
    E_MAILBOX_DEBUG_DEBUG_MESSAGE    = 0x01,
    E_MAILBOX_DEBUG_FREE_RUN         = 0x02,
    E_MAILBOX_DEBUG_LOCK_STATUS_LPLL = 0x03,
    E_MAILBOX_DEBUG_STATUS_HTT_VTT   = 0x04,
} EN_HDMITX4VX1_MAILBOX_DEBUG_TYPE; // mailbox received command type

/// Mailbox HDCP DDC port switch
typedef enum
{
    E_MAILBOX_HDCP_DDC_SWITCH_0 = 0x00,
    E_MAILBOX_HDCP_DDC_SWITCH_1 = 0x01,
    E_MAILBOX_HDCP_DDC_SWITCH_2 = 0x02,
    E_MAILBOX_HDCP_DDC_SWITCH_3 = 0x03,
} EN_HDMITX4VX1_MAILBOX_HDCP_DDC_PORT; // mailbox hdcp ddc port switch type

/// Mailbox HDCP DDC Enable/Disable
typedef enum
{
    E_MAILBOX_HDCP_DDC_DISABLE = 0,
    E_MAILBOX_HDCP_DDC_ENABLE  = 1,
} EN_HDMITX4VX1_MAILBOX_HDCP_DDC_ENABLE; // mailbox hdcp ddc port enable/disable

/// Mailbox HDCP Repeater Reset
typedef enum
{
    E_MAILBOX_HDCP_REPEATER_RESET_DISABLE = 0,
    E_MAILBOX_HDCP_REPEATER_RESET_ENABLE  = 1,
} EN_HDMITX4VX1_MAILBOX_HDCP_REPEATER_RESET_ENABLE; // mailbox hdcp repeater reset enable/disable

/// Mailbox HDCP User HPD Enable/Disable
typedef enum
{
    E_MAILBOX_HDCP_USER_HPD_DISABLE = 0,
    E_MAILBOX_HDCP_USER_HPD_ENABLE  = 1,
} EN_HDMITX4VX1_MAILBOX_HDCP_USER_HPD_ENABLE; // mailbox hdcp user hpd enable/disable

/// Mailbox HDCP 1.4 Repeater Enable/Disable
typedef enum
{
    E_MAILBOX_HDCP_SETHDCP14REPEATER_DISABLE = 0,
    E_MAILBOX_HDCP_SETHDCP14REPEATER_ENABLE  = 1,
} EN_HDMITX4VX1_MAILBOX_HDCP_SETHDCP14REPEATER_ENABLE; // mailbox hdcp 1.4 repeater enable/disable

/// Mailbox HDCP Repeater Source HDMI Enable/Disable
typedef enum
{
    E_MAILBOX_HDCP_REPEATERSOURCEHDMI_DISABLE = 0,
    E_MAILBOX_HDCP_REPEATERSOURCEHDMI_ENABLE  = 1,
} EN_HDMITX4VX1_MAILBOX_HDCP_REPEATERSOURCEHDMI_ENABLE; // mailbox hdcp repeater source hdmi enable/disable

/// Mailbox HDCP Repeater Mode Enable/Disable
typedef enum
{
    E_MAILBOX_HDCP_REPEATEROPEN_DISABLE = 0,
    E_MAILBOX_HDCP_REPEATEROPEN_ENABLE  = 1,
} EN_HDMITX4VX1_MAILBOX_HDCP_REPEATEROPEN_ENABLE; // mailbox hdcp repeater mode enable/disable

/// Mailbox HDCP Repeater Mode Enable/Disable
typedef enum
{
    E_MAILBOX_HDCP_SOCHPD_DISABLE = 0,
    E_MAILBOX_HDCP_SOCHPD_ENABLE  = 1,
} EN_HDMITX4VX1_MAILBOX_HDCP_SOCHPD_ENABLE; // mailbox hdcp SOCHPD enable/disable

/// HDCP Status Type
typedef enum
{
    E_HDMITX4VX1_HDCP_STATUS_UNKNOWN = 0,
    E_HDMITX4VX1_HDCP_STATUS_14      = 1,
    E_HDMITX4VX1_HDCP_STATUS_22      = 2,
} EN_HDMITX4VX1_HDCP_STATUS_TYPE; // HDMITX4VX1 hdcp type index

/// HDCP Key Status Type
typedef enum
{
    E_HDMITX4VX1_REPORT_KEY_NOT_VALID = 0,
    E_HDMITX4VX1_REPORT_KEY_VALID     = 1,
    E_HDMITX4VX1_REPORT_KEY_UNKNOWN   = 2,
} EN_HDMITX4VX1_HDCPKEY_STATUS_TYPE; // HDMITX4VX1 hdcp type index


/// Mailbox Data Struct For Command 0x01 Configure Video
//for IOCTL_HDMITX4VX1_VIDEO_CONFIG
typedef struct ST_MAILBOX_COMMAND_CONFIQ_VIDEO
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_CONFIQ_VIDEO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// flags for actions
    MS_BOOL timing_present;       /// flag of setting timing
    MS_BOOL color_present;        /// flag of setting color flag
    MS_BOOL VSinfo_3D_present;    /// flag of setting 3D_VS flag
    MS_BOOL hdcp_present;    /// flag of setting HDCP
    MS_BOOL analog_present;       /// flag of setting current, pren2, precon, tenpre and ten

    /// data
    EN_HDMITX4VX1_VIDEO_TIMING timing_idx;        /// timing type
    EN_HDMITX4VX1_COLOR_DEPTH color_depth;               /// color depth, 8bit, 10bit, 12bit, 16bit
    EN_HDMITX4VX1_VIDEO_OUTPUT_TYPE output_mode;         /// video output mode, DVI or HDMI
    EN_HDMITX4VX1_INPUT_COLOR_TYPE in_color;             /// input color format, RGB or YUV
    EN_HDMITX4VX1_OUTPUT_COLOR_TYPE out_color;           /// output color format, RGB or YUV
    EN_HDMITX4VX1_VIDEO_3D_STRUCTURE_TYPE vs_3d;  /// 3d vs information
    EN_HDMITX4VX1_VIDEO_HDCP_TYPE hdcp; /// hdcp type, 1.4 or 2.2
    MS_U8 currentforvideo;
    MS_BOOL pren2;
    MS_U8 precon;
    MS_U8 tenpre;
    MS_U8 ten;
}ST_MAILBOX_COMMAND_CONFIQ_VIDEO;// struct of configure video

/// Mailbox Data Struct For Command 0x02 Configure Audio
// for IOCTL_HDMITX4VX1_AUDIO_CONFIG
typedef struct ST_MAILBOX_COMMAND_CONFIQ_AUDIO
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_CONFIQ_AUDIO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// flags for actions
    MS_BOOL inform_present;   /// flag of timing id, frequency, channel number and code type
    MS_BOOL source_present;   /// flag of timing id, source type
    MS_BOOL fmt_present;      /// flag of timing id, source format

    /// data
    EN_HDMITX4VX1_AUDIO_FREQUENCY_TYPE frequency;  /// audio frequency
    EN_HDMITX4VX1_AUDIO_CHANNEL_COUNT channel_num; /// audio channel number
    EN_HDMITX4VX1_AUDIO_CODING_TYPE code_type;     /// audio coding type
    EN_HDMITX4VX1_AUDIO_SOURCE_TYPE source_type;   /// audio source type
    EN_HDMITX4VX1_AUDIO_SOURCE_FORMAT source_fmt;  /// audio source format
}ST_MAILBOX_COMMAND_CONFIQ_AUDIO;// struct of configure audio

/// Mailbox Data Struct For Command 0x03 Video/Audio Mute and Timing Reset
//for IOCTL_HDMITX4VX1_TIMINGCHANGEAVMUTE_CONFIG
typedef struct ST_MAILBOX_COMMAND_TIMING_CHANGE_AVMUTE
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_AVMUTE_TIMING_RESET

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// flags for actions
    MS_BOOL video_flag;   /// flag of video mute
    MS_BOOL audio_flag;   /// flag of audio mute
    MS_BOOL avmute_flag;  /// flag of audio and video mute
    MS_BOOL timing_flag;  /// flag of changing timing
    MS_BOOL hdmitxpower_flag;  /// flag of changing timing

    /// data
    EN_HDMITX4VX1_VIDEO_MUTE enVideo_mute;
    EN_HDMITX4VX1_AUDIO_MUTE enAudio_mute;
    EN_HDMITX4VX1_AVMUTE enAV_mute;
    EN_HDMITX4VX1_TIMING_CHANGE enTiming;
    EN_HDMITX4VX1_POWER_CHANGE enHdmiPower;
}ST_MAILBOX_COMMAND_TIMING_CHANGE_AVMUTE;// struct of Video/Audio Mute and Timing Reset

/// Mailbox Data Struct For Command 0x04 Query EDID Information
typedef struct ST_MAILBOX_COMMAND_QUERY_EDID_INFO
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_QUERY_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type;

    /// data
    EN_HDMITX4VX1_QUERY_3DTIMING timing_3d;
    EN_HDMITX4VX1_QUERY_COLORFMTTIMING timing_colorfmt;
    MS_U8 des_startIdx;  // audio/video descriptor start index
    MS_U8 des_endIdx;    // audio/video descriptor end index
    MS_U8 bytes64_Idx;   // 64 bytes index
    MS_U8 edid_Idx;      // EDID block index
    MS_U8 KSV_B_Idx;      // KSV_B index
}ST_MAILBOX_COMMAND_QUERY_EDID_INFO;// struct of query edid info

/// Mailbox Data Struct For Command 0x05 Report EDID Information (Different Query Type Has Different Struct)
/// Query Type = 4K2K Vic, Query_Type = E_HDMITX4VX1_QUERY_4K2K_VIC 0x01
typedef struct ST_MAILBOX_COMMAND_REPORT_EDID_INFO_4K2KVIC
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_REPORT_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type; /// val = E_HDMITX4VX1_QUERY_4K2K_VIC

    /// data
    MS_U8 vic_num; /// number of 4k2k_vic
    EN_HDMITX4VX1_VIDEO_4K2K_VIC vic4k2k[8]; /// 4k2k_vic
}ST_MAILBOX_COMMAND_REPORT_EDID_INFO_4K2KVIC;// struct of report edid info 4k2k_vic

/// Mailbox Data Struct For Command 0x05 Report EDID Information (Different Query Type Has Different Struct)
/// Query Type = 3D Structure, Query_Type = E_HDMITX4VX1_QUERY_3D_STRUCTURE 0x02
typedef struct ST_MAILBOX_COMMAND_REPORT_EDID_INFO_3DSTRUCT
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_REPORT_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type; /// val = E_HDMITX4VX1_QUERY_3D_STRUCTURE

    /// data
    EN_HDMITX4VX1_QUERY_3DTIMING repTiming_3d; /// 3D timing
    EN_HDMITX4VX1_VIDEO_REPORT_3D_STRUCTURE_TYPE edid_3d; /// 3D edid structure, combination of EN_HDMITX4VX1_VIDEO_REPORT_3D_STRUCTURE_TYPE
}ST_MAILBOX_COMMAND_REPORT_EDID_INFO_3DSTRUCT;// struct of report edid info 3D_structure

/// Mailbox Data Struct For Command 0x05 Report EDID Information (Different Query Type Has Different Struct)
/// Query Type = Number of Audio Descriptor, Query_Type = E_HDMITX4VX1_QUERY_AUDIO_DESCRIPTOR_NUM 0x03
typedef struct ST_MAILBOX_COMMAND_REPORT_EDID_INFO_AUDIODES_NUM
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_REPORT_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type; /// val = E_HDMITX4VX1_QUERY_AUDIO_DESCRIPTOR_NUM

    /// data
    MS_U8 auddes_num; /// number of audio descriptor
}ST_MAILBOX_COMMAND_REPORT_EDID_INFO_AUDIODES_NUM;// struct of number of audio descriptor

/// Mailbox Data Struct For Command 0x05 Report EDID Information (Different Query Type Has Different Struct)
/// Query Type = Audio Descriptor, Query_Type = E_HDMITX4VX1_QUERY_AUDIO_DESCRIPTOR 0x04
typedef struct ST_MAILBOX_COMMAND_REPORT_EDID_INFO_AUDIODES
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_REPORT_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type; /// val = E_HDMITX4VX1_QUERY_AUDIO_DESCRIPTOR

    /// data
    MS_U8 audstr_Idx; /// start index of audio descriptor
    MS_U8 audend_Idx; /// end index of audio descriptor
    MS_U8 aud_des[33];   /// audio descriptors
}ST_MAILBOX_COMMAND_REPORT_EDID_INFO_AUDIODES;// struct of audio descriptor

/// Mailbox Data Struct For Command 0x05 Report EDID Information (Different Query Type Has Different Struct)
/// Query Type = Number of Video Descriptor, Query_Type = E_HDMITX4VX1_QUERY_VIDEO_DESCRIPTOR_NUM 0x05
typedef struct ST_MAILBOX_COMMAND_REPORT_EDID_INFO_VIDEODES_NUM
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_REPORT_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type; /// val = E_HDMITX4VX1_QUERY_VIDEO_DESCRIPTOR_NUM

    /// data
    MS_U8 viddes_num; /// number of audio descriptor
}ST_MAILBOX_COMMAND_REPORT_EDID_INFO_VIDEODES_NUM;// struct of number of video descriptor

/// Mailbox Data Struct For Command 0x05 Report EDID Information (Different Query Type Has Different Struct)
/// Query Type = Video Descriptor, Query_Type = E_HDMITX4VX1_QUERY_VIDEO_DESCRIPTOR 0x06
typedef struct ST_MAILBOX_COMMAND_REPORT_EDID_INFO_VIDEODES
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_REPORT_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type; /// val = E_HDMITX4VX1_QUERY_VIDEO_DESCRIPTOR

    /// data
    MS_U8 vidstr_Idx; /// start index of video descriptor
    MS_U8 vidend_Idx; /// end index of video descriptor
    MS_U8 vid_des[33];   /// video descriptors
}ST_MAILBOX_COMMAND_REPORT_EDID_INFO_VIDEODES;// struct of video descriptor

/// Mailbox Data Struct For Command 0x05 Report EDID Information (Different Query Type Has Different Struct)
/// Query Type = EDID Raw Data, Query_Type = E_HDMITX4VX1_QUERY_EDID_DATA 0x07
typedef struct ST_MAILBOX_COMMAND_REPORT_EDID_INFO_EDIDRAWDATA
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_REPORT_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type; /// val = E_HDMITX4VX1_QUERY_EDID_DATA

    /// data
    MS_U8 Idx_edid;    /// index of edid
    MS_U8 Idx_64bytes; /// index of 64bytes
    MS_U8 raw_data[68];   /// raw data
}ST_MAILBOX_COMMAND_REPORT_EDID_INFO_EDIDRAWDATA;// struct of edid raw data

/// Mailbox Data Struct For Command 0x05 Report EDID Information (Different Query Type Has Different Struct)
/// Query Type = RX EDID Info, Query_Type = E_HDMITX4VX1_QUERY_RX_EDID_INFO 0x08
typedef struct ST_MAILBOX_COMMAND_REPORT_EDID_INFO_RX_EDID_INFO
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_REPORT_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type; /// val = E_HDMITX4VX1_QUERY_RX_EDID_INFO

    /// data
    MS_BOOL bSupportHdmi;
    EN_HDMITX4VX1_COLOR_DEPTH_INFO enColorDepth;
    MS_U8 u8ManufacturerID[3];
    MS_U8 u8Colorimetry;    //0xF~0x0
    MS_U8 u8ManufacturerProductCode[2];
    MS_U8 u8SerialNum[4];
    MS_U8 u8PhyAddr0; ///
    MS_U8 u8PhyAddr1; ///
}ST_MAILBOX_COMMAND_REPORT_EDID_INFO_RX_EDID_INFO;// struct of number of rx edid info

/// Mailbox Data Struct For Command 0x05 Report EDID Information (Different Query Type Has Different Struct)
/// Query Type = Color Format, Query_Type = E_HDMITX4VX1_QUERY_COLOR_FORMAT 0x09
typedef struct ST_MAILBOX_COMMAND_REPORT_EDID_INFO_COLORFMT
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_REPORT_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type; /// val = E_HDMITX4VX1_QUERY_COLOR_FORMAT

    /// data
    EN_HDMITX4VX1_QUERY_COLORFMTTIMING colfmt_timing;
    MS_U8 supColorFmt;


}ST_MAILBOX_COMMAND_REPORT_EDID_INFO_COLORFMT;// struct of number of color format

/// Mailbox Data Struct For Command 0x05 Report EDID Information (Different Query Type Has Different Struct)
/// Query Type = HW Info, Query_Type = E_HDMITX4VX1_QUERY_HW_INFO 0x0A
typedef struct ST_MAILBOX_COMMAND_REPORT_EDID_INFO_HWINFO
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_REPORT_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type; /// val = E_HDMITX4VX1_QUERY_HW_INFO

    /// data
    EN_HDMITX4VX1_VIDEO_TIMING timingID;        /// timing type
    EN_HDMITX4VX1_COLOR_DEPTH color_depth;      /// color depth, 8bit, 10bit, 12bit, 16bit
    EN_HDMITX4VX1_VIDEO_OUTPUT_TYPE output_mode;/// video output mode, DVI or HDMI
    EN_HDMITX4VX1_INPUT_COLOR_TYPE in_color;    /// input color format, RGB or YUV
    EN_HDMITX4VX1_OUTPUT_COLOR_TYPE out_color;  /// output color format, RGB or YUV

    MS_U8 video_config;

}ST_MAILBOX_COMMAND_REPORT_EDID_INFO_HWINFO;// struct of number of hw info

/// Mailbox Data Struct For Command 0x05 Report EDID Information (Different Query Type Has Different Struct)
/// Query Type = KSV & B Status, Query_Type = E_HDMITX4VX1_QUERY_KSV_BSTATUS 0x0B
typedef struct ST_MAILBOX_COMMAND_REPORT_EDID_INFO_KSV_BSTATUS
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_REPORT_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type; /// val = E_HDMITX4VX1_QUERY_KSV_BSTATUS

    /// data
    MS_U8 device_num;
    MS_U8 KSVB_Idx;
    MS_U8 KSVB_data[64];

}ST_MAILBOX_COMMAND_REPORT_EDID_INFO_KSV_BSTATUS;// struct of number of ksv&Bstatus

/// Mailbox Data Struct For Command 0x05 Report EDID Information (Different Query Type Has Different Struct)
/// Query Type = Number of Video Descriptor, Query_Type = E_HDMITX4VX1_QUERY_VIDEO_DESCRIPTOR_NUM 0x05
typedef struct ST_MAILBOX_COMMAND_REPORT_EDID_INFO_HDCPKEY_STATUS
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_REPORT_EDID_INFO

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// query type
    EN_HDMITX4VX1_QUERY_TYPE query_type;  ///val = E_HDMITX4VX1_QUERY_HDCPKEY_STATUS

    /// data
    EN_HDMITX4VX1_HDCPKEY_STATUS_TYPE HDCP14Key_Status; /// 14 key status
    EN_HDMITX4VX1_HDCPKEY_STATUS_TYPE HDCP22Key_Status; /// 22 key status
}ST_MAILBOX_COMMAND_REPORT_EDID_INFO_HDCPKEY_STATUS;// struct of hdcp key status

/// Mailbox Data Struct For Command 0x06 Status Response / ACK-Command
typedef struct ST_MAILBOX_COMMAND_ACK_STATUS_RESPONSE
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_ACK_STATUS_RESPOND

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// received command
    EN_HDMITX4VX1_MAILBOX_RECIEVED_COMMAND_TYPE mbx_reccom; /// received command type

    /// status
    EN_HDMITX4VX1_MAILBOX_STATUS_RESPONSE_TYPE mbx_status;  /// received status type

}ST_MAILBOX_COMMAND_ACK_STATUS_RESPONSE;// struct of status response or ack-command


/// Mailbox Data Struct For Command 0x09 HDCP Command
//for IOCTL_HDMITX4VX1_HDCP_COMMAND
typedef struct ST_MAILBOX_COMMAND_HDCP_COMD
{
    /// command id
    EN_HDMITX4VX1_MAILBOX_COMMAND_TYPE command_idx; /// val = E_MAILBOX_HDCP_COMMAND

    /// length of the following data
    MS_U8 data_len; /// Bytes

    /// flags for actions
    MS_BOOL ddcport_present;   /// flag of ddc port
    MS_BOOL repeaterV_present;   /// flag of set repeater V'
    MS_BOOL repeaterReset_present;  /// flag of reset repeater
    MS_BOOL user_hpd_present;        /// flag of set user hpd
    MS_BOOL hdcp14repeater_present;  /// flag of set hdcp1.4 repeater
    MS_BOOL repeatersourcehdmi_present; /// flag of set source hdmi
    MS_BOOL repeate_en_present; /// flag of enable/disable repeater
    MS_BOOL ext_present; /// flag of extra line in mbx command
    MS_BOOL KSVbstatus_present; /// flag of set Bstatus, this flag's arise will disable other flags
    MS_BOOL SOC_hpd_present; /// flag of soc hpd

    /// data
    EN_HDMITX4VX1_MAILBOX_HDCP_DDC_PORT ddc_port;
    EN_HDMITX4VX1_MAILBOX_HDCP_DDC_ENABLE enddc_port;
    EN_HDMITX4VX1_MAILBOX_HDCP_REPEATER_RESET_ENABLE enRepeaterReset;
    EN_HDMITX4VX1_MAILBOX_HDCP_USER_HPD_ENABLE enUserHPD;
    EN_HDMITX4VX1_MAILBOX_HDCP_SETHDCP14REPEATER_ENABLE enHDCP14Repeater;
    EN_HDMITX4VX1_MAILBOX_HDCP_REPEATERSOURCEHDMI_ENABLE enSourceHDMI;
    EN_HDMITX4VX1_MAILBOX_HDCP_REPEATEROPEN_ENABLE enOpenRepeater;
    EN_HDMITX4VX1_MAILBOX_HDCP_SOCHPD_ENABLE enSOCHPD;

    MS_U8 repeaterV_val[20];
    MS_U8 device_num;
    MS_U8 Idx_forKSVBStatus;
    MS_U8 BStatus[BSTATUS_SIZE];
    MS_U8 KSVlist[MAX_KSV_LIST_SIZE];
}ST_MAILBOX_COMMAND_HDCP_COMD;// struct of HDCP Command

// for read/write register
typedef struct ST_HDMITX4VX1_REGISTER
{
   MS_U16 reg_addr;
   MS_U8 mask;
   MS_U8 reg_val;
}ST_HDMITX4VX1_REGISTER;



//for IOCTL_HDMITX4VX1_LOADOUTPUTSCRIPT
typedef struct ST_HDMITX4VX1_LOADOUTPUTSCRIPT_SETTINGS
{
    EN_HDMITX4VX1_VIDEO_TIMING loadtiming;
    EN_HDMITX4VX1_COLOR_DEPTH loadcolordepth;
    EN_HDMITX4VX1_OUTPUT_COLOR_TYPE loadoutcolorfmt;

}ST_HDMITX4VX1_LOADOUTPUTSCRIPT_SETTINGS;

//for IOCTL_HDMITX4VX1_QUERY_4K2KVIC
typedef struct ST_HDMITX4VX1_IOC_QUERY_4K2K_VIC
{
   ST_MAILBOX_COMMAND_QUERY_EDID_INFO  query_cmd;
   ST_MAILBOX_COMMAND_REPORT_EDID_INFO_4K2KVIC report_cmd;
}ST_HDMITX4VX1_IOC_QUERY_4K2K_VIC;

//for IOCTL_HDMITX4VX1_QUERY_3DSTRUCTURE
typedef struct ST_HDMITX4VX1_IOC_QUERY_3D_STRUCTURE
{
   ST_MAILBOX_COMMAND_QUERY_EDID_INFO  query_cmd;
   ST_MAILBOX_COMMAND_REPORT_EDID_INFO_3DSTRUCT report_cmd;
}ST_HDMITX4VX1_IOC_QUERY_3D_STRUCTURE;

//for IOCTL_HDMITX4VX1_QUERY_AUDIODESCRIPTOR_NUM
typedef struct ST_HDMITX4VX1_IOC_QUERY_NUM_AUDIODESCRIPTOR
{
   ST_MAILBOX_COMMAND_QUERY_EDID_INFO  query_cmd;
   ST_MAILBOX_COMMAND_REPORT_EDID_INFO_AUDIODES_NUM report_cmd;
}ST_HDMITX4VX1_IOC_QUERY_NUM_AUDIODESCRIPTOR;

//for IOCTL_HDMITX4VX1_QUERY_AUDIODESCRIPTOR
typedef struct ST_HDMITX4VX1_IOC_QUERY_AUDIODESCRIPTOR
{
   ST_MAILBOX_COMMAND_QUERY_EDID_INFO  query_cmd;
   ST_MAILBOX_COMMAND_REPORT_EDID_INFO_AUDIODES report_cmd;
}ST_HDMITX4VX1_IOC_QUERY_AUDIODESCRIPTOR;

//for IOCTL_HDMITX4VX1_QUERY_VIDEODESCRIPTOR_NUM
typedef struct ST_HDMITX4VX1_IOC_QUERY_NUM_VIDEODESCRIPTOR
{
   ST_MAILBOX_COMMAND_QUERY_EDID_INFO  query_cmd;
   ST_MAILBOX_COMMAND_REPORT_EDID_INFO_VIDEODES_NUM report_cmd;
}ST_HDMITX4VX1_IOC_QUERY_NUM_VIDEODESCRIPTOR;

//for IOCTL_HDMITX4VX1_QUERY_VIDEODESCRIPTOR
typedef struct ST_HDMITX4VX1_IOC_QUERY_VIDEODESCRIPTOR
{
   ST_MAILBOX_COMMAND_QUERY_EDID_INFO  query_cmd;
   ST_MAILBOX_COMMAND_REPORT_EDID_INFO_VIDEODES report_cmd;
}ST_HDMITX4VX1_IOC_QUERY_VIDEODESCRIPTOR;

//for IOCTL_HDMITX4VX1_QUERY_EDIDRAWDATA
typedef struct ST_HDMITX4VX1_IOC_QUERY_EDIDRAWDATA
{
   ST_MAILBOX_COMMAND_QUERY_EDID_INFO  query_cmd;
   ST_MAILBOX_COMMAND_REPORT_EDID_INFO_EDIDRAWDATA report_cmd;
}ST_HDMITX4VX1_IOC_QUERY_EDIDRAWDATA;

//for IOCTL_HDMITX4VX1_QUERY_RXEDIDINFO
typedef struct ST_HDMITX4VX1_IOC_QUERY_RXEDIDINFO
{
   ST_MAILBOX_COMMAND_QUERY_EDID_INFO  query_cmd;
   ST_MAILBOX_COMMAND_REPORT_EDID_INFO_RX_EDID_INFO report_cmd;
}ST_HDMITX4VX1_IOC_QUERY_RXEDIDINFO;

//for IOCTL_HDMITX4VX1_QUERY_COLORFORMAT
typedef struct ST_HDMITX4VX1_IOC_QUERY_COLORFORMAT
{
   ST_MAILBOX_COMMAND_QUERY_EDID_INFO  query_cmd;
   ST_MAILBOX_COMMAND_REPORT_EDID_INFO_COLORFMT report_cmd;
}ST_HDMITX4VX1_IOC_QUERY_COLORFORMAT;

//for IOCTL_HDMITX4VX1_QUERY_HWINFO
typedef struct ST_HDMITX4VX1_IOC_QUERY_HWINFO
{
   ST_MAILBOX_COMMAND_QUERY_EDID_INFO  query_cmd;
   ST_MAILBOX_COMMAND_REPORT_EDID_INFO_HWINFO report_cmd;
}ST_HDMITX4VX1_IOC_QUERY_HWINFO;

//for IOCTL_HDMITX4VX1_QUERY_KSVBSTATUS
typedef struct ST_HDMITX4VX1_IOC_QUERY_KSVBSTATUS
{
   ST_MAILBOX_COMMAND_QUERY_EDID_INFO  query_cmd;
   ST_MAILBOX_COMMAND_REPORT_EDID_INFO_KSV_BSTATUS report_cmd;
}ST_HDMITX4VX1_IOC_QUERY_KSVBSTATUS;

//for IOCTL_HDMITX4VX1_QUERY_HDCPKEYSTATUS
typedef struct ST_HDMITX4VX1_IOC_QUERY_HDCPKEYSTATUS
{
   ST_MAILBOX_COMMAND_QUERY_EDID_INFO  query_cmd;
   ST_MAILBOX_COMMAND_REPORT_EDID_INFO_HDCPKEY_STATUS report_cmd;
}ST_HDMITX4VX1_IOC_QUERY_HDCPKEYSTATUS;

//for IOCTL_HDMITX4VX1_READREGISTER
typedef struct ST_HDMITX4VX1_IOC_READREGISTER
{
   int register_num;
   ST_HDMITX4VX1_REGISTER *read_register;
}ST_HDMITX4VX1_IOC_READREGISTER;

//for IOCTL_HDMITX4VX1_WRITEREGISTER_MASK
typedef struct ST_HDMITX4VX1_IOC_WRITEREGISTER
{
   int register_num;
   ST_HDMITX4VX1_REGISTER *write_register;
}ST_HDMITX4VX1_IOC_WRITEREGISTER;


#endif //_MDRV_HDMITX4VX1_ST_H
