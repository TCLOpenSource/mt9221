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

#ifndef __DRV_SYSTEM_IO_H__
#define __DRV_SYSTEM_IO_H__

#define IO_SYS_GET_RAW_UART
#define IO_SYS_REG_OP

//-------------------------------------------------------------------------------------------------
//  ioctl method
//-------------------------------------------------------------------------------------------------

// Use 'S' as magic number
#define SYS_IOCTL_MAGIC             'S'

#define IOCTL_SYS_INIT                 _IOWR(SYS_IOCTL_MAGIC, 0x00, int)
#define IOCTL_SYS_SET_PANEL_INFO       _IOW (SYS_IOCTL_MAGIC, 0x01, int)
#define IOCTL_SYS_SET_BOARD_INFO       _IOW (SYS_IOCTL_MAGIC, 0x02, int)
#define IOCTL_SYS_GET_PANEL_RES        _IOR (SYS_IOCTL_MAGIC, 0x03, int)
#define IOCTL_SYS_READ_GEN_REGISTER    _IOR (SYS_IOCTL_MAGIC, 0x04, int)
#define IOCTL_SYS_WRITE_GEN_REGISTER   _IOWR(SYS_IOCTL_MAGIC, 0x05, int)
#define IOCTL_SYS_LOAD_AEON            _IOWR(SYS_IOCTL_MAGIC, 0x06, int)
#define IOCTL_SYS_RESET_AEON           _IOWR(SYS_IOCTL_MAGIC, 0x07, int)
#define IOCTL_SYS_ENABLE_AEON          _IO(SYS_IOCTL_MAGIC, 0x08)
#define IOCTL_SYS_DISABLE_AEON         _IO(SYS_IOCTL_MAGIC, 0x09)
#define IOCTL_SYS_SWITCH_UART          _IOR (SYS_IOCTL_MAGIC, 0x0A, int)
#define IOCTL_SYS_DUMP_AEON_MSG        _IOR (SYS_IOCTL_MAGIC, 0x0B, int)
#define IOCTL_SYS_IS_AEON_ENABLE       _IOR (SYS_IOCTL_MAGIC, 0x0C, int)
#define IOCTL_SYS_CHANGE_UART          _IOWR(SYS_IOCTL_MAGIC, 0x0D, int)
#define IOCTL_SYS_DDC2BI               _IOWR(SYS_IOCTL_MAGIC, 0x0E, int) //2008/10/23 Nick

#define IOCTL_SYS_SET_GFX_GOP_INDEX     _IOW (SYS_IOCTL_MAGIC, 0x11,int)
#define IOCTL_SYS_GET_GFX_GOP_INDEX     _IOR (SYS_IOCTL_MAGIC, 0x12,int)

#define IOCTL_SYS_SET_DISPLAY_CTLR_SEPT_INDEX       _IOW (SYS_IOCTL_MAGIC, 0x15,int)
#define IOCTL_SYS_IS_DISPLAY_CTLR_SEPT_INDEX        _IOR (SYS_IOCTL_MAGIC, 0x16,int)

#define IOCTL_SYS_SET_NEXUS     _IOW (SYS_IOCTL_MAGIC, 0x17,int)
#define IOCTL_SYS_HAS_NEXUS     _IOR (SYS_IOCTL_MAGIC, 0x18,int)

#define IOCTL_SYS_PRINT_MSG             _IOW (SYS_IOCTL_MAGIC, 0x19,int)
#define IOCTL_SYS_GET_GFX_GOP_PIPELINE_DELAY        _IOWR (SYS_IOCTL_MAGIC, 0x1A,int)
#define IOCTL_SYS_GET_PANEL_H_START     _IOR (SYS_IOCTL_MAGIC, 0x1B,int)

#define IOCTL_SYS_SET_NEXUS_PID     _IOW (SYS_IOCTL_MAGIC, 0x1C,int)
#define IOCTL_SYS_GET_NEXUS_PID     _IOR (SYS_IOCTL_MAGIC, 0x1D,int)

#define IOCTL_SYS_PCMCIA_WRITE      _IOW(SYS_IOCTL_MAGIC, 0x40,int)
#define IOCTL_SYS_PCMCIA_READ       _IOR(SYS_IOCTL_MAGIC, 0x41,int)
#define IOCTL_SYS_PCMCIA_WRITE_DATA       _IOW(SYS_IOCTL_MAGIC, 0x42,int)
#define IOCTL_SYS_PCMCIA_READ_DATA       _IOR(SYS_IOCTL_MAGIC, 0x43,int)

#define IOCTL_SYS_FLUSH_MEMORY         _IO(SYS_IOCTL_MAGIC, 0x50)
#define IOCTL_SYS_READ_MEMORY          _IO(SYS_IOCTL_MAGIC, 0x51)

#ifdef IO_SYS_REG_OP
#   define IOCTL_SYS_REG_OP            _IOWR(SYS_IOCTL_MAGIC, 0x54, int)
#endif

#ifdef IO_SYS_GET_RAW_UART
#   define IOCTL_SYS_GET_RAW_UART      _IOWR(SYS_IOCTL_MAGIC, 0x55, int)
#endif

#define IOCTL_SYS_TIMER                _IOWR(SYS_IOCTL_MAGIC, 0x56, int)
#define IOCTL_SYS_RELOAD_AEON          _IOWR(SYS_IOCTL_MAGIC, 0x57, int)

#if 0
//#define IOCTL_SYS_SWITCH_PAD        _IOWR(SYS_IOCTL_MAGIC, 0x01, DevSys_Switch_Pad)
#define IOCTL_SYS_WDT_ENABLE        _IOW (SYS_IOCTL_MAGIC, 0x02, MS_U32)
#define IOCTL_SYS_WDT_CLEAR         _IO  (SYS_IOCTL_MAGIC, 0x03)
#define IOCTL_SYS_WDT_LASTSTATUS    _IOWR(SYS_IOCTL_MAGIC, 0x04, MS_U32)
#define IOCTL_SYS_WDT_SETTIME       _IOW (SYS_IOCTL_MAGIC, 0x05, MS_U32)
#define IOCTL_SYS_RESET_CHIP        _IO  (SYS_IOCTL_MAGIC, 0x06)
#define IOCTL_SYS_RESET_CPU         _IO  (SYS_IOCTL_MAGIC, 0x07)
#endif

#define IOCTL_SYS_HOTEL_MODE           _IOWR(SYS_IOCTL_MAGIC, 0x58, int)
#define IOCTL_SYS_HOTEL_MODE_PRINTF    _IOWR(SYS_IOCTL_MAGIC, 0x59, int)
#define IOCTL_SYS_POWER_DOWN           _IOWR(SYS_IOCTL_MAGIC, 0x60, int)
#define IOCTL_SYS_GET_MBOX_SHM         _IOWR(SYS_IOCTL_MAGIC, 0x61, int)
#define IOCTL_SYS_GET_MSBIN_INFO       _IOWR(SYS_IOCTL_MAGIC, 0x62, int)
#define IOCTL_SYS_GET_MIU1_BASE        _IOWR(SYS_IOCTL_MAGIC, 0x63, int)
#define IOCTL_SYS_GET_MIU1_BUS_BASE    _IOWR(SYS_IOCTL_MAGIC, 0x64, int)

#define IOCTL_SYS_HOLD_KERNEL          _IO(SYS_IOCTL_MAGIC, 0x65)
#define IOCTL_SYS_STOP_UART_CLK          _IO(SYS_IOCTL_MAGIC, 0x66)
#define IOCTL_SYS_RESUME_UART_CLK          _IO(SYS_IOCTL_MAGIC, 0x67)
#define IOCTL_SYS_ENABLE_MUDI          _IO(SYS_IOCTL_MAGIC, 0x68)
#define IOCTL_SYS_DISABLE_MUDI          _IO(SYS_IOCTL_MAGIC, 0x69)

#define IOCTL_SYS_SPI_LOAD             _IOWR (SYS_IOCTL_MAGIC, 0x70, int)
#define IOCTL_SYS_INFO              _IOWR (SYS_IOCTL_MAGIC, 0x99,IO_Sys_Info_t)
#define IOCTL_SYS_INFO_EX           _IOWR (SYS_IOCTL_MAGIC, 0x9A, IO_Sys_Info_t_EX)

#define IOCTL_SYS_MAXNR    0xFF

#endif // __DRV_SYSTEM_IO_H__

