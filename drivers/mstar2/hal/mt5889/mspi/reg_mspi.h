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
/// file    reg_mspi.h
/// @brief  local dimming Module Register Definition
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _REG_MSPI_H_
#define _REG_MSPI_H_
//-------------------------------------------------------------------------------------------------
//  Hardware Capability
//-------------------------------------------------------------------------------------------------

#define REG_MSPI1_BK                0x153A00*2
#define REG_MSPI2_BK                0x153B00*2
#define BK_CLK0                        0x100B00*2
#if defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define REG_MSPI1_BASE           (mstar_pm_base + REG_MSPI1_BK)
#define REG_MSPI2_BASE           (mstar_pm_base + REG_MSPI2_BK)
#else
#define REG_RIU_BASE                0xFD000000

#define REG_MSPI1_BASE           (REG_RIU_BASE + REG_MSPI1_BK)
#define REG_MSPI2_BASE           (REG_RIU_BASE + REG_MSPI2_BK)
#endif

#define MSPI_WRITE_BUF_OFFSET          0x40
#define MSPI_READ_BUF_OFFSET           0x44
#define MSPI_WBF_SIZE_OFFSET           0x48
#define MSPI_RBF_SIZE_OFFSET           0x48

// read/ write buffer size
#define MSPI_RWSIZE_MASK               0xFF
#define MSPI_RSIZE_BIT_OFFSET          0x8
#define MAX_READ_BUF_SIZE              0x8
#define MAX_WRITE_BUF_SIZE             0x8

// CLK config 
#define MSPI_CTRL_OFFSET               0x49
#define MSPI_CLK_CLOCK_OFFSET          0x49
#define MSPI_CLK_CLOCK_BIT_OFFSET      0x08
#define MSPI_CLK_CLOCK_MASK            0xFF
#define MSPI_CLK_PHASE_MASK            0x40
#define MSPI_CLK_PHASE_BIT_OFFSET      0x06
#define MSPI_CLK_POLARITY_MASK         0x80
#define MSPI_CLK_POLARITY_BIT_OFFSET   0x07
#define MSPI_CLK_PHASE_MAX             0x1
#define MSPI_CLK_POLARITY_MAX          0x1
#define MSPI_CLK_CLOCK_MAX             0x7

// DC config 
#define MSPI_DC_MASK                   0xFF
#define MSPI_DC_BIT_OFFSET             0x08
#define MSPI_DC_TR_START_OFFSET        0x4A
#define MSPI_DC_TRSTART_MAX            0xFF
#define MSPI_DC_TR_END_OFFSET          0x4A
#define MSPI_DC_TREND_MAX              0xFF   
#define MSPI_DC_TB_OFFSET              0x4B
#define MSPI_DC_TB_MAX                 0xFF
#define MSPI_DC_TRW_OFFSET             0x4B
#define MSPI_DC_TRW_MAX                0xFF

// Frame Config 
#define MSPI_FRAME_WBIT_OFFSET         0x4C
#define MSPI_FRAME_RBIT_OFFSET         0x4E
#define MSPI_FRAME_BIT_MAX             0x07
#define MSPI_FRAME_BIT_MASK            0x07
#define MSPI_FRAME_BIT_FIELD           0x03
#define MSPI_LSB_FIRST_OFFSET          0x50
#define MSPI_TRIGGER_OFFSET            0x5A
#define MSPI_DONE_OFFSET               0x5B
#define MSPI_DONE_CLEAR_OFFSET         0x5C
#define MSPI_CHIP_SELECT_OFFSET        0x5F

#define MSPI_FULL_DEPLUX_RD00 (0x78)
#define MSPI_FULL_DEPLUX_RD01 (0x78)
#define MSPI_FULL_DEPLUX_RD02 (0x79
#define MSPI_FULL_DEPLUX_RD03 (0x79)
#define MSPI_FULL_DEPLUX_RD04 (0x7a)
#define MSPI_FULL_DEPLUX_RD05 (0x7a)
#define MSPI_FULL_DEPLUX_RD06 (0x7b)
#define MSPI_FULL_DEPLUX_RD07 (0x7b)

#define MSPI_FULL_DEPLUX_RD08 (0x7c)
#define MSPI_FULL_DEPLUX_RD09 (0x7c)
#define MSPI_FULL_DEPLUX_RD10 (0x7d)
#define MSPI_FULL_DEPLUX_RD11 (0x7d)
#define MSPI_FULL_DEPLUX_RD12 (0x7e)
#define MSPI_FULL_DEPLUX_RD13 (0x7e)
#define MSPI_FULL_DEPLUX_RD14 (0x7f)
#define MSPI_FULL_DEPLUX_RD15 (0x7f)


//chip select bit map
#define MSPI_CHIP_SELECT_MAX           0x07  
  
// control bit
#define MSPI_DONE_FLAG                 0x01
#define MSPI_TRIGGER                   0x01
#define MSPI_CLEAR_DONE                0x01
#define MSPI_INT_ENABLE                0x04
#define MSPI_RESET                     0x02
#define MSPI_ENABLE                    0x01

// CLKGEN0
#define  MSPI_CLK_CFG                  0x17
#define  MSPI_CLK_DIV                  0x13
#define  MSPI_CLK_DIV_MASK             0x0F
#define  MSPI_CLK_CFG_OFFSET           0x02
#define  MSPI_CLK_MASK                 0x1F
#define  MSPI_CLK_DEFAULT              0x20
#define  MSPI_MAXCLKLEVEL              0x07


//CLKGEN1 for mspi1
    // clk_spi_m_p1
#define  MSPI1_CLK_DIV                  0x21
#define  MSPI1_CLK_DIV_MASK             0x0F

#define  MSPI1_CLK_CFG                  0x21
#define  MSPI1_CLK_CFG_OFFSET           0x02
#define  MSPI1_CLK_MASK                 0x1F00
#define  MSPI1_CLK_DEFAULT              0x20
#define  MSPI1_MAXCLKLEVEL			    0x07



#endif // _REG_MSPI_H_
