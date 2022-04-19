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

#ifndef _BOARD_H_
#define _BOARD_H_

#include <generated/autoconf.h>
//#include <linux/undefconf.h>

//------------------------------IR_TYPE_SEL--------------
#define IR_TYPE_OLD                 0
#define IR_TYPE_NEW                 1
#define IR_TYPE_MSTAR_DTV           2
#define IR_TYPE_MSTAR_RAW		    3
#define IR_TYPE_RC_V16              4
#define IR_TYPE_CUS03_DTV           5
#define IR_TYPE_MSTAR_FANTASY       6
#define IR_TYPE_MSTAR_SZ1           7
#define IR_TYPE_SKYWORTH            8		//_SLIDE
#define IR_TYPE_HISENSE             9
#define IR_TYPE_CUS08_RC5           10
#define IR_TYPE_KONKA               11
#define IR_TYPE_CUS21SH             21
#define IR_TYPE_HAIER               22	//TOSHIBA
#define IR_TYPE_TCL                 23		//_RCA
#define IR_TYPE_RCMM                24
#define IR_TYPE_TOSHIBA             25
#define IR_TYPE_CHANGHONG           26
#define IR_TYPE_PANASONIC           27

//------------------------------BOARD_TYPE_SEL-----------
// 0x00 ~ 0x1F LCD Demo board made in Taiwan
#define BD_FPGA			0x01
#define BD_GENERIC	        0x02

#define BD_UNKNOWN                  0xFF

#define SVD_CLOCK_250MHZ            0x00
#define SVD_CLOCK_240MHZ            0x01
#define SVD_CLOCK_216MHZ            0x02
#define SVD_CLOCK_SCPLL             0x03
#define SVD_CLOCK_MIU               0x04
#define SVD_CLOCK_144MHZ            0x05
#define SVD_CLOCK_123MHZ            0x06
#define SVD_CLOCK_108MHZ            0x07

#define SVD_CLOCK_ENABLE            TRUE
#define SVD_CLOCK_INVERT            FALSE

#ifndef MS_BOARD_TYPE_SEL

#if defined(CONFIG_MSTAR_ARM_BD_FPGA)
    #define MS_BOARD_TYPE_SEL       BD_FPGA
#elif defined(CONFIG_MSTAR_ARM_BD_GENERIC)
    #define MS_BOARD_TYPE_SEL       BD_GENERIC
#else
    #error "BOARD define not found"
#endif

#endif

//-------------------------------------------------------


///////////////////////////////////////////////////////////
#if   (MS_BOARD_TYPE_SEL == BD_FPGA)
    #include "BD_FPGA.h"
#elif (MS_BOARD_TYPE_SEL == BD_GENERIC)
    #include "BD_GENERIC.h"
#endif


/////////////////////////////////////////////////////////

#endif /* _BOARD_H_ */

