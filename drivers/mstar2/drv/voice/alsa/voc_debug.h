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

#ifndef _VOC_DEBUG_H_
#define _VOC_DEBUG_H_
//------------------------------------------------------------------------------
//  Macros
//------------------------------------------------------------------------------
#include "chip_int.h"

/*#if 0
#define TRACE_LEVEL_TAG        1
#define ERROR_LEVEL_TAG        1
#define DEBUG_LEVEL_TAG        1
#define IRQ_LEVEL_TAG          1
#define WARN_LEVEL_TAG         1
#else
#define TRACE_LEVEL_TAG        1
#define WARN_LEVEL_TAG         1
#define ERROR_LEVEL_TAG        1
#define DEBUG_LEVEL_TAG        1
#define IRQ_LEVEL_TAG          0
#endif*/

#define WARN_LEVEL_TAG        (1<<0)
#define ERROR_LEVEL_TAG       (1<<1)
#define TRACE_LEVEL_TAG       (1<<2)
#define DEBUG_LEVEL_TAG       (1<<3)
#define IRQ_LEVEL_TAG         (1<<4)

#define DEFAULT_MESSAGES (7) //(WARN_LEVEL_TAG | ERROR_LEVEL_TAG)


#define NONE			"\033[m"
#define RED				"\033[0;32;31m"
#define LIGHT_RED		"\033[1;31m"
#define GREEN			"\033[0;32;32m"
#define LIGHT_GREEN		"\033[1;32m"
#define BLUE			"\033[0;32;34m"
#define LIGHT_BLUE		"\033[1;34m"
#define DARY_GRAY		"\033[1;30m"
#define CYAN			"\033[0;36m"
#define LIGHT_CYAN		"\033[1;36m"
#define PURPLE			"\033[0;35m"
#define LIGHT_PURPLE	"\033[1;35m"
#define BROWN			"\033[0;33m"
#define YELLOW			"\033[1;33m"
#define LIGHT_GRAY		"\033[0;37m"
#define WHITE			"\033[1;37m"

#define TRACE_LEVEL          LIGHT_GREEN"[VOC TRACE]"NONE
#define ERROR_LEVEL          LIGHT_RED"[VOC ERROR]"NONE
#define DEBUG_LEVEL          LIGHT_BLUE"[VOC DEBUG]"NONE
#define WARN_LEVEL           "[VOC WARN]"
#define IRQ_LEVEL            "[VOC IRQ]"


extern unsigned int u32VocDbgLogLevel;

#define LOG_MSG 					1
#if LOG_MSG
#if 0
#define VOC_PRINTF(level ,fmt, arg...)		if (level##_TAG) printk(KERN_ERR level fmt, ##arg);
#else
#define VOC_PRINTF(level ,fmt, arg...)		if ((level##_TAG) & u32VocDbgLogLevel) printk(KERN_ERR level fmt, ##arg);
#endif

#else
#define VOC_PRINTF(level ,fmt, arg...)
#endif

//#define __VOICE_AEC_SW_SYNC__
#define __VOICE_I2S__
#define __VOICE_HW_AEC__
//#define __VOICE_MSYS_ALLOC__
#define __VOICE_DA__

#endif  /* _VOC_DEBUG_H_ */
