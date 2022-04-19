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
/// @file   mosWrapper.c
/// @brief  MStar System wrapper header file
/// @author MStar Semiconductor Inc.
/// @attention
/// <b><em></em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MOS_WRAPPER_H
#define _MOS_WRAPPER_H

/* Kernel includes. */
//#include "FreeRTOS.h"
//#include "queue.h"
//#include "task.h"
//#include "semphr.h"
//#include "event_groups.h"
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <mach/io.h>

#include "std_c.h"
//#include "boot.h"
//#include "ms_platform.h"
#include "platform.h"
//#include "irq.h"



//-------------------------------------------------------------------------------------------------
// Defines
//-------------------------------------------------------------------------------------------------
/// Define debug level:
/// Default Debug Level: No any Debug Message Print out
#define MOS_DBG_LEVEL_NONE       0x00
#define MOS_DBG_LEVEL_ERROR      0x01
#define MOS_DBG_LEVEL_VOC        0x02
#define MOS_DBG_LEVEL_CMD        0x04
#define MOS_DBG_LEVEL_CRITICAL   0x08

#define MOS_DBG_LEVEL_AUD        0x00010000
#define MOS_DBG_LEVEL_MBX        0x00020000
#define MOS_DBG_LEVEL_INTC       0x00040000
#define MOS_DBG_LEVEL_CPUINT     0x00080000
#define MOS_DBG_LEVEL_BDMA       0x00100000
#define MOS_DBG_LEVEL_PM         0x00200000
#define MOS_DBG_LEVEL_RTC        0x00400000
#define MOS_DBG_LEVEL_ALL        0xFFFFFFFF

//-------------------------------------------------------------------------------------------------
// Macros
//-------------------------------------------------------------------------------------------------
extern U32 gu32MosDbgLevel;
#define MOS_DBG_PRINT(dbgLevel, fmt, args...)  { \
                                                    if(gu32MosDbgLevel&dbgLevel) \
                                                    { \
                                                        printk(KERN_ERR fmt, ## args); \
                                                    } \
                                                }

#define MOS_DBG_ERROR(fmt, args...)             { \
                                                    if(gu32MosDbgLevel&MOS_DBG_LEVEL_ERROR) \
                                                    { \
                                                        printk(KERN_ERR fmt, ## args); \
                                                    } \
                                                }

#define MOS_DBG_SetLevel(level) { \
                                         gu32MosDbgLevel |= level;\
                                       }


#define MOS_DBG_ClearLevel(level) { \
                                           gu32MosDbgLevel &= ~level;\
                                         }


//-------------------------------------------------------------------------------------------------
// Type and Structure Declaration
//-------------------------------------------------------------------------------------------------

//typedef void (*IRQn_ISR_t)(void);

//-------------------------------------------------------------------------------------------------
// Extern Functions
//-------------------------------------------------------------------------------------------------

//void MOS_IRQn_AttachIsr(IRQn_ISR_t tIntIsr, IRQn_Type eIntNum, U32 u32Priority);
//void MOS_IRQn_Mask(IRQn_Type eIntNum);
//void MOS_IRQn_Unmask(IRQn_Type eIntNum);
void MOS_mDelay(U32 u32MiniSeconds);
void MOS_uDelay(U32 u32MicroSeconds);

void* alloc_dmem(const char* name, unsigned int size, dma_addr_t *addr);
#endif

