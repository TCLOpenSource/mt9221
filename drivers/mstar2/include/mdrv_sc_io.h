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

// PUT THIS FILE under INLCUDE path

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @file   mdrv_temp_io.h
/// @brief  TEMP Driver Interface.
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_SC_IO_H_
#define _MDRV_SC_IO_H_

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#include "mdrv_sc.h"
#endif

//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------

#define SC_IOC_MAGIC                's'

/* IOCTL functions. */
/* int ioctl(int fd,
             int request == MDRV_SC_ATTACH_INTERRUPT);
*/
#define MDRV_SC_ATTACH_INTERRUPT                (int)_IO(SC_IOC_MAGIC, 0U)

/* int ioctl(int fd,
             int request == MDRV_SC_DETACH_INTERRUPT);
*/
#define MDRV_SC_DETACH_INTERRUPT                (int)_IO(SC_IOC_MAGIC, 1U)

/* int ioctl(int fd,
             int request == MDRV_SC_RESET_FIFO);
*/
#define MDRV_SC_RESET_FIFO                      (int)_IO(SC_IOC_MAGIC, 2U)

/* int ioctl(int fd,
             int request == MDRV_SC_GET_EVENTS,
             int *events);
*/
#define MDRV_SC_GET_EVENTS                      _IOR(SC_IOC_MAGIC, 3U, int)

/* int ioctl(int fd,
             int request == MDRV_SC_SET_EVENTS,
             int *events);
*/
#define MDRV_SC_SET_EVENTS                      _IOW(SC_IOC_MAGIC, 4U, int)

/* int ioctl(int fd,
             int request == MDRV_SC_GET_ATTRIBUTE,
             int *events);
*/
#define MDRV_SC_GET_ATTRIBUTE                   _IOR(SC_IOC_MAGIC, 5U, int)


/* int ioctl(int fd,
             int request == MDRV_SC_SET_ATTRIBUTE,
             int *events);
*/
#define MDRV_SC_SET_ATTRIBUTE                   _IOW(SC_IOC_MAGIC, 6U, int)


/* int ioctl(int fd,
             int request == MDRV_SC_CHECK_RST_TO_ATR,
             int *events);
*/
#define MDRV_SC_CHECK_RST_TO_ATR                _IOW(SC_IOC_MAGIC, 7U, int)


#define MDRV_SC_MDB_WRITE_INFO                  _IOW(SC_IOC_MAGIC, 8U, int)


/* int ioctl(int fd,
             int request == MDRV_SC_GET_CWT_RX_ERROR_INDEX,
             int *events);
*/
#define MDRV_SC_GET_CWT_RX_ERROR_INDEX         _IOR(SC_IOC_MAGIC, 9U, int)


/* int ioctl(int fd,
             int request == MDRV_SC_RESET_RST_TO_ATR,
             int *events);
*/
#define MDRV_SC_RESET_RST_TO_ATR               (int)_IO(SC_IOC_MAGIC, 10U)


#define MDRV_SC_MDB_GET_CMD_DATA               _IOR(SC_IOC_MAGIC, 11U, int)


/* int ioctl(int fd,
             int request == MDRV_SC_ENABLE_IRQ,
             int *events);
*/
#define MDRV_SC_ENABLE_IRQ                     _IOW(SC_IOC_MAGIC, 12U, int)

#define SC_IOC_MAXNR                            12

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------

#endif // _MDRV_TEMP_IO_H_

