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

#ifndef _MDRV_SEMUTEX_IO_H_
#define _MDRV_SEMUTEX_IO_H_

//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------

#define SEMUTEX_IOC_MAGIC               'S'
#define MUTEX_INDEX_BEGIN               0xFF
#define RWLOCK_INDEX_BEGIN              0xFF
#define SEMAPHORE_INDEX_BEGIN			0x55660000

/* IOCTL functions. */
/* int ioctl(int fd,
             int request == MDRV_SEMUTEX_LOCK);
*/
#define MDRV_SEMUTEX_LOCK                (int)_IO(SEMUTEX_IOC_MAGIC, 0U)

/* int ioctl(int fd,
             int request == MDRV_SC_DETACH_INTERRUPT);
*/
#define MDRV_SEMUTEX_UNLOCK                (int)_IO(SEMUTEX_IOC_MAGIC, 1U)

/* int ioctl(int fd,
             int request == MDRV_SC_RESET_FIFO);
*/
#define MDRV_SEMUTEX_CREATE_SHAREMEMORY                      (int)_IO(SEMUTEX_IOC_MAGIC, 2U)

/* int ioctl(int fd,
             int request == MDRV_SC_GET_EVENTS,
             int *events);
*/
#define MDRV_SEMUTEX_MAP_SHAREMEMORY                      (int)_IOR(SEMUTEX_IOC_MAGIC, 3U, int)

#define MDRV_SEMUTEX_UNMAP_SHAREMEMORY                      (int)_IOR(SEMUTEX_IOC_MAGIC, 4U, int)

#define MDRV_SEMUTEX_CREATE_MUTEX                      _IOR(SEMUTEX_IOC_MAGIC, 5U, int)

#define MDRV_SEMUTEX_DEL_MUTEX                      _IOR(SEMUTEX_IOC_MAGIC, 6U, int)

#define MDRV_SEMUTEX_TRY_LOCK                     _IOR(SEMUTEX_IOC_MAGIC, 7U, int)

#define MDRV_SEMUTEX_QUERY_ISFIRST_SHAREMEMORY                      (int)_IO(SEMUTEX_IOC_MAGIC, 8U)

#define MDRV_SEMUTEX_LOCK_WITHTIMEOUT                      _IOR(SEMUTEX_IOC_MAGIC, 9U, int)

#define MDRV_SEMUTEX_EXPAND_SHAREMEMORY                      _IOR(SEMUTEX_IOC_MAGIC, 10U, int)

#define MDRV_SEMUTEX_GET_SHMSIZE		             _IOR(SEMUTEX_IOC_MAGIC, 11U, unsigned int)

#define MDRV_SEMUTEX_CREATE_SEMAPHORE               _IOR(SEMUTEX_IOC_MAGIC, 12U, int)

#define MDRV_SEMUTEX_SEMA_LOCK                      _IOR(SEMUTEX_IOC_MAGIC, 13U, int)

#define MDRV_SEMUTEX_SEMA_TRY_LOCK                  _IOR(SEMUTEX_IOC_MAGIC, 14U, int)

#define MDRV_SEMUTEX_SEMA_UNLOCK                    _IOR(SEMUTEX_IOC_MAGIC, 15U, int)

#define MDRV_SEMUTEX_SEMA_RESET                     _IOR(SEMUTEX_IOC_MAGIC, 16U, int)

#define MDRV_SEMUTEX_SET_CROSS_THREAD_UNLOCK        _IOR(SEMUTEX_IOC_MAGIC, 17U, CROSS_THREAD_UNLOCK_INFO)

/* cmd define for read-write lock */
#define MDRV_SEMUTEX_CREATE_RWLOCK                           _IOR(SEMUTEX_IOC_MAGIC, 18U, int) /*create rwlock*/
#define MDRV_SEMUTEX_DEL_RWLOCK                              _IOR(SEMUTEX_IOC_MAGIC, 19U, int) /*delete rwlock*/

#define MDRV_SEMUTEX_TRY_RDLOCK_RWLOCK                       _IOR(SEMUTEX_IOC_MAGIC, 20U, int) /*non-blocking read lock*/
#define MDRV_SEMUTEX_TIMEOUT_RDLOCK_RWLOCK                   _IOR(SEMUTEX_IOC_MAGIC, 21U, int) /*timeout read lock  (unsupport)*/
#define MDRV_SEMUTEX_RDLOCK_RWLOCK                           _IOR(SEMUTEX_IOC_MAGIC, 22U, int) /*blocking read lock*/

#define MDRV_SEMUTEX_TRY_WRLOCK_RWLOCK                       _IOR(SEMUTEX_IOC_MAGIC, 23U, int) /*non-blocking write lock*/
#define MDRV_SEMUTEX_TIMEOUT_WRLOCK_RWLOCK                   _IOR(SEMUTEX_IOC_MAGIC, 24U, int) /*timeout write lock (unsupport)*/
#define MDRV_SEMUTEX_WRLOCK_RWLOCK                           _IOR(SEMUTEX_IOC_MAGIC, 25U, int) /*blocking write lock*/

#define MDRV_SEMUTEX_RDUNLOCK                                _IOR(SEMUTEX_IOC_MAGIC, 26U, int) /*unlock*/
#define MDRV_SEMUTEX_WRUNLOCK                                _IOR(SEMUTEX_IOC_MAGIC, 27U, int) /*unlock*/

#define SEMA_NAME_LEN   64

typedef struct{
int index;
int time;
}LOCKARG;

typedef struct{
    int     semanum;
    char    semaname[SEMA_NAME_LEN];
}CREATE_SEMA_ARG;

typedef enum
{
    E_CROSS_THREAD_UNLOCK_ENABLE = 0,
    E_CROSS_THREAD_UNLOCK_DISABLE,
}CROSS_THREAD_UNLOCK_FLAG;

typedef struct{
    int index;
    CROSS_THREAD_UNLOCK_FLAG flag;
}CROSS_THREAD_UNLOCK_INFO;

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------

#endif // SEMUTEX

