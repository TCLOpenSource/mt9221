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

/*-----------------------------------------------------------------------------
 *
 * $Author: dtvbm11 $
 * $Date: 2015/04/15 $
 * $RCSfile: mt53xx_cb2.h,v $
 * $Revision: #1 $
 *
 *---------------------------------------------------------------------------*/

/** @file cb-mt53xx.h
 *  Define callback interface.
 */

#ifndef CB2_MT53XX_H
#define CB2_MT53XX_H


//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------

#include "cb2_data.h"
#include "x_typedef.h"
#include <linux/ioctl.h>
#include <linux/types.h>

//-----------------------------------------------------------------------------
// Constant definitions
//-----------------------------------------------------------------------------

#define CBR_OK          ((int) 0)
#define CBR_ERR_DEV     ((int)-1)
#define CBR_ERR_SYS     ((int)-2)
#define CBR_NO_MEM      ((int)-3)
#define CBR_NOT_EXIST   ((int)-4)
#define CBR_ERR_REG     ((int)-5)

#define CB_MAX_STRUCT_SIZE  4096

//CB IOCTYPE
#define CB_IOCTYPE		55
//-----------------------------------------------------------------------------
// Type definitions
//-----------------------------------------------------------------------------
/// Ioctl data structure of CB_SET_CALLBACK
typedef struct
{
    CB_FCT_ID_T     eFctId;
    int             i4SubId;
    pid_t           rCbPid;
} CB_SET_CALLBACK_T;

/// Ioctl data structure of CB_GET_CALLBACK
typedef struct
{
    CB_FCT_ID_T     eFctId;
    int             i4TagSize;
    union 
	{
        void     *pvTag;       
        unsigned long long  padding1;
    };
} CB_GET_CALLBACK_T;

typedef struct 
{
  CB_FCT_ID_T     eFctId;
  pid_t           rCbPid;
}CB_GET_PID_T;

typedef struct 
{
  //CB_FCT_ID_T     eFctId;
  //pid_t           rCbPid;
  int             iCount;
}CB_GET_CBQUEUE_COUNT_T;

/// Ioctl command of device node.
//CB IOCTL numbers

#define    CB_SET_CALLBACK 				_IOWR(CB_IOCTYPE, 0, CB_SET_CALLBACK_T)
#define    CB_GET_CALLBACK				_IOWR(CB_IOCTYPE, 1, CB_GET_CALLBACK_T)
#define    CB_UNSET_CALLBACK			_IOWR(CB_IOCTYPE, 2, CB_SET_CALLBACK_T)
#define    CB_GET_PID					_IOWR(CB_IOCTYPE, 3, CB_GET_PID_T)
#define    CB_GET_CBQUEUE_COUNT			_IOWR(CB_IOCTYPE, 4, CB_GET_CBQUEUE_COUNT_T)
#define    CB_SET_TEMINATE				_IOWR(CB_IOCTYPE, 5, INT32)
#define    CB_SET_MULTI_CALLBACK		_IOWR(CB_IOCTYPE, 6, CB_SET_CALLBACK_T)
#define    CB_UNSET_MULTI_CALLBACK		_IOWR(CB_IOCTYPE, 7, CB_SET_CALLBACK_T)
#define    CB_SEND_CALLBACK		        _IOWR(CB_IOCTYPE, 8, CB_SET_CALLBACK_T)

#endif  //CB_MT53XX_H

