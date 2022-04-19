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
// @file   mdrv_mpif_st.h
// @brief  MPIF Driver Interface
// @author MStar Semiconductor Inc.
//
// Driver to initialize and access mailbox.
//     - Provide functions to initialize/de-initialize mailbox
//     - Provide mailbox functional access.
//////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _MDRV_MPIF_ST_H
#define _MDRV_MPIF_ST_H

//=============================================================================
// Includs
//=============================================================================
#include <asm/types.h>
#include "MsTypes.h"
#include "mdrv_mpif.h"

//=============================================================================
// Type and Structure Declaration
//=============================================================================
typedef struct _MS_MPIF_2XCFG
{
	MS_U8 		   slaveid;
	DRV_MPIF_2XCTL  *pst2xctl;
}MS_MPIF_2XCFG;

typedef struct _MS_MPIF_3XCFG
{
	MS_U8 		   slaveid;
	DRV_MPIF_3XCTL  *pst3xctl;
}MS_MPIF_3XCFG;

typedef struct _MS_MPIF_4ACFG
{
	MS_U8 		   slaveid;
	DRV_MPIF_4ACTL  *pst4Actl;
}MS_MPIF_4ACFG;

typedef struct _MS_MPIF_1ARW
{
	MS_U8 		   			slaveid;
	MS_BOOL		   			bWrite;
	MS_U8		   			u8index;
	MS_U8		   			*pu8data;
	DRV_MPIF_DrvStatus		Res;
}MS_MPIF_1ARW;

typedef struct _MS_MPIF_2XRW
{
	MS_U8 		   				slaveid;
	MS_BOOL		   				bWrite;
	MS_U16		   				u16addr;
	MS_U16		   				*pu16data;
	DRV_MPIF_DrvStatus		    Res;
}MS_MPIF_2XRW;

typedef struct _MS_MPIF_3XRIURW
{
	MS_U8 		   				slaveid;
	MS_BOOL		   				bWrite;
	MS_U8		   				u8datalen;
	MS_U8*		   				pu8data;
	DRV_MPIF_DrvStatus		    Res;
}MS_MPIF_3XRIURW;

typedef struct _MS_MPIF_MIURW
{
	MS_U8 		   				slaveid;
	MS_BOOL		   				bWrite;
	MS_U32		   				u32datalen;
	MS_PHYADDR	   				u32miu_addr;
	MS_PHYADDR	   				u32spif_miuaddr;
}MS_MPIF_MIURW;

typedef struct _MS_MPIF_BUS_PARAM
{
	MS_U8 		   u8slaveid;
	MS_U8		   u8cmdwidth;
	MS_U8		   u8datawidth;
}MS_MPIF_BUS_PARAM;

typedef struct _MS_MPIF_1BYTE_PARAM
{
	MS_U8 		   u8slaveid;
	MS_U8		   u8data;
}MS_MPIF_1BYTE_PARAM;






#endif //_MDRV_MPIF_ST_H
