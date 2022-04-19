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

/*
 * $Id: //DAILEO/Columbus/IPCamera/source/iNfinity/iNfinity_ROM/source/include/std_c.h#1 $
 * $Header: //DAILEO/Columbus/IPCamera/source/iNfinity/iNfinity_ROM/source/include/std_c.h#1 $
 * $Date: 2015/05/27 $
 * $DateTime: 2015/05/27 17:11:47 $
 * $Change: 1236156 $
 * $File: //DAILEO/Columbus/IPCamera/source/iNfinity/iNfinity_ROM/source/include/std_c.h $
 * $Revision: #1 $
 */


#ifndef	STD_C_H
#define STD_C_H

#if 0
    /// data type unsigned char, data length 1 byte
    typedef unsigned char               MS_U8;                              // 1 byte
    /// data type unsigned short, data length 2 byte
    typedef unsigned short              MS_U16;                             // 2 bytes
    /// data type unsigned int, data length 4 byte
    typedef unsigned long               MS_U32;                             // 4 bytes
    /// data type unsigned int, data length 8 byte
    typedef unsigned long long          MS_U64;                             // 8 bytes
    /// data type signed char, data length 1 byte
    typedef signed char                 MS_S8;                              // 1 byte
    /// data type signed short, data length 2 byte
    typedef signed short                MS_S16;                             // 2 bytes
    /// data type signed int, data length 4 byte
    typedef signed long                 MS_S32;                             // 4 bytes
    /// data type signed int, data length 8 byte
    typedef signed long long            MS_S64;                             // 8 bytes
    /// data type float, data length 4 byte
    typedef float                       MS_FLOAT;                           // 4 bytes
    /// data type pointer content
    typedef MS_U32                      MS_VIRT;                            // 8 bytes
    /// data type hardware physical address
    typedef MS_U32                      MS_PHYADDR;                         // 8 bytes
    /// data type 64bit physical address
    typedef MS_U32                      MS_PHY;                             // 8 bytes
    /// data type size_t
    typedef MS_U32                      MS_SIZE;                            // 8 bytes
    /// definition for MS_BOOL
    typedef unsigned char               MS_BOOL;                            // 1 byte


typedef unsigned int    Primitive;
typedef unsigned char   bool;
typedef unsigned char   u8;
typedef signed   char   s8;
typedef          char   ascii;
typedef unsigned short  u16;
typedef          short  s16;
typedef unsigned int    u32;
typedef          int    s32;
typedef unsigned long long u64;
typedef 	 long long s64;

#endif

typedef int Bool;
#define True 1
#define False 0

    /***
    ****	Interpretation definition
    ***/

#define TRUE	1
#define FALSE	0

#ifdef NULL
    #undef NULL
#endif
#define NULL	0

    /***
    ****	type definition
    ***/
#define U8	u8//unsigned char
#define U16	u16//unsigned short
#define U32 u32//unsigned long
//#define U64 __int64
#define U64 u64//unsigned long long int
#if 0
#define UCHAR	unsigned char
#define USHORT	unsigned short
#define UINT		unsigned long
#define ULONG	 __int64

#define UINT8 unsigned char
#define UINT16 unsigned short
#define UINT32 unsigned long
#define UINT8_PTR UINT8 *
#define UINT16_PTR UINT16 *

#define UINT32_PTR UINT32 *
#endif
#define S8  s8//signed char
#define S16 s16//signed short
#define S32 s32//signed long
#define S64 s64//signed long long

#define BOOLEAN unsigned char
#define BOOL unsigned char

/* Macro for TDA1236D		                         							*/
/********************************************************************************/
#define Data8	U8
#define Data16	S16
#define Data32	S32
//#define bool	BOOLEAN
#define Bool	BOOLEAN
//#define BOOL    BOOLEAN
//#define true 	TRUE
//#define false 	FALSE


#define BIT0        0x1
#define BIT1        0x2
#define BIT2        0x4
#define BIT3        0x8
#define BIT4        0x10
#define BIT5        0x20
#define BIT6        0x40
#define BIT7        0x80
#define BIT8        0x100
#define BIT9        0x200
#define BIT10       0x400
#define BIT11       0x800
#define BIT12       0x1000
#define BIT13       0x2000
#define BIT14       0x4000
#define BIT15       0x8000



#define MIN(A, B) (((A) < (B)) ? (A) : (B))
#define MAX(A, B) (((A) > (B)) ? (A) : (B))
#define ABS(X)    ((X>=0) ? (X) : (-X))
#define COUNTOF( array )    (sizeof(array) / sizeof((array)[0]))

/// defination for FALSE
//#define FALSE           0
/// defination for TRUE
//#define TRUE            1

/// defination for DISABLE
#define DISABLE     	0
/// defination for ENABLE
#define ENABLE      	1

/// 0: FAIL
#define FAIL   			0
/// 1: PASS
#define PASS   			1

/// 0: NO
#define NO     			0
/// 1: YES
#define YES    			1

#define LOWBYTE(u16)    ((U8)(u16))
#define HIGHBYTE(u16)   ((U8)((u16) >> 8))
#define HINIBBLE(u8)    ((u8) / 0x10)
#define LONIBBLE(u8)    ((u8) & 0x0F)
#define BCD2Dec(x)  	((((x) >> 4) * 10) + ((x) & 0x0F))


#define ALIGN_4(_x_)                (((_x_) + 3) & ~3)
#define ALIGN_8(_x_)                (((_x_) + 7) & ~7)
#define ALIGN_16(_x_)               (((_x_) + 15) & ~15)           // No data type specified, optimized by complier
#define ALIGN_32(_x_)               (((_x_) + 31) & ~31)           // No data type specified, optimized by complier

#define MASK(x)     (((1<<(x##_BITS))-1) << x##_SHIFT)
#ifndef BIT	//for Linux_kernel type, BIT redefined in <linux/bitops.h>
//#define BIT(_bit_)                  (1 << (_bit_))
#endif
#define BIT_(x)                     BIT(x) //[OBSOLETED] //TODO: remove it later
#define BITS(_bits_, _val_)         ((BIT(((1)?_bits_)+1)-BIT(((0)?_bits_))) & (_val_<<((0)?_bits_)))
#define BMASK(_bits_)               (BIT(((1)?_bits_)+1)-BIT(((0)?_bits_)))

#endif /* STD_C_H  */
