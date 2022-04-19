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
/// file    reg_sc_ld.h
/// @brief  Smart Card Module Register Definition
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _REG_SC_LD_H_
#define _REG_SC_LD_H_


//-------------------------------------------------------------------------------------------------
//  Hardware Capability
//-------------------------------------------------------------------------------------------------
#define MHAL_LD_PACKLENGTH    (16)    //monaco 16, muji 16
#define MHAL_LD_MIU_BUS       (16)

#define REG_2E04 	(0x132E00 | (0x04))
#define REG_2E06 	(0x132E00 | (0x06))
#define REG_2E08 	(0x132E00 | (0x08))
#define REG_2E0C 	(0x132E00 | (0x0C))
#define REG_2E10 	(0x132E00 | (0x10))
#define REG_2E14 	(0x132E00 | (0x14))

#define REG_2E3E 	(0x132E00 | (0x3E))
#define REG_2E3F 	(0x132E00 | (0x3F))

#define REG_2E44 	(0x132E00 | (0x44))
#define REG_2E45 	(0x132E00 | (0x45))
#define REG_2E46 	(0x132E00 | (0x46))
#define REG_2E47 	(0x132E00 | (0x47))
#define REG_2E48 	(0x132E00 | (0x48))
#define REG_2E49 	(0x132E00 | (0x49))
#define REG_2E4A 	(0x132E00 | (0x4A))
#define REG_2E4B 	(0x132E00 | (0x4B))
#define REG_2E4C 	(0x132E00 | (0x4C))
#define REG_2E4D 	(0x132E00 | (0x4D))
#define REG_2E4E 	(0x132E00 | (0x4E))
#define REG_2E4F 	(0x132E00 | (0x4F))

#define REG_2E6E 	(0x132E00 | (0x6E))

#define REG_2E80 	(0x132E00 | (0x80))
#define REG_2E81 	(0x132E00 | (0x81))
#define REG_2E82 	(0x132E00 | (0x82))
#define REG_2E83 	(0x132E00 | (0x83))
#define REG_2E84 	(0x132E00 | (0x84))
#define REG_2E86 	(0x132E00 | (0x86))
#define REG_2E87 	(0x132E00 | (0x87))
#define REG_2E88 	(0x132E00 | (0x88))
#define REG_2E89 	(0x132E00 | (0x89))

#define REG_2E9A 	(0x132E00 | (0x9A))
#define REG_2E9E 	(0x132E00 | (0x9E))

#define REG_2EA2 	(0x132E00 | (0xA2))
#define REG_2EA6 	(0x132E00 | (0xA6))

#define REG_2EC6 	(0x132E00 | (0xC6))

#define REG_2EE4 	(0x132E00 | (0xE4))
#define REG_2EED 	(0x132E00 | (0xED))

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define _BIT0			(0x0001)
#define _BIT1			(0x0002)
#define _BIT2			(0x0004)
#define _BIT3			(0x0008)
#define _BIT4			(0x0010)
#define _BIT5			(0x0020)
#define _BIT6			(0x0040)
#define _BIT7			(0x0080)



#ifndef BIT
#define BIT(x) (1UL << (x))
#endif

#if defined(CONFIG_ARM64)
//extern ptrdiff_t mstar_pm_base;
#define REG_ADDR(addr)  (*((volatile unsigned short *)(mstar_pm_base + (addr << 1))))
#else
#define REG_ADDR(addr)  (*((volatile unsigned short *)(0xFD000000 + (addr << 1))))
#endif


// read 2 byte
#define REG_RR(_reg_)                           ({REG_ADDR(_reg_);})

// write low byte
#define REG_WL(_reg_, _val_)    \
        do{ REG_ADDR(_reg_) = (REG_ADDR(_reg_) & 0xFF00) | ((_val_) & 0x00FF); }while(0)

// write high byte
#define REG_WH(_reg_, _val_)    \
        do{ REG_ADDR(_reg_) = (REG_ADDR(_reg_)  & 0x00FF) | ((_val_) << 8); }while(0)

 // write 2 byte
#define REG_W2B(_reg_, _val_)    \
        do{ REG_ADDR(_reg_) =(_val_) ; }while(0)


#endif // _REG_SC_LD_H_
