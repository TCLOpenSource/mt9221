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

#ifndef __DRV_SYSTEM_IO_MIU1_H__
#define __DRV_SYSTEM_IO_MIU1_H__


#if defined(CONFIG_MSTAR_TITANIA_MMAP_64MB)
#define  MIU1_OFFSET      0
#elif defined(CONFIG_MSTAR_TITANIA_MMAP_64MB_64MB)
#define  MIU1_OFFSET      0
#elif defined(CONFIG_MSTAR_MMAP_128MB_128MB_DEFAULT)
#define  MIU1_OFFSET      0
#elif defined(CONFIG_MSTAR_OBERON_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0
#elif defined(CONFIG_MSTAR_EUCLID_FPGA_MMAP_128MB)
#define  MIU1_OFFSET      0
#elif defined(CONFIG_MSTAR_EUCLID_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0
#elif defined(CONFIG_MSTAR_TITANIA3_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x08000000
#elif defined(CONFIG_MSTAR_TITANIA10_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x08000000
#elif defined(CONFIG_MSTAR_TITANIA4_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x08000000
#elif defined(CONFIG_MSTAR_TITANIA4_MMAP_128MB)
#define  MIU1_OFFSET      0x08000000
#elif defined(CONFIG_MSTAR_TITANIA8_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA8_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA8_MMAP_512MB_512MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA8_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA9_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA9_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA9_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA11_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA11_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA11_MMAP_512MB_512MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA11_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA12_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA12_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA12_MMAP_512MB_512MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA12_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA13_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA13_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_TITANIA13_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER1_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER1_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER1_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER2_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER2_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER2_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER5_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER5_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER5_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_URANUS4_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x08000000
#elif defined(CONFIG_MSTAR_JANUS2_MMAP_64MB_64MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_JANUS2_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_JANUS2_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_JANUS2_MMAP_512MB_512MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_KRONUS_MMAP_256MB)
#define  MIU1_OFFSET      0x8000000
#elif defined(CONFIG_MSTAR_KRONUS_MMAP_128MB)
#define  MIU1_OFFSET      0x8000000
#elif defined(CONFIG_MSTAR_KRONUS_MMAP_64MB)
#define  MIU1_OFFSET      0x8000000
#elif defined(CONFIG_MSTAR_KAISERIN_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER6_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER6_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER6_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER7_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER7_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER7_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMETHYST_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMETHYST_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_AMBER3_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_AMBER3_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_AMBER3_MMAP_512MB_512MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_AMBER3_MMAP_128MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_AGATE_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_AGATE_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_AGATE_MMAP_512MB_512MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_AGATE_MMAP_128MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_EAGLE_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_EAGLE_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_EAGLE_MMAP_512MB_512MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_EAGLE_MMAP_128MB)
#define  MIU1_OFFSET      0x00000000
#elif defined(CONFIG_MSTAR_EMERALD_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_EMERALD_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_EMERALD_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_NUGGET_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_NUGGET_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_NUGGET_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_NIKON_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_NIKON_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_NIKON_MMAP_128MB)
#define  MIU1_OFFSET      0x20000000
#elif defined(CONFIG_MSTAR_ARM_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_ARM_MMAP_256MB_256MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_ARM_MMAP_512MB_512MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_ARM_MMAP_128MB)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_ARM_MMAP_GENERIC)
#define  MIU1_OFFSET      0x60000000
#elif defined(CONFIG_MSTAR_MIPS_MMAP_128MB_128MB)
#define  MIU1_OFFSET      0x20000000
#endif

#endif // __DRV_SYSTEM_IO_MIU1_H__

