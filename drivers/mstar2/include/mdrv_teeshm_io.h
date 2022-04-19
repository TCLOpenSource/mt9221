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
/// @file   mdrv_mpool_io.h
/// @brief  Memory Pool  Driver IO Interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_TEESHM_IO_H_
#define _MDRV_TEESHM_IO_H_

/* Use 'S' as magic number(0x53) */
#define TEESHM_IOC_MAGIC                'S'

/* <--- 8_bit --->  <--- 8_bit --->  <--- 8_bit --->  <--- 8_bit ---> */
/*     DIR(R/W)         PARA_SIZE        TYPE(0x53)       NR(0x00~0x05) */
#define TEESHM_IOC_RESOURCE_CREATE			_IOWR(TEESHM_IOC_MAGIC, 0x00, DrvTeeShmResourceCreate)			// having input and output, using IOWR,		TEESHM_IOC_RESOURCE_CREATE is0xC0105300
#define TEESHM_IOC_RESOURCE_DESTROY			_IOW(TEESHM_IOC_MAGIC, 0x01, DrvTeeShmResourceDestroy)			// only input, using _IOW
#define TEESHM_IOC_RESOURCE_OBTAIN			_IOW(TEESHM_IOC_MAGIC, 0x02, DrvTeeShmResourceObtain)
#define TEESHM_IOC_RESOURCE_RELEASE			_IOW(TEESHM_IOC_MAGIC, 0x03, DrvTeeShmResourceRelease)
#define TEESHM_IOC_RESOURCE_MAP				_IOWR(TEESHM_IOC_MAGIC, 0x04, DrvTeeShmResourceMap)				//											TEESHM_IOC_RESOURCE_MAP is 0xC0105304
#define TEESHM_IOC_RESOURCE_UNMAP			_IOW(TEESHM_IOC_MAGIC, 0x05, DrvTeeShmResourceUnmap)			//											TEESHM_IOC_RESOURCE_UNMAP is 0x40105305

/* compat ioctl */
#ifdef CONFIG_COMPAT
#define COMPAT_TEESHM_IOC_RESOURCE_CREATE		_IOWR(TEESHM_IOC_MAGIC, 0x00, DrvTeeShmResourceCreate_32)
#define COMPAT_TEESHM_IOC_RESOURCE_MAP			_IOWR(TEESHM_IOC_MAGIC, 0x04, DrvTeeShmResourceMap_32)
#define COMPAT_TEESHM_IOC_RESOURCE_UNMAP		_IOW(TEESHM_IOC_MAGIC, 0x05, DrvTeeShmResourceUnmap_32)
#endif

#endif
