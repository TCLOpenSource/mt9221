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
 * mdrv_voc_io.h
 *
 *  Created on: Jul 12, 2016
 *      Author: trevor.wu
 */

#ifndef MDRV_VOC_IO_H_
#define MDRV_VOC_IO_H_


#define VOICE_IOCTL_MAGIC               'V'

#define MDRV_VOICE_IOCTL_LOAD_IMAGE		  _IO(VOICE_IOCTL_MAGIC, 0x0)
#define MDRV_VOICE_IOCTL_VQ_ENABLE		  _IOW(VOICE_IOCTL_MAGIC, 0x1, int)
#define MDRV_VOICE_IOCTL_VQ_CONFIG		  _IOW(VOICE_IOCTL_MAGIC, 0x2, VQ_CONFIG_S)
#define MDRV_VOICE_IOCTL_VP_ENABLE		  _IOW(VOICE_IOCTL_MAGIC, 0x3, int)
#define MDRV_VOICE_IOCTL_VP_CONFIG		  _IOW(VOICE_IOCTL_MAGIC, 0x4, VP_CONFIG_S)
#define MDRV_VOICE_IOCTL_SLEEP_CM4        _IOW(VOICE_IOCTL_MAGIC, 0x5, int)
#define MDRV_VOICE_IOCTL_DMIC_GAIN        _IOW(VOICE_IOCTL_MAGIC, 0x6, char)
#define MDRV_VOICE_IOCTL_SINEGEN_ENABLE   _IOR(VOICE_IOCTL_MAGIC, 0x7, int)
#define MDRV_VOICE_IOCTL_KEYWORD_MATCH    _IOR(VOICE_IOCTL_MAGIC, 0x8, int)


#define VOICE_IOCTL_MAXNR 8

#endif /* MDRV_VOC_IO_H_ */
