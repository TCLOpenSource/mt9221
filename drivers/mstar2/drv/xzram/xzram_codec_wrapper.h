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

#ifndef _XZRAM_CODEC_WRAPPER__H
#define _XZRAM_CODEC_WRAPPER__H

#define GCC_VERSION (__GNUC__ * 100 + __GNUC_MINOR__)

#if (GCC_VERSION == 301)
#ifdef CONFIG_XZRAM_COMPRESS_LZ4
#define XZRAM_COMPRESS_LZ4
#endif
#endif

#ifdef XZRAM_COMPRESS_LZ4
#include <linux/lz4.h>
#else
#include <linux/lzo.h>
#endif

enum  ZRAM_CODEC_ENUM {
    ZRAM_CODEC_OK,
    ZRAM_CODEC_ERROR,
    ZRAM_CODEC_OUT_OF_MEMORY,
    ZRAM_CODEC_NOT_COMPRESSIBLE,
    ZRAM_CODEC_INPUT_OVERRUN,
    ZRAM_CODEC_OUTPUT_OVERRUN,
    ZRAM_CODEC_LOOKBEHIND_OVERRUN,
    ZRAM_CODEC_EOF_NOT_FOUND,
    ZRAM_CODEC_INPUT_NOT_CONSUMED,
    ZRAM_CODEC_NOT_YET_IMPLEMENTED,
    ZRAM_CODEC_INVALID_ARGUMENT,
    ZRAM_CODEC_ERROR_CHECKSUM
};

int compress_func(const char* src_buf, int src_len,char* dst_buf, int* dst_len,  void* work_buf);
int decompres_func(const char* src_buf, int src_len,char* dst_buf, int* dst_len);
u64 gettime(void);

#ifdef XZRAM_COMPRESS_LZ4
#define ZRAM_CODEC_MEM (1<<MEMORY_USAGE)
#else
#define ZRAM_CODEC_MEM LZO1X_MEM_COMPRESS
#endif
#endif
