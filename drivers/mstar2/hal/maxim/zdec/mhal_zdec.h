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

#ifndef __MHAL_ZDEC_H__
#define __MHAL_ZDEC_H__

#ifndef GET_LOWORD
#define GET_LOWORD(value)    ((unsigned short)(((unsigned int)(value)) & 0xffff))
#endif
#ifndef GET_HIWORD
#define GET_HIWORD(value)    ((unsigned short)((((unsigned int)(value)) >> 16) & 0xffff))
#endif
#ifndef ALIGN16
#define ALIGN16(value)       ((((value) + 15) >> 4) << 4)
#endif
#ifndef BITS2BYTE
#define BITS2BYTE(x)         ((x) >> 3)
#endif

#define DECODING                 0UL
#define LOADING_PRESET_DICT_MIU  1UL
#define LOADING_PRESET_DICT_RIU  2UL

#define CONTIGUOUS_MODE          0UL
#define SCATTER_MODE             1UL

#define EMMC_TABLE               0UL
#define NAND_TABLE               1UL

// These functions configure a copy of ZDEC reg table in memory.
// The configuration will not take effect until 'MHal_ZDEC_Start_Operation' is called.
void MHal_ZDEC_Conf_Reset(void);
void MHal_ZDEC_Conf_Zmem(U8 miu, U32 addr, U32 size);
void MHal_ZDEC_Conf_FCIE_Handshake(U8 enable);
void MHal_ZDEC_Conf_Input_Shift(U32 skip_words_cnt, U8 shift_byte, U8 shift_bit);
void MHal_ZDEC_Conf_Preset_Dictionary(U32 size);
void MHal_ZDEC_Conf_Contiguous_Mode(U8 obuf_miu, U32 obuf_addr, U32 obuf_size);
void MHal_ZDEC_Conf_Scatter_Mode(U8 dst_tbl_miu, U32 dst_tbl_addr, U8 nand_table, U8 in_nand_page_size, U8 out_nand_page_size);


// Applies configuration stored in memory reg table and starts ZDEC operation.
// After the call, ZDEC will be ready to receive input data.
int MHal_ZDEC_Start_Operation(U8 op_mode);


void MHal_ZDEC_Feed_Data(U8 last, U8 miu, U32 sadr, U32 size);
int MHal_ZDEC_Check_Internal_Buffer(void);
int MHal_ZDEC_Check_ADMA_Table_Done(void);
int MHal_ZDEC_Check_Last_ADMA_Table_Done(void);
int MHal_ZDEC_Check_MIU_Load_Dict_Done(void);
int MHal_ZDEC_Check_Decode_Done(void);

int MHal_ZDEC_RIU_Load_Preset_Dictionary(U8* dict);


#endif
