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

#ifndef _DRV_VOC_H
#define _DRV_VOC_H

#include "mdrv_voc_io_st.h"
#include "mosWrapper.h"

#define MIU_BASE               0x20000000

//void VocDmaInit(struct snd_pcm_substream *substream);
void VocDmaInit(U32 nDmaBufSize, U32 nPeriodSize);
void VocDmaReset(void);
U32 VocDmaGetLevelCnt(void);
U32 VocDmaTrigLevelCnt(U32 nDataSize);
U32 VocDmaUpdateLevelCnt( U32 nPeriodNum, U32 nPeriodSize);

BOOL VocDmaIsDataReady(void);
BOOL VocDmaIsXrun(void);
void VocDmaClrStatus(void);



void VocCm4Init(void);
bool VocCopyBin2Dram(void);
S32 VocUpdateCm4Fw(void);
void VocDcOn(void);
void VocDcOff(void);
void VocInitOff(void);
bool VocIsCm4ShutDown(void);

/* mailbox with CP */
S32 VocDmaInitChannel(U32 nPhysDmaAddr,
                             U32 nBufferSize,
                             U32 nChannels,
                             U32 nSampleWidth,
                             U32 nSampleRate);
S32 VocDmaStartChannel(void);

S32 VocDmaStopChannel(void);

S32 VocDmaResetAudio(void);

S32 VocDmaEnableSinegen(BOOL bEn);
S32 VocEnableVq(BOOL bEn);
S32 VocConfigVq(VQ_CONFIG_S sConfig);
S32 VocEnableVp(BOOL bEn);
S32 VocConfigVp(VP_CONFIG_S sConfig);
S32 VocEnableDa(BOOL bEn);
S32 VocEnableHpf(U8 nStage);
S32 VocConfigHpf(S8 nConfig);
S32 VocSleepCm4(BOOL bEn);
S32 VocDmicNumber(U8 nMic);
S32 VocDmicBitwidth(U8 nBitwidth);
S32 VocDmicGain(U8 nGain);
int VocDmaGetFlag(void);
U32 VocDmaBufferSize(void);
S32 VocI2sEnable(BOOL bEn);
S32 VocHwAecEnable(BOOL bEn);
void VocEnableUart(int nEn);
S32 VocSwGain(U8 nGain);

#endif
