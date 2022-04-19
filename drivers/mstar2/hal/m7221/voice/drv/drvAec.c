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

#include "mosWrapper.h"

#include "drvMBX.h"
#include "drvAec.h"
#include "halCHIPTOP.h"

#define VOC_MSG(fmt, args...) MOS_DBG_PRINT(MOS_DBG_LEVEL_VOC, "[VOC]" fmt, ##args)
#define VOC_ERROR(fmt, args...) MOS_DBG_ERROR("[VOC ERROR]" fmt, ##args);

////////////////////////////////////////////////////////////////////////////////////////

enum
{
  AEC_MODE_NONE = 0,
  AEC_MODE_48KMONO,
  AEC_MODE_48KSTEREO,
  AEC_MODE_16KMONO,
  AEC_MODE_16KSTEREO,
  AEC_MODE_LEN,
};

typedef struct {
  u32 u32Magic;
  u32 u32BufPtr;
  u32 u32BufSize;
  u32 u32BufWptr;
  u64 u64StartTime;
  u32 u32BufPhy;
  struct mutex aec_lock;
} AEC_RefInfo;

typedef struct
{
  u32 u32Channels;
  u32 u32SampleRate;
  u32 u32SampleWidth;
  u32 u32CapChannels;
  u32 u32RefChannels;
  u64 u64RefStartTime;
  u64 u64CapStartTime;
  u32 u32ShareAddr;	/* physical address */
  u32 u32ShareArea;	/* virtual pointer */
  u32 u32ShareBytes;/* buffer size in bytes */
  spinlock_t tlock;
} AEC_Mgr;

typedef struct
{
  u32 u32RefArea;	/* virtual pointer */
  u32 u32RefRdPtr;		/* read virtual pointer  */
  u32 u32RefBytes;		/* buffer size in bytes */
} AEC_Buf;

static AEC_Mgr _mgrAEC ={.u32SampleRate = 16000, .u32SampleWidth = 16, .u32CapChannels = 2, .u32RefChannels = 0};
static AEC_Buf _bufAEC;
static AEC_RefInfo *_ptRefInfoAEC;
//static AEC_RefInfo _tRefInfoAEC; //for test

static u16 g_aAecMap[252000];
static u32 g_u32AecMapFrames;

bool VocAecConfig(u32 nChannels, u32 nSampleRate, u32 nSampleWidth)
{
  char name[16];
  //int i;
  //s16 *ptr;

  if ((nSampleRate != 16000) || (nSampleWidth != 16))
  {
    VOC_MSG("VocAecConfig: parameter not support\n");
    return false;
  }

  if (_mgrAEC.u32RefChannels + _mgrAEC.u32CapChannels != nChannels)
  {
    VOC_MSG("VocAecConfig: channels not match\n");
    return false;
  }

  _mgrAEC.u32Channels = nChannels;
  _mgrAEC.u32SampleRate = nSampleRate;
  _mgrAEC.u32SampleWidth = nSampleWidth;
  _mgrAEC.u64RefStartTime = 0;
  _mgrAEC.u32ShareAddr = 0;
  _mgrAEC.u32ShareArea = 0;
  _mgrAEC.u32ShareBytes = 0;
  _ptRefInfoAEC = NULL;

  g_u32AecMapFrames = sizeof(g_aAecMap)/(_mgrAEC.u32SampleWidth/8 * _mgrAEC.u32Channels);

  if(_mgrAEC.u32RefChannels)
  {
    _mgrAEC.u32ShareBytes = 96*1024*4; //to fix 1s
    snprintf(name, 16, "aec_ref_buf");
#if defined(__VOICE_MSYS_ALLOC__)
    _mgrAEC.u32ShareArea = (u32)alloc_dmem(name, PAGE_ALIGN(sizeof(AEC_RefInfo)) + _mgrAEC.u32ShareBytes, &_mgrAEC.u32ShareAddr);
#endif
    if (!_mgrAEC.u32ShareArea)
    {
        VOC_MSG("VocAecConfig: AEC buffer alloacttion failed\n");
        return false;
    }
    _mgrAEC.u32ShareBytes = PAGE_ALIGN(sizeof(AEC_RefInfo)) + _mgrAEC.u32ShareBytes;

    VOC_MSG("VocAecConfig: AEC share buffer size 0x%x\n", _mgrAEC.u32ShareBytes);
    VOC_MSG("VocAecConfig: physical AEC address 0x%08x\n", _mgrAEC.u32ShareAddr);
    VOC_MSG("VocAecConfig: virtual AEC address 0x%08x\n", _mgrAEC.u32ShareArea);

    _ptRefInfoAEC = (AEC_RefInfo *)_mgrAEC.u32ShareArea;

    //spin_lock_init(&g_tVocAeclock);
/*

    ptr = (s16 *)_mgrAEC.u32ShareArea;
    for (i = 0; i < _mgrAEC.u32ShareBytes/4; i++)
    {
      *(ptr+i*2) = i%16000;
      *(ptr+i*2+1) = i%16000;
    }

    VocAecSetRefInfo(_mgrAEC.u32ShareArea, _mgrAEC.u32ShareBytes); //test
*/
  }

  _bufAEC.u32RefRdPtr = 0;
  _bufAEC.u32RefBytes = 0;
  _bufAEC.u32RefArea = 0;
  return true;
}

s32 VocCapFramesToBytes(s32 size)
{
  return size*_mgrAEC.u32CapChannels*_mgrAEC.u32SampleWidth/8;
}

s32 VocCapBytesToFrames(s32 size)
{
  return size*8/(_mgrAEC.u32CapChannels*_mgrAEC.u32SampleWidth);
}

bool VocAecSetCapMode(u32 u32Mode)
{
  if (u32Mode == 0)
    _mgrAEC.u32CapChannels = 2;
  else if (u32Mode == 1)
    _mgrAEC.u32CapChannels = 4;
  else if (u32Mode == 2)
    _mgrAEC.u32CapChannels = 6;
  else if (u32Mode == 3)
    _mgrAEC.u32CapChannels = 8;
  else
  {
    VOC_ERROR("VocSetCapMode: parameter not support\n");
    return false;
  }

  return true;
}

bool VocAecSetRefMode(u32 u32Mode)
{
  if (u32Mode == AEC_MODE_NONE)
    _mgrAEC.u32RefChannels = 0;
  else if (u32Mode == AEC_MODE_48KMONO)
    _mgrAEC.u32RefChannels = 3;
  else if (u32Mode == AEC_MODE_48KSTEREO)
    _mgrAEC.u32RefChannels = 6;
  else if (u32Mode == AEC_MODE_16KMONO)
    _mgrAEC.u32RefChannels = 1;
  else if (u32Mode == AEC_MODE_16KSTEREO)
    _mgrAEC.u32RefChannels = 2;
  else
  {
    VOC_ERROR("VocSetRefMode: parameter not support\n");
    return false;
  }

  return true;
}

/*
void VocAecSetRefInfo(u32 u32RefArea, u32 u32RefBytes)
{
  _tRefInfoAEC.u32Magic = 0x78787878;
  _tRefInfoAEC.u32BufPtr = u32RefArea;
  _tRefInfoAEC.u32BufSize = u32RefBytes;
  _tRefInfoAEC.u64StartTime = Chip_Get_US_Ticks();

  VOC_PRINTF(DEBUG_LEVEL, "VocAecSetRefBuf: Ref buffer 0x%x, size 0x%x\n", _tRefInfoAEC.u32BufPtr, _tRefInfoAEC.u32BufSize);
  VOC_PRINTF(DEBUG_LEVEL, "VocAecSetRefBuf: Ref start time 0x%llx\n", _tRefInfoAEC.u64StartTime);

  _ptRefInfoAEC = &_tRefInfoAEC;
}
*/
void VocAecSetCapStartTime(u64 u64TimeStamp)
{
  //unsigned long flag;
  //spin_lock_irqsave(&g_tVocAeclock, flag);
  _mgrAEC.u64CapStartTime = u64TimeStamp;
  //g_u8VocAecCapStartTimeChanged = 1;
  //spin_unlock_irqrestore(&g_tVocAeclock, flag);
  VOC_MSG("VocAecSetCapStartTime: Cap start time 0x%llx\n", _mgrAEC.u64CapStartTime);
}

void VocAecUpdateRefInfo(void)
{
  u64 u64Ptr;
  if (_ptRefInfoAEC && _ptRefInfoAEC->u32Magic == 0x78787878)
  {
    mutex_lock(&_ptRefInfoAEC->aec_lock);
    if (!_bufAEC.u32RefArea)
    {
      _bufAEC.u32RefBytes = _ptRefInfoAEC->u32BufSize;
      _bufAEC.u32RefArea = _ptRefInfoAEC->u32BufPtr;
      VOC_MSG("VocAecUpdateRefInfo: AEC ref buffer size 0x%x\n", _bufAEC.u32RefBytes);
      VOC_MSG("VocAecUpdateRefInfo: AEC ref buffer address 0x%08x\n", _bufAEC.u32RefArea);
    }

    if (!_ptRefInfoAEC->u64StartTime)//stop or xrun
    {
      _mgrAEC.u64RefStartTime = 0;
      _bufAEC.u32RefRdPtr = 0;
      VOC_MSG("VocAecUpdateRefInfo: AEC ref start time 00000000000000000\n");
    }
    else if (_mgrAEC.u64RefStartTime < _ptRefInfoAEC->u64StartTime) //runing status
    {
      if (_ptRefInfoAEC->u64StartTime+200000 <= _mgrAEC.u64CapStartTime)
      {
        _mgrAEC.u64RefStartTime = _ptRefInfoAEC->u64StartTime;
        VOC_MSG("VocAecUpdateRefInfo: shift time 0x%llx us\n", _mgrAEC.u64CapStartTime - _mgrAEC.u64RefStartTime);
        u64Ptr = div_u64((_mgrAEC.u64CapStartTime - _mgrAEC.u64RefStartTime - 200000)*10, 625)*_mgrAEC.u32SampleWidth/8*_mgrAEC.u32RefChannels;
        VOC_MSG("VocAecUpdateRefInfo: shift 0x%llx\n", u64Ptr);
        div_u64_rem(u64Ptr, _bufAEC.u32RefBytes, &_bufAEC.u32RefRdPtr);
        _bufAEC.u32RefRdPtr += _bufAEC.u32RefArea;
        VOC_MSG("VocAecUpdateRefInfo: start ref = 0x%x, ref data = 0x%x\n", _bufAEC.u32RefRdPtr,  _ptRefInfoAEC->u32BufWptr);
      }
      else
        _bufAEC.u32RefRdPtr = 0;
    }
    mutex_unlock(&_ptRefInfoAEC->aec_lock);

  }
  else
  {
    _bufAEC.u32RefRdPtr = 0;
    _bufAEC.u32RefBytes = 0;
    _bufAEC.u32RefArea = 0;
  }
}

void VocAecReset(void)
{
  _mgrAEC.u64RefStartTime = 0;
  _bufAEC.u32RefRdPtr = 0;
  VOC_MSG("VocAecResetRefStartTime\n");
}

u32 VocAecCopy(u16 **ppu16AecMap, u16 *pu16Cap, u32 u32FrameCount)
{
    int i;
    //int j;
    if (u32FrameCount > g_u32AecMapFrames)
      u32FrameCount = g_u32AecMapFrames;

    //VOC_PRINTF(DEBUG_LEVEL, "VocAecCopy: cap frame_pos = 0x%x, frame_count = 0x%x\n",
    //           (u32)pu16Cap, u32FrameCount);

    for (i = 0; i < u32FrameCount; ++i) {
        //g_aAecMap[i * _mgrAEC.u32Channels] = pu16Cap[i * _mgrAEC.u32CapChannels];
        //g_aAecMap[i * _mgrAEC.u32Channels + 1] = pu16Cap[i * _mgrAEC.u32CapChannels + 1];
        memcpy(&g_aAecMap[i * _mgrAEC.u32Channels], &pu16Cap[i * _mgrAEC.u32CapChannels], _mgrAEC.u32CapChannels*2);

        if (_bufAEC.u32RefRdPtr)
        {
          //mutex_lock(&_ptRefInfoAEC->aec_lock);
          memcpy(&g_aAecMap[i * _mgrAEC.u32Channels + _mgrAEC.u32CapChannels], (unsigned short *)_bufAEC.u32RefRdPtr, _mgrAEC.u32RefChannels*2);
          _bufAEC.u32RefRdPtr += _mgrAEC.u32RefChannels*2;
          if (_bufAEC.u32RefRdPtr == (_bufAEC.u32RefArea + _bufAEC.u32RefBytes))
            _bufAEC.u32RefRdPtr = _bufAEC.u32RefArea;
          //mutex_unlock(&_ptRefInfoAEC->aec_lock);

        }
        else
        {
          memset(&g_aAecMap[i * _mgrAEC.u32Channels + _mgrAEC.u32CapChannels], 0, _mgrAEC.u32RefChannels*2);
        }
    }

    _mgrAEC.u64CapStartTime += u32FrameCount*625/10;

    //VOC_MSG("VocAecCopy: ref ptr = 0x%x, count = %d\n", _bufAEC.u32RefRdPtr, u32FrameCount);
    //VOC_PRINTF(DEBUG_LEVEL, "VocAecCopy: Cap start time = 0x%llx, t = 0x%llx us\n", _mgrAEC.u64CapStartTime, Chip_Get_US_Ticks()); //blocking risk in Chip_Get_US_Ticks

    *ppu16AecMap = g_aAecMap;
    return u32FrameCount;
}


