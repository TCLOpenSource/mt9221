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
// @file   mdrv_gflip_io.c
// @brief  GFlip KMD Driver Interface
// @author MediaTek Inc.
//////////////////////////////////////////////////////////////////////////////////////////////////

//=============================================================================
// Include Files
//=============================================================================
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/string.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/uaccess.h>
#else
#include <asm/uaccess.h>
#endif
#include <linux/of.h>
//drver header files
#include "mdrv_mstypes.h"
#include "chip_int.h"
#include "mdrv_gflip_io.h"
#include "mdrv_gflip.h"
#include "mhal_gflip.h"
#include "mdrv_gflip_interrupt.h"
#include "mdrv_gflip_ve_io.h"
#include "mst_devid.h"
#include <linux/compat.h>

//=============================================================================
// Local Defines
//=============================================================================
#define     MDRV_GFLIP_DEVICE_COUNT           1
#define     MDRV_GFLIP_NAME                           "GFLIP"

#define MAX_FILE_HANDLE_SUPPRT  64

//-------------------------------------------------------------------------------------------------
// Macros
//-------------------------------------------------------------------------------------------------
//#define     GFLIPIO_DEBUG_ENABLE
#ifdef      GFLIPIO_DEBUG_ENABLE
#define     GFLIPIO_KDBG(_fmt, _args...)        printk(KERN_WARNING _fmt, ## _args)
#define     GFLIPIO_ASSERT(_con)   do {\
                                                            if (!(_con)) {\
                                                                printk(KERN_CRIT "BUG at %s:%d assert(%s)\n",\
                                                                                                    __FILE__, __LINE__, #_con);\
                                                                BUG();\
                                                            }\
                                                          } while (0)
#else
#define     GFLIPIO_KDBG(fmt, args...)
#define     GFLIPIO_ASSERT(arg)
#endif
#define GFLIP_CHK_VERSION(u32InputVer,u32InputLength,u32LibVer,u32LibLength,u32CopiedLength)     do{  \
    if(u32InputVer < 1)  \
    {  \
        printk("[Err][%s][%d]: please check your u32Version, it should not set to 0!!\n",__func__,__LINE__);  \
        u32CopiedLength = 0;  \
    }  \
    else  \
    {  \
        if(u32InputLength > u32LibLength)  \
        {  \
            u32CopiedLength = u32LibLength;  \
        }  \
        else  \
        {  \
            u32CopiedLength = u32InputLength;  \
        }  \
    }  \
    if(u32InputVer != u32LibVer)  \
    {  \
        printk("[Warning][%s][%d]: Your ver = %tu length = %tu,GOP lib ver = %tu lengtg = %td\n",__func__,__LINE__,(ptrdiff_t)u32InputVer,(ptrdiff_t)u32InputLength,(ptrdiff_t)u32LibVer,(ptrdiff_t)u32LibLength);  \
    }  \
    }while(0);
//--------------------------------------------------------------------------------------------------
// IOCtrl functions declaration
//--------------------------------------------------------------------------------------------------
static int _MDrv_GFLIPIO_Open (struct inode *inode, struct file *filp);
static int _MDrv_GFLIPIO_Release(struct inode *inode, struct file *filp);
static long _MDrv_GFLIPIO_IOCtl(struct file *filp, unsigned int cmd, unsigned long arg);
#ifdef CONFIG_COMPAT
static long _Compat_MDrv_GFLIPIO_IOCtl(struct file *filp, unsigned int u32cmd, unsigned long u32arg);
#endif
MSYSTEM_STATIC int _MDrv_GFLIPIO_ModuleInit(void);
MSYSTEM_STATIC void _MDrv_GFLIPIO_ModuleExit(void);

//=============================================================================
// Local Variables: Device handler
//=============================================================================
typedef struct
{
    int s32Major;
    int s32Minor;
    int refCnt;
    int refIndex;
    struct cdev cdev;
    struct file_operations fops;
}GFLIP_DEV;

static GFLIP_DEV _devGFLIP =
{
    .s32Major = MDRV_MAJOR_GFLIP,
    .s32Minor = MDRV_MINOR_GFLIP,
    .refCnt = 0,
    .refIndex =0,
    .cdev =
    {
        .kobj = {.name= MDRV_NAME_GFLIP, },
        .owner = THIS_MODULE,
    },
    .fops =
    {
        .open =                     _MDrv_GFLIPIO_Open,
        .release =                  _MDrv_GFLIPIO_Release,
        .unlocked_ioctl =           _MDrv_GFLIPIO_IOCtl,
#ifdef CONFIG_COMPAT
    .compat_ioctl =                 _Compat_MDrv_GFLIPIO_IOCtl,
#endif

    }
};

struct
{
    struct file* filp;
    MS_U32    u32GOPIdx;
}_filpGopIdxGFLIP[MAX_FILE_HANDLE_SUPPRT];
MS_U16  g_u16GOPRefCnt[MAX_GOP_SUPPORT];
MS_BOOL bfilpGopIdx = FALSE;

#ifdef CONFIG_MSTAR_UDEV_NODE
static struct class *gflip_class;
#endif
//=============================================================================
// Local Function Prototypes
//=============================================================================
MS_U32 _MDrv_GFLIPIO_Init(MS_U32 u32GopIdx);
int _MDrv_GFLIPIO_IOC_Init(struct file *filp, unsigned long arg);
//static int _MDrv_GFLIPIO_IOC_DeInit(struct file *filp, unsigned long arg);
int _MDrv_GFLIPIO_IOC_SetFlipInfo(struct file *filp, unsigned long arg);
int _MDrv_GFLIPIO_IOC_TLBSetTLBInfo(struct file *filp, unsigned long arg);
int _MDrv_GFLIPIO_IOC_SetInputSigStatus(struct file *filp, unsigned long arg);
int _MDrv_GFLIPIO_IOC_SetPixelIDAddr(struct file *filp, unsigned long arg);
int _MDrv_GFLIPIO_IOC_SetGPIO3DPin(struct file *filp, unsigned long arg);
MS_U32 _MDrv_GFLIP_RegisterInterrupt(void);
MS_U32 _MDrv_GFLIP_DeRegisterInterrupt(void);
int _MDrv_GFLIPIO_IOC_Set3DFlipInfo(struct file *filp, unsigned long arg);
int _MDrv_GFLIPIO_IOC_SetTLB3DFlipInfo(struct file *filp, unsigned long arg);
int _MDrv_GFLIPIO_IOC_SetGwinInfo(struct file *filp, unsigned long arg);
int _MDrv_GFLIPIO_IOC_GetDWinIntInfo2(struct file *filp, unsigned long arg);
int _MDrv_GFLIPIO_IOC_ClearFlipQueue(struct file *filp, unsigned long arg);

EXPORT_SYMBOL(_MDrv_GFLIPIO_Init);

#if	( defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT))
int _MDrv_GFLIPIO_IOC_EnableVECapture(struct file *filp, unsigned long arg);
int _MDrv_GFLIPIO_IOC_GetVECaptureState(struct file *filp, unsigned long arg);
int _MDrv_GFLIPIO_EnableVECapture(MS_GFLIP_VECAPTURESTATE* pVECaptureState);
EXPORT_SYMBOL(_MDrv_GFLIPIO_EnableVECapture);
#endif

#ifdef	GFLIP_MULTI_FLIP
int _MDrv_GFLIPIO_IOC_SetMultiFlipInfo(struct file *filp, unsigned long arg);
int _MDrv_GFLIPIO_IOC_SetTLBMultiFlipInfo(struct file *filp, unsigned long arg);
#endif

MS_U32 _MDrv_GFLIPIO_SetIRQ(MS_BOOL bEnableIRQ);
EXPORT_SYMBOL(_MDrv_GFLIPIO_SetIRQ);

int _MDrv_GFLIP_SDR2HDRInfo(MS_SDR2HDR_INFO* pstSDR2HDRInfo);
EXPORT_SYMBOL(_MDrv_GFLIP_SDR2HDRInfo);
int _MDrv_GFLIP_HDRCentralInfo(ST_HDR_CENTRAL_CTRL_INFO* pstHDRCentralCtrl);
EXPORT_SYMBOL(_MDrv_GFLIP_HDRCentralInfo);

//=============================================================================
// Global Variables
//=============================================================================
MS_BOOL bStrEnableIRQFlag = TRUE;
MS_BOOL bGflipResumeDone = FALSE, bGflipIRQState = TRUE;
EN_GFLIP_STR_STATE enGflipSTRstate = E_GFLIP_RESUME_DONE_STATE;
EXPORT_SYMBOL(bStrEnableIRQFlag);
//-------------------------------------------------------------------------------------------------
// IOCtrl Driver functions
//-------------------------------------------------------------------------------------------------
MS_U32 _MDrv_GFLIPIO_Init(MS_U32 u32GopIdx)
{
    MS_U16 u16Idx, u16AllocIdx=MAX_FILE_HANDLE_SUPPRT;
    MS_U16 i = 0;

    //bfilpGopIdx Init
    if(bfilpGopIdx == FALSE)
    {
        for(i=0;i<MAX_FILE_HANDLE_SUPPRT;i++)
        {
            _filpGopIdxGFLIP[i].u32GOPIdx = GFLIP_GOP_IDX_INVALID;
        }
        bfilpGopIdx = TRUE;
    }

    if(MAX_GOP_SUPPORT<= u32GopIdx)
    {
        return -EFAULT;
    }

    if(_devGFLIP.refIndex!=0)
    {
        for(u16Idx=0; u16Idx<MAX_FILE_HANDLE_SUPPRT; u16Idx++)
        {
            if(_filpGopIdxGFLIP[u16Idx].u32GOPIdx==u32GopIdx)
            {//already init:
                return 0;
            }
        }
    }
    printk("PEIFEN   [%s][%d] u16Idx=%d,u16AllocIdx=%d ; u32GopIdx=%d \n",__FUNCTION__,__LINE__,_devGFLIP.refIndex, u16AllocIdx,u32GopIdx);
    _filpGopIdxGFLIP[_devGFLIP.refIndex].u32GOPIdx = u32GopIdx;
    //GFLIPIO_ASSERT(g_u16GOPRefCnt[u32GopIdx] >= 0);

    if(++g_u16GOPRefCnt[u32GopIdx] > 1)
    {
         return 0;
    }
    _devGFLIP.refIndex++;

    return MDrv_GFLIP_Init(u32GopIdx)? 0 : (-EFAULT);
}


int _MDrv_GFLIPIO_IOC_Init(struct file *filp, unsigned long arg)
{
    MS_U32 u32GopIdx;
    MS_U16 u16Idx, u16AllocIdx=MAX_FILE_HANDLE_SUPPRT;

    if(__get_user(u32GopIdx, (MS_U32 __user *)arg))
    {
        return -EFAULT;
    }

    if(MAX_GOP_SUPPORT<= u32GopIdx)
    {
        return -EFAULT;
    }

    for(u16Idx=0; u16Idx<MAX_FILE_HANDLE_SUPPRT; u16Idx++)
    {
        if(_filpGopIdxGFLIP[u16Idx].filp==filp && _filpGopIdxGFLIP[u16Idx].u32GOPIdx==u32GopIdx)
        {//already init:
            return 0;
        }

        if(u16AllocIdx==MAX_FILE_HANDLE_SUPPRT && NULL==_filpGopIdxGFLIP[u16Idx].filp)
        {
            u16AllocIdx= u16Idx;
        }
    }

    if(MAX_FILE_HANDLE_SUPPRT<=u16AllocIdx)
    {
        return -EFAULT;
    }

    _filpGopIdxGFLIP[u16AllocIdx].filp = filp;
    _filpGopIdxGFLIP[u16AllocIdx].u32GOPIdx = u32GopIdx;
    //GFLIPIO_ASSERT(g_u16GOPRefCnt[u32GopIdx] >= 0);

    if(++g_u16GOPRefCnt[u32GopIdx] > 1)
    {
         return 0;
    }

    return MDrv_GFLIP_Init(u32GopIdx) ? 0 : (-EFAULT);
}

/********************************************************
*** Do not export this IO DDI now ****************************
int _MDrv_GFLIPIO_IOC_DeInit(struct file *filp, unsigned long arg)
{
    MS_U32 u32GopIdx;

    if (__get_user(u32GopIdx, (MS_U32 __user *)arg))
    {
        return EFAULT;
    }

    return MDrv_GFLIP_DeInit(u32GopIdx);
}
*/

//Flip function for GOP 3D function
int _MDrv_GFLIPIO_IOC_Set3DFlipInfo(struct file *filp, unsigned long arg)
{
    MS_GFLIP_3DINFO st3DFlipInfo;

    if(copy_from_user(&st3DFlipInfo, (MS_GFLIP_3DINFO __user *)arg, sizeof(MS_GFLIP_3DINFO)))
    {
        return EFAULT;
    }

    if (_MDrv_GFLIP_SetFlipInfo(st3DFlipInfo.u32GopIdx, st3DFlipInfo.u32GwinIdx, st3DFlipInfo.u32MainAddr, st3DFlipInfo.u32SubAddr, st3DFlipInfo.u32TagId,&st3DFlipInfo.u32QEntry,&st3DFlipInfo.u32Result))
    {
        __put_user(st3DFlipInfo.u32QEntry, &(((MS_GFLIP_INFO __user *)arg)->u32QEntry));
        __put_user(st3DFlipInfo.u32Result, &(((MS_GFLIP_INFO __user *)arg)->u32Result));
	    return 0;
    }
    else
    {
        // The return value is not so appropriate.
        return EFAULT;
    }
}

int _MDrv_GFLIPIO_IOC_SetTLB3DFlipInfo(struct file *filp, unsigned long arg)
{
    MS_TLB_GFLIP_3DINFO stTLB3DFlipInfo;

    if(copy_from_user(&stTLB3DFlipInfo, (MS_TLB_GFLIP_3DINFO __user *)arg, sizeof(MS_TLB_GFLIP_3DINFO)))
    {
        return EFAULT;
    }

    if (MDrv_GFLIP_SetTLBFlipInfo(stTLB3DFlipInfo.u32GopIdx, stTLB3DFlipInfo.u32GwinIdx, stTLB3DFlipInfo.u32MainAddr, stTLB3DFlipInfo.u32SubAddr, \
                                stTLB3DFlipInfo.u32TagId,&stTLB3DFlipInfo.u32QEntry,&stTLB3DFlipInfo.u32Result, stTLB3DFlipInfo.bTLBEnable, stTLB3DFlipInfo.u32TLBAddr))
    {
        __put_user(stTLB3DFlipInfo.u32QEntry, &(((MS_TLB_GFLIP_3DINFO __user *)arg)->u32QEntry));
        __put_user(stTLB3DFlipInfo.u32Result, &(((MS_TLB_GFLIP_3DINFO __user *)arg)->u32Result));
	    return 0;
    }
    else
    {
        // The return value is not so appropriate.
        return EFAULT;
    }
}

#ifdef	GFLIP_MULTI_FLIP
int _MDrv_GFLIPIO_IOC_SetMultiFlipInfo(struct file *filp, unsigned long arg)
{
    MS_GFLIP_MULTIINFO stMultiFlipInfo;

    if(copy_from_user(&stMultiFlipInfo, (MS_GFLIP_MULTIINFO __user *)arg, sizeof(MS_GFLIP_MULTIINFO)))
    {
        return EFAULT;
    }

    if (_MDrv_GFLIP_SetMultiFlipInfo(&stMultiFlipInfo))
    {
        __put_user(stMultiFlipInfo.astGopInfo[0].u32QEntry, &(((MS_GFLIP_INFO __user *)arg)->u32QEntry));
        __put_user(stMultiFlipInfo.astGopInfo[0].u32Result, &(((MS_GFLIP_INFO __user *)arg)->u32Result));
	    return 0;
    }
    else
    {
        // The return value is not so appropriate.
        return EFAULT;
    }
}

int _MDrv_GFLIPIO_IOC_SetTLBMultiFlipInfo(struct file *filp, unsigned long arg)
{
    MS_TLB_GFLIP_MULTIINFO stTLBMultiFlipInfo;

    if(copy_from_user(&stTLBMultiFlipInfo, (MS_TLB_GFLIP_MULTIINFO __user *)arg, sizeof(MS_TLB_GFLIP_MULTIINFO)))
    {
        return EFAULT;
    }

    if (_MDrv_GFLIP_SetTLBMultiFlipInfo(&stTLBMultiFlipInfo))
    {
        __put_user(stTLBMultiFlipInfo.astTLBGopInfo[0].u32QEntry, &(((MS_TLB_GFLIP_3DINFO __user *)arg)->u32QEntry));
        __put_user(stTLBMultiFlipInfo.astTLBGopInfo[0].u32Result, &(((MS_TLB_GFLIP_3DINFO __user *)arg)->u32Result));
	    return 0;
    }
    else
    {
        // The return value is not so appropriate.
        return EFAULT;
    }
}

#endif
//Flip function for GOP normal 2D function
int _MDrv_GFLIPIO_IOC_SetFlipInfo(struct file *filp, unsigned long arg)
{
    MS_GFLIP_INFO stFlipInfo;

    if(copy_from_user(&stFlipInfo, (MS_GFLIP_INFO __user *)arg, sizeof(MS_GFLIP_INFO)))
    {
        return EFAULT;
    }

    if (_MDrv_GFLIP_SetFlipInfo(stFlipInfo.u32GopIdx, stFlipInfo.u32GwinIdx, stFlipInfo.u32Addr, 0, stFlipInfo.u32TagId,&stFlipInfo.u32QEntry,&stFlipInfo.u32Result))
    {
        __put_user(stFlipInfo.u32QEntry, &(((MS_GFLIP_INFO __user *)arg)->u32QEntry));
        __put_user(stFlipInfo.u32Result, &(((MS_GFLIP_INFO __user *)arg)->u32Result));
	    return 0;
    }
    else
    {
        // The return value is not so appropriate.
        return EFAULT;
    }
}

//Flip function for GOP normal 2D function
int _MDrv_GFLIPIO_IOC_TLBSetTLBInfo(struct file *filp, unsigned long arg)
{
    MS_TLB_GFLIP_INFO stTLBFlipInfo;

    if(copy_from_user(&stTLBFlipInfo, (MS_TLB_GFLIP_INFO __user *)arg, sizeof(MS_TLB_GFLIP_INFO)))
    {
        return EFAULT;
    }

    if(MDrv_GFLIP_SetTLBFlipInfo(stTLBFlipInfo.u32GopIdx, stTLBFlipInfo.u32GwinIdx, stTLBFlipInfo.u32Addr, 0, stTLBFlipInfo.u32TagId,\
                                 &stTLBFlipInfo.u32QEntry,&stTLBFlipInfo.u32Result,stTLBFlipInfo.bTLBEnable,stTLBFlipInfo.u32TLBAddr))
    {
        __put_user(stTLBFlipInfo.u32QEntry, &(((MS_TLB_GFLIP_INFO __user *)arg)->u32QEntry));
        __put_user(stTLBFlipInfo.u32Result, &(((MS_TLB_GFLIP_INFO __user *)arg)->u32Result));
	    return 0;
    }
    else
    {
        // The return value is not so appropriate.
        return EFAULT;
    }
}


int _MDrv_GFLIPIO_IOC_SetPixelIDAddr(struct file *filp, unsigned long arg)
{
    MS_GFLIP_INFO stFlipInfo;

    if(copy_from_user(&stFlipInfo, (MS_GFLIP_INFO __user *)arg, sizeof(MS_GFLIP_INFO)))
    {
        return EFAULT;
    }

    if (MDrv_GFLIP_SetPixelIDAddr(stFlipInfo.u32GopIdx, stFlipInfo.u32GwinIdx, stFlipInfo.u32Addr,stFlipInfo.u32TagId,&stFlipInfo.u32QEntry,&stFlipInfo.u32Result))
    {
        __put_user(stFlipInfo.u32Result, &(((MS_GFLIP_INFO __user *)arg)->u32Result));
	    return 0;
    }
    else
    {
        // The return value is not so appropriate.
        return EFAULT;
    }
}

int _MDrv_GFLIPIO_IOC_SetGPIO3DPin(struct file *filp, unsigned long arg)
{
    MS_GFLIP_INFO stFlipInfo;

    if(copy_from_user(&stFlipInfo, (MS_GFLIP_INFO __user *)arg, sizeof(MS_GFLIP_INFO)))
    {
        return EFAULT;
    }

    if (_MDrv_GFLIP_SetGPIO3DPin(stFlipInfo.u32Addr, &stFlipInfo.u32Result))
    {
        __put_user(stFlipInfo.u32Result, &(((MS_GFLIP_INFO __user *)arg)->u32Result));
	    return 0;
    }
    else
    {
        // The return value is not so appropriate.
        return EFAULT;
    }
}

#if	( defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT))
//Old method by GOP driver

int _MDrv_GFLIPIO_EnableVECapture(MS_GFLIP_VECAPTURESTATE* pVECaptureState)
{
    MS_GFLIP_VECAPTURESTATE stCurState;

    MDrv_GFLIP_GetVECapCurState(&stCurState.bEnable, &stCurState.u8FrameCount);
    pVECaptureState->u8FrameCount = stCurState.u8FrameCount; //return current FrameCount
    if(stCurState.bEnable != pVECaptureState->bEnable)
    {
        pVECaptureState->u8Result = TRUE;//State change, return TRUE
        MDrv_GFLIP_SetVECapCurState(&pVECaptureState->bEnable);//Change current state
    }
    else
    {
        pVECaptureState->u8Result = FALSE;//No State change mean duplicate set, return FALSE
    }

    return TRUE;

}

int _MDrv_GFLIPIO_IOC_EnableVECapture(struct file *filp, unsigned long arg)
{
    MS_GFLIP_VECAPTURESTATE stVECaptureState;

    if(copy_from_user(&stVECaptureState, (MS_GFLIP_VECAPTURESTATE __user *)arg, sizeof(MS_GFLIP_VECAPTURESTATE)))
    {
        return EFAULT;
    }
    _MDrv_GFLIPIO_EnableVECapture(&stVECaptureState);

    __put_user(stVECaptureState.u8Result, &(((MS_GFLIP_VECAPTURESTATE __user *)arg)->u8Result));
    __put_user(stVECaptureState.u8FrameCount, &(((MS_GFLIP_VECAPTURESTATE __user *)arg)->u8FrameCount));

    return 0;
}

int _MDrv_GFLIPIO_IOC_GetVECaptureState(struct file *filp, unsigned long arg)
{
    MS_GFLIP_VECAPTURESTATE stCurState;

    if(arg == NULL)
    {
        return EFAULT;
    }
    MDrv_GFLIP_GetVECapCurState(&stCurState.bEnable, &stCurState.u8FrameCount);
    __put_user(TRUE, &(((MS_GFLIP_VECAPTURESTATE __user *)arg)->u8Result));
    __put_user(stCurState.u8FrameCount, &(((MS_GFLIP_VECAPTURESTATE __user *)arg)->u8FrameCount));
    __put_user(stCurState.bEnable, &(((MS_GFLIP_VECAPTURESTATE __user *)arg)->bEnable));
    //printk("IO=%u\n", stCurState.u8FrameCount);

    return 0;
}

int _MDrv_GFLIPIO_IOC_VECaptureWaitOnFrame(struct file *filp, unsigned long arg)
{
    MS_GFLIP_VECAPTURESTATE stVECaptureState;

    if(copy_from_user(&stVECaptureState, (MS_GFLIP_VECAPTURESTATE __user *)arg, sizeof(MS_GFLIP_VECAPTURESTATE)))
    {
        return EFAULT;
    }
    stVECaptureState.u8Result = MDrv_GFLIP_VECapWaitOnFrame(&stVECaptureState.bEnable, &stVECaptureState.u8FrameCount);
    __put_user(stVECaptureState.u8Result, &(((MS_GFLIP_VECAPTURESTATE __user *)arg)->u8Result));
    __put_user(stVECaptureState.bEnable, &(((MS_GFLIP_VECAPTURESTATE __user *)arg)->bEnable));

    return 0;
}

//Below is new method by VE driver
MS_U32 _MDrv_GFLIP_VEC_RegisterInterrupt(MS_BOOL bEnable)
{
    if(bEnable)
    {
        if(0 != (request_irq(E_FIQ_VSYNC_VE4VBI, MDrv_GFLIP_VECINT_IntHandler, SA_INTERRUPT, "gop", NULL)))
        {
            GFLIPIO_KDBG("[GFLIP] Fail to request IRQ:%d\n", E_FIQ_VSYNC_VE4VBI);
            return EFAULT;
        }
        /*
        else
        {
            GFLIPIO_KDBG("[GFLIP] Success to request IRQ:%d\n", E_FIQ_VSYNC_VE4VBI);
        }*/
    }
    else
    {
        free_irq(E_FIQ_VSYNC_VE4VBI,NULL);
        GFLIPIO_KDBG("[GFLIP VEC]  dismiss IRQ:%d\n", E_FIQ_VSYNC_VE4VBI);
    }
    return 0;
}

int _MDrv_GFLIP_VECIO_IOC_EnableVECapture(struct file *filp, unsigned long arg)
{
    MS_GFLIP_VEC_STATE stVECState;
    MS_GFLIP_VEC_STATE stCurVECState;
    MS_GFLIP_VEC_CONFIG stGflipVECConfig;

    if(copy_from_user(&stVECState, (MS_GFLIP_VEC_STATE __user *)arg, sizeof(MS_GFLIP_VEC_STATE)))
    {
        return EFAULT;
    }

    MDrv_GFLIP_GetVECapCurState(&stCurVECState.bEnable, &stCurVECState.u8FrameCount);
    if(stCurVECState.bEnable != stVECState.bEnable)
    {
        stVECState.u8Result = TRUE;//State change, return TRUE
        MDrv_GFLIP_SetVECapCurState(&stVECState.bEnable);//Change current state
    }
    else
    {
        stVECState.u8Result = FALSE;//No State change mean duplicate set, return FALSE
    }
    stVECState.u8FrameCount = stCurVECState.u8FrameCount; //return current FrameCount

    //Register/De-register Interrupt
    MDrv_GFLIP_GetVECaptureConfig(&stGflipVECConfig);
    if(stVECState.u8Result && (MS_VEC_ISR_VE == stGflipVECConfig.eIsrType)) //For disable operation
    {
        if(EFAULT == _MDrv_GFLIP_VEC_RegisterInterrupt(stVECState.bEnable))
        {
            stVECState.u8Result = FALSE;
        }
    }
    __put_user(stVECState.u8Result, &(((MS_GFLIP_VEC_STATE __user *)arg)->u8Result));
    __put_user(stVECState.u8FrameCount, &(((MS_GFLIP_VEC_STATE __user *)arg)->u8FrameCount));
    __put_user(stVECState.bEnable, &(((MS_GFLIP_VEC_STATE __user *)arg)->bEnable));
    return 0;
}

int _MDrv_GFLIP_VECIO_IOC_GetVECaptureState(struct file *filp, unsigned long arg)
{
    MS_GFLIP_VEC_STATE stCurVECState;

    if(arg == NULL)
    {
        return EFAULT;
    }
    MDrv_GFLIP_GetVECapCurState(&stCurVECState.bEnable, &stCurVECState.u8FrameCount);
    __put_user(TRUE, &(((MS_GFLIP_VEC_STATE __user *)arg)->u8Result));
    __put_user(stCurVECState.u8FrameCount, &(((MS_GFLIP_VEC_STATE __user *)arg)->u8FrameCount));
    __put_user(stCurVECState.bEnable, &(((MS_GFLIP_VEC_STATE __user *)arg)->bEnable));
    //printk("IO=%u\n", stCurState.u8FrameCount);

    return 0;
}

int _MDrv_GFLIP_VECIO_IOC_VECaptureWaitOnFrame(struct file *filp, unsigned long arg)
{
    MS_GFLIP_VEC_STATE stVECState;

    if(copy_from_user(&stVECState, (MS_GFLIP_VEC_STATE __user *)arg, sizeof(MS_GFLIP_VEC_STATE)))
    {
        return EFAULT;
    }
    stVECState.u8Result = MDrv_GFLIP_VECapWaitOnFrame(&stVECState.bEnable, &stVECState.u8FrameCount);
    __put_user(stVECState.u8Result, &(((MS_GFLIP_VEC_STATE __user *)arg)->u8Result));
    __put_user(stVECState.bEnable, &(((MS_GFLIP_VEC_STATE __user *)arg)->bEnable));

    return 0;
}

int _MDrv_GFLIP_VECIO_IOC_VECaptureConfig(struct file *filp, unsigned long arg)
{
    MS_U8 u8Length;
    MS_GFLIP_VEC_CONFIG stGflipVECConfig;
    memset(&stGflipVECConfig, 0, sizeof(MS_GFLIP_VEC_CONFIG));
    if(copy_from_user(&stGflipVECConfig, (MS_GFLIP_VEC_CONFIG __user *)arg, VE_VEC_CONFIG_LENGTH_1STVERSION))
    {
        return EFAULT;
    }
    u8Length = (stGflipVECConfig.u16Length > sizeof(MS_GFLIP_VEC_CONFIG)) ? sizeof(MS_GFLIP_VEC_CONFIG) : stGflipVECConfig.u16Length;
    if(VE_VEC_CONFIG_LENGTH_1STVERSION < u8Length)
    {
        if(u8Length > sizeof(MS_GFLIP_VEC_CONFIG))
        {
            printk("[Warning][%s][%d]u8Length: %u is larger than sizeof(MS_GFLIP_VEC_CONFIG): %zu\n",__FUNCTION__,__LINE__,u8Length,sizeof(MS_GFLIP_VEC_CONFIG));
            return EFAULT;
        }

        if(copy_from_user(&stGflipVECConfig, (MS_GFLIP_VEC_CONFIG __user *)arg, u8Length))
        {
            return EFAULT;
        }
    }
    MDrv_GFLIP_SetVECaptureConfig(&stGflipVECConfig);
    __put_user(stGflipVECConfig.u8Result, &(((MS_GFLIP_VEC_CONFIG __user *)arg)->u8Result));
    return 0;
}

#endif //CONFIG_MSTAR_VE_CAPTURE_SUPPORT

int _MDrv_GFLIPIO_IOC_SetInputSigStatus(struct file *filp, unsigned long arg)
{
    MS_BOOL bHasSignal;

    if (__get_user(bHasSignal, (MS_BOOL __user *)arg))
    {
        return EFAULT;
    }

    if(bHasSignal)
    {
        _MDrv_GFLIP_RestoreFromVsyncLimitation();
    }

    return 0;
}

int _MDrv_GFLIPIO_IOC_GetDWinIntInfo(struct file *filp, unsigned long arg)
{
    MS_GFLIP_DWININT_INFO stDWinIntInfo;

    if(copy_from_user(&stDWinIntInfo, (MS_GFLIP_DWININT_INFO __user *)arg, sizeof(MS_GFLIP_DWININT_INFO)))
    {
        return EFAULT;
    }

    if (_MDrv_GFLIP_GetDWinIntInfo(&stDWinIntInfo.gflipDWinIntInfo, stDWinIntInfo.bResetDWinIntInfo, 0))
    {
        __put_user(stDWinIntInfo.gflipDWinIntInfo.u8DWinIntInfo, &(((MS_GFLIP_DWININT_INFO __user *)arg)->gflipDWinIntInfo.u8DWinIntInfo));
	    return 0;
    }
    else
    {
        // The return value is not so appropriate.
        return EFAULT;
    }
}

int _MDrv_GFLIPIO_IOC_GetDWinIntInfo2(struct file *filp, unsigned long arg)
{
    MS_GFLIP_DWININT_INFO2 stDWinIntInfo;

    if(copy_from_user(&stDWinIntInfo, (MS_GFLIP_DWININT_INFO2 __user *)arg, sizeof(MS_GFLIP_DWININT_INFO2)))
    {
        return EFAULT;
    }

    if (_MDrv_GFLIP_GetDWinIntInfo(&stDWinIntInfo.gflipDWinIntInfo, stDWinIntInfo.bResetDWinIntInfo,stDWinIntInfo.u32Timeout))
    {
        __put_user(stDWinIntInfo.gflipDWinIntInfo.u8DWinIntInfo, &(((MS_GFLIP_DWININT_INFO __user *)arg)->gflipDWinIntInfo.u8DWinIntInfo));
	    return 0;
    }
    else
    {
        // The return value is not so appropriate.
        return EFAULT;
    }
}

int _MDrv_GFLIPIO_IOC_ClearFlipQueue(struct file *filp, unsigned long arg)
{

    MS_GFLIP_GOPGWINIDX stGFlipQueueIdx;

    if(copy_from_user(&stGFlipQueueIdx, (MS_GFLIP_GOPGWINIDX __user *)arg, sizeof(MS_GFLIP_GOPGWINIDX)))
    {
        return EFAULT;
    }

    if(stGFlipQueueIdx.u32GwinIdx >= MAX_GOP_GWIN || stGFlipQueueIdx.u32GopIdx >= MAX_GOP_SUPPORT)
    {
        return EFAULT;
    }

    if (_MDrv_GFLIP_ClearFlipQueue(stGFlipQueueIdx.u32GopIdx,stGFlipQueueIdx.u32GwinIdx))
    {

	    return 0;
    }
    else
    {
        // The return value is not so appropriate.
        return EFAULT;
    }
}
int _MDrv_GFLIPIO_IOC_SetGwinInfo(struct file *filp, unsigned long arg)
{

    MS_GWIN_INFO stGwinInfo;

    memset(&stGwinInfo, 0, sizeof(stGwinInfo));
    if(copy_from_user(&stGwinInfo.u64Addr, ((MS_U8*)arg), sizeof(MS_PHY) ))
    {
        return EFAULT;
    }
    if(copy_from_user(&stGwinInfo.u16X, ((MS_U8*)arg)+sizeof(MS_PHY), 10UL ))
    {
        return EFAULT;
    }
    if(copy_from_user(&stGwinInfo.clrType, ((MS_U8*)arg)+sizeof(MS_GWIN_INFO) - 4UL, 4UL))
    {
        return EFAULT;
    }


    if(stGwinInfo.u8GwinIdx >= MAX_GOP_GWIN || stGwinInfo.u8GopIdx >= MAX_GOP_SUPPORT)
    {
        return EFAULT;
    }

    if (_MDrv_GFLIP_SetGwinInfo(stGwinInfo))
    {
        return 0;
    }
    else
    {
        // The return value is not so appropriate.
        return EFAULT;
    }

}

int _MDrv_GFLIPIO_IOC_DlcChangeCurveInfo(struct file *filp, unsigned long arg)
{
    printk("This ioctl is obsolete now\n");
    return -1;
}

int _MDrv_GFLIPIO_IOC_DlcInitInfo(struct file *filp, unsigned long arg)
{
    printk("This ioctl is obsolete now\n");
    return -1;
}

MS_U32 _MDrv_GFLIPIO_SetIRQ(MS_BOOL bEnableIRQ)
{
    if((bEnableIRQ== TRUE) && (bGflipResumeDone == TRUE) && (bGflipIRQState == FALSE))
    {
        enable_irq(E_IRQ_GOP);
        bGflipIRQState = TRUE;
    }

    bStrEnableIRQFlag = TRUE;//set to default state

    return TRUE;
}

int _MDrv_GFLIPIO_IOC_StrIRQFlag(struct file *filp, unsigned long arg)
{
    MS_BOOL bStrIRQFlag = 0;

    if (__get_user(bStrIRQFlag, (MS_U32 __user *)arg))
    {
        return EFAULT;
    }

    bStrEnableIRQFlag = bStrIRQFlag;

    return 0;
}

int _MDrv_GFLIPIO_IOC_SetIRQ(struct file *filp, unsigned long arg)
{
    MS_BOOL bEnableIRQ = 0;

    if (__get_user(bEnableIRQ, (MS_U32 __user *)arg))
    {
        return EFAULT;
    }

    _MDrv_GFLIPIO_SetIRQ(bEnableIRQ);

    return 0;
}

int _MDrv_GFLIP_HDRCentralInfo(ST_HDR_CENTRAL_CTRL_INFO* pstHDRCentralCtrl)
{
    if(MHal_GFLIP_SetHDRCentralInfo(pstHDRCentralCtrl) == TRUE)
    {
        return 0;
    }
    else
    {
        return EFAULT;
    }
}

int _MDrv_GFLIPIO_IOC_HDRCentralInfo(struct file *filp, unsigned long arg)
{
    ST_HDR_CENTRAL_CTRL_INFO stHDRCentralCtrl;

    memset(&stHDRCentralCtrl, 0, sizeof(ST_HDR_CENTRAL_CTRL_INFO));
    if(copy_from_user(&stHDRCentralCtrl, (ST_HDR_CENTRAL_CTRL_INFO __user *)arg, sizeof(ST_HDR_CENTRAL_CTRL_INFO)))
    {
        return EFAULT;
    }

    if(MHal_GFLIP_SetHDRCentralInfo(&stHDRCentralCtrl) == TRUE)
    {
        return 0;
    }
    else
    {
        return EFAULT;
    }

    return 0;
}

int _MDrv_GFLIPIO_IOC_HDRInitInfo(struct file *filp, unsigned long arg)
{
    printk("This ioctl is obsolete now\n");
    return -1;
}

int _MDrv_GFLIPIO_IOC_DlcOnOffInfo(struct file *filp, unsigned long arg)
{
    printk("This ioctl is obsolete now\n");
    return -1;
}

int _MDrv_GFLIPIO_IOC_BleChangeSlopPointInfo(struct file *filp, unsigned long arg)
{
    printk("This ioctl is obsolete now\n");
    return -1;
}

int _MDrv_GFLIPIO_IOC_SendDlcHistogram32Info(struct file *filp, unsigned long arg)
{
    printk("This ioctl is obsolete now\n");
    return -1;
}

int _MDrv_GFLIP_SDR2HDRInfo(MS_SDR2HDR_INFO* pstSDR2HDRInfo)
{
    if(MHAL_GFLIP_SetOpmuxInfo(pstSDR2HDRInfo) == TRUE)
    {
        return 0;
    }
    else
    {
        return EFAULT;
    }
}

int _MDrv_GFLIPIO_IOC_SDR2HDRInfo(struct file *filp, unsigned long arg)
{
    MS_SDR2HDR_INFO stSDR2HDRInfo;

    memset(&stSDR2HDRInfo, 0, sizeof(MS_SDR2HDR_INFO));
    if(copy_from_user(&stSDR2HDRInfo, (MS_SDR2HDR_INFO __user *)arg, sizeof(MS_SDR2HDR_INFO)))
    {
        return EFAULT;
    }

    if(MHAL_GFLIP_SetOpmuxInfo(&stSDR2HDRInfo) == TRUE)
    {
        return 0;
    }
    else
    {
        return EFAULT;
    }
}

int _MDrv_GFLIPIO_IOC_GetVsync(struct file *filp, unsigned long arg)
{
    MS_U32 u32GopIdx = 0;

    if (__get_user(u32GopIdx, (MS_U32 __user *)arg))
    {
        return EFAULT;
    }
    if(u32GopIdx >= MAX_GOP_SUPPORT)
    {
		printk("GFLIPIO_IOC_GetVsync GOP = %td is out of range ",(ptrdiff_t)u32GopIdx);
        return EFAULT;
    }
    if(MDrv_GFLIP_WaitForVsync(u32GopIdx) == TRUE)
    {
        return 0;
    }
    else
    {
        return EFAULT;
    }
}

int _MDrv_GFLIPIO_IOC_GetVsync_EX(struct file *filp, unsigned long arg)
{
    MS_U32 u32GopIdx = 0;
    MS_S64 u64VsyncTs;
    GFLIP_GETVSYNC_DATA data;

    if (__get_user(data.vsync_gop, &(((GFLIP_GETVSYNC_DATA __user *)arg)->vsync_gop)))
    {
        return EFAULT;
    }
    u32GopIdx = data.vsync_gop;
    if(u32GopIdx >= MAX_GOP_SUPPORT)
    {
        printk("GFLIPIO_IOC_GetVsync GOP = %td is out of range ",(ptrdiff_t)u32GopIdx);
        return EFAULT;
    }
    u64VsyncTs = MDrv_GFLIP_WaitForVsync_EX(u32GopIdx);
    if (__put_user(u64VsyncTs, &(((GFLIP_GETVSYNC_DATA __user *)arg)->vsync_ts))) {
         return EFAULT;
    }
    if (u64VsyncTs != -1)
    {
        return 0;
    }
    else
    {
        return EFAULT;
    }
}

int _MDrv_GFLIPIO_IOC_BleOnOffInfo(struct file *filp, unsigned long arg)
{
    printk("This ioctl is obsolete now\n");
    return -1;
}

MS_U32 _MDrv_GFLIP_RegisterInterrupt(void)
{
    if(0 != (request_irq(E_IRQ_GOP, MDrv_GFLIPINT_IntHandler, IRQF_SHARED, "gop", &_devGFLIP)))
    {
        GFLIPIO_KDBG("[GFLIP] Fail to request IRQ:%d\n", E_IRQ_GOP);
        return EFAULT;
    }

    return 0;
}

MS_U32 _MDrv_GFLIP_DeRegisterInterrupt(void)
{
    GFLIPIO_KDBG("[GFLIP]  dismiss IRQ:%d\n", E_IRQ_GOP);
    free_irq(E_IRQ_GOP,&_devGFLIP);

    return 0;
}

int _MDrv_GFLIPIO_IOC_CSC_Calc(struct file *filp, unsigned long arg)
{
    ST_GFLIP_GOP_CSC_PARAM stGflipCSCParam;
    MS_U32 u32CopiedLength = 0;

    memset(&stGflipCSCParam, 0, sizeof(ST_GFLIP_GOP_CSC_PARAM));

    ///check version
    if(copy_from_user(&stGflipCSCParam, (ST_GFLIP_GOP_CSC_PARAM __user *)arg, sizeof(MS_U32)*2))
    {
        return EFAULT;
    }
    GFLIP_CHK_VERSION(stGflipCSCParam.u32Version,stGflipCSCParam.u32Length,ST_GFLIP_GOP_CSC_PARAM_VERSION,sizeof(ST_GFLIP_GOP_CSC_PARAM),u32CopiedLength);

    if(u32CopiedLength == 0)
    {
        return EFAULT;
    }

    if(copy_from_user(&stGflipCSCParam, (ST_GFLIP_GOP_CSC_PARAM __user *)arg, u32CopiedLength))
    {
        return EFAULT;
    }

    if(_MDrv_GFLIP_CSC_Calc(&stGflipCSCParam) == FALSE)
    {
        return EFAULT;
    }

    if(copy_to_user((ST_GFLIP_GOP_CSC_PARAM __user *)arg, &stGflipCSCParam, u32CopiedLength))
    {
        return EFAULT;
    }

    return 0;
}

int _MDrv_GFLIPIO_IOC_SetBWPEnFlag(struct file *filp, unsigned long arg)
{
    ST_GFLIP_GOP_BWP_INFO stGflipBWPInfo;

    memset(&stGflipBWPInfo, 0, sizeof(ST_GFLIP_GOP_BWP_INFO));

    if(copy_from_user(&stGflipBWPInfo, (ST_GFLIP_GOP_BWP_INFO __user *)arg, sizeof(ST_GFLIP_GOP_BWP_INFO)))
    {
        return EFAULT;
    }

    if(_MDrv_GFLIP_SetBWPEnFlag(stGflipBWPInfo.u32GOPNum, stGflipBWPInfo.bEnable) == FALSE)
    {
        return EFAULT;
    }

    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------
// IOCtrl Driver interface functions
//-------------------------------------------------------------------------------------------------
int _MDrv_GFLIPIO_Open(struct inode *inode, struct file *filp)
{
    GFLIPIO_KDBG("[GFLIP] GFLIP DRIVER OPEN\n");

    GFLIPIO_ASSERT(_devGFLIP.refCnt>=0);
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
    GFLIPIO_KDBG("[GFLIP] New flip function enable\n");
    printk(KERN_ALERT"[%s,%d][pid:%d][name:%s]_devGFLIP.refCnt=%d\n",__func__,__LINE__,current->pid,current->comm,_devGFLIP.refCnt);
    if(_devGFLIP.refCnt == 0)//Init timier when first open gflip
    {
        MDrv_GFLIP_InitTimer();
    }
#endif
    _devGFLIP.refCnt++;
    return 0;
}

int _MDrv_GFLIPIO_Release(struct inode *inode, struct file *filp)
{
    MS_U32 u32Idx;

    GFLIPIO_KDBG("[GFLIP] GFLIP DRIVER CLOSE\n");

    GFLIPIO_ASSERT(_devGFLIP.refCnt>0);

    for(u32Idx=0; u32Idx<MAX_FILE_HANDLE_SUPPRT; u32Idx++)
    {
        if(_filpGopIdxGFLIP[u32Idx].filp != filp)
        {
            continue;
        }

        GFLIPIO_ASSERT(g_u16GOPRefCnt[_filpGopIdxGFLIP[u32Idx].u32GOPIdx] > 0);

        if(0 == --g_u16GOPRefCnt[_filpGopIdxGFLIP[u32Idx].u32GOPIdx])
        {
                MDrv_GFLIP_DeInit(_filpGopIdxGFLIP[u32Idx].u32GOPIdx);
        }

        _filpGopIdxGFLIP[u32Idx].filp = NULL;
     }

    _devGFLIP.refCnt--;

    return 0;
}

#ifdef CONFIG_COMPAT
long _Compat_MDrv_GFLIPIO_IOCtl(struct file *filp, U32 u32Cmd, unsigned long u32Arg)
{
    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch(u32Cmd)
	{
	    case MDRV_GFLIP_IOC_INIT:
        case MDRV_GFLIP_IOC_SETFLIPINFO:
        case MDRV_GFLIP_IOC_TLBSETFLIPINFO:
        case MDRV_GFLIP_IOC_SET3DFLIPINFO:
        case MDRV_GFLIP_IOC_TLBSET3DFLIPINFO:
#ifdef	GFLIP_MULTI_FLIP
        case MDRV_GFLIP_IOC_SETMULTIFLIPINFO:
        case MDRV_GFLIP_IOC_SETTLBMULTIFLIPINFO:
#endif
        case MDRV_GFLIP_IOC_SETINPUTSIGSTATUS:
        case MDRV_GFLIP_IOC_GETDWININTINFO:
        case MDRV_GFLIP_IOC_GETDWININTINFO2:
        case MDRV_GFLIP_IOC_SETPIXELIDADDR:
        case MDRV_GFLIP_IOC_SETGPIO3DPIN:
#if	( defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT))
        case MDRV_GFLIP_IOC_GETVECAPTURESTATE:
        case MDRV_GFLIP_IOC_VECAPTUREWAITONFRAME:
        case MDRV_GFLIP_IOC_ENABLEVECAPTURE:
        case MDRV_GFLIP_VEC_IOC_CONFIG:
        case MDRV_GFLIP_VEC_IOC_ENABLEVECAPTURE:
        case MDRV_GFLIP_VEC_IOC_GETVECAPTURESTATE:
        case MDRV_GFLIP_VEC_IOC_VECAPTUREWAITONFRAME:
#endif
        case MDRV_GFLIP_IOC_CLEARFLIPQUEUE:
        case MDRV_GFLIP_IOC_SETGWININFO:
        case MDRV_GFLIP_IOC_DLCCHANGECURVE:
        case MDRV_GFLIP_IOC_DLCONOFFINFO:
        case MDRV_GFLIP_IOC_BLECHANGECURVE:
        case MDRV_GFLIP_IOC_BLEONOFFINFO:
        case MDRV_GFLIP_IOC_DLCGETHISTOGRAMINFO:
        case MDRV_GFLIP_IOC_GETVSYNC:
        case MDRV_GFLIP_IOC_SETDLCINITINFO:
        case MDRV_GFLIP_IOC_STRIRQFLAG:
        case MDRV_GFLIP_IOC_SETIRQENABLE:
        case MDRV_GFLIP_IOC_SETHDRFLAG:
        case MDRV_GFLIP_IOC_SETHDRCENTRAL:
        case MDRV_GFLIP_IOC_DLCSETHDRINFO:
        case MDRV_GFLIP_IOC_CSCTUNING:
        case MDRV_GFLIP_IOC_GETVSYNCEX:
        case MDRV_GFLIP_IOC_SETBWPENFLAG:
    	    return filp->f_op->unlocked_ioctl(filp, u32Cmd,
    						(unsigned long)compat_ptr(u32Arg));
            break;
	    default:
            return -ENOTTY;
            break;

    }
}
#endif

long _MDrv_GFLIPIO_IOCtl(struct file *filp, U32 u32Cmd, unsigned long u32Arg)
{
    int err = 0;
    int retval = 0;
    if(_devGFLIP.refCnt <= 0)
    {
        return -EFAULT;
    }
    /* check u32Cmd valid */
    if(MDRV_GFLIP_IOC_MAGIC == _IOC_TYPE(u32Cmd))
    {
        if(_IOC_NR(u32Cmd) >= MDRV_GFLIP_IOC_MAX_NR)
        {
            GFLIPIO_KDBG("[GFLIP] IOCtl NR Error!!! (Cmd=%x)\n",u32Cmd);
            return -ENOTTY;
        }
    }
    else if(MDRV_GFLIP_VEC_IOC_MAGIC == _IOC_TYPE(u32Cmd))
    {
        if(_IOC_NR(u32Cmd) >= MDRV_GFLIP_VEC_IOC_MAX_NR)
        {
            GFLIPIO_KDBG("[GFLIP VEC] IOCtl NR Error!!! (Cmd=%x)\n",u32Cmd);
            return -ENOTTY;
        }
    }
    else
    {
        GFLIPIO_KDBG("[GFLIP] IOCtl MAGIC Error!!! (Cmd=%x)\n",u32Cmd);
        return -ENOTTY;
    }

    /* verify Access */
    if (_IOC_DIR(u32Cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)u32Arg, _IOC_SIZE(u32Cmd));
    }
    else if (_IOC_DIR(u32Cmd) & _IOC_WRITE)
    {
        err =  !access_ok(VERIFY_READ, (void __user *)u32Arg, _IOC_SIZE(u32Cmd));
    }
    if (err)
    {
        return -EFAULT;
    }

    /* handle u32Cmd */
    switch(u32Cmd)
    {
        case MDRV_GFLIP_IOC_INIT:
            retval = _MDrv_GFLIPIO_IOC_Init(filp, u32Arg);
            break;
        //case MDRV_GFLIP_IOC_DEINIT:
            //retval = _MDrv_GFLIPIO_IOC_DeInit(filp, u32Arg);
            //break;
        case MDRV_GFLIP_IOC_SETFLIPINFO:
            retval = _MDrv_GFLIPIO_IOC_SetFlipInfo(filp, u32Arg);
            break;
        case MDRV_GFLIP_IOC_TLBSETFLIPINFO:
            retval = _MDrv_GFLIPIO_IOC_TLBSetTLBInfo(filp, u32Arg);
            break;
        case MDRV_GFLIP_IOC_SET3DFLIPINFO:
            retval = _MDrv_GFLIPIO_IOC_Set3DFlipInfo(filp, u32Arg);
            break;
        case MDRV_GFLIP_IOC_TLBSET3DFLIPINFO:
            retval = _MDrv_GFLIPIO_IOC_SetTLB3DFlipInfo(filp, u32Arg);
            break;
#ifdef	GFLIP_MULTI_FLIP
        case MDRV_GFLIP_IOC_SETMULTIFLIPINFO:
            retval = _MDrv_GFLIPIO_IOC_SetMultiFlipInfo(filp, u32Arg);
            break;
        case MDRV_GFLIP_IOC_SETTLBMULTIFLIPINFO:
            retval = _MDrv_GFLIPIO_IOC_SetTLBMultiFlipInfo(filp, u32Arg);
            break;
#endif
        case MDRV_GFLIP_IOC_SETINPUTSIGSTATUS:
            retval = _MDrv_GFLIPIO_IOC_SetInputSigStatus(filp, u32Arg);
            break;
        case MDRV_GFLIP_IOC_GETDWININTINFO:
            retval = _MDrv_GFLIPIO_IOC_GetDWinIntInfo(filp, u32Arg);
            break;
        case MDRV_GFLIP_IOC_GETDWININTINFO2:
            retval = _MDrv_GFLIPIO_IOC_GetDWinIntInfo2(filp, u32Arg);
            break;
        case MDRV_GFLIP_IOC_SETPIXELIDADDR:
            retval = _MDrv_GFLIPIO_IOC_SetPixelIDAddr(filp, u32Arg);
            break;
        case MDRV_GFLIP_IOC_SETGPIO3DPIN:
            retval = _MDrv_GFLIPIO_IOC_SetGPIO3DPin(filp, u32Arg);
            break;
#if	( defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT))
        //Old method by GOP driver
        case MDRV_GFLIP_IOC_GETVECAPTURESTATE:
            retval = _MDrv_GFLIPIO_IOC_GetVECaptureState(filp, u32Arg);
            break;
        case MDRV_GFLIP_IOC_VECAPTUREWAITONFRAME:
            retval = _MDrv_GFLIPIO_IOC_VECaptureWaitOnFrame(filp, u32Arg);
            break;
        case MDRV_GFLIP_IOC_ENABLEVECAPTURE:
            retval = _MDrv_GFLIPIO_IOC_EnableVECapture(filp, u32Arg);
            break;

        //New method by VE driver
        case MDRV_GFLIP_VEC_IOC_CONFIG:
            retval = _MDrv_GFLIP_VECIO_IOC_VECaptureConfig(filp, u32Arg);
            break;
        case MDRV_GFLIP_VEC_IOC_ENABLEVECAPTURE:
            retval = _MDrv_GFLIP_VECIO_IOC_EnableVECapture(filp, u32Arg);
            break;
        case MDRV_GFLIP_VEC_IOC_GETVECAPTURESTATE:
            retval = _MDrv_GFLIP_VECIO_IOC_GetVECaptureState(filp, u32Arg);
            break;
        case MDRV_GFLIP_VEC_IOC_VECAPTUREWAITONFRAME:
            retval = _MDrv_GFLIP_VECIO_IOC_VECaptureWaitOnFrame(filp, u32Arg);
            break;
#endif //CONFIG_MSTAR_VE_CAPTURE_SUPPORT
        case MDRV_GFLIP_IOC_CLEARFLIPQUEUE:
            retval = _MDrv_GFLIPIO_IOC_ClearFlipQueue(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_SETGWININFO:
            retval = _MDrv_GFLIPIO_IOC_SetGwinInfo(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_DLCCHANGECURVE:
            retval = _MDrv_GFLIPIO_IOC_DlcChangeCurveInfo(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_DLCONOFFINFO:
            retval = _MDrv_GFLIPIO_IOC_DlcOnOffInfo(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_BLECHANGECURVE:
            retval = _MDrv_GFLIPIO_IOC_BleChangeSlopPointInfo(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_BLEONOFFINFO:
            retval = _MDrv_GFLIPIO_IOC_BleOnOffInfo(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_DLCGETHISTOGRAMINFO:
            retval = _MDrv_GFLIPIO_IOC_SendDlcHistogram32Info(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_GETVSYNC:
            retval = _MDrv_GFLIPIO_IOC_GetVsync(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_SETDLCINITINFO:
            retval = _MDrv_GFLIPIO_IOC_DlcInitInfo(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_STRIRQFLAG:
            retval = _MDrv_GFLIPIO_IOC_StrIRQFlag(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_SETIRQENABLE:
            retval = _MDrv_GFLIPIO_IOC_SetIRQ(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_SETHDRFLAG:
            retval = _MDrv_GFLIPIO_IOC_SDR2HDRInfo(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_SETHDRCENTRAL:
            retval = _MDrv_GFLIPIO_IOC_HDRCentralInfo(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_DLCSETHDRINFO:
            retval = _MDrv_GFLIPIO_IOC_HDRInitInfo(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_CSCTUNING:
            retval = _MDrv_GFLIPIO_IOC_CSC_Calc(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_GETVSYNCEX:
            retval = _MDrv_GFLIPIO_IOC_GetVsync_EX(filp,u32Arg);
            break;
        case MDRV_GFLIP_IOC_SETBWPENFLAG:
            retval = _MDrv_GFLIPIO_IOC_SetBWPEnFlag(filp,u32Arg);
            break;
        default:  /* redundant, as cmd was checked against MAXNR */
            GFLIPIO_KDBG("[GFLIP] ERROR IOCtl number %x\n ",u32Cmd);
            return -ENOTTY;
    }

    return (long)retval;
}

static int mstar_gflip_drv_suspend(struct platform_device *dev, pm_message_t state)
{
    if(enGflipSTRstate == E_GFLIP_RESUME_DONE_STATE)
    {
        disable_irq(E_IRQ_GOP);
        bGflipIRQState = FALSE;

        MDrv_GFLIP_Suspend();
        enGflipSTRstate = E_GFLIP_SUSPEND_DONE_STATE;
        bGflipResumeDone = FALSE;
    }

    return 0;
}
static int mstar_gflip_drv_resume(struct platform_device *dev)
{
    if(enGflipSTRstate == E_GFLIP_SUSPEND_DONE_STATE)
    {
        MDrv_GFLIP_Resume();
        bGflipResumeDone = TRUE;

        if((bStrEnableIRQFlag == TRUE) && (bGflipIRQState == FALSE))
        {
            enable_irq(E_IRQ_GOP);
            bGflipIRQState = TRUE;
        }

        enGflipSTRstate = E_GFLIP_RESUME_DONE_STATE;
    }

    return 0;
}

static int mstar_gflip_drv_probe(struct platform_device *pdev)
{
    int retval=0;
    pdev->dev.platform_data=NULL;
	return retval;
}

static int mstar_gflip_drv_remove(struct platform_device *pdev)
{
    pdev->dev.platform_data=NULL;
    return 0;
}
#if defined (CONFIG_OF)
static struct of_device_id mstargflip_of_device_ids[] = {
         {.compatible = "mstar-gflip"},
         {},
};
#endif
static struct platform_driver Mstar_gflip_driver = {
	.probe 		= mstar_gflip_drv_probe,
	.remove 	= mstar_gflip_drv_remove,
    .suspend    = mstar_gflip_drv_suspend,
    .resume     = mstar_gflip_drv_resume,

	.driver = {
#if defined(CONFIG_OF)
	    .of_match_table = mstargflip_of_device_ids,
#endif
		.name	= "Mstar-gflip",
        .owner  = THIS_MODULE,
	}
};
//-------------------------------------------------------------------------------------------------
// Module functions
//-------------------------------------------------------------------------------------------------
int _MDrv_GFLIPIO_ModuleInit(void)
{
    int s32Ret;
    dev_t  dev;

#ifdef CONFIG_MSTAR_UDEV_NODE
    gflip_class = class_create(THIS_MODULE, MDRV_NAME_GFLIP);
    if (IS_ERR(gflip_class))
    {
        return PTR_ERR(gflip_class);
    }
#endif

    memset(_filpGopIdxGFLIP, 0, sizeof(_filpGopIdxGFLIP));
    memset(g_u16GOPRefCnt, 0, sizeof(g_u16GOPRefCnt));
    spin_lock_init(&spinlock_gflip);
    if(_devGFLIP.s32Major)
    {
        dev = MKDEV(_devGFLIP.s32Major, _devGFLIP.s32Minor);
        s32Ret = register_chrdev_region(dev, MDRV_GFLIP_DEVICE_COUNT, MDRV_GFLIP_NAME);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, _devGFLIP.s32Minor, MDRV_GFLIP_DEVICE_COUNT, MDRV_GFLIP_NAME);
        _devGFLIP.s32Major = MAJOR(dev);
    }

    if (0 > s32Ret)
    {
        GFLIPIO_KDBG("[GFLIP] Unable to get major %d\n", _devGFLIP.s32Major);
        return s32Ret;
    }

    cdev_init(&_devGFLIP.cdev, &_devGFLIP.fops);
    if (0 != (s32Ret= cdev_add(&_devGFLIP.cdev, dev, MDRV_GFLIP_DEVICE_COUNT)))
    {
        GFLIPIO_KDBG("[GFLIP] Unable add a character device\n");
        unregister_chrdev_region(dev, MDRV_GFLIP_DEVICE_COUNT);
        return s32Ret;
    }

    /* initial the whole GFLIP Driver */

    if(EFAULT == _MDrv_GFLIP_RegisterInterrupt())
    {
        GFLIPIO_KDBG("[GFLIP] Startup GFLIP Driver Failed! %d\n", _devGFLIP.s32Major);
        cdev_del(&_devGFLIP.cdev);
        unregister_chrdev_region(dev, MDRV_GFLIP_DEVICE_COUNT);
	 return -ENOMEM;
    }

#ifdef CONFIG_MSTAR_UDEV_NODE
    device_create(gflip_class, NULL, dev, NULL, MDRV_NAME_GFLIP);
#endif

    platform_driver_register(&Mstar_gflip_driver);

#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
    printk(KERN_ALERT"[%s,%d][pid:%d][name:%s]_devGFLIP.refCnt=%d\n",__func__,__LINE__,current->pid,current->comm,_devGFLIP.refCnt);
    MDrv_GFLIP_InitTimer();
#endif

    return 0;
}


void _MDrv_GFLIPIO_ModuleExit(void)
{
    /*de-initial the who GFLIPDriver */
    _MDrv_GFLIP_DeRegisterInterrupt();

    cdev_del(&_devGFLIP.cdev);
    unregister_chrdev_region(MKDEV(_devGFLIP.s32Major, _devGFLIP.s32Minor), MDRV_GFLIP_DEVICE_COUNT);
    platform_driver_unregister(&Mstar_gflip_driver);
}



#if defined(CONFIG_MSTAR_MSYSTEM) || defined(CONFIG_MSTAR_MSYSTEM_MODULE)
#else//#if defined(CONFIG_MSTAR_MSYSTEM) || defined(CONFIG_MSTAR_MSYSTEM_MODULE)
module_init(_MDrv_GFLIPIO_ModuleInit);
module_exit(_MDrv_GFLIPIO_ModuleExit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("GFLIP ioctrl driver");
MODULE_LICENSE("GPL");
#endif//#if defined(CONFIG_MSTAR_MSYSTEM) || defined(CONFIG_MSTAR_MSYSTEM_MODULE)
