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
/// @file   mdrv_gflip.c
/// @brief  MediaTek gflip Interface
/// @author MediaTek Inc.
/// @attention
/// <b><em></em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////

#define _MDRV_GFLIP_C

//=============================================================================
// Include Files
//=============================================================================
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#if defined(CONFIG_MIPS)
#elif defined(CONFIG_ARM)
#include <asm/io.h>
#endif

#include "mdrv_mstypes.h"
#include "mdrv_gflip.h"
#include "mhal_gflip.h"
#include "mhal_gflip_reg.h"


//=============================================================================
// Compile options
//=============================================================================


//=============================================================================
// Local Defines
//=============================================================================
#define MAX_FLIP_ADDR_FIFO 	(0x10)
#define FLIP_INTERRUPT_TIMEOUT   (100)  //In MS
#define FLIP_WAITQ_ONESHOT_TIMEOUT   (10)  //In MS

#if	( defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT))
#define VE_CAPTURE_FRAME_INVALID_NUM 0
#endif

#define MAIN_WINDOW         0
#define SUB_WINDOW          1

#define GFLIP_HZ_TIMER 0
#define GFLIP_HR_TIMER 1
#define GFLIP_TIMER_TYPE  GFLIP_HR_TIMER

#if ( !defined (CONFIG_HIGH_RES_TIMERS) && (GFLIP_TIMER_TYPE==GFLIP_HR_TIMER))
#error "can't using high resolution timer when kernel config not support CONFIG_HIGH_RES_TIMERS"
#endif

#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
#define GFLIP_TIMER_CHECK_TIME          	2 //Check every 2ms
#else
#define GFLIP_TIMER_CHECK_TIME          	1 //Check every jiffies
#endif

#define CFD_CTRL_POINT_NUM (2)
#define CFD_DEBUG_LOG (FALSE)
#define CFD_TEST_MODE (FALSE)
//=============================================================================
// Debug Macros
//=============================================================================
#define GFLIP_DEBUG
#ifdef GFLIP_DEBUG
    #define GFLIP_PRINT(fmt, args...)      printk("[GFlip (Driver)][%05d] " fmt, __LINE__, ## args)
    #define GFLIP_ASSERT(_cnd, _fmt, _args...)                   \
                                    if (!(_cnd)) {              \
                                        GFLIP_PRINT(_fmt, ##_args);  \
                                    }
#else
    #define GFLIP_PRINT(_fmt, _args...)
    #define GFLIP_ASSERT(_cnd, _fmt, _args...)
#endif
#define MS_TO_NS(x) (x * 1E6L)

//=============================================================================
// Local Function Prototypes
//=============================================================================
MS_GWIN_INFO MDrv_GFLIP_GetGwinInfo(MS_U8 u8GopIdx,MS_U8 u8GwinIdx);
MS_BOOL _MDrv_GFLIP_SetFlipInfo(MS_U32 u32GopIdx, MS_U32 u32GwinIdx, MS_PHY64 u32MainAddr, MS_PHY64 u32SubAddr, MS_U32 u32TagId, MS_U32 * u32QEntry, MS_U32 *u32Result);

static FP_CFD_GOP_PreProcess _fpGflip_CB_CFD_GOP_PreProcess = NULL;
static FP_CFD_GOP_PreSDR _fpGflip_CB_CFD_GOP_PreSDR = NULL;
static FP_CFD_GOP_TestMode _fpGflip_CB_CFD_GOP_TestMode = NULL;
static MS_U64 g_timer_callback_times = 0x0;
static MS_U64 g_Multi_timer_callback_times = 0x0;

EXPORT_SYMBOL(_MDrv_GFLIP_SetFlipInfo);
#ifdef	GFLIP_MULTI_FLIP
EXPORT_SYMBOL(_MDrv_GFLIP_SetMultiFlipInfo);
EXPORT_SYMBOL(_MDrv_GFLIP_SetTLBMultiFlipInfo);
#endif
EXPORT_SYMBOL(_MDrv_GFLIP_SetGwinInfo);
EXPORT_SYMBOL(_MDrv_GFLIP_RestoreFromVsyncLimitation);
EXPORT_SYMBOL(_MDrv_GFLIP_GetDWinIntInfo);
#if	( defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT))
EXPORT_SYMBOL(MDrv_GFLIP_GetVECapCurState);
EXPORT_SYMBOL(MDrv_GFLIP_VECapWaitOnFrame);
#endif
EXPORT_SYMBOL(_MDrv_GFLIP_ClearFlipQueue);
EXPORT_SYMBOL(_MDrv_GFLIP_SetGPIO3DPin);
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
EXPORT_SYMBOL(MDrv_GFLIP_InitTimer);
EXPORT_SYMBOL(MDrv_GFLIP_Del_Timer);
#endif
EXPORT_SYMBOL(MDrv_GFLIP_Register_CB);
EXPORT_SYMBOL(_MDrv_GFLIP_CSC_Calc);
EXPORT_SYMBOL(_MDrv_GFLIP_SetBWPEnFlag);
//=============================================================================
// Macros
//=============================================================================
//=============================================================================
// Local Variables
//=============================================================================
static volatile GFLIP_INFO _GFlipInfo[MAX_GOP_SUPPORT][MAX_GOP_GWIN][MAX_FLIP_ADDR_FIFO];
static volatile MS_U32 _u32GFlipInfoReadPtr[MAX_GOP_SUPPORT][MAX_GOP_GWIN];
static volatile MS_U32 _u32GFlipInfoWritePtr[MAX_GOP_SUPPORT][MAX_GOP_GWIN];
static volatile MS_BOOL _bGFlipInVsyncLimitation[MAX_GOP_SUPPORT] = { FALSE };
static volatile MS_GWIN_INFO _u32GwinInfo[MAX_GOP_SUPPORT][MAX_GOP_GWIN];
static MS_BOOL bGOPVsyncStatus[6] = { FALSE };
static MS_BOOL bgflipVsyncStatus = FALSE;
static MS_BOOL bgcaptureVsyncStatus = FALSE;

static volatile MS_BOOL _bEnableGOPTLB[MAX_GOP_SUPPORT] = { FALSE };
static volatile MS_U32 _u32GOPTLBaddress[MAX_GOP_SUPPORT];

static GFLIP_DWININT_INFO _GFlipDWinIntInfo = { 0x0 };
static MS_U32 _u32GFlipLatestIntTicket; //The jiffies when Interrupt happen
static DECLARE_WAIT_QUEUE_HEAD(_gflip_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(_gcapture_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(_gvsync_gop0_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(_gvsync_gop1_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(_gvsync_gop2_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(_gvsync_gop3_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(_gvsync_gop4_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(_gvsync_gop5_waitqueue);

#if	( defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT))
static MS_BOOL s_bEnable = FALSE;
static MS_U8   s_u8FrameCount = VE_CAPTURE_FRAME_INVALID_NUM;
static MS_U8   s_u8FrameNumUpdated = VE_CAPTURE_FRAME_INVALID_NUM;
static MS_GFLIP_VEC_CONFIG s_stGflipVECConfig = {0, sizeof(MS_GFLIP_VEC_CONFIG), 0, 0, 3, 4, 0, MS_VEC_CONFIG_INIT}; //the VEC config setting
#endif

#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
static MS_BOOL _bTimerInited = FALSE;
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
static struct hrtimer _stGflip_hrtimer;
#else
static struct timer_list _stGflip_timer;
#endif

#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
static enum hrtimer_restart MDrv_GFLIP_Timer_Callback( struct hrtimer *timer );
#else
static void MDrv_GFLIP_Timer_Callback(unsigned long value);
#endif

#ifdef	GFLIP_MULTI_FLIP
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
enum hrtimer_restart MDrv_GFLIP_MultiGOP_Timer_Callback( struct hrtimer *timer );
#else
static void MDrv_GFLIP_MultiGOP_Timer_Callback(unsigned long value);
#endif
#endif
#endif

#if (MAX_GOP_SUPPORT < 5)
#define GFLIP_REG_BANKS (MAX_GOP_SUPPORT * GFLIP_GOP_BANKOFFSET)
#else
#define GFLIP_REG_BANKS (MAX_GOP_SUPPORT * GFLIP_GOP_BANKOFFSET + 2)
#endif
#define GFLIP_REG16_NUM_PER_BANK 128

typedef struct
{
    unsigned short BankReg[GFLIP_REG_BANKS][GFLIP_REG16_NUM_PER_BANK];
    //unsigned short GWinEnable[MAX_GOP_SUPPORT];
    unsigned short CKG_GopReg[10];
    unsigned short GS_GopReg;
#ifdef GOP_SC_GOPBLENDING_EX
    unsigned short SC_OPBlending[6];
#else
    unsigned short SC_OPBlending[5];
#endif
    unsigned short SC_save[3][128];
#ifdef GOP_MIU_REG
    unsigned short MIU_GopReg[3];
#endif
#ifdef GOP_MIU_IN_SC
    unsigned short MIU_SC_GopReg[3];
#endif

#ifdef SUPPORT_GOP_DUALRATE
    unsigned short SC_GopBlending[3];
    unsigned short SC_GopHsyncShift;
#endif

#ifdef SUPPORT_GOP_AFBC
    unsigned short u16AfbcRegClk;

    unsigned short u16AfbcRegEn[GOP_AFBC_CORE_NUMBER];
    unsigned short u16AfbcRegAddrL[GOP_AFBC_CORE_NUMBER];
    unsigned short u16AfbcRegAddrH[GOP_AFBC_CORE_NUMBER];
    unsigned short u16AfbcRegAddrMSB[GOP_AFBC_CORE_NUMBER];
    unsigned short u16AfbcRegFmt[GOP_AFBC_CORE_NUMBER];
    unsigned short u16AfbcRegWidth[GOP_AFBC_CORE_NUMBER];
    unsigned short u16AfbcRegHeight[GOP_AFBC_CORE_NUMBER];
    unsigned short u16AfbcRegCropX[GOP_AFBC_CORE_NUMBER];
    unsigned short u16AfbcRegCropY[GOP_AFBC_CORE_NUMBER];
    unsigned short u16AfbcRegCropHSize[GOP_AFBC_CORE_NUMBER];
    unsigned short u16AfbcRegCropVSize[GOP_AFBC_CORE_NUMBER];
    unsigned short u16AfbcRegMiu;
    unsigned short u16AfbcTrigger;
#endif

#ifdef NEED_MANUAL_CROP
    unsigned short u16GopManualCrop_PreCalDone[MANUAL_CROP_SETS];
    unsigned short u16GopManualCrop_Hstart[MANUAL_CROP_SETS];
    unsigned short u16GopManualCrop_HEnd[MANUAL_CROP_SETS];
#endif

#ifdef SUPPORT_GOP_MIXER
    unsigned short u16HDRBypass;
    unsigned short u16MixerSWRst[GOP_MIXER_NUMBER];
    unsigned short u16MixerValid[GOP_MIXER_NUMBER];
    unsigned short u16MixerSrcSel[GOP_MIXER_NUMBER];
#endif

#ifdef GEN_GOP_XCIP_TIMING
    unsigned short u16GenGOPXCIPTiming;
#endif

#ifdef DRAM_RBLK_MSB_MOVE_TO_SC
    unsigned short u16GopDramRblkMsb[MAX_GOP_SUPPORT];
#endif

#ifdef SC_BWP_SETTING
    unsigned short u16GopBwpTrgSlow[MAX_GOP_SUPPORT];
    unsigned short u16GopBwpRlsSlow[MAX_GOP_SUPPORT];
    unsigned short u16GopBwpTrgStop[MAX_GOP_SUPPORT];
    unsigned short u16GopBwpRlsStop[MAX_GOP_SUPPORT];
#endif
}GFLIP_REGS_SAVE_AREA;

static GFLIP_REGS_SAVE_AREA _gflip_regs_save={{{0},{0}}};
static MS_U32 _gu32PaletteSave[MAX_GOP_SUPPORT][PALETTE_SIZE];
static MS_S64 s_s64VsyncTs[6] = {0, 0, 0, 0, 0, 0};
static MS_BOOL _gbBWPEnFlag[MAX_GOP_SUPPORT] = {FALSE};
//=============================================================================
// Global Variables
//=============================================================================
DEFINE_MUTEX(Semutex_Gflip_gop0);
DEFINE_MUTEX(Semutex_Gflip_gop1);
DEFINE_MUTEX(Semutex_Gflip_gop2);
DEFINE_MUTEX(Semutex_Gflip_gop3);
DEFINE_MUTEX(Semutex_Gflip_gop4);
DEFINE_MUTEX(Semutex_Gflip_gop5);

//=============================================================================
// Local Function Prototypes
//=============================================================================
static void _MDrv_GFLIP_ResetFlipInfoPtr(MS_U32 u32GopIdx);
//=============================================================================
// Local Function
//=============================================================================
//-------------------------------------------------------------------------------------------------
/// Reset Flip Info Reader/Writer ptr.
/// @param  u32GopIdx    \b IN: the idx of ptr's owner(gop)
/// @return void
/// @attention
/// <b>[OBAMA] <em></em></b>
//-------------------------------------------------------------------------------------------------
void _MDrv_GFLIP_ResetFlipInfoPtr(MS_U32 u32GopIdx)
{
    MS_U16 u16Idx;

    for(u16Idx=0; u16Idx<MAX_GOP_GWIN; u16Idx++)
    {
        _u32GFlipInfoReadPtr[u32GopIdx][u16Idx] = 0;
        _u32GFlipInfoWritePtr[u32GopIdx][u16Idx] = 0;
    }
}

MS_BOOL _MDrv_GFLIP_Mutex_Lock(MS_U32 u32GopIdx, MS_BOOL bLock)
{
    MS_BOOL bRet = TRUE;

    if(bLock == TRUE)
    {
        switch(u32GopIdx)
        {
            case 0:
                mutex_lock(&Semutex_Gflip_gop0);
                break;
            case 1:
                mutex_lock(&Semutex_Gflip_gop1);
                break;
            case 2:
                mutex_lock(&Semutex_Gflip_gop2);
                break;
            case 3:
                mutex_lock(&Semutex_Gflip_gop3);
                break;
            case 4:
                mutex_lock(&Semutex_Gflip_gop4);
                break;
            case 5:
                mutex_lock(&Semutex_Gflip_gop5);
                break;
            default :
                printk("[%s][%d]u32GopIdx =%td is out of case",__FUNCTION__,__LINE__, (ptrdiff_t)u32GopIdx);
                bRet = FALSE;
                break;
        }
    }
    else
    {
        switch(u32GopIdx)
        {
            case 0:
                mutex_unlock(&Semutex_Gflip_gop0);
                break;
            case 1:
                mutex_unlock(&Semutex_Gflip_gop1);
                break;
            case 2:
                mutex_unlock(&Semutex_Gflip_gop2);
                break;
            case 3:
                mutex_unlock(&Semutex_Gflip_gop3);
                break;
            case 4:
                mutex_unlock(&Semutex_Gflip_gop4);
                break;
            case 5:
                mutex_unlock(&Semutex_Gflip_gop5);
                break;
            default :
                printk("[%s][%d]u32GopIdx =%td is out of case",__FUNCTION__,__LINE__, (ptrdiff_t)u32GopIdx);
                bRet = FALSE;
                break;
        }
    }

    return bRet;
}

// for identify GOP reg format
static MS_U32 _MDrv_GFLIP_GetRegForm(MS_U8 u8GOP)
{
    MS_U32 u32Ret = 0;

    if(u8GOP != INVAILD_GOP_NUM)
    {
        switch(u8GOP)
        {
            case E_GOP0:
                u32Ret = GOP0_REG_FORM;
                break;
            case E_GOP1:
                u32Ret = GOP1_REG_FORM;
                break;
            case E_GOP2:
                u32Ret = GOP2_REG_FORM;
                break;
            case E_GOP3:
                u32Ret = GOP3_REG_FORM;
                break;
            case E_GOP4:
                u32Ret = GOP4_REG_FORM;
                break;
            case E_GOP5:
                u32Ret = GOP5_REG_FORM;
                break;
            default:
                u32Ret = 0xffff;
        }
    }
    return u32Ret ;
}

unsigned short SC_R2BYTE(unsigned short addr, unsigned short bk)
{
#ifdef SC_DIR_BASE
    return SC_DIR_REG(addr, bk);
#else
    SC_REG(GOP_SC_BANKSEL) = bk;
    return SC_REG(addr);
#endif
}

void SC_W2BYTE(unsigned short addr, unsigned short bk, unsigned short value)
{
#ifdef SC_DIR_BASE
    SC_DIR_REG(addr, bk) = value;
#else
    SC_REG(GOP_SC_BANKSEL) = bk;
    SC_REG(addr) = value;
#endif
}

//=============================================================================
// GFLIP Driver Function
//=============================================================================
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
MS_U32 MDrv_GFLIP_InitTimer(void)
{
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
    //printk(KERN_ALERT"[%s,%d][pid:%d][name:%s]_bTimerInited=%d\n",__func__,__LINE__,current->pid,current->comm,_bTimerInited);
    if(_bTimerInited == FALSE)
    {
        hrtimer_init( &_stGflip_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
        _bTimerInited = TRUE;
    }
#else
    init_timer( &_stGflip_timer);
#endif
    return 0;
}

MS_U32 MDrv_GFLIP_Del_Timer(void)
{
    MS_U32 u32Idx;

    for(u32Idx=0; u32Idx<MAX_GOP_GWIN; u32Idx++)
    {
        _MDrv_GFLIP_Mutex_Lock(u32Idx,TRUE);
    }
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
    //no delete timer
#else
    del_timer_sync( &_stGflip_timer);
#endif
    for(u32Idx=0; u32Idx<MAX_GOP_GWIN; u32Idx++)
    {
        _MDrv_GFLIP_Mutex_Lock(u32Idx,FALSE);
    }

    return 0;
}
#endif

//-------------------------------------------------------------------------------------------------
/// GFlip Init(reset flip info ptr, interrupt enable, etc..)
/// @param  u32GopIdx                  \b IN:the idx of gop which need be initialized.
/// @return TRUE: success
/// @attention
/// <b>[OBAMA] <em></em></b>
//-------------------------------------------------------------------------------------------------
MS_BOOL MDrv_GFLIP_Init(MS_U32 u32GopIdx)
{
    //reset flip Info:
    _MDrv_GFLIP_ResetFlipInfoPtr(u32GopIdx);
    //enable interrupt:
    MHal_GFLIP_IntEnable(u32GopIdx,TRUE);

    return TRUE;
}

//-------------------------------------------------------------------------------------------------
/// GFlip DeInit(interrupt disable)
/// @param  u32GopIdx                  \b IN:the idx of gop which need be de-initialized.
/// @return TRUE: success
/// @attention
/// <b>[OBAMA] <em></em></b>
//-------------------------------------------------------------------------------------------------
MS_BOOL MDrv_GFLIP_DeInit(MS_U32 u32GopIdx)
{
    //disable interrupt:
    _MDrv_GFLIP_Mutex_Lock(u32GopIdx,TRUE);
    MHal_GFLIP_IntEnable(u32GopIdx,FALSE);
    _MDrv_GFLIP_Mutex_Lock(u32GopIdx,FALSE);

    return TRUE;
}

//-------------------------------------------------------------------------------------------------
/// Clear Irq
/// @param  void
/// @return TRUE: success
/// @attention
/// <b>[OBAMA] <em></em></b>
//-------------------------------------------------------------------------------------------------
MS_BOOL MDrv_GFLIP_ClearIRQ(void)
{
    MS_U32 u32GopIdx;

    MHal_GFLIP_ClearDWINIRQ(&_GFlipDWinIntInfo);

    if ( waitqueue_active(&_gcapture_waitqueue))
    {
        bgcaptureVsyncStatus=TRUE;
        wake_up_interruptible(&_gcapture_waitqueue);
    }

    for(u32GopIdx=0; u32GopIdx<MAX_GOP_SUPPORT; u32GopIdx++)
    {
        if(MHal_GFLIP_IsVSyncInt(u32GopIdx) == FALSE)
        {
            continue;
        }

        MHal_GFLIP_IntEnable(u32GopIdx, FALSE);

        if(MHal_GFLIP_IsVSyncInt(u32GopIdx) == TRUE)
        {   //Wow...The Vsync Issue Happened...
            MHal_GFLIP_HandleVsyncLimitation(u32GopIdx); //different chip maybe has different handle.
            _bGFlipInVsyncLimitation[u32GopIdx] = TRUE;
        }

        MHal_GFLIP_IntEnable(u32GopIdx, TRUE);
    }

    //Record current Time Ticket:
    _u32GFlipLatestIntTicket = jiffies;
    return TRUE;
}

//-------------------------------------------------------------------------------------------------
/// Process Irq
/// @param  void
/// @return TRUE: success kickoff flip to gop
/// @return FALSE: no flip kickoff to gop
/// @attention
/// <b>[OBAMA] <em>
///         step1: check src of vsync. interrupt
///         step2: check if any flip request in queue with this Gop
///         step3: check if TAG back
///         step4: set flip to GOP
///         step5: set ReadPtr to next, this entry consumed!
/// </em></b>
//-------------------------------------------------------------------------------------------------
static MS_U32 g_remaplastGwinAddr = 0;
static MS_U32 g_GPIO3DPin = 0;

extern void Chip_Flush_Memory(void);
extern void MDrv_GPIO_Set_High(U8 u8IndexGPIO);
extern void MDrv_GPIO_Set_Low(U8 u8IndexGPIO);

MS_BOOL MDrv_GFLIP_ProcessIRQ(void)
{
    MS_U32 u32GopIdx = 0;
#if ( !defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
    MS_U8 u8WakeUp = FALSE;
#endif
#if ( defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT))
    static MS_BOOL bVECaptureEna = FALSE, bVEReady = FALSE;
#endif
#if ( defined (CONFIG_MSTAR_3D_LR_REPORT))
    static MS_U32 lasttime=0, thistime = 0;
    static MS_U32 uCount = 0;
#endif

#if (!(defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE)))
    MS_U32 u32GwinIdx=0;
#if (GFLIP_AUTO_DETECT_BUFFER_EN == 1)
    MS_BOOL bNeedToFlipCnt[MAX_GOP_SUPPORT] = {FALSE};
    MS_U16 u16RegAutoDetect = 0, u16RegGwinEn = 0;
#endif
#endif

#if ((defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT)) || (defined (CONFIG_MSTAR_3D_LR_REPORT)))
    MS_BOOL bOPVsyncInterrupt = FALSE;
#endif

#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))

#else
#ifdef GFLIP_GOP_TLB
    MS_PHY64 u32TLBMainAddr=0;
    MS_PHY64 u32TLBSubAddr=0;
    MS_PHY64 u32MainAddress=0;
    MS_PHY64 u32SubAddress=0;
#endif
#endif

    for(u32GopIdx=0; u32GopIdx<MAX_GOP_SUPPORT; u32GopIdx++)
    {
        //step 1: check src of vsync. interrupt
        if(MHal_GFLIP_IsVSyncInt(u32GopIdx) == FALSE)
        {
            continue;
        }
        MDrv_GFLIP_GetVsync(u32GopIdx);
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
        {
#if ((defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT)) || (defined (CONFIG_MSTAR_3D_LR_REPORT)))
            if((bOPVsyncInterrupt == FALSE) && (GFLIP_GOP_DST_OP0 == MHal_GFLIP_GetGopDst(u32GopIdx)))
            {
                bOPVsyncInterrupt = TRUE;
            }
#endif
        }
#else //Old flip logic
#if ((defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT)) || (defined (CONFIG_MSTAR_3D_LR_REPORT)))
        if((bOPVsyncInterrupt == FALSE) && (GFLIP_GOP_DST_OP0 == MHal_GFLIP_GetGopDst(u32GopIdx)))
        {
            bOPVsyncInterrupt = TRUE;
        }
#endif

        for(u32GwinIdx=0; u32GwinIdx<MAX_GOP_GWIN; u32GwinIdx++)
        {
            //step 2:check if any flip request in queue with this Gop:
            if(_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx] == _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]) //no any flip request in queue
            {
                continue;
            }

            //step 3: if get queue, check if TAG back.
            if(MHal_GFLIP_IsTagIDBack(_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32TagId) == FALSE)
            {
                continue;
            }

            //Has Flip Request, and Tag Returned! we need programming flip address:
            //step 4: if Tag Back: set flip to GOP.
#ifdef GFLIP_GOP_TLB
            if(_bEnableGOPTLB[u32GopIdx] == TRUE)
            {
                u32TLBMainAddr=_u32GOPTLBaddress[u32GopIdx]+(_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32MainAddr/GOP_TLB_PAGE_SIZE)*TLB_PER_ENTRY_SIZE;
                u32TLBSubAddr=_u32GOPTLBaddress[u32GopIdx]+(_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32SubAddr/GOP_TLB_PAGE_SIZE)*TLB_PER_ENTRY_SIZE;
                u32TLBMainAddr /= ADDRESSING_8BYTE_UNIT;
                u32TLBSubAddr /= ADDRESSING_8BYTE_UNIT;
                u32MainAddress=_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32MainAddr % (GOP_TLB_PAGE_SIZE*PER_MIU_TLB_ENTRY_COUNT);
                u32SubAddress=_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32SubAddr % (GOP_TLB_PAGE_SIZE*PER_MIU_TLB_ENTRY_COUNT);
                MHal_GFLIP_SetTLBFlipToGop(u32GopIdx,u32GwinIdx,u32TLBMainAddr,u32TLBSubAddr,TRUE);
                MHal_GFLIP_SetFlipToGop(u32GopIdx,u32GwinIdx,u32MainAddress,u32SubAddress,TRUE);
            }
            else
#endif
            {
                MHal_GFLIP_SetFlipToGop(
                    u32GopIdx,
                    u32GwinIdx,
                    _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32MainAddr,
                    _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32SubAddr,
                    TRUE);
            }
            //Step 5: set ReadPtr to next, this entry consumed!
            _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx] = (_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]+1)%MAX_FLIP_ADDR_FIFO;

            u8WakeUp = TRUE;

        }
#if (GFLIP_AUTO_DETECT_BUFFER_EN == 1)
        MHal_GFLIP_ReadGopReg(u32GopIdx, GFLIP_GOP_BANK_IDX_0, REG_GOP_AUTO_DETECT_BUF, &u16RegAutoDetect);
        if(u16RegAutoDetect & GOP_AUTO_DETECT_BUF_EN)
        {
            for(u32GwinIdx=0; u32GwinIdx < MAX_GOP_GWIN; u32GwinIdx++)
            {
                if(_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx] != _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]) //no any flip request in queue
                {
                    bNeedToFlipCnt[u32GopIdx] = TRUE;
                    break;
                }
            }
            if(bNeedToFlipCnt[u32GopIdx] != TRUE)
            {
                 MHal_GFLIP_IntEnable(u32GopIdx, FALSE); //disable ISR of u32GopIdx
                 MHal_GFLIP_ReadGopReg(u32GopIdx, GFLIP_GOP_BANK_IDX_1, REG_GOP4G_GWIN_CTRL0(u32GwinIdx), &u16RegGwinEn);
                 if((u16RegGwinEn & GOP_GWIN_ENABLE_MASK) != 0)
                 {
                     MHal_GFLIP_Fire(1 << u32GopIdx);
                 }
            }
        }
#endif
#endif
#ifdef SC_BWP_SETTING
        if(_gbBWPEnFlag[u32GopIdx] && MHal_GFLIP_CheckBWPEnCondition(u32GopIdx))
        {
            MHal_GFLIP_SetBWPEnable(u32GopIdx, TRUE);
        }
        else
        {
            MHal_GFLIP_SetBWPEnable(u32GopIdx, FALSE);
        }
#endif
    }

#if ( defined (CONFIG_MSTAR_3D_LR_REPORT))
    //[3D TV]when AP call the pixelIDAddr IOCTL, below code will draw two pixel on the screen to tell ursa
    //current LR flag of scaler
    if(bOPVsyncInterrupt)
    {
        if(g_remaplastGwinAddr || g_GPIO3DPin)
        {
            MS_U16 result = (MHal_XC_GetCurrentReadBank() & 0x01);
            if(result == 0)
            {
                if(g_remaplastGwinAddr)
                {
                   (*((volatile MS_U8*)(g_remaplastGwinAddr)))     =  0xFF;
                   (*((volatile MS_U8*)(g_remaplastGwinAddr) + 1)) =  0xFF;
                   (*((volatile MS_U8*)(g_remaplastGwinAddr) + 2)) =  0xFF;
                   (*((volatile MS_U8*)(g_remaplastGwinAddr) + 3)) =  0xFF;

                   (*((volatile MS_U8*)(g_remaplastGwinAddr) + 4)) =  0x01;
                   (*((volatile MS_U8*)(g_remaplastGwinAddr) + 5)) =  0x00;
                   (*((volatile MS_U8*)(g_remaplastGwinAddr) + 6)) =  0x00;
                   (*((volatile MS_U8*)(g_remaplastGwinAddr) + 7)) =  0xFF;

                    Chip_Flush_Memory();
                }

                if(g_GPIO3DPin)
                {
                    MDrv_GPIO_Set_High(g_GPIO3DPin);
                }

            }
            else
            {
                if(g_remaplastGwinAddr)
                {
                    (*((volatile MS_U8*)(g_remaplastGwinAddr)))     =  0x01;
                    (*((volatile MS_U8*)(g_remaplastGwinAddr) + 1)) =  0x00;
                    (*((volatile MS_U8*)(g_remaplastGwinAddr) + 2)) =  0x00;
                    (*((volatile MS_U8*)(g_remaplastGwinAddr) + 3)) =  0xFF;

                    (*((volatile MS_U8*)(g_remaplastGwinAddr) + 4)) =  0xFF;
                    (*((volatile MS_U8*)(g_remaplastGwinAddr) + 5)) =  0xFF;
                    (*((volatile MS_U8*)(g_remaplastGwinAddr) + 6)) =  0xFF;
                    (*((volatile MS_U8*)(g_remaplastGwinAddr) + 7)) =  0xFF;
                    Chip_Flush_Memory();
                }

                if(g_GPIO3DPin)
                {
                    MDrv_GPIO_Set_Low(g_GPIO3DPin);
                }
            }
            #if 1
            thistime = jiffies;
            if(thistime - lasttime >22)
            {
               printk("\n-[%s][%d]---------timeout:----jiffies=%lu, ucount=%lu\n",__FUNCTION__,__LINE__, thistime - lasttime, uCount++);
            }

            lasttime = thistime;
            #endif
        }
    }
#endif
#if ( defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT))
    if((s_stGflipVECConfig.eIsrType == MS_VEC_ISR_GOP_OP) && bOPVsyncInterrupt) //Only GOP OP isr enter here now
    {
        if(s_bEnable != bVECaptureEna)
        {
            //Enable or disable VE capture
            MHal_GFLIP_VECaptureEnable(s_bEnable); //For old version VEC, enable ve in ISR

            bVECaptureEna = s_bEnable;
            bVEReady = FALSE; //Reset HW state
            if(bVECaptureEna)
            {
                s_u8FrameCount = VE_CAPTURE_FRAME_INVALID_NUM; //Valid Frame is 1~3
                s_u8FrameNumUpdated = VE_CAPTURE_FRAME_INVALID_NUM; //Valid Frame is 1~3
            }
        }
        if(bVECaptureEna)
        {
            //Valid Frame is 1~3
            if(!bVEReady)
            {
                bVEReady = MHal_GFLIP_CheckVEReady();
            }
            else
            {
                //printk("%u\n", s_u8FrameCount);
                if(((s_stGflipVECConfig.bInterlace == FALSE) && (s_u8FrameCount >= s_stGflipVECConfig.u8MaxFrameNumber_P)) ||
                   ((s_stGflipVECConfig.bInterlace == TRUE)  && (s_u8FrameCount >= s_stGflipVECConfig.u8MaxFrameNumber_I))
                  )
                {
                    s_u8FrameCount = VE_CAPTURE_FRAME_INVALID_NUM+1;
                }
                else
                {
                    s_u8FrameCount++;
                }
                s_u8FrameNumUpdated = s_u8FrameCount; //Valid Frame is 1~3
            }
        }
    }
#endif

#if ( !defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
    if (u8WakeUp && waitqueue_active(&_gflip_waitqueue))
    {
        bgflipVsyncStatus=TRUE;
        wake_up_interruptible(&_gflip_waitqueue);
    }
#endif

    return TRUE;
}

//-------------------------------------------------------------------------------------------------
/// Restore HW Limitation -- Vsync Limitation.
/// @return TRUE: DDI call success
/// @attention
/// <b>[OBAMA] <em>The Signal of GOP Dst from IP/VE restored and we re-enable GFlip. </em></b>
//-------------------------------------------------------------------------------------------------
MS_BOOL _MDrv_GFLIP_RestoreFromVsyncLimitation(void)
{
    MS_U32 u32GopIdx;

    for(u32GopIdx=0; u32GopIdx<MAX_GOP_SUPPORT; u32GopIdx++)
    {
        if(TRUE == _bGFlipInVsyncLimitation[u32GopIdx])
        {
            MHal_GFLIP_RestoreFromVsyncLimitation(u32GopIdx);
            _bGFlipInVsyncLimitation[u32GopIdx] = FALSE;
        }
    }

    return TRUE;
}

//-------------------------------------------------------------------------------------------------
/// Get DWin Interrupt Info
/// @param  pGFlipDWinIntInfo       \b OUT: the dwin interrupt info
/// @param  bResetDWinIntInfo       \b IN: reset dwin interrupt infor or not
/// @return TRUE: DDI call success
/// @attention
/// <b>[OBAMA] <em>Get DWin Interrupt Info and reset it if requested. </em></b>
//-------------------------------------------------------------------------------------------------
MS_BOOL _MDrv_GFLIP_GetDWinIntInfo(GFLIP_DWININT_INFO *pGFlipDWinIntInfo, MS_BOOL bResetDWinIntInfo, MS_U32 u32Timeout)
{
    pGFlipDWinIntInfo->u8DWinIntInfo = _GFlipDWinIntInfo.u8DWinIntInfo;
    if(bResetDWinIntInfo)
    {
        _GFlipDWinIntInfo.u8DWinIntInfo = 0x0;
    }

    if(!pGFlipDWinIntInfo->sDwinIntInfo.bDWinIntPROG && u32Timeout > 0)
    {
        bgcaptureVsyncStatus=FALSE;
        wait_event_interruptible_timeout(_gcapture_waitqueue, bgcaptureVsyncStatus,msecs_to_jiffies(u32Timeout));
    }


    return TRUE;
}

#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
MS_BOOL MDrv_GFLIP_IsGOPACK(MS_U32 u32GopIdx)
{
    return MHal_GFLIP_IsGOPACK(u32GopIdx);
}

//-------------------------------------------------------------------------------------------------
/// Kick off Flip to Gop HW.
/// @param  u32GopIdx                  \b IN: The gop idx who is the flip owner
/// @param  u32GwinIdx                 \b IN: The gwin idx who is the flip owner
/// @param  u32MainAddr                \b IN: The flip address for normal 2D display
/// @param  u32SubAddr                 \b IN: The 2nd fb flip address for GOP 3D display
/// @param  bForceWriteIn              \b IN: if use force write to update register, TRUE=force write
/// @return TRUE: success
/// @return FALSE: fail
/// @attention
/// <b>[GFLIP] <em> </em></b>
//-------------------------------------------------------------------------------------------------
MS_BOOL MDrv_GFLIP_SetFlipToGop(MS_U32 u32GopIdx, MS_U32 u32GwinIdx, MS_PHY64 u32MainAddr, MS_PHY64 u32SubAddr, MS_BOOL bForceWriteIn)
{
    MS_BOOL bRet= FALSE;
#ifdef GFLIP_GOP_TLB
    MS_PHY64 u32TLBMainAddr=0;
    MS_PHY64 u32TLBSubAddr=0;
    MS_PHY64 u32MainAddress=0;
    MS_PHY64 u32SubAddress=0;
    if(_bEnableGOPTLB[u32GopIdx] == TRUE)
    {
        u32TLBMainAddr=_u32GOPTLBaddress[u32GopIdx]+(_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32MainAddr/GOP_TLB_PAGE_SIZE)*TLB_PER_ENTRY_SIZE;
        u32TLBSubAddr=_u32GOPTLBaddress[u32GopIdx]+(_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32SubAddr/GOP_TLB_PAGE_SIZE)*TLB_PER_ENTRY_SIZE;
        u32TLBMainAddr /= ADDRESSING_8BYTE_UNIT;
        u32TLBSubAddr /= ADDRESSING_8BYTE_UNIT;
        u32MainAddress=_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32MainAddr % (GOP_TLB_PAGE_SIZE*PER_MIU_TLB_ENTRY_COUNT);
        u32SubAddress=_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32SubAddr % (GOP_TLB_PAGE_SIZE*PER_MIU_TLB_ENTRY_COUNT);
        MHal_GFLIP_SetTLBFlipToGop(u32GopIdx,u32GwinIdx,u32TLBMainAddr,u32TLBSubAddr,TRUE);
        bRet=MHal_GFLIP_SetFlipToGop(u32GopIdx,u32GwinIdx,u32MainAddress,u32SubAddress,TRUE);
    }
    else
#endif
    {
        bRet=MHal_GFLIP_SetFlipToGop(u32GopIdx, u32GwinIdx, u32MainAddr, u32SubAddr, bForceWriteIn);
    }
    return bRet;
}

static MS_BOOL _MDrv_GFLIP_FlipBufWithTimer(MS_BOOL* pbContiTimer, MS_BOOL *pbWakeupQue, MS_U32* pu32GOPIdx)
{
    MS_U32 u32GopIdx = 0, u32GwinIdx = 0;
    MS_BOOL bWakeUpQue = FALSE, bContinueTimer = FALSE;
#ifdef GFLIP_GOP_TLB
    MS_PHY64 u32TLBMainAddr=0;
    MS_PHY64 u32TLBSubAddr=0;
    MS_PHY64 u32MainAddress=0;
    MS_PHY64 u32SubAddress=0;
#endif

    *pu32GOPIdx = 0;
    *pbContiTimer = FALSE;
    *pbWakeupQue = FALSE;
    for(u32GopIdx=0; u32GopIdx<MAX_GOP_SUPPORT; u32GopIdx++)
    {
        for(u32GwinIdx=0; u32GwinIdx<MAX_GOP_GWIN; u32GwinIdx++)
        {
            //step 1:check if any flip request in queue with this gwin
            if(_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx] == _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]) //no any flip request in queue
            {
                continue;
            }

            bContinueTimer= TRUE;//There is new flip to check
            //Step 2: check if old request is done, if true:set ReadPtr to next, this entry consumed!
            if(_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].bKickOff == TRUE) //kicked off, check hw ack
            {
                if(MHal_GFLIP_IsGOPACK(u32GopIdx))
                {
                    bgflipVsyncStatus=TRUE;
                    //flip is done
                    bWakeUpQue = TRUE;
                    _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].bKickOff = FALSE;
                    _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx] = (_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]+1)%MAX_FLIP_ADDR_FIFO;
                }
            }
            else //no kick off, try kick off
            {
                //step 3: if get queue, check if TAG back.
                if(MHal_GFLIP_IsTagIDBack(_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32TagId) == FALSE)
                {
                    continue;
                }

                //Has Flip Request, and Tag Returned! we need programming flip address:
                //step 4: if Tag Back: set flip to GOP.
#ifdef GFLIP_GOP_TLB
                if(_bEnableGOPTLB[u32GopIdx] == TRUE)
                {
                    u32TLBMainAddr=_u32GOPTLBaddress[u32GopIdx]+(_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32MainAddr/GOP_TLB_PAGE_SIZE)*TLB_PER_ENTRY_SIZE;
                    u32TLBSubAddr=_u32GOPTLBaddress[u32GopIdx]+(_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32SubAddr/GOP_TLB_PAGE_SIZE)*TLB_PER_ENTRY_SIZE;
                    u32TLBMainAddr /= ADDRESSING_8BYTE_UNIT;
                    u32TLBSubAddr /= ADDRESSING_8BYTE_UNIT;
                    u32MainAddress=_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32MainAddr % (GOP_TLB_PAGE_SIZE*PER_MIU_TLB_ENTRY_COUNT);
                    u32SubAddress=_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32SubAddr % (GOP_TLB_PAGE_SIZE*PER_MIU_TLB_ENTRY_COUNT);
                    MHal_GFLIP_SetTLBFlipToGop(u32GopIdx,u32GwinIdx,u32TLBMainAddr,u32TLBSubAddr,TRUE);
                    MHal_GFLIP_SetFlipToGop(u32GopIdx, u32GwinIdx, _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32MainAddr,
                                            _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32SubAddr, FALSE);

                }
                else
#endif
                {
                    MHal_GFLIP_SetFlipToGop(u32GopIdx, u32GwinIdx, _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32MainAddr,
                                            _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32SubAddr, FALSE);
                }
                _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].bKickOff = TRUE;
                _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32KickOffStartTime = jiffies_to_msecs(jiffies);
            }
        }
    }

    *pu32GOPIdx = u32GopIdx;
    *pbContiTimer = bContinueTimer;
    *pbWakeupQue = bWakeUpQue;

    return TRUE;
}

#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
static enum hrtimer_restart MDrv_GFLIP_Timer_Callback( struct hrtimer *timer )
#else
static void MDrv_GFLIP_Timer_Callback(unsigned long value)
#endif
{
    MS_U32 u32GopIdx = 0, u32GwinIdx = 0;
    MS_BOOL bWakeUpQue = FALSE, bContinueTimer = FALSE;

#if (GFLIP_AUTO_DETECT_BUFFER_EN == 1)
    MS_BOOL bNeedToFlipCnt[MAX_GOP_SUPPORT] = {FALSE};
    MS_U16 u16RegAutoDetect = 0, u16RegGwinEn = 0;
#endif

#if (GFLIP_TIMER_TYPE == GFLIP_HZ_TIMER)
    //avoid reentry
    spin_lock_irq(&spinlock_gflip);
#else
    unsigned long flags;

    spin_lock_irqsave(&spinlock_gflip, flags);
#endif
    g_timer_callback_times++;

    _MDrv_GFLIP_FlipBufWithTimer(&bContinueTimer, &bWakeUpQue, &u32GopIdx);

#if (GFLIP_TIMER_TYPE == GFLIP_HZ_TIMER)
    spin_unlock_irq(&spinlock_gflip);
    if(bContinueTimer)
    {
        _stGflip_timer.data = u32GopIdx;
        _stGflip_timer.function = MDrv_GFLIP_Timer_Callback;
        _stGflip_timer.expires = jiffies + msecs_to_jiffies(GFLIP_TIMER_CHECK_TIME);
        mod_timer(&_stGflip_timer, _stGflip_timer.expires);
    }

#else
    spin_unlock_irqrestore(&spinlock_gflip, flags);
#endif

    if (bWakeUpQue && waitqueue_active(&_gflip_waitqueue))
    {
        wake_up_interruptible(&_gflip_waitqueue);
    }

#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
    if((timer != NULL) && bContinueTimer)
    {
        hrtimer_forward_now( timer, ns_to_ktime(MS_TO_NS(GFLIP_TIMER_CHECK_TIME)));
        return HRTIMER_RESTART;
    }
    else
    {
#if (GFLIP_AUTO_DETECT_BUFFER_EN == 1)
        for(u32GopIdx=0; u32GopIdx < MAX_GOP_SUPPORT; u32GopIdx++)
        {
            MHal_GFLIP_ReadGopReg(u32GopIdx, GFLIP_GOP_BANK_IDX_0, REG_GOP_AUTO_DETECT_BUF, &u16RegAutoDetect);
            if(u16RegAutoDetect & GOP_AUTO_DETECT_BUF_EN)
            {
                for(u32GwinIdx=0; u32GwinIdx < MAX_GOP_GWIN; u32GwinIdx++)
                {
                    if(_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx] != _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx])
                    {
                        bNeedToFlipCnt[u32GopIdx] = TRUE;
                        break;
                    }
                }
                if(bNeedToFlipCnt[u32GopIdx] != TRUE)
                {
                    MHal_GFLIP_IntEnable(u32GopIdx, FALSE); //disable ISR of u32GopIdx
                    for(u32GwinIdx=0; u32GwinIdx < MAX_GOP_GWIN; u32GwinIdx++)
                    {
                        MHal_GFLIP_ReadGopReg(u32GopIdx, GFLIP_GOP_BANK_IDX_1, REG_GOP4G_GWIN_CTRL0(u32GwinIdx), &u16RegGwinEn);
                        if((u16RegGwinEn & GOP_GWIN_ENABLE_MASK) != 0)
                        {
                            MHal_GFLIP_Fire(1 << u32GopIdx);
                            break;
                        }
                    }
                }
            }
        }
#endif
        return HRTIMER_NORESTART;
    }
#endif
}

#ifdef	GFLIP_MULTI_FLIP
static MS_BOOL _MDrv_GFLIP_MultiFlipBufWithTimer(MS_BOOL* pbContiTimer, MS_BOOL *pbWakeupQue, MS_U32* pu32GOPIdx)
{
    MS_U32 u32GopIdx = 0, u32GwinIdx = 0;
    MS_BOOL bWakeUpQue = FALSE, bContinueTimer = FALSE;
    MS_U8 i=0;

#ifdef GFLIP_GOP_TLB
    MS_BOOL bTLBenable = FALSE;
    MS_TLB_GFLIP_MULTIINFO* pTLBMultiFlipInfo;
#endif

    *pbContiTimer = FALSE;
    *pbWakeupQue = FALSE;
    *pu32GOPIdx = 0;
    for(u32GopIdx=0; u32GopIdx<MAX_GOP_SUPPORT; u32GopIdx++)
    {
        for(u32GwinIdx=0; u32GwinIdx<MAX_GOP_GWIN; u32GwinIdx++)
        {
            //step 1:check if any flip request in queue with this gwin
            if(_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx] == _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]) //no any flip request in queue
            {
                continue;
            }

            bContinueTimer= TRUE;//There is new flip to check
            //Step 2: check if old request is done, if true:set ReadPtr to next, this entry consumed!
            if(_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].bKickOff == TRUE) //kicked off, check hw ack
            {
                if(MHal_GFLIP_IsGOPACK(u32GopIdx))
                {
                    bgflipVsyncStatus=TRUE;
                    //flip is done
                    bWakeUpQue = TRUE;
                    _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].bKickOff = FALSE;
                    _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx] = (_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]+1)%MAX_FLIP_ADDR_FIFO;
                }
            }
            else //no kick off, try kick off
            {
                //step 3: if get queue, check if TAG back.
                if(MHal_GFLIP_IsTagIDBack(_GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32TagId) == FALSE)
                {
                    continue;
                }

                //Has Flip Request, and Tag Returned! we need programming flip address:
                //step 4: if Tag Back: set flip to GOP.
#ifdef GFLIP_GOP_TLB
                bTLBenable = FALSE;
                pTLBMultiFlipInfo =(MS_TLB_GFLIP_MULTIINFO* ) _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].pdata;
                for( i=0 ; i < pTLBMultiFlipInfo->u8GopInfoCnt ; i++ )
                {
                    if(pTLBMultiFlipInfo->astTLBGopInfo[i].bTLBEnable == TRUE)
                    {
                       bTLBenable = TRUE;
                    }
                }
                if(bTLBenable == TRUE)
                {
                    MHal_GFLIP_SetTLBMultiFlipToGop(pTLBMultiFlipInfo,FALSE);
                }
                else
#endif
                {
                    MS_GFLIP_MULTIINFO* pMultiFlipInfo =(MS_GFLIP_MULTIINFO* ) _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].pdata;
                    MHal_GFLIP_SetMultiFlipToGop(pMultiFlipInfo,FALSE);
                }
                _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].bKickOff = TRUE;
                _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]].u32KickOffStartTime = jiffies_to_msecs(jiffies);
            }
        }
    }

    *pbContiTimer = bContinueTimer;
    *pbWakeupQue = bWakeUpQue;
    *pu32GOPIdx = u32GopIdx;

    return TRUE;
}

#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
enum hrtimer_restart MDrv_GFLIP_MultiGOP_Timer_Callback( struct hrtimer *timer )
#else
static void MDrv_GFLIP_MultiGOP_Timer_Callback(unsigned long value)
#endif
{
    MS_U32 u32GopIdx = 0, u32GwinIdx = 0;
    MS_BOOL bWakeUpQue = FALSE, bContinueTimer = FALSE;
    MS_U8 i=0;

#if (GFLIP_AUTO_DETECT_BUFFER_EN == 1)
    MS_BOOL bNeedToFlipCnt[MAX_GOP_SUPPORT] = {FALSE};
    MS_U16 u16RegAutoDetect = 0, u16RegGwinEn = 0;
#endif

#if (GFLIP_TIMER_TYPE == GFLIP_HZ_TIMER)
    //avoid reentry
    spin_lock_irq(&spinlock_gflip);
#else
    unsigned long flags;

    spin_lock_irqsave(&spinlock_gflip, flags);
#endif

    g_Multi_timer_callback_times++;
    _MDrv_GFLIP_MultiFlipBufWithTimer(&bContinueTimer, &bWakeUpQue, &u32GopIdx);

#if (GFLIP_TIMER_TYPE == GFLIP_HZ_TIMER)
    spin_unlock_irq(&spinlock_gflip);
    if(bContinueTimer)
    {
        _stGflip_timer.data = u32GopIdx;
        _stGflip_timer.function = MDrv_GFLIP_Timer_Callback;
        _stGflip_timer.expires = jiffies + msecs_to_jiffies(GFLIP_TIMER_CHECK_TIME);
        mod_timer(&_stGflip_timer, _stGflip_timer.expires);
    }
#else
    spin_unlock_irqrestore(&spinlock_gflip, flags);
#endif

    if (bWakeUpQue && waitqueue_active(&_gflip_waitqueue))
    {
        wake_up(&_gflip_waitqueue);
    }

#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
    if((timer != NULL) && bContinueTimer)
    {
        hrtimer_forward_now( timer, ns_to_ktime(MS_TO_NS(GFLIP_TIMER_CHECK_TIME)));
        return HRTIMER_RESTART;
    }
    else
    {
#if (GFLIP_AUTO_DETECT_BUFFER_EN == 1)
        for(u32GopIdx=0; u32GopIdx < MAX_GOP_SUPPORT; u32GopIdx++)
        {
            MHal_GFLIP_ReadGopReg(u32GopIdx, GFLIP_GOP_BANK_IDX_0, REG_GOP_AUTO_DETECT_BUF, &u16RegAutoDetect);
            if(u16RegAutoDetect & GOP_AUTO_DETECT_BUF_EN)
            {
                for(u32GwinIdx=0; u32GwinIdx < MAX_GOP_GWIN; u32GwinIdx++)
                {
                    if(_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx] != _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx])
                    {
                        bNeedToFlipCnt[u32GopIdx] = TRUE;
                        break;
                    }
                }
                if(bNeedToFlipCnt[u32GopIdx] != TRUE)
                {
                    MHal_GFLIP_IntEnable(u32GopIdx, FALSE); //disable ISR of u32GopIdx
                    MHal_GFLIP_ReadGopReg(u32GopIdx, GFLIP_GOP_BANK_IDX_1, REG_GOP4G_GWIN_CTRL0(u32GwinIdx), &u16RegGwinEn);
                    if((u16RegGwinEn & GOP_GWIN_ENABLE_MASK) != 0)
                    {
                        MHal_GFLIP_Fire(1 << u32GopIdx);
                    }
                }
            }
        }
#endif
        return HRTIMER_NORESTART;
    }
#endif

}
#endif
#endif
//-------------------------------------------------------------------------------------------------
/// Set Flip Info(flip request) to GFlip
/// @param  u32GopIdx       \b IN: the gop which flip kickoff to
/// @param  u32GwinIdx       \b IN: the gwin which flip kickoff to
/// @param  u32Addr             \b IN: the flip address which will be kickoff
/// @param  u32TagId            \b IN: the TagId need be waited before flip
/// @param  u32QEntry           \b IN: the queued flips, if current queued flips >= u32QEntry, won't set this flip request to gop
/// @param  u32QEntry           \b out: the current queued flips in GFLIP
/// @param  u32Result   \b out: kickoff flip info success(TRUE) or fail(FALSE)
/// @return TRUE: DDI call success
/// @attention
/// <b>[OBAMA] <em>Flip when vsync interrupt, and the DDI used for set flip info to GFlip. </em></b>
//-------------------------------------------------------------------------------------------------
MS_BOOL _MDrv_GFLIP_SetFlipInfo(MS_U32 u32GopIdx, MS_U32 u32GwinIdx, MS_PHY64 u32MainAddr, MS_PHY64 u32SubAddr, MS_U32 u32TagId, MS_U32 * u32QEntry, MS_U32 *u32Result)
{
    MS_U32 u32NextWritePtr;
    MS_U32 u32QCnt = 0;
    MS_U64 cur_jiffies;
    MS_BOOL bRet = TRUE;
    static MS_U64 cur_timer_callback_times = 0x0;

    bgflipVsyncStatus=FALSE;
    cur_timer_callback_times = g_timer_callback_times;

    if ((u32GopIdx >= MAX_GOP_SUPPORT) || (u32GwinIdx >= MAX_GOP_GWIN)) //overflow
    {
        *u32Result = FALSE;
        return bRet;
    }
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
    unsigned long flags;
#endif

    if(TRUE == _bGFlipInVsyncLimitation[u32GopIdx])
    { //in handling vsync limitation status, just print the error and return true to avoid block app:
        printk("\n@@@@Error!!!! [%s][%d]no Signal for GOP Clock Source and GFlip Vsync Interrupt disabled!!!\n",__FUNCTION__,__LINE__);
        _bGFlipInVsyncLimitation[u32GopIdx] = false;
    }
    _MDrv_GFLIP_Mutex_Lock(u32GopIdx,TRUE);
    u32NextWritePtr = (_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]+1) % MAX_FLIP_ADDR_FIFO;
    if(u32NextWritePtr != _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]) //can write
    {
        if(_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] >= _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx])
        {
            u32QCnt = _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
        }
        else
        {
            u32QCnt = MAX_FLIP_ADDR_FIFO + _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
        }

        if( u32QCnt >= *u32QEntry )
        {
            printk("[GFLIP][%d]: unexpect case: u32QCnt %d, u32QEntry = %d", __LINE__, u32QCnt, *u32QEntry);
        }

        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].u32MainAddr = (u32MainAddr >> GFLIP_ADDR_ALIGN_RSHIFT);
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].u32SubAddr = (u32SubAddr >> GFLIP_ADDR_ALIGN_RSHIFT);
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].u32TagId= u32TagId;
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].bKickOff   = FALSE;
        _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] = u32NextWritePtr;
        _bEnableGOPTLB[u32GopIdx] =  FALSE;
        _u32GOPTLBaddress[u32GopIdx] =  NULL;
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
        spin_lock_irqsave(&spinlock_gflip, flags);
	    if(FALSE == hrtimer_active(&_stGflip_hrtimer)) //no call back timer, then add one
        {
            MS_BOOL bConTimer = FALSE, bWakeQue = FALSE;
            MS_U32 u32Gopid = 0;
            //printk(KERN_ALERT"[%s,%d][pid:%d][name:%s]_bTimerInited=%d\n",__func__,__LINE__,current->pid,current->comm,_bTimerInited);
            _stGflip_hrtimer.function = MDrv_GFLIP_Timer_Callback;

            _MDrv_GFLIP_FlipBufWithTimer(&bConTimer, &bWakeQue, &u32Gopid);

            hrtimer_start( &_stGflip_hrtimer, ns_to_ktime(MS_TO_NS(GFLIP_TIMER_CHECK_TIME)), HRTIMER_MODE_REL );
        }
        else
        {
            if(_stGflip_hrtimer.function == NULL)
            {
                printk(KERN_ALERT"[%s,%d][pid:%d][name:%s]_stGflip_hrtimer.function is NULL\n",__func__,__LINE__,current->pid,current->comm);
            }
        }
        spin_unlock_irqrestore(&spinlock_gflip, flags);
#else
        mb();
        if(FALSE == timer_pending(&_stGflip_timer)) //no call back timer, then add one
        {
            _stGflip_timer.data = u32GopIdx;
            _stGflip_timer.function = MDrv_GFLIP_Timer_Callback;
            _stGflip_timer.expires = jiffies + msecs_to_jiffies(GFLIP_TIMER_CHECK_TIME);
            mod_timer(&_stGflip_timer, _stGflip_timer.expires);
        }
        MDrv_GFLIP_Timer_Callback(0);
#endif
#endif

#if (GFLIP_AUTO_DETECT_BUFFER_EN == 1)
        MS_U16 u16RegAutoDetect = 0;

        if(_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] != _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx])
        {
            MHal_GFLIP_ReadGopReg(u32GopIdx, GFLIP_GOP_BANK_IDX_0, REG_GOP_AUTO_DETECT_BUF, &u16RegAutoDetect);
            if(u16RegAutoDetect & GOP_AUTO_DETECT_BUF_EN)
            {
                MHal_GFLIP_IntEnable(u32GopIdx, TRUE); //enable ISR of u32GopIdx
            }
        }
#endif

        //Hold on when New QCnt can't match requested QCnt:
        //Fix issue if we set Flip Info and back which leads to App write to on-show window.
        cur_jiffies = jiffies;
        while(TRUE)
        {
            if(_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] >= _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx])
            {
                u32QCnt = _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
            }
            else
            {
                u32QCnt = MAX_FLIP_ADDR_FIFO + _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
            }

            if( u32QCnt < *u32QEntry )
            {
                break;
            }

            if(time_after(jiffies, (unsigned long)(msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT)+cur_jiffies)))
            {
                printk("[GFLIP][%d]: flip failed, cur_timer_callback_times %llu, g_timer_callback_times %llu  \r\n",
                        __LINE__, cur_timer_callback_times, g_timer_callback_times);
                *u32Result = FALSE;
                bRet = TRUE;
                goto _Release_Mutex;
            }
            if (0 == wait_event_interruptible_timeout(_gflip_waitqueue, bgflipVsyncStatus,msecs_to_jiffies(FLIP_WAITQ_ONESHOT_TIMEOUT)))
            {
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
                if(FALSE == hrtimer_active(&_stGflip_hrtimer))
                {
                    printk("[%s][%d]timer off now, start hr timer again \n",__FUNCTION__,__LINE__);
                    MS_BOOL bConTimer = FALSE, bWakeQue = FALSE;
                    MS_U32 u32Gopid = 0;
                    _stGflip_hrtimer.function = MDrv_GFLIP_Timer_Callback;
                    _MDrv_GFLIP_FlipBufWithTimer(&bConTimer, &bWakeQue, &u32Gopid);
                    hrtimer_start( &_stGflip_hrtimer, ns_to_ktime(MS_TO_NS(GFLIP_TIMER_CHECK_TIME)), HRTIMER_MODE_REL );
                }
#endif
#endif
            }

        }

        //since do not do the sync. with mutex/spinlock, the return of avaiable queue number maybe not accurate.
        *u32QEntry = u32QCnt + 1;
        *u32Result = TRUE;
        bRet = TRUE;
    }
    else
    {
        *u32QEntry = MAX_FLIP_ADDR_FIFO-1;
        *u32Result = FALSE;
        bRet = TRUE;
    }
_Release_Mutex:
    _MDrv_GFLIP_Mutex_Lock(u32GopIdx,FALSE);
    return bRet;
}
//-------------------------------------------------------------------------------------------------
/// Set Flip Info(flip request) to GFlip
/// @param  u32GopIdx       \b IN: the gop which flip kickoff to
/// @param  u32GwinIdx       \b IN: the gwin which flip kickoff to
/// @param  u32Addr             \b IN: the flip address which will be kickoff
/// @param  u32TagId            \b IN: the TagId need be waited before flip
/// @param  u32QEntry           \b IN: the queued flips, if current queued flips >= u32QEntry, won't set this flip request to gop
/// @param  u32QEntry           \b out: the current queued flips in GFLIP
/// @param  u32Result   \b out: kickoff flip info success(TRUE) or fail(FALSE)
/// @param  u32Addr             \b IN: TLB base address
/// @param  u32Addr             \b IN: TLB enable
/// @return TRUE: DDI call success
/// @attention
/// <b>[OBAMA] <em>Flip when vsync interrupt, and the DDI used for set flip info to GFlip. </em></b>
//-------------------------------------------------------------------------------------------------
MS_BOOL MDrv_GFLIP_SetTLBFlipInfo(MS_U32 u32GopIdx, MS_U32 u32GwinIdx, MS_PHY64 u32MainAddr, MS_PHY64 u32SubAddr, MS_U32 u32TagId, MS_U32 * u32QEntry, MS_U32 *u32Result, MS_BOOL bTLBenable, MS_PHY64 u32TLBAddr)
{
    MS_U32 u32NextWritePtr;
    MS_U32 u32QCnt = 0;
    MS_U64 cur_jiffies;
    MS_BOOL bRet = TRUE;
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE)) && (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
    unsigned long flags;
    enum hrtimer_restart (*timer_cb)(struct hrtimer *);
#endif
   static MS_U64 cur_Multi_timer_callback_times = 0x0;

    bgflipVsyncStatus = FALSE;
    cur_Multi_timer_callback_times = g_Multi_timer_callback_times;
    if ((u32GopIdx >= MAX_GOP_SUPPORT) || (u32GwinIdx >= MAX_GOP_GWIN)) //overflow
    {
        *u32Result = FALSE;
        return bRet;
    }

    if(TRUE == _bGFlipInVsyncLimitation[u32GopIdx])
    { //in handling vsync limitation status, just print the error and return true to avoid block app:
        printk("\n@@@@Error!!!![%s][%d] no Signal for GOP Clock Source and GFlip Vsync Interrupt disabled!!!\n",__FUNCTION__,__LINE__);
        _bGFlipInVsyncLimitation[u32GopIdx] = false;
    }
    _MDrv_GFLIP_Mutex_Lock(u32GopIdx,TRUE);
    u32NextWritePtr = (_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]+1) % MAX_FLIP_ADDR_FIFO;
    if(u32NextWritePtr != _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]) //can write
    {
        if(_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] >= _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx])
        {
            u32QCnt = _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
        }
        else
        {
            u32QCnt = MAX_FLIP_ADDR_FIFO + _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
        }

        if( u32QCnt >= *u32QEntry )
        {
            printk("[GFLIP][%d]: unexpect case: u32QCnt %d, u32QEntry = %d", __LINE__, u32QCnt, *u32QEntry);
        }

        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].u32MainAddr = (u32MainAddr >> GFLIP_ADDR_ALIGN_RSHIFT);
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].u32SubAddr = (u32SubAddr >> GFLIP_ADDR_ALIGN_RSHIFT);
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].u32TagId= u32TagId;
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].bKickOff   = FALSE;
        _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] = u32NextWritePtr;
        _bEnableGOPTLB[u32GopIdx] =  bTLBenable;
        _u32GOPTLBaddress[u32GopIdx] =  u32TLBAddr;
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
        //enum hrtimer_restart (*timer_cb)(struct hrtimer *);

#if ( defined (GFLIP_MULTI_FLIP))
        timer_cb = MDrv_GFLIP_MultiGOP_Timer_Callback;
#else
        timer_cb = MDrv_GFLIP_Timer_Callback;
#endif
        spin_lock_irqsave(&spinlock_gflip, flags);
        if(FALSE == hrtimer_active(&_stGflip_hrtimer)) //no call back timer, then add one
        {
            MS_BOOL bConTimer = FALSE, bWakeQue = FALSE;
            MS_U32 u32Gopid = 0;

            //printk(KERN_ALERT"[%s,%d][pid:%d][name:%s]_bTimerInited=%d\n",__func__,__LINE__,current->pid,current->comm,_bTimerInited);
            _stGflip_hrtimer.function = timer_cb;
#if ( defined (GFLIP_MULTI_FLIP))
            _MDrv_GFLIP_MultiFlipBufWithTimer(&bConTimer, &bWakeQue, &u32Gopid);
#else
            _MDrv_GFLIP_FlipBufWithTimer(&bConTimer, &bWakeQue, &u32Gopid);
#endif
            hrtimer_start( &_stGflip_hrtimer, ns_to_ktime(MS_TO_NS(GFLIP_TIMER_CHECK_TIME)), HRTIMER_MODE_REL );
        }
        spin_unlock_irqrestore(&spinlock_gflip, flags);
#else
        mb();
        if(FALSE == timer_pending(&_stGflip_timer)) //no call back timer, then add one
        {
            _stGflip_timer.data = u32GopIdx;
            _stGflip_timer.function = MDrv_GFLIP_Timer_Callback;
            _stGflip_timer.expires = jiffies + msecs_to_jiffies(GFLIP_TIMER_CHECK_TIME);
            mod_timer(&_stGflip_timer, _stGflip_timer.expires);
        }
        MDrv_GFLIP_Timer_Callback(0);
#endif
#endif
        //Hold on when New QCnt can't match requested QCnt:
        //Fix issue if we set Flip Info and back which leads to App write to on-show window.
        cur_jiffies = jiffies;
        while(TRUE)
        {
            if(_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] >= _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx])
            {
                u32QCnt = _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
            }
            else
            {
                u32QCnt = MAX_FLIP_ADDR_FIFO + _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
            }

            if( u32QCnt < *u32QEntry )
            {
                break;
            }

            if(time_after(jiffies, (unsigned long)(msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT)+cur_jiffies)))
            {
                 printk("[GFLIP][%d]: flip failed, cur_Multi_timer_callback_times %llu, g_Multi_timer_callback_times %llu  \r\n",
                         __LINE__, cur_Multi_timer_callback_times, g_Multi_timer_callback_times);
                *u32Result = FALSE;
                bRet = TRUE;
                goto _Release_Mutex;
            }

            if (0 == wait_event_interruptible_timeout(_gflip_waitqueue, bgflipVsyncStatus ,msecs_to_jiffies(FLIP_WAITQ_ONESHOT_TIMEOUT)))
            {
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
                if(FALSE == hrtimer_active(&_stGflip_hrtimer)) //no call back timer, then add one
                {
                    printk("[%s][%d]timer off now, start hr timer again \n",__FUNCTION__,__LINE__);
                    MS_BOOL bConTimer = FALSE, bWakeQue = FALSE;
                    MS_U32 u32Gopid = 0;
#if ( defined (GFLIP_MULTI_FLIP))
                    timer_cb = MDrv_GFLIP_MultiGOP_Timer_Callback;
                    _MDrv_GFLIP_MultiFlipBufWithTimer(&bConTimer, &bWakeQue, &u32Gopid);
#else
                    timer_cb = MDrv_GFLIP_Timer_Callback;
                    _MDrv_GFLIP_FlipBufWithTimer(&bConTimer, &bWakeQue, &u32Gopid);
#endif
                    hrtimer_start( &_stGflip_hrtimer, ns_to_ktime(MS_TO_NS(GFLIP_TIMER_CHECK_TIME)), HRTIMER_MODE_REL );
                }
#endif
#endif
            }
        }

        //since do not do the sync. with mutex/spinlock, the return of avaiable queue number maybe not accurate.
        *u32QEntry = u32QCnt + 1;
        *u32Result = TRUE;
        bRet = TRUE;
    }
    else
    {
        *u32QEntry = MAX_FLIP_ADDR_FIFO-1;
        *u32Result = FALSE;
        bRet = TRUE;
    }
_Release_Mutex:
    _MDrv_GFLIP_Mutex_Lock(u32GopIdx,FALSE);
    return bRet;
}

#ifdef	GFLIP_MULTI_FLIP
MS_BOOL _MDrv_GFLIP_SetMultiFlipInfo(MS_GFLIP_MULTIINFO* pMultiFlipInfo)
{
    MS_U32 u32NextWritePtr;
    MS_U32 u32QCnt = 0;
    MS_U64 cur_jiffies;
    MS_BOOL bRet = TRUE;
    MS_U32 u32GopIdx = 0,u32GwinIdx = 0,u32TagId = 0;
    MS_U32 *u32QEntry = NULL,*u32Result = NULL;
    MS_PHY64 u32MainAddr = 0,u32SubAddr = 0;

    u32GopIdx = pMultiFlipInfo->astGopInfo[0].u32GopIdx;
    u32GwinIdx = pMultiFlipInfo->astGopInfo[0].u32GwinIdx;
    u32TagId = pMultiFlipInfo->astGopInfo[0].u32TagId;
    u32QEntry = &(pMultiFlipInfo->astGopInfo[0].u32QEntry);
    u32Result = &(pMultiFlipInfo->astGopInfo[0].u32Result);
    u32MainAddr = pMultiFlipInfo->astGopInfo[0].u32MainAddr;
    u32SubAddr = pMultiFlipInfo->astGopInfo[0].u32SubAddr;
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
    unsigned long flags;
#endif
    static MS_U64 cur_Multi_timer_callback_times = 0x0;

    bgflipVsyncStatus = FALSE;
    cur_Multi_timer_callback_times = g_Multi_timer_callback_times;
    if ((u32GopIdx >= MAX_GOP_SUPPORT) || (u32GwinIdx >= MAX_GOP_GWIN)) //overflow
    {
        *u32Result = FALSE;
        printk("\33[0;36m   %s:%d    \33[m \n",__FUNCTION__,__LINE__);
        return bRet;
    }

    if(TRUE == _bGFlipInVsyncLimitation[u32GopIdx])
    { //in handling vsync limitation status, just print the error and return true to avoid block app:
        printk("\n@@@@Error!!!! [%s:%d], no Signal for GOP Clock Source and GFlip Vsync Interrupt disabled!!!\n",__FUNCTION__,__LINE__);
        _bGFlipInVsyncLimitation[u32GopIdx] = false;
    }
    _MDrv_GFLIP_Mutex_Lock(u32GopIdx,TRUE);
    u32NextWritePtr = (_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]+1) % MAX_FLIP_ADDR_FIFO;
    if(u32NextWritePtr != _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]) //can write
    {
        if(_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] >= _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx])
        {
            u32QCnt = _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
        }
        else
        {
            u32QCnt = MAX_FLIP_ADDR_FIFO + _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
        }

        if( u32QCnt >= *u32QEntry )
        {
            printk("[GFLIP][%d]: unexpect case: u32QCnt %d, u32QEntry = %d", __LINE__, u32QCnt, *u32QEntry);
        }

        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].u32MainAddr = (u32MainAddr >> GFLIP_ADDR_ALIGN_RSHIFT) ;
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].u32SubAddr = (u32SubAddr >> GFLIP_ADDR_ALIGN_RSHIFT);;
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].u32TagId= u32TagId;
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].bKickOff   = FALSE;
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].pdata = (MS_U32* )pMultiFlipInfo;
        _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] = u32NextWritePtr;
        _bEnableGOPTLB[u32GopIdx] =  FALSE;
        _u32GOPTLBaddress[u32GopIdx] =  0;
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
        spin_lock_irqsave(&spinlock_gflip, flags);
        if(FALSE == hrtimer_active(&_stGflip_hrtimer)) //no call back timer, then add one
        {
            MS_BOOL bConTimer = FALSE, bWakeQue = FALSE;
            MS_U32 u32Gopid = 0;

            _stGflip_hrtimer.function = MDrv_GFLIP_MultiGOP_Timer_Callback;

            _MDrv_GFLIP_MultiFlipBufWithTimer(&bConTimer, &bWakeQue, &u32Gopid);

            hrtimer_start( &_stGflip_hrtimer, ns_to_ktime(MS_TO_NS(GFLIP_TIMER_CHECK_TIME)), HRTIMER_MODE_REL );
        }
        else
        {
            if(_stGflip_hrtimer.function == NULL)
            {
                printk(KERN_ALERT"[%s,%d][pid:%d][name:%s]_stGflip_hrtimer.function is NULL\n",__func__,__LINE__,current->pid,current->comm);
            }
        }
        spin_unlock_irqrestore(&spinlock_gflip, flags);
#else
        mb();
        if(FALSE == timer_pending(&_stGflip_timer)) //no call back timer, then add one
        {
            _stGflip_timer.data = u32GopIdx;
            _stGflip_timer.function = MDrv_GFLIP_MultiGOP_Timer_Callback;
            _stGflip_timer.expires = jiffies + msecs_to_jiffies(GFLIP_TIMER_CHECK_TIME);
            mod_timer(&_stGflip_timer, _stGflip_timer.expires);
        }
        MDrv_GFLIP_MultiGOP_Timer_Callback(0);
#endif
#endif

#if (GFLIP_AUTO_DETECT_BUFFER_EN == 1)
        MS_U16 u16RegAutoDetect = 0;

        if(_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] != _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx])
        {
            MHal_GFLIP_ReadGopReg(u32GopIdx, GFLIP_GOP_BANK_IDX_0, REG_GOP_AUTO_DETECT_BUF, &u16RegAutoDetect);
            if(u16RegAutoDetect & GOP_AUTO_DETECT_BUF_EN)
            {
                MHal_GFLIP_IntEnable(u32GopIdx, TRUE); //enable ISR of u32GopIdx
            }
        }
#endif

        //Hold on when New QCnt can't match requested QCnt:
        //Fix issue if we set Flip Info and back which leads to App write to on-show window.
        cur_jiffies = jiffies;
        while(TRUE)
        {
            if(_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] >= _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx])
            {
                u32QCnt = _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
            }
            else
            {
                u32QCnt = MAX_FLIP_ADDR_FIFO + _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
            }

            if( u32QCnt < *u32QEntry )
            {
                break;
            }

            if(time_after(jiffies, (unsigned long)(msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT)+cur_jiffies)))
            {
                printk("[GFLIP][%d]: flip failed, cur_Multi_timer_callback_times %llu, g_Multi_timer_callback_times %llu  \r\n",
                        __LINE__, cur_Multi_timer_callback_times, g_Multi_timer_callback_times);
                *u32Result = FALSE;
                bRet = TRUE;
                goto _Release_Mutex;
            }
            if(0 == wait_event_interruptible_timeout(_gflip_waitqueue, bgflipVsyncStatus,msecs_to_jiffies(FLIP_WAITQ_ONESHOT_TIMEOUT)))
            {
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
                 if(FALSE == hrtimer_active(&_stGflip_hrtimer))
                 {
                    printk("[%s][%d]timer off now, start hr timer again \n",__FUNCTION__,__LINE__);
                    MS_BOOL bConTimer = FALSE, bWakeQue = FALSE;
                    MS_U32 u32Gopid = 0;
                    _stGflip_hrtimer.function = MDrv_GFLIP_MultiGOP_Timer_Callback;
                    _MDrv_GFLIP_MultiFlipBufWithTimer(&bConTimer, &bWakeQue, &u32Gopid);
                    hrtimer_start( &_stGflip_hrtimer, ns_to_ktime(MS_TO_NS(GFLIP_TIMER_CHECK_TIME)), HRTIMER_MODE_REL );
                 }
#endif
#endif
            }
        }

        //since do not do the sync. with mutex/spinlock, the return of avaiable queue number maybe not accurate.
        *u32QEntry = u32QCnt + 1;
        *u32Result = TRUE;
        bRet = TRUE;
    }
    else
    {
        *u32QEntry = MAX_FLIP_ADDR_FIFO-1;
        *u32Result = FALSE;
        bRet = TRUE;
    }
_Release_Mutex:
    _MDrv_GFLIP_Mutex_Lock(u32GopIdx,FALSE);
    return bRet;
}

MS_BOOL _MDrv_GFLIP_SetTLBMultiFlipInfo(MS_TLB_GFLIP_MULTIINFO* pTLBMultiFlipInfo)
{
    MS_U32 u32NextWritePtr;
    MS_U32 u32QCnt = 0;
    MS_U64 cur_jiffies;
    MS_BOOL bRet = TRUE;
    MS_U32 u32GopIdx = 0,u32GwinIdx = 0,u32TagId = 0;
    MS_U32 *u32QEntry = NULL,*u32Result = NULL;
    MS_PHY64 u32MainAddr = 0,u32SubAddr = 0;
    MS_BOOL bTLBenable = FALSE;
    MS_PHY64 u32TLBAddr = 0;
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
    unsigned long flags;
#endif
    static MS_U64 cur_Multi_timer_callback_times = 0x0;

    u32GopIdx = pTLBMultiFlipInfo->astTLBGopInfo[0].u32GopIdx;
    u32GwinIdx = pTLBMultiFlipInfo->astTLBGopInfo[0].u32GwinIdx;
    u32TagId = pTLBMultiFlipInfo->astTLBGopInfo[0].u32TagId;
    u32QEntry = &(pTLBMultiFlipInfo->astTLBGopInfo[0].u32QEntry);
    u32Result = &(pTLBMultiFlipInfo->astTLBGopInfo[0].u32Result);
    u32MainAddr = pTLBMultiFlipInfo->astTLBGopInfo[0].u32MainAddr;
    u32SubAddr = pTLBMultiFlipInfo->astTLBGopInfo[0].u32SubAddr;
    bTLBenable = pTLBMultiFlipInfo->astTLBGopInfo[0].bTLBEnable;
    u32TLBAddr = pTLBMultiFlipInfo->astTLBGopInfo[0].u32TLBAddr;

    bgflipVsyncStatus = FALSE;
    cur_Multi_timer_callback_times = g_Multi_timer_callback_times;
    if ((u32GopIdx >= MAX_GOP_SUPPORT) || (u32GwinIdx >= MAX_GOP_GWIN)) //overflow
    {
        *u32Result = FALSE;
        printk("\33[0;36m   %s:%d    \33[m \n",__FUNCTION__,__LINE__);
        return bRet;
    }

    if(TRUE == _bGFlipInVsyncLimitation[u32GopIdx])
    { //in handling vsync limitation status, just print the error and return true to avoid block app:
        printk("\n@@@@Error!!!! [%s][%d], no Signal for GOP Clock Source and GFlip Vsync Interrupt disabled!!!\n",__FUNCTION__,__LINE__);
        _bGFlipInVsyncLimitation[u32GopIdx] = false;
    }
    _MDrv_GFLIP_Mutex_Lock(u32GopIdx,TRUE);
    u32NextWritePtr = (_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]+1) % MAX_FLIP_ADDR_FIFO;
    if(u32NextWritePtr != _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx]) //can write
    {
        if(_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] >= _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx])
        {
            u32QCnt = _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
        }
        else
        {
            u32QCnt = MAX_FLIP_ADDR_FIFO + _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
        }

        if( u32QCnt >= *u32QEntry )
        {
            printk("[GFLIP][%d]: unexpect case: u32QCnt %d, u32QEntry = %d", __LINE__, u32QCnt, *u32QEntry);
        }

        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].u32MainAddr = (u32MainAddr >> GFLIP_ADDR_ALIGN_RSHIFT) ;
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].u32SubAddr = (u32SubAddr >> GFLIP_ADDR_ALIGN_RSHIFT);;
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].u32TagId= u32TagId;
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].bKickOff   = FALSE;
        _GFlipInfo[u32GopIdx][u32GwinIdx][_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]].pdata = (MS_U32* )pTLBMultiFlipInfo;
        _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] = u32NextWritePtr;
        _bEnableGOPTLB[u32GopIdx] =  bTLBenable;
        _u32GOPTLBaddress[u32GopIdx] =  u32TLBAddr;
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
        spin_lock_irqsave(&spinlock_gflip, flags);
        if(FALSE == hrtimer_active(&_stGflip_hrtimer)) //no call back timer, then add one
        {
            MS_BOOL bConTimer = FALSE, bWakeQue = FALSE;
            MS_U32 u32Gopid = 0;

            _stGflip_hrtimer.function = MDrv_GFLIP_MultiGOP_Timer_Callback;
            _MDrv_GFLIP_MultiFlipBufWithTimer(&bConTimer, &bWakeQue, &u32Gopid);
            hrtimer_start( &_stGflip_hrtimer, ns_to_ktime(MS_TO_NS(GFLIP_TIMER_CHECK_TIME)), HRTIMER_MODE_REL );
        }
        spin_unlock_irqrestore(&spinlock_gflip, flags);
#else
        mb();
        if(FALSE == timer_pending(&_stGflip_timer)) //no call back timer, then add one
        {
            _stGflip_timer.data = u32GopIdx;
            _stGflip_timer.function = MDrv_GFLIP_MultiGOP_Timer_Callback;
            _stGflip_timer.expires = jiffies + msecs_to_jiffies(GFLIP_TIMER_CHECK_TIME);
            mod_timer(&_stGflip_timer, _stGflip_timer.expires);
        }
        MDrv_GFLIP_MultiGOP_Timer_Callback(0);
#endif
#endif
        //Hold on when New QCnt can't match requested QCnt:
        //Fix issue if we set Flip Info and back which leads to App write to on-show window.
        cur_jiffies = jiffies;
        while(TRUE)
        {
            if(_u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] >= _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx])
            {
                u32QCnt = _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
            }
            else
            {
                u32QCnt = MAX_FLIP_ADDR_FIFO + _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx] - _u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx];
            }

            if( u32QCnt < *u32QEntry )
            {
                break;
            }

            if(time_after(jiffies, (unsigned long)(msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT)+cur_jiffies)))
            {
                printk("[GFLIP][%d]: flip failed, cur_Multi_timer_callback_times %llu, g_Multi_timer_callback_times %llu  \r\n",
                         __LINE__, cur_Multi_timer_callback_times, g_Multi_timer_callback_times);
                *u32Result = FALSE;
                bRet = TRUE;
                goto _Release_Mutex;
            }

            if(0 == wait_event_interruptible_timeout(_gflip_waitqueue, bgflipVsyncStatus,msecs_to_jiffies(FLIP_WAITQ_ONESHOT_TIMEOUT)))
            {
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
#if (GFLIP_TIMER_TYPE == GFLIP_HR_TIMER)
                if(FALSE == hrtimer_active(&_stGflip_hrtimer)) //no call back timer, then add one
                {
                    MS_BOOL bConTimer = FALSE, bWakeQue = FALSE;
                    MS_U32 u32Gopid = 0;

                    _stGflip_hrtimer.function = MDrv_GFLIP_MultiGOP_Timer_Callback;
                    _MDrv_GFLIP_MultiFlipBufWithTimer(&bConTimer, &bWakeQue, &u32Gopid);
                    hrtimer_start( &_stGflip_hrtimer, ns_to_ktime(MS_TO_NS(GFLIP_TIMER_CHECK_TIME)), HRTIMER_MODE_REL );
                }
#endif
#endif
            }
        }

        //since do not do the sync. with mutex/spinlock, the return of avaiable queue number maybe not accurate.
        *u32QEntry = u32QCnt + 1;
        *u32Result = TRUE;
        bRet = TRUE;
    }
    else
    {
        *u32QEntry = MAX_FLIP_ADDR_FIFO-1;
        *u32Result = FALSE;
        bRet = TRUE;
    }
_Release_Mutex:
    _MDrv_GFLIP_Mutex_Lock(u32GopIdx,FALSE);
    return bRet;
}
#endif
MS_BOOL MDrv_GFLIP_SetPixelIDAddr(MS_U32 u32GopIdx, MS_U32 u32GwinIdx, MS_PHY64 u32Addr, MS_U32 u32TagId, MS_U32 * u32QEntry, MS_U32 *u32Result)
{
    if ((u32GopIdx >= MAX_GOP_SUPPORT) || (u32GwinIdx >= MAX_GOP_GWIN)) //overflow
    {
        *u32Result = FALSE;
        return TRUE;
    }

    *u32Result = TRUE;
    if(u32Addr != 0)
    {
#if defined(CONFIG_MIPS)
        g_remaplastGwinAddr = (MS_U32)__ioremap(u32Addr + 0, 100, _CACHE_UNCACHED);
#elif defined(CONFIG_ARM)
        g_remaplastGwinAddr = (MS_U32) ioremap(u32Addr + 0, 100);
#endif
    }
    else
    {
        g_remaplastGwinAddr = 0;
    }

    return TRUE;
}

MS_BOOL _MDrv_GFLIP_SetGPIO3DPin(MS_PHY64 u32Addr, MS_U32 *u32Result)
{
    *u32Result = TRUE;

    g_GPIO3DPin = u32Addr;

    return TRUE;
}

#if	( defined (CONFIG_MSTAR_VE_CAPTURE_SUPPORT))
//-------------------------------------------------------------------------------------------------
/// Get VE capture state: Enable state and the FrameNumber that has been full captured
/// @param  pbEna                  \b OUT: TRUE: Enable, FALSE: Disable
/// @param  pu8FramCount           \b OUT: Current captured FrameNumber
/// @return TRUE: success
/// @return FALSE: fail
//-------------------------------------------------------------------------------------------------
MS_BOOL MDrv_GFLIP_GetVECapCurState(MS_BOOL *pbEna, MS_U8 *pu8FramCount)
{
    *pbEna = s_bEnable;
    *pu8FramCount = s_u8FrameCount;
    //printk("KL: %u,%u\n", s_bEnable, s_u8FrameCount);
    return TRUE;
}

//-------------------------------------------------------------------------------------------------
/// Wait on the finish of specified frame: Check if the frame is captured,
/// if Yes, return TRUE, otherwise sleep until the next Vsync ISR
/// @param  pbEna                  \b OUT: TRUE: Enable, FALSE: Disable
/// @param  pu8FramCount           \b IN : Specified frame number to waiting
/// @return TRUE: success
/// @return FALSE: fail
//-------------------------------------------------------------------------------------------------
#define TIMEOUT_SLEEP 60
MS_BOOL MDrv_GFLIP_VECapWaitOnFrame(MS_BOOL *pbEna, MS_U8 *pu8FramNum)
{
    MS_BOOL bret = FALSE;
    *pbEna = s_bEnable;
    if(s_bEnable == TRUE)
    {
        if(((s_stGflipVECConfig.bInterlace == FALSE) && (s_u8FrameNumUpdated == *pu8FramNum)) ||
           ((s_stGflipVECConfig.bInterlace == TRUE ) && (s_u8FrameNumUpdated == (*pu8FramNum<<1)))
          )
        {
            bret = TRUE;//State change, return TRUE
            s_u8FrameNumUpdated = VE_CAPTURE_FRAME_INVALID_NUM; //reset to wait next update frame
            //printk("<0>" "WS=%u\n", *pu8FramNum);
        }
        else
        {
            bgcaptureVsyncStatus=FALSE;
            wait_event_interruptible_timeout(_gcapture_waitqueue, bgcaptureVsyncStatus,msecs_to_jiffies(TIMEOUT_SLEEP));
            //printk("<0>" "WF=%u\n", *pu8FramNum);
        }
    }
    else
    {
        bgcaptureVsyncStatus=FALSE;
        wait_event_interruptible_timeout(_gcapture_waitqueue, bgcaptureVsyncStatus,msecs_to_jiffies(TIMEOUT_SLEEP));
    }
    //printk("KL: %u,%u\n", s_bEnable, s_u8FrameCount);
    return bret;
}

//-------------------------------------------------------------------------------------------------
/// Set VE capture state: Enable or Disable VE capture function
/// @param  pbEna                  \b IN: TRUE: Enable, FALSE: Disable
/// @return TRUE: success
/// @return FALSE: fail
/// @attention
///           the enable or disable operation will be taken in the following vsync interrupt process.
//-------------------------------------------------------------------------------------------------
MS_BOOL MDrv_GFLIP_SetVECapCurState(MS_BOOL *pbEna)
{
    //printk("<0>" "VEC state change=%u->%u\n", s_bEnable, *pbEna);
    s_bEnable = *pbEna;
    s_u8FrameCount = VE_CAPTURE_FRAME_INVALID_NUM;
    s_u8FrameNumUpdated = VE_CAPTURE_FRAME_INVALID_NUM; //Valid Frame is 1~3 for P mode or 1~4 for interlace mode
    return TRUE;
}

//-------------------------------------------------------------------------------------------------
/// Config the VE capture
/// otherwise sleep until the next Vsync ISR
/// @param  pstGflipVECConfig      \b IN : @ref PMS_GFLIP_VEC_CONFIG
/// @return TRUE: success
/// @return FALSE: fail
//-------------------------------------------------------------------------------------------------
void MDrv_GFLIP_GetVECaptureConfig(PMS_GFLIP_VEC_CONFIG pstGflipVECConfig)
{
    memcpy(pstGflipVECConfig, &s_stGflipVECConfig, sizeof(MS_GFLIP_VEC_CONFIG));
}

void MDrv_GFLIP_SetVECaptureConfig(PMS_GFLIP_VEC_CONFIG pstGflipVECConfig)
{
    pstGflipVECConfig->u8Result = TRUE;
    if(pstGflipVECConfig->eConfigType == MS_VEC_CONFIG_ENABLE)
    {
        s_stGflipVECConfig.eConfigType = MS_VEC_CONFIG_ENABLE;
        s_stGflipVECConfig.bInterlace  = pstGflipVECConfig->bInterlace;
    }
    else if(pstGflipVECConfig->eConfigType == MS_VEC_CONFIG_INIT)
    {
        memcpy(&s_stGflipVECConfig, pstGflipVECConfig, sizeof(MS_GFLIP_VEC_CONFIG));
        s_stGflipVECConfig.u8MaxFrameNumber_I <<= 1; //In kernel it is counted by field for interlace in ISR
        if(s_stGflipVECConfig.eIsrType >= MS_VEC_ISR_MAXNUM)
        {
            s_stGflipVECConfig.eIsrType = MS_VEC_ISR_GOP_OP;
            pstGflipVECConfig->u8Result = FALSE;//Unknown config type
        }
    }
    else
    {
        pstGflipVECConfig->u8Result = FALSE;//Unknown config type
    }
}

//-------------------------------------------------------------------------------------------------
/// Clear Irq
/// @param  void
/// @return TRUE: success
/// @attention
//-------------------------------------------------------------------------------------------------
MS_BOOL MDrv_GFLIP_VEC_ClearIRQ(void)
{
    if(s_stGflipVECConfig.eIsrType == MS_VEC_ISR_VE)
    {
        //No need to clear for FIQ interrupt, just wake up queue
        if ( waitqueue_active(&_gcapture_waitqueue))
        {
            bgcaptureVsyncStatus=TRUE;
            wake_up_interruptible(&_gcapture_waitqueue);
        }
        return TRUE;
    }
    return FALSE;
}

//-------------------------------------------------------------------------------------------------
/// Process Irq
/// @param  void
/// @return TRUE: success
/// @attention
//-------------------------------------------------------------------------------------------------
MS_BOOL MDrv_GFLIP_VEC_ProcessIRQ(void)
{
    static MS_BOOL bVEReady = FALSE;
    if(s_stGflipVECConfig.eIsrType == MS_VEC_ISR_VE)
    {
        if(s_bEnable && (s_u8FrameCount == VE_CAPTURE_FRAME_INVALID_NUM))
        {
            //VEC restart, reset state
            bVEReady = MHal_GFLIP_CheckVEReady();
        }
        if(bVEReady)
        {
            //Valid Frame is 1~3 for P mode or 1~4 for interlace mode
            //But HW is count from 0, so need conversion

            s_u8FrameCount = MHal_GFLIP_GetFrameIdx();
            if(s_u8FrameCount == 0)
            {
                if(s_stGflipVECConfig.bInterlace == FALSE)
                {
                    s_u8FrameCount = s_stGflipVECConfig.u8MaxFrameNumber_P;
                }
                else
                {
                    s_u8FrameCount = s_stGflipVECConfig.u8MaxFrameNumber_I;
                }
            }
            s_u8FrameNumUpdated = s_u8FrameCount;
        }
        //printk("<0>" "F=%u\n", s_u8FrameCount);
        return TRUE;
    }
    return FALSE;
}

#endif //CONFIG_MSTAR_VE_CAPTURE_SUPPORT
//-------------------------------------------------------------------------------------------------
/// Get histogram data
/// @param pu16Histogram                \b OUT: the value of histogram
/// @return FALSE :fail
//-------------------------------------------------------------------------------------------------
MS_BOOL MDrv_GFLIP_WaitForVsync(MS_U32 u32GopIdx)
{
    MS_BOOL bRet = TRUE;
#if (GFLIP_AUTO_DETECT_BUFFER_EN == 1)
    MS_U16 u16RegDetectBuf = 0, u16RegIntMsk = 0;

    MHal_GFLIP_ReadGopReg(u32GopIdx, GFLIP_GOP_BANK_IDX_0, REG_GOP_AUTO_DETECT_BUF, &u16RegDetectBuf);
    if(u16RegDetectBuf & GOP_AUTO_DETECT_BUF_EN)
    {
        MHal_GFLIP_ReadGopReg(u32GopIdx, GFLIP_GOP_BANK_IDX_0, REG_GOP_INT, &u16RegIntMsk);
        if(u16RegIntMsk & GOP_INTMASK_VS0)
        {
            bRet = FALSE;
            return bRet;
        }
    }
#endif

    switch(u32GopIdx)
    {
        case 0:
            bGOPVsyncStatus[0]=FALSE;
            wait_event_interruptible_timeout(_gvsync_gop0_waitqueue, bGOPVsyncStatus[0],msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT));
            break;
        case 1:
            bGOPVsyncStatus[1]=FALSE;
            wait_event_interruptible_timeout(_gvsync_gop1_waitqueue, bGOPVsyncStatus[1],msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT));
            break;
        case 2:
            bGOPVsyncStatus[2]=FALSE;
            wait_event_interruptible_timeout(_gvsync_gop2_waitqueue, bGOPVsyncStatus[2],msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT));
            break;
        case 3:
            bGOPVsyncStatus[3]=FALSE;
            wait_event_interruptible_timeout(_gvsync_gop3_waitqueue, bGOPVsyncStatus[3],msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT));
            break;
        case 4:
            bGOPVsyncStatus[4]=FALSE;
            wait_event_interruptible_timeout(_gvsync_gop4_waitqueue, bGOPVsyncStatus[4],msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT));
            break;
        case 5:
            bGOPVsyncStatus[5]=FALSE;
            wait_event_interruptible_timeout(_gvsync_gop5_waitqueue, bGOPVsyncStatus[5],msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT));
            break;
        default :
            printk("[%s][%d]u32GopIdx =%td is out of case",__FUNCTION__,__LINE__, (ptrdiff_t)u32GopIdx);
            break;
    }
    return bRet;
}
#ifdef CONFIG_MP_PLATFORM_UTOPIA2K_EXPORT_SYMBOL
EXPORT_SYMBOL(MDrv_GFLIP_WaitForVsync);
#endif

MS_S64 MDrv_GFLIP_WaitForVsync_EX(MS_U32 u32GopIdx)
{
#if (GFLIP_AUTO_DETECT_BUFFER_EN == 1)
    MS_U16 u16RegDetectBuf = 0, u16RegIntMsk = 0;

    MHal_GFLIP_ReadGopReg(u32GopIdx, GFLIP_GOP_BANK_IDX_0, REG_GOP_AUTO_DETECT_BUF, &u16RegDetectBuf);
    if(u16RegDetectBuf & GOP_AUTO_DETECT_BUF_EN)
    {
        MHal_GFLIP_ReadGopReg(u32GopIdx, GFLIP_GOP_BANK_IDX_0, REG_GOP_INT, &u16RegIntMsk);
        if(u16RegIntMsk & GOP_INTMASK_VS0)
        {
            return -1;
        }
    }
#endif

    switch(u32GopIdx)
    {
        case 0:
            bGOPVsyncStatus[0]=FALSE;
            wait_event_interruptible_timeout(_gvsync_gop0_waitqueue, bGOPVsyncStatus[0],msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT));
            break;
        case 1:
            bGOPVsyncStatus[1]=FALSE;
            wait_event_interruptible_timeout(_gvsync_gop1_waitqueue, bGOPVsyncStatus[1],msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT));
            break;
        case 2:
            bGOPVsyncStatus[2]=FALSE;
            wait_event_interruptible_timeout(_gvsync_gop2_waitqueue, bGOPVsyncStatus[2],msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT));
            break;
        case 3:
            bGOPVsyncStatus[3]=FALSE;
            wait_event_interruptible_timeout(_gvsync_gop3_waitqueue, bGOPVsyncStatus[3],msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT));
            break;
        case 4:
            bGOPVsyncStatus[4]=FALSE;
            wait_event_interruptible_timeout(_gvsync_gop4_waitqueue, bGOPVsyncStatus[4],msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT));
            break;
        case 5:
            bGOPVsyncStatus[5]=FALSE;
            wait_event_interruptible_timeout(_gvsync_gop5_waitqueue, bGOPVsyncStatus[5],msecs_to_jiffies(FLIP_INTERRUPT_TIMEOUT));
            break;
        default :
            printk("[%s][%d]u32GopIdx =%td is out of case",__FUNCTION__,__LINE__, (ptrdiff_t)u32GopIdx);
            return 0;
    }
    return s_s64VsyncTs[u32GopIdx];
}
#ifdef CONFIG_MP_PLATFORM_UTOPIA2K_EXPORT_SYMBOL
EXPORT_SYMBOL(MDrv_GFLIP_WaitForVsync_EX);
#endif

MS_BOOL MDrv_GFLIP_GetVsync(MS_U32 u32GopIdx)
{
    MS_BOOL bRet = TRUE;
    MS_S64 s64VsyncTS = ktime_to_ns(ktime_get());

    switch(u32GopIdx)
    {
        case 0:
        {
            if ( waitqueue_active(&_gvsync_gop0_waitqueue))
            {
                bGOPVsyncStatus[0]=TRUE;
                wake_up_interruptible(&_gvsync_gop0_waitqueue);
            }
            break;
        }
        case 1:
        {
            if ( waitqueue_active(&_gvsync_gop1_waitqueue))
            {
                bGOPVsyncStatus[1]=TRUE;
                wake_up_interruptible(&_gvsync_gop1_waitqueue);
            }
            break;
        }
        case 2:
        {
            if ( waitqueue_active(&_gvsync_gop2_waitqueue))
            {
                bGOPVsyncStatus[2]=TRUE;
                wake_up_interruptible(&_gvsync_gop2_waitqueue);
            }
            break;
        }
        case 3:
        {
            if ( waitqueue_active(&_gvsync_gop3_waitqueue))
            {
                bGOPVsyncStatus[3]=TRUE;
                wake_up_interruptible(&_gvsync_gop3_waitqueue);
            }
            break;
        }
        case 4:
        {
            if ( waitqueue_active(&_gvsync_gop4_waitqueue))
            {
                bGOPVsyncStatus[4]=TRUE;
                wake_up_interruptible(&_gvsync_gop4_waitqueue);
            }
            break;
        }
        case 5:
        {
            if ( waitqueue_active(&_gvsync_gop5_waitqueue))
            {
                bGOPVsyncStatus[5]=TRUE;
                wake_up_interruptible(&_gvsync_gop5_waitqueue);
            }
            break;
        }
        default :
            printk("[%s][%d] u32GopIdx =%td is out of case",__FUNCTION__,__LINE__,(ptrdiff_t)u32GopIdx);
            return bRet;
    }

    s_s64VsyncTs[u32GopIdx] = s64VsyncTS;

    return bRet;
}


//-------------------------------------------------------------------------------------------
///Clear the FlipQueue by the  gop index and gwin Index
///@param u32GopIndex
///@param u32GwinIndex
///@return FALSE :fail
//---------------------------------------------------------------------------------------------
MS_BOOL _MDrv_GFLIP_ClearFlipQueue(MS_U32 u32GopIdx,MS_U32 u32GwinIdx)
{
    _MDrv_GFLIP_Mutex_Lock(u32GopIdx,TRUE);
    _u32GFlipInfoWritePtr[u32GopIdx][u32GwinIdx]=_u32GFlipInfoReadPtr[u32GopIdx][u32GwinIdx] = 0;
    //printk("\nClear the gop :%d gwin:%d\n",u32GopIdx,u32GwinIdx);
    _MDrv_GFLIP_Mutex_Lock(u32GopIdx,FALSE);
    return TRUE;
}
//-------------------------------------------------------------------------------------------
///Set GWIN info
///@param stGwinInfo
///@return FALSE :fail
//---------------------------------------------------------------------------------------------
MS_BOOL _MDrv_GFLIP_SetGwinInfo(MS_GWIN_INFO stGwinInfo)
{

    _u32GwinInfo[stGwinInfo.u8GopIdx][stGwinInfo.u8GwinIdx].u64Addr =  stGwinInfo.u64Addr;
    _u32GwinInfo[stGwinInfo.u8GopIdx][stGwinInfo.u8GwinIdx].u16X =  stGwinInfo.u16X;
    _u32GwinInfo[stGwinInfo.u8GopIdx][stGwinInfo.u8GwinIdx].u16Y =  stGwinInfo.u16Y;
    _u32GwinInfo[stGwinInfo.u8GopIdx][stGwinInfo.u8GwinIdx].u16W =  stGwinInfo.u16W;
    _u32GwinInfo[stGwinInfo.u8GopIdx][stGwinInfo.u8GwinIdx].u16H =  stGwinInfo.u16H;
    _u32GwinInfo[stGwinInfo.u8GopIdx][stGwinInfo.u8GwinIdx].u8GopIdx =  stGwinInfo.u8GopIdx;
    _u32GwinInfo[stGwinInfo.u8GopIdx][stGwinInfo.u8GwinIdx].u8GwinIdx =  stGwinInfo.u8GwinIdx;
    _u32GwinInfo[stGwinInfo.u8GopIdx][stGwinInfo.u8GwinIdx].clrType =  stGwinInfo.clrType;

     return TRUE;
}
//-------------------------------------------------------------------------------------------
///Get GWIN info
///@param u32GopIndex
///@param u32GwinIndex
///@return FALSE :MS_GWIN_INFO
//---------------------------------------------------------------------------------------------

MS_GWIN_INFO MDrv_GFLIP_GetGwinInfo(MS_U8 u8GopIdx,MS_U8 u8GwinIdx)
{
    return _u32GwinInfo[u8GopIdx][u8GwinIdx];
}

void SC_Save_Bank(int bkidx, unsigned short bank[])
{
    int i;
    for(i=1;i<128;i++){
        bank[i]=SC_R2BYTE(i, (unsigned short)bkidx);
    }
}
void SC_Restore_Bank(int bkidx, unsigned short bank[])
{
    int i;
    for(i=1;i<128;i++){
        SC_W2BYTE(i, (unsigned short)bkidx, bank[i]);
    }
}

int MDrv_GFLIP_Suspend(void)
{
    int i,j;
    int GopIdx, GopBks;
    MS_U32 u32RegForm = 0;
    MS_U8 u8GOPIndex = 0 ;
    MS_U32 u32PalIndex = 0 ;
    MS_U32 u32PaletteForm = 0;
    MS_U16 u16ClkGated = 0;
    MS_U16 u16tmp = 0;
    MS_U16 u16BankSel = 0;

    for(i=0;i<GFLIP_REG_BANKS;i++){
        if (i < 12)
        {
            GopIdx=i/GFLIP_GOP_BANKOFFSET;
            GopBks=i%GFLIP_GOP_BANKOFFSET;
        }
#if (MAX_GOP_SUPPORT >= 5)
        else if (i == 12)
        {
            //DWIN
            continue;
        }
        else if (i == 13)
        {
            //MIXER
            continue;
        }
        else
        {
            GopIdx=(i-2)/GFLIP_GOP_BANKOFFSET;
            GopBks=(i-2)%GFLIP_GOP_BANKOFFSET;
        }
#endif
        for(j=0;j<GFLIP_REG16_NUM_PER_BANK;j++){
            MHal_GFLIP_ReadGopReg(GopIdx,
                GopBks, j, &(_gflip_regs_save.BankReg[i][j]));
        }
    }

    //Store palette
    for(u8GOPIndex = 0 ;u8GOPIndex <MAX_GOP_SUPPORT;u8GOPIndex ++)
    {
        u32PaletteForm = _MDrv_GFLIP_GetRegForm(u8GOPIndex);
        if((u32PaletteForm & E_GOP_PAL_SIZE_MASK) != E_GOP_PAL_SIZE_NONE)
        {

            MHal_GFLIP_ReadGopReg(u8GOPIndex,0,REG_GOP_4G_OLDADDR,&u16ClkGated); //Get GOP clk dynamical gated
            if((u16ClkGated & (BIT(0) | BIT(1))))
            {
                //Disable clk gated when R/W palette
                MHal_GFLIP_WriteGopReg(u8GOPIndex,0 ,REG_GOP_4G_OLDADDR, 0, (BIT(1) | BIT(0))); //enable GOP clk dynamical gated
            }
            for(u32PalIndex = 0 ;u32PalIndex < PALETTE_SIZE;u32PalIndex++)
            {
                //MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_BANK_SEL, BIT(9), BIT(9));
                MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_4G_PALCTRL, 0, (BIT(12)|BIT(13)) );    // Set RIU access
                MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_4G_PALCTRL, u32PalIndex , BMASK(7:0));
                MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_4G_PALCTRL, BIT(9), BIT(9));   // Enable pallete read
                MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_4G_PALCTRL, 0, BIT(9));   // Clear pallete read
                //MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_BAK_SEL, 0, BIT(9));

                MHal_GFLIP_ReadGopReg(u8GOPIndex,0, REG_GOP_4G_PALDATA_L, &u16tmp);
                _gu32PaletteSave[u8GOPIndex][u32PalIndex] = u16tmp;
                MHal_GFLIP_ReadGopReg(u8GOPIndex,0, REG_GOP_4G_PALDATA_H, &u16tmp);
                _gu32PaletteSave[u8GOPIndex][u32PalIndex] |= (u16tmp  << 16);
            }
            //Disable clk gated when R/W palette
            MHal_GFLIP_WriteGopReg(u8GOPIndex,0 ,REG_GOP_4G_OLDADDR, u16ClkGated , (BIT(1) | BIT(0))); //enable GOP clk dynamical gated
        }
    }

    _gflip_regs_save.CKG_GopReg[0]=CKG_REG(GOP_GOPCLK);
    _gflip_regs_save.CKG_GopReg[1]=CKG_REG(GOP_GOP2CLK);
    _gflip_regs_save.CKG_GopReg[2]=CKG_REG(GOP_GOP3CLK);
    _gflip_regs_save.CKG_GopReg[6]=CKG_REG(GOP_SRAMCLK);        /*Palette SRAM*/

#if (GOP_SCALING_DOWN == 1)
    _gflip_regs_save.CKG_GopReg[7]=CKG_REG(CKG_GOP_MG);
#endif

    u16BankSel = GOP_SC_GOPBLENDING;
#ifdef GOP_SC_VOP2BLENDING_BK
    u16BankSel = GOP_SC_VOP2BLENDING_BK;
#endif
    _gflip_regs_save.SC_OPBlending[0] = SC_R2BYTE(GOP_SC_GOPBLENDING_L, u16BankSel);
    _gflip_regs_save.SC_OPBlending[1] = SC_R2BYTE(GOP_SC_GOPBLENDING_H, u16BankSel);
#ifdef GOP_SC_GOPBLENDING_EX
    _gflip_regs_save.SC_OPBlending[5] = SC_R2BYTE(GOP_SC_GOPBLENDING_EX, u16BankSel);
#endif
#ifdef GOP_SC1_GOPEN
    _gflip_regs_save.SC_OPBlending[3] = SC_R2BYTE(GOP_SC_IP2SC, GOP_SC1_GOPEN);
    _gflip_regs_save.SC_OPBlending[4] = SC_R2BYTE(GOP_SC_GOPBLENDING_H, GOP_SC1_GOPEN);
#endif
    _gflip_regs_save.SC_OPBlending[2] = SC_R2BYTE(GOP_SC_OCMIXER_ALPHA, GOP_SC_OCBANKSEL);

    SC_Save_Bank(0x00,_gflip_regs_save.SC_save[0]);
    SC_Save_Bank(0x0f,_gflip_regs_save.SC_save[1]);
    SC_Save_Bank(0x10,_gflip_regs_save.SC_save[2]);

#ifdef GOP_MIU_REG
    _gflip_regs_save.MIU_GopReg[0] = MIU0_REG(GOP_MIU_REG);
    _gflip_regs_save.MIU_GopReg[1] = MIU1_REG(GOP_MIU_REG);
    _gflip_regs_save.MIU_GopReg[2] = MIU2_REG(GOP_MIU_REG);
#endif

#ifdef GOP_MIU_IN_SC
    _gflip_regs_save.MIU_SC_GopReg[0] = SC_R2BYTE(GOP_SC_MIUSEL_HW_SW, GOP_SC_MIUBANKSEL);
    _gflip_regs_save.MIU_SC_GopReg[1] = SC_R2BYTE(GOP_SC_MIUSEL_L, GOP_SC_MIUBANKSEL);
#ifdef GOP_SC_MIUSEL_H
    _gflip_regs_save.MIU_SC_GopReg[2] = SC_R2BYTE(GOP_SC_MIUSEL_H, GOP_SC_MIUBANKSEL);
#endif //GOP_SC_MIUSEL_H
#endif //GOP_MIU_IN_SC
#ifdef GS_REG_RESTORE_FUNCTION
    _gflip_regs_save.CKG_GopReg[3]=CKG_REG(GOP_SCLCLK);
    _gflip_regs_save.GS_GopReg=GS_REG(REG_GS_VSP_SRAM);
    _gflip_regs_save.CKG_GopReg[4]=CKG_REG(GOP_GOP4CLK);
#ifdef GOP_LB_SRAMCLK
    _gflip_regs_save.CKG_GopReg[5]=CKG_REG(GOP_LB_SRAMCLK);
#endif
#endif


#ifdef SUPPORT_GOP_DUALRATE
    _gflip_regs_save.SC_GopBlending[0] = SC_R2BYTE(GOP_SC_DUALRATE_BLENDING0, GOP_SC_DUALRATE_BLENDING_BNK);
    _gflip_regs_save.SC_GopBlending[1] = SC_R2BYTE(GOP_SC_DUALRATE_BLENDING1, GOP_SC_DUALRATE_BLENDING_BNK);
    _gflip_regs_save.SC_GopBlending[2] = SC_R2BYTE(GOP_SC_DUALRATE_BLENDING2, GOP_SC_DUALRATE_BLENDING_BNK);

    _gflip_regs_save.SC_GopHsyncShift = SC_R2BYTE(GOP_SC_HSYNC_SHIFT, GOP_SC_HSYNC_SHIFT_BNK);
#endif

#ifdef NEED_MANUAL_CROP
    for (i=0; i<MANUAL_CROP_SETS; i++)
    {
        _gflip_regs_save.u16GopManualCrop_PreCalDone[i] = REG_GOP_CROP_PRECALDONE(i);
        _gflip_regs_save.u16GopManualCrop_Hstart[i] = REG_GOP_CROP_ORIHSTART(i);
        _gflip_regs_save.u16GopManualCrop_HEnd[i]= REG_GOP_CROP_ORIHEND(i);
    }
#endif

#ifdef SUPPORT_GOP_AFBC
    _gflip_regs_save.u16AfbcRegClk = CKG_REG(GOP_AFBC_CLK);
#ifdef REG_AFBC_TRIGGER
    _gflip_regs_save.u16AfbcTrigger = REG_AFBC_TRIGGER;
#endif
    for (i=0; i<GOP_AFBC_CORE_NUMBER; i++)
    {
        _gflip_regs_save.u16AfbcRegEn[i] = REG_AFBC_CORE_EN(i);
        _gflip_regs_save.u16AfbcRegAddrL[i] = REG_AFBC_ADDR_L(i);
        _gflip_regs_save.u16AfbcRegAddrH[i] = REG_AFBC_ADDR_H(i);
        _gflip_regs_save.u16AfbcRegAddrMSB[i] = REG_AFBC_ADDR_MSB(i);
        _gflip_regs_save.u16AfbcRegFmt[i] = REG_AFBC_FMT(i);
        _gflip_regs_save.u16AfbcRegWidth[i] = REG_AFBC_WIDTH(i);
        _gflip_regs_save.u16AfbcRegHeight[i] = REG_AFBC_HEIGHT(i);
        _gflip_regs_save.u16AfbcRegCropX[i] = REG_AFBC_CROP_X(i);
        _gflip_regs_save.u16AfbcRegCropY[i] = REG_AFBC_CROP_Y(i);
        _gflip_regs_save.u16AfbcRegCropHSize[i] = REG_AFBC_CROP_HSIZE(i);
        _gflip_regs_save.u16AfbcRegCropVSize[i] = REG_AFBC_CROP_VSIZE(i);
    }
#ifdef REG_AFBC_MIU
    _gflip_regs_save.u16AfbcRegMiu = REG_AFBC_MIU;
#endif
#endif

#ifdef SUPPORT_GOP_MIXER
    MHal_GFLIP_ReadGopReg(GFLIP_GOP_IDX_MIXER, GFLIP_GOP_BANK_IDX_0, GOP_HDR_BYPASS, &_gflip_regs_save.u16HDRBypass);
    for (i=0; i < GOP_MIXER_NUMBER; i++)
    {
        MHal_GFLIP_ReadGopReg(GFLIP_GOP_IDX_MIXER, GFLIP_GOP_BANK_IDX_0, MIXER_SWRST(i), &_gflip_regs_save.u16MixerSWRst[i]);
        MHal_GFLIP_ReadGopReg(GFLIP_GOP_IDX_MIXER, GFLIP_GOP_BANK_IDX_0, MIXER_VALID_NUM(i), &_gflip_regs_save.u16MixerValid[i]);
        MHal_GFLIP_ReadGopReg(GFLIP_GOP_IDX_MIXER, GFLIP_GOP_BANK_IDX_0, MIXER_SRC_SEL(i), &_gflip_regs_save.u16MixerSrcSel[i]);
    }
#endif

#ifdef GEN_GOP_XCIP_TIMING
    _gflip_regs_save.u16GenGOPXCIPTiming = SC_IP_REG(REG_GOP_XCIP_TIMING);
#endif

#ifdef DRAM_RBLK_MSB_MOVE_TO_SC
    for(u8GOPIndex = 0; u8GOPIndex < MAX_GOP_SUPPORT; u8GOPIndex++)
    {
        _gflip_regs_save.u16GopDramRblkMsb[u8GOPIndex] = SC_MSB_REG(REG_GOP_DRAM_RBLK_STR_EX(u8GOPIndex)) & GOP_DRAM_RBLK_STR_EX_MASK;
    }
#endif

#ifdef SC_BWP_SETTING
    for(u8GOPIndex = 0; u8GOPIndex < MAX_GOP_SUPPORT; u8GOPIndex++)
    {
        _gflip_regs_save.u16GopBwpTrgSlow[u8GOPIndex] = SC_BWP_REG(SC_BWP_LV_TRG_SLOW(u8GOPIndex)) & SC_BWP_LV_MASK;
        _gflip_regs_save.u16GopBwpRlsSlow[u8GOPIndex] = SC_BWP_REG(SC_BWP_LV_RLS_SLOW(u8GOPIndex)) & SC_BWP_LV_MASK;
        _gflip_regs_save.u16GopBwpTrgStop[u8GOPIndex] = SC_BWP_REG(SC_BWP_LV_TRG_STOP(u8GOPIndex)) & SC_BWP_LV_MASK;
        _gflip_regs_save.u16GopBwpRlsStop[u8GOPIndex] = SC_BWP_REG(SC_BWP_LV_RLS_STOP(u8GOPIndex)) & SC_BWP_LV_MASK;
    }
#endif

    return 0;
}

int MDrv_GFLIP_Resume(void)
{
    int i,j;
    int GopIdx, GopBks;
    unsigned short GWinEnable[MAX_GOP_SUPPORT]={0};
    MS_U32 u32GOPBitMask;
    MS_U16 u16tmp;
    MS_U32 u32RegForm = 0;
    MS_U8 u8GOPIndex = 0 ;
    MS_U32 u32PalIndex = 0 ;
    MS_U32 u32PaletteForm = 0;
    MS_U16 u16ClkGated = 0;
    MS_U16 u16BankSel = 0;
#ifdef GOP_SC_MIU_ARBITOR_HW_BUG_PATCH
    MS_U16 u16mask0 = 0 , u16mask1 = 0;
#ifdef MIU2_ARB_REG
    MS_U16 u16mask2 = 0;
#endif
#endif

    CKG_REG(GOP_GOPCLK)=_gflip_regs_save.CKG_GopReg[0];
    CKG_REG(GOP_GOP2CLK)=_gflip_regs_save.CKG_GopReg[1];
    CKG_REG(GOP_GOP3CLK)=_gflip_regs_save.CKG_GopReg[2];
    CKG_REG(GOP_SRAMCLK)=_gflip_regs_save.CKG_GopReg[6];        /*Palette SRAM*/

#if (GOP_SCALING_DOWN == 1)
    CKG_REG(CKG_GOP_MG) = _gflip_regs_save.CKG_GopReg[7];
#endif

    u16BankSel = GOP_SC_GOPBLENDING;
#ifdef GOP_SC_VOP2BLENDING_BK
    u16BankSel = GOP_SC_VOP2BLENDING_BK;
#endif
    SC_W2BYTE(GOP_SC_GOPBLENDING_L, u16BankSel, _gflip_regs_save.SC_OPBlending[0]);
    SC_W2BYTE(GOP_SC_GOPBLENDING_H, u16BankSel, _gflip_regs_save.SC_OPBlending[1]);
#ifdef GOP_SC_GOPBLENDING_EX
    SC_W2BYTE(GOP_SC_GOPBLENDING_EX, u16BankSel, _gflip_regs_save.SC_OPBlending[5]);
#endif
#ifdef GOP_SC1_GOPEN
    SC_W2BYTE(GOP_SC_IP2SC, GOP_SC1_GOPEN, _gflip_regs_save.SC_OPBlending[3]);
    SC_W2BYTE(GOP_SC_GOPBLENDING_H, GOP_SC1_GOPEN, _gflip_regs_save.SC_OPBlending[4]);
#endif
    SC_W2BYTE(GOP_SC_OCMIXER_ALPHA, GOP_SC_OCBANKSEL, _gflip_regs_save.SC_OPBlending[2]);

    SC_Restore_Bank(0x00,_gflip_regs_save.SC_save[0]);
    SC_Restore_Bank(0x0f,_gflip_regs_save.SC_save[1]);
    SC_Restore_Bank(0x10,_gflip_regs_save.SC_save[2]);

#ifdef GOP_MIU_REG
    MIU0_REG(GOP_MIU_REG) = _gflip_regs_save.MIU_GopReg[0];
    MIU1_REG(GOP_MIU_REG) = _gflip_regs_save.MIU_GopReg[1];
    MIU2_REG(GOP_MIU_REG) = _gflip_regs_save.MIU_GopReg[2];
#endif

#ifdef GOP_MIU_IN_SC
#ifdef GOP_SC_MIU_ARBITOR_HW_BUG_PATCH
    u16mask0 = MIU0_ARB_REG(GOP_SC_MIU_ARBITOR);
    u16mask1 = MIU1_ARB_REG(GOP_SC_MIU_ARBITOR);
    MIU0_ARB_REG(GOP_SC_MIU_ARBITOR) = u16mask0 | GOP_MIU_CLIENT;
    MIU1_ARB_REG(GOP_SC_MIU_ARBITOR) = u16mask1 | GOP_MIU_CLIENT;
#ifdef MIU2_ARB_REG
    u16mask2 = MIU2_ARB_REG(GOP_SC_MIU_ARBITOR);
    MIU2_ARB_REG(GOP_SC_MIU_ARBITOR) = u16mask2 | GOP_MIU_CLIENT;
#endif
#endif
    u16tmp = (SC_R2BYTE(GOP_SC_MIUSEL_HW_SW, GOP_SC_MIUBANKSEL)&(~GOP_MIU_CLIENT))|(_gflip_regs_save.MIU_SC_GopReg[0]&GOP_MIU_CLIENT);
    SC_W2BYTE(GOP_SC_MIUSEL_HW_SW, GOP_SC_MIUBANKSEL, u16tmp);
    u16tmp = (SC_R2BYTE(GOP_SC_MIUSEL_L, GOP_SC_MIUBANKSEL)&(~GOP_MIU_CLIENT))|(_gflip_regs_save.MIU_SC_GopReg[1]&GOP_MIU_CLIENT);
    SC_W2BYTE(GOP_SC_MIUSEL_L, GOP_SC_MIUBANKSEL, u16tmp);
#ifdef GOP_SC_MIUSEL_H
    u16tmp = (SC_R2BYTE(GOP_SC_MIUSEL_H, GOP_SC_MIUBANKSEL)&(~GOP_MIU_CLIENT))|(_gflip_regs_save.MIU_SC_GopReg[2]&GOP_MIU_CLIENT);
    SC_W2BYTE(GOP_SC_MIUSEL_H, GOP_SC_MIUBANKSEL, u16tmp);
#endif //GOP_SC_MIUSEL_H
#ifdef GOP_SC_MIU_ARBITOR_HW_BUG_PATCH
	MIU0_ARB_REG(GOP_SC_MIU_ARBITOR) = u16mask0;
	MIU1_ARB_REG(GOP_SC_MIU_ARBITOR) = u16mask1;
#ifdef MIU2_ARB_REG
    MIU2_ARB_REG(GOP_SC_MIU_ARBITOR) = u16mask2;
#endif
#endif
#endif //GOP_MIU_IN_SC

#ifdef GS_REG_RESTORE_FUNCTION
        CKG_REG(GOP_SCLCLK)=_gflip_regs_save.CKG_GopReg[3];
        GS_REG(REG_GS_VSP_SRAM)=_gflip_regs_save.GS_GopReg;
        CKG_REG(GOP_GOP4CLK)=_gflip_regs_save.CKG_GopReg[4];
#ifdef GOP_LB_SRAMCLK
        CKG_REG(GOP_LB_SRAMCLK)=_gflip_regs_save.CKG_GopReg[5];
#endif
#endif

#ifdef SUPPORT_GOP_DUALRATE
    SC_W2BYTE(GOP_SC_DUALRATE_BLENDING0, GOP_SC_DUALRATE_BLENDING_BNK, _gflip_regs_save.SC_GopBlending[0]);
    SC_W2BYTE(GOP_SC_DUALRATE_BLENDING1, GOP_SC_DUALRATE_BLENDING_BNK, _gflip_regs_save.SC_GopBlending[1]);
    SC_W2BYTE(GOP_SC_DUALRATE_BLENDING2, GOP_SC_DUALRATE_BLENDING_BNK, _gflip_regs_save.SC_GopBlending[2]);

    SC_W2BYTE(GOP_SC_HSYNC_SHIFT, GOP_SC_HSYNC_SHIFT_BNK, _gflip_regs_save.SC_GopHsyncShift);
#endif

#ifdef SUPPORT_GOP_AFBC
    CKG_REG(GOP_AFBC_CLK) = _gflip_regs_save.u16AfbcRegClk;
#ifdef REG_AFBC_TRIGGER
    REG_AFBC_TRIGGER = _gflip_regs_save.u16AfbcTrigger;
#endif
#ifdef REG_AFBC_MIU
    REG_AFBC_MIU = _gflip_regs_save.u16AfbcRegMiu;
#endif
    for (i=0; i<GOP_AFBC_CORE_NUMBER; i++)
    {
        if(_gflip_regs_save.u16AfbcRegEn[i] & AFBC_CORE_EN)
        {
            REG_AFBC_ADDR_L(i) = _gflip_regs_save.u16AfbcRegAddrL[i];
            REG_AFBC_ADDR_H(i) = _gflip_regs_save.u16AfbcRegAddrH[i];
            REG_AFBC_ADDR_MSB(i) = _gflip_regs_save.u16AfbcRegAddrMSB[i];
            REG_AFBC_FMT(i) = _gflip_regs_save.u16AfbcRegFmt[i];
            REG_AFBC_WIDTH(i) = _gflip_regs_save.u16AfbcRegWidth[i];
            REG_AFBC_HEIGHT(i) = _gflip_regs_save.u16AfbcRegHeight[i];
            REG_AFBC_CROP_X(i) = _gflip_regs_save.u16AfbcRegCropX[i];
            REG_AFBC_CROP_Y(i) = _gflip_regs_save.u16AfbcRegCropY[i];
            REG_AFBC_CROP_HSIZE(i) = _gflip_regs_save.u16AfbcRegCropHSize[i];
            REG_AFBC_CROP_VSIZE(i) = _gflip_regs_save.u16AfbcRegCropVSize[i];

            REG_AFBC_CORE_EN(i) = _gflip_regs_save.u16AfbcRegEn[i];
        }
    }
    if((REG_AFBC_CORE_EN(0) & AFBC_CORE_EN) == 0)
    {
        _gflip_regs_save.BankReg[0][REG_GOP_MIU_SEL] &= (~GOP_AFBC_EN);
    }
#endif

    for(i=0;i<GFLIP_REG_BANKS;i++){
        if (i < 12)
        {
            GopIdx=i/GFLIP_GOP_BANKOFFSET;
            GopBks=i%GFLIP_GOP_BANKOFFSET;
        }
#if (MAX_GOP_SUPPORT >= 5)
        else if (i == 12)
        {
            //DWIN
            continue;
        }
        else if (i == 13)
        {
            //MIXER
            continue;
        }
        else if (i > 13)
        {
            GopIdx=(i-2)/GFLIP_GOP_BANKOFFSET;
            GopBks=(i-2)%GFLIP_GOP_BANKOFFSET;
        }
#endif
        if((GopIdx < MAX_GOP_SUPPORT) && (GopBks==0)){//reset gop
            if( (i+1) < GFLIP_REG_BANKS)
            {
                GWinEnable[GopIdx]=_gflip_regs_save.BankReg[i+1][0];
                _gflip_regs_save.BankReg[i+1][0] &= 0xFFFE;
            }
        }

        //Dwin doesn't need resume
        for(j=0;j<GFLIP_REG16_NUM_PER_BANK;j++){
            MHal_GFLIP_WriteGopReg(GopIdx,
                GopBks, j, (_gflip_regs_save.BankReg[i][j]),0xFFFF);
        }
    }

    //Restore palette
    for(u8GOPIndex = 0 ;u8GOPIndex <MAX_GOP_SUPPORT;u8GOPIndex ++)
    {
        u32PaletteForm = _MDrv_GFLIP_GetRegForm(u8GOPIndex);
        if((u32PaletteForm & E_GOP_PAL_SIZE_MASK) != E_GOP_PAL_SIZE_NONE)
        {
            MHal_GFLIP_ReadGopReg(u8GOPIndex,0,REG_GOP_4G_OLDADDR,&u16ClkGated); //Get GOP clk dynamical gated
            MHal_GFLIP_ReadGopReg(u8GOPIndex,0,REG_GOP_4G_PALCTRL,&u16tmp);
            if((u16ClkGated & (BIT(0) | BIT(1))))
            {
                //Disable clk gated when R/W palette
                MHal_GFLIP_WriteGopReg(u8GOPIndex,0 ,REG_GOP_4G_OLDADDR, 0, (BIT(1) | BIT(0))); //enable GOP clk dynamical gated
            }
            for(u32PalIndex = 0 ;u32PalIndex < PALETTE_SIZE;u32PalIndex++)
            {
                //MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_BANK_SEL, BIT(9), BIT(9));
                MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_4G_PALCTRL, 0, (BIT(12)|BIT(13)) );    // Set RIU access
                MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_4G_PALDATA_L,  (_gu32PaletteSave[u8GOPIndex][u32PalIndex] & 0xffff) , BMASK(15:0));
                MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_4G_PALDATA_H,  ((_gu32PaletteSave[u8GOPIndex][u32PalIndex] & 0xffff0000) >> 16) , BMASK(15:0));
                MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_4G_PALCTRL, u32PalIndex , BMASK(7:0));
                MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_4G_PALCTRL, BIT(8), BIT(8));   // Enable pallete write
                MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_4G_PALCTRL, 0, BIT(8));   // Clear pallete write
                //MHal_GFLIP_WriteGopReg(u8GOPIndex,0, REG_GOP_BAK_SEL, 0, BIT(9));
            }
            //Disable clk gated when R/W palette
            MHal_GFLIP_WriteGopReg(u8GOPIndex,0 ,REG_GOP_4G_PALCTRL, u16tmp , BMASK(15:0));
            MHal_GFLIP_WriteGopReg(u8GOPIndex,0 ,REG_GOP_4G_OLDADDR, u16ClkGated , (BIT(1) | BIT(0))); //enable GOP clk dynamical gated
        }
    }

#ifdef NEED_MANUAL_CROP
    for (i=0; i<MANUAL_CROP_SETS; i++)
    {
        REG_GOP_CROP_PRECALDONE(i) = _gflip_regs_save.u16GopManualCrop_PreCalDone[i];
        REG_GOP_CROP_ORIHSTART(i) = _gflip_regs_save.u16GopManualCrop_Hstart[i];
        REG_GOP_CROP_ORIHEND(i) = _gflip_regs_save.u16GopManualCrop_HEnd[i];
    }
#endif

#ifdef SUPPORT_GOP_MIXER
    MHal_GFLIP_WriteGopReg(GFLIP_GOP_IDX_MIXER, GFLIP_GOP_BANK_IDX_0, GOP_HDR_BYPASS, _gflip_regs_save.u16HDRBypass, GFLIP_REG_WORD_MASK);
    for (i=0; i < GOP_MIXER_NUMBER; i++)
    {
        MHal_GFLIP_WriteGopReg(GFLIP_GOP_IDX_MIXER, GFLIP_GOP_BANK_IDX_0, MIXER_SWRST(i), _gflip_regs_save.u16MixerSWRst[i], GFLIP_REG_WORD_MASK);
        MHal_GFLIP_WriteGopReg(GFLIP_GOP_IDX_MIXER, GFLIP_GOP_BANK_IDX_0, MIXER_VALID_NUM(i), _gflip_regs_save.u16MixerValid[i], GFLIP_REG_WORD_MASK);
        MHal_GFLIP_WriteGopReg(GFLIP_GOP_IDX_MIXER, GFLIP_GOP_BANK_IDX_0, MIXER_SRC_SEL(i), _gflip_regs_save.u16MixerSrcSel[i], GFLIP_REG_WORD_MASK);
    }
    MHal_GFLIP_WriteGopReg(GFLIP_GOP_IDX_MIXER, GFLIP_GOP_BANK_IDX_0, REG_GOP_BANK_SEL_EX, GOP_MIXER_WR_ACK, GOP_MIXER_WR_ACK); //gop mixer write ack
    MHal_GFLIP_WriteGopReg(GFLIP_GOP_IDX_2G, GFLIP_GOP_BANK_IDX_0, GOP_SC0_ZORDER_REG, GOP_SC0_ZORDER_TRIGGER, GOP_SC0_ZORDER_TRIGGER);
#endif

#ifdef GEN_GOP_XCIP_TIMING
    SC_IP_REG(REG_GOP_XCIP_TIMING) = _gflip_regs_save.u16GenGOPXCIPTiming;
#endif

#ifdef DRAM_RBLK_MSB_MOVE_TO_SC
    for(u8GOPIndex = 0; u8GOPIndex < MAX_GOP_SUPPORT; u8GOPIndex++)
    {
        SC_MSB_REG(REG_GOP_DRAM_RBLK_STR_EX(u8GOPIndex)) = (SC_MSB_REG(REG_GOP_DRAM_RBLK_STR_EX(u8GOPIndex)) & ~GOP_DRAM_RBLK_STR_EX_MASK) | _gflip_regs_save.u16GopDramRblkMsb[u8GOPIndex];
    }
#endif

#ifdef SC_BWP_SETTING
    for(u8GOPIndex = 0; u8GOPIndex < MAX_GOP_SUPPORT; u8GOPIndex++)
    {
        SC_BWP_REG(SC_BWP_LV_TRG_SLOW(u8GOPIndex)) = (SC_BWP_REG(SC_BWP_LV_TRG_SLOW(u8GOPIndex)) & ~SC_BWP_LV_MASK) | _gflip_regs_save.u16GopBwpTrgSlow[u8GOPIndex];
        SC_BWP_REG(SC_BWP_LV_RLS_SLOW(u8GOPIndex)) = (SC_BWP_REG(SC_BWP_LV_RLS_SLOW(u8GOPIndex)) & ~SC_BWP_LV_MASK) | _gflip_regs_save.u16GopBwpRlsSlow[u8GOPIndex];
        SC_BWP_REG(SC_BWP_LV_TRG_STOP(u8GOPIndex)) = (SC_BWP_REG(SC_BWP_LV_TRG_STOP(u8GOPIndex)) & ~SC_BWP_LV_MASK) | _gflip_regs_save.u16GopBwpTrgStop[u8GOPIndex];
        SC_BWP_REG(SC_BWP_LV_RLS_STOP(u8GOPIndex)) = (SC_BWP_REG(SC_BWP_LV_RLS_STOP(u8GOPIndex)) & ~SC_BWP_LV_MASK) | _gflip_regs_save.u16GopBwpRlsStop[u8GOPIndex];
    }
#endif

    u32GOPBitMask = 0;
    for(i = 0; i < MAX_GOP_SUPPORT; i++){//restore gwin enable
        MHal_GFLIP_WriteGopReg(i,
        GFLIP_GOP_BANK_IDX_0, REG_GOP4G_CTRL0, GOP_RST_MASK,GOP_RST_MASK);

        if ((i >= 0) && (i <= 3))
        {
            MHal_GFLIP_WriteGopReg(i,
            GFLIP_GOP_BANK_IDX_0, REG_GOP4G_CTRL0, _gflip_regs_save.BankReg[3*i][0], GOP_RST_MASK);
        }
        else if (i==4)
        {
            MHal_GFLIP_WriteGopReg(i,
            GFLIP_GOP_BANK_IDX_0, REG_GOP4G_CTRL0, _gflip_regs_save.BankReg[14][0], GOP_RST_MASK);
        }
        if (GWinEnable[i] > 0)
        {
            // delay 1ms
            // because of the GWIN clock is not gated, GOP clock is gated
            // if we enable GWIN without delay, the GWIN would fetch some garbage
            mdelay(1);
        }
        MHal_GFLIP_WriteGopReg(i,
            GFLIP_GOP_BANK_IDX_1, REG_GOP4G_GWIN_CTRL0(0), GWinEnable[i],0xFFFF);
        _gflip_regs_save.BankReg[(i*3)+1][0] = GWinEnable[i]; //restore gwin enable in _gflip_regs_save
        u32GOPBitMask |= (1 << i);
    }
#ifdef SUPPORT_GOP_OPMUX_TRIGGER
    // mux trigger need to effective before enable GWIN
    MHal_GFLIP_WriteGopReg(0x0, 0x0, REG_GOP_MUX_TRIGGER, REG_GOP_MUX_TRIGGER_BIT, REG_GOP_MUX_TRIGGER_BIT);
#endif
    MHal_GFLIP_Fire(u32GOPBitMask);
    return 0;
}

MS_BOOL MDrv_GFLIP_Register_CB(MS_U8 u8Type,void *fpCB)
{
    if(fpCB == NULL)
    {
        printk("[%s][%d]  fpCB is NULL.\n",__func__,__LINE__);
        return FALSE;
    }

    if(u8Type == CB_MAPI_CFD_GOP_PREPROCESS)
    {
        _fpGflip_CB_CFD_GOP_PreProcess = (FP_CFD_GOP_PreProcess)fpCB;
    }
    else if(u8Type == CB_MAPI_CFD_GOP_PRESDR)
    {
        _fpGflip_CB_CFD_GOP_PreSDR = (FP_CFD_GOP_PreSDR)fpCB;
    }
    else if(u8Type == CB_MApi_DebugCFD_GOP_TestMode)
    {
        _fpGflip_CB_CFD_GOP_TestMode = (FP_CFD_GOP_TestMode)fpCB;
    }
    else
    {
        printk("[%s][%d] Not support this type.\n",__func__,__LINE__);
        return FALSE;
    }

    return TRUE;
}

//CFD function
void _MDrv_GFLIP_Set_709Gamut_Colorimetry(STU_CFD_COLORIMETRY *pstu_Cfd_source_ColorMetry)
{

    //order R->G->B
    //BT709
    //data *0.00002 0xC350 = 1

    pstu_Cfd_source_ColorMetry->u16Display_Primaries_x[0] = 0x7D00; //0.64
    pstu_Cfd_source_ColorMetry->u16Display_Primaries_x[1] = 0x3A98; //0.3
    pstu_Cfd_source_ColorMetry->u16Display_Primaries_x[2] = 0x1D4C; //0.15

    pstu_Cfd_source_ColorMetry->u16Display_Primaries_y[0] = 0x4047; //0.33
    pstu_Cfd_source_ColorMetry->u16Display_Primaries_y[1] = 0x7530; //0.6
    pstu_Cfd_source_ColorMetry->u16Display_Primaries_y[2] = 0x0BB8; //0.06

    pstu_Cfd_source_ColorMetry->u16White_point_x          = 0x3D13; //0.3127
    pstu_Cfd_source_ColorMetry->u16White_point_y          = 0x4042; //0.3290

}

void _MDrv_GFLIP_Set_InitValue_Main_Control_GOP(STU_CFDAPI_MAIN_CONTROL_GOP *pstu_CfdAPI_MainControl_Param_lite)
{

    //E_CFD_MC_SOURCE
    pstu_CfdAPI_MainControl_Param_lite->u8Input_Source  = E_CFD_INPUT_SOURCE_GENERAL;

    //E_CFD_CFIO
    pstu_CfdAPI_MainControl_Param_lite->u8Input_Format  = E_GFLIP_CFD_CFIO_RGB_BT709;
    //pstu_CfdAPI_MainControl_Param_lite->u8Input_Format  = E_CFD_CFIO_RGB_NOTSPECIFIED;
    //pstu_CfdAPI_MainControl_Param_lite->u8Input_Format = E_CFD_CFIO_YUV_BT709;

    //E_CFD_MC_FORMAT
    pstu_CfdAPI_MainControl_Param_lite->u8Input_DataFormat  = E_GFLIP_CFD_MC_FORMAT_RGB;
    //pstu_CfdAPI_MainControl_Param_lite->u8Input_DataFormat = E_CFD_MC_FORMAT_YUV444;

    //E_CFD_CFIO_RANGE
    //0:limit 1:full
    pstu_CfdAPI_MainControl_Param_lite->u8Input_IsFullRange = E_GFLIP_CFD_CFIO_RANGE_FULL;

    //E_CFIO_HDR_STATUS
    //0:SDR 1:HDR1 2:HDR2
    pstu_CfdAPI_MainControl_Param_lite->u8Input_HDRMode     = E_CFIO_MODE_SDR;

    //assign by E_CFD_CFIO_CP
    pstu_CfdAPI_MainControl_Param_lite->u8Input_ext_Colour_primaries         = E_CFD_CFIO_CP_BT709_SRGB_SYCC;

    //assign by E_CFD_CFIO_TR
    pstu_CfdAPI_MainControl_Param_lite->u8Input_ext_Transfer_Characteristics = E_CFD_CFIO_TR_BT709;

    //assign by E_CFD_CFIO_MC
    pstu_CfdAPI_MainControl_Param_lite->u8Input_ext_Matrix_Coeffs            = E_CFD_CFIO_MC_BT709_XVYCC709;

    _MDrv_GFLIP_Set_709Gamut_Colorimetry(&(pstu_CfdAPI_MainControl_Param_lite->stu_Cfd_source_ColorMetry));

    //E_CFD_MC_SOURCE
    pstu_CfdAPI_MainControl_Param_lite->u8Output_Source = E_CFD_OUTPUT_SOURCE_GENERAL;

    //E_CFD_CFIO
    //pstu_CfdAPI_MainControl_Param_lite->u8Output_Format = E_CFD_CFIO_RGB_NOTSPECIFIED;
    pstu_CfdAPI_MainControl_Param_lite->u8Output_Format = E_GFLIP_CFD_CFIO_RGB_BT709;

    //E_CFD_MC_FORMAT
    pstu_CfdAPI_MainControl_Param_lite->u8Output_DataFormat = E_GFLIP_CFD_MC_FORMAT_RGB;
    //pstu_CfdAPI_MainControl_Param_lite->u8Output_DataFormat = E_CFD_MC_FORMAT_YUV444;

    //E_CFD_CFIO_RANGE
    //0:limit 1:full
    pstu_CfdAPI_MainControl_Param_lite->u8Output_IsFullRange = E_GFLIP_CFD_CFIO_RANGE_FULL;
    //pstu_CfdAPI_MainControl_Param_lite->u8Output_IsFullRange = E_CFD_CFIO_RANGE_LIMIT;

    //E_CFIO_HDR_STATUS
    //0:SDR 1:HDR1 2:HDR2
    pstu_CfdAPI_MainControl_Param_lite->u8Output_HDRMode = E_CFIO_MODE_SDR;



    pstu_CfdAPI_MainControl_Param_lite->u8Output_ext_Colour_primaries         = E_CFD_CFIO_CP_BT709_SRGB_SYCC;

    //assign by E_CFD_CFIO_TR
    pstu_CfdAPI_MainControl_Param_lite->u8Output_ext_Transfer_Characteristics = E_CFD_CFIO_TR_BT709;

    //assign by E_CFD_CFIO_MC
    pstu_CfdAPI_MainControl_Param_lite->u8Output_ext_Matrix_Coeffs            = E_CFD_CFIO_MC_BT709_XVYCC709;

    //used this gamut when u8Output_ext_Colour_primaries is E_CFD_CFIO_CP_SOURCE
    _MDrv_GFLIP_Set_709Gamut_Colorimetry(&(pstu_CfdAPI_MainControl_Param_lite->stu_Cfd_target_ColorMetry));

}

E_CFD_API_Status _MDrv_GFLIP_Set_InitValue_UIParam(STU_CFDAPI_UI_CONTROL *pstu_UI_Param)
{
    E_CFD_API_Status APIStatus;

    APIStatus = E_CFD_MC_ERR_NOERR;

    if (NULL == pstu_UI_Param)
    {
        return E_CFD_ERR_NULLPOINTER;
    }

    pstu_UI_Param->u16Hue = 50;
    pstu_UI_Param->u16Saturation = 128;
    pstu_UI_Param->u16Contrast   = 1024;

    pstu_UI_Param->u8ColorCorrection_En = 0x00;

    memset(pstu_UI_Param->s32ColorCorrectionMatrix, 0x00, sizeof(MS_S32)*9);

    pstu_UI_Param->s32ColorCorrectionMatrix[0][0] = 1024;
    pstu_UI_Param->s32ColorCorrectionMatrix[1][1] = 1024;
    pstu_UI_Param->s32ColorCorrectionMatrix[2][2] = 1024;

    pstu_UI_Param->u16Brightness = 1024;
    //pstu_UI_Param->u16Brightness[1] = 1024;
    //pstu_UI_Param->u16Brightness[2] = 1024;

    pstu_UI_Param->u16RGBGGain[0] = 1024;
    pstu_UI_Param->u16RGBGGain[1] = 1024;
    pstu_UI_Param->u16RGBGGain[2] = 1024;


    pstu_UI_Param->u8OSD_UI_En = 1;
    pstu_UI_Param->u8OSD_UI_Mode = 0;
    pstu_UI_Param->u8HDR_UI_H2SMode = 0;// 0:auto 1:HDR2SDR 2:HDRbyPass

    //pstu_UI_Param->u16Hue        = 50;
    //pstu_UI_Param->u16Saturation = 140;
    //pstu_UI_Param->u16Contrast   = 1000;


    return APIStatus;
}

void _MDrv_GFLIP_Cfd_inter_PANEL_Param_Init(STU_CFDAPI_PANEL_FORMAT *pstPanelParamInita)
{
    //order R->G->B
    //BT709
    //data *0.00002 0xC350 = 1

    pstPanelParamInita->stu_Cfd_Panel_ColorMetry.u16Display_Primaries_x[0] = 0x7D00; //0.64
    pstPanelParamInita->stu_Cfd_Panel_ColorMetry.u16Display_Primaries_x[1] = 0x3A98; //0.3
    pstPanelParamInita->stu_Cfd_Panel_ColorMetry.u16Display_Primaries_x[2] = 0x1D4C; //0.15

    pstPanelParamInita->stu_Cfd_Panel_ColorMetry.u16Display_Primaries_y[0] = 0x4047; //0.33
    pstPanelParamInita->stu_Cfd_Panel_ColorMetry.u16Display_Primaries_y[1] = 0x7530; //0.6
    pstPanelParamInita->stu_Cfd_Panel_ColorMetry.u16Display_Primaries_y[2] = 0x0BB8; //0.06

    pstPanelParamInita->stu_Cfd_Panel_ColorMetry.u16White_point_x = 0x3D13; //0.3127
    pstPanelParamInita->stu_Cfd_Panel_ColorMetry.u16White_point_y = 0x4042; //0.3290

    pstPanelParamInita->u16Panel_Med_Luminance = 50;

    pstPanelParamInita->u16Panel_Max_Luminance = 100;
    //data * 1 nits

    pstPanelParamInita->u16Panel_Min_Luminance = 500;
    //data * 0.0001 nits
}

E_CFD_API_Status _MDrv_GFLIP_Cfd_InitPreSdrIpParam(STU_CFDAPI_GOP_PRESDRIP *pstu_PreSDRIP_Param)
{
    E_CFD_API_Status APIStatus;
    APIStatus = E_CFD_MC_ERR_NOERR;

    if (NULL == pstu_PreSDRIP_Param)
    {
        return E_CFD_ERR_NULLPOINTER;
    }

    pstu_PreSDRIP_Param->u32Version                  = STU_CFDAPI_GOP_PRESDRIP_VERSION;
    pstu_PreSDRIP_Param->u16Length                   = sizeof(STU_CFDAPI_GOP_PRESDRIP);

    pstu_PreSDRIP_Param->u8CSC_Mode                  = 0xC0;
    pstu_PreSDRIP_Param->u8CSC_Ratio1                = 0x40;
    pstu_PreSDRIP_Param->u8CSC_Manual_Vars_en        = 0;
    pstu_PreSDRIP_Param->u8CSC_MC                    = E_CFD_CFIO_MC_BT709_XVYCC709;

    return APIStatus;
}

E_CFD_API_Status _MDrv_GFLIP_Set_InitValue_HWParam(STU_CFDAPI_HW_IPS_GOP *pstu_IP_Param)
{
    E_CFD_API_Status APIStatus;

    APIStatus = E_CFD_MC_ERR_NOERR;

    if (NULL == pstu_IP_Param)
    {
        return E_CFD_ERR_NULLPOINTER;
    }

    //pstu_IP_Param->u8HWGroupMode = 1;
    pstu_IP_Param->u8HWGroup     = 0;

    _MDrv_GFLIP_Cfd_InitPreSdrIpParam(&(pstu_IP_Param->pstu_PRESDRIP_Input->stu_GOP_PRESDRIP_Param));

    return APIStatus;
}

E_CFD_API_Status _MDrv_GFLIP_Set_InitValue_GOPFormatParam(STU_CFDAPI_GOP_FORMAT *pstu_Param)
{
    E_CFD_API_Status APIStatus;

    MS_U8 IsPreMulAlpha = 0;
    MS_U8 IsAlphaForGOP = 1;

    //bit[0]:
    //1: GOP use premultiplied Alpha

    //bit[1]: IsAlphaForGOPFlag
    //1: alpha value is for GOP


    APIStatus = E_CFD_MC_ERR_NOERR;

    if (NULL == pstu_Param)
    {
        return E_CFD_ERR_NULLPOINTER;
    }

    pstu_Param->u8GOP_AlphaFormat = (((IsAlphaForGOP&0x01)<<1)|(IsPreMulAlpha&0x01));

    return APIStatus;
}

void _MDrv_GFLIP_Set_InitValue_DebugMode(STU_CFDAPI_DEBUG *pst)
{
    pst->ShowALLInputInCFDEn = 0;
}

//Preprocess to PreSDR Convert
static MS_BOOL _MDrv_GFLIP_Set_SetControlPointBlackAndWhite(STU_CFD_CONTROL_POINT* pstControlPoint,ST_KDRV_XC_CFD_BLACKANDWHITE* pstBlackAndWhite)
{
    if ((pstBlackAndWhite->u8BlackLevel == 0) && (pstBlackAndWhite->u8WhiteLevel == 0))
    {
        if (pstControlPoint->u8IsFullRange == E_GFLIP_CFD_CFIO_RANGE_FULL)
        {
            pstControlPoint->u16BlackLevelC = 0;
            pstControlPoint->u16BlackLevelY = 0;
            pstControlPoint->u16WhiteLevelC = 1023;
            pstControlPoint->u16WhiteLevelY = 1023;
        }
        else
        {
            if(pstControlPoint->u8DataFormat == E_GFLIP_CFD_MC_FORMAT_RGB)
            {
                pstControlPoint->u16BlackLevelC = 64;
                pstControlPoint->u16BlackLevelY = 64;
                pstControlPoint->u16WhiteLevelC = 940;
                pstControlPoint->u16WhiteLevelY = 940;
            }
            else
            {
                if( E_CFD_CFIO_MC_YCGCO != pstControlPoint->u8MatrixCoefficients )
                {
                    pstControlPoint->u16BlackLevelC = 64;
                    pstControlPoint->u16BlackLevelY = 64;
                    pstControlPoint->u16WhiteLevelC = 960;
                    pstControlPoint->u16WhiteLevelY = 940;
                }
                else
                {
                    pstControlPoint->u16BlackLevelC = 44;
                    pstControlPoint->u16BlackLevelY = 64;
                    pstControlPoint->u16WhiteLevelC = 980;
                    pstControlPoint->u16WhiteLevelY = 940;
                }
            }
        }
    }
    else
    {
        MS_U8 u8Blackinput = (MIN(pstBlackAndWhite->u8BlackLevel, 64)) * 2;
        MS_U8 u8Whiteinput = (MIN(pstBlackAndWhite->u8WhiteLevel, 64)) * 2;
        MS_U16 u16BLevel = 64 - ((64 * u8Blackinput) >> 7);
        MS_U16 u16WLevel = 940 + ((83 * u8Whiteinput) >> 7);
        pstControlPoint->u16BlackLevelC = u16BLevel;
        pstControlPoint->u16BlackLevelY = u16BLevel;
        pstControlPoint->u16WhiteLevelC = u16WLevel;
        pstControlPoint->u16WhiteLevelY = u16WLevel;
    }

    return TRUE;
}

MS_BOOL _MDrv_GFLIP_GenarateControlPoints_lite(STU_CFDAPI_GOP_PREPROCESS_OUT *pstPreProcessOutInfo,
                                    STU_CFD_GENERAL_CONTROL_GOP   *pstGeneralControl,
                                    STU_CFD_CONTROL_POINT         *pstControlPoint,
                                    STU_CFDAPI_UI_CONTROL         *pstUI)
{

    ST_KDRV_XC_CFD_BLACKANDWHITE stBlackAndWhite[CFD_CTRL_POINT_NUM];
    memset(&stBlackAndWhite, 0, CFD_CTRL_POINT_NUM * sizeof(ST_KDRV_XC_CFD_BLACKANDWHITE));

    pstGeneralControl->u32Version    = CFD_CONTROL_POINT_ST_VERSION;
    pstGeneralControl->u16Length     = sizeof(STU_CFD_GENERAL_CONTROL_GOP);
    //pstGeneralControl->u8InputSource = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Input_Format;

    pstGeneralControl->u8HWGroup      = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8HWGroup;
    pstGeneralControl->u8HWGroupMode  = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8HWGroupMode;

    pstControlPoint[0].u32Version     = CFD_CONTROL_POINT_ST_VERSION;
    pstControlPoint[0].u16Length      = sizeof(STU_CFD_GENERAL_CONTROL_GOP);
    pstControlPoint[0].u8MainSubMode  = 0;
    pstControlPoint[0].u8Format       = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Input_Format;
    pstControlPoint[0].u8DataFormat   = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Input_DataFormat;
    pstControlPoint[0].u8IsFullRange  = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Input_IsFullRange;
    pstControlPoint[0].u8HDRMode      = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Input_HDRMode;
    pstControlPoint[0].u8SDRIPMode    = 1;
    pstControlPoint[0].u8HDRIPMode    = 1;
    pstControlPoint[0].u8GamutOrderIdx = 1;
    pstControlPoint[0].u8ColorPriamries = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8InputColorPriamries;
    pstControlPoint[0].u8TransferCharacterstics = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8InputTransferCharacterstics;
    pstControlPoint[0].u8MatrixCoefficients = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8InputMatrixCoefficients;

    memcpy(&(pstControlPoint[0].stu_Panel_Param_Colorimetry), &(pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.stu_Cfd_ColorMetry[0]), sizeof(STU_CFD_COLORIMETRY));

    //memory copy for source and target
    //pstControlPoint[0].stu_Panel_Param_Colorimetry

    _MDrv_GFLIP_Set_SetControlPointBlackAndWhite(&pstControlPoint[0], &stBlackAndWhite[0]);

    // always not do HDR2SDR for HDR input
    if (pstUI->u8HDR_UI_H2SMode == 2)
    {
        memcpy(&pstControlPoint[1], &pstControlPoint[0], sizeof(STU_CFD_CONTROL_POINT));
        /*memcpy(&pstControlPoint[2], &pstControlPoint[0], sizeof(STU_CFD_CONTROL_POINT));
        memcpy(&pstControlPoint[3], &pstControlPoint[0], sizeof(STU_CFD_CONTROL_POINT));*/
    }
    else
    {
        pstControlPoint[1].u32Version = CFD_CONTROL_POINT_ST_VERSION;
        pstControlPoint[1].u16Length = sizeof(STU_CFD_GENERAL_CONTROL_GOP);
        pstControlPoint[1].u8MainSubMode = 0;
        pstControlPoint[1].u8Format = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Temp_Format[0];
        pstControlPoint[1].u8DataFormat = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Temp_DataFormat[0];
        pstControlPoint[1].u8IsFullRange = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Temp_IsFullRange[0];
        pstControlPoint[1].u8HDRMode = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Temp_HDRMode[0];
        pstControlPoint[1].u8SDRIPMode = 1;
        pstControlPoint[1].u8HDRIPMode = 1;
        pstControlPoint[1].u8GamutOrderIdx = 1;
        pstControlPoint[1].u8ColorPriamries = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8TempColorPriamries[0];
        pstControlPoint[1].u8TransferCharacterstics = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8TempTransferCharacterstics[0];
        pstControlPoint[1].u8MatrixCoefficients = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8TempMatrixCoefficients[0];
        memcpy(&(pstControlPoint[1].stu_Source_Param_Colorimetry), &(pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.stu_Cfd_ColorMetry[1]), sizeof(STU_CFD_COLORIMETRY));


        _MDrv_GFLIP_Set_SetControlPointBlackAndWhite(&pstControlPoint[1], &stBlackAndWhite[1]);

        /*pstControlPoint[2].u32Version = CFD_CONTROL_POINT_ST_VERSION;
        pstControlPoint[2].u16Length = sizeof(STU_CFD_GENERAL_CONTROL);
        pstControlPoint[2].u8MainSubMode = 0;
        pstControlPoint[2].u8Format = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Temp_Format[1];
        pstControlPoint[2].u8DataFormat = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Temp_DataFormat[1];
        pstControlPoint[2].u8IsFullRange = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Temp_IsFullRange[1];
        pstControlPoint[2].u8HDRMode = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Temp_HDRMode[1];
        pstControlPoint[2].u8SDRIPMode = 1;
        pstControlPoint[2].u8HDRIPMode = 1;
        pstControlPoint[2].u8GamutOrderIdx = 1;
        memcpy(&(pstControlPoint[2].stu_Source_Param_Colorimetry), &(pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.stu_Cfd_ColorMetry[2]), sizeof(STU_CFD_COLORIMETRY));

        if(_IsOpenHDR(pstControlPoint[1].u8HDRMode))
        {
            pstControlPoint[2].u8ColorPriamries = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8TempColorPriamries[1];
            pstControlPoint[2].u8TransferCharacterstics = E_CFD_CFIO_TR_GAMMA2P2;//pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8TempTransferCharacterstics[1];
            pstControlPoint[2].u8MatrixCoefficients = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8TempMatrixCoefficients[1];
        }
        else
        {
            pstControlPoint[2].u8ColorPriamries = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8TempColorPriamries[1];
            pstControlPoint[2].u8TransferCharacterstics = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8TempTransferCharacterstics[1];
            pstControlPoint[2].u8MatrixCoefficients = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8TempMatrixCoefficients[1];
        }
        _MDrv_GFLIP_Set_SetControlPointBlackAndWhite(&pstControlPoint[2], &astBlackAndWhite[2]);

        pstControlPoint[3].u32Version = CFD_CONTROL_POINT_ST_VERSION;
        pstControlPoint[3].u16Length = sizeof(STU_CFD_GENERAL_CONTROL);
        pstControlPoint[3].u8MainSubMode = 0;
        pstControlPoint[3].u8Format = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Output_Format;
        pstControlPoint[3].u8DataFormat = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Output_DataFormat;
        pstControlPoint[3].u8IsFullRange = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Output_IsFullRange;
        pstControlPoint[3].u8HDRMode = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8Output_HDRMode;
        pstControlPoint[3].u8SDRIPMode = 1;
        pstControlPoint[3].u8HDRIPMode = 1;
        pstControlPoint[3].u8GamutOrderIdx = 1;
        pstControlPoint[3].u8ColorPriamries = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8OutputColorPriamries;
        pstControlPoint[3].u8TransferCharacterstics = E_CFD_CFIO_TR_GAMMA2P2;//pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8OutputTransferCharacterstics;
        pstControlPoint[3].u8MatrixCoefficients = pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.u8OutputMatrixCoefficients;
        memcpy(&(pstControlPoint[3].stu_Source_Param_Colorimetry), &(pstPreProcessOutInfo->stu_Cfd_preprocess_status.stu_Cfd_color_format.stu_Cfd_ColorMetry[3]), sizeof(STU_CFD_COLORIMETRY));

        _MDrv_GFLIP_Set_SetControlPointBlackAndWhite(&pstControlPoint[3], &astBlackAndWhite[3]);*/
    }

    return TRUE;
}

MS_BOOL _MDrv_GFLIP_CFD_Wrapper_PreSDR(STU_CFDAPI_TOP_CONTROL_GOP *pstPreprocessTopCtrl,
                                       STU_CFDAPI_GOP_PREPROCESS_OUT *pstPreprocessOut,
                                       ST_CFD_GOP_PRESDR_INPUT *pstPreSDRInput)
{
    STU_CFD_CONTROL_POINT stTmpCtrlPoint[CFD_CTRL_POINT_NUM];

    _MDrv_GFLIP_GenarateControlPoints_lite(pstPreprocessOut,pstPreSDRInput->pstu_Cfd_General_Control,&stTmpCtrlPoint[0],pstPreprocessTopCtrl->pstu_UI_Param);

    memcpy(pstPreSDRInput->pstu_Cfd_Control_Point_Front, &stTmpCtrlPoint[0], sizeof(STU_CFD_CONTROL_POINT));
    memcpy(pstPreSDRInput->pstu_Cfd_Control_Point_End, &stTmpCtrlPoint[1], sizeof(STU_CFD_CONTROL_POINT));
    memcpy(pstPreSDRInput->pstu_SDRIP_Param, pstPreprocessTopCtrl->pstu_HW_IP_Param->pstu_PRESDRIP_Input, sizeof(STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP));
    memcpy(pstPreSDRInput->pstu_UI_control, pstPreprocessTopCtrl->pstu_UI_Param, sizeof(STU_CFDAPI_UI_CONTROL));


    return TRUE;
}

MS_BOOL _MDrv_GFLIP_CSC_Calc(ST_GFLIP_GOP_CSC_PARAM *pstGflipCSCParam)
{
    //Preporcess input
    STU_CFDAPI_TOP_CONTROL_GOP *pstPreprocessTopCtrl = NULL;
    STU_CFDAPI_MAIN_CONTROL_GOP *pstPreprocessMainCtrl = NULL;
    STU_CFDAPI_UI_CONTROL *pstPreprocessUICtrl = NULL;
    STU_CFDAPI_PANEL_FORMAT *pstPreprocessPanelFmt = NULL;
    STU_CFDAPI_HW_IPS_GOP *pstPreprocessHWIPS = NULL;
    STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP *pstPreprocessPreSDRIP = NULL;
    STU_CFDAPI_GOP_FORMAT *pstPreprocessGOPFmt = NULL;
    STU_CFDAPI_DEBUG *pstPreprocessDebug = NULL;
    //PreProcess output
    STU_CFDAPI_GOP_PREPROCESS_OUT *pstPreprocessOut = NULL;
    //PreSDR input
    ST_CFD_GOP_PRESDR_INPUT *pstPreSDRInput = NULL;
    STU_CFD_GENERAL_CONTROL_GOP *pstPreSDRGeneralCtrl = NULL;
    STU_CFD_CONTROL_POINT *pstPreSDRCtrlPointFront = NULL;
    STU_CFD_CONTROL_POINT *pstPreSDRCtrlPointEnd = NULL;
    STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP *pstPreSDRIPParam = NULL;
    STU_CFDAPI_UI_CONTROL *pstPreSDRUICtrl = NULL;
    //PreSDR output
    ST_CFD_GOP_PreSDR_HW_OUTPUT *pstPreSDROut = NULL;
    MS_BOOL bRet = TRUE;

    //Preporcess input kmalloc
    pstPreprocessTopCtrl = kmalloc(sizeof(STU_CFDAPI_TOP_CONTROL_GOP),GFP_KERNEL);
    if(pstPreprocessTopCtrl == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    pstPreprocessMainCtrl = kmalloc(sizeof(STU_CFDAPI_MAIN_CONTROL_GOP),GFP_KERNEL);
    if(pstPreprocessMainCtrl == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    pstPreprocessUICtrl = kmalloc(sizeof(STU_CFDAPI_UI_CONTROL),GFP_KERNEL);
    if(pstPreprocessUICtrl == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    pstPreprocessPanelFmt = kmalloc(sizeof(STU_CFDAPI_PANEL_FORMAT),GFP_KERNEL);
    if(pstPreprocessPanelFmt == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    pstPreprocessHWIPS = kmalloc(sizeof(STU_CFDAPI_HW_IPS_GOP),GFP_KERNEL);
    if(pstPreprocessHWIPS == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    pstPreprocessPreSDRIP = kmalloc(sizeof(STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP),GFP_KERNEL);
    if(pstPreprocessPreSDRIP == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    pstPreprocessGOPFmt = kmalloc(sizeof(STU_CFDAPI_GOP_FORMAT),GFP_KERNEL);
    if(pstPreprocessGOPFmt == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    pstPreprocessDebug = kmalloc(sizeof(STU_CFDAPI_DEBUG),GFP_KERNEL);
    if(pstPreprocessDebug == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    //Preporcess output kmalloc
    pstPreprocessOut = kmalloc(sizeof(STU_CFDAPI_GOP_PREPROCESS_OUT),GFP_KERNEL);
    if(pstPreprocessOut == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    //PreSDR input kmalloc
    pstPreSDRInput = kmalloc(sizeof(ST_CFD_GOP_PRESDR_INPUT),GFP_KERNEL);
    if(pstPreSDRInput == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    pstPreSDRGeneralCtrl = kmalloc(sizeof(STU_CFD_GENERAL_CONTROL_GOP),GFP_KERNEL);
    if(pstPreSDRGeneralCtrl == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    pstPreSDRCtrlPointFront= kmalloc(sizeof(STU_CFD_CONTROL_POINT),GFP_KERNEL);
    if(pstPreSDRCtrlPointFront == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    pstPreSDRCtrlPointEnd = kmalloc(sizeof(STU_CFD_CONTROL_POINT),GFP_KERNEL);
    if(pstPreSDRCtrlPointEnd == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    pstPreSDRIPParam = kmalloc(sizeof(STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP),GFP_KERNEL);
    if(pstPreSDRIPParam == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    pstPreSDRUICtrl = kmalloc(sizeof(STU_CFDAPI_UI_CONTROL),GFP_KERNEL);
    if(pstPreSDRUICtrl == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    //PreSDR output kmalloc
    pstPreSDROut = kmalloc(sizeof(ST_CFD_GOP_PreSDR_HW_OUTPUT),GFP_KERNEL);
    if(pstPreSDROut == NULL)
    {
        printk("[%s][%d] kmalloc fail\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }

    //PrePorcess input init
    memset(pstPreprocessTopCtrl, 0, sizeof(STU_CFDAPI_TOP_CONTROL_GOP));
    memset(pstPreprocessMainCtrl, 0, sizeof(STU_CFDAPI_MAIN_CONTROL_GOP));
    memset(pstPreprocessUICtrl, 0, sizeof(STU_CFDAPI_UI_CONTROL));
    memset(pstPreprocessPanelFmt, 0, sizeof(STU_CFDAPI_PANEL_FORMAT));
    memset(pstPreprocessHWIPS, 0, sizeof(STU_CFDAPI_HW_IPS_GOP));
    memset(pstPreprocessPreSDRIP, 0, sizeof(STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP));
    memset(pstPreprocessGOPFmt, 0, sizeof(STU_CFDAPI_GOP_FORMAT));
    memset(pstPreprocessDebug, 0, sizeof(STU_CFDAPI_DEBUG));
    //PrePorcess output init
    memset(pstPreprocessOut, 0, sizeof(STU_CFDAPI_GOP_PREPROCESS_OUT));
    //PreSDR input init
    memset(pstPreSDRInput, 0, sizeof(ST_CFD_GOP_PRESDR_INPUT));
    memset(pstPreSDRGeneralCtrl, 0, sizeof(STU_CFD_GENERAL_CONTROL_GOP));
    memset(pstPreSDRCtrlPointFront, 0,sizeof(STU_CFD_CONTROL_POINT));
    memset(pstPreSDRCtrlPointEnd, 0,sizeof(STU_CFD_CONTROL_POINT));
    memset(pstPreSDRIPParam, 0, sizeof(STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP));
    memset(pstPreSDRUICtrl, 0, sizeof(STU_CFDAPI_UI_CONTROL));
    //PreSDR output init
    memset(pstPreSDROut, 0, sizeof(ST_CFD_GOP_PreSDR_HW_OUTPUT));

#if(CFD_DEBUG_LOG)
    printk("[%s][%d]u32GOPNum = %u\n",__func__,__LINE__,pstGflipCSCParam->u32GOPNum);
    printk("[%s][%d]u16Hue = %u\n",__func__,__LINE__,pstGflipCSCParam->u16Hue);
    printk("[%s][%d]u16Saturation = %u\n",__func__,__LINE__,pstGflipCSCParam->u16Saturation);
    printk("[%s][%d]u16Contrast = %u\n",__func__,__LINE__,pstGflipCSCParam->u16Contrast);
    printk("[%s][%d]u16Brightness = %u\n",__func__,__LINE__,pstGflipCSCParam->u16Brightness);
    printk("[%s][%d]u16RGBGGain[0] = %u\n",__func__,__LINE__,pstGflipCSCParam->u16RGBGGain[0]);
    printk("[%s][%d]u16RGBGGain[1] = %u\n",__func__,__LINE__,pstGflipCSCParam->u16RGBGGain[1]);
    printk("[%s][%d]u16RGBGGain[2] = %u\n",__func__,__LINE__,pstGflipCSCParam->u16RGBGGain[2]);
    printk("[%s][%d]enInputFormat = %u\n",__func__,__LINE__,pstGflipCSCParam->enInputFormat);
    printk("[%s][%d]enInputDataFormat = %u\n",__func__,__LINE__,pstGflipCSCParam->enInputDataFormat);
    printk("[%s][%d]enInputRange = %u\n",__func__,__LINE__,pstGflipCSCParam->enInputRange);
    printk("[%s][%d]enOutputFormat = %u\n",__func__,__LINE__,pstGflipCSCParam->enOutputFormat);
    printk("[%s][%d]enOutputDataFormat = %u\n",__func__,__LINE__,pstGflipCSCParam->enOutputDataFormat);
    printk("[%s][%d]enOutputRange = %u\n",__func__,__LINE__,pstGflipCSCParam->enOutputRange);
#endif

    //PrePorcess input
    pstPreprocessTopCtrl->pstu_Main_Control = pstPreprocessMainCtrl;
    pstPreprocessTopCtrl->pstu_UI_Param = pstPreprocessUICtrl;
    pstPreprocessTopCtrl->pstu_Panel_Param = pstPreprocessPanelFmt;
    pstPreprocessTopCtrl->pstu_HW_IP_Param = pstPreprocessHWIPS;
    pstPreprocessHWIPS->pstu_PRESDRIP_Input = pstPreprocessPreSDRIP;
    pstPreprocessTopCtrl->pstu_GOP_Param = pstPreprocessGOPFmt;
    pstPreprocessTopCtrl->pstu_Debug_Param = pstPreprocessDebug;
    //PreSDR input
    pstPreSDRInput->pstu_Cfd_General_Control = pstPreSDRGeneralCtrl;
    pstPreSDRInput->pstu_Cfd_Control_Point_Front = pstPreSDRCtrlPointFront;
    pstPreSDRInput->pstu_Cfd_Control_Point_End = pstPreSDRCtrlPointEnd;
    pstPreSDRInput->pstu_SDRIP_Param = pstPreSDRIPParam;
    pstPreSDRInput->pstu_UI_control = pstPreSDRUICtrl;
    //PreSDR Out
    pstPreSDROut->stu_cfd_GOP_PreSDR_CSC_hw_output.stRegTable.pu32Address = &(pstGflipCSCParam->u32CSCAddr[0]);
    pstPreSDROut->stu_cfd_GOP_PreSDR_CSC_hw_output.stRegTable.pu16Value = &(pstGflipCSCParam->u16CSCValue[0]);
    pstPreSDROut->stu_cfd_GOP_PreSDR_CSC_hw_output.stRegTable.pu16Mask = &(pstGflipCSCParam->u16CSCMask[0]);
    pstPreSDROut->stu_cfd_GOP_PreSDR_CSC_hw_output.stRegTable.pu16Client = &(pstGflipCSCParam->u16CSCClient[0]);
    pstPreSDROut->stu_cfd_GOP_PreSDR_CSC_hw_output.stAdlTable.pu8Data = &(pstGflipCSCParam->u8CSCAdlData[0]);
    pstPreSDROut->stu_cfd_GOP_PreSDR_RGBOffset_hw_output.stRegTable.pu32Address = &(pstGflipCSCParam->u32BriAddr[0]);
    pstPreSDROut->stu_cfd_GOP_PreSDR_RGBOffset_hw_output.stRegTable.pu16Value = &(pstGflipCSCParam->u16BriValue[0]);
    pstPreSDROut->stu_cfd_GOP_PreSDR_RGBOffset_hw_output.stRegTable.pu16Mask = &(pstGflipCSCParam->u16BriMask[0]);
    pstPreSDROut->stu_cfd_GOP_PreSDR_RGBOffset_hw_output.stRegTable.pu16Client = &(pstGflipCSCParam->u16BriClient[0]);
    pstPreSDROut->stu_cfd_GOP_PreSDR_RGBOffset_hw_output.stAdlTable.pu8Data = &(pstGflipCSCParam->u8BriAdlData[0]);

    //Init
    _MDrv_GFLIP_Set_InitValue_Main_Control_GOP(pstPreprocessMainCtrl);
    _MDrv_GFLIP_Set_InitValue_UIParam(pstPreprocessUICtrl);
    _MDrv_GFLIP_Cfd_inter_PANEL_Param_Init(pstPreprocessPanelFmt);
    _MDrv_GFLIP_Set_InitValue_HWParam(pstPreprocessHWIPS);
    _MDrv_GFLIP_Set_InitValue_GOPFormatParam(pstPreprocessGOPFmt);
    _MDrv_GFLIP_Set_InitValue_DebugMode(pstPreprocessDebug);

#if(CFD_DEBUG_LOG)
    pstPreprocessTopCtrl->pstu_Debug_Param->ShowALLInputInCFDEn = 1;
    pstPreSDRInput->pstu_Cfd_General_Control->stu_Debug_Param.ShowALLInputInCFDEn = 1;
#endif

    pstPreprocessMainCtrl->u8Input_Format = (MS_U8)pstGflipCSCParam->enInputFormat;
    pstPreprocessMainCtrl->u8Input_DataFormat = (MS_U8)pstGflipCSCParam->enInputDataFormat;
    pstPreprocessMainCtrl->u8Input_IsFullRange = (MS_U8)pstGflipCSCParam->enInputRange;
    pstPreprocessMainCtrl->u8Output_Format = (MS_U8)pstGflipCSCParam->enOutputFormat;
    pstPreprocessMainCtrl->u8Output_DataFormat = (MS_U8)pstGflipCSCParam->enOutputDataFormat;
    pstPreprocessMainCtrl->u8Output_IsFullRange = (MS_U8)pstGflipCSCParam->enOutputRange;

    pstPreprocessUICtrl->u16Hue = pstGflipCSCParam->u16Hue;
    pstPreprocessUICtrl->u16Saturation = pstGflipCSCParam->u16Saturation;
    pstPreprocessUICtrl->u16Contrast = pstGflipCSCParam->u16Contrast;
    pstPreprocessUICtrl->u16Brightness = pstGflipCSCParam->u16Brightness;
    pstPreprocessUICtrl->u16RGBGGain[0] = pstGflipCSCParam->u16RGBGGain[0];
    pstPreprocessUICtrl->u16RGBGGain[1] = pstGflipCSCParam->u16RGBGGain[1];
    pstPreprocessUICtrl->u16RGBGGain[2] = pstGflipCSCParam->u16RGBGGain[2];

    pstPreprocessHWIPS->u8HWGroup = pstGflipCSCParam->u32GOPNum;

    if(_fpGflip_CB_CFD_GOP_PreProcess == NULL)
    {
        printk("[%s][%d] _fpGflip_CB_CFD_GOP_PreProcess is NULL.\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    _fpGflip_CB_CFD_GOP_PreProcess(pstPreprocessTopCtrl,pstPreprocessOut);

    _MDrv_GFLIP_CFD_Wrapper_PreSDR(pstPreprocessTopCtrl,pstPreprocessOut,pstPreSDRInput);

    if(_fpGflip_CB_CFD_GOP_PreSDR == NULL)
    {
        printk("[%s][%d] _fpGflip_CB_CFD_GOP_PreSDR is NULL.\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    _fpGflip_CB_CFD_GOP_PreSDR(pstPreSDRInput,pstPreSDROut);


#if(CFD_TEST_MODE)
    STU_CFDAPI_TOP_CONTROL_GOP_TESTING stu_CfdAPI_Top_Param_gop_testing;

    memset(&stu_CfdAPI_Top_Param_gop_testing, 0, sizeof(STU_CFDAPI_TOP_CONTROL_GOP_TESTING));

    pstPreprocessTopCtrl->pstu_Debug_Param->ShowALLInputInCFDEn = 1;
    stu_CfdAPI_Top_Param_gop_testing.u8TestEn = 1;
    //select test case of STU_CFDAPI_MAIN_CONTROL_GOP parameters
    stu_CfdAPI_Top_Param_gop_testing.stu_Main_Control.u32TestCases = 3;
    //select test case of STU_CFDAPI_UI_CONTROL parameters
    stu_CfdAPI_Top_Param_gop_testing.stu_UI_Param.u32TestCases     = 14;

    if(_fpGflip_CB_CFD_GOP_TestMode == NULL)
    {
        printk("[%s][%d] _fpGflip_CB_CFD_GOP_TestMode is NULL.\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    _fpGflip_CB_CFD_GOP_TestMode(pstPreprocessTopCtrl, &stu_CfdAPI_Top_Param_gop_testing);

    pstPreSDRInput->pstu_Cfd_General_Control->stu_Debug_Param.ShowALLInputInCFDEn = 1;

    if(_fpGflip_CB_CFD_GOP_PreProcess == NULL)
    {
        printk("[%s][%d] _fpGflip_CB_CFD_GOP_PreProcess is NULL.\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    _fpGflip_CB_CFD_GOP_PreProcess(pstPreprocessTopCtrl,pstPreprocessOut);

    _MDrv_GFLIP_CFD_Wrapper_PreSDR(pstPreprocessTopCtrl,pstPreprocessOut,pstPreSDRInput);

    if(_fpGflip_CB_CFD_GOP_PreSDR == NULL)
    {
        printk("[%s][%d] _fpGflip_CB_CFD_GOP_PreSDR is NULL.\n",__func__,__LINE__);
        bRet = FALSE;
        goto CSC_CALC_FREE;
    }
    _fpGflip_CB_CFD_GOP_PreSDR(pstPreSDRInput,pstPreSDROut);
#endif

CSC_CALC_FREE:
    if(NULL != pstPreprocessTopCtrl){kfree(pstPreprocessTopCtrl);}
    if(NULL != pstPreprocessMainCtrl){kfree(pstPreprocessMainCtrl);}
    if(NULL != pstPreprocessUICtrl){kfree(pstPreprocessUICtrl);}
    if(NULL != pstPreprocessPanelFmt){kfree(pstPreprocessPanelFmt);}
    if(NULL != pstPreprocessHWIPS){kfree(pstPreprocessHWIPS);}
    if(NULL != pstPreprocessPreSDRIP){kfree(pstPreprocessPreSDRIP);}
    if(NULL != pstPreprocessGOPFmt){kfree(pstPreprocessGOPFmt);}
    if(NULL != pstPreprocessDebug){kfree(pstPreprocessDebug);}
    if(NULL != pstPreprocessOut){kfree(pstPreprocessOut);}
    if(NULL != pstPreSDRInput){kfree(pstPreSDRInput);}
    if(NULL != pstPreSDRGeneralCtrl){kfree(pstPreSDRGeneralCtrl);}
    if(NULL != pstPreSDRCtrlPointFront){kfree(pstPreSDRCtrlPointFront);}
    if(NULL != pstPreSDRCtrlPointEnd){kfree(pstPreSDRCtrlPointEnd);}
    if(NULL != pstPreSDRIPParam){kfree(pstPreSDRIPParam);}
    if(NULL != pstPreSDRUICtrl){kfree(pstPreSDRUICtrl);}
    if(NULL != pstPreSDROut){kfree(pstPreSDROut);}

        return bRet;
}

MS_BOOL _MDrv_GFLIP_SetBWPEnFlag(MS_U32 u32GopIdx, MS_BOOL bEnable)
{
    if(u32GopIdx >= MAX_GOP_SUPPORT)
    {
        return FALSE;
    }

    _gbBWPEnFlag[u32GopIdx] = bEnable;

    return TRUE;
}

EXPORT_SYMBOL(MDrv_GFLIP_GetGwinInfo);


