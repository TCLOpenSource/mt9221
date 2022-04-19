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
/// file    drvPWM.c
/// @brief  Pulse Width Modulation Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Include Files
////////////////////////////////////////////////////////////////////////////////
//#include "MsCommon.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
//#include "MsVersion.h"
#include "mdrv_pwm.h"
#include "mhal_pwm.h"
//#include "MsOS.h"

////////////////////////////////////////////////////////////////////////////////
// Local defines & local structures
////////////////////////////////////////////////////////////////////////////////

#define PWM_DBG_FUNC()            if (_geDbfLevel >= E_PWM_DBGLV_ALL)                   \
                                       {MS_DEBUG_MSG(printk("\t====   %s   ====\n", __FUNCTION__));}

#define PWM_DBG_INFO(x, args...)  if (_geDbfLevel >= E_PWM_DBGLV_INFO )                 \
                                       {MS_DEBUG_MSG(printk(x, ##args));}

#define PWM_DBG_ERR(x, args...)   if (_geDbfLevel >= E_PWM_DBGLV_ERR_ONLY)				\
                                       {MS_DEBUG_MSG(printk(x, ##args));}

////////////////////////////////////////////////////////////////////////////////
// Local & Global Variables
////////////////////////////////////////////////////////////////////////////////
static BOOL  _gbInitPWM = FALSE;
//static PWM_Arg _gsPWMDesc;
static PWM_DbgLv _geDbfLevel = E_PWM_DBGLV_NONE;

static PWM_DrvStatus   _pwmStatus =
                        {
                            .PWM0 = FALSE,
                            .PWM1 = FALSE,
                            .PWM2 = FALSE,
                            .PWM3 = FALSE,
                            .PWM4 = FALSE,
                            .PWM5 = FALSE,
                            .PWM6 = FALSE,
                            .PWM7 = FALSE,
                            .PWM8 = FALSE,
#ifdef CONFIG_MTKPVR_PWM
                            .PVR_PWM0 = FALSE,
                            .PVR_PWM1 = FALSE,
#endif
                        };

////////////////////////////////////////////////////////////////////////////////
// Local Function
////////////////////////////////////////////////////////////////////////////////
static BOOL _PWM_IsInit(void);

//-------------------------------------------------------------------------------------------------
/// Description: Set debug level for debug message
/// @param eLevel    \b debug level for PWM driver
/// @return E_PWM_OK : succeed
/// @return E_PWM_FAIL : fail before timeout or illegal parameters
//-------------------------------------------------------------------------------------------------
PWM_Result MDrv_PWM_SetDbgLevel(PWM_DbgLv eLevel)
{
    PWM_DBG_INFO("%s(%u)\r\n", __FUNCTION__, eLevel);

    _geDbfLevel = eLevel;

    return E_PWM_OK;
}

//------------------------------------------------------------------------------
/// Description : Show the PWM_PAD is PWM(True) or GPIO(false)
/// @param  pStatus \b OUT: output PWM driver status
/// @return E_PWM_OK : succeed
/// @return E_PWM_FAIL : fail before timeout or illegal parameters
//------------------------------------------------------------------------------
PWM_Result MDrv_PWM_GetStatus(PWM_DrvStatus *pStatus)
{
    memcpy(pStatus, &_pwmStatus, sizeof(PWM_DrvStatus));
    return E_PWM_OK;
}

//-------------------------------------------------------------------------------------------------
/// Description : Check PWM is initial or not
/// @return TRUE : PWM was initied
/// @return FALSE : PWM was not initied
//-------------------------------------------------------------------------------------------------
static BOOL _PWM_IsInit(void)
{
    PWM_DBG_FUNC();
    if (!_gbInitPWM)
        PWM_DBG_ERR("Call MDrv_PWM_Init first!\n");
    return _gbInitPWM;
}

//-------------------------------------------------------------------------------------------------
/// Description: Initial PWM driver
/// @param eLevel    \b debug level for PWM driver
/// @return E_PWM_OK : succeed
/// @return E_PWM_FAIL : fail before timeout or illegal parameters
//-------------------------------------------------------------------------------------------------
PWM_Result MDrv_PWM_Init(PWM_DbgLv eLevel)
{
    MDrv_PWM_SetDbgLevel(eLevel);

    if(!_gbInitPWM)
    {
        PWM_DBG_INFO("%s\n", __FUNCTION__);
        /* Set All pad output and Is PWM. But it still needs to be enable */
        if( !HAL_PWM_Init() )
        {
            PWM_DBG_ERR("PWM Hal Initial Fail\n");
            return E_PWM_FAIL;
        }

        _gbInitPWM = TRUE;
        return E_PWM_OK;
    }
    else
    {
        PWM_DBG_ERR("PWM has be initiated!\n");
        return E_PWM_OK;
    }
}

//-------------------------------------------------------------------------------------------------
/// Description: IOutput enable_bar of PWM pads
/// @param u8IndexPWM    \b which pwm is setting
/// @param bOenPWM    \b True/False for enable/disable
/// @return E_PWM_OK : succeed
/// @return E_PWM_FAIL : fail before timeout or illegal parameters
//-------------------------------------------------------------------------------------------------
PWM_Result MDrv_PWM_Oen(PWM_ChNum u8IndexPWM, BOOL bOenPWM)
{
    PWM_Result ret = E_PWM_OK;
    PWM_DBG_INFO("%s(%u, %u)\r\n", __FUNCTION__, u8IndexPWM, bOenPWM);

    switch(u8IndexPWM)
    {
        case E_PWM_CH0:
            _pwmStatus.PWM0 = TRUE;
            break;
        case E_PWM_CH1:
            _pwmStatus.PWM1 = TRUE;
            break;
        case E_PWM_CH2:
            _pwmStatus.PWM2 = TRUE;
            break;
        case E_PWM_CH3:
            _pwmStatus.PWM3 = TRUE;
            break;
        case E_PWM_CH4:
            _pwmStatus.PWM4 = TRUE;
            break;
        case E_PWM_CH5:
            _pwmStatus.PWM5 = TRUE;
            break;
        case E_PWM_CH6:
            _pwmStatus.PWM6 = TRUE;
            break;
        case E_PWM_CH7:
            _pwmStatus.PWM7 = TRUE;
            break;
        case E_PWM_CH8:
            _pwmStatus.PWM8 = TRUE;
            break;
#ifdef CONFIG_MTKPVR_PWM
        case E_PVR_PWM_CH0:
            _pwmStatus.PVR_PWM0 = TRUE;
            break;
        case E_PVR_PWM_CH1:
            _pwmStatus.PVR_PWM1 = TRUE;
            break;
#endif
        default:
            PWM_ASSERT(0);
    }

    if( !(HAL_PWM_Oen(u8IndexPWM, bOenPWM)||_PWM_IsInit()) )
    {
        ret = E_PWM_FAIL;
    }
    return ret;
}

//-------------------------------------------------------------------------------------------------
/// Description: Set the period of the specific pwm
/// @param u8IndexPWM    \b which pwm is setting
/// @param u16PeriodPWM    \b the 18-bit period value
/// @return E_PWM_OK : succeed
/// @return E_PWM_FAIL : fail before timeout or illegal parameters
//-------------------------------------------------------------------------------------------------
PWM_Result MDrv_PWM_Period(PWM_ChNum u8IndexPWM, U32 u32PeriodPWM)
{
    PWM_Result ret = E_PWM_FAIL;
    PWM_DBG_INFO("%s(%u, 0x%08X)\r\n", __FUNCTION__, u8IndexPWM, (int)u32PeriodPWM);

    do{
        HAL_PWM_Period(u8IndexPWM, u32PeriodPWM);
        ret = E_PWM_OK;
    }while(0);

    return ret;
}

//-------------------------------------------------------------------------------------------------
/// Description: Set the Duty of the specific pwm
/// @param u8IndexPWM    \b which pwm is setting
/// @param u16DutyPWM    \b the 18-bit Duty value
/// @return E_PWM_OK : succeed
/// @return E_PWM_FAIL : fail before timeout or illegal parameters
//-------------------------------------------------------------------------------------------------
PWM_Result MDrv_PWM_DutyCycle(PWM_ChNum u8IndexPWM, U32 u32DutyPWM)
{
    PWM_Result ret = E_PWM_FAIL;
    PWM_DBG_INFO("%s(%u, 0x%08X)\r\n", __FUNCTION__, u8IndexPWM, (int)u32DutyPWM);

    do{
        HAL_PWM_DutyCycle(u8IndexPWM, u32DutyPWM);
        ret = E_PWM_OK;
    }while(0);

    return ret;
}

//-------------------------------------------------------------------------------------------------
/// Description: Set the Unit_Div of the pwm
/// @param u16UnitDivPWM    \b the Unit Div value
/// @return E_PWM_OK : succeed
/// @return E_PWM_FAIL : fail before timeout or illegal parameters
//-------------------------------------------------------------------------------------------------
PWM_Result MDrv_PWM_UnitDiv(U16 u16UnitDivPWM)
{
    PWM_Result ret = E_PWM_OK;
    PWM_DBG_INFO("%s(0x%04X)\r\n", __FUNCTION__, u16UnitDivPWM);

    if(!HAL_PWM_UnitDiv(u16UnitDivPWM))
    {
        ret = E_PWM_FAIL;
    }
    return ret;
}

//-------------------------------------------------------------------------------------------------
/// Description: Set the Div of the specific pwm
/// @param u8IndexPWM    \b which pwm is setting
/// @param u16DivPWM    \b the 16-bit div value
//-------------------------------------------------------------------------------------------------
void MDrv_PWM_Div(PWM_ChNum u8IndexPWM, U16 u16DivPWM)
{
    PWM_DBG_INFO("%s(%u, 0x%04X)\r\n", __FUNCTION__, u8IndexPWM, u16DivPWM);
    HAL_PWM_Div(u8IndexPWM, u16DivPWM);
}

//-------------------------------------------------------------------------------------------------
/// Description:  Set the Polarity of the specific pwm
/// @param u8IndexPWM    \b which pwm is setting
/// @param bPolPWM    \b True/False for Inverse/Non-inverse
//-------------------------------------------------------------------------------------------------
void MDrv_PWM_Polarity(PWM_ChNum u8IndexPWM, BOOL bPolPWM)
{
    PWM_DBG_INFO("%s(%u, %u)\r\n", __FUNCTION__, u8IndexPWM, bPolPWM);
    HAL_PWM_Polarity(u8IndexPWM, bPolPWM);
}

//-------------------------------------------------------------------------------------------------
/// Description: Set the Vsync Double buffer of the specific pwm
/// @param u8IndexPWM    \b which pwm is setting
/// @param bVdbenPWM    \b True/False for Enable/Disable
//-------------------------------------------------------------------------------------------------
void MDrv_PWM_Vdben(PWM_ChNum u8IndexPWM, BOOL bVdbenPWM)
{
    PWM_DBG_INFO("%s(%u, %u)\r\n", __FUNCTION__, u8IndexPWM, bVdbenPWM);
    HAL_PWM_VDBen(u8IndexPWM, bVdbenPWM);
}

//-------------------------------------------------------------------------------------------------
/// Description: Set the Hsync reset of the specific pwm
/// @param u8IndexPWM    \b which pwm is setting
/// @param bRstPWM    \b True/False for Enable/Disable
//-------------------------------------------------------------------------------------------------
void MDrv_PWM_ResetEn(PWM_ChNum u8IndexPWM, BOOL bRstPWM)
{
    PWM_DBG_INFO("%s(%u, %u)\r\n", __FUNCTION__, u8IndexPWM, bRstPWM);
    HAL_PWM_Vrest(u8IndexPWM, bRstPWM);
}

//-------------------------------------------------------------------------------------------------
/// Description:  Set the Double buffer of the specific pwm
/// @param u8IndexPWM    \b which pwm is setting
/// @param bDbenPWM    \b True/False for Enable/Disable
//-------------------------------------------------------------------------------------------------
void MDrv_PWM_Dben(PWM_ChNum u8IndexPWM, BOOL bDbenPWM)
{
    PWM_DBG_INFO("%s(%u, %u)\r\n", __FUNCTION__, u8IndexPWM, bDbenPWM);
    HAL_PWM_DBen(u8IndexPWM, bDbenPWM);
}

void MDrv_PWM_IMPULSE_EN(PWM_ChNum u8IndexPWM, BOOL bdbenPWM)
{
    PWM_DBG_INFO("%s(%u, %u)\r\n", __FUNCTION__, u8IndexPWM, bdbenPWM);
    HAL_PWM_IMPULSE_EN(u8IndexPWM, bdbenPWM);
}

void MDrv_PWM_ODDEVEN_SYNC(PWM_ChNum u8IndexPWM, BOOL bdbenPWM)
{
    PWM_DBG_INFO("%s(%u, %u)\r\n", __FUNCTION__, u8IndexPWM, bdbenPWM);
    HAL_PWM_ODDEVEN_SYNC(u8IndexPWM, bdbenPWM);
}

//-------------------------------------------------------------------------------------------------
/// Description:  Set the Rst Mux of the specific pwm
/// @param u8IndexPWM    \b which pwm is setting
/// @param bMuxPWM    \b True/False for Enable/Disable
//-------------------------------------------------------------------------------------------------
void MDrv_PWM_RstMux(PWM_ChNum u8IndexPWM, BOOL bMuxPWM)
{
    PWM_DBG_INFO("%s(%u, %u)\r\n", __FUNCTION__, u8IndexPWM, bMuxPWM);
    HAL_PWM_RstMux(u8IndexPWM, bMuxPWM);
}

//-------------------------------------------------------------------------------------------------
/// Description: Set the Rst_Cnt of the specific pwm
/// @param u8IndexPWM    \b which pwm is setting
/// @param u8RstCntPWM    \b the Rst_Cnt value
//-------------------------------------------------------------------------------------------------
void MDrv_PWM_RstCnt(PWM_ChNum u8IndexPWM, U8 u8RstCntPWM)
{
    PWM_DBG_INFO("%s(%u, 0x%02X)\r\n", __FUNCTION__, u8IndexPWM, u8RstCntPWM);
    HAL_PWM_RstCnt(u8IndexPWM, u8RstCntPWM);
}

//-------------------------------------------------------------------------------------------------
/// Description: Set the Bypass Unit of the specific pwm
/// @param u8IndexPWM    \b which pwm is setting
/// @param bBypassPWM    \b True/False for Enable/Disable
//-------------------------------------------------------------------------------------------------
void MDrv_PWM_BypassUnit(PWM_ChNum u8IndexPWM, BOOL bBypassPWM)
{
    PWM_DBG_INFO("%s(%u, %u)\r\n", __FUNCTION__, u8IndexPWM, bBypassPWM);
    HAL_PWM_BypassUnit(u8IndexPWM, bBypassPWM);
}

//-------------------------------------------------------------------------------------------------
/// Description: Counter mode for PWM0 and PWM1
/// @param u8CntModePWM    \b Cnt Mode
/// @return E_PWM_OK : succeed
/// @return E_PWM_FAIL : fail before timeout or illegal parameters
/// @note    \n
///     11: PWM1 donate internal divider to PWM0    \n
///     10: PWM0 donate internal divider to PWM1    \n
///     0x: Normal mode    \n
//-------------------------------------------------------------------------------------------------
PWM_Result MDrv_PWM01_CntMode(PWM_CntMode u8CntModePWM)
{
    PWM_Result ret = E_PWM_FAIL;
    PWM_DBG_INFO("%s(%u)\r\n", __FUNCTION__, u8CntModePWM);
    if( HAL_PWM01_CntMode(u8CntModePWM) )
    {
		ret = E_PWM_OK;
    }
    return ret;
}

//-------------------------------------------------------------------------------------------------
/// Description: Counter mode for PWM2 and PWM3
/// @param u8CntModePWM    \b Cnt Mode
/// @return E_PWM_OK : succeed
/// @return E_PWM_FAIL : fail before timeout or illegal parameters
/// @note    \n
///     11: PWM3 donate internal divider to PWM2    \n
///     10: PWM2 donate internal divider to PWM3    \n
///     0x: Normal mode    \n
//-------------------------------------------------------------------------------------------------
PWM_Result MDrv_PWM23_CntMode(PWM_CntMode u8CntModePWM)
{
    PWM_Result ret = E_PWM_FAIL;
    PWM_DBG_INFO("%s(%u)\r\n", __FUNCTION__, u8CntModePWM);
    if( HAL_PWM23_CntMode(u8CntModePWM) )
    {
		ret = E_PWM_OK;
    }
    return ret;
}

//-------------------------------------------------------------------------------------------------
/// Description: Counter mode for PWM6 and PWM7
/// @param u8CntModePWM    \b Cnt Mode
/// @return E_PWM_OK : succeed
/// @return E_PWM_FAIL : fail before timeout or illegal parameters
/// @note    \n
///     11: PWM7 donate internal divider to PWM6    \n
///     10: PWM6 donate internal divider to PWM7    \n
///     0x: Normal mode    \n
//-------------------------------------------------------------------------------------------------
PWM_Result MDrv_PWM67_CntMode(PWM_CntMode u8CntModePWM)
{
    PWM_Result ret = E_PWM_FAIL;
    PWM_DBG_INFO("%s(%u)\r\n", __FUNCTION__, u8CntModePWM);
    if( HAL_PWM67_CntMode(u8CntModePWM) )
    {
		ret = E_PWM_OK;
    }
    return ret;
}

//-------------------------------------------------------------------------------------------------
/// Description: Set the Shift of the specific pwm
/// @param u8IndexPWM    \b which pwm is setting
/// @param u16DutyPWM    \b the 18-bit Shift value
/// @return E_PWM_OK : succeed
/// @return E_PWM_FAIL : fail before timeout or illegal parameters
//-------------------------------------------------------------------------------------------------
PWM_Result MDrv_PWM_Shift(PWM_ChNum u8IndexPWM, U32 u32DutyPWM)
{
    PWM_Result ret = E_PWM_FAIL;
    PWM_DBG_INFO("%s(%u, 0x%08X)\r\n", __FUNCTION__, u8IndexPWM, (int) u32DutyPWM);
    if ( HAL_PWM_Shift(u8IndexPWM, u32DutyPWM) )
    {
        ret = E_PWM_OK;
    }
    return ret;
}

//-------------------------------------------------------------------------------------------------
/// Description: PWM Property Get
/// @param PWM_Property  \b property check
/// @param PWM_ChNum     \b pwm channel
/// @param u32PWMVar     \b pwm property value get
/// @return E_PWM_OK : succeed
/// @return E_PWM_FAIL : fail before timeout or illegal parameters
//-------------------------------------------------------------------------------------------------
PWM_Result MDrv_PWM_GetProperty(PWM_Property eProperty, PWM_ChNum u8IndexPWM, U32 *u32PWMVar)
{
    switch(eProperty)
    {
        case E_PWM_GetPeriod:
            *u32PWMVar = HAL_PWM_GetPeriod(u8IndexPWM);
            break;
        case E_PWM_GetDutyCycle:
            *u32PWMVar = HAL_PWM_GetDutyCycle(u8IndexPWM);
            break;
        case E_PWM_GetPolarity:
            *u32PWMVar = (U32)HAL_PWM_GetPolarity(u8IndexPWM);
            break;
        case E_PWM_GetOen:
            *u32PWMVar = (U32)HAL_PWM_GetOen(u8IndexPWM);
            break;
        default:
            return E_PWM_FAIL;
    }
    return E_PWM_OK;
}


void MDrv_PWM_Nvsync(PWM_ChNum u8IndexPWM, BOOL bNvsPWM)
{
    PWM_DBG_INFO("%s(%u, %u)\r\n", __FUNCTION__, u8IndexPWM, bNvsPWM);
    HAL_PWM_Nvsync(u8IndexPWM, bNvsPWM);
}


void MDrv_PWM_Align(PWM_ChNum u8IndexPWM, BOOL bAliPWM)
{
    PWM_DBG_INFO("%s(%u, %u)\r\n", __FUNCTION__, u8IndexPWM, bAliPWM);
    HAL_PWM_Align(u8IndexPWM, bAliPWM);
}

void MDrv_PWM_AutoCorrect(PWM_ChNum u8IndexPWM, BOOL bCorrect_en)
{
    PWM_DBG_INFO("%s(%u, %u)\r\n", __FUNCTION__, u8IndexPWM, bCorrect_en);
    HAL_PWM_AutoCorrect(u8IndexPWM, bCorrect_en);
}


//-------------PM BASE---------------
void MDrv_PM_PWM_Enable_Ex(PM_PWM_ChNum index, BOOL b_en)
{
    HAL_PM_PWM_Enable(index, b_en);
}

void MDrv_PM_PWM_Period_Ex(PM_PWM_ChNum index, U16 u16PeriodPWM)
{
    HAL_PM_PWM_Period(index, u16PeriodPWM);
}

void MDrv_PM_PWM_DutyCycle_Ex(PM_PWM_ChNum index, U16 u16DutyPWM)
{
    HAL_PM_PWM_DutyCycle(index, u16DutyPWM);
}

void MDrv_PM_PWM_Div_Ex(PM_PWM_ChNum index, U8 u8DivPWM)
{
    HAL_PM_PWM_Div(index, u8DivPWM);
}

void MDrv_PM_PWM_Polarity_Ex(PM_PWM_ChNum index, BOOL bPolPWM)
{
    HAL_PM_PWM_Polarity(index, bPolPWM);
}

void MDrv_PM_PWM_Dben_Ex(PM_PWM_ChNum index, BOOL bdbenPWM)
{
    HAL_PM_PWM_DBen(index, bdbenPWM);
}

void MDrv_PM_PWM_Reset_Ex(PM_PWM_ChNum index, BOOL bresetPWM)
{
    HAL_PM_PWM_Reset(index, bresetPWM);
}

#if 0
void MDrv_PM_PWM_Enable(void)
{
    HAL_PM_PWM_Enable();
}

void MDrv_PM_PWM_Period(U16 u16PeriodPWM)
{
   HAL_PM_PWM_Period(u16PeriodPWM);
}

void MDrv_PM_PWM_DutyCycle(U16 u16DutyPWM)
{
   HAL_PM_PWM_DutyCycle(u16DutyPWM);
}

void MDrv_PM_PWM_Div(U8 u8DivPWM)
{
    HAL_PM_PWM_Div(u8DivPWM);
}

void MDrv_PM_PWM_Polarity(BOOL bPolPWM)
{
    HAL_PM_PWM_Polarity(bPolPWM);
}

void MDrv_PM_PWM_Dben(BOOL bDbenPWM)
{
    HAL_PM_PWM_DBen(bDbenPWM);
}
#endif
