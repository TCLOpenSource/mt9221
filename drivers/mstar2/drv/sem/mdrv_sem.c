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

#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/time.h>

#include "mdrv_sem.h"
#include "mhal_sem.h"

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
static SEM_DbgLvl _gSEMDbgLevel =   E_SEM_DBGLVL_ERROR;
static bool _gbSEMInitialized   =   false;
#define TAG_SEM "SEM"
#define SEM_MAX_NUM                 SEM_ENTRY_NUMBER

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------
#define SEM_PRINT()             if (_gSEMDbgLevel >= E_SEM_DBGLVL_ALL) \
                                        {printk("\t====   %s   ====\n", __FUNCTION__);}
#define SEM_DEBUG(x, args...)   if (_gSEMDbgLevel >= E_SEM_DBGLVL_INFO ) \
                                        {printk("[SEM USER DEBUG][%s]: ", __FUNCTION__);printk(x, ##args);}
#define SEM_ERROR(x, args...)   if (_gSEMDbgLevel >= E_SEM_DBGLVL_ERROR) \
                                        {printk("[SEM USER ERR][%s]: ", __FUNCTION__);printk(x, ##args);}
#define SEM_WARN(x, args...)    if (_gSEMDbgLevel >= E_SEM_DBGLVL_WARNING) \
                                        {printk("[SEM USER WARN][%s]: ", __FUNCTION__);printk(x, ##args);}

#define SEM_PM51_ID                    0x01
#define SEM_AEON_ID                    0x02
#define SEM_ARM_MIPS_ID                0x03
#define SEM_TEE_ID                     0x02

#define SEM_RESOURCE_ID                SEM_ARM_MIPS_ID


static struct mutex _s32SEMMutex[SEM_MAX_NUM + 1] = {-1};
static char _SEMMutexName[SEM_MAX_NUM][13] = {"SEMMUTEX0","SEMMUTEX1","SEMMUTEX2","SEMMUTEX3","SEMMUTEX4","SEMMUTEX5","SEMMUTEX6","SEMMUTEX7",\
                                              "SEMMUTEX8","SEMMUTEX9","SEMMUTEX10","SEMMUTEX11","SEMMUTEX12","SEMMUTEX13","SEMMUTEX14","SEMMUTEX15"};

//define mutex
#define SEM_MUTEX_CREATE(_index)            mutex_init(&_s32SEMMutex[_index])
#define SEM_MUTEX_LOCK_WAIT_FOREVER(_index) mutex_lock(&_s32SEMMutex[_index])
#define SEM_MUTEX_TRYLOCK(_index)           mutex_trylock(&_s32SEMMutex[_index])
#define SEM_MUTEX_UNLOCK(_index)            mutex_unlock(&_s32SEMMutex[_index])
#define SEM_MUTEX_DELETE(_index)

//-------------------------------------------------------------------------------------------------
// Gloabal Functions
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
/// Attempt to get mmio base and create mutex
/// @return true : succeed
/// @return false : fail
//-------------------------------------------------------------------------------------------------
bool MDrv_mstar2_SEM_Init(void)
{
    int intIndex;

    if (_gbSEMInitialized)
        return true;

    for (intIndex = 0; intIndex < SEM_MAX_NUM; intIndex++) {
        SEM_MUTEX_CREATE(intIndex);
        SEM_DEBUG("_s32SEMMutex[%ld] = %ld\n", intIndex, _s32SEMMutex[intIndex]);
    }

    _gbSEMInitialized = true;

    return true;
}

bool MDrv_mstar2_SEM_Get_Resource(u8 u8SemID, u16 u16ResId)
{
    if (!_gbSEMInitialized) {
        SEM_WARN("%s is called before init\n", __FUNCTION__);
        return false;
    }

    SEM_WARN("Only use in verify or debug (Deprecated) Please use MDrv_mstar2_SEM_Lock\n");

    return HAL_SEM_Get_Resource(u8SemID, u16ResId);
}

bool MDrv_mstar2_SEM_Free_Resource(u8 u8SemID, u16 u16ResId)
{
    if (!_gbSEMInitialized) {
        SEM_WARN("%s is called before init\n", __FUNCTION__);
        return false;
    }

    SEM_WARN("Only use in verify or debug (Deprecated) Please use MDrv_mstar2_SEM_Unlock\n");

    return HAL_SEM_Free_Resource(u8SemID, u16ResId);
}


bool MDrv_mstar2_SEM_Reset_Resource(u8 u8SemID)
{
    if (!_gbSEMInitialized) {
        SEM_WARN("%s is called before init\n", __FUNCTION__);
        return false;
    }

    SEM_WARN("Only use in verify or debug (Deprecated)\n");

    return HAL_SEM_Reset_Resource(u8SemID);
}

bool MDrv_mstar2_SEM_Get_ResourceID(u8 u8SemID, u16* pu16ResId)
{
    if (!_gbSEMInitialized) {
        SEM_WARN("%s is called before init\n", __FUNCTION__);
        return false;
    }

    SEM_WARN("Only use in verify or debug (Deprecated)\n");

    return HAL_SEM_Get_ResourceID(u8SemID, pu16ResId);
}

u32 MDrv_mstar2_SEM_Get_Num(void)
{
    return HAL_SEM_Get_Num();
}

//-------------------------------------------------------------------------------------------------
/// Attempt to lock a hardware semaphore
/// @param  SemId       \b IN: hardware semaphore ID
/// @param  u32WaitMs   \b IN: 0 ~ SEM_WAIT_FOREVER: suspend time (ms) if the mutex is locked
/// @return true : succeed
/// @return false : fail
//-------------------------------------------------------------------------------------------------
bool MDrv_mstar2_SEM_Lock(eSemId SemId, u32 u32WaitMs)
{
    short s16SemId;
    bool bRet;
    u32  u32Interval;
    struct timespec stSysOldTime, stCurrentTime;

    if (!_gbSEMInitialized) {
        SEM_WARN("%s is called before init\n", __FUNCTION__);
        return false;
    }

    SEM_DEBUG("Lock enum = %d\n", SemId);
    SEM_DEBUG("Lock u32WaitMs = %ld\n", u32WaitMs);

    s16SemId = HAL_SEM_GetSemId(SemId);

    if (s16SemId < 0) {
        SEM_ERROR("Enum device sem not support! SemId = %d\n", SemId);
        return false;
    }

    SEM_DEBUG("Lock SemId = %d\n", s16SemId);

    getnstimeofday(&stSysOldTime);
    u32Interval = 0;

    /*blocking*/
    if (u32WaitMs == SEM_WAIT_FOREVER) {
        SEM_WARN("TryLock WAIT FOREVER\n");
        SEM_MUTEX_LOCK_WAIT_FOREVER(s16SemId);
        do {
            bRet = HAL_SEM_Get_Resource((u8)s16SemId, SEM_RESOURCE_ID);
        } while(bRet!= true);
    } else { /*blocking with timeout*/
        do {
            bRet = SEM_MUTEX_TRYLOCK(s16SemId);
            getnstimeofday(&stCurrentTime);
            u32Interval = (stCurrentTime.tv_sec - stSysOldTime.tv_sec) * 1000
                        + (stCurrentTime.tv_nsec - stSysOldTime.tv_nsec) / 1000000;

            if(u32Interval > 10 && u32WaitMs - u32Interval > 10)
                mdelay(10);
        } while(bRet != true && u32Interval < u32WaitMs);

        if (bRet == false) {
           SEM_ERROR("Obtain mutex %s failed\n", _SEMMutexName[s16SemId]);
        } else {
            do {
                bRet = HAL_SEM_Get_Resource((u8)s16SemId, SEM_RESOURCE_ID);
                getnstimeofday(&stCurrentTime);
                u32Interval = (stCurrentTime.tv_sec - stSysOldTime.tv_sec) * 1000
                            + (stCurrentTime.tv_nsec - stSysOldTime.tv_nsec) / 1000000;

                if(u32Interval > 10 && u32WaitMs - u32Interval > 10)
                    mdelay(10);
            } while ((bRet!= true) && (u32Interval < u32WaitMs));

            if (bRet == false) {
                SEM_MUTEX_UNLOCK(s16SemId);
                SEM_ERROR("Obtain hardware semaphore %d failed, timeout=%ld\n", s16SemId, u32WaitMs);
            }
        }
    }

    return bRet;
}

//-------------------------------------------------------------------------------------------------
/// Attempt to unlock a hardware semaphore
/// @param  SemId       \b IN: hardware semaphore ID
/// @return true : succeed
/// @return false : fail
//-------------------------------------------------------------------------------------------------
bool MDrv_mstar2_SEM_Unlock(eSemId SemId)
{
    short s16SemId;
    bool bRet;

    if (!_gbSEMInitialized) {
        SEM_WARN("%s is called before init\n", __FUNCTION__);
        return false;
    }

    SEM_DEBUG("Unlock enum = %d\n", SemId);
    s16SemId = HAL_SEM_GetSemId(SemId);

    if (s16SemId < 0) {
        SEM_ERROR("Enum device sem not support! SemId = %d\n", SemId);
        return false;
    }

    SEM_DEBUG("Unlock SemId = %d\n", s16SemId);

    bRet = HAL_SEM_Free_Resource((u8)s16SemId, SEM_RESOURCE_ID);

    if (bRet == false)
        SEM_ERROR("Release hardware semaphore %d failed\n", s16SemId);

    SEM_MUTEX_UNLOCK(s16SemId);

    return bRet;
}

//-------------------------------------------------------------------------------------------------
/// Attempt to delete a hardware semaphore
/// @param  SemId       \b IN: hardware semaphore ID
/// @return true : succeed
/// @return false : fail
//-------------------------------------------------------------------------------------------------
bool MDrv_mstar2_SEM_Delete(eSemId SemId)
{
    short s16SemId;
    bool bRet;

    if (!_gbSEMInitialized) {
        SEM_WARN("%s is called before init\n", __FUNCTION__);
        return false;
    }

    SEM_DEBUG("Delete enum = %d\n", SemId);
    s16SemId = HAL_SEM_GetSemId(SemId);

    if (s16SemId < 0) {
        SEM_ERROR("Enum device sem not support! SemId = %d\n", SemId);
        return false;
    }

    SEM_DEBUG("Delete SemId = %d\n", s16SemId);

    SEM_MUTEX_UNLOCK(s16SemId);

    bRet = HAL_SEM_Reset_Resource((u8)s16SemId);

    if (bRet == false)
        SEM_ERROR("Reset hardware semaphore %d failed\n", s16SemId);

    return bRet;
}
//-------------------------------------------------------------------------------------------------
/// Attempt to change driver debug level
/// @param  eLevel       \b IN: debug level
/// @return true : succeed
/// @return false : fail
//-------------------------------------------------------------------------------------------------
bool MDrv_mstar2_SEM_SetDbgLevel(SEM_DbgLvl eLevel)
{
    _gSEMDbgLevel = eLevel;

    return true;
}
