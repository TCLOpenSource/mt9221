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






#include "cdn_errno.h"
#include "cdn_stdint.h"

#include "cpdi.h"

#include "cps_v2.h"
#include "cpdi_core_driver.h"

CPDI_OBJ CPDDrv = {

    CPDI_Probe   ,  /* probe    */
    CPDI_Init    ,  /* init     */

    CPDI_Destroy,  /* destroy  */
    CPDI_Start  ,  /* start    */
    CPDI_Stop   ,  /* stop     */
    CPDI_Isr    ,  /* isr      */

    /****************************************************************************/
    /* PHYSICAL LAYER TRAINING INFORMATION                                      */
    /****************************************************************************/

    CPDI_IsLinkTrainingComplete                            ,
    CPDI_GetLinkTrainingState                              ,
    CPDI_GetLinkTrainingDirection                          ,
    CPDI_IsCoreStrappedAsEpOrRp                            ,

    /****************************************************************************/
    /* PHYSICAL LAYER LANE COUNT AND LINK SPEED INFORMATION                     */
    /****************************************************************************/



    CPDI_GetNegotiatedLinkSpeed                            ,



    /****************************************************************************/
    /* PHYSICAL LAYER REMOTE INFO RECEIVED DURING TRAINING                      */
    /****************************************************************************/

    CPDI_GetReceivedLinkId                                 ,


    CPDI_IsRemoteLinkwidthUpconfigurable                   ,


    /****************************************************************************/
    /* PHYSICAL_LAYER INFO SENT DURING TRAINING                                 */
    /****************************************************************************/

    CPDI_AccessTransmittedLinkId                           ,

    /****************************************************************************/
    /* VENDOR AND SUBSYSTEM VENDOR IDs                                          */
    /****************************************************************************/

    CPDI_AccessVendorIdSubsystemVendorId                   ,

    /****************************************************************************/
    /* TIMING PARAMS INCLUDE TIMEOUTS, DELAYS, LATENCY SETTINGS AND SCALES      */
    /****************************************************************************/

    CPDI_AccessTimingParams                                ,
    CPDI_AccessL0sTimeout                                  ,

    /****************************************************************************/
    /* TRANSITION INTO L0S                                                      */
    /****************************************************************************/

    CPDI_DisableRpTransitionToL0s                         ,



    /****************************************************************************/
    /* RP RETRAINING CONTROL                                                    */
    /****************************************************************************/




    /****************************************************************************/
    /* TRANSMIT AND RECEIVE CREDIT LIMITS AND UPDATE INTERVAL ROUTINES          */
    /****************************************************************************/

    CPDI_AccessCreditLimitSettings                         ,
    CPDI_AccessTransmitCreditUpdateIntervalSettings        ,

    /****************************************************************************/
    /* BAR APERTURE AND CONTROL CONFIGURATION                                   */
    /****************************************************************************/
    CPDI_AccessRootPortBarApertureSetting               ,
    CPDI_AccessRootPortBarControlSetting                ,
    CPDI_AccessRootPortType1ConfigSetting               ,

    CPDI_ControlRootPortBarCheck                        ,

    /****************************************************************************/
    /* DEBUG CONTROL                                                            */
    /****************************************************************************/

    CPDI_ControlRpMasterLoopback                           ,
    CPDI_AccessDebugMux                                    ,
    CPDI_ControlDebugParams                                ,

    /****************************************************************************/
    /* COUNT STATISTICS ROUTINES                                                */
    /****************************************************************************/

    CPDI_AccessSavedCountValues                            ,

    /****************************************************************************/
    /* EVENT MASKING ROUTINES                                                   */
    /****************************************************************************/

    CPDI_ControlReportingOfAllPhyErrors                    ,
    CPDI_ControlTxSwing                                    ,
    CPDI_ControlMaskingOfLocalInterrupts                   ,
    CPDI_AreThereLocalErrors                               ,
    CPDI_IsLocalError                                      ,
    CPDI_ResetLocalErrorStatusCondition                    ,




/****************************************************************************/
/* AXI Wrapper Functions                                                    */
/****************************************************************************/

    CPDI_UpdateObWrapperTrafficClass                       ,
    CPDI_SetupObWrapperMemIoAccess                         ,
    CPDI_SetupObWrapperMessageAccess                       ,
    CPDI_SetupObWrapperConfigAccess                        ,
    CPDI_SetupIbRootPortAddrTranslation                    ,


    CPDI_DoConfigRead                                      ,
    CPDI_DoConfigWrite                                     ,
    CPDI_DoAriConfigRead                                   ,
    CPDI_DoAriConfigWrite                                  ,
    CPDI_EnableBarAccess                                   ,
    CPDI_GetRootPortBAR                                    ,
    CPDI_EnableRpMemBarAccess                              ,



};

CPDI_OBJ *CPDI_GetInstance(void) {
    return &CPDDrv;
}
