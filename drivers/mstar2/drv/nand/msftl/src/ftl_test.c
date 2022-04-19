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
 * Copyright 2008, Freescale Semiconductor, Inc
 * Andy Fleming
 *
 * Based vaguely on the Linux code
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */


#include "ftl_api.h"

U8 *pu8_FtlTestDatBuf = (U8*)DMA_W_ADDR; 
U8 *pu8_FtlTestTmpBuf = (U8*)DMA_R_ADDR; 
U8 *pu8_FtlTestRddBuf = (U8*)DMA_W_SPARE_ADDR;

// =============================================
#if defined(NC_SEL_FCIE5) && NC_SEL_FCIE5

static U16 au16_reg[3][256], sgu16_i=0xFFFF;

void NC_Test_Reg717274_Set(void)
{
    if(0xFF < sgu16_i)
        return; 
    
    au16_reg[0][sgu16_i] = REG(NC_RPT_CNT);
    au16_reg[1][sgu16_i] = REG(NC_TR_RPT_CNT);
    au16_reg[2][sgu16_i] = REG(NC_REMAIN_RPT_CNT);
    sgu16_i++;
}

void NC_Test_Reg717274_Get(void)
{
    U16 u16_i;

    if(0x100 < sgu16_i)
        return; 

    for(u16_i=0; u16_i<sgu16_i; u16_i++)
    {
        ftl_dbg(0,0,"[%03u]: %4Xh + %4Xh = %4Xh, %4Xh \n", 
            u16_i, au16_reg[1][u16_i], au16_reg[2][u16_i], 
            au16_reg[1][u16_i]+au16_reg[2][u16_i], au16_reg[0][u16_i]);
    }
}

U32 NC_Test_Reg717274(U16 u16_PBA) // RepeatMode ECC correct
{
    U32 u32_err;
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    
    while(0==drvNAND_IsGoodBlk(u16_PBA))
    {
        ftl_dbg(FTL_DBG_LV_LOG, 1, "bad blk: %Xh \n", u16_PBA);
        u16_PBA++;
    }

    u32_err = NC_EraseBlk(u16_PBA<<pNandDrv->u8_BlkPageCntBits);
    if(UNFD_ST_SUCCESS != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"erase %Xh blk fail: %Xh \n", u16_PBA, u32_err);
        return u32_err;
    }

    sgu16_i = 0;
    // put 
    //    - NC_Test_Reg717274_Set
    //    - NC_Test_Reg717274_Get
    // into NC_WaitComplete
    NC_ReadPages(u16_PBA<<pNandDrv->u8_BlkPageCntBits, 
        pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, pNandDrv->u16_BlkPageCnt);
    sgu16_i = 0xFFFF;
    
    return u32_err;        
}

#endif
// ============================================




// ============================================
// lifetime test
// ============================================
#define TEST_ErrBit

typedef struct _BLK_ERR_BITS
{
    U16 u16_PBA, u16_Page, u16_ErrBitCnt; 
    
}BLK_ERR_BITS, *P_BLK_ERR_BITS;

#define IF_FORCE_LFSR      0
#define PECYCLE            1000

// --------------------------
// 1. use 6 blocks (start from the argument of the u16_PBA): 
//    - preset 3 PE cycles: 1, 1K, 2K
//    - each PE cycle: 1 blk uses FB, 1 blk uses LP only.
// 2. check ErrBit status:
//    - 6 blocks, print the Max ErrBitCnt, and the PageIdx.
//    - for block of FB, print LP/HP ErrBitCnt
// --------------------------
void NC_Test_ErrBitStatus_R(U16 u16_PBA)
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U16 u16_i, u16_p, u16_PECnt, au16_PBA[3][2]; // [1,1K,2K] PE cycles,  [FB, LP]
    U32 u32_err, u32_i, u32_j, BlkRow;
    BLK_ERR_BITS a_BlkErrBits[3][2][2]; // [1,1K,2K] [FB,LP] [LP,HP]
    U8  u8_RequireRandomizer = pNandDrv->u8_RequireRandomizer;

    #if IF_FORCE_LFSR  // test with LFSR
    pNandDrv->u8_RequireRandomizer = 1;

    if(pNandDrv->u8_RequireRandomizer)
        NC_EnableLFSR();
    #endif    

    memset(a_BlkErrBits, 0, sizeof(BLK_ERR_BITS)*3*2*2);
    
    u16_i=0;
    while(u16_i < 6)
    {
        #if IF_FORCE_LFSR 
        if(0)//==drvNAND_IsGoodBlk(u16_PBA)) // when LFSR enabled, need BBT.
        #else
        if(0==drvNAND_IsGoodBlk(u16_PBA)) // read 4 pages: 0,1,-2,-1, can not set LFSR.
        #endif
        {
            ftl_dbg(FTL_DBG_LV_LOG, 1, "bad blk: %Xh \n", u16_PBA);
            u16_PBA++;
        }
        else
        {   au16_PBA[u16_i/2][u16_i%2] = u16_PBA;
            u16_PBA++;
            u16_i++;
        }
    }   
    
    ftl_dbg(FTL_DBG_LV_LOG,0," ==============================\n");
    for(u16_i=0; u16_i<6; u16_i++)
    {
        u16_PECnt = PECYCLE*(u16_i/2)+1; // PE cycles
        
        if(0==u16_i%2) // FullBlk
        {
            ftl_dbg(0,0,"PBA %4Xh: %4u PE, FullBlk \n", au16_PBA[u16_i/2][u16_i%2], u16_PECnt);
            
            a_BlkErrBits[u16_i/2][u16_i%2][0].u16_PBA = au16_PBA[u16_i/2][u16_i%2];
            a_BlkErrBits[u16_i/2][u16_i%2][1].u16_PBA = au16_PBA[u16_i/2][u16_i%2];
        }
        else // LP only
        {
            ftl_dbg(0,0,"PBA %4Xh: %4u PE, LP-Only \n", au16_PBA[u16_i/2][u16_i%2], u16_PECnt);
            
            a_BlkErrBits[u16_i/2][u16_i%2][0].u16_PBA = au16_PBA[u16_i/2][u16_i%2];
        }
    }
    ftl_dbg(FTL_DBG_LV_LOG,0," ==============================\n\n");
        
    // --------------------------
    // reading PBAs
    u32_i=0; u32_j=0;
    while(1)
    for(u16_i=0; u16_i<6; u16_i++)
    {
        BlkRow= au16_PBA[u16_i/2][u16_i%2]<<pNandDrv->u8_BlkPageCntBits;
        // reading
        for(u16_p=0; u16_p<pNandDrv->u16_BlkLowPCnt; u16_p++)
        {
            u32_err = NC_ReadPages(
                BlkRow + ga_tPairedPageMap[u16_p].u16_LSB,
                pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, 1);
            if(u32_err != FTL_OK)
            {
                ftl_dbg(FTL_DBG_LV_ERR,1,"read fail: %Xh PBA \n", au16_PBA[u16_i/2][u16_i%2]);
                goto LABEL_END;
            }                
            if(NC_GetECCBits() > a_BlkErrBits[u16_i/2][u16_i%2][0].u16_ErrBitCnt)
            {
                a_BlkErrBits[u16_i/2][u16_i%2][0].u16_ErrBitCnt = NC_GetECCBits();
                a_BlkErrBits[u16_i/2][u16_i%2][0].u16_Page = ga_tPairedPageMap[u16_p].u16_LSB;
            }

            if(ga_tPairedPageMap[u16_p].u16_MSB == 0xFFFF)
                continue;
            
            if(0==u16_i%2) // FullBlk
            {
                u32_err = NC_ReadPages(
                    BlkRow + ga_tPairedPageMap[u16_p].u16_MSB,
                    pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, 1);
                if(u32_err != FTL_OK)
                {
                    ftl_dbg(FTL_DBG_LV_ERR,1,"read fail: %Xh PBA \n", au16_PBA[u16_i/2][u16_i%2]);
                    goto LABEL_END;
                }                
                if(NC_GetECCBits() > a_BlkErrBits[u16_i/2][u16_i%2][1].u16_ErrBitCnt)
                {
                    a_BlkErrBits[u16_i/2][u16_i%2][1].u16_ErrBitCnt = NC_GetECCBits();
                    a_BlkErrBits[u16_i/2][u16_i%2][1].u16_Page = ga_tPairedPageMap[u16_p].u16_MSB;
                }
            }
        }   
        
        // read count        
        if(0 == u32_i%0x400)
        {
            ftl_dbg(0,0,"---------------------\n");
            ftl_dbg(0,0,"LoopCnt: %8Xh %8Xh \n", u32_j, u32_i);
            
            for(u16_p=0; u16_p<6; u16_p++)
            {
                ftl_dbg(0,0,"PBA:%3Xh LP:%3u EBits:%2u \n", 
                    a_BlkErrBits[u16_p/2][u16_p%2][0].u16_PBA, 
                    a_BlkErrBits[u16_p/2][u16_p%2][0].u16_Page,
                    a_BlkErrBits[u16_p/2][u16_p%2][0].u16_ErrBitCnt);
                if(0==u16_p%2)
                    ftl_dbg(0,0,"PBA:%3Xh HP:%3u EBits:%2u \n", 
                        a_BlkErrBits[u16_p/2][u16_p%2][1].u16_PBA, 
                        a_BlkErrBits[u16_p/2][u16_p%2][1].u16_Page,
                        a_BlkErrBits[u16_p/2][u16_p%2][1].u16_ErrBitCnt);
                else
                    ftl_dbg(0,0,"\n");
            }
        }                    
        if(u32_i++ == 0xFFFFFFFF)
        {u32_j++;  u32_i=0;}
    }    

    LABEL_END:
    ftl_dbg(0,0,"LoopCnt: %8Xh %8Xh \n", u32_j, u32_i);
    for(u16_p=0; u16_p<6; u16_p++)
    {
        ftl_dbg(0,0,"PBA:%3Xh LP:%3u EBits:%2u \n", 
            a_BlkErrBits[u16_p/2][u16_p%2][0].u16_PBA, 
            a_BlkErrBits[u16_p/2][u16_p%2][0].u16_Page,
            a_BlkErrBits[u16_p/2][u16_p%2][0].u16_ErrBitCnt);
        if(0==u16_p%2)
            ftl_dbg(0,0,"PBA:%3Xh HP:%3u EBits:%2u \n", 
                a_BlkErrBits[u16_p/2][u16_p%2][1].u16_PBA, 
                a_BlkErrBits[u16_p/2][u16_p%2][1].u16_Page,
                a_BlkErrBits[u16_p/2][u16_p%2][1].u16_ErrBitCnt);
        else
            ftl_dbg(0,0,"\n");
    }
    
    pNandDrv->u8_RequireRandomizer = u8_RequireRandomizer;
    if(0==u8_RequireRandomizer)
        NC_DisableLFSR();
}


void NC_Test_ErrBitStatus_W(U16 u16_PBA)
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U16 u16_i, au16_PBA[3][2]; // [1,PECYCLE,PECYCLE*2] PE cycles,  [FullBlk, LP only]
    U16 u16_PECnt, u16_cnt, u16_p;
    U32 u32_err, u32_i, BlkRow;
    U8  u8_RequireRandomizer = pNandDrv->u8_RequireRandomizer;

    for(u32_i=0; u32_i<pNandDrv->u16_PageByteCnt*pNandDrv->u16_BlkPageCnt; u32_i++)
        pu8_FtlTestDatBuf[u32_i]=0xFF;
    for(u32_i=0; u32_i<pNandDrv->u16_SpareByteCnt; u32_i++)
        pu8_FtlTestRddBuf[u32_i]=0xFF;

    // --------------------------
    LABEL_FIND_PBAs:
    
    //NC_DisableLFSR();
    
    u16_i=0;
    while(u16_i < 6)
    {
        //NC_EraseBlk(BlkRowAddr(u16_PBA));
        
        #if IF_FORCE_LFSR 
        if(0)//==drvNAND_IsGoodBlk(u16_PBA)) // when LFSR enabled, need BBT.
        #else
        if(0==drvNAND_IsGoodBlk(u16_PBA))
        #endif
        {
            ftl_dbg(FTL_DBG_LV_LOG, 1, "bad blk: %Xh \n", u16_PBA);
            u16_PBA++;            
        }
        else
        {   au16_PBA[u16_i/2][u16_i%2] = u16_PBA;
            u16_PBA++;
            u16_i++;
        }
    }

    #if IF_FORCE_LFSR  // test with LFSR
    pNandDrv->u8_RequireRandomizer = 1;

    if(pNandDrv->u8_RequireRandomizer)
        NC_EnableLFSR();
    #else
    ftl_dbg(FTL_DBG_LV_LOG,1,"suggest to enable IF_FORCE_LFSR, or data pattern is all 0xFF \n");
    #endif    
    
    ftl_dbg(FTL_DBG_LV_LOG,1,"if enable LFSR: %Xh \n", IF_LFSR_ENABLED()?1:0);
    
    // --------------------------
    // setup PBAs
    for(u16_i=0; u16_i<6; u16_i++)
    {
        u16_PECnt = PECYCLE*(u16_i/2)+1; // PE cycles

        ftl_dbg(0,0,"PBA %4Xh: %4u PE\n", 
            au16_PBA[u16_i/2][u16_i%2], u16_PECnt);
        
        for(u16_cnt=0; u16_cnt<u16_PECnt; u16_cnt++)
        {
            BlkRow = au16_PBA[u16_i/2][u16_i%2] << pNandDrv->u8_BlkPageCntBits;
                
            u32_err = NC_EraseBlk(BlkRow);
            if(u32_err != FTL_OK)
            {
                drvNAND_MarkBadBlk(au16_PBA[u16_i/2][u16_i%2]);
                ftl_dbg(FTL_DBG_LV_ERR,1,"erase fail: %Xh PBA \n", au16_PBA[u16_i/2][u16_i%2]);
                goto LABEL_FIND_PBAs;
            }

            if(0==u16_i%2) // FullBlk
            for(u16_p=0; u16_p<pNandDrv->u16_BlkPageCnt; u16_p++)
            {   
                u32_err = NC_WritePages(BlkRow + u16_p, 
                    pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, 1);
                if(u32_err != FTL_OK)
                {
                    drvNAND_MarkBadBlk(au16_PBA[u16_i/2][u16_i%2]);
                    ftl_dbg(FTL_DBG_LV_ERR,1,"write fail: PBA.%Xh P.%Xh \n", 
                        au16_PBA[u16_i/2][u16_i%2], u16_p);
                    goto LABEL_FIND_PBAs;
                }
            }
            else // LP-Only
            {
            for(u16_p=0; u16_p<pNandDrv->u16_BlkLowPCnt; u16_p++)
            {   
                u32_err = NC_WritePages(
					BlkRow + ga_tPairedPageMap[u16_p].u16_LSB, 
                    pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, 1);
                if(u32_err != FTL_OK)
                {
                    drvNAND_MarkBadBlk(au16_PBA[u16_i/2][u16_i%2]);
                    ftl_dbg(FTL_DBG_LV_ERR,1,"write fail: %Xh PBA \n", au16_PBA[u16_i/2][u16_i%2]);
                    goto LABEL_FIND_PBAs;
                }
                #if 0
                if(u16_p < pNandDrv->u16_BlkLowPCnt-1)
                    u32_err=drvNAND_WriteDummyToPages(
                        ga_tPairedPageMap[u16_p].u16_LSB+1, 
                        ga_tPairedPageMap[u16_p+1].u16_LSB-1, BlkRow);                    
                else
                    u32_err=drvNAND_WriteDummyToPages(
                        ga_tPairedPageMap[u16_p].u16_LSB+1, 
                        pNandDrv->u16_BlkPageCnt-1, BlkRow);
                if(u32_err != FTL_OK)
                {
                    drvNAND_MarkBadBlk(au16_PBA[u16_i/2][u16_i%2]);
                    ftl_dbg(FTL_DBG_LV_ERR,1,"write fail: %Xh PBA, %Xh \n", 
                        au16_PBA[u16_i/2][u16_i%2], u32_err);
                    goto LABEL_FIND_PBAs;
                }
				#endif
            }
            }
        }                   
    }
    
    ftl_dbg(FTL_DBG_LV_LOG,1,"OK: %Xh \n\n", REG(NC_LFSR_CTRL));    
    pNandDrv->u8_RequireRandomizer = u8_RequireRandomizer;
    if(0==u8_RequireRandomizer)
        NC_DisableLFSR();
    else
        NC_EnableLFSR();
        
    return;
}


// =============================================
// some APIs in drvNAND_utl.c
// =============================================
#define TEST_UTL_API
U32 NC_Test_2PAPIs(U16 u16_PBA)
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U32 *p1=(U32*)pu8_FtlTestDatBuf, *p2=(U32*)pu8_FtlTestTmpBuf, 
        *ps=(U32*)pu8_FtlTestRddBuf, i, err;

    // data pattern
    for(i=0; i<pNandDrv->u16_PageByteCnt/4; i++)
    {
        p1[i] = i;  p2[i] = ~p1[i];
    }
    memset(pu8_FtlTestRddBuf, 0xFF, pNandDrv->u16_SpareByteCnt);

    // erase
    err=NC_EraseBlk2P(u16_PBA<<pNandDrv->u8_BlkPageCntBits);
    if(err)
    {   ftl_dbg(FTL_DBG_LV_ERR,1,"NC_EraseBlk2P fail: %Xh\n", err);
        return err;
    }

    // write page    
    err=NC_WritePages2P(u16_PBA<<pNandDrv->u8_BlkPageCntBits, (U8*)p1, (U8*)ps, 1);
    if(err)
    {   ftl_dbg(FTL_DBG_LV_ERR,1,"NC_WritePages2P fail: %Xh\n", err);
        return err;
    } 
    #if 0
    // read page    
    err=NC_ReadPages2P(u16_PBA<<pNandDrv->u8_BlkPageCntBits, (U8*)p2, (U8*)ps, 1);
    if(err)
    {   ftl_dbg(FTL_DBG_LV_ERR,1,"NC_ReadPages2P fail: %Xh\n", err);
        return err;
    }
    // compare
    err=nand_CompareData((U8*)p1, (U8*)p2, pNandDrv->u16_PageByteCnt);
    if(err)
    {   ftl_dbg(FTL_DBG_LV_ERR,1,"CompareData fail: %Xh\n", err);
        return err;
    }
    #else    
    // read sectors
    err=NC_ReadSectors2P(u16_PBA<<pNandDrv->u8_BlkPageCntBits, 6, (U8*)p2, (U8*)ps, 4);
    if(err)
    {   ftl_dbg(FTL_DBG_LV_ERR,1,"NC_ReadPages2P fail: %Xh\n", err);
        return err;
    }
    // compare
    err=nand_CompareData((U8*)p1+pNandDrv->u16_SectorByteCnt*6, 
        (U8*)p2, pNandDrv->u16_SectorByteCnt*4);
    if(err)
    {   ftl_dbg(FTL_DBG_LV_ERR,1,"CompareData fail: %Xh\n", err);
        return err;
    }
    #endif

    // erase
    err=NC_EraseBlk2P(u16_PBA<<pNandDrv->u8_BlkPageCntBits);
    if(err)
    {   ftl_dbg(FTL_DBG_LV_ERR,1,"NC_EraseBlk2P fail: %Xh\n", err);
        return err;
    }
    ftl_dbg(FTL_DBG_LV_LOG,1,"PBA: %Xh - test ok\n", u16_PBA);
    return FTL_OK;
}

U32 NC_Test_APIs(U16 u16_PBA)
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    volatile U32 u32_err, u32_i;
    volatile U16 u16_FreeP, u16_LPIdx;

    //ftl_trace("\n");
    
    while(0==drvNAND_IsGoodBlk(u16_PBA))
    {
        ftl_dbg(FTL_DBG_LV_LOG, 1, "bad blk: %Xh \n", u16_PBA);
        u16_PBA++;
    }

    for(u32_i=0; u32_i<pNandDrv->u16_PageByteCnt; u32_i++)
        pu8_FtlTestDatBuf[u32_i]=u32_i;
    for(u32_i=0; u32_i<pNandDrv->u16_SpareByteCnt; u32_i++)
        pu8_FtlTestRddBuf[u32_i]=0xFF;

    u32_err = NC_EraseBlk(u16_PBA<<pNandDrv->u8_BlkPageCntBits);
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"erase %Xh blk fail: %Xh \n", u16_PBA, u32_err);
        return u32_err;
    }

    //ftl_trace("\n");
    if(0==drvNAND_IsFreeBlk(u16_PBA))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"IsFreeBlk fail: %Xh PBA \n", u16_PBA);
        return FTL_ERR_TEST_FAIL;
    }

    // ----------------------------------------------
    for(u16_LPIdx=0, u32_i=0; u32_i<=pNandDrv->u16_BlkPageCnt; u32_i++)
    {
        //ftl_dbg(0,1,"LPIdx: %Xh  PIdx: %Xh \n", u16_LPIdx, u32_i);
        
        u16_FreeP = drvNAND_FindFreePage(u16_PBA);
        if(u16_FreeP != u32_i)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,
                "FindFreePage fail: %Xh PBA, %Xh page, wrong page: %Xh \n", u16_PBA, u32_i, u16_FreeP);
            return FTL_ERR_TEST_FAIL;
        }
        if(u32_i == pNandDrv->u16_BlkPageCnt)
            break;

        if(u16_LPIdx < pNandDrv->u16_BlkLowPCnt)
        {
            u16_FreeP = drvNAND_FindFreeLSBPageIdx(u16_PBA);
            if(u16_FreeP != u16_LPIdx)//ga_tPairedPageMap[u16_LPIdx].u16_LSB)
            {   
                ftl_dbg(FTL_DBG_LV_ERR,1,
                    "FindFreeLSBPageIdx fail: LPIdx %Xh, FreeLPIdx: %Xh \n", 
                    u16_LPIdx, u16_FreeP);
                return FTL_ERR_TEST_FAIL;
            }
        }
        
        u32_err = NC_WritePages((u16_PBA<<pNandDrv->u8_BlkPageCntBits)+u32_i, 
            pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, 1);
        if(FTL_OK != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"write %Xh blk, %Xh page fail: %Xh \n", 
                u16_PBA, u16_PBA<<pNandDrv->u8_BlkPageCntBits, u32_err);
            return u32_err;
        }

        if(u32_i == ga_tPairedPageMap[u16_LPIdx].u16_LSB)
            u16_LPIdx++;
    }

    // ----------------------------------------------
    //ftl_trace("\n");
    
    u32_err = NC_EraseBlk(u16_PBA<<pNandDrv->u8_BlkPageCntBits);
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"erase %Xh blk fail: %Xh \n", u16_PBA, u32_err);
        return u32_err;
    }

    if(0==drvNAND_IsFreeBlk(u16_PBA))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"IsFreeBlk fail: %Xh PBA \n", u16_PBA);
        return FTL_ERR_TEST_FAIL;
    }

    //ftl_trace("\n");
    
    for(u32_i=0; u32_i<pNandDrv->u16_BlkLowPCnt; u32_i++)
    {
        //ftl_dbg(0,1,"LPIdx: %Xh \n", u32_i);
        
        u16_FreeP = drvNAND_FindFreePage(u16_PBA);
        if(u16_FreeP != ga_tPairedPageMap[u32_i].u16_LSB)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,
                "FindFreePage fail: %Xh PBA, %Xh page, wrong page: %Xh \n", 
                u16_PBA, u32_i, u16_FreeP);
            return FTL_ERR_TEST_FAIL;
        }

        u16_FreeP = drvNAND_FindFreeLSBPageIdx(u16_PBA);
        if(u16_FreeP != u32_i)//ga_tPairedPageMap[u32_i].u16_LSB)
        {   
            ftl_dbg(FTL_DBG_LV_ERR,1,
                "FindFreeLSBPageIdx fail: %Xh PBA, %Xh LP, wrong LP Idx: %Xh \n", 
                u16_PBA, ga_tPairedPageMap[u32_i].u16_LSB, u16_FreeP);
            return FTL_ERR_TEST_FAIL;
        }

        // ---------------------------------
        u32_err = NC_WritePages(
            (u16_PBA<<pNandDrv->u8_BlkPageCntBits)+ga_tPairedPageMap[u32_i].u16_LSB, 
            pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, 1);
        if(FTL_OK != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"write %Xh blk, %Xh page fail: %Xh \n", 
                u16_PBA, (u16_PBA<<pNandDrv->u8_BlkPageCntBits)+ga_tPairedPageMap[u32_i].u16_LSB, u32_err);
            return u32_err;
        }

        u16_FreeP = drvNAND_FindFreePage(u16_PBA);
        if(u16_FreeP != ga_tPairedPageMap[u32_i].u16_LSB+1)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,
                "FindFreePage fail: %Xh PBA, %Xh page, wrong page: %Xh \n", 
                u16_PBA, ga_tPairedPageMap[u32_i].u16_LSB+1, u16_FreeP);
            return FTL_ERR_TEST_FAIL;
        }

        u16_FreeP = drvNAND_FindFreeLSBPageIdx(u16_PBA);
        if(u16_FreeP != u32_i+1)//ga_tPairedPageMap[u32_i+1].u16_LSB)
        {   
            ftl_dbg(FTL_DBG_LV_ERR,1,
                "FindFreeLSBPageIdx fail: %Xh PBA, %Xh LP, wrong LP Idx: %Xh \n", 
                u16_PBA, ga_tPairedPageMap[u32_i+1].u16_LSB, u16_FreeP);
            return FTL_ERR_TEST_FAIL;
        }

        // ---------------------------------
        if(u32_i+1 < pNandDrv->u16_BlkLowPCnt)
        {
            u32_err = drvNAND_WriteDummyToPages(
                ga_tPairedPageMap[u32_i].u16_LSB+1, 
                ga_tPairedPageMap[u32_i+1].u16_LSB-1,
                u16_PBA<<pNandDrv->u8_BlkPageCntBits);
            if(FTL_OK != u32_err)
            {
                ftl_dbg(FTL_DBG_LV_ERR,1,"drvNAND_WriteDummyToPages fail: %Xh %Xh %Xh\n", 
                    u32_i, ga_tPairedPageMap[u32_i].u16_LSB+1, ga_tPairedPageMap[u32_i+1].u16_LSB-1);
                return u32_err;
            }
        }
    }
       
    ftl_dbg(FTL_DBG_LV_LOG,1,"OK\n");
    NC_EraseBlk(u16_PBA<<pNandDrv->u8_BlkPageCntBits);
    return FTL_OK;
    
}


// ===================================
// BIST
U32 NC_Test_Basic(U16 u16_PBA, U16 u16_Op, U16 LoopCnt)
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U32 u32_Err=UNFD_ST_SUCCESS, u32_Row = u16_PBA<<pNandDrv->u8_BlkPageCntBits, i;
    
    switch(u16_Op)
    {
        case NC_TEST_Init:
            ftl_dbg(0,1,"init \n");
            memset(pNandDrv, 0, sizeof(NAND_DRIVER));
            NC_PlatformInit();
            pNandDrv->u16_BlkCnt = 0x400;
        	pNandDrv->u16_BlkPageCnt = 0x40;
        	pNandDrv->u16_PageByteCnt = 0x200;
        	pNandDrv->u16_SpareByteCnt = 0x10;
            pNandDrv->u16_ECCType = ECC_TYPE_4BIT;
            pNandDrv->u8_Flag_004A30 = 0;
        	NC_ConfigNandFlashContext();
        	u32_Err = NC_Init();
        	if (u32_Err != UNFD_ST_SUCCESS)
        	{
                ftl_dbg(0,1,"init fail: %Xh\n", u32_Err);
        		return u32_Err;    
        	}
            break;

        case NC_TEST_ResetCMD:
            ftl_dbg(0,1,"reset NAND\n");
            NC_PlatformInit();
            NC_ReConfig();
            for(i=0; i<LoopCnt; i++)
                NC_ResetNandFlash();
            break;
            
        case NC_TEST_ReadID:
            ftl_dbg(0,1,"read NAND ID\n");

            for(i=0; i<LoopCnt; i++)
                NC_ReadID();
            
            if( drvNAND_CHECK_FLASH_TYPE() != 0)
            {
                printf("[%s]\tUnsupported NAND Type, Need to update CIS or modify code\n",__func__);
                return -1;
            }
            NC_ConfigNandFlashContext();
            NC_Init();
            break;
            
        case NC_TEST_EraseB:
            ftl_dbg(0,1,"erase PBA:%Xh\n", u16_PBA);
            for(i=0; i<LoopCnt; i++)
                if(UNFD_ST_SUCCESS != (u32_Err=NC_EraseBlk(u32_Row+(i<<pNandDrv->u8_BlkPageCntBits))))
                    ftl_dbg(0,1,"erase fail: %Xh\n", u32_Err);
            break;
            
        case NC_TEST_WriteP:
        case NC_TEST_Write2P:
            ftl_dbg(0,1,"write PBA:%Xh (row:%Xh) %Xh\n", u16_PBA, u32_Row, LoopCnt);
            for(i=0; i<pNandDrv->u16_PageByteCnt/4; i++)
            {
                pNandDrv->pu8_PageDataBuf[i*4+0] = i&0xFF;
                pNandDrv->pu8_PageDataBuf[i*4+1] = (i&0xFF00)>>8;
                pNandDrv->pu8_PageDataBuf[i*4+2] = (i&0xFF0000)>>16;
                pNandDrv->pu8_PageDataBuf[i*4+3] = i>>24;
            }
            memset(pNandDrv->pu8_PageSpareBuf, 0xFF, pNandDrv->u16_SpareByteCnt);

            if(NC_TEST_Write2P==u16_Op)
                u32_Err=NC_WritePages2PCache(u32_Row, pNandDrv->pu8_PageDataBuf, 
                    pNandDrv->pu8_PageSpareBuf, LoopCnt);
            else if(NC_TEST_WriteP==u16_Op)
                u32_Err=NC_WritePages(u32_Row, pNandDrv->pu8_PageDataBuf, 
                    pNandDrv->pu8_PageSpareBuf, LoopCnt);                
            if(UNFD_ST_SUCCESS != u32_Err)
                ftl_dbg(0,1,"write fail: %Xh\n", u32_Err);
            break;
            
        case NC_TEST_ReadP:
            ftl_dbg(0,1,"read PBA:%Xh (row:%Xh)\n", u16_PBA, u32_Row);

            for(i=0; i<LoopCnt; i++)
                u32_Err=NC_ReadPages(u32_Row, pNandDrv->pu8_PageDataBuf, 
                    pNandDrv->pu8_PageSpareBuf, 1);
            if(UNFD_ST_SUCCESS != u32_Err)
                ftl_dbg(0,1,"write fail: %Xh\n", u32_Err);

            for(i=0; i<pNandDrv->u16_PageByteCnt/4; i++)
            {
                if(pNandDrv->pu8_PageDataBuf[i*4+0] != (i&0xFF) ||
                    pNandDrv->pu8_PageDataBuf[i*4+1] != (i&0xFF00)>>8 ||
                    pNandDrv->pu8_PageDataBuf[i*4+2] != (i&0xFF0000)>>16 ||
                    pNandDrv->pu8_PageDataBuf[i*4+3] != (i>>24))
                {
                    ftl_dbg(0,1,"cmp fail: data %Xh %Xh\n", 
                        pNandDrv->pu8_PageDataBuf[i*4+0]+
                        (pNandDrv->pu8_PageDataBuf[i*4+1]<<8)+
                        (pNandDrv->pu8_PageDataBuf[i*4+2]<<16)+
                        (pNandDrv->pu8_PageDataBuf[i*4+3]<<24), i);
                    break;
                }
            }
            break;
    }
    
    return FTL_OK;
}

U32 NC_Test_BIST(U16 u16_PBA) 
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U32 u32_err, u32_i;
    U16 PBA2;

    if(/*u16_PBA < FtlDev.u16_StartPBA ||*/ u16_PBA < 20) // hash/uboot
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"PBA1:%Xh too small\n", u16_PBA);
        return FTL_ERR_OUT_OF_RANGE;
    }

    if((U32)pu8_FtlTestTmpBuf-(U32)pu8_FtlTestDatBuf < pNandDrv->u16_PageByteCnt)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"buf size:%Xh < page size: %Xh\n", 
            (U32)pu8_FtlTestTmpBuf-(U32)pu8_FtlTestDatBuf, 
            pNandDrv->u16_PageByteCnt);
        return FTL_ERR_OUT_OF_RANGE;
    }

    // --------------------------
    // get the next PBA as PBA2 to test
    // --------------------------
    #if 1
    if(0==drvNAND_IsGoodBlk(u16_PBA))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"PBA1:%Xh, is bad blk \n", u16_PBA);
        return FTL_ERR_OUT_OF_RANGE;
    }
    
    PBA2 = u16_PBA + pNandDrv->u8_PlaneCnt;
    if(0==drvNAND_IsGoodBlk(PBA2))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"PBA2:%Xh, is bad blk \n", PBA2);
        return FTL_ERR_OUT_OF_RANGE;
    }
    #endif

    u32_err = NC_EraseBlk(u16_PBA<<pNandDrv->u8_BlkPageCntBits);
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"erase %Xh blk fail: %Xh \n", u16_PBA, u32_err);
        return u32_err;
    }

    for(u32_i=0; u32_i<pNandDrv->u16_PageByteCnt; u32_i++)
    {
        pu8_FtlTestDatBuf[u32_i]=u32_i/4;
        pu8_FtlTestTmpBuf[u32_i]=~pu8_FtlTestDatBuf[u32_i];
    }
    pu8_FtlTestRddBuf[0]=0xFF;
    for(u32_i=1; u32_i<pNandDrv->u16_SpareByteCnt; u32_i++)
        pu8_FtlTestRddBuf[u32_i]=u32_i/4;

    for(u32_i=0; u32_i<pNandDrv->u16_BlkPageCnt; u32_i++)
    //for(u32_i=0; u32_i<1; u32_i++)
    {
        u32_err = NC_WritePages((u16_PBA<<pNandDrv->u8_BlkPageCntBits)+u32_i, 
            pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, 1);
        if(FTL_OK != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"write %Xh blk, %Xh page fail: %Xh \n", 
                u16_PBA, u32_i, u32_err);
            return u32_err;
        }
    }

    u32_err = NC_ReadPages(u16_PBA<<pNandDrv->u8_BlkPageCntBits, 
        pu8_FtlTestTmpBuf, pu8_FtlTestTmpBuf+pNandDrv->u16_PageByteCnt, 1);
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"read %Xh blk, %Xh Row fail: %Xh \n", 
            u16_PBA, u16_PBA<<pNandDrv->u8_BlkPageCntBits, u32_err);
        return u32_err;
    }
    
    u32_err = nand_CompareData(pu8_FtlTestDatBuf, pu8_FtlTestTmpBuf, pNandDrv->u16_PageByteCnt);
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"c data fail: %Xh blk fail: %Xh \n", 
            u16_PBA, u32_err);
        return u32_err;
    }

    for(u32_i=0; u32_i<pNandDrv->u16_SpareByteCnt/pNandDrv->u8_PlaneCnt; u32_i+=pNandDrv->u16_SectorSpareByteCnt)
    {
        u32_err = nand_CompareData(
            pu8_FtlTestRddBuf+u32_i, (pu8_FtlTestTmpBuf+pNandDrv->u16_PageByteCnt)+u32_i, 
            pNandDrv->u16_SectorSpareByteCnt-pNandDrv->u16_ECCCodeByteCnt);
        if(FTL_OK != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"c spare fail: %Xh blk, %Xh-th sector fail: %Xh \n", 
                u16_PBA, u32_i/pNandDrv->u16_SectorSpareByteCnt, u32_err);
            return u32_err;            
        }
    }
    ftl_dbg(FTL_DBG_LV_LOG,1,"E/W/R/C OK\n");

    // ===================================
    #if 0
    u32_err = NC_EraseBlk(PBA2<<pNandDrv->u8_BlkPageCntBits);
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"erase PBA2: %Xh blk fail: %Xh \n", PBA2, u32_err);
        return u32_err;
    }
    
    u32_err = NC_PageCopy(u16_PBA<<pNandDrv->u8_BlkPageCntBits, 
        PBA2<<pNandDrv->u8_BlkPageCntBits,
        pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, pNandDrv->u16_BlkPageCnt, &u32_tmp);    
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"NC_PageCopy fail: %Xh \n", u32_err);
        return u32_err;
    }

    for(u32_i=0; u32_i<pNandDrv->u16_BlkPageCnt; u32_i++)
    {
        u32_err = NC_ReadPages((u16_PBA<<pNandDrv->u8_BlkPageCntBits)+u32_i, 
            pu8_FtlTestDatBuf, pu8_FtlTestDatBuf+pNandDrv->u16_PageByteCnt, 1);
        if(FTL_OK != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"read %Xh blk, %Xh page fail: %Xh \n", 
                u16_PBA, u32_i, u32_err);
            return u32_err;
        }
        u32_err = NC_ReadPages((PBA2<<pNandDrv->u8_BlkPageCntBits)+u32_i, 
            pu8_FtlTestTmpBuf, pu8_FtlTestTmpBuf+pNandDrv->u16_PageByteCnt, 1);
        if(FTL_OK != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"read %Xh blk, %Xh page fail: %Xh \n", 
                u16_PBA, u32_i, u32_err);
            return u32_err;
        }
        u32_err = nand_CompareData(
            pu8_FtlTestDatBuf, pu8_FtlTestTmpBuf, 
            pNandDrv->u16_PageByteCnt+pNandDrv->u16_SpareByteCnt/pNandDrv->u8_PlaneCnt);
        if(FTL_OK != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"cmp fail: %Xh %Xh blk, %Xh page fail: %Xh \n", 
                u16_PBA, PBA2, u32_i, u32_err);
            return u32_err;
        }
    }
    
    ftl_dbg(FTL_DBG_LV_LOG,1,"CopyPage OK\n\n");
    #endif
    NC_EraseBlk(u16_PBA<<pNandDrv->u8_BlkPageCntBits);
    NC_EraseBlk(PBA2<<pNandDrv->u8_BlkPageCntBits);
    return u32_err;
}


U32 NC_Test_PwrCut(U16 LoopCnt)
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U32 u32_err, u32_i;
    U16 PBA, PBA2, u16_i;

    nand_hw_timer_delay(HW_TIMER_DELAY_1s*3);	
    nand_hw_timer_delay(HW_TIMER_DELAY_100ms*3);	
    // ===================================
    ftl_dbg(FTL_DBG_LV_LOG,0,"\n");
    PBA = HWTimer_End();
    if(0==PBA%3)  PBA=52;
    else if(1==PBA%3)  PBA=50;
    else  
        PBA=48;
    PBA2 = PBA+1;
    ftl_dbg(FTL_DBG_LV_LOG,1,"PBA: %Xh %Xh\n", PBA, PBA2);
    // ===================================
    
    // read to check UECC
    ftl_dbg(FTL_DBG_LV_LOG,0,"r\n");
    for(u32_i=0; u32_i<pNandDrv->u16_BlkPageCnt; u32_i++)
    {
        u32_err=NC_ReadPages((PBA<<pNandDrv->u8_BlkPageCntBits)+u32_i, 
            pu8_FtlTestDatBuf, pu8_FtlTestDatBuf+pNandDrv->u16_PageByteCnt, 1);
        if(UNFD_ST_SUCCESS != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"\n");
            ftl_dbg(FTL_DBG_LV_ERR,1,"read %Xh blk, %Xh page fail: %Xh \n", 
                PBA, u32_i, u32_err);
            while(1);
        }

        u32_err=NC_ReadPages((PBA2<<pNandDrv->u8_BlkPageCntBits)+u32_i, 
            pu8_FtlTestDatBuf, pu8_FtlTestDatBuf+pNandDrv->u16_PageByteCnt, 1);
        if(UNFD_ST_SUCCESS != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"\n");
            ftl_dbg(FTL_DBG_LV_ERR,1,"read %Xh blk, %Xh page fail: %Xh \n", 
                PBA, u32_i, u32_err);
            while(1);
        }
    }

    for(u16_i=0; u16_i<LoopCnt; u16_i++)
    {
        ftl_dbg(FTL_DBG_LV_LOG,0,"e\n");
        u32_err=NC_EraseBlk2P(PBA<<pNandDrv->u8_BlkPageCntBits);
        if(u32_err)
        {   
            ftl_dbg(FTL_DBG_LV_ERR,1,"\n");
            ftl_dbg(FTL_DBG_LV_ERR,1,"NC_EraseBlk2P fail: %Xh\n", u32_err);
            while(1);
        }

        for(u32_i=0; u32_i<pNandDrv->u16_BlkPageCnt; /*u32_i++*/){
        ftl_dbg(FTL_DBG_LV_LOG,0,"w: %Xh\n", u32_i);
        u32_err=NC_WritePages2PCache(
            (PBA<<pNandDrv->u8_BlkPageCntBits)+u32_i, 
            pNandDrv->pu8_PageDataBuf, pNandDrv->pu8_PageSpareBuf, 
            pNandDrv->u16_BlkPageCnt/4);
        u32_i += pNandDrv->u16_BlkPageCnt/4;
        if(u32_err)
        {   
            ftl_dbg(FTL_DBG_LV_ERR,1,"\n");
            ftl_dbg(FTL_DBG_LV_ERR,1,"NC_WritePages2PCache fail: %Xh\n", u32_err);
            while(1);
        }}

        if(0x3697==LoopCnt)
            u16_i = 0;
    }

    return FTL_OK;   
}


U32 NC_Test_PwrCut_Toggle(void)
{
    REG_CLR_BITS_UINT16(NC_SIGNAL, BIT_NC_WP_AUTO|BIT_NC_CE_AUTO|BIT_NC_CE_H);
    while(1)
    {
        REG_SET_BITS_UINT16(NC_SIGNAL, BIT_NC_WP_H);
        nand_hw_timer_delay(HW_TIMER_DELAY_1ms/2);
        REG_CLR_BITS_UINT16(NC_SIGNAL, BIT_NC_WP_H);
        nand_hw_timer_delay(HW_TIMER_DELAY_1ms/2);
    }
    return FTL_OK;   
}

// ===================================
void NC_Test_CheckAll0xFF(void)
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U8 *pu8;
    U32 u32_i, u32_j;

    ftl_dbg(0, 0, "test drvNAND_CheckAll0xFF \n");

    for(u32_i=0; u32_i<pNandDrv->u16_PageByteCnt; u32_i++)
        pu8_FtlTestDatBuf[u32_i] = 0xFF;

    HWTimer_Start();
    for(u32_i=0; u32_i<0x10; u32_i++)
    {
        pu8 = pu8_FtlTestDatBuf + u32_i;

        for(u32_j=0; u32_j<0x400; u32_j++)
        {
            pu8[u32_j] = 0;
            if(0xFF == drvNAND_CheckAll0xFF(pu8, 0x400))
            {
                ftl_dbg(FTL_DBG_LV_ERR, 0, "CheckAll0xFF fail: %Xh %Xh \n", u32_i, u32_j);
                return;
            }
            pu8[u32_j] = 0xFF;
        }
        if(0xFF != drvNAND_CheckAll0xFF(pu8, 0x400))
        {
            ftl_dbg(FTL_DBG_LV_ERR, 0, "CheckAll0xFF fail: %Xh %Xh \n", u32_i, u32_j);
            return;
        }
    }
    u32_j = HWTimer_End();
    ftl_dbg(0, 0, "OK: %u \n", u32_j); 
    return;    
}


U32 MsNAND_Test_Timer(void) // verified ok
{
    U32 u32_i, u32_tmp;

    HWTimer_Start();
    
    for(u32_i=0; u32_i<6; u32_i++)
    {
        u32_tmp = HWTimer_End();
        
        nand_hw_timer_delay(DELAY_1s_in_us);
        
        ftl_dbg(0,0,"%u \n", HWTimer_End()-u32_tmp); // 12M
    }
    return u32_i;
}


U32 MsNAND_Test(U16 u16_PBA)
{
    U32 u32_err=UNFD_ST_SUCCESS;
    
    //u32_err = NC_Test_Reg5B5C(u16_PBA);
    //NC_Test_Reg717274(u16_PBA);

    //NC_Test_BIST(u16_PBA);

    //NC_Test_APIs(u16_PBA);

    //NC_Test_CheckAll0xFF();
    //MsNAND_Test_Timer();    

    return u32_err;
}

// =============================================
// FTL test
// =============================================
#define TEST_FTL

#define TEST_PAGE_CNT 20

U32 FtlTest_HAL(U16 u16_PBA) 
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U32 u32_err, u32_i, u32_t0, u32_t1, u32_t;
    U16 PBA2;

    if(u16_PBA < FtlDev.u16_StartPBA || u16_PBA < 20) // hash/uboot
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"PBA1:%Xh < FTL Start: %Xh\n", u16_PBA, FtlDev.u16_StartPBA);
        return FTL_ERR_OUT_OF_RANGE;
    }

    if((U32)pu8_FtlTestTmpBuf-(U32)pu8_FtlTestDatBuf < FtlDev.u16_PageByteCnt)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"buf size:%Xh < page size: %Xh\n", 
            (U32)pu8_FtlTestTmpBuf-(U32)pu8_FtlTestDatBuf, 
            FtlDev.u16_PageByteCnt);
        return FTL_ERR_OUT_OF_RANGE;
    }

    // --------------------------
    // get the next PBA as PBA2 to test
    // --------------------------
    #if 1
    if(0==NFTL_IsGoodBlk(u16_PBA))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"PBA1:%Xh, is bad blk \n", u16_PBA);
        return FTL_ERR_OUT_OF_RANGE;
    }
    
    PBA2 = u16_PBA + FtlDev.u8_PlaneCnt;
    if(0==NFTL_IsGoodBlk(PBA2))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"PBA2:%Xh, is bad blk \n", PBA2);
        return FTL_ERR_OUT_OF_RANGE;
    }

    //===========================================
    // real PBA -> multi-plane PBA (accepted by HAL APIs)
    ftl_dbg(FTL_DBG_LV_LOG,1,"PBA1: %Xh  PBA2: %Xh \n", u16_PBA, PBA2);
    u16_PBA = FtlRoot.Info.u16_StartMainPBA + 
        (u16_PBA-FtlRoot.Info.u16_StartMainPBA)/FtlDev.u8_PlaneCnt;
    PBA2 = FtlRoot.Info.u16_StartMainPBA + 
        (PBA2-FtlRoot.Info.u16_StartMainPBA)/FtlDev.u8_PlaneCnt;

    ftl_dbg(FTL_DBG_LV_LOG,1,"Original:  PBA1: %Xh  PBA2: %Xh \n", 
        PhyPBAFromMultiPlanePBA(u16_PBA), PhyPBAFromMultiPlanePBA(PBA2));
    ftl_dbg(FTL_DBG_LV_LOG,1,"MultiPlane: %Xh  PBA1: %Xh  PBA2: %Xh \n", 
        FtlDev.u8_PlaneCnt, u16_PBA, PBA2);
    //===========================================
    #endif
    HWTimer_Start();
    u32_err = (*FtlDev.pFnEraseBlk)(u16_PBA);
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"erase %Xh blk fail: %Xh \n", u16_PBA, u32_err);
        return u32_err;
    }    
    
    //ftl_dbg(FTL_DBG_LV_LOG,1,"init data\n");    
    u32_t0 = HWTimer_End();
    for(u32_i=0; u32_i<FtlDev.u16_PageByteCnt*TEST_PAGE_CNT; u32_i++)
    {
        if((u32_i % FtlDev.u16_PageByteCnt) == 0)
            pu8_FtlTestDatBuf[u32_i] = u32_i / FtlDev.u16_PageByteCnt;
        else
            pu8_FtlTestDatBuf[u32_i]= rRand(u32_t0 + u32_i, u32_t0 * u32_i) & 0xFF;
        pu8_FtlTestTmpBuf[u32_i]=~pu8_FtlTestDatBuf[u32_i];
    }
    pu8_FtlTestRddBuf[0]=0xFF;
    for(u32_i=1; u32_i<FtlDev.u16_SpareByteCnt; u32_i++)
        pu8_FtlTestRddBuf[u32_i]=u32_i/4;
    
    ftl_dbg(FTL_DBG_LV_LOG,1,"Write2p Hal Speed:");
    HWTimer_Start();    
    FTL_Disable_ErrLog = 1;
    u32_t0 = HWTimer_End();
    u32_err = HAL_WritePages2P(BlkRowAddr(u16_PBA), 
        pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, TEST_PAGE_CNT);
    u32_t1 = HWTimer_End();
    u32_t = u32_t1 > u32_t0 ? u32_t1-u32_t0 : 0xFFFFFFFF-u32_t0+u32_t1;
    u32_t = u32_t / (FTL_HW_TIMER_HZ/1000); // ms
    FTL_Disable_ErrLog = 0;
    ftl_dbg(FTL_DBG_LV_LOG,0,"%u.%u MB/s \n", 
        (((U32)FtlDev.u16_PageByteCnt * TEST_PAGE_CNT)*1000/u32_t)>>20, ((((U32)FtlDev.u16_PageByteCnt * TEST_PAGE_CNT>>20)*10000/u32_t)>>20)%10);    
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"write %Xh blk, %Xh page fail: %Xh \n", 
            u16_PBA, u32_i, u32_err);
        return u32_err;
    }
    
    ftl_dbg(FTL_DBG_LV_LOG,1,"Read2p Hal Speed:");
    HWTimer_Start();
    FTL_Disable_ErrLog = 1;
    u32_t0 = HWTimer_End();    
    u32_err = HAL_ReadPages2P(u16_PBA<<pNandDrv->u8_BlkPageCntBits, 
         pu8_FtlTestTmpBuf, pu8_FtlTestRddBuf+FtlDev.u16_SpareByteCnt, TEST_PAGE_CNT);
    u32_t1 = HWTimer_End();
    u32_t = u32_t1 > u32_t0 ? u32_t1-u32_t0 : 0xFFFFFFFF-u32_t0+u32_t1;
    u32_t = u32_t / (FTL_HW_TIMER_HZ/1000); // ms
    FTL_Disable_ErrLog = 0;
    ftl_dbg(FTL_DBG_LV_LOG,0,"%u.%u MB/s \n", 
        (((U32)FtlDev.u16_PageByteCnt * TEST_PAGE_CNT)*1000/u32_t)>>20, ((((U32)FtlDev.u16_PageByteCnt * TEST_PAGE_CNT)*10000/u32_t)>>20)%10);       
     if(FTL_OK != u32_err)
     {
         ftl_dbg(FTL_DBG_LV_ERR,1,"read %Xh blk, %Xh Row fail: %Xh \n", 
             u16_PBA, u16_PBA<<pNandDrv->u8_BlkPageCntBits, u32_err);
         return u32_err;
     }
     u32_err = nand_CompareData(pu8_FtlTestDatBuf, pu8_FtlTestTmpBuf, 
        FtlDev.u16_PageByteCnt*TEST_PAGE_CNT);
     if(FTL_OK != u32_err)
     {   
         ftl_dbg(FTL_DBG_LV_ERR,1,"c data fail: %Xh blk, fail: %Xh \n", 
             u16_PBA, u32_err);
         return u32_err;
     }
     u32_err = nand_CompareData(pu8_FtlTestRddBuf, 
        pu8_FtlTestRddBuf+FtlDev.u16_SpareByteCnt, 
        FtlDev.u16_SectorSpareByteCnt-FtlDev.u16_ECCCodeByteCnt);
     if(FTL_OK != u32_err)
     {   
         ftl_dbg(FTL_DBG_LV_ERR,1,"c spare fail: %Xh blk, fail: %Xh \n", 
             u16_PBA, u32_err);
         return u32_err;
     }
     
    ftl_dbg(FTL_DBG_LV_LOG,1,"E/W/R/C OK\n");


    u32_err = (*FtlDev.pFnEraseBlk)(u16_PBA);
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"erase %Xh blk fail: %Xh \n", u16_PBA, u32_err);
        return u32_err;
    }
    //ftl_dbg(FTL_DBG_LV_LOG,1,"init data\n");
    for(u32_i=0; u32_i<FtlDev.u16_PageByteCnt*TEST_PAGE_CNT; u32_i++)
    {
        if((u32_i % FtlDev.u16_PageByteCnt) == 0)
            pu8_FtlTestDatBuf[u32_i] = u32_i / FtlDev.u16_PageByteCnt;
        else
            pu8_FtlTestDatBuf[u32_i]= rRand(u32_t0 + u32_i, u32_t0 * u32_i) & 0xFF;
        pu8_FtlTestTmpBuf[u32_i]=~pu8_FtlTestDatBuf[u32_i];
    }
    pu8_FtlTestRddBuf[0]=0xFF;
    for(u32_i=1; u32_i<FtlDev.u16_SpareByteCnt; u32_i++)
        pu8_FtlTestRddBuf[u32_i]=u32_i/4;
    

    ftl_dbg(FTL_DBG_LV_LOG,1,"Write2p Cache Hal Speed:");
    HWTimer_Start();
    FTL_Disable_ErrLog = 1;
    u32_t0 = HWTimer_End();
    u32_err = HAL_WritePages2PCache(BlkRowAddr(u16_PBA), 
        pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, TEST_PAGE_CNT);
    u32_t1 = HWTimer_End();
    u32_t = u32_t1 > u32_t0 ? u32_t1-u32_t0 : 0xFFFFFFFF-u32_t0+u32_t1;
    u32_t = u32_t / (FTL_HW_TIMER_HZ/1000); // ms
    FTL_Disable_ErrLog = 0;
    ftl_dbg(FTL_DBG_LV_LOG,0,"%u.%u MB/s \n", 
        (((U32)FtlDev.u16_PageByteCnt * TEST_PAGE_CNT)*1000/u32_t)>>20, ((((U32)FtlDev.u16_PageByteCnt * TEST_PAGE_CNT)*10000/u32_t)>>20)%10);    
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"write %Xh blk, %Xh page fail: %Xh \n", 
            u16_PBA, u32_i, u32_err);
        return u32_err;
    }
    
    ftl_dbg(FTL_DBG_LV_LOG,1,"Read2p Cache Hal Speed:");
    HWTimer_Start();
    FTL_Disable_ErrLog = 1;
    u32_t0 = HWTimer_End();    
    u32_err = HAL_ReadPages2PCache(u16_PBA<<pNandDrv->u8_BlkPageCntBits, 
         pu8_FtlTestTmpBuf, pu8_FtlTestRddBuf+FtlDev.u16_SpareByteCnt, TEST_PAGE_CNT);
    u32_t1 = HWTimer_End();
    u32_t = u32_t1 > u32_t0 ? u32_t1-u32_t0 : 0xFFFFFFFF-u32_t0+u32_t1;
    u32_t = u32_t / (FTL_HW_TIMER_HZ/1000); // ms
    FTL_Disable_ErrLog = 0;
    ftl_dbg(FTL_DBG_LV_LOG,0,"%u.%u MB/s \n", 
        (((U32)FtlDev.u16_PageByteCnt * TEST_PAGE_CNT)*1000/u32_t)>>20, ((((U32)FtlDev.u16_PageByteCnt * TEST_PAGE_CNT)*10000/u32_t)>>20)%10);       
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"read %Xh blk, %Xh Row fail: %Xh \n", 
            u16_PBA, u16_PBA<<pNandDrv->u8_BlkPageCntBits, u32_err);
        return u32_err;
    }
    u32_err = nand_CompareData(pu8_FtlTestDatBuf, pu8_FtlTestTmpBuf, FtlDev.u16_PageByteCnt*TEST_PAGE_CNT);
    if(FTL_OK != u32_err)
    {   
        ftl_dbg(FTL_DBG_LV_ERR,1,"c data fail: %Xh blk, %Xh page fail: %Xh \n", 
            u16_PBA, u16_PBA&FtlDev.u16_BlkPageCntMask, u32_err);
        return u32_err;
    }
    u32_err = nand_CompareData(pu8_FtlTestRddBuf, 
        pu8_FtlTestRddBuf+FtlDev.u16_SpareByteCnt, 
        FtlDev.u16_SectorSpareByteCnt-FtlDev.u16_ECCCodeByteCnt);
    if(FTL_OK != u32_err)
    {   
        ftl_dbg(FTL_DBG_LV_ERR,1,"c spare fail: %Xh blk, %Xh page fail: %Xh \n", 
            u16_PBA, u16_PBA&FtlDev.u16_BlkPageCntMask, u32_err);
        return u32_err;
    }
    ftl_dbg(FTL_DBG_LV_LOG,1,"E/W/R/C OK\n");
    

	#if 0
    // ===================================
    u32_err = (*FtlDev.pFnEraseBlk)(PBA2);
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"erase PBA2: %Xh blk fail: %Xh \n", PBA2, u32_err);
        return u32_err;
    }
    
    u32_err = HAL_PageCopy(BlkRowAddr(u16_PBA), BlkRowAddr(PBA2),
    //u32_err = NC_PageCopy_P2P(BlkRowAddr(u16_PBA), BlkRowAddr(PBA2),
        pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, TEST_PAGE_CNT);
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"NC_PageCopy fail: %Xh \n", u32_err);
        return u32_err;
    }

    for(u32_i=0; u32_i<TEST_PAGE_CNT; u32_i++)
    {
        u32_err = (*FtlDev.pFnReadPages)(BlkRowAddr(u16_PBA)+u32_i, 
            pu8_FtlTestDatBuf, pu8_FtlTestRddBuf, 1);
        if(FTL_OK != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"read %Xh blk, %Xh page fail: %Xh \n", 
                u16_PBA, u32_i, u32_err);
            return u32_err;
        }
        u32_err = (*FtlDev.pFnReadPages)(BlkRowAddr(PBA2)+u32_i, 
            pu8_FtlTestTmpBuf, pu8_FtlTestRddBuf+FtlDev.u16_SpareByteCnt, 1);
        if(FTL_OK != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"read %Xh blk, %Xh page fail: %Xh \n", 
                u16_PBA, u32_i, u32_err);
            return u32_err;
        }
        u32_err = nand_CompareData(
            pu8_FtlTestDatBuf, pu8_FtlTestTmpBuf, 
            FtlDev.u16_PageByteCnt+FtlDev.u16_SpareByteCnt/FtlDev.u8_PlaneCnt);
        if(FTL_OK != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"c data fail: %Xh %Xh blk, %Xh page fail: %Xh \n", 
                u16_PBA, PBA2, u32_i, u32_err);
            return u32_err;
        }
        u32_err = nand_CompareData(
            pu8_FtlTestRddBuf, pu8_FtlTestRddBuf+FtlDev.u16_SpareByteCnt, 
            FtlDev.u16_SectorSpareByteCnt-FtlDev.u16_ECCCodeByteCnt);
        if(FTL_OK != u32_err)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"c spare fail: %Xh %Xh blk, %Xh page fail: %Xh \n", 
                u16_PBA, PBA2, u32_i, u32_err);
            return u32_err;
        }
    }
    
    ftl_dbg(FTL_DBG_LV_LOG,1,"CopyPage OK\n");
	#endif

    (*FtlDev.pFnEraseBlk)(u16_PBA);
    (*FtlDev.pFnEraseBlk)(PBA2);
    return u32_err;
}


U32 FtlTest_RootCtrl(U32 u32_TestLoop)
{
    U32 u32_err, u32_i, u32_cnt=0;
    
    u32_err = NFTL_Init(0,0);
    if(FTL_OK != u32_err)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"init fail: %Xh \n", u32_err);
    }    
    NFTL_DumpInfo();

    for(u32_i=0; u32_i<u32_TestLoop; (u32_TestLoop==3697)?u32_i=0:u32_i++)
    {
        ftl_dbg(FTL_DBG_LV_LOG,0,"\n==============================================\n");
        
        u32_err = NFTL_WriteCtrlInfo_AllSubID(CTRL_ID_DBPM);

        NFTL_DumpInfo();

        #if defined(BKG_HK_THREAD) && BKG_HK_THREAD
          #if MSFTL_UBOOT
        NFTL_BKG_Housekeep();
          #endif
        #endif

        u32_cnt++;
    }    

    return FTL_OK;        
}


/* =============================================
0. uboot cmd:
  a. ftltest pwr_cut init 1
  b. modify code: 
      in /uboot/arch/arm/lib/board.c  followed the nand_init(); add the code:
        #if 1 // power-cut test 
        {
            extern U32 NFTL_Init(U16 u16_StartPBA, U16 u16_PBACnt);
            extern U32 FtlTest_PwrCutTestCheck(void);
            extern U32 FtlTest_PwrCutTestRun(U32 LoopCnt);
            extern U16 FTL_Disable_ErrLog;
            extern void NFTL_ResetReadyFlag(void);

            puts ("NAND-  ");
            FTL_Disable_ErrLog=0; 
            NFTL_ResetReadyFlag();
            NFTL_Init(0, 0);
            FtlTest_PwrCutTestCheck();
            FtlTest_PwrCutTestRun(0x3697);
        }
        #endif
  c. update mboot.bin, AC on auto test.  
1. seed: 1 sector 1 seed, 1 seed keeps an U32.
2. 1st sector: Test Info: Flags, this SecAdr SecCnt Seed.
    Flags: 0: update TestInfo & old old seeds.
             1: update data
             2. update new seed             
3. old seeds sectors: store old seeds.
4. new seeds sectors: store new seeds.
5. test steps:
    a. init
    b. update Test Info (St.0)
    c. update data.
    d. update Test Info (St.1)
    e. update new seeds.
    f. random power-loss @ step.b~e.
    g. check (may be not)
    h. goto step.b.    
// =============================================*/
#define PWRCUT_AREA_SECTOR_ARD    0x100 // start sector addr for the test area
#define PWRCUT_DATA_SECTOR_CNT    (U32)(8*0x100000>>9)

#define PWRCUT_ST_DATA    0
#define PWRCUT_ST_SEED    1

typedef UNFD_PACK0 struct _FTL_TESTINFO
{
    U32 StFlag, SecAdr, SecCnt, Seed;
    
}UNFD_PACK1 FTL_TESTINFO, *P_FTL_TESTINFO;

UNFD_ALIGN0 FTL_TESTINFO FtlPwrCutTestInfo UNFD_ALIGN1;
static U32 *pSeedBuf=NULL;

extern int Disable_Err_log; // nand driver

void FtlTest_PwrCutTestDumpTInfo(void)
{
    ftl_dbg(FTL_DBG_LV_LOG,0,"\nTestInfo\n");
    ftl_dbg(FTL_DBG_LV_LOG,0,"StFlag: %s  Seed: %Xh\n", 
        FtlPwrCutTestInfo.StFlag==PWRCUT_ST_DATA?"Data":"Seed", 
        FtlPwrCutTestInfo.Seed);
    ftl_dbg(FTL_DBG_LV_LOG,0,"SecAdr: %Xh  SecCnt: %Xh\n", 
        FtlPwrCutTestInfo.SecAdr, FtlPwrCutTestInfo.SecCnt);
}

void FtlTest_PwrCutTestDump(void)
{
    U32 SeedByteCnt, SeedSecCnt;

    ftl_dbg(FTL_DBG_LV_LOG,0,"\n==============================================\n");
    ftl_dbg(FTL_DBG_LV_LOG,0,"TestByteCnt: %Xh  TestSecCnt: %Xh\n",
        PWRCUT_DATA_SECTOR_CNT<<9, PWRCUT_DATA_SECTOR_CNT);

    SeedByteCnt = PWRCUT_DATA_SECTOR_CNT*sizeof(U32);
    SeedSecCnt = SeedByteCnt>>9;
    ftl_dbg(FTL_DBG_LV_LOG,0,"SeedByteCnt: %Xh  SeedSecCnt: %Xh\n", 
        SeedByteCnt, SeedByteCnt>>9);
    
    ftl_dbg(FTL_DBG_LV_LOG,0,"TestInfoSecAdr: %Xh\n", PWRCUT_AREA_SECTOR_ARD);
    ftl_dbg(FTL_DBG_LV_LOG,0,"SeedSecAdr: %Xh\n", PWRCUT_AREA_SECTOR_ARD+1);
    ftl_dbg(FTL_DBG_LV_LOG,0,"DataSecAdr: %Xh ~ %Xh\n", 
        PWRCUT_AREA_SECTOR_ARD+1+SeedSecCnt, 
        PWRCUT_AREA_SECTOR_ARD+1+SeedSecCnt+PWRCUT_DATA_SECTOR_CNT-1);

    FtlTest_PwrCutTestDumpTInfo();
    ftl_dbg(FTL_DBG_LV_LOG,0,"==============================================\n");
}


U32 FtlTest_PwrCutUpdateSeed(U32 seed)
{
    volatile U32 SeedByteCnt, SeedSecCnt, SeedSecAdr, DataSecAdr;
    U32 i, j;

    ftl_trace();

    SeedByteCnt = PWRCUT_DATA_SECTOR_CNT * sizeof(U32);
    SeedSecCnt = SeedByteCnt>>9;
    if(SeedByteCnt & ((1<<9)-1))
        SeedSecCnt++;
    SeedSecAdr = PWRCUT_AREA_SECTOR_ARD+1;
    DataSecAdr = SeedSecAdr + SeedSecCnt;
    
    FtlPwrCutTestInfo.StFlag = PWRCUT_ST_SEED;
    if(NFTL_WriteData((U8*)&FtlPwrCutTestInfo, 1, PWRCUT_AREA_SECTOR_ARD))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"write TestInfo fail \n");
        ftl_die();
    }

    i = FtlPwrCutTestInfo.SecAdr - DataSecAdr;
    for(j=i; j<i+FtlPwrCutTestInfo.SecCnt; j++)
        pSeedBuf[j] = FtlPwrCutTestInfo.Seed;
    
    if(NFTL_WriteData((U8*)pSeedBuf, SeedSecCnt, SeedSecAdr))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"write seed fail \n");
        ftl_die();
    }
    return FTL_OK;
}


U32 FtlTest_PwrCutTestRun_Ex(void)
{
    FtlTest_PwrCutTestDumpTInfo();
    
    if(NFTL_WriteData((U8*)&FtlPwrCutTestInfo, 1, PWRCUT_AREA_SECTOR_ARD))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"write TestInfo fail \n");
        ftl_die();
    }

    if(FtlTest_DataWO(FtlPwrCutTestInfo.SecAdr, 
        FtlPwrCutTestInfo.SecCnt, FtlPwrCutTestInfo.Seed))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"write data fail \n");
        ftl_die();
    }

    if(FtlTest_PwrCutUpdateSeed(FtlPwrCutTestInfo.Seed))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"write seed fail \n");
        ftl_die();
    }

    return FTL_OK;
}


U32 FtlTest_PwrCutTestInit(void)
{
    volatile U32 SeedByteCnt, SeedSecCnt, SeedSecAdr, DataSecAdr;
    
    FtlTest_PwrCutTestDump();
    
    SeedByteCnt = PWRCUT_DATA_SECTOR_CNT * sizeof(U32);
    SeedSecCnt = SeedByteCnt>>9;
    if(SeedByteCnt & ((1<<9)-1))
        SeedSecCnt++;
    SeedSecAdr = PWRCUT_AREA_SECTOR_ARD+1;
    DataSecAdr = SeedSecAdr + SeedSecCnt;

    if(NULL==pSeedBuf)
    {
        pSeedBuf = FMALLOC(SeedByteCnt);
        if(NULL==pSeedBuf)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"alloc pSeedBuf fail: %Xh \n", SeedByteCnt);
            ftl_die();
        }
    }    
    // --------------------------
    FtlPwrCutTestInfo.StFlag = PWRCUT_ST_DATA;
    FtlPwrCutTestInfo.SecAdr = DataSecAdr;
    FtlPwrCutTestInfo.SecCnt = PWRCUT_DATA_SECTOR_CNT;
    FtlPwrCutTestInfo.Seed = 0;
    FtlTest_PwrCutTestRun_Ex();

    ftl_dbg(0,0,"\nfill dummy %Xh sectors before/after test area..\n", PWRCUT_AREA_SECTOR_ARD);
    FtlTest_DataWRC(0,PWRCUT_AREA_SECTOR_ARD,0);
    FtlTest_DataWRC(DataSecAdr+PWRCUT_DATA_SECTOR_CNT,
        PWRCUT_AREA_SECTOR_ARD,0);        
    
    return FtlTest_PwrCutTestCheck();
}


U32 FtlTest_PwrCutTestRC(U32 u32_SectorAddr, U32 u32_SectorCnt, U32 If2Seed)
{
    volatile U32 u32_i, u32_j, u32_k, u32_BufSecCnt, u32_SecCntTmp;
    U32 *pu32_1=(U32*)pu8_FtlTestDatBuf, *pu32_2=(U32*)pu8_FtlTestTmpBuf;
    volatile U32 SeedByteCnt, SeedSecCnt, SeedSecAdr, DataSecAdr;
    int flag=Disable_Err_log;

    u32_BufSecCnt = ((U32)pu32_2 - (U32)pu32_1)>>9;
    SeedByteCnt = PWRCUT_DATA_SECTOR_CNT * sizeof(U32);
    SeedSecCnt = SeedByteCnt>>9;
    if(SeedByteCnt & ((1<<9)-1))
        SeedSecCnt++;
    SeedSecAdr = PWRCUT_AREA_SECTOR_ARD+1;
    DataSecAdr = SeedSecAdr + SeedSecCnt;

    for(u32_k=0; u32_k<u32_SectorCnt; u32_k+=u32_SecCntTmp)
    {
        u32_SecCntTmp = u32_BufSecCnt > u32_SectorCnt-u32_k ?
            u32_SectorCnt-u32_k : u32_BufSecCnt;

        if(NFTL_ReadData((U8*)pu32_2, u32_SecCntTmp, u32_SectorAddr+u32_k))
            return FTL_ERR_TEST_FAIL;
        
        for(u32_i=0; u32_i<u32_SecCntTmp; u32_i++)
            for(u32_j=0; u32_j<512/4; u32_j++)
                pu32_1[(u32_i<<9)/4+u32_j] = u32_SectorAddr+u32_i+u32_k+
                    pSeedBuf[u32_SectorAddr+u32_i+u32_k-DataSecAdr];
            
        if(0==If2Seed && nand_CompareData((U8*)pu32_1, (U8*)pu32_2, u32_SecCntTmp<<9))
        {
            ftl_die();
        }
        else
        {   flag = Disable_Err_log;
            Disable_Err_log = 1;
            for(u32_i=0; u32_i<u32_SecCntTmp; u32_i++)
            {
                if(UNFD_ST_SUCCESS==nand_CompareData(
                    (U8*)&pu32_1[(u32_i<<9)/4], (U8*)&pu32_2[(u32_i<<9)/4], 1<<9))
                    continue;

                for(u32_j=0; u32_j<512/4; u32_j++)
                    pu32_1[(u32_i<<9)/4+u32_j] = u32_SectorAddr+u32_i+u32_k+
                        FtlPwrCutTestInfo.Seed;

                if(UNFD_ST_SUCCESS!=nand_CompareData(
                    (U8*)&pu32_1[(u32_i<<9)/4], (U8*)&pu32_2[(u32_i<<9)/4], 1<<9))
                {
                    Disable_Err_log = 0;
                    nand_CompareData(
                        (U8*)&pu32_1[(u32_i<<9)/4], (U8*)&pu32_2[(u32_i<<9)/4], 1<<9);
                    
                    for(u32_i=0; u32_i<u32_SecCntTmp; u32_i++)
                        for(u32_j=0; u32_j<512/4; u32_j++)
                            pu32_1[(u32_i<<9)/4+u32_j] = u32_SectorAddr+u32_i+u32_k+
                                pSeedBuf[u32_SectorAddr+u32_i+u32_k-DataSecAdr];
                    nand_CompareData(
                        (U8*)&pu32_1[(u32_i<<9)/4], (U8*)&pu32_2[(u32_i<<9)/4], 1<<9);
                    ftl_die();
                }                
            }
            Disable_Err_log = flag;
        }
    }
    
    return FTL_OK;
}

U32 FtlTest_PwrCutTestCheck(void)
{
    volatile U32 SeedByteCnt, SeedSecCnt, SeedSecAdr, DataSecAdr;
    //U32 i; int flag;
    U32 *pu32=(U32*)pu8_FtlTestDatBuf;
    
    // --------------------------
    ftl_dbg(0,0," \n");
    ftl_trace();
    
    SeedByteCnt = PWRCUT_DATA_SECTOR_CNT * sizeof(U32);
    SeedSecCnt = SeedByteCnt>>9;
    if(SeedByteCnt & ((1<<9)-1))
        SeedSecCnt++;
    SeedSecAdr = PWRCUT_AREA_SECTOR_ARD+1;
    DataSecAdr = SeedSecAdr + SeedSecCnt;
    if(NULL==pSeedBuf)
    {
        pSeedBuf = FMALLOC(SeedByteCnt);
        if(NULL==pSeedBuf)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"alloc pSeedBuf fail: %Xh \n", SeedByteCnt);
            ftl_die();
        }
    }
    // --------------------------
    // get TestInfo
    ftl_dbg(0,0,"\nread TestInfo \n");
    if(NFTL_ReadData((U8*)pu32, 1, PWRCUT_AREA_SECTOR_ARD))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"read TestInfo fail \n");
        ftl_die();
    }
    memcpy((void*)&FtlPwrCutTestInfo, (void*)pu32, sizeof(FTL_TESTINFO));
    FtlTest_PwrCutTestDump();

    // get seed
    ftl_dbg(0,0,"\nread Seeds \n");
    if(NFTL_ReadData((U8*)pSeedBuf, SeedSecCnt, SeedSecAdr))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"read TestInfo fail \n");
        ftl_die();
    }

    // --------------------------
    if(PWRCUT_ST_SEED == FtlPwrCutTestInfo.StFlag)
    {
        if(FtlTest_PwrCutUpdateSeed(FtlPwrCutTestInfo.Seed))
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"write seed fail \n");
            ftl_die(); 
        }
    }

    // --------------------------
    // always check data
    ftl_dbg(0,0,"\ncompare data \n");
    #if 1
    if(PWRCUT_ST_DATA == FtlPwrCutTestInfo.StFlag)
    {
        FtlTest_PwrCutTestRC(DataSecAdr, FtlPwrCutTestInfo.SecAdr-DataSecAdr, 0);
        FtlTest_PwrCutTestRC(FtlPwrCutTestInfo.SecAdr, FtlPwrCutTestInfo.SecCnt, 1);
        FtlTest_PwrCutTestRC(FtlPwrCutTestInfo.SecAdr+FtlPwrCutTestInfo.SecCnt, 
            DataSecAdr+PWRCUT_DATA_SECTOR_CNT-(FtlPwrCutTestInfo.SecAdr+FtlPwrCutTestInfo.SecCnt), 
            0);
    }
    else
        FtlTest_PwrCutTestRC(DataSecAdr, PWRCUT_DATA_SECTOR_CNT, 0);
    #else
    for(i=DataSecAdr; i<DataSecAdr+PWRCUT_DATA_SECTOR_CNT; i++)
    {
        if(PWRCUT_ST_DATA == FtlPwrCutTestInfo.StFlag &&
            i >= FtlPwrCutTestInfo.SecAdr && 
            i < FtlPwrCutTestInfo.SecAdr+FtlPwrCutTestInfo.SecCnt)
        {
            flag = Disable_Err_log;
            Disable_Err_log = 1;
            if(FtlTest_DataRC(i, 1, pSeedBuf[i-DataSecAdr]))
                if(FtlTest_DataRC(i, 1, FtlPwrCutTestInfo.Seed))
                {
                    Disable_Err_log = 0;
                    FtlTest_DataRC(i, 1, pSeedBuf[i-DataSecAdr]);
                    FtlTest_DataRC(i, 1, FtlPwrCutTestInfo.Seed);
                    ftl_die();
                }
            Disable_Err_log = flag;
        }
        else
        {   if(FtlTest_DataRC(i, 1, pSeedBuf[i-DataSecAdr]))
                ftl_die();
        }
    }
    #endif
    // --------------------------
    ftl_dbg(0,0,"\ncheck dummy %Xh sectors before/after test area..\n", PWRCUT_AREA_SECTOR_ARD);
    FtlTest_DataRC(0,PWRCUT_AREA_SECTOR_ARD,0);
    FtlTest_DataRC(DataSecAdr+PWRCUT_DATA_SECTOR_CNT,
        PWRCUT_AREA_SECTOR_ARD,0);
    
    // --------------------------    
    if(PWRCUT_ST_DATA == FtlPwrCutTestInfo.StFlag)
    {
        ftl_dbg(0,0,"\nfinish tested area..\n");
        FtlTest_PwrCutTestRun_Ex();
    }
    
    ftl_dbg(FTL_DBG_LV_LOG,1,"OK \n\n");
    return FTL_OK;
}


U32 FtlTest_PwrCutTestRun(U32 LoopCnt)
{
    volatile U32 SeedByteCnt, SeedSecCnt, SeedSecAdr, DataSecAdr;
    volatile U32 tick, Addr512B, Cnt512B, loop=LoopCnt;

    HWTimer_Start();
    SeedByteCnt = PWRCUT_DATA_SECTOR_CNT * sizeof(U32);
    SeedSecCnt = SeedByteCnt>>9;
    if(SeedByteCnt & ((1<<9)-1))
        SeedSecCnt++;
    SeedSecAdr = PWRCUT_AREA_SECTOR_ARD+1;
    DataSecAdr = SeedSecAdr + SeedSecCnt;
    if(NULL==pSeedBuf)
    {
        pSeedBuf = FMALLOC(SeedByteCnt);
        if(NULL==pSeedBuf)
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"alloc pSeedBuf fail: %Xh \n", SeedByteCnt);
            ftl_die();
        }
    }

    while(LoopCnt)
    {
        Addr512B = 0;  Cnt512B = 0;
        while(Addr512B < DataSecAdr || 
            Addr512B+Cnt512B > DataSecAdr+PWRCUT_DATA_SECTOR_CNT)
        {
            tick = HWTimer_End();
            Cnt512B = Ftl_Ext4BlkCnt(tick);
            Addr512B = tick % PWRCUT_DATA_SECTOR_CNT + DataSecAdr;
            Addr512B = (Addr512B>>3)<<3; // addr is 4KB-alighed.
            if(Addr512B < DataSecAdr)
                Addr512B = DataSecAdr;
            
            if(Addr512B+Cnt512B > DataSecAdr+PWRCUT_DATA_SECTOR_CNT)
                Addr512B -= Addr512B+Cnt512B-(DataSecAdr+PWRCUT_DATA_SECTOR_CNT);
        }

        FtlPwrCutTestInfo.StFlag = PWRCUT_ST_DATA;
        FtlPwrCutTestInfo.SecAdr = Addr512B;
        FtlPwrCutTestInfo.SecCnt = Cnt512B;
        FtlPwrCutTestInfo.Seed = HWTimer_End();
        FtlTest_PwrCutTestRun_Ex();        

        if(0x3697 != loop)
            LoopCnt--;
    }    

    return FTL_OK;
}



U8 FtlPwrOffTestFlag=0;
U32 FtlTest_PwrOffWaveform(void)
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U16 aPBA[3], i, cnt=0, p, loop=0;

    if(FTL_OK==NFTL_CheckIfReady())
    {
        memset(pNandDrv->pu8_PageDataBuf, 0, FtlDev.u16_PageByteCnt);
        memset(pNandDrv->pu8_PageSpareBuf, 0xFF, FtlDev.u16_SpareByteCnt);
    }
    else
    {   memset(pNandDrv->pu8_PageDataBuf, 0, pNandDrv->u16_PageByteCnt);
        memset(pNandDrv->pu8_PageSpareBuf, 0xFF, pNandDrv->u16_SpareByteCnt);
    }
    
    ftl_dbg(FTL_DBG_LV_LOG,1,"PlaneCnt: %Xh \n", 
        FTL_OK==NFTL_CheckIfReady()?FtlDev.u8_PlaneCnt:1);
    
    if(FTL_OK==NFTL_CheckIfReady())
    {
        // clean RootBlks
        for(i=0; i<ROOT_BLK_CNT; i++)
            if(!(NFTL_IsGoodBlk(i + FtlDev.u16_StartPBA) && 
                  FTL_OK==HAL_EraseBlk(i + FtlDev.u16_StartPBA)))
                  ftl_dbg(FTL_DBG_LV_ERR,1,"bad blk: %Xh \n", i + FtlDev.u16_StartPBA);

        // pick up 3 good blks
        for(i=FtlDev.u16_StartPBA+FtlDev.u16_PBACnt; i>0; i--)
            if(NFTL_IsGoodBlk(i-1))
            {
                aPBA[cnt] = i-1;
                cnt++;
                if(3==cnt)
                    break;
            }
    }
    else
    {   for(i=10; i>5; i--)
            //if(NFTL_IsGoodBlk(i-1) && FTL_OK==NC_EraseBlk(i-1))
            if(FTL_OK==NC_EraseBlk(i-1))
            {
                aPBA[cnt] = i-1;
                cnt++;
                if(3==cnt)
                    break;
            }
    }
    ftl_dbg(FTL_DBG_LV_LOG,1,"test PBA x%u: %Xh %Xh %Xh \n", 
        cnt, aPBA[0], aPBA[1], aPBA[2]);

    //FtlPwrOffTestFlag=1;
    ftl_dbg(0,1,"erase");
    nand_hw_timer_delay(1000000*5);
    ftl_dbg(0,0," start...\n");
    nand_hw_timer_delay(1000000);
    for(i=0; i<cnt; i++)
        if(FTL_OK==NFTL_CheckIfReady())
            (*FtlDev.pFnEraseBlk)(aPBA[i]);
        else
            NC_EraseBlk(aPBA[i]<<pNandDrv->u8_BlkPageCntBits);

    ftl_dbg(0,1,"write");
    nand_hw_timer_delay(1000000*5);
    ftl_dbg(0,0," start...\n");
    nand_hw_timer_delay(1000000);
    for(i=0; i<cnt; i++)
        if(FTL_OK==NFTL_CheckIfReady())
            for(p=0; p<FtlDev.u16_BlkPageCnt; p++)
                (*FtlDev.pFnWritePages)(BlkRowAddr(aPBA[i])+p, 
                    pNandDrv->pu8_PageDataBuf, pNandDrv->pu8_PageSpareBuf, 1);
        else
            for(p=0; p<pNandDrv->u16_BlkPageCnt; p++)
                NC_WritePages((aPBA[i]<<pNandDrv->u8_BlkPageCntBits)+p, 
                    pNandDrv->pu8_PageDataBuf, pNandDrv->pu8_PageSpareBuf, 1);
                

    ftl_dbg(0,1,"read");
    nand_hw_timer_delay(1000000*5);
    ftl_dbg(0,0," start...\n");
    nand_hw_timer_delay(1000000);

    for(loop=0; loop<20; loop++)
    for(i=0; i<cnt; i++)
        if(FTL_OK==NFTL_CheckIfReady())
            for(p=0; p<FtlDev.u16_BlkPageCnt; p++)
                (*FtlDev.pFnReadPages)(BlkRowAddr(aPBA[i])+p, 
                    pNandDrv->pu8_PageDataBuf, pNandDrv->pu8_PageSpareBuf, 1);
        else
            for(p=0; p<pNandDrv->u16_BlkPageCnt; p++)
                NC_ReadPages((aPBA[i]<<pNandDrv->u8_BlkPageCntBits)+p, 
                    pNandDrv->pu8_PageDataBuf, pNandDrv->pu8_PageSpareBuf, 1);

    // clean the blocks
    for(i=0; i<cnt; i++)
        if(FTL_OK==NFTL_CheckIfReady())
            (*FtlDev.pFnEraseBlk)(aPBA[i]);
        else
            NC_EraseBlk(aPBA[i]<<pNandDrv->u8_BlkPageCntBits);

    FtlPwrOffTestFlag=0;
    return FTL_OK;
}

// =============================================
#define SPEED_TEST_SIZE       (U32)(1*0x100000)   // n x 1MB
U32 u32_TestSecAddr=0;

U32 FtlTest_PerformanceEx(U32 u32_BurstSecCnt)
{
    U32 u32_t0, u32_t1, u32_t;
    U32 u32_i, u32_LoopCnt, u32_SecAddr;

    if(1==u32_BurstSecCnt)
        ftl_dbg(FTL_DBG_LV_LOG,0,"\n burst length: 512 bytes \n");
    else
        ftl_dbg(FTL_DBG_LV_LOG,0,"\n burst length: %u KB \n", u32_BurstSecCnt>>1);
    
    // --------------------------
    ftl_dbg(FTL_DBG_LV_LOG,0,"  w speed: ");
    FTL_Disable_ErrLog = 1;
    u32_LoopCnt = SPEED_TEST_SIZE/(u32_BurstSecCnt<<9);

    HWTimer_Start();
    u32_t = 0;
    for(u32_i=0; u32_i<u32_LoopCnt; u32_i++)
    {
        u32_SecAddr = HWTimer_End() % NFTL_GetDataCapacity()/2;
        u32_t0 = HWTimer_End();
        //NFTL_WriteData(pu8_FtlTestDatBuf, u32_BurstSecCnt, u32_TestSecAddr);//u32_SecAddr);
        NFTL_WriteData(pu8_FtlTestDatBuf, u32_BurstSecCnt, u32_SecAddr);
        u32_t1 = HWTimer_End();
        u32_t += u32_t1 > u32_t0 ? u32_t1-u32_t0 : 0xFFFFFFFF-u32_t0+u32_t1;
        //ftl_dbg(0,0,".");
    }
    u32_t = u32_t / (FTL_HW_TIMER_HZ/1000); // ms
    FTL_Disable_ErrLog = 0;
    ftl_dbg(FTL_DBG_LV_LOG,0,"%u.%u MB/s \n", 
        (SPEED_TEST_SIZE>>20)*1000/u32_t, ((SPEED_TEST_SIZE>>20)*10000/u32_t)%10);

    // --------------------------
    ftl_dbg(FTL_DBG_LV_LOG,0,"  r speed: ");
    FTL_Disable_ErrLog = 1;
    u32_LoopCnt = SPEED_TEST_SIZE/(u32_BurstSecCnt<<9);

    HWTimer_Start();
    u32_t = 0;
    for(u32_i=0; u32_i<u32_LoopCnt; u32_i++)
    {
        u32_SecAddr = HWTimer_End() % NFTL_GetDataCapacity()/2;
        u32_t0 = HWTimer_End();
        //NFTL_ReadData(pu8_FtlTestDatBuf, u32_BurstSecCnt, u32_TestSecAddr);//u32_SecAddr);
        NFTL_ReadData(pu8_FtlTestDatBuf, u32_BurstSecCnt, u32_SecAddr);
        u32_t1 = HWTimer_End();
        u32_t += u32_t1 > u32_t0 ? u32_t1-u32_t0 : 0xFFFFFFFF-u32_t0+u32_t1;
        //ftl_dbg(0,0,".");
    }
    u32_t = u32_t / (FTL_HW_TIMER_HZ/1000); // ms
    FTL_Disable_ErrLog = 0;
    ftl_dbg(FTL_DBG_LV_LOG,0,"%u.%u MB/s \n", 
        (SPEED_TEST_SIZE>>20)*1000/u32_t, ((SPEED_TEST_SIZE>>20)*10000/u32_t)%10);

    return FTL_OK;
}


U32 FtlTest_Performance(void)
{
    volatile U32 u32_BurstSecCnt=1;//, u32_i;

    ftl_dbg(FTL_DBG_LV_LOG,0,"===========================================\n");
	ftl_dbg(FTL_DBG_LV_LOG,0,"CAUTION: will damage images in NAND \n");
    ftl_dbg(FTL_DBG_LV_LOG,0,"Test Data Size: %u MB \n\n", SPEED_TEST_SIZE>>20);

    HWTimer_Start();

    LABEL_TEST_START:
    
    // --------------------------
    if(u32_BurstSecCnt == 1) // 512B
    {
        FtlTest_PerformanceEx(u32_BurstSecCnt);
        u32_BurstSecCnt = 8; 
    }
    else if(u32_BurstSecCnt == 8) // 4KB
    {
        FtlTest_PerformanceEx(u32_BurstSecCnt);
        u32_BurstSecCnt = 32; 
    }
    else if(u32_BurstSecCnt == 32) // 16KB
    {
        FtlTest_PerformanceEx(u32_BurstSecCnt);
        u32_BurstSecCnt = 256; 
    }
    else if(u32_BurstSecCnt == 256) // 128KB
    {
        FtlTest_PerformanceEx(u32_BurstSecCnt);
        u32_BurstSecCnt = 0x100000>>9; 
    }
    else if(u32_BurstSecCnt == 0x100000>>9) // 1MB
    {
        FtlTest_PerformanceEx(u32_BurstSecCnt);
        u32_BurstSecCnt = 0;
    }

    // --------------------------
    if(u32_BurstSecCnt != 0)
        goto LABEL_TEST_START;

    return FTL_OK;    
}


#define LOG_LOOP_CNT  0x1000
void FtlTest_Lifetime(U8 TestMode)
{
    U32 TotalSecCntH=0, TotalSecCntL=0, SecCnt_tmp=0, SecAddr=0;
    U32 TestLoop=0;

    HWTimer_Start();
    ftl_dbg(FTL_DBG_LV_LOG,0,"===========================================\n");
	ftl_dbg(FTL_DBG_LV_LOG,0,"CAUTION: will damage images in NAND \n");
    ftl_dbg(FTL_DBG_LV_LOG,0,"Test Mode: %s \n", 
        TestMode==FTL_LIFETIME_TEST_FIXED?"Fixed":
        TestMode==FTL_LIFETIME_TEST_FILLED?"Filled":
        TestMode==FTL_LIFETIME_TEST_RANDOM?"Random":"unKnown !!");
    ftl_dbg(FTL_DBG_LV_LOG,0,"total FTL sector count: %Xh, %u MB. \n", NFTL_GetDataCapacity(), NFTL_GetDataCapacity()>>11);
	ftl_dbg(FTL_DBG_LV_LOG,0,"===========================================\n");

    // --------------------------
    if(TestMode == FTL_LIFETIME_TEST_FIXED)
    {
        SecAddr = 0;
        SecCnt_tmp = FtlDev.u16_Page512BCnt;
    }   

    LABEL_TEST_START:

    TestLoop++;
    
    if(TestMode == FTL_LIFETIME_TEST_FILLED)
    {
        SecAddr += SecCnt_tmp;
        SecCnt_tmp = FtlDev.u16_Blk512BCnt;
        if(SecAddr+SecCnt_tmp > NFTL_GetDataCapacity())
            SecAddr = 0;
    }
    else if(TestMode == FTL_LIFETIME_TEST_RANDOM)
    {
        do{
            SecAddr = HWTimer_End() % NFTL_GetDataCapacity();
            SecCnt_tmp = HWTimer_End() % NFTL_GetDataCapacity();
        }while(SecAddr+SecCnt_tmp > NFTL_GetDataCapacity() ||
            SecAddr+SecCnt_tmp < SecAddr || SecAddr+SecCnt_tmp < SecCnt_tmp ||
            SecAddr==0 || SecCnt_tmp==0);
    }

    // --------------------------
    ftl_dbg(FTL_DBG_LV_LOG,0,"############################### \n");
    ftl_dbg(FTL_DBG_LV_LOG,0,"SecAddr: %Xh  SecCnt: %Xh \n", SecAddr, SecCnt_tmp);
    ftl_dbg(FTL_DBG_LV_LOG,0,"############################### \n");
    if(FTL_OK != FtlTest_DataWRC(SecAddr, SecCnt_tmp, 0))
        return;

    if(TotalSecCntL + SecCnt_tmp > TotalSecCntL) // no overflow
        TotalSecCntL += SecCnt_tmp;
    else
    {   TotalSecCntH++;
        TotalSecCntL = SecCnt_tmp - (0xFFFFFFFF-TotalSecCntL);
    }

    // --------------------------
    if(0 == TestLoop%LOG_LOOP_CNT)
    {
        if(0==TotalSecCntH)
            ftl_dbg(FTL_DBG_LV_LOG,0,"%u MB        \r", TotalSecCntL>>11);
        else
            ftl_dbg(FTL_DBG_LV_LOG,0,"%u x 2TB + %u MB        \r",
                TotalSecCntH, TotalSecCntL>>11);
    }
    
    goto LABEL_TEST_START;
}

// =============================================
U32 FtlTest_DataWRC(U32 u32_SectorAddr, U32 u32_SectorCnt, U32 seed)
{
    volatile U32 u32_i, u32_j, u32_k, u32_BufSecCnt, u32_SecCntTmp;
    volatile U32 *pu32_1=(U32*)pu8_FtlTestDatBuf, *pu32_2=(U32*)pu8_FtlTestTmpBuf;

    u32_BufSecCnt = ((U32)pu32_2 - (U32)pu32_1)>>9;
    u32_k=0;

    LABEL_TEST:

    u32_SecCntTmp = u32_BufSecCnt > u32_SectorCnt-u32_k ?
        u32_SectorCnt-u32_k : u32_BufSecCnt;
    
    for(u32_i=0; u32_i<u32_SecCntTmp; u32_i++)
        for(u32_j=0; u32_j<512/4; u32_j++)
        {
            pu32_1[(u32_i<<9)/4+u32_j] = u32_SectorAddr+u32_i+u32_k+seed;
            //pu32_2[(u32_i<<9)/4+u32_j] = ~pu32_1[(u32_i<<9)/4+u32_j];
        }

    if(NFTL_WriteData(pu8_FtlTestDatBuf, u32_SecCntTmp, u32_SectorAddr+u32_k))
        return FTL_ERR_TEST_FAIL;
        
    if(NFTL_ReadData(pu8_FtlTestTmpBuf, u32_SecCntTmp, u32_SectorAddr+u32_k))
        return FTL_ERR_TEST_FAIL;
        
    if(nand_CompareData(pu8_FtlTestDatBuf, pu8_FtlTestTmpBuf, u32_SecCntTmp<<9))
        return FTL_ERR_TEST_FAIL;

    u32_k += u32_SecCntTmp;
    if(u32_k < u32_SectorCnt)
        goto LABEL_TEST;
    
    return FTL_OK;
}


U32 FtlTest_DataRC(U32 u32_SectorAddr, U32 u32_SectorCnt, U32 seed)
{
    volatile U32 u32_i, u32_j, u32_k, u32_BufSecCnt, u32_SecCntTmp;
    U32 *pu32_1=(U32*)pu8_FtlTestDatBuf, *pu32_2=(U32*)pu8_FtlTestTmpBuf;

    u32_BufSecCnt = ((U32)pu32_2 - (U32)pu32_1)>>9;

    for(u32_k=0; u32_k<u32_SectorCnt; u32_k+=u32_SecCntTmp)
    {
        u32_SecCntTmp = u32_BufSecCnt > u32_SectorCnt-u32_k ?
            u32_SectorCnt-u32_k : u32_BufSecCnt;

        for(u32_i=0; u32_i<u32_SecCntTmp; u32_i++)
            for(u32_j=0; u32_j<512/4; u32_j++)
            {
                pu32_1[(u32_i<<9)/4+u32_j] = u32_SectorAddr+u32_i+u32_k+seed;
                pu32_2[(u32_i<<9)/4+u32_j] = 0;
            }

        if(NFTL_ReadData((U8*)pu32_2, u32_SecCntTmp, u32_SectorAddr+u32_k))
            return FTL_ERR_TEST_FAIL;
            
        if(nand_CompareData((U8*)pu32_1, (U8*)pu32_2, u32_SecCntTmp<<9))
            return FTL_ERR_TEST_FAIL;
    }
    return FTL_OK;
}


U32 FtlTest_DataWO(U32 u32_SectorAddr, U32 u32_SectorCnt, U32 seed)
{
    volatile U32 u32_i, u32_j, u32_k, u32_BufSecCnt, u32_SecCntTmp;
    U32 *pu32_1=(U32*)pu8_FtlTestDatBuf, *pu32_2=(U32*)pu8_FtlTestTmpBuf;

    u32_BufSecCnt = (U32)pu32_2 - (U32)pu32_1;
    u32_BufSecCnt >>= 9;

    for(u32_k=0; u32_k<u32_SectorCnt; u32_k+=u32_SecCntTmp)
    {
        u32_SecCntTmp = u32_BufSecCnt > u32_SectorCnt-u32_k ?
            u32_SectorCnt-u32_k : u32_BufSecCnt;

        for(u32_i=0; u32_i<u32_SecCntTmp; u32_i++)
            for(u32_j=0; u32_j<512/4; u32_j++)
                pu32_1[(u32_i<<9)/4+u32_j] = u32_SectorAddr+u32_i+u32_k+seed;
            
        if(NFTL_WriteData((U8*)pu32_1, u32_SecCntTmp, u32_SectorAddr+u32_k))
            return FTL_ERR_TEST_FAIL;            
    }
    return FTL_OK;
}


U32 Ftl_Ext4BlkCnt(U32 tick)
{
    if(tick % 10 < 4) // 16KB            
        return 16*1024>>9;
    else if(tick % 10 < 5) // 4KB            
        return 4*1024>>9;
    else if(tick % 10 < 6) // 64KB            
        return 64*1024>>9;
    else if(tick % 10 < 7) // 32KB            
        return 32*1024>>9;
    else if(tick % 10 < 8) // 128KB            
        return 128*1024>>9;
    else if(tick % 10 < 9) // 256KB            
        return 256*1024>>9;
    else if(tick % 10 < 10) // 1MB            
        return 1024*1024>>9;
    else
        return 2048*1024>>9; // 2MB
}


U32 Ftl_Test_Case(U32 u32_Flag) // for special cases
{
    U32 totalSecCnt, i, Loop, x;

    totalSecCnt = NFTL_GetDataCapacity();
    if(totalSecCnt & (4096/512-1))
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"TotalSecCnt: %Xh not aligned %Xh \n", totalSecCnt, 4096/512);
        return FTL_OK;
    }
    Loop = u32_Flag;

    while(u32_Flag)
    {
        ftl_dbg(0,0,"\n\nTest Loop: %u==============\n", Loop-u32_Flag);
        
        for(i=0; i<totalSecCnt; i+=4096/512)
        {
            x = i*100/totalSecCnt;
            if(0==i%100)
                ftl_dbg(0,0,"%02u%% \r", x);
            
            FtlTest_DataWO(i, 4096/512, 0);
        }
        ftl_dbg(0,0,"100%% \n\n");

        for(i=0; i<totalSecCnt; i+=4096/512)
            FtlTest_DataRC(i, 4096/512, 0);

        if(Loop != 3697)
            u32_Flag--;        
    }

    return FTL_OK;
}


U32 Ftl_Test(U32 u32_Flag)
{
    volatile U32 u32_err=FTL_OK, tick, Loop;
    volatile U32 Addr512B, Cnt512B, Addr512B_Prev=0, Cnt512B_Prev=0;

    HWTimer_Start();
    Loop = u32_Flag;
    
    while(u32_Flag)
    {
        // --------------------------
        //ftl_dbg(0,0,"\n\nTest Loop: %u==============\n", Loop-u32_Flag);
        ftl_dbg(0,0,".");

        tick = HWTimer_End();
        
        #if 0 // random pick up 512BAddr & 512BCnt
        
        Addr512B = tick % NFTL_GetDataCapacity();        
        tick = HWTimer_End();
        Cnt512B = tick % ((U32)pu8_FtlTestTmpBuf-(U32)pu8_FtlTestDatBuf) >>9;

        if(Addr512B+Cnt512B > NFTL_GetDataCapacity())
            Cnt512B = NFTL_GetDataCapacity() - Addr512B;

        if(0==Cnt512B)
            Cnt512B = 1;
        
        #else // sim the ext4 behavior
        
        Cnt512B = Ftl_Ext4BlkCnt(tick);
        #if 1
        Addr512B = tick % NFTL_GetDataCapacity(); // random addr
        #else
        Addr512B = tick % (0x400000/0x200); // addr 4MB limited
        #endif
        
        Addr512B = (Addr512B>>3)<<3; // addr is 4KB-alighed.
        
        if(Addr512B+Cnt512B > NFTL_GetDataCapacity())
           Addr512B -= Addr512B+Cnt512B-NFTL_GetDataCapacity();
        #endif

//Addr512B=0x24547;  Cnt512B=0x52;

        //ftl_dbg(0,0,"Test: %8Xh + %8Xh = %Xh \n\n", 
        //    Addr512B, Cnt512B, Addr512B+Cnt512B);

        // --------------------------
        //FTL_Disable_ErrLog=1;
        if(FTL_OK != FtlTest_DataWRC(Addr512B, Cnt512B, 0))
        {
            NFTL_DumpInfo();
            return 1;
        }
        //FTL_Disable_ErrLog=0;

        #if 0
        if(0 == u32_Flag % 100)
            NFTL_DumpECnt();
        #else
        NFTL_BKG_Housekeep();
        #endif

        // --------------------------
        #if 0
        if(0 == u32_Flag % 100){
        ftl_dbg(0,0,"\n......................................\n");
        ftl_dbg(0,0,"......................................\n");
        NFTL_DumpDBPM();
        NFTL_DumpData();
        NFTL_DumpDBPM_Details();

        //NFTL_DumpRowL2P();
        
        ftl_dbg(0,0,"\n......................................\n");        
        ftl_dbg(0,0,"[re-init]\n");
        NFTL_ResetReadyFlag();
        NFTL_Init(0,0);
        
        ftl_dbg(0,0,"\n\n......................................\n");
        NFTL_DumpDBPM();
        NFTL_DumpData();
        NFTL_DumpDBPM_Details();
        //NFTL_DumpRowL2P();
        NFTL_DumpECnt();
        
        ftl_dbg(0,0,"\n......................................\n");
        ftl_dbg(0,0,"......................................\n");
        ftl_dbg(0,0,"[DataRC 1]\n");
        if(FTL_OK != FtlTest_DataRC(Addr512B, Cnt512B, 0))
        {
            ftl_dbg(FTL_DBG_LV_ERR,1,"RC fail \n");
            return 1;
        }        
        if(0!=Cnt512B_Prev)
        {
            ftl_dbg(0,0,"\n......................................\n");
            ftl_dbg(0,0,"[DataRC 2]\n");
            if(FTL_OK != FtlTest_DataRC(Addr512B_Prev, Cnt512B_Prev, 0))
            {
                ftl_dbg(FTL_DBG_LV_ERR,1,"RC Prev fail \n");
                return 1;
            }
        }

        ftl_dbg(FTL_DBG_LV_LOG,0,"\n OK \n");
        }        
        #endif

        Addr512B_Prev = Addr512B;
        Cnt512B_Prev = Cnt512B;
        u32_Flag--;
    }    

    return u32_err;
}


#define FTL_BURN_TOTAL_SECTOR_CNT    (FtlDev.u16_BlkSectorCnt)
#define FTL_BURN_RANGE_SECTOR_CNT    FTL_BURN_TOTAL_SECTOR_CNT*2
#define FTL_BURN_BURST_SECTOR_CNT    (FtlDev.u16_Page512BCnt)

U32 Ftl_Burn(U32 u32_LoopCnt)
{
    volatile U32 err=FTL_OK, TestSectorCnt, u32_i, Burst512Cnt, seed;

    HWTimer_Start();
    
    TestSectorCnt = 
        FTL_BURN_TOTAL_SECTOR_CNT > NFTL_GetDataCapacity() ?
        NFTL_GetDataCapacity() : FTL_BURN_TOTAL_SECTOR_CNT;

    if(TestSectorCnt > FTL_BURN_RANGE_SECTOR_CNT)
        TestSectorCnt = FTL_BURN_RANGE_SECTOR_CNT-1;

    Burst512Cnt = 1;

    while(u32_LoopCnt)
    {
        LABEL_INNER_LOOP:
        ftl_dbg(0,0,"\n\n======================================\n");
        ftl_dbg(0,0,"BurstSecCnt: %Xh\n", Burst512Cnt);

        for(u32_i=0; u32_i<TestSectorCnt; u32_i+=Burst512Cnt)
        {
            seed = HWTimer_End();
            ftl_dbg(0,0,"--------------------------\n");
            ftl_dbg(0,0,"TestSecAdr: %Xh  seed: %Xh\n", u32_i, seed);
            ftl_dbg(0,0,"--------------------------\n");

            err = FtlTest_DataWRC(u32_i, Burst512Cnt, seed);
            if(FTL_OK!=err)
            {
                ftl_dbg(FTL_DBG_LV_ERR,1,"Now DataWRC fail: Adr: %Xh, Cnt: %Xh \n", 
                    u32_i, Burst512Cnt);
                ftl_die();
            }                
            
            err = FtlTest_DataRC(u32_i, Burst512Cnt, seed);
            if(FTL_OK!=err)
            {
                ftl_dbg(FTL_DBG_LV_ERR,1,"DataRC fail. \n");
                ftl_die();
            }
        }

        // --------------------------
        for(u32_i=TestSectorCnt-1; u32_i>0; u32_i-=Burst512Cnt)
        {
            seed = HWTimer_End();
            ftl_dbg(0,0,"--------------------------\n");
            ftl_dbg(0,0,"TestSecAdr: %Xh  seed: %Xh\n", u32_i, seed);
            ftl_dbg(0,0,"--------------------------\n");
            
            err = FtlTest_DataWRC(u32_i, Burst512Cnt, seed);
            if(FTL_OK!=err)
            {
                ftl_dbg(FTL_DBG_LV_ERR,1,"Now DataWRC fail: Adr: %Xh, Cnt: %Xh \n", 
                    u32_i, Burst512Cnt);
                ftl_die();
            }                
            
            err = FtlTest_DataRC(u32_i, Burst512Cnt, seed);
            if(FTL_OK!=err)
            {
                ftl_dbg(FTL_DBG_LV_ERR,1,"DataRC fail. \n");
                ftl_die();
            }

            if(u32_i < Burst512Cnt)
                break;
        }
        
        // --------------------------
        Burst512Cnt++;
        
        if(Burst512Cnt > FTL_BURN_BURST_SECTOR_CNT)
        {
            Burst512Cnt = 1;
            u32_LoopCnt--;        
        }
        else
            goto LABEL_INNER_LOOP;
        
    }

    return err;
}


