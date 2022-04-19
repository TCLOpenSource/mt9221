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

#include "drvNAND.h"
#include "drvNAND_utl.h"

typedef struct {
    const char *msg;
    U8  u8_OpCode_RW_AdrCycle;
    U8  u8_OpCode_Erase_AdrCycle;
    U16 u16_Reg48_Spare;
} drvNAND_ReadSeq;

static const drvNAND_ReadSeq seq[] = {
    {"R_SEQUENCE_003A", ADR_C3TFS0, ADR_C2TRS0, BIT_NC_ONE_COL_ADDR},
    {"R_SEQUENCE_004A", ADR_C4TFS0, ADR_C3TRS0, BIT_NC_ONE_COL_ADDR},
    {"R_SEQUENCE_004A30", ADR_C4TFS0, ADR_C2TRS0, 0},
    {"R_SEQUENCE_005A30", ADR_C5TFS0, ADR_C3TRS0, 0},
    {"R_SEQUENCE_006A30", ADR_C6TFS0, ADR_C3TRS0, 0}, // FIXME
    {NULL, 0, 0, 0},
};

U32 drvNAND_ProbeReadSeq(void)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
    U8 u8_seq_idx = 0;
    U32 u32_Err;
    drvNAND_ReadSeq const *pSeq = seq;

    pNandDrv->u16_BlkCnt = 0x400;
    pNandDrv->u16_BlkPageCnt = 0x40;
    pNandDrv->u16_PageByteCnt = 0x200;
    pNandDrv->u16_SpareByteCnt = 0x10;
    pNandDrv->u16_ECCType = ECC_TYPE_4BIT;
    pNandDrv->u8_Flag_004A30 = 0;

    NC_ConfigContext();
    u32_Err = NC_Init();
    if (u32_Err != UNFD_ST_SUCCESS)
        return u32_Err;

    if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_TOGGLE)
    {
        pSeq +=3;
        u8_seq_idx += 3;
    }

    /* Probe read sequence */
    while (pSeq->msg) {

        nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "%s\n", pSeq->msg);
        pNandDrv->u8_OpCode_RW_AdrCycle = pSeq->u8_OpCode_RW_AdrCycle;
        pNandDrv->u8_OpCode_Erase_AdrCycle = pSeq->u8_OpCode_Erase_AdrCycle;
        pNandDrv->u16_Reg48_Spare &= ~BIT_NC_ONE_COL_ADDR;
        pNandDrv->u16_Reg48_Spare |= pSeq->u16_Reg48_Spare;

        NC_Config();

        u32_Err = NC_ProbeReadSeq();
        if (u32_Err == UNFD_ST_SUCCESS)
        {
            // if R_SEQUENCE_004A30
            if(ADR_C4TFS0 == pNandDrv->u8_OpCode_RW_AdrCycle &&
               0 == (pNandDrv->u16_Reg48_Spare & BIT_NC_ONE_COL_ADDR))
            {
                pNandDrv->u8_Flag_004A30 = 1;
            }
            pNandDrv->u8_AddrCycleIdx = u8_seq_idx;
            return UNFD_ST_SUCCESS;
        }
        NC_ResetFCIE(); // <- should be redundant code (wait for test then remove)
        pSeq++;
        u8_seq_idx ++;
    }

    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: Unknown RSEQ\n");

    return UNFD_ST_ERR_UNKNOWN_RSEQ;
}

U32 drvNAND_CheckPartInfo(U32 u32_PageIdx)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
#if NAND_BUF_USE_STACK
    U8 au8_SectorBuf[1024];
    U8 au8_SpareBuf[64];
#else
    U8 *au8_SectorBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    U8 *au8_SpareBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf;
#endif
    U32 u32_Err;
    U32 u32_ChkSum;
    U32 u32_ByteCnt;

    PARTITION_INFO_t *pPartInfo = (PARTITION_INFO_t*)au8_SectorBuf;

    //u32_Err = NC_ReadSector_RIUMode(u32_PageIdx, 0, au8_SectorBuf, au8_SpareBuf);
    u32_Err = NC_ReadSectors_MTD(u32_PageIdx, 0, au8_SectorBuf, au8_SpareBuf, 1);
    if (u32_Err != UNFD_ST_SUCCESS)
        return u32_Err;

    u32_ChkSum = drvNAND_CheckSum(au8_SectorBuf + 0x04, 0x200 - 0x04);
    if (u32_ChkSum != pPartInfo->u32_ChkSum) {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Skip PartInfo (ChkSum: 0x%08x != 0x%08x)\n",
                   u32_ChkSum, pPartInfo->u32_ChkSum);

        return UNFD_ST_ERR_CIS_PART_ERR;
    }

    u32_ByteCnt = pPartInfo->u16_PartCnt * pPartInfo->u16_UnitByteCnt;
    memcpy(pNandDrv->pPartInfo, pPartInfo, sizeof(*pPartInfo) + u32_ByteCnt);

    dump_part_info(pPartInfo);
    return UNFD_ST_SUCCESS;
}

void drvNAND_ParseNandInfo(NAND_FLASH_INFO_t *pNandInfo)
{
    int i;
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();

    pNandDrv->u16_PageByteCnt  = pNandInfo->u16_PageByteCnt;
    pNandDrv->u16_SpareByteCnt = pNandInfo->u16_SpareByteCnt;
    pNandDrv->u16_BlkPageCnt   = pNandInfo->u16_BlkPageCnt;
    pNandDrv->u16_BlkCnt       = pNandInfo->u16_BlkCnt;
    pNandDrv->u16_ECCType      = pNandInfo->u16_ECCType;
    pNandDrv->u8_IDByteCnt     = pNandInfo->u8_IDByteCnt;
    pNandDrv->u32_Config       = pNandInfo->u32_Config;
    pNandDrv->u16_tRC          = pNandInfo->u16_tRC;
    pNandDrv->u8_tRP           = pNandInfo->u8_tRP;
    pNandDrv->u8_tREH          = pNandInfo->u8_tREH;
    pNandDrv->u8_tREA          = pNandInfo->u8_tREA;
    pNandDrv->u8_tRR           = pNandInfo->u8_tRR;
    pNandDrv->u16_tADL         = pNandInfo->u16_tADL;
    pNandDrv->u16_tRHW         = pNandInfo->u16_tRHW;
    pNandDrv->u16_tWHR         = pNandInfo->u16_tWHR;
    pNandDrv->u16_tCCS         = pNandInfo->u16_tCCS;
    pNandDrv->u8_tCS           = pNandInfo->u8_tCS;
    pNandDrv->u16_tWC          = pNandInfo->u16_tWC;
    pNandDrv->u8_tWP           = pNandInfo->u8_tWP;
    pNandDrv->u8_tWH           = pNandInfo->u8_tWH;
    pNandDrv->u16_tCWAW        = pNandInfo->u16_tCWAW;
    pNandDrv->u8_tCLHZ         = pNandInfo->u8_tCLHZ;
    pNandDrv->u16_tWW          = pNandInfo->u16_tWW;
    pNandDrv->u8_AddrCycleIdx  = pNandInfo->u8_AddrCycleIdx;

    if(pNandDrv->u8_AddrCycleIdx != 0)
    {
        pNandDrv->u8_OpCode_Erase_AdrCycle = seq[pNandDrv->u8_AddrCycleIdx].u8_OpCode_Erase_AdrCycle;
        pNandDrv->u8_OpCode_RW_AdrCycle = seq[pNandDrv->u8_AddrCycleIdx].u8_OpCode_RW_AdrCycle;
        pNandDrv->u16_One_Col_Addr = seq[pNandDrv->u8_AddrCycleIdx].u16_Reg48_Spare;
        pNandDrv->u16_Reg48_Spare &= ~BIT_NC_ONE_COL_ADDR;
        pNandDrv->u16_Reg48_Spare |= seq[pNandDrv->u8_AddrCycleIdx].u16_Reg48_Spare;
    }

    memcpy(pNandDrv->u8_Vendor, pNandInfo->u8_Vendor, 16);
    memcpy(pNandDrv->u8_PartNumber, pNandInfo->u8_PartNumber, 16);

    pNandDrv->u8_CellType      = pNandDrv->u32_Config&0x1;
    pNandDrv->u8_BadBlkMarker      = (pNandDrv->u32_Config>>1)&0x7;
    pNandDrv->u8_PlaneCnt      = ((pNandDrv->u32_Config>>4)&0x7)+1;
    pNandDrv->u8_RequireRandomizer      = (pNandDrv->u32_Config>>8) & 0x1;
    pNandDrv->u8_NANDInterface  = ((pNandDrv->u32_Config>>9) & 3);
    pNandDrv->u8_CacheProgram   = ((pNandDrv->u32_Config >> 11) & 3);
    pNandDrv->u8_CacheRead      = ((pNandDrv->u32_Config >> 13) & 3);
    pNandDrv->u8_RequireReadRetry = ((pNandDrv->u32_Config>>15) & 1);
    pNandDrv->u8_RequireFullBlkPrg = ((pNandDrv->u32_Config>>20) & 1);

    pNandDrv->u8_PairPageMapLoc = pNandInfo->u8_PairPageMapLoc;
    pNandDrv->u8_ReadRetryType =    pNandInfo->u8_ReadRetryType;
    pNandDrv->u16_BitflipThreshold = pNandInfo->u8_BitflipThreshold;
    #if defined(FCIE4_DDR) && FCIE4_DDR || defined(DDR_NAND_SUPPORT) && DDR_NAND_SUPPORT
    memcpy((void *) &pNandDrv->tDefaultDDR, (const void *) &pNandInfo->tDefaultDDR, sizeof(DDR_TIMING_GROUP_t));
    memcpy((void *) &pNandDrv->tMaxDDR, (const void *) &pNandInfo->tMaxDDR, sizeof(DDR_TIMING_GROUP_t));
    memcpy((void *) &pNandDrv->tMinDDR, (const void *) &pNandInfo->tMinDDR, sizeof(DDR_TIMING_GROUP_t));
    #endif

    pNandDrv->u8_BL0PBA = pNandInfo->u8_BL0PBA;
    pNandDrv->u8_BL1PBA = pNandInfo->u8_BL1PBA;
    pNandDrv->u8_UBOOTPBA = pNandInfo->u8_UBOOTPBA;
    pNandDrv->u8_HashPBA[0][0] = pNandInfo->u8_HashPBA[0][0];
    pNandDrv->u8_HashPBA[0][1] = pNandInfo->u8_HashPBA[0][1];
    pNandDrv->u8_HashPBA[1][0] = pNandInfo->u8_HashPBA[1][0];
    pNandDrv->u8_HashPBA[1][1] = pNandInfo->u8_HashPBA[1][1];
    pNandDrv->u8_HashPBA[2][0] = pNandInfo->u8_HashPBA[2][0];
    pNandDrv->u8_HashPBA[2][1] = pNandInfo->u8_HashPBA[2][1];
    pNandDrv->u8_Hash0PageIdx = pNandInfo->u8_Hash0PageIdx;
    pNandDrv->u8_Hash1PageIdx = pNandInfo->u8_Hash1PageIdx;
    pNandDrv->u32_BootSize = pNandInfo->u32_BootSize;

    pNandDrv->u16_BBtPageCheckCount = pNandInfo->u16_BBtPageCheckCount;
    memcpy(pNandDrv->u16_BBtPageIdx, pNandInfo->u16_BBtPageIdx, NAND_BBT_PAGE_COUNT*sizeof(U16));
    memcpy(pNandDrv->u16_BBtMarker, pNandInfo->u16_BBtMarker, NAND_BBT_PAGE_COUNT*sizeof(U16));

    nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1,  "BBtPageCheckCount = %d\n", pNandDrv->u16_BBtPageCheckCount);
    for(i=0; i<NAND_BBT_PAGE_COUNT; i++)
        nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "BBtPageIdx[%d]=%d, BBtMarker[%d]=%d\n", i, pNandDrv->u16_BBtPageIdx[i], i, pNandDrv->u16_BBtMarker[i]);

	pNandDrv->u8_IDByteCnt = pNandInfo->u8_IDByteCnt;
	memcpy(pNandDrv->au8_ID, pNandInfo->au8_ID, pNandInfo->u8_IDByteCnt);
}

void drvNAND_DumpReadRetry(void)
{
    U16 u16_i, u16_j, u16_k;
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();

    nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1,"u8_CustRegNo = %X\n", pNandDrv->ReadRetry_t.u8_CustRegNo);

    nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1,"Cust Register: %X\n", pNandDrv->ReadRetry_t.u8_CustRegNo);
    for(u16_i = 0; u16_i < pNandDrv->ReadRetry_t.u8_CustRegNo; u16_i++)
        nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"%X ", pNandDrv->ReadRetry_t.pu8_CustRegTable[u16_i]);
    nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"\n");

    nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1,"u8_DefaultValueOffset = %X\n", pNandDrv->ReadRetry_t.u8_DefaultValueOffset);

    nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1,"u8_MaxRetryTime = %X\n", pNandDrv->ReadRetry_t.u8_MaxRetryTime);

    nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1,"u8_ByteLenPerCmd = %X\n", pNandDrv->ReadRetry_t.u8_ByteLenPerCmd);

    nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1,"Retry Register Value Table\n ");

    for(u16_i = 0; u16_i < pNandDrv->ReadRetry_t.u8_CustRegNo; u16_i ++)
        nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"Reg%X\t", u16_i);
    nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"\n");

    for(u16_i = 0; u16_i < pNandDrv->ReadRetry_t.u8_MaxRetryTime; u16_i ++)
    {
        for(u16_j = 0; u16_j < pNandDrv->ReadRetry_t.u8_CustRegNo; u16_j ++)
        {
            for(u16_k = 0; u16_k < pNandDrv->ReadRetry_t.u8_ByteLenPerCmd+1; u16_k ++)
                nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"%02X", pNandDrv->ReadRetry_t.pppu8_RetryRegValue[u16_i][u16_j][u16_k]);
            nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"\t");
        }
        nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"\n");
    }
    nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1,"u8_DefaultValueRestore = %X\n", pNandDrv->ReadRetry_t.u8_DefaultValueRestore);

    nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1,"u8_SetCmdLen = %X\n", pNandDrv->ReadRetry_t.u8_SetCmdLen);

    if(pNandDrv->ReadRetry_t.u8_SetCmdLen)
    {
        for(u16_i = 0; u16_i < pNandDrv->ReadRetry_t.u8_SetCmdLen; u16_i ++)
            nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"%X\t", pNandDrv->ReadRetry_t.pu8_SetCmdTypeSeq[u16_i]);
        nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"\n");

        for(u16_i = 0; u16_i < pNandDrv->ReadRetry_t.u8_SetCmdLen; u16_i ++)
            nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"%X\t", pNandDrv->ReadRetry_t.pu8_SetCmdValueSeq[u16_i]);
        nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"\n");
    }
    nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1,"u8_GetCmdLen = %X\n", pNandDrv->ReadRetry_t.u8_GetCmdLen);

    if(pNandDrv->ReadRetry_t.u8_GetCmdLen)
    {
        for(u16_i = 0; u16_i < pNandDrv->ReadRetry_t.u8_GetCmdLen; u16_i ++)
            nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"%X\t", pNandDrv->ReadRetry_t.pu8_GetCmdTypeSeq[u16_i]);
        nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"\n");

        for(u16_i = 0; u16_i < pNandDrv->ReadRetry_t.u8_GetCmdLen; u16_i ++)
            nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"%X\t", pNandDrv->ReadRetry_t.pu8_GetCmdValueSeq[u16_i]);
        nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0,"\n");
    }
}

U32 drvNAND_ParseReadRetryCmd(U8 *pu8_DataBuf)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
    U16 u16_Idx = 0, u16_i, u16_j;

    pNandDrv->u8_RequireReadRetry = 1;
    pNandDrv->ReadRetry_t.u8_CustRegNo = pu8_DataBuf[u16_Idx];
    u16_Idx ++;
    //Number of Custom Register

    pNandDrv->ReadRetry_t.pu8_CustRegTable = (U8*)malloc(pNandDrv->ReadRetry_t.u8_CustRegNo * sizeof(U8));
    if(!pNandDrv->ReadRetry_t.pu8_CustRegTable)
        return UNFD_ST_ERR_INVALID_ADDR;
    //Reg Table
    memcpy(pNandDrv->ReadRetry_t.pu8_CustRegTable, pu8_DataBuf+u16_Idx, pNandDrv->ReadRetry_t.u8_CustRegNo);
    u16_Idx += pNandDrv->ReadRetry_t.u8_CustRegNo;

    pNandDrv->ReadRetry_t.u8_DefaultValueOffset = pu8_DataBuf[u16_Idx];
    u16_Idx ++;
    pNandDrv->ReadRetry_t.u8_MaxRetryTime = pu8_DataBuf[u16_Idx];
    u16_Idx ++;
    pNandDrv->ReadRetry_t.u8_ByteLenPerCmd = pu8_DataBuf[u16_Idx];
    u16_Idx ++;

    pNandDrv->ppu8_ReadRetryDefault = (U8**)malloc(pNandDrv->ReadRetry_t.u8_CustRegNo * sizeof(U8*) +
                                        pNandDrv->ReadRetry_t.u8_CustRegNo * ((U32)(pNandDrv->ReadRetry_t.u8_ByteLenPerCmd + 1)) *sizeof(U8));
    if(!pNandDrv->ppu8_ReadRetryDefault)
        return UNFD_ST_ERR_INVALID_ADDR;

    for(u16_i = 0; u16_i < pNandDrv->ReadRetry_t.u8_CustRegNo; u16_i ++)
        pNandDrv->ppu8_ReadRetryDefault[u16_i] = ((U8*)(pNandDrv->ppu8_ReadRetryDefault + pNandDrv->ReadRetry_t.u8_CustRegNo)
                                                    + u16_i * ((U32)(pNandDrv->ReadRetry_t.u8_ByteLenPerCmd + 1)) *sizeof(U8));

    //alloc Retry Register value table and parse Retry Register value table
    pNandDrv->ReadRetry_t.pppu8_RetryRegValue =
        (U8***) malloc( pNandDrv->ReadRetry_t.u8_MaxRetryTime *sizeof(U8**)
            + pNandDrv->ReadRetry_t.u8_MaxRetryTime * pNandDrv->ReadRetry_t.u8_CustRegNo *sizeof(U8*)
            + pNandDrv->ReadRetry_t.u8_MaxRetryTime * pNandDrv->ReadRetry_t.u8_CustRegNo * (pNandDrv->ReadRetry_t.u8_ByteLenPerCmd + 1) *sizeof(U8) );
    if(!pNandDrv->ReadRetry_t.pppu8_RetryRegValue)
        return UNFD_ST_ERR_INVALID_ADDR;

    for(u16_i = 0 ; u16_i < pNandDrv->ReadRetry_t.u8_MaxRetryTime; u16_i ++)
        pNandDrv->ReadRetry_t.pppu8_RetryRegValue[u16_i] = ((U8**) pNandDrv->ReadRetry_t.pppu8_RetryRegValue +
                                            pNandDrv->ReadRetry_t.u8_MaxRetryTime) + u16_i * pNandDrv->ReadRetry_t.u8_CustRegNo;

    for(u16_i = 0; u16_i < pNandDrv->ReadRetry_t.u8_MaxRetryTime; u16_i ++)
        for(u16_j = 0; u16_j < pNandDrv->ReadRetry_t.u8_CustRegNo; u16_j ++)
            pNandDrv->ReadRetry_t.pppu8_RetryRegValue[u16_i][u16_j] =
                ((U8*) (pNandDrv->ReadRetry_t.pppu8_RetryRegValue + pNandDrv->ReadRetry_t.u8_MaxRetryTime +
                    pNandDrv->ReadRetry_t.u8_MaxRetryTime * pNandDrv->ReadRetry_t.u8_CustRegNo)) +
                        (u16_i*pNandDrv->ReadRetry_t.u8_CustRegNo + u16_j)* (pNandDrv->ReadRetry_t.u8_ByteLenPerCmd + 1) *sizeof(U8);

    memcpy(&pNandDrv->ReadRetry_t.pppu8_RetryRegValue[0][0][0], pu8_DataBuf + u16_Idx,
            pNandDrv->ReadRetry_t.u8_MaxRetryTime * pNandDrv->ReadRetry_t.u8_CustRegNo * (pNandDrv->ReadRetry_t.u8_ByteLenPerCmd + 1) *sizeof(U8));

    u16_Idx+= pNandDrv->ReadRetry_t.u8_MaxRetryTime * pNandDrv->ReadRetry_t.u8_CustRegNo * (pNandDrv->ReadRetry_t.u8_ByteLenPerCmd + 1) *sizeof(U8);
    pNandDrv->ReadRetry_t.u8_DefaultValueRestore = pu8_DataBuf[u16_Idx];
    u16_Idx ++;
    pNandDrv->ReadRetry_t.u8_SetCmdLen = pu8_DataBuf[u16_Idx];
    u16_Idx ++;
    if(pNandDrv->ReadRetry_t.u8_SetCmdLen)
    {
        //parse Set CMD Type Sequence
        pNandDrv->ReadRetry_t.pu8_SetCmdTypeSeq = (U8*)malloc(pNandDrv->ReadRetry_t.u8_SetCmdLen * sizeof(U8));
        if(!pNandDrv->ReadRetry_t.pu8_SetCmdTypeSeq)
            return UNFD_ST_ERR_INVALID_ADDR;
        memcpy(pNandDrv->ReadRetry_t.pu8_SetCmdTypeSeq, pu8_DataBuf + u16_Idx, pNandDrv->ReadRetry_t.u8_SetCmdLen);
        u16_Idx += pNandDrv->ReadRetry_t.u8_SetCmdLen;
        //parse Set CMD Value Sequence
        pNandDrv->ReadRetry_t.pu8_SetCmdValueSeq = (U8*)malloc(pNandDrv->ReadRetry_t.u8_SetCmdLen * sizeof(U8));
        if(!pNandDrv->ReadRetry_t.pu8_SetCmdValueSeq)
            return UNFD_ST_ERR_INVALID_ADDR;
        memcpy(pNandDrv->ReadRetry_t.pu8_SetCmdValueSeq, pu8_DataBuf + u16_Idx, pNandDrv->ReadRetry_t.u8_SetCmdLen);
        u16_Idx += pNandDrv->ReadRetry_t.u8_SetCmdLen;
    }
    pNandDrv->ReadRetry_t.u8_GetCmdLen = pu8_DataBuf[u16_Idx];
    u16_Idx ++;
    if(pNandDrv->ReadRetry_t.u8_GetCmdLen)
    {
        //parse Get CMD Type Sequence
        pNandDrv->ReadRetry_t.pu8_GetCmdTypeSeq = (U8*)malloc(pNandDrv->ReadRetry_t.u8_GetCmdLen * sizeof(U8));
        if(!pNandDrv->ReadRetry_t.pu8_GetCmdTypeSeq)
            return UNFD_ST_ERR_INVALID_ADDR;
        memcpy(pNandDrv->ReadRetry_t.pu8_GetCmdTypeSeq, pu8_DataBuf + u16_Idx, pNandDrv->ReadRetry_t.u8_GetCmdLen);
        u16_Idx += pNandDrv->ReadRetry_t.u8_GetCmdLen;
        //parse Get CMD Value Sequence
        pNandDrv->ReadRetry_t.pu8_GetCmdValueSeq = (U8*)malloc(pNandDrv->ReadRetry_t.u8_GetCmdLen * sizeof(U8));
        if(!pNandDrv->ReadRetry_t.pu8_GetCmdValueSeq)
            return UNFD_ST_ERR_INVALID_ADDR;
        memcpy(pNandDrv->ReadRetry_t.pu8_GetCmdValueSeq, pu8_DataBuf + u16_Idx, pNandDrv->ReadRetry_t.u8_GetCmdLen);
        u16_Idx += pNandDrv->ReadRetry_t.u8_GetCmdLen;
    }
    return UNFD_ST_SUCCESS;
}
U32 drvNAND_SearchCIS(void)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
    U8 *au8_PageBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    U8 *au8_SpareBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf;
    NAND_FLASH_INFO_t *pNandInfo = (NAND_FLASH_INFO_t*)au8_PageBuf;
    BLK_INFO_t *pBlkInfo = (BLK_INFO_t*)au8_SpareBuf;
    U16 u16_PBA;
    U32 u32_PageIdx;
    U32 u32_Err;
    U32 u32_ChkSum;
    U32 u16_i;
	#if defined(FCIE_LFSR) && FCIE_LFSR
	U8 u8_RetryRandomizer;
	#endif

    U8 u8_BlkPageCntBits = 5; /* 32 pages per block */

    pNandDrv->u8_CISBlk = 0xFF;
    for (u16_PBA = 0; u16_PBA < 0x40; u16_PBA++)
    {
		#if defined(FCIE_LFSR) && FCIE_LFSR
		u8_RetryRandomizer = 0;
		LABEL_TRY_RANDOMIZER:
		//force disable Randomizer
		if(!u8_RetryRandomizer)
			NC_DisableLFSR();
		else
			NC_EnableLFSR();
		#endif

        pNandDrv->u16_BlkCnt = 0x400;
        pNandDrv->u16_BlkPageCnt = 0x20;
        pNandDrv->u16_PageByteCnt = 0x800;
        pNandDrv->u16_SpareByteCnt = 0x100;
        pNandDrv->u16_ECCType = NANDINFO_ECC_TYPE;
        #if defined(ECC_TYPE_72BIT1KB) && ECC_TYPE_72BIT1KB
        if(pNandDrv->u16_ECCType == ECC_TYPE_72BIT1KB)
            pNandDrv->u16_SpareByteCnt = 0x180;
        #endif
        pNandDrv->u16_BitflipThreshold = 0;

        NC_ConfigContext();
        NC_ReInit();
        pNandDrv->u16_Reg48_Spare |= (1 << 12);

        NC_Config();
        u32_PageIdx = u16_PBA << u8_BlkPageCntBits;
        nand_debug(UNFD_DEBUG_LEVEL_HIGH, 0, "32Pages: %d\n", u16_PBA);

        // ------------------------------------
        if(pNandDrv->u8_Flag_004A30)
        {
            pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C4TFS0;
            pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C2TRS0;
        }
        LABEL_TRY_005A30:
        // ------------------------------------

        #if defined(ENABLE_NAND_RIU_MODE)&&ENABLE_NAND_RIU_MODE
        u32_Err = NC_ReadSector_RIUMode(u32_PageIdx, 0, au8_PageBuf, au8_SpareBuf);
        #else
        u32_Err = NC_ReadSectors(u32_PageIdx, 0, au8_PageBuf, au8_SpareBuf, 1);
        #endif
        #if 0
        if (u32_Err != UNFD_ST_SUCCESS && 0==(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM))
        {
            //nand_debug(0, 0, "NAND, change WordMode, now: %X\n", pNandDrv->u8_WordMode);

            // If read sector fail, change u8_WordMode and try again
            if (pNandDrv->u8_WordMode == 0)
            {
                pNandDrv->u8_WordMode = 1;
                pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_WORD_MODE;
            }
            else
            {
                pNandDrv->u8_WordMode = 0;
                pNandDrv->u16_Reg50_EccCtrl &= (~BIT_NC_WORD_MODE);
            }

            #if defined(ENABLE_NAND_RIU_MODE)&&ENABLE_NAND_RIU_MODE
            u32_Err = NC_ReadSector_RIUMode(u32_PageIdx, 0, au8_PageBuf, au8_SpareBuf);
            #else
            u32_Err = NC_ReadSectors(u32_PageIdx, 0, au8_PageBuf, au8_SpareBuf, 1);
            #endif
            //nand_debug(UNFD_DEBUG_LEVEL_HIGH, 0, "NC_ReadSectors2=%x\n", u32_Err);

        }
        #endif

        if (u32_Err != UNFD_ST_SUCCESS || pBlkInfo->u8_BadBlkMark != 0xFF)
        {
            if(1==pNandDrv->u8_Flag_004A30 && ADR_C5TFS0!=pNandDrv->u8_OpCode_RW_AdrCycle)
            {
                nand_debug(0,1,"switch to 005A30, 0 \n");
                pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C5TFS0;
                pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C3TRS0;
                goto LABEL_TRY_005A30;
            }

			#if defined(FCIE_LFSR) && FCIE_LFSR
			if(!u8_RetryRandomizer)
			{
				u8_RetryRandomizer = 1;
				goto LABEL_TRY_RANDOMIZER;
			}
			#endif

            continue;
        }

        /* Valid CIS has block address 0 or 1 */
        //if (pBlkInfo->u16_BlkAddr > 1)
        //  continue;

        if (drvNAND_CompareCISTag(pNandInfo->au8_Tag))
        {
            if(1==pNandDrv->u8_Flag_004A30 && ADR_C5TFS0!=pNandDrv->u8_OpCode_RW_AdrCycle)
            {
                //nand_debug(0,1,"switch to 005A30, 1 \n");
                pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C5TFS0;
                pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C3TRS0;
                goto LABEL_TRY_005A30;
            }

			#if defined(FCIE_LFSR) && FCIE_LFSR
			if(!u8_RetryRandomizer)
			{
				u8_RetryRandomizer = 1;
				goto LABEL_TRY_RANDOMIZER;
			}
			#endif

            continue;
        }

        u32_ChkSum = drvNAND_CheckSum(au8_PageBuf + 0x24, 0x32 - 0x24);
        if (u32_ChkSum != pNandInfo->u32_ChkSum)
        {
            nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1, "Skip Blk 0x%08x, ChkSum: 0x%08x != 0x%08x\n",
                       u16_PBA, u32_ChkSum, pNandInfo->u32_ChkSum);

            if(1==pNandDrv->u8_Flag_004A30 && ADR_C5TFS0!=pNandDrv->u8_OpCode_RW_AdrCycle)
            {
                //nand_debug(0,1,"switch to 005A30, 2 \n");
                pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C5TFS0;
                pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C3TRS0;
                goto LABEL_TRY_005A30;
            }

			#if defined(FCIE_LFSR) && FCIE_LFSR
			if(!u8_RetryRandomizer)
			{
				u8_RetryRandomizer = 1;
				goto LABEL_TRY_RANDOMIZER;
			}
			#endif

            continue;
        }

        //u8_CISIdx = pBlkInfo->u16_BlkAddr;

        nand_debug(UNFD_DEBUG_LEVEL_HIGH, 0, "Complete auto-checking 8/16 bits:0x%X\n", pNandDrv->u8_WordMode);
        dump_nand_info(pNandInfo);

        drvNAND_ParseNandInfo(pNandInfo);
        //Read Retry
        if(pNandDrv->u8_RequireReadRetry)
        {
            NC_SetupReadRetryCmd();
            NC_GetRegDefaultValue();
        }

        NC_ConfigContext();
        NC_ReInit();
        #if defined(FCIE_LFSR) && FCIE_LFSR
        if(pNandDrv->u8_RequireRandomizer)
        {
            nand_debug(0,1,"Enable Randomizer\n");
        	NC_EnableLFSR();                            //Compatible with ROM Code
        }
        #endif
        NC_Config();
        if(pNandDrv->u8_CellType == 0)
        {
            for(u16_i=0;u16_i<pNandDrv->u16_BlkPageCnt;u16_i++)
            {
                ga_tPairedPageMap[u16_i].u16_LSB = u16_i;
                ga_tPairedPageMap[u16_i].u16_MSB = u16_i;
            }
        }
        else
        {
            nand_debug(UNFD_DEBUG_LEVEL, 1, "Paired Page Map @0x%08x\n", pNandDrv->u8_PairPageMapLoc);
            u32_Err = NC_ReadPages(u32_PageIdx+pNandDrv->u8_PairPageMapLoc, au8_PageBuf, au8_SpareBuf, 1);
            if (u32_Err != UNFD_ST_SUCCESS)
            {
                nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1, "Skip Blk 0x%08x, Read PPM fail:0x%X\n",
                           u16_PBA, u32_Err);
                continue;
            }
            else
            {
                memcpy(&ga_tPairedPageMap[0], au8_PageBuf, 2048);
            }
        }

        if((u32_Err == UNFD_ST_SUCCESS))
        {
            nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "PartInfo @0x%08x\n", ga_tPairedPageMap[1].u16_LSB);
            pNandDrv->u8_HasPNI = 1;
            u32_Err = drvNAND_CheckPartInfo(u32_PageIdx + ga_tPairedPageMap[1].u16_LSB);
            if (u32_Err != UNFD_ST_SUCCESS) {
                pNandDrv->u8_HasPNI = 0;
                nand_debug(UNFD_DEBUG_LEVEL, 1, "No PartInfo\n");
            }
        }
        // BlkLowPCnt
        for(u16_i=0; u16_i<pNandDrv->u16_BlkPageCnt; u16_i++)
        if(ga_tPairedPageMap[u16_i].u16_LSB==pNandDrv->u16_BlkPageCnt-1 ||
            ga_tPairedPageMap[u16_i].u16_MSB==pNandDrv->u16_BlkPageCnt-1)
            break;
        pNandDrv->u16_BlkLowPCnt = u16_i+1;

        return UNFD_ST_SUCCESS;
    }

    #if defined(FCIE_LFSR) && FCIE_LFSR
    //force disable Randomizer
    NC_DisableLFSR();
    #endif

    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "No CIS found\n");
    return UNFD_ST_ERR_NO_CIS;
}

U32 drvNAND_InitBBT(INIT_BBT_t *pInitBBT)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
#if NAND_BUF_USE_STACK
    U8 au8_PageBuf[4096];
    U8 au8_SpareBuf[128];
#else
    U8 *au8_PageBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    U8 *au8_SpareBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf;
#endif
    U32 u32_PageIdx;
    U32 u32_Err;
    U32 u32_i, u32_k;
    U8 u8_IsBadBlk;
    U8 u8_BadBlkMarker = (pNandDrv->u32_Config>>1)&0x07;
    U8 u8_BusWidth = (pNandDrv->u32_Config>>7)&0x01;

    nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "Bad Blk Col Addr=0x%X\n", u8_BadBlkMarker);
    nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "Bus Width=0x%X\n", u8_BusWidth);
    //pInitBBT->u16_Cnt = 0;
    for(u32_i=0; u32_i<pNandDrv->u16_BlkCnt; u32_i++)
    {
        u8_IsBadBlk = 0;
        #if 0
        if(drvNAND_ErasePhyBlk(u32_i))
        {
            u8_IsBadBlk = 1;
            nand_debug(0,1,"bad blk: %X from erase \n", u32_i);
        }
        #endif
        for(u32_k=0; u32_k<3; u32_k++)
        {
            u32_PageIdx = (u32_i << pNandDrv->u8_BlkPageCntBits) + u32_k;
            u32_Err = NC_ReadSectors_MTD(u32_PageIdx, 0, au8_PageBuf, au8_SpareBuf, 1);
            if (u32_Err != UNFD_ST_SUCCESS )
                nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NC_ReadSectors(0x%X)=0x%X\n", u32_PageIdx, u32_Err);
            if(u8_BusWidth == 0) // 8-bit mode
            {
                if(au8_SpareBuf[u8_BadBlkMarker] != 0xFF)
                {
                    u8_IsBadBlk = 1;
                    break;
                }
            }
            else // 16-bit mode
            {
                if(au8_SpareBuf[(u8_BadBlkMarker<<1)] != 0xFF || au8_SpareBuf[(u8_BadBlkMarker<<1)+1] != 0xFF)
                {
                    u8_IsBadBlk = 1;
                    break;
                }
            }

            u32_PageIdx = ((u32_i + 1) << pNandDrv->u8_BlkPageCntBits) - 1 - u32_k;
            u32_Err = NC_ReadSectors_MTD(u32_PageIdx, 0, au8_PageBuf, au8_SpareBuf, 1);
            if (u32_Err != UNFD_ST_SUCCESS )
                nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NC_ReadSectors(0x%X)=0x%X\n", u32_PageIdx, u32_Err);
            if(u8_BusWidth == 0) // 8-bit mode
            {
                if(au8_SpareBuf[u8_BadBlkMarker] != 0xFF)
                {
                    u8_IsBadBlk = 1;
                    break;
                }
            }
            else // 16-bit mode
            {
                if(au8_SpareBuf[(u8_BadBlkMarker<<1)] != 0xFF || au8_SpareBuf[(u8_BadBlkMarker<<1)+1] != 0xFF)
                {
                    u8_IsBadBlk = 1;
                    break;
                }
            }
        }

        if (u8_IsBadBlk == 1)
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Bad Blk = 0x%X\n", u32_i);

            if (pInitBBT->u16_Cnt <= 250)
            {
                pInitBBT->au8_BadBlkTbl[(pInitBBT->u16_Cnt<<1)] = (u32_i & 0xFF);
                pInitBBT->au8_BadBlkTbl[(pInitBBT->u16_Cnt<<1)+1] = ((u32_i >> 8) & 0xFF);
            }
            pInitBBT->u16_Cnt++;
            drvNAND_MarkBadBlk(u32_i);
        }
    }

    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Total Bad Blk = 0x%X\n", pInitBBT->u16_Cnt);

    pInitBBT->u32_ChkSum = drvNAND_CheckSum((U8*)(&(pInitBBT->u16_Cnt)), (512-4));
    return UNFD_ST_SUCCESS;
}


//--------------------------------------------------------------------------
static UNFD_ALIGN0 TEST_ALIGN_PACK_t g_TestAlignPack_t UNFD_ALIGN1;

U32 drvNAND_CheckAlignPack(U8 u8_AlignByteCnt)
{
    // check alignment
    if((uintptr_t)&(g_TestAlignPack_t.u8_0) & (u8_AlignByteCnt-1))
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR, not aliged. expect %X but %lX \n",
            u8_AlignByteCnt, (uintptr_t)&(g_TestAlignPack_t.u8_0));
        nand_die();
        return UNFD_ST_ERR_NOT_ALIGN;
    }
    // check packed - 0
    if((uintptr_t)&(g_TestAlignPack_t.u16_0)-(uintptr_t)&(g_TestAlignPack_t.u8_0) != 1 ||
       (uintptr_t)&(g_TestAlignPack_t.u32_0)-(uintptr_t)&(g_TestAlignPack_t.u8_0) != 3 ||
       (uintptr_t)&(g_TestAlignPack_t.u32_1)-(uintptr_t)&(g_TestAlignPack_t.u8_0) != 7)
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR, not packed. check err.0 \n");
        nand_die();
        return UNFD_ST_ERR_NOT_PACKED;
    }
    // check packed - 1
    if((uintptr_t)&(g_TestAlignPack_t.u16_0)-(uintptr_t)&(g_TestAlignPack_t.u8_0) != 1 ||
       (uintptr_t)&(g_TestAlignPack_t.u32_0)-(uintptr_t)&(g_TestAlignPack_t.u16_0)!= 2 ||
       (uintptr_t)&(g_TestAlignPack_t.u32_1)-(uintptr_t)&(g_TestAlignPack_t.u32_0)!= 4)
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR, not packed. check err.1 \n");
        nand_die();
        return UNFD_ST_ERR_NOT_PACKED;
    }

    nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "ok\n");
    return UNFD_ST_SUCCESS;
}
//--------------------------------------------------------------------------

static __inline void dump_mem_line(unsigned char *buf, int cnt)
{
    int i;

    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, " 0x%08lx: ", (uintptr_t)buf);
    for (i= 0; i < cnt; i++)
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "%02X ", buf[i]);

    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, " | ");

    for (i = 0; i < cnt; i++)
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "%c", (buf[i] >= 32 && buf[i] < 128) ? buf[i] : '.');

    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "\n");
}

void dump_mem(unsigned char *buf, int cnt)
{
    int i;

    for (i= 0; i < cnt; i+= 16)
        dump_mem_line(buf + i, 16);
}

void dump_nand_info(NAND_FLASH_INFO_t *pNandInfo)
{
    int i;
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "###############################################\n");
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "#        NAND INFO                            #\n");
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "###############################################\n");

    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "pNandInfo: 0x%08lx\n", (uintptr_t)pNandInfo);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "au8_Tag          : [");
    for (i = 0; i < 16; i++)
        nand_debug(UNFD_DEBUG_LEVEL_LOW, 0, "%c", pNandInfo->au8_Tag[i]);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 0, "]\n");

    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u8_IDByteCnt     : 0x%04x\n", pNandInfo->u8_IDByteCnt);

    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "au8_ID           : 0x[ ");
    for (i = 0; i < pNandInfo->u8_IDByteCnt; i++)
        nand_debug(UNFD_DEBUG_LEVEL_LOW, 0, "%02X ", pNandInfo->au8_ID[i]);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 0, "]\n");

    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u32_ChkSum       : 0x%04x\n", pNandInfo->u32_ChkSum);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u16_SpareByteCnt : 0x%04x\n", pNandInfo->u16_SpareByteCnt);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u16_PageByteCnt  : 0x%04x\n", pNandInfo->u16_PageByteCnt);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u16_BlkPageCnt   : 0x%04x\n", pNandInfo->u16_BlkPageCnt);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u16_BlkCnt       : 0x%04x\n", pNandInfo->u16_BlkCnt);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u32_Config       : 0x%08x\n", pNandInfo->u32_Config);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u16_ECCType      : 0x%04x\n", pNandInfo->u16_ECCType);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u16_tRC: 0x%04x\n", pNandInfo->u16_tRC);
}

void dump_part_records(PARTITION_RECORD_t *records, int cnt)
{
    int i;

    if (cnt > 1024) {
        nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "broken\n");
        return;
    }

    for (i = 0; i < cnt; i++) {
        nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "record[%d]       : 0x%04x, 0x%04x, 0x%04x, 0x%04x\n",
                i,
                records[i].u16_StartBlk, records[i].u16_BlkCnt,
                records[i].u16_PartType, records[i].u16_BackupBlkCnt);
    }
}

void dump_general_blk_info(BLK_INFO_t *pBlkInfo)
{
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u8_BadBlkIdx    : 0x%04x\n", pBlkInfo->u8_BadBlkMark);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u8_PartType     : 0x%04x\n", pBlkInfo->u8_PartType);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u16_BlkAddr     : 0x%04x\n", pBlkInfo->u16_BlkAddr);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "au8_Misc        : 0x%04x\n", *(U16*)pBlkInfo->au8_Misc);
}

void dump_part_info(PARTITION_INFO_t *pPartInfo)
{
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "###############################################\n");
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "#        PART INFO                            #\n");
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "###############################################\n");

    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "pPartInfo: 0x%08lx\n", (uintptr_t)pPartInfo);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u32_ChkSum      : 0x%04x\n", pPartInfo->u32_ChkSum);
    //nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u32_BLSectorCnt : 0x%04x\n", pPartInfo->u32_BLSectorCnt);
    //nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u32_OSSectorCnt : 0x%04x\n", pPartInfo->u32_OSSectorCnt);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u16_PartCnt     : 0x%04x\n", pPartInfo->u16_PartCnt);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u16_UnitByteCnt : 0x%04x\n", pPartInfo->u16_UnitByteCnt);

    dump_part_records(pPartInfo->records, pPartInfo->u16_PartCnt);
}

void dump_miu_records(MIU_RECORD_t *pRecords, int cnt)
{
    int i;

    if (cnt > (1024 - 8)) {
        nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "broken\n");
        return;
    }
    for (i = 0; i < (cnt >> 3); i++)
        nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "miu record[%02d]  : 0x%08x, 0x%08x\n",
                i, pRecords[i].u32_RegAddr, pRecords[i].u32_RegValue);
}

void dump_miu_part(MIU_PART_t *pMiuPart)
{
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "###############################################\n");
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "#        MIU PART                             #\n");
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "###############################################\n");

    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "pMiuPart: 0x%08lx\n", (uintptr_t)pMiuPart);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u32_ChkSum      : 0x%04x\n", pMiuPart->u32_ChkSum);
    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "u32_ByteCnt     : 0x%04x\n", pMiuPart->u32_ByteCnt);

    dump_miu_records(pMiuPart->records, pMiuPart->u32_ByteCnt);
}

void dump_nand_driver(NAND_DRIVER *pNandDrv)
{
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "NAND_Info:\n");
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  BlkCnt                 : 0x%X\n", pNandDrv->u16_BlkCnt);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  BlkPageCnt             : 0x%X\n", pNandDrv->u16_BlkPageCnt);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  BlkSectorCnt           : 0x%X\n", pNandDrv->u16_BlkSectorCnt);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  PageByteCnt            : 0x%X\n", pNandDrv->u16_PageByteCnt);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  SpareByteCnt           : 0x%X\n\n", pNandDrv->u16_SpareByteCnt);

    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  BlkPageCntBits         : 0x%X\n", pNandDrv->u8_BlkPageCntBits);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  BlkSectorCntBits       : 0x%X\n", pNandDrv->u8_BlkSectorCntBits);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  PageByteCntBits        : 0x%X\n", pNandDrv->u8_PageByteCntBits);

    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  BlkPageCntMask         : 0x%X\n", pNandDrv->u16_BlkPageCntMask);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  BlkSectorCntMask       : 0x%X\n", pNandDrv->u16_BlkSectorCntMask);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  PageByteCntMask        : 0x%X\n", pNandDrv->u16_PageByteCntMask);

    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  PageSectorCnt          : 0x%X\n", pNandDrv->u16_PageSectorCnt);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  SectorByteCnt          : 0x%X\n", pNandDrv->u16_SectorByteCnt);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  SectorSpareByteCnt     : 0x%X\n", pNandDrv->u16_SectorSpareByteCnt);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  ECCCodeByteCnt         : 0x%X\n\n", pNandDrv->u16_ECCCodeByteCnt);

    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  PageSectorCntBits      : 0x%X\n", pNandDrv->u8_PageSectorCntBits);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  SectorByteCntBits      : 0x%X\n", pNandDrv->u8_SectorByteCntBits);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  PageSectorCntMask      : 0x%X\n", pNandDrv->u16_PageSectorCntMask);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  SectorByteCntMask      : 0x%X\n\n", pNandDrv->u16_SectorByteCntMask);

    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u8_OpCode_Erase_AdrCycle      : 0x%X\n", pNandDrv->u8_OpCode_Erase_AdrCycle);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u8_OpCode_RW_AdrCycle      : 0x%X\n\n", pNandDrv->u8_OpCode_RW_AdrCycle);

    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u16_tRC;      : 0x%X\n", pNandDrv->u16_tRC);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u8_tRP      : 0x%X\n", pNandDrv->u8_tRP);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u8_tREH      : 0x%X\n", pNandDrv->u8_tREH);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u8_tREA      : 0x%X\n", pNandDrv->u8_tREA);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u8_tRR      : 0x%X\n", pNandDrv->u8_tRR);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u16_tADL      : 0x%X\n", pNandDrv->u16_tADL);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u16_tRHW      : 0x%X\n", pNandDrv->u16_tRHW);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u16_tWHR      : 0x%X\n", pNandDrv->u16_tWHR);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u16_tCCS      : 0x%X\n", pNandDrv->u16_tCCS);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u8_tCS      : 0x%X\n", pNandDrv->u8_tCS);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u16_tWC      : 0x%X\n", pNandDrv->u16_tWC);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u8_tWP      : 0x%X\n", pNandDrv->u8_tWP);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u8_tWH      : 0x%X\n", pNandDrv->u8_tWH);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u16_tCWAW      : 0x%X\n", pNandDrv->u16_tCWAW);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u8_tCLHZ      : 0x%X\n", pNandDrv->u8_tCLHZ);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u16_tWW      : 0x%X\n", pNandDrv->u16_tWW);
    nand_debug(UNFD_DEBUG_LEVEL_MEDIUM, 1, "  u8_AddrCycleIdx      : 0x%X\n\n", pNandDrv->u8_AddrCycleIdx);


}

U8 drvNAND_CountBits(U32 u32_x)
{
    U8 u8_i = 0;

    while (u32_x) {
        u8_i++;
        u32_x >>= 1;
    }

    return u8_i-1;
}

U32 drvNAND_CheckSum(U8 *pu8_Data, U16 u16_ByteCnt)
{
    U32 u32_Sum = 0;

    while (u16_ByteCnt--)
        u32_Sum += *pu8_Data++;

    return u32_Sum;
}

/* return 0: same, 1: different */
U32 drvNAND_CompareCISTag(U8 *tag)
{
    const char *str = "MSTARSEMIUNFDCIS";
    int i = 0;

    for (i = 0; i < 16; i++) {
        if (tag[i] != str[i])
            return 1;
    }

    return 0;
}

/* Search for partition of type @u16_PartType, begin from @pRecord */
PARTITION_RECORD_t *drvNAND_SearchPartition(PARTITION_RECORD_t *pRecord,
                                            U16 u16_PartType)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
    PARTITION_INFO_t *pPartInfo = pNandDrv->pPartInfo;

    while (pRecord - pPartInfo->records < pPartInfo->u16_PartCnt) {

        if (pRecord->u16_PartType == u16_PartType)
            return pRecord;

        pRecord++;
    }

    return (void*)0;
}

/* return 1: Good block, 0: Bad block */
U32 drvNAND_IsGoodBlk(U16 u16_PBA)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
#if NAND_BUF_USE_STACK
    if(pNandDrv->u16_BBtPageCheckCount > 0 && pNandDrv->u16_BBtPageCheckCount <= NAND_PAGE_COUNT){
        U8 au8_DataBuf[8192];
        U8 au8_SpareBuf[640];
    }else{
        U8 au8_DataBuf[1024];
        U8 au8_SpareBuf[64];
    }
#else
    U8 *au8_DataBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    U8 *au8_SpareBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf;
#endif
    U32 u32_PageIdx;
    U32 u32_Err;
    U32 i;
    BLK_INFO_t *pBlkInfo = (BLK_INFO_t*)au8_SpareBuf;

    if(pNandDrv->u16_BBtPageCheckCount > 0 && pNandDrv->u16_BBtPageCheckCount <= NAND_BBT_PAGE_COUNT){
        for(i=0; i<pNandDrv->u16_BBtPageCheckCount; i++){
            u32_PageIdx = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + pNandDrv->u16_BBtPageIdx[i];
            u32_Err = NC_ReadPages(u32_PageIdx, au8_DataBuf, au8_SpareBuf, 1);
            if(u32_Err != UNFD_ST_SUCCESS){
                nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NC_ReadPage(0x%X)=0x%X, Mark it bad block!!\n", (unsigned int)u32_PageIdx, (unsigned int)u32_Err);
//              u32_Err = drvNAND_MarkBadBlk(u16_PBA);
//              if(u32_Err != UNFD_ST_SUCCESS)
//                  nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Mark bad block(0x%X) failed!!\n", (unsigned int)u16_PBA);
//              return 0;
            }
            if(au8_SpareBuf[pNandDrv->u16_BBtMarker[i]] != 0xFF)
                return 0;
        }
        return 1;
    }

    u32_PageIdx = u16_PBA << pNandDrv->u8_BlkPageCntBits;
    //u32_Err = NC_ReadSector_RIUMode(u32_PageIdx, 0, au8_DataBuf, au8_SpareBuf);
    u32_Err = NC_ReadSectors_MTD(u32_PageIdx, 0, au8_DataBuf, au8_SpareBuf, 1);
    if (u32_Err != UNFD_ST_SUCCESS)
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NC_ReadSectors(0x%X)=0x%X\n", u32_PageIdx, u32_Err);

    if (pBlkInfo->u8_BadBlkMark != 0xFF)
        return 0;

    return 1;
}
U8 drvNAND_CheckAll0xFF(U8* pu8_Buf, U32 u32_ByteCnt)
{
    register U32 u32_i;

    for(u32_i=0; u32_i<u32_ByteCnt; u32_i++)
        if(0xFF != pu8_Buf[u32_i])
            return 0;

    return 0xFF;
}

/* return 1: Free block, 0: not Free block */
U32 drvNAND_IsFreeBlk(U16 u16_PBA)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
    #if NAND_BUF_USE_STACK
    U8 au8_SectorBuf[1024];
    U8 au8_SpareBuf[64];
    #else
    U8 *au8_SectorBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    U8 *au8_SpareBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf;
    #endif
    U32 u32_PageIdx;
    U32 u32_Err;

    // check first page
    u32_PageIdx = u16_PBA << pNandDrv->u8_BlkPageCntBits;
    u32_Err = NC_ReadSectors_MTD(u32_PageIdx, 0, au8_SectorBuf, au8_SpareBuf, 1);
    if (u32_Err != UNFD_ST_SUCCESS)
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Err Code: %X\n", u32_Err);

    if(UNFD_ST_SUCCESS != u32_Err ||
       0xFF != drvNAND_CheckAll0xFF(au8_SpareBuf, pNandDrv->u16_SectorSpareByteCnt))
        return 0; // not free blk

    // check last page
    u32_PageIdx = ((u16_PBA+1) << pNandDrv->u8_BlkPageCntBits) -1;
    u32_Err = NC_ReadSectors_MTD(u32_PageIdx, 0, au8_SectorBuf, au8_SpareBuf, 1);
    if (u32_Err != UNFD_ST_SUCCESS)
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NC_ReadSectors(0x%X)=0x%X\n", u32_PageIdx, u32_Err);

    if(0xFF != drvNAND_CheckAll0xFF(au8_SpareBuf, pNandDrv->u16_SectorSpareByteCnt))
        return 0; // not free blk

    return 1; // free blk
}

// return the free page
U16 drvNAND_FindFreePage(U16 u16_PBA) // binary search
{
	NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
    U8  *pu8_SpareBuf = pNandDrv->PlatCtx_t.pu8_PageSpareBuf;
    U8  *pu8_DataBuf = pNandDrv->PlatCtx_t.pu8_PageDataBuf;
	U32 u32_Row, u32_Ret;
    U16 u16_i, dir;

	//----------------------------------
    u32_Row = u16_PBA << pNandDrv->u8_BlkPageCntBits;
    #if 1//(defined(NC_SEL_FCIE5) && (NC_SEL_FCIE5))
	u32_Ret = NC_ReadSectors_MTD(u32_Row, 0, pu8_DataBuf, pu8_SpareBuf, 1);
    #else
    u32_Ret = NC_ReadSectors(u32_Row, 0, pu8_DataBuf, pu8_SpareBuf, 1);
    #endif
    if(UNFD_ST_SUCCESS==u32_Ret &&
        0xFF==drvNAND_CheckAll0xFF(pu8_SpareBuf, pNandDrv->u16_SectorSpareByteCnt))
	    return 0;

    dir = 1;

    for(u16_i=1; u16_i<=pNandDrv->u8_BlkPageCntBits; u16_i++)
	{
        if(dir)
            u32_Row += pNandDrv->u16_BlkPageCnt >> u16_i;
        else
            u32_Row -= pNandDrv->u16_BlkPageCnt >> u16_i;

        #if 1//(defined(NC_SEL_FCIE5) && (NC_SEL_FCIE5))
		u32_Ret = NC_ReadSectors_MTD(u32_Row, 0, pu8_DataBuf, pu8_SpareBuf, 1);
        #else
        u32_Ret = NC_ReadSectors(u32_Row, 0, pu8_DataBuf, pu8_SpareBuf, 1);
        #endif
        if(UNFD_ST_SUCCESS==u32_Ret &&
            0xFF==drvNAND_CheckAll0xFF(pu8_SpareBuf, pNandDrv->u16_SectorSpareByteCnt))
            dir = 0;
        else
            dir = 1;
    }

	//----------------------------------
	return (u32_Row & pNandDrv->u16_BlkPageCntMask) + dir;
}

// return free LSB Idx
U16 drvNAND_FindFreeLSBPageIdx(U16 u16_PBA) // binary search
{
	NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
    U8  *pu8_SpareBuf = pNandDrv->PlatCtx_t.pu8_PageSpareBuf;
    U8  *pu8_DataBuf = pNandDrv->PlatCtx_t.pu8_PageDataBuf;
	U32 u32_Row, u32_Err;
	U16 u16_PIdx1, u16_PIdx2, u16_PIdxN;
	U16 u16_BlkPageCnt;

	//----------------------------------
	// binary search

	u16_BlkPageCnt = pNandDrv->u16_BlkLowPCnt;//pNandDrv->u16_BlkPageCnt >> pNandDrv->u8_CellType;
	u16_PIdx1 = 0;
	u16_PIdx2 = u16_BlkPageCnt - 1;
	u32_Row = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + ga_tPairedPageMap[u16_PIdx1].u16_LSB;
	//u32_Err = NC_ReadPages(u32_Row, pu8_DataBuf, pu8_SpareBuf, 1);
	u32_Err = NC_ReadSectors_MTD(u32_Row, 0, pu8_DataBuf, pu8_SpareBuf, 1);
	if(0xFF == drvNAND_CheckAll0xFF(pu8_SpareBuf, pNandDrv->u16_SectorSpareByteCnt)
	    && u32_Err == UNFD_ST_SUCCESS)
		return 0;

	u32_Row = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + ga_tPairedPageMap[u16_PIdx2].u16_LSB;
	//u32_Err = NC_ReadPages(u32_Row, pu8_DataBuf, pu8_SpareBuf, 1);
	u32_Err = NC_ReadSectors_MTD(u32_Row, 0, pu8_DataBuf, pu8_SpareBuf, 1);
	if(0 == drvNAND_CheckAll0xFF(pu8_SpareBuf, pNandDrv->u16_SectorSpareByteCnt)
        && u32_Err == UNFD_ST_SUCCESS)
		return u16_BlkPageCnt;//u16_PIdx2;

	while( u16_PIdx1 < u16_PIdx2 -1)
	{
		u16_PIdxN = (u16_PIdx1+u16_PIdx2)/2;

		u32_Row = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + ga_tPairedPageMap[u16_PIdxN].u16_LSB;
		//read page pn
		//u32_Err = NC_ReadPages(u32_Row, pu8_DataBuf, pu8_SpareBuf,  1);
        u32_Err = NC_ReadSectors_MTD(u32_Row, 0, pu8_DataBuf, pu8_SpareBuf, 1);
		if(0xFF == drvNAND_CheckAll0xFF(pu8_SpareBuf, pNandDrv->u16_SectorSpareByteCnt)
			&& u32_Err == UNFD_ST_SUCCESS)
			u16_PIdx2 = u16_PIdxN;
		else
			u16_PIdx1 = u16_PIdxN;
	}

	//u32_Row = ((u16_PBA << pNandDrv->u8_BlkPageCntBits) +ga_tPairedPageMap[u16_PIdx1 + 1].u16_LSB);

	return u16_PIdx1 + 1; //(u32_Row & pNandDrv->u16_BlkPageCntMask);
}

U32 drvNAND_MoveBlkData(U16 u16_DestBlk, U16 u16_SrcBlk, U16 u16_PageCnt)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
    #if NAND_BUF_USE_STACK
    U8 au8_PageBuf[4096];
    U8 au8_SpareBuf[128];
    #else
    U8 *au8_PageBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    U8 *au8_SpareBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf;
    #endif
    U32 u32_Ret;
    U16 u16_i, u32_SrcRow, u32_DestRow;

    u32_SrcRow = u16_SrcBlk << pNandDrv->u8_BlkPageCntBits;
    u32_DestRow = u16_DestBlk << pNandDrv->u8_BlkPageCntBits;

    for(u16_i=0; u16_i<u16_PageCnt; u16_i++)
    {
        u32_Ret = NC_ReadPages(u32_SrcRow + u16_i, au8_PageBuf, au8_SpareBuf, 1);
        if(UNFD_ST_SUCCESS != u32_Ret)
            return u32_Ret;
        u32_Ret = NC_WritePages(u32_DestRow + u16_i, au8_PageBuf, au8_SpareBuf, 1);
        if(UNFD_ST_SUCCESS != u32_Ret)
        {
            drvNAND_MarkBadBlk(u16_DestBlk);
            return u32_Ret;
        }
    }
    return UNFD_ST_SUCCESS;
}


U32 drvNAND_ReadBlk(U8 *pu8_DestAddr, U8 u8_PartType, U16 u16_PBA,
                            U16 u16_LBA, U32 u32_StartSector, U32 u32_SectorCnt)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
#if NAND_BUF_USE_STACK
    U8 au8_PageBuf[4096];
    U8 au8_SpareBuf[128];
#else
    U8 *au8_PageBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    //U8 *au8_SpareBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf;
#endif
    U32 u32_Err;
    //BLK_INFO_t *pBlkInfo = (BLK_INFO_t*)au8_SpareBuf;
    U32 u32_PageIdx;
    U16 u16_PageCnt;
    U16 u16_SectorCntRead;
    U16 u16_SectorIdxInPage;

    nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "BlkIdx:%d, BlkNo:%d, StartSector:%d, SectorCnt:%d\n", u16_PBA, u16_LBA, u32_StartSector, u32_SectorCnt);
    #if defined(ENABLE_CB_BEFORE_DMA) && ENABLE_CB_BEFORE_DMA
    pNandDrv->u8_StartCB = 1;
    #endif

    /* Read unaligned sectors first */
    if( u32_StartSector & pNandDrv->u16_PageSectorCntMask)
    {
        u16_SectorIdxInPage = u32_StartSector & pNandDrv->u16_PageSectorCntMask;
        u16_SectorCntRead = u32_SectorCnt > ((U32)pNandDrv->u16_PageSectorCnt-(U32)u16_SectorIdxInPage) ? ((U32)pNandDrv->u16_PageSectorCnt-(U32)u16_SectorIdxInPage) : (u32_SectorCnt);
        u32_PageIdx = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + (u32_StartSector>>pNandDrv->u8_PageSectorCntBits);
        #if defined(ENABLE_CB_BEFORE_DMA) && ENABLE_CB_BEFORE_DMA
        pNandDrv->u16_LogiSectorCntCB = u16_SectorCntRead;
        #endif
        u32_Err = NC_ReadPages(u32_PageIdx, au8_PageBuf, (U8*)0, 1);
        if (u32_Err != UNFD_ST_SUCCESS) {

            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Reading page 0x%04x failed with EC: 0x%04x\n",
                       u32_PageIdx, u32_Err);
            #if defined(ENABLE_CB_BEFORE_DMA) && ENABLE_CB_BEFORE_DMA
            pNandDrv->u8_StartCB = 0;
            #endif
            return u32_Err;
        }

        memcpy(pu8_DestAddr, (au8_PageBuf+(u16_SectorIdxInPage<<9)), (u16_SectorCntRead<<9));
        pu8_DestAddr += (u16_SectorCntRead<<9);
        u32_StartSector += u16_SectorCntRead;
        u32_SectorCnt -= u16_SectorCntRead;
    }

    /* Read aligned sectors then */
    u32_PageIdx = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + (u32_StartSector>>pNandDrv->u8_PageSectorCntBits);
    u16_PageCnt = (u32_SectorCnt>>pNandDrv->u8_PageSectorCntBits);
    if(pNandDrv->u8_SwPatchWaitRb)
    {
        while (u16_PageCnt)
        {
            #if defined(ENABLE_CB_BEFORE_DMA) && ENABLE_CB_BEFORE_DMA
            pNandDrv->u16_LogiSectorCntCB = pNandDrv->u16_PageSectorCnt;
            #endif
            u32_Err = NC_ReadPages(u32_PageIdx, pu8_DestAddr, (U8*)0, 1);
            if (u32_Err != UNFD_ST_SUCCESS) {

                nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Reading page 0x%04x failed with EC: 0x%04x\n",
                           u32_PageIdx, u32_Err);
                #if defined(ENABLE_CB_BEFORE_DMA) && ENABLE_CB_BEFORE_DMA
                pNandDrv->u8_StartCB = 0;
                #endif
                return u32_Err;
            }

            pu8_DestAddr += pNandDrv->u16_PageByteCnt;
            u32_PageIdx++;
            u16_PageCnt--;
            u32_StartSector += pNandDrv->u16_PageSectorCnt;
            u32_SectorCnt -= pNandDrv->u16_PageSectorCnt;
        }
    }
    else
    {
        if(u16_PageCnt>0)
        {
            #if defined(ENABLE_CB_BEFORE_DMA) && ENABLE_CB_BEFORE_DMA
            pNandDrv->u16_LogiSectorCntCB = u16_PageCnt*pNandDrv->u16_PageSectorCnt;
            #endif
            u32_Err = NC_ReadPages(u32_PageIdx, pu8_DestAddr, (U8*)0, u16_PageCnt);
            if (u32_Err != UNFD_ST_SUCCESS) {

                nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Reading page 0x%04x failed with EC: 0x%04x\n",
                           u32_PageIdx, u32_Err);
                #if defined(ENABLE_CB_BEFORE_DMA) && ENABLE_CB_BEFORE_DMA
                pNandDrv->u8_StartCB = 0;
                #endif
                return u32_Err;
            }

            pu8_DestAddr += (pNandDrv->u16_PageByteCnt*u16_PageCnt);
            u32_PageIdx += u16_PageCnt;
            u32_StartSector += (pNandDrv->u16_PageSectorCnt*u16_PageCnt);
            u32_SectorCnt -= (pNandDrv->u16_PageSectorCnt*u16_PageCnt);
        }
    }

    /* Read remaining unaligned sectors finally */
    if( u32_SectorCnt )
    {
        #if defined(ENABLE_CB_BEFORE_DMA) && ENABLE_CB_BEFORE_DMA
        pNandDrv->u16_LogiSectorCntCB = u32_SectorCnt;
        #endif
        u32_PageIdx = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + (u32_StartSector>>pNandDrv->u8_PageSectorCntBits);
        u32_Err = NC_ReadPages(u32_PageIdx, au8_PageBuf, (U8*)0, 1);
        if (u32_Err != UNFD_ST_SUCCESS) {

            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Reading page 0x%04x failed with EC: 0x%04x\n",
                       u32_PageIdx, u32_Err);
            #if defined(ENABLE_CB_BEFORE_DMA) && ENABLE_CB_BEFORE_DMA
            pNandDrv->u8_StartCB = 0;
            #endif
            return u32_Err;
        }
        memcpy(pu8_DestAddr, au8_PageBuf, (u32_SectorCnt<<9));
    }
    #if defined(ENABLE_CB_BEFORE_DMA) && ENABLE_CB_BEFORE_DMA
    pNandDrv->u8_StartCB = 0;
    #endif
    return UNFD_ST_SUCCESS;
}


U32 drvNAND_WriteBlkNonBackup(U8 *pu8_DestAddr, U8 u8_PartType, U16 u16_PBA,
                     U16 u16_LBA,   U32 u32_StartSector,
                     U32 u32_SectorCnt)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
#if NAND_BUF_USE_STACK
    U8 au8_PageBuf[4096];
    U8 au8_SpareBuf[128];
#else
    U8 *au8_PageBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    U8 *au8_SpareBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf;
#endif
    U32 u32_Err;
    BLK_INFO_t *pBlkInfo = (BLK_INFO_t*)au8_SpareBuf;
    U16 u16_PageSectorCnt;
    U8 u8_PageSectorCntBits;
    //U16 u16_PageSectorCntMask;
    U32 u32_PageIdx;
    U16 u16_PageCnt;

    nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "BlkIdx:%d, BlkNo:%d, StartSector:%d, SectorCnt:%d\n", u16_PBA, u16_LBA, u32_StartSector, u32_SectorCnt);

    /* sector size from AP is 512 Bytes, not NAND sector size */
    u16_PageSectorCnt = pNandDrv->u16_PageByteCnt >> 9;
    u8_PageSectorCntBits = drvNAND_CountBits(u16_PageSectorCnt);
    //u16_PageSectorCntMask = (1<<u8_PageSectorCntBits)-1;

    memset(au8_SpareBuf, '\0', sizeof(*pBlkInfo));
    pBlkInfo->u8_BadBlkMark = 0xFF;
    pBlkInfo->u8_PartType = u8_PartType;
    pBlkInfo->u16_BlkAddr = u16_LBA;

    u32_PageIdx = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + (u32_StartSector>>u8_PageSectorCntBits);
    u16_PageCnt = (u32_SectorCnt>>u8_PageSectorCntBits);

    while (u16_PageCnt)
    {
        u32_Err = NC_WritePages(u32_PageIdx, pu8_DestAddr, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS) {

            nand_debug(2, 1, "Writting page 0x%04x failed with EC: 0x%04x\n",
                       u32_PageIdx, u32_Err);

            return u32_Err;
        }

        pu8_DestAddr += pNandDrv->u16_PageByteCnt;
        u32_PageIdx++;
        u16_PageCnt--;
        u32_SectorCnt -= u16_PageSectorCnt;
    }

    if( u32_SectorCnt )
    {
        memset(au8_PageBuf, 0xFF, pNandDrv->u16_PageByteCnt);
        memcpy(au8_PageBuf, pu8_DestAddr, u32_SectorCnt<<9);

        u32_Err = NC_WritePages(u32_PageIdx, au8_PageBuf, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS) {

            nand_debug(2, 1, "Writting page 0x%04x failed with EC: 0x%04x\n",
                       u32_PageIdx, u32_Err);

            return u32_Err;
        }
    }

    return UNFD_ST_SUCCESS;
}

U32 drvNAND_WriteBlk(U8 *pu8_DestAddr, U8 u8_PartType, U16 u16_PBA,
                     U16 u16_LBA, U32 u32_StartSector,
                     U32 u32_SectorCnt)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
#if NAND_BUF_USE_STACK
    U8 au8_PageBuf[4096];
    U8 au8_SpareBuf[128];
#else
    U8 *au8_PageBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    U8 *au8_SpareBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf;
#endif
    U32 u32_Err;
    BLK_INFO_t *pBlkInfo = (BLK_INFO_t*)au8_SpareBuf;
    U16 u16_PageSectorCnt;
    U8 u8_PageSectorCntBits;
    U16 u16_PageSectorCntMask;
    U32 u32_SrcPageIdx, u32_BakPageIdx;
    U16 u16_PageCnt;
    U16 u16_SectorCntRead;
    U16 u16_SectorIdxInPage;
    U16 u16_i;
    U16 u16_LastWrittenPage = 0;
    U16 u16_BakBlkIdx;
    U8 u8_GoodBlkCnt;

    /* sector size from AP is 512 Bytes, not NAND sector size */
    u16_PageSectorCnt = pNandDrv->u16_PageByteCnt >> 9;
    u8_PageSectorCntBits = drvNAND_CountBits(u16_PageSectorCnt);
    u16_PageSectorCntMask = (1<<u8_PageSectorCntBits)-1;

    if( (u32_StartSector & u16_PageSectorCntMask)  == 0 )
    {
        u32_SrcPageIdx = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + (u32_StartSector>>u8_PageSectorCntBits);
        u32_Err = NC_ReadPages(u32_SrcPageIdx, au8_PageBuf, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS)
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Reading page 0x%04x failed with EC: 0x%04x\n",
                       u32_SrcPageIdx, u32_Err);

            return u32_Err;
        }

        for(u16_i=0 ; u16_i<pNandDrv->u16_SpareByteCnt ; u16_i++)
        {
            if(au8_SpareBuf[u16_i] != 0xFF)
                goto LABEL_WRITE_BLK;
        }

        for(u16_i=0 ; u16_i<pNandDrv->u16_PageByteCnt ; u16_i++)
        {
            if(au8_PageBuf[u16_i] != 0xFF)
                goto LABEL_WRITE_BLK;
        }

        /* empty page, needn't to backup*/
        u32_Err = drvNAND_WriteBlkNonBackup(pu8_DestAddr, u8_PartType, u16_PBA,
                                    u16_LBA, u32_StartSector, u32_SectorCnt);
        return u32_Err;
    }

LABEL_WRITE_BLK:
    nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "BlkIdx:%d, BlkNo:%d, StartSector:%d, SectorCnt:%d\n", u16_PBA, u16_LBA, u32_StartSector, u32_SectorCnt);

    /* Find physical Backup BlkIdx */
    u16_BakBlkIdx = pNandDrv->pPartInfo->records->u16_StartBlk-1;
    u8_GoodBlkCnt = 0;
    while(1)
    {
        if(!drvNAND_IsGoodBlk(u16_BakBlkIdx))
        {
            nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1, "Skip bad blk: 0x%04x\n", u16_BakBlkIdx);
        }
        else
        {
            // reserve last 4 good blks for E2P0/E2P1/NVRAM0/NVRAM1
            if(++u8_GoodBlkCnt == 5)
                break;
        }

        if((--u16_BakBlkIdx)<(pNandDrv->pPartInfo->records->u16_StartBlk-7))
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "too many bad block\n");
            return UNFD_ST_ERR_LACK_BLK;
        }
    }
    nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "u16_BakBlkIdx:%d\n", u16_BakBlkIdx);

    // erase bak block
    u32_Err = drvNAND_ErasePhyBlk(u16_BakBlkIdx);
    if (u32_Err != UNFD_ST_SUCCESS) {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Erase Blk 0x%04x failed with EC: 0x%08x\n",
                   u16_BakBlkIdx, u32_Err);
        drvNAND_MarkBadBlk(u16_BakBlkIdx);
        return u32_Err;
    }

    for(u16_i=0 ; u16_i<(u32_StartSector>>u8_PageSectorCntBits) ; u16_i++)
    {
        u32_SrcPageIdx = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + u16_i;
        u32_Err = NC_ReadPages(u32_SrcPageIdx, au8_PageBuf, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS)
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Reading page 0x%04x failed with EC: 0x%04x\n",
                       u32_SrcPageIdx, u32_Err);

            return u32_Err;
        }

        u32_BakPageIdx = (u16_BakBlkIdx << pNandDrv->u8_BlkPageCntBits) + u16_i;
        u32_Err = NC_WritePages(u32_BakPageIdx, au8_PageBuf, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS)
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Writing page 0x%04x failed with EC: 0x%04x\n",
                       u32_BakPageIdx, u32_Err);
            drvNAND_MarkBadBlk(u16_BakBlkIdx);
            return u32_Err;
        }
        u16_LastWrittenPage = u16_i;
    }

    /* Read unaligned sectors first */
    if( u32_StartSector & u16_PageSectorCntMask )
    {
        u32_SrcPageIdx = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + (u32_StartSector>>u8_PageSectorCntBits);
        u32_Err = NC_ReadPages(u32_SrcPageIdx, au8_PageBuf, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS) {

            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Reading page 0x%04x failed with EC: 0x%04x\n",
                       u32_SrcPageIdx, u32_Err);

            return u32_Err;
        }
        u16_SectorIdxInPage = u32_StartSector & u16_PageSectorCntMask;
        u16_SectorCntRead = u32_SectorCnt > ((U32)u16_PageSectorCnt-(U32)u16_SectorIdxInPage) ? ((U32)u16_PageSectorCnt-(U32)u16_SectorIdxInPage) : (u32_SectorCnt);

        memcpy((au8_PageBuf+(u16_SectorIdxInPage<<9)), pu8_DestAddr, (u16_SectorCntRead<<9));
        u32_BakPageIdx = (u16_BakBlkIdx << pNandDrv->u8_BlkPageCntBits) + (u32_StartSector>>u8_PageSectorCntBits);
        u32_Err = NC_WritePages(u32_BakPageIdx, au8_PageBuf, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS) {

            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Writing page 0x%04x failed with EC: 0x%04x\n",
                       u32_BakPageIdx, u32_Err);
            drvNAND_MarkBadBlk(u16_BakBlkIdx);
            return u32_Err;
        }

        pu8_DestAddr += (u16_SectorCntRead<<9);
        u32_StartSector += u16_SectorCntRead;
        u32_SectorCnt -= u16_SectorCntRead;
        u16_LastWrittenPage = (u32_StartSector>>u8_PageSectorCntBits);
    }

    /* Read aligned sectors then */
    memset(au8_SpareBuf, '\0', sizeof(*pBlkInfo));
    pBlkInfo->u8_BadBlkMark = 0xFF;
    pBlkInfo->u8_PartType = u8_PartType;
    pBlkInfo->u16_BlkAddr = u16_LBA;

    u32_BakPageIdx = (u16_BakBlkIdx << pNandDrv->u8_BlkPageCntBits) + (u32_StartSector>>u8_PageSectorCntBits);
    u16_PageCnt = (u32_SectorCnt>>u8_PageSectorCntBits);
    while (u16_PageCnt)
    {
        u32_Err = NC_WritePages(u32_BakPageIdx, pu8_DestAddr, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS) {

            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Writing page 0x%04x failed with EC: 0x%04x\n",
                       u32_BakPageIdx, u32_Err);
            drvNAND_MarkBadBlk(u16_BakBlkIdx);
            return u32_Err;
        }

        u16_LastWrittenPage = u32_BakPageIdx -(u16_BakBlkIdx << pNandDrv->u8_BlkPageCntBits);
        pu8_DestAddr += pNandDrv->u16_PageByteCnt;
        u32_BakPageIdx++;
        u16_PageCnt--;
        u32_StartSector += u16_PageSectorCnt;
        u32_SectorCnt -= u16_PageSectorCnt;
    }

    /* Read remaining unaligned sectors finally */
    if( u32_SectorCnt )
    {
        u32_SrcPageIdx = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + (u32_StartSector>>u8_PageSectorCntBits);
        u32_Err = NC_ReadPages(u32_SrcPageIdx, au8_PageBuf, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS) {

            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Reading page 0x%04x failed with EC: 0x%04x\n",
                       u32_SrcPageIdx, u32_Err);

            return u32_Err;
        }

        memcpy(au8_PageBuf, pu8_DestAddr, (u32_SectorCnt<<9));
        u32_BakPageIdx = (u16_BakBlkIdx << pNandDrv->u8_BlkPageCntBits) + (u32_StartSector>>u8_PageSectorCntBits);
        u32_Err = NC_WritePages(u32_BakPageIdx, au8_PageBuf, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS) {

            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Writing page 0x%04x failed with EC: 0x%04x\n",
                       u32_BakPageIdx, u32_Err);
            drvNAND_MarkBadBlk(u16_BakBlkIdx);
            return u32_Err;
        }
        u16_LastWrittenPage = (u32_StartSector>>u8_PageSectorCntBits);
    }

    for(u16_i=(u16_LastWrittenPage+1) ; u16_i<pNandDrv->u16_BlkPageCnt; u16_i++)
    {
        u32_SrcPageIdx = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + u16_i;
        u32_Err = NC_ReadPages(u32_SrcPageIdx, au8_PageBuf, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS)
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Reading page 0x%04x failed with EC: 0x%04x\n",
                       u32_SrcPageIdx, u32_Err);

            return u32_Err;
        }

        u32_BakPageIdx = (u16_BakBlkIdx << pNandDrv->u8_BlkPageCntBits) + u16_i;
        u32_Err = NC_WritePages(u32_BakPageIdx, au8_PageBuf, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS)
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Writing page 0x%04x failed with EC: 0x%04x\n",
                       u32_BakPageIdx, u32_Err);
            drvNAND_MarkBadBlk(u16_BakBlkIdx);
            return u32_Err;
        }
    }

    // erase src block
    u32_Err = drvNAND_ErasePhyBlk(u16_PBA);
    if (u32_Err != UNFD_ST_SUCCESS) {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Erase Blk 0x%04x failed with EC: 0x%08x\n",
                   u16_PBA, u32_Err);
        drvNAND_MarkBadBlk(u16_PBA);
        return u32_Err;
    }

    // copy bak to src
    for(u16_i=0 ; u16_i<pNandDrv->u16_BlkPageCnt; u16_i++)
    {
        u32_BakPageIdx = (u16_BakBlkIdx << pNandDrv->u8_BlkPageCntBits) + u16_i;
        u32_Err = NC_ReadPages(u32_BakPageIdx, au8_PageBuf, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS)
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Reading page 0x%04x failed with EC: 0x%04x\n",
                       u32_BakPageIdx, u32_Err);

            return u32_Err;
        }

        u32_SrcPageIdx = (u16_PBA << pNandDrv->u8_BlkPageCntBits) + u16_i;
        u32_Err = NC_WritePages(u32_SrcPageIdx, au8_PageBuf, au8_SpareBuf, 1);
        if (u32_Err != UNFD_ST_SUCCESS)
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Reading page 0x%04x failed with EC: 0x%04x\n",
                       u32_SrcPageIdx, u32_Err);
            drvNAND_MarkBadBlk(u16_PBA);
            return u32_Err;
        }
    }

    // erase bak block
    u32_Err = drvNAND_ErasePhyBlk(u16_BakBlkIdx);
    if (u32_Err != UNFD_ST_SUCCESS) {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Erase Blk 0x%04x failed with EC: 0x%08x\n",
                   u16_BakBlkIdx, u32_Err);
        drvNAND_MarkBadBlk(u16_BakBlkIdx);
        return u32_Err;
    }

    return UNFD_ST_SUCCESS;
}

U32 drvNAND_MarkBadBlk(U16 u16_PBA)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
    #if 1 //NAND_BUF_USE_STACK
    //U8 au8_PageBuf[4096];
    U8 *au8_PageBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    U8 au8_SpareBuf[128];
    #else
    U8 *au8_PageBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    U8 *au8_SpareBuf = (U8*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf;
    #endif
    U32 u32_Err;
    U32 u32_PageIdx;

    u32_Err = drvNAND_ErasePhyBlk(u16_PBA);

    //memset(au8_PageBuf, '\0', pNandDrv->u16_PageByteCnt);
    memset(au8_SpareBuf, '\0', 128);//pNandDrv->u16_SpareByteCnt);

    /* Clear the 1st page (main + spare) of the block */
    u32_PageIdx = u16_PBA << pNandDrv->u8_BlkPageCntBits;
    u32_Err = NC_WritePages(u32_PageIdx, au8_PageBuf, au8_SpareBuf, 1);
    if (u32_Err != UNFD_ST_SUCCESS)
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Mark Blk 0x%04x failed with ErrCode: 0x%08x\n",
                   u16_PBA, u32_Err);
    /* Clear the last page (main + spare) of the block */
    u32_PageIdx += pNandDrv->u16_BlkPageCnt - 1;
    u32_Err = NC_WritePages(u32_PageIdx, au8_PageBuf, au8_SpareBuf, 1);
    if (u32_Err != UNFD_ST_SUCCESS)
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Mark Blk 0x%04x failed with ErrCode: 0x%08x\n",
                   u16_PBA, u32_Err);

    return u32_Err;
}

U32 drvNAND_ErasePhyBlk(U16 u16_PBA)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();

    return NC_EraseBlk (u16_PBA << pNandDrv->u8_BlkPageCntBits);
}


U32 drvNAND_ReadPhyPage(U32 u32_PageIdx, U8 *pu8_Data, U8 *pu8_Spare)
{
    return NC_ReadPages(u32_PageIdx, pu8_Data, pu8_Spare, 1);
}

U32 drvNAND_WritePhyPage(U32 u32_PageIdx, U8 *pu8_Data, U8 *pu8_Spare)
{
    return NC_WritePages(u32_PageIdx, pu8_Data, pu8_Spare, 1);
}
U8 gau8_OneBitCnt[256]={
// 0   1   2   3   4   5   6   7   8   9   A   B   C  D   E   F
   0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, // 0
   1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, // 1
   1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, // 2
   2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, // 3
   1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, // 4
   2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, // 5
   2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, // 6
   3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, // 7
   1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, // 8
   2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, // 9
   2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, // A
   3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, // B
   2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, // C
   3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, // D
   3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, // E
   4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8  // F
};

U8 gau8_ZeroBitCnt[256]={
 // 0 1  2  3  4 5  6  7  8   9  A  B  C D  E  F
 8, 7, 7, 6, 7, 6, 6, 5, 7, 6, 6, 5, 6, 5, 5, 4, // 0
 7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3, // 1
 7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3, // 2
 6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, // 3
 7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3, // 4
 6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, // 5
 6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, // 6
 5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1, // 7
 7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3, // 8
 6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, // 9
 6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, // A
 5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1, // B
 6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, // C
 5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1, // D
 5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1, // E
 4, 3, 3, 2, 3, 2, 2, 1, 3, 2, 2, 1, 2, 1, 1, 0  // F
};
/*
* Check whether the cause of ECC fail is reading a non-all0xff empty page.
* @main: Main Data of a NAND Page
* @spare: Spare Data of a NAND Page
* @return 1: if False Alarm 0: Real ECC Fail
*/ 
int nand_CheckEmptyPageFalseAlarm(U8 *main, U8 *spare)
{
     NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
     int i, j, ECCErrBitCnt;
    
     //check if false alarm casued by empty page
     for(j = 0; j < pNandDrv->u16_PageSectorCnt; j++)
     {
    	 ECCErrBitCnt = 0;
    	 for(i = 0; i < pNandDrv->u16_SectorSpareByteCnt; i++)
    	 {
    		 if(j*pNandDrv->u16_SectorSpareByteCnt + i < 512)
    			 ECCErrBitCnt += gau8_ZeroBitCnt[spare[ j*pNandDrv->u16_SectorSpareByteCnt + i]];
    		 if(ECCErrBitCnt > pNandDrv->u16_ECCCorretableBit)
    			 return 0;
    	 }
    	 
    	 for(i = 0; i < pNandDrv->u16_SectorByteCnt; i++)
    	 {
    		 ECCErrBitCnt += gau8_ZeroBitCnt[main[ j*pNandDrv->u16_SectorByteCnt + i]];
    		 if(ECCErrBitCnt > pNandDrv->u16_ECCCorretableBit)
    			 return 0;
    	 }
     }
    
     memset(spare, 0xFF, pNandDrv->u16_SpareByteCnt);
     memset(main, 0xFF, pNandDrv->u16_PageByteCnt);  
     return 1;

}

int nand_CheckEmptySectorsFalseAlarm(U8 *main, U8 *spare, U16 u16_SectorCnt)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	int  i, j, ECCErrBitCnt;

	//check if false alarm casued by empty page
	for(j = 0; j < u16_SectorCnt; j++)
	{
		ECCErrBitCnt = 0;
		for(i = 0; i < pNandDrv->u16_SectorSpareByteCnt; i++)
		{
			if(j*pNandDrv->u16_SectorSpareByteCnt + i < 512)
				ECCErrBitCnt += gau8_ZeroBitCnt[spare[ j*pNandDrv->u16_SectorSpareByteCnt + i]];
			if(ECCErrBitCnt > pNandDrv->u16_ECCCorretableBit)
				return 0;
		}
		
		for(i = 0; i < pNandDrv->u16_SectorByteCnt; i++)
		{
			ECCErrBitCnt += gau8_ZeroBitCnt[main[ j*pNandDrv->u16_SectorByteCnt + i]];
			if(ECCErrBitCnt > pNandDrv->u16_ECCCorretableBit)
				return 0;
		}
	}
	memset(spare, 0xFF, pNandDrv->u16_SectorSpareByteCnt*u16_SectorCnt);
	memset(main, 0xFF, pNandDrv->u16_SectorByteCnt*u16_SectorCnt);
	return 1;
}

/*
* Check empty page ot not.
* @spare: Spare Data of a NAND Page
* @return 2: spare area bitflip	1: Empty page 0: Not empty page
*/

int nand_CheckEmptyPage(U8 *spare)
{
      NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
      U16 u16_ecc_start_byte;
      int i, j, ECCErrBitCnt;
      int Spare_bitflips=0;
    
      u16_ecc_start_byte = pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt;
      for(j = 0; j < pNandDrv->u16_PageSectorCnt; j++)
      {
    	  ECCErrBitCnt = 0;
    	  for(i = u16_ecc_start_byte; i < pNandDrv->u16_SectorSpareByteCnt; i++)
    	  {
              if(j*pNandDrv->u16_SectorSpareByteCnt + i < 512)    	  
    		      ECCErrBitCnt += gau8_ZeroBitCnt[spare[ j*pNandDrv->u16_SectorSpareByteCnt + i]];
    		  if(ECCErrBitCnt > pNandDrv->u16_ECCCorretableBit)
    			  return 0;
    	  }
    	  if(ECCErrBitCnt >0)
    		  Spare_bitflips += ECCErrBitCnt;		  
      }
    
      if(Spare_bitflips >0)
    	  return 2;
    
      return 1;   
}

U32 drvNAND_LFSRReadPhyPage(U32 u32_PageIdx, U8 *pu8_Data, U8 *pu8_Spare)
{
    return NC_ReadPages(u32_PageIdx, pu8_Data, pu8_Spare, 1);
}

U32 drvNAND_LFSRWritePhyPage(U32 u32_PageIdx, U8 *pu8_Data, U8 *pu8_Spare)
{
    return NC_WritePages(u32_PageIdx, pu8_Data, pu8_Spare, 1);
}


/*
 * Refresh block data to avoid read disturbance happens.
 * @u32_BlkRow: Row of Target block who has correctable ECC bits
 * @u8_torture: Flag of whether to torture the target block if Read-Retry happens
 * @*nand_markbad: Customize Markbad function ex: mtd mark bad function
 */
int nand_ReadDisturbance_BigImg(U32  u32_BlkRow, U32 u32_BakBlkRow, U32 (*nand_markbad)(U32), U8 u8_CustSpareMarker, U8 u8_CustMarkerIndex)
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U32 u32_GoodBlkIdx, u32_Err, u32_Row;
    U16 u16_i, *pu16_BlkPBA;
    U8 *pu8_PageDataBuf = pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    U8 *pu8_PageSpareBuf = pNandDrv->PlatCtx_t.pu8_PageSpareBuf;

    //uint8_t *pu8_BakPageDataBuf = (uint8_t*)kmalloc(pNandDrv->u16_PageByteCnt, GFP_KERNEL);
    //uint8_t *pu8_BakPageSpareBuf = (uint8_t*)kmalloc(pNandDrv->u16_SpareByteCnt, GFP_KERNEL);
    uint8_t *pu8_BakPageDataBuf = 0;
    uint8_t *pu8_BakPageSpareBuf = 0;

    pu8_BakPageDataBuf = (uint8_t*)kmalloc(pNandDrv->u16_PageByteCnt, GFP_KERNEL);
    if(pu8_BakPageDataBuf == NULL)
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Can't alloc memory for backup buffer\n");
        return -1;
    }

    pu8_BakPageSpareBuf = (uint8_t*)kmalloc(pNandDrv->u16_SpareByteCnt, GFP_KERNEL);
    if(pu8_BakPageSpareBuf == NULL)
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Can't alloc memory for backup buffer\n");
	kfree(pu8_BakPageDataBuf);
        return -1;
    }

    pu16_BlkPBA = (U16 *)(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt);
RETRY:
    //find last good block of cis partition
    u32_GoodBlkIdx = drvNAND_GetBackupBlk();
    if(u32_GoodBlkIdx==0)//No free block to do bakup
    {
        kfree(pu8_BakPageDataBuf);
        kfree(pu8_BakPageSpareBuf);
        return 0;
    }
    nand_debug(UNFD_DEBUG_LEVEL, 1,"Blk %X is the last good block\n", (unsigned int)u32_GoodBlkIdx);

    // write to empty block, so called backup block

    memset(pu8_PageSpareBuf, 0xff, pNandDrv->u16_SpareByteCnt);
    nand_debug(UNFD_DEBUG_LEVEL, 1,"Write data to Blk %X\n", (unsigned int)u32_GoodBlkIdx);
    for(u16_i = 0; u16_i < pNandDrv->u16_BlkPageCnt; u16_i ++)
    {
        //read back each page from target block
        u32_Row = (u32_BlkRow) + u16_i;
        u32_Err = drvNAND_LFSRReadPhyPage(u32_Row, pu8_PageDataBuf, pu8_PageSpareBuf);
        if(u32_Err != UNFD_ST_SUCCESS)
        {
            if(nand_CheckEmptyPageFalseAlarm(pu8_PageDataBuf, pu8_PageSpareBuf) != 1)
            {
                nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NAND fatal error page @ %X,when reading target block\n",(unsigned int)u32_Row);
                nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Please re-program nand\n");
                kfree(pu8_BakPageDataBuf);
                kfree(pu8_BakPageSpareBuf);
                return -1;
            }
        }

        u32_Err = drvNAND_LFSRReadPhyPage(u32_BakBlkRow + u16_i, pu8_BakPageDataBuf, pu8_BakPageSpareBuf);
        if(u32_Err != UNFD_ST_SUCCESS)
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NAND fatal error page @ %X,when reading backup block\n",(unsigned int)(u32_BakBlkRow + u16_i));

            kfree(pu8_BakPageDataBuf);
            kfree(pu8_BakPageSpareBuf);
            return -1;
        }

        if(memcmp((const void *) pu8_PageDataBuf, (const void *) pu8_BakPageDataBuf, pNandDrv->u16_PageByteCnt)!=0)
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "1 NAND fatal error page @ %X,when compare @ %X\n",(unsigned int)u32_Row, (unsigned int)(u32_BakBlkRow + u16_i));
            dump_mem(pu8_PageDataBuf, 16);
            dump_mem(pu8_BakPageDataBuf, 16);
            kfree(pu8_BakPageDataBuf);
            kfree(pu8_BakPageSpareBuf);
            return -1;
        }

        // write data to backup block with target PBA and marker recorded in spare
        // spare of 1st sector   -> only 0xFF
        // spare of 2nd sector  -> blockrow
        // spare of 3rd sector   -> marker of 3697
        // spare of 4th sector   -> Customer Marker

        pu8_PageSpareBuf[0] = 0xFF;
//      SPAREINFO->u8_BadBlkMarker = 0xFF;
//      SPAREINFO->u16_BackupPBA = u32_BlkRow >> pNandDrv->u8_BlkPageCntBits;
        *pu16_BlkPBA = u32_BlkRow >> pNandDrv->u8_BlkPageCntBits;
//      pu8_PageSpareBuf[3] = 0x36;
//      pu8_PageSpareBuf[4] = 0x97;
        pu8_PageSpareBuf[pNandDrv->u16_SectorSpareByteCnt * 2] = 0x36;
        pu8_PageSpareBuf[pNandDrv->u16_SectorSpareByteCnt * 2 + 1] = 0x97;

        pu8_PageSpareBuf[pNandDrv->u16_SectorSpareByteCnt * 3] = u8_CustSpareMarker;
        pu8_PageSpareBuf[pNandDrv->u16_SectorSpareByteCnt * 3 + 1] = u8_CustMarkerIndex;

        u32_Row = (u32_GoodBlkIdx << pNandDrv->u8_BlkPageCntBits) + u16_i;

        u32_Err = drvNAND_LFSRWritePhyPage(u32_Row, pu8_PageDataBuf, pu8_PageSpareBuf);
        if(u32_Err != UNFD_ST_SUCCESS)
        {
            //mark bad and goto research a good block
            if(nand_markbad == NULL)
                drvNAND_MarkBadBlk(u32_GoodBlkIdx);
            else
                nand_markbad(u32_GoodBlkIdx);

            goto RETRY;
        }
    }

    //erase target block
    nand_debug(UNFD_DEBUG_LEVEL, 1,"Erase Target block %X\n", (unsigned int)(u32_BlkRow >> pNandDrv->u8_BlkPageCntBits));
    u32_Err = NC_EraseBlk(u32_BlkRow);
    if(u32_Err != UNFD_ST_SUCCESS)
    {
        //complicated need to do
        //mark bad and do shifting block
    }
    //if torture needed torture block for recheck read retry

    // write data to target block if torture ok
    // if torture fail -> do shifting block...
    // @FIXME

    nand_debug(UNFD_DEBUG_LEVEL, 1, "Write data back to target block with page count = %d\n", pNandDrv->u16_BlkPageCnt);
    //read each page from backup block and write back to target block
    for(u16_i = 0; u16_i < pNandDrv->u16_BlkPageCnt; u16_i ++)
    {
        u32_Row = (u32_GoodBlkIdx << pNandDrv->u8_BlkPageCntBits) + u16_i;
        u32_Err = drvNAND_LFSRReadPhyPage(u32_Row, pu8_PageDataBuf, pu8_PageSpareBuf);

        if(u32_Err != UNFD_ST_SUCCESS)
        {
            if(nand_CheckEmptyPageFalseAlarm(pu8_PageDataBuf, pu8_PageSpareBuf) != 1)
            {
                nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NAND fatal error page @ %X,when reading target block\n",(unsigned int)u32_Row);
                nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Please re-program nand\n");
                kfree(pu8_BakPageDataBuf);
                kfree(pu8_BakPageSpareBuf);
                return -1;
            }
        }

        u32_Err = drvNAND_LFSRReadPhyPage(u32_BakBlkRow + u16_i, pu8_BakPageDataBuf, pu8_BakPageSpareBuf);
        if(u32_Err != UNFD_ST_SUCCESS)
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NAND fatal error page @ %X,when reading backup block\n",(unsigned int)(u32_BakBlkRow + u16_i));

            kfree(pu8_BakPageDataBuf);
            kfree(pu8_BakPageSpareBuf);
            return -1;
        }

        if(memcmp((const void *) pu8_PageDataBuf, (const void *) pu8_BakPageDataBuf, pNandDrv->u16_PageByteCnt)!=0)
        {
            nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "2 NAND fatal error page @ %X,when compare @ %X\n",(unsigned int)u32_Row, (unsigned int)(u32_BakBlkRow + u16_i));
            dump_mem(pu8_PageDataBuf, 16);
            dump_mem(pu8_BakPageDataBuf, 16);
            kfree(pu8_BakPageDataBuf);
            kfree(pu8_BakPageSpareBuf);
            return -1;
        }

        u32_Row = (u32_BlkRow) + u16_i;
        memset(pu8_PageSpareBuf, 0xFF, pNandDrv->u16_SpareByteCnt);
        if(u8_CustMarkerIndex !=0xFF && u8_CustSpareMarker != 0xFF && u8_CustMarkerIndex < pNandDrv->u16_SpareByteCnt)
            pu8_PageSpareBuf[u8_CustMarkerIndex] = u8_CustSpareMarker;

        u32_Err = drvNAND_LFSRWritePhyPage(u32_Row, pu8_PageDataBuf, pu8_PageSpareBuf);
        if(u32_Err != UNFD_ST_SUCCESS)
        {
            //mark bad and do shifting block
            //@FIXME
        }
    }

    //erase backup block
    u32_Err = NC_EraseBlk(u32_GoodBlkIdx << pNandDrv->u8_BlkPageCntBits);
    if(u32_Err != UNFD_ST_SUCCESS)
    {
        //mark bad
        if(nand_markbad == NULL)
            drvNAND_MarkBadBlk(u32_GoodBlkIdx);
        else
            nand_markbad(u32_GoodBlkIdx);
    }

    kfree(pu8_BakPageDataBuf);
    kfree(pu8_BakPageSpareBuf);

    return 0;
}

U16 drvNAND_BBT_Rev_StartBlk(void)
{
    U16 u16_StarBlk;
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();

    /* set bbt block number to 0.8% of total blocks, or blocks * (2 / 256) */
    u16_StarBlk = pNandDrv->u16_BlkCnt-((pNandDrv->u16_BlkCnt >> 8) * 2);
	#if defined(CONFIG_MSTAR_RESERVED_END_OF_NAND) && CONFIG_MSTAR_RESERVED_END_OF_NAND
    u16_StarBlk -= (CONFIG_MSTAR_RESERVED_NAND_BYTE + (1 << (pNandDrv->u8_BlkPageCntBits + pNandDrv->u8_PageByteCntBits)) - 1) >> (pNandDrv->u8_BlkPageCntBits + pNandDrv->u8_PageByteCntBits);
	#endif
    return u16_StarBlk;
}

U16 drvNAND_GetBackupBlk(void)
{
    U16 u16_BackupPBA =0, u16_StarBlk, u16_i;
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();

    u16_StarBlk = drvNAND_BBT_Rev_StartBlk();

    for(u16_i= u16_StarBlk; u16_i< (pNandDrv->u16_BlkCnt-2); u16_i++)
    {
        if(drvNAND_IsGoodBlk(u16_i))
        {
            if(drvNAND_IsFreeBlk(u16_i))
            {
                u16_BackupPBA = u16_i;
                break;
            }
        }
    }
    if(u16_i == (pNandDrv->u16_BlkCnt-2))
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0,"No free block for bakup\n");
        return 0;
    }
    return u16_BackupPBA;
}

/*
return: 1: LP, 0: HP
PIdx: Page Idx.
PairedPageIdx: returned the PPM index for the PIdx.
*/
U32 drvNAND_IsPageLP(U16 PIdx, U16 *PairedPageIdx)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();

    for(*PairedPageIdx=0; *PairedPageIdx<pNandDrv->u16_BlkLowPCnt; (*PairedPageIdx)++)
    {
        if(PIdx == ga_tPairedPageMap[*PairedPageIdx].u16_LSB)
        {
            return 1;
        }
        else if(PIdx == ga_tPairedPageMap[*PairedPageIdx].u16_MSB)
        {
            return 0;
        }
    }
    return 0;
}

/*
 *drvNAND_WriteDummyToMSBPage, for the pages in the assigned range:
 *  - all 0xFF to MSB pages
 *  - random data to LSB pages
 * @StartPIdx : the 1st page needs dummy-write.
 * @EndPIdx : the last page needs dummy-write.
 * @u32_BlkRow : row address of the block.
 */
U32 drvNAND_WriteDummyToPages(U16 StartPIdx, U16 EndPIdx, U32 u32_BlkRow)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
	U16 j, u16_tmp;
	U32 u32_Err=UNFD_ST_SUCCESS;

    if(pNandDrv->u8_CellType == NAND_CellType_SLC || StartPIdx > EndPIdx)
		return UNFD_ST_SUCCESS;
    
    if(StartPIdx >= pNandDrv->u16_BlkPageCnt || EndPIdx >= pNandDrv->u16_BlkPageCnt)
		return UNFD_ST_ERR_INVALID_PARAM;

    memset(pNandDrv->PlatCtx_t.pu8_PageDataBuf, 0xFF, pNandDrv->u16_PageByteCnt);
    memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf, 0xFF, pNandDrv->u16_SpareByteCnt);
    //nand_debug(0,1,"StartPIdx: %Xh  EndPIdx: %Xh  \n", StartPIdx, EndPIdx);
	for(j = StartPIdx; j <= EndPIdx; j++)
	{
		//nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1, "write page :0x%X\n", u32_BlkRow + j);
		if(drvNAND_IsPageLP(j, &u16_tmp))
        {
            pNandDrv->u16_Reg50_EccCtrl &= ~BIT_NC_BYPASS_ECC;
            NC_Config();
            #if defined(FCIE_LFSR) && FCIE_LFSR
			if(pNandDrv->u8_RequireRandomizer)
            	NC_EnableLFSR(); // LP filled with random data
            #endif
            //nand_debug(0,1,"LP[%uh]: %Xh  \n", u16_tmp, j);
        }
        else
        {   //pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_BYPASS_ECC;				
            NC_Config();
            #if defined(FCIE_LFSR) && FCIE_LFSR
			if(pNandDrv->u8_RequireRandomizer)
            	NC_DisableLFSR();
            #endif
            //nand_debug(0,1,"HP[%uh]: %Xh  \n", u16_tmp, j);
        }
			
		u32_Err = NC_WritePages(u32_BlkRow + j, 
		    pNandDrv->PlatCtx_t.pu8_PageDataBuf, pNandDrv->PlatCtx_t.pu8_PageSpareBuf, 1);
		if(u32_Err != UNFD_ST_SUCCESS)
		{
			nand_debug (UNFD_DEBUG_LEVEL_ERROR,1,"NAND write to Row %Xh failed: %Xh\n",
				u32_BlkRow + j, u32_Err);
			nand_debug (UNFD_DEBUG_LEVEL_ERROR,1,"Mark current block as bad block\n");
			//mark bad
			drvNAND_MarkBadBlk(u32_BlkRow >> pNandDrv->u8_BlkPageCntBits);
			break;
		}
	}
    
    pNandDrv->u16_Reg50_EccCtrl &= ~BIT_NC_BYPASS_ECC;
    NC_Config();
    #if defined(FCIE_LFSR) && FCIE_LFSR	
    if(pNandDrv->u8_RequireRandomizer)
        NC_EnableLFSR();
    else
        NC_DisableLFSR();
    #endif
    
	return u32_Err;    
}

//  0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
const U8 u8FSTYPE[256] =
{   0,19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 1
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 2
    0, 0, 0, 0, 0, 0, 0, 0,16, 0, 0, 0, 0, 0, 0, 0, // 3
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 4
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 5
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 6
    0,18, 0, 6, 0, 8,10, 0, 0,12, 0, 0, 0, 0, 0, 0, // 7
    0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, // 8
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // A
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // B
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // C
    0,13, 0,16, 0,17, 3, 0, 0, 0,15, 0,14, 0, 0, 0, // D
    0, 0, 0, 2, 0, 2, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, // E
    20,13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // F
};

PAIRED_PAGE_MAP_t ga_tPairedPageMap[512];

#define VENDOR_FUJITSU                      0x04
#define VENDOR_RENESAS                      0x07
#define VENDOR_ST                           0x20
#define VENDOR_MICRON                       0x2C
#define VENDOR_NATIONAL                     0x8F
#define VENDOR_TOSHIBA                      0x98
#define VENDOR_HYNIX                        0xAD
#define VENDOR_SAMSUNG                      0xEC
#define VENDOR_MXIC                         0xC2
#define VENDOR_ZENTEL                       0x92
#define VENDOR_ZENTEL1                      0xC8
#define VENDOR_SPANSION                     0x01
#define VENDOR_WINBOND                      0xEF
#define VENDOR_PARAGON			  			0x4A


void drvNAND_CHECK_FLASH_TYPE(void)
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U16 u16_i,u16_j=0;
    U8 u8_more_maker=0;

    if(pNandDrv->au8_ID[0] ==0x7F)
    {
        u8_more_maker=1;
    }

    if(u8_more_maker)
    {
        for (u16_i=1; u16_i<NAND_ID_BYTE_CNT;u16_i++)
        {
            if (pNandDrv->au8_ID[u16_i] == 0x7F)
            {
                u16_j = u16_i;
            }
            else
            {
                pNandDrv->au8_ID[u16_i-u16_j-1] = pNandDrv->au8_ID[u16_i];
            }
        }
        pNandDrv->u8_IDByteCnt -= (u16_j+1);
    }

    printk("NAND ID:");
    for(u16_i = 0; u16_i < pNandDrv->u8_IDByteCnt; u16_i++)
      printk("0x%X ", pNandDrv->au8_ID[u16_i]);
    printk("\n");

    if( (pNandDrv->au8_ID[0] != VENDOR_SAMSUNG) &&
        (pNandDrv->au8_ID[0] != VENDOR_TOSHIBA) &&
        (pNandDrv->au8_ID[0] != VENDOR_RENESAS) &&
        (pNandDrv->au8_ID[0] != VENDOR_HYNIX)  &&
        (pNandDrv->au8_ID[0] != VENDOR_FUJITSU) &&
        (pNandDrv->au8_ID[0] != VENDOR_MICRON) &&
        (pNandDrv->au8_ID[0] != VENDOR_NATIONAL)  &&
        (pNandDrv->au8_ID[0] != VENDOR_ST) &&
        (pNandDrv->au8_ID[0] != VENDOR_MXIC) &&
        (pNandDrv->au8_ID[0] != VENDOR_ZENTEL) &&
        (pNandDrv->au8_ID[0] != VENDOR_ZENTEL1) &&
        (pNandDrv->au8_ID[0] != VENDOR_SPANSION) &&
        (pNandDrv->au8_ID[0] != VENDOR_WINBOND) &&
	    (pNandDrv->au8_ID[0] != VENDOR_PARAGON))        

    {
        pNandDrv->u16_BlkCnt = 0;
        pNandDrv->u16_BlkPageCnt = 0;
        pNandDrv->u16_PageByteCnt = 0;
        pNandDrv->u16_SectorByteCnt = 0;
        pNandDrv->u16_SpareByteCnt = 0;

        printk("Unsupport Vendor %02X\n", pNandDrv->au8_ID[0]);

        return; // unsupported flash maker
    }

    pNandDrv->u8_IsMLC = 0;
    pNandDrv->u16_ECCType = ECC_TYPE_4BIT;
    pNandDrv->u8_PlaneCnt = 1;
    pNandDrv->u8_CacheProgram = 0;
    pNandDrv->u8_CacheRead = 0;
    /*Default NAND Timing Setting*/
    pNandDrv->u16_tRC = 25;
    pNandDrv->u8_tRP = 12;
    pNandDrv->u8_tREH = 10;
    pNandDrv->u8_tREA = 20;
    pNandDrv->u8_tRR = 20;
    pNandDrv->u16_tADL = 70;
    pNandDrv->u16_tRHW = 100;
    pNandDrv->u16_tWHR = 60;
    pNandDrv->u16_tCCS = 60;
    pNandDrv->u8_tCS = 20;
    pNandDrv->u16_tWC = 25;
    pNandDrv->u8_tWP = 12;
    pNandDrv->u8_tWH = 10;
    pNandDrv->u16_tCWAW = 0;
    pNandDrv->u8_tCLHZ = 0;
    pNandDrv->u16_tWW = 100;
    switch(u8FSTYPE[pNandDrv->au8_ID[1]])
    {
        case 0:
            pNandDrv->u16_BlkCnt = 0;
            pNandDrv->u16_BlkPageCnt = 0;
            pNandDrv->u16_PageByteCnt = 0;
            pNandDrv->u16_SectorByteCnt = 0;
            pNandDrv->u16_SpareByteCnt = 0;
            break;
        case 2:
            pNandDrv->u16_BlkCnt = 512;
            pNandDrv->u16_BlkPageCnt = 16;
            pNandDrv->u16_PageByteCnt = 512;
            pNandDrv->u16_SectorByteCnt = 512;
            pNandDrv->u16_SpareByteCnt = 16;
            break;
        case 4:
            pNandDrv->u16_BlkCnt = 1024;
            pNandDrv->u16_BlkPageCnt = 16;
            pNandDrv->u16_PageByteCnt = 512;
            pNandDrv->u16_SectorByteCnt = 512;
            pNandDrv->u16_SpareByteCnt = 16;
            pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C3TFS0;
            pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C2TRS0;
            break;
        case 6:
            pNandDrv->u16_BlkCnt = 1024;
            pNandDrv->u16_BlkPageCnt = 32;
            pNandDrv->u16_PageByteCnt = 512;
            pNandDrv->u16_SectorByteCnt = 512;
            pNandDrv->u16_SpareByteCnt = 16;
            pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C3TFS0;
            pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C2TRS0;
            break;
        case 7:
            //_fsinfo.eFlashConfig |= FLASH_WP;
        case 8:
            pNandDrv->u16_BlkCnt = 2048;
            pNandDrv->u16_BlkPageCnt = 32;
            pNandDrv->u16_PageByteCnt = 512;
            pNandDrv->u16_SectorByteCnt = 512;
            pNandDrv->u16_SpareByteCnt = 16;
            pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C3TFS0;
            pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C2TRS0;
            break;
        case 10:
            pNandDrv->u16_BlkCnt = 4096;
            pNandDrv->u16_BlkPageCnt = 32;
            pNandDrv->u16_PageByteCnt = 512;
            pNandDrv->u16_SectorByteCnt = 512;
            pNandDrv->u16_SpareByteCnt = 16;
            pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C4TFS0;
            pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C3TRS0;
            break;
        case 12:
            pNandDrv->u16_BlkCnt = 8192;
            pNandDrv->u16_BlkPageCnt = 32;
            pNandDrv->u16_PageByteCnt = 512;
            pNandDrv->u16_SectorByteCnt = 512;
            pNandDrv->u16_SpareByteCnt = 16;
            pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C4TFS0;
            pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C3TRS0;
            break;
        case 13:
            if(pNandDrv->au8_ID[0]==VENDOR_TOSHIBA)
            {
                pNandDrv->u16_BlkCnt = 1024;
                pNandDrv->u16_BlkPageCnt = 64;
                pNandDrv->u16_PageByteCnt = 2048;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = 64;
                pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C4TFS0;
                pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C2TRS0;
                if((pNandDrv->au8_ID[2] == 0x80) && (pNandDrv->au8_ID[3] == 0x15) &&
                (pNandDrv->au8_ID[4] == 0x72) && (pNandDrv->au8_ID[5] == 0x16))
                {
                    pNandDrv->u16_SpareByteCnt = 128;
                    pNandDrv->u16_ECCType = ECC_TYPE_8BIT;
                }
                if(((pNandDrv->au8_ID[2] & 0x0F) == 0) &&
                    ((pNandDrv->au8_ID[3] & 0x33) == 0x11) &&
                    ((pNandDrv->au8_ID[4] & 0x0C)== 0))
                {
                    pNandDrv->u16_SpareByteCnt = 128;
                    pNandDrv->u16_ECCType = ECC_TYPE_8BIT;
                }
                if((pNandDrv->au8_ID[2] == 0x80) && (pNandDrv->au8_ID[3] == 0x15) &&
                (pNandDrv->au8_ID[4] == 0xF2) && (pNandDrv->au8_ID[5] == 0x16))
                {
                    pNandDrv->u16_SpareByteCnt = 64;
                    pNandDrv->u16_ECCType = ECC_TYPE_8BIT;
                }
            }
            else if(pNandDrv->au8_ID[0]==VENDOR_WINBOND)
            {
                pNandDrv->u16_BlkCnt = 1024;
                pNandDrv->u16_BlkPageCnt = 64;
                pNandDrv->u16_PageByteCnt = 2048;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = 64;
                pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C4TFS0;
                pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C2TRS0;
            }			
            else
            {
                pNandDrv->u16_BlkCnt = 1024;
                pNandDrv->u16_BlkPageCnt = 64;
                pNandDrv->u16_PageByteCnt = 2048;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = 64;
                pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C4TFS0;
                pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C2TRS0;
            }
            break;
        case 14:
            if((pNandDrv->au8_ID[0]==VENDOR_SPANSION))      //spansion 4Gbit
            {
                pNandDrv->u16_BlkCnt = 4096;
                pNandDrv->u16_BlkPageCnt = 64;
                pNandDrv->u16_PageByteCnt = 2048;
                pNandDrv->u16_SectorByteCnt = 512;
                if(pNandDrv->au8_ID[2] == 0x90 &&
                   pNandDrv->au8_ID[3] == 0x95 &&
                   pNandDrv->au8_ID[4] == 0x56)
                {
                    pNandDrv->u16_SpareByteCnt = 128;
                }
                else
                {
                    pNandDrv->u16_SpareByteCnt = 64;
                }
            }
            else if((pNandDrv->au8_ID[0]==VENDOR_MXIC)&&
                    (pNandDrv->au8_ID[2] == 0x90) &&
                    (pNandDrv->au8_ID[3] == 0x95) &&
                    (pNandDrv->au8_ID[4] == 0x57))
            {
                pNandDrv->u16_BlkCnt = 4096;
                pNandDrv->u16_BlkPageCnt = 64;
                pNandDrv->u16_PageByteCnt = 2048;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = 112;
                pNandDrv->u16_ECCType = ECC_TYPE_8BIT;
            }
            else if((pNandDrv->au8_ID[0]==VENDOR_MICRON)&&
                    (pNandDrv->au8_ID[2] == 0x90) && 
                    (pNandDrv->au8_ID[3] == 0xA6) && 
                    (pNandDrv->au8_ID[4] == 0x54))
            {
                pNandDrv->u16_BlkCnt = 2048;
                pNandDrv->u16_BlkPageCnt = 64;
                pNandDrv->u16_PageByteCnt = 4096;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = 224;
                pNandDrv->u16_ECCType = ECC_TYPE_8BIT;
            }
            //TC58NVG2S0FTA00
            else if((pNandDrv->au8_ID[0]==VENDOR_TOSHIBA)&&
                    (pNandDrv->au8_ID[2] == 0x90) && 
                    (pNandDrv->au8_ID[3] == 0x26) && 
                    (pNandDrv->au8_ID[4] == 0x76) &&
                    (pNandDrv->au8_ID[5] == 0x15))
            {
                pNandDrv->u16_BlkCnt = 2048;
                pNandDrv->u16_BlkPageCnt = 64;
                pNandDrv->u16_PageByteCnt = 4096;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = 224;
                pNandDrv->u16_ECCType = ECC_TYPE_8BIT;
            }          
            //H27U4G8F2ETR, H27U4G8F2FTR
            else if((pNandDrv->au8_ID[0]==VENDOR_HYNIX)&&
                    (pNandDrv->au8_ID[2] == 0x90) && 
                    (pNandDrv->au8_ID[3] == 0x95) && 
                    (pNandDrv->au8_ID[4] == 0x56))
            {
                pNandDrv->u16_BlkCnt = 4096;
                pNandDrv->u16_BlkPageCnt = 64;
                pNandDrv->u16_PageByteCnt = 2048;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = 128;
                pNandDrv->u16_tWHR = 200;
            }
            else
            {
                pNandDrv->u16_PageByteCnt = 1024 << (pNandDrv->au8_ID[3] & 3);
                pNandDrv->u16_BlkPageCnt  = ((64 * 1024)  << ((pNandDrv->au8_ID[3] >> 4) & 3)) /  pNandDrv->u16_PageByteCnt;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = (8 << (( pNandDrv->au8_ID[3] >> 2)& 0x01)) * ( pNandDrv->u16_PageByteCnt>>9);
                pNandDrv->u16_BlkCnt = 4096 / (pNandDrv->u16_PageByteCnt/2048);
            }
            pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C5TFS0;
            pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C3TRS0;
            break;
        case 15:
            //spansion 2Gbit
            pNandDrv->u16_BlkCnt = 2048;
            pNandDrv->u16_BlkPageCnt = 64;
            pNandDrv->u16_PageByteCnt = 2048;
            pNandDrv->u16_SectorByteCnt = 512;
            pNandDrv->u16_SpareByteCnt = 64;
            pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C5TFS0;
            pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C3TRS0;
            if(((pNandDrv->au8_ID[0]==VENDOR_SPANSION)||(pNandDrv->au8_ID[0]==VENDOR_HYNIX))&&
               (pNandDrv->au8_ID[2] == 0x90) &&
               (pNandDrv->au8_ID[3] == 0x95) &&
               (pNandDrv->au8_ID[4] == 0x46))

            {
                pNandDrv->u16_SpareByteCnt = 128;
            }

            //Toshiba 2G SLC 24nm
            else if(pNandDrv->au8_ID[0]==VENDOR_TOSHIBA &&
                pNandDrv->au8_ID[2] == 0x90 &&
                pNandDrv->au8_ID[3] == 0x15 &&
                pNandDrv->au8_ID[4] == 0x76 &&
                pNandDrv->au8_ID[5] == 0x16)
            {
                pNandDrv->u16_SpareByteCnt = 128;
                pNandDrv->u16_ECCType = ECC_TYPE_8BIT;
            }
            //MXIC MX30LFxG28AB 2G SLC 2Gbit
            else if(pNandDrv->au8_ID[0]==VENDOR_MXIC &&
                  pNandDrv->au8_ID[2] == 0x90 &&
                  pNandDrv->au8_ID[3] == 0x95 &&
                  pNandDrv->au8_ID[4] == 0x07)
            {
                pNandDrv->u16_SpareByteCnt = 112;
                pNandDrv->u16_ECCType = ECC_TYPE_12BIT;
            }
            //MICRON MT29F2G08ABAFA 2G SLC 2Gbit
            else if(pNandDrv->au8_ID[0]==VENDOR_MICRON &&
                  pNandDrv->au8_ID[2] == 0x90 && 
                  pNandDrv->au8_ID[3] == 0x95 && 
                  pNandDrv->au8_ID[4] == 0x04)
            {
                pNandDrv->u16_SpareByteCnt = 224;
                pNandDrv->u16_ECCType = ECC_TYPE_8BIT;
            }
			//WINBOND W29N02GV SLC 2Gbit
			else if(pNandDrv->au8_ID[0]==VENDOR_WINBOND &&
			      pNandDrv->au8_ID[2] == 0x90 && 
			      pNandDrv->au8_ID[3] == 0x95 && 
			      pNandDrv->au8_ID[4] == 0x04)
			{
				pNandDrv->u16_ECCType = ECC_TYPE_8BIT; //WINBOND recommend
			}			
            else
            {
                pNandDrv->u16_SpareByteCnt = 64;
                pNandDrv->u16_ECCType = ECC_TYPE_4BIT;
            }
            break;
        case 16:

            if((pNandDrv->au8_ID[0]==VENDOR_SAMSUNG))
            {
                pNandDrv->u16_BlkCnt = 4096;
                pNandDrv->u16_BlkPageCnt = 64;
                pNandDrv->u16_PageByteCnt = 4096;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = 128;
            }
            else if((pNandDrv->au8_ID[0]==VENDOR_HYNIX))
            {
                pNandDrv->u16_BlkCnt = 4096;
                pNandDrv->u16_BlkPageCnt = 128;
                pNandDrv->u16_PageByteCnt = 2048;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = 64;
            }
            else if((pNandDrv->au8_ID[0]==VENDOR_MICRON))
            {
                if(pNandDrv->au8_ID[2] == 0x90 && 
                   pNandDrv->au8_ID[3] == 0xA6 && 
                   pNandDrv->au8_ID[4] == 0x64)
                {                       
                    pNandDrv->u16_BlkCnt = 4096;
                    pNandDrv->u16_BlkPageCnt = 64;
                    pNandDrv->u16_PageByteCnt = 4096;
                    pNandDrv->u16_SectorByteCnt = 512;
                    pNandDrv->u16_SpareByteCnt = 224;
                    pNandDrv->u16_ECCType = ECC_TYPE_8BIT;            
                }
                else if(pNandDrv->au8_ID[1] == 0x38 && 
                        pNandDrv->au8_ID[2] == 0x00 && 
                        pNandDrv->au8_ID[3] == 0x26 && 
                        pNandDrv->au8_ID[4] == 0x85)
                {                       
                    pNandDrv->u16_BlkCnt = 2048;
                    pNandDrv->u16_BlkPageCnt = 128;
                    pNandDrv->u16_PageByteCnt = 4096;
                    pNandDrv->u16_SectorByteCnt = 512;
                    pNandDrv->u16_SpareByteCnt = 224;
                    pNandDrv->u16_ECCType = ECC_TYPE_8BIT;            
                }
                else
                {
                    pNandDrv->u16_BlkCnt = 2048;
                    pNandDrv->u16_BlkPageCnt = 128;
                    pNandDrv->u16_PageByteCnt = 4096;
                    pNandDrv->u16_SectorByteCnt = 512;
                    pNandDrv->u16_SpareByteCnt = 128;
                }
            }
            else if((pNandDrv->au8_ID[0] == VENDOR_TOSHIBA))
            {
                pNandDrv->u16_BlkCnt  = 4096;
                pNandDrv->u16_BlkPageCnt = 64;
                pNandDrv->u16_PageByteCnt = 4096;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = 128;
            }
            else if(pNandDrv->au8_ID[0] != VENDOR_ST)
            {
                pNandDrv->u16_BlkCnt = 2048;
                pNandDrv->u16_BlkPageCnt = 64;
                pNandDrv->u16_PageByteCnt = 2048;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = 64;
            }
            else
            {
                pNandDrv->u16_BlkCnt = 8192;
                pNandDrv->u16_BlkPageCnt = 64;
                pNandDrv->u16_PageByteCnt = 2048;
                pNandDrv->u16_SectorByteCnt = 512;
                pNandDrv->u16_SpareByteCnt = 64;
            }
            pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C5TFS0;
            pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C3TRS0;
            break;
        case 17:
            if ((pNandDrv->au8_ID[2] & 0xC) == 0)  // for SLC
            {
                if(pNandDrv->au8_ID[0] != VENDOR_SAMSUNG)
                {
                    pNandDrv->u16_BlkCnt = 8192;
                    pNandDrv->u16_BlkPageCnt = 128;
                    pNandDrv->u16_PageByteCnt = 2048;
                    pNandDrv->u16_SectorByteCnt = 512;
                    pNandDrv->u16_SpareByteCnt = 64;
                }
                else
                {
                    pNandDrv->u16_BlkCnt = 16384;
                    pNandDrv->u16_BlkPageCnt = 64;
                    pNandDrv->u16_PageByteCnt = 2048;
                    pNandDrv->u16_SectorByteCnt = 512;
                    pNandDrv->u16_SpareByteCnt = 64;
                }
            }
            else  // for MLC
            {
                U8 u8PageSize, u8Value;
                pNandDrv->u8_IsMLC = 1;
                pNandDrv->u16_PageByteCnt = 2048 << (pNandDrv->au8_ID[3] & 0x3);
                u8PageSize = pNandDrv->u16_PageByteCnt >> 10;
                u8Value = ((pNandDrv->au8_ID[3] >> 4) & 0x3) | ((pNandDrv->au8_ID[3] >> 5) & 0x4);

                if (pNandDrv->au8_ID[0] == VENDOR_SAMSUNG)
                {
                    pNandDrv->u16_BlkCnt = 2076;
                    pNandDrv->u16_BlkPageCnt = (128 << u8Value) / u8PageSize;
                }
                else if (pNandDrv->au8_ID[0] == VENDOR_HYNIX)
                {
                    pNandDrv->u16_BlkCnt = 1024;

                    if (u8Value == 0x0)
                    {
                        pNandDrv->u16_BlkPageCnt = 128 / u8PageSize;
                    }
                    else if (u8Value == 0x1)
                    {
                        pNandDrv->u16_BlkPageCnt = 256 / u8PageSize;
                    }
                    else if (u8Value == 0x2)
                    {
                        pNandDrv->u16_BlkPageCnt = 512 / u8PageSize;
                    }
                    else if (u8Value == 0x3) // 768 is not power of 2, should fix according to specific chip
                    {
                        pNandDrv->u16_BlkPageCnt = 768 / u8PageSize;
                    }
                    else if (u8Value == 0x4)
                    {
                        pNandDrv->u16_BlkPageCnt = 1024 / u8PageSize;
                    }
                    else if (u8Value == 0x5)
                    {
                        pNandDrv->u16_BlkPageCnt = 2048 / u8PageSize;
                    }
                }
                pNandDrv->u16_SectorByteCnt = 1024;
                pNandDrv->u16_SpareByteCnt = 432;
                pNandDrv->u16_ECCType = ECC_TYPE_24BIT1KB;
            }
            pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C5TFS0;
            pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C3TRS0;
            break;
        case 18:
            pNandDrv->u16_BlkCnt = 16384;
            pNandDrv->u16_BlkPageCnt = 32;
            pNandDrv->u16_PageByteCnt = 512;
            pNandDrv->u16_SectorByteCnt = 512;
            pNandDrv->u16_SpareByteCnt = 16;
            pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C4TFS0;
            pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C3TRS0;
            break;
        case 20:
            pNandDrv->u16_BlkCnt = 1024;
            pNandDrv->u16_BlkPageCnt = 64;
            pNandDrv->u16_PageByteCnt = 2048;
            pNandDrv->u16_SectorByteCnt = 512;
            pNandDrv->u16_SpareByteCnt = 64;
            pNandDrv->u8_OpCode_RW_AdrCycle = ADR_C4TFS0;
            pNandDrv->u8_OpCode_Erase_AdrCycle = ADR_C2TRS0;
            break;
        default:
            pNandDrv->u16_BlkCnt = 0;
            pNandDrv->u16_BlkPageCnt = 0;
            pNandDrv->u16_PageByteCnt = 0;
            pNandDrv->u16_SectorByteCnt = 0;
            pNandDrv->u16_SpareByteCnt = 0;
            break;
    }
}

#if defined(FCIE_REG_TEE_BASE_ADDR)
void NC_Get_REE_Grant(void)
{
    REG_SET_BITS_UINT16(NC_BOOT_MODE, BIT_REE_REQ);
    while(1)
    {
        if((REG(NC_BOOT_MODE) & BIT_GRANT2REE) == BIT_GRANT2REE)
        {
            //nand_debug(0, 1, "Grant to REE\n");
            break;
        }
    }
}

void NC_Release_REE_Grant(void)
{
    REG_CLR_BITS_UINT16(NC_BOOT_MODE, BIT_REE_REQ);
}
#else
void NC_Get_REE_Grant(void)
{
}

void NC_Release_REE_Grant(void)
{
}
#endif

EXPORT_SYMBOL(NC_Get_REE_Grant);
EXPORT_SYMBOL(NC_Release_REE_Grant);
