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


//=======================================================================
//  MStar Semiconductor - Unified Nand Flash Driver
//
//  drvNAND_hal.c - Storage Team, 2009/08/20
//
//  Design Notes:
//    2009/08/21 - FCIE3 physical layer functions
//
//=======================================================================
#include "drvNAND.h"
#include "drvNAND_utl.h"


//========================================================
// HAL pre-processors
//========================================================
#if IF_FCIE_SHARE_PINS
	#define NC_PAD_SWITCH(enable)    //nand_pads_switch(enable);
#else
    // NULL to save CPU a JMP/RET time
	#define NC_PAD_SWITCH(enable)
#endif

#if IF_FCIE_SHARE_CLK
	#define NC_CLK_SETTING(ClkParam) //nand_clock_setting(ClkParam);
#else
    // NULL to save CPU a JMP/RET time
	#define NC_CLK_SETTING(ClkParam)
#endif

#if IF_FCIE_SHARE_IP
	#define NC_LOCK_FCIE()
	#define NC_UNLOCK_FCIE()
	// re-config FCIE3 for NFIE mode
	#define NC_RECONFIG()       //NC_ReConfig();
#else
    // NULL to save CPU a JMP/RET time
    #define NC_LOCK_FCIE()
	#define NC_UNLOCK_FCIE()
	#define NC_RECONFIG()
#endif

#define NAND_TIMEOUT_RETRY_CNT     5

// indicate read ecc status
int gint_IS_ECC_Fail=0;

#if defined(NEED_FLUSH_MIU_PIPE) && NEED_FLUSH_MIU_PIPE
#define FLUSH_MIU_PIPE()		nand_flush_miu_pipe() // Only used in U4 now
#else
#define FLUSH_MIU_PIPE()
#endif
//========================================================
// HAL misc. function definitions
//========================================================
#if (defined(NC_SEL_FCIE3) && (NC_SEL_FCIE3))
void NC_DumpRegisters(void);
void NC_DumpDebugBus(void);
U32  NC_CheckEWStatus(U8 u8_OpType);
U32  NC_WaitComplete(U16 u16_WaitEvent, U32 u32_MicroSec);
U32  NC_WaitComplete_RM(U16 u16WaitEvent, U32 u32MicroSec);

	#define OPTYPE_ERASE        1
	#define OPTYPE_WRITE        2

// ============================================================
#if defined(FCIE4_DDR) && FCIE4_DDR

#define NAND_CMD_READ_ID                0x90
#define NAND_CMD_SET_FEATURE            0xEF
#define NAND_CMD_GET_FEATURE            0xEE
#define NAND_CMD_READ_PARAMETER         0xEC

#define NAND_ADR_READ_JEDEC             0x40
#define NAND_ADR_READ_ONFI              0x20
#define NAND_ADR_READ_PARAMETER         0x00
#define NAND_ADR_TIMING_MODE            0x01
#define NAND_ADR_INTERFACE_MODE         0x80

#define NAND_DDR_REMOVE_2NDBYTE_NA      0
#define NAND_DDR_REMOVE_2NDBYTE_TOGGLE  1
#define NAND_DDR_REMOVE_2NDBYTE_ONFI    2

static U32 NC_RandomIn_1Cmd1Addr(U8 u8_Cmd, U8 u8_Addr, U8 u8_Remove2ndByte, U8 u8_ByteCnt, U8 *pu8_DataBuf);
U32 NC_SetToggleDDRMode(void);
U32 NC_ReadToggleParamPage(void);
#endif

U32 NC_SendReadRetryCmd(U8 u8_RetryIndex, U8 u8_SetToDefault);

#if CONFIG_MSTAR_FTL // read-disturbance
extern void NFTL_RD_Set(U16 PBA, U16 PIdx);
#endif
// ============================================================

static U8 gu8_DisableErrorLog = 0;

#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
extern uint64_t	u64_TotalWriteBytes_Hal;
extern uint64_t	u64_TotalReadBytes_Hal;
#endif

// for /sys files
extern U32 gu32_monitor_read_enable;
extern U32 gu32_monitor_write_enable;
extern U32 gu32_monitor_count_enable;
extern U32 gu32_monitor_read_count;
extern U32 gu32_monitor_write_count;
extern uint64_t gu64_monitor_read_size_kb;
extern uint64_t gu64_monitor_write_size_kb;
extern uint64_t gu64_monitor_read_size;
extern uint64_t gu64_monitor_write_size;

extern  U8 gau8_ZeroBitCnt[256];
#define WAIT_FIFOCLK_READY_CNT  0x10000
U32 NC_waitFifoClkReady(void)
{
	volatile U32 u32_i;
	volatile U16 u16_Reg;

	for(u32_i=0; u32_i<WAIT_FIFOCLK_READY_CNT; u32_i++)
	{
		REG_READ_UINT16(NC_MMA_PRI_REG, u16_Reg);
		if(u16_Reg & BIT_NC_FIFO_CLKRDY)
			break;
		nand_hw_timer_delay(HW_TIMER_DELAY_1us);
	}

	if(WAIT_FIFOCLK_READY_CNT == u32_i)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: FIFO CLK not ready \n");
		NC_ResetFCIE();

		return UNFD_ST_ERR_R_TIMEOUT;
	}

	return UNFD_ST_SUCCESS;

}

#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
U32 NC_wait_MIULastDone(void)
{
	volatile U32 u32_retry=0;
	volatile U16 u16_Reg;

	while(u32_retry < DELAY_1s_in_us)
	{
		//REG_READ_UINT16(NC_PATH_CTL, u16_Reg);
		REG_READ_UINT16(NC_REG_MIU_LAST_DONE, u16_Reg);
		if((u16_Reg & BIT_MIU_LAST_DONE) == BIT_MIU_LAST_DONE)
		{
			REG_W1C_BITS_UINT16(NC_REG_MIU_LAST_DONE,BIT_MIU_LAST_DONE); //Must  Clear LAST DONE @@
			break;
		}

		u32_retry++;
		nand_hw_timer_delay(1);
	}

	if(u32_retry == DELAY_1s_in_us)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: Wait BIT_MIU_LAST_DONE timeout\r\n");
		NC_DumpDebugBus();
		NC_DumpRegisters();
	    #ifdef __NAND_DEBUG_CODE__
		nand_die();
	    #else
		NC_ResetFCIE();
		NC_Config();
		return UNFD_ST_ERR_R_TIMEOUT;
	#endif
	}

	return UNFD_ST_SUCCESS;
}
#endif

static U16 au16_NandDebugBusReg[100];
void NC_DumpDebugBus(void)
{
	volatile U16 u16_Reg, u16_i;
	U16 u16_idx = 0;

	memset(au16_NandDebugBusReg, 0xFF, sizeof(au16_NandDebugBusReg));

	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\n");
	for(u16_i=0; u16_i<8; u16_i++)
	{
		REG_CLR_BITS_UINT16(NC_TEST_MODE, BIT_FCIE_DEBUG_MODE);
		REG_SET_BITS_UINT16(NC_TEST_MODE, u16_i<<BIT_FCIE_DEBUG_MODE_SHIFT);

		REG_READ_UINT16(NC_TEST_MODE, u16_Reg);
		au16_NandDebugBusReg[u16_idx] = u16_Reg;
		u16_idx++;
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "NC_TEST_MODE: %X\n", u16_Reg);

		REG_READ_UINT16(NC_DEBUG_DBUS0, u16_Reg);
		au16_NandDebugBusReg[u16_idx] = u16_Reg;
		u16_idx++;
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "NC_DEBUG_DBUS0: %X\n", u16_Reg);

		REG_READ_UINT16(NC_DEBUG_DBUS1, u16_Reg);
		au16_NandDebugBusReg[u16_idx] = u16_Reg;
		u16_idx++;
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "NC_DEBUG_DBUS1: %X\n", u16_Reg);
	}

	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "\n");
	for(u16_i=0; u16_i<8; u16_i++)
	{
		REG_CLR_BITS_UINT16(NC_WIDTH, BIT_NC_DEB_SEL);
		REG_SET_BITS_UINT16(NC_WIDTH, u16_i<<BIT_NC_DEB_SEL_SHIFT);

		REG_READ_UINT16(NC_TEST_MODE, u16_Reg);
		au16_NandDebugBusReg[u16_idx] = u16_Reg;
		u16_idx++;
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "NC_TEST_MODE: %X\n", u16_Reg);

        REG_READ_UINT16(NC_WIDTH, u16_Reg);
		au16_NandDebugBusReg[u16_idx] = u16_Reg;
		u16_idx++;
	    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "NC_WIDTH: %X\n", u16_Reg);

		REG_READ_UINT16(NC_DEBUG_DBUS0, u16_Reg);
		au16_NandDebugBusReg[u16_idx] = u16_Reg;
		u16_idx++;
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "NC_DEBUG_DBUS0: %X\n", u16_Reg);

		REG_READ_UINT16(NC_DEBUG_DBUS1, u16_Reg);
		au16_NandDebugBusReg[u16_idx] = u16_Reg;
		u16_idx++;
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "NC_DEBUG_DBUS1: %X\n", u16_Reg);
	}

}

void NC_DumpNandStr(void)
{
    #if defined(NAND_DEBUG_MSG) && NAND_DEBUG_MSG
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    #endif

	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NAND_Info:\n");
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  BlkCnt                 : 0x%X\n", pNandDrv->u16_BlkCnt);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  BlkPageCnt             : 0x%X\n", pNandDrv->u16_BlkPageCnt);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  BlkSectorCnt           : 0x%X\n", pNandDrv->u16_BlkSectorCnt);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  PageByteCnt            : 0x%X\n", pNandDrv->u16_PageByteCnt);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  SpareByteCnt           : 0x%X\n\n", pNandDrv->u16_SpareByteCnt);

	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  BlkPageCntBits         : 0x%X\n", pNandDrv->u8_BlkPageCntBits);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  BlkSectorCntBits       : 0x%X\n", pNandDrv->u8_BlkSectorCntBits);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  PageByteCntBits        : 0x%X\n", pNandDrv->u8_PageByteCntBits);

	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  BlkPageCntMask         : 0x%X\n", pNandDrv->u16_BlkPageCntMask);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  BlkSectorCntMask       : 0x%X\n", pNandDrv->u16_BlkSectorCntMask);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  PageByteCntMask        : 0x%X\n", pNandDrv->u16_PageByteCntMask);

	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  PageSectorCnt          : 0x%X\n", pNandDrv->u16_PageSectorCnt);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  SectorByteCnt          : 0x%X\n", pNandDrv->u16_SectorByteCnt);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  SectorSpareByteCnt     : 0x%X\n", pNandDrv->u16_SectorSpareByteCnt);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  ECCCodeByteCnt         : 0x%X\n\n", pNandDrv->u16_ECCCodeByteCnt);

	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  PageSectorCntBits      : 0x%X\n", pNandDrv->u8_PageSectorCntBits);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  SectorByteCntBits      : 0x%X\n", pNandDrv->u8_SectorByteCntBits);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  PageSectorCntMask      : 0x%X\n", pNandDrv->u16_PageSectorCntMask);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  SectorByteCntMask      : 0x%X\n\n", pNandDrv->u16_SectorByteCntMask);

	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u8_OpCode_Erase_AdrCycle      : 0x%X\n", pNandDrv->u8_OpCode_Erase_AdrCycle);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u8_OpCode_RW_AdrCycle      : 0x%X\n\n", pNandDrv->u8_OpCode_RW_AdrCycle);

	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u16_tRC;      : 0x%X\n", pNandDrv->u16_tRC);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u8_tRP      : 0x%X\n", pNandDrv->u8_tRP);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u8_tREH      : 0x%X\n", pNandDrv->u8_tREH);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u8_tREA      : 0x%X\n", pNandDrv->u8_tREA);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u8_tRR      : 0x%X\n", pNandDrv->u8_tRR);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u16_tADL      : 0x%X\n", pNandDrv->u16_tADL);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u16_tRHW      : 0x%X\n", pNandDrv->u16_tRHW);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u16_tWHR      : 0x%X\n", pNandDrv->u16_tWHR);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u16_tCCS      : 0x%X\n", pNandDrv->u16_tCCS);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u8_tCS      : 0x%X\n", pNandDrv->u8_tCS);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u16_tWC      : 0x%X\n", pNandDrv->u16_tWC);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u8_tWP      : 0x%X\n", pNandDrv->u8_tWP);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u8_tWH      : 0x%X\n", pNandDrv->u8_tWH);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u16_tCWAW      : 0x%X\n", pNandDrv->u16_tCWAW);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u8_tCLHZ      : 0x%X\n", pNandDrv->u8_tCLHZ);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u16_tWW      : 0x%X\n", pNandDrv->u16_tWW);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "  u8_AddrCycleIdx      : 0x%X\n\n", pNandDrv->u8_AddrCycleIdx);

}
static U16 au16_NandDebugReg[100];
void NC_DumpRegisters(void)
{
	volatile U16 u16_Reg;
	U16 u16_idx = 0;

	nand_DumpPadClk();
	NC_DumpNandStr();
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Dump FCIE Register\n");
	REG_SET_BITS_UINT16(NC_JOB_BL_CNT, BIT15);
	REG_SET_BITS_UINT16(NC_JOB_BL_CNT, BIT14);

	REG_READ_UINT16(NC_TR_BK_CNT, u16_Reg);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "MIU TR_BL_CNT(0x0C) = %04X\n", u16_Reg);

	REG_CLR_BITS_UINT16(NC_JOB_BL_CNT, BIT14);

	REG_READ_UINT16(NC_TR_BK_CNT, u16_Reg);
	nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "FCIE TR_BL_CNT(0x0C) = %04X\n", u16_Reg);

	REG_CLR_BITS_UINT16(NC_JOB_BL_CNT, BIT15);

	memset(au16_NandDebugReg, 0xFF, sizeof(au16_NandDebugReg));
	for(u16_idx=0 ; u16_idx<0x5E ; u16_idx++)
	{
		au16_NandDebugReg[u16_idx] = REG(GET_REG_ADDR(FCIE_REG_BASE_ADDR, u16_idx));
		nand_debug(0, 0, "0x%04X  ", au16_NandDebugReg[u16_idx]);
		if((u16_idx & 0x7) == 0x7)
			nand_debug(0, 0,"\n");
	}
	nand_debug(0, 0,"\n");

}

//========================================================
// HAL function definitions
//========================================================
U32 NC_ResetFCIE(void)
{
	//volatile U16 u16_i=0;
	volatile U16 u16Reg, u16Cnt, u16Reg41h;

	NC_PlatformResetPre();

	REG_WRITE_UINT16(NC_SIGNAL, ~DEF_REG0x40_VAL);

	// soft reset
	REG_CLR_BITS_UINT16(NC_TEST_MODE, BIT_FCIE_SOFT_RST); /* active low */
	nand_hw_timer_delay(HW_TIMER_DELAY_1us);
	REG_SET_BITS_UINT16(NC_TEST_MODE, BIT_FCIE_SOFT_RST);
	// make sure reset complete
	u16Cnt=0;
	do
	{
		nand_hw_timer_delay(HW_TIMER_DELAY_1us);
	   	if(0x1000 == u16Cnt++)
		{
			nand_DumpPadClk();
		    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_ResetFCIE, ErrCode: 0x%03X\n", UNFD_ST_ERR_NO_NFIE);
		    return UNFD_ST_ERR_NO_NFIE;
		}

		REG_READ_UINT16(NC_SIGNAL, u16Reg);
		REG_WRITE_UINT16(NC_WIDTH, ~FCIE_REG41_VAL);
		REG_READ_UINT16(NC_WIDTH, u16Reg41h);

	}while (DEF_REG0x40_VAL != u16Reg || (U16)(~FCIE_REG41_VAL) != u16Reg41h);

	// miu request reset
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_MIU_REQUEST_RST);
	nand_hw_timer_delay(HW_TIMER_DELAY_1us);
	REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_MIU_REQUEST_RST);

	NC_PlatformResetPost();

	return UNFD_ST_SUCCESS;	// ok
}


U32 NC_ConfigContext(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();

	if (pNandDrv->u16_ECCType < ECC_TYPE_24BIT1KB)
		pNandDrv->u16_SectorByteCnt = 0x200;
	else
		pNandDrv->u16_SectorByteCnt	= 0x400;

	pNandDrv->u8_BlkPageCntBits = drvNAND_CountBits(pNandDrv->u16_BlkPageCnt);
	pNandDrv->u8_PageByteCntBits = drvNAND_CountBits(pNandDrv->u16_PageByteCnt);
	//pNandDrv->u8_SpareByteCntBits = drvNAND_CountBits(pNandDrv->u16_SpareByteCnt);
	pNandDrv->u8_SectorByteCntBits = drvNAND_CountBits(pNandDrv->u16_SectorByteCnt);

	pNandDrv->u16_BlkPageCntMask = (1<<pNandDrv->u8_BlkPageCntBits)-1;
	pNandDrv->u16_PageByteCntMask = (1<<pNandDrv->u8_PageByteCntBits)-1;
	pNandDrv->u16_SectorByteCntMask = (1<<pNandDrv->u8_SectorByteCntBits)-1;
	//pNandDrv->u16_SpareByteCntMask = (1<<pNandDrv->u8_SpareByteCntBits)-1;

	pNandDrv->u16_PageSectorCnt = pNandDrv->u16_PageByteCnt >> pNandDrv->u8_SectorByteCntBits;
	pNandDrv->u8_PageSectorCntBits = drvNAND_CountBits(pNandDrv->u16_PageSectorCnt);
	pNandDrv->u16_PageSectorCntMask = (1<<pNandDrv->u8_PageSectorCntBits)-1;
	pNandDrv->u16_SectorSpareByteCnt = pNandDrv->u16_SpareByteCnt >> pNandDrv->u8_PageSectorCntBits;
	pNandDrv->u16_SectorSpareByteCnt &= ~1;
	pNandDrv->u16_SpareByteCnt = pNandDrv->u16_SectorSpareByteCnt << pNandDrv->u8_PageSectorCntBits;
	//pNandDrv->u8_SectorSpareByteCntBits = drvNAND_CountBits(pNandDrv->u16_SectorSpareByteCnt);

	pNandDrv->u8_BlkSectorCntBits = pNandDrv->u8_BlkPageCntBits + pNandDrv->u8_PageSectorCntBits;
	pNandDrv->u16_BlkSectorCnt = 1<<pNandDrv->u8_BlkSectorCntBits;
	pNandDrv->u16_BlkSectorCntMask = pNandDrv->u16_BlkSectorCnt-1;

	return UNFD_ST_SUCCESS;
}


U32 NC_Init(void)
{
	U32 u32_RetVal;
	NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();

	// disable NC
	REG_CLR_BITS_UINT16(NC_PATH_CTL, BIT_NC_EN);
	REG_WRITE_UINT16(NC_CTRL , 0);
	// reset NC
	u32_RetVal = NC_ResetFCIE();
	if (UNFD_ST_SUCCESS != u32_RetVal)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_Init, ErrCode:%Xh \r\n", UNFD_ST_ERR_NO_NFIE);
		return u32_RetVal;
	}

	// disable interupts
	//REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_MMA_DATA_END | BIT_NC_JOB_END);
	// clean int events
	//REG_W1C_BITS_UINT16(NC_MIE_EVENT, BIT_MMA_DATA_END | BIT_NC_JOB_END);
	// enable NC
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_NC_EN);

	// config NC
	pNandDrv->u16_Reg1B_SdioCtrl = (pNandDrv->u16_SectorByteCnt +
									(pNandDrv->u16_SpareByteCnt >> pNandDrv->u8_PageSectorCntBits)) &~ 0x1UL;

	if (NC_MAX_SECTOR_BYTE_CNT < pNandDrv->u16_Reg1B_SdioCtrl)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: invalid Sector Size: %Xh bytes!\r\n", pNandDrv->u16_Reg1B_SdioCtrl);
		nand_die();
	}
	pNandDrv->u16_Reg1B_SdioCtrl |= BIT_SDIO_BLK_MODE;

//	pNandDrv->u16_Reg40_Signal =
//	    (BIT_NC_CE_AUTO|BIT_NC_CE_H|BIT_NC_WP_H) &
//	    ~(BIT_NC_CHK_RB_EDGEn | BIT_NC_CE_SEL_MASK);
	REG_WRITE_UINT16(NC_SIGNAL, pNandDrv->u16_Reg40_Signal);
	pNandDrv->u16_Reg48_Spare = (pNandDrv->u16_SpareByteCnt >> pNandDrv->u8_PageSectorCntBits);
	if (NC_MAX_SECTOR_SPARE_BYTE_CNT < pNandDrv->u16_Reg48_Spare || (pNandDrv->u16_Reg48_Spare & 1))
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: invalid Sector Spare Size: %Xh bytes!\r\n", pNandDrv->u16_Reg48_Spare);
		nand_die();
	}
    #if 0
	if(pNandDrv->u16_SpareByteCnt >= 512)
		pNandDrv->u16_Reg48_Spare |= BIT_NC_SPARE_DEST_MIU;
    #endif
    #if defined(CONFIG_MSTAR_MIPS) || 0==IF_SPARE_AREA_DMA
    pNandDrv->u16_Reg48_Spare &= ~BIT_NC_SPARE_DEST_MIU;
    #endif
    

	pNandDrv->u16_Reg49_SpareSize = pNandDrv->u16_SpareByteCnt;
	if (NC_MAX_TOTAL_SPARE_BYTE_CNT < pNandDrv->u16_Reg49_SpareSize || (pNandDrv->u16_Reg49_SpareSize & 1))
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: invalid Total Spare Size: %Xh bytes!\r\n", pNandDrv->u16_Reg49_SpareSize);
		nand_die();
	}

	pNandDrv->u16_Reg50_EccCtrl = REG50_ECC_CTRL_INIT_VALUE;
	if (pNandDrv->u8_WordMode)
		pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_WORD_MODE;
	nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1 ,"UNFD, WordMode:%X\r\n", pNandDrv->u8_WordMode);

	switch (pNandDrv->u16_PageByteCnt)
	{
		case 0x0200:
			pNandDrv->u16_Reg50_EccCtrl &= ~BIT_NC_PAGE_SIZE_512Bn;
			pNandDrv->u16_Reg48_Spare |= BIT_NC_ONE_COL_ADDR;
		break;
		case 0x0800:  pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_PAGE_SIZE_2KB;  break;
		case 0x1000:  pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_PAGE_SIZE_4KB;  break;
		case 0x2000:  pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_PAGE_SIZE_8KB;  break;
		case 0x4000:  pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_PAGE_SIZE_16KB;  break;
		case 0x8000:  pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_PAGE_SIZE_32KB;  break;
		default:
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: invalid Page Size: %Xh bytes!\r\n", pNandDrv->u16_PageByteCnt);
			nand_die();
	}

	switch (pNandDrv->u16_ECCType)
	{
		case ECC_TYPE_RS:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_RS;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_RS;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 2;
			pNandDrv->u16_ECCCorretableBit = 4;
			break;
		case ECC_TYPE_4BIT:
			pNandDrv->u16_Reg50_EccCtrl &= ~BIT_NC_ECC_TYPE_4b512Bn;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_4BIT;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 2;
			pNandDrv->u16_ECCCorretableBit = 4;
			break;
		case ECC_TYPE_8BIT:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_8b512B;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_8BIT;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 4;
			pNandDrv->u16_ECCCorretableBit = 8;
			break;
		case ECC_TYPE_12BIT:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_12b512B;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_12BIT;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 6;
			pNandDrv->u16_ECCCorretableBit = 12;
			break;
		case ECC_TYPE_16BIT:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_16b512B;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_16BIT;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 8;
			pNandDrv->u16_ECCCorretableBit = 16;
			break;
		case ECC_TYPE_20BIT:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_20b512B;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_20BIT;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 10;
			pNandDrv->u16_ECCCorretableBit = 20;
			break;
		case ECC_TYPE_24BIT:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_24b512B;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_24BIT;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 12;
			pNandDrv->u16_ECCCorretableBit = 24;
			break;
		case ECC_TYPE_24BIT1KB:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_24b1KB;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_24BIT1KB;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 12;
			pNandDrv->u16_ECCCorretableBit = 24;
			break;
		case ECC_TYPE_32BIT1KB:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_32b1KB;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_32BIT1KB;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 16;
			pNandDrv->u16_ECCCorretableBit = 32;
			break;
		case ECC_TYPE_40BIT1KB:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_40b1KB;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_40BIT1KB;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 20;
			pNandDrv->u16_ECCCorretableBit = 40;
			break;
		default:
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: invalid ECC Type: %Xh \r\n", pNandDrv->u16_ECCType);
			nand_die();
	}
	pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECCERR_NSTOP;

	#if defined(NC_HWCMD_DELAY) && NC_HWCMD_DELAY
	REG_READ_UINT16(NC_RAND_W_CMD, pNandDrv->u16_Reg56_Rand_W_Cmd);
	#endif

	#if defined(NC_TRR_TCS) && NC_TRR_TCS
	REG_READ_UINT16(NC_LFSR_CTRL, pNandDrv->u16_Reg59_LFSRCtrl);
	#endif

	pNandDrv->u8_PadMode = 1;

    // ===================================================
	NC_Config();
	#if defined(FCIE4_DDR) && FCIE4_DDR
    #if defined(FCIE4_DDR_EMMC_PLL) && FCIE4_DDR_EMMC_PLL
    u32_RetVal = NC_FCIE4SetInterface_EMMC_PLL(0, 0, 0);
    #else
	u32_RetVal = NC_FCIE4SetInterface(0,0,0,0);
    #endif
	#else
	u32_RetVal = NC_ResetNandFlash();
	#endif
	if(UNFD_ST_SUCCESS != u32_RetVal)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR,1,"ERROR, NAND reset flash, Err Code:%08Xh \n", u32_RetVal);
		return u32_RetVal;
	}

	u32_RetVal = NC_ReadID();
	if (u32_RetVal != UNFD_ST_SUCCESS) {
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Failed to read ID with EC: 0x%08x\n", u32_RetVal);
		u32_RetVal = UNFD_ST_ERR_UNKNOWN_ID;
		return u32_RetVal;
	}

	// ===================================================
	#if defined(FCIE4_DDR) && FCIE4_DDR

    #if defined(FCIE4_DDR_EMMC_PLL) && FCIE4_DDR_EMMC_PLL
    REG_READ_UINT16(REG_EMMC_PLL_RX09, pNandDrv->u16_Emmc_Pll_Reg09);
    #else
	REG_READ_UINT16(NC_SM_STS, pNandDrv->u16_Reg2C_SMStatus);
    #endif

	REG_READ_UINT16(NC_LATCH_DATA, pNandDrv->u16_Reg57_RELatch);
	REG_READ_UINT16(NC_DDR_CTRL, pNandDrv->u16_Reg58_DDRCtrl);
    // detect if Toggle-DDR
	pNandDrv->u16_Reg58_DDRCtrl &= ~BIT_DDR_MASM;
	u32_RetVal = NC_ProbeIfToggleDDR();
	if(UNFD_ST_ERR_NOT_TOGGLE_DDR == u32_RetVal)
	{
		nand_debug(UNFD_DEBUG_LEVEL_HIGH,1,"NOT Toggle DDR \n");

		if(UNFD_ST_ERR_NOT_ONFI_DDR == NC_ProbeIfONFIDDR())
		{
			nand_debug(UNFD_DEBUG_LEVEL_HIGH,1,"NOT ONFI DDR \n");
			nand_debug(0,1,"SDR NAND is detected\n");
			pNandDrv->u8_PadMode = NAND_PAD_BYPASS_MODE;
		}
		else
		{
			nand_debug(0,1,"YES ONFI DDR \n");

			u32_RetVal = NC_SetONFISyncMode();
			if(UNFD_ST_ERR_PAD_UNSUPPORT_DDR_NAND == u32_RetVal)
			{
				nand_debug(0,1, "SDR NAND is detected\n");
				NC_Config();
				return UNFD_ST_SUCCESS;
			}
			else if(UNFD_ST_SUCCESS != u32_RetVal)
			{
				nand_debug(UNFD_DEBUG_LEVEL_ERROR,1,"ERROR, NAND SetFeature ONFI-DDR, Err Code:%08lXh \n", u32_RetVal);
				return u32_RetVal;
			}

			nand_pads_switch(NAND_PAD_ONFI_SYNC_MODE);
			pNandDrv->u8_PadMode = NAND_PAD_ONFI_SYNC_MODE;

			u32_RetVal= NC_DetectDDRTiming();
			if(UNFD_ST_SUCCESS != u32_RetVal)
			{
				nand_debug(UNFD_DEBUG_LEVEL_ERROR,1,"ERROR, NAND config timing, ONFI-DDR, Err Code:%08lXh \n", u32_RetVal);
				return u32_RetVal;
			}
		}
	}
	else
	{
		nand_debug(0,1,"Yes Toggle DDR \n");

		nand_pads_switch(NAND_PAD_TOGGLE_MODE);
		pNandDrv->u8_PadMode = NAND_PAD_TOGGLE_MODE;

		u32_RetVal = NC_DetectDDRTiming();
		if(UNFD_ST_SUCCESS != u32_RetVal)
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR,1,"ERROR, NAND config timing, Toggle-DDR, Err Code:%08lXh \n", u32_RetVal);
			return u32_RetVal;
		}
	}
	// ===================================================
	#endif

	NC_Config();

	return UNFD_ST_SUCCESS;
}


#if defined(FCIE_LFSR) && FCIE_LFSR
U32 NC_EnableLFSR(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	volatile U16 u16_RegVal;

	REG_READ_UINT16(NC_LFSR_CTRL, u16_RegVal);

	if(0 == (u16_RegVal & BIT_LFSR_ENABLE))
	{
		// tune timing: RE_LATCH
		if(0 == (pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)){
			REG_READ_UINT16(NC_LATCH_DATA, u16_RegVal);
			u16_RegVal = u16_RegVal & BIT_NC_LATCH_DATA_MASK;
			if(u16_RegVal > BIT_NC_LATCH_DATA_1_0_T)
			{
				nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error, RD_LATCH already > 1T, can NOT use LFSR.\n");
				return UNFD_ST_ERR_LFSR_RD_LATCH;
			}
			pNandDrv->u16_Reg57_RELatch &= ~BIT_NC_LATCH_DATA_MASK;
			pNandDrv->u16_Reg57_RELatch += u16_RegVal + BIT_NC_LATCH_DATA_1_0_T;
		}

		#if defined(FCIE4_DDR) && FCIE4_DDR
		else
		{
			// tune timing: DDR_TIMING
			REG_READ_UINT16(NC_LATCH_DATA, u16_RegVal);
			u16_RegVal = u16_RegVal & BIT_RE_DDR_TIMING_MASK;
			if(u16_RegVal == BIT_RE_DDR_TIMING_MASK)
			{
				nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error, DDR_TIMING full, can NOT use LFSR.\n");
				return UNFD_ST_ERR_LFSR_DDRTIMING;
			}
			pNandDrv->u16_Reg57_RELatch += 1<<BIT_RE_DDR_TIMING_SHIFT;
		}
		#endif

		// walk around timing bug
		pNandDrv->u16_Reg57_RELatch &= ~BIT_RE_SEC_TURN_CNT_MASK;
		pNandDrv->u16_Reg57_RELatch |= 0xE << BIT_RE_SEC_TURN_CNT_SHIFT;

		REG_WRITE_UINT16(NC_LATCH_DATA, pNandDrv->u16_Reg57_RELatch);

		// set LFST
		pNandDrv->u16_Reg59_LFSRCtrl |= BIT_LFSR_ENABLE;
		REG_SET_BITS_UINT16(NC_LFSR_CTRL, BIT_LFSR_ENABLE);
	}

	return UNFD_ST_SUCCESS;
}

U32 NC_DisableLFSR(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	volatile U16 u16_RegVal;

	REG_READ_UINT16(NC_LFSR_CTRL, u16_RegVal);

	if(u16_RegVal & BIT_LFSR_ENABLE)
	{
		if(0 == (pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM))
			pNandDrv->u16_Reg57_RELatch -= BIT_NC_LATCH_DATA_1_0_T;

		#if defined(FCIE4_DDR) && FCIE4_DDR
		pNandDrv->u16_Reg57_RELatch -= 1<<BIT_RE_DDR_TIMING_SHIFT;
		#endif

		REG_WRITE_UINT16(NC_LATCH_DATA, pNandDrv->u16_Reg57_RELatch);

		// clear LFST
		pNandDrv->u16_Reg59_LFSRCtrl &= ~BIT_LFSR_ENABLE;
		REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_LFSR_ENABLE);
	}
	return UNFD_ST_SUCCESS;
}
#endif
U32 NC_ReInit(void)
{
	NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();

	// config NC
	pNandDrv->u16_Reg1B_SdioCtrl = (pNandDrv->u16_SectorByteCnt +
									(pNandDrv->u16_SpareByteCnt >> pNandDrv->u8_PageSectorCntBits)) &~ 0x1UL;

	if (NC_MAX_SECTOR_BYTE_CNT < pNandDrv->u16_Reg1B_SdioCtrl)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: invalid Sector Size: %Xh bytes!\r\n", pNandDrv->u16_Reg1B_SdioCtrl);
		nand_die();
	}
	pNandDrv->u16_Reg1B_SdioCtrl |= BIT_SDIO_BLK_MODE;

	pNandDrv->u16_Reg48_Spare = (pNandDrv->u16_SpareByteCnt >> pNandDrv->u8_PageSectorCntBits);
	if (NC_MAX_SECTOR_SPARE_BYTE_CNT < pNandDrv->u16_Reg48_Spare || (pNandDrv->u16_Reg48_Spare & 1))
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: invalid Sector Spare Size: %Xh bytes!\r\n", pNandDrv->u16_Reg48_Spare);
		nand_die();
	}

    #if 0
	if(pNandDrv->u16_SpareByteCnt >= 512)
		pNandDrv->u16_Reg48_Spare |= BIT_NC_SPARE_DEST_MIU;
    #endif
    #if defined(CONFIG_MSTAR_MIPS) || 0==IF_SPARE_AREA_DMA
    pNandDrv->u16_Reg48_Spare &= ~BIT_NC_SPARE_DEST_MIU;
    #endif
    
	pNandDrv->u16_Reg49_SpareSize = pNandDrv->u16_SpareByteCnt;
	if (NC_MAX_TOTAL_SPARE_BYTE_CNT < pNandDrv->u16_Reg49_SpareSize || (pNandDrv->u16_Reg49_SpareSize & 1))
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: invalid Total Spare Size: %Xh bytes!\r\n", pNandDrv->u16_Reg49_SpareSize);
		nand_die();
	}

	pNandDrv->u16_Reg50_EccCtrl = REG50_ECC_CTRL_INIT_VALUE;
	if (pNandDrv->u8_WordMode)
		pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_WORD_MODE;
	nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1 ,"UNFD, WordMode:%X\r\n", pNandDrv->u8_WordMode);

	switch (pNandDrv->u16_PageByteCnt)
	{
		case 0x0200:  pNandDrv->u16_Reg50_EccCtrl &= ~BIT_NC_PAGE_SIZE_512Bn;  break;
		case 0x0800:  pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_PAGE_SIZE_2KB;  break;
		case 0x1000:  pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_PAGE_SIZE_4KB;  break;
		case 0x2000:  pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_PAGE_SIZE_8KB;  break;
		case 0x4000:  pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_PAGE_SIZE_16KB;  break;
		case 0x8000:  pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_PAGE_SIZE_32KB;  break;
		default:
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: invalid Page Size: %Xh bytes!\r\n", pNandDrv->u16_PageByteCnt);
			nand_die();
	}

	pNandDrv->u16_Reg48_Spare |= pNandDrv->u16_One_Col_Addr;

	switch (pNandDrv->u16_ECCType)
	{
		case ECC_TYPE_RS:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_RS;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_RS;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 2;
			pNandDrv->u16_ECCCorretableBit = 4;
			break;
		case ECC_TYPE_4BIT:
			pNandDrv->u16_Reg50_EccCtrl &= ~BIT_NC_ECC_TYPE_4b512Bn;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_4BIT;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 2;
			pNandDrv->u16_ECCCorretableBit = 4;
			break;
		case ECC_TYPE_8BIT:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_8b512B;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_8BIT;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 4;
			pNandDrv->u16_ECCCorretableBit = 8;
			break;
		case ECC_TYPE_12BIT:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_12b512B;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_12BIT;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 6;
			pNandDrv->u16_ECCCorretableBit = 12;
			break;
		case ECC_TYPE_16BIT:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_16b512B;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_16BIT;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 8;
			pNandDrv->u16_ECCCorretableBit = 16;
			break;
		case ECC_TYPE_20BIT:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_20b512B;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_20BIT;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 10;
			pNandDrv->u16_ECCCorretableBit = 20;
			break;
		case ECC_TYPE_24BIT:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_24b512B;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_24BIT;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 12;
			pNandDrv->u16_ECCCorretableBit = 24;
			break;
		case ECC_TYPE_24BIT1KB:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_24b1KB;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_24BIT1KB;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 12;
			pNandDrv->u16_ECCCorretableBit = 24;
			break;
		case ECC_TYPE_32BIT1KB:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_32b1KB;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_32BIT1KB;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 16;
			pNandDrv->u16_ECCCorretableBit = 32;
			break;
		case ECC_TYPE_40BIT1KB:
			pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECC_TYPE_40b1KB;
			pNandDrv->u16_ECCCodeByteCnt = ECC_CODE_BYTECNT_40BIT1KB;
			if(!pNandDrv->u16_BitflipThreshold)
				pNandDrv->u16_BitflipThreshold = 20;
			pNandDrv->u16_ECCCorretableBit = 40;
			break;
		default:
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: invalid ECC Type: %Xh \r\n", pNandDrv->u16_ECCType);
			nand_die();
	}
	pNandDrv->u16_Reg50_EccCtrl |= BIT_NC_ECCERR_NSTOP;

	return UNFD_ST_SUCCESS;
}

void NC_Config(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();

	REG_WRITE_UINT16(NC_SDIO_CTL, pNandDrv->u16_Reg1B_SdioCtrl);
	REG_WRITE_UINT16(NC_SIGNAL, pNandDrv->u16_Reg40_Signal);
	/*sector spare size*/
	REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);
	/* page spare size*/
	REG_WRITE_UINT16(NC_SPARE_SIZE, (U16)pNandDrv->u16_Reg49_SpareSize);
	/* page size and ECC type*/
	REG_WRITE_UINT16(NC_ECC_CTRL, pNandDrv->u16_Reg50_EccCtrl);

	REG_WRITE_UINT16(NC_LATCH_DATA, pNandDrv->u16_Reg57_RELatch);

	#if defined(FCIE4_DDR) && FCIE4_DDR

    #if !(defined(FCIE4_DDR_EMMC_PLL) && FCIE4_DDR_EMMC_PLL)
	REG_SET_BITS_UINT16(NC_SM_STS, pNandDrv->u16_Reg2C_SMStatus & (BIT_DQS_DELAY_CELL_MASK|BIT_DQS_MODE_MASK));
    #endif

	// mark the following line, to save power (disable CLK) for ONFI-DDR
	//REG_WRITE_UINT16(NC_DDR_CTRL, pNandDrv->u16_Reg58_DDRCtrl);
	#endif

	#if defined(NC_TWHR_TCLHZ) && NC_TWHR_TCLHZ
	/*set timing for tCLHZ*/
	REG_WRITE_UINT16(NC_NAND_TIMING, pNandDrv->u16_Reg5A_tWHR_tCLHZ);
	#endif

	#if defined(NC_TCWAW_TADL) && NC_TCWAW_TADL
	/*set timing for tADL and tCWAW*/
	REG_WRITE_UINT16(NC_NAND_TIMING1,pNandDrv->u16_Reg5D_tCWAW_tADL);
	#endif

	#if (defined(FCIE_LFSR) && FCIE_LFSR) || (defined(NC_TRR_TCS) && NC_TRR_TCS)
	REG_WRITE_UINT16(NC_LFSR_CTRL, pNandDrv->u16_Reg59_LFSRCtrl);
	#endif

	#if defined(NC_HWCMD_DELAY) && NC_HWCMD_DELAY
	REG_WRITE_UINT16(NC_RAND_W_CMD, pNandDrv->u16_Reg56_Rand_W_Cmd);
	#endif
}

#if IF_FCIE_SHARE_IP
void NC_ReConfig(void) // re-config FCIE3 for NFIE mode
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	volatile U16 u16Reg;

	#if defined(ENABLE_NAND_INTERRUPT_MODE) && ENABLE_NAND_INTERRUPT_MODE
	// enable interrupts if system allows
	//nand_enable_intr_mode();
	#else
	// disable interupts
	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_MMA_DATA_END | BIT_NC_JOB_END);
	#endif
	// clean int events
	REG_W1C_BITS_UINT16(NC_MIE_EVENT, BIT_MMA_DATA_END | BIT_NC_JOB_END);

	REG_WRITE_UINT16(NC_PATH_CTL, BIT_NC_EN);

	REG_WRITE_UINT16(NC_SDIO_CTL, pNandDrv->u16_Reg1B_SdioCtrl);
	REG_WRITE_UINT16(NC_SIGNAL, pNandDrv->u16_Reg40_Signal);
	REG_READ_UINT16(NC_SIGNAL, u16Reg);
	if(u16Reg != pNandDrv->u16_Reg40_Signal)
	{
	    nand_debug(0,1,"ERROR, NAND, FCIE Reset fail \n");
		nand_die();
	}

	/*sector spare size*/
	REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);
	/* page spare size*/
	REG_WRITE_UINT16(NC_SPARE_SIZE, (U16)pNandDrv->u16_Reg49_SpareSize);
	/* page size and ECC type*/
	REG_WRITE_UINT16(NC_ECC_CTRL, pNandDrv->u16_Reg50_EccCtrl);

	REG_WRITE_UINT16(NC_LATCH_DATA, pNandDrv->u16_Reg57_RELatch);

	#if defined(FCIE4_DDR) && FCIE4_DDR

    #if !(defined(FCIE4_DDR_EMMC_PLL) && FCIE4_DDR_EMMC_PLL)
	REG_SET_BITS_UINT16(NC_SM_STS, pNandDrv->u16_Reg2C_SMStatus & (BIT_DQS_DELAY_CELL_MASK|BIT_DQS_MODE_MASK));
    #endif

	// mark the following line, to save power (disable CLK) for ONFI-DDR
	//REG_WRITE_UINT16(NC_DDR_CTRL, pNandDrv->u16_Reg58_DDRCtrl);
	#endif

	#if defined(NC_TWHR_TCLHZ) && NC_TWHR_TCLHZ
	/*set timing for tCLHZ*/
	REG_WRITE_UINT16(NC_NAND_TIMING, pNandDrv->u16_Reg5A_tWHR_tCLHZ);
	#endif

	#if defined(NC_TCWAW_TADL) && NC_TCWAW_TADL
	/*set timing for tADL and tCWAW*/
	REG_WRITE_UINT16(NC_NAND_TIMING1,pNandDrv->u16_Reg5D_tCWAW_tADL);
	#endif

	#if (defined(FCIE_LFSR) && FCIE_LFSR) || (defined(NC_TRR_TCS) && NC_TRR_TCS)
	REG_WRITE_UINT16(NC_LFSR_CTRL, pNandDrv->u16_Reg59_LFSRCtrl);
	#endif

	#if defined(NC_HWCMD_DELAY) && NC_HWCMD_DELAY
	REG_WRITE_UINT16(NC_RAND_W_CMD, pNandDrv->u16_Reg56_Rand_W_Cmd);
	#endif
}
#endif

#define CHK_EMPTY_RB4W   0
#if CHK_EMPTY_RB4W
extern U8 au8_DataReadBuf[16384];
extern U8 au8_SpareReadBuf[2000];
#endif
// can not cross block
U32 NC_WritePages
(
    U32 u32_PhyRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr=0, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8  u8_RetryCnt_FifoClkRdy=0;
	U32 u32_Ret;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8	*pu8_OrigSpareBuf = pu8_SpareBuf;

    //nand_debug(0,0,"W:%Xh, %Xh \n", u32_PhyRowIdx, u32_PageCnt);
	// can not cross block
	if ((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
		pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_WritePages, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_W_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_W_INVALID_PARAM;
	}
    #if CHK_EMPTY_RB4W
    {
		NC_ReadPages(u32_PhyRowIdx, au8_DataReadBuf, au8_SpareReadBuf, 1);
		if(0==nand_CheckEmptyPageFalseAlarm(au8_DataReadBuf, au8_SpareReadBuf))
		{
			nand_debug(0, 1, "Write no blank page row %Xh\n", u32_PhyRowIdx);
            dump_stack();
			nand_die();
		}
	}
    #endif
	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE(); // to turn on ONFI clk

    nand_redo_write:

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END|BIT_MIU_LAST_DONE);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			if(pu8_OrigSpareBuf != NULL)
				memcpy(pu8_SpareBuf, pu8_OrigSpareBuf, pNandDrv->u16_SpareByteCnt);
			else
				memset(pu8_SpareBuf, 0xFF, pNandDrv->u16_SpareByteCnt);
		}

		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					   0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);

			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_JOB_BL_CNT, u32_PageCnt << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto nand_redo_write;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			NC_DumpRegisters();
			NC_DumpDebugBus();
			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x10 << 8) | ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x70 << 8) | ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_REPEAT << 8) | ACT_CHKSTATUS);
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 1);
	//nand_debug(1, 1, "W Rpt Cnt: %Xh \r\n", u16_PageCnt-1);

	if(u8_IsSpareDMA == 0)
	{
		if (pu8_SpareBuf)
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
		else
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD_Const(0xFF, pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
								 pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
	}

	nand_CheckPowerCut();

	REG_WRITE_UINT16(NC_CTRL,
					 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WritePages Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		#else
		//nand_stop();
		#endif
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);

		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}

	u32_Ret = NC_CheckEWStatus(OPTYPE_WRITE);

	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

	nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_PageCnt * pNandDrv->u16_PageByteCnt;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_write_count++;
            gu64_monitor_write_size_kb += ((pNandDrv->u16_PageByteCnt / 1024) * u32_PageCnt);
        }
	return u32_Ret;
}

U32 NC_ReadPages
(
    U32 u32_PhyRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	U16 u16_Tmp=0;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr = 0, u32_SpareMapAddr = 0;
	U32 u32_Err = UNFD_ST_SUCCESS;
	U8	u8_RetryCnt_FifoClkRdy=0;
#if defined(FCIE_LFSR) && FCIE_LFSR
	U8	u8_RandRetryFlag = 0;
#endif
	U8	u8_RetryCnt_Events=0, u8_RetryCnt_ECCFail=0, u8_RetryDqs=0;
	U8	u8_MaxReadRetryCount = NAND_TIMEOUT_RETRY_CNT;
	U8	u8_RequireReadRetry = pNandDrv->u8_RequireReadRetry;
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8	u8_EnableIRQ = 0;

	//check if IRQ enable-> if yes disable irq check for read
	REG_READ_UINT16(NC_MIE_INT_EN, u16_Tmp);
	if( (u16_Tmp & (BIT_NC_JOB_END|BIT_MMA_DATA_END)) ==
		(BIT_NC_JOB_END|BIT_MMA_DATA_END) )
		u8_EnableIRQ = 1;

	if(pNandDrv->u8_RequireReadRetry)
		u8_MaxReadRetryCount = pNandDrv->ReadRetry_t.u8_MaxRetryTime + 1;

	// can not cross block
	if ((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
		pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_ReadPages, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_R_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_R_INVALID_PARAM;
	}

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();

	nand_redo_read:
	NC_SET_DDR_MODE();
	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END|BIT_MIU_LAST_DONE);

#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
#endif

	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_PageByteCnt * u32_PageCnt, READ_FROM_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, READ_FROM_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)		//main data dma to miu1
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
		}
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		}
		else
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_SER_DIN_BYTECNT_LW, pNandDrv->u16_PageByteCnt & 0xFFFF);
	REG_WRITE_UINT16(NC_SER_DIN_BYTECNT_HW, pNandDrv->u16_PageByteCnt >> 16);
	REG_WRITE_UINT16(NC_JOB_BL_CNT, u32_PageCnt << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);//>>MIU_BUS_WIDTH_BITS));
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);//(MIU_BUS_WIDTH_BITS+16)));
	REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
	#if 0==IF_IP_VERIFY
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, READ_FROM_NAND);

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto nand_redo_read;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			NC_DumpRegisters();
			NC_DumpDebugBus();


			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			goto EXIT;//return u32_Err;
		}
	#else
		//nand_stop();
	#endif

		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, READ_FROM_NAND);
		NC_CLR_DDR_MODE();
		NC_UNLOCK_FCIE();
		goto EXIT;//return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);
	if( pNandDrv->u16_bbm_wa == 0xBAD)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	}

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);
	#endif
	if( pNandDrv->u16_bbm_wa == 0xBAD)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, ((OP_ADR_CYCLE_00|OP_ADR_TYPE_ONE|OP_ADR_SET_1) << 8) | CMD_0x80);
	}
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x00));

	if (pNandDrv->u16_Reg48_Spare& BIT_NC_ONE_COL_ADDR)
	{ // if a page 512B
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | ACT_WAITRB);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_REPEAT);
	}
	else
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x30);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_REPEAT << 8) | ACT_SER_DIN);
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
	}
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 1);

	// check & set - patch for MIU busy case
	// slow down FCIE clock to wait for MIU service,
	// enhance the retry (more reliable) for error handling.
	if(0!=u8_RetryCnt_Events || 0!=u8_RetryCnt_ECCFail || 0!=u8_RetryDqs)
	{
	#if defined(FCIE4_DDR) && FCIE4_DDR
		if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
		{
		#if defined( FCIE4_DDR_RETRY_DQS) && FCIE4_DDR_RETRY_DQS
			if(0!=u8_RetryDqs)
			{
				nand_clock_setting(pNandDrv->u32_minClk);
				NC_FCIE4SetInterface(1, pNandDrv->tMinDDR.u8_DqsMode, pNandDrv->tMinDDR.u8_DelayCell, pNandDrv->tMinDDR.u8_DdrTiming);
			}
		#else
			nand_clock_setting(pNandDrv->u32_minClk);

			#if defined(FCIE4_DDR_EMMC_PLL) && FCIE4_DDR_EMMC_PLL
			NC_FCIE4SetInterface_EMMC_PLL(1, pNandDrv->tMinDDR.u8_DqsMode, pNandDrv->tMinDDR.u8_DdrTiming);
			#else
			NC_FCIE4SetInterface(1, pNandDrv->tMinDDR.u8_DqsMode, pNandDrv->tMinDDR.u8_DelayCell, pNandDrv->tMinDDR.u8_DdrTiming);
		#endif

        #endif
		}
		else
	#endif
		{
			if(pNandDrv->u8_RequireRandomizer == 0)
			{
				nand_clock_setting(FCIE3_SW_SLOWEST_CLK);
				REG_WRITE_UINT16(NC_WIDTH, 0);
				REG_WRITE_UINT16(NC_LATCH_DATA, (REG(NC_LATCH_DATA) & ~BIT_NC_LATCH_DATA_MASK));
			}
		}
		//nand_printf("check & set - patch for MIU busy case \n");
	}

	if(u8_EnableIRQ == 1)
		REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq for read

	REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START) & ~(BIT_NC_IF_DIR_W));

#if defined(ENABLE_CB_BEFORE_DMA) && ENABLE_CB_BEFORE_DMA
	if( pNandDrv->pfCB != NULL && pNandDrv->u8_StartCB == 1)
		pNandDrv->pfCB(pNandDrv->u16_LogiSectorCntCB*512);
#endif

	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME * u32_PageCnt) ==
		(WAIT_READ_TIME * u32_PageCnt ))
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadPages Timeout, RetryCnt:%d, ErrCode:%Xh \r\n", u8_RetryCnt_Events, UNFD_ST_ERR_R_TIMEOUT);
	#if 0==IF_IP_VERIFY
		if((gu8_DisableErrorLog == 0)&&(u8_RetryCnt_Events==(NAND_TIMEOUT_RETRY_CNT-1)))
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, READ_FROM_NAND);
		if( ++u8_RetryCnt_Events < NAND_TIMEOUT_RETRY_CNT )
			goto nand_redo_read;
		else
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);

			//If soft reset still can not solve this problem, show an alert and return a error
			// restore the normal setting before return
		#if defined(FCIE4_DDR) && FCIE4_DDR
			if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
			{
				nand_clock_setting(pNandDrv->u32_Clk);

				#if defined(FCIE4_DDR_EMMC_PLL) && FCIE4_DDR_EMMC_PLL
				NC_FCIE4SetInterface_EMMC_PLL(1, pNandDrv->tDefaultDDR.u8_DqsMode, pNandDrv->tDefaultDDR.u8_DdrTiming);
				#else
				NC_FCIE4SetInterface(1, pNandDrv->tDefaultDDR.u8_DqsMode, pNandDrv->tDefaultDDR.u8_DelayCell, pNandDrv->tDefaultDDR.u8_DdrTiming);
				#endif
			}
			else
		#endif
			{
				nand_clock_setting(pNandDrv->u32_Clk);
				REG_WRITE_UINT16(NC_WIDTH, FCIE_REG41_VAL);
				REG_WRITE_UINT16(NC_LATCH_DATA, pNandDrv->u16_Reg57_RELatch);
			}

			if(u8_EnableIRQ == 1)
				REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq for read

			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			u32_Err = UNFD_ST_ERR_R_TIMEOUT;
			goto EXIT;//return UNFD_ST_ERR_R_TIMEOUT;
		}
	#else
		//nand_stop();
	#endif
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, READ_FROM_NAND);

		if(u8_EnableIRQ == 1)
			REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq for read

		NC_CLR_DDR_MODE();
		u32_Err = UNFD_ST_ERR_R_TIMEOUT;
		goto EXIT;//return UNFD_ST_ERR_R_TIMEOUT;
	}
	//-----------------------------------
	if(u8_EnableIRQ == 1)
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq for read
	FLUSH_MIU_PIPE(); // Only used in U4 now

	#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
	// check until MIU is done
	if( (u32_Err=NC_wait_MIULastDone()) != UNFD_ST_SUCCESS)
	{

		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, READ_FROM_NAND);

		if( ++u8_RetryCnt_Events < NAND_TIMEOUT_RETRY_CNT )
			goto nand_redo_read;
		else
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			goto EXIT;//return u32_Err;
		}

		NC_CLR_DDR_MODE();
		NC_UNLOCK_FCIE();
		goto EXIT;//return u32_Err;
	}
	#else
	for(u16_Tmp=0; u16_Tmp<NAND_DMA_PATCH_WAIT_TIME; u16_Tmp++)
	{
		if(((U32*)pu8_DataBuf)[pNandDrv->u16_SectorByteCnt/sizeof(U32)-1] != NAND_DMA_RACING_PATTERN)
		{
			if(pu8_SpareBuf && ((U32*)pu8_SpareBuf)[pNandDrv->u16_SectorSpareByteCnt/sizeof(U32)-1] != NAND_DMA_RACING_PATTERN)
				break;
		}

		nand_hw_timer_delay(HW_TIMER_DELAY_1us);
	}
	if(NAND_DMA_PATCH_WAIT_TIME == u16_Tmp)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_ReadPages MIU Patch Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_MIU_RPATCH_TIMEOUT);

		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, READ_FROM_NAND);

		if( ++u8_RetryCnt_MIURacing < NAND_TIMEOUT_RETRY_CNT )
			goto nand_redo_read;
		else
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Soft reset over 5 times\n, stop!\n"); //If soft reset still can not solve this problem, show an alert and return a error

			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			u32_Err = UNFD_ST_ERR_MIU_RPATCH_TIMEOUT;
			goto EXIT;//return UNFD_ST_ERR_MIU_RPATCH_TIMEOUT;
		}
	}
	#endif
//JC: dual core with L2 hardcore patch
#ifdef CONFIG_MIPS_CMP
	_dma_cache_wback_inv(pu8_DataBuf, pNandDrv->u16_PageByteCnt * u32_PageCnt);
	if(u8_IsSpareDMA)
	{
		if(pu8_SpareBuf)
			_dma_cache_wback_inv(pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		else
			_dma_cache_wback_inv(pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt);
	}

#endif

#if defined(CONFIG_ARM) // ARM: AMBER3, AGATE
//  Chip_Flush_Cache_Range_VA_PA((U32)pu8_DataBuf,__pa(pu8_DataBuf),pNandDrv->u16_PageByteCnt * u32_PageCnt);
#endif

	// restore - patch for MIU busy case
	// for retry ok, restore the normal setting
	if(0!=u8_RetryCnt_Events || 0!=u8_RetryCnt_ECCFail || 0!=u8_RetryDqs)
	{
	#if defined(FCIE4_DDR) && FCIE4_DDR
		if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
		{
		#if defined( FCIE4_DDR_RETRY_DQS) && FCIE4_DDR_RETRY_DQS
			if(0!=u8_RetryDqs)
			{
				nand_clock_setting(pNandDrv->u32_Clk);
				NC_FCIE4SetInterface(1, pNandDrv->tDefaultDDR.u8_DqsMode, pNandDrv->tDefaultDDR.u8_DelayCell, pNandDrv->tDefaultDDR.u8_DdrTiming);
			}
		#else
			nand_clock_setting(pNandDrv->u32_Clk);
            #if defined(FCIE4_DDR_EMMC_PLL) && FCIE4_DDR_EMMC_PLL
            NC_FCIE4SetInterface_EMMC_PLL(1, pNandDrv->tDefaultDDR.u8_DqsMode, pNandDrv->tDefaultDDR.u8_DdrTiming);
            #else
			NC_FCIE4SetInterface(1, pNandDrv->tDefaultDDR.u8_DqsMode, pNandDrv->tDefaultDDR.u8_DelayCell, pNandDrv->tDefaultDDR.u8_DdrTiming);
		#endif

        #endif
		}
		else
	#endif
		{
			nand_clock_setting(pNandDrv->u32_Clk);
			REG_WRITE_UINT16(NC_WIDTH, FCIE_REG41_VAL);
			REG_WRITE_UINT16(NC_LATCH_DATA, pNandDrv->u16_Reg57_RELatch);
		}
		//nand_printf("restore - patch for MIU busy case \n");
	}

    if (pu8_SpareBuf && u8_IsSpareDMA == 0)
	{
	    #if SPARE640B_CIFD512B_PATCH
		if(pNandDrv->u16_SpareByteCnt > 0x200)
			NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
		else
	    #endif
		NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SpareByteCnt);
	}
    
	// Check ECC
	REG_READ_UINT16(NC_ECC_STAT0, u16_Tmp);
	if (u16_Tmp & BIT_NC_ECC_FAIL)
	{
		//NC_SendCmdForLADebug();		

        #if 0 == IF_IP_VERIFY
		// add retry for ECC error
		if( u8_RetryCnt_ECCFail < u8_MaxReadRetryCount)
		{
            //nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Retry times : %X\n", u8_RetryCnt_ECCFail);
            //nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Blk: %lXh  Page: %lX\r\n",
                //u32_PhyRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PhyRowIdx&pNandDrv->u16_BlkPageCntMask);
			//NOTICE:
			//Randomizer retry works only on pagecnt = 1, otherwise if there are empty and non-empty pages in a job.
			//ECC fail will always happen even the Randomizer is disabled.
			if(!pNandDrv->u8_RequireRandomizer)
			{
				if(u8_RetryCnt_ECCFail >= 1 && u8_RequireReadRetry)
					NC_SendReadRetryCmd(u8_RetryCnt_ECCFail - 1, 0);

				u8_RetryCnt_ECCFail++;
			}
		#if defined(FCIE_LFSR) && FCIE_LFSR
			else
			{
				if(!u8_RandRetryFlag)
				{
					NC_DisableLFSR();
					u8_RandRetryFlag = 1;
				}
				else
				{
					//nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Retry times : %X\n", u8_RetryCnt_ECCFail);
					if(u8_RetryCnt_ECCFail >= 1 && u8_RequireReadRetry)
						NC_SendReadRetryCmd(u8_RetryCnt_ECCFail - 1, 0);

					NC_EnableLFSR();
					u8_RetryCnt_ECCFail++;
				}
			}
		#endif
			goto nand_redo_read;
		}
	#if defined(FCIE_LFSR) && FCIE_LFSR
		else if (u8_RetryCnt_ECCFail == u8_MaxReadRetryCount && pNandDrv->u8_RequireRandomizer == 1)
		{
			if(u8_RequireReadRetry && pNandDrv->au8_ID[0]!= 0xAD)
				NC_SendReadRetryCmd(0, 1);
			NC_DisableLFSR();
			u8_RetryCnt_ECCFail ++;
			goto nand_redo_read;
		}
	#endif
		else
		{
			int falsealarm;
			if(pu8_SpareBuf !=NULL)
				falsealarm = nand_CheckEmptyPageFalseAlarm(pu8_DataBuf, pu8_SpareBuf);
			else
				falsealarm = nand_CheckEmptyPageFalseAlarm(pu8_DataBuf, pNandDrv->PlatCtx_t.pu8_PageSpareBuf);
			
			if(falsealarm == 1)
			{
				gint_IS_ECC_Fail = 1;
				if(u8_RequireReadRetry && pNandDrv->u8_RequireRandomizer == 0  && pNandDrv->au8_ID[0]!= 0xAD)
				{
					NC_SendReadRetryCmd(0, 1);
				}
				//Clean False Alarm Status
				NC_ReConfig();
				NC_CLR_DDR_MODE();
				goto EXIT;
			}
		#if defined( FCIE4_DDR_RETRY_DQS) && FCIE4_DDR_RETRY_DQS
			if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
			{
				if(u8_RetryDqs == 0)
				{
					u8_RetryDqs = 1;
					u8_RetryCnt_ECCFail = 0;
					u8_RetryCnt_Events = 0;
					u8_RandRetryFlag = 0;

					if(u8_RequireReadRetry && pNandDrv->au8_ID[0]!= 0xAD)
					{
						NC_SendReadRetryCmd(0, 1);
					}

				#if defined(FCIE_LFSR) && FCIE_LFSR
					if(pNandDrv->u8_RequireRandomizer)
						NC_EnableLFSR();
				#endif
					goto nand_redo_read;
				}
			}
		    #endif

			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadPages ECC Fail, Reg51:%04Xh \n", u16_Tmp);
			REG_READ_UINT16(NC_ECC_STAT1, u16_Tmp);
			/*Remove register log for ECC fail*/
			//nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "Reg52:%04Xh ", u16_Tmp);
			//REG_READ_UINT16(NC_ECC_STAT2, u16_Tmp);
			//nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "Reg53:%04Xh \r\n", u16_Tmp);
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Blk: %Xh  Page: %Xh\r\n",
				u32_PhyRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PhyRowIdx&pNandDrv->u16_BlkPageCntMask);
			//NC_DumpRegisters();
			//nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR, NAND, RETRY_READ_ECC_FAIL over %d times \n", u8_MaxReadRetryCount);

			if(u8_RequireReadRetry && pNandDrv->u8_RequireRandomizer == 0 && pNandDrv->au8_ID[0]!= 0xAD)
			{
				NC_SendReadRetryCmd(0, 1);
			}
			NC_UNLOCK_FCIE();
		}
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, READ_FROM_NAND);

		NC_CLR_DDR_MODE();
		u32_Err = UNFD_ST_ERR_ECC_FAIL;
		goto EXIT;//return UNFD_ST_ERR_ECC_FAIL;
    #else
		NC_CLR_DDR_MODE();
		if (pNandDrv->u8_IfECCTesting)
		{
			u32_Err = UNFD_ST_SUCCESS;
			goto EXIT;//return UNFD_ST_SUCCESS;
		}
		else
		{
			u32_Err = UNFD_ST_ERR_ECC_FAIL;
			goto EXIT;//return UNFD_ST_ERR_ECC_FAIL;
		}
        #endif
	}

    //-----------------------------------
    if(u8_RequireReadRetry && 0!=u8_RetryCnt_ECCFail)
	{
		S32 s32_ECCStatus;
		s32_ECCStatus = NC_GetECCBits();
		if(pNandDrv->au8_ID[0]!= 0xAD)
		NC_SendReadRetryCmd(0, 1);
//		if(u8_RetryCnt_ECCFail >= (u8_MaxReadRetryCount*3) / 4)
		if(s32_ECCStatus >= (pNandDrv->u16_BitflipThreshold))
			gint_IS_ECC_Fail = 2;
	}
	
#if defined(__VER_UNFD_FTL__) && __VER_UNFD_FTL__
#if UNFD_FTL_RD || BLD_LD_OS_RD
  #if UNFD_FTL_RD_TEST
	pNandDrv->u16_RD_TestCnt++;
	if(0 == (pNandDrv->u16_RD_TestCnt % 180))
	{
		pNandDrv->u16_RD_SrcPBA = u32_PhyRowIdx >> pNandDrv->u8_BlkPageCntBits;
	    pNandDrv->u8_RD_ECCBitsCnt = pNandDrv->u16_BitflipThreshold;
    }
  #else
	if( ((u16_Tmp & BIT_NC_ECC_MAX_BITS_MASK) >> BIT_NC_ECC_MAX_BITS_SHIFT) >= pNandDrv->u16_BitflipThreshold)

	{
		if(0 == pNandDrv->u8_RD_ActiveFlag)
		{
			pNandDrv->u16_RD_SrcPBA = u32_PhyRowIdx >> pNandDrv->u8_BlkPageCntBits;
			pNandDrv->u8_RD_ECCBitsCnt = (u16_Tmp & BIT_NC_ECC_MAX_BITS_MASK)
				>> BIT_NC_ECC_MAX_BITS_SHIFT;
		}
		else
			nand_debug(0,1,"NAND, RD, Active\n");

		if(0 != u16_Tmp)
		{
			nand_debug(0,1,"Reg.51h: %X  BitsCnt: %X \n", u16_Tmp, pNandDrv->u8_RD_ECCBitsCnt);
			nand_debug(0,1,"Blk: %X,  Page: %lX \n", pNandDrv->u16_RD_SrcPBA,
				u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask);
		}
	}
#endif
#endif
    #endif
	//-----------------------------------
	if(u8_IsSpareDMA)
	{
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
			memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
	}
	nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, READ_FROM_NAND);
    if (pu8_SpareBuf)
    {
        u16_Tmp = nand_CheckEmptyPage(pu8_SpareBuf);
    
    	if(u16_Tmp >0)
        {
    	    memset(pu8_DataBuf, 0xFF, pNandDrv->u16_PageByteCnt);
			memset(pu8_SpareBuf, 0xFF, pNandDrv->u16_SpareByteCnt);

    	    if(u16_Tmp ==2)
    	    {
//    		    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\n Spare area bit flips, Blk: %Xh  Page: %Xh\r\n",
//    			    u32_PhyRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PhyRowIdx&pNandDrv->u16_BlkPageCntMask);
    	    }
        }
    }
	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();
    
EXIT:
    #if defined( FCIE4_DDR_RETRY_DQS) && FCIE4_DDR_RETRY_DQS
	if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
		if(0!=u8_RetryDqs)
			nand_retry_dqs_post();
    #endif
	#if defined(FCIE_LFSR) && FCIE_LFSR
	if(pNandDrv->u8_RequireRandomizer)
		NC_EnableLFSR();
	#endif
	
	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalReadBytes_Hal += u32_PageCnt * pNandDrv->u16_PageByteCnt;
	#endif

	if(gu32_monitor_read_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_read_count++;
            gu64_monitor_read_size_kb += ((pNandDrv->u16_PageByteCnt / 1024 ) * u32_PageCnt );
        }

	return u32_Err;
}

U32 NC_PageCopy
(
    U32 u32_SrcPhyRowIdx, U32 u32_DstPhyRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt, U32 *pu32_DonePageCnt
)
{
	NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();
	U16 u16_Tmp = 0;
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr = 0, u32_SpareMapAddr = 0;
	U32 u32_Err, u32_i, u32_Ret;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);

	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_SET_DDR_MODE(); // to turn on ONFI clk
	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END | BIT_MMA_DATA_END);

	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_PageByteCnt * u32_PageCnt, READ_FROM_NAND );
	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, READ_FROM_NAND );

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
	#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
	#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
		}

		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					   0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);

			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_IDLE_CNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 3);

    for(u32_i=0; u32_i<u32_PageCnt; u32_i++)
    {
		REG_WRITE_UINT16(NC_JOB_BL_CNT, pNandDrv->u16_PageSectorCnt);
		REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
		REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
		u32_Err=NC_waitFifoClkReady();
		if(u32_Err != UNFD_ST_SUCCESS)
		{
			NC_CLR_DDR_MODE(); // to turn on ONFI clk
			return u32_Err;
		}
		REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

		REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_SrcPhyRowIdx & 0xFFFF);
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_SrcPhyRowIdx >> 16);
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_DstPhyRowIdx & 0xFFFF);
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_DstPhyRowIdx >> 16);
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);

		REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
		#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);
		#endif
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_PAGECOPY_US << 8) | ACT_WAIT_IDLE);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle << 8) | CMD_0x00);

		if (pNandDrv->u16_Reg48_Spare& BIT_NC_ONE_COL_ADDR) // if a page 512B
			REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | ACT_WAIT_IDLE);
		else
			REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x30);

		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAIT_IDLE << 8) | ACT_SER_DIN);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAIT_IDLE << 8) | ACT_WAIT_MMA);

		u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;
		REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x80 << 8) | ACT_PAGECOPY_DS);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DOUT << 8) | u16_Tmp);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x10);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_CHKSTATUS << 8) | CMD_0x70);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAIT_MMA << 8) | ACT_WAIT_IDLE);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_BREAK);
		nand_CheckPowerCut();

		REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS | BIT_NC_JOB_START));
        u32_SrcPhyRowIdx++;
        u32_DstPhyRowIdx++;
		if (NC_WaitComplete(BIT_NC_JOB_END | BIT_MMA_DATA_END,
		          (WAIT_WRITE_TIME + WAIT_READ_TIME)) ==
		  ((WAIT_WRITE_TIME + WAIT_READ_TIME)))
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadPages Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
			#if 0==IF_IP_VERIFY
			NC_ResetFCIE();
			NC_Config();
			NC_ResetNandFlash();
			#else
			//nand_stop();
			#endif
            u32_Ret = UNFD_ST_ERR_R_TIMEOUT;
            goto LABEL_END;
		}

        if (pu8_SpareBuf && IF_SPARE_DMA() == 0)
		{
			#if SPARE640B_CIFD512B_PATCH
		    if(pNandDrv->u16_SpareByteCnt > 0x200)
		        NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
		    else
		    #endif
			NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SpareByteCnt);
		}
        
		/* Check ECC */
		REG_READ_UINT16(NC_ECC_STAT0, u16_Tmp);
		if (u16_Tmp & BIT_NC_ECC_FAIL)
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadPages ECC Fail, Reg51: 0x%04x ", u16_Tmp);
			REG_READ_UINT16(NC_ECC_STAT1, u16_Tmp);
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "Reg52: 0x%04x ", u16_Tmp);
			REG_READ_UINT16(NC_ECC_STAT2, u16_Tmp);
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "Reg53: 0x%04x\n", u16_Tmp);
	        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Blk: %Xh  Page: %X\r\n",
		        u32_SrcPhyRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_SrcPhyRowIdx&pNandDrv->u16_BlkPageCntMask);
            u32_Ret = UNFD_ST_ERR_ECC_FAIL;
            goto LABEL_END;
		}		
    }
    LABEL_END:

	NC_CLR_DDR_MODE(); // to turn on ONFI clk
    *pu32_DonePageCnt = u32_i;
	return UNFD_ST_SUCCESS;
}


/*--------------------------------------------------
  get ECC corrected bits count

  return: 0xFFFFFFFF -> ECC uncorrectable error,
  other: max ECC corrected bits (within the readed sectors).
  --------------------------------------------------*/
U32  NC_GetECCBits(void)
{
	U16 u16_Tmp;

	REG_READ_UINT16(NC_ECC_STAT0, u16_Tmp);
	if (u16_Tmp & BIT_NC_ECC_FAIL)
		return(U32)(0-1);

	return(u16_Tmp & BIT_NC_ECC_MAX_BITS_MASK) >> 1;
}

U32  NC_ReadSectors_MTD(U32 u32_PhyRowIdx, U8 u8_SectorInPage, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_SectorCnt)
{
	U32 u32_Tmp, u32_SectorCnt_Tmp;
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr = 0, u32_SpareMapAddr = 0;
	U32 u32_Err = UNFD_ST_SUCCESS;
	#if IF_IP_VERIFY == 0
	U8	u8_RetryCnt_FifoClkRdy = 0;
	#endif
	#if defined(FCIE_LFSR) && FCIE_LFSR
	U8	u8_RandRetryFlag = 0;
	#endif
	U8	u8_RetryCnt_Events = 0, u8_RetryCnt_ECCFail = 0, u8_RetryDqs=0;
	U8	u8_MaxReadRetryCount = NAND_TIMEOUT_RETRY_CNT;
	U8	u8_RequireReadRetry = pNandDrv->u8_RequireReadRetry;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;
	U8 u8_EnableIRQ = 0;
	int i,j,ECCErrBitCnt;
	U16 u16_ecc_start_byte;

	//check if IRQ enable-> if yes disable irq check for read
	REG_READ_UINT16(NC_MIE_INT_EN, u16_Tmp);
	if( (u16_Tmp & (BIT_NC_JOB_END|BIT_MMA_DATA_END)) == 
		(BIT_NC_JOB_END|BIT_MMA_DATA_END) )
		u8_EnableIRQ = 1;

	if(pNandDrv->u8_RequireReadRetry)
		u8_MaxReadRetryCount = pNandDrv->ReadRetry_t.u8_MaxRetryTime + 1;

	u32_Tmp = ((u32_PhyRowIdx&pNandDrv->u16_BlkPageCntMask) << pNandDrv->u8_PageSectorCntBits)
	          + u8_SectorInPage + u32_SectorCnt;
	if(u32_Tmp > ((U32)pNandDrv->u16_BlkPageCnt << pNandDrv->u8_PageSectorCntBits))
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_ReadSectors, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_R_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_R_INVALID_PARAM;
	}

	// HW setting
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();

	#if IF_IP_VERIFY == 0
	nand_redo_read_sect:
	#endif

	NC_SET_DDR_MODE(); // to turn on ONFI clk
	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END|BIT_MIU_LAST_DONE);

	//-----------------------------------------------
	// read sectors
	u32_SectorCnt_Tmp = pNandDrv->u16_PageSectorCnt - u8_SectorInPage;
	u32_SectorCnt_Tmp = (u32_SectorCnt_Tmp > u32_SectorCnt) ? u32_SectorCnt : u32_SectorCnt_Tmp;

	if (u32_SectorCnt_Tmp)
	{
		//nand_debug(1, 1, "NC_ReadSectors read sectors: %u \r\n", u32_SectorCnt_Tmp);
		//nand_debug(1, 1, "%X %X %X %X %X \r\n",
		//  u32_PhyRowIdx, u8_SectorInPage, pu8_DataBuf, pu8_SpareBuf, u32_SectorCnt_Tmp);

		#if defined(FCIE_LFSR) && FCIE_LFSR
		REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
		REG_SET_BITS_UINT16(NC_LFSR_CTRL,
			((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
		#endif

		u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);

		u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt*u32_SectorCnt_Tmp, READ_FROM_NAND);


		if(u8_IsSpareDMA)
		{
			if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)		//main data miu1
			{
			#if defined(CONFIG_MIPS)
				printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
				nand_die();
			#endif
				pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			}
			if (pu8_SpareBuf)
			{
				u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			}
			else
			{
				u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			}
			REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
			REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
		}
		REG_WRITE_UINT16(NC_JOB_BL_CNT, u32_SectorCnt_Tmp);
		// if not 512B/page, set Partial Mode
		REG_READ_UINT16(NC_ECC_CTRL, u16_Tmp);
		if (u16_Tmp & BIT_NC_PAGE_SIZE_MASK)
		{
			REG_WRITE_UINT16(NC_PART_MODE, BIT_PARTIAL_MODE_EN);
			REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_CNT_MASK);
			REG_SET_BITS_UINT16(NC_PART_MODE, (u32_SectorCnt_Tmp-1)<<BIT_START_SECTOR_CNT_SHIFT);
			REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_IDX_MASK);
			REG_SET_BITS_UINT16(NC_PART_MODE, u8_SectorInPage<<BIT_START_SECTOR_IDX_SHIFT);
		}
		REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
		REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);

		u32_Err=NC_waitFifoClkReady();
		if(u32_Err != UNFD_ST_SUCCESS)
		{
			#if IF_IP_VERIFY == 0
			NC_ResetFCIE();
			NC_Config();
			NC_ResetNandFlash();
			nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);
			if(u8_IsSpareDMA)
			{
				nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
					memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
			}
			if(++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT)
				goto nand_redo_read_sect;
			else
			{
				//If soft reset still can not solve this problem, show an alert and return a error
				nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
				NC_DumpRegisters();
				NC_DumpDebugBus();
				NC_CLR_DDR_MODE();
				NC_UNLOCK_FCIE();
				goto EXIT;//return u32_Err;
			}
			#endif
			nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);
			if(u8_IsSpareDMA)
			{
				nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
					memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
			}

			NC_CLR_DDR_MODE();
			goto EXIT;//return u32_Err;
		}
		REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

		REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
		if(0 == pNandDrv->u8_WordMode)
			REG_WRITE_UINT16(NC_AUXREG_DAT, u8_SectorInPage << pNandDrv->u8_SectorByteCntBits);
		else
			REG_WRITE_UINT16(NC_AUXREG_DAT, u8_SectorInPage << (pNandDrv->u8_SectorByteCntBits-1));
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);
		if( pNandDrv->u16_bbm_wa == 0xBAD)
		{
			REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
			REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
			REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
		}

		REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
		#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);
		#endif
		if( pNandDrv->u16_bbm_wa == 0xBAD)
		{
			REG_WRITE_UINT16(NC_AUXREG_DAT, ((OP_ADR_CYCLE_00|OP_ADR_TYPE_ONE|OP_ADR_SET_1) << 8) | CMD_0x80);
		}
		REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x00));
		if (pNandDrv->u16_Reg48_Spare& BIT_NC_ONE_COL_ADDR) { // if a page 512B
			REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | ACT_WAITRB);
			REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_BREAK);
		} else {
			REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x30);
			REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_SER_DIN);
		}

		// check & set - patch for MIU busy case
		// slow down FCIE clock to wait for MIU service,
		// enhance the retry (more reliable) for error handling.
		if(0!=u8_RetryCnt_Events || 0!=u8_RetryCnt_ECCFail || 0!=u8_RetryDqs)
		{
			#if defined(FCIE4_DDR) && FCIE4_DDR
			if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
			{
				#if defined( FCIE4_DDR_RETRY_DQS) && FCIE4_DDR_RETRY_DQS
				if(0!=u8_RetryDqs)
				{
					nand_clock_setting(pNandDrv->u32_minClk);
					NC_FCIE4SetInterface(1, pNandDrv->tMinDDR.u8_DqsMode, pNandDrv->tMinDDR.u8_DelayCell, pNandDrv->tMinDDR.u8_DdrTiming);
				}
				#else
				nand_clock_setting(pNandDrv->u32_minClk);
                #if defined(FCIE4_DDR_EMMC_PLL) && FCIE4_DDR_EMMC_PLL
			    NC_FCIE4SetInterface_EMMC_PLL(1, pNandDrv->tDefaultDDR.u8_DqsMode,
				pNandDrv->tDefaultDDR.u8_DdrTiming);
			    #else
				NC_FCIE4SetInterface(1, pNandDrv->tMinDDR.u8_DqsMode, pNandDrv->tMinDDR.u8_DelayCell, pNandDrv->tMinDDR.u8_DdrTiming);
				#endif
				#endif
			}
			else
			#endif
			{
				if(pNandDrv->u8_RequireRandomizer == 0)
				{
					nand_clock_setting(FCIE3_SW_SLOWEST_CLK);
					REG_WRITE_UINT16(NC_WIDTH, 0);
					REG_WRITE_UINT16(NC_LATCH_DATA, (REG(NC_LATCH_DATA) & ~BIT_NC_LATCH_DATA_MASK));
				}
			}
			//nand_printf("check & set - patch for MIU busy case \n");
		}

		if(u8_EnableIRQ == 1)
			REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq for read

		REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START) & ~(BIT_NC_IF_DIR_W));
		if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME) == (WAIT_READ_TIME))
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadSectors Timeout, RetryCnt:%d, ErrCode:%Xh \r\n", u8_RetryCnt_Events, UNFD_ST_ERR_R_TIMEOUT);
			#if IF_IP_VERIFY == 0
		    if((gu8_DisableErrorLog == 0)&&(u8_RetryCnt_Events==(NAND_TIMEOUT_RETRY_CNT-1)))
		    {
			    NC_DumpDebugBus();
			    NC_DumpRegisters();
		    }
			NC_ResetFCIE();
			NC_Config();
			NC_ResetNandFlash();
			nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);
			if(u8_IsSpareDMA)
			{
				nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
					memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
			}
			if( ++u8_RetryCnt_Events < NAND_TIMEOUT_RETRY_CNT )
				goto nand_redo_read_sect;
			else
			{
				nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);

				//If soft reset still can not solve this problem, show an alert and return a error
				// restore the normal setting before return
				#if defined(FCIE4_DDR) && FCIE4_DDR
				if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
				{
					nand_clock_setting(pNandDrv->u32_Clk);
                    #if defined(FCIE4_DDR_EMMC_PLL) && FCIE4_DDR_EMMC_PLL
			        NC_FCIE4SetInterface_EMMC_PLL(1, pNandDrv->tDefaultDDR.u8_DqsMode,
				    pNandDrv->tDefaultDDR.u8_DdrTiming);
			        #else
					NC_FCIE4SetInterface(1, pNandDrv->tDefaultDDR.u8_DqsMode, pNandDrv->tDefaultDDR.u8_DelayCell, pNandDrv->tDefaultDDR.u8_DdrTiming);
                    #endif
				}
				else
				#endif
				{
					nand_clock_setting(pNandDrv->u32_Clk);
					REG_WRITE_UINT16(NC_WIDTH, FCIE_REG41_VAL);
					REG_WRITE_UINT16(NC_LATCH_DATA, pNandDrv->u16_Reg57_RELatch);
				}

				NC_CLR_DDR_MODE();
				REG_WRITE_UINT16(NC_PART_MODE, 0);
				if(u8_EnableIRQ == 1)
					REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq for read

				NC_UNLOCK_FCIE();
				u32_Err = UNFD_ST_ERR_R_TIMEOUT;
				goto EXIT;//return UNFD_ST_ERR_R_TIMEOUT;
			}
			#else
			//nand_stop();
			#endif
			nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);
			if(u8_IsSpareDMA)
			{
				nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
					memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
			}

			NC_CLR_DDR_MODE();
			if(u8_EnableIRQ == 1)
				REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq for read
			REG_WRITE_UINT16(NC_PART_MODE, 0);
			u32_Err = UNFD_ST_ERR_R_TIMEOUT;
			goto EXIT;//return UNFD_ST_ERR_R_TIMEOUT;
		}

		if(u8_EnableIRQ == 1)
			REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq for read
		// check until MIU is done
		#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
		u32_Err=NC_wait_MIULastDone();
		if( u32_Err != UNFD_ST_SUCCESS)
		{
			NC_ResetFCIE();
			NC_Config();
			NC_ResetNandFlash();
			nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);
			if(u8_IsSpareDMA)
			{
				nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
					memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
			}
			if( ++u8_RetryCnt_Events < NAND_TIMEOUT_RETRY_CNT )
				goto nand_redo_read_sect;
			else
			{
				nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
				NC_CLR_DDR_MODE();
				REG_WRITE_UINT16(NC_PART_MODE, 0);
				NC_UNLOCK_FCIE();
				goto EXIT;//return u32_Err;
			}

			NC_CLR_DDR_MODE();
			REG_WRITE_UINT16(NC_PART_MODE, 0);
			NC_UNLOCK_FCIE();
			goto EXIT;//return u32_Err;
		}
		#endif
		FLUSH_MIU_PIPE(); // Only used in U4 now
#ifdef CONFIG_MIPS_CMP
	_dma_cache_wback_inv(pu8_DataBuf, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp);


	if(u8_IsSpareDMA)
	{
		if(pu8_SpareBuf)
			_dma_cache_wback_inv(pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		else
			_dma_cache_wback_inv(pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt);
	}

#endif

#if defined(CONFIG_ARM) // ARM: AMBER3, AGATE
//	Chip_Flush_Cache_Range_VA_PA((U32)pu8_DataBuf,__pa(pu8_DataBuf),pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp);
#endif

		// restore - patch for MIU busy case
		// for retry ok, restore the normal setting
		if(0!=u8_RetryCnt_Events || 0!=u8_RetryCnt_ECCFail || 0!=u8_RetryDqs)
		{
			#if defined(FCIE4_DDR) && FCIE4_DDR
			if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
			{
				#if defined( FCIE4_DDR_RETRY_DQS) && FCIE4_DDR_RETRY_DQS
				if(0!=u8_RetryDqs)
				{
					nand_clock_setting(pNandDrv->u32_Clk);
					NC_FCIE4SetInterface(1, pNandDrv->tDefaultDDR.u8_DqsMode, pNandDrv->tDefaultDDR.u8_DelayCell, pNandDrv->tDefaultDDR.u8_DdrTiming);
				}
				#else
				nand_clock_setting(pNandDrv->u32_Clk);
                #if defined(FCIE4_DDR_EMMC_PLL) && FCIE4_DDR_EMMC_PLL
			    NC_FCIE4SetInterface_EMMC_PLL(1, pNandDrv->tDefaultDDR.u8_DqsMode,
				pNandDrv->tDefaultDDR.u8_DdrTiming);
			    #else
				NC_FCIE4SetInterface(1, pNandDrv->tDefaultDDR.u8_DqsMode, pNandDrv->tDefaultDDR.u8_DelayCell, pNandDrv->tDefaultDDR.u8_DdrTiming);
				#endif
				#endif
			}
			else
			#endif
			{
				nand_clock_setting(pNandDrv->u32_Clk);
				REG_WRITE_UINT16(NC_WIDTH, FCIE_REG41_VAL);
				REG_WRITE_UINT16(NC_LATCH_DATA, pNandDrv->u16_Reg57_RELatch);
			}
			//nand_printf("restore - patch for MIU busy case \n");
		}

        //---------------------------
        // get spare 
        if (pu8_SpareBuf && u8_IsSpareDMA == 0)
		{
            #if 0
			#if SPARE640B_CIFD512B_PATCH
			if(pNandDrv->u16_SpareByteCnt > 0x200)
				NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
			else
			#endif
            #endif
			NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SectorSpareByteCnt);
            //nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "dump spare: \n");
            //dump_mem(pu8_SpareBuf, 0x10);
		}

        //---------------------------
		// Check ECC
		REG_READ_UINT16(NC_ECC_STAT0, u16_Tmp);
		if (u16_Tmp & BIT_NC_ECC_FAIL)
		{
            #if IF_IP_VERIFY == 0
			// add retry for ECC error
			if( u8_RetryCnt_ECCFail < u8_MaxReadRetryCount)
			{
                //NOTICE:
				//Randomizer retry works only on pagecnt = 1, otherwise if there are empty and non-empty pages in a job.
				//ECC fail will always happen even the Randomizer is disabled.
				if(!pNandDrv->u8_RequireRandomizer)
				{
					if(u8_RetryCnt_ECCFail >= 1 && u8_RequireReadRetry)
						NC_SendReadRetryCmd(u8_RetryCnt_ECCFail - 1, 0);
					u8_RetryCnt_ECCFail++;
				}
				#if defined(FCIE_LFSR) && FCIE_LFSR
				else
				{
					if(!u8_RandRetryFlag)
					{
						NC_DisableLFSR();
						u8_RandRetryFlag = 1;
					}
					else
					{   nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "ECC Fail, RetryCnt: %Xh \n", u8_RetryCnt_ECCFail);
                        nand_debug(UNFD_DEBUG_LEVEL_LOW, 0, "Blk: %Xh  Page: %Xh \n",
        			    	u32_PhyRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PhyRowIdx&pNandDrv->u16_BlkPageCntMask);
				
						if(u8_RetryCnt_ECCFail >= 1 && u8_RequireReadRetry)
							NC_SendReadRetryCmd(u8_RetryCnt_ECCFail - 1, 0);

                        NC_EnableLFSR();
						u8_RetryCnt_ECCFail++;
					}
				}
				#endif
	    			goto nand_redo_read_sect;
			}
			#if defined(FCIE_LFSR) && FCIE_LFSR
			else if (u8_RetryCnt_ECCFail == u8_MaxReadRetryCount && pNandDrv->u8_RequireRandomizer == 1)
			{
				if(u8_RequireReadRetry && pNandDrv->au8_ID[0]!= 0xAD)
					NC_SendReadRetryCmd(0, 1);
				NC_DisableLFSR();
				u8_RetryCnt_ECCFail ++;
				goto nand_redo_read_sect;
			}
			#endif
			else
			{
				int falsealarm = 0;
				if(pu8_SpareBuf !=NULL)
					falsealarm = nand_CheckEmptySectorsFalseAlarm(pu8_DataBuf, pu8_SpareBuf, u32_SectorCnt);
				else
					falsealarm = nand_CheckEmptySectorsFalseAlarm(pu8_DataBuf, pNandDrv->PlatCtx_t.pu8_PageSpareBuf, u32_SectorCnt);

				if(falsealarm == 1)
				{
                    nand_debug(UNFD_DEBUG_LEVEL_LOW, 1, "FalseAlarm, RetryCnt: %Xh \n", u8_RetryCnt_ECCFail);
                    nand_debug(UNFD_DEBUG_LEVEL_LOW, 0, "Blk: %Xh  Page: %Xh \n",
    			    	u32_PhyRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PhyRowIdx&pNandDrv->u16_BlkPageCntMask);
                    
					gint_IS_ECC_Fail= 1;
					if(u8_RequireReadRetry && pNandDrv->u8_RequireRandomizer == 0  && pNandDrv->au8_ID[0]!= 0xAD)
					{
						NC_SendReadRetryCmd(0, 1);
					}
					//Clean False Alarm Status
					NC_ReConfig();
					NC_CLR_DDR_MODE();
					goto EXIT;
				}
				#if defined( FCIE4_DDR_RETRY_DQS) && FCIE4_DDR_RETRY_DQS
				if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
				{
					if(u8_RetryDqs == 0)
					{
						u8_RetryDqs = 1;
						u8_RetryCnt_ECCFail = 0;
						u8_RetryCnt_Events = 0;
						u8_RandRetryFlag = 0;

						if(u8_RequireReadRetry && pNandDrv->au8_ID[0]!= 0xAD)
						{
							NC_SendReadRetryCmd(0, 1);
						}

						#if defined(FCIE_LFSR) && FCIE_LFSR
						if(pNandDrv->u8_RequireRandomizer)
							NC_EnableLFSR();
						#endif
						goto nand_redo_read_sect;
					}
				}
				#endif

				nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: ECC Fail, Reg51:%04Xh ", u16_Tmp);
				nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "Error: Blk: %Xh  Page: %Xh\n",
			    	u32_PhyRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PhyRowIdx&pNandDrv->u16_BlkPageCntMask);

				if(u8_RequireReadRetry && pNandDrv->au8_ID[0]!= 0xAD)
				{
					NC_SendReadRetryCmd(0, 1);
				}
				NC_UNLOCK_FCIE();
			}

			nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);
			if(u8_IsSpareDMA)
			{
				nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
					memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
			}

			REG_WRITE_UINT16(NC_PART_MODE, 0);
			NC_CLR_DDR_MODE();
			u32_Err = UNFD_ST_ERR_ECC_FAIL;
			goto EXIT;//return UNFD_ST_ERR_ECC_FAIL;
			#else
			if (pNandDrv->u8_IfECCTesting)
			{
				u32_Err = UNFD_ST_SUCCESS;
				goto EXIT;//return UNFD_ST_SUCCESS;
			}
			else
			{
				u32_Err = UNFD_ST_ERR_ECC_FAIL;
				goto EXIT;//return UNFD_ST_ERR_ECC_FAIL;
			}
			#endif
		}

		if(u8_RequireReadRetry && 0 != u8_RetryCnt_ECCFail)
		{
			S32 s32_ECCStatus;
			s32_ECCStatus = NC_GetECCBits();
			if(pNandDrv->au8_ID[0]!= 0xAD)
			NC_SendReadRetryCmd(0, 1);
//			if(u8_RetryCnt_ECCFail >= (u8_MaxReadRetryCount*3) / 4)
			if(s32_ECCStatus >= (pNandDrv->u16_BitflipThreshold))
				gint_IS_ECC_Fail = 2;
		}
	    #if defined(__VER_UNFD_FTL__) && __VER_UNFD_FTL__
	      #if UNFD_FTL_RD || BLD_LD_OS_RD
	        #if UNFD_FTL_RD_TEST
			pNandDrv->u16_RD_TestCnt++;
			if(0 == (pNandDrv->u16_RD_TestCnt % 180))
			{
				pNandDrv->u16_RD_SrcPBA = u32_PhyRowIdx >> pNandDrv->u8_BlkPageCntBits;
			    pNandDrv->u8_RD_ECCBitsCnt = pNandDrv->u16_BitflipThreshold;
			}
		    #else
			if( ((u16_Tmp & BIT_NC_ECC_MAX_BITS_MASK) >> BIT_NC_ECC_MAX_BITS_SHIFT) >= pNandDrv->u16_BitflipThreshold)

			{
				if(0 == pNandDrv->u8_RD_ActiveFlag)
				{
					pNandDrv->u16_RD_SrcPBA = u32_PhyRowIdx >> pNandDrv->u8_BlkPageCntBits;
					pNandDrv->u8_RD_ECCBitsCnt = (u16_Tmp & BIT_NC_ECC_MAX_BITS_MASK)
						>> BIT_NC_ECC_MAX_BITS_SHIFT;
				}
				else
					nand_debug(0,1,"NAND, RD, Active\n");

				if(0 != u16_Tmp)
				{
					nand_debug(0,1,"Reg.51h: %X  BitsCnt: %X \n", u16_Tmp, pNandDrv->u8_RD_ECCBitsCnt);
					nand_debug(0,1,"Blk: %X,  Page: %lX \n", pNandDrv->u16_RD_SrcPBA,
					u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask);
			}
		}
		    #endif
		  #endif
	    #endif

		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}
	}

	#if 1
	i = 0;
	u16_ecc_start_byte = pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt;

	if (pu8_SpareBuf)
	{
	    for (i = 0; i < u32_SectorCnt_Tmp; i++)
	    {
            ECCErrBitCnt = 0;
    		for(j = u16_ecc_start_byte; j < pNandDrv->u16_SectorSpareByteCnt; j++)
    		{
    			if(i*pNandDrv->u16_SectorSpareByteCnt + j < 512)
    				ECCErrBitCnt += gau8_ZeroBitCnt[pu8_SpareBuf[ i*pNandDrv->u16_SectorSpareByteCnt + j]];
    		}
    		if(ECCErrBitCnt > pNandDrv->u16_ECCCorretableBit)
    		    break;
    	}

	    if(i == u32_SectorCnt_Tmp && u32_SectorCnt_Tmp >0)
	    {
            memset(pu8_DataBuf, 0xFF, pNandDrv->u16_SectorByteCnt*u32_SectorCnt_Tmp);
            memset(pu8_SpareBuf, 0xFF, pNandDrv->u16_SectorSpareByteCnt*u32_SectorCnt_Tmp);
	    }
	}
	#endif	

EXIT:
	#if defined( FCIE4_DDR_RETRY_DQS) && FCIE4_DDR_RETRY_DQS
	if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
		if(0!=u8_RetryDqs)
			nand_retry_dqs_post();
	#endif
    
	#if defined(FCIE_LFSR) && FCIE_LFSR
	if(pNandDrv->u8_RequireRandomizer)
		NC_EnableLFSR();
	#endif

    REG_WRITE_UINT16(NC_PART_MODE, 0);
    NC_CLR_DDR_MODE();

    #if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalReadBytes_Hal += u32_SectorCnt * pNandDrv->u16_SectorByteCnt;
	#endif
	if(gu32_monitor_read_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);
	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_read_count++;
            gu64_monitor_read_size += (pNandDrv->u16_SectorByteCnt * u32_SectorCnt);
        }
    
	return u32_Err;
}

U32  NC_ReadSectors(U32 u32_PhyRowIdx, U8 u8_SectorInPage, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_SectorCnt)
{
	U32 u32_Tmp, u32_SectorCnt_Tmp;
	U16 u16_PageCnt, u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr= 0, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;

	u32_Tmp = ((u32_PhyRowIdx&pNandDrv->u16_BlkPageCntMask) << pNandDrv->u8_PageSectorCntBits)
	          + u8_SectorInPage + u32_SectorCnt;
	if(u32_Tmp > ((U32)pNandDrv->u16_BlkPageCnt << pNandDrv->u8_PageSectorCntBits))
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_ReadSectors, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_R_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_R_INVALID_PARAM;
	}

	// HW setting
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE(); // to turn on ONFI clk

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	//-----------------------------------------------
	// read sectors
	u32_SectorCnt_Tmp = pNandDrv->u16_PageSectorCnt - u8_SectorInPage;
	u32_SectorCnt_Tmp = (u32_SectorCnt_Tmp > u32_SectorCnt) ? u32_SectorCnt : u32_SectorCnt_Tmp;

	if (u32_SectorCnt_Tmp)
	{
		//nand_debug(1, 1, "NC_ReadSectors read sectors: %u \r\n", u32_SectorCnt_Tmp);
		//nand_debug(1, 1, "%X %X %X %X %X \r\n",
		//	u32_PhyRowIdx, u8_SectorInPage, pu8_DataBuf, pu8_SpareBuf, u32_SectorCnt_Tmp);

		#if defined(FCIE_LFSR) && FCIE_LFSR
		REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
		REG_SET_BITS_UINT16(NC_LFSR_CTRL,
			((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
		#endif

		u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);

		u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt*u32_SectorCnt_Tmp, READ_FROM_NAND);

		if(u8_IsSpareDMA)
		{
			if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)		//main data miu1
			{
			#if defined(CONFIG_MIPS)
				printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
				nand_die();
			#endif
				pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			}

			if (pu8_SpareBuf)
			{
				u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			}
			else
			{
				u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			}
			REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
			REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
		}
		REG_WRITE_UINT16(NC_JOB_BL_CNT, u32_SectorCnt_Tmp);
		// if not 512B/page, set Partial Mode
		REG_READ_UINT16(NC_ECC_CTRL, u16_Tmp);
		if (u16_Tmp & BIT_NC_PAGE_SIZE_MASK)
		{
			REG_WRITE_UINT16(NC_PART_MODE, BIT_PARTIAL_MODE_EN);
			REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_CNT_MASK);
			REG_SET_BITS_UINT16(NC_PART_MODE, (u32_SectorCnt_Tmp-1)<<BIT_START_SECTOR_CNT_SHIFT);
			REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_IDX_MASK);
			REG_SET_BITS_UINT16(NC_PART_MODE, u8_SectorInPage<<BIT_START_SECTOR_IDX_SHIFT);
		}
		REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
		REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);

		if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
		{
			nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);
			if(u8_IsSpareDMA)
			{
				nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
					memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
			}

			NC_CLR_DDR_MODE();
			return u32_Err;
		}
		REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

		REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
		if(0 == pNandDrv->u8_WordMode)
		    REG_WRITE_UINT16(NC_AUXREG_DAT, u8_SectorInPage << pNandDrv->u8_SectorByteCntBits);
		else
			REG_WRITE_UINT16(NC_AUXREG_DAT, u8_SectorInPage << (pNandDrv->u8_SectorByteCntBits-1));
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);
		if( pNandDrv->u16_bbm_wa == 0xBAD)
		{
			REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
			REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
			REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
		}

		REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
		#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);
		#endif
		if( pNandDrv->u16_bbm_wa == 0xBAD)
		{
			REG_WRITE_UINT16(NC_AUXREG_DAT, ((OP_ADR_CYCLE_00|OP_ADR_TYPE_ONE|OP_ADR_SET_1) << 8) | CMD_0x80);
		}
		REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x00));
		if (pNandDrv->u16_Reg48_Spare& BIT_NC_ONE_COL_ADDR) { // if a page 512B
			REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | ACT_WAITRB);
			REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_BREAK);
		} else {
			REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x30);
			REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_SER_DIN);
		}

		REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START) & ~(BIT_NC_IF_DIR_W));
		if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME) == (WAIT_READ_TIME))
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadSectors Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
            #if 0==IF_IP_VERIFY
		    if(gu8_DisableErrorLog == 0)
		    {
			    NC_DumpDebugBus();
			    NC_DumpRegisters();
		    }
			NC_ResetFCIE();
			NC_Config();
			NC_CLR_DDR_MODE();
			NC_ResetNandFlash();
			#else
		    //nand_stop();
		    #endif
			nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);
			if(u8_IsSpareDMA)
			{
				nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
					memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
			}

			REG_WRITE_UINT16(NC_PART_MODE, 0);
			return UNFD_ST_ERR_R_TIMEOUT;
		}

		FLUSH_MIU_PIPE(); // Only used in U4 now

		#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
		// check until MIU is done
		if( (u32_Err=NC_wait_MIULastDone()) != UNFD_ST_SUCCESS)
		{
			NC_ResetFCIE();
			NC_Config();
			NC_CLR_DDR_MODE();
			nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);
			if(u8_IsSpareDMA)
			{
				nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
					memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
			}

			NC_UNLOCK_FCIE();
			return u32_Err;
		}
		#endif
#if defined(CONFIG_ARM) // ARM: AMBER3, AGATE
//	Chip_Flush_Cache_Range_VA_PA((U32)pu8_DataBuf,__pa(pu8_DataBuf),pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp);
        #endif

        #if 0==IF_SPARE_AREA_DMA
		if (pu8_SpareBuf)
			NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SectorSpareByteCnt);
		#endif
            
		// Check ECC
		REG_READ_UINT16(NC_ECC_STAT0, u16_Tmp);
		if (u16_Tmp & BIT_NC_ECC_FAIL)
		{
			//nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadSectors ECC Fail, Reg51:%04Xh ", u16_Tmp);
			//REG_READ_UINT16(NC_ECC_STAT1, u16_Tmp);
			//nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "Reg52:%04Xh ", u16_Tmp);
			//REG_READ_UINT16(NC_ECC_STAT2, u16_Tmp);
			//nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "Reg53:%04Xh \r\n", u16_Tmp);
			//nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Blk: %lXh  Page: %lXh\r\n",
			//	u32_PhyRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PhyRowIdx&pNandDrv->u16_BlkPageCntMask);
			nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);
			if(u8_IsSpareDMA)
			{
				nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
				if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
					memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
			}


			NC_CLR_DDR_MODE();
			REG_WRITE_UINT16(NC_PART_MODE, 0);
            #if 0 == IF_IP_VERIFY
			return UNFD_ST_ERR_ECC_FAIL;
            #else
			if (pNandDrv->u8_IfECCTesting)
				return UNFD_ST_SUCCESS;
			else
				return UNFD_ST_ERR_ECC_FAIL;
            #endif
		}

// -------------------------------------
		#if defined(__VER_UNFD_FTL__) && __VER_UNFD_FTL__
          #if UNFD_FTL_RD || BLD_LD_OS_RD
            #if UNFD_FTL_RD_TEST
	    pNandDrv->u16_RD_TestCnt++;
	    if(0 == (pNandDrv->u16_RD_TestCnt % 180))
	    {
		    pNandDrv->u16_RD_SrcPBA = u32_PhyRowIdx >> pNandDrv->u8_BlkPageCntBits;
	        pNandDrv->u8_RD_ECCBitsCnt = pNandDrv->u16_BitflipThreshold;
        }
            #else
	    if( ((u16_Tmp & BIT_NC_ECC_MAX_BITS_MASK) >> BIT_NC_ECC_MAX_BITS_SHIFT) >= pNandDrv->u16_BitflipThreshold)
    	{
		    if(0 == pNandDrv->u8_RD_ActiveFlag)
		    {
			    pNandDrv->u16_RD_SrcPBA = u32_PhyRowIdx >> pNandDrv->u8_BlkPageCntBits;
			    pNandDrv->u8_RD_ECCBitsCnt = (u16_Tmp & BIT_NC_ECC_MAX_BITS_MASK)
				    >> BIT_NC_ECC_MAX_BITS_SHIFT;
		    }
		    else
			    nand_debug(0,1,"NAND, RD, Active\n");

		    if(0 != u16_Tmp)
		    {
			    nand_debug(0,1,"Reg.51h: %X  BitsCnt: %X \n", u16_Tmp, pNandDrv->u8_RD_ECCBitsCnt);
			    nand_debug(0,1,"Blk: %X,  Page: %lX \n", pNandDrv->u16_RD_SrcPBA,
				    u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask);
		    }
	    }
	        #endif
          #endif
        #endif

        // -------------------------------------
        nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt_Tmp, READ_FROM_NAND);
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}

		REG_WRITE_UINT16(NC_PART_MODE, 0);
	}

	//-----------------------------------------------
	// read pages
	u32_SectorCnt -= u32_SectorCnt_Tmp;
	if (0 == u32_SectorCnt)
	{
		NC_CLR_DDR_MODE();
		return UNFD_ST_SUCCESS;
	}

	u32_PhyRowIdx += 1;
	u16_PageCnt = u32_SectorCnt >> pNandDrv->u8_PageSectorCntBits;

	pu8_DataBuf += u32_SectorCnt_Tmp << pNandDrv->u8_SectorByteCntBits;
	if (pu8_SpareBuf)
		//pu8_SpareBuf += u32_SectorCnt_Tmp << pNandDrv->u8_SectorSpareByteCntBits;
		pu8_SpareBuf += u32_SectorCnt_Tmp * pNandDrv->u16_SectorSpareByteCnt;

	if (u16_PageCnt)
	{
		//nand_debug(1, 1, "NC_ReadSectors read pages %X %X %X %X \r\n",
		//	u32_PhyRowIdx, pu8_DataBuf, pu8_SpareBuf, u16_PageCnt);

		u32_Tmp = NC_ReadPages(u32_PhyRowIdx, pu8_DataBuf, pu8_SpareBuf, u16_PageCnt);
		if (UNFD_ST_SUCCESS != u32_Tmp)
		{
			NC_CLR_DDR_MODE();
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadSectors read pages fail, ErrCode:%Xh \r\n", u32_Tmp);
			return u32_Tmp;
		}
	}

	//-----------------------------------------------
	// read sectors
	u32_SectorCnt_Tmp = u16_PageCnt << pNandDrv->u8_PageSectorCntBits;
	pu8_DataBuf += u32_SectorCnt_Tmp << pNandDrv->u8_SectorByteCntBits;
	if (pu8_SpareBuf)
		//pu8_SpareBuf += u32_SectorCnt_Tmp << pNandDrv->u8_SectorSpareByteCntBits;
		pu8_SpareBuf += u32_SectorCnt_Tmp * pNandDrv->u16_SectorSpareByteCnt;

	u32_SectorCnt -= u32_SectorCnt_Tmp;
	u32_PhyRowIdx += u16_PageCnt;
	if (u32_SectorCnt)
	{
		//nand_debug(1, 1, "NC_ReadSectors read last sectors %X %X %X %X \r\n",
		//	u32_PhyRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_SectorCnt);

		u32_Tmp = NC_ReadSectors(u32_PhyRowIdx, 0, pu8_DataBuf, pu8_SpareBuf, u32_SectorCnt);
		if (UNFD_ST_SUCCESS != u32_Tmp)
		{
			NC_CLR_DDR_MODE();
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadSectors read last sectors fail, ErrCode:%Xh \r\n", u32_Tmp);
			return u32_Tmp;
		}
	}

	NC_CLR_DDR_MODE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalReadBytes_Hal += u32_SectorCnt * pNandDrv->u16_SectorByteCnt;
	#endif

	if(gu32_monitor_read_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_read_count++;
            gu64_monitor_read_size += (pNandDrv->u16_SectorByteCnt * u32_SectorCnt);
        }

	return UNFD_ST_SUCCESS;
}

U32  NC_WriteSectors
(
    U32 u32_PhyRowIdx, U8 u8_SectorInPage, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_SectorCnt
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr = 0, u32_DataMapAddr;
	U32 u32_SpareDMAAddr, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8	*pu8_OrigSpareBuf = pu8_SpareBuf;

	if(u8_SectorInPage + u32_SectorCnt > pNandDrv->u16_PageSectorCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR, NAND, over one page sector cnt\n");
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "%X %X %X %X %X\n",
			u32_PhyRowIdx, u8_SectorInPage, (U32)pu8_DataBuf, (U32)pu8_SpareBuf, u32_SectorCnt);
		return UNFD_ST_ERR_INVALID_PARAM;
	}
	if(0 == u32_SectorCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR, NAND, zero sector cnt\n");
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "%X %X %X %X %X\n",
			u32_PhyRowIdx, u8_SectorInPage, (U32)pu8_DataBuf, (U32)pu8_SpareBuf, u32_SectorCnt);
		return UNFD_ST_ERR_INVALID_PARAM;
	}
    #if CHK_EMPTY_RB4W
    {
		NC_ReadPages(u32_PhyRowIdx, au8_DataReadBuf, au8_SpareReadBuf, 1);
		if(0==nand_CheckEmptyPageFalseAlarm(au8_DataReadBuf, au8_SpareReadBuf))
		{
			nand_debug(0, 1, "Write no blank page row %Xh\n", u32_PhyRowIdx);
            dump_stack();
			nand_die();
		}
	}
    #endif
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE();
	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, WRITE_TO_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, WRITE_TO_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
			#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
			#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			if(pu8_OrigSpareBuf != NULL)
				memcpy(pu8_SpareBuf, pu8_OrigSpareBuf, pNandDrv->u16_SpareByteCnt);
			else
				memset(pu8_SpareBuf, 0xFF, pNandDrv->u16_SpareByteCnt);
		}

		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					   0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);

			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_JOB_BL_CNT, u32_SectorCnt);

	// if Partial Mode (if not 512B/page, set Partial Mode)
	REG_READ_UINT16(NC_ECC_CTRL, u16_Tmp);
	if (u16_Tmp & BIT_NC_PAGE_SIZE_MASK)
	{
		REG_WRITE_UINT16(NC_PART_MODE, BIT_PARTIAL_MODE_EN);
		REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_CNT_MASK);
		REG_SET_BITS_UINT16(NC_PART_MODE, (u32_SectorCnt-1)<<BIT_START_SECTOR_CNT_SHIFT);
		REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_IDX_MASK);
		REG_SET_BITS_UINT16(NC_PART_MODE, u8_SectorInPage<<BIT_START_SECTOR_IDX_SHIFT);
	}
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);//>>MIU_BUS_WIDTH_BITS));
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);//(MIU_BUS_WIDTH_BITS+16)));
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);//|BIT_NC_MIU_BURST_8);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, WRITE_TO_NAND);

		NC_CLR_DDR_MODE();
		REG_WRITE_UINT16(NC_PART_MODE, 0);
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	if(0 == pNandDrv->u8_WordMode)
	    REG_WRITE_UINT16(NC_AUXREG_DAT, u8_SectorInPage << pNandDrv->u8_SectorByteCntBits);
	else
		REG_WRITE_UINT16(NC_AUXREG_DAT, u8_SectorInPage << (pNandDrv->u8_SectorByteCntBits-1));
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x10 << 8) | ACT_SER_DOUT);
	if(pNandDrv->u8_SwPatchWaitRb== 0)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x70 << 8)| ACT_WAITRB);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_CHKSTATUS);
	}
	else
    	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_WAITRB);

	if(IF_SPARE_DMA() == 0)
	{
		if (pu8_SpareBuf)
			NC_SetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SectorSpareByteCnt-pNandDrv->u16_ECCCodeByteCnt);
		else
			NC_SetCIFD_Const(0xFF, 0, pNandDrv->u16_SectorSpareByteCnt-pNandDrv->u16_ECCCodeByteCnt);
	}

	nand_CheckPowerCut();

	REG_WRITE_UINT16(NC_CTRL,
					 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WriteSectors Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		if(gu8_DisableErrorLog == 0)
		{
		    #if 0==IF_IP_VERIFY
			NC_DumpDebugBus();
			NC_DumpRegisters();
			#endif
		}
		REG_WRITE_UINT16(NC_PART_MODE, 0);
        #if 0==IF_IP_VERIFY
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, WRITE_TO_NAND);
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
        #else
		//nand_stop();
		#endif
		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}

	if(pNandDrv->u8_SwPatchWaitRb == 1)
	{
	    NC_RECONFIG();
		REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

		REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
		#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
		#endif
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_CHKSTATUS << 8)| CMD_0x70);
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
		nand_CheckPowerCut();

		REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);
		if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WriteSectors Timeout 1, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
			if(gu8_DisableErrorLog == 0)
			{
	            #if 0==IF_IP_VERIFY
				NC_DumpDebugBus();
				NC_DumpRegisters();
	            #endif
			}
			REG_WRITE_UINT16(NC_PART_MODE, 0);
	        #if 0==IF_IP_VERIFY
			if(u8_IsSpareDMA)
				nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);			
			nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, WRITE_TO_NAND);			
			NC_ResetFCIE();
			//pNandDrv->u16_Reg48_Spare |= BIT_NC_SPARE_DEST_MIU;
			pNandDrv->u16_Reg48_Spare &= ~BIT_NC_RANDOM_RW_OP_EN;
			NC_Config();
			NC_CLR_DDR_MODE();
			NC_ResetNandFlash();
	        #else
			//nand_stop();
			#endif
			return UNFD_ST_ERR_W_TIMEOUT; // timeout
		}
	}
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);			
	nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, WRITE_TO_NAND);

	NC_CLR_DDR_MODE();
	REG_WRITE_UINT16(NC_PART_MODE, 0);

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_SectorCnt * pNandDrv->u16_SectorByteCnt;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_write_count++;
            gu64_monitor_write_size += pNandDrv->u16_SectorByteCnt * u32_SectorCnt;
        }

	return NC_CheckEWStatus(OPTYPE_WRITE);
}

U32  NC_ReadSector_RIUMode
(
    U32 u32_PhyRowIdx, U8 u8_SectorInPage, U8 *pu8_DataBuf, U8 *pu8_SpareBuf
)
{
	volatile U16 u16_Tmp, u16_i, u16_j, u16_k, u16_Cnt, u16_ByteLoc;
	U16 u16_Col;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U16 *pu16_DataBuf = (U16*)pu8_DataBuf;
	U32 u32_Err, u32_Tmp;

	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE();

	if(IF_SPARE_DMA())
		REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare &
			~(BIT_NC_SPARE_DEST_MIU|BIT_NC_RANDOM_RW_OP_EN));

	REG_SET_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN); // enable RIU Mode
	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_R2N_ECC|BIT_NC_R2N_RDY|BIT_NC_JOB_END|BIT_MMA_DATA_END);

	// if Partial Mode (if not 512B/page, set Partial Mode)
	REG_READ_UINT16(NC_ECC_CTRL, u16_Tmp);
	if (u16_Tmp & BIT_NC_PAGE_SIZE_MASK) {
		REG_WRITE_UINT16(NC_PART_MODE, BIT_PARTIAL_MODE_EN);
		REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_IDX_MASK);
		REG_SET_BITS_UINT16(NC_PART_MODE, u8_SectorInPage<<BIT_START_SECTOR_IDX_SHIFT);
	}
	REG_WRITE_UINT16(NC_JOB_BL_CNT, 1);	/* read 1 sector only */
	//REG_WRITE_UINT16(NC_SDIO_ADDR0, 0);
	//REG_WRITE_UINT16(NC_SDIO_ADDR1, 0);
	REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	if(0 == pNandDrv->u8_WordMode)
	    u16_Col = u8_SectorInPage << pNandDrv->u8_SectorByteCntBits;
	else
		u16_Col = u8_SectorInPage << (pNandDrv->u8_SectorByteCntBits-1);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u16_Col);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);
	if( pNandDrv->u16_bbm_wa == 0xBAD)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	}

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	if( pNandDrv->u16_bbm_wa == 0xBAD)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, ((OP_ADR_CYCLE_00|OP_ADR_TYPE_ONE|OP_ADR_SET_1) << 8) | CMD_0x80);
	}
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x00));
	if (pNandDrv->u16_Reg48_Spare & BIT_NC_ONE_COL_ADDR)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | ACT_WAITRB);
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
	}
	else
	{	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x30);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_SER_DIN);
	}

	REG_WRITE_UINT16(NC_CTRL,
					 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START)&~(BIT_NC_IF_DIR_W));

	u16_Cnt = pNandDrv->u16_SectorByteCnt >> 9;	// how many 512B per sector
	for (u16_j=0; u16_j<u16_Cnt; u16_j++)
	{
		if (NC_WaitComplete_RM(BIT_RIU_RDY_MMA,
							   pNandDrv->u8_IDByteCnt ? WAIT_READ_TIME : WAIT_PROBE_TIME)==
			(pNandDrv->u8_IDByteCnt ? WAIT_READ_TIME : WAIT_PROBE_TIME))
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Read_RM timeout 0, ErrCode:%Xh\r\n", UNFD_ST_ERR_R_TIMEOUT_RM);
			if(gu8_DisableErrorLog == 0)
			{
				NC_DumpDebugBus();
				NC_DumpRegisters();
			}
			REG_WRITE_UINT16(NC_PART_MODE, 0);
			REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
            #if 0 == IF_IP_VERIFY
			REG_CLR_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);
			REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
			NC_ResetFCIE();
			NC_Config();
			NC_CLR_DDR_MODE();
			//drvNAND_ResetNandFlash(); // save code size
            #else
			//nand_stop();
		    #endif
			return UNFD_ST_ERR_R_TIMEOUT_RM; // timeout
		}

		if (0 == u16_j)
			REG_SET_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_DI_START);

		u16_k = u16_j << 8;

		for (u16_i=0; u16_i<256; u16_i++)
		{
			if (NC_WaitComplete(BIT_NC_R2N_RDY, WAIT_READ_TIME) == WAIT_READ_TIME)
			{
				nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Read_RM timeout 1, ErrCode:%Xh\r\n", UNFD_ST_ERR_R_TIMEOUT_RM);
				if(gu8_DisableErrorLog == 0)
				{
				    #if 0 == IF_IP_VERIFY
					NC_DumpDebugBus();
					NC_DumpRegisters();
					#endif
				}
				REG_WRITE_UINT16(NC_PART_MODE, 0);
				REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
                #if 0 == IF_IP_VERIFY
				REG_CLR_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);
				REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
				NC_ResetFCIE();
				NC_Config();
				NC_CLR_DDR_MODE();
				//NC_ResetNandFlash(); // save code size
				#else
				
				#endif
				return UNFD_ST_ERR_R_TIMEOUT_RM; // timeout
			}
			REG_READ_UINT16(NC_R2N_DATA_RD, pu16_DataBuf[u16_k + u16_i]);
			REG_SET_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_DI_EN);
		}
	}
	REG_WRITE_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN|BIT_R2N_DI_END);

	// wait for events
	u32_Tmp = 0;
	do{
		u16_Tmp = 0; // do not remove this line
		REG_READ_UINT16(NC_MIE_EVENT, u16_Tmp);

		nand_hw_timer_delay(HW_TIMER_DELAY_1us);
		u32_Tmp++;
		if(u32_Tmp > WAIT_READ_TIME)
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR,1,"Error: Read_RM timeout 2, ErrCode:%X\n", UNFD_ST_ERR_R_TIMEOUT_RM);
			REG_CLR_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);
			REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
			NC_ResetFCIE();
			NC_Config();
			NC_CLR_DDR_MODE();
			return UNFD_ST_ERR_R_TIMEOUT_RM;
		}
	}while(((u16_Tmp & BIT_NC_R2N_ECC)==0) && ((u16_Tmp & (BIT_NC_JOB_END | BIT_MMA_DATA_END)) != (BIT_NC_JOB_END | BIT_MMA_DATA_END)));

	// if ECC Fail
	REG_READ_UINT16(NC_ECC_STAT2, u16_Tmp);
	if((u16_Tmp & BIT_NC_ECC_FLGA_MASK) == BIT_NC_ECC_FLGA_FAIL)
	{
		REG_CLR_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);
		REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
		NC_ResetFCIE();
		NC_Config();

		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Read_RM ECC Fail, ErrCode:%Xh \r\n", UNFD_ST_ERR_ECC_FAIL_RM);
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Blk: %Xh  Page: %X\r\n",
			u32_PhyRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PhyRowIdx&pNandDrv->u16_BlkPageCntMask);
		return UNFD_ST_ERR_ECC_FAIL_RM;
	}

	REG_READ_UINT16(NC_MIE_EVENT, u16_Tmp);
	if (u16_Tmp & BIT_NC_R2N_ECC)
	{
		//nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "RIU Mode, ECC happens: ");
		// make CIFD available for SW
		// [CAUTION]: before clear BIT_NC_R2N_ECC, JOB_END would not set.
		REG_W1C_BITS_UINT16(NC_MIE_EVENT, BIT_NC_R2N_ECC);
		if(NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME) == WAIT_READ_TIME)
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR,1,"Error: Read_RM timeout 3, ErrCode:%X\n", UNFD_ST_ERR_R_TIMEOUT_RM);
			if(gu8_DisableErrorLog == 0)
			{
				NC_DumpDebugBus();
				NC_DumpRegisters();
			}
			REG_CLR_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);
			REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
			NC_ResetFCIE();
			NC_Config();
			NC_CLR_DDR_MODE();
		}

		// SW correct ECC bits
		if(pu8_SpareBuf)
			NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SectorSpareByteCnt-pNandDrv->u16_ECCCodeByteCnt);

		REG_READ_UINT16(NC_ECC_STAT2, u16_Tmp);
		u16_Cnt = (u16_Tmp & BIT_NC_ECC_CNT_MASK) >> BIT_NC_ECC_CNT_SHIFT;
		//nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "%u bits\r\n", u16_Cnt);
		for (u16_i=0; u16_i<u16_Cnt; u16_i++)
		{
			REG_CLR_BITS_UINT16(NC_ECC_STAT2, BIT_NC_ECC_SEL_LOC_MASK);
			REG_SET_BITS_UINT16(NC_ECC_STAT2, u16_i<<BIT_NC_ECC_SEL_LOC_SHIFT);
			LABEL_ECC_LOC_READ:
			//REG_READ_UINT16(NC_ECC_LOC, u16_Tmp);
			REG_READ_UINT16(NC_ECC_LOC, u16_j);
			REG_READ_UINT16(NC_ECC_LOC, u16_k);
			if(u16_j != u16_k)
				goto LABEL_ECC_LOC_READ;
			//nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "bit loc: %02Xh \r\n", u16_j);
			u16_ByteLoc = u16_j >> 3;
			if (u16_ByteLoc < pNandDrv->u16_SectorByteCnt)
			{
				pu8_DataBuf[u16_ByteLoc] ^= 1<<(u16_j%8);

			}
			else if (u16_ByteLoc < pNandDrv->u16_SectorByteCnt +
					   pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt)
			{
				pu8_SpareBuf[u16_ByteLoc-pNandDrv->u16_SectorByteCnt] ^= 1<<(u16_j%8);
			}
		}

		u16_Tmp = BIT_NC_R2N_ECC;
	}

	if(0 == (u16_Tmp & BIT_NC_R2N_ECC)){
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME) == WAIT_READ_TIME)
	{
		U16 u16_Reg;
		REG_READ_UINT16(NC_MIE_EVENT, u16_Reg);
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Read_RM timeout 2, ErrCode:%Xh, Reg00h:%Xh\r\n",
				   UNFD_ST_ERR_R_TIMEOUT_RM, u16_Reg);
		if(gu8_DisableErrorLog == 0)
		{
	        #if 0 == IF_IP_VERIFY
			NC_DumpDebugBus();
			NC_DumpRegisters();
	        #endif
		}
		REG_WRITE_UINT16(NC_PART_MODE, 0);
		REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);

        #if 0 == IF_IP_VERIFY
		REG_CLR_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);
		REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		//NC_ResetNandFlash(); // save code size
        #else
		//nand_stop();
		#endif
		return UNFD_ST_ERR_R_TIMEOUT_RM; // timeout
	}

	NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SectorSpareByteCnt-pNandDrv->u16_ECCCodeByteCnt);
	}

	NC_CLR_DDR_MODE();
	REG_WRITE_UINT16(NC_PART_MODE, 0);
	REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
	REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalReadBytes_Hal += pNandDrv->u16_SectorByteCnt;
	#endif

	if(gu32_monitor_read_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_read_count++;
            gu64_monitor_read_size += pNandDrv->u16_SectorByteCnt;
        }

	return UNFD_ST_SUCCESS;
}

U32  NC_WritePage_RIUMode
(
    U32 u32_PhyRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf
)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U16 u16_i, u16_j, u16_Cnt, *pu16_DataBuf=(U16*)pu8_DataBuf;
	U32 u32_Err;

	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE();

	if(IF_SPARE_DMA())
		REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare &
			~(BIT_NC_SPARE_DEST_MIU|BIT_NC_RANDOM_RW_OP_EN));

	REG_SET_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_R2N_ECC|BIT_NC_R2N_RDY|BIT_NC_JOB_END|BIT_MMA_DATA_END);

	REG_WRITE_UINT16(NC_JOB_BL_CNT, pNandDrv->u16_PageSectorCnt);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, 0);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, 0);
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W|BIT_NC_MIU_BURST_8);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle << 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x10 << 8) | ACT_SER_DOUT);
	if(pNandDrv->u8_SwPatchWaitRb == 0)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x70 << 8)| ACT_WAITRB);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_CHKSTATUS);
	}
	else
    	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_WAITRB);

	for (u16_i=0; u16_i < pNandDrv->u16_PageSectorCnt; u16_i++)
	{
		#if SPARE640B_CIFD512B_PATCH
		if((u16_i+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
			break;
		#endif
		NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_i,
				   pNandDrv->u16_SectorSpareByteCnt * u16_i,
				   pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
	}
	nand_CheckPowerCut();
	REG_WRITE_UINT16(NC_CTRL,
					 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	//---------------------------------------------------
	// fill in DO
	for (u16_i=0; u16_i<pNandDrv->u16_PageSectorCnt; u16_i++)
	{	// sector loop
		for (u16_j=0; u16_j<pNandDrv->u16_SectorByteCnt>>9; u16_j++)
		{ // 512 byte loop
			if (NC_WaitComplete_RM(BIT_RIU_RDY_MMA, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
			{
				nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Write_RM timeout 0, ErrCode:%Xh\r\n", UNFD_ST_ERR_W_TIMEOUT_RM);
				if(gu8_DisableErrorLog == 0)
				{
					NC_DumpDebugBus();
					NC_DumpRegisters();
				}
				#if 0 == IF_IP_VERIFY
				REG_CLR_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);
				REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
				NC_ResetFCIE();
				NC_Config();
				NC_CLR_DDR_MODE();
				//NC_ResetNandFlash(); // save code size
				#else
				//nand_stop();
				#endif
				return UNFD_ST_ERR_W_TIMEOUT_RM; // timeout
			}

			if (0==u16_j)
				REG_SET_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_DO_START);

			for (u16_Cnt=0; u16_Cnt<256; u16_Cnt++)
			{
				if (NC_WaitComplete(BIT_NC_R2N_RDY, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
				{
					nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Write_RM timeout 2, ErrCode:%Xh\r\n",
							   UNFD_ST_ERR_W_TIMEOUT_RM);
					#if 0 == IF_IP_VERIFY
		            if(gu8_DisableErrorLog == 0)
		            {
			            NC_DumpDebugBus();
			            NC_DumpRegisters();
		            }
					REG_CLR_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);
					REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
					NC_ResetFCIE();
					NC_Config();
					NC_CLR_DDR_MODE();
					//NC_ResetNandFlash(); // save code size
					#else
					//nand_stop();
					#endif
					return UNFD_ST_ERR_W_TIMEOUT_RM; // timeout
				}
				REG_WRITE_UINT16(NC_R2N_DATA_WR, pu16_DataBuf[
															 u16_i*(pNandDrv->u16_SectorByteCnt>>1) + u16_j*256 + u16_Cnt]);
				REG_SET_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_DO_EN);
			}

		}
	}
	REG_WRITE_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN|BIT_R2N_DO_END);


	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
	{
		U16 u16_Reg;
		REG_READ_UINT16(NC_MIE_EVENT, u16_Reg);
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Write_RM timeout 3, ErrCode:%Xh, Reg00h:%Xh\r\n",
				   UNFD_ST_ERR_W_TIMEOUT_RM, u16_Reg);
		#if 0 == IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		REG_CLR_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);
		REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		//NC_ResetNandFlash(); // save code size
		#else
		//nand_stop();
		#endif
		return UNFD_ST_ERR_W_TIMEOUT_RM; // timeout
	}

	if(pNandDrv->u8_SwPatchWaitRb == 1)
	{
		NC_RECONFIG();
		REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

		REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
		#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
		#endif
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_CHKSTATUS << 8)| CMD_0x70);
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
		nand_CheckPowerCut();

		REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);
		if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Write_RM Timeout 4, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
			#if 0==IF_IP_VERIFY
		    if(gu8_DisableErrorLog == 0)
		    {
			    NC_DumpDebugBus();
			    NC_DumpRegisters();
		    }
			NC_ResetFCIE();
			NC_Config();
			NC_CLR_DDR_MODE();
			NC_ResetNandFlash();
			#else
			//nand_stop();
			#endif
			return UNFD_ST_ERR_W_TIMEOUT; // timeout
		}
	}

	NC_CLR_DDR_MODE();
	REG_CLR_BITS_UINT16(NC_R2N_CTRL, BIT_R2N_MODE_EN);
	REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += pNandDrv->u16_PageByteCnt;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_write_count++;
            gu64_monitor_write_size_kb += (pNandDrv->u16_PageByteCnt / 1024);
        }
	return NC_CheckEWStatus(OPTYPE_WRITE);
}

U32 NC_Read_RandomIn
(
    U32 u32_PhyRow, U32 u32_Col, U8 *pu8DataBuf, U32 u32DataByteCnt
)
{
	U32 u32_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE();

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if 0 // random r / w do not want to go through DMA before
    #if IF_SPARE_AREA_DMA
	pNandDrv->u16_Reg48_Spare &= ~BIT_NC_SPARE_DEST_MIU;
    #endif
	pNandDrv->u16_Reg48_Spare |= BIT_NC_RANDOM_RW_OP_EN;
	REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);
	#else // now should enable
    // data go through CIFD
    
	if(IF_SPARE_DMA())
		REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare &
			~(BIT_NC_SPARE_DEST_MIU|BIT_NC_RANDOM_RW_OP_EN));

	#endif

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	if(pNandDrv->u8_WordMode)
		u32_Col >>= 1;
	if (pNandDrv->u16_Reg48_Spare & BIT_NC_ONE_COL_ADDR)
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_Col<<8);
	else
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_Col);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRow & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRow >> 16);
	if( pNandDrv->u16_bbm_wa == 0xBAD)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	}

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	if( pNandDrv->u16_bbm_wa == 0xBAD)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, ((OP_ADR_CYCLE_00|OP_ADR_TYPE_ONE|OP_ADR_SET_1) << 8) | CMD_0x80);
	}
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle << 8) | (CMD_0x00));
	if (pNandDrv->u16_Reg48_Spare & BIT_NC_ONE_COL_ADDR)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_RAN_DIN << 8) | ACT_WAITRB);
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
	} else
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x30);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_RAN_DIN);
	}

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RAN_CNT);
	u32_Tmp = u32DataByteCnt + (u32DataByteCnt & 0x01);	// byet-count needs to be word-alignment
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_Tmp);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);	// offset;

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START);
	if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_READ_TIME) == WAIT_READ_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Err: drvNAND_ReadRandomData_Ex - timeout \r\n");
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		#else
		//nand_stop();
		#endif
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}

	/* get data from CIFD */
	NC_GetCIFD(pu8DataBuf, 0, u32DataByteCnt);
	REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalReadBytes_Hal += u32DataByteCnt;
	#endif

	if(gu32_monitor_read_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRow);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_read_count++;
            gu64_monitor_read_size += u32DataByteCnt;
        }

	return UNFD_ST_SUCCESS;
}

U32 NC_Write_RandomOut
(
    U32 u32_PhyRow, U32 u32_Col, U8 *pu8_DataBuf, U32 u32_DataByteCnt
)
{
	U32 u32_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();

	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE();

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

    // data go through CIFD
	if(IF_SPARE_DMA())
		REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare &
			~(BIT_NC_SPARE_DEST_MIU|BIT_NC_RANDOM_RW_OP_EN));

	// set data into CIFD
	NC_SetCIFD(pu8_DataBuf, 0, u32_DataByteCnt);
	if (u32_DataByteCnt & 1)
	{
		U8 au8_Tmp[1];
		au8_Tmp[0] = 0xFF; // pad a 0xFF
		NC_SetCIFD(au8_Tmp, u32_DataByteCnt, 1);
	}
	//while(1)  nand_reset_WatchDog();

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	if(pNandDrv->u8_WordMode)
		u32_Col >>= 1;
	if (pNandDrv->u16_Reg48_Spare & BIT_NC_ONE_COL_ADDR)
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_Col<<8);
	else
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_Col);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRow & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRow >> 16);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle << 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x10 << 8) | ACT_RAN_DOUT);

	if(pNandDrv->u8_SwPatchWaitRb == 0)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x70 << 8)| ACT_WAITRB);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_CHKSTATUS);
	}
	else
    	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_WAITRB);


	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RAN_CNT);
	u32_Tmp = u32_DataByteCnt + (u32_DataByteCnt & 0x01); // byet-count needs to be word-alignment
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_Tmp);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);	// offset;
	nand_CheckPowerCut();

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W);
	if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_Write_RandomOut Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		#else
		//nand_stop();
		#endif

		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}

	if(pNandDrv->u8_SwPatchWaitRb == 1)
	{
	    NC_RECONFIG();
		REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

		REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
		#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
		#endif
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_CHKSTATUS << 8)| CMD_0x70);
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
		nand_CheckPowerCut();

		REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);
		if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_Write_RandomOut Timeout 1, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
			#if 0==IF_IP_VERIFY
		    if(gu8_DisableErrorLog == 0)
		    {
			    NC_DumpDebugBus();
			    NC_DumpRegisters();
		    }
			NC_ResetFCIE();
			NC_Config();
			NC_CLR_DDR_MODE();
			NC_ResetNandFlash();
			#else
			//nand_stop();
			#endif

			return UNFD_ST_ERR_W_TIMEOUT; // timeout
		}
	}

	REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);

	NC_CLR_DDR_MODE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_DataByteCnt;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRow);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_write_count++;
            gu64_monitor_write_size += u32_DataByteCnt;
        }
	
	return NC_CheckEWStatus(OPTYPE_WRITE);
}

U32 NC_ReadID(void)
{
	volatile U16 u16_Reg;
	U16 u16_i;
	U16 u16_IDByteCnt;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE();

	// data go through CIFD
	if(IF_SPARE_DMA())
		REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare &
			~(BIT_NC_SPARE_DEST_MIU|BIT_NC_RANDOM_RW_OP_EN));

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ADR_C2T1S0 << 8) | CMD_0x90);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_RAN_DIN);

    #if 0
	if( pNandDrv->u8_WordMode)
		u16_IDByteCnt = ((NAND_ID_BYTE_CNT-1)<<1);
	else
		u16_IDByteCnt = (NAND_ID_BYTE_CNT-1);
    #else
    if( pNandDrv->u8_WordMode)
		u16_IDByteCnt = NAND_ID_BYTE_CNT<<1;
	else
		u16_IDByteCnt = NAND_ID_BYTE_CNT+1;
    #endif


	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RAN_CNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u16_IDByteCnt);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);	/*offset 0*/

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_CIFD_ACCESS | BIT_NC_JOB_START);

	if (NC_WaitComplete(BIT_NC_JOB_END, DELAY_100ms_in_us) == DELAY_100ms_in_us)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error! Read ID timeout!\r\n");

		#if 0==IF_IP_VERIFY
		REG_READ_UINT16(NC_MIE_EVENT, u16_Reg);
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NC_MIE_EVENT: %Xh \r\n", u16_Reg);
		REG_READ_UINT16(NC_CTRL, u16_Reg);
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NC_CTRL: %Xh \r\n", u16_Reg);
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		#else
		//nand_stop();
		#endif

		REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);

		NC_CLR_DDR_MODE();
		NC_UNLOCK_FCIE();
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}

	if( pNandDrv->u8_WordMode)
	{
		for(u16_i=0 ; u16_i<(NAND_ID_BYTE_CNT-1) ; u16_i++)
		{
			NC_GetCIFD(&pNandDrv->au8_ID[u16_i], u16_i<<1, 1);
		}
	}
	else
	{
		NC_GetCIFD(pNandDrv->au8_ID, 0, (NAND_ID_BYTE_CNT-1));
	}

	nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "ID: %02X ", pNandDrv->au8_ID[0]);
	#if 0
	for (u16_i=1; u16_i<(NAND_ID_BYTE_CNT-1); u16_i++) {
		if (pNandDrv->au8_ID[u16_i] == pNandDrv->au8_ID[0] &&
			pNandDrv->au8_ID[u16_i+1] == pNandDrv->au8_ID[1])
			break;

		nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0, "%02X ", pNandDrv->au8_ID[u16_i]);
	}
	#endif

	#if IF_IP_VERIFY
	pNandDrv->u8_IDByteCnt = NAND_DEVICE_ID_LEN;
	#else
	pNandDrv->u8_IDByteCnt = NAND_ID_BYTE_CNT;//u16_i;
	#endif

	for (u16_i=1; u16_i<(NAND_ID_BYTE_CNT-1); u16_i++)
		nand_debug(UNFD_DEBUG_LEVEL_HIGH, 0, "%02X ", pNandDrv->au8_ID[u16_i]);

	/* trim tralling zero(s) */
	//while (pNandDrv->au8_ID[--u16_i] == '\0')
	#if 0
	while (pNandDrv->au8_ID[u16_i-1] == '\0')
	{
		u16_i -= 1;
		pNandDrv->u8_IDByteCnt = u16_i;
	}
	#endif
	nand_debug(UNFD_DEBUG_LEVEL_HIGH, 0, ", total %u bytes ID.\r\n", pNandDrv->u8_IDByteCnt);

	REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);
	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();
	return UNFD_ST_SUCCESS;	// ok
}

U32 NC_ReadUniqueID(U8 *pu8_UniqueIDBuf)
{
    U32 u32_Tmp;
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U8* pu8DataBuf = pNandDrv->PlatCtx_t.pu8_PageDataBuf;
    U16 u16_j, u16_Tmp;

    NC_PAD_SWITCH(pNandDrv->u8_PadMode);
    NC_SET_DDR_MODE();

    REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_CMDREG8);
    REG_WRITE_UINT16(NC_AUXREG_DAT, 0xED);

    // data go through CIFD
    if(IF_SPARE_DMA())
        REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare &
            ~(BIT_NC_SPARE_DEST_MIU|BIT_NC_RANDOM_RW_OP_EN));

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
    REG_WRITE_UINT16(NC_AUXREG_DAT, 0);

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (ADR_C2T1S0 << 8) | (CMD_REG8L));
    REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_RAN_DIN << 8) | ACT_WAITRB);
    REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RAN_CNT);
    u32_Tmp = 32*6; // byet-count needs to be word-alignment
    REG_WRITE_UINT16(NC_AUXREG_DAT, u32_Tmp);
    REG_WRITE_UINT16(NC_AUXREG_DAT, 0); // offset;

    REG_WRITE_UINT16(NC_CTRL, BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START);
    if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_READ_TIME) == WAIT_READ_TIME)
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Err: drvNAND_ReadRandomData_Ex - timeout \r\n");
        #if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
        NC_ResetFCIE();
        NC_Config();
        NC_CLR_DDR_MODE();
        NC_ResetNandFlash();
        #else
        //nand_stop();
        #endif
        return UNFD_ST_ERR_R_TIMEOUT; // timeout
    }

    /* get data from CIFD */
    NC_GetCIFD(pu8DataBuf, 0, u32_Tmp);

    REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);

    //check data by xor
    for(u16_j = 0; u16_j < 16; u16_j ++)
    {
        for(u16_Tmp = 0; u16_Tmp < 6; u16_Tmp ++)
        {
            if((pu8DataBuf[u16_Tmp*32 + u16_j] ^ pu8DataBuf[u16_Tmp*32 + u16_j + 16]) == 0xFF)
            {
                pu8_UniqueIDBuf[u16_j] = pu8DataBuf[u16_Tmp*32 + u16_j];
                break;
            }
        }
    }

    NC_CLR_DDR_MODE();

    return UNFD_ST_SUCCESS;
}

//=========================================================
// [CAUTION]: FCIE4 do NOT support DDR-NAND with x16 IO
//=========================================================
#if defined(FCIE4_DDR) && FCIE4_DDR

static U32 NC_SetFeature_Ex(U8 u8_Cmd, U8 u8_Addr, U8 *pu8_DataBuf, U32 u32_DataByteCnt)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U16 u16_Reg;

	if(u32_DataByteCnt & 1)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1,
			"error, NAND, odd bytes (%lXh) to set \n", u32_DataByteCnt);
		return UNFD_ST_ERR_HAL_W_INVALID_PARAM;
	}

	NC_SET_DDR_MODE();

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	// data go through CIFD
	if(IF_SPARE_DMA())
		REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare &
			~(BIT_NC_SPARE_DEST_MIU|BIT_NC_RANDOM_RW_OP_EN));

	// set data into CIFD
	NC_SetCIFD(pu8_DataBuf, 0, u32_DataByteCnt);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_CMDREG8);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u8_Cmd);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u8_Addr<<8)|u8_Addr);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ADR_C2T1S0 << 8) | CMD_REG8L);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | ACT_RAN_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RAN_CNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_DataByteCnt);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);	// offset;

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W);

	if (NC_WaitComplete(BIT_NC_JOB_END, DELAY_100ms_in_us) == DELAY_100ms_in_us)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error! timeout!\r\n");

		#if 0==IF_IP_VERIFY
		REG_READ_UINT16(NC_MIE_EVENT, u16_Reg);
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NC_MIE_EVENT: %Xh \r\n", u16_Reg);
		REG_READ_UINT16(NC_CTRL, u16_Reg);
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NC_CTRL: %Xh \r\n", u16_Reg);
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		#else
		nand_stop();
		#endif
		return UNFD_ST_ERR_SET_FEATURE_TIMEOUT;
	}

	REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);

	return UNFD_ST_SUCCESS;
}


U32 NC_SetONFISyncMode(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_Err;
	U8  au8_buf[4*3];

	u32_Err = nand_check_DDR_pad();
	if(u32_Err != UNFD_ST_SUCCESS)
		return u32_Err;

	if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
	{
		if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_TOGGLE)
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1,
				"Error, NAND, ToggleDDR can not set to Sync Mode\n");
			return UNFD_ST_ERR_TOGGLE_SET_SYNC_MODE;
		}
		return UNFD_ST_SUCCESS;
	}

	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();

	// Get Feature
	u32_Err = NC_RandomIn_1Cmd1Addr(
	    NAND_CMD_GET_FEATURE, NAND_ADR_TIMING_MODE, NAND_DDR_REMOVE_2NDBYTE_ONFI,
	    4, au8_buf);
	if(UNFD_ST_SUCCESS != u32_Err)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "err code: %lX\n", u32_Err);
		return u32_Err;
	}

	// need to check supported mode fist
	au8_buf[0] &= ~((3<<4)|0xF);
	au8_buf[0] |=   ((1<<4)|0x5); // set Sync Mode

	u32_Err = NC_SetFeature_Ex(
		NAND_CMD_SET_FEATURE, NAND_ADR_TIMING_MODE, au8_buf, 4);
	if(UNFD_ST_SUCCESS != u32_Err)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "err code: %lX\n", u32_Err);
		return u32_Err;
	}

	#if 0 // for LA test
	NC_RandomIn_1Cmd1Addr(
	    NAND_CMD_GET_FEATURE, NAND_ADR_TIMING_MODE, NAND_DDR_REMOVE_2NDBYTE_ONFI,
	    4, au8_buf);
	#endif

	return u32_Err;
}


U32 NC_SetToggleDDRMode(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_Err;
	U8  au8_buf[8];

	pNandDrv = pNandDrv;
	u32_Err = nand_check_DDR_pad();
	if(u32_Err != UNFD_ST_SUCCESS)
		return u32_Err;

	nand_pads_switch(NAND_PAD_BYPASS_MODE);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();

	// Get Feature
	nand_debug(UNFD_DEBUG_LEVEL, 0, "Get Feature\n");
	u32_Err = NC_RandomIn_1Cmd1Addr(
	    NAND_CMD_GET_FEATURE, NAND_ADR_INTERFACE_MODE, NAND_DDR_REMOVE_2NDBYTE_NA,
	    4, au8_buf);
	if(UNFD_ST_SUCCESS != u32_Err)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "err code: %lX\n", u32_Err);
		return u32_Err;
	}
	nand_debug(0,1,"DDR Interface : %X\n", au8_buf[0]);

	if((au8_buf[0] & (BIT8-1)) != 0)
	{
		au8_buf[0] &= (U8)(~(BIT8-1)); // set DDR Interface
		nand_debug(UNFD_DEBUG_LEVEL, 0, "Set Feature\n");
		u32_Err = NC_SetFeature_Ex(
			NAND_CMD_SET_FEATURE, NAND_ADR_INTERFACE_MODE, au8_buf, 4);
		if(UNFD_ST_SUCCESS != u32_Err)
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "err code: %lX\n", u32_Err);
			return u32_Err;
		}
	}
	return u32_Err;

}


#define JEDEC_TOGGLE_DDR_ID_BYTE_CNT  5
static const U8 sgau8_JEDEC[JEDEC_TOGGLE_DDR_ID_BYTE_CNT]
                = {'J','E','D','E','C'};
#define NAND_TOGGLE_MODE_READ_ID_40h  BIT1

#define ONFI_ID_BYTE_CNT  4
static const U8 sgau8_ONFI[ONFI_ID_BYTE_CNT]
                = {'O','N','F','I'};
#define NAND_ONFI_DDR_MODE_READ_BYTE  6
#define NAND_ONFI_DDR_MODE_READ_BIT   BIT5

static U32 NC_RandomIn_1Cmd1Addr(U8 u8_Cmd, U8 u8_Addr, U8 u8_Remove2ndByte, U8 u8_ByteCnt, U8 *pu8_DataBuf)
{
	volatile U16 u16_Reg;
	U16 u16_i;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();

	// data go through CIFD
	if(IF_SPARE_DMA())
		REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare &
			~(BIT_NC_SPARE_DEST_MIU|BIT_NC_RANDOM_RW_OP_EN));


	// HW setting
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	//NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();

	NC_SET_DDR_MODE();

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_CMDREG8);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u8_Cmd);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u8_Addr<<8)|u8_Addr);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ADR_C2T1S0 << 8) | CMD_REG8L);
	if(0x90 != u8_Cmd)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_RAN_DIN << 8) | ACT_WAITRB);
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
	}
	else
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_RAN_DIN);

	#if 0 // FCIE4 can NOT support DDR with x16 IOs
	if( pNandDrv->u8_WordMode)
		u16_IDByteCnt = u8_ByteCnt<<1; // for FCIE: /RE clk count
	else
		u16_IDByteCnt = u8_ByteCnt;
	#endif
	if(u8_ByteCnt & 1)
		u8_ByteCnt++;

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RAN_CNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u8_ByteCnt);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);	/*offset 0*/

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_CIFD_ACCESS | BIT_NC_JOB_START);

	gu8_DisableErrorLog = 1;

	if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_PROBE_TIME) == WAIT_PROBE_TIME)
	{
		#if 0==IF_IP_VERIFY
		REG_READ_UINT16(NC_MIE_EVENT, u16_Reg);
		nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1, "NC_MIE_EVENT: %Xh \r\n", u16_Reg);
		REG_READ_UINT16(NC_CTRL, u16_Reg);
		nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1, "NC_CTRL: %Xh \r\n", u16_Reg);
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		#else
		//nand_stop();
		#endif
		gu8_DisableErrorLog = 0;
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}

	gu8_DisableErrorLog = 0; 

	NC_GetCIFD(pu8_DataBuf, 0, u8_ByteCnt);

	#if IF_IP_VERIFY
	nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1, "ID: %02X ", pu8_DataBuf[0]);

	for (u16_i=1; u16_i<u8_ByteCnt; u16_i++)
		nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0, "%02X ", pu8_DataBuf[u16_i]);

	nand_debug(0,0,"\n");
	#endif

	// remove the double bytes if DDR mode
	if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
	switch(u8_Remove2ndByte)
	{
		case NAND_DDR_REMOVE_2NDBYTE_TOGGLE:
            for(u16_i=2; u16_i < u8_ByteCnt; u16_i++)
		    	pu8_DataBuf[u16_i] = pu8_DataBuf[u16_i<<1];
			break;
		case NAND_DDR_REMOVE_2NDBYTE_ONFI:
            for(u16_i=1; u16_i < u8_ByteCnt; u16_i++)
	    		pu8_DataBuf[u16_i] = pu8_DataBuf[u16_i<<1];
			break;
	}

	#if IF_IP_VERIFY
	nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1, "ID: %02X ", pu8_DataBuf[0]);

	for (u16_i=1; u16_i<u8_ByteCnt; u16_i++)
		nand_debug(UNFD_DEBUG_LEVEL_WARNING, 0, "%02X ", pu8_DataBuf[u16_i]);

	nand_debug(0,0,"\n\n");
	#endif

	REG_WRITE_UINT16(NC_SPARE, pNandDrv->u16_Reg48_Spare);

	NC_CLR_DDR_MODE();
	return UNFD_ST_SUCCESS;	// ok
}


static U32 NC_ProbeIfONFIDDR_Ex(void)
{
	//NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_Err;
	U8 u8_i, au8_buf[sizeof(sgau8_ONFI)*4];

	// read ONFI parameters
	u32_Err = NC_RandomIn_1Cmd1Addr(
		NAND_CMD_READ_PARAMETER, NAND_ADR_READ_PARAMETER, NAND_DDR_REMOVE_2NDBYTE_NA,
		sizeof(sgau8_ONFI)*2+2, au8_buf);
	if(UNFD_ST_SUCCESS != u32_Err)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "err code: %lX\n", u32_Err);
		return u32_Err;
	}

	for(u8_i=0; u8_i<sizeof(sgau8_ONFI); u8_i++)
		if(sgau8_ONFI[u8_i] != au8_buf[u8_i])
			break;
	if(sizeof(sgau8_ONFI) != u8_i)
		return UNFD_ST_ERR_NOT_ONFI_DDR;

	if(0 == (au8_buf[NAND_ONFI_DDR_MODE_READ_BYTE] & NAND_ONFI_DDR_MODE_READ_BIT))
		return UNFD_ST_ERR_NOT_ONFI_DDR;

	return UNFD_ST_SUCCESS;
}


U32 NC_ProbeIfONFIDDR(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_Err;
	volatile U16 u16_Reg50_EccCtrl;

	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_RECONFIG();

	nand_clock_setting(FCIE3_SW_SLOWEST_CLK);

	// can NOT support 16-bits ONFI NAND
	//nand_debug(0,1,"WordMode:%X\n", pNandDrv->u8_WordMode);
	pNandDrv->u8_WordMode = 0;
	u16_Reg50_EccCtrl = pNandDrv->u16_Reg50_EccCtrl;
	pNandDrv->u16_Reg50_EccCtrl &= ~BIT_NC_WORD_MODE;
	REG_WRITE_UINT16(NC_ECC_CTRL, pNandDrv->u16_Reg50_EccCtrl);

	u32_Err = NC_ProbeIfONFIDDR_Ex();
	nand_clock_setting(pNandDrv->u32_Clk);

	if(UNFD_ST_SUCCESS != u32_Err)
	{
		if(u16_Reg50_EccCtrl & BIT_NC_WORD_MODE)
			pNandDrv->u8_WordMode = 1;

		pNandDrv->u16_Reg50_EccCtrl = u16_Reg50_EccCtrl;
		REG_WRITE_UINT16(NC_ECC_CTRL, pNandDrv->u16_Reg50_EccCtrl);

		nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "err code: %lX\n", u32_Err);
		return UNFD_ST_ERR_NOT_ONFI_DDR;
	}

	return u32_Err;

}


static U32 NC_ProbeIfToggleDDR_Ex(void)
{
	//NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_Err;
	U8 u8_i, au8_buf[sizeof(sgau8_JEDEC)*5];

	u32_Err = NC_RandomIn_1Cmd1Addr(
		NAND_CMD_READ_ID, NAND_ADR_READ_JEDEC, NAND_DDR_REMOVE_2NDBYTE_TOGGLE,
		sizeof(sgau8_JEDEC)*2+2, au8_buf);
	if(UNFD_ST_SUCCESS != u32_Err)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "err code: %lX\n", u32_Err);
		return u32_Err;
	}

	// check if repeat "JEDEC"
	for(u8_i=0; u8_i<sizeof(sgau8_JEDEC); u8_i++)
		if(au8_buf[u8_i] != au8_buf[u8_i + sizeof(sgau8_JEDEC)])
			break;
	if(sizeof(sgau8_JEDEC) == u8_i)
		return UNFD_ST_ERR_NOT_TOGGLE_DDR;

	// check if 'J' 'E' 'D' 'E' 'C' Toggle DDR
	for(u8_i=0; u8_i<sizeof(sgau8_JEDEC); u8_i++)
		if(sgau8_JEDEC[u8_i] != au8_buf[u8_i])
			break;
	if(sizeof(sgau8_JEDEC) != u8_i)
		return UNFD_ST_ERR_NOT_TOGGLE_DDR;

	if(0 == (au8_buf[u8_i]& NAND_TOGGLE_MODE_READ_ID_40h))
		return UNFD_ST_ERR_NOT_TOGGLE_DDR;

	return UNFD_ST_SUCCESS;
}

U32 NC_ProbeIfToggleDDR(void)
{
#if 0
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_Err;
	volatile U16 u16_Reg50_EccCtrl;

	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_RECONFIG();

	nand_clock_setting(FCIE3_SW_SLOWEST_CLK);

	// can NOT support 16-bits Toggle NAND
	//nand_debug(0,1,"WordMode:%X\n", pNandDrv->u8_WordMode);
	pNandDrv->u8_WordMode = 0;
	u16_Reg50_EccCtrl = pNandDrv->u16_Reg50_EccCtrl;
	pNandDrv->u16_Reg50_EccCtrl &= ~BIT_NC_WORD_MODE;
	REG_WRITE_UINT16(NC_ECC_CTRL, pNandDrv->u16_Reg50_EccCtrl);

	u32_Err = NC_ProbeIfToggleDDR_Ex();
	nand_clock_setting(pNandDrv->u32_Clk);

	if(UNFD_ST_SUCCESS != u32_Err)
	{
		if(u16_Reg50_EccCtrl & BIT_NC_WORD_MODE)
			pNandDrv->u8_WordMode = 1;

		pNandDrv->u16_Reg50_EccCtrl = u16_Reg50_EccCtrl;
		REG_WRITE_UINT16(NC_ECC_CTRL, pNandDrv->u16_Reg50_EccCtrl);

		nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "err code: %lX\n", u32_Err);
		return u32_Err;
	}

	return u32_Err;
#else
	return UNFD_ST_ERR_NOT_TOGGLE_DDR;
#endif
}
#define MIN_PASS_CNT	3


#if defined(FCIE4_DDR_EMMC_PLL) && FCIE4_DDR_EMMC_PLL
U32 NC_DetectDDRTiming(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_RetVal;
	U8 u8_2Ch_dqsmode, u8_2Ch_delaycell = 0, u8_57h;
	U8 u8_57h_start = 0;
	U8 u8_pass_cnt;
	//U8 u8_max_delay_cell;
	//U8 u8_retry = 0;
	U32 (*pFn_ProbeIfDDR_Ex)(void);
	const U8 au8_dqsmode[DQS_MODE_TABLE_CNT] = DQS_MODE_SEARCH_TABLE;	// defined in platform-depedent .h
	U8 u8_DqsPassCnt;
	U16 u16_DqsMatch;
	U8 u8_DelayCellLen, u8_DelayCellMaxLen;
	U8	u8_DelayCellBegin, u8_DelayCellMaxBegin;

    printk("%s\n", __FUNCTION__);

	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();

	if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_ONFI)
		pFn_ProbeIfDDR_Ex = NC_ProbeIfONFIDDR_Ex;
	else if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_TOGGLE)
		pFn_ProbeIfDDR_Ex = NC_ProbeIfToggleDDR_Ex;
	else
	{	nand_debug(UNFD_DEBUG_LEVEL_ERROR,1,"Error, NAND, no DDR If selected.\n");
		return UNFD_ST_ERR_INVALID_PARAM;
	}

	u16_DqsMatch = 0;
	u8_DqsPassCnt = 0;

	// detect DQS_MODE & DQS_DELAY_CELL & RE_DDR_TIMING
	for(u8_2Ch_dqsmode=0 ; u8_2Ch_dqsmode<DQS_MODE_TABLE_CNT ; u8_2Ch_dqsmode++)
	{
		u8_pass_cnt = 0;

		for(u8_57h=3 ; u8_57h<0x10 ; u8_57h++) // ddr timing shouldn't match from 0~2
		{
	        NC_FCIE4SetInterface_EMMC_PLL(1, au8_dqsmode[u8_2Ch_dqsmode], u8_57h);
			u32_RetVal = (*pFn_ProbeIfDDR_Ex)();
			if(UNFD_ST_SUCCESS == u32_RetVal || UNFD_ST_ERR_PROBABLE_TOGGLE_DDR == u32_RetVal )
			{
				//nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "dqs_mode:%u, 57h:%u\n", au8_dqsmode[u8_2Ch_dqsmode], u8_57h);
				if(u8_pass_cnt == 0)
					u8_57h_start = u8_57h;

				if((++u8_pass_cnt)==MIN_PASS_CNT)
				{
					u16_DqsMatch |= (1<<u8_2Ch_dqsmode);
					u8_DqsPassCnt++;
				}
			}
			else
			{
				if(u8_pass_cnt != 0)
					break;
			}
		}
	}

	nand_debug(UNFD_DEBUG_LEVEL,1, "DqsPass = %d,  u16_DqsMatch = %X \n", u8_DqsPassCnt, u16_DqsMatch);

	if(u8_DqsPassCnt)
		goto MATCH;

	nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "Err, NAND, can't detect right timing \n");
	return UNFD_ST_ERR_NO_TOGGLE_DDR_TIMING;

MATCH:
	if(u8_DqsPassCnt >= 2)
	{
		//find longest sequence and get its central phase
		u8_DelayCellLen = 0;
		u8_DelayCellBegin = 0;
		u8_DelayCellMaxBegin = 0;
		u8_DelayCellMaxLen = 0;
		for(u8_2Ch_dqsmode = 0; u8_2Ch_dqsmode <  DQS_MODE_TABLE_CNT; u8_2Ch_dqsmode ++)
		{
			if(((u16_DqsMatch >> u8_2Ch_dqsmode) & 0x1) == 0x1)
			{
				if(u8_DelayCellLen == 0)
					u8_DelayCellBegin = u8_2Ch_dqsmode;
				u8_DelayCellLen ++;

				if(u8_2Ch_dqsmode == (DQS_MODE_TABLE_CNT - 1))
				{
					if(u8_DelayCellMaxLen < u8_DelayCellLen)
					{
						u8_DelayCellMaxBegin = u8_DelayCellBegin;
						u8_DelayCellMaxLen = u8_DelayCellLen;
					}
				}
			}
			else
			{
				if(u8_DelayCellMaxLen == 0)
				{
					u8_DelayCellMaxBegin = u8_DelayCellBegin;
					u8_DelayCellMaxLen = u8_DelayCellLen;
					nand_debug(UNFD_DEBUG_LEVEL, 0, " len  %d, Begin %d\n",u8_DelayCellMaxLen, u8_DelayCellMaxBegin );
				}
				else
				{
					if(u8_DelayCellMaxLen < u8_DelayCellLen)
					{
						u8_DelayCellMaxBegin = u8_DelayCellBegin;
						u8_DelayCellMaxLen = u8_DelayCellLen;
					}
				}
				u8_DelayCellLen = 0;
			}
		}

		nand_debug(UNFD_DEBUG_LEVEL, 0, " final len  %d, Begin %d\n",u8_DelayCellMaxLen, u8_DelayCellMaxBegin );
		u8_2Ch_dqsmode = u8_DelayCellMaxBegin + (u8_DelayCellMaxLen >> 1);
	}
	else
	{
		for(u8_2Ch_dqsmode = 0; u8_2Ch_dqsmode <  DQS_MODE_TABLE_CNT; u8_2Ch_dqsmode ++)
			if(((u16_DqsMatch >> u8_2Ch_dqsmode) & 0x1) == 0x1)
			{
				break;
			}
	}

	NC_FCIE4SetInterface_EMMC_PLL(1, au8_dqsmode[u8_2Ch_dqsmode], u8_57h_start+1);

	pNandDrv->tMinDDR.u8_ClkIdx = 0;
	pNandDrv->tMinDDR.u8_DqsMode = au8_dqsmode[u8_2Ch_dqsmode];
	pNandDrv->tMinDDR.u8_DelayCell = u8_2Ch_delaycell;
	pNandDrv->tMinDDR.u8_DdrTiming = u8_57h_start+1;

	nand_debug(UNFD_DEBUG_LEVEL,1,"ok, get TDDR timing: EMMC_PLL 09h:%X, 57h:%X\n",
				pNandDrv->u16_Emmc_Pll_Reg09, pNandDrv->u16_Reg57_RELatch);

	return UNFD_ST_SUCCESS;
}


U32 NC_FCIE4SetInterface_EMMC_PLL(U8 u8_IfDDR, U8 u8_dll_phase_sel,U8 u8_rd_ddr_timing)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	volatile U32 u32_Err = UNFD_ST_SUCCESS;

	if(u8_IfDDR)
	{
		#if defined(ENABLE_DELAY_CELL) && ENABLE_DELAY_CELL
		REG_SET_BITS_UINT16(REG_EMMC_PLL_RX1B,BIT1);
		pNandDrv->u16_Emmc_Pll_Reg09 &= ~BIT_EMMC_RXDELAY_CELL_MASK;
		pNandDrv->u16_Emmc_Pll_Reg09 |= u8_dll_phase_sel << BIT_EMMC_RXDELAY_CELL_SHIFT;
		REG_CLR_BITS_UINT16(REG_EMMC_PLL_RX1B,BIT_EMMC_RXDELAY_CELL_MASK);
		REG_SET_BITS_UINT16(REG_EMMC_PLL_RX1B, pNandDrv->u16_Emmc_Pll_Reg09 &BIT_EMMC_RXDELAY_CELL_SHIFT);
		#else
	    // Set DLL Phase Sel
		pNandDrv->u16_Emmc_Pll_Reg09 &= ~BIT_EMMC_RXDLL_PHASE_SEL_MASK;
		pNandDrv->u16_Emmc_Pll_Reg09 |= u8_dll_phase_sel << BIT_EMMC_RXDLL_PHASE_SEL_SHIFT;
		REG_CLR_BITS_UINT16(REG_EMMC_PLL_RX09, BIT_EMMC_RXDLL_PHASE_SEL_MASK);
		REG_SET_BITS_UINT16(REG_EMMC_PLL_RX09, pNandDrv->u16_Emmc_Pll_Reg09 & BIT_EMMC_RXDLL_PHASE_SEL_MASK);
		#endif
        // set RE latch timing
		pNandDrv->u16_Reg57_RELatch &= ~(BIT_RE_DDR_TIMING_MASK|BIT_NC_LATCH_DATA_MASK);
		pNandDrv->u16_Reg57_RELatch |= u8_rd_ddr_timing << BIT_RE_DDR_TIMING_SHIFT;

        // walk around timing bug
		pNandDrv->u16_Reg57_RELatch &= ~BIT_RE_SEC_TURN_CNT_MASK;
		pNandDrv->u16_Reg57_RELatch |= 0xE << BIT_RE_SEC_TURN_CNT_SHIFT;
		REG_WRITE_UINT16(NC_LATCH_DATA, pNandDrv->u16_Reg57_RELatch);
	}
	else
	{
		nand_pads_switch(NAND_PAD_BYPASS_MODE);
		u32_Err = NC_ResetNandFlash(); // switch ONFI to ASync Mode
	}

    return u32_Err;
}
#else

U32 NC_DetectDDRTiming(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_RetVal;
	U8 u8_2Ch_dqsmode, u8_2Ch_delaycell, u8_57h;
	U8 u8_57h_start = 0;
	U8 u8_pass_cnt;
	U8 u8_max_delay_cell;
	U8 u8_retry = 0;
	U32 (*pFn_ProbeIfDDR_Ex)(void);
	const U8 au8_dqsmode[DQS_MODE_TABLE_CNT] = DQS_MODE_SEARCH_TABLE;	// defined in platform-depedent .h

	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();

	if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_ONFI)
		pFn_ProbeIfDDR_Ex = NC_ProbeIfONFIDDR_Ex;
	else if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_TOGGLE)
		pFn_ProbeIfDDR_Ex = NC_ProbeIfToggleDDR_Ex;
	else
	{	nand_debug(UNFD_DEBUG_LEVEL_ERROR,1,"Error, NAND, no DDR If selected.\n");
		return UNFD_ST_ERR_INVALID_PARAM;
	}

	u8_max_delay_cell = 0;
RETRY:
	// detect DQS_MODE & DQS_DELAY_CELL & RE_DDR_TIMING
	for(u8_2Ch_dqsmode=0 ; u8_2Ch_dqsmode<DQS_MODE_TABLE_CNT ; u8_2Ch_dqsmode++)
	{
		for(u8_2Ch_delaycell=0 ; u8_2Ch_delaycell<=u8_max_delay_cell ; u8_2Ch_delaycell++)
		{
			u8_pass_cnt = 0;

			for(u8_57h=3 ; u8_57h<0x10 ; u8_57h++) // ddr timing shouldn't match from 0~2
			{
				NC_FCIE4SetInterface(1, au8_dqsmode[u8_2Ch_dqsmode], u8_2Ch_delaycell, u8_57h);
				u32_RetVal = (*pFn_ProbeIfDDR_Ex)();
				if(UNFD_ST_SUCCESS == u32_RetVal || UNFD_ST_ERR_PROBABLE_TOGGLE_DDR == u32_RetVal )
				{
					//nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "dqs_mode:%u, 57h:%u\n", au8_dqsmode[u8_2Ch_dqsmode], u8_57h);
					if(u8_pass_cnt == 0)
						u8_57h_start = u8_57h;

					if((++u8_pass_cnt)==MIN_PASS_CNT)
						goto MATCH;
				}
				else
				{
					if(u8_pass_cnt != 0)
						break;
				}
			}
		}
	}

	if(u8_retry == 0)
	{
		u8_retry++;
		nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "Retry triple loop\n");
		u8_max_delay_cell = 0xF;
		goto RETRY;
	}

	nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "Err, NAND, can't detect right timing \n");
	return UNFD_ST_ERR_NO_TOGGLE_DDR_TIMING;

MATCH:
	NC_FCIE4SetInterface(1, au8_dqsmode[u8_2Ch_dqsmode], u8_2Ch_delaycell, u8_57h_start+1);

	pNandDrv->tMinDDR.u8_ClkIdx = 0;
	pNandDrv->tMinDDR.u8_DqsMode = au8_dqsmode[u8_2Ch_dqsmode];
	pNandDrv->tMinDDR.u8_DelayCell = u8_2Ch_delaycell;
	pNandDrv->tMinDDR.u8_DdrTiming = u8_57h_start+1;

	nand_debug(UNFD_DEBUG_LEVEL,1,"ok, get TDDR timing: 2Ch:%X, 57h:%X\n",
					pNandDrv->u16_Reg2C_SMStatus, pNandDrv->u16_Reg57_RELatch);

	return UNFD_ST_SUCCESS;
}

U32 NC_FCIE4SetInterface(U8 u8_IfDDR, U8 u8_dqs_mode, U8 u8_dqs_delaycell, U8 u8_rd_ddr_timing)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	volatile U32 u32_Err = UNFD_ST_SUCCESS;

	if(u8_IfDDR)
	{
		// set DQS_MODE
		pNandDrv->u16_Reg2C_SMStatus &= ~BIT_DQS_MODE_MASK;
		pNandDrv->u16_Reg2C_SMStatus |= u8_dqs_mode<<BIT_DQS_MDOE_SHIFT;
		REG_CLR_BITS_UINT16(NC_SM_STS, BIT_DQS_MODE_MASK);
		REG_SET_BITS_UINT16(NC_SM_STS, pNandDrv->u16_Reg2C_SMStatus & BIT_DQS_MODE_MASK);
		#if defined(REG_ANALOG_DELAY_CELL)
		// set Analog delay cell
		pNandDrv->u16_Analog_dqs_delaycell &= ~BIT_ANALOG_DELAY_CELL_MASK;
		pNandDrv->u16_Analog_dqs_delaycell |= u8_dqs_delaycell<<8;
		REG_CLR_BITS_UINT16(REG_ANALOG_DELAY_CELL,BIT_ANALOG_DELAY_CELL_MASK);
		REG_SET_BITS_UINT16(REG_ANALOG_DELAY_CELL,pNandDrv->u16_Analog_dqs_delaycell & BIT_ANALOG_DELAY_CELL_MASK);

		#else

		// set DQS_DELAY_CELL
		pNandDrv->u16_Reg2C_SMStatus &= ~BIT_DQS_DELAY_CELL_MASK;
		pNandDrv->u16_Reg2C_SMStatus |= u8_dqs_delaycell<<BIT_DQS_DELAY_CELL_SHIFT;
		REG_CLR_BITS_UINT16(NC_SM_STS, BIT_DQS_DELAY_CELL_MASK);
		REG_SET_BITS_UINT16(NC_SM_STS, pNandDrv->u16_Reg2C_SMStatus & BIT_DQS_DELAY_CELL_MASK);
		#endif


		// set RE latch timing
		pNandDrv->u16_Reg57_RELatch &= ~(BIT_RE_DDR_TIMING_MASK|BIT_NC_LATCH_DATA_MASK);
		pNandDrv->u16_Reg57_RELatch |= u8_rd_ddr_timing << BIT_RE_DDR_TIMING_SHIFT;

        // walk around timing bug
		pNandDrv->u16_Reg57_RELatch &= ~BIT_RE_SEC_TURN_CNT_MASK;
		pNandDrv->u16_Reg57_RELatch |= 0xE << BIT_RE_SEC_TURN_CNT_SHIFT;
		REG_WRITE_UINT16(NC_LATCH_DATA, pNandDrv->u16_Reg57_RELatch);
	}
	else
	{
		//-------------------------------
		// set for SDR goes through macro
	    //-------------------------------
		nand_pads_switch(NAND_PAD_BYPASS_MODE);
		u32_Err = NC_ResetNandFlash(); // switch ONFI to ASync Mode
	}

	return u32_Err;
}
#endif

//=========================================================
#endif // end of FCIE4_DDR

U32 NC_ProbeReadSeq(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	volatile U16 u16_Reg;
	volatile U32 u32_Count;

	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	#if defined(ENABLE_NAND_INTERRUPT_MODE) && ENABLE_NAND_INTERRUPT_MODE
	// ProbeReadSeq uses polling mode, so disable interupts here
	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_MMA_DATA_END | BIT_NC_JOB_END);
	#endif
	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x00));

	if (pNandDrv->u16_Reg48_Spare& BIT_NC_ONE_COL_ADDR)  // if a page 512B
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_WAITRB);
	}
	else
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x30);
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
	}

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);
	for (u32_Count=0; u32_Count < WAIT_PROBE_TIME; u32_Count++)
	{
		REG_READ_UINT16(NC_MIE_EVENT, u16_Reg);
		if ((u16_Reg & BIT_NC_JOB_END) == BIT_NC_JOB_END)
			break;

		nand_hw_timer_delay(HW_TIMER_DELAY_1us);
		nand_reset_WatchDog();
	}

	if (u32_Count < WAIT_PROBE_TIME)
		REG_W1C_BITS_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END); /*clear events*/
	else
	{
		nand_debug(UNFD_DEBUG_LEVEL_HIGH, 1, "Error: NC_ProbeReadSeq Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		return UNFD_ST_ERR_R_TIMEOUT;
	}

	return UNFD_ST_SUCCESS;
}

U32 NC_EraseBlk(U32 u32_PhyRowIdx)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_Ret;

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE();

	REG_W1C_BITS_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END);
	//nand_debug(0,0,"e B:%03Xh \n", u32_PhyRowIdx>>pNandDrv->u8_BlkPageCntBits);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_Erase_AdrCycle << 8) | CMD_0x60);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0xD0);
	if(pNandDrv->u8_SwPatchWaitRb == 0)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_CHKSTATUS << 8) | CMD_0x70);
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
	}
	else
    	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
	
	nand_CheckPowerCut();
	REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);
	if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_ERASE_TIME) == WAIT_ERASE_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_EraseBlk Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_E_TIMEOUT);

		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		#else
		//nand_stop();
		#endif

		NC_CLR_DDR_MODE();
		NC_UNLOCK_FCIE();
		return UNFD_ST_ERR_E_TIMEOUT;
	}

	if(pNandDrv->u8_SwPatchWaitRb == 1)
	{
		NC_RECONFIG();
		REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

		REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
		#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
		#endif
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_CHKSTATUS << 8)| CMD_0x70);
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
		nand_CheckPowerCut();

		REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);
		if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_ERASE_TIME) == WAIT_ERASE_TIME)
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_EraseBlk Timeout 1, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
			#if 0==IF_IP_VERIFY
		    if(gu8_DisableErrorLog == 0)
		    {
			    NC_DumpDebugBus();
			    NC_DumpRegisters();
		    }
			NC_ResetFCIE();
			NC_Config();
			NC_ResetNandFlash();
			#else
			//nand_stop();
			#endif

			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return UNFD_ST_ERR_W_TIMEOUT; // timeout
		}
	}

	u32_Ret = NC_CheckEWStatus(OPTYPE_ERASE);

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();
	return u32_Ret;
}



U32 NC_CheckEWStatus(U8 u8_OpType)
{
	volatile U16 u16_Tmp;
	U32 u32_ErrCode = UNFD_ST_SUCCESS;

	REG_READ_UINT16(NC_ST_READ, u16_Tmp);

	if ((u16_Tmp & BIT_ST_READ_FAIL) == 1) { // if fail
		if (OPTYPE_ERASE == u8_OpType)
			u32_ErrCode = UNFD_ST_ERR_E_FAIL;
		else if (OPTYPE_WRITE == u8_OpType)
			u32_ErrCode = UNFD_ST_ERR_W_FAIL;

		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_CheckEWStatus Fail, Nand St:%Xh, ErrCode:%Xh \r\n",
				   REG(NC_ST_READ), u32_ErrCode);
		return u32_ErrCode;
	} else if ((u16_Tmp & BIT_ST_READ_BUSYn) == 0) { // if busy
		if (OPTYPE_ERASE == u8_OpType)
			u32_ErrCode = UNFD_ST_ERR_E_BUSY;
		else if (OPTYPE_WRITE == u8_OpType)
			u32_ErrCode = UNFD_ST_ERR_W_BUSY;

		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_CheckEWStatus Busy, Nand St:%Xh, ErrCode:%Xh \r\n",
				   REG(NC_ST_READ), u32_ErrCode);
		return u32_ErrCode;
	} else if ((u16_Tmp & BIT_ST_READ_PROTECTn) == 0) {	// if protected
		if (OPTYPE_ERASE == u8_OpType)
			u32_ErrCode = UNFD_ST_ERR_E_PROTECTED;
		else if (OPTYPE_WRITE == u8_OpType)
			u32_ErrCode = UNFD_ST_ERR_W_PROTECTED;

		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_CheckEWStatus Protected, Nand St:%Xh, ErrCode:%Xh \r\n",
				   REG(NC_ST_READ), u32_ErrCode);
		return u32_ErrCode;
	}

	return u32_ErrCode;
}



U32 NC_ResetNandFlash(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	pNandDrv = pNandDrv;	// for Lint warning

	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE();

	REG_W1C_BITS_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	#if defined(FCIE4_DDR) && FCIE4_DDR
	if(pNandDrv->u16_Reg58_DDRCtrl&BIT_DDR_ONFI)
	{
		// set commnad reg
		REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0xFC);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_REG8L);
	}
	else
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0xFF);
	#else
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0xFF);
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);

	if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_RESET_TIME) == WAIT_RESET_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_ResetNandFlash, ErrCode:%Xh \r\n", UNFD_ST_ERR_RST_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		#else
		//nand_stop();
		#endif

		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_RST_TIMEOUT;
	}

	NC_CLR_DDR_MODE();
	return UNFD_ST_SUCCESS;
}


void NC_SendCmdForLADebug(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();

	pNandDrv = pNandDrv; // for warning
	NC_SET_DDR_MODE();

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_CMDREG8);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0xDD);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK<<8)|CMD_REG8L);

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);

	NC_CLR_DDR_MODE();
}


U32 NC_WaitComplete_RM(U16 u16_WaitEvent, U32 u32_MicroSec)
{
	volatile U32 u32_Count;
	volatile U16 u16_Reg;

	for (u32_Count=0; u32_Count < u32_MicroSec; u32_Count++)
	{
		REG_READ_UINT16(NC_R2N_STAT, u16_Reg);
		if ((u16_Reg & u16_WaitEvent) == u16_WaitEvent)
			break;

		nand_hw_timer_delay(HW_TIMER_DELAY_1us); /* in the unit of 64us */
		nand_reset_WatchDog();
	}

	if(u32_Count == u32_MicroSec)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "[NC_DEBUG_DBUS0] : 0x%02X\r\n", REG(NC_DEBUG_DBUS0));
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "[NC_DEBUG_DBUS1] : 0x%02X\r\n", REG(NC_DEBUG_DBUS1));
	}

	return u32_Count;
}


U32 NC_WaitComplete(U16 u16_WaitEvent, U32 u32_MicroSec)
{
	volatile U32 u32_Count;
	#if defined(ENABLE_NAND_INTERRUPT_MODE) && ENABLE_NAND_INTERRUPT_MODE
	U8	u8_Int2Polling = 0;	
	volatile U32 u32_Err;
	#endif
	volatile U16 u16_Reg;
	U16 u16_WaitedEvent = 0;

    #if defined(ENABLE_NAND_INTERRUPT_MODE) && ENABLE_NAND_INTERRUPT_MODE
	REG_READ_UINT16(NC_MIE_INT_EN, u16_Reg);
	if(u16_Reg & u16_WaitEvent)
	{
		u32_Err = nand_WaitCompleteIntr(u16_WaitEvent, u32_MicroSec, &u16_WaitedEvent);
		if( u32_Err == UNFD_ST_SUCCESS )
			return 0;
		else
		{
			#if (defined(__VER_UNFD_FTL__)&&__VER_UNFD_FTL__)
			if(drvNAND_ChkRdy(0)) // after unfd init
			{
				NC_DumpDebugBus();
				NC_DumpRegisters();

				nand_die();
			}
			#else
	//		NC_DumpDebugBus();
	//		NC_DumpRegisters();

	//		nand_die();
			#endif
			nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1, "Interrupt mode Timeout IRQ Event 0x%X but 0x%X Expected\n", u16_WaitedEvent, u16_WaitEvent);
			
			REG_WRITE_UINT16(NC_MIE_INT_EN, 0);
			u8_Int2Polling = 1;
		}
	}
    #endif

	for (u32_Count=0; u32_Count < u32_MicroSec; u32_Count++)
	{
		REG_READ_UINT16(NC_MIE_EVENT, u16_Reg);
		if (( (u16_Reg | u16_WaitedEvent) & u16_WaitEvent) == u16_WaitEvent)
			break;

		nand_hw_timer_delay(HW_TIMER_DELAY_1us);
		nand_reset_WatchDog();
	}

	if (u32_Count < u32_MicroSec)
	{
		REG_W1C_BITS_UINT16(NC_MIE_EVENT, u16_WaitEvent); /*clear events*/
		#if defined(ENABLE_NAND_INTERRUPT_MODE) && ENABLE_NAND_INTERRUPT_MODE
		if(u8_Int2Polling == 1)
			nand_debug(UNFD_DEBUG_LEVEL_WARNING, 1, "But Polling Mode is OK, False Alarm\n"); 
		#endif
	}
	else
	{

		#if (defined(__VER_UNFD_FTL__)&&__VER_UNFD_FTL__)
		if(drvNAND_ChkRdy(0)) // after unfd init
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();

			nand_die();
		}
		#else
	    //nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "[NC_DEBUG_DBUS0] : 0x%02X\r\n", REG(NC_DEBUG_DBUS0));
	    //nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "[NC_DEBUG_DBUS1] : 0x%02X\r\n", REG(NC_DEBUG_DBUS1));
		#endif
	}
	#if defined(ENABLE_NAND_INTERRUPT_MODE) && ENABLE_NAND_INTERRUPT_MODE
	if(u8_Int2Polling == 1)
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);
    	#endif

	return u32_Count;
}


void NC_SetCIFD_Const(U8 u8_Data, U32 u32_CIFDPos, U32 u32_ByteCnt)
{
	U32 u32_i;
	volatile U16 u16_Tmp;

	if (u32_CIFDPos & 1) {
		REG_READ_UINT16(NC_CIFD_ADDR(u32_CIFDPos>>1), u16_Tmp);
		u16_Tmp &= 0x00FF;
		u16_Tmp += u8_Data << 8;
		//nand_debug(1, 1, "0: %X \r\n", u16_Tmp);
		REG_WRITE_UINT16(NC_CIFD_ADDR(u32_CIFDPos>>1), u16_Tmp);
		u32_CIFDPos += 1;
		u32_ByteCnt -= 1;
	}

	for (u32_i=0; u32_i<u32_ByteCnt>>1; u32_i++) {
		u16_Tmp = u8_Data + (u8_Data << 8);
		//nand_debug(1, 1, "1: %X \r\n", u16_Tmp);
		REG_WRITE_UINT16(NC_CIFD_ADDR(u32_i+(u32_CIFDPos>>1)), u16_Tmp);
	}

	if (u32_ByteCnt - (u32_i<<1)) {
		REG_READ_UINT16(NC_CIFD_ADDR(u32_i+(u32_CIFDPos>>1)), u16_Tmp);
		u16_Tmp = (u16_Tmp & 0xFF00) + u8_Data;
		//nand_debug(1, 1, "2: %X \r\n", u16_Tmp);
		REG_WRITE_UINT16(NC_CIFD_ADDR(u32_i+(u32_CIFDPos>>1)), u16_Tmp);
	}
}

/*lint -save -e661 -e662 stops LINT complaining Possible access of out-of-bounds */
void NC_SetCIFD(U8 *pu8_Buf, U32 u32_CIFDPos, U32 u32_ByteCnt)
{
	U32 u32_i, u32_BufPos;
	volatile U16 u16_Tmp;

	if (u32_CIFDPos & 1) {
		REG_READ_UINT16(NC_CIFD_ADDR(u32_CIFDPos>>1), u16_Tmp);
		u16_Tmp &= 0x00FF;
		u16_Tmp += pu8_Buf[0] << 8;
		//nand_debug(1, 1, "0: %X \r\n", u16_Tmp);
		REG_WRITE_UINT16(NC_CIFD_ADDR(u32_CIFDPos>>1), u16_Tmp);
		u32_CIFDPos += 1;
		u32_ByteCnt -= 1;
		u32_BufPos = 1;
	} else	u32_BufPos = 0;

	for (u32_i=0; u32_i<u32_ByteCnt>>1; u32_i++) {
		u16_Tmp = pu8_Buf[(u32_i<<1)+u32_BufPos] +
				  (pu8_Buf[(u32_i<<1)+u32_BufPos+1] << 8);
		//nand_debug(1, 1, "1: %X \r\n", u16_Tmp);
		REG_WRITE_UINT16(NC_CIFD_ADDR(u32_i+(u32_CIFDPos>>1)), u16_Tmp);
	}

	if (u32_ByteCnt - (u32_i<<1)) {
		REG_READ_UINT16(NC_CIFD_ADDR(u32_i+(u32_CIFDPos>>1)), u16_Tmp);
		u16_Tmp = (u16_Tmp & 0xFF00) + pu8_Buf[(u32_i<<1)+u32_BufPos];
		//nand_debug(1, 1, "2: %X \r\n", u16_Tmp);
		REG_WRITE_UINT16(NC_CIFD_ADDR(u32_i+(u32_CIFDPos>>1)), u16_Tmp);
	}
}


void NC_GetCIFD(U8 *pu8_Buf, U32 u32_CIFDPos, U32 u32_ByteCnt)
{
	U32 u32_i, u32_BufPos;
	U16 u16_Tmp;

	if (u32_CIFDPos & 1) {
		REG_READ_UINT16(NC_CIFD_ADDR(u32_CIFDPos>>1), u16_Tmp);
		pu8_Buf[0] = (U8)(u16_Tmp >> 8);
		u32_CIFDPos += 1;
		u32_ByteCnt -= 1;
		u32_BufPos = 1;
	} else
		u32_BufPos = 0;

	for (u32_i=0; u32_i<u32_ByteCnt>>1; u32_i++) {
		REG_READ_UINT16(NC_CIFD_ADDR(u32_i+(u32_CIFDPos>>1)), u16_Tmp);
		pu8_Buf[(u32_i<<1)+u32_BufPos] = (U8)u16_Tmp;
		pu8_Buf[(u32_i<<1)+u32_BufPos+1] = (U8)(u16_Tmp>>8);
	}

	if (u32_ByteCnt - (u32_i<<1)) {
		REG_READ_UINT16(NC_CIFD_ADDR(u32_i+(u32_CIFDPos>>1)), u16_Tmp);
		pu8_Buf[(u32_i<<1)+u32_BufPos] = (U8)u16_Tmp;
	}

}
/*lint -restore */

// Functions for Linux
U32 NC_ReadStatus(void)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U8 u8Status = 0;
	pNandDrv = pNandDrv;
	#if (defined(IF_FCIE_SHARE_IP)&&IF_FCIE_SHARE_IP)
	NC_LOCK_FCIE();
	#endif
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE();

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_CHKSTATUS<<8) | CMD_0x70);
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);

	if(NC_WaitComplete(BIT_NC_JOB_END, DELAY_100ms_in_us) == DELAY_100ms_in_us)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadStatus Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_CLR_DDR_MODE();
		#if (defined(IF_FCIE_SHARE_IP)&&IF_FCIE_SHARE_IP)
		NC_UNLOCK_FCIE();
		#endif
		return UNFD_ST_ERR_R_TIMEOUT;
	}

	u8Status = (U8)REG(NC_ST_READ);
	NC_SetCIFD(&u8Status, 0, 1);

	NC_CLR_DDR_MODE();
	#if (defined(IF_FCIE_SHARE_IP)&&IF_FCIE_SHARE_IP)
	NC_UNLOCK_FCIE();
	#endif

	return UNFD_ST_SUCCESS;
}

void NC_CheckECC(int *s32ECCStatus)
{
	U16 u16RegValue = 0;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();

	REG_READ_UINT16(NC_ECC_STAT0, u16RegValue);
    
	if (u16RegValue & BIT_NC_ECC_FAIL)
		*s32ECCStatus = -1;
	else
	{
		#if 0
		*s32ECCStatus = NC_GetECCBits();
		/* for MLC, small amount of bits error is very common,
		so we only scrub block when it surpass threshold */
		if (*s32ECCStatus < pNandDrv->u16_BitflipThreshold && *s32ECCStatus >=0 )
		{
			*s32ECCStatus = 0;
		}
		if (1 == gint_IS_ECC_Fail) //nand erase error recovery
		{
			gint_IS_ECC_Fail = 0;
			*s32ECCStatus = 1;
		}
		else if( 2 == gint_IS_ECC_Fail)	//if read retry runs > 3/4 of max retry run, issue torture or markbad
		{
			gint_IS_ECC_Fail = 0;
			//if(*s32ECCStatus < (pNandDrv->u16_ECCCorretableBit * 4 / 5))	//if correctable bit > 4/5 max correctable bit issue markbad 		//disable torture 
			//	*s32ECCStatus = pNandDrv->u16_ECCCorretableBit + 1;
		}
		else
		{
			if(pNandDrv->u8_RequireReadRetry && *s32ECCStatus >= (pNandDrv->u16_ECCCorretableBit * 4 / 5))		//just scrub this MLC block when read retry runs < 3/4 of max runs
			{
				*s32ECCStatus = 1;
			}
		}
		#else
		if(pNandDrv->u8_RequireReadRetry == 0)
		{
			*s32ECCStatus = NC_GetECCBits();
			if (*s32ECCStatus < pNandDrv->u16_BitflipThreshold && *s32ECCStatus >=0 )
			{
				*s32ECCStatus = 0;
			}
			if (1 == gint_IS_ECC_Fail) //nand erase error recovery
			{
				gint_IS_ECC_Fail = 0;
				*s32ECCStatus = 1;
			}
		}
		else
		{ 
			if( 2 == gint_IS_ECC_Fail)	//if read retry happens and bit flip count >threshold bit count => issue the refresh process.
			{
				gint_IS_ECC_Fail = 0;
				*s32ECCStatus = 1;
				//nand_debug(0, 1, "rr + bit flip 30~40\n");
			}
			else 
				*s32ECCStatus = 0;
		}
		#endif
	}
}


/*--------------------------------------------------
	u32_PlaneRowIdx : physical row address of plane1
  --------------------------------------------------*/
U32 NC_EraseBlk2P(U32 u32_PlaneRowIdx)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_Ret;
	U16 u16_Tmp;
	U32 u32_PlaneRowIdx2;

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE();

	REG_W1C_BITS_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END);
	//nand_debug(0,0,"e B:%03Xh \n", u32_PlaneRowIdx>>pNandDrv->u8_BlkPageCntBits);

	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);

	u16_Tmp = (pNandDrv->u8_OpCode_Erase_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_Erase_AdrCycle << 8) | CMD_0x60);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_Tmp << 8) | CMD_0x60);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0xD0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_CHKSTATUS << 8) | CMD_0x70);
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
    nand_CheckPowerCut();
	
	REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);
	if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_ERASE_TIME) == WAIT_ERASE_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_EraseBlk2Plane Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_E_TIMEOUT);

		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		#else
		//nand_stop();
		#endif

		NC_CLR_DDR_MODE();
		NC_UNLOCK_FCIE();
		return UNFD_ST_ERR_E_TIMEOUT;
	}

	u32_Ret = NC_CheckEWStatus(OPTYPE_ERASE);

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();
	return u32_Ret;
}

U32 NC_WritePagesCache
(
    U32 u32_PhyRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr=0, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8  u8_RetryCnt_FifoClkRdy=0;
	U32 u32_Ret;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;

        //nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	// can not cross block
	if ((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
	pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_WritePages, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_W_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_W_INVALID_PARAM;
	}
	if(u32_PageCnt == 1)
	{
		return NC_WritePages(u32_PhyRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt);
	}

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE(); // to turn on ONFI clk

    nand_redo_write:

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END|BIT_MIU_LAST_DONE);
#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
#endif

	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif		
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			if(pu8_OrigSpareBuf != NULL)
				memcpy(pu8_SpareBuf, pu8_OrigSpareBuf, pNandDrv->u16_SpareByteCnt);
			else
				memset(pu8_SpareBuf, 0xFF, pNandDrv->u16_SpareByteCnt);
		}
	
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}
	REG_WRITE_UINT16(NC_JOB_BL_CNT, u32_PageCnt << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto nand_redo_write;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x15 << 8) | ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_REPEAT << 8) | ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x10 << 8) | ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x70 << 8) | ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_CHKSTATUS);
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 2);

	//nand_debug(1, 1, "W Rpt Cnt: %Xh \r\n", u16_PageCnt-1);

	if(IF_SPARE_DMA() ==0)
	{
		if (pu8_SpareBuf)
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
		else
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				NC_SetCIFD_Const(0xFF, pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
	}

	nand_CheckPowerCut();
	

	REG_WRITE_UINT16(NC_CTRL,
		(BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	    //while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WritePages Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		#else
		//nand_stop();
		#endif
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}

	u32_Ret = NC_CheckEWStatus(OPTYPE_WRITE);
	nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_PageCnt * pNandDrv->u16_PageByteCnt;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_write_count++;
            gu64_monitor_write_size_kb += ((pNandDrv->u16_PageByteCnt / 1024 ) * u32_PageCnt);
        }
	
	return u32_Ret;
}


/*--------------------------------------------------
	u32_PlaneRowIdx : physical row address of plane1
	pu8_DataBuf   : data buffer of page data
					page1(plane1)+page1(plane2)+page2(plane1)+page2(plane2)
					...+pageN(plane1)+pageN(plane2)
	pu8_SpareBuf  : spare buffer of page spare data
					only 1 spare buffer can be set
	u32_PageCnt   : page count, used by repeat function
  --------------------------------------------------*/
U32 NC_WritePages2P_Micron
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr=0, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8  u8_RetryCnt_FifoClkRdy=0;
	U32 u32_Ret;
	U32 u32_PlaneRowIdx2;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);	
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;

	//nand_debug(0,0,"W+:%Xh %Xh \n", u32_PlaneRowIdx, u32_PageCnt);
	// can not cross block
	if ((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
		pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_WritePages2Plane, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_W_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_W_INVALID_PARAM;
	}

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE(); // to turn on ONFI clk

    LABEL_NAND_REDO_WRITE:

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif
	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif		
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			if(pu8_OrigSpareBuf != NULL)
				memcpy(pu8_SpareBuf, pu8_OrigSpareBuf, 2 * pNandDrv->u16_SpareByteCnt);
			else
				memset(pu8_SpareBuf, 0xFF, 2 *  pNandDrv->u16_SpareByteCnt);
		}
	
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}	
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					   0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);

			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_JOB_BL_CNT, (2 * u32_PageCnt) << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto LABEL_NAND_REDO_WRITE;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0x81<<8 | 0x11);

	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8L << 8) | ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8H << 8) | ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DOUT << 8) | u16_Tmp);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x10);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_CHKSTATUS << 8)| CMD_0x70);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_REPEAT);
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 1);
	//nand_debug(1, 1, "W Rpt Cnt: %Xh \r\n", u16_PageCnt-1);


	if(u8_IsSpareDMA == 0)
	{
		if (pu8_SpareBuf)
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt*2; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
		else
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt*2; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD_Const(0xFF, pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
								 pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
	}
	
	nand_CheckPowerCut();

	REG_WRITE_UINT16(NC_CTRL,
					 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WritePages2Plane Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		#else
		//nand_stop();
		#endif

		nand_DMA_UNMAP_address(u32_DataMapAddr,2 *  pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr,2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}

	u32_Ret = NC_CheckEWStatus(OPTYPE_WRITE);
	nand_DMA_UNMAP_address(u32_DataMapAddr,2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_PageCnt * pNandDrv->u16_PageByteCnt * 2;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PlaneRowIdx);

	if(gu32_monitor_count_enable)
        {
            gu32_monitor_write_count++;
            gu64_monitor_write_size_kb += ((pNandDrv->u16_PageByteCnt / 1024 * 2) * u32_PageCnt);
        }
	return u32_Ret;
}

/*--------------------------------------------------
	u32_PlaneRowIdx : physical row address of plane1
	pu8_DataBuf   : data buffer of page data
					page1(plane1)+page1(plane2)+page2(plane1)+page2(plane2)
					...+pageN(plane1)+pageN(plane2)
	pu8_SpareBuf  : spare buffer of page spare data
					only 1 spare buffer can be set
	u32_PageCnt   : page count, used by repeat function
  --------------------------------------------------*/
U32 NC_WritePages2P_Hynix
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr=0, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8  u8_RetryCnt_FifoClkRdy=0;
	U32 u32_Ret;
	U32 u32_PlaneRowIdx2;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);	
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;

	//nand_debug(0,0,"W+:%Xh %Xh \n", u32_PlaneRowIdx, u32_PageCnt);
	// can not cross block
	if ((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
		pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_WritePages2Plane, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_W_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_W_INVALID_PARAM;
	}

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE(); // to turn on ONFI clk

    LABEL_NAND_REDO_WRITE:

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif
	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif		
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			if(pu8_OrigSpareBuf != NULL)
				memcpy(pu8_SpareBuf, pu8_OrigSpareBuf, 2 * pNandDrv->u16_SpareByteCnt);
			else
				memset(pu8_SpareBuf, 0xFF, 2 *  pNandDrv->u16_SpareByteCnt);
		}
	
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}	
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					   0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);

			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_JOB_BL_CNT, (2 * u32_PageCnt) << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto LABEL_NAND_REDO_WRITE;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0x81<<8 | 0x11);

	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8L << 8) | ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8H << 8) | ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DOUT << 8) | u16_Tmp);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x10);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_CHKSTATUS << 8)| CMD_0x70);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_REPEAT);
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 1);
	//nand_debug(1, 1, "W Rpt Cnt: %Xh \r\n", u16_PageCnt-1);


	if(u8_IsSpareDMA == 0)
	{
		if (pu8_SpareBuf)
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt*2; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
		else
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt*2; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD_Const(0xFF, pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
								 pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
	}
	
	nand_CheckPowerCut();

	REG_WRITE_UINT16(NC_CTRL,
					 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WritePages2Plane Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		#else
		//nand_stop();
		#endif

		nand_DMA_UNMAP_address(u32_DataMapAddr,2 *  pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr,2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}

	u32_Ret = NC_CheckEWStatus(OPTYPE_WRITE);
	nand_DMA_UNMAP_address(u32_DataMapAddr,2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_PageCnt * pNandDrv->u16_PageByteCnt * 2;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PlaneRowIdx);

	if(gu32_monitor_count_enable)
		gu32_monitor_write_count++;

	return u32_Ret;
}

/*--------------------------------------------------
	u32_PlaneRowIdx : physical row address of plane1
	pu8_DataBuf   : data buffer of page data
					page1(plane1)+page1(plane2)+page2(plane1)+page2(plane2)
					...+pageN(plane1)+pageN(plane2)
	pu8_SpareBuf  : spare buffer of page spare data
					only 1 spare buffer can be set
	u32_PageCnt   : page count, used by repeat function
  --------------------------------------------------*/
U32 NC_WritePages2P
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	if(pNandDrv->au8_ID[0] == 0xAD)
		return NC_WritePages2P_Hynix(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt);
//		return NC_WritePages2P_Hynix(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt);			
	else if(pNandDrv->au8_ID[0] == 0x2C)
		return NC_WritePages2P_Micron(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt); 
	else
		return UNFD_ST_ERR_INVALID_PARAM;
}



/*--------------------------------------------------
	u32_PlaneRowIdx : physical row address of plane1
	pu8_DataBuf   : data buffer of page data
					page1(plane1)+page1(plane2)+page2(plane1)+page2(plane2)
					...+pageN(plane1)+pageN(plane2)
	pu8_SpareBuf  : spare buffer of page spare data
					only 1 spare buffer can be set
	u32_PageCnt   : page count, used by repeat function
  --------------------------------------------------*/
U32 NC_WritePagesCache2P
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr=0, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8  u8_RetryCnt_FifoClkRdy=0;
	U32 u32_Ret;
	U32 u32_PlaneRowIdx2;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	// can not cross block
	if ((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
		pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_WritePages2Plane, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_W_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_W_INVALID_PARAM;
	}

	if(u32_PageCnt == 1)
	{
		return(NC_WritePages2P(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, 1));
	}

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE(); // to turn on ONFI clk

    LABEL_NAND_REDO_WRITE:

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif
	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif		
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			if(pu8_OrigSpareBuf != NULL)
				memcpy(pu8_SpareBuf, pu8_OrigSpareBuf, 2 * pNandDrv->u16_SpareByteCnt);
			else
				memset(pu8_SpareBuf, 0xFF, 2 *  pNandDrv->u16_SpareByteCnt);
		}
	
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}	
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					   0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);

			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_JOB_BL_CNT, (2 * u32_PageCnt) << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	


		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto LABEL_NAND_REDO_WRITE;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0x81<<8 | 0x11);

	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8L << 8) | ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8H << 8) | ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DOUT << 8) | u16_Tmp);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x15);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x80 << 8)| ACT_REPEAT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DOUT << 8)| pNandDrv->u8_OpCode_RW_AdrCycle);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8)| CMD_REG8L);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_Tmp << 8)| CMD_REG8H);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x10 << 8)| ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x70 << 8)| ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_CHKSTATUS);
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 2);

	 if(u8_IsSpareDMA == 0)
	{
		if (pu8_SpareBuf)
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
		else
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD_Const(0xFF, pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
								 pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
	}

	nand_CheckPowerCut();

	REG_WRITE_UINT16(NC_CTRL,
					 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WritePages2Plane Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		#else
		//nand_stop();
		#endif
		nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}

	u32_Ret = NC_CheckEWStatus(OPTYPE_WRITE);
	nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_PageCnt * pNandDrv->u16_PageByteCnt * 2;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PlaneRowIdx);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_write_count++;
            gu64_monitor_write_size_kb += ((pNandDrv->u16_PageByteCnt / 1024 * 2) * u32_PageCnt );
        }
	return u32_Ret;
}


/*--------------------------------------------------
	u32_PlaneRowIdx : physical row address of plane1
	pu8_DataBuf   : data buffer of page data
					page1(plane1)+page1(plane2)+page2(plane1)+page2(plane2)
					...+pageN(plane1)+pageN(plane2)
	pu8_SpareBuf  : spare buffer of page spare data
					only 1 spare buffer can be set
	u32_PageCnt   : page count, used by repeat function
  --------------------------------------------------*/
U32 NC_WritePages2PCache_Micron
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr=0, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8  u8_RetryCnt_FifoClkRdy=0;
	U32 u32_Ret;
	U32 u32_PlaneRowIdx2;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	// can not cross block
	if ((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
		pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_WritePages2Plane, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_W_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_W_INVALID_PARAM;
	}

	if(u32_PageCnt < 2)
	{
		return(NC_WritePages2P(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, 1));
	}

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE(); // to turn on ONFI clk

    LABEL_NAND_REDO_WRITE:

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif
	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif		
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			if(pu8_OrigSpareBuf != NULL)
				memcpy(pu8_SpareBuf, pu8_OrigSpareBuf, 2 * pNandDrv->u16_SpareByteCnt);
			else
				memset(pu8_SpareBuf, 0xFF, 2 *  pNandDrv->u16_SpareByteCnt);
		}
	
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					   0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);

			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_JOB_BL_CNT, (2 * u32_PageCnt) << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);


		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto LABEL_NAND_REDO_WRITE;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0x81<<8 | 0x11);

	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8L << 8) | ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8H << 8) | ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DOUT << 8) | u16_Tmp);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x15);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x80 << 8)| ACT_REPEAT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DOUT << 8)| pNandDrv->u8_OpCode_RW_AdrCycle);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8)| CMD_REG8L);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_Tmp << 8)| CMD_REG8H);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x10 << 8)| ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x70 << 8)| ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_CHKSTATUS);
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 2);

	 if(u8_IsSpareDMA == 0)
	{
		if (pu8_SpareBuf)
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt*2; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
		else
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt*2; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD_Const(0xFF, pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
								 pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
	}

	nand_CheckPowerCut();

	REG_WRITE_UINT16(NC_CTRL,
					 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WritePages2Plane Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		#else
		//nand_stop();
		#endif
		nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}

	u32_Ret = NC_CheckEWStatus(OPTYPE_WRITE);
	nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_PageCnt * pNandDrv->u16_PageByteCnt * 2;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PlaneRowIdx);

	if(gu32_monitor_count_enable)
        {
	    gu32_monitor_write_count++;
            gu64_monitor_write_size_kb += ((pNandDrv->u16_PageByteCnt / 1024 * 2) * u32_PageCnt);
        }

	return u32_Ret;
}

/*--------------------------------------------------
	u32_PlaneRowIdx : physical row address of plane1
	pu8_DataBuf   : data buffer of page data
					page1(plane1)+page1(plane2)+page2(plane1)+page2(plane2)
					...+pageN(plane1)+pageN(plane2)
	pu8_SpareBuf  : spare buffer of page spare data
					only 1 spare buffer can be set
	u32_PageCnt   : page count, used by repeat function
  --------------------------------------------------*/
U32 NC_WritePages2PCache_Hynix
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr=0, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8  u8_RetryCnt_FifoClkRdy=0;
	U32 u32_Ret;
	U32 u32_PlaneRowIdx2;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	// can not cross block
	if ((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
		pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_WritePages2Plane, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_W_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_W_INVALID_PARAM;
	}

	if(u32_PageCnt < 2)
	{
		return(NC_WritePages2P(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, 1));
	}

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE(); // to turn on ONFI clk

    LABEL_NAND_REDO_WRITE:

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif
	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif		
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			if(pu8_OrigSpareBuf != NULL)
				memcpy(pu8_SpareBuf, pu8_OrigSpareBuf, 2 * pNandDrv->u16_SpareByteCnt);
			else
				memset(pu8_SpareBuf, 0xFF, 2 *  pNandDrv->u16_SpareByteCnt);
		}
	
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					   0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);

			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_JOB_BL_CNT, (2 * u32_PageCnt) << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);


		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto LABEL_NAND_REDO_WRITE;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0x81<<8 | 0x11);

	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8L << 8) | ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8H << 8) | ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DOUT << 8) | u16_Tmp);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x15);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x80 << 8)| ACT_REPEAT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DOUT << 8)| pNandDrv->u8_OpCode_RW_AdrCycle);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8)| CMD_REG8L);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_Tmp << 8)| CMD_REG8H);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x10 << 8)| ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x70 << 8)| ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_CHKSTATUS);
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 2);

	 if(u8_IsSpareDMA == 0)
	{
		if (pu8_SpareBuf)
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt*2; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
		else
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt*2; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD_Const(0xFF, pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
								 pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
	}

	nand_CheckPowerCut();

	REG_WRITE_UINT16(NC_CTRL,
					 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_WRITE_TIME) == WAIT_WRITE_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WritePages2Plane Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		#else
		//nand_stop();
		#endif
		nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}

	u32_Ret = NC_CheckEWStatus(OPTYPE_WRITE);
	nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_PageCnt * pNandDrv->u16_PageByteCnt * 2;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PlaneRowIdx);

	if(gu32_monitor_count_enable)
		gu32_monitor_write_count++;

	return u32_Ret;
}

/*--------------------------------------------------
	u32_PlaneRowIdx : physical row address of plane1
	pu8_DataBuf   : data buffer of page data
					page1(plane1)+page1(plane2)+page2(plane1)+page2(plane2)
					...+pageN(plane1)+pageN(plane2)
	pu8_SpareBuf  : spare buffer of page spare data
					only 1 spare buffer can be set
	u32_PageCnt   : page count, used by repeat function
  --------------------------------------------------*/
U32 NC_WritePages2PCache
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	if(pNandDrv->au8_ID[0] == 0xAD)
		return NC_WritePages2PCache_Hynix(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt);
//		return NC_WritePages2P_Hynix(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt);			
	else if(pNandDrv->au8_ID[0] == 0x2C)
		return NC_WritePages2PCache_Micron(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt); 
	else
		return UNFD_ST_ERR_INVALID_PARAM;
}


U32 NC_WritePages_CB
(
    U32 u32_PhyRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt, struct write_info *write
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr=0, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8  u8_RetryCnt_FifoClkRdy=0;
	U32 u32_Ret;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	// can not cross block
	if ((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
		pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_WritePages, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_W_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_W_INVALID_PARAM;
	}

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE(); // to turn on ONFI clk

	LABEL_NAND_REDO_WRITE:

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			if(pu8_OrigSpareBuf != NULL)
				memcpy(pu8_SpareBuf, pu8_OrigSpareBuf, pNandDrv->u16_SpareByteCnt);
			else
				memset(pu8_SpareBuf, 0xFF, pNandDrv->u16_SpareByteCnt);
		}

		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					   0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);

			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}
	REG_WRITE_UINT16(NC_JOB_BL_CNT, u32_PageCnt << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto LABEL_NAND_REDO_WRITE;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);

			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x10 << 8) | ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x70 << 8) | ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_REPEAT << 8) | ACT_CHKSTATUS);
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_BREAK);
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 1);
	//nand_debug(1, 1, "W Rpt Cnt: %Xh \r\n", u16_PageCnt-1);

	if(u8_IsSpareDMA== 0)
	{
		if (pu8_SpareBuf)
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
		else
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD_Const(0xFF, pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
								 pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
	}

	nand_CheckPowerCut();

	REG_WRITE_UINT16(NC_CTRL,
					 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	if(write->callback)
		write->callback(write->priv);
	write->done = 1;

	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_WRITE_TIME * u32_PageCnt) == WAIT_WRITE_TIME * u32_PageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WritePages Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		#else
		//nand_stop();
		#endif
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

		NC_CLR_DDR_MODE();
		NC_UNLOCK_FCIE();
		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}

	u32_Ret = NC_CheckEWStatus(OPTYPE_WRITE);
	nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_PageCnt * pNandDrv->u16_PageByteCnt;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_write_count++;
            gu64_monitor_write_size_kb += ((pNandDrv->u16_PageByteCnt / 1024) * u32_PageCnt);
        }

	return u32_Ret;
}


U32 NC_WritePages2P_CB
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt,
	struct write_info *write
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr=0, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8  u8_RetryCnt_FifoClkRdy=0;
	U32 u32_Ret;
	U32 u32_PlaneRowIdx2;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);	
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	// can not cross block
	if ((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
		pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_WritePages2Plane, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_W_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_W_INVALID_PARAM;
	}

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE(); // to turn on ONFI clk 

	LABEL_NAND_REDO_WRITE:	
	
	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL, 
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif		
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			if(pu8_OrigSpareBuf != NULL)
				memcpy(pu8_SpareBuf, pu8_OrigSpareBuf, 2 * pNandDrv->u16_SpareByteCnt);
			else
				memset(pu8_SpareBuf, 0xFF, 2 *  pNandDrv->u16_SpareByteCnt);
		}
	
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}	
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					   0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);

			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_JOB_BL_CNT, (2 * u32_PageCnt) << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto LABEL_NAND_REDO_WRITE;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);

			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0x81<<8 | 0x11);

	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8L << 8) | ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8H << 8) | ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DOUT << 8) | u16_Tmp);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x10);	
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_CHKSTATUS << 8)| CMD_0x70);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_REPEAT);	
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 1);
	//nand_debug(1, 1, "W Rpt Cnt: %Xh \r\n", u16_PageCnt-1);

	if(u8_IsSpareDMA== 0)
	{
		if (pu8_SpareBuf)
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
		else
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD_Const(0xFF, pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
								 pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
	}

	nand_CheckPowerCut();
	
	REG_WRITE_UINT16(NC_CTRL,
					 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	if(write->callback)
		write->callback(write->priv);
	write->done = 1;

	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_WRITE_TIME * 2 * u32_PageCnt) == WAIT_WRITE_TIME * 2 * u32_PageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WritePages2Plane Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		#else
		//nand_stop();
		#endif
		nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

		NC_CLR_DDR_MODE();
		NC_UNLOCK_FCIE();
		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}

	u32_Ret = NC_CheckEWStatus(OPTYPE_WRITE);
	nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_PageCnt * pNandDrv->u16_PageByteCnt * 2;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PlaneRowIdx);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_write_count++;
            gu64_monitor_write_size_kb += ((pNandDrv->u16_PageByteCnt / 1024 * 2) * u32_PageCnt);
        }

	return u32_Ret;
}


U32 NC_WritePagesCache_CB
(
    U32 u32_PhyRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt,
	struct write_info *write
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr=0, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8  u8_RetryCnt_FifoClkRdy=0;
	U32 u32_Ret;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);	
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;

        //nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	// can not cross block
	if ((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
	pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_WritePages, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_W_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_W_INVALID_PARAM;
	}
	if(u32_PageCnt == 1)
	{
		return NC_WritePages_CB(u32_PhyRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt, write);
	}

	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE(); // to turn on ONFI clk

    nand_redo_write:

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END|BIT_MIU_LAST_DONE);
	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif


	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif		
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			if(pu8_OrigSpareBuf != NULL)
				memcpy(pu8_SpareBuf, pu8_OrigSpareBuf, pNandDrv->u16_SpareByteCnt);
			else
				memset(pu8_SpareBuf, 0xFF, pNandDrv->u16_SpareByteCnt);
		}
	
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);

			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}
	
	REG_WRITE_UINT16(NC_JOB_BL_CNT, u32_PageCnt << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto nand_redo_write;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);

			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		return u32_Err;
	}
	    REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	    REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	    REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
	    REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);

	    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
		#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
		REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
		#endif
	    REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	    REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x15 << 8) | ACT_SER_DOUT);
	    REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_REPEAT << 8) | ACT_WAITRB);
	    REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	    REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x10 << 8) | ACT_SER_DOUT);
	    REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x70 << 8) | ACT_WAITRB);
	    REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_CHKSTATUS);
	    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	    REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 2);

    //nand_debug(1, 1, "W Rpt Cnt: %Xh \r\n", u16_PageCnt-1);

	if(u8_IsSpareDMA== 0)
	{
		if (pu8_SpareBuf)
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
		else
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				NC_SetCIFD_Const(0xFF, pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
	}

	nand_CheckPowerCut();
	

	REG_WRITE_UINT16(NC_CTRL,
		 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	if(write->callback)
		write->callback(write->priv);
	write->done = 1;

    if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_WRITE_TIME * u32_PageCnt) == WAIT_WRITE_TIME * u32_PageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WritePages Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

		NC_CLR_DDR_MODE();
		NC_UNLOCK_FCIE();
		#else
		//nand_stop();
		#endif
		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}
	//	printk(KERN_CRIT"[%s] End Job End\n",__func__);
	u32_Ret = NC_CheckEWStatus(OPTYPE_WRITE);
	nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_PageCnt * pNandDrv->u16_PageByteCnt;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_write_count++;
            gu64_monitor_write_size_kb += ((pNandDrv->u16_PageByteCnt / 1024) * u32_PageCnt);
        }

	return u32_Ret;
}


U32 NC_WritePagesCache2P_CB
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt,
	struct write_info *write
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr=0, u32_SpareMapAddr = 0;
	U32 u32_Err;
	U8  u8_RetryCnt_FifoClkRdy=0;
	U32 u32_Ret;
	U32 u32_PlaneRowIdx2;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);	
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	// can not cross block
	if ((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
		pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR: NC_WritePages2Plane, ErrCode:%Xh \r\n", UNFD_ST_ERR_HAL_W_INVALID_PARAM);
		return UNFD_ST_ERR_HAL_W_INVALID_PARAM;
	}

	if(u32_PageCnt == 1)
	{
		return(NC_WritePages2P_CB(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, 1, write));
	}
	
	NC_LOCK_FCIE();
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_CLK_SETTING(pNandDrv->u32_Clk);
	NC_RECONFIG();
	NC_SET_DDR_MODE(); // to turn on ONFI clk 

    LABEL_NAND_REDO_WRITE:	
	
	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL, 
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, WRITE_TO_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif		
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
			if(pu8_OrigSpareBuf != NULL)
				memcpy(pu8_SpareBuf, pu8_OrigSpareBuf, 2 * pNandDrv->u16_SpareByteCnt);
			else
				memset(pu8_SpareBuf, 0xFF, 2 * pNandDrv->u16_SpareByteCnt);
		}
	
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}	
		else
		{
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
				memset(pNandDrv->PlatCtx_t.pu8_PageSpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
					   0xFF, pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);

			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, 2 * pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}
	REG_WRITE_UINT16(NC_JOB_BL_CNT, (2 * u32_PageCnt) << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	if( (u32_Err=NC_waitFifoClkReady()) != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto LABEL_NAND_REDO_WRITE;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);

			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0x81<<8 | 0x11);

	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x80));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8L << 8) | ACT_SER_DOUT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_REG8H << 8) | ACT_WAITRB);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DOUT << 8) | u16_Tmp);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x15);	
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x80 << 8)| ACT_REPEAT);	
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DOUT << 8)| pNandDrv->u8_OpCode_RW_AdrCycle);	
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8)| CMD_REG8L);		
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_Tmp << 8)| CMD_REG8H);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x10 << 8)| ACT_SER_DOUT);	
	REG_WRITE_UINT16(NC_AUXREG_DAT, (CMD_0x70 << 8)| ACT_WAITRB);	
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_CHKSTATUS);
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 2);

	if(u8_IsSpareDMA == 0)
	{
		if (pu8_SpareBuf)
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
						   pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
		else
			for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
			{
				#if SPARE640B_CIFD512B_PATCH
				if((u16_Tmp+1)*pNandDrv->u16_SectorSpareByteCnt > 0x200)
					break;
				#endif
				NC_SetCIFD_Const(0xFF, pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
								 pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			}
	}
	nand_CheckPowerCut();

	REG_WRITE_UINT16(NC_CTRL,
					 (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START|BIT_NC_IF_DIR_W));

	if(write->callback)
		write->callback(write->priv);
	write->done = 1;

	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_WRITE_TIME * 2 * u32_PageCnt) == WAIT_WRITE_TIME * 2 * u32_PageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_WritePages2Plane Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_W_TIMEOUT);
		#if 0==IF_IP_VERIFY
		if(gu8_DisableErrorLog == 0)
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		#else
		//nand_stop();
		#endif
		nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	

		NC_CLR_DDR_MODE();
		NC_UNLOCK_FCIE();
		return UNFD_ST_ERR_W_TIMEOUT; // timeout
	}

	u32_Ret = NC_CheckEWStatus(OPTYPE_WRITE);
	nand_DMA_UNMAP_address(u32_DataMapAddr, 2 * pNandDrv->u16_PageByteCnt * u32_PageCnt, WRITE_TO_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, 2 *  pNandDrv->u16_SpareByteCnt, WRITE_TO_NAND);	


	NC_CLR_DDR_MODE();
	NC_UNLOCK_FCIE();

	#if defined(MEASURE_PERFORMANCE) && MEASURE_PERFORMANCE
	u64_TotalWriteBytes_Hal += u32_PageCnt * pNandDrv->u16_PageByteCnt * 2;
	#endif

	if(gu32_monitor_write_enable)
		printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PlaneRowIdx);

	if(gu32_monitor_count_enable)
        {   
	    gu32_monitor_write_count++;
            gu64_monitor_write_size_kb += ((pNandDrv->u16_PageByteCnt / 1024 * 2) * u32_PageCnt);
        }

	return u32_Ret;
}

// used for the case cross 2-plane, reg.47h set for 1st plane sector idx.
U32  NC_ReadSectors2P
(
    U32 u32_PhyRowIdx, U8 u8_SectorInPage, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_SectorCnt
)
{
    U16 u16_2PRowAdr, u16_SecCntTmp;
    U8  *pu8_DataBuf2;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr=0, u32_SpareMapAddr = 0;
	U32 u32_PlaneRowIdx2;
	U32 u32_Err;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_SET_DDR_MODE(); // to turn on ONFI clk

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

    REG_CLR_BITS_UINT16(NC_SIGNAL, // keep /CE low
        BIT_NC_CE_H|BIT_NC_CE_AUTO|BIT_NC_WP_H|BIT_NC_WP_AUTO);

	//----------------------------------
	u16_SecCntTmp = pNandDrv->u16_PageSectorCnt-u8_SectorInPage;
	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_SectorByteCnt * u16_SecCntTmp, READ_FROM_NAND);
	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u16_SecCntTmp, READ_FROM_NAND);
	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)	& BIT_MIU1_SELECT)		//main data miu1
		{
			#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
			#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
		}
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		}
		else
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_JOB_BL_CNT, u16_SecCntTmp);

	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);

	u32_Err=NC_waitFifoClkReady();
	if(u32_Err != UNFD_ST_SUCCESS)
	{
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

    //----------------------------------
    // read sectors from the 2-plane page
    REG_WRITE_UINT16(NC_PART_MODE, BIT_PARTIAL_MODE_EN);
	REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_CNT_MASK);
    REG_SET_BITS_UINT16(NC_PART_MODE, (u16_SecCntTmp-1)<<BIT_START_SECTOR_CNT_SHIFT);
	REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_IDX_MASK);
	REG_SET_BITS_UINT16(NC_PART_MODE, u8_SectorInPage<<BIT_START_SECTOR_IDX_SHIFT);

	//----------------------------------
	u32_PlaneRowIdx2 = u32_PhyRowIdx + pNandDrv->u16_BlkPageCnt;

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u8_SectorInPage << pNandDrv->u8_SectorByteCntBits);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0x32);
    REG_WRITE_UINT16(NC_AUXREG_ADR, 0x09);
    REG_WRITE_UINT16(NC_AUXREG_DAT, 0xE0<<8|0x06);
	u16_2PRowAdr = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x00));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_REG8L);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_2PRowAdr << 8) | CMD_0x00);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x30);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_REG9L));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | CMD_REG9H);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK<< 8) | ACT_BREAK);

	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

	REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START) & ~(BIT_NC_IF_DIR_W));
	//----------------------------------
	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME) == WAIT_READ_TIME*1)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadSectors2P Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
		#if 0==IF_IP_VERIFY
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		#else
		//nand_stop();
		#endif
        REG_WRITE_UINT16(NC_PART_MODE, 0);
        REG_WRITE_UINT16(NC_SIGNAL, pNandDrv->u16_Reg40_Signal);
		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}

	REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

	#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
   // check until MIU is done
	if( (u32_Err=NC_wait_MIULastDone()) != UNFD_ST_SUCCESS)
	{
	   NC_ResetFCIE();
	   NC_Config();
	   NC_ResetNandFlash();
	   NC_UNLOCK_FCIE();
	   return u32_Err;
	}
	#endif
    FLUSH_MIU_PIPE(); // Only used in U4 now

	if(REG(NC_ECC_STAT0) & BIT_NC_ECC_FAIL)
	{
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NAND Error: NC_ReadSectors2P ECC Fail \r\n");
	    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Blk: %Xh  Page: %Xh \r\n",
		    u32_PhyRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PhyRowIdx&pNandDrv->u16_BlkPageCntMask);
        #if 0==IF_IP_VERIFY
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		#endif
        REG_WRITE_UINT16(NC_PART_MODE, 0);
        REG_WRITE_UINT16(NC_SIGNAL, pNandDrv->u16_Reg40_Signal);
        return UNFD_ST_ERR_ECC_FAIL;
	}
	if (pu8_SpareBuf && u8_IsSpareDMA == 0)
	{
		#if SPARE640B_CIFD512B_PATCH
		if(pNandDrv->u16_SectorSpareByteCnt*u16_SecCntTmp > 0x200)
			NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
		else
		#endif
		NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SectorSpareByteCnt*u16_SecCntTmp);
		pu8_SpareBuf += pNandDrv->u16_SectorSpareByteCnt*u16_SecCntTmp;
	}
	nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u16_SecCntTmp, READ_FROM_NAND);
    pu8_DataBuf2 = pu8_DataBuf + (u16_SecCntTmp << pNandDrv->u8_SectorByteCntBits);
    u16_SecCntTmp = u32_SectorCnt-u16_SecCntTmp;
	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf2, pNandDrv->u16_SectorByteCnt * u16_SecCntTmp, READ_FROM_NAND);
	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u16_SecCntTmp, READ_FROM_NAND);

    //----------------------------------
    // read sectors from the 2-plane page
	REG_WRITE_UINT16(NC_JOB_BL_CNT, u16_SecCntTmp);

	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);

	u32_Err=NC_waitFifoClkReady();
	if(u32_Err != UNFD_ST_SUCCESS)
	{
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

    REG_WRITE_UINT16(NC_PART_MODE, BIT_PARTIAL_MODE_EN);
	REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_CNT_MASK);
    REG_SET_BITS_UINT16(NC_PART_MODE, (u16_SecCntTmp-1)<<BIT_START_SECTOR_CNT_SHIFT);
	REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_IDX_MASK);
	REG_SET_BITS_UINT16(NC_PART_MODE, 0);

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_2PRowAdr<< 8) | (CMD_REG9L));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | CMD_REG9H);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK<< 8) | ACT_BREAK);

	REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START) & ~(BIT_NC_IF_DIR_W));

    if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME) == WAIT_READ_TIME*1)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadSectors2P.2 Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
		#if 0==IF_IP_VERIFY
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		#else
		//nand_stop();
		#endif
        REG_WRITE_UINT16(NC_PART_MODE, 0);
        REG_WRITE_UINT16(NC_SIGNAL, pNandDrv->u16_Reg40_Signal);
		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}
	#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
	// check until MIU is done
	 if( (u32_Err=NC_wait_MIULastDone()) != UNFD_ST_SUCCESS)
	 {
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		return u32_Err;
	 }
	#endif

    FLUSH_MIU_PIPE(); // Only used in U4 now
	if(REG(NC_ECC_STAT0) & BIT_NC_ECC_FAIL)
	{
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NAND Error: NC_ReadSectors2P.2 ECC Fail \r\n");
	    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Blk: %Xh  Page: %Xh \r\n",
		    (u32_PhyRowIdx+pNandDrv->u16_BlkPageCnt)>>pNandDrv->u8_BlkPageCntBits,
		    u32_PhyRowIdx&pNandDrv->u16_BlkPageCntMask);
        #if 0==IF_IP_VERIFY
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		#endif
        REG_WRITE_UINT16(NC_PART_MODE, 0);
        REG_WRITE_UINT16(NC_SIGNAL, pNandDrv->u16_Reg40_Signal);
        return UNFD_ST_ERR_ECC_FAIL;
	}
	if (pu8_SpareBuf && u8_IsSpareDMA == 0)
	{
		#if SPARE640B_CIFD512B_PATCH
		if(pNandDrv->u16_SectorSpareByteCnt*u16_SecCntTmp > 0x200)
			NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
		else
		#endif
		NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SectorSpareByteCnt*u16_SecCntTmp);
	}
	if(u8_IsSpareDMA && pu8_SpareBuf)
	{
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
	}
	nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u16_SecCntTmp, READ_FROM_NAND);
    //----------------------------------
    REG_WRITE_UINT16(NC_PART_MODE, 0);
    REG_WRITE_UINT16(NC_SIGNAL, pNandDrv->u16_Reg40_Signal);
    NC_CLR_DDR_MODE();
	return UNFD_ST_SUCCESS;
}

/* pu8_SpareBuf  : spare buffer of page spare data
                             only 1 spare buffer can be set. */
U32 NC_ReadPages2P_Micron
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr = 0, u32_SpareMapAddr = 0;
	U32 u32_PlaneRowIdx2;
	U8 u8_RetryCnt_FifoClkRdy = 0;
	U32 u32_Err;
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	#if defined(CHECK_HAL_PARAM)
	// can not cross block
	if ((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
		pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1,
            "ERROR: NC_ReadPages2P, ErrCode: %Xh.  PlaneRowIdx: %lXh  PageCnt: %lXh\n",
            UNFD_ST_ERR_HAL_W_INVALID_PARAM, u32_PlaneRowIdx, u32_PageCnt);
		return UNFD_ST_ERR_HAL_R_INVALID_PARAM;
	}
    #endif
nand_redo_read:
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_SET_DDR_MODE(); // to turn on ONFI clk

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

	//----------------------------------
	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);
	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)		//main data dma to miu1
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
		}
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
		}
		else
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_SER_DIN_BYTECNT_LW, pNandDrv->u16_PageByteCnt & 0xFFFF);
	REG_WRITE_UINT16(NC_SER_DIN_BYTECNT_HW, pNandDrv->u16_PageByteCnt >> 16);
	REG_WRITE_UINT16(NC_JOB_BL_CNT, (2*u32_PageCnt) << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);//>>MIU_BUS_WIDTH_BITS));
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);//(MIU_BUS_WIDTH_BITS+16)));
	REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	u32_Err=NC_waitFifoClkReady();
	if( u32_Err != UNFD_ST_SUCCESS)
	{
	#if 0==IF_IP_VERIFY
		NC_ReConfig();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto nand_redo_read;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Soft reset over %d times, stop!\n", NAND_TIMEOUT_RETRY_CNT);
			NC_DumpRegisters();
			NC_CLR_DDR_MODE();
			return u32_Err;
		}
	#else
		//nand_stop();
	#endif
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	//----------------------------------
	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0x32);
    REG_WRITE_UINT16(NC_AUXREG_ADR, 0x09);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (0xE0<<8)|0x06);
	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x00));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_REG8L);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_Tmp << 8) | CMD_0x00);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x30);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_REG9L));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | CMD_REG9H);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_Tmp<< 8) | (CMD_REG9L));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | CMD_REG9H);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_REPEAT);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 1);

	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

	REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START) & ~(BIT_NC_IF_DIR_W));

	//----------------------------------
	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME*u32_PageCnt) == WAIT_READ_TIME*u32_PageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadPages2P Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
		#if 0==IF_IP_VERIFY
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END); //disable irq
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}
	REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END); //disable irq
	#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
	// check until MIU is done
	 if( (u32_Err=NC_wait_MIULastDone()) != UNFD_ST_SUCCESS)
	 {
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		return u32_Err;
	 }
	#endif
    FLUSH_MIU_PIPE(); // Only used in U4 now

	if(REG(NC_ECC_STAT0) & BIT_NC_ECC_FAIL)
	{
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NAND Error: NC_ReadPages2P ECC Fail \r\n");
	    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Blk: %Xh  Page: %Xh \r\n",
		    u32_PlaneRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PlaneRowIdx&pNandDrv->u16_BlkPageCntMask);
        #if 0==IF_IP_VERIFY
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

		#endif
        return UNFD_ST_ERR_ECC_FAIL;
	}

//	nand_debug(0,1, "pu8_SpareBuf %Xh\n", pu8_SpareBuf);
	if (pu8_SpareBuf && IF_SPARE_DMA() == 0)
	{
	    #if SPARE640B_CIFD512B_PATCH
		if(pNandDrv->u16_SpareByteCnt * 2 > 0x200)
			NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
		else
	    #endif
		NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SpareByteCnt * 2);
	//	nand_debug(0,1, "pu8_SpareBuf[0] %Xh\n", pu8_SpareBuf[0]);
	}
	if(u8_IsSpareDMA)
	{
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
		if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
			memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
	}
	nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

    //----------------------------------
	NC_CLR_DDR_MODE();
	return UNFD_ST_SUCCESS;
}

U32 NC_ReadPages2PCache_Ex_Micron
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt, U8 u8_Cmd
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr = 0, u32_SpareMapAddr = 0;
	U32 u32_PlaneRowIdx2;
	U8 u8_RetryCnt_FifoClkRdy = 0;
	U32 u32_Err;
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
nand_redo_read:
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_SET_DDR_MODE(); // to turn on ONFI clk

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

	//----------------------------------
	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);
	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)		//main data dma to miu1
		{
	#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
	#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
		}
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
		}
		else
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);

	REG_WRITE_UINT16(NC_SER_DIN_BYTECNT_LW, pNandDrv->u16_PageByteCnt & 0xFFFF);
	REG_WRITE_UINT16(NC_SER_DIN_BYTECNT_HW, pNandDrv->u16_PageByteCnt >> 16);
	REG_WRITE_UINT16(NC_JOB_BL_CNT, (2*u32_PageCnt) << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);//>>MIU_BUS_WIDTH_BITS));
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);//(MIU_BUS_WIDTH_BITS+16)));
	REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	u32_Err=NC_waitFifoClkReady();
	if( u32_Err != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ReConfig();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto nand_redo_read;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Soft reset over %d times, stop!\n", NAND_TIMEOUT_RETRY_CNT);
			NC_DumpRegisters();
			NC_CLR_DDR_MODE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	//----------------------------------
	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u8_Cmd<<8)|0x32);
    REG_WRITE_UINT16(NC_AUXREG_ADR, 0x09);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (0xE0<<8)|0x06);
	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
    //for the rest page
//    REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x00));
//    REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_REG8L);
//    REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_Tmp << 8) | CMD_0x00);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_REG8H);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_REG9L));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | CMD_REG9H);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_Tmp<< 8) | (CMD_REG9L));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | CMD_REG9H);        // 2 plane page 00...32 00..u8cmd
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK<< 8)| ACT_REPEAT);
    //for the last page
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 1);

	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END); //disable irq

	REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START) & ~(BIT_NC_IF_DIR_W));

	//----------------------------------
	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME*u32_PageCnt) == WAIT_READ_TIME*u32_PageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Timeout, ErrCode:%Xh Cmd: %Xh\r\n", UNFD_ST_ERR_R_TIMEOUT, u8_Cmd);
		#if 0==IF_IP_VERIFY
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END); //disable irq

		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}
	REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END); //disable irq
	#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
	// check until MIU is done
	 if( (u32_Err=NC_wait_MIULastDone()) != UNFD_ST_SUCCESS)
	 {
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		return u32_Err;
	 }
	#endif
    FLUSH_MIU_PIPE(); // Only used in U4 now

	if(REG(NC_ECC_STAT0) & BIT_NC_ECC_FAIL)
	{
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NAND Error: ECC Fail Cmd:%Xh\r\n", u8_Cmd);
	    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Blk: %Xh  Page: %Xh \r\n",
		    u32_PlaneRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PlaneRowIdx&pNandDrv->u16_BlkPageCntMask);
        #if 0==IF_IP_VERIFY
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

		#endif
        return UNFD_ST_ERR_ECC_FAIL;
	}
	if (pu8_SpareBuf && IF_SPARE_DMA() == 0)
	{
		#if SPARE640B_CIFD512B_PATCH
		if(pNandDrv->u16_SpareByteCnt * 2 > 0x200)
			NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
		else
		#endif
		NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SpareByteCnt * 2);
	//	nand_debug(0,1, "pu8_SpareBuf[0] %Xh\n", pu8_SpareBuf[0]);
	}

	if(u8_IsSpareDMA)
	{
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
		if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
			memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
	}
	nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

    //----------------------------------
	NC_CLR_DDR_MODE();
	return UNFD_ST_SUCCESS;
}


U32 NC_ReadPages2PCache_Micron
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
    U32 u32_i, u32_j, u32_Err;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();


RETRY_2P_CACHE:
    if(u32_PageCnt <= 2)
        return NC_ReadPages2P_Micron(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt);

    u32_Err = NC_ReadPages2P_Micron(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, 1);
    if(u32_Err)
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Read first 2plane fail %Xh\n", u32_Err);
        return u32_Err;
    }
    u32_PlaneRowIdx ++;
    pu8_DataBuf += pNandDrv->u16_PageByteCnt*2;
    u32_PageCnt --;

    //data for first 31 is the data of previous page
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0x31);
    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
    //for the rest page
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_REG8L);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_BREAK);

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);
	if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_READ_TIME) == WAIT_READ_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Timeout, ErrCode:%Xh\r\n", UNFD_ST_ERR_R_TIMEOUT);
    #if 0==IF_IP_VERIFY
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
    #else
		//nand_stop();
    #endif
		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}

    //for the middle page
    for(u32_i = 0; u32_i < u32_PageCnt; u32_i ++)
    {
//        nand_debug(0,1, "2Row %Xh, PageCnt %Xh\n", u32_PlaneRowIdx, u32_PageCnt);
//        nand_debug(0,1, "2Buf %Xh, Spare %Xh\n", pu8_DataBuf, pu8_SpareBuf);
        if(u32_i != u32_PageCnt-1)
           u32_Err = NC_ReadPages2PCache_Ex_Micron(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, 1, 0x31);
        else
           u32_Err = NC_ReadPages2PCache_Ex_Micron(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, 1, 0x3F);
        if(u32_Err != UNFD_ST_SUCCESS)
        {
            //using single plane read for read retry
            for(u32_j = 0; u32_j < 2; u32_j++)
            {
                u32_Err = NC_ReadPages(u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt*u32_j,
                    pu8_DataBuf, pu8_SpareBuf, 1);
                if(UNFD_ST_SUCCESS != u32_Err)
                   return u32_Err;
                pu8_DataBuf += pNandDrv->u16_PageByteCnt;
            }
            u32_PlaneRowIdx ++;
            u32_PageCnt -= u32_i + 1;       //why u32_i+1 => including current fail page
            if(u32_PageCnt)
	            goto RETRY_2P_CACHE;
			else
				return u32_Err;
        }
        u32_PlaneRowIdx ++;
        pu8_DataBuf += pNandDrv->u16_PageByteCnt*2;
    }

    return u32_Err;
}

/* pu8_SpareBuf  : spare buffer of page spare data
                             only 1 spare buffer can be set. */
U32 NC_ReadPages2P_Hynix
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr = 0, u32_SpareMapAddr = 0;
	U32 u32_PlaneRowIdx2;
	U8 u8_RetryCnt_FifoClkRdy = 0;
	U32 u32_Err;
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	#if defined(CHECK_HAL_PARAM)
	// can not cross block
	if ((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) + u32_PageCnt >
		pNandDrv->u16_BlkPageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1,
            "ERROR: NC_ReadPages2P, ErrCode: %Xh.  PlaneRowIdx: %lXh  PageCnt: %lXh\n",
            UNFD_ST_ERR_HAL_W_INVALID_PARAM, u32_PlaneRowIdx, u32_PageCnt);
		return UNFD_ST_ERR_HAL_R_INVALID_PARAM;
	}
    #endif
nand_redo_read:
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_SET_DDR_MODE(); // to turn on ONFI clk

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

	//----------------------------------
	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);
	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)		//main data dma to miu1
		{
		#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
		#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
		}
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
		}
		else
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_SER_DIN_BYTECNT_LW, pNandDrv->u16_PageByteCnt & 0xFFFF);
	REG_WRITE_UINT16(NC_SER_DIN_BYTECNT_HW, pNandDrv->u16_PageByteCnt >> 16);
	REG_WRITE_UINT16(NC_JOB_BL_CNT, (2*u32_PageCnt) << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);//>>MIU_BUS_WIDTH_BITS));
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);//(MIU_BUS_WIDTH_BITS+16)));
	REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	u32_Err=NC_waitFifoClkReady();
	if( u32_Err != UNFD_ST_SUCCESS)
	{
	#if 0==IF_IP_VERIFY
		NC_ReConfig();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto nand_redo_read;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Soft reset over %d times, stop!\n", NAND_TIMEOUT_RETRY_CNT);
			NC_DumpRegisters();
			NC_CLR_DDR_MODE();
			return u32_Err;
		}
	#else
		//nand_stop();
	#endif
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	//----------------------------------
	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (0xE0<<8)|0x05);
	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_Erase_AdrCycle<< 8) | (CMD_0x60));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (((pNandDrv->u8_OpCode_Erase_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1) << 8) | CMD_0x60);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_0x30);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle << 8) | CMD_0x00);
    REG_WRITE_UINT16(NC_AUXREG_DAT, ((OP_ADR_CYCLE_01|OP_ADR_TYPE_COL|OP_ADR_SET_0)<< 8) | (CMD_REG8L));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | CMD_REG8H);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_Tmp << 8) | CMD_0x00);	
    REG_WRITE_UINT16(NC_AUXREG_DAT, ((OP_ADR_CYCLE_01|OP_ADR_TYPE_COL|OP_ADR_SET_1)<< 8) | (CMD_REG8L));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | CMD_REG8H);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_REPEAT);

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 1);

	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

	REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START) & ~(BIT_NC_IF_DIR_W));

	//----------------------------------
	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME*u32_PageCnt) == WAIT_READ_TIME*u32_PageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadPages2P Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
		#if 0==IF_IP_VERIFY
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END); //disable irq
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}
	REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END); //disable irq
	#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
	// check until MIU is done
	 if( (u32_Err=NC_wait_MIULastDone()) != UNFD_ST_SUCCESS)
	 {
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		return u32_Err;
	 }
	#endif
    FLUSH_MIU_PIPE(); // Only used in U4 now

	if(REG(NC_ECC_STAT0) & BIT_NC_ECC_FAIL)
	{
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NAND Error: NC_ReadPages2P ECC Fail \r\n");
	    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Blk: %Xh  Page: %Xh \r\n",
		    u32_PlaneRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PlaneRowIdx&pNandDrv->u16_BlkPageCntMask);
        #if 0==IF_IP_VERIFY
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

		#endif
        return UNFD_ST_ERR_ECC_FAIL;
	}

//	nand_debug(0,1, "pu8_SpareBuf %Xh\n", pu8_SpareBuf);
	if (pu8_SpareBuf && IF_SPARE_DMA() == 0)
	{
	    #if SPARE640B_CIFD512B_PATCH
		if(pNandDrv->u16_SpareByteCnt * 2 > 0x200)
			NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
		else
	    #endif
		NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SpareByteCnt * 2);
	//	nand_debug(0,1, "pu8_SpareBuf[0] %Xh\n", pu8_SpareBuf[0]);
	}
	if(u8_IsSpareDMA)
	{
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
		if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
			memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
	}
	nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

    //----------------------------------
	NC_CLR_DDR_MODE();
	return UNFD_ST_SUCCESS;
}

U32 NC_ReadPages2PCache_Ex_Hynix
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt, U8 u8_Cmd
)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_DataDMAAddr, u32_DataMapAddr;
	U32 u32_SpareDMAAddr = 0, u32_SpareMapAddr = 0;
	U32 u32_PlaneRowIdx2;
	U8 u8_RetryCnt_FifoClkRdy = 0;
	U32 u32_Err;
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
nand_redo_read:
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_SET_DDR_MODE(); // to turn on ONFI clk

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

	//----------------------------------
	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);
	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)  & BIT_MIU1_SELECT)		//main data dma to miu1
		{
	#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
	#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
		}
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
		}
		else
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}

	REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);

	REG_WRITE_UINT16(NC_SER_DIN_BYTECNT_LW, pNandDrv->u16_PageByteCnt & 0xFFFF);
	REG_WRITE_UINT16(NC_SER_DIN_BYTECNT_HW, pNandDrv->u16_PageByteCnt >> 16);
	REG_WRITE_UINT16(NC_JOB_BL_CNT, (2*u32_PageCnt) << pNandDrv->u8_PageSectorCntBits);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);//>>MIU_BUS_WIDTH_BITS));
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);//(MIU_BUS_WIDTH_BITS+16)));
	REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
	u32_Err=NC_waitFifoClkReady();
	if( u32_Err != UNFD_ST_SUCCESS)
	{
		#if 0==IF_IP_VERIFY
		NC_ReConfig();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

		if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
			goto nand_redo_read;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Soft reset over %d times, stop!\n", NAND_TIMEOUT_RETRY_CNT);
			NC_DumpRegisters();
			NC_CLR_DDR_MODE();
			return u32_Err;
		}
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return u32_Err;
	}
	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

	//----------------------------------
	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (0xE0<<8)|0x05);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (u8_Cmd<<8)|0x33);	
	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	if(u8_Cmd != 0x3F)
	{
		REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_Erase_AdrCycle<< 8) | (CMD_0x60));
		REG_WRITE_UINT16(NC_AUXREG_DAT, (((pNandDrv->u8_OpCode_Erase_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1) << 8) | CMD_0x60);
		REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_REG9L);
	}
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_REG9H);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle << 8) | CMD_0x00);
    REG_WRITE_UINT16(NC_AUXREG_DAT, ((OP_ADR_CYCLE_01|OP_ADR_TYPE_COL|OP_ADR_SET_0)<< 8) | (CMD_REG8L));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | CMD_REG8H);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_Tmp << 8) | CMD_0x00);	
    REG_WRITE_UINT16(NC_AUXREG_DAT, ((OP_ADR_CYCLE_01|OP_ADR_TYPE_COL|OP_ADR_SET_1)<< 8) | (CMD_REG8L));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN << 8) | CMD_REG8H);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_REPEAT);
    //for the last page
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - 1);

	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END); //disable irq

	REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START) & ~(BIT_NC_IF_DIR_W));

	//----------------------------------
	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME*u32_PageCnt) == WAIT_READ_TIME*u32_PageCnt)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Timeout, ErrCode:%Xh Cmd: %Xh\r\n", UNFD_ST_ERR_R_TIMEOUT, u8_Cmd);
		#if 0==IF_IP_VERIFY
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END); //disable irq

		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}
	REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END); //disable irq
	#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
	// check until MIU is done
	 if( (u32_Err=NC_wait_MIULastDone()) != UNFD_ST_SUCCESS)
	 {
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		return u32_Err;
	 }
	#endif
    FLUSH_MIU_PIPE(); // Only used in U4 now

	if(REG(NC_ECC_STAT0) & BIT_NC_ECC_FAIL)
	{
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "NAND Error: ECC Fail Cmd:%Xh\r\n", u8_Cmd);
	    nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Blk: %Xh  Page: %Xh \r\n",
		    u32_PlaneRowIdx>>pNandDrv->u8_BlkPageCntBits, u32_PlaneRowIdx&pNandDrv->u16_BlkPageCntMask);
        #if 0==IF_IP_VERIFY
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
		}
		nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

		#endif
        return UNFD_ST_ERR_ECC_FAIL;
	}
	if (pu8_SpareBuf && IF_SPARE_DMA() == 0)
	{
		#if SPARE640B_CIFD512B_PATCH
		if(pNandDrv->u16_SpareByteCnt * 2 > 0x200)
			NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
		else
		#endif
		NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SpareByteCnt * 2);
	//	nand_debug(0,1, "pu8_SpareBuf[0] %Xh\n", pu8_SpareBuf[0]);
	}

	if(u8_IsSpareDMA)
	{
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt*2, READ_FROM_NAND);
		if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
			memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt*2);
	}
	nand_DMA_UNMAP_address(u32_DataMapAddr, (2 * pNandDrv->u16_PageByteCnt) * u32_PageCnt, READ_FROM_NAND);

    //----------------------------------
	NC_CLR_DDR_MODE();
	return UNFD_ST_SUCCESS;
}


U32 NC_ReadPages2PCache_Hynix
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	U32 u32_i, u32_j, u32_Err;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();


RETRY_2P_CACHE:
//	  nand_debug(0,1, "1Row %Xh, PageCnt %Xh\n", u32_PlaneRowIdx, u32_PageCnt);
//	  nand_debug(0,1, "1Buf %Xh, Spare %Xh\n", pu8_DataBuf, pu8_SpareBuf);	
	if(u32_PageCnt <= 2)
		return NC_ReadPages2P_Hynix(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt);

	//first page for normal read page 2p
	u32_Err = NC_ReadPages2P_Hynix(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, 1);
	if(u32_Err)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Read first 2plane fail %Xh\n", u32_Err);
		return u32_Err;
	}
	u32_PlaneRowIdx ++;
	pu8_DataBuf += pNandDrv->u16_PageByteCnt*2;
	u32_PageCnt --;

	for(u32_i = 0; u32_i < u32_PageCnt; u32_i ++)
	{
//		  nand_debug(0,1, "2Row %Xh, PageCnt %Xh, u32_i %Xh\n", u32_PlaneRowIdx, u32_PageCnt, u32_i);
//		  nand_debug(0,1, "2Buf %Xh, Spare %Xh\n", pu8_DataBuf, pu8_SpareBuf);
		if(u32_i != u32_PageCnt-1)
		   u32_Err = NC_ReadPages2PCache_Ex_Hynix(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, 1, 0x31);
		else
		   u32_Err = NC_ReadPages2PCache_Ex_Hynix(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, 1, 0x3F);
		if(u32_Err != UNFD_ST_SUCCESS)
		{
			//using single plane read for read retry
			for(u32_j = 0; u32_j < 2; u32_j++)
			{
				u32_Err = NC_ReadPages(u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt*u32_j,
					pu8_DataBuf, pu8_SpareBuf, 1);
				if(UNFD_ST_SUCCESS != u32_Err)
				   return u32_Err;
				pu8_DataBuf += pNandDrv->u16_PageByteCnt;
			}
			u32_PlaneRowIdx ++;
			u32_PageCnt -= u32_i + 1;		//why u32_i+1 => including current fail page
			if(u32_PageCnt)
				goto RETRY_2P_CACHE;
			else
				return u32_Err;
		}
		u32_PlaneRowIdx ++;
		pu8_DataBuf += pNandDrv->u16_PageByteCnt*2;
	}
	return u32_Err;
}
/* pu8_SpareBuf  : spare buffer of page spare data
                             only 1 spare buffer can be set. */
U32 NC_ReadPages2P
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	if(pNandDrv->au8_ID[0] == 0xAD)
		return NC_ReadPages2P_Hynix(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt);
	else if(pNandDrv->au8_ID[0] == 0x2C)
		return NC_ReadPages2P_Micron(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt);
	else
		return UNFD_ST_ERR_INVALID_PARAM;
}

U32 NC_ReadPages2PCache
(
	U32 u32_PlaneRowIdx, U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt
)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	if(pNandDrv->au8_ID[0] == 0xAD)
		return NC_ReadPages2PCache_Hynix(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt);
	else if(pNandDrv->au8_ID[0] == 0x2C)
		return NC_ReadPages2PCache_Micron(u32_PlaneRowIdx, pu8_DataBuf, pu8_SpareBuf, u32_PageCnt);
	else
		return UNFD_ST_ERR_INVALID_PARAM;
}

U32 NC_PageCopy2P(
    U32 u32_SrcPhyRowIdx, U32 u32_DstPhyRowIdx,
    U8 *pu8_DataBuf, U8 *pu8_SpareBuf, U32 u32_PageCnt, U32 *pu32_DonePageCnt)
{
    NAND_DRIVER *pNandDrv = drvNAND_get_DrvContext_address();

	U32 u32_Err, u32_Ret;
	U32 u32_DonePageCnt1 = 0, u32_DonePageCnt2 = 0;

	u32_Err = NC_PageCopy(u32_SrcPhyRowIdx, u32_DstPhyRowIdx,
		pu8_DataBuf, pu8_SpareBuf, u32_PageCnt, &u32_DonePageCnt1);

	u32_Ret = NC_PageCopy(u32_SrcPhyRowIdx + pNandDrv->u16_BlkPageCnt ,
		u32_DstPhyRowIdx + pNandDrv->u16_BlkPageCnt,
		pu8_DataBuf, pu8_SpareBuf, u32_PageCnt, &u32_DonePageCnt2);
	if(u32_Err || u32_Ret)
	{
		*pu32_DonePageCnt = (u32_DonePageCnt1 > u32_DonePageCnt2) ? u32_DonePageCnt2: u32_DonePageCnt1;
	}
	else
		*pu32_DonePageCnt = u32_DonePageCnt1;
	return u32_Err | u32_Ret;
}

U32 NC_ReadSend2P_Micron(U32 u32_PlaneRowIdx, U8 u8_Cmd)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_PlaneRowIdx2;

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_SET_DDR_MODE(); // to turn on ONFI clk

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

	//----------------------------------
	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u8_Cmd<<8)|0x32);
	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
    //for the rest page
    REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x00));
    REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_REG8L);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (u16_Tmp << 8) | CMD_0x00);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_REG8H);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK<< 8)| ACT_BREAK);
    //for the last page
	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);

	//----------------------------------
	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_READ_TIME) == WAIT_READ_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Timeout, ErrCode:%Xh Cmd: %Xh\r\n", UNFD_ST_ERR_R_TIMEOUT, u8_Cmd);
		#if 0==IF_IP_VERIFY
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}
	REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

    //----------------------------------

	NC_CLR_DDR_MODE();
	return UNFD_ST_SUCCESS;
}

U32 NC_ReadSend2P_Hynix(U32 u32_PlaneRowIdx, U8 u8_Cmd)
{
	U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	U32 u32_PlaneRowIdx2;

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_SET_DDR_MODE(); // to turn on ONFI clk

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

	//----------------------------------
	u32_PlaneRowIdx2 = u32_PlaneRowIdx + pNandDrv->u16_BlkPageCnt;

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx2 >> 16);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (u8_Cmd<<8)|u8_Cmd);
	u16_Tmp = (pNandDrv->u8_OpCode_RW_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1;

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_Erase_AdrCycle<< 8) | (CMD_0x60));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (((pNandDrv->u8_OpCode_Erase_AdrCycle &~ OP_ADR_SET_0) | OP_ADR_SET_1) << 8) | CMD_0x60);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_REG8L);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8)| ACT_BREAK);

    //for the last page
	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);

	//----------------------------------
	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_READ_TIME) == WAIT_READ_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Timeout, ErrCode:%Xh Cmd: %Xh\r\n", UNFD_ST_ERR_R_TIMEOUT, u8_Cmd);
		#if 0==IF_IP_VERIFY
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}
	REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

    //----------------------------------

	NC_CLR_DDR_MODE();
	return UNFD_ST_SUCCESS;
}

U32 NC_ReadSend2P(U32 u32_PlaneRowIdx, U8 u8_Cmd)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	if(pNandDrv->au8_ID[0] == 0xAD)
		return NC_ReadSend2P_Hynix(u32_PlaneRowIdx, u8_Cmd);
	else if(pNandDrv->au8_ID[0] == 0x2C)
		return NC_ReadSend2P_Micron(u32_PlaneRowIdx, u8_Cmd);
	else
		return UNFD_ST_ERR_INVALID_PARAM;
}

U32 NC_ReadSend(U32 u32_PlaneRowIdx, U8 u8_Cmd)
{
	//U16 u16_Tmp;
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();

	//nand_debug(0,1,"%X %X \n", u32_PhyRowIdx, u32_PageCnt);
	NC_PAD_SWITCH(pNandDrv->u8_PadMode);
	NC_SET_DDR_MODE(); // to turn on ONFI clk

	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END);

	#if defined(FCIE_LFSR) && FCIE_LFSR
	REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
	REG_SET_BITS_UINT16(NC_LFSR_CTRL,
		((u32_PlaneRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
	#endif

	//----------------------------------

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	REG_WRITE_UINT16(NC_AUXREG_DAT, 0);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx & 0xFFFF);
	REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PlaneRowIdx >> 16);

	// set commnad reg
	REG_WRITE_UINT16(NC_AUXREG_ADR, 0x08);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (u8_Cmd<<8)|u8_Cmd);

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
    //for the rest page
    REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x00));
    REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB << 8) | CMD_REG8L);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK<< 8)| ACT_BREAK);
    //for the last page
	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

	REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);

	//----------------------------------
	//while(1)  nand_reset_WatchDog();
	if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_READ_TIME) == WAIT_READ_TIME)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: Timeout, ErrCode:%Xh Cmd: %Xh\r\n", UNFD_ST_ERR_R_TIMEOUT, u8_Cmd);
		#if 0==IF_IP_VERIFY
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq
		NC_ReConfig();
		NC_CLR_DDR_MODE();
		NC_ResetNandFlash();
		#else
		//nand_stop();
		#endif
		NC_CLR_DDR_MODE();
		return UNFD_ST_ERR_R_TIMEOUT; // timeout
	}
	REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

    //----------------------------------

	NC_CLR_DDR_MODE();
	return UNFD_ST_SUCCESS;
}


unsigned int NC_ReadSectorsWithOP_Transfer_Micron(unsigned int u32_PhyRowIdx, unsigned char u8_SectorInPage, unsigned char *pu8_DataBuf, unsigned char *pu8_SpareBuf, unsigned int u32_SectorCnt)
{
    U16 u16_Tmp=0;
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U32 u32_DataDMAAddr, u32_DataMapAddr;
    U32 u32_SpareDMAAddr = 0, u32_SpareMapAddr = 0;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;
	U8 u8_RetryCnt_FifoClkRdy = 0;
	U32 u32_Err;

    NC_PAD_SWITCH(pNandDrv->u8_PadMode);
    NC_CLK_SETTING(pNandDrv->u32_Clk);
    NC_RECONFIG();


    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_CMDREG8);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (0xE0<<8) | 0x06);

    NC_SET_DDR_MODE();
nand_redo_read_sect:
	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END|BIT_MIU_LAST_DONE);

    #if defined(FCIE_LFSR) && FCIE_LFSR
    REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
    REG_SET_BITS_UINT16(NC_LFSR_CTRL,
        ((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
    #endif


	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt*u32_SectorCnt, READ_FROM_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)	& BIT_MIU1_SELECT)		//main data miu1
		{
			#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
			#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
		}
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		}
		else
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}
	REG_WRITE_UINT16(NC_JOB_BL_CNT, u32_SectorCnt);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);

	u32_Err=NC_waitFifoClkReady();
	if(u32_Err != UNFD_ST_SUCCESS)
	{
	#if IF_IP_VERIFY == 0
		NC_ReConfig();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}
		if(++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT)
			goto nand_redo_read_sect;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			NC_DumpRegisters();
			NC_DumpDebugBus();
			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
	#endif
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}

		NC_CLR_DDR_MODE();
		return u32_Err;
	}

	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

    REG_READ_UINT16(NC_ECC_CTRL, u16_Tmp);
    if (u16_Tmp & BIT_NC_PAGE_SIZE_MASK)
    {
        REG_WRITE_UINT16(NC_PART_MODE, BIT_PARTIAL_MODE_EN);
        REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_CNT_MASK);
        REG_SET_BITS_UINT16(NC_PART_MODE, (u32_SectorCnt-1)<<BIT_START_SECTOR_CNT_SHIFT);
        REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_IDX_MASK);
        REG_SET_BITS_UINT16(NC_PART_MODE, u8_SectorInPage<<BIT_START_SECTOR_IDX_SHIFT);
    }
	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
    REG_WRITE_UINT16(NC_AUXREG_DAT, u8_SectorInPage << pNandDrv->u8_SectorByteCntBits);
    REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
    REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
    REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_REG8L));
    REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN<< 8) | (CMD_REG8H));
    REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_BREAK);

	REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START) & ~(BIT_NC_IF_DIR_W));
    if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME) == (WAIT_READ_TIME))
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadSectorsWithOP_Transfer Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
		#if 0==IF_IP_VERIFY
        NC_ReConfig();
        NC_CLR_DDR_MODE();
        NC_ResetNandFlash();

        nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
	        nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		#else
        //nand_stop();
		#endif
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

        REG_WRITE_UINT16(NC_PART_MODE, 0);
		return UNFD_ST_ERR_R_TIMEOUT;
    }

	REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

    FLUSH_MIU_PIPE(); // Only used in U4 now
    //-----------------------------------
	#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
	// check until MIU is done
	 if( (u32_Err=NC_wait_MIULastDone()) != UNFD_ST_SUCCESS)
	 {
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		return u32_Err;
	 }
	#endif

    // Check ECC
    REG_READ_UINT16(NC_ECC_STAT0, u16_Tmp);
    if (u16_Tmp & BIT_NC_ECC_FAIL)
    {
        NC_ReConfig();
        NC_CLR_DDR_MODE();
        NC_ResetNandFlash();

        nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
        NC_CLR_DDR_MODE();
        REG_WRITE_UINT16(NC_PART_MODE, 0);

        //return NC_ReadSectors_MTD(u32_PhyRowIdx, u8_SectorInPage, pu8_DataBuf, pu8_SpareBuf, u32_SectorCnt);;
        return UNFD_ST_ERR_ECC_FAIL;
    }

    //-----------------------------------
    nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);

	if (pu8_SpareBuf && u8_IsSpareDMA == 0)
	{
	#if SPARE640B_CIFD512B_PATCH
		if(pNandDrv->u16_SectorSpareByteCnt*u32_SectorCnt > 0x200)
			NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
		else
	#endif
		NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SectorSpareByteCnt*u32_SectorCnt);
	}

    NC_CLR_DDR_MODE();

    #if defined( DDR_NAND_SUPPORT_RETRY_DQS) && DDR_NAND_SUPPORT_RETRY_DQS
    if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
        if(0!=u8_RetryDqs)
            nand_retry_dqs_post();
    #endif

    if(gu32_monitor_read_enable)
        printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);

    if(gu32_monitor_count_enable)
        gu32_monitor_read_count++;

    REG_WRITE_UINT16(NC_PART_MODE, 0);

    return UNFD_ST_SUCCESS;
}

unsigned int NC_ReadSectorsWithOP_Transfer_1p(unsigned int u32_PhyRowIdx, unsigned char u8_SectorInPage, unsigned char *pu8_DataBuf, unsigned char *pu8_SpareBuf, unsigned int u32_SectorCnt)
{
    U16 u16_Tmp=0;
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U32 u32_DataDMAAddr, u32_DataMapAddr;
    U32 u32_SpareDMAAddr = 0, u32_SpareMapAddr = 0;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;
	U8 u8_RetryCnt_FifoClkRdy = 0;
	U32 u32_Err;

    NC_PAD_SWITCH(pNandDrv->u8_PadMode);
    NC_CLK_SETTING(pNandDrv->u32_Clk);
    NC_RECONFIG();


    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_CMDREG8);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (0xE0<<8) | 0x05);


    NC_SET_DDR_MODE();
nand_redo_read_sect:
	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END|BIT_MIU_LAST_DONE);

    #if defined(FCIE_LFSR) && FCIE_LFSR
    REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
    REG_SET_BITS_UINT16(NC_LFSR_CTRL,
        ((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
    #endif


	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt*u32_SectorCnt, READ_FROM_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)	& BIT_MIU1_SELECT)		//main data miu1
		{
			#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
			#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
		}
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		}
		else
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}
	REG_WRITE_UINT16(NC_JOB_BL_CNT, u32_SectorCnt);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);

	u32_Err=NC_waitFifoClkReady();
	if(u32_Err != UNFD_ST_SUCCESS)
	{
	#if IF_IP_VERIFY == 0
		NC_ReConfig();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}
		if(++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT)
			goto nand_redo_read_sect;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			NC_DumpRegisters();
			NC_DumpDebugBus();
			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
	#endif
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}

		NC_CLR_DDR_MODE();
		return u32_Err;
	}

	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

    REG_READ_UINT16(NC_ECC_CTRL, u16_Tmp);
    if (u16_Tmp & BIT_NC_PAGE_SIZE_MASK)
    {
        REG_WRITE_UINT16(NC_PART_MODE, BIT_PARTIAL_MODE_EN);
        REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_CNT_MASK);
        REG_SET_BITS_UINT16(NC_PART_MODE, (u32_SectorCnt-1)<<BIT_START_SECTOR_CNT_SHIFT);
        REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_IDX_MASK);
        REG_SET_BITS_UINT16(NC_PART_MODE, u8_SectorInPage<<BIT_START_SECTOR_IDX_SHIFT);
    }
	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
    REG_WRITE_UINT16(NC_AUXREG_DAT, u8_SectorInPage << pNandDrv->u8_SectorByteCntBits);
    REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
    REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	REG_WRITE_UINT16(NC_AUXREG_DAT, ((OP_ADR_CYCLE_01|OP_ADR_TYPE_COL|OP_ADR_SET_0)<< 8) | (CMD_REG8L));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN<< 8) | (CMD_REG8H));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_BREAK);

	REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START) & ~(BIT_NC_IF_DIR_W));
    if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME) == (WAIT_READ_TIME))
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadSectorsWithOP_Transfer Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
		#if 0==IF_IP_VERIFY
        NC_ReConfig();
        NC_CLR_DDR_MODE();
        NC_ResetNandFlash();

        nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
	        nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		#else
        //nand_stop();
		#endif
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

        REG_WRITE_UINT16(NC_PART_MODE, 0);
		return UNFD_ST_ERR_R_TIMEOUT;
    }

	REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

    FLUSH_MIU_PIPE(); // Only used in U4 now
    //-----------------------------------
	#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
	// check until MIU is done
	 if( (u32_Err=NC_wait_MIULastDone()) != UNFD_ST_SUCCESS)
	 {
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		return u32_Err;
	 }
	#endif

    // Check ECC
    REG_READ_UINT16(NC_ECC_STAT0, u16_Tmp);
    if (u16_Tmp & BIT_NC_ECC_FAIL)
    {
        NC_ReConfig();
        NC_CLR_DDR_MODE();
        NC_ResetNandFlash();

        nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
        NC_CLR_DDR_MODE();
        REG_WRITE_UINT16(NC_PART_MODE, 0);

        //return NC_ReadSectors_MTD(u32_PhyRowIdx, u8_SectorInPage, pu8_DataBuf, pu8_SpareBuf, u32_SectorCnt);;
        return UNFD_ST_ERR_ECC_FAIL;
    }

    //-----------------------------------
    nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);

	if (pu8_SpareBuf && u8_IsSpareDMA == 0)
	{
	    #if SPARE640B_CIFD512B_PATCH
		if(pNandDrv->u16_SectorSpareByteCnt*u32_SectorCnt > 0x200)
			NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
		else
	    #endif
		NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SectorSpareByteCnt*u32_SectorCnt);
	}

    NC_CLR_DDR_MODE();

    #if defined( DDR_NAND_SUPPORT_RETRY_DQS) && DDR_NAND_SUPPORT_RETRY_DQS
    if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
        if(0!=u8_RetryDqs)
            nand_retry_dqs_post();
    #endif

    if(gu32_monitor_read_enable)
        printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);

    if(gu32_monitor_count_enable)
        gu32_monitor_read_count++;

    REG_WRITE_UINT16(NC_PART_MODE, 0);

    return UNFD_ST_SUCCESS;
}

unsigned int NC_ReadSectorsWithOP_Transfer_Hynix(unsigned int u32_PhyRowIdx, unsigned char u8_SectorInPage, unsigned char *pu8_DataBuf, unsigned char *pu8_SpareBuf, unsigned int u32_SectorCnt)
{
    U16 u16_Tmp=0;
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    U32 u32_DataDMAAddr, u32_DataMapAddr;
    U32 u32_SpareDMAAddr = 0, u32_SpareMapAddr = 0;
	U8	u8_IsSpareDMA = (IF_SPARE_DMA() != 0);
	U8 	*pu8_OrigSpareBuf = pu8_SpareBuf;
	U8 u8_RetryCnt_FifoClkRdy = 0;
	U32 u32_Err;

    NC_PAD_SWITCH(pNandDrv->u8_PadMode);
    NC_CLK_SETTING(pNandDrv->u32_Clk);
    NC_RECONFIG();


    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_CMDREG8);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (0xE0<<8) | 0x05);

    NC_SET_DDR_MODE();
nand_redo_read_sect:
	REG_WRITE_UINT16(NC_MIE_EVENT, BIT_NC_JOB_END|BIT_MMA_DATA_END|BIT_MIU_LAST_DONE);

    #if defined(FCIE_LFSR) && FCIE_LFSR
    REG_CLR_BITS_UINT16(NC_LFSR_CTRL, BIT_SEL_PAGE_MASK);
    REG_SET_BITS_UINT16(NC_LFSR_CTRL,
        ((u32_PhyRowIdx & pNandDrv->u16_BlkPageCntMask) & BIT_SEL_PAGE_MASK>>BIT_SEL_PAGE_SHIFT)<<BIT_SEL_PAGE_SHIFT);
    #endif


	u32_DataMapAddr = nand_DMA_MAP_address((void*)pu8_DataBuf, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
	u32_DataDMAAddr = nand_translate_DMA_address_Ex(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt*u32_SectorCnt, READ_FROM_NAND);

	if(u8_IsSpareDMA)
	{
		if(REG(NC_MIU_DMA_SEL)	& BIT_MIU1_SELECT)		//main data miu1
		{
			#if defined(CONFIG_MIPS)
			printk(KERN_CRIT"FCIE does not allow spare dma to miu1 for mips chip\n");
			nand_die();
			#endif
			pu8_SpareBuf = pNandDrv->pu8_Miu1SpareBuf;
		}
		if (pu8_SpareBuf)
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pu8_SpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		}
		else
		{
			u32_SpareMapAddr = nand_DMA_MAP_address((void*)pNandDrv->PlatCtx_t.pu8_PageSpareBuf, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			u32_SpareDMAAddr = nand_translate_DMA_address_Ex(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		}
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR0, u32_SpareDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SPARE_DMA_ADR1, u32_SpareDMAAddr >>16);
	}
	REG_WRITE_UINT16(NC_JOB_BL_CNT, u32_SectorCnt);
	REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
	REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
	REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);

	u32_Err=NC_waitFifoClkReady();
	if(u32_Err != UNFD_ST_SUCCESS)
	{
	#if IF_IP_VERIFY == 0
		NC_ReConfig();
		NC_ResetNandFlash();
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}
		if(++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT)
			goto nand_redo_read_sect;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			NC_DumpRegisters();
			NC_DumpDebugBus();
			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
	#endif
		nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
		{
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
			if(pu8_SpareBuf != pu8_OrigSpareBuf && pu8_OrigSpareBuf != NULL)
				memcpy(pu8_OrigSpareBuf, pu8_SpareBuf, pNandDrv->u16_SpareByteCnt);
		}

		NC_CLR_DDR_MODE();
		return u32_Err;
	}

	REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);

    REG_READ_UINT16(NC_ECC_CTRL, u16_Tmp);
    if (u16_Tmp & BIT_NC_PAGE_SIZE_MASK)
    {
        REG_WRITE_UINT16(NC_PART_MODE, BIT_PARTIAL_MODE_EN);
        REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_CNT_MASK);
        REG_SET_BITS_UINT16(NC_PART_MODE, (u32_SectorCnt-1)<<BIT_START_SECTOR_CNT_SHIFT);
        REG_CLR_BITS_UINT16(NC_PART_MODE, BIT_START_SECTOR_IDX_MASK);
        REG_SET_BITS_UINT16(NC_PART_MODE, u8_SectorInPage<<BIT_START_SECTOR_IDX_SHIFT);
    }
	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
    REG_WRITE_UINT16(NC_AUXREG_DAT, 0);					//keep low
    REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
    REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	REG_WRITE_UINT16(NC_AUXREG_DAT, (pNandDrv->u8_OpCode_RW_AdrCycle<< 8) | (CMD_0x00));
	REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);
    if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_READ_TIME) == (WAIT_READ_TIME))
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadSectorsWithOP_Transfer Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
		#if 0==IF_IP_VERIFY
        NC_ReConfig();
        NC_CLR_DDR_MODE();
        NC_ResetNandFlash();

        nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
	        nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		#else
        //nand_stop();
		#endif
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq
        REG_WRITE_UINT16(NC_PART_MODE, 0);
		return UNFD_ST_ERR_R_TIMEOUT;
    }
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
    REG_WRITE_UINT16(NC_AUXREG_DAT, u8_SectorInPage << pNandDrv->u8_SectorByteCntBits);
    REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx & 0xFFFF);
    REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PhyRowIdx >> 16);

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	REG_WRITE_UINT16(NC_AUXREG_DAT, ((OP_ADR_CYCLE_01|OP_ADR_TYPE_COL|OP_ADR_SET_0)<< 8) | (CMD_REG8L));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_SER_DIN<< 8) | (CMD_REG8H));
	REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK << 8) | ACT_BREAK);

	REG_WRITE_UINT16(NC_CTRL, (BIT_NC_CIFD_ACCESS|BIT_NC_JOB_START) & ~(BIT_NC_IF_DIR_W));
    if (NC_WaitComplete(BIT_NC_JOB_END|BIT_MMA_DATA_END, WAIT_READ_TIME) == (WAIT_READ_TIME))
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_ReadSectorsWithOP_Transfer Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
		#if 0==IF_IP_VERIFY
        NC_ReConfig();
        NC_CLR_DDR_MODE();
        NC_ResetNandFlash();

        nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
	        nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
		#else
        //nand_stop();
		#endif
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

        REG_WRITE_UINT16(NC_PART_MODE, 0);
		return UNFD_ST_ERR_R_TIMEOUT;
    }

	REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

    FLUSH_MIU_PIPE(); // Only used in U4 now
    //-----------------------------------
	#if (defined(MIU_CHECK_LAST_DONE)&&MIU_CHECK_LAST_DONE)
	// check until MIU is done
	 if( (u32_Err=NC_wait_MIULastDone()) != UNFD_ST_SUCCESS)
	 {
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();
		NC_UNLOCK_FCIE();
		return u32_Err;
	 }
	#endif

    // Check ECC
    REG_READ_UINT16(NC_ECC_STAT0, u16_Tmp);
    if (u16_Tmp & BIT_NC_ECC_FAIL)
    {
        NC_ReConfig();
        NC_CLR_DDR_MODE();
        NC_ResetNandFlash();

        nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
		if(u8_IsSpareDMA)
			nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);
        NC_CLR_DDR_MODE();
        REG_WRITE_UINT16(NC_PART_MODE, 0);

        //return NC_ReadSectors_MTD(u32_PhyRowIdx, u8_SectorInPage, pu8_DataBuf, pu8_SpareBuf, u32_SectorCnt);;
        return UNFD_ST_ERR_ECC_FAIL;
    }

    //-----------------------------------
    nand_DMA_UNMAP_address(u32_DataMapAddr, pNandDrv->u16_SectorByteCnt * u32_SectorCnt, READ_FROM_NAND);
	if(u8_IsSpareDMA)
		nand_DMA_UNMAP_address(u32_SpareMapAddr, pNandDrv->u16_SpareByteCnt, READ_FROM_NAND);

	if (pu8_SpareBuf && u8_IsSpareDMA == 0)
	{
	    #if SPARE640B_CIFD512B_PATCH
		if(pNandDrv->u16_SectorSpareByteCnt*u32_SectorCnt > 0x200)
			NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
		else
	    #endif
		NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SectorSpareByteCnt*u32_SectorCnt);
	}

    NC_CLR_DDR_MODE();

    #if defined( DDR_NAND_SUPPORT_RETRY_DQS) && DDR_NAND_SUPPORT_RETRY_DQS
    if(pNandDrv->u16_Reg58_DDRCtrl & BIT_DDR_MASM)
        if(0!=u8_RetryDqs)
            nand_retry_dqs_post();
    #endif

    if(gu32_monitor_read_enable)
        printk(KERN_CRIT"[%s]%Xh\n", __func__, u32_PhyRowIdx);

    if(gu32_monitor_count_enable)
    {
        gu32_monitor_read_count++;
        gu64_monitor_read_size += (pNandDrv->u16_SectorByteCnt * u32_SectorCnt);
    }

    REG_WRITE_UINT16(NC_PART_MODE, 0);

    return UNFD_ST_SUCCESS;
}

unsigned int NC_ReadSectorsWithOP_Transfer(unsigned int u32_PhyRowIdx, unsigned char u8_SectorInPage, unsigned char *pu8_DataBuf, unsigned char *pu8_SpareBuf, unsigned int u32_SectorCnt)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
	if(pNandDrv->au8_ID[0] == 0xAD || pNandDrv->au8_ID[0] == 0x98)
		return NC_ReadSectorsWithOP_Transfer_1p(u32_PhyRowIdx, u8_SectorInPage, pu8_DataBuf, pu8_SpareBuf, u32_SectorCnt);
	else if(pNandDrv->au8_ID[0] == 0x2C)
		return NC_ReadSectorsWithOP_Transfer_Micron(u32_PhyRowIdx, u8_SectorInPage, pu8_DataBuf, pu8_SpareBuf, u32_SectorCnt);
	else
		return UNFD_ST_ERR_INVALID_PARAM;
}

void NC_SendCmd(unsigned char u8_Cmd, int WaitRB)
{
    NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();
    pNandDrv = pNandDrv;
    NC_SET_DDR_MODE();

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_CMDREG8);
    REG_WRITE_UINT16(NC_AUXREG_DAT, u8_Cmd);

    REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	if(WaitRB)
	{
	    REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_WAITRB<<8)|CMD_REG8L);
	    REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK<<8)|ACT_BREAK);
	}
	else
	    REG_WRITE_UINT16(NC_AUXREG_DAT, (ACT_BREAK<<8)|CMD_REG8L);

	REG_CLR_BITS_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//disable irq

    REG_WRITE_UINT16(NC_CTRL, BIT_NC_JOB_START);

	 if (NC_WaitComplete(BIT_NC_JOB_END, WAIT_READ_TIME) == (WAIT_READ_TIME))
    {
        nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: NC_SendCmd Timeout, ErrCode:%Xh \r\n", UNFD_ST_ERR_R_TIMEOUT);
		REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//Enable irq
        NC_ResetFCIE();
        NC_Config();
        NC_ResetNandFlash();
    }
	REG_WRITE_UINT16(NC_MIE_INT_EN, BIT_NC_JOB_END|BIT_MMA_DATA_END);	//Enable irq
    NC_CLR_DDR_MODE();
	return;
}



typedef struct CustCmd
{
	U16 CmdAddr;
	U16 CmdValue;
	U16 CmdReg;
} CustCmd_t;

int NC_SearchCustCmd(CustCmd_t *Cust, U16 CmdValue, int CmdCount)
{
	int i;
	for(i = 0; i < CmdCount; i++)
	{
		if(Cust[i].CmdValue == CmdValue)
			return Cust[i].CmdReg;
	}
	return -1;
}

int NC_SearchAddrCmd(U8 *pu8_AddrCmd, U8 CmdValue, int CmdCount)
{
	int i;
	for(i = 0; i < CmdCount; i++)
	{
		if(pu8_AddrCmd[i] == CmdValue)
			return 1;
	}
	return 0;
}

U32 NC_SendCustCmd
(
	U8* pu8_CmdType, 
	U8* pu8_CmdValue, 
	U32* pu32_AddrRow, 
	U32* pu32_AddrCol, 
	U8 *pu8_DataBuf, U8 *pu8_SpareBuf,
	U8 u8_CmdCount, U8 u8_CacheOp, U8 u8_plane, U32 u32_PageCnt
)
{
	NAND_DRIVER *pNandDrv = (NAND_DRIVER*)drvNAND_get_DrvContext_address();

	U8 u8_i, u8_AddrCmdCount = 0, u8_CmdShift, u8_ValidCmd = 0,
		u8_CustCmdAddr = 0x8, u8_CustCmdCount = 0;
	U8 u8_HasRANByte = 0, u8_HasDMAIOCmd = 0, u8_RWDirect = 0;
	U8  u8_RetryCnt_FifoClkRdy=0, u8_RetryCnt_Events=0, u8_RetryCnt_ECCFail=0;
	U16 u16_Cmd, u16_Tmp, u16_RANByteCnt, u16_Tmp2;
	U32 u32_Err, u32_WaitTimeUnit;
    U32 u32_DataDMAAddr;

    //U32 u32_SpareDMAAddr=0;

	CustCmd_t a_CustCmd[6];
	U8 au8_AddrCmd[4];

nand_redo:
	u16_Cmd = 0;
	u32_Err = 0;
	u16_RANByteCnt = 0;
	u8_AddrCmdCount = 0;
	u8_HasDMAIOCmd = 0;
	u8_ValidCmd = 0;
	u8_CustCmdCount = 0;
	u8_RWDirect = 0;
	u8_CustCmdAddr = 0x8;
	memset(a_CustCmd, 0, sizeof (a_CustCmd));
	memset(au8_AddrCmd, 0, sizeof (au8_AddrCmd));

	//two parse of the Cust Command Array

	//first parse for Custom Command which does not be included in FCIE

	for(u8_i = 0; u8_i < u8_CmdCount; u8_i++)
	{
		//check if cmd type is custom command, which doesn't be included in existent custom command queue 'a_CustCmd'
		if(pu8_CmdType[u8_i] == CUST_CMD && NC_SearchCustCmd(a_CustCmd, pu8_CmdValue[u8_i], 6) == -1) 
		{
			//custom command register could contain 2 custom commands. When even count custom command, write to custom command register.
			if(u8_CustCmdCount % 2 == 0)
			{
				REG_WRITE_UINT16(NC_AUXREG_ADR, u8_CustCmdAddr);
				u8_CustCmdAddr ++;
			}
			u16_Cmd = (u8_CustCmdCount%2) ? u16_Cmd : 0;
			u8_CmdShift = (u8_CustCmdCount%2) ? 8 : 0;

			u16_Tmp = 0;
			u16_Tmp |= (pu8_CmdValue[u8_i] << u8_CmdShift);

			u16_Cmd |= u16_Tmp;
			a_CustCmd[u8_CustCmdCount].CmdAddr = u8_CustCmdAddr;
			a_CustCmd[u8_CustCmdCount].CmdReg = u8_CustCmdCount + 0x10;
			a_CustCmd[u8_CustCmdCount].CmdValue = pu8_CmdValue[u8_i];
			u8_CustCmdCount ++;

			if(u8_CustCmdCount % 2 == 0)
			{
				//printf("Add CustCmd %X\n", u16_Cmd);
				REG_WRITE_UINT16(NC_AUXREG_DAT, u16_Cmd);
			}
		}
		if(pu8_CmdType[u8_i] == ADDR_CYCLE_CMD)
		{
			if(!NC_SearchAddrCmd(au8_AddrCmd, pu8_CmdValue[u8_i], 4))
			{
				au8_AddrCmd[u8_AddrCmdCount] = pu8_CmdValue[u8_i];
				u8_AddrCmdCount ++;
			}	
		}
	}

	if(u8_CustCmdCount % 2)
	{
		//printf("Add CustCmd %X\n", u16_Cmd);
		REG_WRITE_UINT16(NC_AUXREG_DAT, u16_Cmd);
	}

	//second parse for Instruction Queue
	
	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_INSTQUE);
	#if defined(FCIE_EXTEND_tWW) && FCIE_EXTEND_tWW
	REG_WRITE_UINT16(NC_AUXREG_DAT, ACT_WAIT_IDLE | ACT_WAIT_IDLE);	
	#endif
	for(u8_i = 0; u8_i < u8_CmdCount; u8_i++)
	{
		u16_Cmd = (u8_ValidCmd%2) ? u16_Cmd : 0;
		u8_CmdShift = (u8_ValidCmd%2) ? 8 : 0;
		if(pu8_CmdType[u8_i] <= ADDR_CYCLE_CMD)
		{
			u16_Tmp = 0;
			u16_Tmp |= (pu8_CmdValue[u8_i] << u8_CmdShift);
			u8_ValidCmd ++;
		}
		else if(pu8_CmdType[u8_i] == ACT_DMA_CMD)
		{
			u16_Tmp = 0;
			u16_Tmp |= (pu8_CmdValue[u8_i] << u8_CmdShift);
			u8_HasDMAIOCmd = 1;
			u8_ValidCmd ++;
			if(pu8_CmdValue[u8_i] == ACT_SER_DIN && u8_RWDirect != 2)
				u8_RWDirect = 1;
			else if(pu8_CmdValue[u8_i] == ACT_SER_DOUT && u8_RWDirect != 1)
				u8_RWDirect = 2;
			else
			{
				nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "Invalid Command for Command Type \"ACT_DMA_CMD \" u8_RWDirect = %X \n",u8_RWDirect);
				return UNFD_ST_ERR_INVALID_PARAM;
			}
		}
		else if(pu8_CmdType[u8_i] == ACT_RAN_CMD)
		{
			u16_Tmp = 0;
			u16_Tmp |= (pu8_CmdValue[u8_i] << u8_CmdShift);
			u8_HasRANByte = 1;
			u8_ValidCmd ++;
			if(pu8_CmdValue[u8_i] == ACT_RAN_DIN && u8_RWDirect != 2)
				u8_RWDirect = 1;
			else if(pu8_CmdValue[u8_i] == ACT_RAN_DOUT && u8_RWDirect != 1)
				u8_RWDirect = 2;
			else
			{
				nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "Invalid Command for Command Type \"ACT_RAN_CMD \" u8_RWDirect = %X \n", u8_RWDirect);
				return UNFD_ST_ERR_INVALID_PARAM;
			}
		}		
		else if(pu8_CmdType[u8_i] == CUST_CMD)
		{
			U8 u8_CustCmdReg;
			u8_CustCmdReg = NC_SearchCustCmd(a_CustCmd, pu8_CmdValue[u8_i], 6);
			u16_Tmp = 0;
			u16_Tmp |= (u8_CustCmdReg << u8_CmdShift);
			u8_ValidCmd ++;
		}
		else if(pu8_CmdType[u8_i] == RAN_BYTE_CMD)
		{
			u16_RANByteCnt = pu8_CmdValue[u8_i];
			u16_Tmp = 0;
			continue;
		}		
		else
		{
			//unknonwn
			nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "Unknown Type of Instruction Cmd \n");
			return UNFD_ST_ERR_INVALID_PARAM;
		}
		u16_Cmd |= u16_Tmp;
		if(u8_ValidCmd % 2 == 0)
		{
		//	printf("Add InstrQue %X\n", u16_Cmd);
			REG_WRITE_UINT16(NC_AUXREG_DAT, u16_Cmd);
		}
	}
	if(u8_ValidCmd % 2)
	{
	//	printf("Add InstrQue %X\n", u16_Cmd);
		REG_WRITE_UINT16(NC_AUXREG_DAT, u16_Cmd);
	}

	if(u8_HasRANByte == 1 && u8_HasDMAIOCmd == 1)
	{
		nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "Instruction Queue can't have DMA and Random Byte I/O at the same time.\n");
		return UNFD_ST_ERR_INVALID_PARAM;
	}

	REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_ADRSET);
	for(u8_i = 0 ; u8_i < u8_AddrCmdCount ; u8_i++)
	{	
		REG_WRITE_UINT16(NC_AUXREG_DAT, pu32_AddrCol[u8_i] & 0xFFFF);
		REG_WRITE_UINT16(NC_AUXREG_DAT, pu32_AddrRow[u8_i] & 0xFFFF);
		REG_WRITE_UINT16(NC_AUXREG_DAT, pu32_AddrRow[u8_i] >> 16);
	}

	if(u8_HasDMAIOCmd)
	{
		if(u8_RWDirect == 1)
		    u32_DataDMAAddr = nand_translate_DMA_address_Ex((U32)pu8_DataBuf, pNandDrv->u16_PageByteCnt * u8_plane * u32_PageCnt, READ_FROM_NAND );
		else if(u8_RWDirect == 2)
			u32_DataDMAAddr = nand_translate_DMA_address_Ex((U32)pu8_DataBuf, pNandDrv->u16_PageByteCnt * u8_plane * u32_PageCnt, WRITE_TO_NAND );
		else
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "Invalid RW Direction should be 1 (read) or 2 (write)\n");
			return UNFD_ST_ERR_INVALID_PARAM;
		}
		REG_WRITE_UINT16(NC_SDIO_ADDR0, u32_DataDMAAddr & 0xFFFF);
		REG_WRITE_UINT16(NC_SDIO_ADDR1, u32_DataDMAAddr >> 16);
		REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RPTCNT);
		REG_WRITE_UINT16(NC_AUXREG_DAT, u32_PageCnt - (u8_CacheOp + 1));

		REG_WRITE_UINT16(NC_JOB_BL_CNT, (u8_plane * u32_PageCnt) << pNandDrv->u8_PageSectorCntBits);

		if(u8_RWDirect == 1)
		{
			REG_CLR_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
		}
		else if(u8_RWDirect == 2)
		{
			REG_SET_BITS_UINT16(NC_MMA_PRI_REG, BIT_NC_DMA_DIR_W);
		}
		else
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "Invalid RW Direction should be 1 (read) or 2 (write)\n");
			return UNFD_ST_ERR_INVALID_PARAM;
		}

		u32_Err=NC_waitFifoClkReady();
		if( u32_Err != UNFD_ST_SUCCESS)
		{
			NC_ResetFCIE();
			NC_Config();
			NC_ResetNandFlash();
			
			if( ++u8_RetryCnt_FifoClkRdy < NAND_TIMEOUT_RETRY_CNT )
				goto nand_redo;
			else
			{
				//If soft reset still can not solve this problem, show an alert and return a error
				nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
				
				NC_CLR_DDR_MODE();
				NC_UNLOCK_FCIE();
				return u32_Err;
			}
		}
		REG_SET_BITS_UINT16(NC_PATH_CTL, BIT_MMA_EN);
	}

	if(u8_HasRANByte)
	{
		REG_WRITE_UINT16(NC_AUXREG_ADR, AUXADR_RAN_CNT);
		REG_WRITE_UINT16(NC_AUXREG_DAT, u16_RANByteCnt);
		REG_WRITE_UINT16(NC_AUXREG_DAT, 0);	// offset;
	}

	//set CIFD

	if(u8_HasRANByte && u8_RWDirect == 2)
	{
		NC_SetCIFD(pu8_DataBuf, 0, u16_RANByteCnt);
		if (u16_RANByteCnt & 1)
		{
			U8 au8_Tmp[1];
			au8_Tmp[0] = 0xFF; // pad a 0xFF
			NC_SetCIFD(au8_Tmp, u16_RANByteCnt, 1);
		}
	}
	else if(u8_HasDMAIOCmd && u8_RWDirect == 2)
	{
		if(IF_SPARE_DMA() == 0)
		{
			if (pu8_SpareBuf)
				for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
					NC_SetCIFD(pu8_SpareBuf + pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
			        		pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
			        		pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
			else
				for (u16_Tmp=0; u16_Tmp < pNandDrv->u16_PageSectorCnt; u16_Tmp++)
					NC_SetCIFD_Const(0xFF, pNandDrv->u16_SectorSpareByteCnt * u16_Tmp,
							pNandDrv->u16_SectorSpareByteCnt - pNandDrv->u16_ECCCodeByteCnt);
		}
	}

	u16_Tmp = (BIT_NC_JOB_START);
	u16_Tmp2 = BIT_NC_JOB_END;
	if(u8_HasDMAIOCmd)
	{
		if(u8_RWDirect == 1)
		{
			u16_Tmp &= ~(BIT_NC_IF_DIR_W);
			u32_WaitTimeUnit = WAIT_READ_TIME * u32_PageCnt;
		}
		else if(u8_RWDirect == 2)
		{
			u16_Tmp |= BIT_NC_IF_DIR_W;
			u32_WaitTimeUnit = WAIT_WRITE_TIME * u32_PageCnt;
		}
		else
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "Invalid RW Direction should be either 1 (read) or 2 (write) for DMA\n");
			return UNFD_ST_ERR_INVALID_PARAM;
		}
		u16_Tmp2 |= BIT_MMA_DATA_END;
		u16_Tmp |= BIT_NC_CIFD_ACCESS;
	}
	else if(u8_HasRANByte)
	{
		if(u8_RWDirect == 1)
			u32_WaitTimeUnit = WAIT_READ_TIME;
		else if(u8_RWDirect == 2)
			u32_WaitTimeUnit = WAIT_WRITE_TIME;
		else
		{
			nand_debug(UNFD_DEBUG_LEVEL_ERROR,1, "Invalid RW Direction should be either 1 (read) or 2 (write) for Random Byte IO\n");
			return UNFD_ST_ERR_INVALID_PARAM;
		}
		u16_Tmp |= BIT_NC_CIFD_ACCESS;
	}
	else
	{
		u32_WaitTimeUnit = WAIT_ERASE_TIME;
	}
	REG_WRITE_UINT16(NC_CTRL, u16_Tmp);

	if(NC_WaitComplete(u16_Tmp2, u32_WaitTimeUnit) == u32_WaitTimeUnit)
	{
		if((gu8_DisableErrorLog == 0)&&(u8_RetryCnt_Events==(NAND_TIMEOUT_RETRY_CNT-1)))
		{
			NC_DumpDebugBus();
			NC_DumpRegisters();
		}
		NC_ResetFCIE();
		NC_Config();
		NC_ResetNandFlash();

		if( ++u8_RetryCnt_Events < NAND_TIMEOUT_RETRY_CNT )
			goto nand_redo;
		else
		{
			//If soft reset still can not solve this problem, show an alert and return a error
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "\033[31mSoft reset over %d times\n, stop!\033[m\n", NAND_TIMEOUT_RETRY_CNT);
			
			NC_CLR_DDR_MODE();
			NC_UNLOCK_FCIE();
			return u32_Err;
		}
	}

	if(u8_HasRANByte && u8_RWDirect == 1)
	{
		NC_GetCIFD(pu8_DataBuf, 0, u16_RANByteCnt);
	}
	else if (u8_HasDMAIOCmd && u8_RWDirect == 1)
	{
		//check ECC
		REG_READ_UINT16(NC_ECC_STAT0, u16_Tmp);
		if (u16_Tmp & BIT_NC_ECC_FAIL)
		{

			if(IF_SPARE_DMA() == 0)
			{
				if (pu8_SpareBuf)
				{
					#if SPARE640B_CIFD512B_PATCH
					if(pNandDrv->u16_SpareByteCnt > 0x200)
						NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
					else
					#endif
						NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SpareByteCnt);
				}
			}

			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Error: %s ECC Fail, Reg51:%04Xh ",__func__, u16_Tmp);
			REG_READ_UINT16(NC_ECC_STAT1, u16_Tmp);
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "Reg52:%04Xh ", u16_Tmp);
			REG_READ_UINT16(NC_ECC_STAT2, u16_Tmp);
			nand_debug(UNFD_DEBUG_LEVEL_ERROR, 0, "Reg53:%04Xh \r\n", u16_Tmp);
			// add retry for ECC error
			if( u8_RetryCnt_ECCFail < NAND_TIMEOUT_RETRY_CNT)
			{
				u8_RetryCnt_ECCFail++;
				nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "Retry times : %X\n", u8_RetryCnt_ECCFail);
				goto nand_redo;
			}
			else
			{
				nand_debug(UNFD_DEBUG_LEVEL_ERROR, 1, "ERROR, NAND, RETRY_READ_ECC_FAIL over %d times \n", NAND_TIMEOUT_RETRY_CNT);
				NC_UNLOCK_FCIE();
			}
			NC_CLR_DDR_MODE();
			return UNFD_ST_ERR_ECC_FAIL;
		}

		if (pu8_SpareBuf && IF_SPARE_DMA() == 0)
		{
			#if SPARE640B_CIFD512B_PATCH
			if(pNandDrv->u16_SpareByteCnt > 0x200)
				NC_GetCIFD(pu8_SpareBuf, 0, 0x200);
			else
			#endif
			NC_GetCIFD(pu8_SpareBuf, 0, pNandDrv->u16_SpareByteCnt);
		}
	}
	
	return UNFD_ST_SUCCESS;
}

#endif // NC_SEL_FCIE3
