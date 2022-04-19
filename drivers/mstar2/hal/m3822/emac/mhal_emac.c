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
/// @file   Mhal_emac.c
/// @brief  EMAC Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/mii.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/pci.h>
#include "mhal_emac.h"

extern unsigned char phyaddr;

//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------
struct _MHalBasicConfigEMAC
{
    u8 connected;           // 0:No, 1:Yes
    u8 speed;               // 10:10Mbps, 100:100Mbps
    // ETH_CTL Register:
    u8 wes;                 // 0:Disable, 1:Enable (WR_ENABLE_STATISTICS_REGS)
    // ETH_CFG Register:
    u8 duplex;              // 1:Half-duplex, 2:Full-duplex
    u8 cam;                 // 0:No CAM, 1:Yes
    u8 rcv_bcast;           // 0:No, 1:Yes
    u8 rlf;                 // 0:No, 1:Yes receive long frame(1522)
    // MAC Address:
    u8 sa1[6];              // Specific Addr 1 (MAC Address)
    u8 sa2[6];              // Specific Addr 2
    u8 sa3[6];              // Specific Addr 3
    u8 sa4[6];              // Specific Addr 4
};
typedef struct _MHalBasicConfigEMAC MHalBasicConfigEMAC;

struct _MHalUtilityVarsEMAC
{
    u32 cntChkCableConnect;
    u32 cntChkINTCounter;
    u32 readIdxRBQP;        // Reset = 0x00000000
    u32 rxOneFrameAddr;     // Reset = 0x00000000 (Store the Addr of "ReadONE_RX_Frame")
    u8 flagISR_INT_DONE;
};
typedef struct _MHalUtilityVarsEMAC MHalUtilityVarsEMAC;

MHalBasicConfigEMAC MHalThisBCE;
MHalUtilityVarsEMAC MHalThisUVE;

#define MHal_MAX_INT_COUNTER    100
//-------------------------------------------------------------------------------------------------
//  EMAC hardware for Titania
//-------------------------------------------------------------------------------------------------

/*8-bit RIU address*/
u8 MHal_EMAC_ReadReg8( u32 bank, u32 reg )
{
    u8 val;
    u32 address = EMAC_RIU_REG_BASE + bank*0x100*2;
    address = address + (reg << 1) - (reg & 1);

    val = *( ( volatile u8* ) address );
    return val;
}

void MHal_EMAC_WritReg8( u32 bank, u32 reg, u8 val )
{
    u32 address = EMAC_RIU_REG_BASE + bank*0x100*2;
    address = address + (reg << 1) - (reg & 1);

    *( ( volatile u8* ) address ) = val;
}
#ifdef EMAC_SUPPORT_XIU32
u32 MHal_EMAC_ReadReg32( u32 xoffset )
{
    u32 address = REG_ADDR_BASE + xoffset*2;

    u32 xoffsetValue = *( ( volatile u32* ) address ) & 0xFFFFFFFF;
    return( xoffsetValue );
}

u32 MHal_EMAC_ReadReg32_EMAC1( u32 xoffset )
{
    u32 address = REG_ADDR_BASE_EMAC1 + xoffset*2;

    u32 xoffsetValueL = *( ( volatile u32* ) address ) & 0x0000FFFF;
    u32 xoffsetValueH = *( ( volatile u32* ) ( address + 4) ) << 0x10;
    return( xoffsetValueH | xoffsetValueL );
}

void MHal_EMAC_WritReg32( u32 xoffset, u32 xval )
{
    u32 address = REG_ADDR_BASE + xoffset*2;

    *( ( volatile u32 * ) address ) = ( u32 ) ( xval & 0xFFFFFFFF );
}

void MHal_EMAC_WritReg32_EMAC1( u32 xoffset, u32 xval )
{
    u32 address = REG_ADDR_BASE_EMAC1 + xoffset*2;

    *( ( volatile u32 * ) address ) = ( u32 ) ( xval & 0x0000FFFF );
    *( ( volatile u32 * ) ( address + 4 ) ) = ( u32 ) ( xval >> 0x10 );
}
#else
u32 MHal_EMAC_ReadReg32( u32 xoffset )
{
    u32 address = REG_ADDR_BASE + xoffset*2;

    u32 xoffsetValueL = *( ( volatile u32* ) address ) & 0x0000FFFF;
    u32 xoffsetValueH = *( ( volatile u32* ) ( address + 4) ) << 0x10;
    return( xoffsetValueH | xoffsetValueL );
}

void MHal_EMAC_WritReg32( u32 xoffset, u32 xval )
{
    u32 address = REG_ADDR_BASE + xoffset*2;

    *( ( volatile u32 * ) address ) = ( u32 ) ( xval & 0x0000FFFF );
    *( ( volatile u32 * ) ( address + 4 ) ) = ( u32 ) ( xval >> 0x10 );
}
#endif

u32 MHal_EMAC_ReadRam32( u32 uRamAddr, u32 xoffset)
{
    return (*( u32 * ) ( ( char * ) uRamAddr + xoffset ) );
}

void MHal_EMAC_WritRam32( u32 uRamAddr, u32 xoffset, u32 xval )
{
    *( ( u32 * ) ( ( char * ) uRamAddr + xoffset ) ) = xval;
}

void MHal_EMAC_Write_SA1_MAC_Address( u8 m0, u8 m1, u8 m2, u8 m3, u8 m4, u8 m5 )
{
    u32 w0 = ( u32 ) m3 << 24 | m2 << 16 | m1 << 8 | m0;
    u32 w1 = ( u32 ) m5 << 8 | m4;
    MHal_EMAC_WritReg32( REG_ETH_SA1L, w0 );
    MHal_EMAC_WritReg32( REG_ETH_SA1H, w1 );
}

void MHal_EMAC_Write_SA2_MAC_Address( u8 m0, u8 m1, u8 m2, u8 m3, u8 m4, u8 m5 )
{
    u32 w0 = ( u32 ) m3 << 24 | m2 << 16 | m1 << 8 | m0;
    u32 w1 = ( u32 ) m5 << 8 | m4;
    MHal_EMAC_WritReg32( REG_ETH_SA2L, w0 );
    MHal_EMAC_WritReg32( REG_ETH_SA2H, w1 );
}

void MHal_EMAC_Write_SA3_MAC_Address( u8 m0, u8 m1, u8 m2, u8 m3, u8 m4, u8 m5 )
{
    u32 w0 = ( u32 ) m3 << 24 | m2 << 16 | m1 << 8 | m0;
    u32 w1 = ( u32 ) m5 << 8 | m4;
    MHal_EMAC_WritReg32( REG_ETH_SA3L, w0 );
    MHal_EMAC_WritReg32( REG_ETH_SA3H, w1 );
}

void MHal_EMAC_Write_SA4_MAC_Address( u8 m0, u8 m1, u8 m2, u8 m3, u8 m4, u8 m5 )
{
    u32 w0 = ( u32 ) m3 << 24 | m2 << 16 | m1 << 8 | m0;
    u32 w1 = ( u32 ) m5 << 8 | m4;
    MHal_EMAC_WritReg32( REG_ETH_SA4L, w0 );
    MHal_EMAC_WritReg32( REG_ETH_SA4H, w1 );
}

//-------------------------------------------------------------------------------------------------
//  R/W EMAC register for Titania
//-------------------------------------------------------------------------------------------------

void MHal_EMAC_update_HSH(u32 mc0, u32 mc1)
{
    MHal_EMAC_WritReg32( REG_ETH_HSL, mc0 );
    MHal_EMAC_WritReg32( REG_ETH_HSH, mc1 );
}

//-------------------------------------------------------------------------------------------------
//  Read control register
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_CTL( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_CTL );
}

//-------------------------------------------------------------------------------------------------
//  Write Network control register
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_CTL( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_CTL, xval );
}

//-------------------------------------------------------------------------------------------------
//  Read Network configuration register
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_CFG( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_CFG );
}

//-------------------------------------------------------------------------------------------------
//  Write Network configuration register
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_CFG( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_CFG, xval );
}

//-------------------------------------------------------------------------------------------------
//  Read RBQP
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_RBQP( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_RBQP );
}

//-------------------------------------------------------------------------------------------------
//  Write RBQP
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_RBQP( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_RBQP, xval );
}

//-------------------------------------------------------------------------------------------------
//  Write Transmit Address register
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_TAR( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_TAR, xval );
}

//-------------------------------------------------------------------------------------------------
//  Read RBQP
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_TCR( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_TCR);
}

//-------------------------------------------------------------------------------------------------
//  Write Transmit Control register
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_TCR( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_TCR, xval );
}

//-------------------------------------------------------------------------------------------------
//  Transmit Status Register
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_TSR( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_TSR, xval );
}

u32 MHal_EMAC_Read_TSR( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_TSR );
}

void MHal_EMAC_Write_RSR( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_RSR, xval );
}

u32 MHal_EMAC_Read_RSR( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_RSR );
}

//-------------------------------------------------------------------------------------------------
//  Read Interrupt status register
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_ISR( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_ISR, xval );
}

u32 MHal_EMAC_Read_ISR( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_ISR );
}

//-------------------------------------------------------------------------------------------------
//  Read Interrupt enable register
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_IER( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_IER );
}

//-------------------------------------------------------------------------------------------------
//  Write Interrupt enable register
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_IER( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_IER, xval );
}

//-------------------------------------------------------------------------------------------------
//  Read Interrupt disable register
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_IDR( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_IDR );
}

//-------------------------------------------------------------------------------------------------
//  Write Interrupt disable register
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_IDR( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_IDR, xval );
}

//-------------------------------------------------------------------------------------------------
//  Read Interrupt mask register
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_IMR( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_IMR );
}

//-------------------------------------------------------------------------------------------------
//  Read PHY maintenance register
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_MAN( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_MAN );
}

//-------------------------------------------------------------------------------------------------
//  Write PHY maintenance register
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_MAN( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_MAN, xval );
}

//-------------------------------------------------------------------------------------------------
//  Write Receive Buffer Configuration
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_BUFF( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_BUFF, xval );
}

//-------------------------------------------------------------------------------------------------
//  Read Receive Buffer Configuration
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_BUFF( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_BUFF );
}

//-------------------------------------------------------------------------------------------------
//  Read Receive First Full Pointer
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_RDPTR( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_BUFFRDPTR );
}

//-------------------------------------------------------------------------------------------------
//  Write Receive First Full Pointer
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_RDPTR( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_BUFFRDPTR, xval );
}

//-------------------------------------------------------------------------------------------------
//  Write Receive First Full Pointer
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_WRPTR( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_BUFFWRPTR, xval );
}

//-------------------------------------------------------------------------------------------------
//  Frames transmitted OK
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_FRA( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_FRA );
}

//-------------------------------------------------------------------------------------------------
//  Single collision frames
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_SCOL( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_SCOL );
}

//-------------------------------------------------------------------------------------------------
//  Multiple collision frames
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_MCOL( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_MCOL );
}

//-------------------------------------------------------------------------------------------------
//  Frames received OK
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_OK( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_OK );
}

//-------------------------------------------------------------------------------------------------
//  Frame check sequence errors
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_SEQE( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_SEQE );
}

//-------------------------------------------------------------------------------------------------
//  Alignment errors
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_ALE( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_ALE );
}

//-------------------------------------------------------------------------------------------------
//  Late collisions
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_LCOL( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_LCOL );
}

//-------------------------------------------------------------------------------------------------
//  Excessive collisions
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_ECOL( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_ECOL );
}

//-------------------------------------------------------------------------------------------------
//  Transmit under-run errors
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_TUE( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_TUE );
}

//-------------------------------------------------------------------------------------------------
//  Carrier sense errors
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_CSE( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_CSE );
}

//-------------------------------------------------------------------------------------------------
//  Receive resource error
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_RE( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_RE );
}

//-------------------------------------------------------------------------------------------------
//  Received overrun
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_ROVR( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_ROVR );
}

//-------------------------------------------------------------------------------------------------
//  Received symbols error
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_SE( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_SE );
}

//-------------------------------------------------------------------------------------------------
//  Excessive length errors
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_ELR( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_ELR );
}

//-------------------------------------------------------------------------------------------------
//  Receive jabbers
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_RJB( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_RJB );
}

//-------------------------------------------------------------------------------------------------
//  Undersize frames
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_USF( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_USF );
}

//-------------------------------------------------------------------------------------------------
//  SQE test errors
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_SQEE( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_SQEE );
}

//-------------------------------------------------------------------------------------------------
//  Read Julian 100
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_JULIAN_0100( void )
{
#ifndef EMAC_SUPPORT_XIU32
    return MHal_EMAC_ReadReg32( REG_EMAC_JULIAN_0100 );
#else
    return MHal_EMAC_ReadReg32_EMAC1( REG_EMAC_JULIAN_0100 );
#endif
}

//-------------------------------------------------------------------------------------------------
//  Write Julian 100
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_JULIAN_0100( u32 xval )
{
#ifndef EMAC_SUPPORT_XIU32
    MHal_EMAC_WritReg32( REG_EMAC_JULIAN_0100, xval );
#else
    MHal_EMAC_WritReg32_EMAC1( REG_EMAC_JULIAN_0100, xval );
#endif
}

//-------------------------------------------------------------------------------------------------
//  Read Julian 104
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_JULIAN_0104( void )
{
#ifndef EMAC_SUPPORT_XIU32
    return MHal_EMAC_ReadReg32( REG_EMAC_JULIAN_0104 );
#else
    return MHal_EMAC_ReadReg32_EMAC1( REG_EMAC_JULIAN_0104 );
#endif
}

//-------------------------------------------------------------------------------------------------
//  Write Julian 104
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_JULIAN_0104( u32 xval )
{
#ifndef EMAC_SUPPORT_XIU32
    MHal_EMAC_WritReg32( REG_EMAC_JULIAN_0104, xval );
#else
    MHal_EMAC_WritReg32_EMAC1( REG_EMAC_JULIAN_0104, xval );
#endif
}

//-------------------------------------------------------------------------------------------------
//  Read Julian 108
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_Read_JULIAN_0108( void )
{
#ifndef EMAC_SUPPORT_XIU32
    return MHal_EMAC_ReadReg32( REG_EMAC_JULIAN_0108 );
#else
    return MHal_EMAC_ReadReg32_EMAC1( REG_EMAC_JULIAN_0108 );
#endif
}

//-------------------------------------------------------------------------------------------------
//  Write Julian 108
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Write_JULIAN_0108( u32 xval )
{
#ifndef EMAC_SUPPORT_XIU32
    MHal_EMAC_WritReg32( REG_EMAC_JULIAN_0108, xval );
#else
    MHal_EMAC_WritReg32_EMAC1( REG_EMAC_JULIAN_0108, xval );
#endif
}

void MHal_EMAC_Set_Tx_JULIAN_T(u32 xval)
{
    u32 value;
#ifndef EMAC_SUPPORT_XIU32
    value = MHal_EMAC_ReadReg32(0x134);
    value &= 0xff0fffff;
    value |= xval << 20;

    MHal_EMAC_WritReg32(0x134, value);
#else
    value = MHal_EMAC_ReadReg32_EMAC1(0x134);
    value &= 0xff0fffff;
    value |= xval << 20;

    MHal_EMAC_WritReg32_EMAC1(0x134, value);
#endif
}

void MHal_EMAC_Set_TEST(u32 xval)
{
    u32 value = 0xffffffff;
    int i=0;

    for(i = 0x100; i< 0x160;i+=4){
#ifndef EMAC_SUPPORT_XIU32
        MHal_EMAC_WritReg32(i, value);
#else
        MHal_EMAC_WritReg32_EMAC1(i, value);
#endif
        }

}

u32 MHal_EMAC_Get_Tx_FIFO_Threshold(void)
{
#ifndef EMAC_SUPPORT_XIU32
    return (MHal_EMAC_ReadReg32(0x134) & 0x00f00000) >> 20;
#else
    return (MHal_EMAC_ReadReg32_EMAC1(0x134) & 0x00f00000) >> 20;
#endif
}

void MHal_EMAC_Set_Rx_FIFO_Enlarge(u32 xval)
{
    u32 value;
#ifndef EMAC_SUPPORT_XIU32
    value = MHal_EMAC_ReadReg32(0x134);
    value &= 0xfcffffff;
    value |= xval << 24;

    MHal_EMAC_WritReg32(0x134, value);
#else
    value = MHal_EMAC_ReadReg32_EMAC1(0x134);
    value &= 0xfcffffff;
    value |= xval << 24;

    MHal_EMAC_WritReg32_EMAC1(0x134, value);
#endif
}

u32 MHal_EMAC_Get_Rx_FIFO_Enlarge(void)
{
#ifndef EMAC_SUPPORT_XIU32
    return (MHal_EMAC_ReadReg32(0x134) & 0x03000000) >> 24;
#else
    return (MHal_EMAC_ReadReg32_EMAC1(0x134) & 0x03000000) >> 24;
#endif
}

void MHal_EMAC_Set_Miu_Priority(u32 xval)
{
    u32 value;

#ifndef EMAC_SUPPORT_XIU32
    value = MHal_EMAC_ReadReg32(0x100);
    value &= 0xfff7ffff;
    value |= xval << 19;

    MHal_EMAC_WritReg32(0x100, value);
#else
    value = MHal_EMAC_ReadReg32_EMAC1(0x100);
    value &= 0xfff7ffff;
    value |= xval << 19;

    MHal_EMAC_WritReg32_EMAC1(0x100, value);
#endif
}

u32 MHal_EMAC_Get_Miu_Priority(void)
{
#ifndef EMAC_SUPPORT_XIU32
    return (MHal_EMAC_ReadReg32(0x100) & 0x00080000) >> 19;
#else
    return (MHal_EMAC_ReadReg32_EMAC1(0x100) & 0x00080000) >> 19;
#endif
}

void MHal_EMAC_Set_Tx_Hang_Fix_ECO(u32 xval)
{
    u32 value;
#ifndef EMAC_SUPPORT_XIU32
    value = MHal_EMAC_ReadReg32(0x134);
    value &= 0xfffbffff;
    value |= xval << 18;

    MHal_EMAC_WritReg32(0x134, value);
#else
    value = MHal_EMAC_ReadReg32_EMAC1(0x134);
    value &= 0xfffbffff;
    value |= xval << 18;

    MHal_EMAC_WritReg32_EMAC1(0x134, value);
#endif
}

void MHal_EMAC_Set_MIU_Out_Of_Range_Fix(u32 xval)
{
    u32 value;
#ifndef EMAC_SUPPORT_XIU32
    value = MHal_EMAC_ReadReg32(0x134);
    value &= 0xefffffff;
    value |= xval << 28;

    MHal_EMAC_WritReg32(0x134, value);
#else
    value = MHal_EMAC_ReadReg32_EMAC1(0x134);
    value &= 0xefffffff;
    value |= xval << 28;

    MHal_EMAC_WritReg32_EMAC1(0x134, value);
#endif
}

void MHal_EMAC_Set_Rx_Tx_Burst16_Mode(u32 xval)
{
    u32 value;
#ifndef EMAC_SUPPORT_XIU32
    value = MHal_EMAC_ReadReg32(0x134);
    value &= 0xdfffffff;
    value |= xval << 29;

    MHal_EMAC_WritReg32(0x134, value);
#else
    value = MHal_EMAC_ReadReg32_EMAC1(0x134);
    value &= 0xdfffffff;
    value |= xval << 29;

    MHal_EMAC_WritReg32_EMAC1(0x134, value);
#endif
}

void MHal_EMAC_Set_Tx_Rx_Req_Priority_Switch(u32 xval)
{
    u32 value;
#ifndef EMAC_SUPPORT_XIU32
    value = MHal_EMAC_ReadReg32(0x134);
    value &= 0xfff7ffff;
    value |= xval << 19;

    MHal_EMAC_WritReg32(0x134, value);
#else
    value = MHal_EMAC_ReadReg32_EMAC1(0x134);
    value &= 0xfff7ffff;
    value |= xval << 19;

    MHal_EMAC_WritReg32_EMAC1(0x134, value);
#endif
}

void MHal_EMAC_Set_Rx_Byte_Align_Offset(u32 xval)
{
    u32 value;
#ifndef EMAC_SUPPORT_XIU32
    value = MHal_EMAC_ReadReg32(0x134);
    value &= 0xf3ffffff;
    value |= xval << 26;

    MHal_EMAC_WritReg32(0x134, value);
#else
    value = MHal_EMAC_ReadReg32_EMAC1(0x134);
    value &= 0xf3ffffff;
    value |= xval << 26;

    MHal_EMAC_WritReg32_EMAC1(0x134, value);
#endif
}

void MHal_EMAC_Write_Protect(u32 start_addr, u32 length)
{
    u32 value;

#ifndef EMAC_SUPPORT_XIU32
    value = MHal_EMAC_ReadReg32(0x11c);
    value &= 0x0000ffff;
    value |= ((start_addr+length) >> 3) << 16;
    MHal_EMAC_WritReg32(0x11c, value);

    value = MHal_EMAC_ReadReg32(0x120);
    value &= 0x00000000;
    value |= ((start_addr+length) >> 3) >> 16;
    value |= (start_addr >> 3) << 16;
    MHal_EMAC_WritReg32(0x120, value);

    value = MHal_EMAC_ReadReg32(0x124);
    value &= 0xffff0000;
    value |= (start_addr >> 3) >> 16;
    MHal_EMAC_WritReg32(0x124, value);
#else
    value = MHal_EMAC_ReadReg32_EMAC1(0x11c);
    value &= 0x0000ffff;
    value |= ((start_addr+length) >> 3) << 16;
    MHal_EMAC_WritReg32_EMAC1(0x11c, value);

    value = MHal_EMAC_ReadReg32_EMAC1(0x120);
    value &= 0x00000000;
    value |= ((start_addr+length) >> 3) >> 16;
    value |= (start_addr >> 3) << 16;
    MHal_EMAC_WritReg32_EMAC1(0x120, value);

    value = MHal_EMAC_ReadReg32_EMAC1(0x124);
    value &= 0xffff0000;
    value |= (start_addr >> 3) >> 16;
    MHal_EMAC_WritReg32_EMAC1(0x124, value);
#endif
}

void MHal_EMAC_Set_Miu_Highway(u32 xval)
{
    u32 value;
#ifndef EMAC_SUPPORT_XIU32
    value = MHal_EMAC_ReadReg32(0x134);
    value &= 0xbfffffff;
    value |= xval << 30;

    MHal_EMAC_WritReg32(0x134, value);
#else
    value = MHal_EMAC_ReadReg32_EMAC1(0x134);
    value &= 0xbfffffff;
    value |= xval << 30;

    MHal_EMAC_WritReg32_EMAC1(0x134, value);
#endif
}

void MHal_EMAC_HW_init(void)
{
    MHal_EMAC_Set_Miu_Priority(1);
    MHal_EMAC_Set_Tx_JULIAN_T(4);
    MHal_EMAC_Set_Rx_Tx_Burst16_Mode(1);
    MHal_EMAC_Set_Rx_FIFO_Enlarge(2);
    MHal_EMAC_Set_Tx_Hang_Fix_ECO(1);
    MHal_EMAC_Set_MIU_Out_Of_Range_Fix(1);
    #ifdef RX_BYTE_ALIGN_OFFSET
    MHal_EMAC_Set_Rx_Byte_Align_Offset(2);
    #endif
#ifndef EMAC_SUPPORT_XIU32
    MHal_EMAC_WritReg32(REG_EMAC_JULIAN_0138, MHal_EMAC_ReadReg32(REG_EMAC_JULIAN_0138) | 0x00000001);
#else
    MHal_EMAC_WritReg32_EMAC1(REG_EMAC_JULIAN_0138, MHal_EMAC_ReadReg32(REG_EMAC_JULIAN_0138) | 0x00000001);
#endif
//    MHal_EMAC_Set_Miu_Highway(1);
}

//-------------------------------------------------------------------------------------------------
//  PHY INTERFACE
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Enable the MDIO bit in MAC control register
// When not called from an interrupt-handler, access to the PHY must be
// protected by a spinlock.
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_enable_mdi( void )
{
    u32 xval;
    xval = MHal_EMAC_Read_CTL();
    xval |= EMAC_MPE;
    MHal_EMAC_Write_CTL( xval );
}

//-------------------------------------------------------------------------------------------------
//  Disable the MDIO bit in the MAC control register
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_disable_mdi( void )
{
    u32 xval;
    xval = MHal_EMAC_Read_CTL();
    xval &= ~EMAC_MPE;
    MHal_EMAC_Write_CTL( xval );
}

//-------------------------------------------------------------------------------------------------
// Write value to the a PHY register
// Note: MDI interface is assumed to already have been enabled.
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_write_phy( unsigned char phy_addr, unsigned char address, u32 value )
{
#if CONFIG_ETHERNET_ALBANY
    u32 uRegBase = INTERNEL_PHY_REG_BASE;

    phy_addr =0;

    *(volatile unsigned int *)(uRegBase + address*4) = value;
    udelay( 1 );
#else
    u32 uRegVal = 0, uCTL = 0;
    uRegVal =  ( EMAC_HIGH | EMAC_CODE_802_3 | EMAC_RW_W) | (( phy_addr & 0x1F ) << PHY_ADDR_OFFSET )
                | ( address << PHY_REGADDR_OFFSET ) | (value & 0xFFFF);

    uCTL = MHal_EMAC_Read_CTL();
    MHal_EMAC_enable_mdi();

    MHal_EMAC_Write_MAN( uRegVal );
    // Wait until IDLE bit in Network Status register is cleared //
    uRegVal = MHal_EMAC_ReadReg32( REG_ETH_SR );  //Must read Low 16 bit.
    while ( !( uRegVal & EMAC_IDLE ) )
    {
        uRegVal = MHal_EMAC_ReadReg32( REG_ETH_SR );
        barrier();
    }
    MHal_EMAC_Write_CTL(uCTL);
#endif
}
//-------------------------------------------------------------------------------------------------
// Read value stored in a PHY register.
// Note: MDI interface is assumed to already have been enabled.
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_read_phy( unsigned char phy_addr, unsigned char address, u32* value )
{
#if CONFIG_ETHERNET_ALBANY
    u32 uRegBase  = INTERNEL_PHY_REG_BASE;
    u32 tempvalue ;

    phy_addr =0;

    tempvalue = *(volatile unsigned int *)(INTERNEL_PHY_REG_BASE + 0x04);
    tempvalue |= 0x0004;
    *(volatile unsigned int *)(INTERNEL_PHY_REG_BASE + 0x04) = tempvalue;
    udelay( 1 );
    *value = *(volatile unsigned int *)(uRegBase + address*4);
#else
    u32 uRegVal = 0, uCTL = 0;

    uRegVal = (EMAC_HIGH | EMAC_CODE_802_3 | EMAC_RW_R)
            | ((phy_addr & 0x1f) << PHY_ADDR_OFFSET) | (address << PHY_REGADDR_OFFSET) | (0) ;

    uCTL = MHal_EMAC_Read_CTL();
    MHal_EMAC_enable_mdi();
    MHal_EMAC_Write_MAN(uRegVal);

    //Wait until IDLE bit in Network Status register is cleared //
    uRegVal = MHal_EMAC_ReadReg32( REG_ETH_SR );  //Must read Low 16 bit.
    while ( !( uRegVal & EMAC_IDLE ) )
    {
        uRegVal = MHal_EMAC_ReadReg32( REG_ETH_SR );
        barrier();
    }
    *value = ( MHal_EMAC_Read_MAN() & 0x0000ffff );
    MHal_EMAC_Write_CTL(uCTL);
#endif
}

#ifdef CONFIG_ETHERNET_ALBANY
void MHal_EMAC_TX_10MB_lookUp_table( void )
{
    // tx 10T link test pulse
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x0f*4) ) = 0x9800;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x10*4) ) = 0x8484;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x11*4) ) = 0x8888;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x12*4) ) = 0x8c8c;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x13*4) ) = 0xC898;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x14*4) ) = 0x0000;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x15*4) ) = 0x1000;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x16*4) ) = ( (*( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x16*4) ) & 0xFF00) | 0x0000 );

    // tx 10T look up table.
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x44*4) ) = 0x3C3C;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x45*4) ) = 0x3C3C;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x46*4) ) = 0x3C30;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x47*4) ) = 0x687C;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x48*4) ) = 0x7834;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x49*4) ) = 0xD494;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x4A*4) ) = 0x84A0;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x4B*4) ) = 0xE4C8;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x4C*4) ) = 0xC8C8;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x4D*4) ) = 0xC8E8;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x4E*4) ) = 0x3C3C;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x4F*4) ) = 0x3C3C;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x50*4) ) = 0x2430;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x51*4) ) = 0x707C;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x52*4) ) = 0x6420;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x53*4) ) = 0xD4A0;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x54*4) ) = 0x8498;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x55*4) ) = 0xD0C8;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x56*4) ) = 0xC8C8;
    *( ( u32 * ) ( ( char * ) INTERNEL_PHY_REG_BASE +  0x57*4) ) = 0xC8C8;
}
#endif

//-------------------------------------------------------------------------------------------------
// Update MAC speed and H/F duplex
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_update_speed_duplex( u32 uspeed, u32 uduplex )
{
    u32 xval;

    xval = MHal_EMAC_ReadReg32( REG_ETH_CFG ) & ~( EMAC_SPD | EMAC_FD );

    if ( uspeed == SPEED_100 )
    {
        if ( uduplex == DUPLEX_FULL )    // 100 Full Duplex //
        {
            xval = xval | EMAC_SPD | EMAC_FD;
        }
        else                           // 100 Half Duplex ///
        {
            xval = xval | EMAC_SPD;
        }
    }
    else
    {
        if ( uduplex == DUPLEX_FULL )    //10 Full Duplex //
        {
            xval = xval | EMAC_FD;
        }
        else                           // 10 Half Duplex //
        {
        }
    }
    MHal_EMAC_WritReg32( REG_ETH_CFG, xval );
}

u8 MHal_EMAC_CalcMACHash( u8 m0, u8 m1, u8 m2, u8 m3, u8 m4, u8 m5 )
{
    u8 hashIdx0 = ( m0&0x01 ) ^ ( ( m0&0x40 ) >> 6 ) ^ ( ( m1&0x10 ) >> 4 ) ^ ( ( m2&0x04 ) >> 2 )
                 ^ ( m3&0x01 ) ^ ( ( m3&0x40 ) >> 6 ) ^ ( ( m4&0x10 ) >> 4 ) ^ ( ( m5&0x04 ) >> 2 );

     u8 hashIdx1 = ( m0&0x02 ) ^ ( ( m0&0x80 ) >> 6 ) ^ ( ( m1&0x20 ) >> 4 ) ^ ( ( m2&0x08 ) >> 2 )
                ^ ( m3&0x02 ) ^ ( ( m3&0x80 ) >> 6 ) ^ ( ( m4&0x20 ) >> 4 ) ^ ( ( m5&0x08 ) >> 2 );

    u8 hashIdx2 = ( m0&0x04 ) ^ ( ( m1&0x01 ) << 2 ) ^ ( ( m1&0x40 ) >> 4 ) ^ ( ( m2&0x10 ) >> 2 )
                ^ ( m3&0x04 ) ^ ( ( m4&0x01 ) << 2 ) ^ ( ( m4&0x40 ) >> 4 ) ^ ( ( m5&0x10 ) >> 2 );

    u8 hashIdx3 = ( m0&0x08 ) ^ ( ( m1&0x02 ) << 2 ) ^ ( ( m1&0x80 ) >> 4 ) ^ ( ( m2&0x20 ) >> 2 )
                ^ ( m3&0x08 ) ^ ( ( m4&0x02 ) << 2 ) ^ ( ( m4&0x80 ) >> 4 ) ^ ( ( m5&0x20 ) >> 2 );

    u8 hashIdx4 = ( m0&0x10 ) ^ ( ( m1&0x04 ) << 2 ) ^ ( ( m2&0x01 ) << 4 ) ^ ( ( m2&0x40 ) >> 2 )
                ^ ( m3&0x10 ) ^ ( ( m4&0x04 ) << 2 ) ^ ( ( m5&0x01 ) << 4 ) ^ ( ( m5&0x40 ) >> 2 );

    u8 hashIdx5 = ( m0&0x20 ) ^ ( ( m1&0x08 ) << 2 ) ^ ( ( m2&0x02 ) << 4 ) ^ ( ( m2&0x80 ) >> 2 )
                ^ ( m3&0x20 ) ^ ( ( m4&0x08 ) << 2 ) ^ ( ( m5&0x02 ) << 4 ) ^ ( ( m5&0x80 ) >> 2 );

    return( hashIdx0 | hashIdx1 | hashIdx2 | hashIdx3 | hashIdx4 | hashIdx5 );
}

//-------------------------------------------------------------------------------------------------
//Initialize and enable the PHY interrupt when link-state changes
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_enable_phyirq( void )
{
#if 0

#endif
}

//-------------------------------------------------------------------------------------------------
// Disable the PHY interrupt
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_disable_phyirq( void )
{
#if 0

#endif
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------

u32 MHal_EMAC_get_SA1H_addr( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_SA1H );
}

u32 MHal_EMAC_get_SA1L_addr( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_SA1L );
}

u32 MHal_EMAC_get_SA2H_addr( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_SA2H );
}

u32 MHal_EMAC_get_SA2L_addr( void )
{
    return MHal_EMAC_ReadReg32( REG_ETH_SA2L );
}

void MHal_EMAC_Write_SA1H( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_SA1H, xval );
}

void MHal_EMAC_Write_SA1L( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_SA1L, xval );
}

void MHal_EMAC_Write_SA2H( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_SA2H, xval );
}

void MHal_EMAC_Write_SA2L( u32 xval )
{
    MHal_EMAC_WritReg32( REG_ETH_SA2L, xval );
}

void MHal_EMAC_set_wol_mac_addr(u8 m0, u8 m1, u8 m2, u8 m3, u8 m4, u8 m5)
{
    /* m5:m4:m3:m2:m1:m0 */
    MHal_EMAC_WritReg8(REG_ALBANY2_BANK, REG_WOL_ETH_MAC_ADDR_LO_OFFSET, m0);
    MHal_EMAC_WritReg8(REG_ALBANY2_BANK, REG_WOL_ETH_MAC_ADDR_LO_OFFSET + 1, m1);
    MHal_EMAC_WritReg8(REG_ALBANY2_BANK, REG_WOL_ETH_MAC_ADDR_MID_OFFSET, m2);
    MHal_EMAC_WritReg8(REG_ALBANY2_BANK, REG_WOL_ETH_MAC_ADDR_MID_OFFSET + 1, m3);
    MHal_EMAC_WritReg8(REG_ALBANY2_BANK, REG_WOL_ETH_MAC_ADDR_HI_OFFSET, m4);
    MHal_EMAC_WritReg8(REG_ALBANY2_BANK, REG_WOL_ETH_MAC_ADDR_HI_OFFSET + 1, m5);
}

void* MDev_memset( void* s, u32 c, unsigned long count )
{
    char* xs = ( char* ) s;

    while ( count-- )
        *xs++ = c;

    return s;
}

//-------------------------------------------------------------------------------------------------
// Check INT Done
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_CheckINTDone( void )
{
    u32 retIntStatus;
    retIntStatus = MHal_EMAC_Read_ISR();
    MHalThisUVE.cntChkINTCounter = ( MHalThisUVE.cntChkINTCounter %
                                     MHal_MAX_INT_COUNTER );
    MHalThisUVE.cntChkINTCounter ++;
    if ( ( retIntStatus & EMAC_INT_DONE ) ||
         ( MHalThisUVE.cntChkINTCounter == ( MHal_MAX_INT_COUNTER - 1 ) ) )
    {
        MHalThisUVE.flagISR_INT_DONE = 0x01;
        return TRUE;
    }
    return FALSE;
}

extern unsigned char phyaddr;
//-------------------------------------------------------------------------------------------------
// MAC cable connection detection
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_CableConnection( void )
{
    u32 retValue = 0;
    u32 word_ETH_MAN = 0x00000000;
    u32 word_ETH_CTL = MHal_EMAC_Read_CTL();

    MHal_EMAC_Write_CTL( 0x00000010 | word_ETH_CTL );
    MHalThisUVE.flagISR_INT_DONE = 0x00;
    MHalThisUVE.cntChkINTCounter = 0;
    MHal_EMAC_read_phy(phyaddr, MII_BMSR, &word_ETH_MAN);

    if ( word_ETH_MAN & BMSR_LSTATUS )
    {
        retValue = 1;
    }
    else
    {
        retValue = 0;
    }
    MHal_EMAC_Write_CTL( word_ETH_CTL );
    return(retValue);
}

//-------------------------------------------------------------------------------------------------
// EMAC Negotiation PHY
//-------------------------------------------------------------------------------------------------
u32 MHal_EMAC_NegotiationPHY( void )
{
    // Set PHY --------------------------------------------------------------
    u32 retValue = 0;
    u32 bmsr;

    // IMPORTANT: Get real duplex by negotiation with peer.
    u32 word_ETH_CTL = MHal_EMAC_Read_CTL();
    MHal_EMAC_Write_CTL( 0x0000001C | word_ETH_CTL );

    MHalThisBCE.duplex = 1;   // Set default as Half-duplex if negotiation fails.
    retValue = 1;

    MHalThisUVE.flagISR_INT_DONE = 0x00;
    MHalThisUVE.cntChkINTCounter = 0;
    MHalThisUVE.cntChkCableConnect = 0;


    MHal_EMAC_read_phy(phyaddr, MII_BMSR, &bmsr);
    if ( (bmsr & BMSR_100FULL) || (bmsr & BMSR_10FULL) )
    {
       MHalThisBCE.duplex = 2;
       retValue = 2;
    }
    else
    {
        MHalThisBCE.duplex = 1;
        retValue = 1;
    }

    // NOTE: REG_ETH_CFG must be set according to new ThisBCE.duplex.

    MHal_EMAC_Write_CTL( word_ETH_CTL );
    // Set PHY --------------------------------------------------------------
    return(retValue);
}

//-------------------------------------------------------------------------------------------------
// EMAC Hardware register set
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// EMAC Timer set for Receive function
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_timer_callback( unsigned long value )
{
    u32 uRegVal;
    uRegVal = MHal_EMAC_Read_IER();
    uRegVal |= ( EMAC_INT_RCOM );
    MHal_EMAC_Write_IER( uRegVal );
}

void MHal_EMAC_trim_phy( void )
{
    u8 uRegVal;
    u8 uEfuVal0;
    u8 uEfuVal1;
    u8 uEfuVal2;

    uRegVal = MHal_EMAC_ReadReg8(0x0020, 0x4e);
    uRegVal = 0x25;
    MHal_EMAC_WritReg8(0x0020, 0x4e, uRegVal);

    uRegVal = MHal_EMAC_ReadReg8(0x0020, 0x4f);
    uRegVal = 0x00;
    MHal_EMAC_WritReg8(0x0020, 0x4f, uRegVal);

    //reg28[13] = 0x01
    uRegVal = MHal_EMAC_ReadReg8(0x0020, 0x4c);
    uRegVal |= 0x01;
    MHal_EMAC_WritReg8(0x0020, 0x4c, uRegVal);

    do
    {
        uRegVal = MHal_EMAC_ReadReg8(0x0020, 0x4c);
    }while((uRegVal & 0x01) != 0);

    uEfuVal0 = MHal_EMAC_ReadReg8(0x0020, 0x84);
    uEfuVal1 = MHal_EMAC_ReadReg8(0x0020, 0x85);
    uEfuVal2 = MHal_EMAC_ReadReg8(0x0020, 0x86);

    //write efuse into phy trim register
    uRegVal = MHal_EMAC_ReadReg8(0x121a, 0x60);
    uRegVal |= 0x04;
    MHal_EMAC_WritReg8(0x121a, 0x60, uRegVal);

    uRegVal = MHal_EMAC_ReadReg8(0x121a, 0x69);
    uRegVal |= 0x80;
    MHal_EMAC_WritReg8(0x121a, 0x69, uRegVal);

    MHal_EMAC_WritReg8(0x121a, 0x68, uEfuVal0);

    uRegVal = MHal_EMAC_ReadReg8(0x121a, 0x69);
    uRegVal &= 0xc0;
    uRegVal |= (uEfuVal1 & 0x3f);
    MHal_EMAC_WritReg8(0x121a, 0x69, uRegVal);

    uRegVal = MHal_EMAC_ReadReg8(0x121a, 0x61);
    uRegVal &= 0xf0;
    uRegVal |= (uEfuVal1 >> 6);
    uRegVal |= (uEfuVal2 & 0x03) << 2 ;
    MHal_EMAC_WritReg8(0x121a, 0x61, uRegVal);
}

//-------------------------------------------------------------------------------------------------
// EMAC clock on/off
//-------------------------------------------------------------------------------------------------
void MHal_EMAC_Power_On_Clk( void )
{
     u8 uRegVal;

    //swith RX discriptor format to mode 1
    MHal_EMAC_WritReg8(0x1021, 0x3a, 0x00);
    MHal_EMAC_WritReg8(0x1021, 0x3b, 0x01);

    //emac_clk gen
#ifdef CONFIG_ETHERNET_ALBANY
    //PD_ETH33
    uRegVal = MHal_EMAC_ReadReg8(0x000e, 0x60);
    uRegVal &= 0xfe;
    MHal_EMAC_WritReg8(0x000e, 0x60, uRegVal);

    //gain shift
    MHal_EMAC_WritReg8(0x0033, 0xb4, 0x02);

    //det max
    MHal_EMAC_WritReg8(0x0033, 0x4f, 0x02);

    //det min
    MHal_EMAC_WritReg8(0x0033, 0x51, 0x01);

    //snr len (emc noise)
    MHal_EMAC_WritReg8(0x0033, 0x77, 0x18);

    //lpbk_enable set to 0
    MHal_EMAC_WritReg8(0x0032, 0x72, 0xa0);

    MHal_EMAC_WritReg8(0x0033, 0xfc, 0x00);
    MHal_EMAC_WritReg8(0x0033, 0xfd, 0x00);
    MHal_EMAC_WritReg8(0x0033, 0xb7, 0x17);
    MHal_EMAC_WritReg8(0x0033, 0xcb, 0x11);
    MHal_EMAC_WritReg8(0x0033, 0xcc, 0x20);
    MHal_EMAC_WritReg8(0x0033, 0xcd, 0xd0);
    MHal_EMAC_WritReg8(0x0033, 0xd4, 0x00);
    MHal_EMAC_WritReg8(0x0033, 0xb9, 0x40);
    MHal_EMAC_WritReg8(0x0033, 0xbb, 0x05);
    MHal_EMAC_WritReg8(0x0034, 0x3a, 0x03);
    MHal_EMAC_WritReg8(0x0034, 0x3b, 0x00);
    MHal_EMAC_WritReg8(0x0033, 0x3b, 0x01);
    MHal_EMAC_WritReg8(0x0034, 0xa1, 0xc0);
    MHal_EMAC_WritReg8(0x0034, 0x8a, 0x01);
    MHal_EMAC_WritReg8(0x0033, 0xc4, 0x44);
    MHal_EMAC_WritReg8(0x0034, 0x80, 0x30);

    //100 gat
    MHal_EMAC_WritReg8(0x0034, 0xc5, 0x00);

    //200 gat
    MHal_EMAC_WritReg8(0x0034, 0x30, 0x43);

    //en_100t_phase
    MHal_EMAC_WritReg8(0x0034, 0x39, 0x41);

    //Prevent packet drop by inverted waveform
    MHal_EMAC_WritReg8(0x0032, 0x79, 0xd0);
    MHal_EMAC_WritReg8(0x0032, 0x77, 0x5a);

    //10T waveform
    MHal_EMAC_WritReg8(0x0034, 0xe8, 0x06);
    MHal_EMAC_WritReg8(0x0032, 0x2b, 0x00);
    MHal_EMAC_WritReg8(0x0034, 0xe8, 0x00);
    MHal_EMAC_WritReg8(0x0032, 0x2b, 0x00);
    MHal_EMAC_WritReg8(0x0034, 0xe8, 0x06);
    MHal_EMAC_WritReg8(0x0032, 0xaa, 0x1c);
    MHal_EMAC_WritReg8(0x0032, 0xac, 0x1c);
    MHal_EMAC_WritReg8(0x0032, 0xad, 0x1c);
    MHal_EMAC_WritReg8(0x0032, 0xae, 0x1c);
    MHal_EMAC_WritReg8(0x0032, 0xaf, 0x1c);
    MHal_EMAC_WritReg8(0x0034, 0xe8, 0x00);
    MHal_EMAC_WritReg8(0x0032, 0xab, 0x28);
    MHal_EMAC_WritReg8(0x0032, 0xaa, 0x1c);

    //Disable eee
    MHal_EMAC_WritReg8(0x0032, 0x2d, 0x7c);

    // Set MII Mode
    MHal_EMAC_WritReg8(0x100b, 0xc0, 0x00);
    MHal_EMAC_WritReg8(0x100b, 0xc1, 0x00);
    MHal_EMAC_WritReg8(0x100b, 0xc2, 0x00);
    MHal_EMAC_WritReg8(0x100b, 0xc3, 0x00);
    MHal_EMAC_WritReg8(0x100b, 0xc4, 0x01);
    MHal_EMAC_WritReg8(0x100b, 0xc5, 0x00);

    //Speed up timing recovery
    MHal_EMAC_WritReg8(0x0033, 0xf5, 0x02);

    //signal_det_k
    MHal_EMAC_WritReg8(0x0033, 0x0f, 0xc9);

    //snr_h
    MHal_EMAC_WritReg8(0x0033, 0x89, 0x50);
    MHal_EMAC_WritReg8(0x0033, 0x8b, 0x80);
    MHal_EMAC_WritReg8(0x0033, 0x8e, 0x0e);
    MHal_EMAC_WritReg8(0x0033, 0x90, 0x04);

	//10t_8bt
	MHal_EMAC_WritReg8(0x0032, 0xb4, 0x5a);
	MHal_EMAC_WritReg8(0x0032, 0xb6, 0x50);
	MHal_EMAC_WritReg8(0x0032, 0xff, 0x1a);
#else
    MHal_EMAC_WritReg8(0x100bUL, 0xc0UL, 0x00UL);
    MHal_EMAC_WritReg8(0x100bUL, 0xc1UL, 0x04UL);
    MHal_EMAC_WritReg8(0x100bUL, 0xc2UL, 0x04UL);
    MHal_EMAC_WritReg8(0x100bUL, 0xc3UL, 0x00UL);
    MHal_EMAC_WritReg8(0x100bUL, 0xc4UL, 0x00UL);
    MHal_EMAC_WritReg8(0x100bUL, 0xc5UL, 0x00UL);
#endif

    //chiptop [15] allpad_in
    uRegVal = MHal_EMAC_ReadReg8(0x101e, 0xa1);
    uRegVal &= 0x7f;
    MHal_EMAC_WritReg8(0x101e, 0xa1, uRegVal);

    //chiptop pad_top [9:8]
#ifdef CONFIG_ETHERNET_ALBANY
    uRegVal = MHal_EMAC_ReadReg8(0x101e, 0xdf);
    uRegVal &= 0xfe;
    MHal_EMAC_WritReg8(0x101e, 0xdf, uRegVal);
#else
    //et_mode = 1
    uRegVal = MHal_EMAC_ReadReg8(0x101eUL, 0xdfUL);
    uRegVal &= 0x01UL;
    MHal_EMAC_WritReg8(0x101eUL, 0xdfUL, uRegVal);
#endif
}

void MHal_EMAC_Power_Off_Clk( void )
{
}
