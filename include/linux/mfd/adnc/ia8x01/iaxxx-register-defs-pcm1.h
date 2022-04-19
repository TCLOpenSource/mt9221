/*
 * iaxxx-register-defs-pcm1.h
 *
 * Copyright (c) 2018 Knowles, inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**********************************************************
 * This file is generated by running a format script
 * on header files shared by Firmware.
 *
 * DO NOT EDIT.
 *
 *********************************************************/

#ifndef __IAXXX_REGISTER_DEFS_PCM1_H__
#define __IAXXX_REGISTER_DEFS_PCM1_H__

/*** The base address for this set of registers ***/
#define IAXXX_PCM1_REGS_ADDR (0x40043000)

/*** PCM1_SWLR (0x40043024) ***/
/*
 * Static Word Length Register
 */
#define IAXXX_PCM1_SWLR_ADDR (0x40043024)
#define IAXXX_PCM1_SWLR_MASK_VAL 0x0000001f
#define IAXXX_PCM1_SWLR_RMASK_VAL 0x0000001f
#define IAXXX_PCM1_SWLR_WMASK_VAL 0x0000001f
#define IAXXX_PCM1_SWLR_RESET_VAL 0x00000000

/*
 * Bits per word - 1
 */
#define IAXXX_PCM1_SWLR_WORD_LEN_MASK 0x0000001f
#define IAXXX_PCM1_SWLR_WORD_LEN_RESET_VAL 0x0
#define IAXXX_PCM1_SWLR_WORD_LEN_POS 0
#define IAXXX_PCM1_SWLR_WORD_LEN_SIZE 5
#define IAXXX_PCM1_SWLR_WORD_LEN_DECL (4:0)

/*** PCM1_SFLR (0x40043028) ***/
/*
 * Static Frame Length Register
 */
#define IAXXX_PCM1_SFLR_ADDR (0x40043028)
#define IAXXX_PCM1_SFLR_MASK_VAL 0x0000001f
#define IAXXX_PCM1_SFLR_RMASK_VAL 0x0000001f
#define IAXXX_PCM1_SFLR_WMASK_VAL 0x0000001f
#define IAXXX_PCM1_SFLR_RESET_VAL 0x00000000

/*
 * Words per frame - 1
 */
#define IAXXX_PCM1_SFLR_FRAME_LEN_MASK 0x0000001f
#define IAXXX_PCM1_SFLR_FRAME_LEN_RESET_VAL 0x0
#define IAXXX_PCM1_SFLR_FRAME_LEN_POS 0
#define IAXXX_PCM1_SFLR_FRAME_LEN_SIZE 5
#define IAXXX_PCM1_SFLR_FRAME_LEN_DECL (4:0)

/*** PCM1_STDD (0x4004302c) ***/
/*
 * Transmit data delay Register
 */
#define IAXXX_PCM1_STDD_ADDR (0x4004302c)
#define IAXXX_PCM1_STDD_MASK_VAL 0x0000001f
#define IAXXX_PCM1_STDD_RMASK_VAL 0x0000001f
#define IAXXX_PCM1_STDD_WMASK_VAL 0x0000001f
#define IAXXX_PCM1_STDD_RESET_VAL 0x00000000

/*
 * Transmit data delay (number of clocks offset from frame_sync).
 * If ADIS is 0 in Misc Control Register, this value HAS to be 0.
 * If ADIS is 1 in Misc Control Register, the value in this register has an
 * effective +1 added to it.
 */
#define IAXXX_PCM1_STDD_TX_DATA_DLY_MASK 0x0000001f
#define IAXXX_PCM1_STDD_TX_DATA_DLY_RESET_VAL 0x0
#define IAXXX_PCM1_STDD_TX_DATA_DLY_POS 0
#define IAXXX_PCM1_STDD_TX_DATA_DLY_SIZE 5
#define IAXXX_PCM1_STDD_TX_DATA_DLY_DECL (4:0)

/*** PCM1_SRDD (0x40043030) ***/
/*
 * Receive data delay Register
 */
#define IAXXX_PCM1_SRDD_ADDR (0x40043030)
#define IAXXX_PCM1_SRDD_MASK_VAL 0x0000001f
#define IAXXX_PCM1_SRDD_RMASK_VAL 0x0000001f
#define IAXXX_PCM1_SRDD_WMASK_VAL 0x0000001f
#define IAXXX_PCM1_SRDD_RESET_VAL 0x00000000

/*
 * Receive data delay (number of clocks offset from frame sync)
 */
#define IAXXX_PCM1_SRDD_RX_DATA_DLY_MASK 0x0000001f
#define IAXXX_PCM1_SRDD_RX_DATA_DLY_RESET_VAL 0x0
#define IAXXX_PCM1_SRDD_RX_DATA_DLY_POS 0
#define IAXXX_PCM1_SRDD_RX_DATA_DLY_SIZE 5
#define IAXXX_PCM1_SRDD_RX_DATA_DLY_DECL (4:0)

/*** PCM1_STSA (0x40043034) ***/
/*
 * Static Transmit Slots Active Register
 */
#define IAXXX_PCM1_STSA_ADDR (0x40043034)
#define IAXXX_PCM1_STSA_MASK_VAL 0xffffffff
#define IAXXX_PCM1_STSA_RMASK_VAL 0xffffffff
#define IAXXX_PCM1_STSA_WMASK_VAL 0xffffffff
#define IAXXX_PCM1_STSA_RESET_VAL 0x00000000

/*
 * Transmit slots active:bit vector with bit 0 -> slot 0. Any slots greater
 * than the number of words per frame MUST be set to 0. The sum of bits set
 * in this register must be <= 8. NOTE: words per frame includes both
 * inactive and active slots.
 */
#define IAXXX_PCM1_STSA_TX_SLOTS_ACTIVE_MASK 0xffffffff
#define IAXXX_PCM1_STSA_TX_SLOTS_ACTIVE_RESET_VAL 0x0
#define IAXXX_PCM1_STSA_TX_SLOTS_ACTIVE_POS 0
#define IAXXX_PCM1_STSA_TX_SLOTS_ACTIVE_SIZE 32
#define IAXXX_PCM1_STSA_TX_SLOTS_ACTIVE_DECL (31:0)

/*** PCM1_SRSA (0x40043038) ***/
/*
 * Static Receive Slots Active Register
 */
#define IAXXX_PCM1_SRSA_ADDR (0x40043038)
#define IAXXX_PCM1_SRSA_MASK_VAL 0xffffffff
#define IAXXX_PCM1_SRSA_RMASK_VAL 0xffffffff
#define IAXXX_PCM1_SRSA_WMASK_VAL 0xffffffff
#define IAXXX_PCM1_SRSA_RESET_VAL 0x00000000

/*
 * Receive slots active:bit vector with bit 0 -> slot 0. Any slots greater
 * than the number of words per frame MUST be set to 0. The sum of bits set
 * in this register must be <= 8. NOTE: words per frame includes both
 * inactive and active slots.
 */
#define IAXXX_PCM1_SRSA_RX_SLOTS_ACTIVE_MASK 0xffffffff
#define IAXXX_PCM1_SRSA_RX_SLOTS_ACTIVE_RESET_VAL 0x0
#define IAXXX_PCM1_SRSA_RX_SLOTS_ACTIVE_POS 0
#define IAXXX_PCM1_SRSA_RX_SLOTS_ACTIVE_SIZE 32
#define IAXXX_PCM1_SRSA_RX_SLOTS_ACTIVE_DECL (31:0)

/*** PCM1_MC (0x40043040) ***/
/*
 * Miscellaneous Control Register
 */
#define IAXXX_PCM1_MC_ADDR (0x40043040)
#define IAXXX_PCM1_MC_MASK_VAL 0x000071ff
#define IAXXX_PCM1_MC_RMASK_VAL 0x000071ff
#define IAXXX_PCM1_MC_WMASK_VAL 0x000071ff
#define IAXXX_PCM1_MC_RESET_VAL 0x00000000

/*
 * Frame Sync Polarity
 * 0:Active High
 * 1:Active Low
 */
#define IAXXX_PCM1_MC_FSP_MASK 0x00000001
#define IAXXX_PCM1_MC_FSP_RESET_VAL 0x0
#define IAXXX_PCM1_MC_FSP_POS 0
#define IAXXX_PCM1_MC_FSP_SIZE 1
#define IAXXX_PCM1_MC_FSP_DECL 0

/*
 * This is the Frame Sync Edge on which it is safe to sample FSYNC
 * 0:posedge pcm_clk
 * 1:negedge pcm_clk
 *  Note for FPGA: In FPGA, this register bit's function has been elimnated
 * and superceeded by the RCP bit.
 */
#define IAXXX_PCM1_MC_FSE_MASK 0x00000002
#define IAXXX_PCM1_MC_FSE_RESET_VAL 0x0
#define IAXXX_PCM1_MC_FSE_POS 1
#define IAXXX_PCM1_MC_FSE_SIZE 1
#define IAXXX_PCM1_MC_FSE_DECL 1

/*
 * Transmit Clock Polarity
 * 0:posedge
 * 1:negedge
 */
#define IAXXX_PCM1_MC_TCP_MASK 0x00000004
#define IAXXX_PCM1_MC_TCP_RESET_VAL 0x0
#define IAXXX_PCM1_MC_TCP_POS 2
#define IAXXX_PCM1_MC_TCP_SIZE 1
#define IAXXX_PCM1_MC_TCP_DECL 2

/*
 * Receive Clock Polarity
 * 0:posedge
 * 1:negedge
 */
#define IAXXX_PCM1_MC_RCP_MASK 0x00000008
#define IAXXX_PCM1_MC_RCP_RESET_VAL 0x0
#define IAXXX_PCM1_MC_RCP_POS 3
#define IAXXX_PCM1_MC_RCP_SIZE 1
#define IAXXX_PCM1_MC_RCP_DECL 3

/*
 * Endianness of data
 *
 1:MSB First
 *
 0:LSB First
 */
#define IAXXX_PCM1_MC_END_MASK 0x00000010
#define IAXXX_PCM1_MC_END_RESET_VAL 0x0
#define IAXXX_PCM1_MC_END_POS 4
#define IAXXX_PCM1_MC_END_SIZE 1
#define IAXXX_PCM1_MC_END_DECL 4

/*
 * Tri-state enable for the transmit portion of PCM
 */
#define IAXXX_PCM1_MC_TRI_MASK 0x00000020
#define IAXXX_PCM1_MC_TRI_RESET_VAL 0x0
#define IAXXX_PCM1_MC_TRI_POS 5
#define IAXXX_PCM1_MC_TRI_SIZE 1
#define IAXXX_PCM1_MC_TRI_DECL 5

/*
 * Disable last bit tri-state driver at inactive edge
 */
#define IAXXX_PCM1_MC_DL_MASK 0x00000040
#define IAXXX_PCM1_MC_DL_RESET_VAL 0x0
#define IAXXX_PCM1_MC_DL_POS 6
#define IAXXX_PCM1_MC_DL_SIZE 1
#define IAXXX_PCM1_MC_DL_DECL 6

/*
 * Dual Edge Mode enable. Use both sync edges for timing.  Deprecated since
 * eS305.
 */
#define IAXXX_PCM1_MC_I2S_MASK 0x00000080
#define IAXXX_PCM1_MC_I2S_RESET_VAL 0x0
#define IAXXX_PCM1_MC_I2S_POS 7
#define IAXXX_PCM1_MC_I2S_SIZE 1
#define IAXXX_PCM1_MC_I2S_DECL 7

/*
 * Asynchronous timing mode Disable, when set to '1'. Refer to Tx data delay
 * register definition for proper settings.
 */
#define IAXXX_PCM1_MC_ADIS_MASK 0x00000100
#define IAXXX_PCM1_MC_ADIS_RESET_VAL 0x0
#define IAXXX_PCM1_MC_ADIS_POS 8
#define IAXXX_PCM1_MC_ADIS_SIZE 1
#define IAXXX_PCM1_MC_ADIS_DECL 8

/*
 * Enable delay of pcm data by 1 rxclk relative to fsync
 */
#define IAXXX_PCM1_MC_DDLYEN_MASK 0x00001000
#define IAXXX_PCM1_MC_DDLYEN_RESET_VAL 0x0
#define IAXXX_PCM1_MC_DDLYEN_POS 12
#define IAXXX_PCM1_MC_DDLYEN_SIZE 1
#define IAXXX_PCM1_MC_DDLYEN_DECL 12

/*
 * Disable last bit tri-state driver at next active edge. Requires additional
 * clock edge at end of frame.
 */
#define IAXXX_PCM1_MC_DLNACT_MASK 0x00002000
#define IAXXX_PCM1_MC_DLNACT_RESET_VAL 0x0
#define IAXXX_PCM1_MC_DLNACT_POS 13
#define IAXXX_PCM1_MC_DLNACT_SIZE 1
#define IAXXX_PCM1_MC_DLNACT_DECL 13

/*
 * If this bit is set then PCM RX will left justify the received data to
 * Channel buffer and the transmit data should be left justified by SW for
 * PCM TX. For HCF mode, this field must be set to '1'.
 */
#define IAXXX_PCM1_MC_LEFTJUST_MASK 0x00004000
#define IAXXX_PCM1_MC_LEFTJUST_RESET_VAL 0x0
#define IAXXX_PCM1_MC_LEFTJUST_POS 14
#define IAXXX_PCM1_MC_LEFTJUST_SIZE 1
#define IAXXX_PCM1_MC_LEFTJUST_DECL 14

/*** PCM1_RIS (0x40043050) ***/
/*
 * Raw Interrupt Status
 */
#define IAXXX_PCM1_RIS_ADDR (0x40043050)
#define IAXXX_PCM1_RIS_MASK_VAL 0x01070007
#define IAXXX_PCM1_RIS_RMASK_VAL 0x01070007
#define IAXXX_PCM1_RIS_WMASK_VAL 0x01070007
#define IAXXX_PCM1_RIS_RESET_VAL 0x00000000

/*
 * Receive Frame sync too early error status enable.
 */
#define IAXXX_PCM1_RIS_RXFSYNCERR_MASK 0x00000001
#define IAXXX_PCM1_RIS_RXFSYNCERR_RESET_VAL 0x0
#define IAXXX_PCM1_RIS_RXFSYNCERR_POS 0
#define IAXXX_PCM1_RIS_RXFSYNCERR_SIZE 1
#define IAXXX_PCM1_RIS_RXFSYNCERR_DECL 0

/*
 * Extra clocks above what is needed error status. Write 1'b1 to clear.
 */
#define IAXXX_PCM1_RIS_RXFSCLOCKSERR_MASK 0x00000002
#define IAXXX_PCM1_RIS_RXFSCLOCKSERR_RESET_VAL 0x0
#define IAXXX_PCM1_RIS_RXFSCLOCKSERR_POS 1
#define IAXXX_PCM1_RIS_RXFSCLOCKSERR_SIZE 1
#define IAXXX_PCM1_RIS_RXFSCLOCKSERR_DECL 1

/*
 * Receive Deinterleave logic had unaligned data. Write 1'b1 to clear.
 */
#define IAXXX_PCM1_RIS_RXDEINTERR_MASK 0x00000004
#define IAXXX_PCM1_RIS_RXDEINTERR_RESET_VAL 0x0
#define IAXXX_PCM1_RIS_RXDEINTERR_POS 2
#define IAXXX_PCM1_RIS_RXDEINTERR_SIZE 1
#define IAXXX_PCM1_RIS_RXDEINTERR_DECL 2

/*
 * Transmit Frame sync too early error status. Write 1'b1 to clear.
 */
#define IAXXX_PCM1_RIS_TXFSYNCERR_MASK 0x00010000
#define IAXXX_PCM1_RIS_TXFSYNCERR_RESET_VAL 0x0
#define IAXXX_PCM1_RIS_TXFSYNCERR_POS 16
#define IAXXX_PCM1_RIS_TXFSYNCERR_SIZE 1
#define IAXXX_PCM1_RIS_TXFSYNCERR_DECL 16

/*
 * Transmit Hold register underflow occurred. Write 1'b1 to clean
 */
#define IAXXX_PCM1_RIS_TXHREGUFLERR_MASK 0x00020000
#define IAXXX_PCM1_RIS_TXHREGUFLERR_RESET_VAL 0x0
#define IAXXX_PCM1_RIS_TXHREGUFLERR_POS 17
#define IAXXX_PCM1_RIS_TXHREGUFLERR_SIZE 1
#define IAXXX_PCM1_RIS_TXHREGUFLERR_DECL 17

/*
 * Transmit interleave logic had unaligned data. Write 1'b1 to clear.
 */
#define IAXXX_PCM1_RIS_TXINTERR_MASK 0x00040000
#define IAXXX_PCM1_RIS_TXINTERR_RESET_VAL 0x0
#define IAXXX_PCM1_RIS_TXINTERR_POS 18
#define IAXXX_PCM1_RIS_TXINTERR_SIZE 1
#define IAXXX_PCM1_RIS_TXINTERR_DECL 18

/*
 * PCM was enabled by first valid frame sync. Write 1'b1 to clear
 */
#define IAXXX_PCM1_RIS_ON_ENABLE_MASK 0x01000000
#define IAXXX_PCM1_RIS_ON_ENABLE_RESET_VAL 0x0
#define IAXXX_PCM1_RIS_ON_ENABLE_POS 24
#define IAXXX_PCM1_RIS_ON_ENABLE_SIZE 1
#define IAXXX_PCM1_RIS_ON_ENABLE_DECL 24

/*** PCM1_ISEN (0x40043054) ***/
/*
 * Interrupt Status Enable
 */
#define IAXXX_PCM1_ISEN_ADDR (0x40043054)
#define IAXXX_PCM1_ISEN_MASK_VAL 0x01070007
#define IAXXX_PCM1_ISEN_RMASK_VAL 0x01070007
#define IAXXX_PCM1_ISEN_WMASK_VAL 0x01070007
#define IAXXX_PCM1_ISEN_RESET_VAL 0x01070007

/*
 * Enable RXFSYNCERR. Raw RXFSYNCERR unaffected.
 */
#define IAXXX_PCM1_ISEN_RXFSYNCERR_EN_MASK 0x00000001
#define IAXXX_PCM1_ISEN_RXFSYNCERR_EN_RESET_VAL 0x1
#define IAXXX_PCM1_ISEN_RXFSYNCERR_EN_POS 0
#define IAXXX_PCM1_ISEN_RXFSYNCERR_EN_SIZE 1
#define IAXXX_PCM1_ISEN_RXFSYNCERR_EN_DECL 0

/*
 * Enable RXFSCLOCKSERR. Raw RXFSYNCERR unaffected.
 */
#define IAXXX_PCM1_ISEN_RXFSCLOCKSERR_EN_MASK 0x00000002
#define IAXXX_PCM1_ISEN_RXFSCLOCKSERR_EN_RESET_VAL 0x1
#define IAXXX_PCM1_ISEN_RXFSCLOCKSERR_EN_POS 1
#define IAXXX_PCM1_ISEN_RXFSCLOCKSERR_EN_SIZE 1
#define IAXXX_PCM1_ISEN_RXFSCLOCKSERR_EN_DECL 1

/*
 * Enable RXDEINTERR. Raw RXFSYNCERR unaffected.
 */
#define IAXXX_PCM1_ISEN_RXDEINTERR_EN_MASK 0x00000004
#define IAXXX_PCM1_ISEN_RXDEINTERR_EN_RESET_VAL 0x1
#define IAXXX_PCM1_ISEN_RXDEINTERR_EN_POS 2
#define IAXXX_PCM1_ISEN_RXDEINTERR_EN_SIZE 1
#define IAXXX_PCM1_ISEN_RXDEINTERR_EN_DECL 2

/*
 * Enable TXFSYNCERR. Raw RXFSYNCERR unaffected.
 */
#define IAXXX_PCM1_ISEN_TXFSYNCERR_EN_MASK 0x00010000
#define IAXXX_PCM1_ISEN_TXFSYNCERR_EN_RESET_VAL 0x1
#define IAXXX_PCM1_ISEN_TXFSYNCERR_EN_POS 16
#define IAXXX_PCM1_ISEN_TXFSYNCERR_EN_SIZE 1
#define IAXXX_PCM1_ISEN_TXFSYNCERR_EN_DECL 16

/*
 * Enable TXHREGUFLERR. Raw TXHREGUFLERR unaffected.
 */
#define IAXXX_PCM1_ISEN_TXHREGUFLERR_EN_MASK 0x00020000
#define IAXXX_PCM1_ISEN_TXHREGUFLERR_EN_RESET_VAL 0x1
#define IAXXX_PCM1_ISEN_TXHREGUFLERR_EN_POS 17
#define IAXXX_PCM1_ISEN_TXHREGUFLERR_EN_SIZE 1
#define IAXXX_PCM1_ISEN_TXHREGUFLERR_EN_DECL 17

/*
 * Enable TXNTERR. Raw RXFSYNCERR unaffected.
 */
#define IAXXX_PCM1_ISEN_TXINTERR_EN_MASK 0x00040000
#define IAXXX_PCM1_ISEN_TXINTERR_EN_RESET_VAL 0x1
#define IAXXX_PCM1_ISEN_TXINTERR_EN_POS 18
#define IAXXX_PCM1_ISEN_TXINTERR_EN_SIZE 1
#define IAXXX_PCM1_ISEN_TXINTERR_EN_DECL 18

/*
 * Enable ON_ENABLE interrupt. Raw ON_ENABLE unaffected.
 */
#define IAXXX_PCM1_ISEN_ON_ENABLE_EN_MASK 0x01000000
#define IAXXX_PCM1_ISEN_ON_ENABLE_EN_RESET_VAL 0x1
#define IAXXX_PCM1_ISEN_ON_ENABLE_EN_POS 24
#define IAXXX_PCM1_ISEN_ON_ENABLE_EN_SIZE 1
#define IAXXX_PCM1_ISEN_ON_ENABLE_EN_DECL 24

/* Number of registers in the module */
#define IAXXX_PCM1_REG_NUM 9

#endif /* __IAXXX_REGISTER_DEFS_PCM1_H__*/
