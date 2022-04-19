/*
 * iaxxx-register-defs-h2p3.h
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

#ifndef __IAXXX_REGISTER_DEFS_H2P3_H__
#define __IAXXX_REGISTER_DEFS_H2P3_H__

/*** The base address for this set of registers ***/
#define IAXXX_H2P3_REGS_ADDR (0x40051000)

/*** H2P3_H2P3_MSTR_ACC (0x40051000) ***/
/*
 * This register has the access protection controls.
 *
 When access permission is set to "0", write towards any of the H2P3
 * registers generates an error response. Read operation always returns the
 * register content.
 */
#define IAXXX_H2P3_H2P3_MSTR_ACC_ADDR (0x40051000)
#define IAXXX_H2P3_H2P3_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P3_H2P3_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P3_H2P3_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P3_H2P3_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to H2P3 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_H2P3_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P3_H2P3_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P3_H2P3_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P3_H2P3_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P3_H2P3_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access H2P3
 */
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to H2P3 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to H2P3 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P3_H2P3_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access H2P3
 */
#define IAXXX_H2P3_H2P3_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P3_H2P3_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P3_H2P3_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P3_H2P3_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P3_H2P3_MSTR_ACC_DAP_DECL 8

/*** H2P3_CNR7_MSTR_ACC (0x40051004) ***/
/*
 * This register has the access protection controls.
 */
#define IAXXX_H2P3_CNR7_MSTR_ACC_ADDR (0x40051004)
#define IAXXX_H2P3_CNR7_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P3_CNR7_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P3_CNR7_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P3_CNR7_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to CNR7 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_CNR7_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P3_CNR7_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P3_CNR7_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P3_CNR7_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P3_CNR7_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access CNR7
 */
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to CNR7 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to CNR7 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P3_CNR7_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access CNR7
 */
#define IAXXX_H2P3_CNR7_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P3_CNR7_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P3_CNR7_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P3_CNR7_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P3_CNR7_MSTR_ACC_DAP_DECL 8

/*** H2P3_I2C0_MSTR_ACC (0x40051008) ***/
/*
 * This register has the access protection controls.
 */
#define IAXXX_H2P3_I2C0_MSTR_ACC_ADDR (0x40051008)
#define IAXXX_H2P3_I2C0_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P3_I2C0_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P3_I2C0_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P3_I2C0_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to I2C0 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_I2C0_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P3_I2C0_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P3_I2C0_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P3_I2C0_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P3_I2C0_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access I2C0
 */
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to I2C0 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to I2C0 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P3_I2C0_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access I2C0
 */
#define IAXXX_H2P3_I2C0_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P3_I2C0_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P3_I2C0_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P3_I2C0_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P3_I2C0_MSTR_ACC_DAP_DECL 8

/*** H2P3_I2C1_MSTR_ACC (0x4005100c) ***/
/*
 * This register has the access protection controls.
 */
#define IAXXX_H2P3_I2C1_MSTR_ACC_ADDR (0x4005100c)
#define IAXXX_H2P3_I2C1_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P3_I2C1_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P3_I2C1_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P3_I2C1_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to I2C1 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_I2C1_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P3_I2C1_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P3_I2C1_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P3_I2C1_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P3_I2C1_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access I2C1
 */
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to I2C1 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to I2C1 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P3_I2C1_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access I2C1
 */
#define IAXXX_H2P3_I2C1_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P3_I2C1_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P3_I2C1_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P3_I2C1_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P3_I2C1_MSTR_ACC_DAP_DECL 8

/*** H2P3_SPI0_MSTR_ACC (0x40051010) ***/
/*
 * This register has the access protection controls.
 */
#define IAXXX_H2P3_SPI0_MSTR_ACC_ADDR (0x40051010)
#define IAXXX_H2P3_SPI0_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P3_SPI0_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P3_SPI0_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P3_SPI0_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to SPI0 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_SPI0_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P3_SPI0_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P3_SPI0_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P3_SPI0_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P3_SPI0_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access SPI0
 */
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to SPI0 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to SPI0 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P3_SPI0_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access SPI0
 */
#define IAXXX_H2P3_SPI0_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P3_SPI0_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P3_SPI0_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P3_SPI0_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P3_SPI0_MSTR_ACC_DAP_DECL 8

/*** H2P3_SPI1_MSTR_ACC (0x40051014) ***/
/*
 * This register has the access protection controls.
 */
#define IAXXX_H2P3_SPI1_MSTR_ACC_ADDR (0x40051014)
#define IAXXX_H2P3_SPI1_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P3_SPI1_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P3_SPI1_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P3_SPI1_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to SPI1 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_SPI1_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P3_SPI1_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P3_SPI1_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P3_SPI1_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P3_SPI1_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access SPI1
 */
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to SPI1 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to SPI1 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P3_SPI1_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access SPI1
 */
#define IAXXX_H2P3_SPI1_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P3_SPI1_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P3_SPI1_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P3_SPI1_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P3_SPI1_MSTR_ACC_DAP_DECL 8

/*** H2P3_UART0_MSTR_ACC (0x40051018) ***/
/*
 * This register has the access protection controls.
 */
#define IAXXX_H2P3_UART0_MSTR_ACC_ADDR (0x40051018)
#define IAXXX_H2P3_UART0_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P3_UART0_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P3_UART0_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P3_UART0_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to UART0 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_UART0_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P3_UART0_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P3_UART0_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P3_UART0_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P3_UART0_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access UART0
 */
#define IAXXX_H2P3_UART0_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P3_UART0_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P3_UART0_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P3_UART0_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P3_UART0_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to UART0 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_UART0_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P3_UART0_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P3_UART0_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P3_UART0_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P3_UART0_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to UART0 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_UART0_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P3_UART0_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P3_UART0_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P3_UART0_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P3_UART0_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access UART0
 */
#define IAXXX_H2P3_UART0_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P3_UART0_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P3_UART0_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P3_UART0_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P3_UART0_MSTR_ACC_DAP_DECL 8

/*** H2P3_UART1_MSTR_ACC (0x4005101c) ***/
/*
 * This register has the access protection controls.
 */
#define IAXXX_H2P3_UART1_MSTR_ACC_ADDR (0x4005101c)
#define IAXXX_H2P3_UART1_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P3_UART1_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P3_UART1_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P3_UART1_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to UART1 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_UART1_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P3_UART1_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P3_UART1_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P3_UART1_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P3_UART1_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access UART1
 */
#define IAXXX_H2P3_UART1_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P3_UART1_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P3_UART1_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P3_UART1_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P3_UART1_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to UART1 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_UART1_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P3_UART1_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P3_UART1_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P3_UART1_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P3_UART1_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to UART1 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_UART1_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P3_UART1_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P3_UART1_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P3_UART1_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P3_UART1_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access UART1
 */
#define IAXXX_H2P3_UART1_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P3_UART1_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P3_UART1_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P3_UART1_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P3_UART1_MSTR_ACC_DAP_DECL 8

/*** H2P3_AFG_MSTR_ACC (0x40051020) ***/
/*
 * This register has the access protection controls.
 */
#define IAXXX_H2P3_AFG_MSTR_ACC_ADDR (0x40051020)
#define IAXXX_H2P3_AFG_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P3_AFG_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P3_AFG_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P3_AFG_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to AFG registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_AFG_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P3_AFG_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P3_AFG_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P3_AFG_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P3_AFG_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access AFG
 */
#define IAXXX_H2P3_AFG_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P3_AFG_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P3_AFG_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P3_AFG_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P3_AFG_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to AFG registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_AFG_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P3_AFG_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P3_AFG_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P3_AFG_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P3_AFG_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to AFG registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P3_AFG_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P3_AFG_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P3_AFG_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P3_AFG_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P3_AFG_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access AFG
 */
#define IAXXX_H2P3_AFG_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P3_AFG_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P3_AFG_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P3_AFG_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P3_AFG_MSTR_ACC_DAP_DECL 8

/*** H2P3_ACC_ERR (0x40051024) ***/
/*
 * This register has access information that is denied. Only the first error
 * is recorded until the error status is cleared.
 */
#define IAXXX_H2P3_ACC_ERR_ADDR (0x40051024)
#define IAXXX_H2P3_ACC_ERR_MASK_VAL 0x000001f0
#define IAXXX_H2P3_ACC_ERR_RMASK_VAL 0x000001f0
#define IAXXX_H2P3_ACC_ERR_WMASK_VAL 0x000000d0
#define IAXXX_H2P3_ACC_ERR_RESET_VAL 0x00000000

/*
 * Access is denied to HMD. Only the first error among (HMD, DMA_M0, DMA_M1)
 * is recorded until the error status is cleared
 */
#define IAXXX_H2P3_ACC_ERR_HMD_MASK 0x00000010
#define IAXXX_H2P3_ACC_ERR_HMD_RESET_VAL 0x0
#define IAXXX_H2P3_ACC_ERR_HMD_POS 4
#define IAXXX_H2P3_ACC_ERR_HMD_SIZE 1
#define IAXXX_H2P3_ACC_ERR_HMD_DECL 4

/*
 * Reserved (DMX can never be denied access)
 */
#define IAXXX_H2P3_ACC_ERR_DMX_MASK 0x00000020
#define IAXXX_H2P3_ACC_ERR_DMX_RESET_VAL 0x0
#define IAXXX_H2P3_ACC_ERR_DMX_POS 5
#define IAXXX_H2P3_ACC_ERR_DMX_SIZE 1
#define IAXXX_H2P3_ACC_ERR_DMX_DECL 5

/*
 * Access is denied to DMA_M0. Only the first error among (HMD, DMA_M0,
 * DMA_M1) is recorded until the error status is cleared
 */
#define IAXXX_H2P3_ACC_ERR_DMA_M0_MASK 0x00000040
#define IAXXX_H2P3_ACC_ERR_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P3_ACC_ERR_DMA_M0_POS 6
#define IAXXX_H2P3_ACC_ERR_DMA_M0_SIZE 1
#define IAXXX_H2P3_ACC_ERR_DMA_M0_DECL 6

/*
 * Access is denied to DMA_M1. Only the first error among (HMD, DMA_M0,
 * DMA_M1) is recorded until the error status is cleared
 */
#define IAXXX_H2P3_ACC_ERR_DMA_M1_MASK 0x00000080
#define IAXXX_H2P3_ACC_ERR_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P3_ACC_ERR_DMA_M1_POS 7
#define IAXXX_H2P3_ACC_ERR_DMA_M1_SIZE 1
#define IAXXX_H2P3_ACC_ERR_DMA_M1_DECL 7

/*
 * Reserved (DAP can never be denied access)
 */
#define IAXXX_H2P3_ACC_ERR_DAP_MASK 0x00000100
#define IAXXX_H2P3_ACC_ERR_DAP_RESET_VAL 0x0
#define IAXXX_H2P3_ACC_ERR_DAP_POS 8
#define IAXXX_H2P3_ACC_ERR_DAP_SIZE 1
#define IAXXX_H2P3_ACC_ERR_DAP_DECL 8

/*** H2P3_ACC_ERR_PADDR (0x40051028) ***/
/*
 * This register has the H2P address for which access is denied.
 */
#define IAXXX_H2P3_ACC_ERR_PADDR_ADDR (0x40051028)
#define IAXXX_H2P3_ACC_ERR_PADDR_MASK_VAL 0x003fffff
#define IAXXX_H2P3_ACC_ERR_PADDR_RMASK_VAL 0x003fffff
#define IAXXX_H2P3_ACC_ERR_PADDR_WMASK_VAL 0x00000000
#define IAXXX_H2P3_ACC_ERR_PADDR_RESET_VAL 0x00000000

/*
 * H2P Address for which access is denied
 */
#define IAXXX_H2P3_ACC_ERR_PADDR_ADDRESS_MASK 0x003fffff
#define IAXXX_H2P3_ACC_ERR_PADDR_ADDRESS_RESET_VAL 0x0
#define IAXXX_H2P3_ACC_ERR_PADDR_ADDRESS_POS 0
#define IAXXX_H2P3_ACC_ERR_PADDR_ADDRESS_SIZE 22
#define IAXXX_H2P3_ACC_ERR_PADDR_ADDRESS_DECL (21:0)

/* Number of registers in the module */
#define IAXXX_H2P3_REG_NUM 11

#endif /* __IAXXX_REGISTER_DEFS_H2P3_H__*/