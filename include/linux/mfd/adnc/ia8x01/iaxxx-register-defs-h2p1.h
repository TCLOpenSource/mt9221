/*
 * iaxxx-register-defs-h2p1.h
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

#ifndef __IAXXX_REGISTER_DEFS_H2P1_H__
#define __IAXXX_REGISTER_DEFS_H2P1_H__

/*** The base address for this set of registers ***/
#define IAXXX_H2P1_REGS_ADDR (0x40039000)

/*** H2P1_H2P1_MSTR_ACC (0x40039000) ***/
/*
 * This register has the access protection controls.
 *
 When access permission is set to "0", write towards any of the H2P1
 * registers generates an error response. Read operation always returns the
 * register content.
 */
#define IAXXX_H2P1_H2P1_MSTR_ACC_ADDR (0x40039000)
#define IAXXX_H2P1_H2P1_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P1_H2P1_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P1_H2P1_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P1_H2P1_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to H2P1 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_H2P1_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P1_H2P1_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P1_H2P1_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P1_H2P1_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P1_H2P1_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access H2P1
 */
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to H2P1 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to H2P1 registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P1_H2P1_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access H2P1
 */
#define IAXXX_H2P1_H2P1_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P1_H2P1_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P1_H2P1_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P1_H2P1_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P1_H2P1_MSTR_ACC_DAP_DECL 8

/*** H2P1_HMD_MSTR_ACC (0x40039004) ***/
/*
 * This register has the access protection controls.
 */
#define IAXXX_H2P1_HMD_MSTR_ACC_ADDR (0x40039004)
#define IAXXX_H2P1_HMD_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P1_HMD_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P1_HMD_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P1_HMD_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to HMD registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_HMD_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P1_HMD_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P1_HMD_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P1_HMD_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P1_HMD_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access HMD
 */
#define IAXXX_H2P1_HMD_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P1_HMD_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P1_HMD_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P1_HMD_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P1_HMD_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to HMD registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_HMD_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P1_HMD_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P1_HMD_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P1_HMD_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P1_HMD_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to HMD registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_HMD_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P1_HMD_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P1_HMD_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P1_HMD_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P1_HMD_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access HMD
 */
#define IAXXX_H2P1_HMD_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P1_HMD_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P1_HMD_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P1_HMD_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P1_HMD_MSTR_ACC_DAP_DECL 8

/*** H2P1_DMX_MSTR_ACC (0x40039008) ***/
/*
 * This register has the access protection controls.
 */
#define IAXXX_H2P1_DMX_MSTR_ACC_ADDR (0x40039008)
#define IAXXX_H2P1_DMX_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P1_DMX_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P1_DMX_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P1_DMX_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to DMX registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_DMX_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P1_DMX_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P1_DMX_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P1_DMX_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P1_DMX_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access DMX
 */
#define IAXXX_H2P1_DMX_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P1_DMX_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P1_DMX_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P1_DMX_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P1_DMX_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to DMX registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_DMX_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P1_DMX_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P1_DMX_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P1_DMX_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P1_DMX_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to DMX registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_DMX_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P1_DMX_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P1_DMX_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P1_DMX_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P1_DMX_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access DMX
 */
#define IAXXX_H2P1_DMX_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P1_DMX_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P1_DMX_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P1_DMX_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P1_DMX_MSTR_ACC_DAP_DECL 8

/*** H2P1_MSW_MSTR_ACC (0x4003900c) ***/
/*
 * This register has the access protection controls.
 */
#define IAXXX_H2P1_MSW_MSTR_ACC_ADDR (0x4003900c)
#define IAXXX_H2P1_MSW_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P1_MSW_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P1_MSW_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P1_MSW_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to MSW registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_MSW_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P1_MSW_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P1_MSW_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P1_MSW_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P1_MSW_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access MSW
 */
#define IAXXX_H2P1_MSW_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P1_MSW_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P1_MSW_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P1_MSW_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P1_MSW_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to MSW registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_MSW_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P1_MSW_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P1_MSW_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P1_MSW_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P1_MSW_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to MSW registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_MSW_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P1_MSW_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P1_MSW_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P1_MSW_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P1_MSW_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access MSW
 */
#define IAXXX_H2P1_MSW_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P1_MSW_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P1_MSW_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P1_MSW_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P1_MSW_MSTR_ACC_DAP_DECL 8

/*** H2P1_DEBUG_MSTR_ACC (0x40039010) ***/
/*
 * This register has the access protection controls.
 */
#define IAXXX_H2P1_DEBUG_MSTR_ACC_ADDR (0x40039010)
#define IAXXX_H2P1_DEBUG_MSTR_ACC_MASK_VAL 0x000001f0
#define IAXXX_H2P1_DEBUG_MSTR_ACC_RMASK_VAL 0x000001f0
#define IAXXX_H2P1_DEBUG_MSTR_ACC_WMASK_VAL 0x000000d0
#define IAXXX_H2P1_DEBUG_MSTR_ACC_RESET_VAL 0x00000130

/*
 * Controls HMD access to DEBUG registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_DEBUG_MSTR_ACC_HMD_MASK 0x00000010
#define IAXXX_H2P1_DEBUG_MSTR_ACC_HMD_RESET_VAL 0x1
#define IAXXX_H2P1_DEBUG_MSTR_ACC_HMD_POS 4
#define IAXXX_H2P1_DEBUG_MSTR_ACC_HMD_SIZE 1
#define IAXXX_H2P1_DEBUG_MSTR_ACC_HMD_DECL 4

/*
 * DMX can always access DEBUG
 */
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMX_MASK 0x00000020
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMX_RESET_VAL 0x1
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMX_POS 5
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMX_SIZE 1
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMX_DECL 5

/*
 * Controls DMA_M0 access to DEBUG registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMA_M0_MASK 0x00000040
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMA_M0_POS 6
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMA_M0_SIZE 1
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMA_M0_DECL 6

/*
 * Controls DMA_M1 access to DEBUG registers through APB. '1' means access is
 * granted
 */
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMA_M1_MASK 0x00000080
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMA_M1_POS 7
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMA_M1_SIZE 1
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DMA_M1_DECL 7

/*
 * DAP can always access DEBUG
 */
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DAP_MASK 0x00000100
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DAP_RESET_VAL 0x1
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DAP_POS 8
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DAP_SIZE 1
#define IAXXX_H2P1_DEBUG_MSTR_ACC_DAP_DECL 8

/*** H2P1_ACC_ERR (0x40039014) ***/
/*
 * This register has access information that is denied. Only the first error
 * is recorded until the error status is cleared.
 */
#define IAXXX_H2P1_ACC_ERR_ADDR (0x40039014)
#define IAXXX_H2P1_ACC_ERR_MASK_VAL 0x000001f0
#define IAXXX_H2P1_ACC_ERR_RMASK_VAL 0x000001f0
#define IAXXX_H2P1_ACC_ERR_WMASK_VAL 0x000000d0
#define IAXXX_H2P1_ACC_ERR_RESET_VAL 0x00000000

/*
 * Access is denied to HMD. Only the first error among (HMD, DMA_M0, DMA_M1)
 * is recorded until the error status is cleared
 */
#define IAXXX_H2P1_ACC_ERR_HMD_MASK 0x00000010
#define IAXXX_H2P1_ACC_ERR_HMD_RESET_VAL 0x0
#define IAXXX_H2P1_ACC_ERR_HMD_POS 4
#define IAXXX_H2P1_ACC_ERR_HMD_SIZE 1
#define IAXXX_H2P1_ACC_ERR_HMD_DECL 4

/*
 * Reserved (DMX can never be denied access)
 */
#define IAXXX_H2P1_ACC_ERR_DMX_MASK 0x00000020
#define IAXXX_H2P1_ACC_ERR_DMX_RESET_VAL 0x0
#define IAXXX_H2P1_ACC_ERR_DMX_POS 5
#define IAXXX_H2P1_ACC_ERR_DMX_SIZE 1
#define IAXXX_H2P1_ACC_ERR_DMX_DECL 5

/*
 * Access is denied to DMA_M0. Only the first error among (HMD, DMA_M0,
 * DMA_M1) is recorded until the error status is cleared
 */
#define IAXXX_H2P1_ACC_ERR_DMA_M0_MASK 0x00000040
#define IAXXX_H2P1_ACC_ERR_DMA_M0_RESET_VAL 0x0
#define IAXXX_H2P1_ACC_ERR_DMA_M0_POS 6
#define IAXXX_H2P1_ACC_ERR_DMA_M0_SIZE 1
#define IAXXX_H2P1_ACC_ERR_DMA_M0_DECL 6

/*
 * Access is denied to DMA_M1. Only the first error among (HMD, DMA_M0,
 * DMA_M1) is recorded until the error status is cleared
 */
#define IAXXX_H2P1_ACC_ERR_DMA_M1_MASK 0x00000080
#define IAXXX_H2P1_ACC_ERR_DMA_M1_RESET_VAL 0x0
#define IAXXX_H2P1_ACC_ERR_DMA_M1_POS 7
#define IAXXX_H2P1_ACC_ERR_DMA_M1_SIZE 1
#define IAXXX_H2P1_ACC_ERR_DMA_M1_DECL 7

/*
 * Reserved (DAP can never be denied access)
 */
#define IAXXX_H2P1_ACC_ERR_DAP_MASK 0x00000100
#define IAXXX_H2P1_ACC_ERR_DAP_RESET_VAL 0x0
#define IAXXX_H2P1_ACC_ERR_DAP_POS 8
#define IAXXX_H2P1_ACC_ERR_DAP_SIZE 1
#define IAXXX_H2P1_ACC_ERR_DAP_DECL 8

/*** H2P1_ACC_ERR_PADDR (0x40039018) ***/
/*
 * This register has the H2P address for which access is denied.
 */
#define IAXXX_H2P1_ACC_ERR_PADDR_ADDR (0x40039018)
#define IAXXX_H2P1_ACC_ERR_PADDR_MASK_VAL 0xffffffff
#define IAXXX_H2P1_ACC_ERR_PADDR_RMASK_VAL 0xffffffff
#define IAXXX_H2P1_ACC_ERR_PADDR_WMASK_VAL 0x00000000
#define IAXXX_H2P1_ACC_ERR_PADDR_RESET_VAL 0x00000000

/*
 * H2P Address for which access is denied
 */
#define IAXXX_H2P1_ACC_ERR_PADDR_ADDRESS_MASK 0xffffffff
#define IAXXX_H2P1_ACC_ERR_PADDR_ADDRESS_RESET_VAL 0x0
#define IAXXX_H2P1_ACC_ERR_PADDR_ADDRESS_POS 0
#define IAXXX_H2P1_ACC_ERR_PADDR_ADDRESS_SIZE 32
#define IAXXX_H2P1_ACC_ERR_PADDR_ADDRESS_DECL (31:0)

/* Number of registers in the module */
#define IAXXX_H2P1_REG_NUM 7

#endif /* __IAXXX_REGISTER_DEFS_H2P1_H__*/