/*
 * iaxxx-register-defs-fpga.h
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

#ifndef __IAXXX_REGISTER_DEFS_FPGA_H__
#define __IAXXX_REGISTER_DEFS_FPGA_H__

/*** The base address for this set of registers ***/
#define IAXXX_FPGA_REGS_ADDR (0x40005000)

/*** FPGA_BITSTREAM_VERSION (0x40005000) ***/
/*
 * FPGA bitstream version/RTL Snapshot timestamp
 */
#define IAXXX_FPGA_BITSTREAM_VERSION_ADDR (0x40005000)
#define IAXXX_FPGA_BITSTREAM_VERSION_MASK_VAL 0xffffff3f
#define IAXXX_FPGA_BITSTREAM_VERSION_RMASK_VAL 0xffffff3f
#define IAXXX_FPGA_BITSTREAM_VERSION_WMASK_VAL 0x00000000
#define IAXXX_FPGA_BITSTREAM_VERSION_RESET_VAL 0x79e31007

/*
 * Die ID:
 * 0x7 = D4080
 */
#define IAXXX_FPGA_BITSTREAM_VERSION_DIE_ID_MASK 0x0000003f
#define IAXXX_FPGA_BITSTREAM_VERSION_DIE_ID_RESET_VAL 0x7
#define IAXXX_FPGA_BITSTREAM_VERSION_DIE_ID_POS 0
#define IAXXX_FPGA_BITSTREAM_VERSION_DIE_ID_SIZE 6
#define IAXXX_FPGA_BITSTREAM_VERSION_DIE_ID_DECL (5:0)

/*
 * Minor Version of the RTL
 */
#define IAXXX_FPGA_BITSTREAM_VERSION_MINOR_VERSION_MASK 0x00000f00
#define IAXXX_FPGA_BITSTREAM_VERSION_MINOR_VERSION_RESET_VAL 0x0
#define IAXXX_FPGA_BITSTREAM_VERSION_MINOR_VERSION_POS 8
#define IAXXX_FPGA_BITSTREAM_VERSION_MINOR_VERSION_SIZE 4
#define IAXXX_FPGA_BITSTREAM_VERSION_MINOR_VERSION_DECL (11:8)

/*
 * Major Version of the RTL
 */
#define IAXXX_FPGA_BITSTREAM_VERSION_MAJOR_VERSION_MASK 0x0000f000
#define IAXXX_FPGA_BITSTREAM_VERSION_MAJOR_VERSION_RESET_VAL 0x1
#define IAXXX_FPGA_BITSTREAM_VERSION_MAJOR_VERSION_POS 12
#define IAXXX_FPGA_BITSTREAM_VERSION_MAJOR_VERSION_SIZE 4
#define IAXXX_FPGA_BITSTREAM_VERSION_MAJOR_VERSION_DECL (15:12)

/*
 * Release Type:
 * 0x0: Internal Development
 * 0x1: Bronze
 * 0x2: Silver
 * 0x3: Gold
 */
#define IAXXX_FPGA_BITSTREAM_VERSION_REL_TYPE_MASK 0x001f0000
#define IAXXX_FPGA_BITSTREAM_VERSION_REL_TYPE_RESET_VAL 0x3
#define IAXXX_FPGA_BITSTREAM_VERSION_REL_TYPE_POS 16
#define IAXXX_FPGA_BITSTREAM_VERSION_REL_TYPE_SIZE 5
#define IAXXX_FPGA_BITSTREAM_VERSION_REL_TYPE_DECL (20:16)

/*
 * [0] PD4 (HMD) present
 * [1] PD5 (DMX) present
 * [2] DMA present
 */
#define IAXXX_FPGA_BITSTREAM_VERSION_PD_INFO_MASK 0x00e00000
#define IAXXX_FPGA_BITSTREAM_VERSION_PD_INFO_RESET_VAL 0x7
#define IAXXX_FPGA_BITSTREAM_VERSION_PD_INFO_POS 21
#define IAXXX_FPGA_BITSTREAM_VERSION_PD_INFO_SIZE 3
#define IAXXX_FPGA_BITSTREAM_VERSION_PD_INFO_DECL (23:21)

/*
 * FPGA Platform:
 * 0x77: VC707
 * 0x79: VC709
 * 0x2: S2C TAI Single FPGA Module
 */
#define IAXXX_FPGA_BITSTREAM_VERSION_PLATFORM_MASK 0xff000000
#define IAXXX_FPGA_BITSTREAM_VERSION_PLATFORM_RESET_VAL 0x79
#define IAXXX_FPGA_BITSTREAM_VERSION_PLATFORM_POS 24
#define IAXXX_FPGA_BITSTREAM_VERSION_PLATFORM_SIZE 8
#define IAXXX_FPGA_BITSTREAM_VERSION_PLATFORM_DECL (31:24)

/*** FPGA_HW_RESET (0x40005004) ***/
/*
 * FPGA HW reset
 */
#define IAXXX_FPGA_HW_RESET_ADDR (0x40005004)
#define IAXXX_FPGA_HW_RESET_MASK_VAL 0x00000001
#define IAXXX_FPGA_HW_RESET_RMASK_VAL 0x00000000
#define IAXXX_FPGA_HW_RESET_WMASK_VAL 0x00000001
#define IAXXX_FPGA_HW_RESET_RESET_VAL 0x00000000

/*
 * Write 1 to trigger FPGA HW reset.
 */
#define IAXXX_FPGA_HW_RESET_RESET_MASK 0x00000001
#define IAXXX_FPGA_HW_RESET_RESET_RESET_VAL 0x0
#define IAXXX_FPGA_HW_RESET_RESET_POS 0
#define IAXXX_FPGA_HW_RESET_RESET_SIZE 1
#define IAXXX_FPGA_HW_RESET_RESET_DECL 0

/*** FPGA_ROM_WR_EN (0x40005008) ***/
/*
 * ROM write access enable.
 */
#define IAXXX_FPGA_ROM_WR_EN_ADDR (0x40005008)
#define IAXXX_FPGA_ROM_WR_EN_MASK_VAL 0x00000001
#define IAXXX_FPGA_ROM_WR_EN_RMASK_VAL 0x00000001
#define IAXXX_FPGA_ROM_WR_EN_WMASK_VAL 0x00000001
#define IAXXX_FPGA_ROM_WR_EN_RESET_VAL 0x00000000

/*
 * Write 1 in order to enable write access to FPGA ROMs.
 */
#define IAXXX_FPGA_ROM_WR_EN_WR_EN_MASK 0x00000001
#define IAXXX_FPGA_ROM_WR_EN_WR_EN_RESET_VAL 0x0
#define IAXXX_FPGA_ROM_WR_EN_WR_EN_POS 0
#define IAXXX_FPGA_ROM_WR_EN_WR_EN_SIZE 1
#define IAXXX_FPGA_ROM_WR_EN_WR_EN_DECL 0

/*** FPGA_EFUSE (0x4000500c) ***/
/*
 * This register emulates efuse in FPGA. Its contents are preserved during
 * reset.
 */
#define IAXXX_FPGA_EFUSE_ADDR (0x4000500c)
#define IAXXX_FPGA_EFUSE_MASK_VAL 0xffffffff
#define IAXXX_FPGA_EFUSE_RMASK_VAL 0xffffffff
#define IAXXX_FPGA_EFUSE_WMASK_VAL 0xffefffff
#define IAXXX_FPGA_EFUSE_RESET_VAL 0x00000000

/*
 */
#define IAXXX_FPGA_EFUSE_EFUSE_PKG_CFG_MASK 0x000fffff
#define IAXXX_FPGA_EFUSE_EFUSE_PKG_CFG_RESET_VAL 0x0
#define IAXXX_FPGA_EFUSE_EFUSE_PKG_CFG_POS 0
#define IAXXX_FPGA_EFUSE_EFUSE_PKG_CFG_SIZE 20
#define IAXXX_FPGA_EFUSE_EFUSE_PKG_CFG_DECL (19:0)

/*
 */
#define IAXXX_FPGA_EFUSE_EFUSE_HMD_ENABLED_MASK 0x00100000
#define IAXXX_FPGA_EFUSE_EFUSE_HMD_ENABLED_RESET_VAL 0x0
#define IAXXX_FPGA_EFUSE_EFUSE_HMD_ENABLED_POS 20
#define IAXXX_FPGA_EFUSE_EFUSE_HMD_ENABLED_SIZE 1
#define IAXXX_FPGA_EFUSE_EFUSE_HMD_ENABLED_DECL 20

/*
 */
#define IAXXX_FPGA_EFUSE_EFUSE_MFG_MASK 0xffe00000
#define IAXXX_FPGA_EFUSE_EFUSE_MFG_RESET_VAL 0x0
#define IAXXX_FPGA_EFUSE_EFUSE_MFG_POS 21
#define IAXXX_FPGA_EFUSE_EFUSE_MFG_SIZE 11
#define IAXXX_FPGA_EFUSE_EFUSE_MFG_DECL (31:21)

/*** FPGA_SCRATCH (0x40005010) ***/
/*
 * Scratch pad. Its contents are preserved during reset.
 */
#define IAXXX_FPGA_SCRATCH_ADDR (0x40005010)
#define IAXXX_FPGA_SCRATCH_MASK_VAL 0xffffffff
#define IAXXX_FPGA_SCRATCH_RMASK_VAL 0xffffffff
#define IAXXX_FPGA_SCRATCH_WMASK_VAL 0xffffffff
#define IAXXX_FPGA_SCRATCH_RESET_VAL 0x00000000

/*
 */
#define IAXXX_FPGA_SCRATCH_EFUSE_MASK 0xffffffff
#define IAXXX_FPGA_SCRATCH_EFUSE_RESET_VAL 0x0
#define IAXXX_FPGA_SCRATCH_EFUSE_POS 0
#define IAXXX_FPGA_SCRATCH_EFUSE_SIZE 32
#define IAXXX_FPGA_SCRATCH_EFUSE_DECL (31:0)

/* Number of registers in the module */
#define IAXXX_FPGA_REG_NUM 5

#endif /* __IAXXX_REGISTER_DEFS_FPGA_H__*/
