/*
 * iaxxx-register-defs-bluetooth-group.h
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

#ifndef __IAXXX_REGISTER_DEFS_BLUETOOTH_GRP_H__
#define __IAXXX_REGISTER_DEFS_BLUETOOTH_GRP_H__

/*** The base address for this set of registers ***/
#define IAXXX_BLUETOOTH_GRP_REGS_ADDR (0x0c800000)

/*** BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_L (0x0c800000) ***/
/*
 * BD address of the device to be used with the current command.
 */
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_L_ADDR (0x0c800000)
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_L_MASK_VAL 0xffffffff
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_L_RMASK_VAL 0xffffffff
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_L_WMASK_VAL 0xffffffff
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_L_RESET_VAL 0x00000000

/*
 * BD Address.
 */
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_L_BD_ADDR_MASK 0xffffffff
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_L_BD_ADDR_RESET_VAL 0x0
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_L_BD_ADDR_POS 0
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_L_BD_ADDR_SIZE 32

/*** BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_M (0x0c800004) ***/
/*
 * BD address of the device to be used with the current command.
 */
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_M_ADDR (0x0c800004)
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_M_MASK_VAL 0xffffffff
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_M_RMASK_VAL 0xffffffff
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_M_WMASK_VAL 0xffffffff
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_M_RESET_VAL 0x00000000

/*
 * BD Address.
 */
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_M_BD_ADDR_MASK 0xffffffff
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_M_BD_ADDR_RESET_VAL 0x0
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_M_BD_ADDR_POS 0
#define IAXXX_BLUETOOTH_GRP_BLUETOOTH_DEVICE_ADDRESS_M_BD_ADDR_SIZE 32

/* Number of registers in the module */
#define IAXXX_BLUETOOTH_GRP_REG_NUM 2

#endif /* __IAXXX_REGISTER_DEFS_BLUETOOTH_GRP_H__ */