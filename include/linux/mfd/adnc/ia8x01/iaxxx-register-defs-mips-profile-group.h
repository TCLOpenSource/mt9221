/*
 * iaxxx-register-defs-mips-profile-group.h
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

#ifndef __IAXXX_REGISTER_DEFS_MIPS_PROFILE_GRP_H__
#define __IAXXX_REGISTER_DEFS_MIPS_PROFILE_GRP_H__

/*** The base address for this set of registers ***/
#define IAXXX_MIPS_PROFILE_GRP_REGS_ADDR (0x08800000)

/*** MIPS_PROFILE_GRP_IDLE_MIPS_CUR (0x08800000) ***/
/*
 * Current MIPS consumption in idle state.
 */
#define IAXXX_MIPS_PROFILE_GRP_IDLE_MIPS_CUR_ADDR (0x08800000)
#define IAXXX_MIPS_PROFILE_GRP_IDLE_MIPS_CUR_MASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_IDLE_MIPS_CUR_RMASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_IDLE_MIPS_CUR_WMASK_VAL 0x00000000
#define IAXXX_MIPS_PROFILE_GRP_IDLE_MIPS_CUR_RESET_VAL 0x00000000

/*
 * Current MIPS consumption in idle state.
 */
#define IAXXX_MIPS_PROFILE_GRP_IDLE_MIPS_CUR_REG_MASK 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_IDLE_MIPS_CUR_REG_RESET_VAL 0x0
#define IAXXX_MIPS_PROFILE_GRP_IDLE_MIPS_CUR_REG_POS 0
#define IAXXX_MIPS_PROFILE_GRP_IDLE_MIPS_CUR_REG_SIZE 32

/*** MIPS_PROFILE_GRP_SYSTEM_MIPS_PEAK (0x08800004) ***/
/*
 * Peak MIPS consumption of the system.
 */
#define IAXXX_MIPS_PROFILE_GRP_SYSTEM_MIPS_PEAK_ADDR (0x08800004)
#define IAXXX_MIPS_PROFILE_GRP_SYSTEM_MIPS_PEAK_MASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_SYSTEM_MIPS_PEAK_RMASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_SYSTEM_MIPS_PEAK_WMASK_VAL 0x00000000
#define IAXXX_MIPS_PROFILE_GRP_SYSTEM_MIPS_PEAK_RESET_VAL 0x00000000

/*
 * Peak MIPS consumption of the system.
 */
#define IAXXX_MIPS_PROFILE_GRP_SYSTEM_MIPS_PEAK_REG_MASK 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_SYSTEM_MIPS_PEAK_REG_RESET_VAL 0x0
#define IAXXX_MIPS_PROFILE_GRP_SYSTEM_MIPS_PEAK_REG_POS 0
#define IAXXX_MIPS_PROFILE_GRP_SYSTEM_MIPS_PEAK_REG_SIZE 32

/*** MIPS_PROFILE_GRP_ISR_MIPS_CUR (0x08800008) ***/
/*
 * Current MIPS consumption in ISRs.
 */
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_CUR_ADDR (0x08800008)
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_CUR_MASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_CUR_RMASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_CUR_WMASK_VAL 0x00000000
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_CUR_RESET_VAL 0x00000000

/*
 * Current MIPS consumption in ISRs.
 */
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_CUR_REG_MASK 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_CUR_REG_RESET_VAL 0x0
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_CUR_REG_POS 0
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_CUR_REG_SIZE 32

/*** MIPS_PROFILE_GRP_ISR_MIPS_PEAK (0x0880000c) ***/
/*
 * Peak MIPS consumption in ISRs.
 */
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_PEAK_ADDR (0x0880000c)
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_PEAK_MASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_PEAK_RMASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_PEAK_WMASK_VAL 0x00000000
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_PEAK_RESET_VAL 0x00000000

/*
 * Peak MIPS consumption in ISRs.
 */
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_PEAK_REG_MASK 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_PEAK_REG_RESET_VAL 0x0
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_PEAK_REG_POS 0
#define IAXXX_MIPS_PROFILE_GRP_ISR_MIPS_PEAK_REG_SIZE 32

/*** MIPS_PROFILE_GRP_GENTASK_MIPS_CUR (0x08800010) ***/
/*
 * Current MIPS consumption in general tasks.
 */
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_CUR_ADDR (0x08800010)
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_CUR_MASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_CUR_RMASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_CUR_WMASK_VAL 0x00000000
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_CUR_RESET_VAL 0x00000000

/*
 * Current MIPS consumption in general tasks.
 */
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_CUR_REG_MASK 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_CUR_REG_RESET_VAL 0x0
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_CUR_REG_POS 0
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_CUR_REG_SIZE 32

/*** MIPS_PROFILE_GRP_GENTASK_MIPS_PEAK (0x08800014) ***/
/*
 * Peak MIPS consumption in general tasks.
 */
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_PEAK_ADDR (0x08800014)
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_PEAK_MASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_PEAK_RMASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_PEAK_WMASK_VAL 0x00000000
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_PEAK_RESET_VAL 0x00000000

/*
 * Peak MIPS consumption in general tasks.
 */
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_PEAK_REG_MASK 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_PEAK_REG_RESET_VAL 0x0
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_PEAK_REG_POS 0
#define IAXXX_MIPS_PROFILE_GRP_GENTASK_MIPS_PEAK_REG_SIZE 32

/*** MIPS_PROFILE_GRP_PLUGINS_MIPS_CUR (0x08800018) ***/
/*
 * Current MIPS consumption of all plugins running on this processor.
 */
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_CUR_ADDR (0x08800018)
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_CUR_MASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_CUR_RMASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_CUR_WMASK_VAL 0x00000000
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_CUR_RESET_VAL 0x00000000

/*
 * Current MIPS consumption of all plugins running on this processor.
 */
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_CUR_REG_MASK 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_CUR_REG_RESET_VAL 0x0
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_CUR_REG_POS 0
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_CUR_REG_SIZE 32

/*** MIPS_PROFILE_GRP_PLUGINS_MIPS_PEAK (0x0880001c) ***/
/*
 * Peak MIPS consumption of all plugins running on this processor.
 */
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_PEAK_ADDR (0x0880001c)
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_PEAK_MASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_PEAK_RMASK_VAL 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_PEAK_WMASK_VAL 0x00000000
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_PEAK_RESET_VAL 0x00000000

/*
 * Peak MIPS consumption of all plugins running on this processor.
 */
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_PEAK_REG_MASK 0xffffffff
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_PEAK_REG_RESET_VAL 0x0
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_PEAK_REG_POS 0
#define IAXXX_MIPS_PROFILE_GRP_PLUGINS_MIPS_PEAK_REG_SIZE 32

/*** MIPS_PROFILE_GRP_CORE_PROF_STATE (0x08800020) ***/
/*
 * Profiling State of the Processor Core.
 */
#define IAXXX_MIPS_PROFILE_GRP_CORE_PROF_STATE_ADDR (0x08800020)
#define IAXXX_MIPS_PROFILE_GRP_CORE_PROF_STATE_MASK_VAL 0x00000001
#define IAXXX_MIPS_PROFILE_GRP_CORE_PROF_STATE_RMASK_VAL 0x00000001
#define IAXXX_MIPS_PROFILE_GRP_CORE_PROF_STATE_WMASK_VAL 0x00000000
#define IAXXX_MIPS_PROFILE_GRP_CORE_PROF_STATE_RESET_VAL 0x00000000

/*
 * MIPS Profiling state of the core
 * 0 - Disabled
 * 1 - Enabled
 */
#define IAXXX_MIPS_PROFILE_GRP_CORE_PROF_STATE_PROF_STATE_MASK 0x00000001
#define IAXXX_MIPS_PROFILE_GRP_CORE_PROF_STATE_PROF_STATE_RESET_VAL 0x0
#define IAXXX_MIPS_PROFILE_GRP_CORE_PROF_STATE_PROF_STATE_POS 0
#define IAXXX_MIPS_PROFILE_GRP_CORE_PROF_STATE_PROF_STATE_SIZE 1

/*** MIPS_PROFILE_GRP_RESET_MIPS_VALUES (0x08800024) ***/
/*
 * Resetting current/peak MIPS values of the Processor Core. Host need to set
 * the respective bit as per the need and then once reset is done device
 * would clear the particular bit.
 */
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_ADDR (0x08800024)
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_MASK_VAL 0x00000003
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_RMASK_VAL 0x00000003
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_WMASK_VAL 0x00000003
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_RESET_VAL 0x00000000

/*
 * Reset Current MIPS of the core
 * 0 - Disabled
 * 1 - Enabled
 */
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_RESET_MIPS_CURR_MASK 0x00000001
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_RESET_MIPS_CURR_RESET_VAL 0x0
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_RESET_MIPS_CURR_POS 0
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_RESET_MIPS_CURR_SIZE 1

/*
 * Reset Peak MIPS of the core
 * 0 - Disabled
 * 1 - Enabled
 */
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_RESET_MIPS_PEAK_MASK 0x00000002
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_RESET_MIPS_PEAK_RESET_VAL 0x0
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_RESET_MIPS_PEAK_POS 1
#define IAXXX_MIPS_PROFILE_GRP_RESET_MIPS_VALUES_RESET_MIPS_PEAK_SIZE 1

/* Number of registers in the module */
#define IAXXX_MIPS_PROFILE_GRP_REG_NUM 10

#endif /* __IAXXX_REGISTER_DEFS_MIPS_PROFILE_GRP_H__ */