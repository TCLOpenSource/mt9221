/*
 * iaxxx-register-defs-battery-stat.h
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

#ifndef __IAXXX_REGISTER_DEFS_BATTERY_STAT_H__
#define __IAXXX_REGISTER_DEFS_BATTERY_STAT_H__

/*** The base address for this set of registers ***/
#define IAXXX_BATTERY_STAT_REGS_ADDR (0x09000000)

/*** BATTERY_STAT_BATTERY_STATUS (0x09000000) ***/
/*
 * Battery status Register.
 */
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_ADDR (0x09000000)
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_MASK_VAL 0x0000007f
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_RMASK_VAL 0x00000000
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_WMASK_VAL 0x00000000
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_RESET_VAL 0x00000000

/*
 * Battery insertion is detected.
 */
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_INSERTED_MASK 0x00000001
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_INSERTED_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_INSERTED_POS 0
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_INSERTED_SIZE 1

/*
 * Battery full-charge is detected.
 */
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_FULL_CHARGE_MASK 0x00000002
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_FULL_CHARGE_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_FULL_CHARGE_POS 1
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_FULL_CHARGE_SIZE 1

/*
 * Current battery level is <= low threshold(SOC1).
 */
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_LOW_BATTERY_MASK 0x00000004
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_LOW_BATTERY_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_LOW_BATTERY_POS 2
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_LOW_BATTERY_SIZE 1

/*
 * Battery fast charging is allowed.
 */
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_FAST_CHARGING_MASK 0x00000008
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_FAST_CHARGING_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_FAST_CHARGING_POS 3
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_FAST_CHARGING_SIZE 1

/*
 * Battery discharging is detected.
 */
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_DISCHARGING_MASK 0x00000010
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_DISCHARGING_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_DISCHARGING_POS 4
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_DISCHARGING_SIZE 1

/*
 * Over-Temperature condition is detected.
 */
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_OVER_TEMP_MASK 0x00000020
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_OVER_TEMP_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_OVER_TEMP_POS 5
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_OVER_TEMP_SIZE 1

/*
 * Under-Temperature condition is detected.
 */
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_UNDER_TEMP_MASK 0x00000040
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_UNDER_TEMP_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_UNDER_TEMP_POS 6
#define IAXXX_BATTERY_STAT_BATTERY_STATUS_UNDER_TEMP_SIZE 1

/*** BATTERY_STAT_TEMPERATURE (0x09000004) ***/
/*
 * Battery temperature in units of 0.1K.
 */
#define IAXXX_BATTERY_STAT_TEMPERATURE_ADDR (0x09000004)
#define IAXXX_BATTERY_STAT_TEMPERATURE_MASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_TEMPERATURE_RMASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_TEMPERATURE_WMASK_VAL 0x00000000
#define IAXXX_BATTERY_STAT_TEMPERATURE_RESET_VAL 0x00000000

/*
 * Battery temperature in units of 0.1K.
 */
#define IAXXX_BATTERY_STAT_TEMPERATURE_REG_MASK 0xffffffff
#define IAXXX_BATTERY_STAT_TEMPERATURE_REG_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_TEMPERATURE_REG_POS 0
#define IAXXX_BATTERY_STAT_TEMPERATURE_REG_SIZE 32

/*** BATTERY_STAT_VOLTAGE (0x09000008) ***/
/*
 * Battery Cell-pack voltage in mV with a range of 0 to 6000 mV.
 */
#define IAXXX_BATTERY_STAT_VOLTAGE_ADDR (0x09000008)
#define IAXXX_BATTERY_STAT_VOLTAGE_MASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_VOLTAGE_RMASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_VOLTAGE_WMASK_VAL 0x00000000
#define IAXXX_BATTERY_STAT_VOLTAGE_RESET_VAL 0x00000000

/*
 * Battery Cell-pack voltage in mV with a range of 0 to 6000 mV.
 */
#define IAXXX_BATTERY_STAT_VOLTAGE_REG_MASK 0xffffffff
#define IAXXX_BATTERY_STAT_VOLTAGE_REG_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_VOLTAGE_REG_POS 0
#define IAXXX_BATTERY_STAT_VOLTAGE_REG_SIZE 32

/*** BATTERY_STAT_REMAINING_CAPACITY (0x0900000c) ***/
/*
 * Battery remaining capacity in mAh.
 */
#define IAXXX_BATTERY_STAT_REMAINING_CAPACITY_ADDR (0x0900000c)
#define IAXXX_BATTERY_STAT_REMAINING_CAPACITY_MASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_REMAINING_CAPACITY_RMASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_REMAINING_CAPACITY_WMASK_VAL 0x00000000
#define IAXXX_BATTERY_STAT_REMAINING_CAPACITY_RESET_VAL 0x00000000

/*
 * Battery remaining capacity in mAh.
 */
#define IAXXX_BATTERY_STAT_REMAINING_CAPACITY_REG_MASK 0xffffffff
#define IAXXX_BATTERY_STAT_REMAINING_CAPACITY_REG_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_REMAINING_CAPACITY_REG_POS 0
#define IAXXX_BATTERY_STAT_REMAINING_CAPACITY_REG_SIZE 32

/*** BATTERY_STAT_FULLCHARGE_CAPACITY (0x09000010) ***/
/*
 * Battery full charge capacity when fully charged in mAh.
 */
#define IAXXX_BATTERY_STAT_FULLCHARGE_CAPACITY_ADDR (0x09000010)
#define IAXXX_BATTERY_STAT_FULLCHARGE_CAPACITY_MASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_FULLCHARGE_CAPACITY_RMASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_FULLCHARGE_CAPACITY_WMASK_VAL 0x00000000
#define IAXXX_BATTERY_STAT_FULLCHARGE_CAPACITY_RESET_VAL 0x00000000

/*
 * Battery full charge capacity when fully charged in mAh.
 */
#define IAXXX_BATTERY_STAT_FULLCHARGE_CAPACITY_REG_MASK 0xffffffff
#define IAXXX_BATTERY_STAT_FULLCHARGE_CAPACITY_REG_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_FULLCHARGE_CAPACITY_REG_POS 0
#define IAXXX_BATTERY_STAT_FULLCHARGE_CAPACITY_REG_SIZE 32

/*** BATTERY_STAT_AVERAGE_CURRENT (0x09000014) ***/
/*
 * Battery average current flow through the sense resistor in mAh.
 */
#define IAXXX_BATTERY_STAT_AVERAGE_CURRENT_ADDR (0x09000014)
#define IAXXX_BATTERY_STAT_AVERAGE_CURRENT_MASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_AVERAGE_CURRENT_RMASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_AVERAGE_CURRENT_WMASK_VAL 0x00000000
#define IAXXX_BATTERY_STAT_AVERAGE_CURRENT_RESET_VAL 0x00000000

/*
 * Battery average current flow through the sense resistor in mAh.
 */
#define IAXXX_BATTERY_STAT_AVERAGE_CURRENT_REG_MASK 0xffffffff
#define IAXXX_BATTERY_STAT_AVERAGE_CURRENT_REG_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_AVERAGE_CURRENT_REG_POS 0
#define IAXXX_BATTERY_STAT_AVERAGE_CURRENT_REG_SIZE 32

/*** BATTERY_STAT_AVERAGE_POWER (0x09000018) ***/
/*
 * Battery average power during charging and discharging in mW.
 */
#define IAXXX_BATTERY_STAT_AVERAGE_POWER_ADDR (0x09000018)
#define IAXXX_BATTERY_STAT_AVERAGE_POWER_MASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_AVERAGE_POWER_RMASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_AVERAGE_POWER_WMASK_VAL 0x00000000
#define IAXXX_BATTERY_STAT_AVERAGE_POWER_RESET_VAL 0x00000000

/*
 * Battery average power during charging and discharging in mW.
 */
#define IAXXX_BATTERY_STAT_AVERAGE_POWER_REG_MASK 0xffffffff
#define IAXXX_BATTERY_STAT_AVERAGE_POWER_REG_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_AVERAGE_POWER_REG_POS 0
#define IAXXX_BATTERY_STAT_AVERAGE_POWER_REG_SIZE 32

/*** BATTERY_STAT_CURRENT_LEVEL (0x0900001c) ***/
/*
 * Battery remaining capacity in percentage from 0 to 100%.
 */
#define IAXXX_BATTERY_STAT_CURRENT_LEVEL_ADDR (0x0900001c)
#define IAXXX_BATTERY_STAT_CURRENT_LEVEL_MASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_CURRENT_LEVEL_RMASK_VAL 0xffffffff
#define IAXXX_BATTERY_STAT_CURRENT_LEVEL_WMASK_VAL 0x00000000
#define IAXXX_BATTERY_STAT_CURRENT_LEVEL_RESET_VAL 0x00000000

/*
 * Battery remaining capacity in percentage from 0 to 100%.
 */
#define IAXXX_BATTERY_STAT_CURRENT_LEVEL_REG_MASK 0xffffffff
#define IAXXX_BATTERY_STAT_CURRENT_LEVEL_REG_RESET_VAL 0x0
#define IAXXX_BATTERY_STAT_CURRENT_LEVEL_REG_POS 0
#define IAXXX_BATTERY_STAT_CURRENT_LEVEL_REG_SIZE 32

/* Number of registers in the module */
#define IAXXX_BATTERY_STAT_REG_NUM 8

#endif /* __IAXXX_REGISTER_DEFS_BATTERY_STAT_H__ */
