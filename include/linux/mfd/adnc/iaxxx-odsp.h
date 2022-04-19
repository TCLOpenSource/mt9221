
/*
 * iaxxx-odsp.h  --  IAXXX odsp header file
 *
 * Copyright 2017 Knowles, Inc.
 *
 * Author: Sharada Kumar <Sharada.Kumar@knowles.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __IAXXX_ODSP_H__
#define __IAXXX_ODSP_H__

#define MAX_IAXXX_DEVICES	8

#define IAXXX_MAX_FILENAME_SIZE 256

enum {
	IAXXX_DYNAMIC_PKG,
	IAXXX_STATIC_PKG,
	INVALID_PKG
};

enum {
	IAXXX_CHIP_IDLE,
	IAXXX_CHIP_ACTIVE
};

struct iaxxx_plugin_info {
	uint32_t plg_idx;
	uint32_t pkg_id;
	uint32_t block_id;
	uint32_t inst_id;
	uint32_t priority;
	uint32_t config_id;
};

struct iaxxx_plugin_param {
	uint32_t inst_id;
	uint32_t param_id;
	uint32_t param_val;
	uint8_t block_id;

};

struct iaxxx_plugin_param_blk {
	uint32_t inst_id;
	uint32_t param_size;
	uint64_t param_blk;
	uint8_t block_id;
	uint32_t id;
	char file_name[IAXXX_MAX_FILENAME_SIZE];
};

struct iaxxx_plugin_create_cfg {
	char file_name[256];
	uint32_t inst_id;
	uint32_t cfg_size;
	uint64_t cfg_val;
	uint8_t block_id;
};

struct iaxxx_set_event {
	uint8_t inst_id;
	uint32_t event_enable_mask;
	uint32_t block_id;
};

struct iaxxx_evt_info {
	uint16_t src_id;
	uint16_t event_id;
	uint16_t dst_id;
	uint32_t dst_opaque;
};

struct iaxxx_get_event {
	uint16_t event_src;
	uint16_t event_id;
	uint32_t data;
};

struct iaxxx_evt_trigger {
	uint16_t src_id;
	uint16_t evt_id;
	uint32_t src_opaque;
};

struct iaxxx_pkg_mgmt_info {
	char pkg_name[256];
	uint32_t pkg_id;
	uint32_t proc_id;
};

struct iaxxx_pkg_type {
	uint32_t pkg_id;
	uint8_t pkg_type;
};

struct iaxxx_set_chan_gain {
	uint8_t id;
	int8_t target_gain;
	uint16_t gain_ramp;
	uint8_t block_id;
};

struct iaxxx_script_info {
	char script_name[IAXXX_MAX_FILENAME_SIZE];
	uint32_t script_id;
};

/* IOCTL Magic character */
#define IAXXX_ODSP_IOCTL_MAGIC 'I'

/* Create IOCTL */
#define ODSP_PLG_CREATE _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x11)
#define ODSP_PLG_RESET _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x12)
#define ODSP_PLG_ENABLE _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x13)
#define ODSP_PLG_DISABLE _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x14)
#define ODSP_PLG_DESTROY _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x15)
#define ODSP_PLG_SET_PARAM _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x16)
#define ODSP_PLG_GET_PARAM _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x17)
#define ODSP_PLG_SET_PARAM_BLK _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x18)
#define ODSP_PLG_SET_CREATE_CFG _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x19)
#define ODSP_PLG_SET_EVENT _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x1A)
#define ODSP_EVENT_SUBSCRIBE _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x1B)
#define ODSP_GET_EVENT _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x1C)
#define ODSP_EVENT_UNSUBSCRIBE _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x1D)
#define ODSP_LOAD_PACKAGE _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x1E)
#define ODSP_UNLOAD_PACKAGE _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x1F)
#define ODSP_GET_FW_STATE _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x20)
#define ODSP_SET_CHAN_GAIN _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x21)
#define ODSP_SCRIPT_TRIGGER _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x22)
#define ODSP_SCRIPT_LOAD _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x23)
#define ODSP_SCRIPT_UNLOAD _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x24)
#define ODSP_EVENT_TRIGGER _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x25)
#define ODSP_UPDATE_INTF_SPEED _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x26)
#define ODSP_DISABLE_OPTIMAL_TO_NORMAL_PWR_TRANSITION \
	_IO(IAXXX_ODSP_IOCTL_MAGIC, 0x27)
#define ODSP_GET_PACKAGE_TYPE _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x28)
#define ODSP_GET_CHIP_STATUS _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x29)
#define ODSP_RESET_FW _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x2A)
#define ODSP_GET_DEV_ID  _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x2B)
#define ODSP_SET_ROUTE_STATUS _IO(IAXXX_ODSP_IOCTL_MAGIC, 0x2C)
#endif
