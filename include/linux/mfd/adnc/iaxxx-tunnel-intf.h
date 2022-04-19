/*
 * iaxxx-tunnel-intf.h - iaxxx Tunnel Service Interface
 *
 * Copyright 2017 Knowles Corporation
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

#ifndef _IAXXX_TUNNEL_INTF_H
#define _IAXXX_TUNNEL_INTF_H

#define MAX_IAXXX_DEVICES	8

struct tunlMsg {
	uint32_t tunlEP;
	uint32_t tunlSrc;
	uint32_t tunlMode;
	uint32_t tunlEncode;
} __attribute__((__packed__));

struct iaxxx_tunnel_header {
	uint8_t magic[4];
	uint16_t tunnel_id;
	uint8_t crc[6];
	uint64_t ts;
	uint32_t seq_no;
	uint16_t size;
	uint8_t encoding;
	uint8_t sample_rate;
	char buf[0];
} __attribute__((__packed__));


struct iaxxx_tnl_evt_info {
	uint16_t src_id;
	uint16_t event_id;
	uint16_t dst_id;
	uint32_t dst_opaque;
	uint32_t evt_threshold;
} __attribute__((__packed__));


enum iaxxx_tunnel_mode_type {
	TNL_MODE_SYNC = 0,
	TNL_MODE_ASYNC,
};

enum iaxxx_tunnel_encode_type {
	TNL_ENC_OPAQUE = 0,
	TNL_ENC_AFLOAT = 1,
	TNL_ENC_Q15 = 0xF,
};

enum iaxxx_tunnel_dir_type {
	TNL_HOST_DEVICE_RX = 0,
	TNL_DEVICE_HOST_TX,
};

#define IAXXX_TNL_IOCTL_MAGIC		'K'

#define TUNNEL_SETUP			_IOWR(IAXXX_TNL_IOCTL_MAGIC, 0x011, \
					      struct tunlMsg)
#define TUNNEL_TERMINATE		_IOWR(IAXXX_TNL_IOCTL_MAGIC, 0x012, \
					      struct tunlMsg)
#define TUNNEL_SUBSCRIBE_META		_IO(IAXXX_TNL_IOCTL_MAGIC, 0x013)
#define TUNNEL_SUBSCRIBE_ALL		_IO(IAXXX_TNL_IOCTL_MAGIC, 0x014)
#define TUNNEL_SUBSCRIBE_CVQ		_IO(IAXXX_TNL_IOCTL_MAGIC, 0x015)
#define TUNNEL_UNSUBSCRIBE_META		_IO(IAXXX_TNL_IOCTL_MAGIC, 0x016)
#define TUNNEL_UNSUBSCRIBE_ALL		_IO(IAXXX_TNL_IOCTL_MAGIC, 0x017)
#define TUNNEL_UNSUBSCRIBE_CVQ		_IO(IAXXX_TNL_IOCTL_MAGIC, 0x018)
#define TUNNEL_SUBSCRIBE_META_DOA	_IO(IAXXX_TNL_IOCTL_MAGIC, 0x019)
#define TUNNEL_SUBSCRIBE_META_VQ	_IO(IAXXX_TNL_IOCTL_MAGIC, 0x01a)
#define TUNNEL_UNSUBSCRIBE_META_DOA	_IO(IAXXX_TNL_IOCTL_MAGIC, 0x01b)
#define TUNNEL_UNSUBSCRIBE_META_VQ	_IO(IAXXX_TNL_IOCTL_MAGIC, 0x01c)
#define TUNNEL_EVENT_SUBSCRIBE		_IOWR(IAXXX_TNL_IOCTL_MAGIC, 0x01d, \
					struct iaxxx_tnl_evt_info)
#define TUNNEL_EVENT_UNSUBSCRIBE	_IOWR(IAXXX_TNL_IOCTL_MAGIC, 0x01e, \
					struct iaxxx_tnl_evt_info)
#define TUNNEL_RESUME			_IO(IAXXX_TNL_IOCTL_MAGIC, 0x01f)
#define TUNNEL_PAUSE			_IO(IAXXX_TNL_IOCTL_MAGIC, 0x020)
#define TUNNEL_GET_OUTPUT_BUF_SIZE	_IO(IAXXX_TNL_IOCTL_MAGIC, 0x021)
#define TUNNEL_SET_OUTPUT_BUF_SIZE	_IO(IAXXX_TNL_IOCTL_MAGIC, 0x022)

#endif
