/*
 * iaxxx-debug-intf.h - iaxxx debug Interface
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

#ifndef _IAXXX_DEBUG_INTF_H
#define _IAXXX_DEBUG_INTF_H

#define MAX_IAXXX_DEVICES	8

#define IAXXX_RAW_IOCTL_MAGIC	'R'

#define IAXXX_BUS_CONFIG	_IO(IAXXX_RAW_IOCTL_MAGIC, 0x011)
#define IAXXX_RESET		_IO(IAXXX_RAW_IOCTL_MAGIC, 0x012)

#endif
