/*
 * iaxxx-register-defs-pkg-mgmt.h
 *
 * Copyright (c) 2018 Knowles, inc.
 *
 * This file is to select the header file as per the platform
 *
 */
#ifndef __IAXXX_PACKAGE_REGISTER_H__
#define __IAXXX_PACKAGE_REGISTER_H__

#include "ia8x01/iaxxx-register-defs-pkg-mgmt.h"
#include "ia8x01/iaxxx-register-defs-pkg-group.h"

#define IAXXX_PKG_GRP_PACKAGE_INFO_REG(I) \
	(IAXXX_PKG_GRP_PKG_INFO_ADDR + \
	(4 * (I) * IAXXX_PKG_GRP_REG_NUM))

#define IAXXX_PKG_GRP_PACKAGE_VER_STR_REG(I) \
	(IAXXX_PKG_GRP_PKG_VER_STR_ADDR + \
	(4 * (I) * IAXXX_PKG_GRP_REG_NUM))

#endif /* __IAXXX_PACKAGE_REGISTER_H__ */
