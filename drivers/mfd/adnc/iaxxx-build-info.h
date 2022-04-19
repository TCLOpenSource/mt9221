/*
 * iaxxx-build-info.h -- iaxxx Build data
 *
 * Copyright (c) 2018 Knowles Corporation
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

#ifndef _IAXXX_BUILD_INFO_H
#define _IAXXX_BUILD_INFO_H

/* check HSW version String */
#if defined(IAXXX_HSW_VER_STRING)

#define HOST_SOFTWARE_VERSION_STR IAXXX_HSW_VER_STRING

#else

#if defined(CONFIG_MFD_IA8X01_AIS)

#define HOST_SOFTWARE_VERSION_STR "CHL_MOB_SYS2_AIS_VoiceQ_R9.0.2"

#elif defined(CONFIG_MFD_IA8X01_AZ)

#define HOST_SOFTWARE_VERSION_STR "IA8201_MOB_AZA_SYS_REL_2.0.0"

#else

#define HOST_SOFTWARE_VERSION_STR "CHL_MOB_SYS2_KN_VoiceQ_R8.0.0"

#endif
#endif


/* check FW version String */
#ifdef IAXXX_FW_VER_STRING

#define FW_VERSION_IN_HOST_STR IAXXX_FW_VER_STRING

#else

#if defined(CONFIG_MFD_IA8X01_AIS)

#define FW_VERSION_IN_HOST_STR "ROME_IA8201_MOB_SYS2_8.0.2_AIS_VoiceUI"

#elif defined(CONFIG_MFD_IA8X01_AZ)

#define FW_VERSION_IN_HOST_STR "ROME_IA8201_REL_1_0_2_Kn_AZA:1000200"

#else

#define FW_VERSION_IN_HOST_STR "ROME_IA8201_REL_11_0_0"

#endif
#endif

#endif
