#  SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
# *****************************************************************************
#
#  This file is provided under a dual license.  When you use or
#  distribute this software, you may choose to be licensed under
#  version 2 of the GNU General Public License ("GPLv2 License")
#  or BSD License.
#
#  GPLv2 License
#
#  Copyright(C) 2019 MediaTek Inc.
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of version 2 of the GNU General Public License as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but
#  WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See http://www.gnu.org/licenses/gpl-2.0.html for more details.
#
#  BSD LICENSE
#
#  Copyright(C) 2019 MediaTek Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in
#     the documentation and/or other materials provided with the
#     distribution.
#   * Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# ****************************************************************************/

ifeq ($(BUILD_WITH_KERNEL), true)


KERNEL_SRC_TOP := $(ANDROID_BUILD_TOP)/vendor/mstar/kernel/linaro
DOT_CONFIG := $(KERNEL_SRC_TOP)/.config
TOOLCHAIN := /tools/arm/MStar/linaro_aarch64_linux-2014.09_r20170413

ifneq ($(filter arm arm64, $(TARGET_ARCH)),)
    MSTAR_CONFIG := $(KERNEL_SRC_TOP)/arch/$(TARGET_ARCH)/configs/mstar_config
    KERNEL_TARGET_OUT := $(KERNEL_SRC_TOP)/arch/$(TARGET_ARCH)/boot/Image
else
    $(error Not a supported TARGET_ARCH: $(TARGET_ARCH))
endif


# mainz
ifneq ($(filter bennet, $(TARGET_DEVICE)),)
    KERNEL_CONFIG := .config_mainz_SMP_arm64_andorid_emmc_nand_optee
    KERNEL_BIN_TARGET_OUT := $(KERNEL_SRC_TOP)/arch/$(TARGET_ARCH)/boot/dts/mainz_an.dtb

# maserati
else ifneq ($(filter plum, $(TARGET_DEVICE)),)
    KERNEL_CONFIG := .config_maserati_SMP_arm64_andorid_emmc_nand_optee
    KERNEL_BIN_TARGET_OUT := $(KERNEL_SRC_TOP)/arch/$(TARGET_ARCH)/boot/dts/maserati_an.dtb

# m7221
else ifneq ($(filter sugarcane, $(TARGET_DEVICE)),)
    KERNEL_CONFIG := .config_m7221_SMP_arm64_android_emmc_nand
    KERNEL_BIN_TARGET_OUT := $(KERNEL_SRC_TOP)/arch/$(TARGET_ARCH)/boot/dts/m7221_an.dtb

# maxim - china
else ifneq ($(filter synsepalum, $(TARGET_DEVICE)),)
    KERNEL_CONFIG := .config_maserati_SMP_arm64_andorid_emmc_nand_optee
    KERNEL_BIN_TARGET_OUT := $(KERNEL_SRC_TOP)/arch/$(TARGET_ARCH)/boot/dts/maxim_an.dtb

# maxim - sri
else ifneq ($(filter denali, $(TARGET_DEVICE)),)
    KERNEL_CONFIG := .config_maxim_SMP_arm64_andorid_emmc_nand_optee
    KERNEL_BIN_TARGET_OUT := $(KERNEL_SRC_TOP)/arch/$(TARGET_ARCH)/boot/dts/maxim_an.dtb

# k7u
else ifneq ($(filter lime, $(TARGET_DEVICE)),)
    KERNEL_CONFIG := .config_k7u_SMP_arm_andorid_emmc_nand_utopia2K
    KERNEL_BIN_TARGET_OUT := $(KERNEL_SRC_TOP)/arch/$(TARGET_ARCH)/boot/dts/k7u_an.dtb

# c2p
else ifneq ($(filter pineapple, $(TARGET_DEVICE)),)
    KERNEL_CONFIG := .config_c2p_SMP_arm64_android_emmc_nand_optee
    KERNEL_BIN_TARGET_OUT := $(KERNEL_SRC_TOP)/arch/$(TARGET_ARCH)/boot/dts/c2p_an.dtb

# k6
else ifneq ($(filter waxapple, $(TARGET_DEVICE)),)
    KERNEL_CONFIG := .config_k6_SMP_arm64_android_emmc_nand_cma_utopia2K
    KERNEL_BIN_TARGET_OUT := $(KERNEL_SRC_TOP)/arch/$(TARGET_ARCH)/boot/dts/k6_an.dtb

else
    $(error Not a supported TARGET_DEVICE: $(TARGET_DEVICE))

endif


KERNEL_BZIMAGE := $(PRODUCT_OUT)/kernel
KERNEL_DTB := $(PRODUCT_OUT)/images/prebuilts/dtb.bin

$(DOT_CONFIG): $(KERNEL_SRC_TOP)/$(KERNEL_CONFIG)
	cd $(KERNEL_SRC_TOP); \
	cp $(KERNEL_CONFIG) .config; cp $(KERNEL_CONFIG) $(MSTAR_CONFIG);

$(KERNEL_BZIMAGE): $(DOT_CONFIG)
	cd $(KERNEL_SRC_TOP); \
	export PATH=$(TOOLCHAIN)/bin:$(PATH); sh genlink.sh; \
	make defconfig KBUILD_DEFCONFIG=mstar_config; \
	make clean; make -j32;
	@cp -f $(KERNEL_TARGET_OUT) $@
	@cp -f $(KERNEL_BIN_TARGET_OUT) $(KERNEL_DTB)

kernel_clean: $(DOT_CONFIG)
	cd $(KERNEL_SRC_TOP); \
	export PATH=$(TOOLCHAIN)/bin:$(PATH); sh genlink.sh; \
	make defconfig KBUILD_DEFCONFIG=mstar_config; \
	make clean
	@rm -f $(KERNEL_BZIMAGE)
	@rm -f $(KERNEL_DTB)

$(PRODUCT_OUT)/boot.img: $(KERNEL_BZIMAGE)


endif #ifeq ($(BUILD_WITH_KERNEL), true)

# ==============================================================================
#include $(call all-subdir-makefiles)
