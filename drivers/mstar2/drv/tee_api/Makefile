#
# Makefile for MStar cpu power manager.
#

EXTRA_CFLAGS += -Idrivers/mstar2/include

# general options
EXTRA_CFLAGS += -Idrivers/mstar2/include/sys/common
EXTRA_CFLAGS += -Idrivers/mstar2/include
EXTRA_CFLAGS += -Idrivers/mstar2/drv/cputee
EXTRA_CFLAGS += -Iinclude/linux
EXTRA_CFLAGS += -Idrivers/mstar2/drv/dvfs/include
EXTRA_CFLAGS += -Idrivers/mstar2/drv/optee_linuxdriver/core/
EXTRA_CFLAGS += -Idrivers/mstar2/drv/optee_linuxdriver/armtz/mstar_sec_arm_mbx/
EXTRA_CFLAGS += -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/optee_linuxdriver/

ifeq ($(CONFIG_ARM),y)
# "smc" assembly intruction requires dedicated "armv7 secure extension"
secext := $(call as-instr,.arch_extension sec,+sec)
AFLAGS_tee_smc-arm.o := -Wa,-march=armv7-a$(secext)
obj-y += \
	smccc-call_32.o
endif

ifeq ($(CONFIG_ARM64),y)
obj-y += \
	smccc-call_64.o
endif

obj-$(CONFIG_MP_PLATFORM_ARM)	+= mdrv_tee_general.o
obj-$(CONFIG_MP_PLATFORM_ARM)	+= mdrv_tee_cpu_state.o
