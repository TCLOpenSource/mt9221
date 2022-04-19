#
# Makefile for MStar IIC device drivers.
#

ifeq ($(MAKE_TYPE), MODULE_STANDALONE)
include $(TOP_DIR)/modules.mk
endif

CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))

# general options
# EXTRA_CFLAGS      +=  -Idrivers/mstar2/common
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/rtc
EXTRA_CFLAGS      +=  -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/rtc

# specific options
EXTRA_CFLAGS      +=  -DMSOS_TYPE_LINUX

# files
obj-$(CONFIG_MSTAR_RTC)	+= mdrv-rtc.o
mdrv-rtc-objs			+= ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/rtc/mhal_rtc.o
mdrv-rtc-objs			+= mdrv_rtc.o
mdrv-rtc-objs           += mstar-rtc.o


# export header files
EXPORT_H_FILES    += mdrv_rtc_io.h
