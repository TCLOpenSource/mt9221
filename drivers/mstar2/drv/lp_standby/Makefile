#
# Makefile for MStar auto str device drivers.
#

CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))
ifdef CONFIG_MSTAR_PROJECT_NAME
    CONFIG_MSTAR_PROJECT_NAME := $(subst ",,$(CONFIG_MSTAR_PROJECT_NAME))
endif

# general options
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include/sys/common
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/lp_standby
EXTRA_CFLAGS      +=  -Iinclude/linux
EXTRA_CFLAGS      +=  -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/lp_standby
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/gpio
# specific options
EXTRA_CFLAGS      +=


# files
obj-$(CONFIG_MSTAR_LP_STANDBY)              := mdrv_lp_standby.o
obj-$(CONFIG_MSTAR_LP_STANDBY)              += ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/lp_standby/hal_lp_standby.o
