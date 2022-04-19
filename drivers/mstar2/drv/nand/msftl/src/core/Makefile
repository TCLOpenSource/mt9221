#
# Makefile for MStar NAND device drivers.
#
CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))
CONFIG_CROSS_COMPILE := $(subst ",,$(CONFIG_CROSS_COMPILE))

# general options

EXTRA_CFLAGS    += -Idrivers/mstar2/include
EXTRA_CFLAGS    += -Idrivers/mstar2/drv/nand/inc/api
EXTRA_CFLAGS    += -Idrivers/mstar2/drv/nand/inc/common
EXTRA_CFLAGS    += -Idrivers/mstar2/drv/nand/inc/config
EXTRA_CFLAGS    += -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/nand
EXTRA_CFLAGS    += -Idrivers/mstar2/drv/nand/msftl/inc/

# specific options

#core
obj-y += $(CONFIG_CROSS_COMPILE)mstarftlcore.z
