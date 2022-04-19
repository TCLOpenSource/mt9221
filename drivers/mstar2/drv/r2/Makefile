#
# Makefile for MStar R2 device drivers.
#
CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))
ifdef CONFIG_MSTAR_PROJECT_NAME
    CONFIG_MSTAR_PROJECT_NAME := $(subst ",,$(CONFIG_MSTAR_PROJECT_NAME))
endif

# general options
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/r2
EXTRA_CFLAGS      +=  -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/r2
EXTRA_CFLAGS      +=  -Iinclude/linux

obj-y   := mdrv_r2.o
obj-y   += mdrv_r2_io.o
obj-y    += ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/r2/mhal_r2.o
