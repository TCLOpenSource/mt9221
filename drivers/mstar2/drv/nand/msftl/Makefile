#
# Makefile for MStar NAND device drivers.
#
CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))

# general options

EXTRA_CFLAGS    += -Idrivers/mstar2/include
EXTRA_CFLAGS    += -Idrivers/mstar2/drv/nand/inc/api
EXTRA_CFLAGS    += -Idrivers/mstar2/drv/nand/inc/common
EXTRA_CFLAGS    += -Idrivers/mstar2/drv/nand/inc/config
EXTRA_CFLAGS    += -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/nand
EXTRA_CFLAGS    += -Idrivers/mstar2/drv/nand/msftl/inc/

# specific options

obj-y += ./ftl.o
obj-y += ./src/ftl_api.o
obj-y += ./src/ftl_dbg.o
obj-y += ./src/ftl_err.o
obj-y += ./src/ftl_hal.o
obj-y += ./src/ftl_hal_plane.o
obj-y += ./src/ftl_utl.o

obj-y += src/core/
