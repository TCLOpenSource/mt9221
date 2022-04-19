#
# Makefile for MStar UFS device drivers.
#
CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))

# general options

EXTRA_CFLAGS    += -Idrivers/scsi/ufs
EXTRA_CFLAGS    += -Idrivers/mstar2/include
EXTRA_CFLAGS    += -Idrivers/mstar2/drv/ufs
EXTRA_CFLAGS    += -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/ufs

# specific options

obj-y           += ufs-mstar.o
obj-y           += ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/ufs/ufs-mstar-pltfrm.o

