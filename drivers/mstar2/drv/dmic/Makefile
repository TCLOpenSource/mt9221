#
# Makefile for MStar ALSA audio device drivers.
#

ifeq ($(MAKE_TYPE), MODULE_STANDALONE)
include $(TOP_DIR)/modules.mk
endif

CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))

# general options
EXTRA_CFLAGS += -Idrivers/mstar2/drv/dmic
EXTRA_CFLAGS += -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/dmic

# specific options
EXTRA_CFLAGS += -DTITANIA
EXTRA_CFLAGS += -DMSOS_TYPE_LINUX

# files
obj-$(CONFIG_MSTAR_DMIC_DRIVER_DRV) += mdrv-dmic.o
mdrv-dmic-objs += mdrv_dmic.o
obj-$(CONFIG_MSTAR_DMIC_DRIVER_HAL) += mhal-dmic.o
mhal-dmic-objs += ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/dmic/mhal_dmic.o
