ifeq ($(CONFIG_ONEHAL_ARCH_SUPPORT),y)
CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))
ONEHAL_VERSION= $(subst ",,$(CONFIG_ONEHAL_VERSION))
ONEHAL_DIR = ver_$(ONEHAL_VERSION)
EXTRA_CFLAGS	+=  -Idrivers/mstar2/hal/$(subst ",,$(CONFIG_MSTAR_CHIP_NAME))/mzc/$(ONEHAL_DIR)
EXTRA_CFLAGS    +=  -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/mzc/
obj-y +=    ../../hal/$(subst ",,$(CONFIG_MSTAR_CHIP_NAME))/mzc/$(ONEHAL_DIR)/mhal_mzc_hal_diff.o
obj-y +=    ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/mzc/mhal_mzc_hal.o
obj-y +=    mdrv_mzc_drv.o
else
CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))
EXTRA_CFLAGS    +=  -Idrivers/mstar2/drv/mzc/
EXTRA_CFLAGS    +=  -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/mzc/
obj-y +=    ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/mzc/mhal_mzc_hal.o
obj-y +=    mdrv_mzc_drv.o
endif
EXTRA_CFLAGS    +=  -Iarch/$(SRCARCH)/
