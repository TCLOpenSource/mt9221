#
# Makefile for MStar GPIO device drivers.
#

ifeq ($(MAKE_TYPE), MODULE_STANDALONE)
include $(TOP_DIR)/modules.mk
endif

CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))

# general options
#EXTRA_CFLAGS      +=  -Idrivers/mstar/common
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/sem
EXTRA_CFLAGS      +=  -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/sem

# specific options
EXTRA_CFLAGS      +=  -DTITANIA
EXTRA_CFLAGS      +=  -DMSOS_TYPE_LINUX

# files
obj-$(CONFIG_MSTAR_HW_SEM)	+= mdrv_sem.o
obj-$(CONFIG_MSTAR_HW_SEM)	+= mdrv_sem.o
obj-$(CONFIG_MSTAR_HW_SEM)	+= ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/sem/mhal_sem.o

#export header files
