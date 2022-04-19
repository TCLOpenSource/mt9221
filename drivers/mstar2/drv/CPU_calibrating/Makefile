#
# Makefile for MStar CPU_calibrating device drivers.
#

ifeq ($(MAKE_TYPE), MODULE_STANDALONE)
include $(TOP_DIR)/modules.mk
endif

CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))

# general options
EXTRA_CFLAGS += -Idrivers/mstar2/drv/CPU_calibrating
EXTRA_CFLAGS += -Idrivers/mstar2/include
EXTRA_CFLAGS += -Idrivers/mstar2/drv/dvfs/include
EXTRA_CFLAGS += -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/dvfs
EXTRA_CFLAGS += -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/cpu
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
EXTRA_CFLAGS += -Idrivers/opp
#else
EXTRA_CFLAGS += -Idrivers/base/power/opp
#endif
#EXTRA_CFLAGS += -Idrivers/mstar/dvfs/hal/$(CONFIG_MSTAR_CHIP_NAME)/include

# files
obj-$(CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)	+= mdrv_CPU_cluster_calibrating.o
obj-$(CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)     += mdrv_dvfs_debug.o

#export header files
