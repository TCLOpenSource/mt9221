#
# Makefile for MStar CMA Pool device drivers.
#

CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))


# general options
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include/sys/common
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/cma_pool
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/miu
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/mma
# specific options
EXTRA_CFLAGS      +=  -Idrivers/staging/android/ion
EXTRA_CFLAGS      +=  -Imm

EXTRA_CFLAGS      += -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/miu

# files
ifeq ($(CONFIG_CMA),y)
obj-$(CONFIG_MSTAR_CMAPOOL)     += mdrv-cma_pool.o
mdrv-cma_pool-objs                       += mdrv_cma_pool.o
endif
