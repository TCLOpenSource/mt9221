#
# Makefile for MStar System device drivers.
#
CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))
ifdef CONFIG_MSTAR_PROJECT_NAME
    CONFIG_MSTAR_PROJECT_NAME := $(subst ",,$(CONFIG_MSTAR_PROJECT_NAME))
endif


# general options
EXTRA_CFLAGS      += -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/dvfs
EXTRA_CFLAGS      += -Idrivers/mstar2/drv/dvfs/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/system
EXTRA_CFLAGS      +=  -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/system
ifdef CONFIG_MSTAR_SPI_FLASH
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/flash
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include/sys/common
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include/sys/drv
endif
# specific options
EXTRA_CFLAGS      +=

# files
obj-$(CONFIG_MSTAR_SYSTEM) += mdrv-system.o
obj-$(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING) += mdrv_cpu_dvfs_scaling_list.o
obj-$(CONFIG_MP_BENCHMARK_ACCEL87) += mdrv_accel87.o
obj-$(CONFIG_MP_BENCHMARK_LAUNCH_BOOST) += mdrv_cpu_dvfs_scaling_list.o

mdrv-system-y := mdrv_system.o mdrv_system_io.o \
		../../hal/$(CONFIG_MSTAR_CHIP_NAME)/system/mhal_system.o
mdrv-system-$(CONFIG_AMZ_MISC) += amazon_util.o
