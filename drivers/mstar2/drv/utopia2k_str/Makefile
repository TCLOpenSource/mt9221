#
# Makefile for MStar Utopia2K STR device drivers.
#

# general options
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/utopia2k_str
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include/sys/common
EXTRA_CFLAGS      +=  -Idrivers/base/power
# specific options
EXTRA_CFLAGS      +=

# files
obj-$(CONFIG_MSTAR_UTOPIA2K_STR)    += mdrv_utopia2k_str_main.o
obj-$(CONFIG_MSTAR_UTOPIA2K_STR)    += mdrv_utopia2k_str.o
obj-$(CONFIG_MSTAR_UTOPIA2K_STR)    += mdrv_utopia2k_str_v2.o

mdrv-utopia2k-str-objs              += mdrv_utopia2k_str_main.o
mdrv-utopia2k-str-objs              += mdrv_utopia2k_str.o
mdrv-utopia2k-str-objs              += mdrv_utopia2k_str_v2.o

CFLAGS_mdrv_utopia2k_str_v2.o       := -DDEBUG
