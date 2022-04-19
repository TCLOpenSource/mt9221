#
# Makefile for MStar Util device drivers.
#

# general options
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/util
# specific options
EXTRA_CFLAGS      +=

# files
obj-$(CONFIG_MSTAR_UTIL)	+= mdrv-util.o
mdrv-util-objs		+= mdrv_util.o
mdrv-util-objs		+= mdrv_util_io.o
mdrv-util-objs		+= mdrv_tee_ramlog.o
