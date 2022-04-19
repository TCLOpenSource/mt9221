#
# Makefile for MStar DISH device drivers.
#

ifeq ($(MAKE_TYPE), MODULE_STANDALONE)
include $(TOP_DIR)/modules.mk
endif

CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))

# general options
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/dish
EXTRA_CFLAGS      +=  -Idrivers/mstar3party/dish

# specific options
EXTRA_CFLAGS      +=  -DTITANIA
EXTRA_CFLAGS      +=  -DMSOS_TYPE_LINUX

# files Tuner Types

ifeq ($(CONFIG_MSTAR_DISH_RT5047), y)
obj-$(CONFIG_MSTAR_DISH)	        += mdrv-dish-RT5047.o
mdrv-dish-RT5047-objs				+= dish_RT5047/mdrv_dish_RT5047.o mdrv_dish_io.o
endif

#export header files
