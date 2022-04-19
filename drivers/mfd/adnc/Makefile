#
# Makefile for Audience MFD & related Modules
#

# Check if HSW version is provided in the Kernel Defconfig
ifneq ($(CONFIG_IAXXX_HSW_VER_STR),"")
KBUILD_CFLAGS += -DIAXXX_HSW_VER_STRING=CONFIG_IAXXX_HSW_VER_STR

else

# move to the path of this file
LOCAL_PATH := $(dir $(realpath $(lastword $(MAKEFILE_LIST))))


#  Use mstar gpio functions to adapt for MT9615
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/gpio
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/gpio
EXTRA_CFLAGS      +=  -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/gpio

# pick the version string from the latest Tag applied on this branch
GIT_VERSION = $(shell cd $(LOCAL_PATH) && \
	git describe --abbrev=4 --dirty --always --tags || \
	echo "unknown")

KBUILD_CFLAGS += -DIAXXX_HSW_VER_STRING='"$(GIT_VERSION)"'

endif

# Check if FW version is provided in the Kernel Defconfig
ifneq ($(CONFIG_IAXXX_FW_VER_STR),"")
KBUILD_CFLAGS += -DIAXXX_FW_VER_STRING=CONFIG_IAXXX_FW_VER_STR
endif

iaxxx-mfd-core-objs := iaxxx-core.o iaxxx-boot.o iaxxx-srb.o \
				iaxxx-plugin.o iaxxx-event.o iaxxx-sysfs.o \
				iaxxx-regmap.o iaxxx-evnt-mgr.o iaxxx-dbgfs.o \
				iaxxx-debug.o iaxxx-cdev.o iaxxx-pwr-mgmt.o \
				iaxxx-script-mgr.o

ifeq ($(CONFIG_MFD_IAXXX_I2C), y)
iaxxx-mfd-core-objs	+= iaxxx-i2c.o
endif

ifeq ($(CONFIG_MFD_IAXXX_SPI), y)
iaxxx-mfd-core-objs	+= iaxxx-spi.o
endif

ifeq ($(CONFIG_MFD_IAXXX_UART), y)
iaxxx-mfd-core-objs	+= iaxxx-uart.o
endif

ifeq ($(CONFIG_MFD_IAXXX_SOUNDWIRE), y)
iaxxx-mfd-core-objs	+= iaxxx-swr.o
endif

iaxxx-tunnel-driver-objs  := iaxxx-tunnel.o iaxxx-tunnel-dev.o

ifeq ($(CONFIG_MFD_IAXXX), m)

obj-$(CONFIG_MFD_IAXXX)	:= iaxxx-mfd-core.o
obj-$(CONFIG_MFD_IAXXX)	+= iaxxx-tunnel-driver.o

else
obj-$(CONFIG_MFD_IAXXX)	+= iaxxx-mfd-core.o
obj-$(CONFIG_MFD_IAXXX)	+= iaxxx-tunnel-driver.o
endif
