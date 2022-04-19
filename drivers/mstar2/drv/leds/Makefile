#
# Makefile for MStar localdimming device drivers.
CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))
ifdef CONFIG_MSTAR_PROJECT_NAME
    CONFIG_MSTAR_PROJECT_NAME := $(subst ",,$(CONFIG_MSTAR_PROJECT_NAME))
endif

ccflags-y      +=  -Idrivers/mstar2/include
ccflags-y      += -Idrivers/mstar2/drv/ldm/include
ccflags-y      +=  -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/ldm/
ccflags-y 	  += -Idrivers/mstar2/drv/mbx


ccflags-y      +=  -DRED_LION


# build moudle
ifeq ($(CONFIG_HAS_LED),m)
obj-m += lcd.o
endif

# build-in
ifeq ($(CONFIG_HAS_LED),y)
obj-$(CONFIG_HAS_LED)	+= led.o
endif

