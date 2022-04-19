#
# Makefile for MStar Voice drivers.
#

CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))

#EXTRA_CFLAGS      +=  -Idrivers/mstar/include
#EXTRA_CFLAGS      +=  -Idrivers/mstar/include/$(CONFIG_MSTAR_CHIP_NAME)
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/voice/alsa
EXTRA_CFLAGS      +=  -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/inc
EXTRA_CFLAGS      +=  -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/hal/$(CONFIG_MSTAR_CHIP_NAME)/inc

# iNfinity3e Platform Support
obj-$(CONFIG_MSTAR_VOICE_DRIVER_DRV) += alsa/voc_pcm.o alsa/voc_soc.o alsa/voc_dai.o

obj-$(CONFIG_MSTAR_VOICE_DRIVER_HAL) += ms_drv.o

# ms_drv-objs +=  ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/drvVoc.o ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/drvAec.o ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/halCHIPTOP.o ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/halCPUINT.o ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/halBDMA.o ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/drvBDMA.o

ms_drv-objs +=  ../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/hal/$(CONFIG_MSTAR_CHIP_NAME)/src/halMBX.o \
../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/hal/$(CONFIG_MSTAR_CHIP_NAME)/src/halCHIPTOP.o \
../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/hal/$(CONFIG_MSTAR_CHIP_NAME)/src/halCPUINT.o  \
../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/hal/$(CONFIG_MSTAR_CHIP_NAME)/src/halBDMA.o  \
../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/drv/drvMBX.o \
../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/drv/drvVoc.o \
../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/drv/drvBDMA.o \
../../hal/$(CONFIG_MSTAR_CHIP_NAME)/voice/sys/mosWrapper.o