#
# Makefile for Audience audio & related Modules
#
iaxxx-codec-driver-objs	:= iaxxx-codec.o
iaxxx-alsa-tunnel-driver-objs	:= iaxxx-alsa-tnl.o
dummy-machine-iaxxx-objs := dummy-machine.o

ifeq ($(CONFIG_MFD_IAXXX), m)

ifeq ($(CONFIG_IAXXX_SND_SOC_ADAU1772), m)
iaxxx-adau1772-driver-objs	+= adau1772.o
endif

obj-$(CONFIG_MFD_IAXXX)		+= iaxxx-codec-driver.o
obj-$(CONFIG_MFD_IAXXX)		+= iaxxx-alsa-tunnel-driver.o
obj-$(CONFIG_MFD_IAXXX)		+= iaxxx-adau1772-driver.o

else

ifeq ($(CONFIG_IAXXX_SND_SOC_ADAU1772), y)
iaxxx-codec-driver-objs		+= adau1772.o
endif

obj-$(CONFIG_MFD_IAXXX)		+= iaxxx-codec-driver.o
obj-$(CONFIG_MFD_IAXXX)		+= iaxxx-alsa-tunnel-driver.o
endif

obj-$(CONFIG_MFD_IAXXX)		+= dummy-machine-iaxxx.o
