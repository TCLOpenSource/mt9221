#//========================================================
#// Makefile for MStar DVFS Mechanism
#//========================================================

CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))
ifdef CONFIG_MSTAR_PROJECT_NAME
CONFIG_MSTAR_PROJECT_NAME := $(subst ",,$(CONFIG_MSTAR_PROJECT_NAME))
endif

EXTRA_CFLAGS += -Idrivers/mstar2/include
EXTRA_CFLAGS += -Iinclude/linux

obj-$(CONFIG_ARM_MTKTV_CPUFREQ)	:= mtktv-regulator.o

