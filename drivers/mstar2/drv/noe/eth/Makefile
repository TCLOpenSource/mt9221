#
# Makefile for MStar NOE device drivers.
#


CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))
ifdef CONFIG_MSTAR_PROJECT_NAME
    CONFIG_MSTAR_PROJECT_NAME := $(subst ",,$(CONFIG_MSTAR_PROJECT_NAME))
endif

IS_LRO_SUPPORT = $(shell if [ $(VERSION) -lt 4 ] || ( [ $(VERSION) -eq 4 ] && [ $(PATCHLEVEL) -lt 7 ] ); then echo y; fi)

EXTRA_CFLAGS      += -DMDRV_NOE_DBG_SKB_DUMP
# general options
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include/sys/common
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/noe
EXTRA_CFLAGS      +=  -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/noe
EXTRA_CFLAGS      +=  -Iinclude/linux
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/noe/eth

obj-$(CONFIG_NOE) += mdrv-noe.o
mdrv-noe-objs :=  mdrv_noe_mac.o mdrv_noe_phy.o mdrv_noe_log.o mdrv_noe_proc.o mdrv_noe.o mdrv_noe_pdma.o mdrv_noe_utils.o


ifeq ($(CONFIG_NOE_QDMA),y)
mdrv-noe-objs += mdrv_noe_qdma.o
endif

ifeq ($(CONFIG_NOE_ETHTOOL),y)
mdrv-noe-objs += mdrv_noe_ethtool.o
endif

ifeq ($(CONFIG_NOE_HW_LRO),y)
mdrv-noe-objs += mdrv_noe_lro.o
mdrv-noe-objs += mdrv_noe_lro_proc.o
endif


ifeq ($(CONFIG_NOE_NAT_HW),y)
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/noe/nat/hw_nat
endif

# specific options
EXTRA_CFLAGS      +=  -DMSOS_TYPE_LINUX

ccflags-y += -Werror

mdrv-noe-objs    += ../../../hal/$(CONFIG_MSTAR_CHIP_NAME)/noe/mhal_internal_gphy.o
mdrv-noe-objs    += ../../../hal/$(CONFIG_MSTAR_CHIP_NAME)/noe/mhal_noe.o

