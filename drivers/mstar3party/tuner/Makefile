#
# Makefile for MStar TUNER device drivers.
#

ifeq ($(MAKE_TYPE), MODULE_STANDALONE)
include $(TOP_DIR)/modules.mk
endif

CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))

# general options
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/tuner
EXTRA_CFLAGS      +=  -Idrivers/mstar3party/tuner
EXTRA_CFLAGS      +=  -Idrivers/mstar3party/BypassEWBSIIC

# specific options
EXTRA_CFLAGS      +=  -DTITANIA
EXTRA_CFLAGS      +=  -DMSOS_TYPE_LINUX

# files Tuner Types

ifeq ($(CONFIG_MSTAR_TUNER_Si2151), y)
obj-$(CONFIG_MSTAR_TUNER)	+= mdrv-tuner-si_2151.o
mdrv-tuner-si_2151-objs				+= tuner_Si2151/mdrv_tuner_Si2151.o mdrv_tuner_io.o
# tuner code
mdrv-tuner-si_2151-objs				+= tuner_Si2151/Si2151/si2151_i2c_api.o
mdrv-tuner-si_2151-objs				+= tuner_Si2151/Si2151/Si2151_L1_API.o
mdrv-tuner-si_2151-objs				+= tuner_Si2151/Si2151/Si2151_L1_Commands.o
mdrv-tuner-si_2151-objs				+= tuner_Si2151/Si2151/Si2151_L1_Properties.o
mdrv-tuner-si_2151-objs				+= tuner_Si2151/Si2151/Si2151_L2_API.o
mdrv-tuner-si_2151-objs				+= tuner_Si2151/Si2151/Si2151_L3_Test.o
mdrv-tuner-si_2151-objs				+= tuner_Si2151/Si2151/Si2151_Properties_strings.o
mdrv-tuner-si_2151-objs				+= tuner_Si2151/Si2151/Si2151_User_Properties.o
mdrv-tuner-si_2151-objs				+= tuner_Si2151/Si2151/Silabs_L0_Connection.o
mdrv-tuner-si_2151-objs				+=../BypassEWBSIIC/BypassEWBSIIC.o
endif

ifeq ($(CONFIG_MSTAR_TUNER_MXL661), y)
obj-$(CONFIG_MSTAR_TUNER)	+= mdrv-tuner-MXL661.o
mdrv-tuner-MXL661-objs				+= tuner_MXL661/mdrv_tuner_MXL661.o mdrv_tuner_io.o
# tuner code
mdrv-tuner-MXL661-objs				+= tuner_MXL661/MxL661/MxL661_Main.o
mdrv-tuner-MXL661-objs				+= tuner_MXL661/MxL661/MxL661_OEM_Drv.o
mdrv-tuner-MXL661-objs				+= tuner_MXL661/MxL661/MxL661_TunerApi.o
mdrv-tuner-MXL661-objs				+= tuner_MXL661/MxL661/MxL661_TunerCfg.o
mdrv-tuner-MXL661-objs				+= tuner_MXL661/MxL661/MxL661_TunerSpurTable.o
mdrv-tuner-MXL661-objs		          	+=../BypassEWBSIIC/BypassEWBSIIC.o
endif

ifeq ($(CONFIG_MSTAR_TUNER_AV2018), y)
obj-$(CONFIG_MSTAR_TUNER)	+= mdrv-tuner-AV2018.o
mdrv-tuner-AV2018-objs				+= tuner_AV2018/mdrv_tuner_AV2018.o mdrv_tuner_io.o
endif

ifeq ($(CONFIG_MSTAR_TUNER_AV2017), y)
obj-$(CONFIG_MSTAR_TUNER)	+= mdrv-tuner-AV2017.o
mdrv-tuner-AV2017-objs				+= tuner_AV2017/mdrv_tuner_AV2017.o mdrv_tuner_io.o
endif

ifeq ($(CONFIG_MSTAR_TUNER_AV2012), y)
obj-$(CONFIG_MSTAR_TUNER)	+= mdrv-tuner-AV2012.o
mdrv-tuner-AV2012-objs				+= tuner_AV2012/mdrv_tuner_AV2012.o mdrv_tuner_io.o
endif

ifeq ($(CONFIG_MSTAR_TUNER_R842), y)
obj-$(CONFIG_MSTAR_TUNER)	+= mdrv-tuner-R842.o
mdrv-tuner-R842-objs				+= tuner_R842/mdrv_tuner_R842.o mdrv_tuner_io.o
# tuner code
mdrv-tuner-R842-objs				+= tuner_R842/R842/R842.o
mdrv-tuner-R842-objs                            +=../BypassEWBSIIC/BypassEWBSIIC.o
endif


ifeq ($(CONFIG_MSTAR_TUNER_RT710), y)
obj-$(CONFIG_MSTAR_TUNER)	+= mdrv-tuner-RT710_720.o
mdrv-tuner-RT710_720-objs				+=tuner_RT710/mdrv_tuner_RT710.o mdrv_tuner_io.o
endif

#export header files
