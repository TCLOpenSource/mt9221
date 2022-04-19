#
# Makefile for MStar device drivers.
#



# noe
obj-$(CONFIG_NOE) += eth/

ifeq ($(CONFIG_NOE_NAT_HW),y)
obj-$(CONFIG_NOE_NAT_HW)  += nat/hook/
obj-m         += nat/hw_nat/
endif


