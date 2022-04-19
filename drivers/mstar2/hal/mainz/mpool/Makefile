#
# Makefile for the linux kernel.
#
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
ifdef CONFIG_ARM64
EXTRA_CFLAGS	  +=  -I../cpu/arm64
else
EXTRA_CFLAGS	  +=  -I../cpu/arm
endif
obj-y             += mhal_mpool.o
