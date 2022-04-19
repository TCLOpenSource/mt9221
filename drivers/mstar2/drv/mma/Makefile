#EXTRA_CFLAGS = -I mstar2/drv/optee_linuxdriver/include/

#obj-m += mma.o
#mma-objs += mma_interface.o

#KDIR = /home/fred.huang/iommu/kernel_3.10.40/tegra

#ifneq ($(KDIR), $(wildcard $(KDIR) ))
#        $(info KDIR not exist,you should appoint proper KDIR value in Makefile.)
#endif

#all:
#	make -C $(KDIR)/ SUBDIRS=$(PWD) modules
#clean:
#	make -C $(KDIR)/ SUBDIRS=$(PWD) clean
ccflags-y += -Idrivers/mstar2/drv/optee_linuxdriver/include/

EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Iarch/arm/mm
EXTRA_CFLAGS      +=  -Idrivers/mstar2/include/sys/common
EXTRA_CFLAGS      +=  -Ikernel/power

obj-y += mma.o
mma-objs += mma_core.o
mma-objs +=	mma_api.o
mma-objs +=	mma_ion.o
mma-objs +=	mma_debugfs.o
mma-objs +=	mma_of.o
mma-objs +=	mma_tee_inf.o
mma-objs +=     mma_iommu_debug_info.o

obj-m += mma_test.o


