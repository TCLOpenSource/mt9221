EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/fusion

obj-$(CONFIG_MTK_FUSION) += fusion.o

fusion-y := fusioncore_impl.o call.o debug.o entries.o fifo.o fusiondev.o fusionee.o list.o property.o reactor.o ref.o skirmish.o shmpool.o 
$(obj)/call.o: include/linux/fusion.h
include/linux/fusion.h: FORCE
	@cp drivers/mtk_misc/mtk-fusion/include/fusion.h include/linux/fusion.h -f

