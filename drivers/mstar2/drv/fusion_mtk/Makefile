EXTRA_CFLAGS      +=  -Idrivers/mstar2/include
EXTRA_CFLAGS      +=  -Idrivers/mstar2/drv/fusion_mtk

obj-$(CONFIG_MTK_FUSION_MTK) += fusion_mtk.o

fusion_mtk-y := fusioncore_impl.o call.o debug.o entries.o fifo.o fusiondev.o fusionee.o list.o property.o reactor.o ref.o skirmish.o shmpool.o 
$(addprefix $(obj)/, $(fusion_mtk-y)):include/linux/fusion.h
#$(obj)/fusioncore_impl.o call.o debug.o entries.o fifo.o fusiondev.o fusionee.o list.o property.o reactor.o ref.o skirmish.o shmpool.o: include/linux/fusion.h
include/linux/fusion.h: FORCE
	@cp drivers/mstar2/drv/fusion_mtk/include/fusion.h include/linux/fusion.h -au

