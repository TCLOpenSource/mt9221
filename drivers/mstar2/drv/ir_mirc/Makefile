#
# Makefile for the kernel IrDA char device drivers.
#
CONFIG_MSTAR_CHIP_NAME := $(subst ",,$(CONFIG_MSTAR_CHIP_NAME))
ifdef CONFIG_MSTAR_PROJECT_NAME
    CONFIG_MSTAR_PROJECT_NAME := $(subst ",,$(CONFIG_MSTAR_PROJECT_NAME))
endif

CHIP    = titania

EXTRA_CFLAGS += -Idrivers/mstar2/hal/mbx/drv
EXTRA_CFLAGS += -Idrivers/mstar2/include
EXTRA_CFLAGS += -Idrivers/mstar2/drv/ir_mirc
EXTRA_CFLAGS += -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/ir
EXTRA_CFLAGS +=  -Iinclude/linux

obj-$(CONFIG_MSTAR_IR_REFACTOR) += mdrv-ir.o
mdrv-ir-objs	 += mstar_ir.o \
					ir_core.o \
					protocols/ir-nec-decoder.o \
					protocols/ir-rca-decoder.o \
					protocols/ir-p7051-decoder.o \
					protocols/ir-rc5-decoder.o \
					protocols/ir-rc6-decoder.o \
					protocols/ir-toshiba-decoder.o \
					protocols/ir-metz-decoder.o \
					protocols/ir-rcmm-decoder.o \
					protocols/ir-konka-decoder.o \
					protocols/ir-panasonic-decoder.o \
					protocols/ir-rc311-decoder.o \
					protocols/ir-beko-rc5-decoder.o \
					protocols/ir-sharp-decoder.o

ifeq ($(CONFIG_IR_KEYMAP_MSTAR_NEC), y)
mdrv-ir-objs +=	keymaps/keymap-mstar-tv.o
endif
ifeq ($(CONFIG_IR_KEYMAP_TCL_RCA), y)
mdrv-ir-objs +=	keymaps/keymap-tcl-rca-tv.o
endif
ifeq ($(CONFIG_IR_KEYMAP_TCL), y)
mdrv-ir-objs +=	keymaps/keymap-tcl-tv.o
endif
ifeq ($(CONFIG_IR_KEYMAP_CHANGHONG), y)
mdrv-ir-objs +=	keymaps/keymap-changhong-tv.o
endif
ifeq ($(CONFIG_IR_KEYMAP_HISENSE), y)
mdrv-ir-objs +=	keymaps/keymap-hisense-tv.o
endif
ifeq ($(CONFIG_IR_KEYMAP_HAIER), y)
mdrv-ir-objs +=	keymaps/keymap-haier-tv.o
endif
ifeq ($(CONFIG_IR_KEYMAP_KONKA), y)
mdrv-ir-objs +=	keymaps/keymap-konka-tv.o
endif
ifeq ($(CONFIG_IR_KEYMAP_SKYWORTH), y)
mdrv-ir-objs +=	keymaps/keymap-skyworth-tv.o
endif
ifeq ($(CONFIG_IR_KEYMAP_PANASONIC_7051), y)
mdrv-ir-objs +=	keymaps/keymap-p7051-tv.o
endif
ifeq ($(CONFIG_IR_KEYMAP_KATHREIN), y)
mdrv-ir-objs +=	keymaps/keymap-rc6-kathrein.o
endif
ifeq ($(CONFIG_IR_KEYMAP_RC5), y)
mdrv-ir-objs +=	keymaps/keymap-rc5-tv.o
endif
ifeq ($(CONFIG_IR_KEYMAP_METZ), y)
mdrv-ir-objs += keymaps/keymap-metz-rm18-tv.o
mdrv-ir-objs += keymaps/keymap-metz-rm19-tv.o
endif
ifeq ($(CONFIG_IR_KEYMAP_SAMPO), y)
mdrv-ir-objs += keymaps/keymap-rc311-sampo.o
endif
ifeq ($(CONFIG_IR_KEYMAP_GOME_NEC), y)
mdrv-ir-objs += keymaps/keymap-gome-tv.o
endif
ifeq ($(CONFIG_IR_KEYMAP_BEKO_RC5), y)
mdrv-ir-objs += keymaps/keymap-beko-rc5-tv.o
endif
obj-$(CONFIG_IR_DYNAMIC_CONFIG) += ir_dynamic_config/
