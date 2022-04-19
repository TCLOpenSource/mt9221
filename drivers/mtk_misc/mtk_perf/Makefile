obj-$(CONFIG_HAVE_HW_BREAKPOINT) += perf_event.o
ifeq ("$(BUILD_WITH_AOSP_MK)","true")
CFLAGS_perf_event.o:=-DMT_PERF_TRACE_DIR=$(srctree)/$(src)
else
CFLAGS_perf_event.o:=-DMT_PERF_TRACE_DIR=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))
endif
$(info ++show DMT_PERF_TRACE_DIR info (pwd,src,srctree) = ($(shell pwd),$(src),$(srctree))++ )
