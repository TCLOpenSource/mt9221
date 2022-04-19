#undef TRACE_SYSTEM
#define TRACE_SYSTEM mtk_perf_trace

#if !defined(_TRACE_TASK_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_TASK_H
#include <linux/tracepoint.h>
#define MAX_REG_NAME 4

TRACE_EVENT(mtk_perf_mem_val,

    TP_PROTO(unsigned long virt_addr,unsigned long val),

    TP_ARGS(virt_addr,val),

    TP_STRUCT__entry(
        __field(unsigned long,addr)
        __field(unsigned long,val)
    ),

    TP_fast_assign(
        __entry->addr = virt_addr;
        __entry->val = val;
    ),

    TP_printk("addr=%lx,val=%lx",
        __entry->addr,__entry->val)
);
TRACE_EVENT(mtk_perf_reg_val,

    TP_PROTO(char *regname,unsigned long val),

    TP_ARGS(regname,val),

    TP_STRUCT__entry(
        __array(char,regname,MAX_REG_NAME)
        __field(unsigned long,val)
    ),

    TP_fast_assign(
        memcpy(__entry->regname,regname,MAX_REG_NAME);
        __entry->val = val;
    ),

    TP_printk("regname=%s,val=%lx(%pS)",
        __entry->regname,__entry->val,(void *)(__entry->val))
);
#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
