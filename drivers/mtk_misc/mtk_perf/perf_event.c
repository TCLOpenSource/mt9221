/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2019 MediaTek Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#ifdef CONFIG_STACKTRACE 
#define pr_fmt(fmt) "mt_perf: " fmt
//#define DEBUG 1
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/perf_event.h>
#include <linux/hw_breakpoint.h>
#include <linux/sched.h>
#include <linux/cpu_pm.h>
#include <linux/smp.h>
#include <linux/signal.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/stacktrace.h>
#ifdef CONFIG_KPROBES
#include <linux/kprobes.h>
#endif
#include <linux/version.h>
//#include <asm/debug-monitors.h>
//#include <asm/system_misc.h>
#ifdef CONFIG_ARM64
#include <asm/insn.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
#include <linux/cpuhotplug.h>
#endif
#pragma push_macro("linux")
#undef linux
#define CREATE_TRACE_POINTS
#define TRACE_INCLUDE_PATH MT_PERF_TRACE_DIR
#include "mtk_perf_trace.h"
#pragma pop_macro("linux")

#if LINUX_VERSION_CODE >=KERNEL_VERSION(4, 19, 0) && !defined(CONFIG_ARM64)
#include <asm/hw_breakpoint.h>
#define FIX_ARM_BREAKPOINT_STEP_CTRL_ISSUE
#endif

#define attr_name(_attr) (_attr).attr.name
#define MIN(x,y) (x>y)?(y):(x)
#define VM_VM_AREA  0x04 //copy from vmalloc.c
#define BRP_SYM_OFFSET_LEN (KSYM_NAME_LEN+sizeof(unsigned long)+4+MODULE_NAME_LEN+1)//symbol+(0x)offset+modulename
#if 0
#ifndef ARM_BREAKPOINT_LEN_3
#define ARM_BREAKPOINT_LEN_3 0x7
#endif
#ifndef ARM_BREAKPOINT_LEN_5
#define ARM_BREAKPOINT_LEN_5 0x1f
#endif
#ifndef ARM_BREAKPOINT_LEN_6
#define ARM_BREAKPOINT_LEN_6 0x3f
#endif
#ifndef ARM_BREAKPOINT_LEN_7
#define ARM_BREAKPOINT_LEN_7 0x7f
#endif
#endif

enum
{
    WP_TYPE_NONE,
    WP_IO_PHYS,
    WP_DRAM_PHYS,
    WP_DRAM_VIRT,
    BP_TYPE_SYM,
    BP_TYPE_VIRT
};
enum
{
    OUTPUT_TO_TRACE,
    OUTPUT_TO_CONSOLE,
};
#define IO_PHYS_ADDR_KW "io"
#define DRAM_PHYS_ADDR_KW "phys"
#define DRAM_VIRT_ADDR_KW  "virt"
#define SYMBOL_NAME_KW  "sym"

#define TRIG_USR_BACKTRACE_SIGNAL (SIGRTMIN+3) //keep the same as android crash dump DEBUGGER_SIGNAL
#define HWBP_STACK_ENTTRIES 10
#ifdef CONFIG_KPROBES
struct kprobe_info
{
    struct kprobe kp;
    u8 out_type;
    bool once;//only trigger once?
};
#endif
struct per_cpu_data
{
    cpumask_t saved_cpumask;
    unsigned long saved_pc;
};
struct wbp_info
{
    struct list_head list;
    struct perf_event  * __percpu * wbp;
    struct per_cpu_data  __percpu *saved_data;
    bool once;//only trigger once?
    bool dump_cond;//dump stack by condition?
    unsigned long cond_mask;
    unsigned long cond_val;
    unsigned char enable_map;
    u8 out_type;
};
struct wbp_option
{
    unsigned long addr;
    bool once;//only trigger once?
    bool dump_cond;//dump stack by condition?
    u8 type;
    unsigned long cond_mask;
    unsigned long cond_val;
    u8 out_type;
};
struct hw_breakpoint_info
{
    struct list_head list;
    char symbol[BRP_SYM_OFFSET_LEN];
    struct wbp_option pend_opt;
};
extern struct list_head vmap_area_list;
void * mt_perf_register_wp_bp(u8 bwp,unsigned long addr,bool once,bool is_cond_dump,unsigned long mask,unsigned long value,u8 outto_trace);
void mt_perf_unregister_wp_bp(unsigned long addr);
void watchpoint_selftest_func(u8 test);

static LIST_HEAD(wbp_list);
static bool _bnotify_usr=false;
static bool _bshow_regs=false;

static struct kobject *mt_perf_kobj=NULL;
static DEFINE_RWLOCK(access_wp_lock);
static unsigned char bEnableTrace=0;
#if defined(CONFIG_KALLSYMS) && defined(CONFIG_MODULES)
static LIST_HEAD(pending_brp_list);
static unsigned long _symbol2addr(char *symbol,struct module *mod)
{
    unsigned long vaddr,offset;
    unsigned long len;
    char name[KSYM_NAME_LEN];
    char symbolname[KSYM_NAME_LEN];
    char *ptmp;

    if(!symbol)
        return 0;

    ptmp=strchr(symbol,'+');

    if(ptmp)//handle offset
    {
        len=MIN(ptmp-symbol,sizeof(symbolname));
        strncpy(symbolname,symbol,len);
        symbolname[len]=0;
        ptmp++;
        if (kstrtoul(ptmp, 0, &offset))
        {
            pr_info("invalid @%d\n",__LINE__);
            return 0;
        }
    }
    else//no offset
    {
        len=MIN(strlen(symbol),sizeof(symbolname));
        strncpy(symbolname,symbol,len);
        symbolname[len]=0;
        offset=0;
    }
    pr_debug("symbolname=%s,mod:%s\n",symbolname,mod?mod->name:"non-mod");
    if(mod)//find symbol from module
    {
        if(!(ptmp=strchr(symbolname,':')))
        {
            snprintf(name,sizeof(name),"%s:%s",mod->name,symbolname);
            vaddr=module_kallsyms_lookup_name(name);
        }
        else
        {
            if(strncmp(mod->name,symbolname,ptmp-symbolname))
            {
                pr_debug("mod %s is not desired\n",mod->name);
                return 0;
            }   
            vaddr=module_kallsyms_lookup_name(symbolname);
            pr_debug("convert %s in mod %s,addr is %px\n",symbolname,mod->name,vaddr);
        }    
    }
    else
    {
        vaddr=kallsyms_lookup_name(symbolname);
    }

    if(!vaddr)return 0;

    return vaddr+offset;
}
static void _enable_trace_event(void)
{
    //enable trace event
    #ifdef CONFIG_TRACEPOINTS
    int ret=0;
    if((ret=trace_set_clr_event(__stringify(TRACE_SYSTEM),NULL,1)))
    {
        pr_err("%s:Failed to enable trace event under %s,ret=%d\n",__func__,__stringify(TRACE_SYSTEM),ret);            
        bEnableTrace=1;
    }
    else
    {
        bEnableTrace=0;
        pr_debug("%s:Success to enable trace event under %s\n",__func__,__stringify(TRACE_SYSTEM));               
    }
    #endif
}
static void _add_pending_hwbrp(struct list_head *head,char *symbol,struct wbp_option *opt)
{
    struct hw_breakpoint_info  *hwbrp;

    hwbrp=kmalloc(sizeof(struct hw_breakpoint_info),GFP_KERNEL);

    if(!hwbrp)
        return;
    strncpy(hwbrp->symbol,symbol,BRP_SYM_OFFSET_LEN);
    memcpy(&hwbrp->pend_opt,opt,sizeof(hwbrp->pend_opt));
    list_add_tail(&hwbrp->list,head);
}
#endif
static int _parse_hwbp_user_option(const char *str,bool bwp,struct wbp_option *option,unsigned char *sym)
{
    char * ptmp,*ptmp2;
    char tmp_buf[20]={0};
    unsigned long len;
    //wp format:addr_type,addr,once/repeat[,outto:console/trace] [,cond_mask][,cond_val]
    //brp format:addr_type,addr,once/repeat[,outto:console/trace]
    //brp format:addr_type,symbol_name+offset,once/repeat [,outto:console/trace]
    memset(option,0,sizeof(*option));
    ptmp=strchr(str, ',');
    if(!ptmp)
        return -EINVAL;
    //address type
    if(bwp)
    {
        if(!strncmp(str,IO_PHYS_ADDR_KW,ptmp-str))
            option->type=WP_IO_PHYS;
        else if(!strncmp(str,DRAM_PHYS_ADDR_KW,ptmp-str))
            option->type=WP_DRAM_PHYS;
        else if(!strncmp(str,DRAM_VIRT_ADDR_KW,ptmp-str))
            option->type=WP_DRAM_VIRT;
        else
            return -EINVAL;
    }
    else//breakpoint
    {
        if(!strncmp(str,SYMBOL_NAME_KW,ptmp-str))
            option->type=BP_TYPE_SYM;
        else if(!strncmp(str,DRAM_VIRT_ADDR_KW,ptmp-str))
            option->type=BP_TYPE_VIRT;
        else
            return -EINVAL;
    }

    ptmp++;

    ptmp2=strchr(ptmp, ',');
    if(!ptmp2)
        return -EINVAL;

    if(option->type==BP_TYPE_SYM)
    {
        len=MIN(ptmp2-ptmp,BRP_SYM_OFFSET_LEN-1);
        strncpy(sym,ptmp,len);
        sym[len]=0;
    }
    else
    {
        //address
        strncpy(tmp_buf,ptmp,MIN(ptmp2-ptmp,sizeof(tmp_buf)-1));

        pr_debug("tmp_buf=%s\n",tmp_buf);
        if (kstrtoul(tmp_buf, 0, &option->addr))
        {
            pr_info("invalid @%d\n",__LINE__);
            return -EINVAL;
        }
    }

    ptmp=ptmp2;
    ptmp++;
    pr_debug("once? %s\n",ptmp);
    //once
    if(!strncmp(ptmp,"once",4))option->once=true;
    else if(!strncmp(ptmp,"repeat",6))option->once=false;
    else return -EINVAL;

    //cond or output
    option->out_type=OUTPUT_TO_CONSOLE;
    option->dump_cond=false;
    ptmp=strchr(ptmp, ',');
    pr_debug("cond/out_type:%s(%d)\n",ptmp,option->out_type);
    if(ptmp)
    {
        ptmp++;

        if(!strncmp(ptmp,"outto:",6))//out path select
        {
            ptmp2=strchr(ptmp, ':');
            if(!ptmp2)return -EINVAL;
            ptmp2++;
            if(!strncmp(ptmp2,"console",7))//output to console
            {
                option->out_type=OUTPUT_TO_CONSOLE;
            }
            else if(!strncmp(ptmp2,"trace",5))//output to trace buffer
            {
                option->out_type=OUTPUT_TO_TRACE;
                _enable_trace_event();
            }
            else return -EINVAL;
            ptmp=strchr(ptmp, ',');
            if(ptmp)ptmp++;
            else return 0;
        }
        if(!bwp || option->once)
        {
            return 0;
        }
        //cond_mask
        ptmp2=strchr(ptmp, ',');
        if(!ptmp2)return 0;
        option->dump_cond=true;
        memset(tmp_buf,0,sizeof(tmp_buf));
        strncpy(tmp_buf,ptmp,MIN(ptmp2-ptmp,sizeof(tmp_buf)-1));

        pr_debug("cond_mask=%s\n",tmp_buf);

        if (kstrtoul(tmp_buf, 0, &option->cond_mask))
            return -EINVAL;

        if(!option->cond_mask)//cond_mask can not be 0
            return -EINVAL;

        ptmp2++;
        pr_debug("cond_val=%s\n",ptmp2);
        if (kstrtoul(ptmp2, 0, &option->cond_val))
                return -EINVAL;
    }
    return 0;
}
static struct wbp_info * get_global_event_info(struct perf_event *bp)
{
    struct wbp_info *new_wbp_info=NULL;
    struct perf_event   *wbp=NULL;

    read_lock(&access_wp_lock);
    list_for_each_entry(new_wbp_info,&wbp_list,list)
    {
        wbp=per_cpu(*(new_wbp_info->wbp), smp_processor_id());
        if(wbp==bp)
        {
            break;
        }
    }
    read_unlock(&access_wp_lock);
    if(wbp!=bp)
    {
        return NULL;
    }
    return new_wbp_info;
}
static void _show_simple_regs(struct pt_regs *regs,u8 out_type)
{
    char regname[MAX_REG_NAME];
#ifdef CONFIG_ARM
    unsigned long val;
    u8 index;
    char *regseq[]={"r0","r1","r2","r3","r4","r5","r6","r7",
                "r8","r9","r10","fp","ip","sp","lr","pc","psr"
                };
    for(index=0;index<sizeof(regseq)/sizeof(regseq[0]);index++)
    {
        snprintf(regname,MAX_REG_NAME,"%s",regseq[index]);
        val=*(&regs->ARM_r0+index);
        if(out_type==OUTPUT_TO_TRACE)
        {
            trace_mtk_perf_reg_val(regname,val);
        }
        else
        {
            if((index%4)==0)
            {
                printk("\n");
            }
            printk("%s:%08lx ",regname,val);
        }
    }
    printk("\n");
#endif
#ifdef CONFIG_ARM64
    int i, top_reg;
    u64 lr, sp;

    if (compat_user_mode(regs)) 
    {
        lr = regs->compat_lr;
        sp = regs->compat_sp;
        top_reg = 12;
    } 
    else 
    {
        lr = regs->regs[30];
        sp = regs->sp;
        top_reg = 29;
    }
    for(i=0;i<=top_reg;i++)
    {
        snprintf(regname,MAX_REG_NAME,"%c%d",compat_user_mode(regs)?'R':'X',i);
        if(out_type==OUTPUT_TO_TRACE)
        {
            trace_mtk_perf_reg_val(regname,(unsigned long)(regs->regs[i]));
            continue;
        }
        if(i && ((i%4)==0))printk("\n");
        printk("%s:%016llx ",regname,regs->regs[i]);
    }
    if(out_type==OUTPUT_TO_TRACE)
    {
        snprintf(regname,MAX_REG_NAME,"%s","sp");
        trace_mtk_perf_reg_val(regname,(unsigned long)(sp));
        snprintf(regname,MAX_REG_NAME,"%s","lr");
        trace_mtk_perf_reg_val(regname,(unsigned long)(sp));
        snprintf(regname,MAX_REG_NAME,"%s","pc");
        trace_mtk_perf_reg_val(regname,(unsigned long)(regs->pc));
        snprintf(regname,MAX_REG_NAME,"%s","pst");
        trace_mtk_perf_reg_val(regname,(unsigned long)(regs->pstate));
    }
    else
    {
        printk("sp:%016llx  ",sp);
        printk("lr:%016llx  ",lr);
        printk("pc:%016llx  ",regs->pc);
        printk("pstate:%08llx  ",regs->pstate); 

        printk("\n");       
    }
#endif
}
static void show_stacktrace_hwbp(struct pt_regs *regs,u8 out_type,unsigned long saved_pc)
{
#ifdef CONFIG_STACKTRACE
    struct stack_trace trace;
    unsigned long entries[HWBP_STACK_ENTTRIES];
    unsigned int level=0;
    char comm[TASK_COMM_LEN];

    if(out_type==OUTPUT_TO_CONSOLE)
    {
        pr_info("stacktrace for %s-%d:\n",get_task_comm(comm,current),task_pid_nr(current));
        trace.nr_entries    = 0;
        trace.skip      = 0;
        trace.max_entries   = HWBP_STACK_ENTTRIES;
        trace.entries=entries;
        save_stack_trace_regs(regs,&trace);
        while(level<trace.nr_entries)
        {
                if(level==0 && saved_pc)
                {
                    pr_info("[%lx] %pS\n",saved_pc,(void *)saved_pc);
                }
                pr_info("[%lx] %pS\n",trace.entries[level],(void *)trace.entries[level]);
            level++;
        }
    }
    else
    {
        pr_info("stacktrace for %s-%d is in the trace buffer\n",get_task_comm(comm,current),task_pid_nr(current));
        if(saved_pc)
        {
            trace_mtk_perf_reg_val("pc",saved_pc);
        }
        else
		{
            trace_mtk_perf_reg_val("pc",instruction_pointer(regs));			
		}
        //trace_dump_stack(0);
    }
#else
    if(saved_pc)
    {
        pr_info("[%lx] %pS\n",saved_pc,(void *)saved_pc);
    }
    dump_stack();
#endif
    if(_bshow_regs)
    {
        _show_simple_regs(regs,out_type);
    }
}
static void _enable_single_step(struct perf_event *bp, unsigned long addr)
{
    struct arch_hw_breakpoint *info = counter_arch_bp(bp);

    arch_uninstall_hw_breakpoint(bp);
    info->ctrl.type=ARM_BREAKPOINT_EXECUTE;
    info->address=addr;
    info->ctrl.len = ARM_BREAKPOINT_LEN_4;
    arch_install_hw_breakpoint(bp);
}
static unsigned long _get_return_addr(struct pt_regs *regs)
{
#define RETURN_ADDR_MAX_ENTRIES 2
    struct stack_trace trace;
    unsigned long entries[RETURN_ADDR_MAX_ENTRIES];
    
    trace.nr_entries    = 0;
    trace.skip      = 0;
    trace.max_entries   = RETURN_ADDR_MAX_ENTRIES;
    trace.entries       = entries;
    save_stack_trace_regs(regs,&trace);
    if(RETURN_ADDR_MAX_ENTRIES==trace.nr_entries)
    {
        return entries[1];
    }
    return 0;   
}
static u8 enable_single_step(struct perf_event *bp,struct pt_regs *regs)
{
    u64 step_addr;  
#ifdef CONFIG_ARM64
    if (aarch64_insn_is_store_ex(le32_to_cpu(*((unsigned long *)instruction_pointer(regs)))))
#else
//STREX/STREXD/STREXB/STREXH
    u32 inst=le32_to_cpu(*((unsigned long *)instruction_pointer(regs)));
    if((inst &0x1900090)==0x1800090)
#endif
    {
        step_addr=_get_return_addr(regs);
        pr_info("store exclusive instruction,step over to return addr %llx\n",
        step_addr);
    }
    else
    {
        step_addr=instruction_pointer(regs)+4;
    }
    _enable_single_step(bp,(unsigned long)step_addr);
    return (step_addr==(instruction_pointer(regs)+4));
}
static void disable_single_step(struct perf_event *bp)
{
    struct arch_hw_breakpoint *info = counter_arch_bp(bp);
    struct perf_event_attr *wp_attr;

    arch_uninstall_hw_breakpoint(bp);
    wp_attr=&bp->attr;

    switch (wp_attr->bp_type) {
    case HW_BREAKPOINT_X:
        info->ctrl.type = ARM_BREAKPOINT_EXECUTE;
        break;
    case HW_BREAKPOINT_R:
        info->ctrl.type = ARM_BREAKPOINT_LOAD;
        break;
    case HW_BREAKPOINT_W:
        info->ctrl.type = ARM_BREAKPOINT_STORE;
        break;
    case HW_BREAKPOINT_RW:
        info->ctrl.type = ARM_BREAKPOINT_LOAD | ARM_BREAKPOINT_STORE;
        break;
    default:
        info->ctrl.type=ARM_BREAKPOINT_STORE;
        break;
    }
    if(info->ctrl.type!=ARM_BREAKPOINT_EXECUTE)
    {
        switch (wp_attr->bp_len) {
        case HW_BREAKPOINT_LEN_1:
            info->ctrl.len = ARM_BREAKPOINT_LEN_1;
            break;
        case HW_BREAKPOINT_LEN_2:
            info->ctrl.len = ARM_BREAKPOINT_LEN_2;
            break;
		case HW_BREAKPOINT_LEN_4:
            info->ctrl.len = ARM_BREAKPOINT_LEN_4;
            break;
#if 0			
        case HW_BREAKPOINT_LEN_3:
            info->ctrl.len = ARM_BREAKPOINT_LEN_3;
            break;
        case HW_BREAKPOINT_LEN_5:
            info->ctrl.len = ARM_BREAKPOINT_LEN_5;
            break;
        case HW_BREAKPOINT_LEN_6:
            info->ctrl.len = ARM_BREAKPOINT_LEN_6;
            break;
        case HW_BREAKPOINT_LEN_7:
            info->ctrl.len = ARM_BREAKPOINT_LEN_7;
            break;
#endif
        case HW_BREAKPOINT_LEN_8:
            info->ctrl.len = ARM_BREAKPOINT_LEN_8;
            break;
        default:
            pr_warning("unknown bp len %llx\n",wp_attr->bp_len);
            break;
        }
    }
    info->address=wp_attr->bp_addr;
    arch_install_hw_breakpoint(bp);
}
static void _notify_hwbp_to_user(void *addr,struct pt_regs *regs,u8 out_type,bool bnotify)
{
    struct siginfo siginfo;
    struct sighand_struct *sighand;
    struct k_sigaction *ka;
    unsigned char bsend_signal=0;

    if(!bnotify)return;

    if((interrupts_enabled(regs)) && current->mm && (current->mm!=&init_mm))
    {
        sighand = current->sighand;
        spin_lock_irq(&sighand->siglock);
        ka = &sighand->action[TRIG_USR_BACKTRACE_SIGNAL-1];
        if ((ka->sa.sa_handler != SIG_IGN) && (ka->sa.sa_handler != SIG_DFL))
        {
            bsend_signal=1;
        }
        spin_unlock_irq(&sighand->siglock);//we accept the race condition of changing signal handler to IGN or DFL between this gap,but we must release spin lock since force_sig_info will acquire the spin lock
        if(bsend_signal)
        {
        memset(&siginfo,0,sizeof(siginfo));
            siginfo.si_signo = TRIG_USR_BACKTRACE_SIGNAL;
            siginfo.si_errno = 1;//short format
            if(out_type==OUTPUT_TO_TRACE)
            {
                siginfo.si_errno |= 1UL<<28;//output the user space stacktrace to trace buffer
            }
            siginfo.si_code  = TRAP_HWBKPT;
            siginfo.si_addr  = (void __user *)addr;
            force_sig_info(siginfo.si_signo,&siginfo, current);
            pr_info("notify signal %d to user space for backtrace\n",TRIG_USR_BACKTRACE_SIGNAL);
        }
    }
}
static void wbp_handler(struct perf_event *wbp,struct perf_sample_data *data,struct pt_regs *regs)
{
    struct arch_hw_breakpoint *info;
    struct wbp_info *wbp_info=NULL;
    struct perf_event_attr *wp_attr;
    unsigned long mon_data=0;
    bool action=false;
    unsigned long len;
    unsigned long mask;
    cpumask_t *cpu_mask;
    struct per_cpu_data *saved_data;
    int cpu=get_cpu();

    info=counter_arch_bp(wbp);
    wp_attr=&wbp->attr;

   // pr_debug("cpu %d:wbp:%p trigger:%llx,address %llx orig addr %llx pc %pS\n",smp_processor_id(),wbp,info->trigger,info->address,wbp->attr.bp_addr,instruction_pointer(regs));

    wbp_info=get_global_event_info(wbp);
    if(!wbp_info)
    {
        pr_err("can not find the wbp %p\n",wbp);
        #if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
        __perf_event_disable(wbp);
        #else
        perf_event_disable_local(wbp);
        #endif
        wbp_info->enable_map &=~(1U<<cpu);
        put_cpu();
        return;
    }
#ifdef CONFIG_ARM64
    if(wp_attr->bp_type==HW_BREAKPOINT_X)
    {
        mask=(~0x03UL);
    }
    else
    {
        mask=(~0x07UL);
    }
#endif
#ifdef CONFIG_ARM
    mask=(~0x03UL);
#endif
    saved_data=(struct per_cpu_data *)per_cpu_ptr(wbp_info->saved_data,cpu);
    if(info->address==(wp_attr->bp_addr & mask))//first time trigger
    {
        cpu_mask=&saved_data->saved_cpumask;
        cpumask_copy(cpu_mask,&current->cpus_allowed);
        set_cpus_allowed_ptr(current,cpumask_of(cpu));//bind current task to current cpu,avoid step over happend to other cpus
        put_cpu();
        //enable step over
        if(enable_single_step(wbp,regs))
        {
            saved_data->saved_pc=0;
        }
        else
        {
            saved_data->saved_pc=instruction_pointer(regs);         
        }
    }//second time trigger
    else
    {
        if(wbp_info->once)
        {
            #if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
                __perf_event_disable(wbp);
            #else
                perf_event_disable_local(wbp);
            #endif
            wbp_info->enable_map &=~(1U<<cpu);
        }
        else
        {
            disable_single_step(wbp);//restore
        }
        put_cpu();
        if (wp_attr->bp_type & (HW_BREAKPOINT_W|HW_BREAKPOINT_R))//watchpoint 
        {
            len=wp_attr->bp_len;

            while(len)
            {
                mon_data <<=8;
                mon_data |=*(volatile u8 *)(unsigned long)(wp_attr->bp_addr+len-1);
                len--;
            }
            if(!wbp_info)//in case debug event is issued during wp registration
            {
                action=true;
            }
            else if(!wbp_info->dump_cond)//always dump
            {
                action=true;
            }
            else//conditional dump
            {
                if((mon_data & wbp_info->cond_mask)==wbp_info->cond_val)
                {
                   action=true;
                }
            }
            if(action)
            {
                if(wbp_info->out_type==OUTPUT_TO_TRACE)
                {
                    trace_mtk_perf_mem_val((unsigned long)wp_attr->bp_addr,mon_data);
                }
                else
                {
                    pr_info("The monitored value in addr %lx is %lx\n",(unsigned long)wp_attr->bp_addr,mon_data);
                }
                show_stacktrace_hwbp(regs,wbp_info->out_type,saved_data->saved_pc);
                _notify_hwbp_to_user((void *)(counter_arch_bp(wbp)->trigger),regs,wbp_info->out_type,_bnotify_usr);
            }
        }
        else if (wp_attr->bp_type==HW_BREAKPOINT_X)//breakpoint just print backtrace
        {
            show_stacktrace_hwbp(regs,wbp_info->out_type,saved_data->saved_pc);
            _notify_hwbp_to_user((void *)(counter_arch_bp(wbp)->trigger),regs,wbp_info->out_type,_bnotify_usr);
        }
        cpu_mask=&saved_data->saved_cpumask;
        set_cpus_allowed_ptr(current,cpu_mask);//restore current task cpumask
    }
}
static void _wp_attr_init(struct perf_event_attr * wp_attr,unsigned char bwp,struct wbp_option *opt)
{
    unsigned long wp_addr=opt->addr;
    u8 i=sizeof(unsigned long);
    int hi=-1;

    wp_attr->bp_type = bwp?HW_BREAKPOINT_W:HW_BREAKPOINT_X;

    if(opt->type==WP_IO_PHYS)//io address must be aligned to word
    {
        wp_addr &=~0x03UL;
        wp_attr->bp_len=HW_BREAKPOINT_LEN_4;
    }
    else if(!bwp)//breakpoint only support word
    {
        wp_addr &=~0x03UL;
        wp_attr->bp_len=HW_BREAKPOINT_LEN_4;
    }
    else
    {
        if(!opt->dump_cond)// always trigger type  align to word
        {
#ifdef CONFIG_ARM64
            wp_addr &=~0x07UL;
            wp_attr->bp_len=HW_BREAKPOINT_LEN_8;             
#else
            wp_addr &=~0x03UL;
            wp_attr->bp_len=HW_BREAKPOINT_LEN_4;             
#endif
        }
        else//repeat conditional trigger
        {
            while(i)
            {
                if(opt->cond_mask & (0xffUL<<((i-1)*8)))
                {
                    if(i>hi)hi=i;
                }
                i--;
            }
            wp_attr->bp_len=HW_BREAKPOINT_LEN_1+hi-1;//how many bytes
        }
    }
    wp_attr->bp_addr=wp_addr;
}
#ifdef CONFIG_KPROBES
static void kprobe_brp_post_handler(struct kprobe *p, struct pt_regs *regs,
                unsigned long flags)
{
    struct kprobe_info * kp_info=container_of(p,struct kprobe_info,kp);

    pr_info("kprobe_brp_post_handler:%pS\n",(void *)p->addr);

    show_stacktrace_hwbp(regs,kp_info->out_type,0);
    _notify_hwbp_to_user((void *)(p->addr),regs,kp_info->out_type,_bnotify_usr);

    if(kp_info->once)
    {
        if(disable_kprobe(p))
        {
            pr_err("disable kprobes %p fails\n",p);
            return;
        }
        pr_info("disable kprobes %pS after trigger one time\n",(void *)p->addr);
    }
}
#endif
static void * _register_wp_bp(unsigned char bwp,struct wbp_option *opt)
{
    struct perf_event * __percpu * wbp;
    struct perf_event_attr wp_attr;
    int ret=0;
    struct wbp_info *new_wbp_info=NULL;
    int cpu;
    unsigned long wp_addr=opt->addr;
#ifdef FIX_ARM_BREAKPOINT_STEP_CTRL_ISSUE
    struct arch_hw_breakpoint *arch_info=NULL;
#endif
    pr_info("%s-%d-%d:Enable %s: 0x%lx\n",current->comm,current->pid,current->tgid,bwp?"hwwp":"hwbrp",wp_addr);

#ifdef CONFIG_KPROBES
    if(!bwp)//kprobes only support breakpoint now
    {
       struct kprobe_info * kp_infp=(struct kprobe_info *)kzalloc(sizeof(struct kprobe_info),GFP_KERNEL);
       struct kprobe *kp;
       if(!kp_infp)return ERR_PTR(-ENOMEM);
       kp=&kp_infp->kp;
       kp->addr=(void *)wp_addr;
       kp->post_handler = kprobe_brp_post_handler;
       kp_infp->once=opt->once;
       kp_infp->out_type=opt->out_type;
       if((ret=register_kprobe(kp)))
       {
            pr_err("register kprobe addr %lx Fail %d\n",wp_addr,ret);
            kfree(kp_infp);
            return NULL;
       }
       pr_info("%lx registration OK\n",wp_addr);
       return kp_infp;
    }
#endif
    hw_breakpoint_init(&wp_attr);
    wp_attr.disabled=1;//disable firstly to ignore the debug event during registration
    _wp_attr_init(&wp_attr,bwp,opt);

    if(wp_addr<TASK_SIZE)//user space address
    {
        return ERR_PTR(-EINVAL);
    }

    wbp = register_wide_hw_breakpoint(&wp_attr,wbp_handler, NULL);

    if (IS_ERR((void __force *)wbp)) {
        ret = PTR_ERR((void __force *)wbp);
        pr_info("registration Fail %d\n",ret);
        return wbp;
    }

    new_wbp_info=(struct wbp_info *)kzalloc(sizeof(struct wbp_info),GFP_KERNEL);
    if(!new_wbp_info)return ERR_PTR(-ENOMEM);
    INIT_LIST_HEAD(&new_wbp_info->list);
    new_wbp_info->wbp=wbp;

    write_lock(&access_wp_lock);
    list_add_tail(&new_wbp_info->list,&wbp_list);
    write_unlock(&access_wp_lock);
    new_wbp_info->once=opt->once;
    new_wbp_info->dump_cond=opt->dump_cond;
    new_wbp_info->cond_mask=opt->cond_mask;
    new_wbp_info->cond_val=opt->cond_val;
    new_wbp_info->out_type=opt->out_type;
    new_wbp_info->saved_data=alloc_percpu(struct per_cpu_data); 
    if(!new_wbp_info->saved_data)
    {
        goto alloc_saved_data_fail;
    }
    get_online_cpus();
    for_each_online_cpu(cpu)
    {
    #ifdef FIX_ARM_BREAKPOINT_STEP_CTRL_ISSUE	
        arch_info=counter_arch_bp(per_cpu(*wbp,cpu));
        arch_info->step_ctrl.enabled=0;//due to register_perf_hw_breakpoint does not initialize,so step_ctrl may be enabled by mistake
    #endif
        perf_event_enable(per_cpu(*wbp,cpu));
        new_wbp_info->enable_map |=(1U<<cpu);
    }
    put_online_cpus();

    pr_info("%lx registration OK\n",wp_addr);
    return new_wbp_info;
    
alloc_saved_data_fail:
    write_lock(&access_wp_lock);
    list_del(&new_wbp_info->list);
    write_unlock(&access_wp_lock);      
    unregister_wide_hw_breakpoint(new_wbp_info->wbp);
    kfree(new_wbp_info);
    return ERR_PTR(-ENOMEM);
}
static void _unregister_wp_bp(unsigned long addr)
{
    struct perf_event  * wbp;
    struct wbp_info *new_wbp_info=NULL;
    struct wbp_info *tmp=NULL;
    unsigned char found=0;
    struct perf_event_attr *wp_attr;

#ifdef CONFIG_KPROBES
    {
        struct kprobe *p;

        write_lock(&access_wp_lock);
        p=get_kprobe((void *)addr);
        write_unlock(&access_wp_lock);
        if(p)
        {
            struct kprobe_info * kp_info=container_of(p,struct kprobe_info,kp);
            pr_info("uninstall %lx in kprobes\n",addr);
            unregister_kprobe(p);
            kfree(kp_info);
            return;
        }
    }
#endif
    write_lock(&access_wp_lock);
    list_for_each_entry_safe(new_wbp_info,tmp,&wbp_list,list)
    {
        wbp=per_cpu(*(new_wbp_info->wbp), smp_processor_id());
        wp_attr=&wbp->attr;
        if((unsigned long)(wp_attr->bp_addr)==addr)
        {
            list_del(&new_wbp_info->list);
            found=1;
            break;
        }
    }
    write_unlock(&access_wp_lock);
    if(found)
    {
        unregister_wide_hw_breakpoint(new_wbp_info->wbp);
        if(new_wbp_info->saved_data)
        {
            free_percpu(new_wbp_info->saved_data);
        }
        kfree(new_wbp_info);
    }
    pr_debug("uninstall %lx in wbp list\n",addr);
}
static void reinstall_all_wide_hw_breakpoint_on_cpu(void *info)
{
    struct wbp_info *new_wbp_info=NULL;
    struct perf_event  * wbp;

    read_lock(&access_wp_lock);
    list_for_each_entry(new_wbp_info,&wbp_list,list)
    {
        wbp=per_cpu(*(new_wbp_info->wbp), smp_processor_id());
        //pr_info("cpu %d:reinstall wbp %p\n",smp_processor_id(),wbp);
        if((new_wbp_info->enable_map & (1U<<smp_processor_id())) && arch_install_hw_breakpoint(wbp))
        {
            pr_err("reinstall %p fails\n",wbp);
        }
    }
    read_unlock(&access_wp_lock);
}
static void uninstall_all_wide_hw_breakpoint_on_cpu(void *info)
{
    struct wbp_info *new_wbp_info=NULL;
    struct perf_event  * wbp;

    read_lock(&access_wp_lock);
    list_for_each_entry(new_wbp_info,&wbp_list,list)
    {
        wbp=per_cpu(*(new_wbp_info->wbp), smp_processor_id());
        if(!(new_wbp_info->enable_map & (1<<smp_processor_id())))// disabled
        {
            //pr_info("the wbp %lx is already uninstalled\n",wbp->attr.bp_addr);
            continue;
        }
        
        if((smp_processor_id()!=0) && PERF_EVENT_STATE_ACTIVE==wbp->state)
        {
            pr_info("event state is ACTIVE,perf core will uninstall it\n");
            continue;
        }
        pr_debug("cpu %d:uninstall wbp %p,state %d\n",smp_processor_id(),wbp,wbp->state);
        arch_uninstall_hw_breakpoint(wbp);
    }
    read_unlock(&access_wp_lock);
}
static int _register_io_phys_wp(struct wbp_option *opt)
{
    struct vmap_area *va;
    struct vm_struct *vm;
    struct vmap_area *tmp;
    void *ret_ptr;
    unsigned long vaddr;
    unsigned long phys=opt->addr;

    list_for_each_entry_safe(va,tmp,&vmap_area_list, list)
    {
        if (!(va->flags & VM_VM_AREA))
            continue;
        vm = va->vm;
        if(!(vm->flags & VM_IOREMAP))
            continue;
        if(!vm->phys_addr)
            continue;
        if((phys>=vm->phys_addr) && (phys<(vm->phys_addr+get_vm_area_size(vm))))
        {
            vaddr=(unsigned long)vm->addr+(phys-vm->phys_addr);
            opt->addr=vaddr;
            ret_ptr=_register_wp_bp(true,opt);
            if(!ret_ptr)return 0;
            if(IS_ERR(ret_ptr))
            {
                pr_err("register wp addr %lx fails\n",vaddr);
                return PTR_ERR(ret_ptr);
            }
        }
    }
    return 0;
}
#ifdef CONFIG_PM
static int cpu_pm_notify(struct notifier_block *self, unsigned long action,
                 void *v)
{
    pr_debug("%s:cpu %d action %ld\n",__func__,smp_processor_id(),action);
    if (action == CPU_PM_EXIT)//for boot cpu resume
    {
        reinstall_all_wide_hw_breakpoint_on_cpu(NULL);
        pr_debug("cpu %d:CPU_PM_EXIT\n",smp_processor_id());
    }
    else if(action==CPU_PM_ENTER)//for boot cpu suspend
    {
        uninstall_all_wide_hw_breakpoint_on_cpu(NULL);
        pr_debug("cpu %d:CPU_PM_ENTER\n",smp_processor_id());        
    }
    return NOTIFY_OK;
}
static struct notifier_block cpu_pm_nb = {
    .notifier_call = cpu_pm_notify,
};
#endif
#ifdef CONFIG_HOTPLUG_CPU
/*
 * The CPU hotplug notifiers are history, remove it!
 * see f4c09f8
*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0)
static int cpu_hotplug_notify(struct notifier_block *self,
                      unsigned long action, void *hcpu)
{
        int cpu = (long)hcpu;
    if ((action & ~CPU_TASKS_FROZEN) == CPU_ONLINE)
    {
            pr_debug("cpu %d:cpu_online\n",cpu);
            smp_call_function_single((int)cpu, reinstall_all_wide_hw_breakpoint_on_cpu, NULL, 1);
    }
    if ((action & ~CPU_TASKS_FROZEN) == CPU_DOWN_PREPARE)
    {
            pr_debug("cpu %d:cpu_down\n",cpu);
            smp_call_function_single((int)cpu, uninstall_all_wide_hw_breakpoint_on_cpu, NULL, 1);
    }
    return NOTIFY_OK;
}

static struct notifier_block cpu_hotplug_nb = {
    .notifier_call = cpu_hotplug_notify,
};
#else
int mt_perf_cpu_online(unsigned int cpu)
{
    pr_debug("%s-%d:%s-%d\n",__func__,cpu,current->comm,smp_processor_id());
    reinstall_all_wide_hw_breakpoint_on_cpu(NULL);
    return 0;
}
int mt_perf_cpu_offline(unsigned int cpu)
{
    pr_debug("%s-%d:%s-%d\n",__func__,cpu,current->comm,smp_processor_id());
    uninstall_all_wide_hw_breakpoint_on_cpu(NULL);
    return 0;
}   
#endif
#endif
#if defined(CONFIG_KALLSYMS) && defined(CONFIG_MODULES)
static int module_event_notify(struct notifier_block *self, unsigned long action,
                 void *v)
{
    struct module *mod=v;
    struct hw_breakpoint_info *hw_brp;
    struct hw_breakpoint_info *tmp;
    unsigned long vaddr;
    void *ret_ptr;
    struct wbp_option * option;

    if(action==MODULE_STATE_COMING)
    {
        list_for_each_entry_safe(hw_brp,tmp,&pending_brp_list,list)
        {
            vaddr=_symbol2addr(hw_brp->symbol,mod);
            if(!vaddr)
                continue;
            list_del(&hw_brp->list);
            option=&hw_brp->pend_opt;
            option->addr=vaddr;
            ret_ptr=_register_wp_bp(false,option);
            kfree(hw_brp);
            if(!ret_ptr)break;
            if(IS_ERR(ret_ptr))
            {
                pr_err("register wp addr %lx fails\n",option->addr);
                break;
            }
        }
    }
    return NOTIFY_OK;
}
static struct notifier_block module_event_nb = {
    .notifier_call = module_event_notify,
};
#endif
static ssize_t watchpoint_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t count)
{
    const char * name=attr?attr_name(*attr):NULL;
    unsigned char bwp=1;
    void *ret_ptr;
    struct wbp_option option;
    unsigned char symbolname[BRP_SYM_OFFSET_LEN];

    if(name && !strncmp(name,"hwbrp",5))
    {
        bwp=0;
    }
    if(_parse_hwbp_user_option(buf,bwp,&option,symbolname))
    {
        return -EINVAL;
    }
    if(bwp && (option.type==WP_IO_PHYS))
    {
        if(_register_io_phys_wp(&option))
        {
            return -EINVAL;
        }
        goto done;
    }
    #if defined(CONFIG_KALLSYMS) && defined(CONFIG_MODULES)
    if(option.type==BP_TYPE_SYM)
    {
        //symbolname[BRP_SYM_OFFSET_LEN-1]=0;
        option.addr=_symbol2addr(symbolname,NULL);
        if(!option.addr)
        {
            pr_err("can not find the symbol %s,search it in module later\n",symbolname);
            _add_pending_hwbrp(&pending_brp_list,symbolname,&option);
            return -EAGAIN;
        }
    }
    #endif
    ret_ptr=_register_wp_bp(bwp,&option);
    if(!ret_ptr)goto done;
    if(IS_ERR(ret_ptr))return PTR_ERR(ret_ptr);

done:

    return count;
}
static ssize_t hwbrp_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t count)
{
    return watchpoint_store(kobj,attr,buf,count);
}
static ssize_t watchpoint_show(struct kobject *kobj,struct kobj_attribute *pattr, char *buf)
{
    ssize_t count = 0;
    struct perf_event   * wbp;
    struct wbp_info *new_wbp_info=NULL;
    char *type_str;
    int cpu;

    read_lock(&access_wp_lock);
    get_online_cpus();
    for_each_online_cpu(cpu)
    {
        count += snprintf(buf+count,max((ssize_t)PAGE_SIZE - count, (ssize_t)0),"cpu %d:\n",cpu);
        list_for_each_entry(new_wbp_info,&wbp_list,list)
        {
            wbp=per_cpu(*(new_wbp_info->wbp), smp_processor_id());
            if(wbp->attr.bp_type==HW_BREAKPOINT_X)type_str="x";
            else if(wbp->attr.bp_type==HW_BREAKPOINT_R)type_str="r";
            else if(wbp->attr.bp_type==HW_BREAKPOINT_W)type_str="w";
            else if(wbp->attr.bp_type==HW_BREAKPOINT_RW)type_str="rw";
            else type_str="na";
#ifdef CONFIG_ARM64
            count += snprintf(buf+count,max((ssize_t)PAGE_SIZE - count, (ssize_t)0), "\t%s:0x%llx,len %x,%s,enable_map %x ",type_str,counter_arch_bp(wbp)->address,counter_arch_bp(wbp)->ctrl.len,new_wbp_info->once?"once":"repeat",new_wbp_info->enable_map);
#else
            count += snprintf(buf+count,max((ssize_t)PAGE_SIZE - count, (ssize_t)0), "\t%s:0x%x,len %x,%s,enable_map %x ",type_str,counter_arch_bp(wbp)->address,counter_arch_bp(wbp)->ctrl.len,new_wbp_info->once?"once":"repeat",new_wbp_info->enable_map);
#endif
            if(new_wbp_info->dump_cond)
            {
                count += snprintf(buf+count,max((ssize_t)PAGE_SIZE - count, (ssize_t)0), "cond_msk:%lx cond_val:%lx",new_wbp_info->cond_mask,new_wbp_info->cond_val);
            }
            count += snprintf(buf+count,max((ssize_t)PAGE_SIZE - count, (ssize_t)0), "  outto:%s ",new_wbp_info->out_type==OUTPUT_TO_CONSOLE?"console":"trace");
            count += snprintf(buf+count,max((ssize_t)PAGE_SIZE - count, (ssize_t)0), "\n");
        }
    }
    put_online_cpus();
    read_unlock(&access_wp_lock);
    return count;
}
static ssize_t uninstall_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t count)
{
    unsigned long wp_addr;

    if (kstrtoul(buf, 0, &wp_addr))
        return -EINVAL;
    _unregister_wp_bp(wp_addr);
    return count;
}
static ssize_t notify_usr_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t count)
{
    unsigned long is_notify_usr=0;

    if (kstrtoul(buf, 0, &is_notify_usr))
        return -EINVAL;
    if(is_notify_usr)
    {
        _bnotify_usr=true;
    }
    else
    {
        _bnotify_usr=false;
    }
    return count;
}
static ssize_t notify_usr_show(struct kobject *kobj,struct kobj_attribute *pattr, char *buf)
{
    ssize_t count = 0;
    count += snprintf(buf+count,max((ssize_t)PAGE_SIZE - count, (ssize_t)0),"%d\n",_bnotify_usr?1:0);
    return count;
}
static ssize_t show_regs_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t count)
{
    unsigned long is_show_regs=0;

    if (kstrtoul(buf, 0, &is_show_regs))
        return -EINVAL;
    if(is_show_regs)
    {
        _bshow_regs=true;
    }
    else
    {
        _bshow_regs=false;
    }
    return count;
}
static ssize_t show_regs_show(struct kobject *kobj,struct kobj_attribute *pattr, char *buf)
{
    ssize_t count = 0;
    count += snprintf(buf+count,max((ssize_t)PAGE_SIZE - count, (ssize_t)0),"%d\n",_bshow_regs?1:0);
    return count;
}
static unsigned long _selftest_get_pte(unsigned long addr)
{
    pgd_t *pgd;
    pud_t *pud;
    pmd_t *pmd;
    pte_t *pte; 
    
    pgd = pgd_offset_k(addr);
    if (pgd_none(*pgd))
        return 0;
     
    pud = pud_offset(pgd, addr);
    if (pud_none(*pud))
        return 0;

    pmd = pmd_offset(pud, addr);
    if (pmd_none(*pmd) || pmd_bad(*pmd))
        return 0;    
    
    pte = pte_offset_kernel(pmd, addr);
    if (pte_none(*pte))
        return 0;
    return (unsigned long)pte;
}
static ssize_t selftest_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t count)
{
    unsigned long is_test=0;
    unsigned long *ptr=NULL;
    void *wp_ret;
    unsigned long ptr2=0;
    
    if (kstrtoul(buf, 0, &is_test))
        return -EINVAL;
    if(is_test)
    {
        //test watchpoint
        ptr=kmalloc(0x1000,GFP_KERNEL);
        if(!ptr)return -ENOMEM;
        pr_info("test watchpoint %px\n",ptr);
        wp_ret=mt_perf_register_wp_bp(1,(unsigned long)ptr,
                      false,true,0xffff,0x5678,0);
        if(IS_ERR(wp_ret))
        {
            return PTR_ERR(wp_ret);
        }
        *ptr=0x12345678;
        pr_info("write %lx\n",*ptr);
        *ptr=0x12345688;
        pr_info("write %lx\n",*ptr);
        *(ptr+1)=0x55667788;
        mt_perf_unregister_wp_bp((unsigned long)ptr);
        pr_info("test watchpoint with kmalloc %px end\n",ptr);
        *ptr=0x12345678;
        *(ptr+1)=0x55667788;
        kfree(ptr);
        
        //test watchpoint with vmalloc
        ptr=vmalloc(0x100);
        if(!ptr)return -ENOMEM;
        pr_info("test watchpoint with pte %px\n",ptr);
        ptr2=(unsigned long)_selftest_get_pte((unsigned long)ptr);
        wp_ret=mt_perf_register_wp_bp(1,(unsigned long)ptr2,false,false,0,0,0);
        if(IS_ERR(wp_ret))
        {
            return PTR_ERR(wp_ret);
        }
        *ptr=0x12345678;
        *(ptr+1)=0x55667788;
        vfree(ptr);
        mt_perf_unregister_wp_bp(ptr2);
        
        pr_info("test watchpoint with pte %lx end\n",ptr2);
        //test watchpoint
        ptr=kmalloc(0x1000,GFP_KERNEL);
        if(!ptr)return -ENOMEM;
        pr_info("test watchpoint %px outtype trace\n",ptr);
        wp_ret=mt_perf_register_wp_bp(1,(unsigned long)ptr,
                      false,true,0xffff,0x5678,1);
        if(IS_ERR(wp_ret))
        {
            return PTR_ERR(wp_ret);
        }
        *ptr=0x12345678;
        pr_info("write %lx\n",*ptr);
        *ptr=0x12345688;
        pr_info("write %lx\n",*ptr);
        *(ptr+1)=0x55667788;
        mt_perf_unregister_wp_bp((unsigned long)ptr);
        pr_info("test watchpoint with kmalloc %px end\n",ptr);
        *ptr=0x12345678;
        *(ptr+1)=0x55667788;
        kfree(ptr);
        
        //test watchpoint with vmalloc
        ptr=vmalloc(0x100);
        if(!ptr)return -ENOMEM;
        pr_info("test watchpoint with pte %px outtype trace\n",ptr);
        ptr2=(unsigned long)_selftest_get_pte((unsigned long)ptr);
        wp_ret=mt_perf_register_wp_bp(1,ptr2,false,false,0,0,1);
        if(IS_ERR(wp_ret))
        {
            return PTR_ERR(wp_ret);
        }
        *ptr=0x12345678;
        *(ptr+1)=0x55667788;
        vfree(ptr);
        mt_perf_unregister_wp_bp(ptr2);
        
        pr_info("test watchpoint with pte %lx end\n",ptr2);
    }
    return count;    
}
static struct kobj_attribute wp_attr=__ATTR_RW(watchpoint);
static struct kobj_attribute uninstall_attr=__ATTR_WO(uninstall);
static struct kobj_attribute hp_attr=__ATTR_WO(hwbrp);
static struct kobj_attribute notify_usr_attr=__ATTR_RW(notify_usr);
static struct kobj_attribute show_regs_attr=__ATTR_RW(show_regs);
static struct kobj_attribute st_attr=__ATTR_WO(selftest);

static const struct attribute *mt_perf_event_attrs[] = {
    &wp_attr.attr,
    &uninstall_attr.attr,
    &hp_attr.attr,
    &notify_usr_attr.attr,
    &show_regs_attr.attr,
    &st_attr.attr,
    NULL,
};
static int __init mt_perf_event_init(void)
{
    int error=0;

    mt_perf_kobj=kobject_create_and_add("mt_perf", kernel_kobj);
    if(!mt_perf_kobj)
    {
        return -ENOMEM;
    }
    error = sysfs_create_files(mt_perf_kobj, mt_perf_event_attrs);
    if (error)
    {
        kobject_put(mt_perf_kobj);
        return error;
    }
    #ifdef CONFIG_PM
    cpu_pm_register_notifier(&cpu_pm_nb);//care PM_EXIT
    #endif
    #ifdef CONFIG_HOTPLUG_CPU
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0)
    register_cpu_notifier(&cpu_hotplug_nb); //care online
#else
    {
        int state;
        state=cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN,"ap:mtk_perf_event",
        mt_perf_cpu_online,mt_perf_cpu_offline);
        pr_debug("%s:return state=%d(%d)\n",__func__,state,CPUHP_AP_ONLINE_DYN);
    }
#endif
    #endif
    #if defined(CONFIG_KALLSYMS) && defined(CONFIG_MODULES)
    register_module_notifier(&module_event_nb);
    #endif
    return 0;
}
device_initcall(mt_perf_event_init);

/*
set hw breakpoint or hw watchpoint
@param bwp:set watchpoint(1) or breakpoint(0)
@param addr:the virtual address to assert
@param once:the trigger type is once(1) or repeat(0)
@param is_cond_dump:dump stack by condition(set by following parameter)
@param mask:the filter mask
@param value:the filter value
@param outto_trace:if the result including stacktrace and register 
       value store to trace buffer
*/
void * mt_perf_register_wp_bp(u8 bwp,unsigned long addr,bool once,bool is_cond_dump,unsigned long mask,unsigned long value,u8 outto_trace)
{
    void *ret_ptr;
    struct wbp_option option;

    option.addr=addr;
    option.once=once;
    option.dump_cond=is_cond_dump;
    option.cond_mask=mask;
    option.cond_val=value;
    option.out_type=outto_trace?OUTPUT_TO_TRACE:OUTPUT_TO_CONSOLE;
    if(outto_trace)
    {
        _enable_trace_event();
    }
    ret_ptr=_register_wp_bp(bwp,&option);

    if(!ret_ptr)return NULL;
    if(IS_ERR(ret_ptr))
    {
        pr_err("register wp addr %lx fails\n",addr);
        return ret_ptr;
    }
    pr_info("%s %lx registration OK\n",bwp?"watchpoint":"breakpoint",addr);
    return ret_ptr;
}
EXPORT_SYMBOL(mt_perf_register_wp_bp);
void mt_perf_unregister_wp_bp(unsigned long addr)
{
    _unregister_wp_bp(addr);
}
EXPORT_SYMBOL(mt_perf_unregister_wp_bp);

#define MAX_AVAILABLE_BPWP_NUM 5
static __initdata unsigned long  boot_wp_addr[MAX_AVAILABLE_BPWP_NUM]={ULONG_MAX,ULONG_MAX,ULONG_MAX,ULONG_MAX,ULONG_MAX};
static __initdata bool  boot_wp_trig_once[MAX_AVAILABLE_BPWP_NUM];
static __initdata bool  boot_wp_dump_cond[MAX_AVAILABLE_BPWP_NUM];
static __initdata unsigned long  boot_wp_cond_mask[MAX_AVAILABLE_BPWP_NUM];
static __initdata unsigned long  boot_wp_cond_val[MAX_AVAILABLE_BPWP_NUM];
static __initdata u8  boot_wp_out_type[MAX_AVAILABLE_BPWP_NUM];
static __initdata unsigned long  boot_brp_addr[MAX_AVAILABLE_BPWP_NUM]={ULONG_MAX,ULONG_MAX,ULONG_MAX,ULONG_MAX,ULONG_MAX};
static __initdata bool  boot_brp_trig_once[MAX_AVAILABLE_BPWP_NUM];
static __initdata u8  boot_brp_out_type[MAX_AVAILABLE_BPWP_NUM];
static __initdata unsigned int   boot_wp_type[MAX_AVAILABLE_BPWP_NUM]={WP_TYPE_NONE};
static __initdata unsigned char  boot_brp_sym[MAX_AVAILABLE_BPWP_NUM][BRP_SYM_OFFSET_LEN]={{0}};
extern void *tv_reg_base;

static int __init setup_hw_wp(char *str)
{
    struct wbp_option option;
    unsigned int index=0;
    char *hwwp_str;

    if (*str++ != '=' || !*str)
        return -EINVAL;
    while (true) {
        hwwp_str = strsep(&str, ";");

        if (!hwwp_str)
            break;
        if(index>=MAX_AVAILABLE_BPWP_NUM)break;
        pr_debug("hwwp_str=%s\n",hwwp_str);
        if (*hwwp_str)
        {
            if(_parse_hwbp_user_option(hwwp_str,true,&option,NULL))
            {
                return -EINVAL;
            }
            boot_wp_type[index]=option.type;
            boot_wp_addr[index]=option.addr;
            boot_wp_trig_once[index]=option.once;
            boot_wp_dump_cond[index]=option.dump_cond;
            boot_wp_cond_mask[index]=option.cond_mask;
            boot_wp_cond_val[index]=option.cond_val;
            boot_wp_out_type[index]=option.out_type;

            pr_debug("%d:boot_wp_addr=%lx,boot_wp_type=%d\n",index,boot_wp_addr[index],boot_wp_type[index]);
        }
        index++;
        /* Put back the semicolon to allow this to be called again */
        if (str)
            *(str - 1) = ';';       
    }
    return 1;
}
static int __init setup_hw_brp(char *str)
{
    struct wbp_option option;
    char *hwbrp_str;
    unsigned int index=0;
    
    if (*str++ != '=' || !*str)
        return -EINVAL;
    
    while (true) {
        hwbrp_str = strsep(&str, ";");

        if (!hwbrp_str)
            break;
        if(index>=MAX_AVAILABLE_BPWP_NUM)break;
        pr_debug("hwbrp_str=%s\n",hwbrp_str);
        if (*hwbrp_str)
        {
            if(_parse_hwbp_user_option(hwbrp_str,false,&option,boot_brp_sym[index]))
            {
                return -EINVAL;
            }
            boot_brp_addr[index]=option.addr;
            boot_brp_trig_once[index]=option.once;
            boot_brp_out_type[index]=option.out_type;
            pr_debug("%d:boot_brp_addr=%lx boot_brp_out_type=%d boot_brp_sym=%s\n",index,
                    boot_brp_addr[index],boot_brp_out_type[index],boot_brp_sym);           
        }
        index++;
        /* Put back the semicolon to allow this to be called again */
        if (str)
            *(str - 1) = ';';
    }    


    return 1;
}
static int __init setup_notify_usr(char *str)
{
    unsigned long notify_usr_val=0;

    if (*str++ != '=' || !*str)
        return -EINVAL;
    if (kstrtoul(str, 0, &notify_usr_val))
        return -EINVAL;
    _bnotify_usr=notify_usr_val?true:false;
    return 1;
}
static int __init setup_show_regs(char *str)
{
    unsigned long show_regs_val=0;

    if (*str++ != '=' || !*str)
        return -EINVAL;
    if (kstrtoul(str, 0, &show_regs_val))
        return -EINVAL;
    _bshow_regs=show_regs_val?true:false;
    return 1;
}
static int __init init_boot_hw_brp(void)
{
    unsigned long addr=0;
    void *ret_ptr;
    struct wbp_option option;
    unsigned int index=0;
    
    do
    {
        pr_debug("%d:boot_wp_addr=%lx,out_type=%d\n",index,boot_wp_addr[index],boot_wp_out_type[index]);
        if(boot_wp_addr[index]!=ULONG_MAX)
        {
            option.out_type=boot_wp_out_type[index];
            if(boot_wp_type[index]==WP_IO_PHYS)
            {
                //addr=(unsigned long)tv_reg_base+(boot_wp_addr-__pfn_to_phys(vmalloc_to_pfn(tv_reg_base)));
                option.type = boot_wp_type[index];
                option.addr=boot_wp_addr[index];
                option.once=boot_wp_trig_once[index];
                option.dump_cond=boot_wp_dump_cond[index];
                option.cond_mask=boot_wp_cond_mask[index];
                option.cond_val=boot_wp_cond_val[index];
                _register_io_phys_wp(&option);
                index++;
                continue;
            }
            else if(boot_wp_type[index]==WP_DRAM_PHYS)
            {
                addr=(unsigned long)phys_to_virt(boot_wp_addr[index]);
            }
            else
            {
                addr=boot_wp_addr[index];
            }
            option.addr=addr;
            option.once=boot_wp_trig_once[index];
            option.dump_cond=boot_wp_dump_cond[index];
            option.cond_mask=boot_wp_cond_mask[index];
            option.cond_val=boot_wp_cond_val[index];

            ret_ptr=_register_wp_bp(true,&option);
            if(!ret_ptr)goto done;
            if(IS_ERR(ret_ptr))
            {
                pr_err("register wp addr %lx fails\n",addr);
                return PTR_ERR(ret_ptr);
            }
        }
        index++;
    }while(index<MAX_AVAILABLE_BPWP_NUM);
    index=0;
    do
    {
        pr_debug("%d:boot_brp_addr=%lx\n",index,boot_brp_addr[index]);
        if(boot_brp_addr[index]!=ULONG_MAX)
        {
            option.out_type=boot_brp_out_type[index];
            option.once=boot_brp_trig_once[index];
            option.dump_cond=false;
#if defined(CONFIG_KALLSYMS) && defined(CONFIG_MODULES)
            if(strlen(boot_brp_sym[index]))
            {
                boot_brp_addr[index]=_symbol2addr(boot_brp_sym[index],NULL);
                if(!boot_brp_addr[index])
                {
                    pr_err("can not find the symbol %s,search it in module later\n",boot_brp_sym[index]);
                    _add_pending_hwbrp(&pending_brp_list,boot_brp_sym[index],&option);
                    index++;
                    continue;
                    //return -EAGAIN;
                }
            }
#endif
            option.addr=boot_brp_addr[index];

            ret_ptr=_register_wp_bp(false,&option);
            if(!ret_ptr)continue;
            if(IS_ERR(ret_ptr))
            {
                pr_err("register breakpoint addr %lx fails\n",addr);
                index++;
                continue;
                //return PTR_ERR(ret_ptr);
            }
        }
        index++;
    }while(index<MAX_AVAILABLE_BPWP_NUM);   
done:
    return 0;
}
static int __init init_perf_trace(void)
{
    if(bEnableTrace)
    {
		pr_debug("enable mtk_perf_trace again\n");
		_enable_trace_event();
	}
    return 0;
}
__setup("hwwp", setup_hw_wp);
__setup("hwbrp", setup_hw_brp);
__setup("notify_usr", setup_notify_usr);
__setup("showregs", setup_show_regs);
arch_initcall_sync(init_boot_hw_brp);//must after arch_initcall(arch_hw_breakpoint_init);
fs_initcall_sync(init_perf_trace);
#endif
