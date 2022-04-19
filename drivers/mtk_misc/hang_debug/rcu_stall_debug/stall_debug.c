/*
 * Copyright (C) 2008 Steven Rostedt <srostedt@redhat.com>
 *
 */
#define pr_fmt(fmt) "rcu_stall: " fmt
 
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/ftrace.h>
#include <linux/sysfs.h>
#include <linux/smp.h>
#include <linux/kthread.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#ifdef CONFIG_FREEZER
#include <linux/suspend.h>
#endif
#include <linux/sort.h>
#include <linux/irqnr.h>
#include <linux/kernel_stat.h>
#include <linux/slab.h>
#include <linux/slub_def.h>
#include <linux/printk.h>
#include <linux/sysrq.h>
#include <linux/io.h>
#include <asm/cputype.h>

#if defined(CONFIG_MSTAR_CHIP)
#define CPU_DBG_REG_START (0x1f200000+(((0x40<<1)+0x1d00)<<1))
#define CPU_DBG_REG_LEN (0x18*4)
#endif

static void stall_dump(void);
#define ENABLE_SELF_TEST 1
struct irq_count_dist
{
    unsigned int irqcount;
    unsigned int irqno;
};
static unsigned int *  __percpu * percpu_irq_count=NULL;
static struct irq_count_dist * irqcount_dist=NULL;
static u32 *ptr_cpu_dbg=NULL;

static int cmp_irqnum(const void *a, const void *b)
{
    const struct irq_count_dist *pre = a, *next = b;
    int i=0;
    //sort by size
    i=pre->irqcount>next->irqcount?-1:pre->irqcount<next->irqcount;
    return i;
} 
static void dump_irqnum(void)
{
    unsigned int *cpu_irqs;
    int cpu;
    int irqnum=0;
    int tmp_idx=0;
    u64 cap_time=0;
    static DEFINE_PER_CPU(u64,last_ktime);
    
    if(!percpu_irq_count || !irqcount_dist)return;
    for_each_online_cpu(cpu)
    {
        cpu_irqs=(unsigned int *)per_cpu_ptr(percpu_irq_count,cpu);
        cap_time=ktime_to_us(ktime_get());
        for(irqnum=0;(irqnum<nr_irqs) && (irqnum<NR_IRQS);irqnum++)
        {
            if(per_cpu(last_ktime,cpu))
            {
                if(kstat_irqs_cpu(irqnum, cpu)>=cpu_irqs[irqnum])
                {
                    irqcount_dist[irqnum].irqcount=kstat_irqs_cpu(irqnum, cpu)-cpu_irqs[irqnum];
                    irqcount_dist[irqnum].irqno=irqnum;
                }
                else
                {
                    pr_err_ratelimited("irqcount of irq %d at cpu %d is strange:%d vs %d\n",
                    irqnum,cpu,kstat_irqs_cpu(irqnum, cpu),cpu_irqs[irqnum]);
                }   
            }
            cpu_irqs[irqnum]=kstat_irqs_cpu(irqnum, cpu);           
        }
        if(per_cpu(last_ktime,cpu))
        {
            sort(irqcount_dist,irqnum, sizeof(irqcount_dist[0]),cmp_irqnum, NULL);
            pr_err("the top 5 irqcount in %lld us at cpu %d:\n",cap_time-per_cpu(last_ktime,cpu),cpu);
            for(tmp_idx=0;tmp_idx<irqnum && tmp_idx<5;tmp_idx++)
            {
                if(!irqcount_dist[tmp_idx].irqcount)break;
                pr_err("\t%d:irq %d,count %d\n",tmp_idx,irqcount_dist[tmp_idx].irqno,
                        irqcount_dist[tmp_idx].irqcount);
            }
        }
        per_cpu(last_ktime,cpu)=cap_time;
    }
}
static void
rcu_stall_call(unsigned long ip, unsigned long parent_ip,
     struct ftrace_ops *op, struct pt_regs *pt_regs)
{
    stall_dump();
}
static struct ftrace_ops trace_rcu_stall __read_mostly =
{
    .func = rcu_stall_call,
};
static __init int _enable_rcu_stall_hook(void)
{
    int ret;
    ret=ftrace_set_filter(&trace_rcu_stall,"rcu_dump_cpu_stacks",
               strlen("rcu_dump_cpu_stacks"),1);
    if(ret!=0)
    {
        pr_err("ftrace_set_filter fail %d\n",ret);
        return ret;
    }
    ret=register_ftrace_function(&trace_rcu_stall); 
    if(ret!=0)
    {
        pr_err("register_ftrace_function fail %d\n",ret);
        return ret;
    }
    percpu_irq_count=__alloc_percpu(sizeof(unsigned long)*nr_irqs,__alignof__(unsigned int));
    if(percpu_irq_count)
    {
        irqcount_dist=kmalloc(sizeof(irqcount_dist[0])*nr_irqs,GFP_KERNEL);
        if(!irqcount_dist)
        {
            free_percpu(percpu_irq_count);
            percpu_irq_count=NULL;
            pr_err("allocate irqcount_dist fail\n");
        }
    }   
    return 0;
}
#if ENABLE_SELF_TEST
static DEFINE_SPINLOCK(cpu_test_lock);
static int spinlock_thread_fn(void *data)
{
    unsigned long flags;
    
    do
    {
        pr_info("spinlock irqsave test \n");
        spin_lock_irqsave(&cpu_test_lock,flags);
        while(1)
        {
            mdelay(1);
        }
        spin_unlock_irqrestore(&cpu_test_lock,flags);
    }while(0);
    
    return 0;
    
}
static ssize_t stall_debug_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t count)
{
    struct task_struct *tsk;
    int cpu;
    if(kstrtoint(buf,0,&cpu))
        return -EINVAL;
    if(!cpu_online(cpu))
        return -EINVAL;
    
    tsk = kthread_create_on_cpu(spinlock_thread_fn, NULL,cpu,
                    "test_spin");
    if (IS_ERR(tsk)) {
        return PTR_ERR(tsk);
    }
    pr_info("create test_spin thread on cpu %d\n",cpu);
    wake_up_process(tsk);
    return count;
}
static ssize_t stall_debug_show(struct kobject *kobj,struct kobj_attribute *pattr, char *buf)
{
    stall_dump();
    return 0;
}
static struct kobject *AT_kobj=NULL;
static struct kobj_attribute AT_attr=__ATTR_RW(stall_debug);
static const struct attribute *AT_attrs[] = {
    &AT_attr.attr,
    NULL,
};
#endif
static __init int stall_debug_init(void)
{
    int error=0;
#if ENABLE_SELF_TEST
    
    AT_kobj=kobject_create_and_add("stall_debug", kernel_kobj);
    if(!AT_kobj)
    {
        return -ENOMEM;
    }
    error = sysfs_create_files(AT_kobj, AT_attrs);
    if (error)
    {
        kobject_put(AT_kobj);
        return error;
    }
#endif
#ifdef CPU_DBG_REG_START
    ptr_cpu_dbg=ioremap(CPU_DBG_REG_START,CPU_DBG_REG_LEN);
    if(!ptr_cpu_dbg)return -EINVAL;
#endif	
    error=_enable_rcu_stall_hook();
    if (error)
    {
#if ENABLE_SELF_TEST
        kobject_put(AT_kobj);
#endif
#ifdef CPU_DBG_REG_START
        iounmap(ptr_cpu_dbg);
		ptr_cpu_dbg=NULL;
#endif		
        return error;
    }

    return error;
}

device_initcall(stall_debug_init);
void show_cpu_dbgpcsr(void)
{
	unsigned long pc[NR_CPUS];
	int i;
	u32 cpu_part;
	
	if(ptr_cpu_dbg==NULL)return;
	
	cpu_part=(read_cpuid_id()&0xfff0)>>4;
    if(cpu_part==0xd05)//ca55
		writew(0x3<<14,ptr_cpu_dbg+0x17);
	
	for_each_possible_cpu (i) {
#if BITS_PER_LONG==64
        pc[i]=readw(ptr_cpu_dbg+i);;
		pc[i]<<=16;
		pc[i]|=readw(ptr_cpu_dbg+6+(i*3));
		pc[i]<<=16;
		pc[i]|=readw(ptr_cpu_dbg+5+(i*3));
		pc[i]<<=16;
        pc[i]|=readw(ptr_cpu_dbg+4+(i*3));;
#else
        pc[i]=readw(ptr_cpu_dbg+5+(i*3));
		pc[i]<<=16;
        pc[i]|=readw(ptr_cpu_dbg+4+(i*3));
#endif	
	}
	for_each_possible_cpu (i) 
		pr_err("CPU%d PC: 0x%lx(%pS)\n",i,pc[i],pc[i]);
	
}
static void stall_dump(void)
{
    int i=0;
    console_verbose();
    while(i<10)
    {
        show_cpu_dbgpcsr(); 
        i++;
    }
    dump_irqnum();
    handle_sysrq('l');    
}
