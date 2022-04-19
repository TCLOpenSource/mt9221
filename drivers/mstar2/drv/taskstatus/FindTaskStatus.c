#include <linux/version.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/dma-contiguous.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/freezer.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/swap.h>
#include <linux/ptrace.h>
#include <linux/dma-contiguous.h>
#include <linux/version.h>
#include <mstar/mpatch_macro.h>

#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif
#include <linux/vmalloc.h>

#include "mdrv_types.h"
#include "mst_devid.h"
#include "mdrv_system.h"

#if defined(CONFIG_MP_FindTaskStatus)
#include "mdrv_FindTaskStatus.h"
#endif
int iTASK_PID = 0xFFFF;
int iTASK_PID_OOM = 0xFFFF;
char TaskStatus_name_list[LIST_MAX_LENGTH]="com.netflix.ninja:";
extern struct All_Task_list *FindTask_list;

/*
echo > :app_thread_name > proc/FindTask_LIST/TaskStatus_list
ex: app_thread_name = "com.johnny" echo ":com.johnny" > proc/FindTask_LIST/TaskStatus_list"
*/
#define DVFS_SCALLING_LIST_DIR "FindTask_LIST"

DEFINE_MUTEX(TaskStatus_list_lock);
static struct proc_dir_entry *Findtask_list_dir;
int Get_Target_pid(void)
{
	return iTASK_PID;
}
EXPORT_SYMBOL(Get_Target_pid);
int Set_Target_pid(int pid)
{
	iTASK_PID=pid;
	pr_err("Set_Target_pid :(tgid,iTASK_PID_OOM)=(%d,%d)\n",iTASK_PID,iTASK_PID_OOM);
	return 0;
}
EXPORT_SYMBOL(Set_Target_pid);
int Delete_Target_pid(void)
{
	iTASK_PID=0xFFFF;
	iTASK_PID_OOM=0xFFFF;
	pr_err("Delete_Target_pid :(tgid,iTASK_PID_OOM)=(%d,%d)\n",iTASK_PID,iTASK_PID_OOM);
	return 0;
}
EXPORT_SYMBOL(Delete_Target_pid);
int Set_Target_pid_OOM_adj(int oom_adj)
{
	iTASK_PID_OOM=oom_adj;
	pr_err("Set_Target_pid_OOM_adj :(tgid,iTASK_PID_OOM)=(%d,%d) \n",iTASK_PID,iTASK_PID_OOM);
	return 0;
}
EXPORT_SYMBOL(Set_Target_pid_OOM_adj);
int Get_Target_pid_OOM_adj(void)
{
	pr_err("Get_Target_pid_OOM_adj :(tgid,iTASK_PID_OOM)=(%d,%d) \n",iTASK_PID,iTASK_PID_OOM);
	return iTASK_PID_OOM;
}
EXPORT_SYMBOL(Get_Target_pid_OOM_adj);
static ssize_t FindTask_list_write(struct file *file, const char __user *buffer,
                                 size_t count, loff_t *ppos){
    char local_buf[LIST_MAX_LENGTH]= {0};
    if (!count)
        return count;
    if (count >= LIST_MAX_LENGTH)
        count = LIST_MAX_LENGTH - 1;

    if (copy_from_user(local_buf, buffer, count))
        return -EFAULT;

    local_buf[count] = '\0';

    return count;
}
static ssize_t FindTask_list_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
    int pid=Get_Target_pid();
    int oom=Get_Target_pid_OOM_adj();
    pr_err("TaskStatus_list : (tgid,stauts)=(%d,%d) \n",pid,oom);
    return 0;
}

static int FindTask_list_open(struct inode *inode, struct file *file){
    return 0;
}

static const struct file_operations FindTask_list_fops = {
    .owner = THIS_MODULE,
    .write = FindTask_list_write,
    .read = FindTask_list_read,
    .open = FindTask_list_open,
    .llseek = seq_lseek,
};

MSYSTEM_STATIC int __init mod_FindTask_init(void)
{
    struct proc_dir_entry *size;
    struct proc_dir_entry *timeout;
    Findtask_list_dir = proc_mkdir(DVFS_SCALLING_LIST_DIR, NULL);

    if (NULL==Findtask_list_dir) {
        pr_err("Create dir /proc/%s error!\n",DVFS_SCALLING_LIST_DIR);
    }
    timeout = proc_create("Task_list", 0644, Findtask_list_dir, &FindTask_list_fops);

	return 0;
}
bool found_Special_Task(char* searchSource, char* pattern,int len)
{
    int result = false;
    char * pch = NULL;
#define SEPARATE_SYMBOL ":"
    char tmpbuf[LIST_MAX_LENGTH] = {0};
    char* base = tmpbuf;
    if (len > LIST_MAX_LENGTH)
        len = LIST_MAX_LENGTH;
    strncpy(tmpbuf, searchSource, len);
    while ((pch = strsep(&base, SEPARATE_SYMBOL)) != NULL) {
        if (strstr(pch,pattern)) {
            result = true;
            break;
        }
    }

    return result;
}
EXPORT_SYMBOL(found_Special_Task);
MSYSTEM_STATIC void __exit mod_FindTask_exit(void)
{
}
module_init(mod_FindTask_init);
module_exit(mod_FindTask_exit);
