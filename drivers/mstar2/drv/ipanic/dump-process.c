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

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @file   dump-process.c
/// @brief  dump process info
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/sched/task_stack.h>
#include "ipanic.h"

extern union IPANIC_MEM  ipanic_memory;
static void _LOG(const char *fmt, ...)
{
    char buf[256];
    int len=0;
    va_list ap;

    va_start(ap, fmt);
    len = strlen(ipanic_memory.NativeInfo);
    if ((len+sizeof(buf)) < MAX_NATIVEINFO)
        vsnprintf(&ipanic_memory.NativeInfo[len], sizeof(buf), fmt, ap);
    va_end(ap);
}

static void dump_registers(struct pt_regs *r)
{
#ifdef CONFIG_64BIT
    int i, top_reg;
    u64 lr, sp;
#endif

    if(r == NULL)
        return;

#ifdef CONFIG_64BIT
    if (compat_user_mode(r)) {
        lr = r->compat_lr;
        sp = r->compat_sp;
        top_reg = 12;
    } else {
        lr = r->regs[30];
        sp = r->sp;
        top_reg = 29;
    }

    _LOG(" pc/lr/sp 0x%016llx/0x%016llx/0x%016llx\n", instruction_pointer(r), lr, sp);
    for (i = top_reg; i >= 0; i--)
    {
        _LOG("x%-2d: %016llx    ", i, r->regs[i]);
    }
    _LOG("\n");
#else
    _LOG(" pc/lr/sp 0x%08x/0x%08x/0x%08x\n", r->ARM_pc, r->ARM_lr, r->ARM_sp);
    _LOG(" r0 %08x r1 %08x r2 %08x r3 %08x\n",
     r->ARM_r0, r->ARM_r1, r->ARM_r2, r->ARM_r3);
    _LOG(" r4 %08x r5 %08x r6 %08x r7 %08x\n",
     r->ARM_r4, r->ARM_r5, r->ARM_r6, r->ARM_r7);
    _LOG(" r8 %08x r9 %08x 10 %08x fp %08x\n",
     r->ARM_r8, r->ARM_r9, r->ARM_r10, r->ARM_fp);
    _LOG(" ip %08x sp %08x lr %08x pc %08x cpsr %08x\n",
     r->ARM_ip, r->ARM_sp, r->ARM_lr, r->ARM_pc, r->ARM_cpsr);
#endif
}

int DumpNativeInfo(void)
{
    struct task_struct *current_task;
    struct pt_regs *user_ret;
    struct vm_area_struct *vma;
    unsigned long userstack_start = 0;
    unsigned long userstack_end = 0;
    int mapcount = 0;
    struct file *file;
    int flags;
    struct mm_struct *mm;
    int i=0;

    memset(ipanic_memory.NativeInfo, 0, sizeof(ipanic_memory.NativeInfo));
    //memset(User_Stack, 0, sizeof(User_Stack));

    current_task = get_current();
    //current_task = find_task_by_vpid(982);
    user_ret = task_pt_regs(current_task);

    if (!user_mode(user_ret))
        return 0;

    dump_registers(user_ret);
#ifdef CONFIG_64BIT
    if (compat_user_mode(user_ret))
        userstack_start = user_ret->compat_sp;
    else
        userstack_start = user_ret->sp;
#else
    userstack_start = (unsigned long)user_ret->ARM_sp;
#endif

    if(current_task->mm == NULL)
       return 0;

    vma = current_task->mm->mmap;
    while (vma && (mapcount < current_task->mm->map_count)) {
        file = vma->vm_file;
        flags = vma->vm_flags;
        if(file)
        {
            _LOG("%08lx-%08lx %c%c%c%c %s\n",vma->vm_start,vma->vm_end,
             flags & VM_READ ? 'r' : '-',
             flags & VM_WRITE ? 'w' : '-',
             flags & VM_EXEC ? 'x' : '-',
             flags & VM_MAYSHARE ? 's' : 'p',file->f_path.dentry->d_iname);

             //IPANIC_DPRINTK("file %s\n", file->f_path.dentry->d_iname);
        }
        else
        {
            const char *name = arch_vma_name(vma);
            mm = vma->vm_mm;
            if(!name)
            {
                if(mm)
                {
                    if(vma->vm_start <= mm->start_brk && vma->vm_end >= mm->brk)
                    {
                        name = "[heap]";
                    }
                    else if(vma->vm_start <= mm->start_stack && vma->vm_end >= mm->start_stack)
                    {
                        name = "[stack]";
                    }
                }
                else
                {
                    name = "[vdso]";
                }
            }

            //if (name)
            {
                _LOG("%08lx-%08lx %c%c%c%c %s\n",vma->vm_start,vma->vm_end,
                 flags & VM_READ ? 'r' : '-',
                 flags & VM_WRITE ? 'w' : '-',
                 flags & VM_EXEC ? 'x' : '-',
                 flags & VM_MAYSHARE ? 's' : 'p',name);
            }
        }
        vma = vma->vm_next;
        mapcount++;

    }

    vma = current_task->mm->mmap;
    while (vma != NULL)
    {
        if (vma->vm_start <= userstack_start
         && vma->vm_end >= userstack_start)
        {
            userstack_end = vma->vm_end;
            break;
        }
        vma = vma->vm_next;
        if (vma == current_task->mm->mmap)
        {
            break;
        }
    }

    if (userstack_end == 0) {
        IPANIC_DPRINTK("Dump native stack failed:\n");
        return 0;
    }

    _LOG ("Dump stack range (0x%08x:0x%08x)\n", userstack_start, userstack_end);

    //length = ((userstack_end-userstack_start) < (sizeof(User_Stack) - 1)) ? (userstack_end-userstack_start) : (sizeof(User_Stack) - 1);
    //ret=copy_from_user((void *)(User_Stack), (const void __user *)( userstack_start), length);
    //IPANIC_DPRINTK("copy_from_user ret(0x%08x),len:%x\n",ret,length);
    i=0;
    while (userstack_start < userstack_end) {
        _LOG ("0x%08x: 0x%08x\n", userstack_start, *((unsigned int *)userstack_start));
        //printk(KERN_ERR "0x%08x: 0x%08x\n", userstack_start, *((unsigned int *)userstack_start));
        userstack_start+=4;  // 4 bytes
        i++;
    }
    _LOG ("end dump native stack:\n");
    return 0;
}
