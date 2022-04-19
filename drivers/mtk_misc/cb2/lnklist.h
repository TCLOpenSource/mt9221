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

/*-----------------------------------------------------------------------------
 * $RCSfile: lnklist.h,v $
 * $Revision: #1 $
 * $Date: 2015/04/15 $
 * $Author: dtvbm11 $
 *
 * Description: 
 *         This header file contains single and double link list macros.
 *---------------------------------------------------------------------------*/

#ifndef _LNKLIST_H_
#define _LNKLIST_H_

/*-----------------------------------------------------------------------------
                    include files
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
                    macros, defines, typedefs, enums
-----------------------------------------------------------------------------*/

/*
 *  Double link list
 *
 *          pt_head
 *         ---------------|
 *         |              v
 *  -----------           -------------           -------------
 *  |         |           |           |  pt_next  |           |  pt_next
 *  |         |           |  DLIST_   |---------->|  DLIST_   |-----> NULL
 *  | DLIST_T |           |  ENTRY_T  |           |  ENTRY_T  |
 *  |         | NULL <----|           |<----------|           |
 *  |         |  pt_prev  |           |  pt_prev  |           |
 *  -----------           -------------           -------------
 *       |                                        ^
 *       -----------------------------------------|
 *          pt_tail
 *
 */
#define DLIST_T(type)           \
    struct {                    \
        struct type *pt_head;   \
        struct type *pt_tail;   \
    }
    
#define DLIST_ENTRY_T(type)     \
    struct {                    \
        struct type *pt_prev;   \
        struct type *pt_next;   \
    }
    
#define DLIST_HEAD(q) ((q)->pt_head)

#define DLIST_TAIL(q) ((q)->pt_tail)

#define DLIST_IS_EMPTY(q) ((q)->pt_head == NULL)

#define DLIST_NEXT(ent, field)  ((ent)->field.pt_next)

#define DLIST_PREV(ent, field)  ((ent)->field.pt_prev)

#define DLIST_INIT(q)                       \
    do                                      \
    {                                       \
        (q)->pt_head = (q)->pt_tail = NULL; \
    } while(0)
 
#define DLIST_FOR_EACH(var, q, field)       \
    for ((var) = (q)->pt_head;              \
         (var);                             \
         (var) = (var)->field.pt_next)
         
#define DLIST_FOR_EACH_BACKWARD(var, q, field)  \
    for ((var) = (q)->pt_tail;                  \
         (var);                                 \
         (var) = (var)->field.pt_prev)

#define DLIST_INSERT_HEAD(ent, q, field)    \
    do                                      \
    {                                       \
        (ent)->field.pt_prev = NULL;                        \
        if (((ent)->field.pt_next = (q)->pt_head) == NULL)  \
        {                                                   \
            (q)->pt_tail = (ent);                           \
        }                                                   \
        else                                                \
        {                                                   \
            ((q)->pt_head)->field.pt_prev = (ent);          \
        }                                                   \
        (q)->pt_head = (ent);                               \
    } while(0)
    
#define DLIST_INSERT_TAIL(ent, q, field)                    \
    do                                                      \
    {                                                       \
        (ent)->field.pt_next = NULL;                        \
        if (((ent)->field.pt_prev = (q)->pt_tail) == NULL)  \
        {                                                   \
            (q)->pt_head = (ent);                           \
        }                                                   \
        else                                                \
        {                                                   \
            ((q)->pt_tail)->field.pt_next = (ent);          \
        }                                                   \
        (q)->pt_tail = (ent);                               \
    } while(0)

#define DLIST_INSERT_BEFORE(new, old, q, field)             \
    do                                                      \
    {                                                       \
        (new)->field.pt_next = (old);                       \
        if (((new)->field.pt_prev = (old)->field.pt_prev) == NULL)  \
        {                                                   \
            (q)->pt_head = (new);                           \
        }                                                   \
        else                                                \
        {                                                   \
            ((old)->field.pt_prev)->field.pt_next = (new);  \
        }                                                   \
        (old)->field.pt_prev = (new);                       \
    } while(0)


#define DLIST_INSERT_AFTER(new, old, q, field)              \
    do                                                      \
    {                                                       \
        (new)->field.pt_prev = (old);                       \
        if (((new)->field.pt_next = (old)->field.pt_next) == NULL)  \
        {                                                   \
            (q)->pt_tail = (new);                           \
        }                                                   \
        else                                                \
        {                                                   \
            ((old)->field.pt_next)->field.pt_prev = (new);  \
        }                                                   \
        (old)->field.pt_next = (new);                       \
    } while(0)
    
#define DLIST_REMOVE(ent, q, field)                         \
    do                                                      \
    {                                                       \
        if ((q)->pt_tail == (ent))                          \
        {                                                   \
            (q)->pt_tail = (ent)->field.pt_prev;            \
        }                                                   \
        else                                                \
        {                                                   \
            ((ent)->field.pt_next)->field.pt_prev = (ent)->field.pt_prev;   \
        }                                                   \
        if ((q)->pt_head == (ent))                          \
        {                                                   \
            (q)->pt_head = (ent)->field.pt_next;            \
        }                                                   \
        else                                                \
        {                                                   \
            ((ent)->field.pt_prev)->field.pt_next = (ent)->field.pt_next;   \
        }                                                   \
        (ent)->field.pt_next = NULL;                        \
        (ent)->field.pt_prev = NULL;                        \
    } while(0)

 
/*
 *  Single link list
 *
 *              pt_head
 *            ------------|
 *            |^          v
 *  -----------|          -------------           -------------
 *  |         ||          |           |  pt_next  |           |  pt_next
 *  |         ||          |  SLIST_   |---------->|  SLIST_   |-----> NULL
 *  | SLIST_T ||          |  ENTRY_T  |^          |  ENTRY_T  |
 *  |         |-----------|           |-----------|           |
 *  |         |  pt_prev  |           |  pt_prev  |           |
 *  -----------           -------------           -------------
 *
 */
#define SLIST_T(type)           \
    struct {                    \
        struct type *pt_head;   \
    }

#define SLIST_ENTRY_T(type)     \
    struct {                    \
        struct type *pt_next;   \
        struct type **pt_prev;  /* address of previous pt_next */ \
    }
    
#define SLIST_FIRST(list)       ((list)->pt_head)

#define SLIST_IS_EMPTY(list)    ((list)->pt_head == NULL)

#define	SLIST_NEXT(ent, field)	((ent)->field.pt_next)

#define SLIST_INIT(list)            \
    {                               \
        ((list)->pt_head = NULL);   \
    }

#define SLIST_FOR_EACH(var, list, field)    \
    for ((var) = (list)->pt_head;           \
         (var);                             \
         (var) = (var)->field.pt_next)
         
#define SLIST_INSERT_HEAD(new, list, field)                             \
    do                                                                  \
    {                                                                   \
        if (((new)->field.pt_next = (list)->pt_head) != NULL)           \
        {                                                               \
            ((list)->pt_head)->field.pt_prev = &((new)->field.pt_next); \
        }                                                               \
        (list)->pt_head = (new);                                        \
        (new)->field.pt_prev = &((list)->pt_head);                      \
    } while (0)
    
#define SLIST_INSERT_AFTER(new, old, field)                         \
    do                                                              \
    {                                                               \
        if (((new)->field.pt_next = (old)->field.pt_next) != NULL)  \
        {                                                           \
            ((old)->field.pt_next)->field.pt_prev = &((new)->field.pt_next); \
        }                                                           \
        (old)->field.pt_next = (new);                               \
        (new)->field.pt_prev = &((old)->field.pt_next);             \
    } while (0)
    

#define SLIST_INSERT_BEFORE(new, old, field)                        \
    do                                                              \
    {                                                               \
        (new)->field.pt_next = (old);                               \
        (new)->field.pt_prev = (old)->field.pt_prev;                \
        *((old)->field.pt_prev) = (new);                            \
        (old)->field.pt_prev = &((new)->field.pt_next);             \
    } while (0)


#define SLIST_REMOVE(ent, field)                                            \
    do                                                                      \
    {                                                                       \
        *((ent)->field.pt_prev) = (ent)->field.pt_next;                     \
        if ((ent)->field.pt_next != NULL)                                   \
        {                                                                   \
            ((ent)->field.pt_next)->field.pt_prev = (ent)->field.pt_prev;   \
        }                                                                   \
        (ent)->field.pt_next = NULL;                                        \
        (ent)->field.pt_prev = NULL;                                        \
    } while (0)


#endif /* _LNKLIST_H_ */
