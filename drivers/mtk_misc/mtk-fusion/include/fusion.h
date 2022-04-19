/*
   (c) Copyright 2002-2011  The world wide DirectFB Open Source Community (directfb.org)
   (c) Copyright 2002-2004  Convergence (integrated media) GmbH

   All rights reserved.

   Written by Denis Oliver Kropp <dok@directfb.org>

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version
   2 of the License, or (at your option) any later version.
*/

#ifndef __LINUX__FUSION_H__
#define __LINUX__FUSION_H__

#include <asm/ioctl.h>
#include <asm-generic/ioctl.h>

#include <linux/limits.h>

#define CC_64BIT_SUPPORT 1
/* Fusion supports all API versions up to this version */
#define FUSION_API_MAJOR_PROVIDED 8
#define FUSION_API_MINOR_PROVIDED 7
#define FUSION_API_MICRO_PROVIDED 0

/*
 * Fusion Kernel Device API Version
 * Default behaviour: 3.3 -> DirectFB 1.0.x
 */
#ifndef FUSION_API_MAJOR
#undef FUSION_API_MAJOR
#undef FUSION_API_MINOR
#undef FUSION_API_MICRO
#define FUSION_API_MAJOR      3
#define FUSION_API_MINOR      3
#define FUSION_API_MICRO      0
#endif

/*
 * The Fusion ID is a unique identifier for one process consisting of threads.
 */
#if defined(CC_64BIT_SUPPORT)
typedef unsigned long long FusionID;
#else
typedef unsigned long long FusionID;
#endif

#define FUSION_ID_MASTER      1              /* This is the fusion id of the master (first process). */

/*
 * Entering a world
 * API version (major/minor): master determines API of this world. slave has to follow.
 * supported are API 3.x for DirectFB 1.0.x, API 4.x for DirectFB 1.1.x, and API 8.x for DirectFB 1.2.x and beyond.
 */
#if defined(CC_64BIT_SUPPORT)
typedef struct {
     struct {
          int major;
          int minor;
     } api;

	union{
	 FusionID                 fusion_id;     /* Returns the fusion id of the entering process. */
	 unsigned long long 	u8padding;
	};
	 
     int                      secure;
} FusionEnter;
#else
typedef struct {
     struct {
          int major;
          int minor;
     } api;

	 FusionID                 fusion_id;     /* Returns the fusion id of the entering process. */
	 

     int                      secure;
} FusionEnter;

#endif
/*
 * Forking in world
 */
#ifdef CC_64BIT_SUPPORT
typedef struct {
	union{
     FusionID                 fusion_id;     /* Returns the fusion id of the new (forked) fusionee. */
	 unsigned long long u8padding;
	};
} FusionFork;
#else
typedef struct {
     FusionID                 fusion_id;     /* Returns the fusion id of the new (forked) fusionee. */
} FusionFork;
#endif

/*
 * Sending a message
 */

#ifdef CC_64BIT_SUPPORT
typedef struct {
	union {
     FusionID                 fusion_id;     /* recipient */
	 unsigned long long u8padding;
	};

     int                      msg_id;        /* optional message identifier */
     int                      msg_channel;   /* optional channel number */
     int                      msg_size;      /* message size, must be greater than zero */
	 int 					  u4padding;
	 union {
     const void              *msg_data;      /* message data, must not be NULL */
	 unsigned int compat_value;
	 unsigned long long u8msg_data;
	 };
} FusionSendMessage;

#else
typedef struct {
     FusionID                 fusion_id;     /* recipient */

     int                      msg_id;        /* optional message identifier */
     int                      msg_channel;   /* optional channel number */
     int                      msg_size;      /* message size, must be greater than zero */
     const void              *msg_data;      /* message data, must not be NULL */
} FusionSendMessage;
#endif
/*
 * Receiving a message
 */
typedef enum {
     FMT_SEND,                               /* msg_id is an optional custom id */
     FMT_CALL,                               /* msg_id is the call id */
     FMT_REACTOR,                            /* msg_id is the reactor id */
     FMT_SHMPOOL,                            /* msg_id is the pool id */
     FMT_CALL3,                              /* msg_id is the call id */
     FMT_LEAVE                               /* FusionID in message data */
} FusionMessageType;

typedef struct {
     FusionMessageType        msg_type;      /* type (origin) of message */

     int                      msg_id;        /* message id (custom id or call/reactor/pool id) */
     int                      msg_size;      /* size of the following message data */
     int                      msg_channel;   /* optional or reactor channel */

     /* message data follows */
} FusionReadMessage;

/*
 * Dispatching a message via a reactor
 */

#ifdef CC_64BIT_SUPPORT
typedef struct {
     int                      reactor_id;    /* id of target reactor */
     int                      channel;       /* optional reactor channel (0-1023) */
     int                      self;          /* send to ourself if attached */

     int                      msg_size;      /* message size, must be greater than zero */
	 union {
     const void              *msg_data;      /* message data, must not be NULL */
	 unsigned int compat_value;
	 unsigned long long u8msg_data;
	 };
} FusionReactorDispatch;
#else
typedef struct {
     int                      reactor_id;    /* id of target reactor */
     int                      channel;       /* optional reactor channel (0-1023) */
     int                      self;          /* send to ourself if attached */

     int                      msg_size;      /* message size, must be greater than zero */
     const void              *msg_data;      /* message data, must not be NULL */
} FusionReactorDispatch;
#endif

/*
 * Attaching to a reactor
 */
typedef struct {
     int                      reactor_id;
     int                      channel;
} FusionReactorAttach;

/*
 * Detaching from a reactor
 */
typedef struct {
     int                      reactor_id;
     int                      channel;
} FusionReactorDetach;

/*
 * Registering a dispatch callback
 *
 * The call_arg will be set to the channel number
 */

#ifdef CC_64BIT_SUPPORT
typedef struct {
     int                      reactor_id;

     int                      call_id;       /* id of the call to execute when a message has been
                                                processed by all recipients of the dispatch */
	union {
     int                     *call_ptr;      /* optional call parameter, e.g. the pointer of a user
                                                space resource associated with that reference */
	 unsigned int compat_value;
	 unsigned long long u8call_ptr;
     };
} FusionReactorSetCallback;
#else
typedef struct {
     int                      reactor_id;

     int                      call_id;       /* id of the call to execute when a message has been
                                                processed by all recipients of the dispatch */
     int                     *call_ptr;      /* optional call parameter, e.g. the pointer of a user
                                                space resource associated with that reference */
} FusionReactorSetCallback;
#endif

/*
 * Calling (synchronous RPC)
 */
#ifdef CC_64BIT_SUPPORT
typedef struct {
     int                      call_id;       /* new call id returned */
	 int 	u4padding;
	union {
     void                    *handler;       /* function pointer of handler to install */
	 unsigned int handler_compat_value;
	 unsigned long long u8handler;
	};
	union {
     void                    *ctx;           /* optional handler context */
	 unsigned int ctx_compat_value;
	 unsigned long long u8ctx;
	};
} FusionCallNew;
#else 
typedef struct {
     int                      call_id;       /* new call id returned */

     void                    *handler;       /* function pointer of handler to install */
     void                    *ctx;           /* optional handler context */
} FusionCallNew;
#endif

typedef enum {
     FCEF_NONE                = 0x00000000,
     FCEF_ONEWAY              = 0x00000001,
     FCEF_QUEUE               = 0x00000002,
     FCEF_FOLLOW              = 0x00000004,
     FCEF_ERROR               = 0x00000008,
     FCEF_RESUMABLE           = 0x00000010,
     FCEF_DONE                = 0x00000020,
     FCEF_ALL                 = 0x0000003f
} FusionCallExecFlags;

#ifdef CC_64BIT_SUPPORT
typedef struct {
     int                      ret_val;       /* return value of the call */

     int                      call_id;       /* id of the requested call, each call has a fixed owner */

     int                      call_arg;      /* optional int argument */
	 int 	u4padding;
	 union {
     void                    *call_ptr;      /* optional pointer argument (shared memory) */
	 unsigned int compat_value;
	 unsigned long long       u8call_ptr;
	 }; 

     FusionCallExecFlags      flags;         /* execution flags */
} FusionCallExecute;

typedef struct {
     int                      ret_val;       /* return value of the call */

     int                      call_id;       /* id of the requested call, each call has a fixed owner */

     int                      call_arg;      /* optional int argument */
	 int 					padding;
	 union {
     void                    *ptr;           /* optional argument, data will be appended (copy) to message and passed to callee */
	 unsigned int compat_value;
	 unsigned long long u8ptr;
	 };
     unsigned int             length;

     FusionCallExecFlags      flags;         /* execution flags */
} FusionCallExecute2;

typedef struct {
     int                      call_id;       /* id of the requested call, each call has a fixed owner */

     int                      call_arg;      /* optional int argument */
	 union {
     void                    *ptr;           /* optional argument, data will be appended (copy) to message and passed to callee */
	 unsigned int ptr_compat_value;
	 unsigned long long u8ptr;
	 };
     unsigned int             length;
	 unsigned int padding;

	union {
     void                    *ret_ptr;       /* pointer to return buffer */
	 unsigned int ret_compat_value;
	 unsigned long long u8ret_ptr;
	};
     unsigned int             ret_length;    /* maximum (input) and actual (output) length of return buffer */

     FusionCallExecFlags      flags;         /* execution flags */
} FusionCallExecute3;

#else
typedef struct {
     int                      ret_val;       /* return value of the call */

     int                      call_id;       /* id of the requested call, each call has a fixed owner */

     int                      call_arg;      /* optional int argument */
     void                    *call_ptr;      /* optional pointer argument (shared memory) */

     FusionCallExecFlags      flags;         /* execution flags */
} FusionCallExecute;


typedef struct {
     int                      ret_val;       /* return value of the call */

     int                      call_id;       /* id of the requested call, each call has a fixed owner */

     int                      call_arg;      /* optional int argument */
     void                    *ptr;           /* optional argument, data will be appended (copy) to message and passed to callee */
     unsigned int             length;

     FusionCallExecFlags      flags;         /* execution flags */
} FusionCallExecute2;

typedef struct {
     int                      call_id;       /* id of the requested call, each call has a fixed owner */

     int                      call_arg;      /* optional int argument */
     void                    *ptr;           /* optional argument, data will be appended (copy) to message and passed to callee */
     unsigned int             length;

     void                    *ret_ptr;       /* pointer to return buffer */
     unsigned int             ret_length;    /* maximum (input) and actual (output) length of return buffer */

     FusionCallExecFlags      flags;         /* execution flags */
} FusionCallExecute3;
#endif

typedef struct {
     int                      call_id;       /* id of currently executing call */

     int                      val;           /* value to return */

     unsigned int             serial;
} FusionCallReturn;

#ifdef CC_64BIT_SUPPORT
typedef struct {
     int                      call_id;       /* id of currently executing call */

     unsigned int             serial;

	union {
     void                    *ptr;           /* pointer to return buffer */
	 unsigned int compat_value;
	 unsigned long long u8ptr;
	};
     unsigned int             length;        /* length of return buffer */
} FusionCallReturn3;
#else
typedef struct {
     int                      call_id;       /* id of currently executing call */

     unsigned int             serial;

     void                    *ptr;           /* pointer to return buffer */
     unsigned int             length;        /* length of return buffer */
} FusionCallReturn3;
#endif

#ifdef CC_64BIT_SUPPORT
typedef struct {
	union {
     void                    *handler;       /* function pointer of handler to call */
	 unsigned int compat_handler;
	 unsigned long long u8handler;
	};
	union {
     void                    *ctx;           /* optional handler context */
	 unsigned int compat_ctx;
	 unsigned long long u8ctx;
	};

     int                      caller;        /* fusion id of the caller or zero if called from Fusion */
     int                      call_arg;      /* optional call parameter */
	 union{
     void                    *call_ptr;      /* optional call parameter */
	 unsigned int compat_call_ptr;
	 unsigned long long u8call_ptr;
	 };

     unsigned int             serial;        /* serial number of call, used for return, zero if nothing shall be returned */
} FusionCallMessage;

typedef struct {
	union {
     void                    *handler;       /* function pointer of handler to call */
	 unsigned int compat_handler;
	 unsigned long long u8handler;
	};
	union {
     void                    *ctx;           /* optional handler context */
	 unsigned int compat_ctx;
	 unsigned long long u8ctx;
	};

     int                      caller;        /* fusion id of the caller or zero if called from Fusion */
     int                      call_arg;      /* optional call parameter */
	 union {
     void                    *call_ptr;      /* optional data */
	 unsigned int compat_call_ptr;
	 unsigned long long u8call_ptr;
	 };
     unsigned int             call_length;   /* length of data */
     unsigned int             ret_length;    /* maximum length of return data */

     unsigned int             serial;        /* serial number of call, used for return, zero if nothing shall be returned */
} FusionCallMessage3;

#else
typedef struct {
     void                    *handler;       /* function pointer of handler to call */
     void                    *ctx;           /* optional handler context */

     int                      caller;        /* fusion id of the caller or zero if called from Fusion */
     int                      call_arg;      /* optional call parameter */
     void                    *call_ptr;      /* optional call parameter */

     unsigned int             serial;        /* serial number of call, used for return, zero if nothing shall be returned */
} FusionCallMessage;

typedef struct {
     void                    *handler;       /* function pointer of handler to call */
     void                    *ctx;           /* optional handler context */

     int                      caller;        /* fusion id of the caller or zero if called from Fusion */
     int                      call_arg;      /* optional call parameter */
     void                    *call_ptr;      /* optional data */
     unsigned int             call_length;   /* length of data */
     unsigned int             ret_length;    /* maximum length of return data */

     unsigned int             serial;        /* serial number of call, used for return, zero if nothing shall be returned */
} FusionCallMessage3;
#endif
/*
 * Watching a reference
 *
 * This information is needed to have a specific call being executed if the
 * reference count reaches zero. Currently one watch per reference is allowed.
 *
 * The call is made by Fusion and therefor has a caller id of zero.
 *
 */
typedef struct {
     int                      id;            /* id of the reference to watch */

     int                      call_id;       /* id of the call to execute */
     int                      call_arg;      /* optional call parameter, e.g. the id of a user
                                                space resource associated with that reference */
} FusionRefWatch;

/*
 * Inheriting local count from other reference
 */
typedef struct {
     int                      id;            /* own reference id */
     int                      from;          /* id of the reference to inherit from */
} FusionRefInherit;

/*
 * Throwing a reference
 */
typedef struct {
     int                      id;            /* reference id */
     int                      catcher;       /* fusion id of the catcher */
} FusionRefThrow;

/*
 * Killing other fusionees (experimental)
 */
 #ifdef CC_64BIT_SUPPORT
 typedef struct {
 	union {
     FusionID                 fusion_id;     /* fusionee to kill, zero means all but ourself */
	 unsigned long long u8padding;
 	};
     int                      signal;        /* signal to be delivered, e.g. SIGTERM */
     int                      timeout_ms;    /* -1 means no timeout, 0 means infinite, otherwise the
                                                max. time to wait until the fusionee(s) terminated */
} FusionKill;
 #else
typedef struct {
     FusionID                 fusion_id;     /* fusionee to kill, zero means all but ourself */
     int                      signal;        /* signal to be delivered, e.g. SIGTERM */
     int                      timeout_ms;    /* -1 means no timeout, 0 means infinite, otherwise the
                                                max. time to wait until the fusionee(s) terminated */
} FusionKill;
#endif
/*
 * Wait for a skirmish notification
 */
typedef struct {
     int                      id;            /* skirmish id */
     unsigned int             timeout;       /* timeout in ms (0 = unlimited) */

     unsigned int             lock_count;    /* MUST be set to zero, MUST NOT be reset when the system call is resumed. */
     unsigned int             notify_count;  /* MUST NOT be reset when the system call is resumed after a signal. */
} FusionSkirmishWait;

/*
 * Shared memory pools
 */
#ifdef CC_64BIT_SUPPORT
typedef struct {
     int                      max_size;      /* Maximum size that this pool will be allowed to grow to. */

     int                      pool_id;       /* Returns the new pool id. */
	 union {
     void                    *addr_base;     /* Returns the base of the reserved virtual memory address space. */
	 unsigned int 			compat_value;
	 unsigned long long u8addr_base;
	 };
} FusionSHMPoolNew;

typedef struct {
     int                      pool_id;       /* The id of the pool to attach to. */
	 int padding;
	 union {
     void                    *addr_base;     /* Returns the base of the reserved virtual memory address space. */
	 unsigned int 			compat_value;
	 unsigned long long u8addr_base;
	 };
     int                      size;          /* Returns the current size of the pool. */
} FusionSHMPoolAttach;
#else 
typedef struct {
     int                      max_size;      /* Maximum size that this pool will be allowed to grow to. */

     int                      pool_id;       /* Returns the new pool id. */
     void                    *addr_base;     /* Returns the base of the reserved virtual memory address space. */
} FusionSHMPoolNew;


typedef struct {
     int                      pool_id;       /* The id of the pool to attach to. */

     void                    *addr_base;     /* Returns the base of the reserved virtual memory address space. */
     int                      size;          /* Returns the current size of the pool. */
} FusionSHMPoolAttach;
#endif

typedef struct {
     int                      pool_id;       /* The id of the pool to notify. */

     int                      size;          /* New size of the pool. */
} FusionSHMPoolDispatch;

typedef enum {
     FSMT_REMAP,                             /* Remap the pool due to a change of its size. */
     FSMT_UNMAP                              /* Unmap the pool due to its destruction. */
} FusionSHMPoolMessageType;

typedef struct {
     FusionSHMPoolMessageType type;          /* Type of the message. */

     int                      size;          /* New size of the pool, if type is FSMT_REMAP. */
} FusionSHMPoolMessage;

/*
 * Fusion types
 */
typedef enum {
     FT_LOUNGE,
     FT_MESSAGING,
     FT_CALL,
     FT_REF,
     FT_SKIRMISH,
     FT_PROPERTY,
     FT_REACTOR,
     FT_SHMPOOL
} FusionType;

/*
 * Set attributes like 'name' for an entry of the specified type.
 */
#define FUSION_ENTRY_INFO_NAME_LENGTH   24

typedef struct {
     FusionType               type;
     int                      id;

     char                     name[FUSION_ENTRY_INFO_NAME_LENGTH];
} FusionEntryInfo;

#ifdef CC_64BIT_SUPPORT
typedef struct {
     FusionType               type;
     int                      id;

	union {
     FusionID                 fusion_id;
	 unsigned long long u8padding;
	};
     unsigned int             permissions;
} FusionEntryPermissions;
#else
typedef struct {
     FusionType               type;
     int                      id;

     FusionID                 fusion_id;
     unsigned int             permissions;
} FusionEntryPermissions;
#endif


#define FUSION_ENTRY_PERMISSIONS_ADD( p, ioc )    \
     do {                                         \
          (p) |= 1 << (_IOC_NR(ioc));             \
     } while (0)

#define FUSION_ENTRY_PERMISSIONS_REMOVE( p, ioc ) \
     do {                                         \
          (p) &= ~(1 << (_IOC_NR(ioc)));          \
     } while (0)

#define FUSION_ENTRY_PERMISSIONS_HAVE( p, ioc )   \
     ((p) & (1 << (_IOC_NR(ioc))))



#ifdef CC_64BIT_SUPPORT
typedef struct {
	union{
     FusionID  fusion_id;
	 unsigned long long u8padding;
	};

     char      exe_file[PATH_MAX];
} FusionGetFusioneeInfo;
#else
typedef struct {
     FusionID  fusion_id;

     char      exe_file[PATH_MAX];
} FusionGetFusioneeInfo;
#endif

#define FUSION_ENTER                         _IOR(FT_LOUNGE,    0x00, FusionEnter)
#define FUSION_UNBLOCK                       _IO (FT_LOUNGE,    0x01)
#define FUSION_KILL                          _IOW(FT_LOUNGE,    0x02, FusionKill)

#define FUSION_ENTRY_SET_INFO                _IOW(FT_LOUNGE,    0x03, FusionEntryInfo)
#define FUSION_ENTRY_GET_INFO                _IOW(FT_LOUNGE,    0x04, FusionEntryInfo)

#define FUSION_FORK                          _IOW(FT_LOUNGE,    0x05, FusionFork)

#define FUSION_SYNC                          _IO (FT_LOUNGE,    0x06)

#define FUSION_ENTRY_ADD_PERMISSIONS         _IOW(FT_LOUNGE,    0x07, FusionEntryPermissions)

#define FUSION_SHM_GET_BASE                  _IOR(FT_LOUNGE,    0x08, unsigned long)

#define FUSION_GET_FUSIONEE_INFO             _IOR(FT_LOUNGE,    0x09, FusionGetFusioneeInfo)


#define FUSION_SEND_MESSAGE                  _IOW(FT_MESSAGING, 0x00, FusionSendMessage)

#define FUSION_CALL_NEW                      _IOW(FT_CALL,      0x00, FusionCallNew)
#define FUSION_CALL_EXECUTE                  _IOW(FT_CALL,      0x01, FusionCallExecute)
#define FUSION_CALL_RETURN                   _IOW(FT_CALL,      0x02, FusionCallReturn)
#define FUSION_CALL_DESTROY                  _IOW(FT_CALL,      0x03, int)
#define FUSION_CALL_EXECUTE2                 _IOW(FT_CALL,      0x04, FusionCallExecute2)
#define FUSION_CALL_EXECUTE3                 _IOW(FT_CALL,      0x05, FusionCallExecute3)
#define FUSION_CALL_RETURN3                  _IOW(FT_CALL,      0x06, FusionCallReturn3)

#define FUSION_REF_NEW                       _IOW(FT_REF,       0x00, int)
#define FUSION_REF_UP                        _IOW(FT_REF,       0x01, int)
#define FUSION_REF_UP_GLOBAL                 _IOW(FT_REF,       0x02, int)
#define FUSION_REF_DOWN                      _IOW(FT_REF,       0x03, int)
#define FUSION_REF_DOWN_GLOBAL               _IOW(FT_REF,       0x04, int)
#define FUSION_REF_ZERO_LOCK                 _IOW(FT_REF,       0x05, int)
#define FUSION_REF_ZERO_TRYLOCK              _IOW(FT_REF,       0x06, int)
#define FUSION_REF_UNLOCK                    _IOW(FT_REF,       0x07, int)
#define FUSION_REF_STAT                      _IOW(FT_REF,       0x08, int)
#define FUSION_REF_WATCH                     _IOW(FT_REF,       0x09, FusionRefWatch)
#define FUSION_REF_INHERIT                   _IOW(FT_REF,       0x0A, FusionRefInherit)
#define FUSION_REF_DESTROY                   _IOW(FT_REF,       0x0B, int)
#define FUSION_REF_CATCH                     _IOW(FT_REF,       0x0C, int)
#define FUSION_REF_THROW                     _IOW(FT_REF,       0x0D, FusionRefThrow)

#define FUSION_SKIRMISH_NEW                  _IOW(FT_SKIRMISH,  0x00, int)
#define FUSION_SKIRMISH_PREVAIL              _IOW(FT_SKIRMISH,  0x01, int)
#define FUSION_SKIRMISH_SWOOP                _IOW(FT_SKIRMISH,  0x02, int)
#define FUSION_SKIRMISH_DISMISS              _IOW(FT_SKIRMISH,  0x03, int)
#define FUSION_SKIRMISH_DESTROY              _IOW(FT_SKIRMISH,  0x04, int)
#define FUSION_SKIRMISH_LOCK_COUNT           _IOW(FT_SKIRMISH,  0x05, int)
#define FUSION_SKIRMISH_WAIT                 _IOW(FT_SKIRMISH,  0x06, FusionSkirmishWait)
#define FUSION_SKIRMISH_NOTIFY               _IOW(FT_SKIRMISH,  0x07, int)

#define FUSION_PROPERTY_NEW                  _IOW(FT_PROPERTY,  0x00, int)
#define FUSION_PROPERTY_LEASE                _IOW(FT_PROPERTY,  0x01, int)
#define FUSION_PROPERTY_PURCHASE             _IOW(FT_PROPERTY,  0x02, int)
#define FUSION_PROPERTY_CEDE                 _IOW(FT_PROPERTY,  0x03, int)
#define FUSION_PROPERTY_HOLDUP               _IOW(FT_PROPERTY,  0x04, int)
#define FUSION_PROPERTY_DESTROY              _IOW(FT_PROPERTY,  0x05, int)

#define FUSION_REACTOR_NEW                   _IOW(FT_REACTOR,   0x00, int)
#define FUSION_REACTOR_ATTACH                _IOW(FT_REACTOR,   0x01, FusionReactorAttach)
#define FUSION_REACTOR_DETACH                _IOW(FT_REACTOR,   0x02, FusionReactorDetach)
#define FUSION_REACTOR_DISPATCH              _IOW(FT_REACTOR,   0x03, FusionReactorDispatch)
#define FUSION_REACTOR_DESTROY               _IOW(FT_REACTOR,   0x04, int)
#define FUSION_REACTOR_SET_DISPATCH_CALLBACK _IOW(FT_REACTOR,   0x05, FusionReactorSetCallback)

#define FUSION_SHMPOOL_NEW                   _IOW(FT_SHMPOOL,   0x00, FusionSHMPoolNew)
#define FUSION_SHMPOOL_ATTACH                _IOW(FT_SHMPOOL,   0x01, FusionSHMPoolAttach)
#define FUSION_SHMPOOL_DETACH                _IOW(FT_SHMPOOL,   0x02, int)
#define FUSION_SHMPOOL_DISPATCH              _IOW(FT_SHMPOOL,   0x03, FusionSHMPoolDispatch)
#define FUSION_SHMPOOL_DESTROY               _IOW(FT_SHMPOOL,   0x04, int)
#define FUSION_SHMPOOL_GET_BASE              _IOR(FT_SHMPOOL,   0x05, unsigned long)

#endif
