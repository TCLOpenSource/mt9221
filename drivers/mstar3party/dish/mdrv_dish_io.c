////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2007 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (¡§MStar Confidential Information¡¨) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// file    mdrv_dish_io.c
/// @brief  DISH Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////


//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
//#include "MsCommon.h"
#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>

#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/rtc.h>
#include <linux/slab.h>

#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

#include "mst_devid.h"

#include "mdrv_dish.h"
#include "mdrv_dish_io.h"
#include "mdrv_dish_datatype.h"
#include "mdrv_iic_io.h"

#if 0//alex//defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#include <linux/namei.h>
#include <linux/proc_fs.h>
#include "mdrv_types.h"
#include "mdrv_system.h"
#endif

//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------

#define DISH_PRINT(fmt, args...)        printk("[DISH][%05d] " fmt, __LINE__, ## args)
#ifdef DISH_DEBUG
    #define DBG_DISH(x) x
#else
    #define DBG_DISH(x) //x
#endif
typedef struct
{
    S32                          s32MajorDISH;
    S32                          s32MinorDISH;
    struct cdev                 cDevice;
    struct file_operations      DISHFop;
    struct fasync_struct        *async_queue; /* asynchronous readers */
} DISH_ModHandle_t;


#define MOD_DISH_DEVICE_COUNT         1
#define MOD_DISH_NAME                 "dish"
#define MOD_DISH_SUPPORT_NUM   16

//const char Tuner_Name0[]="dish";
//const char Tuner_Name1[]="tunerS";
//const char* Tuner_Name;
//const char Tuner_Name[];
const char Dish_Name[]="dish";

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
//DECLARE_MUTEX(PfModeSem);


//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------
static int                      _MDrv_DISH_Open (struct inode *inode, struct file *filp);
static int                      _MDrv_DISH_Release(struct inode *inode, struct file *filp);
static ssize_t                  _MDrv_DISH_Read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t                  _MDrv_DISH_Write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static unsigned int             _MDrv_DISH_Poll(struct file *filp, poll_table *wait);
#ifdef HAVE_UNLOCKED_IOCTL
static long _MDrv_DISH_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int _MDrv_DISH_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif
#if defined(CONFIG_COMPAT)
static long _Compat_MDrv_DISH_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif
static int                      _MDrv_DISH_Fasync(int fd, struct file *filp, int mode);

/* IIC kernel is in charge of bus initialization */
//extern void MDrv_IIC_Init(void);
//extern void MDrv_SW_IIC_Init(SWI2C_BusCfg SWI2CCBusCfg[], u8 u8CfgBusNum);

//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------

static struct class *dish_class;
static DISH_ModHandle_t DISHDev=
{
    .s32MajorDISH = MDRV_MAJOR_DISH,
    .s32MinorDISH = MDRV_MINOR_DISH,
    .cDevice =
    {
        .kobj = {.name= MOD_DISH_NAME, },
        .owner = THIS_MODULE,
    },
    .DISHFop =
    {
        .open =     _MDrv_DISH_Open,
        .release =  _MDrv_DISH_Release,
        .read =     _MDrv_DISH_Read,
        .write =    _MDrv_DISH_Write,
        .poll =     _MDrv_DISH_Poll,
        #ifdef HAVE_UNLOCKED_IOCTL
        .unlocked_ioctl = _MDrv_DISH_Ioctl,
        #else
        .ioctl = _MDrv_DISH_Ioctl,
        #endif
        #if defined(CONFIG_COMPAT)
        .compat_ioctl = _Compat_MDrv_DISH_Ioctl,
        #endif
        .fasync =   _MDrv_DISH_Fasync,
    },
};

// add for procfs
#if 0//alex//defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
struct proc_dir_entry *mdb_proc_entry = NULL;
extern const struct file_operations mdb_tuner_node_operations = {
    .read = MDrv_DISH_GetDISHMdbInfo,//_MDrv_DISH_Read,
    .write = MDrv_DISH_WriteDISHMdbInfo,//_MDrv_DISH_Write,
};
#endif
//
u8 dish_i2c_channel_bus=0;
extern const char match_name[16];
static int dish[MOD_DISH_SUPPORT_NUM],param_num;
module_param_array(dish,int,&param_num,0644);

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------

static int _MDrv_DISH_Open (struct inode *inode, struct file *filp)
{
    DISH_ModHandle_t *dev;

    DISH_PRINT("%s is invoked\n", __FUNCTION__);

    dev = container_of(inode->i_cdev, DISH_ModHandle_t , cDevice);
    filp->private_data = dev;
    return 0;
}

static int _MDrv_DISH_Release(struct inode *inode, struct file *filp)
{
    DISH_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

static ssize_t _MDrv_DISH_Read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    DISH_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

static ssize_t _MDrv_DISH_Write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    DISH_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

static unsigned int _MDrv_DISH_Poll(struct file *filp, poll_table *wait)
{
    DISH_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

static int _MDrv_DISH_Fasync(int fd, struct file *filp, int mode)
{
    DISH_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

#if defined(CONFIG_COMPAT)
static long _Compat_MDrv_DISH_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    U8 u8;
    U32 u32, ret;
    compat_uptr_t ptr;
    MS_DISH_GetLNBPower_Info __user *GetLNBPower_Info;
    COMPAT_MS_DISH_GetLNBPower_Info __user *COMPAT_GetLNBPower_Info;
    MS_DISH_Get22KStatus_Info __user *Get22KStatus_Info;
    COMPAT_MS_DISH_Get22KStatus_Info __user *COMPAT_Get22KStatus_Info;

    DISH_PRINT("%s is invoked\n", __FUNCTION__);

    switch(cmd)
    {
        case MDRV_DISH_GetLNBPower:
            COMPAT_GetLNBPower_Info = compat_ptr(arg);
            GetLNBPower_Info = (MS_DISH_GetLNBPower_Info *)compat_alloc_user_space(sizeof(MS_DISH_GetLNBPower_Info));
            if (GetLNBPower_Info == NULL)
                return -EFAULT;
            get_user(ptr, &COMPAT_GetLNBPower_Info->pu32LNBPower_bOn);
            put_user(ptr, &GetLNBPower_Info->pu32LNBPower_bOn);
            get_user(u32, &COMPAT_GetLNBPower_Info->retval);
            put_user(u32, &GetLNBPower_Info->retval);

            ret = filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)GetLNBPower_Info);

            get_user(ptr, &GetLNBPower_Info->pu32LNBPower_bOn);
            put_user(ptr, &COMPAT_GetLNBPower_Info->pu32LNBPower_bOn);
            get_user(u32, &GetLNBPower_Info->retval);
            put_user(u32, &COMPAT_GetLNBPower_Info->retval);
            return ret;
        case MDRV_DISH_Get22KStatus:
            COMPAT_Get22KStatus_Info = compat_ptr(arg);
            Get22KStatus_Info = (MS_DISH_Get22KStatus_Info *)compat_alloc_user_space(sizeof(MS_DISH_Get22KStatus_Info));
            if (Get22KStatus_Info == NULL)
                return -EFAULT;
            get_user(ptr, &COMPAT_Get22KStatus_Info->pu32Get22KStatus);
            put_user(ptr, &Get22KStatus_Info->pu32Get22KStatus);
            get_user(u32, &COMPAT_Get22KStatus_Info->retval);
            put_user(u32, &Get22KStatus_Info->retval);

            ret = filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)Get22KStatus_Info);

            get_user(ptr, &Get22KStatus_Info->pu32Get22KStatus);
            put_user(ptr, &COMPAT_Get22KStatus_Info->pu32Get22KStatus);
            get_user(u32, &Get22KStatus_Info->retval);
            put_user(u32, &COMPAT_Get22KStatus_Info->retval);
            return ret;
        case MDRV_DISH_Init:
        case MDRV_DISH_SetLNBPower:
        case MDRV_DISH_Set22KOnOff:
        case MDRV_DISH_IsOverCurrent:
            return filp->f_op->unlocked_ioctl(filp, cmd,(unsigned long)compat_ptr(arg));
        default:
            DISH_PRINT("ioctl: unknown command\n");
            return -ENOTTY;
    }
    return 0;
}
#endif

#ifdef HAVE_UNLOCKED_IOCTL
static long _MDrv_DISH_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int _MDrv_DISH_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
    MS_DISH_Common_Info Common_Info = {0};
    MS_DISH_DishInit_Info DishInit_Info = {0};
    MS_DISH_SetLNBPower_Info SetLNBPower_Info = {0};
    MS_DISH_GetLNBPower_Info GetLNBPower_Info = {0};
    MS_DISH_Set22KOnOff_Info Set22KOnOff_Info = {0};
    MS_DISH_Get22KStatus_Info Get22KStatus_Info = {0};
    s_Dish_dev_info devDishInit ={E_CLOSE, E_LNB_POWER_OFF, E_TONE0, 0,0,0,0,0,0,0,0};
    EN_SAT_LNBPOWER_TYPE LNBPowerType = E_LNB_POWER_OFF;

    int retval=0, minor;

    void *pKernelParamGet = NULL;
    void *pUserParamGet = NULL;
    int sizeofParamGet = 0;

    minor = iminor(filp->f_path.dentry->d_inode);
    DBG_DISH(DISH_PRINT("%s is invoked, device minor is %d\n", __FUNCTION__, minor));

    if ((DISH_IOC_MAGIC!= _IOC_TYPE(cmd)) || (_IOC_NR(cmd)> DISH_IOC_MAXNR))
    {
        return -ENOTTY;
    }

    switch(cmd)
    {
        case MDRV_DISH_Init:
            DBG_DISH(DISH_PRINT("ioctl: MDRV_DISH_Init\n"));
            if (copy_from_user(&DishInit_Info, (MS_DISH_DishInit_Info *) arg, sizeof(MS_DISH_DishInit_Info)))
            {
                return -EFAULT;
            }

            dish_i2c_channel_bus = DishInit_Info.u8ChIdx;
            devDishInit.u8_slaveID = DishInit_Info.u8SlaveId;
            devDishInit.u8_i2c_bus = DishInit_Info.u8ChIdx;
            devDishInit.u16LNBEnablePinNum = DishInit_Info.u16LNBEnablePinNum;
            devDishInit.u8LNBEnablePinInv = DishInit_Info.u8LNBEnablePinInv;
            devDishInit.u16LNBSelectPin1Num= DishInit_Info.u16LNBSelectPin1Num;
            devDishInit.u8LNBSelectPin1Inv= DishInit_Info.u8LNBSelectPin1Inv;
            devDishInit.u16LNBSelectPin2Num= DishInit_Info.u16LNBSelectPin2Num;
            devDishInit.u8LNBSelectPin2Inv= DishInit_Info.u8LNBSelectPin2Inv;
            retval=MDrv_DISH_Init(minor,&devDishInit);

            if (copy_to_user((int *) arg, &retval, sizeof(int)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_DISH_SetLNBPower:
            DBG_DISH(DISH_PRINT("ioctl: MDRV_DISH_SetLNBPower\n"));
            if (copy_from_user(&SetLNBPower_Info, (MS_DISH_SetLNBPower_Info *) arg, sizeof(MS_DISH_SetLNBPower_Info)))
            {
                return -EFAULT;
            }

            LNBPowerType = (EN_SAT_LNBPOWER_TYPE)SetLNBPower_Info.u32LNBPower;
            retval=MDrv_DISH_SetLNBPower(minor, LNBPowerType);
            SetLNBPower_Info.retval = retval;
            if (copy_to_user((MS_DISH_SetLNBPower_Info *) arg, &SetLNBPower_Info, sizeof(MS_DISH_SetLNBPower_Info)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_DISH_GetLNBPower:
            DBG_DISH(DISH_PRINT("ioctl: MDRV_DISH_GetLNBPower\n"));
            if (copy_from_user(&GetLNBPower_Info, (MS_DISH_GetLNBPower_Info *) arg, sizeof(MS_DISH_GetLNBPower_Info)))
            {
                return -EFAULT;
            }

            pUserParamGet = GetLNBPower_Info.pu32LNBPower_bOn; //save userspace pointer
            if(pKernelParamGet == NULL)
            {
                sizeofParamGet=sizeof(U32);
            }
            if (sizeofParamGet == 0)
            {
                DBG_DISH(DISH_PRINT("ioctl: --sizeofParamGet=0 can't change to kernel pointer\n"));
                return -EFAULT;
            }
            pKernelParamGet = kmalloc(sizeofParamGet, GFP_KERNEL);
            if (pKernelParamGet == NULL)
            {
                DBG_DISH(DISH_PRINT("ioctl: --sizeofParamGetkmalloc fail\n"));
                return -EFAULT;
            }
            if (GetLNBPower_Info.pu32LNBPower_bOn == NULL)
            {
                DBG_DISH(DISH_PRINT("ioctl: --GetLNBPower_Info.pu32LNBPower_bOn == NULL\n"));
                kfree(pKernelParamGet);
                return -EFAULT;
            }
            if(copy_from_user(pKernelParamGet, GetLNBPower_Info.pu32LNBPower_bOn, sizeofParamGet))
            {
                DBG_DISH(DISH_PRINT("ioctl: --MDRV_DISH_GetLNBPower copy_from_user fail\n"));
                kfree(pKernelParamGet);
                return -EFAULT;
            }
            GetLNBPower_Info.pu32LNBPower_bOn = pKernelParamGet;
            retval=MDrv_DISH_GetLNBPower(minor, (EN_SAT_LNBPOWER_TYPE*)GetLNBPower_Info.pu32LNBPower_bOn);
            if(copy_to_user(&(((MS_DISH_GetLNBPower_Info *) arg)->retval), &retval, sizeof(U32)))
            {
                DBG_DISH(DISH_PRINT("ioctl: --MDRV_DISH_GetLNBPower copy_to_user fail retval\n"));
                kfree(pKernelParamGet);
                return -EFAULT;
            }
            if(retval != TRUE)
            {
                DBG_DISH(DISH_PRINT("ioctl: --MDrv_DISH_GetLNBPower fail\n"));
                kfree(pKernelParamGet);
                return 0;
            }
            if(copy_to_user(pUserParamGet, pKernelParamGet, sizeofParamGet))
            {
                DBG_DISH(DISH_PRINT("ioctl: --copy_to_user fail pKernelParamGet\n"));
                kfree(pKernelParamGet);
                return -EFAULT;
            }
            kfree(pKernelParamGet);
            DBG_DISH(DISH_PRINT("ioctl: --MDRV_DISH_GetLNBPower\n"));
            break;

        case MDRV_DISH_Set22KOnOff:
            DBG_DISH(DISH_PRINT("ioctl: MDRV_DISH_Set22KOnOff\n"));
            if (copy_from_user(&Set22KOnOff_Info, (MS_DISH_Set22KOnOff_Info *) arg, sizeof(MS_DISH_Set22KOnOff_Info)))
            {
                return -EFAULT;
            }
            retval=MDrv_DISH_Set22KOnOff(minor, Set22KOnOff_Info.u32Set22KOnOff);
            Set22KOnOff_Info.retval = retval;
            if (copy_to_user((MS_DISH_Set22KOnOff_Info *) arg, &Set22KOnOff_Info, sizeof(MS_DISH_Set22KOnOff_Info)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_DISH_Get22KStatus:
            DBG_DISH(DISH_PRINT("ioctl: MDRV_DISH_Get22KStatus\n"));
            if (copy_from_user(&Get22KStatus_Info, (MS_DISH_Get22KStatus_Info *) arg, sizeof(MS_DISH_Get22KStatus_Info)))
            {
                return -EFAULT;
            }

            pUserParamGet = Get22KStatus_Info.pu32Get22KStatus; //save userspace pointer
            if(pKernelParamGet == NULL)
            {
                sizeofParamGet=sizeof(U32);
            }
            if (sizeofParamGet == 0)
            {
                DBG_DISH(DISH_PRINT("ioctl: --sizeofParamGet=0 can't change to kernel pointer\n"));
                return -EFAULT;
            }
            pKernelParamGet = kmalloc(sizeofParamGet, GFP_KERNEL);
            if (pKernelParamGet == NULL)
            {
                DBG_DISH(DISH_PRINT("ioctl: --sizeofParamGetkmalloc fail\n"));
                return -EFAULT;
            }
            if (Get22KStatus_Info.pu32Get22KStatus == NULL)
            {
                DBG_DISH(DISH_PRINT("ioctl: --Get22KStatus_Info.pu32Get22KStatus == NULL\n"));
                kfree(pKernelParamGet);
                return -EFAULT;
            }
            if(copy_from_user(pKernelParamGet, Get22KStatus_Info.pu32Get22KStatus, sizeofParamGet))
            {
                DBG_DISH(DISH_PRINT("ioctl: --MDRV_DISH_Get22KStatus copy_from_user fail\n"));
                kfree(pKernelParamGet);
                return -EFAULT;
            }
            Get22KStatus_Info.pu32Get22KStatus = pKernelParamGet;
            retval=MDrv_DISH_Get22KStatus(minor, Get22KStatus_Info.pu32Get22KStatus);
            if(copy_to_user(&(((MS_DISH_Get22KStatus_Info *) arg)->retval), &retval, sizeof(U32)))
            {
                DBG_DISH(DISH_PRINT("ioctl: --MDRV_DISH_Get22KStatus copy_to_user fail retval\n"));
                kfree(pKernelParamGet);
                return -EFAULT;
            }
            if(retval != TRUE)
            {
                DBG_DISH(DISH_PRINT("ioctl: --MDRV_DISH_Get22KStatus fail\n"));
                kfree(pKernelParamGet);
                return 0;
            }
            if(copy_to_user(pUserParamGet, pKernelParamGet, sizeofParamGet))
            {
                DBG_DISH(DISH_PRINT("ioctl: --copy_to_user fail pKernelParamGet\n"));
                kfree(pKernelParamGet);
                return -EFAULT;
            }
            kfree(pKernelParamGet);
            DBG_DISH(DISH_PRINT("ioctl: --MDRV_DISH_Get22KStatus\n"));
            break;
        case MDRV_DISH_IsOverCurrent:
            DBG_DISH(DISH_PRINT("ioctl: MDRV_DISH_IsOverCurrent\n"));
            retval=MDrv_DISH_IsOverCurrent(minor);
            if (copy_to_user((int *) arg, &retval, sizeof(int)))
            {
                return -EFAULT;
            }
            break;
        default:
            DBG_DISH(DISH_PRINT("_MDrv_DISH_Ioctl: unknown command\n"));
            return -ENOTTY;
    }
    return 0;
}

static int mod_dish_init(struct platform_device *pdev)
{
    S32         s32Ret;
    dev_t       dev;
    int  Dish_Group_Counter = 0;
    bool bDish_Name_Check = false;
    U32 u32Dish_Name_Check_counter = 0,u32Dish_Number_MAX_Size = 0;

    DISH_PRINT("%s is invoked\n", __FUNCTION__);
    DISHDev.s32MinorDISH = pdev->id;

    u32Dish_Number_MAX_Size = sizeof(DISH_MODULE_NAME);

    do
    {
        if(!(strcmp(DISH_MODULE_NAME[Dish_Group_Counter],match_name)))
        {
            DISHDev.s32MajorDISH = MDRV_MAJOR_DISH;
            DISHDev.s32MinorDISH = MDRV_MINOR_DISH;
            dish_class = class_create(THIS_MODULE, "dish");
            //Dish_Name = Tuner_Name0;
            //strcpy(Dish_Name,Tuner_Name0);
            bDish_Name_Check = true;
            // add for proc
#if 0//alex//defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
            MDrv_SYS_UtopiaMdbMkdir(); //no need utopia
            mdb_proc_entry = proc_create("utopia_mdb/dish", S_IRUSR, NULL, &mdb_tuner_node_operations);
            if (mdb_proc_entry == NULL)
            {
                printk("[dish] %s: unable to create proc node\n", __FUNCTION__);
            }
#endif
        }

        u32Dish_Name_Check_counter += UNIT_DISH_MODULE_CHAR;
        Dish_Group_Counter++;
    } while((u32Dish_Name_Check_counter != u32Dish_Number_MAX_Size) && (bDish_Name_Check == false));
    u32Dish_Name_Check_counter = 0;
    Dish_Group_Counter = 0;

    if(bDish_Name_Check == false)
    {
        DISHDev.s32MajorDISH = MDRV_MAJOR_DISH;
        DISHDev.s32MinorDISH = MDRV_MINOR_DISH;
        dish_class = class_create(THIS_MODULE, "dish");
        //strcpy(Dish_Name,Tuner_Name0);
        DISH_PRINT("[ERROR!!!][%s][%d]No dish is match to the module table\n",__FUNCTION__,__LINE__);
    }

    DISH_PRINT("DISHDev.s32MajorDISH: %d\nDISHDev.s32MinorDISH: %d\n", DISHDev.s32MajorDISH, DISHDev.s32MinorDISH);

    if (IS_ERR(dish_class))
    {
        DISH_PRINT("class_create failed!!!\n");
        return PTR_ERR(dish_class);
    }

    if (DISHDev.s32MajorDISH)
    {
        dev = MKDEV(DISHDev.s32MajorDISH, DISHDev.s32MinorDISH);
        s32Ret = register_chrdev_region(dev, MOD_DISH_DEVICE_COUNT, Dish_Name);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, DISHDev.s32MinorDISH, MOD_DISH_DEVICE_COUNT, Dish_Name);
        DISHDev.s32MajorDISH = MAJOR(dev);
    }

    if ( 0 > s32Ret)
    {
        DISH_PRINT("Unable to get major %d\n", DISHDev.s32MajorDISH);
        return s32Ret;
    }

    cdev_init(&DISHDev.cDevice, &DISHDev.DISHFop);
    if (0!= (s32Ret= cdev_add(&DISHDev.cDevice, dev, MOD_DISH_DEVICE_COUNT)))
    {
        DISH_PRINT("Unable add a character device\n");
        unregister_chrdev_region(dev, MOD_DISH_DEVICE_COUNT);
        return s32Ret;
    }
    device_create(dish_class, NULL, dev, NULL, Dish_Name);



    return 0;
}

static void mod_dish_exit(void)
{
    DISH_PRINT("%s is invoked\n", __FUNCTION__);

#if 0//alex//defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
    if (mdb_proc_entry != NULL)
        proc_remove(mdb_proc_entry);
#endif

    cdev_del(&DISHDev.cDevice);
    unregister_chrdev_region(MKDEV(DISHDev.s32MajorDISH, DISHDev.s32MinorDISH), MOD_DISH_DEVICE_COUNT);
}

static int mstar_dish_drv_probe(struct platform_device *pdev)
{
    //DISH_PRINT("%s \n", __FUNCTION__);
    int retval=0;
    if( !(pdev->name) || strcmp(pdev->name,match_name)
        || pdev->id!=0)
    {
        retval = -ENXIO;
    }
    DISH_PRINT("%s  pdev->name: [%s] match_name:[%s] pdev->id:[%d]\n", __FUNCTION__,pdev->name,match_name,pdev->id);
    retval = mod_dish_init(pdev);
    if(!retval)
    {
        pdev->dev.platform_data=&DISHDev;
    }
	  return retval;
}

static int mstar_dish_drv_remove(struct platform_device *pdev)
{
    if( !(pdev->name) || strcmp(pdev->name,match_name)
        || pdev->id!=0)
    {
        return -1;
    }

    mod_dish_exit();
    pdev->dev.platform_data=NULL;
    return 0;
}

static int mstar_dish_drv_suspend(struct platform_device *dev, pm_message_t state)
{
    DISH_PRINT("%s is invoked\n", __FUNCTION__);
    MDrv_DISH_Suspend();
    return 0;
}
static int mstar_dish_drv_resume(struct platform_device *dev)
{
    DISH_PRINT("%s is invoked\n", __FUNCTION__);
    MDrv_DISH_Resume();
    return 0;
}

static const struct dev_pm_ops dish_pm_ops =
{
    .suspend_late = mstar_dish_drv_suspend,
    .resume_early  = mstar_dish_drv_resume,
};

static struct platform_driver Mstar_dish_driver = {
    .probe      = mstar_dish_drv_probe,
    .remove     = mstar_dish_drv_remove,

    .driver = {
        .name = match_name,
        .owner = THIS_MODULE,
        .pm     = &dish_pm_ops,
    }
};
static struct platform_device Mstar_dish_device[MOD_DISH_SUPPORT_NUM];

static int __init mstar_dish_drv_init_module(void)
{
    int retval=0, i;
    DISH_PRINT("%s is invoked\n", __FUNCTION__);

    for(i=0; i<param_num && i<MOD_DISH_SUPPORT_NUM; i++)
    {
        Mstar_dish_device[i].name=match_name;
        Mstar_dish_device[i].id=dish[i];
        DISH_PRINT("mstar dish device [%d]: %s   match name:%s  id=%d\n",i,Mstar_dish_device[i].name,match_name,dish[i]);
        platform_device_register(&(Mstar_dish_device[i]));
    }

    retval = platform_driver_register(&Mstar_dish_driver);
    return retval;
}

static void __exit mstar_dish_drv_exit_module(void)
{
    platform_driver_unregister(&Mstar_dish_driver);
}

module_init(mstar_dish_drv_init_module);
module_exit(mstar_dish_drv_exit_module);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("DISH driver");
MODULE_LICENSE("GPL");

