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
/// file    mdrv_tuner_io.c
/// @brief  TUNER Driver Interface
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

#include "mdrv_tuner.h"
#include "mdrv_tuner_io.h"
#include "mdrv_iic_io.h"
#include "mdrv_tuner_datatype.h"

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
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

#define TUNER_PRINT(fmt, args...)        printk("[TUNER][%05d] " fmt, __LINE__, ## args)
#ifdef TUNER_DEBUG
    #define DBG_TUNER(x) x
#else
    #define DBG_TUNER(x) //x
#endif
typedef struct
{
    S32                          s32MajorTUNER;
    S32                          s32MinorTUNER;
    struct cdev                 cDevice;
    struct file_operations      TUNERFop;
    struct fasync_struct        *async_queue; /* asynchronous readers */
} TUNER_ModHandle_t;


#define MOD_TUNER_DEVICE_COUNT         1
#define MOD_TUNER_NAME                 "tuner"
#define MOD_TUNER_SUPPORT_NUM   16

const char Tuner_Name0[]="tuner";
const char Tuner_Name1[]="tunerS";
static const char* Tuner_Name;
//const char Tuner_Name[];

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
//DECLARE_MUTEX(PfModeSem);


//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------
static int                      _MDrv_TUNER_Open (struct inode *inode, struct file *filp);
static int                      _MDrv_TUNER_Release(struct inode *inode, struct file *filp);
static ssize_t                  _MDrv_TUNER_Read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t                  _MDrv_TUNER_Write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static unsigned int             _MDrv_TUNER_Poll(struct file *filp, poll_table *wait);
#ifdef HAVE_UNLOCKED_IOCTL
static long _MDrv_TUNER_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int _MDrv_TUNER_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif
#if defined(CONFIG_COMPAT)
static long _Compat_MDrv_TUNER_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif
static int                      _MDrv_TUNER_Fasync(int fd, struct file *filp, int mode);

/* IIC kernel is in charge of bus initialization */
//extern void MDrv_IIC_Init(void);
//extern void MDrv_SW_IIC_Init(SWI2C_BusCfg SWI2CCBusCfg[], u8 u8CfgBusNum);

//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------

static struct class *tuner_class;
static TUNER_ModHandle_t TUNERDev=
{
    .s32MajorTUNER = MDRV_MAJOR_TUNER,
    .s32MinorTUNER = MDRV_MINOR_TUNER,
    .cDevice =
    {
        .kobj = {.name= MOD_TUNER_NAME, },
        .owner = THIS_MODULE,
    },
    .TUNERFop =
    {
        .open =     _MDrv_TUNER_Open,
        .release =  _MDrv_TUNER_Release,
        .read =     _MDrv_TUNER_Read,
        .write =    _MDrv_TUNER_Write,
        .poll =     _MDrv_TUNER_Poll,
        #ifdef HAVE_UNLOCKED_IOCTL
        .unlocked_ioctl = _MDrv_TUNER_Ioctl,
        #else
        .ioctl = _MDrv_TUNER_Ioctl,
        #endif
        #if defined(CONFIG_COMPAT)
        .compat_ioctl = _Compat_MDrv_TUNER_Ioctl,
        #endif
        .fasync =   _MDrv_TUNER_Fasync,
    },
};

// add for procfs
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
struct proc_dir_entry *mdb_proc_entry = NULL;
extern const struct file_operations mdb_tuner_node_operations = {
    .read = MDrv_TUNER_GetTUNERMdbInfo,//_MDrv_TUNER_Read,
    .write = MDrv_TUNER_WriteTUNERMdbInfo,//_MDrv_TUNER_Write,
};
#endif
//
u8 tuner_i2c_channel_bus=0;
extern const char match_name[16];
static int tuner[MOD_TUNER_SUPPORT_NUM],param_num;
module_param_array(tuner,int,&param_num,0644);

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------

static int _MDrv_TUNER_Open (struct inode *inode, struct file *filp)
{
    TUNER_ModHandle_t *dev;

    TUNER_PRINT("%s is invoked\n", __FUNCTION__);

    dev = container_of(inode->i_cdev, TUNER_ModHandle_t , cDevice);
    filp->private_data = dev;
    return 0;
}

static int _MDrv_TUNER_Release(struct inode *inode, struct file *filp)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

static ssize_t _MDrv_TUNER_Read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

static ssize_t _MDrv_TUNER_Write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

static unsigned int _MDrv_TUNER_Poll(struct file *filp, poll_table *wait)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

static int _MDrv_TUNER_Fasync(int fd, struct file *filp, int mode)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    return 0;
}

#if defined(CONFIG_COMPAT)
static long _Compat_MDrv_TUNER_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    U8 u8;
    U32 u32, ret;
    compat_uptr_t ptr;
    MS_TUNER_ExtendCommend_Info __user *ExtendCommend_Info;
    COMPAT_MS_TUNER_ExtendCommend_Info __user *COMPAT_ExtendCommend_Info;

    TUNER_PRINT("%s is invoked\n", __FUNCTION__);

    switch(cmd)
    {
        case MDRV_TUNER_ExtendCommand:
            COMPAT_ExtendCommend_Info = compat_ptr(arg);
            ExtendCommend_Info = (MS_TUNER_ExtendCommend_Info *)compat_alloc_user_space(sizeof(MS_TUNER_ExtendCommend_Info));
            if (ExtendCommend_Info == NULL)
                return -EFAULT;
            get_user(u8, &COMPAT_ExtendCommend_Info->u8SubCmd);
            put_user(u8, &ExtendCommend_Info->u8SubCmd);
            get_user(u32, &COMPAT_ExtendCommend_Info->u32Param1);
            put_user(u32, &ExtendCommend_Info->u32Param1);
            get_user(u32, &COMPAT_ExtendCommend_Info->u32Param2);
            put_user(u32, &ExtendCommend_Info->u32Param2);
            get_user(ptr, &COMPAT_ExtendCommend_Info->pvoidParam3);
            put_user(ptr, &ExtendCommend_Info->pvoidParam3);
            get_user(u32, &COMPAT_ExtendCommend_Info->retval);
            put_user(u32, &ExtendCommend_Info->retval);

            ret = filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)ExtendCommend_Info);

            get_user(u8, &ExtendCommend_Info->u8SubCmd);
            put_user(u8, &COMPAT_ExtendCommend_Info->u8SubCmd);
            get_user(u32, &ExtendCommend_Info->u32Param1);
            put_user(u32, &COMPAT_ExtendCommend_Info->u32Param1);
            get_user(u32, &ExtendCommend_Info->u32Param2);
            put_user(u32, &COMPAT_ExtendCommend_Info->u32Param2);
            get_user(ptr, &ExtendCommend_Info->pvoidParam3);
            put_user(ptr, &COMPAT_ExtendCommend_Info->pvoidParam3);
            get_user(u32, &ExtendCommend_Info->retval);
            put_user(u32, &COMPAT_ExtendCommend_Info->retval);
            return ret;

        case MDRV_TUNER_Connect:
        case MDRV_TUNER_Disconnect:
        case MDRV_TUNER_ATV_SetTune:
        case MDRV_TUNER_DVBS_SetTune:
        case MDRV_TUNER_DTV_SetTune:
        case MDRV_TUNER_TunerInit:
        case MDRV_TUNER_ConfigAGCMode:
        case MDRV_TUNER_SetTunerInScanMode:
        case MDRV_TUNER_SetTunerFinetuneMode:
        case MDRV_TUNER_GetCableStatus:
        case MDRV_TUNER_TunerReset:
        case MDRV_TUNER_IsLocked:
        case MDRV_TUNER_GetRSSI:
            return filp->f_op->unlocked_ioctl(filp, cmd,(unsigned long)compat_ptr(arg));
        default:
            TUNER_PRINT("ioctl: unknown command\n");
            return -ENOTTY;
    }
    return 0;
}
#endif

#ifdef HAVE_UNLOCKED_IOCTL
static long _MDrv_TUNER_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int _MDrv_TUNER_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
    MS_TUNER_TunerInit_Info TunerInit_Info = {0};
    MS_TUNER_ATV_SetTune_Info ATV_SetTune_Info = {0};
    MS_TUNER_DVBS_SetTune_Info DVBS_SetTune_Info = {0};
    MS_TUNER_DTV_SetTune_Info DTV_SetTune_Info = {0};
    MS_TUNER_ExtendCommend_Info ExtendCommend_Info = {0};
    MS_TUNER_Common_Info Common_Info = {0};
    MS_TUNER_GetRSSI_Info GetRSSI_Info = {0};
    s_Tuner_dev_info devTunerInit ={E_TUNER_DTV_DVB_T_MODE, E_CLOSE, E_RF_CH_BAND_8MHz, 0,0,0,0,0,0,0,0,0,0,0};
    int retval=0, minor;
    //SWI2C_BusCfg BusCfg = {0};
    void *pKernelParam3 = NULL;
    void *pUserParam3 = NULL;
    int sizeofP3 = 0;

    minor = iminor(filp->f_path.dentry->d_inode);
    DBG_TUNER(TUNER_PRINT("%s is invoked, device minor is %d\n", __FUNCTION__, minor));

    if ((TUNER_IOC_MAGIC!= _IOC_TYPE(cmd)) || (_IOC_NR(cmd)> TUNER_IOC_MAXNR))
    {
        return -ENOTTY;
    }

    switch(cmd)
    {
        case MDRV_TUNER_Connect:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_Connect\n"));
            retval=MDrv_TUNER_Connect(minor);
            if (copy_to_user((int *) arg, &retval, sizeof(int)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_TUNER_Disconnect:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_Disconnect\n"));
            retval=MDrv_TUNER_Disconnect(minor);
            if (copy_to_user((int *) arg, &retval, sizeof(int)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_TUNER_ATV_SetTune:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_ATV_SetTune\n"));
            if (copy_from_user(&ATV_SetTune_Info, (MS_TUNER_ATV_SetTune_Info *) arg, sizeof(MS_TUNER_ATV_SetTune_Info)))
            {
                return -EFAULT;
            }
            retval=MDrv_TUNER_ATV_SetTune(minor, ATV_SetTune_Info.u32FreqKHz, ATV_SetTune_Info.eBand, ATV_SetTune_Info.eMode, ATV_SetTune_Info.otherMode);
            ATV_SetTune_Info.retval = retval;
            if (copy_to_user((MS_TUNER_ATV_SetTune_Info *) arg, &ATV_SetTune_Info, sizeof(MS_TUNER_ATV_SetTune_Info)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_TUNER_DVBS_SetTune:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_DVBS_SetTune\n"));
            if (copy_from_user(&DVBS_SetTune_Info, (MS_TUNER_DVBS_SetTune_Info *) arg, sizeof(MS_TUNER_DVBS_SetTune_Info)))
            {
                return -EFAULT;
            }
            retval=MDrv_TUNER_DVBS_SetTune(minor, DVBS_SetTune_Info.u16CenterFreqMHz, DVBS_SetTune_Info.u32SymbolRateKs);
            DVBS_SetTune_Info.retval = retval;
            if (copy_to_user((MS_TUNER_DVBS_SetTune_Info *) arg, &DVBS_SetTune_Info, sizeof(MS_TUNER_DVBS_SetTune_Info)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_TUNER_DTV_SetTune:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_DTV_SetTune\n"));
            if (copy_from_user(&DTV_SetTune_Info, (MS_TUNER_DTV_SetTune_Info *) arg, sizeof(MS_TUNER_DTV_SetTune_Info)))
            {
                return -EFAULT;
            }
            retval=MDrv_TUNER_DTV_SetTune(minor, DTV_SetTune_Info.freq, DTV_SetTune_Info.eBandWidth, DTV_SetTune_Info.eMode);
            DTV_SetTune_Info.retval = retval;
            if (copy_to_user((MS_TUNER_DTV_SetTune_Info *) arg, &DTV_SetTune_Info, sizeof(MS_TUNER_DTV_SetTune_Info)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_TUNER_ExtendCommand:
            DBG_TUNER(TUNER_PRINT("ioctl: ++MDRV_TUNER_ExtendCommand\n"));
            if (copy_from_user(&ExtendCommend_Info, (MS_TUNER_ExtendCommend_Info *) arg, sizeof(MS_TUNER_ExtendCommend_Info)))
            {
                return -EFAULT;
            }
            DBG_TUNER(TUNER_PRINT("ioctl: == MDRV_TUNER_ExtendCommand subcmd [%d]P1[%d]P2[%d]\n",ExtendCommend_Info.u8SubCmd,ExtendCommend_Info.u32Param1,ExtendCommend_Info.u32Param2));
            //If no use param3 add in here.
            if((ExtendCommend_Info.u8SubCmd==E_TUNER_SUBCMD_SELECT_AGC_PIN)||ExtendCommend_Info.u8SubCmd==E_TUNER_SUBCMD_CONTROL_GPIO_PIN)
            {
                retval=MDrv_TUNER_ExtendCommand(minor, ExtendCommend_Info.u8SubCmd, ExtendCommend_Info.u32Param1, ExtendCommend_Info.u32Param2, ExtendCommend_Info.pvoidParam3);
                if(copy_to_user(&(((MS_TUNER_ExtendCommend_Info *) arg)->retval), &retval, sizeof(U32)))
                {
                    DBG_TUNER(TUNER_PRINT("ioctl: --copy_to_user fail retval\n"));
                    return -EFAULT;
                }
                if(retval != TRUE)
                {
                    DBG_TUNER(TUNER_PRINT("ioctl: --copy_to_user fail retval\n"));
                }
                return 0;
            }
            //If use param3 add type of P3 in below switch case.
            pUserParam3 = ExtendCommend_Info.pvoidParam3; //save userspace pointer
            if(pKernelParam3 == NULL)
            {
                switch(ExtendCommend_Info.u8SubCmd)
                {
                    case E_TUNER_SUBCMD_GET_FREQ_STEP:
                    case E_TUNER_SUBCMD_GET_VIF_TUNER_TYPE:
                    case E_TUNER_SUBCMD_GET_IQ_SWAP:
                    sizeofP3=sizeof(U8);
                    break;
                 case E_TUNER_SUBCMD_GET_IF_FREQ:
                 case E_TUNER_SUBCMD_GET_L_PRIME_IF_FREQ:
                 case E_TUNER_SUBCMD_GET_DTV_IF_FREQ:
                        sizeofP3=sizeof(U16);
                    break;
                 case E_TUNER_SUBCMD_GET_VHF_LOWMIN_FREQ:
                 case E_TUNER_SUBCMD_GET_VHF_LOWMAX_FREQ:
                 case E_TUNER_SUBCMD_GET_VHF_HIGHMIN_FREQ:
                 case E_TUNER_SUBCMD_GET_VHF_HIGHMAX_FREQ:
                 case E_TUNER_SUBCMD_GET_UHF_MIN_FREQ:
                 case E_TUNER_SUBCMD_GET_UHF_MAX_FREQ:
                        sizeofP3=sizeof(U32);
                    break;
                 case E_TUNER_SUBCMD_GET_VIF_PARA:
                        sizeofP3=sizeof(stVIFInitialIn);
                    break;
                 case E_TUNER_SUBCMD_GET_RF_LEVEL:
                        sizeofP3=sizeof(int);
                    break;
                 case E_TUNER_SUBCMD_GET_VIF_NOTCH_SOSFILTER:
                        sizeofP3=((U8)ExtendCommend_Info.u32Param2+10)*sizeof(U16);
                        break;
                    case E_TUNER_SUBCMD_GET_RF_TABLE:
                        DBG_TUNER(TUNER_PRINT("ioctl: --MDRV_TUNER_ExtendCommand(not support double pointer)\n"));
                        break;
                    case E_TUNER_SUBCMD_GET_DEMOD_CONFIG:
                        if(ExtendCommend_Info.u32Param1==E_TUNER_ATV_NTSC_MODE)
                        {
                            sizeofP3=sizeof(U32);
                        }
                        else if(ExtendCommend_Info.u32Param1==E_TUNER_DTV_DVB_T2_MODE)
                        {
                            sizeofP3=sizeof(U8);
                        }
                    else
                    {
                             DBG_TUNER(TUNER_PRINT("ioctl: --MDRV_TUNER_ExtendCommand(not support double pointer)\n"));
                    }
                        break;
                    case E_TUNER_SUBCMD_GET_PEAKING_PARAMETER:
                        sizeofP3=sizeof(stVIFUserFilter);
                        break;
                    case E_TUNER_SUBCMD_CHECK_RF_ROBUSTNESS:
                    case E_TUNER_SUBCMD_GETJTEMPERATRRE:
                    case E_TUNER_SUBCMD_USER_DEFINE:
                        break;
                    default:
                        break;
                }
            }
            if (sizeofP3 == 0)
            {
                DBG_TUNER(TUNER_PRINT("ioctl: --sizeofP3=0 can't change to kernel pointer\n"));
                return -EFAULT;
            }
            pKernelParam3 = kmalloc(sizeofP3, GFP_KERNEL);//kernel space
            if (pKernelParam3 == NULL)
            {
                DBG_TUNER(TUNER_PRINT("ioctl: --Param3kmalloc fail\n"));
                return -EFAULT;
            }
            if (ExtendCommend_Info.pvoidParam3 == NULL)
            {
                DBG_TUNER(TUNER_PRINT("ioctl: --ExtendCommend_Info.pvoidParam3 == NULL\n"));
                kfree(pKernelParam3);
                return -EFAULT;
            }
            // (address < TASK_SIZE) for user space
            // (address >= TASK_SIZE) for kernel space
            if (ExtendCommend_Info.pvoidParam3 >= TASK_SIZE)
            {
                DBG_TUNER(TUNER_PRINT("ioctl: --ExtendCommend_Info.pvoidParam3 >= TASK_SIZE\n"));
                return -EFAULT;
            }
            if(copy_from_user(pKernelParam3, ExtendCommend_Info.pvoidParam3, sizeofP3))
            {
                DBG_TUNER(TUNER_PRINT("ioctl: --copy_from_user fail\n"));
                kfree(pKernelParam3);
                return -EFAULT;
            }
            ExtendCommend_Info.pvoidParam3 = pKernelParam3;
            retval=MDrv_TUNER_ExtendCommand(minor, ExtendCommend_Info.u8SubCmd, ExtendCommend_Info.u32Param1, ExtendCommend_Info.u32Param2, ExtendCommend_Info.pvoidParam3);
            if(copy_to_user(&(((MS_TUNER_ExtendCommend_Info *) arg)->retval), &retval, sizeof(U32)))
            {
                DBG_TUNER(TUNER_PRINT("ioctl: --copy_to_user fail retval\n"));
                kfree(pKernelParam3);
                return -EFAULT;
            }
            if(retval != TRUE)
            {
                DBG_TUNER(TUNER_PRINT("ioctl: --MDrv_TUNER_ExtendCommand fail\n"));
                kfree(pKernelParam3);
                return 0;
            }
            if(copy_to_user(pUserParam3, pKernelParam3, sizeofP3))
            {
                DBG_TUNER(TUNER_PRINT("ioctl: --copy_to_user fail pKernelParam3\n"));
                kfree(pKernelParam3);
                return -EFAULT;
            }
            kfree(pKernelParam3);
            DBG_TUNER(TUNER_PRINT("ioctl: --MDRV_TUNER_ExtendCommand\n"));
            break;
        case MDRV_TUNER_TunerInit:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_TunerInit\n"));
            if (copy_from_user(&TunerInit_Info, (MS_TUNER_TunerInit_Info *) arg, sizeof(MS_TUNER_TunerInit_Info)))
            {
                return -EFAULT;
            }
            /* IIC init is the job of IIC kernel driver */
            //MDrv_IIC_Init();

            //BusCfg.u8ChIdx = TunerInit_Info.u8ChIdx;
            //BusCfg.u16PadSCL = TunerInit_Info.u16PadSCL;
            //BusCfg.u16PadSDA = TunerInit_Info.u16PadSDA;
            //BusCfg.u16SpeedKHz = TunerInit_Info.u16SpeedKHz;
            //BusCfg.u8Enable = TRUE;
            //MDrv_SW_IIC_Init(&BusCfg, 1);
            tuner_i2c_channel_bus=TunerInit_Info.u8ChIdx;
            devTunerInit.u8_slaveID=TunerInit_Info.u8SlaveId;
            devTunerInit.u8_i2c_bus=TunerInit_Info.u8ChIdx;
            devTunerInit.s8_Xtalcap=TunerInit_Info.s8xtralCap;
            TUNER_PRINT("%s  IOXtalCap : %d\n",__FUNCTION__,TunerInit_Info.s8xtralCap);
            retval=MDrv_TUNER_TunerInit(minor,&devTunerInit);
            if (copy_to_user((int *) arg, &retval, sizeof(int)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_TUNER_ConfigAGCMode:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_ConfigAGCMode\n"));
            if (copy_from_user(&Common_Info, (MS_TUNER_Common_Info *) arg, sizeof(MS_TUNER_Common_Info)))
            {
                return -EFAULT;
            }
            retval=MDrv_TUNER_ConfigAGCMode(minor, Common_Info.u32param);
            Common_Info.retval = retval;
            if (copy_to_user((MS_TUNER_Common_Info *) arg, &Common_Info, sizeof(MS_TUNER_Common_Info)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_TUNER_SetTunerInScanMode:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_SetTunerInScanMode\n"));
            if (copy_from_user(&Common_Info, (MS_TUNER_Common_Info *) arg, sizeof(MS_TUNER_Common_Info)))
            {
                return -EFAULT;
            }
            retval=MDrv_TUNER_SetTunerInScanMode(minor, Common_Info.u32param);
            Common_Info.retval = retval;
            if (copy_to_user((MS_TUNER_Common_Info *) arg, &Common_Info, sizeof(MS_TUNER_Common_Info)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_TUNER_SetTunerFinetuneMode:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_SetTunerFinetuneMode\n"));
            if (copy_from_user(&Common_Info, (MS_TUNER_Common_Info *) arg, sizeof(MS_TUNER_Common_Info)))
            {
                return -EFAULT;
            }
            retval=MDrv_TUNER_SetTunerInFinetuneMode(minor, Common_Info.u32param);
            Common_Info.retval = retval;
            if (copy_to_user((MS_TUNER_Common_Info *) arg, &Common_Info, sizeof(MS_TUNER_Common_Info)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_TUNER_GetCableStatus:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_GetCableStatus\n"));
            if (copy_from_user(&Common_Info, (MS_TUNER_Common_Info *) arg, sizeof(MS_TUNER_Common_Info)))
            {
                return -EFAULT;
            }
            retval=MDrv_TUNER_GetCableStatus(minor, Common_Info.u32param);
            Common_Info.retval = retval;
            if (copy_to_user((MS_TUNER_Common_Info *) arg, &Common_Info, sizeof(MS_TUNER_Common_Info)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_TUNER_TunerReset:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_TunerReset\n"));
            retval=MDrv_TUNER_TunerReset(minor);
            if (copy_to_user((int *) arg, &retval, sizeof(int)))
            {
               return -EFAULT;
            }
            break;
        case MDRV_TUNER_IsLocked:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_IsLocked\n"));
            retval=MDrv_TUNER_IsLocked(minor);
            if (copy_to_user((int *) arg, &retval, sizeof(int)))
            {
                return -EFAULT;
            }
            break;
        case MDRV_TUNER_GetRSSI:
            DBG_TUNER(TUNER_PRINT("ioctl: MDRV_TUNER_GetRSSI\n"));
            retval=MDrv_TUNER_GetRSSI(minor, GetRSSI_Info.u16Gain, GetRSSI_Info.u8DType);
            GetRSSI_Info.retval = retval;
            if (copy_to_user((MS_TUNER_GetRSSI_Info *) arg, &GetRSSI_Info, sizeof(MS_TUNER_GetRSSI_Info)))
            {
                return -EFAULT;
            }
            break;
        default:
            if (_IOC_NR(cmd)==TUNER_INIT_NR)
            {
                TUNER_PRINT("%s [error init fail]MDrv_TUNER_TunerInit Fail \n", __FUNCTION__);
                if (copy_from_user(&TunerInit_Info, (MS_TUNER_TunerInit_Info *) arg,( sizeof(MS_TUNER_TunerInit_Info))))
                {
                    return -EFAULT;
                }
                devTunerInit.u8_slaveID=TunerInit_Info.u8SlaveId;
                devTunerInit.u8_i2c_bus=TunerInit_Info.u8ChIdx;
                TUNER_PRINT("%s  IOXtalCap : %d\n",__FUNCTION__,TunerInit_Info.s8xtralCap);
                TUNER_PRINT("%s  u8_slaveID : %d\n",__FUNCTION__,TunerInit_Info.u8SlaveId);
                TUNER_PRINT("%s  u8_i2c_bus : %d\n",__FUNCTION__,TunerInit_Info.u8ChIdx);
                devTunerInit.s8_Xtalcap=-1;

                retval=MDrv_TUNER_TunerInit(minor,&devTunerInit);
                if (copy_to_user((int *) arg, &retval, sizeof(int)))
                {
                    return -EFAULT;
                }
                break;
            }
            DBG_TUNER(TUNER_PRINT("_MDrv_TUNER_Ioctl: unknown command\n"));
            return -ENOTTY;
    }
    return 0;
}

static int mod_tuner_init(struct platform_device *pdev)
{
    S32         s32Ret;
    dev_t       dev;
    int  Tuner_Group_Counter = 0;
    bool bTuner_Name_Check = false,bTuner_Name_Check_DVBS = false;
    U32 u32Tuner_Name_Check_counter = 0,u32Tuner_Number_MAX_Size = 0;

    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
    TUNERDev.s32MinorTUNER = pdev->id;

    if(sizeof(TUNER_MODULE_NAME)>=sizeof(TUNER_MODULE_NAME_DVBS))
    {
        u32Tuner_Number_MAX_Size = sizeof(TUNER_MODULE_NAME);
    }
    else
    {
        u32Tuner_Number_MAX_Size = sizeof(TUNER_MODULE_NAME_DVBS);
    }

    do
    {
        if(!(strcmp(TUNER_MODULE_NAME[Tuner_Group_Counter],match_name)))
        {
            TUNERDev.s32MajorTUNER = MDRV_MAJOR_TUNER;
            TUNERDev.s32MinorTUNER = MDRV_MINOR_TUNER;
            tuner_class = class_create(THIS_MODULE, "tuner");
            Tuner_Name = Tuner_Name0;
            //strcpy(Tuner_Name,Tuner_Name0);
            bTuner_Name_Check = true;
            // add for proc
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
            MDrv_SYS_UtopiaMdbMkdir(); //no need utopia
            mdb_proc_entry = proc_create("utopia_mdb/tuner", S_IRUSR, NULL, &mdb_tuner_node_operations);
            if (mdb_proc_entry == NULL)
            {
                printk("[tuner] %s: unable to create proc node\n", __FUNCTION__);
            }
#endif
        }
        else if(!(strcmp(TUNER_MODULE_NAME_DVBS[Tuner_Group_Counter],match_name)))
        {
            TUNERDev.s32MajorTUNER = MDRV_MAJOR_TUNER_DVBS;
            TUNERDev.s32MinorTUNER = MDRV_MINOR_TUNER_DVBS;
            tuner_class = class_create(THIS_MODULE, "tunerS");
            Tuner_Name = Tuner_Name1;
            //strcpy(Tuner_Name,Tuner_Name1);
            bTuner_Name_Check_DVBS = true;
            // add for proc
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
            MDrv_SYS_UtopiaMdbMkdir(); //no need utopia
            mdb_proc_entry = proc_create("utopia_mdb/tunerS", S_IRUSR, NULL, &mdb_tuner_node_operations);
            if (mdb_proc_entry == NULL)
            {
                printk("[tunerS] %s: unable to create proc node\n", __FUNCTION__);
            }
#endif
        }
        u32Tuner_Name_Check_counter += UNIT_TUNER_MODULE_CHAR;
        Tuner_Group_Counter++;
    } while((u32Tuner_Name_Check_counter != u32Tuner_Number_MAX_Size) && (bTuner_Name_Check == false) && (bTuner_Name_Check_DVBS == false));
    u32Tuner_Name_Check_counter = 0;
    Tuner_Group_Counter = 0;

    if((bTuner_Name_Check == false) && (bTuner_Name_Check_DVBS == false))
    {
        TUNERDev.s32MajorTUNER = MDRV_MAJOR_TUNER;
        TUNERDev.s32MinorTUNER = MDRV_MINOR_TUNER;
        tuner_class = class_create(THIS_MODULE, "tuner");
        Tuner_Name = Tuner_Name0;
        //strcpy(Tuner_Name,Tuner_Name0);
        TUNER_PRINT("[ERROR!!!][%s][%d]No tuner is match to the module table\n",__FUNCTION__,__LINE__);
    }

    TUNER_PRINT("TUNERDev.s32MajorTUNER: %d\nTUNERDev.s32MinorTUNER: %d\n", TUNERDev.s32MajorTUNER, TUNERDev.s32MinorTUNER);

    if (IS_ERR(tuner_class))
    {
        TUNER_PRINT("class_create failed!!!\n");
        return PTR_ERR(tuner_class);
    }

    if (TUNERDev.s32MajorTUNER)
    {
        dev = MKDEV(TUNERDev.s32MajorTUNER, TUNERDev.s32MinorTUNER);
        s32Ret = register_chrdev_region(dev, MOD_TUNER_DEVICE_COUNT, Tuner_Name);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, TUNERDev.s32MinorTUNER, MOD_TUNER_DEVICE_COUNT, Tuner_Name);
        TUNERDev.s32MajorTUNER = MAJOR(dev);
    }

    if ( 0 > s32Ret)
    {
        TUNER_PRINT("Unable to get major %d\n", TUNERDev.s32MajorTUNER);
        return s32Ret;
    }

    cdev_init(&TUNERDev.cDevice, &TUNERDev.TUNERFop);
    if (0!= (s32Ret= cdev_add(&TUNERDev.cDevice, dev, MOD_TUNER_DEVICE_COUNT)))
    {
        TUNER_PRINT("Unable add a character device\n");
        unregister_chrdev_region(dev, MOD_TUNER_DEVICE_COUNT);
        return s32Ret;
    }
    device_create(tuner_class, NULL, dev, NULL, Tuner_Name);



	  return 0;
}

static void mod_tuner_exit(void)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
    if (mdb_proc_entry != NULL)
        proc_remove(mdb_proc_entry);
#endif

    cdev_del(&TUNERDev.cDevice);
    unregister_chrdev_region(MKDEV(TUNERDev.s32MajorTUNER, TUNERDev.s32MinorTUNER), MOD_TUNER_DEVICE_COUNT);
}

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
static struct str_waitfor_dev waitfor;
static struct dev_pm_ops tuner_pm_ops;
static int mstar_tuner_drv_suspend(struct device *dev);
static int mstar_tuner_drv_resume(struct device *dev);
#endif
static int mstar_tuner_drv_probe(struct platform_device *pdev)
{
    //TUNER_PRINT("%s \n", __FUNCTION__);
    int retval=0;
    TUNER_PRINT("%s  pdev->name: [%s] match_name:[%s] pdev->id:[%d]\n", __FUNCTION__,pdev->name,match_name,pdev->id);
    retval = mod_tuner_init(pdev);
    if(!retval)
    {
        pdev->dev.platform_data=&TUNERDev;
    }

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
    of_mstar_str(MOD_TUNER_NAME, &pdev->dev,
                &tuner_pm_ops, &waitfor,
                &mstar_tuner_drv_suspend,
		&mstar_tuner_drv_resume,
                NULL, NULL);
#endif

    return retval;
}

static int mstar_tuner_drv_remove(struct platform_device *pdev)
{
    if( !(pdev->name) || strcmp(pdev->name,match_name)
        || pdev->id!=0)
    {
        return -1;
    }

    mod_tuner_exit();
    pdev->dev.platform_data=NULL;
    return 0;
}

static int mstar_tuner_drv_suspend(struct device *dev)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
    if (waitfor.stage1_s_wait)
        wait_for_completion(&(waitfor.stage1_s_wait->power.completion));
#endif
    MDrv_TUNER_Suspend();
    return 0;
}
static int mstar_tuner_drv_resume(struct device *dev)
{
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
    if (waitfor.stage1_r_wait)
        wait_for_completion(&(waitfor.stage1_r_wait->power.completion));
#endif
    MDrv_TUNER_Resume();
    return 0;
}

#ifndef CONFIG_MP_MSTAR_STR_OF_ORDER
static const struct dev_pm_ops tuner_pm_ops =
{
    .suspend_late = mstar_tuner_drv_suspend,
    .resume_early  = mstar_tuner_drv_resume,
};
#endif

static struct platform_driver Mstar_tuner_driver = {
    .probe      = mstar_tuner_drv_probe,
    .remove     = mstar_tuner_drv_remove,

    .driver = {
        .name = match_name,
        .owner = THIS_MODULE,
        .pm     = &tuner_pm_ops,
    }
};
static struct platform_device Mstar_tuner_device[MOD_TUNER_SUPPORT_NUM];

static int __init mstar_tuner_drv_init_module(void)
{
    int retval=0, i;
    TUNER_PRINT("%s is invoked\n", __FUNCTION__);

    for(i=0; i<param_num && i<MOD_TUNER_SUPPORT_NUM; i++)
    {
        Mstar_tuner_device[i].name=match_name;
        Mstar_tuner_device[i].id=tuner[i];
        TUNER_PRINT("mstar tuner device [%d]: %s   match name:%s  id=%d\n",i,Mstar_tuner_device[i].name,match_name,tuner[i]);
        platform_device_register(&(Mstar_tuner_device[i]));
    }

    retval = platform_driver_register(&Mstar_tuner_driver);
    return retval;
}

static void __exit mstar_tuner_drv_exit_module(void)
{
    platform_driver_unregister(&Mstar_tuner_driver);
}

module_init(mstar_tuner_drv_init_module);
module_exit(mstar_tuner_drv_exit_module);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("TUNER driver");
MODULE_LICENSE("GPL");

