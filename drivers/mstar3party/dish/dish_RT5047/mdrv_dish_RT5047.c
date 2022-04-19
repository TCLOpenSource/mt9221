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

#include "mst_devid.h"
#include "mdrv_types.h"
#include "mdrv_dish.h"

#include "mdrv_dish_datatype.h"
#include <linux/gpio.h>

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
const char match_name[16]="Mstar-RT5047";

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------

#define DISH_PRINT(fmt, args...)        printk("[DISH][%05d] " fmt, __LINE__, ## args)
#define DISH_DEBUG // for  DBG
#ifdef DISH_DEBUG
    #define DBG_DISH(x) x
#else
    #define DBG_DISH(x) //x
#endif
#ifndef UNUSED //to avoid compile warnings...
#define UNUSED(var) (void)((var) = (var))
#endif

#define MAX_DISH_DEV_NUM 2
static bool LNBEnablePinSupport = TRUE;
static bool LNBSelectPin2Support = TRUE;
//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
static s_Dish_dev_info *p_dev = NULL;
static s_Dish_dev_info dev[MAX_DISH_DEV_NUM] =
{
    {E_CLOSE, E_LNB_POWER_OFF, E_TONE0, 0,0,0,0,0,0,0,0},
    {E_CLOSE, E_LNB_POWER_OFF, E_TONE0, 0,0,0,0,0,0,0,0},
};
static EN_SAT_LNBPOWER_TYPE CurLNBPwr = E_LNB_POWER_OFF;
//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
static int _SetGPIOHigh(U16 GPIOPinNum, U8 Invert)
{
    int ret = FALSE;

    if(Invert == TRUE)
    {
        gpio_free(GPIOPinNum);
        if(gpio_request(GPIOPinNum,NULL)<0)
        {
            DBG_DISH(DISH_PRINT("GPIO pin num [%d] request enable fail! \n",GPIOPinNum));
            ret = FALSE;
        }
        if(gpio_direction_output(GPIOPinNum, 0)<0)
        {
            DBG_DISH(DISH_PRINT("GPIO pin num [%d] gpio_direction_output fail! \n",GPIOPinNum));
            ret = FALSE;
        }
        gpio_set_value(GPIOPinNum, 0);
        ret = TRUE;
    }
    else
    {
        gpio_free(GPIOPinNum);
        if(gpio_request(GPIOPinNum,NULL)<0)
        {
            DBG_DISH(DISH_PRINT("GPIO pin num [%d] request enable fail! \n",GPIOPinNum));
            ret = FALSE;
        }
        if(gpio_direction_output(GPIOPinNum, 1)<0)
        {
            DBG_DISH(DISH_PRINT("GPIO pin num [%d] gpio_direction_output fail! \n",GPIOPinNum));
            ret = FALSE;
        }
        gpio_set_value(GPIOPinNum, 1);
        ret = TRUE;
    }
    return ret;
}

static int _SetGPIOLow(U16 GPIOPinNum, U8 Invert)
{
    int ret = FALSE;

    if(Invert == TRUE)
    {
        gpio_free(GPIOPinNum);
        if(gpio_request(GPIOPinNum,NULL)<0)
        {
            DBG_DISH(DISH_PRINT("GPIO pin num [%d] request enable fail! \n",GPIOPinNum));
            ret = FALSE;
        }
        if(gpio_direction_output(GPIOPinNum, 1)<0)
        {
            DBG_DISH(DISH_PRINT("GPIO pin num [%d] gpio_direction_output fail! \n",GPIOPinNum));
            ret = FALSE;
        }
        gpio_set_value(GPIOPinNum, 1);
        ret = TRUE;
    }
    else
    {
        gpio_free(GPIOPinNum);
        if(gpio_request(GPIOPinNum,NULL)<0)
        {
            DBG_DISH(DISH_PRINT("GPIO pin num [%d] request enable fail! \n",GPIOPinNum));
            ret = FALSE;
        }
        if(gpio_direction_output(GPIOPinNum, 0)<0)
        {
            DBG_DISH(DISH_PRINT("GPIO pin num [%d] gpio_direction_output fail! \n",GPIOPinNum));
            ret = FALSE;
        }
        gpio_set_value(GPIOPinNum, 0);
        ret = TRUE;
    }
    return ret;
}


//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
int MDrv_DISH_Init(int minor, s_Dish_dev_info *devDishInit)
{
    int ret = FALSE;

    DBG_DISH(DISH_PRINT("%s is invoked\n", __FUNCTION__));
    DBG_DISH(DISH_PRINT("u16LNBEnablePinNum = %d \n", devDishInit->u16LNBEnablePinNum));
    DBG_DISH(DISH_PRINT("u8LNBEnablePinInv = %d \n", devDishInit->u8LNBEnablePinInv));
    DBG_DISH(DISH_PRINT("u16LNBSelectPin1Num = %d \n", devDishInit->u16LNBSelectPin1Num));
    DBG_DISH(DISH_PRINT("u8LNBSelectPin1Inv = %d \n", devDishInit->u8LNBSelectPin1Inv));
    DBG_DISH(DISH_PRINT("u16LNBSelectPin2Num = %d \n", devDishInit->u16LNBSelectPin2Num));
    DBG_DISH(DISH_PRINT("u8LNBSelectPin2Inv = %d \n", devDishInit->u8LNBSelectPin2Inv));

    if (minor < MAX_DISH_DEV_NUM)
    {
        p_dev=&(dev[minor]);
        p_dev->u16LNBEnablePinNum = devDishInit->u16LNBEnablePinNum;
        p_dev->u8LNBEnablePinInv = devDishInit->u8LNBEnablePinInv;
        p_dev->u16LNBSelectPin1Num = devDishInit->u16LNBSelectPin1Num;
        p_dev->u8LNBSelectPin1Inv = devDishInit->u8LNBSelectPin1Inv;
        p_dev->u16LNBSelectPin2Num = devDishInit->u16LNBSelectPin2Num;
        p_dev->u8LNBSelectPin2Inv = devDishInit->u8LNBSelectPin2Inv;

        if(p_dev->u16LNBEnablePinNum == (GPIOPinNumNotSupport - GPIOPinKernelOffSet))
        {
            LNBEnablePinSupport = FALSE;
        }
        if(p_dev->u16LNBSelectPin2Num == (GPIOPinNumNotSupport - GPIOPinKernelOffSet))
        {
            LNBSelectPin2Support = FALSE;
        }
        if(LNBEnablePinSupport)
        {
            if(_SetGPIOHigh(p_dev->u16LNBEnablePinNum, p_dev->u8LNBEnablePinInv) != FALSE)
            {
                p_dev->e_status = E_WORKING;
                ret = TRUE;
            }
            else
            {
                DISH_PRINT("DISH Enable fail !\n");
                ret = FALSE;
            }
        }
        else
        {
            p_dev->e_status = E_WORKING;
            DISH_PRINT("DISH Enable Pin is NULL !\n");
            ret = FALSE;
        }
    }
    else
    {
        ret = FALSE;
    }

    return ret;
}
int MDrv_DISH_SetLNBPower(int minor, EN_SAT_LNBPOWER_TYPE eLNBPower)
{
    int ret = FALSE;

    DBG_DISH(DISH_PRINT("%s is invoked\n", __FUNCTION__));
    DBG_DISH(DISH_PRINT(" [kernel] MDrv_DISH_SetLNBPower eLNBPower = %d !\n",eLNBPower));
    p_dev=&(dev[minor]);
    if(eLNBPower == E_LNB_POWER_OFF)
    {
        DBG_DISH(DISH_PRINT(" MDrv_DISH_SetLNBPower E_LNB_POWER_OFF !\n"));
        if(LNBEnablePinSupport)
        {
            if(_SetGPIOLow(p_dev->u16LNBEnablePinNum, p_dev->u8LNBEnablePinInv) != FALSE)
            {
                p_dev->e_status = E_CLOSE;
                CurLNBPwr = E_LNB_POWER_OFF;
                ret = TRUE;
            }
            else
            {
                DISH_PRINT("DISH Disable fail !\n");
            }
        }
        else
        {
            DISH_PRINT("DISH Enable Pin is NULL !\n");
        }
    }
    else
    {
        // Enable pin pull high
        if(LNBEnablePinSupport)
        {
            if(_SetGPIOHigh(p_dev->u16LNBEnablePinNum, p_dev->u8LNBEnablePinInv) != FALSE)
            {
                p_dev->e_status = E_WORKING;
                ret = TRUE;
            }
            else
            {
                DISH_PRINT("DISH Enable fail !\n");
                ret = FALSE;
            }
        }
        else
        {
            p_dev->e_status = E_WORKING;
            DISH_PRINT("DISH Enable Pin is NULL !\n");
        }

        switch (eLNBPower)
        {
            case E_LNB_POWER_19V:
                DBG_DISH(DISH_PRINT(" MDrv_DISH_SetLNBPower E_LNB_POWER_19V !\n"));
                ret &= _SetGPIOHigh(p_dev->u16LNBSelectPin1Num, p_dev->u8LNBSelectPin1Inv);
                if(LNBSelectPin2Support)
                {
                    ret &= _SetGPIOHigh(p_dev->u16LNBSelectPin2Num, p_dev->u8LNBSelectPin2Inv);
                }
                if(ret == TRUE)
                {
                    if(LNBSelectPin2Support)
                    {
                        CurLNBPwr = E_LNB_POWER_19V;
                    }
                    else
                    {
                        CurLNBPwr = E_LNB_POWER_18V;
                    }
                }
                break;
            case E_LNB_POWER_18V:
                DBG_DISH(DISH_PRINT(" MDrv_DISH_SetLNBPower E_LNB_POWER_18V !\n"));
                ret &= _SetGPIOHigh(p_dev->u16LNBSelectPin1Num, p_dev->u8LNBSelectPin1Inv);
                if(LNBSelectPin2Support)
                {
                    ret &= _SetGPIOLow(p_dev->u16LNBSelectPin2Num, p_dev->u8LNBSelectPin2Inv);
                }
                if(ret == TRUE)
                    CurLNBPwr = E_LNB_POWER_18V;
                break;
            case E_LNB_POWER_14V:
                DBG_DISH(DISH_PRINT(" MDrv_DISH_SetLNBPower E_LNB_POWER_14V !\n"));
                ret &= _SetGPIOLow(p_dev->u16LNBSelectPin1Num, p_dev->u8LNBSelectPin1Inv);
                if(LNBSelectPin2Support)
                {
                    ret &= _SetGPIOHigh(p_dev->u16LNBSelectPin2Num, p_dev->u8LNBSelectPin2Inv);
                }
                if(ret == TRUE)
                {
                    if(LNBSelectPin2Support)
                    {
                        CurLNBPwr = E_LNB_POWER_14V;
                    }
                    else
                    {
                        CurLNBPwr = E_LNB_POWER_13V;
                    }
                }
                break;
            case E_LNB_POWER_13V:
                DBG_DISH(DISH_PRINT(" MDrv_DISH_SetLNBPower E_LNB_POWER_13V !\n"));
                ret &= _SetGPIOLow(p_dev->u16LNBSelectPin1Num, p_dev->u8LNBSelectPin1Inv);
                if(LNBSelectPin2Support)
                {
                    ret &= _SetGPIOLow(p_dev->u16LNBSelectPin2Num, p_dev->u8LNBSelectPin2Inv);
                }
                if(ret == TRUE)
                    CurLNBPwr = E_LNB_POWER_13V;
                break;
            default:
                DISH_PRINT("Wrong LNB power type !!!\n");
                ret = FALSE;
                break;
        }
    }
    return ret;
}
int MDrv_DISH_GetLNBPower(int minor, EN_SAT_LNBPOWER_TYPE *eLNBPower)
{
    int ret = FALSE;

    DBG_DISH(DISH_PRINT("%s is invoked\n", __FUNCTION__));
    p_dev=&(dev[minor]);

    if(p_dev != NULL)
    {
        EN_SAT_LNBPOWER_TYPE *LNBPwr = (EN_SAT_LNBPOWER_TYPE *)eLNBPower;
        *LNBPwr = CurLNBPwr;
        ret = TRUE;
    }
    else
    {
        ret = FALSE;
    }
    return ret;
}
int MDrv_DISH_Set22KOnOff(int minor, int bOn)
{
    DISH_PRINT("Not Support MDrv_DISH_Set22KOnOff !!!\n");
    return FALSE;
}
int MDrv_DISH_Get22KStatus(int minor, int *bOn)
{
    DISH_PRINT("Not Support MDrv_DISH_Set22KOnOff !!!\n");
    return FALSE;
}
int MDrv_DISH_IsOverCurrent(int minor)
{
    DISH_PRINT("Not Support MDrv_DISH_IsOverCurrent !!!\n");
    return FALSE;
}
int MDrv_DISH_Suspend(void)
{
    U8 i = 0;

    DBG_DISH(DISH_PRINT("%s is invoked\n", __FUNCTION__));
    for (i=0; i<MAX_DISH_DEV_NUM; i++)
    {
        if (dev[i].e_status == E_WORKING)
        {
            dev[i].e_status = E_SUSPEND;
            dev[i].e_lnbpower = CurLNBPwr;
            if(LNBEnablePinSupport)
            {
                _SetGPIOLow(dev[i].u16LNBEnablePinNum, dev[i].u8LNBEnablePinInv);
            }
            _SetGPIOLow(dev[i].u16LNBSelectPin1Num, dev[i].u8LNBSelectPin1Inv);
            if(LNBSelectPin2Support)
            {
            _SetGPIOLow(dev[i].u16LNBSelectPin2Num, dev[i].u8LNBSelectPin2Inv);
            }
        }
    }
    return TRUE;
}
int MDrv_DISH_Resume(void)
{
    U8 i = 0;
    int ret = TRUE;

    DBG_DISH(DISH_PRINT("%s is invoked\n", __FUNCTION__));
    for (i=0; i<MAX_DISH_DEV_NUM; i++)
    {
        if (dev[i].e_status == E_SUSPEND)
        {
            dev[i].e_status = E_WORKING;
            if(LNBEnablePinSupport)
            {
                _SetGPIOHigh(dev[i].u16LNBEnablePinNum, dev[i].u8LNBEnablePinInv);
            }
            else
            {
                DISH_PRINT("DISH Enable Pin is NULL !\n");
            }
            switch (dev[i].e_lnbpower)
            {
                case E_LNB_POWER_19V:
                    ret &= _SetGPIOHigh(p_dev->u16LNBSelectPin1Num, p_dev->u8LNBSelectPin1Inv);
                    if(LNBSelectPin2Support)
                    {
                        ret &= _SetGPIOHigh(p_dev->u16LNBSelectPin2Num, p_dev->u8LNBSelectPin2Inv);
                    }
                    if(ret == TRUE)
                    {
                        if(LNBSelectPin2Support)
                        {
                            CurLNBPwr = E_LNB_POWER_19V;
                        }
                        else
                        {
                            CurLNBPwr = E_LNB_POWER_18V;
                        }
                    }
                    break;
                case E_LNB_POWER_18V:
                    ret &= _SetGPIOHigh(p_dev->u16LNBSelectPin1Num, p_dev->u8LNBSelectPin1Inv);
                    if(LNBSelectPin2Support)
                    {
                        ret &= _SetGPIOLow(p_dev->u16LNBSelectPin2Num, p_dev->u8LNBSelectPin2Inv);
                    }
                    if(ret == TRUE)
                        CurLNBPwr = E_LNB_POWER_18V;
                    break;
                case E_LNB_POWER_14V:
                    ret &= _SetGPIOLow(p_dev->u16LNBSelectPin1Num, p_dev->u8LNBSelectPin1Inv);
                    if(LNBSelectPin2Support)
                    {
                        ret &= _SetGPIOHigh(p_dev->u16LNBSelectPin2Num, p_dev->u8LNBSelectPin2Inv);
                    }
                    if(ret == TRUE)
                    {
                        if(LNBSelectPin2Support)
                        {
                            CurLNBPwr = E_LNB_POWER_14V;
                        }
                        else
                        {
                            CurLNBPwr = E_LNB_POWER_13V;
                        }
                    }
                    break;
                case E_LNB_POWER_13V:
                    ret &= _SetGPIOLow(p_dev->u16LNBSelectPin1Num, p_dev->u8LNBSelectPin1Inv);
                    if(LNBSelectPin2Support)
                    {
                        ret &= _SetGPIOLow(p_dev->u16LNBSelectPin2Num, p_dev->u8LNBSelectPin2Inv);
                    }
                    if(ret == TRUE)
                        CurLNBPwr = E_LNB_POWER_13V;
                    break;
                default:
                    DISH_PRINT("Wrong LNB power type !!!\n");
                    ret = FALSE;
                    break;
            }
        }
    }
    return ret;
}
int MDrv_DISH_GetDISHMdbInfo(void)
{
    DBG_DISH(DISH_PRINT("%s is invoked\n", __FUNCTION__));
    return FALSE;
}
int MDrv_DISH_WriteDISHMdbInfo(void)
{
    DBG_DISH(DISH_PRINT("%s is invoked\n", __FUNCTION__));
    return FALSE;
}



