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

#if defined(CONFIG_HAS_LED)


#include <linux/types.h>
#include <linux/fs.h>
#include <linux/io.h>

#include "mdrv_led.h"
#include "mdrv_mstypes.h"
#include "mdrv_mbx.h"

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include <mstar/mstar_chip.h>

#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/sign_of_life.h>

#if defined(CONFIG_AMAZON_METRICS_LOG)
#include <linux/metricslog.h>
#endif

static struct mutex lock;

typedef enum{
	LED_DRV_INIT = 1,
	LED_ON_1,
	LED_OFF_1,
	LED_BREATH_1,
	LED_BLINK_ONCE_1,
	LED_KEEP_BLINK_1,
	LED_ON_PERCENTAGE_1,
	LED_BLINK_ONCE_AND_ON_1,
	LED_INVERT_1,
	LED_ON_2,
	LED_OFF_2,
	LED_BREATH_2,
	LED_BLINK_ONCE_2,
	LED_KEEP_BLINK_2,
	LED_ON_PERCENTAGE_2,
	LED_BLINK_ONCE_AND_ON_2,
	LED_INVERT_2,
} LED_CONTROL;


#define INVALID_MBXCLASS(mbxClass)  (mbxClass >= E_MBX_CLASS_MAX)
#define PM_MBX_TIMEOUT      5000
#define PM_MBX_QUEUESIZE    8
#define PM_CMDIDX_LED       0x42

/* Message from SN to decide LED status */
#define LED_PWMGPIO_INVERT_1             0xB9 /* Set LED 1 invert flag */
#define LED_PWMGPIO_LIGHT_PERCENTAGE_1   0xBA            /* Turn on LED 1 with 0~100% */
#define LED_PWMGPIO_LIGHT_1   0xBB            /* Turn on LED 1		*/
#define LED_PWMGPIO_DARK_1    0xBC            /* Turn off LED 1		*/
#define LED_PWMGPIO_BREATH_1  0xBD            /* LED 1 breath			*/
#define LED_PWMGPIO_FLICKER_1 0xBE            /* LED 1 blink once		*/
#define LED_PWMGPIO_FLICKER_POWERON_1 0xBF    /* LED 1 keep blinking	*/


#define LED_PWMGPIO_INVERT_2           0xC9 /* Set LED 2 invert flag */
#define LED_PWMGPIO_LIGHT_PERCENTAGE_2 0xCA			/* Turn on LED 2 0~100% */
#define LED_PWMGPIO_LIGHT_2 0xCB			/* Turn LED 2			*/
#define LED_PWMGPIO_DARK_2 0xCC				/* Turn off LED 2		*/
#define LED_PWMGPIO_BREATH_2 0xCD			/* LED 2 breath			*/
#define LED_PWMGPIO_FLICKER_2 0xCE			/* LED 2 blink once		*/
#define LED_PWMGPIO_FLICKER_POWERON_2 0xCF	/* LED 2 keep blinking	*/
#define E_PM_CMDIDX_ACK_51ToARM 0x31  /*Need to double confirm this once get the RT_PM source code */

#define LED_1_CTRL_FLAG 0x5500
#define LED_2_CTRL_FLAG 0xAA00

static int debug_enable_led;
static int led_init_status;
static int led_1_status;
static int led_2_status;
#define LEDS_DRV_DEBUG(format, args...) do { \
if (debug_enable_led) {\
	pr_err(format, ##args);\
	} \
} while (0)


/*=============================================================================
 * Local Variables
 *============================================================================= */
static DEFINE_MUTEX(Semutex_LD);
static DEFINE_SPINLOCK(spinlock_ld);
static MS_BOOL bReceived;

#if defined(CONFIG_AMAZON_METRICS_LOG)

struct metrics_info {
	int flags;

	/* Time when system enters full suspend */
	struct timespec suspend_time;
};
static struct metrics_info info;


static void bq_log_metrics(char *msg,
	char *metricsmsg)
{
	char buf[512];
	struct timespec curr = current_kernel_time();
	/* Compute elapsed time and determine screen off or on drainage */
	struct timespec diff = timespec_sub(curr,
			info.suspend_time);

	snprintf(buf, sizeof(buf),
		"%s:def:value=0;CT;1,elapsed=%ld;TI;1:NR",
		metricsmsg,
		diff.tv_sec * 1000 + diff.tv_nsec / NSEC_PER_MSEC);
	log_to_metrics(ANDROID_LOG_INFO, "drain_metrics", buf);
	/* Mark the suspend or resume time */
	info.suspend_time = curr;
}

#endif

/*=============================================================================
 * Local Functions
 *============================================================================= */


static int MDrv_LED_RecHandler(void)
{
	MBX_Result enMbxResult = E_MBX_UNKNOW_ERROR;
	MBX_Msg stMbxCommand;
	MS_U8 u8Ack = 0;

	if (TRUE != bReceived)
		return E_MBX_SUCCESS;

	/* Waiting for message done */
	memset((void *)&stMbxCommand, 0x00, sizeof(MBX_Msg));
	spin_lock_irq(&spinlock_ld);
	enMbxResult = MDrv_MBX_RecvMsg(E_MBX_CLASS_PM_NOWAIT, &stMbxCommand, \
		MBX_CHECK_NORMAL_MSG, 0);
	spin_unlock_irq(&spinlock_ld);
	printk("   rec enMbxResult:%d \n", enMbxResult);

   /*check result */
	if (enMbxResult == E_MBX_SUCCESS) {
		if ((stMbxCommand.u8Ctrl != 0) && (stMbxCommand.u8Ctrl != 1)) {
			enMbxResult = E_MBX_ERR_NOT_IMPLEMENTED;
			pr_err("Not Implemented!\n");
		}

		switch (stMbxCommand.u8Index) {
		case E_PM_CMDIDX_ACK_51ToARM:
			u8Ack = (MS_U8)stMbxCommand.u8Parameters[0];
			break;
		default:
			break;
		}
	} else if (E_MBX_ERR_TIME_OUT == enMbxResult) {
		pr_err("Handler receive timeout! nMbxResult:%d \n", enMbxResult);
	} else {
		pr_err("Handler receive error nMbxResult:%d \n", enMbxResult);
	}

	return enMbxResult;
}


static MS_S8 MDrv_LD_SetupMbx(void)
{
	MBX_Result enMbxResult = E_MBX_UNKNOW_ERROR;

	enMbxResult = MDrv_MBX_Init(E_MBX_CPU_MIPS, E_MBX_ROLE_HK, PM_MBX_TIMEOUT);
	if (E_MBX_SUCCESS != enMbxResult) {
		pr_err("!! MDrv_MBX_Init fail nMbxResult:%d \n", enMbxResult);
		return -1;
	} else {
		extern int MDrv_MBX_NotifyMsgRecCbFunc(void *);
		MDrv_MBX_Enable(TRUE);
		MDrv_MBX_NotifyMsgRecCbFunc(MDrv_LED_RecHandler);

		enMbxResult = MDrv_MBX_RegisterMSG(E_MBX_CLASS_PM_NOWAIT, PM_MBX_QUEUESIZE);
		if (E_MBX_SUCCESS != enMbxResult) {
			pr_err("!!MDrv_MBX_RegisterMSG fail nMbxResult:%d \n", enMbxResult);
		}

		return enMbxResult;
	}
}




MBX_Result LED_DRV_Ctrl(MS_U16 var, const unsigned int parameter)
{
	MBX_Msg stMbxCommand;
	MS_U16 command;
	MBX_Result enMbxResult = E_MBX_UNKNOW_ERROR;
	printk("LED_DRV_Ctrl var(%d) parameter(%d)\n", var, parameter);
	if (led_init_status == 0) {
		if (MDrv_LD_SetupMbx() != E_MBX_SUCCESS) {
			pr_err("MDrv_LD_SetupMbx failed\n");
		}
		led_init_status = 1;
	}
	/*send msg to PM */
	memset((void *)&stMbxCommand, 0, sizeof(MBX_Msg));
	stMbxCommand.eRoleID = E_MBX_ROLE_PM;               /* Do Not Change */
	stMbxCommand.eMsgType = E_MBX_MSG_TYPE_INSTANT;     /* Do Not Change */
	stMbxCommand.u8Ctrl = 0;                            /* Do Not Change */
	stMbxCommand.u8MsgClass = E_MBX_CLASS_PM_NOWAIT;    /* Do Not Change */
	stMbxCommand.u8ParameterCount = 2;                  /* Implement by yourself Between ARM & PM */
	stMbxCommand.u8Parameters[0] = (MS_U8)(0xAA);
	stMbxCommand.u8Parameters[1] = (MS_U8)(0xBB);       /* reserve for support freq in Breath and Flickr mode */

	switch (var) {
	case LED_DRV_INIT:
		if (led_init_status == 0) {
			if (MDrv_LD_SetupMbx() != E_MBX_SUCCESS) {
				pr_err("MDrv_LD_SetupMbx failed\n");
				return -1;
			}
			led_init_status = 1;
		}
		return 1;
	case LED_ON_1:
		command = LED_PWMGPIO_LIGHT_1;
		led_1_status = 1;
		printk("Turn on LED 1\n");
		break;
	case LED_OFF_1:
		command = LED_PWMGPIO_DARK_1;
		led_1_status = 0;
		printk("Turn off LED 1\n");
		break;
	case LED_BREATH_1:
		command = LED_PWMGPIO_BREATH_1;
		if (led_1_status == 0) {
			command |= LED_1_CTRL_FLAG;
		}
		printk("Set LED 1 to breath mode \n");
		break;

	case LED_BLINK_ONCE_1:
		command = LED_PWMGPIO_FLICKER_1;
		if (led_1_status == 0) {
			command |= LED_1_CTRL_FLAG;
		}
		printk("LED 1 blink once 0x%x \n", command);
		break;
	case LED_KEEP_BLINK_1:
		command = LED_PWMGPIO_FLICKER_POWERON_1;
		if (led_1_status == 0) {
			command |= LED_1_CTRL_FLAG;
		}
		stMbxCommand.u8Parameters[0] = (MS_U8)(parameter);
		printk("LED 1 keep blinking parameter=%d\n", parameter);
		break;

	case LED_ON_PERCENTAGE_1:
		command = LED_PWMGPIO_LIGHT_PERCENTAGE_1;
		if (led_1_status == 0) {
			command |= LED_1_CTRL_FLAG;
		}
		stMbxCommand.u8Parameters[0] = (MS_U8)(parameter);
		led_1_status = 1;
		printk("Turn on LED 1 percentage=%d%%\n", parameter);
		break;

	case LED_PWMGPIO_INVERT_1:
		command = LED_PWMGPIO_INVERT_1;
		stMbxCommand.u8Parameters[0] = (MS_U8)(parameter);
		pr_info("Turn on LED 1 invert=%d\n", parameter);
		break;

	case LED_ON_2:
		command = LED_PWMGPIO_LIGHT_2;
		led_2_status = 1;
		printk("Turn on LED 2\n");
		break;
	case LED_OFF_2:
		command = LED_PWMGPIO_DARK_2;
		led_2_status = 0;
		printk("Turn off  LED 2 \n");
		break;
	case LED_BREATH_2:
		command = LED_PWMGPIO_BREATH_2;
		if (led_2_status == 0) {
			command |= LED_2_CTRL_FLAG;
		}
		printk("LED 2 breath \n");
		break;
	case LED_BLINK_ONCE_2:
		command = LED_PWMGPIO_FLICKER_2;
		if (led_2_status == 0) {
			command |= LED_2_CTRL_FLAG;
		}
		printk("LED 2 blink once \n");
		break;
	case LED_KEEP_BLINK_2:
		command = LED_PWMGPIO_FLICKER_POWERON_2;
		if (led_2_status == 0) {
			command |= LED_2_CTRL_FLAG;
		}
		printk("LED 2 keep blinkig \n");
		break;
	case LED_ON_PERCENTAGE_2:
		command = LED_PWMGPIO_LIGHT_PERCENTAGE_2;
		if (led_2_status == 0) {
			command |= LED_2_CTRL_FLAG;
		}
		stMbxCommand.u8Parameters[0] = (MS_U8)(parameter);
		led_2_status = 1;
		printk("Turn on LED 2 percentage=%d%%\n", parameter);
		break;

	case LED_PWMGPIO_INVERT_2:
		command = LED_PWMGPIO_INVERT_2;
		stMbxCommand.u8Parameters[0] = (MS_U8)(parameter);
		pr_info("Turn on LED 2 invert=%d\n", parameter);
		break;

	default:
		pr_err("Wrong LED control command \n");
		return 0;
	}

	if (command & LED_1_CTRL_FLAG) {
		stMbxCommand.u8Index = LED_PWMGPIO_LIGHT_1;
		enMbxResult = MDrv_MBX_SendMsg(&stMbxCommand);
		printk("LED_DRV_Ctrl, Turn on led 1 firstly, result is 0x%x\n", enMbxResult);
		led_1_status = 1;
		udelay(5);
	}
	if (command & LED_2_CTRL_FLAG) {
		stMbxCommand.u8Index = LED_PWMGPIO_LIGHT_2;
		enMbxResult = MDrv_MBX_SendMsg(&stMbxCommand);
		printk("LED_DRV_Ctrl, Turn on led 2 firstly, result is 0x%x\n", enMbxResult);
		led_2_status = 1;
		udelay(5);
	}
	stMbxCommand.u8Index = (U8)(command&0x00FF);                     /* Implement by yourself Between ARM & PM */
	enMbxResult = MDrv_MBX_SendMsg(&stMbxCommand);
	return enMbxResult;
}

int MDrv_LED_Resume(void)
{
#if defined(CONFIG_IDME)
    if (strstr(saved_command_line, "product_name=brandenburg") != NULL) {
        /* Set the led inverse for brandenburg board */
        LED_DRV_Ctrl(LED_INVERT_1, 1);
        pr_info("[%s:%d] Set led 1 inverse\n", __FUNCTION__, __LINE__);
    }
#endif
    return 0;
}

/****************************************************************************
 * driver functions
 ***************************************************************************/

struct margo_led {
	struct led_classdev cdev;
	const char *name;

};
static struct margo_led leds[] = {
	{
		.name = "tv_led",
	},
#if defined(CONFIG_AMAZON_METRICS_LOG)
	{
		.name = "dummy_light",
	}
#endif
};

static ssize_t led_set(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t size)
{
	unsigned int action = 0;
	unsigned int parameter = 0;
	int readCount = 0;
	int ret = 0;
	mutex_lock(&lock);
	if (buf != NULL && size != 0) {
		printk("[led action] buf is %s and size is %zd\n", buf, size);
		readCount = sscanf(buf, "%d", &action);
		if (readCount != 1) {
			ret = -EINVAL;
			goto led_set_end;
		}
		if (action > 0xff) {
			ret = -EINVAL;
			goto led_set_end;
		}
		if (led_init_status == 0) {
			LED_DRV_Ctrl(LED_DRV_INIT, parameter);
			led_init_status = 1;
		}

		if (action == LED_BLINK_ONCE_AND_ON_1) {
			if (led_1_status == 1) {
				LED_DRV_Ctrl(LED_OFF_1, 0);
				udelay(100);
			}
			LED_DRV_Ctrl(LED_ON_1, 0);
			msleep(100);
			LED_DRV_Ctrl(LED_OFF_1, 0);
			msleep(400);

			parameter = 15;
			LED_DRV_Ctrl(LED_ON_PERCENTAGE_1, parameter);
			goto led_set_end;
		}

		if (action == LED_KEEP_BLINK_1) {
			parameter = 1;
			LED_DRV_Ctrl(LED_KEEP_BLINK_1, parameter);
			goto led_set_end;
		}

		if (action == LED_BLINK_ONCE_AND_ON_2) {
			LED_DRV_Ctrl(LED_BLINK_ONCE_2, parameter);
			parameter = 15;
			LED_DRV_Ctrl(LED_ON_PERCENTAGE_2, parameter);
			goto led_set_end;
		}

		if ((action == LED_ON_PERCENTAGE_1) || (action == LED_ON_PERCENTAGE_2)) {
			readCount = sscanf(buf, "%d %d", &action, &parameter);
			if (readCount != 2) {
				ret = -EINVAL;
				goto led_set_end;
			}
		}

		ret = LED_DRV_Ctrl(action, parameter);
	}
led_set_end:
	mutex_unlock(&lock);
	return size;
}
static DEVICE_ATTR(tv_led_set, 0220, NULL, led_set);

#if defined(CONFIG_AMAZON_METRICS_LOG)

static ssize_t dummy_light_set(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t size)
{
	unsigned int action = 0;
	int ret = 0;
	if (buf != NULL && size != 0) {
		printk("[dummy light action] buf is %s and size is %zd\n", buf, size);
		ret = sscanf(buf, "%d", &action);
		if (ret != 1)
			return -EINVAL;
	}
	switch (action) {
	case 0:
		mstar_set_screen_flag();
		pr_info("backlight is off \n");
		bq_log_metrics("Screen on drainage", "screen_on_drain");
		break;
	case 1:
		mstar_clear_screen_flag();
		pr_info("backlight is on\n");
		bq_log_metrics("Screen off drainage", "screen_off_drain");
		break;
	default:
		break;
	}

	return size;
}
static DEVICE_ATTR(light_set, 0220, NULL, dummy_light_set);
#endif

static int mstar_leds_probe(struct platform_device *pdev)
{
	int i;
	int ret, rc;
#if defined(CONFIG_AMAZON_METRICS_LOG)
	info.suspend_time = current_kernel_time();
#endif

	printk("[LED]%s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(leds); i++) {

		leds[i].cdev.name = leds[i].name;
		ret = led_classdev_register(&pdev->dev, &leds[i].cdev);
		if (ret < 0)
			goto err;
		if (strcmp(leds[i].name, "tv_led") == 0) {
			rc = device_create_file(leds[i].cdev.dev, &dev_attr_tv_led_set);
			if (rc)
				pr_err("[LED]device_create_file led_pattern fail!\n");
		}
	#if defined(CONFIG_AMAZON_METRICS_LOG)
		if (strcmp(leds[i].name, "dummy_light") == 0) {
			rc = device_create_file(leds[i].cdev.dev, &dev_attr_light_set);
			if (rc)
				pr_err("[LED]device_create_file dummy_light fail!\n");
		}
	#endif
	}
	led_init_status = 0;
	mutex_init(&lock);
	return 0;

 err:
	if (i) {
		for (i = i - 1; i >= 0; i--) {
			led_classdev_unregister(&leds[i].cdev);
		}
	}
	return ret;
}

static int mstar_leds_remove(struct platform_device *pdev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(leds); i++) {
		led_classdev_unregister(&leds[i].cdev);
	}

	return 0;
}

static struct platform_driver mstar_leds_driver = {
	.driver = {
		   .name = "leds-mstar",
		   .owner = THIS_MODULE,
		   },
	.probe = mstar_leds_probe,
	.remove = mstar_leds_remove,
};

static struct platform_device mstar_leds_device = {
	.name = "leds-mstar",
	.id = -1
};

static int __init mstar_leds_init(void)
{
	int ret;

	printk("[LED]%s\n", __func__);
	ret = platform_device_register(&mstar_leds_device);
	if (ret)
		pr_info("[LED]mstar_leds_init:dev:E%d\n", ret);

	ret = platform_driver_register(&mstar_leds_driver);

	if (ret) {
		pr_info("[LED]mstar_leds_init:drv:E%d\n", ret);
		platform_device_unregister(&mstar_leds_device);
		return ret;
	}

	return ret;
}

static void __exit mstar_leds_exit(void)
{
	platform_driver_unregister(&mstar_leds_driver);
	platform_device_unregister(&mstar_leds_device);
}



late_initcall(mstar_leds_init);
module_exit(mstar_leds_exit);

MODULE_AUTHOR("Mstar.");
MODULE_DESCRIPTION("LED driver for Mstar chip");
MODULE_LICENSE("GPL");
MODULE_ALIAS("leds-mstar");
#endif

