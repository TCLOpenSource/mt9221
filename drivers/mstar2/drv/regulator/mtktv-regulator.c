#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#if CONFIG_MSTAR_PWM
#include "mdrv_pwm.h"
#endif



/*********************************************************************
 *                         External Functions                        *
 *********************************************************************/
extern s32 MDrv_HW_IIC_WriteBytes(u8 u8Port, u8 u8SlaveIdIIC, u8 u8AddrSizeIIC, u8 *pu8AddrIIC, u32 u32BufSizeIIC, u8 *pu8BufIIC);
extern s32 MDrv_HW_IIC_ReadBytes(u8 u8Port, u8 u8SlaveIdIIC, u8 u8AddrSizeIIC, u8 *pu8AddrIIC, u32 u32BufSizeIIC, u8 *pu8BufIIC);
extern void MDrv_GPIO_Set_High(u8 u8IndexGPIO);
extern void MDrv_GPIO_Set_Low(u8 u8IndexGPIO);
extern void MDrv_GPIO_Set_Input(u8 u8IndexGPIO);

/*********************************************************************
 *                         Private Structure                         *
 *********************************************************************/
typedef enum {
	E_REGULATOR_PRADO = 0,
	E_REGULATOR_PWM   = 1,
	E_REGULATOR_GPIO  = 2,
	E_REGULATOR_NONE  = 3,
} E_REGULATOR_TYPE;

typedef enum {
	E_GPIO_IN = 0,
	E_GPIO_OUT_LOW = 1,
	E_GPIO_OUT_HIGH = 2,
	E_GPIO_NONE = 3,
} E_GPIO_MODE;


typedef struct {
	u8 *gpio_num;
	E_GPIO_MODE *mode;
} ST_GPIO_PARM;


/*********************************************************************
 *                         Private Macro                             *
 *********************************************************************/
#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define BASE_ADDRESS                    (0xfd000000)
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define BASE_ADDRESS                    (mstar_pm_base)
#endif

#define REGULATOR_FIND_BY_NAME(parent,name,val) \
	val = of_find_node_by_name(parent,name); \
	if (val == NULL) { \
		pr_err("\033[37m[mtktv-regulator] can't find dts node: %s\033[m\n",name); \
		BUG_ON(1); \
	}


#define REGULATOR_READ_U32(parent,name,val) \
	if (of_property_read_u32(parent,name,&val)) { \
		pr_err("\033[37m[mtktv-regulator] can't get u32 value in parent node: %s\033[m\n",name); \
		BUG_ON(1); \
	}

#define REGULATOR_READ_STR(parent,name,val) \
	if (of_property_read_string(parent,name,&val)) { \
		pr_err("\033[37m[mtktv-regulator] can't get string in parent node: %s\033[m\n",name); \
		BUG_ON(1); \
	}

#define REGULATOR_READ_U32_ARRAY(parent,name,val,size) \
	if (of_property_read_u32_array(parent,name,val,size)) { \
		pr_err("\033[37m[mtktv-regulator] can't get u32 array in parent node: %s\033[m\n",name); \
		BUG_ON(1); \
	}

#define HW_IIC_WriteByte(u8AddrSizeIIC,pu8AddrIIC,u32BufSizeIIC,pu8BufIIC,ret) \
	if (MDrv_HW_IIC_WriteBytes(port,address,u8AddrSizeIIC,pu8AddrIIC,u32BufSizeIIC,pu8BufIIC) < 0) { \
		pr_err("\033[37m[mtktv-regulator][%d] IIC Write Fail!\033[m\n",__LINE__); \
		ret = -1; \
	}

#define HW_IIC_ReadByte(u8AddrSizeIIC,pu8AddrIIC,u32BufSizeIIC,pu8BufIIC,ret) \
	if (MDrv_HW_IIC_ReadBytes(port,address,u8AddrSizeIIC,pu8AddrIIC,u32BufSizeIIC,pu8BufIIC) < 0) { \
		pr_err("\033[37m[mtktv-regulator][%d] IIC Read Fail!\033[m\n",__LINE__); \
		ret = -1; \
	}

#define sample_cnt (128)

/*********************************************************************
 *                         Private Data                              *
 *********************************************************************/
DEFINE_MUTEX(INIT_MUTEX);

static u8 regulator_parsed = 0;
static u8 regulator_init = 0;
static E_REGULATOR_TYPE type = E_REGULATOR_NONE;
static u32 default_power = 0;
static u32 min_power = 0;
static u32 max_power = 0;

// prado
static u32 address = 0;
static u32 id = 0;
static u32 power_shift = 0;
static u32 power_step = 0;
static u32 port = 0;

// pwm
static u32 channel = 0;
static u32 scale = 0;

// gpio
static u32 power_level = 0;
static u32 *power = NULL;
static u32 *num_of_pins = NULL;
static ST_GPIO_PARM *parm = NULL;

/*********************************************************************
 *                          Local Function                          *
 *********************************************************************/

#define regulator_writew(data, addr) \
	writew(data, (void __iomem *)(BASE_ADDRESS + (addr << 1)))

#define regulator_readw(addr) \
	readw((void __iomem *)(BASE_ADDRESS + (addr << 1)))

static int mtktv_get_voltage_bysar(void)
{
#if defined(CONFIG_MSTAR_M7332) || defined(CONFIG_MSTAR_M7632) || defined(CONFIG_MSTAR_M7642)
	u32 dwRegisterBackup = 0, dwTempData = 0;
	dwRegisterBackup = regulator_readw(0x001400);
	regulator_writew(0x0AB0, 0x001400);
	regulator_writew(0x100, 0x001404);
	usleep_range(500,500);
	regulator_writew(regulator_readw(0x001400) | (0x1 << 14), 0x001400);
	usleep_range(500,500);
	dwTempData = regulator_readw(0x001480);
	regulator_writew(0x0, 0x001404);
	regulator_writew(dwRegisterBackup, 0x001400);
	dwTempData = dwTempData * 3300/1024;
#else
	u32 dwTempData = 0;
	regulator_writew(0x0A20, 0x001400);
	regulator_writew(regulator_readw(0x00140A) & ~(0x3), 0x00140A);
	regulator_writew(regulator_readw(0x001432) & ~(0x1 << 8), 0x001432);
	udelay(300);
	regulator_writew(regulator_readw(0x001400) | (0x1 << 14), 0x001400);
	while( (regulator_readw(0x001400) & (0x1 << 14)) != 0 );
	dwTempData = regulator_readw(0x001490);
	dwTempData = dwTempData * 2000/1024;
#endif
	return dwTempData;
}

static int mtktv_set_voltage(int power)
{
	switch (type) {
		case E_REGULATOR_PRADO:
		{
			u32  dwRegisterValue = 0;
			u32  dwOriginalCpuPowerVoltage = 0;
			u32  dwSourceRegisterSetting = 0;
			u32  dwTargetRegisterSetting = 0;
			u8   ret = 0;
			u8   byTargetRegAddress[5] = {
				0xFF, 0xFF, 0xFF, 0xFF, 0xFF
			};
			u8   byTargetData[5] = {
				0xFF, 0xFF, 0xFF, 0xFF, 0xFF
			};

			power /= 10;
			byTargetRegAddress[0] = 0x10;
			byTargetRegAddress[1] = (0x06 << 1);
			HW_IIC_ReadByte(2,byTargetRegAddress,2,byTargetData,ret);
			dwOriginalCpuPowerVoltage = (unsigned int) byTargetData[1] + power_shift;
			if (dwOriginalCpuPowerVoltage > max_power || dwOriginalCpuPowerVoltage < min_power)
				dwOriginalCpuPowerVoltage = max_power;
			dwSourceRegisterSetting = dwOriginalCpuPowerVoltage - power_shift;
			dwTargetRegisterSetting = power - power_shift;
			if (power > dwOriginalCpuPowerVoltage) {
				for (;dwSourceRegisterSetting <= dwTargetRegisterSetting; dwSourceRegisterSetting += power_step) {
					dwRegisterValue = dwSourceRegisterSetting;
					byTargetRegAddress[0] = 0x10;
					byTargetRegAddress[1] = (0x06 << 1);
					byTargetRegAddress[2] = 0x10;
					byTargetRegAddress[3] = dwRegisterValue;
					HW_IIC_WriteByte(0,byTargetRegAddress,4,byTargetRegAddress,ret);
				}
			}
			else if (power < dwOriginalCpuPowerVoltage)
			{
				for (;dwSourceRegisterSetting >= dwTargetRegisterSetting; dwSourceRegisterSetting -= power_step) {
					dwRegisterValue = dwSourceRegisterSetting;
					byTargetRegAddress[0] = 0x10;
					byTargetRegAddress[1] = (0x06 << 1);
					byTargetRegAddress[2] = 0x10;
					byTargetRegAddress[3] = dwRegisterValue;
					HW_IIC_WriteByte(0,byTargetRegAddress,4,byTargetRegAddress,ret);
				}
			}
			if (dwSourceRegisterSetting != dwTargetRegisterSetting) {
				dwRegisterValue = (power - power_shift);
				byTargetRegAddress[0] = 0x10;
				byTargetRegAddress[1] = (0x06 << 1);
				byTargetRegAddress[2] = 0x10;
				byTargetRegAddress[3] = dwRegisterValue;
				HW_IIC_WriteByte(0,byTargetRegAddress,4,byTargetRegAddress,ret);
			}
			break;
		}
		case E_REGULATOR_PWM:
		{
#ifdef CONFIG_MSTAR_PWM
#if defined(CONFIG_MSTAR_MT5862)
			u32 adjust_duty = 0x85;
#elif defined(CONFIG_MSTAR_MT5867)
			u32 adjust_duty = 0x65;
#else
			u32 adjust_duty = 0x85;
#endif
			if (power > default_power/1000) {
				if ( ((power - default_power/1000)*1000/scale) > adjust_duty)
					adjust_duty = 0;
				else
					adjust_duty -= (power - default_power/1000)*1000/scale;
			}
			else {
				adjust_duty += (default_power/1000 - power)*1000/scale;
			}
			if (channel == 0)
				MDrv_PWM_DutyCycle(E_PVR_PWM_CH0, adjust_duty);
			else
				MDrv_PWM_DutyCycle(E_PVR_PWM_CH1, adjust_duty);
			msleep(1);
#endif
			break;
		}
		case E_REGULATOR_GPIO:
		{
			break;
		}
		case E_REGULATOR_NONE:
		default:
			pr_err("[mtktv-regulator] not support power type!\n");
	}
	return 0;
}


static int mtktv_regulator_parse(void)
{
	struct device_node *node, *child_node, *gchild_node;
	const char *name;
	int ret = 0, idx = 0, idx2 = 0;
	char node_name[100];

	REGULATOR_FIND_BY_NAME(NULL,"mtktv-regulator",node);
	REGULATOR_READ_U32(node,"regulator-default",default_power);
	REGULATOR_READ_U32(node,"regulator-min-microvolt",min_power);
	REGULATOR_READ_U32(node,"regulator-max-microvolt",max_power);
	REGULATOR_READ_STR(node,"regulator-type", name);
	pr_info("[mtktv-regulator] regulator-default = %d, min = %d, max = %d\n", default_power, min_power, max_power);
	min_power /= 10000;
	max_power /= 10000;

	if (strcmp((const char *)name,"prado") == 0) {
		type = E_REGULATOR_PRADO;
		REGULATOR_READ_U32(node,"regulator-id",id);
		REGULATOR_READ_U32(node,"regulator-step",power_step);
		REGULATOR_READ_U32(node,"regulator-shift",power_shift);
		REGULATOR_READ_U32(node,"regulator-port",port);
		REGULATOR_READ_U32(node,"regulator-addr",address);
		pr_info("[mtktv-regulator] regulator type: pardo\n");
		pr_info("[mtktv-regulator] id = 0x%x, step  = %d, shift = %d, port = %d, address = 0x%x\n", id, power_step, power_shift,port,address);
	}
	else if (strcmp((const char *)name,"pwm") == 0) {
		type = E_REGULATOR_PWM;
		REGULATOR_READ_U32(node,"regulator-channel",channel);
		REGULATOR_READ_U32(node,"regulator-scale",scale);
		pr_info("[mtktv-regulaotr] regulator type: pwm\n");
		pr_info("[mtktv-regulator] channel = %d,scale = %d\n", channel,scale);
	}
	else if (strcmp((const char *)name,"gpio") == 0) {
		type = E_REGULATOR_GPIO;
		REGULATOR_READ_U32(node,"regulator-level",power_level);
		power = (u32 *)kmalloc(sizeof(u32)*power_level,GFP_KERNEL);
		num_of_pins = (u32 *)kmalloc(sizeof(u32)*power_level,GFP_KERNEL);
		parm = (ST_GPIO_PARM *)kmalloc(sizeof(ST_GPIO_PARM)*power_level,GFP_KERNEL);
		REGULATOR_FIND_BY_NAME(node,"regulator-parm",child_node);

		for(idx = 0;idx < power_level;idx++) {
			memset(node_name,0,100);
			snprintf((char *)name, 10, "control_%d", idx);
			REGULATOR_FIND_BY_NAME(child_node,(const char *)name,gchild_node);
			REGULATOR_READ_U32(gchild_node,"power", power[idx]);
			REGULATOR_READ_U32(gchild_node,"pins", num_of_pins[idx]);
			parm[idx].gpio_num = (u8 *)kmalloc(sizeof(u8)*num_of_pins[idx], GFP_KERNEL);
			parm[idx].mode = (E_GPIO_MODE *)kmalloc(sizeof(E_GPIO_MODE)*num_of_pins[idx], GFP_KERNEL);
			REGULATOR_READ_U32_ARRAY(gchild_node,"gpio_num", (U32 *)parm[idx].gpio_num, num_of_pins[idx]);
			REGULATOR_READ_U32_ARRAY(gchild_node,"gpio_mode", parm[idx].mode, num_of_pins[idx]);
		}

		pr_info("[mtktv-regulator] regulator typ: gpio\n");
		pr_info("[mtktv-regulator] power level: %d\n", power_level);
		for(idx = 0; idx < power_level; idx++) {
			pr_info("================================================\n");
			pr_info("[mtktv-regulator] power: %d, num of pins: %d\n", power[idx], num_of_pins[idx]);
			for(idx2 = 0; idx2 < num_of_pins[idx]; idx2++)
				pr_info("[mtktv-regulator] gpio num: %d, gpio mode: %d\n",parm[idx].gpio_num[idx2], parm[idx].mode[idx2]);
		}
	}
	else {
		type = E_REGULATOR_NONE;
		pr_err("[mtktv-regualtor] undefined regulator type!\n");
		ret = 1;
	}

	return ret;
}

static int mtktv_pmic_init(void)
{
	int ret = 0;
#if defined(CONFIG_MSTAR_PWM)
	PWM_ChNum num = E_PVR_PWM_CH1;
	u32 detect_vol = 0;
	u32 index = 0,total = 0;
	u16 val = 0;
#endif

	if (regulator_parsed == 0) {
		ret = mtktv_regulator_parse();
		if (ret)
			pr_err("[mtktv-regualtor] parsing fail\n");
		else
			regulator_parsed = 1;
	}


	mutex_lock(&INIT_MUTEX);
	if (regulator_init == 1) {
		mutex_unlock(&INIT_MUTEX);
		return 0;
	}

	switch(type) {
		case E_REGULATOR_PRADO:
		{
			u8  byTargetRegAddress[5] = {
				0x53, 0x45, 0x52, 0x44, 0x42
			};
			u8  byTargetData[5] = {
				0xFF, 0xFF, 0xFF, 0xFF, 0xFF
			};
			u32 ChipId = 0, dwRegisterValue = 0;
			HW_IIC_WriteByte(0,byTargetRegAddress,5,byTargetRegAddress,ret);
			if (ret == -1) {
				pr_err("[mtktv-regulator][%d] power password verify fail\n",__LINE__);
				//return ret;
			}
			byTargetRegAddress[0] = 0x7F;
			HW_IIC_WriteByte(0,byTargetRegAddress,1,byTargetRegAddress,ret);
			byTargetRegAddress[0] = 0x7D;
			HW_IIC_WriteByte(0,byTargetRegAddress,1,byTargetRegAddress,ret);
			byTargetRegAddress[0] = 0x50;
			HW_IIC_WriteByte(0,byTargetRegAddress,1,byTargetRegAddress,ret);
			byTargetRegAddress[0] = 0x55;
			HW_IIC_WriteByte(0,byTargetRegAddress,1,byTargetRegAddress,ret);
			byTargetRegAddress[0] = 0x35;
			HW_IIC_WriteByte(0,byTargetRegAddress,1,byTargetRegAddress,ret);
			byTargetRegAddress[0] = 0x10;
			byTargetRegAddress[1] = 0xc0;
			HW_IIC_ReadByte(2,byTargetRegAddress,2,byTargetData,ret);

			if (ret != -1) {
				pr_info("[mtktv-regulator][%d] power ic read successful\n",__LINE__);
				ChipId = (unsigned int) byTargetData[1];
			}

			if (ChipId == id) {
				pr_info("[mtktv-regulator][%d] set default power(%d0mV)\n",__LINE__,default_power/10000);
				dwRegisterValue = (default_power/10000 - power_shift);
				byTargetRegAddress[0] = 0x10;
				byTargetRegAddress[1] = (0x06 << 1);
				byTargetRegAddress[2] = 0x10;
				byTargetRegAddress[3] = dwRegisterValue;
				HW_IIC_WriteByte(0,byTargetRegAddress,4,byTargetRegAddress,ret);
				byTargetRegAddress[0] = 0x10;
				byTargetRegAddress[1] = (0x05 << 1);
				byTargetRegAddress[2] = 0x40;
				byTargetRegAddress[3] = 0x00;
				HW_IIC_WriteByte(0,byTargetRegAddress,4,byTargetRegAddress,ret);
				byTargetRegAddress[0] = 0x10;
				byTargetRegAddress[1] = (0x0C << 1);
				byTargetRegAddress[2] = 0xbe;
				byTargetRegAddress[3] = 0xaf;
				HW_IIC_WriteByte(0,byTargetRegAddress,4,byTargetRegAddress,ret);
				if (ret != -1)
					pr_info("[mtktv-regulator][%d] power initialization successful\n",__LINE__);
			}
			else {
				pr_err("[mtktv-regulator][%d] the power ic is not support\n",__LINE__);
				ret = -1;
			}
			break;
		}
		case E_REGULATOR_PWM:
		{
#if defined(CONFIG_MSTAR_PWM)
			pr_info("[mtktv-regulator] PWM DAC INIT\n");
			num = E_PVR_PWM_CH1;

			detect_vol = 0;
			val = 0;
			if (channel == 0)
				num = E_PVR_PWM_CH0;
			else
				num = E_PVR_PWM_CH1;

			regulator_writew(regulator_readw(0x1522E0) | (0x1 << 2), 0x1522E0);
			regulator_writew(regulator_readw(0x152204) | (0x1 << 8), 0x152204);
			MDrv_PWM_Period(num, 0x1AF);
			MDrv_PWM_Shift(num,0);
			MDrv_PWM_AutoCorrect(num,1);

#if defined(CONFIG_MSTAR_MT5862)
			MDrv_PWM_DutyCycle(num, 0x85);
			val = regulator_readw(0x101E0A);

			/* reg_pwm_dac0_mode */
			val |= 0x3;

			/* reg_pwm_dac0_oen */
			val &= ~(0x1 << 4);

			regulator_writew(val, 0x101E0A);
#elif defined(CONFIG_MSTAR_MT5867)
			MDrv_PWM_DutyCycle(num, 0x65);
			val = regulator_readw(0x101E0A);

			/* reg_pwm_dac0_mode */
			val &= ~(0x7);
			val |= (0x1 << 2);

			/* reg_pwm_dac0_oen */
			val &= ~(0x1 << 6);

			regulator_writew(val, 0x101E0A);
#endif
			total = 0;
			for (index = 0; index < 32; index++)
				total += mtktv_get_voltage_bysar();
			detect_vol = total/32;
			if (detect_vol > default_power/1000)
				MDrv_PWM_Shift(num,  (detect_vol - default_power/1000)*1000/scale);
			else
				MDrv_PWM_Shift(num, BIT7 | ((default_power/1000 - detect_vol)*1000/scale));
			printk("[mtktv-regulator] PWM DAC DONE\n");
#endif
			break;
		}
		case E_REGULATOR_GPIO:
			break;
		default:
			pr_err("[mtktv-regulator] not support power type\n");
	}
	regulator_init = 1;
	mutex_unlock(&INIT_MUTEX);
	return ret;
}



static int mtktv_regulator_enable(struct regulator_dev *dev)
{
	return 0;
}

static int mtktv_regulator_disable(struct regulator_dev *dev)
{
	mutex_lock(&INIT_MUTEX);
	regulator_init = 0;
	mutex_unlock(&INIT_MUTEX);
	return 0;
}

static int mtktv_regulator_is_enabled(struct regulator_dev *dev)
{
	u8 bEnable = 1;
	mutex_lock(&INIT_MUTEX);
	bEnable = regulator_init;
	mutex_unlock(&INIT_MUTEX);
	return (bEnable==1)?0:1;
}

static int mtktv_regulator_get_voltage(struct regulator_dev *rdev)
{
	return mtktv_get_voltage_bysar()*1000;
}

static int mtktv_regulator_set_voltage(struct regulator_dev *rdev,int reg_min, int reg_max, unsigned int *selector)
{
	u32  reg_target = 0;

	if (regulator_init == 0) {
		mtktv_pmic_init();
	}

	reg_target = (reg_min + reg_max)/2;
	reg_target = reg_target/1000;
	mtktv_set_voltage(reg_target);

	//pr_info("[%s][%d]min = %d uV, max = %d uV\n",__func__,__LINE__,reg_min,reg_max);
	return 0;
}

static struct regulator_ops mtktv_regulator_voltage_continuous_ops = {
	.get_voltage    = mtktv_regulator_get_voltage,
	.set_voltage    = mtktv_regulator_set_voltage,
	.enable         = mtktv_regulator_enable,
	.disable        = mtktv_regulator_disable,
	.is_enabled     = mtktv_regulator_is_enabled,
};

static struct regulator_desc mtktv_regulator_desc = {
	.name           = "mtktv-regulator",
	.type           = REGULATOR_VOLTAGE,
	.owner          = THIS_MODULE,
	.supply_name    = "mtktv",
};

static int mtktv_regulator_probe(struct platform_device *pdev)
{
	const struct regulator_init_data *init_data;
	struct regulator_dev *regulator;
	struct regulator_config config = { };
	struct device_node *np = pdev->dev.of_node;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "Device Tree node missing\n");
		return -EINVAL;
	}

	init_data = of_get_regulator_init_data(&pdev->dev, np, &mtktv_regulator_desc);
	if (!init_data)
		return -ENOMEM;
	config.of_node = np;
	config.dev = &pdev->dev;
	config.driver_data = NULL;
	config.init_data = init_data;
	mtktv_regulator_desc.ops = &mtktv_regulator_voltage_continuous_ops;
	mtktv_regulator_desc.continuous_voltage_range =  true;
	regulator = devm_regulator_register(&pdev->dev, &mtktv_regulator_desc, &config);
	if (IS_ERR(regulator)) {
		ret = PTR_ERR(regulator);
		dev_err(&pdev->dev, "Failed to register regulator %s: %d\n", mtktv_regulator_desc.name, ret);
		return ret;
	}
	pr_info("[mtktv-regulator][%s][%d] End\n",__func__,__LINE__);
	return 0;
}

static const struct of_device_id mtktv_regulator_of_match[] = {
	{ .compatible = "mtktv-regulator" },
	{ },
};
MODULE_DEVICE_TABLE(of, mtktv_regulator_of_match);

static struct platform_driver mtktv_regulator_driver = {
	.driver = {
		.name			= "mtktv-regulator",
		.owner			= THIS_MODULE,
		.of_match_table	= mtktv_regulator_of_match,
	},
	.probe = mtktv_regulator_probe,
};

static int __init mtktv_regulator_init(void)
{
	platform_driver_register(&mtktv_regulator_driver);
	pr_info("[mtktv-regulator] Register mtktv-regulator driver\n");
	return 0;
}

late_initcall(mtktv_regulator_init);

MODULE_DESCRIPTION("MediaTek Regulator driver");
MODULE_AUTHOR("Chun-Jie.Chen <Chun-Jie.Chen@mediatek.com>");
MODULE_LICENSE("GPL");
