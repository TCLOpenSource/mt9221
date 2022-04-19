#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/thermal.h>
#include <linux/reset.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/io.h>

#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define BASE_ADDRESS                    0xfd000000
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define BASE_ADDRESS                    mstar_pm_base
#endif

#define DEFAULT_T_SENSOR_REF            (400)
#define DEFAULT_T_SENSOR_SHIFT          (27500)
#define DEFAULT_T_SENSOR_RANGE          (15)
#define REG_SAR_VBE                     (0x10050C)
#define VBE_SIZE                        (10)
#define REG_SENSOR_SHIFT                (0x10050E)

static u16 tsensor_ref = 0;
static u16 tsensor_shift = 0;

#ifdef CONFIG_ARM_MTKTV_CPUFREQ
extern u32 temperature_offset;
#endif

#define thermal_writew(data, addr) \
	writew(data, (void __iomem *)(BASE_ADDRESS + (addr << 1)))

#define thermal_readw(addr) \
	readw((void __iomem *)(BASE_ADDRESS + (addr << 1)))

DEFINE_MUTEX(thermal_get_sar_mutex);

static void get_tsensor_data(void)
{
	u16 value = 0;

        value = thermal_readw(REG_SAR_VBE);
        value = value & ((0x1 << VBE_SIZE)-1);

	if (value == 0) {
		tsensor_ref = DEFAULT_T_SENSOR_REF;
		pr_info("trim value is zero, use default value: %d\n", tsensor_ref);
	} else if ((value > DEFAULT_T_SENSOR_REF + DEFAULT_T_SENSOR_RANGE) ||
			(value < DEFAULT_T_SENSOR_REF - DEFAULT_T_SENSOR_RANGE)) {
                tsensor_ref = DEFAULT_T_SENSOR_REF;
		pr_info("trim value is over boundary, use default value: %d\n",tsensor_ref);
		pr_info("trim value: %d\n", tsensor_ref);
	} else {
                tsensor_ref = value;
		pr_info("trim value: %d\n", tsensor_ref);
	}

	tsensor_shift = thermal_readw(REG_SENSOR_SHIFT);
	if (tsensor_shift == 0)
		tsensor_shift = DEFAULT_T_SENSOR_SHIFT;
        pr_info("tsensor shift: %d\n", tsensor_shift);
}

static int get_soc_temperature_bysar(int type)
{
        // 0: CPU, 1: GPU, 2: PM
	u32 temperature = 0;

        mutex_lock(&thermal_get_sar_mutex);

	if ( (tsensor_ref == 0) || (tsensor_shift == 0))
		get_tsensor_data();

	if (type == 0 || type ==1) {
		/* sar7/sar8 ref vol as 2.0v */
		thermal_writew(thermal_readw(0x001432) & ~(0x1 << 6 | 0x1 << 7),0x001432);
		/* reg_en_tsen */
		thermal_writew(thermal_readw(0x000E5E) | (0x1 << 2), 0x000E5E);
		/* reg_gcr_sel_ext_int */
		thermal_writew(thermal_readw(0x000EC8) | (0x1 << 10), 0x000EC8);
		/* select external t-sen from CPU side */
		thermal_writew(thermal_readw(0x001420) | (0x1),0x001420);
		/* set freerun mode */
		thermal_writew(0x0A20, 0x001400);
                msleep(1);
		thermal_writew(0x4A20, 0x001400);
                usleep_range(100,100);
		temperature = (type == 0)?thermal_readw(0x00148E):thermal_readw(0x00148C);
	}
	else if (type == 2) {
		/* sar7/sar8 ref vol as 2.0v */
		thermal_writew(thermal_readw(0x001432) & ~(0x1 << 6 | 0x1 << 7),0x001432);
		/* reg_en_tsen */
		thermal_writew(thermal_readw(0x000E5E) | (0x1 << 2), 0x000E5E);
		/* reg_gcr_sel_ext_int */
		thermal_writew(thermal_readw(0x000EC8) & ~(0x1 << 10), 0x000EC8);
		/* select external t-sen from CPU side */
		thermal_writew(thermal_readw(0x001420) & ~(0x1),0x001420);
		/* set freerun mode */
		thermal_writew(0x0A20, 0x001400);
		msleep(1);
		thermal_writew(0x4A20, 0x001400);
		usleep_range(100,100);
		temperature = thermal_readw(0x00148C);
	}

	if (tsensor_ref > temperature)
		temperature = (tsensor_ref - temperature) * 1250 + tsensor_shift;
	else
		temperature = tsensor_shift - (temperature - tsensor_ref) * 1250;

	temperature /= 1000;
#ifdef CONFIG_ARM_MTKTV_CPUFREQ
	temperature += temperature_offset;
#endif
	mutex_unlock(&thermal_get_sar_mutex);
	return temperature;
}

int get_cpu_temperature_bysar(void)
{
	return get_soc_temperature_bysar(0);
}
EXPORT_SYMBOL(get_cpu_temperature_bysar);

int get_gpu_temperature_bysar(void)
{
	return get_soc_temperature_bysar(1);
}

int get_pm_temperature_bysar(void)
{
	return get_soc_temperature_bysar(2);
}

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static int get_temp(void *thermal, int *temp)
#else
static int get_temp(struct thermal_zone_device *thermal, int *temp)
#endif
{
	*temp = get_cpu_temperature_bysar()*1000;
	return 0;
}

static int notify_callback(struct thermal_zone_device *tz, int trip, enum thermal_trip_type type)
{
	pr_emerg("[mtktv-thermal] trip point: %d\n",trip);
	pr_emerg("[mtktv-thermal] Reach reset temperature point, trigger WDT reset to protect SOC!!!\n");

	thermal_writew(0x00, 0x00300A);
	thermal_writew(0x00, 0x003008);
	thermal_writew(0x05, 0x00300A);
	thermal_writew(0x01, 0x003000);

	return 0;
}


static struct thermal_zone_of_device_ops mtktv_thermal_of_ops = {
	.get_temp = get_temp,
};

static int mtktv_thermal_probe(struct platform_device *pdev)
{
	struct thermal_zone_device *tzdev;
	tzdev = devm_thermal_zone_of_sensor_register(&pdev->dev, 0, NULL,&mtktv_thermal_of_ops);

	if (IS_ERR(tzdev)) {
		dev_warn(&pdev->dev, "Error registering sensor: %ld\n", PTR_ERR(tzdev));
		return PTR_ERR(tzdev);
	}

	tzdev->ops->notify = notify_callback;
	pr_info("[mtktv-thermal][%s][%d] End\n",__func__,__LINE__);
	return 0;
}

static int mtktv_thermal_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id mtktv_thermal_of_match[] = {
	{ .compatible = "cpu-thermal" },
	{},
};
MODULE_DEVICE_TABLE(of, mtktv_thermal_of_match);

static struct platform_driver mtktv_thermal_platdrv = {
	.driver = {
		.name       = "mtk-thermal",
		.owner      = THIS_MODULE,
		.of_match_table = mtktv_thermal_of_match,
	},
	.probe  = mtktv_thermal_probe,
	.remove = mtktv_thermal_remove,
};


static int __init mtktv_thermal_init(void)
{
	platform_driver_register(&mtktv_thermal_platdrv);
	pr_info("[mtktv-thermal] Register mtktv-thermal driver\n");
	return 0;
}

late_initcall(mtktv_thermal_init);

MODULE_DESCRIPTION("MediaTek Thermal driver");
MODULE_AUTHOR("Chun-Jie.Chen <Chun-Jie.Chen@mediatek.com>");
MODULE_LICENSE("GPL");
