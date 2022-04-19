#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/suspend.h>

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
#include <linux/sched/signal.h>
#include <linux/sched/debug.h>
#include <uapi/linux/sched/types.h>
#endif

#include "mtktv-cpufreq.h"

#if CONFIG_MSTAR_PWM
#include "mdrv_pwm.h"
#endif

#if defined(CONFIG_ARM_MTKTV_CPUFREQ_CA)
extern int _tee_get_supplicant_count(void);
static int ta_init;
static struct task_struct *lunch_boost_task;

/* for optee suspend/resume handler */
static struct notifier_block cpufreq_pm_suspend_notifier;
static struct notifier_block cpufreq_pm_resume_notifier;

DECLARE_WAIT_QUEUE_HEAD(cold_boot_wait);
unsigned long cold_boot_finish;
#endif

extern const struct thermal_trip *of_thermal_get_trip_points(
		struct thermal_zone_device *tz);
extern int of_thermal_get_ntrips(struct thermal_zone_device *tz);
extern struct thermal_zone_device *thermal_zone_get_zone_by_name(
		const char *name);

/*********************************************************************
 *                         Private Structure                         *
 *********************************************************************/
struct mtktv_boost_client {
	mtktv_boost_client_id id;
	boost_type type;
	unsigned int time;
	unsigned int rate;
};

struct mtktv_cpu_dvfs_info {
	struct regulator *reg;
	struct cpumask cpus;
	struct device *cpu_dev;
	struct list_head list_head;

	/* for boost used */
	unsigned int default_min;
	unsigned int default_max;
};

enum corner_type {
	E_CORNER_SS   = 0,
	E_CORNER_TT   = 1,
	E_CORNER_FF   = 2,
	E_CORNER_NONE = 3,
};

u32 temperature_offset;
EXPORT_SYMBOL(temperature_offset);
/*********************************************************************
 *                         Private Macro                             *
 *********************************************************************/
#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define BASE_ADDRESS                    (0xfd000000)
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define BASE_ADDRESS                    mstar_pm_base
#endif

#define LUNCH_BOOST_TIME_MS             (30*1000)
#define MAX_BUFFER_SIZE                 (100)
#define MAX_NAME_SIZE                   (100)
#define MAX_APP_NUM                     (10)

#define CPUFREQ_DEBUG(x, args...) \
	{if (debug_enable) \
		pr_emerg(x, ##args); }

#define CPUFREQ_INFO(x, args...)  \
	{if (info_enable) \
		pr_emerg(x, ##args); }

#define CPUFREQ_TRACER(x, args...)  \
	{if (tracer_enable) \
		pr_emerg(x, ##args); }

/* corner */
#define REG_DVFS_CORNER       (0x10050A)
#define CPU_CORNER            (0x3)

/* common */
#define DEVICE_NAME           (100)
#define BUFFER_SIZE           (100)
#define SAMPLE_TIMES          (32)

/* clk @REE DVFS ONLY */
#define REG_CLK_CALC          (0x101DF6)
#define REG_CLK_REPORT        (0x101DF4)
#define REG_CLK_SCU_PLL       (0x101DF8)
#define REG_MIPSPLL_EN        (0x110C2E)

#define REG_LPF_SET_LOW_L     (0x110CA0)
#define REG_LPF_SET_LOW_H     (0x110CA2)
#define REG_LPF_SET_HIGH_L    (0x110CA4)
#define REG_LPF_SET_HIGH_H    (0x110CA6)

#define REG_LPF_ENABLE        (0x110CA8)
#define REG_LPF_MU            (0x110CAA)
#define REG_LPF_FREEZE        (0x110CAC)
#define REG_LPF_UPDATE_CNT    (0x110CAE)
#define REG_LPF_USE_HW        (0x110CB0)
#define REG_DVFS              (0x110CB2)
#define REG_LPF_DONE          (0x110CBA)

/* chip revision */
#define REG_PMTOP_VERSION     (0x1E02)
#define REG_PMTOP_REVISION    (0x1E03)

/*********************************************************************
 *                         Local Function                            *
 *********************************************************************/
/* cpufreq driver CB */
static int mtktv_cpufreq_init(struct cpufreq_policy *policy);
static int mtktv_cpufreq_exit(struct cpufreq_policy *policy);
static void mtktv_cpufreq_ready(struct cpufreq_policy *policy);
static int mtktv_cpufreq_resume(struct cpufreq_policy *policy);
static int mtktv_cpufreq_suspend(struct cpufreq_policy *policy);
static int mtktv_cpufreq_set_target(struct cpufreq_policy *policy,
		unsigned int index);

/* regulator @REE DVFS ONLY*/
static int mtktv_cpufreq_set_voltage(struct mtktv_cpu_dvfs_info *info,
		int vproc);

/* clk @REE DVFS ONLY*/
static unsigned int mtktv_cpufreq_get_clk(unsigned int cpu);
static int mtktv_cpufreq_set_clk(unsigned int u32Current,
		unsigned int u32Target);

/* common */
static void cpufreq_writew(u16 data, unsigned int addr);
static u16 cpufreq_readw(unsigned int addr);
static struct mtktv_cpu_dvfs_info *mtktv_cpu_dvfs_info_lookup(int cpu);

/* boost */
static void mtktv_boost_init(void);
static void mtktv_boost_enable(unsigned int bEnable, unsigned int rate);

/* corner ic @REE DVFS ONLY*/
static enum corner_type cpufreq_get_corner(void);
static int mtktv_cpufreq_opp_init(void);

static void mtktv_boost_enable(unsigned int bEnable, unsigned int rate);

/*********************************************************************
 *                         Private Data                              *
 *********************************************************************/
DEFINE_MUTEX(boost_mutex);
DEFINE_MUTEX(clk_mutex);
DEFINE_MUTEX(target_mutex);

static LIST_HEAD(dvfs_info_list);

static u32 u32Current;
static int thermal_done;

struct delayed_work disable_work;

static struct mtktv_boost_client available_boost_list[E_MTKTV_BOOST_NONE];
static struct mtktv_boost_client current_boost;

static int suspend;
static int regulator_init;
static int deep_suspend = -1;
static int suspend_frequency;

static struct cpufreq_frequency_table *mtktv_freq_table[CONFIG_NR_CPUS];
static struct thermal_cooling_device *cdev;

/* for procfs used */
static struct proc_dir_entry *mtktv_cpufreq_dir;
static atomic_t mtktv_voltage_op_is_open = ATOMIC_INIT(0);
static atomic_t mtktv_frequency_op_is_open = ATOMIC_INIT(0);
static atomic_t mtktv_temperature_op_is_open = ATOMIC_INIT(0);
static atomic_t mtktv_temperature_info_op_is_open = ATOMIC_INIT(0);
static atomic_t mtktv_boost_list_op_is_open = ATOMIC_INIT(0);
static atomic_t mtktv_voltage_core_op_is_open = ATOMIC_INIT(0);

/* for debug used */
static int auto_measurement = 0;
static int debug_enable = 0;
static int info_enable = 0;
static int tracer_enable = 0;
static int slttest_enable = 0;
static int slttest_disable_boost = 0;

/* for boosting */
static char app_info[MAX_APP_NUM][TASK_COMM_LEN];
static int number_of_app;

DEFINE_MUTEX(boosting_boost_mutex);
static struct task_struct *boosting_detect_task;
static struct thermal_zone_device *tzd;

/*********************************************************************
 *                          Common Function                          *
 *********************************************************************/
static struct mtktv_cpu_dvfs_info *mtktv_cpu_dvfs_info_lookup(int cpu)
{
	struct mtktv_cpu_dvfs_info *info;

	list_for_each_entry(info, &dvfs_info_list, list_head) {
		if (cpumask_test_cpu(cpu, &info->cpus))
			return info;
	}
	return NULL;
}

static void cpufreq_writew(u16 data, unsigned int addr)
{
	writew(data, (void __iomem *)(BASE_ADDRESS + (addr << 1)));
}

static u16 cpufreq_readw(unsigned int addr)
{
	return (u16)readw((void __iomem *)(BASE_ADDRESS + (addr << 1)));
}

/*********************************************************************
 *                             CLK Related                           *
 *********************************************************************/
static unsigned int mtktv_cpufreq_get_clk(unsigned int cpu)
{
	u32 u32Clock = 0;

	mutex_lock(&clk_mutex);

	cpufreq_writew(0x0000, REG_CLK_CALC);
	udelay(200);
	cpufreq_writew(0x0001, REG_CLK_CALC);
	udelay(200);

	/* check done */
	do {
		udelay(200);
	} while ((cpufreq_readw(REG_CLK_CALC) & 0x3) != 0x3);

	u32Clock = (u32)cpufreq_readw(REG_CLK_REPORT);
	u32Clock *= 192;
	u32Clock /= 1000;
	u32Clock *= 1000;
	u32Current = u32Clock;

	u32Clock = u32Current;
	mutex_unlock(&clk_mutex);
	return u32Clock; //KHz
}

static int mtktv_cpufreq_set_clk(unsigned int u32Current,
		unsigned int u32Target)
{
	u32 dwRegisterValue = 0;
	u32 iLeftShit = 0;
	u32 high = 0, low = 0;

	CPUFREQ_TRACER("[CPUFREQ_TRACER] start:\n"
			"current clock = %d\n"
			"target clock = %d\n",
			u32Current, u32Target);

	mutex_lock(&clk_mutex);
	/* set CPU Clock Mux to ARMPLL clk */
	cpufreq_writew(0x1, REG_CLK_SCU_PLL);
	dwRegisterValue = cpufreq_readw(REG_MIPSPLL_EN);

	if (u32Target < 600000) {
		/* Range : 180M ~ 599M, Div4 */
		dwRegisterValue |= 0x7;
		iLeftShit = 2;
	} else {
		/* Range : 600M ~ 2.5G, Div1 */
		dwRegisterValue &= ~(0x7);
		dwRegisterValue |= 0x1;
		iLeftShit = 0;
	}

	cpufreq_writew(dwRegisterValue, REG_MIPSPLL_EN);

	if (u32Current > u32Target) {
		high = u32Current/1000;
		low = u32Target/1000;
	} else {
		high = u32Target/1000;
		low = u32Current/1000;
	}

	/*
	 * Set LPF_LOW
	 * Range : 600M ~ 2.5G  :
	 *  Calculate register dac of Current Freq =
	 *  3623875 /low_clkfreq_GHz
	 * Range : 180M ~ 599M :
	 *  Calculate register dac of Current Freq =
	 *  3623875 /(low_clkfreq_GHz*4)
	 */
	dwRegisterValue = ((3623878UL / (low<<iLeftShit)) * 1000);
	cpufreq_writew(dwRegisterValue & 0xFFFF, REG_LPF_SET_LOW_L);
	cpufreq_writew((dwRegisterValue >> 16) & 0xFFFF, REG_LPF_SET_LOW_H);

	/*
	 * Set LPF_HIGH
	 * Range : 600M ~ 2.5G  :
	 *  Calculate register dac of Current Freq =
	 *  3623875 /high_clkfreq_GHz
	 * Range : 180M ~ 599M :
	 *  Calculate register dac of Current Freq =
	 *  3623875 /(high_clkfreq_GHz*4)
	 */
	dwRegisterValue = ((3623878UL / (high<<iLeftShit)) * 1000);
	cpufreq_writew(dwRegisterValue & 0xFFFF, REG_LPF_SET_HIGH_L);
	cpufreq_writew((dwRegisterValue >> 16) & 0xFFFF, REG_LPF_SET_HIGH_H);

	/* Set freq tune step , reg_lpu_mu */
	cpufreq_writew(0x6, REG_LPF_MU);
	/* Set interval between 2 freq , reg_lpu_update_cnt */
	cpufreq_writew(0x8, REG_LPF_UPDATE_CNT);
	/* eg_lpf_use_hw */
	cpufreq_writew(0x1, REG_LPF_USE_HW);
	cpufreq_writew(cpufreq_readw(REG_LPF_USE_HW) | 0x100, REG_LPF_USE_HW);

	if (u32Current < u32Target) {
		/* Start from Low to High ,  reg_lpf_low2high */
		cpufreq_writew(0x1330, REG_DVFS);
	} else {
		/* Start from High to Low ,  reg_lpf_low2high */
		cpufreq_writew(0x0330, REG_DVFS);
	}

	cpufreq_writew(0x1, REG_LPF_ENABLE);
	udelay(200);

	/* check done */
	do {
		udelay(200);
	} while ((cpufreq_readw(REG_LPF_DONE) & 0x1) == 0);

	cpufreq_writew(0x0, REG_LPF_ENABLE);
	udelay(200);
	mutex_unlock(&clk_mutex);

	CPUFREQ_TRACER("\033[1;31m[CPUFREQ_TRACER] end:\n"
			"current clock = %d \033[0m\n",
			mtktv_cpufreq_get_clk(0));
	return 0;
}

/*********************************************************************
 *                         Regulator Related                         *
 *********************************************************************/
static int mtktv_cpufreq_set_voltage(struct mtktv_cpu_dvfs_info *info,
		int vproc)
{
	int ret = 0;

	CPUFREQ_TRACER("[CPUFREQ_TRACER] start:\n"
			"current voltage = %d\n",
			regulator_get_voltage(info->reg));

	if (regulator_init == 0) {
		regulator_init = 1;
		if (deep_suspend == 1) {
			regulator_enable(info->reg);
			deep_suspend = 0;
		}
	}
	ret = regulator_set_voltage(info->reg, vproc, vproc);

	CPUFREQ_TRACER("[CPUFREQ_TRACER] end:\n"
			"current voltage = %d\n",
			regulator_get_voltage(info->reg));

	return ret;
}

/*********************************************************************
 *                          Boost Related (General)                  *
 *********************************************************************/

static void mtktv_boost_init(void)
{
	unsigned int index, opp_cnt;
	struct mtktv_cpu_dvfs_info *info;
	int cpu = 0, value = 0;
	struct device_node *node = NULL;
	char node_name[DEVICE_NAME];

	info = mtktv_cpu_dvfs_info_lookup(cpu);
	if (!info) {
		pr_err("[mtktv-cpufreq] dvfs info for cpu%d is not initialized.\n",
				cpu);
		return;
	}

	opp_cnt = dev_pm_opp_get_opp_count(info->cpu_dev);

	node = of_find_node_by_name(NULL, "opp-table");
	if (node == NULL) {
		pr_err("[mtktv-cpufreq] opp-table not found.\n");
		return;
	};

	for (index = 0; index < E_MTKTV_BOOST_NONE; index++) {
		available_boost_list[index].id = (mtktv_boost_client_id)index;
		available_boost_list[index].time = 0;
		memset(node_name, 0, DEVICE_NAME);
		snprintf(node_name, DEVICE_NAME, "boost-%d", index);
		of_property_read_u32(node, node_name, &value);
		available_boost_list[index].rate = value/1000;
		CPUFREQ_INFO("[mtktv-cpufreq][available_boost_list]\n"
				"index = %d, clk = %d, %s\n",
				index,
				available_boost_list[index].rate,
				node_name);
	}

	current_boost.id = E_MTKTV_BOOST_NONE;
}

static void mtktv_boost_enable(unsigned int bEnable, unsigned int rate)
{
	struct cpufreq_policy *policy = NULL;
	unsigned int cpu = 0;
	struct mtktv_cpu_dvfs_info *info;

	policy = cpufreq_cpu_get_raw(cpu);
	info = mtktv_cpu_dvfs_info_lookup(cpu);
	if (!info) {
		pr_err("[mtktv-cpufreq] dvfs info for cpu%d is not initialized.\n",
				cpu);
		return;
	}

	down_write(&policy->rwsem);

	if (bEnable == 0) {
		policy->user_policy.min = info->default_min;
		policy->user_policy.max = info->default_max;
		current_boost.id = E_MTKTV_BOOST_NONE;
	} else {
		if (rate == 0) {
			policy->user_policy.min =
				available_boost_list[current_boost.id].rate;
			policy->user_policy.max =
				available_boost_list[current_boost.id].rate;
		} else {
			policy->user_policy.min = rate;
			policy->user_policy.max = rate;
		}
	}
	CPUFREQ_DEBUG("[mtktv-cpufreq] min = %d, max = %d\n",
			policy->user_policy.min, policy->user_policy.max);

	up_write(&policy->rwsem);
	cpufreq_update_policy(cpu);
}

static void mtktv_disable_boost(struct work_struct *work)
{
	unsigned int cpu = 0;
	struct mtktv_cpu_dvfs_info *info;

	mutex_lock(&boost_mutex);
	if (suspend != 1)
		mtktv_boost_enable(0, 0);
	if (slttest_enable == 1)
	{
		slttest_disable_boost = 1;
		info = mtktv_cpu_dvfs_info_lookup(cpu);
		if (!info) {
			pr_err("[mtktv-cpufreq] dvfs info for cpu%d is not initialized.\n",
					cpu);
			return;
		}
		mtktv_boost_enable(1, info->default_min);
	}
	mutex_unlock(&boost_mutex);
}

#if defined(CONFIG_ARM_MTKTV_CPUFREQ_CA)
static int lunch_boost_thread(void *arg)
{
	wait_event_interruptible(cold_boot_wait, cold_boot_finish);
	mtktv_boost_register(E_MTKTV_LUNCH_BOOST, 0,
		E_BOOST_ENABLE_WITH_TIME, LUNCH_BOOST_TIME_MS);
	return 0;
}
#endif

boost_result mtktv_boost_register(mtktv_boost_client_id id,
		int freq,
		boost_type type,
		int time)
{
	boost_result ret = E_BOOST_RESULT_OK;

	if (id >= E_MTKTV_BOOST_NONE) {
		pr_err("[mtktv-cpufreq] boost id(%d) is invalid\n", id);
		return E_BOOST_RESULT_INVALID_ID;
	}

	mutex_lock(&boost_mutex);

	if (suspend == 1 || slttest_disable_boost == 1) {
		mutex_unlock(&boost_mutex);
		return ret;
	}

	if (current_boost.id == E_MTKTV_BOOST_NONE) {
		if (type != E_BOOST_DISABLE) {
			current_boost.id = id;

			/*
			 * only allow the debug boost client to
			 * adjust user defiend frequency
			 */
			if ((id == E_MTKTV_DEBUG_BOOST) && (freq != 0)) {
				mtktv_boost_enable(1, freq);
			} else {
				mtktv_boost_enable(1,
					available_boost_list[id].rate);
			}

			/*
			 * request a delay kwork to
			 * disable boosting with user defined time
			 */
			if ((type == E_BOOST_ENABLE_WITH_TIME)
					&& (time != 0)) {
				current_boost.time = time;
				schedule_delayed_work(&disable_work,
						msecs_to_jiffies(time));
			}
		}
	} else {
		/*
		 *if request client has lower priority compare to
		 * current boost client, then skip the request
		 */
		if (current_boost.id < id) {
			ret = E_BOOST_RESULT_IGNORE;
		} else if (current_boost.id == id) {
			if (type == E_BOOST_DISABLE) {
				current_boost.id = E_MTKTV_BOOST_NONE;
				mtktv_boost_enable(0, 0);
			} else if ((type == E_BOOST_ENABLE_WITH_TIME)
						&& (time != 0)) {
				/*
				 * if current boost client has
				 * new time interval to boost,
				 * then delete previous kwork and request a new
				 */
				cancel_delayed_work(&disable_work);
				current_boost.id = id;
				current_boost.time = time;
				mtktv_boost_enable(1,
						available_boost_list[id].rate);
				schedule_delayed_work(&disable_work,
						msecs_to_jiffies(time));
			}
		} else {
			/*
			 * request boost client has higher priority
			 * than current boost client
			 */
			cancel_delayed_work(&disable_work);
			current_boost.id = id;
			mtktv_boost_enable(1,
					available_boost_list[id].rate);

			if ((type == E_BOOST_ENABLE_WITH_TIME)
					&& (time != 0)) {
				current_boost.time = time;
				schedule_delayed_work(&disable_work,
						msecs_to_jiffies(time));
			}
		}
	}

	mutex_unlock(&boost_mutex);
	return ret;
}
EXPORT_SYMBOL(mtktv_boost_register);

/*********************************************************************
 *                          Bench Boost Related                      *
 *********************************************************************/

static int check_app_exist(const char *name)
{
	int find_index = -1, i = 0;
	unsigned int len = 0;
	char app_name[TASK_COMM_LEN];

	len = strlen(name);

	if (len > (TASK_COMM_LEN-1)) {
		memcpy(app_name, name + (len - TASK_COMM_LEN) + 1,
				TASK_COMM_LEN-1);
		app_name[TASK_COMM_LEN-1] = '\0';
	} else {
		memcpy(app_name, name, len);
		app_name[len] = '\0';
	}

	for (i = 0 ; i < number_of_app; i++) {
		if (strncmp(app_name, app_info[i], len) == 0) {
			find_index = i;
			break;
		}
	}
	return find_index;
}

static void show_boost_application(void)
{
	u8 i = 0;

	for (i = 0; i < number_of_app; i++)
		pr_info("Application: %s\n", app_info[i]);
}

static void add_boost_application(const char *name)
{
	unsigned int len = 0;
	int find_index = -1;

	len = strlen(name);

	find_index = check_app_exist(name);

	mutex_lock(&boosting_boost_mutex);

	if (-1 == find_index) {
		if (len > (TASK_COMM_LEN - 1)) {
			memcpy(app_info[number_of_app],
					name + (len - TASK_COMM_LEN) + 1,
					TASK_COMM_LEN-1);
			app_info[number_of_app][TASK_COMM_LEN-1] = '\0';
		} else {
			memcpy(app_info[number_of_app], name, len);
			app_info[number_of_app][len] = '\0';
		}
		number_of_app++;
	}

	mutex_unlock(&boosting_boost_mutex);
}

static void __attribute__((unused)) del_boost_application(const char *name)
{
	u8 i = 0;
	int find_index = check_app_exist(name);

	mutex_lock(&boosting_boost_mutex);

	if (-1 != find_index) {
		for (i = find_index; i < number_of_app-1; i++)
			memcpy(app_info[i], app_info[i+1], TASK_COMM_LEN);

		number_of_app--;
	}

	mutex_unlock(&boosting_boost_mutex);
}

static int boosting_detect_thread(void *arg)
{
	struct task_struct *process;
	int find_index = -1, app_boosting = 0;

	while (1) {
		/*
		 * the task will be parked when driver suspend
		 * and unparked when driver resume
		 */
		kthread_parkme();

		mutex_lock(&boosting_boost_mutex);
		read_lock(&tasklist_lock);

		for_each_process(process) {
			find_index = check_app_exist(process->comm);
			/*
			 * when process is forground,
			 * oom_socre_adj will be adjust to zero
			 */
			if ((-1 != find_index) &&
				(process->signal->oom_score_adj == 0)) {
				if (app_boosting == 0) {
					app_boosting = 1;
					mtktv_boost_register(
						E_MTKTV_BENCH_BOOST,
						0, E_BOOST_ENABLE, 0);
					pr_info("%s application[pid=%d] detect, start boosting\n",
							process->comm,
							process->pid);
				}
				break;
			}
			find_index = -1;
		}

		if ((find_index == -1) && (app_boosting == 1)) {
			app_boosting = 0;
			mtktv_boost_register(E_MTKTV_BENCH_BOOST,
					0, E_BOOST_DISABLE, 0);
			pr_info("no registered application detect, stop boosting\n");
		}
		read_unlock(&tasklist_lock);
		mutex_unlock(&boosting_boost_mutex);
		msleep(1000);
	}
	return 0;
}

/*********************************************************************
 *                         CORNER IC Related                         *
 *********************************************************************/
static enum corner_type cpufreq_get_corner(void)
{
	u16 value = 0;

	value = cpufreq_readw(REG_DVFS_CORNER);
	value = value & CPU_CORNER;
	if (value == 0)
		return E_CORNER_SS;
	else
		return E_CORNER_FF;
}

static int mtktv_cpufreq_opp_init(void)
{
	struct device_node *node = NULL, *child = NULL;
	struct mtktv_cpu_dvfs_info *info;
	struct dev_pm_opp *opp;
	unsigned long freq_hz = 0;
	int vproc = 0, old_vproc = 0, opp_cnt = 0, value = 0, i = 0;
	struct cpufreq_frequency_table *freq_table;
	char node_name[MAX_NAME_SIZE];

	node = of_find_node_by_name(NULL, "opp-table");
	if (node == NULL)
		return -EINVAL;

	info = mtktv_cpu_dvfs_info_lookup(0);
	if (!info)
		return -EINVAL;

	if (cpufreq_get_corner() == E_CORNER_FF) {
		opp_cnt = dev_pm_opp_get_opp_count(info->cpu_dev);

		dev_pm_opp_init_cpufreq_table(info->cpu_dev, &freq_table);
		for (i = 0; i < opp_cnt; i++) {
			freq_hz = freq_table[i].frequency;
			freq_hz *= 1000;

			dev_pm_opp_remove(info->cpu_dev, freq_hz);
			memset(node_name, 0, MAX_NAME_SIZE);
			snprintf(node_name, MAX_NAME_SIZE, "opp-%ld",
							freq_hz);
			child = of_find_node_by_name(node, node_name);
			of_property_read_u32(child,
					"opp-microvolt", &old_vproc);
			of_property_read_u32(child,
					"opp-corner-offset", &value);

			dev_pm_opp_add(info->cpu_dev, freq_hz,
						old_vproc-value);
		}
		kfree(freq_table);
	}

	if (info_enable) {
		opp_cnt = dev_pm_opp_get_opp_count(info->cpu_dev);

		dev_pm_opp_init_cpufreq_table(info->cpu_dev, &freq_table);
		CPUFREQ_INFO("[mtktv-cpufreq] CORNER TYPE:%d\n",
				cpufreq_get_corner());

		for (i = 0; i < opp_cnt; i++) {
			freq_hz = freq_table[i].frequency;
			freq_hz *= 1000;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
			opp = dev_pm_opp_find_freq_ceil(info->cpu_dev, &freq_hz);
			rcu_read_lock();
#else
			rcu_read_lock();
			opp = dev_pm_opp_find_freq_ceil(info->cpu_dev, &freq_hz);
#endif
			vproc = dev_pm_opp_get_voltage(opp);
			rcu_read_unlock();
			CPUFREQ_INFO("[mtktv-cpufreq] clock = %ld, voltage = %d\n",
					freq_hz, vproc);
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
			dev_pm_opp_put(opp);
#endif
		}
		kfree(freq_table);
	}
	return 0;
}

/*********************************************************************
 *                          PROCFS DEBUG                             *
 *********************************************************************/
/* voltage operation */
ssize_t mtktv_voltage_op_write(struct file *file,
		const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int set_voltage = 0, set_cpu = 0, temp = 0;
	char buffer[MAX_BUFFER_SIZE];
	struct mtktv_cpu_dvfs_info *info;

	if (!count)
		return count;

	if (count >= MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE-1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	if (sscanf(buffer, "%d %d %d", &set_cpu, &set_voltage, &temp) == 3) {
		return -EFAULT;
	} else if (sscanf(buffer, "%d %d", &set_cpu, &set_voltage) == 2) {
		if (set_cpu >= CONFIG_NR_CPUS)
			return -EINVAL;

		/* set_voltage: XXX0mV */
		info = mtktv_cpu_dvfs_info_lookup(set_cpu);
		mtktv_cpufreq_set_voltage(info, set_voltage*1000);
		return count;
	}

	return -EFAULT;
}

static int mtktv_voltage_op_show(struct seq_file *s, void *v)
{
	unsigned int  vol = 0;
	unsigned int  i = 0, j = 0, total = 0;
	struct mtktv_cpu_dvfs_info *info;

	for_each_online_cpu(i) {
		info = mtktv_cpu_dvfs_info_lookup(i);
		total = 0;

		for (j = 0; j < SAMPLE_TIMES; j++)
			total += regulator_get_voltage(info->reg);

		vol = total/SAMPLE_TIMES;
		seq_printf(s, "CPU_%d_vol:%d\n", i, vol/1000); // mV
	}
	return 0;
}

static int mtktv_voltage_op_open(struct inode *inode, struct file *file)
{
	if (atomic_read(&mtktv_voltage_op_is_open))
		return -EACCES;

	atomic_set(&mtktv_voltage_op_is_open, 1);
	return single_open(file, &mtktv_voltage_op_show, NULL);
}

static int mtktv_voltage_op_release(struct inode *inode, struct file *file)
{
	WARN_ON(!atomic_read(&mtktv_voltage_op_is_open));
	atomic_set(&mtktv_voltage_op_is_open, 0);
	return single_release(inode, file);
}

/* end of voltage operation */

/* core power operation */

static int core_init = 0;

static int get_core_power_bysar(void)
{
	u32 dwTempData = 0;

	cpufreq_writew(0x0A20, 0x001400);
	cpufreq_writew(cpufreq_readw(0x00140A) & ~(0x3), 0x00140A);
	cpufreq_writew(cpufreq_readw(0x001432) & ~(0x1 << 9), 0x001432);
	udelay(300);
	cpufreq_writew(cpufreq_readw(0x001400) | (0x1 << 14), 0x001400);
	while( (cpufreq_readw(0x001400) & (0x1 << 14)) != 0 );
	dwTempData = cpufreq_readw(0x001492);
	dwTempData = dwTempData * 2000/1024;

	return dwTempData;
}


static void set_core_power(int power)
{
	u16 val = 0;
	u32 index = 0,total = 0;
	u32 default_power = 1000, detect_vol;
	u32 scale = 1950;
#if defined(CONFIG_MSTAR_MT5867)
	u32 adjust_duty = 0x65;
#else
	u32 adjust_duty = 0x69;
#endif

	if (core_init == 0) {
		cpufreq_writew(cpufreq_readw(0x1522E0) | (0x1 << 2), 0x1522E0);
		MDrv_PWM_Period(E_PVR_PWM_CH1, 0x1AF);
		MDrv_PWM_DutyCycle(E_PVR_PWM_CH1, adjust_duty);
		MDrv_PWM_Shift(E_PVR_PWM_CH1, 0);
		MDrv_PWM_AutoCorrect(E_PVR_PWM_CH1, 1);

		val = cpufreq_readw(0x101E0A);

		/* reg_pwm_dac1_mode */
		val &= ~(0x3 << 3);
		val |= (0x1 << 4);

		/* reg_pwm_dac1_oen */
		val &= ~(0x1 << 7);

		cpufreq_writew(val, 0x101E0A);

		for (index = 0; index < 32; index++)
			total += get_core_power_bysar();

		detect_vol = total/32;
		if (detect_vol > default_power)
			MDrv_PWM_Shift(E_PVR_PWM_CH1,  (detect_vol - default_power)*1000/scale);
		else
			MDrv_PWM_Shift(E_PVR_PWM_CH1, BIT7 | ((default_power - detect_vol)*1000/scale));
		core_init = 1;
	}

	if (power > default_power) {
		adjust_duty -= (power - default_power)*1000/scale;
	} else {
		adjust_duty += (default_power - power)*1000/scale;
	}

	MDrv_PWM_DutyCycle(E_PVR_PWM_CH1, adjust_duty);
	msleep(1);
}


ssize_t mtktv_voltage_core_op_write(struct file *file,
                const char __user *buf,
                size_t count, loff_t *ppos)
{
	unsigned int set_voltage = 0, set_cpu = 0, temp = 0;
	char buffer[MAX_BUFFER_SIZE];

	if (!count)
		return count;

	if (count >= MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE-1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	if (sscanf(buffer, "%d %d %d", &set_cpu, &set_voltage, &temp) == 3) {
		return -EFAULT;
	} else if (sscanf(buffer, "%d %d", &set_cpu, &set_voltage) == 2) {
		if (set_cpu >= CONFIG_NR_CPUS)
			return -EINVAL;
		set_core_power(set_voltage);
		return count;
	}

	return -EFAULT;
}

static int mtktv_voltage_core_op_show(struct seq_file *s, void *v)
{
	unsigned int  vol = 0;
	unsigned int  i = 0, j = 0, total = 0;

	for_each_online_cpu(i) {
		total = 0;

		for (j = 0; j < SAMPLE_TIMES; j++)
			total += get_core_power_bysar();

		vol = total/SAMPLE_TIMES;
		seq_printf(s, "GPU_%d_vol:%d\n", i, vol); // mV
	}
	return 0;
}

static int mtktv_voltage_core_op_open(struct inode *inode, struct file *file)
{
	if (atomic_read(&mtktv_voltage_core_op_is_open))
		return -EACCES;

	atomic_set(&mtktv_voltage_core_op_is_open, 1);
	return single_open(file, &mtktv_voltage_core_op_show, NULL);
}

static int mtktv_voltage_core_op_release(struct inode *inode, struct file *file)
{
	WARN_ON(!atomic_read(&mtktv_voltage_core_op_is_open));
	atomic_set(&mtktv_voltage_core_op_is_open, 0);
	return single_release(inode, file);
}

/* frequency operation */
ssize_t mtktv_frequency_op_write(struct file *file,
		const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[MAX_BUFFER_SIZE];
	unsigned int set_cpu = 0, set_frequency = 0, temp = 0;

	if (!count)
		return count;

	if (count >= MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE-1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	if (sscanf(buffer, "%d %d %d", &set_cpu, &set_frequency, &temp) == 3) {
		return -EFAULT;
	} else if (sscanf(buffer, "%d %d", &set_cpu, &set_frequency) == 2) {
		if (set_cpu >= CONFIG_NR_CPUS)
			return -EINVAL;

		/* set_frequency: XXXX MHz */
		mtktv_cpufreq_set_clk(mtktv_cpufreq_get_clk(set_cpu),
				set_frequency * 1000);

		return count;
	}
	return -EFAULT;
}

static int mtktv_frequency_op_show(struct seq_file *s, void *v)
{
	u32 clock, i;

	for_each_online_cpu(i) {
		clock = mtktv_cpufreq_get_clk(i);
		seq_printf(s, "CPU_%d_freq:%u\n", i, clock/1000);
	}
	return 0;
}

static int mtktv_frequency_op_open(struct inode *inode, struct file *file)
{
	if (atomic_read(&mtktv_frequency_op_is_open))
		return -EACCES;
	atomic_set(&mtktv_frequency_op_is_open, 1);
	return single_open(file, &mtktv_frequency_op_show, NULL);
}

static int mtktv_frequency_op_release(struct inode *inode, struct file *file)
{
	WARN_ON(!atomic_read(&mtktv_frequency_op_is_open));
	atomic_set(&mtktv_frequency_op_is_open, 0);
	return single_release(inode, file);
}

/* end of frequency operation */

/* temperature operation */
ssize_t mtktv_temperature_op_write(struct file *file,
		const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int set_temperature_offset = 0;
	char buffer[MAX_BUFFER_SIZE];
	int res = 0;

	if (!count)
		return count;

	if (count >= MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE-1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	res = kstrtoint(buffer, 10, &set_temperature_offset);
	if (res == 0)
		temperature_offset = set_temperature_offset;

	return count;
}

extern int get_cpu_temperature_bysar(void);
static int mtktv_temperature_op_show(struct seq_file *s, void *v)
{
	s32 dwTempData = 0;
	u32 i = 0;

	for_each_online_cpu(i) {
		dwTempData = get_cpu_temperature_bysar();
		if (temperature_offset == 0)
			seq_printf(s, "CPU_%d_temp:%d\n", i, dwTempData);
		else
			seq_printf(s, "CPU_%d_temp:%d (offset: %d)\n", i,
					dwTempData, temperature_offset);
	}
	return 0;
}

static int mtktv_temperature_op_open(struct inode *inode, struct file *file)
{
	if (atomic_read(&mtktv_temperature_op_is_open))
		return -EACCES;
	atomic_set(&mtktv_temperature_op_is_open, 1);
	return single_open(file, &mtktv_temperature_op_show, NULL);
}

static int mtktv_temperature_op_release(struct inode *inode,
		struct file *file)
{
	WARN_ON(!atomic_read(&mtktv_temperature_op_is_open));
	atomic_set(&mtktv_temperature_op_is_open, 0);
	return single_release(inode, file);
}

extern int get_gpu_temperature_bysar(void);
extern int get_pm_temperature_bysar(void);
static int mtktv_temperature_info_op_show(struct seq_file *s, void *v)
{
	s32 dwTempData = 0;

	dwTempData = get_gpu_temperature_bysar();
	if (temperature_offset == 0)
		seq_printf(s, "GPU_temp:%d\n", dwTempData);
	else
		seq_printf(s, "GPU_temp:%d (offset: %d)\n",
				dwTempData, temperature_offset);

	dwTempData = get_pm_temperature_bysar();
	if (temperature_offset == 0)
		seq_printf(s, "PM_temp:%d\n", dwTempData);
	else
		seq_printf(s, "PM_temp:%d (offset: %d)\n",
				dwTempData, temperature_offset);
	return 0;
}

static int mtktv_temperature_info_op_open(struct inode *inode, struct file *file)
{
	if (atomic_read(&mtktv_temperature_info_op_is_open))
		return -EACCES;
	atomic_set(&mtktv_temperature_info_op_is_open, 1);
	return single_open(file, &mtktv_temperature_info_op_show, NULL);
}

static int mtktv_temperature_info_op_release(struct inode *inode,
		struct file *file)
{
	WARN_ON(!atomic_read(&mtktv_temperature_info_op_is_open));
	atomic_set(&mtktv_temperature_info_op_is_open, 0);
	return single_release(inode, file);
}
/* end of temperature operation */

/* boost_list operation */
ssize_t mtktv_boost_list_op_write(struct file *file,
		const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[MAX_BUFFER_SIZE];

	if (!count)
		return count;

	if (count >= MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE-1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count-1] = '\0';
	add_boost_application(buffer);

	return count;
}

static int mtktv_boost_list_op_show(struct seq_file *s, void *v)
{
	show_boost_application();
	return 0;
}

static int mtktv_boost_list_op_open(struct inode *inode, struct file *file)
{
	if (atomic_read(&mtktv_boost_list_op_is_open))
		return -EACCES;
	atomic_set(&mtktv_boost_list_op_is_open, 1);
	return single_open(file, &mtktv_boost_list_op_show, NULL);
}

static int mtktv_boost_list_op_release(struct inode *inode, struct file *file)
{
	WARN_ON(!atomic_read(&mtktv_boost_list_op_is_open));
	atomic_set(&mtktv_boost_list_op_is_open, 0);
	return single_release(inode, file);
}
/* end of boost_list operation */

static const struct file_operations mtktv_voltage_fileops = {
	.owner      = THIS_MODULE,
	.open       = mtktv_voltage_op_open,
	.write      = mtktv_voltage_op_write,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = mtktv_voltage_op_release,
};

static const struct file_operations mtktv_voltage_core_fileops = {
	.owner      = THIS_MODULE,
	.open       = mtktv_voltage_core_op_open,
	.write      = mtktv_voltage_core_op_write,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = mtktv_voltage_core_op_release,
};

static const struct file_operations mtktv_frequency_fileops = {
	.owner      = THIS_MODULE,
	.open       = mtktv_frequency_op_open,
	.write      = mtktv_frequency_op_write,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = mtktv_frequency_op_release,
};

static const struct file_operations mtktv_temperature_fileops = {
	.owner      = THIS_MODULE,
	.open       = mtktv_temperature_op_open,
	.write      = mtktv_temperature_op_write,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = mtktv_temperature_op_release,
};

static const struct file_operations mtktv_temperature_info_fileops = {
	.owner      = THIS_MODULE,
	.open       = mtktv_temperature_info_op_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = mtktv_temperature_info_op_release,
};

static const struct file_operations mtktv_boost_list_fileops = {
	.owner     = THIS_MODULE,
	.open      = mtktv_boost_list_op_open,
	.write     = mtktv_boost_list_op_write,
	.read      = seq_read,
	.llseek    = seq_lseek,
	.release   = mtktv_boost_list_op_release,
};

static int mtvtv_cpufreq_procfs_init(void)
{
	struct proc_dir_entry *entry;

	mtktv_cpufreq_dir = proc_mkdir("mstar_dvfs", NULL);

	if (!mtktv_cpufreq_dir) {
		pr_err("[mtktv-cpufreq][%d] procfs initialization fail\n",
			__LINE__);
		return -ENOMEM;
	}

	entry = proc_create("voltage", 0600, mtktv_cpufreq_dir,
					&mtktv_voltage_fileops);
	if (!mtktv_cpufreq_dir) {
		pr_err("[mtktv-cpufreq][%d] procfs initialization fail\n",
			__LINE__);
		return -ENOMEM;
	}

	entry = proc_create("core_voltage", 0600, mtktv_cpufreq_dir,
					&mtktv_voltage_core_fileops);
	if (!mtktv_cpufreq_dir) {
		pr_err("[mtktv-cpufreq][%d] procfs initialization fail\n",
			__LINE__);
		return -ENOMEM;
	}

	entry = proc_create("frequency", 0600, mtktv_cpufreq_dir,
					&mtktv_frequency_fileops);
	if (!mtktv_cpufreq_dir) {
		pr_err("[mtktv-cpufreq][%d] procfs initialization fail\n",
			__LINE__);
		return -ENOMEM;
	}

	entry = proc_create("temperature", 0600, mtktv_cpufreq_dir,
					&mtktv_temperature_fileops);
	if (!mtktv_cpufreq_dir) {
		pr_err("[mtktv-cpufreq][%d] procfs initialization fail\n",
			__LINE__);
		return -ENOMEM;
	}

	entry = proc_create("temperature_info", 0600, mtktv_cpufreq_dir,
					&mtktv_temperature_info_fileops);
	if (!mtktv_cpufreq_dir) {
		pr_err("[mtktv-cpufreq][%d] procfs initialization fail\n",
			__LINE__);
		return -ENOMEM;
	}

	entry = proc_create("boost_list", 0600, mtktv_cpufreq_dir,
			&mtktv_boost_list_fileops);
	if (!mtktv_cpufreq_dir) {
		pr_err("[mtktv-cpufreq][%d] procfs initialization fail\n",
			__LINE__);
		return -ENOMEM;
	}

	return 0;
}


/*********************************************************************
 *                         CPUFREQ Driver CB                         *
 *********************************************************************/

static int mtktv_cpufreq_init(struct cpufreq_policy *policy)
{
	struct mtktv_cpu_dvfs_info *info;
	struct cpufreq_frequency_table *freq_table;
	int ret, i;
	unsigned int cpu = policy->cpu;
	unsigned int index = 0;
	unsigned int opp_cnt = 0;

	info = mtktv_cpu_dvfs_info_lookup(cpu);
	if (!info) {
		pr_err("[mtktv-cpufreq] dvfs info for cpu%d is not initialized.\n",
				policy->cpu);
		return -EINVAL;
	}

	opp_cnt = dev_pm_opp_get_opp_count(info->cpu_dev);

	if (mtktv_freq_table[cpu] == NULL) {
		ret = dev_pm_opp_init_cpufreq_table(info->cpu_dev, &freq_table);
		if (ret) {
			pr_err("[mtktv-cpufreq] failed to init cpufreq table\n"
				"for cpu%d: %d\n", policy->cpu, ret);
			return ret;
		}
		mtktv_freq_table[cpu] = kcalloc((opp_cnt + 1),
			sizeof(struct cpufreq_frequency_table), GFP_KERNEL);
		for (index = 0; index < opp_cnt; index++) {
			mtktv_freq_table[cpu][index].frequency =
				freq_table[index].frequency;
			CPUFREQ_INFO("[mtktv-cpufreq] [CPU%d]\n"
				"index = %d, frequency = %d\n",
				cpu, index,
				mtktv_freq_table[cpu][index].frequency);
		}
		mtktv_freq_table[cpu][opp_cnt].frequency = CPUFREQ_TABLE_END;
		kfree(freq_table);
	}

	suspend_frequency = mtktv_freq_table[cpu][1].frequency;
	info->default_min = policy->min = mtktv_freq_table[cpu][1].frequency;
	info->default_max = policy->max =
		available_boost_list[E_MTKTV_DEBUG_BOOST].rate;
	policy->cpuinfo.min_freq = mtktv_freq_table[cpu][0].frequency;
	policy->cpuinfo.max_freq = mtktv_freq_table[cpu][opp_cnt-1].frequency;

	for_each_online_cpu(i)
		if (i < CONFIG_NR_CPUS)
			cpumask_set_cpu(i, policy->cpus);

	policy->freq_table = mtktv_freq_table[cpu];
	policy->cur = mtktv_cpufreq_get_clk(cpu);

	CPUFREQ_INFO("[mtktv-cpufreq] [CPU%d]\n"
			"scaling_max: %d, scaling_min: %d\n"
			"cpu_max: %d, cpu_min: %d\n",
			cpu, policy->max, policy->min,
			policy->cpuinfo.max_freq, policy->cpuinfo.min_freq);

	return 0;
}

static int mtktv_cpufreq_exit(struct cpufreq_policy *policy)
{
	return 0;
}

static void mtktv_cpufreq_ready(struct cpufreq_policy *policy)
{
	struct device_node *np;
	char node[BUFFER_SIZE];

	snprintf(node, BUFFER_SIZE, "cluster%d", 0);
	np = of_find_node_by_name(NULL, node);
	if (!np)
		pr_err("[mtktv-cpufreq] node not found\n");

	if (thermal_done == 0) {
		CPUFREQ_INFO("[mtktv-cpufreq] register cooling device\n");

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
		cdev = of_cpufreq_cooling_register(policy);
#else
		cdev = of_cpufreq_power_cooling_register(np,
			(const struct cpumask *)(&policy->cpus), 0, NULL);
#endif
		if (IS_ERR(cdev))
			pr_err("[mtktv-cpufreq] fail to register cooling device\n");
		else
			CPUFREQ_INFO("[mtktv-cpufreq]\n"
				"successful to register cooling device\n");
		thermal_done = 1;

		tzd = thermal_zone_get_zone_by_name("mtk-thermal");
		if (IS_ERR(tzd))
			pr_err("[mtktv-cpufreq] Thermal Zone(mtk-thermal) Get Fail\n");


	}

#if !defined(CONFIG_ARM_MTKTV_CPUFREQ_CA)
	/* trigger boost event for ac boosting performance */
	mtktv_boost_register(E_MTKTV_LUNCH_BOOST, 0,
		E_BOOST_ENABLE_WITH_TIME, LUNCH_BOOST_TIME_MS);
#endif

	boosting_detect_task = kthread_create(boosting_detect_thread,
		NULL, "cpufreq_boosting_detect");
	wake_up_process(boosting_detect_task);
}

#if !defined(CONFIG_ARM_MTKTV_CPUFREQ_CA)
static int resume_thread(void *arg)
{
	/* trigger boost event for resume booting performance */
	mtktv_boost_register(E_MTKTV_LUNCH_BOOST, 0,
		E_BOOST_ENABLE_WITH_TIME, LUNCH_BOOST_TIME_MS);
	return 0;
}
#endif

static int mtktv_cpufreq_resume(struct cpufreq_policy *policy)
{
	struct mtktv_cpu_dvfs_info *info;
#if !defined(CONFIG_ARM_MTKTV_CPUFREQ_CA)
	static struct task_struct *resume_task;
#endif

	info = mtktv_cpu_dvfs_info_lookup(policy->cpu);

#if !defined(CONFIG_ARM_MTKTV_CPUFREQ_CA)
	mutex_lock(&boost_mutex);
	suspend = 0;
	mutex_unlock(&boost_mutex);

	resume_task = kthread_create(resume_thread,
		NULL, "cpufreq_resume_thread");
	wake_up_process(resume_task);
#endif
	/* unpark boosting detect thead */
	kthread_unpark(boosting_detect_task);

	return 0;
}

static int mtktv_cpufreq_set_target(struct cpufreq_policy *policy,
		unsigned int index)
{
	struct cpufreq_frequency_table *freq_table = policy->freq_table;
	struct mtktv_cpu_dvfs_info *info;
	struct device *cpu_dev;
	struct dev_pm_opp *opp;
	long freq_hz, old_freq_hz;
	int vproc, old_vproc, ret;
	unsigned int cpu = policy->cpu;

	if (auto_measurement)
		return 0;

	mutex_lock(&target_mutex);
	info = mtktv_cpu_dvfs_info_lookup(policy->cpu);
	if (!info) {
		pr_err("[mtktv-cpufreq] dvfs info for cpu%d is not initialized.\n",
				policy->cpu);
		mutex_unlock(&target_mutex);
		return -EINVAL;
	}

	cpu_dev = info->cpu_dev;

	old_freq_hz = mtktv_cpufreq_get_clk(cpu) * 1000;
	freq_hz = freq_table[index].frequency * 1000;

#if defined(CONFIG_ARM_MTKTV_CPUFREQ_CA)
	if (suspend == 1) {
		mutex_unlock(&target_mutex);
		return 0;
	}

	if (_tee_get_supplicant_count() && !ta_init) {
		if (MTKTV_CPUFREQ_TEEC_TA_Open() != TA_SUCCESS) {
			pr_err("[mtktv-cpufreq] CPUFREQ TA Open fail\n");
			mutex_unlock(&target_mutex);
			return 0;
		}

		if (MTKTV_CPUFREQ_TEEC_Init() != TA_SUCCESS) {
			pr_err("[mtktv-cpufreq] CPUFREQ Init fail\n");
			mutex_unlock(&target_mutex);
			return 0;
		}
		ta_init = 1;
		cold_boot_finish = 1;
		wake_up_interruptible(&cold_boot_wait);
	}

	if (!ta_init) {
		mutex_unlock(&target_mutex);
		return 0;
	}
#endif

	/* get current voltage */
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_hz);
	if (IS_ERR(opp)) {
		pr_err("[mtktv-cpufreq] cpu%d: failed to find OPP for %ld\n",
				cpu, freq_hz);
		mutex_unlock(&target_mutex);
		return PTR_ERR(opp);
	}
	rcu_read_lock();
#else
	rcu_read_lock();
	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_hz);
	if (IS_ERR(opp)) {
		pr_err("[mtktv-cpufreq] cpu%d: failed to find OPP for %ld\n",
				cpu, freq_hz);
		rcu_read_unlock();
		mutex_unlock(&target_mutex);
		return PTR_ERR(opp);
	}
#endif
	vproc = dev_pm_opp_get_voltage(opp);
	rcu_read_unlock();
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	dev_pm_opp_put(opp);
#endif
	/* get target voltage in target frequency */
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &old_freq_hz);
	if (IS_ERR(opp)) {
		pr_err("[mtktv-cpufreq] cpu%d: failed to find OPP for %ld\n",
				cpu, old_freq_hz);
		mutex_unlock(&target_mutex);
		return PTR_ERR(opp);
	}
	rcu_read_lock();
#else
	rcu_read_lock();
	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &old_freq_hz);
	if (IS_ERR(opp)) {
		pr_err("[mtktv-cpufreq] cpu%d: failed to find OPP for %ld\n",
				cpu, old_freq_hz);
		rcu_read_unlock();
		mutex_unlock(&target_mutex);
		return PTR_ERR(opp);
	}
#endif
	old_vproc = dev_pm_opp_get_voltage(opp);
	rcu_read_unlock();
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	dev_pm_opp_put(opp);
#endif
#if defined(CONFIG_ARM_MTKTV_CPUFREQ_CA)
	if (old_freq_hz != freq_hz) {
		if (suspend == 0) {
			if (MTKTV_CPUFREQ_TEEC_Adjust(freq_hz/1000000)
				!= TA_SUCCESS) {
				pr_err("[mtktv-cpufreq] secure adjust fail\n"
						"clock: %ld MHz\n",
						freq_hz/1000000);
			} else {
				u32Current = freq_hz/1000;
			}
		}
	}
#else
	/*
	 * If the new voltage or the intermediate voltage is higher than the
	 * current voltage, scale up voltage first.
	 */
	if (old_vproc < vproc) {
		ret = mtktv_cpufreq_set_voltage(info, vproc);
		if (ret) {
			pr_err("[mtktv-cpufreq] cpu%d: failed to scale up voltage!\n",
					cpu);
			mutex_unlock(&target_mutex);
			return ret;
		}
#ifdef CONFIG_MSTAR_PWM
		usleep_range(20,20);
#endif
	}

	if (old_freq_hz != freq_hz) {
		mtktv_cpufreq_set_clk(old_freq_hz/1000, freq_hz/1000);
		u32Current = freq_hz/1000;
	}

	/*
	 * If the new voltage is lower than the intermediate voltage or the
	 * original voltage, scale down to the new voltage.
	 */
	if (vproc < old_vproc) {
		ret = mtktv_cpufreq_set_voltage(info, vproc);
		if (ret) {
			pr_err("[mtktv-cpufreq] cpu%d: failed to scale down voltage!\n",
					cpu);
			mutex_unlock(&target_mutex);
			return ret;
		}
	}
#endif
	mutex_unlock(&target_mutex);
	return 0;
}

static int mtktv_cpufreq_suspend(struct cpufreq_policy *policy)
{
	struct mtktv_cpu_dvfs_info *info;
	unsigned int cpu = policy->cpu;

	mutex_lock(&boost_mutex);

	/* park boosting thread */
	kthread_park(boosting_detect_task);

	/* cancel all possible kwork */
	cancel_delayed_work(&disable_work);
	current_boost.id = E_MTKTV_BOOST_NONE;
	current_boost.time = 0;
	current_boost.rate = 0;
	suspend = 1;

	/*
	 * set cpufreq policy to default state and
	 * set frequency to default clock
	 */
	info = mtktv_cpu_dvfs_info_lookup(cpu);
	policy->user_policy.min = info->default_min;
	policy->user_policy.max = info->default_max;
	cpufreq_update_policy(cpu);
	mtktv_cpufreq_set_target(policy, 1);

	regulator_init = 0;
#if !defined(CONFIG_ARM_MTKTV_CPUFREQ_CA)
	if (deep_suspend != -1)
		regulator_disable(info->reg);
#endif
	mutex_unlock(&boost_mutex);
	return 0;
}

static int mtktv_cpu_dvfs_info_init(struct mtktv_cpu_dvfs_info *info, int cpu)
{
	struct device *cpu_dev;
	struct dev_pm_opp *opp;
	struct regulator *reg = ERR_PTR(-ENODEV);
	unsigned long rate;
	int ret;

	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev) {
		pr_err("[mtktv-cpufreq] failed to get cpu%d device\n", cpu);
		return -ENODEV;
	}
	reg = regulator_get(NULL, "mtktv-regulator");
	if (IS_ERR(reg)) {
		if (PTR_ERR(reg) == -EPROBE_DEFER)
			pr_err("[mtktv-cpufreq] proc regulator\n"
					"for cpu%d not ready\n"
					"retry.\n", cpu);
		else
			pr_err("[mtktv-cpufreq] failed to\n"
					"get proc regulator\n"
					"for cpu%d\n", cpu);
		ret = PTR_ERR(reg);
		goto out_free_resources;
	}
	info->reg = reg;

	/*
	 * Get OPP-sharing information from
	 * "operating-points-v2" bindings
	 */
	ret = dev_pm_opp_of_get_sharing_cpus(cpu_dev, &info->cpus);
	if (ret) {
		pr_err("[mtktv-cpufreq] failed to\n"
			"get OPP-sharing information\n"
			"for cpu%d\n", cpu);
		goto out_free_resources;
	}

	ret = dev_pm_opp_of_cpumask_add_table(&info->cpus);
	if (ret) {
		pr_err("[mtktv-cpufreq] no OPP table for cpu%d\n", cpu);
		goto out_free_resources;
	}

	rate = mtktv_cpufreq_get_clk(cpu)*1000;
#if KERNEL_VERSION(4, 19, 0) > LINUX_VERSION_CODE
	rcu_read_lock();
#endif
	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &rate);
#if KERNEL_VERSION(4, 19, 0) > LINUX_VERSION_CODE
	rcu_read_unlock();
#endif
	if (IS_ERR(opp)) {
		pr_err("[mtktv-cpufreq] failed to get intermediate opp\n"
			"for cpu%d\n", cpu);
		ret = PTR_ERR(opp);
		goto out_free_opp_table;
	}
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	dev_pm_opp_put(opp);
#endif
	info->cpu_dev = cpu_dev;
	return 0;

out_free_opp_table:
	dev_pm_opp_of_cpumask_remove_table(&info->cpus);

out_free_resources:
	if (!IS_ERR(reg))
		regulator_put(reg);
	return ret;
}

static void mtktv_cpu_dvfs_info_release(struct mtktv_cpu_dvfs_info *info)
{
	if (!IS_ERR(info->reg))
		regulator_put(info->reg);
	dev_pm_opp_of_cpumask_remove_table(&info->cpus);
}

static struct cpufreq_driver mtktv_cpufreq_driver = {
	.verify         = cpufreq_generic_frequency_table_verify,
	.target_index   = mtktv_cpufreq_set_target,
	.get            = mtktv_cpufreq_get_clk,
	.init           = mtktv_cpufreq_init,
	.exit           = mtktv_cpufreq_exit,
	.ready          = mtktv_cpufreq_ready,
	.resume         = mtktv_cpufreq_resume,
	.suspend        = mtktv_cpufreq_suspend,
	.name           = "mtktv-cpufreq",
	.attr           = cpufreq_generic_attr,
};

#if defined(CONFIG_ARM_MTKTV_CPUFREQ_CA)
static int cpufreq_driver_suspend_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	if (auto_measurement)
		return NOTIFY_DONE;
	switch (event) {
	case PM_SUSPEND_PREPARE:
	{
		mutex_lock(&target_mutex);
		pr_info("[%s][%d]: PM_SUSPEND_PREPARE\n",
			__func__, __LINE__);
		if (MTKTV_CPUFREQ_TEEC_Suspend() != TA_SUCCESS) {
			pr_err("[mtktv-cpufreq]\n"
				"CPUFREQ TA Suspend fail\n");
			mutex_unlock(&target_mutex);
			return NOTIFY_BAD;
		};
		suspend = 1;
		u32Current = suspend_frequency;
		pr_info("[%s][%d] Suspend Clock %d KHz\n",
			__func__, __LINE__, u32Current);
		mutex_unlock(&target_mutex);
		break;
	}
	default:
		break;
	}
	return NOTIFY_DONE;
}

static int cpufreq_driver_resume_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	if (auto_measurement)
		return NOTIFY_DONE;
	switch (event) {
	case PM_POST_SUSPEND:
	{
		mutex_lock(&target_mutex);
		pr_info("[%s][%d]: PM_POST_SUSPEND\n",
			__func__, __LINE__);
		if (deep_suspend == 1) {
			if (MTKTV_CPUFREQ_TEEC_Resume() != TA_SUCCESS) {
				pr_err("[mtktv-cpufreq]\n"
					"CPUFREQ TA Resume fail\n");
					mutex_unlock(&target_mutex);
				return NOTIFY_BAD;
			};
			deep_suspend = 0;
		}
		suspend = 0;
		mutex_unlock(&target_mutex);
		mtktv_boost_register(E_MTKTV_LUNCH_BOOST, 0,
			E_BOOST_ENABLE_WITH_TIME, LUNCH_BOOST_TIME_MS);
		break;
	}
	default:
		break;
	}
	return NOTIFY_DONE;
}
#endif

static int mtktv_cpufreq_suspend_noirq(struct device *dev)
{
	deep_suspend = 1;
	return 0;
}

static int mtktv_cpufreq_probe(struct platform_device *pdev)
{
	struct mtktv_cpu_dvfs_info *info, *tmp;
	int cpu, ret;

	for_each_possible_cpu(cpu) {
		info = mtktv_cpu_dvfs_info_lookup(cpu);
		if (info)
			continue;

		info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
		if (!info) {
			ret = -ENOMEM;
			goto release_dvfs_info_list;
		}

		ret = mtktv_cpu_dvfs_info_init(info, cpu);
		if (ret) {
			dev_err(&pdev->dev,
			"[mtktv-cpufreq] failed to initialize dvfs info\n"
			"for cpu%d\n", cpu);
			goto release_dvfs_info_list;
		}

		list_add(&info->list_head, &dvfs_info_list);
	}

	INIT_DELAYED_WORK(&disable_work, mtktv_disable_boost);
	mtktv_cpufreq_opp_init();
	mtktv_boost_init();
	ret = cpufreq_register_driver(&mtktv_cpufreq_driver);
	if (ret) {
		dev_err(&pdev->dev, "[mtktv-cpufreq] failed to register\n"
				"mtk cpufreq driver\n");
		goto release_dvfs_info_list;
	}

	mtvtv_cpufreq_procfs_init();

#if defined(CONFIG_ARM_MTKTV_CPUFREQ_CA)
	/* need to do suspend handler before optee suspend */
	cpufreq_pm_suspend_notifier.notifier_call =
		cpufreq_driver_suspend_event;
	cpufreq_pm_suspend_notifier.priority = 99;
	register_pm_notifier(&cpufreq_pm_suspend_notifier);

	/* need to do resume handler after optee resume */
	cpufreq_pm_resume_notifier.notifier_call =
		cpufreq_driver_resume_event;
	cpufreq_pm_resume_notifier.priority = 0;
	register_pm_notifier(&cpufreq_pm_resume_notifier);

	lunch_boost_task = kthread_create(lunch_boost_thread,
		NULL, "Lunch_Boost_Check");
	wake_up_process(lunch_boost_task);
#endif
	return 0;

release_dvfs_info_list:
	list_for_each_entry_safe(info, tmp, &dvfs_info_list, list_head) {
		mtktv_cpu_dvfs_info_release(info);
		list_del(&info->list_head);
	}
	return ret;
}

static const struct dev_pm_ops mtktv_cpufreq_dev_pm_ops = {
	.suspend_noirq = mtktv_cpufreq_suspend_noirq,
};

static struct platform_driver mtktv_cpufreq_platdrv = {
	.driver = {
		.name   = "mtktv-cpufreq",
		.pm = &mtktv_cpufreq_dev_pm_ops,
	},
	.probe = mtktv_cpufreq_probe,
};

/* List of machines supported by this driver */
static const struct of_device_id mtktv_cpufreq_machines[] __initconst = {
	{ .compatible = "arm,m7332",  },
	{ .compatible = "arm,m7632",  },
	{ .compatible = "arm,m7642",  },
	{ .compatible = "arm,mt5862", },
	{ .compatible = "arm,mt5867", },
	{ .compatible = "arm,mt5889", },
	{},
};

static int __init mtktv_cpufreq_driver_init(void)
{
	struct device_node *np;
	const struct of_device_id *match;
	struct platform_device *pdev;
	int err;

	np = of_find_node_by_path("/");
	if (!np)
		return -ENODEV;
	match = of_match_node(mtktv_cpufreq_machines, np);
	of_node_put(np);
	if (!match) {
		pr_err("[mtktv-cpufreq] Machine is not support dvfs\n");
		return -ENODEV;
	}
#if defined(CONFIG_MSTAR_MT5867)
	if (cpufreq_readw(REG_PMTOP_VERSION) == 0) {
		pr_err("[mtktv-cpufreq] Machine is not support dvfs\n");
		return -ENODEV;
	}
#endif
	err = platform_driver_register(&mtktv_cpufreq_platdrv);
	if (err)
		return err;
	/*
	 * Since there's no place to hold device registration code and no
	 * device tree based way to match cpufreq driver yet, both the driver
	 * and the device registration codes are put here to handle defer
	 * probing.
	 */
	pdev = platform_device_register_simple("mtktv-cpufreq", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("[mtktv-cpufreq] failed to register\n"
			"mtk-cpufreq platform device\n");
		return PTR_ERR(pdev);
	}
	return 0;
}



static int __init auto_test(char *str)
{
	if (strcmp(str, "enable") == 0) {
		pr_info("\033[32m auto_measurement enable \033[0m\n");
		auto_measurement = 1;
	} else {
		auto_measurement = 0;
	}
	return 0;
}

static int __init cpufreq_debug(char *str)
{
	if (strcmp(str, "1") == 0) {
		pr_info("\033[32m cpufreq debug enable \033[0m\n");
		debug_enable = 1;
	} else {
		debug_enable = 0;
	}
	return 0;
}

static int __init cpufreq_info(char *str)
{
	if (strcmp(str, "1") == 0) {
		pr_info("\033[32m cpufreq info enable \033[0m\n");
		info_enable = 1;
	} else {
		info_enable = 0;
	}
	return 0;
}

static int __init cpufreq_tracer(char *str)
{
	if (strcmp(str, "1") == 0) {
		pr_info("\033[32m cpufreq tracer enable \033[0m\n");
		tracer_enable = 1;
	} else {
		tracer_enable = 0;
	}
	return 0;
}

static int __init cpufreq_slttest(char *str)
{
	if (strcmp(str, "1") == 0) {
		pr_info("\033[32m cpufreq slt test enable \033[0m\n");
		slttest_enable = 1;
	} else {
		slttest_enable = 0;
	}
	return 0;
}

early_param("DVFS_MEASURE", auto_test);
early_param("CPUFREQ_DEBUG", cpufreq_debug);
early_param("CPUFREQ_INFO", cpufreq_info);
early_param("CPUFREQ_TRACER", cpufreq_tracer);
early_param("CPUFREQ_SLTTEST", cpufreq_slttest);

late_initcall(mtktv_cpufreq_driver_init);

MODULE_DESCRIPTION("MediaTek CPUFreq driver");
MODULE_AUTHOR("XXX <XXX@mediatek.com>");
MODULE_LICENSE("GPL");
