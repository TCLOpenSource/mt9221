#ifndef CPUFREQ_MTKTV_H
#define CPUFREQ_MTKTV_H

#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/types.h>

typedef enum {
	E_MTKTV_DEBUG_BOOST     = 0,
	E_MTKTV_LUNCH_BOOST     = 1,
	E_MTKTV_BENCH_BOOST     = 2,
	E_MTKTV_IR_BOOST        = 3,
	E_MTKTV_BOOST_NONE      = 4,
} mtktv_boost_client_id;

typedef enum {
	E_BOOST_RESULT_OK            = 0x0,
	E_BOOST_RESULT_IGNORE        = 0x1,
	E_BOOST_RESULT_INVALID_ID    = 0x2,
	E_BOOST_RESULT_INVALID_TYPE  = 0x3,
	E_BOOST_RESULT_INVALID_TIME  = 0x4,
} boost_result;

typedef enum {
	E_BOOST_ENABLE             = 0x0,
	E_BOOST_DISABLE            = 0x1,
	E_BOOST_ENABLE_WITH_TIME   = 0x2,
} boost_type;

#if defined(CONFIG_ARM_MTKTV_CPUFREQ_CA)

#define E_DVFS_OPTEE_INIT       (0)
#define E_DVFS_OPTEE_ADJUST     (1)
#define E_DVFS_OPTEE_SUSPEND    (2)
#define E_DVFS_OPTEE_RESUME     (3)

typedef enum {
	TA_SUCCESS = 0x00000000,
	TA_ERROR   = 0xFFFF0006,
} ta_result;

u32 MTKTV_CPUFREQ_TEEC_TA_Open(void);
u32 MTKTV_CPUFREQ_TEEC_Init(void);
u32 MTKTV_CPUFREQ_TEEC_Adjust(u32 frequency);
u32 MTKTV_CPUFREQ_TEEC_Suspend(void);
u32 MTKTV_CPUFREQ_TEEC_Resume(void);
#endif

boost_result mtktv_boost_register(mtktv_boost_client_id id, int freq, boost_type type, int time);
int mtktv_get_cpu_temeprature(void);
#endif

