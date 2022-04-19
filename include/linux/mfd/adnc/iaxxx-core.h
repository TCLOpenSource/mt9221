
/*
 * iaxxx-core.h  --  Knowles ALSA SoC Audio PCM driver header
 *
 * Copyright 2017 Knowles Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __IAXXX_CORE_H__
#define __IAXXX_CORE_H__

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>

typedef int (*iaxxx_cb_func_ptr_t)(struct device *dev);
typedef int (*iaxxx_cb_bc_func_ptr_t)(struct device *dev, u32 iaxxx_intf_speed);
typedef u32 (*iaxxx_cb_un_func_ptr_t)(struct device *dev);
struct workqueue_struct;

#define MAX_FILENAME_BUFFER_SIZE	128

/* SRB is at the end of the memory map */
#define IAXXX_RBDT_NUM_ENTRIES	32

#define MAX_IAXXX_DEVICES	8

#define IAXXX_REGMAP_NAME  "regmap_main"
#define IAXXX_REGMAP_NO_PM_NAME  "regmap_no_pm"

#define IAXXX_BLOCK_0 0 /* Update for DMX PROC */
#define IAXXX_BLOCK_1 1 /* Update for HMD PROC */

#define IAXXX_PKG_ID_MASK	0x00FF
#define IAXXX_PLGIN_ID_MASK	0xF

#define MAX_CORES	2

#define IAXXX_MAX_KW_EVENTS	5

/*********************/
/*   GLOBAL MEMORY   */
/*********************/
#define GLOBAL_DRAM_START	0x40090000
#define GLOBAL_DRAM_END		0x400A0000
/*  GLOBAL MEMORY END  */

/*********************/
/*   HMD Processor   */
/*********************/
#define HMD_SYS_OFFSET		0x86000000

/********* HMD DATA MEMORY START *********/
#define HMD_DMEM_LCL_START	0x20000000
#define HMD_DMEM_SYS_START	(HMD_DMEM_LCL_START + HMD_SYS_OFFSET)

#define HMD_IMEM_LCL_END	0x20600000
#define HMD_IMEM_SYS_END	(HMD_IMEM_LCL_END + HMD_SYS_OFFSET)
/**************** HMD PROCESSOR MEMORY END *****************/

/*********************/
/*   DMX Processor   */
/*********************/
#define DMX_SYS_OFFSET		0x88000000

/********* DMX DATA MEMORY START *********/
#define DMX_DMEM_LCL_START	0x20000000
#define DMX_DMEM_SYS_START	(DMX_DMEM_LCL_START + DMX_SYS_OFFSET)

#define DMX_IMEM_LCL_END	0x20600000
#define DMX_IMEM_SYS_END	(DMX_IMEM_LCL_END + DMX_SYS_OFFSET)
/**************** DMX PROCESSOR MEMORY END *****************/

/* Flags denoting various states of FW and events */
enum {
	IAXXX_FLG_STARTUP, /* Initial state */
	IAXXX_FLG_FW_READY, /* FW is downloaded successully and is in app mode*/
	IAXXX_FLG_FW_CRASH, /* FW has crashed */
	IAXXX_FLG_PM_SUSPEND, /* Suspend state */
	IAXXX_FLG_RESUME_BY_STARTUP, /* System boot up */
	IAXXX_FLG_RESUME_BY_RECOVERY, /* FW update and resume after fw crash */
	IAXXX_FLG_CHIP_WAKEUP, /* chip Wakeup event for HOST0 */
	IAXXX_FLG_FW_RELOAD, /* FW reload request start */
	IAXXX_FLG_RESUME_BY_FW_RELOAD, /* FW reload request complete */
	IAXXX_FLG_FW_RESET, /* FW reset request from HAL, when HAL crashes */
	IAXXX_FLG_RESUME_BY_FW_RESET, /* FW reset request complete */
};

enum {
	IAXXX_READ,
	IAXXX_WRITE,
};

struct pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *active_gpios;
	u32 has_pinctrl;
};

struct iaxxx_client {
	struct regmap *regmap;
};

/* Probed Bus */
enum iaxxx_bus {
	IAXXX_I2C = 0,
	IAXXX_SPI,
	IAXXX_UART,
};

enum iaxxx_events {
	IAXXX_EV_UNKNOWN = 0,		/* Reserve value 0 */
	IAXXX_EV_APP_MODE,		/* FW entered in application mode */
	IAXXX_EV_STARTUP,		/* First ready system startup */
	IAXXX_EV_RECOVERY,		/* Recovery complete after fw crash */
	IAXXX_EV_CRASH,			/* Notify for FW crash */
	IAXXX_EV_FW_RESET,		/* FW reset forcefully*/
	IAXXX_EV_ROUTE_ACTIVE,		/* Audio routing path is done */
	IAXXX_EV_PACKAGE,		/* Loaded plugin */
	IAXXX_EV_TUNNEL,		/* Tunnelling-related event */
	IAXXX_EV_RELOAD_START,		/* FW load start */
	IAXXX_EV_RELOAD_DONE,		/* FW load done */
};

enum iaxxx_plugin_state {
	IAXXX_PLUGIN_UNLOADED = 0,
	IAXXX_PLUGIN_LOADED,
	IAXXX_PLUGIN_ACTIVE,
	IAXXX_PLUGIN_RESET,
};

enum iaxxx_pkg_state {
	IAXXX_PKG_UNLOADED = 0,
	IAXXX_PKG_LOADED,
};

struct iaxxx_plugin_data {
	uint32_t proc_id;
	uint32_t inst_id;
	enum iaxxx_plugin_state plugin_state;
	bool log_enabled;
	u8 block_id;
	struct list_head plugin_node;
};

struct iaxxx_pkg_data {
	uint32_t proc_id;
	uint32_t pkg_id;
	enum iaxxx_pkg_state pkg_state;
	struct list_head pkg_node;
};

struct iaxxx_block_err {
	uint32_t plg_inst_id[MAX_CORES];
	uint32_t error[MAX_CORES];
};

struct iaxxx_system_state {
	struct mutex plg_pkg_list_lock;
	struct list_head plugin_head_list;
	struct list_head pkg_head_list;
	/* mutex to sync script load/unload events */
	struct mutex script_lock;
	/* mutex to sync script list load/unload events */
	struct mutex script_list_lock;
	struct list_head script_head_list;
	atomic_t power_state;
	struct iaxxx_block_err err;
	/* Control interface enabled or disabled */
	atomic_t disable_ctrl_intf;
};

struct log_header {
	u16 block_id;
	u16 log_type;
	u32 log_info;
	u32 start_addr;
	u32 size;
};

struct iaxxx_crashlog {
	struct log_header *header;
	char *log_buffer;
	uint32_t log_buffer_size;
	uint32_t logs_read;
};

struct iaxxx_debug_log {
	struct log_header *header;
	char *kbuf;
	u32 data_written;
	u32 read_data;
	bool read_status;
};

enum {
	IAXXX_POLL,
	IAXXX_EVENT,
};

struct iaxxx_rbdt_info {
	uint32_t hdr_base_addr;
	uint32_t grp_base_addr;
	uint16_t hdr_size;
	uint16_t grp_size;
	uint8_t num_of_hdr;
	uint8_t num_of_grp;
};

/**
 * Description of driver private data
 *
 * @dev: device pointer
 * @regmap: the device register map
 * @regmap_config: regmap configuration data
 * @regmap_init: bus specific regmap init function
 * @rbdt_info: cached copy of the Register Block Descriptor Table
 */
struct iaxxx_priv {
	struct device *dev;
	u32 dev_id;
	struct device *codec_dev;
	struct pinctrl_info pnctrl_info;
	struct regmap *regmap;
	struct regmap_config *regmap_config;
	struct regmap *regmap_no_pm;
	struct regmap_config *regmap_no_pm_config;
	int (*regmap_init_bus)(struct iaxxx_priv *priv);
	int (*bus_read)(struct device *dev, uint32_t reg,
				void *buf, size_t words);
	int (*bus_write)(struct device *dev, uint32_t reg,
				const void *buf, size_t words);

	struct iaxxx_rbdt_info rbdt_info[IAXXX_RBDT_NUM_ENTRIES];

	const struct firmware *fw;
	char fw_name[MAX_FILENAME_BUFFER_SIZE];

	/* GPIOs */
	int reset_gpio;
	int event_gpio;
	int wakeup_gpio;
	int tvonoff_gpio;

	u32 app_speed;
	u32 spi_sbl_speed;

	/* Regulators */
	struct regulator *vdd_io;
	struct regulator *vdd_core;

	/* External Clock */
	struct clk *ext_clk;

	/* Update block lock */
	struct mutex update_block_lock;

	/* Register plugin locks */
	struct mutex plugin_lock;

	/* Event work queue */
	struct mutex event_work_lock;
	struct mutex event_queue_lock;
	struct work_struct event_work_struct;
	struct workqueue_struct *event_workq;

	/* Core kthread */
	struct task_struct *thread;
	struct kthread_worker worker;

	/* Work for device init */
	struct kthread_work fw_update_work;
	struct kthread_work fw_crash_work;
	struct kthread_work runtime_work;
	struct kthread_work stop_kw_buffering_work;
	/* Adapt for MT9615 who doesn't support request_threaded_irq() */
	struct kthread_work threaded_isr_work;

	wait_queue_head_t boot_wq;
	wait_queue_head_t wakeup_wq;
	wait_queue_head_t irq_wake;

	void *tunnel_data;

	/* Event Manager */
	struct mutex event_lock;

	enum iaxxx_bus bus;

	/* For factory testing */
	struct work_struct dev_fw_update_test_work;
	struct work_struct dev_cmem_test_work;
	struct completion bootup_done;
	struct completion cmem_done;
	bool test_result;
	iaxxx_cb_bc_func_ptr_t intf_speed_setup;
	iaxxx_cb_func_ptr_t reset_cb;
	iaxxx_cb_un_func_ptr_t get_intf_speed;
	uint32_t fw_retry_count;
	struct mutex test_mutex;
	struct iaxxx_evt_queue *event_queue;

	/* Showing mode, boot (0) or app (1) */
	bool is_application_mode;
	struct iaxxx_raw_bus_ops *raw_ops;
	struct iaxxx_reg_dump_priv *reg_dump;
	void *intf_priv;
	atomic_t pm_resume;
	void *dfs_node;

	/* Notifiers */
	struct srcu_notifier_head core_notifier_list;
	struct notifier_block notifier_core;

	/* iaxxx core flags for atomic bit field operations */
	unsigned long flags;

	struct iaxxx_system_state *iaxxx_state;
	uint32_t crash_count;
	bool core_crashed;
	atomic_t route_status;
	struct iaxxx_crashlog *crashlog;
	struct mutex crashdump_lock;

	struct iaxxx_debug_log *debug_log;

	/* Debug flags */
	bool   debug_isr_disable;
	bool   debug_fw_crash_handling_disable;
	bool   debug_runtime_pm_disable;

	bool is_irq_enabled;
	bool boot_completed;

	int (*bus_open)(struct iaxxx_priv *priv);
	int (*bus_close)(struct iaxxx_priv *priv);

	bool need_extra_delay_for_rd_wr;
	u32 tunnel_threshold;
	bool tunnel_method;
	/* System clock */
	u32 sys_clk_src;
	u32 sys_clk_in_freq;
	u32 sys_clk_out_freq;

	u32 power_status;

	/*
	 * Control Interface speed required after switching power
	 * mode from Optimal to Normal. This speed should be updated
	 * before set the power mode to Optimal
	 */
	u32 intf_speed_addr;
	u32 intf_max_speed;
	u32 update_intf_max_speed;

	/* FW Crash info */
	int fw_crash_reasons;
	int recovery_try_count;
	int try_count;

	bool in_suspend;
	bool in_resume;
	struct mutex pm_mutex;
	u32 kw_detect_count[IAXXX_MAX_KW_EVENTS];
	/* Flag to change the wakeup pin to SPI CS */
	bool wakeup_from_cs;
	/* To disable optimal to normal power transition */
	bool disable_optimal_to_normal;
	/* To store core memory retention state */
	bool retain_mem[MAX_CORES];
	bool jump_to_app_mode;
	bool isr_from_resume;
	bool is_wifi_wakeup;
};

enum update_block_options_t	{
	UPDATE_BLOCK_NO_OPTIONS = 0x0,
	UPDATE_BLOCK_FIXED_WAIT = 0x1,
	UPDATE_BLOCK_NO_LOCK = 0x2,
};

static inline struct iaxxx_priv *to_iaxxx_priv(struct device *dev)
{
	return dev ? dev_get_drvdata(dev) : NULL;
}

static inline bool iaxxx_core_get_route_status(struct iaxxx_priv *priv)
{
	return atomic_read(&priv->route_status);
}

static inline void iaxxx_core_set_route_status(struct iaxxx_priv *priv,
		bool route_status)
{
	/* Set the route status only if FW is in normal mode */
	if (test_bit(IAXXX_FLG_FW_READY, &priv->flags))
		atomic_set(&priv->route_status, route_status);
}

static inline bool iaxxx_is_firmware_ready(struct iaxxx_priv *priv)
{
	return test_bit(IAXXX_FLG_FW_READY, &priv->flags);
}

int iaxxx_send_update_block_request(struct device *dev, uint32_t *status,
							int id);
int iaxxx_send_update_block_request_with_options(struct device *dev,
					int block_id,
					struct regmap *regmap,
					uint32_t wait_time_in_ms,
					enum update_block_options_t options,
					uint32_t *status);
int iaxxx_core_plg_get_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t *param_val, uint32_t block_id);
int iaxxx_core_plg_set_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t param_val, uint32_t block_id);
int iaxxx_core_create_plg(struct device *dev, uint32_t inst_id,
				uint32_t priority, uint32_t pkg_id,
				uint32_t plg_idx, uint8_t block_id,
				uint32_t config_id);
int iaxxx_core_change_plg_state(struct device *dev, uint32_t inst_id,
				uint8_t is_enable, uint8_t block_id);
int iaxxx_core_destroy_plg(struct device *dev, uint32_t inst_id,
				uint8_t block_id);
int iaxxx_core_reset_plg(struct device *dev, uint32_t inst_id,
				uint8_t block_id);
int iaxxx_core_plg_set_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t param_val, uint32_t block_id);
int iaxxx_core_set_create_cfg(struct device *dev, uint32_t inst_id,
		uint32_t cfg_size, uint64_t cfg_val, uint32_t block_id,
		char *file);
int iaxxx_core_set_param_blk(struct device *dev, uint32_t inst_id,
			uint32_t blk_size, const void *ptr_blk,
			uint32_t block_id,
			uint32_t param_blk_id);
int iaxxx_core_set_param_blk_from_file(struct device *dev, uint32_t inst_id,
			uint32_t block_id, uint32_t param_blk_id,
			const char *file);
int iaxxx_core_evt_subscribe(struct device *dev, uint16_t src_id,
		uint16_t event_id, uint16_t dst_id, uint32_t dst_opaque);
int iaxxx_core_evt_unsubscribe(struct device *dev, uint16_t src_id,
		uint16_t event_id, uint16_t dst_id);
int iaxxx_core_retrieve_event(struct device *dev, uint16_t *event_id,
		uint32_t *data, uint16_t *event_src);
int iaxxx_core_set_event(struct device *dev, uint8_t inst_id,
			uint32_t event_enable_mask, uint32_t block_id);

int iaxxx_core_resume_rt(struct device *dev);
int iaxxx_core_suspend_rt(struct device *dev);
int iaxxx_core_dev_resume(struct device *dev);
int iaxxx_core_dev_suspend(struct device *dev);
void iaxxx_pm_complete(struct device *dev);

int iaxxx_package_load(struct device *dev, const char *pkg_name,
					uint32_t pkg_id, uint32_t *proc_id);
int iaxxx_package_unload(struct device *dev, uint32_t pkg_id);
int iaxxx_get_package_type(struct device *dev, uint32_t pkg_id,
			   uint8_t *pkg_type);
int iaxxx_set_channel_gain(struct device *dev, uint8_t id,
			int8_t target_gain, uint16_t gain_ramp,
			uint8_t block_id);
int iaxxx_clr_pkg_plg_list(struct iaxxx_priv *priv);

void register_transac_log(struct device *dev, uint32_t reg, uint32_t val,
								uint8_t op);
int iaxxx_get_crashlog_header(struct iaxxx_priv *priv);
int iaxxx_dump_crashlogs(struct iaxxx_priv *priv);
int plugin_log_enable(struct iaxxx_priv *priv, u32 plg_inst, u8 block_id);

int iaxxx_fw_notifier_register(struct device *dev, struct notifier_block *nb);
int iaxxx_fw_notifier_unregister(struct device *dev, struct notifier_block *nb);
int iaxxx_fw_notifier_call(struct device *dev, unsigned long val, void *v);

int iaxxx_pm_get_sync(struct device *dev);
int iaxxx_pm_put_autosuspend(struct device *dev);
int iaxxx_pm_put_sync_suspend(struct device *dev);

bool iaxxx_core_plg_is_valid_pkg_id(uint32_t pkg_id);
bool iaxxx_core_plg_is_valid_priority(uint32_t priority);
bool iaxxx_core_plg_is_valid_block_id(uint32_t block_id);
bool iaxxx_core_plg_is_valid_plg_idx(uint32_t plg_idx);
bool iaxxx_core_plg_is_valid_param_id(uint32_t param_id);
bool iaxxx_core_plg_is_valid_param_val(uint32_t param_val);
bool iaxxx_core_plg_is_valid_param_blk_id(uint32_t param_id);
bool iaxxx_core_plg_is_valid_param_blk_size(uint32_t param_size);
bool iaxxx_core_plg_is_valid_cfg_size(uint32_t cfg_size);
int iaxxx_check_proc_pkg_plg_list_empty(struct iaxxx_priv *priv,
		uint32_t proc_id, bool *status);
bool iaxxx_core_pkg_plg_list_empty(struct iaxxx_priv *priv);

bool iaxxx_core_evt_is_valid_src_id(uint32_t src_id);
bool iaxxx_core_evt_is_valid_dst_id(uint32_t dst_id);
bool iaxxx_core_evt_is_valid_event_id(uint32_t event_id);
bool iaxxx_core_evt_is_valid_dst_opaque(uint32_t dst_opaque);
struct iaxxx_plugin_data *iaxxx_core_plugin_exist(struct iaxxx_priv *priv,
							uint32_t inst_id);
int iaxxx_core_plg_get_package_version(struct device *dev,
		uint8_t inst_id, char *ver, uint32_t len);
int iaxxx_core_plg_get_plugin_version(struct device *dev,
		uint8_t inst_id, char *ver, uint32_t len);
unsigned int iaxxx_get_num_packages(struct iaxxx_priv *priv);
int iaxxx_boot_proc(struct iaxxx_priv *priv, u32 proc_id_mask);
void *iaxxx_kvmalloc(size_t size, gfp_t flags);
int iaxxx_core_evt_trigger(struct device *dev,
		uint16_t src_id, uint16_t evt_id, uint32_t src_opaque);
int iaxxx_update_intf_speed(struct device *dev, uint32_t intf_speed);
int iaxxx_fw_reset(struct iaxxx_priv *priv);

bool is_valid_tnl_cfg_params(uint32_t tunlSrc, uint32_t tunlMode,
							uint32_t tunlEncode);
int _tunneling_open_stub(struct iaxxx_priv *priv, void **client_ptr);
int _tunnel_release_stub(struct iaxxx_priv *priv, void *client_data);
ssize_t _tunneling_read_to_alsa_tnl_bfr(struct iaxxx_priv *priv,
				void *client_data, void *buf, size_t count);
int tunnel_setup_an_endpoint_stub(void *client_data,
				uint32_t src, uint32_t mode, uint32_t encode);
int tunnel_term_an_endpoint_stub(void *client_data,
				uint32_t src, uint32_t mode, uint32_t encode);
int iaxxx_set_tunnel_output_buffer_size(struct iaxxx_priv *priv,
					uint32_t tunnel_buff_size);

#endif /*__IAXXX_CORE_H__ */
