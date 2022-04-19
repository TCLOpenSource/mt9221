#include <linux/version.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <generated/autoconf.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/async.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/atomic.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 44)
#include <linux/seq_file.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 44)
#include <trace/events/power.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 53)
#include <linux/sched/debug.h>
#endif
#include <power.h>
#include "mdrv_utopia2k_str_io.h"

#define NR_STRCB_MAX 48

#undef pr_fmt
#define pr_fmt(fmt) "PM-UTPA: " fmt

#ifdef pm_pr_dbg
#define utpa_pr_dbg	pm_pr_dbg
#else
#define utpa_pr_dbg	pr_debug
#endif

/*
 * UTOPIA2K_STR_POWER_SUSPEND start at 1 in old version,
 * need to convert to 0 thus match the power_mode and array index
*/
#define CONVERT_PM_MODE(_m)	(_m - 1)
#define UTPA2K_PM_SUSPEND	CONVERT_PM_MODE(UTOPIA2K_STR_POWER_SUSPEND)
#define UTPA2K_PM_RESUME	CONVERT_PM_MODE(UTOPIA2K_STR_POWER_RESUME)
#define UTPA2K_PM_MAX		CONVERT_PM_MODE(UTOPIA2K_STR_POWER_MAX)
/*
 * This allows printing both to /proc and
 * to the console
*/
#define UTPA_printf(_m, x...)			\
do {						\
	if (_m)					\
		seq_printf(_m, x);		\
	else					\
		pr_info(x);			\
} while (0)


#define UTPA_GET_TIME(_t, available)		\
do {						\
	if (debug_enabled && available)		\
		_t = ktime_get();		\
} while(0)

#define UTPA_DIFF_TIME(_d, _e, _p, available)		\
do {							\
	if (debug_enabled && available)			\
		_d = (typeof(_d))ktime_us_delta(_e, _p);\
} while(0)

struct runtime_cond {
	bool called_send;
	bool called_wait;
	bool runtime_pre;
	bool runtime_post;
	int nr_runtime_pre;
};

struct cond_member {
	char name[16];
	struct list_head list;
	struct utpa2k_str_node *cond_node;
};

struct cond_info {
	bool arranged;
	struct runtime_cond rt_cond;
	struct list_head cond_head;
};

struct utpa2k_str_node {
	FUtopiaSTR fpSTR;
	void *data;
	char name[16];
	struct timer_list timer;
	struct task_struct *tsk;
	int mode;
	s64 waittime;
	struct cond_info ci[UTPA2K_PM_MAX];
	struct completion done;
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	enum stage_nr stage;
#endif
};

struct utpa2k_str_data {
	void *key;
	void *val;
};

static const char *power_mode[] = {
	"suspend",
	"resume"
};

#define DFT_WAITFOR_TIMEOUT_MS 2000
#define DFT_UTPA2K_TIMEOUT_MS 800

#ifndef CONSOLE_LOGLEVEL_QUIET
#define CONSOLE_LOGLEVEL_QUIET 4
#endif

static struct utpa2k_str_data *data_pool;
static struct utpa2k_str_node *cb_pool;
static int total_cb = 0;
static DEFINE_MUTEX(cb_pool_mtx);
static unsigned int current_mode;
static unsigned int utpa2k_timeout_ms = DFT_UTPA2K_TIMEOUT_MS ;
static unsigned int waitfor_timeout_ms = DFT_WAITFOR_TIMEOUT_MS;
static ASYNC_DOMAIN_EXCLUSIVE(condition_domain);
static bool debug_enabled;

#ifdef CONFIG_MP_POTENTIAL_BUG
static bool timeout_show_stack = true;
#else
static bool timeout_show_stack;
#endif
static bool timeout_as_panic;
static bool timeout_auto_update = true;
static bool wd_disabled;

bool str_fr_enabled;
atomic_t str_fr_utpa_resume_done = ATOMIC_INIT(1);
DECLARE_WAIT_QUEUE_HEAD(str_fr_waitqueue);
/* export for utopia2k-700 */
EXPORT_SYMBOL(str_fr_enabled);
EXPORT_SYMBOL(str_fr_utpa_resume_done);
EXPORT_SYMBOL(str_fr_waitqueue);

static int __init utpa2k_debug(char *str)
{
	debug_enabled = true;
	return 1;
}
__setup("utpa_str_dbg", utpa2k_debug);

static int __init utpa2k_freerun(char *str)
{
	str_fr_enabled = true;
	return 1;
}
__setup("utpa_str_fr", utpa2k_freerun);

static inline void update_timeout_threshold(void)
{
	if (!timeout_auto_update) {
		pr_info("Timeout update threshold by loglevel is disabled, "
			"please echo 1 > /sys/utpa_power/timeout_autoupdate "
			"to enable it.\n");
		return;
	}

	if (console_loglevel > CONSOLE_LOGLEVEL_QUIET) {
		utpa_pr_dbg("loglevel is %d, finetune timeout threshold: "
			"utpa2k_timeout_ms %d ms, waitfor_timeout_ms %d ms\n",
			console_loglevel,
			DFT_UTPA2K_TIMEOUT_MS * 10, DFT_WAITFOR_TIMEOUT_MS * 4);
		utpa2k_timeout_ms = DFT_UTPA2K_TIMEOUT_MS * 10;
		waitfor_timeout_ms = DFT_WAITFOR_TIMEOUT_MS * 4;
	} else {
		utpa_pr_dbg("loglevel is %d, restore timeout threshold\n",
				console_loglevel);
		utpa2k_timeout_ms = DFT_UTPA2K_TIMEOUT_MS;
		waitfor_timeout_ms = DFT_WAITFOR_TIMEOUT_MS;
	}
}

static inline void show_time(struct utpa2k_str_node *node,
			ktime_t starttime, ktime_t endtime)
{
	u64 usecs64;
	u64 wait;
	int usecs;
	int wait_usecs;

	if (!debug_enabled)
		return;

	wait = node->waittime;
	usecs64 = (u64)ktime_us_delta(endtime, starttime) - wait;
	do_div(usecs64, NSEC_PER_USEC);
	do_div(wait, NSEC_PER_USEC);
	usecs = usecs64;
	wait_usecs = wait;
	if (usecs == 0)
		usecs = 1;

	pr_info("do %s callback %s, wait-cond after %ld.%03ld msecs, "
		"complete after %ld.%03ld msecs\n",
		power_mode[CONVERT_PM_MODE(node->mode)], node->name,
		wait_usecs / USEC_PER_MSEC, wait_usecs % USEC_PER_MSEC,
		usecs / USEC_PER_MSEC, usecs % USEC_PER_MSEC);

	node->waittime = 0;
}

static DEFINE_SPINLOCK(show_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
static void utpa2k_watchdog_handler(struct timer_list *t)
{
	struct utpa2k_str_node *node = from_timer(node, t, timer);
#else
static void utpa2k_watchdog_handler(unsigned long data)
{
	struct utpa2k_str_node *node = (void *)data;
#endif
	unsigned long flags;

#ifdef CONFIG_MP_POTENTIAL_BUG
	console_verbose();
#endif
	pr_err("***** STR module timeout, utopia2k %s %s timeout *****\n",
		node->name, power_mode[CONVERT_PM_MODE(current_mode)]);

	if (timeout_show_stack) {
		spin_lock_irqsave(&show_lock, flags);
		show_stack(node->tsk, NULL);
		spin_unlock_irqrestore(&show_lock, flags);
	}

	if (timeout_as_panic)
		panic("%s timeout\n", node->name);
}

static void utpa2k_watchdog_set(struct utpa2k_str_node *node)
{
	struct timer_list *timer = &node->timer;

	if (wd_disabled)
		return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	timer_setup(timer, utpa2k_watchdog_handler, 0);
#else
	init_timer(timer);
	timer->function = utpa2k_watchdog_handler;
	timer->data = (unsigned long)node;
#endif
	timer->expires = jiffies + msecs_to_jiffies(utpa2k_timeout_ms);
	add_timer(timer);
	node->tsk = current;
}

static void utpa2k_watchdog_clear(struct utpa2k_str_node *node)
{
	struct timer_list *timer = &node->timer;

	if (wd_disabled)
		return;

	del_timer_sync(timer);
}

static inline void _find_with_fixup(const char *buf, const char rm)
{
	char *p;
	for (p = (char *)buf; *p != '\0'; ++p) {
		if (*p == rm)
			memmove(p, p+1, strlen(p));
	}
}

static struct utpa2k_str_node *find_cb_in_pool(const char *name)
{
	int i;
	int rty = 0;
	char tmp[16];

	snprintf(tmp, sizeof(tmp), "%s", name);
retry:
	/*
	 * no one will modify cb_pool[n].name in this moment,
	 * don't need the lock
	*/
	for (i = 0; i < total_cb; ++i) {
		if (strstr(cb_pool[i].name, tmp))
			return &cb_pool[i];
	}

	if (i == total_cb) {
		rty++;
		switch (rty) {
		case 1:
			// ignore '_', cb_foo equal to cbfoo
			pr_warn("WARNING, couldn't find %s in pool\n", tmp);
			_find_with_fixup(tmp, '_');
			pr_warn("retry %d time(s) using %s\n", rty, tmp);
			goto retry;
		case 2:
			// ignore '-', cb-foo equal to cbfoo
			pr_warn("WARNING, couldn't find %s in pool\n", tmp);
			_find_with_fixup(tmp, '-');
			pr_warn("retry %d time(s) using %s\n", rty, tmp);
			goto retry;

		case 3:
			// ignore ' ', cb foo equal to cbfoo
			pr_warn("WARNING, couldn't find %s in pool\n", tmp);
			_find_with_fixup(tmp, ' ');
			pr_warn("retry %d time(s) using %s\n", rty, tmp);
			goto retry;
		// TODO: add more retry here if needed
		case 4:
		case 5:
		case 6:
		default:
			break;
		}
	}
	pr_err("couldn't find %s in pool\n", name);

	return NULL;
}

static inline void dump_cond_info(struct seq_file *s)
{
	int i;
	struct cond_member *cm;

	UTPA_printf(s, "The Utopia2K-STR %s version condition list\n",
			str_fr_enabled ? "free-run" : "normal");

	for (i = 0; i < total_cb; ++i) {
		if (!cb_pool[i].fpSTR)
			continue;

		UTPA_printf(s, "[Module] %s SUSEPND:\n", cb_pool[i].name);
		list_for_each_entry(cm,
			&cb_pool[i].ci[UTPA2K_PM_SUSPEND].cond_head, list)
			UTPA_printf(s, "+wt:\t%s\n", cm->name);

		UTPA_printf(s, "<RT> called-send %s, called-wait %s, rt-pre %s, "
		"rt-post %s, rt-nr-pre %d\n",
		cb_pool[i].ci[UTPA2K_PM_SUSPEND].rt_cond.called_send ? "o" : "x",
		cb_pool[i].ci[UTPA2K_PM_SUSPEND].rt_cond.called_wait ? "o" : "x",
		cb_pool[i].ci[UTPA2K_PM_SUSPEND].rt_cond.runtime_pre ? "o" : "x",
		cb_pool[i].ci[UTPA2K_PM_SUSPEND].rt_cond.runtime_post ? "o" : "x",
		cb_pool[i].ci[UTPA2K_PM_SUSPEND].rt_cond.nr_runtime_pre);

		UTPA_printf(s, "[Module] %s RESUME:\n", cb_pool[i].name);
		list_for_each_entry(cm,
			&cb_pool[i].ci[UTPA2K_PM_RESUME].cond_head, list)
			UTPA_printf(s, "+wt:\t%s\n", cm->name);

		UTPA_printf(s, "<RT> called-send %s, called-wait %s, rt-pre %s, "
		"rt-post %s, rt-nr-pre %d\n\n",
		cb_pool[i].ci[UTPA2K_PM_RESUME].rt_cond.called_send ? "o" : "x",
		cb_pool[i].ci[UTPA2K_PM_RESUME].rt_cond.called_wait ? "o" : "x",
		cb_pool[i].ci[UTPA2K_PM_RESUME].rt_cond.runtime_pre ? "o" : "x",
		cb_pool[i].ci[UTPA2K_PM_RESUME].rt_cond.runtime_post ? "o" : "x",
		cb_pool[i].ci[UTPA2K_PM_RESUME].rt_cond.nr_runtime_pre);
	}
}

static inline struct cond_member *__add_cond_to_list(struct cond_info *ci, const char *cond)
{
	struct cond_member *cm;
	struct utpa2k_str_node *cond_node;

	cond_node = find_cb_in_pool(cond);
	if (!cond_node)
		return NULL;

	cm = kmalloc(sizeof(struct cond_member), GFP_KERNEL);
	if (!cm) {
		pr_err("kmalloc %s condition failed!\n", cond);
		return NULL;
	}

	INIT_LIST_HEAD(&cm->list);
	strncpy(cm->name, cond, sizeof(cm->name) - 1);
	cm->cond_node = cond_node;
	list_add(&cm->list, &ci->cond_head);

	return cm;
}

static int _fillin_cond_info(struct device_node *stg, struct cond_info *ci,
				const char *name)
{
	int i;
	int rc, err = 0;
	const char *conds[16] = {0};
	bool has_precond = false;
	bool has_postcond = false;

	mutex_lock(&cb_pool_mtx);
	rc = of_property_read_string_array(stg, "pre-condition",
			conds, ARRAY_SIZE(conds));
	if (rc < 0) {
		pr_info("No pre-condiction registered in %s\n", name);
	} else if (rc > ARRAY_SIZE(conds)) {
		pr_err("Set number of %s pre-condiction overflow "
			"(more than 32)\n", stg->name);
		err = -EINVAL;
		goto out;
	} else {
		pr_info("Register %d pre-condiction in %s\n", rc, name);
		for (i = 0; i < rc; ++i) {
			if (!__add_cond_to_list(ci, conds[i])) {
				err = -EINVAL;
				goto out;
			}
		}
		has_precond = true;
	}

	/*
	 * We don't need to know post-condition now,
	 * just check it
	*/
	if (of_find_property(stg, "post-condition", NULL))
		has_postcond = true;

	if (!has_precond && !has_postcond) {
		pr_err("WARNING! %s condition defined in DTS is wired!,"
			" please refine it!\n", name);
		err = -EINVAL;
		goto out;
	}

out:
	mutex_unlock(&cb_pool_mtx);
	return err;
}

static void wait_for_each_cond(struct cond_info *ci, const char *name, unsigned int mode)
{
	long t;
	struct cond_member *cm;

	list_for_each_entry(cm, &ci->cond_head, list) {
		utpa_pr_dbg("%s waitfor %s\n", name, cm->cond_node->name);
		t = wait_for_completion_interruptible_timeout(
				&cm->cond_node->done,
				msecs_to_jiffies(waitfor_timeout_ms));
		if (t < 0) {
			pr_err("%s waitfor %s done "
					"but receive interrupt signal\n",
					name, cm->cond_node->name);
			dump_stack();
		} else if (t == 0) {
			pr_err("%s waitfor %s done but timeout\n",
					name, cm->cond_node->name);
			// TODO: if timeout, what can we do?
			show_stack(cm->cond_node->tsk, NULL);
		}
		utpa_pr_dbg("%s wait %s complete!\n", name, cm->cond_node->name);
	}
}

static inline void reinit_all_complete_barrier(void)
{
	int i;
	for (i = 0; i < total_cb; ++i) {
		if (!cb_pool[i].fpSTR)
			continue;
		 reinit_completion(&cb_pool[i].done);
	}
}

static int utpa2k_post_utpacore(void *d)
{
	async_synchronize_full_domain(&condition_domain);
	pr_info("------- free-run: All utopia modules resume done --------\n");
	// to relase utpa-core
	atomic_set(&str_fr_utpa_resume_done, 1);
	wake_up_interruptible_all(&str_fr_waitqueue);

	return 0;
}

static void utpa2k_async_sched(void *data, async_cookie_t cookie)
{
	int err;
	ktime_t starttime = ktime_set(0, 0);
	ktime_t endtime = ktime_set(0, 0);
	struct utpa2k_str_node *node = (struct utpa2k_str_node *)data;

	if (!node->fpSTR) {
		pr_err("ERROR! Callback does not exist!\n");
		return;
	}

	// runtime wait
	if (node->ci[CONVERT_PM_MODE(current_mode)].rt_cond.runtime_pre) {
		pr_info("<RUNTIME> %s wait %s conditions\n",
			node->name, power_mode[CONVERT_PM_MODE(current_mode)]);
		wait_for_each_cond(&node->ci[CONVERT_PM_MODE(current_mode)],
				node->name, CONVERT_PM_MODE(current_mode));
	}

	utpa_pr_dbg("Execute %s\n", node->name);

	if (current_mode == UTOPIA2K_STR_POWER_RESUME)
		utpa2k_watchdog_set(node);

	UTPA_GET_TIME(starttime, true);
	err = node->fpSTR(node->mode, node->data);
	UTPA_GET_TIME(endtime, true);
	show_time(node, starttime, endtime);

	if (current_mode == UTOPIA2K_STR_POWER_RESUME)
		utpa2k_watchdog_clear(node);

	// runtime send complete
	if (node->ci[CONVERT_PM_MODE(current_mode)].rt_cond.runtime_post) {
		pr_info("<RUNTIME> %s send %s complete\n",
			node->name, power_mode[CONVERT_PM_MODE(current_mode)]);
		complete_all(&node->done);
	}
}

static inline int check_pm_mode(unsigned int mode)
{
	// someone stupid use the wrong mode
	if (mode <= 0 || mode >= UTOPIA2K_STR_POWER_MAX) {
		pr_err("ERROR! no such mode %d\n", mode);
		dump_stack();
		return -EINVAL;
	}

	if (mode != current_mode) {
		pr_err("ERROR! set wrong mode, should be %s\n",
			power_mode[CONVERT_PM_MODE(current_mode)]);
		dump_stack();
		return -EINVAL;
	}

	return 0;
}

/*
 * Convert the module name to lower letter and remove suffix
*/
static void inline str_tolower(char *str)
{
	char ch;

	while (*str != ' ') {
		ch = tolower(*str);
		*str++ = ch;
	}
	// we don't want to see [utopia] letters
	*str = '\0';
}

/* ----------------------- the APIs is exported for MSOS ---------------------*/
int utopia2k_str_setup_function_ptr_v2(void *data, FUtopiaSTR fpSTR)
{
	int err = 0;
	struct utpa2k_str_node *node;

	mutex_lock(&cb_pool_mtx);
	// shit! too many utopia modules
	if (total_cb >= NR_STRCB_MAX) {
		pr_err("There is no vacant to register module STR callback, "
			"please increse NR_STRCB_MAX value in %s!\n", __FILE__);
		err = -ENOMEM;
		goto out;
	}

	node = &cb_pool[total_cb];
	node->fpSTR = fpSTR;
	node->data = data;
	snprintf(node->name, sizeof(node->name), "%pf", fpSTR);
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	node->stage = check_stage(node->name);
#endif
	str_tolower(node->name);
	init_completion(&node->done);
	INIT_LIST_HEAD(&node->ci[UTPA2K_PM_SUSPEND].cond_head);
	INIT_LIST_HEAD(&node->ci[UTPA2K_PM_RESUME].cond_head);
	pr_info("register STR callback %s at %d\n", node->name, total_cb);
	total_cb++;

out:
	mutex_unlock(&cb_pool_mtx);
	return err;
}

int utopia2k_str_wait_condition_v2(const char *name,
				MS_U32 mode, MS_U32 stage)
{
	int err;
	int pm_mode;
	struct utpa2k_str_node *node;
	struct device_node *stg;
	char dts_path[64] = {0};
	ktime_t starttime = ktime_set(0, 0);
	ktime_t endtime = ktime_set(0, 0);

	utpa_pr_dbg("%s call %s, mode %d, stage %d\n",
			current->comm, name, mode, stage);

	err = check_pm_mode(mode);
	if (err)
		return err;

	node = find_cb_in_pool(name);
	if (!node)
		return -ENODEV;

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	if (node->stage == STR_STAGE_2) {
		pr_err("%s is registered in stage2, "
		"condition is not available\n", node->name);
		return -EINVAL;
	}
#endif
	pm_mode = CONVERT_PM_MODE(mode);
	/*
	 * mark for module call wait,
	 * to recognize add condition in runtime or not
	*/
	node->ci[pm_mode].rt_cond.called_wait = true;
	/*
	 * Stored module STR conditions from DTS to list first,
	 * then get the conditions from list,
	 * don't measure waiitime at first
	*/
	if (!node->ci[pm_mode].arranged) {
		utpa_pr_dbg("%s in %s but not arranged!\n",
					name, power_mode[pm_mode]);
		snprintf(dts_path, sizeof(dts_path),
			"/%s/%s/%s/stage%d",
			UTOPIA2K_STR_NAME, name, power_mode[pm_mode], stage);
		stg = of_find_node_by_path(dts_path);
		if (!stg) {
			pr_err("<%s> Could not find %s in DTS\n",
				__func__, name);
			return -ENODEV;
		}
		_fillin_cond_info(stg, &node->ci[pm_mode], name);
		node->ci[pm_mode].arranged = true;
	}

	utpa_pr_dbg("%s in %s arranged!\n", name, power_mode[pm_mode]);

	// measure waittime if arranged
	UTPA_GET_TIME(starttime, node->ci[pm_mode].arranged);

	// waitfor
	wait_for_each_cond(&node->ci[pm_mode], name, pm_mode);

	UTPA_GET_TIME(endtime, node->ci[pm_mode].arranged);
	UTPA_DIFF_TIME(node->waittime, endtime,
			starttime, node->ci[pm_mode].arranged);

	return 0;
}

int utopia2k_str_send_condition_v2(const char *name,
				MS_U32 mode, MS_U32 stage)
{
	int err;
	int pm_mode;
	struct utpa2k_str_node *node;

	utpa_pr_dbg("%s call %s, mode %d, stage %d\n",
			current->comm, name, mode, stage);

	err = check_pm_mode(mode);
	if (err)
		return err;

	node = find_cb_in_pool(name);
	if (!node)
		return -ENODEV;

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	if (node->stage == STR_STAGE_2) {
		pr_err("%s is registered in stage2, "
		"condition is not available\n", node->name);
		return -EINVAL;
	}
#endif
	pm_mode = CONVERT_PM_MODE(mode);
	/*
	 * mark for module call wait,
	 * to recognize add condition in runtime or not
	*/
	node->ci[pm_mode].rt_cond.called_send = true;

	utpa_pr_dbg("%s send %s compete\n", node->name, power_mode[pm_mode]);
	complete_all(&node->done);

	return 0;
}

static inline int _alloc_copy_str_data(void **d, char *value)
{
	*d = kmalloc(strlen(value) + 1, GFP_KERNEL);
	if (!*d) {
		pr_err("%s kmalloc failed!\n", __func__);
		return -ENOMEM;
	}
	memcpy(*d, value, strlen(value)+1);
	return 0;
}

int utopia2k_str_set_data_v2(char *key, char *value)
{
	int i, err;
	int vacancy = -1;

	if (!key || !strcmp(key, ""))
		return -EINVAL;

	pr_info("key %s value %s\n", key, value);

	for (i = 0; i < NR_STRCB_MAX; ++i) {
		if (!data_pool[i].key) {
			if (vacancy == -1)
				vacancy = i;
			continue;
		}

		if (!strcmp(data_pool[i].key, key)) { //alread exist
			if (!strcmp(data_pool[i].val, value)) {
				pr_info("key(%s) value(%s) already exists!\n",
					key, value);
				return 0;
			}

			if (data_pool[i].val) {
				kfree(data_pool[i].val);
				data_pool[i].val = NULL;
			}

			if (value && strcmp(value, "")) {
				err = _alloc_copy_str_data(&data_pool[i].val,
						value);
				if (err)
					return err;
				pr_info("replace key(%s) value(%s)\n",
						key, value);
			} else {
				kfree(data_pool[i].key);
				data_pool[i].key = NULL;
				pr_info("remove key(%s)\n", key);
			}
			break;
		}
	}

	if (i == NR_STRCB_MAX) { // new
		err = _alloc_copy_str_data(&data_pool[vacancy].key, key);
		if (err)
			return err;

		err = _alloc_copy_str_data(&data_pool[vacancy].val, value);
		if (err)
			return err;

		pr_info("register key(%s) value(%s)\n", key, value);
	}

	return 0;
}

int utopia2k_str_get_data_v2(char *key, char *value)
{
	int i;

	for (i = 0; i < NR_STRCB_MAX; ++i) {
		if (!data_pool[i].key)
			continue;

		if (!strcmp(data_pool[i].key, key)) {
			memcpy(value, data_pool[i].val,
				strlen(data_pool[i].val) + 1);
			pr_info("get key(%s) value(%s)\n",
				key, value);
			return 0;
		}
	}
	pr_err("cannot find key(%s)\n", key);

	return 0;
}
/* ---------------------------------------------------------------------------*/

#define RUNTIME_COND_PRE	0
#define RUNTIME_COND_POST	1
#define RUNTIME_ADD		0
#define RUNTIME_REMOVE		1

static int runtime_cond_add(struct utpa2k_str_node *root,
			char *name, unsigned int mode, int cond)
{
	struct cond_member *cm;

	if (cond == RUNTIME_COND_PRE) {
		// check duplicate
		list_for_each_entry(cm, &root->ci[mode].cond_head, list) {
			if (!strncmp(cm->name, name, strlen(name))) {
				pr_err("%s is already in %s condition list\n",
					name, root->name);
				return -EINVAL;
			}
		}

		if (!__add_cond_to_list(&root->ci[mode], name))
			return -EINVAL;

		// record number of runtime pre-conditions
		root->ci[mode].rt_cond.nr_runtime_pre++;
		/*
		 * the module didn't call the wait-condiciotn API,
		 * So use runtime wait
		*/
		if (!root->ci[mode].rt_cond.called_wait) {
			utpa_pr_dbg("%s enable runtime pre-condition\n",
				root->name);
			root->ci[mode].rt_cond.runtime_pre = true;
		}
	} else if (cond == RUNTIME_COND_POST) {
		pr_info("%s enable post-condition in runtime\n", root->name);
		if (!root->ci[mode].rt_cond.called_send)
			root->ci[mode].rt_cond.runtime_post = true;
	} else {
		pr_err("%s: invaild condition %d\n", __func__, cond);
		return -EINVAL;
	}

	return 0;
}

static int runtime_cond_rm(struct utpa2k_str_node *root,
			char *name, int mode, int cond)
{
	bool found = false;
	struct cond_member *cm;

	if (cond == RUNTIME_COND_PRE) {
		list_for_each_entry(cm, &root->ci[mode].cond_head, list) {
			if (!strncmp(cm->cond_node->name, name, strlen(name))) {
				pr_info("Found %s in %s condition list\n",
					name, root->name);
				found = true;
				break;
			}
		}

		if (!found) {
			pr_err("%s is not exist in %s condition list\n",
				name, root->name);
			return -ENODEV;
		}

		pr_info("Remove %s in %s condition list\n", name, root->name);
		list_del(&cm->list);
		kfree(cm);

		// record number of runtime pre-conditions
		root->ci[mode].rt_cond.nr_runtime_pre--;
		if (root->ci[mode].rt_cond.nr_runtime_pre == 0) {
			utpa_pr_dbg("%s disable runtime pre-condition\n",
					root->name);
			root->ci[mode].rt_cond.runtime_pre = false;
		}
	} else if (cond == RUNTIME_COND_POST) {
		pr_info("%s disable post-condition in runtime\n", root->name);
		if (root->ci[mode].rt_cond.runtime_post)
			root->ci[mode].rt_cond.runtime_post = false;
	} else {
		pr_err("%s: invaild condition %d\n", __func__, cond);
		return -EINVAL;
	}

	return 0;
}

static int utpa_proc_show(struct seq_file *m, void *v)
{
	dump_cond_info(m);
	return 0;
}

static int utpa2k_open(struct inode *inode, struct file *file)
{
	return single_open(file, utpa_proc_show, PDE_DATA(inode));
}

static const struct file_operations utpa2k_proc = {
	.owner		= THIS_MODULE,
	.open		= utpa2k_open,
	.read		= seq_read,
	.release	= single_release,
};

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
static struct dev_pm_ops utpa2k_pm_ops;
static struct str_waitfor_dev waitfor;
#endif

static int utpa2k_suspend(struct device *dev)
{
	int i;
	struct utpa2k_str_node *node;
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	static enum stage_nr current_stage;
#endif
	current_mode = UTOPIA2K_STR_POWER_SUSPEND;
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	current_stage = STR_STAGE_1;

	if (waitfor.stage1_s_wait)
        	wait_for_completion(&(waitfor.stage1_s_wait->power.completion));
#endif
	/*
	 * if previous resume abort from free-run mode,
	 * should wait for utpa resume thread done
	*/
	if (str_fr_enabled)
		wait_event_interruptible_timeout(str_fr_waitqueue,
			atomic_read(&str_fr_utpa_resume_done) == 1,
			msecs_to_jiffies(5000));

	update_timeout_threshold();
	reinit_all_complete_barrier();

//	dump_cond_info(NULL);

	pr_info("--------- %s Start ---------\n", __func__);

	for (i = 0; i < total_cb; ++i) {
		node = &cb_pool[i];

		if (!node->fpSTR)
			continue;

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
		if (node->stage != current_stage)
			continue;
#endif
		node->mode = UTOPIA2K_STR_POWER_SUSPEND;
		utpa_pr_dbg("Put %s (%d/%d) into workqueue\n",
				node->name, (i + 1), total_cb);
		async_schedule_domain(utpa2k_async_sched, node,
					&condition_domain);
        }
	async_synchronize_full_domain(&condition_domain);
	atomic_set(&str_fr_utpa_resume_done, 0);

	pr_info("--------- %s End ---------\n", __func__);

	return 0;
}

/* The STR condition is not available here */
static int utpa2k_suspend_no_condition(struct device *dev)
{
	int i;
	struct utpa2k_str_node *node;
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	static enum stage_nr current_stage;
#endif
	current_mode = UTOPIA2K_STR_POWER_SUSPEND;
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	current_stage = STR_STAGE_2;

	if (waitfor.stage2_s_wait)
        	wait_for_completion(&(waitfor.stage2_s_wait->power.completion));
#endif

	pr_info("--------- %s Start ---------\n", __func__);

	for (i = 0; i < total_cb; ++i) {
		node = &cb_pool[i];

		if (!node->fpSTR)
			continue;

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
		if (node->stage != current_stage)
			continue;
#endif
		node->mode = UTOPIA2K_STR_POWER_SUSPEND;
		utpa_pr_dbg("Put %s (%d/%d) into workqueue\n",
				node->name, (i + 1), total_cb);
		async_schedule_domain(utpa2k_async_sched,
				node, &condition_domain);
        }
	async_synchronize_full_domain(&condition_domain);

	pr_info("--------- %s End ---------\n", __func__);

	return 0;
}

static int utpa2k_resume(struct device *dev)
{
	int i;
	struct utpa2k_str_node *node;
	struct task_struct *t;
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	static enum stage_nr current_stage;
#endif
	current_mode = UTOPIA2K_STR_POWER_RESUME;
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	current_stage = STR_STAGE_1;

	if (waitfor.stage1_r_wait)
        	wait_for_completion(&(waitfor.stage1_r_wait->power.completion));
#endif
	reinit_all_complete_barrier();

	pr_info("--------- %s (%s mode) Start ---------\n", __func__,
			str_fr_enabled ? "free-run" : "normal");

	for (i = 0; i < total_cb; ++i) {
		node = &cb_pool[i];

		if (!node->fpSTR)
			continue;

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
		if (node->stage != current_stage)
			continue;
#endif
		node->mode = UTOPIA2K_STR_POWER_RESUME;
		utpa_pr_dbg("Put %s(%d/%d) into workqueue\n",
				node->name, (i + 1), total_cb);
		async_schedule_domain(utpa2k_async_sched,
				node, &condition_domain);
        }

	if (str_fr_enabled) {
		t =  kthread_run(utpa2k_post_utpacore, NULL,
				"utpa2k_post_utpacore");
		if (IS_ERR(t)) {
			pr_err("Create utpa2k_post_mi_thread faied!\n");
			return PTR_ERR(t);
		}
        } else {
		async_synchronize_full_domain(&condition_domain);
        }

	pr_info("--------- %s (%s mode) End ---------\n", __func__,
				str_fr_enabled ? "free-run" : "normal");

	return 0;
}

/* The STR condition is not available here */
static int utpa2k_resume_no_condition(struct device *dev)
{
	int i;
	struct utpa2k_str_node *node;
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	static enum stage_nr current_stage;
#endif
	current_mode = UTOPIA2K_STR_POWER_RESUME;
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	current_stage = STR_STAGE_2;

	if (waitfor.stage2_r_wait)
        	wait_for_completion(&(waitfor.stage2_r_wait->power.completion));
#endif

	pr_info("--------- %s Start ---------\n", __func__);

	for (i = 0; i < total_cb; ++i) {
		node = &cb_pool[i];

		if (!node->fpSTR)
			continue;

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
		if (node->stage != current_stage)
			continue;
#endif
		node->mode = UTOPIA2K_STR_POWER_RESUME;
		utpa_pr_dbg("Put %s(%d/%d) into workqueue\n",
				node->name, (i + 1), total_cb);
		async_schedule_domain(utpa2k_async_sched,
				node, &condition_domain);
        }
	// stage 2 is not support freerun
	async_synchronize_full_domain(&condition_domain);

	pr_info("--------- %s End ---------\n", __func__);
	return 0;
}

static ssize_t pm_utpa2k_freerun_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", str_fr_enabled ? 1 : 0);
}

static ssize_t pm_utpa2k_freerun_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 2)
		return -EINVAL;

	str_fr_enabled = val ? true : false;
	return n;
}

static ssize_t pm_utpa2k_timeout_panic_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", timeout_as_panic ? 1 : 0);
}

static ssize_t pm_utpa2k_timeout_panic_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 2)
		return -EINVAL;

	timeout_as_panic = val ? true : false;
	return n;
}

static ssize_t pm_utpa2k_timeout_showstack_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", timeout_show_stack ? 1 : 0);
}

static ssize_t pm_utpa2k_timeout_showstack_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 2)
		return -EINVAL;

	timeout_show_stack = val ? true : false;
	return n;
}

static ssize_t pm_utpa2k_watchdog_dis_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", wd_disabled ? 1 : 0);
}

static ssize_t pm_utpa2k_watchdog_dis_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 2)
		return -EINVAL;

	wd_disabled = val ? true : false;
	return n;
}

static ssize_t pm_utpa2k_debug_message_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", debug_enabled ? 1 : 0);
}

static ssize_t pm_utpa2k_debug_message_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 1)
		return -EINVAL;

	debug_enabled = val ? true : false;
	return n;
}

static ssize_t pm_utpa2k_timeout_ms_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", utpa2k_timeout_ms);
}

static ssize_t pm_utpa2k_timeout_ms_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 8000)
		return -EINVAL;

	if (val < 50)
		return -EINVAL;

	utpa2k_timeout_ms = val;
	/*
	 * disable timeout auto update
	 * if someone modify timeout threshold in runtime
	*/
	timeout_auto_update = false;
	return n;
}

static ssize_t pm_utpa2k_timeout_auto_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", timeout_auto_update ? 1 : 0);
}

static ssize_t pm_utpa2k_timeout_auto_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 1)
		return -EINVAL;

	timeout_auto_update = val ? true : false;
	return n;
}

static ssize_t pm_utpa2k_rtcond_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "-----------------------------------------------\n");
	s += sprintf(s, "| ** Runtime Utopia2k STR Condition Modify ** |\n");
	s += sprintf(s, "-----------------------------------------------\n");
	s += sprintf(s, "Usage:\n");
	s += sprintf(s, "echo \"[targ-mod] [suspend(0)] [add   (0)] "
			"[pre (0)] [wait-mod]\" > /sys/utpa2k_power/rtcond\n");
	s += sprintf(s, "                 [resume (1)] [remove(1)] [post(1)]\n");

	/* Convert the last space to a newline if needed. */
	if (s != buf)
		*(s-1) = '\n';

	return (s - buf);
}

/*
 * Usage :
 *	echo "[Module] [suspend-0/resume-1] [add-0/rm-1]
 *		[pre-0/post-1] [wait-module]" > /sys
*/
static ssize_t pm_utpa2k_rtcond_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	int ret;
	char targ[16];
	char wait[16];
	int mode;	// suspend/resume
	int opt;	// add/remove
	int cond;	// pre/post
	struct utpa2k_str_node *t;
	const char *operate[] = {
		"add",
		"remove",
	};
	const char *condictions[] = {
		"pre",
		"post",
	};

	utpa_pr_dbg("echo command %s\n", buf);

	ret = sscanf(buf, "%s %d %d %d %s",
		targ, &mode, &opt, &cond, wait);
	if (ret != 5) {
		pr_err("%s: parser %s failed!\n", __func__, buf);
		return -EFAULT;
	}

	if (opt < 0 || opt > ARRAY_SIZE(operate) - 1) {
		pr_err("opterate %d is invalid\n", opt);
		return -EINVAL;
	}

	if (cond < 0 || cond > ARRAY_SIZE(condictions) - 1) {
		pr_err("condictions %d is invalid\n", cond);
		return -EINVAL;
	}

	if (mode < 0 || mode > UTPA2K_PM_MAX -1) {
		pr_err("mode %d is invalid\n", mode);
		return -EINVAL;
	}

	t = find_cb_in_pool(targ);
	if (!t)
		return -EFAULT;

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	if (t->stage == STR_STAGE_2) {
		pr_err("%s is registered in stage2, "
		"runtime condition setting is not available\n", t->name);
		return -EINVAL;
	}
#endif
	pr_info("%s %s %s-condition %s to %s\n",
		operate[opt], power_mode[mode], condictions[cond], wait, targ);

	switch (opt) {
	case RUNTIME_ADD:
		ret = runtime_cond_add(t, wait, mode, cond);
		if (ret)
			return -EINVAL;
		break;
	case RUNTIME_REMOVE:
		ret = runtime_cond_rm(t, wait, mode, cond);
		if (ret)
			return -EINVAL;
		break;
	}
	return n;
}

static struct kobj_attribute pm_utpa2k_rtcond_attr =
	__ATTR(rt_condition, 0644,
		pm_utpa2k_rtcond_show,
		pm_utpa2k_rtcond_store);

static struct kobj_attribute pm_utpa2k_timeout_panic_attr =
	__ATTR(timeout_as_panic, 0644,
		pm_utpa2k_timeout_panic_show,
		pm_utpa2k_timeout_panic_store);

static struct kobj_attribute pm_utpa2k_timeout_showstack_attr =
	__ATTR(timeout_showstack, 0644,
		pm_utpa2k_timeout_showstack_show,
		pm_utpa2k_timeout_showstack_store);

static struct kobj_attribute pm_utpa2k_watchdog_dis_attr =
	__ATTR(str_watchdog_disable, 0644,
		pm_utpa2k_watchdog_dis_show,
		pm_utpa2k_watchdog_dis_store);

static struct kobj_attribute pm_utpa2k_debug_message_attr =
	__ATTR(debug_message, 0644,
		pm_utpa2k_debug_message_show,
		pm_utpa2k_debug_message_store);

static struct kobj_attribute pm_utpa2k_freerun_attr =
	__ATTR(freerun, 0644,
		pm_utpa2k_freerun_show,
		pm_utpa2k_freerun_store);

static struct kobj_attribute pm_utpa2k_timeout_ms_attr =
	__ATTR(timeout_ms, 0644,
		pm_utpa2k_timeout_ms_show,
		pm_utpa2k_timeout_ms_store);

static struct kobj_attribute pm_utpa2k_timeout_auto_attr =
	__ATTR(timeout_autoupdate, 0644,
		pm_utpa2k_timeout_auto_show,
		pm_utpa2k_timeout_auto_store);

static struct attribute * g[] = {
	&pm_utpa2k_rtcond_attr.attr,
	&pm_utpa2k_freerun_attr.attr,
	&pm_utpa2k_debug_message_attr.attr,
	&pm_utpa2k_timeout_ms_attr.attr,
	&pm_utpa2k_timeout_panic_attr.attr,
	&pm_utpa2k_watchdog_dis_attr.attr,
	&pm_utpa2k_timeout_showstack_attr.attr,
	&pm_utpa2k_timeout_auto_attr.attr,
	NULL,
};

static const struct attribute_group attr_group = {
	.attrs = g,
};

static int utpa2k_drv_probe(struct platform_device *pdev)
{
	int err;
	struct proc_dir_entry *d;
	struct kobject *utpa2k_kobj;

	cb_pool = kzalloc(sizeof(struct utpa2k_str_node) * NR_STRCB_MAX,
				GFP_KERNEL);
	if (!cb_pool)
		return -ENOMEM;

	data_pool = kzalloc(sizeof(struct utpa2k_str_data) * NR_STRCB_MAX,
				GFP_KERNEL);
	if (!data_pool)
		return -ENOMEM;

	d = proc_create(UTOPIA2K_STR_NAME, S_IRUGO | S_IWUGO,
					NULL, &utpa2k_proc);
	if (!d) {
		pr_err("%s: proc_create failed!\n", __func__);
		return -EIO;
	}

	utpa2k_kobj = kobject_create_and_add("utpa2k_power", NULL);
	if (!utpa2k_kobj)
		return -ENOMEM;

	err = sysfs_create_group(utpa2k_kobj, &attr_group);
	if (err) {
		pr_err("%s: sysfs_create_group failed!\n", __func__);
		return err;
	}

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
	of_mstar_str(UTOPIA2K_STR_NAME, &pdev->dev,
		&utpa2k_pm_ops, &waitfor,
		&utpa2k_suspend, &utpa2k_resume,
		&utpa2k_suspend_no_condition,
		&utpa2k_resume_no_condition);
#endif
	pr_info("%s %s mode is initialization\n", UTOPIA2K_STR_NAME,
			str_fr_enabled ? "free-run" : "normal");

	return 0;
}

static int utpa2k_drv_remove(struct platform_device *pdev)
{
	int i;
	struct cond_member *cm, *n;

	for (i = 0; i < total_cb; ++i) {
		mutex_lock(&cb_pool_mtx);
		list_for_each_entry_safe(cm, n,
			&cb_pool[i].ci[UTPA2K_PM_SUSPEND].cond_head, list) {
			list_del(&cm->list);
			kfree(cm);
		}
		list_for_each_entry_safe(cm, n,
			&cb_pool[i].ci[UTPA2K_PM_RESUME].cond_head, list) {
			list_del(&cm->list);
			kfree(cm);
		}
		mutex_unlock(&cb_pool_mtx);
	}

	for (i = 0; i < NR_STRCB_MAX; ++i) {
		if (data_pool[i].key)
			kfree(data_pool[i].key);
		if (data_pool[i].val)
			kfree(data_pool[i].val);
	}

	kfree(cb_pool);
	kfree(data_pool);

	return 0;
}

#ifndef CONFIG_MP_MSTAR_STR_OF_ORDER
SIMPLE_DEV_PM_OPS(utpa2k_pm_ops, utpa2k_suspend, utpa2k_resume);
#endif

#ifdef CONFIG_OF
static struct of_device_id utpa2k_of_device_ids[] = {
	{.compatible = UTOPIA2K_STR_NAME},
};
#endif

static struct platform_driver utpa2k_drv = {
	.probe      = utpa2k_drv_probe,
	.remove     = utpa2k_drv_remove,
	.driver = {
#ifdef CONFIG_OF
		.of_match_table = utpa2k_of_device_ids,
#endif
		.pm     = &utpa2k_pm_ops,
		.name   = UTOPIA2K_STR_NAME,
		.owner  = THIS_MODULE,
	},
};

int __init utopia2k_str_init_v2(void)
{
	pr_info("utopia2k STR V2 init\n");
	return platform_driver_register(&utpa2k_drv);
}

void __exit utopia2k_str_exit_v2(void)
{
	platform_driver_unregister(&utpa2k_drv);
}

