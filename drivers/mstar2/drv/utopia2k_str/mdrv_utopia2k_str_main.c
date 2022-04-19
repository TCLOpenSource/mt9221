#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include "mdrv_utopia2k_str_io.h"

static struct utopia2k_str_ops {
	int (*init) (void);
	void (*exit) (void);
	int (*setup_func) (void* pModuleTmp,
			FUtopiaSTR fpSTR);
	int (*wait_condition) (const char* name,
			MS_U32 mode, MS_U32 stage);
	int (*send_condition) (const char* name,
			MS_U32 mode, MS_U32 stage);
	int (*set_data) (char *key, char *value);
	int (*get_data) (char *key, char *value);
} *str_ops;

/*
 * Assume 2019-3M projects use utpa STR-v2
*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 118)
static bool use_utpa2k_str_old;
static int __init utopia2k_str_old_enable(char *str)
{
	use_utpa2k_str_old = true;
	return 1;
}
__setup("utopia2k_str_old", utopia2k_str_old_enable);
#else
static bool use_utpa2k_str_old = true;
#endif

int mdrv_utopia2k_str_setup_function_ptr(void* pModuleTmp,
					FUtopiaSTR fpSTR)
{
	return str_ops->setup_func(pModuleTmp, fpSTR);
}
EXPORT_SYMBOL(mdrv_utopia2k_str_setup_function_ptr);

int mdrv_utopia2k_str_wait_condition(const char* name,
					MS_U32 mode, MS_U32 stage)
{
	return str_ops->wait_condition(name, mode, stage);
}
EXPORT_SYMBOL(mdrv_utopia2k_str_wait_condition);

int mdrv_utopia2k_str_send_condition(const char* name,
					MS_U32 mode, MS_U32 stage)
{
	return str_ops->send_condition(name, mode, stage);
}
EXPORT_SYMBOL(mdrv_utopia2k_str_send_condition);

int mdrv_utopia2k_str_set_data(char *key, char *value)
{
	return str_ops->set_data(key, value);
}
EXPORT_SYMBOL(mdrv_utopia2k_str_set_data);

int mdrv_utopia2k_str_get_data(char *key, char *value)
{
	return str_ops->get_data(key, value);
}
EXPORT_SYMBOL(mdrv_utopia2k_str_get_data);

static int __init mstar_utopia2k_str_init(void)
{
	str_ops = kmalloc(sizeof(struct utopia2k_str_ops), GFP_KERNEL);
	if (!str_ops)
		return -ENOMEM;

	if (use_utpa2k_str_old) {
		pr_info("PM: support utopia2k STR\n");
		str_ops->init = utopia2k_str_init;
		str_ops->setup_func = utopia2k_str_setup_function_ptr;
		str_ops->wait_condition = utopia2k_str_wait_condition;
		str_ops->send_condition = utopia2k_str_send_condition;
		str_ops->set_data = utopia2k_str_set_data;
		str_ops->get_data = utopia2k_str_get_data;
		str_ops->exit = utopia2k_str_exit;
	} else {
		pr_info("PM: support utopia2k STR V2\n");
		str_ops->init = utopia2k_str_init_v2;
		str_ops->setup_func = utopia2k_str_setup_function_ptr_v2;
		str_ops->wait_condition = utopia2k_str_wait_condition_v2;
		str_ops->send_condition = utopia2k_str_send_condition_v2;
		str_ops->set_data = utopia2k_str_set_data_v2;
		str_ops->get_data = utopia2k_str_get_data_v2;
		str_ops->exit = utopia2k_str_exit_v2;
	}

	return str_ops->init();
}

static void __exit mstar_utopia2k_str_exit(void)
{
	return str_ops->exit();
}

module_init(mstar_utopia2k_str_init);
module_exit(mstar_utopia2k_str_exit);

MODULE_DESCRIPTION("Mstar utopia2k STR Device Driver");
MODULE_LICENSE("GPL");

