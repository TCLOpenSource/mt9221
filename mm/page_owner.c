// SPDX-License-Identifier: GPL-2.0
#include <linux/debugfs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/bootmem.h>
#include <linux/stacktrace.h>
#include <linux/page_owner.h>
#include <linux/jump_label.h>
#include <linux/migrate.h>
#include <linux/stackdepot.h>
#include <linux/seq_file.h>

#include "internal.h"

/*
 * TODO: teach PAGE_OWNER_STACK_DEPTH (__dump_page_owner and save_stack)
 * to use off stack temporal storage
 */
#define PAGE_OWNER_STACK_DEPTH (16)

struct page_owner {
	unsigned short order;
	short last_migrate_reason;
	gfp_t gfp_mask;
	depot_stack_handle_t handle;
};

static bool page_owner_disabled = true;
DEFINE_STATIC_KEY_FALSE(page_owner_inited);
#ifdef CONFIG_MP_DEBUG_TOOL_MSTAR_PAGE_OWNER
#define ERROR_PAGE_OWNER_BUF_SIZE (4096)
static DEFINE_SPINLOCK(err_page_owner_buf_lock);
static char *err_page_owner_buf = NULL;
#endif

static depot_stack_handle_t dummy_handle;
static depot_stack_handle_t failure_handle;
static depot_stack_handle_t early_handle;

static void init_early_allocated_pages(void);

static int __init early_page_owner_param(char *buf)
{
	if (!buf)
		return -EINVAL;

	if (strcmp(buf, "on") == 0)
		page_owner_disabled = false;

	return 0;
}
early_param("page_owner", early_page_owner_param);

static bool need_page_owner(void)
{
	if (page_owner_disabled)
		return false;

	return true;
}

static __always_inline depot_stack_handle_t create_dummy_stack(void)
{
	unsigned long entries[4];
	struct stack_trace dummy;

	dummy.nr_entries = 0;
	dummy.max_entries = ARRAY_SIZE(entries);
	dummy.entries = &entries[0];
	dummy.skip = 0;

	save_stack_trace(&dummy);
	return depot_save_stack(&dummy, GFP_KERNEL);
}

static noinline void register_dummy_stack(void)
{
	dummy_handle = create_dummy_stack();
}

static noinline void register_failure_stack(void)
{
	failure_handle = create_dummy_stack();
}

static noinline void register_early_stack(void)
{
	early_handle = create_dummy_stack();
}

static void init_page_owner(void)
{
	if (page_owner_disabled)
		return;

	register_dummy_stack();
	register_failure_stack();
	register_early_stack();
	static_branch_enable(&page_owner_inited);
	init_early_allocated_pages();
}

struct page_ext_operations page_owner_ops = {
	.size = sizeof(struct page_owner),
	.need = need_page_owner,
	.init = init_page_owner,
};

static inline struct page_owner *get_page_owner(struct page_ext *page_ext)
{
	return (void *)page_ext + page_owner_ops.offset;
}

void __reset_page_owner(struct page *page, unsigned int order)
{
	int i;
	struct page_ext *page_ext;

	for (i = 0; i < (1 << order); i++) {
		page_ext = lookup_page_ext(page + i);
		if (unlikely(!page_ext))
			continue;
		__clear_bit(PAGE_EXT_OWNER, &page_ext->flags);
	}
}

static inline bool check_recursive_alloc(struct stack_trace *trace,
					unsigned long ip)
{
	int i;

	if (!trace->nr_entries)
		return false;

	for (i = 0; i < trace->nr_entries; i++) {
		if (trace->entries[i] == ip)
			return true;
	}

	return false;
}

static noinline depot_stack_handle_t save_stack(gfp_t flags)
{
	unsigned long entries[PAGE_OWNER_STACK_DEPTH];
	struct stack_trace trace = {
		.nr_entries = 0,
		.entries = entries,
		.max_entries = PAGE_OWNER_STACK_DEPTH,
		.skip = 2
	};
	depot_stack_handle_t handle;

	save_stack_trace(&trace);
	if (trace.nr_entries != 0 &&
	    trace.entries[trace.nr_entries-1] == ULONG_MAX)
		trace.nr_entries--;

	/*
	 * We need to check recursion here because our request to stackdepot
	 * could trigger memory allocation to save new entry. New memory
	 * allocation would reach here and call depot_save_stack() again
	 * if we don't catch it. There is still not enough memory in stackdepot
	 * so it would try to allocate memory again and loop forever.
	 */
	if (check_recursive_alloc(&trace, _RET_IP_))
		return dummy_handle;

	handle = depot_save_stack(&trace, flags);
	if (!handle)
		handle = failure_handle;

	return handle;
}

static inline void __set_page_owner_handle(struct page_ext *page_ext,
	depot_stack_handle_t handle, unsigned int order, gfp_t gfp_mask)
{
	struct page_owner *page_owner;

	page_owner = get_page_owner(page_ext);
	page_owner->handle = handle;
	page_owner->order = order;
	page_owner->gfp_mask = gfp_mask;
	page_owner->last_migrate_reason = -1;

	__set_bit(PAGE_EXT_OWNER, &page_ext->flags);
}

noinline void __set_page_owner(struct page *page, unsigned int order,
					gfp_t gfp_mask)
{
	struct page_ext *page_ext = lookup_page_ext(page);
	depot_stack_handle_t handle;

	if (unlikely(!page_ext))
		return;

	handle = save_stack(gfp_mask);
	__set_page_owner_handle(page_ext, handle, order, gfp_mask);
}

void __set_page_owner_migrate_reason(struct page *page, int reason)
{
	struct page_ext *page_ext = lookup_page_ext(page);
	struct page_owner *page_owner;

	if (unlikely(!page_ext))
		return;

	page_owner = get_page_owner(page_ext);
	page_owner->last_migrate_reason = reason;
}

void __split_page_owner(struct page *page, unsigned int order)
{
	int i;
	struct page_ext *page_ext = lookup_page_ext(page);
	struct page_owner *page_owner;

	if (unlikely(!page_ext))
		return;

	page_owner = get_page_owner(page_ext);
	page_owner->order = 0;
	for (i = 1; i < (1 << order); i++)
		__copy_page_owner(page, page + i);
}

void __copy_page_owner(struct page *oldpage, struct page *newpage)
{
	struct page_ext *old_ext = lookup_page_ext(oldpage);
	struct page_ext *new_ext = lookup_page_ext(newpage);
	struct page_owner *old_page_owner, *new_page_owner;

	if (unlikely(!old_ext || !new_ext))
		return;

	old_page_owner = get_page_owner(old_ext);
	new_page_owner = get_page_owner(new_ext);
	new_page_owner->order = old_page_owner->order;
	new_page_owner->gfp_mask = old_page_owner->gfp_mask;
	new_page_owner->last_migrate_reason =
		old_page_owner->last_migrate_reason;
	new_page_owner->handle = old_page_owner->handle;

	/*
	 * We don't clear the bit on the oldpage as it's going to be freed
	 * after migration. Until then, the info can be useful in case of
	 * a bug, and the overal stats will be off a bit only temporarily.
	 * Also, migrate_misplaced_transhuge_page() can still fail the
	 * migration and then we want the oldpage to retain the info. But
	 * in that case we also don't need to explicitly clear the info from
	 * the new page, which will be freed.
	 */
	__set_bit(PAGE_EXT_OWNER, &new_ext->flags);
}

void pagetypeinfo_showmixedcount_print(struct seq_file *m,
				       pg_data_t *pgdat, struct zone *zone)
{
	struct page *page;
	struct page_ext *page_ext;
	struct page_owner *page_owner;
	unsigned long pfn = zone->zone_start_pfn, block_end_pfn;
	unsigned long end_pfn = pfn + zone->spanned_pages;
	unsigned long count[MIGRATE_TYPES] = { 0, };
	int pageblock_mt, page_mt;
	int i;

	/* Scan block by block. First and last block may be incomplete */
	pfn = zone->zone_start_pfn;

	/*
	 * Walk the zone in pageblock_nr_pages steps. If a page block spans
	 * a zone boundary, it will be double counted between zones. This does
	 * not matter as the mixed block count will still be correct
	 */
	for (; pfn < end_pfn; ) {
		page = pfn_to_online_page(pfn);
		if (!page) {
			pfn = ALIGN(pfn + 1, MAX_ORDER_NR_PAGES);
			continue;
		}

		block_end_pfn = ALIGN(pfn + 1, pageblock_nr_pages);
		block_end_pfn = min(block_end_pfn, end_pfn);

		pageblock_mt = get_pageblock_migratetype(page);

		for (; pfn < block_end_pfn; pfn++) {
			if (!pfn_valid(pfn))
				continue;

			/* The pageblock is online, no need to recheck. */
			page = pfn_to_page(pfn);

			if (page_zone(page) != zone)
				continue;

			if (PageBuddy(page)) {
				unsigned long freepage_order;

				freepage_order = page_order_unsafe(page);
				if (freepage_order < MAX_ORDER)
					pfn += (1UL << freepage_order) - 1;
				continue;
			}

			if (PageReserved(page))
				continue;

			page_ext = lookup_page_ext(page);
			if (unlikely(!page_ext))
				continue;

			if (!test_bit(PAGE_EXT_OWNER, &page_ext->flags))
				continue;

			page_owner = get_page_owner(page_ext);
			page_mt = gfpflags_to_migratetype(
					page_owner->gfp_mask);
			if (pageblock_mt != page_mt) {
				if (is_migrate_cma(pageblock_mt))
					count[MIGRATE_MOVABLE]++;
				else
					count[pageblock_mt]++;

				pfn = block_end_pfn;
				break;
			}
			pfn += (1UL << page_owner->order) - 1;
		}
	}

	/* Print counts */
	seq_printf(m, "Node %d, zone %8s ", pgdat->node_id, zone->name);
	for (i = 0; i < MIGRATE_TYPES; i++)
		seq_printf(m, "%12lu ", count[i]);
	seq_putc(m, '\n');
}

static ssize_t
print_page_owner(char __user *buf, size_t count, unsigned long pfn,
		struct page *page, struct page_owner *page_owner,
		depot_stack_handle_t handle)
{
	int ret;
	int pageblock_mt, page_mt;
	char *kbuf;
	unsigned long entries[PAGE_OWNER_STACK_DEPTH];
	struct stack_trace trace = {
		.nr_entries = 0,
		.entries = entries,
		.max_entries = PAGE_OWNER_STACK_DEPTH,
		.skip = 0
	};

	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	ret = snprintf(kbuf, count,
			"Page allocated via order %u, mask %#x(%pGg)\n",
			page_owner->order, page_owner->gfp_mask,
			&page_owner->gfp_mask);

	if (ret >= count)
		goto err;

	/* Print information relevant to grouping pages by mobility */
	pageblock_mt = get_pageblock_migratetype(page);
	page_mt  = gfpflags_to_migratetype(page_owner->gfp_mask);
	ret += snprintf(kbuf + ret, count - ret,
			"PFN %lu type %s Block %lu type %s Flags %#lx(%pGp)\n",
			pfn,
			migratetype_names[page_mt],
			pfn >> pageblock_order,
			migratetype_names[pageblock_mt],
			page->flags, &page->flags);

	if (ret >= count)
		goto err;

	depot_fetch_stack(handle, &trace);
	ret += snprint_stack_trace(kbuf + ret, count - ret, &trace, 0);
	if (ret >= count)
		goto err;

	if (page_owner->last_migrate_reason != -1) {
		ret += snprintf(kbuf + ret, count - ret,
			"Page has been migrated, last migrate reason: %s\n",
			migrate_reason_names[page_owner->last_migrate_reason]);
		if (ret >= count)
			goto err;
	}

	ret += snprintf(kbuf + ret, count - ret, "\n");
	if (ret >= count)
		goto err;

	if (copy_to_user(buf, kbuf, ret))
		ret = -EFAULT;

	kfree(kbuf);
	return ret;

err:
	kfree(kbuf);
	return -ENOMEM;
}

void __dump_page_owner(struct page *page)
{
	struct page_ext *page_ext = lookup_page_ext(page);
	struct page_owner *page_owner;
	unsigned long entries[PAGE_OWNER_STACK_DEPTH];
	struct stack_trace trace = {
		.nr_entries = 0,
		.entries = entries,
		.max_entries = PAGE_OWNER_STACK_DEPTH,
		.skip = 0
	};
	depot_stack_handle_t handle;
	gfp_t gfp_mask;
	int mt;

	if (unlikely(!page_ext)) {
		pr_alert("There is not page extension available.\n");
		return;
	}

	page_owner = get_page_owner(page_ext);
	gfp_mask = page_owner->gfp_mask;
	mt = gfpflags_to_migratetype(gfp_mask);

	if (!test_bit(PAGE_EXT_OWNER, &page_ext->flags)) {
		pr_alert("page_owner info is not active (free page?)\n");
		return;
	}

	handle = READ_ONCE(page_owner->handle);
	if (!handle) {
		pr_alert("page_owner info is not active (free page?)\n");
		return;
	}

	depot_fetch_stack(handle, &trace);
	pr_alert("page allocated via order %u, migratetype %s, gfp_mask %#x(%pGg)\n",
		 page_owner->order, migratetype_names[mt], gfp_mask, &gfp_mask);
	print_stack_trace(&trace, 0);

	if (page_owner->last_migrate_reason != -1)
		pr_alert("page has been migrated, last migrate reason: %s\n",
			migrate_reason_names[page_owner->last_migrate_reason]);
}

#ifdef CONFIG_MP_DEBUG_TOOL_MSTAR_PAGE_OWNER
static ssize_t
print_page_owner_v2(char __user *buf, size_t count, unsigned long pfn,
		struct page *page, struct page_owner *page_owner,
		depot_stack_handle_t handle)
{
	int ret;
	int pageblock_mt, page_mt;
	char *kbuf = buf;
	unsigned long entries[PAGE_OWNER_STACK_DEPTH];
	struct stack_trace trace = {
		.nr_entries = 0,
		.entries = entries,
		.max_entries = PAGE_OWNER_STACK_DEPTH,
		.skip = 0
	};

	ret = snprintf(kbuf, count,
			"Page allocated via order %u, mask %#x(%pGg)\n",
			page_owner->order, page_owner->gfp_mask,
			&page_owner->gfp_mask);

	if (ret >= count)
		goto err;

	/* Print information relevant to grouping pages by mobility */
	pageblock_mt = get_pageblock_migratetype(page);
	page_mt  = gfpflags_to_migratetype(page_owner->gfp_mask);
	ret += snprintf(kbuf + ret, count - ret,
			"PFN %lu type %s Block %lu type %s Flags %#lx(%pGp)\n",
			pfn,
			migratetype_names[page_mt],
			pfn >> pageblock_order,
			migratetype_names[pageblock_mt],
			page->flags, &page->flags);

	if (ret >= count)
		goto err;

	depot_fetch_stack(handle, &trace);
	ret += snprint_stack_trace(kbuf + ret, count - ret, &trace, 0);
	if (ret >= count)
		goto err;

	if (page_owner->last_migrate_reason != -1) {
		ret += snprintf(kbuf + ret, count - ret,
				"Page has been migrated, last migrate reason: %s\n",
				migrate_reason_names[page_owner->last_migrate_reason]);
		if (ret >= count)
			goto err;
	}

	ret += snprintf(kbuf + ret, count - ret, "\n");
	if (ret >= count)
		goto err;

	return ret;

err:
	return -ENOMEM;
}

/* This is used to print the page_owner info of a selected pfn.
   While doing read_page_owner() to print the page_owner of all allcated pages,
   we call print_error_page_trace() to print 1st pfn for allocated pages.
   By doing that, we can verify the print_error_page_trace() works correctly.
   */
void print_error_page_trace(unsigned long pfn)
{
	struct page *page;
	struct page_ext *page_ext;
	struct page_owner *page_owner;
	depot_stack_handle_t handle;
	char *buf = err_page_owner_buf;
	unsigned long irq_flags;

	if (!static_branch_unlikely(&page_owner_inited))
	{
		printk(KERN_EMERG "\033[35mFunction = %s, Line = %d\033[m\n", __PRETTY_FUNCTION__, __LINE__);
		return -EINVAL;
	}

	if (!buf)
	{
		printk(KERN_EMERG "\033[35mFunction = %s, Line = %d, buf alloc failed\033[m\n", __PRETTY_FUNCTION__, __LINE__);
		return -ENOMEM;
	}

	page = pfn_to_page(pfn);
	if (PageBuddy(page)) {
		printk(KERN_EMERG "\033[31mFunction = %s, Line = %d, this pfn 0x%lX is @ PageBuddy\033[m\n", __PRETTY_FUNCTION__, __LINE__, pfn);
		return;
	}

	page_ext = lookup_page_ext(page);
	if (unlikely(!page_ext))
	{
		printk(KERN_EMERG "\033[31mFunction = %s, Line = %d, this pfn 0x%lX has no page_ext\033[m\n", __PRETTY_FUNCTION__, __LINE__, pfn);
		return;
	}

	/*
	 * Some pages could be missed by concurrent allocation or free,
	 * because we don't hold the zone lock.
	 */
	if (!test_bit(PAGE_EXT_OWNER, &page_ext->flags))
	{
		printk(KERN_EMERG "\033[31mFunction = %s, Line = %d, this pfn 0x%lX does not have PAGE_EXT_OWNER\033[m\n", __PRETTY_FUNCTION__, __LINE__, pfn);
		return;
	}

	page_owner = get_page_owner(page_ext);

	/*
	 * Access to page_ext->handle isn't synchronous so we should
	 * be careful to access it.
	 */
	handle = READ_ONCE(page_owner->handle);
	if (!handle)
	{
		printk(KERN_EMERG "\033[31mFunction = %s, Line = %d, this pfn 0x%lX does not have page_owner->handle\033[m\n", __PRETTY_FUNCTION__, __LINE__, pfn);
		return;
	}

	spin_lock_irqsave(&err_page_owner_buf_lock,irq_flags);
	print_page_owner_v2(buf, count, pfn, page,	page_owner, handle);
	spin_unlock_irqrestore(&err_page_owner_buf_lock,irq_flags);

	printk(KERN_EMERG "\033[35mFunction = %s, Line = %d, \n\n\n%s\033[m\n\n\n", __PRETTY_FUNCTION__, __LINE__, buf);
	kfree(buf);
}

static volatile int test_print_pfn_owner = 0;
#endif

static ssize_t
read_page_owner(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	unsigned long pfn;
	struct page *page;
	struct page_ext *page_ext;
	struct page_owner *page_owner;
	depot_stack_handle_t handle;

	if (!static_branch_unlikely(&page_owner_inited))
		return -EINVAL;

	page = NULL;
	pfn = min_low_pfn + *ppos;

	/* Find a valid PFN or the start of a MAX_ORDER_NR_PAGES area */
	while (!pfn_valid(pfn) && (pfn & (MAX_ORDER_NR_PAGES - 1)) != 0)
		pfn++;

	drain_all_pages(NULL);

	/* Find an allocated page */
	for (; pfn < max_pfn; pfn++) {
		/*
		 * If the new page is in a new MAX_ORDER_NR_PAGES area,
		 * validate the area as existing, skip it if not
		 */
		if ((pfn & (MAX_ORDER_NR_PAGES - 1)) == 0 && !pfn_valid(pfn)) {
			pfn += MAX_ORDER_NR_PAGES - 1;
			continue;
		}

		/* Check for holes within a MAX_ORDER area */
		if (!pfn_valid(pfn))
			continue;

		page = pfn_to_page(pfn);
		if (PageBuddy(page)) {
			unsigned long freepage_order = page_order_unsafe(page);

			if (freepage_order < MAX_ORDER)
				pfn += (1UL << freepage_order) - 1;
			continue;
		}

		page_ext = lookup_page_ext(page);
		if (unlikely(!page_ext))
			continue;

		/*
		 * Some pages could be missed by concurrent allocation or free,
		 * because we don't hold the zone lock.
		 */
		if (!test_bit(PAGE_EXT_OWNER, &page_ext->flags))
			continue;

		page_owner = get_page_owner(page_ext);

		/*
		 * Access to page_ext->handle isn't synchronous so we should
		 * be careful to access it.
		 */
		handle = READ_ONCE(page_owner->handle);
		if (!handle)
			continue;

		/* Record the next PFN to read in the file offset */
		*ppos = (pfn - min_low_pfn) + 1;
#ifdef CONFIG_MP_DEBUG_TOOL_MSTAR_PAGE_OWNER
		if (!test_print_pfn_owner) {
			printk("\033[35mFunction = %s, Line = %d, [Start] print_error_page_trace\033[m\n", __PRETTY_FUNCTION__, __LINE__);
			print_error_page_trace(pfn);
			test_print_pfn_owner = 1;
			printk("\033[35mFunction = %s, Line = %d, [End] print_error_page_trace\033[m\n", __PRETTY_FUNCTION__, __LINE__);
		}
#endif
		return print_page_owner(buf, count, pfn, page,
				page_owner, handle);
	}

	return 0;
}

static void init_pages_in_zone(pg_data_t *pgdat, struct zone *zone)
{
	unsigned long pfn = zone->zone_start_pfn;
	unsigned long end_pfn = zone_end_pfn(zone);
	unsigned long count = 0;

	/*
	 * Walk the zone in pageblock_nr_pages steps. If a page block spans
	 * a zone boundary, it will be double counted between zones. This does
	 * not matter as the mixed block count will still be correct
	 */
	for (; pfn < end_pfn; ) {
		unsigned long block_end_pfn;

		if (!pfn_valid(pfn)) {
			pfn = ALIGN(pfn + 1, MAX_ORDER_NR_PAGES);
			continue;
		}

		block_end_pfn = ALIGN(pfn + 1, pageblock_nr_pages);
		block_end_pfn = min(block_end_pfn, end_pfn);

		for (; pfn < block_end_pfn; pfn++) {
			struct page *page;
			struct page_ext *page_ext;

			if (!pfn_valid(pfn))
				continue;

			page = pfn_to_page(pfn);

			if (page_zone(page) != zone)
				continue;

			/*
			 * To avoid having to grab zone->lock, be a little
			 * careful when reading buddy page order. The only
			 * danger is that we skip too much and potentially miss
			 * some early allocated pages, which is better than
			 * heavy lock contention.
			 */
			if (PageBuddy(page)) {
				unsigned long order = page_order_unsafe(page);

				if (order > 0 && order < MAX_ORDER)
					pfn += (1UL << order) - 1;
				continue;
			}

			if (PageReserved(page))
				continue;

			page_ext = lookup_page_ext(page);
			if (unlikely(!page_ext))
				continue;

			/* Maybe overlapping zone */
			if (test_bit(PAGE_EXT_OWNER, &page_ext->flags))
				continue;

			/* Found early allocated page */
			__set_page_owner_handle(page_ext, early_handle, 0, 0);
			count++;
		}
		cond_resched();
	}

	pr_info("Node %d, zone %8s: page owner found early allocated %lu pages\n",
		pgdat->node_id, zone->name, count);
}

static void init_zones_in_node(pg_data_t *pgdat)
{
	struct zone *zone;
	struct zone *node_zones = pgdat->node_zones;

	for (zone = node_zones; zone - node_zones < MAX_NR_ZONES; ++zone) {
		if (!populated_zone(zone))
			continue;

		init_pages_in_zone(pgdat, zone);
	}
}

static void init_early_allocated_pages(void)
{
	pg_data_t *pgdat;

	for_each_online_pgdat(pgdat)
		init_zones_in_node(pgdat);
}

static const struct file_operations proc_page_owner_operations = {
	.read		= read_page_owner,
};

static int __init pageowner_init(void)
{
	struct dentry *dentry;

	if (!static_branch_unlikely(&page_owner_inited)) {
		pr_info("page_owner is disabled\n");
		return 0;
	}

#ifdef CONFIG_MP_DEBUG_TOOL_MSTAR_PAGE_OWNER
	err_page_owner_buf = kmalloc(ERROR_PAGE_OWNER_BUF_SIZE, GFP_KERNEL);
	if(!err_page_owner_buf)
		pr_err("allloc %x for printing error page owner fails\n",
					ERROR_PAGE_OWNER_BUF_SIZE);
#endif

	dentry = debugfs_create_file("page_owner", 0400, NULL,
				     NULL, &proc_page_owner_operations);

	return PTR_ERR_OR_ZERO(dentry);
}
late_initcall(pageowner_init)
