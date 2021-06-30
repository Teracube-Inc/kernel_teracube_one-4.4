/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
#include <sspm_reservedmem_define.h>
#endif
#include <mtk_dramc.h>
#include <mtk_swpm_common.h>
#include <mtk_swpm_platform.h>
#include <mtk_swpm.h>

/****************************************************************************
 *  Macro Definitions
 ****************************************************************************/
#define DEFAULT_AVG_WINDOW		(50)

#define MAX(a, b)			((a) >= (b) ? (a) : (b))
#define MIN(a, b)			((a) >= (b) ? (b) : (a))

/* PROCFS */
#define PROC_FOPS_RW(name)                                                 \
	static int name ## _proc_open(struct inode *inode,                 \
		struct file *file)                                         \
	{                                                                  \
		return single_open(file, name ## _proc_show,               \
			PDE_DATA(inode));                                  \
	}                                                                  \
	static const struct file_operations name ## _proc_fops = {         \
		.owner      = THIS_MODULE,                                 \
		.open       = name ## _proc_open,                          \
		.read	    = seq_read,                                    \
		.llseek	    = seq_lseek,                                   \
		.release    = single_release,                              \
		.write      = name ## _proc_write,                         \
	}
#define PROC_FOPS_RO(name)                                                 \
	static int name ## _proc_open(struct inode *inode,                 \
		struct file *file)                                         \
	{                                                                  \
		return single_open(file, name ## _proc_show,               \
			PDE_DATA(inode));                                  \
	}                                                                  \
	static const struct file_operations name ## _proc_fops = {         \
		.owner = THIS_MODULE,                                      \
		.open  = name ## _proc_open,                               \
		.read  = seq_read,                                         \
		.llseek = seq_lseek,                                       \
		.release = single_release,                                 \
	}
#define PROC_ENTRY(name)	{__stringify(name), &name ## _proc_fops}

/****************************************************************************
 *  Type Definitions
 ****************************************************************************/

/****************************************************************************
 *  Local Variables
 ****************************************************************************/
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
static phys_addr_t rec_phys_addr, rec_virt_addr;
static unsigned long long rec_size;
#endif
static bool swpm_enable = true;
static unsigned char avg_window = DEFAULT_AVG_WINDOW;

/****************************************************************************
 *  Global Variables
 ****************************************************************************/
struct swpm_rec_data *swpm_info_ref;
bool swpm_debug = true;
DEFINE_MUTEX(swpm_mutex);

/****************************************************************************
 *  Static Function
 ****************************************************************************/
static char *_copy_from_user_for_proc(const char __user *buffer, size_t count)
{
	char *buf = (char *)__get_free_page(GFP_USER);

	if (!buf)
		return NULL;

	if (count >= PAGE_SIZE)
		goto out;

	if (copy_from_user(buf, buffer, count))
		goto out;

	buf[count] = '\0';

	return buf;

out:
	free_page((unsigned long)buf);

	return NULL;
}

static int dump_power_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Avg CPU pwr = %dmA\n",
		swpm_get_avg_power(CPU_POWER_METER, avg_window));
	seq_printf(m, "Avg GPU pwr = %dmA\n",
		swpm_get_avg_power(GPU_POWER_METER, avg_window));
	seq_printf(m, "Avg CORE pwr = %dmA\n",
		swpm_get_avg_power(CORE_POWER_METER, avg_window));
	seq_printf(m, "Avg MEM pwr = %dmA\n",
		swpm_get_avg_power(MEM_POWER_METER, avg_window));

	return 0;
}

static int debug_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "\nSWPM debug is %s\n",
		(swpm_debug == true) ? "enabled" : "disabled");

	return 0;
}

static ssize_t debug_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{
	int enable;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (!kstrtouint(buf, 10, &enable))
		swpm_debug = (enable) ? true : false;
	else
		swpm_err("echo 1/0 > /proc/swpm/debug\n");

	free_page((unsigned long)buf);
	return count;
}

static int profile_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "read EMI time avg/max = %lluus/%lluus, cnt = %llu\n",
		swpm_info_ref->avg_latency[READ_EMI_TIME],
		swpm_info_ref->max_latency[READ_EMI_TIME],
		swpm_info_ref->prof_cnt[READ_EMI_TIME]);
	seq_printf(m, "monitor time avg/max = %lluus/%lluus, cnt = %llu\n",
		swpm_info_ref->avg_latency[MON_TIME],
		swpm_info_ref->max_latency[MON_TIME],
		swpm_info_ref->prof_cnt[MON_TIME]);
	seq_printf(m, "calculate time avg/max = %lluus/%lluus, cnt = %llu\n",
		swpm_info_ref->avg_latency[CALC_TIME],
		swpm_info_ref->max_latency[CALC_TIME],
		swpm_info_ref->prof_cnt[CALC_TIME]);
	seq_printf(m, "proc record time avg/max = %lluus/%lluus, cnt = %llu\n",
		swpm_info_ref->avg_latency[REC_TIME],
		swpm_info_ref->max_latency[REC_TIME],
		swpm_info_ref->prof_cnt[REC_TIME]);
	seq_printf(m, "total time avg/max = %lluus/%lluus, cnt = %llu\n",
		swpm_info_ref->avg_latency[TOTAL_TIME],
		swpm_info_ref->max_latency[TOTAL_TIME],
		swpm_info_ref->prof_cnt[TOTAL_TIME]);

	seq_printf(m, "\nSWPM profile is %s\n",
		(swpm_info_ref->profile_enable) ? "enabled" : "disabled");

	return 0;
}

static ssize_t profile_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{
	int enable;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (!kstrtouint(buf, 10, &enable))
		swpm_info_ref->profile_enable = enable;
	else
		swpm_err("echo <1/0> > /proc/swpm/profile\n");

	free_page((unsigned long)buf);
	return count;
}

static int avg_window_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Current Avg Window is %d\n",
		MIN(MAX_RECORD_CNT, avg_window));

	return 0;
}

static ssize_t avg_window_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int window;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (!kstrtouint(buf, 10, &window))
		avg_window = MIN(MAX_RECORD_CNT, window);
	else
		swpm_err("echo <window> > /proc/swpm/avg_window\n");

	free_page((unsigned long)buf);
	return count;
}

PROC_FOPS_RO(dump_power);
PROC_FOPS_RW(debug);
PROC_FOPS_RW(profile);
PROC_FOPS_RW(avg_window);

static int create_procfs(void)
{
	struct proc_dir_entry *swpm_dir = NULL;
	int i = 0;

	struct pentry {
		const char *name;
		const struct file_operations *fops;
	};

	struct pentry swpm_entries[] = {
		PROC_ENTRY(dump_power),
		PROC_ENTRY(debug),
		PROC_ENTRY(profile),
		PROC_ENTRY(avg_window),
	};

	swpm_dir = proc_mkdir("swpm", NULL);
	if (!swpm_dir) {
		swpm_err("[%s] mkdir /proc/swpm failed\n", __func__);
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(swpm_entries); i++) {
		if (!proc_create(swpm_entries[i].name,
			S_IRUGO | S_IWUSR | S_IWGRP,
			swpm_dir, swpm_entries[i].fops)) {
			swpm_err("[%s]: create /proc/swpm/%s failed\n",
				__func__, swpm_entries[i].name);
			return -1;
		}
	}

	return 0;
}

static void get_rec_addr(void)
{
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	int i;
	unsigned char *ptr;

	/* get sspm reserved mem */
	rec_phys_addr = sspm_reserve_mem_get_phys(SWPM_MEM_ID);
	rec_virt_addr = sspm_reserve_mem_get_virt(SWPM_MEM_ID);
	rec_size = sspm_reserve_mem_get_size(SWPM_MEM_ID);

	swpm_info("phy_addr = 0x%llx, virt_addr=0x%llx, size = %llu\n",
		(unsigned long long)rec_phys_addr,
		(unsigned long long)rec_virt_addr,
		rec_size);

	/* clear */
	ptr = (unsigned char *)(uintptr_t)rec_virt_addr;
	for (i = 0; i < rec_size; i++)
		ptr[i] = 0x0;

	swpm_info_ref = (struct swpm_rec_data *)(uintptr_t)rec_virt_addr;
#endif
}

static int __init swpm_init(void)
{
	if (swpm_enable == false) {
		swpm_err("swpm is disabled\n");
		return 0;
	}

	get_rec_addr();
	if (!swpm_info_ref) {
		swpm_err("get sspm dram addr failed\n");
		return 0;
	}
	create_procfs();

	swpm_platform_init();

#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
#ifdef CONFIG_MTK_DRAMC
	swpm_send_init_ipi((unsigned int)(rec_phys_addr & 0xFFFFFFFF),
		(unsigned int)(rec_size & 0xFFFFFFFF), get_emi_ch_num());
#else
	swpm_send_init_ipi((unsigned int)(rec_phys_addr & 0xFFFFFFFF),
		(unsigned int)(rec_size & 0xFFFFFFFF), 2);
#endif
#endif

	swpm_info("SWPM init done!\n");

	return 0;
}
late_initcall(swpm_init);

/***************************************************************************
 *  API
 ***************************************************************************/
unsigned int swpm_get_avg_power(unsigned int type, unsigned int avg_window)
{
	unsigned int *ptr;
	unsigned int cnt, idx, sum = 0, pwr = 0;

	if (type >= NR_POWER_METER) {
		swpm_err("Invalid SWPM type = %d\n", type);
		return 0;
	}

	/* window should be 1 to MAX_RECORD_CNT */
	avg_window = MAX(avg_window, 1);
	avg_window = MIN(avg_window, MAX_RECORD_CNT);

	/* get ptr of the target meter record */
	ptr = &swpm_info_ref->pwr[type][0];

	/* calculate avg */
	for (idx = swpm_info_ref->cur_idx, cnt = 0; cnt < avg_window; cnt++) {
		sum += ptr[idx];

		if (!idx)
			idx = MAX_RECORD_CNT - 1;
		else
			idx--;
	}

	pwr = sum / avg_window;

	swpm_dbg("avg pwr of meter %d = %d mA\n", type, pwr);

	return pwr;
}

