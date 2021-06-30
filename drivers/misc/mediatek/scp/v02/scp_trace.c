/*
 * Copyright (C) 2011-2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/fs.h>           /* needed by file_operations* */
#include <linux/miscdevice.h>   /* needed by miscdevice* */
#include <linux/sysfs.h>
#include <linux/device.h>       /* needed by device_* */
#include <linux/fs.h>           /* needed by file_operations* */
#include <mt-plat/sync_write.h>
#include <scp_helper.h>
#include <scp_ipi.h>
#include <linux/delay.h>
#include "scp_helper.h"
#include "scp_excep.h"

MODULE_LICENSE("GPL");

#define TRACE_BUFFER (0x4000)
typedef struct {
	uint32_t MARKER1;
	uint32_t MARKER2;
	uint32_t trace_data_start;
	uint32_t trace_data_end;
	uint32_t cpu_id;
	uint32_t configuration;
} trace_info_t;

static trace_info_t trace_info[SCP_CORE_TOTAL];

unsigned int scp_trace_run_flag;
unsigned int scp_trace_run_command;
unsigned int trace_data_selected_id;
unsigned int trace_r_pos[SCP_CORE_TOTAL] = {0, 0};
unsigned int sram_offset[SCP_CORE_TOTAL] = {0, 0};
static void scp_trace_handler(int id, void *data, unsigned int len)
{
	trace_info_t *tmp = (trace_info_t *)data;

	memcpy((void *)&trace_info[tmp->cpu_id], data, sizeof(trace_info_t));
}

static ssize_t scp_trace_run_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int sz;

	mutex_lock(&dev->mutex);
	sz = snprintf(buf, PAGE_SIZE, "%d\n", scp_trace_run_flag);
	mutex_unlock(&dev->mutex);
	return sz;
}

#define TRACE_OP_START        0x00000001
#define TRACE_OP_STOP         0x00000002
#define TRACE_OP_EXTRACT      0x00000003
#define FUNC_BIT_SHIF       18
#define TRACE_OP            (1 << FUNC_BIT_SHIF)

static ssize_t scp_trace_run_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int prev_run_state;
	unsigned int value;
	int ret;

	mutex_lock(&dev->mutex);

	prev_run_state = scp_trace_run_flag;

	if (kstrtoint(buf, 10, &scp_trace_run_flag) != 0)
		return -EINVAL;

	pr_err("[scp_trace] status:%d\n", scp_trace_run_flag);
	if (prev_run_state == 0 && scp_trace_run_flag == 1) {
		value = TRACE_OP|TRACE_OP_START;
		if (scp_trace_run_command & (1<<SCP_A_ID))
			ret = scp_ipi_send(IPI_TRACE_SCP, &value, sizeof(value), 0, SCP_A_ID);
		udelay(1000);
		if (scp_trace_run_command & (1<<SCP_B_ID))
			ret = scp_ipi_send(IPI_TRACE_SCP, &value, sizeof(value), 0, SCP_B_ID);
		trace_r_pos[SCP_A_ID] = 0;
		trace_r_pos[SCP_B_ID] = 0;
		sram_offset[SCP_A_ID] = 0;
		sram_offset[SCP_B_ID] = SCP_A_TCM_SIZE;
	} else if (prev_run_state == 1 && scp_trace_run_flag == 0) {
		if (scp_trace_run_command & (1<<SCP_A_ID)) {
			value = TRACE_OP|TRACE_OP_STOP;
			ret = scp_ipi_send(IPI_TRACE_SCP, &value, sizeof(value), 0, SCP_A_ID);
			udelay(1000);
			value = TRACE_OP|TRACE_OP_EXTRACT;
			ret = scp_ipi_send(IPI_TRACE_SCP, &value, sizeof(value), 0, SCP_A_ID);
		}
		udelay(1000);
		if (scp_trace_run_command & (1<<SCP_B_ID)) {
			value = TRACE_OP|TRACE_OP_STOP;
			ret = scp_ipi_send(IPI_TRACE_SCP, &value, sizeof(value), 0, SCP_B_ID);
			udelay(1000);
			value = TRACE_OP|TRACE_OP_EXTRACT;
			ret = scp_ipi_send(IPI_TRACE_SCP, &value, sizeof(value), 0, SCP_B_ID);
		}


	}

	mutex_unlock(&dev->mutex);

	return count;
}

static DEVICE_ATTR(scp_trace_run, 0660, scp_trace_run_show, scp_trace_run_store);
static ssize_t scp_trace_setup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int sz;

	mutex_lock(&dev->mutex);
	sz = snprintf(buf, PAGE_SIZE, "%d\n", scp_trace_run_command);
	mutex_unlock(&dev->mutex);
	return sz;
}
static ssize_t scp_trace_setup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int prev_run_state;

	mutex_lock(&dev->mutex);

	prev_run_state = scp_trace_run_command;

	if (kstrtoint(buf, 10, &scp_trace_run_command) != 0)
		return -EINVAL;

	pr_err("[scp_trace] command:%d\n", scp_trace_run_command);

	mutex_unlock(&dev->mutex);

	return count;
}
static DEVICE_ATTR(scp_trace_setup, 0660, scp_trace_setup_show, scp_trace_setup_store);

static ssize_t scp_trace_select_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int sz;

	mutex_lock(&dev->mutex);
	sz = snprintf(buf, PAGE_SIZE, "%d\n", trace_data_selected_id);
	mutex_unlock(&dev->mutex);
	return sz;
}
static ssize_t scp_trace_select_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int prev_run_state;

	mutex_lock(&dev->mutex);

	prev_run_state = trace_data_selected_id;

	if (kstrtoint(buf, 10, &trace_data_selected_id) != 0)
		return -EINVAL;

	pr_err("[scp_trace] trace_data_selected_id:%d\n", trace_data_selected_id);

	mutex_unlock(&dev->mutex);

	return count;
}
static DEVICE_ATTR(scp_trace_select, 0660, scp_trace_select_show, scp_trace_select_store);

ssize_t scp_trace_read(char __user *data, size_t len)
{
	char *buf;
	unsigned int *scp_base;
	unsigned int lock;
	unsigned int *reg;


	pr_err("[scp_trace] selected id:%d\n", trace_data_selected_id);
	reg = (unsigned int *) (scpreg.cfg + SCP_LOCK_OFS);
	lock = *reg;
	*reg &= ~SCP_TCM_LOCK_BIT;
	dsb(SY);
	if ((*reg & SCP_TCM_LOCK_BIT)) {
		pr_debug("[SCP]unlock failed, skip dump\n");
		return 0;
	}

	buf = ((char *) SCP_TCM)
		+ trace_info[trace_data_selected_id].trace_data_start
		+ sram_offset[trace_data_selected_id];

	len = trace_info[trace_data_selected_id].trace_data_end
		- trace_info[trace_data_selected_id].trace_data_start
		- trace_r_pos[trace_data_selected_id];

	pr_err("[scp_trace] len:%d\n", (int) len);
	scp_base = (unsigned int *) buf;
	/*SCP keep awake */
	scp_awake_lock(trace_data_selected_id);
	/*memory copy from log buf*/
	memcpy_from_scp(data, scp_base, len);
	/*SCP leaave awake */
	scp_awake_unlock(trace_data_selected_id);

	trace_r_pos[trace_data_selected_id] = TRACE_BUFFER;

	return len;

}

static ssize_t scp_trace_if_read(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
	ssize_t ret;
#if 0
	unsigned int *buff;
	int i;
#endif


	ret = 0;

	/*if (access_ok(VERIFY_WRITE, data, len))*/
	ret = scp_trace_read(data, len);
#if 0
	buff = (unsigned int *)data;
	for (i = 0; i < 0x1000; i++)
		pr_err("0x%x\n", buff[i]);
#endif

	return ret;
}

static ssize_t scp_trace_if_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	ssize_t ret;

	ret = 0;
	if (kstrtoint(data, 10, &trace_data_selected_id) != 0)
		return -EINVAL;

	pr_err("[scp_trace] selected id:%d\n", trace_data_selected_id);

	return ret;
}
static int scp_trace_if_open(struct inode *inode, struct file *file)
{
	/*pr_debug("[SCP A] scp_A_log_if_open\n");*/
	return nonseekable_open(inode, file);
}

const struct file_operations scp_trace_ops = {
	.owner = THIS_MODULE,
	.read  = scp_trace_if_read,
	.write = scp_trace_if_write,
	.open  = scp_trace_if_open,
};

static struct miscdevice scp_trace_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "scp_trace",
	.fops = &scp_trace_ops
};

static int __init init_scp_trace(void)
{
	int ret;

	ret = misc_register(&scp_trace_device);
	if (unlikely(ret != 0)) {
		pr_err("[SCP] misc register failed\n");
		return ret;
	}

	ret = device_create_file(scp_trace_device.this_device, &dev_attr_scp_trace_run);
	if (unlikely(ret != 0)) {
		pr_err("[SCP] misc register failed\n");
		return ret;
	}

	ret = device_create_file(scp_trace_device.this_device, &dev_attr_scp_trace_setup);
	if (unlikely(ret != 0)) {
		pr_err("[SCP] misc register failed\n");
		return ret;
	}

	ret = device_create_file(scp_trace_device.this_device, &dev_attr_scp_trace_select);
	if (unlikely(ret != 0)) {
		pr_err("[SCP] misc register failed\n");
		return ret;
	}


	scp_ipi_registration(IPI_TRACE_SCP, scp_trace_handler, "scp trace");
	return 0;
}

static void __exit exit_scp_trace(void)
{
}

module_init(init_scp_trace);
module_exit(exit_scp_trace);

