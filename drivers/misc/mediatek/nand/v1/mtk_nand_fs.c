/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/random.h>
#include "mtk_nand_util.h"
#include "mtk_nand_device_feature.h"
#include "mtk_nand_ops.h"
#include "mtk_nand_ops_test.h"
#include "mtk_nand_fs.h"

#define SEQ_printf(m, x...)		\
do {			\
	if (m)			\
		seq_printf(m, x);	\
	else			\
		pr_debug(x);		\
} while (0)

/******************************************************************************
 * mtk_nand_proc_read
 *
 * DESCRIPTION:
 *Read the proc file to get the interrupt scheme setting !
 *
 * PARAMETERS:
 *char *page, char **start, off_t off, int count, int *eof, void *data
 *
 * RETURNS:
 *None
 *
 * NOTES:
 *None
 *
 ******************************************************************************/
int mtk_nand_proc_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	char *p = buffer;
	int len = 0;
	int i;

	p += sprintf(p, "ID: ");
	for (i = 0; i < gn_devinfo.id_length; i++)
		p += sprintf(p, " 0x%x", gn_devinfo.id[i]);

	p += sprintf(p, "\n");
	p += sprintf(p, "total size: %dMiB; part number: %s\n", gn_devinfo.totalsize,
			 gn_devinfo.devciename);
	p += sprintf(p, "Current working in %s mode\n", g_i4Interrupt ? "interrupt" : "polling");
	p += sprintf(p, "NFI_ACCON = 0x%x\n", DRV_Reg32(NFI_ACCCON_REG32));
	p += sprintf(p, "NFI_NAND_TYPE_CNFG_REG32 = 0x%x\n", DRV_Reg32(NFI_NAND_TYPE_CNFG_REG32));
	len = p - buffer;

	return len < count ? len : count;
}

/******************************************************************************
 * mtk_nand_proc_write
 *
 * DESCRIPTION:
 *Write the proc file to set the interrupt scheme !
 *
 * PARAMETERS:
 *struct file* file, const char* buffer,	unsigned long count, void *data
 *
 * RETURNS:
 *None
 *
 * NOTES:
 *None
 *
 ******************************************************************************/
ssize_t  mtk_nand_proc_write(struct file *file, const char *buffer, size_t  count, loff_t *data)
{
	struct mtd_info *mtd = &host->mtd;
	char buf[16];
	char cmd;
	int value;
	int len = (int)count;
	int ret;
	unsigned int tmp;
#ifdef __HWA_DBG__
	struct nand_chip *chip = mtd->priv;
	loff_t from, to;
	int offset;
	u8 *ptr;
#endif

	if (len >= sizeof(buf))
		len = sizeof(buf) - 1;

	if (copy_from_user(buf, buffer, len))
		return -EFAULT;

	ret = sscanf(buf, "%c%x", &cmd, &value);
	if (ret)
		pr_debug("ret = %d\n", ret);

	switch (cmd) {
	case '1':
		PEM_INIT();
		break;
	case '2':
		PEM_DUMP_D(NULL);
		break;
	case 'A':		/* NFIA driving setting */
		tmp = DRV_Reg32(NFI_ACCCON_REG32);
		pr_info("NFI_ACCON = 0x%x\n", tmp);
		tmp = DRV_Reg32(NFI_ACCCON1_REG32);
		pr_info("NFI_ACCON1  = 0x%x\n", tmp);
		tmp = DRV_Reg32(NFI_DLYCTRL_REG32);
		pr_info("NFI_DLYCTRL = 0x%x\n", tmp);
		tmp = DRV_Reg16(NFI_NAND_TYPE_CNFG_REG32);
		pr_info("NFI_NAND_TYPE = 0x%x (DDR %d)\n", tmp, DDR_INTERFACE);
		break;
	case 'B':		/* NFIB driving setting */
		break;
	case 'D':
#ifdef _MTK_NAND_DUMMY_DRIVER_
	pr_debug("Enable dummy driver\n");
		dummy_driver_debug = 1;
#endif
		break;
	case 'H':
		value = !!value;
		g_i4Homescreen = value;
		break;
	case 'I':
		value = !!value;
		if (value != g_i4InterruptRdDMA) {
			nand_get_device(mtd, FL_READING);

			g_i4InterruptRdDMA = value;
#if 0
			if (g_i4Interrupt) {
				DRV_Reg16(NFI_INTR_REG16);
				enable_irq(MT_NFI_IRQ_ID);
			} else
				disable_irq(MT_NFI_IRQ_ID);
#endif
			nand_release_device(mtd);
		}
		break;
	case 'P':
		PFM_INIT();
		CALL_TRACE_INIT();
		break;
	case 'R':
		PFM_DUMP_D(NULL);
		DUMP_CALL_TRACE(NULL);

		/* dump_log(); */
		break;
	case 'T':
		nand_get_device(mtd, FL_READING);
		DRV_WriteReg32(NFI_ACCCON_REG32, value);
		nand_release_device(mtd);
		break;
#ifdef __HWA_DBG__
	case 'U':
	nand_get_device(mtd, FL_WRITING);
		mtk_nand_erase(mtd, (0x36000000/16384));
		mtk_nand_erase(mtd, (0x36600000/16384));
		for (offset = 0; offset < 32; offset++) {
			from = 0x16800000 + (offset * 0x60000);
			to = 0x36000000 + (offset * 0x60000);
			mtk_nand_merge_tlc_block(mtd, from, to, 0x60000);
		}
		nand_release_device(mtd);
		break;
	case 'V':
		nand_get_device(mtd, FL_WRITING);
		from = 0x16800000;
		offset = 0x16800000 / 16384;
		ptr = test_tlc_buffer;
		for (offset = (0x16800000 / 16384); offset < ((0x16800000 / 16384) + 384); offset++) {
			mtk_nand_read_page(mtd, chip, ptr, offset);
			ptr += 16384;
		}
		for (offset = 0; offset < 32; offset++) {
			mtk_nand_erase(mtd, ((0x36000000 + (offset * 0x600000)) / 16384));
			mtk_nand_write_tlc_block
			(mtd, chip, test_tlc_buffer, ((0x36000000 + (offset * 0x600000)) / 16384), 0x600000);
		}
		nand_release_device(mtd);
		break;
	case 'W':
		nand_get_device(mtd, FL_WRITING);
		offset = 0x36000000 / 16384;
		ptr = test_tlc_buffer;
		for (value = 0; value < 10000; value++) {
			pr_info("%d times\n", value);
			for (offset = (0x36000000 / 16384); offset < ((0x36000000 / 16384) + (384*32)); offset++) {
				mtk_nand_read_page(mtd, chip, ptr, offset);
				if (value%100 == 0) {
					msleep(20);
/*	pr_info("page\t0x%x\t%dth\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
 *	offset, read_count, bit_error[0], bit_error[1], bit_error[2], bit_error[3], bit_error[4], bit_error[5],
 *	bit_error[6], bit_error[7], bit_error[8], bit_error[9], bit_error[10], bit_error[11],
 *	bit_error[12], bit_error[13], bit_error[14], bit_error[15]);
 */
				} else {
					if (offset%100 == 0)
						pr_info("check alive\n");
				}
			}
		}
		break;
	case 'X':
		nand_get_device(mtd, FL_WRITING);
		offset = 0x36000000 / 16384;
		ptr = test_tlc_buffer;
		for (offset = (0x36000000 / 16384); offset < ((0x36000000 / 16384) + (384*2)); offset++) {
			mtk_nand_read_page(mtd, chip, ptr, offset);
				msleep(20);
				ptr += 16384;
/*	pr_info("page\t0x%x\t%dth\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
 *	offset, read_count, bit_error[0], bit_error[1], bit_error[2], bit_error[3], bit_error[4], bit_error[5],
 *	bit_error[6], bit_error[7], bit_error[8], bit_error[9],
 *	bit_error[10], bit_error[11], bit_error[12], bit_error[13], bit_error[14], bit_error[15]);
 */
		}
		ptr = test_tlc_buffer;
		for (offset = (0x16800000 / 16384); offset < ((0x16800000 / 16384) + (384*2)); offset++) {
			empty_true = FALSE;
			mtk_nand_read_page(mtd, chip, test_buffer, offset);
			if (empty_true)
				pr_info("empty_true 0x%x\n", offset);
			if (memcmp(test_buffer, ptr, 16384))
				pr_info("cmp fail 0x%x\n", offset);
			ptr += 16384;
		}
		break;
#endif
	default:
		break;
	}

	return len;
}

int mtk_nand_proc_show(struct seq_file *m, void *v)
{
#ifndef __D_PFM__
	int i;
	unsigned int tmp;

	SEQ_printf(m, "ID:");
	for (i = 0; i < gn_devinfo.id_length; i++)
		SEQ_printf(m, " 0x%x", gn_devinfo.id[i]);

	SEQ_printf(m, "\n");
	SEQ_printf(m, "total size: %dMiB; part number: %s\n", gn_devinfo.totalsize,
		gn_devinfo.devciename);
	SEQ_printf(m, "Current working in %s mode\n", g_i4Interrupt ? "interrupt" : "polling");
	SEQ_printf(m, "Read DMA mode in %s mode\n", g_i4InterruptRdDMA ? "interrupt" : "polling");

	tmp = DRV_Reg32(NFI_ACCCON_REG32);
	SEQ_printf(m, "NFI_ACCON = 0x%x\n", tmp);
	tmp = DRV_Reg32(NFI_ACCCON1_REG32);
	SEQ_printf(m, "NFI_ACCON1  = 0x%x\n", tmp);
	tmp = DRV_Reg32(NFI_DLYCTRL_REG32);
	SEQ_printf(m, "NFI_DLYCTRL = 0x%x\n", tmp);
	tmp = DRV_Reg16(NFI_NAND_TYPE_CNFG_REG32);
	SEQ_printf(m, "NFI_NAND_TYPE = 0x%x (DDR %d)\n", tmp, DDR_INTERFACE);
#endif
	PEM_DUMP_D(m);
	PFM_DUMP_D(m);
	DUMP_CALL_TRACE(m);

	return 0;
}


static int mtk_nand_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_nand_proc_show, inode->i_private);
}


static const struct file_operations mtk_nand_fops = {
	.open = mtk_nand_proc_open,
	.write = mtk_nand_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

ssize_t mtk_nand_rd_para_proc_write(struct file *file, const char __user *buffer, size_t count,
				loff_t *data)
{
	char buf[32];
	char para_name[16];
	int value;
	int len = count;

	if (len >= sizeof(buf))
		len = sizeof(buf) - 1;

	if (copy_from_user(buf, buffer, len))
		return -EFAULT;
	nand_err("buf:%s", buf);
	if (sscanf(buf, "%s%x", para_name, &value) != 2)
		return -EINVAL;
	nand_err("para_name:%s, value:%d", para_name, value);

	if (!strcmp(para_name, "sample_count"))
		host->rd_para.sample_count = value;
	else if (!strcmp(para_name, "t_sector_ahb"))
		host->rd_para.t_sector_ahb = value;
	else if (!strcmp(para_name, "t_schedule"))
		host->rd_para.t_schedule = value;
	else if (!strcmp(para_name, "t_sleep_range"))
		host->rd_para.t_sleep_range = value;
	else if (!strcmp(para_name, "t_threshold"))
		host->rd_para.t_threshold = value;

	return len;
}

int mtk_nand_rd_para_proc_show(struct seq_file *m, void *v)
{
	SEQ_printf(m, "sample_count:%d\n", host->rd_para.sample_count);
	SEQ_printf(m, "t_sector_ahb:%d\n", host->rd_para.t_sector_ahb);
	SEQ_printf(m, "t_schedule:%d\n", host->rd_para.t_schedule);
	SEQ_printf(m, "t_sleep_range:%d\n", host->rd_para.t_sleep_range);
	SEQ_printf(m, "t_threshold:%d\n", host->rd_para.t_threshold);

	return 0;
}

static int mtk_nand_rd_para_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_nand_rd_para_proc_show, inode->i_private);
}

static const struct file_operations mtk_nand_rd_para_fops = {
	.open = mtk_nand_rd_para_open,
	.write = mtk_nand_rd_para_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
int mtk_nand_set_err_para(
	struct err_para *para, const char *buf)
{
	char type[16];
	int value;

	if (sscanf(buf, "%s%d", type, &value) != 2)
		return -1;

	if (!strcmp(type, "rate"))
		para->rate = value;
	else if (!strcmp(type, "count"))
		para->count = value;
	else if (!strcmp(type, "block"))
		para->block = value;
	else if (!strcmp(type, "page"))
		para->page = value;

	return 0;
}
int mtk_nand_show_err_para(
	struct seq_file *m, struct err_para *para,
	const char *name)
{
	SEQ_printf(m, "======%s======\n", name);
	SEQ_printf(m, " rate:%d", para->rate);
	SEQ_printf(m, " count:%d", para->count);
	SEQ_printf(m, " block:%d", para->block);
	SEQ_printf(m, " page:%d\n", para->page);

	return 0;
}

ssize_t mtk_nand_sim_err_proc_write(
	struct file *file, const char __user *buffer,
	size_t count, loff_t *data)
{
	char buf[32];
	char name[16];
	char *pbuf = buf;
	int len = count;
	struct sim_err *err;

	if (len >= sizeof(buf))
		len = sizeof(buf) - 1;

	if (copy_from_user(buf, buffer, len))
		return -EFAULT;
	nand_err("buf:%s", buf);
	if (sscanf(buf, "%s", name) != 1)
		return -EINVAL;
	pbuf = pbuf + strlen(name);
	nand_err("name:%s", name);

	err = &data_info->err;

	if (!strcmp(name, "erase"))
		mtk_nand_set_err_para(&err->erase_fail, pbuf);
	else if (!strcmp(name, "write"))
		mtk_nand_set_err_para(&err->write_fail, pbuf);
	else if (!strcmp(name, "read"))
		mtk_nand_set_err_para(&err->read_fail, pbuf);
	else if (!strcmp(name, "bitflip"))
		mtk_nand_set_err_para(&err->bitflip_fail, pbuf);
	else if (!strcmp(name, "bad"))
		mtk_nand_set_err_para(&err->bad_block, pbuf);
	return len;
}

int mtk_nand_sim_err_proc_show(struct seq_file *m, void *v)
{
	struct sim_err *err = &data_info->err;

	mtk_nand_show_err_para(m, &err->erase_fail, "Erase Fail");
	mtk_nand_show_err_para(m, &err->write_fail, "Write Fail");
	mtk_nand_show_err_para(m, &err->read_fail, "Read Fail");
	mtk_nand_show_err_para(m, &err->bitflip_fail, "Bitflip Error");
	mtk_nand_show_err_para(m, &err->bad_block, "Bad Block Error");
	return 0;
}

static int mtk_nand_sim_err_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_nand_sim_err_proc_show, inode->i_private);
}

static const struct file_operations mtk_nand_sim_err_fops = {
	.open = mtk_nand_sim_err_open,
	.write = mtk_nand_sim_err_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

enum operation_types op_erase, op_write;
static unsigned char *w_page_buf, *w_oob_buf;
static unsigned char *r_page_buf, *r_oob_buf;
int mtk_nand_ftl_callback(
	struct mtk_nand_chip_info *info,
	unsigned char *data_buffer,
	unsigned char *oob_buffer, unsigned int block,
	unsigned int page, int status, void *userdata)
{
	enum operation_types *op = (enum operation_types *)userdata;

	if (status)
		pr_info("op_type:%d, status:%d, block:%d, page:%d\n", *op, status, block, page);

	return 0;
}

static int mtk_nand_init_buf(unsigned char **buf, int len, unsigned int value)
{
	if (*buf)
		return 0;

	*buf = kmalloc(len, GFP_KERNEL);
	if (*buf == NULL) {
		nand_err("kmalloc fail, len:%d", len);
		return -1;
	}
	if (value > 0xff)
		get_random_bytes(*buf, len);
	else
		memset(*buf, 0, len);

	return 0;
}
static int mtk_nand_ftl_init_buf(struct mtk_nand_chip_info *info)
{
	int page_size, oob_size;

	page_size = max(info->data_page_size, info->log_page_size);
	oob_size = max(info->data_oob_size, info->log_oob_size);

	mtk_nand_init_buf(&w_page_buf, page_size, 0x100);
	mtk_nand_init_buf(&w_oob_buf, oob_size, 0x100);
	mtk_nand_init_buf(&r_page_buf, page_size, 0);
	mtk_nand_init_buf(&r_oob_buf, oob_size, 0);

	return 0;
}

static void mtk_nand_free_buf(unsigned char **buf)
{
	if (*buf != NULL) {
		kfree(buf);
		*buf = NULL;
	}
}

static int mntl_block_thread_stop(void)
{
	return kthread_stop(data_info->blk_thread);
}

static int mntl_block_thread_wakeup(void)
{
	if (!IS_ERR(data_info->blk_thread))
		return wake_up_process(data_info->blk_thread);

	return -1;
}

void mtk_nand_ftl_print_buf(char *buf, int len)
{
	int i;

	pr_info("print buf:\n");
	for (i = 0; i < len; i++)
		pr_info("%x ", buf[i]);
	pr_info("\nprint buf End\n");
}
bool print_cmd;
int mtk_nand_ftl_compare_buf(char *buf0, char *buf1, int len)
{
	int i = 0;

	if (len <= 0)
		pr_info("mtk_nand_ftl_compare_buf, len <= 0!!\n");

	for (i = 0; i < len; i++)
		if (buf0[i] != buf1[i])
			break;
	if (i == len)
		return 0;
	else
		return -1;
}

void mtk_nand_ftl_write_page(struct mtk_nand_chip_info *info,
		unsigned int block, unsigned int page, bool more,
		mtk_nand_callback_func func)
{
	int ret;

	ret = mtk_nand_chip_write_page(info, w_page_buf, w_oob_buf, block, page,
		more, func, &op_write);
	if (ret)
		pr_info("mtk_nand_chip_write_page return fail:%d\n", ret);
}

int mtk_nand_ftl_read_page(struct mtk_nand_chip_info *info,
		unsigned int block, unsigned int page,
		unsigned int offset, unsigned int size,
		bool print, bool compare)
{
	int page_size, oob_size, read_oob_size, oob_offset;
	int ret;

	page_size = (block < info->data_block_num) ? info->data_page_size : info->log_page_size;
	oob_size = (block < info->data_block_num) ? info->data_oob_size : info->log_oob_size;
	read_oob_size = oob_size/(page_size/size);
	oob_offset = oob_size * offset / page_size;

	memset(r_page_buf, 0, size);
	memset(r_oob_buf, 0, read_oob_size);

	ret = mtk_nand_chip_read_page(info, r_page_buf, r_oob_buf, block, page, offset, size);
	if (ret) {
		pr_info("mtk_nand_chip_read_page return fail:%d\n", ret);
		return ret;
	} else if (print) {
		mtk_nand_ftl_print_buf(r_page_buf, size);
		mtk_nand_ftl_print_buf(r_oob_buf, read_oob_size);
	}

	ret = mtk_nand_ftl_compare_buf(r_page_buf, w_page_buf + offset, size);
	if (ret)
		goto OUT;
	ret = mtk_nand_ftl_compare_buf(r_oob_buf, w_oob_buf + oob_offset, read_oob_size);

OUT:
	if (ret)
		pr_info("read compare fail: block:%d, page:%d\n", block, page);
	return ret;
}

ssize_t mtk_nand_ftl_proc_write(
	struct file *file, const char __user *buffer,
	size_t count, loff_t *data)
{
	char buf[64];
	char cmd[16];
	char *pbuf = buf;
	int len = count;
	struct mtk_nand_data_info *ftl_data = data_info;
	struct mtk_nand_chip_info *info = &ftl_data->chip_info;
	struct data_bmt_struct *data_bmt = &ftl_data->bmt;
	mtk_nand_callback_func func = &mtk_nand_ftl_callback;
	unsigned int block, page, more, offset, size;
	unsigned long input;
	int ret = 0;

	if (len >= sizeof(buf))
		len = sizeof(buf) - 1;

	if (copy_from_user(buf, buffer, len))
		return -EFAULT;
	pr_info("buf:%s\n", buf);
	buf[len-1] = 0;
	if (sscanf(buf, "%s", cmd) != 1)
		return -EINVAL;
	pbuf = pbuf + strlen(cmd) + 1;
	pr_info("cmd:%s\n", cmd);

	op_erase = MTK_NAND_OP_ERASE;
	op_write = MTK_NAND_OP_WRITE;

	if (!strcmp(cmd, "init")) {
		mtk_nand_ftl_init_buf(info);
	} else if (!strcmp(cmd, "exit")) {
		mtk_nand_free_buf(&w_page_buf);
		mtk_nand_free_buf(&w_oob_buf);
		mtk_nand_free_buf(&r_page_buf);
		mtk_nand_free_buf(&r_oob_buf);
	} else if (!strcmp(cmd, "stopbld")) {
		ret = mntl_block_thread_stop();
		if (ret)
			pr_info("stop mntl block thread fail, ret:%d\n", ret);
		else
			pr_info("stop mntl block thread success\n");
	} else if (!strcmp(cmd, "wakebld")) {
		ret = mntl_block_thread_wakeup();
		if (ret)
			pr_info("wakeup mntl block thread fail, ret:%d\n", ret);
		else
			pr_info("wakeup mntl block thread success\n");
	} else if (!strcmp(cmd, "erase")) {
		if (sscanf(pbuf, "%d%d", &block, &more) != 2)
			goto INVALID_ARG;
		ret = mtk_nand_chip_erase_block(info, block, more, func, &op_erase);
		if (ret)
			pr_info("mtk_nand_chip_erase_block return fail:%d\n", ret);
	} else if (!strcmp(cmd, "write")) {
		mtk_nand_ftl_init_buf(info);
		if (sscanf(pbuf, "%d%d%d", &block, &page, &more) != 3)
			goto INVALID_ARG;

		mtk_nand_ftl_write_page(info, block, page, more, func);
	}  else if (!strcmp(cmd, "write2block")) {
		int block2, pages, i, page_size;

		mtk_nand_ftl_init_buf(info);

		if (sscanf(pbuf, "%d%d", &block, &block2) != 2)
			goto INVALID_ARG;
		print_cmd = false;
		pages = (block < info->data_block_num) ? info->data_page_num : info->log_page_num;
		page_size = (block < info->data_block_num) ? info->data_page_size : info->log_page_size;
		for (i = 0; i < pages; i++) {
			mtk_nand_ftl_write_page(info, block, i, 1, func);
			mtk_nand_ftl_write_page(info, block2, i, 1, func);
		}
		mtk_nand_chip_sync(info);
		print_cmd = false;
		for (i = 0; i < pages; i++) {
			ret = mtk_nand_ftl_read_page(info, block, i, 0, page_size, 0, true);
			if (ret) {
				pr_info("read or compare fail:block:%d, page:%d\n", block, page);
				break;
			}
			ret = mtk_nand_ftl_read_page(info, block2, i, 0, page_size, 0, true);
			if (ret) {
				pr_info("read or compare fail:block:%d, page:%d\n", block2, page);
				break;
			}
		}
		if (i == pages)
			pr_info("All pages readback and compare pass ^^\n");
		else
			pr_info("Some page readback or compare fail ==!!^\n");
	} else if (!strcmp(cmd, "write1block")) {
		int pages, i;

		mtk_nand_ftl_init_buf(info);

		ret = kstrtoul(pbuf, 0, &input);
		if (ret)
			goto INVALID_ARG;
		block = (int) input;
		pages = (block < info->data_block_num) ? info->data_page_num : info->log_page_num;
		pr_info("block:%d\n", block);
		for (i = 0; i < pages; i++)
			mtk_nand_ftl_write_page(info, block, i, 1, func);

		mtk_nand_chip_sync(info);
	} else if (!strcmp(cmd, "read")) {
		int print = 0;

		mtk_nand_ftl_init_buf(info);
		if (sscanf(pbuf, "%d%d%d%d%d", &block, &page, &offset, &size, &print) < 4)
			goto INVALID_ARG;

		mtk_nand_ftl_read_page(info, block, page, offset, size, print, true);
	} else if (!strcmp(cmd, "row2bp")) {
		int rowAddr = 0;
		int multiple = 1;

		/*row address to block, page num*/
		ret = kstrtoul(pbuf, 0, &input);
		if (ret)
			goto INVALID_ARG;
		rowAddr = (int) input;
		pr_info("rowAddr:0x%x\n", rowAddr);
		block = rowAddr/info->data_page_num - data_bmt->start_block;
		if (block < 0 || block >= (info->data_block_num + info->log_block_num))
			goto INVALID_ARG;

		multiple = ((gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_TLC) &&
				(block > info->data_block_num)) ? MTK_TLC_DIV : 1;
		page = (rowAddr%info->data_page_num)/multiple;
		pr_info("rowAddr:0x%x, block:%d, page:%d\n", rowAddr, block, page);
	} else if (!strcmp(cmd, "bp2row")) {
		int rowAddr = 0;
		int multiple = 1;

		/*row address to block, page num*/
		if (sscanf(pbuf, "%d%d", &block, &page) != 2)
			goto INVALID_ARG;
		block = rowAddr/info->data_page_num - data_bmt->start_block;
		if (block < 0 || block > (info->data_block_num + info->log_block_num))
			goto INVALID_ARG;

		multiple = ((gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_TLC) &&
				(block >= info->data_block_num)) ? MTK_TLC_DIV : 1;

		rowAddr = (block+data_info->bmt.start_block)*info->data_page_num+page*multiple;

		pr_info("rowAddr:0x%x, block:%d, page:%d\n", rowAddr, block, page);
	} else {
		goto INVALID_ARG;
	}

	return len;
INVALID_ARG:
	pr_info("Invalid argument, ret:%d\n", ret);
	return len;
}

int mtk_nand_ftl_proc_show(struct seq_file *m, void *v)
{
	struct mtk_nand_data_info *ftl_data = data_info;
	struct nand_ftl_partition_info *part = &ftl_data->partition_info;
	struct data_bmt_struct *data_bmt = &ftl_data->bmt;
	struct data_bmt_entry *bmt_entry = data_bmt->entry, *entry;
	struct mtk_nand_chip_bbt_info *chip_bbt = &ftl_data->chip_bbt;
	struct mtk_nand_chip_info *chip_info = &ftl_data->chip_info;
	struct worklist_ctrl *elist_ctrl = &ftl_data->elist_ctrl;
	struct worklist_ctrl *wlist_ctrl = &ftl_data->wlist_ctrl;
	int multiple = 1;
	int i;

	SEQ_printf(m, "FTL Data bmt info:\n");
	SEQ_printf(m, " version:%d\n", data_bmt->version);
	SEQ_printf(m, " start_block:%d\n", data_bmt->start_block);
	SEQ_printf(m, " end_block:%d\n", data_bmt->end_block);
	SEQ_printf(m, " checksum:%d\n", data_bmt->checksum);
	SEQ_printf(m, " bad_count:%d\n", data_bmt->bad_count);

	SEQ_printf(m, " FTL Data bmt entry info:\n");
	for (i = 0; i < data_bmt->bad_count; i++) {
		entry = &bmt_entry[i];
		SEQ_printf(m, "  [%d], bad_index:%d, flag:%d\n", i, entry->bad_index, entry->flag);
	}

	SEQ_printf(m, "FTL chip_bbt info:\n");
	SEQ_printf(m, " initial_bad_num:%d\n", chip_bbt->initial_bad_num);
	SEQ_printf(m, " bad_block_num:%d\n", chip_bbt->bad_block_num);
	for (i = 0; i < chip_bbt->bad_block_num; i++)
		SEQ_printf(m, "  [%d], bad_block:%d\n", i, chip_bbt->bad_block_table[i]);

	SEQ_printf(m, "FTL chip_info info:\n");
	SEQ_printf(m, " info_version:%d\n", chip_info->info_version);
	SEQ_printf(m, " data_block_num:%d\n", chip_info->data_block_num);
	SEQ_printf(m, " data_page_num:%d\n", chip_info->data_page_num);
	SEQ_printf(m, " data_page_size:%d\n", chip_info->data_page_size);
	SEQ_printf(m, " data_oob_size:%d\n", chip_info->data_oob_size);
	SEQ_printf(m, " data_block_size:%d\n", chip_info->data_block_size);
	SEQ_printf(m, " data_pe:%d\n", chip_info->data_pe);
	SEQ_printf(m, " log_block_num:%d\n", chip_info->log_block_num);
	SEQ_printf(m, " log_page_num:%d\n", chip_info->log_page_num);
	SEQ_printf(m, " log_page_size:%d\n", chip_info->log_page_size);
	SEQ_printf(m, " log_oob_size:%d\n", chip_info->log_oob_size);
	SEQ_printf(m, " log_block_size:%d\n", chip_info->log_block_size);
	SEQ_printf(m, " log_pe:%d\n", chip_info->log_pe);
	SEQ_printf(m, " slc_ratio:%d\n", chip_info->slc_ratio);
	SEQ_printf(m, " start_block:%d\n", chip_info->start_block);
	SEQ_printf(m, " sector_size_shift:%d\n", chip_info->sector_size_shift);
	SEQ_printf(m, " max_keep_pages:%d\n", chip_info->max_keep_pages);
	SEQ_printf(m, " nand_flash_type:%d\n", chip_info->types);
	SEQ_printf(m, " plane_mask:%d\n", chip_info->plane_mask);
	SEQ_printf(m, " plane_num:%d\n", chip_info->plane_num);
	SEQ_printf(m, " chip_num:%d\n", chip_info->chip_num);
	SEQ_printf(m, " option:0x%x\n", chip_info->option);

	SEQ_printf(m, "FTL partition_info info:\n");
	SEQ_printf(m, " start_block:%d\n", part->start_block);
	SEQ_printf(m, " total_block:%d\n", part->total_block);
	SEQ_printf(m, " slc_ratio:%d\n", part->slc_ratio);
	SEQ_printf(m, " slc_block:%d\n", part->slc_block);

	SEQ_printf(m, "FTL elist_ctrl info:\n");
	SEQ_printf(m, " head:%p\n", &elist_ctrl->head);
	SEQ_printf(m, " total_num:%d\n", elist_ctrl->total_num);

	SEQ_printf(m, "FTL wlist_ctrl info:\n");
	SEQ_printf(m, " head:%p\n", &wlist_ctrl->head);
	SEQ_printf(m, " total_num:%d\n", wlist_ctrl->total_num);

	SEQ_printf(m, "Row Address Range:\n");
	SEQ_printf(m, " Data Block Row Address Range:(0x%x, 0x%x)\n",
		data_bmt->start_block * chip_info->data_page_num,
		(chip_info->data_block_num + data_bmt->start_block) * chip_info->data_page_num - 1);

	multiple = (gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_TLC) ? MTK_TLC_DIV : 1;
	SEQ_printf(m, " Log Block Row Address Range:(0x%x, 0x%x)\n",
		(chip_info->data_block_num + data_bmt->start_block) * chip_info->data_page_num,
		(chip_info->data_block_num + chip_info->log_block_num + data_bmt->start_block - 1) *
		chip_info->data_page_num + (chip_info->log_page_num - 1) * multiple);

	return 0;
}

static int mtk_nand_ftl_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_nand_ftl_proc_show, inode->i_private);
}

static const struct file_operations mtk_nand_ftl_fops = {
	.open = mtk_nand_ftl_open,
	.write = mtk_nand_ftl_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

ssize_t mtk_nand_debug_proc_write(
	struct file *file, const char __user *buffer,
	size_t count, loff_t *data)
{
	char buf[32];
	char para[16];
	char *pbuf = buf;
	int len = count;
	struct mtk_nand_debug *debug = &host->debug;

	if (len >= sizeof(buf))
		len = sizeof(buf) - 1;

	if (copy_from_user(buf, buffer, len))
		return -EFAULT;
	pr_info("buf:%s\n", buf);
	if (sscanf(buf, "%s", para) != 1)
		return -EINVAL;
	pbuf = pbuf + strlen(para) + 1;
	pr_info("cmd:%s\n", para);

	if (!strcmp(para, "err")) {
		int print;
		char member[16];

		if (sscanf(pbuf, "%s%d", member, &print) != 2)
			goto INVALID_ARG;
		if (!strcmp(member, "en_print"))
			debug->err.en_print = print ? true : false;
	} else {
		goto INVALID_ARG;
	}

	return len;
INVALID_ARG:
	pr_info("Invalid argument\n");
	return len;
}

static int mtk_nand_debug_proc_show(struct seq_file *m, void *v)
{
	struct mtk_nand_debug *debug = &host->debug;

	SEQ_printf(m, "Host debug err info:\n");
	SEQ_printf(m, " en_print:%d\n", debug->err.en_print);

	return 0;
}

static int mtk_nand_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_nand_debug_proc_show, inode->i_private);
}

static const struct file_operations mtk_nand_debug_fops = {
	.open = mtk_nand_debug_open,
	.write = mtk_nand_debug_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry *debugfs_root;
static void nand_init_device_debugfs(void)
{
	debugfs_root = debugfs_create_dir("nand", NULL);
	if (IS_ERR(debugfs_root) || !debugfs_root) {
		nand_err("failed to create debugfs directory");
		debugfs_root = NULL;
		return;
	}

	debugfs_create_file("rd_para", S_IFREG | S_IRUGO,
						debugfs_root, NULL, &mtk_nand_rd_para_fops);
	debugfs_create_file("sim_err", S_IFREG | S_IRUGO,
						debugfs_root, NULL, &mtk_nand_sim_err_fops);
	debugfs_create_file("ftl", S_IFREG | S_IRUGO,
						debugfs_root, NULL, &mtk_nand_ftl_fops);
	debugfs_create_file("debug", S_IFREG | S_IRUGO,
						debugfs_root, NULL, &mtk_nand_debug_fops);
}

static void nand_init_device_procfs(void)
{
	struct proc_dir_entry *entry;

	entry = proc_create(PROCNAME, 0664, NULL, &mtk_nand_fops);
	if (!entry)
		nand_err("create proc nand file fail!!!");
}

int mtk_nand_fs_init(void)
{
	nand_init_device_procfs();
	nand_init_device_debugfs();

	return 0;
}

void mtk_nand_fs_exit(void)
{
	remove_proc_entry(PROCNAME, NULL);
	debugfs_remove_recursive(debugfs_root);
}
