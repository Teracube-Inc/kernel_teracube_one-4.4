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
 *
 *
 * mtk_nand_chip.c - MTK NAND Flash Device Driver for FTL partition
 *
 * DESCRIPTION:
 *	This file provid the other drivers nand relative functions
 *	1) GPL Related Function;
 *	2) BMT Realtaed Function;
 *	3) MTK Nand Driver Related Functnions
 *	4) API for nand Wrapper
 *
 * modification history
 * ----------------------------------------
 * v1.0, 2 Aug 2016, mtk
 * ----------------------------------------
 */
#include <linux/mutex.h>

/*#include "mtk_nand_ops.h"*/
#if defined(CONFIG_PWR_LOSS_MTK_SPOH)
#include <mach/power_loss_test.h>
#endif

#include "bmt.h"

int g_i4Homescreen;
#if defined(CONFIG_PWR_LOSS_MTK_SPOH)
struct mvg_case_stack *cstack;
#endif
struct mtk_nand_data_info *data_info;

static inline bool block_page_num_is_valid(
		struct mtk_nand_chip_info *info, unsigned int block,
		unsigned int page);

static bool is_tlc_nand(void);
static bool is_slc_block(struct mtk_nand_chip_info *info, unsigned int block);

#define ALL_SLC_BUFFER 1

/* mtk_isbad_block
 * Mark specific block as bad block,and update bad block list and bad block table.
 * @block: block address to markbad
 */
bool mtk_isbad_block(unsigned int block)
{
	struct mtk_nand_chip_bbt_info *chip_bbt;
	unsigned int i;

	chip_bbt = &data_info->chip_bbt;
	for (i = 0; i < chip_bbt->bad_block_num; i++) {
		if (block == chip_bbt->bad_block_table[i]) {
			nand_debug("Check block:0x%x is bad", block);
			return TRUE;
		}
	}

	/* nand_debug("Check block:0x%x is Good", block); */
	return FALSE;
}

/**
 * nand_release_device - [GENERIC] release chip
 * @mtd: MTD device structure
 *
 * Release chip lock and wake up anyone waiting on the device.
 */
void mtk_nand_release_device(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;

	if (chip->state != FL_READY && chip->state != FL_PM_SUSPENDED)
		nand_disable_clock();

	/* Release the controller and the chip */
	spin_lock(&chip->controller->lock);
	chip->controller->active = NULL;
	chip->state = FL_READY;
	wake_up(&chip->controller->wq);
	spin_unlock(&chip->controller->lock);
}

static int mtk_nand_read_pages(struct mtk_nand_chip_info *info,
		unsigned char *data_buffer, unsigned char *oob_buffer,
		unsigned int block, unsigned int page,
		unsigned int offset, unsigned int size)
{
	struct mtd_info *mtd = data_info->mtd;
	struct nand_chip *chip = mtd->priv;
	static unsigned char *fdm_buf;
#ifdef MTK_FORCE_READ_FULL_PAGE
	static unsigned char *page_buf;
#endif
	unsigned int page_addr, page_size, oob_size;
	unsigned int sect_num, start_sect;
	unsigned int col_addr, sect_read;
	unsigned int backup_corrected;
	int ret = 0;
#ifdef ALL_SLC_BUFFER
	bool block_type;
#endif

	nand_debug("block:%d page:%d offset:%d size:%d",
		block, page, offset, size);

	/* Grab the lock and see if the device is available */
	nand_get_device(mtd, FL_READING);

	chip->select_chip(mtd, 0);

	backup_corrected = mtd->ecc_stats.corrected;
#ifdef ALL_SLC_BUFFER
	block_type = is_slc_block(info, block);

	if ((gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_TLC)
		&& block_type)
		page_addr = (block+data_info->bmt.start_block)*info->data_page_num+page*MTK_TLC_DIV;
	else
		page_addr = (block+data_info->bmt.start_block)*info->data_page_num+page;

	/* For log area access */
	if (!block_type) {
		gn_devinfo.tlcControl.slcopmodeEn = FALSE;
		page_size = info->data_page_size;
		oob_size = info->log_oob_size;
	} else {
		/* nand_debug("Read SLC mode"); */
		gn_devinfo.tlcControl.slcopmodeEn = TRUE;
		page_size = info->log_page_size;
		oob_size = info->log_oob_size;
	}
#else
	if ((gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_TLC)
		&& (block >= info->data_block_num))
		page_addr = (block+data_info->bmt.start_block)*info->data_page_num+page*MTK_TLC_DIV;

	else
		page_addr = (block+data_info->bmt.start_block)*info->data_page_num+page;

	/* For log area access */
	if (block < info->data_block_num) {
		gn_devinfo.tlcControl.slcopmodeEn = FALSE;
		page_size = info->data_page_size;
		oob_size = info->log_oob_size;
	} else {
		/* nand_debug("Read SLC mode"); */
		gn_devinfo.tlcControl.slcopmodeEn = TRUE;
		page_size = info->log_page_size;
		oob_size = info->log_oob_size;
	}
#endif
	if (fdm_buf == NULL) {
		fdm_buf = kmalloc(1024, GFP_KERNEL);
		if (fdm_buf == NULL) {
			ret = -ENOMEM;
			nand_err("kmalloc fdm_buf fail\n");
		goto exit;
		} else {
			nand_info("kmalloc success. fdm_buf:%p\n", fdm_buf);
		}
	}
	PFM_BEGIN();

	if (size < page_size) {

		/* Sector read case */
		sect_num = page_size >> info->sector_size_shift;
		start_sect  = offset >> info->sector_size_shift;
		col_addr = start_sect*((1<<info->sector_size_shift)+mtd->oobsize/sect_num);
		sect_read = size >> info->sector_size_shift;
		/* nand_debug("Sector read col_addr:0x%x sect_read:%d, sect_num:%d, start_sect:%d", */
			/* col_addr, sect_read, sect_num, start_sect); */

#ifdef MTK_FORCE_READ_FULL_PAGE
		if (page_buf == NULL) {
			page_buf = kmalloc(page_size, GFP_KERNEL);
			if (page_buf == NULL) {
				ret = -ENOMEM;
				nand_err("kmalloc page_buf fail!!!size:%d\n", page_size);
				goto exit;
			} else {
				nand_info("kmalloc success. page_buf:%p\n", page_buf);
			}
		}

		PEM_PAGE_EN(TRUE);
		ret = mtk_nand_exec_read_page(mtd, page_addr, page_size, page_buf, fdm_buf);
		if (gn_devinfo.tlcControl.slcopmodeEn)
			PFM_END_OP(MNTL_PART_READ_SLC, info->data_page_size);
		else
			PFM_END_OP(MNTL_PART_READ_TLC, info->data_page_size);
		PEM_PAGE_EN(FALSE);

		memcpy(data_buffer, page_buf+offset, size);
		memcpy(oob_buffer, fdm_buf+start_sect*host->hw->nand_fdm_size,
			sect_read*host->hw->nand_fdm_size);
#else
		PEM_SUBPAGE_EN(TRUE);
		ret = mtk_nand_exec_read_sector(mtd, page_addr, col_addr, page_size,
				data_buffer, fdm_buf, sect_read);
		PFM_END_OP(MNTL_PART_READ_SECTOR, sect_read<<info->sector_size_shift);
		PEM_SUBPAGE_EN(FALSE);

		memcpy(oob_buffer, fdm_buf, sect_read*host->hw->nand_fdm_size);
#endif
	} else {
		PEM_PAGE_EN(TRUE);
		ret = mtk_nand_exec_read_page(mtd, page_addr, page_size, data_buffer, fdm_buf);

		if (gn_devinfo.tlcControl.slcopmodeEn)
			PFM_END_OP(MNTL_PART_READ_SLC, info->data_page_size);
		else
			PFM_END_OP(MNTL_PART_READ_TLC, info->data_page_size);
		PEM_PAGE_EN(FALSE);

		memcpy(oob_buffer, fdm_buf, oob_size);
	}

	if (ret != ERR_RTN_SUCCESS) {
		ret = -ENANDREAD;
	} else if (mtd->ecc_stats.corrected - backup_corrected) {
		nand_err("Detect bitflip here block:%d, page:%d", block, page);
		ret = -ENANDFLIPS;
	} else
		ret = 0;

exit:
	chip->select_chip(mtd, -1);
	mtk_nand_release_device(mtd);

	return ret;

}
#ifdef SUPPORT_MICRON_DEVICE
static int mtk_nand_read_multi_pages(struct mtk_nand_chip_info *info, int page_num,
			struct mtk_nand_chip_read_param *param)
{
	struct mtd_info *mtd = data_info->mtd;
	struct nand_chip *chip = mtd->priv;
	unsigned int backup_corrected;
	int ret;

	nand_get_device(mtd, FL_READING);
	chip->select_chip(mtd, 0);
	backup_corrected = mtd->ecc_stats.corrected;

	gn_devinfo.tlcControl.slcopmodeEn = is_slc_block(info, param[0].block);
	ret = mtk_nand_multi_plane_read(mtd, info, page_num, param);

	chip->select_chip(mtd, -1);
	mtk_nand_release_device(mtd);

	return ret;
}
#endif
/**
 * mtk_nand_write_pages -  NAND write with ECC
 * @ops: mtk_nand_chip_operation structure
 * @lecountn: number of pages to write
 *
 * NAND write with ECC.
 */
static int mtk_nand_write_pages(struct mtk_nand_chip_operation *ops0,
		struct mtk_nand_chip_operation *ops1)
{
	struct mtd_info *mtd = data_info->mtd;
	struct mtk_nand_chip_info *info = &data_info->chip_info;
	unsigned char *fdm_buf = NULL;
#if defined(CONFIG_MTK_TLC_NAND_SUPPORT)
	unsigned char *temp_page_buf = NULL;
	unsigned char *temp_fdm_buf = NULL;
#endif
	unsigned int page_addr0, page_addr1;
	unsigned int page_size, oob_size, sect_num;
	int ret = 0;

	if (ops0 == NULL) {
		nand_err("ops0 is NULL");
		return -EINVAL;
	}

	if (ops1 != NULL) {
		/* Check both in data or log area */
#ifdef ALL_SLC_BUFFER
		if (is_slc_block(info, ops1->block) != is_slc_block(info, ops0->block)) {
			nand_err("do not in same area ops0->block:0x%x ops1->block:0x%x ",
				ops0->block, ops1->block);
			return -EINVAL;
		}
#else
		if (((ops0->block < info->data_block_num)
				&& (ops1->block >= info->data_block_num))
			|| ((ops0->block >= info->data_block_num)
				&& (ops1->block < info->data_block_num))) {
			nand_err("do not in same area ops0->block:0x%x ops1->block:0x%x ",
				ops0->block, ops1->block);
			return -EINVAL;
		}
#endif
		if (mtk_isbad_block(ops1->block))
			page_addr1 = 0;
		else {
#ifdef ALL_SLC_BUFFER
			if ((gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_TLC)
				&& is_slc_block(info, ops1->block)) {
				page_addr1 = (ops1->block+data_info->bmt.start_block)*info->data_page_num
						+ ops1->page*MTK_TLC_DIV;
			} else {
				page_addr1 = (ops1->block+data_info->bmt.start_block)*info->data_page_num
					+ ops1->page;
			}
#else
			if ((gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_TLC)
				&& (ops1->block >= info->data_block_num)) {
				page_addr1 = (ops1->block+data_info->bmt.start_block)*info->data_page_num
						+ ops1->page*MTK_TLC_DIV;
			} else {
				page_addr1 = (ops1->block+data_info->bmt.start_block)*info->data_page_num
					+ ops1->page;
			}
#endif
		}
	} else {
		page_addr1 = 0;
	}

#ifdef ALL_SLC_BUFFER
	if ((gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_TLC)
		&& is_slc_block(info, ops0->block))
		page_addr0 = (ops0->block+data_info->bmt.start_block)*info->data_page_num
				+ ops0->page*MTK_TLC_DIV;
	else
		page_addr0 = (ops0->block+data_info->bmt.start_block)*info->data_page_num
				+ ops0->page;
#else
	if ((gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_TLC)
		&& (ops0->block >= info->data_block_num))
		page_addr0 = (ops0->block+data_info->bmt.start_block)*info->data_page_num
				+ ops0->page*MTK_TLC_DIV;
	else
		page_addr0 = (ops0->block+data_info->bmt.start_block)*info->data_page_num
				+ ops0->page;
#endif
	nand_debug("page_addr0= 0x%x page_addr1=0x%x",
		page_addr0, page_addr1);
#ifdef ALL_SLC_BUFFER
	if (!is_slc_block(info, ops0->block)) {
		page_size = info->data_page_size;
		gn_devinfo.tlcControl.slcopmodeEn = FALSE;
		oob_size = info->data_oob_size;
		if (page_addr1)
			host->wb_cmd = PROGRAM_LEFT_PLANE_CMD;
		else
			host->wb_cmd = ((ops0->page % 3 == WL_HIGH_PAGE) ||
				(gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_MLC_HYBER)) ?
				NAND_CMD_PAGEPROG : PROGRAM_RIGHT_PLANE_CMD;
	} else {
		nand_debug("write SLC mode");
		page_size = info->log_page_size;
		gn_devinfo.tlcControl.slcopmodeEn = TRUE;
		host->wb_cmd = page_addr1 ? PROGRAM_LEFT_PLANE_CMD : NAND_CMD_PAGEPROG;
		oob_size = info->log_oob_size;
	}
#else
	/* For log area access */
	if (ops0->block < info->data_block_num) {
		page_size = info->data_page_size;
		gn_devinfo.tlcControl.slcopmodeEn = FALSE;
		oob_size = info->data_oob_size;
		if (page_addr1)
			host->wb_cmd = PROGRAM_LEFT_PLANE_CMD;
		else
			host->wb_cmd = ((ops0->page % 3 == WL_HIGH_PAGE) ||
				(gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_MLC_HYBER)) ?
				NAND_CMD_PAGEPROG : PROGRAM_RIGHT_PLANE_CMD;
	} else {
		nand_debug("write SLC mode");
		page_size = info->log_page_size;
		gn_devinfo.tlcControl.slcopmodeEn = TRUE;
		host->wb_cmd = page_addr1 ? PROGRAM_LEFT_PLANE_CMD : NAND_CMD_PAGEPROG;
		oob_size = info->log_oob_size;
	}
#endif
	sect_num = page_size/(1<<info->sector_size_shift);

	fdm_buf = kmalloc(1024, GFP_KERNEL);
	if (fdm_buf == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(fdm_buf, 0xff, 1024);
#if defined(CONFIG_MTK_TLC_NAND_SUPPORT)
	if (gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_TLC) {
		if ((gn_devinfo.tlcControl.normaltlc) && (gn_devinfo.tlcControl.pPlaneEn)) {
			page_size /= 2;
			sect_num /= 2;
			tlc_snd_phyplane = FALSE;
			tlc_lg_left_plane = TRUE;
			memcpy(fdm_buf, ops0->oob_buffer, oob_size);
			temp_page_buf = ops0->data_buffer;
			temp_fdm_buf = fdm_buf;
			ret = mtk_nand_exec_write_page_hw(mtd, page_addr0,
						page_size, temp_page_buf, fdm_buf);
			if (ret != 0) {
				nand_err("write failed for page_addr0:0x%x", page_addr0);
				ret = -ENANDWRITE;
				goto exit;
			}

			tlc_lg_left_plane = FALSE;
			temp_page_buf += page_size;
			temp_fdm_buf += (sect_num * host->hw->nand_fdm_size);

			ret = mtk_nand_exec_write_page_hw(mtd, page_addr0,
						page_size, temp_page_buf, fdm_buf);
			if (ret != 0) {
				nand_err("write failed for page_addr0:0x%x", page_addr0);
				ret = -ENANDWRITE;
				goto exit;
			}

			if ((gn_devinfo.advancedmode & MULTI_PLANE)  && page_addr1) {
				nand_debug("write Multi_plane mode page_addr0:0x%x page_addr1:0x%x",
					page_addr0, page_addr1);
				tlc_snd_phyplane = TRUE;
				tlc_lg_left_plane = TRUE;
				memcpy(fdm_buf, ops1->oob_buffer, oob_size);

				temp_page_buf = ops1->data_buffer;
				temp_fdm_buf = fdm_buf;

				ret = mtk_nand_exec_write_page_hw(mtd, page_addr1, page_size,
						temp_page_buf, temp_fdm_buf);
				if (ret != 0) {
					nand_err("write failed for page_addr1:0x%x", page_addr1);
					ret = -ENANDWRITE;
					goto exit;
				}

				tlc_lg_left_plane = FALSE;
				temp_page_buf += page_size;
				temp_fdm_buf += (sect_num * host->hw->nand_fdm_size);

				ret  = mtk_nand_exec_write_page_hw(mtd, page_addr1, page_size,
						temp_page_buf, temp_fdm_buf);
				if (ret != 0) {
					nand_err("write failed for page_addr1:0x%x", page_addr1);
					ret = -ENANDWRITE;
					goto exit;
				}
				tlc_snd_phyplane = FALSE;
			}
		} else {
			host->pre_wb_cmd = true;
			memcpy(fdm_buf, ops0->oob_buffer, oob_size);
			temp_page_buf = ops0->data_buffer;
			temp_fdm_buf = fdm_buf;
			ret = mtk_nand_exec_write_page_hw(mtd, page_addr0,
					page_size, temp_page_buf, fdm_buf);
			if (ret != 0) {
				nand_err("write failed for page_addr0:0x%x", page_addr0);
				ret = -ENANDWRITE;
				goto exit;
			}

			if ((gn_devinfo.tlcControl.normaltlc)
					&& ((gn_devinfo.advancedmode & MULTI_PLANE) && page_addr1)) {
				nand_debug("write Multi_plane mode page_addr0:0x%x page_addr1:0x%x",
					page_addr0, page_addr1);
#ifdef ALL_SLC_BUFFER
				if (ops1->page % 3 == WL_HIGH_PAGE || is_slc_block(info, ops1->block))
					host->wb_cmd = NAND_CMD_PAGEPROG;
				else
					host->wb_cmd = PROGRAM_RIGHT_PLANE_CMD;
#else
				if (ops1->page % 3 == WL_HIGH_PAGE || (ops1->block >= info->data_block_num))
					host->wb_cmd = NAND_CMD_PAGEPROG;
				else
					host->wb_cmd = PROGRAM_RIGHT_PLANE_CMD;
#endif
				memcpy(fdm_buf, ops1->oob_buffer, oob_size);

				temp_page_buf = ops1->data_buffer;
				temp_fdm_buf = fdm_buf;

				ret = mtk_nand_exec_write_page_hw(mtd, page_addr1, page_size,
						temp_page_buf, temp_fdm_buf);
				if (ret != 0) {
					nand_err("write failed for page_addr1:0x%x", page_addr1);
					ret = -ENANDWRITE;
					goto exit;
				}
			}
		}
	} else
#endif
	{
		host->pre_wb_cmd = true;
		memcpy(fdm_buf, ops0->oob_buffer, oob_size);

		ret = mtk_nand_exec_write_page_hw(mtd, page_addr0,
				page_size, ops0->data_buffer, fdm_buf);
		if (ret != 0) {
			nand_err("write failed for page_addr0:0x%x", page_addr0);
			ret = -ENANDWRITE;
			goto exit;
		}

		if ((gn_devinfo.advancedmode & MULTI_PLANE)  && page_addr1) {
			host->wb_cmd = NAND_CMD_PAGEPROG;
			nand_debug("write Multi_plane mode page_addr0:0x%x page_addr1:0x%x",
				page_addr0, page_addr1);
			memcpy(fdm_buf, ops1->oob_buffer, oob_size);
			ret = mtk_nand_exec_write_page_hw(mtd, page_addr1,
						page_size, ops1->data_buffer, fdm_buf);
			if (ret != 0) {
				nand_err("write failed for page_addr1:0x%x", page_addr1);
				ret = -ENANDWRITE;
				goto exit;
			}
		}

	}

exit:
	kfree(fdm_buf);
	host->pre_wb_cmd = false;

	if (ret) {
		nand_err("fail!!!ret:%d, block0:%d, page0:%d\n", ret, ops0->block, ops0->page);
		if (ops1)
			nand_err("fail!!!ret:%d, block1:%d, page1:%d\n", ret, ops1->block, ops1->page);
	}
	return ret;
}

/**
 * nand_erase_blocks - [INTERN] erase block(s)
 * @ops: erase instruction
 * @count: block count to be ease
 *
 * Erase one ore more blocks.
 */
static int mtk_nand_erase_blocks(struct mtk_nand_chip_operation *ops0,
			struct mtk_nand_chip_operation *ops1)
{
	struct mtd_info *mtd = data_info->mtd;
	struct nand_chip *chip = mtd->priv;
	struct mtk_nand_chip_info *info = &data_info->chip_info;
	unsigned int page_addr0, page_addr1;
	int status, status_mask;
	int ret = 0;

	PFM_BEGIN();
	nand_debug("block0= 0x%x", ops0->block);

	if (ops0 == NULL) {
		nand_err("ops0 is NULL");
		return -EINVAL;
	}

	if (ops1 != NULL) {
		/* Check both in data or log area */
#ifdef ALL_SLC_BUFFER
		if (is_slc_block(info, ops1->block) != is_slc_block(info, ops0->block)) {
			nand_err("do not in same area ops0->block:0x%x ops1->block:0x%x ",
				ops0->block, ops1->block);
			return -EINVAL;
		}
#else

		if (((ops0->block < info->data_block_num)
				&& (ops1->block >= info->data_block_num))
			|| ((ops0->block >= info->data_block_num)
				&& (ops1->block < info->data_block_num))) {
			nand_err("do not in same area ops0->block:0x%x ops1->block:0x%x ",
				ops0->block, ops1->block);
			return -EINVAL;
		}
#endif
		if (mtk_isbad_block(ops1->block))
			page_addr1 = 0;
		else
			page_addr1 = (ops1->block+data_info->bmt.start_block)*info->data_page_num;
	} else {
		page_addr1 = 0;
	}

	page_addr0 = (ops0->block+data_info->bmt.start_block)*info->data_page_num;

	/* Grab the lock and see if the device is available */
	nand_get_device(mtd, FL_ERASING);
#ifdef ALL_SLC_BUFFER
	if (!is_slc_block(info, ops0->block)) {
		gn_devinfo.tlcControl.slcopmodeEn = FALSE;
		if (ops1 != NULL && is_slc_block(info, ops0->block))
			nand_err("Error!!!Invalid argu ops0->block:%d, ops1->block:%d\n",
				ops0->block, ops1->block);
	} else {
		nand_debug("erase SLC mode");
		gn_devinfo.tlcControl.slcopmodeEn = TRUE;
		if (ops1 != NULL && !is_slc_block(info, ops0->block))
			nand_err("Error!!!Invalid argu ops0->block:%d, ops1->block:%d\n",
				ops0->block, ops1->block);
	}
#else

	if (ops0->block < info->data_block_num) {
		gn_devinfo.tlcControl.slcopmodeEn = FALSE;
		if (ops1 != NULL && ops1->block >= info->data_block_num)
			nand_err("Error!!!Invalid argu ops0->block:%d, ops1->block:%d\n",
				ops0->block, ops1->block);
	} else {
		nand_debug("erase SLC mode");
		gn_devinfo.tlcControl.slcopmodeEn = TRUE;
		if (ops1 != NULL && ops1->block < info->data_block_num)
			nand_err("Error!!!Invalid argu ops0->block:%d, ops1->block:%d\n",
				ops0->block, ops1->block);
	}
#endif
	chip->select_chip(mtd, 0);

	status = mtk_chip_erase_blocks(mtd, page_addr0, page_addr1);

	if (!(status & NAND_STATUS_READY))
		nand_err("SR[6] is not ready state, status:0x%x", status);

	if (status & (NAND_STATUS_FAIL | SLC_MODE_OP_FALI))
		nand_err("warning erase, ops0->block:%d, status:0x%x\n",
				ops0->block, status);

	/* See if block erase is successful */
	status_mask = ((gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_TLC) &&
			gn_devinfo.tlcControl.slcopmodeEn) ? SLC_MODE_OP_FALI : NAND_STATUS_FAIL;
	ret = (status & status_mask) ? (-ENANDERASE) : 0;

	nand_debug("%s: done start page_addr0= 0x%x page_addr1= 0x%x",
			__func__, page_addr0, page_addr1);

	/* Deselect and wake up anyone waiting on the device */
	chip->select_chip(mtd, -1);
	mtk_nand_release_device(mtd);

	if (ret == -ENANDERASE)
		nand_err("erase fail,ops0->block:%d,status:0x%x\n", ops0->block, status);

#ifdef MTK_NAND_SIM_ERR
	if (!ret) {
		ret = sim_nand_err(MTK_NAND_OP_ERASE, ops0->block, -1);
		if (!ret && (ops1 != NULL))
			ret = sim_nand_err(MTK_NAND_OP_ERASE, ops1->block, -1);
		if (ret)
			return ret;
	}
#endif
	PFM_END_OP(MNTL_PART_ERASE, 0);
	/* Return more or less happy */
	return ret;
}



static inline int get_list_item_index(
	struct list_node *head, struct list_node *cur)
{
	int index = 0;
	struct list_node *item;

	item = head->next;
	while ((item != cur) && (item != NULL)) {
		item = item->next;
		index++;
	}

	if (item != cur) {
		nand_err("check no node cur:%p", cur);
		return 0;
	}

	return index;
}

static inline struct list_node *get_list_item_prev(
	struct list_node *head, struct list_node *cur)
{
	struct list_node *item;

	item = head;
	while ((item->next != cur) && (item->next != NULL))
		item = item->next;

	if (item->next != cur) {
		nand_err("check no node cur:%p", cur);
		return NULL;
	}

	return item;
}

static inline struct list_node *get_list_tail(
	struct list_node *head)
{
	struct list_node *item;

	item = head;
	while (item->next != NULL)
		item = item->next;

	return item;
}

static inline struct nand_work *get_list_work(
	struct list_node *node)
{
	return containerof(node, struct nand_work, list);
}

u32 get_ftl_row_addr(struct mtk_nand_chip_info *info,
	unsigned int block, unsigned int page)
{
	int div = 1;
	unsigned int start_block;

	start_block = data_info->bmt.start_block;

	if (is_tlc_nand() && is_slc_block(info, block))
		div = MTK_TLC_DIV;

	return (block + start_block) * info->data_page_num + page * div;
}

static struct list_node *seek_list_item(
	struct list_node *cur, unsigned int offset)
{
	struct list_node *item;
	int i;

	i = 0;
	item = cur;
	while (i < offset) {
		item = item->next;
		i++;
	}
	return item;
}

static inline void lock_list(
	struct worklist_ctrl *list_ctrl)
{
	spin_lock(&list_ctrl->list_lock);
}

static inline void unlock_list(
	struct worklist_ctrl *list_ctrl)
{
	spin_unlock(&list_ctrl->list_lock);
}

static inline void add_list_node(
	struct worklist_ctrl *list_ctrl, struct list_node *node)
{
	struct list_node *tail;

	lock_list(list_ctrl);
	tail = get_list_tail(&list_ctrl->head);
	tail->next = node;
	list_ctrl->total_num++;
	nand_debug("add node:%p", node);
	unlock_list(list_ctrl);
}

static inline int get_list_work_cnt(
	struct worklist_ctrl *list_ctrl)
{
	int cnt = 0;

	lock_list(list_ctrl);
	cnt = list_ctrl->total_num;
	unlock_list(list_ctrl);

	return cnt;
}

bool is_tlc_nand(void)
{
	return gn_devinfo.NAND_FLASH_TYPE == NAND_FLASH_TLC;
}

bool is_slc_block(struct mtk_nand_chip_info *info, unsigned int block)
{
#ifdef ALL_SLC_BUFFER
	int index, bit;

	index = block / 8;
	bit = block % 8;

	if ((info->block_type_bitmap[index] >> bit) & 1)
		return FALSE;
	return TRUE;
#else
	return block >= info->data_block_num;
#endif
}

static bool is_multi_plane(struct mtk_nand_chip_info *info)
{
	return (gn_devinfo.advancedmode & MULTI_PLANE);
}

static bool is_multi_read_support(struct mtk_nand_chip_info *info)
{
	return is_multi_plane(info) && (gn_devinfo.vendor == VEND_MICRON);
}

bool is_read_parameter_valid(struct mtk_nand_chip_operation *op)
{
	bool ret = false;

	if (op->data_buffer == NULL || op->oob_buffer == NULL)
		goto OUT;

	if ((op->offset % (1 << op->info->sector_size_shift) != 0)
		|| (op->size % (1 << op->info->sector_size_shift) != 0))
		goto OUT;

	if (!block_page_num_is_valid(op->info, op->block, op->page))
		goto OUT;

	ret = true;
OUT:
	if (!ret)
		nand_err("Invalid argu:(%p, %p, %d, %d, %d, %d)\n",
			op->data_buffer, op->oob_buffer, op->block, op->page,
			op->offset, op->size);
	return ret;
}

static bool are_on_diff_planes(unsigned int block0,
	unsigned int block1)
{
	return (block0 + block1) & 0x1;
}

static bool can_multi_plane(unsigned int block0, unsigned int page0,
				unsigned int block1, unsigned int page1)
{

	return ((block0 + block1) & 0x1) && (page0 == page1);
}

static bool can_ops_multi_plane(struct mtk_nand_chip_operation *ops0,
		struct mtk_nand_chip_operation *ops1)
{
	struct mtk_nand_chip_info *info = ops0->info;

	if (!is_multi_plane(info))
		return false;

	return can_multi_plane(ops0->block, ops0->page, ops1->block,
				ops1->page);
}

static void call_multi_work_callback(
	struct list_node *cur, int count, int status)
{
	int i;
	struct list_node *item;
	struct nand_work *work;
	struct mtk_nand_chip_operation *ops;

	item = cur;
	for (i = 0; i < count; i++)  {
		if (item == NULL) {
			nand_err("NULL item");
			return;
		}
		work = get_list_work(item);
		ops = &work->ops;
		nand_debug("callback i:%d, block:%d page:%d",
			i, ops->block, ops->page);
		ops->callback(ops->info, ops->data_buffer,
			ops->oob_buffer, ops->block,
			ops->page, status, ops->userdata);
		if (i >= count)
			break;
		item = item->next;
	}
}

static void call_tlc_page_group_callback(
	struct list_node *base_node, int start, int end,
	bool multi_plane, int status)
{
	struct list_node *item;
	int i;
	int count = multi_plane ? 2 : 1;

	item = seek_list_item(base_node, start * count);
	for (i = 0; i < end - start + 1; i++) {
		call_multi_work_callback(
			item, count, status);

		if (i >= end - start)
			break;
		if (count == 1)
			item = item->next;
		else
			item = item->next->next;
	}
}

static struct list_node *free_multi_work(
	struct worklist_ctrl *list_ctrl,
	struct list_node *head,
	struct list_node *start_node, int count)
{
	int i;
	struct list_node *item, *prev, *next;
	struct nand_work *work;

	item = start_node;

	lock_list(list_ctrl);
	prev = get_list_item_prev(head, start_node);
	if (prev == NULL) {
		nand_err("prev is NULL!!");
		return NULL;
	}
	for (i = 0; i < count; i++)  {
		if (item == NULL) {
			nand_err("NULL item");
			return NULL;
		}
		work = get_list_work(item);

		nand_debug("i:%d, page:%d",
			i, work->ops.page);

		next = item->next;
		list_ctrl->total_num--;
		kfree(work);
		item = next;
	}
	prev->next = item;
	unlock_list(list_ctrl);
	return item;
}

static struct list_node *free_multi_write_work(
	struct worklist_ctrl *list_ctrl,
	struct list_node *start_node, int count)
{
	struct list_node *head, *next;

	head = &list_ctrl->head;
	next = free_multi_work(list_ctrl,
			head, start_node, count);
	return next;
}

static struct list_node *free_multi_erase_work(
	struct worklist_ctrl *list_ctrl,
	struct list_node *start_node, int count)
{
	struct list_node *head, *next;

	head = &list_ctrl->head;
	next = free_multi_work(list_ctrl,
			head, start_node, count);
	return next;
}
static struct list_node *free_tlc_page_group_work(
	struct worklist_ctrl *list_ctrl,
	struct list_node *base_node, int start,
	int end, bool multi_plane)
{
	struct list_node *start_node;
	int i;
	int count = multi_plane ? 2 : 1;

	start_node = seek_list_item(base_node, start * count);
	for (i = 0; i < end - start + 1; i++) {
		start_node = free_multi_write_work(
			list_ctrl, start_node, count);
	}
	return start_node;
}


static int do_multi_work_erase(
	struct list_node *cur, int count)
{
	int i, status = 0;
	struct list_node *item;
	struct nand_work *work;
	struct mtk_nand_chip_operation *ops0, *ops1;
	bool multi_plane = false;

	ops0 = NULL;
	ops1 = NULL;

	item = cur;
	for (i = 0; i < count; i++)  {
		if (item == NULL) {
			nand_err("i:%d, NULL item\n", i);
			return -ENANDERASE;
		}
		work = get_list_work(item);
		if (i == 0)
			ops0 = &work->ops;
		else if (i == 1)
			ops1 = &work->ops;

		if (i+1 >= count)
			break;
		item = item->next;
	}
	if (ops1 != NULL)
		multi_plane = are_on_diff_planes(ops0->block, ops1->block);

	if (multi_plane)
		return mtk_nand_erase_blocks(ops0, ops1);

	status = mtk_nand_erase_blocks(ops0, NULL);
	if (status)
		return status;
	if (ops1 != NULL)
		status = mtk_nand_erase_blocks(ops1, NULL);

	return status;
}

static int do_multi_work_write(struct list_node *cur, int count)
{
	int i, status = 0;
	struct list_node *item;
	struct nand_work *work;
	struct mtk_nand_chip_operation *ops0, *ops1;
	bool multi_plane = false;
	bool multi_plane_en;

	ops0 = NULL;
	ops1 = NULL;
	item = cur;
	for (i = 0; i < count; i++)  {
		if (item == NULL) {
			nand_err("NULL item");
			return -ENANDWRITE;
		}
		work = get_list_work(item);
		if (i == 0)
			ops0 = &work->ops;
		else if (i == 1)
			ops1 = &work->ops;

		if (i+1 >= count)
			break;
		item = item->next;
	}

	multi_plane_en = (gn_devinfo.advancedmode & MULTI_PLANE) ? true : false;

	if (multi_plane_en) {
		if (ops1 != NULL)
			multi_plane = are_on_diff_planes(ops0->block, ops1->block);

		if (multi_plane)
			return mtk_nand_write_pages(ops0, ops1);
	}

	if (multi_plane_en)
		gn_devinfo.advancedmode &= (~MULTI_PLANE);

	status = mtk_nand_write_pages(ops0, NULL);
	if (status)
		goto OUT;
	if (ops1 != NULL)
		status = mtk_nand_write_pages(ops1, NULL);

OUT:
	if (multi_plane_en)
		gn_devinfo.advancedmode |= MULTI_PLANE;
	return status;
}

static struct list_node *do_mlc_multi_plane_write(
	struct mtk_nand_chip_info *info,
	struct worklist_ctrl *list_ctrl,
	struct list_node *start_node,
	int count)
{
	struct list_node *item;
	int status = 0;
	struct mtd_info *mtd = data_info->mtd;
	struct nand_chip *chip = mtd->priv;

	nand_debug("do_mlc_multi_plane_write enter");
	if (count != info->max_keep_pages) {
		nand_err("error:count:%d max:%d",
			count, info->max_keep_pages);
		return start_node;
	}

	if (start_node == NULL) {
		nand_err("error:start node is NULL!");
		return NULL;
	}

	item = start_node;

	/* Grab the lock and see if the device is available */
	nand_get_device(mtd, FL_WRITING);
	/* Select the NAND device */
	chip->select_chip(mtd, 0);

	status = do_multi_work_write(item, count);
	/* Deselect and wake up anyone waiting on the device */
	chip->select_chip(mtd, -1);
	mtk_nand_release_device(mtd);

	call_multi_work_callback(item,
			count, status);

	return free_multi_write_work(
			list_ctrl, item, count);
}

static int do_tlc_page_group_write(struct list_node *cur, int start,
	int step, enum NFI_TLC_PG_CYCLE program_cycle, enum TLC_MULTI_PROG_MODE prog_mode)
{
	struct list_node *item;
	int status = 0, i;
	int count;
	struct mtd_info *mtd = data_info->mtd;
	struct nand_chip *chip = mtd->priv;


	item = seek_list_item(cur, start * step);

	nand_debug("start:%d, item:%p", start, item);

	/* Grab the lock and see if the device is available */
	nand_get_device(mtd, FL_WRITING);
	/* Select the NAND device */
	chip->select_chip(mtd, 0);

	gn_devinfo.tlcControl.slcopmodeEn = FALSE;
	tlc_program_cycle = program_cycle;

	count = (prog_mode == MULTI_BLOCK) ? 2 : 1;
	item = (prog_mode == BLOCK1_ONLY) ? item->next : item;

	for (i = 0; i < 3; i++) {
		status = do_multi_work_write(item, count);
		if (status)
			break;
		if (i >= 2)
			break;
		if (step == 1)
			item = item->next;
		else
			item = item->next->next;
	}

	/* Deselect and wake up anyone waiting on the device */
	chip->select_chip(mtd, -1);
	mtk_nand_release_device(mtd);

	return status;
}

static int do_tlc_wl_write(struct mtk_nand_chip_info *info, struct list_node *start_node,
		enum TLC_MULTI_PROG_MODE prog_mode)
{
	struct nand_work *work;
	struct list_node *item;
	struct mtk_nand_chip_operation *ops;
	int status, step;

	item = start_node;
	work = get_list_work(item);
	ops = &work->ops;

	if ((ops->page % MTK_TLC_DIV) != 0) {
		nand_err("Not TLC(3 pages) alsigned ops->page:0x%x",
			ops->page);
		return -1;
	}

	step = (gn_devinfo.advancedmode & MULTI_PLANE) ? 2 : 1;

	if (ops->page == 0) {
		status = do_tlc_page_group_write(item, 0, step, PROGRAM_1ST_CYCLE, prog_mode);
		if (status)
			return status;
		status = do_tlc_page_group_write(item, 3, step, PROGRAM_1ST_CYCLE, prog_mode);
		if (status)
			return status;
		status = do_tlc_page_group_write(item, 0, step, PROGRAM_2ND_CYCLE, prog_mode);
		if (status)
			return status;
	}


	if ((ops->page/3 + 2) < (info->data_page_num/3)) {
		status = do_tlc_page_group_write(item, 6, step, PROGRAM_1ST_CYCLE, prog_mode);
		if (status)
			return status;
	}

	if ((ops->page/3 + 1) < (info->data_page_num/3)) {
		status = do_tlc_page_group_write(item, 3, step, PROGRAM_2ND_CYCLE, prog_mode);
		if (status)
			return status;
	}

	status = do_tlc_page_group_write(item, 0, step, PROGRAM_3RD_CYCLE, prog_mode);
	if (status)
		return status;

	if (ops->page == (info->data_page_num - 9)) {
		status = do_tlc_page_group_write(item, 6, step, PROGRAM_2ND_CYCLE, prog_mode);
		if (status)
			return status;

		status = do_tlc_page_group_write(item, 3, step, PROGRAM_3RD_CYCLE, prog_mode);
		if (status)
			return status;

		status = do_tlc_page_group_write(item, 6, step, PROGRAM_3RD_CYCLE, prog_mode);
		if (status)
			return status;
	}

	return status;
}

static struct list_node *do_tlc_write(
	struct mtk_nand_chip_info *info,
	struct worklist_ctrl *list_ctrl,
	struct list_node *start_node,
	int count)
{
	struct nand_work *work, *work1;
	struct mtk_nand_chip_operation *ops;
	struct list_node *item;
	int status;
	bool multi_plan = false;
	enum TLC_MULTI_PROG_MODE prog_mode;

	if (count != info->max_keep_pages) {
		nand_err("error:count:%d max:%d",
			count, info->max_keep_pages);
		return start_node;
	}

	if (start_node == NULL) {
		nand_err("error:start node is NULL!");
		return NULL;
	}

	item = start_node;

	multi_plan = (gn_devinfo.advancedmode & MULTI_PLANE) ? true : false;

	work = get_list_work(item);
	ops = &work->ops;

	nand_debug("page num:%d", ops->page);

	if ((ops->page % MTK_TLC_DIV) != 0) {
		nand_err("Not TLC(3 pages) alsigned ops->page:0x%x", ops->page);
		return NULL;
	}

	if (multi_plan) {
		work1 = get_list_work(item->next);
		prog_mode = are_on_diff_planes(work->ops.block, work1->ops.block) ? MULTI_BLOCK : BLOCK0_ONLY;
	} else {
		prog_mode = BLOCK0_ONLY;
	}

	PFM_BEGIN();
	status = do_tlc_wl_write(info, item, prog_mode);
	PFM_END_OP(MNTL_PART_TLC_BLOCK_SW, count);

	if (!status && multi_plan && (prog_mode == BLOCK0_ONLY))
		status = do_tlc_wl_write(info, item, BLOCK1_ONLY);

	call_tlc_page_group_callback(item, 0, 2, multi_plan, status);

	if (multi_plan)
		work = get_list_work(seek_list_item(item, 8 * 2));
	else
		work = get_list_work(seek_list_item(item, 8));

	item = free_tlc_page_group_work(list_ctrl, item,
						0, 2, multi_plan);

	ops = &work->ops;

	if (ops->page == (info->data_page_num - 1)) {
		/*3~5*/
		call_tlc_page_group_callback(item, 0, 2,
						multi_plan, status);
		/*6~8*/
		call_tlc_page_group_callback(item, 3, 5,
						multi_plan, status);
		/*3~5*/
		item = free_tlc_page_group_work(list_ctrl, item,
						0, 2, multi_plan);
		/*6~8*/
		item = free_tlc_page_group_work(list_ctrl, item,
						0, 2, multi_plan);
	}

	return item;
}


static unsigned int complete_erase_count(struct mtk_nand_chip_info *info,
					 struct worklist_ctrl *list_ctrl, int total)
{
	struct list_node *head;
	struct nand_work *work0, *work1;
	bool is2p;

	if (!total)
		return 0;

	if (!is_multi_plane(info))
		return 1;

	/* multi-plane */
	head = &list_ctrl->head;
	if (total < info->plane_num) {
		work0 = get_list_work(seek_list_item(head, total));
		return (!work0->ops.more) ? total : 0;
	}

	/* total >= plane_num */
	work0 = get_list_work(head->next);
	if (!head->next->next)
		nand_err("second NULL, total:%d, work0->block:%d\n", total,
				work0->ops.block);
	work1 = get_list_work(head->next->next);
	is2p = are_on_diff_planes(work0->ops.block, work1->ops.block);
	return is2p ? info->plane_num : 1;
}

static unsigned int complete_slc_write_count(struct mtk_nand_chip_info *info,
				struct worklist_ctrl *list_ctrl, int total)
{
	struct list_node *head;
	struct nand_work *work0, *work1;
	bool multi_op;

	if (!total)
		return 0;

	if (!is_multi_plane(info))
		return 1;

	/* multi-plane*/
	if (total < info->plane_num)
		return 0;

	/* total >= plane_num */
	head = &list_ctrl->head;
	work0 = get_list_work(head->next);
	work1 = get_list_work(head->next->next);
	multi_op = can_ops_multi_plane(&work0->ops, &work1->ops);
	return multi_op ? 2 : 1;
}

static unsigned int complete_write_count(struct mtk_nand_chip_info *info,
				struct worklist_ctrl *list_ctrl, int total)
{
	return (total >= info->max_keep_pages) ? info->max_keep_pages : 0;
}

static int init_list_ctrl(struct worklist_ctrl *list_ctrl,
		enum worklist_type type, get_ready_count get_ready_count_func,
			process_list_data process_data_func)
{
	mutex_init(&list_ctrl->sync_lock);
	spin_lock_init(&list_ctrl->list_lock);
	list_ctrl->type = type;
	list_ctrl->total_num = 0;
	list_ctrl->head.next = NULL;
	list_ctrl->get_ready_count_func = get_ready_count_func;
	list_ctrl->process_data_func = process_data_func;

	return 0;
}

static inline bool block_num_is_valid(
	struct mtk_nand_chip_info *info,
	unsigned int block)
{
	return (block >= 0 && block <
		(info->data_block_num + info->log_block_num));
}

static inline bool block_page_num_is_valid(
	struct mtk_nand_chip_info *info,
	unsigned int block, unsigned int page)
{
	if (!block_num_is_valid(info, block))
		return false;

	if (page < 0)
		return false;

#ifdef ALL_SLC_BUFFER
	if ((!is_slc_block(info, block) &&
		page < info->data_page_num) ||
		(is_slc_block(info, block) &&
		page < info->log_page_num))
		return true;
	else
		return false;
#else
	if ((block < info->data_block_num &&
		page < info->data_page_num) ||
		(block >= info->data_block_num &&
		page < info->log_page_num))
		return true;
	else
		return false;
#endif
}

static struct list_node *mtk_nand_do_erase(struct mtk_nand_chip_info *info,
		struct worklist_ctrl *list_ctrl, int count)
{
	struct list_node *node;
	int i, status, op_cnt;

	i = 0;
	node = list_ctrl->head.next;

	if (!count)
		return NULL;

	if (count != 1 && count != 2)
		nand_err("count:%d\n", count);

	if (node == NULL) {
		nand_err("head next null, count:%d\n", count);
		dump_stack();
		return NULL;
	}
	while ((i < count) && node) {
		op_cnt = ((count-i) > info->plane_num) ? info->plane_num : (count-i);
		nand_debug(" i:%d, op_cnt:%d", i, op_cnt);
		status = do_multi_work_erase(node, op_cnt);
		call_multi_work_callback(node, op_cnt, status);
		node = free_multi_erase_work(list_ctrl, node, op_cnt);
		i += op_cnt;
	}

	nand_debug("mtk_nand_do_erase done\n");

	return node;
}

static struct list_node *mtk_nand_do_slc_write(
	struct mtk_nand_chip_info *info,
		struct worklist_ctrl *list_ctrl, int count)
{
	struct mtd_info *mtd = data_info->mtd;
	struct nand_chip *chip = mtd->priv;
	struct mtk_nand_chip_operation *ops[2];
	struct nand_work *work;
	struct list_node *node;
	int i, status;

	if (!count)
		return NULL;

	if (count != 1 && count != 2) {
		nand_err("count not 1 or 2, count:%d\n", count);
		return NULL;
	}

	ops[1] = NULL;
	node = list_ctrl->head.next;
	for (i = 0; i < count; i++) {
		work = get_list_work(node);
		ops[i] = &work->ops;
		node = node->next;
	}

	/* Grab the lock and see if the device is available */
	nand_get_device(mtd, FL_WRITING);
	/* Select the NAND device */
	chip->select_chip(mtd, 0);
	PFM_BEGIN();
	status = mtk_nand_write_pages(ops[0], ops[1]);
	PFM_END_OP(MNTL_PART_SLC_WRITE, info->data_page_size);
	/* Deselect and wake up anyone waiting on the device */
	chip->select_chip(mtd, -1);
	mtk_nand_release_device(mtd);

	call_multi_work_callback(list_ctrl->head.next, count, status);
	node = free_multi_write_work(list_ctrl, list_ctrl->head.next, count);
	return node;
}


static struct list_node *mtk_nand_do_write(struct mtk_nand_chip_info *info,
		struct worklist_ctrl *list_ctrl, int count)
{
	struct list_node *node;

	node = list_ctrl->head.next;
	if (info->types == NAND_FLASH_MLC)
		return do_mlc_multi_plane_write(info, list_ctrl, node, count);

	if (info->types == NAND_FLASH_TLC)
		return do_tlc_write(info, list_ctrl, node, count);

	nand_err("unhandled work!\n");
	return NULL;
}

static int mtk_nand_process_list(
	struct mtk_nand_chip_info *info,
	struct worklist_ctrl *list_ctrl,
	int sync_num)
{
	struct list_node *item;
	unsigned int total, ready_cnt;
	unsigned int process_cnt = 0, left_cnt;

	mutex_lock(&list_ctrl->sync_lock);

	total = get_list_work_cnt(list_ctrl);

	if (((sync_num > 0) && (total < sync_num)) || (total == 0))
		goto OUT;

	do {
		ready_cnt = list_ctrl->get_ready_count_func(info, list_ctrl,
								total - process_cnt);
		if (!ready_cnt)
			break;

		item = list_ctrl->process_data_func(info, list_ctrl, ready_cnt);
		process_cnt += ready_cnt;
	} while (item);

	if ((sync_num == -1) && total > process_cnt) {
		/*only force sync erase and slc write work*/
		if (list_ctrl->type == LIST_NS_WRITE)
			goto OUT;

		left_cnt = total - process_cnt;
		if (left_cnt > 1)
			nand_err("left_cnt:%d\n", left_cnt);
		list_ctrl->process_data_func(info, list_ctrl, left_cnt);
		process_cnt += left_cnt;
	}

OUT:
	mutex_unlock(&list_ctrl->sync_lock);
	return process_cnt;
}


static int mtk_nand_work_thread(void *u)
{
	struct mtk_nand_chip_info *info = &data_info->chip_info;
	struct worklist_ctrl *elist_ctrl = &data_info->elist_ctrl;
	struct worklist_ctrl *swlist_ctrl = &data_info->swlist_ctrl;
	struct worklist_ctrl *wlist_ctrl = &data_info->wlist_ctrl;

	pr_info("%s started, PID %d\n", __func__, task_pid_nr(current));
	for (;;) {
		if (kthread_should_stop())
			break;

		wait_for_completion(&data_info->ops_ctrl);

		mtk_nand_process_list(info, elist_ctrl, 0);
		mtk_nand_process_list(info, swlist_ctrl, 0);
		mtk_nand_process_list(info, wlist_ctrl, 0);

	}

	pr_info("%s exit, PID %d\n", __func__, task_pid_nr(current));

	return 0;
}

static void mtk_nand_dump_bbt_info(struct mtk_nand_chip_bbt_info *chip_bbt)
{
	unsigned int i;

	nand_info("bad_block_num:%d, initial_bad_num:%d", chip_bbt->bad_block_num,
		chip_bbt->initial_bad_num);
	for (i = 0; i < chip_bbt->bad_block_num; i++)
		nand_info("bad_index:%d", chip_bbt->bad_block_table[i]);
}

int mtk_chip_bbt_init(struct data_bmt_struct *data_bmt)
{
	struct mtk_nand_chip_bbt_info *chip_bbt = &data_info->chip_bbt;
	unsigned int i, initial_bad_num;
	unsigned int initial_bad, ftl_mark_bad;
	u16 bad_block;

	chip_bbt->bad_block_num = data_bmt->bad_count;
	chip_bbt->initial_bad_num = data_bmt->bad_count;
	if (data_bmt->bad_count > BAD_BLOCK_MAX_NUM) {
		nand_err("bad block count > max(%d)", BAD_BLOCK_MAX_NUM);
		return -1;
	}

	initial_bad_num = 0;
	for (i = 0; i < data_bmt->bad_count; i++)
		if (data_bmt->entry[i].flag != FTL_MARK_BAD)
			initial_bad_num++;

	chip_bbt->initial_bad_num = initial_bad_num;
	initial_bad = 0;
	ftl_mark_bad = initial_bad_num;

	for (i = 0; i < data_bmt->bad_count; i++) {
		bad_block = data_bmt->entry[i].bad_index - data_bmt->start_block;
		if (data_bmt->entry[i].flag != FTL_MARK_BAD) {
			chip_bbt->bad_block_table[initial_bad] = bad_block;
			initial_bad++;
		} else {
			chip_bbt->bad_block_table[ftl_mark_bad] = bad_block;
			ftl_mark_bad++;
		}
	}

	for (i = data_bmt->bad_count; i < BAD_BLOCK_MAX_NUM; i++)
		chip_bbt->bad_block_table[i] = 0xffff;

	mtk_nand_dump_bbt_info(chip_bbt);

	return 0;
}

static int mtk_chip_info_init(struct mtk_nand_chip_info *chip_info)
{
	unsigned int page_per_block;

	page_per_block = gn_devinfo.blocksize*1024/gn_devinfo.pagesize;

	chip_info->log_block_num = data_info->partition_info.total_block
		*data_info->partition_info.slc_ratio/100;

	chip_info->data_page_num = page_per_block;
	chip_info->data_page_size = gn_devinfo.pagesize;
	chip_info->data_block_size = gn_devinfo.blocksize*1024;
	chip_info->log_page_size = gn_devinfo.pagesize;

	if (gn_devinfo.NAND_FLASH_TYPE == MTK_NAND_FLASH_TLC) {
		chip_info->log_page_num =
			page_per_block/MTK_TLC_DIV;
		chip_info->log_block_size =
			chip_info->data_block_size/MTK_TLC_DIV;
	} else {

		chip_info->log_page_num =
			page_per_block/MTK_MLC_DIV;
		chip_info->log_block_size =
			chip_info->data_block_size/MTK_MLC_DIV;
	}

	if (gn_devinfo.advancedmode & MULTI_PLANE) {
		chip_info->log_block_num >>= 1;
		chip_info->log_block_num <<= 1;
	}

	chip_info->data_block_num =
		data_info->partition_info.total_block - chip_info->log_block_num;

	if (gn_devinfo.advancedmode & MULTI_PLANE) {
		chip_info->data_block_num >>= 1;
		chip_info->data_block_num <<= 1;
	}

	chip_info->data_oob_size = (gn_devinfo.pagesize >> host->hw->nand_sec_shift)
					* host->hw->nand_fdm_size;
	chip_info->log_oob_size = chip_info->data_oob_size;

	chip_info->slc_ratio = data_info->partition_info.slc_ratio;
	chip_info->start_block = data_info->partition_info.start_block;

	chip_info->sector_size_shift = 10;

	if (gn_devinfo.NAND_FLASH_TYPE == MTK_NAND_FLASH_TLC)
		chip_info->max_keep_pages = (gn_devinfo.advancedmode & MULTI_PLANE)?18:9;
	else
		chip_info->max_keep_pages = (gn_devinfo.advancedmode & MULTI_PLANE)?2:1;

	if (gn_devinfo.NAND_FLASH_TYPE == MTK_NAND_FLASH_TLC)
		chip_info->types = MTK_NAND_FLASH_TLC;
	else
		chip_info->types = MTK_NAND_FLASH_MLC;

	chip_info->plane_mask = 0x1;
	chip_info->plane_num = (gn_devinfo.advancedmode & MULTI_PLANE)?2:1;
	chip_info->chip_num = 1;

	if (gn_devinfo.advancedmode & MULTI_PLANE)
		chip_info->option = MTK_NAND_PLANE_MODE_SUPPORT |
					MTK_NAND_MULTI_READ_DIFFERENT_OFFSET;

	chip_info->data_pe = gn_devinfo.lifepara.data_pe;
	chip_info->log_pe = gn_devinfo.lifepara.slc_pe;
	chip_info->info_version = MTK_NAND_CHIP_INFO_VERSION;

	return 0;
}

static void mtk_nand_dump_chip_info(struct mtk_nand_chip_info *chip_info)
{
	nand_debug("mtk_nand_chip_info dump info here");
	nand_debug("data_block_num:0x%x", chip_info->data_block_num);
	nand_debug("data_page_num:0x%x", chip_info->data_page_num);
	nand_debug("data_page_size:0x%x", chip_info->data_page_size);
	nand_debug("data_oob_size:0x%x", chip_info->data_oob_size);
	nand_debug("data_block_size:0x%x", chip_info->data_block_size);
	nand_debug("log_block_num:0x%x", chip_info->log_block_num);
	nand_debug("log_page_num:0x%x", chip_info->log_page_num);
	nand_debug("log_page_size:0x%x", chip_info->log_page_size);
	nand_debug("log_block_size:0x%x", chip_info->log_block_size);
	nand_debug("log_oob_size:0x%x", chip_info->log_oob_size);
	nand_debug("slc_ratio:0x%x", chip_info->slc_ratio);
	nand_debug("start_block:0x%x", chip_info->start_block);
	nand_debug("sector_size_shift:0x%x", chip_info->sector_size_shift);
	nand_debug("max_keep_pages:0x%x", chip_info->max_keep_pages);
	nand_debug("types:0x%x", chip_info->types);
	nand_debug("plane_mask:0x%x", chip_info->plane_mask);
	nand_debug("plane_num:0x%x", chip_info->plane_num);
	nand_debug("chip_num:0x%x", chip_info->chip_num);
	nand_debug("option:0x%x", chip_info->option);
}

static void mtk_nand_dump_partition_info(
		struct nand_ftl_partition_info *partition_info)
{
	nand_debug("nand_ftl_partition_info dump info here");
	nand_debug("start_block:0x%x", partition_info->start_block);
	nand_debug("total_block:0x%x", partition_info->total_block);
	nand_debug("slc_ratio:%d", partition_info->slc_ratio);
	nand_debug("slc_block:%d", partition_info->slc_block);
}

static void mtk_nand_dump_bmt_info(struct data_bmt_struct *data_bmt)
{
	unsigned int i;

	nand_debug("nand_ftl_partition_info dump info here");
	nand_debug("bad_count:0x%x", data_bmt->bad_count);
	nand_debug("start_block:0x%x", data_bmt->start_block);
	nand_debug("end_block:0x%x", data_bmt->end_block);
	nand_debug("version:%d", data_bmt->version);

	for (i = 0; i < data_bmt->bad_count; i++) {
		nand_debug("bad_index:0x%x, flag:%d",
			data_bmt->entry[i].bad_index,
			data_bmt->entry[i].flag);
	}
}



/*********	API for nand Wrapper *********/

/* struct mtk_nand_chip_info *mtk_nand_chip_init(void)
 * Init mntl_chip_info to nand wrapper, after nand driver init.
 * Return: On success, return mtk_nand_chip_info. On error, return error num.
 *//* void dump_block_bit_map(u8 *array); */
struct mtk_nand_chip_info *mtk_nand_chip_init(void)
{

	if (&data_info->chip_info != NULL) {
		/* dump_block_bit_map(data_info->chip_info.block_type_bitmap); */
		return &data_info->chip_info;
	} else
		return 0;
}
EXPORT_SYMBOL(mtk_nand_chip_init);

/* mtk_nand_chip_read_page
 * Only one page data and FDM data read, support partial read.
 *
 * @info: NAND handler
 * @data_buffer/oob_buffer: Null for no need data/oob, must contiguous address space.
 * @block/page: The block/page to read data.
 * @offset: data offset to start read, and must be aligned to sector size.
 * @size: data size to read. size <= pagesize, less than page size will partial read,
 *    and OOB is only related sectors, uncompleted refer to whole page.
 * return : 0 on success, On error, return error num.
 */
int mtk_nand_chip_read_page(struct mtk_nand_chip_info *info,
		unsigned char *data_buffer, unsigned char *oob_buffer,
		unsigned int block, unsigned int page,
		unsigned int offset, unsigned int size)
{
	int ret = 0;

	if (data_buffer == NULL) {
		nand_err("data_buffer is null");
		return -EINVAL;
	}

	if (oob_buffer == NULL) {
		nand_err("oob_buffer is null");
		return -EINVAL;
	}

	if ((offset % (1 << info->sector_size_shift) != 0)
		|| (size % (1 << info->sector_size_shift) != 0)) {
		nand_err("offset or size is invalid:offset:%d, size:%d",
			offset, size);
		return -EINVAL;
	}

	if (!block_page_num_is_valid(info, block, page)) {
		nand_err("block or page num is invalid:block:%d, page:%d",
			block, page);
		dump_stack();
		return -EINVAL;
	}

	if (mtk_isbad_block(block))
		return -ENANDBAD;

	ret = mtk_nand_read_pages(info, data_buffer,
		oob_buffer, block, page, offset, size);

	if (ret) {
		nand_err("read err:%d, block:%d, page:%d, offset:%d, size:%d\n",
			ret, block, page, offset, size);
	}

	return ret;
}
EXPORT_SYMBOL(mtk_nand_chip_read_page);

/* mtk_nand_chip_read_pages
 * Read multiple pages of data/FDM at once. Support partial read.
 *
 * The driver can choose the number of pages it actually read.
 * Driver only guarantee to read at least one page. Caller must handle
 * unread pages by itself.
 *
 * If there are any error in 2nd or following pages, just return numbers
 * of page read without any error. Driver shouldn't retry/re-read other pages.
 *
 * @info: NAND handler
 * @page_num: the page numbers to read.
 * @param: parameters for each page read
 * return : >0 number of pages read without any error (including ENANDFLIPS)
 *          On first page read error, return error number.
 */
int mtk_nand_chip_read_multi_pages(struct mtk_nand_chip_info *info, int page_num,
			struct mtk_nand_chip_read_param *param)

{
	struct mtk_nand_chip_read_param *p = param;
	int ret;

	if (!page_num) {
		nand_err("page_num is 0, so return 0\n");
		dump_stack();
		return 0;
	}

	if (page_num == 1 || !is_multi_read_support(info)) {
		ret = mtk_nand_chip_read_page(info, p->data_buffer, p->oob_buffer,
					p->block, p->page, p->offset, p->size);
		return ret ? ret : 1;
	}

	if (!can_multi_plane(p->block, p->page, p[1].block, p[1].page)) {
		ret = mtk_nand_chip_read_page(info, p->data_buffer, p->oob_buffer,
					p->block, p->page, p->offset, p->size);
		return ret ? ret : 1;
	}
#ifdef SUPPORT_MICRON_DEVICE
	return mtk_nand_read_multi_pages(info, 2, param);
#else
	ret = mtk_nand_chip_read_page(info, p->data_buffer, p->oob_buffer,
					p->block, p->page, p->offset, p->size);
		return ret ? ret : 1;
#endif
}
EXPORT_SYMBOL(mtk_nand_chip_read_multi_pages);

/* mtk_nand_chip_write_page
 * write page API. Only one page data write, async mode.
 * Just return 0 and add to write worklist as below:
 *  a) For TLC WL write, NAND handler call nand driver WL write function.
 *  b) For Multi-plane program, if more_page is TRUE,
 *  wait for the next pages write and do multi-plane write on time.
 *  c) For cache  program, driver will depend on more_page flag for TLC program,
 *  can not used for SLC/MLC program.
 * after Nand driver erase/write operation, callback function will be done.
 *
 * @info: NAND handler
 * @data_buffer/oob_buffer: must contiguous address space.
 * @block/page: The block/page to write data.
 * @more_page: for TLC WL write and multi-plane program operation.
 *    if more_page is true, driver will wait complete operation and call driver function.
 * @*callback: callback for wrapper, called after driver finish the operation.
 * @*userdata : for callback function
 * return : 0 on success, On error, return error num casted by ERR_PTR
 */
int mtk_nand_chip_write_page(struct mtk_nand_chip_info *info,
		unsigned char *data_buffer, unsigned char *oob_buffer,
		unsigned int block, unsigned int page, bool more_page,
		mtk_nand_callback_func callback, void *userdata)
{
	/* Add to worklist here*/
	struct worklist_ctrl *list_ctrl;
	struct nand_work *work;
	struct mtk_nand_chip_operation *ops;
	int total_num;
	unsigned int max_keep_pages;
	int page_num;

	nand_debug("write block:%d page:%d more_page:%d\n", block, page, more_page);

	if (data_buffer == NULL) {
		nand_err("data_buffer is null");
		return -EINVAL;
	}

	if (oob_buffer == NULL) {
		nand_err("oob_buffer is null");
		return -EINVAL;
	}

	if (!block_page_num_is_valid(info, block, page)) {
		nand_err("block or page num is invalid:block:%d, page:%d",
			block, page);
		return -EINVAL;
	}
#ifdef ALL_SLC_BUFFER
	list_ctrl = is_slc_block(info, block) ?
			(&data_info->swlist_ctrl) : (&data_info->wlist_ctrl);
#else
	list_ctrl = (block < info->data_block_num) ?
			(&data_info->wlist_ctrl) : (&data_info->swlist_ctrl);
#endif
	total_num = get_list_work_cnt(list_ctrl);
#ifdef ALL_SLC_BUFFER
	max_keep_pages = is_slc_block(info, block) ?
			info->plane_num : info->max_keep_pages;
#else
	max_keep_pages = (block < info->data_block_num) ?
			info->max_keep_pages : info->plane_num;
#endif
	while (total_num >= max_keep_pages) {
		nand_debug("total_num:%d", total_num);
		mtk_nand_process_list(info,
				list_ctrl, max_keep_pages);
		total_num = get_list_work_cnt(list_ctrl);
	};

	work = kmalloc(sizeof(struct nand_work), GFP_KERNEL);
	if (work == NULL)
		return -ENOMEM;

	work->list.next = NULL;
	ops = &work->ops;

	ops->info = info;
	ops->types = MTK_NAND_OP_WRITE;
	ops->data_buffer = data_buffer;
	ops->oob_buffer = oob_buffer;
	ops->block = block;
	ops->page = page;
	ops->more = more_page;
	ops->callback = callback;
	ops->userdata = userdata;

	nand_debug("block:%d, page:%d, add(%p)", block, page, &work->list);



	add_list_node(list_ctrl, &work->list);
#ifdef ALL_SLC_BUFFER
	page_num = is_slc_block(info, block) ? info->log_page_num :
						info->data_page_num;
#else

	page_num = (block < info->data_block_num) ? info->data_page_num :
						info->log_page_num;
#endif
	total_num = get_list_work_cnt(list_ctrl);
	if (total_num >= max_keep_pages || (page == page_num - 1))
		complete(&data_info->ops_ctrl);

	return 0;
}
EXPORT_SYMBOL(mtk_nand_chip_write_page);

/*
 * mtk_nand_chip_erase_block
 * Erase API for nand wrapper, async mode for erase, just return success,
 * put erase operation into write worklist.
 * After Nand driver erase/write operation, callback function will be done.
 * @block: The block to erase
 * @*callback: Callback for wrapper, called after driver finish the operation.
 * @* userdata: for callback function
 * return : 0 on success, On error, return error num casted by ERR_PTR
 */
int mtk_nand_chip_erase_block(struct mtk_nand_chip_info *info,
		unsigned int block, unsigned int more_block,
		mtk_nand_callback_func callback, void *userdata)
{
	struct worklist_ctrl *list_ctrl;
	struct nand_work *work;
	struct mtk_nand_chip_operation *ops;
	int total_num;

	nand_debug("erase block:%d more_page:%d\n", block, more_block);

	if (!block_num_is_valid(info, block)) {
		nand_err("block num is invalid:block:%d", block);
		return -EINVAL;
	}

	list_ctrl = &data_info->elist_ctrl;
	total_num = get_list_work_cnt(list_ctrl);

	while (total_num >= info->max_keep_pages) {
		nand_debug("total_num:%d", total_num);
		mtk_nand_process_list(info,
				list_ctrl, info->max_keep_pages);
		list_ctrl = &data_info->elist_ctrl;
		total_num = get_list_work_cnt(list_ctrl);
	};

	work = kmalloc(sizeof(struct nand_work), GFP_KERNEL);
	if (work == NULL)
		return -ENOMEM;

	work->list.next = NULL;
	ops = &work->ops;

	ops->info = info;
	ops->types = MTK_NAND_OP_ERASE;
	ops->block = block;
	ops->page = 0;
	ops->more = more_block;
	ops->data_buffer = NULL;
	ops->oob_buffer = NULL;
	ops->callback = callback;
	ops->userdata = userdata;

	list_ctrl = &data_info->elist_ctrl;

	nand_debug("block:%d, add(%p)",
		block, &work->list);

	add_list_node(list_ctrl, &work->list);
	total_num = get_list_work_cnt(list_ctrl);
	if (total_num >= info->plane_num)
		complete(&data_info->ops_ctrl);

	return 0;
}
EXPORT_SYMBOL(mtk_nand_chip_erase_block);

/*
 * mtk_nand_chip_sync
 * flush all async worklist to nand driver.
 * return : On success, return 0. On error, return error num
 */
int mtk_nand_chip_sync(struct mtk_nand_chip_info *info)
{
	struct worklist_ctrl *elist_ctrl, *swlist_ctrl, *wlist_ctrl;
	int ret = 0;

	nand_debug("%s enter", __func__);

	elist_ctrl = &data_info->elist_ctrl;
	swlist_ctrl = &data_info->swlist_ctrl;
	wlist_ctrl = &data_info->wlist_ctrl;

	ret = mtk_nand_process_list(info, elist_ctrl, -1);
	nand_debug(" sync erase done");
	ret |= mtk_nand_process_list(info, swlist_ctrl, -1);
	ret |= mtk_nand_process_list(info, wlist_ctrl, -1);

	nand_debug("sync done, ret:%d", ret);
	return ret;
}
EXPORT_SYMBOL(mtk_nand_chip_sync);

void mtk_nand_chip_set_blk_thread(struct task_struct *thead)
{
	data_info->blk_thread = thead;
}
EXPORT_SYMBOL(mtk_nand_chip_set_blk_thread);

/* mtk_nand_chip_bmt, bad block table maintained by driver, and read only for wrapper
 * @info: NAND handler
 * Return FTL partition's bad block table for nand wrapper.
 */
const struct mtk_nand_chip_bbt_info *mtk_nand_chip_bmt(struct mtk_nand_chip_info *info)
{

	if (&data_info->chip_bbt != NULL)
		return &data_info->chip_bbt;
	else
		return 0;
}
EXPORT_SYMBOL(mtk_nand_chip_bmt);

/* mtk_chip_mark_bad_block
 * Mark specific block as bad block,and update bad block list and bad block table.
 * @block: block address to markbad
 */
void mtk_chip_mark_bad_block(struct mtk_nand_chip_info *info, unsigned int block)
{
	struct mtk_nand_chip_bbt_info *chip_bbt = &data_info->chip_bbt;
	struct mtd_info *mtd = data_info->mtd;
	unsigned int i;
	bool success;

	pr_info("markbad block:%d", block);
	for (i = 0; i < chip_bbt->bad_block_num; i++) {
		if (block == chip_bbt->bad_block_table[i])
			return;
	}

	nand_get_device(mtd, FL_WRITING);
	chip_bbt->bad_block_table[chip_bbt->bad_block_num++] = block;

	success = update_bmt(((u64)(block + data_info->bmt.start_block)) * info->data_block_size,
			FTL_MARK_BAD, NULL, NULL);
	if (!success)
		nand_err("mark block(%d) as bad fail!!!\n", block);

	mtk_nand_release_device(mtd);
}
EXPORT_SYMBOL(mtk_chip_mark_bad_block);

int mtk_nand_data_info_init(void)
{
	data_info = kmalloc(sizeof(struct mtk_nand_data_info), GFP_KERNEL);
	if (data_info == NULL) {
		nand_err("Malloc datainfo size: 0x%x failed\n",
			(u32)(sizeof(struct mtk_nand_data_info)));
		return -ENOMEM;
	}
	memset(data_info, 0, sizeof(struct mtk_nand_data_info));

	return 0;
}

int mtk_nand_ops_init(struct mtd_info *mtd, struct nand_chip *chip)
{
	int ret = 0;

	data_info->mtd = mtd;

	ret = get_data_partition_info(&data_info->partition_info, &data_info->chip_info);
	if (ret) {
		nand_err("Get FTL partition info failed\n");
		goto err_out;
	}
	mtk_nand_dump_partition_info(&data_info->partition_info);

	ret = get_data_bmt(&data_info->bmt);
	if (ret) {
		pr_info("Get FTL bmt info failed\n");
		goto err_out;
	}
	mtk_nand_dump_bmt_info(&data_info->bmt);

	ret = mtk_chip_info_init(&data_info->chip_info);
	if (ret) {
		pr_info("Get chip info failed\n");
		goto err_out;
	}
	mtk_nand_dump_chip_info(&data_info->chip_info);

	ret = mtk_chip_bbt_init(&data_info->bmt);
	if (ret) {
		pr_info("Get chip bbt info failed\n");
		goto err_out;
	}

	init_list_ctrl(&data_info->elist_ctrl, LIST_ERASE,
			complete_erase_count, mtk_nand_do_erase);
	init_list_ctrl(&data_info->swlist_ctrl, LIST_SLC_WRITE,
			complete_slc_write_count, mtk_nand_do_slc_write);
	init_list_ctrl(&data_info->wlist_ctrl, LIST_NS_WRITE,
			complete_write_count, mtk_nand_do_write);

	init_completion(&data_info->ops_ctrl);
	data_info->nand_bgt = kthread_run(mtk_nand_work_thread,
				data_info, "nand_bgt");

	if (IS_ERR(data_info->nand_bgt)) {
		ret = PTR_ERR(data_info->nand_bgt);
		data_info->nand_bgt = NULL;
		nand_err("kthread_create failed error %d", ret);
		goto err_out;
	}

#ifdef MTK_NAND_CHIP_TEST
	mtk_chip_unit_test();
#endif
	return 0;

err_out:
	return ret;
}
void mtk_nand_update_call_trace(unsigned int *address, char type, unsigned int block, unsigned int page)
{
#ifdef __REPLAY_CALL__
	if (mntl_record_en) {
		if (call_idx == 0) {
			call_trace[call_idx].call_address = address;
			call_trace[call_idx].op_type = type;
			call_trace[call_idx].block = block;
			call_trace[call_idx].page = page;
			call_trace[call_idx].times++;
			call_idx = (call_idx + 1) % 4096;
		} else {
			if (call_trace[call_idx - 1].op_type == type && call_trace[call_idx - 1].block == block) {
				call_trace[call_idx - 1].times++;
			} else {
				call_trace[call_idx].call_address = address;
				call_trace[call_idx].op_type = type;
				call_trace[call_idx].block = block;
				call_trace[call_idx].page = page;
				call_trace[call_idx].times++;
				call_idx = (call_idx + 1) % 4096;
			}
		}
	}
#endif
}
EXPORT_SYMBOL(mtk_nand_update_call_trace);

void dump_block_bit_map(u8 *array)
{
	int i, j;
	int num;
	bool check;

	pr_info("dump_block_bit_map\n");
	for (i = 0; i < 250; i++) {
		num = array[i];
		if (num) {
			for (j = 0; j < 8; j++) {
				if (num & (1 << j)) {
					check = is_slc_block(&data_info->chip_info, ((i * 8) + j));
					pr_info("block %d is tlc check = %d\n", ((i * 8) + j), check);
				}
			}
		}
	}
}

int mtk_nand_update_block_type(int num, unsigned int *blk)
{
	struct mtd_info *mtd = data_info->mtd;
	struct mtk_nand_chip_info *info = &data_info->chip_info;
	int page_addr0, page_addr1;
	int i, status;

	page_addr1 = 0;
	nand_get_device(mtd, FL_ERASING);
	gn_devinfo.tlcControl.slcopmodeEn = TRUE;
	for (i = 0; i < num; i++) {
		page_addr0 = (blk[i]+data_info->bmt.start_block)*info->data_page_num;
			status = mtk_chip_erase_blocks(mtd, page_addr0, page_addr1);

		if (!(status & NAND_STATUS_READY))
			nand_err("SR[6] is not ready state, status:0x%x", status);
	}
	nand_release_device(mtd);

	return mntl_update_part_tab(mtd, &data_info->chip_info, num, blk);
}
EXPORT_SYMBOL(mtk_nand_update_block_type);

int os_mvg_enabled(void)
{
#if defined(CONFIG_PWR_LOSS_MTK_SPOH)
	return 1;
#else
	return 0;
#endif
}
EXPORT_SYMBOL(os_mvg_enabled);

int init_case_trigger;

int os_mvg_on_group_case(const char *gname, const char *cname)
{
#if defined(CONFIG_PWR_LOSS_MTK_SPOH)
	int trigger = mvg_trigger_get();

	if (trigger != 0 && trigger != -1)
		init_case_trigger = trigger;

	if (mvg_on_group_case(gname, cname)) {
		if (mvg_trigger()) {
			/* pr_info("[MVG_TEST]: random reset on case %s: %s\n", gname, cname); */
			mvg_set_trigger(-1);
		}
	} else {
		mvg_set_trigger(init_case_trigger);
	}
#endif
	return 0;
}
EXPORT_SYMBOL(os_mvg_on_group_case);
#if defined(CONFIG_PWR_LOSS_MTK_SPOH)
static void push_case(const char *gname, const char *cname)
{
	struct mvg_case_stack *case_stack = kmalloc(sizeof(struct mvg_case_stack), GFP_KERNEL);

	strcpy(case_stack->gname, gname);
	strcpy(case_stack->cname, cname);
	case_stack->next = cstack;
	cstack = case_stack;
}

static void pop_case(void)
{
	struct mvg_case_stack *case_stack = cstack;

	cstack = cstack->next;
	kfree(case_stack);
}

int mvg_current_case_check(void)
{
	if (cstack)
		return mvg_on_group_case(cstack->gname, cstack->cname);
	return 0;
}


#endif

int mvg_set_current_case(const char *gname, const char *cname)
{
#if defined(CONFIG_PWR_LOSS_MTK_SPOH)
	if (mvg_on_group_case(gname, cname)) {
		if (cstack) {
			pr_info("[MVG] set case reentry on %s-%s\n", gname, cname);
			return -EINVAL;
		}
		push_case(gname, cname);
	} else {
		if (cstack)
			push_case(gname, cname);
	}
#endif
	return 0;
}
EXPORT_SYMBOL(mvg_set_current_case);
int mvg_case_exit(const char *gname, const char *cname)
{
#if defined(CONFIG_PWR_LOSS_MTK_SPOH)
	int s1, s2;

	if (!cstack)
		return 0;
	s1 = strcmp(cstack->gname, gname);
	s2 = strcmp(cstack->cname, cname);

	if (s1 || s2) {
		pr_info("[MVG] case exit un-balance current %s-%s, exit %s-%s\n",
			cstack->gname, cstack->cname, gname, cname);
		return -EINVAL;
	}

	pop_case();
#endif
	return 0;
}
EXPORT_SYMBOL(mvg_case_exit);

char *os_get_task_comm(char *buf, struct task_struct *tsk)
{
	return get_task_comm(buf, tsk);
}
EXPORT_SYMBOL(os_get_task_comm);

unsigned long long os_sched_clock(void)
{
	return sched_clock();
}
EXPORT_SYMBOL(os_sched_clock);

/* mtk_is_hs
 * Return 1 if it is in home screen now.
 */
int mtk_is_hs(void)
{
	return g_i4Homescreen;
}
EXPORT_SYMBOL(mtk_is_hs);

void os_mutex_destroy(struct mutex *lock)
{
	mutex_destroy(lock);
}
EXPORT_SYMBOL(os_mutex_destroy);

void __sched
os_mutex_lock(struct mutex *lock)
{
	mutex_lock(lock);
}
EXPORT_SYMBOL(os_mutex_lock);

void dump_nfi_op(void)
{
	dump_record();
}
EXPORT_SYMBOL(dump_nfi_op);

