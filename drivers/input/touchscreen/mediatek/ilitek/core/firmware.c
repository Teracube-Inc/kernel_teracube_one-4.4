/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 * Based on TDD v7.0 implemented by Mstar & ILITEK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/fd.h>
#include <linux/file.h>
#include <linux/version.h>
#include <asm/uaccess.h>

#include "../common.h"
#include "../platform.h"
#include "config.h"
#include "i2c.h"
#include "firmware.h"
#include "flash.h"

#ifdef BOOT_FW_UPGRADE
#include "ilitek_fw.h"
#endif

extern uint32_t SUP_CHIP_LIST[];
extern int nums_chip;

/*
 * the size of two arrays is different depending on
 * which of methods to upgrade firmware you choose for.
 */
uint8_t *flash_fw = NULL;
uint8_t iram_fw[MAX_IRAM_FIRMWARE_SIZE] = {0};

/* the length of array in each sector */
int _gSectionLen = 0;
int _gTotalSector = 0;

#ifdef BOOT_FW_UPGRADE
/* The addr of block reserved for customers */
int _gStartResrv = 0x1C000;
int _gEndResrv = 0x1CFFF;
#endif

struct flash_sector
{
	uint32_t ss_addr;
	uint32_t se_addr;
	uint32_t checksum;
	uint32_t crc32;
	uint32_t dlength;
	bool data_flag;
	bool inside_block;
};

struct flash_block_info
{
	uint32_t start_addr;
	uint32_t end_addr;
};

struct flash_sector *_gFlashSector = NULL;
struct flash_block_info _gFlashBlockInfo[4];
struct core_firmware_data *core_firmware = NULL;

static uint32_t HexToDec(char *pHex, int32_t nLength)
{
	uint32_t nRetVal = 0, nTemp = 0, i;
	int32_t nShift = (nLength - 1) * 4;

	for (i = 0; i < nLength; nShift -= 4, i++)
	{
		if ((pHex[i] >= '0') && (pHex[i] <= '9'))
		{
			nTemp = pHex[i] - '0';
		}
		else if ((pHex[i] >= 'a') && (pHex[i] <= 'f'))
		{
			nTemp = (pHex[i] - 'a') + 10;
		}
		else if ((pHex[i] >= 'A') && (pHex[i] <= 'F'))
		{
			nTemp = (pHex[i] - 'A') + 10;
		}
		else
		{
			return -1;
		}

		nRetVal |= (nTemp << nShift);
	}

	return nRetVal;
}

static uint32_t calc_crc32(uint32_t start_addr, uint32_t end_addr, uint8_t *data)
{
	int i, j;
	uint32_t CRC_POLY = 0x04C11DB7;
	uint32_t ReturnCRC = 0xFFFFFFFF;
	uint32_t len = start_addr + end_addr;

	for (i = start_addr; i < len; i++)
	{
		ReturnCRC ^= (data[i] << 24);

		for (j = 0; j < 8; j++)
		{
			if ((ReturnCRC & 0x80000000) != 0)
			{
				ReturnCRC = ReturnCRC << 1 ^ CRC_POLY;
			}
			else
			{
				ReturnCRC = ReturnCRC << 1;
			}
		}
	}

	return ReturnCRC;
}

static uint32_t tddi_check_data(uint32_t start_addr, uint32_t end_addr)
{
	int timer = 500;
	uint32_t write_len = 0;
	uint32_t iram_check = 0;

	write_len = end_addr;

	DBG(DEBUG_FIRMWARE, "start = 0x%x , write_len = 0x%x, max_count = %x\n",
				start_addr, end_addr, core_firmware->max_count);

	if (write_len > core_firmware->max_count)
	{
		DBG_ERR("The length (%x) written to firmware is greater than max count (%x)\n",
				write_len, core_firmware->max_count);
		goto out;
	}

	core_config_ice_mode_write(0x041000, 0x0, 1); /* CS low */
	core_config_ice_mode_write(0x041004, 0x66aa55, 3); /* Key */

	core_config_ice_mode_write(0x041008, 0x3b, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0xFF0000) >> 16, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x00FF00) >> 8, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x0000FF), 1);

	core_config_ice_mode_write(0x041003, 0x01, 1); /* Enable Dio_Rx_dual */
	core_config_ice_mode_write(0x041008, 0xFF, 1); /* Dummy */

	/* Set Receive count */
	if(core_firmware->max_count == 0xFFFF)
		core_config_ice_mode_write(0x04100C, write_len, 2);
	else if(core_firmware->max_count == 0x1FFFF)
		core_config_ice_mode_write(0x04100C, write_len, 3);

	core_config_ice_mode_write(0x041014, 0x10000, 3); /* Checksum enable */
	core_config_ice_mode_write(0x041010, 0xFF, 1); 	/* Start to receive */

	mdelay(1);

	while (timer > 0)
	{
		mdelay(1);

		if ((core_config_read_write_onebyte(0x041014) & 0x01) == 0x01)
			break;

		timer--;
	}

	core_config_ice_mode_write(0x041000, 0x1, 1); /* CS high */

	if (timer >= 0)
	{
		/* Disable dio_Rx_dual */
		core_config_ice_mode_write(0x041003, 0x0, 1);
		iram_check = core_firmware->isCRC ? core_config_ice_mode_read(0x4101C) : core_config_ice_mode_read(0x041018);
	}
	else
	{
		DBG_ERR("TIME OUT\n");
		goto out;
	}

	return iram_check;

out:
	DBG_ERR("Failed to read Checksum/CRC from IC\n");
	return -1;

}

static void calc_verify_data(uint32_t sa, uint32_t se, uint32_t *check)
{
	uint32_t i = 0;
	uint32_t tmp_ck = 0, tmp_crc = 0;

	if(core_firmware->isCRC)
	{
		tmp_crc = calc_crc32(sa, se, flash_fw);
		*check = tmp_crc;
	}
	else
	{
		for(i = sa; i < (sa + se); i++)
			tmp_ck = tmp_ck + flash_fw[i];

		*check = tmp_ck;
	}
}

static int do_check(uint32_t start, uint32_t len)
{
	int res = 0;
	uint32_t vd = 0, lc = 0;

	calc_verify_data(start, len, &lc);
	vd = tddi_check_data(start, len);
	res = CHECK_EQUAL(vd, lc);

	DBG_INFO("%s (%x) : (%x)\n", (res < 0 ? "Invalid !" : "Correct !"), vd, lc );

	return res;
}

static int verify_flash_data(void)
{
	int i = 0, res = 0, len = 0;
	int fps = flashtab->sector;
	uint32_t ss = 0x0;

	/* check chip type with its max count */
	if(core_config->chip_id == CHIP_TYPE_ILI7807 &&
		core_config->chip_type == ILI7807_TYPE_H)
	{
			core_firmware->max_count = 0x1FFFF;
			core_firmware->isCRC = true;
	}

	for(i = 0; i < _gSectionLen + 1; i++)
	{
		if(_gFlashSector[i].data_flag)
		{
			if(ss > _gFlashSector[i].ss_addr || len == 0)
				ss = _gFlashSector[i].ss_addr;

			len = len + _gFlashSector[i].dlength;

			/* if larger than max count, then committing data to check */
			if(len >= (core_firmware->max_count - fps))
			{
				res = do_check(ss, len);
				if(res < 0)
					goto out;

				ss = _gFlashSector[i].ss_addr;
				len = 0;
			}
		}
		else
		{
			/* split flash sector and commit the last data to fw */
			if(len != 0)
			{
				res = do_check(ss, len);
				if(res < 0)
					goto out;

				ss = _gFlashSector[i].ss_addr;
				len = 0;
			}
		}
	}

	/* it might be lower than the size of sector if calc the last array. */
	if(len != 0 && res != -1)
		res = do_check(ss, core_firmware->end_addr - ss);

out:
	return res;
}

static int do_program_flash(uint32_t start_addr)
{
	int res = 0;
	uint32_t k;
	uint8_t buf[512] = {0};

	res = core_flash_write_enable();
	if (res < 0)
		goto out;

	core_config_ice_mode_write(0x041000, 0x0, 1); /* CS low */
	core_config_ice_mode_write(0x041004, 0x66aa55, 3); /* Key */

	core_config_ice_mode_write(0x041008, 0x02, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0xFF0000) >> 16, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x00FF00) >> 8, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x0000FF), 1);

	buf[0] = 0x25;
	buf[3] = 0x04;
	buf[2] = 0x10;
	buf[1] = 0x08;

	for (k = 0; k < flashtab->program_page; k++)
	{
		if (start_addr + k <= core_firmware->end_addr)
			buf[4 + k] = flash_fw[start_addr + k];
		else
			buf[4 + k] = 0xFF;
	}

	if (core_i2c_write(core_config->slave_i2c_addr, buf, flashtab->program_page + 4) < 0)
	{
		DBG_ERR("Failed to write data at start_addr = 0x%X, k = 0x%X, addr = 0x%x\n",
			start_addr, k, start_addr + k);
		res = -EIO;
		goto out;
	}

	core_config_ice_mode_write(0x041000, 0x1, 1); /* CS high */

	res = core_flash_poll_busy();
	if (res < 0)
		goto out;

	core_firmware->update_status = (start_addr * 101) / core_firmware->end_addr;

	/* holding the status until finish this upgrade. */
	if(core_firmware->update_status > 90)
		core_firmware->update_status = 90;

	printk("%cUpgrading firmware ... start_addr = 0x%x, %02d%c", 0x0D, start_addr, core_firmware->update_status, '%');

out:
	return res;
}

static int flash_program_sector(void)
{
	int i, j, res = 0;

	for(i = 0; i < _gSectionLen + 1; i++)
	{
		/*
		 * If running the boot stage, fw will only be upgrade data with the flag of block,
		 * otherwise data with its flag will be programed.
		 */
		if(core_firmware->isboot)
		{
			if(!_gFlashSector[i].inside_block)
				continue;
		}
		else
		{
			if(!_gFlashSector[i].data_flag)
				continue;
		}

		/* programming flash by its page size */
		for(j = _gFlashSector[i].ss_addr; j < _gFlashSector[i].se_addr; j+= flashtab->program_page)
		{
			if(j > core_firmware->end_addr)
				goto out;

			res = do_program_flash(j);
			if(res < 0)
				goto out;
		}
	}

out:
	return res;
}

static int do_erase_flash(uint32_t start_addr)
{
	int res = 0;
	uint32_t temp_buf = 0;

	res = core_flash_write_enable();
	if (res < 0)
	{
		DBG_ERR("Failed to config write enable\n");
		goto out;
	}

	core_config_ice_mode_write(0x041000, 0x0, 1); /* CS low */
	core_config_ice_mode_write(0x041004, 0x66aa55, 3); /* Key */

	core_config_ice_mode_write(0x041008, 0x20, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0xFF0000) >> 16, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x00FF00) >> 8, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x0000FF), 1);

	core_config_ice_mode_write(0x041000, 0x1, 1); /* CS high */

	mdelay(1);

	res = core_flash_poll_busy();
	if (res < 0)
		goto out;

	core_config_ice_mode_write(0x041000, 0x0, 1); /* CS low */
	core_config_ice_mode_write(0x041004, 0x66aa55, 3); /* Key */

	core_config_ice_mode_write(0x041008, 0x3, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0xFF0000) >> 16, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x00FF00) >> 8, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x0000FF), 1);
	core_config_ice_mode_write(0x041008, 0xFF, 1);

	temp_buf = core_config_read_write_onebyte(0x041010);
	if (temp_buf != 0xFF)
	{
		DBG_ERR("Failed to erase data(0x%x) at 0x%x\n", temp_buf, start_addr);
		res = -EINVAL;
		goto out;
	}

	core_config_ice_mode_write(0x041000, 0x1, 1); /* CS high */

	DBG(DEBUG_FIRMWARE, "Earsing data at start addr: %x \n", start_addr);

out:
	return res;
}

static int flash_erase_sector(void)
{
	int i, res = 0;

	for(i = 0; i < _gTotalSector; i++)
	{
		if(core_firmware->isboot)
		{
			if(!_gFlashSector[i].inside_block)
				continue;
		}
		else
		{
			if(!_gFlashSector[i].data_flag && !_gFlashSector[i].inside_block)
				continue;
		}

		res = do_erase_flash(_gFlashSector[i].ss_addr);
		if(res < 0)
			goto out;
	}

out:
	return res;
}

static int iram_upgrade(void)
{
	int i, j, res = 0;
	uint8_t buf[512];
	int upl = flashtab->program_page;

	/* doing reset for erasing iram data before upgrade it. */
	ilitek_platform_tp_hw_reset(true);

	mdelay(1);

	DBG_INFO("Upgrade firmware written data into IRAM directly\n");

	res = core_config_ice_mode_enable();
	if (res < 0)
	{
		DBG_ERR("Failed to enter ICE mode, res = %d\n", res);
		return res;
	}

	mdelay(20);

	core_config_reset_watch_dog();

	DBG(DEBUG_FIRMWARE, "nStartAddr = 0x%06X, nEndAddr = 0x%06X, nChecksum = 0x%06X\n",
		core_firmware->start_addr, core_firmware->end_addr, core_firmware->checksum);

	/* write hex to the addr of iram */
	DBG_INFO("Writting data into IRAM ...\n");
	for (i = core_firmware->start_addr; i < core_firmware->end_addr; i += upl)
	{
		if ((i + 256) > core_firmware->end_addr)
		{
			upl = core_firmware->end_addr % upl;
		}

		buf[0] = 0x25;
		buf[3] = (char)((i & 0x00FF0000) >> 16);
		buf[2] = (char)((i & 0x0000FF00) >> 8);
		buf[1] = (char)((i & 0x000000FF));

		for (j = 0; j < upl; j++)
			buf[4 + j] = iram_fw[i + j];

		if (core_i2c_write(core_config->slave_i2c_addr, buf, upl + 4))
		{
			DBG_ERR("Failed to write data via i2c, address = 0x%X, start_addr = 0x%X, end_addr = 0x%X\n",
					 (int)i, (int)core_firmware->start_addr, (int)core_firmware->end_addr);
			res = -EIO;
			return res;
		}

		core_firmware->update_status = (i * 101) / core_firmware->end_addr;
		printk("%cupgrade firmware(ap code), %02d%c", 0x0D, core_firmware->update_status, '%');

		mdelay(3);
	}

	/* ice mode code reset */
	DBG_INFO("Doing code reset ...\n");
	core_config_ice_mode_write(0x40040, 0xAE, 1);
	core_config_ice_mode_write(0x40040, 0x00, 1);

	mdelay(10);

	core_config_ice_mode_disable();

	/*TODO: check iram status */

	return res;
}

static int tddi_fw_upgrade(bool isIRAM)
{
	int res = 0;

	if (isIRAM)
	{
		res = iram_upgrade();
		return res;
	}

	ilitek_platform_tp_hw_reset(true);

	DBG_INFO("Enter to ICE Mode\n");

	res = core_config_ice_mode_enable();
	if (res < 0)
	{
		DBG_ERR("Failed to enable ICE mode\n");
		goto out;
	}

	mdelay(5);

	/*
	 * This command is used to fix the bug of spi clk in 7807F-AB
	 * while operating with flash.
	 */
	if (core_config->chip_id == CHIP_TYPE_ILI7807
			&& core_config->chip_type == ILI7807_TYPE_F_AB)
	{
		res = core_config_ice_mode_write(0x4100C, 0x01, 1);
		if (res < 0)
			goto out;
	}

	mdelay(25);

	/* It's not necessary to disable WTD if you're using 9881 */
	if (core_config->chip_id != CHIP_TYPE_ILI9881)
		core_config_reset_watch_dog();

	/* Disable flash protection from being written */
	core_flash_enable_protect(false);

	res = flash_erase_sector();
	if(res < 0)
	{
		DBG_ERR("Failed to erase flash\n");
		goto out;
	}

	mdelay(1);

	res = flash_program_sector();
	if(res < 0)
	{
		DBG_ERR("Failed to program flash\n");
		goto out;
	}

	/* We do have to reset chip in order to move new code from flash to iram. */
	DBG_INFO("Doing Soft Reset ..\n");
	core_config_ic_reset();

	/* the delay time moving code depends on what the touch IC you're using. */
	mdelay(core_firmware->delay_after_upgrade);

	/* ensure that the chip has been updated */
	DBG_INFO("Enter to ICE Mode again\n");
	res = core_config_ice_mode_enable();
	if (res < 0)
	{
		DBG_ERR("Failed to enable ICE mode\n");
		goto out;
	}

	mdelay(20);

	if (core_config->chip_id != CHIP_TYPE_ILI9881)
		core_config_reset_watch_dog();

	/* check the data that we've just written into the iram. */
	res = verify_flash_data();
	if(res == 0)
		DBG_INFO("Data Correct !\n");

out:
	//core_flash_enable_protect(true);
	core_config_ice_mode_disable();
	return res;
}

#ifdef BOOT_FW_UPGRADE
static int convert_hex_array(void)
{
	int i, j, index = 0;
	int block = 0, blen = 0, bindex = 0;
	uint32_t tmp_addr = 0x0;

	core_firmware->start_addr = 0;
	core_firmware->end_addr = 0;
	core_firmware->checksum = 0;
	core_firmware->crc32 = 0;
	core_firmware->hasBlockInfo = false;

	DBG_INFO("CTPM_FW = %d\n", (int)ARRAY_SIZE(CTPM_FW));

	if(ARRAY_SIZE(CTPM_FW) <= 0)
	{
		DBG_ERR("The size of CTPM_FW is invaild (%d)\n", (int)ARRAY_SIZE(CTPM_FW));
		goto out;
	}

	/* Get new version from ILI array */
	core_firmware->new_fw_ver[0] = CTPM_FW[19];
	core_firmware->new_fw_ver[1] = CTPM_FW[20];
	core_firmware->new_fw_ver[2] = CTPM_FW[21];

	/* The process will be executed if the comparison is different with origin ver */
	for(i = 0; i < ARRAY_SIZE(core_firmware->old_fw_ver); i++)
	{
		if(core_firmware->old_fw_ver[i] != core_firmware->new_fw_ver[i])
		{
			DBG_INFO("FW version is different, preparing to upgrade FW\n");
			break;
		}
	}

	if(i == ARRAY_SIZE(core_firmware->old_fw_ver))
	{
		DBG_ERR("FW version is the same as previous version\n");
		goto out;
	}

	/* Extract block info */
	block = CTPM_FW[33];

	if(block > 0)
	{
		core_firmware->hasBlockInfo = true;

		/* Initialize block's index and length */
		blen = 6;
		bindex = 34;

		for(i = 0; i < block; i++)
		{
			for(j = 0; j < blen; j++)
			{
				if(j < 3)
					_gFlashBlockInfo[i].start_addr = (_gFlashBlockInfo[i].start_addr << 8) | CTPM_FW[bindex+j];
				else
					_gFlashBlockInfo[i].end_addr = (_gFlashBlockInfo[i].end_addr << 8) | CTPM_FW[bindex+j];
			}

			bindex += blen;
		}
	}

	/* Fill data into buffer */
	for(i = 0; i < ARRAY_SIZE(CTPM_FW) - 64; i++)
	{
		flash_fw[i] = CTPM_FW[i+64];
		index = i / flashtab->sector;
		if(!_gFlashSector[index].data_flag)
		{
			_gFlashSector[index].ss_addr = index * flashtab->sector;
			_gFlashSector[index].se_addr = (index + 1) * flashtab->sector - 1;
			_gFlashSector[index].dlength = (_gFlashSector[index].se_addr - _gFlashSector[index].ss_addr) + 1;
			_gFlashSector[index].data_flag = true;
		}
	}

	_gSectionLen = index;

	if(_gFlashSector[_gSectionLen].se_addr > flashtab->mem_size)
	{
		DBG_ERR("The size written to flash is larger than it required (%x) (%x)\n",
					_gFlashSector[_gSectionLen].se_addr, flashtab->mem_size);
		goto out;
	}

	for(i = 0; i < _gTotalSector; i++)
	{
		/* fill meaing address in an array where is empty */
		if(_gFlashSector[i].ss_addr == 0x0 && _gFlashSector[i].se_addr == 0x0)
		{
			_gFlashSector[i].ss_addr = tmp_addr;
			_gFlashSector[i].se_addr = (i + 1) * flashtab->sector - 1;
		}

		tmp_addr += flashtab->sector;

		/* set erase flag in the block if the addr of sectors is between them. */
		if(core_firmware->hasBlockInfo)
		{
			for(j = 0; j < ARRAY_SIZE(_gFlashBlockInfo); j++)
			{
				if(_gFlashSector[i].ss_addr >= _gFlashBlockInfo[j].start_addr && _gFlashSector[i].se_addr <= _gFlashBlockInfo[j].end_addr)
				{
					_gFlashSector[i].inside_block = true;
					break;
				}
			}
		}

		/*
		 * protects the reserved address been written and erased.
		 * This feature only applies on the boot upgrade. The addr is progrmmable in normal case.
		 */
		if(_gFlashSector[i].ss_addr == _gStartResrv && _gFlashSector[i].se_addr == _gEndResrv)
		{
			_gFlashSector[i].inside_block = false;
		}
	}

	/* DEBUG: for showing data with address that will write into fw or be erased */
	for(i = 0; i < _gTotalSector; i++)
	{
		DBG_INFO("_gFlashSector[%d]: ss_addr = 0x%x, se_addr = 0x%x, length = %x, data = %d, inside_block = %d\n",
		i, _gFlashSector[i].ss_addr, _gFlashSector[i].se_addr, _gFlashSector[index].dlength, _gFlashSector[i].data_flag, _gFlashSector[i].inside_block);
	}

	core_firmware->start_addr = 0x0;
	core_firmware->end_addr = _gFlashSector[_gSectionLen].se_addr;
	DBG_INFO("start_addr = 0x%06X, end_addr = 0x%06X\n", core_firmware->start_addr, core_firmware->end_addr);
	return 0;

out:
	DBG_ERR("Failed to convert ILI FW array\n");
	return -1;
}

int core_firmware_boot_upgrade(void)
{
	int res = 0;
	bool power = false;

	DBG_INFO("BOOT: Starting to upgrade firmware ... \n");

	core_firmware->isUpgrading = true;
	core_firmware->update_status = 0;

	if(ipd->isEnablePollCheckPower)
	{
		ipd->isEnablePollCheckPower = false;
		cancel_delayed_work_sync(&ipd->check_power_status_work);
		power = true;
	}

	/* store old version before upgrade fw */
	core_firmware->old_fw_ver[0] = core_config->firmware_ver[1];
	core_firmware->old_fw_ver[1] = core_config->firmware_ver[2];
	core_firmware->old_fw_ver[2] = core_config->firmware_ver[3];

	if(flashtab == NULL)
	{
		DBG_ERR("Flash table isn't created\n");
		res = -ENOMEM;
		goto out;
	}

	flash_fw = kzalloc(sizeof(uint8_t) * flashtab->mem_size, GFP_KERNEL);
	if(ERR_ALLOC_MEM(flash_fw))
	{
		DBG_ERR("Failed to allocate flash_fw memory, %ld\n", PTR_ERR(flash_fw));
		res = -ENOMEM;
		goto out;
	}
	memset(flash_fw, 0xff, (int)sizeof(uint8_t) * flashtab->mem_size);

	_gTotalSector = flashtab->mem_size / flashtab->sector;
	if(_gTotalSector <= 0)
	{
		DBG_ERR("Flash configure is wrong\n");
		res = -1;
		goto out;
	}

	_gFlashSector = kcalloc(_gTotalSector, sizeof(uint32_t) * _gTotalSector, GFP_KERNEL);
	if(ERR_ALLOC_MEM(_gFlashSector))
	{
		DBG_ERR("Failed to allocate _gFlashSector memory, %ld\n", PTR_ERR(_gFlashSector));
		res = -ENOMEM;
		goto out;
	}

	res = convert_hex_array();
	if (res < 0)
	{
		DBG_ERR("Failed to covert firmware data, res = %d\n", res);
		goto out;
	}
	else
	{
		/* calling that function defined at init depends on chips. */
		res = core_firmware->upgrade_func(false);
		if (res < 0)
		{
			core_firmware->update_status = res;
			DBG_ERR("Failed to upgrade firmware, res = %d\n", res);
			goto out;
		}
		else
		{
			core_firmware->update_status = 100;
			DBG_INFO("Update firmware information...\n");
			core_config_get_fw_ver();
			core_config_get_protocol_ver();
			core_config_get_core_ver();
			core_config_get_tp_info();
			core_config_get_key_info();
		}
	}

out:
	if(power)
	{
		ipd->isEnablePollCheckPower = true;
		queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);
	}

	kfree(flash_fw);
	flash_fw = NULL;
	kfree(_gFlashSector);
	_gFlashSector = NULL;

	core_firmware->isUpgrading = false;
	return res;
}
#endif

static int convert_hex_file(uint8_t *pBuf, uint32_t nSize, bool isIRAM)
{
	uint32_t i = 0, j = 0, k = 0;
	uint32_t nLength = 0, nAddr = 0, nType = 0;
	uint32_t nStartAddr = 0x0, nEndAddr = 0x0, nChecksum = 0x0,nExAddr = 0;
	uint32_t tmp_addr = 0x0;
	int index = 0, block = 0;

	core_firmware->start_addr = 0;
	core_firmware->end_addr = 0;
	core_firmware->checksum = 0;
	core_firmware->crc32 = 0;
	core_firmware->hasBlockInfo = false;

	memset(_gFlashBlockInfo, 0x0, sizeof(_gFlashBlockInfo));

	for (; i < nSize;)
	{
		int32_t nOffset;
		nLength = HexToDec(&pBuf[i + 1], 2);
		nAddr = HexToDec(&pBuf[i + 3], 4);
		nType = HexToDec(&pBuf[i + 7], 2);

		// calculate checksum
		for (j = 8; j < (2 + 4 + 2 + (nLength * 2)); j += 2)
		{
			if (nType == 0x00)
			{
				// for ice mode write method
				nChecksum = nChecksum + HexToDec(&pBuf[i + 1 + j], 2);
			}
		}

		if (nType == 0x04)
		{
			nExAddr = HexToDec(&pBuf[i + 9], 4);
		}

		if (nType == 0x02)
		{
			nExAddr = HexToDec(&pBuf[i + 9], 4);
			nExAddr = nExAddr >> 12;
		}

		if (nType == 0xAE)
		{
			core_firmware->hasBlockInfo = true;
			/* insert block info extracted from hex */
			if(block < 4)
			{
				_gFlashBlockInfo[block].start_addr = HexToDec(&pBuf[i + 9], 6);
				_gFlashBlockInfo[block].end_addr = HexToDec(&pBuf[i + 9 + 6], 6);
				DBG(DEBUG_FIRMWARE, "Block[%d]: start_addr = %x, end = %x\n",
					block, _gFlashBlockInfo[block].start_addr, _gFlashBlockInfo[block].end_addr);
			}
			block++;
		}

		nAddr = nAddr + (nExAddr << 16);
		if (pBuf[i + 1 + j + 2] == 0x0D)
		{
			nOffset = 2;
		}
		else
		{
			nOffset = 1;
		}

		if (nType == 0x00)
		{
			if (nAddr > MAX_HEX_FILE_SIZE)
			{
				DBG_ERR("Invalid hex format\n");
				goto out;
			}

			if (nAddr < nStartAddr)
			{
				nStartAddr = nAddr;
			}
			if ((nAddr + nLength) > nEndAddr)
			{
				nEndAddr = nAddr + nLength;
			}

			// fill data
			for (j = 0, k = 0; j < (nLength * 2); j += 2, k++)
			{
				if (isIRAM)
					iram_fw[nAddr + k] = HexToDec(&pBuf[i + 9 + j], 2);
				else
				{
					flash_fw[nAddr + k] = HexToDec(&pBuf[i + 9 + j], 2);

					if((nAddr + k) != 0)
					{
						index = ((nAddr + k) / flashtab->sector);
						if(!_gFlashSector[index].data_flag)
						{
							_gFlashSector[index].ss_addr = index * flashtab->sector;
							_gFlashSector[index].se_addr = (index + 1) * flashtab->sector - 1;
							_gFlashSector[index].dlength = (_gFlashSector[index].se_addr - _gFlashSector[index].ss_addr) + 1;
							_gFlashSector[index].data_flag = true;
						}
					}
				}
			}
		}
		i += 1 + 2 + 4 + 2 + (nLength * 2) + 2 + nOffset;
	}

	_gSectionLen = index;

	if(_gFlashSector[_gSectionLen-1].se_addr > flashtab->mem_size)
	{
		DBG_ERR("The size written to flash is larger than it required (%x) (%x)\n",
					_gFlashSector[_gSectionLen-1].se_addr, flashtab->mem_size);
		goto out;
	}

	for(i = 0; i < _gTotalSector; i++)
	{
		/* fill meaing address in an array where is empty*/
		if(_gFlashSector[i].ss_addr == 0x0 && _gFlashSector[i].se_addr == 0x0)
		{
			_gFlashSector[i].ss_addr = tmp_addr;
			_gFlashSector[i].se_addr = (i + 1) * flashtab->sector - 1;
		}

		tmp_addr += flashtab->sector;

		/* set erase flag in the block if the addr of sectors is between them. */
		if(core_firmware->hasBlockInfo)
		{
			for(j = 0; j < ARRAY_SIZE(_gFlashBlockInfo); j++)
			{
				if(_gFlashSector[i].ss_addr >= _gFlashBlockInfo[j].start_addr && _gFlashSector[i].se_addr <= _gFlashBlockInfo[j].end_addr)
				{
					_gFlashSector[i].inside_block = true;
					break;
				}
			}
		}
	}

	/* DEBUG: for showing data with address that will write into fw or be erased */
	for(i = 0; i < _gTotalSector; i++)
	{
		DBG(DEBUG_FIRMWARE, "_gFlashSector[%d]: ss_addr = 0x%x, se_addr = 0x%x, length = %x, data = %d, inside_block = %d\n",
		i, _gFlashSector[i].ss_addr, _gFlashSector[i].se_addr, _gFlashSector[index].dlength, _gFlashSector[i].data_flag, _gFlashSector[i].inside_block);
	}

	core_firmware->start_addr = nStartAddr;
	core_firmware->end_addr = nEndAddr;
	DBG_INFO("nStartAddr = 0x%06X, nEndAddr = 0x%06X\n", nStartAddr, nEndAddr);
	return 0;

out:
	DBG_ERR("Failed to convert HEX data\n");
	return -1;
}

/*
 * It would basically be called by ioctl when users want to upgrade firmware.
 *
 * @pFilePath: pass a path where locates user's firmware file.
 *
 */
int core_firmware_upgrade(const char *pFilePath, bool isIRAM)
{
	int res = 0, fsize;
	uint8_t *hex_buffer = NULL;
	bool power = false;

	struct file *pfile = NULL;
	struct inode *inode;
	mm_segment_t old_fs;
	loff_t pos = 0;

	core_firmware->isUpgrading = true;
	core_firmware->update_status = 0;

	if(ipd->isEnablePollCheckPower)
	{
		ipd->isEnablePollCheckPower = false;
		cancel_delayed_work_sync(&ipd->check_power_status_work);
		power = true;
	}

	pfile = filp_open(pFilePath, O_RDONLY, 0);
	if (ERR_ALLOC_MEM(pfile))
	{
		DBG_ERR("Failed to open the file at %s.\n", pFilePath);
		res = -ENOENT;
		return res;
	}
	else
	{
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 0)
		inode = pfile->f_dentry->d_inode;
#else
		inode = pfile->f_path.dentry->d_inode;
#endif

		fsize = inode->i_size;

		DBG_INFO("fsize = %d\n", fsize);

		if (fsize <= 0)
		{
			DBG_ERR("The size of file is zero\n");
			res = -EINVAL;
			goto out;
		}
		else
		{
			if(flashtab == NULL)
			{
				DBG_ERR("Flash table isn't created\n");
				res = -ENOMEM;
				goto out;
			}

			hex_buffer = kzalloc(sizeof(uint8_t) * fsize, GFP_KERNEL);
			if(ERR_ALLOC_MEM(hex_buffer))
			{
				DBG_ERR("Failed to allocate hex_buffer memory, %ld\n", PTR_ERR(hex_buffer));
				res = -ENOMEM;
				goto out;
			}

			flash_fw = kzalloc(sizeof(uint8_t) * flashtab->mem_size, GFP_KERNEL);
			if(ERR_ALLOC_MEM(flash_fw))
			{
				DBG_ERR("Failed to allocate flash_fw memory, %ld\n", PTR_ERR(flash_fw));
				res = -ENOMEM;
				goto out;
			}
			memset(flash_fw, 0xff, sizeof(uint8_t) * flashtab->mem_size);

			_gTotalSector = flashtab->mem_size / flashtab->sector;
			if(_gTotalSector <= 0)
			{
				DBG_ERR("Flash configure is wrong\n");
				res = -1;
				goto out;
			}

			_gFlashSector = kcalloc(_gTotalSector, sizeof(uint32_t) * _gTotalSector, GFP_KERNEL);
			if(ERR_ALLOC_MEM(_gFlashSector))
			{
				DBG_ERR("Failed to allocate _gFlashSector memory, %ld\n", PTR_ERR(_gFlashSector));
				res = -ENOMEM;
				goto out;
			}

			/* store current userspace mem segment. */
			old_fs = get_fs();

			/* set userspace mem segment equal to kernel's one. */
			set_fs(get_ds());

			/* read firmware data from userspace mem segment */
			vfs_read(pfile, hex_buffer, fsize, &pos);

			/* restore userspace mem segment after read. */
			set_fs(old_fs);

			res = convert_hex_file(hex_buffer, fsize, isIRAM);
			if (res < 0)
			{
				DBG_ERR("Failed to covert firmware data, res = %d\n", res);
				goto out;
			}
			else
			{
				/* calling that function defined at init depends on chips. */
				res = core_firmware->upgrade_func(isIRAM);
				if (res < 0)
				{
					DBG_ERR("Failed to upgrade firmware, res = %d\n", res);
					goto out;
				}
				else
				{
					DBG_INFO("Update firmware information...\n");
					core_config_get_fw_ver();
					core_config_get_protocol_ver();
					core_config_get_core_ver();
					core_config_get_tp_info();
					core_config_get_key_info();
				}
			}
		}
	}

out:
	if(power)
	{
		ipd->isEnablePollCheckPower = true;
		queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);
	}

	filp_close(pfile, NULL);
	kfree(hex_buffer);
	hex_buffer = NULL;
	kfree(flash_fw);
	flash_fw = NULL;
	kfree(_gFlashSector);
	_gFlashSector = NULL;

	core_firmware->isUpgrading = false;
	return res;
}

int core_firmware_init(void)
{
	int i = 0, j = 0;

	core_firmware = kzalloc(sizeof(*core_firmware), GFP_KERNEL);
	if(ERR_ALLOC_MEM(core_firmware))
	{
		DBG_ERR("Failed to allocate core_firmware mem, %ld\n", PTR_ERR(core_firmware));
		core_firmware_remove();
		return -ENOMEM;
	}

	core_firmware->hasBlockInfo = false;
	core_firmware->isboot = false;

	for (; i < ARRAY_SIZE(ipio_chip_list); i++)
	{
		if (ipio_chip_list[i] == TP_TOUCH_IC)
		{
			for (j = 0; j < 4; j++)
			{
				core_firmware->old_fw_ver[i] = core_config->firmware_ver[i];
				core_firmware->new_fw_ver[i] = 0x0;
			}

			if (ipio_chip_list[i] == CHIP_TYPE_ILI7807)
			{
				core_firmware->max_count = 0xFFFF;
				core_firmware->isCRC = false;
				core_firmware->upgrade_func = tddi_fw_upgrade;
				core_firmware->delay_after_upgrade = 100;
			}
			else if (ipio_chip_list[i] == CHIP_TYPE_ILI9881)
			{
				core_firmware->max_count = 0x1FFFF;
				core_firmware->isCRC = true;
				core_firmware->upgrade_func = tddi_fw_upgrade;
				core_firmware->delay_after_upgrade = 200;
			}
			return 0;
		}
	}

	DBG_ERR("Can't find this chip in support list\n");
	return 0;
}

void core_firmware_remove(void)
{
	DBG_INFO("Remove core-firmware members\n");

	if(core_firmware != NULL)
		kfree(core_firmware);
}
