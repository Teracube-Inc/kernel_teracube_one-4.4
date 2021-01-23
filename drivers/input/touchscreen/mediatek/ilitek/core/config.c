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
#include <linux/i2c.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#endif

#include "../common.h"
#include "../platform.h"
#include "config.h"
#include "protocol.h"
#include "i2c.h"
#include "flash.h"

/* the list of support chip */
uint32_t ipio_chip_list[] = {
	CHIP_TYPE_ILI7807,
	CHIP_TYPE_ILI9881,
};

uint8_t _gReadBuf[128] = {0};

struct core_config_data *core_config = NULL;

static void read_flash_info(uint8_t cmd, int len)
{
	int i;
	uint16_t flash_id = 0, flash_mid = 0;
	uint8_t buf[4] = {0};

	/*
	 * This command is used to fix the bug of spi clk for 7807F-AB
	 * when operating with its flash.
	 */
	if (core_config->chip_id == CHIP_TYPE_ILI7807
			&& core_config->chip_type == ILI7807_TYPE_F_AB)
	{
		core_config_ice_mode_write(0x4100C, 0x01, 1);
		mdelay(25);
	}

	core_config_ice_mode_write(0x41000, 0x0, 1); /* CS high */
	core_config_ice_mode_write(0x41004, 0x66aa55, 3); /* Key */

	core_config_ice_mode_write(0x41008, cmd, 1);
	for(i = 0; i < len; i++)
	{
		core_config_ice_mode_write(0x041008, 0xFF, 1);
		buf[i] = core_config_ice_mode_read(0x41010);
	}

	core_config_ice_mode_write(0x041000, 0x1, 1); /* CS high */

	/* look up flash info and init its struct after obtained flash id. */
	flash_mid = buf[0];
	flash_id = buf[1] << 8 | buf[2];
	core_flash_init(flash_mid, flash_id);
}

/*
 * It checks chip id shifting sepcific bits based on chip's requirement.
 *
 * @pid_data: 4 bytes, reading from firmware.
 *
 */
static uint32_t check_chip_id(uint32_t pid_data)
{
	uint32_t id = 0;

	if (core_config->chip_id == CHIP_TYPE_ILI7807)
	{
		id = pid_data >> 16;
		core_config->chip_type = pid_data & 0x0000FFFF;

		if (core_config->chip_type == ILI7807_TYPE_F_AB)
		{
			core_config->ic_reset_addr = 0x04004C;
		}
		else if (core_config->chip_type == ILI7807_TYPE_H)
		{
			core_config->ic_reset_addr = 0x040050;
		}
	}
	else if (core_config->chip_id == CHIP_TYPE_ILI9881)
	{
		id = pid_data >> 16;
		core_config->ic_reset_addr = 0x040050;
	}
	else
	{
		DBG_ERR("The Chip isn't supported by the driver\n");
	}

	return id;
}

/*
 * Read & Write one byte in ICE Mode.
 */
uint32_t core_config_read_write_onebyte(uint32_t addr)
{
	int res = 0;
	uint32_t data = 0;
	uint8_t szOutBuf[64] = {0};

	szOutBuf[0] = 0x25;
	szOutBuf[1] = (char)((addr & 0x000000FF) >> 0);
	szOutBuf[2] = (char)((addr & 0x0000FF00) >> 8);
	szOutBuf[3] = (char)((addr & 0x00FF0000) >> 16);

	res = core_i2c_write(core_config->slave_i2c_addr, szOutBuf, 4);
	if (res < 0)
		goto out;

	mdelay(1);

	res = core_i2c_read(core_config->slave_i2c_addr, szOutBuf, 1);
	if (res < 0)
		goto out;

	data = (szOutBuf[0]);

	return data;

out:
	DBG_ERR("Failed to read/write data in ICE mode, res = %d\n", res);
	return res;
}
EXPORT_SYMBOL(core_config_read_write_onebyte);

uint32_t core_config_ice_mode_read(uint32_t addr)
{
	int res = 0;
	uint8_t szOutBuf[64] = {0};
	uint32_t data = 0;

	szOutBuf[0] = 0x25;
	szOutBuf[1] = (char)((addr & 0x000000FF) >> 0);
	szOutBuf[2] = (char)((addr & 0x0000FF00) >> 8);
	szOutBuf[3] = (char)((addr & 0x00FF0000) >> 16);

	res = core_i2c_write(core_config->slave_i2c_addr, szOutBuf, 4);
	if (res < 0)
		goto out;

	mdelay(10);

	res = core_i2c_read(core_config->slave_i2c_addr, szOutBuf, 4);
	if (res < 0)
		goto out;

	data = (szOutBuf[0] + szOutBuf[1] * 256 + szOutBuf[2] * 256 * 256 + szOutBuf[3] * 256 * 256 * 256);

	return data;

out:
	DBG_ERR("Failed to read data in ICE mode, res = %d\n", res);
	return res;
}
EXPORT_SYMBOL(core_config_ice_mode_read);

/*
 * Write commands into firmware in ICE Mode.
 *
 */
int core_config_ice_mode_write(uint32_t addr, uint32_t data, uint32_t size)
{
	int res = 0, i;
	uint8_t szOutBuf[64] = {0};

	szOutBuf[0] = 0x25;
	szOutBuf[1] = (char)((addr & 0x000000FF) >> 0);
	szOutBuf[2] = (char)((addr & 0x0000FF00) >> 8);
	szOutBuf[3] = (char)((addr & 0x00FF0000) >> 16);

	for (i = 0; i < size; i++)
	{
		szOutBuf[i + 4] = (char)(data >> (8 * i));
	}

	res = core_i2c_write(core_config->slave_i2c_addr, szOutBuf, size + 4);

	if (res < 0)
		DBG_ERR("Failed to write data in ICE mode, res = %d\n", res);

	return res;
}
EXPORT_SYMBOL(core_config_ice_mode_write);

/*
 * Doing soft reset on ic.
 *
 * It resets ic's status, moves code and leave ice mode automatically if in
 * that mode.
 */
void core_config_ic_reset(void)
{
	uint32_t key = 0;

	if (core_config->chip_id == CHIP_TYPE_ILI7807)
	{
		if(core_config->chip_type == ILI7807_TYPE_H)
			key = 0x00117807;
		else
			key = 0x00017807;
	}
	if (core_config->chip_id == CHIP_TYPE_ILI9881)
	{
		key = 0x00019881;
	}

	DBG(DEBUG_CONFIG, "key = 0x%x\n", key);
	if(key != 0)
	{
		core_config->do_ic_reset = true;
		core_config_ice_mode_write(core_config->ic_reset_addr, key, 4);
		core_config->do_ic_reset = false;
	}

	msleep(300);
}
EXPORT_SYMBOL(core_config_ic_reset);

void core_config_sense_ctrl(bool start)
{
	DBG_INFO("sense start = %d\n", start);

	return core_protocol_func_control(0, start);
}
EXPORT_SYMBOL(core_config_sense_ctrl);

void core_config_sleep_ctrl(bool out)
{
	DBG_INFO("Sleep Out = %d\n", out);

	return core_protocol_func_control(1, out);
}
EXPORT_SYMBOL(core_config_sleep_ctrl);

void core_config_glove_ctrl(bool enable, bool seamless)
{
	int cmd = 0x2; // default as semaless

	if(!seamless)
	{
		if(enable)
			cmd = 0x1;
		else
			cmd = 0x0;
	}

	DBG_INFO("Glove = %d, seamless = %d, cmd = %d\n", enable, seamless, cmd);

	return core_protocol_func_control(2, cmd);
}
EXPORT_SYMBOL(core_config_glove_ctrl);

void core_config_stylus_ctrl(bool enable, bool seamless)
{
	int cmd = 0x2; // default as semaless

	if(!seamless)
	{
		if(enable)
			cmd = 0x1;
		else
			cmd = 0x0;
	}

	DBG_INFO("stylus = %d, seamless = %d, cmd = %x\n", enable, seamless, cmd);

	return core_protocol_func_control(3, cmd);
}
EXPORT_SYMBOL(core_config_stylus_ctrl);

void core_config_tp_scan_mode(bool mode)
{
	DBG_INFO("TP Scan mode = %d\n", mode);

	return core_protocol_func_control(4, mode);
}
EXPORT_SYMBOL(core_config_tp_scan_mode);

void core_config_lpwg_ctrl(bool enable)
{
	DBG_INFO("LPWG = %d\n", enable);

	return core_protocol_func_control(5, enable);
}
EXPORT_SYMBOL(core_config_lpwg_ctrl);

void core_config_gesture_ctrl(uint8_t func)
{
	uint8_t max_byte = 0x0, min_byte = 0x0;

	DBG_INFO("Gesture function = 0x%x\n", func);

	max_byte = 0x3F;
	min_byte = 0x20;

	if(func > max_byte || func < min_byte)
	{
		DBG_ERR("Gesture ctrl error, 0x%x\n", func);
		return;
	}

	return core_protocol_func_control(6, func);
}
EXPORT_SYMBOL(core_config_gesture_ctrl);

void core_config_phone_cover_ctrl(bool enable)
{
	DBG_INFO("Phone Cover = %d\n", enable);

	return core_protocol_func_control(7, enable);
}
EXPORT_SYMBOL(core_config_phone_cover_ctrl);

void core_config_finger_sense_ctrl(bool enable)
{
	DBG_INFO("Finger sense = %d\n", enable);

	return core_protocol_func_control(0, enable);
}
EXPORT_SYMBOL(core_config_finger_sense_ctrl);

void core_config_proximity_ctrl(bool enable)
{
	DBG_INFO("Proximity = %d\n", enable);

	return core_protocol_func_control(11, enable);
}
EXPORT_SYMBOL(core_config_proximity_ctrl);

void core_config_plug_ctrl(bool out)
{
	DBG_INFO("Plug Out = %d\n", out);

	return core_protocol_func_control(12, out);
}
EXPORT_SYMBOL(core_config_plug_ctrl);

void core_config_set_phone_cover(uint8_t *pattern)
{
	uint8_t ul_x_l = UL_X_LOW, ul_x_h = UL_X_HIGH;
	uint8_t ul_y_l = UL_Y_LOW, ul_y_h = UL_Y_HIGH;
	uint8_t br_x_l = BR_X_LOW, br_x_h = BR_X_HIGH;
	uint8_t br_y_l = BR_Y_LOW, br_y_h = BR_Y_HIGH;

	DBG_INFO("pattern = 0x%x\n", *pattern);

	if(*pattern < 0 || pattern == NULL)
	{
		DBG_ERR("Invaild width or height\n");
		return;
	}

	if(*pattern == 0)
	{
		protocol->phone_cover_window[1] = ul_x_l;
		protocol->phone_cover_window[2] = ul_x_h;
		protocol->phone_cover_window[3] = ul_y_l;
		protocol->phone_cover_window[4] = ul_y_h;
		protocol->phone_cover_window[5] = br_x_l;
		protocol->phone_cover_window[6] = br_x_h;
		protocol->phone_cover_window[7] = br_y_l;
		protocol->phone_cover_window[8] = br_y_h;
	}
	else
	{
		/* TODO */
	}

	DBG_INFO("window: cmd = 0x%x\n", protocol->phone_cover_window[0]);
	DBG_INFO("window: ul_x_l = 0x%x, ul_x_h = 0x%x\n", protocol->phone_cover_window[1], protocol->phone_cover_window[2]);
	DBG_INFO("window: ul_y_l = 0x%x, ul_y_l = 0x%x\n", protocol->phone_cover_window[3], protocol->phone_cover_window[4]);
	DBG_INFO("window: br_x_l = 0x%x, br_x_l = 0x%x\n", protocol->phone_cover_window[5], protocol->phone_cover_window[6]);
	DBG_INFO("window: br_y_l = 0x%x, br_y_l = 0x%x\n", protocol->phone_cover_window[7], protocol->phone_cover_window[8]);

	core_protocol_func_control(9, 0);
}
EXPORT_SYMBOL(core_config_set_phone_cover);

/*
 * ic_suspend: Get IC to suspend called from system.
 *
 * The timing when goes to sense stop or houw much times the command need to be called
 * is depending on customer's system requirement, which might be different due to
 * the DDI design or other conditions.
 */
void core_config_ic_suspend(void)
{
	DBG_INFO("Starting to suspend ... \n");

	/* sense stop */
	core_config_sense_ctrl(false);

	/* check system busy */
	if(core_config_check_cdc_busy() < 0)
		DBG_ERR("Check busy is timout !\n");

	DBG_INFO("Enabled Gesture = %d\n", core_config->isEnableGesture);

	if(core_config->isEnableGesture)
	{
		core_config_lpwg_ctrl(true);
	}
	else
	{
		/* sleep in */
		core_config_sleep_ctrl(false);
	}

	DBG_INFO("Suspend done\n");
}
EXPORT_SYMBOL(core_config_ic_suspend);

/*
 * ic_resume: Get IC to resume called from system.
 *
 * The timing when goes to sense start or houw much times the command need to be called
 * is depending on customer's system requirement, which might be different due to
 * the DDI design or other conditions.
 */
void core_config_ic_resume(void)
{
	DBG_INFO("Starting to resume ... \n");

	/* sleep out */
	core_config_sleep_ctrl(true);

	/* check system busy */
	if(core_config_check_cdc_busy() < 0)
		DBG_ERR("Check busy is timout !\n");

	/* sense start for TP */
	core_config_sense_ctrl(true);

	/* Soft reset */
	core_config_ice_mode_enable();
	mdelay(10);
	core_config_ic_reset();

	DBG_INFO("Resume done \n");
}
EXPORT_SYMBOL(core_config_ic_resume);

int core_config_ice_mode_disable(void)
{
	uint8_t cmd[4];

	cmd[0] = 0x1b;
	cmd[1] = 0x62;
	cmd[2] = 0x10;
	cmd[3] = 0x18;

	DBG_INFO("ICE Mode disabled\n")

	return core_i2c_write(core_config->slave_i2c_addr, cmd, 4);
}
EXPORT_SYMBOL(core_config_ice_mode_disable);

int core_config_ice_mode_enable(void)
{
	DBG_INFO("ICE Mode enabled\n");

	if(core_config_ice_mode_write(0x181062, 0x0, 0) < 0)
		return -1;
	core_config_ice_mode_read(core_config->pid_addr);
	return 0;
}
EXPORT_SYMBOL(core_config_ice_mode_enable);

int core_config_reset_watch_dog(void)
{
	 if (core_config->chip_id == CHIP_TYPE_ILI7807)
	{
		core_config_ice_mode_write(0x5100C, 0x7, 1);
		core_config_ice_mode_write(0x5100C, 0x78, 1);
	}

	return 0;
}
EXPORT_SYMBOL(core_config_reset_watch_dog);

int core_config_check_cdc_busy(void)
{
	int timer = 50, res = -1;
    uint8_t cmd[2] = {0};
	uint8_t busy = 0;

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_cdc_busy;

	while(timer > 0)
	{
		core_i2c_write(core_config->slave_i2c_addr, cmd, 2);
		mdelay(1);
		core_i2c_write(core_config->slave_i2c_addr, &cmd[1], 1);
		mdelay(1);
		core_i2c_read(core_config->slave_i2c_addr, &busy, 1);
		DBG(DEBUG_CONFIG, "CDC busy state = 0x%x\n", busy);
		if(busy == 0x41 || busy == 0x51)
		{
			res = 0;
			break;
		}
		mdelay(10);
		timer--;
	}

	return res;
}
EXPORT_SYMBOL(core_config_check_cdc_busy);

int core_config_check_int_status_high(void)
{
	int timer = 50, res = -1;
	while (timer)
	{
		if (gpio_get_value(ipd->int_gpio))
		{
			res = 0;
			break;
		}
		mdelay(10);
		timer--;
	}
	return res;
}
EXPORT_SYMBOL(core_config_check_int_status_high);

int core_config_check_int_status_low(void)
{
	int timer = 50, res = -1;
	while (timer)
	{
		if (!gpio_get_value(ipd->int_gpio))
		{
			res = 0;
			break;
		}
		mdelay(10);
		timer--;
	}
	return res;
}
EXPORT_SYMBOL(core_config_check_int_status_low);

int core_config_get_key_info(void)
{
	int res = 0, i;
	uint8_t cmd[2] = {0};

	memset(_gReadBuf, 0, sizeof(_gReadBuf));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_key_info;

	res = core_i2c_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0)
	{
		DBG_ERR("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	mdelay(1);

	res = core_i2c_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0)
	{
		DBG_ERR("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	mdelay(1);

	res = core_i2c_read(core_config->slave_i2c_addr, &_gReadBuf[0], protocol->key_info_len);
	if (res < 0)
	{
		DBG_ERR("Failed to read data via I2C, %d\n", res);
		goto out;
	}

	if (core_config->tp_info->nKeyCount)
	{
		/* NOTE: Firmware not ready yet */
		core_config->tp_info->nKeyAreaXLength = (_gReadBuf[0] << 8) + _gReadBuf[1];
		core_config->tp_info->nKeyAreaYLength = (_gReadBuf[2] << 8) + _gReadBuf[3];

		DBG_INFO("key: length of X area = %x\n", core_config->tp_info->nKeyAreaXLength);
		DBG_INFO("key: length of Y area = %x\n", core_config->tp_info->nKeyAreaYLength);

		for (i = 0; i < core_config->tp_info->nKeyCount; i ++)
		{
			core_config->tp_info->virtual_key[i].nId = _gReadBuf[i*5+4];
			core_config->tp_info->virtual_key[i].nX = (_gReadBuf[i*5+5] << 8) + _gReadBuf[i*5+6];
			core_config->tp_info->virtual_key[i].nY = (_gReadBuf[i*5+7] << 8) + _gReadBuf[i*5+8];
			core_config->tp_info->virtual_key[i].nStatus = 0;

			DBG_INFO("key: id = %d, X = %d, Y = %d\n", core_config->tp_info->virtual_key[i].nId,
			core_config->tp_info->virtual_key[i].nX, core_config->tp_info->virtual_key[i].nY);
		}
	}

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_key_info);

int core_config_get_tp_info(void)
{
	int res = 0;
	uint8_t cmd[2] = {0};

	memset(_gReadBuf, 0, sizeof(_gReadBuf));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_tp_info;

	res = core_i2c_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0)
	{
		DBG_ERR("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	mdelay(1);

	res = core_i2c_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0)
	{
		DBG_ERR("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	mdelay(1);

	res = core_i2c_read(core_config->slave_i2c_addr, &_gReadBuf[0], protocol->tp_info_len);
	if (res < 0)
	{
		DBG_ERR("Failed to read data via I2C, %d\n", res);
		goto out;
	}

	/* in protocol v5, ignore the first btye because of a header. */
	core_config->tp_info->nMinX = _gReadBuf[1];
	core_config->tp_info->nMinY = _gReadBuf[2];
	core_config->tp_info->nMaxX = (_gReadBuf[4] << 8) + _gReadBuf[3];
	core_config->tp_info->nMaxY = (_gReadBuf[6] << 8) + _gReadBuf[5];
	core_config->tp_info->nXChannelNum = _gReadBuf[7];
	core_config->tp_info->nYChannelNum = _gReadBuf[8];
	core_config->tp_info->self_tx_channel_num = _gReadBuf[11];
	core_config->tp_info->self_rx_channel_num = _gReadBuf[12];
	core_config->tp_info->side_touch_type = _gReadBuf[13];
	core_config->tp_info->nMaxTouchNum = _gReadBuf[9];
	core_config->tp_info->nKeyCount = _gReadBuf[10];

	core_config->tp_info->nMaxKeyButtonNum = 5;

	DBG_INFO("minX = %d, minY = %d, maxX = %d, maxY = %d\n",
				core_config->tp_info->nMinX, core_config->tp_info->nMinY,
				core_config->tp_info->nMaxX, core_config->tp_info->nMaxY);
	DBG_INFO("xchannel = %d, ychannel = %d, self_tx = %d, self_rx = %d\n",
				core_config->tp_info->nXChannelNum, core_config->tp_info->nYChannelNum,
				core_config->tp_info->self_tx_channel_num, core_config->tp_info->self_rx_channel_num);
	DBG_INFO("side_touch_type = %d, max_touch_num= %d, touch_key_num = %d, max_key_num = %d\n",
				core_config->tp_info->side_touch_type, core_config->tp_info->nMaxTouchNum,
				core_config->tp_info->nKeyCount, core_config->tp_info->nMaxKeyButtonNum);

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_tp_info);

int core_config_get_protocol_ver(void)
{
	int res = 0, i = 0;
	int major, mid, minor;
	uint8_t cmd[2] = {0};

	memset(_gReadBuf, 0, sizeof(_gReadBuf));
	memset(core_config->protocol_ver, 0x0, sizeof(core_config->protocol_ver));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_pro_ver;

	res = core_i2c_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0)
	{
		DBG_ERR("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	mdelay(1);

	res = core_i2c_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0)
	{
		DBG_ERR("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	mdelay(1);

	res = core_i2c_read(core_config->slave_i2c_addr, &_gReadBuf[0], protocol->pro_ver_len);
	if (res < 0)
	{
		DBG_ERR("Failed to read data via I2C, %d\n", res);
		goto out;
	}

	/* ignore the first btye because of a header. */
	for (; i < protocol->pro_ver_len; i++)
		core_config->protocol_ver[i] = _gReadBuf[i+1];

	DBG_INFO("Procotol Version = %d.%d.%d\n",
			core_config->protocol_ver[0],core_config->protocol_ver[1],core_config->protocol_ver[2]);

	major = core_config->protocol_ver[0];
	mid   = core_config->protocol_ver[1];
	minor = core_config->protocol_ver[2];

	/* update protocol if they're different with the default ver set by driver */
	if(major != PROTOCOL_MAJOR || mid != PROTOCOL_MID || minor != PROTOCOL_MINOR)
	{
		res = core_protocol_update_ver(major, mid, minor);
		if(res < 0)
			DBG_ERR("Protocol version is invalid\n");
	}

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_protocol_ver);

int core_config_get_core_ver(void)
{
	int res = 0, i = 0;
	uint8_t cmd[2] = {0};

	memset(_gReadBuf, 0, sizeof(_gReadBuf));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_core_ver;

	res = core_i2c_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0)
	{
		DBG_ERR("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	mdelay(1);

	res = core_i2c_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0)
	{
		DBG_ERR("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	mdelay(1);

	res = core_i2c_read(core_config->slave_i2c_addr, &_gReadBuf[0], protocol->core_ver_len);
	if (res < 0)
	{
		DBG_ERR("Failed to read data via I2C, %d\n", res);
		goto out;
	}

	for (; i < protocol->core_ver_len; i++)
		core_config->core_ver[i] = _gReadBuf[i];

	/* in protocol v5, ignore the first btye because of a header. */
	DBG_INFO("Core Version = %d.%d.%d.%d\n",
			core_config->core_ver[1], core_config->core_ver[2],
			core_config->core_ver[3], core_config->core_ver[4]);

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_core_ver);

/*
 * Getting the version of firmware used on the current one.
 *
 */
int core_config_get_fw_ver(void)
{
	int res = 0, i = 0;
	uint8_t cmd[2] = {0};

	memset(_gReadBuf, 0, sizeof(_gReadBuf));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_fw_ver;

	res = core_i2c_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0)
	{
		DBG_ERR("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	mdelay(1);

	res = core_i2c_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0)
	{
		DBG_ERR("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	mdelay(1);

	res = core_i2c_read(core_config->slave_i2c_addr, &_gReadBuf[0], protocol->fw_ver_len);
	if (res < 0)
	{
		DBG_ERR("Failed to read fw version %d\n", res);
		goto out;
	}

	for (; i < protocol->fw_ver_len ; i++)
		core_config->firmware_ver[i] = _gReadBuf[i];

	/* in protocol v5, ignore the first btye because of a header. */
	DBG_INFO("Firmware Version = %d.%d.%d\n",
				core_config->firmware_ver[1], core_config->firmware_ver[2],
				core_config->firmware_ver[3]);

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_fw_ver);

int core_config_get_chip_id(void)
{
	int res = 0;
	static int do_once = 0;
	uint32_t RealID = 0, PIDData = 0;

	res = core_config_ice_mode_enable();
	if (res < 0)
	{
		DBG_ERR("Failed to enter ICE mode, res = %d\n", res);
		goto out;
	}

	mdelay(20);

	PIDData = core_config_ice_mode_read(core_config->pid_addr);

	if (PIDData)
	{
		RealID = check_chip_id(PIDData);

		DBG_INFO("CHIP ID = 0x%x, CHIP TYPE = %04x\n", RealID, core_config->chip_type);

		if (RealID != core_config->chip_id)
		{
			DBG_ERR("CHIP ID ERROR: 0x%x, TP_TOUCH_IC = 0x%x\n",
						RealID, TP_TOUCH_IC);
			res = -ENODEV;
			goto out;
		}
	}
	else
	{
		DBG_ERR("PID DATA error : 0x%x\n", PIDData);
		res = -EINVAL;
		goto out;
	}

	if(do_once == 0)
	{
		/* reading flash id needs to let ic entry to ICE mode */
		read_flash_info(0x9F, 4);
		do_once = 1;
	}

out:
	core_config_ic_reset();
	mdelay(150);
	return res;
}
EXPORT_SYMBOL(core_config_get_chip_id);

int core_config_init(void)
{
	int i = 0;

	core_config = kzalloc(sizeof(*core_config) * sizeof(uint8_t) * 6, GFP_KERNEL);
	if(ERR_ALLOC_MEM(core_config))
	{
		DBG_ERR("Failed to allocate core_config mem, %ld\n", PTR_ERR(core_config));
		core_config_remove();
		return -ENOMEM;
	}

	core_config->tp_info = kzalloc(sizeof(*core_config->tp_info), GFP_KERNEL);
	if(ERR_ALLOC_MEM(core_config->tp_info))
	{
		DBG_ERR("Failed to allocate core_config->tp_info mem, %ld\n", PTR_ERR(core_config->tp_info));
		core_config_remove();
		return -ENOMEM;
	}

	for (; i < ARRAY_SIZE(ipio_chip_list); i++)
	{
		if (ipio_chip_list[i] == TP_TOUCH_IC)
		{
			core_config->chip_id = ipio_chip_list[i];
			core_config->chip_type = 0x0000;

			core_config->do_ic_reset = false;
			core_config->isEnableGesture = true;//false;

			if (core_config->chip_id == CHIP_TYPE_ILI7807)
			{
				core_config->slave_i2c_addr = ILI7807_SLAVE_ADDR;
				core_config->ice_mode_addr = ILI7807_ICE_MODE_ADDR;
				core_config->pid_addr = ILI7807_PID_ADDR;
			}
			else if (core_config->chip_id == CHIP_TYPE_ILI9881)
			{
				core_config->slave_i2c_addr = ILI9881_SLAVE_ADDR;
				core_config->ice_mode_addr = ILI9881_ICE_MODE_ADDR;
				core_config->pid_addr = ILI9881_PID_ADDR;
			}
			return 0;
		}
	}

	DBG_ERR("Can't find this chip in support list\n");
	return 0;
}
EXPORT_SYMBOL(core_config_init);

void core_config_remove(void)
{
	DBG_INFO("Remove core-config memebers\n");

	if(core_config != NULL)
	{
		if(core_config->tp_info != NULL)
			kfree(core_config->tp_info);

		kfree(core_config);
	}
}
EXPORT_SYMBOL(core_config_remove);
