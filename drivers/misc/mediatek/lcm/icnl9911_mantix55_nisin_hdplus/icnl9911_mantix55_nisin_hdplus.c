/*
 * Copyright (C) 2015 MediaTek Inc.
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

#define LOG_TAG "LCM"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_ID	 0x9911

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
    lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
    lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

/* static unsigned char lcd_id_pins_value = 0xFF; */
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1440)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH									(61880)
#define LCM_PHYSICAL_HEIGHT									(123770)
#define LCM_DENSITY											(320)

#define REGFLAG_DELAY		    0xFFFC
#define REGFLAG_UDELAY	        0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	    0xFFFE
#define REGFLAG_RESET_HIGH	    0xFFFF

struct LCM_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
    {REGFLAG_DELAY, 20, {} },
    {0x28, 0, {} },
    {REGFLAG_DELAY, 20, {} },
    {0x10, 0, {} },
    {REGFLAG_DELAY, 120, {} },
    {REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_init_setting[] = {
	//-------------  Display Initial Code Setting  -------------------------
	{0xF0,2 ,{0x5A,0x5A}},
	{0xF1,2 ,{0xA5,0xA5}},
	{0xB0,16,{0x21,0x54,0x76,0x54,0x66,0x66,0x33,0x33,0x0c,0x03,0x03,0x8c,0x03,0x03,0x0F,0x00}},
	{0xB1,16,{0x13,0xD4,0x02,0x86,0x00,0x01,0x01,0x88,0x01,0x01,0x53,0x00,0x00,0x00,0x00,0x00}},
	{0xB2,16,{0x67,0x2A,0x05,0x8A,0x65,0x02,0x08,0x20,0x30,0x91,0x22,0x33,0x44,0x00,0x18,0xA1}},
	{0xB3,16,{0x01,0x00,0x00,0x33,0x00,0x26,0x26,0xC0,0x3F,0xAA,0x33,0xC3,0xAA,0x30,0xC3,0xAA}},
	{0xB6,16,{0x0a,0x02,0x14,0x15,0x1b,0x02,0x02,0x02,0x02,0x13,0x11,0x02,0x02,0x0F,0x0D,0x05}},
	{0xB4,16,{0x0b,0x02,0x14,0x15,0x1b,0x02,0x02,0x02,0x02,0x12,0x10,0x02,0x02,0x0E,0x0C,0x04}},
	{0xBB,16,{0x00,0x00,0x00,0x00,0x02,0xFF,0xFC,0x0B,0x23,0x01,0x73,0x44,0x44,0x00,0x00,0x00}},
	{0xBC,10,{0x61,0x03,0xff,0xDE,0x72,0xE0,0x2E,0x04,0x88,0x3e}},
	{0xBD,16,{0x6E,0x0E,0x65,0x65,0x15,0x15,0x50,0x41,0x14,0x66,0x23,0x02,0x00,0x00,0x00,0x00}},
	{0xBE, 5,{0x60,0x60,0x50,0x60,0x77}},
	{0xC1,16,{0x70,0x68,0x0c,0x7c,0x04,0x0C,0x10,0x04,0x2A,0x31,0x00,0x07,0x10,0x10,0x00,0x00}},
	{0xC2, 1,{0x00}},
	{0xC3, 8,{0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x0d}},
	{0xC4, 8,{0xB4,0xA3,0xee,0x41,0x04,0x2F,0x00,0x00}},
	{0xC5,12,{0x07,0x1F,0x42,0x26,0x52,0x44,0x14,0x1A,0x04,0x00,0x0A,0x08}},
	{0xC6,16,{0x81,0x01,0x67,0x01,0x33,0xA0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xC7,16,{0x7C,0x60,0x4E,0x63,0x51,0x3C,0x2E,0x34,0x21,0x3E,0x40,0x32,0x4E,0x47,0x56,0x48}},
	{0xC8, 5,{0x35,0x5F,0x38,0x28,0x0C}},
	{0xC9,16,{0x7C,0x60,0x4E,0x63,0x51,0x3C,0x2E,0x34,0x21,0x3E,0x40,0x32,0x4E,0x47,0x56,0x48}},
	{0xCA, 5,{0x35,0x5F,0x38,0x28,0x0C}},
	{0xCB,11,{0x00,0x00,0x00,0x01,0x6C,0x00,0x33,0x00,0x17,0xFF,0xEF}},
	{0xF0,2 ,{0xB4,0x4B}},
	{0xD0,8 ,{0x80,0x0D,0xFF,0x0F,0x63,0x2B,0x08,0x08}},
	{0xD2,10,{0x41,0x0C,0x00,0x01,0x80,0x26,0x04,0x00,0x16,0x42}},
	{0xd5,1 ,{0x0f}},
	{0x35,1 ,{0x00}},
	{0xF0,2 ,{0xA5,0xA5}},
	{0xF1,2 ,{0x5A,0x5A}},
	{0x26,1 ,{0x01}},

	{0x11,1,{0x00}},
	{REGFLAG_DELAY,120,{}},

	{0x29,1,{0x00}},
	{REGFLAG_DELAY,10,{}},
	{REGFLAG_END_OF_TABLE,0x00,{}}
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
        unsigned int count, unsigned char force_update)
{
    unsigned int i;
    unsigned cmd;

    for (i = 0; i < count; i++) {
        cmd = table[i].cmd;

        switch (cmd) {
            case REGFLAG_DELAY:
                if (table[i].count <= 10)
                    MDELAY(table[i].count);
                else
                    MDELAY(table[i].count);
                break;
            case REGFLAG_UDELAY:
                UDELAY(table[i].count);
                break;
            case REGFLAG_END_OF_TABLE:
                break;
            default:
                dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type = LCM_TYPE_DSI;
    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    params->physical_width = LCM_PHYSICAL_WIDTH/1000;
    params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
    params->physical_width_um = LCM_PHYSICAL_WIDTH;
    params->physical_height_um = LCM_PHYSICAL_HEIGHT;
    params->density            = LCM_DENSITY;

    params->dsi.mode = SYNC_PULSE_VDO_MODE;
    params->dsi.switch_mode = CMD_MODE;
    params->dsi.switch_mode_enable = 0;

    /* DSI */
    /* Command mode setting */
    params->dsi.LANE_NUM = LCM_FOUR_LANE;
    /* The following defined the fomat for data coming from LCD engine. */
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

    /* Highly depends on LCD driver capability. */
    params->dsi.packet_size = 256;
    /* video mode timing */

    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch				= 12;
	params->dsi.vertical_frontporch				= 124;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active			= 4;
	params->dsi.horizontal_backporch				= 45;//55
	params->dsi.horizontal_frontporch				= 45;//55
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 251;

	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.ssc_disable = 1;
}

static void lcm_init_power(void)
{
    display_bias_enable();
}

static void lcm_suspend_power(void)
{
    display_bias_disable();
}

static void lcm_resume_power(void)
{
    display_bias_enable();
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(150);

    push_table(NULL, lcm_init_setting, sizeof(lcm_init_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
    push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(10);
    /*SET_RESET_PIN(0);
      MDELAY(10);*/
}

static void lcm_resume(void)
{
    lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	unsigned int array[4];
	unsigned short device_id;
	unsigned char buffer[4];

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

	array[0]=0x00033902;
	array[1]=0x005A5AF0;
	dsi_set_cmdq(array, 2, 1);

	array[0]=0x00033902;
	array[1]=0x00A5A5F1;
	dsi_set_cmdq(array, 2, 1);

	read_reg_v2(0xfa, buffer, 2);	
	device_id = (buffer[0]<<8) | (buffer[1]);

	//printk("-------------wuxiaotian----------- device_id = 0x%4x\n",device_id);
	return (LCM_ID == device_id)?1:0;

}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER icnl9911_mantix55_nisin_hdplus_lcm_drv = 
{
	.name			= "icnl9911_mantix55_nisin_hdplus",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
    .init_power     = lcm_init_power,
    .resume_power   = lcm_resume_power,
    .suspend_power  = lcm_suspend_power,
};
