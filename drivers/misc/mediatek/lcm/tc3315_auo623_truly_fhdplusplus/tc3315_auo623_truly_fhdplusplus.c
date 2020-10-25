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

#define LCM_ID_HX83112A (0xf5)

static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;
bool g_gesture_wakeup_enable = false;

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
#define read_reg_v2_pd(cmd0, cmd1, buffer, buffer_size) \
    lcm_util.dsi_dcs_read_lcm_reg_v2_parade(cmd0, cmd1, buffer, buffer_size)

/* static unsigned char lcd_id_pins_value = 0xFF; */
#define FRAME_WIDTH										(1080)
#define FRAME_HEIGHT									(2280)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH									(67720)
#define LCM_PHYSICAL_HEIGHT									(142960)
#define LCM_DENSITY											(480)

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

    {0x11, 0, {} },
    {REGFLAG_DELAY, 120, {} },
    {0x29, 0, {} },
    {REGFLAG_DELAY, 20, {} },
    {0xdf,3,{0xe0,0x1d,0xff}},
    {0xdf,3,{0xe0,0x1e,0xff}},
    {0xdf,3,{0xe4,0x20,0x04}},
    {0xdf,3,{0xe4,0x21,0x09}},
    {0xdf,3,{0xe4,0x22,0x23}},
    {0xdf,3,{0xe4,0x23,0x31}},
    {0xdf,3,{0xe4,0x24,0x0a}},
    {0xdf,3,{0xe4,0x25,0x13}},
    {0xdf,3,{0xe4,0x26,0x1b}},
    {0xdf,3,{0xe4,0x27,0x21}},
    {0xdf,3,{0xe4,0x28,0x2d}},
    {0xdf,3,{0xe4,0x29,0x37}},
    {0xdf,3,{0xe4,0x2a,0x2a}},
    {0xdf,3,{0xe4,0x2b,0x07}},
    {0xdf,3,{0xe4,0x2c,0x11}},
    {0xdf,3,{0xe4,0x2d,0x20}},
    {0xdf,3,{0xe4,0x2e,0x22}},
    {0xdf,3,{0xe4,0x2f,0x30}},
    {0xdf,3,{0xe4,0x30,0x38}},
    {0xdf,3,{0xe4,0x31,0x1f}},
    {0xdf,3,{0xe4,0x32,0x09}},
    {0xdf,3,{0xe4,0x33,0x14}},
    {0xdf,3,{0xe4,0x34,0x1e}},
    {0xdf,3,{0xe4,0x35,0x25}},
    {0xdf,3,{0xe4,0x36,0x2c}},
    {0xdf,3,{0xe4,0x37,0x35}},
    {0xdf,3,{0xe4,0x38,0x30}},
    {0xdf,3,{0xe4,0x39,0x34}},
    {0xdf,3,{0xe4,0x3a,0x37}},
    {0xdf,3,{0xe4,0x40,0x04}},
    {0xdf,3,{0xe4,0x41,0x09}},
    {0xdf,3,{0xe4,0x42,0x23}},
    {0xdf,3,{0xe4,0x43,0x31}},
    {0xdf,3,{0xe4,0x44,0x0a}},
    {0xdf,3,{0xe4,0x45,0x13}},
    {0xdf,3,{0xe4,0x46,0x1b}},
    {0xdf,3,{0xe4,0x47,0x21}},
    {0xdf,3,{0xe4,0x48,0x2d}},
    {0xdf,3,{0xe4,0x49,0x37}},
    {0xdf,3,{0xe4,0x4a,0x2a}},
    {0xdf,3,{0xe4,0x4b,0x07}},
    {0xdf,3,{0xe4,0x4c,0x11}},
    {0xdf,3,{0xe4,0x4d,0x20}},
    {0xdf,3,{0xe4,0x4e,0x22}},
    {0xdf,3,{0xe4,0x4f,0x30}},
    {0xdf,3,{0xe4,0x50,0x38}},
    {0xdf,3,{0xe4,0x51,0x1f}},
    {0xdf,3,{0xe4,0x52,0x09}},
    {0xdf,3,{0xe4,0x53,0x14}},
    {0xdf,3,{0xe4,0x54,0x1e}},
    {0xdf,3,{0xe4,0x55,0x25}},
    {0xdf,3,{0xe4,0x56,0x2c}},
    {0xdf,3,{0xe4,0x57,0x35}},
    {0xdf,3,{0xe4,0x58,0x30}},
    {0xdf,3,{0xe4,0x59,0x34}},
    {0xdf,3,{0xe4,0x5a,0x37}},
    {0xdf,3,{0xe1,0x00,0x9f}},
    {0xdf,3,{0xe1,0x01,0x00}},
    {0xdf,3,{0xe1,0x02,0x00}},
    {0xdf,3,{0xe1,0x00,0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {} }
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

    params->dsi.vertical_sync_active = 8;
    params->dsi.vertical_backporch = 6;
    params->dsi.vertical_frontporch = 51;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 32;
    params->dsi.horizontal_backporch = 40;
    params->dsi.horizontal_frontporch = 8;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    params->dsi.PLL_CLOCK = 540;
    params->dsi.ssc_disable = 0;
    //	params->dsi.cont_clock=1;
    params->dsi.clk_lp_per_line_enable = 0;
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 0;
    params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;
}

static void lcm_init_power(void)
{
    SET_RESET_PIN(0);
    display_bias_disable();
    MDELAY(150);
    display_bias_enable();
    MDELAY(20);
}

static void lcm_suspend_power(void)
{
    if (!g_gesture_wakeup_enable) {
        LCM_LOGD("tc3315 gesture_wakeup disable, suspend power\n");
        display_bias_disable();
    } else {
        LCM_LOGD("tc3315 gesture_wakeup enable, suspend power do nothing\n");
    }
}

static void lcm_resume_power(void)
{
    if (!g_gesture_wakeup_enable) {
        LCM_LOGD("tc3315 gesture_wakeup disable, resume power\n");
        display_bias_enable();
    } else {
        LCM_LOGD("tc3315 gesture_wakeup enable, resume power do nothing\n");
    }
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
    unsigned char buffer[1];
    unsigned int array[16];
    unsigned int id = 0;
    unsigned int id0 = 0;
    unsigned int id1 = 0;

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

    array[0] = 0x00023700;/* read id return two bytes of data */
    dsi_set_cmdq(array, 1, 1);
    MDELAY(10);

    read_reg_v2_pd(0xe4,0xf3,buffer, 1);
    id0 = buffer[0];
    read_reg_v2_pd(0xe4,0xf2,buffer, 1);
    id1 = buffer[0];

    id = id0 << 8 | id1;
#if defined(BUILD_LK)
    printf("%s,tc3315_auo623_truly_fhdplusplus id0 = 0x%x,id1 = 0x%x,id = 0x%x\n", __func__, id0,id1,id);
#else
    printk("%s,tc3315_auo623_truly_fhdplusplus id0 = 0x%x,id1 = 0x%x,id = 0x%x\n", __func__, id0,id1,id);
#endif

    return ((id == 0x3315) ? 1 : 0);
}

LCM_DRIVER tc3315_auo623_truly_fhdplusplus_lcm_drv = {
    .name           = "tc3315_auo623_truly_fhdplusplus",
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
