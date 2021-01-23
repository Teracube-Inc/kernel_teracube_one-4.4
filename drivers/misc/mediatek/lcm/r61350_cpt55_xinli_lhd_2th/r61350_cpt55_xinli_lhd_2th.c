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

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"


#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif

#ifdef BUILD_LK
#define printk(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define printk(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_ID_R61350 (0xff)

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

#define set_gpio_lcd_enp(cmd) \
  lcm_util.set_gpio_lcd_enp_bias(cmd)

/* static unsigned char lcd_id_pins_value = 0xFF; */
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1440)

#ifndef CONFIG_FPGA_EARLY_PORTING
//#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
#endif
#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER

//static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE (0)
#endif

#ifndef TRUE
#define TRUE  (1)
#endif

//#define GPIO_LCM_ID_PIN GPIO_LCD_ID_PIN
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(gpio_num,val)    						(lcm_util.set_gpio_out((gpio_num),(val)))


#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                                                                   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)  
#define GPIO_LCD_ID_PIN  (0x80 | 0x80000000)
#define GPIO_TP_RST_1         (GPIO8 | 0x80000000)
#define GPIO_LCM_RST_1         (GPIO83 | 0x80000000)
#define GPIO_LCM_RST           (83+343)
#define GPIO_LCM_ENP_1         (GPIO117 | 0x80000000)
#define GPIO_LCM_ENN_1         (GPIO118 | 0x80000000)
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

#ifdef BUILD_LK
static void lcm_set_enp_bias_lk(int output)
{
    mt_set_gpio_mode(GPIO_LCM_ENP_1, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_ENP_1, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_ENP_1, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);

    mt_set_gpio_mode(GPIO_LCM_ENN_1, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_ENN_1, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_ENN_1, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}
static void lcm_set_rst_lk(int output)
{
    mt_set_gpio_mode(GPIO_LCM_RST_1, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_RST_1, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_RST_1, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);

    mt_set_gpio_mode(GPIO_LCM_RST_1, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_RST_1, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_RST_1, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}

#endif
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
  memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
  memset(params, 0, sizeof(LCM_PARAMS));

  params->type   = LCM_TYPE_DSI;

  params->width  = FRAME_WIDTH;
  params->height = FRAME_HEIGHT;

  // enable tearing-free
  params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
  params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

  //params->dsi.mode   = SYNC_PULSE_VDO_MODE;
  params->dsi.mode   = BURST_VDO_MODE;	

  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM				= LCM_FOUR_LANE;

  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;

  params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

  params->dsi.vertical_sync_active				= 8; //8;
  params->dsi.vertical_backporch					= 7;  //16;
  params->dsi.vertical_frontporch					= 15;  //50; //200; //16;
  params->dsi.vertical_active_line				= FRAME_HEIGHT; 

  params->dsi.horizontal_sync_active				= 20; //40;
  params->dsi.horizontal_backporch				= 60; //100;
  params->dsi.horizontal_frontporch				= 80; //70;
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


  params->dsi.PLL_CLOCK = 251;
}

static void lcm_init(void)
{
  unsigned int data_array[16];
#ifdef BUILD_LK
  lcm_set_enp_bias_lk(1);
#else
  set_gpio_lcd_enp(1);
#endif

#ifdef BUILD_LK
  lcm_set_rst_lk(1);
  MDELAY(15);
  lcm_set_rst_lk(0);
  MDELAY(10);
  lcm_set_rst_lk(1);
  MDELAY(120);
#else
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(15);
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(120);
#endif

  data_array[0] = 0x00110500;
  dsi_set_cmdq(data_array, 1, 1); 
  MDELAY(200);//200

  data_array[0] = 0x00290500;
  dsi_set_cmdq(data_array, 1, 1); 
  MDELAY(20);

}

static void lcm_suspend(void)
{
  unsigned int data_array[16];
  //unsigned char buffer[2];

  data_array[0]=0x00280500; // Display Off
  dsi_set_cmdq(data_array, 1, 1); 

  MDELAY(10);
  data_array[0] = 0x00100500; // Sleep In
  dsi_set_cmdq(data_array, 1, 1); 

  MDELAY(120);
#ifdef BUILD_LK
  lcm_set_enp_bias_lk(0);
  lcm_set_rst_lk(0);
#else
  set_gpio_lcd_enp(0);
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
#endif
}

static void lcm_resume(void)
{
  lcm_init();
}

static unsigned int lcm_compare_id(void)
{
    unsigned int data_array[16]; 
    unsigned int array[16]; 

    unsigned int id = 0xFF;
    unsigned int id1 = 0xFF;
    unsigned char read_buf[16];

#ifdef BUILD_LK
    lcm_set_enp_bias_lk(1);
#else
    set_gpio_lcd_enp(1);
#endif

#ifdef BUILD_LK
  lcm_set_rst_lk(1);
  MDELAY(15);
  lcm_set_rst_lk(0);
  MDELAY(10);
  lcm_set_rst_lk(1);
  MDELAY(50);
#else
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(15);
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(50);
#endif
    data_array[0] = 0x00022902; 
    data_array[1] = 0x000004b0; 
    dsi_set_cmdq(data_array, 2, 1); 
    array[0] = 0x00073700; 
    dsi_set_cmdq(array, 1, 1); 
    MDELAY(2); 

    read_reg_v2(0x04, read_buf, 7);
    id = read_buf[6];
    id1 = read_buf[0];
    

#if defined(BUILD_LK)
    printf("%s,[LK] r61350_cpt55_xinli_lhd id = 0x%x\n", __func__, id);
#else
    printk("%s, r61350_cpt55_xinli_lhd id1 = 0x%x\n", __func__, id);
#endif

    if (id==LCM_ID_R61350 && id1==0x17)
        return 1;
    else
        return 0;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER r61350_cpt55_xinli_lhd_2th_lcm_drv = 
{
  .name          = "r61350_cpt55_xinli_lhd_2th",
  .set_util_funcs 	= lcm_set_util_funcs,
  .get_params     	= lcm_get_params,
  .init           	= lcm_init,
  .suspend        	= lcm_suspend,
  .resume         	= lcm_resume,
  .compare_id     	= lcm_compare_id,
};
