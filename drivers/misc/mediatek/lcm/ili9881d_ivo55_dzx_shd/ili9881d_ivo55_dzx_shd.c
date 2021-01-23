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
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH  (600)
#define FRAME_HEIGHT (1280)

#define LCM_PHYSICAL_WIDTH                                                                     (64440)
#define LCM_PHYSICAL_HEIGHT                                                                    (140160)


#ifndef CONFIG_FPGA_EARLY_PORTING
#endif
#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE (0)
#endif

#ifndef TRUE
#define TRUE  (1)
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(gpio_num,val)    						(lcm_util.set_gpio_out((gpio_num),(val)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
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

struct LCM_setting_table
{
  unsigned char cmd;
  unsigned char count;
  unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] ={
  {0xff, 3,{0x98,0x81,0x03}},
  {0x01, 1,{0x00}},
  {0x02, 1,{0x00}},
  {0x03, 1,{0x73}},
  {0x04, 1,{0x73}},
  {0x05, 1,{0x00}},
  {0x06, 1,{0x06}},
  {0x07, 1,{0x02}},
  {0x08, 1,{0x00}},
  {0x09, 1,{0x00}},
  {0x0a, 1,{0x00}},
  {0x0b, 1,{0x00}},
  {0x0c, 1,{0x00}},
  {0x0d, 1,{0x00}},
  {0x0e, 1,{0x00}},
  {0x0f, 1,{0x00}},
  {0x10, 1,{0x00}},
  {0x11, 1,{0x00}},
  {0x12, 1,{0x00}},
  {0x13, 1,{0x00}},
  {0x14, 1,{0x00}},
  {0x15, 1,{0x08}},
  {0x16, 1,{0x08}},
  {0x17, 1,{0x00}},
  {0x18, 1,{0x08}},
  {0x19, 1,{0x00}},
  {0x1a, 1,{0x00}},
  {0x1b, 1,{0x00}},
  {0x1c, 1,{0x00}},
  {0x1d, 1,{0x00}},
  {0x1e, 1,{0xc0}},
  {0x1f, 1,{0x00}},
  {0x20, 1,{0x03}},
  {0x21, 1,{0x02}},
  {0x22, 1,{0x00}},
  {0x23, 1,{0x00}},
  {0x24, 1,{0x00}},
  {0x25, 1,{0x00}},
  {0x26, 1,{0x00}},
  {0x27, 1,{0x00}},
  {0x28, 1,{0x33}},
  {0x29, 1,{0x02}},
  {0x2a, 1,{0x00}},
  {0x2b, 1,{0x00}},
  {0x2c, 1,{0x00}},
  {0x2d, 1,{0x00}},
  {0x2e, 1,{0x00}},
  {0x2f, 1,{0x00}},
  {0x30, 1,{0x00}},
  {0x31, 1,{0x00}},
  {0x32, 1,{0x00}},
  {0x33, 1,{0x00}},
  {0x34, 1,{0x00}},
  {0x35, 1,{0x00}},
  {0x36, 1,{0x00}},
  {0x37, 1,{0x00}},
  {0x38, 1,{0x00}},
  {0x39, 1,{0x38}},
  {0x3A, 1,{0x00}},
  {0x3B, 1,{0x40}},
  {0x3C, 1,{0x00}},
  {0x3D, 1,{0x00}},
  {0x3E, 1,{0x00}},
  {0x3F, 1,{0x00}},
  {0x40, 1,{0x38}},
  {0x41, 1,{0x88}},
  {0x42, 1,{0x00}},
  {0x43, 1,{0x00}},
  {0x44, 1,{0x3F}},
  {0x45, 1,{0x20}},
  {0x46, 1,{0x00}},
  {0x50, 1,{0x01}},
  {0x51, 1,{0x23}},
  {0x52, 1,{0x45}},
  {0x53, 1,{0x67}},
  {0x54, 1,{0x89}},
  {0x55, 1,{0xab}},
  {0x56, 1,{0x01}},
  {0x57, 1,{0x23}},
  {0x58, 1,{0x45}},
  {0x59, 1,{0x67}},
  {0x5a, 1,{0x89}},
  {0x5b, 1,{0xab}},
  {0x5c, 1,{0xcd}},
  {0x5d, 1,{0xef}},
  {0x5e, 1,{0x10}},
  {0x5f, 1,{0x02}},
  {0x60, 1,{0x02}},
  {0x61, 1,{0x02}},
  {0x62, 1,{0x09}},
  {0x63, 1,{0x08}},
  {0x64, 1,{0x0F}},
  {0x65, 1,{0x0E}},
  {0x66, 1,{0x0D}},
  {0x67, 1,{0x0C}},
  {0x68, 1,{0x02}},
  {0x69, 1,{0x02}},
  {0x6a, 1,{0x02}},
  {0x6b, 1,{0x02}},
  {0x6c, 1,{0x02}},
  {0x6d, 1,{0x02}},
  {0x6e, 1,{0x02}},
  {0x6f, 1,{0x02}},
  {0x70, 1,{0x02}},
  {0x71, 1,{0x06}},
  {0x72, 1,{0x07}},
  {0x73, 1,{0x02}},
  {0x74, 1,{0x02}},
  {0x75, 1,{0x02}},
  {0x76, 1,{0x02}},
  {0x77, 1,{0x02}},
  {0x78, 1,{0x06}},
  {0x79, 1,{0x07}},
  {0x7a, 1,{0x0F}},
  {0x7b, 1,{0x0C}},
  {0x7c, 1,{0x0D}},
  {0x7d, 1,{0x0E}},
  {0x7e, 1,{0x02}},
  {0x7f, 1,{0x02}},
  {0x80, 1,{0x02}},
  {0x81, 1,{0x02}},
  {0x82, 1,{0x02}},
  {0x83, 1,{0x02}},
  {0x84, 1,{0x02}},
  {0x85, 1,{0x02}},
  {0x86, 1,{0x02}},
  {0x87, 1,{0x09}},
  {0x88, 1,{0x08}},
  {0x89, 1,{0x02}},
  {0x8A, 1,{0x02}},
  {0xFF, 3,{0x98,0x81,0x04}},
  //{0x00, 1,{0x00}},//3lane add 00 //4lane del 00
  {0x6D, 1,{0x08}},
  {0x6F, 1,{0x05}},
  {0x70, 1,{0x00}},
  {0x71, 1,{0x00}},
  {0x66, 1,{0xFE}},
  {0x82, 1,{0x12}},
  {0x84, 1,{0x11}},
  {0x85, 1,{0x0E}},
  {0x32, 1,{0xAC}},
  {0x8C, 1,{0x80}},
  {0x3C, 1,{0xF5}},
  {0x3A, 1,{0x24}},
  {0xB5, 1,{0x07}},
  {0x31, 1,{0x45}},
  {0x88, 1,{0x33}},
  {0x89, 1,{0xBA}},
  {0x38, 1,{0x01}},
  {0x39, 1,{0x00}},
  {0xFF, 3,{0x98,0x81,0x01}},
  {0x22, 1,{0x0a}},
  {0x31, 1,{0x00}},
  {0x41, 1,{0x24}},
  {0x53, 1,{0x2C}},
  {0x55, 1,{0x2B}},
  {0x50, 1,{0x73}},
  {0x51, 1,{0x73}},
  {0x60, 1,{0x1D}},
  {0x61, 1,{0x00}},
  {0x62, 1,{0x0D}},
  {0x63, 1,{0x00}},
  {0xB6, 1,{0x09}},
  {0xA0, 1,{0x00}},
  {0xA1, 1,{0x1C}},
  {0xA2, 1,{0x2E}},
  {0xA3, 1,{0x17}},
  {0xA4, 1,{0x1C}},
  {0xA5, 1,{0x31}},
  {0xA6, 1,{0x25}},
  {0xA7, 1,{0x25}},
  {0xA8, 1,{0xA0}},
  {0xA9, 1,{0x1C}},
  {0xAA, 1,{0x28}},
  {0xAB, 1,{0x81}},
  {0xAC, 1,{0x1C}},
  {0xAD, 1,{0x1C}},
  {0xAE, 1,{0x51}},
  {0xAF, 1,{0x25}},
  {0xB0, 1,{0x2A}},
  {0xB1, 1,{0x4D}},
  {0xB2, 1,{0x59}},
  {0xB3, 1,{0x23}},
  {0xC0, 1,{0x00}},
  {0xC1, 1,{0x1C}},
  {0xC2, 1,{0x2E}},
  {0xC3, 1,{0x17}},
  {0xC4, 1,{0x1C}},
  {0xC5, 1,{0x31}},
  {0xC6, 1,{0x25}},
  {0xC7, 1,{0x25}},
  {0xC8, 1,{0xA0}},
  {0xC9, 1,{0x1C}},
  {0xCA, 1,{0x28}},
  {0xCB, 1,{0x81}},
  {0xCC, 1,{0x1C}},
  {0xCD, 1,{0x1C}},
  {0xCE, 1,{0x51}},
  {0xCF, 1,{0x25}},
  {0xD0, 1,{0x2A}},
  {0xD1, 1,{0x4D}},
  {0xD2, 1,{0x59}},
  {0xD3, 1,{0x23}},
  {0xFF, 3,{0x98,0x81,0x00}},
  {0x35,01,{0x00}}, 
  {0x11,1,{0x00}},        // Sleep-Out
  {REGFLAG_DELAY, 120, {}},
  {0x29,1, {0x00}},       // Display On
  {REGFLAG_DELAY, 20, {}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_sleep_mode_in_setting[] =
{
	{0xFF, 3,{0x98,0x81,0x01}}, 
	{0x53, 1,{0x0F}},//2C
	{0xb3, 1,{0x3F}},
	{0xd3, 1,{0x3F}},
	{0xFF, 3,{0x98,0x81,0x04}}, 
	{0x2C, 1,{0x02}},
	{0x2F, 1,{0x01}},		
  {REGFLAG_DELAY, 100, {}},	
  // Display off sequence
  {0xFF, 3,{0x98,0x81,0x00}}, 
  {0x28, 0, {0x00}},
  {REGFLAG_DELAY, 120, {}},

  // Sleep Mode On
  {0x10, 0, {0x00}},
  {REGFLAG_DELAY, 200, {}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
  unsigned int i;

  for(i = 0; i < count; i++)
  {

    unsigned cmd;
    cmd = table[i].cmd;

    switch (cmd)
    {

      case REGFLAG_DELAY :
        MDELAY(table[i].count);
        break;

      case REGFLAG_END_OF_TABLE :
        break;

      default:
        dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        //MDELAY(2);
    }
  }

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
  // enable tearing-free
  //params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
  //params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

  params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;

  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM = LCM_FOUR_LANE;
  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

  params->dsi.intermediat_buffer_num = 0;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

  params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
  params->dsi.word_count = 720 * 3;

  params->dsi.vertical_sync_active				= 8;
  params->dsi.vertical_backporch					= 24;
  params->dsi.vertical_frontporch					= 16;
  params->dsi.vertical_active_line				= FRAME_HEIGHT; 

  params->dsi.horizontal_sync_active				= 20;
  params->dsi.horizontal_backporch				= 80;
  params->dsi.horizontal_frontporch				= 80;
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


  params->dsi.HS_TRAIL=20;
  params->dsi.PLL_CLOCK = 208;
  params->dsi.ssc_disable = 1;
  //params->dsi.clk_lp_per_line_enable = 1;
  params->dsi.esd_check_enable = 1;
  params->dsi.customization_esd_check_enable = 1;
  params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
  params->dsi.lcm_esd_check_table[0].count = 1;
  params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
  params->dsi.noncont_clock = 0;
}

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
static int adc_read_vol(void)
{

  int adc[1];
  int data[4] ={0,0,0,0};
  int sum = 0;
  int adc_vol=0;
  int num = 0;

  for(num=0;num<10;num++)
  {
    IMM_GetOneChannelValue(12, data, adc);
    sum+=(data[0]*100+data[1]);
  }
  adc_vol = sum/10;

#if defined(BUILD_LK)
  printf("ili9881c adc_vol is %d\n",adc_vol);
#else
  printk("ili9881c adc_vol is %d\n",adc_vol);
#endif
  return (adc_vol < 75) ? 0: 1;
}
static void lcm_init(void)
{
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

  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
  /*unsigned int data_array[16];

  data_array[0]=0x00280500;
  dsi_set_cmdq(data_array, 1, 1);
  MDELAY(60);
  data_array[0]=0x00100500;
  dsi_set_cmdq(data_array, 1, 1);
  MDELAY(120);*/
  
  push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
#ifdef BUILD_LK
  lcm_set_enp_bias_lk(0);
  lcm_set_rst_lk(0);
  //lcm_set_rst_lk(1);
#else
  printk("%s\n",__func__);
  set_gpio_lcd_enp(0);
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
  //gpio_set_value_cansleep(GPIO_LCM_RST, 1);
#endif


  //adc_read_vol();
}

static void lcm_resume(void)
{
  lcm_init();
  //adc_read_vol();
}

static unsigned int lcm_compare_id(void)
{

  int array[4];
  char buffer[3];
  int id=0;

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


  //{0x39, 0xFF, 5, { 0xFF,0x98,0x06,0x04,0x01}}, // Change to Page 1 CMD
  array[0] = 0x00043902;
  array[1] = 0x068198FF;
  dsi_set_cmdq(array, 2, 1);

  array[0] = 0x00013700;
  dsi_set_cmdq(array, 1, 1);
  read_reg_v2(0xF2, &buffer[0], 1);  //0xOd

  id = buffer[0];
#if defined(BUILD_LK)
  printf("%s, [ili9881d_ivo50_txd_hd]  buffer[0] = [0x%x]  ID = [0x%x]\n",__func__,buffer[0], id);
#else
  printk("%s, [ili9881d_ivo50_txd_hd]  buffer[0] = [0x%x]  ID = [0x%x]\n",__func__,buffer[0], id);
#endif


  return (0x0d == (id+adc_read_vol()))?1:0;

}

LCM_DRIVER ili9881d_ivo55_dzx_shd_lcm_drv =
{
  .name			  = "ili9881d_ivo55_dzx_shd",
  .set_util_funcs = lcm_set_util_funcs,
  .get_params     = lcm_get_params,
  .init           = lcm_init,
  .suspend        = lcm_suspend,
  .resume         = lcm_resume,
  .compare_id     = lcm_compare_id,
};
