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
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
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
#ifndef BUILD_LK
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
#endif
/* static unsigned char lcd_id_pins_value = 0xFF; */
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1512)
#define LCM_DENSITY                                     (320)

#define LCM_PHYSICAL_WIDTH                                  (64000)
#define LCM_PHYSICAL_HEIGHT                                 (134000)

#define LCM_ID			(0x93)
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

static unsigned int lcm_compare_id(void);
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

#define GPIO_LCM_RST_1         (GPIO83 | 0x80000000)
#define GPIO_LCM_RST           (83+343)
#define GPIO_LCM_ENP_1         (GPIO106 | 0x80000000)
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

#ifdef BUILD_LK
static void lcm_set_enp_bias_lk(int output)
{
  mt_set_gpio_mode(GPIO_LCM_ENP_1, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_ENP_1, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_ENP_1, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}
static void lcm_set_rst_lk(int output)
{
  mt_set_gpio_mode(GPIO_LCM_RST_1, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_RST_1, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_RST_1, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}

#endif
struct LCM_setting_table
{
  unsigned char cmd;
  unsigned char count;
  unsigned char para_list[64];
};
static struct LCM_setting_table lcm_initialization_setting[] =
{
{0xE0,1,{0x00}},

{0xE1,1,{0x93}},
{0xE2,1,{0x65}},
{0xE3,1,{0xF8}},
{0x80,1,{0x03}},    	//02 3lane   03  4lane

{0xE0,1,{0x01}},

{0x24,1,{0xFE}},
{0x00,1,{0x00}},
{0x01,1,{0x70}},     	//VCOM
{0x03,1,{0x00}},
{0x04,1,{0x75}},

{0x17,1,{0x00}},
{0x18,1,{0xEF}},
{0x19,1,{0x01}},
{0x1A,1,{0x00}},
{0x1B,1,{0xEF}},
{0x1C,1,{0x01}},

{0x25,1,{0x20}},

{0x37,1,{0x09}},

{0x38,1,{0x04}},
{0x39,1,{0x08}},
{0x3A,1,{0x12}},
{0x3C,1,{0x68}},
{0x3D,1,{0xFF}},
{0x3E,1,{0xFF}},
{0x3F,1,{0xFF}},

{0x40,1,{0x04}},
{0x41,1,{0xBD}},
{0x42,1,{0x6B}},
{0x43,1,{0x12}},
{0x44,1,{0x0F}},
{0x45,1,{0x46}},

{0x55,1,{0x0F}},      //power mode 0F-3power  01-2power
{0x56,1,{0x01}},
{0x57,1,{0x69}},
{0x58,1,{0x0A}},
{0x59,1,{0x0A}},
{0x5A,1,{0x29}},
{0x5B,1,{0x10}},

{0x5D,1,{0x6F}},
{0x5E,1,{0x49}},
{0x5F,1,{0x37}},
{0x60,1,{0x2A}},
{0x61,1,{0x26}},
{0x62,1,{0x18}},
{0x63,1,{0x1D}},
{0x64,1,{0x09}},
{0x65,1,{0x23}},
{0x66,1,{0x23}},
{0x67,1,{0x23}},
{0x68,1,{0x40}},
{0x69,1,{0x2D}},
{0x6A,1,{0x34}},
{0x6B,1,{0x26}},
{0x6C,1,{0x24}},
{0x6D,1,{0x1A}},
{0x6E,1,{0x0F}},
{0x6F,1,{0x09}},
{0x70,1,{0x6F}},
{0x71,1,{0x49}},
{0x72,1,{0x37}},
{0x73,1,{0x2A}},
{0x74,1,{0x26}},
{0x75,1,{0x18}},
{0x76,1,{0x1D}},
{0x77,1,{0x09}},
{0x78,1,{0x23}},
{0x79,1,{0x23}},
{0x7A,1,{0x23}},
{0x7B,1,{0x40}},
{0x7C,1,{0x2D}},
{0x7D,1,{0x34}},
{0x7E,1,{0x26}},
{0x7F,1,{0x24}},
{0x80,1,{0x1A}},
{0x81,1,{0x0F}},
{0x82,1,{0x09}},

{0xE0,1,{0x02}},

{0x00,1,{0x1E}},
{0x01,1,{0x1F}},
{0x02,1,{0x57}},
{0x03,1,{0x58}},
{0x04,1,{0x44}},
{0x05,1,{0x46}},
{0x06,1,{0x48}},
{0x07,1,{0x4A}},
{0x08,1,{0x40}},
{0x09,1,{0x1D}},
{0x0A,1,{0x1D}},
{0x0B,1,{0x1D}},
{0x0C,1,{0x1D}},
{0x0D,1,{0x1D}},
{0x0E,1,{0x1D}},
{0x0F,1,{0x50}},
{0x10,1,{0x1F}},
{0x11,1,{0x1F}},
{0x12,1,{0x1F}},
{0x13,1,{0x1F}},
{0x14,1,{0x1F}},
{0x15,1,{0x1F}},

{0x16,1,{0x1E}},
{0x17,1,{0x1F}},
{0x18,1,{0x57}},
{0x19,1,{0x58}},
{0x1A,1,{0x45}},
{0x1B,1,{0x47}},
{0x1C,1,{0x49}},
{0x1D,1,{0x4B}},
{0x1E,1,{0x41}},
{0x1F,1,{0x1D}},
{0x20,1,{0x1D}},
{0x21,1,{0x1D}},
{0x22,1,{0x1D}},
{0x23,1,{0x1D}},
{0x24,1,{0x1D}},
{0x25,1,{0x51}},
{0x26,1,{0x1F}},
{0x27,1,{0x1F}},
{0x28,1,{0x1F}},
{0x29,1,{0x1F}},
{0x2A,1,{0x1F}},
{0x2B,1,{0x1F}},

{0x2C,1,{0x1F}},
{0x2D,1,{0x1E}},
{0x2E,1,{0x17}},
{0x2F,1,{0x18}},
{0x30,1,{0x0B}},
{0x31,1,{0x09}},
{0x32,1,{0x07}},
{0x33,1,{0x05}},
{0x34,1,{0x11}},
{0x35,1,{0x1F}},
{0x36,1,{0x1F}},
{0x37,1,{0x1F}},
{0x38,1,{0x1F}},
{0x39,1,{0x1F}},
{0x3A,1,{0x1F}},
{0x3B,1,{0x01}},
{0x3C,1,{0x1F}},
{0x3D,1,{0x1F}},
{0x3E,1,{0x1F}},
{0x3F,1,{0x1F}},
{0x40,1,{0x1F}},
{0x41,1,{0x1F}},

{0x42,1,{0x1F}},
{0x43,1,{0x1E}},
{0x44,1,{0x17}},
{0x45,1,{0x18}},
{0x46,1,{0x0A}},
{0x47,1,{0x08}},
{0x48,1,{0x06}},
{0x49,1,{0x04}},
{0x4A,1,{0x10}},
{0x4B,1,{0x1F}},
{0x4C,1,{0x1F}},
{0x4D,1,{0x1F}},
{0x4E,1,{0x1F}},
{0x4F,1,{0x1F}},
{0x50,1,{0x1F}},
{0x51,1,{0x00}},
{0x52,1,{0x1F}},
{0x53,1,{0x1F}},
{0x54,1,{0x1F}},
{0x55,1,{0x1F}},
{0x56,1,{0x1F}},
{0x57,1,{0x1F}},

{0x58,1,{0x40}},
{0x59,1,{0x00}},
{0x5A,1,{0x00}},
{0x5B,1,{0x10}},
{0x5C,1,{0x0B}},
{0x5D,1,{0x30}},
{0x5E,1,{0x01}},
{0x5F,1,{0x02}},
{0x60,1,{0x30}},
{0x61,1,{0x03}},
{0x62,1,{0x04}},
{0x63,1,{0x06}},
{0x64,1,{0x54}},
{0x65,1,{0x55}},
{0x66,1,{0xF7}},
{0x67,1,{0x73}},
{0x68,1,{0x0D}},
{0x69,1,{0x06}},
{0x6A,1,{0x54}},
{0x6B,1,{0x00}},

{0x6C,1,{0x00}},
{0x6D,1,{0x04}},
{0x6E,1,{0x04}},
{0x6F,1,{0x08}},
{0x70,1,{0x00}},
{0x71,1,{0x00}},
{0x72,1,{0x06}},
{0x73,1,{0x7B}},
{0x74,1,{0x00}},
{0x75,1,{0xBC}},
{0x76,1,{0x01}},
{0x77,1,{0x0E}},
{0x78,1,{0x0C}},
{0x79,1,{0x00}},
{0x7A,1,{0x00}},
{0x7B,1,{0x00}},
{0x7C,1,{0x00}},
{0x7D,1,{0x03}},
{0x7E,1,{0x7B}},

{0xE0,1,{0x04}},
{0x09,1,{0x10}},
{0x0E,1,{0x4A}},

{0xE0,1,{0x00}},
{0x35,1,{0x00}},
{0x11,1,{0x00}},
{REGFLAG_DELAY, 120,{}},
{0x29,1,{0x00}},
{REGFLAG_DELAY, 20,{}},
{REGFLAG_END_OF_TABLE, 0x00,{}}

};
/*
static struct LCM_setting_table lcm_sleep_mode_in_setting[] =
{
  // Display off sequence
  {0x28, 0, {0x00}},
  {REGFLAG_DELAY, 120, {}},

  // Sleep Mode On
  {0x10, 0, {0x00}},
  {REGFLAG_DELAY, 200, {}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/
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

  params->physical_width = LCM_PHYSICAL_WIDTH/1000;
  params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
  params->physical_width_um = LCM_PHYSICAL_WIDTH;
  params->physical_height_um = LCM_PHYSICAL_HEIGHT;
//  params->density = LCM_DENSITY;


  // enable tearing-free
  params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
  params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

  params->dsi.mode   = SYNC_PULSE_VDO_MODE;
  //params->dsi.mode   = BURST_VDO_MODE;	

  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM				= LCM_FOUR_LANE;

  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;

  params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

  params->dsi.vertical_sync_active				= 4; //8;
  params->dsi.vertical_backporch					= 12 ; //20;  //16;
  params->dsi.vertical_frontporch					= 18;  //50; //200; //16;
  params->dsi.vertical_active_line				= FRAME_HEIGHT; 

  params->dsi.horizontal_sync_active				= 30; //40;
  params->dsi.horizontal_backporch				= 30; //100;60;
  params->dsi.horizontal_frontporch				= 40; //70;60;
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


  params->dsi.PLL_CLOCK = 230; //244; //245; //258;//244;260;258;256;262;264;262;256;
  params->dsi.ssc_disable= 1;
 	params->dsi.esd_check_enable = 1;
  	params->dsi.customization_esd_check_enable = 0;
  	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
  	params->dsi.lcm_esd_check_table[0].count        = 1;
  	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;
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
  MDELAY(10);
  lcm_set_rst_lk(0);
  MDELAY(10);
  lcm_set_rst_lk(1);
  MDELAY(80);
#else
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(80);
#endif

  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);


}

static void lcm_suspend(void)
{

#ifdef BUILD_LK
  lcm_set_rst_lk(0);
  MDELAY(120);
  lcm_set_enp_bias_lk(0);
 // MDELAY(20);
#else
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
  MDELAY(120);
  set_gpio_lcd_enp(0);
  //MDELAY(20);
#endif
  //push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
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
  printf("jd9365 adc_vol is %d\n",adc_vol);
#else
  printk("jd9365 adc_vol is %d\n",adc_vol);
#endif
 return (adc_vol >50) ? 0: 1;
}

static void lcm_resume(void)
{
  lcm_init();
}

static unsigned int lcm_compare_id(void)
{
  unsigned int id=0;
  unsigned char buffer[1];
  unsigned int array[16];
  unsigned char id_high=0;
  unsigned char id_midd=0;
  unsigned char id_low=0;



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
  array[0] = 0x00013700;// read id return two byte,version and id
  dsi_set_cmdq(array, 1, 1);
  MDELAY(10);

  read_reg_v2(0xda, buffer, 1);
  id_high = buffer[0];
  read_reg_v2(0xdb, buffer, 1);
  id_midd = buffer[1];
  read_reg_v2(0xdc, buffer, 1);
  id_low = buffer[2];
  id = id_high+adc_read_vol();

#if defined(BUILD_LK)
  printf("%s,jd9365 id_high = 0x%08x,id = 0x%08x\n", __func__, id_high,id);
#else
  printk("%s,jd9365  id_high= 0x%08x,id = 0x%08x\n", __func__, id_high,id);
#endif

  return (LCM_ID == id || 0xce == id)?1:0;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER jd9365d_hsd585_hlt_llhd_lcm_drv = 
{
  .name          = "jd9365d_hsd585_hlt_llhd",
  .set_util_funcs 	= lcm_set_util_funcs,
  .get_params     	= lcm_get_params,
  .init           	= lcm_init,
  .suspend        	= lcm_suspend,
  .resume         	= lcm_resume,
  .compare_id     	= lcm_compare_id,
};
