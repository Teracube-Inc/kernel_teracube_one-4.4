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
#define FRAME_HEIGHT 										(1512)

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

static unsigned int lcm_compare_id(void);
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
  {0xFF,3,{0x98,0x81,0x01}},

  {0x01,1,{0x00}},
  {0x02,1,{0x00}},
  {0x03,1,{0x53}},
  {0x04,1,{0x13}},
  {0x05,1,{0x00}},
  {0x06,1,{0x04}},
  {0x07,1,{0x00}},
  {0x08,1,{0x00}},
  {0x09,1,{0x20}},   //00
  {0x0a,1,{0x20}},   //00
  {0x0b,1,{0x20}},    //00
  {0x0c,1,{0x00}},   //21
  {0x0d,1,{0x00}},   //21
  {0x0e,1,{0x00}},    //21
  {0x0f,1,{0x20}},   //00
  {0x10,1,{0x20}},   //00
  {0x11,1,{0x00}},
  {0x12,1,{0x00}},
  {0x13,1,{0x00}},   //21
  {0x14,1,{0x00}},
  {0x15,1,{0x00}},
  {0x16,1,{0x00}},
  {0x17,1,{0x00}},
  {0x18,1,{0x00}},
  {0x19,1,{0x00}},
  {0x1a,1,{0x00}},
  {0x1b,1,{0x00}},
  {0x1c,1,{0x00}},
  {0x1d,1,{0x00}},
  {0x1e,1,{0x44}},
  {0x1f,1,{0x00}},
  {0x20,1,{0x02}},
  {0x21,1,{0x03}},
  {0x22,1,{0x00}},
  {0x23,1,{0x00}},
  {0x24,1,{0x00}},
  {0x25,1,{0x00}},
  {0x26,1,{0x00}},
  {0x27,1,{0x00}},
  {0x28,1,{0x33}},
  {0x29,1,{0x03}},
  {0x2a,1,{0x00}},
  {0x2b,1,{0x00}},
  {0x2c,1,{0x00}},
  {0x2d,1,{0x00}},
  {0x2e,1,{0x00}},
  {0x2f,1,{0x00}},
  {0x30,1,{0x00}},
  {0x31,1,{0x00}},
  {0x32,1,{0x00}},
  {0x33,1,{0x00}},
  {0x34,1,{0x04}},
  {0x35,1,{0x00}},
  {0x36,1,{0x00}},
  {0x37,1,{0x00}},
  {0x38,1,{0x3C}},   //78
  {0x39,1,{0x07}},
  {0x3a,1,{0x00}},
  {0x3b,1,{0x00}},
  {0x3c,1,{0x00}},
  {0x3d,1,{0x00}},
  {0x3e,1,{0x00}},
  {0x3f,1,{0x00}},
  {0x40,1,{0x00}},
  {0x41,1,{0x20}},
  {0x42,1,{0x00}},
  {0x43,1,{0x00}},
  {0x44,1,{0x0f}},
  {0x45,1,{0x00}},
  {0x46,1,{0x00}},
  {0x47,1,{0x08}},
  {0x48,1,{0x00}},
  {0x49,1,{0x00}},
  {0x4a,1,{0x00}},
  {0x4b,1,{0x00}},

  // ==== GOUT_BW_L[3:0] ====
  {0x4c,1,{0x01}},
  {0x4d,1,{0x54}},
  {0x4e,1,{0x64}},
  {0x4f,1,{0xa8}},
  {0x50,1,{0x2a}},
  {0x51,1,{0x22}},
  {0x52,1,{0x22}},
  {0x53,1,{0x62}},
  {0x54,1,{0x22}},
  {0x55,1,{0x22}},
  {0x56,1,{0x22}},

  // ==== GOUT_BW_R[3:0] ====
  {0x57,1,{0x01}},
  {0x58,1,{0x54}},
  {0x59,1,{0x75}},
  {0x5a,1,{0xb9}},
  {0x5b,1,{0x2b}},
  {0x5c,1,{0x22}},
  {0x5d,1,{0x22}},
  {0x5e,1,{0x72}},
  {0x5f,1,{0x22}},
  {0x60,1,{0x22}},
  {0x61,1,{0x22}},

  {0x62,1,{0x06}},

  // ==== GOUT_BW_L ====
  {0x63,1,{0x01}},
  {0x64,1,{0x00}},
  {0x65,1,{0xa4}},
  {0x66,1,{0xa5}},
  {0x67,1,{0x54}},
  {0x68,1,{0x56}},
  {0x69,1,{0x58}},
  {0x6a,1,{0x5a}},
  {0x6b,1,{0x06}},
  {0x6c,1,{0x02}},
  {0x6d,1,{0x02}},
  {0x6e,1,{0x02}},
  {0x6f,1,{0x02}},
  {0x70,1,{0x02}},
  {0x71,1,{0x02}},
  {0x72,1,{0x0a}},
  {0x73,1,{0x02}},
  {0x74,1,{0x02}},
  {0x75,1,{0x02}},
  {0x76,1,{0x02}},
  {0x77,1,{0x02}},
  {0x78,1,{0x02}},

  // ==== GOUT_BW_R ====
  {0x79,1,{0x01}},
  {0x7a,1,{0x00}},
  {0x7b,1,{0xa4}},
  {0x7c,1,{0xa5}},
  {0x7d,1,{0x55}},
  {0x7e,1,{0x57}},
  {0x7f,1,{0x59}},
  {0x80,1,{0x5b}},
  {0x81,1,{0x07}},
  {0x82,1,{0x02}},
  {0x83,1,{0x02}},
  {0x84,1,{0x02}},
  {0x85,1,{0x02}},
  {0x86,1,{0x02}},
  {0x87,1,{0x02}},
  {0x88,1,{0x0b}},
  {0x89,1,{0x02}},
  {0x8a,1,{0x02}},
  {0x8b,1,{0x02}},
  {0x8c,1,{0x02}},
  {0x8d,1,{0x02}},
  {0x8e,1,{0x02}},
  {0x8f,1,{0x00}},
  {0x90,1,{0x00}},


{0xFF,03,{0x98,0x81,0x02}},
{0x42,01,{0x30}},    //20
{0x08,01,{0x22}},
//{0x14,01,{0x41}},    //bb
//{0x10,01,{0xcc}},    //bb
  //{0x33,1,{0x51}},   //BIST
{0x15,01,{0x10}},    //2-powermode
{0x57,01,{0x00}},	  //Gamma
{0x58,01,{0x1E}},	
{0x59,01,{0x2C}},	
{0x5A,01,{0x15}},	
{0x5B,01,{0x19}},	
{0x5C,01,{0x2B}},	
{0x5D,01,{0x1F}},	
{0x5E,01,{0x20}},	
{0x5F,01,{0x96}},	
{0x60,01,{0x19}},	
{0x61,01,{0x25}},	
{0x62,01,{0x80}},	
{0x63,01,{0x1C}},	
{0x64,01,{0x1B}},	
{0x65,01,{0x50}},	
{0x66,01,{0x24}},	
{0x67,01,{0x28}},	
{0x68,01,{0x4E}},	
{0x69,01,{0x5A}},	
{0x6A,01,{0x30}},	

  {0x6B,1,{0x00}},	
  {0x6C,1,{0x1E}},	
  {0x6D,1,{0x2C}},	
  {0x6E,1,{0x15}},	
  {0x6F,1,{0x19}},	
  {0x70,1,{0x2B}},	
  {0x71,1,{0x1F}},	
  {0x72,1,{0x20}},	
  {0x73,1,{0x96}},	
  {0x74,1,{0x19}},	
  {0x75,1,{0x25}},	
  {0x76,1,{0x80}},	
  {0x77,1,{0x1C}},	
  {0x78,1,{0x1B}},	
  {0x79,1,{0x50}},	
  {0x7A,1,{0x24}},	
  {0x7B,1,{0x28}},	
  {0x7C,1,{0x4E}},	
  {0x7D,1,{0x5A}},	
  {0x7E,1,{0x30}},	



{0xFF,03,{0x98,0x81,0x05}},
{0x30,01,{0x77}},         //VGH&VGLSeting
{0x54,01,{0x39}},          //VGH=15V
{0x55,01,{0x1A}},         //VGL=-10V
{0x56,01,{0x7b}},          //VREG1=4.8V
{0x57,01,{0x7b}},          //VREG2=-4.8V
{0x04,01,{0x6d}},          //VCOM   0X6b
{0x06,01,{0x6D}},           //VCOM

//{0xFF,03,{0x98,0x81,0x03}},
//{0x92,01,{0x77}},
{0xFF,03,{0x98,0x81,0x06}},
{0xc0,01,{0xf3}},           //
{0xc1,01,{0x2a}},   //0a           //
//{0x30,01,{0xb0}},  
//{0x33,01,{0x09}},
  {0xFF,3,{0x98,0x81,0x00}},
  {0x36,1,{0x0a}},  //08 07 02ok 00 03 01 05 13 
  {0x11,1,{0x00}},        // Sleep-Out
  {REGFLAG_DELAY, 120, {}},
  {0x29,1, {0x00}},       // Display On
  {REGFLAG_DELAY, 20, {}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_sleep_mode_in_setting[] =
{
  // Display off sequence
  {0x28, 0, {0x00}},
  {REGFLAG_DELAY, 20, {}},

  // Sleep Mode On
  {0x10, 0, {0x00}},
  {REGFLAG_DELAY, 120, {}},
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
    params->dbi.te_mode             = LCM_DBI_TE_MODE_DISABLED;
  params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

  //params->dsi.mode   = SYNC_PULSE_VDO_MODE;
    params->dsi.mode   = SYNC_EVENT_VDO_MODE;

  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM				= LCM_FOUR_LANE;

  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;

    params->dsi.packet_size=256;
    params->dsi.intermediat_buffer_num = 2;
  params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.word_count=FRAME_WIDTH*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_sync_active				= 6;
    params->dsi.vertical_backporch				= 15; //12
    params->dsi.vertical_frontporch				= 16; //6	
  params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active				= 10; //10
    params->dsi.horizontal_backporch				= 80; //70
  params->dsi.horizontal_frontporch				= 90; //70;
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


params->dsi.PLL_CLOCK = 258 ;//260;//212;//208;	
params->dsi.ssc_disable = 1;	
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
  MDELAY(120);
#else
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(120);
#endif

  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);


}

static void lcm_suspend(void)
{

  push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
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
  data_array[0] = 0x00043902; 
  data_array[1] = 0x068198FF; 
  dsi_set_cmdq(data_array, 2, 1); 
  array[0] = 0x00013700; 
  dsi_set_cmdq(array, 1, 1); 
  MDELAY(2); 

  read_reg_v2(0xF2, read_buf, 1);
  id = read_buf[0];

#if defined(BUILD_LK)
  printf("%s,[LK] ili9881a_hsd585_ykl_llhd_lcm_drv id = 0x%x\n", __func__, id);
#else
  printk("%s, ili9881a_hsd585_ykl_llhd_lcm_drv id1 = 0x%x\n", __func__, id);
#endif


  return (0x10 == id)?1:0;

}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9881a_hsd585_ykl_llhd_lcm_drv = 
{
  .name          = "ili9881a_hsd585_ykl_llhd",
  .set_util_funcs 	= lcm_set_util_funcs,
  .get_params     	= lcm_get_params,
  .init           	= lcm_init,
  .suspend        	= lcm_suspend,
  .resume         	= lcm_resume,
  .compare_id     	= lcm_compare_id,
};
