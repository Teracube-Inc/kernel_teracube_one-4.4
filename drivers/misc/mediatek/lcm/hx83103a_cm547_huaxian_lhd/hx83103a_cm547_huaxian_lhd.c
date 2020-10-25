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

#define LCM_ID_NT35521 (0xf5)

static const unsigned int BL_MIN_LEVEL = 20;
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
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>


#define I2C_I2C_LCD_BIAS_CHANNEL 0
#ifndef CONFIG_FPGA_EARLY_PORTING
#define I2C_I2C_LCD_BIAS_CHANNEL 0
#define TPS_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL	/* for I2C channel 0 */
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E

#if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info tps65132_board_info __initdata = { I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR) };
#endif
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
  {.compatible = "mediatek,I2C_LCD_BIAS"},
  {},
};
#endif

/*static struct i2c_client *tps65132_i2c_client;*/
struct i2c_client *tps65132_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct tps65132_dev {
  struct i2c_client *client;

};

static const struct i2c_device_id tps65132_id[] = {
  {I2C_ID_NAME, 0},
  {}
};

/* #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)) */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */
/* #endif */
static struct i2c_driver tps65132_iic_driver = {
  .id_table = tps65132_id,
  .probe = tps65132_probe,
  .remove = tps65132_remove,
  /* .detect               = mt6605_detect, */
  .driver = {
    .owner = THIS_MODULE,
    .name = "tps65132",
#if !defined(CONFIG_MTK_LEGACY)
    .of_match_table = lcm_of_match,
#endif
  },
};

static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  printk("tps65132_iic_probe\n");
  printk("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
  tps65132_i2c_client = client;
  return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
  printk("tps65132_remove\n");
  tps65132_i2c_client = NULL;
  i2c_unregister_device(client);
  return 0;
}

/*static int tps65132_write_bytes(unsigned char addr, unsigned char value)*/
#if !defined(CONFIG_ARCH_MT6797)
int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
  int ret = 0;
  struct i2c_client *client = tps65132_i2c_client;
  char write_data[2] = { 0 };

  write_data[0] = addr;
  write_data[1] = value;
  ret = i2c_master_send(client, write_data, 2);
  if (ret < 0)
    printk("tps65132 write data fail !!\n");
  return ret;
}
#endif

static int __init tps65132_iic_init(void)
{
  printk("tps65132_iic_init\n");
#if defined(CONFIG_MTK_LEGACY)
  i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
#endif
  printk("tps65132_iic_init2\n");
  i2c_add_driver(&tps65132_iic_driver);
  printk("tps65132_iic_init success\n");
  return 0;
}

static void __exit tps65132_iic_exit(void)
{
  printk("tps65132_iic_exit\n");
  i2c_del_driver(&tps65132_iic_driver);
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Mike Liu");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL");
#endif
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
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

  params->dsi.vertical_sync_active				= 4; //8;
  params->dsi.vertical_backporch					= 6;  //16;
  params->dsi.vertical_frontporch					= 20;  //50; //200; //16;
  params->dsi.vertical_active_line				= FRAME_HEIGHT; 

  params->dsi.horizontal_sync_active				= 22; //40;
  params->dsi.horizontal_backporch				= 22; //100;
  params->dsi.horizontal_frontporch				= 32; //70;
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


  params->dsi.PLL_CLOCK = 212;
}

#ifdef BUILD_LK
#ifndef CONFIG_FPGA_EARLY_PORTING
#define TPS65132_SLAVE_ADDR_WRITE  0x7C
static struct mt_i2c_t TPS65132_i2c;

static int TPS65132_write_byte(kal_uint8 addr, kal_uint8 value)
{
  kal_uint32 ret_code = I2C_OK;
  kal_uint8 write_data[2];
  kal_uint16 len;

  write_data[0] = addr;
  write_data[1] = value;

  TPS65132_i2c.id = 0; //I2C_I2C_LCD_BIAS_CHANNEL;	/* I2C2; */
  /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
  TPS65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
  TPS65132_i2c.mode = ST_MODE;
  TPS65132_i2c.speed = 100;
  len = 2;

  ret_code = i2c_write(&TPS65132_i2c, write_data, len);
  /* printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code); */

  return ret_code;
}

#else

/* extern int mt8193_i2c_write(u16 addr, u32 data); */
/* extern int mt8193_i2c_read(u16 addr, u32 *data); */

/* #define TPS65132_write_byte(add, data)  mt8193_i2c_write(add, data) */
/* #define TPS65132_read_byte(add)  mt8193_i2c_read(add) */

#endif
#endif
static void lcm_init(void)
{
  unsigned int data_array[16];
  unsigned char cmd = 0x0;
  unsigned char data = 0x12;
#ifndef CONFIG_FPGA_EARLY_PORTING
  int ret = 0;
#endif
#ifdef BUILD_LK
  lcm_set_enp_bias_lk(1);
#else
  set_gpio_lcd_enp(1);
#endif
  cmd = 0x00;
  data = 0x12;

#ifdef BUILD_LK
  lcm_set_rst_lk(0);
#else
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef CONFIG_MTK_LEGACY
  //	mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
  //	mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
  //	mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ONE);
  //#else
  //	set_gpio_lcd_enp(1);
#endif
  MDELAY(5);
#ifdef BUILD_LK
  ret = TPS65132_write_byte(cmd, data);
#else
#if !defined(CONFIG_ARCH_MT6797)
  ret = tps65132_write_bytes(cmd, data);
#endif
#endif

  if (ret < 0)
    printk("nt35521----tps6132----cmd=%0x--i2c write error----\n", cmd);
  else
    printk("nt35521----tps6132----cmd=%0x--i2c write success----\n", cmd);

  cmd = 0x01;
  data = 0x12;

#ifdef BUILD_LK
  ret = TPS65132_write_byte(cmd, data);
#else
#if !defined(CONFIG_ARCH_MT6797)
  ret = tps65132_write_bytes(cmd, data);
#endif
#endif

  if (ret < 0)
    printk("nt35521----tps6132----cmd=%0x--i2c write error----\n", cmd);
  else
    printk("nt35521----tps6132----cmd=%0x--i2c write success----\n", cmd);

#endif
#ifdef BUILD_LK
  mt_set_gpio_mode(GPIO_TP_RST_1, GPIO_LCM_RST_M_GPIO);
  mt_set_gpio_pull_enable(GPIO_TP_RST_1, GPIO_PULL_ENABLE);
  mt_set_gpio_dir(GPIO_TP_RST_1, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_TP_RST_1, GPIO_OUT_ONE);
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
  unsigned int id = 0xFF;
  unsigned int id1 = 0xFF;
  unsigned int id2 = 0xFF;
   unsigned char read_buf[16];

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
  read_reg_v2(0x04, read_buf, 16);
  id = read_buf[0];//83
  id1 = read_buf[1];//11
  id2 = read_buf[2];//0a

#if defined(BUILD_LK)
  printf("%s,[LK] ili9881_hd720_dsi_vdo_gpo id = 0x%x--0x%x---0x%x\n", __func__, id,id1,id2);
#else
  printk("%s, ili9881_hd720_dsi_vdo_gpo id1 = 0x%x--0x%x---0x%x\n", __func__, id,id1,id2);

#endif

  if ((0x83==id)&&(0x11==id1)&&(0x0a==id2))	
    return 1;
  else
    return 0;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hx83103a_cm547_huaxian_lhd_lcm_drv = 
{
  .name          = "hx83103a_cm547_huaxian_lhd",
  .set_util_funcs 	= lcm_set_util_funcs,
  .get_params     	= lcm_get_params,
  .init           	= lcm_init,
  .suspend        	= lcm_suspend,
  .resume         	= lcm_resume,
  .compare_id     	= lcm_compare_id,
};
