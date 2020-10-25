/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
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
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

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
#endif



// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (960)
#define LCM_ID 0x8802
#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#ifndef FALSE
  #define FALSE (0)
#endif

#ifndef TRUE
  #define TRUE  (1)
#endif

//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

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
#define GPIO_LCM_RST_1         (GPIO83 | 0x80000000)
#define GPIO_LCM_RST           (83+343)
#ifdef BUILD_LK

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
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x10}},
	{0xC6, 1,{0x07}},
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x00}},
	{0x11,  0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x10}},
	{0xC0, 2,{0x77, 0x00}},
	{0xC1, 2,{0x12, 0x02}},
	{0xC2, 2,{0x07, 0x02}},
	{0xCC, 1,{0x10}},
#if 0
	{0xB0, 16, {0x00,0x10,0x1B,0x0F,0x14,0x08,0x0D,0x08,0x08,0x25,0x06,0x15,0x13,0xE6,0x2C,0x11} },
	{0xB1, 16, {0x00,0x10,0x1B,0x0F,0x14,0x08,0x0E,0x08,0x08,0x25,0x04,0x11,0x0F,0x27,0x2C,0x11} },
#else	//gamma 2.5
	{0xB0, 16, {0x00,0x0E,0x1C,0x0F,0x12,0x07,0x0C,0x08,0x07,0x25,0x06,0x15,0x13,0x24,0x28,0x08} },
	{0xB1, 16, {0x00,0x0E,0x16,0x0F,0x14,0x09,0x0E,0x08,0x08,0x26,0x07,0x17,0x14,0x23,0x28,0x0B} },
#endif
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x11} },
	{0xB0,  1, {0x5d} },
	{0xB1,  1, {0x12} }, //12
	{0xB2, 1,{0x87}},
	{0xB3, 1,{0x80}},
	{0xB5, 1,{0x49}},
	{0xB7, 1,{0x89}},
	{0xB8, 1,{0x21}},
	{0xC1, 1,{0x78}},
	{0xC2, 1,{0x78}},
	{0xD0, 1,{0x88}},
	{REGFLAG_DELAY, 10, {}},
	{0xE0, 3,{0x00, 0x00, 0x02}},
	{0xE1, 11,{0x0A,0x96,0x0C,0x96,0x0B,0x96,0x0D,0x96,0x00,0x44,0x44}},
	{0xE2, 13,{0x33,0x33,0x44,0x44,0xCF,0x96,0xD1,0x96,0xD0,0x96,0xD2,0x96,0x00}},
	{0xE3, 4,{0x00, 0x00, 0x33, 0x33}},
	{0xE4, 2,{0x44, 0x44}},
	{0xE5, 16,{0x0C,0xD0,0x86,0x86,0x0E,0xD2,0x86,0x86,0x10,0xD4,0x86,0x86,0x12,0xD6,0x86,0x86}},
	{0xE6, 4,{0x00, 0x00, 0x33, 0x33}},
	{0xE7, 2,{0x44, 0x44}},
	{0xE8, 16,{0x0D,0xD1,0x86,0x86,0x0F,0xD3,0x86,0x86,0x11,0xD5,0x86,0x86,0x13,0xD7,0x86,0x86}},
	{0xEB, 7,{0x02,0x01,0x4E,0x4E,0xEE,0x44,0x00}},
	{0xEC, 2,{0x00, 0x00}},
	{0xED, 16,{0xFF,0xF1,0x04,0x56,0x72,0x3F,0xFF,0xFF,0xFF,0xFF,0xF3,0x27,0x65,0x40,0x1F,0xFF}},

	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x00}},
	{0x29,  0, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	#if 1 
	//{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x10}},
	//{0xC2, 2,{0x00, 0x02}}, //1 Dot
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x12} },
	{0xd1, 1,{0x81}},
	{0xd2, 1,{0x0c}},
    //{0xd1, 14,{0x81, 0x15, 0x03, 0xc0, 0x0c, 0x01, 0x30, 0x01,0xe0, 0x80,0x01, 0xe0, 0x03, 0xc0} },
	//{0xd2, 10,{0x0c, 0x00, 0x00, 0xa0, 0x01, 0x40, 0x01, 0x40, 0x02, 0x80} }, //Bist Blanking
	{REGFLAG_DELAY, 120, {}},
		{REGFLAG_DELAY, 120, {}},
			{REGFLAG_DELAY, 120, {}},
				{REGFLAG_DELAY, 120, {}},

	{0xd1,  1, {0x01} }, //Return
	
	//{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x10}},
	//{0xC2, 2,{0x07, 0x02}}, //Column
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x00}},
	#endif 
	{REGFLAG_END_OF_TABLE, 0x00, {} }

};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
#if 0
    {0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x10}},
	{0xC2, 2,{0x00, 0x02}}, //1 Dot
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x12} },
	{0xd1, 1,{0x81}},
	{0xd2, 1,{0x0c}},//É¨ºÚ
    
	{REGFLAG_DELAY, 120, {}},

	{0xd1,  1, {0x01} }, //Return
#endif
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x10} },
    {0x28, 0, {}},
	{REGFLAG_DELAY, 20, {}},

	{0xC2, 2,{0x07, 0x06}}, //1 Dot
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x11} },
	{0xC0,  1, {0x07} },
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x13} },
	{0xeb,  1, {0x12} },
	{REGFLAG_DELAY, 20, {}},
	{0xe8,  1, {0x01} },
	{0xeb,  1, {0x1e} },
	{REGFLAG_DELAY, 80, {}},
    {0x10, 0, {}},
	{REGFLAG_DELAY, 40, {}},
	{0xeb,  1, {0x00} },
	{0xe8,  1, {0x00} },






#if 0


	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x10}},
	{0xC2, 2,{0x07, 0x06}}, //1 Dot
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x00} },
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x11} },
	{0xC0,  1, {0x07} },
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x00}},
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x13} },

	{0xeb,  1, {0x12} },
	{REGFLAG_DELAY, 20, {}},
	{0xe8,  1, {0x01} },
	{0xeb,  1, {0x1e} },
	{REGFLAG_DELAY, 120, {}},
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x00}},
    {0x10, 0, {}},
	{REGFLAG_DELAY, 120, {}},
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x13} },
	{0xeb,  1, {0x00} },
	{0xe8,  1, {0x00} },
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
#endif
};
static struct LCM_setting_table lcm_deep_sleep_mode_out_setting[] = {
#if 0
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x11} },
	{0xC0,  1, {0x87} },
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x00}},
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x13} },
	{0xeb,  1, {0x00} },
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x00}},
	#endif
	
    {0x11, 0, {}},
	{REGFLAG_DELAY, 120, {}},
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x10}},
	{0xC2, 2,{0x07, 0x02}}, //1 Dot
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x00} },
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x11} },
	{0xC0,  1, {0x87} },
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x00}},
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x13} },
	{0xeb,  1, {0x00} },
	{REGFLAG_DELAY, 120, {}},

	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x00}},
    {0x29, 0, {}},
	{REGFLAG_DELAY, 20, {}},
	#if 0
	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x10}},
	{0xC2, 2,{0x00, 0x02}}, //1 Dot
	{0xFF,  5, {0x77, 0x01, 0x00, 0x00, 0x12} },
	{0xd1, 1,{0x81}},
	{0xd2, 1,{0x0c}},
   // {0xd1,  14,{0x81, 0x15, 0x03, 0xc0, 0x0c, 0x01, 0x30, 0x01,0xe0, 0x80,0x01, 0xe0, 0x03, 0xc0} },
	//{0xd2,  10, {0x05, 0x80, 0x00, 0xa0, 0x01, 0x40, 0x01, 0x40, 0x02, 0x80} }, //Bist Blanking
	{REGFLAG_DELAY, 120, {}},

	{0xd1,  1, {0x01} }, //Return

	{0xFF, 5,{0x77, 0x01, 0x00, 0x00, 0x10}},
	{0xC2, 2,{0x07, 0x02}}, //Column
	#endif
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

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

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

  // enable tearing-free
  params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
  params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

    //params->dsi.mode   = SYNC_PULSE_VDO_MODE;
  params->dsi.mode = SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;

  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM				= LCM_TWO_LANE;
  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

  params->dsi.packet_size=256;
  params->dsi.intermediat_buffer_num = 2;

  params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

  params->dsi.vertical_sync_active				= 8;
  params->dsi.vertical_backporch					= 20; //18
  params->dsi.vertical_frontporch					= 20;
  params->dsi.vertical_active_line				= FRAME_HEIGHT; 

  params->dsi.horizontal_sync_active				= 6;
  params->dsi.horizontal_backporch				= 40;//46;
  params->dsi.horizontal_frontporch				= 40;//46;
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


  params->dsi.PLL_CLOCK = 212; //dpi clock customization: should config 
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
  printf("st7701s adc_vol is %d\n",adc_vol);
#else
  printk("st7701s adc_vol is %d\n",adc_vol);
#endif
 return (adc_vol > 65) ? 0: 1;
}
static void lcm_init(void)
{
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
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
  push_table(lcm_deep_sleep_mode_out_setting, sizeof(lcm_deep_sleep_mode_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void)
{

 	unsigned int array[4];
	unsigned short device_id;
	unsigned char buffer[4];

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

  array[0]=0x00063902;
    array[1]=0x000177ff;
    array[2]=0x00001000;
    dsi_set_cmdq(array, 3, 1);
	MDELAY(10); 

	read_reg_v2(0xa1, buffer, 2);	
	device_id = ((buffer[0]<<8) | (buffer[1]) | adc_read_vol());

#if defined(BUILD_LK)
	printf("st7701_ivo50_hongzhan_fwvga  %s,line=%d, id = 0x%x, buffer[0]=0x%08x,buffer[1]=0x%08x\n",__func__,__LINE__, device_id, buffer[0],buffer[1]);
#else
	printk("st7701_ivo50_hongzhan_fwvga  %s,line=%d, id = 0x%x, buffer[0]=0x%08x,buffer[1]=0x%08x\n",__func__,__LINE__, device_id, buffer[0],buffer[1]);
#endif
  return (LCM_ID == device_id)?1:0;


}

LCM_DRIVER st7701s_ivo55_chaoyue_lfwvga_lcm_drv =
{
  .name			= "st7701s_ivo55_chaoyue_lfwvga",
  .set_util_funcs = lcm_set_util_funcs,
  .get_params     = lcm_get_params,
  .init           = lcm_init,
  .suspend        = lcm_suspend,
  .resume         = lcm_resume,
  .compare_id     = lcm_compare_id,
};
