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
#define REGFLAG_END_OF_TABLE      							0xFA    //0xFFF ??   // END OF REGISTERS MARKER

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
{0x11, 1, {0x00}},
{REGFLAG_DELAY, 120, {}}, 
{0xff, 5,{0x77,0x01,0x00,0x00,0x10}},
{0xc0, 2,{0xf7,0x01}},
{0xc1, 2,{0x09,0x02}},
{0xc6, 1,{0x21}},
{0xc2, 2,{0x07,0x03}},
{0xcc, 1,{0x30}},
{0xb0, 16,{0x00,0x09,0x13,0x0c,0x10,0x04,0x01,0x05,0x05,0x1b,0x03,0x12,0x10,0x2b,0x33,0x19}},
{0xb1, 16,{0x00,0x0a,0x0f,0x0d,0x12,0x08,0x00,0x0a,0x09,0x1b,0x05,0x12,0x11,0x2a,0x32,0x19}},
{0xff, 5,{0x77,0x01,0x00,0x00,0x11}},
{0xb0, 1,{0x68}},
{0xb1, 1,{0x27}},
{0xb2, 1,{0x88}},
{0xb3, 1,{0x80}},
{0xb5, 1,{0x49}},
{0xb7, 1,{0x85}},
{0xb8, 1,{0x10}},
{0xb9, 2,{0x00,0x13}},
{0xc1, 1,{0x78}},
{0xc2, 1,{0x78}},
{0xd0, 1,{0x88}},
{0xff, 5,{0x77,0x01,0x00,0x00,0x11}},
{0xe0, 3,{0x00,0x00,0x02}},
{0xe1, 11,{0x02,0x91,0x00,0x00,0x01,0x91,0x00,0x00,0x00,0x60,0x60}},
{0xe2, 13,{0x30,0x30,0x40,0x40,0xc9,0x91,0x00,0x00,0xc8,0x91,0x00,0x00,0x00}},
{0xe3, 4,{0x00,0x00,0x33,0x33}},
{0xe4, 2,{0x44,0x44}},
{0xe5, 16,{0x06,0xca,0x4b,0x91,0x08,0xcc,0x4b,0x91,0x0a,0xce,0x4b,0x91,0x0c,0xd0,0x4b,0x91}},
{0xe6, 4,{0x00,0x00,0x33,0x33}},
{0xe7, 2,{0x44,0x44}},
{0xe8, 16,{0x05,0xc9,0x4b,0x91,0x07,0xcb,0x4b,0x91,0x09,0xcd,0x4b,0x91,0x0b,0xcf,0x4b,0x91}},
{0xeb, 7,{0x02,0x01,0xe4,0xe4,0x88,0x00,0x00}},
{0xec, 2,{0x00,0x00}},
{0xed, 16,{0xfa,0xb0,0x2f,0xf4,0x65,0x7f,0xff,0xff,0xff,0xff,0xf7,0x56,0x4f,0xf2,0x0b,0xaf}},
{0xff, 5,{0x77,0x01,0x00,0x00,0x00}},
{0x29,1, {0x00}},       // Display On
{REGFLAG_DELAY, 20, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}

};
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

  params->dsi.horizontal_sync_active				= 20;
  params->dsi.horizontal_backporch				= 80;//46;
  params->dsi.horizontal_frontporch				= 80;//46;
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


  params->dsi.PLL_CLOCK = 212; //dpi clock customization: should config 
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
 #ifdef BUILD_LK

  lcm_set_rst_lk(0);
#else
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
#endif
  push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
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
  printf("st7701s_cpt55_chaoyue_lfwvga adc_vol is %d\n",adc_vol);
#else
  printk("st7701s_cpt55_chaoyue_lfwvga adc_vol is %d\n",adc_vol);
#endif
 return (adc_vol < 65) ? 0: 1;
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
	device_id = ((buffer[0]<<8) | (buffer[1]+adc_read_vol()));

#if defined(BUILD_LK)
	printf("st7701s_cpt55_chaoyue_lfwvga  %s,line=%d, id = 0x%x, buffer[0]=0x%08x,buffer[1]=0x%08x\n",__func__,__LINE__, device_id, buffer[0],buffer[1]);
#else
	printk("st7701s_cpt55_chaoyue_lfwvga  %s,line=%d, id = 0x%x, buffer[0]=0x%08x,buffer[1]=0x%08x\n",__func__,__LINE__, device_id, buffer[0],buffer[1]);
#endif
  return (LCM_ID == device_id)?1:0;


}
static void lcm_resume(void)
{
  lcm_init();
}

LCM_DRIVER st7701s_cpt55_chaoyue_lfwvga_lcm_drv =
{
  .name			= "st7701s_cpt55_chaoyue_lfwvga",
  .set_util_funcs = lcm_set_util_funcs,
  .get_params     = lcm_get_params,
  .init           = lcm_init,
  .suspend        = lcm_suspend,
  .resume         = lcm_resume,
  .compare_id     = lcm_compare_id,
};
