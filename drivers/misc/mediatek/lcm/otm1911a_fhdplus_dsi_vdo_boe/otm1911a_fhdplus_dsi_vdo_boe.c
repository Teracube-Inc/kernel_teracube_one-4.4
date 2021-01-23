/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 *****************************************************************************/

#if defined(BUILD_LK)
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#endif

#if !defined(BUILD_LK)
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#if defined(BUILD_LK)
#else
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH     (1080)
#define FRAME_HEIGHT    (2160)
/* physical size in um */
#define LCM_PHYSICAL_WIDTH									(67392)
#define LCM_PHYSICAL_HEIGHT									(134784)
#define LCM_DENSITY											(480)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY             							0xFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table
{
  unsigned cmd;
  unsigned char count;
  unsigned char para_list[64];
};

static void lcm_init_power(void)
{
  display_bias_enable();
  MDELAY(20);
}

static void lcm_suspend_power(void)
{
  display_bias_disable();
}

static void lcm_resume_power(void)
{
  SET_RESET_PIN(0);
  display_bias_enable();
  MDELAY(20);
}

static struct LCM_setting_table lcm_initialization_setting[] = {

  {0x00, 1,{0x00}},
  {0xFF, 3,{0x19,0x11,0x01}},
  {0x00, 1,{0x80}},
  {0xFF, 2,{0x19,0x11}},
  {0x00, 1,{0x93}},
  {0xB3, 1,{0x06}},
  {0x00, 1,{0xB0}},
  {0xB3, 4,{0x04,0x38,0x08,0x70}},
  {0x00, 1,{0x80}},
  {0xC5, 7,{0x55,0x05,0x11,0xA2,0xB7,0x96,0xAA}},
  {0x00, 1,{0x90}},
  {0xC5, 5,{0x88,0xA0,0x75,0x65,0x80}},
  {0x00, 1,{0xA0}},
  {0xC5, 4,{0x9E,0xB2,0x00,0x88}},

  {0x11,0,{0x00}},
  {REGFLAG_DELAY, 120, {}},
  {0x29,0,{0x00}},
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

  for (i = 0; i < count; i++)
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

  /* physical size in um */
  params->physical_width = LCM_PHYSICAL_WIDTH/1000;
  params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
  params->physical_width_um = LCM_PHYSICAL_WIDTH;
  params->physical_height_um = LCM_PHYSICAL_HEIGHT;
  params->density            = LCM_DENSITY;

  // enable tearing-free
  params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
  params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

  params->dsi.mode   = SYNC_EVENT_VDO_MODE;//SYNC_EVENT_VDO_MODE;//SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;

  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM = LCM_FOUR_LANE;
  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

  // Highly depends on LCD driver capability.
  // Not support in MT6573
  params->dsi.packet_size=256;
  params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

  params->dsi.vertical_sync_active                = 8; //2
  params->dsi.vertical_backporch                  = 32; //14
  params->dsi.vertical_frontporch                 = 32;  //16
  params->dsi.vertical_active_line                = FRAME_HEIGHT;

  params->dsi.horizontal_sync_active				= 10;//10 8
  params->dsi.horizontal_backporch				= 40;//34 32; 
  params->dsi.horizontal_frontporch				= 20;//24 43;
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

  params->dsi.cont_clock	= 1;
  params->dsi.ssc_disable = 1;
  params->dsi.PLL_CLOCK = 455;

  //params->dsi.cont_clock=1;
  params->dsi.clk_lp_per_line_enable = 0;
  params->dsi.esd_check_enable = 0;
  params->dsi.customization_esd_check_enable = 0;
  params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
  params->dsi.lcm_esd_check_table[0].count = 1;
  params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}

static void lcm_init(void)
{
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(10);
  SET_RESET_PIN(1);
  MDELAY(120);

  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
  push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
  //SET_RESET_PIN(0);
  //MDELAY(50);
}

static void lcm_resume(void)
{
  lcm_init();
  MDELAY(9);
}

#if 0
static unsigned int lcm_compare_id(void)
{
  int array[4];
  char buffer[3];
  int id=0;

  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(20);
  SET_RESET_PIN(1);
  MDELAY(120);

  array[0] = 0x00013700;
  dsi_set_cmdq(array, 1, 1);
  MDELAY(10);

  read_reg_v2(0xbf, buffer, 2);
  id = (buffer[0] << 8) + buffer[1];

#if defined(BUILD_LK)
  printf("%s,otm1911a_fhdplus_dsi_vdo_boe buffer[0] = 0x%08x, id = 0x%08x\n", __func__, buffer[0], id);
#else
  printk("%s,otm1911a_fhdplus_dsi_vdo_boe buffer[0] = 0x%08x, buffer[1] = 0x%08x, id = 0x%08x\n", __func__, buffer[0], buffer[1], id);
#endif
  return (0x023c == id)?1:0;
}
#endif

LCM_DRIVER otm1911a_fhdplus_dsi_vdo_boe_lcm_drv =
{
  .name			  = "otm1911a_fhdplus_dsi_vdo_boe",
  .set_util_funcs = lcm_set_util_funcs,
  .get_params     = lcm_get_params,
  .init           = lcm_init,
  .suspend        = lcm_suspend,
  .resume         = lcm_resume,
  //.compare_id     = lcm_compare_id,
  .init_power     = lcm_init_power,
  .resume_power   = lcm_resume_power,
  .suspend_power  = lcm_suspend_power,
};
