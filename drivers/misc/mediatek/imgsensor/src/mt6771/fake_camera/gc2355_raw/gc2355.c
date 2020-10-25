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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   gc2355yuv_Sensor.c
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *   V1.2.3
 *
 * Author:
 * -------
 *   Leo
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 * 2012.02.29  kill bugs
 *   
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

//#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "gc2355.h"
#include "fake_camera_define.h"

#define Sleep(ms) mdelay(ms)

UINT16 GC2355_read_cmos_sensor(kal_uint8 addr)
{
  UINT16 get_byte = 0;
  char puSendCmd = { (char)(addr & 0xFF) };
  kdSetI2CSpeed(100);
  if(0 != iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, GC2355_READ_ID))
  {
    YUV_DBG("ERROR: gc2355MIPI_read_cmos_sensor \n");
    return 0;
  }
  if(get_byte == 0)  //for ata yuv sensor shutter
  {
    get_byte = 1;
  }
  return get_byte;
}

static void GC2355_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
  char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
  kdSetI2CSpeed(100);
  iWriteRegI2C(puSendCmd , 2, GC2355_WRITE_ID);
  //Sleep(10);
  //GC2355_read_cmos_sensor(addr);
}

UINT16 GC2355_MIPI_GetYUVSensorBV(void)
{
  UINT8 temp_reg1, temp_reg2;
  UINT16 shutter,light;

  //GC2355_GC2355_write_cmos_sensor(0xfe,0x01);
  //light = GC2355_read_cmos_sensor(0x14);

  //GC2355_GC2355_write_cmos_sensor(0xfe,0x00);
  //temp_reg1 = GC2355_read_cmos_sensor(0x04);
  //temp_reg2 = GC2355_read_cmos_sensor(0x03);

  shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

  light = GC2355_read_cmos_sensor(0xef);
  YUV_DBG("gc2355 light = %d shutter = %d\n",light,shutter);

  return light;
}

void GC2355_Sensor_Init(void)
{
  	GC2355_write_cmos_sensor(0xfe,0x80);
	GC2355_write_cmos_sensor(0xfe,0x80);
	GC2355_write_cmos_sensor(0xfe,0x80);
	GC2355_write_cmos_sensor(0xf2,0x00); //sync_pad_io_ebi
	GC2355_write_cmos_sensor(0xf6,0x00); //up down
	GC2355_write_cmos_sensor(0xf7,0x31); //19 //pll enable
	GC2355_write_cmos_sensor(0xf8,0x06); //Pll mode 2  /////86--Ƶ
	GC2355_write_cmos_sensor(0xf9,0x0e); //de//[0] pll enable
	GC2355_write_cmos_sensor(0xfa,0x00); //div
	GC2355_write_cmos_sensor(0xfc,0x06); //4e
	//GC2355_write_cmos_sensor(0xfd,0x00);
	GC2355_write_cmos_sensor(0xfe,0x00);

	/* ANALOG & CISCTL*/
	GC2355_write_cmos_sensor(0x03,0x0b);
	GC2355_write_cmos_sensor(0x04,0xb8);
	GC2355_write_cmos_sensor(0x05,0x01); //max 30fps  03
	GC2355_write_cmos_sensor(0x06,0x22);
	GC2355_write_cmos_sensor(0x07,0x00);
	GC2355_write_cmos_sensor(0x08,0x14); //22
	GC2355_write_cmos_sensor(0x0a,0x00); //row start
	GC2355_write_cmos_sensor(0x0c,0x04); //0c//col start
	GC2355_write_cmos_sensor(0x0d,0x04);
	GC2355_write_cmos_sensor(0x0e,0xc0); //c0
	GC2355_write_cmos_sensor(0x0f,0x06);
	GC2355_write_cmos_sensor(0x10,0x50); //Window setting 1616x1216
	/*GC2355_write_cmos_sensor(0x11,0x00);
	GC2355_write_cmos_sensor(0x12,0x18); //sh_delay
	GC2355_write_cmos_sensor(0x13,0x11);
	GC2355_write_cmos_sensor(0x14,0x01);
	GC2355_write_cmos_sensor(0x15,0x00);
	GC2355_write_cmos_sensor(0x16,0xc1);*/
	GC2355_write_cmos_sensor(0x17,0x14);//14
	//GC2355_write_cmos_sensor(0x18,0x02);
	GC2355_write_cmos_sensor(0x19,0x0b);
	GC2355_write_cmos_sensor(0x1b,0x49); //48
	GC2355_write_cmos_sensor(0x1c,0x12);
	GC2355_write_cmos_sensor(0x1d,0x10); //double reset
	GC2355_write_cmos_sensor(0x1e,0xbc); //a8//col_r/rowclk_mode/rsthigh_en FPN
	GC2355_write_cmos_sensor(0x1f,0xc8); //c8//rsgl_s_mode/vpix_s_mode ƹܺ
	GC2355_write_cmos_sensor(0x20,0x71);
	GC2355_write_cmos_sensor(0x21,0x20); //rsg ƹܺ   //////40
	GC2355_write_cmos_sensor(0x22,0xa0); //e0   //80  //f0
	GC2355_write_cmos_sensor(0x23,0x51); //51
	GC2355_write_cmos_sensor(0x24,0x19); //0b //55
	//GC2355_write_cmos_sensor(0x25,0x10);  //10
	//GC2355_write_cmos_sensor(0x26,0x00);
	GC2355_write_cmos_sensor(0x27,0x20);
	GC2355_write_cmos_sensor(0x28,0x00);//10
	//GC2355_write_cmos_sensor(0x29,0x00);
	//GC2355_write_cmos_sensor(0x2a,0x00);
	GC2355_write_cmos_sensor(0x2b,0x81); //00 sf_s_mode FPN
	GC2355_write_cmos_sensor(0x2c,0x38); //50 //5c ispg FPN
	//GC2355_write_cmos_sensor(0x2d,0x15);
	GC2355_write_cmos_sensor(0x2e,0x16); //05//eq width
	GC2355_write_cmos_sensor(0x2f,0x14); //[3:0]tx_width д0ܸ
	GC2355_write_cmos_sensor(0x30,0x00);
	GC2355_write_cmos_sensor(0x31,0x01);
	GC2355_write_cmos_sensor(0x32,0x02);
	GC2355_write_cmos_sensor(0x33,0x03);
	GC2355_write_cmos_sensor(0x34,0x07);
	GC2355_write_cmos_sensor(0x35,0x0b);
	GC2355_write_cmos_sensor(0x36,0x0f);

	/* gain */
	GC2355_write_cmos_sensor(0xb0,0x50);
	GC2355_write_cmos_sensor(0xb1,0x01);
	GC2355_write_cmos_sensor(0xb2,0x00);
	GC2355_write_cmos_sensor(0xb3,0x40);
	GC2355_write_cmos_sensor(0xb4,0x40);
	GC2355_write_cmos_sensor(0xb5,0x40);
	GC2355_write_cmos_sensor(0xb6,0x00);

	/* crop */
	GC2355_write_cmos_sensor(0x92,0x02);
	GC2355_write_cmos_sensor(0x95,0x04);
	GC2355_write_cmos_sensor(0x96,0xb0);
	GC2355_write_cmos_sensor(0x97,0x06);
	GC2355_write_cmos_sensor(0x98,0x40); //out window set 1600x1200

	/*	BLK	 */
	GC2355_write_cmos_sensor(0x18,0x02);//1a-lily
	GC2355_write_cmos_sensor(0x1a,0x01);//09-lily //01
	GC2355_write_cmos_sensor(0x40,0x42);//43-lily
	GC2355_write_cmos_sensor(0x4e,0x3c); //BLK select
	GC2355_write_cmos_sensor(0x4f,0x00);
	GC2355_write_cmos_sensor(0x5e,0x00);//18-lily //offset ratio
	GC2355_write_cmos_sensor(0x66,0x20);//20-lily //dark ratio
	GC2355_write_cmos_sensor(0x6a,0x00);//39
	GC2355_write_cmos_sensor(0x6b,0x00);
	GC2355_write_cmos_sensor(0x6c,0x00);
	GC2355_write_cmos_sensor(0x6d,0x00);
	GC2355_write_cmos_sensor(0x6e,0x00);
	GC2355_write_cmos_sensor(0x6f,0x00);
	GC2355_write_cmos_sensor(0x70,0x00);
	GC2355_write_cmos_sensor(0x71,0x00); //manual offset

	/* Dark sun */
	GC2355_write_cmos_sensor(0x87,0x03); //
	GC2355_write_cmos_sensor(0xe0,0xe7); //dark sun en/extend mode
	GC2355_write_cmos_sensor(0xe2,0x03);
	GC2355_write_cmos_sensor(0xe3,0xc0); //clamp

	/*MIPI*/
	  GC2355_write_cmos_sensor(0xfe, 0x03);
	  GC2355_write_cmos_sensor(0x10, 0x81);
	  GC2355_write_cmos_sensor(0x01, 0x87);
	  GC2355_write_cmos_sensor(0x02, 0x11);
	  GC2355_write_cmos_sensor(0x03, 0x91);
	  GC2355_write_cmos_sensor(0x04, 0x01);
	  GC2355_write_cmos_sensor(0x05, 0x00);
	  GC2355_write_cmos_sensor(0x06, 0xa2);

	  GC2355_write_cmos_sensor(0x11, 0x2b);//2b
	  GC2355_write_cmos_sensor(0x15, 0x60);
	  GC2355_write_cmos_sensor(0x12, 0xd0); //d0//40
	  GC2355_write_cmos_sensor(0x13, 0x07); //07//06
	  GC2355_write_cmos_sensor(0x21, 0x10);
	  GC2355_write_cmos_sensor(0x22, 0x03);
	  GC2355_write_cmos_sensor(0x23, 0x20);
	  GC2355_write_cmos_sensor(0x24, 0x02);
	  GC2355_write_cmos_sensor(0x25, 0x10);
	  GC2355_write_cmos_sensor(0x26, 0x08);
	//  GC2355_write_cmos_sensor(0x27, 0x06);
	  GC2355_write_cmos_sensor(0x29, 0x02);//04
	  GC2355_write_cmos_sensor(0x2a, 0x0a);
	  GC2355_write_cmos_sensor(0x2b, 0x08);
	  GC2355_write_cmos_sensor(0x10, 0x81);//91//94//1lane raw8
	  GC2355_write_cmos_sensor(0x40, 0x00);
	  GC2355_write_cmos_sensor(0x41, 0x00);
	  GC2355_write_cmos_sensor(0x42, 0x40);
	  GC2355_write_cmos_sensor(0x43, 0x06);
	  GC2355_write_cmos_sensor(0xfe, 0x00);
}

/*************************************************************************
 * FUNCTION
 *	GC2355Open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 gc2355_read_id(void)
{
  volatile signed char i;
  UINT16 sensor_id = 0;
  for(i=0;i<3;i++)
  {
    sensor_id = ((GC2355_read_cmos_sensor(0xf0) << 8) | GC2355_read_cmos_sensor(0xf1));
    if(sensor_id==GC2355_YUV_ID)
      break;
  }
  YUV_DBG("yuv sensor_id = %x\n",sensor_id);
  return ((sensor_id==GC2355_YUV_ID)?1:0);
}

void gc2355_open(void)
{
  YUV_DBG("%s\n",__func__);
  GC2355_Sensor_Init();
}

YUV_SENSOR_FUNC gc2355_sensor_func = {

  gc2355_open,
  GC2355_MIPI_GetYUVSensorBV,

};

UINT32 GC2355_YUV_SENSOR_INIT(PYUV_SENSOR_FUNC *pfFunc)
{
  if(NULL != pfFunc)
    *pfFunc = &gc2355_sensor_func;
  YUV_DBG("%s\n",__func__);
  return gc2355_read_id();
}
