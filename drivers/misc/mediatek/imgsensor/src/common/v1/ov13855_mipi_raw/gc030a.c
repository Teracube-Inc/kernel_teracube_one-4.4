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
 *   gc0310yuv_Sensor.c
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

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc030a.h"


#define Sleep(ms) mdelay(ms)

kal_uint16 GC0310_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint16 get_byte=0;
  char puSendCmd = { (char)(addr & 0xFF) };
   if(0 != iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, GC0310_READ_ID))
  {
    printk("ERROR: gc0310MIPI_read_cmos_sensor \n");
    return 0;
  }
  if(get_byte == 0)  //for ata yuv sensor shutter
  {
    get_byte = 1;
  }
  return get_byte;
}

kal_uint16 GC0310_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
  char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
    iWriteRegI2C(puSendCmd , 2, GC0310_WRITE_ID);
  //Sleep(10);
  //GC0310_read_cmos_sensor(addr);
  return 0;
}

kal_uint16 GC0310_Read_Shutter(void)
{
  kal_uint8 temp_reg1, temp_reg2;
  kal_uint16 shutter,light;

  //GC0310_write_cmos_sensor(0xfe,0x01);
  //light = GC0310_read_cmos_sensor(0x14);

  //GC0310_write_cmos_sensor(0xfe,0x00);
  //temp_reg1 = GC0310_read_cmos_sensor(0x04);
  //temp_reg2 = GC0310_read_cmos_sensor(0x03);

  //shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

  light = GC0310_read_cmos_sensor(0xef);
  printk("gc0310 light = %d\n",light);

  return light;
}

void GC0310_Sensor_Init(void)
{
  /////////////////	system reg	/////////////////
  /////////////////////////////////////////////////
 	/*SYS*/
	GC0310_write_cmos_sensor(0xfe,0x80);
	GC0310_write_cmos_sensor(0xfe,0x80);
	GC0310_write_cmos_sensor(0xfe,0x80);
	GC0310_write_cmos_sensor(0xf7,0x01);
	GC0310_write_cmos_sensor(0xf8,0x05);
	GC0310_write_cmos_sensor(0xf9,0x0f);
	GC0310_write_cmos_sensor(0xfa,0x00);
	GC0310_write_cmos_sensor(0xfc,0x0f);
	GC0310_write_cmos_sensor(0xfe,0x00);
	
	/*ANALOG & CISCTL*/
	GC0310_write_cmos_sensor(0x03,0x01);
	GC0310_write_cmos_sensor(0x04,0xc8);
	GC0310_write_cmos_sensor(0x05,0x03);
	GC0310_write_cmos_sensor(0x06,0x7b);
	GC0310_write_cmos_sensor(0x07,0x00);
	GC0310_write_cmos_sensor(0x08,0x06);
	GC0310_write_cmos_sensor(0x0a,0x00);
	GC0310_write_cmos_sensor(0x0c,0x08);
	GC0310_write_cmos_sensor(0x0d,0x01);
	GC0310_write_cmos_sensor(0x0e,0xe8);
	GC0310_write_cmos_sensor(0x0f,0x02);
	GC0310_write_cmos_sensor(0x10,0x88);
	GC0310_write_cmos_sensor(0x12,0x28);//23 add 20170110	
	GC0310_write_cmos_sensor(0x17,0x54);//Don't Change Here!!!
	GC0310_write_cmos_sensor(0x18,0x12);
	GC0310_write_cmos_sensor(0x19,0x07);
	GC0310_write_cmos_sensor(0x1a,0x1b);
	GC0310_write_cmos_sensor(0x1d,0x48);//40 travis20160318
	GC0310_write_cmos_sensor(0x1e,0x50);
	GC0310_write_cmos_sensor(0x1f,0x80);
	GC0310_write_cmos_sensor(0x23,0x01);
	GC0310_write_cmos_sensor(0x24,0xc8);
	GC0310_write_cmos_sensor(0x27,0xaf);
	GC0310_write_cmos_sensor(0x28,0x24);
	GC0310_write_cmos_sensor(0x29,0x1a);
	GC0310_write_cmos_sensor(0x2f,0x14);
	GC0310_write_cmos_sensor(0x30,0x00);
	GC0310_write_cmos_sensor(0x31,0x04);
	GC0310_write_cmos_sensor(0x32,0x08);
	GC0310_write_cmos_sensor(0x33,0x0c);
	GC0310_write_cmos_sensor(0x34,0x0d);
	GC0310_write_cmos_sensor(0x35,0x0e);
	GC0310_write_cmos_sensor(0x36,0x0f);	 
	GC0310_write_cmos_sensor(0x72,0x98); 
	GC0310_write_cmos_sensor(0x73,0x9a);
	GC0310_write_cmos_sensor(0x74,0x47);
	GC0310_write_cmos_sensor(0x76,0x82);
	GC0310_write_cmos_sensor(0x7a,0xcb);	
	GC0310_write_cmos_sensor(0xc2,0x0c);
	GC0310_write_cmos_sensor(0xce,0x03);
	GC0310_write_cmos_sensor(0xcf,0x48);
	GC0310_write_cmos_sensor(0xd0,0x10);
	GC0310_write_cmos_sensor(0xdc,0x75);	
	GC0310_write_cmos_sensor(0xeb,0x78);
	
	/*ISP*/
	GC0310_write_cmos_sensor(0x90,0x01);
	GC0310_write_cmos_sensor(0x92,0x01);//Don't Change Here!!!
	GC0310_write_cmos_sensor(0x94,0x01);//Don't Change Here!!!
	GC0310_write_cmos_sensor(0x95,0x01);
	GC0310_write_cmos_sensor(0x96,0xe0);
	GC0310_write_cmos_sensor(0x97,0x02);
	GC0310_write_cmos_sensor(0x98,0x80);
	
	/*Gain*/
	GC0310_write_cmos_sensor(0xb0,0x46);
	GC0310_write_cmos_sensor(0xb1,0x01);
	GC0310_write_cmos_sensor(0xb2,0x00);
	GC0310_write_cmos_sensor(0xb3,0x40);
	GC0310_write_cmos_sensor(0xb4,0x40);
	GC0310_write_cmos_sensor(0xb5,0x40);
	GC0310_write_cmos_sensor(0xb6,0x00);
	
	/*BLK*/
	GC0310_write_cmos_sensor(0x40,0x26);
	GC0310_write_cmos_sensor(0x4e,0x00);
	GC0310_write_cmos_sensor(0x4f,0x3c);
	
	/*Dark Sun*/
	GC0310_write_cmos_sensor(0xe0,0x9f);
	GC0310_write_cmos_sensor(0xe1,0x90); 
	GC0310_write_cmos_sensor(0xe4,0x0f);
	GC0310_write_cmos_sensor(0xe5,0xff);
		
	/*MIPI*/
	GC0310_write_cmos_sensor(0xfe,0x03);
	GC0310_write_cmos_sensor(0x01,0x03);
	GC0310_write_cmos_sensor(0x02,0x33);
	GC0310_write_cmos_sensor(0x03,0x96);
	GC0310_write_cmos_sensor(0x04,0x01);
	GC0310_write_cmos_sensor(0x05,0x00);
	GC0310_write_cmos_sensor(0x06,0x80);
	GC0310_write_cmos_sensor(0x10,0x90);
	GC0310_write_cmos_sensor(0x11,0x2b);
	GC0310_write_cmos_sensor(0x12,0x20);
	GC0310_write_cmos_sensor(0x13,0x03);
	GC0310_write_cmos_sensor(0x15,0x00);
	GC0310_write_cmos_sensor(0x21,0x10);
	GC0310_write_cmos_sensor(0x22,0x00);
	GC0310_write_cmos_sensor(0x23,0x30);
	GC0310_write_cmos_sensor(0x24,0x02);
	GC0310_write_cmos_sensor(0x25,0x12);
	GC0310_write_cmos_sensor(0x26,0x02);
	GC0310_write_cmos_sensor(0x29,0x01);
	GC0310_write_cmos_sensor(0x2a,0x0a);
	GC0310_write_cmos_sensor(0x2b,0x03);
	
	GC0310_write_cmos_sensor(0xfe,0x00);
	GC0310_write_cmos_sensor(0xf9,0x0e);
	GC0310_write_cmos_sensor(0xfc,0x0e);	
	GC0310_write_cmos_sensor(0xfe,0x00);
	GC0310_write_cmos_sensor(0x25,0xa2);
	GC0310_write_cmos_sensor(0x3f,0x1a);
	Sleep(100);
	GC0310_write_cmos_sensor(0x25,0xe2);
}

/*************************************************************************
 * FUNCTION
 *	GC0310Open
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
UINT32 gc0310_read_id(void)
{
  volatile signed char i;
  kal_uint16 sensor_id=0;
  for(i=0;i<3;i++)
  {
    sensor_id = ((GC0310_read_cmos_sensor(0xf0) << 8) | GC0310_read_cmos_sensor(0xf1));
  printk("yuting :for in :0x%x\n",sensor_id);
    if(sensor_id==0x030a)
    {
      break;
    }
  }
  printk("yuting :for end :0x%x\n",sensor_id);
  return ((sensor_id==0x030a)?1:0);
}

UINT32 gc0310open(void)
{
  printk("[gc0310] %s \n",__func__);
  gc0310_read_id();
  GC0310_Sensor_Init();
  return 1;



}
