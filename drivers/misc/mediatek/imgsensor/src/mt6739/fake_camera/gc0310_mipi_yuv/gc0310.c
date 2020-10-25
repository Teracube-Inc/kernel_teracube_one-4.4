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
//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "gc0310.h"
#include "fake_camera_define.h"

#define Sleep(ms) mdelay(ms)

UINT16 GC0310_read_cmos_sensor(kal_uint8 addr)
{
  UINT16 get_byte = 0;
  char puSendCmd = { (char)(addr & 0xFF) };
  kdSetI2CSpeed(100);
  if(0 != iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, GC0310_READ_ID))
  {
    YUV_DBG("ERROR: gc0310MIPI_read_cmos_sensor \n");
    return 0;
  }
  if(get_byte == 0)  //for ata yuv sensor shutter
  {
    get_byte = 1;
  }
  return get_byte;
}

static void GC0310_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
  char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
  kdSetI2CSpeed(100);
  iWriteRegI2C(puSendCmd , 2, GC0310_WRITE_ID);
  //Sleep(10);
  //GC0310_read_cmos_sensor(addr);
}

UINT16 GC0310_MIPI_GetYUVSensorBV(void)
{
  UINT8 temp_reg1, temp_reg2;
  UINT16 shutter,light;

  //GC0310_write_cmos_sensor(0xfe,0x01);
  //light = GC0310_read_cmos_sensor(0x14);

  //GC0310_write_cmos_sensor(0xfe,0x00);
  temp_reg1 = GC0310_read_cmos_sensor(0x04);
  temp_reg2 = GC0310_read_cmos_sensor(0x03);

  shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

  light = GC0310_read_cmos_sensor(0xef);
  YUV_DBG("gc0310 light = %d shutter = %d\n",light,shutter);

  return light;
}

void GC0310_Sensor_Init(void)
{
  /////////////////	system reg	/////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0xfe,0xf0);
  GC0310_write_cmos_sensor(0xfe,0xf0);
  GC0310_write_cmos_sensor(0xfe,0x00);
  GC0310_write_cmos_sensor(0xfc,0x0e);
  GC0310_write_cmos_sensor(0xfc,0x0e);
  GC0310_write_cmos_sensor(0xf2,0x80);
  GC0310_write_cmos_sensor(0xf3,0x00);
  GC0310_write_cmos_sensor(0xf7,0x1b);
  GC0310_write_cmos_sensor(0xf8,0x04); 
  GC0310_write_cmos_sensor(0xf9,0x8e);
  GC0310_write_cmos_sensor(0xfa,0x11);
  /////////////////////////////////////////////////      
  ///////////////////   MIPI   ////////////////////      
  /////////////////////////////////////////////////      
  GC0310_write_cmos_sensor(0xfe,0x03);
  GC0310_write_cmos_sensor(0x40,0x08);
  GC0310_write_cmos_sensor(0x42,0x00);
  GC0310_write_cmos_sensor(0x43,0x00);
  GC0310_write_cmos_sensor(0x01,0x03);
  GC0310_write_cmos_sensor(0x10,0x84);

  GC0310_write_cmos_sensor(0x01,0x03);             
  GC0310_write_cmos_sensor(0x02,0x11);  // 00 20150522        
  GC0310_write_cmos_sensor(0x03,0x94);             
  GC0310_write_cmos_sensor(0x04,0x01);            
  GC0310_write_cmos_sensor(0x05,0x00);             
  GC0310_write_cmos_sensor(0x06,0x80);             
  GC0310_write_cmos_sensor(0x11,0x1e);             
  GC0310_write_cmos_sensor(0x12,0x00);      
  GC0310_write_cmos_sensor(0x13,0x05);             
  GC0310_write_cmos_sensor(0x15,0x10);                                                                    
  GC0310_write_cmos_sensor(0x21,0x10);             
  GC0310_write_cmos_sensor(0x22,0x01);             
  GC0310_write_cmos_sensor(0x23,0x10);                                             
  GC0310_write_cmos_sensor(0x24,0x02);                                             
  GC0310_write_cmos_sensor(0x25,0x10);                                             
  GC0310_write_cmos_sensor(0x26,0x03);                                             
  GC0310_write_cmos_sensor(0x29,0x02);                                             
  GC0310_write_cmos_sensor(0x2a,0x0a);                                             
  GC0310_write_cmos_sensor(0x2b,0x04);                                             
  GC0310_write_cmos_sensor(0xfe,0x00);

  /////////////////////////////////////////////////
  /////////////////  CISCTL reg	/////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0x00,0x2f);
  GC0310_write_cmos_sensor(0x01,0x0f);
  GC0310_write_cmos_sensor(0x02,0x04);
  GC0310_write_cmos_sensor(0x03,0x03);
  GC0310_write_cmos_sensor(0x04,0x50);
  GC0310_write_cmos_sensor(0x09,0x00);
  GC0310_write_cmos_sensor(0x0a,0x00);
  GC0310_write_cmos_sensor(0x0b,0x00);
  GC0310_write_cmos_sensor(0x0c,0x04);
  GC0310_write_cmos_sensor(0x0d,0x01);
  GC0310_write_cmos_sensor(0x0e,0xe8);
  GC0310_write_cmos_sensor(0x0f,0x02);
  GC0310_write_cmos_sensor(0x10,0x88);
  GC0310_write_cmos_sensor(0x16,0x00);	
  GC0310_write_cmos_sensor(0x17,0x14);
  GC0310_write_cmos_sensor(0x18,0x1a);
  GC0310_write_cmos_sensor(0x19,0x14);
  GC0310_write_cmos_sensor(0x1b,0x48);
  GC0310_write_cmos_sensor(0x1e,0x6b);
  GC0310_write_cmos_sensor(0x1f,0x28);
  GC0310_write_cmos_sensor(0x20,0x8b);//0x89 travis20140801
  GC0310_write_cmos_sensor(0x21,0x49);
  GC0310_write_cmos_sensor(0x22,0xb0);
  GC0310_write_cmos_sensor(0x23,0x04);
  GC0310_write_cmos_sensor(0x24,0x16);
  GC0310_write_cmos_sensor(0x34,0x20);

  /////////////////////////////////////////////////
  ////////////////////   BLK	 ////////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0x26,0x23);
  GC0310_write_cmos_sensor(0x28,0xff);
  GC0310_write_cmos_sensor(0x29,0x00);
  GC0310_write_cmos_sensor(0x33,0x10); 
  GC0310_write_cmos_sensor(0x37,0x20);
  GC0310_write_cmos_sensor(0x38,0x10);
  GC0310_write_cmos_sensor(0x47,0x80);
  GC0310_write_cmos_sensor(0x4e,0x66);
  GC0310_write_cmos_sensor(0xa8,0x02);
  GC0310_write_cmos_sensor(0xa9,0x80);

  /////////////////////////////////////////////////
  //////////////////	ISP reg   ///////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0x40,0xff);
  GC0310_write_cmos_sensor(0x41,0x21);
  GC0310_write_cmos_sensor(0x42,0xcf);
  GC0310_write_cmos_sensor(0x44,0x02);
  GC0310_write_cmos_sensor(0x45,0xa8); 
  GC0310_write_cmos_sensor(0x46,0x02); 
  GC0310_write_cmos_sensor(0x4a,0x11);
  GC0310_write_cmos_sensor(0x4b,0x01);
  GC0310_write_cmos_sensor(0x4c,0x20);
  GC0310_write_cmos_sensor(0x4d,0x05);
  GC0310_write_cmos_sensor(0x4f,0x01);
  GC0310_write_cmos_sensor(0x50,0x01);
  GC0310_write_cmos_sensor(0x55,0x01);
  GC0310_write_cmos_sensor(0x56,0xe0);
  GC0310_write_cmos_sensor(0x57,0x02);
  GC0310_write_cmos_sensor(0x58,0x80);

  /////////////////////////////////////////////////
  ///////////////////   GAIN   ////////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0x70,0x70);
  GC0310_write_cmos_sensor(0x5a,0x84);
  GC0310_write_cmos_sensor(0x5b,0xc9);
  GC0310_write_cmos_sensor(0x5c,0xed);
  GC0310_write_cmos_sensor(0x77,0x74);
  GC0310_write_cmos_sensor(0x78,0x40);
  GC0310_write_cmos_sensor(0x79,0x5f);

  ///////////////////////////////////////////////// 
  ///////////////////   DNDD  /////////////////////
  ///////////////////////////////////////////////// 
  GC0310_write_cmos_sensor(0x82,0x14); 
  GC0310_write_cmos_sensor(0x83,0x0b);
  GC0310_write_cmos_sensor(0x89,0xf0);

  ///////////////////////////////////////////////// 
  //////////////////   EEINTP  ////////////////////
  ///////////////////////////////////////////////// 
  GC0310_write_cmos_sensor(0x8f,0xaa); 
  GC0310_write_cmos_sensor(0x90,0x8c); 
  GC0310_write_cmos_sensor(0x91,0x90);
  GC0310_write_cmos_sensor(0x92,0x03); 
  GC0310_write_cmos_sensor(0x93,0x03); 
  GC0310_write_cmos_sensor(0x94,0x05); 
  GC0310_write_cmos_sensor(0x95,0x65); 
  GC0310_write_cmos_sensor(0x96,0xf0); 

  ///////////////////////////////////////////////// 
  /////////////////////  ASDE  ////////////////////
  ///////////////////////////////////////////////// 
  GC0310_write_cmos_sensor(0xfe,0x00);

  GC0310_write_cmos_sensor(0x9a,0x20);
  GC0310_write_cmos_sensor(0x9b,0x80);
  GC0310_write_cmos_sensor(0x9c,0x40);
  GC0310_write_cmos_sensor(0x9d,0x80);

  GC0310_write_cmos_sensor(0xa1,0x30);
  GC0310_write_cmos_sensor(0xa2,0x32);
  GC0310_write_cmos_sensor(0xa4,0x30);
  GC0310_write_cmos_sensor(0xa5,0x30);
  GC0310_write_cmos_sensor(0xaa,0x10); 
  GC0310_write_cmos_sensor(0xac,0x22);


  /////////////////////////////////////////////////
  ///////////////////   GAMMA   ///////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0xfe,0x00);//default
  GC0310_write_cmos_sensor(0xbf,0x08);
  GC0310_write_cmos_sensor(0xc0,0x16);
  GC0310_write_cmos_sensor(0xc1,0x28);
  GC0310_write_cmos_sensor(0xc2,0x41);
  GC0310_write_cmos_sensor(0xc3,0x5a);
  GC0310_write_cmos_sensor(0xc4,0x6c);
  GC0310_write_cmos_sensor(0xc5,0x7a);
  GC0310_write_cmos_sensor(0xc6,0x96);
  GC0310_write_cmos_sensor(0xc7,0xac);
  GC0310_write_cmos_sensor(0xc8,0xbc);
  GC0310_write_cmos_sensor(0xc9,0xc9);
  GC0310_write_cmos_sensor(0xca,0xd3);
  GC0310_write_cmos_sensor(0xcb,0xdd);
  GC0310_write_cmos_sensor(0xcc,0xe5);
  GC0310_write_cmos_sensor(0xcd,0xf1);
  GC0310_write_cmos_sensor(0xce,0xfa);
  GC0310_write_cmos_sensor(0xcf,0xff);

  /* 
     GC0310_write_cmos_sensor(0xfe,0x00);//big gamma
     GC0310_write_cmos_sensor(0xbf,0x08);
     GC0310_write_cmos_sensor(0xc0,0x1d);
     GC0310_write_cmos_sensor(0xc1,0x34);
     GC0310_write_cmos_sensor(0xc2,0x4b);
     GC0310_write_cmos_sensor(0xc3,0x60);
     GC0310_write_cmos_sensor(0xc4,0x73);
     GC0310_write_cmos_sensor(0xc5,0x85);
     GC0310_write_cmos_sensor(0xc6,0x9f);
     GC0310_write_cmos_sensor(0xc7,0xb5);
     GC0310_write_cmos_sensor(0xc8,0xc7);
     GC0310_write_cmos_sensor(0xc9,0xd5);
     GC0310_write_cmos_sensor(0xca,0xe0);
     GC0310_write_cmos_sensor(0xcb,0xe7);
     GC0310_write_cmos_sensor(0xcc,0xec);
     GC0310_write_cmos_sensor(0xcd,0xf4);
     GC0310_write_cmos_sensor(0xce,0xfa);
     GC0310_write_cmos_sensor(0xcf,0xff);
   */	

  /*
     GC0310_write_cmos_sensor(0xfe,0x00);//small gamma
     GC0310_write_cmos_sensor(0xbf,0x08);
     GC0310_write_cmos_sensor(0xc0,0x18);
     GC0310_write_cmos_sensor(0xc1,0x2c);
     GC0310_write_cmos_sensor(0xc2,0x41);
     GC0310_write_cmos_sensor(0xc3,0x59);
     GC0310_write_cmos_sensor(0xc4,0x6e);
     GC0310_write_cmos_sensor(0xc5,0x81);
     GC0310_write_cmos_sensor(0xc6,0x9f);
     GC0310_write_cmos_sensor(0xc7,0xb5);
     GC0310_write_cmos_sensor(0xc8,0xc7);
     GC0310_write_cmos_sensor(0xc9,0xd5);
     GC0310_write_cmos_sensor(0xca,0xe0);
     GC0310_write_cmos_sensor(0xcb,0xe7);
     GC0310_write_cmos_sensor(0xcc,0xec);
     GC0310_write_cmos_sensor(0xcd,0xf4);
     GC0310_write_cmos_sensor(0xce,0xfa);
     GC0310_write_cmos_sensor(0xcf,0xff);
   */
  /////////////////////////////////////////////////
  ///////////////////   YCP  //////////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0xd0,0x40);
  GC0310_write_cmos_sensor(0xd1,0x34); 
  GC0310_write_cmos_sensor(0xd2,0x34); 
  GC0310_write_cmos_sensor(0xd3,0x40); 
  GC0310_write_cmos_sensor(0xd6,0xf2);
  GC0310_write_cmos_sensor(0xd7,0x1b);
  GC0310_write_cmos_sensor(0xd8,0x18);
  GC0310_write_cmos_sensor(0xdd,0x03); 

  /////////////////////////////////////////////////
  ////////////////////   AEC   ////////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0xfe,0x01);
  GC0310_write_cmos_sensor(0x05,0x30); 
  GC0310_write_cmos_sensor(0x06,0x75); 
  GC0310_write_cmos_sensor(0x07,0x40); 
  GC0310_write_cmos_sensor(0x08,0xb0); 
  GC0310_write_cmos_sensor(0x0a,0xc5); 
  GC0310_write_cmos_sensor(0x0b,0x11); 
  GC0310_write_cmos_sensor(0x0c,0x00);
  GC0310_write_cmos_sensor(0x12,0x52); 
  GC0310_write_cmos_sensor(0x13,0x38); 
  GC0310_write_cmos_sensor(0x18,0x95); 
  GC0310_write_cmos_sensor(0x19,0x96); 
  GC0310_write_cmos_sensor(0x1f,0x20); 
  GC0310_write_cmos_sensor(0x20,0xc0); 
  GC0310_write_cmos_sensor(0x3e,0x40); 
  GC0310_write_cmos_sensor(0x3f,0x57); 
  GC0310_write_cmos_sensor(0x40,0x7d); 
  GC0310_write_cmos_sensor(0x03,0x60);
  GC0310_write_cmos_sensor(0x44,0x02);

  /////////////////////////////////////////////////
  ////////////////////   AWB   ////////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0x1c,0x91); 
  GC0310_write_cmos_sensor(0x21,0x15); 
  GC0310_write_cmos_sensor(0x50,0x80); 
  GC0310_write_cmos_sensor(0x56,0x04); 
  GC0310_write_cmos_sensor(0x59,0x08); 
  GC0310_write_cmos_sensor(0x5b,0x02);
  GC0310_write_cmos_sensor(0x61,0x8d); 
  GC0310_write_cmos_sensor(0x62,0xa7); 
  GC0310_write_cmos_sensor(0x63,0xd0); 
  GC0310_write_cmos_sensor(0x65,0x06);
  GC0310_write_cmos_sensor(0x66,0x06); 
  GC0310_write_cmos_sensor(0x67,0x84); 
  GC0310_write_cmos_sensor(0x69,0x08);
  GC0310_write_cmos_sensor(0x6a,0x25);
  GC0310_write_cmos_sensor(0x6b,0x01); 
  GC0310_write_cmos_sensor(0x6c,0x00); 
  GC0310_write_cmos_sensor(0x6d,0x02); 
  GC0310_write_cmos_sensor(0x6e,0xf0); 
  GC0310_write_cmos_sensor(0x6f,0x80); 
  GC0310_write_cmos_sensor(0x76,0x80);
  GC0310_write_cmos_sensor(0x78,0xaf); 
  GC0310_write_cmos_sensor(0x79,0x75);
  GC0310_write_cmos_sensor(0x7a,0x40);
  GC0310_write_cmos_sensor(0x7b,0x50);	
  GC0310_write_cmos_sensor(0x7c,0x0c);

  GC0310_write_cmos_sensor(0xa4,0xb9); 
  GC0310_write_cmos_sensor(0xa5,0xa0);
  GC0310_write_cmos_sensor(0x90,0xc9);
  GC0310_write_cmos_sensor(0x91,0xbe);

  GC0310_write_cmos_sensor(0xa6,0xb8);
  GC0310_write_cmos_sensor(0xa7,0x95);
  GC0310_write_cmos_sensor(0x92,0xe6);
  GC0310_write_cmos_sensor(0x93,0xca);

  GC0310_write_cmos_sensor(0xa9,0xbc); 
  GC0310_write_cmos_sensor(0xaa,0x95); 
  GC0310_write_cmos_sensor(0x95,0x23);
  GC0310_write_cmos_sensor(0x96,0xe7);

  GC0310_write_cmos_sensor(0xab,0x9d);
  GC0310_write_cmos_sensor(0xac,0x80);
  GC0310_write_cmos_sensor(0x97,0x43);
  GC0310_write_cmos_sensor(0x98,0x24);

  GC0310_write_cmos_sensor(0xae,0xb7);
  GC0310_write_cmos_sensor(0xaf,0x9e);
  GC0310_write_cmos_sensor(0x9a,0x43);
  GC0310_write_cmos_sensor(0x9b,0x24);

  GC0310_write_cmos_sensor(0xb0,0xc8);
  GC0310_write_cmos_sensor(0xb1,0x97);
  GC0310_write_cmos_sensor(0x9c,0xc4);
  GC0310_write_cmos_sensor(0x9d,0x44);

  GC0310_write_cmos_sensor(0xb3,0xb7);
  GC0310_write_cmos_sensor(0xb4,0x7f);
  GC0310_write_cmos_sensor(0x9f,0xc7);
  GC0310_write_cmos_sensor(0xa0,0xc8);

  GC0310_write_cmos_sensor(0xb5,0x00);
  GC0310_write_cmos_sensor(0xb6,0x00);
  GC0310_write_cmos_sensor(0xa1,0x00);
  GC0310_write_cmos_sensor(0xa2,0x00);

  GC0310_write_cmos_sensor(0x86,0x60);
  GC0310_write_cmos_sensor(0x87,0x08);
  GC0310_write_cmos_sensor(0x88,0x00);
  GC0310_write_cmos_sensor(0x89,0x00);
  GC0310_write_cmos_sensor(0x8b,0xde);
  GC0310_write_cmos_sensor(0x8c,0x80);
  GC0310_write_cmos_sensor(0x8d,0x00);
  GC0310_write_cmos_sensor(0x8e,0x00);

  GC0310_write_cmos_sensor(0x94,0x55);
  GC0310_write_cmos_sensor(0x99,0xa6);
  GC0310_write_cmos_sensor(0x9e,0xaa);
  GC0310_write_cmos_sensor(0xa3,0x0a);
  GC0310_write_cmos_sensor(0x8a,0x0a);
  GC0310_write_cmos_sensor(0xa8,0x55);
  GC0310_write_cmos_sensor(0xad,0x55);
  GC0310_write_cmos_sensor(0xb2,0x55);
  GC0310_write_cmos_sensor(0xb7,0x05);
  GC0310_write_cmos_sensor(0x8f,0x05);

  GC0310_write_cmos_sensor(0xb8,0xcc); 
  GC0310_write_cmos_sensor(0xb9,0x9a); 
  /////////////////////////////////////////////////
  ////////////////////   CC    ////////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0xfe,0x01);

  GC0310_write_cmos_sensor(0xd0,0x38);//skin red
  GC0310_write_cmos_sensor(0xd1,0x00);
  GC0310_write_cmos_sensor(0xd2,0x02);
  GC0310_write_cmos_sensor(0xd3,0x04);
  GC0310_write_cmos_sensor(0xd4,0x38);
  GC0310_write_cmos_sensor(0xd5,0x12);

  /*       
           GC0310_write_cmos_sensor(0xd0,0x38);//skin white
           GC0310_write_cmos_sensor(0xd1,0xfd);
           GC0310_write_cmos_sensor(0xd2,0x06);
           GC0310_write_cmos_sensor(0xd3,0xf0);
           GC0310_write_cmos_sensor(0xd4,0x40);
           GC0310_write_cmos_sensor(0xd5,0x08);              
   */

  /*                       
                           GC0310_write_cmos_sensor(0xd0,0x38);//guodengxiang
                           GC0310_write_cmos_sensor(0xd1,0xf8);
                           GC0310_write_cmos_sensor(0xd2,0x06);
                           GC0310_write_cmos_sensor(0xd3,0xfd);
                           GC0310_write_cmos_sensor(0xd4,0x40);
                           GC0310_write_cmos_sensor(0xd5,0x00);
   */
  GC0310_write_cmos_sensor(0xd6,0x30);
  GC0310_write_cmos_sensor(0xd7,0x00);
  GC0310_write_cmos_sensor(0xd8,0x0a);
  GC0310_write_cmos_sensor(0xd9,0x16);
  GC0310_write_cmos_sensor(0xda,0x39);
  GC0310_write_cmos_sensor(0xdb,0xf8);

  /////////////////////////////////////////////////
  ////////////////////   LSC   ////////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0xfe,0x01);
  GC0310_write_cmos_sensor(0xc1,0x3c);
  GC0310_write_cmos_sensor(0xc2,0x50);
  GC0310_write_cmos_sensor(0xc3,0x00);
  GC0310_write_cmos_sensor(0xc4,0x40);
  GC0310_write_cmos_sensor(0xc5,0x30);
  GC0310_write_cmos_sensor(0xc6,0x30);
  GC0310_write_cmos_sensor(0xc7,0x10);
  GC0310_write_cmos_sensor(0xc8,0x00);
  GC0310_write_cmos_sensor(0xc9,0x00);
  GC0310_write_cmos_sensor(0xdc,0x20);
  GC0310_write_cmos_sensor(0xdd,0x10);
  GC0310_write_cmos_sensor(0xdf,0x00);
  GC0310_write_cmos_sensor(0xde,0x00);

  /////////////////////////////////////////////////
  ///////////////////  Histogram	/////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0x01,0x10);
  GC0310_write_cmos_sensor(0x0b,0x31);
  GC0310_write_cmos_sensor(0x0e,0x50);
  GC0310_write_cmos_sensor(0x0f,0x0f);
  GC0310_write_cmos_sensor(0x10,0x6e);
  GC0310_write_cmos_sensor(0x12,0xa0);
  GC0310_write_cmos_sensor(0x15,0x60);
  GC0310_write_cmos_sensor(0x16,0x60);
  GC0310_write_cmos_sensor(0x17,0xe0);

  /////////////////////////////////////////////////
  //////////////	Measure Window	  ///////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0xcc,0x0c); 
  GC0310_write_cmos_sensor(0xcd,0x10);
  GC0310_write_cmos_sensor(0xce,0xa0);
  GC0310_write_cmos_sensor(0xcf,0xe6);

  /////////////////////////////////////////////////
  /////////////////	dark sun   //////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0x45,0xf7);
  GC0310_write_cmos_sensor(0x46,0xff);
  GC0310_write_cmos_sensor(0x47,0x15);
  GC0310_write_cmos_sensor(0x48,0x03); 
  GC0310_write_cmos_sensor(0x4f,0x60);

  //////////////////banding//////////////////////
  GC0310_write_cmos_sensor(0xfe,0x00);
  GC0310_write_cmos_sensor(0x05,0x02);
  GC0310_write_cmos_sensor(0x06,0xd1); //HB
  GC0310_write_cmos_sensor(0x07,0x00);
  GC0310_write_cmos_sensor(0x08,0x22); //VB
  GC0310_write_cmos_sensor(0xfe,0x01);
  GC0310_write_cmos_sensor(0x25,0x00); //step 
  GC0310_write_cmos_sensor(0x26,0x6a); 
  GC0310_write_cmos_sensor(0x27,0x02); //20fps
  GC0310_write_cmos_sensor(0x28,0x12);  
  GC0310_write_cmos_sensor(0x29,0x03); //12.5fps
  GC0310_write_cmos_sensor(0x2a,0x53); 
  GC0310_write_cmos_sensor(0x2b,0x05); //7.14fps
  GC0310_write_cmos_sensor(0x2c,0xcc); 
  GC0310_write_cmos_sensor(0x2d,0x07); //5.55fps
  GC0310_write_cmos_sensor(0x2e,0x74);
  GC0310_write_cmos_sensor(0x3c,0x30);
  GC0310_write_cmos_sensor(0xfe,0x00);

  /////////////////////////////////////////////////
  ///////////////////   MIPI	 ////////////////////
  /////////////////////////////////////////////////
  GC0310_write_cmos_sensor(0xfe,0x03);
  GC0310_write_cmos_sensor(0x10,0x94);
  GC0310_write_cmos_sensor(0xfe,0x00); 
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
  UINT16 sensor_id = 0;
  for(i=0;i<3;i++)
  {
    sensor_id = ((GC0310_read_cmos_sensor(0xf0) << 8) | GC0310_read_cmos_sensor(0xf1));
    if(sensor_id==GC0310_YUV_ID)
      break;
  }
  YUV_DBG("yuv sensor_id = %x\n",sensor_id);
  return ((sensor_id==GC0310_YUV_ID)?1:0);
}

void gc0310_open(void)
{
  YUV_DBG("%s\n",__func__);
  GC0310_Sensor_Init();
}

YUV_SENSOR_FUNC gc0310_sensor_func = {

  gc0310_open,
  GC0310_MIPI_GetYUVSensorBV,

};

UINT32 GC0310_YUV_SENSOR_INIT(PYUV_SENSOR_FUNC *pfFunc)
{
  if(NULL != pfFunc)
    *pfFunc = &gc0310_sensor_func;
  YUV_DBG("%s\n",__func__);
  return gc0310_read_id();
}
