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
 *   SP0A38yuv_Sensor.c
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

#include "sp0a38.h"
#include "fake_camera_define.h"

#define SP0A38_SET_REG0x10 0xc8
#define SP0A38_SET_REG0x11 0xca
#define SP0A38_SET_REG0x12 0xca
#define SP0A38_SET_REG0x13 0xca
#define SP0A38_SET_REG0x14 0xd8
#define SP0A38_SET_REG0x15 0xe0
#define SP0A38_SET_REG0x16 0xe0-0x08
#define SP0A38_SET_REG0x17 0xe0-0x10

static UINT16 SP0A38_read_cmos_sensor(UINT8 addr)
{
  UINT16 get_byte = 0;
  char puSendCmd = { (char)(addr & 0xFF) };
  kdSetI2CSpeed(100);
  if(0 != iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, SP0A38_READ_ID))
  {
    YUV_DBG("ERROR: SP0A38MIPI_read_cmos_sensor \n");
    return 0;
  }
  if(get_byte == 0)  //for ata yuv sensor shutter
  {
    get_byte = 1;
  }
  return get_byte;
}

static void SP0A38_write_cmos_sensor(UINT8 addr, UINT8 para)
{
  char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
  kdSetI2CSpeed(100);
  iWriteRegI2C(puSendCmd , 2, SP0A38_WRITE_ID);
}

UINT16 SP0A38_MIPI_GetYUVSensorBV(void)
{
  UINT8 temp_reg1, temp_reg2;
  UINT16 shutter,light;

  SP0A38_write_cmos_sensor(0xfd,0x00);
  temp_reg1 = SP0A38_read_cmos_sensor(0x04); //LOW 8
  temp_reg2 = SP0A38_read_cmos_sensor(0x03); //HI 3

  shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

  SP0A38_write_cmos_sensor(0xfd,0x01);
  light = SP0A38_read_cmos_sensor(0xf1);
  YUV_DBG("light = %d shutter = %d\n",light,shutter);

  return light;
}

void SP0A38_Sensor_Init(void)
{
  SP0A38_write_cmos_sensor(0xfd,0x00);
  SP0A38_write_cmos_sensor(0x0b,0x02); //0x2d for 30fps
  SP0A38_write_cmos_sensor(0x0c,0x00);
  SP0A38_write_cmos_sensor(0x0f,0xff);
  SP0A38_write_cmos_sensor(0x10,0xfe);
  SP0A38_write_cmos_sensor(0x1a,0x0d);
  SP0A38_write_cmos_sensor(0x1b,0x0c);
  SP0A38_write_cmos_sensor(0x1d,0x0a);
  SP0A38_write_cmos_sensor(0x21,0x4a);
  SP0A38_write_cmos_sensor(0x22,0x35);
  SP0A38_write_cmos_sensor(0x27,0xbb);
  SP0A38_write_cmos_sensor(0x28,0x7d);
  SP0A38_write_cmos_sensor(0x1b,0x30);
  SP0A38_write_cmos_sensor(0x1c,0x03);
  SP0A38_write_cmos_sensor(0x2d,0x40);
  SP0A38_write_cmos_sensor(0x1d,0x0f);
  SP0A38_write_cmos_sensor(0x1a,0x0d);
  SP0A38_write_cmos_sensor(0x13,0x18);
  SP0A38_write_cmos_sensor(0x6c,0x19);
  SP0A38_write_cmos_sensor(0x6d,0x19);
  SP0A38_write_cmos_sensor(0x6f,0x19);
  SP0A38_write_cmos_sensor(0x6e,0x1a);
  SP0A38_write_cmos_sensor(0x16,0x26);
  SP0A38_write_cmos_sensor(0x17,0x26);
  SP0A38_write_cmos_sensor(0x70,0x28);
  SP0A38_write_cmos_sensor(0x69,0x22);
  SP0A38_write_cmos_sensor(0x71,0x21);
  SP0A38_write_cmos_sensor(0x14,0x00);
  SP0A38_write_cmos_sensor(0x15,0x20);
  SP0A38_write_cmos_sensor(0x6a,0x18);
  SP0A38_write_cmos_sensor(0x72,0x1a);
  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0x32,0x00);
  SP0A38_write_cmos_sensor(0x33,0xef);
  SP0A38_write_cmos_sensor(0x34,0xef);
  SP0A38_write_cmos_sensor(0x35,0x00);
  SP0A38_write_cmos_sensor(0xfb,0x23);
  SP0A38_write_cmos_sensor(0xfd,0x00);
  SP0A38_write_cmos_sensor(0x09,0x00);
  SP0A38_write_cmos_sensor(0x0a,0x00);
  SP0A38_write_cmos_sensor(0xfd,0x02);
  SP0A38_write_cmos_sensor(0x00,0x80);
  SP0A38_write_cmos_sensor(0x01,0x80);
  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0x22,0x00);
  SP0A38_write_cmos_sensor(0x23,0x00);
  SP0A38_write_cmos_sensor(0x24,0x00);
  SP0A38_write_cmos_sensor(0x25,0x00);

  SP0A38_write_cmos_sensor(0xfd,0x00);
  SP0A38_write_cmos_sensor(0x03,0x01);
  SP0A38_write_cmos_sensor(0x04,0xaa);
  SP0A38_write_cmos_sensor(0x05,0x00);
  SP0A38_write_cmos_sensor(0x06,0x00);
  SP0A38_write_cmos_sensor(0x07,0x00);
  SP0A38_write_cmos_sensor(0x08,0x00);
  SP0A38_write_cmos_sensor(0x09,0x00);
  SP0A38_write_cmos_sensor(0x0a,0x00);
  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0xf7,0x47);
  SP0A38_write_cmos_sensor(0x02,0x0c);
  SP0A38_write_cmos_sensor(0x03,0x01);
  SP0A38_write_cmos_sensor(0x06,0x47);
  SP0A38_write_cmos_sensor(0x07,0x00);
  SP0A38_write_cmos_sensor(0x08,0x01);
  SP0A38_write_cmos_sensor(0x09,0x00);
  SP0A38_write_cmos_sensor(0xfd,0x02);
  SP0A38_write_cmos_sensor(0xbe,0x54);
  SP0A38_write_cmos_sensor(0xbf,0x03);
  SP0A38_write_cmos_sensor(0xd0,0x54);
  SP0A38_write_cmos_sensor(0xd1,0x03);

  SP0A38_write_cmos_sensor(0xfd,0x02);
  SP0A38_write_cmos_sensor(0xb8,0x60);
  SP0A38_write_cmos_sensor(0xb9,0x70);
  SP0A38_write_cmos_sensor(0xba,0x28);
  SP0A38_write_cmos_sensor(0xbb,0x38);
  SP0A38_write_cmos_sensor(0xbc,0x60);
  SP0A38_write_cmos_sensor(0xbd,0x40);
  //RPC
  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0xc0,0x4c);
  SP0A38_write_cmos_sensor(0xc1,0x3a);
  SP0A38_write_cmos_sensor(0xc2,0x32);
  SP0A38_write_cmos_sensor(0xc3,0x30);
  SP0A38_write_cmos_sensor(0xc4,0x2e);
  SP0A38_write_cmos_sensor(0xc5,0x2c);
  SP0A38_write_cmos_sensor(0xc6,0x2c);
  SP0A38_write_cmos_sensor(0xc7,0x2a);
  SP0A38_write_cmos_sensor(0xc8,0x2a);
  SP0A38_write_cmos_sensor(0xc9,0x2a);
  SP0A38_write_cmos_sensor(0xca,0x28);
  SP0A38_write_cmos_sensor(0xf3,0x28);
  SP0A38_write_cmos_sensor(0xf4,0x28);

  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0x04,0x80);

  SP0A38_write_cmos_sensor(0x05,0x28); //min_rpc_indoor 24
  SP0A38_write_cmos_sensor(0x0a,0x80); //max_rpc_indoor  0x80

  SP0A38_write_cmos_sensor(0x0b,0x28);

  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0xcb,0x90);
  SP0A38_write_cmos_sensor(0xcc,0x88);
  SP0A38_write_cmos_sensor(0xcd,0x07);
  SP0A38_write_cmos_sensor(0xce,0x0a);

  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0x1a,0x80);
  SP0A38_write_cmos_sensor(0x1b,0x4f);
  SP0A38_write_cmos_sensor(0x1c,0x00);
  SP0A38_write_cmos_sensor(0x1d,0x20);
  SP0A38_write_cmos_sensor(0x1e,0x00);
  SP0A38_write_cmos_sensor(0x1f,0x03);
  SP0A38_write_cmos_sensor(0x20,0x00);
  SP0A38_write_cmos_sensor(0x21,0x00);
  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0x84,0x2c);
  SP0A38_write_cmos_sensor(0x85,0x28);
  SP0A38_write_cmos_sensor(0x86,0x2e);
  SP0A38_write_cmos_sensor(0x87,0x1e);
  SP0A38_write_cmos_sensor(0x88,0x24);
  SP0A38_write_cmos_sensor(0x89,0x28);
  SP0A38_write_cmos_sensor(0x8a,0x2e);
  SP0A38_write_cmos_sensor(0x8b,0x1e);
  SP0A38_write_cmos_sensor(0x8c,0x23);
  SP0A38_write_cmos_sensor(0x8d,0x24);
  SP0A38_write_cmos_sensor(0x8e,0x2e);
  SP0A38_write_cmos_sensor(0x8f,0x1e);
  SP0A38_write_cmos_sensor(0x90,0x0d);
  SP0A38_write_cmos_sensor(0x91,0x0a);
  SP0A38_write_cmos_sensor(0x92,0x08);
  SP0A38_write_cmos_sensor(0x93,0x0c);
  SP0A38_write_cmos_sensor(0x94,0x0b);
  SP0A38_write_cmos_sensor(0x95,0x02);
  SP0A38_write_cmos_sensor(0x96,0x08);
  SP0A38_write_cmos_sensor(0x97,0x08);
  SP0A38_write_cmos_sensor(0x98,0x02);
  SP0A38_write_cmos_sensor(0x99,0x04);
  SP0A38_write_cmos_sensor(0x9a,0x08);
  SP0A38_write_cmos_sensor(0x9b,0x04);
  SP0A38_write_cmos_sensor(0xfd,0x02);
  SP0A38_write_cmos_sensor(0x09,0x09);
  SP0A38_write_cmos_sensor(0x0d,0x1a);
  SP0A38_write_cmos_sensor(0x1d,0x03);
  SP0A38_write_cmos_sensor(0x1f,0x07);
  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0x32,0x00);

  SP0A38_write_cmos_sensor(0xfd,0x02);
  SP0A38_write_cmos_sensor(0x26,0xb7);
  SP0A38_write_cmos_sensor(0x27,0x9c);
  SP0A38_write_cmos_sensor(0x10,0x00);
  SP0A38_write_cmos_sensor(0x11,0x00);
  SP0A38_write_cmos_sensor(0x1b,0x80);
  SP0A38_write_cmos_sensor(0x1a,0x80);
  SP0A38_write_cmos_sensor(0x18,0x27);
  SP0A38_write_cmos_sensor(0x19,0x26);
  SP0A38_write_cmos_sensor(0x2a,0x01);
  SP0A38_write_cmos_sensor(0x2b,0x10);
  SP0A38_write_cmos_sensor(0x28,0xf8);
  SP0A38_write_cmos_sensor(0x29,0x08);
  //d65
  SP0A38_write_cmos_sensor(0x66,0x52);
  SP0A38_write_cmos_sensor(0x67,0x74);
  SP0A38_write_cmos_sensor(0x68,0xc9);
  SP0A38_write_cmos_sensor(0x69,0xec);
  SP0A38_write_cmos_sensor(0x6a,0xa5);
  //indoor
  SP0A38_write_cmos_sensor(0x7c,0x40);
  SP0A38_write_cmos_sensor(0x7d,0x60);
  SP0A38_write_cmos_sensor(0x7e,0xf7);
  SP0A38_write_cmos_sensor(0x7f,0x16);
  SP0A38_write_cmos_sensor(0x80,0xa6);
  //cwf
  SP0A38_write_cmos_sensor(0x70,0x3a);
  SP0A38_write_cmos_sensor(0x71,0x57);
  SP0A38_write_cmos_sensor(0x72,0x19);
  SP0A38_write_cmos_sensor(0x73,0x37);
  SP0A38_write_cmos_sensor(0x74,0xaa);
  //tl84
  SP0A38_write_cmos_sensor(0x6b,0x17);
  SP0A38_write_cmos_sensor(0x6c,0x3a);
  SP0A38_write_cmos_sensor(0x6d,0x21);
  SP0A38_write_cmos_sensor(0x6e,0x43);
  SP0A38_write_cmos_sensor(0x6f,0xaa);
  //a
  SP0A38_write_cmos_sensor(0x61,0xfc);
  SP0A38_write_cmos_sensor(0x62,0x17);
  SP0A38_write_cmos_sensor(0x63,0x3f);
  SP0A38_write_cmos_sensor(0x64,0x57);
  SP0A38_write_cmos_sensor(0x65,0x6a);

  SP0A38_write_cmos_sensor(0x75,0x80);
  SP0A38_write_cmos_sensor(0x76,0x09);
  SP0A38_write_cmos_sensor(0x77,0x02);
  SP0A38_write_cmos_sensor(0x0e,0x12);
  SP0A38_write_cmos_sensor(0x3b,0x09);

  SP0A38_write_cmos_sensor(0xfd,0x02);
  SP0A38_write_cmos_sensor(0xde,0x0f);
  SP0A38_write_cmos_sensor(0xd7,0x08);
  SP0A38_write_cmos_sensor(0xd8,0x08);
  SP0A38_write_cmos_sensor(0xd9,0x10);
  SP0A38_write_cmos_sensor(0xda,0x14);

  SP0A38_write_cmos_sensor(0xe8,0x60);
  SP0A38_write_cmos_sensor(0xe9,0x5c);
  SP0A38_write_cmos_sensor(0xea,0x24);
  SP0A38_write_cmos_sensor(0xeb,0x10);

  SP0A38_write_cmos_sensor(0xec,0x64);
  SP0A38_write_cmos_sensor(0xed,0x50);
  SP0A38_write_cmos_sensor(0xee,0x28);
  SP0A38_write_cmos_sensor(0xef,0x10);

  SP0A38_write_cmos_sensor(0xd3,0x20);
  SP0A38_write_cmos_sensor(0xd4,0x48);
  SP0A38_write_cmos_sensor(0xd5,0x20);
  SP0A38_write_cmos_sensor(0xd6,0x08);
  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0xb1,0x20);
  SP0A38_write_cmos_sensor(0xfd,0x02);
  SP0A38_write_cmos_sensor(0xdc,0x05);
  SP0A38_write_cmos_sensor(0x05,0x20);
  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0x62,0xf0);
  SP0A38_write_cmos_sensor(0x63,0x80);
  SP0A38_write_cmos_sensor(0x64,0x80);
  SP0A38_write_cmos_sensor(0x65,0x20);

  SP0A38_write_cmos_sensor(0xfd,0x02);
  SP0A38_write_cmos_sensor(0xdd,0x0f);

  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0xa8,0x08);
  SP0A38_write_cmos_sensor(0xa9,0x10);
  SP0A38_write_cmos_sensor(0xaa,0x12);
  SP0A38_write_cmos_sensor(0xab,0x15);
  SP0A38_write_cmos_sensor(0xd3,0x04);
  SP0A38_write_cmos_sensor(0xd4,0x07);
  SP0A38_write_cmos_sensor(0xd5,0x0e);
  SP0A38_write_cmos_sensor(0xd6,0x15);
  SP0A38_write_cmos_sensor(0xcf,0xe0);
  SP0A38_write_cmos_sensor(0xd0,0x80);
  SP0A38_write_cmos_sensor(0xd1,0x40);
  SP0A38_write_cmos_sensor(0xd2,0x20);
  SP0A38_write_cmos_sensor(0xdf,0xe0);
  SP0A38_write_cmos_sensor(0xe0,0x80);
  SP0A38_write_cmos_sensor(0xe1,0x30);
  SP0A38_write_cmos_sensor(0xe2,0x20);
  SP0A38_write_cmos_sensor(0xe3,0xe0);
  SP0A38_write_cmos_sensor(0xe4,0x80);
  SP0A38_write_cmos_sensor(0xe5,0x36);
  SP0A38_write_cmos_sensor(0xe6,0x20);

  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0x6e,0x00);
  SP0A38_write_cmos_sensor(0x6f,0x0d);
  SP0A38_write_cmos_sensor(0x70,0x19);
  SP0A38_write_cmos_sensor(0x71,0x27);
  SP0A38_write_cmos_sensor(0x72,0x34);
  SP0A38_write_cmos_sensor(0x73,0x48);
  SP0A38_write_cmos_sensor(0x74,0x5a);
  SP0A38_write_cmos_sensor(0x75,0x69);
  SP0A38_write_cmos_sensor(0x76,0x75);
  SP0A38_write_cmos_sensor(0x77,0x89);
  SP0A38_write_cmos_sensor(0x78,0x97);
  SP0A38_write_cmos_sensor(0x79,0xa3);
  SP0A38_write_cmos_sensor(0x7a,0xb0);
  SP0A38_write_cmos_sensor(0x7b,0xba);
  SP0A38_write_cmos_sensor(0x7c,0xc4);
  SP0A38_write_cmos_sensor(0x7d,0xcd);
  SP0A38_write_cmos_sensor(0x7e,0xd4);
  SP0A38_write_cmos_sensor(0x7f,0xdd);
  SP0A38_write_cmos_sensor(0x80,0xe6);
  SP0A38_write_cmos_sensor(0x81,0xef);
  SP0A38_write_cmos_sensor(0x82,0xf7);
  SP0A38_write_cmos_sensor(0x83,0xff);

  SP0A38_write_cmos_sensor(0xfd,0x02);
  SP0A38_write_cmos_sensor(0x15,0xc6);
  SP0A38_write_cmos_sensor(0x16,0x92);

  SP0A38_write_cmos_sensor(0xa0,0x86);
  SP0A38_write_cmos_sensor(0xa1,0xd9);
  SP0A38_write_cmos_sensor(0xa2,0x20);
  SP0A38_write_cmos_sensor(0xa3,0xe8);
  SP0A38_write_cmos_sensor(0xa4,0x99);
  SP0A38_write_cmos_sensor(0xa5,0xfd);
  SP0A38_write_cmos_sensor(0xa6,0xe6);
  SP0A38_write_cmos_sensor(0xa7,0xd9);
  SP0A38_write_cmos_sensor(0xa8,0xc0);

  SP0A38_write_cmos_sensor(0xac,0x66);
  SP0A38_write_cmos_sensor(0xad,0x59);
  SP0A38_write_cmos_sensor(0xae,0xc0);
  SP0A38_write_cmos_sensor(0xaf,0xf9);
  SP0A38_write_cmos_sensor(0xb0,0x80);
  SP0A38_write_cmos_sensor(0xb1,0x06);
  SP0A38_write_cmos_sensor(0xb2,0xec);
  SP0A38_write_cmos_sensor(0xb3,0xc6);
  SP0A38_write_cmos_sensor(0xb4,0xcc);

  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0xb3,0xa8);
  SP0A38_write_cmos_sensor(0xb4,0xa8);
  SP0A38_write_cmos_sensor(0xb5,0x75);
  SP0A38_write_cmos_sensor(0xb6,0x70);//0x60
  SP0A38_write_cmos_sensor(0xb7,0xa8);
  SP0A38_write_cmos_sensor(0xb8,0xa8);
  SP0A38_write_cmos_sensor(0xb9,0x75);
  SP0A38_write_cmos_sensor(0xba,0x70);//0x60

  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0xbd,0x20);
  SP0A38_write_cmos_sensor(0xbe,0x10);
  SP0A38_write_cmos_sensor(0xbf,0xff);
  SP0A38_write_cmos_sensor(0x00,0x00);
  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0x9c,0xaa);
  SP0A38_write_cmos_sensor(0x9d,0xaa);
  SP0A38_write_cmos_sensor(0x9e,0x77);
  SP0A38_write_cmos_sensor(0x9f,0x77);
  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0xa4,0x10);
  SP0A38_write_cmos_sensor(0xa5,0x1f);
  SP0A38_write_cmos_sensor(0xa6,0x30);
  SP0A38_write_cmos_sensor(0xa7,0x45);
  SP0A38_write_cmos_sensor(0xfd,0x02);
  //SP0A38_write_cmos_sensor(0x31,0x60);
  SP0A38_write_cmos_sensor(0x32,0x60);
  SP0A38_write_cmos_sensor(0x33,0xff);
  SP0A38_write_cmos_sensor(0x35,0x60);
  SP0A38_write_cmos_sensor(0x36,0x28);
  SP0A38_write_cmos_sensor(0x37,0x13);
  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0x0e,0x80);
  SP0A38_write_cmos_sensor(0x0f,0x20);

  SP0A38_write_cmos_sensor(0x10,SP0A38_SET_REG0x10);
  SP0A38_write_cmos_sensor(0x11,SP0A38_SET_REG0x11);
  SP0A38_write_cmos_sensor(0x12,SP0A38_SET_REG0x12);
  SP0A38_write_cmos_sensor(0x13,SP0A38_SET_REG0x13);

  SP0A38_write_cmos_sensor(0x14,SP0A38_SET_REG0x14);
  SP0A38_write_cmos_sensor(0x15,SP0A38_SET_REG0x15);
  SP0A38_write_cmos_sensor(0x16,SP0A38_SET_REG0x16);
  SP0A38_write_cmos_sensor(0x17,SP0A38_SET_REG0x17);

  //SP0A38_write_cmos_sensor(0xfd, 0x02);
  //SP0A38_write_cmos_sensor(0x75, 0x91);
  //SP0A38_write_cmos_sensor(0x2a, 0x00);
  //SP0A38_write_cmos_sensor(0x3b, 0x00);
  //SP0A38_write_cmos_sensor(0x1d, 0x7f);
  //SP0A38_write_cmos_sensor(0x14, 0x02);

  //SP0A38_write_cmos_sensor(0x7c, 0x00);
  //SP0A38_write_cmos_sensor(0x7d, 0xff);
  //SP0A38_write_cmos_sensor(0x7e, 0x00);
  //SP0A38_write_cmos_sensor(0x7f, 0xff);
  //SP0A38_write_cmos_sensor(0x80, 0x66);

  SP0A38_write_cmos_sensor(0xfd,0x02);
  SP0A38_write_cmos_sensor(0x48,0xf8);
  SP0A38_write_cmos_sensor(0x49,0xf4);
  SP0A38_write_cmos_sensor(0x4a,0x0f);
  SP0A38_write_cmos_sensor(0xfd,0x01);
  SP0A38_write_cmos_sensor(0x32,0x15);
  SP0A38_write_cmos_sensor(0x33,0xef);
  SP0A38_write_cmos_sensor(0x34,0xef);
  SP0A38_write_cmos_sensor(0xfb,0x33);
  SP0A38_write_cmos_sensor(0xf2,0x6c);
  SP0A38_write_cmos_sensor(0x35,0x00);
  SP0A38_write_cmos_sensor(0x5d,0x01);
  SP0A38_write_cmos_sensor(0xfd,0x00);
  SP0A38_write_cmos_sensor(0xfd,0x00);
  SP0A38_write_cmos_sensor(0xe7,0x00);
  SP0A38_write_cmos_sensor(0xfd,0x00);

  SP0A38_write_cmos_sensor(0x31,0x00);

  SP0A38_write_cmos_sensor(0x30,0x09);  //0x1a //dclk = 1/2pclk
  SP0A38_write_cmos_sensor(0x1e,0x95);
  SP0A38_write_cmos_sensor(0x2c,0x1d);
  SP0A38_write_cmos_sensor(0x2d,0x40);
  SP0A38_write_cmos_sensor(0x2e,0xf9);
}

/*************************************************************************
 * FUNCTION
 *	SP0A38Open
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
UINT32 sp0a38_read_id(void)
{
  volatile signed char i;
  UINT16 sensor_id = 0;
  for(i = 0;i < 3;i++)
  {
    SP0A38_write_cmos_sensor(0xfd,0x00);
    sensor_id = ((SP0A38_read_cmos_sensor(0x01) << 8) | SP0A38_read_cmos_sensor(0x02));
    if(sensor_id == SP0A38_YUV_ID)
      break;
  }
  YUV_DBG("sp0a38 read yuv sensor_id=0x%x\n",sensor_id);
  return ((sensor_id == SP0A38_YUV_ID)?1:0);
}

void sp0a38_open(void)
{
  YUV_DBG("%s\n",__func__);
  SP0A38_Sensor_Init();
}

YUV_SENSOR_FUNC sp0a38_sensor_func = {

  sp0a38_open,
  SP0A38_MIPI_GetYUVSensorBV,

};

UINT32 SP0A38_YUV_SENSOR_INIT(PYUV_SENSOR_FUNC *pfFunc)
{
  if(NULL != pfFunc)
    *pfFunc = &sp0a38_sensor_func;
  YUV_DBG("%s\n",__func__);
  return sp0a38_read_id();
}
