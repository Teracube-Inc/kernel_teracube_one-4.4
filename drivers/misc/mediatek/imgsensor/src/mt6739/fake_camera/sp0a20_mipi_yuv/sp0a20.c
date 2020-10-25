#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>

#include "kd_camera_typedef.h"
//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "sp0a20.h"
#include "fake_camera_define.h"

static UINT8 SP0A20_read_cmos_sensor(UINT8 addr)
{
  UINT8 in_buff[1] = {0xFF};
  UINT8 out_buff[1];

  out_buff[0] = addr;

  if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), SP0A20_WRITE_ID)) 
  {
    YUV_DBG("ERROR: SP0A20_read_cmos_sensor \n");
  }
  if(in_buff[0] == 0)  //for ata yuv sensor shutter
  {
    in_buff[0] = 1;
  }
  return in_buff[0];
}

static void SP0A20_write_cmos_sensor(UINT8 addr, UINT8 para)
{
  UINT8 out_buff[2];

  out_buff[0] = addr;
  out_buff[1] = para;

  iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), SP0A20_WRITE_ID); 
}

UINT16 SP0A20_MIPI_GetYUVSensorBV(void)
{
  UINT16 light;
  SP0A20_write_cmos_sensor(0xfd,0x01);
  light = SP0A20_read_cmos_sensor(0xf1);
  SP0A20_write_cmos_sensor(0xfd,0x00);
  YUV_DBG("SP0A20_MIPI_GetYUVSensorBV val:%d\n",light);
  return light;
}

static void SP0A20_Sensor_Driver_Init(void)
{
  SP0A20_write_cmos_sensor(0xfd,0x00);
  //SP0A20_write_cmos_sensor(0x36,0x02);
  SP0A20_write_cmos_sensor(0x0c,0x00);
  SP0A20_write_cmos_sensor(0x12,0x02);
  SP0A20_write_cmos_sensor(0x13,0x2f);
  SP0A20_write_cmos_sensor(0x6d,0x32);
  SP0A20_write_cmos_sensor(0x6c,0x32);
  SP0A20_write_cmos_sensor(0x6f,0x33);
  SP0A20_write_cmos_sensor(0x6e,0x34);
  SP0A20_write_cmos_sensor(0xfd,0x00);
  SP0A20_write_cmos_sensor(0x92,0x11);
  SP0A20_write_cmos_sensor(0x99,0x05);
  SP0A20_write_cmos_sensor(0x16,0x38);
  SP0A20_write_cmos_sensor(0x17,0x38);
  SP0A20_write_cmos_sensor(0x70,0x3a);
  SP0A20_write_cmos_sensor(0x14,0x02);
  SP0A20_write_cmos_sensor(0x15,0x20);
  SP0A20_write_cmos_sensor(0x71,0x23);
  SP0A20_write_cmos_sensor(0x69,0x25);
  SP0A20_write_cmos_sensor(0x6a,0x1a);
  SP0A20_write_cmos_sensor(0x72,0x1c);
  SP0A20_write_cmos_sensor(0x75,0x1e);
  SP0A20_write_cmos_sensor(0x73,0x3c);
  SP0A20_write_cmos_sensor(0x74,0x21);
  SP0A20_write_cmos_sensor(0x79,0x00);
  SP0A20_write_cmos_sensor(0x77,0x10);
  SP0A20_write_cmos_sensor(0x1a,0x4d);
  SP0A20_write_cmos_sensor(0x1b,0x27);
  SP0A20_write_cmos_sensor(0x1c,0x07);
  SP0A20_write_cmos_sensor(0x1e,0x15);
  SP0A20_write_cmos_sensor(0x21,0x08);
  SP0A20_write_cmos_sensor(0x22,0x28);
  SP0A20_write_cmos_sensor(0x26,0x66);
  SP0A20_write_cmos_sensor(0x28,0x0b);
  SP0A20_write_cmos_sensor(0x37,0x5a);  //4a
  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0x01,0x80);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0x41,0x00);
  SP0A20_write_cmos_sensor(0x42,0x00);
  SP0A20_write_cmos_sensor(0x43,0x00);
  SP0A20_write_cmos_sensor(0x44,0x00);

  ///capture 50Hz 24M 7-12fps          
  SP0A20_write_cmos_sensor(0xfd,0x00); 
  SP0A20_write_cmos_sensor(0x03,0x01); 
  SP0A20_write_cmos_sensor(0x04,0xc2);
  SP0A20_write_cmos_sensor(0x05,0x00); 
  SP0A20_write_cmos_sensor(0x06,0x00); 
  SP0A20_write_cmos_sensor(0x07,0x00); 
  SP0A20_write_cmos_sensor(0x08,0x00); 
  SP0A20_write_cmos_sensor(0x09,0x02);
  SP0A20_write_cmos_sensor(0x0a,0xf4);
  SP0A20_write_cmos_sensor(0xfd,0x01); 
  SP0A20_write_cmos_sensor(0xf0,0x00); 
  SP0A20_write_cmos_sensor(0xf7,0x4b);
  SP0A20_write_cmos_sensor(0x02,0x0e); 
  SP0A20_write_cmos_sensor(0x03,0x01); 
  SP0A20_write_cmos_sensor(0x06,0x4b);
  SP0A20_write_cmos_sensor(0x07,0x00); 
  SP0A20_write_cmos_sensor(0x08,0x01); 
  SP0A20_write_cmos_sensor(0x09,0x00); 
  SP0A20_write_cmos_sensor(0xfd,0x02); 
  SP0A20_write_cmos_sensor(0xbe,0x1a);
  SP0A20_write_cmos_sensor(0xbf,0x04);
  SP0A20_write_cmos_sensor(0xd0,0x1a);
  SP0A20_write_cmos_sensor(0xd1,0x04);


  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0x5a,0x40);
  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0xbc,0x70);
  SP0A20_write_cmos_sensor(0xbd,0x50);
  SP0A20_write_cmos_sensor(0xb8,0x66);
  SP0A20_write_cmos_sensor(0xb9,0x8f);
  SP0A20_write_cmos_sensor(0xba,0x30);
  SP0A20_write_cmos_sensor(0xbb,0x45);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xe0,0x60);
  SP0A20_write_cmos_sensor(0xe1,0x48);
  SP0A20_write_cmos_sensor(0xe2,0x40);
  SP0A20_write_cmos_sensor(0xe3,0x3a);
  SP0A20_write_cmos_sensor(0xe4,0x3a);
  SP0A20_write_cmos_sensor(0xe5,0x38);
  SP0A20_write_cmos_sensor(0xe6,0x38);
  SP0A20_write_cmos_sensor(0xe7,0x34);
  SP0A20_write_cmos_sensor(0xe8,0x34);
  SP0A20_write_cmos_sensor(0xe9,0x34);
  SP0A20_write_cmos_sensor(0xea,0x32);
  SP0A20_write_cmos_sensor(0xf3,0x32);
  SP0A20_write_cmos_sensor(0xf4,0x32);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0x04,0xa0);
  SP0A20_write_cmos_sensor(0x05,0x32);
  SP0A20_write_cmos_sensor(0x0a,0xa0);
  SP0A20_write_cmos_sensor(0x0b,0x32);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xeb,0x7f);
  SP0A20_write_cmos_sensor(0xec,0x7f);
  SP0A20_write_cmos_sensor(0xed,0x05);
  SP0A20_write_cmos_sensor(0xee,0x0a);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xf2,0x4d);
  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0x54,0x0a);
  SP0A20_write_cmos_sensor(0x5b,0x05);
  SP0A20_write_cmos_sensor(0x5c,0xa0);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0x26,0x80);
  SP0A20_write_cmos_sensor(0x27,0x4f);
  SP0A20_write_cmos_sensor(0x28,0x00);
  SP0A20_write_cmos_sensor(0x29,0x20);
  SP0A20_write_cmos_sensor(0x2a,0x00);
  SP0A20_write_cmos_sensor(0x2b,0x03);
  SP0A20_write_cmos_sensor(0x2c,0x00);
  SP0A20_write_cmos_sensor(0x2d,0x20);
  SP0A20_write_cmos_sensor(0x30,0x00);
  SP0A20_write_cmos_sensor(0x31,0x00);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xa1,0x3B);
  SP0A20_write_cmos_sensor(0xa2,0x3B);
  SP0A20_write_cmos_sensor(0xa3,0x3B);
  SP0A20_write_cmos_sensor(0xa4,0x3B);
  SP0A20_write_cmos_sensor(0xa5,0x3B);
  SP0A20_write_cmos_sensor(0xa6,0x3B);
  SP0A20_write_cmos_sensor(0xa7,0x3B);
  SP0A20_write_cmos_sensor(0xa8,0x3B);
  SP0A20_write_cmos_sensor(0xa9,0x3B);
  SP0A20_write_cmos_sensor(0xaa,0x3B);
  SP0A20_write_cmos_sensor(0xab,0x3B);
  SP0A20_write_cmos_sensor(0xac,0x3B);
  SP0A20_write_cmos_sensor(0xad,0x0e);
  SP0A20_write_cmos_sensor(0xae,0x08);
  SP0A20_write_cmos_sensor(0xaf,0x0a);
  SP0A20_write_cmos_sensor(0xb0,0x08);
  SP0A20_write_cmos_sensor(0xb1,0x06);
  SP0A20_write_cmos_sensor(0xb2,0x01);
  SP0A20_write_cmos_sensor(0xb3,0x04);
  SP0A20_write_cmos_sensor(0xb4,0x01);
  SP0A20_write_cmos_sensor(0xb5,0x07);
  SP0A20_write_cmos_sensor(0xb6,0x00);
  SP0A20_write_cmos_sensor(0xb7,0x00);
  SP0A20_write_cmos_sensor(0xb8,0x00);
  SP0A20_write_cmos_sensor(0xfd,0x00);

  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0x09,0x09);
  SP0A20_write_cmos_sensor(0x0d,0x1a);
  SP0A20_write_cmos_sensor(0x1d,0x03);
  SP0A20_write_cmos_sensor(0x1f,0x04);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0x32,0x00);
  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0x26,0xbf);
  SP0A20_write_cmos_sensor(0x27,0xa3);
  SP0A20_write_cmos_sensor(0x10,0x00);
  SP0A20_write_cmos_sensor(0x11,0x00);
  SP0A20_write_cmos_sensor(0x1b,0x80);
  SP0A20_write_cmos_sensor(0x1a,0x80);
  SP0A20_write_cmos_sensor(0x18,0x27);
  SP0A20_write_cmos_sensor(0x19,0x26);
  SP0A20_write_cmos_sensor(0x2a,0x01);
  SP0A20_write_cmos_sensor(0x2b,0x10);
  SP0A20_write_cmos_sensor(0x28,0xf8);
  SP0A20_write_cmos_sensor(0x29,0x08);

  SP0A20_write_cmos_sensor(0x66,0x4b);
  SP0A20_write_cmos_sensor(0x67,0x6b);
  SP0A20_write_cmos_sensor(0x68,0xcd);
  SP0A20_write_cmos_sensor(0x69,0xe7);
  SP0A20_write_cmos_sensor(0x6a,0xa5);

  SP0A20_write_cmos_sensor(0x7c,0x38);
  SP0A20_write_cmos_sensor(0x7d,0x60);
  SP0A20_write_cmos_sensor(0x7e,0xf0);
  SP0A20_write_cmos_sensor(0x7f,0x10);
  SP0A20_write_cmos_sensor(0x80,0xa6);

  SP0A20_write_cmos_sensor(0x70,0x2d);
  SP0A20_write_cmos_sensor(0x71,0x4f);
  SP0A20_write_cmos_sensor(0x72,0x1d);
  SP0A20_write_cmos_sensor(0x73,0x37);
  SP0A20_write_cmos_sensor(0x74,0xaa);

  SP0A20_write_cmos_sensor(0x6b,0x11);
  SP0A20_write_cmos_sensor(0x6c,0x2d);
  SP0A20_write_cmos_sensor(0x6d,0x22);
  SP0A20_write_cmos_sensor(0x6e,0x43);
  SP0A20_write_cmos_sensor(0x6f,0xaa);

  SP0A20_write_cmos_sensor(0x61,0xef);
  SP0A20_write_cmos_sensor(0x62,0x0e);
  SP0A20_write_cmos_sensor(0x63,0x48);
  SP0A20_write_cmos_sensor(0x64,0x6c);
  SP0A20_write_cmos_sensor(0x65,0x6a);

  SP0A20_write_cmos_sensor(0x75,0x80);
  SP0A20_write_cmos_sensor(0x76,0x09);
  SP0A20_write_cmos_sensor(0x77,0x02);
  SP0A20_write_cmos_sensor(0x24,0x25);
  SP0A20_write_cmos_sensor(0x0e,0x16);
  SP0A20_write_cmos_sensor(0x3b,0x09);
  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0x08,0x00);
  SP0A20_write_cmos_sensor(0x09,0x06);

  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0xde,0x0f);
  SP0A20_write_cmos_sensor(0xd7,0x0a);
  SP0A20_write_cmos_sensor(0xd8,0x0c);
  SP0A20_write_cmos_sensor(0xd9,0x12);
  SP0A20_write_cmos_sensor(0xda,0x18);
  SP0A20_write_cmos_sensor(0xe8,0x20);
  SP0A20_write_cmos_sensor(0xe9,0x20);
  SP0A20_write_cmos_sensor(0xea,0x20);
  SP0A20_write_cmos_sensor(0xeb,0x20);
  SP0A20_write_cmos_sensor(0xec,0x20);
  SP0A20_write_cmos_sensor(0xed,0x20);
  SP0A20_write_cmos_sensor(0xee,0x20);
  SP0A20_write_cmos_sensor(0xef,0x20);
  SP0A20_write_cmos_sensor(0xd3,0x20);
  SP0A20_write_cmos_sensor(0xd4,0x48);
  SP0A20_write_cmos_sensor(0xd5,0x20);
  SP0A20_write_cmos_sensor(0xd6,0x08);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xd1,0x20);
  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0xdc,0x05);
  SP0A20_write_cmos_sensor(0x05,0x20);

  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0x81,0x00);
  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xfc,0x00);
  SP0A20_write_cmos_sensor(0x7d,0x05);
  SP0A20_write_cmos_sensor(0x7e,0x05);
  SP0A20_write_cmos_sensor(0x7f,0x09);
  SP0A20_write_cmos_sensor(0x80,0x08);

  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0xdd,0x0f);
  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0x86,0x20);

  SP0A20_write_cmos_sensor(0x6d,0x12);
  SP0A20_write_cmos_sensor(0x6e,0x1a);
  SP0A20_write_cmos_sensor(0x6f,0x1e);
  SP0A20_write_cmos_sensor(0x70,0x24);
  SP0A20_write_cmos_sensor(0x86,0x20);
  SP0A20_write_cmos_sensor(0x71,0x12);
  SP0A20_write_cmos_sensor(0x72,0x1a);
  SP0A20_write_cmos_sensor(0x73,0x1e);
  SP0A20_write_cmos_sensor(0x74,0x24);

  SP0A20_write_cmos_sensor(0x75,0x08);
  SP0A20_write_cmos_sensor(0x76,0x0b);
  SP0A20_write_cmos_sensor(0x77,0x0e);
  SP0A20_write_cmos_sensor(0x78,0x12);
  SP0A20_write_cmos_sensor(0x79,0x25);
  SP0A20_write_cmos_sensor(0x7a,0x23);
  SP0A20_write_cmos_sensor(0x7b,0x22);
  SP0A20_write_cmos_sensor(0x7c,0x00);

  SP0A20_write_cmos_sensor(0x81,0x0d);
  SP0A20_write_cmos_sensor(0x82,0x18);
  SP0A20_write_cmos_sensor(0x83,0x28);
  SP0A20_write_cmos_sensor(0x84,0x30);

  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0x83,0x14);
  SP0A20_write_cmos_sensor(0x84,0x18);
  SP0A20_write_cmos_sensor(0x86,0x04);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0x61,0x60);
  SP0A20_write_cmos_sensor(0x62,0x28);
  SP0A20_write_cmos_sensor(0x8a,0x10);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0x8b,0x00);
  SP0A20_write_cmos_sensor(0x8c,0x09);
  SP0A20_write_cmos_sensor(0x8d,0x13);
  SP0A20_write_cmos_sensor(0x8e,0x1b);
  SP0A20_write_cmos_sensor(0x8f,0x23);
  SP0A20_write_cmos_sensor(0x90,0x32);
  SP0A20_write_cmos_sensor(0x91,0x41);
  SP0A20_write_cmos_sensor(0x92,0x4d);
  SP0A20_write_cmos_sensor(0x93,0x5c);
  SP0A20_write_cmos_sensor(0x94,0x70);
  SP0A20_write_cmos_sensor(0x95,0x84);
  SP0A20_write_cmos_sensor(0x96,0x94);
  SP0A20_write_cmos_sensor(0x97,0xa3);
  SP0A20_write_cmos_sensor(0x98,0xb5);
  SP0A20_write_cmos_sensor(0x99,0xc2);
  SP0A20_write_cmos_sensor(0x9a,0xcf);
  SP0A20_write_cmos_sensor(0x9b,0xd9);
  SP0A20_write_cmos_sensor(0x9c,0xe3);
  SP0A20_write_cmos_sensor(0x9d,0xec);
  SP0A20_write_cmos_sensor(0x9e,0xf4);
  SP0A20_write_cmos_sensor(0x9f,0xfa);
  SP0A20_write_cmos_sensor(0xa0,0xff);

  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0x15,0xca);
  SP0A20_write_cmos_sensor(0x16,0x8a);

  SP0A20_write_cmos_sensor(0xa0,0x8c);
  SP0A20_write_cmos_sensor(0xa1,0xfa);
  SP0A20_write_cmos_sensor(0xa2,0xfa);
  SP0A20_write_cmos_sensor(0xa3,0xf4);
  SP0A20_write_cmos_sensor(0xa4,0x99);
  SP0A20_write_cmos_sensor(0xa5,0xf4);
  SP0A20_write_cmos_sensor(0xa6,0x06);
  SP0A20_write_cmos_sensor(0xa7,0xc7);
  SP0A20_write_cmos_sensor(0xa8,0xb3);
  SP0A20_write_cmos_sensor(0xa9,0x3c);
  SP0A20_write_cmos_sensor(0xaa,0x33);
  SP0A20_write_cmos_sensor(0xab,0x0c);

  SP0A20_write_cmos_sensor(0xac,0x8c);
  SP0A20_write_cmos_sensor(0xad,0x0c);
  SP0A20_write_cmos_sensor(0xae,0xe7);
  SP0A20_write_cmos_sensor(0xaf,0xf1);
  SP0A20_write_cmos_sensor(0xb0,0xa8);
  SP0A20_write_cmos_sensor(0xb1,0xe7);
  SP0A20_write_cmos_sensor(0xb2,0xe7);
  SP0A20_write_cmos_sensor(0xb3,0xcd);
  SP0A20_write_cmos_sensor(0xb4,0xcc);
  SP0A20_write_cmos_sensor(0xb5,0x30);
  SP0A20_write_cmos_sensor(0xb6,0x33);
  SP0A20_write_cmos_sensor(0xb7,0x0f);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xd3,0x7A);
  SP0A20_write_cmos_sensor(0xd4,0x7A);
  SP0A20_write_cmos_sensor(0xd5,0x60);
  SP0A20_write_cmos_sensor(0xd6,0x58);

  SP0A20_write_cmos_sensor(0xd7,0x7A);
  SP0A20_write_cmos_sensor(0xd8,0x7A);
  SP0A20_write_cmos_sensor(0xd9,0x60);
  SP0A20_write_cmos_sensor(0xda,0x58);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xdd,0x30);
  SP0A20_write_cmos_sensor(0xde,0x10);
  SP0A20_write_cmos_sensor(0xdf,0xff);
  SP0A20_write_cmos_sensor(0x00,0x00);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xc2,0xaa);
  SP0A20_write_cmos_sensor(0xc3,0x88);
  SP0A20_write_cmos_sensor(0xc4,0x77);
  SP0A20_write_cmos_sensor(0xc5,0x66);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0xcd,0x10);
  SP0A20_write_cmos_sensor(0xce,0x1f);
  SP0A20_write_cmos_sensor(0xcf,0x30);
  SP0A20_write_cmos_sensor(0xd0,0x45);

  SP0A20_write_cmos_sensor(0xfd,0x02);
  SP0A20_write_cmos_sensor(0x31,0x60);
  SP0A20_write_cmos_sensor(0x32,0x60);
  SP0A20_write_cmos_sensor(0x33,0xc0);
  SP0A20_write_cmos_sensor(0x35,0x60);
  SP0A20_write_cmos_sensor(0x36,0x28);
  SP0A20_write_cmos_sensor(0x37,0x13);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0x0e,0x80);
  SP0A20_write_cmos_sensor(0x0f,0x20);
  SP0A20_write_cmos_sensor(0x10,0x80);
  SP0A20_write_cmos_sensor(0x11,0x80);
  SP0A20_write_cmos_sensor(0x12,0x80);
  SP0A20_write_cmos_sensor(0x13,0x80);
  SP0A20_write_cmos_sensor(0x14,0x90);
  SP0A20_write_cmos_sensor(0x15,0x90);
  SP0A20_write_cmos_sensor(0x16,0x90);
  SP0A20_write_cmos_sensor(0x17,0x90);

  SP0A20_write_cmos_sensor(0xfd,0x00);
  SP0A20_write_cmos_sensor(0x31,0x00);

  SP0A20_write_cmos_sensor(0xfd,0x01);
  SP0A20_write_cmos_sensor(0x32,0x15);
  SP0A20_write_cmos_sensor(0x33,0xef);
  SP0A20_write_cmos_sensor(0x34,0x07);
  SP0A20_write_cmos_sensor(0xd2,0x01);
  SP0A20_write_cmos_sensor(0xfb,0x25);
  SP0A20_write_cmos_sensor(0xf2,0x49);
  SP0A20_write_cmos_sensor(0x35,0x00);
  SP0A20_write_cmos_sensor(0x5d,0x11);

}

UINT32 sp0a20_read_id(void)
{
  int retry = 10; 
  UINT16 sensor_id = 0;
  do {

    SP0A20_write_cmos_sensor(0xfd,0x00);
    sensor_id = SP0A20_read_cmos_sensor(0x02);
    if (sensor_id == SP0A20_YUV_ID) {
      break; 
    }
    YUV_DBG("Read Sensor ID Fail = 0x%x\n", sensor_id); 

    retry--; 
  }while (retry > 0); 

  YUV_DBG("sp0a20_id= 0x%x\n", sensor_id); 
  return ((sensor_id == SP0A20_YUV_ID)?1:0);
}

void sp0a20_open(void)
{
  YUV_DBG("%s\n",__func__);
  SP0A20_Sensor_Driver_Init();
}

YUV_SENSOR_FUNC sp0a20_sensor_func = {

  sp0a20_open,
  SP0A20_MIPI_GetYUVSensorBV,

};

UINT32 SP0A20_YUV_SENSOR_INIT(PYUV_SENSOR_FUNC *pfFunc)
{
  if(NULL != pfFunc)
    *pfFunc = &sp0a20_sensor_func;
  YUV_DBG("%s\n",__func__);
  return sp0a20_read_id();
}
