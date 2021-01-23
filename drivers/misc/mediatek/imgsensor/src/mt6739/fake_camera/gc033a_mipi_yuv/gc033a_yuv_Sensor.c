/*****************************************************************************
 *
 * Filename:
 * ---------
 *     GC033Amipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
//#include <linux/xlog.h>

#include "kd_camera_typedef.h"
//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "gc033a_yuv_Sensor.h"
#include "fake_camera_define.h"

#define Sleep(ms) mdelay(ms)

UINT16 read_cmos_sensor(kal_uint8 addr)
{
  UINT16 get_byte = 0;
  char puSendCmd = { (char)(addr & 0xFF) };
  kdSetI2CSpeed(100);
  if(0 != iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, GC033A_READ_ID))
  {
    YUV_DBG("ERROR: gc033aMIPI_read_cmos_sensor \n");
    return 0;
  }
  if(get_byte == 0)  //for ata yuv sensor shutter
  {
    get_byte = 1;
  }
  return get_byte;
}

static void write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
  char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
  kdSetI2CSpeed(100);
  iWriteRegI2C(puSendCmd , 2, GC033A_WRITE_ID);
  //Sleep(10);
  //GC0310_read_cmos_sensor(addr);
}

UINT16 GC033A_MIPI_GetYUVSensorBV(void)
{
  UINT8 temp_reg1, temp_reg2;
  UINT16 shutter,light;

  //GC033A_write_cmos_sensor(0xfe,0x01);
  //light = GC033A_read_cmos_sensor(0x14);

  //GC033A_write_cmos_sensor(0xfe,0x00);
  temp_reg1 = read_cmos_sensor(0x04);
  temp_reg2 = read_cmos_sensor(0x03);

  shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);
  write_cmos_sensor(0xfe, 0x00);
  light = read_cmos_sensor(0xef);
  YUV_DBG("gc033a light = %d shutter = %d\n",light,shutter);

  return light;
}



static void sensor_init(void)
{


	/*SYS*/
	write_cmos_sensor(0xfe, 0x80);
	write_cmos_sensor(0xfe, 0x80);
	write_cmos_sensor(0xfe, 0x80);
	write_cmos_sensor(0xf7, 0x01);
	write_cmos_sensor(0xf8, 0x05);
	write_cmos_sensor(0xf9, 0x0f);
	write_cmos_sensor(0xfa, 0x00);
	write_cmos_sensor(0xfc, 0x0f);
	write_cmos_sensor(0xfe, 0x00);
	
	/*ANALOG & CISCTL*/
	write_cmos_sensor(0x03, 0x01);
	write_cmos_sensor(0x04, 0xc5);
	write_cmos_sensor(0x05, 0x03);
	write_cmos_sensor(0x06, 0x7b);
	write_cmos_sensor(0x07, 0x00);
	write_cmos_sensor(0x08, 0x06);
	write_cmos_sensor(0x0a, 0x00);
	write_cmos_sensor(0x0c, 0x08);
	write_cmos_sensor(0x0d, 0x01);
	write_cmos_sensor(0x0e, 0xe8);
	write_cmos_sensor(0x0f, 0x02);
	write_cmos_sensor(0x10, 0x88);
	write_cmos_sensor(0x17, 0x54);//Don't Change Here!!!
	write_cmos_sensor(0x18, 0x12);
	write_cmos_sensor(0x19, 0x07);
	write_cmos_sensor(0x1a, 0x1b);
	write_cmos_sensor(0x1d, 0x48);//40 travis20160318
	write_cmos_sensor(0x1e, 0x50);
	write_cmos_sensor(0x1f, 0x80);
	write_cmos_sensor(0x23, 0x01);
	write_cmos_sensor(0x24, 0xc8);
    write_cmos_sensor(0x27, 0xaf);
    write_cmos_sensor(0x28, 0x24);
	write_cmos_sensor(0x29, 0x1a);
	write_cmos_sensor(0x2f, 0x14);
	write_cmos_sensor(0x30, 0x00);
	write_cmos_sensor(0x31, 0x04);
	write_cmos_sensor(0x32, 0x08);
	write_cmos_sensor(0x33, 0x0c);
	write_cmos_sensor(0x34, 0x0d);
	write_cmos_sensor(0x35, 0x0e);
    write_cmos_sensor(0x36, 0x0f);
	write_cmos_sensor(0x72, 0x98);
	write_cmos_sensor(0x73, 0x1a);
	write_cmos_sensor(0x74, 0x47);
	write_cmos_sensor(0x76, 0x82);
	write_cmos_sensor(0x7a, 0xcb);
	write_cmos_sensor(0xc2, 0x0c);
	write_cmos_sensor(0xce, 0x03);	
	write_cmos_sensor(0xcf, 0x48);
	write_cmos_sensor(0xd0, 0x10);
	write_cmos_sensor(0xdc, 0x75);
	write_cmos_sensor(0xeb, 0x78);
	
	/*ISP*/
	write_cmos_sensor(0x89, 0x03);
	write_cmos_sensor(0x8d, 0x02);
	write_cmos_sensor(0x90, 0x01);
	write_cmos_sensor(0x92, 0x03);//Don't Change Here!!!
	write_cmos_sensor(0x94, 0x01);//Don't Change Here!!!
	write_cmos_sensor(0x95, 0x01);
	write_cmos_sensor(0x96, 0xe0);
	write_cmos_sensor(0x97, 0x02);
	write_cmos_sensor(0x98, 0x80);
	
	/*Gain*/
	write_cmos_sensor(0xb0, 0x50);
	write_cmos_sensor(0xb1, 0x01);
	write_cmos_sensor(0xb2, 0x00);
	write_cmos_sensor(0xb3, 0x40);
	write_cmos_sensor(0xb4, 0x40);
	write_cmos_sensor(0xb5, 0x40);
	write_cmos_sensor(0xb6, 0x00);
	
	/*BLK*/
	write_cmos_sensor(0x40, 0x26); 
	write_cmos_sensor(0x4e, 0x00);
	write_cmos_sensor(0x4f, 0x3c);
	
	/*Dark Sun*/
	write_cmos_sensor(0xe0, 0x9f);
	write_cmos_sensor(0xe1, 0x90);
	write_cmos_sensor(0xe4, 0x0f);
	write_cmos_sensor(0xe5, 0xff);
		
	/*MIPI*/
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x10, 0x00);	
	write_cmos_sensor(0x01, 0x03);
	write_cmos_sensor(0x02, 0x22);
	write_cmos_sensor(0x03, 0x96);
	write_cmos_sensor(0x04, 0x01);
	write_cmos_sensor(0x05, 0x00);
	write_cmos_sensor(0x06, 0x80);	
	write_cmos_sensor(0x11, 0x2b);
	write_cmos_sensor(0x12, 0x20);
	write_cmos_sensor(0x13, 0x03);
	write_cmos_sensor(0x15, 0x00);
	write_cmos_sensor(0x21, 0x10);
	write_cmos_sensor(0x22, 0x00);
	write_cmos_sensor(0x23, 0x10);
	write_cmos_sensor(0x24, 0x10);
	write_cmos_sensor(0x25, 0x10);
	write_cmos_sensor(0x26, 0x03);
	write_cmos_sensor(0x29, 0x01);
	write_cmos_sensor(0x2a, 0x0a);
	write_cmos_sensor(0x2b, 0x03);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xf9, 0x0e);
	write_cmos_sensor(0xfc, 0x0e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x25, 0xa2);
	write_cmos_sensor(0x3f, 0x1a);
	Sleep(100);
	write_cmos_sensor(0x25,0xe2);
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0x89,0x03);
}    /*    sensor_init  */


UINT32 gc033a_read_id(void)
{
  volatile signed char i;
  UINT16 sensor_id = 0;
  for(i=0;i<3;i++)
  {
    sensor_id = ((read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1));
    if(sensor_id==GC033A_YUV_ID)
      break;
  }
  YUV_DBG("yuv sensor_id = %x\n",sensor_id);
  return ((sensor_id==GC033A_YUV_ID)?1:0);
}


/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void gc033a_open(void)
{
  YUV_DBG("%s\n",__func__);

    sensor_init();
}

YUV_SENSOR_FUNC gc033a_sensor_func = {

  gc033a_open,
  GC033A_MIPI_GetYUVSensorBV,

};

UINT32 GC033A_YUV_SENSOR_INIT(PYUV_SENSOR_FUNC *pfFunc)
{
  if(NULL != pfFunc)
    *pfFunc = &gc033a_sensor_func;
  YUV_DBG("%s\n",__func__);
  return gc033a_read_id();
}
