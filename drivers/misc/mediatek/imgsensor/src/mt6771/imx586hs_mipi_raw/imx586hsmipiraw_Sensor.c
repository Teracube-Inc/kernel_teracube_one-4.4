/*
 * Copyright (C) 2017 MediaTek Inc.
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
 *	 IMX586HSHSmipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
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
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx586hsmipiraw_Sensor.h"
#include "imx586hs_eeprom.h"

#undef VENDOR_EDIT

#define USE_BURST_MODE 1
/***************Modify Following Strings for Debug**********************/
#define PFX "IMX586HS_camera_sensor"
#define LOG_1 LOG_INF("IMX586HS,MIPI 4LANE\n")
/****************************   Modify end	**************************/

#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

#define CUS1_12M_60_FPS 1/* 0:CUS1_12M_100_FPS, 1:CUS1_12M_60_FPS */
#define I2C_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */

static kal_uint8 qsc_flag;
#if USE_BURST_MODE
static kal_uint16
	imx586hs_table_write_cmos_sensor(kal_uint16 * para, kal_uint32 len);
#endif
#define FAST_READOUT 0
#define VIDEO_60_FPS 1
static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX586HS_SENSOR_ID,

	.checksum_value = 0x8ac2d94a,
#if FAST_READOUT
	.pre = {
		.pclk = 1728000000,
		.linelength = 7872,
		.framelength = 3658,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.mipi_pixel_rate = 1629260000,
		.max_framerate = 600, /* 30fps */
	},
#else
	.pre = {
		.pclk = 1473600000,
		.linelength = 7872,
		.framelength = 3118,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.mipi_pixel_rate = 1017600000,
		.max_framerate = 600, /* 30fps */
	},
#endif
	.cap = { /* reg_E Full size @30fps*/
		.pclk = 1718400000,
		.linelength = 9440,
		.framelength = 6067,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8000,
		.grabwindow_height = 6000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1692340000,
		.max_framerate = 300,
	},
#if VIDEO_60_FPS
	.normal_video = {
		.pclk = 1051000000,
		.linelength = 7872,
		.framelength = 2234,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3840,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 670630000,
		.max_framerate = 600,
	},
#else
	.normal_video = {
		.pclk = 528000000,
		.linelength = 7872,
		.framelength = 2234,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3840,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 331890000,
		.max_framerate = 300,
	},

#endif
	.hs_video = { /* reg_G 1920x1080 @120fps (binning)*/
		.pclk = 1728000000,
		.linelength = 5376,
		.framelength = 2678,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1280,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 789940000,
		.max_framerate = 1200,
	},

	.slim_video = { /* reg_F 2000x1500 @30fps (binning) */
		.pclk = 1728000000,
		.linelength = 5376,
		.framelength = 10714,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2000,
		.grabwindow_height = 1500,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 814630000,
		.max_framerate = 300,
	},

#if CUS1_12M_60_FPS
	.custom1 = { /*reg_B 4000x3000 @60fps (binning)*/
		.pclk = 1003200000,
		.linelength = 5376,
		.framelength = 3110,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 846510000,
		.max_framerate = 600, /*60fps*/
	},
#else /*CUS1_12M_100_FPS*/
	.custom1 = { /*reg_B 4000x3000 @100fps (binning)*/
		.pclk = 1702400000,
		.linelength = 5376,
		.framelength = 3166,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1492110000,
		.max_framerate = 1000, /*100fps*/
	},
#endif


	.custom2 = { /* reg_E 1280x720 @ 240fps (binning)*/
		.pclk = 1728000000,
		.linelength = 5376,
		.framelength = 1338,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_pixel_rate = 563660000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 2400,
	},

	.custom3 = { /* reg_E 8000x6000 @30fps*/
		.pclk = 1718400000,
		.linelength = 9440,
		.framelength = 6067,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8000,
		.grabwindow_height = 6000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1692340000,
		.max_framerate = 300,
	},

	.custom4 = { /* reg_G 2000x1128 @240fps */
		.pclk = 1708800000,
		.linelength = 5376,
		.framelength = 1324,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2000,
		.grabwindow_height = 1128,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 857140000,
		.max_framerate = 2400,
	},

	.margin = 48,		/* sensor framelength & shutter margin */
	.min_shutter = 1,	/* min shutter */
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 6,	/* support sensor mode num */

	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	
	.custom1_delay_frame = 2,	/*32M */
	.custom2_delay_frame = 2,	/*48M@15fps*/
	.custom3_delay_frame = 2,	/*stero@34fps*/
	.custom4_delay_frame = 2,	/* enter custom2 delay frame num */
	.frame_time_delay_frame = 3,
	
	.isp_driving_current = ISP_DRIVING_2MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY, /* 0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 0, /* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */

	//[agold][xfl][20190313][start]
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
	//[agold][xfl][20190313][end]

	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.i2c_addr_table = {0x34, 0x20, 0xff},
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_speed = 1000, /* i2c read/write speed */
};


static struct imgsensor_struct imgsensor = {
	//[agold][xfl][20190313][start]
	.mirror = IMAGE_NORMAL,				/* mirrorflip information */
	//[agold][xfl][20190313][end]
	
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview,
	*  Capture, Video,High Speed Video, Slim Video
	*/
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,					/* current shutter */
	.gain = 0x100,						/* current gain */
	.dummy_pixel = 0,					/* current dummypixel */
	.dummy_line = 0,					/* current dummyline */
	.current_fps = 300,
	/* auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker */
	.autoflicker_en = KAL_FALSE,
	/* test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output */
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/* current scenario id */
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x34, /* record current sensor's i2c write id */
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[9] = {
	{8000, 6000, 0000, 0000, 8000, 6000, 4000, 3000,
	 0000, 0000, 4000, 3000,    0,    0, 4000, 3000}, /* Preview */
	{8000, 6000, 0000, 0000, 8000, 6000, 8000, 6000,
	 0000, 0000, 8000, 6000,    0,    0, 8000, 6000}, /* capture */
	{8000, 6000, 0000,  840, 8000, 4320, 4000, 2160,
	   80, 0000, 3840, 2160,    0,    0, 3840, 2160}, /* video */
	{8000, 6000, 0000,  440, 8000, 5120, 2000, 1280,
	   40, 0000, 1920, 1280,    0,    0, 1920, 1280}, /* high speed video */
	{8000, 6000, 0000, 0000, 8000, 6000, 2000, 1500,
	 0000, 0000, 2000, 1500,    0,    0, 2000, 1500}, /*  slim */
#if CUS1_12M_60_FPS
	{8000, 6000, 0000, 0000, 8000, 6000, 4000, 3000,
	 0000, 0000, 4000, 3000,    0,    0, 4000, 3000}, /*  custom1 */
#else /*CUS1_12M_100_FPS*/
	{8000, 6000, 0000, 0000, 8000, 6000, 4000, 3000,
	 0000, 0000, 4000, 3000,    0,    0, 4000, 3000}, /*  custom1 */
#endif
	{8000, 6000, 0000, 1560, 8000, 2880, 2000,  720,
	  360, 0000, 1280,  720,    0,    0, 1280,  720}, /* custom2 */
	{8000, 6000, 0,   0, 8000, 6000, 8000, 6000,
	0,   0, 8000, 6000,  0,  0, 8000, 6000}, /* custom3 */
	{8000, 6000, 0, 440, 8000, 5120, 2000, 1280,
	0,  76, 2000, 1128,  0,  0, 2000, 1128}, /* custom4 */
};

static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[4] = {
	/* Preview mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0FA0, 0x0BB8, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x34, 0x04D8, 0x05D0, 0x00, 0x00, 0x0000, 0x0000},
	/* Normal_Video mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1F40, 0x1770, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x34, 0x026C, 0x0BA0, 0x00, 0x00, 0x0000, 0x0000},
	/* Video mode setting */
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1F40, 0x11A0, 0x00, 0x00, 0x0000, 0x0000,
	 0x00, 0x34, 0x04b0, 0x0430, 0x00, 0x00, 0x0000, 0x0000},
	 	/* Slim_Video mode setting */
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1F40, 0x11A0, 0x00, 0x00, 0x0000, 0x0000,
	 0x00, 0x34, 0x04D8, 0x0460, 0x00, 0x00, 0x0000, 0x0000}
};


/* If mirror flip */
static SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning = {
	.i4OffsetX = 17,
	.i4OffsetY = 12,
	.i4PitchX  =  8,
	.i4PitchY  = 16,
	.i4PairNum  = 8,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = { {20, 13}, {18, 15}, {22, 17}, {24, 19},
		   {20, 21}, {18, 23}, {22, 25}, {24, 27} },
	.i4PosR = { {19, 13}, {17, 15}, {21, 17}, {23, 19},
		   {19, 21}, {17, 23}, {21, 25}, {23, 27} },
	.iMirrorFlip = 3,
	.i4BlockNumX = 496,
	.i4BlockNumY = 186,
	.i4Crop = {{0, 0}, {0, 0}, {0, 372}, {0, 0}, {0, 0},
		   {0, 0}, {0, 0}, {0,   0}, {0, 0}, {0, 0} }
};

static SET_PD_BLOCK_INFO_T imgsensor_pd_info_video = {
	.i4OffsetX = 17,
	.i4OffsetY = 12,
	.i4PitchX = 8,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 2,
	.i4PosL = {{20, 13}, {18, 15}, {22, 17}, {24, 19},
		   {20, 21}, {18, 23}, {22, 25}, {24, 27} },
	.i4PosR = {{19, 13}, {17, 15}, {21, 17}, {23, 19},
		   {19, 21}, {17, 23}, {21, 25}, {23, 27} },
	.iMirrorFlip = 3,
	.i4BlockNumX = 480,
	.i4BlockNumY = 134,
	.i4Crop = {{0, 0}, {0, 0}, {80, 420}, {0, 0}, {0, 0},
		   {0, 0}, {0, 0}, {0,   0}, {0, 0}, {0, 0} }
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF),
			     (char)(para >> 8), (char)(para & 0xFF)};

	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/ /* Add this func to set i2c speed by each sensor */
	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/ /* Add this func to set i2c speed by each sensor */
	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
			(char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void imx586hs_get_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		regDa[idx + 1] = read_cmos_sensor_8(regDa[idx]);
		LOG_INF("%x %x", regDa[idx], regDa[idx+1]);
	}
}
static void imx586hs_set_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		write_cmos_sensor_8(regDa[idx], regDa[idx + 1]);
		LOG_INF("%x %x", regDa[idx], regDa[idx+1]);
	}
}

static kal_uint16 imx586hs_QSC_setting[2304 * 2];
static kal_uint16 imx586hs_LRC_setting[384 * 2];


static void write_sensor_QSC(void)
{
	#if USE_BURST_MODE
	imx586hs_table_write_cmos_sensor(imx586hs_QSC_setting,
		sizeof(imx586hs_QSC_setting)/sizeof(kal_uint16));
	#else
	kal_uint16 idx = 0, addr_qsc = 0x7F00;

	for (idx = 0; idx < 2304; idx++) {
		addr_qsc = 0x7F00 + idx;
		write_cmos_sensor_8(addr_qsc, imx586hs_QSC_setting[2 * idx + 1]);
	}
	#endif
}

static void write_sensor_LRC(void)
{
	#if USE_BURST_MODE
	imx586hs_table_write_cmos_sensor(imx586hs_LRC_setting,
		sizeof(imx586hs_LRC_setting)/sizeof(kal_uint16));
	#else
	kal_uint16 idx = 0, sensor_lrc_0 = 0x7510, sensor_lrc_1 = 0x7600;

	for (idx = 0; idx < 192; idx++) {
		write_cmos_sensor_8(sensor_lrc_0+idx,
			imx586hs_LRC_setting[2 * idx + 1]);
		write_cmos_sensor_8(sensor_lrc_1+idx,
			imx586hs_LRC_setting[2 * idx + 1 + 192]);
	}
	#endif
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* return;*/ /* for test */
	write_cmos_sensor_8(0x0104, 0x01);

	write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor_8(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor_8(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor_8(0x0104, 0x00);

}	/*	set_dummy  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 itemp;

	LOG_INF("image_mirror = %d\n", image_mirror);
	itemp = read_cmos_sensor_8(0x0101);
	itemp &= ~0x03;

	switch (image_mirror) {

	case IMAGE_NORMAL:
	write_cmos_sensor_8(0x0101, itemp);
	break;

	case IMAGE_V_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x02);
	break;

	case IMAGE_H_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x01);
	break;

	case IMAGE_HV_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x03);
	break;
	}
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d\n", framerate,
		min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	#ifdef VENDOR_EDIT
	/*Yijun.Tan@camera.driver,20180116,add for slow shutter */
	int longexposure_times = 0;
	static int long_exposure_status;
	#endif

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10
				/ imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length*/
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340,
				 imgsensor.frame_length >> 8);
			write_cmos_sensor_8(0x0341,
				 imgsensor.frame_length & 0xFF);
			write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
		/* Extend frame length*/
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);

		LOG_INF("(else)imgsensor.frame_length = %d\n",
			imgsensor.frame_length);

	}
	#ifdef VENDOR_EDIT
	/*Yijun.Tan@camera.driver,20180116,add for slow shutter */
	while (shutter >= 65535) {
		shutter = shutter / 2;
		longexposure_times += 1;
	}

	if (longexposure_times > 0) {
		LOG_INF("enter long exposure mode, time is %d",
			longexposure_times);
		long_exposure_status = 1;
		imgsensor.frame_length = shutter + 32;
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x3100, longexposure_times & 0x07);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	} else if (long_exposure_status == 1) {
		long_exposure_status = 0;
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x3100, 0x00);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);

		LOG_INF("exit long exposure mode");
	}
	#endif
	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	LOG_INF("shutter =%d, framelength =%d\n",
		shutter, imgsensor.frame_length);

}	/*	write_shutter  */

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
} /* set_shutter */


/*************************************************************************
 * FUNCTION
 *	set_shutter_frame_length
 *
 * DESCRIPTION
 *	for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(kal_uint16 shutter,
				     kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/*0x3500, 0x3501, 0x3502 will increase VBLANK to
	 *get exposure larger than frame exposure
	 *AE doesn't update sensor gain at capture mode,
	 *thus extra exposure lines must be updated here.
	 */

	/* OV Recommend Solution */
	/*if shutter bigger than frame_length,
	 *should extend frame length first
	 */
	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;
	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
				imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340,
					imgsensor.frame_length >> 8);
			write_cmos_sensor_8(0x0341,
					imgsensor.frame_length & 0xFF);
			write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	LOG_INF(
		"Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter, imgsensor.frame_length, frame_length,
		dummy_line, read_cmos_sensor(0x0350));

}	/* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	 kal_uint16 reg_gain = 0x0;

	reg_gain = 1024 - (1024*64)/gain;
	return (kal_uint16) reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain, max_gain = 64 * BASEGAIN;

	if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3) {
		/* 48M@30FPS */
		max_gain = 16 * BASEGAIN;
	}

	if (gain < BASEGAIN || gain > max_gain) {
		printk("Error max gain setting: %d\n", max_gain);

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > max_gain)
			gain = max_gain;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	printk("gain = %d, reg_gain = 0x%x, max_gain:0x%x\n ",
		gain, reg_gain, max_gain);

	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0204, (reg_gain>>8) & 0xFF);
	write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	return gain;
} /* set_gain */

static kal_uint32 imx586hs_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	#if 0
	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

	grgain_32 = (pSetSensorAWB->ABS_GAIN_GR + 1) >> 1;
	rgain_32 = (pSetSensorAWB->ABS_GAIN_R + 1) >> 1;
	bgain_32 = (pSetSensorAWB->ABS_GAIN_B + 1) >> 1;
	gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB + 1) >> 1;
	printk("[%s] ABS_GAIN_GR:%d, grgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_GR, grgain_32);
	printk("[%s] ABS_GAIN_R:%d, rgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_R, rgain_32);
	printk("[%s] ABS_GAIN_B:%d, bgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_B, bgain_32);
	printk("[%s] ABS_GAIN_GB:%d, gbgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_GB, gbgain_32);

	write_cmos_sensor_8(0x0b8e, (grgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(0x0b8f, grgain_32 & 0xFF);
	write_cmos_sensor_8(0x0b90, (rgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(0x0b91, rgain_32 & 0xFF);
	write_cmos_sensor_8(0x0b92, (bgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(0x0b93, bgain_32 & 0xFF);
	write_cmos_sensor_8(0x0b94, (gbgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(0x0b95, gbgain_32 & 0xFF);

	imx586hs_awb_gain_table[1]  = (grgain_32 >> 8) & 0xFF;
	imx586hs_awb_gain_table[3]  = grgain_32 & 0xFF;
	imx586hs_awb_gain_table[5]  = (rgain_32 >> 8) & 0xFF;
	imx586hs_awb_gain_table[7]  = rgain_32 & 0xFF;
	imx586hs_awb_gain_table[9]  = (bgain_32 >> 8) & 0xFF;
	imx586hs_awb_gain_table[11] = bgain_32 & 0xFF;
	imx586hs_awb_gain_table[13] = (gbgain_32 >> 8) & 0xFF;
	imx586hs_awb_gain_table[15] = gbgain_32 & 0xFF;
	imx586hs_table_write_cmos_sensor(imx586hs_awb_gain_table,
		sizeof(imx586hs_awb_gain_table)/sizeof(kal_uint16));
	#endif

	return ERROR_NONE;
}

static kal_uint16 imx586hs_feedback_awbgain[] = {
	0x0b90, 0x00,
	0x0b91, 0x01,
	0x0b92, 0x00,
	0x0b93, 0x01,
};
/*write AWB gain to sensor*/
static void feedback_awbgain(kal_uint32 r_gain, kal_uint32 b_gain)
{
	UINT32 r_gain_int = 0;
	UINT32 b_gain_int = 0;

	r_gain_int = r_gain / 512;
	b_gain_int = b_gain / 512;
	#if 0
	/*write r_gain*/
	write_cmos_sensor_8(0x0B90, r_gain_int);
	write_cmos_sensor_8(0x0B91,
		(((r_gain*100) / 512) - (r_gain_int * 100)) * 2);

	/*write _gain*/
	write_cmos_sensor_8(0x0B92, b_gain_int);
	write_cmos_sensor_8(0x0B93,
		(((b_gain * 100) / 512) - (b_gain_int * 100)) * 2);
	#else
	imx586hs_feedback_awbgain[1] = r_gain_int;
	imx586hs_feedback_awbgain[3] = (
		((r_gain*100) / 512) - (r_gain_int * 100)) * 2;
	imx586hs_feedback_awbgain[5] = b_gain_int;
	imx586hs_feedback_awbgain[7] = (
		((b_gain * 100) / 512) - (b_gain_int * 100)) * 2;
	imx586hs_table_write_cmos_sensor(imx586hs_feedback_awbgain,
		sizeof(imx586hs_feedback_awbgain)/sizeof(kal_uint16));
	#endif
}

static void imx586hs_set_lsc_reg_setting(
		kal_uint8 index, kal_uint16 *regDa, MUINT32 regNum)
{
	int i;
	int startAddr[4] = {0x9D88, 0x9CB0, 0x9BD8, 0x9B00};
	/*0:B,1:Gb,2:Gr,3:R*/

	printk("E! index:%d, regNum:%d\n", index, regNum);

	write_cmos_sensor_8(0x0B00, 0x01); /*lsc enable*/
	write_cmos_sensor_8(0x9014, 0x01);
	write_cmos_sensor_8(0x4439, 0x01);
	mdelay(1);
	printk("Addr 0xB870, 0x380D Value:0x%x %x\n",
		read_cmos_sensor_8(0xB870), read_cmos_sensor_8(0x380D));
	/*define Knot point, 2'b01:u3.7*/
	write_cmos_sensor_8(0x9750, 0x01);
	write_cmos_sensor_8(0x9751, 0x01);
	write_cmos_sensor_8(0x9752, 0x01);
	write_cmos_sensor_8(0x9753, 0x01);

	for (i = 0; i < regNum; i++)
		write_cmos_sensor(startAddr[index] + 2*i, regDa[i]);

	write_cmos_sensor_8(0x0B00, 0x00); /*lsc disable*/
}
/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n",
		enable);
	if (enable)
		write_cmos_sensor_8(0x0100, 0X01);
	else
		write_cmos_sensor_8(0x0100, 0x00);
	return ERROR_NONE;
}


#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 765 /* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 3
#endif
static kal_uint16 imx586hs_table_write_cmos_sensor(kal_uint16 *para,
						 kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;

	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		/* Write when remain buffer size is less than 3 bytes
		 * or reach end of data
		 */
		if ((I2C_BUFFER_LEN - tosend) < 3
			|| IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd,
						tosend,
						imgsensor.i2c_write_id,
						3,
						imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;

#endif
	}
	return 0;
}

static kal_uint16 imx586hs_init_setting[] = {
	/*External Clock Setting*/
	0x0136, 0x18,
	0x0137, 0x00,
	0x3C7E, 0x03,
	0x3C7F, 0x06,
	0x0111, 0x03,


	0x3702, 0x1F,
	0x3706, 0x17,
	0x3707, 0x6F,
	0x380C, 0x00,
	0x3C00, 0x10,
	0x3C01, 0x10,
	0x3C02, 0x10,
	0x3C03, 0x10,
	0x3C04, 0x10,
	0x3C05, 0x01,
	0x3C06, 0x00,
	0x3C07, 0x00,
	0x3C08, 0x03,
	0x3C09, 0xFF,
	0x3C0A, 0x01,
	0x3C0B, 0x00,
	0x3C0C, 0x00,
	0x3C0D, 0x03,
	0x3C0E, 0xFF,
	0x3C0F, 0x20,
	0x3F86, 0x01,
	0x3F88, 0x00,
	0x3F89, 0x01,
	0x3F8E, 0x00,
	0x3F8F, 0x01,
	0x4D14, 0xA6,
	0x4D29, 0xB0,
	0x4D45, 0x74,
	0x4D49, 0x00,
	0x4D53, 0xB1,
	0x4D55, 0x00,
	0x4D5C, 0xA6,
	0x4D71, 0xB0,
	0x4D7B, 0x51,
	0x4D8D, 0x86,
	0x4D91, 0x00,
	0x4D99, 0x3C,
	0x4D9B, 0x17,
	0x4D9D, 0x00,
	0x4DA4, 0xA6,
	0x4DB9, 0xB0,
	0x4DD5, 0x72,
	0x4DD9, 0x00,
	0x4DE3, 0xF5,
	0x4DE5, 0x00,
	0x4DEC, 0xA6,
	0x4E01, 0xB0,
	0x4E1D, 0x4A,
	0x4E2B, 0x69,
	0x4E2D, 0x00,
	0x4E34, 0xA6,
	0x4E49, 0xB0,
	0x4E65, 0x49,
	0x4E69, 0x00,
	0x4E73, 0x7C,
	0x4E75, 0x00,
	0x4E81, 0x0D,
	0x4E85, 0x51,
	0x4E87, 0x82,
	0x4E95, 0xBF,
	0x4E97, 0x00,
	0x5282, 0x01,
	0x5715, 0x09,
	0x5717, 0x93,
	0x5729, 0x93,
	0x572B, 0x0A,
	0x572D, 0x0C,
	0x572F, 0x93,
	0x578F, 0x0D,
	0x57BF, 0x5C,
	0x5855, 0xA0,
	0x5857, 0x82,
	0x585B, 0x52,
	0x585F, 0x52,
	0x5861, 0x84,
	0x5863, 0x9E,
	0x586F, 0xA4,
	0x5C49, 0x59,
	0x5C4A, 0x01,
	0x5C4B, 0x9C,
	0x5D0A, 0x01,
	0x5D0B, 0x93,
	0x5D28, 0x01,
	0x5D29, 0x93,
	0x5D4A, 0x01,
	0x5D4B, 0x93,
	0x5D68, 0x01,
	0x5D69, 0x93,
	0x646F, 0x1E,
	0x6607, 0x1E,
	0x6630, 0x1E,
	0x6659, 0x1E,
	0x6682, 0x1E,
	0x66AB, 0x1E,
	0x66D4, 0x1E,
	0x66FD, 0x1E,
	0x6726, 0x1E,
	0x674F, 0x1E,
	0x6778, 0x1E,
	0x6C1C, 0x00,
	0x6C1D, 0x01,
	0x6C1E, 0x03,
	0x6C1F, 0x04,
	0x6C5C, 0x68,
	0x6C5D, 0x74,
	0x6C5E, 0x12,
	0x6C60, 0x95,
	0x6C61, 0x1B,
	0x6C63, 0x1E,
	0x6C64, 0x15,
	0x6C65, 0x03,
	0x6C66, 0x81,
	0x6C67, 0x85,
	0x6C68, 0x4A,
	0x6C6A, 0x08,
	0x6C6B, 0x06,
	0x6C6C, 0x49,
	0x6C6D, 0x20,
	0x6C6E, 0x99,
	0x6C6F, 0x13,
	0x6C71, 0x43,
	0x6C72, 0x62,
	0x6C73, 0x5E,
	0x6C74, 0x78,
	0x6C76, 0x74,
	0x6C77, 0xF9,
	0x6C79, 0xB0,
	0x6C7A, 0x54,
	0x6C7B, 0x8E,
	0x6C7D, 0x95,
	0x6C7E, 0x8B,
	0x6C80, 0x2E,
	0x6C81, 0x1D,
	0x6C82, 0x44,
	0x6C83, 0x81,
	0x6C84, 0x45,
	0x6C85, 0x62,
	0x6C87, 0x0C,
	0x6C88, 0x86,
	0x6C89, 0x51,
	0x6C8A, 0x20,
	0x6C8B, 0x89,
	0x6C8C, 0x19,
	0x6C8E, 0x45,
	0x6C8F, 0x02,
	0x6C90, 0xA2,
	0x6C91, 0x48,
	0x6C92, 0x1C,
	0x6C94, 0xB9,
	0x6C96, 0x60,
	0x6C97, 0x73,
	0x6C98, 0x02,
	0x6C99, 0x05,
	0x6C9A, 0x89,
	0x6C9B, 0x48,
	0x6C9C, 0xC4,
	0x6C9D, 0x1C,
	0x6C9E, 0x1C,
	0x6C9F, 0xC6,
	0x6CA0, 0x80,
	0x6CA1, 0xE5,
	0x6CA2, 0x56,
	0x6CA4, 0x07,
	0x6CA5, 0x85,
	0x6CA6, 0x30,
	0x6CA7, 0x60,
	0x6CA8, 0x81,
	0x6CA9, 0x15,
	0x6CAB, 0x43,
	0x6CAC, 0x62,
	0x6CAD, 0x58,
	0x6CAE, 0x48,
	0x6CAF, 0x22,
	0x6CB0, 0xA5,
	0x6CB1, 0xFC,
	0x6CB2, 0xD0,
	0x6CB3, 0x68,
	0x6CB4, 0x63,
	0x6CB5, 0x8A,
	0x6CB6, 0x05,
	0x6CB7, 0x09,
	0x6CB8, 0x4B,
	0x6CBA, 0x1E,
	0x6CBB, 0x18,
	0x6CBC, 0xE2,
	0x6CBD, 0x81,
	0x6CBE, 0x4A,
	0x6CBF, 0x56,
	0x6CC1, 0x08,
	0x6CC2, 0x06,
	0x6CC3, 0x38,
	0x6CC4, 0xE0,
	0x6CC5, 0x82,
	0x6CC6, 0x96,
	0x6CC8, 0x43,
	0x6CC9, 0x62,
	0x6CCA, 0x5A,
	0x6CCB, 0x28,
	0x6CCC, 0x22,
	0x6CCD, 0x75,
	0x6CCE, 0xF9,
	0x6CD0, 0x60,
	0x6CD1, 0x53,
	0x6CD2, 0x82,
	0x6CD3, 0x05,
	0x6CD4, 0x89,
	0x6CD5, 0x38,
	0x6CD6, 0xC4,
	0x6CD7, 0x1C,
	0x6CD8, 0x18,
	0x6CD9, 0xE3,
	0x6CDA, 0x80,
	0x6CDB, 0xE5,
	0x6CDC, 0x52,
	0x6CDE, 0x07,
	0x6CDF, 0x85,
	0x6CE0, 0x38,
	0x6CE1, 0xE0,
	0x6CE2, 0x81,
	0x6CE3, 0x15,
	0x6CE5, 0x43,
	0x6CE6, 0x62,
	0x6CE7, 0x5A,
	0x6CE8, 0x88,
	0x6CE9, 0x22,
	0x6CEA, 0xA5,
	0x6CEB, 0x69,
	0x6CED, 0x70,
	0x6CEE, 0x63,
	0x6CEF, 0x86,
	0x6CF0, 0x05,
	0x6CF1, 0x09,
	0x6CF2, 0x4B,
	0x6CF4, 0x1E,
	0x6CF5, 0x18,
	0x6CF6, 0xE2,
	0x6CF7, 0x81,
	0x6CF8, 0x6A,
	0x6CF9, 0x52,
	0x6CFB, 0x08,
	0x6E47, 0x01,
	0x6F29, 0x07,
	0x6F2A, 0x08,
	0x7100, 0x06,
	0x7101, 0x41,
	0x7102, 0x20,
	0x7103, 0x92,
	0x7104, 0x95,
	0x7106, 0x43,
	0x7107, 0x62,
	0x7108, 0x5A,
	0x7109, 0x88,
	0x710A, 0x22,
	0x710B, 0x75,
	0x710C, 0xB9,
	0x710E, 0x68,
	0x710F, 0x53,
	0x7110, 0x06,
	0x7111, 0x05,
	0x7112, 0x89,
	0x7113, 0x08,
	0x7114, 0x84,
	0x7115, 0x1E,
	0x7116, 0x1C,
	0x7117, 0xC2,
	0x7118, 0x80,
	0x7119, 0xE5,
	0x711A, 0x46,
	0x711C, 0x07,
	0x711D, 0x86,
	0x711E, 0x30,
	0x711F, 0xE0,
	0x7120, 0x61,
	0x7121, 0x11,
	0x7123, 0x43,
	0x7124, 0xA2,
	0x7125, 0x58,
	0x7126, 0x68,
	0x7127, 0x28,
	0x7128, 0x64,
	0x7129, 0xB9,
	0x712B, 0x68,
	0x712C, 0x53,
	0x712D, 0x06,
	0x712E, 0x04,
	0x712F, 0x89,
	0x7130, 0x0B,
	0x7132, 0x1E,
	0x7133, 0x1C,
	0x7134, 0xC2,
	0x7135, 0x81,
	0x7136, 0x65,
	0x7137, 0x46,
	0x7139, 0x08,
	0x713A, 0x86,
	0x713B, 0x38,
	0x713C, 0xE0,
	0x713D, 0xA1,
	0x713E, 0x12,
	0x7140, 0x43,
	0x7141, 0xA2,
	0x7142, 0x58,
	0x7143, 0x68,
	0x7144, 0x28,
	0x7145, 0x64,
	0x7146, 0xB9,
	0x7148, 0x70,
	0x7149, 0x63,
	0x714A, 0x0E,
	0x714B, 0x05,
	0x714C, 0x89,
	0x714D, 0x28,
	0x714E, 0x84,
	0x714F, 0x20,
	0x7150, 0x1C,
	0x7151, 0xE3,
	0x7152, 0x80,
	0x7153, 0xE5,
	0x7154, 0x4E,
	0x7156, 0x08,
	0x7157, 0x86,
	0x7158, 0x39,
	0x7159, 0x20,
	0x715A, 0x61,
	0x715B, 0x14,
	0x715D, 0x44,
	0x715E, 0x02,
	0x715F, 0x5A,
	0x7160, 0x78,
	0x7161, 0x1C,
	0x7162, 0xA5,
	0x7163, 0x79,
	0x7165, 0x80,
	0x7166, 0x63,
	0x7167, 0x8E,
	0x7168, 0x07,
	0x7169, 0x09,
	0x716A, 0x3B,
	0x716C, 0x22,
	0x716D, 0x18,
	0x716E, 0xE4,
	0x716F, 0x81,
	0x7170, 0xA5,
	0x7171, 0x52,
	0x7173, 0x09,
	0x7174, 0x86,
	0x7175, 0x41,
	0x7176, 0x60,
	0x7177, 0x72,
	0x7178, 0x14,
	0x717E, 0x1C,
	0x7180, 0x79,
	0x7189, 0xA2,
	0x71C4, 0xE4,
	0x71C9, 0xC9,
	0x71CB, 0x3B,
	0x71CD, 0x9D,
	0x71CE, 0x53,
	0x71D0, 0xC9,
	0x71D2, 0x1B,
	0x71D4, 0x9D,
	0x71D5, 0x63,
	0x71D7, 0xC9,
	0x71D9, 0xE2,
	0x71DA, 0x75,
	0x71E7, 0xDA,
	0x71EC, 0xDA,
	0x895C, 0x01,
	0x895D, 0x00,
	0x8962, 0x06,
	0x8967, 0x10,
	0x896B, 0x0D,
	0x896F, 0x25,
	0x8976, 0x00,
	0x8977, 0x00,
	0x9004, 0x14,
	0x9200, 0xF4,
	0x9201, 0xA7,
	0x9202, 0xF4,
	0x9203, 0xAA,
	0x9204, 0xF4,
	0x9205, 0xAD,
	0x9206, 0xF4,
	0x9207, 0xB0,
	0x9208, 0xF4,
	0x9209, 0xB3,
	0x920A, 0xB7,
	0x920B, 0x34,
	0x920C, 0xB7,
	0x920D, 0x36,
	0x920E, 0xB7,
	0x920F, 0x37,
	0x9210, 0xB7,
	0x9211, 0x38,
	0x9212, 0xB7,
	0x9213, 0x39,
	0x9214, 0xB7,
	0x9215, 0x3A,
	0x9216, 0xB7,
	0x9217, 0x3C,
	0x9218, 0xB7,
	0x9219, 0x3D,
	0x921A, 0xB7,
	0x921B, 0x3E,
	0x921C, 0xB7,
	0x921D, 0x3F,
	0x921E, 0x77,
	0x921F, 0x77,
	0x9222, 0xC4,
	0x9223, 0x4B,
	0x9224, 0xC4,
	0x9225, 0x4C,
	0x9226, 0xC4,
	0x9227, 0x4D,
	0x9385, 0xE6,
	0x9387, 0x55,
	0x9389, 0x55,
	0x938B, 0x55,
	0x938D, 0x50,
	0x938F, 0x3C,
	0x9391, 0x3C,
	0x9393, 0x32,
	0x9395, 0x26,
	0x9397, 0x26,
	0x9399, 0xA0,
	0x939B, 0x78,
	0x939D, 0x46,
	0x939F, 0x26,
	0x93A1, 0x26,
	0x93A3, 0xB4,
	0x93A5, 0x8C,
	0x93A7, 0x5A,
	0x93A9, 0x26,
	0x93AB, 0x26,
	0x93AD, 0xB4,
	0x93AF, 0x8C,
	0x93B1, 0x70,
	0x93B3, 0x26,
	0x93B5, 0x26,
	0x93B7, 0x64,
	0x93B9, 0x64,
	0x93BB, 0x50,
	0x93BD, 0x26,
	0x93BF, 0x26,
	0x93C1, 0x78,
	0x93C3, 0x78,
	0x93C5, 0x5A,
	0x93C7, 0x26,
	0x93C9, 0x26,
	0x93CB, 0x8C,
	0x93CD, 0x8C,
	0x93CF, 0x78,
	0x93D1, 0x26,
	0x93D3, 0x26,
	0x93D5, 0x78,
	0x93D7, 0x78,
	0x93D9, 0x5A,
	0x93DB, 0x26,
	0x93DD, 0x26,
	0x93DF, 0x50,
	0x93E1, 0x50,
	0x93E3, 0x50,
	0x93E5, 0x26,
	0x93E7, 0x26,
	0x9810, 0x14,
	0x9814, 0x14,
	0x99B2, 0x20,
	0x99B3, 0x0F,
	0x99B4, 0x0F,
	0x99B5, 0x0F,
	0x99B6, 0x0F,
	0x99E4, 0x0F,
	0x99E5, 0x0F,
	0x99E6, 0x0F,
	0x99E7, 0x0F,
	0x99E8, 0x0F,
	0x99E9, 0x0F,
	0x99EA, 0x0F,
	0x99EB, 0x0F,
	0x99EC, 0x0F,
	0x99ED, 0x0F,
	0xBC76, 0x0F,
	0xBC77, 0x88,
	0xBC79, 0xA0,
	0xBC7B, 0x78,
	0xBC7C, 0x10,
	0xBC7D, 0xA0,
	0xBC7F, 0x30,
	0xC020, 0x01,
	0xC027, 0x00,
	0xC13C, 0x01,
	0xC140, 0x08,
	0xC141, 0x30,
	0xC142, 0x07,
	0xC143, 0x01,
	0xC145, 0x00,
	0xC146, 0x01,
	0xC149, 0x00,
	0xC448, 0x01,
	0xC44B, 0x05,
	0xC44C, 0x05,
	0xC44D, 0x2D,
	0xC44F, 0x01,
	0xC451, 0x00,
	0xC452, 0x01,
	0xC455, 0x00,
	0xC61D, 0x00,
	0xC625, 0x00,
	0xC638, 0x03,
	0xC63B, 0x01,
	0xE286, 0x31,
	0xE2A6, 0x32,
	0xE2C6, 0x33,
	0xEC00, 0x01,

	0x9852, 0x00,
	0x9954, 0x0F,
	0xA7AD, 0x01,
	0xA7CB, 0x01,
	0xAE09, 0xFF,
	0xAE0A, 0xFF,
	0xAE12, 0x58,
	0xAE13, 0x58,
	0xAE15, 0x10,
	0xAE16, 0x10,
	0xAF05, 0x48,
	0xB07C, 0x02,

};

#if FAST_READOUT
static kal_uint16 imx586hs_preview_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x1E,
	0x0343, 0xC0,
	0x0340, 0x0E,
	0x0341, 0x4A,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	/*Mode Setting*/
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	/*Digital Crop & Scaling*/
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	/*Output Size Setting*/
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	/*Clock Setting*/
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x8C,
	0x0310, 0x01,
	/*Other Setting*/
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x01,
	0x3F81, 0x90,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x01,
	0x3FF9, 0x2A,
	0x3FFE, 0x00,
	0x3FFF, 0x6C,
	0x0202, 0x0E,
	0x0203, 0x1A,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};

#else
static kal_uint16 imx586hs_preview_setting[] = {
	/*MIPI output setting*/
	0x0112,	0x0A,
	0x0113,	0x0A,
	0x0114,	0x02,
	0x0342,	0x1E,
	0x0343,	0xC0,
	0x0340,	0x0C,
	0x0341,	0x2E,
	0x0344,	0x00,
	0x0345,	0x00,
	0x0346,	0x00,
	0x0347,	0x00,
	0x0348,	0x1F,
	0x0349,	0x3F,
	0x034A,	0x17,
	0x034B,	0x6F,
	0x0220,	0x62,
	0x0222,	0x01,
	0x0900,	0x01,
	0x0901,	0x22,
	0x0902,	0x08,
	0x3140,	0x00,
	0x3246,	0x81,
	0x3247,	0x81,
	0x3F15,	0x00,
	0x0401,	0x00,
	0x0404,	0x00,
	0x0405,	0x10,
	0x0408,	0x00,
	0x0409,	0x00,
	0x040A,	0x00,
	0x040B,	0x00,
	0x040C,	0x0F,
	0x040D,	0xA0,
	0x040E,	0x0B,
	0x040F,	0xB8,
	0x034C,	0x0F,
	0x034D,	0xA0,
	0x034E,	0x0B,
	0x034F,	0xB8,
	0x0301,	0x05,
	0x0303,	0x02,
	0x0305,	0x04,
	0x0306,	0x01,
	0x0307,	0x33,
	0x030B,	0x01,
	0x030D,	0x06,
	0x030E,	0x01,
	0x030F,	0x73,
	0x0310,	0x01,
	0x3620,	0x00,
	0x3621,	0x00,
	0x3C11,	0x04,
	0x3C12,	0x03,
	0x3C13,	0x2D,
	0x3F0C,	0x01,
	0x3F14,	0x00,
	0x3F80,	0x01,
	0x3F81,	0x90,
	0x3F8C,	0x00,
	0x3F8D,	0x14,
	0x3FF8,	0x01,
	0x3FF9,	0x2A,
	0x3FFE,	0x00,
	0x3FFF,	0x6C,
	0x0202,	0x0B,
	0x0203,	0xFE,
	0x0224,	0x01,
	0x0225,	0xF4,
	0x3FE0,	0x01,
	0x3FE1,	0xF4,
	0x0204,	0x00,
	0x0205,	0x70,
	0x0216,	0x00,
	0x0217,	0x70,
	0x0218,	0x01,
	0x0219,	0x00,
	0x020E,	0x01,
	0x020F,	0x00,
	0x0210,	0x01,
	0x0211,	0x00,
	0x0212,	0x01,
	0x0213,	0x00,
	0x0214,	0x01,
	0x0215,	0x00,
	0x3FE2,	0x00,
	0x3FE3,	0x70,
	0x3FE4,	0x01,
	0x3FE5,	0x00,
	0x3E20,	0x02,
	0x3E3B,	0x01,
	0x4434,	0x01,
	0x4435,	0xF0,

};
#endif
#if VIDEO_60_FPS
static kal_uint16 imx586hs_normal_video_setting_60fps[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	0x0342, 0x1E,
	0x0343, 0xC0,
	0x0340, 0x08,
	0x0341, 0xB0,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x03,
	0x0347, 0x48,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x14,
	0x034B, 0x27,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x09,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x50,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0x00,
	0x040E, 0x08,
	0x040F, 0x70,
	0x034C, 0x0F,
	0x034D, 0x00,
	0x034E, 0x08,
	0x034F, 0x70,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xDB,
	0x030B, 0x02,
	0x030D, 0x0C,
	0x030E, 0x03,
	0x030F, 0xD2,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x01,
	0x3F81, 0x90,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x01,
	0x3FF9, 0x2A,
	0x3FFE, 0x00,
	0x3FFF, 0x6C,
	0x0202, 0x08,
	0x0203, 0x80,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xE0,
};
#else
static kal_uint16 imx586hs_normal_video_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x1E,
	0x0343, 0xC0,
	0x0340, 0x08,
	0x0341, 0xBA,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x03,
	0x0347, 0x48,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x14,
	0x034B, 0x27,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x09,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	/*Digital Crop & Scaling*/
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x50,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0x00,
	0x040E, 0x08,
	0x040F, 0x70,
	0x034C, 0x0F,
	0x034D, 0x00,
	0x034E, 0x08,
	0x034F, 0x70,
	0x0301, 0x05,
	0x0303, 0x04,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xDC,
	0x030B, 0x04,
	0x030D, 0x0C,
	0x030E, 0x03,
	0x030F, 0xC8,
	0x0310, 0x01,
	/*Other Setting*/
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x01,
	0x3F81, 0x90,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x01,
	0x3FF9, 0x2A,
	0x3FFE, 0x00,
	0x3FFF, 0x6C,
	0x0202, 0x08,
	0x0203, 0x8A,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xE0,
};
#endif
static kal_uint16 imx586hs_hs_video_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x15,
	0x0343, 0x00,
	/*Frame Length Lines Setting*/
	0x0340, 0x0A,
	0x0341, 0x76,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x01,
	0x0347, 0xB8,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x15,
	0x034B, 0xB7,
	/*Mode Setting*/
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x09,
	0x3140, 0x00,
	0x3246, 0x89,
	0x3247, 0x89,
	0x3F15, 0x00,
	/*Digital Crop & Scaling*/
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x28,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x07,
	0x040D, 0x80,
	0x040E, 0x05,
	0x040F, 0x00,
	/*Output Size Setting*/
	0x034C, 0x07,
	0x034D, 0x80,
	0x034E, 0x05,
	0x034F, 0x00,
	/*Clock Setting*/
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xC0,
	0x0310, 0x01,
	/*Other Setting*/
	0x3620, 0x00,
	0x3621, 0x00,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x00,
	0x3FFC, 0x00,
	0x3FFD, 0x00,
	/*Integration Setting*/
	0x0202, 0x0A,
	0x0203, 0x46,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x00,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
};

static kal_uint16 imx586hs_slim_video_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x15,
	0x0343, 0x00,
	/*Frame Length Lines Setting*/
	0x0340, 0x29,
	0x0341, 0xDA,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	/*Mode Setting*/
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x09,
	0x3140, 0x00,
	0x3246, 0x89,
	0x3247, 0x89,
	0x3F15, 0x00,
	/*Digital Crop & Scaling*/
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x07,
	0x040D, 0xD0,
	0x040E, 0x05,
	0x040F, 0xDC,
	/*Output Size Setting*/
	0x034C, 0x07,
	0x034D, 0xD0,
	0x034E, 0x05,
	0x034F, 0xDC,
	/*Clock Setting*/
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xC6,
	0x0310, 0x01,
	/*Other Setting*/
	0x3620, 0x00,
	0x3621, 0x00,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x00,
	0x3FFC, 0x00,
	0x3FFD, 0x00,
	/*Integration Setting*/
	0x0202, 0x29,
	0x0203, 0xAA,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x00,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
};
static kal_uint16 imx586hs_custom3_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x24,
	0x0343, 0xE0,
	/*Frame Length Lines Setting*/
	0x0340, 0x17,
	0x0341, 0xB3,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	/*Mode Setting*/
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x01,
	0x3247, 0x01,
	0x3F15, 0x00,
	/*Digital Crop & Scaling*/
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x1F,
	0x040D, 0x40,
	0x040E, 0x17,
	0x040F, 0x70,
	/*Output Size Setting*/
	0x034C, 0x1F,
	0x034D, 0x40,
	0x034E, 0x17,
	0x034F, 0x70,
	/*Clock Setting*/
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x66,
	0x030B, 0x01,
	0x030D, 0x0C,
	0x030E, 0x04,
	0x030F, 0xD2,
	0x0310, 0x01,
	/*Other Setting*/
	0x3620, 0x01,
	0x3621, 0x01,
	0x3C11, 0x08,
	0x3C12, 0x08,
	0x3C13, 0x2A,
	0x3F0C, 0x00,
	0x3F14, 0x01,
	0x3F80, 0x02,
	0x3F81, 0x20,
	0x3F8C, 0x01,
	0x3F8D, 0x9A,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x02,
	0x3FFF, 0x0E,
	/*Integration Setting*/
	0x0202, 0x17,
	0x0203, 0x83,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	/*PDAF TYPE2 Setting*/
	0x3E20, 0x02,
	0x3E3B, 0x00,
	0x4434, 0x00,
	0x4435, 0xF8,
};

static kal_uint16 imx586hs_custom4_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x15,
	0x0343, 0x00,
	/*Frame Length Lines Setting*/
	0x0340, 0x05,
	0x0341, 0x2C,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x02,
	0x0347, 0xE8,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x14,
	0x034B, 0x87,
	/*Mode Setting*/
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x89,
	0x3247, 0x89,
	0x3F15, 0x00,
	/*Digital Crop & Scaling*/
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x07,
	0x040D, 0xD0,
	0x040E, 0x04,
	0x040F, 0x68,
	/*Output Size Setting*/
	0x034C, 0x07,
	0x034D, 0xD0,
	0x034E, 0x04,
	0x034F, 0x68,
	/*Clock Setting*/
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x64,
	0x030B, 0x01,
	0x030D, 0x0C,
	0x030E, 0x02,
	0x030F, 0x71,
	0x0310, 0x01,
	/*Other Setting*/
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x0C,
	0x3C12, 0x05,
	0x3C13, 0x2C,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x02,
	0x3F81, 0x67,
	0x3F8C, 0x02,
	0x3F8D, 0x44,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x01,
	0x3FFF, 0x90,
	/*Integration Setting*/
	0x0202, 0x04,
	0x0203, 0xFC,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	/*PDAF TYPE2 Setting*/
	0x3E20, 0x01,
	0x3E3B, 0x00,
	0x4434, 0x00,
	0x4435, 0xF8,
};
#if CUS1_12M_60_FPS
static kal_uint16 imx586hs_custom1_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x15,
	0x0343, 0x00,
	/*Frame Length Lines Setting*/
	0x0340, 0x0C,
	0x0341, 0x26,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	/*Mode Setting*/
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x09,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	/*Digital Crop & Scaling*/
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	/*Output Size Setting*/
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	/*Clock Setting*/
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xD1,
	0x030B, 0x02,
	0x030D, 0x08,
	0x030E, 0x03,
	0x030F, 0x37,
	0x0310, 0x01,
	/*Other Setting*/
	0x3620, 0x00,
	0x3621, 0x00,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x00,
	0x3FFC, 0x00,
	0x3FFD, 0x00,
	/*Integration Setting*/
	0x0202, 0x0B,
	0x0203, 0xF6,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x00,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
};

#else /* CUS1_12M_100_FPS */

static kal_uint16 imx586hs_custom1_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x15,
	0x0343, 0x00,
	/*Frame Length Lines Setting*/
	0x0340, 0x0C,
	0x0341, 0x5E,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	/*Mode Setting*/
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x09,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	/*Digital Crop & Scaling*/
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	/*Output Size Setting*/
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	/*Clock Setting*/
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x03,
	0x0306, 0x01,
	0x0307, 0x0A,
	0x030B, 0x01,
	0x030D, 0x03,
	0x030E, 0x01,
	0x030F, 0x10,
	0x0310, 0x01,
	/*Other Setting*/
	0x3620, 0x00,
	0x3621, 0x00,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x00,
	0x3FFC, 0x00,
	0x3FFD, 0x00,
	/*Integration Setting*/
	0x0202, 0x0C,
	0x0203, 0x2E,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x00,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
};
#endif

static kal_uint16 imx586hs_custom2_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x15,
	0x0343, 0x00,
	/*Frame Length Lines Setting*/
	0x0340, 0x05,
	0x0341, 0x3A,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x06,
	0x0347, 0x18,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x11,
	0x034B, 0x57,
	/*Mode Setting*/
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x09,
	0x3140, 0x00,
	0x3246, 0x89,
	0x3247, 0x89,
	0x3F15, 0x00,
	/*Digital Crop & Scaling*/
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x01,
	0x0409, 0x68,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x05,
	0x040D, 0x00,
	0x040E, 0x02,
	0x040F, 0xD0,
	/*Output Size Setting*/
	0x034C, 0x05,
	0x034D, 0x00,
	0x034E, 0x02,
	0x034F, 0xD0,
	/*Clock Setting*/
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x12,
	0x0310, 0x01,
	/*Other Setting*/
	0x3620, 0x00,
	0x3621, 0x00,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x00,
	0x3FFC, 0x00,
	0x3FFD, 0x00,
	/*Integration Setting*/
	0x0202, 0x05,
	0x0203, 0x0A,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x00,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
};
static void sensor_init(void)
{
	LOG_INF("E\n");

	imx586hs_table_write_cmos_sensor(imx586hs_init_setting,
		sizeof(imx586hs_init_setting)/sizeof(kal_uint16));
	set_mirror_flip(imgsensor.mirror);

	LOG_INF("X");
}	/*	  sensor_init  */

static void preview_setting(void)
{
	LOG_INF("E\n");

	imx586hs_table_write_cmos_sensor(imx586hs_preview_setting,
		sizeof(imx586hs_preview_setting)/sizeof(kal_uint16));
	LOG_INF("X");
} /* preview_setting */


/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("%s 30 fps E! currefps:%d\n", __func__, currefps);
	/*************MIPI output setting************/
	/*MIPI output setting*/
	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x02);
	/*Line Length PCK Setting*/
	write_cmos_sensor_8(0x0342, 0x24);
	write_cmos_sensor_8(0x0343, 0xE0);
	/*Frame Length Lines Setting*/
	write_cmos_sensor_8(0x0340, 0x17);
	write_cmos_sensor_8(0x0341, 0xB3);
	/*ROI Setting*/
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x00);
	write_cmos_sensor_8(0x0347, 0x00);
	write_cmos_sensor_8(0x0348, 0x1F);
	write_cmos_sensor_8(0x0349, 0x3F);
	write_cmos_sensor_8(0x034A, 0x17);
	write_cmos_sensor_8(0x034B, 0x6F);
	/*Mode Setting*/
	write_cmos_sensor_8(0x0220, 0x62);
	write_cmos_sensor_8(0x0222, 0x01);
	write_cmos_sensor_8(0x0900, 0x00);
	write_cmos_sensor_8(0x0901, 0x11);
	write_cmos_sensor_8(0x0902, 0x08);
	write_cmos_sensor_8(0x3140, 0x00);
	write_cmos_sensor_8(0x3246, 0x01);
	write_cmos_sensor_8(0x3247, 0x01);
	write_cmos_sensor_8(0x3F15, 0x00);
	/*Digital Crop & Scaling*/
	write_cmos_sensor_8(0x0401, 0x00);
	write_cmos_sensor_8(0x0404, 0x00);
	write_cmos_sensor_8(0x0405, 0x10);
	write_cmos_sensor_8(0x0408, 0x00);
	write_cmos_sensor_8(0x0409, 0x00);
	write_cmos_sensor_8(0x040A, 0x00);
	write_cmos_sensor_8(0x040B, 0x00);
	write_cmos_sensor_8(0x040C, 0x1F);
	write_cmos_sensor_8(0x040D, 0x40);
	write_cmos_sensor_8(0x040E, 0x17);
	write_cmos_sensor_8(0x040F, 0x70);
	/*Output Size Setting*/
	write_cmos_sensor_8(0x034C, 0x1F);
	write_cmos_sensor_8(0x034D, 0x40);
	write_cmos_sensor_8(0x034E, 0x17);
	write_cmos_sensor_8(0x034F, 0x70);
	/*Clock Setting*/
	write_cmos_sensor_8(0x0301, 0x05);
	write_cmos_sensor_8(0x0303, 0x02);
	write_cmos_sensor_8(0x0305, 0x04);
	write_cmos_sensor_8(0x0306, 0x01);
	write_cmos_sensor_8(0x0307, 0x66);
	write_cmos_sensor_8(0x030B, 0x01);
	write_cmos_sensor_8(0x030D, 0x0C);
	write_cmos_sensor_8(0x030E, 0x04);
	write_cmos_sensor_8(0x030F, 0xD2);
	write_cmos_sensor_8(0x0310, 0x01);
	/*Other Setting*/
	write_cmos_sensor_8(0x3620, 0x01);
	write_cmos_sensor_8(0x3621, 0x01);
	write_cmos_sensor_8(0x3C11, 0x08);
	write_cmos_sensor_8(0x3C12, 0x08);
	write_cmos_sensor_8(0x3C13, 0x2A);
	write_cmos_sensor_8(0x3F0C, 0x00);
	write_cmos_sensor_8(0x3F14, 0x01);
	write_cmos_sensor_8(0x3F80, 0x02);
	write_cmos_sensor_8(0x3F81, 0x20);
	write_cmos_sensor_8(0x3F8C, 0x01);
	write_cmos_sensor_8(0x3F8D, 0x9A);
	write_cmos_sensor_8(0x3FF8, 0x00);
	write_cmos_sensor_8(0x3FF9, 0x00);
	write_cmos_sensor_8(0x3FFE, 0x02);
	write_cmos_sensor_8(0x3FFF, 0x0E);
	/*Integration Setting*/
	write_cmos_sensor_8(0x0202, 0x17);
	write_cmos_sensor_8(0x0203, 0x83);
	write_cmos_sensor_8(0x0224, 0x01);
	write_cmos_sensor_8(0x0225, 0xF4);
	write_cmos_sensor_8(0x3FE0, 0x01);
	write_cmos_sensor_8(0x3FE1, 0xF4);
	/*Gain Setting*/
	write_cmos_sensor_8(0x0204, 0x00);
	write_cmos_sensor_8(0x0205, 0x70);
	write_cmos_sensor_8(0x0216, 0x00);
	write_cmos_sensor_8(0x0217, 0x70);
	write_cmos_sensor_8(0x0218, 0x01);
	write_cmos_sensor_8(0x0219, 0x00);
	write_cmos_sensor_8(0x020E, 0x01);
	write_cmos_sensor_8(0x020F, 0x00);
	write_cmos_sensor_8(0x0210, 0x01);
	write_cmos_sensor_8(0x0211, 0x00);
	write_cmos_sensor_8(0x0212, 0x01);
	write_cmos_sensor_8(0x0213, 0x00);
	write_cmos_sensor_8(0x0214, 0x01);
	write_cmos_sensor_8(0x0215, 0x00);
	write_cmos_sensor_8(0x3FE2, 0x00);
	write_cmos_sensor_8(0x3FE3, 0x70);
	write_cmos_sensor_8(0x3FE4, 0x01);
	write_cmos_sensor_8(0x3FE5, 0x00);
	/*PDAF TYPE1 Setting*/
	write_cmos_sensor_8(0x3E20, 0x02);
	write_cmos_sensor_8(0x3E37, 0x00);
	/*PDAF TYPE2 Setting*/
	write_cmos_sensor_8(0x3E20, 0x02);
	write_cmos_sensor_8(0x3E3B, 0x01);
	write_cmos_sensor_8(0x4434, 0x00);
	write_cmos_sensor_8(0x4435, 0xF8);

	LOG_INF("%s 30 fpsX\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("%s E! currefps:%d\n", __func__, currefps);

#if VIDEO_60_FPS
	LOG_INF("imx586hs VIDEO_60_FPS\n");
	imx586hs_table_write_cmos_sensor(imx586hs_normal_video_setting_60fps,
		sizeof(imx586hs_normal_video_setting_60fps)/sizeof(kal_uint16));
#else
	LOG_INF("imx586hs VIDEO_30_FPS\n");
	imx586hs_table_write_cmos_sensor(imx586hs_normal_video_setting,
		sizeof(imx586hs_normal_video_setting)/sizeof(kal_uint16));
#endif
	LOG_INF("X\n");
}

static void hs_video_setting(void)
{
	LOG_INF("%s E! currefps 120\n", __func__);


	imx586hs_table_write_cmos_sensor(imx586hs_hs_video_setting,
		sizeof(imx586hs_hs_video_setting)/sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void slim_video_setting(void)
{
	LOG_INF("%s E\n", __func__);

	LOG_INF("%s 30 fps E! currefps\n", __func__);


	imx586hs_table_write_cmos_sensor(imx586hs_slim_video_setting,
		sizeof(imx586hs_slim_video_setting)/sizeof(kal_uint16));

	LOG_INF("X\n");
}


/*full size 30fps*/
static void custom3_setting(void)
{
	pr_debug("%s 30 fps E!\n", __func__);
	/*************MIPI output setting************/

	imx586hs_table_write_cmos_sensor(imx586hs_custom3_setting,
		sizeof(imx586hs_custom3_setting)/sizeof(kal_uint16));

	pr_debug("%s 30 fpsX\n", __func__);
}

static void custom4_setting(void)
{
	pr_debug("%s 720p@240 fps E! currefps\n", __func__);


	imx586hs_table_write_cmos_sensor(imx586hs_custom4_setting,
		sizeof(imx586hs_custom4_setting)/sizeof(kal_uint16));
	pr_debug("X\n");
}

static void custom1_setting(void)
{
	LOG_INF("%s E! currefps %d\n",
		__func__, imgsensor.current_fps);
	/*************MIPI output setting************/

	imx586hs_table_write_cmos_sensor(imx586hs_custom1_setting,
		sizeof(imx586hs_custom1_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}

static void custom2_setting(void)
{
	LOG_INF("%s 1280x720@240fps E! currefps\n", __func__);
	/*************MIPI output setting************/

	imx586hs_table_write_cmos_sensor(imx586hs_custom2_setting,
		sizeof(imx586hs_custom2_setting)/sizeof(kal_uint16));
	LOG_INF("X");
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
//extern int g_CameraSensorIdx;
extern bool hs_selective_read_eeprom(kal_uint16 addr, BYTE *data);
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
    kal_uint8 eeprom_id = 0;
	/*sensor have two i2c address 0x34 & 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
					| read_cmos_sensor_8(0x0017));
			pr_debug(
				"read_0x0000=0x%x, 0x0001=0x%x,0x0000_0001=0x%x\n",
				read_cmos_sensor_8(0x0016),
				read_cmos_sensor_8(0x0017),
				read_cmos_sensor(0x0000));
            hs_selective_read_eeprom(0x0000,&eeprom_id);
            //printk("imx586hs hs eeprom_id = 0x%x\n",eeprom_id); 
            if(eeprom_id == 0x01)
                eeprom_id = 2;
            else
                eeprom_id = 0xf;
            *sensor_id += eeprom_id;
            //printk("imx586hs sensor_id = 0x%x\n",*sensor_id); 
			if (*sensor_id == imgsensor_info.sensor_id) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}

			pr_debug("Read sensor id fail, id: 0x%x\n",
				imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	open
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
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;
    kal_uint8 eeprom_id = 0;

	pr_debug("%s +\n", __func__);
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
					| read_cmos_sensor_8(0x0017));

            hs_selective_read_eeprom(0x0000,&eeprom_id);
            //printk("imx586hs hs eeprom_id = 0x%x\n",eeprom_id); 
            if(eeprom_id == 0x01)
                eeprom_id = 2;
            else
                eeprom_id = 0xf;
            sensor_id += eeprom_id;
            //printk("imx586hs sensor_id = 0x%x\n",sensor_id); 

			if (sensor_id == imgsensor_info.sensor_id) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			pr_debug("Read sensor id fail, id: 0x%x\n",
				imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail sequence write in  */

	sensor_init();

	write_sensor_LRC();
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
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
static kal_uint32 close(void)
{
	LOG_INF("E\n");
	/* No Need to implement this function */
	streaming_control(KAL_FALSE);
	qsc_flag = 0;
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();

	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		printk(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate / 10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}	/*	slim_video	 */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting();


	return ERROR_NONE;
}

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom2_setting();

	return ERROR_NONE;
}

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	if (!qsc_flag) {
		printk("write_sensor_QSC Start\n");
		write_sensor_QSC();
		printk("write_sensor_QSC End\n");
		qsc_flag = 1;
	}
	custom3_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* custom3 */

static kal_uint32 custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. 720P@240FPS\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	imgsensor.pclk = imgsensor_info.custom4.pclk;
	imgsensor.line_length = imgsensor_info.custom4.linelength;
	imgsensor.frame_length = imgsensor_info.custom4.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom4_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* custom4 */

static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width =
		imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height =
		imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width =
		imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height =
		imgsensor_info.custom2.grabwindow_height;

	sensor_resolution->SensorCustom3Width =
		imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height =
		imgsensor_info.custom3.grabwindow_height;

	sensor_resolution->SensorCustom4Width =
		imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height =
		imgsensor_info.custom4.grabwindow_height;

	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 2;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;

	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
		break;

	case MSDK_SCENARIO_ID_CUSTOM2:
		sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
		break;

	case MSDK_SCENARIO_ID_CUSTOM3:
		sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
		break;

	case MSDK_SCENARIO_ID_CUSTOM4:
		sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;
		break;

		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		custom2(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		custom3(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		custom4(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else /* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
		MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10
				/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
		? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk /
				framerate * 10 /
				imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.normal_video.framelength)
		? (frame_length - imgsensor_info.normal_video.framelength)
		: 0;
		imgsensor.frame_length =
			imgsensor_info.normal_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		printk(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
			, framerate, imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10
				/ imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			  ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength
				+ imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10
				/ imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength)
			  ? (frame_length - imgsensor_info.hs_video.framelength)
			  : 0;
		imgsensor.frame_length =
			imgsensor_info.hs_video.framelength
				+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10
			/ imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.slim_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10
				/ imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom1.framelength)
			? (frame_length - imgsensor_info.custom1.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom1.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10
				/ imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom2.framelength)
			? (frame_length - imgsensor_info.custom2.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom2.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10
				/ imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom3.framelength)
		? (frame_length - imgsensor_info.custom3.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.custom3.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk / framerate * 10
				/ imgsensor_info.custom4.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom4.framelength)
		? (frame_length - imgsensor_info.custom4.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.custom4.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
		MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		*framerate = imgsensor_info.custom4.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable)
		write_cmos_sensor_8(0x0601, 0x0002); /*100% Color bar*/
	else
		write_cmos_sensor_8(0x0601, 0x0000); /*No pattern*/

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/* unsigned long long *feature_return_para
	 *  = (unsigned long long *) feature_para;
	 */
	SET_PD_BLOCK_INFO_T *PDAFinfo;
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	SENSOR_VC_INFO_STRUCT *pvcinfo;
	/* SET_SENSOR_AWB_GAIN *pSetSensorAWB
	 *  = (SET_SENSOR_AWB_GAIN *)feature_para;
	 */
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*printk("feature_id = %d\n", feature_id);*/
	switch (feature_id) {

	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		 set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		 /* night_mode((BOOL) *feature_data); */
		break;
	#ifdef VENDOR_EDIT
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = imgsensor_info.module_id;
		break;
	#endif
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_8(sensor_reg_data->RegAddr,
				    sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor_8(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/*get the lens driver ID from EEPROM
		 * or just return LENS_DRIVER_ID_DO_NOT_CARE
		 * if EEPROM does not exist in camera module.
		 */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,
				      *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		 set_max_framerate_by_scenario(
				(MSDK_SCENARIO_ID_ENUM)*feature_data,
				*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		 get_default_framerate_by_scenario(
				(MSDK_SCENARIO_ID_ENUM)*(feature_data),
				(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		pr_debug("SENSOR_FEATURE_GET_PDAF_DATA\n");
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		pr_debug("current fps :%d\n", (UINT32)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		pr_debug("ihdr enable :%d\n", (BOOL)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);
		wininfo =
	(SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
printk("MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG\n");
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
printk("MSDK_SCENARIO_ID_VIDEO_PREVIEW\n");
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
printk("MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO\n");
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[3],
			sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
printk("MSDK_SCENARIO_ID_SLIM_VIDEO\n");
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[4],
			sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
printk("MSDK_SCENARIO_ID_CUSTOM1\n");
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[5],
			sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
printk("MSDK_SCENARIO_ID_CUSTOM2\n");
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[6],
			sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
printk("MSDK_SCENARIO_ID_CUSTOM3\n");
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[7],
			sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
printk("MSDK_SCENARIO_ID_CUSTOM4\n");
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[8],
			sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
printk("MSDK_SCENARIO_ID_CAMERA_PREVIEW default\n");
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[0],
			sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		pr_debug("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16) *feature_data);
		PDAFinfo =
		  (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_binning,
				 sizeof(SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info_binning,
				sizeof(SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info_video,
				sizeof(SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:

		default:
			break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF(
		"SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
			(UINT16) *feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			/* video & capture use same setting */
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		pr_debug("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		imx586hs_get_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32)
					   , feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
		pr_debug("SENSOR_FEATURE_SET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		imx586hs_set_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32)
					   , feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF:
		pr_debug("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data),
					(UINT16) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		pr_debug("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		#if 0
		ihdr_write_shutter((UINT16)*feature_data,
				   (UINT16)*(feature_data+1));
		#endif
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom4.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
break;

	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n",
			(UINT16)*feature_data);
		pvcinfo =
		 (SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
				sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
				sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		default:
			#if 0
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			#endif
			break;
		}
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		/* modify to separate 3hdr and remosaic */
		if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3) {
			/*write AWB gain to sensor*/
			feedback_awbgain((UINT32)*(feature_data_32 + 1),
					(UINT32)*(feature_data_32 + 2));
		} else {
			imx586hs_awb_gain(
				(struct SET_SENSOR_AWB_GAIN *) feature_para);
		}
		break;
	case SENSOR_FEATURE_SET_LSC_TBL:
		{
		kal_uint8 index =
			*(((kal_uint8 *)feature_para) + (*feature_para_len));

		imx586hs_set_lsc_reg_setting(index, feature_data_16,
					  (*feature_para_len)/sizeof(UINT16));
		}
		break;
	default:
		break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX586HS_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{

	if (pfFunc != NULL)
		*pfFunc =  &sensor_func;
	return ERROR_NONE;
}	
