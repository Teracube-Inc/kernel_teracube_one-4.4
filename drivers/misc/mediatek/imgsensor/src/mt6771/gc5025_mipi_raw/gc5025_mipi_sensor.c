 /*
 *
 * Filename:
 * ---------
 *     gc5025mipi_Sensor.c
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
 *-----------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 */

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

#include "gc5025_mipi_sensor.h"

/************************** Modify Following Strings for Debug **************************/
#define PFX "gc5025_camera_sensor"
#define LOG_1 LOG_INF("GC5025, MIPI 2LANE\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);
kal_bool GC5025DuringTestPattern = KAL_FALSE;

static kal_uint32 Dgain_ratio = 256;
static kal_bool DR_State = KAL_TRUE;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = GC5025MIPI_SENSOR_ID,
	.checksum_value = 0xf7375923,

	.pre = {
		.pclk = 288000000,
		.linelength = 4800,
		.framelength = 2000,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 172800000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 288000000,
		.linelength = 4800,
		.framelength = 2000,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 172800000,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 288000000,
		.linelength = 4800,
		.framelength = 2000,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 172800000,
		.max_framerate = 240,
	},
	.normal_video = {
		.pclk = 288000000,
		.linelength = 4800,
		.framelength = 2000,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 172800000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 288000000,
		.linelength = 4800,
		.framelength = 2000,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 172800000,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 288000000,
		.linelength = 4800,
		.framelength = 2000,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 172800000,
		.max_framerate = 300,
	},
	.margin = 16,
	.min_shutter = 4,
	.max_frame_length = 0x27af,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 3,

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
#if GC5025_MIRROR_FLIP_ENABLE
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
#else
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
#endif
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x6e, 0xff},
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3ED,
	.gain = 0x40,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x6e,
};

/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0, 0, 2592, 1944},/* Preview */
	{2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0, 0, 2592, 1944},/* capture */
	{2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0, 0, 2592, 1944},/* video  */
	{2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0, 0, 2592, 1944},/* hs video */
	{2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0, 0, 2592, 1944} /*slim video */
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = {(char)(addr & 0xff)};

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = {(char)(addr & 0xff), (char)(para & 0xff)};

	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}


static struct gc5025_otp gc5025_otp_info;

static kal_uint8 gc5025_read_otp(kal_uint8 addr)
{
	kal_uint8 value;
	kal_uint8 regd4;
	kal_uint16 realaddr = addr * 8;

	regd4 = read_cmos_sensor(0xd4);

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xd4, (regd4 & 0xfc) + ((realaddr >> 8) & 0x03));
	write_cmos_sensor(0xd5, realaddr & 0xff);
	write_cmos_sensor(0xf3, 0x20);
	value = read_cmos_sensor(0xd7);

	return value;
}

static void gc5025_read_otp_group(kal_uint8 addr, kal_uint8 *buff, int size)
{
	kal_uint8 i;
	kal_uint8 regd4;
	kal_uint16 realaddr = addr * 8;

	regd4 = read_cmos_sensor(0xd4);

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xd4, (regd4 & 0xfc) + ((realaddr >> 8) & 0x03));
	write_cmos_sensor(0xd5, realaddr);
	write_cmos_sensor(0xf3, 0x20);
	write_cmos_sensor(0xf3, 0x88);

	for (i = 0; i < size; i++)
		buff[i] = read_cmos_sensor(0xd7);
}

static void gc5025_select_page_otp(kal_uint8 otp_select_page)
{
	kal_uint8 page;

	write_cmos_sensor(0xfe, 0x00);
	page = read_cmos_sensor(0xd4);

	switch (otp_select_page) {
	case otp_page0:
		page = page & 0xfb;
		break;
	case otp_page1:
		page = page | 0x04;
		break;
	default:
		break;
	}

	mdelay(5);
	write_cmos_sensor(0xd4, page);
}

static void gc5025_gcore_read_otp_info(void)
{
	kal_uint8 flagdd, flag_chipversion;
	kal_uint8 i, j, cnt = 0;
	kal_uint8 total_number = 0;
	kal_uint8 ddtempbuff[50 * 4];
#if GC5025OTP_FOR_CUSTOMER
	kal_uint8 flag1, flag_golden, index;
	kal_uint8 info[8];
	kal_uint8 wb[5];
	kal_uint8 golden[5];
#endif

	memset(&gc5025_otp_info, 0, sizeof(gc5025_otp_info));

	/*TODO*/
	gc5025_select_page_otp(otp_page0);
	flagdd = gc5025_read_otp(0x00);
	LOG_INF("flag_dd = 0x%x\n", flagdd);
	flag_chipversion = gc5025_read_otp(0x7f);

	/* DD */
	switch (flagdd & 0x03) {
	case 0x00:
		LOG_INF("DD is empty!\n");
		gc5025_otp_info.dd_flag = 0x00;
		break;
	case 0x01:
		LOG_INF("DD is valid!\n");
		total_number = gc5025_read_otp(0x01) + gc5025_read_otp(0x02);
		LOG_INF("total_number = %d\n", total_number);

		if (total_number <= 31)
			gc5025_read_otp_group(0x03, &ddtempbuff[0], total_number * 4);
		else {
			gc5025_read_otp_group(0x03, &ddtempbuff[0], 31 * 4);
			gc5025_select_page_otp(otp_page1);
			gc5025_read_otp_group(0x29, &ddtempbuff[31 * 4], (total_number - 31) * 4);
		}

		for (i = 0; i < total_number; i++) {
			LOG_INF("index = %d, data[0] = %x, data[1] = %x, data[2] = %x, data[3] = %x\n",
				i, ddtempbuff[4 * i], ddtempbuff[4 * i + 1],
				ddtempbuff[4 * i + 2], ddtempbuff[4 * i + 3]);

			if (ddtempbuff[4 * i + 3] & 0x10) {
				switch (ddtempbuff[4 * i + 3] & 0x0f) {
				case 3:
					for (j = 0; j < 4; j++) {
						gc5025_otp_info.dd_param_x[cnt] =
							(((kal_uint16)ddtempbuff[4 * i + 1] & 0x0f) << 8)
							+ ddtempbuff[4 * i];
						gc5025_otp_info.dd_param_y[cnt] =
							((kal_uint16)ddtempbuff[4 * i + 2] << 4)
							+ ((ddtempbuff[4 * i + 1] & 0xf0) >> 4) + j;
						gc5025_otp_info.dd_param_type[cnt++] = 2;
					}
					break;
				case 4:
					for (j = 0; j < 2; j++) {
						gc5025_otp_info.dd_param_x[cnt] =
							(((kal_uint16)ddtempbuff[4 * i + 1] & 0x0f) << 8)
							+ ddtempbuff[4 * i];
						gc5025_otp_info.dd_param_y[cnt] =
							((kal_uint16)ddtempbuff[4 * i + 2] << 4)
							+ ((ddtempbuff[4 * i + 1] & 0xf0) >> 4) + j;
						gc5025_otp_info.dd_param_type[cnt++] = 2;
					}
					break;
				default:
					gc5025_otp_info.dd_param_x[cnt] =
						(((kal_uint16)ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
					gc5025_otp_info.dd_param_y[cnt] =
						((kal_uint16)ddtempbuff[4 * i + 2] << 4)
						+ ((ddtempbuff[4 * i + 1] & 0xf0) >> 4);
					gc5025_otp_info.dd_param_type[cnt++] = ddtempbuff[4 * i + 3] & 0x0f;
					break;
				}
			} else
				LOG_INF("check_id[%d] = %x,checkid error!\n", i, ddtempbuff[4 * i + 3] & 0xf0);
		}

		gc5025_otp_info.dd_cnt = cnt;
		gc5025_otp_info.dd_flag = 0x01;
		break;
	case 0x02:
	case 0x03:
		LOG_INF("DD is invalid!\n");
		gc5025_otp_info.dd_flag = 0x02;
		break;
	default:
		break;
	}

	/*For Chip Version*/
	LOG_INF("flag_chipversion = 0x%x\n", flag_chipversion);

	switch ((flag_chipversion >> 4) & 0x03) {
	case 0x00:
		LOG_INF("CHIPVERSION is empty!\n");
		break;
	case 0x01:
		LOG_INF("CHIPVERSION is valid!\n");
		gc5025_select_page_otp(otp_page1);
		i = 0;
		do {
			gc5025_otp_info.reg_addr[i] = gc5025_read_otp(REG_ROM_START + i*2);
			gc5025_otp_info.reg_value[i] = gc5025_read_otp(REG_ROM_START + i * 2 + 1);
			LOG_INF("reg_addr[%d] = 0x%x, reg_value[%d] = 0x%x\n",
				i, gc5025_otp_info.reg_addr[i], i, gc5025_otp_info.reg_value[i]);
			i++;
		} while ((gc5025_otp_info.reg_addr[i - 1] != 0) && (i < 10));
		gc5025_otp_info.reg_num = i - 1;
		break;
	case 0x02:
		LOG_INF("CHIPVERSION is invalid!\n");
		break;
	default:
		break;
	}

#if GC5025OTP_FOR_CUSTOMER
	gc5025_select_page_otp(otp_page1);
	flag1 = gc5025_read_otp(0x00);
	flag_golden = gc5025_read_otp(0x1b);
	LOG_INF("flag1 = 0x%x , flag_golden = 0x%x\n", flag1, flag_golden);

	/* INFO & WB */
	for (index = 0; index < 2; index++) {
		switch ((flag1 >> (4 + 2 * index)) & 0x03) {
		case 0x00:
			LOG_INF("module info group %d is empty!\n", index + 1);
			break;
		case 0x01:
			LOG_INF("module info group %d is valid!\n", index + 1);
			check = 0;
			gc5025_read_otp_group(INFO_ROM_START + index * INFO_WIDTH, &info[0], INFO_WIDTH);
			for (i = 0; i < INFO_WIDTH - 1; i++)
				check += info[i];

			if ((check % 255 + 1) == info[INFO_WIDTH-1]) {
				gc5025_otp_info.module_id = info[0];
				gc5025_otp_info.lens_id = info[1];
				gc5025_otp_info.vcm_driver_id = info[2];
				gc5025_otp_info.vcm_id = info[3];
				gc5025_otp_info.year = info[4];
				gc5025_otp_info.month = info[5];
				gc5025_otp_info.day = info[6];
			} else
				LOG_INF("module info checksum %d Error!\n", index + 1);
			break;
		case 0x02:
		case 0x03:
			LOG_INF("module info group %d is invalid!\n", index + 1);
			break;
		default:
			break;
		}

		switch ((flag1 >> (2 * index)) & 0x03) {
		case 0x00:
			LOG_INF("wb group %d is empty!\n", index + 1);
			gc5025_otp_info.wb_flag = gc5025_otp_info.wb_flag | 0x00;
			break;
		case 0x01:
			LOG_INF("wb group %d is valid!\n", index + 1);
			check = 0;
			gc5025_read_otp_group(WB_ROM_START + index * WB_WIDTH, &wb[0], WB_WIDTH);
			for (i = 0; i < WB_WIDTH - 1; i++)
				check += wb[i];

			if ((check % 255 + 1) == wb[WB_WIDTH - 1]) {
				gc5025_otp_info.rg_gain = (((wb[0] << 8) & 0xff00) | wb[1]) > 0 ?
					(((wb[0] << 8) & 0xff00) | wb[1]) : 0x400;
				gc5025_otp_info.bg_gain = (((wb[2] << 8) & 0xff00) | wb[3]) > 0 ?
					(((wb[2] << 8) & 0xff00) | wb[3]) : 0x400;
				gc5025_otp_info.wb_flag = gc5025_otp_info.wb_flag | 0x01;
			} else
				LOG_INF("wb checksum %d error!\n", index + 1);
			break;
		case 0x02:
		case 0x03:
			LOG_INF("wb group %d is invalid!\n", index + 1);
			gc5025_otp_info.wb_flag = gc5025_otp_info.wb_flag | 0x02;
			break;
		default:
			break;
		}

		switch ((flag_golden >> (2 * index)) & 0x03) {
		case 0x00:
			LOG_INF("wb golden group %d is empty!\n", index + 1);
			gc5025_otp_info.golden_flag = gc5025_otp_info.golden_flag | 0x00;
			break;
		case 0x01:
			LOG_INF("wb golden group %d is valid!\n", index + 1);
			check = 0;
			gc5025_read_otp_group(GOLDEN_ROM_START + index * GOLDEN_WIDTH, &golden[0], GOLDEN_WIDTH);
			for (i = 0; i < GOLDEN_WIDTH - 1; i++)
				check += golden[i];

			if ((check % 255 + 1) == golden[GOLDEN_WIDTH - 1]) {
				gc5025_otp_info.golden_rg = (((golden[0] << 8) & 0xff00) | golden[1]) > 0 ?
					(((golden[0] << 8) & 0xff00) | golden[1]) : RG_TYPICAL;
				gc5025_otp_info.golden_bg = (((golden[2] << 8) & 0xff00) | golden[3]) > 0 ?
					(((golden[2] << 8) & 0xff00) | golden[3]) : BG_TYPICAL;
				gc5025_otp_info.golden_flag = gc5025_otp_info.golden_flag | 0x01;
			} else
				LOG_INF("wb golden checksum %d error!\n", index + 1);
			break;
		case 0x02:
		case 0x03:
			LOG_INF("wb golden group %d is invalid!\n", index + 1);
			gc5025_otp_info.golden_flag = gc5025_otp_info.golden_flag | 0x02;
			break;
		default:
			break;
		}
	}

	/* print otp information */
	LOG_INF("module_id = 0x%x\n", gc5025_otp_info.module_id);
	LOG_INF("lens_id = 0x%x\n", gc5025_otp_info.lens_id);
	LOG_INF("vcm_id = 0x%x\n", gc5025_otp_info.vcm_id);
	LOG_INF("vcm_driver_id = 0x%x\n", gc5025_otp_info.vcm_driver_id);
	LOG_INF("data = %d-%d-%d\n", gc5025_otp_info.year, gc5025_otp_info.month, gc5025_otp_info.day);
	LOG_INF("r/g = 0x%x\n", gc5025_otp_info.rg_gain);
	LOG_INF("b/g = 0x%x\n", gc5025_otp_info.bg_gain);
	LOG_INF("golden_rg = 0x%x\n", gc5025_otp_info.golden_rg);
	LOG_INF("golden_bg = 0x%x\n", gc5025_otp_info.golden_bg);
#endif
}

static void gc5025_gcore_update_dd(void)
{
	kal_uint16 i = 0, j = 0;
	kal_uint16 temp_x = 0, temp_y = 0;
	kal_uint8 flag = 0;
	kal_uint8 temp_type = 0;
	kal_uint8 temp_val0, temp_val1, temp_val2;

	if (gc5025_otp_info.dd_flag == 0x01) {
#if GC5025_MIRROR_FLIP_ENABLE
		for (i = 0; i < gc5025_otp_info.dd_cnt; i++) {
			if (gc5025_otp_info.dd_param_type[i] == 0) {
				gc5025_otp_info.dd_param_x[i] = WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i] + 1;
				gc5025_otp_info.dd_param_y[i] = WINDOW_HEIGHT - gc5025_otp_info.dd_param_y[i] + 1;
			} else if (gc5025_otp_info.dd_param_type[i] == 1) {
				gc5025_otp_info.dd_param_x[i] = WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i] - 1;
				gc5025_otp_info.dd_param_y[i] = WINDOW_HEIGHT - gc5025_otp_info.dd_param_y[i] + 1;
			} else {
				gc5025_otp_info.dd_param_x[i] = WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i];
				gc5025_otp_info.dd_param_y[i] = WINDOW_HEIGHT - gc5025_otp_info.dd_param_y[i] + 1;
			}
		}
#else
		/* do nothing */
#endif

		for (i = 0; i < gc5025_otp_info.dd_cnt - 1; i++)
			for (j = i + 1; j < gc5025_otp_info.dd_cnt; j++)
				if (gc5025_otp_info.dd_param_y[i] * WINDOW_WIDTH
					+ gc5025_otp_info.dd_param_x[i]
					> gc5025_otp_info.dd_param_y[j] * WINDOW_WIDTH
					+ gc5025_otp_info.dd_param_x[j]) {
					temp_x = gc5025_otp_info.dd_param_x[i];
					gc5025_otp_info.dd_param_x[i] = gc5025_otp_info.dd_param_x[j];
					gc5025_otp_info.dd_param_x[j] = temp_x;
					temp_y = gc5025_otp_info.dd_param_y[i];
					gc5025_otp_info.dd_param_y[i] = gc5025_otp_info.dd_param_y[j];
					gc5025_otp_info.dd_param_y[j] = temp_y;
					temp_type = gc5025_otp_info.dd_param_type[i];
					gc5025_otp_info.dd_param_type[i] = gc5025_otp_info.dd_param_type[j];
					gc5025_otp_info.dd_param_type[j] = temp_type;
				}

		/* write SRAM */
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0xa8, 0x00);
		write_cmos_sensor(0x9d, 0x04);
		write_cmos_sensor(0xbe, 0x00);
		write_cmos_sensor(0xa9, 0x01);

		for (i = 0; i < gc5025_otp_info.dd_cnt; i++) {
			temp_val0 = gc5025_otp_info.dd_param_x[i] & 0x00ff;
			temp_val1 = (((gc5025_otp_info.dd_param_y[i]) << 4) & 0x00f0)
				+ ((gc5025_otp_info.dd_param_x[i] >> 8) & 0x000f);
			temp_val2 = (gc5025_otp_info.dd_param_y[i] >> 4) & 0xff;
			write_cmos_sensor(0xaa, i);
			write_cmos_sensor(0xac, temp_val0);
			write_cmos_sensor(0xac, temp_val1);
			write_cmos_sensor(0xac, temp_val2);
			while ((i < gc5025_otp_info.dd_cnt - 1)
				&& (gc5025_otp_info.dd_param_x[i] == gc5025_otp_info.dd_param_x[i+1])
				&& (gc5025_otp_info.dd_param_y[i] == gc5025_otp_info.dd_param_y[i+1])) {
				flag = 1;
				i++;
			}
			if (flag)
				write_cmos_sensor(0xac, 0x02);
			else
				write_cmos_sensor(0xac, gc5025_otp_info.dd_param_type[i]);

			LOG_INF("val0 = 0x%x, val1 = 0x%x, val2 = 0x%x\n", temp_val0, temp_val1, temp_val2);
			LOG_INF("x = %d, y = %d\n",
				((temp_val1 & 0x0f) << 8) + temp_val0, (temp_val2 << 4) + ((temp_val1 & 0xf0) >> 4));
		}

		write_cmos_sensor(0xbe, 0x01);
		write_cmos_sensor(0xfe, 0x00);
	}
}

#if GC5025OTP_FOR_CUSTOMER
static void gc5025_gcore_update_wb(void)
{
	kal_uint16 r_gain_current = 0, g_gain_current = 0, b_gain_current = 0, base_gain = 0;
	kal_uint16 r_gain = 1024, g_gain = 1024, b_gain = 1024;
	kal_uint16 rg_typical, bg_typical;

	if (gc5025_otp_info.golden_flag == 0x02)
		return;

	if (0x00 == (gc5025_otp_info.golden_flag & 0x01)) {
		rg_typical = RG_TYPICAL;
		bg_typical = BG_TYPICAL;
	}

	if (0x01 == (gc5025_otp_info.golden_flag & 0x01)) {
		rg_typical = gc5025_otp_info.golden_rg;
		bg_typical = gc5025_otp_info.golden_bg;
		LOG_INF("rg_typical = 0x%x, bg_typical = 0x%x\n", rg_typical, bg_typical);
	}

	if (0x01 == (gc5025_otp_info.wb_flag & 0x01)) {
		r_gain_current = 1024 * rg_typical / gc5025_otp_info.rg_gain;
		b_gain_current = 1024 * bg_typical / gc5025_otp_info.bg_gain;
		g_gain_current = 1024;

		base_gain = (r_gain_current < b_gain_current) ? r_gain_current : b_gain_current;
		base_gain = (base_gain < g_gain_current) ? base_gain : g_gain_current;
		LOG_INF("r_gain_current = 0x%x, b_gain_current = 0x%x, base_gain = 0x%x\n",
			r_gain_current, b_gain_current, base_gain);

		r_gain = 0x400 * r_gain_current / base_gain;
		g_gain = 0x400 * g_gain_current / base_gain;
		b_gain = 0x400 * b_gain_current / base_gain;
		LOG_INF("r_gain = 0x%x, g_gain = 0x%x, b_gain = 0x%x\n", r_gain, g_gain, b_gain);

		/*TODO*/
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0xc6, g_gain >> 3);
		write_cmos_sensor(0xc7, r_gain >> 3);
		write_cmos_sensor(0xc8, b_gain >> 3);
		write_cmos_sensor(0xc9, g_gain >> 3);
		write_cmos_sensor(0xc4, ((g_gain & 0x07) << 4) + (r_gain & 0x07));
		write_cmos_sensor(0xc5, ((b_gain & 0x07) << 4) + (g_gain & 0x07));
	}
}
#endif

static void gc5025_gcore_update_chipversion(void)
{
	kal_uint8 i;

	LOG_INF("reg_num = %d\n", gc5025_otp_info.reg_num);
	for (i = 0; i < gc5025_otp_info.reg_num; i++) {
		write_cmos_sensor(gc5025_otp_info.reg_addr[i], gc5025_otp_info.reg_value[i]);
		LOG_INF("{0x%x, 0x%x}\n", gc5025_otp_info.reg_addr[i], gc5025_otp_info.reg_value[i]);
	}
}

static void gc5025_gcore_update_otp(void)
{
	gc5025_gcore_update_dd();
#if GC5025OTP_FOR_CUSTOMER
	gc5025_gcore_update_wb();
#endif
	gc5025_gcore_update_chipversion();
}

static void gc5025_gcore_enable_otp(kal_uint8 state)
{
	kal_uint8 otp_clk, otp_en;

	otp_clk = read_cmos_sensor(0xfa);
	otp_en = read_cmos_sensor(0xd4);
	if (state) {
		otp_clk = otp_clk | 0x10;
		otp_en = otp_en | 0x80;
		mdelay(5);
		write_cmos_sensor(0xfa, otp_clk);
		write_cmos_sensor(0xd4, otp_en);
		LOG_INF("Enable OTP!\n");
	} else {
		otp_en = otp_en & 0x7f;
		otp_clk = otp_clk & 0xef;
		mdelay(5);
		write_cmos_sensor(0xd4, otp_en);
		write_cmos_sensor(0xfa, otp_clk);
		LOG_INF("Disable OTP!\n");
	}
}


static void gc5025_gcore_initial_otp(void)
{
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xf7, 0x01);
	write_cmos_sensor(0xf8, 0x11);
	write_cmos_sensor(0xf9, 0x00);
	write_cmos_sensor(0xfa, 0xa0);
	write_cmos_sensor(0xfc, 0x2e);
}

static void gc5025_gcore_identify_otp(void)
{
	gc5025_gcore_initial_otp();
	gc5025_gcore_enable_otp(otp_open);
	gc5025_gcore_read_otp_info();
	gc5025_gcore_enable_otp(otp_close);
}

static int gc5025_gcore_check_version(void)
{
	kal_uint8 flag_doublereset = 0, flag_GC5025A = 0;

	gc5025_gcore_initial_otp();
	gc5025_gcore_enable_otp(otp_open);
	gc5025_select_page_otp(otp_page1);

	flag_doublereset = gc5025_read_otp(0x26);
	flag_GC5025A = gc5025_read_otp(0x27);

	if ((flag_GC5025A & 0x01) == 0x01) {
		LOG_INF("GC5025A sensor!\n");
		DR_State = KAL_FALSE;
	} else {
		if ((flag_doublereset & 0x03) == 0x01) {
			DR_State = KAL_FALSE;
			LOG_INF("GC5025 sensor double reset off\n");
		} else {
			DR_State = KAL_TRUE;
			LOG_INF("GC5025 sensor double reset on\n");
		}
	}

	gc5025_gcore_enable_otp(otp_close);

	return 0;
}

static void set_dummy(void)
{
	kal_uint32 vb = 32;
	kal_uint32 basic_line = 1968;

	vb = imgsensor.frame_length - basic_line;
	vb = vb / 2;
	vb = vb * 2;
	vb = (vb < 32) ? 32 : vb;

	write_cmos_sensor(0x07, (vb >> 8) & 0x1f);
	write_cmos_sensor(0x08, vb & 0xff);
}

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1));
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/* kal_int16 dummy_line;*/
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)
		? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}

static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0, cal_shutter = 0;
	/* kal_uint32 frame_length = 0; */
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

/*	if (imgsensor.autoflicker_en) {
*		if (realtime_fps >= 297 && realtime_fps <= 305)
*			set_max_framerate(296, 0);
*		else if (realtime_fps >= 147 && realtime_fps <= 150)
*			set_max_framerate(146, 0);
*		else
*			set_max_framerate(realtime_fps, 0);
*	} else
*		set_max_framerate(realtime_fps, 0);
*/
	cal_shutter = shutter / 2;
	cal_shutter = cal_shutter * 2;
	Dgain_ratio = 256 * shutter / cal_shutter;

	if (!DR_State) {
		if (cal_shutter <= 10) {
			write_cmos_sensor(0xfe, 0x00);
			write_cmos_sensor(0xd9, 0xdd);
		} else {
			write_cmos_sensor(0xfe, 0x00);
			write_cmos_sensor(0xd9, 0xaa);
		}
	}

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x03, (cal_shutter >> 8) & 0x3f);
	write_cmos_sensor(0x04, cal_shutter & 0xff);

	LOG_INF("Exit! shutter = %d, framelength = %d\n", shutter, imgsensor.frame_length);
}

/*
*static kal_uint16 gain2reg(const kal_uint16 gain)
*{
*    kal_uint16 reg_gain = 0x0000;
*
*    reg_gain = ((gain / BASEGAIN) << 4) + ((gain % BASEGAIN) * 16 / BASEGAIN);
*    reg_gain = reg_gain & 0xFFFF;
*    return (kal_uint16)reg_gain;
*}
*/

#define ANALOG_GAIN_1  64 /* 1.000x */
#define ANALOG_GAIN_2  92 /* 1.445x */

static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 iReg, temp;

	iReg = gain;

	if (iReg < 0x40)
		iReg = 0x40;

	if ((iReg >= ANALOG_GAIN_1) && (iReg < ANALOG_GAIN_2)) {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0xb6, 0x00);
		temp = iReg * Dgain_ratio / 256;
		write_cmos_sensor(0xb1, temp >> 6);
		write_cmos_sensor(0xb2, (temp << 2) & 0xfc);
		LOG_INF("analogic gain 1x, add pregain = %d\n", temp);
	} else {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0xb6, 0x01);
		temp = 64 * iReg / ANALOG_GAIN_2;
		temp = temp * Dgain_ratio / 256;
		write_cmos_sensor(0xb1, temp >> 6);
		write_cmos_sensor(0xb2, (temp << 2) & 0xfc);
		LOG_INF("analogic gain 1.4x, add pregain = %d\n", temp);
	}

	return gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le: 0x%x, se: 0x%x, gain: 0x%x\n", le, se, gain);
}


/*static void set_mirror_flip(kal_uint8 image_mirror)
*{
*	LOG_INF("image_mirror = %d\n", image_mirror);
*}
*/

static void night_mode(kal_bool enable)
{
	/* No Need to implement this function */
}

static void sensor_init(void)
{
	LOG_INF("E\n");
	/* SYS */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xf7, 0x01);
	write_cmos_sensor(0xf8, 0x11);
	write_cmos_sensor(0xf9, 0x00);
	write_cmos_sensor(0xfa, 0xa0);
	write_cmos_sensor(0xfc, 0x2a);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x07);
	write_cmos_sensor(0xfc, 0x2e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x88, 0x03);
	write_cmos_sensor(0x3f, 0x00);

    /*Cisctl&Analog*/
	write_cmos_sensor(0x03, 0x07);
	write_cmos_sensor(0x04, 0x08);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0x58);
	write_cmos_sensor(0x08, 0x20);
	write_cmos_sensor(0x0a, 0x1c);
	write_cmos_sensor(0x0c, 0x04);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0x9c);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x17, GC5025_MIRROR);
	write_cmos_sensor(0x18, 0x02);
	write_cmos_sensor(0x19, 0x17);
	write_cmos_sensor(0x1a, 0x1a);
	write_cmos_sensor(0x1e, 0x90);
	write_cmos_sensor(0x1f, 0xb0);
	write_cmos_sensor(0x20, 0x2b);
	write_cmos_sensor(0x21, 0x2b);
	write_cmos_sensor(0x26, 0x2b);
	write_cmos_sensor(0x25, 0xc1);
	write_cmos_sensor(0x27, 0x64);
	write_cmos_sensor(0x28, 0x00);
	write_cmos_sensor(0x29, 0x3f);
	write_cmos_sensor(0x2b, 0x80);
	write_cmos_sensor(0x30, 0x11);
	write_cmos_sensor(0x31, 0x20);
	write_cmos_sensor(0x32, 0xa0);
	write_cmos_sensor(0x33, 0x00);
	write_cmos_sensor(0x34, 0x55);
	write_cmos_sensor(0x3a, 0x00);
	write_cmos_sensor(0x3b, 0x00);
	write_cmos_sensor(0x81, 0x60);
	write_cmos_sensor(0xcb, 0x02);
	write_cmos_sensor(0xcd, 0x2d);
	write_cmos_sensor(0xcf, 0x50);
	write_cmos_sensor(0xd0, 0xb3);
	write_cmos_sensor(0xd1, 0x18);
	write_cmos_sensor(0xd9, 0xaa);
	write_cmos_sensor(0xdc, 0x03);
	write_cmos_sensor(0xdd, 0xaa);
	write_cmos_sensor(0xe0, 0x00);
	write_cmos_sensor(0xe1, 0x0a);
	write_cmos_sensor(0xe3, 0x2a);
	write_cmos_sensor(0xe4, 0xa0);
	write_cmos_sensor(0xe5, 0x06);
	write_cmos_sensor(0xe6, 0x10);
	write_cmos_sensor(0xe7, 0xc2);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);

	if (DR_State) {
		write_cmos_sensor(0x1c, 0x1c);
		write_cmos_sensor(0x2f, 0x4a);
		write_cmos_sensor(0x38, 0x02);
		write_cmos_sensor(0x39, 0x00);
		write_cmos_sensor(0x3c, 0x02);
		write_cmos_sensor(0x3d, 0x02);
		write_cmos_sensor(0xd3, 0xcc);
		write_cmos_sensor(0x43, 0x03);
		write_cmos_sensor(0x1d, 0x13);
	} else {
		write_cmos_sensor(0x1c, 0x2c);
		write_cmos_sensor(0x2f, 0x4d);
		write_cmos_sensor(0x38, 0x04);
		write_cmos_sensor(0x39, 0x02);
		write_cmos_sensor(0x3c, 0x08);
		write_cmos_sensor(0x3d, 0x0f);
		write_cmos_sensor(0xd3, 0xc4);
		write_cmos_sensor(0x43, 0x08);
		write_cmos_sensor(0x1d, 0x00);
	}

	/* ISP */
	write_cmos_sensor(0x80, 0x10);
	write_cmos_sensor(0x89, 0x03);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x88, 0xf7);
	write_cmos_sensor(0x8a, 0x03);
	write_cmos_sensor(0x8e, 0xc7);

	/* BLK */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x40, 0x22);
	write_cmos_sensor(0x41, 0x28);
	write_cmos_sensor(0x42, 0x04);
	write_cmos_sensor(0x4e, 0x0f);
	write_cmos_sensor(0x4f, 0xf0);
	write_cmos_sensor(0x67, 0x0c);
	write_cmos_sensor(0xae, 0x40);
	write_cmos_sensor(0xaf, 0x04);
	write_cmos_sensor(0x60, 0x00);
	write_cmos_sensor(0x61, 0x80);

	/* gain */
	write_cmos_sensor(0xb0, 0x58);
	write_cmos_sensor(0xb1, 0x01);
	write_cmos_sensor(0xb2, 0x00);
	write_cmos_sensor(0xb6, 0x00);

	/* Crop window */
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, GC5025_STARTY);
	write_cmos_sensor(0x94, GC5025_STARTX);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x03);
	write_cmos_sensor(0x03, 0x8e);
	write_cmos_sensor(0x06, 0x80);
	write_cmos_sensor(0x15, 0x00);
	write_cmos_sensor(0x16, 0x09);
	write_cmos_sensor(0x18, 0x0a);
	write_cmos_sensor(0x21, 0x10);
	write_cmos_sensor(0x22, 0x05);
	write_cmos_sensor(0x23, 0x20);
	write_cmos_sensor(0x24, 0x02);
	write_cmos_sensor(0x25, 0x20);
	write_cmos_sensor(0x26, 0x08);
	write_cmos_sensor(0x29, 0x06);
	write_cmos_sensor(0x2a, 0x0a);
	write_cmos_sensor(0x2b, 0x08);
	write_cmos_sensor(0xfe, 0x00);
}

static void preview_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3f, 0x91);
}

static void capture_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3f, 0x91);
}

static void normal_video_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3f, 0x91);
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3f, 0x91);
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3f, 0x91);
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x8c, 0x11);
	} else {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x8c, 0x10);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
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

static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
	    retry = 2;
	}

	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	gc5025_gcore_check_version();

	/* Don't Remove!! */
	gc5025_gcore_identify_otp();

	/* initail sequence write in */
	sensor_init();

	/* write registers to sram */
	gc5025_gcore_update_otp();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	GC5025DuringTestPattern = KAL_FALSE;

	return ERROR_NONE;
}

static kal_uint32 close(void)
{
	LOG_INF("E\n");
	/* No Need to implement this function */
	return ERROR_NONE;
}

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting();
	return ERROR_NONE;
}

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
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	return ERROR_NONE;
}

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
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	return ERROR_NONE;
}

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;
	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;
	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;
	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}

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
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

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
	sensor_info->SensorHightSampling = 0;  /* 0 is default 1x */
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
		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
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
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n", framerate);
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

static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length =
			imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
			(frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ?
				(frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
				(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
			(frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
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
	default:
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
	UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *)feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
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
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;

			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				rate = imgsensor_info.cap.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				rate = imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				rate = imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				rate = imgsensor_info.slim_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				rate = imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		LOG_INF("adb_i2c_read 0x%x = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
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
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps: %d\n", (UINT32)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable: %d\n", (BOOL)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (BOOL)*feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId: %d\n", (UINT32)*feature_data);
		wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE = %d, SE = %d, Gain = %d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));
		ihdr_write_shutter_gain((UINT16)*feature_data, (UINT16)*(feature_data + 1),
			(UINT16)*(feature_data + 2));
		break;
	default:
		break;
	}
	return ERROR_NONE;
}

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 GC5025MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
