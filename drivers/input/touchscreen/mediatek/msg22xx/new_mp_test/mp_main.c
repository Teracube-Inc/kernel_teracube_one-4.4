/*
 * Copyright (C) 2006-2017 ILITEK TECHNOLOGY CORP.
 *
 * Description: ILITEK I2C touchscreen driver for linux platform.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Author: Dicky Chiang
 * Maintain: Luca Hsu, Tigers Huang
 */

#include "../ilitek_drv_common.h"
#include "../ilitek_drv_mp_test.h"
#include "mp_common.h"

#define RUN_OPEN_TEST	0
#define RUN_SHORT_TEST	1

extern struct mutex g_Mutex;

MutualMpTest_t *mp_test_data;
MutualMpTestResult_t *mp_test_result;
struct mp_main_func *p_mp_func;

int ana_count = 0;
u8 ana_ver[100] = {0};
int mapping_barker_ini = 0;
u32 g_IsInMpTest = 0;

static void mp_calc_golden_range(int *goldenTbl, u16 weight, u16 weight_up, int *maxTbl, int *minTbl, int length)
{
    int i, value = 0, value_up = 0;

    for (i = 0; i < length; i++) {
    	value = (int)weight * abs(goldenTbl[i]) / 100;
        value_up = (int)weight_up * abs(goldenTbl[i]) / 100;

        maxTbl[i] = goldenTbl[i] + value + value_up;
        minTbl[i] = goldenTbl[i] - value;
    }
}

// static u16 mp_get_fw_info(void)
// {
// 	u8 *info_fw_ver;
// 	int official_ver = 0;
// 	u8 nChipType = 0;

// 	mp_info("*** %s() ***\n", __func__);

//     info_fw_ver = (u8 *)kcalloc(1, 4, GFP_KERNEL);
//     if(ERR_ALLOC_MEM(info_fw_ver)) {
//         mp_err("Failed to allocate info_fw mem\n");
//         return -1;
//     }

// 	EnterDBBus();
//     StopMCU();

// 	nChipType = RegGet16BitValue(0x1ECC) & 0xFF;
// 	ReadFlash(nChipType,0x7f8,EMEM_TYPE_INFO_BLOCK,4,info_fw_ver);

// 	mp_info("info_fw_ver[0] = %x, info_fw_ver[1] = %x, info_fw_ver[2] = %x, info_fw_ver[3] = %x \n",
//             info_fw_ver[0], info_fw_ver[1], info_fw_ver[2], info_fw_ver[3]);

// 	info_fw_ver[1] = 0;

// 	official_ver = str_to_hex((char *) info_fw_ver);
// 	kfree(info_fw_ver);
// 	StartMCU();
// 	return (u16)official_ver;
// }

static int mp_load_ini(char * pFilePath)
{
	int res = 0, nSize = 0;
	char *token = NULL, str[512]={0};
	long s_to_long = 0;

	mp_info("*** %s() ***\n", __func__);

	if(mp_parse(pFilePath) < 0) {
         mp_err("Failed to parse file = %s\n", pFilePath);
         return -1;		
	}

    mp_info("Parsed %s successfully!\n", pFilePath);

    mp_test_data = kcalloc(1, sizeof(*mp_test_data), GFP_KERNEL);
    mp_test_result = kcalloc(1, sizeof(*mp_test_result), GFP_KERNEL);
	if(ERR_ALLOC_MEM(mp_test_result) || ERR_ALLOC_MEM(mp_test_data)) {
		pr_err("Failed to allocate mp_test mem \n");
		return -1;
	}

	token = kmalloc(100, GFP_KERNEL);
	ms_get_ini_data("INFOMATION", "MAPPING_BARKER_INI",str);
	mapping_barker_ini = ms_atoi(str);
	mp_info(" mapping_barker_ini = %d \n", mapping_barker_ini);

	ms_get_ini_data("UI_CONFIG","OpenTest",str);
	mp_test_data->UIConfig.bOpenTest = ms_atoi(str);
	ms_get_ini_data("UI_CONFIG","ShortTest",str);
	mp_test_data->UIConfig.bShortTest = ms_atoi(str);
	ms_get_ini_data("UI_CONFIG","WpTest",str);
	mp_test_data->UIConfig.bWpTest = ms_atoi(str);

    mp_test_data->ana_version = kmalloc(FILENAME_MAX * sizeof(char), GFP_KERNEL);
	if(ERR_ALLOC_MEM(mp_test_data->ana_version)) {
		pr_err("Failed to allocate Ana mem \n");
		return -1;
	}

	ms_get_ini_data("UI_CONFIG","ANAGEN_VER", str);
	strcpy(mp_test_data->ana_version, str);
	ana_count = ms_ini_split_u8_array(mp_test_data->ana_version, ana_ver);
	mp_info("Ana count = %d , mem = %p\n", ana_count, mp_test_data->ana_version);

	ms_get_ini_data("SENSOR","DrvNum",str);
	mp_test_data->sensorInfo.numDrv = ms_atoi(str);
	ms_get_ini_data("SENSOR","SenNum",str);
	mp_test_data->sensorInfo.numSen = ms_atoi(str);
	ms_get_ini_data("SENSOR","KeyNum",str);
	mp_test_data->sensorInfo.numKey = ms_atoi(str);
	ms_get_ini_data("SENSOR","KeyLine",str);
	mp_test_data->sensorInfo.numKeyLine = ms_atoi(str);
	ms_get_ini_data("SENSOR","GrNum",str);
	mp_test_data->sensorInfo.numGr = ms_atoi(str);

	ms_get_ini_data("OPEN_TEST_N","CSUB_REF",str);
	mp_test_data->Open_test_csub = ms_atoi(str);
	ms_get_ini_data("OPEN_TEST_N","CFB_REF",str);
	mp_test_data->Open_test_cfb = ms_atoi(str);
	ms_get_ini_data("OPEN_TEST_N","OPEN_MODE",str);
	mp_test_data->Open_mode = ms_atoi(str);
	ms_get_ini_data("OPEN_TEST_N","FIXED_CARRIER",str);
	mp_test_data->Open_fixed_carrier = ms_atoi(str);
	ms_get_ini_data("OPEN_TEST_N","FIXED_CARRIER1",str);
	mp_test_data->Open_fixed_carrier1 = ms_atoi(str);
	ms_get_ini_data("OPEN_TEST_N","CHARGE_PUMP",str);
	mp_test_data->Open_test_chargepump = ms_atoi(str);

	ms_get_ini_data("INFOMATION","MutualKey",str);
	mp_test_data->Mutual_Key = ms_atoi(str);
	ms_get_ini_data("INFOMATION","Pattern_type",str);
	mp_test_data->Pattern_type = ms_atoi(str);
	ms_get_ini_data("INFOMATION","1T2R_MODEL",str);
	mp_test_data->Pattern_model = ms_atoi(str);

	ms_get_ini_data("RULES","DC_Range",str);
	mp_test_data->ToastInfo.persentDC_VA_Range = ms_atoi(str);
	ms_get_ini_data("RULES","DC_Range_UP",str);
	mp_test_data->ToastInfo.persentDC_VA_Range_up = ms_atoi(str);
	ms_get_ini_data("RULES","DC_Ratio",str);
	mp_test_data->ToastInfo.persentDC_VA_Ratio = ms_atoi(str);
	ms_get_ini_data("RULES","DC_Ratio_UP",str);
	mp_test_data->ToastInfo.persentDC_VA_Ratio_up = ms_atoi(str);
	ms_get_ini_data("RULES","DC_Border_Ratio",str);
	mp_test_data->ToastInfo.persentDC_Border_Ratio = ms_atoi(str);
	ms_get_ini_data("RULES","opentestmode",str);
	mp_test_data->Open_test_mode = ms_atoi(str);
	ms_get_ini_data("RULES","shorttestmode",str);
	mp_test_data->Short_test_mode = ms_atoi(str);

	ms_get_ini_data("BASIC","DEEP_STANDBY",str);
	mp_test_data->deep_standby = ms_atoi(str);

	ms_get_ini_data("BASIC","DEEP_STANDBY_TIMEOUT",str);
	mp_test_data->deep_standby_timeout = ms_atoi(str);

    if ((mp_test_data->Mutual_Key == 1) && (mp_test_data->Mutual_Key == 2)) {
		ms_get_ini_data("SENSOR","KEY_CH",str);
		mp_test_data->sensorInfo.KEY_CH = ms_atoi(str);
	}

	ms_get_ini_data("OPEN_TEST_N", "KEY_SETTING_BY_FW", str);
	mp_test_data->Open_KeySettingByFW = ms_atoi(str);
	ms_get_ini_data("OPEN_TEST_N", "INVERT_MODE", str);
	mp_test_data->inverter_mode = ms_atoi(str);

	ms_get_ini_data("CDTIME", "OPEN_CHARGE", str);
	mp_test_data->OPEN_Charge = ms_atoi(str);
	strcpy(token, str);
	res = kstrtol((const char *)token, 0, &s_to_long);
	if(res == 0)
	 	mp_test_data->OPEN_Charge = s_to_long;	

	ms_get_ini_data("CDTIME", "OPEN_DUMP", str);
	mp_test_data->OPEN_Dump = ms_atoi(str);
	strcpy(token, str);
	res = kstrtol((const char *)token, 0, &s_to_long);
	if(res == 0)
	 	mp_test_data->OPEN_Dump = s_to_long;


	ms_get_ini_data("CDTIME", "SHORT_Charge", str);
	mp_test_data->SHORT_Charge = ms_atoi(str);
	strcpy(token, str);
	res = kstrtol((const char *)token, 0, &s_to_long);
	if(res == 0)
	 	mp_test_data->SHORT_Charge = s_to_long;

	ms_get_ini_data("CDTIME", "SHORT_Dump1", str);
	mp_test_data->SHORT_Dump1 = ms_atoi(str);
	strcpy(token, str);
	res = kstrtol((const char *)token, 0, &s_to_long);
	if(res == 0)
	 	mp_test_data->SHORT_Dump1 = s_to_long;

    mp_info("ANAGEN_VER:    [%s]\n", mp_test_data->ana_version);
    mp_info("OpenTest:      [%d]\n", mp_test_data->UIConfig.bOpenTest);
    mp_info("ShortTest:      [%d]\n", mp_test_data->UIConfig.bShortTest);
    mp_info("WPTest:      [%d]\n", mp_test_data->UIConfig.bWpTest);
    mp_info("DrvNum:      [%d]\n", mp_test_data->sensorInfo.numDrv);
    mp_info("SenNum:      [%d]\n", mp_test_data->sensorInfo.numSen);
    mp_info("KeyNum:      [%d]\n", mp_test_data->sensorInfo.numKey);
    mp_info("KeyLine:      [%d]\n", mp_test_data->sensorInfo.numKeyLine);
    mp_info("DEEP_STANDBY = [%d] \n", mp_test_data->deep_standby);
    mp_info("GrNum:      [%d]\n", mp_test_data->sensorInfo.numGr);
    mp_info("CSUB_REF:      [%d]\n", mp_test_data->Open_test_csub);
    mp_info("CFB_REF:      [%d]\n", mp_test_data->Open_test_cfb);
    mp_info("OPEN_MODE:      [%d]\n", mp_test_data->Open_mode);
    mp_info("FIXED_CARRIER:      [%d]\n", mp_test_data->Open_fixed_carrier);
    mp_info("FIXED_CARRIER1:      [%d]\n", mp_test_data->Open_fixed_carrier1);
    mp_info("CHARGE_PUMP:      [%d]\n", mp_test_data->Open_test_chargepump);
    mp_info("MutualKey:      [%d]\n", mp_test_data->Mutual_Key);
    mp_info("KEY_CH:      [%d]\n", mp_test_data->sensorInfo.KEY_CH);
    mp_info("Pattern_type:      [%d]\n", mp_test_data->Pattern_type);
    mp_info("Pattern_model:      [%d]\n", mp_test_data->Pattern_model);
    mp_info("DC_Range:      [%d]\n", mp_test_data->ToastInfo.persentDC_VA_Range);
    mp_info("DC_Ratio:      [%d]\n", mp_test_data->ToastInfo.persentDC_VA_Ratio);
    mp_info("DC_Range_up:      [%d]\n", mp_test_data->ToastInfo.persentDC_VA_Range_up);
    mp_info("DC_Ratio_up:      [%d]\n", mp_test_data->ToastInfo.persentDC_VA_Ratio_up);
    mp_info("KEY_SETTING_BY_FW:      [%d]\n", mp_test_data->Open_KeySettingByFW);
    mp_info("INVERT_MODE:      [%d]\n", mp_test_data->inverter_mode);
    mp_info("DEEP_STANDBY_TIMEOUT:      [%d]\n", mp_test_data->deep_standby_timeout);
    mp_info("OPEN_CHARGE:      [%d]\n", mp_test_data->OPEN_Charge);
    mp_info("OPEN_DUMP:      [%d]\n", mp_test_data->OPEN_Dump);
    mp_info("SHORT_Charge:      [%d]\n", mp_test_data->SHORT_Charge);
    mp_info("SHORT_Dump1:      [%d]\n", mp_test_data->SHORT_Dump1);
	
    if(mp_test_data->sensorInfo.numKey > 0) {
		ms_get_ini_data("SENSOR","KeyDrv_o",str);
		mp_test_data->sensorInfo.KeyDrv_o = ms_atoi(str);

		ms_get_ini_data("SENSOR","KEYSEN",str);
		mp_test_data->KeySen = kcalloc(mp_test_data->sensorInfo.numKey, sizeof(int), GFP_KERNEL);
        if(ERR_ALLOC_MEM(mp_test_data->KeySen)) {
			mp_err("Failed to allocate mp_test_data->KeySen mem\n");
			return -1;
		}

		ms_ini_split_int_array(str, mp_test_data->KeySen);		

		ms_get_ini_data("SENSOR","KEY_TYPE",str);
		mp_test_data->sensorInfo.key_type = kmalloc(64 * sizeof(char), GFP_KERNEL);
		if(ERR_ALLOC_MEM(mp_test_data->sensorInfo.key_type)) {
			mp_err("Failed to allocate mp_test_data->sensorInfo.key_type mem\n");
			return -1;
		}

		strcpy(mp_test_data->sensorInfo.key_type, str);
    }

	mp_test_data->UIConfig.sSupportIC = kmalloc(FILENAME_MAX * sizeof(char), GFP_KERNEL);

	if(ERR_ALLOC_MEM(mp_test_data->UIConfig.sSupportIC)) {
		mp_err("Failed to allocate mp_test_data->UIConfig.sSupportIC mem\n");
		return -1;
	}

	memset(mp_test_data->UIConfig.sSupportIC, 0, FILENAME_MAX * sizeof(char));
	if(ms_get_ini_data("UI_CONFIG","SupportIC",str) != 0)
		strcpy(mp_test_data->UIConfig.sSupportIC, str);
	
	mp_info("SupportIC:      [%s]\n", mp_test_data->UIConfig.sSupportIC);

 	mp_test_data->project_name = (char *)kmalloc(FILENAME_MAX * sizeof(char), GFP_KERNEL);
	if(ERR_ALLOC_MEM(mp_test_data->UIConfig.sSupportIC))
	{
		mp_err("Failed to allocate mp_test_data->project_name mem\n");
		return -1;
	}

	memset(mp_test_data->project_name, 0, FILENAME_MAX * sizeof(char));
	if(ms_get_ini_data("INFOMATION", "PROJECT", str) != 0)
		strcpy(mp_test_data->project_name, str);

	mp_info("PROJECT:      [%s]\n", mp_test_data->project_name);

    mp_test_data->Goldensample_CH_0 = (int *)kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	if(ERR_ALLOC_MEM(mp_test_data->Goldensample_CH_0)) {
		mp_err("Failed to allocate mp_test_data->Goldensample_CH_0 mem\n");
		return -1;
	}

	nSize = ms_ini_split_golden(mp_test_data->Goldensample_CH_0, mp_test_data->sensorInfo.numSen);
	mp_info("The number of Golden line = %d\n",nSize);

	mp_test_data->Goldensample_CH_0_Max = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL); 
	mp_test_data->Goldensample_CH_0_Max_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	mp_test_data->Goldensample_CH_0_Min = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	mp_test_data->Goldensample_CH_0_Min_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	if(ERR_ALLOC_MEM(mp_test_data->Goldensample_CH_0_Max) || ERR_ALLOC_MEM(mp_test_data->Goldensample_CH_0_Max_Avg) ||
			 ERR_ALLOC_MEM(mp_test_data->Goldensample_CH_0_Min)|| ERR_ALLOC_MEM(mp_test_data->Goldensample_CH_0_Min_Avg))
	{
		mp_err("Failed to allocate Goldensample mem\n");
		return -1;
	}

	if (mp_test_data->sensorInfo.numDrv && mp_test_data->sensorInfo.numSen) {

		mp_test_data->PAD2Drive = kmalloc(mp_test_data->sensorInfo.numDrv * sizeof(u16), GFP_KERNEL);
		if(ERR_ALLOC_MEM(mp_test_data->PAD2Drive)) {
			mp_err("Failed to allocate PAD2Drive mem\n");
			return -1;
		}

		ms_get_ini_data("PAD_TABLE","DRIVE",str);
		mp_info("PAD_TABLE(DRIVE):      [%s]\n", str);
		p_mp_func->drive_len = ms_ini_split_u16_array(str, mp_test_data->PAD2Drive);

		mp_test_data->PAD2Sense = kmalloc(mp_test_data->sensorInfo.numSen * sizeof(u16), GFP_KERNEL);
		if(ERR_ALLOC_MEM(mp_test_data->PAD2Sense)) {
			mp_err("Failed to allocate PAD2Sense mem\n");
			return -1;
		}

		ms_get_ini_data("PAD_TABLE","SENSE",str);
		mp_info("PAD_TABLE(SENSE):      [%s]\n", str);
		p_mp_func->sense_len = ms_ini_split_u16_array(str, mp_test_data->PAD2Sense);
	}

	if (mp_test_data->sensorInfo.numGr) {
		mp_test_data->PAD2GR = kmalloc(mp_test_data->sensorInfo.numGr * sizeof(u16), GFP_KERNEL);
		if(ERR_ALLOC_MEM(mp_test_data->PAD2GR)) {
			mp_err("Failed to allocate PAD2GR mem\n");
			return -1;
		}

		ms_get_ini_data("PAD_TABLE","GR",str);
		printk("PAD_TABLE(GR):      [%s]\n", str);
		p_mp_func->gr_len = ms_ini_split_u16_array(str, mp_test_data->PAD2GR);
	}

	ms_get_ini_data("RULES","SHORTVALUE",str);
	mp_test_data->sensorInfo.thrsShort = ms_atoi(str);
	mp_info("SHORTVALUE:      [%d]\n", mp_test_data->sensorInfo.thrsShort);

	ms_get_ini_data("RULES","ICPINSHORT",str);
	mp_test_data->sensorInfo.thrsICpin = ms_atoi(str);
	mp_info("ICPINSHORT:      [%d]\n", mp_test_data->sensorInfo.thrsICpin);

	mp_kfree(token);
	mp_info("MEM free token\n");
	return 0;
}

static int mp_main_init_var(void)
{
	mp_calc_golden_range(mp_test_data->Goldensample_CH_0,
		mp_test_data->ToastInfo.persentDC_VA_Range, mp_test_data->ToastInfo.persentDC_VA_Range_up,
		mp_test_data->Goldensample_CH_0_Max, mp_test_data->Goldensample_CH_0_Min, MAX_MUTUAL_NUM);

	mp_test_result->nRatioAvg_max = (int) (100 + mp_test_data->ToastInfo.persentDC_VA_Ratio + mp_test_data->ToastInfo.persentDC_VA_Ratio_up) / 100;
	mp_test_result->nRatioAvg_min =(int) (100 - mp_test_data->ToastInfo.persentDC_VA_Ratio) / 100;
	mp_test_result->nBorder_RatioAvg_max = (int) (100 + mp_test_data->ToastInfo.persentDC_Border_Ratio + mp_test_data->ToastInfo.persentDC_VA_Ratio_up) / 100;
	mp_test_result->nBorder_RatioAvg_min = (int) (100 - mp_test_data->ToastInfo.persentDC_Border_Ratio) / 100;

	mp_test_result->pCheck_Fail =               kcalloc(TEST_ITEM_NUM, sizeof(int), GFP_KERNEL);
	mp_test_result->pOpenResultData =           kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	mp_test_result->pOpenFailChannel =          kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	mp_test_result->pOpenRatioFailChannel =     kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);

	mp_test_result->pGolden_CH =                kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	mp_test_result->pGolden_CH_Max =            kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	mp_test_result->pGolden_CH_Max_Avg =        kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	mp_test_result->pGolden_CH_Min =            kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	mp_test_result->pGolden_CH_Min_Avg =        kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);

	mp_test_result->pShortFailChannel =         kcalloc(p_mp_func->max_channel_num, sizeof(int), GFP_KERNEL);
	mp_test_result->pShortResultData =          kcalloc(p_mp_func->max_channel_num, sizeof(int), GFP_KERNEL);
	mp_test_result->pICPinChannel =             kcalloc(p_mp_func->max_channel_num, sizeof(int), GFP_KERNEL);
	mp_test_result->pICPinShortFailChannel =    kcalloc(p_mp_func->max_channel_num, sizeof(int), GFP_KERNEL);
	mp_test_result->pICPinShortResultData =     kcalloc(p_mp_func->max_channel_num, sizeof(int), GFP_KERNEL);

	mp_test_result->pICPinShortRData =          kcalloc(p_mp_func->max_channel_num, sizeof(int), GFP_KERNEL);
	mp_test_result->pShortRData =               kcalloc(p_mp_func->max_channel_num, sizeof(int), GFP_KERNEL);
    /* Check allocated memory status  */
	if(ERR_ALLOC_MEM(mp_test_result->pCheck_Fail) || ERR_ALLOC_MEM(mp_test_result->pOpenResultData) ||
			 ERR_ALLOC_MEM(mp_test_result->pOpenFailChannel) || ERR_ALLOC_MEM(mp_test_result->pOpenRatioFailChannel))
	{
		mp_err("Failed to allocate channels' mem\n");
		return -1;
	}

	if(ERR_ALLOC_MEM(mp_test_result->pGolden_CH) || ERR_ALLOC_MEM(mp_test_result->pGolden_CH_Max) ||
	    ERR_ALLOC_MEM(mp_test_result->pGolden_CH_Max_Avg)|| ERR_ALLOC_MEM(mp_test_result->pGolden_CH_Min) ||
		ERR_ALLOC_MEM(mp_test_result->pGolden_CH_Min_Avg))
	{
		mp_err("Failed to allocate pGolden_CH' mem\n");
		return -1;
	}

	if(ERR_ALLOC_MEM(mp_test_result->pShortFailChannel) || ERR_ALLOC_MEM(mp_test_result->pShortResultData) ||
	    ERR_ALLOC_MEM(mp_test_result->pICPinChannel)|| ERR_ALLOC_MEM(mp_test_result->pICPinShortFailChannel) ||
		ERR_ALLOC_MEM(mp_test_result->pICPinShortResultData))
	{
		mp_err("Failed to allocate pShortFailChannel' mem\n");
		return -1;
	}

	if(ERR_ALLOC_MEM(mp_test_result->pICPinShortRData) || ERR_ALLOC_MEM(mp_test_result->pShortRData))
	{
		mp_err("Failed to allocate pICPinShortRData' mem\n");
		return -1;
	}

	return 0;
}

static int mp_start_test(void)
{
	int i, res = 0;
	//u16 fw_ver = 0;

	mp_info("*** %s() ***\n", __func__);

    DrvDisableFingerTouchReport();
    DrvTouchDeviceHwReset();
    //EnterDBBus();

	// fw_ver = mp_get_fw_info();
    // if (fw_ver == -1) {
    //     res = -ENOMEM;
    //     goto out;
    // }

	// if (fw_ver == 0xFF)
	// 	fw_ver = 0;

	// mp_info("info fw_ver = %x\n", fw_ver);

    res = mp_main_init_var();
	if(res < 0)
		goto out;

    //DrvDisableFingerTouchReport();
    // DrvTouchDeviceHwReset();
    // EnterDBBus();

    // res = p_mp_func->enter_mp_mode();
    // if(res < 0) {
    //     mp_err("Failed to enter MP mode\n");
    //     goto out;
    // }

    // mdelay(100);
    // StopMCU();

	/* Open Test */
    if(mp_test_data->UIConfig.bOpenTest == 1)
	{
    	mp_test_result->nOpenResult = mp_new_flow_main(RUN_OPEN_TEST);
    	for (i = 0; i < MAX_MUTUAL_NUM; i++)
    	{
    		mp_test_result->pGolden_CH[i] = mp_test_data->Goldensample_CH_0[i];
    		mp_test_result->pGolden_CH_Max[i] = mp_test_data->Goldensample_CH_0_Max[i];
    		mp_test_result->pGolden_CH_Min[i] = mp_test_data->Goldensample_CH_0_Min[i];
    	}
    }
    else 
	{
    	mp_test_result->nOpenResult = ITO_NO_TEST;
    }

	/* Short Test */
    if(mp_test_data->UIConfig.bShortTest == 1)
	{
    	mp_test_result->nShortResult = mp_new_flow_main(RUN_SHORT_TEST);
    }
    else 
	{
    	mp_test_result->nShortResult = ITO_NO_TEST;
    }

	/* Return final result */
	if(mp_test_result->nOpenResult == ITO_NO_TEST)
	{
		res = mp_test_result->nShortResult;
	}
	else if(mp_test_result->nShortResult == ITO_NO_TEST)
	{
		res = mp_test_result->nOpenResult;
	}
	else
	{
		if(mp_test_result->nShortResult == ITO_TEST_OK && 
					mp_test_result->nOpenResult == ITO_TEST_OK)
			res = ITO_TEST_OK;
		else
			res = ITO_TEST_FAIL;
	}

	printk("*************************Result(%d): Short = %d, Open = %d \n",res, mp_test_result->nShortResult,
			mp_test_result->nOpenResult);

out:
	DrvTouchDeviceHwReset();
	mdelay(300);
	DrvEnableFingerTouchReport();
	return res;
}

void mp_main_free(void)
{
	mp_info("*** %s *** \n",__func__);

	mp_kfree(mp_test_data->ana_version);
	if(mp_test_data->sensorInfo.numKey > 0)
	{
		mp_kfree(mp_test_data->KeySen);
		mp_kfree(mp_test_data->sensorInfo.key_type);
		mp_info("MEM free mp_test_data->KeySen\n");
		mp_info("MEM free mp_test_data->sensorInfo.key_type\n");
	}
	mp_kfree(mp_test_data->UIConfig.sSupportIC );
	mp_kfree(mp_test_data->project_name);
	mp_kfree(mp_test_data->Goldensample_CH_0 );
	mp_kfree(mp_test_data->Goldensample_CH_0_Max);
	mp_kfree(mp_test_data->Goldensample_CH_0_Max_Avg);
	mp_kfree(mp_test_data->Goldensample_CH_0_Min);
	mp_kfree(mp_test_data->Goldensample_CH_0_Min_Avg);
	mp_kfree(mp_test_data->PAD2Drive);
	mp_kfree(mp_test_data->PAD2Sense);
	if (mp_test_data->sensorInfo.numGr)
	{
		mp_kfree(mp_test_data->PAD2GR);
		mp_info("MEM free mp_test_data->PAD2GR\n");
	}
	

	mp_kfree(mp_test_result->pCheck_Fail);
	mp_kfree(mp_test_result->pOpenResultData);
	mp_kfree(mp_test_result->pOpenFailChannel);
	mp_kfree(mp_test_result->pOpenRatioFailChannel);

	mp_kfree(mp_test_result->pGolden_CH);
	mp_kfree(mp_test_result->pGolden_CH_Max);
	mp_kfree(mp_test_result->pGolden_CH_Max_Avg);
	mp_kfree(mp_test_result->pGolden_CH_Min);
	mp_kfree(mp_test_result->pGolden_CH_Min_Avg);

	mp_kfree(mp_test_result->pShortFailChannel);
	mp_kfree(mp_test_result->pShortResultData);
	mp_kfree(mp_test_result->pShortRData);

	mp_kfree(mp_test_result->pICPinChannel);
	mp_kfree(mp_test_result->pICPinShortFailChannel);
	mp_kfree(mp_test_result->pICPinShortResultData);
	mp_kfree(mp_test_result->pICPinShortRData);

	mp_kfree(mp_test_data);
	mp_kfree(mp_test_result);
	mp_kfree(p_mp_func);
}

int startMPTest(int nChipType, char *pFilePath)
{
	int res = 0;

	mp_info("*** nChipType = 0x%x *** \n",nChipType);
    mp_info("*** iniPath = %s *** \n", pFilePath);

	mutex_lock(&g_Mutex);
    g_IsInMpTest = 1;
    mutex_unlock(&g_Mutex);

	/* Init main structure members */
    p_mp_func = kmalloc(sizeof(struct mp_main_func), GFP_KERNEL);
    if(ERR_ALLOC_MEM(p_mp_func)) {
        mp_err("Failed to allocate mp_func mem\n");
        res = -ENOMEM;
        goto out;
    }

	p_mp_func->chip_type = nChipType;

    if (nChipType == CHIP_TYPE_MSG58XXA) {
        p_mp_func->check_mp_switch = msg30xx_check_mp_switch;
        p_mp_func->enter_mp_mode = msg30xx_enter_mp_mode;
		p_mp_func->open_judge = msg30xx_open_judge;
		p_mp_func->short_judge = msg30xx_short_judge;
		p_mp_func->max_channel_num = MAX_CHANNEL_NUM_30XX;
		p_mp_func->fout_data_addr = 0x1361;
    } else if (nChipType == CHIP_TYPE_MSG28XX) {
        p_mp_func->check_mp_switch = msg28xx_check_mp_switch;
        p_mp_func->enter_mp_mode = msg28xx_enter_mp_mode;
		p_mp_func->open_judge = msg28xx_open_judge;
		p_mp_func->short_judge = msg28xx_short_judge;
		p_mp_func->max_channel_num = MAX_CHANNEL_NUM_28XX;
		p_mp_func->fout_data_addr = 0x136E;
    } else {
        mp_err("New MP Flow doesn't support this chip type\n");
        res = -1;
        goto out;
    }

	/* Parsing ini file and prepare to run MP test */
    res = mp_load_ini(pFilePath);
    if(res < 0) {
        mp_err("Failed to load ini\n");
        goto out;
    }

    res = mp_start_test();

	mp_save_result(res);

out:
	mp_main_free();
	mutex_lock(&g_Mutex);
    g_IsInMpTest = 0;
    mutex_unlock(&g_Mutex);
	return res;
}
