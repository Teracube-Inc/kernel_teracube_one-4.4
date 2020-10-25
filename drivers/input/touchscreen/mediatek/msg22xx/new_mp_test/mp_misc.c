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

#include "mp_common.h"

extern u32 SLAVE_I2C_ID_DBBUS;
extern u32 SLAVE_I2C_ID_DWI2C;

int msg28xx_check_mp_switch(void)
{
    u32 nRegData = 0;
    int nTimeOut = 280;
    int nT = 0;

    mp_info("*** %s() ***\n", __func__);
    do
    {
        nRegData = RegGet16BitValue(0x1402);
        mdelay(15);
        nT++;
        if (nT > nTimeOut)
        {
            return -1;
        }
        //DEBUG("nRegData = %x", nRegData);

    } while (nRegData != 0x7447);

    return 0;
}

int msg30xx_check_mp_switch(void)
{
    u16 nRegData = 0;
    int nTimeOut = 280;
    int nT = 0;

    mp_info("*** %s() ***\n", __func__);
    do
    {
        nRegData = RegGet16BitValueByAddressMode(0x1401, ADDRESS_MODE_16BIT);
        mdelay(15);
        nT++;
        if (nT > nTimeOut)
        {
            return -1;
        }
        mp_info("nT = %d, nRegData = 0x%04x\n", nT, nRegData);
    } while (nRegData != 0x7447);

    return 0;
}

int msg28xx_enter_mp_mode(void)
{
	mp_info("*** %s() ***\n", __func__);
	StopMCU();
	//udelay(100000);
    mdelay(100);
	RegSet16BitValue(0X3C60, 0xAA55);    // disable watch dog
	// RegSet16BitValue(0X3D08, 0xFFFF);   // clear interrupt status
	// RegSet16BitValue(0X3D18, 0xFFFF);   // clear interrupt status

	RegSet16BitValue(0x1402, 0x7474);   // enter mp mode

	RegSet16BitValue(0x1E06, 0x0000);
	RegSet16BitValue(0x1E06, 0x0001);
	StartMCU();
	//udelay(300000);
    mdelay(300);

	// if (msg28xx_check_mp_switch()<0)
	// {
	// 	mp_err("*** Msg28xx MP Test# CheckSwitchStatus failed! ***\n");
	// 	return -1;
	// }
	return 0;
}

int msg30xx_enter_mp_mode(void)
{
	mp_info("*** %s() ***\n", __func__);
	StopMCU();
	mdelay(100);
	RegSet16BitValue(0X3C60, 0xAA55);    // disable watch dog
	// RegSet16BitValue(0X3D08, 0xFFFF);   // clear interrupt status
	// RegSet16BitValue(0X3D18, 0xFFFF);   // clear interrupt status

	RegSet16BitValue(0x1402, 0x7474);   // enter mp mode

	RegSet16BitValue(0x1E06, 0x0000);
	RegSet16BitValue(0x1E06, 0x0001);
	StartMCU();
	mdelay(300);
    
    return 0;
}

void Msg30xxDBBusSetDefault(void)
{
    u8 nParCmdSelUseCfg  = 0x7F;
    u8 nParCmdAdByteEn1  = 0x51;
    u8 nParCmdSetSelB0   = 0x81;
    u8 nParCmdSetSelB1   = 0x83;    
    u8 nParCmdNSetSelB2  = 0x84;
    u8 nParCmdIicUse     = 0x35;
    u8 nParCmdNSelUseCfg = 0x7E;
    u8 nParCmdNIicUse    = 0x34;

    mp_info("*** %s() ***\n", __func__);
    RegSetLByteValue(0, 0);       

    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdSelUseCfg, 1);
    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdAdByteEn1, 1);
    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdSetSelB0, 1);
    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdSetSelB1, 1);    
    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdNSetSelB2, 1);
    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdIicUse, 1);        
    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdNSelUseCfg, 1); 
    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdNIicUse,  1);     
}

void Msg30xxDBBusReadDQMemStart(void)
{
    u8 nParCmdSelUseCfg = 0x7F;
    //u8 nParCmdAdByteEn0 = 0x50;
    u8 nParCmdAdByteEn2 = 0x52;
    //u8 nParCmdAdByteEn1 = 0x51;
    //u8 nParCmdDaByteEn0 = 0x54;
    u8 nParCmdUSetSelB0 = 0x80;
    u8 nParCmdUSetSelB1 = 0x82;
    u8 nParCmdSetSelB2  = 0x85;
    u8 nParCmdIicUse    = 0x35;
    //u8 nParCmdWr        = 0x10;

    mp_info("*** %s() ***\n", __func__);

    RegSet16BitValueOff(0x0F50, BIT1);

    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdSelUseCfg, 1);//0x7F
    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdAdByteEn2, 1);//0x52
    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdUSetSelB0, 1);//0x80
    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdUSetSelB1, 1);//0x82
    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdSetSelB2,  1);//0x85
    IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdIicUse,    1);//0x35
}

void Msg30xxDBBusReadDQMemEnd(void)
{ 
    //u8 nParCmdNSelUseCfg = 0x7E;
    u8 nParCmdNSelUseCfg = 0x7F;

    mp_info("*** %s() ***\n", __func__);

    Msg30xxDBBusSetDefault();

    //IicWriteData(SLAVE_I2C_ID_DBBUS, &nParCmdNSelUseCfg, 1);
    RegSet16BitValueOn(0x0F50, BIT1);
}

void ExitDBBus(void)
{
    mp_info( "*** %s *** \n", __func__);
    DbBusIICNotUseBus();
    DbBusNotStopMCU();
    DbBusExitSerialDebugMode();
}

void EnterDBBus(void)
{
    mp_info( "*** %s *** \n", __func__);
    DbBusEnterSerialDebugMode();
    DbBusStopMCU();
    DbBusIICUseBus();
    DbBusIICReshape();
}

void StartMCU(void)
{
    mp_info( "*** %s *** \n", __func__);
    RegSetLByteValue(0x0FE6, 0x00); //bank:mheg5, addr:h0073
}

void StopMCU(void)
{
    mp_info( "*** %s *** \n", __func__);
    RegSetLByteValue(0x0FE6, 0x01); //bank:mheg5, addr:h0073
}

void DebugShowArray2(void *pBuf, u16 nLen, int nDataType, int nCarry, int nChangeLine)
{
    int i;
    char szStrTmp[10] = {0};
    char szStrBuf[512] = {0};
    u8 * pU8Buf = NULL;
    s8 * pS8Buf = NULL;
    u16 * pU16Buf = NULL;
    s16 * pS16Buf = NULL;
    u32 * pU32Buf = NULL;
    s32 * pS32Buf = NULL;

    mp_info("*** %s ***\n",__func__);

    if(nDataType == 8)
        pU8Buf = (u8 *)pBuf;
    else if(nDataType == -8)
        pS8Buf = (s8 *)pBuf;
    else if(nDataType == 16)
        pU16Buf = (u16 *)pBuf;
    else if(nDataType == -16)
        pS16Buf = (s16 *)pBuf;
    else if(nDataType == 32)
        pU32Buf = (u32 *)pBuf;
    else if(nDataType == -32)
        pS32Buf = (s32 *)pBuf;

    for(i=0; i < nLen; i++)
    {
        if(nCarry == 16)
        {
            if(nDataType == 8)
            	sprintf(szStrTmp, "%02X ", pU8Buf[i]);
            else if(nDataType == -8)
            	sprintf(szStrTmp, "%02X ", pS8Buf[i]);
            else if(nDataType == 16)
            	sprintf(szStrTmp, "%04X ", pU16Buf[i]);
            else if(nDataType == -16)
            	sprintf(szStrTmp, "%04X ", pS16Buf[i]);
            else if(nDataType == 32)
            	sprintf(szStrTmp, "%08X ", pU32Buf[i]);
            else if(nDataType == -32)
            	sprintf(szStrTmp, "%08X ", pS32Buf[i]);
        }
        else if(nCarry == 10)
        {
            if(nDataType == 8)
            	sprintf(szStrTmp, "%6u ", pU8Buf[i]);
            else if(nDataType == -8)
            	sprintf(szStrTmp, "%6d ", pS8Buf[i]);
            else if(nDataType == 16)
            	sprintf(szStrTmp, "%6u ", pU16Buf[i]);
            else if(nDataType == -16)
            	sprintf(szStrTmp, "%6d ", pS16Buf[i]);
            else if(nDataType == 32)
            	sprintf(szStrTmp, "%6u ", pU32Buf[i]);
            else if(nDataType == -32)
            	sprintf(szStrTmp, "%6d ", pS32Buf[i]);
        }

        strcat(szStrBuf, szStrTmp);
        memset(szStrTmp, 0, 10);         
        if(i%nChangeLine==nChangeLine-1){
        	printk("%s\n", szStrBuf);
        	memset(szStrBuf, 0, 512);
        }
    }
    printk("\n");
}

int Msg30xxCovertRValue(int dump_time, s32 deltaR) 
{
    s32 result = 0;

   	if (deltaR >= IIR_MAX)
   	   	return 0;
   	if (deltaR == 0)
   	    deltaR = 1;

    if(deltaR > -1000)
        result = (int)(73318 * dump_time * 1000) / (55 * (abs(deltaR) - 0));
    else
        result = (int)(53248 * dump_time * 1000) / (55 * ((deltaR) - 0));

    return result;
}

int Msg28xxCovertRValue(s32 deltaR)
{
    s32 result = 0;
   	if (deltaR >= IIR_MAX)
   	   	return 0;
   	if (deltaR == 0)
   	    deltaR = 1;
    if(deltaR > -1000)
        result = (int)(187541 * 20) / ((abs(deltaR) - 0));
    else
        result = (int)(100745 * 20) / (((deltaR) - 0));
    mp_info("*** result %d ***\n", result);
   	return result;
}

/* global vars for open */
u16 nNormalTestResultCheck[MAX_MUTUAL_NUM];
u16 normalTestFail_check_Deltac[MAX_MUTUAL_NUM];
u16 normalTestFail_check_Ratio[MAX_MUTUAL_NUM];
int ratio_border[MAX_MUTUAL_NUM];
int ratio_move[MAX_MUTUAL_NUM];
int ratio[MAX_MUTUAL_NUM];

/* global vars for short */
s8 normalTestFail[TEST_ITEM_NUM] = {0};//0:golden    1:ratio
u16 normalTestFail_check[TEST_ITEM_NUM][MAX_MUTUAL_NUM];//6:max subframe    14:max afe
int _gMsg30xxGRtestPins[14] = {0};
int _gGRtestPins[13] = {0};

u16 Msg28xxreadTestPins(u16 itemID, int *testPins)
{
    int nSize = 0, i;
	char str[512] = {0};

	mp_info("*** %s() ***\n", __func__);

    switch (itemID)
    {
    	case 1:
        case 11:			
			ms_get_ini_data("SHORT_TEST_N1","TEST_PIN",str);
			nSize = ms_ini_split_int_array(str, testPins);
        	DBG(0, "SHORT_TEST_N1 nSize = %d\n", nSize);
            break;
        case 2:
        case 12:
			ms_get_ini_data("SHORT_TEST_N2","TEST_PIN",str);
			nSize = ms_ini_split_int_array(str, testPins);
        	DBG(0, "SHORT_TEST_N2 nSize = %d\n", nSize);
            break;
        case 3:
        case 13:
			ms_get_ini_data("SHORT_TEST_S1","TEST_PIN",str);
			nSize = ms_ini_split_int_array(str, testPins);
        	DBG(0, "SHORT_TEST_S1 nSize = %d\n", nSize);
            break;
        case 4:
        case 14:
			ms_get_ini_data("SHORT_TEST_S2","TEST_PIN",str);
			nSize = ms_ini_split_int_array(str, testPins);
        	DBG(0, "SHORT_TEST_S2 nSize = %d\n", nSize);
            break;

        case 5:
        case 15:
        	if (ms_get_ini_data("SHORT_TEST_N3", "MUX_MEM_20_3E", str) != 0) {
				ms_get_ini_data("SHORT_TEST_N3","TEST_PIN",str);
				nSize = ms_ini_split_int_array(str, testPins);
        		DBG(0, "SHORT_TEST_N3 nSize = %d\n", nSize);
        	}
            else if (ms_get_ini_data("SHORT_TEST_S3", "MUX_MEM_20_3E", str) != 0) {
				ms_get_ini_data("SHORT_TEST_S3","TEST_PIN",str);
				nSize = ms_ini_split_int_array(str, testPins);				
            	DBG(0, "SHORT_TEST_S3 nSize = %d\n", nSize);
            }
            else if (ms_get_ini_data("SHORT_TEST_GR", "MUX_MEM_20_3E", str) != 0) {
            	if (mp_test_data->sensorInfo.numGr == 0)
            		nSize = 0;
            	else
            		nSize = mp_test_data->sensorInfo.numGr;

            	for (i = 0; i < sizeof(_gGRtestPins) / sizeof(_gGRtestPins[0]); i++)
            		testPins[i] = _gGRtestPins[i];
            	mp_info("SHORT_TEST_GR nSize = %d\n", nSize);
            }

            break;

        case 0:
        	default:
            return 0;
    }

    for (i = nSize; i < MAX_CHANNEL_NUM_28XX; i++) {
    	testPins[i] = PIN_NO_ERROR;
    }

    return nSize;
}

u16 Msg30xxreadTestPins(u16 itemID, int *testPins)
{
    int nSize = 0, i;
    char str[512] = {0};

	mp_info("*** %s() ***\n", __func__);

    switch (itemID)
    {
    	case 0:
        case 10:
			ms_get_ini_data("SHORT_TEST_N1","TEST_PIN",str);
			nSize = ms_ini_split_int_array(str, testPins);
        	mp_info("SHORT_TEST_N1 nSize = %d\n", nSize);
            break;
        case 1:
        case 11:
			ms_get_ini_data("SHORT_TEST_N2","TEST_PIN",str);
			nSize = ms_ini_split_int_array(str, testPins);
        	mp_info("SHORT_TEST_N2 nSize = %d\n", nSize);
            break;
        case 2:
        case 12:
			ms_get_ini_data("SHORT_TEST_S1","TEST_PIN",str);
			nSize = ms_ini_split_int_array(str, testPins);
        	mp_info("SHORT_TEST_S1 nSize = %d\n", nSize);
            break;
        case 3:
        case 13:
			ms_get_ini_data("SHORT_TEST_S2","TEST_PIN",str);
			nSize = ms_ini_split_int_array(str, testPins);
        	mp_info("SHORT_TEST_S2 nSize = %d\n", nSize);
            break;

        case 4:
        case 14:
        	if (ms_get_ini_data("SHORT_TEST_N3", "MUX_MEM_20_3E", str) != 0) {
				ms_get_ini_data("SHORT_TEST_N3","TEST_PIN",str);
				nSize = ms_ini_split_int_array(str, testPins);
        		mp_info("SHORT_TEST_N3 nSize = %d\n", nSize);
        	}
            else if (ms_get_ini_data("SHORT_TEST_S3", "MUX_MEM_20_3E", str) != 0) {
				ms_get_ini_data("SHORT_TEST_S3","TEST_PIN",str);
				nSize = ms_ini_split_int_array(str, testPins);				
            	mp_info("SHORT_TEST_S3 nSize = %d\n", nSize);
            }
            else if (ms_get_ini_data("SHORT_TEST_GR", "MUX_MEM_20_3E", str) != 0) {
            	if (mp_test_data->sensorInfo.numGr == 0)
            		nSize = 0;
            	else
            		nSize = mp_test_data->sensorInfo.numGr;

            	for (i = 0; i < sizeof(_gMsg30xxGRtestPins) / sizeof(_gMsg30xxGRtestPins[0]); i++)
            		testPins[i] = _gMsg30xxGRtestPins[i];
            	mp_info("SHORT_TEST_GR nSize = %d\n", nSize);
            }
            break;

        default:
            return 0;
    }

    for (i = nSize; i < MAX_CHANNEL_NUM_30XX; i++) {
    	testPins[i] = PIN_NO_ERROR;
    }

    return nSize;
}

s32 Msg28xxShortTestJudge(int *deltac_data, u8 nItemID, s8 *TestFail, u16 TestFail_check[][MAX_MUTUAL_NUM])
{
	int nRet = 1, i, count_test_pin = 0;
	int testPins[MAX_CHANNEL_NUM_28XX] = {0};

	mp_info("*** %s() ***\n", __func__);

	Msg28xxreadTestPins(nItemID, testPins);

	for (i = 0; i < sizeof(testPins) / sizeof(testPins[0]); i++) {
		if (testPins[i] != PIN_NO_ERROR)
			count_test_pin++;
	}
	
	for (i = 0; i < count_test_pin; i++) {
		TestFail_check[nItemID][i] = testPins[i];
		if (0 == check_thrs(deltac_data[i + (nItemID - 1) * 13], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
	    {
	    	TestFail[nItemID] = 1;
            mp_test_result->nShortResult = ITO_TEST_FAIL;
            mp_info("*** (%d) ***\n", __LINE__);
	    	nRet = 0;
	    }
	}

	return nRet;
}

s32 Msg30xxShortTestJudge(int *deltac_data, u8 nItemID, s8 *TestFail, u16 TestFail_check[][MAX_MUTUAL_NUM])
{
	int nRet = 1, i, count_test_pin = 0;
	int testPins[MAX_CHANNEL_NUM_30XX] = {0};

	mp_info("*** %s() ***\n", __func__);

	Msg30xxreadTestPins(nItemID, testPins);

    for(i = 0; i < ARRAY_SIZE(testPins); i++)
        mp_info("testPins[%d] = %d\n", i , testPins[i]);

	for (i = 0; i < sizeof(testPins) / sizeof(testPins[0]); i++) {
		if (testPins[i] != PIN_NO_ERROR)
			count_test_pin++;
	}

	for (i = 0; i < count_test_pin; i++) {
		TestFail_check[nItemID][i] = testPins[i];
		if (0 == check_thrs(deltac_data[i + nItemID * 14], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
	    {
	    	TestFail[nItemID] = 1;
            mp_test_result->nShortResult = ITO_TEST_FAIL;
            mp_info("*** (%d) ***\n", __LINE__);
	    	nRet = 0;
	    }
	}

	return nRet;
}

int msg28xx_short_judge(int *deltac_data)
{
    int i, j;
    int *senseR = NULL, sense = mp_test_data->sensorInfo.numSen;
	int *driveR = NULL, drive = mp_test_data->sensorInfo.numDrv;
	int *GRR = NULL, gr = mp_test_data->sensorInfo.numGr;
    int count_test_pin = 0, shift;
    int thrs = 0, RealThrs = 0;
    u16 Temp_20_3E_Settings[16] = {0};
    u16 nTestItemLoop = 6;
    u16 nTestItem = 0;
    u32 nRetVal = 1;
    char str[512] = {0};
    int datalen = MAX_CHANNEL_NUM_28XX;
    int *deltaC = NULL;
    int sum = 0, cunt = 0, avg = 0, convert[MAX_CHANNEL_NUM_28XX] = {0};

    deltaC = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    senseR = kcalloc(sense, sizeof(int), GFP_KERNEL);
	driveR = kcalloc(drive, sizeof(int), GFP_KERNEL);
	GRR = kcalloc(gr, sizeof(int), GFP_KERNEL);
    if(ERR_ALLOC_MEM(senseR) || ERR_ALLOC_MEM(driveR) || ERR_ALLOC_MEM(GRR)) {
        mp_err("Failed to allocate senseR/driveR/GRR mem\n");
        nRetVal = -1;
        goto ITO_TEST_END;
    }

    //dump_time = (((mp_test_data->SHORT_Dump1 + 1) * 4 * 2)*100) / 5166;
    //mp_info("%s, dump_time = %d, short_thrs = %d\n", __func__, dump_time, mp_test_data->sensorInfo.thrsShort);

    thrs = Msg28xxCovertRValue(mp_test_data->sensorInfo.thrsShort);

    for (i = 0; i < sense; i++) senseR[i] = thrs;
    for (i = 0; i < drive; i++) driveR[i] = thrs;
    for (i = 0; i < gr; i++) GRR[i] = thrs;

    for (i = 1; i < 6; i++)  {//max 6 subframe
        for (j = 0; j < 13; j++) {// max 13 AFE
            if (((i-1) * 13 + j) < MAX_CHANNEL_NUM_28XX)
                mp_test_result->pShortFailChannel[(i-1) * 13 + j] = (u32)PIN_UN_USE;
        // *(normalTestFail_check+i*TEST_ITEM_NUM+j)=PIN_UN_USE;
            normalTestFail_check[i][j] = PIN_UN_USE;
        }
    }

    memset(normalTestFail, 0, TEST_ITEM_NUM * sizeof(s8));
    //?�FWMapping填錯,?��??�[0]<-->[1] / [2]<-->[3]
    for (i = 0; i < datalen; i+=2) // 2 AFE * 28 subframe,but AFE0 un-use
    {
        //deltaC[i] = ideltaC[i];
        deltaC[i] = deltac_data[i+1];
        deltaC[i + 1] = deltac_data[i];

        if ((deltaC[i] > -1000) && (deltaC[i] < 1000))
        {
            sum += deltaC[i];
            cunt++;
        }

        if ((deltaC[i+1] > -1000) && (deltaC[i+1] < 1000))
        {
            sum += deltaC[i + 1];
            cunt++;
        }
    }
    if (cunt != 0)
    {
        avg = (int)(sum / cunt);

        for (i = 0; i < datalen; i++)
        {
            deltac_data[i] = deltaC[i] - avg;
           // mp_info("*** deltac_data[%d] = %d ***\n", i, deltac_data[i]);
        }
    }
    /* New judge - not verified yet */
    for(i = 0; i < datalen; i++) {

        convert[i] = Msg28xxCovertRValue(deltac_data[i]);
        //mp_info("*** convert[%d] = %d ***\n", i, convert[i]);

        if(mp_test_data->PAD2Sense != NULL) {
            for(j = 0; j < p_mp_func->sense_len; j++) {
                
                if(i == mp_test_data->PAD2Sense[j]) {

                    mp_test_result->pShortRData[j] = convert[i];
                    mp_test_result->pShortResultData[j] = deltac_data[i];

                    if (convert[i] >= 10000)
                        mp_test_result->pShortRData[j] = 10000;

                    if (0 == check_thrs(deltac_data[i], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
                    {
                        nRetVal = -1;
                        mp_test_result->nShortResult = ITO_TEST_FAIL;
                        mp_test_result->pShortFailChannel[j] = ITO_TEST_FAIL;
                    }   
                }
            }

        }

        if(mp_test_data->PAD2Drive != NULL) {

            for(j = 0; j < p_mp_func->drive_len; j++) {

                if(i == mp_test_data->PAD2Drive[j]) {

                    mp_test_result->pShortRData[p_mp_func->sense_len + j] = convert[i];
                    mp_test_result->pShortResultData[p_mp_func->sense_len + j] = deltac_data[i];

                    if (convert[i] >= 10000)
                        mp_test_result->pShortRData[p_mp_func->sense_len + j] = 10000;

                    if (0 == check_thrs(deltac_data[i], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
                    {
                        nRetVal = -1;
                        mp_test_result->nShortResult = ITO_TEST_FAIL;
                        mp_test_result->pShortFailChannel[p_mp_func->sense_len + j] = -1;
                    }
                }                
            }
        }

        if(mp_test_data->PAD2GR != NULL) {

            for(j = 0; j < p_mp_func->gr_len; j++) {

                if(i == mp_test_data->PAD2GR[j]) {

                    mp_test_result->pShortRData[p_mp_func->sense_len + p_mp_func->drive_len + j] = convert[i];
                    mp_test_result->pShortResultData[p_mp_func->sense_len + p_mp_func->drive_len + j] = deltac_data[i];

                    if (convert[i] >= 10000)
                        mp_test_result->pShortRData[p_mp_func->sense_len + p_mp_func->drive_len + j] = 10000;

                    if (0 == check_thrs(deltac_data[i], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
                    {
                        nRetVal = -1;
                        mp_test_result->nShortResult = ITO_TEST_FAIL;
                        mp_test_result->pShortFailChannel[p_mp_func->sense_len + p_mp_func->drive_len + j] = -1;
                    }
                }                
            }
        }
    }

    mp_info("Short Judge done \n");
    goto ITO_TEST_END;
     /*
      * Here is a new judge flow copied from older one.
      * The given data is sorted by PAD TAble written in INI.
      */
     for (nTestItem = 0; nTestItem < nTestItemLoop; nTestItem++) {
        if (!Msg28xxShortTestJudge(deltac_data, nTestItem, normalTestFail, normalTestFail_check)) {
            mp_err("*** New ITO Short Judge failed! ***\n");
            nRetVal = -1;
        }

        count_test_pin = 0;

        for (i = 0; i < 13; i++) {
            if (normalTestFail_check[nTestItem][i] != PIN_UN_USE)
                count_test_pin++;
        }

        mp_info("nTestItem = %d, count_test_pin = %d\n", nTestItem, count_test_pin);

        memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
        ms_get_ini_data("SHORT_TEST_N3", "MUX_MEM_20_3E", str);

        if (nTestItem == 0 || nTestItem == 1 || (nTestItem == 4 && (ms_ini_split_u16_array(str, Temp_20_3E_Settings)))) {
            mp_info("NEW SHORT_TEST_N3\n");
            for (i = 0; i < count_test_pin; i++) {
                for (j = 0; j < sense; j++) {
                    if (normalTestFail_check[nTestItem][i] == mp_test_data->PAD2Sense[j]) {
                        shift = (i + (nTestItem - 1) * 13);
                        senseR[j] = Msg28xxCovertRValue(deltac_data[shift]);

                        mp_info("senseR[%d] = %.2d, deltac_data[%d] = %d\n", j, senseR[j], shift, deltac_data[shift]);

                        mp_test_result->pShortRData[j] = senseR[j];
                        mp_test_result->pShortResultData[j] = deltac_data[shift];

                        if (senseR[j] >= 10)
                            mp_test_result->pShortRData[j] = 10;

                        if (0 == check_thrs(deltac_data[shift], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
                            mp_test_result->pShortFailChannel[j] = (u32)normalTestFail_check[nTestItem][i];
                    }
                }
            }
        }

        memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
        ms_get_ini_data("SHORT_TEST_S3", "MUX_MEM_20_3E", str);

        if (nTestItem == 2 || nTestItem == 3 || (nTestItem == 4 && (ms_ini_split_u16_array(str, Temp_20_3E_Settings)))) {
            mp_info("NEW SHORT_TEST_S3\n");
            for (i = 0; i < count_test_pin; i++) {
                for (j = 0; j < drive; j++) {
                    if (normalTestFail_check[nTestItem][i] == mp_test_data->PAD2Drive[j]) {
                        shift = (i + (nTestItem - 1) * 13);
                        driveR[j] = Msg28xxCovertRValue(deltac_data[shift]);

                        mp_info("driveR[%d] = %.2d, deltac_data[%d] = %d\n", j, driveR[j], shift, deltac_data[shift]);

                        mp_test_result->pShortRData[sense + j] = driveR[j];
                        mp_test_result->pShortResultData[sense + j] = deltac_data[shift];

                        if (driveR[j] >= 10)
                            mp_test_result->pShortRData[sense + j] = 10;

                        if (0 == check_thrs(deltac_data[shift], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
                            mp_test_result->pShortFailChannel[sense + j] = (u32)normalTestFail_check[nTestItem][i];
                    }
                }
            }
        }

        memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
        ms_get_ini_data("SHORT_TEST_GR", "MUX_MEM_20_3E", str);

        if (nTestItem == 4 && (ms_ini_split_u16_array(str, Temp_20_3E_Settings))) {
            for (i = 0; i < count_test_pin; i++) {
                for (j = 0; j < gr; j++) {
                    if (normalTestFail_check[nTestItem][i] == mp_test_data->PAD2GR[j]) {
                        shift = (i + (nTestItem - 1) * 13);
                        GRR[j] = Msg28xxCovertRValue(deltac_data[shift]);

                        mp_info("GRR[%d] = %.2d, deltac_data[%d] = %d\n", j, GRR[j], shift, deltac_data[shift]);

                        mp_test_result->pShortRData[sense + drive + j] = GRR[j];
                        mp_test_result->pShortResultData[sense + drive + j] = deltac_data[shift];

                        if (GRR[j] >= 10)
                            mp_test_result->pShortRData[sense + drive + j] = 10.0;

                        if (0 == check_thrs(deltac_data[shift], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
                            mp_test_result->pShortFailChannel[sense + drive + j] = (u32)normalTestFail_check[nTestItem][i];
                    }
                }
            }
        }

        if (normalTestFail[nTestItem]) {
            mp_test_result->pCheck_Fail[3] = normalTestFail[nTestItem];// ito short fail
            nRetVal = -1;
        }
     }

ITO_TEST_END:
    mp_kfree(deltaC);
    mp_kfree(senseR);
    mp_kfree(driveR);
    mp_kfree(GRR);
    return nRetVal;
}

int msg30xx_short_judge(int *deltac_data)
{
    int i, j, shift;
    int *senseR = NULL, sense = mp_test_data->sensorInfo.numSen;
	int *driveR = NULL, drive = mp_test_data->sensorInfo.numDrv;
	int *GRR = NULL, gr = mp_test_data->sensorInfo.numGr;
    int count_test_pin = 0;
    int thrs = 0, RealThrs = 0, dump_time;
    u16 Temp_20_3E_Settings[16] = {0};
    u16 nTestItemLoop = 6 - 1;
    u16 nTestItem = 0;
    u32 nRetVal = 1;
    char str[512] = {0};
    int datalen1 = 0;
    int *deltaC = NULL;
    int sum = 0, cunt = 0, avg = 0, convert[MAX_AFENUMs_30XX * MAX_SUBFRAMES_30XX] = {0};

    deltaC = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);

    senseR = kcalloc(sense, sizeof(int), GFP_KERNEL);
	driveR = kcalloc(drive, sizeof(int), GFP_KERNEL);
	GRR = kcalloc(gr, sizeof(int), GFP_KERNEL);
    if(ERR_ALLOC_MEM(senseR) || ERR_ALLOC_MEM(driveR) || ERR_ALLOC_MEM(GRR)) {
        mp_err("Failed to allocate senseR/driveR/GRR mem\n");
        nRetVal = -1;
        goto ITO_TEST_END;
    }
    dump_time = (((mp_test_data->SHORT_Dump1 + 1) * 4 * 2)*100) / 5166;
    mp_info("%s, dump_time = %d, short_thrs = %d\n", __func__, dump_time, mp_test_data->sensorInfo.thrsShort);

    thrs = Msg30xxCovertRValue(dump_time, mp_test_data->sensorInfo.thrsShort);
    
    for (i = 0; i < sense; i++) senseR[i] = thrs;
    
    for (i = 0; i < drive; i++) driveR[i] = thrs;
    
    for (i = 0; i < gr; i++) GRR[i] = thrs;
    
    for (i = 0; i < 6; i++)  {//max 6 subframe
        for (j = 0; j < 14; j++) {// max 14 AFE
            if ((i * 14 + j) < MAX_CHANNEL_NUM_30XX)
                mp_test_result->pShortFailChannel[i * 14 + j] = (u32)PIN_UN_USE;
            normalTestFail_check[i][j] = PIN_UN_USE;
        }
    }

    memset(normalTestFail, 0, TEST_ITEM_NUM * sizeof(s8));

    datalen1 = MAX_AFENUMs_30XX * MAX_SUBFRAMES_30XX;
    for (i = 0; i < datalen1; i++) // 2 AFE * 28 subframe,but AFE0 un-use
    {
        deltaC[i] = deltac_data[i];

        if ((deltaC[i] > -1000) && (deltaC[i] < 1000))
        {
            sum += deltaC[i];
            cunt++;
        }      
        mp_info("*** deltaC[%d] = %d ***\n", i, deltaC[i]);
        //result += string.Format("deltaC[{0}]={1}\n", i, deltaC[i]);
    }
    
    if (cunt != 0)
    {
        avg = (int)(sum / cunt);

        mp_info("avg = %d, sum = %d, cunt = %d\n", avg, sum, cunt);

        //int datalen2 = MAX_AFENUMs_30XX * MAX_SUBFRAMES_30XX;
        for (i = 0; i < datalen1; i++)
        {
            deltac_data[i] = deltaC[i] - avg;
            mp_info("*** deltac_data[%d] = %d ***\n", i, deltac_data[i]);
        }
    }
    
    /* New judge - not verified yet */
    for(i = 0; i < datalen1; i++) {

        convert[i] = Msg30xxCovertRValue(dump_time, deltac_data[i]);
        mp_info("*** convert[%d] = %d ***\n", i, convert[i]);

        if(mp_test_data->PAD2Sense != NULL) {
            for(j = 0; j < p_mp_func->sense_len; j++) {
                
                if(i == mp_test_data->PAD2Sense[j]) {

                    mp_test_result->pShortRData[j] = convert[i];
                    mp_test_result->pShortResultData[j] = deltac_data[i];

                    if (convert[i] >= 10000)
                        mp_test_result->pShortRData[j] = 10000;

                    if (0 == check_thrs(deltac_data[i], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
                    {
                        nRetVal = -1;
                        mp_test_result->nShortResult = ITO_TEST_FAIL;
                        mp_test_result->pShortFailChannel[j] = ITO_TEST_FAIL;
                    }   
                }
            }

        }
        if(mp_test_data->PAD2Drive != NULL) {

            for(j = 0; j < p_mp_func->drive_len; j++) {

                if(i == mp_test_data->PAD2Drive[j]) {

                    mp_test_result->pShortRData[p_mp_func->sense_len + j] = convert[i];
                    mp_test_result->pShortResultData[p_mp_func->sense_len + j] = deltac_data[i];

                    if (convert[i] >= 10000)
                        mp_test_result->pShortRData[p_mp_func->sense_len + j] = 10000;

                    if (0 == check_thrs(deltac_data[i], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
                    {
                        nRetVal = -1;
                        mp_test_result->nShortResult = ITO_TEST_FAIL;
                        mp_test_result->pShortFailChannel[p_mp_func->sense_len + j] = -1;
                    }
                }                
            }
        }
    
        if(mp_test_data->PAD2GR != NULL) {

            for(j = 0; j < p_mp_func->gr_len; j++) {

                if(i == mp_test_data->PAD2GR[j]) {

                    mp_test_result->pShortRData[p_mp_func->sense_len + p_mp_func->drive_len + j] = convert[i];
                    mp_test_result->pShortResultData[p_mp_func->sense_len + p_mp_func->drive_len + j] = deltac_data[i];

                    if (convert[i] >= 10000)
                        mp_test_result->pShortRData[p_mp_func->sense_len + p_mp_func->drive_len + j] = 10000;

                    if (0 == check_thrs(deltac_data[i], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
                    {
                        nRetVal = -1;
                        mp_test_result->nShortResult = ITO_TEST_FAIL;
                        mp_test_result->pShortFailChannel[p_mp_func->sense_len + p_mp_func->drive_len + j] = -1;
                    }
                }                
            }
        }
    }
    
    //nRetVal = -1;
    mp_info("Short Judge done \n");
    goto ITO_TEST_END;


     /*
      * Here is a new judge flow copied from older one.
      * The given data is sorted by PAD TAble written in INI.
      */
     for (nTestItem = 0; nTestItem < nTestItemLoop; nTestItem++) {
        if (!Msg30xxShortTestJudge(deltac_data, nTestItem, normalTestFail, normalTestFail_check)) {
            mp_err("*** New ITO Short Judge failed! ***\n");
            nRetVal = -1;
            //goto ITO_TEST_END;
        }

        count_test_pin = 0;

        for (i = 0; i < 14; i++) {
            //printk("normalTestFail_check[%d][%d] = %x", nTestItem, i, normalTestFail_check[nTestItem][i]);
            if (normalTestFail_check[nTestItem][i] != PIN_UN_USE)
                count_test_pin++;
        }

        mp_info("nTestItem = %d, count_test_pin = %d\n", nTestItem, count_test_pin);

        memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
        ms_get_ini_data("SHORT_TEST_N3", "MUX_MEM_20_3E", str);

        if (nTestItem == 0 || nTestItem == 1 || (nTestItem == 4 && (ms_ini_split_u16_array(str, Temp_20_3E_Settings)))) {
            mp_info("NEW SHORT_TEST_N3\n");
            for (i = 0; i < count_test_pin; i++) {
                for (j = 0; j < sense; j++) {
                    mp_info("check[%d][%d] = %d, PAD2Sens[%d] = %d\n",nTestItem,i,normalTestFail_check[nTestItem][i],j,mp_test_data->PAD2Sense[j]);
                    if (normalTestFail_check[nTestItem][i] == mp_test_data->PAD2Sense[j]) {
                        shift = (i + nTestItem  * 14);
                        senseR[j] = Msg30xxCovertRValue(dump_time, deltac_data[shift]);
                    
                        mp_info("senseR[%d] = %.2d, deltac_data[%d] = %d\n", j, senseR[j], shift, deltac_data[shift]);

                        mp_test_result->pShortRData[j] = senseR[j];
                        mp_test_result->pShortResultData[j] = deltac_data[shift];

                        mp_info("pShortRData[%d] = %.2d, pShortResultData[%d] = %d\n", j, mp_test_result->pShortRData[j], j, mp_test_result->pShortResultData[j]);

                        if (senseR[j] >= 10000)
                            mp_test_result->pShortRData[j] = 10000;

                        if (0 == check_thrs(deltac_data[shift], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
                            mp_test_result->pShortFailChannel[j] = (u32)normalTestFail_check[nTestItem][i];
                    }
                }
            }
        }

        memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
        ms_get_ini_data("SHORT_TEST_S3", "MUX_MEM_20_3E", str);

        if (nTestItem == 2 || nTestItem == 3 || (nTestItem == 4 && (ms_ini_split_u16_array(str, Temp_20_3E_Settings)))) {
            mp_info("NEW SHORT_TEST_S3\n");
            for (i = 0; i < count_test_pin; i++) {
                for (j = 0; j < drive; j++) {
                    //mp_info("check[%d][%d] = %d, PAD2Drive[%d] = %d\n",nTestItem,i,normalTestFail_check[nTestItem][i],j,mp_test_data->PAD2Drive[j]);
                    if (normalTestFail_check[nTestItem][i] == mp_test_data->PAD2Drive[j]) {
                        shift = (i + nTestItem  * 14);
                        driveR[j] = Msg30xxCovertRValue(dump_time, deltac_data[shift]);

                        mp_info("driveR[%d] = %.2d, deltac_data[%d] = %d\n", j, driveR[j], shift, deltac_data[shift]);

                        mp_test_result->pShortRData[sense + j] = driveR[j];
                        mp_test_result->pShortResultData[sense + j] = deltac_data[shift];

                        mp_info("pShortRData[%d] = %.2d, pShortResultData[%d] = %d\n", j, mp_test_result->pShortRData[sense + j], j, mp_test_result->pShortResultData[sense + j]);

                        if (driveR[j] >= 10000)
                            mp_test_result->pShortRData[sense + j] = 10000;

                        if (0 == check_thrs(deltac_data[shift], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
                            mp_test_result->pShortFailChannel[sense + j] = (u32)normalTestFail_check[nTestItem][i];
                    }
                }
            }
        }

        memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
        ms_get_ini_data("SHORT_TEST_GR", "MUX_MEM_20_3E", str);

        if (nTestItem == 4 && (ms_ini_split_u16_array(str, Temp_20_3E_Settings))) {
            for (i = 0; i < count_test_pin; i++) {
                for (j = 0; j < gr; j++) {
                    if (normalTestFail_check[nTestItem][i] == mp_test_data->PAD2GR[j]) {
                        shift = (i + nTestItem  * 14);
                        GRR[j] = Msg30xxCovertRValue(dump_time, deltac_data[shift]);

                        mp_test_result->pShortRData[sense + drive + j] = GRR[j];
                        mp_test_result->pShortResultData[sense + drive + j] = deltac_data[shift];

                        mp_info("GR[%d] = %.2d, deltac_data[%d] = %d\n", j, GRR[j], shift, deltac_data[shift]);

                        if (GRR[j] >= 10000)
                            mp_test_result->pShortRData[sense + drive + j] = 10000;

                        if (0 == check_thrs(deltac_data[shift], mp_test_data->sensorInfo.thrsShort, -mp_test_data->sensorInfo.thrsShort))
                            mp_test_result->pShortFailChannel[sense + drive + j] = (u32)normalTestFail_check[nTestItem][i];
                    }
                }
            }
        }

        if (normalTestFail[nTestItem]) {
            mp_test_result->pCheck_Fail[3] = normalTestFail[nTestItem];// ito short fail
            nRetVal = -1;
        }
     }

ITO_TEST_END:
    mp_kfree(senseR);
    mp_kfree(driveR);
    mp_kfree(GRR);
    mp_kfree(deltaC);
    return nRetVal;
}

int msg28xx_open_judge(int *deltac_data, int deltac_size)
{
    u16 i, j, k;
    u16 nRowNum = 0, nColumnNum = 0;
    u16 nCSub, nCfb;
    s32 nRetVal = 1, bg_per_csub;
    int ratioAvg = 0, ratioAvg_max = 0, ratioAvg_min = 0, passCount = 0;
    int ratioAvg_border = 0, ratioAvg_border_max = 0, ratioAvg_border_min = 0, passCount1 = 0;
    int ratioAvg_move = 0, ratioAvg_border_move = 0;
    int sense_num = 0, drv_num = 0, *result_buf = NULL;
    s8 nNormalTestResult[2] = {0};

    mp_info("*** %s() ***\n", __func__);

    sense_num = mp_test_data->sensorInfo.numSen;
    drv_num = mp_test_data->sensorInfo.numDrv;
    result_buf = mp_test_result->pOpenResultData;
    nCSub = mp_test_data->Open_test_csub;

    /* if open mode is sine mode, Csub must be zero. */
    if ((mp_test_data->Open_mode == 1))
    	nCSub = 0;

    if (!mp_test_data->Open_test_cfb)
    	nCfb = 2;
    else
    	nCfb = mp_test_data->Open_test_cfb;

    bg_per_csub = (int)(92012 / (11 * nCfb));

    for (i = 0; i < sense_num * drv_num; i++)
    {
        if (deltac_data[i] > 31000)
            return -1;

        result_buf[i] = bg_per_csub * nCSub - deltac_data[i];

        // For mutual key, last column if not be used, show number "one".
        if ((mp_test_data->Mutual_Key == 1 || mp_test_data->Mutual_Key == 2) && (mp_test_data->sensorInfo.numKey != 0))
        {
        	if (mp_test_data->Pattern_type == 5)
        	{
        		// KEY_CH = 1, it mean keys in same drive. Current one key project only KEY_CH = 1 type.
        		if (mp_test_data->sensorInfo.KEY_CH != mp_test_data->sensorInfo.numKey)
        		{
        			if (!((i + 1) % drv_num))
        			{
        				result_buf[i] = NULL_DATA;
        				for (k = 0; k < mp_test_data->sensorInfo.numKey; k++)
        					if ((i + 1) / drv_num == mp_test_data->KeySen[k])
        					{
                                if(mp_test_data->Pattern_model == 1)
                                    result_buf[i] = bg_per_csub * nCSub - deltac_data[i];
                                else
                                    result_buf[i] = 1673 * nCSub - deltac_data[i];
        					}
        			}
        		}
        		else
        		{
        			if (i > ((sense_num - 1) * drv_num - 1))
        			{
        				result_buf[i] = NULL_DATA;
        				for (k = 0; k < mp_test_data->sensorInfo.numKey; k++)
        					if (((i + 1) - (sense_num - 1) * drv_num) == mp_test_data->KeySen[k])
        				    {
                                if(mp_test_data->Pattern_model == 1)
                                    result_buf[i] = bg_per_csub * nCSub - deltac_data[i];
                                else
                                    result_buf[i] = 1673 * nCSub - deltac_data[i];
        				    }
        			}
        		}
        	}
        	else
        	{
        		if ((sense_num < drv_num) && ((i + 1) % drv_num == 0))
        		{
        			result_buf[i] = NULL_DATA;
        			for (k = 0; k < mp_test_data->sensorInfo.numKey; k++)
        				if ((i + 1) / drv_num == mp_test_data->KeySen[k])
        				{
                            result_buf[i] = 1673 * nCSub - deltac_data[i];
        				}
        		}

        		if ((sense_num > drv_num) && (i > (sense_num - 1) * drv_num - 1))
        		{
        			result_buf[i] = NULL_DATA;
        			for (k = 0; k < mp_test_data->sensorInfo.numKey; k++)
        			{
        				if (((i + 1) - (sense_num - 1) * drv_num) == mp_test_data->KeySen[k])
        				{
                            result_buf[i] = 1673 * nCSub - deltac_data[i];
        				}
        			}
        		}
        	}
        }
    }

    memset(normalTestFail_check_Deltac, 0xFFFF, sizeof(normalTestFail_check_Deltac));
    memset(normalTestFail_check_Ratio, 0xFFFF, sizeof(normalTestFail_check_Ratio));
    memset(ratio,0,sizeof(ratio));
    memset(ratio_border,0,sizeof(ratio_border));
    memset(ratio_move,0,sizeof(ratio_move));

    nRowNum = drv_num;
    nColumnNum = sense_num;

    mp_info("*** Msg28xx Open Test# Show _gResult ***\n");
    DebugShowArray2(result_buf, nRowNum*nColumnNum, -32, 10, nColumnNum);
    mp_info("*** Msg28xx Open Test# Show Goldensample ***\n");
    DebugShowArray2(mp_test_data->Goldensample_CH_0, nRowNum*nColumnNum, -32, 10, nColumnNum);

    for (k = 0; k < deltac_size; k++)
    {
    	if (0 == mp_test_data->Goldensample_CH_0[k]) {
    		if (k == 0)
    		    nNormalTestResult[0] = 1;	// no golden sample
    		break;
    	}

    	if (result_buf[k] != NULL_DATA)
    	{
    		ratio[k] = (result_buf[k] * 1000) / mp_test_data->Goldensample_CH_0[k];

    		if (0 == check_thrs(result_buf[k], mp_test_data->Goldensample_CH_0_Max[k], mp_test_data->Goldensample_CH_0_Min[k]))
    		{
    			nNormalTestResult[0] = 1;
    			nNormalTestResultCheck[k] = (u16)(((k / drv_num) + 1) * 100 + ((k % drv_num) + 1));
    		}
    		else
    		{
    			nNormalTestResultCheck[k] = PIN_NO_ERROR;
    			if ((mp_test_data->Pattern_type == 3) && (mp_test_data->sensorInfo.numKey == 0) && ((k % drv_num == 0 )||((k + 1) % drv_num == 0)))
    			{
    				ratioAvg_border += ratio[k];
    				passCount1 += 1;
    			}
    			else if ((mp_test_data->Pattern_type == 3) && (mp_test_data->sensorInfo.numKey != 0) && ((k % drv_num == 0) || ((k + 2) % drv_num == 0)))
    			{
    				ratioAvg_border += ratio[k];
    				passCount1 += 1;
    			}
    			else
    			{
    				ratioAvg += ratio[k];
    				passCount += 1;
    			}
    		}
    	}
    	else
    	{
    		nNormalTestResultCheck[k] = PIN_NO_ERROR;
    	}
    	normalTestFail_check_Deltac[k] = nNormalTestResultCheck[k];
    }

    mp_info("*** Msg28xx Open Test# normalTestFail_check_Deltac Channel ***\n");

    ratioAvg_max = (int)(100000 + (mp_test_data->ToastInfo.persentDC_VA_Ratio * 1000) + (mp_test_data->ToastInfo.persentDC_VA_Ratio_up * 1000)) / 100;
    ratioAvg_min = (int)(100000 - (mp_test_data->ToastInfo.persentDC_VA_Ratio * 1000)) / 100;

    ratioAvg_border_max=(int)(100000 + (mp_test_data->ToastInfo.persentDC_Border_Ratio * 1000) + (mp_test_data->ToastInfo.persentDC_VA_Ratio_up * 1000)) / 100;
    ratioAvg_border_min = (int)(100000 - (mp_test_data->ToastInfo.persentDC_Border_Ratio * 1000)) / 100;

    if (passCount != 0)
    {
        if (passCount1 != 0)
        {
            ratioAvg_border_move = ratioAvg_border / passCount1;

            ratioAvg_move = ratioAvg / passCount;

            for (i = 0; i < sizeof(ratio) / sizeof(ratio[0]); i++)
            {
                if ((mp_test_data->sensorInfo.numKey == 0) && ((i % drv_num == 0) || ((i + 1) % drv_num == 0)))
                {
                    ratio_move[i] = ratio[i] - ratioAvg_border_move + 1000;
                }
                else if ((mp_test_data->sensorInfo.numKey != 0) && ((i % drv_num == 0) || ((i + 2) % drv_num == 0)))
                {
                      ratio_move[i] = ratio[i] - ratioAvg_border_move + 1000;
                }
                else
                {
                    ratio_move[i] = ratio[i] - ratioAvg_move + 1000;
                }

            }
        }
        else
        {
            ratioAvg_move = ratioAvg / passCount;

            for (i = 0; i < sizeof(ratio) / sizeof(ratio[0]); i++)
            {
               ratio_move[i] = ratio[i] - ratioAvg_move + 1000;
            }
        }
    }
    else
    {
        memcpy(ratio, ratio_move, sizeof(ratio));
    }

    /* Based on the size of deltac_data for th loop */
    for (j = 0; j < deltac_size; j++)
    {
        if (0 == mp_test_data->Goldensample_CH_0[j]) {
        	if (j == 0)
        		nNormalTestResult[1] = 1;	// no golden sample
        	break;
        }

        if (PIN_NO_ERROR == nNormalTestResultCheck[j])
        {
            if (result_buf[j] != NULL_DATA)
            {
                if ((mp_test_data->Pattern_type == 3) && (mp_test_data->sensorInfo.numKey == 0) && ((j % drv_num == 0) || ((j + 1) % drv_num == 0)))
                {
                    if (0 == check_thrs(ratio_move[j], ratioAvg_border_max, ratioAvg_border_min))
                    {
                    	nNormalTestResult[1] = 1;
                    	nNormalTestResultCheck[j] = (u16)(((j / drv_num) + 1) * 100 + ((j % drv_num) + 1));
                    }
                    else
                    {
                    	nNormalTestResultCheck[j] = PIN_NO_ERROR;
                    }
                }
                else if ((mp_test_data->Pattern_type == 3) && (mp_test_data->sensorInfo.numKey != 0) && ((j % drv_num == 0) || ((j + 2) % drv_num == 0)))
                {
                    if (0 == check_thrs(ratio_move[j], ratioAvg_border_max, ratioAvg_border_min))
                    {
                    	nNormalTestResult[1] = 1;
                    	nNormalTestResultCheck[j] = (u16)(((j / drv_num) + 1) * 100 + ((j % drv_num) + 1));
                    }
                    else
                    {
                    	nNormalTestResultCheck[j] = PIN_NO_ERROR;
                    }
                }
                else
                {
                    if (0 == check_thrs(ratio_move[j], ratioAvg_max, ratioAvg_min))
                    {
                    	nNormalTestResult[1] = 1;
                    	nNormalTestResultCheck[j] = (u16)(((j / drv_num) + 1) * 100 + ((j % drv_num) + 1));
                    }
                    else
                    {
                    	nNormalTestResultCheck[j] = PIN_NO_ERROR;
                    }
                }
            }
            else
            {
            	nNormalTestResultCheck[j] = PIN_NO_ERROR;
            }
        }
        else
        {
            normalTestFail_check_Ratio[j] = nNormalTestResultCheck[j];
            continue;
        }
        normalTestFail_check_Ratio[j] = nNormalTestResultCheck[j];
    }

    for (k = 0; k < MAX_MUTUAL_NUM; k++)
    {
        if (0 == mp_test_data->Goldensample_CH_0[k])
        {
        	nNormalTestResultCheck[k] = PIN_NO_ERROR;
            normalTestFail_check_Deltac[k] = PIN_NO_ERROR;
            normalTestFail_check_Ratio[k] = PIN_NO_ERROR;
        }
    }

    if ((nNormalTestResult[0] != 0) || (nNormalTestResult[1] != 0))
    	nRetVal = -1;

    for (i = 0; i < 2; i++)
    {
    	mp_test_result->pCheck_Fail[i] = nNormalTestResult[i];
    }
    for (i = 0; i < sizeof(normalTestFail_check_Deltac) / sizeof(normalTestFail_check_Deltac[0]); i++)	// reduce memory operation instead of memcpy
    {
    	mp_test_result->pOpenFailChannel[i] = normalTestFail_check_Deltac[i];
    	mp_test_result->pOpenRatioFailChannel[i] = normalTestFail_check_Ratio[i];
    	mp_test_result->pGolden_CH_Max_Avg[i] = ratio_move[i];
    }

    return nRetVal;
}

int msg30xx_open_judge(int *deltac_data, int deltac_size)
{
    u16 i, j, k;
    u16 nRowNum = 0, nColumnNum = 0;
    u16 nCSub, nCfb;
    s32 nRetVal = 1, bg_per_csub;
    int ratioAvg = 0, ratioAvg_max = 0, ratioAvg_min = 0, passCount = 0;
    int ratioAvg_border = 0, ratioAvg_border_max = 0, ratioAvg_border_min = 0, passCount1 = 0;
    int ratioAvg_move = 0, ratioAvg_border_move = 0;
    int sense_num = 0, drv_num = 0, *result_buf = NULL;

    s8 nNormalTestResult[2] = {0};

    mp_info("*** %s() ***\n", __func__);

    sense_num = mp_test_data->sensorInfo.numSen;
    drv_num = mp_test_data->sensorInfo.numDrv;
    result_buf = mp_test_result->pOpenResultData;
    nCSub = mp_test_data->Open_test_csub;

    /* if open mode is sine mode, Csub must be zero. */
    if ((mp_test_data->Open_mode == 1))
    	nCSub = 0;

    if (!mp_test_data->Open_test_cfb)
    	nCfb = 2;
    else
    	nCfb = mp_test_data->Open_test_cfb;

    bg_per_csub = (int)(92012 / (11 * nCfb));

    for (i = 0; i < sense_num * drv_num; i++)
    {
        if (deltac_data[i] > 31000)
            return -1;

        result_buf[i] = bg_per_csub * nCSub - deltac_data[i];

        // For mutual key, last column if not be used, show number "one".
        if ((mp_test_data->Mutual_Key == 1 || mp_test_data->Mutual_Key == 2) && (mp_test_data->sensorInfo.numKey != 0))
        {
        	if (mp_test_data->Pattern_type == 5)
        	{
        		// KEY_CH = 1, it mean keys in same drive. Current one key project only KEY_CH = 1 type.
        		if (mp_test_data->sensorInfo.KEY_CH != mp_test_data->sensorInfo.numKey)
        		{
        			if (!((i + 1) % drv_num))
        			{
        				result_buf[i] = NULL_DATA;
        				for (k = 0; k < mp_test_data->sensorInfo.numKey; k++)
        					if ((i + 1) / drv_num == mp_test_data->KeySen[k])
        					{
        						// if (_gMsg30xxsssqrt_en)
                                //     result_buf[i] = deltac_data[i];
                                // else
                                    result_buf[i] = bg_per_csub * nCSub - deltac_data[i];
        					}
        			}
        		}
        		else
        		{
        			if (i > ((sense_num - 1) * drv_num - 1))
        			{
        				result_buf[i] = NULL_DATA;
        				for (k = 0; k < mp_test_data->sensorInfo.numKey; k++)
        					if (((i + 1) - (sense_num - 1) * drv_num) == mp_test_data->KeySen[k])
        				    {
        						// if (_gMsg30xxsssqrt_en)
                                //     result_buf[i] = deltac_data[i];
                                // else
                                    result_buf[i] = bg_per_csub * nCSub - deltac_data[i];
        				    }
        			}
        		}
        	}
        	else
        	{
        		if ((sense_num < drv_num) && ((i + 1) % drv_num == 0))
        		{
        			result_buf[i] = NULL_DATA;
        			for (k = 0; k < mp_test_data->sensorInfo.numKey; k++)
        				if ((i + 1) / drv_num == mp_test_data->KeySen[k])
        				{
        					// if (_gMsg30xxsssqrt_en)
                            //     result_buf[i] = deltac_data[i];
                            // else
                                result_buf[i] = bg_per_csub * nCSub - deltac_data[i];
        				}
        		}

        		if ((sense_num > drv_num) && (i > (sense_num - 1) * drv_num - 1))
        		{
        			result_buf[i] = NULL_DATA;
        			for (k = 0; k < mp_test_data->sensorInfo.numKey; k++)
        			{
        				if (((i + 1) - (sense_num - 1) * drv_num) == mp_test_data->KeySen[k])
        				{
        					// if (_gMsg30xxsssqrt_en)
                            //     result_buf[i] = deltac_data[i];
                            // else
                                result_buf[i] = bg_per_csub * nCSub - deltac_data[i];
        				}
        			}
        		}
        	}
        }
    }

    memset(normalTestFail_check_Deltac, 0xFFFF, sizeof(normalTestFail_check_Deltac));
    memset(normalTestFail_check_Ratio, 0xFFFF, sizeof(normalTestFail_check_Ratio));
    memset(ratio,0,sizeof(ratio));
    memset(ratio_border,0,sizeof(ratio_border));
    memset(ratio_move,0,sizeof(ratio_move));

    nRowNum = drv_num;
    nColumnNum = sense_num;

    mp_info("*** Msg30xx Open Test# Show _gResult ***\n");
    DebugShowArray2(result_buf, nRowNum*nColumnNum, -32, 10, nColumnNum);
    mp_info("*** Msg30xx Open Test# Show Goldensample ***\n");
    DebugShowArray2(mp_test_data->Goldensample_CH_0, nRowNum*nColumnNum, -32, 10, nColumnNum);

    for (k = 0; k < deltac_size; k++)
    {
    	if (0 == mp_test_data->Goldensample_CH_0[k]) {
    		if (k == 0)
    		    nNormalTestResult[0] = 1;	// no golden sample
    		break;
    	}

    	if (result_buf[k] != NULL_DATA)
    	{
    		ratio[k] = (result_buf[k] * 1000) / mp_test_data->Goldensample_CH_0[k];

    		if (0 == check_thrs(result_buf[k], mp_test_data->Goldensample_CH_0_Max[k], mp_test_data->Goldensample_CH_0_Min[k]))
    		{
    			nNormalTestResult[0] = 1;
    			nNormalTestResultCheck[k] = (u16)(((k / drv_num) + 1) * 100 + ((k % drv_num) + 1));
    		}
    		else
    		{
    			nNormalTestResultCheck[k] = PIN_NO_ERROR;
    			if ((mp_test_data->Pattern_type == 3) && (mp_test_data->sensorInfo.numKey == 0) && ((k % drv_num == 0 )||((k + 1) % drv_num == 0)))
    			{
    				ratioAvg_border += ratio[k];
    				passCount1 += 1;
    			}
    			else if ((mp_test_data->Pattern_type == 3) && (mp_test_data->sensorInfo.numKey != 0) && ((k % drv_num == 0) || ((k + 2) % drv_num == 0)))
    			{
    				ratioAvg_border += ratio[k];
    				passCount1 += 1;
    			}
    			else
    			{
    				ratioAvg += ratio[k];
    				passCount += 1;
    			}
    		}
    	}
    	else
    	{
    		nNormalTestResultCheck[k] = PIN_NO_ERROR;
    	}
    	normalTestFail_check_Deltac[k] = nNormalTestResultCheck[k];
    }

    mp_info("*** Msg30xx Open Test# normalTestFail_check_Deltac Channel ***\n");

    ratioAvg_max = (int)(100000 + (mp_test_data->ToastInfo.persentDC_VA_Ratio * 1000) + (mp_test_data->ToastInfo.persentDC_VA_Ratio_up * 1000)) / 100;
    ratioAvg_min = (int)(100000 - (mp_test_data->ToastInfo.persentDC_VA_Ratio * 1000)) / 100;

    ratioAvg_border_max=(int)(100000 + (mp_test_data->ToastInfo.persentDC_Border_Ratio * 1000) + (mp_test_data->ToastInfo.persentDC_VA_Ratio_up * 1000)) / 100;
    ratioAvg_border_min = (int)(100000 - (mp_test_data->ToastInfo.persentDC_Border_Ratio * 1000)) / 100;

    if (passCount != 0)
    {
        if (passCount1 != 0)
        {
            ratioAvg_border_move = ratioAvg_border / passCount1;

            ratioAvg_move = ratioAvg / passCount;

            for (i = 0; i < sizeof(ratio) / sizeof(ratio[0]); i++)
            {
                if ((mp_test_data->sensorInfo.numKey == 0) && ((i % drv_num == 0) || ((i + 1) % drv_num == 0)))
                {
                    ratio_move[i] = ratio[i] - ratioAvg_border_move + 1000;
                }
                else if ((mp_test_data->sensorInfo.numKey != 0) && ((i % drv_num == 0) || ((i + 2) % drv_num == 0)))
                {
                      ratio_move[i] = ratio[i] - ratioAvg_border_move + 1000;
                }
                else
                {
                    ratio_move[i] = ratio[i] - ratioAvg_move + 1000;
                }

            }
        }
        else
        {
            ratioAvg_move = ratioAvg / passCount;

            for (i = 0; i < sizeof(ratio) / sizeof(ratio[0]); i++)
            {
               ratio_move[i] = ratio[i] - ratioAvg_move + 1000;
            }
        }
    }
    else
    {
        memcpy(ratio, ratio_move, sizeof(ratio));
    }

    /* Based on the size of deltac_data for th loop */
    for (j = 0; j < deltac_size; j++)
    {
        if (0 == mp_test_data->Goldensample_CH_0[j]) {
        	if (j == 0)
        		nNormalTestResult[1] = 1;	// no golden sample
        	break;
        }

        if (PIN_NO_ERROR == nNormalTestResultCheck[j])
        {
            if (result_buf[j] != NULL_DATA)
            {
                if ((mp_test_data->Pattern_type == 3) && (mp_test_data->sensorInfo.numKey == 0) && ((j % drv_num == 0) || ((j + 1) % drv_num == 0)))
                {
                    if (0 == check_thrs(ratio_move[j], ratioAvg_border_max, ratioAvg_border_min))
                    {
                    	nNormalTestResult[1] = 1;
                    	nNormalTestResultCheck[j] = (u16)(((j / drv_num) + 1) * 100 + ((j % drv_num) + 1));
                    }
                    else
                    {
                    	nNormalTestResultCheck[j] = PIN_NO_ERROR;
                    }
                }
                else if ((mp_test_data->Pattern_type == 3) && (mp_test_data->sensorInfo.numKey != 0) && ((j % drv_num == 0) || ((j + 2) % drv_num == 0)))
                {
                    if (0 == check_thrs(ratio_move[j], ratioAvg_border_max, ratioAvg_border_min))
                    {
                    	nNormalTestResult[1] = 1;
                    	nNormalTestResultCheck[j] = (u16)(((j / drv_num) + 1) * 100 + ((j % drv_num) + 1));
                    }
                    else
                    {
                    	nNormalTestResultCheck[j] = PIN_NO_ERROR;
                    }
                }
                else
                {
                    if (0 == check_thrs(ratio_move[j], ratioAvg_max, ratioAvg_min))
                    {
                    	nNormalTestResult[1] = 1;
                    	nNormalTestResultCheck[j] = (u16)(((j / drv_num) + 1) * 100 + ((j % drv_num) + 1));
                    }
                    else
                    {
                    	nNormalTestResultCheck[j] = PIN_NO_ERROR;
                    }
                }
            }
            else
            {
            	nNormalTestResultCheck[j] = PIN_NO_ERROR;
            }
        }
        else
        {
            normalTestFail_check_Ratio[j] = nNormalTestResultCheck[j];
            continue;
        }
        normalTestFail_check_Ratio[j] = nNormalTestResultCheck[j];
    }

    for (k = 0; k < MAX_MUTUAL_NUM; k++)
    {
        if (0 == mp_test_data->Goldensample_CH_0[k])
        {
        	nNormalTestResultCheck[k] = PIN_NO_ERROR;
            normalTestFail_check_Deltac[k] = PIN_NO_ERROR;
            normalTestFail_check_Ratio[k] = PIN_NO_ERROR;
        }
    }

    if ((nNormalTestResult[0] != 0) || (nNormalTestResult[1] != 0))
    	nRetVal = -1;

    for (i = 0; i < 2; i++)
    {
    	mp_test_result->pCheck_Fail[i] = nNormalTestResult[i];
    }
    for (i = 0; i < sizeof(normalTestFail_check_Deltac) / sizeof(normalTestFail_check_Deltac[0]); i++)	// reduce memory operation instead of memcpy
    {
    	mp_test_result->pOpenFailChannel[i] = normalTestFail_check_Deltac[i];
    	mp_test_result->pOpenRatioFailChannel[i] = normalTestFail_check_Ratio[i];
    	mp_test_result->pGolden_CH_Max_Avg[i] = ratio_move[i];
    }

    return nRetVal;
}
