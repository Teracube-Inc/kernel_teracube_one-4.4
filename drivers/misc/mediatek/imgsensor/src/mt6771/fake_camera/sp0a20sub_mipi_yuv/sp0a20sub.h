/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.h
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   CMOS sensor header file
 *
 ****************************************************************************/
#ifndef __SENSOR_H
#define __SENSOR_H

#include "kd_camera_typedef.h"

#define SP0A20SUB_WRITE_ID								0x42
#define SP0A20SUB_READ_ID								0x43

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

UINT32 sp0a20sub_read_id(void);
void sp0a20sub_open(void);
UINT16 SP0A20SUB_MIPI_GetYUVSensorBV(void);

#endif /* __SENSOR_H */ 
