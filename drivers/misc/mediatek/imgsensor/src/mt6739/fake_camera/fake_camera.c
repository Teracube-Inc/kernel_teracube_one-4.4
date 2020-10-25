/*
design for fake dual camera
*/
#include <linux/kernel.h>
#include "fake_camera_define.h"
#include "fake_camera_list.h"

static int listNum = -1;
static bool isExist = false;
static PYUV_SENSOR_FUNC pfFunc;
static PYUV_SENSOR_FUNC *pfFuncs = &pfFunc;
extern void v_set_dev_name(int id, char *name);

UINT32 sensor_num = sizeof(yuvSensorList) / sizeof(YUV_SENSOR) ;

static void check_yuv_sensor_id(void)
{
  UINT32 i;

  for(i = 0 ; i < sensor_num - 1;i ++)
  {
    if(1 == yuvSensorList[i].sensor_init(pfFuncs))
    {
      isExist= true;
      listNum = i;

#ifdef VANZO_DEVICE_NAME_SUPPORT
          extern void v_set_dev_name(int id, char *name);
            v_set_dev_name(4,yuvSensorList[i].sensor_name);
           //v_set_dev_name(4, (char *)psensor_inst->psensor_name);
#endif

      YUV_DBG("listNum = %d sensor_num = %d\n",listNum,sensor_num-1);
      break;
    }
  }
  if(!isExist)
  {
    YUV_DBG("search no sensor exist !\n");
  }
}

static UINT16 yuv_sensor_read_shutter(void)
{
  UINT16 light = 0;

  if((NULL != pfFuncs) && (isExist == true))
    light = (*pfFuncs)->getYUVSensorBV();
  else
    YUV_DBG("get bv fail!\n");

  return light;
}

void GetYUVSensorBV(UINT32 *val)
{
    *val = yuv_sensor_read_shutter();
    YUV_DBG("GetYUVSensorBV val:%d\n",*val);
}

int openYuvSensor(void)
{
  check_yuv_sensor_id();

  if((NULL != pfFuncs) && (isExist == true))
  {
    (*pfFuncs)->open();
  }
  else
  {
    YUV_DBG("open yuv sensor fail\n!");
    return -1;
  }
  return yuvSensorList[listNum].sensor_id;
}
