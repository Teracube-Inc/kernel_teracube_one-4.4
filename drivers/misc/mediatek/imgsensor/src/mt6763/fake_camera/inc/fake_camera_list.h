#ifndef FAKE_CAMERA_LIST
#define FAKE_CAMERA_LIST

#include "fake_camera_define.h"

/*init yuvSensorList*/
YUV_SENSOR yuvSensorList[] = {

#if defined(GC0310_MIPI_YUV)
  {GC0310_YUV_ID,GC0310_YUV_NAME,GC0310_YUV_SENSOR_INIT},
#endif

#if defined(SP0A20_MIPI_YUV)
  {SP0A20_YUV_ID,SP0A20_YUV_NAME,SP0A20_YUV_SENSOR_INIT},
#endif

#if defined(SP0A38_MIPI_YUV)
  {SP0A38_YUV_ID,SP0A38_YUV_NAME,SP0A38_YUV_SENSOR_INIT},
#endif

#if defined(GC033A_MIPI_YUV)
  {GC033A_YUV_ID,GC033A_YUV_NAME,GC033A_YUV_SENSOR_INIT},
#endif

#if defined(SP0A20SUB_MIPI_YUV)
  {SP0A20SUB_YUV_ID,SP0A20SUB_YUV_NAME,SP0A20SUB_YUV_SENSOR_INIT},
#endif
/*add yuv sensor driver before this line */
  {0,{0},NULL}, //end of list

};

#endif
