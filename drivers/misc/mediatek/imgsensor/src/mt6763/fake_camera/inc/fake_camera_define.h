/*
design for fake dual camera
*/
#ifndef FAKE_CAMERA_DEFINE
#define FAKE_CAMERA_DEFINE

#include "kd_camera_typedef.h"

/* debug log setting */
#define YUV_DEBUG
#ifdef YUV_DEBUG
#define YUV_DBG(a,arg...) printk("yuv_log:" a,##arg)
#else
#define YUV_DBG(a,arg...)
#endif

/*define yuv sensor id*/
#define GC0310_YUV_ID                   0xa310
#define SP0A20_YUV_ID                   0x002b
#define SP0A20SUB_YUV_ID                0x002c
#define SP0A38_YUV_ID                   0x0a10
#define GC033A_YUV_ID                   0x033a

/*define yuv sensor name*/
#define GC0310_YUV_NAME                 "gc0310mipiyuv"
#define SP0A20_YUV_NAME                 "sp0a20mipiyuv"
#define SP0A20SUB_YUV_NAME              "sp0a20submipiyuv"
#define SP0A38_YUV_NAME                 "sp0a38mipiyuv"
#define GC033A_YUV_NAME                 "gc033amipiyuv"

/*define struct YUV_SENSOR_FUNC*/
typedef struct {

  void (*open)(void);
  UINT16 (*getYUVSensorBV)(void);

} YUV_SENSOR_FUNC, *PYUV_SENSOR_FUNC;

/*define struct YUV_SENSOR*/
typedef struct {

  int sensor_id;
  char sensor_name[16];
  UINT32 (*sensor_init)(PYUV_SENSOR_FUNC*);

} YUV_SENSOR;

/*declare yuv sensor init func*/
UINT32 GC0310_YUV_SENSOR_INIT(PYUV_SENSOR_FUNC *pfFunc);
UINT32 SP0A20_YUV_SENSOR_INIT(PYUV_SENSOR_FUNC *pfFunc);
UINT32 SP0A20SUB_YUV_SENSOR_INIT(PYUV_SENSOR_FUNC *pfFunc);
UINT32 SP0A38_YUV_SENSOR_INIT(PYUV_SENSOR_FUNC *pfFunc);
UINT32 GC033A_YUV_SENSOR_INIT(PYUV_SENSOR_FUNC *pfFunc);

#endif
