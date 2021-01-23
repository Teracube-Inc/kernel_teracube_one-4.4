#ifndef PT_COMMON
#define PT_COMMON

/*define to enable Device Tree support*/
#define CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT

/*define to enable I2C bus interface to Parade TrueTouch*/
#define CONFIG_TOUCHSCREEN_PARADE_I2C

/*Select to enable MultiTouch touch reporting using protocol A*/
//#define CONFIG_TOUCHSCREEN_PARADE_MT_A

/*Select to enable MultiTouch touch reporting using protocol B*/
#define CONFIG_TOUCHSCREEN_PARADE_MT_B

/*define to enable CapSense reporting on Parade TrueTouch*/
//#define CONFIG_TOUCHSCREEN_PARADE_BUTTON

/*define to enable proximity reporting on Parade TrueTouch*/
//#define CONFIG_TOUCHSCREEN_PARADE_PROXIMITY

/*define to enable FW upgrade from header file*/
//#define CONFIG_TOUCHSCREEN_PARADE_PLATFORM_FW_UPGRADE

/*define to enable FW upgrade from binary file*/
#define CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE

/*define to enable TT Configuration upgrade from header file*/
//#define CONFIG_TOUCHSCREEN_PARADE_PLATFORM_TTCONFIG_UPGRADE

/*define to enable TT Configuration upgrade via SysFs*/
//#define CONFIG_TOUCHSCREEN_PARADE_MANUAL_TTCONFIG_UPGRADE

#endif
