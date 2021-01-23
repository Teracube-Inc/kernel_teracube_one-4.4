/* drivers/input/touchscreen/gt1x_tpd_custom.h
 *
 * 2010 - 2014 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * Version: 1.4   
 * Release Date:  2015/07/10
 */

#ifndef GT1X_TPD_CUSTOM_H__
#define GT1X_TPD_CUSTOM_H__
#define GPIO_CTP_EN_PIN
#include <asm/uaccess.h>
//#include <linux/rtpm_prio.h>
//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_typedefs.h>
#include "mtk_boot_common.h"
#include "tpd.h"

#define PLATFORM_MTK
#define TPD_I2C_NUMBER		        1
#ifdef CONFIG_MTK_I2C_EXTENSION
#define TPD_SUPPORT_I2C_DMA         1	/* if gt9l, better enable it if hardware platform supported*/
#else
#define TPD_SUPPORT_I2C_DMA         0
#endif

#if (defined(FHDPLUS) || defined(HDPLUS))
#define TPD_HAVE_BUTTON             0	//report key as coordinate,Vibration feedback
#else
#define TPD_HAVE_BUTTON             1	//report key as coordinate,Vibration feedback
#endif
#define GTP_LDOEN_GPIO_OUTPUT(level) tpd_ldoen_gpio_output(level)
#define GTP_GPIO_OUTPUT(pin, level) tpd_gpio_output(pin, level)
#define GTP_GPIO_AS_INT(pin) tpd_gpio_as_int(pin)

/* s960t */
//#define TPD_POWER_SOURCE_CUSTOM	MT6323_POWER_LDO_VGP1	//MT6323_POWER_LDO_VGP1

/* k53v1_64 */
#define TPD_POWER_SOURCE_CUSTOM	PMIC_APP_CAP_TOUCH_VDD

/* y900 */
//#define TPD_POWER_SOURCE_CUSTOM       MT65XX_POWER_LDO_VGP4    //MT6323_POWER_LDO_VGP1

#if TPD_HAVE_BUTTON
#define TPD_KEY_COUNT   4
#if (defined(WVGA) || defined(CU_WVGA) || defined(CMCC_WVGA) || defined(CMCC_LTE_WVGA))
#define key_1           60,870
#define key_2           180,870
#define key_3           300,870
#define key_4           420,870
#elif (defined(FWVGA) || defined(CU_FWVGA) || defined(CMCC_FWVGA) || defined(CMCC_LTE_FWVGA))
#define key_1           60,920
#define key_2           180,920
#define key_3           300,920
#define key_4           420,920
#elif (defined(QHD) || defined(CU_QHD) || defined(CMCC_QHD) || defined(CMCC_LTE_QHD))
#define key_1           85,1030
#define key_2           185,1030
#define key_3           350,1030
#define key_4           500,1030
#elif (defined(HD) || defined(HD720) || defined(CU_HD720) || defined(CMCC_HD720)|| defined(CMCC_LTE_HD720))
#define key_1           90,1350
#define key_2           270,1350
#define key_3           430,1350
#define key_4           630,1350
#elif (defined(FHD) || defined(CU_FHD) || defined(CMCC_FHD) || defined(CMCC_LTE_FHD))
#define key_1           200,2100
#define key_2           500,2100
#define key_3           800,2100
#define key_4           1000,2100
#elif (defined(FHDPLUS))
#define key_1           200,2300
#define key_2           500,2300
#define key_3           800,2300
#define key_4           1000,2300
#elif (defined(HVGA))
#define key_1           40,530
#define key_2           120,530
#define key_3           200,530
#define key_4           280,530
#elif (defined(LQHD))
#define key_1           50,1030
#define key_2           185,1030
#define key_3           350,1030
#define key_4           500,1030
#else
#define key_1           60,920
#define key_2           180,920
#define key_3           300,920
#define key_4           420,920
#endif

#define TPD_KEY_MAP_ARRAY {{key_1},{key_2},{key_3},{key_4}}
#define TPD_KEYS        {KEY_MENU, KEY_HOMEPAGE,KEY_BACK, KEY_SEARCH}
#define TPD_KEYS_DIM    {{key_1,60,50},{key_2,60,50},{key_3,60,50},{key_3,60,50}}

#endif

/*// Change I/O define & I/O operation mode.
#define GTP_RST_PORT    GPIO_CTP_RST_PIN
#define GTP_INT_PORT    GPIO_CTP_EINT_PIN

#define GTP_GPIO_AS_INPUT(pin)          do{\
                                            if(pin == GPIO_CTP_EINT_PIN)\
                                                mt_set_gpio_mode(pin, GPIO_CTP_EINT_PIN_M_EINT);\
                                            else\
                                                mt_set_gpio_mode(pin, GPIO_CTP_RST_PIN_M_GPIO);\
                                            mt_set_gpio_dir(pin, GPIO_DIR_IN);\
                                            mt_set_gpio_pull_enable(pin, GPIO_PULL_DISABLE);\
                                        }while(0)
#define GTP_GPIO_AS_INT(pin)            do{\
                                            mt_set_gpio_mode(pin, GPIO_CTP_EINT_PIN_M_EINT);\
                                            mt_set_gpio_dir(pin, GPIO_DIR_IN);\
                                            mt_set_gpio_pull_enable(pin, GPIO_PULL_DISABLE);\
                                        }while(0)
#define GTP_GPIO_GET_VALUE(pin)         mt_get_gpio_in(pin)
#define GTP_GPIO_OUTPUT(pin,level)      do{\
                                            if(pin == GPIO_CTP_EINT_PIN)\
                                                mt_set_gpio_mode(pin, GPIO_CTP_EINT_PIN_M_GPIO);\
                                            else\
                                                mt_set_gpio_mode(pin, GPIO_CTP_RST_PIN_M_GPIO);\
                                            mt_set_gpio_dir(pin, GPIO_DIR_OUT);\
                                            mt_set_gpio_out(pin, level);\
                                        }while(0)

#define GTP_IRQ_TAB                     {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH}
*/
#ifdef MT6589
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
#define mt_eint_mask mt65xx_eint_mask
#define mt_eint_unmask mt65xx_eint_unmask
#endif

#define IIC_MAX_TRANSFER_SIZE         8
#define IIC_DMA_MAX_TRANSFER_SIZE     250
#define I2C_MASTER_CLOCK              300

#define TPD_MAX_RESET_COUNT           3

#define TPD_HAVE_CALIBRATION
#define TPD_CALIBRATION_MATRIX        {962,0,0,0,1600,0,0,0};

extern void tpd_on(void);
extern void tpd_off(void);

#endif /* GT1X_TPD_CUSTOM_H__ */
