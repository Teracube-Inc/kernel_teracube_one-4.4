/*
 * Copyright (C) 2015 MediaTek Inc.
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

/* Stoneoim:hanshengpeng on: Mon, 04 Dec 2017 17:41:45 +0800
 * for build lk lcm
 */
#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>

#define LOG_TAG "LCM"
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)

#define I2C_MT6370_PMU_CHANNEL      5
#define I2C_MT6370_PMU_SLAVE_7_BIT_ADDR 0x34

static int RT5081_read_byte (kal_uint8 addr, kal_uint8 *dataBuffer)
{
    kal_uint32 ret = I2C_OK;
    kal_uint16 len;
    struct mt_i2c_t RT5081_i2c;
    *dataBuffer = addr;

    RT5081_i2c.id = I2C_MT6370_PMU_CHANNEL;
    RT5081_i2c.addr = I2C_MT6370_PMU_SLAVE_7_BIT_ADDR;
    RT5081_i2c.mode = ST_MODE;
    RT5081_i2c.speed = 100;
    len = 1;

    ret = i2c_write_read(&RT5081_i2c, dataBuffer, len, len);
    if (I2C_OK != ret)
        LCM_LOGI("%s: i2c_read  failed! ret: %d\n", __func__, ret);

    return ret;
}

static int RT5081_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;
    struct mt_i2c_t RT5081_i2c;

    write_data[0] = addr;
    write_data[1] = value;

    RT5081_i2c.id = I2C_MT6370_PMU_CHANNEL;
    RT5081_i2c.addr = I2C_MT6370_PMU_SLAVE_7_BIT_ADDR;
    RT5081_i2c.mode = ST_MODE;
    RT5081_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&RT5081_i2c, write_data, len);

    return ret_code;
}

static int RT5081_REG_MASK (kal_uint8 addr, kal_uint8 val, kal_uint8 mask)
{
    kal_uint8 RT5081_reg = 0;
    kal_uint32 ret = 0;

    ret = RT5081_read_byte(addr, &RT5081_reg);

    RT5081_reg &= ~mask;
    RT5081_reg |= val;

    ret = RT5081_write_byte(addr, RT5081_reg);

    return ret;
}

int display_bias_enable(void)
{
	int ret = 0;
    /*config rt5081 register 0xB2[7:6]=0x3, that is set db_delay=4ms.*/
    ret = RT5081_REG_MASK(0xB2, (0x3 << 6), (0x3 << 6));

    /* set AVDD 5.4v, (4v+28*0.05v) */
    /*ret = RT5081_write_byte(0xB3, (1 << 6) | 28);*/
    ret = RT5081_REG_MASK(0xB3, 28, (0x3F << 0));
    if (ret < 0)
        LCM_LOGI("nt35695----tps6132----cmd=%0x--i2c write error----\n", 0xB3);
    else
        LCM_LOGI("nt35695----tps6132----cmd=%0x--i2c write success----\n", 0xB3);

    /* set AVEE */
    /*ret = RT5081_write_byte(0xB4, (1 << 6) | 28);*/
    ret = RT5081_REG_MASK(0xB4, 28, (0x3F << 0));
    if (ret < 0)
        LCM_LOGI("nt35695----tps6132----cmd=%0x--i2c write error----\n", 0xB4);
    else
        LCM_LOGI("nt35695----tps6132----cmd=%0x--i2c write success----\n", 0xB4);

    /* enable AVDD & AVEE */
    /* 0x12--default value; bit3--Vneg; bit6--Vpos; */
    /*ret = RT5081_write_byte(0xB1, 0x12 | (1<<3) | (1<<6));*/
    ret = RT5081_REG_MASK(0xB1, (1<<3) | (1<<6), (1<<3) | (1<<6));
    if (ret < 0)
        LCM_LOGI("nt35695----tps6132----cmd=%0x--i2c write error----\n", 0xB1);
    else
        LCM_LOGI("nt35695----tps6132----cmd=%0x--i2c write success----\n", 0xB1);

    return ret;
}

int display_bias_disable(void)
{
	int ret = 0;
    /* enable AVDD & AVEE */
    /* 0x12--default value; bit3--Vneg; bit6--Vpos; */
    /*ret = RT5081_write_byte(0xB1, 0x12);*/
    ret = RT5081_REG_MASK(0xB1, (0<<3) | (0<<6), (1<<3) | (1<<6));
    if (ret < 0)
        LCM_LOGI("nt35695----tps6132----cmd=%0x--i2c write error----\n", 0xB1);
    else
        LCM_LOGI("nt35695----tps6132----cmd=%0x--i2c write success----\n", 0xB1);

    return ret;
}

#else

#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;
static int regulator_inited;

int display_bias_regulator_init(void)
{
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_err("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_err("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}
EXPORT_SYMBOL(display_bias_regulator_init);

int display_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

#if 0
	/* get voltage */
	ret = mtk_regulator_get_voltage(&disp_bias_pos);
	if (ret < 0)
		pr_err("get voltage disp_bias_pos fail\n");
	pr_debug("pos voltage = %d\n", ret);

	ret = mtk_regulator_get_voltage(&disp_bias_neg);
	if (ret < 0)
		pr_err("get voltage disp_bias_neg fail\n");
	pr_debug("neg voltage = %d\n", ret);
#endif
	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_enable);

int display_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_disable);

#else
int display_bias_regulator_init(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_regulator_init);

int display_bias_enable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_enable);

int display_bias_disable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_disable);
#endif

#endif
// End of Stoneoim:hanshengpeng
