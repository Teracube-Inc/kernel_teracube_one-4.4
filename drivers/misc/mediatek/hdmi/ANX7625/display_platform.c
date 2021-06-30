/******************************************************************************
*
Copyright (c) 2016, Analogix Semiconductor, Inc.

PKG Ver  : V1.0

Filename :

Project  : ANX7625

Created  : 02 Oct. 2016

Devices  : ANX7625

Toolchain: Android

Description:

Revision History:

******************************************************************************/
/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include "display_platform.h"
#include "display.h"
#include "anx7625_driver.h"

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#endif

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include "extd_platform.h"
#ifdef CONFIG_MACH_MT6799
#include "mt6336.h"
#include "mt-plat/upmu_common.h"
#endif

#ifdef CONFIG_IO_DRIVING
#include "mt-plat/sync_write.h"
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#define HDMI_REG_SET_FIELD(field, reg32, val)  \
	do {  \
		unsigned int regval; \
		regval = __raw_readl((/*volatile*/unsigned long*)(reg32)); \
		regval  = ((regval & (~(field))) | val); \
		mt_reg_sync_writel(regval, (reg32));  \
	} while (0)
#endif



/*unsigned int eint_pin_num = 140;*/

/**
* LOG For HDMI Driver
*/

#define SLIMPORT_TAG "[EXTD][DISP] "

#define SLIMPORT_DBG(fmt, arg...) do { \
		if (slimport_log_on) \
			pr_err(SLIMPORT_TAG "%s: " fmt, __func__, ##arg); \
	} while (0)

#ifdef NEVER
#define SLIMPORT_DBG(fmt, arg...) \
	pr_err("[EXTD][DISP]"fmt, ##arg)

#endif /* NEVER */

struct i2c_dev_info {
	uint8_t			dev_addr;
	struct i2c_client	*client;
};

#define I2C_DEV_INFO(addr) \
	{.dev_addr = addr >> 1, .client = NULL}


int	debug_msgs	= 3;	/* print all msgs, default should be '0'*/

/* request to reset hw before unloading driver*/
static bool reset_on_exit;
#ifdef CONFIG_MACH_MT6799
struct mt6336_ctrl *mt6336_ctrl;
#endif

module_param(debug_msgs, int, S_IRUGO);
module_param(reset_on_exit, bool, S_IRUGO);

#define USE_DEFAULT_I2C_CODE 0

struct i2c_client *mClient;
unsigned int mhl_eint_number = 0xffff;
unsigned int mhl_comm_eint_number = 0xffff;

static unsigned int mask_flag;
static unsigned int mask_flag_comm;

struct anx7625_platform_gpio_data {
	int anx7625_gpio_p_on;
	int anx7625_gpio_reset;
	int anx7625_gpio_cbl_det;
	int anx7625_gpio_intr_comm;
};

struct anx7625_platform_gpio_data *anx7625_pdata;

int get_mhl_irq_num(void)
{
	return mhl_eint_number;
}
int get_mhl_comm_irq_num(void)
{
	return mhl_comm_eint_number;
}

int confirm_mhl_irq_level(void)
{
	return gpio_get_value(anx7625_pdata->anx7625_gpio_cbl_det);
}

int confirm_mhl_comm_irq_level(void)
{
	return gpio_get_value(anx7625_pdata->anx7625_gpio_intr_comm);
}


void Mask_Slimport_Intr(bool irq_context)
{
	SLIMPORT_DBG("Mask_Slimport_Intr: in\n");

	SLIMPORT_DBG("Mask_Slimport_Intr: mask_flag:%d\n", mask_flag);

	if (mask_flag == 0) {
		if (irq_context)
			disable_irq_nosync(get_mhl_irq_num());
		else
			disable_irq(get_mhl_irq_num());
		mask_flag++;
	}

}

void Unmask_Slimport_Intr(void)
{
	SLIMPORT_DBG("Unmask_Slimport_Intr: mask_flag:%d\n", mask_flag);
	if (mask_flag != 0) {
		enable_irq(get_mhl_irq_num());
		mask_flag = 0;
	}
}

void Mask_Slimport_Comm_Intr(bool irq_context)
{
	SLIMPORT_DBG("Mask_Slimport_Comm_Intr: mask_flag:%d\n", mask_flag_comm);

	if (mask_flag_comm == 0) {
		if (irq_context)
			disable_irq_nosync(get_mhl_comm_irq_num());
		else
			disable_irq(get_mhl_comm_irq_num());
		mask_flag_comm++;
	}

}

void Unmask_Slimport_Comm_Intr(void)
{
	SLIMPORT_DBG("Unmask_Slimport_Comm_Intr: mask_flag:%d\n",
		mask_flag_comm);

	if (mask_flag_comm != 0) {
		enable_irq(get_mhl_comm_irq_num());
		mask_flag_comm = 0;
	}

}

void register_slimport_eint(void)
{
	struct device_node *node = NULL;
	u32 ints[2] = {0, 0};

	SLIMPORT_DBG("register_slimport_eint\n");
	node = of_find_compatible_node(NULL, NULL, "mediatek,extd_dev");
	if (node) {
		of_property_read_u32_array(node, "debounce",
			ints, ARRAY_SIZE(ints));
		mhl_eint_number = irq_of_parse_and_map(node, 0);
		SLIMPORT_DBG("mhl_eint_number, node %p-irq %d!!\n",
			node, get_mhl_irq_num());
		/*debounce time is microseconds*/
		gpio_set_debounce(anx7625_pdata->anx7625_gpio_cbl_det, 50000);
		if (request_irq(mhl_eint_number, anx7625_cbl_det_isr,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			IRQF_ONESHOT,
			"mediatek,extd_dev", NULL)) {
			SLIMPORT_DBG("request_irq fail\n");
		} else {
			SLIMPORT_DBG("MHL EINT IRQ node %p-irq %d!!\n",
				node, get_mhl_irq_num());
			Mask_Slimport_Intr(false);
			enable_irq_wake(mhl_eint_number);
			return;
		}
	} else {
		SLIMPORT_DBG("of_find_compatible_node error\n");
	}
	Mask_Slimport_Intr(false);
	SLIMPORT_DBG("Error: MHL EINT IRQ NOT AVAILABLE, node %p-irq %d!!\n",
		node, get_mhl_irq_num());
}

void register_slimport_comm_eint(void)
{
	struct device_node *node = NULL;
	u32 ints[2] = {0, 0};

	SLIMPORT_DBG("register_slimport_eint\n");
	node = of_find_compatible_node(NULL, NULL, "mediatek,hdmi_dev");
	if (node) {
		of_property_read_u32_array(node, "debounce",
			ints, ARRAY_SIZE(ints));
		mhl_comm_eint_number = irq_of_parse_and_map(node, 0);
		SLIMPORT_DBG("mhl_eint_number, node %p-irq %d!!\n",
			node, get_mhl_comm_irq_num());
		if (request_irq(mhl_comm_eint_number, anx7625_intr_comm_isr,
			IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
			"mediatek,hdmi_dev", NULL)) {
			SLIMPORT_DBG("request_irq fail\n");
		} else {
			SLIMPORT_DBG("MHL EINT IRQ node %p-irq %d!!\n",
				node, get_mhl_comm_irq_num());
			Mask_Slimport_Comm_Intr(false);
			return;
		}
	} else {
		SLIMPORT_DBG("of_find_compatible_node error\n");
	}
	Mask_Slimport_Comm_Intr(false);
	SLIMPORT_DBG("Error: MHL EINT IRQ NOT AVAILABLE, node %p-irq %d!!\n",
		node, get_mhl_comm_irq_num());
}

struct pinctrl *mhl_pinctrl;
struct pinctrl_state *pin_state;
struct regulator *reg_v10_power;
struct regulator *reg_v18_power;
struct regulator *reg_v30_power;

char *dpi_gpio_name[32] = {
	"dpi_d0_def", "dpi_d0_cfg", "dpi_d1_def", "dpi_d1_cfg",
	"dpi_d2_def", "dpi_d2_cfg", "dpi_d3_def", "dpi_d3_cfg",
	"dpi_d4_def", "dpi_d4_cfg", "dpi_d5_def", "dpi_d5_cfg",
	"dpi_d6_def", "dpi_d6_cfg", "dpi_d7_def", "dpi_d7_cfg",
	"dpi_d8_def", "dpi_d8_cfg", "dpi_d9_def", "dpi_d9_cfg",
	"dpi_d10_def", "dpi_d10_cfg", "dpi_d11_def", "dpi_d11_cfg",
	"dpi_ck_def", "dpi_ck_cfg", "dpi_de_def", "dpi_de_cfg",
	"dpi_hsync_def", "dpi_hsync_cfg", "dpi_vsync_def", "dpi_vsync_cfg"
};

char *i2s_gpio_name[10] = {
	"i2s_dat_def", "i2s_dat_cfg", "i2s_ws_def", "i2s_ws_cfg",
	"i2s_ck_def", "i2s_ck_cfg", "i2s_dat1_def", "i2s_dat1_cfg",
	"i2s_dat2_def", "i2s_dat2_cfg"
};

char *rst_gpio_name[2] = {
	"rst_low_cfg", "rst_high_cfg"
};

char *eint_gpio_name[1] = {
	"eint_input_cfg"
};

char *eint_comm_name[1] = {
	"comm_input_cfg"
};

char *power_ctl_gpio_name[2] = {
	"power_ctrl_low_cfg", "power_ctrl_high_cfg"
};

void dpi_gpio_ctrl(int enable)
{
	int offset = 0;
	int ret = 0;
#ifdef CONFIG_IO_DRIVING
	struct device_node *node;
	void __iomem *iocfg_1_base;
#endif

#ifdef ANX7625_MTK_PLATFORM
	if (dst_is_dsi)
		return;
#endif

	SLIMPORT_DBG("dpi_gpio_ctrl+  %ld !!\n", sizeof(dpi_gpio_name));
	if (IS_ERR(mhl_pinctrl)) {
		ret = PTR_ERR(mhl_pinctrl);
		SLIMPORT_DBG("No find MHL RST pinctrl for dpi_gpio_ctrl!\n");
		return;
	}

	if (enable)
		offset = 1;

	for (; offset < 32 ;) {
		pin_state = pinctrl_lookup_state(
			mhl_pinctrl, dpi_gpio_name[offset]);
		if (IS_ERR(pin_state)) {
			ret = PTR_ERR(pin_state);
			SLIMPORT_DBG("Cannot find MHL pinctrl--%s!!\n",
				dpi_gpio_name[offset]);
		} else
			pinctrl_select_state(mhl_pinctrl, pin_state);

		offset += 2;
	}

#ifdef CONFIG_IO_DRIVING
	/* config DPI IO_DRIVING*/
#ifdef CONFIG_MACH_MT6799
	node = of_find_compatible_node(NULL, NULL, "mediatek,iocfg_rb");
	if (!node)
		pr_debug("[iocfg_rb] find node failed\n");

	iocfg_1_base = of_iomap(node, 0);
	if (!iocfg_1_base)
		pr_debug("[iocfg_rb] base failed\n");
	else {
		if (enable)
			HDMI_REG_SET_FIELD(0xF00, (iocfg_1_base + 0xB0), 0x400);
		else
			HDMI_REG_SET_FIELD(0xF00, (iocfg_1_base + 0xB0), 0x0);
	}
#endif
#endif

}

void i2s_gpio_ctrl(int enable)
{
	int offset = 0;
	int ret = 0;

	SLIMPORT_DBG("i2s_gpio_ctrl+  %ld !!\n", sizeof(i2s_gpio_name));

	if (IS_ERR(mhl_pinctrl)) {
		ret = PTR_ERR(mhl_pinctrl);
		SLIMPORT_DBG("No find MHL RST pinctrl for i2s_gpio_ctrl!\n");
		return;
	}

	if (enable)
		offset = 1;

#ifdef MTK_AUDIO_MULTI_CHANNEL_SUPPORT
	for (; offset < 10 ;) {

		pin_state = pinctrl_lookup_state(
			mhl_pinctrl, i2s_gpio_name[offset]);
		if (IS_ERR(pin_state)) {
			ret = PTR_ERR(pin_state);
			SLIMPORT_DBG("No find MHL pinctrl--%s!!\n",
				i2s_gpio_name[offset]);
		} else
			pinctrl_select_state(mhl_pinctrl, pin_state);

		offset += 2;
	}
#else
	for (; offset < 6 ;) {
		pin_state = pinctrl_lookup_state(
			mhl_pinctrl, i2s_gpio_name[offset]);
		if (IS_ERR(pin_state)) {
			ret = PTR_ERR(pin_state);
			SLIMPORT_DBG("Cannot find MHL pinctrl--%s!!\n",
				i2s_gpio_name[offset]);
		} else
			pinctrl_select_state(mhl_pinctrl, pin_state);

		offset += 2;
	}
#endif

}

void mhl_power_ctrl(int enable)
{
	struct pinctrl_state *power_low_state = NULL;
	struct pinctrl_state *power_high_state = NULL;
	int err_cnt = 0;
	int ret = 0;

	SLIMPORT_DBG("slimport_power_ctrl+	%ld, enable: %d !!\n", sizeof(power_ctl_gpio_name), enable);
	if (IS_ERR(mhl_pinctrl)) {
		ret = PTR_ERR(mhl_pinctrl);
		SLIMPORT_DBG("Cannot find DP POWER_DOWN pinctrl for mhl_power_ctrl!\n");
		return;
	}

	power_low_state = pinctrl_lookup_state(mhl_pinctrl, power_ctl_gpio_name[0]);
	if (IS_ERR(power_low_state)) {
		ret = PTR_ERR(power_low_state);
		SLIMPORT_DBG("Cannot find DP pinctrl--%s!!\n", power_ctl_gpio_name[0]);
		err_cnt++;
	}

	power_high_state = pinctrl_lookup_state(mhl_pinctrl, power_ctl_gpio_name[1]);
	if (IS_ERR(power_high_state)) {
		ret = PTR_ERR(power_high_state);
		SLIMPORT_DBG("Cannot find DP pinctrl--%s!!\n", power_ctl_gpio_name[1]);
		err_cnt++;
	}

	if (err_cnt > 0) {
		cust_power_on(enable);
		return;
	}

	if (enable) {
		/* out high */
		SLIMPORT_DBG("%s PWR_EN HIGH\n", __func__);
		pinctrl_select_state(mhl_pinctrl, power_high_state);
	} else {
		SLIMPORT_DBG("%s PWR_EN LOW\n", __func__);
		pinctrl_select_state(mhl_pinctrl, power_low_state);
	}
}

void reset_mhl_board(int hwResetPeriod, int hwResetDelay, int is_power_on)
{
	struct pinctrl_state *rst_low_state = NULL;
	struct pinctrl_state *rst_high_state = NULL;
	int err_cnt = 0;
	int ret = 0;

	SLIMPORT_DBG("reset_mhl_board+	%ld, is_power_on: %d !!\n", sizeof(rst_gpio_name), is_power_on);
	if (IS_ERR(mhl_pinctrl)) {
		ret = PTR_ERR(mhl_pinctrl);
		SLIMPORT_DBG("Cannot find MHL RST pinctrl for reset_mhl_board!\n");
		return;
	}

	rst_low_state = pinctrl_lookup_state(mhl_pinctrl, rst_gpio_name[0]);
	if (IS_ERR(rst_low_state)) {
		ret = PTR_ERR(rst_low_state);
		SLIMPORT_DBG("Cannot find MHL pinctrl--%s!!\n", rst_gpio_name[0]);
		err_cnt++;
	}

	rst_high_state = pinctrl_lookup_state(mhl_pinctrl, rst_gpio_name[1]);
	if (IS_ERR(rst_high_state)) {
		ret = PTR_ERR(rst_high_state);
		SLIMPORT_DBG("Cannot find MHL pinctrl--%s!!\n", rst_gpio_name[1]);
		err_cnt++;
	}

	if (err_cnt > 0) {
		cust_reset(hwResetPeriod, hwResetDelay, is_power_on);
		return;
	}

	if (is_power_on) {
		pinctrl_select_state(mhl_pinctrl, rst_high_state);
		mdelay(hwResetPeriod);
		pinctrl_select_state(mhl_pinctrl, rst_low_state);
		mdelay(hwResetPeriod);
		pinctrl_select_state(mhl_pinctrl, rst_high_state);
		mdelay(hwResetDelay);
	} else {
		pinctrl_select_state(mhl_pinctrl, rst_low_state);
		mdelay(hwResetPeriod);
	}
}

void set_pin_high_low(enum PIN_TYPE pin, bool is_high)
{
	struct pinctrl_state *pin_state = NULL;
	char *str = NULL;
	int ret = 0;

	SLIMPORT_DBG("reset_mhl_board+  %d, is_power_on: %d !!\n",
		pin, is_high);
	if (IS_ERR(mhl_pinctrl)) {
		ret = PTR_ERR(mhl_pinctrl);
		SLIMPORT_DBG("No find MHL RST pinctrl for reset_mhl_board!\n");
		return;
	}

	if (pin == RESET_PIN && is_high == false)
		str = rst_gpio_name[0];
	else if (pin == RESET_PIN && is_high == true)
		str = rst_gpio_name[1];
	else if (pin == PD_PIN && is_high == false)
		str = power_ctl_gpio_name[0];
	else if (pin == PD_PIN && is_high == true)
		str = power_ctl_gpio_name[1];

	SLIMPORT_DBG("pinctrl--%s!!\n", str);
	pin_state = pinctrl_lookup_state(mhl_pinctrl, str);
	if (IS_ERR(pin_state)) {
		ret = PTR_ERR(pin_state);
		SLIMPORT_DBG("Cannot find MHL pinctrl--%s!!\n", str);
	}

	pinctrl_select_state(mhl_pinctrl, pin_state);

}

void cust_power_init(void)
{

}

void cust_power_on(int enable)
{
#ifdef CONFIG_MACH_MT6799
	if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
					"mediatek/evb99v1_64_ufs_mhl", 27) == 0) {
		/* Enable mt6336 controller before use mt6336 */
		mt6336_ctrl_enable(mt6336_ctrl);
		/* Config MT6336 GPIO3(MT6336 HW GPIO3 == MT6336  SW GPIO7) */
		if (enable) {
			/* out high */
			SLIMPORT_DBG("%s PWR_EN HIGH\n", __func__);
			mt6336_set_flag_register_value(MT6336_GPIO_DOUT0_SET, 0x80);
		} else {
			SLIMPORT_DBG("%s PWR_EN LOW\n", __func__);
			mt6336_set_flag_register_value(MT6336_GPIO_DOUT0_CLR, 0x80);
		}
		/* Disable mt6336 controller when unuse mt6336 */
		mt6336_ctrl_disable(mt6336_ctrl);
	} else if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
					"mediatek/k99v2na_64", 19) == 0) {
		/* Enable mt6336 controller before use mt6336 */
		mt6336_ctrl_enable(mt6336_ctrl);
		/* Config MT6336 GPIO1(MT6336 HW GPIO1 == MT6336  SW GPIO5) */
		if (enable) {
			/* out high */
			SLIMPORT_DBG("%s PWR_EN HIGH\n", __func__);
			mt6336_set_flag_register_value(MT6336_GPIO_DOUT0_SET, 0x20);
		} else {
			SLIMPORT_DBG("%s PWR_EN LOW\n", __func__);
			mt6336_set_flag_register_value(MT6336_GPIO_DOUT0_CLR, 0x20);
		}
		/* Disable mt6336 controller when unuse mt6336 */
		mt6336_ctrl_disable(mt6336_ctrl);
	}
#endif
}

void cust_reset(int hwResetPeriod, int hwResetDelay, int is_power_on)
{
#ifdef CONFIG_MACH_MT6799
	if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
						"mediatek/k99v2na_64", 19) == 0) {
		/* Enable mt6336 controller before use mt6336 */
		mt6336_ctrl_enable(mt6336_ctrl);
		/* Config MT6336 GPIO3(MT6336 HW GPIO2 == MT6336  SW GPIO6) */
		/* out low*/
		SLIMPORT_DBG("%s RESET_N LOW\n", __func__);
		mt6336_set_flag_register_value(MT6336_GPIO_DOUT0_CLR, 0x40);
		usleep_range(hwResetPeriod * 1000, (hwResetPeriod + 1) * 1000);

		if (is_power_on) {
			/* out high */
			SLIMPORT_DBG("%s RESET_N HIGH\n", __func__);
			mt6336_set_flag_register_value(MT6336_GPIO_DOUT0_SET, 0x40);
			usleep_range(hwResetDelay * 1000, (hwResetDelay + 1) * 1000);
		}
		/* Disable mt6336 controller when unuse mt6336 */
		mt6336_ctrl_disable(mt6336_ctrl);
	} else if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
					"mediatek/evb99v1_64_ufs_mhl", 27) == 0) {
		/* Enable mt6336 controller before use mt6336 */
		mt6336_ctrl_enable(mt6336_ctrl);
		/* Config MT6336 GPIO3(MT6336 HW GPIO2 == MT6336  SW GPIO6) */
		/* out low*/
		SLIMPORT_DBG("%s RESET_N LOW\n", __func__);
		mt6336_set_flag_register_value(MT6336_GPIO_DOUT0_CLR, 0x40);
		usleep_range(hwResetPeriod * 1000, (hwResetPeriod + 1) * 1000);

		if (is_power_on) {
			/* out high */
			SLIMPORT_DBG("%s RESET_N HIGH\n", __func__);
			mt6336_set_flag_register_value(MT6336_GPIO_DOUT0_SET, 0x40);
			usleep_range(hwResetDelay * 1000, (hwResetDelay + 1) * 1000);
		}
		/* Disable mt6336 controller when unuse mt6336 */
		mt6336_ctrl_disable(mt6336_ctrl);
	}
#endif
}

void cust_power_on_10v(int enable)
{
}

void cust_power_on_18v(int enable)
{
}

void cust_power_on_30v(int enable)
{
}

void cust_reset_init(void)
{
#ifdef CONFIG_MACH_MT6799
	/* RESET GPIO INIT */
	if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
						"mediatek/k99v2na_64", 19) == 0) {
		/* Get mt6336 controller when the first time of use mt6336 */
		mt6336_ctrl = mt6336_ctrl_get("mt6336_mhl");

		/* Enable mt6336 controller before use mt6336 */
		mt6336_ctrl_enable(mt6336_ctrl);

		/* Config MT6336 GPIO3(MT6336 HW GPIO2 == MT6336  SW GPIO6) */
		/* set to GPIO mode */
		mt6336_set_flag_register_value(MT6336_GPIO6_MODE, 0x0);
		/* set to OUTPUT mode */
		mt6336_set_flag_register_value(MT6336_GPIO_DIR0_SET, 0x40);
		/* output low */
		SLIMPORT_DBG("%s RESET_N LOW\n", __func__);
		mt6336_set_flag_register_value(MT6336_GPIO_DOUT0_CLR, 0x40);

		/* Disable mt6336 controller when unuse mt6336 */
		mt6336_ctrl_disable(mt6336_ctrl);
	} else if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
					"mediatek/evb99v1_64_ufs_mhl", 27) == 0) {
		/* Get mt6336 controller when the first time of use mt6336 */
		mt6336_ctrl = mt6336_ctrl_get("mt6336_mhl");

		/* Enable mt6336 controller before use mt6336 */
		mt6336_ctrl_enable(mt6336_ctrl);

		/* Config MT6336 GPIO3(MT6336 HW GPIO2 == MT6336  SW GPIO6) */
		/* set to GPIO mode */
		mt6336_set_flag_register_value(MT6336_GPIO6_MODE, 0x0);
		/* set to OUTPUT mode */
		mt6336_set_flag_register_value(MT6336_GPIO_DIR0_SET, 0x40);
		/* output low */
		SLIMPORT_DBG("%s RESET_N LOW\n", __func__);
		mt6336_set_flag_register_value(MT6336_GPIO_DOUT0_CLR, 0x40);

		/* Disable mt6336 controller when unuse mt6336 */
		mt6336_ctrl_disable(mt6336_ctrl);
	}
#endif
}

void cust_power_enable_init(void)
{
#ifdef CONFIG_MACH_MT6799
	/* POWER_EN GPIO INIT */
	if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
					"mediatek/evb99v1_64_ufs_mhl", 27) == 0) {
		/* Get mt6336 controller when the first time of use mt6336 */
		mt6336_ctrl = mt6336_ctrl_get("mt6336_mhl");

		/* Enable mt6336 controller before use mt6336 */
		mt6336_ctrl_enable(mt6336_ctrl);

		/* Config MT6336 GPIO3(MT6336 HW GPIO3 == MT6336  SW GPIO7) */
		/* set to GPIO mode */
		mt6336_set_flag_register_value(MT6336_GPIO7_MODE, 0x0);
		/* set to OUTPUT mode */
		mt6336_set_flag_register_value(MT6336_GPIO_DIR0_SET, 0x80);
		/* out low */
		SLIMPORT_DBG("%s PWR_EN LOW\n", __func__);
		mt6336_set_flag_register_value(MT6336_GPIO_DOUT0_CLR, 0x80);

		/* Disable mt6336 controller when unuse mt6336 */
		mt6336_ctrl_disable(mt6336_ctrl);
	} else if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
					"mediatek/k99v2na_64", 19) == 0) {
		/* Get mt6336 controller when the first time of use mt6336 */
		mt6336_ctrl = mt6336_ctrl_get("mt6336_mhl");

		/* Enable mt6336 controller before use mt6336 */
		mt6336_ctrl_enable(mt6336_ctrl);

		/* Config MT6336 GPIO1(MT6336 HW GPIO1 == MT6336  SW GPIO5) */
		/* set to GPIO mode */
		mt6336_set_flag_register_value(MT6336_GPIO5_MODE, 0x0);
		/* set to OUTPUT mode */
		mt6336_set_flag_register_value(MT6336_GPIO_DIR0_SET, 0x20);
		/* out low */
		SLIMPORT_DBG("%s PWR_EN LOW\n", __func__);
		mt6336_set_flag_register_value(MT6336_GPIO_DOUT0_CLR, 0x20);

		/* Enable LDO output 5V to ANX7625 VCONN_IN */
		/* Config MT6336 GPIO6(MT6336 HW GPIO6 == MT6336  SW GPIO10) */
		/* set to GPIO mode */
		mt6336_set_flag_register_value(MT6336_GPIO10_MODE, 0x0);
		/* set to OUTPUT mode */
		mt6336_set_flag_register_value(MT6336_GPIO_DIR1_SET, 0x4);
		/* out high */
		SLIMPORT_DBG("%s LDO output 5V to VCONN is Enable\n", __func__);
		mt6336_set_flag_register_value(MT6336_GPIO_DOUT1_SET, 0x4);

		/* Disable mt6336 controller when unuse mt6336 */
		mt6336_ctrl_disable(mt6336_ctrl);
	}
#endif
}

void cust_usb_misc_init(void)
{
#ifdef CONFIG_MACH_MT6799
	if (strncmp(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES,
					"mediatek/k99v2na_64", 19) == 0) {
		/* Get mt6336 controller when the first time of use mt6336 */
		mt6336_ctrl = mt6336_ctrl_get("mt6336_mhl");

		/* Enable mt6336 controller before use mt6336 */
		mt6336_ctrl_enable(mt6336_ctrl);

		SLIMPORT_DBG("%s disable CC on MT6336\n", __func__);

		mt6336_config_interface(MT6336_RG_A_ANABASE_RSV_ADDR, 0x1, 0x1, 0x3);
		SLIMPORT_DBG("CORE_ANA_CON109(0x566)=0x%x [0x8]\n",
			mt6336_get_register_value(MT6336_RG_A_ANABASE_RSV_ADDR));

		mt6336_set_flag_register_value(MT6336_CLK_TYPE_C_CSR_LOWQ_PDN_DIS, 0x1);
		SLIMPORT_DBG("CLK_TYPE_C_CSR_LOWQ_PDN_DIS=0x%x\n",
			mt6336_get_register_value(MT6336_CLK_TYPE_C_CSR_LOWQ_PDN_DIS_ADDR));

		mt6336_set_flag_register_value(MT6336_CLK_TYPE_C_CC_LOWQ_PDN_DIS, 0x1);
		SLIMPORT_DBG("CLK_TYPE_C_CC_LOWQ_PDN_DIS=0x%x\n",
			mt6336_get_register_value(MT6336_CLK_TYPE_C_CC_LOWQ_PDN_DIS_ADDR));

		/*0x164 = 0*/
		mt6336_set_register_value(MT6336_PMIC_TYPE_C_CC_SW_FORCE_MODE_VAL_0, 0x0);

		/*0x160[0] = 1*/
		mt6336_set_flag_register_value(MT6336_REG_TYPE_C_SW_FORCE_MODE_EN_DA_CC_RCC1, 0x1);
		/*0x160[1] = 1*/
		mt6336_set_flag_register_value(MT6336_REG_TYPE_C_SW_FORCE_MODE_EN_DA_CC_RCC2, 0x1);

		/* Disable mt6336 controller when unuse mt6336 */
		mt6336_ctrl_disable(mt6336_ctrl);
	}
#endif
}

static int anx7625_gpio_parse_dt(struct device *dev,
			    struct anx7625_platform_gpio_data *pdata)
{
	struct device_node *np = dev->of_node;
	int gpio;

	gpio = of_get_named_gpio(np, "analogix,anx7625_power-on-gpio", 0);
	if (gpio_is_valid(gpio))
		pdata->anx7625_gpio_p_on = gpio;
	else
		pr_err("can not get valid gpio analogix,anx7625_power-on-gpio\n");

	gpio = of_get_named_gpio(np, "analogix,anx7625_reset-gpio", 0);
	if (gpio_is_valid(gpio))
		pdata->anx7625_gpio_reset = gpio;
	else
		pr_err("can not get valid gpio analogix,anx7625_reset-gpio\n");

	gpio = of_get_named_gpio(np, "analogix,anx7625_cbl-det-gpio", 0);
	if (gpio_is_valid(gpio))
		pdata->anx7625_gpio_cbl_det = gpio;
	else
		pr_err("can not get valid gpio analogix,anx7625_cbl-det-gpio\n");

	gpio = of_get_named_gpio(np, "analogix,anx7625_intr-comm-gpio", 0);
	if (gpio_is_valid(gpio))
		pdata->anx7625_gpio_intr_comm = gpio;
	else
		pr_err("can not get valid gpio analogix,anx7625_intr-comm-gpio\n");

	SLIMPORT_DBG("anx7625 p_on=%d, reset=%d, cbl_det=%d, intp=%d\n",
	       pdata->anx7625_gpio_p_on, pdata->anx7625_gpio_reset,
	       pdata->anx7625_gpio_cbl_det, pdata->anx7625_gpio_intr_comm);

	return 0;
}

void slimport_platform_init(void)
{
	int ret = 0;
	int retval = 0;

	SLIMPORT_DBG("slimport_platform_init start !!\n");

	if (ext_dev_context == NULL) {
		SLIMPORT_DBG("Cannot find device in platform_init!\n");
		goto plat_init_exit;

	}

	reg_v30_power = devm_regulator_get(ext_dev_context, "mediatek,3_0v");
	if (IS_ERR(reg_v30_power))
		cust_power_on_30v(1);
	else {
		regulator_set_voltage(reg_v30_power, 3000000, 3000000);
		ret = regulator_enable(reg_v30_power);
		if (ret)
			SLIMPORT_DBG("regulator enable 3_0v fail!\n");
		else
			SLIMPORT_DBG("regulator enable 3_0v success!\n");
	}

	reg_v18_power = devm_regulator_get(ext_dev_context, "mediatek,1_8v");
	if (IS_ERR(reg_v18_power))
		cust_power_on_18v(1);
	else {
		regulator_set_voltage(reg_v18_power, 1800000, 1800000);
		ret = regulator_enable(reg_v18_power);
		if (ret)
			SLIMPORT_DBG("regulator enable 1_8v fail!\n");
		else
			SLIMPORT_DBG("regulator enable 1_8v success!\n");
	}

	reg_v10_power = devm_regulator_get(ext_dev_context, "mediatek,1_0v");
	if (IS_ERR(reg_v10_power))
		cust_power_on_10v(1);
	else {
		regulator_set_voltage(reg_v10_power, 1000000, 1000000);
		ret = regulator_enable(reg_v10_power);
		if (ret)
			SLIMPORT_DBG("regulator enable 1_0v fail!\n");
		else
			SLIMPORT_DBG("regulator enable 1_0v success!\n");
	}

	mhl_pinctrl = devm_pinctrl_get(ext_dev_context);
	if (IS_ERR(mhl_pinctrl)) {
		ret = PTR_ERR(mhl_pinctrl);
		SLIMPORT_DBG("Cannot find Slimport Pinctrl!!!!\n");
		goto plat_init_exit;
	}

	pin_state = pinctrl_lookup_state(mhl_pinctrl, power_ctl_gpio_name[1]);
	if (IS_ERR(pin_state)) {
		ret = PTR_ERR(pin_state);
		SLIMPORT_DBG("Cannot find Slimport power enable pinctrl high!!\n");
		cust_power_enable_init();
	} else
		pinctrl_select_state(mhl_pinctrl, pin_state);
	SLIMPORT_DBG("slimport_platform_init power enable gpio init done!!\n");

	pin_state = pinctrl_lookup_state(mhl_pinctrl, rst_gpio_name[1]);
	if (IS_ERR(pin_state)) {
		ret = PTR_ERR(pin_state);
		SLIMPORT_DBG("Cannot find Slimport RST pinctrl low!!\n");
		cust_reset_init();
	} else
		pinctrl_select_state(mhl_pinctrl, pin_state);
	SLIMPORT_DBG("slimport_platform_init reset gpio init done!!\n");

	cust_usb_misc_init();

	pin_state = pinctrl_lookup_state(mhl_pinctrl, eint_gpio_name[0]);
	if (IS_ERR(pin_state)) {
		ret = PTR_ERR(pin_state);
		SLIMPORT_DBG("Cannot find MHL eint pinctrl low!!\n");
	} else
		pinctrl_select_state(mhl_pinctrl, pin_state);
	SLIMPORT_DBG("slimport_platform_init eint gpio init done!!\n");

	pin_state = pinctrl_lookup_state(mhl_pinctrl, eint_comm_name[0]);
	if (IS_ERR(pin_state)) {
		ret = PTR_ERR(pin_state);
		SLIMPORT_DBG("Cannot find MHL eint_det pinctrl high!!\n");
	} else
		pinctrl_select_state(mhl_pinctrl, pin_state);
	SLIMPORT_DBG("slimport_platform_init eint_det gpio init done!!\n");

	i2s_gpio_ctrl(0);
	dpi_gpio_ctrl(0);

	anx7625_pdata = devm_kzalloc(ext_dev_context,
			     sizeof(struct anx7625_platform_gpio_data),
			     GFP_KERNEL);

	/* device tree parsing function call */
	retval = anx7625_gpio_parse_dt(ext_dev_context, anx7625_pdata);
	if (retval != 0)	/* if occurs error */
		retval = -1;

plat_init_exit:
	SLIMPORT_DBG("mhl_platform_init init done !!\n");
}

