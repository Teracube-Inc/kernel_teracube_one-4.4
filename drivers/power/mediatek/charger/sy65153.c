/*
 * Copyright (C) 2016 MediaTek Inc.
 * ShuFanLee <shufan_lee@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>




#include "mtk_charger_intf.h"
#include "sy65153.h"

static struct i2c_client *sy65153_client;

struct sy65153_info {
	struct i2c_client *i2c;
	struct device *dev;
	u32 intr_gpio;
	u32 en_gpio;
};
static DEFINE_MUTEX(sy65153_i2c_access);
extern void charger_ignore_usb(bool ignore);
#if 1
/* ========================= */
/* I2C operations            */
/* ========================= */
unsigned int sy65153_read_byte(unsigned char cmd, unsigned char *returnData)
{
	unsigned char xfers = 2;
	int ret, retries = 1;
    u8 reg_buf[2];
    reg_buf[0]=0;
    reg_buf[1]=cmd;
	mutex_lock(&sy65153_i2c_access);

	do {
		struct i2c_msg msgs[2] = {
			{
				.addr = sy65153_client->addr,
				.flags = 0,
				.len = 2,
				.buf = reg_buf,
			},
			{

				.addr = sy65153_client->addr,
				.flags = I2C_M_RD,
				.len = 2,
				.buf = returnData,
			}
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(sy65153_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			pr_info("skipping non-existent adapter %s\n", sy65153_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&sy65153_i2c_access);

	return ret == xfers ? 1 : -1;
}
//static int sy65153_device_write(void *client, u8 addr, u8 leng,
unsigned int sy65153_write_byte(unsigned char cmd, unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&sy65153_i2c_access);

	buf[0] = 0;
	buf[1] = cmd;
	buf[2] = writeData;

	do {
		struct i2c_msg msgs[1] = {
			{
				.addr = sy65153_client->addr,
				.flags = 0,
				.len = 2 + 1,
				.buf = buf,
			},
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(sy65153_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			pr_info("skipping non-existent adapter %s\n", sy65153_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&sy65153_i2c_access);

	return ret == xfers ? 1 : -1;
}
#endif
/* ================== */
/* Internal Functions */
/* ================== */

/* The following APIs will be reference in internal functions */
#if 1
bool sy65153_is_charging_en(void)
{
  int val,ret;
  u8 reg[2] = {0}; 
  struct sy65153_info *info = i2c_get_clientdata(sy65153_client);
  val = gpio_get_value(info->en_gpio);
  if(val == 0)
  {
    return false;
  }
  else
  {
    charger_ignore_usb(false);
    ret = sy65153_read_byte(0x0,reg);
    pr_info("sy65153 test 0x0=%x,0x1=%x\n",reg[0],reg[1]);
    ret = sy65153_read_byte(0x2,reg);
    pr_info("sy65153 test 0x2=%x,0x3=%x\n",reg[0],reg[1]);
    ret = sy65153_read_byte(0x4,reg);
    pr_info("sy65153 test 0x4=%x,0x5=%x\n",reg[0],reg[1]);
    ret = sy65153_read_byte(0x6,reg);
    pr_info("sy65153 test 0x6=%x,0x7=%x\n",reg[0],reg[1]);
    ret = sy65153_read_byte(0x34,reg);
    pr_info("sy65153 read charger type is %x %x\n",reg[0],reg[1]);
    if(ret < 0)
      pr_info("sy65153 read erro \n");

    if((reg[0]&0x30) == 0x10)
      reg[0] = 0x0f;    
    else
      reg[0] = 0x37;    
    sy65153_write_byte(0x3e,reg[0]);
  }
  return true;
}
#endif
static int sy65153_parse_dt(struct sy65153_info *info, struct device *dev)
{
	int ret = 0;
	struct device_node *np = dev->of_node;

	dev_info(info->dev, "%s\n", __func__);

	if (!np) {
		dev_err(info->dev, "%s: no device node\n", __func__);
		return -EINVAL;
	}

	/*
	 * For dual charger, one is primary_chg;
	 * another one will be secondary_chg
      */
#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
	ret = of_get_named_gpio(np, "sy,en_gpio", 0);
	if (ret < 0)
		return ret;
	info->en_gpio = ret;
#else
	ret = of_property_read_u32(np, "sy,en_gpio_num", &info->en_gpio);
	if (ret < 0)
		return ret;
#endif /* !CONFIG_MTK_GPIO || CONFIG_MTK_GPIOLIB_STAND */

	dev_info(info->dev, "%s: intr/en gpio = %d, %d\n", __func__,
		info->intr_gpio, info->en_gpio);

#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
	ret = of_get_named_gpio(np, "sy,intr_gpio", 0);
	if (ret < 0)
		return ret;
	info->intr_gpio = ret;
#else
	ret = of_property_read_u32(np, "sy,intr_gpio_num", &info->intr_gpio);
	if (ret < 0)
		return ret;
#endif
	dev_info(info->dev, "%s: intr/en gpio = %d, %d\n", __func__,
		info->intr_gpio, info->en_gpio);
	return 0;
}
#if 0
static int sy65153_init_setting(void)
{
	int ret = 0;
	return ret;
}
#endif
/* I2C driver function       */
/* ========================= */

static int sy65153_probe(struct i2c_client *i2c,
	const struct i2c_device_id *dev_id)
{
	int ret = 0;
	struct sy65153_info *info = NULL;

	pr_info("%s\n", __func__);
    sy65153_client = i2c;
	info = devm_kzalloc(&i2c->dev, sizeof(struct sy65153_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->i2c = i2c;
	info->dev = &i2c->dev;

	/* Must parse en gpio */
	ret = sy65153_parse_dt(info, &i2c->dev);
	if (ret < 0) {
		dev_err(info->dev, "%s: parse dt failed\n", __func__);
		goto err_parse_dt;
	}
	i2c_set_clientdata(i2c, info);

	dev_info(info->dev, "%s: successfully\n", __func__);

	return ret;

err_parse_dt:
	return ret;
}

static int sy65153_remove(struct i2c_client *i2c)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	return ret;
}

static void sy65153_shutdown(struct i2c_client *i2c)
{
	pr_info("%s\n", __func__);
}

static int sy65153_suspend(struct device *dev)
{

	dev_info(dev, "%s\n", __func__);

	return 0;
}

static int sy65153_resume(struct device *dev)
{

	dev_info(dev, "%s\n", __func__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(sy65153_pm_ops, sy65153_suspend, sy65153_resume);

static const struct i2c_device_id sy65153_i2c_id[] = {
	{"sy65153", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sy65153_i2c_id);

static const struct of_device_id sy65153_of_match[] = {
	{ .compatible = "Silergy,sy65153", },
	{},
};
MODULE_DEVICE_TABLE(of, sy65153_of_match);

static struct i2c_driver sy65153_i2c_driver = {
	.driver = {
		.name = "sy65153",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sy65153_of_match),
		.pm = &sy65153_pm_ops,
	},
	.probe = sy65153_probe,
	.remove = sy65153_remove,
	.shutdown = sy65153_shutdown,
	.id_table = sy65153_i2c_id,
};

int sy65153_config_interface (unsigned char RegNum, unsigned char val, unsigned char MASK, unsigned char SHIFT)
{
    unsigned char sy65153_reg = 0;
    int ret = 0;

    //pr_debug("--------------------------------------------------\n");

    ret = sy65153_read_byte(RegNum, &sy65153_reg);
    //pr_debug("[sy65153_config_interface] Reg[%x]=0x%x\n", RegNum, sy65153_reg);

    sy65153_reg &= ~(MASK << SHIFT);
    sy65153_reg |= (val << SHIFT);

    ret = sy65153_write_byte(RegNum, sy65153_reg);
    //pr_debug("[sy65153_config_interface] Write Reg[%x]=0x%x\n", RegNum, sy65153_reg);

    /* Check */
    sy65153_read_byte(RegNum, &sy65153_reg); 
    //pr_debug("[sy65153_config_interface] Check Reg[%x]=0x%x\n", RegNum, sy65153_reg); 

    return ret;
}
/**********************************************************
 *
 *   [Read / Write Function]
 *
 *********************************************************/
int sy65153_read_interface (unsigned char RegNum, unsigned char *val, unsigned char MASK, unsigned char SHIFT)
{
    unsigned char sy65153_reg = 0;
    int ret = 0;


    ret = sy65153_read_byte(RegNum, &sy65153_reg);
    //pr_debug("[sy65153_read_interface] Reg[%x]=0x%x\n", RegNum, sy65153_reg);

    sy65153_reg &= (MASK << SHIFT);
    *val = (sy65153_reg >> SHIFT);
    //pr_debug("[sy65153_read_interface] Val=0x%x\n", *val);

    return ret;
}
/**********************************************************
 *
 *   [platform_driver API]
 *
 *********************************************************/
unsigned char g_reg_value_sy65153 = 0;
static ssize_t show_sy65153_access(struct device *dev, struct device_attribute *attr, char *buf)
{
    pr_debug("[show_sy65153_access] 0x%x\n", g_reg_value_sy65153);
    return sprintf(buf, "%u\n", g_reg_value_sy65153);
}

static ssize_t store_sy65153_access(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t size)
{
    int ret = 0;
    char *pvalue = NULL;
    unsigned int reg_value = 0;
    unsigned int reg_address = 0;

    pr_debug("[store_sy65153_access]\n");

    if (buf != NULL && size != 0) {
        pr_debug("[store_sy65153_access] buf is %s and size is %zu\n", buf, size);
        reg_address = simple_strtoul(buf, &pvalue, 16);

        if (size > 3) {
            reg_value = simple_strtoul((pvalue + 1), NULL, 16);
            pr_debug
                ("[store_sy65153_access] write sy65153 reg 0x%x with value 0x%x !\n",
                 reg_address, reg_value);
            ret = sy65153_config_interface(reg_address, reg_value, 0xFF, 0x0);
        } else {
            ret = sy65153_read_interface(reg_address, &g_reg_value_sy65153, 0xFF, 0x0);
            pr_debug("[store_sy65153_access] read sy65153 reg 0x%x with value 0x%x !\n",
                    reg_address, g_reg_value_sy65153);
            pr_debug
                ("[store_sy65153_access] Please use \"cat sy65153_access\" to get value\r\n");
        }
    }
    return size;
}

static DEVICE_ATTR(sy65153_access, 0664, show_sy65153_access, store_sy65153_access);	/* 664 */

static int sy65153_user_space_probe(struct platform_device *dev)
{
    int ret_device_file = 0;

    pr_debug("******** sy65153_user_space_probe!! ********\n");

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_sy65153_access);

    return 0;
}

struct platform_device sy65153_user_space_device = {
    .name = "sy65153-user",
    .id = -1,
};

static struct platform_driver sy65153_user_space_driver = {
    .probe = sy65153_user_space_probe,
    .driver = {
        .name = "sy65153-user",
    },
};
static int __init sy65153_init(void)
{
	int ret = 0;
	pr_info("%s: with dts\n", __func__);

	ret = i2c_add_driver(&sy65153_i2c_driver);
	if (ret < 0)
		pr_err("%s: register i2c driver fail\n", __func__);

    /* sy65153 user space access interface */
    ret = platform_device_register(&sy65153_user_space_device);
    if (ret) {
        pr_debug("****[sy65153_init] Unable to device register(%d)\n", ret);
        return ret;
    }
    ret = platform_driver_register(&sy65153_user_space_driver);
    if (ret) {
        pr_debug("****[sy65153_init] Unable to register driver (%d)\n", ret);
        return ret;
    }

	return ret;
}
module_init(sy65153_init);


static void __exit sy65153_exit(void)
{
	i2c_del_driver(&sy65153_i2c_driver);
}
module_exit(sy65153_exit);

MODULE_LICENSE("FOEEC");
MODULE_AUTHOR("zhangqingzhan <zhangqingzhan@foeec.com>");
MODULE_DESCRIPTION("SY65153 Charger Driver");
