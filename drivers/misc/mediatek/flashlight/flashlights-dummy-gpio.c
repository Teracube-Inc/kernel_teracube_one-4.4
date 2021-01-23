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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* define device tree */
/* TODO: modify temp device tree name */
#ifndef DUMMY_DTNAME
#define DUMMY_DTNAME "mediatek,flashlights_dummy_gpio"
#endif

/* TODO: define driver name */
#define DUMMY_NAME "flashlights-dummy-gpio"

/* define registers */
/* TODO: define register */
static int g_duty = 0;

/* define mutex and work queue */
static DEFINE_MUTEX(dummy_mutex);
static struct work_struct dummy_work;

/* flashlight pinctrl enum */
typedef enum {
    FLASHLIGHT_PIN_HWEN,    /* GPIO pin HWEN */
    FLASHLIGHT_PIN_TORCH,   /* GPIO pin TORCH */
    FLASHLIGHT_PIN_FLASH,   /* GPIO pin FLASH */
    SUB_FLASHLIGHT_PIN_FLASH    /* GPIO pin SUB_FLASH */
}FLASHLIGHT_GPIO_PIN_ENUM; 
typedef enum {
    STATE_LOW,
    STATE_HIGH
} FLASHLIGHT_GPIO_STATE_ENUM;

/* ============================== */
/* Pinctrl */
/* ============================== */
static struct pinctrl *flashlight_pinctrl;
static struct pinctrl_state *flashlight_hwen_high;
static struct pinctrl_state *flashlight_hwen_low;
static struct pinctrl_state *flashlight_torch_high;
static struct pinctrl_state *flashlight_torch_low;
static struct pinctrl_state *flashlight_flash_high;
static struct pinctrl_state *flashlight_flash_low;
static struct pinctrl_state *sub_flashlight_flash_high;
static struct pinctrl_state *sub_flashlight_flash_low;

/* define usage count */
static int use_count;

/* platform data */
struct dummy_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int dummy_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

    flashlight_pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(flashlight_pinctrl)) {
        pr_debug("Cannot find flashlight pinctrl!");
        ret = PTR_ERR(flashlight_pinctrl);
    }
    /* Flashlight HWEN pin initialization */
    flashlight_hwen_high = pinctrl_lookup_state(flashlight_pinctrl, "hwen_high");
    if (IS_ERR(flashlight_hwen_high)) {
        ret = PTR_ERR(flashlight_hwen_high);
        pr_debug("%s : init err, flashlight_hwen_high\n", __func__);
    }

    flashlight_hwen_low = pinctrl_lookup_state(flashlight_pinctrl, "hwen_low");
    if (IS_ERR(flashlight_hwen_low)) {
        ret = PTR_ERR(flashlight_hwen_low);
        pr_debug("%s : init err, flashlight_hwen_low\n", __func__);
    }

    /* Flashlight TORCH pin initialization */
    flashlight_torch_high = pinctrl_lookup_state(flashlight_pinctrl, "torch_high");
    if (IS_ERR(flashlight_torch_high)) {
        ret = PTR_ERR(flashlight_torch_high);
        pr_debug("%s : init err, flashlight_torch_high\n", __func__);
    }

    flashlight_torch_low = pinctrl_lookup_state(flashlight_pinctrl, "torch_low");
    if (IS_ERR(flashlight_torch_low)) {
        ret = PTR_ERR(flashlight_torch_low);
        pr_debug("%s : init err, flashlight_torch_low\n", __func__);
    }

    /* Flashlight FLASH pin initialization */
    flashlight_flash_high = pinctrl_lookup_state(flashlight_pinctrl, "flash_high");
    if (IS_ERR(flashlight_flash_high)) {
        ret = PTR_ERR(flashlight_flash_high);
        pr_debug("%s : init err, flashlight_flash_high\n", __func__);
    }
    flashlight_flash_low = pinctrl_lookup_state(flashlight_pinctrl, "flash_low");
    if (IS_ERR(flashlight_flash_low)) {
        ret = PTR_ERR(flashlight_flash_low);
        pr_debug("%s : init err, flashlight_flash_low\n", __func__);
    }

    /*Sub Flashlight FLASH pin initialization */
    sub_flashlight_flash_high = pinctrl_lookup_state(flashlight_pinctrl, "sub_flash_high");
    if (IS_ERR(sub_flashlight_flash_high)) {
        ret = PTR_ERR(sub_flashlight_flash_high);
        pr_debug("%s : init err, sub_flashlight_flash_high\n", __func__);
    }

    sub_flashlight_flash_low = pinctrl_lookup_state(flashlight_pinctrl, "sub_flash_low");
    if (IS_ERR(sub_flashlight_flash_low)) {
        ret = PTR_ERR(sub_flashlight_flash_low);
        pr_debug("%s : init err, sub_flashlight_flash_low\n", __func__);
    }

    return 0;

}

int flashlight_gpio_set(int pin , int state)
{
    int ret = 0;

    if (IS_ERR(flashlight_pinctrl)) {
        pr_debug("%s : set err, flashlight_pinctrl not available\n", __func__);
        return -1;
    }

    switch (pin) {
    case FLASHLIGHT_PIN_HWEN:
        if (state == STATE_LOW && !IS_ERR(flashlight_hwen_low))
            pinctrl_select_state(flashlight_pinctrl, flashlight_hwen_low);
        else if (state == STATE_HIGH && !IS_ERR(flashlight_hwen_high))
            pinctrl_select_state(flashlight_pinctrl, flashlight_hwen_high);
        else
            pr_debug("%s : set err, pin(%d) state(%d)\n", __func__, pin, state);
        break;
    case FLASHLIGHT_PIN_TORCH:
        if (state == STATE_LOW && !IS_ERR(flashlight_torch_low))
            pinctrl_select_state(flashlight_pinctrl, flashlight_torch_low);
        else if (state == STATE_HIGH && !IS_ERR(flashlight_torch_high))
            pinctrl_select_state(flashlight_pinctrl, flashlight_torch_high);
        else
            pr_debug("%s : set err, pin(%d) state(%d)\n", __func__, pin, state);
        break;
    case FLASHLIGHT_PIN_FLASH:
        if (state == STATE_LOW && !IS_ERR(flashlight_flash_low))
            pinctrl_select_state(flashlight_pinctrl, flashlight_flash_low);
        else if (state == STATE_HIGH && !IS_ERR(flashlight_flash_high))
            pinctrl_select_state(flashlight_pinctrl, flashlight_flash_high);
        else
            pr_debug("%s : set err, pin(%d) state(%d)\n", __func__, pin, state);
        break;
    case SUB_FLASHLIGHT_PIN_FLASH:
        if (state == STATE_LOW && !IS_ERR(sub_flashlight_flash_low))
            pinctrl_select_state(flashlight_pinctrl, sub_flashlight_flash_low);
        else if (state == STATE_HIGH && !IS_ERR(sub_flashlight_flash_high))
            pinctrl_select_state(flashlight_pinctrl, sub_flashlight_flash_high);
        else
            pr_debug("%s : set err, pin(%d) state(%d)\n", __func__, pin, state);
        break;
    default:
            pr_debug("%s : set err, pin(%d) state(%d)\n", __func__, pin, state);
        break;
    }
    pr_debug("%s : pin(%d) state(%d)\n", __func__, pin, state);
    return ret;
}


/******************************************************************************
 * dummy operations
 *****************************************************************************/
/* flashlight enable function */
static int dummy_enable(void)
{
  printk("%s line %d \n",__func__,__LINE__);
  printk("g_duty %d \n",g_duty);
  if(g_duty < 6)
  {
    flashlight_gpio_set(FLASHLIGHT_PIN_TORCH,STATE_HIGH);
    flashlight_gpio_set(FLASHLIGHT_PIN_FLASH,STATE_LOW);
  }
  else
  {
    flashlight_gpio_set(FLASHLIGHT_PIN_TORCH,STATE_HIGH);
    flashlight_gpio_set(FLASHLIGHT_PIN_FLASH,STATE_HIGH);
  }
  return 0;

}

/* flashlight disable function */
static int dummy_disable(void)
{
  printk("%s line %d \n",__func__,__LINE__);
  flashlight_gpio_set(FLASHLIGHT_PIN_TORCH,STATE_LOW);
  flashlight_gpio_set(FLASHLIGHT_PIN_FLASH,STATE_LOW);
  return 0;
}
int sub_flashlight_gpio_enable(bool en)
{
    printk("%s en = %d\n",__func__,en);
    if(en)
    {
        flashlight_gpio_set(SUB_FLASHLIGHT_PIN_FLASH,STATE_HIGH);
    }
    else
    {
        flashlight_gpio_set(SUB_FLASHLIGHT_PIN_FLASH,STATE_LOW);
    }
  return 0;
}
/* set flashlight level */
static int dummy_set_level(int level)
{
    printk(" FL_dim_duty line=%d\n", __LINE__);
    g_duty = level;
    return 0;
}

/* flashlight init */
static int dummy_init(void)
{
  printk(KERN_ERR "%s line %d \n",__func__,__LINE__);
  flashlight_gpio_set(FLASHLIGHT_PIN_TORCH,STATE_LOW);
  flashlight_gpio_set(FLASHLIGHT_PIN_FLASH,STATE_LOW);
  return 0;
}

/* flashlight uninit */
static int dummy_uninit(void)
{
    dummy_disable();
    return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer dummy_timer;
static unsigned int dummy_timeout_ms;

static void dummy_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	dummy_disable();
}

static enum hrtimer_restart dummy_timer_func(struct hrtimer *timer)
{
	schedule_work(&dummy_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int dummy_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		dummy_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		dummy_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (dummy_timeout_ms) {
				s = dummy_timeout_ms / 1000;
				ns = dummy_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&dummy_timer, ktime,
						HRTIMER_MODE_REL);
			}
			dummy_enable();
		} else {
			dummy_disable();
			hrtimer_cancel(&dummy_timer);
		}
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int dummy_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int dummy_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int dummy_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&dummy_mutex);
	if (set) {
		if (!use_count)
			ret = dummy_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = dummy_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&dummy_mutex);

	return ret;
}

static ssize_t dummy_strobe_store(struct flashlight_arg arg)
{
	dummy_set_driver(1);
	dummy_set_level(arg.level);
	dummy_timeout_ms = 0;
	dummy_enable();
	msleep(arg.dur);
	dummy_disable();
	dummy_set_driver(0);

	return 0;
}

static struct flashlight_operations dummy_ops = {
	dummy_open,
	dummy_release,
	dummy_ioctl,
	dummy_strobe_store,
	dummy_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int dummy_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * dummy_init();
	 */

	return 0;
}

static int dummy_parse_dt(struct device *dev,
		struct dummy_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				DUMMY_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int dummy_probe(struct platform_device *pdev)
{
	struct dummy_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	pr_debug("Probe start.\n");

	/* init pinctrl */
	if (dummy_pinctrl_init(pdev)) {
		pr_debug("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = dummy_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&dummy_work, dummy_work_disable);

	/* init timer */
	hrtimer_init(&dummy_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dummy_timer.function = dummy_timer_func;
	dummy_timeout_ms = 100;

	/* init chip hw */
	dummy_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&dummy_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(DUMMY_NAME, &dummy_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	pr_debug("Probe done.\n");

	return 0;
err:
	return err;
}

static int dummy_remove(struct platform_device *pdev)
{
	struct dummy_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(DUMMY_NAME);

	/* flush work queue */
	flush_work(&dummy_work);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dummy_gpio_of_match[] = {
	{.compatible = DUMMY_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, dummy_gpio_of_match);
#else
static struct platform_device dummy_gpio_platform_device[] = {
	{
		.name = DUMMY_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, dummy_gpio_platform_device);
#endif

static struct platform_driver dummy_platform_driver = {
	.probe = dummy_probe,
	.remove = dummy_remove,
	.driver = {
		.name = DUMMY_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = dummy_gpio_of_match,
#endif
	},
};

static int __init flashlight_dummy_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&dummy_gpio_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&dummy_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_dummy_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&dummy_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_dummy_init);
module_exit(flashlight_dummy_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight DUMMY GPIO Driver");

