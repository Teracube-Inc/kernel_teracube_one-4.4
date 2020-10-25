#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/delay.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/time.h>

#include <linux/string.h>

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
struct scanner_data_t{
    int power_state;
    int wakeup_state;
    int trig_state;
    int rfid_state;
    struct platform_device *platform_device_addr;
};

static struct scanner_data_t *n4313_data_t = NULL;
/* ============================== */
/* Pinctrl */
/* ============================== */
static struct pinctrl *scanner_pinctrl;
static struct pinctrl_state *scanner_laser_power_high;
static struct pinctrl_state *scanner_laser_power_low;
static struct pinctrl_state *scanner_wakeup_high;
static struct pinctrl_state *scanner_wakeup_low;
static struct pinctrl_state *scanner_trig_high;
static struct pinctrl_state *scanner_trig_low;
static struct pinctrl_state *rfid_power_high;
static struct pinctrl_state *rfid_power_low;
static struct pinctrl_state *rfid_rf_high;
static struct pinctrl_state *rfid_rf_low;
static ssize_t scanner_show_powersupply(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, 20, "%d\n",n4313_data_t->power_state);
}
static ssize_t scanner_store_powersupply(struct device *dev, struct device_attribute *attr,
    const char *buf, size_t count)
{
  if (!strncmp(buf, "1", 1)){
    pinctrl_select_state(scanner_pinctrl,scanner_laser_power_high);
    n4313_data_t->power_state=1;
  } else if (!strncmp(buf, "0", 1)) {
    pinctrl_select_state(scanner_pinctrl,scanner_laser_power_low);
    n4313_data_t->power_state=0;
  }else{
    pr_err(" not support now\n");
  }
  return count;
}
static ssize_t scanner_show_wakeup(struct device *dev, struct device_attribute *attr, char *buf)
{

  return snprintf(buf, 20, "%d\n",n4313_data_t->wakeup_state);
}
static ssize_t scanner_store_wakeup(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{

  if (!strncmp(buf, "1", 1)){
    pinctrl_select_state(scanner_pinctrl,scanner_wakeup_high);
    n4313_data_t->wakeup_state=1;
  } else if (!strncmp(buf, "0", 1)) {
    pinctrl_select_state(scanner_pinctrl,scanner_wakeup_low);
    n4313_data_t->wakeup_state=0;
  }else{
    pr_err(" not support now\n");
  }
  return count;
}
static ssize_t scanner_show_trig(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, 20, "%d\n",n4313_data_t->trig_state);
}
static ssize_t scanner_store_trig(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
  if (!strncmp(buf, "1", 1)){
    pinctrl_select_state(scanner_pinctrl,scanner_trig_high);
    n4313_data_t->trig_state=1;
  } else if (!strncmp(buf, "0", 1)) {
    pinctrl_select_state(scanner_pinctrl,scanner_trig_low);
    n4313_data_t->trig_state=0;
  }else{
    pr_err(" not support now\n");
  }
  return count;
}
static ssize_t scanner_show_rfidpower(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, 20, "%d\n",n4313_data_t->rfid_state);
}
static ssize_t scanner_store_rfidpower(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
  if (!strncmp(buf, "1", 1)){
    pinctrl_select_state(scanner_pinctrl,rfid_power_high);
    mdelay(100);
    pinctrl_select_state(scanner_pinctrl,rfid_rf_high);
    n4313_data_t->rfid_state=1;
  } else if (!strncmp(buf, "0", 1)) {
    pinctrl_select_state(scanner_pinctrl,rfid_power_low);
    mdelay(100);
    pinctrl_select_state(scanner_pinctrl,rfid_rf_low);
    n4313_data_t->rfid_state=0;
  }else{
    pr_err(" not support now\n");
  }
  return count;
}
DEVICE_ATTR(powersupply, S_IWUSR | S_IRUGO, scanner_show_powersupply, scanner_store_powersupply);
DEVICE_ATTR(wakeup,S_IWUSR | S_IRUGO, scanner_show_wakeup, scanner_store_wakeup);
DEVICE_ATTR(trig,  S_IWUSR | S_IRUGO, scanner_show_trig,  scanner_store_trig);
DEVICE_ATTR(rfidpower,  S_IWUSR | S_IRUGO, scanner_show_rfidpower,  scanner_store_rfidpower);

static struct attribute *scanner_attributes[] = {
    &dev_attr_powersupply.attr,
    &dev_attr_wakeup.attr,
    &dev_attr_trig.attr,
    &dev_attr_rfidpower.attr,
    NULL
};

static struct attribute_group scanner_attribute_group = {
    .attrs = scanner_attributes
};

static int scanner_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct scanner_data_t  *s_data = NULL;
    printk("----qingzhan enter scanner driver begin----\n");
    s_data = kzalloc(sizeof(*s_data), GFP_KERNEL);
    if (!s_data) {
        pr_err("kernel memory alocation was failed\n");
    }
    s_data->power_state=0;
    s_data->wakeup_state=0;
    s_data->trig_state=0;
    s_data->platform_device_addr=pdev;
    n4313_data_t = s_data;
    scanner_pinctrl = devm_pinctrl_get(&pdev->dev);
    if (scanner_pinctrl == NULL || IS_ERR(scanner_pinctrl)) {
        ret = PTR_ERR(scanner_pinctrl);
        pr_err("Cannot find scanner pinctrl!\n");
    }
    /* scanner control pin initialization */
    scanner_laser_power_high = pinctrl_lookup_state(scanner_pinctrl,"laser_gpio_high");
    if (scanner_laser_power_high == NULL || IS_ERR(scanner_laser_power_high)) {
        pr_err("%s : init err,scanner_laser_power_high\n", __func__);
    }

    scanner_laser_power_low = pinctrl_lookup_state(scanner_pinctrl,"laser_gpio_low");
    if (scanner_laser_power_low == NULL || IS_ERR(scanner_laser_power_low)) {
        pr_err("%s : init err, scanner_laser_power_low\n", __func__);
    }

    scanner_wakeup_high = pinctrl_lookup_state(scanner_pinctrl,"wakeup_high");
    if (scanner_wakeup_high == NULL || IS_ERR(scanner_wakeup_high)) {
        pr_err("%s : init err, scanner_wakeup_high\n", __func__);
    }

    scanner_wakeup_low = pinctrl_lookup_state(scanner_pinctrl,"wakeup_low");
    if (scanner_wakeup_low == NULL || IS_ERR(scanner_wakeup_low)) {
        pr_err("%s : init err, scanner_wakeup_low\n", __func__);
    }
    scanner_trig_high = pinctrl_lookup_state(scanner_pinctrl,"trig_high");
    if (scanner_trig_high == NULL || IS_ERR(scanner_trig_high)) {
        pr_err("%s : init err, scanner_trig_high\n", __func__);
    }

    scanner_trig_low = pinctrl_lookup_state(scanner_pinctrl,"trig_low");
    if (scanner_trig_low == NULL || IS_ERR(scanner_trig_low)) {
        pr_err("%s : init err, scanner_trig_low\n", __func__);
    }
    rfid_power_high = pinctrl_lookup_state(scanner_pinctrl,"rfid_power_high");
    if (rfid_power_high == NULL || IS_ERR(rfid_power_high)) {
        pr_err("%s : init err, rfid_power_high\n", __func__);
    }

    rfid_power_low = pinctrl_lookup_state(scanner_pinctrl,"rfid_power_low");
    if (rfid_power_low == NULL || IS_ERR(rfid_power_low)) {
        pr_err("%s : init err, rfid_power_low\n", __func__);
    }
    rfid_rf_high = pinctrl_lookup_state(scanner_pinctrl,"rfid_rf_high");
    if (rfid_rf_high == NULL || IS_ERR(rfid_rf_high)) {
        pr_err("%s : init err, rfid_rf_high\n", __func__);
    }

    rfid_rf_low = pinctrl_lookup_state(scanner_pinctrl,"rfid_rf_low");
    if (rfid_rf_low == NULL || IS_ERR(rfid_rf_low)) {
        pr_err("%s : init err, rfid_rf_low\n", __func__);
    }
    ret = sysfs_create_group(&s_data->platform_device_addr->dev.kobj, &scanner_attribute_group);
    if (ret) {
        pr_err("sysfs_create_group was failed(%d)\n", ret);
    }

    printk("----qingzhan enter scanner driver end----\n");
    return 0;
}

static int scanner_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int scanner_resume(struct platform_device *pdev)
{
    return 0;
}
static int scanner_remove(struct platform_device *pdev)
{
    sysfs_remove_group(&pdev->dev.kobj, &scanner_attribute_group);
    return 0;
}
static const struct of_device_id scanner_of_match[] = {
    { .compatible = "mediatek,scanner_driver", },
    {},
};

static struct platform_driver scanner_driver = {
    .probe = scanner_probe,
    .suspend = scanner_suspend,
    .resume = scanner_resume,
    .remove = scanner_remove,
    .driver = {
        .owner  = THIS_MODULE,
        .name = "scanner_driver",
        .of_match_table = scanner_of_match,
    },
};

static int scanner_mod_init(void)
{

    if(platform_driver_register(&scanner_driver) != 0)
    {
        pr_debug("unable to register scanner driver\n");
        return -1;
    }

    return 0;
}

static void scanner_mod_exit(void)
{
    platform_driver_unregister(&scanner_driver);
}
module_init(scanner_mod_init);
module_exit(scanner_mod_exit);

MODULE_DESCRIPTION("FE scanner driver");
MODULE_AUTHOR("zhangqingzhan <zhangqingzhan@foeec.com>");
MODULE_LICENSE("GPL");
