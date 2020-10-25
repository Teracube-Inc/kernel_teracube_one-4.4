/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

//andy_kernel_compiling_test

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/workqueue.h>
#define DEBUG
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/spi/spi.h>
//fpc, andy hong, for ree porting, crash at enroll, 20161122
#if 1
#include <linux/wait.h>
#include <linux/kthread.h>
#endif
#include "mt-plat/mtkgpio.h"
#include <mt-plat/mtk_gpio.h>
#include <mt-plat/mtk_gpio_core.h>
//end, fpc, andy hong, for ree porting, crash at enroll, 20161122

#define FPC1020_RESET_LOW_US 1000
#define FPC1020_RESET_HIGH1_US 100
#define FPC1020_RESET_HIGH2_US 1250

#define FPC_TTW_HOLD_TIME 1000

/* Uncomment if DeviceTree should be used */
//#define USE_DT


static struct device_node *irq_node;
extern int fpc_finger_get_gpio_info(struct platform_device *pdev);
extern int fpc_finger_set_power(int cmd);
extern int fpc_finger_set_reset(int cmd);
extern int fpc_finger_set_eint(int cmd);
extern int mt_get_gpio_in(unsigned long pin);

static int irq_gpio = 0;
struct mt_spi_t *fpc_ms;

typedef struct {
    struct spi_device      *spi;
    struct class           *class;
    struct device          *device;
    //	struct cdev            cdev;
    dev_t                  devno;
    u8                     *huge_buffer;
    size_t                 huge_buffer_size;
    struct input_dev       *input_dev;
} fpc1020_data_t;

//static struct platform_device *fpc_irq_platform_device;

struct vreg_config {
    char *name;
    unsigned long vmin;
    unsigned long vmax;
    int ua_load;
};
#if 0
static const struct vreg_config const vreg_conf[] = {
    { "vdd_ana", 1800000UL, 1800000UL, 6000, },
    { "vcc_spi", 1800000UL, 1800000UL, 10, },
    { "vdd_io", 1800000UL, 1800000UL, 6000, },
};
#endif
struct fpc1020_data {
    struct device *dev;
    struct platform_device *pldev;
    //struct regulator *vreg[ARRAY_SIZE(vreg_conf)];
    struct input_dev *idev;
    int irq_num;
    char idev_name[32];
    int event_type;
    int event_code;
    struct mutex lock;
    bool prepared;
    bool wakeup_enabled;
    struct wake_lock ttw_wl;
};

static struct work_struct work;
static struct workqueue_struct *workq = NULL;

//fpc, andy hong, for ree porting, crash at enroll, 20161122
#if 1
static wait_queue_head_t fpc1020_irq_return;

volatile int fpc1020_irq_occur = 0;

static struct task_struct *fpc1020_irq_thread = NULL;

static int fpc1020_irq_function(void *pData);

static int fpc1020_irq_init(struct fpc1020_data *fpc1020);

static int fpc1020_irq_destroy(struct fpc1020_data *fpc1020);
#endif
//end, fpc, andy hong, for ree porting, crash at enroll, 20161122

static int hw_reset(struct  fpc1020_data *fpc1020)
{
    //struct device *dev = fpc1020->dev;
    fpc_finger_set_reset(0);
    usleep_range(FPC1020_RESET_LOW_US, FPC1020_RESET_LOW_US + 100);

    fpc_finger_set_reset(1);
    usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

    return 0;
}

static ssize_t hw_reset_set(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int rc;
    struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

    if (!strncmp(buf, "reset", strlen("reset")))
        rc = hw_reset(fpc1020);
    else
        return -EINVAL;
    return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

    if (!strncmp(buf, "enable", strlen("enable")))
    {
        fpc1020->wakeup_enabled = true;
        smp_wmb();
    }
    else if (!strncmp(buf, "disable", strlen("disable")))
    {
        fpc1020->wakeup_enabled = false;
        smp_wmb();
    }
    else
        return -EINVAL;

    return count;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

int fpc1020_get_irqNum(void);

/**
 * sysfs node for sending event to make the system interactive,
 * i.e. waking up
 */
static ssize_t do_wakeup_set(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

    if( count > 0 )
    {
        /* Sending power key event creates a toggling 
           effect that may be desired. It could be 
           replaced by another event such as KEY_WAKEUP. */
        input_report_key(fpc1020->idev, KEY_POWER, 1);
        input_report_key(fpc1020->idev, KEY_POWER, 0);
        input_sync(fpc1020->idev);
    }
    else
    {
        return -EINVAL;
    }

    return count;
}

static DEVICE_ATTR(do_wakeup, S_IWUSR, NULL, do_wakeup_set);
/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *device,
        struct device_attribute *attribute,
        char* buffer)
{
    int irq_state = 0;
    u32 ints[4] = {0, 0,0,0};
    if (irq_node) {
        of_property_read_u32_array(irq_node, "interrupts", ints, ARRAY_SIZE(ints));
        gpio_request(irq_gpio, "fpc");
        irq_state = gpio_get_value(irq_gpio);
        printk("######### %s line %d gpio %d irq_state %d\n",__func__,__LINE__, ints[0], irq_state);
    }
    return scnprintf(buffer, PAGE_SIZE, "%i\n", irq_state);
}


/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *device,
        struct device_attribute *attribute,
        const char *buffer, size_t count)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
    dev_dbg(fpc1020->dev, "%s\n", __func__);
    return count;
}

static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);


static struct attribute *attributes[] = {
    &dev_attr_hw_reset.attr,
    &dev_attr_wakeup_enable.attr,
    &dev_attr_do_wakeup.attr,
    //fpc, andy hong, ree porting, remove, 20160719
#if 0	
    &dev_attr_clk_enable.attr,
#endif
    //end, fpc, andy hong, ree porting, remove, 20160719
    &dev_attr_irq.attr,
    NULL
};

static const struct attribute_group attribute_group = {
    .attrs = attributes,
};
struct fpc1020_data *fpc_wdata = NULL;
static void fpc_work(struct work_struct *pws) {
    printk("%s: start.\n", __func__);
    if (fpc_wdata->wakeup_enabled) {
        wake_lock_timeout(&fpc_wdata->ttw_wl,
                msecs_to_jiffies(FPC_TTW_HOLD_TIME));
    }

    //fpc, andy hong, for ree porting, crash at enroll, 20161122
#if 0 //Originally
    sysfs_notify(&fpc_wdata->dev->kobj, NULL, dev_attr_irq.attr.name);
#else
    fpc1020_irq_occur = 1;
    wake_up_interruptible(&fpc1020_irq_return);
#endif
    //end, fpc, andy hong, for ree porting, crash at enroll, 20161122
}
static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
    fpc_wdata = handle;
    smp_rmb();
    queue_work(workq, &work);
    return IRQ_HANDLED;
}

//fpc, andy hong, for ree porting, crash at enroll, 20161122
#if 1
static int fpc1020_irq_function(void *pData)
{
    struct fpc1020_data *fpc1020 = (struct fpc1020_data *)(pData);

    printk("fpc, andy hong, fpc1020_irq_function, entry\n");

    while (!kthread_should_stop()) {
        printk("fpc, andy hong, fpc1020_irq_function, wait_event_interruptible\n");
        wait_event_interruptible(fpc1020_irq_return,
                (fpc1020_irq_occur != 0));
        printk("fpc, andy hong, fpc1020_irq_function, sysfs_notify\n");
        fpc1020_irq_occur = 0;
        sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);
    }

    return 0;
}

static int fpc1020_irq_init(struct fpc1020_data *fpc1020)
{
    int error = 0;

    printk(KERN_INFO "fpc, andy hong, %s\n", __func__);

    init_waitqueue_head(&fpc1020_irq_return);

    fpc1020_irq_occur = 0;

    fpc1020_irq_thread = kthread_run(fpc1020_irq_function,
            fpc1020, "fpc1020_irq_function");

    if (IS_ERR(fpc1020_irq_thread)) {
        printk("fpc, andy hong, fpc1020_irq_thread, kthread_run failed.\n");
        error = (int)PTR_ERR(fpc1020_irq_thread);
    }

    return error;
}

/* -------------------------------------------------------------------- */
static int fpc1020_irq_destroy(struct fpc1020_data *fpc1020)
{
    int error = 0;

    printk(KERN_INFO "fpc, andy hong, %s\n", __func__);

    if (fpc1020_irq_thread) {
        fpc1020_irq_occur = 1;
        wake_up_interruptible(&fpc1020_irq_return);
        kthread_stop(fpc1020_irq_thread);
    }

    return error;
}
#endif
//end, fpc, andy hong, for ree porting, crash at enroll, 20161122

static int fpc1020_probe(struct platform_device *pldev)
{
    struct device *dev = &pldev->dev;
    int rc = 0;
    int irqf;
    struct device_node *np = dev->of_node;
    u32 val;
    const char *idev_name;
    struct fpc1020_data *fpc1020;

    //mt_spi_enable_clk();
    //printk("spi enabled\n");
    fpc_finger_get_gpio_info(pldev);
    fpc_finger_set_power(1);

    fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020), GFP_KERNEL);
    if (!fpc1020) {
        dev_err(dev,
                "failed to allocate memory for struct fpc1020_data\n");
        rc = -ENOMEM;
        goto exit;
    }

    fpc1020->dev = dev;
    dev_set_drvdata(dev, fpc1020);
    fpc1020->pldev = pldev;

    if (!np) {
        dev_err(dev, "no of node found\n");
        dev_err(dev, "fiona test no of node found.\n" );
        rc = -EINVAL;
        goto exit;
    }

    /* Configure the direction of the gpios */

    fpc_finger_set_reset(0);
    usleep_range(FPC1020_RESET_LOW_US, FPC1020_RESET_LOW_US + 100);

    fpc_finger_set_reset(1);
    usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

    fpc1020->irq_num=fpc1020_get_irqNum();

    rc = of_property_read_u32(np, "fpc,event-type", &val);
    fpc1020->event_type = rc < 0 ? EV_MSC : val;

    rc = of_property_read_u32(np, "fpc,event-code", &val);
    fpc1020->event_code = rc < 0 ? MSC_SCAN : val;

    irq_gpio = of_get_named_gpio(np,"irq_gpio",0);

    fpc1020->idev = devm_input_allocate_device(dev);
    if (!fpc1020->idev) {
        dev_err(dev, "failed to allocate input device\n");
        rc = -ENOMEM;
        goto exit;
    }
    input_set_capability(fpc1020->idev, fpc1020->event_type,
            fpc1020->event_code);

    if (!of_property_read_string(np, "input-device-name", &idev_name)) {
        fpc1020->idev->name = idev_name;
    } else {
        snprintf(fpc1020->idev_name, sizeof(fpc1020->idev_name),
                "fpc_irq@%s", dev_name(dev));
        fpc1020->idev->name = fpc1020->idev_name;
    }
    printk("fiona test 000.\n" );
    /* Also register the key for wake up */
    set_bit(EV_KEY,	fpc1020->idev->evbit);
    set_bit(EV_PWR,	fpc1020->idev->evbit);
    set_bit(KEY_WAKEUP, fpc1020->idev->keybit);
    set_bit(KEY_POWER, fpc1020->idev->keybit);
    rc = input_register_device(fpc1020->idev);
    fpc1020->wakeup_enabled = false;
    if (rc) {
        dev_err(dev, "failed to register input device\n");
        goto exit;
    }
    dev_err( dev, "fiona test 001.\n" );
    irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
    if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
        irqf |= IRQF_NO_SUSPEND;
        device_init_wakeup(dev, 1);
    }
    mutex_init(&fpc1020->lock);

    //fpc, andy hong, for ree porting, crash at enroll, 20161122
#if 1
    printk(KERN_INFO "fpc, andy hong, %s, fpc1020_irq_init\n", __func__);
    fpc1020_irq_init(fpc1020);
#endif
    //end, fpc, andy hong, for ree porting, crash at enroll, 20161122

    dev_err( dev, "fiona test 002.\n" );
    INIT_WORK(&work, fpc_work);
    pr_debug("irq_num=%d/n",fpc1020->irq_num);
    /*rc = devm_request_threaded_irq(dev, fpc1020->irq_num,
      NULL, fpc1020_irq_handler, irqf,
      dev_name(dev), fpc1020);*/
    workq = create_singlethread_workqueue("fpc_workqueue");
    if (!workq) {
        printk("%s: create_single_workqueue failed\n", __func__);
        return -ENOMEM;
    }

    rc = request_threaded_irq(fpc1020->irq_num,
            fpc1020_irq_handler, NULL,irqf,
            dev_name(dev), fpc1020);
    pr_debug("rc=%d/n",rc);

    if (rc) {
        dev_err(dev, "could not request irq %d\n",fpc1020->irq_num);
        goto exit;
    }
    dev_info(dev, "requested irq %d\n",fpc1020->irq_num);

    /* Request that the interrupt should be wakeable */
    enable_irq_wake( fpc1020->irq_num);
    wake_lock_init(&fpc1020->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");
    dev_err( dev, "fiona test 003.\n" );	
    rc = sysfs_create_group(&dev->kobj, &attribute_group);
    printk("rc=%d/n",rc);
    if (rc) {
        dev_err(dev, "could not create sysfs\n");
        goto exit;
    }

    printk("%s: ok\n", __func__);
exit:
    //fpc, andy hong, for ree porting, crash at enroll, 20161122
#if 1
    if(rc)
    {
        printk("fpc, andy hong, fpc1020_probe, fpc1020_irq_destroy\n");
        fpc1020_irq_destroy(fpc1020);
    }
#endif
    //end, fpc, andy hong, for ree porting, crash at enroll, 20161122
    return rc;
}

int fpc1020_get_irqNum(void)
{
    unsigned int gpiopin, debounce, fpc_irq = 0;
    u32 ints[4] = {0, 0,0,0};

    irq_node = of_find_compatible_node(NULL, NULL, "mediatek, FINGERPRINT-eint");
    if (irq_node) {
        of_property_read_u32_array(irq_node, "interrupts", ints, ARRAY_SIZE(ints));
        gpiopin = ints[0];
        debounce = ints[3];
        /*mt_gpio_set_debounce(gpiopin, debounce);*/
        fpc_irq = irq_of_parse_and_map(irq_node, 0);
        pr_err("fpc_irq = %u\n", fpc_irq);
    } else
        pr_err("[fpc]%s can't find compatible irq_node\n", __func__);

    return fpc_irq;
}

static int fpc1020_remove(struct platform_device *pldev)
{
    struct  fpc1020_data *fpc1020 = dev_get_drvdata(&pldev->dev);

    //fpc, andy hong, for ree porting, crash at enroll, 20161122
#if 1
    printk(KERN_INFO "fpc, andy hong, %s, fpc1020_irq_destroy\n", __func__);
    fpc1020_irq_destroy(fpc1020);
#endif
    //end, fpc, andy hong, for ree porting, crash at enroll, 20161122
    sysfs_remove_group(&pldev->dev.kobj, &attribute_group);
    mutex_destroy(&fpc1020->lock);
    wake_lock_destroy(&fpc1020->ttw_wl);  
    dev_info(&pldev->dev, "%s\n", __func__);
    return 0;
}


static struct of_device_id fpc1020_of_match[] = {
    { .compatible = "mediatek,fpc_finger", },
    {}
};

MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
    .driver = {
        .name	= "fpc_irq",
        .owner	= THIS_MODULE,
        .of_match_table = fpc1020_of_match,
    },
    .probe	= fpc1020_probe,
    .remove	= fpc1020_remove
};


MODULE_DEVICE_TABLE(of, fpc1020_spi_of_match);

#ifndef CONFIG_SPI_MT65XX
static struct mt_chip_conf fpc_spi_chip_config = {
    .setuptime = 15,
    .holdtime = 15,
    .high_time = 11,
    .low_time = 11,
    .cs_idletime = 20,
    .ulthgh_thrsh = 0,
    //.cs_pol = ,
    //.sample_sel = ,
    .cpol = 0,
    .cpha = 0,
    .rx_mlsb = 1,
    .tx_mlsb = 1,
    .tx_endian = 0,
    .rx_endian = 0,
    .com_mod = DMA_TRANSFER,
    .pause = 0,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};
static struct spi_board_info spi_board_devs_fpc[] __initdata = {
    [0] = {
        .modalias = "spidev",
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
        .controller_data = (void *)(&fpc_spi_chip_config),
    }
};
#endif

static int __init fpc1020_init(void)
{
    printk("%s\n", __func__);
    //spi_register_board_info(spi_board_devs_fpc, 1);
    return (platform_driver_register(&fpc1020_driver) != 0)? EINVAL : 0;
}

static void __exit fpc1020_exit(void)
{
    printk("%s\n", __func__);

    platform_driver_unregister(&fpc1020_driver);
}

module_init(fpc1020_init);
module_exit(fpc1020_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
