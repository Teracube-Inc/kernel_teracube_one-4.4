/* Goodix's  fingerprint sensor linux driver 
 * 2010 - 2015 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#else
#include <linux/clk.h>
#endif

#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

/* MTK header */
#ifndef CONFIG_SPI_MT65XX
#include "mtk_spi.h"
#include "mtk_spi_hal.h"
#endif
#include "mtk_gpio.h"

#include "gf_common.h"


/**************************defination******************************/
#define GF_DEV_NAME "goodix_fp"  // atb modify 20161222
#define GF_DEV_MAJOR 0	/* assigned */

#define GF_CLASS_NAME "goodix_fp"
#define GF_INPUT_NAME "gf-keys"

#define GF_LINUX_VERSION "V1.01.04"

#define GF_NETLINK_ROUTE 29   /* for GF test temporary, need defined in include/uapi/linux/netlink.h */
#define MAX_NL_MSG_LEN 16

#if GF358
#define GF_CHIP_ID_LO				0x0000
#define GF_CHIP_ID_HI				0x0002
#endif 
/*************************************************************/

/* debug log setting */
u8 g_debug_level = ERR_LOG;

/* align=2, 2 bytes align */
/* align=4, 4 bytes align */
/* align=8, 8 bytes align */
#define ROUND_UP(x, align)		((x+(align-1))&~(align-1))
/* for Upstream SPI ,just tell SPI about the clock */
#ifdef CONFIG_SPI_MT65XX
u32 gf_spi_speed = 1*1000000;
#endif
#define QMM_LOG()  printk("*** %s, %d ***\n", __func__, __LINE__)
/*************************************************************/
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned int bufsiz = (20 * 1024);
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "maximum data bytes for SPI message");

#ifdef CONFIG_OF
static const struct of_device_id gf_of_match[] = {
    { .compatible = "mediatek,vanzo-fp", },
    {},
};
MODULE_DEVICE_TABLE(of, gf_of_match);
#endif


#if GF358
static unsigned int g_chipID = 0;
#endif 

/* for netlink use */
static int pid = 0;

static u8 g_vendor_id = 0;

static ssize_t gf_debug_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t gf_debug_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, gf_debug_show, gf_debug_store);

static struct attribute *gf_debug_attrs[] = {
    &dev_attr_debug.attr,
    NULL
};

static const struct attribute_group gf_debug_attr_group = {
    .attrs = gf_debug_attrs,
    .name = "debug"
};
#ifndef CONFIG_SPI_MT65XX
const struct mt_chip_conf spi_ctrdata = {
    .setuptime = 10,
    .holdtime = 10,
    .high_time = 50, /* 1MHz */
    .low_time = 50,
    .cs_idletime = 10,
    .ulthgh_thrsh = 0,

    .cpol = SPI_CPOL_0,
    .cpha = SPI_CPHA_0,

    .rx_mlsb = SPI_MSB,
    .tx_mlsb = SPI_MSB,

    .tx_endian = SPI_LENDIAN,
    .rx_endian = SPI_LENDIAN,

    .com_mod = FIFO_TRANSFER,
    /* .com_mod = DMA_TRANSFER, */

    .pause = 0,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};
#endif
/* -------------------------------------------------------------------- */
/* timer function								*/
/* -------------------------------------------------------------------- */
#define TIME_START	   0
#define TIME_STOP	   1

static long int prev_time, cur_time;

long int kernel_time(unsigned int step)
{
    cur_time = ktime_to_us(ktime_get());
    if (step == TIME_START) {
        prev_time = cur_time;
        return 0;
    } else if (step == TIME_STOP) {
        gf_debug(DEBUG_LOG, "%s, use: %ld us\n", __func__, (cur_time - prev_time));
        return cur_time - prev_time;
    }
    prev_time = cur_time;
    return -1;
}

/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration								  */
/* -------------------------------------------------------------------- */
static int gf_get_gpio_dts_info(struct gf_device *gf_dev)
{
#ifdef CONFIG_OF
    int ret;

             printk("%s, from dts pinctrl\n", __func__);

            gf_dev->pinctrl_gpios = devm_pinctrl_get(&gf_dev->spi->dev);
            if (IS_ERR(gf_dev->pinctrl_gpios)) {
                ret = PTR_ERR(gf_dev->pinctrl_gpios);
                printk("%s can't find fingerprint pinctrl\n", __func__);
                return ret;
            }
    gf_dev->pins_irq = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "fp_irq_mode");
    if (IS_ERR(gf_dev->pins_irq)) {
        ret = PTR_ERR(gf_dev->pins_irq);
        gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl irq\n", __func__);
        return ret;
    }

    gf_dev->pins_reset_high = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "fp_rst_high");
    if (IS_ERR(gf_dev->pins_reset_high)) {
        ret = PTR_ERR(gf_dev->pins_reset_high);
        gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl reset_high\n", __func__);
        return ret;
    }
    gf_dev->pins_reset_low = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "fp_rst_low");
    if (IS_ERR(gf_dev->pins_reset_low)) {
        ret = PTR_ERR(gf_dev->pins_reset_low);
        gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl reset_low\n", __func__);
        return ret;
    }
    gf_debug(DEBUG_LOG, "%s, get pinctrl success!\n", __func__);

#endif
    return 0;
}

static int gf_get_sensor_dts_info(void)
{
    struct device_node *node = NULL;
    int value;

    node = of_find_compatible_node(NULL, NULL, "goodix,goodix-fp");
    if (node) {
        of_property_read_u32(node, "netlink-event", &value);
        gf_debug(DEBUG_LOG, "%s, get netlink event[%d] from dts\n", __func__, value);
    } else {
        gf_debug(ERR_LOG, "%s failed to get device node!\n", __func__);
        return -ENODEV;
    }

    return 0;
}

//struct regulator *reg = NULL;
//static void gf_hw_power_enable(u8 onoff)
static void gf_hw_power_enable(struct gf_device* gf_dev)
{
#if 0
    /* TODO: LDO configure */
    //int ret = 0;
    static int enable = 1;
    if (onoff && enable) {
        /* TODO:  set power  according to actual situation  */
        //		hwPowerOn(MT6331_POWER_LDO_VIBR, VOL_2800, "fingerprint");
        //2.8V
        //reg = regulator_get(NULL,"vfingerprint");
        //ret = regulator_set_voltage(reg, 2800000, 2800000);
        //ret = regulator_enable(reg);	/*enable regulator*/
        enable = 0;
    }
    else if (!onoff && !enable) {
        //		hwPowerDown(MT6331_POWER_LDO_VIBR, "fingerprint");
        //if(reg != NULL)
        //{
        //regulator_disable(reg);
        //}		
        enable = 1;
    }
#endif

#if 1
    return;
#endif
}

static void gf_spi_clk_enable(struct gf_device *gf_dev, u8 bonoff)
{
    static int count = 0;
#ifdef CONFIG_MTK_CLKMGR
    if (bonoff && (count == 0)) {
        gf_debug(DEBUG_LOG, "%s, start to enable spi clk && count = %d.\n", __func__, count);
        enable_clock(MT_CG_PERI_SPI0, "spi");
        count = 1;
    } else if ((count > 0) && (bonoff == 0)) {
        gf_debug(DEBUG_LOG, "%s, start to disable spi clk&& count = %d.\n", __func__, count);
        disable_clock(MT_CG_PERI_SPI0, "spi");
        count = 0;
    }
#elif defined(MT6797)
    /* changed after MT6797 platform */
    struct mt_spi_t *ms = NULL;

    ms = spi_master_get_devdata(gf_dev->spi->master);

    if (bonoff && (count == 0)) {
        mt_spi_enable_clk(ms);
        count = 1;
    } else if ((count > 0) && (bonoff == 0)) {
        mt_spi_disable_clk(ms);
        count = 0;
    }
#else
    if (bonoff && (count == 0)) {
        mt_spi_enable_master_clk(gf_dev->spi);
        count = 1;
    } else if ((count > 0) && (bonoff == 0)) {
        mt_spi_disable_master_clk(gf_dev->spi);
        count = 0;
    }
#endif
}

static void gf_bypass_flash_gpio_cfg(void)
{
    /* TODO: by pass flash IO config, default connect to GND */
}

static void gf_irq_gpio_cfg(struct gf_device *gf_dev)
{
#ifdef CONFIG_OF
    struct device_node *node;

    pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_irq);

    node = of_find_compatible_node(NULL, NULL, "mediatek, FINGERPRINT-eint");
    if (node) {
        gf_dev->irq_num = irq_of_parse_and_map(node, 0);
        gf_debug(INFO_LOG, "%s, gf_irq = %d\n", __func__, gf_dev->irq_num);
        gf_dev->irq = gf_dev->irq_num;
    } else
        gf_debug(ERR_LOG, "%s can't find compatible node\n", __func__);

#endif
}

static void gf_reset_gpio_cfg(struct gf_device *gf_dev)
{
#ifdef CONFIG_OF
    pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_high);
#endif

}

/* delay ms after reset */
static void gf_hw_reset(struct gf_device *gf_dev, u8 delay)
{
#ifdef CONFIG_OF
    pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_low);
    //mdelay(3);
    mdelay((delay > 11) ? delay : 11);
    pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_high);
#endif

    if (delay) {
        /* delay is configurable */
        mdelay(delay);
    }
}

static void gf_enable_irq(struct gf_device *gf_dev)
{
    if (1 == gf_dev->irq_count) {
        gf_debug(ERR_LOG, "%s, irq already enabled\n", __func__);
    } else {
        enable_irq(gf_dev->irq);
        gf_dev->irq_count = 1;
        gf_debug(DEBUG_LOG, "%s enable interrupt!\n", __func__);
    }
}

static void gf_disable_irq(struct gf_device *gf_dev)
{
    if (0 == gf_dev->irq_count) {
        gf_debug(ERR_LOG, "%s, irq already disabled\n", __func__);
    } else {
        disable_irq(gf_dev->irq);
        gf_dev->irq_count = 0;
        gf_debug(DEBUG_LOG, "%s disable interrupt!\n", __func__);
    }
}


/* -------------------------------------------------------------------- */
/* netlink functions                 */
/* -------------------------------------------------------------------- */
void gf_netlink_send(struct gf_device *gf_dev, const int command)
{
    struct nlmsghdr *nlh = NULL;
    struct sk_buff *skb = NULL;
    int ret;

    gf_debug(INFO_LOG, "[%s] : enter, send command %d\n", __func__, command);
    if (NULL == gf_dev->nl_sk) {
        gf_debug(ERR_LOG, "[%s] : invalid socket\n", __func__);
        return;
    }

    if (0 == pid) {
        gf_debug(ERR_LOG, "[%s] : invalid native process pid\n", __func__);
        return;
    }

    /*alloc data buffer for sending to native*/
    /*malloc data space at least 1500 bytes, which is ethernet data length*/
    skb = alloc_skb(MAX_NL_MSG_LEN, GFP_ATOMIC);
    if (skb == NULL) {
        return;
    }

    nlh = nlmsg_put(skb, 0, 0, 0, MAX_NL_MSG_LEN, 0);
    if (!nlh) {
        gf_debug(ERR_LOG, "[%s] : nlmsg_put failed\n", __func__);
        kfree_skb(skb);
        return;
    }

    NETLINK_CB(skb).portid = 0;
    NETLINK_CB(skb).dst_group = 0;

    *(char *)NLMSG_DATA(nlh) = command;
    ret = netlink_unicast(gf_dev->nl_sk, skb, pid, MSG_DONTWAIT);
    if (ret == 0) {
        gf_debug(ERR_LOG, "[%s] : send failed\n", __func__);
        return;
    }

    gf_debug(INFO_LOG, "[%s] : send done, data length is %d\n", __func__, ret);
}

static void gf_netlink_recv(struct sk_buff *__skb)
{
    struct sk_buff *skb = NULL;
    struct nlmsghdr *nlh = NULL;
    char str[128];

    gf_debug(INFO_LOG, "[%s] : enter \n", __func__);

    skb = skb_get(__skb);
    if (skb == NULL) {
        gf_debug(ERR_LOG, "[%s] : skb_get return NULL\n", __func__);
        return;
    }

    /* presume there is 5byte payload at leaset */
    if (skb->len >= NLMSG_SPACE(0)) {
        nlh = nlmsg_hdr(skb);
        memcpy(str, NLMSG_DATA(nlh), sizeof(str));
        pid = nlh->nlmsg_pid;
        gf_debug(INFO_LOG, "[%s] : pid: %d, msg: %s\n", __func__, pid, str);

    } else {
        gf_debug(ERR_LOG, "[%s] : not enough data length\n", __func__);
    }

    kfree_skb(skb);
}

static int gf_netlink_init(struct gf_device *gf_dev)
{
    struct netlink_kernel_cfg cfg;

    memset(&cfg, 0, sizeof(struct netlink_kernel_cfg));
    cfg.input = gf_netlink_recv;

    gf_dev->nl_sk = netlink_kernel_create(&init_net, GF_NETLINK_ROUTE, &cfg);
    if (gf_dev->nl_sk == NULL) {
        gf_debug(ERR_LOG, "[%s] : netlink create failed\n", __func__);
        return -1;
    }

    gf_debug(INFO_LOG, "[%s] : netlink create success\n", __func__);
    return 0;
}

static int gf_netlink_destroy(struct gf_device *gf_dev)
{
    if (gf_dev->nl_sk != NULL) {
        netlink_kernel_release(gf_dev->nl_sk);
        gf_dev->nl_sk = NULL;
        return 0;
    }

    gf_debug(ERR_LOG, "[%s] : no netlink socket yet\n", __func__);
    return -1;
}

/* -------------------------------------------------------------------- */
/* early suspend callback and suspend/resume functions          */
/* -------------------------------------------------------------------- */
#ifdef CONFIG_HAS_EARLYSUSPEND
static void gf_early_suspend(struct early_suspend *handler)
{
    struct gf_device *gf_dev = NULL;

    gf_dev = container_of(handler, struct gf_device, early_suspend);
    gf_debug(INFO_LOG, "[%s] enter\n", __func__);

    gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_OFF);
}

static void gf_late_resume(struct early_suspend *handler)
{
    struct gf_device *gf_dev = NULL;

    gf_dev = container_of(handler, struct gf_device, early_suspend);
    gf_debug(INFO_LOG, "[%s] enter\n", __func__);

    gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_ON);
}
#else

static int gf_fb_notifier_callback(struct notifier_block *self,
        unsigned long event, void *data)
{
    struct gf_device *gf_dev = NULL;
    struct fb_event *evdata = data;
    unsigned int blank;
    int retval = 0;
    FUNC_ENTRY();

    /* If we aren't interested in this event, skip it immediately ... */
    if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */)
        return 0;

    gf_dev = container_of(self, struct gf_device, notifier);
    blank = *(int *)evdata->data;

    gf_debug(INFO_LOG, "[%s] : enter, blank=0x%x\n", __func__, blank);

    switch (blank) {
        case FB_BLANK_UNBLANK:
            gf_debug(INFO_LOG, "[%s] : lcd on notify\n", __func__);
            gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_ON);
            break;

        case FB_BLANK_POWERDOWN:
            gf_debug(INFO_LOG, "[%s] : lcd off notify\n", __func__);
            gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_OFF);
            break;

        default:
            gf_debug(INFO_LOG, "[%s] : other notifier, ignore\n", __func__);
            break;
    }
    FUNC_EXIT();
    return retval;
}

#endif //CONFIG_HAS_EARLYSUSPEND

/* -------------------------------------------------------------------- */
/* file operation function                                              */
/* -------------------------------------------------------------------- */
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    gf_debug(ERR_LOG, "%s: Not support write opertion \n", __func__);
    return -EFAULT;
}

static ssize_t gf_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
    gf_debug(ERR_LOG, "%s: Not support write opertion \n", __func__);
    return -EFAULT;
}

static irqreturn_t gf_irq(int irq, void *handle)
{
    struct gf_device *gf_dev = (struct gf_device *)handle;
    FUNC_ENTRY();
    gf_netlink_send(gf_dev, GF_NETLINK_IRQ);
    FUNC_EXIT();
    return IRQ_HANDLED;
}


static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct gf_device *gf_dev = NULL;
    struct gf_key gf_key;
    gf_nav_event_t nav_event = GF_NAV_NONE;
    uint32_t nav_input = 0;
    uint32_t key_input = 0;

    int retval = 0;
    u8  buf    = 0;
    u8 netlink_route = GF_NETLINK_ROUTE;
    struct gf_ioc_chip_info info;

    FUNC_ENTRY();
    if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
        return -EINVAL;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
        retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));

    if (retval == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

    if (retval)
        return -EINVAL;

    gf_dev = (struct gf_device *)filp->private_data;
    if (!gf_dev) {
        gf_debug(ERR_LOG, "%s: gf_dev IS NULL ======\n", __func__);
        return -EINVAL;
    }

    switch (cmd) {
        case GF_IOC_INIT:
            gf_debug(INFO_LOG, "%s: GF_IOC_INIT gf init======\n", __func__);
            gf_debug(INFO_LOG, "%s: Linux Version %s\n", __func__, GF_LINUX_VERSION);

            if (copy_to_user((void __user *)arg, (void *)&netlink_route, sizeof(u8))) {
                retval = -EFAULT;
                break;
            }

            if (gf_dev->system_status) {
                gf_debug(INFO_LOG, "%s: system re-started======\n", __func__);
                break;
            }

            //gf_dev->irq_count = 1;
            gf_dev->sig_count = 0;
            gf_dev->system_status = 1;

            gf_debug(INFO_LOG, "%s: gf init finished======\n", __func__);
            break;

        case GF_IOC_CHIP_INFO:
            if (copy_from_user(&info, (struct gf_ioc_chip_info *)arg, sizeof(struct gf_ioc_chip_info))) {
                retval = -EFAULT;
                break;
            }
            g_vendor_id = info.vendor_id;

            gf_debug(INFO_LOG, "%s: vendor_id 0x%x\n", __func__, g_vendor_id);
            gf_debug(INFO_LOG, "%s: mode 0x%x\n", __func__, info.mode);
            gf_debug(INFO_LOG, "%s: operation 0x%x\n", __func__, info.operation);
            break;

        case GF_IOC_EXIT:
            gf_debug(INFO_LOG, "%s: GF_IOC_EXIT ======\n", __func__);
            gf_disable_irq(gf_dev);
            if (gf_dev->irq) {
                free_irq(gf_dev->irq, gf_dev);
                gf_dev->irq_count = 0;
                gf_dev->irq = 0;
            }

#ifdef CONFIG_HAS_EARLYSUSPEND
            if (gf_dev->early_suspend.suspend)
                unregister_early_suspend(&gf_dev->early_suspend);
#else
            fb_unregister_client(&gf_dev->notifier);
#endif

            gf_dev->system_status = 0;
            gf_debug(INFO_LOG, "%s: gf exit finished ======\n", __func__);
            break;

        case GF_IOC_RESET:
            gf_debug(INFO_LOG, "%s: chip reset command\n", __func__);
            gf_hw_reset(gf_dev, 60);
            break;

        case GF_IOC_ENABLE_IRQ:
            gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_IRQ ======\n", __func__);
            gf_enable_irq(gf_dev);
            break;

        case GF_IOC_DISABLE_IRQ:
            gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_IRQ ======\n", __func__);
            gf_disable_irq(gf_dev);
            break;

        case GF_IOC_ENABLE_SPI_CLK:
            gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_SPI_CLK ======\n", __func__);
            gf_spi_clk_enable(gf_dev, 1);
            break;

        case GF_IOC_DISABLE_SPI_CLK:
            gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_SPI_CLK ======\n", __func__);
            gf_spi_clk_enable(gf_dev, 0);
            break;

        case GF_IOC_ENABLE_POWER:
            gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_POWER ======\n", __func__);
            //gf_hw_power_enable(1);
            gf_hw_power_enable(gf_dev);
            break;

        case GF_IOC_DISABLE_POWER:
            gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_POWER ======\n", __func__);
            //gf_hw_power_enable(0);
            break;

        case GF_IOC_INPUT_KEY_EVENT:
            if (copy_from_user(&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
                gf_debug(ERR_LOG, "Failed to copy input key event from user to kernel\n");
                retval = -EFAULT;
                break;
            }

            if (GF_KEY_HOME == gf_key.key) {
                key_input = GF_KEY_INPUT_HOME;
            } else if (GF_KEY_POWER == gf_key.key) {
                key_input = GF_KEY_INPUT_POWER;
            } else if (GF_KEY_CAMERA == gf_key.key) {
                key_input = GF_KEY_INPUT_CAMERA;
            } else {
                /* add special key define */
                key_input = gf_key.key;
            }
            gf_debug(INFO_LOG, "%s: received key event[%d], key=%d, value=%d\n",
                    __func__, key_input, gf_key.key, gf_key.value);

            if ((GF_KEY_POWER == gf_key.key || GF_KEY_CAMERA == gf_key.key) && (gf_key.value == 1)) {
                input_report_key(gf_dev->input, key_input, 1);
                input_sync(gf_dev->input);
                input_report_key(gf_dev->input, key_input, 0);
                input_sync(gf_dev->input);
            }

            if (GF_KEY_HOME == gf_key.key) {
                key_input = KEY_F11;
                input_report_key(gf_dev->input, key_input, gf_key.value);
                input_sync(gf_dev->input);
            }

            break;

        case GF_IOC_NAV_EVENT:
            gf_debug(ERR_LOG, "nav event");
            if (copy_from_user(&nav_event, (gf_nav_event_t *)arg, sizeof(gf_nav_event_t))) {
                gf_debug(ERR_LOG, "Failed to copy nav event from user to kernel\n");
                retval = -EFAULT;
                break;
            }

            switch (nav_event) {
                case GF_NAV_FINGER_DOWN:
                    gf_debug(ERR_LOG, "nav finger down");
                    break;

                case GF_NAV_FINGER_UP:
                    gf_debug(ERR_LOG, "nav finger up");
                    break;

                case GF_NAV_DOWN:
                    nav_input = GF_NAV_INPUT_DOWN;
                    gf_debug(ERR_LOG, "nav down");
                    break;

                case GF_NAV_UP:
                    nav_input = GF_NAV_INPUT_UP;
                    gf_debug(ERR_LOG, "nav up");
                    break;

                case GF_NAV_LEFT:
                    nav_input = GF_NAV_INPUT_LEFT;
                    gf_debug(ERR_LOG, "nav left");
                    break;

                case GF_NAV_RIGHT:
                    nav_input = GF_NAV_INPUT_RIGHT;
                    gf_debug(ERR_LOG, "nav right");
                    break;

                case GF_NAV_CLICK:
                    nav_input = GF_NAV_INPUT_CLICK;
                    gf_debug(ERR_LOG, "nav click");
                    break;

                case GF_NAV_HEAVY:
                    nav_input = GF_NAV_INPUT_HEAVY;
                    break;

                case GF_NAV_LONG_PRESS:
                    nav_input = GF_NAV_INPUT_LONG_PRESS;
                    break;

                case GF_NAV_DOUBLE_CLICK:
                    nav_input = GF_NAV_INPUT_DOUBLE_CLICK;
                    break;

                default:
                    gf_debug(INFO_LOG, "%s: not support nav event nav_event: %d ======\n", __func__, nav_event);
                    break;
            }

            if ((nav_event != GF_NAV_FINGER_DOWN) && (nav_event != GF_NAV_FINGER_UP)) {
                input_report_key(gf_dev->input, nav_input, 1);
                input_sync(gf_dev->input);
                input_report_key(gf_dev->input, nav_input, 0);
                input_sync(gf_dev->input);
            }
            break;

        case GF_IOC_ENTER_SLEEP_MODE:
            gf_debug(INFO_LOG, "%s: GF_IOC_ENTER_SLEEP_MODE ======\n", __func__);
            break;

        case GF_IOC_GET_FW_INFO:
            gf_debug(INFO_LOG, "%s: GF_IOC_GET_FW_INFO ======\n", __func__);
            buf = gf_dev->need_update;

            gf_debug(DEBUG_LOG, "%s: firmware info  0x%x\n", __func__, buf);
            if (copy_to_user((void __user *)arg, (void *)&buf, sizeof(u8))) {
                gf_debug(ERR_LOG, "Failed to copy data to user\n");
                retval = -EFAULT;
            }

            break;
        case GF_IOC_REMOVE:
            gf_debug(INFO_LOG, "%s: GF_IOC_REMOVE ======\n", __func__);
            break;

        case GF_IOC_TRANSFER_CMD:
            gf_debug(INFO_LOG, "%s: GF_IOC_TRANSFER_CMD ======\n", __func__);
            break;

        case GF_IOC_TRANSFER_RAW_CMD:
            mutex_lock(&gf_dev->buf_lock);
            retval =gf_ioctl_transfer_raw_cmd(gf_dev,arg,bufsiz);
            mutex_unlock(&gf_dev->buf_lock);
            break;
#ifndef CONFIG_SPI_MT65XX
        case GF_IOC_SPI_INIT_CFG_CMD:
            retval = gf_ioctl_spi_init_cfg_cmd(&gf_dev->spi_mcc,arg);
            break;
#endif
        default:
            gf_debug(ERR_LOG, "gf doesn't support this command(%x)\n", cmd);
            break;
    }

    FUNC_EXIT();
    return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int retval = 0;

    FUNC_ENTRY();

    retval = filp->f_op->unlocked_ioctl(filp, cmd, arg);

    FUNC_EXIT();
    return retval;
}
#endif

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
    gf_debug(ERR_LOG, "Not support poll opertion in TEE version\n");
    return -EFAULT;
}


/* -------------------------------------------------------------------- */
/* devfs                                                              */
/* -------------------------------------------------------------------- */
static ssize_t gf_debug_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    gf_debug(INFO_LOG, "%s: Show debug_level = 0x%x\n", __func__, g_debug_level);
    return sprintf(buf, "vendor id 0x%x\n", g_vendor_id);
}

static ssize_t gf_debug_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct gf_device *gf_dev =  dev_get_drvdata(dev);
    int retval = 0;
    //u8 flag = 0;
    struct mt_spi_t *ms = NULL;

    ms = spi_master_get_devdata(gf_dev->spi->master);

    if (!strncmp(buf, "-8", 2)) {
        gf_debug(INFO_LOG, "%s: parameter is -8, enable spi clock test===============\n", __func__);
#if defined(MT6797)
        mt_spi_enable_clk(ms);
#else
        mt_spi_enable_master_clk(gf_dev->spi);
#endif
    } else if (!strncmp(buf, "-9", 2)) {
        gf_debug(INFO_LOG, "%s: parameter is -9, disable spi clock test===============\n", __func__);
#if defined(MT6797)
        mt_spi_disable_clk(ms);
#else
        mt_spi_disable_master_clk(gf_dev->spi);
#endif
    } else if (!strncmp(buf, "-10", 3)) {
        gf_debug(INFO_LOG, "%s: parameter is -10, gf init start===============\n", __func__);

        gf_irq_gpio_cfg(gf_dev);
        retval = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
                IRQF_TRIGGER_RISING | IRQF_ONESHOT, dev_name(&(gf_dev->spi->dev)), gf_dev);
        if (!retval)
            gf_debug(INFO_LOG, "%s irq thread request success!\n", __func__);
        else
            gf_debug(ERR_LOG, "%s irq thread request failed, retval=%d\n", __func__, retval);

        gf_dev->irq_count = 1;
        gf_disable_irq(gf_dev);

#if defined(CONFIG_HAS_EARLYSUSPEND)
        gf_debug(INFO_LOG, "[%s] : register_early_suspend\n", __func__);
        gf_dev->early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
        gf_dev->early_suspend.suspend = gf_early_suspend,
            gf_dev->early_suspend.resume = gf_late_resume,
            register_early_suspend(&gf_dev->early_suspend);
#else
        /* register screen on/off callback */
        gf_dev->notifier.notifier_call = gf_fb_notifier_callback;
        fb_register_client(&gf_dev->notifier);
#endif

        gf_dev->sig_count = 0;

        gf_debug(INFO_LOG, "%s: gf init finished======\n", __func__);

    } else if (!strncmp(buf, "-11", 3)) {
        gf_debug(INFO_LOG, "%s: parameter is -11, enable irq===============\n", __func__);
        gf_enable_irq(gf_dev);

    } else if (!strncmp(buf, "-12", 3)) {
        gf_debug(INFO_LOG, "%s: parameter is -12, GPIO test===============\n", __func__);
        gf_reset_gpio_cfg(gf_dev);

#if 0
#ifdef CONFIG_OF
        if (flag == 0) {
            pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_miso_pulllow);
            gf_debug(INFO_LOG, "%s: set miso PIN to low\n", __func__);
            flag = 1;
        } else {
            pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_miso_pullhigh);
            gf_debug(INFO_LOG, "%s: set miso PIN to high\n", __func__);
            flag = 0;
        }
#endif
#endif

    } else if (!strncmp(buf, "-13", 3)) {
        gf_debug(INFO_LOG, "%s: parameter is -13, Vendor ID test --> 0x%x\n", __func__, g_vendor_id);
    } else {
        gf_debug(ERR_LOG, "%s: wrong parameter!===============\n", __func__);
    }

    return count;
}

/* -------------------------------------------------------------------- */
/* device function								  */
/* -------------------------------------------------------------------- */
static int gf_open(struct inode *inode, struct file *filp)
{
    struct gf_device *gf_dev = NULL;
    int status = -ENXIO;

    FUNC_ENTRY();
    //vcorefs_request_dvfs_opp(KIR_REESPI, OPPI_PERF);
    mutex_lock(&device_list_lock);
    list_for_each_entry(gf_dev, &device_list, device_entry) {
        if (gf_dev->devno == inode->i_rdev) {
            gf_debug(INFO_LOG, "%s, Found\n", __func__);
            status = 0;
            break;
        }
    }
    mutex_unlock(&device_list_lock);

    if (status == 0) {
        filp->private_data = gf_dev;
        nonseekable_open(inode, filp);
        gf_debug(INFO_LOG, "%s, Success to open device. irq = %d\n", __func__, gf_dev->irq);
    } else {
        gf_debug(ERR_LOG, "%s, No device for minor %d\n", __func__, iminor(inode));
    }

    FUNC_EXIT();
    return status;
}

static int gf_release(struct inode *inode, struct file *filp)
{
    struct gf_device *gf_dev = NULL;
    int    status = 0;

    FUNC_ENTRY();
    gf_dev = filp->private_data;
    if (gf_dev->irq)
        gf_disable_irq(gf_dev);
    gf_dev->need_update = 0;
    FUNC_EXIT();
    return status;
}

static const struct file_operations gf_fops = {
    .owner =	THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.	It'll simplify things
     * too, except for the locking.
     */
    .write =	gf_write,
    .read =		gf_read,
    .unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = gf_compat_ioctl,
#endif
    .open =		gf_open,
    .release =	gf_release,
    .poll	= gf_poll,
};

/*-------------------------------------------------------------------------*/

static int gf_probe(struct spi_device *spi)
{
    struct gf_device *gf_dev = NULL;
    int retval = 0;
    int status = -EINVAL;

    unsigned char rx_test[10] = {0};

#if GF358
    unsigned short chip_id_1 = 0;
    unsigned short chip_id_2 = 0;

#endif 
    //while(1)
    {
        printk("gf_probe\n");
    }
    FUNC_ENTRY();
    QMM_LOG();
    /* Allocate driver data */
    gf_dev = kzalloc(sizeof(struct gf_device), GFP_KERNEL);
    if (!gf_dev) {
        status = -ENOMEM;
        goto err;
    }
    QMM_LOG();
//    spin_lock_init(&gf_dev->spi_lock);
    mutex_init(&gf_dev->buf_lock);
    mutex_init(&gf_dev->release_lock);

    INIT_LIST_HEAD(&gf_dev->device_entry);
    QMM_LOG();
    gf_dev->device_count     = 0;
    gf_dev->probe_finish     = 0;
    gf_dev->system_status    = 0;
    gf_dev->need_update      = 0;

    /*setup gf configurations.*/
    gf_debug(INFO_LOG, "%s, Setting gf device configuration==========\n", __func__);

    /* Initialize the driver data */
    gf_dev->spi = spi;

    /* setup SPI parameters */
    /* CPOL=CPHA=0, speed 1MHz */
    gf_dev->spi->mode = SPI_MODE_0;
    gf_dev->spi->bits_per_word = 8;
    gf_dev->spi->max_speed_hz = 8 * 1000 * 1000;
#ifndef CONFIG_SPI_MT65XX
    memcpy(&gf_dev->spi_mcc, &spi_ctrdata, sizeof(struct mt_chip_conf));
    gf_dev->spi->controller_data = (void *)&gf_dev->spi_mcc;
    QMM_LOG();
    spi_setup(gf_dev->spi);
#endif
    gf_dev->irq = 0;
    spi_set_drvdata(spi, gf_dev);
    QMM_LOG();
    /* allocate buffer for SPI transfer */
    gf_dev->spi_buffer = kzalloc(bufsiz, GFP_KERNEL);
    if (gf_dev->spi_buffer == NULL) {
        status = -ENOMEM;
        goto err_buf;
    }
    QMM_LOG();
    /* get gpio info from dts or defination */
    gf_get_gpio_dts_info(gf_dev);
    gf_get_sensor_dts_info();
    QMM_LOG();
    /*enable the power*/
    //gf_hw_power_enable(1);
    gf_hw_power_enable(gf_dev);
    gf_hw_reset(gf_dev, 60);
    gf_bypass_flash_gpio_cfg();
    gf_spi_clk_enable(gf_dev, 1);
#if 1 //add for test SPI communication in kernel level start
    mdelay(1);
    gf_spi_read_bytes(gf_dev, 0x0000, 4, rx_test);
    printk("%s rx_test chip id:0x%x 0x%x 0x%x 0x%x \n", __func__,rx_test[0],rx_test[1],rx_test[2],rx_test[3]);
#endif //add for test SPI communication in kernel level end
    QMM_LOG();
    /* check firmware Integrity */
    //gf_debug(INFO_LOG, "%s, Sensor type : %s.\n", __func__, CONFIG_GOODIX_SENSOR_TYPE);

    /* create class */
    gf_dev->class = class_create(THIS_MODULE, GF_CLASS_NAME);
    if (IS_ERR(gf_dev->class)) {
        gf_debug(ERR_LOG, "%s, Failed to create class.\n", __func__);
        status = -ENODEV;
        goto err_class;
    }
    QMM_LOG();
    /* get device no */
    if (GF_DEV_MAJOR > 0) {
        gf_dev->devno = MKDEV(GF_DEV_MAJOR, gf_dev->device_count++);
        status = register_chrdev_region(gf_dev->devno, 1, GF_DEV_NAME);
    } else {
        status = alloc_chrdev_region(&gf_dev->devno, gf_dev->device_count++, 1, GF_DEV_NAME);
    }
    if (status < 0) {
        gf_debug(ERR_LOG, "%s, Failed to alloc devno.\n", __func__);
        goto err_devno;
    } else {
        gf_debug(INFO_LOG, "%s, major=%d, minor=%d\n", __func__, MAJOR(gf_dev->devno), MINOR(gf_dev->devno));
    }
    QMM_LOG();
    /* create device */
    gf_dev->device = device_create(gf_dev->class, &spi->dev, gf_dev->devno, gf_dev, GF_DEV_NAME);
    if (IS_ERR(gf_dev->device)) {
        gf_debug(ERR_LOG, "%s, Failed to create device.\n", __func__);
        status = -ENODEV;
        goto err_device;
    } else {
        mutex_lock(&device_list_lock);
        list_add(&gf_dev->device_entry, &device_list);
        mutex_unlock(&device_list_lock);
        gf_debug(INFO_LOG, "%s, device create success.\n", __func__);
    }
    QMM_LOG();
    /* create sysfs */
    status = sysfs_create_group(&spi->dev.kobj, &gf_debug_attr_group);
    if (status) {
        gf_debug(ERR_LOG, "%s, Failed to create sysfs file.\n", __func__);
        status = -ENODEV;
        goto err_sysfs;
    } else {
        gf_debug(INFO_LOG, "%s, Success create sysfs file.\n", __func__);
    }
    QMM_LOG();
    /* cdev init and add */
    cdev_init(&gf_dev->cdev, &gf_fops);
    gf_dev->cdev.owner = THIS_MODULE;
    status = cdev_add(&gf_dev->cdev, gf_dev->devno, 1);
    if (status) {
        gf_debug(ERR_LOG, "%s, Failed to add cdev.\n", __func__);
        goto err_cdev;
    }


#if GF358
    gf_spi_read_word(gf_dev, GF_CHIP_ID_LO, &chip_id_1);  // care gf_dev
    gf_spi_read_word(gf_dev, GF_CHIP_ID_HI, &chip_id_2);
    g_chipID = ((chip_id_2<<16)|(chip_id_1)) >> 8;
    if(0x00002202 != g_chipID)	// atb 20161222 goodix 0x00002202
    {
        printk("Finger Print: Have not used Goodix, return\n");
        //atb close gf_release_pinctrl(gf_dev);
        status = -ENODEV;
        goto error;
    }
    pr_info("%s: chip_id_hi = 0x%x,chip_id_low = 0x%x, g_chipID is 0x%08x \n", __func__, chip_id_2, chip_id_1, g_chipID);
#endif 

    QMM_LOG();
    /*register device within input system.*/
    gf_dev->input = input_allocate_device();
    if (gf_dev->input == NULL) {
        gf_debug(ERR_LOG, "%s, Failed to allocate input device.\n", __func__);
        status = -ENOMEM;
        goto err_input;
    }
    QMM_LOG();
    __set_bit(EV_KEY, gf_dev->input->evbit);
    __set_bit(GF_KEY_INPUT_HOME, gf_dev->input->keybit);
    __set_bit(KEY_F11, gf_dev->input->keybit);

    __set_bit(GF_KEY_INPUT_MENU, gf_dev->input->keybit);
    __set_bit(GF_KEY_INPUT_BACK, gf_dev->input->keybit);
    __set_bit(GF_KEY_INPUT_POWER, gf_dev->input->keybit);

    __set_bit(GF_NAV_INPUT_UP, gf_dev->input->keybit);
    __set_bit(GF_NAV_INPUT_DOWN, gf_dev->input->keybit);
    __set_bit(GF_NAV_INPUT_RIGHT, gf_dev->input->keybit);
    __set_bit(GF_NAV_INPUT_LEFT, gf_dev->input->keybit);
    __set_bit(GF_KEY_INPUT_CAMERA, gf_dev->input->keybit);
    __set_bit(GF_NAV_INPUT_CLICK, gf_dev->input->keybit);
    __set_bit(GF_NAV_INPUT_DOUBLE_CLICK, gf_dev->input->keybit);
    __set_bit(GF_NAV_INPUT_LONG_PRESS, gf_dev->input->keybit);
    __set_bit(GF_NAV_INPUT_HEAVY, gf_dev->input->keybit);

    gf_dev->input->name = GF_INPUT_NAME;
    if (input_register_device(gf_dev->input)) {
        gf_debug(ERR_LOG, "%s, Failed to register input device.\n", __func__);
        status = -ENODEV;
        goto err_input_2;
    }
    QMM_LOG();
    gf_irq_gpio_cfg(gf_dev);
    retval = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
            IRQF_TRIGGER_RISING | IRQF_ONESHOT, "goodix_fp_irq", gf_dev);
    if (!retval)
        gf_debug(INFO_LOG, "%s irq thread request success!\n", __func__);
    else
        gf_debug(ERR_LOG, "%s irq thread request failed, retval=%d\n", __func__, retval);

    gf_dev->irq_count = 1;
    gf_disable_irq(gf_dev);
    QMM_LOG();
    /* netlink interface init */
    status = gf_netlink_init(gf_dev);
    if (status == -1) {
        mutex_lock(&gf_dev->release_lock);
        input_unregister_device(gf_dev->input);
        gf_dev->input = NULL;
        mutex_unlock(&gf_dev->release_lock);
        goto err_input;
    }
    QMM_LOG();
#if defined(CONFIG_HAS_EARLYSUSPEND)
    gf_debug(INFO_LOG, "[%s] : register_early_suspend\n", __func__);
    gf_dev->early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
    gf_dev->early_suspend.suspend = gf_early_suspend,
        gf_dev->early_suspend.resume = gf_late_resume,
        register_early_suspend(&gf_dev->early_suspend);
#else
    /* register screen on/off callback */
    gf_dev->notifier.notifier_call = gf_fb_notifier_callback;
    fb_register_client(&gf_dev->notifier);
#endif
    QMM_LOG();
    gf_dev->probe_finish = 1;
    gf_dev->is_sleep_mode = 0;

#if 0 //add for test SPI communication in kernel level start
    gf_spi_read_bytes(gf_dev, 0x0000, 4, rx_test);
    gf_debug(INFO_LOG, "%s rx_test chip id:0x%x 0x%x 0x%x 0x%x \n", __func__,rx_test[0],rx_test[1],rx_test[2],rx_test[3]);
    printk("%s rx_test chip id:0x%x 0x%x 0x%x 0x%x \n", __func__,rx_test[0],rx_test[1],rx_test[2],rx_test[3]);
#endif //add for test SPI communication in kernel level end
    gf_debug(INFO_LOG, "%s probe finished\n", __func__);
    gf_spi_clk_enable(gf_dev, 0);

#if 0 //add for test SPI communication in kernel level start
    mdelay(100);
    gf_spi_read_bytes(gf_dev, 0x0000, 4, rx_test);
    gf_debug(INFO_LOG, "%s rx_test chip id:0x%x 0x%x 0x%x 0x%x \n", __func__,rx_test[0],rx_test[1],rx_test[2],rx_test[3]);
    printk("%s rx_test chip id:0x%x 0x%x 0x%x 0x%x \n", __func__,rx_test[0],rx_test[1],rx_test[2],rx_test[3]);
#endif //add for test SPI communication in kernel level end

    QMM_LOG();
    FUNC_EXIT();
    return 0;

#if GF358
error:

#endif 

err_input_2:
    mutex_lock(&gf_dev->release_lock);
    input_free_device(gf_dev->input);
    gf_dev->input = NULL;
    mutex_unlock(&gf_dev->release_lock);

err_input:
    cdev_del(&gf_dev->cdev);

err_cdev:
    sysfs_remove_group(&spi->dev.kobj, &gf_debug_attr_group);

err_sysfs:
    device_destroy(gf_dev->class, gf_dev->devno);
    list_del(&gf_dev->device_entry);

err_device:
    unregister_chrdev_region(gf_dev->devno, 1);

err_devno:
    class_destroy(gf_dev->class);

err_class:
    //gf_hw_power_enable(0);
    gf_spi_clk_enable(gf_dev, 0);
    kfree(gf_dev->spi_buffer);
err_buf:
    mutex_destroy(&gf_dev->buf_lock);
    mutex_destroy(&gf_dev->release_lock);
    spi_set_drvdata(spi, NULL);
    gf_dev->spi = NULL;
    kfree(gf_dev);
    gf_dev = NULL;
err:

    FUNC_EXIT();
    return status;
}

static int gf_remove(struct spi_device *spi)
{
    struct gf_device *gf_dev = spi_get_drvdata(spi);

    FUNC_ENTRY();

    /* make sure ops on existing fds can abort cleanly */
    if (gf_dev->irq) {
        free_irq(gf_dev->irq, gf_dev);
        gf_dev->irq_count = 0;
        gf_dev->irq = 0;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    if (gf_dev->early_suspend.suspend)
        unregister_early_suspend(&gf_dev->early_suspend);
#else
    fb_unregister_client(&gf_dev->notifier);
#endif

    mutex_lock(&gf_dev->release_lock);
    if (gf_dev->input == NULL) {
        kfree(gf_dev);
        mutex_unlock(&gf_dev->release_lock);
        FUNC_EXIT();
        return 0;
    }
    input_unregister_device(gf_dev->input);
    gf_dev->input = NULL;
    mutex_unlock(&gf_dev->release_lock);

    mutex_lock(&gf_dev->release_lock);
    if (gf_dev->spi_buffer != NULL) {
        kfree(gf_dev->spi_buffer);
        gf_dev->spi_buffer = NULL;
    }
    mutex_unlock(&gf_dev->release_lock);

    gf_netlink_destroy(gf_dev);
    cdev_del(&gf_dev->cdev);
    sysfs_remove_group(&spi->dev.kobj, &gf_debug_attr_group);
    device_destroy(gf_dev->class, gf_dev->devno);
    list_del(&gf_dev->device_entry);

    unregister_chrdev_region(gf_dev->devno, 1);
    class_destroy(gf_dev->class);
    //gf_hw_power_enable(0);
    gf_spi_clk_enable(gf_dev, 0);

//    spin_lock_irq(&gf_dev->spi_lock);
    spi_set_drvdata(spi, NULL);
    gf_dev->spi = NULL;
//    spin_unlock_irq(&gf_dev->spi_lock);

    mutex_destroy(&gf_dev->buf_lock);
    mutex_destroy(&gf_dev->release_lock);

    kfree(gf_dev);
    FUNC_EXIT();
    return 0;
}

/*-------------------------------------------------------------------------*/
/*
static struct spi_board_info spi_board_gf3258_dev[] __initdata = {
    [0] = {
        .modalias = GF_DEV_NAME,
        .bus_num = 1,
        .chip_select = 0,
        .mode = SPI_MODE_0,
        //.controller_data = &spi_ctrdata,
    },
};*/
static struct spi_driver gf_spi_driver = {
    .driver = {
        .name = GF_DEV_NAME,
        .bus = &spi_bus_type,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = gf_of_match,
#endif
    },
    .probe = gf_probe,
    .remove = gf_remove,
};

static int __init gf_init(void)
{
    int status = 0;

    QMM_LOG();
    FUNC_ENTRY();

//    spi_register_board_info(spi_board_gf3258_dev, ARRAY_SIZE(spi_board_gf3258_dev));

    status = spi_register_driver(&gf_spi_driver);
    if (status < 0) {
        gf_debug(ERR_LOG, "%s, Failed to register SPI driver.\n", __func__);
        return -EINVAL;
    }

    QMM_LOG();
    FUNC_EXIT();
    return status;
}
//module_init(gf_init);
late_initcall(gf_init);

static void __exit gf_exit(void)
{
    FUNC_ENTRY();
    spi_unregister_driver(&gf_spi_driver);
    FUNC_EXIT();
}
module_exit(gf_exit);


MODULE_AUTHOR("goodix");
MODULE_DESCRIPTION("Goodix Fingerprint chip GF316M/GF318M/GF3118M/GF518M/GF5118M/GF516M/GF816M/GF3208/GF5206 TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf_spi");
