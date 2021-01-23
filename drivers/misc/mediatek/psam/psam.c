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

//#include <cust_gpio_usage.h>
#include <mt-plat/mtk_boot.h>
#include <mt-plat/mtk_gpio.h>
#include "psam.h"

int psam_cur_eint_state = PSAM_OUT;
extern int pinctrl_select_state(struct pinctrl *p, struct pinctrl_state *state);
static int psam_setup_eint(void);

static struct workqueue_struct * psam_eint_workqueue = NULL;
static struct work_struct psam_eint_work;
static struct switch_dev psam_data;

static unsigned int psam_power_gpio;
static unsigned int psam_reset_gpio;
static unsigned int psam_pws_gpio;
static unsigned int psam_irq_gpio;

static struct device_node *irq_node;
int psam_irq;
struct pinctrl *psamctrl;
    
void psam_eint_work_callback(struct work_struct *work)
{
	PSAM_FUNC();

    
	if(psam_cur_eint_state == PSAM_IN)
	{
		PSAM_DEBUG("PSAM_IN\n");
        gpio_set_value(psam_power_gpio,1);
        gpio_set_value(psam_reset_gpio,1);
        switch_set_state((struct switch_dev *)&psam_data, PSAM_IN);
	}
	else
	{
		PSAM_DEBUG("PSAM_OUT\n");
        gpio_set_value(psam_power_gpio,1);
        switch_set_state((struct switch_dev *)&psam_data, PSAM_OUT);
	}
    
    enable_irq(psam_irq);
}

static irqreturn_t psam_eint_func(int irq, void *desc)
{
	int ret;
	
	PSAM_FUNC();
    disable_irq_nosync(psam_irq);
    
	if(psam_cur_eint_state ==  PSAM_OUT ) 
	{
		irq_set_irq_type(psam_irq, IRQF_TRIGGER_HIGH);
		psam_cur_eint_state = PSAM_IN;
	}
	else
	{
		irq_set_irq_type(psam_irq, IRQF_TRIGGER_LOW);
		psam_cur_eint_state = PSAM_OUT;
	}

	ret = queue_work(psam_eint_workqueue, &psam_eint_work);

    return IRQ_HANDLED;
}

static int psam_setup_eint(void)
{
    u32 ints[2] = {0, 0};
    
    PSAM_FUNC();
    
    if(irq_node)
    {
		psam_irq = irq_of_parse_and_map(irq_node, 0);
		pr_err("psam_irq = %d\n", psam_irq);
		if (!psam_irq) {
			pr_err("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
      
		if (request_irq(psam_irq, psam_eint_func,psam_cur_eint_state?IRQF_TRIGGER_LOW:IRQF_TRIGGER_HIGH , "PSAM-eint", NULL)) {
			pr_err("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
              
		enable_irq(psam_irq);
	} else {
		pr_err("null irq node!!\n");
		return -EINVAL;
	}
    
	return 0;
}
static int psam_parse_dt(struct device *dev)
{
    int r = 0;
    struct device_node *np = dev->of_node;
    np = of_find_compatible_node(NULL, NULL, "mediatek,psam_driver");
    if (np) {
        r = of_get_named_gpio(np, "gpio-rst-std", 0);
        if (r < 0)
            pr_err("%s: get PSAM RST GPIO failed (%d)", __FILE__, r);
        else
            psam_reset_gpio = r;
        r = of_get_named_gpio(np, "gpio-power-std", 0);
        if (r < 0)
            pr_err("%s: get PSAM power GPIO failed (%d)", __FILE__, r);
        else
            psam_power_gpio = r;
        r = of_get_named_gpio(np, "gpio-pws-std", 0);
        if (r < 0)
            pr_err("%s: get PSAM pws  GPIO failed (%d)", __FILE__, r);
        else
            psam_pws_gpio = r;
        r = of_get_named_gpio(np, "gpio-irq-std", 0);
        if (r < 0)
            pr_err("%s: get PSAM irq GPIO failed (%d)", __FILE__, r);
        else
            psam_irq_gpio = r;
        r = 0;
    } else {
            pr_debug("%s : get gpio num err.\n", __func__);
        return -1;
    }
    pr_info(
        "[dsc]%s : get reset_gpio[%d], power_gpio[%d],pws_gpio[%d]\n",
        __func__, psam_reset_gpio, psam_power_gpio, psam_pws_gpio);
    return r;
}
static int psam_probe(struct platform_device *dev)
{
	int ret = 0;
    bool curr_state = 0;
    u32 ints[2] = {0, 0};
	
	PSAM_FUNC();

    psam_parse_dt(dev);

    psamctrl = devm_pinctrl_get(&dev->dev);    
	if (IS_ERR(psamctrl)) {
		ret = PTR_ERR(psamctrl);
		pr_err("Cannot find psam psamctrl!\n");
	}

    irq_node = of_find_compatible_node(NULL, NULL, "mediatek,psam-eint");
    if (irq_node) {
        pr_err("find irq node success!!\n");
    } else {
        pr_err("null irq node!!\n");
        return -1;
    }
    
    if (irq_node) {
        curr_state = gpio_get_value(psam_irq_gpio);
	    pr_err("%s line %d curr_state %d\n",__func__,__LINE__, curr_state);
    }
    
	if (curr_state) {
		psam_data.name = "psam";
		psam_data.index = 0;
		psam_data.state = PSAM_OUT;
		psam_cur_eint_state = PSAM_OUT;
	} else {
		psam_data.name = "psam";
		psam_data.index = 0;
		psam_data.state = PSAM_IN;
		psam_cur_eint_state = PSAM_IN;
	}

	ret = switch_dev_register(&psam_data);
	if(ret)
	{
		PSAM_DEBUG("switch_dev_register return %d\n", ret);
	}

	psam_eint_workqueue = create_singlethread_workqueue("psam_eint");
	INIT_WORK(&psam_eint_work, psam_eint_work_callback);
	psam_setup_eint();
	
    gpio_set_value(psam_power_gpio,1);
    gpio_set_value(psam_reset_gpio,1);

	return 0;
}

static int psam_remove(struct platform_device *dev)
{
	PSAM_FUNC();

	destroy_workqueue(psam_eint_workqueue);
	switch_dev_unregister(&psam_data);

	return 0;
}
static int psam_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int psam_resume(struct platform_device *pdev)
{
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id psam_of_match[] = {
    { .compatible = "mediatek,psam_driver", },
    {},
};
#endif

static struct platform_driver psam_driver = {
	.probe = psam_probe,
	.suspend = psam_suspend,
	.resume = psam_resume,
	.remove = psam_remove,
	.driver = {
        .owner	= THIS_MODULE,
		.name = "psam",
        #ifdef CONFIG_OF
        .of_match_table = psam_of_match,
        #endif
	},
};

static int psam_mod_init(void)
{

	PSAM_FUNC();
	if(platform_driver_register(&psam_driver) != 0)
	{
		PSAM_DEBUG("unable to register psam driver\n");
		return -1;
	}
	
	return 0;
}

static void psam_mod_exit(void)
{
	PSAM_FUNC();

	platform_driver_unregister(&psam_driver);
}

module_init(psam_mod_init);
module_exit(psam_mod_exit);

MODULE_DESCRIPTION("FE psam driver");
MODULE_AUTHOR("zhangqingzhan <zhangqingzhan@foeec.com>");
MODULE_LICENSE("GPL");
