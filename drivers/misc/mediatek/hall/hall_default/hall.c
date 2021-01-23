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
//#include <mach/gpio_const.h>

#include "hall.h"

//#define GPIO_HALL_EINT_PIN GPIO116	//move to dct
//#define CUST_EINT_HALL_NUM 11		//move to dct

int hall_cur_eint_state = HALL_FAR;
/*
extern void mt_eint_mask(unsigned int eint_num);                                                                                                                         
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms); 
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
*/
extern int pinctrl_select_state(struct pinctrl *p, struct pinctrl_state *state);
static int hall_setup_eint(void);

static struct workqueue_struct * hall_eint_workqueue = NULL;
static struct work_struct hall_eint_work;
static struct switch_dev hall_data;

static struct device_node *irq_node;
int hall_irq;
struct pinctrl *hallctrl;
    
void hall_eint_work_callback(struct work_struct *work)
{
	HALL_FUNC();

    
	if(hall_cur_eint_state == HALL_NEAR)
	{
		HALL_DEBUG("HALL_NEAR\n");
        switch_set_state((struct switch_dev *)&hall_data, HALL_NEAR);
	}
	else
	{
		HALL_DEBUG("HALL_FAR\n");
        switch_set_state((struct switch_dev *)&hall_data, HALL_FAR);
	}
    
    enable_irq(hall_irq);
}

static irqreturn_t hall_eint_func(int irq, void *desc)
{
	int ret;
	
	HALL_FUNC();
    disable_irq_nosync(hall_irq);
    
	if(hall_cur_eint_state ==  HALL_FAR ) 
	{
		irq_set_irq_type(hall_irq, IRQF_TRIGGER_HIGH);
		hall_cur_eint_state = HALL_NEAR;
	}
	else
	{
		irq_set_irq_type(hall_irq, IRQF_TRIGGER_LOW);
		hall_cur_eint_state = HALL_FAR;
	}

	ret = queue_work(hall_eint_workqueue, &hall_eint_work);

    return IRQ_HANDLED;
}

static int hall_setup_eint(void)
{
    u32 ints[2] = {0, 0};
    
    HALL_FUNC();
    
    if(irq_node)
    {
		of_property_read_u32_array(irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "hall");
		gpio_set_debounce(ints[0], ints[1]);
		pr_err("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		hall_irq = irq_of_parse_and_map(irq_node, 0);
		pr_err("hall_irq = %d\n", hall_irq);
		if (!hall_irq) {
			pr_err("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
      
		if (request_irq(hall_irq, hall_eint_func,hall_cur_eint_state?IRQF_TRIGGER_LOW:IRQF_TRIGGER_HIGH , "HALL-eint", NULL)) {
			pr_err("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
              
		enable_irq(hall_irq);
	} else {
		pr_err("null irq node!!\n");
		return -EINVAL;
	}
    
	return 0;
}

static int hall_probe(struct platform_device *dev)
{
	int ret = 0;
    bool curr_state = 0;
    u32 ints[2] = {0, 0};
	
	HALL_FUNC();

/* Stoneoim:yangbinbin on: Mon, 24 Oct 2016 16:22:41 +0800
 * TODO: replace this line with your comment
	mt_set_gpio_mode(GPIO_HALL_1_PIN, 0);
	mt_set_gpio_dir(GPIO_HALL_1_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_1_PIN, GPIO_PULL_DISABLE);
	curr_state = mt_get_gpio_in(GPIO_HALL_1_PIN);
	printk("%s line %d curr_state %d\n",__func__,__LINE__, curr_state);
 */
// End of Stoneoim: yangbinbin

    hallctrl = devm_pinctrl_get(&dev->dev);    
	if (IS_ERR(hallctrl)) {
		ret = PTR_ERR(hallctrl);
		pr_err("Cannot find hall hallctrl!\n");
	}

    irq_node = of_find_compatible_node(NULL, NULL, "mediatek, hall-eint");
    if (irq_node) {
        pr_err("find irq node success!!\n");
    } else {
        pr_err("null irq node!!\n");
        return -1;
    }
    
    if (irq_node) {
		of_property_read_u32_array(irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "hall");
        curr_state = gpio_get_value(ints[0]);
	    pr_err("%s line %d curr_state %d\n",__func__,__LINE__, curr_state);
    }
    
	if (curr_state) {
		hall_data.name = "hall";
		hall_data.index = 0;
		hall_data.state = HALL_FAR;
		hall_cur_eint_state = HALL_FAR;
	} else {
		hall_data.name = "hall";
		hall_data.index = 0;
		hall_data.state = HALL_NEAR;
		hall_cur_eint_state = HALL_NEAR;
	}

	ret = switch_dev_register(&hall_data);
	if(ret)
	{
		HALL_DEBUG("switch_dev_register return %d\n", ret);
	}

	hall_eint_workqueue = create_singlethread_workqueue("hall_eint");
	INIT_WORK(&hall_eint_work, hall_eint_work_callback);

	hall_setup_eint();
	
	return 0;
}

static int hall_remove(struct platform_device *dev)
{
	HALL_FUNC();

	destroy_workqueue(hall_eint_workqueue);
	switch_dev_unregister(&hall_data);

	return 0;
}
/*
struct platform_device hall_device = {
  .name = "hall_driver",
  .id = -1,
};
*/
static int hall_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int hall_resume(struct platform_device *pdev)
{
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hall_of_match[] = {
    { .compatible = "mediatek, hall_driver", },
    {},
};
#endif

static struct platform_driver hall_driver = {
	.probe = hall_probe,
	.suspend = hall_suspend,
	.resume = hall_resume,
	.remove = hall_remove,
	.driver = {
        .owner	= THIS_MODULE,
		.name = "hall",
        #ifdef CONFIG_OF
        .of_match_table = hall_of_match,
        #endif
	},
};

static int hall_mod_init(void)
{

	HALL_FUNC();
/*    
    retval = platform_device_register(&hall_device);
    printk("register hall device\n");

    if(retval != 0)
    {
      printk("platform_device_register hall error:(%d)\n", retval);
    }
    else
    {
      printk("platform_device_register hall done!\n");
    }
*/	
	if(platform_driver_register(&hall_driver) != 0)
	{
		HALL_DEBUG("unable to register hall driver\n");
		return -1;
	}
	
	return 0;
}

static void hall_mod_exit(void)
{
	HALL_FUNC();

	platform_driver_unregister(&hall_driver);
}

module_init(hall_mod_init);
module_exit(hall_mod_exit);

MODULE_DESCRIPTION("Stoneoim Hall driver");
MODULE_AUTHOR("AL <lubaoquan@vanzotec.com>");
MODULE_LICENSE("GPL");
