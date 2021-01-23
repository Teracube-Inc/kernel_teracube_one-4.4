#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/compat.h>
#include <linux/platform_device.h>



struct pinctrl *fpc_finger_pinctrl;
struct pinctrl_state *fpc_finger_reset_high,*fpc_finger_reset_low;

int fpc_finger_get_gpio_info(struct platform_device *pdev)
{
    int ret;

    fpc_finger_pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(fpc_finger_pinctrl)) {
        ret = PTR_ERR(fpc_finger_pinctrl);
        dev_err(&pdev->dev, "fpc_finger cannot find pinctrl\n");
        return ret;
    }
    pr_debug("[%s] fpc_finger_pinctrl+++++++++++++++++\n",pdev->name);


    fpc_finger_reset_high = pinctrl_lookup_state(fpc_finger_pinctrl, "fp_rst_high");
    if (IS_ERR(fpc_finger_reset_high)) {
        ret = PTR_ERR(fpc_finger_reset_high);
        dev_err(&pdev->dev, " Cannot find fpc_finger pinctrl fpc_finger_reset_high!\n");
        return ret;
    }
    fpc_finger_reset_low = pinctrl_lookup_state(fpc_finger_pinctrl, "fp_rst_low");
    if (IS_ERR(fpc_finger_reset_low)) {
        ret = PTR_ERR(fpc_finger_reset_low);
        dev_err(&pdev->dev, " Cannot find fpc_finger pinctrl fpc_finger_reset_low!\n");
        return ret;
    }
    pr_debug("fpc_finger get gpio info ok--------");
    return 0;
}

int fpc_finger_set_power(int cmd)
{
	switch (cmd)
		{
		case 0 : 		
		break;
		case 1 : 		
		break;
		}
	return 0;
}

int fpc_finger_set_reset(int cmd)
{
	switch (cmd)
		{
		case 0 : 		
			pinctrl_select_state(fpc_finger_pinctrl, fpc_finger_reset_low);
		break;
		case 1 : 		
			pinctrl_select_state(fpc_finger_pinctrl, fpc_finger_reset_high);
		break;
		}
	return 0;
}

int fpc_finger_set_eint(int cmd)
{
	switch (cmd)
		{
		case 0 : 		
			return -1;
		break;
		case 1 : 		
		break;
		}
	return 0;
}

