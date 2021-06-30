/*
 * Copyright (C) 2017 MediaTek Inc.
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


#include <linux/module.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>

#include <linux/phy/mediatek/mtk_usb_phy.h>
#include <phy-mtk.h>

#define DRIVER_NAME "mtk_phy"


u32 phy_debug_level = K_ERR;

module_param(phy_debug_level, int, 0644);
MODULE_PARM_DESC(phy_debug_level, "Debug Print Log for mtkphy");

int usb_mtkphy_switch_to_usb(struct phy *phy)
{
	int ret = 0;

	if (phy) {
		struct mtk_phy_instance *instance = phy_get_drvdata(phy);

		if (instance->phycfg && instance->phycfg->usb_phy_switch_to_usb)
			instance->phycfg->usb_phy_switch_to_usb(instance);
		else
			ret = -ENOTSUPP;
	} else
		ret = -EINVAL;

	return ret;
}
EXPORT_SYMBOL_GPL(usb_mtkphy_switch_to_usb);

int usb_mtkphy_switch_to_uart(struct phy *phy)
{
	int ret = 0;

	if (phy) {
		struct mtk_phy_instance *instance = phy_get_drvdata(phy);

		if (instance->phycfg && instance->phycfg->usb_phy_switch_to_uart)
			instance->phycfg->usb_phy_switch_to_uart(instance);
		else
			ret = -ENOTSUPP;
	} else
		ret = -EINVAL;

	return ret;
}
EXPORT_SYMBOL_GPL(usb_mtkphy_switch_to_uart);

int usb_mtkphy_check_in_uart_mode(struct phy *phy)
{
	int ret = 0;

	if (phy) {
		struct mtk_phy_instance *instance = phy_get_drvdata(phy);

		if (instance->phycfg && instance->phycfg->usb_phy_check_in_uart_mode)
			ret = instance->phycfg->usb_phy_check_in_uart_mode(instance);
		else
			ret = -ENOTSUPP;
	} else
		ret = -EINVAL;

	return ret;
}
EXPORT_SYMBOL_GPL(usb_mtkphy_check_in_uart_mode);

int usb_mtkphy_sib_enable_switch(struct phy *phy, bool enable)
{
	int ret = 0;

	if (phy) {
		struct mtk_phy_instance *instance = phy_get_drvdata(phy);

		if (instance->phycfg && instance->phycfg->usb_phy_sib_enable_switch)
			instance->phycfg->usb_phy_sib_enable_switch(instance, enable);
		else
			ret = -ENOTSUPP;
	} else
		ret = -EINVAL;

	return ret;
}
EXPORT_SYMBOL_GPL(usb_mtkphy_sib_enable_switch);

int usb_mtkphy_u3_loop_back_test(struct phy *phy)
{
	int ret;

	if (phy) {
		struct mtk_phy_instance *instance = phy_get_drvdata(phy);

		if (instance->phycfg && instance->phycfg->usb_phy_u3_loop_back_test)
			ret = instance->phycfg->usb_phy_u3_loop_back_test(instance);
		else
			ret = -ENOTSUPP;
	} else
		ret = -EINVAL;

	return ret;
}
EXPORT_SYMBOL_GPL(usb_mtkphy_u3_loop_back_test);

int usb_mtkphy_slew_rate_calibration(struct phy *phy)
{
	int ret = 0;

	return ret;
}
EXPORT_SYMBOL_GPL(usb_mtkphy_slew_rate_calibration);

int usb_mtkphy_switch_to_bc11(struct phy *phy, bool on)
{
	int ret = 0;

	if (phy) {
		struct mtk_phy_instance *instance = phy_get_drvdata(phy);

		if (instance->phycfg && instance->phycfg->usb_phy_switch_to_bc11)
			instance->phycfg->usb_phy_switch_to_bc11(instance, on);
		else
			ret = -ENOTSUPP;
	} else
		ret = -EINVAL;

	return ret;
}
EXPORT_SYMBOL_GPL(usb_mtkphy_switch_to_bc11);

int usb_mtkphy_lpm_enable(struct phy *phy, bool on)
{
	int ret = 0;

	if (phy) {
		struct mtk_phy_instance *instance = phy_get_drvdata(phy);

		if (instance->phycfg && instance->phycfg->usb_phy_lpm_enable)
			ret = instance->phycfg->usb_phy_lpm_enable(instance, on);
		else
			ret = -ENOTSUPP;
	} else
		ret = -EINVAL;

	return ret;
}
EXPORT_SYMBOL_GPL(usb_mtkphy_lpm_enable);

int usb_mtkphy_host_mode(struct phy *phy, bool on)
{
	int ret = 0;

	if (phy) {
		struct mtk_phy_instance *instance = phy_get_drvdata(phy);

		if (instance->phycfg && instance->phycfg->usb_phy_host_mode)
			ret = instance->phycfg->usb_phy_host_mode(instance, on);
		else
			ret = -ENOTSUPP;
	} else
		ret = -EINVAL;

	return ret;
}
EXPORT_SYMBOL_GPL(usb_mtkphy_host_mode);

int usb_mtkphy_io_read(struct phy *phy, int mode, u32 reg)
{
	int ret = 0;

	if (phy) {
		struct mtk_phy_instance *instance = phy_get_drvdata(phy);

		if (instance->phycfg && instance->phycfg->usb_phy_io_read)
			ret = instance->phycfg->usb_phy_io_read(instance, mode, reg);
		else
			ret = -ENOTSUPP;
	} else
		ret = -EINVAL;

	return ret;
}
EXPORT_SYMBOL_GPL(usb_mtkphy_io_read);

int usb_mtkphy_io_write(struct phy *phy, int mode, u32 val, u32 reg)
{
	int ret = 0;

	if (phy) {
		struct mtk_phy_instance *instance = phy_get_drvdata(phy);

		if (instance->phycfg && instance->phycfg->usb_phy_io_write)
			instance->phycfg->usb_phy_io_write(instance, mode, val, reg);
		else
			ret = -ENOTSUPP;
	} else
		ret = -EINVAL;

	return ret;
}
EXPORT_SYMBOL_GPL(usb_mtkphy_io_write);


static int mtk_phy_init(struct phy *phy)
{
	struct mtk_phy_instance *instance = phy_get_drvdata(phy);

	instance->phycfg->usb_phy_init(instance);

	return 0;
}


static int mtk_phy_power_on(struct phy *phy)
{
	struct mtk_phy_instance *instance = phy_get_drvdata(phy);

	instance->phycfg->usb_phy_recover(instance);
	return 0;
}

static int mtk_phy_power_off(struct phy *phy)
{
	struct mtk_phy_instance *instance = phy_get_drvdata(phy);

	instance->phycfg->usb_phy_savecurrent(instance);
	return 0;
}


static const struct of_device_id mtk_phy_of_match[] = {
#ifdef CONFIG_PHY_MTK_SSUSB
	{
		.compatible = "mediatek,mt6758-phy",
		.data = &ssusb_phy_config,
	},
#endif
	{ },
};

static struct phy_ops mtk_u3phy_ops = {
	.init		= mtk_phy_init,
	.exit		= NULL,
	.power_on	= mtk_phy_power_on,
	.power_off	= mtk_phy_power_off,
	.owner		= THIS_MODULE,
};

static struct phy *mtk_phy_xlate(struct device *dev,
					struct of_phandle_args *args)
{
	struct mtk_phy_drv *drv;

	drv = dev_get_drvdata(dev);
	if (!drv)
		return ERR_PTR(-EINVAL);

	if (WARN_ON(args->args[0] >= drv->phycfg->num_phys))
		return ERR_PTR(-ENODEV);

	return drv->phys[args->args[0]]->phy;
}

static int mtk_usb_phy_probe(struct platform_device *pdev)
{
	int retval, i;
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	const struct mtk_usbphy_config *mtkcfg;
	struct device_node *np = pdev->dev.of_node;
	struct mtk_phy_drv *mtkphy;
	struct phy_provider *provider;

	if (!pdev->dev.of_node) {
		dev_err(dev, "This driver is required to be instantiated from device tree\n");
		return -EINVAL;
	}

	match = of_match_node(mtk_phy_of_match, pdev->dev.of_node);
	if (!match) {
		dev_err(dev, "of_match_node() failed\n");
		return -EINVAL;
	}

	mtkcfg = match->data;
	mtkphy = devm_kzalloc(dev, sizeof(*mtkphy), GFP_KERNEL);
	if (!mtkphy)
		return -ENOMEM;

	mtkphy->nphys = mtkcfg->num_phys;
	mtkphy->phys = devm_kcalloc(dev, mtkphy->nphys,
				       sizeof(*mtkphy->phys), GFP_KERNEL);

	mtkphy->phycfg = mtkcfg;
	mtkphy->dev = dev;

	platform_set_drvdata(pdev, mtkphy);

	/* init phy driver */
	retval = mtkcfg->usb_drv_init(pdev, mtkphy);
	if (retval)
		return retval;

	for (i = 0; i < mtkphy->phycfg->num_phys; i++) {
		struct mtk_phy_instance *instance = mtkphy->phys[i];
		char *name = mtkphy->phycfg->phys[i].name;
		struct phy *phy;

		instance = devm_kzalloc(dev, sizeof(*instance), GFP_KERNEL);
		if (!instance) {
			retval = -ENOMEM;
						goto drv_exit;
		}
		instance->port_base = mtkphy->phy_base + mtkphy->phycfg->phys[i].reg_offset;
		phy = devm_phy_create(dev, np, &mtk_u3phy_ops);
		if (IS_ERR(phy)) {
			dev_err(dev, "Failed to create usb2_phy \"%s\"\n",
				name);
			retval = PTR_ERR(phy);
			goto drv_exit;
		}
		instance->phy = phy;
		instance->phycfg = &mtkphy->phycfg->phys[i];
		instance->phy_drv = mtkphy;
		mtkphy->phys[i] = instance;
		phy_set_drvdata(instance->phy, instance);
	}

	provider = devm_of_phy_provider_register(dev,
						mtk_phy_xlate);

	return PTR_ERR_OR_ZERO(provider);

drv_exit:
	mtkcfg->usb_drv_exit(pdev, mtkphy);
	return retval;
}

static int mtk_usb_phy_remove(struct platform_device *pdev)
{

	return 0;
}


static struct platform_driver mtk_usb_phy_driver = {
	.probe = mtk_usb_phy_probe,
	.remove = mtk_usb_phy_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = mtk_phy_of_match,
	},
};


static int __init usb_phy_mtk_init(void)
{
	return platform_driver_register(&mtk_usb_phy_driver);
}
module_init(usb_phy_mtk_init);

static void __exit usb_phy_mtk_exit(void)
{
	platform_driver_unregister(&mtk_usb_phy_driver);
}
module_exit(usb_phy_mtk_exit);



MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Mediatek USB transceiver driver");
