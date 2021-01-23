/*
 * pt_i2c.c
 * Parade TrueTouch(TM) Standard Product I2C Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * TMA5XX
 * TMA448
 * TMA445A
 * TT21XXX
 * TT31XXX
 * TT4XXXX
 * TT7XXX
 * TC3XXX
 *
 * Copyright (C) 2015-2018 Parade Technologies
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Parade Technologies at www.paradetech.com <ttdrivers@paradetech.com>
 */

#include "pt_regs.h"

#include <linux/i2c.h>
#include <linux/version.h>

#define PT_I2C_DATA_SIZE  (2 * 256)
#define _base(x) ((x >= '0' && x <= '9') ? '0' : \
	(x >= 'a' && x <= 'f') ? 'a' - 10 : \
	(x >= 'A' && x <= 'F') ? 'A' - 10 : \
	'\255')
#define HEXOF(x) (x - _base(x))

#ifdef TTDL_DIAGNOSTICS
static unsigned char pt_dut_cmd_buf[300];
static unsigned char pt_dut_out_buf[300];
static int pt_dut_cmd_len;
static int pt_dut_out_len;

/*******************************************************************************
 * FUNCTION: virt_i2c_transfer
 *
 * SUMMARY: Copies the current i2c output message to the temporary buffer
 *	used by the dut_cmd sysfs node
 *
 * RETURN VALUE:
 *	Number of messages transfered which in this function will be 1
 *
 * PARAMETERS:
 *      *buf - pointer to i2c command
 *	 len - length of command in the buffer
 ******************************************************************************/
static int virt_i2c_transfer(u8 *buf, int len)
{
	if (len <= sizeof(pt_dut_cmd_buf)) {
		memcpy(pt_dut_cmd_buf, buf, len);
		pt_dut_cmd_len = len;
		return 1;
	} else
		return 0;
}

/*******************************************************************************
 * FUNCTION: virt_i2c_master_recv
 *
 * SUMMARY: Copies the i2c input message from the dut_out sysfs node into a
 *	temporary buffer. If the entire message is read then clear the buffer
 *	length varible to indicate the contents have been consumed. TTDL will
 *	perform a double read on a buffer, first reading the 2 byte length
 *	followed by reading the entire packet based on that length.
 *
 * RETURN VALUE:
 *	Length of data transfered
 *
 * PARAMETERS:
 *	*dev - pointer to device struct
 *      *buf - pointer to i2c incoming report
 ******************************************************************************/
static int virt_i2c_master_recv(struct device *dev, u8 *buf)
{
	int ret = pt_dut_out_len;

	pt_debug(dev, DL_INFO,
		"%s: Copy msg from dut_out to i2c buffer, len=%d\n",
		__func__, pt_dut_out_len);

	memcpy(buf, pt_dut_out_buf, pt_dut_out_len + 1);

	/* After copying msg into buf clear length ready for next msg */
	pt_dut_out_len = 0;

	/* Return original pt_dut_out_len */
	return ret;
}

/*******************************************************************************
 * FUNCTION: pt_dut_cmd_show
 *
 * SUMMARY: The show function for the dut_cmd sysfs node. Provides read access
 *	to the pt_dut_cmd_buf and clears it after it has been read.
 *
 * RETURN VALUE:
 *	Number of bytes transfered
 *
 * PARAMETERS:
 *      *dev  - pointer to device structure
 *      *attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_dut_cmd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	int index = 0;

	/* Only print to sysfs if the buffer has data */
	if (pt_dut_cmd_len > 0) {
		for (i = 0; i < pt_dut_cmd_len; i++)
			index += sprintf(buf + index, "%02X",
				pt_dut_cmd_buf[i]);
		index += sprintf(buf + index, "\n");
	}
	pt_dut_cmd_len = 0;
	return index;
}
static DEVICE_ATTR(dut_cmd, S_IRUGO, pt_dut_cmd_show, NULL);

/*******************************************************************************
 * FUNCTION: pt_dut_out_store
 *
 * SUMMARY: The store function for the dut_out sysfs node. Provides write
 *	access to the pt_dut_out_buf. The smallest valid PIP response is 2
 *	bytes so don't update buffer if only 1 byte passed in.
 *
 * RETURN VALUE:
 *	Number of bytes read from virtual DUT
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to buffer that hold the command parameters
 *	 size - size of buf
 ******************************************************************************/
static ssize_t pt_dut_out_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int loop_max = ARRAY_SIZE(pt_dut_out_buf);
	int hex_str_len = strlen(buf)/2;
	int i;
	const char *pos = buf;

	/* Only update the dut_out buffer if at least 2 byte payload */
	pt_debug(dev, DL_INFO,
		"%s: Virtual DUT Response written to TTDL, len = %d\n",
		__func__, hex_str_len);

	if (size >= 2 && hex_str_len <= loop_max) {
		/* Convert string of hex values to byte array */
		for (i = 0; i < hex_str_len; i++) {
			pt_dut_out_buf[i] = ((HEXOF(*pos)) << 4) +
					     HEXOF(*(pos + 1));
			pos += 2;
		}
		pt_dut_out_len = hex_str_len;
	}
	return size;
}
static DEVICE_ATTR(dut_out, S_IWUSR, NULL, pt_dut_out_store);
#endif  /* TTDL_DIAGNOSTICS */

/*******************************************************************************
 * FUNCTION: pt_i2c_read_default
 *
 * SUMMARY: Read a certain number of bytes from the I2C bus
 *
 * PARAMETERS:
 *      *dev  - pointer to Device structure
 *      *buf  - pointer to buffer where the data read will be stored
 *       size - size to be read
 ******************************************************************************/
static int pt_i2c_read_default(struct device *dev, void *buf, int size)
{
#ifdef TTDL_DIAGNOSTICS
	struct pt_core_data *cd = dev_get_drvdata(dev);
#endif
	struct i2c_client *client = to_i2c_client(dev);
	int rc;

	if (!buf || !size || size > PT_I2C_DATA_SIZE)
		return -EINVAL;

#ifdef TTDL_DIAGNOSTICS
	if (cd->route_i2c_virt_dut) {
		size = pt_dut_out_len;
		rc = virt_i2c_master_recv(dev, buf);
	} else
		rc = i2c_master_recv(client, buf, size);
#else
	rc = i2c_master_recv(client, buf, size);
#endif

	return (rc < 0) ? rc : rc != size ? -EIO : 0;
}

/*******************************************************************************
 * FUNCTION: pt_i2c_read_default_nosize
 *
 * SUMMARY: Read from the I2C bus in two transactions first reading the HID
 *	packet size (2 bytes) followed by reading the rest of the packet based
 *	on the size initially read.
 *	NOTE: The empty buffer 'size' was redefined in PIP version 1.7.
 *
 * PARAMETERS:
 *      *dev  - pointer to Device structure
 *      *buf  - pointer to buffer where the data read will be stored
 *       max  - max size that can be read
 ******************************************************************************/
static int pt_i2c_read_default_nosize(struct device *dev, u8 *buf, u32 max)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;
	u32 size;
#ifdef TTDL_DIAGNOSTICS
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (cd->route_i2c_virt_dut) {
		size = pt_dut_out_len;
		goto skip_read_len;
	}
#endif

	if (!buf)
		return -EINVAL;

	msgs[0].addr = client->addr;
	msgs[0].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
	msgs[0].len = 2;
	msgs[0].buf = buf;
	rc = i2c_transfer(client->adapter, msgs, msg_count);
	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;

	size = get_unaligned_le16(&buf[0]);
	if (!size || size == 2 || size >= PT_PIP_1P7_EMPTY_BUF)
		/*
		 * Before PIP 1.7, empty buffer is 0x0002;
		 * From PIP 1.7, empty buffer is 0xFFXX
		 */
		return 0;

	if (size > max)
		return -EINVAL;

#ifdef TTDL_DIAGNOSTICS
skip_read_len:
	if (cd->route_i2c_virt_dut)
		rc = virt_i2c_master_recv(dev, buf);
	else
		rc = i2c_master_recv(client, buf, size);
	pt_debug(dev, DL_DEBUG, "%s: rc = %d\n", __func__, rc);
#else
	rc = i2c_master_recv(client, buf, size);
#endif
	return (rc < 0) ? rc : rc != (int)size ? -EIO : 0;
}

/*******************************************************************************
 * FUNCTION: pt_i2c_write_read_specific
 *
 * SUMMARY: Write the contents of write_buf to the I2C device and then read
 *	the response using pt_i2c_read_default_nosize()
 *
 * PARAMETERS:
 *      *dev       - pointer to Device structure
 *       write_len - length of data buffer write_buf
 *      *write_buf - pointer to buffer to write
 *      *read_buf  - pointer to buffer to read response into
 ******************************************************************************/
static int pt_i2c_write_read_specific(struct device *dev, u8 write_len,
		u8 *write_buf, u8 *read_buf)
{
	struct i2c_client *client = to_i2c_client(dev);
#ifdef TTDL_DIAGNOSTICS
	struct pt_core_data *cd = dev_get_drvdata(dev);
#endif
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;

	if (!write_buf || !write_len) {
		if (!write_buf)
			pt_debug(dev, DL_ERROR,
				"%s write_buf is NULL", __func__);
		if (!write_len)
			pt_debug(dev, DL_ERROR,
				"%s write_len is NULL", __func__);
		return -EINVAL;

	}

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = write_len;
	msgs[0].buf = write_buf;
#ifdef TTDL_DIAGNOSTICS
	if (cd->route_i2c_virt_dut) {
		rc = virt_i2c_transfer(msgs[0].buf, msgs[0].len);
		pt_debug(dev, DL_DEBUG, "%s: Virt transfer size = %d",
			__func__, msgs[0].len);
	} else
		rc = i2c_transfer(client->adapter, msgs, msg_count);
#else
	rc = i2c_transfer(client->adapter, msgs, msg_count);
#endif

	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;

	rc = 0;

	if (read_buf) {
		rc = pt_i2c_read_default_nosize(dev, read_buf,
				PT_I2C_DATA_SIZE);
	}

	return rc;
}

static struct pt_bus_ops pt_i2c_bus_ops = {
	.bustype = BUS_I2C,
	.read_default = pt_i2c_read_default,
	.read_default_nosize = pt_i2c_read_default_nosize,
	.write_read_specific = pt_i2c_write_read_specific,
};

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
static const struct of_device_id pt_i2c_of_match[] = {
	{ .compatible = "parade,pt_i2c_adapter", },
	{ }
};
MODULE_DEVICE_TABLE(of, pt_i2c_of_match);
#endif


/*******************************************************************************
 * FUNCTION: pt_i2c_probe
 *
 * SUMMARY: Probe functon for the I2C module
 *
 * PARAMETERS:
 *      *client - pointer to i2c client structure
 *      *i2c_id - pointer to i2c device structure
 ******************************************************************************/
static int pt_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *i2c_id)
{
	struct device *dev = &client->dev;
#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	const struct of_device_id *match;
#endif
	int rc;

	rc = regulator_enable(tpd->reg);
	if (rc != 0)
		PD_DBG("Failed to enable reg-vgp6: %d\n", rc);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pt_debug(dev, DL_ERROR, "I2C functionality not Supported\n");
		return -EIO;
	}

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(pt_i2c_of_match), dev);
	if (match) {
		rc = pt_devtree_create_and_get_pdata(dev);
		if (rc < 0)
			return rc;
	}
#endif

	rc = pt_probe(&pt_i2c_bus_ops, &client->dev, client->irq,
			  PT_I2C_DATA_SIZE);

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	if (rc && match)
		pt_devtree_clean_pdata(dev);
#endif
#ifdef TTDL_DIAGNOSTICS
	device_create_file(dev, &dev_attr_dut_cmd);
	device_create_file(dev, &dev_attr_dut_out);
#endif

    if(rc == 0)
      tpd_load_status = 1;

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_i2c_remove
 *
 * SUMMARY: Remove functon for the I2C module
 *
 * PARAMETERS:
 *      *client - pointer to i2c client structure
 ******************************************************************************/
static int pt_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	const struct of_device_id *match;
#ifndef TTDL_DIAGNOSTICS
struct device *dev = &client->dev;
#endif
#endif
	struct pt_core_data *cd = i2c_get_clientdata(client);

#ifdef TTDL_DIAGNOSTICS
	struct device *dev = &client->dev;

	device_remove_file(dev, &dev_attr_dut_cmd);
	device_remove_file(dev, &dev_attr_dut_out);
#endif
	pt_release(cd);

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(pt_i2c_of_match), dev);
	if (match)
		pt_devtree_clean_pdata(dev);
#endif

	return 0;
}

static const struct i2c_device_id pt_i2c_id[] = {
	{ PT_I2C_NAME, 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pt_i2c_id);

static struct i2c_driver pt_i2c_driver = {
	.driver = {
		.name  = PT_I2C_NAME,
		.owner = THIS_MODULE,
		//.pm    = &pt_pm_ops,
#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
		.of_match_table = pt_i2c_of_match,
#endif
	},
	.probe    = pt_i2c_probe,
	.remove   = pt_i2c_remove,
	.id_table = pt_i2c_id,
};

/*******************************************************************************
 * FUNCTION: pt_i2c_init
 *
 * SUMMARY: Initialize function to register i2c module to kernel.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 ******************************************************************************/

static int parade_common_local_init(void)
{
  int retval;

  PD_DBG("Parade I2C Touchscreen Driver local init\n");

  tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
  retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);

  if (retval != 0)
  {
    PD_DBG("Failed to set voltage 2V8: %d\n", retval);
  }

  if (i2c_add_driver(&pt_i2c_driver) != 0)
  {
    PD_DBG("unable to add i2c driver.\n");
    return -EFAULT;
  }

  if (tpd_load_status == 0)
  {
    i2c_del_driver(&pt_i2c_driver);
    return -1;
  }

#if defined(PD_PLATFOME_DEFINE_KEY)
  if (tpd_dts_data.use_tpd_button)
  {
    PD_DBG("tpd_dts_data.use_tpd_button %d\n", tpd_dts_data.use_tpd_button);
    tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local, tpd_dts_data.tpd_key_dim_local);
  }
#endif

  PD_DBG("end %s, %d\n", __FUNCTION__, __LINE__);

  return 0;
}

static void parade_common_suspend(struct device *dev)
{
  return ;
}

static void parade_common_resume(struct device *dev)
{
  return ;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name    = "parade",
	.tpd_local_init     = parade_common_local_init,
	.suspend            = parade_common_suspend,
	.resume             = parade_common_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button    = 1,
#else
	.tpd_have_button    = 0,
#endif
};

static int __init parade_common_init(void)
{
  int ret = 0;
  PD_DBG("parade_common_init");
  tpd_get_dts_info();

  ret = tpd_driver_add(&tpd_device_driver);
  if(ret < 0)
    PD_DBG("Failed to add Driver!\n");
  return 0;
}

static void __exit parade_common_exit(void)
{
	tpd_driver_remove(&tpd_device_driver);
}

module_init(parade_common_init);
module_exit(parade_common_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parade TrueTouch(R) Standard Product I2C driver");
MODULE_AUTHOR("Parade Technologies <ttdrivers@paradetech.com>");
