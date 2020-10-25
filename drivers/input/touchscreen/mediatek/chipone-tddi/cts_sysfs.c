#define LOG_TAG         "Sysfs"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_test.h"
#include "cts_sfctrl.h"
#include "cts_spi_flash.h"
#include "cts_firmware.h"

#ifdef CONFIG_CTS_SYSFS
#define MAX_ARG_NUM                 (100)
#define MAX_ARG_LENGTH              (1024)
static char cmdline_param[MAX_ARG_LENGTH + 1];
static int  argc;
static char *argv[MAX_ARG_NUM];

static int parse_arg(const char *buf, size_t count)
{
    char *p;

    memcpy(cmdline_param, buf, min((size_t)MAX_ARG_LENGTH, count));
    cmdline_param[count] = '\0';

    argc = 0;
    p = strim(cmdline_param);
    if (p == NULL || p[0] == '\0') {
        return 0;
    }

    while (p && p[0] != '\0' && argc < MAX_ARG_NUM) {
        argv[argc++] = strsep(&p, " ,");
    }

    return argc;
}

/* echo addr value1 value2 value3 ... valueN > write_reg */
static ssize_t write_firmware_register_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 addr;
    int i, ret;
    u8 *data = NULL;

    parse_arg(buf, count);

    cts_info("Write firmware register '%.*s'", (int)count, buf);

    if (argc < 2) {
        cts_err("Too few args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou16(argv[0], 0, &addr);
    if (ret) {
        cts_err("Invalid address %s", argv[0]);
        return -EINVAL;
    }

    data = (u8 *)kmalloc(argc - 1, GFP_KERNEL);
    if (data == NULL) {
        cts_err("Allocate buffer for write data failed\n");
        return -ENOMEM;
    }

    for (i = 1; i < argc; i++) {
        ret = kstrtou8(argv[i], 0, data + i - 1);
        if (ret) {
            cts_err("Invalid value %s", argv[i]);
            goto free_data;
        }
    }

    ret = cts_fw_reg_writesb(cts_dev, addr, data, argc - 1);
    if (ret) {
        cts_err("Write firmware register addr: 0x%04x size: %d failed",
            addr, argc - 1);
        goto free_data;
    }

free_data:
    kfree(data);

	return (ret < 0 ? ret : count);
}
static DEVICE_ATTR(write_reg, S_IWUSR, NULL, write_firmware_register_store);

static ssize_t read_firmware_register_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#define PRINT_ROW_SIZE          (16)
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 addr, size, i, remaining;
    u8 *data = NULL;
    ssize_t count = 0;
    int ret;

    cts_info("Read firmware register '%.*s'", (int)count, buf);

    if (argc != 2) {
        return sprintf(buf,
            "Invalid num args %d\n"
            "  1. echo addr size > read_reg\n"
            "  2. cat read_reg\n", argc);
    }

    ret = kstrtou16(argv[0], 0, &addr);
    if (ret) {
        return sprintf(buf, "Invalid address: %s\n", argv[0]);
    }
    ret = kstrtou16(argv[1], 0, &size);
    if (ret) {
        return sprintf(buf, "Invalid size: %s\n", argv[1]);
    }

    data = (u8 *)kmalloc(size, GFP_KERNEL);
    if (data == NULL) {
        return sprintf(buf, "Allocate buffer for read data failed\n");
    }

    cts_info("Read firmware register from 0x%04x size %u", addr, size);

    ret = cts_fw_reg_readsb(cts_dev, addr, data, (size_t)size);
    if (ret) {
        count = sprintf(buf,
            "Read firmware register from 0x%04x size %u failed %d\n",
            addr, size, ret);
        goto err_free_data;
    }

    remaining = size;
	for (i = 0; i < size && count <= PAGE_SIZE; i += PRINT_ROW_SIZE) {
        size_t linelen = min((size_t)remaining, (size_t)PRINT_ROW_SIZE);
        remaining -= PRINT_ROW_SIZE;

        count += snprintf(buf + count, PAGE_SIZE - count, "%04x: ", addr);

        /* Lower version kernel return void */
        hex_dump_to_buffer(data + i, linelen, PRINT_ROW_SIZE, 1,
                    buf + count, PAGE_SIZE - count, true);
        count += strlen(buf + count);

        if (count < PAGE_SIZE) {
            buf[count++] = '\n';
            addr += PRINT_ROW_SIZE;
        } else {
            break;
        }
    }

err_free_data:
    kfree(data);

	return count;
#undef PRINT_ROW_SIZE
}

/* echo addr size > read_reg */
static ssize_t read_firmware_register_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    parse_arg(buf, count);

	return (argc == 0 ? 0 : count);
}
static DEVICE_ATTR(read_reg, S_IWUSR | S_IRUSR,
    read_firmware_register_show, read_firmware_register_store);

static ssize_t curr_firmware_version_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return sprintf(buf, "Current firmware version: %04x\n",
	    cts_data->cts_dev.fwdata.version);
}
static DEVICE_ATTR(curr_version, S_IRUGO, curr_firmware_version_show, NULL);

static ssize_t rows_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return sprintf(buf, "Num rows: %u\n",
	    cts_data->cts_dev.fwdata.rows);
}
static DEVICE_ATTR(rows, S_IRUGO, rows_show, NULL);

static ssize_t cols_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return sprintf(buf, "Num cols: %u\n",
	    cts_data->cts_dev.fwdata.cols);
}
static DEVICE_ATTR(cols, S_IRUGO, cols_show, NULL);

static ssize_t res_x_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return sprintf(buf, "X Resolution: %u\n",
	    cts_data->cts_dev.fwdata.res_x);
}
static DEVICE_ATTR(res_x, S_IRUGO, res_x_show, NULL);

static ssize_t res_y_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return sprintf(buf, "Y Resolution: %u\n",
	    cts_data->cts_dev.fwdata.res_y);
}
static DEVICE_ATTR(res_y, S_IRUGO, res_y_show, NULL);

static ssize_t esd_protection_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	int ret;
    u8 esd_protection;

    ret = cts_fw_reg_readb(&cts_data->cts_dev, 0x8000 + 342, &esd_protection);
    if (ret) {
        return sprintf(buf, "Read firmware ESD protection register failed %d\n", ret);
    }

	return sprintf(buf, "ESD protection: %u\n", esd_protection);
}
static DEVICE_ATTR(esd_protection, S_IRUGO, esd_protection_show, NULL);

static ssize_t monitor_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	int ret;
    u8  value;

    ret = cts_fw_reg_readb(&cts_data->cts_dev, 0x8000 + 344, &value);
    if (ret) {
        return sprintf(buf, "Read firmware monitor enable register failed %d\n", ret);
    }

	return sprintf(buf, "Monitor mode: %s\n",
	    value & BIT(0) ? "Enable" : "Disable");
}

static ssize_t monitor_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	int ret;
    u8  value, enable = 0;

    if (buf[0] == 'Y' || buf[0] == 'y' || buf[0] == '1') {
        enable = 1;
    }

    cts_info("Write firmware monitor mode to '%c', %s",
        buf[0], value ? "Enable" : "Disable");

    ret = cts_fw_reg_readb(&cts_data->cts_dev, 0x8000 + 344, &value);
    if (ret) {
        cts_err("Write firmware monitor enable register failed %d", ret);
        return -EIO;
    }

    if ((value & BIT(0)) && enable) {
        cts_info("Monitor mode already enabled");
    } else if ((value & BIT(0)) == 0 && enable == 0) {
        cts_info("Monitor mode already disabled");
    } else {
        if (enable) {
            value |= BIT(0);
        } else {
            value &= ~BIT(0);
        }

        ret = cts_fw_reg_writeb(&cts_data->cts_dev, 0x8000 + 344, value);
        if (ret) {
            cts_err("Write firmware monitor enable register failed %d", ret);
            return -EIO;
        }
    }

	return count;
}
static DEVICE_ATTR(monitor_mode, S_IRUGO, monitor_mode_show, monitor_mode_store);

static ssize_t auto_compensate_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	int ret;
    u8  value;

    ret = cts_fw_reg_readb(&cts_data->cts_dev, 0x8000 + 276, &value);
    if (ret) {
        return sprintf(buf, "Read auto compensate enable register failed %d\n", ret);
    }

	return sprintf(buf, "Auto compensate: %s\n", value ? "Enable" : "Disable");
}
static DEVICE_ATTR(auto_compensate, S_IRUGO, auto_compensate_show, NULL);

#ifdef CFG_CTS_DRIVER_BUILTIN_FIRMWARE
static ssize_t driver_builtin_firmware_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int i, count = 0;

    count += snprintf(buf + count, PAGE_SIZE - count,
            "Total %d builtin firmware:\n",
            cts_get_num_driver_builtin_firmware());

    for (i = 0; i < cts_get_num_driver_builtin_firmware(); i++) {
        const struct cts_firmware *firmware =
            cts_request_driver_builtin_firmware_by_index(i);
        if (firmware) {
            count += snprintf(buf + count, PAGE_SIZE - count,
                        "%-2d: hwid: %04x fwid: %04x size: %6zu desc: %s\n",
                        i, firmware->hwid, firmware->fwid,
                        firmware->size, firmware->name);
         } else {
            count += snprintf(buf + count, PAGE_SIZE - count,
                        "%-2d: INVALID\n", i);
         }
    }

    return count;
}

/* echo index/name [flash/sram] > driver_builtin_firmware */
static ssize_t driver_builtin_firmware_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    const struct cts_firmware *firmware;
    bool to_flash = true;
    int ret, index = -1;

    parse_arg(buf, count);

    if (argc != 1 && argc != 2) {
        cts_err("Invalid num args %d\n"
                "  echo index/name [flash/sram] > driver_builtin_firmware\n", argc);
        return -EFAULT;
    }

    if (isdigit(*argv[0])) {
        index = simple_strtoul(argv[0], NULL, 0);
    }

    if (argc > 1) {
        if (strncasecmp(argv[1], "flash", 5) == 0) {
            to_flash = true;
        } else if (strncasecmp(argv[1], "sram", 4) == 0) {
            to_flash = false;
        } else {
            cts_err("Invalid location '%s', must be 'flash' or 'sram'", argv[1]);
            return -EINVAL;
        }
    }

    cts_info("Update driver builtin firmware '%s' to %s",
        argv[1], to_flash ? "flash" : "sram");

    if (index >= 0 && index < cts_get_num_driver_builtin_firmware()) {
        firmware = cts_request_driver_builtin_firmware_by_index(index);
    } else {
        firmware = cts_request_driver_builtin_firmware_by_name(argv[0]);
    }

    if (firmware) {
        ret = cts_stop_device(cts_dev);
        if (ret) {
            cts_err("Stop device failed %d", ret);
            return ret;
        }

        ret = cts_update_firmware(cts_dev, firmware, to_flash);
        if (ret) {
            cts_err("Update firmware failed %d", ret);
            goto err_start_device;
        }

        ret = cts_start_device(cts_dev);
        if (ret) {
            cts_err("Start device failed %d", ret);
            return ret;
        }
    } else {
        cts_err("Firmware '%s' NOT found", argv[0]);
        return -ENOENT;
    }

    return count;

err_start_device:
    cts_start_device(cts_dev);

    return ret;
}
static DEVICE_ATTR(driver_builtin_firmware, S_IWUSR | S_IRUGO,
        driver_builtin_firmware_show, driver_builtin_firmware_store);
#endif /* CFG_CTS_DRIVER_BUILTIN_FIRMWARE */

#ifdef CFG_CTS_FIRMWARE_IN_FS
/* echo filepath [flash/sram] > update_firmware_from_file */
static ssize_t update_firmware_from_file_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    const struct cts_firmware *firmware;
    bool to_flash = true;
    int ret;
    // int index;

    parse_arg(buf, count);

    if (argc > 2) {
        cts_err("Invalid num args %d\n"
                       "  echo filepath [flash/sram] > update_from_file\n", argc);
        return -EFAULT;
    } else if (argc > 1) {
        if (strncasecmp(argv[1], "flash", 5) == 0) {
            to_flash = true;
        } else if (strncasecmp(argv[1], "sram", 4) == 0) {
            to_flash = false;
        } else {
            cts_err("Invalid location '%s', must be 'flash' or 'sram'", argv[1]);
            return -EINVAL;
        }
    }

    cts_info("Update firmware from file '%s'", argv[0]);

    firmware = cts_request_firmware_from_fs(argv[0]);
    if (firmware) {
        ret = cts_stop_device(cts_dev);
        if (ret) {
            cts_err("Stop device failed %d", ret);
            return ret;
        }

        ret = cts_update_firmware(cts_dev, firmware, to_flash);
        if (ret) {
            cts_err("Update firmware failed %d", ret);
            //return ret;
        }

        ret = cts_start_device(cts_dev);
        if (ret) {
            cts_err("Start device failed %d", ret);
            return ret;
        }

        cts_release_firmware(firmware);
    } else {
        cts_err("Request firmware from file '%s' failed", argv[0]);
        return -ENOENT;
    }

    return count;
}
static DEVICE_ATTR(update_from_file, S_IWUSR, NULL, update_firmware_from_file_store);
#endif /* CFG_CTS_FIRMWARE_IN_FS */

static ssize_t updating_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return sprintf(buf, "Updating: %s\n",
	    cts_data->cts_dev.rtdata.updating ? "Y" : "N");
}
static DEVICE_ATTR(updating, S_IRUGO, updating_show, NULL);

static struct attribute *cts_dev_firmware_atts[] = {
    &dev_attr_read_reg.attr,
    &dev_attr_write_reg.attr,
	&dev_attr_curr_version.attr,
	&dev_attr_rows.attr,
    &dev_attr_cols.attr,
	&dev_attr_res_x.attr,
    &dev_attr_res_y.attr,
    &dev_attr_esd_protection.attr,
    &dev_attr_monitor_mode.attr,
    &dev_attr_auto_compensate.attr,
#ifdef CFG_CTS_DRIVER_BUILTIN_FIRMWARE
    &dev_attr_driver_builtin_firmware.attr,
#endif /* CFG_CTS_DRIVER_BUILTIN_FIRMWARE */
#ifdef CFG_CTS_FIRMWARE_IN_FS
	&dev_attr_update_from_file.attr,
#endif /* CFG_CTS_FIRMWARE_IN_FS */
    &dev_attr_updating.attr,
	NULL
};

static const struct attribute_group cts_dev_firmware_attr_group = {
    .name  = "firmware",
	.attrs = cts_dev_firmware_atts,
};

static ssize_t flash_info_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    const struct cts_flash *flash;

    if (cts_dev->flash == NULL) {
        bool program_mode;
        bool enabled;
        int  ret;

        program_mode = cts_is_device_program_mode(cts_dev);
        enabled = cts_is_device_enabled(cts_dev);
        
        ret = cts_prepare_flash_operation(cts_dev);
        if (ret) {
            return sprintf(buf, "Prepare flash operation failed %d", ret);
        }

        cts_post_flash_operation(cts_dev);

        if (!program_mode) {
            ret = cts_enter_normal_mode(cts_dev);
            if (ret) {
                return sprintf(buf, "Enter normal mode failed %d", ret);
            }
        }

        if (enabled) {
            ret = cts_start_device(cts_dev);
            if (ret) {
                return sprintf(buf, "Start device failed %d", ret);
            }
        }

        if (cts_dev->flash == NULL) {
            return sprintf(buf, "Flash not found\n");
        }
    }

    flash = cts_dev->flash;
    return snprintf(buf, PAGE_SIZE, 
                        "%s:\n"
                        "  JEDEC ID   : %06X\n"
                        "  Page size  : 0x%zx\n"
                        "  Sector size: 0x%zx\n"
                        "  Block size : 0x%zx\n"
                        "  Total size : 0x%zx\n",
        flash->name, flash->jedec_id, flash->page_size,
        flash->sector_size, flash->block_size, flash->total_size);
}
static DEVICE_ATTR(info, S_IRUGO, flash_info_show, NULL);

static ssize_t read_flash_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u32 flash_addr, size, i, remaining;
    u8 *data = NULL;
    ssize_t count = 0;
    int ret;
    bool program_mode;
    bool enabled;

    if (argc != 2 && argc != 3) {
        return sprintf(buf, "Invalid num args %d\n", argc);
    }

    ret = kstrtou32(argv[0], 0, &flash_addr);
    if (ret) {
        return sprintf(buf, "Invalid flash addr: %s\n", argv[0]);
    }
    ret = kstrtou32(argv[1], 0, &size);
    if (ret) {
        return sprintf(buf, "Invalid size: %s\n", argv[1]);
    }

    data = (u8 *)kmalloc(size, GFP_KERNEL);
    if (data == NULL) {
        return sprintf(buf, "Allocate buffer for read data failed\n");
    }

    cts_info("Read flash from 0x%06x size %u%s%s",
        flash_addr, size, argc == 3 ? " to file " : "",
        argc == 3 ? argv[2] : "");

    program_mode = cts_is_device_program_mode(cts_dev);
    enabled = cts_is_device_enabled(cts_dev);

    ret = cts_prepare_flash_operation(cts_dev);
    if (ret) {
        count += sprintf(buf, "Prepare flash operation failed %d", ret);
        goto err_free_data;
    }

    ret = cts_read_flash(cts_dev, flash_addr, data, size);
    if (ret) {
        count = sprintf(buf, "Read flash data failed %d\n", ret);
        goto err_post_flash_operation;
    }

    if (argc == 3) {
        struct file *file;

        cts_info("Write flash data to file '%s'", argv[2]);

        file = filp_open(argv[2], O_RDWR | O_CREAT, 0666);
        if (IS_ERR(file)) {
            count += sprintf(buf, "Open file '%s' failed %ld",
                argv[2], PTR_ERR(file));
            goto err_post_flash_operation;
        }

        ret = kernel_write(file, data, size, 0);
        if (ret != size) {
            count += sprintf(buf, "Write flash data to file '%s' failed %d",
                argv[2], ret);
        }

        ret = filp_close(file, NULL);
        if (ret) {
            count += sprintf(buf, "Close file '%s' failed %d", argv[2], ret);
        }
    } else {
#define PRINT_ROW_SIZE          (16)
        remaining = size;
        for (i = 0; i < size && count <= PAGE_SIZE; i += PRINT_ROW_SIZE) {
            size_t linelen = min((size_t)remaining, (size_t)PRINT_ROW_SIZE);
            remaining -= PRINT_ROW_SIZE;
        
            count += snprintf(buf + count, PAGE_SIZE - count - 1,
                        "%04x-%04x: ", flash_addr >> 16, flash_addr & 0xFFFF);
            /* Lower version kernel return void */
            hex_dump_to_buffer(data + i, linelen, PRINT_ROW_SIZE, 1,
                        buf + count, PAGE_SIZE - count - 1, true);
            count += strlen(buf + count);
            buf[count++] = '\n';
            flash_addr += linelen;
#undef PRINT_ROW_SIZE
        }
    }

err_post_flash_operation:
    cts_post_flash_operation(cts_dev);

    if (!program_mode) {
        int r = cts_enter_normal_mode(cts_dev);
        if (r) {
            count += sprintf(buf, "Enter normal mode failed %d", r);
        }
    }

    if (enabled) {
        int r = cts_start_device(cts_dev);
        if (r) {
            return sprintf(buf, "Start device failed %d", r);
        }
    }
err_free_data:
    kfree(data);

    return (ret < 0 ? ret : count);
}

/* echo start_addr size [filepath] > read */
static ssize_t read_flash_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    parse_arg(buf, count);

    return count;
}
static DEVICE_ATTR(read, S_IWUSR | S_IRUGO, read_flash_show, read_flash_store);

/* echo addr size > erase */
static ssize_t erase_flash_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u32 flash_addr, size;
    int ret;
    bool program_mode;
    bool enabled;

    parse_arg(buf, count);

    if (argc != 2) {
        cts_err("Invalid num args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou32(argv[0], 0, &flash_addr);
    if (ret) {
        cts_err("Invalid flash addr: %s", argv[0]);
        return -EINVAL;
    }
    ret = kstrtou32(argv[1], 0, &size);
    if (ret) {
        cts_err("Invalid size: %s", argv[1]);
        return -EINVAL;
    }

    cts_info("Erase flash from 0x%06x size %u", flash_addr, size);

    program_mode = cts_is_device_program_mode(cts_dev);
    enabled = cts_is_device_enabled(cts_dev);

    ret = cts_prepare_flash_operation(cts_dev);
    if (ret) {
        cts_err("Prepare flash operation failed %d", ret);
        return ret;
    }

    ret = cts_erase_flash(cts_dev, flash_addr, size);
    if (ret) {
        cts_err("Erase flash from 0x%06x size %u failed %d",
            flash_addr, size, ret);
        goto err_post_flash_operation;
    }

err_post_flash_operation:
    cts_post_flash_operation(cts_dev);

    if (!program_mode) {
        int r = cts_enter_normal_mode(cts_dev);
        if (r) {
            cts_err("Enter normal mode failed %d", r);
        }
    }

    if (enabled) {
        int r = cts_start_device(cts_dev);
        if (r) {
            cts_err("Start device failed %d", r);
        }
    }

    return (ret < 0 ? ret : count);
}
static DEVICE_ATTR(erase, S_IWUSR, NULL, erase_flash_store);

static struct attribute *cts_dev_flash_attrs[] = {
	&dev_attr_info.attr,
	&dev_attr_read.attr,
	&dev_attr_erase.attr,

	NULL
};

static const struct attribute_group cts_dev_flash_attr_group = {
    .name  = "flash",
	.attrs = cts_dev_flash_attrs,
};

static ssize_t open_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 threshold;
    int ret;

    if (argc != 1) {
        return sprintf(buf, "Invalid num args %d\n", argc);
    }

    ret = kstrtou16(argv[0], 0, &threshold);
    if (ret) {
        return sprintf(buf, "Invalid threshold: %s\n", argv[0]);
    }

    cts_info("Open test, threshold = %u", threshold);

    ret = cts_open_test(cts_dev, threshold);
    if (ret) {
        return sprintf(buf, "Open test FAILED %d, threshold = %u\n",
            ret, threshold);
    } else {
        return sprintf(buf, "Open test PASSED, threshold = %u\n",
            threshold);
    }
}

/* echo threshod > open_test */
static ssize_t open_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    parse_arg(buf, count);
    
    return count;
}
static DEVICE_ATTR(open_test, S_IWUSR | S_IRUGO, open_test_show, open_test_store);

static ssize_t short_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 threshold;
    int ret;

    if (argc != 1) {
        return sprintf(buf, "Invalid num args %d\n", argc);
    }

    ret = kstrtou16(argv[0], 0, &threshold);
    if (ret) {
        return sprintf(buf, "Invalid threshold: %s\n", argv[0]);
    }

    cts_info("Short test, threshold = %u", threshold);

    ret = cts_short_test(cts_dev, threshold);
    if (ret) {
        return sprintf(buf, "Short test FAILED %d, threshold = %u\n",
            ret, threshold);
    } else {
        return sprintf(buf, "Short test PASSED, threshold = %u\n",
            threshold);
    }
}

/* echo threshod > short_test */
static ssize_t short_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    parse_arg(buf, count);

    return count;
}
static DEVICE_ATTR(short_test, S_IWUSR | S_IRUGO,
        short_test_show, short_test_store);

static ssize_t testing_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return sprintf(buf, "Testting: %s\n",
	    cts_data->cts_dev.rtdata.testing ? "Y" : "N");
}
static DEVICE_ATTR(testing, S_IRUGO, testing_show, NULL);

static struct attribute *cts_dev_test_atts[] = {
	&dev_attr_open_test.attr,
	&dev_attr_short_test.attr,
    &dev_attr_testing.attr,
	NULL
};

static const struct attribute_group cts_dev_test_attr_group = {
    .name  = "test",
	.attrs = cts_dev_test_atts,
};

static ssize_t ic_type_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return sprintf(buf, "IC Type : %s\n",
	    cts_data->cts_dev.hwdata->name);
}
static DEVICE_ATTR(ic_type, S_IRUGO, ic_type_show, NULL);

static ssize_t program_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return sprintf(buf, "Program mode: %s\n",
	    cts_data->cts_dev.rtdata.program_mode ? "Y" : "N");
}
static ssize_t program_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    parse_arg(buf, count);

    if (argc != 1) {
        cts_err("Invalid num args %d", argc);
        return -EFAULT;
    }

    if (*argv[0] == '0' || tolower(*argv[0]) == 'y') {
    	int ret = cts_enter_program_mode(&cts_data->cts_dev);
    	if (ret) {
            cts_err("Enter program mode failed %d", ret);
            return ret;
    	}
    }

	return count;
}
static DEVICE_ATTR(program_mode, S_IWUSR | S_IRUGO,
        program_mode_show, program_mode_store);

static ssize_t rawdata_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#define RAWDATA_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)

	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 *rawdata = NULL;
    int ret, r, c, count = 0;
    u32 max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    bool data_valid = true;

    cts_info("Show rawdata");

    rawdata = (u16 *)kmalloc(RAWDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (rawdata == NULL) {
        return sprintf(buf, "Allocate memory for rawdata failed\n");
    }

    ret = cts_enable_get_rawdata(cts_dev);
    if (ret) {
        count += sprintf(buf, "Enable read raw data failed %d\n", ret);
        goto err_free_rawdata;
    }

    ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
    if (ret) {
        count += sprintf(buf, "Send cmd QUIT_GESTURE_MONITOR failed %d\n", ret);
        goto err_free_rawdata;
    }
    msleep(50);

    ret = cts_get_rawdata(cts_dev, rawdata);
    if(ret) {
        count += sprintf(buf, "Get raw data failed %d\n", ret);
        data_valid = false;
        // Fall through to disable get rawdata
    }
    ret = cts_disable_get_rawdata(cts_dev);
    if (ret) {
        count += sprintf(buf, "Disable read raw data failed %d\n", ret);
        // Fall through to show rawdata
    }

    if (data_valid) {
#define SPLIT_LINE_STR \
                    "----------------------------------------------------------------------------------------------\n"
#define ROW_NUM_FORMAT_STR  "%2d | "
#define COL_NUM_FORMAT_STR  " %2u  "
#define DATA_FORMAT_STR     "%4u "

        max = min = rawdata[0];
        sum = 0;
        max_r = max_c = min_r = min_c = 0;
        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                u16 val = rawdata[r * cts_dev->fwdata.cols + c];

                sum += val;
                if (val > max) {
                    max = val;
                    max_r = r;
                    max_c = c;
                } else if (val < min) {
                    min = val;
                    min_r = r;
                    min_c = c;
                }
            }
        }
        average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

        count += sprintf(buf + count,
            SPLIT_LINE_STR
            "Raw data MIN: [%d][%d]=%u, MAX: [%d][%d]=%u, AVG=%u\n"
            SPLIT_LINE_STR
            "   | ", min_r, min_c, min, max_r, max_c, max, average);
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            count += sprintf(buf + count, COL_NUM_FORMAT_STR, c);
        }
        count += sprintf(buf + count, "\n" SPLIT_LINE_STR);

        for (r = 0; r < cts_dev->fwdata.rows && count < PAGE_SIZE; r++) {
            count += sprintf(buf + count, ROW_NUM_FORMAT_STR, r);
            for (c = 0; c < cts_dev->fwdata.cols && count < PAGE_SIZE; c++) {
                count += snprintf(buf + count, PAGE_SIZE - count - 1,
                    DATA_FORMAT_STR, rawdata[r * cts_dev->fwdata.cols + c]);
            }
            buf[count++] = '\n';
        }
#undef SPLIT_LINE_STR
#undef ROW_NUM_FORMAT_STR
#undef COL_NUM_FORMAT_STR
#undef DATA_FORMAT_STR
    }

err_free_rawdata:
    kfree(rawdata);

	return (data_valid ? count : ret);
	
#undef RAWDATA_BUFFER_SIZE
}
static DEVICE_ATTR(rawdata, S_IRUGO, rawdata_show, NULL);

static ssize_t diffdata_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#define DIFFDATA_BUFFER_SIZE(cts_dev) \
        (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)

	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s16 *diffdata = NULL;
    int ret, r, c, count = 0;
    int max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    bool data_valid = true;

    cts_info("Show diffdata");

    diffdata = (s16 *)kmalloc(DIFFDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (diffdata == NULL) {
        cts_err("Allocate memory for diffdata failed");
        return -ENOMEM;
    }

    ret = cts_enable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("Enable read diff data failed %d", ret);
        goto err_free_diffdata;
    }

    ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
    if (ret) {
        cts_err("Send cmd QUIT_GESTURE_MONITOR failed %d", ret);
        goto err_free_diffdata;
    }
    msleep(50);

    ret = cts_get_diffdata(cts_dev, diffdata);
    if(ret) {
        cts_err("Get diff data failed %d", ret);
        data_valid = false;
        // Fall through to disable get diffdata
    }
    ret = cts_disable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("Disable read diff data failed %d", ret);
        // Fall through to show diffdata
    }

    if (data_valid) {
#define SPLIT_LINE_STR \
                    "----------------------------------------------------------------------------------------------\n"
#define ROW_NUM_FORMAT_STR  "%2d | "
#define COL_NUM_FORMAT_STR  "%4u "
#define DATA_FORMAT_STR     "%4d "

        max = min = diffdata[0];
        sum = 0;
        max_r = max_c = min_r = min_c = 0;
        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                s16 val = diffdata[r * cts_dev->fwdata.cols + c];

                sum += val;
                if (val > max) {
                    max = val;
                    max_r = r;
                    max_c = c;
                } else if (val < min) {
                    min = val;
                    min_r = r;
                    min_c = c;
                }
            }
        }
        average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

        count += sprintf(buf + count,
            SPLIT_LINE_STR
            "Diff data MIN: [%d][%d]=%d, MAX: [%d][%d]=%d, AVG=%d\n"
            SPLIT_LINE_STR
            "   | ", min_r, min_c, min, max_r, max_c, max, average);
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            count += sprintf(buf + count, COL_NUM_FORMAT_STR, c);
        }
        count += sprintf(buf + count, "\n" SPLIT_LINE_STR);

        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            count += sprintf(buf + count, ROW_NUM_FORMAT_STR, r);
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                count += snprintf(buf + count, PAGE_SIZE - count,
                    DATA_FORMAT_STR, diffdata[r * cts_dev->fwdata.cols + c]);
           }
           buf[count++] = '\n';
        }
#undef SPLIT_LINE_STR
#undef ROW_NUM_FORMAT_STR
#undef COL_NUM_FORMAT_STR
#undef DATA_FORMAT_STR
    }

err_free_diffdata:
    kfree(diffdata);

	return (data_valid ? count : ret);
#undef DIFFDATA_BUFFER_SIZE
}
static DEVICE_ATTR(diffdata, S_IRUGO, diffdata_show, NULL);

static struct attribute *cts_dev_misc_atts[] = {
	&dev_attr_ic_type.attr,
	&dev_attr_program_mode.attr,
    &dev_attr_rawdata.attr,
    &dev_attr_diffdata.attr,

	NULL
};

static const struct attribute_group cts_dev_misc_attr_group = {
    .name  = "misc",
	.attrs = cts_dev_misc_atts,
};

static const struct attribute_group *cts_dev_attr_groups[] = {
    &cts_dev_firmware_attr_group,
    &cts_dev_flash_attr_group,
    &cts_dev_test_attr_group,
    &cts_dev_misc_attr_group,
    NULL
};

int cts_sysfs_add_device(struct device *dev)
{
    int ret, i;

	cts_info("Add device attr groups");

    // Low version kernel NOT support sysfs_create_groups()
    for (i = 0; cts_dev_attr_groups[i]; i++) {
        ret = sysfs_create_group(&dev->kobj, cts_dev_attr_groups[i]);
        if (ret) {
            while (--i >= 0) {
                sysfs_remove_group(&dev->kobj, cts_dev_attr_groups[i]);
            }
            break;
        }
    }

    if (ret) {
        cts_err("Add device attr failed %d", ret);
    }

    return ret;
}

void cts_sysfs_remove_device(struct device *dev)
{
    int i;

	cts_info("Remove device attr groups");

    // Low version kernel NOT support sysfs_remove_groups()
    for (i = 0; cts_dev_attr_groups[i]; i++) {
        sysfs_remove_group(&dev->kobj, cts_dev_attr_groups[i]);
    }
}

#endif /* CONFIG_CTS_SYSFS */

