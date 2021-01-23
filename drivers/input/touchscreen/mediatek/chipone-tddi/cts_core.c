#define LOG_TAG         "Core"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_sfctrl.h"
#include "cts_spi_flash.h"
#include "cts_firmware.h"

static int cts_i2c_writeb(const struct cts_device *cts_dev,
        u32 addr, u8 b, int retry, int delay)
{
    u8  buff[8];

    cts_dbg("Write to i2c_addr: 0x%02x reg: 0x%0*x val: 0x%02x",
        cts_dev->rtdata.i2c_addr, cts_dev->rtdata.addr_width * 2, addr, b);

    if (cts_dev->rtdata.addr_width == 2) {
        put_unaligned_be16(addr, buff);
    } else if (cts_dev->rtdata.addr_width == 3) {
        put_unaligned_be24(addr, buff);
    } else {
        cts_err("Writeb invalid address width %u",
            cts_dev->rtdata.addr_width);
        return -EINVAL;
    }
    buff[cts_dev->rtdata.addr_width] = b;

    return cts_plat_i2c_write(cts_dev->pdata, cts_dev->rtdata.i2c_addr,
            buff, cts_dev->rtdata.addr_width + 1, retry ,delay);
}

static int cts_i2c_writew(const struct cts_device *cts_dev,
        u32 addr, u16 w, int retry, int delay)
{
    u8  buff[8];

    cts_dbg("Write to i2c_addr: 0x%02x reg: 0x%0*x val: 0x%04x",
        cts_dev->rtdata.i2c_addr, cts_dev->rtdata.addr_width * 2, addr, w);

    if (cts_dev->rtdata.addr_width == 2) {
        put_unaligned_be16(addr, buff);
    } else if (cts_dev->rtdata.addr_width == 3) {
        put_unaligned_be24(addr, buff);
    } else {
        cts_err("Writew invalid address width %u",
            cts_dev->rtdata.addr_width);
        return -EINVAL;
    }

    put_unaligned_le16(w, buff + cts_dev->rtdata.addr_width);

    return cts_plat_i2c_write(cts_dev->pdata, cts_dev->rtdata.i2c_addr,
            buff, cts_dev->rtdata.addr_width + 2, retry, delay);
}

static int cts_i2c_writel(const struct cts_device *cts_dev,
        u32 addr, u32 l, int retry, int delay)
{
    u8  buff[8];

    cts_dbg("Write to i2c_addr: 0x%02x reg: 0x%0*x val: 0x%08x",
        cts_dev->rtdata.i2c_addr, cts_dev->rtdata.addr_width * 2, addr, l);

    if (cts_dev->rtdata.addr_width == 2) {
        put_unaligned_be16(addr, buff);
    } else if (cts_dev->rtdata.addr_width == 3) {
        put_unaligned_be24(addr, buff);
    } else {
        cts_err("Writel invalid address width %u",
            cts_dev->rtdata.addr_width);
        return -EINVAL;
    }

    put_unaligned_le32(l, buff + cts_dev->rtdata.addr_width);

    return cts_plat_i2c_write(cts_dev->pdata, cts_dev->rtdata.i2c_addr,
            buff, cts_dev->rtdata.addr_width + 4, retry, delay);
}

static int cts_i2c_writesb(const struct cts_device *cts_dev, u32 addr,
        const u8 *src, size_t len, int retry, int delay)
{
    int ret;
    u8 *data;
    size_t max_xfer_size;
    size_t payload_len;
    size_t xfer_len;

    cts_dbg("Write to i2c_addr: 0x%02x reg: 0x%0*x len: %zu",
        cts_dev->rtdata.i2c_addr, cts_dev->rtdata.addr_width * 2, addr, len);

    max_xfer_size = cts_plat_get_max_i2c_xfer_size(cts_dev->pdata);
    data = cts_plat_get_i2c_xfer_buf(cts_dev->pdata, len);
    while (len) {
        payload_len =
            min((size_t)(max_xfer_size - cts_dev->rtdata.addr_width), len);
        xfer_len = payload_len + cts_dev->rtdata.addr_width;

        if (cts_dev->rtdata.addr_width == 2) {
            put_unaligned_be16(addr, data);
        } else if (cts_dev->rtdata.addr_width == 3) {
            put_unaligned_be24(addr, data);
        } else {
            cts_err("Writesb invalid address width %u",
                cts_dev->rtdata.addr_width);
            return -EINVAL;
        }

        memcpy(data + cts_dev->rtdata.addr_width, src, payload_len);

        ret = cts_plat_i2c_write(cts_dev->pdata, cts_dev->rtdata.i2c_addr,
                data, xfer_len, retry, delay);
        if (ret) {
            cts_err("Platform i2c write failed %d", ret);
            return ret;
        }

        src  += payload_len;
        len  -= payload_len;
        addr += payload_len;
    }

    return 0;
}

static int cts_i2c_readb(const struct cts_device *cts_dev,
        u32 addr, u8 *b, int retry, int delay)
{
    u8 addr_buf[4];

    cts_dbg("Readb from i2c_addr: 0x%02x reg: 0x%0*x",
        cts_dev->rtdata.i2c_addr, cts_dev->rtdata.addr_width * 2, addr);

    if (cts_dev->rtdata.addr_width == 2) {
        put_unaligned_be16(addr, addr_buf);
    } else if (cts_dev->rtdata.addr_width == 3) {
        put_unaligned_be24(addr, addr_buf);
    } else {
        cts_err("Readb invalid address width %u",
            cts_dev->rtdata.addr_width);
        return -EINVAL;
    }

    return cts_plat_i2c_read(cts_dev->pdata, cts_dev->rtdata.i2c_addr,
            addr_buf, cts_dev->rtdata.addr_width, b, 1, retry, delay);
}

static int cts_i2c_readw(const struct cts_device *cts_dev,
        u32 addr, u16 *w, int retry, int delay)
{
    int ret;
    u8  addr_buf[4];
    u8  buff[2];

    cts_dbg("Readw from i2c_addr: 0x%02x reg: 0x%0*x",
        cts_dev->rtdata.i2c_addr, cts_dev->rtdata.addr_width * 2, addr);

    if (cts_dev->rtdata.addr_width == 2) {
        put_unaligned_be16(addr, addr_buf);
    } else if (cts_dev->rtdata.addr_width == 3) {
        put_unaligned_be24(addr, addr_buf);
    } else {
        cts_err("Readw invalid address width %u",
            cts_dev->rtdata.addr_width);
        return -EINVAL;
    }

    ret = cts_plat_i2c_read(cts_dev->pdata, cts_dev->rtdata.i2c_addr,
            addr_buf, cts_dev->rtdata.addr_width, buff, 2, retry, delay);
    if (ret == 0) {
        *w = get_unaligned_le16(buff);
    }

    return ret;
}

static int cts_i2c_readl(const struct cts_device *cts_dev,
        u32 addr, u32 *l, int retry, int delay)
{
    int ret;
    u8  addr_buf[4];
    u8  buff[4];

    cts_dbg("Readl from i2c_addr: 0x%02x reg: 0x%0*x",
        cts_dev->rtdata.i2c_addr, cts_dev->rtdata.addr_width * 2, addr);

    if (cts_dev->rtdata.addr_width == 2) {
        put_unaligned_be16(addr, addr_buf);
    } else if (cts_dev->rtdata.addr_width == 3) {
        put_unaligned_be24(addr, addr_buf);
    } else {
        cts_err("Readl invalid address width %u",
            cts_dev->rtdata.addr_width);
        return -EINVAL;
    }

    ret = cts_plat_i2c_read(cts_dev->pdata, cts_dev->rtdata.i2c_addr,
            addr_buf, cts_dev->rtdata.addr_width, buff, 4, retry, delay);
    if (ret == 0) {
        *l = get_unaligned_le32(buff);
    }

    return ret;
}

static int cts_i2c_readsb(const struct cts_device *cts_dev,
        u32 addr, void *dst, size_t len, int retry, int delay)
{
    int ret;
    u8 addr_buf[4];
    size_t max_xfer_size, xfer_len;

    cts_dbg("Readsb from i2c_addr: 0x%02x reg: 0x%0*x len: %zu",
        cts_dev->rtdata.i2c_addr, cts_dev->rtdata.addr_width * 2, addr, len);

    max_xfer_size = cts_plat_get_max_i2c_xfer_size(cts_dev->pdata);
    while (len) {
        xfer_len = min(max_xfer_size, len);

        if (cts_dev->rtdata.addr_width == 2) {
            put_unaligned_be16(addr, addr_buf);
        } else if (cts_dev->rtdata.addr_width == 3) {
            put_unaligned_be24(addr, addr_buf);
        } else {
            cts_err("Readsb invalid address width %u",
                cts_dev->rtdata.addr_width);
            return -EINVAL;
        }

        ret = cts_plat_i2c_read(cts_dev->pdata, cts_dev->rtdata.i2c_addr,
                addr_buf, cts_dev->rtdata.addr_width, dst, xfer_len, retry, delay);
        if (ret) {
            cts_err("Platform i2c read failed %d", ret);
            return ret;
        }

        dst  += xfer_len;
        len  -= xfer_len;
        addr += xfer_len;
    }

    return 0;
}

static int cts_write_sram_normal_mode(const struct cts_device *cts_dev,
        u32 addr, const void *src, size_t len, int retry, int delay)
{
    int i, ret;
    u8    buff[5];

    for (i = 0; i < len; i++) {
        put_unaligned_le32(addr, buff);
        buff[4] = *(u8 *)src;
        
        addr++;
        src++;

        ret = cts_i2c_writesb(cts_dev,
                CTS_DEVICE_FW_REG_DEBUG_INTF, buff, 5, retry, delay);
        if (ret) {
            cts_err("Write rDEBUG_INTF len=5B failed %d",
                    ret);
            return ret;
        }
    }

    return 0;
}

int cts_sram_writeb_retry(const struct cts_device *cts_dev,
        u32 addr, u8 b, int retry, int delay)
{
    if (cts_dev->rtdata.program_mode) {
        return cts_i2c_writeb(cts_dev, addr, b, retry, delay);
    } else {
        return cts_write_sram_normal_mode(cts_dev, addr, &b, 1, retry, delay);
    }
}

int cts_sram_writew_retry(const struct cts_device *cts_dev,
        u32 addr, u16 w, int retry, int delay)
{
    u8 buff[2];

    if (cts_dev->rtdata.program_mode) {
        return cts_i2c_writew(cts_dev, addr, w, retry, delay);
    } else {
        put_unaligned_le16(w, buff);

        return cts_write_sram_normal_mode(cts_dev, addr, buff, 2, retry, delay);
    }
}

int cts_sram_writel_retry(const struct cts_device *cts_dev,
        u32 addr, u32 l, int retry, int delay)
{
    u8 buff[4];

    if (cts_dev->rtdata.program_mode) {
        return cts_i2c_writel(cts_dev, addr, l, retry, delay);
    } else {
        put_unaligned_le32(l, buff);

        return cts_write_sram_normal_mode(cts_dev, addr, buff, 4, retry, delay);
    }
}

int cts_sram_writesb_retry(const struct cts_device *cts_dev,
        u32 addr, const void *src, size_t len, int retry, int delay)
{
    if (cts_dev->rtdata.program_mode) {
        return cts_i2c_writesb(cts_dev, addr, src, len, retry, delay);
    } else {
        return cts_write_sram_normal_mode(cts_dev, addr, src, len, retry, delay);
    }
}

static int cts_calc_sram_crc(const struct cts_device *cts_dev,
    u32 sram_addr, size_t size, u32 *crc)
{
    cts_info("Calc crc from sram 0x%06x size %zu", sram_addr, size);

    return cts_dev->hwdata->sfctrl->ops->calc_sram_crc(cts_dev,
        sram_addr, size, crc);
}

int cts_sram_writesb_check_crc_retry(const struct cts_device *cts_dev,
        u32 addr, const void *src, size_t len, u32 crc, int retry)
{
    int ret, retries;

    retries = 0;
    do {
        u32 crc_sram;

        retries++;

        if ((ret = cts_sram_writesb(cts_dev, 0, src, len)) != 0) {
            cts_err("SRAM writesb failed %d", ret);
            continue;
        }
    
        if ((ret = cts_calc_sram_crc(cts_dev, 0, len, &crc_sram)) != 0) {
            cts_err("Get CRC for sram writesb failed %d retries %d",
                ret, retries);
            continue;
        }

        if (crc == crc_sram) {
            return 0;
        }

        cts_err("Check CRC for sram writesb mismatch %x != %x retries %d",
                crc, crc_sram, retries);
        ret = -EFAULT;
    }while (retries < retry);

    return ret;
}

static int cts_read_sram_normal_mode(const struct cts_device *cts_dev,
        u32 addr, void *dst, size_t len, int retry, int delay)
{
    int i, ret;

    for (i = 0; i < len; i++) {
        ret = cts_i2c_writel(cts_dev,
                CTS_DEVICE_FW_REG_DEBUG_INTF, addr, retry, delay);
        if (ret) {
            cts_err("Write addr to rDEBUG_INTF failed %d", ret);
            return ret;
        }

        ret = cts_i2c_readb(cts_dev,
                CTS_DEVICE_FW_REG_DEBUG_INTF + 4, (u8 *)dst, retry, delay);
        if (ret) {
            cts_err("Read value from rDEBUG_INTF + 4 failed %d",
                ret);
            return ret;
        }

        addr++;
        dst++;
    }

    return 0;
}

int cts_sram_readb_retry(const struct cts_device *cts_dev,
        u32 addr, u8 *b, int retry, int delay)
{
    if (cts_dev->rtdata.program_mode) {
        return cts_i2c_readb(cts_dev, addr, b, retry, delay);
    } else {
        return cts_read_sram_normal_mode(cts_dev, addr, b, 1, retry, delay);
    }
}

int cts_sram_readw_retry(const struct cts_device *cts_dev,
        u32 addr, u16 *w, int retry, int delay)
{
    int ret;
    u8 buff[2];

    if (cts_dev->rtdata.program_mode) {
        return cts_i2c_readw(cts_dev, addr, w, retry, delay);
    } else {
        ret = cts_read_sram_normal_mode(cts_dev, addr, buff, 2, retry, delay);
        if (ret) {
            cts_err("SRAM readw in normal mode failed %d", ret);
            return ret;
        }

        *w = get_unaligned_le16(buff);

        return 0;
    }
}

int cts_sram_readl_retry(const struct cts_device *cts_dev,
        u32 addr, u32 *l, int retry, int delay)
{
    int ret;
    u8 buff[4];
    
    if (cts_dev->rtdata.program_mode) {
        return cts_i2c_readl(cts_dev, addr, l, retry, delay);
    } else {
        ret = cts_read_sram_normal_mode(cts_dev, addr, buff, 4, retry, delay);
        if (ret) {
            cts_err("SRAM readl in normal mode failed %d", ret);
            return ret;
        }

        *l = get_unaligned_le32(buff);

        return 0;
    }
}

int cts_sram_readsb_retry(const struct cts_device *cts_dev,
        u32 addr, void *dst, size_t len, int retry, int delay)
{
    if (cts_dev->rtdata.program_mode) {
        return cts_i2c_readsb(cts_dev, addr, dst, len, retry, delay);
    } else {
        return cts_read_sram_normal_mode(cts_dev, addr, dst, len, retry, delay);
    }
}

int cts_fw_reg_writeb_retry(const struct cts_device *cts_dev,
        u32 reg_addr, u8 b, int retry, int delay)
{
    if (cts_dev->rtdata.program_mode) {
        cts_err("Writeb to fw reg 0x%04x under program mode", reg_addr);
        return -ENODEV;
    }

    return cts_i2c_writeb(cts_dev, reg_addr, b, retry, delay);
}

int cts_fw_reg_writew_retry(const struct cts_device *cts_dev,
        u32 reg_addr, u16 w, int retry, int delay)
{
    if (cts_dev->rtdata.program_mode) {
        cts_err("Writew to fw reg 0x%04x under program mode", reg_addr);
        return -ENODEV;
    }

    return cts_i2c_writew(cts_dev, reg_addr, w, retry, delay);
}

int cts_fw_reg_writel_retry(const struct cts_device *cts_dev,
        u32 reg_addr, u32 l, int retry, int delay)
{
    if (cts_dev->rtdata.program_mode) {
        cts_err("Writel to fw reg 0x%04x under program mode", reg_addr);
        return -ENODEV;
    }

    return cts_i2c_writel(cts_dev, reg_addr, l, retry, delay);
}

int cts_fw_reg_writesb_retry(const struct cts_device *cts_dev,
        u32 reg_addr, const void *src, size_t len, int retry, int delay)
{
    if (cts_dev->rtdata.program_mode) {
        cts_err("Writesb to fw reg 0x%04x under program mode", reg_addr);
        return -ENODEV;
    }

    return cts_i2c_writesb(cts_dev, reg_addr, src, len, retry, delay);
}

int cts_fw_reg_readb_retry(const struct cts_device *cts_dev,
        u32 reg_addr, u8 *b, int retry, int delay)
{
    if (cts_dev->rtdata.program_mode) {
        cts_err("Readb from fw reg under program mode");
        return -ENODEV;
    }

    return cts_i2c_readb(cts_dev, reg_addr, b, retry, delay);
}

int cts_fw_reg_readw_retry(const struct cts_device *cts_dev,
        u32 reg_addr, u16 *w, int retry, int delay)
{
    if (cts_dev->rtdata.program_mode) {
        cts_err("Readw from fw reg under program mode");
        return -ENODEV;
    }

    return cts_i2c_readw(cts_dev, reg_addr, w, retry, delay);
}

int cts_fw_reg_readl_retry(const struct cts_device *cts_dev,
        u32 reg_addr, u32 *l, int retry, int delay)
{
    if (cts_dev->rtdata.program_mode) {
        cts_err("Readl from fw reg under program mode");
        return -ENODEV;
    }

    return cts_i2c_readl(cts_dev, reg_addr, l, retry, delay);
}

int cts_fw_reg_readsb_retry(const struct cts_device *cts_dev,
        u32 reg_addr, void *dst, size_t len, int retry, int delay)
{
    if (cts_dev->rtdata.program_mode) {
        cts_err("Readsb from fw reg under program mode");
        return -ENODEV;
    }

    return cts_i2c_readsb(cts_dev, reg_addr, dst, len, retry, delay);
}

int cts_hw_reg_writeb_retry(const struct cts_device *cts_dev,
        u32 reg_addr, u8 b, int retry, int delay)
{
    return cts_sram_writeb_retry(cts_dev, reg_addr, b, retry, delay);
}

int cts_hw_reg_writew_retry(const struct cts_device *cts_dev,
        u32 reg_addr, u16 w, int retry, int delay)
{
    return cts_sram_writew_retry(cts_dev, reg_addr, w, retry, delay);
}

int cts_hw_reg_writel_retry(const struct cts_device *cts_dev,
        u32 reg_addr, u32 l, int retry, int delay)
{
    return cts_sram_writel_retry(cts_dev, reg_addr, l, retry, delay);
}

int cts_hw_reg_writesb_retry(const struct cts_device *cts_dev,
        u32 reg_addr, const void *src, size_t len, int retry, int delay)
{
    return cts_sram_writesb_retry(cts_dev, reg_addr, src, len, retry, delay);
}

int cts_hw_reg_readb_retry(const struct cts_device *cts_dev,
        u32 reg_addr, u8 *b, int retry, int delay)
{
    return cts_sram_readb_retry(cts_dev, reg_addr, b, retry, delay);
}

int cts_hw_reg_readw_retry(const struct cts_device *cts_dev,
        u32 reg_addr, u16 *w, int retry, int delay)
{
    return cts_sram_readw_retry(cts_dev, reg_addr, w, retry, delay);
}

int cts_hw_reg_readl_retry(const struct cts_device *cts_dev,
        u32 reg_addr, u32 *l, int retry, int delay)
{
    return cts_sram_readl_retry(cts_dev, reg_addr, l, retry, delay);
}

int cts_hw_reg_readsb_retry(const struct cts_device *cts_dev,
        u32 reg_addr, void *dst, size_t len, int retry, int delay)
{
    return cts_sram_readsb_retry(cts_dev, reg_addr, dst, len, retry, delay);
}

const static struct cts_sfctrl icn9911_sfctrl = {
    .reg_base = 0x34000,
    .xchg_sram_base = (80 - 1) * 1024,
    .xchg_sram_size = 1024, /* For non firmware programming */
    .ops = &cts_sfctrlv2_ops
};

const static struct cts_device_hwdata cts_device_hwdatas[] = {
    {
        .name = "ICNL9911",
        .hwid = CTS_HWID_ICNL9911,
        .fwid = CTS_FWID_ICNL9911,
        .num_row = 32,
        .num_col = 18,
        .sram_size = 80 * 1024,

        .program_addr_width = 3,

        .sfctrl = &icn9911_sfctrl,
    }
};

static int cts_init_device_hwdata(struct cts_device *cts_dev,
        u16 hwid, u16 fwid)
{
    int i;

    cts_dbg("Init hardware data hwid: %04x fwid: %04x", hwid, fwid);

    for (i = 0; i < ARRAY_SIZE(cts_device_hwdatas); i++) {
        if (hwid == cts_device_hwdatas[i].hwid ||
            fwid == cts_device_hwdatas[i].fwid) {
            cts_dev->hwdata = &cts_device_hwdatas[i];
            return 0;
        }
    }

    return -EINVAL;
}

void cts_lock_device(const struct cts_device *cts_dev)
{
    cts_dbg("*** Lock ***");

    rt_mutex_lock(&cts_dev->pdata->dev_lock);
}

void cts_unlock_device(const struct cts_device *cts_dev)
{
    cts_dbg("### Un-Lock ###");

    rt_mutex_unlock(&cts_dev->pdata->dev_lock);
}

int cts_set_work_mode(const struct cts_device *cts_dev, u8 mode)
{
    cts_info("Set work mode to %u", mode);

    return cts_fw_reg_writeb(cts_dev, CTS_DEVICE_FW_REG_WORK_MODE, mode);
}

int cts_get_work_mode(const struct cts_device *cts_dev, u8 *mode)
{
    return cts_fw_reg_readb(cts_dev, CTS_DEVICE_FW_REG_WORK_MODE, mode);
}

int cts_get_firmware_version(const struct cts_device *cts_dev, u16 *version)
{
    int ret = cts_fw_reg_readw(cts_dev, CTS_DEVICE_FW_REG_VERSION, version);

    if (ret) {
        *version = 0;
    } else {
        *version = be16_to_cpup(version);
    }

    return ret;
}

int cts_get_data_ready_flag(const struct cts_device *cts_dev, u8 *flag)
{
    return cts_fw_reg_readb(cts_dev, CTS_DEVICE_FW_REG_DATA_READY, flag);
}

int cts_clr_data_ready_flag(const struct cts_device *cts_dev)
{
    return cts_fw_reg_writeb(cts_dev, CTS_DEVICE_FW_REG_DATA_READY, 0);
}

int cts_send_command(const struct cts_device *cts_dev, u8 cmd)
{
    cts_info("Send command 0x%02x", cmd);

    if (cts_dev->rtdata.program_mode) {
        cts_warn("Send command %u while chip in program mode", cmd);
        return -ENODEV;
    }

    return cts_fw_reg_writeb_retry(cts_dev, CTS_DEVICE_FW_REG_CMD, cmd, 3, 0);
}

static int cts_get_touchinfo(const struct cts_device *cts_dev,
        struct cts_device_touch_info *touch_info)
{
    cts_dbg("Get touch info");

    if (cts_dev->rtdata.program_mode) {
        cts_warn("Get touch info in program mode");
        return -ENODEV;
    }

    if (cts_dev->rtdata.suspended) {
        cts_warn("Get touch info while is suspended");
        return -ENODEV;
    }

    return cts_fw_reg_readsb(cts_dev, CTS_DEVICE_FW_REG_TOUCH_INFO,
            touch_info, sizeof(*touch_info));
}

int cts_get_panel_param(const struct cts_device *cts_dev,
        void *param, size_t size)
{
    cts_info("Get panel parameter");

    if (cts_dev->rtdata.program_mode) {
        cts_warn("Get panel parameter in program mode");
        return -ENODEV;
    }

    return cts_fw_reg_readsb(cts_dev,
            CTS_DEVICE_FW_REG_PANEL_PARAM, param, size);
}

int cts_set_panel_param(const struct cts_device *cts_dev,
        const void *param, size_t size)
{
    cts_info("Set panel parameter");

    if (cts_dev->rtdata.program_mode) {
        cts_warn("Set panel parameter in program mode");
        return -ENODEV;
    }
    return cts_fw_reg_writesb(cts_dev,
            CTS_DEVICE_FW_REG_PANEL_PARAM, param, size);
}

int cts_get_x_resolution(const struct cts_device *cts_dev, u16 *resolution)
{
    return cts_fw_reg_readw(cts_dev, CTS_DEVICE_FW_REG_X_RESOLUTION, resolution);
}

int cts_get_y_resolution(const struct cts_device *cts_dev, u16 *resolution)
{
    return cts_fw_reg_readw(cts_dev, CTS_DEVICE_FW_REG_Y_RESOLUTION, resolution);
}

int cts_get_num_rows(const struct cts_device *cts_dev, u8 *num_rows)
{
    return cts_fw_reg_readb(cts_dev, CTS_DEVICE_FW_REG_NUM_TX, num_rows);
}

int cts_get_num_cols(const struct cts_device *cts_dev, u8 *num_cols)
{
    return cts_fw_reg_readb(cts_dev, CTS_DEVICE_FW_REG_NUM_RX, num_cols);
}

#ifdef CONFIG_CTS_LEGACY_TOOL
int cts_enable_get_rawdata(const struct cts_device *cts_dev)
{
    cts_info("Enable get raw/diff data");
    return cts_send_command(cts_dev, CTS_CMD_ENABLE_READ_RAWDATA);
}

int cts_disable_get_rawdata(const struct cts_device *cts_dev)
{
    cts_info("Disable get raw/diff data");
    return cts_send_command(cts_dev, CTS_CMD_DISABLE_READ_RAWDATA);
}

static int cts_get_scan_data(const struct cts_device *cts_dev,
        u32 addr, u8 n_col, void *buf)
{
    int i, ret;
    int row;

    /** - Wait data ready flag set */
    for (i = 0; i < 1000; i++) {
        u8 ready;

        mdelay(1);
        ret = cts_get_data_ready_flag(cts_dev, &ready);
        if (ret) {
            cts_err("Get data ready flag failed %d", ret);
            return ret;
        }

        if (ready) {
            goto read_data;
        }
    }

    return -ENODEV;

read_data:
    for (row = 0; row < cts_dev->fwdata.rows; row++) {
        ret = cts_fw_reg_readsb(cts_dev, addr, buf, cts_dev->fwdata.cols * 2);
        if (ret) {
            cts_err("Read raw data failed %d", ret);
            return ret;
        }

        addr += n_col * 2;
        buf  += cts_dev->fwdata.cols * 2;
    }

    ret = cts_clr_data_ready_flag(cts_dev);
    if (ret) {
        cts_err("Clear data ready flag failed %d", ret);
        return ret;
    }

    return 0;
}

int cts_get_rawdata(const struct cts_device *cts_dev, void *buf)
{
    cts_info("Get raw data");

    return cts_get_scan_data(cts_dev, CTS_DEVICE_FW_REG_RAW_DATA,
            cts_dev->hwdata->num_col, buf);
}

int cts_get_diffdata(const struct cts_device *cts_dev, void *buf)
{
    cts_info("Get diff data");

    return cts_get_scan_data(cts_dev,
            CTS_DEVICE_FW_REG_DIFF_DATA + (cts_dev->hwdata->num_col + 2 + 1) * 2,
            cts_dev->hwdata->num_col + 2, buf);
}
#endif /* CONFIG_CTS_LEGACY_TOOL */

static int cts_init_fwdata(struct cts_device *cts_dev)
{
    int ret;

    cts_info("Init firmware data");

    if (cts_dev->rtdata.program_mode) {
        cts_err("Init firmware data while in program mode");
        return -EINVAL;
    }

    ret = cts_get_firmware_version(cts_dev, &cts_dev->fwdata.version);
    if (ret) {
        cts_err("Read firmware version failed %d", ret);
        return ret;
    }
    cts_info("  %-12s: %04x", "Version", cts_dev->fwdata.version);

    ret = cts_get_x_resolution(cts_dev, &cts_dev->fwdata.res_x);
    if (ret) {
        cts_err("Read firmware X resoltion failed %d", ret);
        return ret;
    }
    cts_info("  %-12s: %u", "X resolution", cts_dev->fwdata.res_x);

    ret = cts_get_y_resolution(cts_dev, &cts_dev->fwdata.res_y);
    if (ret) {
        cts_err("Read firmware Y resoltion failed %d", ret);
        return ret;
    }
    cts_info("  %-12s: %u", "Y resolution", cts_dev->fwdata.res_y);

    ret = cts_get_num_rows(cts_dev, &cts_dev->fwdata.rows);
    if (ret) {
        cts_err("Read firmware num TX failed %d", ret);
        return ret;
    }
    cts_info("  %-12s: %u", "Num rows", cts_dev->fwdata.rows);

    ret = cts_get_num_cols(cts_dev, &cts_dev->fwdata.cols);
    if (ret) {
        cts_err("Read firmware num RX failed %d", ret);
        return ret;
    }
    cts_info("  %-12s: %u", "Num cols", cts_dev->fwdata.cols);

    return 0;
}

int cts_irq_handler(struct cts_device *cts_dev)
{
    int ret;

    cts_dbg("Enter IRQ handler");

    if (cts_dev->rtdata.program_mode) {
        cts_err("IRQ triggered in program mode");
        return -EINVAL;
    }

    if (unlikely(cts_dev->rtdata.suspended)) {
#ifdef CONFIG_CTS_GESTURE
        if (cts_dev->rtdata.gesture_wakeup_enabled) {
            struct cts_device_gesture_info gesture_info;

            ret = cts_get_gesture_info(cts_dev,
                    &gesture_info, CFG_CTS_GESTURE_REPORT_TRACE);
            if (ret) {
                cts_warn("Get gesture info failed %d", ret);
                //return ret;
            }

            /** - Issure another suspend with gesture wakeup command to device
             * after get gesture info.
             */
            cts_send_command(cts_dev, CTS_CMD_SUSPEND_WITH_GESTURE);

            ret = cts_plat_process_gesture_info(cts_dev->pdata, &gesture_info);
            if (ret) {
                cts_err("Process gesture info failed %d", ret);
                return ret;
            }
        } else {
            cts_warn("IRQ triggered while device suspended "
                    "without gesture wakeup enable");
        }
#endif /* CONFIG_CTS_GESTURE */
    } else {
        struct cts_device_touch_info touch_info;

        ret = cts_get_touchinfo(cts_dev, &touch_info);
        if (ret) {
            cts_err("Get touch info failed %d", ret);
            return ret;
        }
        cts_dbg("Touch info: vkey_state %x, num_msg %u",
            touch_info.vkey_state, touch_info.num_msg);

        ret = cts_plat_process_touch_msg(cts_dev->pdata,
            touch_info.msgs, touch_info.num_msg);
        if (ret) {
            cts_err("Process touch msg failed %d", ret);
            return ret;
        }

#ifdef CONFIG_CTS_VIRTUALKEY
        ret = cts_plat_process_vkey(cts_dev->pdata, touch_info.vkey_state);
        if (ret) {
            cts_err("Process vkey failed %d", ret);
            return ret;
        }
#endif /* CONFIG_CTS_VIRTUALKEY */
    }

    return 0;
}

bool cts_is_device_suspended(const struct cts_device *cts_dev)
{
    return cts_dev->rtdata.suspended;
}

int cts_suspend_device(struct cts_device *cts_dev)
{
    int ret;

    cts_info("Suspend device");

    if (cts_dev->rtdata.suspended) {
        cts_warn("Suspend device while already suspended");
        return 0;
    }

    ret = cts_send_command(cts_dev,
        cts_dev->rtdata.gesture_wakeup_enabled ? 
            CTS_CMD_SUSPEND_WITH_GESTURE : CTS_CMD_SUSPEND);

    if (ret){    
        cts_err("Suspend device failed %d", ret);
       
        return ret;
    }

    cts_info("Device suspended ...");
    cts_dev->rtdata.suspended = true;

    return 0;
}

int cts_resume_device(struct cts_device *cts_dev)
{
    int ret;

    cts_info("Resume device");

    /* DDI has just do chip reset, do nothing here */

    /* Check whether device is in normal mode */
    if (!cts_plat_is_i2c_online(cts_dev->pdata, CTS_NORMAL_MODE_I2CADDR)) {
        const struct cts_firmware *firmware;

        firmware = cts_request_firmware(cts_dev->hwdata->hwid,
                cts_dev->hwdata->fwid, 0);
        if (firmware) {
            ret = cts_update_firmware(cts_dev, firmware, true);
            cts_release_firmware(firmware);

            if (ret) {
                cts_err("Update default firmware failed %d", ret);
                goto err_set_program_mode;
            }
        } else {
            cts_err("Request default firmware failed %d, "
                    "please update manually!!", ret);

            goto err_set_program_mode;
        }
        
    }

#ifdef CONFIG_CTS_CHARGER_DETECT
    if (cts_is_charger_exist(cts_dev)) {
        cts_charger_plugin(cts_dev);
    }
#endif /* CONFIG_CTS_CHARGER_DETECT */

    cts_dev->rtdata.suspended = false;
    return 0;

err_set_program_mode:
    cts_dev->rtdata.program_mode = true;
    cts_dev->rtdata.i2c_addr     = CTS_PROGRAM_MODE_I2CADDR;
    cts_dev->rtdata.addr_width   = CTS_PROGRAM_MODE_I2C_ADDR_WIDTH;

    return ret;
}

bool cts_is_device_program_mode(const struct cts_device *cts_dev)
{
    return cts_dev->rtdata.program_mode;
}

static inline void cts_init_rtdata_with_normal_mode(struct cts_device *cts_dev)
{
    memset(&cts_dev->rtdata, 0, sizeof(cts_dev->rtdata));

    cts_dev->rtdata.i2c_addr        = CTS_NORMAL_MODE_I2CADDR;
    cts_dev->rtdata.addr_width      = 2;
    cts_dev->rtdata.program_mode    = false;
    cts_dev->rtdata.suspended       = false;
    cts_dev->rtdata.updating        = false;
    cts_dev->rtdata.testing         = false;
}

int cts_enter_program_mode(struct cts_device *cts_dev)
{
    const static u8 magic_num[] = {0xCC, 0x33, 0x55, 0x5A};
    u8  boot_mode;
    int ret;

    cts_info("Enter program mode");

    if (cts_dev->rtdata.program_mode) {
        cts_warn("Enter program mode while alredy in");
        return 0;
    }

    ret = cts_plat_i2c_write(cts_dev->pdata,
            CTS_PROGRAM_MODE_I2CADDR, magic_num, 4, 5, 10);
    if (ret) {
        cts_err("Write magic number to i2c_dev: 0x%02x failed %d",
            CTS_PROGRAM_MODE_I2CADDR, ret);
        return ret;
    }

    cts_dev->rtdata.program_mode = true;
    cts_dev->rtdata.i2c_addr     = CTS_PROGRAM_MODE_I2CADDR;
    cts_dev->rtdata.addr_width   = CTS_PROGRAM_MODE_I2C_ADDR_WIDTH;

    /* Write i2c deglitch register */
    ret = cts_hw_reg_writeb_retry(cts_dev, 0x37001, 0x0F, 5, 1);
    if (ret) {
        cts_err("Write i2c deglitch register failed\n");
        // FALL through
    }

    ret = cts_hw_reg_readb_retry(cts_dev, 0x30010, &boot_mode, 5, 10);
    if (ret) {
        cts_err("Read rBOOT_MODE failed %d", ret);
        return ret;
    }

    if (boot_mode != 2) {
        cts_err("rBOOT_MODE readback %u != PROMGRAM_MODE", boot_mode);
        return -EFAULT;
    }

    return 0;
}

int cts_enter_normal_mode(struct cts_device *cts_dev)
{
    int ret = 0;
    u8  boot_mode;

    cts_info("Enter normal mode");

    if (!cts_dev->rtdata.program_mode) {
        cts_warn("Enter normal mode while already in");
        return 0;
    }

    ret = cts_hw_reg_writeb_retry(cts_dev, 0x30010, 0x03, 5, 5);
    if (ret) {
        cts_err("Write rBOOT_MODE failed %d, try to reset device", ret);

        ret = cts_plat_reset_device(cts_dev->pdata);
        if (ret) {
            cts_err("Platform reset device failed %d", ret);
            return ret;
        }
    } else {
        mdelay(30);
    }

    if (cts_plat_is_i2c_online(cts_dev->pdata, CTS_NORMAL_MODE_I2CADDR)) {
        cts_dev->rtdata.program_mode = false;
        cts_dev->rtdata.i2c_addr     = CTS_NORMAL_MODE_I2CADDR;
        cts_dev->rtdata.addr_width   = 2;
    } else {
        goto err_init_i2c_program_mode;
    }

    ret = cts_hw_reg_readb_retry(cts_dev, 0x30010, &boot_mode, 5, 10);
    if (ret) {
        cts_err("Read rBOOT_MODE failed %d", ret);
        goto err_init_i2c_program_mode;
    }

    if (boot_mode != 3) {
        cts_err("rBOOT_MODE readback %u != SRAM_BOOT", boot_mode);
        ret = -EFAULT;
        goto err_init_i2c_program_mode;
    }

    ret = cts_init_fwdata(cts_dev);
    if (ret) {
        cts_err("Device init firmware data failed %d", ret);
        return ret;
    }

    return 0;

err_init_i2c_program_mode:
    cts_dev->rtdata.program_mode = true;
    cts_dev->rtdata.i2c_addr     = CTS_PROGRAM_MODE_I2CADDR;
    cts_dev->rtdata.addr_width   = CTS_PROGRAM_MODE_I2C_ADDR_WIDTH;

    return ret;
}

bool cts_is_device_enabled(const struct cts_device *cts_dev)
{
    return cts_dev->enabled;
}

int cts_start_device(struct cts_device *cts_dev)
{
#ifdef CONFIG_CTS_ESD_PROTECTION
    struct chipone_ts_data *cts_data =
        container_of(cts_dev, struct chipone_ts_data, cts_dev);
#endif /* CONFIG_CTS_ESD_PROTECTION */
    int ret;

    cts_info("Start device...");

    if (cts_is_device_enabled(cts_dev)) {
        cts_warn("Start device while already started");
        return 0;
    }

#ifdef CONFIG_CTS_ESD_PROTECTION
    cts_enable_esd_protection(cts_data);
#endif /* CONFIG_CTS_ESD_PROTECTION */

    if ((ret = cts_plat_enable_irq(cts_dev->pdata)) < 0) {
        cts_err("Enable IRQ failed %d", ret);
        return ret;
    }

    cts_dev->enabled = true;

    cts_info("Start device successfully");

    return 0;
}

int cts_stop_device(struct cts_device *cts_dev)
{
    struct chipone_ts_data *cts_data =
        container_of(cts_dev, struct chipone_ts_data, cts_dev);
    int ret;

    cts_info("Stop device...");

    if (!cts_is_device_enabled(cts_dev)) {
        cts_warn("Stop device while halted");
        return 0;
    }

    if (cts_is_firmware_updating(cts_dev)) {
        cts_warn("Stop device while firmware updating, please try again");
        return -EAGAIN;
    }

    if ((ret = cts_plat_disable_irq(cts_dev->pdata)) < 0) {
        cts_err("Disable IRQ failed %d", ret);
        return ret;
    }

    cts_dev->enabled = false;

#ifdef CONFIG_CTS_ESD_PROTECTION
    cts_disable_esd_protection(cts_data);
#endif /* CONFIG_CTS_ESD_PROTECTION */

    flush_workqueue(cts_data->workqueue);

    ret = cts_plat_release_all_touch(cts_dev->pdata);
    if (ret) {
        cts_err("Release all touch failed %d", ret);
        return ret;
    }

#ifdef CONFIG_CTS_VIRTUALKEY
    ret = cts_plat_release_all_vkey(cts_dev->pdata);
    if (ret) {
        cts_err("Release all vkey failed %d", ret);
        return ret;
    }
#endif /* CONFIG_CTS_VIRTUALKEY */

    return 0;
}

static bool cts_is_fwid_valid(u16 fwid)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(cts_device_hwdatas); i++) {
        if (cts_device_hwdatas[i].fwid == fwid) {
            return true;
        }
    }

    return false;
}

static bool cts_is_hwid_valid(u16 hwid)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(cts_device_hwdatas); i++) {
        if (cts_device_hwdatas[i].hwid == hwid) {
            return true;
        }
    }

    return false;
}

int cts_get_fwid(struct cts_device *cts_dev, u16 *fwid)
{
    int ret;

    cts_info("Get device firmware id");

    if (cts_dev->hwdata) {
        *fwid = cts_dev->hwdata->fwid;
        return 0;
    }

    if (cts_dev->rtdata.program_mode) {
        cts_err("Get device firmware id while in program mode");
        ret = -ENODEV;
        goto err_out;
    }

    ret = cts_fw_reg_readw_retry(cts_dev,
            CTS_DEVICE_FW_REG_CHIP_TYPE, fwid, 5, 1);
    if (ret) {
        goto err_out;
    }

    *fwid = be16_to_cpu(*fwid);

    cts_info("Device firmware id: %04x", *fwid);

    if (!cts_is_fwid_valid(*fwid)) {
        cts_warn("Get invalid firmware id %04x", *fwid);
        ret = -EINVAL;
        goto err_out;
    }

    return 0;

err_out:
    *fwid = CTS_FWID_INVALID;
    return ret;
}

int cts_get_hwid(struct cts_device *cts_dev, u16 *hwid)
{
    int ret;

    cts_info("Get device hardware id");

    if (cts_dev->hwdata) {
        *hwid = cts_dev->hwdata->hwid;
        return 0;
    }

    cts_info("Device hardware data not initialized, try to read from register");

    if (!cts_dev->rtdata.program_mode) {
        ret = cts_enter_program_mode(cts_dev);
        if (ret) {
            cts_err("Enter program mode failed %d", ret);
            goto err_out;
        }
    }

    ret = cts_hw_reg_readw_retry(cts_dev, 0x30001, hwid, 5, 0);
    if (ret) {
        goto err_out;
    }

    *hwid = le16_to_cpu(*hwid);

    cts_info("Device hardware id: %04x", *hwid);

    if (!cts_is_hwid_valid(*hwid)) {
        cts_warn("Device hardware id %04x invalid", *hwid);
        ret = -EINVAL;
        goto err_out;
    }

    return 0;

err_out:
    *hwid = CTS_HWID_INVALID;
    return ret;
}

int cts_probe_device(struct cts_device *cts_dev)
{
    int  ret, retries = 0;
    u16  fwid = CTS_FWID_INVALID;
    u16  hwid = CTS_HWID_INVALID;
    u16  device_fw_ver = 0;
    const struct cts_firmware *firmware = NULL;

    cts_info("Probe device");

read_fwid:
    if (cts_plat_is_i2c_online(cts_dev->pdata, CTS_NORMAL_MODE_I2CADDR)) {
        cts_init_rtdata_with_normal_mode(cts_dev);

        ret = cts_get_fwid(cts_dev, &fwid);
        if (ret) {
            cts_err("Get firmware id failed %d, retries %d", ret, retries);
        } else {
            ret = cts_fw_reg_readw_retry(cts_dev,
                    CTS_DEVICE_FW_REG_VERSION, &device_fw_ver, 5, 0);
            if (ret) {
                cts_err("Read firmware version failed %d", ret);
                device_fw_ver = 0;
                // TODO: Handle this error
            } else {
                device_fw_ver = be16_to_cpu(device_fw_ver);
                cts_info("Device firmware version: %04x", device_fw_ver);
            }
            goto init_hwdata;
        }
    } else {
        cts_warn("Normal mode i2c addr is offline");
    }

    /** - Try to read hardware id,
        it will enter program mode as normal */
    ret = cts_get_hwid(cts_dev, &hwid);
    if (ret || hwid == CTS_HWID_INVALID) {
        retries++;

        cts_err("Get hardware id failed %d retries %d", ret, retries);

        if (retries < 3) {
            cts_plat_reset_device(cts_dev->pdata);
            goto read_fwid;
        } else {
            return -ENODEV;
        }
    }

init_hwdata:
    ret = cts_init_device_hwdata(cts_dev, hwid, fwid);
    if (ret) {
        cts_err("Device hwid: %04x fwid: %04x not found", hwid, fwid);
        return -ENODEV;
    }

#ifdef CFG_CTS_FIRMWARE_FORCE_UPDATE
    cts_warn("Force update firmware");
    firmware = cts_request_firmware(CTS_HWID_ANY, CTS_FWID_ANY, 0);
#else /* CFG_CTS_FIRMWARE_FORCE_UPDATE */
    firmware = cts_request_firmware(hwid, fwid, device_fw_ver);
#endif /* CFG_CTS_FIRMWARE_FORCE_UPDATE */


    retries = 0;
update_firmware:
    if (firmware) {
        ++retries;
        ret = cts_update_firmware(cts_dev, firmware, true);
        if (ret) {
            cts_err("Update firmware failed %d retries %d", ret, retries);

            if (retries < 3) {
                cts_plat_reset_device(cts_dev->pdata);
                goto update_firmware;
            } else {
                cts_release_firmware(firmware);
                return ret;
            }
        } else {
            cts_release_firmware(firmware);
        }
    } else {
        if (fwid == CTS_FWID_INVALID) {
            /* Device without firmware running && no updatable firmware found */
            return -ENODEV;
        } else {
            ret = cts_init_fwdata(cts_dev);
            if (ret) {
                cts_err("Device init firmware data failed %d", ret);
                return ret;
            }
        }
    }

    return 0;
}

#ifdef CONFIG_CTS_GESTURE
void cts_enable_gesture_wakeup(struct cts_device *cts_dev)
{
    cts_info("Enable gesture wakeup");
    cts_dev->rtdata.gesture_wakeup_enabled = true;
}

void cts_disable_gesture_wakeup(struct cts_device *cts_dev)
{
    cts_info("Disable gesture wakeup");
    cts_dev->rtdata.gesture_wakeup_enabled = false;
}

bool cts_is_gesture_wakeup_enabled(const struct cts_device *cts_dev)
{
    return cts_dev->rtdata.gesture_wakeup_enabled;
}

int cts_get_gesture_info(const struct cts_device *cts_dev,
        void *gesture_info, bool trace_point)
{
    int    ret;

    cts_info("Get gesture info");

    if (cts_dev->rtdata.program_mode) {
        cts_warn("Get gesture info in program mode");
        return -ENODEV;
    }

    if (!cts_dev->rtdata.suspended) {
        cts_warn("Get gesture info while not suspended");
        return -ENODEV;
    }

    if (!cts_dev->rtdata.gesture_wakeup_enabled) {
        cts_warn("Get gesture info while gesture wakeup not enabled");
        return -ENODEV;
    }

    ret = cts_fw_reg_readsb(cts_dev,
            CTS_DEVICE_FW_REG_GESTURE_INFO, gesture_info, 2);
    if(ret) {
        cts_err("Read gesture info header failed %d", ret);
        return ret;
    }

    if (trace_point) {
        ret = cts_fw_reg_readsb(cts_dev, CTS_DEVICE_FW_REG_GESTURE_INFO + 2,
                gesture_info + 2,
                (((u8 *)gesture_info))[1] * sizeof(struct cts_device_gesture_point));
        if(ret) {
            cts_err("Read gesture trace points failed %d", ret);
            return ret;
        }
    }

    return 0;
}
#endif /* CONFIG_CTS_GESTURE */

#ifdef CONFIG_CTS_ESD_PROTECTION
static void cts_esd_protection_work(struct work_struct *work)
{
    struct chipone_ts_data *cts_data;
    int ret;
    u16 version;

    cts_info("ESD protection work");

    cts_data = container_of(work, struct chipone_ts_data, esd_work.work);

    cts_lock_device(&cts_data->cts_dev);

    ret = cts_get_firmware_version(&cts_data->cts_dev, &version);
    if (ret) {
        cts_err("ESD protection read rVERSION failed %d", ret);
        cts_data->esd_check_fail_cnt++;

        if (cts_data->esd_check_fail_cnt >= 3) {
            cts_warn("ESD protection check failed, reset chip !!!");

            cts_stop_device(&cts_data->cts_dev);

            ret = cts_plat_reset_device(cts_data->pdata);
            if (ret) {
                cts_err("ESD protection reset chip failed %d", ret);
            }

            cts_start_device(cts_data);
        }
    } else {
        cts_data->esd_check_fail_cnt = 0;
    }

    cts_unlock_device(&cts_data->cts_dev);

    queue_delayed_work(cts_data->workqueue,
        &cts_data->esd_work, CFG_CTS_ESD_PROTECTION_CHECK_PERIOD);
}

void cts_enable_esd_protection(struct chipone_ts_data *cts_data)
{
    if (cts_data->workqueue && !cts_data->esd_enabled) {
        cts_info("ESD protection enable");

        cts_data->esd_enabled = true;
        queue_delayed_work(cts_data->workqueue,
            &cts_data->esd_work, CFG_CTS_ESD_PROTECTION_CHECK_PERIOD);
    }
}

void cts_disable_esd_protection(struct chipone_ts_data *cts_data)
{
    if (cts_data->workqueue && cts_data->esd_enabled) {
        cts_info("ESD protection disable");

        cts_data->esd_enabled = false;
        cancel_delayed_work(&cts_data->esd_work);
    }
}

void cts_init_esd_protection(struct chipone_ts_data *cts_data)
{
    cts_info("Init ESD protection");

    INIT_DELAYED_WORK(&cts_data->esd_work, cts_esd_protection_work);

    cts_data->esd_enabled = false;
    cts_data->esd_check_fail_cnt = 0;
}

void cts_deinit_esd_protection(struct chipone_ts_data *cts_data)
{
    cts_info("De-Init ESD protection");

    if (cts_data->workqueue && cts_data->esd_enabled) {
        cts_data->esd_enabled = false;
        cancel_delayed_work(&cts_data->esd_work);
    }
}
#endif /* CONFIG_CTS_ESD_PROTECTION */

#ifdef CONFIG_CTS_GLOVE
int cts_enter_glove_mode(const struct cts_device *cts_dev)
{
    cts_info("Enter glove mode");
    return cts_fw_reg_writeb(cts_dev, 0x8000 + 149, 1);
}

int cts_exit_glove_mode(const struct cts_device *cts_dev)
{
    cts_info("Exit glove mode");
    return cts_fw_reg_writeb(cts_dev, 0x8000 + 149, 0);
}
#endif /* CONFIG_CTS_GLOVE */

#ifdef CONFIG_CTS_CHARGER_DETECT
bool cts_is_charger_exist(struct cts_device *cts_dev)
{
    return cts_dev->rtdata.charger_exist;
}

int cts_charger_plugin(struct cts_device *cts_dev)
{
    int ret;

    cts_info("Charger plugin");
    ret = cts_send_command(cts_dev, CTS_CMD_CHARGER_PLUG_IN);
    if (ret) {
        cts_err("Send CMD_CHARGER_PLUG_IN failed %d", ret);
    } else {
        cts_dev->rtdata.charger_exist = true;
    }
}

int cts_charger_plugout(struct cts_device *cts_dev)
{
    int ret;

    cts_info("Charger plugout");
    ret = cts_send_command(cts_dev, CTS_CMD_CHARGER_PLUG_OUT);
    if (ret) {
        cts_err("Send CMD_CHARGER_PLUG_OUT failed %d", ret);
    } else {
        cts_dev->rtdata.charger_exist = false;
    }
}
#endif /* CONFIG_CTS_GLOVE */

