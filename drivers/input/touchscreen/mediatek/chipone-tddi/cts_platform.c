#define LOG_TAG         "Plat"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"

int tpd_rst_gpio_index = 0;
int tpd_int_gpio_index = 1;

size_t cts_plat_get_max_i2c_xfer_size(struct cts_platform_data *pdata)
{
#ifdef TPD_SUPPORT_I2C_DMA
    if (pdata->dma_available) {
        return CFG_CTS_MAX_I2C_XFER_SIZE;
    } else {
        return CFG_CTS_MAX_I2C_FIFO_XFER_SIZE;
    }
#else /* TPD_SUPPORT_I2C_DMA */
    return CFG_CTS_MAX_I2C_XFER_SIZE;
#endif /* TPD_SUPPORT_I2C_DMA */
}

u8 *cts_plat_get_i2c_xfer_buf(struct cts_platform_data *pdata, 
    size_t xfer_size)
{
#ifdef TPD_SUPPORT_I2C_DMA
    if (pdata->dma_available && xfer_size > CFG_CTS_MAX_I2C_FIFO_XFER_SIZE) {
        return pdata->i2c_dma_buff_va;
    } else
#endif /* TPD_SUPPORT_I2C_DMA */
        return pdata->i2c_fifo_buf;
}

int cts_plat_i2c_write(struct cts_platform_data *pdata, u8 i2c_addr,
        const void *src, size_t len, int retry, int delay)
{
    int ret = 0, retries = 0;

#ifdef TPD_SUPPORT_I2C_DMA
    struct i2c_msg msg = {
        .addr   = i2c_addr;
        .flags  = !I2C_M_RD,
        .len    = len,
        .timing = 300,
    };

    if (pdata->dma_available && len > CFG_CTS_MAX_I2C_FIFO_XFER_SIZE) {
        msg.ext_flag = (pdata->i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG);
        msg.buf = (u8 *)pdata->i2c_dma_buff_pa;
        memcpy(pdata->i2c_dma_buff_va, src, len);
    } else {
        msg.buf = (u8 *)src;
    }
    msg->len  = len;
#else
    struct i2c_msg msg = {
        .addr  = i2c_addr,
        .flags = !I2C_M_RD,
        .len   = len,
        .buf   = (u8 *)src,
    };
#endif /* TPD_SUPPORT_I2C_DMA */

    do {
        ret = i2c_transfer(pdata->i2c_client->adapter, &msg, 1);
        if (ret != 1) {
            if (ret >= 0) {
                ret = -EIO;
            }

            if (delay) {
                mdelay(delay);
            }
            continue;
        } else {
            return 0;
        }
    } while (++retries < retry);

    return ret;
}

int cts_plat_i2c_read(struct cts_platform_data *pdata, u8 i2c_addr,
        const u8 *wbuf, size_t wlen, void *rbuf, size_t rlen,
        int retry, int delay)
{
    int num_msg, ret = 0, retries = 0;

#ifdef TPD_SUPPORT_I2C_DMA
    struct i2c_msg msgs[2] = {
        {
            .addr   = i2c_addr,
            .flags  = !I2C_M_RD,
            .len    = wlen,
            .buf    = (u8 *)wbuf,
            .timing = 300,
        },
        {
            .addr     = i2c_addr,
            .flags    = I2C_M_RD,
            .len      = rlen,
            .timing   = 300,
        },
    };
    
    if (pdata->dma_available && rlen > CFG_CTS_MAX_I2C_FIFO_XFER_SIZE) {
        msgs[1].ext_flag = (pdata->i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG);
        msgs[1].buf      = (u8 *)pdata->i2c_dma_buff_pa;
    } else {
        msgs[1].buf = (u8 *)rbuf;
    }
#else /* TPD_SUPPORT_I2C_DMA */
    struct i2c_msg msgs[2] = {
        {
            .addr  = i2c_addr,
            .flags = !I2C_M_RD,
            .buf   = (u8 *)wbuf,
            .len   = wlen
        },
        {
            .addr  = i2c_addr,
            .flags = I2C_M_RD,
            .buf   = (u8 *)rbuf,
            .len   = rlen
        }
    };
#endif /* TPD_SUPPORT_I2C_DMA */

    if (wbuf == NULL || wlen == 0) {
        num_msg = 1;
    } else {
        num_msg = 2;
    }

    do {
        ret = i2c_transfer(pdata->i2c_client->adapter,
                msgs + ARRAY_SIZE(msgs) - num_msg, num_msg);

        if (ret != num_msg) {
            if (ret >= 0) {
                ret = -EIO;
            }

            if (delay) {
                mdelay(delay);
            }
            continue;
        } else {
#ifdef TPD_SUPPORT_I2C_DMA
            memcpy(rbuf, pdata->i2c_dma_buff_va, rlen);
#endif /* TPD_SUPPORT_I2C_DMA */

            return 0;
        }
    } while (++retries < retry);

    return ret;
}

int cts_plat_is_i2c_online(struct cts_platform_data *pdata, u8 i2c_addr)
{
    u8 dummy_bytes[2] = {0x00, 0x00};
    int ret;

    ret = cts_plat_i2c_write(pdata, i2c_addr, dummy_bytes, sizeof(dummy_bytes), 5, 2);
    if (ret) {
        cts_err("!!! I2C addr 0x%02x is offline !!!", i2c_addr);
        return false;
    } else {
        cts_dbg("I2C addr 0x%02x is online", i2c_addr);
        return true;
    }
}

static void cts_plat_handle_irq(struct cts_platform_data *pdata)
{
    int ret;

    cts_dbg("Handle IRQ");

    cts_lock_device(pdata->cts_dev);
    ret = cts_irq_handler(pdata->cts_dev);
    if (ret) {
        cts_err("Device handle IRQ failed %d", ret);
    }
    cts_unlock_device(pdata->cts_dev);
}

static irqreturn_t cts_plat_irq_handler(int irq, void *dev_id)
{
    struct cts_platform_data *pdata;
#ifndef CONFIG_GENERIC_HARDIRQS
    struct chipone_ts_data *cts_data;
#endif /* CONFIG_GENERIC_HARDIRQS */

    cts_dbg("IRQ handler");

    pdata = (struct cts_platform_data *)dev_id;
    if (pdata == NULL) {
        cts_err("IRQ handler with NULL dev_id");
        return IRQ_NONE;
    }

#ifdef CONFIG_GENERIC_HARDIRQS
    cts_plat_handle_irq(pdata);
#else /* CONFIG_GENERIC_HARDIRQS */
    cts_data = container_of(pdata->cts_dev, struct chipone_ts_data, cts_dev);

    cts_plat_disable_irq(pdata);

    queue_work(cts_data->workqueue, &pdata->ts_irq_work);
#endif /* CONFIG_GENERIC_HARDIRQS */

    return IRQ_HANDLED;
}

#ifndef CONFIG_GENERIC_HARDIRQS
static void cts_plat_touch_dev_irq_work(struct work_struct *work)
{
    struct cts_platform_data *pdata =
        container_of(work, struct cts_platform_data, ts_irq_work);

    cts_dbg("IRQ work");

    cts_plat_handle_irq(pdata);

    cts_plat_enable_irq(pdata);
}
#endif /* CONFIG_GENERIC_HARDIRQS */

int cts_init_platform_data(struct cts_platform_data *pdata,
        struct i2c_client *i2c_client)
{
    struct device_node *node = NULL;
	u32 ints[2] = { 0, 0 };

    cts_info("Init");

    pdata->i2c_client = i2c_client;
    pdata->ts_input_dev = tpd->dev;

    rt_mutex_init(&pdata->dev_lock);

#if !defined(CONFIG_GENERIC_HARDIRQS)
    /* Init work for bottom half of interrupt */
    INIT_WORK(&pdata->ts_irq_work, cts_plat_touch_dev_irq_work);
#endif /* CONFIG_GENERIC_HARDIRQS */

    if ((node = of_find_matching_node(node, touch_of_match)) == NULL) {
        cts_err("Find touch eint node failed");
        return -ENODATA;
    }
    if (of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints)) == 0) {
		gpio_set_debounce(ints[0], ints[1]);
	} else {
		cts_info("Debounce time not found");
	}
    pdata->irq = irq_of_parse_and_map(node, 0);
    if (pdata->irq == 0) {
        cts_err("Parse irq in dts failed");
        return -ENODEV;
    }
    pdata->i2c_client->irq = pdata->irq;
    spin_lock_init(&pdata->irq_lock);

#ifdef CONFIG_CTS_VIRTUALKEY
    pdata->vkey_num = tpd_dts_data.tpd_keycnt;
#endif /* CONFIG_CTS_VIRTUALKEY */

#ifdef CONFIG_CTS_GESTURE
    {
        u8 gesture_keymap[CFG_CTS_NUM_GESTURE][2] = CFG_CTS_GESTURE_KEYMAP;
        memcpy(pdata->gesture_keymap, gesture_keymap, sizeof(gesture_keymap));
        pdata->gesture_num = CFG_CTS_NUM_GESTURE;
    }
#endif /* CONFIG_CTS_GESTURE */

#ifdef TPD_SUPPORT_I2C_DMA
    tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
    pdata->i2c_dma_buff_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev,
            CFG_CTS_MAX_I2C_XFER_SIZE, &pdata->i2c_dma_buff_pa, GFP_KERNEL);
    if (pdata->i2c_dma_buff_va == NULL) {
        cts_err("Allocate I2C DMA Buffer failed!");
        //return -ENOMEM;
    } else {
        pdata->dma_available = true;
    }
#endif /* TPD_SUPPORT_I2C_DMA */

    return 0;
}

int cts_plat_request_resource(struct cts_platform_data *pdata)
{
    cts_info("Request resource");

    tpd_gpio_as_int(tpd_int_gpio_index);
    tpd_gpio_output(tpd_rst_gpio_index, 1);

    return 0;
}

int cts_plat_request_irq(struct cts_platform_data *pdata)
{
    int ret;

    cts_info("Request IRQ");

#ifdef CONFIG_GENERIC_HARDIRQS
    /* Note:
     * If IRQ request succeed, IRQ will be enbled !!!
     */
    ret = request_threaded_irq(pdata->irq,
            NULL, cts_plat_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
            pdata->i2c_client->dev.driver->name, pdata);
#else /* CONFIG_GENERIC_HARDIRQS */
    ret = request_irq(pdata->irq,
            cts_plat_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
            pdata->i2c_client->dev.driver->name, pdata);
#endif /* CONFIG_GENERIC_HARDIRQS */
    if (ret) {
        cts_err("Request IRQ failed %d", ret);
        return ret;
    }

    cts_plat_disable_irq(pdata);

    return 0;
}

void cts_plat_free_irq(struct cts_platform_data *pdata)
{
    free_irq(pdata->irq, pdata);
}

void cts_plat_free_resource(struct cts_platform_data *pdata)
{
    cts_info("Free resource");

    /**
     * Note:
     *    If resource request without managed, should free all resource
     *    requested in cts_plat_request_resource().
     */
     
#ifdef TPD_SUPPORT_I2C_DMA
    if (pdata->i2c_dma_buff_va) {
        dma_free_coherent(&tpd->dev->dev, CFG_CTS_MAX_I2C_XFER_SIZE,
            pdata->i2c_dma_buff_va, pdata->i2c_dma_buff_pa);
        pdata->i2c_dma_buff_va = NULL;
        pdata->i2c_dma_buff_pa = 0;
    }
#endif /* TPD_SUPPORT_I2C_DMA */
}

int cts_plat_enable_irq(struct cts_platform_data *pdata)
{
    unsigned long irqflags;

    cts_dbg("Enable IRQ");

    if (pdata->irq > 0) {
        spin_lock_irqsave(&pdata->irq_lock, irqflags);
        if (pdata->irq_is_disable)/* && !cts_is_device_suspended(pdata->chip)) */{  
            cts_dbg("Real enable IRQ");
            enable_irq(pdata->irq);
            pdata->irq_is_disable = false;
        }
        spin_unlock_irqrestore(&pdata->irq_lock, irqflags);

        return 0;
    }

    return -ENODEV;
}

int cts_plat_disable_irq(struct cts_platform_data *pdata)
{
    unsigned long irqflags;

    cts_dbg("Disable IRQ");

    if (pdata->irq > 0) {
        spin_lock_irqsave(&pdata->irq_lock, irqflags);
        if (!pdata->irq_is_disable) {
            cts_dbg("Real disable IRQ");
            disable_irq_nosync(pdata->irq);
            pdata->irq_is_disable = true;
        }
        spin_unlock_irqrestore(&pdata->irq_lock, irqflags);

        return 0;
    }

    return -ENODEV;
}

#ifdef CFG_CTS_HAS_RESET_PIN
int cts_plat_reset_device(struct cts_platform_data *pdata)
{
    cts_info("Reset device");

    tpd_gpio_output(tpd_rst_gpio_index, 0);
    mdelay(1);
    tpd_gpio_output(tpd_rst_gpio_index, 1);
    mdelay(50);

    return 0;
}
#endif /* CFG_CTS_HAS_RESET_PIN */

int cts_plat_power_up_device(struct cts_platform_data *pdata)
{
    cts_info("Power up device");

    return 0;
}

int cts_plat_power_down_device(struct cts_platform_data *pdata)
{
    cts_info("Power down device");

    return 0;
}

int cts_plat_init_touch_device(struct cts_platform_data *pdata)
{
    cts_info("Init touch device");

    return input_mt_init_slots(pdata->ts_input_dev,
        tpd_dts_data.touch_max_num, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
}

void cts_plat_deinit_touch_device(struct cts_platform_data *pdata)
{
    cts_info("De-init touch device");

    if (pdata->ts_input_dev) {
#ifndef CONFIG_GENERIC_HARDIRQS
        if (work_pending(&pdata->ts_irq_work)) {
            cancel_work_sync(&pdata->ts_irq_work);
        }
#endif /* CONFIG_GENERIC_HARDIRQS */
    }
}

static int tpd_history_x, tpd_history_y;
int cts_plat_process_touch_msg(struct cts_platform_data *pdata,
            struct cts_device_touch_msg *msgs, int num)
{
    struct input_dev *input_dev = pdata->ts_input_dev;
    int i;
    int contact = 0;

    cts_dbg("Process touch %d msgs", num);

    for (i = 0; i < num; i++) {
        u16 x, y;

        x = le16_to_cpu(msgs[i].x);
        y = le16_to_cpu(msgs[i].y);

#ifdef CFG_CTS_SWAP_XY
        swap(x,y);
#endif /* CFG_CTS_SWAP_XY */
#ifdef CFG_CTS_WRAP_X
        x = wrap(TPD_RES_X,x);
#endif /* CFG_CTS_WRAP_X */
#ifdef CFG_CTS_WRAP_Y
        y = wrap(TPD_RES_Y,y);
#endif /* CFG_CTS_WRAP_Y */

        cts_dbg("  Process touch msg[%d]: id[%u] ev=%u x=%u y=%u p=%u",
            i, msgs[i].id, msgs[i].event, x, y, msgs[i].pressure);

#ifdef CONFIG_CTS_SLOTPROTOCOL
        input_mt_slot(input_dev, msgs[i].id);
        switch (msgs[i].event) {
            case CTS_DEVICE_TOUCH_EVENT_DOWN:
                TPD_DEBUG_SET_TIME;
                TPD_EM_PRINT(x, y, x, y, msgs[i].id, 1);
                tpd_history_x = x;
                tpd_history_y = y;
#ifdef CONFIG_MTK_BOOT
                if (tpd_dts_data.use_tpd_button) {
                    if (FACTORY_BOOT == get_boot_mode() ||
                        RECOVERY_BOOT == get_boot_mode())
                        tpd_button(x, y, 1);
                }
#endif /* CONFIG_MTK_BOOT */
            case CTS_DEVICE_TOUCH_EVENT_MOVE:
            case CTS_DEVICE_TOUCH_EVENT_STAY:
                contact++;
                input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
                input_report_abs(input_dev, ABS_MT_POSITION_X, x);
                input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
                input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, msgs[i].pressure);
                input_report_abs(input_dev, ABS_MT_PRESSURE, msgs[i].pressure);
                break;

            case CTS_DEVICE_TOUCH_EVENT_UP:
                TPD_DEBUG_SET_TIME;
                TPD_EM_PRINT(tpd_history_x, tpd_history_y, tpd_history_x, tpd_history_y, msgs[i].id, 0);
                tpd_history_x = 0;
                tpd_history_y = 0;
#ifdef CONFIG_MTK_BOOT
                if (tpd_dts_data.use_tpd_button) {
                    if (FACTORY_BOOT == get_boot_mode() ||
                        RECOVERY_BOOT == get_boot_mode())
                        tpd_button(0, 0, 0);
                }
#endif /* CONFIG_MTK_BOOT */
                //input_report_key(input_dev, BTN_TOUCH, 0);
                input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
                break;

            default:
                cts_warn("Process touch msg with unknwon event %u id %u",
                    msgs[i].event, msgs[i].id);
                break;
        }
#else /* CONFIG_CTS_SLOTPROTOCOL */
        /**
         * If the driver reports one of BTN_TOUCH or ABS_PRESSURE
         * in addition to the ABS_MT events, the last SYN_MT_REPORT event
         * may be omitted. Otherwise, the last SYN_REPORT will be dropped
         * by the input core, resulting in no zero-contact event
         * reaching userland.
         */
        switch (msgs[i].event) {
            case CTS_CHIP_TOUCH_EVENT_DOWN:
            case CTS_CHIP_TOUCH_EVENT_MOVE:
            case CTS_CHIP_TOUCH_EVENT_STAY:
                input_report_abs(input_dev, ABS_MT_POSITION_X, x);
                input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
                input_report_abs(input_dev, ABS_MT_PRESSURE, msgs[i].pressure);
                input_mt_sync(input_dev);
                break;

            case CTS_CHIP_TOUCH_EVENT_UP:
                input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
                input_report_key(input_dev, BTN_TOUCH, 0);
                input_mt_sync(input_dev);
            default:
                cts_warn("Process touch msg with unknwon event %u id %u",
                    msgs[i].event, msgs[i].id);
                break;
        }
#endif /* CONFIG_CTS_SLOTPROTOCOL */
    }

    input_report_key(input_dev, BTN_TOUCH, contact > 0);

    input_mt_sync_frame(input_dev);
    input_sync(input_dev);

    return 0;
}

int cts_plat_release_all_touch(struct cts_platform_data *pdata)
{
    struct input_dev *input_dev = pdata->ts_input_dev;

#if !defined(CONFIG_CTS_SLOTPROTOCOL)
    int id;
#endif /* CONFIG_CTS_SLOTPROTOCOL */

    cts_info("Release all touch");

#ifdef CONFIG_CTS_SLOTPROTOCOL
    input_mt_sync_frame(input_dev);
#else /* CONFIG_CTS_SLOTPROTOCOL */
    for (id = 0; id < CFG_CTS_MAX_TOUCH_NUM; id++) {
        input_report_key(input_dev, BTN_TOUCH, 0);
        input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
        input_mt_sync(input_dev);
    }
#endif /* CONFIG_CTS_SLOTPROTOCOL */

    input_sync(input_dev);

    return 0;
}

#ifdef CONFIG_CTS_VIRTUALKEY
int cts_plat_init_vkey_device(struct cts_platform_data *pdata)
{
    pdata->vkey_state = 0;

    cts_info("Init Vkey");

    if (tpd_dts_data.use_tpd_button) {
        cts_info("Init vkey");

        pdata->vkey_state = 0;
        tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
                           tpd_dts_data.tpd_key_dim_local);
    }

    return 0;
}

void cts_plat_deinit_vkey_device(struct cts_platform_data *pdata)
{
    cts_info("De-init VKey");

    pdata->vkey_state = 0;
}

int cts_plat_process_vkey(struct cts_platform_data *pdata, u8 vkey_state)
{
    u8  event;
    int i;

    event = pdata->vkey_state ^ vkey_state;

    cts_dbg("Process vkey state=0x%02x, event=0x%02x", vkey_state, event);

    for (i = 0; i < pdata->vkey_num; i++) {
        if (event & BIT(i)) {
            tpd_button(x, y, vkey_state & BIT(i));

            /* MTK fobidon more than one key pressed in the same time */
            break;
        }
    }

    pdata->vkey_state = vkey_state;

    return 0;
}

int cts_plat_release_all_vkey(struct cts_platform_data *pdata)
{
    int i;

    cts_info("Release all vkeys");

    for (i = 0; i < pdata->vkey_num; i++) {
        if (pdata->vkey_state & BIT(i)) {
            tpd_button(x, y, 0);
        }
    }

    pdata->vkey_state = 0;

    return 0;
}
#endif /* CONFIG_CTS_VIRTUALKEY */

#ifdef CONFIG_CTS_GESTURE
int cts_plat_enable_irq_wake(struct cts_platform_data *pdata)
{
    cts_info("Enable IRQ wake");

    if (pdata->irq > 0) {
        if (!pdata->irq_wake_enabled) {
            pdata->irq_wake_enabled = true;
            return enable_irq_wake(pdata->irq);
        }

        cts_warn("Enable irq wake while already disabled");
        return -EINVAL;
    }

    cts_warn("Enable irq wake while irq invalid %d", pdata->irq);
    return -ENODEV;
}

int cts_plat_disable_irq_wake(struct cts_platform_data *pdata)
{
    cts_info("Disable IRQ wake");

    if (pdata->irq > 0) {
        if (pdata->irq_wake_enabled) {
            pdata->irq_wake_enabled = false;
            return disable_irq_wake(pdata->irq);
        }

        cts_warn("Disable irq wake while already disabled");
        return -EINVAL;
    }

    cts_warn("Disable irq wake while irq invalid %d", pdata->irq);
    return -ENODEV;
}

int cts_plat_init_gesture(struct cts_platform_data *pdata)
{
    int i;

    cts_info("Init gesture");

    // TODO: If system will issure enable/disable command, comment following line.
    //cts_enable_gesture_wakeup(pdata->cts_dev);

    for (i = 0; i < pdata->gesture_num; i ++) {
        input_set_capability(pdata->ts_input_dev, EV_KEY,
            pdata->gesture_keymap[i][1]);
    }

    return 0;
}

void cts_plat_deinit_gesture(struct cts_platform_data *pdata)
{
    cts_info("De-init gesture");
}

int cts_plat_process_gesture_info(struct cts_platform_data *pdata,
    struct cts_device_gesture_info *gesture_info)
{
    int i;

    cts_info("Process gesture, id=0x%02x", gesture_info->gesture_id);

#if defined(CFG_CTS_GESTURE_REPORT_KEY)
    for (i = 0; i < CFG_CTS_NUM_GESTURE; i++) {
        if (gesture_info->gesture_id == pdata->gesture_keymap[i][0]) {
            cts_info("Report key[%u]", pdata->gesture_keymap[i][1]);
            input_report_key(pdata->ts_input_dev,
                pdata->gesture_keymap[i][1], 1);
            input_sync(pdata->ts_input_dev);

            input_report_key(pdata->ts_input_dev,
                pdata->gesture_keymap[i][1], 0);
            input_sync(pdata->ts_input_dev);

            return 0;
        }
    }
#endif /* CFG_CTS_GESTURE_REPORT_KEY */

    cts_warn("Process unrecognized gesture id=%u",
        gesture_info->gesture_id);

    return -EINVAL;
}

#endif /* CONFIG_CTS_GESTURE */

