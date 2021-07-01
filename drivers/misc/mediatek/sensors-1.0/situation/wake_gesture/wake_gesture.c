/* wakehub motion sensor driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/pm_wakeup.h>

#include <hwmsensor.h>
#include <sensors_io.h>
#include "wake_gesture.h"
#include "situation.h"

#include <hwmsen_helper.h>

#include <SCP_sensorHub.h>
#include <linux/notifier.h>
#include "scp_helper.h"


#define WAKEHUB_TAG                  "[wakehub] "
#define WAKEHUB_FUN(f)               pr_debug(WAKEHUB_TAG"%s\n", __func__)
#define WAKEHUB_PR_ERR(fmt, args...)    pr_err(WAKEHUB_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define WAKEHUB_LOG(fmt, args...)    pr_debug(WAKEHUB_TAG fmt, ##args)

static struct wakeup_source w_gesture_wake_lock;

static int wake_gesture_get_data(int *probability, int *status)
{
	int err = 0;
	struct data_unit_t data;
	uint64_t time_stamp = 0;

	err = sensor_get_data_from_hub(ID_WAKE_GESTURE, &data);
	if (err < 0) {
		WAKEHUB_PR_ERR("sensor_get_data_from_hub fail!!\n");
		return -1;
	}
	time_stamp = data.time_stamp;
	*probability = data.gesture_data_t.probability;
	WAKEHUB_LOG("recv ipi: timestamp: %lld, probability: %d!\n", time_stamp,
		*probability);
	return 0;
}
static int wake_gesture_open_report_data(int open)
{
	int ret = 0;

	WAKEHUB_LOG("%s : enable=%d\n", __func__, open);
#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	if (open == 1)
		ret = sensor_set_delay_to_hub(ID_WAKE_GESTURE, 120);
#elif defined CONFIG_NANOHUB

#else

#endif
	ret = sensor_enable_to_hub(ID_WAKE_GESTURE, open);
	return ret;
}
static int wake_gesture_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return sensor_batch_to_hub(ID_WAKE_GESTURE, flag, samplingPeriodNs, maxBatchReportLatencyNs);
}
static int wake_gesture_recv_data(struct data_unit_t *event, void *reserved)
{
	int err = 0;

	if (event->flush_action == FLUSH_ACTION)
		WAKEHUB_LOG("wake_gesture do not support flush\n");
	else if (event->flush_action == DATA_ACTION) {
		__pm_wakeup_event(&w_gesture_wake_lock, msecs_to_jiffies(100));
		err = situation_notify(ID_WAKE_GESTURE);
	}
	return err;
}

static int wakehub_local_init(void)
{
	struct situation_control_path ctl = {0};
	struct situation_data_path data = {0};
	int err = 0;

	ctl.open_report_data = wake_gesture_open_report_data;
	ctl.batch = wake_gesture_batch;
	ctl.is_support_wake_lock = true;
	err = situation_register_control_path(&ctl, ID_WAKE_GESTURE);
	if (err) {
		WAKEHUB_PR_ERR("register wake_gesture control path err\n");
		goto exit;
	}

	data.get_data = wake_gesture_get_data;
	err = situation_register_data_path(&data, ID_WAKE_GESTURE);
	if (err) {
		WAKEHUB_PR_ERR("register wake_gesture data path err\n");
		goto exit;
	}
	err = scp_sensorHub_data_registration(ID_WAKE_GESTURE, wake_gesture_recv_data);
	if (err) {
		WAKEHUB_PR_ERR("SCP_sensorHub_data_registration fail!!\n");
		goto exit_create_attr_failed;
	}
	wakeup_source_init(&w_gesture_wake_lock, "wake_gesture_wake_lock");
	return 0;
exit:
exit_create_attr_failed:
	return -1;
}
static int wakehub_local_uninit(void)
{
	return 0;
}

static struct situation_init_info wakehub_init_info = {
	.name = "wake_gesture_hub",
	.init = wakehub_local_init,
	.uninit = wakehub_local_uninit,
};

static int __init wakehub_init(void)
{
	situation_driver_add(&wakehub_init_info, ID_WAKE_GESTURE);
	return 0;
}

static void __exit wakehub_exit(void)
{
	WAKEHUB_FUN();
}

module_init(wakehub_init);
module_exit(wakehub_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GLANCE_GESTURE_HUB driver");
MODULE_AUTHOR("hongxu.zhao@mediatek.com");

