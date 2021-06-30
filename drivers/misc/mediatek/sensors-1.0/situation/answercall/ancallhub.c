/* ancallhub motion sensor driver
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
#include "ancallhub.h"
#include <situation.h>
#include <hwmsen_helper.h>

#include <SCP_sensorHub.h>
#include <linux/notifier.h>
#include "scp_helper.h"


#define ANCALLHUB_TAG                  "[ancallhub] "
#define ANCALLHUB_FUN(f)               pr_debug(ANCALLHUB_TAG"%s\n", __func__)
#define ANCALLHUB_PR_ERR(fmt, args...)    pr_err(ANCALLHUB_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define ANCALLHUB_LOG(fmt, args...)    pr_debug(ANCALLHUB_TAG fmt, ##args)

typedef enum {
	ANCALLHUBH_TRC_INFO = 0X10,
} ANCALLHUB_TRC;

static struct situation_init_info ancallhub_init_info;
static struct wakeup_source answer_c_wake_lock;

static int answer_call_get_data(int *probability, int *status)
{
	int err = 0;
	struct data_unit_t data;
	uint64_t time_stamp = 0;

	err = sensor_get_data_from_hub(ID_ANSWER_CALL, &data);
	if (err < 0) {
		ANCALLHUB_PR_ERR("sensor_get_data_from_hub fail!!\n");
		return -1;
	}
	time_stamp		= data.time_stamp;
	*probability	= data.gesture_data_t.probability;
	ANCALLHUB_LOG("recv ipi: timestamp: %lld, probability: %d!\n", time_stamp,
		*probability);
	return 0;
}
static int answer_call_open_report_data(int open)
{
	int ret = 0;

#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	if (open == 1)
		ret = sensor_set_delay_to_hub(ID_ANSWER_CALL, 120);
#elif defined CONFIG_NANOHUB

#else

#endif
	pr_debug("%s : type=%d, open=%d\n", __func__, ID_ANSWER_CALL, open);
	ret = sensor_enable_to_hub(ID_ANSWER_CALL, open);
	return ret;
}
static int answer_call_gesture_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return sensor_batch_to_hub(ID_ANSWER_CALL, flag, samplingPeriodNs, maxBatchReportLatencyNs);
}
static int answer_call_recv_data(struct data_unit_t *event, void *reserved)
{
	int err = 0;

	if (event->flush_action == FLUSH_ACTION)
		ANCALLHUB_LOG("answer_call do not support flush\n");
	else if (event->flush_action == DATA_ACTION) {
		__pm_wakeup_event(&answer_c_wake_lock, msecs_to_jiffies(100));
		err = situation_notify(ID_ANSWER_CALL);
	}
	return err;
}

static int ancallhub_local_init(void)
{
	struct situation_control_path ctl = {0};
	struct situation_data_path data = {0};
	int err = 0;

	ctl.open_report_data = answer_call_open_report_data;
	ctl.batch = answer_call_gesture_batch;
	ctl.is_support_wake_lock = true;
	err = situation_register_control_path(&ctl, ID_ANSWER_CALL);
	if (err) {
		ANCALLHUB_PR_ERR("register answer_call control path err\n");
		goto exit;
	}

	data.get_data = answer_call_get_data;
	err = situation_register_data_path(&data, ID_ANSWER_CALL);
	if (err) {
		ANCALLHUB_PR_ERR("register answer_call data path err\n");
		goto exit;
	}
	err = scp_sensorHub_data_registration(ID_ANSWER_CALL, answer_call_recv_data);
	if (err) {
		ANCALLHUB_PR_ERR("SCP_sensorHub_data_registration fail!!\n");
		goto exit_create_attr_failed;
	}
	wakeup_source_init(&answer_c_wake_lock, "answer_call_wake_lock");
	return 0;
exit:
exit_create_attr_failed:
	return -1;
}
static int ancallhub_local_uninit(void)
{
	return 0;
}

static struct situation_init_info ancallhub_init_info = {
	.name = "answer_call_hub",
	.init = ancallhub_local_init,
	.uninit = ancallhub_local_uninit,
};

static int __init ancallhub_init(void)
{
	situation_driver_add(&ancallhub_init_info, ID_ANSWER_CALL);
	return 0;
}

static void __exit ancallhub_exit(void)
{
	ANCALLHUB_FUN();
}

module_init(ancallhub_init);
module_exit(ancallhub_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ANSWER_CALL_HUB driver");
MODULE_AUTHOR("hongxu.zhao@mediatek.com");
