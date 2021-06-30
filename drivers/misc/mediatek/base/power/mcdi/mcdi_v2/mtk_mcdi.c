/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/cpu.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_qos.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/tick.h>
#include <linux/uaccess.h>

#include <mtk_cpuidle.h>
#include <mtk_idle.h>
#include <mtk_idle_profile.h>

#include <mtk_mcdi.h>
#include <mtk_mcdi_governor.h>

#include <mtk_mcdi_state.h>

#include <mtk_mcdi_governor_hint.h>
#include <mtk_mcdi_mbox.h>
#include <mtk_mcdi_profile.h>

#include <trace/events/mtk_idle_event.h>
#include <mt-plat/mtk_secure_api.h>


/* #define WORST_LATENCY_DBG */
/* #define USING_TICK_BROADCAST */

#define MCDI_CPU_OFF        1
#define MCDI_CLUSTER_OFF    1

#define NF_CMD_BUF          128
#define LOG_BUF_LEN         1024

enum MCUPM_FW_LOAD_STA {
	MCUPM_SMC_CALL    = 0x00F,
	MCUPM_FW_PARSE    = 0x0F0,
	MCUPM_FW_KICK     = 0x0FF,
};

static unsigned int mcupm_fw_load_success;
static unsigned long mcdi_cnt_cpu[NF_CPU];
static unsigned long mcdi_cnt_cluster[NF_CLUSTER];

void __iomem *mcdi_sysram_base;
void __iomem *mcdi_mcupm_base;
void __iomem *mcdi_mcupm_sram_base;

static unsigned long mcdi_cnt_cpu_last[NF_CPU];
static unsigned long mcdi_cnt_cluster_last[NF_CLUSTER];

static unsigned long any_core_cpu_cond_info_last[NF_ANY_CORE_CPU_COND_INFO];

static const char *any_core_cpu_cond_name[NF_ANY_CORE_CPU_COND_INFO] = {
	"pause",
	"multi core",
	"residency",
	"last core"
};

static const char *mcdi_kernel_profile_item_name[NF_MCDI_PROFILE - 1] = {
	"mcdi gov select",
	"before cpu/cluster off",
	"cpu/cluster off-on",
	"mcdi gov reflect"
};

static unsigned long long mcdi_heart_beat_log_prev;
static DEFINE_SPINLOCK(mcdi_heart_beat_spin_lock);

static unsigned int mcdi_heart_beat_log_dump_thd = 5000;          /* 5 sec */

#define log2buf(p, s, fmt, args...) \
	(p += scnprintf(p, sizeof(s) - strlen(s), fmt, ##args))

#undef mcdi_log
#define mcdi_log(fmt, args...)	log2buf(p, dbg_buf, fmt, ##args)

struct mtk_mcdi_buf {
	char buf[LOG_BUF_LEN];
	char *p_idx;
};

#define reset_mcdi_buf(mcdi) ((mcdi).p_idx = (mcdi).buf)
#define get_mcdi_buf(mcdi)   ((mcdi).buf)
#define mcdi_buf_append(mcdi, fmt, args...) \
	((mcdi).p_idx += scnprintf((mcdi).p_idx, LOG_BUF_LEN - strlen((mcdi).buf), fmt, ##args))

int __attribute__((weak)) soidle_enter(int cpu)
{
	return 1;
}

int __attribute__((weak)) dpidle_enter(int cpu)
{
	return 1;
}

int __attribute__((weak)) soidle3_enter(int cpu)
{
	return 1;
}

/* if success, return 1. If fail, return 0 */
bool mcdi_fw_is_ready(void)
{
	if (mcupm_fw_load_success == 0) {
		mcupm_fw_load_success =
			mt_secure_call(MTK_SIP_KERNEL_MCUPM_FW_LOAD_STATUS, 0, 0, 0);

		/* return val of mt_scure_call:
		 * 0x0F: mcupm smc call fail
		 * 0xF0: mcupm parse fw fail
		 * 0xFF: mcupm fw kick pass
		 */
		if (mcupm_fw_load_success != MCUPM_FW_KICK) {
#ifdef CONFIG_MTK_RAM_CONSOLE
			aee_rr_rec_mcdi_r15_val(mcupm_fw_load_success);
#endif
			set_mcdi_enable_status(false);
		}
	}
	return (mcupm_fw_load_success == MCUPM_FW_KICK) ? true : false;
}

int cluster_idx_get(int cpu)
{
	int ret = 0;

	/* if cpu < 0  or cpu > NR_CPU, return 0 */
	if (!(cpu >= 0 && cpu < NF_CPU)) {
		WARN_ON(!(cpu >= 0 && cpu < NF_CPU));

		return 0;
	}

	/* cpu / 2 = cluster index */
	ret = (cpu >> 2) & 0xff;

	return ret;
}

unsigned int get_pwr_stat_check_map(int type, int idx)
{
	if (!(type >= 0 && type < NF_PWR_STAT_MAP_TYPE))
		return 0;

	if (!(idx >= 0 && idx < NF_CPU))
		return 0;

	return cpu_cluster_pwr_stat_map[type][idx];
}

void wakeup_all_cpu(void)
{
	int cpu = 0;

	/*
	 * smp_proccessor_id() will be called in the flow of
	 * smp_send_reschedule(), hence disable preemtion to
	 * avoid being scheduled out.
	 */
	preempt_disable();

	for (cpu = 0; cpu < NF_CPU; cpu++) {
		if (cpu_online(cpu))
			smp_send_reschedule(cpu);
	}

	preempt_enable();
}

void wait_until_all_cpu_powered_on(void)
{
	while (!(mcdi_get_gov_data_num_mcusys() == 0x0))
		;
}

void mcdi_wakeup_all_cpu(void)
{
	wakeup_all_cpu();

	wait_until_all_cpu_powered_on();
}
/* debugfs */
static char dbg_buf[4096] = { 0 };
static char cmd_buf[512] = { 0 };

/* mcdi_state */
static int _mcdi_state_open(struct seq_file *s, void *data)
{
	return 0;
}

static int mcdi_state_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, _mcdi_state_open, inode->i_private);
}

static ssize_t mcdi_state_read(struct file *filp,
			       char __user *userbuf, size_t count, loff_t *f_pos)
{
	int len = 0;
	int i;
	char *p = dbg_buf;
	unsigned long any_core_cpu_cond_info[NF_ANY_CORE_CPU_COND_INFO];
	int latency_req = pm_qos_request(PM_QOS_CPU_DMA_LATENCY);

	struct mcdi_feature_status feature_stat;

	get_mcdi_feature_status(&feature_stat);

	mcdi_log("Feature:\n");
	mcdi_log("\tenable = %d\n", feature_stat.enable);
	mcdi_log("\tpause = %d\n", feature_stat.pause);
	mcdi_log("\tmax s_state = %d\n", feature_stat.s_state);
	mcdi_log("\tcluster_off = %d\n", feature_stat.cluster_off);
	mcdi_log("\tany_core = %d\n", feature_stat.any_core);

	mcdi_log("\n");

	mcdi_log("mcdi_cnt_cpu: ");
	for (i = 0; i < NF_CPU; i++)
		mcdi_log("%lu ", mcdi_cnt_cpu[i]);
	mcdi_log("\n");

	mcdi_log("mcdi_cnt_cluster: ");
	for (i = 0; i < NF_CLUSTER; i++) {
		mcdi_cnt_cluster[i] = mcdi_mbox_read(MCDI_MBOX_CLUSTER_0_CNT + i);
		mcdi_log("%lu ", mcdi_cnt_cluster[i]);
	}
	mcdi_log("\n");

	any_core_cpu_cond_get(any_core_cpu_cond_info);

	for (i = 0; i < NF_ANY_CORE_CPU_COND_INFO; i++) {
		mcdi_log("%s = %lu\n",
			any_core_cpu_cond_name[i],
			any_core_cpu_cond_info[i]
		);
	}

	mcdi_log("pm_qos latency_req = %d\n", latency_req);

	mcdi_log("system_idle_hint = %08x\n", system_idle_hint_result_raw());

	len = p - dbg_buf;

	return simple_read_from_buffer(userbuf, count, f_pos, dbg_buf, len);
}

static ssize_t mcdi_state_write(struct file *filp,
				const char __user *userbuf, size_t count, loff_t *f_pos)
{
	int ret = 0;
	unsigned long param = 0;
	char *cmd_ptr = cmd_buf;
	char *cmd_str = NULL;
	char *param_str = NULL;

	count = min(count, sizeof(cmd_buf) - 1);

	if (copy_from_user(cmd_buf, userbuf, count))
		return -EFAULT;

	cmd_buf[count] = '\0';

	cmd_str = strsep(&cmd_ptr, " ");

	if (cmd_str == NULL)
		return -EINVAL;

	param_str = strsep(&cmd_ptr, " ");

	if (param_str == NULL)
		return -EINVAL;

	ret = kstrtoul(param_str, 16, &param);

	if (ret < 0)
		return -EINVAL;

	if (!strncmp(cmd_str, "enable", strlen("enable"))) {
		if (mcdi_fw_is_ready())
			set_mcdi_enable_status(param != 0);
		return count;
	} else if (!strncmp(cmd_str, "s_state", strlen("s_state"))) {
		set_mcdi_s_state(param);
		return count;
	} else if (!strncmp(cmd_str, "hint", strlen("hint"))) {
		system_idle_hint_request(SYSTEM_IDLE_HINT_USER_MCDI_TEST, param != 0);
		return count;
	} else {
		return -EINVAL;
	}
}

/* mcdi profile */
static int _mcdi_profile_open(struct seq_file *s, void *data)
{
	return 0;
}

static int mcdi_profile_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, _mcdi_profile_open, inode->i_private);
}

static ssize_t mcdi_profile_read(struct file *filp,
			       char __user *userbuf, size_t count, loff_t *f_pos)
{
	int i, len = 0;
	unsigned int cnt[8] = {0};
	char *p = dbg_buf;
	unsigned int ratio_raw = 0;
	unsigned int ratio_int = 0;
	unsigned int ratio_fraction = 0;
	unsigned int ratio_dur = 0;

	/* distribution */
	for (i = 0; i < 3; i++) {
		cnt[i]   = mcdi_read(PROF_OFF_CNT_REG(i));
		cnt[i+4] = mcdi_read(PROF_ON_CNT_REG(i));
		cnt[3]  += cnt[i];
		cnt[7]  += cnt[i+4];
	}

	mcdi_log("mcdi cpu off    : max_id = 0x%x, avg = %4dus, max = %5dus, cnt = %d\n",
		mcdi_read(CPU_OFF_LATENCY_REG(0x0)),
		mcdi_read(CPU_OFF_LATENCY_REG(0x4)),
		mcdi_read(CPU_OFF_LATENCY_REG(0x8)),
		mcdi_read(CPU_OFF_LATENCY_REG(0xC)));
	mcdi_log("mcdi cpu on     : max_id = 0x%x, avg = %4dus, max = %5dus, cnt = %d\n",
		mcdi_read(CPU_ON_LATENCY_REG(0x0)),
		mcdi_read(CPU_ON_LATENCY_REG(0x4)),
		mcdi_read(CPU_ON_LATENCY_REG(0x8)),
		mcdi_read(CPU_ON_LATENCY_REG(0xC)));
	mcdi_log("mcdi cluster off: max_id = 0x%x, avg = %4dus, max = %5dus, cnt = %d\n",
		mcdi_read(Cluster_OFF_LATENCY_REG(0x0)),
		mcdi_read(Cluster_OFF_LATENCY_REG(0x4)),
		mcdi_read(Cluster_OFF_LATENCY_REG(0x8)),
		mcdi_read(Cluster_OFF_LATENCY_REG(0xC)));
	mcdi_log("mcdi cluster on : max_id = 0x%x, avg = %4dus, max = %5dus, cnt = %d\n",
		mcdi_read(Cluster_ON_LATENCY_REG(0x0)),
		mcdi_read(Cluster_ON_LATENCY_REG(0x4)),
		mcdi_read(Cluster_ON_LATENCY_REG(0x8)),
		mcdi_read(Cluster_ON_LATENCY_REG(0xC)));
	mcdi_log("\n");

	if (cnt[3] > 0 && cnt[7] > 0) {
		mcdi_log("pwr off latency  < 100 us : %2d%% (%d)\n", (100 * cnt[0]) / cnt[3], cnt[0]);
		mcdi_log("pwr off latency 100-500 us : %2d%% (%d)\n", (100 * cnt[1]) / cnt[3], cnt[1]);
		mcdi_log("pwr off latency   > 500 us : %2d%% (%d)\n", (100 * cnt[2]) / cnt[3], cnt[2]);
		mcdi_log("pwr on  latency  < 100 us : %2d%% (%d)\n", (100 * cnt[4]) / cnt[7], cnt[4]);
		mcdi_log("pwr on  latency 100-500 us : %2d%% (%d)\n", (100 * cnt[5]) / cnt[7], cnt[5]);
		mcdi_log("pwr on  latency   > 500 us : %2d%% (%d)\n", (100 * cnt[6]) / cnt[7], cnt[6]);
	}

#ifdef WORST_LATENCY_DBG
	mcdi_log("\n");
	mcdi_log("mcdi max latency     : %dus\n", mcdi_read(SYSRAM_PROF_DATA_REG));
	mcdi_log("mcdi ts wfi isr      : %u\n", mcdi_read(SYSRAM_PROF_DATA_REG + 0x04));
	mcdi_log("mcdi ts gic isr      : %u\n", mcdi_read(SYSRAM_PROF_DATA_REG + 0x08));
	mcdi_log("mcdi last ts wfi isr : %u\n", mcdi_read(SYSRAM_PROF_DATA_REG + 0x18));
	mcdi_log("mcdi last ts gic isr : %u\n", mcdi_read(SYSRAM_PROF_DATA_REG + 0x1C));
	mcdi_log("mcdi ts pwr on       : %u\n", mcdi_read(SYSRAM_PROF_DATA_REG + 0x10));
	mcdi_log("mcdi ts pwr off      : %u\n", mcdi_read(SYSRAM_PROF_DATA_REG + 0x14));
	mcdi_log("mcdi ts pause        : %u\n", mcdi_read(SYSRAM_PROF_DATA_REG + 0x0C));
	mcdi_log("mcdi last ts pwr on  : %u\n", mcdi_read(SYSRAM_PROF_DATA_REG + 0x24));
	mcdi_log("mcdi last ts pwr off : %u\n", mcdi_read(SYSRAM_PROF_DATA_REG + 0x28));
	mcdi_log("mcdi last ts pause   : %u\n", mcdi_read(SYSRAM_PROF_DATA_REG + 0x20));
#endif

	ratio_dur = mcdi_read(SYSRAM_PROF_RARIO_DUR);

	if (ratio_dur == 0)
		ratio_dur = ~0;

	mcdi_log("\nOFF %% (cpu):\n");

	mcdi_log("ratio_dur=%d\n", ratio_dur);
	for (i = 0; i < NF_CPU; i++) {
		ratio_raw      = 100 * mcdi_read(SYSRAM_PROF_RATIO_REG + i * 0x8 + 0x4);
		ratio_int      = ratio_raw / ratio_dur;
		ratio_fraction = (1000 * (ratio_raw % ratio_dur)) / ratio_dur;

		mcdi_log("%d: %3u.%03u%% (%u)\n", i, ratio_int, ratio_fraction, ratio_raw/100);
	}

	mcdi_log("\nOFF %% (cluster):\n");

	for (i = 0; i < NF_CLUSTER; i++) {
		ratio_raw      = 100 * mcdi_read(SYSRAM_PROF_RATIO_REG + (i + NF_CPU) * 0x8 + 0x4);
		ratio_int      = ratio_raw / ratio_dur;
		ratio_fraction = (1000 * (ratio_raw % ratio_dur)) / ratio_dur;

		mcdi_log("%d: %3u.%03u%% (%u)\n", i, ratio_int, ratio_fraction, ratio_raw/100);
	}

	mcdi_log("\nprof cpu = %d, count = %d, state = %d\n",
				get_mcdi_profile_cpu(),
				get_mcdi_profile_cnt(),
				mcdi_mbox_read(MCDI_MBOX_PROF_CMD));

	for (i = 0; i < (NF_MCDI_PROFILE - 1); i++)
		mcdi_log("%-22s: %u (us)\n", mcdi_kernel_profile_item_name[i], get_mcdi_profile_sum_us(i));

	len = p - dbg_buf;

	return simple_read_from_buffer(userbuf, count, f_pos, dbg_buf, len);
}

static ssize_t mcdi_profile_write(struct file *filp,
				const char __user *userbuf, size_t count, loff_t *f_pos)
{
	int ret = 0;
	unsigned long param = 0;
	char *cmd_ptr = cmd_buf;
	char *cmd_str = NULL;
	char *param_str = NULL;

	count = min(count, sizeof(cmd_buf) - 1);

	if (copy_from_user(cmd_buf, userbuf, count))
		return -EFAULT;

	cmd_buf[count] = '\0';

	cmd_str = strsep(&cmd_ptr, " ");

	if (cmd_str == NULL)
		return -EINVAL;

	param_str = strsep(&cmd_ptr, " ");

	if (param_str == NULL)
		return -EINVAL;

	ret = kstrtoul(param_str, 16, &param);

	if (ret < 0)
		return -EINVAL;

	if (!strncmp(cmd_str, "reg", strlen("reg"))) {
		if (!(param >= 0 && param < MCDI_SYSRAM_SIZE && (param % 4) == 0))
			return -EINVAL;

		pr_info("mcdi_reg: 0x%lx=0x%x(%d)\n",
			param, mcdi_read(mcdi_sysram_base + param), mcdi_read(mcdi_sysram_base + param));

		return count;

	} else if (!strncmp(cmd_str, "enable", strlen("enable"))) {
		if (param == MCDI_PROF_FLAG_STOP || param == MCDI_PROF_FLAG_START) {
			set_mcdi_profile_sampling(param);
			mcdi_mbox_write(MCDI_MBOX_PROF_CMD, param);
		}
		return count;

	} else if (!strncmp(cmd_str, "cpu", strlen("cpu"))) {
		set_mcdi_profile_target_cpu(param);
		return count;

	} else {
		return -EINVAL;

	}
}

static const struct file_operations mcdi_state_fops = {
	.open = mcdi_state_open,
	.read = mcdi_state_read,
	.write = mcdi_state_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations mcdi_profile_fops = {
	.open = mcdi_profile_open,
	.read = mcdi_profile_read,
	.write = mcdi_profile_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/* debugfs entry */
static const char mcdi_debugfs_dir_name[] = "mcdi";
static struct dentry *root_entry;

static int mcdi_debugfs_init(void)
{
	/* TODO: check if debugfs_create_file() failed */
	/* Initialize debugfs */
	root_entry = debugfs_create_dir(mcdi_debugfs_dir_name, NULL);
	if (!root_entry) {
		pr_debug("Can not create debugfs dir `%s`\n", mcdi_debugfs_dir_name);
		return 1;
	}

	debugfs_create_file("mcdi_state", 0644, root_entry, NULL, &mcdi_state_fops);
	debugfs_create_file("mcdi_profile", 0644, root_entry, NULL, &mcdi_profile_fops);

	return 0;
}

static void __go_to_wfi(void)
{
	isb();
	/* memory barrier before WFI */
	mb();
	__asm__ __volatile__("wfi" : : : "memory");
}

unsigned int mcdi_mbox_read(void __iomem *id)
{
	unsigned int val = 0;

	val = mcdi_read(id);

	return val;
}

void mcdi_mbox_write(void __iomem *id, unsigned int val)
{
	mcdi_write(id, val);
}

void mcdi_sysram_init(void)
{
	if (!mcdi_sysram_base)
		return;

	memset_io((void __iomem *)(mcdi_sysram_base + MCDI_DEBUG_INFO_NON_REPLACE_OFFSET),
				0,
				MCDI_SYSRAM_SIZE - MCDI_DEBUG_INFO_NON_REPLACE_OFFSET - MCDI_RESERVE_FOR_TIMESYNC);
}

static void mcdi_cluster_counter_set_cpu_residency(int state, int cpu)
{
#ifdef MCDI_CLUSTER_COUNTER
	struct cpuidle_driver *tbl = mcdi_state_tbl_get(cpu);
	struct cpuidle_state *s = &tbl->states[state];
	unsigned int target_residency = s->target_residency;

	/* DVT test */
	/* target_residency = 0xfffff; */
	mcdi_write(MCUPM_CFGREG_MP0_CPU0_RES + (cpu * 4), target_residency);
#endif
}

void mcdi_cpu_off(int cpu)
{
#if MCDI_CPU_OFF
	int state = 0;

	state = get_residency_latency_result(cpu);
	mcdi_cluster_counter_set_cpu_residency(state, cpu);

	switch (state) {
	case MCDI_STATE_CPU_OFF:

#ifdef USING_TICK_BROADCAST
		tick_broadcast_enter();
#endif

		mcdi_profile_ts(MCDI_PROFILE_CPU_DORMANT_ENTER);

		mtk_enter_idle_state(MTK_MCDI_CPU_MODE);

		mcdi_profile_ts(MCDI_PROFILE_CPU_DORMANT_LEAVE);

#ifdef USING_TICK_BROADCAST
		tick_broadcast_exit();
#endif

		break;
	case MCDI_STATE_CLUSTER_OFF:
	case MCDI_STATE_SODI:
	case MCDI_STATE_DPIDLE:
	case MCDI_STATE_SODI3:

#ifdef USING_TICK_BROADCAST
		tick_broadcast_enter();
#endif

		mcdi_profile_ts(MCDI_PROFILE_CPU_DORMANT_ENTER);

		mtk_enter_idle_state(MTK_MCDI_CLUSTER_MODE);

		mcdi_profile_ts(MCDI_PROFILE_CPU_DORMANT_LEAVE);

#ifdef USING_TICK_BROADCAST
		tick_broadcast_exit();
#endif

		break;
	default:
		/* should NOT happened */
		__go_to_wfi();

		break;
	}
#else
	__go_to_wfi();
#endif
}

void mcdi_cluster_off(int cpu)
{
#if MCDI_CLUSTER_OFF
	mcdi_profile_ts(MCDI_PROFILE_CPU_DORMANT_ENTER);
	mtk_enter_idle_state(MTK_MCDI_CLUSTER_MODE);
	mcdi_profile_ts(MCDI_PROFILE_CPU_DORMANT_LEAVE);
#elif MCDI_CPU_OFF
	mcdi_cpu_off(cpu);
#else
	__go_to_wfi();
#endif
}

void mcdi_heart_beat_log_dump(void)
{
	static struct mtk_mcdi_buf buf;
	int i;
	unsigned long long mcdi_heart_beat_log_curr = 0;
	unsigned long flags;
	bool dump_log = false;
	unsigned long mcdi_cnt;
	unsigned long any_core_info = 0;
	unsigned long any_core_cpu_cond_info[NF_ANY_CORE_CPU_COND_INFO];
	unsigned int cpu_mask = 0;
	unsigned int cluster_mask = 0;
	struct mcdi_feature_status feature_stat;

	if (!mcdi_mcupm_sram_base)
		return;

	spin_lock_irqsave(&mcdi_heart_beat_spin_lock, flags);

	mcdi_heart_beat_log_curr = idle_get_current_time_ms();

	if (mcdi_heart_beat_log_prev == 0)
		mcdi_heart_beat_log_prev = mcdi_heart_beat_log_curr;

	if ((mcdi_heart_beat_log_curr - mcdi_heart_beat_log_prev) > mcdi_heart_beat_log_dump_thd) {
		dump_log = true;
		mcdi_heart_beat_log_prev = mcdi_heart_beat_log_curr;
	}

	spin_unlock_irqrestore(&mcdi_heart_beat_spin_lock, flags);

	if (!dump_log)
		return;

	reset_mcdi_buf(buf);

	mcdi_buf_append(buf, "mcdi cpu: ");

	for (i = 0; i < NF_CPU; i++) {
		mcdi_cnt = mcdi_cnt_cpu[i] - mcdi_cnt_cpu_last[i];
		mcdi_buf_append(buf, "%lu, ", mcdi_cnt);
		mcdi_cnt_cpu_last[i] = mcdi_cnt_cpu[i];
	}

	mcdi_buf_append(buf, "cluster : ");

	for (i = 0; i < NF_CLUSTER; i++) {
		mcdi_cnt_cluster[i] = mcdi_mbox_read(MCDI_MBOX_CLUSTER_0_CNT + i);
		mcdi_cnt = mcdi_cnt_cluster[i] - mcdi_cnt_cluster_last[i];
		mcdi_buf_append(buf, "%lu, ", mcdi_cnt);
		mcdi_cnt_cluster_last[i] = mcdi_cnt_cluster[i];
	}

	any_core_cpu_cond_get(any_core_cpu_cond_info);

	for (i = 0; i < NF_ANY_CORE_CPU_COND_INFO; i++) {
		any_core_info = any_core_cpu_cond_info[i] - any_core_cpu_cond_info_last[i];
		mcdi_buf_append(buf, "%s = %lu, ", any_core_cpu_cond_name[i], any_core_info);
		any_core_cpu_cond_info_last[i] = any_core_cpu_cond_info[i];
	}

	get_mcdi_avail_mask(&cpu_mask, &cluster_mask);

	mcdi_buf_append(buf, "avail cpu = %04x, cluster = %04x", cpu_mask, cluster_mask);

	get_mcdi_feature_status(&feature_stat);

	mcdi_buf_append(buf, ", enabled = %d, max_s_state = %d",
						feature_stat.enable,
						feature_stat.s_state);

	mcdi_buf_append(buf, ", system_idle_hint = %08x ", system_idle_hint_result_raw());

	if (mcupm_fw_load_success != MCUPM_FW_KICK)
		mcdi_buf_append(buf, "(mcdi_fw_fail: 0x%x)\n", mcupm_fw_load_success);
	else
		mcdi_buf_append(buf, "\n");

	pr_info("%s\n", get_mcdi_buf(buf));
}

int mcdi_enter(int cpu)
{
	int cluster_idx = cluster_idx_get(cpu);
	int state = -1;

	idle_refcnt_inc();
	mcdi_profile_ts(MCDI_PROFILE_ENTER);

	if (likely(mcdi_fw_is_ready()))
		state = mcdi_governor_select(cpu, cluster_idx);
	else
		state = MCDI_STATE_WFI;

	mcdi_profile_ts(MCDI_PROFILE_MCDI_GOVERNOR_SELECT_LEAVE);

	if (state >= MCDI_STATE_WFI && state <= MCDI_STATE_CLUSTER_OFF)
		sched_idle_set_state(&(mcdi_state_tbl_get(cpu)->states[state]), state);

	switch (state) {
	case MCDI_STATE_WFI:
		__go_to_wfi();

		break;
	case MCDI_STATE_CPU_OFF:

		trace_mcdi_rcuidle(cpu, 1);

#ifdef CONFIG_MTK_RAM_CONSOLE
		aee_rr_rec_mcdi_val(cpu, 0xff);
#endif

		mcdi_cpu_off(cpu);

#ifdef CONFIG_MTK_RAM_CONSOLE
		aee_rr_rec_mcdi_val(cpu, 0x0);
#endif

		trace_mcdi_rcuidle(cpu, 0);

		mcdi_cnt_cpu[cpu]++;

		break;
	case MCDI_STATE_CLUSTER_OFF:

		trace_mcdi_rcuidle(cpu, 1);

#ifdef CONFIG_MTK_RAM_CONSOLE
		aee_rr_rec_mcdi_val(cpu, 0xff);
#endif

		mcdi_cluster_off(cpu);

#ifdef CONFIG_MTK_RAM_CONSOLE
		aee_rr_rec_mcdi_val(cpu, 0x0);
#endif

		trace_mcdi_rcuidle(cpu, 0);

		mcdi_cnt_cpu[cpu]++;

		break;
	case MCDI_STATE_SODI:

		mcdi_profile_ts(MCDI_PROFILE_CPU_DORMANT_ENTER);

		soidle_enter(cpu);

		mcdi_profile_ts(MCDI_PROFILE_CPU_DORMANT_LEAVE);

		break;
	case MCDI_STATE_DPIDLE:

		mcdi_profile_ts(MCDI_PROFILE_CPU_DORMANT_ENTER);

		dpidle_enter(cpu);

		mcdi_profile_ts(MCDI_PROFILE_CPU_DORMANT_LEAVE);

		break;
	case MCDI_STATE_SODI3:

		mcdi_profile_ts(MCDI_PROFILE_CPU_DORMANT_ENTER);

		soidle3_enter(cpu);

		mcdi_profile_ts(MCDI_PROFILE_CPU_DORMANT_LEAVE);

		break;
	}

	if (state >= MCDI_STATE_WFI && state <= MCDI_STATE_CLUSTER_OFF)
		sched_idle_set_state(NULL, -1);

	mcdi_governor_reflect(cpu, state);

	idle_refcnt_dec();
	mcdi_profile_ts(MCDI_PROFILE_LEAVE);

	/* if (state == MCDI_STATE_DPIDLE) */
	mcdi_profile_calc();

	return 0;
}

bool mcdi_pause(bool paused)
{
	struct mcdi_feature_status feature_stat;

	get_mcdi_feature_status(&feature_stat);

	if (!feature_stat.enable)
		return true;

	if (paused) {
		mcdi_state_pause(true);
		mcdi_wakeup_all_cpu();
	} else {
		mcdi_state_pause(false);
	}

	return true;
}

bool mcdi_task_pause(bool paused)
{
	if (!is_mcdi_working())
		return true;

	/* TODO */
	if (paused) {
		trace_mcdi_task_pause_rcuidle(smp_processor_id(), true);

		/* Notify SSPM to disable MCDI */
		mcdi_mbox_write(MCDI_MBOX_PAUSE_ACTION, 1);

		/* Polling until MCDI Task stopped */
		while (!(mcdi_mbox_read(MCDI_MBOX_PAUSE_ACK) == 1))
			;
	} else {
		/* Notify SSPM to enable MCDI */
		mcdi_mbox_write(MCDI_MBOX_PAUSE_ACTION, 0);
		/* Polling until MCDI Task resume */
		while (!(mcdi_mbox_read(MCDI_MBOX_PAUSE_ACK) == 0))
			;

		trace_mcdi_task_pause_rcuidle(smp_processor_id(), 0);
	}

	return true;
}

void update_avail_cpu_mask_to_mcdi_controller(unsigned int cpu_mask)
{
	mcdi_mbox_write(MCDI_MBOX_AVAIL_CPU_MASK, cpu_mask);
}

void update_cpu_isolation_mask_to_mcdi_controller(unsigned int iso_mask)
{

}

bool is_cpu_pwr_on_event_pending(void)
{
	return (!(mcdi_mbox_read(MCDI_MBOX_PENDING_ON_EVENT) == 0));
}

/* Disable MCDI during cpu_up/cpu_down period */
static int mcdi_cpu_callback(struct notifier_block *nfb,
				   unsigned long action, void *hcpu)
{
	switch (action) {
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
	case CPU_DOWN_PREPARE:
	case CPU_DOWN_PREPARE_FROZEN:
		mcdi_pause(true);
		break;
	}

	return NOTIFY_OK;
}

static int mcdi_cpu_callback_leave_hotplug(struct notifier_block *nfb,
				   unsigned long action, void *hcpu)
{
	switch (action) {
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
	case CPU_UP_CANCELED:
	case CPU_UP_CANCELED_FROZEN:
	case CPU_DOWN_FAILED:
	case CPU_DOWN_FAILED_FROZEN:
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		mcdi_avail_cpu_cluster_update();

		mcdi_pause(false);

		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block mcdi_cpu_notifier = {
	.notifier_call = mcdi_cpu_callback,
	.priority   = INT_MAX,
};

static struct notifier_block mcdi_cpu_notifier_leave_hotplug = {
	.notifier_call = mcdi_cpu_callback_leave_hotplug,
	.priority   = INT_MIN,
};

static int mcdi_hotplug_cb_init(void)
{
	register_cpu_notifier(&mcdi_cpu_notifier);
	register_cpu_notifier(&mcdi_cpu_notifier_leave_hotplug);

	return 0;
}

static void __init mcdi_pm_qos_init(void)
{
}

/* set cluster threshold and enable cluster counter */
static void mcdi_enable_mcupm_cluster_counter(void)
{
#ifdef MCDI_CLUSTER_COUNTER
	mcdi_write(MCUPM_CFGREG_MP0_SLEEP_TH, (MCDI_CLUSTER_THRESHOLD | (1 << 31)));
#endif
}

static int __init mcdi_init(void)
{
	/* Activate MCDI after SMP */
	pr_debug("mcdi_init\n");

	/* of init */
	mcdi_of_init();

	/* Register CPU up/down callbacks */
	mcdi_hotplug_cb_init();

	/* debugfs init */
	mcdi_debugfs_init();

	/* MCDI governor init */
	mcdi_governor_init();

	/* MCDI sysram space init */
	mcdi_sysram_init();

	mcdi_pm_qos_init();

	mcdi_enable_mcupm_cluster_counter();

	return 0;
}

late_initcall(mcdi_init);
