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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/seq_file.h>
#include <linux/security.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>

#include <fpsgo_common.h>

#include "fpsgo_base.h"
#include "fpsgo_common.h"
#include "fpsgo_usedext.h"
#include "fps_composer.h"
#include "fbt_cpu.h"
#include "fstb.h"
#include "xgf.h"

/*#define FPSGO_COM_DEBUG*/

#ifdef FPSGO_COM_DEBUG
#define FPSGO_COM_TRACE(...)	xgf_trace("fpsgo_com:" __VA_ARGS__)
#else
#define FPSGO_COM_TRACE(...)
#endif

#define COMP_TAG "FPSGO_COMP"

static struct rb_root ui_pid_tree;
static struct rb_root connect_api_tree;
static struct dentry *fpsgo_com_debugfs_dir;

static inline int fpsgo_com_check_is_surfaceflinger(int pid)
{
	struct task_struct *tsk;
	int is_surfaceflinger = FPSGO_COM_IS_RENDER;

	rcu_read_lock();
	tsk = find_task_by_vpid(pid);

	if (tsk)
		get_task_struct(tsk);
	rcu_read_unlock();

	if (!tsk)
		return FPSGO_COM_TASK_NOT_EXIST;

	if (strstr(tsk->comm, "surfaceflinger"))
		is_surfaceflinger = FPSGO_COM_IS_SF;
	put_task_struct(tsk);

	return is_surfaceflinger;
}

struct connect_api_info *fpsgo_com_search_and_add_connect_api_info(int pid,
	unsigned long long buffer_id, int force)
{
	struct rb_node **p = &connect_api_tree.rb_node;
	struct rb_node *parent = NULL;
	struct connect_api_info *tmp = NULL;
	unsigned long long buffer_key;
	int tgid;

	fpsgo_lockprove(__func__);

	tgid = fpsgo_get_tgid(pid);
	buffer_key = (buffer_id & 0xFFFF) | (tgid << 16);
	FPSGO_COM_TRACE("%s key:%X tgid:%d buffer_id:%llu (tgid << 16):%x",
		__func__, buffer_key, tgid, buffer_id, (tgid << 16));
	while (*p) {
		parent = *p;
		tmp = rb_entry(parent, struct connect_api_info, rb_node);

		if (buffer_key < tmp->buffer_key)
			p = &(*p)->rb_left;
		else if (buffer_key > tmp->buffer_key)
			p = &(*p)->rb_right;
		else
			return tmp;
	}

	if (!force)
		return NULL;

	tmp = kzalloc(sizeof(*tmp), GFP_KERNEL);
	if (!tmp)
		return NULL;

	INIT_LIST_HEAD(&(tmp->render_list));

	tmp->pid = pid;
	tmp->tgid = tgid;
	tmp->buffer_id = buffer_id;
	tmp->buffer_key = buffer_key;

	rb_link_node(&tmp->rb_node, parent, p);
	rb_insert_color(&tmp->rb_node, &connect_api_tree);

	return tmp;
}

struct ui_pid_info *fpsgo_com_search_and_add_ui_pid_info(int ui_pid, int force)
{
	struct rb_node **p = &ui_pid_tree.rb_node;
	struct rb_node *parent = NULL;
	struct ui_pid_info *tmp = NULL;

	fpsgo_lockprove(__func__);

	while (*p) {
		parent = *p;
		tmp = rb_entry(parent, struct ui_pid_info, rb_node);

		if (ui_pid < tmp->ui_pid)
			p = &(*p)->rb_left;
		else if (ui_pid > tmp->ui_pid)
			p = &(*p)->rb_right;
		else
			return tmp;
	}

	if (!force)
		return NULL;

	tmp = kzalloc(sizeof(*tmp), GFP_KERNEL);
	if (!tmp)
		return NULL;

	INIT_LIST_HEAD(&(tmp->render_list));

	tmp->ui_pid = ui_pid;
	tmp->render_method = -1;

	rb_link_node(&tmp->rb_node, parent, p);
	rb_insert_color(&tmp->rb_node, &ui_pid_tree);

	return tmp;
}

void fpsgo_base2com_clear_ui_pid_info(void)
{
	struct rb_node *n;
	struct ui_pid_info *iter;

	fpsgo_lockprove(__func__);

	n = rb_first(&ui_pid_tree);

	while (n) {
		iter = rb_entry(n, struct ui_pid_info, rb_node);
		n = rb_next(n);
		rb_erase(&iter->rb_node, &ui_pid_tree);
		kfree(iter);
	}
}

void fpsgo_base2com_delete_ui_pid_info(int ui_pid)
{
	struct ui_pid_info *data;

	if (!ui_pid)
		return;

	fpsgo_lockprove(__func__);

	data = fpsgo_com_search_and_add_ui_pid_info(ui_pid, 0);

	if (!data)
		return;

	if (!list_empty(&data->render_list))
		return;

	rb_erase(&data->rb_node, &ui_pid_tree);
	kfree(data);
}

int fpsgo_com_check_frame_type(int api, int type)
{
	int new_type = 0;

	switch (api) {
	case NATIVE_WINDOW_API_MEDIA:
	case NATIVE_WINDOW_API_CAMERA:
		new_type = BY_PASS_TYPE;
		return new_type;
	default:
		break;
	}
	return type;
}

int fpsgo_com_update_render_api_info(struct render_info *f_render)
{
	struct connect_api_info *connect_api;
	int new_type;

	fpsgo_lockprove(__func__);
	fpsgo_thread_lockprove(__func__, &(f_render->thr_mlock));

	connect_api =
		fpsgo_com_search_and_add_connect_api_info(f_render->pid, f_render->buffer_id, 0);

	if (!connect_api) {
		FPSGO_COM_TRACE("no pair connect api pid[%d] buffer_id[%llu]",
			f_render->pid, f_render->buffer_id);
		return 0;
	}

	f_render->api = connect_api->api;
	list_add(&(f_render->bufferid_list), &(connect_api->render_list));
	FPSGO_COM_TRACE("add connect api pid[%d] key[%X] buffer_id[%llu]",
		connect_api->pid, connect_api->buffer_key, connect_api->buffer_id);
	new_type = fpsgo_com_check_frame_type(f_render->api, f_render->frame_type);
	f_render->frame_type = new_type;
	return 1;
}

void fpsgo_ctrl2comp_enqueue_start(int pid,
	unsigned long long enqueue_start_time, unsigned long long buffer_id, int queue_SF)
{
	struct render_info *f_render;
	int xgf_ret = 0;
	int check_render;
	unsigned long long slptime = 0;

	FPSGO_COM_TRACE("%s pid[%d] buffer_id %llu", __func__, pid, buffer_id);

	if (!queue_SF)
		return;

	check_render = fpsgo_com_check_is_surfaceflinger(pid);

	if (check_render != FPSGO_COM_IS_RENDER)
		return;

	fpsgo_render_tree_lock(__func__);

	f_render = fpsgo_search_and_add_render_info(pid, 1);

	if (!f_render) {
		fpsgo_render_tree_unlock(__func__);
		FPSGO_COM_TRACE("%s: store frame info fail : %d !!!!\n",
			__func__, pid);
		return;
	}

	fpsgo_thread_lock(&f_render->thr_mlock);

	if (!f_render->api && buffer_id) {
		int ret;

		f_render->buffer_id = buffer_id;
		ret = fpsgo_com_update_render_api_info(f_render);
		if (!ret) {
			fpsgo_render_tree_unlock(__func__);
			fpsgo_thread_unlock(&f_render->thr_mlock);
			return;
		}
	}

	fpsgo_render_tree_unlock(__func__);

	switch (f_render->frame_type) {
	case VSYNC_ALIGNED_TYPE:
		f_render->t_enqueue_start = enqueue_start_time;
		FPSGO_COM_TRACE("pid[%d] type[%d] enqueue_s:%llu frame_time:%llu buffer_id:%llu",
			pid, f_render->frame_type, enqueue_start_time, f_render->self_time, buffer_id);
		if (f_render->render_method == GLSURFACE) {
			xgf_ret = fpsgo_comp2xgf_qudeq_notify(pid, XGF_QUEUE_START, &slptime);
			if (xgf_ret != XGF_SLPTIME_OK)
				pr_debug(COMP_TAG"%s xgf_ret:%d", __func__, xgf_ret);
			f_render->sleep_time = slptime;
		}
		fpsgo_comp2fbt_enq_start(f_render, enqueue_start_time);
		break;
	case NON_VSYNC_ALIGNED_TYPE:
		if (f_render->t_enqueue_start) {
			f_render->self_time =
				enqueue_start_time - f_render->t_enqueue_end - f_render->dequeue_length;
			f_render->Q2Q_time = enqueue_start_time - f_render->t_enqueue_start;
		}
		f_render->t_enqueue_start = enqueue_start_time;
		FPSGO_COM_TRACE("pid[%d] type[%d] enqueue_s:%llu frame_time:%llu",
			pid, f_render->frame_type, enqueue_start_time, f_render->self_time);
		FPSGO_COM_TRACE("update pid[%d] tgid[%d] buffer_id:%llu api:%d",
			f_render->pid, f_render->tgid, f_render->buffer_id, f_render->api);
		fpsgo_systrace_c_fbt_gm(-300, f_render->self_time,
			"%d_%d-self_time", pid, f_render->frame_type);
		fpsgo_systrace_c_fbt_gm(-300, f_render->Q2Q_time,
			"%d_%d-Q2Q_time", pid, f_render->frame_type);
		xgf_ret = fpsgo_comp2xgf_qudeq_notify(pid, XGF_QUEUE_START, &slptime);
			if (xgf_ret != XGF_SLPTIME_OK)
				pr_debug(COMP_TAG"%s xgf_ret:%d", __func__, xgf_ret);
			f_render->sleep_time = slptime;
		fpsgo_comp2fbt_enq_start(f_render, enqueue_start_time);
		fpsgo_comp2fbt_frame_start(f_render, enqueue_start_time, slptime);
		fpsgo_comp2fstb_queue_time_update(pid, f_render->frame_type,
			f_render->render_method, enqueue_start_time, f_render->buffer_id, f_render->api);
		break;
	case BY_PASS_TYPE:
		f_render->t_enqueue_start = enqueue_start_time;
		fpsgo_comp2fbt_bypass_enq();
		fpsgo_comp2fstb_queue_time_update(pid, f_render->frame_type,
					0, enqueue_start_time, f_render->buffer_id, f_render->api);
		fpsgo_systrace_c_fbt_gm(-100, 0, "%d-frame_time", pid);
		break;
	default:
		FPSGO_COM_TRACE("type not found pid[%d] type[%d]",
			pid, f_render->frame_type);
		break;
	}
	fpsgo_thread_unlock(&f_render->thr_mlock);
}

void fpsgo_ctrl2comp_enqueue_end(int pid,
	unsigned long long enqueue_end_time, unsigned long long buffer_id, int queue_SF)
{
	struct render_info *f_render;
	int xgf_ret = 0;
	int check_render;

	FPSGO_COM_TRACE("%s pid[%d]", __func__, pid);

	if (!queue_SF)
		return;

	check_render = fpsgo_com_check_is_surfaceflinger(pid);

	if (check_render != FPSGO_COM_IS_RENDER)
		return;


	fpsgo_render_tree_lock(__func__);

	f_render = fpsgo_search_and_add_render_info(pid, 0);

	if (!f_render) {
		fpsgo_render_tree_unlock(__func__);
		FPSGO_COM_TRACE("%s: NON pair frame info : %d !!!!\n",
			__func__, pid);
		return;
	}

	fpsgo_thread_lock(&f_render->thr_mlock);
	fpsgo_render_tree_unlock(__func__);
	switch (f_render->frame_type) {
	case VSYNC_ALIGNED_TYPE:
		f_render->t_enqueue_end = enqueue_end_time;
		f_render->enqueue_length =
			enqueue_end_time - f_render->t_enqueue_start;
		FPSGO_COM_TRACE("pid[%d] type[%d] enqueue_e:%llu enqueue_l:%llu",
			pid, f_render->frame_type, enqueue_end_time, f_render->enqueue_length);
		if (f_render->render_method == GLSURFACE) {
			xgf_ret = fpsgo_comp2xgf_qudeq_notify(pid, XGF_QUEUE_END, NULL);
			if (xgf_ret != XGF_NOTIFY_OK)
				pr_debug(COMP_TAG"%s xgf_ret:%d", __func__, xgf_ret);
		}
		fpsgo_comp2fbt_enq_end(f_render, enqueue_end_time);
		break;
	case NON_VSYNC_ALIGNED_TYPE:
		f_render->t_enqueue_end = enqueue_end_time;
		f_render->enqueue_length =
			enqueue_end_time - f_render->t_enqueue_start;
		FPSGO_COM_TRACE("pid[%d] type[%d] enqueue_e:%llu enqueue_l:%llu",
			pid, f_render->frame_type, enqueue_end_time, f_render->enqueue_length);
		xgf_ret = fpsgo_comp2xgf_qudeq_notify(pid, XGF_QUEUE_END, NULL);
		if (xgf_ret != XGF_NOTIFY_OK)
			pr_debug(COMP_TAG"%s xgf_ret:%d", __func__, xgf_ret);
		fpsgo_comp2fbt_enq_end(f_render, enqueue_end_time);
		fpsgo_systrace_c_fbt_gm(-300, f_render->enqueue_length,
			"%d_%d-enqueue_length", pid, f_render->frame_type);
		break;
	case BY_PASS_TYPE:
		break;
	default:
		FPSGO_COM_TRACE("type not found pid[%d] type[%d]",
			pid, f_render->frame_type);
		break;
	}
	fpsgo_thread_unlock(&f_render->thr_mlock);

}

void fpsgo_ctrl2comp_dequeue_start(int pid,
	unsigned long long dequeue_start_time, unsigned long long buffer_id, int queue_SF)
{
	struct render_info *f_render;
	int xgf_ret = 0;
	int check_render;

	if (!queue_SF)
		return;

	check_render = fpsgo_com_check_is_surfaceflinger(pid);

	if (check_render != FPSGO_COM_IS_RENDER)
		return;

	FPSGO_COM_TRACE("%s pid[%d]", __func__, pid);

	fpsgo_render_tree_lock(__func__);

	f_render = fpsgo_search_and_add_render_info(pid, 0);

	if (!f_render) {
		fpsgo_render_tree_unlock(__func__);
		FPSGO_COM_TRACE("%s: NON pair frame info : %d !!!!\n",
			__func__, pid);
		return;
	}

	fpsgo_thread_lock(&f_render->thr_mlock);
	fpsgo_render_tree_unlock(__func__);

	switch (f_render->frame_type) {
	case VSYNC_ALIGNED_TYPE:
		f_render->t_dequeue_start = dequeue_start_time;
		FPSGO_COM_TRACE("pid[%d] type[%d] dequeue_s:%llu",
			pid, f_render->frame_type, dequeue_start_time);
		if (f_render->render_method == GLSURFACE) {
			xgf_ret = fpsgo_comp2xgf_qudeq_notify(pid, XGF_DEQUEUE_START, NULL);
			if (xgf_ret != XGF_NOTIFY_OK)
				pr_debug(COMP_TAG"%s xgf_ret:%d", __func__, xgf_ret);
		}
		fpsgo_comp2fbt_deq_start(f_render, dequeue_start_time);
		break;
	case NON_VSYNC_ALIGNED_TYPE:
		f_render->t_dequeue_start = dequeue_start_time;
		FPSGO_COM_TRACE("pid[%d] type[%d] dequeue_s:%llu",
			pid, f_render->frame_type, dequeue_start_time);
		xgf_ret = fpsgo_comp2xgf_qudeq_notify(pid, XGF_DEQUEUE_START, NULL);
		if (xgf_ret != XGF_NOTIFY_OK)
			pr_debug(COMP_TAG"%s xgf_ret:%d", __func__, xgf_ret);
		fpsgo_comp2fbt_deq_start(f_render, dequeue_start_time);
		break;
	case BY_PASS_TYPE:
		break;
	default:
		FPSGO_COM_TRACE("type not found pid[%d] type[%d]",
			pid, f_render->frame_type);
		break;
	}
	fpsgo_thread_unlock(&f_render->thr_mlock);

}

void fpsgo_ctrl2comp_dequeue_end(int pid,
	unsigned long long dequeue_end_time, unsigned long long buffer_id, int queue_SF)
{
	struct render_info *f_render;
	int xgf_ret = 0;
	int check_render;

	FPSGO_COM_TRACE("%s pid[%d]", __func__, pid);

	if (!queue_SF)
		return;

	check_render = fpsgo_com_check_is_surfaceflinger(pid);

	if (check_render != FPSGO_COM_IS_RENDER)
		return;


	fpsgo_render_tree_lock(__func__);

	f_render = fpsgo_search_and_add_render_info(pid, 0);

	if (!f_render) {
		fpsgo_render_tree_unlock(__func__);
		FPSGO_COM_TRACE("%s: NON pair frame info : %d !!!!\n",
			__func__, pid);
		return;
	}

	fpsgo_thread_lock(&f_render->thr_mlock);
	fpsgo_render_tree_unlock(__func__);

	switch (f_render->frame_type) {
	case VSYNC_ALIGNED_TYPE:
		f_render->t_dequeue_end = dequeue_end_time;
		f_render->dequeue_length =
			dequeue_end_time - f_render->t_dequeue_start;
		FPSGO_COM_TRACE("pid[%d] type[%d] dequeue_e:%llu dequeue_l:%llu",
			pid, f_render->frame_type, dequeue_end_time, f_render->dequeue_length);
		if (f_render->render_method == GLSURFACE) {
			xgf_ret = fpsgo_comp2xgf_qudeq_notify(pid, XGF_DEQUEUE_END, NULL);
			if (xgf_ret != XGF_NOTIFY_OK)
				pr_debug(COMP_TAG"%s xgf_ret:%d", __func__, xgf_ret);
		}
		fpsgo_comp2fbt_deq_end(f_render, dequeue_end_time);
		break;
	case NON_VSYNC_ALIGNED_TYPE:
		f_render->t_dequeue_end = dequeue_end_time;
		f_render->dequeue_length =
			dequeue_end_time - f_render->t_dequeue_start;
		FPSGO_COM_TRACE("pid[%d] type[%d] dequeue_e:%llu dequeue_l:%llu",
			pid, f_render->frame_type, dequeue_end_time, f_render->dequeue_length);
		xgf_ret = fpsgo_comp2xgf_qudeq_notify(pid, XGF_DEQUEUE_END, NULL);
		if (xgf_ret != XGF_NOTIFY_OK)
			pr_debug(COMP_TAG"%s xgf_ret:%d", __func__, xgf_ret);
		fpsgo_comp2fbt_deq_end(f_render, dequeue_end_time);
		fpsgo_systrace_c_fbt_gm(-300, f_render->dequeue_length,
			"%d_%d-dequeue_length", pid, f_render->frame_type);
		break;
	case BY_PASS_TYPE:
		break;
	default:
		FPSGO_COM_TRACE("type not found pid[%d] type[%d]",
			pid, f_render->frame_type);
		break;
	}
	fpsgo_thread_unlock(&f_render->thr_mlock);

}

void fpsgo_ctrl2comp_vysnc_aligned_frame_start(int pid,
	unsigned long long t_frame_start, unsigned long long id)
{
	struct ui_pid_info *ui;
	struct render_info *pos, *next;
	int i;
	int check_render;

	FPSGO_COM_TRACE("%s pid[%d] id[%llu]", __func__, pid, id);

	check_render = fpsgo_com_check_is_surfaceflinger(pid);

	if (check_render != FPSGO_COM_IS_RENDER)
		return;

	fpsgo_render_tree_lock(__func__);

	ui = fpsgo_com_search_and_add_ui_pid_info(pid, 0);

	if (!ui) {
		fpsgo_render_tree_unlock(__func__);
		FPSGO_COM_TRACE("%s: NON pair frame info : %d !!!!\n", __func__, pid);
		return;
	}

	list_for_each_entry_safe(pos, next, &ui->render_list, ui_list) {
		fpsgo_thread_lock(&pos->thr_mlock);
		if (pos->frame_type != BY_PASS_TYPE) {
			if (pos->render_method != GLSURFACE) {
				for (i = 0; i < 2; i++) {
					if (pos->frame_id[i].complete == 1) {
						pos->frame_id[i].id = id;
						pos->frame_id[i].complete = 0;
						pos->frame_id[i].draw = 0;
						FPSGO_COM_TRACE("%s [%d] pid[%d] id[%llu] complete[%d] draw[%d]",
							__func__, i, pos->pid, pos->frame_id[i].id,
							pos->frame_id[i].complete, pos->frame_id[i].draw);
						break;
					}
				}
			}
			fpsgo_systrace_c_fbt_gm(-300, pos->self_time,
				"%d_%d-frame_time", pos->pid, pos->frame_type);
			fpsgo_comp2fbt_frame_start(pos, t_frame_start, pos->sleep_time);
		}
		fpsgo_thread_unlock(&pos->thr_mlock);
	}
	fpsgo_render_tree_unlock(__func__);
}

void fpsgo_ctrl2comp_vysnc_aligned_no_render(int pid,
	int render, unsigned long long t_frame_done, unsigned long long id)
{
	struct ui_pid_info *ui;
	struct render_info *pos, *next;
	int i;
	int check_render;

	FPSGO_COM_TRACE("%s pid[%d] render[%d] id[%llu]", __func__, pid, render, id);

	check_render = fpsgo_com_check_is_surfaceflinger(pid);

	if (check_render != FPSGO_COM_IS_RENDER)
		return;

	fpsgo_render_tree_lock(__func__);

	ui = fpsgo_com_search_and_add_ui_pid_info(pid, 0);

	if (!ui) {
		fpsgo_render_tree_unlock(__func__);
		FPSGO_COM_TRACE("%s: NON pair frame info : %d !!!!\n", __func__, pid);
		return;
	}

	list_for_each_entry_safe(pos, next, &ui->render_list, ui_list) {
		fpsgo_thread_lock(&pos->thr_mlock);
		if (pos->frame_type != BY_PASS_TYPE) {
			pos->render = render;
			if (pos->render_method != GLSURFACE) {
				for (i = 0; i < 2; i++) {
					if (pos->frame_id[i].id == id &&
						pos->frame_id[i].complete == 0) {
						pos->frame_id[i].complete = 1;
						FPSGO_COM_TRACE("%s [%d] pid[%d] id[%llu] complete[%d] draw[%d]",
							__func__, i, pos->pid, pos->frame_id[i].id,
							pos->frame_id[i].complete, pos->frame_id[i].draw);
						break;
					}
				}
			}
			fpsgo_systrace_c_fbt_gm(-300, render, "%d_%d-render", pos->pid, pos->frame_type);
			FPSGO_COM_TRACE("%s pid[%d] ui_pid[%d] type[%d] render[%d]",
				__func__, pos->pid, pos->ui_pid, pos->frame_type, render);
			fpsgo_comp2fbt_frame_complete(pos, t_frame_done);
		}
		fpsgo_thread_unlock(&pos->thr_mlock);
	}
	fpsgo_render_tree_unlock(__func__);

}

void fpsgo_ctrl2comp_vysnc_aligned_draw_start(int pid, unsigned long long id)
{
	struct ui_pid_info *ui;
	struct render_info *pos, *next;
	int i;
	int check_render;

	FPSGO_COM_TRACE("%s pid[%d] id[%llu]", __func__, pid, id);

	check_render = fpsgo_com_check_is_surfaceflinger(pid);

	if (check_render != FPSGO_COM_IS_RENDER)
		return;

	fpsgo_render_tree_lock(__func__);

	ui = fpsgo_com_search_and_add_ui_pid_info(pid, 0);

	if (!ui) {
		fpsgo_render_tree_unlock(__func__);
		FPSGO_COM_TRACE("%s: NON pair frame info : %d !!!!\n", __func__, pid);
		return;
	}

	list_for_each_entry_safe(pos, next, &ui->render_list, ui_list) {
		fpsgo_thread_lock(&pos->thr_mlock);
		if (pos->frame_type != BY_PASS_TYPE && pos->render_method != GLSURFACE) {
			for (i = 0; i < 2; i++) {
				if (pos->frame_id[i].id == id &&
					pos->frame_id[i].complete == 0) {
					pos->frame_id[i].draw++;
					FPSGO_COM_TRACE("%s pid[%d] id[%llu] complete[%d] draw[%d]",
						__func__, pos->pid, pos->frame_id[i].id,
						pos->frame_id[i].complete, pos->frame_id[i].draw);
					break;
				}
			}
		}
		fpsgo_thread_unlock(&pos->thr_mlock);
	}
	fpsgo_render_tree_unlock(__func__);

}

void fpsgo_ctrl2comp_vysnc_aligned_frame_done(int pid,
	int ui_pid, unsigned long long frame_time, int render,
	unsigned long long t_frame_done, int render_method, unsigned long long id)
{
	struct render_info *f_render;
	int i;
	int check_render;
	struct ui_pid_info *ui_info;
	struct render_info *pos, *next;
	int temp_pid;

	FPSGO_COM_TRACE("%s pid[%d] ui_pid[%d] render[%d] id[%llu]", __func__, pid, ui_pid, render, id);

	check_render = fpsgo_com_check_is_surfaceflinger(ui_pid);

	if (check_render != FPSGO_COM_IS_RENDER)
		return;

	fpsgo_render_tree_lock(__func__);


	ui_info = fpsgo_com_search_and_add_ui_pid_info(ui_pid, 0);

	if (ui_info && ui_info->render_method != render_method
		&& (!list_empty(&ui_info->render_list))) {
		list_for_each_entry_safe(pos, next,
			&ui_info->render_list, ui_list) {
			fpsgo_thread_lock(&pos->thr_mlock);
			if (!pos->frame_id[0].complete ||
				!pos->frame_id[1].complete) {
				pos->frame_id[0].complete = 1;
				pos->frame_id[1].complete = 1;
				fpsgo_comp2fbt_frame_complete(pos,
					t_frame_done);
			}
			temp_pid = pos->pid;
			fpsgo_comp2fstb_queue_time_update(pos->pid,
				pos->frame_type,
				pos->render_method, t_frame_done,
				pos->buffer_id, pos->api);
			fpsgo_thread_unlock(&pos->thr_mlock);

			FPSGO_COM_TRACE("%s del pid[%d]", __func__, temp_pid);
			fpsgo_delete_render_info(temp_pid);
		}
		FPSGO_COM_TRACE("%s del ui_pid[%d]", __func__, ui_pid);
		fpsgo_base2com_delete_ui_pid_info(ui_pid);
		fpsgo_render_tree_unlock(__func__);
		return;
	}
	if (ui_info)
		ui_info->render_method = render_method;

	f_render = fpsgo_search_and_add_render_info(pid, 1);

	if (!f_render) {
		fpsgo_render_tree_unlock(__func__);
		FPSGO_COM_TRACE("%s: store frame info fail : %d !!!!\n",
			__func__, pid);
		return;
	}

	fpsgo_thread_lock(&f_render->thr_mlock);

	if (ui_pid != f_render->ui_pid) {
		struct ui_pid_info *ui;

		if (!list_empty(&f_render->ui_list)) {
			list_del_init(&(f_render->ui_list));
			pr_debug(COMP_TAG"%s ui_pid:%d f_render->ui_pid:%d",
				__func__, ui_pid, f_render->ui_pid);
		}

		ui = fpsgo_com_search_and_add_ui_pid_info(ui_pid, 1);

		if (!ui) {
			fpsgo_render_tree_unlock(__func__);
			fpsgo_thread_unlock(&f_render->thr_mlock);
			FPSGO_COM_TRACE("%s: store ui info fail : %d !!!!\n",
				__func__, pid);
			return;
		}
		ui->render_method = render_method;

		f_render->frame_type = VSYNC_ALIGNED_TYPE;
		f_render->ui_pid = ui_pid;
		f_render->frame_id[0].complete = 1;
		f_render->frame_id[1].complete = 1;
		list_add(&(f_render->ui_list), &(ui->render_list));
		FPSGO_COM_TRACE("%s: add ui info : ui_pid:%d !!!!\n",
				__func__, ui_pid);
	}

	if (!f_render->api && f_render->buffer_id) {
		int ret;

		ret = fpsgo_com_update_render_api_info(f_render);
		if (!ret) {
			fpsgo_render_tree_unlock(__func__);
			fpsgo_thread_unlock(&f_render->thr_mlock);
			return;
		}
	}

	fpsgo_render_tree_unlock(__func__);

	if (f_render->frame_type == BY_PASS_TYPE) {
		fpsgo_thread_unlock(&f_render->thr_mlock);
		return;
	}

	f_render->render = render;
	f_render->render_method = render_method;
	if (render) {
		int frame_done = 0;

		f_render->self_time = frame_time;
		f_render->Q2Q_time = frame_time;

		if (f_render->render_method == GLSURFACE) {
			fpsgo_comp2fbt_frame_complete(f_render, t_frame_done);
			goto out;
		}

		for (i = 0; i < 2; i++) {
			if (f_render->frame_id[i].id == id &&
				f_render->frame_id[i].complete == 0) {
				if (f_render->frame_id[i].draw > 1)
					f_render->frame_id[i].draw--;
				else
					f_render->frame_id[i].complete = 1;
				frame_done = f_render->frame_id[i].complete;
				FPSGO_COM_TRACE("%s [%d] pid[%d] id[%llu] complete[%d] draw[%d]",
					__func__, i, f_render->pid, f_render->frame_id[i].id,
					f_render->frame_id[i].complete, f_render->frame_id[i].draw);
				fpsgo_systrace_c_fbt_gm(-300, f_render->frame_id[i].complete,
						"%d_%d-complete", f_render->pid, f_render->frame_type);
				break;
			}
		}
		if (frame_done == 1)
			fpsgo_comp2fbt_frame_complete(f_render, t_frame_done);
	}

out:
	fpsgo_comp2fstb_queue_time_update(pid, f_render->frame_type,
			f_render->render_method, t_frame_done, f_render->buffer_id, f_render->api);
	fpsgo_systrace_c_fbt_gm(-300, render, "%d_%d-render", pid, f_render->frame_type);
	FPSGO_COM_TRACE("frame_done pid[%d] ui[%d] type[%d] frame_t:%llu render:%d method:%d",
		pid, f_render->ui_pid, f_render->frame_type, frame_time, render, render_method);
	fpsgo_thread_unlock(&f_render->thr_mlock);

}

void fpsgo_ctrl2comp_connect_api(int pid, unsigned long long buffer_id, int api)
{
	struct connect_api_info *connect_api;
	int check_render;

	check_render = fpsgo_com_check_is_surfaceflinger(pid);

	if (check_render != FPSGO_COM_IS_RENDER)
		return;

	FPSGO_COM_TRACE("%s pid[%d]", __func__, pid);

	fpsgo_render_tree_lock(__func__);
	connect_api = fpsgo_com_search_and_add_connect_api_info(pid, buffer_id, 1);
	if (!connect_api) {
		fpsgo_render_tree_unlock(__func__);
		FPSGO_COM_TRACE("%s: store frame info fail : %d !!!!\n",
			__func__, pid);
		return;
	}

	connect_api->api = api;
	fpsgo_render_tree_unlock(__func__);

}

void fpsgo_com_clear_connect_api_render_list(struct connect_api_info *connect_api)
{
	struct render_info *pos, *next;

	fpsgo_lockprove(__func__);

	list_for_each_entry_safe(pos, next, &connect_api->render_list, bufferid_list) {
		int ui_pid = pos->ui_pid;

		fpsgo_delete_render_info(pos->pid);
		fpsgo_base2com_delete_ui_pid_info(ui_pid);
	}

}

void fpsgo_ctrl2comp_disconnect_api(int pid, unsigned long long buffer_id, int api)
{
	struct connect_api_info *connect_api;
	int check_render;

	check_render = fpsgo_com_check_is_surfaceflinger(pid);

	if (check_render != FPSGO_COM_IS_RENDER)
		return;


	FPSGO_COM_TRACE("%s pid[%d]", __func__, pid);

	fpsgo_render_tree_lock(__func__);

	connect_api = fpsgo_com_search_and_add_connect_api_info(pid, buffer_id, 0);
	if (!connect_api) {
		FPSGO_COM_TRACE("%s: FPSGo composer distory connect api fail : %d !!!!\n",
			__func__, pid);
		fpsgo_render_tree_unlock(__func__);
		return;
	}
	fpsgo_com_clear_connect_api_render_list(connect_api);
	rb_erase(&connect_api->rb_node, &connect_api_tree);
	kfree(connect_api);

	fpsgo_comp2fbt_bypass_disconnect();

	fpsgo_render_tree_unlock(__func__);
}

void fpsgo_fstb2comp_check_connect_api(void)
{
	struct rb_node *n;
	struct connect_api_info *iter;
	struct task_struct *tsk;

	FPSGO_COM_TRACE("%s ", __func__);

	fpsgo_render_tree_lock(__func__);

	n = rb_first(&connect_api_tree);

	while (n) {
		iter = rb_entry(n, struct connect_api_info, rb_node);
		rcu_read_lock();
		tsk = find_task_by_vpid(iter->tgid);
		rcu_read_unlock();
		if (!tsk) {
			fpsgo_com_clear_connect_api_render_list(iter);
			rb_erase(&iter->rb_node, &connect_api_tree);
			n = rb_first(&connect_api_tree);
			kfree(iter);
		} else
			n = rb_next(n);
	}

	fpsgo_render_tree_unlock(__func__);

}

#define FPSGO_COM_DEBUGFS_ENTRY(name) \
static int fspgo_com_##name##_open(struct inode *i, struct file *file) \
{ \
	return single_open(file, fspgo_com_##name##_show, i->i_private); \
} \
\
static const struct file_operations fspgo_com_##name##_fops = { \
	.owner = THIS_MODULE, \
	.open = fspgo_com_##name##_open, \
	.read = seq_read, \
	.write = fspgo_com_##name##_write, \
	.llseek = seq_lseek, \
	.release = single_release, \
}

static int fspgo_com_connect_api_info_show
	(struct seq_file *m, void *unused)
{
	struct rb_node *n;
	struct connect_api_info *iter;
	struct task_struct *tsk;
	struct render_info *pos, *next;
	struct ui_pid_info *ui_iter;

	seq_puts(m, "=================================\n");

	fpsgo_render_tree_lock(__func__);
	rcu_read_lock();

	for (n = rb_first(&connect_api_tree); n != NULL; n = rb_next(n)) {
		iter = rb_entry(n, struct connect_api_info, rb_node);
		tsk = find_task_by_vpid(iter->tgid);
		if (tsk) {
			get_task_struct(tsk);
			seq_puts(m, "PID  TGID  NAME    BufferID    API    Key\n");
			seq_printf(m, "%5d %5d %5s %4llu %5d %5X\n",
			iter->pid, iter->tgid, tsk->comm, iter->buffer_id, iter->api, iter->buffer_key);
			put_task_struct(tsk);
		}
		seq_puts(m, "******render list******\n");
		list_for_each_entry_safe(pos, next, &iter->render_list, bufferid_list) {
			fpsgo_thread_lock(&pos->thr_mlock);
			seq_puts(m, "  PID  TGID	 BufferID	API    TYPE\n");
			seq_printf(m, "%5d %5d %4llu %5d %5d\n",
			pos->pid, pos->tgid, pos->buffer_id, pos->api, pos->frame_type);
			fpsgo_thread_unlock(&pos->thr_mlock);
		}
		seq_puts(m, "***********************\n");
		seq_puts(m, "=================================\n");
	}

	n = rb_first(&ui_pid_tree);

	while (n) {
		struct render_info *pos, *next;

		ui_iter = rb_entry(n, struct ui_pid_info, rb_node);
		seq_printf(m, "%5d:", ui_iter->ui_pid);
		list_for_each_entry_safe(pos, next,
				&ui_iter->render_list, ui_list)
			seq_printf(m, "[%d]", pos->pid);
		seq_puts(m, "\n***********************\n");
		n = rb_next(n);
	}

	rcu_read_unlock();
	fpsgo_render_tree_unlock(__func__);

	return 0;

}

static ssize_t fspgo_com_connect_api_info_write(struct file *flip,
			const char *ubuf, size_t cnt, loff_t *data)
{
	int val;
	int ret;

	ret = kstrtoint_from_user(ubuf, cnt, 0, &val);
	if (ret)
		return ret;

	return cnt;
}

FPSGO_COM_DEBUGFS_ENTRY(connect_api_info);

void __exit fpsgo_composer_exit(void)
{

}

int __init fpsgo_composer_init(void)
{
	ui_pid_tree = RB_ROOT;
	connect_api_tree = RB_ROOT;

	if (fpsgo_debugfs_dir) {
		fpsgo_com_debugfs_dir = debugfs_create_dir("composer", fpsgo_debugfs_dir);

		if (fpsgo_com_debugfs_dir) {
			debugfs_create_file("connect_api_info",
					S_IRUGO | S_IWUSR | S_IWGRP,
					fpsgo_com_debugfs_dir,
					NULL,
					&fspgo_com_connect_api_info_fops);
		}
	}

	return 0;
}

