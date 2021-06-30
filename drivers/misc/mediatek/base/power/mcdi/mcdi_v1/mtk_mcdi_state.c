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

#include <linux/types.h>

#include <mtk_mcdi.h>
#include <mtk_mcdi_state.h>

/*
 * Used for mcdi_governor
 * only use exit_latency & target_residency
 */
static struct cpuidle_driver mt67xx_acao_mcdi_state[NF_MCDI_STATE_TABLE_TYPE] = {
	[0] = {
		.name             = "mt67xx_acao_mcdi_set_0",
		.owner            = THIS_MODULE,
		.states[0] = {
			.enter              = NULL,
			.exit_latency       = 1,
			.target_residency   = 1,
			.name               = "wfi",
			.desc               = "wfi"
		},
		.states[1] = {
			.enter              = NULL,
			.exit_latency       = 300,
			.target_residency   = 4500,
			.name               = "cpu_off",
			.desc               = "cpu_off",
		},
		.states[2] = {
			.enter              = NULL,
			.exit_latency       = 600,
			.target_residency   = 4500,
			.name               = "cluster_off",
			.desc               = "cluster_off",
		},
		.states[3] = {
			.enter              = NULL,
			.exit_latency       = 1200,
			.target_residency   = 5000,
			.name               = "sodi",
			.desc               = "sodi",
		},
		.states[4] = {
			.enter              = NULL,
			.exit_latency       = 1200,
			.target_residency   = 5000,
			.name               = "dpidle",
			.desc               = "dpidle",
		},
		.states[5] = {
			.enter              = NULL,
			.exit_latency       = 5000,
			.target_residency   = 10500,
			.name               = "sodi3",
			.desc               = "sodi3",
		},
		.state_count = 6,
		.safe_state_index = 0,
	},
	[1] = {
		.name             = "mt67xx_acao_mcdi_set_1",
		.owner            = THIS_MODULE,
		.states[0] = {
			.enter              = NULL,
			.exit_latency       = 1,
			.target_residency   = 1,
			.name               = "wfi",
			.desc               = "wfi"
		},
		.states[1] = {
			.enter              = NULL,
			.exit_latency       = 300,
			.target_residency   = 1500,
			.name               = "cpu_off",
			.desc               = "cpu_off",
		},
		.states[2] = {
			.enter              = NULL,
			.exit_latency       = 600,
			.target_residency   = 1500,
			.name               = "cluster_off",
			.desc               = "cluster_off",
		},
		.states[3] = {
			.enter              = NULL,
			.exit_latency       = 1200,
			.target_residency   = 5000,
			.name               = "sodi",
			.desc               = "sodi",
		},
		.states[4] = {
			.enter              = NULL,
			.exit_latency       = 1200,
			.target_residency   = 5000,
			.name               = "dpidle",
			.desc               = "dpidle",
		},
		.states[5] = {
			.enter              = NULL,
			.exit_latency       = 5000,
			.target_residency   = 10500,
			.name               = "sodi3",
			.desc               = "sodi3",
		},
		.state_count = 6,
		.safe_state_index = 0,
	},
	[2] = {
		.name             = "mt67xx_acao_mcdi_set_2",
		.owner            = THIS_MODULE,
		.states[0] = {
			.enter              = NULL,
			.exit_latency       = 1,
			.target_residency   = 1,
			.name               = "wfi",
			.desc               = "wfi"
		},
		.states[1] = {
			.enter              = NULL,
			.exit_latency       = 300,
			.target_residency   = 850,
			.name               = "cpu_off",
			.desc               = "cpu_off",
		},
		.states[2] = {
			.enter              = NULL,
			.exit_latency       = 600,
			.target_residency   = 1500,
			.name               = "cluster_off",
			.desc               = "cluster_off",
		},
		.states[3] = {
			.enter              = NULL,
			.exit_latency       = 1200,
			.target_residency   = 5000,
			.name               = "sodi",
			.desc               = "sodi",
		},
		.states[4] = {
			.enter              = NULL,
			.exit_latency       = 1200,
			.target_residency   = 5000,
			.name               = "dpidle",
			.desc               = "dpidle",
		},
		.states[5] = {
			.enter              = NULL,
			.exit_latency       = 5000,
			.target_residency   = 10500,
			.name               = "sodi3",
			.desc               = "sodi3",
		},
		.state_count = 6,
		.safe_state_index = 0,
	}
};

struct cpuidle_driver *mcdi_state_tbl_get(int cpu)
{
	int tbl_idx = 0;

	tbl_idx = mcdi_get_state_tbl(cpu);

	return &mt67xx_acao_mcdi_state[tbl_idx];
}

