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

#include <audio_ipi_platform.h>

#include <scp_ipi.h>



bool audio_opendsp_ready(const uint8_t task)
{
	return is_scp_ready(audio_get_opendsp_id(task));
}


uint32_t audio_get_opendsp_id(const uint8_t task)
{
	return AUDIO_OPENDSP_USE_CM4_A;
}


uint32_t audio_get_ipi_id(const uint8_t task)
{
	return IPI_AUDIO;
}



