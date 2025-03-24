/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2013--2024 Intel Corporation */

#ifndef IPU6_ISYS_QUEUE_H
#define IPU6_ISYS_QUEUE_H

#include <linux/container_of.h>
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/spinlock_types.h>

#include <media/videobuf2-v4l2.h>

#include "../ipu/ipu-isys.h"

#include "ipu6-fw-isys.h"
#include "ipu6-isys-video.h"

struct ipu_isys_stream;

void
ipu6_isys_buf_to_fw_frame_buf(struct ipu6_fw_isys_frame_buff_set_abi *set,
			      struct ipu_isys_stream *stream,
			      struct ipu_isys_buffer_list *bl);
void
ipu6_isys_buf_calc_sequence_time(struct ipu_isys_buffer *ib,
				 struct ipu6_fw_isys_resp_info_abi *info);
void ipu6_isys_queue_buf_done(struct ipu_isys_buffer *ib);
void ipu6_isys_queue_buf_ready(struct ipu_isys_stream *stream, void *info);
int ipu6_isys_queue_init(struct ipu_isys_queue *aq);
#endif /* IPU6_ISYS_QUEUE_H */
