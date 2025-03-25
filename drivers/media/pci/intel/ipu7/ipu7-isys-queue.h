/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#ifndef IPU7_ISYS_QUEUE_H
#define IPU7_ISYS_QUEUE_H

#include <linux/atomic.h>
#include <linux/container_of.h>
#include <linux/list.h>
#include <linux/spinlock_types.h>

#include <media/videobuf2-v4l2.h>

#include "../ipu/ipu-isys.h"

struct ipu_isys_stream;
struct ipu7_insys_resp;
struct ipu7_insys_buffset;

void ipu7_isys_buffer_to_fw_frame_buff(struct ipu7_insys_buffset *set,
				       struct ipu_isys_stream *stream,
				       struct ipu_isys_buffer_list *bl);

void ipu7_isys_queue_buf_done(struct ipu_isys_buffer *ib);
void ipu7_isys_queue_buf_ready(struct ipu_isys_stream *stream, void *info);
int ipu7_isys_queue_init(struct ipu_isys_queue *aq);
#endif /* IPU7_ISYS_QUEUE_H */
