/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#ifndef IPU7_ISYS_VIDEO_H
#define IPU7_ISYS_VIDEO_H

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/container_of.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include <media/media-entity.h>
#include <media/v4l2-dev.h>

#include "../ipu/ipu.h"
#include "ipu7-isys-queue.h"

#define ipu7_isys_queue_to_video(__aq)			\
	container_of(__aq, struct ipu7_isys_video, aq)

extern const struct ipu_isys_pixelformat ipu7_isys_pfmts[];

const struct ipu_isys_pixelformat *
ipu7_isys_get_isys_format(u32 pixelformat, u32 code);
int ipu7_isys_video_prepare_stream(struct ipu7_isys_video *av,
				   struct media_entity *source_entity,
				   int nr_queues);
int ipu7_isys_video_set_streaming(struct ipu7_isys_video *av, int state,
				  struct ipu_isys_buffer_list *bl);
int ipu7_isys_fw_open(struct ipu7_isys *isys);
void ipu7_isys_fw_close(struct ipu7_isys *isys);
int ipu7_isys_setup_video(struct ipu7_isys_video *av,
			  struct media_entity **source_entity, int *nr_queues);
int ipu7_isys_video_init(struct ipu7_isys_video *av);
void ipu7_isys_video_cleanup(struct ipu7_isys_video *av);
void ipu7_isys_put_stream(struct ipu_isys_stream *stream);
struct ipu_isys_stream *
ipu7_isys_query_stream_by_handle(struct ipu7_isys *isys,
				 u8 stream_handle);
struct ipu_isys_stream *
ipu7_isys_query_stream_by_source(struct ipu7_isys *isys, int source, u8 vc);

u32 ipu7_isys_get_format(struct ipu7_isys_video *av);
u32 ipu7_isys_get_data_size(struct ipu7_isys_video *av);
u32 ipu7_isys_get_bytes_per_line(struct ipu7_isys_video *av);
u32 ipu7_isys_get_frame_width(struct ipu7_isys_video *av);
u32 ipu7_isys_get_frame_height(struct ipu7_isys_video *av);

#endif /* IPU7_ISYS_VIDEO_H */
