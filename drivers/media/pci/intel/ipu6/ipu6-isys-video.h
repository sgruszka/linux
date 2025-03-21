/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2013--2024 Intel Corporation */

#ifndef IPU6_ISYS_VIDEO_H
#define IPU6_ISYS_VIDEO_H

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/container_of.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include <media/media-entity.h>
#include <media/v4l2-dev.h>

#include "../ipu/ipu-isys.h"

struct video_stream_watermark {
	u32 width;
	u32 height;
	u32 hblank;
	u32 frame_rate;
	u64 pixel_rate;
	u64 stream_data_rate;
	u16 sram_gran_shift;
	u16 sram_gran_size;
	struct list_head stream_node;
};

struct ipu6_isys_video  {
	struct ipu_isys_video ipu;

	struct video_stream_watermark watermark;
	u32 source_stream;
};

#define ipu_isys_queue_to_video(__aq) \
	container_of(__aq, struct ipu_isys_video, aq)

extern const struct ipu_isys_pixelformat ipu6_isys_pfmts[];
extern const struct ipu_isys_pixelformat ipu6_isys_pfmts_packed[];

const struct ipu_isys_pixelformat *
ipu6_isys_get_isys_format(u32 pixelformat, u32 code);
int ipu6_isys_video_prepare_stream(struct ipu_isys_video *av,
				   struct media_entity *source_entity,
				   int nr_queues);
int ipu6_isys_video_set_streaming(struct ipu6_isys_video *av, int state,
				  struct ipu_isys_buffer_list *bl);
int ipu6_isys_fw_open(struct ipu6_isys *isys);
void ipu6_isys_fw_close(struct ipu6_isys *isys);
int ipu6_isys_setup_video(struct ipu6_isys_video *av,
			  struct media_entity **source_entity, int *nr_queues);
int ipu6_isys_video_init(struct ipu6_isys_video *av);
void ipu6_isys_video_cleanup(struct ipu6_isys_video *av);
void ipu6_isys_put_stream(struct ipu_isys_stream *stream);
struct ipu_isys_stream *
ipu6_isys_query_stream_by_handle(struct ipu6_isys *isys, u8 stream_handle);
struct ipu_isys_stream *
ipu6_isys_query_stream_by_source(struct ipu_isys *isys, int source, u8 vc);

void ipu6_isys_configure_stream_watermark(struct ipu6_isys_video *av,
					  bool state);
void ipu6_isys_update_stream_watermark(struct ipu6_isys_video *av, bool state);

#endif /* IPU6_ISYS_VIDEO_H */
