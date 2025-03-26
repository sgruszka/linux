// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#include <linux/atomic.h>
#include <linux/bug.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/lockdep.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include <media/media-entity.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-sg.h>
#include <media/videobuf2-v4l2.h>

#include "abi/ipu7_fw_isys_abi.h"

#include "ipu7-buttress.h"
#include "ipu7-dma.h"
#include "ipu7-fw-isys.h"
#include "ipu7-isys.h"
#include "ipu7-isys-video.h"

#define IPU_MAX_FRAME_COUNTER	(U8_MAX + 1)

static int ipu7_isys_buf_init(struct vb2_buffer *vb)
{
	struct ipu7_isys *isys = vb2_get_drv_priv(vb->vb2_queue);
	struct sg_table *sg = vb2_dma_sg_plane_desc(vb, 0);
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct ipu_isys_video_buffer *ivb =
		vb2_buffer_to_ipu_isys_video_buffer(vvb);
	int ret;

	ret = ipu7_dma_map_sgtable(isys->ipu.adev, sg, DMA_TO_DEVICE, 0);
	if (ret)
		return ret;

	ivb->dma_addr = sg_dma_address(sg->sgl);

	return 0;
}

static void ipu7_isys_buf_cleanup(struct vb2_buffer *vb)
{
	struct ipu7_isys *isys = vb2_get_drv_priv(vb->vb2_queue);
	struct sg_table *sg = vb2_dma_sg_plane_desc(vb, 0);
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct ipu_isys_video_buffer *ivb =
		vb2_buffer_to_ipu_isys_video_buffer(vvb);

	ivb->dma_addr = 0;
	ipu7_dma_unmap_sgtable(isys->ipu.adev, sg, DMA_TO_DEVICE, 0);
}

static void ipu7_isys_buf_to_fw_frame_buf_pin(struct vb2_buffer *vb,
					      struct ipu7_insys_buffset *set)
{
	struct ipu_isys_queue *aq = vb2_queue_to_isys_queue(vb->vb2_queue);
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct ipu_isys_video_buffer *ivb =
		vb2_buffer_to_ipu_isys_video_buffer(vvb);

	set->output_pins[aq->fw_output].addr = ivb->dma_addr;
	set->output_pins[aq->fw_output].user_token = (uintptr_t)set;
}

/*
 * Convert a buffer list to a isys fw ABI framebuffer set. The
 * buffer list is not modified.
 */
void ipu7_isys_buffer_to_fw_frame_buff(struct ipu7_insys_buffset *set,
				       struct ipu_isys_stream *stream,
				       struct ipu_isys_buffer_list *bl)
{
	struct ipu_isys_buffer *ib;
	u32 buf_id;

	WARN_ON(!bl->nbufs);

	set->skip_frame = 0;
	set->capture_msg_map = IPU_INSYS_FRAME_ENABLE_MSG_SEND_RESP |
			       IPU_INSYS_FRAME_ENABLE_MSG_SEND_IRQ;

	/* WHY THIS IS NOT SEQUENCE */
	buf_id = atomic_fetch_inc(&stream->buf_id);
	set->frame_id = buf_id % IPU_MAX_FRAME_COUNTER;

	list_for_each_entry(ib, &bl->head, head) {
		struct vb2_buffer *vb = ipu_isys_buffer_to_vb2_buffer(ib);

		ipu7_isys_buf_to_fw_frame_buf_pin(vb, set);
	}
}

/* Start streaming for real. The buffer list must be available. */
static int ipu7_isys_stream_start(struct ipu_isys_video *av,
				  struct ipu_isys_buffer_list *bl, bool error)
{
	struct ipu_isys_stream *stream = av->stream;
	struct device *dev = isys_to_dev(to_isys7(stream));
	struct ipu_isys_buffer_list __bl;
	int ret;

	mutex_lock(&to_isys(stream)->stream_mutex);
	ret = ipu7_isys_video_set_streaming(av, 1, bl);
	mutex_unlock(&to_isys(stream)->stream_mutex);
	if (ret)
		goto out_requeue;

	stream->streaming = 1;

	bl = &__bl;

	do {
		struct ipu7_insys_buffset *buf = NULL;
		struct isys_fw_msgs *msg;
		enum ipu7_insys_send_type send_type =
			IPU_INSYS_SEND_TYPE_STREAM_CAPTURE;

		ret = ipu_buffer_list_get(stream, bl);
		if (ret < 0)
			break;

		msg = ipu7_get_fw_msg_buf(stream);
		if (!msg)
			return -ENOMEM;

		buf = &msg->fw_msg.frame;

		ipu7_isys_buffer_to_fw_frame_buff(buf, stream, bl);

		ipu7_fw_isys_dump_frame_buff_set(dev, buf,
						 stream->nr_output_pins);

		ipu_isys_buffer_list_queue(bl, IPU_ISYS_BUFFER_LIST_FL_ACTIVE, 0);

		ret = ipu7_fw_isys_complex_cmd(to_isys7(stream),
					       stream->stream_handle, buf,
					       msg->dma_addr, sizeof(*buf),
					       send_type);
	} while (!WARN_ON(ret));

	return 0;

out_requeue:
	if (bl && bl->nbufs)
		ipu_isys_buffer_list_queue(bl,
					   IPU_ISYS_BUFFER_LIST_FL_INCOMING |
					   (error ?
					    IPU_ISYS_BUFFER_LIST_FL_SET_STATE :
					    0), error ? VB2_BUF_STATE_ERROR :
					    VB2_BUF_STATE_QUEUED);
	ipu_flush_firmware_streamon_fail(stream);

	return ret;
}

static void buf_queue(struct vb2_buffer *vb)
{
	struct ipu_isys_queue *aq = vb2_queue_to_isys_queue(vb->vb2_queue);
	struct ipu_isys_video *av = ipu_isys_queue_to_video(aq);
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct ipu_isys_video_buffer *ivb =
		vb2_buffer_to_ipu_isys_video_buffer(vvb);
	struct media_pipeline *media_pipe =
		media_entity_pipeline(&av->vdev.entity);
	struct device *dev = isys_to_dev(to_isys7(av));
	struct ipu_isys_stream *stream = av->stream;
	struct ipu_isys_buffer *ib = &ivb->ib;
	struct ipu7_insys_buffset *buf = NULL;
	struct ipu_isys_buffer_list bl;
	struct isys_fw_msgs *msg;
	unsigned long flags;
	dma_addr_t dma;
	int ret;

	dev_dbg(dev, "queue buffer %u for %s\n", vb->index, av->vdev.name);

	dma = ivb->dma_addr;
	dev_dbg(dev, "iova: iova %pad\n", &dma);

	spin_lock_irqsave(&aq->lock, flags);
	list_add(&ib->head, &aq->incoming);
	spin_unlock_irqrestore(&aq->lock, flags);

	if (!media_pipe || !vb->vb2_queue->start_streaming_called) {
		dev_dbg(dev, "media pipeline is not ready for %s\n",
			av->vdev.name);
		return;
	}

	mutex_lock(&stream->mutex);

	if (stream->nr_streaming != stream->nr_queues) {
		dev_dbg(dev, "not streaming yet, adding to incoming\n");
		goto out;
	}

	/*
	 * We just put one buffer to the incoming list of this queue
	 * (above). Let's see whether all queues in the pipeline would
	 * have a buffer.
	 */
	ret = ipu_buffer_list_get(stream, &bl);
	if (ret < 0) {
		dev_dbg(dev, "No buffers available\n");
		goto out;
	}

	msg = ipu7_get_fw_msg_buf(stream);
	if (!msg) {
		ret = -ENOMEM;
		goto out;
	}

	buf = &msg->fw_msg.frame;

	ipu7_isys_buffer_to_fw_frame_buff(buf, stream, &bl);

	ipu7_fw_isys_dump_frame_buff_set(dev, buf, stream->nr_output_pins);

	if (!stream->streaming) {
		ret = ipu7_isys_stream_start(av, &bl, true);
		if (ret)
			dev_err(dev, "stream start failed.\n");
		goto out;
	}

	/*
	 * We must queue the buffers in the buffer list to the
	 * appropriate video buffer queues BEFORE passing them to the
	 * firmware since we could get a buffer event back before we
	 * have queued them ourselves to the active queue.
	 */
	ipu_isys_buffer_list_queue(&bl, IPU_ISYS_BUFFER_LIST_FL_ACTIVE, 0);

	ret = ipu7_fw_isys_complex_cmd(to_isys7(stream), stream->stream_handle,
				       buf, msg->dma_addr, sizeof(*buf),
				       IPU_INSYS_SEND_TYPE_STREAM_CAPTURE);
	if (ret < 0)
		dev_err(dev, "send stream capture failed\n");

out:
	mutex_unlock(&stream->mutex);
}

static void ipu7_isys_stream_cleanup(struct ipu_isys_video *av)
{
	video_device_pipeline_stop(&av->vdev);
	ipu7_isys_put_stream(av->stream);
	av->stream = NULL;
}

static int start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct ipu_isys_queue *aq = vb2_queue_to_isys_queue(q);
	struct ipu_isys_video *av = ipu_isys_queue_to_video(aq);
	struct device *dev = isys_to_dev(to_isys7(av));
	const struct ipu_isys_pixelformat *pfmt =
		ipu_isys_get_isys_format(to_isys(av), ipu_isys_get_format(av), 0);
	struct ipu_isys_buffer_list __bl, *bl = NULL;
	struct ipu_isys_stream *stream;
	struct media_entity *source_entity = NULL;
	int nr_queues, ret;

	dev_dbg(dev, "stream: %s: width %u, height %u, css pixelformat %u\n",
		av->vdev.name, ipu_isys_get_frame_width(av),
		ipu_isys_get_frame_height(av), pfmt->css_pixelformat);

	ret = ipu7_isys_setup_video(av, &source_entity, &nr_queues);
	if (ret < 0) {
		dev_dbg(dev, "failed to setup video\n");
		goto out_return_buffers;
	}

	ret = ipu_isys_link_fmt_validate(aq);
	if (ret) {
		dev_dbg(dev,
			"%s: link format validation failed (%d)\n",
			av->vdev.name, ret);
		goto out_pipeline_stop;
	}

	stream = av->stream;
	mutex_lock(&stream->mutex);
	if (!stream->nr_streaming) {
		ret = ipu7_isys_video_prepare_stream(av, source_entity,
						     nr_queues);
		if (ret) {
			mutex_unlock(&stream->mutex);
			goto out_pipeline_stop;
		}
	}

	stream->nr_streaming++;
	dev_dbg(dev, "queue %u of %u\n", stream->nr_streaming,
		stream->nr_queues);

	list_add(&aq->node, &stream->queues);

	if (stream->nr_streaming != stream->nr_queues)
		goto out;

	bl = &__bl;
	ret = ipu_buffer_list_get(stream, bl);
	if (ret < 0) {
		dev_warn(dev, "no buffer available, DRIVER BUG?\n");
		goto out;
	}

	ret = ipu7_isys_fw_open(to_isys7(av));
	if (ret)
		goto out_stream_start;

	ret = ipu7_isys_stream_start(av, bl, false);
	if (ret)
		goto out_isys_fw_close;

out:
	mutex_unlock(&stream->mutex);

	return 0;

out_isys_fw_close:
	ipu7_isys_fw_close(to_isys7(av));

out_stream_start:
	list_del(&aq->node);
	stream->nr_streaming--;
	mutex_unlock(&stream->mutex);

out_pipeline_stop:
	ipu7_isys_stream_cleanup(av);

out_return_buffers:
	ipu_return_buffers(aq, VB2_BUF_STATE_QUEUED);

	return ret;
}

static void stop_streaming(struct vb2_queue *q)
{
	struct ipu_isys_queue *aq = vb2_queue_to_isys_queue(q);
	struct ipu_isys_video *av = ipu_isys_queue_to_video(aq);
	struct ipu_isys_stream *stream = av->stream;

	mutex_lock(&stream->mutex);
	mutex_lock(&to_isys(av)->stream_mutex);
	if (stream->nr_streaming == stream->nr_queues && stream->streaming)
		ipu7_isys_video_set_streaming(av, 0, NULL);
	mutex_unlock(&to_isys(av)->stream_mutex);

	stream->nr_streaming--;
	list_del(&aq->node);
	stream->streaming = 0;

	mutex_unlock(&stream->mutex);

	ipu7_isys_stream_cleanup(av);

	ipu_return_buffers(aq, VB2_BUF_STATE_ERROR);

	ipu7_isys_fw_close(to_isys7(av));
}

void ipu7_isys_queue_buf_done(struct ipu_isys_buffer *ib)
{
	struct vb2_buffer *vb = ipu_isys_buffer_to_vb2_buffer(ib);

	if (atomic_read(&ib->str2mmio_flag)) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		/*
		 * Operation on buffer is ended with error and will be reported
		 * to the userspace when it is de-queued
		 */
		atomic_set(&ib->str2mmio_flag, 0);
	} else {
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
	}
}

void ipu7_isys_queue_buf_ready(struct ipu_isys_stream *stream, void *_info)
{
	struct ipu7_insys_resp *info = _info;
	struct ipu_isys_queue *aq = stream->output_pins[info->pin_id].aq;
	u64 time = (u64)info->timestamp[1] << 32 | info->timestamp[0];

	dev_dbg(isys_to_dev(to_isys(stream)),
		"buffer: %s: received buffer %8.8x %d\n",
		ipu_isys_queue_to_video(aq)->vdev.name,
		info->pin.addr, info->frame_id);

	ipu_stream_buf_ready(stream, info->pin_id, info->pin.addr, time, false);
}


static const struct vb2_ops ipu7_isys_queue_ops = {
	.queue_setup = ipu_isys_queue_setup,
	.buf_init = ipu7_isys_buf_init,
	.buf_prepare = ipu_isys_buf_prepare,
	.buf_cleanup = ipu7_isys_buf_cleanup,
	.start_streaming = start_streaming,
	.stop_streaming = stop_streaming,
	.buf_queue = buf_queue,
};

int ipu7_isys_queue_init(struct ipu_isys_queue *aq)
{
	struct ipu_isys *isys = ipu_isys_queue_to_video(aq)->isys;
	struct ipu_isys_video *av = ipu_isys_queue_to_video(aq);
	struct ipu_bus_device *adev = isys->adev;
	int ret;

	if (!aq->vbq.io_modes)
		aq->vbq.io_modes = VB2_MMAP | VB2_DMABUF;

	aq->vbq.drv_priv = isys;
	aq->vbq.ops = &ipu7_isys_queue_ops;
	aq->vbq.lock = &av->mutex;
	aq->vbq.mem_ops = &vb2_dma_sg_memops;
	aq->vbq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	aq->vbq.min_queued_buffers = 1;
	aq->vbq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	ret = vb2_queue_init(&aq->vbq);
	if (ret)
		return ret;

	aq->dev = &adev->auxdev.dev;
	aq->vbq.dev = &adev->isp->pdev->dev;
	spin_lock_init(&aq->lock);
	INIT_LIST_HEAD(&aq->active);
	INIT_LIST_HEAD(&aq->incoming);

	return 0;
}
