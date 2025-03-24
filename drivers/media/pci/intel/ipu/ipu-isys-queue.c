#include "ipu-isys.h"

int ipu_isys_buf_prepare(struct vb2_buffer *vb)
{
	struct ipu_isys_queue *aq = vb2_queue_to_isys_queue(vb->vb2_queue);
	struct ipu_isys_video *av = ipu_isys_queue_to_video(aq);
	struct device *dev = isys_to_dev(to_isys(av));
	u32 bytesperline = ipu_isys_get_bytes_per_line(av);
	u32 height = ipu_isys_get_frame_height(av);
	u32 size = ipu_isys_get_data_size(av);

	dev_dbg(dev, "buffer: %s: configured size %u, buffer size %lu\n",
		av->vdev.name, size, vb2_plane_size(vb, 0));

	if (size > vb2_plane_size(vb, 0))
		return -EINVAL;

	dev_dbg(dev, "buffer: %s: bytesperline %u, height %u\n",
		av->vdev.name, bytesperline, height);

	vb2_set_plane_payload(vb, 0, bytesperline * height);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_isys_buf_prepare);

int ipu_isys_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			 unsigned int *num_planes, unsigned int sizes[],
			 struct device *alloc_devs[])
{
	struct ipu_isys_queue *aq = vb2_queue_to_isys_queue(q);
	struct ipu_isys_video *av = ipu_isys_queue_to_video(aq);
	struct device *dev = isys_to_dev(av->isys);
	u32 size = ipu_isys_get_data_size(av);

	/* num_planes == 0: we're being called through VIDIOC_REQBUFS */
	if (!*num_planes) {
		sizes[0] = size;
	} else if (sizes[0] < size) {
		dev_dbg(dev, "%s: queue setup: size %u < %u\n",
			av->vdev.name, sizes[0], size);
		return -EINVAL;
	}

	*num_planes = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_isys_queue_setup);

/*
 * Queue a buffer list back to incoming or active queues. The buffers
 * are removed from the buffer list.
 */
void ipu_isys_buffer_list_queue(struct ipu_isys_buffer_list *bl,
				unsigned long op_flags,
				enum vb2_buffer_state state)
{
	struct ipu_isys_buffer *ib, *ib_safe;
	unsigned long flags;
	bool first = true;

	if (!bl)
		return;

	WARN_ON_ONCE(!bl->nbufs);
	WARN_ON_ONCE(op_flags & IPU_ISYS_BUFFER_LIST_FL_ACTIVE &&
		     op_flags & IPU_ISYS_BUFFER_LIST_FL_INCOMING);

	list_for_each_entry_safe(ib, ib_safe, &bl->head, head) {
		struct ipu_isys_video *av;
		struct vb2_buffer *vb = ipu_isys_buffer_to_vb2_buffer(ib);
		struct ipu_isys_queue *aq =
			vb2_queue_to_isys_queue(vb->vb2_queue);

		av = ipu_isys_queue_to_video(aq);
		spin_lock_irqsave(&aq->lock, flags);
		list_del(&ib->head);
		if (op_flags & IPU_ISYS_BUFFER_LIST_FL_ACTIVE)
			list_add(&ib->head, &aq->active);
		else if (op_flags & IPU_ISYS_BUFFER_LIST_FL_INCOMING)
			list_add_tail(&ib->head, &aq->incoming);
		spin_unlock_irqrestore(&aq->lock, flags);

		if (op_flags & IPU_ISYS_BUFFER_LIST_FL_SET_STATE)
			vb2_buffer_done(vb, state);

		if (first) {
			dev_dbg(isys_to_dev(to_isys(av)),
				"queue buf list %p flags %lx, s %d, %d bufs\n",
				bl, op_flags, state, bl->nbufs);
			first = false;
		}

		bl->nbufs--;
	}

	WARN_ON_ONCE(bl->nbufs);
}
EXPORT_SYMBOL_GPL(ipu_isys_buffer_list_queue);

/*
 * flush_firmware_streamon_fail() - Flush in cases where requests may
 * have been queued to firmware and the *firmware streamon fails for a
 * reason or another.
 */
void ipu_flush_firmware_streamon_fail(struct ipu_isys_stream *stream)
{
	struct device *dev = isys_to_dev(to_isys(stream));
	struct ipu_isys_queue *aq;
	unsigned long flags;

	lockdep_assert_held(&stream->mutex);

	list_for_each_entry(aq, &stream->queues, node) {
		struct ipu_isys_video *av = ipu_isys_queue_to_video(aq);
		struct ipu_isys_buffer *ib, *ib_safe;

		spin_lock_irqsave(&aq->lock, flags);
		list_for_each_entry_safe(ib, ib_safe, &aq->active, head) {
			struct vb2_buffer *vb =
				ipu_isys_buffer_to_vb2_buffer(ib);

			list_del(&ib->head);
			if (av->streaming) {
				dev_dbg(dev,
					"%s: queue buffer %u back to incoming\n",
					av->vdev.name, vb->index);
				/* Queue already streaming, return to driver. */
				list_add(&ib->head, &aq->incoming);
				continue;
			}
			/* Queue not yet streaming, return to user. */
			dev_dbg(dev, "%s: return %u back to videobuf2\n",
				av->vdev.name, vb->index);
			vb2_buffer_done(ipu_isys_buffer_to_vb2_buffer(ib),
					VB2_BUF_STATE_QUEUED);
		}
		spin_unlock_irqrestore(&aq->lock, flags);
	}
}
EXPORT_SYMBOL_GPL(ipu_flush_firmware_streamon_fail);

/*
 * Attempt obtaining a buffer list from the incoming queues, a list of buffers
 * that contains one entry from each video buffer queue. If a buffer can't be
 * obtained from every queue, the buffers are returned back to the queue.
 */
int ipu_buffer_list_get(struct ipu_isys_stream *stream,
			struct ipu_isys_buffer_list *bl)
{
	unsigned long buf_flag = IPU_ISYS_BUFFER_LIST_FL_INCOMING;
	struct device *dev = isys_to_dev(to_isys7(stream));
	struct ipu_isys_queue *aq;
	unsigned long flags;

	bl->nbufs = 0;
	INIT_LIST_HEAD(&bl->head);

	list_for_each_entry(aq, &stream->queues, node) {
		struct ipu_isys_buffer *ib;

		spin_lock_irqsave(&aq->lock, flags);
		if (list_empty(&aq->incoming)) {
			spin_unlock_irqrestore(&aq->lock, flags);
			if (!list_empty(&bl->head))
				ipu_isys_buffer_list_queue(bl, buf_flag, 0);
			return -ENODATA;
		}

		ib = list_last_entry(&aq->incoming,
				     struct ipu_isys_buffer, head);

		dev_dbg(dev, "buffer: %s: buffer %u\n",
			ipu_isys_queue_to_video(aq)->vdev.name,
			ipu_isys_buffer_to_vb2_buffer(ib)->index);

		list_del(&ib->head);
		list_add(&ib->head, &bl->head);
		spin_unlock_irqrestore(&aq->lock, flags);

		bl->nbufs++;
	}

	dev_dbg(dev, "get buffer list %p, %u buffers\n", bl, bl->nbufs);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_buffer_list_get);

int ipu_isys_link_fmt_validate(struct ipu_isys_queue *aq)
{
	struct ipu_isys_video *av = ipu_isys_queue_to_video(aq);
	struct device *dev = isys_to_dev(to_isys7(av));
	struct media_pad *remote_pad =
		media_pad_remote_pad_first(av->vdev.entity.pads);
	struct v4l2_mbus_framefmt format;
	struct v4l2_subdev *sd;
	u32 r_stream, code, data_fmt;
	int ret;

	if (!remote_pad)
		return -ENOTCONN;

	sd = media_entity_to_v4l2_subdev(remote_pad->entity);
	r_stream = ipu_isys_get_src_stream_by_src_pad(sd, remote_pad->index);

	ret = ipu_isys_get_stream_pad_fmt(sd, remote_pad->index, r_stream,
					   &format);
	if (ret) {
		dev_dbg(dev, "failed to get %s: pad %d, stream:%d format\n",
			sd->entity.name, remote_pad->index, r_stream);
		return ret;
	}

	if (format.width != ipu_isys_get_frame_width(av) ||
	    format.height != ipu_isys_get_frame_height(av)) {
		dev_err(dev, "wrong width or height %ux%u (%ux%u expected)\n",
			ipu_isys_get_frame_width(av),
			ipu_isys_get_frame_height(av), format.width,
			format.height);
		return -EINVAL;
	}

	data_fmt = ipu_isys_get_format(av);
	code = ipu_isys_get_isys_format(to_isys(av), data_fmt, 0)->code;
	if (format.code != code) {
		dev_dbg(dev, "wrong mbus code 0x%8.8x (0x%8.8x expected)\n",
			code, format.code);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_isys_link_fmt_validate);

void ipu_return_buffers(struct ipu_isys_queue *aq, enum vb2_buffer_state state)
{
	struct ipu_isys_video *av = ipu_isys_queue_to_video(aq);
	struct ipu_isys_buffer *ib;
	bool need_reset = false;
	struct vb2_buffer *vb;
	unsigned long flags;

	spin_lock_irqsave(&aq->lock, flags);
	while (!list_empty(&aq->incoming)) {
		ib = list_first_entry(&aq->incoming, struct ipu_isys_buffer,
				      head);
		vb = ipu_isys_buffer_to_vb2_buffer(ib);
		list_del(&ib->head);
		spin_unlock_irqrestore(&aq->lock, flags);

		vb2_buffer_done(vb, state);

		spin_lock_irqsave(&aq->lock, flags);
	}

	/*
	 * Something went wrong (FW crash / HW hang / not all buffers
	 * returned from isys) if there are still buffers queued in active
	 * queue. We have to clean up places a bit.
	 */
	while (!list_empty(&aq->active)) {
		ib = list_first_entry(&aq->active, struct ipu_isys_buffer,
				      head);
		vb = ipu_isys_buffer_to_vb2_buffer(ib);

		list_del(&ib->head);
		spin_unlock_irqrestore(&aq->lock, flags);

		vb2_buffer_done(vb, state);

		spin_lock_irqsave(&aq->lock, flags);
		need_reset = true;
	}

	spin_unlock_irqrestore(&aq->lock, flags);

	if (need_reset) {
		mutex_lock(&to_isys(av)->mutex);
		to_isys(av)->need_reset = true;
		mutex_unlock(&to_isys(av)->mutex);
	}
}
EXPORT_SYMBOL_GPL(ipu_return_buffers);
