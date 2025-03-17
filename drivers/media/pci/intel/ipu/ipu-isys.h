#ifndef INTEL_IPU_ISYS_H
#define INTEL_IPU_ISYS_H

#include <linux/list.h>

#include <media/videobuf2-v4l2.h>

struct ipu_isys_queue {
	struct vb2_queue vbq;
	struct list_head node;
	struct device *dev;
	/*
	 * @lock: serialise access to queued and pre_streamon_queued
	 */
	spinlock_t lock;
	struct list_head active;
	struct list_head incoming;
	unsigned int fw_output;
};

struct ipu_isys_buffer {
	struct list_head head;
	atomic_t str2mmio_flag;
};

struct ipu_isys_video_buffer {
	struct vb2_v4l2_buffer vb_v4l2;
	struct ipu_isys_buffer ib;
	dma_addr_t dma_addr;
};

struct ipu_isys_buffer_list {
	struct list_head head;
	unsigned int nbufs;
};

#define IPU_ISYS_BUFFER_LIST_FL_INCOMING	BIT(0)
#define IPU_ISYS_BUFFER_LIST_FL_ACTIVE		BIT(1)
#define IPU_ISYS_BUFFER_LIST_FL_SET_STATE	BIT(2)

#define vb2_queue_to_isys_queue(__vb2) \
	container_of(__vb2, struct ipu_isys_queue, vbq)

#define ipu_isys_to_isys_video_buffer(__ib) \
	container_of(__ib, struct ipu_isys_video_buffer, ib)

#define vb2_buffer_to_ipu_isys_video_buffer(__vvb) \
	container_of(__vvb, struct ipu_isys_video_buffer, vb_v4l2)

#define ipu_isys_buffer_to_vb2_buffer(__ib) \
	(&ipu_isys_to_isys_video_buffer(__ib)->vb_v4l2.vb2_buf)


#endif
