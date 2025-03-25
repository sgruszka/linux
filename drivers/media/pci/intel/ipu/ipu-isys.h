#ifndef INTEL_IPU_ISYS_H
#define INTEL_IPU_ISYS_H

#include <linux/list.h>
#include <linux/container_of.h>

#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "ipu.h"

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

struct ipu_isys_subdev {
	struct v4l2_subdev sd;
	u32 const *supported_codes;
	struct media_pad *pad;
	struct v4l2_ctrl_handler ctrl_handler;
	void (*ctrl_init)(struct v4l2_subdev *sd);
	int source; /* SSI stream source; -1 if unset */
};

#define to_ipu_isys_subdev(__sd) container_of(__sd, struct ipu_isys_subdev, sd)

#define IPU6_ISYS_MIN_WIDTH	2U
#define IPU6_ISYS_MIN_HEIGHT	2U
#define IPU6_ISYS_MAX_WIDTH	4672U
#define IPU6_ISYS_MAX_HEIGHT	3416U

struct ipu_isys_pixelformat {
	u32 pixelformat;
	u32 bpp;
	u32 bpp_packed;
	u32 code;
	u32 css_pixelformat;
	bool is_meta;
};

struct ipu_sequence_info {
	unsigned int sequence;
	u64 timestamp;
};

struct ipu_isys_stream;

struct ipu_output_pin_data {
	void (*pin_ready)(struct ipu_isys_stream *stream, void *info);
	struct ipu_isys_queue *aq;
};

#define IPU_ISYS_OUTPUT_PINS 11
#define IPU_ISYS_MAX_PARALLEL_SOF 2

/*
 * Align with firmware stream. Each stream represents a CSI virtual channel.
 * May map to multiple video devices
 */
struct ipu_isys_stream {
	struct mutex mutex;
	struct media_entity *source_entity;
	atomic_t sequence;
	atomic_t buf_id; /* IPU7 */
	unsigned int seq_index;
	struct ipu_sequence_info seq[IPU_ISYS_MAX_PARALLEL_SOF];
	int stream_source;
	int stream_handle;
	unsigned int nr_output_pins;
	struct ipu_isys_subdev *asd;

	int nr_queues; /* Number of capture queues */
	int nr_streaming;
	int streaming; /* Has streaming been really started? */
	struct list_head queues;
	struct completion stream_open_completion;
	struct completion stream_close_completion;
	struct completion stream_start_completion;
	struct completion stream_stop_completion;
	struct ipu_isys *isys;

	struct ipu_output_pin_data output_pins[IPU_ISYS_OUTPUT_PINS];
	int error;
	u8 vc;
};

struct ipu_isys_video {
	struct ipu_isys_queue aq;
	/* Serialise access to other fields in the struct. */
	struct mutex mutex;
	struct media_pad pad;
	struct video_device vdev;
	struct v4l2_pix_format pix_fmt;
	struct v4l2_meta_format meta_fmt;

	struct ipu_isys_stream *stream;
	unsigned int streaming;
	u8 vc;
	u8 dt;

	struct ipu_isys *isys;
};

#define ipu_isys_queue_to_video(__aq) \
	container_of(__aq, struct ipu_isys_video, aq)

static inline struct ipu_isys *ipu_stream_to_isys(struct ipu_isys_stream *stream)
{
	return stream->isys;
}

static inline struct ipu_isys *ipu_video_to_isys(struct ipu_isys_video *video)
{
	return video->isys;
}

#define to_isys(p)                                            \
	_Generic(p,                                           \
		struct ipu_isys_stream *: ipu_stream_to_isys, \
		struct ipu_isys_video *: ipu_video_to_isys)(p)

static inline struct ipu6_isys * ipu_stream_to_isys6(struct ipu_isys_stream *stream)
{
	return (struct ipu6_isys *)stream->isys;
}

static inline struct ipu6_isys *ipu_video_to_isys6(struct ipu_isys_video *video)
{
	return (struct ipu6_isys *)video->isys;
}

#define to_isys6(p)                                            \
	_Generic(p,                                            \
		struct ipu_isys_stream *: ipu_stream_to_isys6, \
		struct ipu_isys_video *: ipu_video_to_isys6)(p)


static inline struct ipu7_isys * ipu_stream_to_isys7(struct ipu_isys_stream *stream)
{
	return (struct ipu7_isys *)stream->isys;
}

static inline struct ipu7_isys *ipu_video_to_isys7(struct ipu_isys_video *video)
{
	return (struct ipu7_isys *)video->isys;
}

#define to_isys7(p)                                            \
	_Generic(p,                                            \
		struct ipu_isys_stream *: ipu_stream_to_isys7, \
		struct ipu_isys_video *: ipu_video_to_isys7)(p)

#define IPU_ISYS_MAX_STREAMS 16

/*
 * struct ipu_isys
 *
 * @media_dev: Media device
 * @v4l2_dev: V4L2 device
 * @adev: ISYS bus device
 * @power: Is ISYS powered on or not?
 * @isr_bits: Which bits does the ISR handle?
 * @power_lock: Serialise access to power (power state in general)
 * @csi2_rx_ctrl_cached: cached shared value between all CSI2 receivers
 * @streams_lock: serialise access to streams
 * @streams: streams per firmware stream ID
 * @syscom: fw communication layer context
 * @line_align: line alignment in memory
 * @need_reset: Isys requires d0i0->i3 transition
 * @ref_count: total number of callers fw open
 * @mutex: serialise access isys video open/release related operations
 * @stream_mutex: serialise stream start and stop, queueing requests
 */

struct ipu_isys {
	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct ipu_bus_device *adev;

	const struct ipu_isys_pixelformat *pfmts;
	unsigned int num_pfmts;

	int power;
	spinlock_t power_lock;
	u32 isr_csi2_bits;
	u32 csi2_rx_ctrl_cached;
	spinlock_t streams_lock;
	struct ipu_isys_stream streams[IPU_ISYS_MAX_STREAMS];
	int streams_ref_count[IPU_ISYS_MAX_STREAMS];
	unsigned int line_align;
	bool need_reset;
	bool icache_prefetch;
	bool csi2_cse_ipc_not_supported;
	unsigned int ref_count;
	unsigned int stream_opened;
	unsigned int sensor_type;

	struct mutex mutex;
	struct mutex stream_mutex;
};

struct ipu6_isys;
struct ipu7_isys;

static inline struct device *ipu_isys_to_dev(struct ipu_isys *isys)
{
	return &isys->adev->auxdev.dev;
}

static inline struct device *ipu6_isys_to_dev(struct ipu6_isys *isys)
{
	return &((struct ipu_isys *)isys)->adev->auxdev.dev;
}

static inline struct device *ipu7_isys_to_dev(struct ipu7_isys *isys)
{
	return &((struct ipu_isys *)isys)->adev->auxdev.dev;
}


#define isys_to_dev(_isys) \
	_Generic(_isys, \
		 struct ipu_isys *  : ipu_isys_to_dev,  \
		 struct ipu6_isys * : ipu6_isys_to_dev, \
		 struct ipu7_isys * : ipu7_isys_to_dev  \
	) (_isys)


unsigned int ipu_isys_mbus_code_to_bpp(u32 code);
unsigned int ipu_isys_mbus_code_to_mipi(u32 code);
bool ipu_isys_is_bayer_format(u32 code);
u32 ipu_isys_convert_bayer_order(u32 code, int x, int y);

int ipu_isys_subdev_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *fmt);
int ipu_isys_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_mbus_code_enum *code);
u32 ipu_isys_get_src_stream_by_src_pad(struct v4l2_subdev *sd, u32 pad);
int ipu_isys_get_stream_pad_fmt(struct v4l2_subdev *sd, u32 pad, u32 stream,
				struct v4l2_mbus_framefmt *format);
int ipu_isys_get_stream_pad_crop(struct v4l2_subdev *sd, u32 pad, u32 stream,
				 struct v4l2_rect *crop);
int ipu_isys_subdev_set_routing(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				enum v4l2_subdev_format_whence which,
				struct v4l2_subdev_krouting *routing);
int ipu_isys_subdev_init(struct device *dev, struct ipu_isys_subdev *asd,
			 const struct v4l2_subdev_ops *ops,
			 unsigned int nr_ctrls, unsigned int num_sink_pads,
			 unsigned int num_source_pads);
void ipu_isys_subdev_cleanup(struct ipu_isys_subdev *asd);

static inline u32 ipu_isys_get_format(struct ipu_isys_video *av)
{
	if (av->aq.vbq.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return av->pix_fmt.pixelformat;

	if (av->aq.vbq.type == V4L2_BUF_TYPE_META_CAPTURE)
		return av->meta_fmt.dataformat;

	return 0;
}

static inline u32 ipu_isys_get_data_size(struct ipu_isys_video *av)
{
	if (av->aq.vbq.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return av->pix_fmt.sizeimage;

	if (av->aq.vbq.type == V4L2_BUF_TYPE_META_CAPTURE)
		return av->meta_fmt.buffersize;

	return 0;
}

static inline u32 ipu_isys_get_bytes_per_line(struct ipu_isys_video *av)
{
	if (av->aq.vbq.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return av->pix_fmt.bytesperline;

	if (av->aq.vbq.type == V4L2_BUF_TYPE_META_CAPTURE)
		return av->meta_fmt.bytesperline;

	return 0;
}

static inline u32 ipu_isys_get_frame_width(struct ipu_isys_video *av)
{
	if (av->aq.vbq.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return av->pix_fmt.width;

	if (av->aq.vbq.type == V4L2_BUF_TYPE_META_CAPTURE)
		return av->meta_fmt.width;

	return 0;
}

static inline u32 ipu_isys_get_frame_height(struct ipu_isys_video *av)
{
	if (av->aq.vbq.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return av->pix_fmt.height;

	if (av->aq.vbq.type == V4L2_BUF_TYPE_META_CAPTURE)
		return av->meta_fmt.height;

	return 0;
}

const struct ipu_isys_pixelformat *
ipu_isys_get_isys_format(struct ipu_isys *isys, u32 pixelformat, u32 type);

int ipu_isys_buf_prepare(struct vb2_buffer *vb);
int ipu_isys_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			 unsigned int *num_planes, unsigned int sizes[],
			 struct device *alloc_devs[]);
void ipu_isys_buffer_list_queue(struct ipu_isys_buffer_list *bl,
				unsigned long op_flags,
				enum vb2_buffer_state state);
void ipu_flush_firmware_streamon_fail(struct ipu_isys_stream *stream);
int ipu_buffer_list_get(struct ipu_isys_stream *stream,
			struct ipu_isys_buffer_list *bl);
int ipu_isys_link_fmt_validate(struct ipu_isys_queue *aq);
void ipu_return_buffers(struct ipu_isys_queue *aq, enum vb2_buffer_state state);
void ipu_stream_buf_ready(struct ipu_isys_stream *stream, u8 pin_id, u32 pin_addr,
			  u64 time, bool error_check);
#endif
