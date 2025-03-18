#ifndef INTEL_IPU_ISYS_H
#define INTEL_IPU_ISYS_H

#include <linux/list.h>
#include <linux/container_of.h>

#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

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
	int source;	/* SSI stream source; -1 if unset */
};

#define to_ipu_isys_subdev(__sd) \
	container_of(__sd, struct ipu_isys_subdev, sd)

#define IPU6_ISYS_MIN_WIDTH		2U
#define IPU6_ISYS_MIN_HEIGHT		2U
#define IPU6_ISYS_MAX_WIDTH		4672U
#define IPU6_ISYS_MAX_HEIGHT		3416U

unsigned int ipu_isys_mbus_code_to_bpp(u32 code);
unsigned int ipu_isys_mbus_code_to_mipi(u32 code);
bool ipu_isys_is_bayer_format(u32 code);
u32 ipu_isys_convert_bayer_order(u32 code, int x, int y);

int ipu_isys_subdev_set_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt);
int ipu_isys_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum
				    *code);
u32 ipu_isys_get_src_stream_by_src_pad(struct v4l2_subdev *sd, u32 pad);
int ipu_isys_get_stream_pad_fmt(struct v4l2_subdev *sd, u32 pad, u32 stream,
				 struct v4l2_mbus_framefmt *format);
int ipu_isys_get_stream_pad_crop(struct v4l2_subdev *sd, u32 pad, u32 stream,
				  struct v4l2_rect *crop);
int ipu_isys_subdev_set_routing(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 enum v4l2_subdev_format_whence which,
				 struct v4l2_subdev_krouting *routing);
int ipu_isys_subdev_init(struct device *dev,
			  struct ipu_isys_subdev *asd,
			  const struct v4l2_subdev_ops *ops,
			  unsigned int nr_ctrls,
			  unsigned int num_sink_pads,
			  unsigned int num_source_pads);
void ipu_isys_subdev_cleanup(struct ipu_isys_subdev *asd);
#endif
