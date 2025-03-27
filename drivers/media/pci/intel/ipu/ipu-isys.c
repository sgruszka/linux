#include "ipu-isys.h"

const struct ipu_isys_pixelformat *
ipu_isys_get_isys_format(struct ipu_isys *isys, u32 pixelformat, u32 type)
{
	const struct ipu_isys_pixelformat *default_pfmt = NULL;
	unsigned int i;

	for (i = 0; i < isys->num_pfmts; i++) {
		const struct ipu_isys_pixelformat *pfmt = &isys->pfmts[i];

		if (type &&
		    ((!pfmt->is_meta && type != V4L2_BUF_TYPE_VIDEO_CAPTURE) ||
		     (pfmt->is_meta && type != V4L2_BUF_TYPE_META_CAPTURE)))
			continue;

		if (!default_pfmt)
			default_pfmt = pfmt;

		if (pfmt->pixelformat != pixelformat)
			continue;

		return pfmt;
	}

	return default_pfmt;
}
EXPORT_SYMBOL_GPL(ipu_isys_get_isys_format);

struct ipu_isys_stream *
ipu_isys_get_stream(struct ipu_isys_video *av, struct ipu_isys_subdev *asd)
{
	struct ipu_isys_stream *stream = NULL;
	struct ipu_isys *isys = to_isys(av);
	unsigned long flags;
	unsigned int i;
	u8 vc = av->vc;

	if (!isys)
		return NULL;

	spin_lock_irqsave(&isys->streams_lock, flags);
	for (i = 0; i < IPU_ISYS_MAX_STREAMS; i++) {
		if (isys->streams_ref_count[i] && isys->streams[i].vc == vc &&
		    isys->streams[i].asd == asd) {
			isys->streams_ref_count[i]++;
			stream = &isys->streams[i];
			break;
		}
	}

	if (!stream) {
		for (i = 0; i < IPU_ISYS_MAX_STREAMS; i++) {
			if (!isys->streams_ref_count[i]) {
				isys->streams_ref_count[i]++;
				stream = &isys->streams[i];
				stream->vc = vc;
				stream->asd = asd;
				break;
			}
		}
	}
	spin_unlock_irqrestore(&isys->streams_lock, flags);

	return stream;
}
EXPORT_SYMBOL_GPL(ipu_isys_get_stream);

void ipu_isys_put_stream(struct ipu_isys_stream *stream)
{
	struct ipu_isys *isys = to_isys(stream);
	struct device *dev = isys_to_dev(isys);
	unsigned int i;
	unsigned long flags;

	if (!stream) {
		dev_err(dev, "ipu6-isys: no available stream\n");
		return;
	}

	dev = isys_to_dev(stream->isys);

	spin_lock_irqsave(&isys->streams_lock, flags);
	for (i = 0; i < IPU_ISYS_MAX_STREAMS; i++) {
		if (&isys->streams[i] == stream) {
			if (isys->streams_ref_count[i] > 0)
				isys->streams_ref_count[i]--;
			else
				dev_warn(dev, "invalid stream %d\n", i);
			break;
		}
	}
	spin_unlock_irqrestore(&isys->streams_lock, flags);
}
EXPORT_SYMBOL_GPL(ipu_isys_put_stream);

static void isys_streams_init(struct ipu_isys *isys)
{
	unsigned int i;

	for (i = 0; i < IPU_ISYS_MAX_STREAMS; i++) {
		mutex_init(&isys->streams[i].mutex);
		init_completion(&isys->streams[i].stream_open_completion);
		init_completion(&isys->streams[i].stream_close_completion);
		init_completion(&isys->streams[i].stream_start_completion);
		init_completion(&isys->streams[i].stream_stop_completion);
		INIT_LIST_HEAD(&isys->streams[i].queues);
		isys->streams[i].isys = isys;
		isys->streams[i].stream_handle = i;
		isys->streams[i].vc = -1; /* invalid */
	}
}

static void isys_streams_cleanup(struct ipu_isys *isys)
{
	unsigned int i;

	for (i = 0; i < IPU_ISYS_MAX_STREAMS; i++)
		mutex_destroy(&isys->streams[i].mutex);
}

void ipu_isys_init(struct ipu_isys *isys)
{
	spin_lock_init(&isys->streams_lock);
	spin_lock_init(&isys->power_lock);

	mutex_init(&isys->mutex);
	mutex_init(&isys->stream_mutex);

	isys->power = 0;
	isys->line_align = 64; //TODO remove
	isys->icache_prefetch = 0;

	isys_streams_init(isys);
}
EXPORT_SYMBOL_GPL(ipu_isys_init);

void ipu_isys_cleanup(struct ipu_isys *isys)
{
	isys_streams_cleanup(isys);

	mutex_destroy(&isys->mutex);
	mutex_destroy(&isys->stream_mutex);
}
EXPORT_SYMBOL_GPL(ipu_isys_cleanup);
