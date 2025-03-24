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
