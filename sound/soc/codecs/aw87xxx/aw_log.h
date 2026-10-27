#ifndef __AW_LOG_H__
#define __AW_LOG_H__

#include <linux/device.h>
#include <linux/kernel.h>

/*
 * Informational messages are debug level only: the driver dumps the contents
 * of its firmware container and monitor tables on every probe, which is far
 * too verbose for the normal console.  Enable them with dynamic debug on the
 * individual call sites when the firmware blobs need to be inspected.
 */
#define AW_LOGI(fmt, ...)\
	pr_debug("[Awinic] %s: " fmt "\n", __func__, ##__VA_ARGS__)

#define AW_LOGD(fmt, ...)\
	pr_debug("[Awinic] %s: " fmt "\n", __func__, ##__VA_ARGS__)

#define AW_LOGE(fmt, ...)\
	pr_err("[Awinic] %s: " fmt "\n", __func__, ##__VA_ARGS__)


#define AW_DEV_LOGI(dev, fmt, ...)\
	dev_dbg(dev, fmt "\n", ##__VA_ARGS__)

#define AW_DEV_LOGD(dev, fmt, ...)\
	dev_dbg(dev, fmt "\n", ##__VA_ARGS__)

#define AW_DEV_LOGE(dev, fmt, ...)\
	dev_err(dev, fmt "\n", ##__VA_ARGS__)

#endif
