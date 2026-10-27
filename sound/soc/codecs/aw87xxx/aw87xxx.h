#ifndef __AW87XXX_H__
#define __AW87XXX_H__
#include <linux/kernel.h>
#include <sound/control.h>
#include <sound/soc.h>

#include "aw_device.h"
#include "aw_monitor.h"
#include "aw_acf_bin.h"

#define AW_CFG_UPDATE_DELAY
#define AW_CFG_UPDATE_DELAY_TIMER	(3000)

#define AW87XXX_NO_OFF_BIN		(0)
#define AW87XXX_OFF_BIN_OK		(1)

#define AW87XXX_KCONTROL_NUM		(2)

#define AW_I2C_RETRIES			(5)
#define AW_I2C_RETRY_DELAY		(2)
#define AW_I2C_READ_MSG_NUM		(2)

#define AW87XXX_FW_NAME_MAX		(64)
#define AW_NAME_BUF_MAX			(64)
#define AW_LOAD_FW_RETRIES		(3)

#define AW_DEV_REG_RD_ACCESS		(1 << 0)
#define AW_DEV_REG_WR_ACCESS		(1 << 1)

#define AWRW_ADDR_BYTES			(1)
#define AWRW_DATA_BYTES			(1)
#define AWRW_HDR_LEN			(24)

struct aw_component_codec_ops {
	int (*add_codec_controls)(struct snd_soc_component *codec,
		const struct snd_kcontrol_new *controls, unsigned int num_controls);
};


/********************************************
 *
 * aw87xxx devices attributes
 *
 *******************************************/
enum {
	AWRW_FLAG_WRITE = 0,
	AWRW_FLAG_READ,
};

enum {
	AWRW_I2C_ST_NONE = 0,
	AWRW_I2C_ST_READ,
	AWRW_I2C_ST_WRITE,
};

enum {
	AWRW_HDR_WR_FLAG = 0,
	AWRW_HDR_ADDR_BYTES,
	AWRW_HDR_DATA_BYTES,
	AWRW_HDR_REG_NUM,
	AWRW_HDR_REG_ADDR,
	AWRW_HDR_MAX,
};

struct aw_i2c_packet {
	char status;
	unsigned int reg_num;
	unsigned int reg_addr;
	char *reg_data;
};


/********************************************
 *
 * aw87xxx device struct
 *
 *******************************************/
struct aw87xxx {
	char fw_name[AW87XXX_FW_NAME_MAX];
	int32_t dev_index;
	char *current_profile;
	char prof_off_name[AW_PROFILE_STR_MAX];
	uint32_t off_bin_status;
	struct device *dev;

	struct mutex reg_lock;
	struct aw_device aw_dev;
	struct aw_i2c_packet i2c_packet;

	struct delayed_work fw_load_work;
	struct acf_bin_info acf_info;

	struct snd_soc_component *codec;

	struct list_head list;

	struct aw_monitor monitor;
};

char *aw87xxx_show_current_profile(int dev_index);
int aw87xxx_show_current_profile_index(int dev_index);
int aw87xxx_set_profile(int dev_index, char *profile);
int aw87xxx_add_codec_controls(void *codec);
int aw87xxx_update_profile(struct aw87xxx *aw87xxx, char *profile);
int aw87xxx_esd_update_profile(struct aw87xxx *aw87xxx, char *profile);

#endif
