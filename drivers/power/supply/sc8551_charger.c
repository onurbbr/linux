// SPDX-License-Identifier: GPL-2.0-only
/*
 * Southchip SC8551 charge pump driver.
 *
 * Registers the charge pump as a power supply exposing the VBUS/IBAT
 * protection status and the ADC readings of the battery and bus rails.
 */

#define pr_fmt(fmt)	"[sc8551] " fmt

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include "sc8551_reg.h"

enum sc8551_adc_channel {
	ADC_IBUS,
	ADC_VBUS,
	ADC_VAC,
	ADC_VOUT,
	ADC_VBAT,
	ADC_IBAT,
	ADC_TBUS,
	ADC_TBAT,
	ADC_TDIE,
	ADC_MAX_NUM,
};

static const int sc8551_adc_lsb[] = {
	[ADC_IBUS]	= SC8551_IBUS_ADC_LSB,
	[ADC_VBUS]	= SC8551_VBUS_ADC_LSB,
	[ADC_VAC]	= SC8551_VAC_ADC_LSB,
	[ADC_VOUT]	= SC8551_VOUT_ADC_LSB,
	[ADC_VBAT]	= SC8551_VBAT_ADC_LSB,
	[ADC_IBAT]	= SC8551_IBAT_ADC_LSB,
	[ADC_TBUS]	= SC8551_TSBUS_ADC_LSB,
	[ADC_TBAT]	= SC8551_TSBAT_ADC_LSB,
	[ADC_TDIE]	= SC8551_TDIE_ADC_LSB,
};

enum sc8551_role {
	SC8551_ROLE_STDALONE,
	SC8551_ROLE_SLAVE,
	SC8551_ROLE_MASTER,
};

#define SC8551_DEVICE_VENDOR	8551

static const int sc8551_mode_data[] = {
	[SC8551_ROLE_STDALONE] = SC8551_ROLE_STDALONE,
	[SC8551_ROLE_SLAVE] = SC8551_ROLE_SLAVE,
	[SC8551_ROLE_MASTER] = SC8551_ROLE_MASTER,
};

#define	BAT_OVP_ALARM		BIT(7)
#define BAT_OCP_ALARM		BIT(6)
#define	BUS_OVP_ALARM		BIT(5)
#define	BUS_OCP_ALARM		BIT(4)
#define	BAT_UCP_ALARM		BIT(3)
#define	VBUS_INSERT			BIT(2)
#define VBAT_INSERT			BIT(1)
#define	ADC_DONE			BIT(0)

#define BAT_OVP_FAULT		BIT(7)
#define BAT_OCP_FAULT		BIT(6)
#define BUS_OVP_FAULT		BIT(5)
#define BUS_OCP_FAULT		BIT(4)
#define TBUS_TBAT_ALARM		BIT(3)
#define TS_BAT_FAULT		BIT(2)
#define	TS_BUS_FAULT		BIT(1)
#define	TS_DIE_FAULT		BIT(0)

/*below used for comm with other module*/
#define	BAT_OVP_FAULT_SHIFT			0
#define	BAT_OCP_FAULT_SHIFT			1
#define	BUS_OVP_FAULT_SHIFT			2
#define	BUS_OCP_FAULT_SHIFT			3
#define	BAT_THERM_FAULT_SHIFT		4
#define	BUS_THERM_FAULT_SHIFT		5
#define	DIE_THERM_FAULT_SHIFT		6

#define	BAT_OVP_FAULT_MASK			(1 << BAT_OVP_FAULT_SHIFT)
#define	BAT_OCP_FAULT_MASK			(1 << BAT_OCP_FAULT_SHIFT)
#define	BUS_OVP_FAULT_MASK			(1 << BUS_OVP_FAULT_SHIFT)
#define	BUS_OCP_FAULT_MASK			(1 << BUS_OCP_FAULT_SHIFT)
#define	BAT_THERM_FAULT_MASK		(1 << BAT_THERM_FAULT_SHIFT)
#define	BUS_THERM_FAULT_MASK		(1 << BUS_THERM_FAULT_SHIFT)
#define	DIE_THERM_FAULT_MASK		(1 << DIE_THERM_FAULT_SHIFT)

#define	BAT_OVP_ALARM_SHIFT			0
#define	BAT_OCP_ALARM_SHIFT			1
#define	BUS_OVP_ALARM_SHIFT			2
#define	BUS_OCP_ALARM_SHIFT			3
#define	BAT_THERM_ALARM_SHIFT		4
#define	BUS_THERM_ALARM_SHIFT		5
#define	DIE_THERM_ALARM_SHIFT		6
#define BAT_UCP_ALARM_SHIFT			7

#define	BAT_OVP_ALARM_MASK			(1 << BAT_OVP_ALARM_SHIFT)
#define	BAT_OCP_ALARM_MASK			(1 << BAT_OCP_ALARM_SHIFT)
#define	BUS_OVP_ALARM_MASK			(1 << BUS_OVP_ALARM_SHIFT)
#define	BUS_OCP_ALARM_MASK			(1 << BUS_OCP_ALARM_SHIFT)
#define	BAT_THERM_ALARM_MASK		(1 << BAT_THERM_ALARM_SHIFT)
#define	BUS_THERM_ALARM_MASK		(1 << BUS_THERM_ALARM_SHIFT)
#define	DIE_THERM_ALARM_MASK		(1 << DIE_THERM_ALARM_SHIFT)
#define	BAT_UCP_ALARM_MASK			(1 << BAT_UCP_ALARM_SHIFT)

#define VBAT_REG_STATUS_SHIFT		0
#define IBAT_REG_STATUS_SHIFT		1

#define VBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)
#define IBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)

#define sc_err(fmt, ...)	dev_err(sc->dev, fmt, ##__VA_ARGS__)
#define sc_info(fmt, ...)	dev_info(sc->dev, fmt, ##__VA_ARGS__)
#define sc_dbg(fmt, ...)	dev_dbg(sc->dev, fmt, ##__VA_ARGS__)

/*end*/

struct sc8551_cfg {
	bool bat_ovp_disable;
	bool bat_ocp_disable;
	bool bat_ovp_alm_disable;
	bool bat_ocp_alm_disable;

	int bat_ovp_th;
	int bat_ovp_alm_th;
	int bat_ocp_th;
	int bat_ocp_alm_th;

	bool bus_ovp_alm_disable;
	bool bus_ocp_disable;
	bool bus_ocp_alm_disable;

	int bus_ovp_th;
	int bus_ovp_alm_th;
	int bus_ocp_th;
	int bus_ocp_alm_th;

	bool bat_ucp_alm_disable;

	int bat_ucp_alm_th;
	int ac_ovp_th;

	bool bat_therm_disable;
	bool bus_therm_disable;
	bool die_therm_disable;

	int bat_therm_th; /*in %*/
	int bus_therm_th; /*in %*/
	int die_therm_th; /*in degC*/

	int sense_r_mohm;
};

struct sc8551 {
	struct device *dev;
	struct i2c_client *client;

	int part_no;
	int chip_vendor;

	int mode;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex irq_complete;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool batt_present;
	bool vbus_present;

	bool usb_present;
	bool charge_enabled;	/* Register bit status */

	bool is_sc8551;
	int  vbus_error;

	/* ADC reading */
	int vbat_volt;
	int vbus_volt;
	int vout_volt;
	int vac_volt;

	int ibat_curr;
	int ibus_curr;

	int bat_temp;
	int bus_temp;
	int die_temp;

	/* alarm/fault status */
	bool bat_ovp_fault;
	bool bat_ocp_fault;
	bool bus_ovp_fault;
	bool bus_ocp_fault;

	bool bat_ovp_alarm;
	bool bat_ocp_alarm;
	bool bus_ovp_alarm;
	bool bus_ocp_alarm;

	bool bat_ucp_alarm;

	bool bat_therm_alarm;
	bool bus_therm_alarm;
	bool die_therm_alarm;

	bool bat_therm_fault;
	bool bus_therm_fault;
	bool die_therm_fault;

	bool therm_shutdown_flag;
	bool therm_shutdown_stat;

	bool vbat_reg;
	bool ibat_reg;

	int  prev_alarm;
	int  prev_fault;

	int chg_ma;
	int chg_mv;

	int charge_state;

	struct sc8551_cfg *cfg;

	struct power_supply_desc psy_desc;
	struct power_supply_config psy_cfg;
	struct power_supply *fc2_psy;
};

/************************************************************************/
static int __sc8551_read_byte(struct sc8551 *sc, u8 reg, u8 *data)
{
	s32 ret;
	int cnt = 3;

	while (cnt--) {
		ret = i2c_smbus_read_byte_data(sc->client, reg);
		if (ret < 0) {
			sc_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		} else {
			*data = (u8) ret;
			return 0;
		}
		udelay(200);
	}

	return ret;
}

static int __sc8551_write_byte(struct sc8551 *sc, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(sc->client, reg, val);
	if (ret < 0) {
		sc_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	return 0;
}

static int sc8551_read_byte(struct sc8551 *sc, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc8551_read_byte(sc, reg, data);
	mutex_unlock(&sc->i2c_rw_lock);

	return ret;
}

static int sc8551_write_byte(struct sc8551 *sc, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc8551_write_byte(sc, reg, data);
	mutex_unlock(&sc->i2c_rw_lock);

	return ret;
}

static int sc8551_update_bits(struct sc8551*sc, u8 reg,
				    u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc8551_read_byte(sc, reg, &tmp);
	if (ret) {
		sc_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __sc8551_write_byte(sc, reg, tmp);
	if (ret)
		sc_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&sc->i2c_rw_lock);
	return ret;
}

/*********************************************************************/

static int sc8551_enable_charge(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_CHG_ENABLE;
	else
		val = SC8551_CHG_DISABLE;

	val <<= SC8551_CHG_EN_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_0C,
				SC8551_CHG_EN_MASK, val);

	return ret;
}

static int sc8551_check_charge_enabled(struct sc8551 *sc, bool *enabled)
{
	int ret;
	u8 val;

	ret = sc8551_read_byte(sc, SC8551_REG_0C, &val);
	if (!ret)
		*enabled = !!(val & SC8551_CHG_EN_MASK);
	return ret;
}

static int sc8551_enable_wdt(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_WATCHDOG_ENABLE;
	else
		val = SC8551_WATCHDOG_DISABLE;

	val <<= SC8551_WATCHDOG_DIS_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_0B,
				SC8551_WATCHDOG_DIS_MASK, val);
	return ret;
}

static int sc8551_set_reg_reset(struct sc8551 *sc)
{
	int ret;
	u8 val = 1;

	sc_err("mode : %d  reg reset\n", sc->mode);

	val = SC8551_REG_RST_ENABLE;

	val <<= SC8551_REG_RST_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_0B,
				SC8551_REG_RST_MASK, val);
	return ret;
}

static int sc8551_enable_batovp(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_BAT_OVP_ENABLE;
	else
		val = SC8551_BAT_OVP_DISABLE;

	val <<= SC8551_BAT_OVP_DIS_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_00,
				SC8551_BAT_OVP_DIS_MASK, val);
	return ret;
}

static int sc8551_set_batovp_th(struct sc8551 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8551_BAT_OVP_BASE)
		threshold = SC8551_BAT_OVP_BASE;

	val = (threshold - SC8551_BAT_OVP_BASE) / SC8551_BAT_OVP_LSB;

	val <<= SC8551_BAT_OVP_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_00,
				SC8551_BAT_OVP_MASK, val);
	return ret;
}

static int sc8551_enable_batovp_alarm(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_BAT_OVP_ALM_ENABLE;
	else
		val = SC8551_BAT_OVP_ALM_DISABLE;

	val <<= SC8551_BAT_OVP_ALM_DIS_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_01,
				SC8551_BAT_OVP_ALM_DIS_MASK, val);
	return ret;
}

static int sc8551_set_batovp_alarm_th(struct sc8551 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8551_BAT_OVP_ALM_BASE)
		threshold = SC8551_BAT_OVP_ALM_BASE;

	val = (threshold - SC8551_BAT_OVP_ALM_BASE) / SC8551_BAT_OVP_ALM_LSB;

	val <<= SC8551_BAT_OVP_ALM_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_01,
				SC8551_BAT_OVP_ALM_MASK, val);
	return ret;
}

static int sc8551_enable_batocp(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_BAT_OCP_ENABLE;
	else
		val = SC8551_BAT_OCP_DISABLE;

	val <<= SC8551_BAT_OCP_DIS_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_02,
				SC8551_BAT_OCP_DIS_MASK, val);
	return ret;
}

static int sc8551_set_batocp_th(struct sc8551 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8551_BAT_OCP_BASE)
		threshold = SC8551_BAT_OCP_BASE;

	val = (threshold - SC8551_BAT_OCP_BASE) / SC8551_BAT_OCP_LSB;

	val <<= SC8551_BAT_OCP_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_02,
				SC8551_BAT_OCP_MASK, val);
	return ret;
}

static int sc8551_enable_batocp_alarm(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_BAT_OCP_ALM_ENABLE;
	else
		val = SC8551_BAT_OCP_ALM_DISABLE;

	val <<= SC8551_BAT_OCP_ALM_DIS_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_03,
				SC8551_BAT_OCP_ALM_DIS_MASK, val);
	return ret;
}

static int sc8551_set_batocp_alarm_th(struct sc8551 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8551_BAT_OCP_ALM_BASE)
		threshold = SC8551_BAT_OCP_ALM_BASE;

	val = (threshold - SC8551_BAT_OCP_ALM_BASE) / SC8551_BAT_OCP_ALM_LSB;

	val <<= SC8551_BAT_OCP_ALM_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_03,
				SC8551_BAT_OCP_ALM_MASK, val);
	return ret;
}


static int sc8551_set_busovp_th(struct sc8551 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8551_BUS_OVP_BASE)
		threshold = SC8551_BUS_OVP_BASE;

	val = (threshold - SC8551_BUS_OVP_BASE) / SC8551_BUS_OVP_LSB;

	val <<= SC8551_BUS_OVP_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_06,
				SC8551_BUS_OVP_MASK, val);
	return ret;
}

static int sc8551_enable_busovp_alarm(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_BUS_OVP_ALM_ENABLE;
	else
		val = SC8551_BUS_OVP_ALM_DISABLE;

	val <<= SC8551_BUS_OVP_ALM_DIS_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_07,
				SC8551_BUS_OVP_ALM_DIS_MASK, val);
	return ret;
}

static int sc8551_set_busovp_alarm_th(struct sc8551 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8551_BUS_OVP_ALM_BASE)
		threshold = SC8551_BUS_OVP_ALM_BASE;

	val = (threshold - SC8551_BUS_OVP_ALM_BASE) / SC8551_BUS_OVP_ALM_LSB;

	val <<= SC8551_BUS_OVP_ALM_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_07,
				SC8551_BUS_OVP_ALM_MASK, val);
	return ret;
}

static int sc8551_enable_busocp(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_BUS_OCP_ENABLE;
	else
		val = SC8551_BUS_OCP_DISABLE;

	val <<= SC8551_BUS_OCP_DIS_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_08,
				SC8551_BUS_OCP_DIS_MASK, val);
	return ret;
}


static int sc8551_set_busocp_th(struct sc8551 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8551_BUS_OCP_BASE)
		threshold = SC8551_BUS_OCP_BASE;

	val = (threshold - SC8551_BUS_OCP_BASE) / SC8551_BUS_OCP_LSB;

	val <<= SC8551_BUS_OCP_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_08,
				SC8551_BUS_OCP_MASK, val);
	return ret;
}

static int sc8551_enable_busocp_alarm(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_BUS_OCP_ALM_ENABLE;
	else
		val = SC8551_BUS_OCP_ALM_DISABLE;

	val <<= SC8551_BUS_OCP_ALM_DIS_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_09,
				SC8551_BUS_OCP_ALM_DIS_MASK, val);
	return ret;
}

static int sc8551_set_busocp_alarm_th(struct sc8551 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8551_BUS_OCP_ALM_BASE)
		threshold = SC8551_BUS_OCP_ALM_BASE;

	val = (threshold - SC8551_BUS_OCP_ALM_BASE) / SC8551_BUS_OCP_ALM_LSB;

	val <<= SC8551_BUS_OCP_ALM_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_09,
				SC8551_BUS_OCP_ALM_MASK, val);
	return ret;
}

static int sc8551_enable_batucp_alarm(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_BAT_UCP_ALM_ENABLE;
	else
		val = SC8551_BAT_UCP_ALM_DISABLE;

	val <<= SC8551_BAT_UCP_ALM_DIS_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_04,
				SC8551_BAT_UCP_ALM_DIS_MASK, val);
	return ret;
}

static int sc8551_set_batucp_alarm_th(struct sc8551 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8551_BAT_UCP_ALM_BASE)
		threshold = SC8551_BAT_UCP_ALM_BASE;

	val = (threshold - SC8551_BAT_UCP_ALM_BASE) / SC8551_BAT_UCP_ALM_LSB;

	val <<= SC8551_BAT_UCP_ALM_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_04,
				SC8551_BAT_UCP_ALM_MASK, val);
	return ret;
}

static int sc8551_set_acovp_th(struct sc8551 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8551_AC_OVP_BASE)
		threshold = SC8551_AC_OVP_BASE;

	if (threshold == SC8551_AC_OVP_6P5V)
		val = 0x07;
	else
		val = (threshold - SC8551_AC_OVP_BASE) /  SC8551_AC_OVP_LSB;

	val <<= SC8551_AC_OVP_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_05,
				SC8551_AC_OVP_MASK, val);

	return ret;

}

static int sc8551_set_vdrop_th(struct sc8551 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold == 300)
		val = SC8551_VDROP_THRESHOLD_300MV;
	else
		val = SC8551_VDROP_THRESHOLD_400MV;

	val <<= SC8551_VDROP_THRESHOLD_SET_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_05,
				SC8551_VDROP_THRESHOLD_SET_MASK,
				val);

	return ret;
}

static int sc8551_set_vdrop_deglitch(struct sc8551 *sc, int us)
{
	int ret;
	u8 val;

	if (us == 8)
		val = SC8551_VDROP_DEGLITCH_8US;
	else
		val = SC8551_VDROP_DEGLITCH_5MS;

	val <<= SC8551_VDROP_DEGLITCH_SET_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_05,
				SC8551_VDROP_DEGLITCH_SET_MASK,
				val);
	return ret;
}

static int sc8551_enable_bat_therm(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_TSBAT_ENABLE;
	else
		val = SC8551_TSBAT_DISABLE;

	val <<= SC8551_TSBAT_DIS_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_0C,
				SC8551_TSBAT_DIS_MASK, val);
	return ret;
}

/*
 * the input threshold is the raw value that would write to register directly.
 */
static int sc8551_set_bat_therm_th(struct sc8551 *sc, u8 threshold)
{
	int ret;

	ret = sc8551_write_byte(sc, SC8551_REG_29, threshold);
	return ret;
}

static int sc8551_enable_bus_therm(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_TSBUS_ENABLE;
	else
		val = SC8551_TSBUS_DISABLE;

	val <<= SC8551_TSBUS_DIS_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_0C,
				SC8551_TSBUS_DIS_MASK, val);
	return ret;
}

/*
 * the input threshold is the raw value that would write to register directly.
 */
static int sc8551_set_bus_therm_th(struct sc8551 *sc, u8 threshold)
{
	int ret;

	ret = sc8551_write_byte(sc, SC8551_REG_28, threshold);
	return ret;
}

/*
 * please be noted that the unit here is degC
 */
static int sc8551_set_die_therm_th(struct sc8551 *sc, u8 threshold)
{
	int ret;
	u8 val;

	/*BE careful, LSB is here is 1/LSB, so we use multiply here*/
	val = (threshold - SC8551_TDIE_ALM_BASE) * 10/SC8551_TDIE_ALM_LSB;
	val <<= SC8551_TDIE_ALM_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_2A,
				SC8551_TDIE_ALM_MASK, val);
	return ret;
}

static int sc8551_enable_adc(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_ADC_ENABLE;
	else
		val = SC8551_ADC_DISABLE;

	val <<= SC8551_ADC_EN_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_14,
				SC8551_ADC_EN_MASK, val);
	return ret;
}


static int sc8551_set_adc_scanrate(struct sc8551 *sc, bool oneshot)
{
	int ret;
	u8 val;

	if (oneshot)
		val = SC8551_ADC_RATE_ONESHOT;
	else
		val = SC8551_ADC_RATE_CONTINOUS;

	val <<= SC8551_ADC_RATE_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_14,
				SC8551_ADC_EN_MASK, val);
	return ret;
}


#define ADC_REG_BASE SC8551_REG_16
static int sc8551_get_adc_data(struct sc8551 *sc, int channel,  int *result)
{
	int ret;
	u16 val;
	u8 val_l, val_h;
	s16 t;

	if(channel < 0 || channel >= ADC_MAX_NUM) return -EINVAL;

	ret = sc8551_read_byte(sc, ADC_REG_BASE + (channel << 1), &val_h);
	if (ret < 0)
		return ret;

	ret = sc8551_read_byte(sc, ADC_REG_BASE + (channel << 1) + 1, &val_l);
	if (ret < 0)
		return ret;

	val = (val_h << 8) | val_l;
	*result = val;

	if (sc->chip_vendor == SC8551_DEVICE_VENDOR) {
		/* Registers are 16 bit signed; convert to mV, mA, 0.1 K
		 * or 0.1 degC depending on the channel.
		 */
		t = (s16)val;
		*result = (u64)t * (u64)sc8551_adc_lsb[channel] / 10000000;
	}
	return 0;
}

static int sc8551_set_adc_scan(struct sc8551 *sc, int channel, bool enable)
{
	int ret;
	u8 reg;
	u8 mask;
	u8 shift;
	u8 val;

	if (channel < 0 || channel >= ADC_MAX_NUM)
		return -EINVAL;

	if (channel == ADC_IBUS) {
		reg = SC8551_REG_14;
		shift = SC8551_IBUS_ADC_DIS_SHIFT;
		mask = SC8551_IBUS_ADC_DIS_MASK;
	} else {
		reg = SC8551_REG_15;
		shift = 8 - channel;
		mask = 1 << shift;
	}

	if (enable)
		val = 0 << shift;
	else
		val = 1 << shift;

	ret = sc8551_update_bits(sc, reg, mask, val);

	return ret;
}

static int sc8551_set_alarm_int_mask(struct sc8551 *sc, u8 mask)
{
	int ret;
	u8 val;

	ret = sc8551_read_byte(sc, SC8551_REG_0F, &val);
	if (ret)
		return ret;

	val |= mask;

	ret = sc8551_write_byte(sc, SC8551_REG_0F, val);

	return ret;
}

static int sc8551_set_sense_resistor(struct sc8551 *sc, int r_mohm)
{
	int ret;
	u8 val;

	if (r_mohm == 2)
		val = SC8551_SET_IBAT_SNS_RES_2MHM;
	else if (r_mohm == 5)
		val = SC8551_SET_IBAT_SNS_RES_5MHM;
	else
		return -EINVAL;

	val <<= SC8551_SET_IBAT_SNS_RES_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_2B,
				SC8551_SET_IBAT_SNS_RES_MASK,
				val);
	return ret;
}

static int sc8551_enable_regulation(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8551_EN_REGULATION_ENABLE;
	else
		val = SC8551_EN_REGULATION_DISABLE;

	val <<= SC8551_EN_REGULATION_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_2B,
				SC8551_EN_REGULATION_MASK,
				val);

	return ret;

}

static int sc8551_set_ss_timeout(struct sc8551 *sc, int timeout)
{
	int ret;
	u8 val;

	switch (timeout) {
	case 0:
		val = SC8551_SS_TIMEOUT_DISABLE;
		break;
	case 12:
		val = SC8551_SS_TIMEOUT_12P5MS;
		break;
	case 25:
		val = SC8551_SS_TIMEOUT_25MS;
		break;
	case 50:
		val = SC8551_SS_TIMEOUT_50MS;
		break;
	case 100:
		val = SC8551_SS_TIMEOUT_100MS;
		break;
	case 400:
		val = SC8551_SS_TIMEOUT_400MS;
		break;
	case 1500:
		val = SC8551_SS_TIMEOUT_1500MS;
		break;
	case 100000:
		val = SC8551_SS_TIMEOUT_100000MS;
		break;
	default:
		val = SC8551_SS_TIMEOUT_DISABLE;
		break;
	}

	val <<= SC8551_SS_TIMEOUT_SET_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_2B,
				SC8551_SS_TIMEOUT_SET_MASK,
				val);

	return ret;
}

static int sc8551_set_ibat_reg_th(struct sc8551 *sc, int th_ma)
{
	int ret;
	u8 val;

	if (th_ma == 200)
		val = SC8551_IBAT_REG_200MA;
	else if (th_ma == 300)
		val = SC8551_IBAT_REG_300MA;
	else if (th_ma == 400)
		val = SC8551_IBAT_REG_400MA;
	else if (th_ma == 500)
		val = SC8551_IBAT_REG_500MA;
	else
		val = SC8551_IBAT_REG_500MA;

	val <<= SC8551_IBAT_REG_SHIFT;
	ret = sc8551_update_bits(sc, SC8551_REG_2C,
				SC8551_IBAT_REG_MASK,
				val);

	return ret;

}

static int sc8551_set_vbat_reg_th(struct sc8551 *sc, int th_mv)
{
	int ret;
	u8 val;

	if (th_mv == 50)
		val = SC8551_VBAT_REG_50MV;
	else if (th_mv == 100)
		val = SC8551_VBAT_REG_100MV;
	else if (th_mv == 150)
		val = SC8551_VBAT_REG_150MV;
	else
		val = SC8551_VBAT_REG_200MV;

	val <<= SC8551_VBAT_REG_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_2C,
				SC8551_VBAT_REG_MASK,
				val);

	return ret;
}

static int sc8551_check_vbus_error_status(struct sc8551 *sc)
{
	int ret;
	u8 data;

	ret = sc8551_read_byte(sc, SC8551_REG_0A, &data);
	if (ret == 0)
		sc->vbus_error = data;

	return ret;
}

static int sc8551_detect_device(struct sc8551 *sc)
{
	u8 data;
	int ret;

	ret = sc8551_read_byte(sc, SC8551_REG_13, &data);
	if (ret)
		return ret;

	if (data != SC8551_DEVICE_ID && data != SC8551A_DEVICE_ID)
		return -ENODEV;

	sc->part_no = (data & SC8551_DEV_ID_MASK) >> SC8551_DEV_ID_SHIFT;
	sc->chip_vendor = SC8551_DEVICE_VENDOR;

	return 0;
}

static int sc8551_parse_dt(struct sc8551 *sc, struct device *dev)
{
	int ret;
	struct device_node *np = dev->of_node;

	sc->cfg = devm_kzalloc(dev, sizeof(struct sc8551_cfg),
					GFP_KERNEL);

	if (!sc->cfg)
		return -ENOMEM;

	sc->cfg->bat_ovp_disable = of_property_read_bool(np,
			"sc,sc8551,bat-ovp-disable");
	sc->cfg->bat_ocp_disable = of_property_read_bool(np,
			"sc,sc8551,bat-ocp-disable");
	sc->cfg->bat_ovp_alm_disable = of_property_read_bool(np,
			"sc,sc8551,bat-ovp-alarm-disable");
	sc->cfg->bat_ocp_alm_disable = of_property_read_bool(np,
			"sc,sc8551,bat-ocp-alarm-disable");
	sc->cfg->bus_ocp_disable = of_property_read_bool(np,
			"sc,sc8551,bus-ocp-disable");
	sc->cfg->bus_ovp_alm_disable = of_property_read_bool(np,
			"sc,sc8551,bus-ovp-alarm-disable");
	sc->cfg->bus_ocp_alm_disable = of_property_read_bool(np,
			"sc,sc8551,bus-ocp-alarm-disable");
	sc->cfg->bat_ucp_alm_disable = of_property_read_bool(np,
			"sc,sc8551,bat-ucp-alarm-disable");
	sc->cfg->bat_therm_disable = of_property_read_bool(np,
			"sc,sc8551,bat-therm-disable");
	sc->cfg->bus_therm_disable = of_property_read_bool(np,
			"sc,sc8551,bus-therm-disable");

	ret = of_property_read_u32(np, "sc,sc8551,bat-ovp-threshold",
			&sc->cfg->bat_ovp_th);
	if (ret) {
		sc_err("failed to read bat-ovp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8551,bat-ovp-alarm-threshold",
			&sc->cfg->bat_ovp_alm_th);
	if (ret) {
		sc_err("failed to read bat-ovp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8551,bat-ocp-threshold",
			&sc->cfg->bat_ocp_th);
	if (ret) {
		sc_err("failed to read bat-ocp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8551,bat-ocp-alarm-threshold",
			&sc->cfg->bat_ocp_alm_th);
	if (ret) {
		sc_err("failed to read bat-ocp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8551,bus-ovp-threshold",
			&sc->cfg->bus_ovp_th);
	if (ret) {
		sc_err("failed to read bus-ovp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8551,bus-ovp-alarm-threshold",
			&sc->cfg->bus_ovp_alm_th);
	if (ret) {
		sc_err("failed to read bus-ovp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8551,bus-ocp-threshold",
			&sc->cfg->bus_ocp_th);
	if (ret) {
		sc_err("failed to read bus-ocp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8551,bus-ocp-alarm-threshold",
			&sc->cfg->bus_ocp_alm_th);
	if (ret) {
		sc_err("failed to read bus-ocp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8551,bat-ucp-alarm-threshold",
			&sc->cfg->bat_ucp_alm_th);
	if (ret) {
		sc_err("failed to read bat-ucp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8551,bat-therm-threshold",
			&sc->cfg->bat_therm_th);
	if (ret) {
		sc_err("failed to read bat-therm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8551,bus-therm-threshold",
			&sc->cfg->bus_therm_th);
	if (ret) {
		sc_err("failed to read bus-therm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "sc,sc8551,die-therm-threshold",
			&sc->cfg->die_therm_th);
	if (ret) {
		sc_err("failed to read die-therm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,ac-ovp-threshold",
			&sc->cfg->ac_ovp_th);
	if (ret) {
		sc_err("failed to read ac-ovp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,sense-resistor-mohm",
			&sc->cfg->sense_r_mohm);
	if (ret) {
		sc_err("failed to read sense-resistor-mohm\n");
		return ret;
	}


	return 0;
}


static int sc8551_init_protection(struct sc8551 *sc)
{
	struct sc8551_cfg *cfg = sc->cfg;
	int ret;

	ret = sc8551_set_batovp_th(sc, cfg->bat_ovp_th);
	if (ret)
		return ret;

	ret = sc8551_enable_batovp(sc, !cfg->bat_ovp_disable);
	if (ret)
		return ret;

	ret = sc8551_set_batocp_th(sc, cfg->bat_ocp_th);
	if (ret)
		return ret;

	ret = sc8551_enable_batocp(sc, !cfg->bat_ocp_disable);
	if (ret)
		return ret;

	ret = sc8551_set_batovp_alarm_th(sc, cfg->bat_ovp_alm_th);
	if (ret)
		return ret;

	ret = sc8551_enable_batovp_alarm(sc, !cfg->bat_ovp_alm_disable);
	if (ret)
		return ret;

	ret = sc8551_set_batocp_alarm_th(sc, cfg->bat_ocp_alm_th);
	if (ret)
		return ret;

	ret = sc8551_enable_batocp_alarm(sc, !cfg->bat_ocp_alm_disable);
	if (ret)
		return ret;

	ret = sc8551_set_batucp_alarm_th(sc, cfg->bat_ucp_alm_th);
	if (ret)
		return ret;

	ret = sc8551_enable_batucp_alarm(sc, !cfg->bat_ucp_alm_disable);
	if (ret)
		return ret;

	ret = sc8551_set_busovp_th(sc, cfg->bus_ovp_th);
	if (ret)
		return ret;

	ret = sc8551_set_busovp_alarm_th(sc, cfg->bus_ovp_alm_th);
	if (ret)
		return ret;

	ret = sc8551_enable_busovp_alarm(sc, !cfg->bus_ovp_alm_disable);
	if (ret)
		return ret;

	ret = sc8551_set_busocp_th(sc, cfg->bus_ocp_th);
	if (ret)
		return ret;

	ret = sc8551_enable_busocp(sc, !cfg->bus_ocp_disable);
	if (ret)
		return ret;

	ret = sc8551_set_busocp_alarm_th(sc, cfg->bus_ocp_alm_th);
	if (ret)
		return ret;

	ret = sc8551_enable_busocp_alarm(sc, !cfg->bus_ocp_alm_disable);
	if (ret)
		return ret;

	ret = sc8551_set_acovp_th(sc, cfg->ac_ovp_th);
	if (ret)
		return ret;

	ret = sc8551_set_bat_therm_th(sc, cfg->bat_therm_th);
	if (ret)
		return ret;

	ret = sc8551_set_bus_therm_th(sc, cfg->bus_therm_th);
	if (ret)
		return ret;

	ret = sc8551_set_die_therm_th(sc, cfg->die_therm_th);
	if (ret)
		return ret;

	if (!cfg->bat_therm_disable) {
		ret = sc8551_enable_bat_therm(sc, true);
		if (ret)
			return ret;
	}

	if (!cfg->bus_therm_disable)
		return sc8551_enable_bus_therm(sc, true);

	return 0;
}

static int sc8551_init_adc(struct sc8551 *sc)
{
	int ret;

	ret = sc8551_set_adc_scanrate(sc, false);
	if (ret)
		return ret;

	ret = sc8551_set_adc_scan(sc, ADC_IBUS, true);
	if (ret)
		return ret;

	ret = sc8551_set_adc_scan(sc, ADC_VBUS, true);
	if (ret)
		return ret;

	ret = sc8551_set_adc_scan(sc, ADC_VOUT, true);
	if (ret)
		return ret;

	ret = sc8551_set_adc_scan(sc, ADC_VBAT, true);
	if (ret)
		return ret;

	ret = sc8551_set_adc_scan(sc, ADC_IBAT, true);
	if (ret)
		return ret;

	ret = sc8551_set_adc_scan(sc, ADC_TBUS, true);
	if (ret)
		return ret;

	ret = sc8551_set_adc_scan(sc, ADC_TBAT, true);
	if (ret)
		return ret;

	ret = sc8551_set_adc_scan(sc, ADC_TDIE, true);
	if (ret)
		return ret;

	ret = sc8551_set_adc_scan(sc, ADC_VAC, true);
	if (ret)
		return ret;

	/* Improve the ADC accuracy. */
	ret = sc8551_write_byte(sc, SC8551_REG_34, 0x01);
	if (ret)
		return ret;

	return sc8551_enable_adc(sc, true);
}

static int sc8551_init_int_src(struct sc8551 *sc)
{
	int ret;
	/*TODO:be careful ts bus and ts bat alarm bit mask is in
	 *	fault mask register, so you need call
	 *	sc8551_set_fault_int_mask for tsbus and tsbat alarm
	 */
	ret = sc8551_set_alarm_int_mask(sc, ADC_DONE
		/*			| BAT_UCP_ALARM */
					| BAT_OVP_ALARM);
	if (ret) {
		sc_err("failed to set alarm mask:%d\n", ret);
		return ret;
	}
	return ret;
}

static int sc8551_init_regulation(struct sc8551 *sc)
{
	sc8551_set_ibat_reg_th(sc, 300);
	sc8551_set_vbat_reg_th(sc, 100);

	sc8551_set_vdrop_deglitch(sc, 5000);
	sc8551_set_vdrop_th(sc, 400);

	sc8551_enable_regulation(sc, false);

	if(sc->is_sc8551)
	{
		sc8551_write_byte(sc, SC8551_REG_2E, 0x08);
		sc8551_write_byte(sc, SC8551_REG_34, 0x01);
	}
	return 0;
}

static int sc8551_disable_vbus_range(struct sc8551 *sc, bool disable)
{
	int ret;
	u8 val;
	ret = sc8551_read_byte(sc, SC8551_REG_35, &val);

	if (disable)
		val = SC8551_VBUS_RANGE_DISABLE;
	else
		val = SC8551_VBUS_RANGE_ENABLE;

	val <<= SC8551_VBUS_RANGE_DIS_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_35,
				SC8551_VBUS_RANGE_DIS_MASK, val);

	sc8551_read_byte(sc, SC8551_REG_35, &val);
	return ret;
}
static int sc8551_init_device(struct sc8551 *sc)
{
	int ret;

	ret = sc8551_set_reg_reset(sc);
	if (ret)
		return ret;

	ret = sc8551_enable_wdt(sc, false);
	if (ret)
		return ret;

	ret = sc8551_set_ss_timeout(sc, 100000);
	if (ret)
		return ret;

	ret = sc8551_set_sense_resistor(sc, sc->cfg->sense_r_mohm);
	if (ret)
		return ret;

	ret = sc8551_init_protection(sc);
	if (ret)
		return ret;

	ret = sc8551_init_adc(sc);
	if (ret)
		return ret;

	ret = sc8551_init_int_src(sc);
	if (ret)
		return ret;

	ret = sc8551_init_regulation(sc);
	if (ret)
		return ret;

	if (sc->mode == SC8551_ROLE_SLAVE)
		return sc8551_disable_vbus_range(sc, true);

	return 0;
}


static int sc8551_set_present(struct sc8551 *sc, bool present)
{
	sc->usb_present = present;

	if (present)
		return sc8551_init_device(sc);

	return 0;
}

static ssize_t sc8551_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sc8551 *sc = dev_get_drvdata(dev);
	int idx = 0;
	u8 addr, val;
	int ret;

	for (addr = 0x00; addr <= 0x36; addr++) {
		ret = sc8551_read_byte(sc, addr, &val);
		if (ret)
			continue;

		idx += sysfs_emit_at(buf, idx, "Reg[%02X] = 0x%02x\n",
				     addr, val);
	}

	return idx;
}

static DEVICE_ATTR(registers, 0444, sc8551_show_registers, NULL);

static enum power_supply_property sc8551_charger_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_SC_BATTERY_PRESENT,
	POWER_SUPPLY_PROP_SC_VBUS_PRESENT,
	POWER_SUPPLY_PROP_SC_BATTERY_VOLTAGE,
	POWER_SUPPLY_PROP_SC_BATTERY_CURRENT,
	POWER_SUPPLY_PROP_SC_BATTERY_TEMPERATURE,
	POWER_SUPPLY_PROP_SC_BUS_VOLTAGE,
	POWER_SUPPLY_PROP_SC_BUS_CURRENT,
	POWER_SUPPLY_PROP_SC_BUS_TEMPERATURE,
	POWER_SUPPLY_PROP_SC_DIE_TEMPERATURE,
	POWER_SUPPLY_PROP_SC_ALARM_STATUS,
	POWER_SUPPLY_PROP_SC_FAULT_STATUS,
	POWER_SUPPLY_PROP_SC_VBUS_ERROR_STATUS,
	POWER_SUPPLY_PROP_SC_CHIP_VENDOR,

};



static void sc8551_check_alarm_status(struct sc8551 *sc);
static void sc8551_check_fault_status(struct sc8551 *sc);

static int sc8551_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sc8551 *sc = power_supply_get_drvdata(psy);
	int result;
	int ret;
	u8 reg_val;


	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		sc8551_check_charge_enabled(sc, &sc->charge_enabled);
		val->intval = sc->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = sc->usb_present;
		break;
	case POWER_SUPPLY_PROP_SC_BATTERY_PRESENT:
		ret = sc8551_read_byte(sc, SC8551_REG_0D, &reg_val);
		if (!ret)
			sc->batt_present  = !!(reg_val & VBAT_INSERT);
		val->intval = sc->batt_present;
		break;
	case POWER_SUPPLY_PROP_SC_VBUS_PRESENT:
		ret = sc8551_read_byte(sc, SC8551_REG_0D, &reg_val);
		if (!ret)
			sc->vbus_present  = !!(reg_val & VBUS_INSERT);
		val->intval = sc->vbus_present;
		break;
	case POWER_SUPPLY_PROP_SC_BATTERY_VOLTAGE:
		ret = sc8551_get_adc_data(sc, ADC_VBAT, &result);
		if (!ret)
			sc->vbat_volt = result;

		val->intval = sc->vbat_volt;
		break;
	case POWER_SUPPLY_PROP_SC_BATTERY_CURRENT:
		ret = sc8551_get_adc_data(sc, ADC_IBAT, &result);
		if (!ret)
			sc->ibat_curr = result;

		val->intval = sc->ibat_curr;
		break;
	case POWER_SUPPLY_PROP_SC_BATTERY_TEMPERATURE:
		ret = sc8551_get_adc_data(sc, ADC_TBAT, &result);
		if (!ret)
			sc->bat_temp = result;

		val->intval = sc->bat_temp;
		break;
	case POWER_SUPPLY_PROP_SC_BUS_VOLTAGE:
		ret = sc8551_get_adc_data(sc, ADC_VBUS, &result);
		if (!ret)
			sc->vbus_volt = result;

		val->intval = sc->vbus_volt;
		break;
	case POWER_SUPPLY_PROP_SC_BUS_CURRENT:
		ret = sc8551_get_adc_data(sc, ADC_IBUS, &result);
		if (!ret)
			sc->ibus_curr = result;

		val->intval = sc->ibus_curr;
		break;
	case POWER_SUPPLY_PROP_SC_BUS_TEMPERATURE:
		ret = sc8551_get_adc_data(sc, ADC_TBUS, &result);
		if (!ret)
			sc->bus_temp = result;

		val->intval = sc->bus_temp;

		break;
	case POWER_SUPPLY_PROP_SC_DIE_TEMPERATURE:
		ret = sc8551_get_adc_data(sc, ADC_TDIE, &result);
		if (!ret)
			sc->die_temp = result;

		val->intval = sc->die_temp;
		break;
	case POWER_SUPPLY_PROP_SC_ALARM_STATUS:

		sc8551_check_alarm_status(sc);

		val->intval = ((sc->bat_ovp_alarm << BAT_OVP_ALARM_SHIFT)
			| (sc->bat_ocp_alarm << BAT_OCP_ALARM_SHIFT)
			| (sc->bat_ucp_alarm << BAT_UCP_ALARM_SHIFT)
			| (sc->bus_ovp_alarm << BUS_OVP_ALARM_SHIFT)
			| (sc->bus_ocp_alarm << BUS_OCP_ALARM_SHIFT)
			| (sc->bat_therm_alarm << BAT_THERM_ALARM_SHIFT)
			| (sc->bus_therm_alarm << BUS_THERM_ALARM_SHIFT)
			| (sc->die_therm_alarm << DIE_THERM_ALARM_SHIFT));
		break;

	case POWER_SUPPLY_PROP_SC_FAULT_STATUS:
		sc8551_check_fault_status(sc);

		val->intval = ((sc->bat_ovp_fault << BAT_OVP_FAULT_SHIFT)
			| (sc->bat_ocp_fault << BAT_OCP_FAULT_SHIFT)
			| (sc->bus_ovp_fault << BUS_OVP_FAULT_SHIFT)
			| (sc->bus_ocp_fault << BUS_OCP_FAULT_SHIFT)
			| (sc->bat_therm_fault << BAT_THERM_FAULT_SHIFT)
			| (sc->bus_therm_fault << BUS_THERM_FAULT_SHIFT)
			| (sc->die_therm_fault << DIE_THERM_FAULT_SHIFT));
		break;

	case POWER_SUPPLY_PROP_SC_VBUS_ERROR_STATUS:
		sc8551_check_vbus_error_status(sc);
		val->intval = sc->vbus_error;
		break;

	case POWER_SUPPLY_PROP_SC_CHIP_VENDOR:
		val->intval = sc->chip_vendor;
		break;
	default:
		return -EINVAL;

	}

	return 0;
}


static int sc8551_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct sc8551 *sc = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		sc8551_enable_charge(sc, val->intval);
		sc8551_check_charge_enabled(sc, &sc->charge_enabled);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		sc8551_set_present(sc, !!val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int sc8551_charger_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int sc8551_psy_register(struct sc8551 *sc)
{
	sc->psy_cfg.drv_data = sc;
	/* Mainline struct power_supply_config has no of_node. */


	if (sc->mode == SC8551_ROLE_MASTER)
		sc->psy_desc.name = "sc8551-master";
	else if (sc->mode == SC8551_ROLE_SLAVE)
		sc->psy_desc.name = "sc8551-slave";
	else
		sc->psy_desc.name = "sc8551-standalone";

	sc->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
	sc->psy_desc.properties = sc8551_charger_props;
	sc->psy_desc.num_properties = ARRAY_SIZE(sc8551_charger_props);
	sc->psy_desc.get_property = sc8551_charger_get_property;
	sc->psy_desc.set_property = sc8551_charger_set_property;
	sc->psy_desc.property_is_writeable = sc8551_charger_is_writeable;


	sc->fc2_psy = devm_power_supply_register(sc->dev,
			&sc->psy_desc, &sc->psy_cfg);
	if (IS_ERR(sc->fc2_psy)) {
		sc_err("failed to register fc2_psy\n");
		return PTR_ERR(sc->fc2_psy);
	}

	return 0;
}

static void sc8551_check_alarm_status(struct sc8551 *sc)
{
	u8 flag, stat;
	int ret;

	mutex_lock(&sc->data_lock);

	/* Read the flag registers to release the latched alarm lines. */
	sc8551_read_byte(sc, SC8551_REG_08, &flag);
	sc8551_read_byte(sc, SC8551_REG_2D, &flag);
	sc8551_read_byte(sc, SC8551_REG_0E, &flag);

	ret = sc8551_read_byte(sc, SC8551_REG_0D, &stat);
	if (ret)
		goto out;

	if (stat != sc->prev_alarm) {
		sc->prev_alarm = stat;
		sc->bat_ovp_alarm = !!(stat & BAT_OVP_ALARM);
		sc->bat_ocp_alarm = !!(stat & BAT_OCP_ALARM);
		sc->bus_ovp_alarm = !!(stat & BUS_OVP_ALARM);
		sc->bus_ocp_alarm = !!(stat & BUS_OCP_ALARM);
		sc->batt_present = !!(stat & VBAT_INSERT);
		sc->vbus_present = !!(stat & VBUS_INSERT);
		sc->bat_ucp_alarm = !!(stat & BAT_UCP_ALARM);
	}

	ret = sc8551_read_byte(sc, SC8551_REG_08, &stat);
	if (!ret && (stat & 0x50))
		sc_dbg("BUS_UCPOVP status 0x%02X\n", stat);

	ret = sc8551_read_byte(sc, SC8551_REG_0A, &stat);
	if (!ret && (stat & 0x02))
		sc_dbg("CONV_OCP status 0x%02X\n", stat);

out:
	mutex_unlock(&sc->data_lock);
}

static void sc8551_check_fault_status(struct sc8551 *sc)
{
	u8 flag, stat;
	int ret;

	mutex_lock(&sc->data_lock);

	/* Reading the flag register releases the latched fault line. */
	ret = sc8551_read_byte(sc, SC8551_REG_10, &stat);
	if (!ret && stat)
		pr_debug("fault status 0x%02X\n", stat);

	ret = sc8551_read_byte(sc, SC8551_REG_11, &flag);
	if (ret)
		goto out;

	if (flag != sc->prev_fault) {
		sc->prev_fault = flag;
		sc->bat_ovp_fault = !!(flag & BAT_OVP_FAULT);
		sc->bat_ocp_fault = !!(flag & BAT_OCP_FAULT);
		sc->bus_ovp_fault = !!(flag & BUS_OVP_FAULT);
		sc->bus_ocp_fault = !!(flag & BUS_OCP_FAULT);
		sc->bat_therm_fault = !!(flag & TS_BAT_FAULT);
		sc->bus_therm_fault = !!(flag & TS_BUS_FAULT);

		sc->bat_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
		sc->bus_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
	}

out:
	mutex_unlock(&sc->data_lock);
}

/*
 * interrupt does nothing, just info event chagne, other module could get info
 * through power supply interface
 */
static irqreturn_t sc8551_charger_interrupt(int irq, void *dev_id)
{
	struct sc8551 *sc = dev_id;

	mutex_lock(&sc->irq_complete);
	sc->irq_waiting = true;
	if (!sc->resume_completed) {
		dev_dbg(sc->dev, "IRQ triggered before device-resume\n");
		if (!sc->irq_disabled) {
			disable_irq_nosync(irq);
			sc->irq_disabled = true;
		}
		mutex_unlock(&sc->irq_complete);
		return IRQ_HANDLED;
	}
	sc->irq_waiting = false;
	/*
	 * Read-to-clear the latched alarm and fault flags; the interrupt
	 * line stays asserted until they are cleared, then publish the
	 * new state.
	 */
	sc8551_check_alarm_status(sc);
	sc8551_check_fault_status(sc);
	power_supply_changed(sc->fc2_psy);

	mutex_unlock(&sc->irq_complete);

	return IRQ_HANDLED;
}

static struct of_device_id sc8551_charger_match_table[] = {
	{
		.compatible = "sc,sc8551-standalone",
		.data = &sc8551_mode_data[SC8551_ROLE_STDALONE],
	},
	{
		.compatible = "sc,sc8551-master",
		.data = &sc8551_mode_data[SC8551_ROLE_MASTER],
	},
	{
		.compatible = "sc,sc8551-slave",
		.data = &sc8551_mode_data[SC8551_ROLE_SLAVE],
	},
	{},
};

static int sc8551_charger_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	const struct of_device_id *match;
	struct sc8551 *sc;
	int ret;

	sc = devm_kzalloc(dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->dev = dev;
	sc->client = client;

	mutex_init(&sc->i2c_rw_lock);
	mutex_init(&sc->data_lock);
	mutex_init(&sc->irq_complete);

	sc->resume_completed = true;
	sc->irq_waiting = false;
	sc->is_sc8551 = true;

	ret = sc8551_detect_device(sc);
	if (ret) {
		sc_err("No sc8551 device found\n");
		return -ENODEV;
	}

	match = of_match_node(sc8551_charger_match_table, dev->of_node);
	if (!match) {
		sc_err("device tree match not found\n");
		return -ENODEV;
	}
	sc->mode = *(int *)match->data;

	ret = sc8551_parse_dt(sc, dev);
	if (ret)
		return ret;

	ret = sc8551_init_device(sc);
	if (ret) {
		sc_err("Failed to init device\n");
		return ret;
	}

	ret = device_create_file(dev, &dev_attr_registers);
	if (ret)
		return ret;

	i2c_set_clientdata(client, sc);

	ret = sc8551_psy_register(sc);
	if (ret)
		goto err_remove_file;

	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sc8551_charger_interrupt,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						"sc8551 charger irq", sc);
		if (ret) {
			sc_err("request irq for irq=%d failed, ret =%d\n",
			       client->irq, ret);
			goto err_unregister_psy;
		}
		enable_irq_wake(client->irq);
		device_init_wakeup(dev, true);
	}

	sc8551_check_alarm_status(sc);
	sc8551_check_fault_status(sc);

	return 0;

err_unregister_psy:
	power_supply_unregister(sc->fc2_psy);
err_remove_file:
	device_remove_file(dev, &dev_attr_registers);
	mutex_destroy(&sc->irq_complete);
	mutex_destroy(&sc->data_lock);
	mutex_destroy(&sc->i2c_rw_lock);
	return ret;
}

static int sc8551_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8551 *sc = i2c_get_clientdata(client);

	mutex_lock(&sc->irq_complete);
	sc->resume_completed = false;
	mutex_unlock(&sc->irq_complete);

	return 0;
}

static int sc8551_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8551 *sc = i2c_get_clientdata(client);

	if (sc->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int sc8551_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8551 *sc = i2c_get_clientdata(client);


	mutex_lock(&sc->irq_complete);
	sc->resume_completed = true;
	if (sc->irq_waiting) {
		sc->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&sc->irq_complete);
		sc8551_charger_interrupt(client->irq, sc);
	} else {
		mutex_unlock(&sc->irq_complete);
	}

	power_supply_changed(sc->fc2_psy);

	return 0;
}
static void sc8551_charger_remove(struct i2c_client *client)
{
	struct sc8551 *sc = i2c_get_clientdata(client);

	sc8551_enable_adc(sc, false);
	device_remove_file(&client->dev, &dev_attr_registers);
	power_supply_unregister(sc->fc2_psy);

	mutex_destroy(&sc->data_lock);
	mutex_destroy(&sc->i2c_rw_lock);
	mutex_destroy(&sc->irq_complete);
}

static void sc8551_charger_shutdown(struct i2c_client *client)
{
	struct sc8551 *sc = i2c_get_clientdata(client);

	sc8551_enable_adc(sc, false);
}

static const struct dev_pm_ops sc8551_pm_ops = {
	.resume		= sc8551_resume,
	.suspend_noirq = sc8551_suspend_noirq,
	.suspend	= sc8551_suspend,
};

static const struct i2c_device_id sc8551_charger_id[] = {
	{"sc8551-standalone", SC8551_ROLE_STDALONE},
	{},
};

static struct i2c_driver sc8551_charger_driver = {
	.driver		= {
		.name	= "sc8551-charger",
		.of_match_table = sc8551_charger_match_table,
		.pm	= &sc8551_pm_ops,
	},
	.id_table	= sc8551_charger_id,

	.probe		= sc8551_charger_probe,
	.remove		= sc8551_charger_remove,
	.shutdown	= sc8551_charger_shutdown,
};

module_i2c_driver(sc8551_charger_driver);

MODULE_DESCRIPTION("SC SC8551 Charge Pump Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aiden-yu@southchip.com");
