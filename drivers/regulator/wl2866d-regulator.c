// SPDX-License-Identifier: GPL-2.0-only
/*
 * Willsemi WL2866D camera PMIC regulator driver
 *
 * The WL2866D is a four channel LDO block used to power camera sensors:
 * two digital rails (DVDD1/DVDD2) and two analog rails (AVDD1/AVDD2). It
 * sits on a normal I2C bus behind an enable pin that has to be driven high
 * before the chip answers any transfer.
 *
 * Register map, as used by the vendor driver:
 *
 *   0x02  output discharge control (global to all four outputs)
 *   0x03  DVDD1 voltage          Vout = 600 mV + code * 6 mV
 *   0x04  DVDD2 voltage          Vout = 600 mV + code * 6 mV
 *   0x05  AVDD1 voltage          Vout = 1200 mV + code * 12.5 mV
 *   0x06  AVDD2 voltage          Vout = 1200 mV + code * 12.5 mV
 *   0x0e  output enable          bit0 DVDD1, bit1 DVDD2, bit2 AVDD1, bit3 AVDD2
 *
 * The vendor driver treats the reset value of the DVDD1 register as a chip
 * ID and rejects anything that is not 0x64 or 0x55. Those are really just
 * voltage codes: 0x64 is 1200 mV and 0x55 is 1110 mV. Once a bootloader or
 * a previous OS has programmed a different rail voltage the register holds
 * that code instead, so the value cannot be used to identify the part. Keep
 * the read as a presence check and ignore what it returns.
 *
 * Register 0x02 is deliberately left alone. It is one global control for all
 * four outputs, so pulsing it to bleed the output capacitors (as the vendor
 * driver does when a rail is disabled) would also discharge the rails that
 * are still in use by another sensor. Letting the outputs ramp down on their
 * own is harmless and avoids that.
 */

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#define WL2866D_REG_DVDD1_VOLT		0x03
#define WL2866D_REG_DVDD2_VOLT		0x04
#define WL2866D_REG_AVDD1_VOLT		0x05
#define WL2866D_REG_AVDD2_VOLT		0x06
#define WL2866D_REG_ENABLE		0x0e

#define WL2866D_DVDD_MIN_UV		600000
#define WL2866D_DVDD_STEP_UV		6000
#define WL2866D_DVDD_N_VOLTAGES		151	/* (1500 - 600) / 6 + 1 */

#define WL2866D_AVDD_MIN_UV		1200000
#define WL2866D_AVDD_STEP_UV		12500
#define WL2866D_AVDD_N_VOLTAGES		145	/* (3000 - 1200) / 12.5 + 1 */

/* The enable pin has to settle before the chip responds on I2C. */
#define WL2866D_ENABLE_SETTLE_US	2000

static int wl2866d_reg_enable(struct regulator_dev *rdev)
{
	return regmap_set_bits(rdev_get_regmap(rdev), WL2866D_REG_ENABLE,
			       rdev->desc->enable_mask);
}

static int wl2866d_reg_disable(struct regulator_dev *rdev)
{
	return regmap_clear_bits(rdev_get_regmap(rdev), WL2866D_REG_ENABLE,
				 rdev->desc->enable_mask);
}

static const struct regulator_ops wl2866d_reg_ops = {
	.enable		= wl2866d_reg_enable,
	.disable	= wl2866d_reg_disable,
	.is_enabled	= regulator_is_enabled_regmap,
	.list_voltage	= regulator_list_voltage_linear,
	.map_voltage	= regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

#define WL2866D_REG_DESC(_name, _id, _vreg, _min, _step, _n)	\
	{							\
		.name		= _name,			\
		.of_match	= _name,			\
		.regulators_node = "regulators",		\
		.id		= _id,				\
		.ops		= &wl2866d_reg_ops,		\
		.type		= REGULATOR_VOLTAGE,		\
		.owner		= THIS_MODULE,			\
		.min_uV		= _min,				\
		.uV_step	= _step,			\
		.n_voltages	= _n,				\
		.vsel_reg	= _vreg,			\
		.vsel_mask	= 0xff,				\
		.enable_reg	= WL2866D_REG_ENABLE,		\
		.enable_mask	= BIT(_id),			\
		.enable_time	= 1000,				\
	}

static const struct regulator_desc wl2866d_regs[] = {
	WL2866D_REG_DESC("dvdd1", 0, WL2866D_REG_DVDD1_VOLT,
			 WL2866D_DVDD_MIN_UV, WL2866D_DVDD_STEP_UV,
			 WL2866D_DVDD_N_VOLTAGES),
	WL2866D_REG_DESC("dvdd2", 1, WL2866D_REG_DVDD2_VOLT,
			 WL2866D_DVDD_MIN_UV, WL2866D_DVDD_STEP_UV,
			 WL2866D_DVDD_N_VOLTAGES),
	WL2866D_REG_DESC("avdd1", 2, WL2866D_REG_AVDD1_VOLT,
			 WL2866D_AVDD_MIN_UV, WL2866D_AVDD_STEP_UV,
			 WL2866D_AVDD_N_VOLTAGES),
	WL2866D_REG_DESC("avdd2", 3, WL2866D_REG_AVDD2_VOLT,
			 WL2866D_AVDD_MIN_UV, WL2866D_AVDD_STEP_UV,
			 WL2866D_AVDD_N_VOLTAGES),
};

static const struct regmap_config wl2866d_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= 0x0f,
};

static int wl2866d_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config cfg = {};
	struct gpio_desc *enable;
	struct regmap *regmap;
	unsigned int id;
	int i, ret;

	regmap = devm_regmap_init_i2c(client, &wl2866d_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to init regmap\n");

	/*
	 * The chip only answers on I2C once the enable pin is high, and it
	 * needs a moment to come out of power-on reset afterwards.
	 */
	enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(enable))
		return dev_err_probe(dev, PTR_ERR(enable),
				     "failed to get enable gpio\n");

	usleep_range(WL2866D_ENABLE_SETTLE_US, WL2866D_ENABLE_SETTLE_US + 1000);

	ret = regmap_read(regmap, WL2866D_REG_DVDD1_VOLT, &id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read DVDD1\n");

	cfg.dev = dev;
	cfg.regmap = regmap;

	for (i = 0; i < ARRAY_SIZE(wl2866d_regs); i++) {
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(dev, &wl2866d_regs[i], &cfg);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     wl2866d_regs[i].name);
	}

	return 0;
}

static const struct of_device_id wl2866d_of_match[] = {
	{ .compatible = "willsemi,wl2866d" },
	{ .compatible = "ovti,wl2866d" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wl2866d_of_match);

static const struct i2c_device_id wl2866d_id[] = {
	{ "wl2866d" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, wl2866d_id);

static struct i2c_driver wl2866d_regulator_driver = {
	.driver = {
		.name = "wl2866d-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = wl2866d_of_match,
	},
	.probe = wl2866d_probe,
	.id_table = wl2866d_id,
};
module_i2c_driver(wl2866d_regulator_driver);

MODULE_DESCRIPTION("Willsemi WL2866D camera PMIC regulator driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Onur BÜBER <onurbuber@engineer.com>");
