// SPDX-License-Identifier: GPL-2.0-only
/*
 * LED driver : leds-ktd2691.c
 *
 * Copyright (C) 2026 Onur BÜBER <onurbuber@engineer.com>
 *
 * The KTD2691 is an inductorless single flash LED driver with one
 * programmable current source that is shared by the flash, torch and IR
 * modes. Everything except the STROBE pin is programmed over I2C.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/led-class-flash.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>

#define KTD2691_REG_ENABLE		0x01
#define KTD2691_REG_CONFIG		0x02
#define KTD2691_REG_FLASH_CURRENT	0x03
#define KTD2691_REG_TORCH_CURRENT	0x04
#define KTD2691_REG_FAULT		0x05
#define KTD2691_REG_ID			0x06

/* Enable register */
#define KTD2691_MODE_MASK		GENMASK(1, 0)
#define KTD2691_MODE_STANDBY		0x00
#define KTD2691_MODE_IR			0x01
#define KTD2691_MODE_TORCH		0x02
#define KTD2691_MODE_FLASH		0x03
#define KTD2691_ENABLE_STROBE_PIN	BIT(2)
#define KTD2691_STROBE_EDGE_TRIGGER	BIT(3)
#define KTD2691_ENABLE_LVP		BIT(4)

/* Configuration register */
#define KTD2691_LVP_MASK		GENMASK(7, 5)
#define KTD2691_TIMEOUT_MASK		GENMASK(4, 1)
#define KTD2691_TIMEOUT_SHIFT		1
#define KTD2691_TORCH_NO_RAMP		BIT(0)

/* Fault register, cleared by reading it */
#define KTD2691_FAULT_TIMEOUT		BIT(0)
#define KTD2691_FAULT_UVLO		BIT(1)
#define KTD2691_FAULT_TSD		BIT(2)
#define KTD2691_FAULT_LED_SHORT		BIT(5)
#define KTD2691_FAULT_LVP		BIT(6)

/* Device ID register */
#define KTD2691_DEVICE_ID_MASK		GENMASK(5, 3)
#define KTD2691_DEVICE_ID		0x01
#define KTD2691_SOFTWARE_RESET		BIT(7)

#define KTD2691_MAX_CURRENT_CODE	127
#define KTD2691_FLASH_MAX_UA		1500000
#define KTD2691_TORCH_MAX_UA		376000

/* Flash time-out steps, indexed by the register value */
static const u32 ktd2691_timeouts_us[] = {
	40000, 80000, 120000, 160000, 200000, 240000, 280000, 320000,
	360000, 400000, 600000, 800000, 1000000, 1200000, 1400000, 1600000,
};

struct ktd2691 {
	struct led_classdev_flash fled_cdev;
	struct i2c_client *client;
	struct mutex lock;	/* serialises register accesses */
	struct regulator *regulator;
	struct gpio_desc *strobe_gpio;
	u8 torch_max_code;
	u32 torch_max_ua;
	bool use_strobe_pin;
};

static struct ktd2691 *fled_cdev_to_led(struct led_classdev_flash *fled_cdev)
{
	return container_of(fled_cdev, struct ktd2691, fled_cdev);
}

static int ktd2691_write(struct ktd2691 *led, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(led->client, reg, value);
}

static int ktd2691_read(struct ktd2691 *led, u8 reg)
{
	return i2c_smbus_read_byte_data(led->client, reg);
}

static int ktd2691_set_mode(struct ktd2691 *led, u8 mode)
{
	u8 value = mode & KTD2691_MODE_MASK;

	if (led->use_strobe_pin)
		value |= KTD2691_ENABLE_STROBE_PIN;

	return ktd2691_write(led, KTD2691_REG_ENABLE, value);
}

/*
 * Both current registers take a seven bit code where zero is the lowest
 * current the part can deliver and 127 the maximum of that mode.
 */
static u8 ktd2691_ua_to_code(u32 microamp, u32 max_microamp, u32 max_code)
{
	u32 code;

	code = DIV_ROUND_CLOSEST(microamp * max_code, max_microamp);
	return clamp_t(u32, code, 0, max_code);
}

static int ktd2691_led_brightness_set(struct led_classdev *led_cdev,
				      enum led_brightness brightness)
{
	struct led_classdev_flash *fled_cdev = lcdev_to_flcdev(led_cdev);
	struct ktd2691 *led = fled_cdev_to_led(fled_cdev);
	u32 microamp;
	int ret;

	mutex_lock(&led->lock);

	if (!brightness) {
		ret = ktd2691_set_mode(led, KTD2691_MODE_STANDBY);
		goto out;
	}

	microamp = DIV_ROUND_UP((u32)brightness * led->torch_max_ua,
				led_cdev->max_brightness);

	ret = ktd2691_write(led, KTD2691_REG_TORCH_CURRENT,
			    ktd2691_ua_to_code(microamp, KTD2691_TORCH_MAX_UA,
					       KTD2691_MAX_CURRENT_CODE));
	if (ret)
		goto out;

	ret = ktd2691_set_mode(led, KTD2691_MODE_TORCH);
out:
	mutex_unlock(&led->lock);

	return ret;
}

static int ktd2691_flash_brightness_set(struct led_classdev_flash *fled_cdev,
					u32 brightness)
{
	struct ktd2691 *led = fled_cdev_to_led(fled_cdev);
	int ret;

	mutex_lock(&led->lock);
	ret = ktd2691_write(led, KTD2691_REG_FLASH_CURRENT,
			    ktd2691_ua_to_code(brightness, KTD2691_FLASH_MAX_UA,
					       KTD2691_MAX_CURRENT_CODE));
	mutex_unlock(&led->lock);

	return ret;
}

static int ktd2691_flash_brightness_get(struct led_classdev_flash *fled_cdev,
					u32 *brightness)
{
	struct ktd2691 *led = fled_cdev_to_led(fled_cdev);
	int code;

	mutex_lock(&led->lock);
	code = ktd2691_read(led, KTD2691_REG_FLASH_CURRENT);
	mutex_unlock(&led->lock);

	if (code < 0)
		return code;

	*brightness = (code & KTD2691_MAX_CURRENT_CODE) *
		      KTD2691_FLASH_MAX_UA / KTD2691_MAX_CURRENT_CODE;

	return 0;
}

static u8 ktd2691_timeout_to_code(u32 timeout_us)
{
	u8 code;

	for (code = 0; code < ARRAY_SIZE(ktd2691_timeouts_us) - 1; code++)
		if (timeout_us <= ktd2691_timeouts_us[code])
			return code;

	return ARRAY_SIZE(ktd2691_timeouts_us) - 1;
}

static int ktd2691_flash_strobe_set(struct led_classdev_flash *fled_cdev,
				    bool state)
{
	struct ktd2691 *led = fled_cdev_to_led(fled_cdev);
	struct i2c_client *client = led->client;
	int config, ret;

	mutex_lock(&led->lock);

	if (!state) {
		if (led->use_strobe_pin)
			gpiod_set_value_cansleep(led->strobe_gpio, 0);

		ret = ktd2691_set_mode(led, KTD2691_MODE_STANDBY);
		fled_cdev->led_cdev.brightness = LED_OFF;
		goto out;
	}

	config = ktd2691_read(led, KTD2691_REG_CONFIG);
	if (config < 0) {
		ret = config;
		goto out;
	}

	config &= ~KTD2691_TIMEOUT_MASK;
	config |= ktd2691_timeout_to_code(fled_cdev->timeout.val)
		  << KTD2691_TIMEOUT_SHIFT;

	ret = ktd2691_write(led, KTD2691_REG_CONFIG, config);
	if (ret)
		goto out;

	ret = ktd2691_set_mode(led, KTD2691_MODE_FLASH);
	if (ret)
		goto out;

	if (led->use_strobe_pin)
		gpiod_set_value_cansleep(led->strobe_gpio, 1);

	dev_dbg(&client->dev, "flash pulse, timeout %u us\n",
		fled_cdev->timeout.val);
out:
	mutex_unlock(&led->lock);

	return ret;
}

static int ktd2691_flash_strobe_get(struct led_classdev_flash *fled_cdev,
				    bool *state)
{
	struct ktd2691 *led = fled_cdev_to_led(fled_cdev);
	int mode;

	mutex_lock(&led->lock);
	mode = ktd2691_read(led, KTD2691_REG_ENABLE);
	mutex_unlock(&led->lock);

	if (mode < 0)
		return mode;

	*state = (mode & KTD2691_MODE_MASK) == KTD2691_MODE_FLASH;

	return 0;
}

static int ktd2691_flash_timeout_set(struct led_classdev_flash *fled_cdev,
				     u32 timeout)
{
	/* Applied together with the current when the pulse is started. */
	return 0;
}

static int ktd2691_flash_fault_get(struct led_classdev_flash *fled_cdev,
				   u32 *fault)
{
	struct ktd2691 *led = fled_cdev_to_led(fled_cdev);
	int flags;

	mutex_lock(&led->lock);
	flags = ktd2691_read(led, KTD2691_REG_FAULT);
	mutex_unlock(&led->lock);

	if (flags < 0)
		return flags;

	*fault = 0;

	if (flags & KTD2691_FAULT_LED_SHORT)
		*fault |= LED_FAULT_SHORT_CIRCUIT;
	if (flags & KTD2691_FAULT_TSD)
		*fault |= LED_FAULT_OVER_TEMPERATURE;
	if (flags & KTD2691_FAULT_UVLO)
		*fault |= LED_FAULT_UNDER_VOLTAGE;
	if (flags & KTD2691_FAULT_LVP)
		*fault |= LED_FAULT_INPUT_VOLTAGE;
	if (flags & KTD2691_FAULT_TIMEOUT)
		*fault |= LED_FAULT_TIMEOUT;

	return 0;
}

static const struct led_flash_ops ktd2691_flash_ops = {
	.flash_brightness_set = ktd2691_flash_brightness_set,
	.flash_brightness_get = ktd2691_flash_brightness_get,
	.strobe_set = ktd2691_flash_strobe_set,
	.strobe_get = ktd2691_flash_strobe_get,
	.timeout_set = ktd2691_flash_timeout_set,
	.fault_get = ktd2691_flash_fault_get,
};

static int ktd2691_parse_dt(struct ktd2691 *led, struct device *dev,
			    struct led_init_data *init_data)
{
	struct led_classdev_flash *fled_cdev = &led->fled_cdev;
	struct led_flash_setting *setting;
	struct fwnode_handle *child;
	u32 flash_max_ua, torch_max_ua, timeout_us;
	int ret;

	led->regulator = devm_regulator_get_optional(dev, "vin");
	if (IS_ERR(led->regulator)) {
		ret = PTR_ERR(led->regulator);
		led->regulator = NULL;
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret, "cannot get vin supply\n");
	}

	if (led->regulator) {
		ret = regulator_enable(led->regulator);
		if (ret)
			return dev_err_probe(dev, ret, "failed to enable vin\n");
	}

	led->strobe_gpio = devm_gpiod_get_optional(dev, "strobe", GPIOD_OUT_LOW);
	if (IS_ERR(led->strobe_gpio))
		return dev_err_probe(dev, PTR_ERR(led->strobe_gpio),
				     "cannot get strobe-gpios\n");

	led->use_strobe_pin = !!led->strobe_gpio;

	child = fwnode_get_next_available_child_node(dev_fwnode(dev), NULL);
	if (!child)
		return dev_err_probe(dev, -EINVAL,
				     "no child node for the LED\n");

	init_data->fwnode = child;
	init_data->default_label = "ktd2691:flash";

	ret = fwnode_property_read_u32(child, "flash-max-microamp",
				       &flash_max_ua);
	if (ret) {
		dev_err(dev, "failed to parse flash-max-microamp\n");
		goto out;
	}

	ret = fwnode_property_read_u32(child, "led-max-microamp", &torch_max_ua);
	if (ret) {
		dev_err(dev, "failed to parse led-max-microamp\n");
		goto out;
	}

	ret = fwnode_property_read_u32(child, "flash-max-timeout-us",
				       &timeout_us);
	if (ret) {
		dev_err(dev, "failed to parse flash-max-timeout-us\n");
		goto out;
	}

	if (flash_max_ua > KTD2691_FLASH_MAX_UA || !flash_max_ua ||
	    torch_max_ua > KTD2691_TORCH_MAX_UA || !torch_max_ua) {
		ret = -EINVAL;
		dev_err(dev, "current out of the supported range\n");
		goto out;
	}

	setting = &fled_cdev->brightness;
	setting->min = 0;
	setting->max = flash_max_ua;
	setting->step = 1;
	setting->val = flash_max_ua;

	setting = &fled_cdev->timeout;
	setting->min = ktd2691_timeouts_us[0];
	setting->max = min_t(u32, timeout_us,
			     ktd2691_timeouts_us[ARRAY_SIZE(ktd2691_timeouts_us) - 1]);
	setting->step = 1;
	setting->val = setting->max;

	/*
	 * One brightness step per code keeps the sysfs scale linear in
	 * current, but cap it so that the configured torch current is what
	 * full brightness actually delivers.
	 */
	led->torch_max_ua = torch_max_ua;
	led->torch_max_code = DIV_ROUND_CLOSEST(torch_max_ua *
						KTD2691_MAX_CURRENT_CODE,
						KTD2691_TORCH_MAX_UA);
	led->fled_cdev.led_cdev.max_brightness = led->torch_max_code + 1;
	ret = 0;
out:
	fwnode_handle_put(child);

	return ret;
}

static int ktd2691_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct led_classdev_flash *fled_cdev;
	struct led_classdev *led_cdev;
	struct led_init_data init_data = {};
	struct ktd2691 *led;
	int id, ret;

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->client = client;
	mutex_init(&led->lock);

	fled_cdev = &led->fled_cdev;
	led_cdev = &fled_cdev->led_cdev;

	ret = ktd2691_parse_dt(led, dev, &init_data);
	if (ret)
		return ret;

	/* Reset the part so that it never starts in the previous mode. */
	ret = ktd2691_write(led, KTD2691_REG_ID, KTD2691_SOFTWARE_RESET);
	if (ret)
		return dev_err_probe(dev, ret, "failed to reset the device\n");

	usleep_range(1000, 2000);

	id = ktd2691_read(led, KTD2691_REG_ID);
	if (id < 0)
		return dev_err_probe(dev, id, "failed to read the device ID\n");

	if (((id & KTD2691_DEVICE_ID_MASK) >> 3) != KTD2691_DEVICE_ID)
		return dev_err_probe(dev, -ENODEV,
				     "unexpected device ID 0x%02x\n", id);

	fled_cdev->ops = &ktd2691_flash_ops;

	led_cdev->brightness_set_blocking = ktd2691_led_brightness_set;
	led_cdev->flags |= LED_CORE_SUSPENDRESUME | LED_DEV_CAP_FLASH;

	ret = ktd2691_set_mode(led, KTD2691_MODE_STANDBY);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enter standby\n");

	i2c_set_clientdata(client, led);

	ret = led_classdev_flash_register_ext(dev, fled_cdev, &init_data);
	if (ret)
		return dev_err_probe(dev, ret, "can't register the LED\n");

	dev_info(dev, "KTD2691 flash LED, revision %u\n", id & 0x7);

	return 0;
}

static const struct of_device_id ktd2691_of_match[] = {
	{ .compatible = "kinetic,ktd2691" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ktd2691_of_match);

static struct i2c_driver ktd2691_driver = {
	.driver = {
		.name = "ktd2691",
		.of_match_table = ktd2691_of_match,
	},
	.probe = ktd2691_probe,
};
module_i2c_driver(ktd2691_driver);

MODULE_DESCRIPTION("Kinetic KTD2691 flash LED driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Onur BÜBER <onurbuber@engineer.com>");
