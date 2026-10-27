/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Bit-banged 1-Wire master used to talk to the Maxim DS28E16 battery
 * authenticator on Xiaomi boards.
 *
 * The protocol timing is too tight for gpiolib: a read slot must sample the
 * line 500ns after releasing it, which requires switching the pad between
 * input and output in a couple of register accesses. The driver therefore
 * drives the TLMM pad directly, using the register window described by the
 * "mi,onewire-gpio-cfg-addr" property, and only uses the GPIO descriptor to
 * claim the line and set up its initial direction.
 *
 * Copyright (c) 2016 Xiaomi Inc.
 */

#define pr_fmt(fmt) "[onewire] " fmt

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include "onewire_gpio.h"

#define DRV_STRENGTH_16MA (0x7 << 6)
#define GPIO_ENABLE (0x1 << 12)
#define GPIO_OUTPUT (0x1 << 9)
#define GPIO_INPUT (0x0 << 9)
#define GPIO_PULL_UP 0x3
#define OUTPUT_HIGH 0x2
#define OUTPUT_LOW 0x0

struct onewire_gpio_data {
	struct platform_device *pdev;
	struct device *dev;

	struct gpio_desc *ow_desc;

	void __iomem *gpio_in_out_reg;
	void __iomem *gpio_cfg_reg;

	raw_spinlock_t lock;

	struct pinctrl *ow_gpio_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_sleep;

	unsigned int onewire_gpio_cfg_addr;
	unsigned int onewire_gpio_level_addr;
};

/*
 * The DS28E16 driver has no phandle to this controller - the two nodes are
 * independent platform devices - so the bus primitives below are exported
 * and look the instance up through this pointer. It is only published once
 * probe has fully succeeded and cleared in remove().
 */
static struct onewire_gpio_data *onewire_data;

static void ow_config_out(struct onewire_gpio_data *ow)
{
	writel_relaxed(GPIO_ENABLE | DRV_STRENGTH_16MA | GPIO_OUTPUT |
			       GPIO_PULL_UP,
		       ow->gpio_cfg_reg);
}

static void ow_config_in(struct onewire_gpio_data *ow)
{
	writel_relaxed(GPIO_ENABLE | DRV_STRENGTH_16MA | GPIO_INPUT |
			       GPIO_PULL_UP,
		       ow->gpio_cfg_reg);
}

static void ow_out_high(struct onewire_gpio_data *ow)
{
	writel_relaxed(OUTPUT_HIGH, ow->gpio_in_out_reg);
}

static void ow_out_low(struct onewire_gpio_data *ow)
{
	writel_relaxed(OUTPUT_LOW, ow->gpio_in_out_reg);
}

void Delay_us(unsigned int us)
{
	udelay(us);
}
EXPORT_SYMBOL_GPL(Delay_us);

void Delay_ns(unsigned int ns)
{
	ndelay(ns);
}
EXPORT_SYMBOL_GPL(Delay_ns);

/*
 * Recovery for a bus that was left in an unknown state: hold the line low
 * long enough for every device to see a reset, then release it. Repeated
 * calls are pointless, hence the attempt limit.
 */
void Software_Reset(void)
{
	struct onewire_gpio_data *ow = onewire_data;
	static int step;
	unsigned int i;

	if (!ow)
		return;

	if (step > 2)
		return;

	ow_config_out(ow);
	ow_out_low(ow);
	for (i = 0; i < 4000; i++)
		Delay_us(100); /* pull down for 400ms */
	ow_out_high(ow);
	for (i = 0; i < 40; i++)
		Delay_us(100); /* release for 4ms */
	step++;
}
EXPORT_SYMBOL_GPL(Software_Reset);

unsigned char ow_reset(void)
{
	struct onewire_gpio_data *ow = onewire_data;
	unsigned long flags;
	unsigned char presence;

	if (!ow)
		return 0x01;

	raw_spin_lock_irqsave(&ow->lock, flags);
	ow_config_out(ow);
	ow_out_low(ow);
	Delay_us(50);
	ow_out_high(ow);
	ow_config_in(ow);
	Delay_us(7);
	presence = (unsigned char)readl_relaxed(ow->gpio_in_out_reg) & 0x01;
	Delay_us(50);
	raw_spin_unlock_irqrestore(&ow->lock, flags);

	return presence;
}
EXPORT_SYMBOL_GPL(ow_reset);

static unsigned char read_bit(struct onewire_gpio_data *ow)
{
	unsigned int val;

	ow_config_out(ow);
	ow_out_low(ow);
	Delay_us(1);
	ow_config_in(ow);
	Delay_ns(500);
	val = readl_relaxed(ow->gpio_in_out_reg);
	Delay_us(5);
	ow_out_high(ow);
	ow_config_out(ow);
	Delay_us(6);

	return (unsigned char)val & 0x01;
}

static void write_bit(struct onewire_gpio_data *ow, char bitval)
{
	ow_out_low(ow);
	Delay_us(1);
	if (bitval)
		ow_out_high(ow);
	Delay_us(10);
	ow_out_high(ow);
	Delay_us(6);
}

unsigned char read_byte(void)
{
	struct onewire_gpio_data *ow = onewire_data;
	unsigned long flags;
	unsigned char value = 0;
	unsigned int i;

	if (!ow)
		return 0xFF;

	raw_spin_lock_irqsave(&ow->lock, flags);
	for (i = 0; i < 8; i++)
		if (read_bit(ow))
			value |= 0x01 << i;

	raw_spin_unlock_irqrestore(&ow->lock, flags);

	return value;
}
EXPORT_SYMBOL_GPL(read_byte);

void write_byte(char val)
{
	struct onewire_gpio_data *ow = onewire_data;
	unsigned long flags;
	unsigned int i;

	if (!ow)
		return;

	raw_spin_lock_irqsave(&ow->lock, flags);
	ow_config_out(ow);
	for (i = 0; i < 8; i++)
		write_bit(ow, (val >> i) & 0x01);

	raw_spin_unlock_irqrestore(&ow->lock, flags);
}
EXPORT_SYMBOL_GPL(write_byte);

/* True once this driver has probed; ds28e16 defers on this. */
bool onewire_gpio_is_ready(void)
{
	return onewire_data != NULL;
}
EXPORT_SYMBOL_GPL(onewire_gpio_is_ready);

static int onewire_gpio_parse_dt(struct device *dev,
				 struct onewire_gpio_data *ow)
{
	unsigned int gpio_reg[2];
	int ret;

	ret = of_property_read_u32_array(
		dev->of_node, "mi,onewire-gpio-cfg-addr", gpio_reg, 2);
	if (ret) {
		dev_err(dev, "missing mi,onewire-gpio-cfg-addr: %d\n", ret);
		return ret;
	}

	ow->onewire_gpio_cfg_addr = gpio_reg[0];
	ow->onewire_gpio_level_addr = gpio_reg[0] + gpio_reg[1];

	return 0;
}

static int onewire_gpio_pinctrl_init(struct onewire_gpio_data *ow)
{
	int ret;

	ow->ow_gpio_pinctrl = devm_pinctrl_get(ow->dev);
	if (IS_ERR(ow->ow_gpio_pinctrl))
		return dev_err_probe(ow->dev, PTR_ERR(ow->ow_gpio_pinctrl),
				     "failed to get pinctrl\n");

	ow->pinctrl_state_active =
		pinctrl_lookup_state(ow->ow_gpio_pinctrl, "onewire_active");
	if (IS_ERR(ow->pinctrl_state_active))
		return dev_err_probe(ow->dev, PTR_ERR(ow->pinctrl_state_active),
				     "missing onewire_active pinstate\n");

	ow->pinctrl_state_sleep =
		pinctrl_lookup_state(ow->ow_gpio_pinctrl, "onewire_sleep");
	if (IS_ERR(ow->pinctrl_state_sleep))
		return dev_err_probe(ow->dev, PTR_ERR(ow->pinctrl_state_sleep),
				     "missing onewire_sleep pinstate\n");

	ret = pinctrl_select_state(ow->ow_gpio_pinctrl,
				   ow->pinctrl_state_active);
	if (ret)
		dev_err(ow->dev, "failed to select active pinstate: %d\n", ret);

	return ret;
}

static int onewire_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct onewire_gpio_data *ow;
	int ret;

	ow = devm_kzalloc(dev, sizeof(*ow), GFP_KERNEL);
	if (!ow)
		return -ENOMEM;

	ow->pdev = pdev;
	ow->dev = dev;
	raw_spin_lock_init(&ow->lock);
	platform_set_drvdata(pdev, ow);

	ret = onewire_gpio_parse_dt(dev, ow);
	if (ret)
		return ret;

	ret = onewire_gpio_pinctrl_init(ow);
	if (ret)
		return ret;

	ow->ow_desc = devm_gpiod_get(dev, "xiaomi,ow", GPIOD_ASIS);
	if (IS_ERR(ow->ow_desc))
		return dev_err_probe(dev, PTR_ERR(ow->ow_desc),
				     "failed to get the 1-Wire gpio\n");

	ret = gpiod_direction_output(ow->ow_desc, 1);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to drive the 1-Wire gpio\n");

	ow->gpio_in_out_reg =
		devm_ioremap(dev, ow->onewire_gpio_level_addr, 0x4);
	ow->gpio_cfg_reg = devm_ioremap(dev, ow->onewire_gpio_cfg_addr, 0x4);
	if (!ow->gpio_in_out_reg || !ow->gpio_cfg_reg)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to map the TLMM registers\n");

	onewire_data = ow;

	return 0;
}

static void onewire_gpio_remove(struct platform_device *pdev)
{
	struct onewire_gpio_data *ow = platform_get_drvdata(pdev);

	/* Invalidate the bit-bang entry points; they all NULL-check. */
	if (onewire_data == ow)
		onewire_data = NULL;
}

static const struct of_device_id onewire_gpio_dt_match[] = {
	{ .compatible = "xiaomi,onewire_gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, onewire_gpio_dt_match);

static struct platform_driver onewire_gpio_driver = {
	.driver = {
		.name = "onewire_gpio",
		.of_match_table = onewire_gpio_dt_match,
	},
	.probe = onewire_gpio_probe,
	.remove = onewire_gpio_remove,
};
module_platform_driver(onewire_gpio_driver);

MODULE_AUTHOR("Xiaomi Inc.");
MODULE_DESCRIPTION(
	"Bit-banged 1-Wire master for the DS28E16 battery authenticator");
MODULE_LICENSE("GPL");
