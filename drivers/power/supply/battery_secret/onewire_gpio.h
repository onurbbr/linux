/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ONEWIRE_GPIO_H__
#define __ONEWIRE_GPIO_H__

#include <linux/types.h>

void Delay_us(unsigned int us);
void Delay_ns(unsigned int ns);
unsigned char ow_reset(void);
unsigned char read_byte(void);
void write_byte(char val);
void Software_Reset(void);
bool onewire_gpio_is_ready(void);

#endif /* __ONEWIRE_GPIO_H__ */
