/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include "cilo72/hw/gpio.h"
#include "cilo72/ic/tmc5160.h"
#include "cilo72/motion/tmc5160.h"
#include <tmc5xxx.h>


const struct device *tmc5xxx0 = DEVICE_DT_GET(DT_ALIAS(tmc5xxx0));

const struct gpio_dt_spec dt_led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

#define SLEEP_TIME_MS   500

int main(void)
{
	cilo72::hw::Gpio        led(&dt_led0, cilo72::hw::Gpio::Level::Low);
	cilo72::ic::Tmc5160     tmc(tmc5xxx0, 75);
	cilo72::motion::Tmc5160 axis(tmc, 256, 200, 256*200);
	int32_t distance = 256*200;

	usb_enable(NULL);

	axis.ic().enable(true);
	axis.power(true);
	

	while (1) 
	{
		if(axis.isPositionReached())
		{
			axis.moveRel(distance, 50000, 20000, 100, 300);
			distance *=-1;

		}
		led.toggle();
		k_msleep(SLEEP_TIME_MS);

	}
	return 0;
}
