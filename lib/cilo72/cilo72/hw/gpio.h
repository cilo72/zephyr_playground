/*
  Copyright (c) 2023 Daniel Zwirner
  SPDX-License-Identifier: MIT-0
*/

#pragma once

#include <zephyr/drivers/gpio.h>
#include <assert.h>

namespace cilo72
{
    namespace hw
    {
        class Gpio
        {
        public:
            enum class Level
            {
                Low,
                High
            };

            Gpio(const struct gpio_dt_spec * dt_gpio, Level level = Level::Low)
                : dt_gpio_(dt_gpio)
            {
                int ret;
                gpio_flags_t extra_flags = {};


                if (!gpio_is_ready_dt(dt_gpio))
                {
                    assert(0);
                    return;
                }

                if(level == Level::Low)
                {
                    extra_flags |= GPIO_OUTPUT_INACTIVE;
                }
                else
                {
                    extra_flags |= GPIO_OUTPUT_ACTIVE;
                }

                ret = gpio_pin_configure_dt(dt_gpio, extra_flags);


            }

            void set(Level level) const
            {
                if(level == Level::High)
                {
                    set();
                }
                else{
                    clear();
                }
            }

            void set() const
            {
                gpio_pin_set_dt(dt_gpio_, 1);
            }

            void clear() const
            {
                gpio_pin_set_dt(dt_gpio_, 0);
            }

            void toggle() const
            {
                gpio_pin_toggle_dt(dt_gpio_);
            }

            bool get() const
            {
                int ret = gpio_pin_get_dt(dt_gpio_);
                assert(ret >= 0);
                return ret ;
            }

        private:
            const struct gpio_dt_spec * dt_gpio_;
        };
    }
}