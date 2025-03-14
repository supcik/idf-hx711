/**
 ******************************************************************************
 * @file        : hx711.hpp
 * @brief       : HX711 controller
 * @author      : Jacques Supcik <jacques@supcik.net>
 * @date        : 20 November 2024
 ******************************************************************************
 * @copyright   : Copyright (c) 2024 Jacques Supcik
 * @attention   : SPDX-License-Identifier: MIT
 ******************************************************************************
 * @details     : HX711 controller
 ******************************************************************************
 */

#pragma once

#include <inttypes.h>
#include <unistd.h>

#include "driver/dedic_gpio.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class HX711 {
   public:
    static const int32_t kUndefined = 0x7fffffff;
    enum class Mode { kChannelA128 = 0, kChannelB32 = 1, kChannelA64 = 2 };
    HX711(gpio_num_t clock_pin, gpio_num_t data_pin, Mode mode = Mode::kChannelA128);
    int32_t Read(TickType_t tick_to_wait);

   private:
    gpio_num_t clock_pin_;
    gpio_num_t data_pin_;
    int clock_cycles_;
    dedic_gpio_bundle_handle_t clock_;
    dedic_gpio_bundle_handle_t data_;
    gptimer_handle_t gptimer_;
};
