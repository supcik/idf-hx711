/**
 ******************************************************************************
 * @file        : hx711.cpp
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

#include "hx711.hpp"

#include <inttypes.h>
#include <unistd.h>

#include "driver/dedic_gpio.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char* kTag = "HX711";

// Not all clock sources are available on all chips.
#ifdef CONFIG_IDF_TARGET_ESP32C6
#define GPTIMER_CLK_SOURCE GPTIMER_CLK_SRC_DEFAULT
#else  // Fallback.
#define GPTIMER_CLK_SOURCE GPTIMER_CLK_SRC_APB
#endif

HX711::HX711(gpio_num_t clock_pin, gpio_num_t data_pin, Mode mode)
    : clock_pin_(clock_pin), data_pin_(data_pin) {
    if (mode == Mode::kChannelA128) {
        clock_cycles_ = 25;
    } else if (mode == Mode::kChannelB32) {
        clock_cycles_ = 26;
    } else if (mode == Mode::kChannelA64) {
        clock_cycles_ = 27;
    }

    gpio_set_level(clock_pin, 0);
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << clock_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Configure the data pin
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << data_pin);
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    const int clocks_gpios[] = {clock_pin};
    dedic_gpio_bundle_config_t clocks_config = {};
    clocks_config.gpio_array = clocks_gpios;
    clocks_config.array_size = 1;
    clocks_config.flags.out_en = 1;
    ESP_ERROR_CHECK(dedic_gpio_new_bundle(&clocks_config, &clock_));

    const int data_gpios[] = {data_pin};
    dedic_gpio_bundle_config_t data_config = {};
    data_config.gpio_array = data_gpios;
    data_config.array_size = 1;
    data_config.flags.in_en = 1;
    ESP_ERROR_CHECK(dedic_gpio_new_bundle(&data_config, &data_));

    gptimer_config_t timer_conf = {};
    timer_conf.clk_src = GPTIMER_CLK_SOURCE;
    timer_conf.direction = GPTIMER_COUNT_UP;
    timer_conf.resolution_hz = 10000000;  // 10MhZ
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_conf, &gptimer_));
    ESP_ERROR_CHECK(gptimer_enable(gptimer_));

    // Read the first value to configure the HX711
    int32_t v = Read(pdMS_TO_TICKS(1000));
    if (v == kUndefined) {
        ESP_LOGE("HX711", "Failed to read the first value");
    }
}

int32_t HX711::Read(TickType_t tick_to_wait) {
    dedic_gpio_bundle_write(clock_, 1, 0);

    // Wait for the data pin to be low
    int count = 0;
    while (dedic_gpio_bundle_read_in(data_) != 0) {
        if (count > tick_to_wait) {
            ESP_LOGE(kTag, "Data pin is not going low");
            return kUndefined;
        }
        count++;
        vTaskDelay(1);
    }

    int32_t value = 0;
    ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer_, 0));
    ESP_ERROR_CHECK(gptimer_start(gptimer_));

    for (int i = 0; i < clock_cycles_; i++) {
        uint64_t t0, t1;
        ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer_, &t0));
        do {  // wait 1 us
            ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer_, &t1));
        } while ((t1 - t0) <= 10);

        dedic_gpio_bundle_write(clock_, 1, 1);  // rising edge
        ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer_, &t0));
        do {  // wait 1 us
            ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer_, &t1));
        } while ((t1 - t0) <= 10);
        if (i < 24) {
            int bit = dedic_gpio_bundle_read_in(data_);
            value = (value << 1) | bit;
        }
        dedic_gpio_bundle_write(clock_, 1, 0);  // falling edge
    }
    ESP_ERROR_CHECK(gptimer_stop(gptimer_));

    // propagate the sign bit
    if (value >= (1 << 23)) {
        value = value - (1 << 24);
    }
    return value;
}
