/**
 ******************************************************************************
 * @file        : main.cpp
 * @brief       : hx711 example
 * @author      : Jacques Supcik <jacques@supcik.net>
 * @date        : 20 November 2024
 ******************************************************************************
 * @copyright   : Copyright (c) 2024 Jacques Supcik
 * @attention   : SPDX-License-Identifier: MIT
 ******************************************************************************
 * @details
 *
 ******************************************************************************
 */

#include <inttypes.h>
#include <unistd.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "hx711.hpp"

extern "C" {
void app_main(void);
}

static const char* kTag = "app";

static const gpio_num_t kClockPin = GPIO_NUM_18;
static const gpio_num_t kDataPin = GPIO_NUM_16;

void app_main(void) {
    ESP_LOGI(kTag, "Starting App");

    HX711 hx711(kClockPin, kDataPin, HX711::Mode::kChannelA128);

    while (true) {
        int32_t value = hx711.Read(pdMS_TO_TICKS(1000));
        if (value != HX711::kUndefined) {
            ESP_LOGI(kTag, "Value: %" PRId32, value);
        } else {
            ESP_LOGE(kTag, "Failed to read the value");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
