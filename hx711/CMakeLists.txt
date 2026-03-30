idf_component_register(SRCS "src/hx711.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_driver_gpio esp_driver_gptimer esp_timer)
