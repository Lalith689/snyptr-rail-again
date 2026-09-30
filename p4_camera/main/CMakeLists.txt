idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_video esp_cam_sensor esp_timer esp_driver_jpeg esp_driver_i2c esp_driver_uart esp_driver_gpio vfs)
