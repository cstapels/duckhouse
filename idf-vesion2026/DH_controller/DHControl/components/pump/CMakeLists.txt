idf_component_register(SRCS "pump.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_gpio esp_timer freertos board sensors config)
