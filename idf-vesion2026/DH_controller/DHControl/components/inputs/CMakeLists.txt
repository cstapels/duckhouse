idf_component_register(SRCS "inputs.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_gpio esp_timer freertos board)
