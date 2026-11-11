idf_component_register(SRCS "pump.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_timer freertos board sensors config)
