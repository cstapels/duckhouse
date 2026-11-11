idf_component_register(SRCS "actuator.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_timer board inputs sensors config)
