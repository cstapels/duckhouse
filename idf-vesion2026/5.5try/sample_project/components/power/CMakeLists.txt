idf_component_register(SRCS "power.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_hw_support esp_rom esp_timer freertos
                                actuator board inputs led pump net)
