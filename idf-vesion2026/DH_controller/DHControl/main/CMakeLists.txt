idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES board led config sensors inputs actuator net pump power esp_system)
