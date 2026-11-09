idf_component_register(SRCS "net.c" "wifi_mgr.c" "ts.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_netif esp_event esp_http_client mbedtls cjson
                                esp_timer nvs_flash config actuator sensors led pump)
