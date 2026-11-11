#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef enum { WIFI_RES_OK, WIFI_RES_FAILED, WIFI_RES_CANCELLED } wifi_res_t;

esp_err_t wifi_mgr_init(void);

/* Up to 3 attempts, 15 s each, 2 s then 5 s between them (software spec 13).
 * Returns early with CANCELLED if *cancel becomes true. */
wifi_res_t wifi_mgr_connect(volatile bool *cancel);

/* Wi-Fi is turned off between poll cycles to save power. */
void wifi_mgr_disconnect(void);

int wifi_mgr_rssi(void);   /* dBm, 0 if unknown */
