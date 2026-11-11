#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/* Small NVS wrapper. Writes only happen on change (software spec section 12). */
esp_err_t config_init(void);
int32_t   config_get_i32(const char *key, int32_t def);
esp_err_t config_set_i32(const char *key, int32_t value);

/* String value (Wi-Fi credentials, ThingSpeak keys). Falls back to `def`.
 * Secrets are provisioned into NVS, never kept in source (software spec 13). */
esp_err_t config_get_str(const char *key, char *out, size_t out_size, const char *def);
