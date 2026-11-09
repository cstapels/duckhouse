#include "config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "config";
#define NS "duck"

esp_err_t config_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

int32_t config_get_i32(const char *key, int32_t def)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        return def;
    }
    int32_t v = def;
    if (nvs_get_i32(h, key, &v) != ESP_OK) {
        v = def;
    }
    nvs_close(h);
    return v;
}

esp_err_t config_set_i32(const char *key, int32_t value)
{
    if (config_get_i32(key, INT32_MIN) == value) {
        return ESP_OK;                    /* no flash write when unchanged */
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_i32(h, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t config_get_str(const char *key, char *out, size_t out_size, const char *def)
{
    nvs_handle_t h;
    size_t len = out_size;
    if (nvs_open(NS, NVS_READONLY, &h) == ESP_OK) {
        esp_err_t err = nvs_get_str(h, key, out, &len);
        nvs_close(h);
        if (err == ESP_OK) {
            return ESP_OK;
        }
    }
    if (def) {
        strlcpy(out, def, out_size);
        return ESP_OK;
    }
    out[0] = '\0';
    return ESP_ERR_NOT_FOUND;
}
