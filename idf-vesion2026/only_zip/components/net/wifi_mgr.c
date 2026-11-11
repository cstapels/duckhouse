#include "wifi_mgr.h"
#include "config.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "wifi";

#define ATTEMPTS        3
#define ATTEMPT_MS      15000
#define STARTED_WAIT_MS 3000
static const int GAP_MS[ATTEMPTS - 1] = { 2000, 5000 };

#define BIT_STARTED BIT0
#define BIT_UP      BIT1
#define BIT_DOWN    BIT2

static EventGroupHandle_t s_ev;

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        xEventGroupSetBits(s_ev, BIT_STARTED);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_ev, BIT_UP);
        xEventGroupSetBits(s_ev, BIT_DOWN);            /* no auto-reconnect here */
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupClearBits(s_ev, BIT_DOWN);
        xEventGroupSetBits(s_ev, BIT_UP);
    }
}

esp_err_t wifi_mgr_init(void)
{
    s_ev = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    return ESP_OK;
}

/* Waits in 100 ms slices so a button press can cancel quickly. */
static wifi_res_t wait_slices(int ms, EventBits_t want, volatile bool *cancel)
{
    for (int waited = 0; waited < ms; waited += 100) {
        if (cancel && *cancel) {
            return WIFI_RES_CANCELLED;
        }
        if (want == 0) {                               /* plain delay */
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        EventBits_t b = xEventGroupWaitBits(s_ev, want | BIT_DOWN, pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(100));
        if (b & want) {
            return WIFI_RES_OK;
        }
        if ((want & BIT_UP) && (b & BIT_DOWN)) {
            return WIFI_RES_FAILED;                    /* refused: fail this attempt early */
        }
    }
    return WIFI_RES_FAILED;
}

wifi_res_t wifi_mgr_connect(volatile bool *cancel)
{
    char ssid[33] = "", pass[65] = "";
    config_get_str("wifi_ssid", ssid, sizeof ssid, NULL);
    config_get_str("wifi_pass", pass, sizeof pass, "");
    if (ssid[0] == '\0') {
        ESP_LOGE(TAG, "no wifi_ssid in NVS (see provision/README.md)");
        return WIFI_RES_FAILED;
    }

    wifi_config_t wc = { 0 };
    strlcpy((char *)wc.sta.ssid, ssid, sizeof wc.sta.ssid);
    strlcpy((char *)wc.sta.password, pass, sizeof wc.sta.password);
    wc.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    wc.sta.pmf_cfg.capable = true;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));

    xEventGroupClearBits(s_ev, BIT_STARTED | BIT_UP | BIT_DOWN);
    if (esp_wifi_start() != ESP_OK) {
        return WIFI_RES_FAILED;
    }
    wifi_res_t r = wait_slices(STARTED_WAIT_MS, BIT_STARTED, cancel);
    if (r != WIFI_RES_OK) {
        return r;
    }

    for (int i = 0; i < ATTEMPTS; i++) {
        xEventGroupClearBits(s_ev, BIT_UP | BIT_DOWN);
        ESP_LOGI(TAG, "connect attempt %d/%d", i + 1, ATTEMPTS);
        esp_wifi_connect();
        r = wait_slices(ATTEMPT_MS, BIT_UP, cancel);
        if (r == WIFI_RES_OK || r == WIFI_RES_CANCELLED) {
            return r;
        }
        esp_wifi_disconnect();
        if (i < ATTEMPTS - 1) {
            r = wait_slices(GAP_MS[i], 0, cancel);     /* just a delay, cancellable */
            if (r == WIFI_RES_CANCELLED) {
                return r;
            }
        }
    }
    return WIFI_RES_FAILED;
}

void wifi_mgr_disconnect(void)
{
    esp_wifi_disconnect();
    esp_wifi_stop();
    xEventGroupClearBits(s_ev, BIT_STARTED | BIT_UP | BIT_DOWN);
}

int wifi_mgr_rssi(void)
{
    wifi_ap_record_t ap;
    return esp_wifi_sta_get_ap_info(&ap) == ESP_OK ? ap.rssi : 0;
}
