#include "sensors.h"
#include "pins.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/touch_pad.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensors";

#define ADC_SAMPLES        64     /* [ASSUMED] software spec 8.3 */
#define POT_SAMPLES        16
#define TOUCH_SAMPLES      8
#define TRAVEL_MIN_MS      20000  /* 20 s to 50 s, set by the owner */
#define TRAVEL_MAX_MS      50000

static adc_oneshot_unit_handle_t s_adc;
static adc_channel_t s_ch_vbat, s_ch_photo, s_ch_pot_lift, s_ch_pot_door;

static esp_err_t config_pin(int gpio, adc_channel_t *out)
{
    adc_unit_t unit;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(gpio, &unit, out));
    adc_oneshot_chan_cfg_t cfg = {
        .atten = ADC_ATTEN_DB_12,      /* "11 dB" in the older Arduino naming */
        .bitwidth = ADC_BITWIDTH_12,
    };
    return adc_oneshot_config_channel(s_adc, *out, &cfg);
}

esp_err_t sensors_init(void)
{
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &s_adc));
    ESP_ERROR_CHECK(config_pin(PIN_ADC_VBAT, &s_ch_vbat));
    ESP_ERROR_CHECK(config_pin(PIN_ADC_PHOTO, &s_ch_photo));
    ESP_ERROR_CHECK(config_pin(PIN_ADC_POT_LIFT, &s_ch_pot_lift));
    ESP_ERROR_CHECK(config_pin(PIN_ADC_POT_DOOR, &s_ch_pot_door));

    /* Touch sensor in software-start mode, so nothing runs between reads and
     * light sleep is not disturbed. Legacy ESP32 touch API (ESP-IDF v5.5). */
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_NUM7, 0));
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW));
    ESP_LOGI(TAG, "ADC1 and touch pad 7 ready");
    return ESP_OK;
}

static int adc_avg(adc_channel_t ch, int samples)
{
    int64_t sum = 0;
    int n = 0;
    for (int i = 0; i < samples; i++) {
        int raw;
        if (adc_oneshot_read(s_adc, ch, &raw) == ESP_OK) {
            sum += raw;
            n++;
        }
    }
    return n ? (int)(sum / n) : -1;
}

int sensors_battery_raw(void) { return adc_avg(s_ch_vbat, ADC_SAMPLES); }
int sensors_photo_raw(void)   { return adc_avg(s_ch_photo, ADC_SAMPLES); }

uint32_t sensors_travel_ms(bool lift)
{
    int raw = adc_avg(lift ? s_ch_pot_lift : s_ch_pot_door, POT_SAMPLES);
    if (raw < 0) {
        raw = 0;
    }
    return TRAVEL_MIN_MS + (uint32_t)((uint64_t)raw * (TRAVEL_MAX_MS - TRAVEL_MIN_MS) / 4095);
}

int sensors_tank_raw(void)
{
    int64_t sum = 0;
    int n = 0;
    for (int i = 0; i < TOUCH_SAMPLES; i++) {
        uint16_t v;
        touch_pad_sw_start();
        vTaskDelay(pdMS_TO_TICKS(2));
        if (touch_pad_read(TOUCH_PAD_NUM7, &v) == ESP_OK) {
            sum += v;
            n++;
        }
    }
    return n ? (int)(sum / n) : -1;
}
