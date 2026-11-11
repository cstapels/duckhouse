#include "power.h"
#include "actuator.h"
#include "board.h"
#include "inputs.h"
#include "led.h"
#include "pump.h"
#include "net.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "power";

#define IDLE_BEFORE_SLEEP_MS 5000     /* [ASSUMED] software spec 7 */
#define MAX_SLEEP_US         (60LL * 1000000)   /* safety cap on one sleep */
#define POLL_MS              250

/* Anything that means "stay awake" (software spec 7): a move, the BTS7960 logic
 * supply still on, a debounce in progress, a pump run, a poll cycle, or an LED
 * pattern. */
static bool busy(void)
{
    return actuator_any_active() || board_motor_logic_on() || inputs_busy() ||
           pump_running() || net_busy() || !led_idle();
}

static void power_task(void *arg)
{
    int64_t last_busy_us = esp_timer_get_time();

    for (;;) {
        int64_t now = esp_timer_get_time();
        if (busy()) {
            last_busy_us = now;
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            continue;
        }
        /* Mode bit 1 of the pump channel's operation mode: 0 = never sleep. */
        if (!pump_sleep_enabled()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if ((now - last_busy_us) < (int64_t)IDLE_BEFORE_SLEEP_MS * 1000) {
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            continue;
        }

        /* Sleep until the next scheduled event: the poll, the pump, or the cap.
         * Buttons wake it through the GPIO wake source. */
        int64_t wake = now + MAX_SLEEP_US;
        int64_t t = net_next_poll_us();
        if (t < wake) wake = t;
        t = pump_next_wake_us();
        if (t < wake) wake = t;
        int64_t us = wake - now;
        if (us < 2000) {
            vTaskDelay(1);
            continue;
        }

        esp_sleep_enable_timer_wakeup((uint64_t)us);
        esp_rom_output_tx_wait_idle(0);
        esp_light_sleep_start();

        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        if (cause == ESP_SLEEP_WAKEUP_GPIO) {
            inputs_scan_after_wake();
        }
        ESP_LOGD(TAG, "woke, cause %d", (int)cause);
        /* A timer wake with nothing to do goes straight back to sleep: the 5 s
         * idle time is measured from the last real activity, not from the wake. */
    }
}

esp_err_t power_init(void)
{
    esp_err_t err = esp_sleep_enable_gpio_wakeup();   /* per-pin wake is set in inputs_init() */
    if (err != ESP_OK) {
        return err;
    }
    xTaskCreatePinnedToCore(power_task, "power", 3072, NULL, 2, NULL, 0);
    ESP_LOGI(TAG, "light sleep manager started");
    return ESP_OK;
}
