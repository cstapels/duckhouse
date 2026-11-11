/* Duck House Controller firmware, first slice.
 * Implements: safe start-up, buttons with interrupt + timer debounce, lift and
 * door actuator control, status LED, Wi-Fi + ThingSpeak poll, pump, light sleep.
 * Auto mode and temperature are the next slices (see README.md). Spec: Duck House Controller Software
 * Specification. */
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"
#include "config.h"
#include "led.h"
#include "sensors.h"
#include "inputs.h"
#include "actuator.h"
#include "net.h"
#include "pump.h"
#include "power.h"

static const char *TAG = "main";

static void handle_button(button_id_t b)
{
    act_id_t id = (b == BTN_LIFT_CLOSE || b == BTN_LIFT_OPEN) ? ACT_LIFT : ACT_DOOR;
    act_dir_t dir = (b == BTN_LIFT_OPEN || b == BTN_DOOR_OPEN) ? DIR_OPEN : DIR_CLOSE;
    act_result_t r = actuator_button(id, dir);
    ESP_LOGI(TAG, "button %d -> %s %s: result %d", (int)b,
             id == ACT_LIFT ? "lift" : "door",
             dir == DIR_OPEN ? "open" : "close", (int)r);
}

/* Control task. While any actuator is active it runs a 10 ms tick (software
 * spec section 2). When everything is idle it blocks on the input queue, so the
 * chip can light-sleep; the task watchdog is only subscribed while active. */
static void control_task(void *arg)
{
    bool wdt = false;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        button_id_t b;
        if (actuator_any_active()) {
            if (!wdt) {
                ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
                wdt = true;
                last = xTaskGetTickCount();
            }
            while (inputs_get_button_event(&b, 0)) {
                net_cancel_poll();         /* a press cancels the poll cycle until the next period */
                handle_button(b);
            }
            actuator_tick();
            led_set_moving(true);
            esp_task_wdt_reset();
            vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
        } else {
            led_set_moving(false);
            if (wdt) {
                esp_task_wdt_delete(NULL);
                wdt = false;
            }
            /* Block until a button event or a poke from the net task. */
            if (inputs_get_button_event(&b, portMAX_DELAY)) {
                net_cancel_poll();
                handle_button(b);
            }
            actuator_tick();               /* start waiting for logic power right away */
        }
    }
}

void app_main(void)
{
    /* Software spec section 4: outputs LOW before anything else. */
    board_init_safe();

    ESP_LOGI(TAG, "reset reason %d", (int)esp_reset_reason());
    ESP_ERROR_CHECK(config_init());
    led_init();
    ESP_ERROR_CHECK(sensors_init());
    ESP_ERROR_CHECK(inputs_init());
    ESP_ERROR_CHECK(actuator_init());      /* restores saved state, no recovery move */
    ESP_ERROR_CHECK(pump_init());          /* last-good settings; pump stays off until enabled */
    ESP_ERROR_CHECK(net_init());           /* Wi-Fi + ThingSpeak poll task (core 0) */

    led_flash(2);                          /* 2 flashes at startup */

    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 6, NULL, 1);
    ESP_ERROR_CHECK(power_init());         /* light sleep manager, last */
}
