#include "board.h"
#include "pins.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define LOGIC_SETTLE_MS          200   /* [ASSUMED] software spec 6.5 */
#define BOARD_LOGIC_OFF_DELAY_MS 5000  /* [ASSUMED] software spec 6.5 / 11 */
#define LIMIT_SETTLE_MS          5     /* [ASSUMED] software spec 8.1 */

static const char *TAG = "board";
static SemaphoreHandle_t s_mutex;
static esp_timer_handle_t s_off_timer;

static int s_logic_users;
static bool s_logic_on;
static int64_t s_logic_on_us;

static int s_limit_users;
static int64_t s_limit_on_us;

static const int OUTPUTS[] = {
    PIN_LIMIT_PWR, PIN_STATUS_LED, PIN_DOOR_EN, PIN_LIFT_LPWM, PIN_LIFT_EN,
    PIN_LIFT_RPWM, PIN_DOOR_LPWM, PIN_MOTOR_LOGIC_PWR, PIN_DOOR_RPWM,
    PIN_PUMP_CTRL,
};

static void logic_off_cb(void *arg)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_logic_users == 0 && s_logic_on) {
        gpio_set_level(PIN_MOTOR_LOGIC_PWR, 0);
        s_logic_on = false;
        ESP_LOGI(TAG, "BTS7960 logic power off");
    }
    xSemaphoreGive(s_mutex);
}

void board_init_safe(void)
{
    /* Set the output latch LOW before enabling the driver so there is no glitch.
     * The PCB pulldowns already hold these pins LOW through reset. */
    for (size_t i = 0; i < sizeof(OUTPUTS) / sizeof(OUTPUTS[0]); i++) {
        gpio_set_level(OUTPUTS[i], 0);
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << OUTPUTS[i],
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        gpio_set_level(OUTPUTS[i], 0);
    }

    s_mutex = xSemaphoreCreateMutex();
    const esp_timer_create_args_t args = { .callback = logic_off_cb, .name = "logic_off" };
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_off_timer));
    ESP_LOGI(TAG, "outputs in safe state");
}

void board_motor_logic_acquire(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_logic_users++;
    esp_timer_stop(s_off_timer);          /* ok if it is not running */
    if (!s_logic_on) {
        gpio_set_level(PIN_MOTOR_LOGIC_PWR, 1);
        s_logic_on = true;
        s_logic_on_us = esp_timer_get_time();
        ESP_LOGI(TAG, "BTS7960 logic power on");
    }
    xSemaphoreGive(s_mutex);
}

void board_motor_logic_release(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_logic_users > 0) {
        s_logic_users--;
    }
    if (s_logic_users == 0) {
        esp_timer_stop(s_off_timer);
        esp_timer_start_once(s_off_timer, (uint64_t)BOARD_LOGIC_OFF_DELAY_MS * 1000);
    }
    xSemaphoreGive(s_mutex);
}

bool board_motor_logic_ready(void)
{
    bool ready;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    ready = s_logic_on &&
            (esp_timer_get_time() - s_logic_on_us) >= (int64_t)LOGIC_SETTLE_MS * 1000;
    xSemaphoreGive(s_mutex);
    return ready;
}

void board_limits_acquire(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_limit_users++ == 0) {
        gpio_set_level(PIN_LIMIT_PWR, 1);
        s_limit_on_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_mutex);
}

void board_limits_release(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_limit_users > 0 && --s_limit_users == 0) {
        gpio_set_level(PIN_LIMIT_PWR, 0);
    }
    xSemaphoreGive(s_mutex);
}

bool board_limits_ready(void)
{
    bool ready;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    ready = s_limit_users > 0 &&
            (esp_timer_get_time() - s_limit_on_us) >= (int64_t)LIMIT_SETTLE_MS * 1000;
    xSemaphoreGive(s_mutex);
    return ready;
}

bool board_motor_logic_on(void)
{
    bool on;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    on = s_logic_on || s_logic_users > 0;
    xSemaphoreGive(s_mutex);
    return on;
}
