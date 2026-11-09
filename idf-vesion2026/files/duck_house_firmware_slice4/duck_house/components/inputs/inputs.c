#include "inputs.h"
#include "pins.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "inputs";

#define DEBOUNCE_MS 30   /* [ASSUMED] software spec section 7 */

static const int BTN_PIN[BTN_COUNT] = {
    [BTN_LIFT_CLOSE] = PIN_BTN_LIFT_CLOSE,
    [BTN_LIFT_OPEN]  = PIN_BTN_LIFT_OPEN,
    [BTN_DOOR_CLOSE] = PIN_BTN_DOOR_CLOSE,
    [BTN_DOOR_OPEN]  = PIN_BTN_DOOR_OPEN,
};

static const int LIM_PIN[LIM_COUNT] = {
    [LIM_DOOR_OPEN]  = PIN_LIM_DOOR_OPEN,
    [LIM_DOOR_CLOSE] = PIN_LIM_DOOR_CLOSE,
    [LIM_LIFT_CLOSE] = PIN_LIM_LIFT_CLOSE,
};

typedef enum { B_IDLE, B_DEBOUNCE, B_WAIT_RELEASE } bstate_t;

static struct {
    esp_timer_handle_t timer;
    volatile bstate_t state;
} s_btn[BTN_COUNT];

static QueueHandle_t s_isr_q;     /* ISR -> input task */
static QueueHandle_t s_event_q;   /* debounced presses -> control task */

/* The interrupt handler does the minimum: disable this pin's interrupt and
 * hand the id to the input task, which starts the debounce timer. */
static void IRAM_ATTR button_isr(void *arg)
{
    int id = (int)(intptr_t)arg;
    BaseType_t woken = pdFALSE;
    gpio_intr_disable(BTN_PIN[id]);
    xQueueSendFromISR(s_isr_q, &id, &woken);
    if (woken) {
        portYIELD_FROM_ISR();
    }
}

/* Runs in the esp_timer task, never in the ISR. */
static void debounce_cb(void *arg)
{
    int id = (int)(intptr_t)arg;
    bool high = gpio_get_level(BTN_PIN[id]) == 1;

    switch (s_btn[id].state) {
    case B_DEBOUNCE:
        if (high) {
            /* Both buttons of one object pressed together: ignore both. */
            if (gpio_get_level(BTN_PIN[id ^ 1]) == 1) {
                s_btn[id].state = B_WAIT_RELEASE;
            } else {
                xQueueSend(s_event_q, &id, 0);
                s_btn[id].state = B_WAIT_RELEASE;
            }
            esp_timer_start_once(s_btn[id].timer, DEBOUNCE_MS * 1000);
        } else {                          /* bounce or noise */
            s_btn[id].state = B_IDLE;
            gpio_intr_enable(BTN_PIN[id]);
        }
        break;
    case B_WAIT_RELEASE:
        if (!high) {                      /* released for one interval: re-arm */
            s_btn[id].state = B_IDLE;
            gpio_intr_enable(BTN_PIN[id]);
        } else {
            esp_timer_start_once(s_btn[id].timer, DEBOUNCE_MS * 1000);
        }
        break;
    default:
        break;
    }
}

static void input_task(void *arg)
{
    int id;
    while (1) {
        if (xQueueReceive(s_isr_q, &id, portMAX_DELAY) == pdTRUE) {
            s_btn[id].state = B_DEBOUNCE;
            esp_timer_start_once(s_btn[id].timer, DEBOUNCE_MS * 1000);
        }
    }
}

esp_err_t inputs_init(void)
{
    s_isr_q = xQueueCreate(8, sizeof(int));
    s_event_q = xQueueCreate(8, sizeof(int));

    for (int i = 0; i < LIM_COUNT; i++) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << LIM_PIN[i],
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
    }

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (int i = 0; i < BTN_COUNT; i++) {
        const esp_timer_create_args_t targs = {
            .callback = debounce_cb,
            .arg = (void *)(intptr_t)i,
            .name = "btn_db",
        };
        ESP_ERROR_CHECK(esp_timer_create(&targs, &s_btn[i].timer));
        s_btn[i].state = B_IDLE;

        /* External 10 k pulldown on each button; no internal pulls. */
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << BTN_PIN[i],
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_HIGH_LEVEL,   /* also the light-sleep wake type */
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_PIN[i], button_isr, (void *)(intptr_t)i));
        /* HIGH on any button wakes the chip from light sleep (software spec 7).
         * The ISR disables the pin's interrupt on the first hit, and it is only
         * re-armed after release, so a held button gives one event. */
        ESP_ERROR_CHECK(gpio_wakeup_enable(BTN_PIN[i], GPIO_INTR_HIGH_LEVEL));
    }
    xTaskCreate(input_task, "inputs", 2048, NULL, 7, NULL);
    ESP_LOGI(TAG, "buttons armed, debounce %d ms", DEBOUNCE_MS);
    return ESP_OK;
}

bool inputs_get_button_event(button_id_t *id, TickType_t wait)
{
    int v;
    if (xQueueReceive(s_event_q, &v, wait) == pdTRUE && v >= 0) {
        *id = (button_id_t)v;
        return true;
    }
    return false;                      /* timeout, or a poke from another task */
}

void inputs_poke(void)
{
    int v = -1;
    xQueueSend(s_event_q, &v, 0);
}

bool inputs_busy(void)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        if (s_btn[i].state != B_IDLE) {
            return true;
        }
    }
    return uxQueueMessagesWaiting(s_isr_q) > 0 || uxQueueMessagesWaiting(s_event_q) > 0;
}

/* After a GPIO wake the level interrupt normally fires by itself. This catches
 * a press whose interrupt did not run. */
void inputs_scan_after_wake(void)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        if (s_btn[i].state == B_IDLE && gpio_get_level(BTN_PIN[i]) == 1) {
            gpio_intr_disable(BTN_PIN[i]);
            xQueueSend(s_isr_q, &i, 0);
        }
    }
}

bool inputs_limit_active(limit_id_t id)
{
    return gpio_get_level(LIM_PIN[id]) == 1;
}
