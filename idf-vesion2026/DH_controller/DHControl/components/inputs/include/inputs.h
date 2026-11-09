#pragma once
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

/* Order matters: opposite button of the same object = id ^ 1. */
typedef enum {
    BTN_LIFT_CLOSE = 0,
    BTN_LIFT_OPEN  = 1,
    BTN_DOOR_CLOSE = 2,
    BTN_DOOR_OPEN  = 3,
    BTN_COUNT
} button_id_t;

typedef enum {
    LIM_DOOR_OPEN = 0,
    LIM_DOOR_CLOSE,
    LIM_LIFT_CLOSE,
    LIM_COUNT
} limit_id_t;

esp_err_t inputs_init(void);

/* Pops one debounced button press. One event per press; a held button does not
 * repeat (software spec section 7). */
bool inputs_get_button_event(button_id_t *id, TickType_t wait);

/* Limit switch state. Only meaningful while board_limits_ready() is true. */
bool inputs_limit_active(limit_id_t id);

/* Wake the control task so it runs a tick (used after another task starts a move). */
void inputs_poke(void);
/* True while a debounce is in progress or events are queued. */
bool inputs_busy(void);
/* Call after a GPIO wake from light sleep. */
void inputs_scan_after_wake(void);
