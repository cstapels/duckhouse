#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Starts the ThingSpeak poll task (software spec section 13). */
esp_err_t net_init(void);

/* A button press cancels the running poll cycle until the next period. */
void net_cancel_poll(void);

/* Manual/Auto mode set through field 5 of the duck door channel. Boots Manual. */
bool net_is_auto(void);

/* True once the clock has been set from a ThingSpeak response. */
bool net_time_valid(void);

/* For the power manager: a poll cycle is running / when the next one is due
 * (esp_timer time, microseconds). */
bool net_busy(void);
int64_t net_next_poll_us(void);
