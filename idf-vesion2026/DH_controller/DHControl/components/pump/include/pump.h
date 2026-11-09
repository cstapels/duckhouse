#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/* Pump settings come from the pump ThingSpeak channel (software spec 9.3, 13):
 *   field 1 run length (s)          field 4 daily window (encoded)
 *   field 2 time between starts (s) field 5 battery set point (ADC count)
 *   field 3 operation mode (bits)   field 6 tank level set point (touch count)
 * Mode bits (bit 1 = value 1):
 *   1 sleep enable   2 pump enable   4 overnight off (window applies)
 *   8 level check    16 battery check                                       */
esp_err_t pump_init(void);

/* Apply values read from the channel. have[i]/v[i] are fields 1..6 (index 0..5).
 * Invalid values are ignored and the previous value kept. Saved to NVS. */
void pump_apply(const bool have[6], const int v[6]);

bool pump_sleep_enabled(void);        /* mode bit 1 (the sleep slice will use this) */
bool pump_running(void);
bool pump_battery_low(void);          /* below the field 5 set point, with hysteresis */

/* Pump activity since the last successful write, for the status message.
 * Empty string if the pump has not run. */
void pump_summary(char *out, size_t cap);
void pump_report_done(void);          /* call after a successful write */

/* Poll scheduling for the pump channel (hourly, between wake and sleep hour). */
bool pump_poll_due(void);
void pump_mark_read(void);

/* esp_timer time (us) of the next scheduled pump evaluation, INT64_MAX if none.
 * The power manager wakes for it. */
int64_t pump_next_wake_us(void);
