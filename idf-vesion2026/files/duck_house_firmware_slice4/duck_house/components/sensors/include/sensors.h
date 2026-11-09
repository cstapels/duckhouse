#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t sensors_init(void);

/* Raw 12-bit ADC1 counts (0-4095), averaged. No calibration is applied:
 * battery and photosensor are reported as counts (software spec 8.2, 8.3). */
int sensors_battery_raw(void);
int sensors_photo_raw(void);

/* Travel-time pots mapped to milliseconds. range 20 s to 50 s
 * (software spec 6.6). Read before each move, never during one. */
uint32_t sensors_travel_ms(bool lift);

/* Capacitive tank level wire on GPIO27 (touch pad 7). The raw count falls as
 * the tank fills (software spec 8.6). Returns -1 if the read failed. */
int sensors_tank_raw(void);
