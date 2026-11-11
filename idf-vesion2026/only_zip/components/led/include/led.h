#pragma once
#include <stdbool.h>

void led_init(void);
/* Flash the communication LED n times (2 at startup, 3 on Wi-Fi connect). */
void led_flash(int n);
/* Flash continuously while a lift or door move is in progress. */
void led_set_moving(bool moving);

/* True when no pattern is running (the chip may sleep). */
bool led_idle(void);
