#pragma once
#include <stdbool.h>

/* Put every output into its safe (LOW) state. Call first thing in app_main. */
void board_init_safe(void);

/* BTS7960 5 V logic supply (GPIO17). Reference counted: the supply is on
 * before any move (acquire) and goes off BOARD_LOGIC_OFF_DELAY_MS after the
 * last user releases it (software spec 6.5 step 9). */
void board_motor_logic_acquire(void);
void board_motor_logic_release(void);
bool board_motor_logic_ready(void);   /* on and settled */

/* Limit-switch supply (GPIO2). HIGH only while someone needs the limits. */
void board_limits_acquire(void);
void board_limits_release(void);
bool board_limits_ready(void);        /* on and settled */

bool board_motor_logic_on(void);      /* GPIO17 high, or a user still holds it */
