#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef enum { ACT_DOOR = 0, ACT_LIFT, ACT_COUNT } act_id_t;
typedef enum { DIR_NONE = 0, DIR_OPEN, DIR_CLOSE } act_dir_t;
typedef enum { POS_UNKNOWN = 0, POS_OPEN, POS_CLOSED } act_pos_t;

typedef enum {
    ACT_OK,          /* move started */
    ACT_REVERSED,    /* an active move was reversed */
    ACT_AT_TARGET,   /* already open/closed: rejected */
    ACT_BUSY,        /* same direction already running, or inside dead time */
    ACT_FAULT,       /* reserved */
} act_result_t;

esp_err_t actuator_init(void);

/* Remote or automatic command (ThingSpeak field 1, Auto mode). */
act_result_t actuator_command(act_id_t id, act_dir_t dir);

/* Button press. If the object is moving, ANY of its two buttons reverses the
 * motion (software spec section 7); otherwise it starts a move in `dir`. */
act_result_t actuator_button(act_id_t id, act_dir_t dir);

/* Call every 10 ms from the control task. */
void actuator_tick(void);

bool actuator_is_active(act_id_t id);
bool actuator_any_active(void);
bool actuator_fault(act_id_t id);

/* Value for the ThingSpeak limit-state fields: -1 closed, 1 open, 0 between,
 * 2 = both door limits active (fault). Lift: -1 closed, otherwise 1. */
int actuator_report_state(act_id_t id);
