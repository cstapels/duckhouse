#include "actuator.h"
#include "pins.h"
#include "board.h"
#include "inputs.h"
#include "sensors.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "actuator";

#define PWM_FREQ_HZ   20000     /* [ASSUMED] software spec 6.2 */
#define DUTY_MAX      1023      /* 10-bit */
#define RAMP_MS       500       /* [ASSUMED] soft start */
#define DEAD_MS       500       /* [ASSUMED] reversal dead time, 6.1 */
#define STAGGER_MS    500       /* [ASSUMED] second actuator starts later, 6.7 */

typedef struct {
    int en_pin;
    int pin_open, pin_close;
    ledc_channel_t ch_open, ch_close;
    const char *key_pos;
} hw_t;

static const hw_t HW[ACT_COUNT] = {
    [ACT_DOOR] = { PIN_DOOR_EN, PIN_DOOR_RPWM, PIN_DOOR_LPWM,
                   LEDC_CHANNEL_0, LEDC_CHANNEL_1, "door_pos" },
    [ACT_LIFT] = { PIN_LIFT_EN, PIN_LIFT_RPWM, PIN_LIFT_LPWM,
                   LEDC_CHANNEL_2, LEDC_CHANNEL_3, "lift_pos" },
};

typedef enum { S_IDLE, S_WAIT_POWER, S_DEAD_TIME, S_RUNNING } mstate_t;

typedef struct {
    mstate_t st;
    act_dir_t dir, pending;
    act_pos_t pos;
    bool fault;
    bool powered;
    int64_t t_state, t_move, t_last_tick;
    uint32_t travel_ms, run_limit_ms;
    bool timed;                 /* lift open: no limit switch, run by time */
    int32_t est_ms;             /* lift position estimate, ms from closed */
    bool est_valid;
} act_t;

static act_t A[ACT_COUNT];
static SemaphoreHandle_t s_lock;   /* the net task issues commands; the control task ticks */
static int64_t s_last_start_ms = -100000;

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }
static act_dir_t opposite(act_dir_t d) { return d == DIR_OPEN ? DIR_CLOSE : DIR_OPEN; }

static void drive(act_id_t id, act_dir_t dir, uint32_t duty)
{
    const hw_t *h = &HW[id];
    uint32_t d_open  = (dir == DIR_OPEN)  ? duty : 0;
    uint32_t d_close = (dir == DIR_CLOSE) ? duty : 0;
    /* RPWM and LPWM are never both active (software spec 6.1). */
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, h->ch_open, d_open);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, h->ch_open);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, h->ch_close, d_close);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, h->ch_close);
    gpio_set_level(h->en_pin, dir == DIR_NONE ? 0 : 1);
}

static void persist(act_id_t id)
{
    config_set_i32(HW[id].key_pos, (int32_t)A[id].pos);
    if (id == ACT_LIFT) {
        config_set_i32("lift_est", A[id].est_valid ? A[id].est_ms : -1);
    }
}

static void release_power(act_t *a)
{
    if (a->powered) {
        a->powered = false;
        board_limits_release();
        board_motor_logic_release();
    }
}

static bool limit_hit(act_id_t id, act_dir_t dir)
{
    if (id == ACT_DOOR) {
        return inputs_limit_active(dir == DIR_OPEN ? LIM_DOOR_OPEN : LIM_DOOR_CLOSE);
    }
    return dir == DIR_CLOSE && inputs_limit_active(LIM_LIFT_CLOSE);   /* no Open limit */
}

static void finish(act_id_t id, act_pos_t pos)
{
    act_t *a = &A[id];
    drive(id, DIR_NONE, 0);
    a->st = S_IDLE;
    a->dir = a->pending = DIR_NONE;
    a->pos = pos;
    if (id == ACT_LIFT) {
        if (pos == POS_CLOSED) { a->est_ms = 0; a->est_valid = true; }
        if (pos == POS_OPEN)   { a->est_ms = (int32_t)a->travel_ms; a->est_valid = true; }
    }
    release_power(a);
    persist(id);
    ESP_LOGI(TAG, "%s finished, pos=%d", id == ACT_DOOR ? "door" : "lift", (int)pos);
}

static void abort_move(act_id_t id, const char *why)
{
    act_t *a = &A[id];
    ESP_LOGW(TAG, "%s: %s", id == ACT_DOOR ? "door" : "lift", why);
    a->fault = true;
    finish(id, POS_UNKNOWN);
}

static void begin_move(act_id_t id)
{
    act_t *a = &A[id];
    a->travel_ms = sensors_travel_ms(id == ACT_LIFT);   /* read before the move, not during */

    if (limit_hit(id, a->dir)) {                        /* already at the end */
        finish(id, a->dir == DIR_OPEN ? POS_OPEN : POS_CLOSED);
        return;
    }
    if (id == ACT_DOOR && inputs_limit_active(LIM_DOOR_OPEN) && inputs_limit_active(LIM_DOOR_CLOSE)) {
        abort_move(id, "both door limits active");
        return;
    }

    a->timed = (id == ACT_LIFT && a->dir == DIR_OPEN);
    if (a->timed) {
        int32_t remain = a->est_valid ? (int32_t)a->travel_ms - a->est_ms : (int32_t)a->travel_ms;
        if (remain <= 0) {
            finish(id, POS_OPEN);
            return;
        }
        a->run_limit_ms = (uint32_t)remain;
    } else {
        a->run_limit_ms = a->travel_ms;                 /* max run (travel-timeout fault) */
    }

    a->pos = POS_UNKNOWN;                               /* an interrupted move leaves UNKNOWN */
    persist(id);
    a->t_move = a->t_last_tick = now_ms();
    s_last_start_ms = a->t_move;
    a->st = S_RUNNING;
    drive(id, a->dir, 0);
    ESP_LOGI(TAG, "%s start %s, limit %u ms", id == ACT_DOOR ? "door" : "lift",
             a->dir == DIR_OPEN ? "open" : "close", (unsigned)a->run_limit_ms);
}

static void tick_one(act_id_t id, int64_t now)
{
    act_t *a = &A[id];
    switch (a->st) {
    case S_IDLE:
        break;

    case S_WAIT_POWER:
        if (board_motor_logic_ready() && board_limits_ready() &&
            (now - s_last_start_ms) >= STAGGER_MS) {
            begin_move(id);
        }
        break;

    case S_DEAD_TIME:
        if (now - a->t_state >= DEAD_MS) {
            a->dir = a->pending;
            a->pending = DIR_NONE;
            a->st = S_WAIT_POWER;                       /* power is still held */
        }
        break;

    case S_RUNNING: {
        uint32_t el = (uint32_t)(now - a->t_move);
        int32_t dt = (int32_t)(now - a->t_last_tick);
        a->t_last_tick = now;

        drive(id, a->dir, el >= RAMP_MS ? DUTY_MAX : (el * DUTY_MAX) / RAMP_MS);

        if (id == ACT_LIFT && a->est_valid) {
            a->est_ms += (a->dir == DIR_OPEN) ? dt : -dt;
            if (a->est_ms < 0) a->est_ms = 0;
        }
        if (id == ACT_DOOR && inputs_limit_active(LIM_DOOR_OPEN) && inputs_limit_active(LIM_DOOR_CLOSE)) {
            abort_move(id, "both door limits active");
        } else if (limit_hit(id, a->dir)) {
            finish(id, a->dir == DIR_OPEN ? POS_OPEN : POS_CLOSED);
        } else if (el >= a->run_limit_ms) {
            if (a->timed) {
                finish(id, POS_OPEN);                   /* timed travel complete */
            } else {
                abort_move(id, "travel timeout");       /* limit never reached */
            }
        }
        break;
    }
    }
}

void actuator_tick(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int64_t now = now_ms();
    for (int i = 0; i < ACT_COUNT; i++) {
        tick_one((act_id_t)i, now);
    }
    xSemaphoreGive(s_lock);
}

static act_result_t reverse(act_id_t id)
{
    act_t *a = &A[id];
    if (a->st == S_RUNNING) {
        a->pending = opposite(a->dir);
        drive(id, DIR_NONE, 0);
        a->st = S_DEAD_TIME;
        a->t_state = now_ms();
        a->pos = POS_UNKNOWN;
        return ACT_REVERSED;
    }
    if (a->st == S_WAIT_POWER) {
        a->dir = opposite(a->dir);
        return ACT_REVERSED;
    }
    return ACT_BUSY;                                    /* dead time: presses are ignored */
}

static act_result_t command_nolock(act_id_t id, act_dir_t dir)
{
    act_t *a = &A[id];
    if (a->st != S_IDLE) {
        if (dir == a->dir || dir == a->pending) {
            return ACT_BUSY;
        }
        return reverse(id);
    }
    a->fault = false;                                   /* a new command clears a travel fault */
    if ((dir == DIR_OPEN && a->pos == POS_OPEN) || (dir == DIR_CLOSE && a->pos == POS_CLOSED)) {
        return ACT_AT_TARGET;
    }
    a->dir = dir;
    a->pending = DIR_NONE;
    a->powered = true;
    board_motor_logic_acquire();                        /* 5 V before EN or PWM */
    board_limits_acquire();
    a->st = S_WAIT_POWER;
    a->t_state = now_ms();
    return ACT_OK;
}

static act_result_t button_nolock(act_id_t id, act_dir_t dir)
{
    if (A[id].st != S_IDLE) {
        return reverse(id);                             /* either button reverses */
    }
    return command_nolock(id, dir);
}

act_result_t actuator_command(act_id_t id, act_dir_t dir)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    act_result_t r = command_nolock(id, dir);
    xSemaphoreGive(s_lock);
    return r;
}

act_result_t actuator_button(act_id_t id, act_dir_t dir)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    act_result_t r = button_nolock(id, dir);
    xSemaphoreGive(s_lock);
    return r;
}

bool actuator_is_active(act_id_t id) { return A[id].st != S_IDLE; }   /* single word read */

bool actuator_any_active(void)
{
    for (int i = 0; i < ACT_COUNT; i++) {
        if (A[i].st != S_IDLE) return true;
    }
    return false;
}

bool actuator_fault(act_id_t id) { return A[id].fault; }

int actuator_report_state(act_id_t id)
{
    const act_t *a = &A[id];
    if (id == ACT_DOOR) {
        if (a->fault && a->pos == POS_UNKNOWN) {
            return 0;
        }
        return a->pos == POS_OPEN ? 1 : (a->pos == POS_CLOSED ? -1 : 0);
    }
    return a->pos == POS_CLOSED ? -1 : 1;               /* lift has no Open limit */
}

esp_err_t actuator_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    ledc_timer_config_t t = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    for (int i = 0; i < ACT_COUNT; i++) {
        const int pins[2] = { HW[i].pin_open, HW[i].pin_close };
        const ledc_channel_t chs[2] = { HW[i].ch_open, HW[i].ch_close };
        for (int k = 0; k < 2; k++) {
            ledc_channel_config_t c = {
                .gpio_num = pins[k],
                .speed_mode = LEDC_HIGH_SPEED_MODE,
                .channel = chs[k],
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = LEDC_TIMER_0,
                .duty = 0,
                .hpoint = 0,
            };
            ESP_ERROR_CHECK(ledc_channel_config(&c));
        }
        drive((act_id_t)i, DIR_NONE, 0);

        /* Software spec 6.4: restore the saved state, make no recovery move. */
        A[i].st = S_IDLE;
        A[i].pos = (act_pos_t)config_get_i32(HW[i].key_pos, POS_UNKNOWN);
    }
    int32_t est = config_get_i32("lift_est", -1);
    A[ACT_LIFT].est_valid = est >= 0;
    A[ACT_LIFT].est_ms = est >= 0 ? est : 0;
    ESP_LOGI(TAG, "restored door=%d lift=%d", (int)A[ACT_DOOR].pos, (int)A[ACT_LIFT].pos);
    return ESP_OK;
}
