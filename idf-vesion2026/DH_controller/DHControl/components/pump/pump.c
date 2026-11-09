#include "pump.h"
#include "pins.h"
#include "sensors.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static const char *TAG = "pump";

#define MIN_EPOCH        1767225600LL   /* clock counts as set after 2026-01-01 */
#define MAX_RUN_S        1800           /* [ASSUMED] 30 min hard limit, spec 9.3 */
#define MIN_INTERVAL_S   60             /* [ASSUMED] 1 min hard limit */
#define BLOCKED_RETRY_S  30             /* [ASSUMED] re-check a start blocked by battery or tank */
#define TANK_CHECK_S     5              /* [ASSUMED] tank check during a run, spec 8.6 */
#define BATT_CLEAR_MARGIN 20            /* counts, spec 11 */
#define POLL_INTERVAL_S  3600           /* pump channel read hourly, spec 13 */

#define MODE_SLEEP    1
#define MODE_PUMP     2
#define MODE_OVERNIGHT 4
#define MODE_LEVEL    8
#define MODE_BATTERY  16

typedef struct {
    int run_s, interval_s, mode, window, bat_set, tank_set;
    bool have;                 /* settings known (read or stored) */
} cfg_t;

static const char *KEYS[6] = { "pump_run", "pump_int", "pump_mode", "pump_win", "pump_bat", "pump_tank" };

static SemaphoreHandle_t s_lock;
static cfg_t C;

static bool s_running;
static int64_t s_run_start_s, s_last_start_s, s_next_tank_s;
static int s_runs, s_total_s;
static time_t s_last_run_wall;
static bool s_batt_low;
static bool s_read_since_boot;
static time_t s_last_read_wall;

static int64_t now_s(void) { return esp_timer_get_time() / 1000000; }
static bool clock_valid(void) { return time(NULL) >= MIN_EPOCH; }

/* Field 4: wake hour = v mod 12, sleep hour = floor(v / 12) + 13 (spec 9.3). */
static bool decode_window(int v, int *wake, int *sleep)
{
    if (v < 0 || v > 143) {
        return false;
    }
    *wake = v % 12;
    *sleep = v / 12 + 13;
    return true;
}

static bool in_window_now(const cfg_t *c)
{
    int wake, sleep;
    if (!decode_window(c->window, &wake, &sleep)) {
        return false;
    }
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);                    /* Boston local time, TZ set in ts_init() */
    return tm.tm_hour >= wake && tm.tm_hour < sleep;
}

static bool tank_full(const cfg_t *c)
{
    int raw = sensors_tank_raw();            /* count falls as the tank fills */
    return raw >= 0 && raw <= c->tank_set;
}

static void pump_off(const char *why)
{
    gpio_set_level(PIN_PUMP_CTRL, 0);
    if (s_running) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_running = false;
        s_total_s += (int)(now_s() - s_run_start_s);
        s_last_run_wall = clock_valid() ? time(NULL) : 0;
        xSemaphoreGive(s_lock);
    }
    ESP_LOGI(TAG, "pump off: %s", why);
}

/* Event driven, so the chip can light-sleep between evaluations. The task is
 * woken by a one-shot esp_timer (esp_timer time keeps running through light
 * sleep, FreeRTOS ticks do not) or by pump_apply(). */
static esp_timer_handle_t s_timer;
static TaskHandle_t s_task;
static volatile int64_t s_next_wake_us = INT64_MAX;

static void arm(int64_t delay_s)
{
    esp_timer_stop(s_timer);                         /* ok if not running */
    if (delay_s < 0) {
        s_next_wake_us = INT64_MAX;                  /* nothing scheduled */
        return;
    }
    if (delay_s < 1) {
        delay_s = 1;
    }
    s_next_wake_us = esp_timer_get_time() + delay_s * 1000000;
    esp_timer_start_once(s_timer, (uint64_t)delay_s * 1000000);
}

static void timer_cb(void *arg) { xTaskNotifyGive(s_task); }

/* Seconds until the next top of the local hour: when outside the window there
 * is nothing to do before then. */
static int secs_to_next_hour(void)
{
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    return 3600 - (tm.tm_min * 60 + tm.tm_sec);
}

static void evaluate(void)
{
    int64_t now = now_s();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    cfg_t c = C;
    xSemaphoreGive(s_lock);

    if (s_running) {
        const char *why = NULL;
        int run = c.run_s > MAX_RUN_S ? MAX_RUN_S : c.run_s;
        if (!(c.mode & MODE_PUMP)) {
            why = "pump disabled";
        } else if (now - s_run_start_s >= run) {
            why = "run complete";
        } else if ((c.mode & MODE_OVERNIGHT) && clock_valid() && !in_window_now(&c)) {
            why = "window end";                      /* a running pump stops at the sleep hour */
        } else if ((c.mode & MODE_LEVEL) && now >= s_next_tank_s) {
            s_next_tank_s = now + TANK_CHECK_S;
            if (tank_full(&c)) {
                why = "tank full";
            }
        }
        if (!why) {
            arm(1);                                  /* awake while running anyway */
            return;                                  /* low battery never stops a run */
        }
        pump_off(why);
        now = now_s();
    }

    if (!c.have || !(c.mode & MODE_PUMP) || c.run_s <= 0) {
        arm(-1);                                     /* pump_apply() wakes us again */
        return;
    }
    int interval = c.interval_s < MIN_INTERVAL_S ? MIN_INTERVAL_S : c.interval_s;
    int64_t since = now - s_last_start_s;            /* first run is one interval after boot [ASSUMED] */
    if (since < interval) {
        arm(interval - since);
        return;
    }

    const char *blocked = NULL;
    int retry = BLOCKED_RETRY_S;
    if (c.mode & MODE_OVERNIGHT) {
        if (!clock_valid()) {
            blocked = "clock not set";
            retry = 60;
        } else if (!in_window_now(&c)) {
            blocked = "outside window";
            retry = secs_to_next_hour();             /* [ASSUMED] re-check at the next hour */
        }
    }
    if (!blocked && (c.mode & MODE_BATTERY) && sensors_battery_raw() < c.bat_set) {
        blocked = "battery below set point";
    }
    if (!blocked && (c.mode & MODE_LEVEL) && tank_full(&c)) {
        blocked = "tank full";
    }
    if (blocked) {
        ESP_LOGD(TAG, "start blocked: %s", blocked);
        arm(retry);
        return;
    }

    gpio_set_level(PIN_PUMP_CTRL, 1);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_running = true;
    s_run_start_s = s_last_start_s = now;
    s_next_tank_s = now + TANK_CHECK_S;
    s_runs++;
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "pump on for up to %d s", c.run_s > MAX_RUN_S ? MAX_RUN_S : c.run_s);
    arm(1);
}

static void pump_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        evaluate();
    }
}

int64_t pump_next_wake_us(void) { return s_next_wake_us; }

esp_err_t pump_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    /* Last-good settings. With nothing stored: sleep enabled, pump disabled. */
    int32_t mode = config_get_i32(KEYS[2], -1);
    C.have = mode >= 0;
    C.mode = mode >= 0 ? (int)mode : MODE_SLEEP;
    C.run_s = (int)config_get_i32(KEYS[0], 0);
    C.interval_s = (int)config_get_i32(KEYS[1], 0);
    C.window = (int)config_get_i32(KEYS[3], -1);
    C.bat_set = (int)config_get_i32(KEYS[4], 0);
    C.tank_set = (int)config_get_i32(KEYS[5], 0);
    s_last_start_s = now_s();                 /* first run one interval after boot */
    const esp_timer_create_args_t targs = { .callback = timer_cb, .name = "pump" };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_timer));
    xTaskCreatePinnedToCore(pump_task, "pump", 4096, NULL, 3, &s_task, 1);
    xTaskNotifyGive(s_task);                  /* first evaluation arms the timer */
    ESP_LOGI(TAG, "settings %s, mode %d", C.have ? "restored" : "not yet read", C.mode);
    return ESP_OK;
}

void pump_apply(const bool have[6], const int v[6])
{
    /* Range checks; a bad value keeps the previous one (spec 9.3 Validation). */
    static const int LO[6] = { 0, 0, 0, 0, 0, 0 };
    static const int HI[6] = { 86400, 604800, 255, 143, 4095, 65535 };
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int *dst[6] = { &C.run_s, &C.interval_s, &C.mode, &C.window, &C.bat_set, &C.tank_set };
    for (int i = 0; i < 6; i++) {
        if (!have[i]) {
            continue;
        }
        if (v[i] < LO[i] || v[i] > HI[i]) {
            ESP_LOGW(TAG, "field %d value %d out of range, ignored", i + 1, v[i]);
            continue;
        }
        *dst[i] = (i == 2) ? (v[i] & 31) : v[i];  /* only the lowest 5 mode bits are used */
        config_set_i32(KEYS[i], *dst[i]);
    }
    C.have = true;
    if (s_task) {
        xTaskNotifyGive(s_task);              /* re-evaluate with the new settings */
    }
    ESP_LOGI(TAG, "settings: run %d s, every %d s, mode %d, window %d, bat %d, tank %d",
             C.run_s, C.interval_s, C.mode, C.window, C.bat_set, C.tank_set);
    xSemaphoreGive(s_lock);
}

bool pump_sleep_enabled(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool e = (C.mode & MODE_SLEEP) != 0;
    xSemaphoreGive(s_lock);
    return e;
}

bool pump_running(void) { return s_running; }

bool pump_battery_low(void)
{
    if (s_running) {
        return s_batt_low;                    /* readings sag under load: keep the last at-rest state */
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int set = C.bat_set;
    xSemaphoreGive(s_lock);
    if (set <= 0) {
        return false;
    }
    int raw = sensors_battery_raw();
    if (raw >= 0) {
        if (raw < set) {
            s_batt_low = true;
        } else if (raw >= set + BATT_CLEAR_MARGIN) {
            s_batt_low = false;
        }
    }
    return s_batt_low;
}

void pump_summary(char *out, size_t cap)
{
    out[0] = '\0';
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int runs = s_runs, total = s_total_s;
    time_t last = s_last_run_wall;
    bool running = s_running;
    xSemaphoreGive(s_lock);
    if (runs == 0) {
        return;
    }
    int n = snprintf(out, cap, "pump %d run%s %d s total", runs, runs == 1 ? "" : "s", total);
    if (last >= MIN_EPOCH && n > 0 && (size_t)n < cap) {
        struct tm tm;
        localtime_r(&last, &tm);
        n += snprintf(out + n, cap - n, ", last %02d:%02d", tm.tm_hour, tm.tm_min);
    }
    if (running && n > 0 && (size_t)n < cap) {
        snprintf(out + n, cap - n, " (running)");
    }
}

void pump_report_done(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_runs = s_running ? 1 : 0;               /* a run in progress counts toward the next report */
    s_total_s = 0;
    s_last_run_wall = 0;
    xSemaphoreGive(s_lock);
}

bool pump_poll_due(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool due;
    time_t now = time(NULL);
    if (!C.have) {
        due = true;                           /* fresh device: read at once */
    } else if (now < MIN_EPOCH) {
        due = false;                          /* wait until the clock is set */
    } else if (!s_read_since_boot) {
        due = true;                           /* once after the clock is first set */
    } else {
        int wake = 0, sleep = 24;
        decode_window(C.window, &wake, &sleep);   /* invalid window: poll all day */
        struct tm tm;
        localtime_r(&now, &tm);
        due = tm.tm_hour >= wake && tm.tm_hour < sleep &&
              (now - s_last_read_wall) >= POLL_INTERVAL_S;
    }
    xSemaphoreGive(s_lock);
    return due;
}

void pump_mark_read(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_read_since_boot = true;
    s_last_read_wall = time(NULL);
    xSemaphoreGive(s_lock);
}
