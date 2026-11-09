#include "net.h"
#include "wifi_mgr.h"
#include "ts.h"
#include "actuator.h"
#include "sensors.h"
#include "led.h"
#include "pump.h"
#include "inputs.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "net";

#define POLL_MS          (4 * 60 * 1000)    /* software spec 13 */
#define BACKOFF_MS       (15 * 60 * 1000)   /* after 5 failed cycles in a row */
#define BACKOFF_AFTER    5
#define FIRST_DELAY_MS   10000

static volatile bool s_cancel;
static volatile bool s_busy;                /* a poll cycle is running */
static volatile int64_t s_next_poll_us;
static esp_timer_handle_t s_poll_timer;
static TaskHandle_t s_task;
static volatile bool s_auto;                /* boots Manual (spec 5) */
static int s_fail_cycles;                   /* consecutive failed cycles */
static int s_failed_since_report;
static bool s_restarted = true;
static char s_events[160];                  /* rejected commands etc. */

bool net_is_auto(void)  { return s_auto; }
bool net_time_valid(void) { return ts_time_valid(); }
void net_cancel_poll(void) { s_cancel = true; }
bool net_busy(void) { return s_busy; }
int64_t net_next_poll_us(void) { return s_next_poll_us; }

static void event_add(const char *fmt, ...)
{
    char tmp[80];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    size_t have = strlen(s_events);
    if (have + strlen(tmp) + 3 < sizeof s_events) {
        snprintf(s_events + have, sizeof s_events - have, "%s%s", have ? "; " : "", tmp);
    }
}

/* Status message: faults first, then everything else (spec 13, Status Message).
 * Phrases are plain language separated by semicolons. */
static void put_phrase(char *out, size_t cap, const char *fmt, ...)
{
    char tmp[100];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    size_t have = strlen(out);
    if (have + strlen(tmp) + 3 < cap) {
        snprintf(out + have, cap - have, "%s%s", have ? "; " : "", tmp);
    }
}

static void compose_status(char *out, size_t cap)
{
    out[0] = '\0';
    if (actuator_fault(ACT_DOOR)) put_phrase(out, cap, "door fault");
    if (actuator_fault(ACT_LIFT)) put_phrase(out, cap, "lift fault");
    if (s_failed_since_report > 0) put_phrase(out, cap, "wifi failed %d cycles", s_failed_since_report);
    if (s_restarted) put_phrase(out, cap, "restarted");
    if (pump_battery_low()) put_phrase(out, cap, "battery low");
    char ps[80];
    pump_summary(ps, sizeof ps);
    if (ps[0]) put_phrase(out, cap, "%s", ps);
    if (s_events[0]) put_phrase(out, cap, "%s", s_events);
}

static void handle_mode(int mode)
{
    if (mode == 1) {
        s_auto = false;
        ESP_LOGI(TAG, "mode -> Manual");
    } else if (mode == 2) {
        s_auto = true;
        ESP_LOGI(TAG, "mode -> Auto");
    } else {
        event_add("mode command rejected: %d", mode);
    }
}

static void handle_command(int cmd)
{
    act_id_t id;
    act_dir_t dir;
    switch (cmd) {
    case  1: id = ACT_DOOR; dir = DIR_OPEN;  break;
    case -1: id = ACT_DOOR; dir = DIR_CLOSE; break;
    case  2: id = ACT_LIFT; dir = DIR_OPEN;  break;
    case -2: id = ACT_LIFT; dir = DIR_CLOSE; break;
    default:
        event_add("command rejected: unknown value %d", cmd);
        return;
    }
    const char *what = id == ACT_DOOR ? "door" : "lift";
    act_result_t res = actuator_command(id, dir);
    inputs_poke();                              /* wake the control task so it runs the move */
    switch (res) {
    case ACT_AT_TARGET:
        event_add("command rejected: %s already %s", what, dir == DIR_OPEN ? "open" : "closed");
        break;
    case ACT_BUSY:
        event_add("command rejected: %s busy", what);
        break;
    default:
        ESP_LOGI(TAG, "command %d accepted (%s)", cmd, what);
        break;
    }
}

static void run_cycle(void)
{
    static int handled_cmd_entry, handled_mode_entry;

    if (actuator_any_active()) {
        ESP_LOGI(TAG, "actuator moving: poll skipped until the next period");
        return;
    }
    s_cancel = false;

    wifi_res_t w = wifi_mgr_connect(&s_cancel);
    if (w == WIFI_RES_CANCELLED) {
        ESP_LOGI(TAG, "poll cancelled by a button press");
        wifi_mgr_disconnect();
        return;                                 /* does not count as a failed cycle */
    }
    if (w == WIFI_RES_FAILED) {
        s_fail_cycles++;
        s_failed_since_report++;
        ESP_LOGW(TAG, "wifi failed, %d cycles in a row", s_fail_cycles);
        wifi_mgr_disconnect();
        return;
    }
    led_flash(3);                               /* 3 flashes on Wi-Fi connect */

    ts_cmds_t c;
    if (!ts_read_commands(&c)) {
        s_fail_cycles++;
        wifi_mgr_disconnect();
        return;
    }
    if (s_cancel) {
        wifi_mgr_disconnect();
        return;
    }

    /* Mode first, then the open/close command (spec 13). Each entry id is
     * remembered so a command is never run twice if the write of 0 fails. */
    if (c.have_mode && c.mode != 0 && c.mode_entry != handled_mode_entry) {
        handle_mode(c.mode);
        handled_mode_entry = c.mode_entry;
    }
    if (c.have_cmd && c.cmd != 0 && c.cmd_entry != handled_cmd_entry) {
        handle_command(c.cmd);
        handled_cmd_entry = c.cmd_entry;
    }
    if (s_cancel) {
        wifi_mgr_disconnect();
        return;
    }

    /* Pump settings: read hourly between the wake and sleep hours, in this same
     * Wi-Fi session. A failed read keeps the last-good settings. */
    if (pump_poll_due()) {
        ts_pump_t p;
        if (ts_read_pump(&p)) {
            pump_apply(p.have, p.v);
            pump_mark_read();
        } else {
            event_add("pump channel read failed");
        }
        if (s_cancel) {
            wifi_mgr_disconnect();
            return;
        }
    }

    char status[200];
    compose_status(status, sizeof status);
    ts_report_t r = {
        .rssi = wifi_mgr_rssi(),
        .door = actuator_report_state(ACT_DOOR),
        .lift = actuator_report_state(ACT_LIFT),
        .light = sensors_photo_raw(),
        .battery = sensors_battery_raw(),
        .have_temp = false,                     /* DS18B20 not built yet */
        .status = status,
    };
    int entry = 0;
    if (ts_write_report(&r, &entry)) {
        ESP_LOGI(TAG, "reported, entry %d, time valid %d", entry, (int)ts_time_valid());
        s_fail_cycles = 0;
        s_failed_since_report = 0;
        s_restarted = false;
        s_events[0] = '\0';
        pump_report_done();
    } else {
        s_fail_cycles++;
    }
    wifi_mgr_disconnect();
}

/* The poll is scheduled with an esp_timer, not vTaskDelay: esp_timer time keeps
 * running through light sleep, FreeRTOS ticks do not (software spec 7). */
static void schedule_poll(uint32_t ms)
{
    esp_timer_stop(s_poll_timer);
    s_next_poll_us = esp_timer_get_time() + (int64_t)ms * 1000;
    esp_timer_start_once(s_poll_timer, (uint64_t)ms * 1000);
}

static void poll_timer_cb(void *arg) { xTaskNotifyGive(s_task); }

static void poll_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        s_busy = true;
        run_cycle();
        s_busy = false;
        schedule_poll((s_fail_cycles >= BACKOFF_AFTER) ? BACKOFF_MS : POLL_MS);
    }
}

esp_err_t net_init(void)
{
    ts_init();
    ESP_ERROR_CHECK(wifi_mgr_init());
    const esp_timer_create_args_t targs = { .callback = poll_timer_cb, .name = "poll" };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_poll_timer));
    xTaskCreatePinnedToCore(poll_task, "net", 8192, NULL, 4, &s_task, 0);
    schedule_poll(FIRST_DELAY_MS);
    return ESP_OK;
}
