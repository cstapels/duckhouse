#include "led.h"
#include "pins.h"
#include "driver/gpio.h"
#include "esp_timer.h"

/* Software spec section 10. The timer only runs while a pattern is active, so
 * an idle LED never wakes the chip from light sleep. Tick is 125 ms: one flash
 * is one tick on and one tick off (about 150 ms in the spec), the moving
 * pattern toggles every 2 ticks (2 Hz), and there is a 1 s pause after a
 * flash sequence. */
#define TICK_MS        125
#define PAUSE_TICKS    8

static esp_timer_handle_t s_timer;
static volatile bool s_running;
static volatile bool s_moving;
static volatile int s_flashes_left;
static bool s_on;
static int s_ticks;
static int s_pause;

static void set(bool on)
{
    s_on = on;
    gpio_set_level(PIN_STATUS_LED, on ? 1 : 0);
}

static void tick_cb(void *arg)
{
    if (s_moving) {
        if ((++s_ticks & 1) == 0) {
            set(!s_on);
        }
        return;
    }
    s_ticks = 0;
    if (s_flashes_left > 0) {
        if (!s_on) {
            set(true);
        } else {
            set(false);
            if (--s_flashes_left == 0) {
                s_pause = PAUSE_TICKS;
            }
        }
        return;
    }
    if (s_pause > 0) {
        s_pause--;
        return;
    }
    set(false);
    s_running = false;
    esp_timer_stop(s_timer);
}

static void ensure_running(void)
{
    if (!s_running) {
        s_running = true;
        esp_timer_start_periodic(s_timer, TICK_MS * 1000);
    }
}

void led_init(void)
{
    const esp_timer_create_args_t args = { .callback = tick_cb, .name = "led" };
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_timer));
}

void led_flash(int n)
{
    s_flashes_left += n;
    ensure_running();
}

void led_set_moving(bool moving)
{
    if (moving == s_moving) {
        return;
    }
    s_moving = moving;
    if (moving) {
        ensure_running();
    }
}

bool led_idle(void) { return !s_running && !s_moving; }
