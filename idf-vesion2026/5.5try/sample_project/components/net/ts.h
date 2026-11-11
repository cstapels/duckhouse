#pragma once
#include <stdbool.h>

/* Newest non-empty command and mode values on the duck door channel
 * (software spec 13, "How Commands Reach the Channel"). Empty fields count as
 * "no value". */
typedef struct {
    bool have_cmd;  int cmd;  int cmd_entry;     /* field 1 */
    bool have_mode; int mode; int mode_entry;    /* field 5 */
} ts_cmds_t;

typedef struct {
    int rssi;        /* field 2 */
    int door;        /* field 3 */
    int light;       /* field 4 */
    int lift;        /* field 6 */
    bool have_temp;
    float temp;      /* field 7 */
    int battery;     /* field 8, raw ADC count */
    const char *status;   /* ThingSpeak status message, may be "" */
} ts_report_t;

/* Pump channel fields 1..6 (index 0..5): newest non-empty value of each. */
typedef struct {
    bool have[6];
    int v[6];
} ts_pump_t;

void ts_init(void);
bool ts_read_commands(ts_cmds_t *out);
/* Writes the report with field 1 and field 5 set to 0, parses created_at from
 * the JSON reply and sets the system clock. */
bool ts_write_report(const ts_report_t *r, int *entry_id);
bool ts_read_pump(ts_pump_t *out);
bool ts_time_valid(void);
