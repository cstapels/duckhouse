#include "ts.h"
#include "config.h"
#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

static const char *TAG = "ts";

#define DOOR_CHANNEL_DEFAULT "1415485"
#define FEED_RESULTS   10
#define BODY_CAP       12288
#define PUMP_CHANNEL_DEFAULT "1160824"
#define PUMP_RESULTS   20
#define HTTP_TIMEOUT   10000     /* ms, software spec 13 */
#define MIN_EPOCH      1767225600LL   /* 2026-01-01 UTC: reject earlier times */
#define MAX_JUMP_S     3600           /* after the first sync, reject jumps > 1 h */

static bool s_time_valid;

bool ts_time_valid(void) { return s_time_valid; }

void ts_init(void)
{
    /* Boston: EST (UTC-5) / EDT (UTC-4), daylight saving automatic. */
    setenv("TZ", "EST5EDT,M3.2.0,M11.1.0", 1);
    tzset();
}

/* ---------- HTTP ---------- */

typedef struct { char *buf; size_t cap, len; } body_t;

static esp_err_t http_evt(esp_http_client_event_t *e)
{
    body_t *b = (body_t *)e->user_data;
    if (e->event_id == HTTP_EVENT_ON_DATA && b && e->data_len > 0) {
        size_t n = (size_t)e->data_len;
        if (b->len + n < b->cap - 1) {
            memcpy(b->buf + b->len, e->data, n);
            b->len += n;
            b->buf[b->len] = '\0';
        }
    }
    return ESP_OK;
}

/* One retry on failure (software spec 13). Returns the HTTP status, or -1. */
static int http_request(esp_http_client_method_t method, const char *url,
                        const char *post, body_t *out)
{
    for (int attempt = 0; attempt < 2; attempt++) {
        out->len = 0;
        out->buf[0] = '\0';
        esp_http_client_config_t cfg = {
            .url = url,
            .method = method,
            .timeout_ms = HTTP_TIMEOUT,
            .event_handler = http_evt,
            .user_data = out,
            .crt_bundle_attach = esp_crt_bundle_attach,
        };
        esp_http_client_handle_t c = esp_http_client_init(&cfg);
        if (!c) {
            return -1;
        }
        if (post) {
            esp_http_client_set_header(c, "Content-Type", "application/x-www-form-urlencoded");
            esp_http_client_set_post_field(c, post, (int)strlen(post));
        }
        esp_err_t err = esp_http_client_perform(c);
        int status = esp_http_client_get_status_code(c);
        esp_http_client_cleanup(c);
        if (err == ESP_OK) {
            return status;
        }
        ESP_LOGW(TAG, "http attempt %d failed: %s", attempt + 1, esp_err_to_name(err));
    }
    return -1;
}

/* ---------- time ---------- */

static int64_t days_from_civil(int y, int m, int d)     /* Howard Hinnant's algorithm */
{
    y -= m <= 2;
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

/* "2026-11-07T14:22:31Z" or "2026-11-07T09:22:31-05:00" (both appear in the
 * ThingSpeak documentation). */
static bool parse_created_at(const char *s, int64_t *epoch)
{
    int Y, M, D, h, m, sec;
    if (sscanf(s, "%d-%d-%dT%d:%d:%d", &Y, &M, &D, &h, &m, &sec) != 6) {
        return false;
    }
    int offset = 0;
    const char *p = strchr(s, 'T');
    p = p ? p + 9 : NULL;                               /* after HH:MM:SS */
    if (p && (*p == '+' || *p == '-')) {
        int oh = 0, om = 0;
        sscanf(p + 1, "%d:%d", &oh, &om);
        offset = (*p == '-' ? -1 : 1) * (oh * 3600 + om * 60);
    }
    *epoch = days_from_civil(Y, M, D) * 86400 + h * 3600 + m * 60 + sec - offset;
    return true;
}

static void set_clock(int64_t epoch)
{
    if (epoch < MIN_EPOCH) {
        ESP_LOGW(TAG, "ignoring timestamp before 2026");
        return;
    }
    struct timeval now;
    gettimeofday(&now, NULL);
    if (s_time_valid && llabs((long long)(epoch - now.tv_sec)) > MAX_JUMP_S) {
        ESP_LOGW(TAG, "ignoring timestamp jump of %lld s", (long long)(epoch - now.tv_sec));
        return;
    }
    struct timeval tv = { .tv_sec = (time_t)epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    s_time_valid = true;
}

/* ---------- read ---------- */

static bool field_int(const cJSON *entry, const char *name, int *out)
{
    const cJSON *f = cJSON_GetObjectItemCaseSensitive(entry, name);
    if (cJSON_IsString(f) && f->valuestring && f->valuestring[0]) {
        *out = atoi(f->valuestring);
        return true;
    }
    if (cJSON_IsNumber(f)) {
        *out = (int)f->valuedouble;
        return true;
    }
    return false;                                       /* null or missing: empty */
}

bool ts_read_commands(ts_cmds_t *out)
{
    memset(out, 0, sizeof *out);
    char key[40] = "", chan[16] = "", url[200];
    config_get_str("ts_door_read", key, sizeof key, "");
    config_get_str("ts_door_chan", chan, sizeof chan, DOOR_CHANNEL_DEFAULT);
    if (key[0] == '\0') {
        ESP_LOGE(TAG, "no ts_door_read key in NVS (see provision/README.md)");
        return false;
    }
    snprintf(url, sizeof url,
             "https://api.thingspeak.com/channels/%s/feeds.json?api_key=%s&results=%d",
             chan, key, FEED_RESULTS);

    body_t b = { .buf = malloc(BODY_CAP), .cap = BODY_CAP };
    if (!b.buf) {
        return false;
    }
    int status = http_request(HTTP_METHOD_GET, url, NULL, &b);
    bool ok = false;
    if (status == 200) {
        cJSON *root = cJSON_Parse(b.buf);
        const cJSON *feeds = root ? cJSON_GetObjectItemCaseSensitive(root, "feeds") : NULL;
        if (cJSON_IsArray(feeds)) {
            ok = true;
            /* Entries run oldest to newest. Take the newest non-empty value of
             * each field: the device writes 0 after handling, so a handled
             * command is always masked by a newer 0. */
            for (int i = cJSON_GetArraySize(feeds) - 1; i >= 0; i--) {
                const cJSON *e = cJSON_GetArrayItem(feeds, i);
                int id = 0, v;
                field_int(e, "entry_id", &id);
                if (!out->have_cmd && field_int(e, "field1", &v)) {
                    out->have_cmd = true; out->cmd = v; out->cmd_entry = id;
                }
                if (!out->have_mode && field_int(e, "field5", &v)) {
                    out->have_mode = true; out->mode = v; out->mode_entry = id;
                }
            }
        }
        cJSON_Delete(root);
    } else {
        ESP_LOGW(TAG, "feed read failed, http %d", status);
    }
    free(b.buf);
    return ok;
}

/* ---------- write ---------- */

static void urlencode(const char *in, char *out, size_t cap)
{
    static const char hex[] = "0123456789ABCDEF";
    size_t o = 0;
    for (; *in && o + 4 < cap; in++) {
        unsigned char c = (unsigned char)*in;
        if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            c == '-' || c == '_' || c == '.') {
            out[o++] = (char)c;
        } else {
            out[o++] = '%'; out[o++] = hex[c >> 4]; out[o++] = hex[c & 15];
        }
    }
    out[o] = '\0';
}

bool ts_write_report(const ts_report_t *r, int *entry_id)
{
    char key[40] = "";
    config_get_str("ts_door_write", key, sizeof key, "");
    if (key[0] == '\0') {
        ESP_LOGE(TAG, "no ts_door_write key in NVS (see provision/README.md)");
        return false;
    }
    char status[300];
    urlencode(r->status ? r->status : "", status, sizeof status);

    char post[700];
    int n = snprintf(post, sizeof post,
                     "api_key=%s&field1=0&field5=0&field2=%d&field3=%d&field4=%d&field6=%d&field8=%d",
                     key, r->rssi, r->door, r->light, r->lift, r->battery);
    if (r->have_temp) {
        n += snprintf(post + n, sizeof post - n, "&field7=%.2f", r->temp);
    }
    snprintf(post + n, sizeof post - n, "&status=%s", status);

    body_t b = { .buf = malloc(BODY_CAP), .cap = BODY_CAP };
    if (!b.buf) {
        return false;
    }
    /* update.json, not the text endpoint: the JSON reply carries created_at. */
    int http = http_request(HTTP_METHOD_POST, "https://api.thingspeak.com/update.json", post, &b);
    bool ok = false;
    if (http == 200) {
        cJSON *root = cJSON_Parse(b.buf);
        const cJSON *id = root ? cJSON_GetObjectItemCaseSensitive(root, "entry_id") : NULL;
        const cJSON *ts = root ? cJSON_GetObjectItemCaseSensitive(root, "created_at") : NULL;
        if (cJSON_IsNumber(id) && id->valueint > 0) {
            ok = true;
            if (entry_id) *entry_id = id->valueint;
            int64_t epoch;
            if (cJSON_IsString(ts) && parse_created_at(ts->valuestring, &epoch)) {
                set_clock(epoch);
            }
        }
        cJSON_Delete(root);
    }
    if (!ok) {
        ESP_LOGW(TAG, "write failed, http %d", http);
    }
    free(b.buf);
    return ok;
}

/* Pump channel (software spec 13, Pump Channel). The owner edits it rarely and
 * entries may hold only some fields, so take the newest non-empty value of each
 * field from the last PUMP_RESULTS entries. Fields not found keep their value. */
bool ts_read_pump(ts_pump_t *out)
{
    memset(out, 0, sizeof *out);
    char key[40] = "", chan[16] = "", url[200];
    config_get_str("ts_pump_read", key, sizeof key, "");
    config_get_str("ts_pump_chan", chan, sizeof chan, PUMP_CHANNEL_DEFAULT);
    if (key[0] == '\0') {
        ESP_LOGE(TAG, "no ts_pump_read key in NVS (see provision/README.md)");
        return false;
    }
    snprintf(url, sizeof url,
             "https://api.thingspeak.com/channels/%s/feeds.json?api_key=%s&results=%d",
             chan, key, PUMP_RESULTS);

    body_t b = { .buf = malloc(BODY_CAP), .cap = BODY_CAP };
    if (!b.buf) {
        return false;
    }
    int status = http_request(HTTP_METHOD_GET, url, NULL, &b);
    bool ok = false;
    if (status == 200) {
        cJSON *root = cJSON_Parse(b.buf);
        const cJSON *feeds = root ? cJSON_GetObjectItemCaseSensitive(root, "feeds") : NULL;
        if (cJSON_IsArray(feeds)) {
            ok = true;
            for (int i = cJSON_GetArraySize(feeds) - 1; i >= 0; i--) {
                const cJSON *e = cJSON_GetArrayItem(feeds, i);
                for (int f = 0; f < 6; f++) {
                    char name[8];
                    int v;
                    snprintf(name, sizeof name, "field%d", f + 1);
                    if (!out->have[f] && field_int(e, name, &v)) {
                        out->have[f] = true;
                        out->v[f] = v;
                    }
                }
            }
        }
        cJSON_Delete(root);
    } else {
        ESP_LOGW(TAG, "pump channel read failed, http %d", status);
    }
    free(b.buf);
    return ok;
}
