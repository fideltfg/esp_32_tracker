// gps.c — GPS UART driver, NMEA parsing, UBX 5 Hz configuration, EMA filter.

#include "gps.h"
#include "config.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = "GPS";

// ── Internal state ───────────────────────────────────────────────────────────
static char   s_line[128];
static int    s_line_pos = 0;
static uint8_t *s_uart_buf = NULL;

// EMA filter state (double precision for lat/lon to avoid float error)
static double  s_ema_lat = 0.0, s_ema_lon = 0.0;
static float   s_ema_alt = 0.0f;
static bool    s_ema_init = false;

// Position lock
static bool    s_pos_locked   = false;
static float   s_locked_lat   = 0.0f;
static float   s_locked_lon   = 0.0f;
static float   s_locked_alt   = 0.0f;

static bool    s_has_fix = false;

// ── NMEA Parsing ─────────────────────────────────────────────────────────────

static bool parse_gprmc(const char *nmea, gps_data_t *gps)
{
    char copy[128];
    strncpy(copy, nmea, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char *token = strtok(copy, ",");
    if (!token || strcmp(token, "$GPRMC") != 0) return false;

    // Time
    char time_str[16] = {0};
    token = strtok(NULL, ",");
    if (token) strncpy(time_str, token, sizeof(time_str) - 1);

    // Status
    token = strtok(NULL, ",");
    if (!token || token[0] != 'A') { gps->valid = false; return false; }
    gps->valid = true;

    // Latitude
    token = strtok(NULL, ",");
    if (token) {
        float raw = atof(token);
        int deg = (int)(raw / 100);
        gps->latitude = deg + (raw - deg * 100) / 60.0f;
        token = strtok(NULL, ",");
        if (token && token[0] == 'S') gps->latitude = -gps->latitude;
    }

    // Longitude
    token = strtok(NULL, ",");
    if (token) {
        float raw = atof(token);
        int deg = (int)(raw / 100);
        gps->longitude = deg + (raw - deg * 100) / 60.0f;
        token = strtok(NULL, ",");
        if (token && token[0] == 'W') gps->longitude = -gps->longitude;
    }

    // Speed (knots → km/h)
    token = strtok(NULL, ",");
    if (token) gps->speed = atof(token) * 1.852f;

    // Course (skip)
    token = strtok(NULL, ",");

    // Date
    token = strtok(NULL, ",");
    if (token && strlen(token) >= 6 && strlen(time_str) >= 6) {
        struct tm t = {0};
        t.tm_hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
        t.tm_min  = (time_str[2] - '0') * 10 + (time_str[3] - '0');
        t.tm_sec  = (time_str[4] - '0') * 10 + (time_str[5] - '0');
        t.tm_mday = (token[0] - '0') * 10 + (token[1] - '0');
        int mon   = (token[2] - '0') * 10 + (token[3] - '0');
        int yy    = (token[4] - '0') * 10 + (token[5] - '0');
        t.tm_mon  = mon - 1;
        t.tm_year = (yy < 70 ? 2000 : 1900) + yy - 1900;
        t.tm_isdst = 0;
        gps->gps_utc_time  = t;
        gps->gps_time_valid = true;
    }
    return true;
}

static bool parse_gpgga(const char *nmea, gps_data_t *gps)
{
    char copy[128];
    strncpy(copy, nmea, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char *token = strtok(copy, ",");
    if (!token || strcmp(token, "$GPGGA") != 0) return false;

    token = strtok(NULL, ","); // Time
    token = strtok(NULL, ","); // Lat
    token = strtok(NULL, ","); // N/S
    token = strtok(NULL, ","); // Lon
    token = strtok(NULL, ","); // E/W

    token = strtok(NULL, ","); // Fix status
    if (!token || token[0] == '0') return false;

    token = strtok(NULL, ","); // Satellites
    if (token) gps->num_sv = atoi(token);

    token = strtok(NULL, ","); // HDOP
    if (token) gps->hdop = atof(token);

    token = strtok(NULL, ","); // Altitude
    if (token) gps->altitude = atof(token);

    return true;
}

// ── UBX Protocol ─────────────────────────────────────────────────────────────

static void ubx_checksum(const uint8_t *data, size_t len,
                          uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0; *ck_b = 0;
    for (size_t i = 0; i < len; i++) {
        *ck_a = (uint8_t)(*ck_a + data[i]);
        *ck_b = (uint8_t)(*ck_b + *ck_a);
    }
}

static void ubx_send(uint8_t *msg, size_t len)
{
    ubx_checksum(&msg[2], len - 4, &msg[len - 2], &msg[len - 1]);
    uart_write_bytes(GPS_UART_NUM, (const char *)msg, (int)len);
}

static bool ubx_wait_ack(uint8_t ack_cls, uint8_t ack_id)
{
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(300);
    uint8_t buf[1];
    uint8_t state = 0, ack_type = 0;

    while (xTaskGetTickCount() < deadline) {
        int n = uart_read_bytes(GPS_UART_NUM, buf, 1, pdMS_TO_TICKS(10));
        if (n <= 0) continue;

        switch (state) {
            case 0: if (buf[0] == 0xB5) state = 1; break;
            case 1: state = (buf[0] == 0x62) ? 2 : (buf[0] == 0xB5 ? 1 : 0); break;
            case 2: state = (buf[0] == 0x05) ? 3 : (buf[0] == 0xB5 ? 1 : 0); break;
            case 3:
                if (buf[0] == 0x01 || buf[0] == 0x00) { ack_type = buf[0]; state = 4; }
                else state = (buf[0] == 0xB5) ? 1 : 0;
                break;
            case 4: state = (buf[0] == 0x02) ? 5 : (buf[0] == 0xB5 ? 1 : 0); break;
            case 5: state = (buf[0] == 0x00) ? 6 : (buf[0] == 0xB5 ? 1 : 0); break;
            case 6: if (buf[0] == ack_cls) state = 7; else state = (buf[0] == 0xB5) ? 1 : 0; break;
            case 7:
                if (buf[0] == ack_id) return (ack_type == 0x01);
                state = (buf[0] == 0xB5) ? 1 : 0;
                break;
            default: state = 0;
        }
    }
    return false;
}

static void gps_configure_5hz(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));

    // Disable GLL, GSA, GSV, VTG
    static const struct { uint8_t cls; uint8_t id; const char *name; } disable[] = {
        { 0xF0, 0x01, "GLL" }, { 0xF0, 0x02, "GSA" },
        { 0xF0, 0x03, "GSV" }, { 0xF0, 0x05, "VTG" },
    };
    for (int i = 0; i < 4; i++) {
        uint8_t msg[] = { 0xB5,0x62, 0x06,0x01, 0x03,0x00,
                          disable[i].cls, disable[i].id, 0x00, 0x00,0x00 };
        ubx_send(msg, sizeof(msg));
        if (ubx_wait_ack(0x06, 0x01))
            ESP_LOGI(TAG, "Disabled %s", disable[i].name);
        else
            ESP_LOGE(TAG, "Disable %s failed", disable[i].name);
    }

    // 5 Hz rate
    uint8_t rate[] = { 0xB5,0x62, 0x06,0x08, 0x06,0x00,
                       0xC8,0x00, 0x01,0x00, 0x01,0x00, 0x00,0x00 };
    ubx_send(rate, sizeof(rate));
    ESP_LOGI(TAG, "5 Hz rate: %s", ubx_wait_ack(0x06, 0x08) ? "OK" : "FAIL");

    // Pedestrian model + static hold
    uint8_t nav5[] = {
        0xB5,0x62, 0x06,0x24, 0x24,0x00,
        0x41,0x00, 0x03, 0x02,
        0x00,0x00,0x00,0x00, 0x10,0x27,0x00,0x00,
        0x05, 0x00, 0xFA,0x00, 0xFA,0x00, 0x64,0x00, 0x2C,0x01,
        DEFAULT_GPS_STATIC_HOLD, 0x00,
        0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
        0x00,0x00
    };
    ubx_send(nav5, sizeof(nav5));
    ESP_LOGI(TAG, "NAV5: %s", ubx_wait_ack(0x06, 0x24) ? "OK" : "FAIL");

    // SBAS
    uint8_t sbas[] = {
        0xB5,0x62, 0x06,0x16, 0x08,0x00,
        0x07, 0x03, 0x03, 0x00,
        (uint8_t)(DEFAULT_GPS_SBAS_MASK & 0xFF),
        (uint8_t)((DEFAULT_GPS_SBAS_MASK >> 8) & 0xFF),
        (uint8_t)((DEFAULT_GPS_SBAS_MASK >> 16) & 0xFF),
        (uint8_t)((DEFAULT_GPS_SBAS_MASK >> 24) & 0xFF),
        0x00,0x00
    };
    ubx_send(sbas, sizeof(sbas));
    ESP_LOGI(TAG, "SBAS: %s", ubx_wait_ack(0x06, 0x16) ? "OK" : "FAIL");
}

// ── Public API ───────────────────────────────────────────────────────────────

void gps_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate  = GPS_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(GPS_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_NUM, &uart_cfg);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "UART%d initialised (TX=%d RX=%d)", GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN);

    s_uart_buf = malloc(UART_BUF_SIZE);
    configASSERT(s_uart_buf);

    gps_configure_5hz();
}

bool gps_read(gps_data_t *out)
{
    bool new_fix = false;

    int len = uart_read_bytes(GPS_UART_NUM, s_uart_buf, UART_BUF_SIZE,
                              pdMS_TO_TICKS(GPS_SAMPLE_PERIOD_MS));
    if (len <= 0) return false;

    for (int i = 0; i < len; i++) {
        char c = (char)s_uart_buf[i];
        if (c == '\n' || c == '\r') {
            if (s_line_pos > 0) {
                s_line[s_line_pos] = '\0';
                if (strncmp(s_line, "$GPRMC", 6) == 0) {
                    parse_gprmc(s_line, out);
                    if (out->valid) { new_fix = true; s_has_fix = true; }
                } else if (strncmp(s_line, "$GPGGA", 6) == 0) {
                    parse_gpgga(s_line, out);
                }
                s_line_pos = 0;
            }
        } else if (s_line_pos < (int)sizeof(s_line) - 1) {
            s_line[s_line_pos++] = c;
        }
    }
    return new_fix;
}

bool gps_has_fix(void) { return s_has_fix; }

// ── EMA Position Filter ─────────────────────────────────────────────────────

void gps_filter_position(gps_data_t *data, bool is_static)
{
    const tracker_config_t *cfg = config_get();

    // Quality gate
    if (data->hdop > cfg->gps_max_hdop || data->num_sv < cfg->gps_min_sv)
        return;

    if (!s_ema_init) {
        s_ema_lat  = data->latitude;
        s_ema_lon  = data->longitude;
        s_ema_alt  = data->altitude;
        s_ema_init = true;
    } else if (is_static && !s_pos_locked) {
        // Outlier rejection
        double dlat   = data->latitude  - s_ema_lat;
        double dlon   = data->longitude - s_ema_lon;
        double coslat = cos(s_ema_lat * M_PI / 180.0);
        double dist_m = sqrt((dlat*dlat + dlon*dlon*coslat*coslat) * 111120.0 * 111120.0);
        if (dist_m < cfg->gps_outlier_m) {
            float a = cfg->gps_ema_alpha;
            s_ema_lat = a * data->latitude  + (1.0 - a) * s_ema_lat;
            s_ema_lon = a * data->longitude + (1.0 - a) * s_ema_lon;
            s_ema_alt = a * data->altitude  + (1.0f - a) * s_ema_alt;
        }
    } else if (!is_static) {
        // Moving — snap to raw
        s_ema_lat = data->latitude;
        s_ema_lon = data->longitude;
        s_ema_alt = data->altitude;
    }

    // Apply filtered or locked position
    if (s_pos_locked) {
        data->latitude  = s_locked_lat;
        data->longitude = s_locked_lon;
        data->altitude  = s_locked_alt;
    } else if (is_static) {
        data->latitude  = (float)s_ema_lat;
        data->longitude = (float)s_ema_lon;
        data->altitude  = s_ema_alt;
    }
}

void gps_lock_position(void)
{
    if (s_pos_locked) return;
    s_pos_locked  = true;
    s_locked_lat  = (float)s_ema_lat;
    s_locked_lon  = (float)s_ema_lon;
    s_locked_alt  = s_ema_alt;
    ESP_LOGI(TAG, "Position locked at %.6f, %.6f",
             (double)s_locked_lat, (double)s_locked_lon);
}

void gps_unlock_position(void)
{
    if (!s_pos_locked) return;
    s_pos_locked = false;
    ESP_LOGI(TAG, "Position lock released");
}

bool gps_is_position_locked(void) { return s_pos_locked; }

// ── Timezone Detection ──────────────────────────────────────────────────────

const char *gps_get_timezone(float latitude, float longitude)
{
    // North America
    if (latitude >= 14.0f && latitude <= 83.0f &&
        longitude >= -168.0f && longitude <= -52.0f) {
        if (latitude >= 18.0f && latitude <= 23.0f &&
            longitude >= -162.0f && longitude <= -154.0f) return "HST10";
        if (latitude >= 54.0f && longitude <= -130.0f)
            return "AKST9AKDT,M3.2.0,M11.1.0";
        if (latitude >= 46.0f && longitude >= -59.0f)
            return "NST3:30NDT,M3.2.0/0:01,M11.1.0/0:01";
        if (latitude >= 43.0f && latitude <= 52.0f &&
            longitude >= -66.0f && longitude <= -59.0f)
            return "AST4ADT,M3.2.0,M11.1.0";
        if (latitude >= 49.0f && latitude <= 60.0f &&
            longitude >= -110.0f && longitude <= -101.0f) return "CST6";
        if (latitude >= 60.0f && longitude <= -124.0f) return "MST7";
        if (longitude < -120.0f) return "PST8PDT,M3.2.0,M11.1.0";
        if (longitude < -102.0f) return "MST7MDT,M3.2.0,M11.1.0";
        if (longitude <  -82.0f) return "CST6CDT,M3.2.0,M11.1.0";
        return "EST5EDT,M3.2.0/2,M11.1.0";
    }

    // Australia
    if (latitude >= -44.0f && latitude <= -10.0f &&
        longitude >= 113.0f && longitude <= 167.0f) {
        if (longitude >= 159.0f) return "LHST-10:30LHDT-11,M10.1.0,M4.1.0";
        if (longitude < 129.0f) return "AWST-8";
        if (longitude < 138.0f) return "ACST-9:30ACDT,M10.1.0,M4.1.0/3";
        return "AEST-10AEDT,M10.1.0,M4.1.0/3";
    }

    // New Zealand
    if (latitude >= -48.0f && latitude <= -34.0f &&
        longitude >= 165.0f && longitude <= 178.5f)
        return "NZST-12NZDT,M9.5.0,M4.1.0/3";

    // Europe: Iceland / Azores
    if (latitude >= 35.0f && latitude <= 71.0f &&
        longitude >= -28.0f && longitude < -15.0f) return "GMT0";

    // Europe
    if (latitude >= 35.0f && latitude <= 71.0f &&
        longitude >= -15.0f && longitude <= 40.0f) {
        if (longitude <  7.5f) return "GMT0BST,M3.5.0/1,M10.5.0";
        if (longitude < 22.5f) return "CET-1CEST,M3.5.0,M10.5.0/3";
        return "EET-2EEST,M3.5.0/3,M10.5.0/4";
    }

    // Israel
    if (latitude >= 29.0f && latitude <= 34.0f &&
        longitude >= 34.0f && longitude <= 36.5f) return "IST-2IDT,M3.4.4/26,M10.5.0";

    // Gulf
    if (latitude >= 12.0f && latitude <= 30.0f &&
        longitude >= 44.0f && longitude <= 60.0f) return "GST-4";

    // India
    if (latitude >= 8.0f && latitude <= 37.0f &&
        longitude >= 68.0f && longitude <= 97.0f) return "IST-5:30";

    // China
    if (latitude >= 18.0f && latitude <= 54.0f &&
        longitude >= 73.0f && longitude <= 135.0f) return "CST-8";

    // Japan / Korea
    if (latitude >= 24.0f && latitude <= 46.0f &&
        longitude >= 123.0f && longitude <= 146.0f) return "JST-9";

    // Generic longitude bands
    int offset = (int)((longitude + 7.5f) / 15.0f);
    if (offset == 0)               return "GMT0";
    static char tz_buf[16];
    if (offset > 0) snprintf(tz_buf, sizeof(tz_buf), "UTC-%d", offset);
    else            snprintf(tz_buf, sizeof(tz_buf), "UTC%d", -offset);
    return tz_buf;
}
