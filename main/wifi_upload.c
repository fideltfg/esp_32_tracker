#include "wifi_upload.h"
#include "sync_config.h"
#include "sd_storage.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_upload";

// Bit flags for event group
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static EventGroupHandle_t s_wifi_event_group = NULL;
static int                s_retry_count       = 0;
static bool               s_initialized       = false;

// ─── Event handler ────────────────────────────────────────────────────────────

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGI(TAG, "Retry %d/%d ...", s_retry_count, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGW(TAG, "WiFi connection failed after %d retries", WIFI_MAX_RETRY);
        }
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ─── Public API ───────────────────────────────────────────────────────────────

void wifi_stack_init(void)
{
    if (s_initialized) return;

    // Check whether the WiFi driver is already up (e.g. from tracker's wifi_init_sta).
    // esp_wifi_get_mode() returns ESP_ERR_WIFI_NOT_INIT when the driver has not been
    // initialised yet.
    wifi_mode_t mode;
    bool wifi_already_up = (esp_wifi_get_mode(&mode) != ESP_ERR_WIFI_NOT_INIT);

    if (!wifi_already_up) {
        // ── Standalone use: perform a full WiFi stack initialisation ──
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ESP_ERROR_CHECK(nvs_flash_init());
        }

        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        s_wifi_event_group = xEventGroupCreate();
        configASSERT(s_wifi_event_group);

        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                        ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                        IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    }
    // When the tracker already owns the WiFi stack, wifi_is_connected() queries the
    // driver directly via esp_wifi_sta_get_ap_info(), so no event group is needed.

    s_initialized = true;
    ESP_LOGI(TAG, "WiFi upload module ready (driver %s)",
             wifi_already_up ? "shared" : "owned");
}

bool wifi_connect(void)
{
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    s_retry_count = 0;

    wifi_config_t wifi_cfg = { 0 };
    // Safe copies — SSID and password are compile-time constants
    strncpy((char *)wifi_cfg.sta.ssid,     WIFI_SSID,     sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASSWORD, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    // WiFi is already started in STA mode by the state machine.
    // Just apply the config and kick off the connection.
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    esp_wifi_connect();

    // Block until connected or exhausted retries
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            portMAX_DELAY);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

void wifi_disconnect(void)
{
    esp_wifi_disconnect();
    // esp_wifi_stop() is called by the state machine after espnow_deinit()
    ESP_LOGI(TAG, "WiFi disconnected");
}

bool wifi_is_connected(void)
{
    // Query the driver directly so this works even when the WiFi stack was
    // initialised externally (e.g. by the tracker's wifi_init_sta()) and the
    // upload module's event group never received the initial connect event.
    wifi_ap_record_t ap_info;
    return (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);
}

// ─── CSV → JSON conversion ────────────────────────────────────────────────────
// Converts CSV text (with header row) into a JSON array of objects.
// Returns a heap-allocated null-terminated string; caller must free().
// Returns NULL if there are no data rows or on allocation failure.
//
// CSV columns: timestamp,lat,lon,accel_x,accel_y,accel_z,device_mac
static char *csv_to_json(const char *csv, int csv_len)
{
    // Allocate a JSON buffer — each row expands to roughly 150 bytes of JSON.
    // We count newlines as a proxy for row count, then size generously.
    int newlines = 0;
    for (int i = 0; i < csv_len; i++) {
        if (csv[i] == '\n') newlines++;
    }
    // newlines - 1 data rows (subtract header), 160 bytes each + brackets
    size_t json_cap = (size_t)(newlines > 1 ? newlines : 2) * 160 + 8;
    char *json = malloc(json_cap);
    if (!json) return NULL;

    char *out = json;
    char *end = json + json_cap - 2; // safety margin

    *out++ = '[';

    // Work on a mutable copy
    char *buf = malloc(csv_len + 1);
    if (!buf) { free(json); return NULL; }
    memcpy(buf, csv, csv_len);
    buf[csv_len] = '\0';

    bool first_row = true;
    bool first_data = true;
    char *saveptr = NULL;
    char *line = strtok_r(buf, "\n", &saveptr);

    while (line) {
        // Strip '\r'
        size_t ll = strlen(line);
        if (ll > 0 && line[ll - 1] == '\r') line[--ll] = '\0';
        if (ll == 0) { line = strtok_r(NULL, "\n", &saveptr); continue; }

        // Skip header
        if (first_row) {
            first_row = false;
            if (strncmp(line, "timestamp", 9) == 0) {
                line = strtok_r(NULL, "\n", &saveptr);
                continue;
            }
        }

        // Parse the 7 fields
        char f[7][64] = {{0}};
        char *p = line;
        for (int i = 0; i < 7; i++) {
            char *comma = strchr(p, ',');
            size_t flen;
            if (comma) {
                flen = (size_t)(comma - p);
            } else {
                // last field — stop at newline/end
                flen = strcspn(p, "\r\n");
            }
            if (flen >= sizeof(f[i])) flen = sizeof(f[i]) - 1;
            memcpy(f[i], p, flen);
            f[i][flen] = '\0';
            if (comma) p = comma + 1; else break;
        }

        // Guard against buffer overrun
        if (out + 160 >= end) {
            ESP_LOGW(TAG, "JSON buffer full, truncating upload");
            break;
        }

        if (!first_data) *out++ = ',';
        first_data = false;

        // timestamp is an integer; lat/lon/accel_* are floats; mac is a string
        out += snprintf(out, (size_t)(end - out),
                        "{\"timestamp\":%s,\"lat\":%s,\"lon\":%s,"
                        "\"accel_x\":%s,\"accel_y\":%s,\"accel_z\":%s,"
                        "\"device_mac\":\"%s\"}",
                        f[0], f[1], f[2], f[3], f[4], f[5], f[6]);

        line = strtok_r(NULL, "\n", &saveptr);
    }

    free(buf);

    if (first_data) {
        // No data rows were converted
        free(json);
        return NULL;
    }

    *out++ = ']';
    *out   = '\0';
    return json;
}

// ─── Generic file upload ──────────────────────────────────────────────────────

static bool upload_file(const char *label, int (*reader)(char *, size_t))
{
    const size_t READ_BUF = 32 * 1024;
    char *csv_buf = malloc(READ_BUF);
    if (!csv_buf) {
        ESP_LOGE(TAG, "OOM allocating read buffer");
        return false;
    }

    int csv_len = reader(csv_buf, READ_BUF);
    if (csv_len <= 0) {
        ESP_LOGI(TAG, "%s: nothing to upload", label);
        free(csv_buf);
        return true; // nothing to do is not a failure
    }

    char *json = csv_to_json(csv_buf, csv_len);
    free(csv_buf);
    if (!json) {
        ESP_LOGI(TAG, "%s: no data rows after CSV\u2192JSON conversion", label);
        return true;
    }

    ESP_LOGI(TAG, "%s: uploading %d bytes of JSON", label, (int)strlen(json));

    const size_t RESP_BUF = 512;
    char *resp_buf = calloc(1, RESP_BUF);
    if (!resp_buf) { free(json); return false; }

    esp_http_client_config_t http_cfg = {
        .url                         = UPLOAD_URL,
        .method                      = HTTP_METHOD_POST,
        .timeout_ms                  = 3000,
        .skip_cert_common_name_check = true,
        .crt_bundle_attach           = NULL,
        .use_global_ca_store         = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept",       "application/json");
    esp_http_client_set_post_field(client, json, (int)strlen(json));

    esp_err_t err = esp_http_client_perform(client);
    bool ok = false;
    if (err == ESP_OK) {
        int status  = esp_http_client_get_status_code(client);
        int rlen    = esp_http_client_read_response(client, resp_buf, (int)(RESP_BUF - 1));
        if (rlen > 0) resp_buf[rlen] = '\0';
        ESP_LOGI(TAG, "%s: HTTP %d  %s", label, status, resp_buf);
        if ((status == 200 || status == 201) &&
            (strstr(resp_buf, "\"success\":true") ||
             strstr(resp_buf, "\"success\": true"))) {
            ok = true;
        } else {
            ESP_LOGW(TAG, "%s: rejected by server (status=%d)", label, status);
        }
    } else {
        ESP_LOGE(TAG, "%s: HTTP error: %s", label, esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    free(json);
    free(resp_buf);
    return ok;
}

bool wifi_upload_csv(void)
{
    return upload_file("own-data", sd_read_all);
}

bool wifi_upload_merged_csv(void)
{
    return upload_file("peer-data", sd_read_merged);
}

bool wifi_upload_all_csv(void)
{
    bool own_ok    = wifi_upload_csv();
    if(!own_ok) {
        ESP_LOGW(TAG, "Own data upload failed – skipping peer upload");
        return false;
    }
    bool merged_ok = wifi_upload_merged_csv();
    (void)merged_ok; // peer upload failure is non-fatal
    return own_ok;
}
