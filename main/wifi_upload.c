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

// esp_crt_bundle_attach lives in the esp_tls component whose include path is
// not exposed to main; forward-declare the symbol so the compiler accepts it
// (the linker will resolve it via esp_http_client → esp_tls).
esp_err_t esp_crt_bundle_attach(void *conf);

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
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
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
    }

    // Always create the event group and register this module's event handlers.
    // wifi_connect() calls xEventGroupClearBits / xEventGroupWaitBits on
    // s_wifi_event_group regardless of who initialised the WiFi driver, so the
    // handle must never be NULL when wifi_connect() is called.
    s_wifi_event_group = xEventGroupCreate();
    configASSERT(s_wifi_event_group);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                    IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    s_initialized = true;
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
    // one buffer slot per data row, 220 bytes each + brackets
    size_t json_cap = (size_t)(newlines > 1 ? newlines : 2) * 220 + 8;
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

    bool first_data = true;
    char *saveptr = NULL;
    char *line = strtok_r(buf, "\n", &saveptr);

    while (line) {
        // Strip '\r'
        size_t ll = strlen(line);
        if (ll > 0 && line[ll - 1] == '\r') line[--ll] = '\0';
        if (ll == 0) { line = strtok_r(NULL, "\n", &saveptr); continue; }

        // Parse the 9 fields: timestamp,lat,lon,alt,speed,accel_x,accel_y,accel_z,device_mac
        char f[9][64] = {{0}};
        char *p = line;
        for (int i = 0; i < 9; i++) {
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

        // Skip rows that don't start with a digit — guards against old header
        // rows or any other non-data lines that may exist on the SD card.
        if (f[0][0] < '0' || f[0][0] > '9') { line = strtok_r(NULL, "\n", &saveptr); continue; }
        // Skip rows with an empty device_mac
        if (f[8][0] == '\0') { line = strtok_r(NULL, "\n", &saveptr); continue; }

        // Guard against buffer overrun
        if (out + 220 >= end) {
            break;
        }

        if (!first_data) *out++ = ',';
        first_data = false;

        // timestamp is an integer; lat/lon/alt/speed/accel_* are floats; mac is a string
        out += snprintf(out, (size_t)(end - out),
                        "{\"timestamp\":%s,\"lat\":%s,\"lon\":%s,"
                        "\"alt\":%s,\"speed\":%s,"
                        "\"accel_x\":%s,\"accel_y\":%s,\"accel_z\":%s,"
                        "\"device_mac\":\"%s\"}",
                        f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8]);

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

// ─── HTTP event handler — captures response body ──────────────────────────────

typedef struct {
    char  *buf;
    size_t len;
    size_t cap;
} resp_ctx_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    resp_ctx_t *ctx = (resp_ctx_t *)evt->user_data;
    if (!ctx) return ESP_OK;

    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data_len > 0) {
        size_t remaining = ctx->cap - ctx->len - 1;
        size_t copy = (size_t)evt->data_len < remaining ? (size_t)evt->data_len : remaining;
        memcpy(ctx->buf + ctx->len, evt->data, copy);
        ctx->len += copy;
        ctx->buf[ctx->len] = '\0';
    } else if (evt->event_id == HTTP_EVENT_ON_CONNECTED) {
        // Reset the response buffer at the start of each new connection.
        ctx->len = 0;
        if (ctx->buf) ctx->buf[0] = '\0';
    }
    return ESP_OK;
}

// \u2500\u2500\u2500 HTTP POST helper \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
// POST a pre-built JSON string to UPLOAD_URL.  Returns true on HTTP 2xx.

static bool http_post_json(const char *label, const char *json)
{
    const size_t RESP_BUF = 2048;
    char *resp_buf = calloc(1, RESP_BUF);
    if (!resp_buf) return false;

    resp_ctx_t resp_ctx = { .buf = resp_buf, .len = 0, .cap = RESP_BUF };

    bool is_https = (strncmp(UPLOAD_URL, "https://", 8) == 0);

    esp_http_client_config_t http_cfg = {
        .url               = UPLOAD_URL,
        .method            = HTTP_METHOD_POST,
        .timeout_ms        = 10000,
        .crt_bundle_attach = is_https ? esp_crt_bundle_attach : NULL,
        .event_handler     = http_event_handler,
        .user_data         = &resp_ctx,
    };

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) {
        ESP_LOGE(TAG, "%s: Failed to initialize HTTP client", label);
        free(resp_buf);
        return false;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept",       "application/json");
    esp_http_client_set_post_field(client, json, (int)strlen(json));

    esp_err_t err = esp_http_client_perform(client);
    bool ok = false;
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status >= 200 && status < 300) {
            ESP_LOGI(TAG, "%s: HTTP %d  %s", label, status, resp_buf);
            ok = true;
        } else {
            ESP_LOGW(TAG, "%s: server rejected chunk (HTTP %d): %s", label, status, resp_buf);
        }
    } else {
        ESP_LOGE(TAG, "%s: HTTP error: %s", label, esp_err_to_name(err));
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(resp_buf);

    // Brief pause to let the TCP stack release the socket.
    vTaskDelay(pdMS_TO_TICKS(100));
    return ok;
}

// \u2500\u2500\u2500 Paged file upload \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
// Reads a staged file in 32 KB pages (aligned to complete CSV lines) and POSTs
// each page as a separate JSON array.  Returns true only when every page is
// confirmed by the server; on failure the staged file is preserved for retry.

static bool upload_staged_file(const char *label, const char *filepath)
{
    const size_t READ_BUF = 32 * 1024;
    char *csv_buf = malloc(READ_BUF);
    if (!csv_buf) {
        ESP_LOGE(TAG, "%s: OOM allocating read buffer", label);
        return false;
    }

    long offset    = 0;
    int  chunks_ok = 0;

    while (true) {
        int csv_len = sd_read_chunk(filepath, csv_buf, READ_BUF, &offset);
        if (csv_len == 0) break;   // clean EOF
        if (csv_len  < 0) {
            ESP_LOGE(TAG, "%s: read error at offset %ld", label, offset);
            free(csv_buf);
            return false;
        }

        char *json = csv_to_json(csv_buf, csv_len);
        if (!json) {
            // No uploadable rows in this chunk (header-only, etc.) -- skip.
            continue;
        }

        ESP_LOGI(TAG, "%s: uploading chunk %d (%d JSON bytes)",
                 label, chunks_ok + 1, (int)strlen(json));

        bool ok = http_post_json(label, json);
        free(json);

        if (!ok) {
            // Staged file preserved -- will be retried on next upload window.
            free(csv_buf);
            return false;
        }

        chunks_ok++;
        vTaskDelay(pdMS_TO_TICKS(200)); // gap between chunks
    }

    free(csv_buf);

    if (chunks_ok == 0) {
        ESP_LOGI(TAG, "%s: file had no uploadable data rows", label);
    }

    return true; // all chunks confirmed (or file was empty)
}

// \u2500\u2500\u2500 Public API \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500

static wifi_upload_result_t do_csv_upload(
    const char *label,
    bool (*stage_fn)(void), bool (*ensure_fn)(void),
    const char *stage_path, const char *bak_path)
{
    if (!stage_fn()) return WIFI_UPLOAD_NO_DATA;
    ensure_fn();
    bool ok = upload_staged_file(label, stage_path);
    if (ok) {
        // Append the staged file's contents to the single accumulating .bak
        // archive, then remove the staged file.
        sd_lock();
        FILE *fsrc = fopen(stage_path, "r");
        FILE *fdst = fsrc ? fopen(bak_path, "a") : NULL;
        if (fsrc && fdst) {
            char iobuf[512];
            size_t n;
            while ((n = fread(iobuf, 1, sizeof(iobuf), fsrc)) > 0)
                fwrite(iobuf, 1, n, fdst);
            ESP_LOGI(TAG, "%s: appended to %s", label, bak_path);
        } else {
            ESP_LOGW(TAG, "%s: could not open files for archive copy", label);
        }
        if (fdst)  fclose(fdst);
        if (fsrc)  fclose(fsrc);
        remove(stage_path);
        sd_unlock();
    }
    return ok ? WIFI_UPLOAD_UPLOADED : WIFI_UPLOAD_FAILED;
}

wifi_upload_result_t wifi_upload_csv(void)
{
    return do_csv_upload("own-data",
                         sd_stage_data_for_upload, sd_ensure_data_file,
                         CSV_UPLOAD_STAGE, CSV_DATA_BAK);
}

wifi_upload_result_t wifi_upload_merged_csv(void)
{
    return do_csv_upload("peer-data",
                         sd_stage_merged_for_upload, sd_ensure_merged_file,
                         CSV_MERGE_STAGE, CSV_MERGED_BAK);
}

bool wifi_upload_all_csv(wifi_upload_report_t *report)
{
    wifi_upload_report_t local_report = {
        .own_data    = WIFI_UPLOAD_NO_DATA,
        .merged_data = WIFI_UPLOAD_NO_DATA,
    };

    local_report.own_data = wifi_upload_csv();
    if (local_report.own_data == WIFI_UPLOAD_FAILED) {
        ESP_LOGW(TAG, "Own data upload failed - staging file preserved for retry");
        if (report) *report = local_report;
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(200));

    local_report.merged_data = wifi_upload_merged_csv();
    if (report) *report = local_report;

    return local_report.merged_data != WIFI_UPLOAD_FAILED;
}