// upload.c — HTTP CSV upload + OTA firmware updates.
//
// WiFi management is delegated to the wifi_manager module.

#include "upload.h"
#include "config.h"
#include "sd_storage.h"

#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"

// esp_crt_bundle_attach lives in the esp_tls component
esp_err_t esp_crt_bundle_attach(void *conf);

static const char *TAG = "UPLOAD";

// ── CSV → JSON ───────────────────────────────────────────────────────────────

static char *csv_to_json(const char *csv, int csv_len)
{
    int newlines = 0;
    for (int i = 0; i < csv_len; i++)
        if (csv[i] == '\n') newlines++;

    size_t cap = (size_t)(newlines > 1 ? newlines : 2) * 220 + 8;
    char *json = malloc(cap);
    if (!json) return NULL;

    char *out = json, *end = json + cap - 2;
    *out++ = '[';

    char *buf = malloc(csv_len + 1);
    if (!buf) { free(json); return NULL; }
    memcpy(buf, csv, csv_len);
    buf[csv_len] = '\0';

    bool first = true;
    char *save = NULL;
    char *line = strtok_r(buf, "\n", &save);

    while (line) {
        size_t ll = strlen(line);
        if (ll > 0 && line[ll - 1] == '\r') line[--ll] = '\0';
        if (ll == 0) { line = strtok_r(NULL, "\n", &save); continue; }

        char f[9][64] = {{0}};
        char *p = line;
        for (int i = 0; i < 9; i++) {
            char *comma = strchr(p, ',');
            size_t flen = comma ? (size_t)(comma - p) : strcspn(p, "\r\n");
            if (flen >= sizeof(f[i])) flen = sizeof(f[i]) - 1;
            memcpy(f[i], p, flen);
            f[i][flen] = '\0';
            if (comma) p = comma + 1; else break;
        }

        if (f[0][0] < '0' || f[0][0] > '9') { line = strtok_r(NULL, "\n", &save); continue; }
        if (f[8][0] == '\0') { line = strtok_r(NULL, "\n", &save); continue; }
        if (out + 220 >= end) break;

        if (!first) *out++ = ',';
        first = false;

        out += snprintf(out, (size_t)(end - out),
            "{\"timestamp\":%s,\"lat\":%s,\"lon\":%s,"
            "\"alt\":%s,\"speed\":%s,"
            "\"accel_x\":%s,\"accel_y\":%s,\"accel_z\":%s,"
            "\"device_mac\":\"%s\"}",
            f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8]);

        line = strtok_r(NULL, "\n", &save);
    }

    free(buf);
    if (first) { free(json); return NULL; }
    *out++ = ']'; *out = '\0';
    return json;
}

// ── HTTP helpers ─────────────────────────────────────────────────────────────

typedef struct { char *buf; size_t len; size_t cap; } resp_ctx_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    resp_ctx_t *ctx = (resp_ctx_t *)evt->user_data;
    if (!ctx) return ESP_OK;
    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data_len > 0) {
        size_t rem = ctx->cap - ctx->len - 1;
        size_t cp = (size_t)evt->data_len < rem ? (size_t)evt->data_len : rem;
        memcpy(ctx->buf + ctx->len, evt->data, cp);
        ctx->len += cp;
        ctx->buf[ctx->len] = '\0';
    } else if (evt->event_id == HTTP_EVENT_ON_CONNECTED) {
        ctx->len = 0;
        if (ctx->buf) ctx->buf[0] = '\0';
    }
    return ESP_OK;
}

static bool http_post_json(const char *label, const char *json)
{
    const char *url = config_get()->upload_url;
    const size_t RESP_BUF = 2048;
    char *resp = calloc(1, RESP_BUF);
    if (!resp) return false;

    resp_ctx_t ctx = { .buf = resp, .len = 0, .cap = RESP_BUF };
    bool is_https = (strncmp(url, "https://", 8) == 0);

    esp_http_client_config_t cfg = {
        .url               = url,
        .method            = HTTP_METHOD_POST,
        .timeout_ms        = 10000,
        .crt_bundle_attach = is_https ? esp_crt_bundle_attach : NULL,
        .event_handler     = http_event_handler,
        .user_data         = &ctx,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) { free(resp); return false; }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_post_field(client, json, (int)strlen(json));

    esp_err_t err = esp_http_client_perform(client);
    bool ok = false;
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status >= 200 && status < 300) {
            ESP_LOGI(TAG, "%s: HTTP %d  %s", label, status, resp);
            ok = true;
        } else {
            ESP_LOGW(TAG, "%s: HTTP %d: %s", label, status, resp);
        }
    } else {
        ESP_LOGE(TAG, "%s: %s", label, esp_err_to_name(err));
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(resp);
    vTaskDelay(pdMS_TO_TICKS(100));
    return ok;
}

// ── Paged upload ─────────────────────────────────────────────────────────────

static bool upload_staged_file(const char *label, const char *path)
{
    const size_t BUF = 32 * 1024;
    char *csv = malloc(BUF);
    if (!csv) return false;

    long offset = 0;
    int chunks = 0;

    while (true) {
        int len = sd_read_chunk(path, csv, BUF, &offset);
        if (len == 0) break;
        if (len < 0) { free(csv); return false; }

        char *json = csv_to_json(csv, len);
        if (!json) continue;

        ESP_LOGI(TAG, "%s: chunk %d (%d bytes)", label, chunks + 1, (int)strlen(json));
        bool ok = http_post_json(label, json);
        free(json);
        if (!ok) { free(csv); return false; }

        chunks++;
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    free(csv);
    return true;
}

// ── Per-file upload ──────────────────────────────────────────────────────────

static upload_result_t do_upload(const char *label,
    bool (*stage_fn)(void), bool (*ensure_fn)(void),
    const char *stage_path, const char *bak_path)
{
    if (!stage_fn()) return UPLOAD_NO_DATA;
    ensure_fn();
    bool ok = upload_staged_file(label, stage_path);
    if (ok) {
        sd_lock();
        FILE *src = fopen(stage_path, "r");
        FILE *dst = src ? fopen(bak_path, "a") : NULL;
        if (src && dst) {
            char io[512]; size_t n;
            while ((n = fread(io, 1, sizeof(io), src)) > 0) fwrite(io, 1, n, dst);
        }
        if (dst) fclose(dst);
        if (src) fclose(src);
        remove(stage_path);
        sd_unlock();
    }
    return ok ? UPLOAD_OK : UPLOAD_FAILED;
}

// ── Public API ───────────────────────────────────────────────────────────────

upload_result_t upload_own_csv(void)
{
    return do_upload("own", sd_stage_data_for_upload, sd_ensure_data_file,
                     CSV_UPLOAD_STAGE, CSV_DATA_BACKUP);
}

upload_result_t upload_merged_csv(void)
{
    return do_upload("peer", sd_stage_merged_for_upload, sd_ensure_merged_file,
                     CSV_MERGE_STAGE, CSV_MERGED_BACKUP);
}

bool upload_all_csv(upload_report_t *report)
{
    upload_report_t r = { .own_data = UPLOAD_NO_DATA, .merged_data = UPLOAD_NO_DATA };

    r.own_data = upload_own_csv();
    if (r.own_data == UPLOAD_FAILED) {
        if (report) *report = r;
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    r.merged_data = upload_merged_csv();
    if (report) *report = r;
    return r.merged_data != UPLOAD_FAILED;
}

bool upload_reupload_bak(int *out_count)
{
    int uploaded = 0;
    bool failed = false;

    DIR *d = opendir(SD_MOUNT_POINT);
    if (!d) { if (out_count) *out_count = 0; return false; }

    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        size_t len = strlen(ent->d_name);
        if (len <= 4 || strcmp(ent->d_name + len - 4, ".bak") != 0) continue;

        char full[272];
        snprintf(full, sizeof(full), SD_MOUNT_POINT "/%s", ent->d_name);
        struct stat st;
        if (stat(full, &st) != 0 || st.st_size == 0) continue;

        if (upload_staged_file(ent->d_name, full)) uploaded++;
        else failed = true;
    }
    closedir(d);

    if (out_count) *out_count = uploaded;
    return uploaded > 0 && !failed;
}

// ── OTA ──────────────────────────────────────────────────────────────────────

bool upload_ota_from_buffer(const uint8_t *data, size_t len)
{
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        ESP_LOGE(TAG, "No OTA partition available");
        return false;
    }

    esp_ota_handle_t handle;
    esp_err_t err = esp_ota_begin(update, len, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_ota_write(handle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));
        esp_ota_abort(handle);
        return false;
    }

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_ota_set_boot_partition(update);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA set boot partition failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "OTA update successful — reboot to activate");
    return true;
}
