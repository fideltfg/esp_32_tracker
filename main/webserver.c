// webserver.c — HTTP server: dashboard, live GPS map, config editor, OTA upload.

#include "webserver.h"
#include "config.h"
#include "types.h"
#include "gps.h"
#include "imu.h"
#include "power.h"
#include "upload.h"
#include "wifi_manager.h"
#include "sd_storage.h"
#include "espnow_sync.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <dirent.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "WEBSERVER";

// ── URL decode helper ────────────────────────────────────────────────────────

static void url_decode(char *str)
{
    char *s = str, *d = str;
    while (*s) {
        if (*s == '%' && s[1] && s[2]) {
            char hex[3] = { s[1], s[2], '\0' };
            *d++ = (char)strtol(hex, NULL, 16);
            s += 3;
        } else if (*s == '+') {
            *d++ = ' '; s++;
        } else {
            *d++ = *s++;
        }
    }
    *d = '\0';
}

// Shared state — set from outside via globals (written by gps_logging_task)
extern gps_data_t  g_current_gps;
extern imu_data_t  g_current_imu;
extern char        g_own_mac_str[5];

// ── Helpers ──────────────────────────────────────────────────────────────────

static void emit_file_row(httpd_req_t *req, const char *display, const char *path)
{
    struct stat st;
    if (stat(path, &st) != 0) return;
    char buf[640];
    snprintf(buf, sizeof(buf),
        "<tr><td><a href='/download?path=%s'>%s</a></td>"
        "<td style='padding-left:16px;color:#888'>%ld B</td></tr>",
        path, display, st.st_size);
    httpd_resp_sendstr_chunk(req, buf);
}

// ── GET / — Dashboard ────────────────────────────────────────────────────────

static esp_err_t handle_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req,
        "<html><head><title>GPS Tracker v2</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        "body{font-family:monospace;padding:16px;max-width:700px;margin:0 auto}"
        "h1{border-bottom:1px solid #ccc;padding-bottom:8px}"
        "h2{margin-top:24px;color:#444}table{border-collapse:collapse;width:100%}"
        "td{padding:4px 0}a{color:#0066cc}"
        ".btn{border:none;padding:8px 16px;font-family:monospace;font-size:14px;"
        "cursor:pointer;border-radius:3px;margin-right:8px;color:#fff}"
        ".action{background:#2980b9}.action:hover{background:#1a6fa0}"
        ".warn{background:#e67e22}.warn:hover{background:#ca6f1e}"
        ".danger{background:#c0392b}.danger:hover{background:#a93226}"
        "</style></head><body>");

    // Header + device info
    char header[256];
    snprintf(header, sizeof(header),
        "<h1>GPS Tracker v2</h1>"
        "<p style='color:#555;margin-top:0'>MAC: <b>%s</b> | "
        "<a href='/map'>Live Map</a> | "
        "<a href='/config'>Config</a> | "
        "<a href='/ota'>OTA Update</a></p>",
        g_own_mac_str[0] ? g_own_mac_str : "??");
    httpd_resp_sendstr_chunk(req, header);

    // Sync files
    httpd_resp_sendstr_chunk(req, "<h2>Sync Files</h2><table>");
    emit_file_row(req, "sync_data.csv", CSV_DATA_FILE);
    emit_file_row(req, "sync_merged.csv", CSV_MERGED_FILE);
    httpd_resp_sendstr_chunk(req, "</table>");

    // Backup files
    httpd_resp_sendstr_chunk(req, "<h2>Backup Files</h2><table>");
    DIR *d = opendir(SD_MOUNT_POINT);
    if (d) {
        struct dirent *ent;
        while ((ent = readdir(d)) != NULL) {
            size_t len = strlen(ent->d_name);
            if (len > 4 && strcmp(ent->d_name + len - 4, ".bak") == 0) {
                char full[272];
                snprintf(full, sizeof(full), SD_MOUNT_POINT "/%s", ent->d_name);
                emit_file_row(req, ent->d_name, full);
            }
        }
        closedir(d);
    }
    httpd_resp_sendstr_chunk(req, "</table>");

    // Connection log
    httpd_resp_sendstr_chunk(req, "<h2>Connection Log</h2><table>");
    emit_file_row(req, "connections.csv", CONNECTIONS_LOG_FILE);
    httpd_resp_sendstr_chunk(req, "</table>");

    // Debug log
    httpd_resp_sendstr_chunk(req, "<h2>Debug Log</h2><table>");
    emit_file_row(req, "debug.log (current)", DEBUG_LOG_FILE);
    emit_file_row(req, "debug_old.log (previous)", DEBUG_LOG_OLD_FILE);
    httpd_resp_sendstr_chunk(req, "</table>");

    // Actions
    httpd_resp_sendstr_chunk(req,
        "<h2>Actions</h2>"
        "<form method='POST' action='/reupload_bak' style='display:inline'>"
        "<button class='btn action'>Re-upload backups</button></form>"
        "<form method='POST' action='/clear_bak' style='display:inline' "
        "onsubmit=\"return confirm('Delete all .bak files?');\">"
        "<button class='btn warn'>Clear backups</button></form>"
        "<form method='POST' action='/clear' style='display:inline' "
        "onsubmit=\"return confirm('Clear ALL data files?');\">"
        "<button class='btn danger'>Clear all data</button></form>"
        "<form method='POST' action='/clear_log' style='display:inline;margin-left:8px' "
        "onsubmit=\"return confirm('Delete debug log files?');\">"
        "<button class='btn warn'>Clear debug log</button></form>"
        "<form method='POST' action='/reboot' style='display:inline;margin-left:16px' "
        "onsubmit=\"return confirm('Reboot the device?');\">"
        "<button class='btn warn'>Reboot</button></form>");

    httpd_resp_sendstr_chunk(req, "</body></html>");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// ── GET /download ────────────────────────────────────────────────────────────

static esp_err_t handle_download(httpd_req_t *req)
{
    char query[512], path[384];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "path", path, sizeof(path)) != ESP_OK) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    if (strstr(path, "..") != NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid path");
        return ESP_FAIL;
    }
    if (strncmp(path, SD_MOUNT_POINT, strlen(SD_MOUNT_POINT)) != 0) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Access denied");
        return ESP_FAIL;
    }

    FILE *f = fopen(path, "r");
    if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }

    httpd_resp_set_type(req, "text/csv");
    const char *bn = strrchr(path, '/');
    if (bn) {
        char cd[160];
        snprintf(cd, sizeof(cd), "attachment; filename=\"%s\"", bn + 1);
        httpd_resp_set_hdr(req, "Content-Disposition", cd);
    }

    char buf[512]; size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) break;
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// ── POST /clear ──────────────────────────────────────────────────────────────

static esp_err_t handle_clear(httpd_req_t *req)
{
    sd_delete_data_file();
    sd_ensure_data_file();
    sd_delete_merged_file();
    sd_ensure_merged_file();
    espnow_sync_reset_peer_states();
    remove(CONNECTIONS_LOG_FILE);

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_sendstr(req, "");
    return ESP_OK;
}

// ── POST /reupload_bak ──────────────────────────────────────────────────────

static esp_err_t handle_reupload_bak(httpd_req_t *req)
{
    if (!wifi_mgr_is_connected()) {
        if (!wifi_mgr_connect()) {
            httpd_resp_sendstr(req, "<html><body><p>WiFi failed.</p>"
                               "<a href='/'>Back</a></body></html>");
            return ESP_OK;
        }
    }

    int count = 0;
    bool ok = upload_reupload_bak(&count);
    char msg[256];
    snprintf(msg, sizeof(msg),
        "<html><body><p>%s %d file(s).</p><a href='/'>Back</a></body></html>",
        ok ? "Re-uploaded" : "Failed or partial:", count);
    httpd_resp_sendstr(req, msg);
    return ESP_OK;
}

// ── POST /clear_bak ─────────────────────────────────────────────────────────

static esp_err_t handle_clear_bak(httpd_req_t *req)
{
    int deleted = 0;
    DIR *d = opendir(SD_MOUNT_POINT);
    if (d) {
        struct dirent *ent;
        while ((ent = readdir(d)) != NULL) {
            size_t len = strlen(ent->d_name);
            if (len > 4 && strcmp(ent->d_name + len - 4, ".bak") == 0) {
                char full[272];
                snprintf(full, sizeof(full), SD_MOUNT_POINT "/%s", ent->d_name);
                if (remove(full) == 0) deleted++;
            }
        }
        closedir(d);
    }

    char msg[256];
    snprintf(msg, sizeof(msg),
        "<html><body><p>Deleted %d .bak file(s).</p>"
        "<a href='/'>Back</a></body></html>", deleted);
    httpd_resp_sendstr(req, msg);
    return ESP_OK;
}

// ── GET /api/gps — JSON GPS data ────────────────────────────────────────────

static esp_err_t handle_api_gps(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    char json[256];
    snprintf(json, sizeof(json),
        "{\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"speed\":%.1f,"
        "\"hdop\":%.1f,\"sats\":%d,\"valid\":%s,\"power_state\":%d}",
        g_current_gps.latitude, g_current_gps.longitude,
        g_current_gps.altitude, g_current_gps.speed,
        g_current_gps.hdop, g_current_gps.num_sv,
        g_current_gps.valid ? "true" : "false",
        (int)power_get_state());
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// ── GET /api/system — system info ────────────────────────────────────────────

static esp_err_t handle_api_system(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    const esp_app_desc_t *app = esp_app_get_description();
    char json[384];
    snprintf(json, sizeof(json),
        "{\"version\":\"%s\",\"idf\":\"%s\","
        "\"free_heap\":%lu,\"uptime_s\":%lld,"
        "\"power_state\":%d,\"mac\":\"%s\"}",
        app->version, app->idf_ver,
        (unsigned long)esp_get_free_heap_size(),
        esp_timer_get_time() / 1000000LL,
        (int)power_get_state(),
        g_own_mac_str);
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// ── GET /api/config — dump config as JSON ────────────────────────────────────

static esp_err_t handle_api_config_get(httpd_req_t *req)
{
    const tracker_config_t *cfg = config_get();
    httpd_resp_set_type(req, "application/json");

    char *json = malloc(2048);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    int off = 0;
    off += snprintf(json + off, 2048 - off, "{\"wifi_aps\":[");
    for (int i = 0; i < cfg->wifi_ap_count; i++) {
        if (i > 0) off += snprintf(json + off, 2048 - off, ",");
        off += snprintf(json + off, 2048 - off,
            "{\"ssid\":\"%s\",\"has_pass\":%s}",
            cfg->wifi_aps[i].ssid,
            cfg->wifi_aps[i].password[0] ? "true" : "false");
    }
    off += snprintf(json + off, 2048 - off,
        "],\"wifi_max_retry\":%d,"
        "\"upload_url\":\"%s\",\"timezone\":\"%s\","
        "\"gps_max_hdop\":%.1f,\"gps_min_sv\":%d,"
        "\"gps_outlier_m\":%.1f,\"gps_ema_alpha\":%.2f,"
        "\"gps_static_kmh\":%.1f,\"imu_accel_dev\":%.2f,\"imu_gyro_dps\":%.1f,"
        "\"log_moving_ms\":%lu,\"log_stage1_ms\":%lu,"
        "\"log_stage2_ms\":%lu,\"log_stage3_ms\":%lu,"
        "\"stage1_thresh_ms\":%lu,\"stage2_thresh_ms\":%lu,"
        "\"stage3_thresh_ms\":%lu,"
        "\"relock_stage1_ms\":%lu,\"relock_stage2_ms\":%lu,"
        "\"relock_stage3_ms\":%lu,"
        "\"lcd_timeout_sec\":%d,"
        "\"sync_search_max\":%d,\"sync_upload_interval_s\":%d}",
        cfg->wifi_max_retry,
        cfg->upload_url, cfg->timezone,
        cfg->gps_max_hdop, cfg->gps_min_sv,
        cfg->gps_outlier_m, cfg->gps_ema_alpha,
        cfg->gps_static_kmh, cfg->imu_accel_dev, cfg->imu_gyro_dps,
        (unsigned long)cfg->log_moving_ms, (unsigned long)cfg->log_stage1_ms,
        (unsigned long)cfg->log_stage2_ms, (unsigned long)cfg->log_stage3_ms,
        (unsigned long)cfg->stage1_thresh_ms, (unsigned long)cfg->stage2_thresh_ms,
        (unsigned long)cfg->stage3_thresh_ms,
        (unsigned long)cfg->relock_stage1_ms, (unsigned long)cfg->relock_stage2_ms,
        (unsigned long)cfg->relock_stage3_ms,
        cfg->lcd_timeout_sec,
        cfg->sync_search_max, cfg->sync_upload_interval_s);

    httpd_resp_sendstr(req, json);
    free(json);
    return ESP_OK;
}

// ── POST /api/config — update config ─────────────────────────────────────────
// Simple key=value parsing from form POST body (not full JSON parser)

static esp_err_t handle_api_config_post(httpd_req_t *req)
{
    int total = req->content_len;
    if (total <= 0 || total > 4096) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too large");
        return ESP_FAIL;
    }

    char *body = malloc(total + 1);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    int len = httpd_req_recv(req, body, total);
    if (len <= 0) {
        free(body);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Receive failed");
        return ESP_FAIL;
    }
    body[len] = '\0';

    // Start with a copy of the current config
    tracker_config_t cfg = *config_get();
    char val[256];

    // ── WiFi APs ─────────────────────────────────────────────────────────────
    const tracker_config_t *cur = config_get();
    uint8_t new_ap_count = 0;
    config_ap_entry_t new_aps[CONFIG_MAX_APS] = {0};

    for (int i = 0; i < CONFIG_MAX_APS; i++) {
        char key[20];
        snprintf(key, sizeof(key), "wifi_ssid_%d", i);
        if (httpd_query_key_value(body, key, val, sizeof(val)) == ESP_OK) {
            url_decode(val);
            if (val[0] == '\0') continue;   // empty SSID = skip slot

            strncpy(new_aps[new_ap_count].ssid, val, CONFIG_SSID_MAX - 1);
            new_aps[new_ap_count].ssid[CONFIG_SSID_MAX - 1] = '\0';

            // Password: if empty, preserve existing password for matching SSID
            snprintf(key, sizeof(key), "wifi_pass_%d", i);
            if (httpd_query_key_value(body, key, val, sizeof(val)) == ESP_OK && val[0]) {
                url_decode(val);
                strncpy(new_aps[new_ap_count].password, val, CONFIG_PASS_MAX - 1);
                new_aps[new_ap_count].password[CONFIG_PASS_MAX - 1] = '\0';
            } else {
                // Look for matching SSID in current config to preserve password
                for (int j = 0; j < cur->wifi_ap_count; j++) {
                    if (strcmp(new_aps[new_ap_count].ssid, cur->wifi_aps[j].ssid) == 0) {
                        memcpy(new_aps[new_ap_count].password,
                               cur->wifi_aps[j].password, CONFIG_PASS_MAX);
                        break;
                    }
                }
            }
            new_ap_count++;
        }
    }
    if (new_ap_count > 0) {
        memcpy(cfg.wifi_aps, new_aps, sizeof(new_aps));
        cfg.wifi_ap_count = new_ap_count;
    }

    // ── Scalar fields ────────────────────────────────────────────────────────
    if (httpd_query_key_value(body, "wifi_max_retry", val, sizeof(val)) == ESP_OK)
        cfg.wifi_max_retry = (uint8_t)atoi(val);

    if (httpd_query_key_value(body, "upload_url", val, sizeof(val)) == ESP_OK) {
        url_decode(val);
        strncpy(cfg.upload_url, val, CONFIG_URL_MAX - 1);
        cfg.upload_url[CONFIG_URL_MAX - 1] = '\0';
    }
    if (httpd_query_key_value(body, "timezone", val, sizeof(val)) == ESP_OK) {
        url_decode(val);
        strncpy(cfg.timezone, val, CONFIG_TZ_MAX - 1);
        cfg.timezone[CONFIG_TZ_MAX - 1] = '\0';
    }

    if (httpd_query_key_value(body, "gps_max_hdop", val, sizeof(val)) == ESP_OK)
        cfg.gps_max_hdop = strtof(val, NULL);
    if (httpd_query_key_value(body, "gps_min_sv", val, sizeof(val)) == ESP_OK)
        cfg.gps_min_sv = atoi(val);
    if (httpd_query_key_value(body, "gps_outlier_m", val, sizeof(val)) == ESP_OK)
        cfg.gps_outlier_m = strtof(val, NULL);
    if (httpd_query_key_value(body, "gps_ema_alpha", val, sizeof(val)) == ESP_OK)
        cfg.gps_ema_alpha = strtof(val, NULL);

    if (httpd_query_key_value(body, "gps_static_kmh", val, sizeof(val)) == ESP_OK)
        cfg.gps_static_kmh = strtof(val, NULL);
    if (httpd_query_key_value(body, "imu_accel_dev", val, sizeof(val)) == ESP_OK)
        cfg.imu_accel_dev = strtof(val, NULL);
    if (httpd_query_key_value(body, "imu_gyro_dps", val, sizeof(val)) == ESP_OK)
        cfg.imu_gyro_dps = strtof(val, NULL);

    if (httpd_query_key_value(body, "log_moving_ms", val, sizeof(val)) == ESP_OK)
        cfg.log_moving_ms = (uint32_t)strtoul(val, NULL, 10);
    if (httpd_query_key_value(body, "log_stage1_ms", val, sizeof(val)) == ESP_OK)
        cfg.log_stage1_ms = (uint32_t)strtoul(val, NULL, 10);
    if (httpd_query_key_value(body, "log_stage2_ms", val, sizeof(val)) == ESP_OK)
        cfg.log_stage2_ms = (uint32_t)strtoul(val, NULL, 10);
    if (httpd_query_key_value(body, "log_stage3_ms", val, sizeof(val)) == ESP_OK)
        cfg.log_stage3_ms = (uint32_t)strtoul(val, NULL, 10);

    if (httpd_query_key_value(body, "stage1_thresh_ms", val, sizeof(val)) == ESP_OK)
        cfg.stage1_thresh_ms = (uint32_t)strtoul(val, NULL, 10);
    if (httpd_query_key_value(body, "stage2_thresh_ms", val, sizeof(val)) == ESP_OK)
        cfg.stage2_thresh_ms = (uint32_t)strtoul(val, NULL, 10);
    if (httpd_query_key_value(body, "stage3_thresh_ms", val, sizeof(val)) == ESP_OK)
        cfg.stage3_thresh_ms = (uint32_t)strtoul(val, NULL, 10);

    if (httpd_query_key_value(body, "relock_stage1_ms", val, sizeof(val)) == ESP_OK)
        cfg.relock_stage1_ms = (uint32_t)strtoul(val, NULL, 10);
    if (httpd_query_key_value(body, "relock_stage2_ms", val, sizeof(val)) == ESP_OK)
        cfg.relock_stage2_ms = (uint32_t)strtoul(val, NULL, 10);
    if (httpd_query_key_value(body, "relock_stage3_ms", val, sizeof(val)) == ESP_OK)
        cfg.relock_stage3_ms = (uint32_t)strtoul(val, NULL, 10);

    if (httpd_query_key_value(body, "lcd_timeout_sec", val, sizeof(val)) == ESP_OK)
        cfg.lcd_timeout_sec = (uint16_t)strtoul(val, NULL, 10);

    if (httpd_query_key_value(body, "sync_search_max", val, sizeof(val)) == ESP_OK)
        cfg.sync_search_max = (uint16_t)strtoul(val, NULL, 10);
    if (httpd_query_key_value(body, "sync_upload_interval_s", val, sizeof(val)) == ESP_OK)
        cfg.sync_upload_interval_s = (uint16_t)strtoul(val, NULL, 10);

    config_update(&cfg);
    free(body);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

// ── GET /map — Live GPS map ──────────────────────────────────────────────────

static esp_err_t handle_map(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req,
        "<!DOCTYPE html><html><head><title>GPS Map</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'/>"
        "<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>"
        "<style>#map{height:100vh;width:100%}body{margin:0}"
        "#info{position:absolute;top:10px;right:10px;z-index:1000;"
        "background:rgba(255,255,255,0.9);padding:10px;border-radius:5px;"
        "font-family:monospace;font-size:13px}</style></head><body>"
        "<div id='info'>Loading GPS...</div>"
        "<div id='map'></div>"
        "<script>"
        "var map=L.map('map').setView([0,0],2);"
        "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{"
        "attribution:'&copy; OpenStreetMap'}).addTo(map);"
        "var marker=null,first=true;"
        "function update(){"
        "fetch('/api/gps').then(r=>r.json()).then(d=>{"
        "if(!d.valid)return;"
        "var ll=[d.lat,d.lon];"
        "if(!marker){marker=L.circleMarker(ll,{radius:8,color:'#e74c3c',fillOpacity:0.8}).addTo(map);"
        "}else{marker.setLatLng(ll);}"
        "if(first){map.setView(ll,16);first=false;}"
        "document.getElementById('info').innerHTML="
        "'<b>'+d.lat.toFixed(6)+', '+d.lon.toFixed(6)+'</b><br>'"
        "+'Speed: '+d.speed.toFixed(1)+' km/h<br>'"
        "+'Alt: '+d.alt.toFixed(0)+' m<br>'"
        "+'Sats: '+d.sats+' HDOP: '+d.hdop.toFixed(1);"
        "}).catch(()=>{});}"
        "setInterval(update,2000);update();"
        "</script></body></html>");
    return ESP_OK;
}

// ── GET /config — Config editor page ─────────────────────────────────────────

static esp_err_t handle_config_page(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    httpd_resp_sendstr_chunk(req,
        "<!DOCTYPE html><html><head><title>Config</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        "body{font-family:monospace;padding:16px;max-width:700px;margin:0 auto}"
        "h2{border-bottom:1px solid #ccc;padding-bottom:4px;margin-top:24px}"
        "label{display:block;margin:8px 0 2px;font-size:13px;color:#555}"
        "input{width:100%;padding:6px;font-family:monospace;box-sizing:border-box}"
        ".row{display:flex;gap:8px}.row input{flex:1}"
        ".btn{background:#2980b9;color:#fff;border:none;padding:10px 20px;"
        "font-family:monospace;cursor:pointer;border-radius:3px;margin-top:16px}"
        ".btn-danger{background:#c0392b}"
        "#msg{margin-top:8px;font-weight:bold}"
        "</style></head><body>"
        "<h1>Configuration</h1>"
        "<p><a href='/'>&#8592; Dashboard</a></p>"
        "<div id='form'>Loading...</div>"
        "<div id='msg'></div>");

    httpd_resp_sendstr_chunk(req,
        "<script>"
        "function F(n,l,v,t){t=t||'text';"
        "return '<label>'+l+'</label>"
        "<input name=\"'+n+'\" type=\"'+t+'\" value=\"'+(v!=null?v:'')+'\" step=\"any\">';}"
        "function N(n,l,v){return F(n,l,v,'number');}"
        "fetch('/api/config').then(r=>r.json()).then(function(c){"
        "var h='<form id=\"cf\">';"

        /* ── WiFi ── */
        "h+='<h2>WiFi Networks</h2>';"
        "for(var i=0;i<4;i++){"
        "var s=c.wifi_aps&&c.wifi_aps[i]?c.wifi_aps[i]:{ssid:\"\",has_pass:false};"
        "h+='<div class=\"row\">'+"
        "'<input name=\"wifi_ssid_'+i+'\" placeholder=\"SSID '+(i+1)+'\" value=\"'+s.ssid+'\">'+"
        "'<input name=\"wifi_pass_'+i+'\" type=\"password\" placeholder=\"'+"
        "(s.has_pass?'(unchanged)':'Password')+'\">'+"
        "'</div>';}"
        "h+=N('wifi_max_retry','Max Retry',c.wifi_max_retry);"

        /* ── Upload ── */
        "h+='<h2>Upload</h2>';"
        "h+=F('upload_url','Upload URL',c.upload_url);"

        /* ── Timezone ── */
        "h+='<h2>Timezone</h2>';"
        "h+=F('timezone','POSIX TZ String',c.timezone);");

    httpd_resp_sendstr_chunk(req,
        /* ── GPS Quality ── */
        "h+='<h2>GPS Quality Filters</h2>';"
        "h+=N('gps_max_hdop','Max HDOP',c.gps_max_hdop);"
        "h+=N('gps_min_sv','Min Satellites',c.gps_min_sv);"
        "h+=N('gps_outlier_m','Outlier Distance (m)',c.gps_outlier_m);"
        "h+=N('gps_ema_alpha','EMA Alpha',c.gps_ema_alpha);"

        /* ── Motion Detection ── */
        "h+='<h2>Motion Detection</h2>';"
        "h+=N('gps_static_kmh','Static Speed (km/h)',c.gps_static_kmh);"
        "h+=N('imu_accel_dev','Accel Deviation (g)',c.imu_accel_dev);"
        "h+=N('imu_gyro_dps','Gyro Threshold (deg/s)',c.imu_gyro_dps);"

        /* ── Logging Intervals ── */
        "h+='<h2>Logging Intervals (ms)</h2>';"
        "h+=N('log_moving_ms','Moving',c.log_moving_ms);"
        "h+=N('log_stage1_ms','Stage 1',c.log_stage1_ms);"
        "h+=N('log_stage2_ms','Stage 2',c.log_stage2_ms);"
        "h+=N('log_stage3_ms','Stage 3',c.log_stage3_ms);");

    httpd_resp_sendstr_chunk(req,
        /* ── Power Stage Thresholds ── */
        "h+='<h2>Power Stage Thresholds (ms)</h2>';"
        "h+=N('stage1_thresh_ms','Stage 1 Entry',c.stage1_thresh_ms);"
        "h+=N('stage2_thresh_ms','Stage 2 Entry',c.stage2_thresh_ms);"
        "h+=N('stage3_thresh_ms','Stage 3 Entry',c.stage3_thresh_ms);"

        /* ── GPS Re-lock Budgets ── */
        "h+='<h2>GPS Re-lock Budgets (ms)</h2>';"
        "h+=N('relock_stage1_ms','Stage 1',c.relock_stage1_ms);"
        "h+=N('relock_stage2_ms','Stage 2',c.relock_stage2_ms);"
        "h+=N('relock_stage3_ms','Stage 3',c.relock_stage3_ms);"

        /* ── Display ── */
        "h+='<h2>Display</h2>';"
        "h+=N('lcd_timeout_sec','LCD Timeout (sec)',c.lcd_timeout_sec);"

        /* ── Sync ── */
        "h+='<h2>Sync State Machine</h2>';"
        "h+=N('sync_search_max','Search Iterations Max',c.sync_search_max);"
        "h+=N('sync_upload_interval_s','Upload Interval (sec)',c.sync_upload_interval_s);");

    httpd_resp_sendstr_chunk(req,
        /* ── Buttons + submit handler ── */
        "h+='<br><button type=\"submit\" class=\"btn\">Save</button>';"
        "h+=' <button type=\"button\" class=\"btn btn-danger\" onclick=\"resetCfg()\">Factory Reset</button>';"
        "h+='</form>';"
        "document.getElementById('form').innerHTML=h;"
        "document.getElementById('cf').onsubmit=function(e){"
        "e.preventDefault();"
        "var fd=new URLSearchParams(new FormData(this));"
        "fetch('/api/config',{method:'POST',body:fd}).then(r=>r.json()).then(function(d){"
        "var m=document.getElementById('msg');"
        "m.textContent=d.status==='ok'?'Saved! Some changes need reboot.':'Error: '+JSON.stringify(d);"
        "m.style.color=d.status==='ok'?'green':'red';"
        "});};});"

        "function resetCfg(){"
        "if(!confirm('Reset ALL settings to factory defaults?'))return;"
        "fetch('/api/reset_config',{method:'POST'}).then(r=>r.json()).then(function(){"
        "document.getElementById('msg').textContent='Reset to defaults.';"
        "document.getElementById('msg').style.color='orange';"
        "setTimeout(function(){location.reload();},1000);"
        "});}"
        "</script></body></html>");

    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// ── GET /ota — OTA upload page ───────────────────────────────────────────────

static esp_err_t handle_ota_page(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    const esp_app_desc_t *app = esp_app_get_description();

    httpd_resp_sendstr_chunk(req,
        "<!DOCTYPE html><html><head><title>OTA Update</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>body{font-family:monospace;padding:16px;max-width:600px;margin:0 auto}"
        ".btn{background:#2980b9;color:#fff;border:none;padding:10px 20px;"
        "font-family:monospace;cursor:pointer;border-radius:3px;margin-top:16px}"
        "#status{margin-top:16px;font-weight:bold}</style></head><body>"
        "<h1>OTA Firmware Update</h1>"
        "<p><a href='/'>&#8592; Dashboard</a></p>");

    char ver_line[128];
    snprintf(ver_line, sizeof(ver_line),
        "<p>Current version: <b>%s</b></p>", app->version);
    httpd_resp_sendstr_chunk(req, ver_line);

    httpd_resp_sendstr_chunk(req,
        "<form id='uf' enctype='multipart/form-data'>"
        "<input type='file' name='firmware' accept='.bin' required>"
        "<br><button type='submit' class='btn'>Upload &amp; Flash</button>"
        "</form><div id='status'></div>"
        "<script>"
        "document.getElementById('uf').onsubmit=function(e){"
        "e.preventDefault();"
        "var f=this.querySelector('input[type=file]').files[0];"
        "if(!f)return;"
        "document.getElementById('status').textContent='Uploading...';"
        "var xhr=new XMLHttpRequest();"
        "xhr.open('POST','/ota');"
        "xhr.onload=function(){"
        "document.getElementById('status').textContent="
        "xhr.status===200?'Success! Rebooting...':'Error: '+xhr.responseText;};"
        "xhr.send(f);};"
        "</script></body></html>");

    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// ── POST /ota — Receive firmware binary ──────────────────────────────────────

static esp_err_t handle_ota_upload(httpd_req_t *req)
{
    int total = req->content_len;
    if (total <= 0 || total > 1536 * 1024) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid firmware size");
        return ESP_FAIL;
    }

    uint8_t *fw = malloc(total);
    if (!fw) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    int received = 0;
    while (received < total) {
        int ret = httpd_req_recv(req, (char *)fw + received, total - received);
        if (ret <= 0) {
            free(fw);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }
        received += ret;
    }

    bool ok = upload_ota_from_buffer(fw, total);
    free(fw);

    if (ok) {
        httpd_resp_sendstr(req, "OTA OK — rebooting");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA failed");
    }
    return ESP_OK;
}

// ── POST /reboot — Reboot device ────────────────────────────────────────────

static esp_err_t handle_reboot(httpd_req_t *req)
{
    httpd_resp_sendstr(req, "<html><body><p>Rebooting...</p>"
                       "<a href='/'>Back</a></body></html>");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// ── GET /status — JSON status ────────────────────────────────────────────────

static esp_err_t handle_status(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

// ── POST /api/reset_config ───────────────────────────────────────────────────

static esp_err_t handle_reset_config(httpd_req_t *req)
{
    config_reset_defaults();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"reset\"}");
    return ESP_OK;
}

// ── POST /api/test_sleep — Force deep sleep immediately (test only) ──────────
// Allows manual testing of the deep sleep wake sources without waiting for
// the power state machine to reach Stage 3 naturally.
// Wake: connect CHARGER_DET_GPIO (or IMU_INT_GPIO) to 3.3 V, or press BOOT.

#if CHARGER_DET_GPIO > 0 || IMU_INT_GPIO > 0
static esp_err_t handle_test_sleep(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"entering deep sleep\"}");
    vTaskDelay(pdMS_TO_TICKS(200));   // let the response flush before sleeping
    power_enter_deep_sleep();
    return ESP_OK;   // unreachable — device sleeps above
}
#endif

// ── POST /clear_log — Delete debug log files ─────────────────────────────────

static esp_err_t handle_clear_log(httpd_req_t *req)
{
    remove(DEBUG_LOG_FILE);
    remove(DEBUG_LOG_OLD_FILE);

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_sendstr(req, "");
    return ESP_OK;
}

// ── Server start / stop ──────────────────────────────────────────────────────

httpd_handle_t webserver_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 19;
    config.stack_size = 8192;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    // Dashboard + file ops
    httpd_uri_t uris[] = {
        { "/",              HTTP_GET,  handle_root,          NULL },
        { "/download",      HTTP_GET,  handle_download,      NULL },
        { "/status",        HTTP_GET,  handle_status,        NULL },
        { "/clear",         HTTP_POST, handle_clear,         NULL },
        { "/reupload_bak",  HTTP_POST, handle_reupload_bak,  NULL },
        { "/clear_bak",     HTTP_POST, handle_clear_bak,     NULL },
        // API
        { "/api/gps",       HTTP_GET,  handle_api_gps,       NULL },
        { "/api/system",    HTTP_GET,  handle_api_system,    NULL },
        { "/api/config",    HTTP_GET,  handle_api_config_get, NULL },
        { "/api/config",    HTTP_POST, handle_api_config_post, NULL },
        { "/api/reset_config", HTTP_POST, handle_reset_config, NULL },
        { "/reboot",          HTTP_POST, handle_reboot,        NULL },
        { "/clear_log",       HTTP_POST, handle_clear_log,     NULL },
#if CHARGER_DET_GPIO > 0 || IMU_INT_GPIO > 0
        { "/api/test_sleep",  HTTP_POST, handle_test_sleep,    NULL },
#endif
        // Pages
        { "/map",           HTTP_GET,  handle_map,           NULL },
        { "/config",        HTTP_GET,  handle_config_page,   NULL },
        { "/ota",           HTTP_GET,  handle_ota_page,      NULL },
        { "/ota",           HTTP_POST, handle_ota_upload,    NULL },
    };

    for (int i = 0; i < (int)(sizeof(uris) / sizeof(uris[0])); i++)
        httpd_register_uri_handler(server, &uris[i]);

    ESP_LOGI(TAG, "HTTP server started (%d handlers)", (int)(sizeof(uris)/sizeof(uris[0])));
    return server;
}

void webserver_stop(httpd_handle_t server)
{
    if (server) httpd_stop(server);
}
