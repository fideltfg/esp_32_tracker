// config.c — NVS-backed runtime configuration.

#include <string.h>
#include "config.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "CONFIG";
#define NVS_NAMESPACE "tracker_cfg"

static tracker_config_t s_config;

// ── Defaults ─────────────────────────────────────────────────────────────────

static void apply_defaults(tracker_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    // WiFi APs
    uint8_t n = DEFAULT_WIFI_AP_COUNT;
    if (n > CONFIG_MAX_APS) n = CONFIG_MAX_APS;
    for (uint8_t i = 0; i < n; i++) {
        strncpy(cfg->wifi_aps[i].ssid,     DEFAULT_WIFI_APS[i].ssid,     CONFIG_SSID_MAX - 1);
        strncpy(cfg->wifi_aps[i].password, DEFAULT_WIFI_APS[i].password, CONFIG_PASS_MAX - 1);
    }
    cfg->wifi_ap_count  = n;
    cfg->wifi_max_retry = DEFAULT_WIFI_MAX_RETRY;

    // Upload
    strncpy(cfg->upload_url, DEFAULT_UPLOAD_URL, CONFIG_URL_MAX - 1);

    // Timezone
    strncpy(cfg->timezone, DEFAULT_TIMEZONE, CONFIG_TZ_MAX - 1);

    // GPS quality
    cfg->gps_max_hdop  = DEFAULT_GPS_MAX_HDOP;
    cfg->gps_min_sv    = DEFAULT_GPS_MIN_SV;
    cfg->gps_outlier_m = DEFAULT_GPS_OUTLIER_M;
    cfg->gps_ema_alpha = DEFAULT_GPS_EMA_ALPHA;

    // Motion detection
    cfg->gps_static_kmh = DEFAULT_GPS_STATIC_KMH;
    cfg->imu_accel_dev  = DEFAULT_IMU_ACCEL_DEV;
    cfg->imu_gyro_dps   = DEFAULT_IMU_GYRO_DPS;

    // Logging intervals
    cfg->log_moving_ms = DEFAULT_LOG_MOVING_MS;
    cfg->log_stage1_ms = DEFAULT_LOG_STAGE1_MS;
    cfg->log_stage2_ms = DEFAULT_LOG_STAGE2_MS;
    cfg->log_stage3_ms = DEFAULT_LOG_STAGE3_MS;

    // Stationary thresholds
    cfg->stage1_thresh_ms = DEFAULT_STAGE1_THRESH_MS;
    cfg->stage2_thresh_ms = DEFAULT_STAGE2_THRESH_MS;
    cfg->stage3_thresh_ms = DEFAULT_STAGE3_THRESH_MS;

    // Re-lock budgets
    cfg->relock_stage1_ms = DEFAULT_RELOCK_STAGE1_MS;
    cfg->relock_stage2_ms = DEFAULT_RELOCK_STAGE2_MS;
    cfg->relock_stage3_ms = DEFAULT_RELOCK_STAGE3_MS;

    // LCD
    cfg->lcd_timeout_sec = DEFAULT_LCD_TIMEOUT_SEC;

    // Sync
    cfg->sync_search_max       = DEFAULT_SYNC_SEARCH_MAX;
    cfg->sync_upload_interval_s = DEFAULT_SYNC_UPLOAD_INTERVAL;
}

// ── NVS load / save ──────────────────────────────────────────────────────────

static bool load_from_nvs(tracker_config_t *cfg)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
        return false;

    size_t len = sizeof(tracker_config_t);
    esp_err_t err = nvs_get_blob(h, "cfg", cfg, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof(tracker_config_t)) {
        ESP_LOGW(TAG, "NVS config blob missing or size mismatch — using defaults");
        return false;
    }
    return true;
}

static bool save_to_nvs(const tracker_config_t *cfg)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing");
        return false;
    }

    esp_err_t err = nvs_set_blob(h, "cfg", cfg, sizeof(tracker_config_t));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save config to NVS: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Config saved to NVS");
    return true;
}

// ── Public API ───────────────────────────────────────────────────────────────

void config_init(void)
{
    if (!load_from_nvs(&s_config)) {
        ESP_LOGI(TAG, "Loading defaults (first boot or factory reset)");
        apply_defaults(&s_config);
        save_to_nvs(&s_config);
    } else {
        ESP_LOGI(TAG, "Config loaded from NVS");
    }

    ESP_LOGI(TAG, "WiFi APs configured: %d", s_config.wifi_ap_count);
    ESP_LOGI(TAG, "Upload URL: %s", s_config.upload_url);
    ESP_LOGI(TAG, "Timezone: %s", s_config.timezone);
}

const tracker_config_t *config_get(void)
{
    return &s_config;
}

bool config_save(void)
{
    return save_to_nvs(&s_config);
}

void config_reset_defaults(void)
{
    apply_defaults(&s_config);
    save_to_nvs(&s_config);
    ESP_LOGW(TAG, "Config reset to factory defaults");
}

bool config_set_wifi_ap(uint8_t index, const char *ssid, const char *password)
{
    if (index >= CONFIG_MAX_APS || !ssid) return false;

    strncpy(s_config.wifi_aps[index].ssid, ssid, CONFIG_SSID_MAX - 1);
    s_config.wifi_aps[index].ssid[CONFIG_SSID_MAX - 1] = '\0';

    if (password) {
        strncpy(s_config.wifi_aps[index].password, password, CONFIG_PASS_MAX - 1);
        s_config.wifi_aps[index].password[CONFIG_PASS_MAX - 1] = '\0';
    } else {
        s_config.wifi_aps[index].password[0] = '\0';
    }

    if (index >= s_config.wifi_ap_count)
        s_config.wifi_ap_count = index + 1;

    return save_to_nvs(&s_config);
}

bool config_set_upload_url(const char *url)
{
    if (!url) return false;
    strncpy(s_config.upload_url, url, CONFIG_URL_MAX - 1);
    s_config.upload_url[CONFIG_URL_MAX - 1] = '\0';
    return save_to_nvs(&s_config);
}

bool config_set_timezone(const char *tz)
{
    if (!tz) return false;
    strncpy(s_config.timezone, tz, CONFIG_TZ_MAX - 1);
    s_config.timezone[CONFIG_TZ_MAX - 1] = '\0';
    return save_to_nvs(&s_config);
}

bool config_update(const tracker_config_t *new_cfg)
{
    if (!new_cfg) return false;
    if (new_cfg->wifi_ap_count > CONFIG_MAX_APS) return false;

    memcpy(&s_config, new_cfg, sizeof(tracker_config_t));
    // Ensure string null-termination
    for (int i = 0; i < CONFIG_MAX_APS; i++) {
        s_config.wifi_aps[i].ssid[CONFIG_SSID_MAX - 1] = '\0';
        s_config.wifi_aps[i].password[CONFIG_PASS_MAX - 1] = '\0';
    }
    s_config.upload_url[CONFIG_URL_MAX - 1] = '\0';
    s_config.timezone[CONFIG_TZ_MAX - 1] = '\0';

    return save_to_nvs(&s_config);
}
