#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// config.h — All configuration for the GPS tracker.
//
// Copy this file to config.h and fill in your WiFi credentials and upload
// endpoint.  Do not commit config.h to source control.
//
// This file contains:
//   1. Hardware pin assignments (compile-time, cannot change at runtime)
//   2. Protocol constants and file paths (compile-time)
//   3. Compile-time defaults for NVS-backed settings (first boot / factory reset)
//   4. Runtime config structure and API (NVS-backed, editable via web UI)
// ─────────────────────────────────────────────────────────────────────────────

#include <stdbool.h>
#include <stdint.h>
#include "types.h"

// ═════════════════════════════════════════════════════════════════════════════
// SECTION 1 — Hardware Pin Assignments (compile-time only)
// ═════════════════════════════════════════════════════════════════════════════

// ─── GPS UART ────────────────────────────────────────────────────────────────
#define GPS_UART_NUM     UART_NUM_1
#define GPS_TX_PIN       23
#define GPS_RX_PIN       22
#define GPS_BAUD_RATE    9600
#define UART_BUF_SIZE    1024
#define GPS_SAMPLE_PERIOD_MS  200   // 5 Hz (must match CFG-RATE measRate)

// ─── I2C Bus 0 — RTC + IMU ───────────────────────────────────────────────────
#define I2C_MASTER_NUM   I2C_NUM_0
#define I2C_SDA_PIN      18
#define I2C_SCL_PIN      19
#define I2C_FREQ_HZ      100000

// ─── I2C Bus 1 — LCD ─────────────────────────────────────────────────────────
#define LCD_I2C_NUM      I2C_NUM_1
#define LCD_I2C_SDA_PIN  32
#define LCD_I2C_SCL_PIN  33
#define LCD_I2C_FREQ_HZ  100000

// ─── I2C device addresses ────────────────────────────────────────────────────
#define RTC_I2C_ADDR     0x68
#define MPU6050_I2C_ADDR 0x69   // AD0 HIGH; use 0x68 if AD0 LOW
#define LCD_I2C_ADDR     0x27   // Try 0x3F if not found

// ─── SD card pins ────────────────────────────────────────────────────────────
#define SDMMC_CMD_GPIO   15
#define SDMMC_CLK_GPIO   14
#define SDMMC_D0_GPIO    2
#define SD_MOUNT_POINT   "/sdcard"

// ─── Button ──────────────────────────────────────────────────────────────────
#define BUTTON_GPIO      0

// ─── LCD feature flags ───────────────────────────────────────────────────────
#define LCD_ENABLED      1

// ─── IMU wake-on-motion (deep sleep) ─────────────────────────────────────────
// MPU6050 INT pin must be wired to an RTC-capable GPIO for ext0 wakeup.
// Set to 0 if not wired — deep sleep will fall back to timer-based wake.
#define IMU_INT_GPIO     0

// ═════════════════════════════════════════════════════════════════════════════
// SECTION 2 — Protocol Constants & File Paths (compile-time only)
// ═════════════════════════════════════════════════════════════════════════════

// ─── Sync CSV file paths ─────────────────────────────────────────────────────
#define CSV_DATA_FILE        SD_MOUNT_POINT "/sync_data.csv"
#define CSV_MERGED_FILE      SD_MOUNT_POINT "/sync_merged.csv"
#define CSV_MERGE_TMP        SD_MOUNT_POINT "/sync_tmp.csv"
#define CSV_DATA_BACKUP      SD_MOUNT_POINT "/sync_data.bak"
#define CSV_DATA_BAK_FMT     SD_MOUNT_POINT "/sync_data.%d.bak"
#define CSV_MERGED_BACKUP    SD_MOUNT_POINT "/sync_merged.bak"
#define CSV_MERGED_BAK_FMT   SD_MOUNT_POINT "/sync_merged.%d.bak"
#define CSV_UPLOAD_STAGE     SD_MOUNT_POINT "/sync_upload.csv"
#define CSV_MERGE_STAGE      SD_MOUNT_POINT "/sync_merge_stage.csv"
#define CONNECTIONS_LOG_FILE SD_MOUNT_POINT "/connections.csv"

#define CSV_HEADER  "timestamp,lat,lon,alt,speed,accel_x,accel_y,accel_z,device_mac\n"

// ─── ESP-NOW protocol ────────────────────────────────────────────────────────
#define ESPNOW_BROADCAST_MAC    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
#define ESPNOW_CHANNEL          1
#define ESPNOW_RX_BUF_RECORDS   200
#define ESPNOW_RECV_TIMEOUT_MS  8000
#define ESPNOW_DISCOVER_MS      30000

// ─── Test mode (disabled in production) ──────────────────────────────────────
#define TEST_MODE              0
#define TEST_DEVICE_IS_STATIC  0

// ═════════════════════════════════════════════════════════════════════════════
// SECTION 3 — Compile-time Defaults for NVS-backed Config
// ═════════════════════════════════════════════════════════════════════════════
// Used on first boot or after factory reset.  Once saved to NVS the runtime
// values take precedence and can be changed via the web UI at /config.

// WiFi AP list — add/remove rows as needed (up to CONFIG_MAX_APS).
static const wifi_ap_entry_t DEFAULT_WIFI_APS[] = {
    { "<YOUR_SSID_1>", "<YOUR_PASSWORD_1>" },
    { "<YOUR_SSID_2>", "<YOUR_PASSWORD_2>" },
};
#define DEFAULT_WIFI_AP_COUNT  (sizeof(DEFAULT_WIFI_APS) / sizeof(DEFAULT_WIFI_APS[0]))
#define DEFAULT_WIFI_MAX_RETRY    3

// Upload
#define DEFAULT_UPLOAD_URL        "https://<YOUR_SERVER>/api/esp/upload"

// Timezone (POSIX string)
// Common values:
//   "EST5EDT,M3.2.0/2,M11.1.0"           US Eastern
//   "CST6CDT,M3.2.0,M11.1.0"             US Central
//   "MST7MDT,M3.2.0,M11.1.0"             US Mountain
//   "PST8PDT,M3.2.0,M11.1.0"             US Pacific
//   "GMT0BST,M3.5.0/1,M10.5.0"           UK
//   "CET-1CEST,M3.5.0,M10.5.0/3"         Central Europe
//   "AEST-10AEDT,M10.1.0,M4.1.0/3"       Australia Eastern
// Overridden at runtime by GPS-derived timezone on first valid fix.
#define DEFAULT_TIMEZONE          "MST7MDT,M3.2.0,M11.1.0"

// GPS quality filters
#define DEFAULT_GPS_MAX_HDOP      2.5f
#define DEFAULT_GPS_MIN_SV        4
#define DEFAULT_GPS_OUTLIER_M     50.0f
#define DEFAULT_GPS_EMA_ALPHA     0.3f

// GPS NEO-6M UBX tuning
#define DEFAULT_GPS_STATIC_HOLD   10         // cm/s
#define DEFAULT_GPS_SBAS_MASK     0x00000000U

// Motion detection
#define DEFAULT_GPS_STATIC_KMH    2.0f
#define DEFAULT_IMU_ACCEL_DEV     0.15f
#define DEFAULT_IMU_GYRO_DPS      5.0f

// Logging intervals (ms)
#define DEFAULT_LOG_MOVING_MS     1000
#define DEFAULT_LOG_STAGE1_MS     60000
#define DEFAULT_LOG_STAGE2_MS     300000
#define DEFAULT_LOG_STAGE3_MS     1800000

// Stationary thresholds (ms)
#define DEFAULT_STAGE1_THRESH_MS  180000
#define DEFAULT_STAGE2_THRESH_MS  300000
#define DEFAULT_STAGE3_THRESH_MS  1800000

// GPS re-lock budgets (ms)
#define DEFAULT_RELOCK_STAGE1_MS  30000
#define DEFAULT_RELOCK_STAGE2_MS  120000
#define DEFAULT_RELOCK_STAGE3_MS  300000

// LCD
#define DEFAULT_LCD_TIMEOUT_SEC   30

// Sync state machine
#define DEFAULT_SYNC_SEARCH_MAX       300
#define DEFAULT_SYNC_UPLOAD_INTERVAL  30

// ═════════════════════════════════════════════════════════════════════════════
// SECTION 4 — Runtime Config Structure & API
// ═════════════════════════════════════════════════════════════════════════════

#define CONFIG_MAX_APS       4
#define CONFIG_SSID_MAX      33
#define CONFIG_PASS_MAX      65
#define CONFIG_URL_MAX       128
#define CONFIG_TZ_MAX        48

// WiFi AP entry stored in NVS
typedef struct {
    char ssid[CONFIG_SSID_MAX];
    char password[CONFIG_PASS_MAX];
} config_ap_entry_t;

// Complete runtime configuration
typedef struct {
    // WiFi
    config_ap_entry_t wifi_aps[CONFIG_MAX_APS];
    uint8_t           wifi_ap_count;
    uint8_t           wifi_max_retry;

    // Upload
    char upload_url[CONFIG_URL_MAX];

    // Timezone
    char timezone[CONFIG_TZ_MAX];

    // GPS quality filters
    float    gps_max_hdop;
    int      gps_min_sv;
    float    gps_outlier_m;
    float    gps_ema_alpha;

    // Motion detection
    float    gps_static_kmh;
    float    imu_accel_dev;
    float    imu_gyro_dps;

    // Logging intervals (ms)
    uint32_t log_moving_ms;
    uint32_t log_stage1_ms;
    uint32_t log_stage2_ms;
    uint32_t log_stage3_ms;

    // Stationary thresholds (ms)
    uint32_t stage1_thresh_ms;
    uint32_t stage2_thresh_ms;
    uint32_t stage3_thresh_ms;

    // GPS re-lock budgets (ms)
    uint32_t relock_stage1_ms;
    uint32_t relock_stage2_ms;
    uint32_t relock_stage3_ms;

    // LCD
    uint16_t lcd_timeout_sec;

    // Sync state machine
    uint16_t sync_search_max;
    uint16_t sync_upload_interval_s;
} tracker_config_t;

/**
 * @brief Initialise the config module. Loads from NVS or writes defaults.
 *        Must be called after nvs_flash_init().
 */
void config_init(void);

/**
 * @brief Get a read-only pointer to the current config.
 *        Thread-safe — the returned pointer is valid for the lifetime of the app.
 */
const tracker_config_t *config_get(void);

/**
 * @brief Save the current config to NVS.
 * @return true on success.
 */
bool config_save(void);

/**
 * @brief Reset config to compile-time defaults and save to NVS.
 */
void config_reset_defaults(void);

/**
 * @brief Update a WiFi AP entry. Index must be < CONFIG_MAX_APS.
 * @return true if saved successfully.
 */
bool config_set_wifi_ap(uint8_t index, const char *ssid, const char *password);

/**
 * @brief Set the upload URL. Saves to NVS.
 */
bool config_set_upload_url(const char *url);

/**
 * @brief Set the POSIX timezone string. Saves to NVS.
 */
bool config_set_timezone(const char *tz);

/**
 * @brief Overwrite the entire config and save to NVS.
 */
bool config_update(const tracker_config_t *new_cfg);
