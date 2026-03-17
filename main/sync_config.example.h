#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// sync_config.h — Configuration for the ESP-NOW sync subsystem.
//
// Included by espnow_sync.c, sd_storage.c, data_merge.c, wifi_upload.c, and
// main.c.  Keep WiFi credentials in sync with the tracker's main WiFi config.
// ─────────────────────────────────────────────────────────────────────────────

// ─── WiFi credentials ────────────────────────────────────────────────────────
// Must match the AP the tracker connects to.
#define WIFI_SSID        "<YOUR_WIFI_SSID>"
#define WIFI_PASSWORD    "<YOUR_WIFI_PASSWORD>"
#define WIFI_MAX_RETRY   3

// ─── Upload endpoint ─────────────────────────────────────────────────────────
#define UPLOAD_URL       "<YOUR_UPLOAD_URL>"

// ─── SD card pins (must match tracker hardware wiring) ───────────────────────
#define SDMMC_CMD_GPIO   15
#define SDMMC_CLK_GPIO   14
#define SDMMC_D0_GPIO    2
#define SD_MOUNT_POINT   "/sdcard"

// ─── Sync CSV file paths (separate from the tracker's GPS daily logs) ────────
#define CSV_DATA_FILE    SD_MOUNT_POINT "/sync_data.csv"
#define CSV_MERGED_FILE  SD_MOUNT_POINT "/sync_merged.csv"
#define CSV_MERGE_TMP    SD_MOUNT_POINT "/sync_tmp.csv"

// CSV column layout for records exchanged via ESP-NOW.
// The tracker's log_to_csv() writes a matching row to CSV_DATA_FILE so that
// own GPS records are included in peer exchanges.
#define CSV_HEADER  "timestamp,lat,lon,accel_x,accel_y,accel_z,device_mac\n"

// ─── ESP-NOW protocol parameters ─────────────────────────────────────────────
#define ESPNOW_BROADCAST_MAC    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
#define ESPNOW_CHANNEL          1
#define ESPNOW_RX_BUF_RECORDS   200
#define ESPNOW_RECV_TIMEOUT_MS  8000
#define ESPNOW_DISCOVER_MS      30000

// ─── Sync state machine ───────────────────────────────────────────────────────
#define SYNC_SEARCH_TIME_MAX         300  // loop iterations before resetting
#define SYNC_WIFI_UPLOAD_INTERVAL_S   30  // seconds between upload attempts

// ─── Test mode (disabled in tracker build) ───────────────────────────────────
#define TEST_MODE              0
#define TEST_DEVICE_IS_STATIC  0
