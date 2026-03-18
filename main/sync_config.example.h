#pragma once

// =============================================================================
// tracker_config — single source of truth for all build-time settings.
//
// Copy this file to sync_config.h, fill in your credentials and hardware
// details, and do not commit the resulting sync_config.h to source control.
// =============================================================================

// ─── WiFi credentials ────────────────────────────────────────────────────────
#define WIFI_SSID        "<YOUR_WIFI_SSID>"
#define WIFI_PASSWORD    "<YOUR_WIFI_PASSWORD>"
#define WIFI_MAX_RETRY   3

// ─── Upload endpoint ─────────────────────────────────────────────────────────
#define UPLOAD_URL       "<YOUR_UPLOAD_URL>"

// ─── Timezone (POSIX string) ─────────────────────────────────────────────────
// Common values:
//   "EST5EDT,M3.2.0/2,M11.1.0"           US Eastern
//   "CST6CDT,M3.2.0,M11.1.0"             US Central
//   "MST7MDT,M3.2.0,M11.1.0"             US Mountain
//   "PST8PDT,M3.2.0,M11.1.0"             US Pacific
//   "GMT0BST,M3.5.0/1,M10.5.0"           UK
//   "CET-1CEST,M3.5.0,M10.5.0/3"         Central Europe
//   "AEST-10AEDT,M10.1.0,M4.1.0/3"       Australia Eastern
// Overridden at runtime by GPS-derived timezone on first valid fix.
#define TIMEZONE         "MST7MDT,M3.2.0,M11.1.0"

// ─── GPS UART ────────────────────────────────────────────────────────────────
#define GPS_UART_NUM     UART_NUM_1
#define GPS_TX_PIN       23
#define GPS_RX_PIN       22
#define GPS_BAUD_RATE    9600
#define UART_BUF_SIZE    1024

// ─── I2C Bus 0 — RTC + IMU ───────────────────────────────────────────────────
#define I2C_MASTER_NUM   I2C_NUM_0
#define I2C_SDA_PIN      18
#define I2C_SCL_PIN      19
#define I2C_FREQ_HZ      100000

// ─── I2C Bus 1 — LCD (dedicated, no contention with RTC/IMU) ─────────────────
#define LCD_I2C_NUM      I2C_NUM_1
#define LCD_I2C_SDA_PIN  32
#define LCD_I2C_SCL_PIN  33
#define LCD_I2C_FREQ_HZ  100000

// ─── I2C device addresses ────────────────────────────────────────────────────
#define RTC_I2C_ADDR     0x68
#define MPU6050_I2C_ADDR 0x69   // AD0 HIGH; use 0x68 if AD0 LOW
#define LCD_I2C_ADDR     0x27   // Try 0x3F if not found

// ─── SD card (SDMMC 1-bit) ───────────────────────────────────────────────────
#define SDMMC_CMD_GPIO   15
#define SDMMC_CLK_GPIO   14
#define SDMMC_D0_GPIO    2
#define SD_MOUNT_POINT   "/sdcard"
#define GPS_DIR          SD_MOUNT_POINT "/gps"

// ─── Button ──────────────────────────────────────────────────────────────────
#define BUTTON_GPIO      0      // BOOT button on most ESP32 boards

// ─── LCD feature flags ───────────────────────────────────────────────────────
#define LCD_ENABLED      1      // Set to 0 to build without LCD support
#define LCD_TIMEOUT_SEC  30     // Seconds of inactivity before auto-off (0 = always on)

// ─── Sync CSV file paths ─────────────────────────────────────────────────────
#define CSV_DATA_FILE    SD_MOUNT_POINT "/sync_data.csv"
#define CSV_MERGED_FILE  SD_MOUNT_POINT "/sync_merged.csv"
#define CSV_MERGE_TMP    SD_MOUNT_POINT "/sync_tmp.csv"

// CSV column layout for records exchanged via ESP-NOW.
#define CSV_HEADER  "timestamp,lat,lon,accel_x,accel_y,accel_z,device_mac\n"

// ─── Motion detection thresholds ─────────────────────────────────────────────
#define GPS_STATIC_SPEED_KMH  2.0f    // km/h  — below this = stationary
#define IMU_ACCEL_DEVIATION   0.15f   // g     — deviation from 1 g = stationary
#define IMU_GYRO_STATIC_DPS   5.0f    // deg/s — below this = stationary

// ─── ESP-NOW protocol parameters ─────────────────────────────────────────────
#define ESPNOW_BROADCAST_MAC    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
#define ESPNOW_CHANNEL          1
#define ESPNOW_RX_BUF_RECORDS   200
#define ESPNOW_RECV_TIMEOUT_MS  8000
#define ESPNOW_DISCOVER_MS      30000

// ─── Sync state machine ───────────────────────────────────────────────────────
#define SYNC_SEARCH_TIME_MAX         300  // loop iterations before resetting
#define SYNC_WIFI_UPLOAD_INTERVAL_S   30  // seconds between WiFi upload attempts

// ─── Test mode ───────────────────────────────────────────────────────────────
#define TEST_MODE              0
#define TEST_DEVICE_IS_STATIC  0
