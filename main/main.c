/*
 * ESP32 GPS Tracker — main.c
 *
 * Thin boot sequence + two FreeRTOS tasks.  All driver logic is in modules:
 *   gps.c, imu.c, rtc.c, lcd.c, power.c, config.c, wifi_manager.c,
 *   upload.c, webserver.c, sd_storage.c, espnow_sync.c, data_merge.c
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_timer.h"

#include "types.h"
#include "config.h"
#include "gps.h"
#include "imu.h"
#include "rtc.h"
#include "lcd.h"
#include "power.h"
#include "wifi_manager.h"
#include "upload.h"
#include "webserver.h"
#include "sd_storage.h"
#include "espnow_sync.h"
#include "serial_log.h"

static const char *TAG = "GPS_TRACKER";

// ── Shared state (read by webserver.c, sync task) ───────────────────────────
gps_data_t  g_current_gps = {0};
imu_data_t  g_current_imu = {0};
char        g_own_mac_str[5] = {0};

// ── Globals ──────────────────────────────────────────────────────────────────
static i2c_master_bus_handle_t i2c_bus0 = NULL;
static i2c_master_bus_handle_t i2c_bus1 = NULL;
static httpd_handle_t          server   = NULL;
static sdmmc_card_t           *card     = NULL;

static volatile uint32_t last_button_time = 0;

// Time flags
static bool time_synced         = false;
static bool time_set_from_rtc   = false;
static bool timezone_set_from_gps = false;

// ── I2C bus initialisation ───────────────────────────────────────────────────

static esp_err_t i2c_init_buses(void)
{
    i2c_master_bus_config_t bus0_cfg = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus0_cfg, &i2c_bus0);
    if (ret != ESP_OK) return ret;

    i2c_master_bus_config_t bus1_cfg = {
        .i2c_port = LCD_I2C_NUM,
        .sda_io_num = LCD_I2C_SDA_PIN,
        .scl_io_num = LCD_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus1_cfg, &i2c_bus1);
}

// ── SD card init ─────────────────────────────────────────────────────────────

static esp_err_t sd_card_init(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 10,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = SDMMC_CLK_GPIO;
    slot_config.cmd = SDMMC_CMD_GPIO;
    slot_config.d0  = SDMMC_D0_GPIO;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host,
                                             &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(ret));
        return ret;
    }
    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}

// ── Button ISR (LCD wake + mode toggle) ──────────────────────────────────────

static void IRAM_ATTR button_isr_handler(void *arg)
{
    uint32_t now = xTaskGetTickCountFromISR();
    if (now - last_button_time > pdMS_TO_TICKS(300)) {
        last_button_time = now;
        lcd_touch_activity();
        if (lcd_request_wake()) {
            // LCD was off — wake it; don't toggle mode
        } else {
            lcd_toggle_mode();
        }
    }
}

static void button_init(void)
{
    gpio_config_t io_conf = {
        .intr_type   = GPIO_INTR_NEGEDGE,
        .mode        = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .pull_up_en  = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL));
}

// ── SNTP ─────────────────────────────────────────────────────────────────────

static void time_sync_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "NTP time sync event");
    time_synced = true;

    time_t now = tv->tv_sec;
    struct tm ti;
    localtime_r(&now, &ti);
    if (rtc_set_time(&ti) == ESP_OK)
        ESP_LOGI(TAG, "RTC updated from NTP");
}

static void sntp_start(void)
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_cb);
    esp_sntp_init();
}

// ── Portable timegm ─────────────────────────────────────────────────────────

static time_t portable_timegm(struct tm *utc_tm)
{
    char saved[64] = {0};
    const char *cur = getenv("TZ");
    if (cur) strncpy(saved, cur, sizeof(saved) - 1);
    setenv("TZ", "UTC0", 1);  tzset();
    struct tm tmp = *utc_tm;
    tmp.tm_isdst = 0;
    time_t result = mktime(&tmp);
    if (saved[0]) setenv("TZ", saved, 1);
    else          unsetenv("TZ");
    tzset();
    return result;
}

// ── CSV logging ──────────────────────────────────────────────────────────────

static esp_err_t log_to_csv(const gps_data_t *gps, const imu_data_t *imu)
{
    if (!gps->valid || (gps->latitude == 0.0f && gps->longitude == 0.0f)){
        ESP_LOGE(TAG, "Invalid GPS data — skipping log");
        return ESP_OK;
    }

  
    static float   last_lat = 0, last_lon = 0, last_alt = 0;
    static int64_t last_ts  = 0;
    int64_t now_ts = (int64_t)time(NULL);
    bool same = (gps->latitude == last_lat && gps->longitude == last_lon &&
                 gps->altitude == last_alt);

    // get th current IMU static state for this log entry (used in gps_filter_position)
    bool imu_static = imu_is_static(imu);



    // Skip unchanged positions unless heartbeat is due
    // if (same && imu_static && (now_ts - last_ts) < 90) {
    //     ESP_LOGW(TAG, "Position: (%.6f, %.6f, %.1f) unchanged — skipping log",
    //              gps->latitude, gps->longitude, gps->altitude);
    //     return ESP_OK;
    // }

    // if position is the same speed must be zero.
    float speed = same ? 0.0f : gps->speed;

    // create CSV row and append to file
    char row[128];
    snprintf(row, sizeof(row),
             "%" PRId64 ",%.6f,%.6f,%.1f,%.1f,%.4f,%.4f,%.4f,%s\n",
             (int64_t)time(NULL),
             gps->latitude, gps->longitude, gps->altitude, speed,
             imu->accel_x, imu->accel_y, imu->accel_z,
             g_own_mac_str);
    ESP_LOGI(TAG, "Logging GPS fix data: %s", row);     
    if (!sd_append_record(row)) return ESP_FAIL;
    last_lat = gps->latitude;
    last_lon = gps->longitude;
    last_alt = gps->altitude;
    last_ts  = now_ts;
    return ESP_OK;
}

// ── GPS logging task (Core 1) ────────────────────────────────────────────────

static void gps_logging_task(void *arg)
{
    gps_data_t gps = {0}, log_gps = {0};
    imu_data_t imu = {0};
    uint32_t   lcd_resync_ctr = 0;

    // IMU averaging accumulators
    float      imu_sum_ax = 0, imu_sum_ay = 0, imu_sum_az = 0;
    float      imu_sum_gx = 0, imu_sum_gy = 0, imu_sum_gz = 0;
    uint32_t   imu_sample_count = 0;

    uint32_t   log_interval_ms = power_get_log_interval_ms();
    TickType_t last_log_tick   = 0;
    bool       imu_static_last = false;

    imu_calibrate(); // runs on Core 1, parallel to sync task startup

    ESP_LOGI(TAG, "GPS logging task started");
    while (1) {
        // ── Sample GPS + IMU at ~5 Hz ────────────────────────────────────────
        bool new_fix = gps_read(&gps);

        esp_err_t imu_ret = imu_read(&imu);
        if (imu_ret == ESP_OK) {
            imu_sum_ax += imu.accel_x; imu_sum_ay += imu.accel_y; imu_sum_az += imu.accel_z;
            imu_sum_gx += imu.gyro_x;  imu_sum_gy += imu.gyro_y;  imu_sum_gz += imu.gyro_z;
            imu_sample_count++;
        }

        // Update shared imu state every cycle; gps state updated after filter below.
        g_current_imu = imu;

        // LCD housekeeping
        lcd_process_wake();
        lcd_check_timeout();

        // Per-fix EMA update (runs at GPS rate for maximum smoothing)
        if (new_fix && gps.valid) {
            gps_filter_position(&gps, imu_static_last);
            log_gps = gps;
        }

        // Share post-filter GPS so webserver and sync task see clean speed/position.
        g_current_gps = gps;

        // Fast motion exit from Stage1/2/3 (runs every GPS_SAMPLE_PERIOD_MS)
        power_fast_motion_check(&gps, &imu, &log_interval_ms);

        // ── Log-tick gate ────────────────────────────────────────────────────
        TickType_t now_tick = xTaskGetTickCount();
        if ((TickType_t)(now_tick - last_log_tick) < pdMS_TO_TICKS(log_interval_ms))
            continue;
        last_log_tick = now_tick;

        // Average accumulated IMU samples
        imu_data_t avg_imu = imu;
        if (imu_sample_count > 0) {
            avg_imu.accel_x = imu_sum_ax / (float)imu_sample_count;
            avg_imu.accel_y = imu_sum_ay / (float)imu_sample_count;
            avg_imu.accel_z = imu_sum_az / (float)imu_sample_count;
            avg_imu.gyro_x  = imu_sum_gx / (float)imu_sample_count;
            avg_imu.gyro_y  = imu_sum_gy / (float)imu_sample_count;
            avg_imu.gyro_z  = imu_sum_gz / (float)imu_sample_count;
            imu_sum_ax = imu_sum_ay = imu_sum_az = 0;
            imu_sum_gx = imu_sum_gy = imu_sum_gz = 0;
            imu_sample_count = 0;
        }

        // Update static gate for next batch of per-fix EMA updates.
        // Use IMU only — no GPS speed dependency (circular: filter zeroes speed
        // only when imu_static_last is already true, which would make this check
        // self-defeating if GPS speed is stale from a previous moving state).
        imu_static_last = imu_is_static(&avg_imu);

        // Power state evaluation (handles stage transitions + position lock)
        power_evaluate(&gps, &avg_imu, &log_interval_ms);

        // Safety net: if IMU confirms static, guarantee logged speed is zero
        // even if the 5 Hz per-fix filter couldn't apply it this cycle.
        if (imu_static_last) {
            log_gps.speed = 0.0f;
        }

        // Set timezone from GPS on first valid fix
        if (gps.valid && !timezone_set_from_gps) {
            const char *tz = gps_get_timezone(gps.latitude, gps.longitude);
            setenv("TZ", tz, 1);  tzset();
            timezone_set_from_gps = true;
            ESP_LOGI(TAG, "Timezone set from GPS: %s", tz);

            // GPS UTC fallback if no NTP/RTC
            if (!time_synced && !time_set_from_rtc && gps.gps_time_valid) {
                struct timeval tv = { .tv_sec = portable_timegm(&gps.gps_utc_time) };
                if (tv.tv_sec > 0) settimeofday(&tv, NULL);
            }
        }

        // Write CSV + update LCD
        log_to_csv(&log_gps, &avg_imu);
        lcd_update(&log_gps, &avg_imu, gps.valid);

        // Periodic LCD resync (~5 s)
        if (lcd_is_initialized() && lcd_is_on()) {
            lcd_resync_ctr++;
            if (lcd_resync_ctr >= (5000 / log_interval_ms)) {
                lcd_resync_ctr = 0;
                lcd_resync();
            }
        }
    }
}

// ── Sync state machine task (Core 0) ─────────────────────────────────────────

static void sync_state_machine_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Sync state machine started");

    while (true) {
        // Wait until device is stationary
        if (!power_is_static(&g_current_gps, &g_current_imu)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Device static — entering sync loop");

        const tracker_config_t *cfg = config_get();
        int     search_time       = 0;
        int64_t next_wifi_attempt = 0;
        static bool s_sntp_started = false;

        while (power_is_static(&g_current_gps, &g_current_imu) &&
               search_time < cfg->sync_search_max) {

            // ── WiFi upload ──────────────────────────────────────────────────
            int64_t interval_s = (power_get_state() >= POWER_STATE_STAGE2)
                ? (int64_t)(config_get()->log_stage2_ms / 1000)
                : (int64_t)cfg->sync_upload_interval_s;
            int64_t now_s = esp_timer_get_time() / 1000000LL;

            if (now_s >= next_wifi_attempt) {
                if (!wifi_mgr_is_connected()) {
                    wifi_mgr_connect();
                }
                if (wifi_mgr_is_connected()) {
                    if (!s_sntp_started) { sntp_start(); s_sntp_started = true; }
                    upload_report_t report = {0};
                    if (upload_all_csv(&report)) {
                        if (report.own_data == UPLOAD_OK)
                            espnow_sync_reset_own_state();
                        if (report.merged_data == UPLOAD_OK)
                            espnow_sync_reset_merged_state();
                    }
                }
                next_wifi_attempt = (esp_timer_get_time() / 1000000LL) + interval_s;
            }

            // ── ESP-NOW sync ─────────────────────────────────────────────────
            if (espnow_init()) {
                int synced = espnow_sync_round();
                ESP_LOGI(TAG, "[%d] Synced with %d peer(s)", search_time, synced);
                espnow_deinit();
            }

            if (!power_is_static(&g_current_gps, &g_current_imu)) break;
            // Once Stage 3 grace period is satisfied, exit the sync loop after
            // this round rather than burning through all sync_search_max iters.
            if (power_deep_sleep_ready()) break;
            search_time++;
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        ESP_LOGI(TAG, "Sync window done (iter=%d)", search_time);

        // Enter deep sleep when STAGE3 sync is complete to conserve battery.
        // power_deep_sleep_ready() enforces a minimum uptime grace period after
        // deep-sleep wake so the system has time to detect motion / get GPS fix.
        if (power_deep_sleep_ready()) {
            ESP_LOGW(TAG, "STAGE3 sync complete — entering deep sleep");
            wifi_mgr_disconnect();
            power_enter_deep_sleep();
            // Does not return; device reboots on wake
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ── app_main ─────────────────────────────────────────────────────────────────

void app_main(void)
{
    ESP_LOGI(TAG, "GPS Tracker starting...");

    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Load config from NVS (or write defaults)
    config_init();
    const tracker_config_t *cfg = config_get();

    // DS3231 power — must precede I2C so the module is on the bus at init time.
    // No-op if RTC_PWR_GPIO == 0 (feature disabled).
    power_rtc_pwr_init();

    // I2C buses
    ESP_ERROR_CHECK(i2c_init_buses());
    ESP_LOGI(TAG, "I2C Bus 0 (IMU/RTC): SDA=%d SCL=%d", I2C_SDA_PIN, I2C_SCL_PIN);
    ESP_LOGI(TAG, "I2C Bus 1 (LCD): SDA=%d SCL=%d", LCD_I2C_SDA_PIN, LCD_I2C_SCL_PIN);

    // Set timezone before RTC read
    setenv("TZ", cfg->timezone, 1);  tzset();

    // RTC → system time
    if (rtc_module_init(i2c_bus0) == ESP_OK) {
        struct tm ti;
        if (rtc_get_time(&ti) == ESP_OK) {
            struct timeval tv = { .tv_sec = mktime(&ti) };
            settimeofday(&tv, NULL);
            time_set_from_rtc = true;
            ESP_LOGI(TAG, "System time set from RTC");
        }
    }

    // GPS — start UART as early as possible for fastest first fix
    gps_init();

    // IMU — init only; calibration deferred to Core 1 (gps_logging_task)
    if (imu_init(i2c_bus0) != ESP_OK) {
        ESP_LOGW(TAG, "IMU not found");
    }

    // SD card
    if (sd_card_init() != ESP_OK) {
        ESP_LOGE(TAG, "SD card init failed");
    }
    if (!sd_init()) {
        ESP_LOGE(TAG, "Sync SD init failed");
    }

    // Start mirroring serial output to SD card
    serial_log_init();

    // MAC suffix for CSV records
    {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        snprintf(g_own_mac_str, sizeof(g_own_mac_str), "%02X%02X", mac[4], mac[5]);
        ESP_LOGI(TAG, "Device MAC suffix: %s", g_own_mac_str);
    }

    sd_ensure_data_file();
    sd_ensure_merged_file();

    // WiFi stack init (fast, non-blocking — connect is handled by the sync task)
    wifi_mgr_init();

    // Web server
    server = webserver_start();

    // LCD — deferred past GPS/IMU/SD; splash delay removed (task updates immediately)
#if LCD_ENABLED
    if (lcd_init(i2c_bus1) == ESP_OK) {
        lcd_clear();
        lcd_printf(0, 0, "GPS Tracker");
        lcd_printf(0, 1, "Initializing...");
    }
#endif

    // Button
    button_init();

    // Power manager
    power_init();

    // Check deep sleep wake reason
    if (power_check_wake_reason()) {
        ESP_LOGI(TAG, "Woke from deep sleep");
    }

    // Tasks — start BEFORE attempting WiFi so GPS logging begins immediately,
    // regardless of whether a network is reachable.
    xTaskCreatePinnedToCore(gps_logging_task, "gps_log", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(sync_state_machine_task, "sync_sm", 8192, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "System initialized — logging GPS data");
}
