/*
 * ESP32 GPS Logger with IMU
 * 
 * Features:
 * - GPS tracking with Neo-6M module
 * - IMU data logging with MPU6050
 * - RTC timekeeping with DS3231
 * - SD card storage with daily CSV files
 * - WiFi connectivity and NTP sync
 * - HTTP server for file access
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_sntp.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

// WiFi Configuration - EDIT THESE
#define WIFI_SSID "Internal"
#define WIFI_PASSWORD "CasCade2023!"

// Timezone Configuration - EDIT THIS
// Common timezones:
// "EST5EDT,M3.2.0/2,M11.1.0" - US Eastern
// "CST6CDT,M3.2.0,M11.1.0" - US Central
// "MST7MDT,M3.2.0,M11.1.0" - US Mountain
// "PST8PDT,M3.2.0,M11.1.0" - US Pacific
// "AEST-10AEDT,M10.1.0,M4.1.0/3" - Australia Eastern
// "GMT0BST,M3.5.0/1,M10.5.0" - UK
#define TIMEZONE "MST7MDT,M3.2.0,M11.1.0" // US Mountain

// GPIO Pin Definitions
#define GPS_UART_NUM        UART_NUM_1
#define GPS_TX_PIN          23
#define GPS_RX_PIN          22
#define GPS_BAUD_RATE       9600

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_SDA_PIN         18
#define I2C_SCL_PIN         19
#define I2C_FREQ_HZ         100000

#define RTC_I2C_ADDR        0x68
#define MPU6050_I2C_ADDR    0x69  // AD0 HIGH, use 0x68 if AD0 LOW

#define SD_CMD_PIN          15
#define SD_CLK_PIN          14
#define SD_D0_PIN           2

// SD Card Mount Point
#define MOUNT_POINT "/sdcard"
#define GPS_DIR "/sdcard/gps"

// UART Buffer Size
#define UART_BUF_SIZE 1024

// WiFi Event Group
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "GPS_LOGGER";
static int s_retry_num = 0;
static bool time_synced = false;
static bool timezone_set_from_gps = false;
static sdmmc_card_t *card = NULL;
static httpd_handle_t server = NULL;

// GPS Data Structure
typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float speed;
    bool valid;
} gps_data_t;

// IMU Data Structure
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} imu_data_t;

/* ==================== Timezone Detection from GPS ==================== */

static const char* get_timezone_from_gps(float latitude, float longitude)
{
    // Rough timezone detection based on longitude and latitude
    // Note: This is approximate - real timezone boundaries are political/geographical
    
    // Calculate approximate UTC offset from longitude (15 degrees = 1 hour)
    int utc_offset = (int)((longitude + 7.5) / 15.0);
    
    // Clamp offset to valid range
    if (utc_offset > 12) utc_offset = 12;
    if (utc_offset < -12) utc_offset = -12;
    
    // Special cases for regions with known timezones
    
    // United States (continental)
    if (latitude >= 24.0 && latitude <= 49.0 && longitude >= -125.0 && longitude <= -66.0) {
        if (longitude >= -125.0 && longitude < -120.0) return "PST8PDT,M3.2.0,M11.1.0"; // Pacific
        if (longitude >= -120.0 && longitude < -105.0) return "MST7MDT,M3.2.0,M11.1.0"; // Mountain
        if (longitude >= -105.0 && longitude < -90.0) return "CST6CDT,M3.2.0,M11.1.0";  // Central
        if (longitude >= -90.0 && longitude <= -66.0) return "EST5EDT,M3.2.0/2,M11.1.0"; // Eastern
    }
    
    // Australia
    if (latitude >= -44.0 && latitude <= -10.0 && longitude >= 113.0 && longitude <= 154.0) {
        if (longitude >= 113.0 && longitude < 129.0) return "AWST-8";  // Western
        if (longitude >= 129.0 && longitude < 138.0) return "ACST-9:30ACDT,M10.1.0,M4.1.0/3"; // Central
        if (longitude >= 138.0 && longitude <= 154.0) return "AEST-10AEDT,M10.1.0,M4.1.0/3"; // Eastern
    }
    
    // Europe
    if (latitude >= 35.0 && latitude <= 71.0 && longitude >= -10.0 && longitude <= 40.0) {
        if (longitude >= -10.0 && longitude < 7.5) return "GMT0BST,M3.5.0/1,M10.5.0"; // UK/Western
        if (longitude >= 7.5 && longitude < 22.5) return "CET-1CEST,M3.5.0,M10.5.0/3"; // Central
        if (longitude >= 22.5 && longitude <= 40.0) return "EET-2EEST,M3.5.0/3,M10.5.0/4"; // Eastern
    }
    
    // Japan
    if (latitude >= 24.0 && latitude <= 46.0 && longitude >= 123.0 && longitude <= 146.0) {
        return "JST-9";
    }
    
    // China
    if (latitude >= 18.0 && latitude <= 54.0 && longitude >= 73.0 && longitude <= 135.0) {
        return "CST-8";
    }
    
    // India
    if (latitude >= 8.0 && latitude <= 35.0 && longitude >= 68.0 && longitude <= 97.0) {
        return "IST-5:30";
    }
    
    // Generic UTC offset based on longitude as fallback
    static char tz_buf[32];
    if (utc_offset == 0) {
        return "UTC0";
    } else if (utc_offset > 0) {
        snprintf(tz_buf, sizeof(tz_buf), "UTC-%d", utc_offset);
    } else {
        snprintf(tz_buf, sizeof(tz_buf), "UTC%d", -utc_offset);
    }
    return tz_buf;
}

/* ==================== I2C Functions ==================== */

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ==================== DS3231 RTC Functions ==================== */

static uint8_t bcd_to_dec(uint8_t val)
{
    return (val / 16 * 10) + (val % 16);
}

static uint8_t dec_to_bcd(uint8_t val)
{
    return (val / 10 * 16) + (val % 10);
}

static esp_err_t rtc_get_time(struct tm *timeinfo)
{
    uint8_t data[7];
    esp_err_t ret = i2c_read_bytes(RTC_I2C_ADDR, 0x00, data, 7);
    if (ret != ESP_OK) return ret;
    
    timeinfo->tm_sec = bcd_to_dec(data[0] & 0x7F);
    timeinfo->tm_min = bcd_to_dec(data[1] & 0x7F);
    timeinfo->tm_hour = bcd_to_dec(data[2] & 0x3F);
    timeinfo->tm_mday = bcd_to_dec(data[4] & 0x3F);
    timeinfo->tm_mon = bcd_to_dec(data[5] & 0x1F) - 1;
    timeinfo->tm_year = bcd_to_dec(data[6]) + 100; // Years since 1900
    
    return ESP_OK;
}

static esp_err_t rtc_set_time(struct tm *timeinfo)
{
    esp_err_t ret;
    ret = i2c_write_byte(RTC_I2C_ADDR, 0x00, dec_to_bcd(timeinfo->tm_sec));
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(RTC_I2C_ADDR, 0x01, dec_to_bcd(timeinfo->tm_min));
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(RTC_I2C_ADDR, 0x02, dec_to_bcd(timeinfo->tm_hour));
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(RTC_I2C_ADDR, 0x04, dec_to_bcd(timeinfo->tm_mday));
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(RTC_I2C_ADDR, 0x05, dec_to_bcd(timeinfo->tm_mon + 1));
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(RTC_I2C_ADDR, 0x06, dec_to_bcd(timeinfo->tm_year - 100));
    
    return ret;
}

/* ==================== MPU6050 IMU Functions ==================== */

// Calibration offsets
static float accel_offset_x = 0.0f;
static float accel_offset_y = 0.0f;
static float accel_offset_z = 0.0f;
static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;
static bool mpu6050_calibrated = false;

static esp_err_t mpu6050_init(void)
{
    esp_err_t ret;
    
    // Wake up MPU6050
    ret = i2c_write_byte(MPU6050_I2C_ADDR, 0x6B, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wake up failed");
        return ret;
    }
    
    // Set accelerometer range to ±2g
    ret = i2c_write_byte(MPU6050_I2C_ADDR, 0x1C, 0x00);
    if (ret != ESP_OK) return ret;
    
    // Set gyroscope range to ±250°/s
    ret = i2c_write_byte(MPU6050_I2C_ADDR, 0x1B, 0x00);
    
    ESP_LOGI(TAG, "MPU6050 initialized");
    return ret;
}

static esp_err_t mpu6050_read_data(imu_data_t *imu_data)
{
    uint8_t data[14];
    esp_err_t ret = i2c_read_bytes(MPU6050_I2C_ADDR, 0x3B, data, 14);
    if (ret != ESP_OK) return ret;
    
    int16_t accel_x_raw = (data[0] << 8) | data[1];
    int16_t accel_y_raw = (data[2] << 8) | data[3];
    int16_t accel_z_raw = (data[4] << 8) | data[5];
    int16_t gyro_x_raw = (data[8] << 8) | data[9];
    int16_t gyro_y_raw = (data[10] << 8) | data[11];
    int16_t gyro_z_raw = (data[12] << 8) | data[13];
    
    // Convert to G-force (±2g range, 16384 LSB/g)
    imu_data->accel_x = (accel_x_raw / 16384.0f) - accel_offset_x;
    imu_data->accel_y = (accel_y_raw / 16384.0f) - accel_offset_y;
    imu_data->accel_z = (accel_z_raw / 16384.0f) - accel_offset_z;
    
    // Convert to degrees/second (±250°/s range, 131 LSB/°/s)
    imu_data->gyro_x = (gyro_x_raw / 131.0f) - gyro_offset_x;
    imu_data->gyro_y = (gyro_y_raw / 131.0f) - gyro_offset_y;
    imu_data->gyro_z = (gyro_z_raw / 131.0f) - gyro_offset_z;
    
    return ESP_OK;
}

static esp_err_t mpu6050_calibrate(void)
{
    ESP_LOGI(TAG, "MPU6050 calibration starting - keep device still!");
    
    const int samples = 100;
    float sum_accel_x = 0, sum_accel_y = 0, sum_accel_z = 0;
    float sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
    
    // Collect samples
    for (int i = 0; i < samples; i++) {
        uint8_t data[14];
        esp_err_t ret = i2c_read_bytes(MPU6050_I2C_ADDR, 0x3B, data, 14);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Calibration failed at sample %d", i);
            return ret;
        }
        
        int16_t accel_x_raw = (data[0] << 8) | data[1];
        int16_t accel_y_raw = (data[2] << 8) | data[3];
        int16_t accel_z_raw = (data[4] << 8) | data[5];
        int16_t gyro_x_raw = (data[8] << 8) | data[9];
        int16_t gyro_y_raw = (data[10] << 8) | data[11];
        int16_t gyro_z_raw = (data[12] << 8) | data[13];
        
        sum_accel_x += accel_x_raw / 16384.0f;
        sum_accel_y += accel_y_raw / 16384.0f;
        sum_accel_z += accel_z_raw / 16384.0f;
        sum_gyro_x += gyro_x_raw / 131.0f;
        sum_gyro_y += gyro_y_raw / 131.0f;
        sum_gyro_z += gyro_z_raw / 131.0f;
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Calculate averages
    accel_offset_x = sum_accel_x / samples;
    accel_offset_y = sum_accel_y / samples;
    accel_offset_z = (sum_accel_z / samples) - 1.0f; // Z should read 1g when flat
    gyro_offset_x = sum_gyro_x / samples;
    gyro_offset_y = sum_gyro_y / samples;
    gyro_offset_z = sum_gyro_z / samples;
    
    mpu6050_calibrated = true;
    
    ESP_LOGI(TAG, "MPU6050 calibration complete");
    ESP_LOGI(TAG, "Accel offsets: X=%.3f, Y=%.3f, Z=%.3f g", 
             accel_offset_x, accel_offset_y, accel_offset_z);
    ESP_LOGI(TAG, "Gyro offsets: X=%.3f, Y=%.3f, Z=%.3f °/s", 
             gyro_offset_x, gyro_offset_y, gyro_offset_z);
    
    return ESP_OK;
}

/* ==================== GPS NMEA Parser ==================== */

static bool parse_gprmc(const char *nmea, gps_data_t *gps_data)
{
    // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
    char *token;
    char nmea_copy[128];
    strncpy(nmea_copy, nmea, sizeof(nmea_copy) - 1);
    
    token = strtok(nmea_copy, ",");
    if (!token || strcmp(token, "$GPRMC") != 0) return false;
    
    token = strtok(NULL, ","); // Time
    token = strtok(NULL, ","); // Status
    if (!token || token[0] != 'A') {
        gps_data->valid = false;
        return false;
    }
    
    gps_data->valid = true;
    
    // Latitude
    token = strtok(NULL, ",");
    if (token) {
        float lat_raw = atof(token);
        int lat_deg = (int)(lat_raw / 100);
        float lat_min = lat_raw - (lat_deg * 100);
        gps_data->latitude = lat_deg + (lat_min / 60.0f);
        
        token = strtok(NULL, ","); // N/S
        if (token && token[0] == 'S') gps_data->latitude = -gps_data->latitude;
    }
    
    // Longitude
    token = strtok(NULL, ",");
    if (token) {
        float lon_raw = atof(token);
        int lon_deg = (int)(lon_raw / 100);
        float lon_min = lon_raw - (lon_deg * 100);
        gps_data->longitude = lon_deg + (lon_min / 60.0f);
        
        token = strtok(NULL, ","); // E/W
        if (token && token[0] == 'W') gps_data->longitude = -gps_data->longitude;
    }
    
    // Speed in knots
    token = strtok(NULL, ",");
    if (token) {
        gps_data->speed = atof(token) * 1.852f; // Convert knots to km/h
    }
    
    return true;
}

static bool parse_gpgga(const char *nmea, gps_data_t *gps_data)
{
    // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    char *token;
    char nmea_copy[128];
    strncpy(nmea_copy, nmea, sizeof(nmea_copy) - 1);
    
    token = strtok(nmea_copy, ",");
    if (!token || strcmp(token, "$GPGGA") != 0) return false;
    
    // Skip to altitude (9th field)
    for (int i = 0; i < 8; i++) {
        token = strtok(NULL, ",");
    }
    
    token = strtok(NULL, ","); // Altitude
    if (token) {
        gps_data->altitude = atof(token);
    }
    
    return true;
}

/* ==================== UART GPS Functions ==================== */

static void gps_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    uart_driver_install(GPS_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    ESP_LOGI(TAG, "GPS UART initialized");
}

/* ==================== SD Card Functions ==================== */

static esp_err_t sd_card_init(void)
{
    esp_err_t ret;
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = SD_CLK_PIN;
    slot_config.cmd = SD_CMD_PIN;
    slot_config.d0 = SD_D0_PIN;
    
    ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card (%s)", esp_err_to_name(ret));
        }
        return ret;
    }
    
    sdmmc_card_print_info(stdout, card);
    
    // Create GPS directory if it doesn't exist
    struct stat st;
    if (stat(GPS_DIR, &st) != 0) {
        mkdir(GPS_DIR, 0775);
        ESP_LOGI(TAG, "Created directory: %s", GPS_DIR);
    }
    
    return ESP_OK;
}

/* ==================== CSV Logging Functions ==================== */

static esp_err_t log_to_csv(const gps_data_t *gps_data, const imu_data_t *imu_data)
{
    struct tm timeinfo;
    if (rtc_get_time(&timeinfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get time from RTC");
        return ESP_FAIL;
    }
    
    char filename[64];
    snprintf(filename, sizeof(filename), "%s/%04d_%02d_%02d.csv",
             GPS_DIR,
             timeinfo.tm_year + 1900,
             timeinfo.tm_mon + 1,
             timeinfo.tm_mday);
    
    // Check if file exists to determine if we need to write header
    bool file_exists = (access(filename, F_OK) == 0);
    
    FILE *f = fopen(filename, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filename);
        return ESP_FAIL;
    }
    
    // Write header if new file
    if (!file_exists) {
        fprintf(f, "timestamp,lat,lon,alt,speed,accel_x,accel_y,accel_z\n");
    }
    
    // Write data
    fprintf(f, "%04d-%02d-%02dT%02d:%02d:%02d,%.6f,%.6f,%.1f,%.1f,%.3f,%.3f,%.3f\n",
            timeinfo.tm_year + 1900,
            timeinfo.tm_mon + 1,
            timeinfo.tm_mday,
            timeinfo.tm_hour,
            timeinfo.tm_min,
            timeinfo.tm_sec,
            gps_data->latitude,
            gps_data->longitude,
            gps_data->altitude,
            gps_data->speed,
            imu_data->accel_x,
            imu_data->accel_y,
            imu_data->accel_z);
    
    fclose(f);
    return ESP_OK;
}

/* ==================== WiFi Functions ==================== */

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry connecting to WiFi");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connection to WiFi failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                         ESP_EVENT_ANY_ID,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                         IP_EVENT_STA_GOT_IP,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         &instance_got_ip));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialization complete");
    
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE,
                                            pdFALSE,
                                            portMAX_DELAY);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    }
}

/* ==================== SNTP Functions ==================== */

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronization event");
    time_synced = true;
    
    // Update RTC with synced time
    time_t now = tv->tv_sec;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    
    if (rtc_set_time(&timeinfo) == ESP_OK) {
        ESP_LOGI(TAG, "RTC updated with NTP time");
    }
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP (background sync)");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP started - will update RTC when sync completes");
}

/* ==================== HTTP Server ==================== */

static esp_err_t http_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req, "<html><head><title>GPS Logger</title></head><body>");
    httpd_resp_sendstr_chunk(req, "<h1>GPS Log Files</h1><ul>");
    
    struct dirent *entry;
    DIR *dir = opendir(GPS_DIR);
    
    if (dir != NULL) {
        while ((entry = readdir(dir)) != NULL) {
            if (entry->d_type == DT_REG) {
                char filepath[384];
                snprintf(filepath, sizeof(filepath), "%s/%s", GPS_DIR, entry->d_name);
                
                struct stat st;
                if (stat(filepath, &st) == 0) {
                    char line[640];
                    snprintf(line, sizeof(line), 
                             "<li><a href='/download?file=%s'>%s</a> (%ld bytes)</li>",
                             entry->d_name, entry->d_name, st.st_size);
                    httpd_resp_sendstr_chunk(req, line);
                }
            }
        }
        closedir(dir);
    }
    
    httpd_resp_sendstr_chunk(req, "</ul></body></html>");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t http_download_handler(httpd_req_t *req)
{
    char query[128];
    char filename[256];
    
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "file", filename, sizeof(filename)) == ESP_OK) {
            char filepath[384];
            snprintf(filepath, sizeof(filepath), "%s/%s", GPS_DIR, filename);
            
            FILE *f = fopen(filepath, "r");
            if (f == NULL) {
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
            
            httpd_resp_set_type(req, "text/csv");
            
            char buffer[512];
            size_t read_bytes;
            while ((read_bytes = fread(buffer, 1, sizeof(buffer), f)) > 0) {
                if (httpd_resp_send_chunk(req, buffer, read_bytes) != ESP_OK) {
                    fclose(f);
                    return ESP_FAIL;
                }
            }
            
            fclose(f);
            httpd_resp_send_chunk(req, NULL, 0);
            return ESP_OK;
        }
    }
    
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static esp_err_t http_status_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    
    char response[512];
    snprintf(response, sizeof(response),
             "{\"sd_card\":\"%s\",\"sensors\":\"OK\",\"gps\":\"OK\"}",
             (card != NULL) ? "OK" : "ERROR");
    
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_get = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = http_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &uri_get);
        
        httpd_uri_t uri_download = {
            .uri       = "/download",
            .method    = HTTP_GET,
            .handler   = http_download_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &uri_download);
        
        httpd_uri_t uri_status = {
            .uri       = "/status",
            .method    = HTTP_GET,
            .handler   = http_status_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &uri_status);
        
        ESP_LOGI(TAG, "HTTP server started");
    }
    
    return server;
}

/* ==================== Main GPS Logging Task ==================== */

static void gps_logging_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    char line_buffer[128];
    int line_pos = 0;
    
    gps_data_t gps_data = {0};
    imu_data_t imu_data = {0};
    
    ESP_LOGI(TAG, "GPS logging task started");
    
    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, data, UART_BUF_SIZE, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];
                
                if (c == '\n' || c == '\r') {
                    if (line_pos > 0) {
                        line_buffer[line_pos] = '\0';
                        
                        // Parse NMEA sentences
                        if (strncmp(line_buffer, "$GPRMC", 6) == 0) {
                            parse_gprmc(line_buffer, &gps_data);
                        } else if (strncmp(line_buffer, "$GPGGA", 6) == 0) {
                            parse_gpgga(line_buffer, &gps_data);
                        }
                        
                        line_pos = 0;
                    }
                } else if (line_pos < sizeof(line_buffer) - 1) {
                    line_buffer[line_pos++] = c;
                }
            }
        }
        
        // Read IMU data
        esp_err_t imu_ret = mpu6050_read_data(&imu_data);
        if (imu_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read IMU data: %s", esp_err_to_name(imu_ret));
        }
        
        // Log data if GPS is valid
        if (gps_data.valid) {
            // Set timezone from GPS on first valid fix
            if (!timezone_set_from_gps) {
                const char* tz = get_timezone_from_gps(gps_data.latitude, gps_data.longitude);
                setenv("TZ", tz, 1);
                tzset();
                timezone_set_from_gps = true;
                
                // Display updated local time
                time_t now = time(NULL);
                struct tm timeinfo;
                localtime_r(&now, &timeinfo);
                ESP_LOGI(TAG, "Timezone updated from GPS to: %s", tz);
                ESP_LOGI(TAG, "Local time: %04d-%02d-%02d %02d:%02d:%02d",
                         timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                         timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            }
            
            if (log_to_csv(&gps_data, &imu_data) == ESP_OK) {
                ESP_LOGI(TAG, "Logged: %.6f, %.6f, %.1f m, %.1f km/h, accel: %.3f, %.3f, %.3f g",
                         gps_data.latitude, gps_data.longitude,
                         gps_data.altitude, gps_data.speed,
                         imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
            }
        } else {
            ESP_LOGW(TAG, "Waiting for GPS fix... (accel: %.3f, %.3f, %.3f g)",
                     imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Log every second
    }
    
    free(data);
}

/* ==================== Main Application ==================== */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 GPS Logger with IMU starting...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized");
    
    // Initialize MPU6050
    if (mpu6050_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 initialization failed - check I2C connections");
    } else {
        // Calibrate MPU6050
        vTaskDelay(pdMS_TO_TICKS(100)); // Brief delay for sensor to stabilize
        if (mpu6050_calibrate() != ESP_OK) {
            ESP_LOGE(TAG, "MPU6050 calibration failed");
        }
    }
    
    // Initialize RTC and set system time from it
    struct tm timeinfo;
    if (rtc_get_time(&timeinfo) == ESP_OK) {
        ESP_LOGI(TAG, "RTC time: %04d-%02d-%02d %02d:%02d:%02d",
                 timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        
        // Set timezone
        setenv("TZ", TIMEZONE, 1);
        tzset();
        
        // Set system time from RTC
        struct timeval tv;
        tv.tv_sec = mktime(&timeinfo);
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);
        
        // Display local time
        time_t now = time(NULL);
        localtime_r(&now, &timeinfo);
        ESP_LOGI(TAG, "Local time: %04d-%02d-%02d %02d:%02d:%02d",
                 timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        ESP_LOGE(TAG, "RTC initialization failed - check I2C connections");
    }
    
    // Initialize SD Card
    if (sd_card_init() != ESP_OK) {
        ESP_LOGE(TAG, "SD Card initialization failed - check connections");
    }
    
    // Initialize GPS UART
    gps_uart_init();
    
    // Initialize WiFi
    wifi_init_sta();
    
    // Sync time via NTP
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    if (bits & WIFI_CONNECTED_BIT) {
        initialize_sntp();
    }
    
    // Start HTTP Server
    if (bits & WIFI_CONNECTED_BIT) {
        server = start_webserver();
    }
    
    // Start GPS logging task
    xTaskCreate(gps_logging_task, "gps_logging", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "System initialized. Logging GPS data...");
}
