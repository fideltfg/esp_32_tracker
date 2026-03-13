/*
 * ESP32 GPS Logger with IMU
 * 
 * Features:
 * - GPS tracking with Neo-6M module
 * - IMU data logging with MPU6050/MPU6500/MPU9250
 * - RTC timekeeping with DS3231
 * - SD card storage with daily CSV files
 * - WiFi connectivity and NTP sync
 * - HTTP server for file access
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
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
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "rom/ets_sys.h"

// WiFi Configuration - EDIT THESE
#define WIFI_SSID "Zero Gravitas"
#define WIFI_PASSWORD "ILoveLydia!"

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

#define LCD_I2C_NUM         I2C_NUM_1
#define LCD_I2C_SDA_PIN     32
#define LCD_I2C_SCL_PIN     33
#define LCD_I2C_FREQ_HZ     100000  // Dedicated bus for LCD

#define RTC_I2C_ADDR        0x68
#define MPU6050_I2C_ADDR    0x69  // AD0 HIGH, use 0x68 if AD0 LOW
#define LCD_I2C_ADDR        0x27  // LCD1602A with I2C backpack, try 0x3F if not found

#define SD_CMD_PIN          15
#define SD_CLK_PIN          14
#define SD_D0_PIN           2

#define BUTTON_GPIO         0    // BOOT button on most ESP32 boards

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

// Display mode control
static volatile uint8_t display_mode = 0; // 0 = GPS/IMU, 1 = Time/IP
static volatile uint32_t last_button_time = 0;

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

static i2c_master_bus_handle_t i2c_bus0 = NULL;
static i2c_master_bus_handle_t i2c_bus1 = NULL;
static i2c_master_dev_handle_t i2c_dev_rtc = NULL;
static i2c_master_dev_handle_t i2c_dev_imu = NULL;
static i2c_master_dev_handle_t i2c_dev_lcd = NULL;

static esp_err_t i2c_master_init(void)
{
    // Initialize I2C_NUM_0 for IMU and RTC
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

    // Pre-create RTC device handle (fixed address)
    i2c_device_config_t rtc_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = RTC_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(i2c_bus0, &rtc_dev_cfg, &i2c_dev_rtc);
    if (ret != ESP_OK) return ret;

    // Initialize I2C_NUM_1 for LCD (separate bus - no contention!)
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

static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C Bus 0 (IMU/RTC)...");
    int devices_found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_master_probe(i2c_bus0, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  Bus 0: Found device at 0x%02X", addr);
            devices_found++;
        }
    }

    if (devices_found == 0) {
        ESP_LOGW(TAG, "  Bus 0: No I2C devices found!");
    } else {
        ESP_LOGI(TAG, "  Bus 0: Found %d device(s)", devices_found);
    }

    // Scan LCD bus
    ESP_LOGI(TAG, "Scanning I2C Bus 1 (LCD)...");
    devices_found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_master_probe(i2c_bus1, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  Bus 1: Found device at 0x%02X", addr);
            devices_found++;
        }
    }

    if (devices_found == 0) {
        ESP_LOGW(TAG, "  Bus 1: No I2C devices found - LCD not connected?");
    } else {
        ESP_LOGI(TAG, "  Bus 1: Found %d device(s)", devices_found);
    }
}

static esp_err_t i2c_write_byte(i2c_master_dev_handle_t dev, uint8_t reg_addr, uint8_t data)
{
    uint8_t buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev, buf, 2, 1000);
}

static esp_err_t i2c_read_bytes(i2c_master_dev_handle_t dev, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg_addr, 1, data, len, 1000);
}

/* ==================== LCD1602A I2C Functions ==================== */

// LCD Commands
#define LCD_CMD_CLEAR           0x01
#define LCD_CMD_HOME            0x02
#define LCD_CMD_ENTRY_MODE      0x04
#define LCD_CMD_DISPLAY_CTRL    0x08
#define LCD_CMD_CURSOR_SHIFT    0x10
#define LCD_CMD_FUNCTION_SET    0x20
#define LCD_CMD_SET_CGRAM_ADDR  0x40
#define LCD_CMD_SET_DDRAM_ADDR  0x80

// Flags for display entry mode
#define LCD_ENTRY_RIGHT         0x00
#define LCD_ENTRY_LEFT          0x02
#define LCD_ENTRY_SHIFT_INC     0x01
#define LCD_ENTRY_SHIFT_DEC     0x00

// Flags for display on/off control
#define LCD_DISPLAY_ON          0x04
#define LCD_DISPLAY_OFF         0x00
#define LCD_CURSOR_ON           0x02
#define LCD_CURSOR_OFF          0x00
#define LCD_BLINK_ON            0x01
#define LCD_BLINK_OFF           0x00

// Flags for function set
#define LCD_8BIT_MODE           0x10
#define LCD_4BIT_MODE           0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10_DOTS           0x04
#define LCD_5x8_DOTS            0x00

// Backlight control
#define LCD_BACKLIGHT           0x08
#define LCD_NO_BACKLIGHT        0x00

#define LCD_EN                  0x04  // Enable bit
#define LCD_RW                  0x02  // Read/Write bit
#define LCD_RS                  0x01  // Register select bit

static uint8_t lcd_backlight = LCD_BACKLIGHT;
static uint8_t lcd_actual_addr = 0;
static bool lcd_initialized = false;
static char lcd_last_line[2][17] = {{0}, {0}}; // Last content for resync recovery

static esp_err_t lcd_i2c_write_byte(uint8_t data)
{
    if (i2c_dev_lcd == NULL) return ESP_ERR_INVALID_STATE;

    esp_err_t ret = ESP_FAIL;
    for (int retry = 0; retry < 3 && ret != ESP_OK; retry++) {
        ret = i2c_master_transmit(i2c_dev_lcd, &data, 1, 100);
        if (ret != ESP_OK) vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ret;
}

static void lcd_pulse_enable(uint8_t data)
{
    lcd_i2c_write_byte(data | LCD_EN);
    esp_rom_delay_us(1);  // Enable pulse >450ns
    lcd_i2c_write_byte(data);
    esp_rom_delay_us(50); // Command settle time >37us
}

static esp_err_t lcd_write_nibble(uint8_t nibble, bool is_data)
{
    uint8_t data = nibble | lcd_backlight;
    if (is_data) {
        data |= LCD_RS;
    }
    
    lcd_pulse_enable(data);
    return ESP_OK;
}

static esp_err_t lcd_write_byte(uint8_t byte, bool is_data)
{
    esp_err_t ret;
    
    // Send high nibble
    ret = lcd_write_nibble(byte & 0xF0, is_data);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LCD write high nibble failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Small delay between nibbles
    esp_rom_delay_us(10);
    
    // Send low nibble
    ret = lcd_write_nibble((byte << 4) & 0xF0, is_data);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LCD write low nibble failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t lcd_send_cmd(uint8_t cmd)
{
    return lcd_write_byte(cmd, false);
}

static esp_err_t lcd_send_data(uint8_t data)
{
    return lcd_write_byte(data, true);
}

static esp_err_t lcd_init(void)
{
    esp_err_t ret;
    uint8_t addresses[] = {0x27, 0x3F};
    bool found = false;
    
    // Try both common I2C LCD addresses
    for (int i = 0; i < 2; i++) {
        if (i2c_master_probe(i2c_bus1, addresses[i], 50) == ESP_OK) {
            i2c_device_config_t lcd_dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addresses[i],
                .scl_speed_hz = LCD_I2C_FREQ_HZ,
            };
            ret = i2c_master_bus_add_device(i2c_bus1, &lcd_dev_cfg, &i2c_dev_lcd);
            if (ret == ESP_OK) {
                lcd_actual_addr = addresses[i];
                found = true;
                ESP_LOGI(TAG, "LCD1602A found at address 0x%02X", addresses[i]);
            }
            break;
        }
    }
    
    if (!found) {
        ESP_LOGW(TAG, "LCD1602A not found at 0x27 or 0x3F");
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "Initializing LCD1602A...");
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for LCD to power up
    
    // Initialize in 4-bit mode following HD44780 datasheet
    // Step 1: Wait for power-on, then send 0x03 (Function Set 8-bit) three times
    lcd_write_nibble(0x30, false);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    lcd_write_nibble(0x30, false);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    lcd_write_nibble(0x30, false);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Step 2: Switch to 4-bit mode
    lcd_write_nibble(0x20, false);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Step 3: Now in 4-bit mode, configure the display
    lcd_send_cmd(LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8_DOTS);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    lcd_send_cmd(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_OFF);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    lcd_send_cmd(LCD_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(10));  // Clear needs longer delay
    
    lcd_send_cmd(LCD_CMD_ENTRY_MODE | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DEC);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    lcd_send_cmd(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    lcd_initialized = true;
    ESP_LOGI(TAG, "LCD1602A initialized successfully");
    return ESP_OK;
}

static esp_err_t lcd_clear(void)
{
    if (!lcd_initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = lcd_send_cmd(LCD_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(10));  // Clear command needs longer delay
    return ret;
}

static esp_err_t lcd_set_cursor(uint8_t col, uint8_t row)
{
    if (!lcd_initialized) return ESP_ERR_INVALID_STATE;
    uint8_t row_offsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    if (col > 15) col = 15;
    return lcd_send_cmd(LCD_CMD_SET_DDRAM_ADDR | (col + row_offsets[row]));
}

static esp_err_t lcd_print(const char *str)
{
    if (!lcd_initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = ESP_OK;
    while (*str && ret == ESP_OK) {
        ret = lcd_send_data((uint8_t)*str);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "LCD print failed at char '%c': %s", *str, esp_err_to_name(ret));
            break;
        }
        str++;
    }
    return ret;
}

// Re-syncs HD44780 nibble alignment without clearing the display.
// Called periodically to silently fix any corruption.
static void lcd_resync(void)
{
    if (!lcd_initialized) return;
    
    // Re-run the 4-bit init sequence — this puts the HD44780 back into
    // a known state regardless of how many orphaned nibbles it received.
    lcd_write_nibble(0x30, false); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x30, false); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x30, false); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x20, false); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_cmd(LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8_DOTS);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_cmd(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Immediately redraw last known content so the resync is invisible
    if (lcd_last_line[0][0]) {
        lcd_set_cursor(0, 0);
        lcd_print(lcd_last_line[0]);
    }
    if (lcd_last_line[1][0]) {
        lcd_set_cursor(0, 1);
        lcd_print(lcd_last_line[1]);
    }
}

static void lcd_printf(uint8_t col, uint8_t row, const char *format, ...)
{
    if (!lcd_initialized) return;
    
    char buffer[17];  // 16 chars + null terminator
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // Pad with spaces to always write exactly 16 characters (overwrites old text)
    int len = strlen(buffer);
    for (int i = len; i < 16; i++) {
        buffer[i] = ' ';
    }
    buffer[16] = '\0';
    
    // Save for resync recovery
    if (row < 2) memcpy(lcd_last_line[row], buffer, 17);
    
    esp_err_t ret = lcd_set_cursor(col, row);
    if (ret == ESP_OK) {
        lcd_print(buffer);
    } else {
        ESP_LOGW(TAG, "LCD set cursor failed: %s", esp_err_to_name(ret));
    }
}

/* ==================== Button Handler ==================== */

static void IRAM_ATTR button_isr_handler(void* arg)
{
    uint32_t now = xTaskGetTickCountFromISR();
    // Debounce: ignore presses within 300ms
    if (now - last_button_time > pdMS_TO_TICKS(300)) {
        last_button_time = now;
        display_mode = (display_mode + 1) % 2; // Toggle between 0 and 1
    }
}

static esp_err_t button_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // Trigger on button press (falling edge)
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL));
    
    ESP_LOGI(TAG, "Button initialized on GPIO %d", BUTTON_GPIO);
    return ESP_OK;
}

/* ==================== Helper Functions ==================== */

static void get_ip_string(char *ip_str, size_t max_len)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif != NULL) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            snprintf(ip_str, max_len, "" IPSTR "", IP2STR(&ip_info.ip));
            return;
        }
    }
    snprintf(ip_str, max_len, "No IP");
}

static void get_datetime_string(char *date_str, char *time_str, size_t max_len)
{
    struct tm timeinfo;
    time_t now;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    // Format: "Mar 12, 2026" and "14:35:22"
    strftime(date_str, max_len, "%b %d, %Y", &timeinfo);
    strftime(time_str, max_len, "%H:%M:%S", &timeinfo);
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
    esp_err_t ret = i2c_read_bytes(i2c_dev_rtc, 0x00, data, 7);
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
    ret = i2c_write_byte(i2c_dev_rtc, 0x00, dec_to_bcd(timeinfo->tm_sec));
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(i2c_dev_rtc, 0x01, dec_to_bcd(timeinfo->tm_min));
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(i2c_dev_rtc, 0x02, dec_to_bcd(timeinfo->tm_hour));
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(i2c_dev_rtc, 0x04, dec_to_bcd(timeinfo->tm_mday));
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(i2c_dev_rtc, 0x05, dec_to_bcd(timeinfo->tm_mon + 1));
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(i2c_dev_rtc, 0x06, dec_to_bcd(timeinfo->tm_year - 100));
    
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
static uint8_t mpu6050_actual_addr = 0;  // Store detected address

static esp_err_t mpu6050_init(void)
{
    esp_err_t ret;
    uint8_t addresses[] = {0x69, 0x68};  // Try 0x69 first, then 0x68
    bool found = false;
    
    // Try both possible I2C addresses
    for (int i = 0; i < 2; i++) {
        uint8_t test_addr = addresses[i];

        ESP_LOGI(TAG, "Trying MPU6050 at address 0x%02X...", test_addr);

        // Check if any device ACKs at this address
        if (i2c_master_probe(i2c_bus0, test_addr, 50) != ESP_OK) {
            ESP_LOGW(TAG, "No device found at 0x%02X", test_addr);
            continue;
        }

        // Add a temporary device handle for detection
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = test_addr,
            .scl_speed_hz = I2C_FREQ_HZ,
        };
        i2c_master_dev_handle_t temp_dev = NULL;
        ret = i2c_master_bus_add_device(i2c_bus0, &dev_cfg, &temp_dev);
        if (ret != ESP_OK) continue;

        // Try to wake up the device
        ret = i2c_write_byte(temp_dev, 0x6B, 0x00);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to wake MPU6050 at 0x%02X (error: %d)", test_addr, ret);
            i2c_master_bus_rm_device(temp_dev);
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // Wait for sensor to wake up

        // Try to read WHO_AM_I register (0x75)
        uint8_t who_am_i = 0;
        ret = i2c_read_bytes(temp_dev, 0x75, &who_am_i, 1);

        ESP_LOGI(TAG, "WHO_AM_I read: ret=%d, value=0x%02X", ret, who_am_i);

        // MPU6050 WHO_AM_I should return 0x68, but accept compatible sensors:
        // 0x68 = MPU6050
        // 0x69, 0x72, 0x98 = MPU6050 clones
        // 0x70 = MPU6500 / MPU9250
        // 0x71 = MPU6555
        // 0x73 = MPU9255
        if (ret == ESP_OK && (who_am_i == 0x68 || who_am_i == 0x69 || who_am_i == 0x70 ||
                              who_am_i == 0x71 || who_am_i == 0x72 || who_am_i == 0x73 || who_am_i == 0x98)) {
            // Found compatible MPU sensor — keep the device handle
            mpu6050_actual_addr = test_addr;
            i2c_dev_imu = temp_dev;
            found = true;

            // Identify the sensor type
            const char* sensor_name;
            if (who_am_i == 0x68) sensor_name = "MPU6050";
            else if (who_am_i == 0x70) sensor_name = "MPU6500/MPU9250";
            else if (who_am_i == 0x71) sensor_name = "MPU6555";
            else if (who_am_i == 0x73) sensor_name = "MPU9255";
            else sensor_name = "MPU6050-compatible";

            ESP_LOGI(TAG, "%s found at address 0x%02X (WHO_AM_I=0x%02X)", sensor_name, test_addr, who_am_i);

            if (test_addr == 0x68) {
                ESP_LOGW(TAG, "IMU using same address as RTC (0x68)!");
                ESP_LOGW(TAG, "Consider connecting AD0 pin to 3.3V to use address 0x69");
            }
            break;
        } else {
            if (ret == ESP_OK) {
                ESP_LOGW(TAG, "Device at 0x%02X responded but WHO_AM_I is 0x%02X (unknown sensor)", test_addr, who_am_i);
            }
            i2c_master_bus_rm_device(temp_dev);
        }
    }
    
    if (!found) {
        ESP_LOGE(TAG, "MPU6050/6500 compatible IMU not found at 0x68 or 0x69");
        ESP_LOGE(TAG, "Check wiring: SDA=%d, SCL=%d", I2C_SDA_PIN, I2C_SCL_PIN);
        ESP_LOGE(TAG, "Note: Device at 0x57 is typically DS3231 EEPROM");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Set accelerometer range to ±2g
    ret = i2c_write_byte(i2c_dev_imu, 0x1C, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return ret;
    }
    
    // Set gyroscope range to ±250°/s
    ret = i2c_write_byte(i2c_dev_imu, 0x1B, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return ret;
    }
    
    // Set sample rate (1kHz / (1 + SMPLRT_DIV))
    ret = i2c_write_byte(i2c_dev_imu, 0x19, 0x07);  // 125 Hz
    if (ret != ESP_OK) return ret;
    
    // Configure digital low pass filter
    ret = i2c_write_byte(i2c_dev_imu, 0x1A, 0x03);  // ~44 Hz cutoff
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully at 0x%02X", mpu6050_actual_addr);
    return ret;
}

static esp_err_t mpu6050_read_data(imu_data_t *imu_data)
{
    uint8_t data[14];
    esp_err_t ret = i2c_read_bytes(i2c_dev_imu, 0x3B, data, 14);
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
        esp_err_t ret = i2c_read_bytes(i2c_dev_imu, 0x3B, data, 14);
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
    uint8_t last_display_mode = 0;
    uint32_t lcd_resync_counter = 0;
    
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
            
            // Update LCD with GPS data (now fast with dedicated bus!)
            if (lcd_initialized) {
                // Clear display if mode changed
                if (display_mode != last_display_mode) {
                    lcd_clear();
                    last_display_mode = display_mode;
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                
                if (display_mode == 0) {
                    // GPS/IMU mode
                    lcd_printf(0, 0, "%.1fkm %.1fm", gps_data.speed, gps_data.altitude);
                    lcd_printf(0, 1, "%.2f,%.2f", gps_data.latitude, gps_data.longitude);
                } else {
                    // Time/IP mode
                    char date_str[17], time_str[17], ip_str[17];
                    get_datetime_string(date_str, time_str, sizeof(date_str));
                    get_ip_string(ip_str, sizeof(ip_str));
                    lcd_printf(0, 0, "%s", time_str);
                    lcd_printf(0, 1, "%s", ip_str);
                }
            }
        } else {
            ESP_LOGW(TAG, "Waiting for GPS fix... (accel: %.3f, %.3f, %.3f g)",
                     imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
            
            // Update LCD showing "No GPS" and IMU data (now fast with dedicated bus!)
            if (lcd_initialized) {
                // Clear display if mode changed
                if (display_mode != last_display_mode) {
                    lcd_clear();
                    last_display_mode = display_mode;
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                
                if (display_mode == 0) {
                    // GPS/IMU mode
                    lcd_printf(0, 0, "No GPS Fix");
                    lcd_printf(0, 1, "A:%.2f,%.2f,%.2f", 
                              imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
                } else {
                    // Time/IP mode
                    char date_str[17], time_str[17], ip_str[17];
                    get_datetime_string(date_str, time_str, sizeof(date_str));
                    get_ip_string(ip_str, sizeof(ip_str));
                    lcd_printf(0, 0, "%s", time_str);
                    lcd_printf(0, 1, "%s", ip_str);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Log every second
        
        // Resync LCD every 5 seconds to silently fix any nibble misalignment
        if (lcd_initialized) {
            lcd_resync_counter++;
            if (lcd_resync_counter >= 5) {
                lcd_resync_counter = 0;
                lcd_resync();
            }
        }
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
    
    // Initialize I2C (both buses)
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized on two buses");
    ESP_LOGI(TAG, "  Bus 0 (IMU/RTC): SDA=GPIO%d, SCL=GPIO%d", I2C_SDA_PIN, I2C_SCL_PIN);
    ESP_LOGI(TAG, "  Bus 1 (LCD): SDA=GPIO%d, SCL=GPIO%d", LCD_I2C_SDA_PIN, LCD_I2C_SCL_PIN);
    
    // Scan I2C bus to detect devices
    i2c_scan();
    
    // Initialize LCD1602A
    if (lcd_init() == ESP_OK) {
        lcd_clear();
        lcd_printf(0, 0, "GPS Logger");
        lcd_printf(0, 1, "Initializing...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Initialize button
    button_init();
    ESP_LOGI(TAG, "Press BOOT button to toggle LCD display mode");
    
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
