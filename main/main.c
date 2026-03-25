/*
 * ESP32 GPS Logger with IMU
 * 
 * Features:
 * - GPS tracking with Neo-6M module
 * - IMU data logging with MPU6050/MPU6500/MPU9250
 * - RTC timekeeping with DS3231
 * - SD card storage with CSV files
 * - WiFi connectivity and NTP sync
 * - HTTP server for file access
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_mac.h"
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
#include <math.h>
#include <inttypes.h>

#include "esp_timer.h"
#include "sync_config.h"
#include "sd_storage.h"
#include "wifi_upload.h"
#include "espnow_sync.h"

// WiFi
static const char *TAG = "GPS_LOGGER";
static int s_retry_num __attribute__((unused)) = 0;
static bool time_synced = false;
static bool time_set_from_rtc = false;  // RTC successfully set system time on boot
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
    float hdop;    // horizontal dilution of precision (from GPGGA)
    int   num_sv;  // satellites in use (from GPGGA)
    bool valid;
    struct tm gps_utc_time;  // UTC date+time parsed from GPRMC (fields 1 and 9)
    bool   gps_time_valid;  // true once a complete date+time has been parsed
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

// Shared motion state - written by gps_logging_task, read by sync_state_machine_task
static gps_data_t current_gps = {0};
static imu_data_t current_imu = {0};
static char g_own_mac_str[5] = {0}; // "AABB\0" – last 4 hex digits of WiFi MAC

// Power management state - tracks logging intervals and upload behavior
typedef enum {
    POWER_STATE_MOVING = 0,  // Device moving: 1s logging, full uploads
    POWER_STATE_STAGE1 = 1,  // Static 3+ mins: 1min logging, full uploads
    POWER_STATE_STAGE2 = 2,  // Static 5+ mins: 5min logging, periodic uploads, low power
    POWER_STATE_STAGE3 = 3,  // Static 30+ mins: 30min heartbeat, periodic uploads, parked
} power_state_t;

static volatile power_state_t g_power_state = POWER_STATE_MOVING;

/* ==================== Timezone Detection from GPS ==================== */

static const char* get_timezone_from_gps(float latitude, float longitude)
{
    // Comprehensive timezone detection from GPS coordinates.
    // Political exceptions are tested before generic longitude bands so that
    // regions that do not observe DST (Saskatchewan, Yukon, Hawaii, etc.) are
    // handled correctly even when they share a longitude with DST-observing zones.

    // ── North America (lat 14–83 N, lon -168 to -52) ─────────────────────────
    if (latitude >= 14.0f && latitude <= 83.0f &&
        longitude >= -168.0f && longitude <= -52.0f) {

        // Hawaii — UTC-10, no DST
        if (latitude >= 18.0f && latitude <= 23.0f &&
            longitude >= -162.0f && longitude <= -154.0f)
            return "HST10";

        // Alaska — UTC-9, observes DST
        if (latitude >= 54.0f && longitude <= -130.0f)
            return "AKST9AKDT,M3.2.0,M11.1.0";

        // Newfoundland — UTC-3:30, observes DST
        if (latitude >= 46.0f && longitude >= -59.0f)
            return "NST3:30NDT,M3.2.0/0:01,M11.1.0/0:01";

        // Atlantic Canada (NB, NS, PEI, eastern QC) — UTC-4, observes DST
        if (latitude >= 43.0f && latitude <= 52.0f &&
            longitude >= -66.0f && longitude <= -59.0f)
            return "AST4ADT,M3.2.0,M11.1.0";

        // Saskatchewan — permanent CST (UTC-6), no DST
        if (latitude >= 49.0f && latitude <= 60.0f &&
            longitude >= -110.0f && longitude <= -101.0f)
            return "CST6";

        // Yukon — permanent MST (UTC-7), no DST since 2020
        if (latitude >= 60.0f && longitude <= -124.0f)
            return "MST7";

        // General longitude bands — all observe DST
        // -120° aligns with the BC/Alberta border; -102° with the AB/SK border
        if (longitude < -120.0f) return "PST8PDT,M3.2.0,M11.1.0";  // Pacific
        if (longitude < -102.0f) return "MST7MDT,M3.2.0,M11.1.0";  // Mountain
        if (longitude <  -82.0f) return "CST6CDT,M3.2.0,M11.1.0";  // Central
        return "EST5EDT,M3.2.0/2,M11.1.0";                          // Eastern
    }

    // ── Australia ─────────────────────────────────────────────────────────────
    if (latitude >= -44.0f && latitude <= -10.0f &&
        longitude >= 113.0f && longitude <= 167.0f) {
        // Lord Howe Island — UTC+10:30/+11, half-hour DST
        if (longitude >= 159.0f)
            return "LHST-10:30LHDT-11,M10.1.0,M4.1.0";
        if (longitude < 129.0f) return "AWST-8";                           // Western
        if (longitude < 138.0f) return "ACST-9:30ACDT,M10.1.0,M4.1.0/3"; // Central
        return "AEST-10AEDT,M10.1.0,M4.1.0/3";                            // Eastern
    }

    // ── New Zealand — UTC+12, observes DST ───────────────────────────────────
    if (latitude >= -48.0f && latitude <= -34.0f &&
        longitude >= 165.0f && longitude <= 178.5f)
        return "NZST-12NZDT,M9.5.0,M4.1.0/3";

    // ── Europe: Iceland / Azores — UTC+0, NO DST (checked before main block) ─
    if (latitude >= 35.0f && latitude <= 71.0f &&
        longitude >= -28.0f && longitude < -15.0f)
        return "GMT0";

    // ── Europe ────────────────────────────────────────────────────────────────
    if (latitude >= 35.0f && latitude <= 71.0f &&
        longitude >= -15.0f && longitude <= 40.0f) {
        if (longitude <  7.5f) return "GMT0BST,M3.5.0/1,M10.5.0";   // UK / W. Europe / Portugal
        if (longitude < 22.5f) return "CET-1CEST,M3.5.0,M10.5.0/3"; // Central Europe
        return "EET-2EEST,M3.5.0/3,M10.5.0/4";                       // Eastern Europe
    }

    // ── Israel / Palestine — UTC+2, observes DST ─────────────────────────────
    if (latitude >= 29.0f && latitude <= 34.0f &&
        longitude >= 34.0f && longitude <= 36.5f)
        return "IST-2IDT,M3.4.4/26,M10.5.0";

    // ── Gulf Standard Time — UTC+4, no DST ───────────────────────────────────
    if (latitude >= 12.0f && latitude <= 30.0f &&
        longitude >= 44.0f && longitude <= 60.0f)
        return "GST-4";

    // ── India — UTC+5:30, no DST ──────────────────────────────────────────────
    if (latitude >= 8.0f && latitude <= 37.0f &&
        longitude >= 68.0f && longitude <= 97.0f)
        return "IST-5:30";

    // ── China — UTC+8, no DST ────────────────────────────────────────────────
    if (latitude >= 18.0f && latitude <= 54.0f &&
        longitude >= 73.0f && longitude <= 135.0f)
        return "CST-8";

    // ── Japan — UTC+9, no DST ────────────────────────────────────────────────
    if (latitude >= 24.0f && latitude <= 46.0f &&
        longitude >= 123.0f && longitude <= 146.0f)
        return "JST-9";

    // ── South Africa — UTC+2, no DST ─────────────────────────────────────────
    if (latitude >= -35.0f && latitude <= -22.0f &&
        longitude >= 16.0f && longitude <= 33.0f)
        return "SAST-2";

    // ── Argentina — UTC-3, no DST ────────────────────────────────────────────
    if (latitude >= -55.0f && latitude <= -22.0f &&
        longitude >= -73.0f && longitude <= -53.0f)
        return "ART3";

    // ── Brazil East (São Paulo / Rio / Brasília) — UTC-3, no DST ─────────────
    if (latitude >= -34.0f && latitude <= 5.0f &&
        longitude >= -46.0f && longitude <= -35.0f)
        return "BRT3";

    // ── Generic fallback: UTC offset derived from longitude ───────────────────
    // 15° per hour; bias +7.5° so mid-zone rounds correctly.
    int utc_offset = (int)((longitude + 7.5f) / 15.0f);
    if (utc_offset >  14) utc_offset =  14;
    if (utc_offset < -12) utc_offset = -12;
    static char tz_buf[16];
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
static volatile bool lcd_display_on = false;
static volatile uint32_t lcd_last_activity_time = 0;
static volatile bool lcd_wake_requested = false;

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
    
    lcd_display_on = true;
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

static void lcd_set_power(bool on)
{
    if (!lcd_initialized) return;
    lcd_backlight = on ? LCD_BACKLIGHT : LCD_NO_BACKLIGHT;
    if (on) {
        lcd_send_cmd(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
        lcd_display_on = true;
    } else {
        lcd_send_cmd(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_OFF);
        lcd_i2c_write_byte(LCD_NO_BACKLIGHT);  // Also kill backlight
        lcd_display_on = false;
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
        lcd_last_activity_time = now;
        if (!lcd_display_on) {
            lcd_wake_requested = true;  // Wake LCD; don't toggle mode on first press
        } else {
            display_mode = (display_mode + 1) % 2;
        }
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

        // Accept device if WHO_AM_I read succeeded — any value is treated as a compatible
        // MPU6050-family sensor. Known values:
        //   0x68 = MPU6050, 0x70 = MPU6500/MPU9250, 0x71 = MPU6555,
        //   0x72/0x73/0x98/0x19/0x69 = various clones
        if (ret == ESP_OK) {
            // Found compatible MPU sensor — keep the device handle
            mpu6050_actual_addr = test_addr;
            i2c_dev_imu = temp_dev;
            found = true;

            const char* sensor_name;
            if (who_am_i == 0x68)      sensor_name = "MPU6050";
            else if (who_am_i == 0x70) sensor_name = "MPU6500/MPU9250";
            else if (who_am_i == 0x71) sensor_name = "MPU6555";
            else if (who_am_i == 0x73) sensor_name = "MPU9255";
            else                        sensor_name = "MPU6050-compatible";

            ESP_LOGI(TAG, "%s found at address 0x%02X (WHO_AM_I=0x%02X)", sensor_name, test_addr, who_am_i);

            if (test_addr == 0x68) {
                ESP_LOGW(TAG, "IMU using same address as RTC (0x68)!");
                ESP_LOGW(TAG, "Consider connecting AD0 pin to 3.3V to use address 0x69");
            }
            break;
        } else {
            ESP_LOGW(TAG, "Device at 0x%02X ACKed probe but WHO_AM_I read failed (err %d)", test_addr, ret);
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
    if (i2c_dev_imu == NULL) return ESP_ERR_INVALID_STATE;
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
    // $GPRMC,HHMMSS,A,DDMM.mm,N,DDDMM.mm,E,SSS.S,CCC.C,DDMMYY,MAG,VAR*hh
    //  [0]   [1]   [2] [3]   [4] [5]    [6] [7]  [8]   [9]
    char *token;
    char nmea_copy[128];
    strncpy(nmea_copy, nmea, sizeof(nmea_copy) - 1);
    nmea_copy[sizeof(nmea_copy) - 1] = '\0';

    token = strtok(nmea_copy, ",");
    if (!token || strcmp(token, "$GPRMC") != 0) return false;

    // Field [1]: UTC time  HHMMSS(.ss) — save for date+time struct below
    char time_str[16] = {0};
    token = strtok(NULL, ",");
    if (token) strncpy(time_str, token, sizeof(time_str) - 1);

    // Field [2]: Status A=valid, V=warning
    token = strtok(NULL, ",");
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

    // Field [7]: Speed in knots → km/h
    token = strtok(NULL, ",");
    if (token) {
        gps_data->speed = atof(token) * 1.852f;
    }

    // Field [8]: Course over ground (skip)
    token = strtok(NULL, ",");

    // Field [9]: Date  DDMMYY — combine with saved time_str to build UTC struct tm
    token = strtok(NULL, ",");
    if (token && strlen(token) >= 6 && strlen(time_str) >= 6) {
        struct tm t = {0};
        t.tm_hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
        t.tm_min  = (time_str[2] - '0') * 10 + (time_str[3] - '0');
        t.tm_sec  = (time_str[4] - '0') * 10 + (time_str[5] - '0');
        t.tm_mday = (token[0] - '0') * 10 + (token[1] - '0');
        int mon   = (token[2] - '0') * 10 + (token[3] - '0');  // 1–12
        int yy    = (token[4] - '0') * 10 + (token[5] - '0');  // 0–99
        t.tm_mon  = mon - 1;                 // struct tm: 0 = January
        t.tm_year = (yy < 70 ? 2000 : 1900) + yy - 1900; // years since 1900
        t.tm_isdst = 0;                      // GPS always provides UTC
        gps_data->gps_utc_time  = t;
        gps_data->gps_time_valid = true;
    }

    return true;
}

static bool parse_gpgga(const char *nmea, gps_data_t *gps_data)
{
    // $GPGGA,HHMMSS,Lat,N/S,Lon,E/W,FS,NoSV,HDOP,AltMSL,M,...
    //   FS   = fix status (0=invalid, 1=GPS, 2=DGPS/SBAS)
    //   NoSV = satellites in use
    //   HDOP = horizontal dilution of precision
    char *token;
    char nmea_copy[128];
    strncpy(nmea_copy, nmea, sizeof(nmea_copy) - 1);
    nmea_copy[sizeof(nmea_copy) - 1] = '\0';

    token = strtok(nmea_copy, ",");
    if (!token || strcmp(token, "$GPGGA") != 0) return false;

    token = strtok(NULL, ","); // Time
    token = strtok(NULL, ","); // Latitude (skip — already in RMC)
    token = strtok(NULL, ","); // N/S
    token = strtok(NULL, ","); // Longitude (skip)
    token = strtok(NULL, ","); // E/W

    token = strtok(NULL, ","); // Fix status
    if (!token || token[0] == '0') return false; // no fix

    token = strtok(NULL, ","); // Number of satellites
    if (token) gps_data->num_sv = atoi(token);

    token = strtok(NULL, ","); // HDOP
    if (token) gps_data->hdop = atof(token);

    token = strtok(NULL, ","); // Altitude (MSL)
    if (token) gps_data->altitude = atof(token);

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

// Compute the Fletcher-8 checksum used by all UBX packets.
// The checksum covers every byte from the Class field up to (but not
// including) the two checksum bytes at the end of the message.
static void ubx_checksum(const uint8_t *data, size_t len,
                          uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    for (size_t i = 0; i < len; i++) {
        *ck_a = (uint8_t)(*ck_a + data[i]);
        *ck_b = (uint8_t)(*ck_b + *ck_a);
    }
}

// Send one UBX packet (sync chars + payload + checksum) over GPS_UART_NUM.
static void ubx_send(uint8_t *msg, size_t len)
{
    // msg[] layout: 0xB5, 0x62, class, id, lenL, lenH, payload..., 0x00, 0x00
    // Checksum covers bytes [2 .. len-3].
    ubx_checksum(&msg[2], len - 4, &msg[len - 2], &msg[len - 1]);
    uart_write_bytes(GPS_UART_NUM, (const char *)msg, (int)len);
}

// Wait for a UBX-ACK-ACK or UBX-ACK-NAK response to the command identified
// by (ack_cls, ack_id).  Returns true on ACK-ACK, false on ACK-NAK or timeout.
// The module may still be emitting NMEA before the ACK arrives so we scan the
// raw byte stream for the UBX sync preamble rather than assuming clean framing.
static bool ubx_wait_ack(uint8_t ack_cls, uint8_t ack_id)
{
    // UBX-ACK packet layout (10 bytes total):
    //   B5 62  — sync
    //   05 xx  — class 0x05, id 0x01=ACK-ACK / 0x00=ACK-NAK
    //   02 00  — payload length = 2
    //   cc ii  — clsID and msgID of the acknowledged message
    //   CK_A CK_B
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(300);
    uint8_t buf[1];
    uint8_t state = 0; // simple byte-by-byte state machine

    // Byte positions we care about while scanning:
    //  state 0: looking for 0xB5
    //  state 1: looking for 0x62
    //  state 2: looking for class 0x05
    //  state 3: ACK id byte (0x01 or 0x00)
    //  state 4: payload len low  (must be 0x02)
    //  state 5: payload len high (must be 0x00)
    //  state 6: clsID of acked msg
    //  state 7: msgID of acked msg
    uint8_t ack_type = 0;

    while (xTaskGetTickCount() < deadline) {
        int n = uart_read_bytes(GPS_UART_NUM, buf, 1, pdMS_TO_TICKS(10));
        if (n <= 0) continue;

        switch (state) {
            case 0: if (buf[0] == 0xB5) state = 1; break;
            case 1: state = (buf[0] == 0x62) ? 2 : (buf[0] == 0xB5 ? 1 : 0); break;
            case 2: state = (buf[0] == 0x05) ? 3 : (buf[0] == 0xB5 ? 1 : 0); break;
            case 3:
                if (buf[0] == 0x01 || buf[0] == 0x00) {
                    ack_type = buf[0]; // 0x01 = ACK, 0x00 = NAK
                    state = 4;
                } else {
                    state = (buf[0] == 0xB5) ? 1 : 0;
                }
                break;
            case 4: state = (buf[0] == 0x02) ? 5 : (buf[0] == 0xB5 ? 1 : 0); break;
            case 5: state = (buf[0] == 0x00) ? 6 : (buf[0] == 0xB5 ? 1 : 0); break;
            case 6:
                if (buf[0] == ack_cls) { state = 7; }
                else                   { state = (buf[0] == 0xB5) ? 1 : 0; }
                break;
            case 7:
                if (buf[0] == ack_id) {
                    return (ack_type == 0x01); // true = ACK-ACK
                }
                state = (buf[0] == 0xB5) ? 1 : 0;
                break;
            default: state = 0; break;
        }
    }
    return false; // timeout
}

// Configure the NEO-6M for 5 Hz updates and disable the NMEA sentences that
// the parser never reads (GLL, GSA, GSV, VTG).  Only GGA and RMC are kept,
// which keeps the UART load comfortably within 9600 baud at 5 Hz.
//
// Changes take effect immediately but are lost on power-cycle unless the
// module has battery-backed RAM and a CFG-CFG save is sent.
static void gps_configure_5hz(void)
{
    // A brief warm-up delay lets the module finish its startup NMEA burst
    // before we send UBX config, reducing the chance of a NACK.
    vTaskDelay(pdMS_TO_TICKS(500));

    // ── Disable unneeded NMEA sentences ──────────────────────────────────────
    // UBX-CFG-MSG (3-byte payload): msgClass, msgId, rate
    //   rate = 0  → disabled on all ports
    //   rate = 1  → enabled  on all ports
    // Sentences we turn off: GLL (F0 01), GSA (F0 02), GSV (F0 03), VTG (F0 05)
    static const struct { uint8_t cls; uint8_t id; const char *name; } disable[] = {
        { 0xF0, 0x01, "GLL" },
        { 0xF0, 0x02, "GSA" },
        { 0xF0, 0x03, "GSV" },
        { 0xF0, 0x05, "VTG" },
    };
    for (int i = 0; i < (int)(sizeof(disable) / sizeof(disable[0])); i++) {
        uint8_t msg[] = {
            0xB5, 0x62,                       // UBX sync
            0x06, 0x01,                       // CFG-MSG
            0x03, 0x00,                       // payload length = 3
            disable[i].cls, disable[i].id,    // sentence class / id
            0x00,                             // rate = 0 (disabled)
            0x00, 0x00                        // CK_A, CK_B (filled by ubx_send)
        };
        ubx_send(msg, sizeof(msg));
        if (ubx_wait_ack(0x06, 0x01)) {
            ESP_LOGI(TAG, "NEO-6M: disabled %s (ACK-ACK)", disable[i].name);
        } else {
            ESP_LOGE(TAG, "NEO-6M: disable %s failed (ACK-NAK or timeout)", disable[i].name);
        }
    }

    // ── Set 5 Hz navigation update rate ──────────────────────────────────────
    // UBX-CFG-RATE payload:
    //   measRate = 200 ms (0x00C8) — measurement period
    //   navRate  = 1      (0x0001) — navigation cycles per measurement
    //   timeRef  = 1      (0x0001) — GPS time reference
    uint8_t rate_msg[] = {
        0xB5, 0x62,              // UBX sync
        0x06, 0x08,              // CFG-RATE
        0x06, 0x00,              // payload length = 6
        0xC8, 0x00,              // measRate = 200 ms
        0x01, 0x00,              // navRate  = 1
        0x01, 0x00,              // timeRef  = 1 (GPS time)
        0x00, 0x00               // CK_A, CK_B (filled by ubx_send)
    };
    ubx_send(rate_msg, sizeof(rate_msg));
    if (ubx_wait_ack(0x06, 0x08)) {
        ESP_LOGI(TAG, "NEO-6M: 5 Hz rate set (ACK-ACK)");
    } else {
        ESP_LOGE(TAG, "NEO-6M: 5 Hz rate set failed (ACK-NAK or timeout)");
    }

    // ── Navigation engine: pedestrian model + static hold ────────────────────
    // CFG-NAV5 (0x06 0x24), 36-byte payload.
    // mask = 0x0041: apply dynModel (bit 0) and staticHoldThresh (bit 6) only.
    // dynModel = 3 (pedestrian) improves stationary accuracy vs. the default
    //   portable (0) model by tightening the receiver's motion model.
    // staticHoldThresh = GPS_STATIC_HOLD_CM_S: below this speed the receiver
    //   freezes its reported position to suppress multipath wander.
    uint8_t nav5_msg[] = {
        0xB5, 0x62,              // UBX sync
        0x06, 0x24,              // CFG-NAV5
        0x24, 0x00,              // payload length = 36
        0x41, 0x00,              // mask = 0x0041 (dynModel + staticHold)
        0x03,                    // dynModel = 3 (pedestrian)
        0x02,                    // fixMode  = 2 (not applied by mask)
        0x00, 0x00, 0x00, 0x00,  // fixedAlt
        0x10, 0x27, 0x00, 0x00,  // fixedAltVar
        0x05,                    // minElev  = 5°
        0x00,                    // drLimit
        0xFA, 0x00,              // pDop = 25.0
        0xFA, 0x00,              // tDop = 25.0
        0x64, 0x00,              // pAcc = 100 m
        0x2C, 0x01,              // tAcc = 300 m
        GPS_STATIC_HOLD_CM_S,    // staticHoldThresh (cm/s)
        0x00,                    // dgpsTimeOut
        0x00, 0x00, 0x00, 0x00,  // reserved2
        0x00, 0x00, 0x00, 0x00,  // reserved3
        0x00, 0x00, 0x00, 0x00,  // reserved4
        0x00, 0x00               // CK_A, CK_B
    };
    ubx_send(nav5_msg, sizeof(nav5_msg));
    if (ubx_wait_ack(0x06, 0x24)) {
        ESP_LOGI(TAG, "NEO-6M: pedestrian model + static hold set (ACK-ACK)");
    } else {
        ESP_LOGE(TAG, "NEO-6M: CFG-NAV5 failed (ACK-NAK or timeout)");
    }

    // ── SBAS (WAAS / EGNOS / MSAS / GAGAN) ───────────────────────────────────
    // CFG-SBAS (0x06 0x16), 8-byte payload.
    // mode  = 0x07: enable + ranging + differential corrections
    // usage = 0x03: use SBAS for ranging and differential corrections
    // scanmode1 = 0: auto-detect all available SBAS PRNs
    uint8_t sbas_msg[] = {
        0xB5, 0x62,              // UBX sync
        0x06, 0x16,              // CFG-SBAS
        0x08, 0x00,              // payload length = 8
        0x07,                    // mode: enable + ranging + correction
        0x03,                    // usage: ranging + diffCorr
        0x03,                    // maxSBAS = 3 channels
        0x00,                    // scanmode2
        // scanmode1: PRN bitmask derived from GPS_SBAS_REGION in sync_config.h.
        // Bit N = PRN (120+N).  0x00000000 = auto-scan all.
        (uint8_t)((GPS_SBAS_REGION)        & 0xFF),
        (uint8_t)((GPS_SBAS_REGION >>  8)  & 0xFF),
        (uint8_t)((GPS_SBAS_REGION >> 16)  & 0xFF),
        (uint8_t)((GPS_SBAS_REGION >> 24)  & 0xFF),
        0x00, 0x00               // CK_A, CK_B
    };
    ubx_send(sbas_msg, sizeof(sbas_msg));
    if (ubx_wait_ack(0x06, 0x16)) {
        ESP_LOGI(TAG, "NEO-6M: SBAS enabled, region mask=0x%08lX (ACK-ACK)",
                 (unsigned long)GPS_SBAS_REGION);
    } else {
        ESP_LOGE(TAG, "NEO-6M: SBAS enable failed (ACK-NAK or timeout)");
    }
}

/* ==================== SD Card Functions ==================== */

static esp_err_t sd_card_init(void)
{
    esp_err_t ret;
    
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
    slot_config.d0 = SDMMC_D0_GPIO;
    
    ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card (%s)", esp_err_to_name(ret));
        }
        return ret;
    }
    
    sdmmc_card_print_info(stdout, card);
    
    return ESP_OK;
}

/* ==================== CSV Logging Functions ==================== */

static esp_err_t log_to_csv(const gps_data_t *gps_data, const imu_data_t *imu_data)
{
    // Skip logging if GPS location is invalid or at 0,0
    bool gps_valid = gps_data->valid;
    if (!gps_valid || (gps_data->latitude == 0.0 && gps_data->longitude == 0.0)) {
        return ESP_OK; // Don't log invalid positions
    }

    // Skip duplicate positions: if lat, lon AND altitude are all identical to
    // the last logged entry the device has not moved — no new information to store.
    // Exception: always write a heartbeat entry at least once per minute so
    // there is a continuous presence record even during long static periods.
    static float   last_lat = 0.0f, last_lon = 0.0f, last_alt = 0.0f;
    static int64_t last_write_ts = 0;
    #define HEARTBEAT_INTERVAL_S  60
    int64_t now_ts = (int64_t)time(NULL);
    bool position_unchanged = (gps_data->latitude  == last_lat &&
                               gps_data->longitude == last_lon &&
                               gps_data->altitude  == last_alt);
    bool heartbeat_due = (now_ts - last_write_ts) >= HEARTBEAT_INTERVAL_S;
    if (position_unchanged && !heartbeat_due) {
        return ESP_OK;
    }

    // Position unchanged → speed must be zero (GPS noise guard).
    float logged_speed = position_unchanged ? 0.0f : gps_data->speed;

    char row[128];
    snprintf(row, sizeof(row),
             "%" PRId64 ",%.6f,%.6f,%.1f,%.1f,%.4f,%.4f,%.4f,%s\n",
             (int64_t)time(NULL),
             gps_data->latitude,
             gps_data->longitude,
             gps_data->altitude,
             logged_speed,
             imu_data->accel_x,
             imu_data->accel_y,
             imu_data->accel_z,
             g_own_mac_str);

    if (!sd_append_record(row)) return ESP_FAIL;

    // Only update the watermark after a successful write
    last_lat = gps_data->latitude;
    last_lon = gps_data->longitude;
    last_alt = gps_data->altitude;
    last_write_ts = now_ts;
    return ESP_OK;
}

/* ==================== WiFi Functions ==================== */

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Enable WiFi power save mode for better battery life
    // WIFI_PS_MIN_MODEM allows ESP-NOW to work while saving power
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_LOGI(TAG, "WiFi hardware initialised (STA mode)");
}

/* ==================== Portable timegm ==================== */
// Converts a UTC struct tm to time_t without relying on the non-standard timegm().
// Temporarily overrides TZ to UTC0 so mktime() treats the struct as UTC.
static time_t portable_timegm(struct tm *utc_tm)
{
    char saved_tz[64] = {0};
    const char *cur = getenv("TZ");
    if (cur) strncpy(saved_tz, cur, sizeof(saved_tz) - 1);
    setenv("TZ", "UTC0", 1);
    tzset();
    struct tm tmp = *utc_tm;
    tmp.tm_isdst = 0;
    time_t result = mktime(&tmp);
    if (saved_tz[0]) setenv("TZ", saved_tz, 1);
    else             unsetenv("TZ");
    tzset();
    return result;
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

// Emit a single table row with a download link; silently skips if file absent.
static void http_emit_file_row(httpd_req_t *req, const char *display, const char *full_path)
{
    struct stat st;
    if (stat(full_path, &st) != 0) return;
    char buf[640];
    snprintf(buf, sizeof(buf),
             "<tr><td><a href='/download?path=%s'>%s</a></td>"
             "<td style='padding-left:16px;color:#888'>%ld B</td></tr>",
             full_path, display, st.st_size);
    httpd_resp_sendstr_chunk(req, buf);
}

static esp_err_t http_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr_chunk(req,
        "<html><head><title>GPS Logger</title>"
        "<style>body{font-family:monospace;padding:16px;max-width:700px}"
        "h1{border-bottom:1px solid #ccc;padding-bottom:8px}"
        "h2{margin-top:24px;color:#444}table{border-collapse:collapse;width:100%}"
        "td{padding:4px 0}a{color:#0066cc}"
        ".danger{background:#c0392b;color:#fff;border:none;padding:8px 16px;"
        "font-family:monospace;font-size:14px;cursor:pointer;border-radius:3px}"
        ".danger:hover{background:#a93226}"
        ".action{background:#2980b9;color:#fff;border:none;padding:8px 16px;"
        "font-family:monospace;font-size:14px;cursor:pointer;border-radius:3px;margin-right:8px}"
        ".action:hover{background:#1a6fa0}"
        ".warn{background:#e67e22;color:#fff;border:none;padding:8px 16px;"
        "font-family:monospace;font-size:14px;cursor:pointer;border-radius:3px;margin-right:8px}"
        ".warn:hover{background:#ca6f1e}</style></head><body>");
    httpd_resp_sendstr_chunk(req, "<h1>GPS Logger</h1>");

    // Device identity line
    char mac_line[96];
    snprintf(mac_line, sizeof(mac_line),
             "<p style='color:#555;margin-top:0'>Device MAC: <strong>%s</strong></p>",
             g_own_mac_str[0] ? g_own_mac_str : "unknown");
    httpd_resp_sendstr_chunk(req, mac_line);

    // \u2500\u2500 Sync files \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
    httpd_resp_sendstr_chunk(req, "<h2>Sync Files</h2><table>");
    http_emit_file_row(req, "sync_data.csv",   CSV_DATA_FILE);
    http_emit_file_row(req, "sync_merged.csv", CSV_MERGED_FILE);
    httpd_resp_sendstr_chunk(req, "</table>");

    // ── Backup files (kept after successful upload for verification) ───────────
    httpd_resp_sendstr_chunk(req, "<h2>Backup Files</h2><table>");
    {
        DIR *d = opendir(SD_MOUNT_POINT);
        if (d) {
            struct dirent *ent;
            while ((ent = readdir(d)) != NULL) {
                size_t len = strlen(ent->d_name);
                if (len > 4 && strcmp(ent->d_name + len - 4, ".bak") == 0) {
                    char full[272]; /* /sdcard/ (8) + 255-char name + NUL */
                    snprintf(full, sizeof(full), SD_MOUNT_POINT "/%s", ent->d_name);
                    http_emit_file_row(req, ent->d_name, full);
                }
            }
            closedir(d);
        }
    }
    httpd_resp_sendstr_chunk(req, "</table>");

    
    httpd_resp_sendstr_chunk(req, "<h2>Connection Log</h2><table>");
    http_emit_file_row(req, "connections.csv", CONNECTIONS_LOG_FILE);
    httpd_resp_sendstr_chunk(req, "</table>");

    
    httpd_resp_sendstr_chunk(req,
        "<h2>Actions</h2>"
        "<form method='POST' action='/reupload_bak' style='display:inline'>"
        "<button type='submit' class='action'>Re-upload backups</button>"
        "</form>"
        "<form method='POST' action='/clear_bak' style='display:inline' "
        "onsubmit=\"return confirm('Delete all backup (.bak) files?');\">"
        "<button type='submit' class='warn'>Clear backups</button>"
        "</form>"
        "<br><br>"
        "<button type='submit' class='danger'>Clear all data</button>"
        "</form>");

    httpd_resp_sendstr_chunk(req, "</body></html>");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t http_download_handler(httpd_req_t *req)
{
    char query[512];
    char raw_path[384];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "path", raw_path, sizeof(raw_path)) != ESP_OK) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Security: reject path traversal sequences
    if (strstr(raw_path, "..") != NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid path");
        return ESP_FAIL;
    }

    // Security: path must be within the SD mount point
    if (strncmp(raw_path, SD_MOUNT_POINT, strlen(SD_MOUNT_POINT)) != 0) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Access denied");
        return ESP_FAIL;
    }

    FILE *f = fopen(raw_path, "r");
    if (f == NULL) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/csv");

    // Suggest a download filename for the browser
    const char *basename = strrchr(raw_path, '/');
    if (basename) {
        char cd[160];
        snprintf(cd, sizeof(cd), "attachment; filename=\"%s\"", basename + 1);
        httpd_resp_set_hdr(req, "Content-Disposition", cd);
    }

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

static esp_err_t http_clear_handler(httpd_req_t *req)
{
    // Truncate sync files back to header-only
    sd_delete_data_file();
    sd_ensure_data_file();
    sd_delete_merged_file();
    sd_ensure_merged_file();
    espnow_sync_reset_peer_states();

    // Delete connection log
    remove(CONNECTIONS_LOG_FILE);

    ESP_LOGI(TAG, "All log files cleared via web interface");

    // Redirect back to the index page
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_sendstr(req, "");
    return ESP_OK;
}

static esp_err_t http_reupload_bak_handler(httpd_req_t *req)
{
    if (!wifi_is_connected()) {
        if (!wifi_connect()) {
            httpd_resp_set_type(req, "text/html");
            httpd_resp_sendstr(req,
                "<html><body><p>WiFi connection failed — cannot re-upload.</p>"
                "<a href='/'>Back</a></body></html>");
            return ESP_OK;
        }
    }

    int uploaded = 0;
    bool ok = wifi_reupload_bak_files(&uploaded);

    char msg[512];
    httpd_resp_set_type(req, "text/html");
    if (ok) {
        snprintf(msg, sizeof(msg),
                 "<html><body><p>Re-uploaded %d backup file(s) successfully.</p>"
                 "<a href='/'>Back</a></body></html>", uploaded);
    } else if (uploaded > 0) {
        snprintf(msg, sizeof(msg),
                 "<html><body><p>Partial: %d file(s) uploaded but some failed. "
                 "Failed files are preserved for retry.</p>"
                 "<a href='/'>Back</a></body></html>", uploaded);
    } else {
        snprintf(msg, sizeof(msg),
                 "<html><body><p>Re-upload failed or no backup files found.</p>"
                 "<a href='/'>Back</a></body></html>");
    }
    httpd_resp_sendstr(req, msg);
    return ESP_OK;
}

static esp_err_t http_clear_bak_handler(httpd_req_t *req)
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
             "<html><body><p>Deleted %d backup file(s).</p>"
             "<a href='/'>Back</a></body></html>", deleted);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, msg);
    return ESP_OK;
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

        httpd_uri_t uri_clear = {
            .uri       = "/clear",
            .method    = HTTP_POST,
            .handler   = http_clear_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &uri_clear);

        httpd_uri_t uri_reupload_bak = {
            .uri       = "/reupload_bak",
            .method    = HTTP_POST,
            .handler   = http_reupload_bak_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &uri_reupload_bak);

        httpd_uri_t uri_clear_bak = {
            .uri       = "/clear_bak",
            .method    = HTTP_POST,
            .handler   = http_clear_bak_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &uri_clear_bak);

        ESP_LOGI(TAG, "HTTP server started");
    }
    
    return server;
}

/* ==================== Main GPS Logging Task ==================== */

// Forward declaration for power management function
static void update_power_mode(power_state_t new_state, power_state_t old_state);

static void lcd_update_display(uint8_t *last_mode, const gps_data_t *gps,
                                const imu_data_t *imu, bool has_fix)
{
    if (!lcd_initialized || !lcd_display_on) return;
    if (display_mode != *last_mode) {
        lcd_clear();
        *last_mode = display_mode;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (display_mode == 0) {
        if (has_fix) {
            lcd_printf(0, 0, "%.1fkm %.1fm", gps->speed, gps->altitude);
            lcd_printf(0, 1, "%.4f,%.4f", gps->latitude, gps->longitude);
        } else {
            lcd_printf(0, 0, "No GPS Fix");
            lcd_printf(0, 1, "A:%.2f,%.2f,%.2f", imu->accel_x, imu->accel_y, imu->accel_z);
        }
    } else {
        char date_str[17], time_str[17], ip_str[17];
        get_datetime_string(date_str, time_str, sizeof(date_str));
        get_ip_string(ip_str, sizeof(ip_str));
        lcd_printf(0, 0, "%s", time_str);
        lcd_printf(0, 1, "%s", ip_str);
    }
}

static void gps_logging_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    char line_buffer[128];
    int line_pos = 0;
    
    gps_data_t gps_data = {0};
    imu_data_t imu_data = {0};
    uint8_t last_display_mode = 0;
    uint32_t lcd_resync_counter = 0;

    // EMA filter state — smooths GPS lat/lon/alt while the device is stationary
    // double accumulators avoid float precision loss (~0.85 m/bit at mid-latitudes)
    double ema_lat = 0.0, ema_lon = 0.0;
    float  ema_alt = 0.0f;
    bool   ema_initialized = false;

    // Adaptive logging interval state — reduces log rate when stationary
    uint32_t stationary_time_ms = 0;
    uint32_t current_log_interval_ms = LOG_INTERVAL_MOVING_MS;
    bool was_stationary = false;
    // Debounce for motion detection — require this many consecutive "moving" log
    // ticks before resetting stationary_time_ms.  Prevents GPS speed noise spikes
    // from restarting the 3-min / 5-min accumulation.
    uint8_t  motion_confirm_count = 0;
    #define MOTION_CONFIRM_TICKS  5   // ~5 s at 1 s log rate

    // Sub-second IMU accumulator — IMU is sampled at GPS_SAMPLE_PERIOD_MS and
    // averaged over the full log interval for more stable motion detection.
    float      imu_sum_ax = 0.0f, imu_sum_ay = 0.0f, imu_sum_az = 0.0f;
    float      imu_sum_gx = 0.0f, imu_sum_gy = 0.0f, imu_sum_gz = 0.0f;
    uint32_t   imu_sample_count = 0;
    TickType_t last_log_tick = 0;
    // Persistent filtered position — updated on each GPS fix, used for CSV / LCD
    gps_data_t log_gps = {0};
    // Track whether a new GPS fix arrived this loop iteration
    bool gps_fix_updated = false;
    // Last known IMU static state — computed at log rate but applied at GPS rate
    bool imu_static_last = false;
    // Position lock — once the device has been continuously stationary for
    // GPS_RELOCK_TIME_STAGE1_MS the EMA position is frozen and all subsequent
    // logged rows use it unchanged.  lock_release_tick records when the lock
    // was last released so the re-lock window is time-based, not tick-based.
    TickType_t lock_release_tick = 0;   // xTaskGetTickCount() when lock last dropped
    bool       position_locked   = false;
    float      locked_lat_f      = 0.0f;
    float      locked_lon_f      = 0.0f;
    float      locked_alt_f      = 0.0f;
    // Fast motion exit counter — ticks every GPS_SAMPLE_PERIOD_MS (200 ms).
    // When Stage1 or Stage2 is active, movement is detected at full GPS rate
    // without waiting for the slow log-tick.  MOTION_CONFIRM_TICKS consecutive
    // moving samples = ~1 s at 200 ms/tick.
    uint8_t fast_motion_ticks = 0;
    
    ESP_LOGI(TAG, "GPS logging task started");
    
    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, data, UART_BUF_SIZE, pdMS_TO_TICKS(GPS_SAMPLE_PERIOD_MS));
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];
                
                if (c == '\n' || c == '\r') {
                    if (line_pos > 0) {
                        line_buffer[line_pos] = '\0';
                        
                        // Parse NMEA sentences
                        if (strncmp(line_buffer, "$GPRMC", 6) == 0) {
                            parse_gprmc(line_buffer, &gps_data);
                            if (gps_data.valid) gps_fix_updated = true;
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
        if (imu_ret != ESP_OK && imu_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGD(TAG, "Failed to read IMU data: %s", esp_err_to_name(imu_ret));
        }

        // Accumulate IMU at GPS_SAMPLE_PERIOD_MS rate for averaging over the log interval
        if (imu_ret == ESP_OK) {
            imu_sum_ax += imu_data.accel_x;
            imu_sum_ay += imu_data.accel_y;
            imu_sum_az += imu_data.accel_z;
            imu_sum_gx += imu_data.gyro_x;
            imu_sum_gy += imu_data.gyro_y;
            imu_sum_gz += imu_data.gyro_z;
            imu_sample_count++;
        }

        // Update shared motion state used by the ESP-NOW sync task
        current_gps = gps_data;
        current_imu = imu_data;

        // Handle LCD wake request (button pressed while LCD was off)
        if (lcd_initialized && lcd_wake_requested) {
            lcd_wake_requested = false;
            lcd_set_power(true);
            last_display_mode = 0xFF;  // Force full redraw on next update
        }

        // Auto-off: power down LCD after LCD_TIMEOUT_SEC seconds of inactivity
#if LCD_TIMEOUT_SEC > 0
        if (lcd_initialized && lcd_display_on) {
            uint32_t now_ticks = xTaskGetTickCount();
            if (now_ticks - lcd_last_activity_time > pdMS_TO_TICKS((uint32_t)LCD_TIMEOUT_SEC * 1000UL)) {
                lcd_set_power(false);
            }
        }
#endif

        // ── Per-fix EMA update (runs at GPS rate, up to 5 Hz) ────────────────
        // Update the position filter on every new GPS fix so the EMA has the
        // maximum number of samples to converge on, rather than only once per
        // log interval.  imu_static_last is kept current by the log-tick block.
        if (gps_fix_updated && gps_data.valid) {
            gps_fix_updated = false;

            // Fix quality gate: skip fixes with poor geometry or too few SVs.
            // Both conditions must be checked together so a bad HDOP fix cannot
            // pollute the EMA even when the outlier distance check would pass.
            if (gps_data.hdop > GPS_MAX_HDOP || gps_data.num_sv < GPS_MIN_SV) {
                ESP_LOGD(TAG, "GPS fix skipped: HDOP=%.1f SVs=%d",
                         gps_data.hdop, gps_data.num_sv);
            } else {
                // ── EMA filter ────────────────────────────────────────────────
                if (!ema_initialized) {
                    ema_lat       = gps_data.latitude;
                    ema_lon       = gps_data.longitude;
                    ema_alt       = gps_data.altitude;
                    ema_initialized = true;
                } else if (imu_static_last && !position_locked) {
                    double dlat    = gps_data.latitude  - ema_lat;
                    double dlon    = gps_data.longitude - ema_lon;
                    double cos_lat = cos(ema_lat * M_PI / 180.0);
                    double dist_m  = sqrt((dlat * dlat + dlon * dlon * cos_lat * cos_lat)
                                          * 111120.0 * 111120.0);
                    if (dist_m < GPS_OUTLIER_REJECT_M) {
                        ema_lat = GPS_EMA_ALPHA * gps_data.latitude  + (1.0 - GPS_EMA_ALPHA) * ema_lat;
                        ema_lon = GPS_EMA_ALPHA * gps_data.longitude + (1.0 - GPS_EMA_ALPHA) * ema_lon;
                        ema_alt = GPS_EMA_ALPHA * gps_data.altitude  + (1.0f - GPS_EMA_ALPHA) * ema_alt;
                    } else {
                        ESP_LOGD(TAG, "GPS outlier rejected: %.1f m from EMA", (float)dist_m);
                    }
                } else {
                    // Moving — snap filter to raw position so tracking stays responsive
                    ema_lat = gps_data.latitude;
                    ema_lon = gps_data.longitude;
                    ema_alt = gps_data.altitude;
                }

                // Keep log_gps current at GPS rate so the log-tick always has
                // the freshest filtered position ready to write.  Bad-quality
                // fixes do not update log_gps so the last good position persists.
                log_gps = gps_data;
                if (position_locked) {
                    // Position lock active: freeze coordinates to the locked mean.
                    log_gps.latitude  = locked_lat_f;
                    log_gps.longitude = locked_lon_f;
                    log_gps.altitude  = locked_alt_f;
                } else if (imu_static_last) {
                    log_gps.latitude  = (float)ema_lat;
                    log_gps.longitude = (float)ema_lon;
                    log_gps.altitude  = ema_alt;
                }
            }
        }

        // ── Fast motion detection (runs every GPS_SAMPLE_PERIOD_MS) ───────────
        // When the device is in Stage1 or Stage2 the log tick fires too
        // infrequently (1 min / 5 min) to detect movement quickly.  This block
        // uses instantaneous IMU + GPS speed readings to exit the power-saving
        // states immediately without waiting for the next slow log tick.
        if (g_power_state != POWER_STATE_MOVING && gps_data.valid) {
            float fam = sqrtf(imu_data.accel_x * imu_data.accel_x +
                              imu_data.accel_y * imu_data.accel_y +
                              imu_data.accel_z * imu_data.accel_z);
            float fgm = sqrtf(imu_data.gyro_x  * imu_data.gyro_x  +
                              imu_data.gyro_y  * imu_data.gyro_y  +
                              imu_data.gyro_z  * imu_data.gyro_z);
            // When position is locked, GPS speed is unreliable (~1 km/h static
            // noise) — use IMU only so a parked device cannot self-wake on noise.
            bool fast_moving = (!position_locked && (gps_data.speed >= GPS_STATIC_SPEED_KMH)) ||
                               (fabsf(fam - 1.0f) >= IMU_ACCEL_DEVIATION) ||
                               (fgm >= IMU_GYRO_STATIC_DPS);
            if (fast_moving) {
                fast_motion_ticks++;
                if (fast_motion_ticks >= MOTION_CONFIRM_TICKS) {
                    power_state_t old_state  = g_power_state;
                    g_power_state            = POWER_STATE_MOVING;
                    current_log_interval_ms  = LOG_INTERVAL_MOVING_MS;
                    stationary_time_ms       = 0;
                    motion_confirm_count     = 0;
                    was_stationary           = false;
                    if (position_locked) {
                        position_locked   = false;
                        lock_release_tick = xTaskGetTickCount();
                        ESP_LOGI(TAG, "GPS position lock released (fast motion)");
                    }
                    // Snap EMA to raw position so tracking resumes responsively
                    ema_lat = gps_data.latitude;
                    ema_lon = gps_data.longitude;
                    ema_alt = gps_data.altitude;
                    update_power_mode(g_power_state, old_state);
                    fast_motion_ticks = 0;
                }
            } else {
                fast_motion_ticks = 0;
            }
        } else {
            fast_motion_ticks = 0;
        }

        // ── Log tick gate ─────────────────────────────────────────────────────
        // The loop runs at GPS_SAMPLE_PERIOD_MS so the IMU is sampled at ~10 Hz.
        // The CSV write, EMA filter, and power management only fire when the
        // log interval has elapsed.
        {
            TickType_t now_tick = xTaskGetTickCount();
            if ((TickType_t)(now_tick - last_log_tick) < pdMS_TO_TICKS(current_log_interval_ms)) {
                continue;
            }
            last_log_tick = now_tick;
        }

        // ── Average accumulated IMU samples for this log interval ─────────────
        // Falls back to the most recent instant reading if no samples accumulated.
        imu_data_t avg_imu = imu_data;
        if (imu_sample_count > 0) {
            avg_imu.accel_x = imu_sum_ax / (float)imu_sample_count;
            avg_imu.accel_y = imu_sum_ay / (float)imu_sample_count;
            avg_imu.accel_z = imu_sum_az / (float)imu_sample_count;
            avg_imu.gyro_x  = imu_sum_gx / (float)imu_sample_count;
            avg_imu.gyro_y  = imu_sum_gy / (float)imu_sample_count;
            avg_imu.gyro_z  = imu_sum_gz / (float)imu_sample_count;
            imu_sum_ax = imu_sum_ay = imu_sum_az = 0.0f;
            imu_sum_gx = imu_sum_gy = imu_sum_gz = 0.0f;
            imu_sample_count = 0;
        }

        // ── GPS position jitter filter ────────────────────────────────────────
        // The EMA is now updated at GPS rate (up to 5 Hz) in the per-fix block
        // above.  Here we only (re-)compute imu_static so the per-fix block has
        // an up-to-date gate value for the next batch of fixes.
        bool stationary = false;

        if (gps_data.valid) {
            float am = sqrtf(avg_imu.accel_x * avg_imu.accel_x +
                             avg_imu.accel_y * avg_imu.accel_y +
                             avg_imu.accel_z * avg_imu.accel_z);
            float gm = sqrtf(avg_imu.gyro_x  * avg_imu.gyro_x  +
                             avg_imu.gyro_y  * avg_imu.gyro_y  +
                             avg_imu.gyro_z  * avg_imu.gyro_z);
            {
                bool imu_stat = (fabsf(am - 1.0f) < IMU_ACCEL_DEVIATION) && (gm < IMU_GYRO_STATIC_DPS);
                bool gps_stat = (gps_data.speed < GPS_STATIC_SPEED_KMH);
                // When locked or in Stage 2+ the GPS speed noise floor (~1 km/h)
                // would register as motion — rely on IMU only to gate debounce.
                bool imu_only = position_locked || (g_power_state >= POWER_STATE_STAGE2);
                stationary = imu_stat && (imu_only || gps_stat);
            }

            // Update gate for the per-fix EMA block.
            // GPS speed is the primary indicator: if the receiver reports low
            // speed the EMA is preserved regardless of IMU reading.  The EMA
            // only snaps to raw position when BOTH GPS speed AND IMU confirm
            // real motion, preventing occasional IMU noise from invalidating
            // hours of accumulated EMA smoothing.
            imu_static_last = (gps_data.speed < GPS_STATIC_SPEED_KMH) ||
                              ((fabsf(am - 1.0f) < IMU_ACCEL_DEVIATION) &&
                               (gm < IMU_GYRO_STATIC_DPS));

            // ── Progressive power management ──────────────────────────────────
            // Track stationary time and transition through power states:
            //   Moving       -> 0-3 mins:   1s logging, full uploads, normal power
            //   Stage 1      -> 3-5 mins:   1min logging, full uploads, normal power
            //   Stage 2 (LP) -> 5-30 mins:  5min logging, periodic uploads, low power
            //   Stage 3 (Pk) -> 30+ mins:   30min heartbeat, periodic uploads, parked
            if (stationary) {
                stationary_time_ms += current_log_interval_ms;

                // ── Position lock ──────────────────────────────────────────────
                // Use wall-clock elapsed time so the re-lock window is
                // independent of the current log interval.
                if (!position_locked) {
                    uint32_t relock_ms =
                        (g_power_state >= POWER_STATE_STAGE3) ? GPS_RELOCK_TIME_STAGE3_MS :
                        (g_power_state == POWER_STATE_STAGE2) ? GPS_RELOCK_TIME_STAGE2_MS :
                                                                GPS_RELOCK_TIME_STAGE1_MS;
                    uint32_t elapsed_ms = (uint32_t)(
                        (xTaskGetTickCount() - lock_release_tick) * portTICK_PERIOD_MS);
                    if (elapsed_ms >= relock_ms) {
                        position_locked = true;
                        locked_lat_f = (float)ema_lat;
                        locked_lon_f = (float)ema_lon;
                        locked_alt_f = ema_alt;
                        ESP_LOGI(TAG, "GPS position locked at %.6f, %.6f after %lu ms static",
                                 (double)locked_lat_f, (double)locked_lon_f, elapsed_ms);
                    }
                }

                power_state_t old_state = g_power_state;
                
                // Transition to Stage 3: Parked heartbeat (30+ minutes static)
                if (stationary_time_ms >= STATIC_STAGE3_THRESHOLD_MS) {
                    if (g_power_state != POWER_STATE_STAGE3) {
                        g_power_state = POWER_STATE_STAGE3;
                        current_log_interval_ms = LOG_INTERVAL_STAGE3_MS;
                        ESP_LOGI(TAG, "Entering Stage 3 (Parked): static %lu ms, log rate now %lu ms",
                                 stationary_time_ms, current_log_interval_ms);
                        update_power_mode(g_power_state, old_state);
                    }
                }
                // Transition to Stage 2: Low power mode (5+ minutes static)
                else if (stationary_time_ms >= STATIC_STAGE2_THRESHOLD_MS) {
                    if (g_power_state != POWER_STATE_STAGE2) {
                        g_power_state = POWER_STATE_STAGE2;
                        current_log_interval_ms = LOG_INTERVAL_STAGE2_MS;
                        ESP_LOGI(TAG, "Entering Stage 2 (Low Power): static %lu ms, log rate now %lu ms",
                                 stationary_time_ms, current_log_interval_ms);
                        update_power_mode(g_power_state, old_state);
                    }
                }
                // Transition to Stage 1: Reduced logging (3-5 minutes static)
                else if (stationary_time_ms >= STATIC_STAGE1_THRESHOLD_MS) {
                    if (g_power_state != POWER_STATE_STAGE1) {
                        g_power_state = POWER_STATE_STAGE1;
                        current_log_interval_ms = LOG_INTERVAL_STAGE1_MS;
                        ESP_LOGI(TAG, "Entering Stage 1: static %lu ms, log rate now %lu ms",
                                 stationary_time_ms, current_log_interval_ms);
                        update_power_mode(g_power_state, old_state);
                    }
                }
            } else {
                // Debounce: require MOTION_CONFIRM_TICKS consecutive moving reads
                // before treating the device as actually moving.  This prevents
                // brief GPS speed noise spikes from resetting stationary_time_ms.
                motion_confirm_count++;
                if (motion_confirm_count >= MOTION_CONFIRM_TICKS) {
                    // Device is confirmed moving - restore fast logging and full power
                    if (was_stationary || g_power_state != POWER_STATE_MOVING) {
                        power_state_t old_state = g_power_state;
                        ESP_LOGI(TAG, "Motion confirmed (%u ticks) - exiting power saving mode", motion_confirm_count);
                        ESP_LOGI(TAG, "Restoring fast log rate (%lu ms) and full uploads", LOG_INTERVAL_MOVING_MS);
                        g_power_state = POWER_STATE_MOVING;
                        update_power_mode(g_power_state, old_state);
                    }
                    stationary_time_ms = 0;
                    current_log_interval_ms = LOG_INTERVAL_MOVING_MS;
                    // Release position lock so EMA can track the moving position
                    if (position_locked) {
                        ESP_LOGI(TAG, "GPS position lock released (motion confirmed)");
                        position_locked   = false;
                        lock_release_tick = xTaskGetTickCount();
                    }
                }
                // While debouncing, leave stationary_time_ms untouched so a
                // subsequent static reading can resume accumulating correctly.
            }
            if (stationary) motion_confirm_count = 0;
            was_stationary = stationary;
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

                // GPS UTC time fallback: use GPS time if neither NTP nor RTC succeeded.
                // Priority: NTP (best) → RTC → GPS UTC (this block).
                if (!time_synced && !time_set_from_rtc && gps_data.gps_time_valid) {
                    struct timeval tv_gps;
                    tv_gps.tv_sec  = portable_timegm(&gps_data.gps_utc_time);
                    tv_gps.tv_usec = 0;
                    if (tv_gps.tv_sec > 0) {
                        settimeofday(&tv_gps, NULL);
                        ESP_LOGW(TAG, "System time set from GPS UTC (no NTP/RTC available): "
                                      "%04d-%02d-%02d %02d:%02d:%02d UTC",
                                 gps_data.gps_utc_time.tm_year + 1900,
                                 gps_data.gps_utc_time.tm_mon  + 1,
                                 gps_data.gps_utc_time.tm_mday,
                                 gps_data.gps_utc_time.tm_hour,
                                 gps_data.gps_utc_time.tm_min,
                                 gps_data.gps_utc_time.tm_sec);
                    }
                }
            }
            
            log_to_csv(&log_gps, &avg_imu);
            lcd_update_display(&last_display_mode, &log_gps, &avg_imu, true);
        } else {
            // No GPS fix — log a local-only entry (fix=0) with available IMU data.
            // These rows are written to sync_data.csv but not streamed to peers.
            if (log_to_csv(&log_gps, &avg_imu) != ESP_OK) {
                ESP_LOGW(TAG, "Failed to write no-fix log entry");
            }
            ESP_LOGW(TAG, "Waiting for GPS fix... (accel: %.3f, %.3f, %.3f g)",
                     avg_imu.accel_x, avg_imu.accel_y, avg_imu.accel_z);
            lcd_update_display(&last_display_mode, &log_gps, &avg_imu, false);
        }
        
        // Resync LCD every ~5 seconds to silently fix any nibble misalignment
        if (lcd_initialized && lcd_display_on) {
            lcd_resync_counter++;
            if (lcd_resync_counter >= (5000 / current_log_interval_ms)) {
                lcd_resync_counter = 0;
                lcd_resync();
            }
        }
    }
    
    free(data);
}

/* ==================== Power Management ==================== */

// Called when transitioning between power states to adjust system behavior
static void update_power_mode(power_state_t new_state, power_state_t old_state)
{
    if (new_state == old_state) return;

    const char *from = (old_state == POWER_STATE_MOVING) ? "MOVING" :
                       (old_state == POWER_STATE_STAGE1) ? "STAGE1" :
                       (old_state == POWER_STATE_STAGE2) ? "STAGE2" : "STAGE3";
    const char *to   = (new_state == POWER_STATE_MOVING) ? "MOVING" :
                       (new_state == POWER_STATE_STAGE1) ? "STAGE1" :
                       (new_state == POWER_STATE_STAGE2) ? "STAGE2" : "STAGE3";

    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "  POWER STATE: %s  -->  %s", from, to);
    ESP_LOGW(TAG, "========================================");

    switch (new_state) {
        case POWER_STATE_MOVING:
            ESP_LOGW(TAG, "  Motion detected — full operation restored");
            ESP_LOGW(TAG, "  Log rate   : %lu ms (%.1f s)",
                     LOG_INTERVAL_MOVING_MS, LOG_INTERVAL_MOVING_MS / 1000.0f);
            ESP_LOGW(TAG, "  WiFi upload: ENABLED");
            ESP_LOGW(TAG, "  ESP-NOW    : ACTIVE");
            break;

        case POWER_STATE_STAGE1:
            ESP_LOGW(TAG, "  Static >%lu min — reducing log frequency",
                     STATIC_STAGE1_THRESHOLD_MS / 60000UL);
            ESP_LOGW(TAG, "  Log rate   : %lu ms (%lu min)",
                     LOG_INTERVAL_STAGE1_MS, LOG_INTERVAL_STAGE1_MS / 60000UL);
            ESP_LOGW(TAG, "  WiFi upload: ENABLED");
            ESP_LOGW(TAG, "  ESP-NOW    : ACTIVE");
            break;

        case POWER_STATE_STAGE2:
            ESP_LOGW(TAG, "  Static >%lu min — entering low power mode",
                     STATIC_STAGE2_THRESHOLD_MS / 60000UL);
            ESP_LOGW(TAG, "  Log rate   : %lu ms (%lu min)",
                     LOG_INTERVAL_STAGE2_MS, LOG_INTERVAL_STAGE2_MS / 60000UL);
            ESP_LOGW(TAG, "  WiFi upload: PERIODIC (%lu min interval)",
                     LOG_INTERVAL_STAGE2_MS / 60000UL);
            ESP_LOGW(TAG, "  ESP-NOW    : ACTIVE (peer sync continues)");
            break;

        case POWER_STATE_STAGE3:
            ESP_LOGW(TAG, "  Static >%lu min — entering parked (heartbeat) mode",
                     STATIC_STAGE3_THRESHOLD_MS / 60000UL);
            ESP_LOGW(TAG, "  Log rate   : %lu ms (%lu min)",
                     LOG_INTERVAL_STAGE3_MS, LOG_INTERVAL_STAGE3_MS / 60000UL);
            ESP_LOGW(TAG, "  WiFi upload: PERIODIC (%lu min interval)",
                     LOG_INTERVAL_STAGE2_MS / 60000UL);
            ESP_LOGW(TAG, "  ESP-NOW    : ACTIVE (peer sync continues)");
            break;
    }

    ESP_LOGW(TAG, "========================================");
}

/* ==================== Motion Detection ==================== */

// Thresholds for deciding the device is stationary
// GPS_STATIC_SPEED_KMH, IMU_ACCEL_DEVIATION, IMU_GYRO_STATIC_DPS — defined in sync_config.h
#define STATIC_REQUIRED_MS    5000    // ms of continuous stillness before sync

// Returns true when the device is confirmed (or assumed) to be stationary.
// Logic:
//   - If BOTH sensors are unavailable, assume static (safe fallback — better
//     to sync unnecessarily than never sync when hardware is missing).
//   - If only GPS is unavailable, rely solely on IMU.
//   - If only IMU is unavailable, rely solely on GPS.
//   - If both are available, both must agree the device is still.
static bool motion_is_static(void)
{
    bool gps_available = current_gps.valid;          // valid fix received at least once
    bool imu_available = (mpu6050_actual_addr != 0); // IMU detected during init

    // Neither sensor working — assume static so sync can still happen
    if (!gps_available && !imu_available) {
        return true;
    }

    // GPS check: speed below threshold (only used when a valid fix exists)
    bool gps_static = !gps_available || (current_gps.speed < GPS_STATIC_SPEED_KMH);

    // IMU check: accel magnitude ≈ 1 g (gravity only) and gyro near zero
    bool imu_static = true;
    if (imu_available) {
        float am = sqrtf(current_imu.accel_x * current_imu.accel_x +
                         current_imu.accel_y * current_imu.accel_y +
                         current_imu.accel_z * current_imu.accel_z);
        float gm = sqrtf(current_imu.gyro_x  * current_imu.gyro_x  +
                         current_imu.gyro_y  * current_imu.gyro_y  +
                         current_imu.gyro_z  * current_imu.gyro_z);
        imu_static = (fabsf(am - 1.0f) < IMU_ACCEL_DEVIATION) && (gm < IMU_GYRO_STATIC_DPS);
    }

    // In STAGE2+ (long-term parked) GPS speed noise (~1 km/h static floor)
    // would falsely signal motion — rely on IMU alone to avoid spurious wakeups.
    if (g_power_state >= POWER_STATE_STAGE2) {
        return !imu_available || imu_static;
    }

    // Only evaluate sensors that are actually available
    if (gps_available && imu_available) return gps_static && imu_static;
    if (gps_available)                  return gps_static;
    return imu_static;
}

/* ==================== Sync State Machine (Core 0) ==================== */
/*
 * Runs on Core 0 so it does not compete with the Core 1 GPS/IMU/LCD task.
 *
 * Algorithm:
 *  1. Wait until motion_is_static() is true (GPS speed + IMU confirm device
 *     is parked).
 *  2. While still static (and under SYNC_SEARCH_TIME_MAX iterations):
 *       a. Check power state (set by gps_logging_task based on stationary time):
 *          - POWER_STATE_MOVING / STAGE1: Full WiFi uploads enabled
 *          - POWER_STATE_STAGE2 (5+ mins static): WiFi uploads DISABLED
 *       b. If NOT in Stage 2 and WiFi is connected and the upload interval 
 *          has elapsed, POST sync_data.csv and sync_merged.csv to the server.
 *          On success delete and recreate the files so the next upload stays small.
 *       c. Run one ESP-NOW discovery + bidirectional CSV sync round.
 *          Received peer records are merged into sync_merged.csv.
 *          ESP-NOW continues to work in all power states (including Stage 2).
 *  3. When the device starts moving (or the iteration limit is reached) reset
 *     and go back to step 1.
 *
 * Power Management:
 *  - WiFi stays active with WIFI_PS_MIN_MODEM power save mode
 *  - ESP-NOW operates in all power states (can wake device from light sleep)
 *  - Stage 2 low power: WiFi uploads disabled, ESP-NOW sync still active
 *
 * The WiFi driver is already  initialised and connected by wifi_init_sta() on
 * the main core; we never call esp_wifi_start/stop here.  ESP-NOW runs on top
 * of the existing STA driver.
 */

static void sync_state_machine_task(void *pv)
{
    // Wait for the rest of the system to finish booting
    vTaskDelay(pdMS_TO_TICKS(5000));

    // sd_init() was already called in app_main (will detect the existing mount
    // and just set up the mutex).  Ensure the sync CSV files exist.
    sd_ensure_data_file();
    sd_ensure_merged_file();

    ESP_LOGI(TAG, "Sync state machine started on Core 0");

    while (true) {
        // ── Wait until the device is confirmed static ─────────────────────────
        if (!motion_is_static()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Device is static \u2013 entering sync loop");

        int     search_time       = 0;
        int64_t next_wifi_attempt = 0; // epoch-s; 0 = try on first iteration

        while (motion_is_static() && search_time < SYNC_SEARCH_TIME_MAX) {

            // ── WiFi upload ───────────────────────────────────────────────────
            // Upload in all power states.  In Stage 2+ use the 5-min interval
            // so data is not lost if the device is never recovered.
            {
                int64_t upload_interval_s = (g_power_state >= POWER_STATE_STAGE2)
                                            ? (int64_t)(LOG_INTERVAL_STAGE2_MS / 1000)
                                            : (int64_t)SYNC_WIFI_UPLOAD_INTERVAL_S;
                int64_t now_s = esp_timer_get_time() / 1000000LL;
                if (now_s >= next_wifi_attempt) {
                    if (g_power_state >= POWER_STATE_STAGE2) {
                        ESP_LOGI(TAG, "[%d] Low-power periodic upload (every %lld s)...",
                                 search_time, upload_interval_s);
                    }
                    // Attempt to connect if not already connected
                    if (!wifi_is_connected()) {
                        ESP_LOGI(TAG, "[%d] WiFi not connected - attempting to connect...", search_time);
                        if (!wifi_connect()) {
                            ESP_LOGW(TAG, "[%d] WiFi connection failed - skipping upload", search_time);
                        }
                    }

                    // Upload if we're now connected
                    if (wifi_is_connected()) {
                        ESP_LOGI(TAG, "[%d] WiFi up – uploading sync files...", search_time);
                        wifi_upload_report_t upload_report = { 0 };
                        if (wifi_upload_all_csv(&upload_report)) {
                            bool own_uploaded = (upload_report.own_data == WIFI_UPLOAD_UPLOADED);
                            bool merged_uploaded = (upload_report.merged_data == WIFI_UPLOAD_UPLOADED);

                            if (own_uploaded || merged_uploaded) {
                                ESP_LOGI(TAG, "Upload OK – sync files backed up");
                            } else {
                                ESP_LOGI(TAG, "No new sync rows uploaded");
                            }

                            if (own_uploaded) {
                                espnow_sync_reset_own_state();
                            }
                            if (merged_uploaded) {
                                espnow_sync_reset_merged_state();
                            }
                        } else {
                            ESP_LOGW(TAG, "Upload failed - preserved pending sync files for retry");
                        }
                    }

                    next_wifi_attempt = (esp_timer_get_time() / 1000000LL) + upload_interval_s;
                }
            }

            // ── ESP-NOW peer discovery and CSV sync ───────────────────────────
            if (!espnow_init()) {
                ESP_LOGE(TAG, "ESP-NOW init failed \u2013 skipping sync round");
            } else {
                ESP_LOGI(TAG, "[%d] Running ESP-NOW sync round...", search_time);
                int synced = espnow_sync_round();
                ESP_LOGI(TAG, "[%d] Synced with %d peer(s)", search_time, synced);
                espnow_deinit();
            }

            // ── Re-check motion ───────────────────────────────────────────────
            if (!motion_is_static()) {
                ESP_LOGI(TAG, "Device started moving \u2013 exiting sync loop");
                break;
            }

            search_time++;
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        ESP_LOGI(TAG, "Sync window complete (search_time=%d) \u2013 resetting",
                 search_time);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
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
    
#if LCD_ENABLED
    // Initialize LCD1602A
    if (lcd_init() == ESP_OK) {
        lcd_clear();
        lcd_printf(0, 0, "GPS Logger");
        lcd_printf(0, 1, "Initializing...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        lcd_last_activity_time = xTaskGetTickCount();
    }
#endif
    
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
    
    // Set timezone before the RTC read so mktime() converts local→UTC correctly
    // regardless of whether the RTC is present. The timezone will be refined later
    // by get_timezone_from_gps() on the first valid GPS fix.
    setenv("TZ", TIMEZONE, 1);
    tzset();

    // Initialize RTC and set system time from it
    struct tm timeinfo;
    if (rtc_get_time(&timeinfo) == ESP_OK) {
        ESP_LOGI(TAG, "RTC time: %04d-%02d-%02d %02d:%02d:%02d",
                 timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        // Set system time from RTC (TZ already configured above)
        struct timeval tv;
        tv.tv_sec = mktime(&timeinfo);
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);
        time_set_from_rtc = true;

        // Display local time
        time_t now = time(NULL);
        localtime_r(&now, &timeinfo);
        ESP_LOGI(TAG, "Local time: %04d-%02d-%02d %02d:%02d:%02d",
                 timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        ESP_LOGE(TAG, "RTC not available - time will be set from GPS UTC on first fix");
    }
    
    // Initialize SD Card
    if (sd_card_init() != ESP_OK) {
        ESP_LOGE(TAG, "SD Card initialization failed - check connections");
    }

    // Attach the sync-storage module to the already-mounted SD card.
    // sd_init() detects the existing mount and only creates its mutex.
    if (!sd_init()) {
        ESP_LOGE(TAG, "Sync SD init failed");
    }

    // Read own WiFi MAC and format as 4 hex digits for sync CSV records.
    // Use esp_read_mac() (reads eFuse) so WiFi doesn't need to be started yet.
    {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        snprintf(g_own_mac_str, sizeof(g_own_mac_str), "%02X%02X", mac[4], mac[5]);
        ESP_LOGI(TAG, "Sync device MAC suffix: %s", g_own_mac_str);
    }

    // Create the sync CSV files if they do not yet exist.
    sd_ensure_data_file();
    sd_ensure_merged_file();
    
    // Initialize GPS UART and configure module for 5 Hz
    gps_uart_init();
    gps_configure_5hz();
    
    // Initialize WiFi hardware (STA mode, no connection attempt yet)
    wifi_init_sta();

    // Register the WiFi-upload module's event handlers (single set of handlers
    // for the entire lifetime of the app — avoids dual-handler conflicts).
    wifi_stack_init();

    // Attempt initial WiFi connection using the AP list
    if (wifi_connect()) {
        ESP_LOGI(TAG, "WiFi connected at boot");
        initialize_sntp();
    } else {
        ESP_LOGW(TAG, "WiFi not available at boot — sync task will retry later");
    }

    // Start HTTP Server — always start so it's reachable whenever WiFi connects,
    // even if the initial connection attempt failed or is still in progress.
    server = start_webserver();
    
    // Start GPS logging task on Core 1 (GPS UART, IMU I2C, LCD I2C, RTC I2C)
    xTaskCreatePinnedToCore(gps_logging_task, "gps_logging", 4096, NULL, 5, NULL, 1);

    // Start ESP-NOW sync state machine on Core 0 (WiFi radio + SD file exchange)
    xTaskCreatePinnedToCore(sync_state_machine_task, "sync_sm", 8192, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "System initialized. Logging GPS data...");
}
