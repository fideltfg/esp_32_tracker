// lcd.c — LCD1602A I2C driver with resync, auto-off, and power control.

#include "lcd.h"
#include "config.h"
#include "wifi_manager.h"

#include <string.h>
#include <stdarg.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

static const char *TAG = "LCD";

// HD44780 commands
#define CMD_CLEAR        0x01
#define CMD_HOME         0x02
#define CMD_ENTRY_MODE   0x04
#define CMD_DISPLAY_CTRL 0x08
#define CMD_FUNCTION_SET 0x20
#define CMD_SET_DDRAM    0x80

// Flags
#define DISPLAY_ON   0x04
#define DISPLAY_OFF  0x00
#define CURSOR_OFF   0x00
#define BLINK_OFF    0x00
#define ENTRY_LEFT   0x02
#define MODE_4BIT    0x00
#define MODE_2LINE   0x08
#define DOTS_5x8     0x00

// I2C backpack bits
#define BL_ON   0x08
#define BL_OFF  0x00
#define EN_BIT  0x04
#define RS_BIT  0x01

static i2c_master_dev_handle_t s_dev = NULL;
static uint8_t s_backlight = BL_ON;
static bool    s_initialized = false;
static bool    s_display_on = false;
static char    s_lines[2][17] = {{0}, {0}};

static volatile bool     s_wake_requested = false;
static volatile uint32_t s_last_activity   = 0;
static volatile uint8_t  s_display_mode    = 0;
static uint8_t           s_last_disp_mode  = 0xFF;

// ── Low-level I2C ────────────────────────────────────────────────────────────

static esp_err_t i2c_write(uint8_t data)
{
    esp_err_t ret = ESP_FAIL;
    for (int r = 0; r < 3 && ret != ESP_OK; r++) {
        ret = i2c_master_transmit(s_dev, &data, 1, 100);
        if (ret != ESP_OK) vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ret;
}

static void pulse_enable(uint8_t data)
{
    i2c_write(data | EN_BIT);
    esp_rom_delay_us(1);
    i2c_write(data);
    esp_rom_delay_us(50);
}

static void write_nibble(uint8_t nibble, bool rs)
{
    uint8_t data = nibble | s_backlight;
    if (rs) data |= RS_BIT;
    pulse_enable(data);
}

static void write_byte(uint8_t byte, bool rs)
{
    write_nibble(byte & 0xF0, rs);
    esp_rom_delay_us(10);
    write_nibble((byte << 4) & 0xF0, rs);
}

static void send_cmd(uint8_t cmd) { write_byte(cmd, false); }
static void send_data(uint8_t d)  { write_byte(d, true); }

static void set_cursor(uint8_t col, uint8_t row)
{
    uint8_t offsets[] = { 0x00, 0x40 };
    if (row > 1) row = 1;
    if (col > 15) col = 15;
    send_cmd(CMD_SET_DDRAM | (col + offsets[row]));
}

static void print_str(const char *s)
{
    while (*s) send_data((uint8_t)*s++);
}

// ── Public API ───────────────────────────────────────────────────────────────

esp_err_t lcd_init(i2c_master_bus_handle_t bus)
{
    uint8_t addrs[] = { 0x27, 0x3F };
    bool found = false;

    for (int i = 0; i < 2; i++) {
        if (i2c_master_probe(bus, addrs[i], 50) == ESP_OK) {
            i2c_device_config_t cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address  = addrs[i],
                .scl_speed_hz    = LCD_I2C_FREQ_HZ,
            };
            if (i2c_master_bus_add_device(bus, &cfg, &s_dev) == ESP_OK) {
                found = true;
                ESP_LOGI(TAG, "LCD1602A at 0x%02X", addrs[i]);
            }
            break;
        }
    }
    if (!found) {
        ESP_LOGW(TAG, "LCD1602A not found");
        return ESP_ERR_NOT_FOUND;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // HD44780 4-bit init sequence
    write_nibble(0x30, false); vTaskDelay(pdMS_TO_TICKS(5));
    write_nibble(0x30, false); vTaskDelay(pdMS_TO_TICKS(5));
    write_nibble(0x30, false); vTaskDelay(pdMS_TO_TICKS(1));
    write_nibble(0x20, false); vTaskDelay(pdMS_TO_TICKS(1));

    send_cmd(CMD_FUNCTION_SET | MODE_4BIT | MODE_2LINE | DOTS_5x8);
    vTaskDelay(pdMS_TO_TICKS(1));
    send_cmd(CMD_DISPLAY_CTRL | DISPLAY_OFF);
    vTaskDelay(pdMS_TO_TICKS(1));
    send_cmd(CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(10));
    send_cmd(CMD_ENTRY_MODE | ENTRY_LEFT);
    vTaskDelay(pdMS_TO_TICKS(1));
    send_cmd(CMD_DISPLAY_CTRL | DISPLAY_ON | CURSOR_OFF | BLINK_OFF);
    vTaskDelay(pdMS_TO_TICKS(1));

    s_display_on  = true;
    s_initialized = true;
    s_last_activity = xTaskGetTickCount();
    return ESP_OK;
}

esp_err_t lcd_clear(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    send_cmd(CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

void lcd_printf(uint8_t col, uint8_t row, const char *fmt, ...)
{
    if (!s_initialized) return;

    char buf[17];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // Pad to 16 chars
    int len = strlen(buf);
    for (int i = len; i < 16; i++) buf[i] = ' ';
    buf[16] = '\0';

    if (row < 2) memcpy(s_lines[row], buf, 17);

    set_cursor(col, row);
    print_str(buf);
}

void lcd_resync(void)
{
    if (!s_initialized) return;

    write_nibble(0x30, false); vTaskDelay(pdMS_TO_TICKS(5));
    write_nibble(0x30, false); vTaskDelay(pdMS_TO_TICKS(5));
    write_nibble(0x30, false); vTaskDelay(pdMS_TO_TICKS(5));
    write_nibble(0x20, false); vTaskDelay(pdMS_TO_TICKS(5));
    send_cmd(CMD_FUNCTION_SET | MODE_4BIT | MODE_2LINE | DOTS_5x8);
    vTaskDelay(pdMS_TO_TICKS(1));
    send_cmd(CMD_DISPLAY_CTRL | DISPLAY_ON | CURSOR_OFF | BLINK_OFF);
    vTaskDelay(pdMS_TO_TICKS(1));

    if (s_lines[0][0]) { set_cursor(0, 0); print_str(s_lines[0]); }
    if (s_lines[1][0]) { set_cursor(0, 1); print_str(s_lines[1]); }
}

void lcd_set_power(bool on)
{
    if (!s_initialized) return;
    s_backlight = on ? BL_ON : BL_OFF;
    if (on) {
        send_cmd(CMD_DISPLAY_CTRL | DISPLAY_ON | CURSOR_OFF | BLINK_OFF);
        s_display_on = true;
    } else {
        send_cmd(CMD_DISPLAY_CTRL | DISPLAY_OFF);
        i2c_write(BL_OFF);
        s_display_on = false;
    }
}

bool lcd_is_initialized(void) { return s_initialized; }
bool lcd_is_on(void)          { return s_display_on; }

void lcd_touch_activity(void)
{
    s_last_activity = xTaskGetTickCount();
}

void lcd_check_timeout(void)
{
    if (!s_initialized || !s_display_on) return;
    uint16_t timeout = config_get()->lcd_timeout_sec;
    if (timeout == 0) return;
    if ((xTaskGetTickCount() - s_last_activity) >
        pdMS_TO_TICKS((uint32_t)timeout * 1000UL)) {
        lcd_set_power(false);
    }
}

bool lcd_request_wake(void)
{
    if (!s_display_on) {
        s_wake_requested = true;
        return true;
    }
    return false;
}

bool lcd_process_wake(void)
{
    if (!s_initialized || !s_wake_requested) return false;
    s_wake_requested = false;
    lcd_set_power(true);
    s_last_activity = xTaskGetTickCount();
    return true;
}

void lcd_toggle_mode(void)
{
    s_display_mode = (s_display_mode + 1) % 2;
}

void lcd_update(const gps_data_t *gps, const imu_data_t *imu, bool has_fix)
{
    if (!s_initialized || !s_display_on) return;
    if (s_display_mode != s_last_disp_mode) {
        lcd_clear();
        s_last_disp_mode = s_display_mode;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s_display_mode == 0) {
        if (has_fix) {
            lcd_printf(0, 0, "%.1fkm %.1fm", gps->speed, gps->altitude);
            lcd_printf(0, 1, "%.4f,%.4f", gps->latitude, gps->longitude);
        } else {
            lcd_printf(0, 0, "No GPS Fix");
            lcd_printf(0, 1, "A:%.2f,%.2f,%.2f",
                       imu->accel_x, imu->accel_y, imu->accel_z);
        }
    } else {
        time_t now = time(NULL);
        struct tm ti;
        localtime_r(&now, &ti);
        char ip[16];
        wifi_mgr_get_ip_str(ip, sizeof(ip));
        lcd_printf(0, 0, "%02d:%02d:%02d", ti.tm_hour, ti.tm_min, ti.tm_sec);
        lcd_printf(0, 1, "%s", ip);
    }
}
