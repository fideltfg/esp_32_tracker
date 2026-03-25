// rtc.c — DS3231 RTC I2C driver.

#include "rtc.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "RTC";
static i2c_master_dev_handle_t s_dev = NULL;

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, 2, 1000);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, 1000);
}

static uint8_t bcd_to_dec(uint8_t v) { return (v / 16 * 10) + (v % 16); }
static uint8_t dec_to_bcd(uint8_t v) { return (v / 10 * 16) + (v % 10); }

esp_err_t rtc_module_init(i2c_master_bus_handle_t bus)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = RTC_I2C_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add DS3231 device");
        return ret;
    }
    ESP_LOGI(TAG, "DS3231 at 0x%02X", RTC_I2C_ADDR);
    return ESP_OK;
}

esp_err_t rtc_get_time(struct tm *timeinfo)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    uint8_t data[7];
    esp_err_t ret = read_regs(0x00, data, 7);
    if (ret != ESP_OK) return ret;

    timeinfo->tm_sec  = bcd_to_dec(data[0] & 0x7F);
    timeinfo->tm_min  = bcd_to_dec(data[1] & 0x7F);
    timeinfo->tm_hour = bcd_to_dec(data[2] & 0x3F);
    timeinfo->tm_mday = bcd_to_dec(data[4] & 0x3F);
    timeinfo->tm_mon  = bcd_to_dec(data[5] & 0x1F) - 1;
    timeinfo->tm_year = bcd_to_dec(data[6]) + 100;
    return ESP_OK;
}

esp_err_t rtc_set_time(const struct tm *timeinfo)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    esp_err_t ret;
    ret = write_reg(0x00, dec_to_bcd(timeinfo->tm_sec));  if (ret != ESP_OK) return ret;
    ret = write_reg(0x01, dec_to_bcd(timeinfo->tm_min));  if (ret != ESP_OK) return ret;
    ret = write_reg(0x02, dec_to_bcd(timeinfo->tm_hour)); if (ret != ESP_OK) return ret;
    ret = write_reg(0x04, dec_to_bcd(timeinfo->tm_mday)); if (ret != ESP_OK) return ret;
    ret = write_reg(0x05, dec_to_bcd(timeinfo->tm_mon + 1)); if (ret != ESP_OK) return ret;
    ret = write_reg(0x06, dec_to_bcd(timeinfo->tm_year - 100));
    return ret;
}
