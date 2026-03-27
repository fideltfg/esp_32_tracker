// imu.c — MPU6050 I2C driver, calibration, motion detection.

#include "imu.h"
#include "config.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "IMU";

static i2c_master_dev_handle_t s_dev = NULL;
static uint8_t s_addr = 0;

// Calibration offsets
static float s_off_ax = 0, s_off_ay = 0, s_off_az = 0;
static float s_off_gx = 0, s_off_gy = 0, s_off_gz = 0;
static bool s_calibrated = false;

// ── I2C helpers ──────────────────────────────────────────────────────────────

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, 1000);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, 1000);
}

// ── Public API ───────────────────────────────────────────────────────────────

esp_err_t imu_init(i2c_master_bus_handle_t bus)
{
    uint8_t try_addrs[] = {0x69, 0x68};
    bool found = false;

    for (int i = 0; i < 2; i++)
    {
        uint8_t addr = try_addrs[i];
        if (i2c_master_probe(bus, addr, 50) != ESP_OK)
            continue;

        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = I2C_FREQ_HZ,
        };
        i2c_master_dev_handle_t dev = NULL;
        if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK)
            continue;

        // Wake up
        uint8_t wake[] = {0x6B, 0x00};
        if (i2c_master_transmit(dev, wake, 2, 1000) != ESP_OK)
        {
            i2c_master_bus_rm_device(dev);
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(50));

        // Read WHO_AM_I
        uint8_t wai = 0;
        uint8_t reg = 0x75;
        if (i2c_master_transmit_receive(dev, &reg, 1, &wai, 1, 1000) != ESP_OK)
        {
            i2c_master_bus_rm_device(dev);
            continue;
        }

        s_dev = dev;
        s_addr = addr;
        found = true;

        const char *name = (wai == 0x68) ? "MPU6050" : (wai == 0x70) ? "MPU6500/MPU9250"
                                                   : (wai == 0x71)   ? "MPU6555"
                                                   : (wai == 0x73)   ? "MPU9255"
                                                                     : "MPU6050-compatible";
        ESP_LOGI(TAG, "%s at 0x%02X (WHO_AM_I=0x%02X)", name, addr, wai);
        break;
    }

    if (!found)
    {
        ESP_LOGE(TAG, "MPU6050 not found at 0x68 or 0x69");
        return ESP_ERR_NOT_FOUND;
    }

    // Configure: ±2g accel, ±250°/s gyro, 125 Hz sample, DLPF ~44 Hz
    write_reg(0x1C, 0x00); // accel ±2g
    write_reg(0x1B, 0x00); // gyro ±250°/s
    write_reg(0x19, 0x07); // sample rate divider → 125 Hz
    write_reg(0x1A, 0x03); // DLPF ~44 Hz

    ESP_LOGI(TAG, "Initialised at 0x%02X", s_addr);
    return ESP_OK;
}

esp_err_t imu_calibrate(void)
{
    if (!s_dev)
        return ESP_ERR_INVALID_STATE;

    ESP_LOGW(TAG, "Calibrating IMU — keep device still!");
    const int N = 100;
    float sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;

    for (int i = 0; i < N; i++)
    {
        uint8_t raw[14];
        if (read_regs(0x3B, raw, 14) != ESP_OK)
        {
            ESP_LOGE(TAG, "Calibration read failed at sample %d", i);
            return ESP_FAIL;
        }
        sax += (int16_t)((raw[0] << 8) | raw[1]) / 16384.0f;
        say += (int16_t)((raw[2] << 8) | raw[3]) / 16384.0f;
        saz += (int16_t)((raw[4] << 8) | raw[5]) / 16384.0f;
        sgx += (int16_t)((raw[8] << 8) | raw[9]) / 131.0f;
        sgy += (int16_t)((raw[10] << 8) | raw[11]) / 131.0f;
        sgz += (int16_t)((raw[12] << 8) | raw[13]) / 131.0f;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    s_off_ax = sax / N;
    s_off_ay = say / N;
    s_off_az = (saz / N) - 1.0f; // Z expects 1g
    s_off_gx = sgx / N;
    s_off_gy = sgy / N;
    s_off_gz = sgz / N;
    s_calibrated = true;

    ESP_LOGI(TAG, "Calibration done: accel(%.3f,%.3f,%.3f) gyro(%.3f,%.3f,%.3f)",
             s_off_ax, s_off_ay, s_off_az, s_off_gx, s_off_gy, s_off_gz);
    return ESP_OK;
}

esp_err_t imu_read(imu_data_t *out)
{
    if (!s_dev)
        return ESP_ERR_INVALID_STATE;

    uint8_t raw[14];
    esp_err_t ret = read_regs(0x3B, raw, 14);
    if (ret != ESP_OK)
        return ret;

    out->accel_x = ((int16_t)((raw[0] << 8) | raw[1])) / 16384.0f - s_off_ax;
    out->accel_y = ((int16_t)((raw[2] << 8) | raw[3])) / 16384.0f - s_off_ay;
    out->accel_z = ((int16_t)((raw[4] << 8) | raw[5])) / 16384.0f - s_off_az;
    out->gyro_x = ((int16_t)((raw[8] << 8) | raw[9])) / 131.0f - s_off_gx;
    out->gyro_y = ((int16_t)((raw[10] << 8) | raw[11])) / 131.0f - s_off_gy;
    out->gyro_z = ((int16_t)((raw[12] << 8) | raw[13])) / 131.0f - s_off_gz;
    return ESP_OK;
}

bool imu_is_static(const imu_data_t *data)
{
    const tracker_config_t *cfg = config_get();
    float am = sqrtf(data->accel_x * data->accel_x +
                     data->accel_y * data->accel_y +
                     data->accel_z * data->accel_z);
    float gm = sqrtf(data->gyro_x * data->gyro_x +
                     data->gyro_y * data->gyro_y +
                     data->gyro_z * data->gyro_z);
    return (fabsf(am - 1.0f) < cfg->imu_accel_dev) && (gm < cfg->imu_gyro_dps);
}

bool imu_is_available(void) { return s_dev != NULL; }
uint8_t imu_get_address(void) { return s_addr; }
