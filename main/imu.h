#pragma once

// imu.h — MPU6050 I2C driver, calibration, motion detection.

#include "types.h"
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/**
 * @brief Initialise MPU6050 on the given I2C bus.
 *        Auto-detects at 0x69 then 0x68.
 * @param bus  I2C master bus handle (Bus 0 shared with RTC).
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no MPU detected.
 */
esp_err_t imu_init(i2c_master_bus_handle_t bus);

/**
 * @brief Calibrate IMU (100 samples, device must be still).
 */
esp_err_t imu_calibrate(void);

/**
 * @brief Read calibrated accel + gyro into out.
 * @return ESP_ERR_INVALID_STATE if not initialised.
 */
esp_err_t imu_read(imu_data_t *out);

/**
 * @brief Returns true if device is stationary (accel ≈ 1g, gyro ≈ 0).
 *        Thresholds are read from config at call time.
 */
bool imu_is_static(const imu_data_t *data);

/**
 * @brief Returns true if IMU was detected and initialised.
 */
bool imu_is_available(void);

/**
 * @brief Return the detected I2C address (0x68 or 0x69), or 0 if not found.
 */
uint8_t imu_get_address(void);

/**
 * @brief Prepare IMU for deep sleep.
 *        Disables all interrupt outputs (INT_ENABLE = 0) and reads INT_STATUS
 *        to clear any latched interrupt, ensuring the INT pin is LOW before
 *        esp_deep_sleep_start() is called.  Without this, a latched data-ready
 *        interrupt keeps the pin HIGH and the device wakes immediately.
 */
void imu_prepare_sleep(void);
