#pragma once

// rtc.h — DS3231 RTC I2C driver.

#include <time.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/**
 * @brief Initialise DS3231 RTC on the given I2C bus.
 * @param bus  I2C master bus handle (Bus 0 shared with IMU).
 */
esp_err_t rtc_module_init(i2c_master_bus_handle_t bus);

/**
 * @brief Read current time from DS3231.
 */
esp_err_t rtc_get_time(struct tm *timeinfo);

/**
 * @brief Write time to DS3231.
 */
esp_err_t rtc_set_time(const struct tm *timeinfo);
