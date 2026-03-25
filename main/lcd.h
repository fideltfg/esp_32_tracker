#pragma once

// lcd.h — LCD1602A I2C driver with resync and auto-off.

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/**
 * @brief Initialise LCD1602A on the given I2C bus.
 *        Auto-detects at 0x27 then 0x3F.
 * @return ESP_ERR_NOT_FOUND if no LCD detected.
 */
esp_err_t lcd_init(i2c_master_bus_handle_t bus);

/**
 * @brief Clear the display.
 */
esp_err_t lcd_clear(void);

/**
 * @brief Printf to a specific row/col (padded to 16 chars).
 */
void lcd_printf(uint8_t col, uint8_t row, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

/**
 * @brief Resync HD44780 nibble alignment (call periodically).
 */
void lcd_resync(void);

/**
 * @brief Turn display + backlight on/off.
 */
void lcd_set_power(bool on);

/**
 * @brief Returns true if LCD was detected and initialised.
 */
bool lcd_is_initialized(void);

/**
 * @brief Returns true if the display is currently on.
 */
bool lcd_is_on(void);

/**
 * @brief Record user activity (resets auto-off timer).
 */
void lcd_touch_activity(void);

/**
 * @brief Call periodically — turns off display after timeout.
 */
void lcd_check_timeout(void);

/**
 * @brief Request LCD wake from ISR context.
 *        Returns true if the LCD was off and needs waking.
 */
bool lcd_request_wake(void);

/**
 * @brief Process pending wake request (call from task context).
 *        Returns true if the display was actually woken.
 */
bool lcd_process_wake(void);
