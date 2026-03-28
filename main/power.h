#pragma once

// power.h — Power state machine, progressive logging, deep sleep.

#include "types.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Power on the DS3231 RTC module via GPIO-controlled P-MOSFET.
 *        Must be called BEFORE I2C bus init so the module is on the bus.
 *        Also releases any deep-sleep GPIO hold from the previous sleep cycle.
 *        No-op if RTC_PWR_GPIO == 0.
 */
void power_rtc_pwr_init(void);

/**
 * @brief Initialise power manager.
 */
void power_init(void);

/**
 * @brief Evaluate motion data and update power state.
 *        Call from the GPS logging task at log-tick rate.
 * @param gps       Current GPS data.
 * @param imu       Current averaged IMU data.
 * @param interval_ms  Current log interval (updated on state change).
 */
void power_evaluate(const gps_data_t *gps, const imu_data_t *imu,
                    uint32_t *interval_ms);

/**
 * @brief Fast motion check (call at GPS_SAMPLE_PERIOD_MS rate).
 *        Exits Stage1/2/3 immediately on confirmed movement.
 * @param gps  Current instantaneous GPS data.
 * @param imu  Current instantaneous IMU data.
 * @param interval_ms  Updated on state change.
 */
void power_fast_motion_check(const gps_data_t *gps, const imu_data_t *imu,
                             uint32_t *interval_ms);

/**
 * @brief Get current power state.
 */
power_state_t power_get_state(void);

/**
 * @brief Get current log interval in ms.
 */
uint32_t power_get_log_interval_ms(void);

/**
 * @brief Returns true if device is confirmed stationary.
 *        Uses GPS speed + IMU, with sensor fallback logic.
 */
bool power_is_static(const gps_data_t *gps, const imu_data_t *imu);

/**
 * @brief Returns true when STAGE3 and minimum uptime grace period has elapsed.
 *        Use this before entering deep sleep to avoid immediate re-sleep.
 */
bool power_deep_sleep_ready(void);

/**
 * @brief Enter deep sleep with IMU wake-on-motion or timer fallback.
 *        Only call from STAGE3 when no pending work remains.
 */
void power_enter_deep_sleep(void);

/**
 * @brief Check wake reason at boot. Returns true if waking from deep sleep.
 */
bool power_check_wake_reason(void);
