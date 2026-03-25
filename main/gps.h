#pragma once

// gps.h — GPS UART driver, NMEA parsing, EMA position filter.

#include "types.h"
#include <stdbool.h>

/**
 * @brief Initialise GPS UART and configure module for 5 Hz.
 */
void gps_init(void);

/**
 * @brief Non-blocking read of UART + NMEA parsing.
 *        Should be called at GPS_SAMPLE_PERIOD_MS rate (~5 Hz).
 * @param out  Updated with latest parsed data.
 * @return true if a new RMC fix was parsed this call.
 */
bool gps_read(gps_data_t *out);

/**
 * @brief Returns true if a valid GPS fix has been received at least once.
 */
bool gps_has_fix(void);

/**
 * @brief Apply EMA filter to the current fix position.
 *        Called at GPS rate when stationary for smoothing.
 * @param data       Current GPS fix (will be modified with filtered position).
 * @param is_static  true if IMU considers the device stationary.
 */
void gps_filter_position(gps_data_t *data, bool is_static);

/**
 * @brief Lock the filtered position (freeze coords for parked device).
 */
void gps_lock_position(void);

/**
 * @brief Unlock position lock (motion detected — resume tracking).
 */
void gps_unlock_position(void);

/**
 * @brief Returns true if position is currently locked.
 */
bool gps_is_position_locked(void);

/**
 * @brief Get the timezone string for a GPS coordinate.
 *        Uses approximate longitude bands with political exceptions.
 */
const char *gps_get_timezone(float latitude, float longitude);
