#pragma once

// types.h — Shared data structures used across all modules.

#include <stdbool.h>
#include <time.h>

// GPS fix data (produced by gps.c, consumed by power.c, webserver.c, logging)
typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float speed;        // km/h
    float hdop;         // horizontal dilution of precision
    int   num_sv;       // satellites in use
    bool  valid;        // true if we have a valid fix
    struct tm gps_utc_time;
    bool  gps_time_valid;
} gps_data_t;

// IMU reading (produced by imu.c, consumed by power.c, logging)
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} imu_data_t;

// Progressive power states — drives log interval and upload behavior
typedef enum {
    POWER_STATE_MOVING = 0,  // 1s logging, full uploads
    POWER_STATE_STAGE1 = 1,  // 1min logging, full uploads
    POWER_STATE_STAGE2 = 2,  // 5min logging, periodic uploads
    POWER_STATE_STAGE3 = 3,  // 30min heartbeat, periodic uploads
} power_state_t;

// WiFi AP credential entry
typedef struct {
    const char *ssid;
    const char *password;
} wifi_ap_entry_t;
