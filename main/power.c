// power.c — Power state machine (staging disabled; always MOVING rate).

#include "power.h"
#include "config.h"
#include "gps.h"
#include "imu.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"

static const char *TAG = "POWER";

// RTC attrs retained so layout is unchanged across flash cycles,
// but staging is not restored — device always boots into MOVING.
static RTC_DATA_ATTR power_state_t  s_rtc_state         = POWER_STATE_MOVING;
static RTC_DATA_ATTR uint32_t       s_rtc_stationary_ms  = 0;

static volatile power_state_t s_state = POWER_STATE_MOVING;
static uint32_t s_log_interval_ms = 0;

// Position lock tracking (independent of power staging)
static TickType_t s_stationary_since = 0;
static bool       s_was_stationary   = false;
static uint8_t    s_motion_count     = 0;
#define LOCK_MOTION_CONFIRM 3   // consecutive moving samples (at log rate) required to unlock

// ── Public API ───────────────────────────────────────────────────────────────

void power_init(void)
{
    const tracker_config_t *cfg = config_get();
    s_state             = POWER_STATE_MOVING;
    s_log_interval_ms   = cfg->log_moving_ms;
    s_rtc_state         = POWER_STATE_MOVING;
    s_rtc_stationary_ms = 0;
    s_stationary_since  = xTaskGetTickCount();
    s_was_stationary    = false;
    ESP_LOGI(TAG, "Power staging disabled — always MOVING rate (%lu ms)",
             (unsigned long)cfg->log_moving_ms);
}

power_state_t power_get_state(void)       { return s_state; }
uint32_t power_get_log_interval_ms(void) { return s_log_interval_ms; }

bool power_deep_sleep_ready(void)
{
    return false;   // staging disabled — deep sleep never triggered automatically
}

bool power_is_static(const gps_data_t *gps, const imu_data_t *imu)
{
    const tracker_config_t *cfg = config_get();
    bool gps_ok = gps->valid;
    bool imu_ok = imu_is_available();

    if (!gps_ok && !imu_ok) return true;

    bool gps_static = !gps_ok || (gps->speed < cfg->gps_static_kmh);

    bool imu_static = true;
    if (imu_ok) {
        imu_data_t d = *imu;
        imu_static = imu_is_static(&d);
    }

    if (gps_ok && imu_ok) return gps_static && imu_static;
    if (gps_ok)           return gps_static;
    return imu_static;
}

void power_evaluate(const gps_data_t *gps, const imu_data_t *imu,
                    uint32_t *interval_ms)
{
    const tracker_config_t *cfg = config_get();

    // GPS is authoritative when valid and the position is not locked.
    // When the position lock is active, gps_filter_position() forces speed to 0,
    // so GPS speed cannot signal motion — fall back to IMU so the lock can be
    // released when the device starts moving again.
    // Fall back to IMU also when GPS has no fix.
    bool stationary;
    if (gps->valid && !gps_is_position_locked()) {
        stationary = (gps->speed < cfg->gps_static_kmh);
    } else {
        float am = sqrtf(imu->accel_x*imu->accel_x + imu->accel_y*imu->accel_y +
                         imu->accel_z*imu->accel_z);
        float gm = sqrtf(imu->gyro_x*imu->gyro_x + imu->gyro_y*imu->gyro_y +
                         imu->gyro_z*imu->gyro_z);
        stationary = (fabsf(am - 1.0f) < cfg->imu_accel_dev) && (gm < cfg->imu_gyro_dps);
    }

    if (stationary) {
        s_motion_count = 0;
        if (!s_was_stationary) {
            s_stationary_since = xTaskGetTickCount();
            s_was_stationary = true;
        }
        if (!gps_is_position_locked()) {
            uint32_t stationary_ms = (uint32_t)((xTaskGetTickCount() - s_stationary_since) * portTICK_PERIOD_MS);
            if (stationary_ms >= cfg->relock_stage1_ms)
                gps_lock_position();
        }
    } else {
        // Require several consecutive moving samples before releasing lock
        // so brief vibration spikes do not break it.
        s_motion_count++;
        if (s_motion_count >= LOCK_MOTION_CONFIRM) {
            s_motion_count   = 0;
            s_was_stationary = false;
            if (gps_is_position_locked()) {
                gps_unlock_position();
            }
            s_stationary_since = xTaskGetTickCount();
        }
    }

    // Staging disabled — log interval is always log_moving_ms.
    *interval_ms = s_log_interval_ms;
}

void power_fast_motion_check(const gps_data_t *gps, const imu_data_t *imu,
                             uint32_t *interval_ms)
{
    (void)gps; (void)imu; (void)interval_ms;
    // No-op — state is always MOVING; no stage to exit.
}

void power_enter_deep_sleep(void)
{
    ESP_LOGW(TAG, "Entering deep sleep...");

#if IMU_INT_GPIO > 0
    esp_sleep_enable_ext0_wakeup((gpio_num_t)IMU_INT_GPIO, 1);
    ESP_LOGI(TAG, "Wake source: IMU INT on GPIO %d", IMU_INT_GPIO);
#else
    // Timer fallback: wake every 30 minutes
    esp_sleep_enable_timer_wakeup(30 * 60 * 1000000ULL);
    ESP_LOGI(TAG, "Wake source: timer (30 min)");
#endif

    // Button as secondary wake source
    esp_sleep_enable_ext1_wakeup(1ULL << BUTTON_GPIO, ESP_EXT1_WAKEUP_ALL_LOW);

    esp_deep_sleep_start();
}

bool power_check_wake_reason(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) return false;

    const char *reason = (cause == ESP_SLEEP_WAKEUP_EXT0)  ? "IMU motion" :
                         (cause == ESP_SLEEP_WAKEUP_EXT1)  ? "button" :
                         (cause == ESP_SLEEP_WAKEUP_TIMER) ? "timer" : "other";
    ESP_LOGI(TAG, "Woke from deep sleep: %s", reason);
    return true;
}
