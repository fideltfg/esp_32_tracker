// power.c — Progressive power state machine + deep sleep.

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

// Persist across deep sleep so we resume at STAGE3 without re-waiting
static RTC_DATA_ATTR power_state_t  s_rtc_state         = POWER_STATE_MOVING;
static RTC_DATA_ATTR uint32_t       s_rtc_stationary_ms  = 0;

static volatile power_state_t s_state = POWER_STATE_MOVING;
static uint32_t s_stationary_ms   = 0;
static uint32_t s_log_interval_ms = 0;

// Debounce
#define MOTION_CONFIRM_TICKS 5
static uint8_t s_motion_count    = 0;
static uint8_t s_fast_motion     = 0;
static bool    s_was_stationary  = false;

// Position lock timing
static TickType_t s_lock_release_tick = 0;

// Grace period: minimum uptime before allowing deep sleep re-entry (60 s)
#define DEEP_SLEEP_GRACE_MS 60000
static TickType_t s_boot_tick = 0;

// ── Helpers ──────────────────────────────────────────────────────────────────

static void log_transition(power_state_t from, power_state_t to)
{
    static const char *names[] = { "MOVING", "STAGE1", "STAGE2", "STAGE3" };
    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "  POWER STATE: %s --> %s", names[from], names[to]);
    ESP_LOGW(TAG, "========================================");
}

// ── Public API ───────────────────────────────────────────────────────────────

void power_init(void)
{
    const tracker_config_t *cfg = config_get();

    // Restore state if waking from deep sleep
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED &&
        s_rtc_state >= POWER_STATE_STAGE3) {
        s_state           = s_rtc_state;
        s_stationary_ms   = s_rtc_stationary_ms;
        s_log_interval_ms = cfg->log_stage3_ms;
        ESP_LOGI(TAG, "Restored STAGE3 from deep sleep (stationary %lu ms)",
                 (unsigned long)s_stationary_ms);
    } else {
        s_state           = POWER_STATE_MOVING;
        s_log_interval_ms = cfg->log_moving_ms;
    }

    s_lock_release_tick = xTaskGetTickCount();
    s_boot_tick = s_lock_release_tick;
}

power_state_t power_get_state(void)       { return s_state; }
uint32_t power_get_log_interval_ms(void) { return s_log_interval_ms; }

bool power_deep_sleep_ready(void)
{
    if (s_state < POWER_STATE_STAGE3) return false;
    uint32_t uptime_ms = (uint32_t)((xTaskGetTickCount() - s_boot_tick) * portTICK_PERIOD_MS);
    return uptime_ms >= DEEP_SLEEP_GRACE_MS;
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

    if (s_state >= POWER_STATE_STAGE2)
        return !imu_ok || imu_static;

    if (gps_ok && imu_ok) return gps_static && imu_static;
    if (gps_ok)           return gps_static;
    return imu_static;
}

void power_evaluate(const gps_data_t *gps, const imu_data_t *imu,
                    uint32_t *interval_ms)
{
    const tracker_config_t *cfg = config_get();

    float am = sqrtf(imu->accel_x*imu->accel_x + imu->accel_y*imu->accel_y +
                     imu->accel_z*imu->accel_z);
    float gm = sqrtf(imu->gyro_x*imu->gyro_x + imu->gyro_y*imu->gyro_y +
                     imu->gyro_z*imu->gyro_z);

    bool imu_stat = (fabsf(am - 1.0f) < cfg->imu_accel_dev) && (gm < cfg->imu_gyro_dps);
    bool gps_stat = !gps->valid || (gps->speed < cfg->gps_static_kmh);
    bool imu_only = gps_is_position_locked() || (s_state >= POWER_STATE_STAGE2);
    bool stationary = imu_stat && (imu_only || gps_stat);

    if (stationary) {
        s_stationary_ms += s_log_interval_ms;
        s_motion_count = 0;

        // Position lock evaluation
        if (!gps_is_position_locked()) {
            uint32_t relock_ms = (s_state >= POWER_STATE_STAGE3) ? cfg->relock_stage3_ms :
                                 (s_state == POWER_STATE_STAGE2) ? cfg->relock_stage2_ms :
                                                                    cfg->relock_stage1_ms;
            uint32_t elapsed = (uint32_t)((xTaskGetTickCount() - s_lock_release_tick) * portTICK_PERIOD_MS);
            if (elapsed >= relock_ms) gps_lock_position();
        }

        power_state_t old = s_state;

        if (s_stationary_ms >= cfg->stage3_thresh_ms && s_state != POWER_STATE_STAGE3) {
            s_state = POWER_STATE_STAGE3;
            s_log_interval_ms = cfg->log_stage3_ms;
            s_rtc_state        = s_state;
            s_rtc_stationary_ms = s_stationary_ms;
            log_transition(old, s_state);
        } else if (s_stationary_ms >= cfg->stage2_thresh_ms && s_state < POWER_STATE_STAGE2) {
            s_state = POWER_STATE_STAGE2;
            s_log_interval_ms = cfg->log_stage2_ms;
            log_transition(old, s_state);
        } else if (s_stationary_ms >= cfg->stage1_thresh_ms && s_state < POWER_STATE_STAGE1) {
            s_state = POWER_STATE_STAGE1;
            s_log_interval_ms = cfg->log_stage1_ms;
            log_transition(old, s_state);
        }

        s_was_stationary = true;
    } else {
        s_motion_count++;
        if (s_motion_count >= MOTION_CONFIRM_TICKS) {
            if (s_was_stationary || s_state != POWER_STATE_MOVING) {
                power_state_t old = s_state;
                s_state = POWER_STATE_MOVING;
                s_log_interval_ms = cfg->log_moving_ms;
                log_transition(old, s_state);
            }
            s_stationary_ms = 0;
            s_rtc_state         = POWER_STATE_MOVING;
            s_rtc_stationary_ms = 0;
            if (gps_is_position_locked()) {
                gps_unlock_position();
                s_lock_release_tick = xTaskGetTickCount();
            }
        }
        s_was_stationary = false;
    }

    *interval_ms = s_log_interval_ms;
}

void power_fast_motion_check(const gps_data_t *gps, const imu_data_t *imu,
                             uint32_t *interval_ms)
{
    if (s_state == POWER_STATE_MOVING) {
        s_fast_motion = 0;
        return;
    }

    // Need at least one working sensor
    if (!gps->valid && !imu_is_available()) {
        s_fast_motion = 0;
        return;
    }

    const tracker_config_t *cfg = config_get();
    float am = sqrtf(imu->accel_x*imu->accel_x + imu->accel_y*imu->accel_y +
                     imu->accel_z*imu->accel_z);
    float gm = sqrtf(imu->gyro_x*imu->gyro_x + imu->gyro_y*imu->gyro_y +
                     imu->gyro_z*imu->gyro_z);

    bool imu_moving = imu_is_available() &&
                      ((fabsf(am - 1.0f) >= cfg->imu_accel_dev) ||
                       (gm >= cfg->imu_gyro_dps));
    bool gps_moving = gps->valid && !gps_is_position_locked() &&
                      (gps->speed >= cfg->gps_static_kmh);
    bool moving = imu_moving || gps_moving;

    if (moving) {
        s_fast_motion++;
        if (s_fast_motion >= MOTION_CONFIRM_TICKS) {
            power_state_t old = s_state;
            s_state = POWER_STATE_MOVING;
            s_log_interval_ms  = cfg->log_moving_ms;
            s_stationary_ms    = 0;
            s_motion_count     = 0;
            s_was_stationary   = false;
            s_rtc_state         = POWER_STATE_MOVING;
            s_rtc_stationary_ms = 0;
            if (gps_is_position_locked()) {
                gps_unlock_position();
                s_lock_release_tick = xTaskGetTickCount();
            }
            log_transition(old, s_state);
            s_fast_motion = 0;
            *interval_ms = s_log_interval_ms;
        }
    } else {
        s_fast_motion = 0;
    }
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
