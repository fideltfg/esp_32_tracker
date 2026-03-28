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
#include "driver/gpio.h"
#include "driver/rtc_io.h"

static const char *TAG = "POWER";

// RTC attrs retained so layout is unchanged across flash cycles,
// but staging is not restored — device always boots into MOVING.
static RTC_DATA_ATTR power_state_t  s_rtc_state         = POWER_STATE_MOVING;
static RTC_DATA_ATTR uint32_t       s_rtc_stationary_ms  = 0;

static volatile power_state_t s_state = POWER_STATE_MOVING;
static uint32_t s_log_interval_ms  = 0;
static uint32_t s_stationary_ms    = 0;

// Debounce
#define MOTION_CONFIRM_TICKS  2
#define LOCK_MOTION_CONFIRM   3
static uint8_t s_motion_count    = 0;
//static uint8_t s_fast_motion     = 0;
static bool    s_was_stationary  = false;

// Position lock / stationary timing
static TickType_t s_lock_release_tick = 0;
static TickType_t s_stationary_since  = 0;

// Grace period: minimum uptime before allowing deep sleep re-entry (60 s)
#define DEEP_SLEEP_GRACE_MS 60000
//static TickType_t s_boot_tick = 0;

// ── Helpers ──────────────────────────────────────────────────────────────────

static void log_transition(power_state_t from, power_state_t to)
{
    static const char *names[] = { "MOVING", "STAGE1", "STAGE2", "STAGE3" };
    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "  POWER STATE: %s --> %s", names[from], names[to]);
    ESP_LOGW(TAG, "========================================");
}

// ── Public API ───────────────────────────────────────────────────────────────

void power_rtc_pwr_init(void)
{
#if RTC_PWR_GPIO > 0
    rtc_gpio_hold_dis((gpio_num_t)RTC_PWR_GPIO);  // release any hold from previous sleep
    rtc_gpio_init((gpio_num_t)RTC_PWR_GPIO);
    rtc_gpio_set_direction((gpio_num_t)RTC_PWR_GPIO, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pullup_dis((gpio_num_t)RTC_PWR_GPIO);
    rtc_gpio_pulldown_dis((gpio_num_t)RTC_PWR_GPIO);
    rtc_gpio_set_level((gpio_num_t)RTC_PWR_GPIO, 1);  // HIGH → DS3231 on VCC
    vTaskDelay(pdMS_TO_TICKS(5));  // allow DS3231 VCC to settle before I2C access
    ESP_LOGI(TAG, "DS3231 VCC on (GPIO %d HIGH)", RTC_PWR_GPIO);
#endif
}

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

    //float am = sqrtf(imu->accel_x*imu->accel_x + imu->accel_y*imu->accel_y +
                     //imu->accel_z*imu->accel_z);
    //float gm = sqrtf(imu->gyro_x*imu->gyro_x + imu->gyro_y*imu->gyro_y +
                     //imu->gyro_z*imu->gyro_z);

    //bool imu_stat = (fabsf(am - 1.0f) < cfg->imu_accel_dev) && (gm < cfg->imu_gyro_dps);
    //bool gps_stat = !gps->valid || (gps->speed < cfg->gps_static_kmh);
    //bool imu_only = gps_is_position_locked() || (s_state >= POWER_STATE_STAGE2);
    //bool stationary = imu_stat && (imu_only || gps_stat);

    if (imu_is_static(imu)){//} && gps_stat) {
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

    // Put the IMU to sleep and clear its latched interrupt.
    imu_prepare_sleep();

    // Wait longer than one IMU sample period (8 ms at 125 Hz) to guarantee
    // any in-flight data-ready pulse has finished before we arm ext0.
    vTaskDelay(pdMS_TO_TICKS(20));

#if IMU_INT_GPIO > 0
    // Configure RTC pull-down on the INT pin before arming ext0.
    // This is essential: without it a floating or disconnected pin reads HIGH
    // and causes an immediate spurious wakeup.  The pull-down ensures the pin
    // is LOW unless the IMU actively drives it HIGH.
    rtc_gpio_init((gpio_num_t)IMU_INT_GPIO);
    rtc_gpio_set_direction((gpio_num_t)IMU_INT_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_dis((gpio_num_t)IMU_INT_GPIO);
    rtc_gpio_pulldown_en((gpio_num_t)IMU_INT_GPIO);

    // Verify the INT pin is actually LOW before arming ext0.
    if (gpio_get_level((gpio_num_t)IMU_INT_GPIO) == 1) {
        ESP_LOGW(TAG, "IMU INT still HIGH after pull-down — falling back to timer wake");
        esp_sleep_enable_timer_wakeup(30 * 60 * 1000000ULL);
    } else {
        esp_sleep_enable_ext0_wakeup((gpio_num_t)IMU_INT_GPIO, 1);
        ESP_LOGI(TAG, "Wake source: IMU INT on GPIO %d", IMU_INT_GPIO);
    }
#elif CHARGER_DET_GPIO > 0
    // External power / charger detect wake (ext0, active HIGH).
    // The IMU path is not wired, so ext0 is free for this purpose.
    esp_sleep_enable_ext0_wakeup((gpio_num_t)CHARGER_DET_GPIO, 1);
    ESP_LOGI(TAG, "Wake source: charger detect on GPIO %d", CHARGER_DET_GPIO);
#else
    // Timer fallback: wake every 30 minutes
    esp_sleep_enable_timer_wakeup(30 * 60 * 1000000ULL);
    ESP_LOGI(TAG, "Wake source: timer (30 min)");
#endif

    // Button as secondary wake source (active LOW)
    esp_sleep_enable_ext1_wakeup(1ULL << BUTTON_GPIO, ESP_EXT1_WAKEUP_ALL_LOW);

#if RTC_PWR_GPIO > 0
    // Cut VCC to the DS3231 module — it falls back to VBAT backup (~0.84 µA).
    // Hold the GPIO LOW through deep sleep so VCC stays cut.
    rtc_gpio_set_level((gpio_num_t)RTC_PWR_GPIO, 0);
    rtc_gpio_hold_en((gpio_num_t)RTC_PWR_GPIO);
    ESP_LOGI(TAG, "DS3231 VCC cut (GPIO %d LOW, held)", RTC_PWR_GPIO);
#endif

    esp_deep_sleep_start();
}

bool power_check_wake_reason(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) return false;

#if IMU_INT_GPIO > 0
    const char *ext0_label = "IMU motion";
#elif CHARGER_DET_GPIO > 0
    const char *ext0_label = "charger/external power";
#else
    const char *ext0_label = "ext0";
#endif

    const char *reason = (cause == ESP_SLEEP_WAKEUP_EXT0)  ? ext0_label :
                         (cause == ESP_SLEEP_WAKEUP_EXT1)  ? "button" :
                         (cause == ESP_SLEEP_WAKEUP_TIMER) ? "timer" : "other";
    ESP_LOGI(TAG, "Woke from deep sleep: %s", reason);
    return true;
}
