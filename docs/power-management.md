# Power Management

## Progressive Power States

The power state machine adapts logging frequency and connectivity based on how long the device has been stationary:

| State | Stationary Time | Log Interval | WiFi Uploads | ESP-NOW Sync | Deep Sleep |
|-------|----------------|--------------|--------------|--------------|------------|
| Moving | — | 1 s | Yes | Yes (when static) | No |
| Stage 1 | 3+ min | 1 min | Yes | Yes | No |
| Stage 2 | 5+ min | 5 min | Periodic (5 min) | Yes | No |
| Stage 3 | 30+ min | 30 min | Periodic | Yes | Yes (IMU wake) |

Motion detected at any stage immediately restores full-rate 1 s logging.

**Detection:** GPS speed below 2 km/h combined with IMU accelerometer and gyroscope readings below their configured thresholds. All thresholds are configurable via the web UI or `sync_config.h`.

## Deep Sleep Wake Sources

When in Stage 3 the device enters deep sleep. Wake sources (evaluated in priority order):

| Wake Source | Trigger | Config |
|-------------|---------|--------|
| IMU motion interrupt | `IMU_INT_GPIO` pin goes HIGH | `IMU_INT_GPIO` in `config.h` |
| External power / charger detect | `CHARGER_DET_GPIO` pin goes HIGH | `CHARGER_DET_GPIO` in `config.h` |
| Timer | Fallback — wakes every 30 minutes | — |
| Button | BOOT button (GPIO 0) held LOW | Always active |

> `IMU_INT_GPIO` and `CHARGER_DET_GPIO` both use the ESP32 `ext0` wakeup source. Only one can be active at a time — `IMU_INT_GPIO` takes priority. If you need both simultaneously, wire the charger detect signal to a second RTC-capable GPIO and add it to the `ext1` bitmask in `power_enter_deep_sleep()`.

## DS3231 VCC Cut During Deep Sleep

By default the DS3231 stays powered from 3.3V during deep sleep (~200 µA). To eliminate this, connect the module VCC to a spare RTC-capable GPIO and set `RTC_PWR_GPIO` in `config.h`:

```c
#define RTC_PWR_GPIO  25   // any RTC-capable GPIO: 0,2,4,12,13,14,15,25,26,27,32,33
```

The firmware drives the pin HIGH to power the module and LOW (held) before deep sleep. The DS3231 falls back to its coin-cell backup (~0.84 µA). On wake, power is restored before the I2C bus is initialised.

No extra components needed — connect module VCC directly to the GPIO. Optionally remove the power LED on the DS3231 breakout board (or its current-limiting resistor) to save an additional 1–3 mA during sleep.

## Charger Detect Wiring

Set `CHARGER_DET_GPIO` in `config.h` to an available RTC-capable GPIO. GPIO 34–39 (input-only) are ideal:

```c
#define CHARGER_DET_GPIO 34   // example — match to your wiring
```

| Source | Signal | Notes |
|--------|--------|-------|
| USB VBUS (5 V) | Resistor divider to ≤3.3 V | 100 kΩ / 68 kΩ gives ~3.06 V |
| TP4056 `CHRG` pin | LOW when charging | Needs NPN inverter or use `ext1` ALL_LOW |
| IP5306 power-good / `EN` | HIGH when power applied | Direct connection works |

On wake, `power_check_wake_reason()` logs `"Woke from deep sleep: charger/external power"`.

## Power Consumption Reference

| Component | Current | Notes |
|-----------|---------|-------|
| GPS | ~50 mA | Active tracking |
| IMU | ~3.9 mA | Active |
| RTC | ~200 µA | Active; ~0.84 µA on coin-cell backup during deep sleep |
| LCD | ~30 mA / ~5 mA | Backlight on / off |
| SD Card | ~100 mA | During writes; <1 mA idle |
| ESP32 | ~240 mA | Active; reduced with sleep modes |
