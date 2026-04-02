# Hardware

## ESP32 Module

The recommended board is the Freenove ESP32-WROVER-E. Any ESP32 model with an SD card slot should work with minor code changes.

- ESP32-D0WD-V3 (WROVER or similar)
- 4 MB Flash minimum
- Built-in SD card slot (SDMMC interface)

## Sensors

### GPS — Neo-6M

| Parameter | Value |
|-----------|-------|
| RX | GPIO 22 (input from GPS TX) |
| TX | GPIO 23 (output to GPS RX) |
| Baud rate | 9600 |
| Sample rate | 5 Hz |

### RTC — DS3231

| Parameter | Value |
|-----------|-------|
| I2C Address | 0x68 |
| I2C Bus | I2C_NUM_0 (shared with IMU) |
| SDA | GPIO 18 |
| SCL | GPIO 19 |

**Optional — power-cut GPIO:** Connect the module's VCC pin to a free RTC-capable GPIO (e.g. GPIO 25) instead of the 3.3V rail. Set `RTC_PWR_GPIO` in `config.h`. The firmware drives the pin HIGH at boot and LOW before deep sleep, switching the module to its coin-cell backup (~0.84 µA). See [Power Management](power-management.md#ds3231-vcc-cut-during-deep-sleep) for full wiring details.

### IMU — MPU6050 / MPU6500 / MPU9250

| Parameter | Value |
|-----------|-------|
| I2C Address | 0x69 (AD0 HIGH) or 0x68 (AD0 LOW) |
| I2C Bus | I2C_NUM_0 (shared with RTC) |
| SDA | GPIO 18 |
| SCL | GPIO 19 |

The firmware auto-detects MPU6050, MPU6500, and MPU9250 variants at both addresses.

## LCD (Optional) — LCD1602A with I2C Backpack

| Parameter | Value |
|-----------|-------|
| I2C Address | 0x27 or 0x3F (auto-detected) |
| I2C Bus | I2C_NUM_1 (dedicated) |
| SDA | GPIO 32 |
| SCL | GPIO 33 |
| Power | 5V rail |

The LCD is fully optional — the system runs normally if no LCD is found.

## SD Card

If your ESP32 does not have a built-in SD card reader you can connect an external module.

| Parameter | Value |
|-----------|-------|
| Interface | SDMMC 1-bit mode |
| Format | FAT32 with long filename support |
| CMD | GPIO 15 |
| CLK | GPIO 14 |
| D0 | GPIO 2 |

## Pin Summary

| Component | Signal | GPIO | Notes |
|-----------|--------|------|-------|
| GPS | RX | 22 | Input from GPS TX |
| GPS | TX | 23 | Output to GPS RX |
| I2C Bus 0 | SDA | 18 | RTC + IMU |
| I2C Bus 0 | SCL | 19 | RTC + IMU |
| I2C Bus 1 | SDA | 32 | LCD (dedicated) |
| I2C Bus 1 | SCL | 33 | LCD (dedicated) |
| SD Card | CMD | 15 | — |
| SD Card | CLK | 14 | — |
| SD Card | D0 | 2 | — |
| Button | — | 0 | BOOT button (built-in) |
| Charger Detect | — | user-defined | Optional: wake from deep sleep |
| RTC Power | — | user-defined | Optional: cut DS3231 VCC in deep sleep |
