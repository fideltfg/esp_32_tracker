# Architecture

The firmware is split into dedicated modules, each with a single responsibility:

| Module | Files | Purpose |
|--------|-------|---------|
| GPS | `gps.c/h` | UART driver, NMEA parsing, EMA filtering, position lock, timezone lookup |
| IMU | `imu.c/h` | MPU6050 I2C driver, calibration, static detection |
| RTC | `rtc.c/h` | DS3231 I2C driver, time get/set |
| LCD | `lcd.c/h` | LCD1602A display, auto-off, ISR-safe wake, periodic resync |
| Power | `power.c/h` | Progressive power state machine, deep sleep with IMU wake |
| Config | `config.c/h` | NVS-backed runtime configuration with defaults from `sync_config.h` |
| WiFi Manager | `wifi_manager.c/h` | Multi-AP scanning, connection management, IP reporting |
| Upload | `upload.c/h` | HTTP CSV upload, OTA firmware flashing |
| Web Server | `webserver.c/h` | HTTP server: dashboard, live GPS map, config, OTA |
| SD Storage | `sd_storage.c/h` | CSV file management, atomic staging, mutex-based concurrent access |
| ESP-NOW Sync | `espnow_sync.c/h` | Peer discovery, chunked data exchange, watermark tracking |
| Data Merge | `data_merge.c/h` | Peer CSV deduplication and merge |

See [Operation](operation.md) for how these modules interact at runtime.
