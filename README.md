# ESP32 GPS Logger with IMU

A GPS tracking system with acceleration logging and peer-to-peer data synchronisation, designed for ESP32-WROVER-E with SD card storage.

![prototype picture 1](media/prototype_2.3.jpg)

This project develops a system to record metrics from static and mobile ESP32 devices within a limited area with sporadic Wi-Fi coverage.

**Concept:** Each ESP32 collects its own data (GPS and IMU values) and attempts to upload it to a server via Wi-Fi if in range. Some devices remain somewhat static while others are mobile. As ESP32 devices move close to each other, they share collected data over ESP-NOW. As mobile devices move around, all data eventually reaches Wi-Fi range and is uploaded to the server. The server handles final parsing while anti-duplication runs at share time to reduce transfer volume.

> This project is in active development. Expect changes and occasional bugs.

## Features

- **GPS Tracking**: Neo-6M at 5 Hz with EMA smoothing, quality gates, and position lock
- **IMU Data**: MPU6050/MPU6500/MPU9250 6-axis accelerometer and gyroscope with auto-detection and calibration
- **Real-Time Clock**: DS3231 for accurate timestamps when offline; auto-timezone from GPS coordinates
- **LCD Display** (Optional): LCD1602A with auto-off and button-toggle display modes
- **SD Card Storage**: CSV logging with atomic upload staging and backup archiving
- **Web Interface**: Dashboard, map, runtime config editor, and OTA firmware updates
- **Multi-AP WiFi**: Connects to the strongest configured AP; NTP sync on connect
- **HTTP Upload**: CSV-to-JSON chunked POST with backup re-upload on failure
- **ESP-NOW Peer Sync**: Bidirectional CSV exchange with nearby devices using per-peer watermark deduplication
- **Progressive Power Management**: Four stages (Moving -> Stage 3 deep sleep); wake via IMU, button, timer, or charger detect
- **NVS-Backed Configuration**: All parameters editable at runtime via web UI; factory reset support
- **WiFi Provisioning**: Open AP provisioning mode after factory reset - no reflashing required
- **OTA Firmware Updates**: Upload new firmware via the web interface

## Documentation

| Topic | File |
|-------|------|
| **Build & flash tutorial (start here)** | [docs/tutorial-build-and-flash.md](docs/tutorial-build-and-flash.md) |
| Hardware & pin wiring | [docs/hardware.md](docs/hardware.md) |
| Module architecture | [docs/architecture.md](docs/architecture.md) |
| Configuration | [docs/configuration.md](docs/configuration.md) |
| Data format (CSV schema) | [docs/data-format.md](docs/data-format.md) |
| Power management & deep sleep | [docs/power-management.md](docs/power-management.md) |
| Web interface & API | [docs/web-interface.md](docs/web-interface.md) |
| Building & flashing | [docs/building.md](docs/building.md) |
| Operation & task architecture | [docs/operation.md](docs/operation.md) |
| Troubleshooting | [docs/troubleshooting.md](docs/troubleshooting.md) |

## License

This project is provided as-is for educational and development purposes.

## Credits

- ESP-IDF framework by Espressif Systems
- NMEA parsing for GPS data
- DS3231 RTC driver
- MPU6050/MPU6500/MPU9250 IMU driver
