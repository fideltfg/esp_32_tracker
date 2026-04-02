# Operation

## Task Architecture

The firmware runs two FreeRTOS tasks pinned to separate cores:

| Task | Core | Responsibility |
|------|------|----------------|
| `gps_logging_task` | Core 1 | Samples GPS at 5 Hz, accumulates IMU readings, applies EMA position filtering, evaluates power state transitions, writes CSV at the current log interval, updates the LCD |
| `sync_state_machine_task` | Core 0 | Waits for the device to become stationary, then alternates between WiFi upload attempts and ESP-NOW peer discovery/exchange rounds |

## First Boot Sequence

1. NVS initialised; config loaded (or defaults written)
2. I2C buses initialised (Bus 0: RTC + IMU, Bus 1: LCD)
3. LCD splash screen shown ("GPS Tracker v2")
4. IMU calibrated (device must be still)
5. RTC time loaded into system clock
6. SD card mounted; sync CSV files created if absent
7. MAC suffix derived for CSV record identification
8. GPS UART initialised
9. Power manager initialised; deep sleep wake reason checked
10. WiFi credentials check — if blank or placeholder, provisioning mode is started (see [Configuration](configuration.md#wifi-provisioning-after-factory-reset)); otherwise connects to the strongest configured AP and starts NTP sync
11. HTTP server started
12. `gps_logging_task` (Core 1) and `sync_state_machine_task` (Core 0) launched

## GPS Logging

- Valid GPS fixes are logged with IMU data at the current interval (1 s when moving)
- Timezone is initially set from the `TIMEZONE` define then updated automatically from GPS coordinates on first valid fix
- Logging continues when WiFi is disconnected; the RTC maintains accurate timestamps
- Duplicate suppression: a record is skipped if position and altitude are unchanged, unless a heartbeat is due (forced write every 60 s)

## ESP-NOW Peer Synchronisation

When the device is stationary, the sync state machine activates:

1. Broadcasts an ANNOUNCE frame over ESP-NOW to discover nearby devices
2. Responds to peer SYNC_REQ by streaming `sync_data.csv` in chunks
3. Receives peer CSV data and appends it to `sync_merged.csv`
4. Periodically uploads both files to the configured `UPLOAD_URL` via HTTP POST
5. On a successful upload, files are truncated
6. When the device starts moving, the sync loop exits and waits for the next stationary period

## LCD Display Modes

Press the BOOT button (GPIO 0) to cycle between display modes. 300 ms debounce. Pressing the button also wakes the LCD if auto-off is active.

| Mode | Line 1 | Line 2 |
|------|--------|--------|
| GPS/IMU | Speed (km/h) and Altitude (m) | Lat/Lon (with GPS fix) or Accel X/Y/Z (without fix) |
| Time/IP | Current local time (HH:MM:SS) | WiFi IP address |

Auto-off triggers after 30 seconds of inactivity (configurable with `LCD_TIMEOUT_SEC`; set to 0 to disable).

## File Management

> Note: The file server is for development and testing only and is not secured. Do not use it in production builds.

Log files are accessible via the web dashboard. To download a file:

1. Connect to the same WiFi network as the ESP32.
2. Open a browser to the device IP address.
3. Click a filename to download.

Sync files are truncated automatically after a successful server upload. Backup archives are retained until deleted via `/clear_bak`.
