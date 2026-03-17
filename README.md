   # ESP32 GPS Logger with IMU

A GPS tracking system with acceleration logging and peer-to-peer data synchronisation, designed for ESP32-WROVER-E with SD card storage.

![prtotype picture 1](media/prototype_2.3.jpg)

This a project to develop a system to record metrics from static and mobile ESP32 devcies within a limited area with sparadic Wi-Fi Covarage.

Concept: Each ESP32 will collect its own data (ATM  GPS and MPU values) and attempt to upload it to a server via Wi-Fi if in range. Some devices a will remain somewhat static while others will be mobile. As the ESP32 devices move close to other ESP32 devcies, using ESPNOW, they begin sharing their collected data. As the mobile device move around, eventualy all data will find it's way within Wi-Fi range and be uploaded to the server. The server will take care of final parsing while anti-duplication methods are run at share time to reduce data transfer times.

This project is in active development and in the final stages of prototyping before feild testing. Expect this code base to change and probably have a few bugs!

## Features

- **GPS Tracking**: Neo-6M GPS module for location, altitude, and speed data
- **IMU Data**: MPU6050/MPU6500/MPU9250 6-axis accelerometer and gyroscope
- **Real-Time Clock**: DS3231 RTC for accurate timestamps when offline
- **Automatic Timezone**: Timezone updated automatically from GPS coordinates on first valid fix
- **LCD Display** (Optional): LCD1602A shows live GPS, IMU, time, and IP address with auto-off timeout. (Requires 5v)
- **Button Control**: (Optional)Toggle LCD display modes with BOOT button; wakes LCD if auto-off is active
- **SD Card Storage**: Date-based GPS CSV files plus separate sync CSV files
- **HTTP Server**: Web interface for GPS log file listing and download
- **WiFi Connectivity**: Automatic time synchronisation via NTP and periodic CSV upload to a server
- **ESP-NOW Peer Sync**: When stationary, discovers nearby devices and exchanges CSV data bidirectionally
- **Motion Detection**: GPS speed and IMU data used together to determine stationary state before syncing


## Hardware

The hardwear used in this project should be readly availiable online and costs around $40 per device. As this project is aimed more towards the syncing and uploading of data, you can substitute any sensor set you wish. You will of cause need to update the code to reflect your changes.

### ESP32 Module
I use the Freenove ESP32-WROVER-E dev board for this project. But I think if you are willing to try this you should have no issues modifiying this code base to work with just about any ESP32 model. A SD card is required for the project to record large data sets over days or weeks without syncing or uploading.
- ESP32-D0WD-V3 (WROVER or similar with SD card slot)
- 4MB Flash minimum
- Built-in SD card slot (SDMMC interface)

### Sensors

- **GPS**: Neo-6M GPS module
  - RX: GPIO 23
  - TX: GPIO 22
  - Baud: 9600

- **RTC**: DS3231 Real-Time Clock
  - I2C Address: 0x68
  - I2C Bus: I2C_NUM_0 (shared with IMU)
  - SDA: GPIO 18
  - SCL: GPIO 19

- **IMU**: MPU6050/MPU6500/MPU9250 6-Axis IMU
  - I2C Address: 0x69 (AD0 HIGH) or 0x68 (AD0 LOW)
  - I2C Bus: I2C_NUM_0 (shared with RTC)
  - SDA: GPIO 18
  - SCL: GPIO 19
  - Auto-detects compatible sensors

- **LCD** (Optional): LCD1602A with I2C backpack
  - I2C Address: 0x27 or 0x3F (auto-detected)
  - I2C Bus: I2C_NUM_1 (dedicated, no bus contention with IMU/RTC)
  - SDA: GPIO 32
  - SCL: GPIO 33
  - Displays GPS fix, speed, altitude, and IMU data
  - Auto-off after 30 seconds of inactivity (configurable via `LCD_TIMEOUT_SEC`)
  - Pulls power from the 5v rail

### SD Card
If your ESP32 does not have a built in SD card reader, you can connect an external reader.

- SDMMC interface (1-bit mode)
  - CMD: GPIO 15
  - CLK: GPIO 14
  - D0: GPIO 2
- FAT32 formatted
- Long filename support enabled

## Pin Configuration

| Component | Pin | GPIO |
|-----------|-----|------|
| GPS RX | GPIO 22 | Input from GPS TX |
| GPS TX | GPIO 23 | Output to GPS RX |
| I2C Bus 0 SDA | GPIO 18 | RTC + IMU |
| I2C Bus 0 SCL | GPIO 19 | RTC + IMU |
| I2C Bus 1 SDA | GPIO 32 | LCD (dedicated) |
| I2C Bus 1 SCL | GPIO 33 | LCD (dedicated) |
| SD CMD | GPIO 15 | SD Card |
| SD CLK | GPIO 14 | SD Card |
| SD D0 | GPIO 2 | SD Card |
| Button | GPIO 0 | BOOT button (built-in) |

## Configuration

All WiFi credentials, the upload endpoint, and ESP-NOW parameters are set in `main/sync_config.h`:

```c
#define WIFI_SSID        "Your_SSID"
#define WIFI_PASSWORD    "Your_Password"
#define WIFI_MAX_RETRY   3

#define UPLOAD_URL       "http://<server-ip>/api/esp/upload"
```

The default timezone is set in `main/main.c` and is automatically overridden by GPS-derived timezone on first valid fix:

```c
#define TIMEZONE "MST7MDT,M3.2.0,M11.1.0"  // US Mountain — edit as needed
```

## Data Format

### GPS Daily Logs

Stored in `/sdcard/gps/` with date-based filenames: `YYYY_MM_DD.csv`

```
timestamp,lat,lon,alt,speed,accel_x,accel_y,accel_z
```

| Column | Description |
|--------|-------------|
| timestamp | ISO 8601 format (YYYY-MM-DDTHH:MM:SS) |
| lat | Latitude (decimal degrees, 6 decimals) |
| lon | Longitude (decimal degrees, 6 decimals) |
| alt | Altitude (meters) |
| speed | Speed (km/h) |
| accel_x, accel_y, accel_z | Acceleration (G-force) |

### Sync Data Files

Two additional files are maintained on the SD card for the ESP-NOW sync subsystem:

- `/sdcard/sync_data.csv` — own records in the sync schema, used for peer exchange and server upload
- `/sdcard/sync_merged.csv` — records received from nearby peer devices

Both files share the same column layout:

```
timestamp,lat,lon,accel_x,accel_y,accel_z,device_mac
```

`device_mac` contains the last 4 hex digits of the device's WiFi MAC address to identify record origin.

## HTTP Server

When connected to WiFi, access the device at `http://<ESP32_IP>/`

NOTE: The service the data is uploaded to is not part of this project. You will need to provide a URL that can accept the JSON data and return success (200) to trigger file trucation.


### Endpoints

| Endpoint | Description |
|----------|-------------|
| `/` | Lists all GPS log files with sizes |
| `/download?file=YYYY_MM_DD.csv` | Downloads a specific GPS log file |
| `/status` | Returns system status as JSON (SD card, sensors, GPS) |

## Building and Flashing

### Prerequisites
- ESP-IDF v5.5.3 or compatible
- Python 3.7+

### Build
```bash
idf.py build
```

### Flash
```bash
idf.py -p COM3 flash monitor
```

### Configuration
```bash
idf.py menuconfig
```

Important settings:
- `Component config → FAT Filesystem → Long filename support` = **Heap**
- `Component config → ESP32-specific → Main XTAL frequency` = **40 MHz**

## Operation

### Task Architecture

The firmware runs two pinned FreeRTOS tasks:

- **`gps_logging_task`** (Core 1): Reads GPS UART, reads IMU, writes GPS daily CSV, updates LCD
- **`sync_state_machine_task`** (Core 0): Handles ESP-NOW peer discovery, CSV exchange, and WiFi upload

### First Boot
1. I2C buses initialised (Bus 0: RTC + IMU, Bus 1: LCD)
2. RTC time loaded into system clock
3. SD card mounted; sync CSV files created if absent
4. WiFi connected; NTP time sync performed and RTC updated
5. HTTP server started
6. GPS logging task and sync state machine task started

### GPS Logging
- Logs valid GPS fixes with IMU data once per second
- Creates a new daily file at midnight
- Timezone is initially set from `TIMEZONE` define then updated automatically from GPS coordinates on first valid fix
- Continues logging when WiFi is disconnected; RTC maintains accurate timestamps

### ESP-NOW Peer Synchronisation

When the device is determined to be stationary (GPS speed below 2 km/h and IMU confirms no movement), the sync state machine activates:

1. Broadcasts an ANNOUNCE frame over ESP-NOW to discover nearby devices
2. Responds to peer SYNC_REQ by streaming own `sync_data.csv` in chunks
3. Receives peer CSV data and appends it to `sync_merged.csv`
4. Periodically uploads both `sync_data.csv` and `sync_merged.csv` to `UPLOAD_URL` via HTTP POST
5. On successful upload, the files truncated.
6. When the device starts moving, the sync loop exits and waits for the next stationary period

### LCD Display Modes

If an LCD1602A is connected, press the BOOT button (GPIO 0) to toggle between display modes. The button has 300 ms debounce. If the LCD has auto-off'd, pressing the button wakes it.

**Mode 1: GPS/IMU Data**
- Line 1: Speed (km/h) and Altitude (m)
- Line 2: Latitude and Longitude (with GPS fix), or Accelerometer X, Y, Z (without fix)

**Mode 2: Time and IP Address**
- Line 1: Current local time (HH:MM:SS)
- Line 2: WiFi IP address

The LCD turns off automatically after 30 seconds of inactivity. The timeout is controlled by `LCD_TIMEOUT_SEC` in `main.c` (set to 0 to disable auto-off).

## File Management
Note this is not secure, is for testing only and should not be used in production builds.

Log files are automatically organised by date. To manualy download avalaible files:
1. Connect to the same WiFi network as the ESP32
2. Open a web browser to the device IP address
3. Click on a filename to download

Sync files (`sync_data.csv`, `sync_merged.csv`) are managed automatically and deleted after a successful server upload.

## Troubleshooting

### SD Card Issues
- Ensure card is FAT32 formatted
- Check card is properly inserted
- Verify pin connections (CMD=15, CLK=14, D0=2)
- Try a different SD card (some cards do not support 1-bit mode)

### GPS Not Getting Fix
- Ensure clear view of sky
- Wait 30-60 seconds for cold start
- Check antenna connection
- Verify baud rate is 9600
- Check antenna is conencted correctly.

### RTC Not Found
- Check I2C connections (SDA=18, SCL=19)
- Verify I2C address is 0x68
- Ensure pull-up resistors are present (usually internal)

### MPU6050/6500/9250 Not Found
- Check I2C connections (shared with RTC on Bus 0)
- Verify AD0 pin state: AD0 HIGH (3.3V) = address 0x69, AD0 LOW (GND) = address 0x68
- Code auto-detects MPU6050, MPU6500, and MPU9250 variants

### LCD1602A Not Working (Optional)
- Verify I2C backpack is installed on LCD
- Check I2C connections on Bus 1 (SDA=32, SCL=33)
- Code tries addresses 0x27 and 0x3F automatically
- If not found, the system continues without LCD
- Adjust the contrast potentiometer on the I2C backpack if the display is blank

### WiFi Connection Failed
- Verify SSID and password in `sync_config.h`
- Ensure router is in range
- Check MAC address is not filtered

### ESP-NOW Sync Not Working
- Both devices must be on the same WiFi channel (`ESPNOW_CHANNEL` in `sync_config.h`)
- Both devices must be stationary at the same time for a sync round to occur
- Check serial monitor output for ESP-NOW init errors

### CSV Upload Failing
- Verify `UPLOAD_URL` in `sync_config.h` points to a reachable server
- Confirm the server returns HTTP 200 or 201 on a successful POST
- Device must have a WiFi IP address before upload is attempted

## Power Considerations

The system is designed for portable operation:
- GPS: ~50mA active
- IMU: ~3.9mA active
- RTC: ~200uA (continues on backup battery)
- LCD: ~30mA with backlight, ~5mA without
- SD Card: ~100mA during writes, <1mA idle
- ESP32: ~240mA active, can be reduced with sleep modes

## License

This project is provided as-is for educational and development purposes.

## Credits

- ESP-IDF framework by Espressif Systems
- NMEA parsing for GPS data
- DS3231 RTC driver
- MPU6050/MPU6500/MPU9250 IMU driver
