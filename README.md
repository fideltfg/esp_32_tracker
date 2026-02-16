# ESP32 GPS Logger with IMU

A comprehensive GPS tracking system with acceleration logging, designed for ESP32 with SD card storage.

## Features

- **GPS Tracking**: Neo-6M GPS module for location, altitude, and speed data
- **IMU Data**: MPU6050 6-axis accelerometer and gyroscope
- **Real-Time Clock**: DS3231 RTC for accurate timestamps when offline
- **SD Card Storage**: Unlimited capacity with date-based CSV files
- **HTTP Server**: Web interface for file listing and download
- **WiFi Connectivity**: Automatic time synchronization via NTP
- **Low Power**: Optimized for battery operation

## Hardware

### ESP32 Module
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
  - SDA: GPIO 18
  - SCL: GPIO 19

- **IMU**: MPU6050 6-Axis IMU
  - I2C Address: 0x69 (AD0 HIGH) or 0x68 (AD0 LOW)
  - SDA: GPIO 18 (shared with RTC)
  - SCL: GPIO 19 (shared with RTC)

### SD Card
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
| I2C SDA | GPIO 18 | RTC + MPU6050 |
| I2C SCL | GPIO 19 | RTC + MPU6050 |
| SD CMD | GPIO 15 | SD Card |
| SD CLK | GPIO 14 | SD Card |
| SD D0 | GPIO 2 | SD Card |

## WiFi Configuration

Edit the WiFi credentials in `main.c`:
```c
#define WIFI_SSID "Your_SSID"
#define WIFI_PASSWORD "Your_Password"
```

The system uses WPA3-SAE authentication. If your network uses WPA2, update the authentication mode.

## Data Format

CSV files are stored in `/sdcard/gps/` with date-based filenames: `YYYY_MM_DD.csv`

### CSV Columns
```
timestamp,lat,lon,alt,speed,accel_x,accel_y,accel_z
```

- **timestamp**: ISO 8601 format (YYYY-MM-DDTHH:MM:SS)
- **lat**: Latitude (decimal degrees, 6 decimals)
- **lon**: Longitude (decimal degrees, 6 decimals)
- **alt**: Altitude (meters)
- **speed**: Speed (km/h)
- **accel_x, accel_y, accel_z**: Acceleration (G-force)

## HTTP Server

When connected to WiFi, access the device at: `http://<ESP32_IP>/`

### Endpoints
- `/` - List all GPS log files with sizes
- `/download?file=YYYY_MM_DD.csv` - Download specific file
- `/status` - System status (storage, sensors, GPS)

## Building and Flashing

### Prerequisites
- ESP-IDF v5.5.2 or compatible
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

### First Boot
1. Device initializes SD card and sensors
2. Connects to WiFi for time synchronization
3. Updates RTC with current time from NTP
4. Begins GPS logging

### GPS Logging
- Logs valid GPS fixes with IMU data every second
- Creates new file each day at midnight
- Continues logging even when WiFi is disconnected
- Uses RTC to maintain accurate timestamps offline

### LED Indicators
- Varies by board - check your specific ESP32 module

## File Management

Files are automatically organized by date. To download files:
1. Connect to the same WiFi network as ESP32
2. Open web browser to device IP address
3. Click on filename to download

## Troubleshooting

### SD Card Issues
- Ensure card is FAT32 formatted
- Check card is properly inserted
- Verify pin connections (CMD=15, CLK=14, D0=2)
- Try different SD card (some cards don't support 1-bit mode)

### GPS Not Getting Fix
- Ensure clear view of sky
- Wait 30-60 seconds for cold start
- Check antenna connection
- Verify baud rate is 9600

### RTC Not Found
- Check I2C connections (SDA=18, SCL=19)
- Verify I2C address is 0x68
- Ensure pull-up resistors present (usually internal)

### MPU6050 Not Found
- Check I2C connections (shared with RTC)
- Verify AD0 pin state:
  - AD0 HIGH (3.3V) = address 0x69
  - AD0 LOW (GND) = address 0x68
- Ensure RTC is working first (same I2C bus)

### WiFi Connection Failed
- Verify SSID and password
- Check authentication mode (WPA2/WPA3)
- Ensure router is in range
- Check MAC address not filtered

## Power Considerations

The system is designed for portable operation:
- GPS: ~50mA active
- IMU: ~3.9mA active
- RTC: ~200µA (continues on backup battery)
- SD Card: ~100mA during writes, <1mA idle
- ESP32: ~240mA active, can be reduced with sleep modes

## License

This project is based on ESP-IDF examples and is provided as-is for educational and development purposes.

## Credits

- ESP-IDF framework by Espressif Systems
- NMEA parsing for GPS data
- DS3231 RTC driver
- MPU6050 IMU driver
