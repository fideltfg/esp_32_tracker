# Troubleshooting

## SD Card Issues

- Ensure the card is FAT32 formatted
- Check the card is properly inserted
- Verify pin connections (CMD=15, CLK=14, D0=2)
- Try a different SD card — some cards do not support 1-bit mode

## GPS Not Getting a Fix

- Ensure a clear view of the sky
- Wait 30–60 seconds for a cold start
- Verify the antenna is correctly connected
- Verify baud rate is 9600

## RTC Not Found

- Check I2C connections (SDA=18, SCL=19)
- Verify I2C address is 0x68
- Ensure pull-up resistors are present (usually internal)
- If `RTC_PWR_GPIO` is set, confirm the GPIO is wired to the module VCC pin and the correct number is in `config.h` — if misconfigured the module will not receive power

## MPU6050 / 6500 / 9250 Not Found

- Check I2C connections (shared with RTC on Bus 0, SDA=18, SCL=19)
- Verify AD0 pin state: HIGH (3.3V) = address 0x69, LOW (GND) = address 0x68
- The firmware auto-detects MPU6050, MPU6500, and MPU9250 variants at both addresses

## LCD1602A Not Working (Optional)

- Verify the I2C backpack is installed on the LCD
- Check I2C connections on Bus 1 (SDA=32, SCL=33)
- The firmware tries addresses 0x27 and 0x3F automatically; if not found, the system continues without LCD
- Adjust the contrast potentiometer on the I2C backpack if the display is blank

## WiFi Connection Failed

- Verify SSID and password in `sync_config.h`
- Ensure the router is in range
- Check the MAC address is not filtered by the router

## WiFi Provisioning Not Working

- Confirm the device is in provisioning mode: the serial monitor should print `Starting provisioning AP` and show the SSID `ESP32-Tracker-XXXX`
- Ensure you are connected to the `ESP32-Tracker-XXXX` network before opening the browser
- Use `http://192.168.4.1` — HTTPS and mDNS are not available in provisioning mode
- If the page does not load, disable mobile data on your phone so traffic is forced over the ESP32 AP
- To re-enter provisioning mode at any time, call `POST /api/reset_config` via the web UI and reboot

## ESP-NOW Sync Not Working

- Both devices must be on the same WiFi channel (`ESPNOW_CHANNEL` in `sync_config.h`)
- Both devices must be stationary at the same time for a sync round to occur
- Check the serial monitor for ESP-NOW init errors

## CSV Upload Failing

- Verify `UPLOAD_URL` in `sync_config.h` points to a reachable server
- Confirm the server returns HTTP 200 or 201 on a successful POST
- The device must have a WiFi IP address before upload is attempted
