# Web Interface

When connected to WiFi, access the device at `http://<ESP32_IP>/`.

> The upload service at the configured `UPLOAD_URL` is not part of this project. You must provide a server endpoint that accepts the JSON POST and returns HTTP 200 or 201 to trigger file truncation.

## Pages

| Page | Path | Description |
|------|------|-------------|
| Dashboard | `/` | File listing with sizes and backup management (redirects to Provision page if WiFi is not configured) |
| Provision | `/provision` | WiFi setup form — only served while in provisioning mode |
| Map | `/map` | Map visualisation of GPS data |
| Config | `/config` | Runtime configuration editor |
| OTA | `/ota` | Firmware upload page |

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/status` | GET | System status JSON (SD card, sensors, GPS) |
| `/download` | GET | Download a specific file (`?path=/sdcard/...`) |
| `/clear` | POST | Clear sync data files |
| `/reupload_bak` | POST | Re-upload all backup archive files |
| `/clear_bak` | POST | Delete backup archive files |
| `/api/gps` | GET | Live GPS data as JSON |
| `/api/system` | GET | System information as JSON |
| `/api/config` | GET | Current configuration as JSON |
| `/api/config` | POST | Update configuration values |
| `/api/reset_config` | POST | Reset all config to compile-time defaults |
| `/provision` | POST | Save WiFi credentials (provisioning mode only) and reboot |
| `/ota` | POST | Upload and flash new firmware |
