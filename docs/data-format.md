# Data Format

## Sync CSV Files

Two CSV files are maintained on the SD card:

| File | Contents |
|------|----------|
| `/sdcard/sync_data.csv` | This device's own GPS records; used for peer exchange and server upload |
| `/sdcard/sync_merged.csv` | Records received from nearby peer devices |

Both files share the same column layout:

```
timestamp,lat,lon,alt,speed,accel_x,accel_y,accel_z,device_mac
```

| Column | Description |
|--------|-------------|
| `timestamp` | Unix epoch (seconds) |
| `lat` | Latitude (decimal degrees, 6 decimals) |
| `lon` | Longitude (decimal degrees, 6 decimals) |
| `alt` | Altitude (metres, 1 decimal) |
| `speed` | Speed (km/h, 1 decimal; `0.0` if position unchanged) |
| `accel_x`, `accel_y`, `accel_z` | Acceleration (G-force, 4 decimals) |
| `device_mac` | Last 4 hex digits of the WiFi MAC address |

## Additional Files

| File | Description |
|------|-------------|
| `/sdcard/connections.csv` | Peer connection metadata log |
| `/sdcard/sync_data.bak`, `/sdcard/sync_merged.bak` | Backup archives of uploaded data (sequential numbering) |

Sync files are truncated automatically on a successful server upload. Backup archives are retained until explicitly deleted via the web UI (`/clear_bak`).
