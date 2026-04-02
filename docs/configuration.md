# Configuration

## Initial Setup

Copy the example config and fill in your credentials:

```bash
cp main/sync_config.example.h main/sync_config.h
```

Edit `main/sync_config.h` with your WiFi credentials and upload endpoint. This file defines hardware pin assignments and compile-time defaults. On first boot these defaults are written to NVS; subsequent changes can be made at runtime via the web UI.

```c
// WiFi AP list (up to 4 APs — device scans and connects to strongest)
#define DEFAULT_WIFI_AP1_SSID     "Your_SSID"
#define DEFAULT_WIFI_AP1_PASS     "Your_Password"
#define DEFAULT_WIFI_MAX_RETRY    3

// Upload endpoint
#define DEFAULT_UPLOAD_URL        "https://<server>/api/esp/upload"
```

## WiFi Provisioning (After Factory Reset)

If the stored WiFi credentials are blank or contain placeholder values (e.g. after calling `/api/reset_config`), the device enters **provisioning mode** at boot.

1. The ESP32 starts a soft AP with SSID `ESP32-Tracker-XXXX` (XXXX = last 4 hex digits of MAC). The AP is open (no password).
2. Connect your phone or laptop to that network.
3. Open a browser to **`http://192.168.4.1`** — the device redirects to the setup page automatically.
4. Enter your WiFi SSID and password and tap **Save**.
5. The device saves credentials to NVS and reboots into normal STA mode.

Provisioning mode is fully automatic — no reflashing or serial access is required. Once valid credentials are stored, the provisioning AP is not started again.

## Runtime Configuration

Once running, all tunable parameters can be changed via the web UI at `http://<ESP32_IP>/config`:

- WiFi AP credentials (up to 4 APs)
- Upload URL
- Timezone
- GPS quality filters (HDOP, min satellites, outlier distance, EMA alpha)
- Motion detection thresholds (speed, accel deviation, gyro threshold)
- Logging intervals per power stage
- Stationary stage thresholds
- LCD timeout
- Sync state machine parameters

Use the `/api/reset_config` endpoint to restore all values to compile-time defaults.
