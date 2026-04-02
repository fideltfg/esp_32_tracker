# Tutorial: Build and Flash the ESP32 GPS Tracker

This tutorial walks you through installing the toolchain, configuring the firmware, building it, and flashing it to your ESP32 for the first time.

---

## Step 1 — Install ESP-IDF

The project requires **ESP-IDF v5.5.2** or greater release.

### Windows

1. Download the [ESP-IDF Windows Installer](https://dl.espressif.com/dl/esp-idf/) (choose the offline installer for the relevant version).
2. Run the installer and follow the prompts. Accept the default install path unless you have a reason to change it.
3. When the installer finishes, open the **ESP-IDF Command Prompt** or **ESP-IDF PowerShell** shortcut it adds to your Start menu. All subsequent commands in this tutorial should be run from that shell — it has all required environment variables pre-configured.

### Linux / macOS

Follow the [official Espressif get-started guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html):

```bash
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.5.2
./install.sh esp32
. ./export.sh
```

Add `. ~/esp/esp-idf/export.sh` to your shell profile so the environment is available in every new terminal.

### Verify the installation

```bash
idf.py --version
```

You should see something like `ESP-IDF v5.5.2`.

---

## Step 2 — Get the Project

Clone the repository (or copy the project folder) to your machine. The instructions below assume the project is at `~/esp/esp_32_tracker` on Linux/macOS or `D:\ESP\esp_32_tracker` on Windows — adjust paths to match your setup.

```bash
cd ~/esp
git clone <repository_url> esp_32_tracker
cd esp_32_tracker
```

---

## Step 3 — Create Your Config File

The project ships with an example configuration file. **The firmware will not compile without a real `sync_config.h`.**

```bash
cp main/config.example.h main/config.h
```

Open `main/config.h` in your editor and fill in at minimum:

```c
// WiFi credentials (up to 4 APs)
#define DEFAULT_WIFI_AP1_SSID     "Your_Network_Name"
#define DEFAULT_WIFI_AP1_PASS     "Your_Password"

// Upload endpoint — set to your server, or leave as placeholder for now
#define DEFAULT_UPLOAD_URL        "https://<your-server>/api/esp/upload"
```

All other values have sensible defaults. See [Configuration](configuration.md) for the full list of parameters and what they control.

---

## Step 4 — Set the Target Chip

Tell ESP-IDF you are building for ESP32 (not ESP32-S2, S3, C3, etc.):

```bash
idf.py set-target esp32
```

This only needs to be done once, or whenever you switch target chips.

---

## Step 5 — Apply Required menuconfig Settings

Two non-default settings are required:

```bash
idf.py menuconfig
```

Navigate to and change the following:

| Menu Path | Setting | Required Value |
|-----------|---------|---------------|
| `Component config` > `FAT Filesystem support` > `Long filename support` | — | **Heap buffer** |
| `Component config` > `ESP32-specific` > `Main XTAL frequency` | — | **40 MHz** |

Save with **S**, then exit with **Q**.

> **Why these?** Long filename support allows the FAT driver to handle filenames longer than 8.3 characters on the SD card. The 40 MHz crystal is required to match the Freenove WROVER-E board's oscillator; using 26 MHz will cause the baud rate to be wrong and the serial monitor will be garbled.

---

## Step 6 — Build

```bash
idf.py build
```

On a first build this takes several minutes while the full ESP-IDF component tree compiles. Subsequent builds are incremental and much faster.

A successful build ends with output similar to:

```
Project build complete. To flash, run:
  idf.py flash
or
  idf.py -p PORT flash
```

If the build fails, see [Troubleshooting](troubleshooting.md) or check that your `config.h` is present and the IDF target is set to `esp32`.

---

## Step 7 — Connect Your ESP32

1. Plug the ESP32 into your PC via USB.
2. Find the serial port it enumerates as:
   - **Windows**: Open Device Manager > Ports (COM & LPT). Look for **Silicon Labs CP210x** or **CH340** — note the COM number (e.g. `COM6`).
   - **Linux**: Usually `/dev/ttyUSB0` or `/dev/ttyACM0`. Run `ls /dev/tty*` before and after plugging in to identify it.
   - **macOS**: Usually `/dev/cu.usbserial-*`. Run `ls /dev/cu.*` to list candidates.

3. On Linux you may need to add yourself to the `dialout` group so you have permission to access the port:

   ```bash
   sudo usermod -aG dialout $USER
   # Log out and back in for the change to take effect
   ```

---

## Step 8 — Flash the Firmware

Replace `COM6` with your actual port:

```bash
idf.py -p COM6 flash
```

This erases only the necessary flash partitions and writes the bootloader, partition table, and application. The flash process takes around 15–30 seconds.

> **Board won't enter flash mode?** Hold the **BOOT** button, press **EN** (reset), then release BOOT. Some boards do this automatically; others require manual intervention.

---

## Step 9 — Open the Serial Monitor

Watch the boot log to confirm everything initialised correctly:

```bash
idf.py -p COM6 monitor
```

To flash and open the monitor in a single command:

```bash
idf.py -p COM6 flash monitor
```

Press **Ctrl+]** to exit the monitor.

### Expected first-boot output

```
I (xxx) main: NVS initialised, config loaded
I (xxx) imu: MPU6050 found at 0x69, calibrating...
I (xxx) rtc: DS3231 time loaded: 2026-01-01 00:00:00
I (xxx) sd_storage: SD card mounted, FAT volume size: ...
I (xxx) gps: UART initialised at 9600 baud on pins 22/23
I (xxx) wifi_manager: Connecting to 'Your_Network_Name'...
I (xxx) wifi_manager: Got IP: 192.168.1.xxx
I (xxx) webserver: HTTP server started
I (xxx) main: Launching gps_logging_task and sync_state_machine_task
```

If you see **SD card mount failed** or **IMU not found**, consult [Troubleshooting](troubleshooting.md).

---

## Step 10 — Confirm the Device Is Working

1. Find the IP address printed in the serial monitor (e.g. `192.168.1.42`).
2. On a device connected to the same network, open a browser to `http://192.168.1.42/`.
3. You should see the dashboard with file listings.
4. Navigate to `http://192.168.1.42/status` — it returns a JSON block showing SD card state, GPS fix status, and sensor health.

If WiFi credentials were left as placeholders, the device starts in **provisioning mode** instead. Connect to the `ESP32-Tracker-XXXX` access point it broadcasts and open `http://192.168.4.1` to enter your credentials. See [Configuration — WiFi Provisioning](configuration.md#wifi-provisioning-after-factory-reset).

---

## What Next?

| Task | Guide |
|------|-------|
| Understand the hardware wiring | [Hardware](hardware.md) |
| Adjust GPS, IMU, and power thresholds | [Configuration](configuration.md) |
| Understand the CSV data schema | [Data Format](data-format.md) |
| Understand the power stages and deep sleep | [Power Management](power-management.md) |
| Use the web interface and API | [Web Interface](web-interface.md) |
| Something is not working | [Troubleshooting](troubleshooting.md) |
