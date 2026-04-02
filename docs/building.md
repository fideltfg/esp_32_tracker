# Building and Flashing

## Prerequisites

- ESP-IDF v5.5.2 or compatible
- Python 3.7+

## Build

```bash
idf.py build
```

## Flash

```bash
idf.py -p COM3 flash monitor
```

Replace `COM3` with your device's serial port.

## menuconfig

```bash
idf.py menuconfig
```

| Setting | Required Value |
|---------|---------------|
| `Component config → FAT Filesystem → Long filename support` | Heap |
| `Component config → ESP32-specific → Main XTAL frequency` | 40 MHz |
