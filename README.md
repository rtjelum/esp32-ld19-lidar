# ESP32 LD19 LiDAR Project

This project implements a portable LiDAR scanner and visualization tool using a **LilyGo T-Display-S3** (ESP32-S3 with AMOLED display) and an **LD19 LiDAR** sensor.

<img src="IMG_3437.jpeg" alt="Project Overview" width="450">

*The assembled LiDAR scanner in its custom enclosure.*

## Features

- **Real-time Visualization:** Displays live LiDAR point cloud data on the 536x240 AMOLED screen.
- **Web Interface:** Includes a built-in web server to monitor data and configure the device over WiFi.
- **Custom Enclosure:** A two-piece 3D-printable snap-fit box designed specifically for these components.
- **Integrated Game:** Includes a built-in Sokoban game (reachable via the display/buttons).
- **Closed-loop Speed Control:** Manages the LiDAR motor speed for consistent data acquisition.

## Hardware

- **MCU:** [LilyGo T-Display-S3](https://www.lilygo.cc/products/t-display-s3) (ESP32-S3)
- **LiDAR:** LD19 LiDAR Sensor
- **Enclosure:** Custom 3D-printed case (see `build_box.py`)

<img src="IMG_3438.jpeg" alt="Internal View" width="350">

*Internal layout and component fitting.*

<img src="IMG_3439.jpg" alt="Component Placement" width="350">

*LiDAR and T-Display seated in the bottom enclosure.*

<img src="IMG_3440.JPG" alt="Wiring Detail" width="350">

*Detail of the wiring connection between the MCU and the sensor.*

## Software Setup

### Firmware
The firmware is written in C++ for the Arduino framework. It uses the `Arduino_GFX` library for the AMOLED display.

**Build Requirements:**
- `arduino-cli`
- ESP32 Board Support (esp32:esp32:esp32s3)

To compile and flash:
```bash
make
```

### WiFi Setup

On first boot — or any boot with no stored credentials — the device starts a SoftAP captive setup:

1. Connect your phone or laptop to the WiFi network **`battle-setup`** (open, no password).
2. Open **http://192.168.4.1/** in a browser. The setup page scans for nearby networks.
3. Tap an SSID to fill the field (or type it manually), enter the password, and press **Connect**.
4. The device stores the credentials in flash and reboots. The AMOLED then shows the station-mode URL (typically `http://battle.local/` via mDNS).

To re-do setup later, open a serial console at 115200 baud and type one of:

- `clear` — wipes stored WiFi credentials and reboots into setup mode
- `reboot` — restart the device
- `help` — list commands

### Sokoban

The device serves a built-in Sokoban game at **http://battle.local/sokoban** (or `/sokoban` on whichever IP is shown on the display).

<img src="IMG_3441.jpg" alt="Sokoban running in a browser" width="250">

*Sokoban running on the embedded web server, rendered isometrically.*

Push the green crates onto the pink goal pads. Each pad must hold one crate to clear the level — there are 50 levels in total, advancing automatically when the last crate is placed. Crates can only be pushed (never pulled) and only when the square behind them is empty, so plan ahead.

Two control modes:

- **D-pad / arrow keys** — tap the on-screen pad or use arrow keys on a desktop.
- **LiDAR motion control** — toggle **Tracker: ON** to stand in front of the sensor and step into one of the four direction cells in the 3×3 floor grid (the centre cell is "idle"). Each cell-entry fires one move; you have to step back to idle before re-entering the same cell.

### Enclosure Design
The 3D-printable box is generated using a Python script that leverages geometric libraries.

**Requirements:**
- Python 3
- `numpy`
- `trimesh`
- `shapely`

To generate the STL files:
```bash
python3 build_box.py
```
This will produce `lidar_tdisplay_box_bottom.stl` and `lidar_tdisplay_box_top.stl`.

## File Structure

- `esp32-ld19-lidar.ino`: Main Arduino firmware.
- `lidar_page.h`, `sokoban_page.h`, `setup_page.h`, `sokoban_levels.h`: Web-page HTML and Sokoban level data, included by the sketch.
- `build_box.py`: Python script to generate the 3D enclosure.
- `makefile`: Build system for the firmware.
- `*.stl`: Generated 3D models for printing.
- `IMG_*.jpeg`: Project photos.
