# ESP32 LD19 LiDAR Project

This project implements a portable LiDAR scanner and visualization tool using a **LilyGo T-Display-S3** (ESP32-S3 with AMOLED display) and an **LD19 LiDAR** sensor.

![Project Overview](IMG_3437.jpeg)
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

![Internal View](IMG_3438.jpeg)
*Internal layout and component fitting.*

![Component Placement](IMG_3439.jpg)
*LiDAR and T-Display seated in the bottom enclosure.*

![Wiring Detail](IMG_3440.JPG)
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
