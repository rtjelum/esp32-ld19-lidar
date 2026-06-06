FQBN       = esp32:esp32:esp32s3:FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc
PORT       = /dev/cu.usbmodem*
BUILD_DIR  = build
SKETCH     = esp32-ld19-lidar.ino
ESPTOOL    = $(HOME)/Library/Arduino15/packages/esp32/tools/esptool_py/4.5.1/esptool

# Standalone touch-panel test sketch (touch_test/touch_test.ino). Flash it over
# the main firmware to sanity-check the CST816 capacitive touch in isolation,
# then `make flash` to restore the LiDAR firmware.
TOUCH_DIR       = touch_test
TOUCH_BUILD_DIR = build_touch

# .ldim toolchain (pull recordings over WiFi, view, floorplan, mcap, viz).
# Pin to python3.13 — the PRBonn visualizer / rosbags lack wheels for 3.14.
HOST     = lidar.local
PYTHON3 ?= python3.13
VENV     = tools/.venv
PY       = $(VENV)/bin/python
VIZ_REPO ?= https://github.com/rtjelum/lidar-visualizer_copy.git
# Newest recording by name. Keep the comment on its own line: a trailing inline
# comment would fold its leading spaces into the value and break quoted uses.
LDIM     ?= $(lastword $(sort $(wildcard recordings/*.ldim)))

# SKIP_MERGE no-ops the ESP32 post-link "esptool merge-bin" hook. That step
# builds a 16 MB merged.bin that `arduino-cli upload` (and `verify` /
# `bootloader` below) don't use — they stream the individual bootloader,
# partitions, and sketch .bin files. Skipping it saves ~8 s per build.
SKIP_MERGE = --build-property "recipe.hooks.objcopy.postobjcopy.3.pattern=/usr/bin/true"

.PHONY: all compile compile-full flash verify bootloader clean list \
        touch-compile touch-flash venv pull-recordings list-recordings \
        view floorplan mcap viz viz3d viz3d-static viz3d-merge

all: compile flash

compile:
	arduino-cli compile --fqbn $(FQBN) --build-path $(BUILD_DIR) $(SKIP_MERGE) $(CURDIR)

# Full build that also produces the single merged.bin (use when you want to
# flash everything in one shot via a raw esptool invocation).
compile-full:
	arduino-cli compile --fqbn $(FQBN) --build-path $(BUILD_DIR) $(CURDIR)

flash: compile
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) --verify --build-path $(BUILD_DIR) $(CURDIR)

verify:
	$(ESPTOOL) --port $(PORT) --baud 921600 verify-flash \
	  0x0000 $(BUILD_DIR)/$(SKETCH).bootloader.bin \
	  0x8000 $(BUILD_DIR)/$(SKETCH).partitions.bin \
	  0x10000 $(BUILD_DIR)/$(SKETCH).bin

bootloader:
	$(ESPTOOL) --port $(PORT) --baud 921600 write-flash 0x0000 $(BUILD_DIR)/$(SKETCH).bootloader.bin

# ── Touch test ──────────────────────────────────────────────────────────────
touch-compile:
	arduino-cli compile --fqbn $(FQBN) --build-path $(TOUCH_BUILD_DIR) $(SKIP_MERGE) $(CURDIR)/$(TOUCH_DIR)

touch-flash: touch-compile
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) --verify --build-path $(TOUCH_BUILD_DIR) $(CURDIR)/$(TOUCH_DIR)

clean:
	rm -rf $(BUILD_DIR) $(TOUCH_BUILD_DIR)

list:
	arduino-cli board list

# ── .ldim toolchain ───────────────────────────────────────────────────────────
# lidar-visualizer (rtjelum fork of PRBonn), cloned on demand (installed editable by the venv via
# `-e ./lidar-visualizer` in requirements.txt). Not tracked in this repo.
lidar-visualizer:
	git clone --depth 1 $(VIZ_REPO) lidar-visualizer

# Python venv for view/floorplan/mcap/viz (pull uses only the stdlib).
venv: $(VENV)/bin/python
$(VENV)/bin/python: tools/requirements.txt | lidar-visualizer
	$(PYTHON3) -m venv $(VENV)
	$(PY) -m pip install -q -U pip -r tools/requirements.txt
	touch $(VENV)/bin/python

# Download new recordings from the device into ./recordings/ (HOST=lidar.local).
pull-recordings:
	python3 tools/pull_recordings.py --host $(HOST)

list-recordings:
	python3 tools/pull_recordings.py --host $(HOST) --list

# Quick static matplotlib scatter of a recording (no extra repo needed).
# Override the file with LDIM=recordings/scan_002.ldim.
view: venv
	$(PY) tools/view_ldim.py $(LDIM)

# Build a floorplan PNG from a recording (gyro deskew + ICP + loop closure).
floorplan: venv
	$(PY) tools/ldim_to_floorplan.py $(LDIM)

# Convert a .ldim to an MCAP (LaserScan + PointCloud2 + Imu) for the visualizer.
mcap: venv
	$(PY) tools/ldim_to_mcap.py $(LDIM)

# Launch the interactive PRBonn lidar-visualizer on a recording (auto-converts
# to MCAP first). Override with LDIM=recordings/scan_002.ldim.
viz: venv
	@test -n "$(LDIM)" || { echo "no .ldim under recordings/ — run 'make pull-recordings' first"; exit 1; }
	@out="$(LDIM:.ldim=.mcap)"; \
	  if [ ! -f "$$out" ] || [ "$(LDIM)" -nt "$$out" ]; then $(PY) tools/ldim_to_mcap.py "$(LDIM)"; fi; \
	  $(VENV)/bin/lidar_visualizer "$$out" --topic /points

# Launch the 3D visualizer with full tilt compensation and frame-by-frame sequence.
# Override with LDIM=recordings/scan_007.ldim.
viz3d: venv
	@test -n "$(LDIM)" || { echo "no .ldim under recordings/ — run 'make pull-recordings' first"; exit 1; }
	@out="$(LDIM:.ldim=_3d.mcap)"; \
	  if [ ! -f "$$out" ] || [ "$(LDIM)" -nt "$$out" ]; then $(PY) tools/ldim_to_3d_mcap.py "$(LDIM)"; fi; \
	  $(VENV)/bin/lidar_visualizer "$$out" --topic /points

# Launch a static merged 3D scan (one frame with all points). Assumes the rig
# only rotates (tripod/fixed-pivot tilt+pan); handheld translation will smear it.
viz3d-static: venv
	@test -n "$(LDIM)" || { echo "no .ldim under recordings/ — run 'make pull-recordings' first"; exit 1; }
	@out="$(LDIM:.ldim=_static.mcap)"; \
	  if [ ! -f "$$out" ] || [ "$(LDIM)" -nt "$$out" ]; then $(PY) tools/ldim_to_3d_mcap.py "$(LDIM)" --merge; fi; \
	  $(VENV)/bin/lidar_visualizer "$$out" --topic /points

# Launch a merged 3D scan with 6-DoF point-to-plane drift correction. Use this
# instead of viz3d-static for handheld tilt scans: it registers keyframe submaps
# to the growing map and corrects the translation drift that smears/doubles
# walls. Override with LDIM=recordings/scan_007.ldim.
viz3d-merge: venv
	@test -n "$(LDIM)" || { echo "no .ldim under recordings/ — run 'make pull-recordings' first"; exit 1; }
	@out="$(LDIM:.ldim=_merged6.mcap)"; \
	  if [ ! -f "$$out" ] || [ "$(LDIM)" -nt "$$out" ]; then $(PY) tools/ldim_to_3d_mcap.py "$(LDIM)" --merge6; fi; \
	  $(VENV)/bin/lidar_visualizer "$$out" --topic /points
