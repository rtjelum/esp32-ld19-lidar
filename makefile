FQBN       = esp32:esp32:esp32s3:FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc
PORT       = /dev/cu.usbmodem*
BUILD_DIR  = build
SKETCH     = esp32-ld19-lidar.ino
ESPTOOL    = $(HOME)/Library/Arduino15/packages/esp32/tools/esptool_py/4.5.1/esptool

.PHONY: all compile compile-full flash verify bootloader clean list

all: compile flash

# The sketch is named after its parent folder (esp32-ld19-lidar), so arduino-cli
# can build it in place — no need to stage a scratch sketch directory.
#
# SKIP_MERGE replaces the ESP32 post-link "esptool merge-bin" hook with a no-op.
# That step builds a 16 MB merged.bin that `arduino-cli upload` (and `verify` /
# `bootloader` below) don't use — they stream the individual bootloader,
# partitions, and sketch .bin files. Skipping it saves ~8s per build.
SKIP_MERGE = --build-property "recipe.hooks.objcopy.postobjcopy.3.pattern=/usr/bin/true"

compile:
	@BUILD_DIR=$(BUILD_DIR) ./scripts/compile.sh --fqbn $(FQBN) --build-path $(BUILD_DIR) $(SKIP_MERGE) $(CURDIR)

# Full build that also produces the single merged.bin (use when you want to
# flash everything in one shot via a raw esptool invocation).
compile-full:
	@BUILD_DIR=$(BUILD_DIR) ./scripts/compile.sh --fqbn $(FQBN) --build-path $(BUILD_DIR) $(CURDIR)

flash: compile
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) --verify --build-path $(BUILD_DIR) $(CURDIR)

verify:
	$(ESPTOOL) --port $(PORT) --baud 921600 verify-flash \
	  0x0000 $(BUILD_DIR)/$(SKETCH).bootloader.bin \
	  0x8000 $(BUILD_DIR)/$(SKETCH).partitions.bin \
	  0x10000 $(BUILD_DIR)/$(SKETCH).bin

bootloader:
	$(ESPTOOL) --port $(PORT) --baud 921600 write-flash 0x0000 $(BUILD_DIR)/$(SKETCH).bootloader.bin

clean:
	rm -rf $(BUILD_DIR)

list:
	arduino-cli board list
