FQBN       = esp32:esp32:esp32s3:FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc
PORT       = /dev/cu.usbmodem*
BUILD_DIR  = build
SKETCH     = tdisps3_proj.ino
SKETCH_DIR = $(BUILD_DIR)/sketch/tdisps3_proj
ESPTOOL    = $(HOME)/Library/Arduino15/packages/esp32/tools/esptool_py/4.5.1/esptool

.PHONY: all compile flash verify bootloader clean list sketchdir

all: compile flash

# arduino-cli requires the .ino filename to match the parent directory name,
# so we stage the sketch in a correctly-named subdir under the build tree.
sketchdir:
	@mkdir -p $(SKETCH_DIR)
	@ln -sf $(CURDIR)/$(SKETCH) $(SKETCH_DIR)/tdisps3_proj.ino

compile: sketchdir
	arduino-cli compile --fqbn $(FQBN) --build-path $(BUILD_DIR) $(SKETCH_DIR)

flash: compile
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) --verify --build-path $(BUILD_DIR) $(SKETCH_DIR)

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
