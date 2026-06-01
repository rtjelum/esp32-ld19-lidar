// Touch-screen test for the LilyGO T-Display-S3 AMOLED 1.91" (RM67162 + CST816).
//
// Standalone sketch — flashes over the main firmware so you can sanity-check the
// capacitive touch panel in isolation. It:
//   1. brings up the AMOLED exactly like the main sketch,
//   2. scans the I2C bus and prints every device it finds (serial + screen),
//   3. reads the CST816 chip-ID register so you can confirm the controller,
//   4. then live-tracks touches: draws a dot where you press and prints the
//      raw + mapped coordinates and gesture to serial at 115200 baud.
//
// Build/flash with the makefile:  make touch-flash
//
// If the I2C scan shows no device, your board is likely the non-touch AMOLED
// variant, or SDA/SCL differ — adjust TOUCH_SDA / TOUCH_SCL below.

#include <Wire.h>
#include <Arduino_GFX_Library.h>

// ── AMOLED (RM67162, 536×240) — identical wiring to the main sketch ──────────
#define LCD_CS   6
#define LCD_SCLK 47
#define LCD_D0   18
#define LCD_D1   7
#define LCD_D2   48
#define LCD_D3   5
#define LCD_RST  17
#define LCD_PWR  15

#define BLACK  0x0000
#define WHITE  0xFFFF
#define RED    0xF800
#define GREEN  0x07E0
#define BLUE   0x001F
#define YELLOW 0xFFE0
#define CYAN   0x07FF

static const int DISP_W = 536;
static const int DISP_H = 240;

Arduino_DataBus *bus     = nullptr;
Arduino_GFX     *gfx_dev = nullptr;
Arduino_Canvas  *gfx     = nullptr; // off-screen PSRAM framebuffer

// ── CST816 capacitive touch (I2C) ───────────────────────────────────────────
#define TOUCH_SDA  3
#define TOUCH_SCL  2
#define TOUCH_INT  21
#define TOUCH_RST  -1            // shared with display reset / not wired on this board
#define TOUCH_ADDR 0x15         // CST816S/T 7-bit address

static bool     touchPresent = false;
static uint8_t  touchChipId  = 0;

struct TouchPt { bool pressed; uint16_t x, y; uint8_t gesture; };

// Read `len` bytes starting at `reg` from the touch controller.
static bool touchRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(TOUCH_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((int)TOUCH_ADDR, (int)len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// One CST816 report: gesture(0x01), finger count(0x02), then X/Y hi/lo.
static TouchPt touchPoll() {
  TouchPt t = {false, 0, 0, 0};
  uint8_t d[6];
  if (!touchRead(0x01, d, 6)) return t;
  uint8_t fingers = d[1];
  if (fingers == 0 || fingers > 2) return t; // no/garbage contact
  t.pressed = true;
  t.gesture = d[0];
  t.x = ((uint16_t)(d[2] & 0x0F) << 8) | d[3];
  t.y = ((uint16_t)(d[4] & 0x0F) << 8) | d[5];
  return t;
}

// The CST816 already reports in the landscape frame (X along the 536 axis,
// Y along the 240 axis), so map raw → screen directly. If an axis comes out
// mirrored, negate just that line, e.g. sx = (DISP_W - 1) - rx.
static void mapToScreen(uint16_t rx, uint16_t ry, int &sx, int &sy) {
  sx = rx;
  sy = ry;
  if (sx < 0) sx = 0; if (sx >= DISP_W) sx = DISP_W - 1;
  if (sy < 0) sy = 0; if (sy >= DISP_H) sy = DISP_H - 1;
}

static void banner(const char *line1, const char *line2, uint16_t color) {
  gfx->fillScreen(BLACK);
  gfx->setTextColor(color);
  gfx->setTextSize(4);
  gfx->setCursor(10, 20);
  gfx->print(line1);
  if (line2) {
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    gfx->setCursor(10, 80);
    gfx->print(line2);
  }
  gfx->flush();
}

// Scan the whole I2C bus and report. Returns true if the touch addr answered.
static bool i2cScan() {
  Serial.println("[i2c] scanning bus...");
  gfx->fillScreen(BLACK);
  gfx->setTextColor(CYAN); gfx->setTextSize(3);
  gfx->setCursor(10, 8); gfx->print("I2C scan");
  gfx->setTextSize(2); gfx->setTextColor(WHITE);
  int found = 0, y = 50;
  bool sawTouch = false;
  for (uint8_t addr = 1; addr < 0x7F; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found++;
      Serial.printf("[i2c]  found device 0x%02X\n", addr);
      if (y < DISP_H - 16) {
        gfx->setCursor(10, y);
        gfx->printf("0x%02X%s", addr, addr == TOUCH_ADDR ? " <- touch" : "");
        y += 22;
      }
      if (addr == TOUCH_ADDR) sawTouch = true;
    }
  }
  if (found == 0) {
    Serial.println("[i2c]  no devices found!");
    gfx->setCursor(10, 50); gfx->setTextColor(RED);
    gfx->print("NO DEVICES");
    gfx->setTextColor(WHITE); gfx->setTextSize(1);
    gfx->setCursor(10, 90);
    gfx->print("Non-touch board? Check SDA=3/SCL=2 wiring.");
  }
  gfx->flush();
  delay(2500);
  return sawTouch;
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== AMOLED touch test ===");

  // Display power + init, identical to the main firmware.
  pinMode(LCD_PWR, OUTPUT); digitalWrite(LCD_PWR, HIGH);
  pinMode(38, OUTPUT); digitalWrite(38, HIGH);
  delay(200);

  bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
  gfx_dev = new Arduino_RM67162(bus, LCD_RST, 1, false);
  gfx_dev->begin();
  gfx_dev->invertDisplay(false);
  gfx_dev->fillScreen(BLACK);
  gfx = new Arduino_Canvas(DISP_W, DISP_H, gfx_dev);
  gfx->begin(GFX_SKIP_OUTPUT_BEGIN);

  banner("TOUCH TEST", "Bringing up I2C...", CYAN);
  delay(800);

  // Bring up the touch controller.
  if (TOUCH_RST >= 0) {
    pinMode(TOUCH_RST, OUTPUT);
    digitalWrite(TOUCH_RST, LOW);  delay(20);
    digitalWrite(TOUCH_RST, HIGH); delay(60);
  }
  pinMode(TOUCH_INT, INPUT_PULLUP);
  Wire.begin(TOUCH_SDA, TOUCH_SCL, 400000);

  touchPresent = i2cScan();

  if (touchPresent) {
    uint8_t id = 0;
    if (touchRead(0xA7, &id, 1)) {        // CST816 chip-ID register
      touchChipId = id;
      Serial.printf("[touch] chip ID = 0x%02X\n", id);
    }
    char l2[40];
    snprintf(l2, sizeof(l2), "CST816 id=0x%02X  press screen", touchChipId);
    banner("READY", l2, GREEN);
  } else {
    banner("NO TOUCH", "Addr 0x15 silent - see I2C scan", RED);
  }
  delay(1500);
  gfx->fillScreen(BLACK); gfx->flush();
}

void loop() {
  static uint32_t lastDraw = 0;
  static bool wasPressed = false;

  TouchPt t = touchPoll();

  if (t.pressed) {
    int sx, sy;
    mapToScreen(t.x, t.y, sx, sy);

    // Draw a crosshair + dot where the finger is.
    gfx->fillScreen(BLACK);
    gfx->drawFastHLine(0, sy, DISP_W, 0x2965);
    gfx->drawFastVLine(sx, 0, DISP_H, 0x2965);
    gfx->fillCircle(sx, sy, 14, YELLOW);
    gfx->drawCircle(sx, sy, 14, WHITE);

    gfx->setTextSize(2); gfx->setTextColor(WHITE);
    gfx->setCursor(10, 10);
    gfx->printf("raw  %4u,%4u", t.x, t.y);
    gfx->setCursor(10, 34);
    gfx->printf("scr  %4d,%4d", sx, sy);
    gfx->setCursor(10, 58);
    gfx->printf("gest 0x%02X", t.gesture);
    gfx->flush();

    if (millis() - lastDraw > 100) {
      lastDraw = millis();
      Serial.printf("[touch] raw=(%u,%u) screen=(%d,%d) gesture=0x%02X\n",
                    t.x, t.y, sx, sy, t.gesture);
    }
    wasPressed = true;
  } else if (wasPressed) {
    // Released: clear once.
    gfx->fillScreen(BLACK);
    gfx->setTextSize(3); gfx->setTextColor(0x4208);
    gfx->setCursor(10, 100); gfx->print("press screen...");
    gfx->flush();
    wasPressed = false;
  }

  delay(15);
}
