#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <FFat.h>
#include <esp_timer.h>
#include "lidar_page.h"
#include "sokoban_page.h"
#include "sokoban_levels.h"
#include "setup_page.h"
#include "localizer.h"

// ── AMOLED (RM67162, 536×240 physical) ──────────────────────────────────────
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
#define GREEN  0x0500
#define BLUE   0x0016

// ── Touch (CST816 capacitive, I2C) ──────────────────────────────────────────
#define TOUCH_SDA  3
#define TOUCH_SCL  2
#define TOUCH_INT  21
#define TOUCH_ADDR 0x15
struct TouchPt { bool pressed; uint16_t x, y; };

// ── IMU on a second I2C bus (Wire1) ─────────────────────────────────────────
#define IMU_SDA  41
#define IMU_SCL  39

Arduino_DataBus *bus = nullptr;
Arduino_GFX     *gfx_dev = nullptr;
Arduino_Canvas  *gfx = nullptr; // off-screen PSRAM framebuffer; flush() pushes to gfx_dev

static const int DISP_W = 536;
static const int DISP_H = 240;

Preferences prefs;
char ssid[64]     = "";
char password[64] = "";

constexpr uint32_t fnv1a(const char* s, uint32_t h = 2166136261u) {
    return *s ? fnv1a(s + 1, (h ^ (uint8_t)*s) * 16777619u) : h;
}
constexpr uint32_t BUILD_HASH = fnv1a(__DATE__ " " __TIME__);

static bool          apMode = false;
static unsigned long skLastSeen = 0;
WebServer server(80);

// ── LiDAR LD19 ────────────────────────────────────────────────────────────────
#define LIDAR_RX_PIN  44
#define LIDAR_PWM_PIN 46
#define LIDAR_PWM_FREQ 1000   // LD19P external speed control: 500 Hz – 1.5 kHz
#define LIDAR_PWM_RES  8
#define LIDAR_BAUD   230400
#define LIDAR_PKT_LEN 47
#define LIDAR_POINTS_PER_PKT 12
// Returns are clamped to the LD19's rated maximum range. The sensor does not
// report valid hits beyond this, so it acts as a safety ceiling rather than an
// artificial limit. Front and rear are treated symmetrically to avoid the
// rendering discontinuity at the 90°/270° boundary.
#define LIDAR_MAX_RANGE_MM 12000
static const int LIDAR_BUF_POINTS = 480; // ~40 packets, ≈ 1 full rotation @10Hz
static volatile uint8_t  lidarPwmDuty   = 128;   // 50% — also LD19 external-mode entry duty
static volatile bool     lidarMotorOn   = true;
static volatile uint32_t lidarPwmFreq   = LIDAR_PWM_FREQ;
static volatile bool     lidarPwmInvert = false;
static volatile uint16_t lidarTargetRpm = 600;   // default 10 Hz closed-loop (0 = manual duty)
static inline void applyLidarPwm(){
  uint32_t d = lidarMotorOn ? lidarPwmDuty : 0;
  if (lidarPwmInvert) d = 255 - d;
  ledcWrite(LIDAR_PWM_PIN, d);
}

struct LidarPt { uint16_t angle_q2; uint16_t dist; uint8_t intensity; };
static LidarPt lidarBuf[LIDAR_BUF_POINTS];
static volatile uint16_t lidarHead = 0;
static volatile uint32_t lidarSeq  = 0;
static volatile uint16_t lidarSpeed = 0; // deg/sec
static portMUX_TYPE lidarMux = portMUX_INITIALIZER_UNLOCKED;

// ── Person detection state (posted back from browser via /people) ────────────
static portMUX_TYPE personMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t personLastSeen = 0;
static volatile uint8_t  personCount = 0;
static const uint8_t PERSON_MAX = 6;
static float personAngle[PERSON_MAX]; // degrees, 0°=left, 90°=front, 180°=right
static float personDist[PERSON_MAX];  // meters
static float personX[PERSON_MAX];     // meters, +right
static float personY[PERSON_MAX];     // meters, +forward
static float bgDist[360];    // mm, per-bin rolling background for shadow detection
static volatile float lidarAreaM2 = 0.0f; // scanned polygon area in m²

// ── Touch UI state ───────────────────────────────────────────────────────────
static volatile bool personTrackingOn = true;  // TRACK button: gate personTask detection
static volatile bool bigAreaView      = false; // AREA button: full-screen m² readout
static volatile bool displayDirty     = false; // set by touchTask to force an immediate repaint
static volatile int  btnPressedIdx    = -1;    // which button is held (for press highlight)

// ── Recording state (REC button) ──────────────────────────────────────────────
// Writes raw LD19 packets to a .ldim file on the onboard 9 MB FAT partition, in
// the same format as ../rasp_pi_zero2_ld19_lidar (tools/ldim_*.py can read them).
// recWant is toggled by the touch task; all file I/O happens in lidarTask, which
// owns recFile, so no cross-task file locking is needed.
static volatile bool     recWant   = false;  // desired state (set by touch)
static volatile bool     recActive = false;  // actual state (set by lidarTask)
static volatile uint32_t recBytes  = 0;      // bytes written so far
static volatile int      recNo     = 0;      // current scan_NNN number
static File              recFile;
static int64_t           recStartUs = 0;
static SemaphoreHandle_t recMutex   = nullptr; // serializes recFile across lidar/imu tasks

// ── BMI160 IMU on Wire1 (addr 0x69) ────────────────────────────────────────────
#define BMI160_ADDR     0x69
#define BMI160_CHIPID   0xD1
static volatile bool  imuOk = false;
static volatile bool  imuView = false;                 // IMU button: live readout view
static volatile float imuAx = 0, imuAy = 0, imuAz = 0; // g, EMA-smoothed
static volatile float imuGx = 0, imuGy = 0, imuGz = 0; // dps, EMA-smoothed

// ── Localization state ──────────────────────────────────────────────────────
static portMUX_TYPE poseMux = portMUX_INITIALIZER_UNLOCKED;
static volatile float locX = 0.0f, locY = 0.0f, locTheta = 0.0f;
static volatile float locErr = 0.0f;
static volatile int   locInliers = 0;
static volatile bool  locValid = false;     // true once first ICP converges
static volatile bool  seedResetRequested = false;  // /lidar/map/reset sets this

static const uint8_t LD19_CRC_TABLE[256] = {
  0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
  0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
  0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
  0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
  0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
  0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
  0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
  0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
  0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
  0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
  0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
  0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
  0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
  0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
  0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
  0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
};

static uint8_t ld19Crc(const uint8_t* p, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; i++) c = LD19_CRC_TABLE[(c ^ p[i]) & 0xff];
  return c;
}

// ── Touch panel (CST816) + on-screen buttons ───────────────────────────────────
static bool touchRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(TOUCH_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)TOUCH_ADDR, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// One CST816 report: gesture, finger count, then X/Y hi/lo. The panel reports
// already in the landscape (536×240) frame, so x/y map straight to the canvas.
static TouchPt touchPoll() {
  TouchPt t = {false, 0, 0};
  uint8_t d[6];
  if (!touchRead(0x01, d, 6)) return t;
  uint8_t fingers = d[1];
  if (fingers == 0 || fingers > 2) return t;
  t.pressed = true;
  t.x = ((uint16_t)(d[2] & 0x0F) << 8) | d[3];
  t.y = ((uint16_t)(d[4] & 0x0F) << 8) | d[5];
  return t;
}

// Bottom button bar: four buttons across the 536-wide screen.
static const int BTN_N = 4;
static const int BTN_Y = 186, BTN_H = 48, BTN_W = 124;
static const int BTN_X[BTN_N] = {8, 140, 272, 404};

static int hitTestButton(int sx, int sy) {
  if (sy < BTN_Y || sy > BTN_Y + BTN_H) return -1;
  for (int i = 0; i < BTN_N; i++)
    if (sx >= BTN_X[i] && sx < BTN_X[i] + BTN_W) return i;
  return -1;
}

static void drawButton(int i, const char* label, bool active, uint16_t accent) {
  bool pressed = (btnPressedIdx == i);
  uint16_t fill = pressed ? 0x39C7 : (active ? ((accent >> 1) & 0x7BEF) : BLACK);
  uint16_t edge = active ? accent : 0x4a69;
  uint16_t txt  = active ? accent : WHITE;
  gfx->fillRoundRect(BTN_X[i], BTN_Y, BTN_W, BTN_H, 8, fill);
  gfx->drawRoundRect(BTN_X[i], BTN_Y, BTN_W, BTN_H, 8, edge);
  gfx->setTextSize(2);
  gfx->setTextColor(txt);
  int tw = (int)strlen(label) * 12;
  gfx->setCursor(BTN_X[i] + (BTN_W - tw) / 2, BTN_Y + (BTN_H - 16) / 2);
  gfx->print(label);
}

static void drawTouchButtons() {
  drawButton(0, personTrackingOn ? "TRACK ON" : "TRACK OFF", personTrackingOn, 0x07E0);
  drawButton(1, "AREA", bigAreaView, 0x07E0);
  drawButton(2, "IMU", imuView, 0x07E0);
  char rl[12];
  if (recActive) snprintf(rl, sizeof(rl), "REC %.1fM", recBytes / 1048576.0);
  else           strcpy(rl, "REC");
  drawButton(3, rl, recActive, 0xF800);  // red accent
}

// ── Recording (.ldim) — only ever called from lidarTask ────────────────────────
static int recNextNumber() {
  int mx = 0;
  File root = FFat.open("/");
  if (root) {
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      const char* nm = f.name();
      const char* slash = strrchr(nm, '/');
      if (slash) nm = slash + 1;
      int n;
      if (sscanf(nm, "scan_%d.ldim", &n) == 1 && n > mx) mx = n;
      f.close();
    }
    root.close();
  }
  return mx + 1;
}

// Append a record (12-byte header + payload). Caller must hold recMutex.
static void recAppend(uint8_t type, const uint8_t* payload, uint16_t len) {
  if (!recActive || !recFile) return;
  uint64_t ns = (uint64_t)(esp_timer_get_time() - recStartUs) * 1000ull;
  uint8_t rh[12];
  memcpy(rh + 0, &ns, 8);
  rh[8] = type;
  rh[9] = 0;
  memcpy(rh + 10, &len, 2);
  recFile.write(rh, 12);
  recFile.write(payload, len);
  recBytes += 12 + len;
}

static void recStart() {
  recNo = recNextNumber();
  char path[32];
  snprintf(path, sizeof(path), "/scan_%03d.ldim", recNo);
  xSemaphoreTake(recMutex, portMAX_DELAY);
  recFile = FFat.open(path, "w");
  if (!recFile) {
    xSemaphoreGive(recMutex);
    Serial.printf("[rec] open %s failed\n", path);
    recWant = false; recActive = false;
    return;
  }
  uint8_t hdr[16] = {0};
  uint32_t magic = 0x4D49444C; // "LDIM"
  uint16_t ver   = 1;
  uint64_t startns = 0;        // no wallclock (no RTC/NTP)
  memcpy(hdr + 0, &magic, 4);
  memcpy(hdr + 4, &ver,   2);
  memcpy(hdr + 8, &startns, 8);
  recFile.write(hdr, 16);
  recBytes   = 16;
  recStartUs = esp_timer_get_time();
  recActive  = true;
  xSemaphoreGive(recMutex);
  Serial.printf("[rec] start %s%s\n", path, imuOk ? " (LD19+IMU)" : " (LD19 only)");
}

static void recStop() {
  xSemaphoreTake(recMutex, portMAX_DELAY);
  recActive = false;
  if (recFile) { recFile.flush(); recFile.close(); }
  xSemaphoreGive(recMutex);
  Serial.printf("[rec] stop scan_%03d.ldim (%u bytes)\n", recNo, (unsigned)recBytes);
}

// Delete every recording on the FAT partition except the one being recorded.
// Returns the number of files removed. Re-scans from the root each pass so we
// never remove a file out from under an open directory iterator.
static int recRemoveAll() {
  char curPath[32] = "";
  if (recActive) snprintf(curPath, sizeof(curPath), "/scan_%03d.ldim", recNo);
  int removed = 0;
  for (;;) {
    String victim;
    File root = FFat.open("/");
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      String p = f.name();
      if (!p.startsWith("/")) p = "/" + p;
      f.close();
      if (p != curPath) { victim = p; break; }
    }
    root.close();
    if (victim.length() == 0) break;
    if (!FFat.remove(victim)) break;   // stop rather than spin if a remove fails
    removed++;
  }
  return removed;
}

// Append one raw LD19 packet (47 bytes), type 0. Called from lidarTask.
static void recWritePacket(const uint8_t* pkt) {
  if (!recActive) return;
  xSemaphoreTake(recMutex, portMAX_DELAY);
  recAppend(0, pkt, LIDAR_PKT_LEN);
  xSemaphoreGive(recMutex);
}

// Append one BMI160 sample (12 bytes: GYR XYZ then ACC XYZ, mount-corrected
// int16 LE), type 1. Called from imuTask. Matches the .ldim IMU payload spec.
static void recWriteImu(const uint8_t* d12) {
  if (!recActive) return;
  xSemaphoreTake(recMutex, portMAX_DELAY);
  recAppend(1, d12, 12);
  xSemaphoreGive(recMutex);
}

// ── BMI160 IMU driver (Wire1) ──────────────────────────────────────────────────
static bool imuWrite(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(BMI160_ADDR);
  Wire1.write(reg); Wire1.write(val);
  return Wire1.endTransmission() == 0;
}

static bool imuReadReg(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire1.beginTransmission(BMI160_ADDR);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom((int)BMI160_ADDR, (int)len) != (int)len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire1.read();
  return true;
}

static bool imuInit() {
  uint8_t id = 0;
  if (!imuReadReg(0x00, &id, 1) || id != BMI160_CHIPID) {
    Serial.printf("[imu] chip id 0x%02X (expected 0x%02X)\n", id, BMI160_CHIPID);
    return false;
  }
  imuWrite(0x7E, 0x11); delay(5);    // CMD: accel normal power mode
  imuWrite(0x7E, 0x15); delay(80);   // CMD: gyro normal power mode
  imuWrite(0x41, 0x03);              // ACC_RANGE ±2g  -> 16384 LSB/g
  imuWrite(0x43, 0x00);              // GYR_RANGE ±2000 dps -> 16.4 LSB/dps
  imuWrite(0x40, 0x28);              // ACC_CONF: ODR 100 Hz, normal filter
  imuWrite(0x42, 0x28);              // GYR_CONF: ODR 100 Hz, normal filter
  Serial.println("[imu] BMI160 ready @ 0x69 on Wire1 (accel + gyro)");
  return true;
}

// ── Display ───────────────────────────────────────────────────────────────────

void updateDisplay() {
  if (!gfx) return;
  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE);
  
  if (apMode) {
    gfx->setTextSize(4);
    const char* t = "WIFI SETUP";
    gfx->setCursor((DISP_W - (int)strlen(t) * 24) / 2, 40);
    gfx->print(t);
    gfx->setTextSize(2);
    gfx->setCursor(60, 120); gfx->print("Connect: lidar-setup");
    gfx->setCursor(60, 150); gfx->print("Open:    http://192.168.4.1/");
    gfx->flush();
    return;
  }

  // AREA button: dedicated full-screen m² readout, LIDAR/persons UI hidden.
  if (bigAreaView) {
    gfx->setTextSize(3);
    gfx->setTextColor(0xFFE0); // yellow
    const char* hdr = "SCAN AREA";
    gfx->setCursor((DISP_W - (int)strlen(hdr) * 18) / 2, 12);
    gfx->print(hdr);

    char ar[16];
    snprintf(ar, sizeof(ar), "%.2f", (double)lidarAreaM2);
    int sz = 8, cw = 48;
    if ((int)strlen(ar) * cw > DISP_W - 16) { sz = 6; cw = 36; }
    gfx->setTextSize(sz);
    gfx->setTextColor(WHITE);
    gfx->setCursor((DISP_W - (int)strlen(ar) * cw) / 2, 66);
    gfx->print(ar);

    gfx->setTextSize(3);
    gfx->setTextColor(0xFFE0);
    const char* unit = "m^2";
    gfx->setCursor((DISP_W - (int)strlen(unit) * 18) / 2, 150);
    gfx->print(unit);

    drawTouchButtons();
    gfx->flush();
    return;
  }

  // IMU button: live BMI160 readout for aligning the (vertical) mount.
  if (imuView) {
    float ax = imuAx, ay = imuAy, az = imuAz;
    float gx = imuGx, gy = imuGy, gz = imuGz;
    char b[48];

    gfx->setTextSize(2);
    gfx->setTextColor(imuOk ? 0x07FF : 0xF800);
    gfx->setCursor(8, 6);
    gfx->print(imuOk ? "IMU  BMI160 @0x69" : "IMU  NOT FOUND");

    gfx->setTextSize(3);
    gfx->setTextColor(WHITE);
    snprintf(b, sizeof(b), "A %+.2f %+.2f %+.2f", (double)ax, (double)ay, (double)az);
    gfx->setCursor(8, 38); gfx->print(b);
    gfx->setTextSize(2); gfx->setTextColor(0x8410);
    gfx->setCursor(8, 66); gfx->print("accel g  (x  y  z)");

    gfx->setTextSize(3);
    gfx->setTextColor(WHITE);
    snprintf(b, sizeof(b), "G %+5.0f %+5.0f %+5.0f", (double)gx, (double)gy, (double)gz);
    gfx->setCursor(8, 92); gfx->print(b);
    gfx->setTextSize(2); gfx->setTextColor(0x8410);
    gfx->setCursor(8, 120); gfx->print("gyro dps");

    // Tilt from gravity — align so the up axis reads close to +/-1.00 g.
    float roll  = atan2f(ay, az) * 57.29578f;
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
    gfx->setTextSize(2); gfx->setTextColor(0xFFE0);
    snprintf(b, sizeof(b), "roll %+6.1f  pitch %+6.1f", (double)roll, (double)pitch);
    gfx->setCursor(8, 150); gfx->print(b);

    drawTouchButtons();
    gfx->flush();
    return;
  }

  uint32_t pLast; uint8_t pCnt; float pAng[PERSON_MAX], pDst[PERSON_MAX];
  portENTER_CRITICAL(&personMux);
  pLast = personLastSeen; pCnt = personCount;
  for (uint8_t i = 0; i < pCnt && i < PERSON_MAX; i++) { pAng[i] = personAngle[i]; pDst[i] = personDist[i]; }
  portEXIT_CRITICAL(&personMux);

  if (pCnt > 0 && (millis() - pLast) < 1500) {
    // ── LEFT HALF: person tracking ──
    const int leftW = 280;
    const int leftCx = leftW / 2;

    gfx->setTextSize(4);
    gfx->setTextColor(0xFD20); // amber
    const char* lbl = "Humans";
    gfx->setCursor(leftCx - (int)strlen(lbl) * 12, 10);
    gfx->print(lbl);

    gfx->setTextSize(3);
    gfx->setTextColor(0xFD20);
    uint8_t show = pCnt > 3 ? 3 : pCnt;
    for (uint8_t i = 0; i < show; i++) {
      char buf[24];
      snprintf(buf, sizeof(buf), "%+4d  %4.2fm", (int)roundf(pAng[i]), (double)pDst[i]);
      gfx->setCursor(leftCx - (int)strlen(buf) * 9, 70 + i * 45);
      gfx->print(buf);
    }

    // ── RIGHT HALF: big area ──
    const int rightX = leftW;
    const int rightCx = rightX + (DISP_W - rightX) / 2;

    gfx->setTextSize(4);
    gfx->setTextColor(0xFFE0); // yellow
    const char* hdr = "Scan size:";
    gfx->setCursor(rightX + 6, 10);
    gfx->print(hdr);

    gfx->setTextSize(8);
    char ar2[16];
    snprintf(ar2, sizeof(ar2), "%.2f", (double)lidarAreaM2);
    int textW = (int)strlen(ar2) * 48;
    if (textW > DISP_W - rightX - 8) {
      gfx->setTextSize(6); textW = (int)strlen(ar2) * 36;
    }
    int curX = rightCx - textW / 2;
    if (curX < rightX + 4) curX = rightX + 4;
    gfx->setCursor(curX, 80);
    gfx->print(ar2);

    gfx->setTextSize(3);
    const char* unit = "m^2";
    gfx->setCursor(rightCx - (int)strlen(unit) * 9, 148);
    gfx->print(unit);

    drawTouchButtons();
    gfx->flush();
    return;
  }

  gfx->setTextSize(5);
  gfx->setTextColor(0x07FF); // Cyan
  const char* title = "LIDAR";
  gfx->setCursor((DISP_W - (int)strlen(title) * 30) / 2, 20);
  gfx->print(title);

  uint16_t speed;
  portENTER_CRITICAL(&lidarMux);
  speed = lidarSpeed;
  portEXIT_CRITICAL(&lidarMux);
  bool spinning = speed > 60; // > 10 RPM

  if (spinning) {
    gfx->setTextSize(6);
    gfx->setTextColor(0xFFE0); // yellow
    char ar[24];
    snprintf(ar, sizeof(ar), "%.2f m^2", (double)lidarAreaM2);
    gfx->setCursor((DISP_W - (int)strlen(ar) * 36) / 2, 95);
    gfx->print(ar);
  } else {
    gfx->setTextSize(3);
    gfx->setTextColor(WHITE);
    const char* st = "WAITING LD19...";
    gfx->setCursor((DISP_W - (int)strlen(st) * 18) / 2, 110);
    gfx->print(st);
  }

  gfx->setTextSize(2);
  gfx->setTextColor(WHITE);
  String url = "http://" + WiFi.localIP().toString() + "/";
  if (WiFi.localIP().toString() == "0.0.0.0") url = "Connecting WiFi...";
  gfx->setCursor((DISP_W - (int)url.length() * 12) / 2, 162);
  gfx->print(url);

  drawTouchButtons();
  gfx->flush();
}

// ── HTTP Handlers ─────────────────────────────────────────────────────────────

static String jsonEsc(const String& s) {
  String out;
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"') out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else if (c >= 0x20) out += c;
  }
  return out;
}

void handlePing() {
  skLastSeen = millis();
  server.send(200, "text/plain", "ok");
}

// SLAM-lite. Wakes at ~5 Hz, snapshots the ring buffer, converts polar→
// Cartesian (sensor frame), and runs a three-phase lifecycle:
//   Phase A (first SEED_FRAMES valid frames, robot must be stationary):
//     accumulate scan points into the map at pose (0,0,0). This defines the
//     map's coordinate frame.
//   Phase B (after seeding): run ICP each frame to track pose against the
//     current map.
//   Phase C (only when pose is trusted — low residual + lots of inliers):
//     fold new scan points into the map so unexplored geometry gets added
//     as the robot moves.
void localizerTask(void *pv) {
  static const int   SEED_FRAMES = 20;
  static const float TRUST_ERR_M = 0.06f;       // mean inlier residual ≤ 6 cm
  static const float TRUST_INLIER_RATIO = 0.15f; // ≥15% of scan must match map
  static const int   TRUST_INLIER_MIN   = 20;    // absolute floor
  static const float EXPAND_MIN_NEW_M = 0.08f;   // add points >8 cm from any neighbor

  static LidarPt snap[LIDAR_BUF_POINTS];
  static float sx[LOC_MAX_SRC], sy[LOC_MAX_SRC];
  Pose2D pose = {0.0f, 0.0f, 0.0f};
  int seedFramesDone = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(200));   // ~5 Hz

    // Honor a re-seed request from /lidar/map/reset.
    if (seedResetRequested) {
      loc_map_clear();
      pose = {0.0f, 0.0f, 0.0f};
      seedFramesDone = 0;
      seedResetRequested = false;
      portENTER_CRITICAL(&poseMux);
      locX = 0; locY = 0; locTheta = 0;
      locErr = 0; locInliers = 0; locValid = false;
      portEXIT_CRITICAL(&poseMux);
    }

    portENTER_CRITICAL(&lidarMux);
    memcpy(snap, lidarBuf, sizeof(snap));
    portEXIT_CRITICAL(&lidarMux);

    // Convert valid points to sensor-frame Cartesian, evenly subsampled to
    // ≤ LOC_MAX_SRC. Throw out range-clamped hits and obvious noise (<10 cm).
    int n = 0;
    int stride = LIDAR_BUF_POINTS / LOC_MAX_SRC; if (stride < 1) stride = 1;
    for (int i = 0; i < LIDAR_BUF_POINTS && n < LOC_MAX_SRC; i += stride) {
      uint16_t d_mm = snap[i].dist;
      if (d_mm < 100 || d_mm >= LIDAR_MAX_RANGE_MM) continue;
      float ang = (snap[i].angle_q2 / 100.0f) * (PI / 180.0f);
      float d_m = d_mm / 1000.0f;
      sx[n] = d_m * cosf(ang);
      sy[n] = d_m * sinf(ang);
      n++;
    }
    if (n < 30) continue;

    if (seedFramesDone < SEED_FRAMES) {
      // Phase A: build the initial map at the assumed origin pose.
      loc_map_seed_add(sx, sy, n);
      seedFramesDone++;
      portENTER_CRITICAL(&poseMux);
      locX = 0; locY = 0; locTheta = 0;
      locErr = 0; locInliers = 0; locValid = false;
      portEXIT_CRITICAL(&poseMux);
      continue;
    }

    // Phase B: track pose against the existing map.
    LocalizeResult r = loc_run(sx, sy, n, pose);
    pose = r.pose;

    // Phase C: grow the map with newly-seen geometry. Ratio-based gate so
    // moving into a partially-unknown area still keeps expansion alive —
    // as long as a fraction of the scan still anchors the pose to known
    // geometry, we trust the result enough to add the unmatched returns.
    int minInliers = (int)(n * TRUST_INLIER_RATIO);
    if (minInliers < TRUST_INLIER_MIN) minInliers = TRUST_INLIER_MIN;
    bool trusted = (r.err < TRUST_ERR_M) && (r.inliers >= minInliers);

    if (trusted) {
      loc_map_expand(sx, sy, n, pose, EXPAND_MIN_NEW_M);
    }

    portENTER_CRITICAL(&poseMux);
    locX = pose.x; locY = pose.y; locTheta = pose.theta;
    locErr = r.err; locInliers = r.inliers; locValid = trusted;
    portEXIT_CRITICAL(&poseMux);
  }
}

// Shadow-based person detection runs entirely in firmware so it works without the web GUI.
// Sensor's physical front is at sensor angle 270° (see SENSOR_ROT_DEG mirror in the page JS);
// we therefore look at sensor bins 180..359 — the front 180°.
void personTask(void *pv) {
  static LidarPt snap[LIDAR_BUF_POINTS];
  static float   cur[360];
  uint32_t lastPrint = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));

    portENTER_CRITICAL(&lidarMux);
    memcpy(snap, lidarBuf, sizeof(snap));
    portEXIT_CRITICAL(&lidarMux);

    for (int b = 0; b < 360; b++) cur[b] = 0;
    for (int i = 0; i < LIDAR_BUF_POINTS; i++) {
      uint16_t dist = snap[i].dist;
      if (dist == 0) continue;
      int b = (snap[i].angle_q2 / 100) % 360;
      if (b < 0) b += 360;
      if (cur[b] == 0 || dist < cur[b]) cur[b] = dist;
    }

    for (int i = 0; i < 180; i++) {
      int b = (180 + i) % 360;
      float dd = cur[b];
      if (dd > 0) {
        if (dd > bgDist[b]) bgDist[b] = dd;
        else { float dec = bgDist[b] * 0.999f; bgDist[b] = (dd > dec) ? dd : dec; }
      }
    }

    // Scanned area: front semicircle only (sensor bins 180..359 — front 180°).
    // Polar shoelace from sensor origin: A = 0.5 Σ r_i * r_{i+1} * sin(Δθ).
    // Rear bins are excluded so the area reflects the forward-facing scan only.
    float areaSum_mm2 = 0;
    int lastB = -1;
    float lastR = 0;
    for (int b = 180; b < 360; b++) {
      float r = cur[b];
      if (r <= 0) continue;
      if (lastB < 0) { lastB = b; lastR = r; continue; }
      float dth = (b - lastB) * (float)PI / 180.0f;
      areaSum_mm2 += 0.5f * lastR * r * sinf(dth);
      lastB = b; lastR = r;
    }
    if (areaSum_mm2 < 0) areaSum_mm2 = -areaSum_mm2;
    lidarAreaM2 = areaSum_mm2 / 1.0e6f;

    // TRACK button off: keep measuring area, but stop person detection.
    if (!personTrackingOn) {
      portENTER_CRITICAL(&personMux);
      personCount = 0;
      portEXIT_CRITICAL(&personMux);
      continue;
    }

    // Raw-candidate filters. Tightened to suppress false positives:
    //   - bgDist - cur > 400 mm: shadow must be clearly in front of background, not edge noise.
    //   - nds >= 4 valid bin samples: a real torso projects ≥4 LiDAR points; isolated spikes don't.
    //   - density >= 0.6: most bins inside the shadow span must carry a valid return,
    //     so half-empty smears (reflective floors, partial occlusions) get rejected.
    //   - pw in [300, 1200] mm: human shoulder/torso range.
    const float PERSON_MIN_WIDTH_MM = 300.0f;
    const float PERSON_MAX_WIDTH_MM = 1200.0f;
    const float BG_DELTA_MM         = 400.0f;
    const int   MIN_SAMPLES         = 4;
    const float MIN_DENSITY         = 0.6f;
    const int   MAX_CANDIDATES      = 12;
    const int   GAP_TOL = 2;
    int start = -1, gap = 0;
    uint8_t ncand = 0;
    float cpa[MAX_CANDIDATES], cpd[MAX_CANDIDATES], cpx[MAX_CANDIDATES], cpy[MAX_CANDIDATES], cpw[MAX_CANDIDATES];
    for (int i = 0; i <= 180; i++) {
      int b = i < 180 ? (180 + i) % 360 : -1;
      bool on = b >= 0 && cur[b] > 0 && bgDist[b] > 400.0f && bgDist[b] - cur[b] > BG_DELTA_MM;
      // A bin with a valid return at ~background distance is a ray pass-through:
      // the LiDAR's ray went past — there is no blocking object at this angle.
      // We must NOT count those bins as part of a person's shadow. The previous
      // logic tolerated them under GAP_TOL, which let a gap between two adjacent
      // people merge into one fake "wide" shadow. We now end the span the moment
      // we hit such a bin. Only cur==0 (no return) bins remain tolerated by GAP_TOL.
      bool passThrough = b >= 0 && cur[b] > 0 && bgDist[b] > 400.0f && bgDist[b] - cur[b] <= BG_DELTA_MM;
      if (on) { if (start < 0) start = i; gap = 0; }
      else if (start >= 0) {
        gap++;
        if (gap > GAP_TOL || i == 180 || passThrough) {
          int lastShadow = i - gap;
          int len = lastShadow - start + 1;
          if (len >= 2 && len <= 60 && ncand < MAX_CANDIDATES) {
            float ds[60]; int nds = 0;
            for (int k = start; k <= lastShadow && nds < 60; k++) {
              int bk = (180 + k) % 360;
              if (cur[bk] > 0) ds[nds++] = cur[bk];
            }
            float density = (float)nds / (float)len;
            if (nds >= MIN_SAMPLES && density >= MIN_DENSITY) {
              for (int a = 1; a < nds; a++) { // insertion sort, nds ≤ 60
                float v = ds[a]; int q = a - 1;
                while (q >= 0 && ds[q] > v) { ds[q+1] = ds[q]; q--; }
                ds[q+1] = v;
              }
              float md = ds[nds/2];
              float pw = 2.0f * md * sinf((float)len * PI / 360.0f);
              if (pw >= PERSON_MIN_WIDTH_MM && pw <= PERSON_MAX_WIDTH_MM) {
                int midBin = (180 + (start + lastShadow) / 2) % 360;
                float rotDeg = (float)midBin - 270.0f; // -90..+89 (front semicircle)
                float rad    = rotDeg * PI / 180.0f;
                float xm     = -sinf(rad) * md / 1000.0f;
                float ym     =  cosf(rad) * md / 1000.0f;
                cpx[ncand] = xm; cpy[ncand] = ym;
                cpa[ncand] = atan2f(xm, ym) * 180.0f / PI + 90.0f;
                cpd[ncand] = md / 1000.0f;
                cpw[ncand] = pw;
                ncand++;
              }
            }
          }
          start = -1; gap = 0;
        }
      }
    }

    // Sort raw candidates by shadow width, biggest first, and take top PERSON_MAX.
    // No temporal persistence: a clear shadow appears the same frame the person is seen.
    for (int a = 1; a < ncand; a++) {
      float wv = cpw[a], av = cpa[a], dv = cpd[a], xv = cpx[a], yv = cpy[a];
      int q = a - 1;
      while (q >= 0 && cpw[q] < wv) {
        cpw[q+1] = cpw[q]; cpa[q+1] = cpa[q]; cpd[q+1] = cpd[q]; cpx[q+1] = cpx[q]; cpy[q+1] = cpy[q];
        q--;
      }
      cpw[q+1] = wv; cpa[q+1] = av; cpd[q+1] = dv; cpx[q+1] = xv; cpy[q+1] = yv;
    }
    uint8_t cnt = ncand > PERSON_MAX ? PERSON_MAX : ncand;
    float pa[PERSON_MAX], pd[PERSON_MAX], px[PERSON_MAX], py[PERSON_MAX];
    for (uint8_t i = 0; i < cnt; i++) {
      pa[i] = cpa[i]; pd[i] = cpd[i]; px[i] = cpx[i]; py[i] = cpy[i];
    }

    portENTER_CRITICAL(&personMux);
    personCount = cnt;
    for (uint8_t i = 0; i < cnt; i++) {
      personAngle[i] = pa[i]; personDist[i] = pd[i];
      personX[i] = px[i]; personY[i] = py[i];
    }
    personLastSeen = millis();
    portEXIT_CRITICAL(&personMux);

    uint32_t now = millis();
    if (now - lastPrint >= 500) {
      lastPrint = now;
      Serial.printf("Movement: N=%u", (unsigned)cnt);
      for (uint8_t i = 0; i < cnt; i++) {
        Serial.printf(" %u:(%.2f,%.2f,%d,%.2f)", (unsigned)(i+1), px[i], py[i], (int)roundf(pa[i]), pd[i]);
      }
      Serial.println();
    }
  }
}

void handleSetup() {
  server.send(200, "text/html", SETUP_PAGE_HTML);
}

void handleWifiScan() {
  int n = WiFi.scanNetworks();
  String j = "[";
  for (int i = 0; i < n; i++) {
    if (i) j += ",";
    j += "{\"ssid\":\"" + jsonEsc(WiFi.SSID(i)) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
  }
  j += "]";
  WiFi.scanDelete();
  server.send(200, "application/json", j);
}

void handleWifiConnect() {
  String newSsid = server.arg("ssid");
  String newPass = server.arg("pass");
  prefs.begin("wifi", false);
  prefs.putString("ssid", newSsid);
  prefs.putString("pass", newPass);
  prefs.end();
  server.send(200, "text/plain", "ok");
  delay(500);
  ESP.restart();
}

void handleLidarScan() {
  // Snapshot the ring buffer, oldest -> newest
  static LidarPt snap[LIDAR_BUF_POINTS];
  uint16_t head;
  uint16_t speed;
  portENTER_CRITICAL(&lidarMux);
  head  = lidarHead;
  speed = lidarSpeed;
  memcpy(snap, lidarBuf, sizeof(snap));
  portEXIT_CRITICAL(&lidarMux);

  // Build compact JSON: {"rpm":N,"pts":[[ang_deg_x100,dist_mm,intensity],...]}
  String j;
  j.reserve(LIDAR_BUF_POINTS * 18 + 32);
  j += "{\"rpm\":";
  j += String(speed / 6); // deg/sec -> rpm
  j += ",\"area\":";
  j += String(lidarAreaM2, 2);
  j += ",\"pts\":[";
  bool first = true;
  for (int i = 0; i < LIDAR_BUF_POINTS; i++) {
    int idx = (head + i) % LIDAR_BUF_POINTS;
    LidarPt& p = snap[idx];
    if (p.dist == 0) continue;
    if (!first) j += ',';
    first = false;
    j += '[';
    j += String(p.angle_q2);
    j += ',';
    j += String(p.dist);
    j += ',';
    j += String(p.intensity);
    j += ']';
  }
  j += "]";

  // attach current detections so the browser can render smileys without re-doing detection
  uint8_t pCnt; float pA[PERSON_MAX], pD[PERSON_MAX], pX[PERSON_MAX], pY[PERSON_MAX];
  portENTER_CRITICAL(&personMux);
  pCnt = personCount;
  for (uint8_t i = 0; i < pCnt; i++) { pA[i]=personAngle[i]; pD[i]=personDist[i]; pX[i]=personX[i]; pY[i]=personY[i]; }
  portEXIT_CRITICAL(&personMux);
  j += ",\"persons\":[";
  for (uint8_t i = 0; i < pCnt; i++) {
    if (i) j += ',';
    j += "{\"a\":"; j += String(pA[i], 1);
    j += ",\"d\":"; j += String(pD[i], 3);
    j += ",\"x\":"; j += String(pX[i], 3);
    j += ",\"y\":"; j += String(pY[i], 3);
    j += "}";
  }
  j += "]";

  // Localized pose (if ICP has converged at least once)
  float px, py, pth, pe; int pi; bool pv;
  portENTER_CRITICAL(&poseMux);
  px = locX; py = locY; pth = locTheta; pe = locErr; pi = locInliers; pv = locValid;
  portEXIT_CRITICAL(&poseMux);
  j += ",\"pose\":{\"valid\":";
  j += (pv ? "true" : "false");
  j += ",\"x\":";   j += String(px, 3);
  j += ",\"y\":";   j += String(py, 3);
  j += ",\"th\":";  j += String(pth, 4);
  j += ",\"err\":"; j += String(pe, 3);
  j += ",\"in\":";  j += String(pi);
  j += "}";
  j += ",\"map_n\":";
  j += String(loc_map_count());
  j += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

// Request a re-seed: clears the map and re-enters phase A on the next
// localizer tick. Replies immediately; actual reset is asynchronous.
void handleLidarMapReset() {
  seedResetRequested = true;
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", "{\"ok\":true}");
}

// Returns the live in-RAM map as JSON: {"n":N,"pts":[[x,y],...]}.
// The page refetches when map_n in /lidar/scan changes (map grows).
void handleLidarMap() {
  int n = loc_map_count();
  String j;
  j.reserve(n * 16 + 32);
  j += "{\"n\":";
  j += String(n);
  j += ",\"pts\":[";
  for (int i = 0; i < n; i++) {
    if (i) j += ',';
    j += '[';
    j += String(locMap[i][0], 3);
    j += ',';
    j += String(locMap[i][1], 3);
    j += ']';
  }
  j += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

void handleLidarSpeed() {
  if (server.hasArg("duty")) {
    int d = server.arg("duty").toInt();
    if (d < 0) d = 0;
    if (d > 255) d = 255;
    lidarPwmDuty = (uint8_t)d;
  }
  if (server.hasArg("on")) {
    lidarMotorOn = (server.arg("on") == "1");
  }
  if (server.hasArg("invert")) {
    lidarPwmInvert = (server.arg("invert") == "1");
  }
  if (server.hasArg("freq")) {
    uint32_t f = (uint32_t)server.arg("freq").toInt();
    if (f >= 100 && f <= 200000) {
      lidarPwmFreq = f;
      ledcDetach(LIDAR_PWM_PIN);
      ledcAttach(LIDAR_PWM_PIN, lidarPwmFreq, LIDAR_PWM_RES);
    }
  }
  if (server.hasArg("target")) {
    int t = server.arg("target").toInt();
    if (t < 0) t = 0;
    if (t > 1500) t = 1500;
    lidarTargetRpm = (uint16_t)t;
  }
  applyLidarPwm();
  String j = "{\"duty\":";
  j += String(lidarPwmDuty);
  j += ",\"on\":";
  j += (lidarMotorOn ? "true" : "false");
  j += ",\"freq\":";
  j += String(lidarPwmFreq);
  j += ",\"inv\":";
  j += (lidarPwmInvert ? "true" : "false");
  j += ",\"target\":";
  j += String(lidarTargetRpm);
  j += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", LIDAR_PAGE_HTML);
}

void handleSokoban() {
  server.send(200, "text/html", SOKOBAN_PAGE_HTML);
}

// ── Recording download API ──────────────────────────────────────────────────
// GET /rec            -> JSON {"recording":bool,"free":N,"files":[{name,size}]}
// DELETE /rec         -> wipe all recordings (skips the active one)
// GET /rec/<file>     -> download the .ldim (octet-stream)
// DELETE /rec/<file>  -> delete it
// pull_recordings.py uses these to fetch .ldim files over WiFi.
void handleRecList() {
  String j = "{\"recording\":";
  j += (recActive ? "true" : "false");
  j += ",\"free\":";
  j += String((uint32_t)FFat.freeBytes());
  j += ",\"files\":[";
  File root = FFat.open("/");
  bool first = true;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    if (!first) j += ',';
    first = false;
    j += "{\"name\":\"";
    j += f.name();
    j += "\",\"size\":";
    j += String((uint32_t)f.size());
    j += '}';
    f.close();
  }
  root.close();
  j += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

// DELETE /rec -> remove every recording except the one being recorded.
void handleRecClear() {
  int n = recRemoveAll();
  String j = "{\"deleted\":";
  j += String(n);
  j += '}';
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

// Serves or deletes /rec/<file>. Registered via onNotFound so the trailing
// filename can be a path parameter.
void handleRecFile() {
  String uri = server.uri();
  if (!uri.startsWith("/rec/")) {
    server.send(404, "text/plain", "not found");
    return;
  }
  String name = uri.substring(5);
  if (name.indexOf('/') >= 0 || name.length() == 0) {   // no nested paths
    server.send(400, "text/plain", "bad name");
    return;
  }
  String path = "/" + name;

  if (server.method() == HTTP_DELETE) {
    bool ok = FFat.remove(path);
    server.send(ok ? 200 : 404, "text/plain", ok ? "deleted" : "not found");
    return;
  }
  char curPath[32];
  snprintf(curPath, sizeof(curPath), "/scan_%03d.ldim", recNo);
  if (recActive && path == curPath) {
    server.send(409, "text/plain", "still recording this file");
    return;
  }
  File f = FFat.open(path, "r");
  if (!f) { server.send(404, "text/plain", "not found"); return; }
  server.sendHeader("Content-Disposition", "attachment; filename=" + name);
  server.streamFile(f, "application/octet-stream");
  f.close();
}

// ── System Functions ──────────────────────────────────────────────────────────

void startAPMode() {
  apMode = true;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("lidar-setup");
  server.on("/",          handleSetup);
  server.on("/wifi/scan",    handleWifiScan);
  server.on("/wifi/connect", handleWifiConnect);
  server.begin();
  updateDisplay();
}

void setupWiFi() {
  prefs.begin("wifi", true);
  String s = prefs.getString("ssid", "");
  String p = prefs.getString("pass", "");
  prefs.end();
  
  if (s == "") { startAPMode(); return; }
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(s.c_str(), p.c_str());
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500); attempts++;
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) { startAPMode(); }
  else {
    Serial.println("\n[wifi] connected");
    server.on("/", handleRoot);
    server.on("/sokoban", handleSokoban);
    server.on("/lidar/scan", handleLidarScan);
    server.on("/lidar/map",  handleLidarMap);
    server.on("/lidar/map/reset", HTTP_POST, handleLidarMapReset);
    server.on("/lidar/speed", handleLidarSpeed);
    server.on("/ping", handlePing);
    server.on("/rec", HTTP_GET, handleRecList);
    server.on("/rec", HTTP_DELETE, handleRecClear);
    server.onNotFound(handleRecFile);   // serves /rec/<file> downloads + deletes
    server.begin();
    MDNS.begin("lidar");
    updateDisplay();   // one-shot paint with the active IP, then leave the screen alone
  }
}

void lidarControlTask(void* pv) {
  // P-controller: when target RPM is set, nudge duty toward target using
  // the speed reported by the LD19 in its data packets (deg/sec ÷ 6 = RPM)
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(200));
    if (!lidarMotorOn || lidarTargetRpm == 0) continue;
    uint16_t s = lidarSpeed;
    if (s == 0) continue;
    int meas = (int)s / 6;
    int err  = (int)lidarTargetRpm - meas;
    int adj  = err / 2;
    if (adj > 5)  adj = 5;
    if (adj < -5) adj = -5;
    int d = (int)lidarPwmDuty + adj;
    if (d < 38)  d = 38;    // LD19P duty control range: 15–84%
    if (d > 214) d = 214;
    lidarPwmDuty = (uint8_t)d;
    applyLidarPwm();
  }
}

void lidarTask(void* pv) {
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, -1);
  Serial1.setRxBufferSize(1024);
  uint8_t buf[LIDAR_PKT_LEN];
  int len = 0;
  for (;;) {
    // Honor record start/stop requests from the REC button (file I/O lives here).
    if (recWant && !recActive)      recStart();
    else if (!recWant && recActive) recStop();

    while (Serial1.available()) {
      uint8_t b = Serial1.read();
      if (len == 0) {
        if (b == 0x54) buf[len++] = b;
      } else if (len == 1) {
        if (b == 0x2C) { buf[len++] = b; }
        else { len = 0; if (b == 0x54) buf[len++] = b; }
      } else {
        buf[len++] = b;
        if (len == LIDAR_PKT_LEN) {
          if (ld19Crc(buf, LIDAR_PKT_LEN - 1) == buf[LIDAR_PKT_LEN - 1]) {
            uint16_t speed  = buf[2] | (buf[3] << 8);
            uint16_t startA = buf[4] | (buf[5] << 8);
            uint16_t endA   = buf[42] | (buf[43] << 8);
            int32_t span = (int32_t)endA - (int32_t)startA;
            if (span < 0) span += 36000;
            float step = span / (float)(LIDAR_POINTS_PER_PKT - 1);
            portENTER_CRITICAL(&lidarMux);
            lidarSpeed = speed;
            for (int i = 0; i < LIDAR_POINTS_PER_PKT; i++) {
              int o = 6 + i * 3;
              uint16_t dist = buf[o] | (buf[o+1] << 8);
              uint8_t  inten = buf[o+2];
              uint32_t a = (uint32_t)startA + (uint32_t)(step * i);
              uint16_t ang = (uint16_t)(a % 36000);
              // Clamp to the sensor's rated max range (see LIDAR_MAX_RANGE_MM).
              if (dist > LIDAR_MAX_RANGE_MM) dist = LIDAR_MAX_RANGE_MM;
              lidarBuf[lidarHead] = { ang, dist, inten };
              lidarHead = (lidarHead + 1) % LIDAR_BUF_POINTS;
              lidarSeq++;
            }
            portEXIT_CRITICAL(&lidarMux);
            recWritePacket(buf);  // outside the critical section — flash I/O
          }
          len = 0;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void displayTask(void *pv) {
  unsigned long lastDraw = 0;
  for (;;) {
    unsigned long now = millis();
    if (displayDirty || now - lastDraw >= 1000) {
      lastDraw = now; displayDirty = false; updateDisplay();
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Polls the CST816 and fires button actions on a debounced press edge. Two
// consecutive 20 ms samples must agree before the state flips, which rejects the
// single-sample I2C glitch the panel emits right after power-up. Only this task
// and displayTask touch the framebuffer/state, so no extra locking is needed.
void touchTask(void *pv) {
  vTaskDelay(pdMS_TO_TICKS(1500));  // let the panel settle after boot
  bool prev = false;  // previous raw sample
  bool down = false;  // confirmed (debounced) pressed state
  for (;;) {
    TouchPt t = touchPoll();
    bool raw = t.pressed;
    if (raw && prev && !down) {            // confirmed new press
      down = true;
      int idx = hitTestButton(t.x, t.y);   // panel reports in landscape coords
      if (idx >= 0) {
        btnPressedIdx = idx;
        if (idx == 0)      personTrackingOn = !personTrackingOn;
        else if (idx == 1) bigAreaView      = !bigAreaView;
        else if (idx == 2) imuView          = !imuView;   // live IMU readout
        else if (idx == 3) recWant          = !recWant;  // start/stop recording
        displayDirty = true;
      }
    } else if (!raw && !prev && down) {    // confirmed release
      down = false;
      btnPressedIdx = -1;
      displayDirty = true;
    }
    prev = raw;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Samples the BMI160 at ~100 Hz, records type-1 IMU records while recording, and
// prints a live reading once a second so the sensor can be verified.
void imuTask(void *pv) {
  imuOk = imuInit();
  uint32_t lastPrint = 0, lastDirty = 0;
  for (;;) {
    if (!imuOk) { vTaskDelay(pdMS_TO_TICKS(1000)); imuOk = imuInit(); continue; }
    uint8_t d[12];
    if (imuReadReg(0x0C, d, 12)) {     // 0x0C: GYR XYZ then ACC XYZ, raw int16 LE
      int16_t gx = d[0]|(d[1]<<8),  gy = d[2]|(d[3]<<8),  gz = d[4]|(d[5]<<8);
      int16_t ax = d[6]|(d[7]<<8),  ay = d[8]|(d[9]<<8),  az = d[10]|(d[11]<<8);

      // Mount transform (+Y up -> conventional +Z up): X'=X, Y'=-Z, Z'=Y.
      // Done as pure int16 sign/permutation so the *recorded* sample is already
      // in the corrected frame — this matches the .ldim "mount-corrected" spec
      // and makes recorded gyro Z the yaw axis the floorplan tool deskews on.
      int16_t cg[3] = { gx, (int16_t)(-gz), gy };
      int16_t ca[3] = { ax, (int16_t)(-az), ay };
      uint8_t c[12];
      memcpy(c + 0, cg, 6);
      memcpy(c + 6, ca, 6);
      recWriteImu(c);

      // EMA-smooth the corrected values for a steady on-screen readout.
      const float a = 0.2f;
      imuAx += a * (ca[0] / 16384.0f - imuAx);
      imuAy += a * (ca[1] / 16384.0f - imuAy);
      imuAz += a * (ca[2] / 16384.0f - imuAz);
      imuGx += a * (cg[0] / 16.4f - imuGx);
      imuGy += a * (cg[1] / 16.4f - imuGy);
      imuGz += a * (cg[2] / 16.4f - imuGz);

      uint32_t now = millis();
      if (imuView && now - lastDirty >= 100) { lastDirty = now; displayDirty = true; }
      if (now - lastPrint >= 1000) {
        lastPrint = now;
        Serial.printf("[imu] acc=%.2f,%.2f,%.2f g  gyro=%.1f,%.1f,%.1f dps\n",
                      (double)imuAx, (double)imuAy, (double)imuAz,
                      (double)imuGx, (double)imuGy, (double)imuGz);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));     // 100 Hz
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(LCD_PWR, OUTPUT); digitalWrite(LCD_PWR, HIGH);
  pinMode(38, OUTPUT); digitalWrite(38, HIGH);
  delay(200);
  
  bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
  gfx_dev = new Arduino_RM67162(bus, LCD_RST, 1, false);
  gfx_dev->begin();
  gfx_dev->invertDisplay(false);
  gfx_dev->fillScreen(BLACK);
  gfx = new Arduino_Canvas(DISP_W, DISP_H, gfx_dev);
  gfx->begin(GFX_SKIP_OUTPUT_BEGIN); // device is already begun above
  updateDisplay();

  // CST816 capacitive touch on the AMOLED's I2C bus.
  pinMode(TOUCH_INT, INPUT_PULLUP);
  Wire.begin(TOUCH_SDA, TOUCH_SCL, 400000);

  // Second I2C bus for the external IMU.
  Wire1.begin(IMU_SDA, IMU_SCL, 400000);

  // Onboard 9 MB FAT partition for .ldim recordings (format on first boot).
  recMutex = xSemaphoreCreateMutex();
  if (!FFat.begin(true)) Serial.println("[rec] FFat mount failed - recording disabled");

  setupWiFi();
  // LD19P external-speed-control trigger: hold 500–1500 Hz @ 50% duty for >100 ms.
  // Once entered, the LD19P stays in external mode until power-cycle.
  ledcAttach(LIDAR_PWM_PIN, LIDAR_PWM_FREQ, LIDAR_PWM_RES);
  lidarPwmDuty = 128; lidarMotorOn = true; lidarPwmInvert = false;
  applyLidarPwm();
  delay(200);
  // After external mode is latched, hand off to the closed-loop controller,
  // which holds the default 10 Hz target (lidarTargetRpm = 600). Start near
  // 50% duty so it spins up and converges quickly.
  lidarPwmDuty = 128;
  applyLidarPwm();
  xTaskCreate([](void*){ for(;;){ server.handleClient(); vTaskDelay(1); } }, "web", 8192, NULL, 1, NULL);
  xTaskCreate(lidarTask, "lidar", 4096, NULL, 2, NULL);
  xTaskCreate(lidarControlTask, "lctrl", 2048, NULL, 1, NULL);
  xTaskCreate(personTask, "person", 4096, NULL, 1, NULL);
  xTaskCreate(displayTask, "disp", 4096, NULL, 1, NULL);
  xTaskCreate(localizerTask, "loc", 8192, NULL, 1, NULL);
  xTaskCreate(touchTask, "touch", 4096, NULL, 1, NULL);
  xTaskCreate(imuTask, "imu", 4096, NULL, 1, NULL);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "clear") {
      Serial.println("[cmd] Clearing WiFi settings...");
      prefs.begin("wifi", false);
      prefs.clear();
      prefs.end();
      WiFi.disconnect(true, true);
      Serial.println("[cmd] Done. Restarting...");
      delay(500);
      ESP.restart();
    } else if (cmd == "reboot") {
      Serial.println("[cmd] Restarting...");
      delay(500);
      ESP.restart();
    } else if (cmd == "ls") {
      File root = FFat.open("/");
      uint32_t total = 0; int count = 0;
      for (File f = root.openNextFile(); f; f = root.openNextFile()) {
        Serial.printf("  %-20s %8u bytes\n", f.name(), (unsigned)f.size());
        total += f.size(); count++;
        f.close();
      }
      root.close();
      Serial.printf("[ls] %d file(s), %u bytes used, %u free\n",
                    count, (unsigned)total, (unsigned)FFat.freeBytes());
    } else if (cmd.startsWith("rm ")) {
      String fn = cmd.substring(3); fn.trim();
      if (!fn.startsWith("/")) fn = "/" + fn;
      Serial.println(FFat.remove(fn) ? "[rm] ok" : "[rm] failed");
    } else if (cmd == "rmall") {
      int n = recRemoveAll();
      Serial.printf("[rmall] deleted %d file(s)\n", n);
    } else if (cmd == "i2c") {
      Serial.println("[i2c] scanning bus (SDA=3 SCL=2)...");
      int found = 0;
      for (uint8_t addr = 1; addr < 0x7F; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
          const char* note = "";
          if (addr == 0x15) note = " (CST816 touch)";
          else if (addr == 0x68 || addr == 0x69) note = " (IMU? MPU/BMI160/QMI8658)";
          else if (addr == 0x6A || addr == 0x6B) note = " (IMU? LSM6/BMI270)";
          else if (addr == 0x18 || addr == 0x19) note = " (accel?)";
          Serial.printf("  0x%02X%s\n", addr, note);
          found++;
        }
      }
      Serial.printf("[i2c] %d device(s) found\n", found);
      Serial.println("[i2c1] scanning IMU bus (SDA=41 SCL=39)...");
      int f1 = 0;
      for (uint8_t addr = 1; addr < 0x7F; addr++) {
        Wire1.beginTransmission(addr);
        if (Wire1.endTransmission() == 0) {
          Serial.printf("  0x%02X  whoami:", addr);
          // Dump candidate WHO_AM_I registers so we can identify the chip.
          for (uint8_t reg : {0x00, 0x0F, 0x75}) {
            Wire1.beginTransmission(addr);
            Wire1.write(reg);
            if (Wire1.endTransmission(false) == 0 && Wire1.requestFrom((int)addr, 1) == 1)
              Serial.printf(" [0x%02X]=0x%02X", reg, Wire1.read());
          }
          Serial.println();
          f1++;
        }
      }
      Serial.printf("[i2c1] %d device(s) found\n", f1);
    } else if (cmd == "help") {
      Serial.println("Available commands:");
      Serial.println("  clear  - Wipe WiFi credentials and reboot to AP mode");
      Serial.println("  reboot - Restart the ESP32");
      Serial.println("  ls     - List recordings on the FAT partition");
      Serial.println("  rm F   - Delete recording F (e.g. rm scan_001.ldim)");
      Serial.println("  rmall  - Delete all recordings (skips the active one)");
      Serial.println("  i2c    - Scan the I2C bus (touch / IMU detection)");
    }
  }
  delay(100);
}
