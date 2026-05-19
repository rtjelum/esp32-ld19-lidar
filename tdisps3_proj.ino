#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Arduino_GFX_Library.h>

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

static const char ZK_LEVEL_DATA[] PROGMEM =
  "####\n# .#\n#  ###\n#*@  #\n#  $ #\n#  ###\n####"
  "|"
  "######\n#    #\n# #@ #\n# $* #\n# .* #\n#    #\n######"
  "|"
  "  ####\n###  ####\n#     $ #\n# #  #$ #\n# . .#@ #\n#########"
  "|"
  "########\n#      #\n# .**$@#\n#      #\n#####  #\n    ####"
  "|"
  " #######\n #     #\n # .$. #\n## $@$ #\n #  .$. #\n#      #\n########"
  "|"
  "###### #####\n#    ###   #\n# $$     #@#\n# $ #...   #\n#   ########\n#####"
  "|"
  "#######\n#     #\n# .$. #\n# $.$ #\n# .$. #\n# $.$ #\n#  @  #\n#######"
  "|"
  "  ######\n  # ..@#\n  # $$ #\n  ## ###\n   # #\n   # #\n#### #\n#    ##\n# #   #\n#   # #\n###   #\n  #####"
  "|"
  "#####\n#.  ##\n#@$$ #\n##   #\n ##  #\n  ##.#\n   ###"
  "|"
  "      #####\n      #.  #\n      #.# #\n#######.# #\n# @ $ $ $ #\n# # # # ###\n#       #\n#########"
  "|"
  "  ######\n  #    #\n  # ##@##\n### # $ #\n# ..# $ #\n#       #\n#  ######\n####"
  "|"
  "#####\n#   ##\n# $  #\n## $ ####\n ###@.  #\n  #  .# #\n  #     #\n  #######"
  "|"
  "####\n#. ##\n#.@ #\n#. $#\n##$ ###\n # $  #\n #    #\n #  ###\n ####"
  "|"
  "#######\n#     #\n# # # #\n# . $*@#\n#   ###\n#####";

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
static const int LIDAR_BUF_POINTS = 480; // ~40 packets, ≈ 1 full rotation @10Hz
static volatile uint8_t  lidarPwmDuty   = 128;   // 50% — also LD19 external-mode entry duty
static volatile bool     lidarMotorOn   = true;
static volatile uint32_t lidarPwmFreq   = LIDAR_PWM_FREQ;
static volatile bool     lidarPwmInvert = false;
static volatile uint16_t lidarTargetRpm = 0;     // 0 = manual duty, >0 = closed-loop target
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
static float personAngle[4]; // degrees, 0°=left, 90°=front, 180°=right
static float personDist[4];  // meters
static float personX[4];     // meters, +right
static float personY[4];     // meters, +forward
static float bgDist[360];    // mm, per-bin rolling background for shadow detection
static volatile float lidarAreaM2 = 0.0f; // scanned polygon area in m²

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
    gfx->setCursor(60, 120); gfx->print("Connect: battle-setup");
    gfx->setCursor(60, 150); gfx->print("Open:    http://192.168.4.1/");
    gfx->flush();
    return;
  }

  uint32_t pLast; uint8_t pCnt; float pAng[4], pDst[4];
  portENTER_CRITICAL(&personMux);
  pLast = personLastSeen; pCnt = personCount;
  for (uint8_t i = 0; i < pCnt && i < 4; i++) { pAng[i] = personAngle[i]; pDst[i] = personDist[i]; }
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
    gfx->setCursor(rightCx - (int)strlen(unit) * 9, 175);
    gfx->print(unit);

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
  gfx->setCursor((DISP_W - (int)url.length() * 12) / 2, 190);
  gfx->print(url);

  gfx->setTextSize(1);
  gfx->setTextColor(0x8410);
  const char* sk = "/sokoban  for game";
  gfx->setCursor((DISP_W - (int)strlen(sk) * 6) / 2, 215);
  gfx->print(sk);
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
    // Rear bins are excluded so the clamped 4 m back ring doesn't inflate the result.
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

    const int GAP_TOL = 2;
    int start = -1, gap = 0;
    uint8_t cnt = 0;
    float pa[4], pd[4], px[4], py[4];
    for (int i = 0; i <= 180; i++) {
      int b = i < 180 ? (180 + i) % 360 : -1;
      bool on = b >= 0 && cur[b] > 0 && bgDist[b] > 400.0f && bgDist[b] - cur[b] > 300.0f;
      if (on) { if (start < 0) start = i; gap = 0; }
      else if (start >= 0) {
        gap++;
        if (gap > GAP_TOL || i == 180) {
          int lastShadow = i - gap;
          int len = lastShadow - start + 1;
          if (len >= 2 && len <= 60 && cnt < 4) {
            float ds[60]; int nds = 0;
            for (int k = start; k <= lastShadow && nds < 60; k++) {
              int bk = (180 + k) % 360;
              if (cur[bk] > 0) ds[nds++] = cur[bk];
            }
            if (nds >= 2) {
              for (int a = 1; a < nds; a++) { // insertion sort, nds ≤ 60
                float v = ds[a]; int q = a - 1;
                while (q >= 0 && ds[q] > v) { ds[q+1] = ds[q]; q--; }
                ds[q+1] = v;
              }
              float md = ds[nds/2];
              float pw = 2.0f * md * sinf((float)len * PI / 360.0f);
              if (pw >= 100.0f && pw <= 1200.0f) {
                int midBin = (180 + (start + lastShadow) / 2) % 360;
                float rotDeg = (float)midBin - 270.0f; // -90..+89 (front semicircle)
                float rad    = rotDeg * PI / 180.0f;
                float xm     = -sinf(rad) * md / 1000.0f;
                float ym     =  cosf(rad) * md / 1000.0f;
                px[cnt] = xm; py[cnt] = ym;
                pa[cnt] = atan2f(xm, ym) * 180.0f / PI + 90.0f;
                pd[cnt] = md / 1000.0f;
                cnt++;
              }
            }
          }
          start = -1; gap = 0;
        }
      }
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
  server.send(200, "text/html", R"rawhtml(<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Setup</title><style>
body{font-family:sans-serif;background:#050510;color:#fff;padding:20px;text-align:center}
input{width:100%;max-width:300px;padding:10px;margin:10px 0;border-radius:5px;border:none}
button{background:#7c6fff;color:#fff;padding:10px 20px;border:none;border-radius:5px;cursor:pointer}
.net{cursor:pointer;padding:8px;border-bottom:1px solid #222}
</style></head><body><h1>WiFi Setup</h1>
<div id='f'>
<input id='s' placeholder='SSID'><br><input id='p' type='password' placeholder='Password'><br>
<button onclick='c()'>Connect</button></div>
<div id='l' style='margin-top:20px'>Scanning for networks...</div>
<script>
function c(){
  fetch('/wifi/connect?ssid='+encodeURIComponent(document.getElementById('s').value)+'&pass='+encodeURIComponent(document.getElementById('p').value))
  .then(()=>alert('Rebooting...'));
}
fetch('/wifi/scan').then(r=>r.json()).then(d=>{
  if(!d.length){document.getElementById('l').innerHTML='No networks found.';return;}
  let h='<h3>Available Networks:</h3>';
  d.forEach(n=>h+='<div class="net" onclick="document.getElementById(\'s\').value=\''+n.ssid+'\'">'+n.ssid+' ('+n.rssi+'dBm)</div>');
  document.getElementById('l').innerHTML=h;
}).catch(e=>{
  document.getElementById('l').innerHTML='Scan failed. Please enter details manually.';
});
</script></body></html>)rawhtml");
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
  uint8_t pCnt; float pA[4], pD[4], pX[4], pY[4];
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
  server.send(200, "text/html", R"rawhtml(<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>LiDAR</title><style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:sans-serif;background:#050510;color:#e8eaff;min-height:100vh;display:flex;flex-direction:column;align-items:center;padding:10px;gap:8px}
h1{font-size:1.2rem;letter-spacing:3px;color:#7cf0ff;margin:4px 0}
#hud{font-size:.85rem;color:#a8b0d0;display:flex;gap:14px;flex-wrap:wrap;justify-content:center}
#hud b{color:#7cf0ff}
canvas{background:#02050d;border:1px solid #1c2848;border-radius:8px;max-width:100%;height:auto;touch-action:none}
.row{display:flex;gap:8px;align-items:center;font-size:.85rem;color:#a8b0d0;flex-wrap:wrap;justify-content:center}
button{background:#142036;border:1px solid #2a3a60;color:#cfe1ff;padding:5px 12px;border-radius:6px;cursor:pointer}
button.on{background:#1d3a6e;border-color:#7cf0ff;color:#7cf0ff}
a{color:#7cf0ff;font-size:.8rem}
#ppllist{font-size:.8rem;color:#cfe1ff;background:#0a1530;border:1px solid #1c2848;border-radius:6px;padding:6px 12px;min-width:240px;text-align:left;font-family:monospace;white-space:pre;line-height:1.4em}
</style></head><body>
<h1>LiDAR LD19</h1>
<div id='hud'><span>RPM <b id='rpm'>-</b></span><span>Points <b id='np'>-</b></span><span>Area <b id='area'>-</b> m&sup2;</span><span>FPS <b id='fps'>-</b></span><span>Ppl <b id='ppl'>-</b></span></div>
<canvas id='c'></canvas>
<div class='row'>
  <button id='pplBtn' class='on'>People</button>
  <button id='raw'>Raw</button>
  <button id='hold'>Hold</button>
</div>
<div id='ppllist'>People detection off</div>
<a href='/sokoban'>Sokoban &rarr;</a>
<script>
var c=document.getElementById('c'),ctx=c.getContext('2d');
var dpr=Math.min(window.devicePixelRatio||1,2);
var paused=false,rawOn=false,frames=0,lastFps=performance.now();
// Fixed world rectangle: x in [-4,+4] m left..right, y in [0,5] m sensor..forward
var WX0=-4,WX1=4,WY0=0,WY1=5;
var prevX=new Float32Array(360),prevY=new Float32Array(360),prevTime=new Float64Array(360);
var prevSeen=new Uint8Array(360),speedBins=new Float32Array(360);
var personsOn=true,lastPersons=[];
var SENSOR_ROT_DEG=270; // physical front sits at sensor angle 270°
var WALL_CSS=70;   // wall extrusion height in CSS px
var WALL_PX=WALL_CSS*dpr;
function sz(){
  var availW=window.innerWidth-20, availH=window.innerHeight-260;
  var aspect=(WX1-WX0)/(WY1-WY0); // floor width:height (world units)
  var floorW=Math.min(availW, (availH-WALL_CSS)*aspect, 900);
  if(floorW<200)floorW=200;
  var floorH=floorW/aspect;
  var h=floorH+WALL_CSS;
  c.style.width=floorW+'px';c.style.height=h+'px';
  c.width=floorW*dpr;c.height=h*dpr;
}
sz();window.addEventListener('resize',sz);
document.getElementById('hold').onclick=function(){
  paused=!paused;this.innerText=paused?'Run':'Hold';
};
document.getElementById('raw').onclick=function(){
  rawOn=!rawOn;this.classList.toggle('on',rawOn);
};
document.getElementById('pplBtn').onclick=function(){
  personsOn=!personsOn;this.classList.toggle('on',personsOn);
};
function w2x(x){return (x-WX0)/(WX1-WX0)*c.width;}
function w2y(y){return WALL_PX + (y-WY0)/(WY1-WY0)*(c.height-WALL_PX);}
function drawSmiley(px,py,r){
  ctx.fillStyle='rgba(0,0,0,0.45)';ctx.beginPath();ctx.ellipse(px,py+r*0.95,r*0.75,r*0.22,0,0,Math.PI*2);ctx.fill();
  ctx.fillStyle='#ffd54f';ctx.beginPath();ctx.arc(px,py,r,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle='#3e2723';ctx.lineWidth=Math.max(1,r*0.09);ctx.stroke();
  ctx.fillStyle='#3e2723';
  ctx.beginPath();ctx.arc(px-r*0.35,py-r*0.18,r*0.13,0,Math.PI*2);ctx.fill();
  ctx.beginPath();ctx.arc(px+r*0.35,py-r*0.18,r*0.13,0,Math.PI*2);ctx.fill();
  ctx.beginPath();ctx.arc(px,py+r*0.05,r*0.45,0.15*Math.PI,0.85*Math.PI);ctx.stroke();
}
function draw(d){
  if(d&&typeof d.area==='number')document.getElementById('area').innerText=d.area.toFixed(2);
  var w=c.width,h=c.height;
  ctx.fillStyle='#02050d';ctx.fillRect(0,0,w,h);
  if(!d||!d.pts){document.getElementById('np').innerText='0';return;}
  // Build per-degree bin array (display frame, 0..359). Take min distance per bin.
  var bins=new Float32Array(360);
  for(var i=0;i<d.pts.length;i++){
    var q=d.pts[i];
    if(q[1]<=0) continue;
    var ra=q[0]-SENSOR_ROT_DEG*100;if(ra<0)ra+=36000;
    var bin=Math.floor(ra/100)%360;if(bin<0)bin+=360;
    if(bins[bin]===0 || q[1]<bins[bin]) bins[bin]=q[1];
  }
  // Fill gaps with average of nearest valid neighbors so missing bins (90° dead zone, etc.) interpolate cleanly
  for(var k=0;k<360;k++){
    if(bins[k]>0) continue;
    var L=0,R=0;
    for(var s=1;s<=180;s++){
      if(L===0){var li=(k-s+360)%360; if(bins[li]>0)L=bins[li];}
      if(R===0){var ri=(k+s)%360; if(bins[ri]>0)R=bins[ri];}
      if(L>0&&R>0)break;
    }
    if(L>0&&R>0) bins[k]=(L+R)/2;
    else if(L>0) bins[k]=L;
    else if(R>0) bins[k]=R;
  }
  var pp=[];
  var now=performance.now();
  for(var b=0;b<360;b++)speedBins[b]=0;
  for(var bin=0;bin<360;bin++){
    var dmm=bins[bin];
    if(dmm<=0) continue;
    var ang=bin*Math.PI/180;
    var wx=-Math.sin(ang)*dmm/1000;
    var wy= Math.cos(ang)*dmm/1000;
    var xmm=wx*1000,ymm=wy*1000;
    if(prevSeen[bin]){
      var dxw=xmm-prevX[bin], dyw=ymm-prevY[bin];
      var dt=(now-prevTime[bin])/1000;
      if(dt>0.01&&dt<2){speedBins[bin]=Math.max(speedBins[bin],Math.sqrt(dxw*dxw+dyw*dyw)/dt);}
    }
    prevX[bin]=xmm; prevY[bin]=ymm; prevTime[bin]=now; prevSeen[bin]=1;
    if(wx<WX0)wx=WX0;else if(wx>WX1)wx=WX1;
    if(wy<WY0)wy=WY0;else if(wy>WY1)wy=WY1;
    pp.push([wx,wy,bin*100,bin]);
  }
  pp.sort(function(a,b){return a[2]-b[2];});
  lastPersons=(personsOn&&d.persons)?d.persons:[];
  document.getElementById('ppl').innerText=personsOn?lastPersons.length:'-';
  var pl=document.getElementById('ppllist');
  if(!personsOn){pl.innerText='People detection off';}
  else if(lastPersons.length===0){pl.innerText='People: 0';}
  else{var lines=['People: '+lastPersons.length];
    for(var k=0;k<lastPersons.length;k++){var pn=lastPersons[k];
      lines.push((k+1)+': x='+(pn.x>=0?'+':'')+pn.x.toFixed(2)+' y='+(pn.y>=0?'+':'')+pn.y.toFixed(2)+' m  a='+Math.round(pn.a)+' d='+pn.d.toFixed(2)+'m');}
    pl.innerText=lines.join('\n');}
  if(rawOn){
    for(var i=0;i<pp.length;i++){
      var p=pp[i];
      var spd=speedBins[p[3]];
      var px=w2x(p[0]), py=w2y(p[1]);
      var t=Math.min(1,spd/1000);
      var hue=60-60*t;
      ctx.fillStyle='hsla('+hue+',90%,55%,0.95)';
      ctx.beginPath();ctx.arc(px,py,(2+2*t)*dpr,0,Math.PI*2);ctx.fill();
    }
    if(personsOn)for(var pi=0;pi<lastPersons.length;pi++){var p=lastPersons[pi];drawSmiley(w2x(p.x),w2y(p.y),14*dpr);}
    document.getElementById('np').innerText=pp.length;
    document.getElementById('rpm').innerText=d.rpm;
    return;
  }
  if(pp.length>=2){
    // floor polygon: sensor (0,0) → sorted points → close
    var gp=[];
    gp.push([w2x(0),w2y(0)]);
    for(var i=0;i<pp.length;i++){gp.push([w2x(pp[i][0]),w2y(pp[i][1])]);}
    ctx.beginPath();
    for(var i=0;i<gp.length;i++){if(i===0)ctx.moveTo(gp[i][0],gp[i][1]);else ctx.lineTo(gp[i][0],gp[i][1]);}
    ctx.closePath();
    ctx.fillStyle='rgba(18,38,64,0.55)';ctx.fill();
    // extruded walls along the LiDAR perimeter
    var segs=[];
    for(var i=1;i<gp.length;i++){segs.push({a:gp[i],b:gp[i===gp.length-1?1:i+1]});}
    segs.sort(function(s1,s2){return (s1.a[1]+s1.b[1])-(s2.a[1]+s2.b[1]);});
    for(var i=0;i<segs.length;i++){
      var s=segs[i];
      var ax=s.a[0],ay=s.a[1],bx=s.b[0],by=s.b[1];
      var tay=ay-WALL_PX,tby=by-WALL_PX;
      var topY=Math.min(tay,tby),botY=Math.max(ay,by);
      var grad=ctx.createLinearGradient(0,topY,0,botY);
      grad.addColorStop(0,   'rgba(200,235,250,0.95)');
      grad.addColorStop(0.35,'rgba(95,165,210,0.9)');
      grad.addColorStop(1,   'rgba(15,45,85,0.85)');
      ctx.beginPath();
      ctx.moveTo(ax,ay);ctx.lineTo(bx,by);ctx.lineTo(bx,tby);ctx.lineTo(ax,tay);
      ctx.closePath();
      ctx.fillStyle=grad;ctx.fill();
      ctx.strokeStyle='rgba(230,250,255,0.95)';
      ctx.lineWidth=1.4*dpr;
      ctx.beginPath();ctx.moveTo(ax,tay);ctx.lineTo(bx,tby);ctx.stroke();
    }
  }
  // Sensor dot
  ctx.fillStyle='#7cf0ff';
  ctx.beginPath();ctx.arc(w2x(0),w2y(0),5*dpr,0,Math.PI*2);ctx.fill();

  if(personsOn)for(var pi=0;pi<lastPersons.length;pi++){var p=lastPersons[pi];var prad=Math.max(12,28-22*p.d/4)*dpr;drawSmiley(w2x(p.x),w2y(p.y)-WALL_PX*1.3,prad);}
  document.getElementById('np').innerText=pp.length;
  document.getElementById('rpm').innerText=d.rpm;
}
async function poll(){
  if(!paused){
    try{
      var r=await fetch('/lidar/scan',{cache:'no-store'});
      var d=await r.json();
      draw(d);
      frames++;
      var now=performance.now();
      if(now-lastFps>1000){document.getElementById('fps').innerText=(frames*1000/(now-lastFps)).toFixed(1);frames=0;lastFps=now;}
    }catch(e){}
  }
  setTimeout(poll,80);
}
poll();
</script></body></html>)rawhtml");
}

void handleSokoban() {
  server.send(200, "text/html", R"rawhtml(<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Sokoban</title><style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',sans-serif;background:linear-gradient(160deg,#3a1c5a 0%,#1a0d2e 60%,#050510 100%);color:#fff3e0;min-height:100vh;padding:14px;display:flex;flex-direction:column;align-items:center;gap:10px}
h1{font-size:1.4rem;letter-spacing:4px;background:linear-gradient(90deg,#ffd54f,#ff8a65,#f06292);-webkit-background-clip:text;background-clip:text;color:transparent;margin:8px 0;font-weight:800}
#st{font-size:.9rem;color:#ffd54f;text-align:center}
canvas{background:#1b0d2e;border:2px solid #ffd54f;border-radius:12px;box-shadow:0 4px 24px rgba(255,167,38,.25);max-width:100%;height:auto;touch-action:none}
.btn{width:100%;max-width:310px;padding:12px;background:linear-gradient(135deg,#ff7043,#ffca28);border:none;border-radius:10px;color:#3e2723;font-size:.9rem;font-weight:800;cursor:pointer;margin-top:5px}
.btn.s{background:rgba(255,255,255,.08);color:#ffe082;font-weight:500;border:1px solid rgba(255,213,79,.2)}
.pad{display:grid;grid-template-columns:repeat(3,60px);gap:10px;margin-top:10px}
.pb{width:60px;height:60px;background:rgba(255,213,79,.15);border:1px solid rgba(255,213,79,.4);border-radius:10px;color:#ffe082;display:flex;align-items:center;justify-content:center;font-size:24px;user-select:none;-webkit-tap-highlight-color:transparent}
.pb:active{background:rgba(255,167,38,.5);color:#fff}
@media (hover:hover) and (pointer:fine){.pad,#btnRst,#btnReset{display:none!important}}
</style></head><body>
<h1>SOKOBAN</h1><div id='st'>Moves: 0</div><canvas id='g' width='320' height='240'></canvas>
<div class='pad'>
<div></div><div class='pb' onclick='mvK(0,-1)'>&uarr;</div><div></div>
<div class='pb' onclick='mvK(-1,0)'>&larr;</div><div class='pb' onclick='mvK(0,1)'>&darr;</div><div class='pb' onclick='mvK(1,0)'>&rarr;</div>
</div>
<button class='btn' id='btnRst' onclick='reset()'>Restart Level</button>
<button class='btn s' onclick='location.href="/"'>Back to Menu</button>
<script>
var L=[["####","# .#","#  ###","#*@  #","#  $ #","#  ###","####"],
["######","#    #","# #@ #","# $* #","# .* #","#    #","######"],
["  ####","###  ####","#     $ #","# #  #$ #","# . .#@ #","#########"],
["########","#      #","# .**$@#","#      #","#####  #","    ####"],
[" #######"," #     #"," # .$. #","## $@$ #"," #  .$. #","#      #","########"],
["###### #####","#    ###   #","# $$     #@#","# $ #...   #","#   ########","#####"],
["#######","#     #","# .$. #","# $.$ #","# .$. #","# $.$ #","#  @  #","#######"],
["  ######","  # ..@#","  # $$ #","  ## ###","   # #","   # #","#### #","#    ##","# #   #","#   # #","###   #","  #####"],
["#####","#.  ##","#@$$ #","##   #"," ##  #","  ##.#","   ###"],
["      #####","      #.  #","      #.# #","#######.# #","# @ $ $ $ #","# # # # ###","#       #","#########"],
["  ######","  #    #","  # ##@##","### # $ #","# ..# $ #","#       #","#  ######","####"],
["#####","#   ##","# $  #","## $ ####"," ###@.  #","  #  .# #","  #     #","  #######"],
["####","#. ##","#.@ #","#. $#","##$ ###"," # $  #"," #    #"," #  ###"," ####"],
["#######","#     #","# # # #","#. $*@#","#   ###","#####"],
["     ###","######@##","#    .* #","#   #   #","#####$# #","    #   #","    #####"],
[" ####"," #  ####"," #     ##","## ##   #","#. .# @$##","#   # $$ #","#  .#    #","##########"],
["#####","# @ #","#...#","#$$$##","#    #","#    #","######"],
["#######","#     #","#. .  #","# ## ##","#  $ #","###$ #","  #@ #","  #  #","  ####"],
["########","#   .. #","#  @$$ #","##### ##","   #  #","   #  #","   #  #","   ####"],
["#######","#     ###","#  @$$..#","#### ## #","  #     #","  #  ####","  #  #","  ####"],
["####","#  ####","# . . #","# $$#@#","##    #"," ######"],
["#####","#   ###","#. .  #","#   # #","## #  #"," #@$$ #"," #    #"," #  ###"," ####"],
["#######","#  *  #","#     #","## # ##"," #$@.#"," #   #"," #####"],
["# #####","  #   #","###$$@#","#   ###","#     #","# . . #","#######"],
[" ####"," #  ###"," # $$ #","##... #","#  @$ #","#   ###","#####"],
[" #####"," # @ #"," #   #","###$ #","# ...#","# $$ #","###  #","  ####"],
["######","#   .#","# ## ##","#  $$@#","# #   #","#.  ###","#####"],
["#####","#   #","# @ #","# $$###","##. . #"," #    #"," ######"],
["     #####","     #   ##","     #    #"," ######   #","##     #. #","# $ $ @  ##","# ######.#","#        #","##########"],
["####","#  ###","# $$ #","#... #","# @$ #","#   ##","#####"],
["  ####"," ##  #","##@$.##","# $$  #","# . . #","###   #","  #####"],
[" ####","##  ###","#     #","#.**$@#","#   ###","##  #"," ####"],
["#######","#. #  #","#  $  #","#. $#@#","#  $  #","#. #  #","#######"],
["  ####","###  ####","#       #","#@$***. #","#       #","#########"],
["  ####"," ##  #"," #. $#"," #.$ #"," #.$ #"," #.$ #"," #. $##"," #   @#"," ##   #","  #####"],
["####","#  ############","# $ $ $ $ $ @ #","# .....       #","###############"],
["      ###","##### #.#","#   ###.#","#   $ #.#","# $  $  #","#####@# #","    #   #","    #####"],
["##########","#        #","# ##.### #","# # $$ . #","# . @$## #","#####    #","    ######"],
["#####","#   ####","# # # .#","#    $ ###","### #$.  #","#   #@   #","# # ######","#   #","#####"],
[" #####"," #   #","##   ##","# $$$ #","# .+. #","#######"],
["#######","#     #","#@$$$ ##","#  #...#","##    ##"," ######"],
["   ####","   #  #","   #@ #","####$.#","#   $.#","# # $.#","#    ##","######"],
["     ####","     # @#","     #  #","###### .#","#   $  .#","#  $$# .#","#    ####","###  #","  ####"],
["#####","#@$.#","#####"],
["######","#... #","#  $ #","# #$##","#  $ #","#  @ #","######"],
[" ######","##    #","#  ## #","# # $ #","#  * .#","## #@##"," #   #"," #####"],
["  #######","###     #","# $ $   #","# ### #####","# @ . .   #","#   ###   #","##### #####"],
["######","#  @ #","#  # ##","# .#  ##","# .$$$ #","# .#   #","####   #","   #####"],
["######","# @  #","# $# #","# $  #","# $ ##","### ####"," #  #  #"," #...  #"," #     #"," #######"],
["  ####","###  #####","#  $  @..#","# $    # #","### #### #","  #      #","  ########"]];
var cur=0,map=[],px,py,mv=0;
var c=document.getElementById('g'),ctx=c.getContext('2d');
// Isometric constants
var TW=20,TH=12,WH=11,BH=14,PH=36,offX=0,offY=0;
function ix(x,y){return (x-y)*TW+offX;}
function iy(x,y){return (x+y)*TH+offY;}
function diamond(cx,cy,fill,stroke){
  ctx.beginPath();
  ctx.moveTo(cx,cy);ctx.lineTo(cx+TW,cy+TH);ctx.lineTo(cx,cy+2*TH);ctx.lineTo(cx-TW,cy+TH);ctx.closePath();
  if(fill){ctx.fillStyle=fill;ctx.fill();}
  if(stroke){ctx.strokeStyle=stroke;ctx.stroke();}
}
function cube(x,y,ht,topC,leftC,rightC,edge){
  var cx=ix(x,y),cy=iy(x,y);
  ctx.fillStyle=rightC;
  ctx.beginPath();
  ctx.moveTo(cx+TW,cy+TH);ctx.lineTo(cx,cy+2*TH);ctx.lineTo(cx,cy+2*TH-ht);ctx.lineTo(cx+TW,cy+TH-ht);
  ctx.closePath();ctx.fill();
  if(edge){ctx.strokeStyle=edge;ctx.lineWidth=1;ctx.stroke();}
  ctx.fillStyle=leftC;
  ctx.beginPath();
  ctx.moveTo(cx-TW,cy+TH);ctx.lineTo(cx,cy+2*TH);ctx.lineTo(cx,cy+2*TH-ht);ctx.lineTo(cx-TW,cy+TH-ht);
  ctx.closePath();ctx.fill();
  if(edge)ctx.stroke();
  ctx.fillStyle=topC;
  ctx.beginPath();
  ctx.moveTo(cx,cy-ht);ctx.lineTo(cx+TW,cy+TH-ht);ctx.lineTo(cx,cy+2*TH-ht);ctx.lineTo(cx-TW,cy+TH-ht);
  ctx.closePath();ctx.fill();
  if(edge)ctx.stroke();
}
function drawPawn(x,y,colA,colB){
  var cx=ix(x,y),cy=iy(x,y)+TH;
  ctx.fillStyle='rgba(0,0,0,0.35)';
  ctx.beginPath();ctx.ellipse(cx,cy+TH*0.6,TW*0.45,TH*0.45,0,0,Math.PI*2);ctx.fill();
  var bodyH=Math.max(TW*0.95,18);
  ctx.fillStyle=colA;
  ctx.beginPath();ctx.ellipse(cx,cy-bodyH*0.35,TW*0.38,bodyH*0.55,0,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle=colB;ctx.lineWidth=1.5;ctx.stroke();
  var hr=Math.max(TW*0.28,5);
  ctx.fillStyle='#ffe0b2';
  ctx.beginPath();ctx.arc(cx,cy-bodyH*0.95,hr,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle='#5d4037';ctx.lineWidth=1;ctx.stroke();
  ctx.fillStyle='#3e2723';
  ctx.beginPath();ctx.arc(cx-hr*0.4,cy-bodyH*0.95,Math.max(hr*0.18,1),0,Math.PI*2);ctx.fill();
  ctx.beginPath();ctx.arc(cx+hr*0.4,cy-bodyH*0.95,Math.max(hr*0.18,1),0,Math.PI*2);ctx.fill();
}
function drawGoal(x,y){
  var cx=ix(x,y),cy=iy(x,y)+TH;
  ctx.fillStyle='#f8bbd0';
  ctx.beginPath();ctx.ellipse(cx,cy,TW*0.42,TH*0.62,0,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle='#e91e63';ctx.lineWidth=2;ctx.stroke();
  ctx.fillStyle='#e91e63';
  ctx.beginPath();ctx.ellipse(cx,cy,TW*0.16,TH*0.24,0,0,Math.PI*2);ctx.fill();
  ctx.lineWidth=1;
}
function drawSparkle(x,y){
  var cx=ix(x,y),cy=iy(x,y)+TH-BH;
  ctx.strokeStyle='#fff59d';ctx.lineWidth=2;
  ctx.beginPath();
  ctx.moveTo(cx-TW*0.25,cy);ctx.lineTo(cx+TW*0.25,cy);
  ctx.moveTo(cx,cy-TH*0.4);ctx.lineTo(cx,cy+TH*0.4);
  ctx.stroke();ctx.lineWidth=1;
}
function load(i){
  mv=0;map=L[i].map(function(r){return r.split('');});
  for(var y=0;y<map.length;y++)for(var x=0;x<map[y].length;x++){
    if(map[y][x]=='@'){px=x;py=y;map[y][x]=' ';}
    if(map[y][x]=='+'){px=x;py=y;map[y][x]='.';}
  }
  draw();
}
function cliffR(x,y){
  var cx=ix(x,y),cy=iy(x,y);
  ctx.fillStyle='#5d4037';
  ctx.beginPath();
  ctx.moveTo(cx+TW,cy+TH);ctx.lineTo(cx,cy+2*TH);ctx.lineTo(cx,cy+2*TH+PH);ctx.lineTo(cx+TW,cy+TH+PH);
  ctx.closePath();ctx.fill();
}
function cliffL(x,y){
  var cx=ix(x,y),cy=iy(x,y);
  ctx.fillStyle='#8d6e63';
  ctx.beginPath();
  ctx.moveTo(cx-TW,cy+TH);ctx.lineTo(cx,cy+2*TH);ctx.lineTo(cx,cy+2*TH+PH);ctx.lineTo(cx-TW,cy+TH+PH);
  ctx.closePath();ctx.fill();
}
function draw(){
  var h=map.length,w=0;
  for(var i=0;i<h;i++)if(map[i].length>w)w=map[i].length;
  if(w<1||h<1)return;
  TW=Math.min(22,Math.max(10,Math.floor(440/(w+h))));
  TH=Math.round(TW*0.58);
  WH=Math.round(TW*0.55);
  BH=Math.round(TW*0.75);
  PH=Math.round(TW*1.8);
  var topPad=Math.round(TW*1.2);
  c.width=(w+h)*TW+4;
  c.height=(w+h)*TH+topPad+PH+8;
  offX=h*TW+2;
  offY=topPad+4;
  // Sunset gradient backdrop
  var grad=ctx.createLinearGradient(0,0,0,c.height);
  grad.addColorStop(0,'#7e57c2');
  grad.addColorStop(0.55,'#f06292');
  grad.addColorStop(1,'#ffb74d');
  ctx.fillStyle=grad;ctx.fillRect(0,0,c.width,c.height);
  // Flood-fill reachable cells from player
  var reach={};
  var st=[[px,py]];
  while(st.length){
    var p=st.pop(),qx=p[0],qy=p[1],k=qx+','+qy;
    if(reach[k])continue;
    if(qy<0||qy>=h)continue;
    var row=map[qy]||[];
    if(qx<0||qx>=row.length)continue;
    if(row[qx]=='#')continue;
    reach[k]=true;
    st.push([qx+1,qy]);st.push([qx-1,qy]);st.push([qx,qy+1]);st.push([qx,qy-1]);
  }
  // Mesa = reachable cells, plus '#' cells whose orthogonal neighbours are all on-grid
  // and are either walls or reachable (i.e. interior walls). Perimeter '#' cells are excluded.
  function isMesa(x,y){
    if(y<0||y>=h)return false;
    var row=map[y];if(!row||x<0||x>=row.length)return false;
    if(reach[x+','+y])return true;
    if(row[x]!='#')return false;
    var dirs=[[1,0],[-1,0],[0,1],[0,-1]];
    for(var d=0;d<4;d++){
      var nx=x+dirs[d][0],ny=y+dirs[d][1];
      if(ny<0||ny>=h)return false;
      var nr=map[ny];if(!nr||nx<0||nx>=nr.length)return false;
      var v=nr[nx];
      if(v!='#'&&!reach[nx+','+ny])return false;
    }
    return true;
  }
  // Render row-major: cliffs first (extend down), then top diamond, then objects
  for(var y=0;y<h;y++){
    var row=map[y];
    for(var x=0;x<row.length;x++){
      if(!isMesa(x,y))continue;
      if(!isMesa(x+1,y))cliffR(x,y);
      if(!isMesa(x,y+1))cliffL(x,y);
      var even=((x+y)%2)===0;
      diamond(ix(x,y),iy(x,y),even?'#e1f5fe':'#b3e5fc','#4fc3f7');
      var v=row[x];
      if(v=='#'){
        cube(x,y,WH,'#ffe082','#ffb74d','#fb8c00','#5d4037');
      } else {
        if(v=='.')drawGoal(x,y);
        if(v=='$'||v=='*'){
          if(v=='*'){cube(x,y,BH,'#ffd700','#ffc107','#ff8f00','#4e342e');drawSparkle(x,y);}
          else      {cube(x,y,BH,'#a5d6a7','#66bb6a','#388e3c','#1b5e20');}
        }
      }
      if(x===px&&y===py)drawPawn(x,y,'#26c6da','#006064');
    }
  }
  document.getElementById('st').innerText="Level "+(cur+1)+" — Moves: "+mv;
}
function mvK(dx,dy){
    var nx=px+dx,ny=py+dy,t=map[ny]&&map[ny][nx];
    if(t==' '||t=='.'){px=nx;py=ny;mv++;}
    else if(t=='$'||t=='*'){
      var bx=nx+dx,by=ny+dy,bt=map[by]&&map[by][bx];
      if(bt==' '||bt=='.'){
        map[ny][nx]=(t=='$')?' ':'.';
        map[by][bx]=(bt==' ')?'$':'*';
        px=nx;py=ny;mv++;
      }
    }
    draw();
    if(!map.some(r=>r.includes('.'))){
      setTimeout(()=>{if(cur<L.length-1){cur++;load(cur);}else{alert('All levels cleared!');cur=0;load(0);}},100);
    }
}
window.onkeydown=e=>{
  var dx=0,dy=0;
  if(e.key=='ArrowUp')dy=-1;if(e.key=='ArrowDown')dy=1;
  if(e.key=='ArrowLeft')dx=-1;if(e.key=='ArrowRight')dx=1;
  if(dx||dy)mvK(dx,dy);
};
function reset(){load(cur);}
setInterval(()=>fetch('/ping'),1000);
load(0);
</script></body></html>)rawhtml");
}

// ── System Functions ──────────────────────────────────────────────────────────

void startAPMode() {
  apMode = true;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("battle-setup");
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
    server.on("/lidar/speed", handleLidarSpeed);
    server.on("/ping", handlePing);
    server.begin();
    MDNS.begin("battle");
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
              // Cap all returns at 5 m. Treating front and rear symmetrically avoids the
              // hard discontinuity at the 90°/270° boundary that produced a black wedge
              // when the rear was force-clamped to 5 m while the front used live values.
              if (dist > 5000) dist = 5000;
              lidarBuf[lidarHead] = { ang, dist, inten };
              lidarHead = (lidarHead + 1) % LIDAR_BUF_POINTS;
              lidarSeq++;
            }
            portEXIT_CRITICAL(&lidarMux);
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
    if (now - lastDraw >= 1000) { lastDraw = now; updateDisplay(); }
    vTaskDelay(pdMS_TO_TICKS(200));
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
  
  setupWiFi();
  // LD19P external-speed-control trigger: hold 500–1500 Hz @ 50% duty for >100 ms.
  // Once entered, the LD19P stays in external mode until power-cycle.
  ledcAttach(LIDAR_PWM_PIN, LIDAR_PWM_FREQ, LIDAR_PWM_RES);
  lidarPwmDuty = 128; lidarMotorOn = true; lidarPwmInvert = false;
  applyLidarPwm();
  delay(200);
  // After external mode is latched, pin duty to 90% for normal operation.
  lidarPwmDuty = 230;
  applyLidarPwm();
  xTaskCreate([](void*){ for(;;){ server.handleClient(); vTaskDelay(1); } }, "web", 8192, NULL, 1, NULL);
  xTaskCreate(lidarTask, "lidar", 4096, NULL, 2, NULL);
  xTaskCreate(lidarControlTask, "lctrl", 2048, NULL, 1, NULL);
  xTaskCreate(personTask, "person", 4096, NULL, 1, NULL);
  xTaskCreate(displayTask, "disp", 4096, NULL, 1, NULL);
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
    } else if (cmd == "help") {
      Serial.println("Available commands:");
      Serial.println("  clear  - Wipe WiFi credentials and reboot to AP mode");
      Serial.println("  reboot - Restart the ESP32");
    }
  }
  delay(100);
}
