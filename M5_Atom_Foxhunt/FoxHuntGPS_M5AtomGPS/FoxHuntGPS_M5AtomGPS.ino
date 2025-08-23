// M5 ATOM FOXHUNT + GPS SD Logger
// - FoxHunt passive BLE/WiFi detector with RSSI-driven LED
// - Requires: M5AtomGPS

#include <M5Atom.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <WiFi.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <NimBLEAdvertisedDevice.h>
#include <esp_wifi.h>
#include <FastLED.h>

// ================================
// Hardware - M5 Atom Lite + Atomic GPS Base + TF (SD)
// ================================
#define LED_PIN 27
#undef NUM_LEDS
#define ATOM_NUM_LEDS 1
#define LED_BRIGHTNESS 80
CRGB leds[ATOM_NUM_LEDS];

// Wi-Fi scan state
int wifiChannels[14] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };  // configure for your region
int wifiChanCount = 11;
int timePerChannel[14] = { 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 50, 50, 50 };
bool adaptiveScan = true;
unsigned long lastWifiScanTick = 0;
unsigned long wifiScanPeriodMs = 300;  // scan one channel every 300 ms
int wifiChanIdx = 0;

// SD pins
static const int SD_CS = 15;  // CS
// SPI SCK=23, MISO=33, MOSI=19 via SPI.begin(23,33,19,-1)

// GPS
TinyGPSPlus gps;
// Serial1.begin(9600, SERIAL_8N1, 22, -1); // RX=22, TX unused
static const uint32_t GPS_BAUD = 9600;

// ================================
// Network Config Portal
// ================================
const char* AP_SSID = "snoopuntothem";
const char* AP_PASSWORD = "astheysnoopuntous";
const unsigned long CONFIG_TIMEOUT = 20000;

enum OperatingMode { CONFIG_MODE,
                     TRACKING_MODE };
OperatingMode currentMode = CONFIG_MODE;

AsyncWebServer server(80);
Preferences preferences;

// ================================
// BLE
// ================================
NimBLEScan* pBLEScan;
String targetMAC = "";
unsigned long lastTargetSeen = 0;
bool targetDetected = false;
bool firstDetection = true;
volatile bool newTargetDetected = false;
int currentRSSI = -100;

// ================================
// LED Proximity Blink
// ================================
unsigned long lastBlinkTime = 0;

int calculateBlinkInterval(int rssi) {
  if (rssi >= -40) return map(rssi, -40, -30, 200, 50);
  else if (rssi >= -50) return map(rssi, -50, -40, 400, 200);
  else if (rssi >= -60) return map(rssi, -60, -50, 800, 400);
  else if (rssi >= -70) return map(rssi, -70, -60, 1500, 800);
  else if (rssi >= -80) return map(rssi, -80, -70, 2500, 1500);
  return 3000;
}

void tripleBlink() {
  for (int i = 0; i < 3; i++) {
    leds[0] = CRGB::Green;
    FastLED.show();
    delay(100);
    leds[0] = CRGB::Black;
    FastLED.show();
    if (i < 2) delay(100);
  }
}

void readySignal() {
  for (int b = 0; b <= 255; b += 5) {
    leds[0] = CRGB(0, 0, b);
    FastLED.show();
    delay(10);
  }
  for (int b = 255; b >= 0; b -= 5) {
    leds[0] = CRGB(0, 0, b);
    FastLED.show();
    delay(10);
  }
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(300);
}

void proximityBlink() {
  leds[0] = CRGB::Orange;
  FastLED.show();
  delay(80);
  leds[0] = CRGB::Black;
  FastLED.show();
}

void handleProximityBlinking() {
  unsigned long now = millis();
  int interval = calculateBlinkInterval(currentRSSI);
  if (now - lastBlinkTime >= interval) {
    proximityBlink();
    lastBlinkTime = now;
  }
}

String normalizeMAC(String mac) {
  mac.trim();
  mac.replace("-", ":");
  mac.toUpperCase();
  return mac;
}

// ================================
// SD Logging (Wigler-style CSV)
// ================================
char fileName[64];
bool sdReady = false;

// CSV header same family style as Wigle header then columns
void initializeFile() {
  int fileNumber = 0;
  bool isNewFile = false;
  char dateStamp[16];

  int y = gps.date.year();
  int m = gps.date.month();
  int d = gps.date.day();
  if (y == 0) {  // fallback if date not populated yet
    y = 1970;
    m = 1;
    d = 1;
  }
  sprintf(dateStamp, "%04d-%02d-%02d-", y, m, d);

  do {
    snprintf(fileName, sizeof(fileName), "/FoxHunt-%s%d.csv", dateStamp, fileNumber);
    isNewFile = !SD.exists(fileName);
    fileNumber++;
  } while (!isNewFile);

  File dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println("FoxHunt-1.0,device=M5ATOMGPS,board=ESP32,brand=M5");
    dataFile.println("WhenUTC,TargetMAC,Address,Name,RSSI,Lat,Lon,AltM,HDOP,Type");
    dataFile.close();
    Serial.println(String("New file created: ") + fileName);
  } else {
    Serial.println("Failed to create log file.");
  }
}

void logEncounter(const String& addr, const String& name, int rssi) {
  if (!sdReady) return;

  char utc[21];
  if (gps.date.isValid() && gps.time.isValid()) {
    sprintf(utc, "%04d-%02d-%02d %02d:%02d:%02d",
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    // if GPS time not yet valid, write placeholder
    sprintf(utc, "1970-01-01 00:00:00");
  }

  double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  double lon = gps.location.isValid() ? gps.location.lng() : 0.0;
  double alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  double hdop = gps.hdop.isValid() ? gps.hdop.hdop() : -1.0;

  char line[384];
  snprintf(
    line, sizeof(line),
    "%s,%s,%s,\"%s\",%d,%.6f,%.6f,%.2f,%.2f,BLE",
    utc,
    targetMAC.c_str(),
    addr.c_str(),
    name.c_str(),
    rssi,
    lat,
    lon,
    alt,
    hdop);

  File dataFile = SD.open(fileName, FILE_APPEND);
  if (dataFile) {
    dataFile.println(line);
    dataFile.close();
  } else {
    Serial.println(String("Error opening ") + fileName);
  }
}

void logWifiRow(int netIdx, int channel) {
  char utc[21];
  if (gps.date.isValid() && gps.time.isValid()) {
    sprintf(utc, "%04d-%02d-%02d %02d:%02d:%02d",
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    sprintf(utc, "1970-01-01 00:00:00");
  }

  double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  double lon = gps.location.isValid() ? gps.location.lng() : 0.0;
  double alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  double hdop = gps.hdop.isValid() ? gps.hdop.hdop() : -1.0;

  String bssid = WiFi.BSSIDstr(netIdx);
  String ssid = WiFi.SSID(netIdx);
  int rssi = WiFi.RSSI(netIdx);

  char line[512];
  snprintf(line, sizeof(line),
           "%s,%s,%s,\"%s\",%d,%.6f,%.6f,%.2f,%.2f,WIFI",
           utc,
           targetMAC.c_str(),  // keep TargetMAC column for uniformity (can be empty or same)
           bssid.c_str(),
           ssid.c_str(),
           rssi,
           lat, lon, alt, hdop);

  File f = SD.open(fileName, FILE_APPEND);
  if (f) {
    f.println(line);
    f.close();
  }
}

// ================================
// GPS Helpers
// ================================
bool buttonLedState = true;
bool ledState = false;

void blinkLEDFaster(int numSatellites) {
  unsigned long interval;
  if (numSatellites <= 1) interval = 1000;
  else interval = max(50UL, 1000UL / (unsigned long)numSatellites);

  static unsigned long previousBlinkMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousBlinkMillis >= interval) {
    ledState = !ledState;
    M5.dis.drawpix(0, ledState ? 0x800080 : 0x000000);  // PURPLE/Off
    previousBlinkMillis = currentMillis;
  }
}

void waitForGPSFix() {
  Serial.println("Waiting for GPS fix...");
  unsigned long lastSerialFeed = millis();
  while (!gps.location.isValid()) {
    if (Serial1.available() > 0) gps.encode(Serial1.read());
    int sats = gps.satellites.value();
    blinkLEDFaster(sats);

    // Also periodically print satellites
    if (millis() - lastSerialFeed > 1000) {
      Serial.printf("Satellites: %d\n", sats);
      lastSerialFeed = millis();
    }
    M5.update();
    if (M5.Btn.wasPressed()) {
      // allow abort if needed
      break;
    }
  }
  M5.dis.clear();
  Serial.println("GPS fix obtained or aborted.");
}

// ================================
// Config Storage
// ================================
void saveConfiguration() {
  preferences.begin("tracker", false);
  preferences.putString("targetMAC", normalizeMAC(targetMAC));
  preferences.end();
  Serial.println("Configuration saved");
}

void loadConfiguration() {
  preferences.begin("tracker", true);
  targetMAC = preferences.getString("targetMAC", "");
  preferences.end();
  targetMAC = normalizeMAC(targetMAC);
  if (targetMAC.length() == 17) {
    Serial.println("Loaded targetMAC: " + targetMAC);
  }
}

// ================================
// HTML UI
// ================================
String generateConfigHTML() {
  String randomMAC = "";
  randomSeed(analogRead(0) + micros());
  for (int i = 0; i < 6; i++) {
    if (i > 0) randomMAC += ":";
    byte r = random(0, 256);
    if (r < 16) randomMAC += "0";
    randomMAC += String(r, HEX);
  }
  randomMAC.toLowerCase();

  String html;
  html.reserve(4000);  // avoid reallocs

  html += R"html(<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>M5 AtomGPS FOXHUNT</title>
<style>
body{font-family:Segoe UI,Tahoma,Arial;background:#1a1a2e;color:#fff;margin:0;padding:20px}
.container{max-width:700px;margin:0 auto;background:rgba(255,255,255,.05);padding:24px;border-radius:12px}
h1{text-align:center}
textarea{width:100%;min-height:100px;padding:12px;border-radius:8px;border:1px solid #3a3a3a;background:rgba(255,255,255,.05);color:#fff}
button{padding:12px 16px;border:0;border-radius:8px;cursor:pointer;margin:8px}
.primary{background:linear-gradient(135deg,#ff6b6b,#4ecdc4);color:#fff}
.danger{background:linear-gradient(135deg,#8b0000,#dc143c);color:#fff}
.info{background:linear-gradient(135deg,#182848,#4b6cb7);color:#fff}
</style></head><body><div class="container">
<h1>M5 ATOM FOXHUNT</h1>
<p>Enter a BLE/WiFi target MAC. LED blinks faster as RSSI increases. Encounters are logged to SD with GPS.</p>
<form method="POST" action="/save">
<label>Target MAC</label>
<textarea name="targetMAC" placeholder="XX:XX:XX:XX:XX:XX (e.g. )html)html";

  // Insert randomMAC into placeholder example
  html += randomMAC;

  html += R"html(">)html)html";

  // Insert current target value inside the textarea
  html += targetMAC;

  html += R"html(</textarea>
<div style="margin-top:16px">
  <button class="primary" type="submit">Save & Start</button>
  <button class="danger" type="button" onclick="fetch('/clear',{method:'POST'}).then(()=>location.reload())">Clear</button>
  <button class="info" type="button" onclick="fetch('/device-reset',{method:'POST'})">Device Reset</button>
</div>
</form>
</div></body></html>)html";

  return html;
}

// ================================
// Web Server
// ================================
unsigned long configStartTime = 0;
unsigned long lastConfigActivity = 0;
unsigned long modeSwitchScheduled = 0;
unsigned long deviceResetScheduled = 0;

void startConfigMode() {
  currentMode = CONFIG_MODE;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  delay(500);

  configStartTime = millis();
  lastConfigActivity = millis();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    lastConfigActivity = millis();
    request->send(200, "text/html", generateConfigHTML());
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest* request) {
    lastConfigActivity = millis();
    if (request->hasParam("targetMAC", true)) {
      targetMAC = normalizeMAC(request->getParam("targetMAC", true)->value());
      saveConfiguration();
      request->send(200, "text/plain", "Saved. Switching to tracking in 3s.");
      modeSwitchScheduled = millis() + 3000;
    } else {
      request->send(400, "text/plain", "Missing target MAC");
    }
  });

  server.on("/clear", HTTP_POST, [](AsyncWebServerRequest* request) {
    lastConfigActivity = millis();
    targetMAC = "";
    saveConfiguration();
    request->send(200, "text/plain", "Cleared");
  });

  server.on("/device-reset", HTTP_POST, [](AsyncWebServerRequest* request) {
    lastConfigActivity = millis();
    request->send(200, "text/plain", "Resetting...");
    deviceResetScheduled = millis() + 1000;
  });

  server.begin();

  Serial.println("\n=== CONFIG MODE ===");
  Serial.print("AP: ");
  Serial.println(AP_SSID);
  Serial.print("Portal: http://");
  Serial.println(WiFi.softAPIP());
}

class MyScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (currentMode != TRACKING_MODE) return;

    String addr = dev->getAddress().toString().c_str();
    addr.toUpperCase();

    if (addr == targetMAC) {
      currentRSSI = dev->getRSSI();
      lastTargetSeen = millis();
      targetDetected = true;
      if (firstDetection) {
        newTargetDetected = true;
      }

      // BLE name if present
      std::string n = dev->getName();
      String name = n.c_str();

      // Log encounter immediately (debounce: not too spammy)
      static unsigned long lastLog = 0;
      if (millis() - lastLog > 1500) {
        logEncounter(addr, name, currentRSSI);
        lastLog = millis();
      }
    }
  }
  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    Serial.printf("Scan end reason=%d, count=%d\n", reason, results.getCount());
  }
};

void startTrackingMode() {
  if (targetMAC.length() != 17) {
    Serial.println("No valid target set, staying in config mode.");
    return;
  }

  currentMode = TRACKING_MODE;
  server.end();
  WiFi.mode(WIFI_OFF);

  Serial.println("\n=== TRACKING MODE ===");
  Serial.println("Target: " + targetMAC);

  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setScanCallbacks(new MyScanCallbacks(), false);
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->setMaxResults(0);
  pBLEScan->setDuplicateFilter(false);
  pBLEScan->start(0, false, true);

  Serial.println("Passive BLE scanning started.");
  readySignal();
}

// ================================
// Setup
// ================================
void setup() {
  Serial.begin(115200);
  delay(200);
  M5.begin(true, false, true);
  M5.dis.clear();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // no association; we only scan
  delay(50);

  // LED
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, ATOM_NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  leds[0] = CRGB::White;
  FastLED.show();
  delay(300);
  leds[0] = CRGB::Black;
  FastLED.show();

  // SPI + SD
  SPI.begin(23, 33, 19, -1);
  delay(200);
  while (!SD.begin(SD_CS, SPI, 40000000)) {
    Serial.println("SD init failed, retrying...");
    M5.dis.drawpix(0, 0xff0000);  // RED
    delay(1000);
  }
  M5.dis.clear();
  sdReady = true;
  Serial.println("SD ready.");

  // GPS serial
  Serial1.begin(GPS_BAUD, SERIAL_8N1, 22, -1);
  delay(500);
  Serial.println("GPS serial ready.");

  // Randomize WiFi MAC
  uint8_t newMAC[6];
  randomSeed(analogRead(0) + micros());
  for (int i = 0; i < 6; i++) newMAC[i] = random(0, 256);
  newMAC[0] |= 0x02;  // locally administered
  newMAC[0] &= 0xFE;  // unicast
  esp_wifi_set_mac(WIFI_IF_STA, newMAC);
  esp_wifi_set_mac(WIFI_IF_AP, newMAC);

  // Load target
  loadConfiguration();

  // Get GPS first, then create log file (enables proper UTC date in filename)
  waitForGPSFix();
  if (sdReady) initializeFile();

  // Start AP config
  startConfigMode();
}

// ================================
// Loop
// ================================
void loop() {
  // Feed GPS parser continuously
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // Button toggles small green heartbeat while GPS valid
  M5.update();
  if (M5.Btn.wasPressed()) {
    buttonLedState = !buttonLedState;
    delay(50);
  }

  // Small heartbeat if GPS has valid location in any mode
  static unsigned long lastBeat = 0;
  if (gps.location.isValid() && buttonLedState) {
    unsigned long now = millis();
    if (now - lastBeat >= 2500) {
      M5.dis.drawpix(0, 0x00ff00);  // GREEN
      delay(60);
      M5.dis.clear();
      lastBeat = now;
    }
  } else if (!gps.location.isValid()) {
    // blink purple while GPS invalid (background)
    blinkLEDFaster(gps.satellites.value());
  }

  unsigned long now = millis();

  // Scheduled transitions
  if (modeSwitchScheduled > 0 && now >= modeSwitchScheduled) {
    modeSwitchScheduled = 0;
    startTrackingMode();
    return;
  }

  if (deviceResetScheduled > 0 && now >= deviceResetScheduled) {
    deviceResetScheduled = 0;
    preferences.begin("tracker", false);
    preferences.clear();
    preferences.end();
    delay(500);
    ESP.restart();
  }

  if (currentMode == CONFIG_MODE) {
    // Timeout to tracking if idle and have a target
    int clients = WiFi.softAPgetStationNum();
    if (now - lastConfigActivity > CONFIG_TIMEOUT && clients == 0 && targetMAC.length() == 17) {
      Serial.println("Config timeout -> tracking");
      startTrackingMode();
    }
  } else if (currentMode == TRACKING_MODE) {
    // Non-blocking Wi-Fi scan step: one channel per period
    unsigned long now = millis();
    if (now - lastWifiScanTick >= wifiScanPeriodMs) {
      lastWifiScanTick = now;

      int ch = wifiChannels[wifiChanIdx];
      int dwell = timePerChannel[ch - 1];
      // Perform a single-channel, active scan with short dwell
      // scanNetworks(async=false, show_hidden=true, passive=false, max_ms_per_chan, channel)
      int found = WiFi.scanNetworks(false, true, false, dwell, ch);

      for (int i = 0; i < found; i++) {
        logWifiRow(i, ch);
      }

      // Adaptive tuning (optional)
      if (adaptiveScan) {
        const int FEW = 1, MANY = 7, INC = 50, MAXT = 500, MINT = 50;
        if (found >= MANY) timePerChannel[ch - 1] = min(timePerChannel[ch - 1] + INC, MAXT);
        else if (found <= FEW) timePerChannel[ch - 1] = max(timePerChannel[ch - 1] - INC, MINT);
      }

      WiFi.scanDelete();  // free results

      wifiChanIdx = (wifiChanIdx + 1) % wifiChanCount;
    }
    
    // Handle new detection
    if (newTargetDetected) {
      newTargetDetected = false;
      if (firstDetection) {
        tripleBlink();
        firstDetection = false;
        Serial.println("TARGET ACQUIRED: " + targetMAC + " RSSI: " + String(currentRSSI));
      }
    }

    // Proximity blink while recently seen
    if (targetDetected && (now - lastTargetSeen < 5000)) {
      handleProximityBlinking();
    } else if (now - lastTargetSeen >= 5000) {
      targetDetected = false;
      firstDetection = true;
      leds[0] = CRGB::Black;
      FastLED.show();
    }
  }

  delay(5);
}