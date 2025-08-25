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
#include <esp_task_wdt.h>
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

// LED Pattern States
enum LEDPattern {
  LED_OFF,
  LED_STATUS_BLINK,    // Background heartbeat
  LED_PROXIMITY_BLINK, // Variable speed based on RSSI
  LED_SINGLE_BLINK,    // Red x1
  LED_TRIPLE_BLINK,    // Green x3
  LED_READY_SIGNAL,    // Blue fade
  LED_GPS_WAIT         // Purple variable brightness
};

// LED control 
volatile LEDPattern currentLEDPattern = LED_OFF;
volatile int ledPatternParam = 0;  // For RSSI value, GPS satellites count, etc.

// Wi-Fi scan state
int wifiChannels[14] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };  // configure for your region
int wifiChanCount = 11;
int timePerChannel[14] = { 300, 100, 100, 100, 100, 300, 100, 100, 100, 100, 300, 50, 50, 50 };
bool adaptiveScan = true;
unsigned long lastWifiScanTick = 0;
unsigned long wifiScanPeriodMs = 110;  // scan one channel every 110ms 
int wifiChanIdx = 0;

// SD pins
static const int SD_CS = 15;  // CS
// SPI SCK=23, MISO=33, MOSI=19 via SPI.begin(23,33,19,-1)

// GPS
TinyGPSPlus gps;
// Serial1.begin(9600, SERIAL_8N1, 22, -1); // RX=22, TX unused
static const uint32_t GPS_BAUD = 9600;

// ================================
// FreeRTOS Task Handles
// ================================
TaskHandle_t bleTaskHandle = NULL;
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t gpsTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;

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
// LED Task Function
// ================================
void ledTask(void* parameter) {
  Serial.println("LED control task running on Core " + String(xPortGetCoreID()));
  
  unsigned long patternStartTime = 0;
  bool patternActive = false;
  unsigned long previousMillis = 0;
  int brightness = 0;
  bool ascending = true;
  
  Serial.printf("Initial LED pattern: %d\n", currentLEDPattern);
  
  while (true) {
    unsigned long currentMillis = millis();
    
    static unsigned long lastPatternLog = 0;
    
    switch (currentLEDPattern) {
      case LED_OFF:
        leds[0] = CRGB::Black;
        FastLED.show();
        patternActive = false;
        break;
        
      case LED_STATUS_BLINK:
        if (currentMillis - previousMillis >= 1000) {
          leds[0] = CRGB::Green;
          FastLED.show();
          vTaskDelay(60 / portTICK_PERIOD_MS);
          leds[0] = CRGB::Black;
          FastLED.show();
          previousMillis = currentMillis;
        }
        break;
        
      case LED_PROXIMITY_BLINK:
        {
          // Calculate blink interval based on RSSI (stored in ledPatternParam)
          int rssi = ledPatternParam;
          int interval = calculateBlinkInterval(rssi);
          
          if (currentMillis - previousMillis >= interval) {
            leds[0] = CRGB::Orange;
            FastLED.show();
            vTaskDelay(80 / portTICK_PERIOD_MS);
            leds[0] = CRGB::Black;
            FastLED.show();
            previousMillis = currentMillis;
          }
        }
        break;
        
      case LED_SINGLE_BLINK: 
        if (!patternActive) {
          patternStartTime = currentMillis;
          patternActive = true;
          Serial.println("Single blink started");
        }
        {
          unsigned long elapsed = currentMillis - patternStartTime;
          if (elapsed <= 100) {
            leds[0] = CRGB::Red;
            FastLED.show();
          } else if (elapsed <= 200) {
            leds[0] = CRGB::Black;
            FastLED.show();
          } else {
            leds[0] = CRGB::Black;
            FastLED.show();
            currentLEDPattern = LED_STATUS_BLINK; // Return to status blinking after single blink
            patternActive = false;
            Serial.println("Single blink completed, returning to status");
          }
        }
        break;
        
      case LED_TRIPLE_BLINK:
        {
          if (!patternActive) {
            patternStartTime = currentMillis;
            patternActive = true;
            Serial.println("Triple blink started");
          }
          
          unsigned long elapsed = currentMillis - patternStartTime;
          
          if (elapsed < 100) {
            leds[0] = CRGB::Green;
            FastLED.show();
          } else if (elapsed < 200) {
            leds[0] = CRGB::Black;
            FastLED.show();
          } else if (elapsed < 300) {
            leds[0] = CRGB::Green;
            FastLED.show();
          } else if (elapsed < 400) {
            leds[0] = CRGB::Black;
            FastLED.show();
          } else if (elapsed < 500) {
            leds[0] = CRGB::Green;
            FastLED.show();
          } else if (elapsed < 600) {
            leds[0] = CRGB::Black;
            FastLED.show();
          } else {
            leds[0] = CRGB::Black;
            FastLED.show();
            currentLEDPattern = LED_STATUS_BLINK; // Return to status blinking after triple blink
            patternActive = false;
            Serial.println("Triple blink completed, returning to status");
          }
        }
        break;
        
      case LED_READY_SIGNAL: // Blue fade
        {
          if (!patternActive) {
            patternStartTime = currentMillis;
            patternActive = true;
            Serial.println("Ready signal started");
          }
          
          unsigned long elapsed = currentMillis - patternStartTime;
          if (elapsed > 2000) { // 2 second fade
            leds[0] = CRGB::Black;
            FastLED.show();
            // Important: Transition to status blink, not OFF
            currentLEDPattern = LED_STATUS_BLINK;
            patternActive = false;
            Serial.println("Ready signal completed, switching to status blink");
          } else {
            // Blue fade up and down
            int brightness = (elapsed < 1000) ? (elapsed * 255 / 1000) : (255 - ((elapsed - 1000) * 255 / 1000));
            leds[0] = CRGB(0, 0, brightness);
            FastLED.show();
          }
        }
        break;
        
      case LED_GPS_WAIT:
        {
          // Get current satellite count
          int numSat = ledPatternParam;
          
          // Calculate appropriate interval based on satellite count
          // Faster pulse with more satellites
          unsigned long interval = (numSat <= 1) ? 50UL : max(20UL, 300UL / (unsigned long)numSat);
          
          if (currentMillis - previousMillis >= interval) {
            // Update brightness in correct direction
            if (ascending) {
              brightness += 5;
              if (brightness >= 255) {
                brightness = 255;
                ascending = false;
              }
            } else {
              brightness -= 5;
              if (brightness <= 0) {
                brightness = 0;
                ascending = true;
              }
            }
            
            // Update LED
            leds[0] = CRGB(brightness, 0, brightness);  // Purple
            FastLED.show();
            previousMillis = currentMillis;
            if (currentMillis % 5000 == 0) {
              Serial.printf("GPS Wait - Sats: %d\n", numSat);
            }
          }
        }
        break;
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// LED Helper Functions
int calculateBlinkInterval(int rssi) {
  if (rssi >= -40) return map(rssi, -40, -30, 200, 50);
  else if (rssi >= -50) return map(rssi, -50, -40, 400, 200);
  else if (rssi >= -60) return map(rssi, -60, -50, 800, 400);
  else if (rssi >= -70) return map(rssi, -70, -60, 1500, 800);
  else if (rssi >= -80) return map(rssi, -80, -70, 2500, 1500);
  return 3000;
}

void setLEDPattern(LEDPattern newPattern, int param = 0) {
  LEDPattern oldPattern = currentLEDPattern;
  currentLEDPattern = newPattern;
  ledPatternParam = param;
  delay(5); // cool it down
}

void triggerTripleBlink() {
  setLEDPattern(LED_TRIPLE_BLINK);
}

void triggerReadySignal() {
  setLEDPattern(LED_READY_SIGNAL);
}

void startGPSWaitPattern(int numSatellites) {
  setLEDPattern(LED_GPS_WAIT, numSatellites);
}

void stopGPSWaitPattern() {
  setLEDPattern(LED_STATUS_BLINK);
}

void startStatusBlinking() {
  setLEDPattern(LED_STATUS_BLINK);
}

void startProximityBlinking(int rssi) {
  setLEDPattern(LED_PROXIMITY_BLINK, rssi);
}

void stopProximityBlinking() {
  startStatusBlinking();
  Serial.println("Proximity blinking stopped, switching to status blink");
}

String normalizeMAC(String mac) {
  mac.trim();
  mac.replace("-", ":");
  mac.toUpperCase();
  return mac;
}

// ================================
// SD Logging
// ================================
char fileName[64];
bool sdReady = false;

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

void waitForGPSFix() {
  Serial.println("Waiting for GPS fix...");
  unsigned long lastSerialFeed = millis();
  while (!gps.location.isValid()) {
    if (Serial1.available() > 0) gps.encode(Serial1.read());
    int sats = gps.satellites.value();
    startGPSWaitPattern(sats);

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
  stopGPSWaitPattern();
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
<p>Enter a BLE/WiFi target MAC. LED blinks faster as RSSI increases.</p>
<form method="POST" action="/save">
<label>Target MAC</label>
<textarea name="targetMAC" placeholder="XX:XX:XX:XX:XX:XX";

  // Insert randomMAC into placeholder example
  html += randomMAC;

  html += R"html(">)html";

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

// ================================
// Task Functions
// ================================
void gpsTask(void* parameter) {
  Serial.println("GPS processing task running on Core " + String(xPortGetCoreID()));

  while (true) {
    // Feed GPS parser continuously
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ================================
// BLE Task Function
// ================================
void bleTask(void* parameter) {
  Serial.println("BLE scanning task running on Core " + String(xPortGetCoreID()));
  
  while(true) {
    if (currentMode == TRACKING_MODE) {
      // BLE scanning is mostly handled by the NimBLE library
      // This task just monitors and restarts if needed
      
      static unsigned long lastRestartCheck = 0;
      if (millis() - lastRestartCheck > 30000) {  // Every 30 seconds
        if (pBLEScan) {
          pBLEScan->stop();
          vTaskDelay(50 / portTICK_PERIOD_MS);
          pBLEScan->start(0, false, true);
          Serial.println("BLE scan refreshed");
        }
        lastRestartCheck = millis();
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void wifiTask(void* parameter) {
  Serial.println("WiFi scanning task running on Core " + String(xPortGetCoreID()));

  while (true) {
    if (currentMode == TRACKING_MODE) {
      unsigned long now = millis();

      // Non-blocking Wi-Fi scan step: one channel per period
      if (now - lastWifiScanTick >= wifiScanPeriodMs) {
        lastWifiScanTick = now;

        int ch = wifiChannels[wifiChanIdx];
        int dwell = timePerChannel[ch - 1];
        // Perform a single-channel, active scan with short dwell
        // scanNetworks(async=false, show_hidden=true, passive=false, max_ms_per_chan, channel)
        int found = WiFi.scanNetworks(false, true, false, dwell, ch);

        for (int i = 0; i < found; i++) {
          String bssid = WiFi.BSSIDstr(i);
          if (bssid == targetMAC) {
            currentRSSI = WiFi.RSSI(i);
            lastTargetSeen = millis();
            targetDetected = true;
            newTargetDetected = true;
          }

          // Queue for logging in main thread or use mutex... TODO
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
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
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
  WiFi.mode(WIFI_STA);  // Set to station mode for scanning
  WiFi.disconnect();    // Disconnect from any networks
  delay(500);

  Serial.println("\n=== TRACKING MODE ===");
  Serial.println("Target: " + targetMAC);

  // Initialize BLE scanning
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setScanCallbacks(new MyScanCallbacks(), false);
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->setMaxResults(0);
  pBLEScan->setDuplicateFilter(false);

  // Create tasks for scanning
  xTaskCreatePinnedToCore(gpsTask, "GPSTask", 4096, NULL, 2, &gpsTaskHandle, 1);
  xTaskCreatePinnedToCore(bleTask, "BLETask", 4096, NULL, 1, &bleTaskHandle, 1);
  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 4096, NULL, 1, &wifiTaskHandle, 0);

  // Start BLE scanning
  pBLEScan->start(0, false, true);
  Serial.println("BLE scanning started!");
  Serial.println("WiFi scanning started!");
  Serial.println("Dual-core scanning system active");

  // Set the LED patterns with small delay between them
  startStatusBlinking();
  delay(100);
  triggerReadySignal();
  
  Serial.printf("LED Pattern set to: %d\n", currentLEDPattern);
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

  // Create LED task on Core 0
  xTaskCreatePinnedToCore(
    ledTask,         // Task function
    "LEDTask",       // Name of task
    2048,            // Stack size (smaller than other tasks)
    NULL,            // Parameter
    1,               // Priority
    &ledTaskHandle,  // Task handle
    0                // Core 0 (protocol core)
  );

  // SPI + SD
  SPI.begin(23, 33, 19, -1);
  delay(200);
  while (!SD.begin(SD_CS, SPI, 40000000)) {
    Serial.println("SD init failed, retrying...");
    leds[0] = CRGB::Red;
    FastLED.show();
    delay(1000);
    leds[0] = CRGB::Black;
    FastLED.show();
  }
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
  M5.update();
  if (M5.Btn.wasPressed()) {
    buttonLedState = !buttonLedState;
    delay(50);
  }

  // Scheduled transitions
  unsigned long now = millis();
  
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
    // Handle new detection
    if (newTargetDetected) {
      newTargetDetected = false;
      if (firstDetection) {
        triggerTripleBlink();
        firstDetection = false;
        Serial.println("TARGET ACQUIRED: " + targetMAC + " RSSI: " + String(currentRSSI));
      }
    }

    // Update proximity blinking (via LED task)
    if (targetDetected && (now - lastTargetSeen < 5000)) {
      startProximityBlinking(currentRSSI);
    } else if (now - lastTargetSeen >= 5000 && currentLEDPattern != LED_STATUS_BLINK) {
      targetDetected = false;
      firstDetection = true;
      startStatusBlinking();
      Serial.println("Target lost, returning to status blink");
    }
  }

  delay(5);  // Short delay to yield to other tasks
}