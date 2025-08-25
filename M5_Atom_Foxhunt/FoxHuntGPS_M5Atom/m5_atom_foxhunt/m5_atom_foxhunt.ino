#include <WiFi.h>
#include <AsyncTCP.h>           // ESP32Async v3.8.0
#include <ESPAsyncWebServer.h>  // ESP32Async v3.8.0
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <NimBLEAdvertisedDevice.h>
#include <esp_wifi.h>
#include <FastLED.h>
#include <esp_task_wdt.h>

// ================================
// Hardware Configuration - M5 Atom Lite
// ================================
#define LED_PIN 27         // M5 Atom Lite RGB LED pin
#define NUM_LEDS 1         // Single LED on Atom Lite
#define LED_BRIGHTNESS 80  // 0-255, moderate brightness

// LED setup
CRGB leds[NUM_LEDS];

// Network configuration
const char* AP_SSID = "snoopuntothem";
const char* AP_PASSWORD = "astheysnoopuntous";
const unsigned long CONFIG_TIMEOUT = 20000;  // 20 seconds

// Operating modes
enum OperatingMode {
  CONFIG_MODE,
  TRACKING_MODE
};

// LED Pattern States
enum LEDPattern {
  LED_OFF,
  LED_STATUS_BLINK,
  LED_PROXIMITY_BLINK,
  LED_SINGLE_BLINK,
  LED_TRIPLE_BLINK,
  LED_READY_SIGNAL
};

// ================================
// WiFi Scanning Configuration
// ================================
int wifiChannels[14] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };  // configure for your region
int wifiChanCount = 11;
int timePerChannel[14] = { 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 50, 50, 50 };
bool adaptiveScan = true;
unsigned long lastWifiScanTick = 0;
unsigned long wifiScanPeriodMs = 300;  // scan one channel every 300 ms
int wifiChanIdx = 0;

// ================================
// FreeRTOS Task Handles
// ================================
TaskHandle_t bleTaskHandle = NULL;
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;

// Global variables
OperatingMode currentMode = CONFIG_MODE;
AsyncWebServer server(80);  // ESP32Async v3.8.0 works with const methods
Preferences preferences;
NimBLEScan* pBLEScan;

String targetMAC = "";
bool hasTargetOUI = false;
uint8_t targetOUI[3];  // first 3 bytes of configured MAC
unsigned long configStartTime = 0;
unsigned long lastConfigActivity = 0;
unsigned long modeSwitchScheduled = 0;
unsigned long deviceResetScheduled = 0;
unsigned long lastBlinkTime = 0;
bool targetDetected = false;
int currentRSSI = -100;
unsigned long lastTargetSeen = 0;
bool firstDetection = true;

// LED pattern control - volatile as accessed from different cores
volatile LEDPattern currentLEDPattern = LED_OFF;
volatile int ledPatternParam = 0;  // For RSSI value, etc.

// LED blink synchronization
volatile bool newTargetDetected = false;

// Helper for matching
String normalizeMAC(String mac) {
  mac.trim();
  mac.replace("-", ":");
  mac.replace(" ", "");
  mac.toUpperCase();
  return mac;
}

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
  bool debugPrint = true;
  
  while (true) {
    unsigned long currentMillis = millis();
    
    // Debug the current pattern occasionally
    if (debugPrint && currentMillis - previousMillis >= 5000) {
      Serial.printf("Current LED pattern: %d, param: %d\n", currentLEDPattern, ledPatternParam);
      debugPrint = true;
    }
    
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
          debugPrint = true;
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
            debugPrint = true;
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
            // Important: return to status blinking, not OFF
            currentLEDPattern = LED_STATUS_BLINK;
            patternActive = false;
            Serial.println("Triple blink completed, returning to status");
          }
        }
        break;
        
      case LED_READY_SIGNAL:
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
            // Important: transition to status blinking, not OFF
            currentLEDPattern = LED_STATUS_BLINK;
            patternActive = false;
            Serial.println("Ready signal completed, returning to status");
          } else {
            // Blue fade up and down
            int brightness = (elapsed < 1000) ? (elapsed * 255 / 1000) : (255 - ((elapsed - 1000) * 255 / 1000));
            leds[0] = CRGB(0, 0, brightness);
            FastLED.show();
          }
        }
        break;
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Protected pattern setter with logging
void setLEDPattern(LEDPattern newPattern, int param = 0) {
  LEDPattern oldPattern = currentLEDPattern;
  currentLEDPattern = newPattern;
  ledPatternParam = param;
}

// ================================
// RSSI-based LED Blink Patterns
// ================================

// Calculate blink interval based on RSSI
int calculateBlinkInterval(int rssi) {
  if (rssi >= -40) {
    return map(rssi, -40, -30, 200, 50);  // 200ms to 50ms for very strong signals
  } else if (rssi >= -50) {
    return map(rssi, -50, -40, 400, 200);  // 400ms to 200ms
  } else if (rssi >= -60) {
    return map(rssi, -60, -50, 800, 400);  // 800ms to 400ms
  } else if (rssi >= -70) {
    return map(rssi, -70, -60, 1500, 800);  // 1.5s to 800ms
  } else if (rssi >= -80) {
    return map(rssi, -80, -70, 2500, 1500);  // 2.5s to 1.5s
  } else {
    return 3000;  // 3 seconds for very weak signals
  }
}

// LED pattern functions
void startStatusBlinking() {
  setLEDPattern(LED_STATUS_BLINK);
}

void startProximityBlinking(int rssi) {
  setLEDPattern(LED_PROXIMITY_BLINK, rssi);
}

void stopProximityBlinking() {
  setLEDPattern(LED_STATUS_BLINK);
  Serial.println("Proximity blinking stopped, switching to status blink");
}

void triggerSingleBlink() {
  setLEDPattern(LED_SINGLE_BLINK);
}

void triggerTripleBlink() {
  setLEDPattern(LED_TRIPLE_BLINK);
}

void triggerReadySignal() {
  setLEDPattern(LED_READY_SIGNAL);
}

// ================================
// Configuration Storage Functions
// ================================
void saveConfiguration() {
  preferences.begin("tracker", false);
  preferences.putString("targetMAC", normalizeMAC(targetMAC));
  preferences.end();
  Serial.println("Configuration saved to NVS");
}

void loadConfiguration() {
  preferences.begin("tracker", true);
  targetMAC = preferences.getString("targetMAC", "");
  preferences.end();

  targetMAC = normalizeMAC(targetMAC);
  hasTargetOUI = false;

  if (targetMAC.length() == 17) {
    Serial.println("Configuration loaded from NVS");
    Serial.println("Target MAC: " + targetMAC);
    int b0, b1, b2;
    if (sscanf(targetMAC.substring(0, 8).c_str(), "%02x:%02x:%02x", &b0, &b1, &b2) == 3) {
      targetOUI[0] = (uint8_t)b0;
      targetOUI[1] = (uint8_t)b1;
      targetOUI[2] = (uint8_t)b2;
      hasTargetOUI = true;
    }
  }
}

// ================================
// HTML Configuration Interface
// ================================
String generateConfigHTML() {
  String randomMAC = "";
  randomSeed(analogRead(0) + micros());
  for (int i = 0; i < 6; i++) {
    if (i > 0) randomMAC += ":";
    byte randByte = random(0, 256);
    if (randByte < 16) randomMAC += "0";
    randomMAC += String(randByte, HEX);
  }
  randomMAC.toLowerCase();

  String html = R"html(<!DOCTYPE html>
<html>
<head>
    <title>M5 Atom FOXHUNT</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0; padding: 20px; background: #1a1a2e; color: #ffffff;
        }
        .container {
            max-width: 700px; margin: 0 auto; background: rgba(255, 255, 255, 0.05);
            padding: 40px; border-radius: 16px; backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.1);
        }
        h1 {
            text-align: center; font-size: 36px; font-weight: 700;
            background: linear-gradient(45deg, #ff6b6b, #4ecdc4);
            -webkit-background-clip: text; -webkit-text-fill-color: transparent;
            margin-bottom: 30px;
        }
        .section {
            margin-bottom: 30px; padding: 25px;
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 12px; background: rgba(255, 255, 255, 0.02);
        }
        .section h3 { margin-top: 0; color: #4ecdc4; font-size: 18px; }
        textarea {
            width: 100%; min-height: 120px; padding: 15px;
            border: 1px solid rgba(255, 255, 255, 0.3);
            border-radius: 8px; background: rgba(255, 255, 255, 0.05);
            color: #ffffff; font-family: 'Courier New', monospace;
            font-size: 14px; resize: vertical;
        }
        textarea:focus {
            outline: none; border-color: #4ecdc4;
            box-shadow: 0 0 0 3px rgba(78, 205, 196, 0.3);
        }
        .help-text {
            font-size: 13px; color: #a0a0a0; margin-top: 8px;
            line-height: 1.4;
        }
        button {
            background: linear-gradient(135deg, #ff6b6b 0%, #4ecdc4 100%);
            color: #ffffff; padding: 14px 28px; border: none;
            border-radius: 8px; cursor: pointer; font-size: 16px;
            font-weight: 500; margin: 10px 5px; transition: all 0.3s;
        }
        button:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 25px rgba(255, 107, 107, 0.4);
        }
        .button-container {
            text-align: center; margin-top: 40px;
            padding-top: 30px; border-top: 1px solid #404040;
        }
        .status {
            padding: 15px; border-radius: 8px; margin-bottom: 30px;
            border-left: 4px solid #4ecdc4;
            background: rgba(78, 205, 196, 0.1);
            color: #ffffff; border: 1px solid rgba(78, 205, 196, 0.3);
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>M5 ATOM FOXHUNT</h1>
        
        <div class="status">
            Enter the target MAC address for foxhunt tracking. LED blink speed indicates proximity: RAPID when close, SLOW when far.
        </div>
        
        <form method="POST" action="/save">
            <div class="section">
                <h3>Target MAC Address</h3>
                <textarea name="targetMAC" placeholder="Enter target MAC address:
)html" + randomMAC
                + R"html(">)html" + targetMAC + R"html(</textarea>
                <div class="help-text">
                    Single MAC address for directional tracking.<br>
                    Format: XX:XX:XX:XX:XX:XX (17 characters with colons)<br>
                    LED blinks: 50ms (RAPID) to 3s (SLOW) intervals
                </div>
            </div>
            
            <div class="button-container">
                <button type="submit">Save Configuration & Start Tracking</button>
                <button type="button" onclick="clearConfig()" style="background: linear-gradient(135deg, #8b0000 0%, #dc143c 100%);">Clear Target</button>
                <button type="button" onclick="deviceReset()" style="background: linear-gradient(135deg, #4a0000 0%, #800000 100%); font-size: 12px;">Device Reset</button>
            </div>
        </form>
        
        <script>
        function clearConfig() {
            if (confirm('Clear the target MAC? This action cannot be undone.')) {
                document.querySelector('textarea[name="targetMAC"]').value = '';
                fetch('/clear', { method: 'POST' })
                    .then(() => { alert('Target cleared!'); location.reload(); })
                    .catch(error => alert('Error: ' + error));
            }
        }
        
        function deviceReset() {
            if (confirm('DEVICE RESET: Complete wipe and restart. Are you sure?')) {
                if (confirm('This cannot be undone. Continue?')) {
                    fetch('/device-reset', { method: 'POST' })
                        .then(() => {
                            alert('Device resetting...');
                            setTimeout(() => window.location.href = '/', 5000);
                        });
                }
            }
        }
        </script>
    </div>
</body>
</html>)html";

  return html;
}

// ================================
// Web Server Functions
// ================================
void startConfigMode() {
  currentMode = CONFIG_MODE;
  Serial.println("\n=== STARTING M5 ATOM FOXHUNT CONFIG MODE ===");
  Serial.println("SSID: " + String(AP_SSID));
  Serial.println("Password: " + String(AP_PASSWORD));

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  delay(2000);

  configStartTime = millis();
  lastConfigActivity = millis();

  Serial.println("✓ Access Point created!");
  Serial.println("IP: " + WiFi.softAPIP().toString());
  Serial.println("Portal: http://" + WiFi.softAPIP().toString());
  Serial.println("==============================\n");

  // Web routes - ESP32Async v3.8.0 syntax
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    lastConfigActivity = millis();
    request->send(200, "text/html", generateConfigHTML());
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest* request) {
    lastConfigActivity = millis();

    if (request->hasParam("targetMAC", true)) {
      targetMAC = normalizeMAC(request->getParam("targetMAC", true)->value());
      targetMAC.trim();

      Serial.println("Target MAC received: " + targetMAC);
      saveConfiguration();

      String responseHTML = R"html(<!DOCTYPE html>
<html>
<head>
    <title>Configuration Saved</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { 
            font-family: 'Segoe UI', sans-serif; margin: 0; padding: 20px;
            background: #1a1a2e; color: #ffffff; text-align: center;
        }
        .container { 
            max-width: 600px; margin: 0 auto; background: rgba(255,255,255,0.05);
            padding: 40px; border-radius: 12px; backdrop-filter: blur(10px);
        }
        h1 { color: #4ecdc4; margin-bottom: 30px; }
        .success { 
            background: rgba(78, 205, 196, 0.2); color: #4ecdc4;
            border: 1px solid #4ecdc4; padding: 20px; border-radius: 8px;
            margin: 30px 0;
        }
    </style>
    <script>
        setTimeout(() => {
            document.getElementById('countdown').innerHTML = 'Switching to tracking mode now...';
        }, 5000);
    </script>
</head>
<body>
    <div class="container">
        <h1>Configuration Saved</h1>
        <div class="success">
            <p><strong>Target MAC configured successfully!</strong></p>
            <p id="countdown">Switching to tracking mode in 5 seconds...</p>
        </div>
        <p>M5 Atom will now start tracking your target device.</p>
        <p>Watch the LED - it will blink faster as you get closer!</p>
    </div>
</body>
</html>)html";

      request->send(200, "text/html", responseHTML);
      modeSwitchScheduled = millis() + 5000;
      Serial.println("Mode switch scheduled for 5 seconds");
    } else {
      request->send(400, "text/plain", "Missing target MAC");
    }
  });

  server.on("/clear", HTTP_POST, [](AsyncWebServerRequest* request) {
    lastConfigActivity = millis();
    targetMAC = "";
    saveConfiguration();
    Serial.println("Target MAC cleared");
    request->send(200, "text/plain", "Target cleared");
  });

  server.on("/device-reset", HTTP_POST, [](AsyncWebServerRequest* request) {
    lastConfigActivity = millis();
    request->send(200, "text/plain", "Device reset initiated");
    deviceResetScheduled = millis() + 1000;
  });

  server.begin();
  Serial.println("Web server started!");
  
  // Start status blinking in config mode
  startStatusBlinking();
}

// ================================
// WiFi Scanning Functions
// ================================
void updateTimePerChannel(int channel, int networksFound) {
  const int FEW_NETWORKS_THRESHOLD = 1;
  const int MANY_NETWORKS_THRESHOLD = 7;
  const int TIME_INCREMENT = 50;
  const int MAX_TIME = 500;
  const int MIN_TIME = 50;

  if (networksFound >= MANY_NETWORKS_THRESHOLD) {
    timePerChannel[channel - 1] = min(timePerChannel[channel - 1] + TIME_INCREMENT, MAX_TIME);
  } else if (networksFound <= FEW_NETWORKS_THRESHOLD) {
    timePerChannel[channel - 1] = max(timePerChannel[channel - 1] - TIME_INCREMENT, MIN_TIME);
  }
}

// ================================
// BLE Scan Callback Class (NimBLE 2.x)
// ================================
class MyScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    if (currentMode != TRACKING_MODE) return;

    String deviceMAC = advertisedDevice->getAddress().toString().c_str();
    deviceMAC.toUpperCase();

    if (deviceMAC == targetMAC) {
      currentRSSI = advertisedDevice->getRSSI();
      lastTargetSeen = millis();
      targetDetected = true;
      newTargetDetected = true;
      Serial.println("*** TARGET MATCH FOUND! ***");
      Serial.println("MAC: " + deviceMAC + " | RSSI: " + String(currentRSSI));
    }
  }

  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    Serial.printf("Scan ended, reason: %d, found %d devices\n", reason, results.getCount());
    // Don't modify the LED pattern here
  }
};

void startTrackingMode() {
  if (targetMAC.length() != 17) {
    Serial.println("Invalid or empty target MAC; staying in config mode");
    return;
  }

  currentMode = TRACKING_MODE;
  server.end();

  Serial.println("\n=== STARTING M5 ATOM TRACKING MODE ===");
  Serial.println("Target MAC: " + targetMAC);
  Serial.println("==============================\n");

  // Initialize BLE
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setScanCallbacks(new MyScanCallbacks(), false);

  // PASSIVE SCANNING CONFIGURATION
  pBLEScan->setActiveScan(false);       // *** PASSIVE SCANNING ***
  pBLEScan->setInterval(100);           // 100ms intervals for better coverage
  pBLEScan->setWindow(99);              // 99ms scan window (99% duty cycle)
  pBLEScan->setMaxResults(0);           // Don't store results to save memory
  pBLEScan->setDuplicateFilter(false);  // Allow duplicate detections for RSSI updates

  // Initialize WiFi for scanning
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Create BLE scanning task on Core 1 (application core)
  xTaskCreatePinnedToCore(
    bleTask,         // Task function
    "BLETask",       // Name of task
    4096,            // Stack size of task
    NULL,            // Parameter of the task
    1,               // Priority of the task
    &bleTaskHandle,  // Task handle to track the task
    1                // Core where the task should run (1 = application core)
  );

  // Create WiFi scanning task on Core 0 (protocol core)
  xTaskCreatePinnedToCore(
    wifiTask,         // Task function
    "WiFiTask",       // Name of task
    4096,             // Stack size of task
    NULL,             // Parameter of the task
    1,                // Priority of the task
    &wifiTaskHandle,  // Task handle to track the task
    0                 // Core where the task should run (0 = protocol core)
  );

  // Start continuous passive scanning
  pBLEScan->start(0, false, true);

  Serial.println("BLE PASSIVE tracking started!");
  Serial.println("WiFi scanning started!");
  Serial.println("Dual-core scanning (BLE on Core 1, WiFi on Core 0)");

  // Start with status blinking and then show ready signal
  startStatusBlinking();
  delay(100);  // Small delay to ensure status pattern is set
  triggerReadySignal();
  
  // Debug verification
  Serial.printf("LED Pattern set to: %d\n", currentLEDPattern);
}

// ================================
// Task Functions
// ================================
void bleTask(void* parameter) {
  Serial.println("BLE scanning task running on Core " + String(xPortGetCoreID()));

  while (true) {
    if (currentMode == TRACKING_MODE) {
      // BLE task only needs to monitor when scan completes and restart as needed
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

        // Perform a single-channel scan
        int found = WiFi.scanNetworks(false, true, false, dwell, ch);

        for (int i = 0; i < found; i++) {
          String bssid = WiFi.BSSIDstr(i);
          bssid.toUpperCase();
          
          if (normalizeMAC(bssid) == targetMAC) {
            currentRSSI = WiFi.RSSI(i);
            lastTargetSeen = millis();
            targetDetected = true;
            newTargetDetected = true;
            Serial.println("*** TARGET MATCH FOUND ON WIFI! ***");
            Serial.println("SSID: " + WiFi.SSID(i) + ", RSSI: " + String(currentRSSI) + ", Channel: " + String(ch));
          }
        }

        // Adaptive tuning if enabled
        if (adaptiveScan && found > 0) {
          updateTimePerChannel(ch, found);
        }

        WiFi.scanDelete();  // Free memory
        wifiChanIdx = (wifiChanIdx + 1) % wifiChanCount;
      }

      vTaskDelay(10 / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

// ================================
// Main Setup Function
// ================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== M5 ATOM LITE FOXHUNT ===");
  Serial.println("Initializing...\n");

  // Initialize LED
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  // Startup LED test
  leds[0] = CRGB::White;
  FastLED.show();
  delay(500);
  leds[0] = CRGB::Black;
  FastLED.show();

  // Create LED task on Core 0 (lower priority than WiFi)
  xTaskCreatePinnedToCore(
    ledTask,         // Task function
    "LEDTask",       // Name of task
    2048,            // Stack size
    NULL,            // Parameter
    1,               // Priority
    &ledTaskHandle,  // Task handle
    0                // Core 0 (protocol core)
  );

  // MAC randomization for stealth
  uint8_t newMAC[6];
  WiFi.macAddress(newMAC);
  Serial.print("Original MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02x", newMAC[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  randomSeed(analogRead(0) + micros());
  for (int i = 0; i < 6; i++) {
    newMAC[i] = random(0, 256);
  }
  newMAC[0] |= 0x02;  // Set locally administered bit
  newMAC[0] &= 0xFE;  // Clear multicast bit

  esp_wifi_set_mac(WIFI_IF_STA, newMAC);
  esp_wifi_set_mac(WIFI_IF_AP, newMAC);

  Serial.print("Randomized MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02x", newMAC[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  // Load configuration
  loadConfiguration();

  // Start in configuration mode
  startConfigMode();
}

// ================================
// Main Loop Function
// ================================
void loop() {
  unsigned long currentTime = millis();

  // Handle scheduled mode switch
  if (modeSwitchScheduled > 0 && currentTime >= modeSwitchScheduled) {
    modeSwitchScheduled = 0;
    startTrackingMode();
    return;
  }

  // Handle scheduled device reset
  if (deviceResetScheduled > 0 && currentTime >= deviceResetScheduled) {
    deviceResetScheduled = 0;
    Serial.println("Device reset triggered");

    preferences.begin("tracker", false);
    preferences.clear();
    preferences.end();

    delay(1000);
    ESP.restart();
    return;
  }

  if (currentMode == CONFIG_MODE) {
    // Config mode timeout logic
    int connectedClients = WiFi.softAPgetStationNum();
    if (currentTime - lastConfigActivity > CONFIG_TIMEOUT && connectedClients == 0) {
      if (targetMAC.length() == 17) {
        Serial.println("Configuration timeout - switching to tracking mode");
        startTrackingMode();
      } else {
        Serial.println("No valid target MAC set - remaining in config mode");
      }
    }
  } else if (currentMode == TRACKING_MODE) {
    // Handle target detection
    if (newTargetDetected) {
      newTargetDetected = false;

      if (firstDetection) {
        triggerTripleBlink();
        firstDetection = false;
        Serial.println("TARGET ACQUIRED!");
        Serial.println("MAC: " + targetMAC);
        Serial.println("RSSI: " + String(currentRSSI));
      }
    }

    // Handle proximity blinking
    if (targetDetected && (currentTime - lastTargetSeen < 5000)) {
      startProximityBlinking(currentRSSI);
    } else if (currentTime - lastTargetSeen >= 5000) {
      targetDetected = false;
      firstDetection = true;
      // Don't turn LED off - return to status blinking
      stopProximityBlinking();
    }
  }

  delay(10);
}