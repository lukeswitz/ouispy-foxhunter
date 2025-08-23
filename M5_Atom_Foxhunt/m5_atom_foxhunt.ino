#include <WiFi.h>
#include <AsyncTCP.h>           // ESP32Async v3.8.0
#include <ESPAsyncWebServer.h>  // ESP32Async v3.8.0
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <NimBLEAdvertisedDevice.h>
#include <esp_wifi.h>
#include <FastLED.h>

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

// Global variables
OperatingMode currentMode = CONFIG_MODE;
AsyncWebServer server(80);  // ESP32Async v3.8.0 works with const methods
Preferences preferences;
NimBLEScan* pBLEScan;

String targetMAC = "";
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

// LED blink synchronization
volatile bool newTargetDetected = false;

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
void singleBlink() {
  leds[0] = CRGB::Red;
  FastLED.show();
  delay(150);
  leds[0] = CRGB::Black;
  FastLED.show();
}

void tripleBlink() {
  // Three fast green blinks for new target
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
  // Blue fade in/out to indicate ready state
  for (int brightness = 0; brightness <= 255; brightness += 5) {
    leds[0] = CRGB(0, 0, brightness);
    FastLED.show();
    delay(10);
  }
  for (int brightness = 255; brightness >= 0; brightness -= 5) {
    leds[0] = CRGB(0, 0, brightness);
    FastLED.show();
    delay(10);
  }
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(500);
}

void proximityBlink() {
  // Orange/yellow blink for proximity indication
  leds[0] = CRGB::Orange;
  FastLED.show();
  delay(100);
  leds[0] = CRGB::Black;
  FastLED.show();
}

void handleProximityBlinking() {
  unsigned long currentTime = millis();
  int blinkInterval = calculateBlinkInterval(currentRSSI);

  if (currentTime - lastBlinkTime >= blinkInterval) {
    proximityBlink();
    lastBlinkTime = currentTime;
  }
}

// ================================
// Configuration Storage Functions
// ================================
void saveConfiguration() {
  preferences.begin("tracker", false);
  preferences.putString("targetMAC", targetMAC);
  preferences.end();
  Serial.println("Configuration saved to NVS");
}

void loadConfiguration() {
  preferences.begin("tracker", true);
  targetMAC = preferences.getString("targetMAC", "");
  preferences.end();

  if (targetMAC.length() > 0) {
    Serial.println("Configuration loaded from NVS");
    Serial.println("Target MAC: " + targetMAC);
    int b0, b1, b2;
    sscanf(targetMAC.substring(0, 8).c_str(),
           "%02x:%02x:%02x", &b0, &b1, &b2);
    targetOUI[0] = (uint8_t)b0;
    targetOUI[1] = (uint8_t)b1;
    targetOUI[2] = (uint8_t)b2;
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
      targetMAC = request->getParam("targetMAC", true)->value();
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
}

// ================================
// BLE Scan Callback Class (NimBLE 2.x)
// ================================
class MyScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    if (currentMode != TRACKING_MODE) return;

    String deviceMAC = advertisedDevice->getAddress().toString().c_str();
    deviceMAC.toUpperCase();

    // Serial.printf("LOOKING FOR: %s\n", targetMAC.c_str());
    // Serial.printf("DETECTED: %s (RSSI: %d)\n", deviceMAC.c_str(), advertisedDevice->getRSSI());

    if (deviceMAC == targetMAC || isRandomizedVariant(deviceMAC)) {
      currentRSSI = advertisedDevice->getRSSI();
      lastTargetSeen = millis();
      targetDetected = true;
      newTargetDetected = true;
      Serial.println("*** TARGET MATCH FOUND! ***");
    }
  }

  bool isRandomizedVariant(const String& mac) {
    if (mac.length() != 17) return false;   // quick sanity check

    int b0, b1, b2;
    sscanf(mac.substring(0, 8).c_str(),
           "%02x:%02x:%02x", &b0, &b1, &b2);
    uint8_t oui[3] = { (uint8_t)b0, (uint8_t)b1, (uint8_t)b2 };

    // Same vendor OUI?
    if (memcmp(oui, targetOUI, 3) != 0) return false;

    // Accept either value of the locally-administered bit (bit 1)
    uint8_t targetFirst = targetOUI[0];
    uint8_t observedFirst = oui[0];
    return ((targetFirst & 0xFD) == (observedFirst & 0xFD));
}

  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    Serial.printf("Scan ended, reason: %d, found %d devices\n", reason, results.getCount());
  }
};

void startTrackingMode() {
  if (targetMAC.length() == 0) {
    Serial.println("No target MAC configured, staying in config mode");
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

  // Start continuous passive scanning
  pBLEScan->start(0, false, true);

  Serial.println("BLE PASSIVE tracking started!");
  Serial.println("This will detect ALL BLE advertisements including hidden/random devices");

  // Ready signal
  readySignal();
}

// ================================
// Main Setup Function
// ================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== M5 ATOM LITE FOXHUNT ===");
  Serial.println("Hardware: M5 Atom Lite ESP32");
  Serial.println("LED: GPIO27 (Single RGB LED)");
  Serial.println("Libraries: ESP32Async v3.8.0, NimBLE 2.x");
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
      Serial.println("Configuration timeout - switching to tracking mode");
      startTrackingMode();
    }
  } else if (currentMode == TRACKING_MODE) {
    // Handle target detection
    if (newTargetDetected) {
      newTargetDetected = false;

      if (firstDetection) {
        tripleBlink();
        firstDetection = false;
        Serial.println("TARGET ACQUIRED!");
        Serial.println("MAC: " + targetMAC);
        Serial.println("RSSI: " + String(currentRSSI));
      }
    }

    // Handle proximity blinking
    if (targetDetected && (currentTime - lastTargetSeen < 5000)) {
      handleProximityBlinking();
    } else if (currentTime - lastTargetSeen >= 5000) {
      targetDetected = false;
      firstDetection = true;
      leds[0] = CRGB::Black;  // Turn off LED when target lost
      FastLED.show();
    }
  }

  delay(10);
}