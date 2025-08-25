# OUI‑Spy Fox Hunter 

### (BLE + Wi‑Fi) — M5Stack ATOM (Lite / ATOM GPS)

Single‑target foxhunter firmware for BLE and Wi‑Fi with GPS+SD logging, Web configuration, and RSSI‑driven proximity LED.
 
Related fork [Detector variant](https://github.com/lukeswitz/ouispy-detector/tree/main/M5_Atom_Detector)

---

## Hardware

- Boards:
  - ATOM Lite (ESP32, 1x NeoPixel)
  - ATOM GPS (ESP32 with integrated GPS; equivalent to ATOM Lite + Atomic GPS)

---

## Software / Build

- Board: M5Stack‑ATOM (ESP32)
- Libraries:
  - M5Atom
  - TinyGPSPlus
  - FastLED
  - SD, SPI
  - NimBLE‑Arduino 2.3.4
  - ESPAsyncWebServer v3.8.0, AsyncTCP
- Arduino IDE or PlatformIO

---

## What It Does

- Tracks a single target MAC in BLE and optionally Wi‑Fi
- BLE passive scan; optional time‑sliced Wi‑Fi channel hopping
- LED proximity blink rate scales with RSSI (faster = stronger)
- Logs encounters (BLE or Wi‑Fi) with UTC and GPS

CSV format:
```csv
WhenUTC,TargetMAC,Address,Name,RSSI,Lat,Lon,AltM,HDOP,Type
```
- `Type`: `BLE` or `WIFI`

---

## Quick Start

1) Insert FAT32 microSD and power on  
2) Wait for GPS fix (purple LED blink) or press button to skip
3) Connect to SoftAP:
   - SSID: `snoopuntothem`
   - Password: `astheysnoopuntous`
   - Open the IP printed to Serial to access Web UI
4) Enter Target MAC `AA:BB:CC:DD:EE:FF` and Save → tracking starts  
5) LED blinks turn yellow & faster as RSSI increases; CSV created at:
   - `/FoxHunt-YYYY-MM-DD-N.csv`

---

## LED Indicators

| LED Behavior | Meaning |
|--------------|---------|
| Green  | Config & Default state when no target is detected |
| Yellow | Target detected (blinks faster when closer) |
| Triple Blink | Target acquired for the first time |
| Purple Pulsing | Waiting for GPS fix (pulses faster with more satellites) |

---

## Troubleshooting

- SD init failed:
  - Format card as FAT32
  - Insert before powering

---
