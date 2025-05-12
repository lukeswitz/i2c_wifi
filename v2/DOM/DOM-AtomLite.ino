/*
   M5Atom-Hydra (Dom)
   Optimized Master Node for M5Atom Lite
   Version 2.0 (May 2025)
*/

// CHOOSE COMMUNICATION MODE
#define COMM_I2C
//#define COMM_NOW

// ENABLE WEB SERVER
#define DOM_SERVER

// HARDWARE CONFIG - Atom Lite
#define ATOMLITE

#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <FastLED.h>

#ifdef DOM_SERVER
#include <WebServer.h>
#include <ESPmDNS.h>
#define DOM_SSID "HydraWiFi"
#define DOM_PASS "passpass"
WebServer server(80);
#endif

// Pin definitions for Atom Lite
#define LED_PIN 27
#define SUB_SDA 26
#define SUB_SCL 32
#define GPS_RX 22
#define SD_CLK 23
#define SD_MISO 33
#define SD_MOSI 19
#define BTN 39
#define GPSSERIAL Serial1
#define SD_CS 15  // SD card chip select pin

// I2C settings
#define TCAADDR 0x70
#define I2C_SLAVE_ADDRESS 0x55
#define NUM_PORTS 6             // Support for 6 subs
#define I2C_CLOCK_SPEED 100000  // 100kHz standard I2C speed

// LED
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// Network information structure
struct NetworkInfo {
  char ssid[32];
  char bssid[18];
  int32_t rssi;
  char security[20];
  uint8_t channel;
  char type;
#ifdef COMM_NOW
  int boardId;
#endif
};

// SD card variables
File dataFile;
String fileName;
TinyGPSPlus gps;

// Network counters
int totalNetworks = 0;
int channelNetworks[15] = { 0 };  // Count per channel (0 = BLE, 1-14 = WiFi channels)
bool gpsFixObtained = false;
bool sdCardInitialized = false;

// For tracking sub activity
unsigned long lastSeenSub[NUM_PORTS] = { 0 };
int networksSentBySub[NUM_PORTS] = { 0 };
NetworkInfo receivedNetworks[NUM_PORTS];

// For button management
bool buttonPressed = false;
int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// I2C health tracking
unsigned long lastI2CResetTime = 0;
const unsigned long I2C_RESET_INTERVAL = 300000;  // Reset I2C every 5 minutes if needed
int i2cErrorCount = 0;
bool i2cHealthy = true;

// For non-blocking operations
unsigned long lastNetworkFetchTime = 0;
const unsigned long NETWORK_FETCH_INTERVAL = 500;  // Try fetching networks every 500ms
unsigned long lastStatusUpdateTime = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 10000;  // Update status every 10 seconds

// For runtime stats
unsigned long startTime = 0;
unsigned long lastSuccessfulRead[NUM_PORTS] = { 0 };

// Track unique MAC addresses to avoid duplicates in the CSV
#define MAX_MAC_HISTORY 1000
String seenMacAddresses[MAX_MAC_HISTORY];
int macHistoryIndex = 0;
bool macHistoryFull = false;

// LED colors
CRGB statusColors[] = {
  CRGB::Blue,    // Idle/Starting
  CRGB::Green,   // Good/Success
  CRGB::Red,     // Error
  CRGB::Purple,  // Bluetooth activity
  CRGB::Yellow,  // Warning
  CRGB::Orange   // Processing
};

// Forward declarations
void blinkLED(CRGB color, int times = 1, int duration = 50);
void setLED(CRGB color);
void waitForGPSFix();
void initializeFile();
void logData(const NetworkInfo& network, uint8_t port);
void checkButton();
bool tcaselect(uint8_t i);
bool requestNetworkData(uint8_t port);
void scanI2C();
void resetI2CBus();
bool isPortActive(int port);

#ifdef DOM_SERVER
void handleRoot();
void handleData();
void handleChannelData();
void handleNetworks();
void handleScanI2C();
void handleResetCounters();
void handleDownloadCSV();
void handleResetI2C();
void handleSystemStatus();
#endif

// Initialize the I2C bus
void initI2C() {
  // Reset pins to known state
  pinMode(SUB_SDA, INPUT_PULLUP);
  pinMode(SUB_SCL, INPUT_PULLUP);
  delay(50);

  // Initialize I2C
  Wire.begin(SUB_SDA, SUB_SCL, I2C_CLOCK_SPEED);
  Wire.setTimeout(50);  // 50ms timeout to prevent hanging

  Serial.println("[MASTER] I2C bus initialized at " + String(I2C_CLOCK_SPEED) + " Hz");
}

// Reset the I2C bus if it's not responding
void resetI2CBus() {
  Serial.println("Resetting I2C bus...");

  // End current I2C
  Wire.end();

  // Reset pins to known state
  pinMode(SUB_SDA, INPUT_PULLUP);
  pinMode(SUB_SCL, INPUT_PULLUP);
  delay(50);

  // Toggle SCL to release SDA
  pinMode(SUB_SCL, OUTPUT);
  for (int i = 0; i < 16; i++) {
    digitalWrite(SUB_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(SUB_SCL, LOW);
    delayMicroseconds(5);
  }
  digitalWrite(SUB_SCL, HIGH);
  delayMicroseconds(5);

  // Reset the multiplexer by writing to all channels
  initI2C();
  Wire.beginTransmission(TCAADDR);
  Wire.write(0xFF);  // Enable all channels
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(TCAADDR);
  Wire.write(0x00);  // Disable all channels
  Wire.endTransmission();

  // Reset error count
  i2cErrorCount = 0;
  lastI2CResetTime = millis();

  blinkLED(CRGB::Yellow, 3, 100);
  Serial.println("I2C bus reset complete");
}

// Select TCA multiplexer channel with error handling
bool tcaselect(uint8_t i) {
  if (i >= NUM_PORTS) return false;

  // Use a direct approach without excessive checks that might cause delays
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  byte error = Wire.endTransmission(true);

  if (error == 0) {
    return true;
  } else {
    i2cErrorCount++;
    return false;
  }
}


// Request network data from a sub via I2C with better error handling
bool requestNetworkData(uint8_t port) {
  // Set a shorter, explicit timeout
  Wire.setTimeout(20);  // 20ms timeout is plenty

  // Make a more efficient request without waiting
  Wire.requestFrom(I2C_SLAVE_ADDRESS, sizeof(NetworkInfo));

  // Check if enough data was received immediately - fail fast
  if (Wire.available() < sizeof(NetworkInfo)) {
    return false;
  }

  // Read the data more efficiently
  byte* bufferPtr = (byte*)&receivedNetworks[port];
  for (size_t i = 0; i < sizeof(NetworkInfo); i++) {
    if (Wire.available()) {
      *(bufferPtr + i) = Wire.read();
    } else {
      return false;
    }
  }

  // Basic validation check
  if (receivedNetworks[port].channel > 14) {
    return false;
  }

  // Success - update stats
  lastSeenSub[port] = millis();
  lastSuccessfulRead[port] = millis();
  networksSentBySub[port]++;

  // setLED(statusColors[1]);  // Just briefly set a color, no blinking

  return true;
}


// Check if a port is currently active
bool isPortActive(int port) {
  if (lastSeenSub[port] == 0) {
    return false;  // Never seen
  }

  unsigned long timeSinceLastSeen = millis() - lastSeenSub[port];
  return (timeSinceLastSeen < 60000);  // Consider active if seen in the last minute
}

// Scan for I2C devices (diagnostic function)
void scanI2C() {
  byte error, address;
  int deviceCount = 0;
  unsigned long scanStart = millis();

  Serial.println("Scanning I2C bus...");

  // Test for the TCA multiplexer first
  Serial.print("TCA9548A multiplexer at address 0x");
  Serial.print(TCAADDR, HEX);
  Serial.print("... ");

  Wire.beginTransmission(TCAADDR);
  error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("FOUND!");
    deviceCount++;
  } else {
    Serial.print("NOT FOUND (error ");
    Serial.print(error);
    Serial.println(")");
  }

  // Scan each port of the multiplexer
  for (uint8_t port = 0; port < NUM_PORTS; port++) {
    if (tcaselect(port)) {
      Serial.println("Scanning devices on port " + String(port) + ":");

      // First check specifically for the Hydra Sub address
      Wire.beginTransmission(I2C_SLAVE_ADDRESS);
      error = Wire.endTransmission();

      if (error == 0) {
        Serial.print("  Found Hydra Sub at address 0x");
        if (I2C_SLAVE_ADDRESS < 16) Serial.print("0");
        Serial.println(I2C_SLAVE_ADDRESS, HEX);

        // Mark this node as seen, even if it hasn't reported networks
        lastSeenSub[port] = millis();
        deviceCount++;

        // Continue to scan for other devices on this port
      }

      // Now scan all addresses
      for (address = 1; address < 127; address++) {
        if (address == TCAADDR || address == I2C_SLAVE_ADDRESS) continue;  // Skip addresses we already checked

        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
          Serial.print("  Found device at address 0x");
          if (address < 16) Serial.print("0");
          Serial.println(address, HEX);
          deviceCount++;
        }
      }
    } else {
      Serial.println("Could not access port " + String(port));
    }
  }

  if (deviceCount == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.println("I2C scan complete, found " + String(deviceCount) + " devices");
  }

  Serial.println("Scan duration: " + String(millis() - scanStart) + "ms");
}


// Set the LED to a specific color
void setLED(CRGB color) {
  leds[0] = color;
  FastLED.show();
}

// Blink the LED
void blinkLED(CRGB color, int times, int duration) {
  for (int i = 0; i < times; i++) {
    setLED(color);
    delay(duration);
    setLED(CRGB::Black);

    if (i < times - 1) {
      delay(duration);  // Delay between blinks
    }
  }
}

// Wait for GPS fix with timeout and progress indicator
void waitForGPSFix() {
  Serial.println("Waiting for GPS fix...");
  unsigned long startTime = millis();
  const unsigned long gpsTimeout = 120000;  // 2 minute timeout
  unsigned long lastDotTime = 0;

  while (!gps.location.isValid() && (millis() - startTime < gpsTimeout)) {
    // Process any GPS data
    while (GPSSERIAL.available() > 0) {
      gps.encode(GPSSERIAL.read());
    }

    // Visual indicator - blink LED
    if ((millis() / 500) % 2 == 0) {
      setLED(CRGB::Purple);
    } else {
      setLED(CRGB::Black);
    }

    // Print progress dots
    if (millis() - lastDotTime > 5000) {
      Serial.print(".");
      lastDotTime = millis();
    }

    // Check button for early bypass
    checkButton();
    if (buttonPressed) {
      Serial.println("\nButton pressed, skipping GPS wait");
      break;
    }

    delay(10);
  }

  if (gps.location.isValid()) {
    gpsFixObtained = true;
    Serial.println("\nGPS fix obtained!");
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
    Serial.print("HDOP: ");
    Serial.println(gps.hdop.value());

    // Show success
    blinkLED(CRGB::Green, 3, 100);
  } else {
    Serial.println("\nGPS fix not obtained. Continuing without GPS.");
    // Show warning
    blinkLED(CRGB::Yellow, 5, 100);
  }
}

// Check if a MAC address has been seen before
bool isMacAlreadySeen(const String& bssid) {
  // Check against our history of seen MAC addresses
  for (int i = 0; i < (macHistoryFull ? MAX_MAC_HISTORY : macHistoryIndex); i++) {
    if (seenMacAddresses[i] == bssid) {
      return true;
    }
  }

  // New MAC address - add to history
  seenMacAddresses[macHistoryIndex] = bssid;
  macHistoryIndex++;

  // Reset index if we've reached capacity
  if (macHistoryIndex >= MAX_MAC_HISTORY) {
    macHistoryIndex = 0;
    macHistoryFull = true;
  }

  return false;
}

// Initialize the log file
void initializeFile() {
  int fileNumber = 0;
  bool isNewFile = false;

  // Create a datestamp for the filename
  char fileDateStamp[16];
  if (gps.date.isValid()) {
    sprintf(fileDateStamp, "%04d-%02d-%02d-",
            gps.date.year(), gps.date.month(), gps.date.day());
  } else {
    // If no GPS date, use a default
    time_t now;
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    sprintf(fileDateStamp, "%04d-%02d-%02d-",
            timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
  }

  // Find an available filename
  do {
    fileName = "/wifi-scans-" + String(fileDateStamp) + String(fileNumber) + ".csv";
    isNewFile = !SD.exists(fileName);
    fileNumber++;
  } while (!isNewFile && fileNumber < 100);

  if (isNewFile) {
    File dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("WigleWifi-1.4,appRelease=1.300000,model=GPS Kit,release=1.100000F+00,device=M5ATOMHydra,display=ATOMLITE,board=ESP32,brand=M5");
      dataFile.println("MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type");
      dataFile.close();
      Serial.println("New file created: " + fileName);
      sdCardInitialized = true;

      // Show success
      blinkLED(CRGB::Green, 2, 100);
    } else {
      Serial.println("Error creating file on SD card");
      blinkLED(CRGB::Red, 3, 100);
    }
  } else {
    Serial.println("Could not create file: too many files exist");
    blinkLED(CRGB::Red, 5, 100);
  }
}

// Log network data to SD card
void logData(const NetworkInfo& network, uint8_t port) {
  // Skip logging our own WiFi AP if present
  if (strcmp("HydraWiFi", network.ssid) == 0) {
    return;
  }


  // Check if we've already seen this MAC address - DEDUPLICATION
  if (isMacAlreadySeen(network.bssid)) {
    Serial.println("Skip duplicate MAC: " + String(network.bssid));
    return;
  }

  if (sdCardInitialized) {
    // Format timestamp
    String utc;
    if (gpsFixObtained && gps.date.isValid() && gps.time.isValid()) {
      utc = String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day()) + " " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
    } else {
      // Use internal time if GPS not available
      time_t now;
      time(&now);
      struct tm timeinfo;
      localtime_r(&now, &timeinfo);
      char timeStr[20];
      sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d",
              timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
              timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      utc = String(timeStr);
    }

    // Create CSV row
    String dataString = String(network.bssid) + "," + "\"" + network.ssid + "\"" + "," + network.security + "," + utc + "," + String(network.channel) + "," + String(network.rssi) + ",";

    // Add GPS coordinates if available
    if (gpsFixObtained && gps.location.isValid()) {
      dataString += String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "," + String(gps.altitude.meters(), 2) + "," + String(gps.hdop.hdop(), 2);
    } else {
      // Add placeholders if GPS not available
      dataString += "0,0,0,0";
    }

    // Add network type
    if (network.type == 'w') {
      dataString += ",WIFI";
    } else if (network.type == 'b') {
      dataString += ",BLE";
    } else {
      dataString += ",UNKNOWN";
    }

    // Write to SD card
    File dataFile = SD.open(fileName, FILE_APPEND);
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();

      // Update counters
      totalNetworks++;
      channelNetworks[network.channel]++;

      // Serial.println("Data written: " + dataString);
    } else {
      Serial.println("Error writing to " + fileName);
      blinkLED(CRGB::Red, 1, 50);
    }
  } else {
    // No SD card, still count networks
    totalNetworks++;
    channelNetworks[network.channel]++;
  }
}

// Check button state with debouncing
void checkButton() {
  int reading = digitalRead(BTN);

  // Check if the button state changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // If enough time has passed, consider the button state stable
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has changed
    if (reading != buttonState) {
      buttonState = reading;

      // On M5Atom, button is pulled high, so LOW means pressed
      if (buttonState == LOW) {
        buttonPressed = true;

        // Print stats on button press
        Serial.println("Button pressed!");
        Serial.println("Total networks logged: " + String(totalNetworks));

        for (int i = 0; i < NUM_PORTS; i++) {
          Serial.println("Sub " + String(i) + " networks: " + String(networksSentBySub[i]));
        }

        // Show channel distribution
        Serial.println("Channel distribution:");
        for (int i = 0; i < 15; i++) {
          if (i == 0) {
            Serial.println("BLE: " + String(channelNetworks[i]));
          } else {
            Serial.println("CH" + String(i) + ": " + String(channelNetworks[i]));
          }
        }

        // Run an I2C scan when button is pressed (for debugging)
        scanI2C();
      }
    }
  }

  lastButtonState = reading;
}

// Get system uptime string
String getUptimeString() {
  unsigned long uptime = millis() / 1000;  // Convert to seconds

  unsigned long seconds = uptime % 60;
  unsigned long minutes = (uptime / 60) % 60;
  unsigned long hours = (uptime / 3600) % 24;
  unsigned long days = uptime / 86400;

  if (days > 0) {
    return String(days) + " days, " + String(hours) + "h " + String(minutes) + "m";
  } else if (hours > 0) {
    return String(hours) + "h " + String(minutes) + "m " + String(seconds) + "s";
  } else {
    return String(minutes) + "m " + String(seconds) + "s";
  }
}

#ifdef DOM_SERVER
// Web server handlers
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>HYDRA NETWORK SCANNER</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "* {margin: 0; padding: 0; box-sizing: border-box; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;}";
  html += "body {background: #000; color: #0f0; padding: 0; margin: 0; overflow-x: hidden;}";
  html += "h1, h2, h3 {color: #0f0; text-shadow: 0 0 5px #0f0; text-transform: uppercase; letter-spacing: 2px; margin: 15px 0;}";
  html += ".wrapper {max-width: 1200px; margin: 0 auto; padding: 10px;}";
  html += ".header {background: linear-gradient(to bottom, #111, #000); padding: 20px; border-bottom: 1px solid #0f0; position: relative; overflow: hidden;}";
  html += ".header::before {content: ''; position: absolute; top: 0; left: 0; right: 0; bottom: 0; background: url('data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxMDAiIGhlaWdodD0iMTAwIj48ZyBvcGFjaXR5PSIwLjI1Ij48cGF0aCBkPSJNMCAwaDEwMHYxMDBIMHoiIGZpbGw9Im5vbmUiLz48cGF0aCBkPSJNMCAwaDEwMHYyNUwwIDEwMHoiIGZpbGw9IiMwZjAiLz48L2c+PC9zdmc+'); opacity: 0.05; z-index: 0;}";
  html += ".content {padding: 20px; margin-top: 20px;}";
  html += ".panel {background: rgba(0, 20, 0, 0.4); border: 1px solid #0f0; border-radius: 0; padding: 20px; margin-bottom: 20px; box-shadow: 0 0 10px rgba(0, 255, 0, 0.2), inset 0 0 20px rgba(0, 20, 0, 0.4); position: relative; overflow: hidden;}";
  html += ".panel::after {content: ''; position: absolute; top: 0; left: 0; width: 100%; height: 2px; background: linear-gradient(90deg, transparent, rgba(0, 255, 0, 0.5), transparent);}";
  html += ".indicator {display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 10px; box-shadow: 0 0 5px currentColor;}";
  html += ".online {color: #0f0;}";
  html += ".idle {color: #fa0;}";
  html += ".offline {color: #f00;}";
  html += ".never {color: #777;}";
  html += ".btn {background: rgba(0, 30, 0, 0.8); border: 1px solid #0f0; color: #0f0; padding: 10px 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 14px; margin: 5px 2px; cursor: pointer; transition: all 0.3s ease; text-transform: uppercase; letter-spacing: 1px; position: relative; overflow: hidden;}";
  html += ".btn:hover {background: rgba(0, 40, 0, 0.9); box-shadow: 0 0 10px rgba(0, 255, 0, 0.5);}";
  html += ".btn::after {content: ''; position: absolute; top: -50%; left: -50%; width: 200%; height: 200%; background: linear-gradient(rgba(255, 255, 255, 0.1), transparent); transform: rotate(30deg); transition: all 0.3s ease;}";
  html += ".btn:hover::after {transform: rotate(30deg) translate(-10%, -10%);}";
  html += ".btn-reset {border-color: #f00; color: #f00;}";
  html += ".btn-reset:hover {background: rgba(40, 0, 0, 0.9); box-shadow: 0 0 10px rgba(255, 0, 0, 0.5);}";
  html += ".btn-scan {border-color: #0af; color: #0af;}";
  html += ".btn-scan:hover {background: rgba(0, 10, 40, 0.9); box-shadow: 0 0 10px rgba(0, 160, 255, 0.5);}";
  html += ".status-grid {display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); grid-gap: 15px; margin: 20px 0;}";
  html += ".status-item {background: rgba(0, 10, 0, 0.6); border: 1px solid rgba(0, 255, 0, 0.2); padding: 15px; position: relative;}";
  html += ".status-item::before {content: ''; position: absolute; top: 0; left: 0; width: 100%; height: 2px; background: linear-gradient(90deg, transparent, rgba(0, 255, 0, 0.3), transparent);}";
  html += ".chart {position: relative; height: 180px; margin: 30px 0; background: rgba(0, 10, 0, 0.6); border: 1px solid rgba(0, 255, 0, 0.3); padding: 10px;}";
  html += ".chart-label {position: absolute; bottom: -20px; text-align: center; font-size: 10px; color: #0f0;}";
  html += ".bar {position: absolute; bottom: 0; width: 6%; background: linear-gradient(to top, #0f0, #040); margin-right: 0.5%; text-align: center; color: #fff; transition: height 0.5s ease; border-top: 1px solid #0f0; box-shadow: 0 0 5px rgba(0, 255, 0, 0.5); z-index: 2;}";
  html += ".bar span {position: absolute; top: -20px; left: 0; width: 100%; text-align: center; font-size: 10px; color: #0f0;}";
  html += ".ble-bar {background: linear-gradient(to top, #f0f, #404);}";
  html += ".grid-line {position: absolute; width: 100%; height: 1px; background: rgba(0, 255, 0, 0.1); z-index: 1;}";
  html += ".grid-label {position: absolute; left: -35px; font-size: 10px; color: rgba(0, 255, 0, 0.7);}";
  html += ".scanner-status {display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; margin-bottom: 15px; border-bottom: 1px dashed rgba(0, 255, 0, 0.2); padding-bottom: 10px;}";
  html += ".scanner-status div {margin: 5px 10px 5px 0; padding: 5px 10px; background: rgba(0, 20, 0, 0.5); border: 1px solid rgba(0, 255, 0, 0.2);}";
  html += ".blinking {animation: blink 1.5s linear infinite;}";
  html += "@keyframes blink {0% {opacity: 1;} 50% {opacity: 0.3;} 100% {opacity: 1;}}";
  html += ".total-counter {font-size: 24px; font-weight: bold; color: #0f0; text-align: center; margin: 20px 0; text-shadow: 0 0 10px rgba(0, 255, 0, 0.5);}";
  html += ".version {position: absolute; bottom: 5px; right: 10px; font-size: 10px; color: #0a0;}";
  html += ".scan-animation {display: inline-block; position: relative; width: 10px; height: 10px; margin-right: 10px;}";
  html += ".scan-animation::before {content: ''; position: absolute; top: 0; left: 0; width: 100%; height: 100%; border-radius: 50%; background: #0f0; animation: pulse 1.5s ease-out infinite;}";
  html += "@keyframes pulse {0% {transform: scale(0.1); opacity: 1;} 100% {transform: scale(2); opacity: 0;}}";
  html += ".server-info {font-size: 12px; margin-top: 20px; color: rgba(0, 255, 0, 0.7); text-align: center;}";
  html += ".server-info a {color: #0f0; text-decoration: none;}";
  html += ".tab-container {margin-bottom: 20px;}";
  html += ".tab {overflow: hidden; border: 1px solid #0f0; background-color: rgba(0, 20, 0, 0.5);}";
  html += ".tab button {background-color: inherit; float: left; border: none; outline: none; cursor: pointer; padding: 14px 16px; transition: 0.3s; color: #0f0; text-transform: uppercase; letter-spacing: 1px;}";
  html += ".tab button:hover {background-color: rgba(0, 40, 0, 0.7);}";
  html += ".tab button.active {background-color: rgba(0, 60, 0, 0.7); border-bottom: 2px solid #0f0;}";
  html += ".tabcontent {display: none; padding: 20px; border: 1px solid #0f0; border-top: none; background-color: rgba(0, 10, 0, 0.5);}";
  html += ".tabcontent.active {display: block;}";
  html += ".system-stats {display: flex; justify-content: space-between; flex-wrap: wrap;}";
  html += ".system-stats > div {margin: 10px; padding: 15px; background: rgba(0, 15, 0, 0.5); border: 1px solid rgba(0, 255, 0, 0.2); min-width: 150px; flex: 1;}";
  html += ".system-stats h3 {margin-top: 0; font-size: 14px;}";
  html += ".system-stats .value {font-size: 20px; margin-top: 5px;}";
  html += ".debug-log {background: rgba(0, 20, 0, 0.5); border: 1px solid rgba(0, 255, 0, 0.2); height: 200px; overflow-y: auto; padding: 10px; font-family: monospace; margin-top: 20px;}";
  html += ".debug-log p {margin: 2px 0; font-size: 12px; color: #0f0;}";
  html += ".debug-log .error {color: #f00;}";
  html += ".debug-log .warning {color: #ff0;}";
  html += ".debug-log .success {color: #0f0;}";
  html += "</style>";

  // Add JavaScript for dynamic content and tabs
  html += "<script>";
  // Tab functionality
  html += "function openTab(evt, tabName) {";
  html += "  var i, tabcontent, tablinks;";
  html += "  tabcontent = document.getElementsByClassName('tabcontent');";
  html += "  for (i = 0; i < tabcontent.length; i++) {";
  html += "    tabcontent[i].className = tabcontent[i].className.replace(' active', '');";
  html += "  }";
  html += "  tablinks = document.getElementsByClassName('tablinks');";
  html += "  for (i = 0; i < tablinks.length; i++) {";
  html += "    tablinks[i].className = tablinks[i].className.replace(' active', '');";
  html += "  }";
  html += "  document.getElementById(tabName).className += ' active';";
  html += "  evt.currentTarget.className += ' active';";
  html += "  localStorage.setItem('activeTab', tabName);";
  html += "}";

  // Data refresh functions
  html += "function updateMainData() {";
  html += "  fetch('/data').then(response => response.text()).then(data => {";
  html += "    document.getElementById('main-data-container').innerHTML = data;";
  html += "  }).catch(error => console.error('Error:', error));";
  html += "}";

  html += "function updateChannelData() {";
  html += "  fetch('/channel_data').then(response => response.text()).then(data => {";
  html += "    document.getElementById('channel-data-container').innerHTML = data;";
  html += "  }).catch(error => console.error('Error:', error));";
  html += "}";

  html += "function updateNetworkData() {";
  html += "  fetch('/networks').then(response => response.text()).then(data => {";
  html += "    document.getElementById('network-data-container').innerHTML = data;";
  html += "  }).catch(error => console.error('Error:', error));";
  html += "}";

  html += "function updateSystemData() {";
  html += "  fetch('/system_status').then(response => response.text()).then(data => {";
  html += "    document.getElementById('system-data-container').innerHTML = data;";
  html += "  }).catch(error => console.error('Error:', error));";
  html += "}";

  html += "function updateAllData() {";
  html += "  updateMainData();";
  html += "  updateChannelData();";
  html += "  updateNetworkData();";
  html += "  updateSystemData();";
  html += "}";

  // Initial setup
  html += "window.onload = function() {";
  html += "  updateAllData();";
  html += "  setInterval(updateAllData, 5000);";  // Refresh every 5 seconds

  // Restore active tab
  html += "  var activeTab = localStorage.getItem('activeTab') || 'Dashboard';";
  html += "  document.getElementById(activeTab).className += ' active';";
  html += "  document.querySelector('button[onclick=\"openTab(event, \\'' + activeTab + '\\')\"').className += ' active';";
  html += "};";
  html += "</script>";

  html += "</head><body>";

  // Header with styling
  html += "<div class='header'>";
  html += "<div class='wrapper'>";
  html += "<h1>HYDRA NETWORK SCANNER</h1>";
  html += "<p>Wireless Intelligence Collection System</p>";
  html += "<div class='version'>v2.0-ATOM</div>";
  html += "</div>";
  html += "</div>";

  html += "<div class='wrapper'>";

  // Tabs
  html += "<div class='tab-container'>";
  html += "<div class='tab'>";
  html += "<button class='tablinks active' onclick=\"openTab(event, 'Dashboard')\">Dashboard</button>";
  html += "<button class='tablinks' onclick=\"openTab(event, 'Channels')\">Channel Data</button>";
  html += "<button class='tablinks' onclick=\"openTab(event, 'Networks')\">Network Log</button>";
  html += "<button class='tablinks' onclick=\"openTab(event, 'System')\">System Status</button>";
  html += "</div>";

  // Dashboard Tab
  html += "<div id='Dashboard' class='tabcontent active'>";
  html += "<div id='main-data-container'>Loading dashboard data...</div>";
  html += "</div>";

  // Channels Tab
  html += "<div id='Channels' class='tabcontent'>";
  html += "<div id='channel-data-container'>Loading channel data...</div>";
  html += "</div>";

  // Networks Tab
  html += "<div id='Networks' class='tabcontent'>";
  html += "<div id='network-data-container'>Loading network data...</div>";
  html += "</div>";

  // System Tab
  html += "<div id='System' class='tabcontent'>";
  html += "<div id='system-data-container'>Loading system status...</div>";
  html += "</div>";
  html += "</div>";

  // Control panel
  html += "<div class='panel'>";
  html += "<h2>CONTROLS</h2>";
  html += "<a href='/scan_i2c' class='btn btn-scan'>SCAN I²C BUS</a> ";
  html += "<a href='/reset_counters' class='btn btn-reset'>RESET COUNTERS</a> ";
  html += "<a href='/reset_i2c' class='btn btn-scan'>RESET I²C BUS</a> ";
  html += "<a href='/download_csv' class='btn'>EXPORT DATA</a>";
  html += "</div>";

  // Server information section with IP address
  html += "<div class='server-info'>";
  html += "SERVER: HYDRA COMMAND NODE | IP: <a href='http://192.168.4.1'>192.168.4.1</a> | SSID: " + String(DOM_SSID);
  html += "</div>";

  html += "</div>";  // End wrapper

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleData() {
  String data = "<div class='panel'>";
  data += "<h2>MISSION STATUS</h2>";
  data += "<div class='scanner-status'>";

  // GPS Status indicator
  data += "<div><span class='indicator " + String(gpsFixObtained ? "online" : "offline") + "'></span>";
  data += "GPS: " + String(gpsFixObtained ? "ONLINE" : "OFFLINE") + "</div>";

  // SD Card Status indicator
  data += "<div><span class='indicator " + String(sdCardInitialized ? "online" : "offline") + "'></span>";
  data += "STORAGE: " + String(sdCardInitialized ? "READY" : "UNAVAILABLE") + "</div>";

  // I2C Bus Status
  data += "<div><span class='indicator " + String(i2cHealthy ? "online" : "offline") + "'></span>";
  data += "I2C BUS: " + String(i2cHealthy ? "HEALTHY" : "ISSUES") + "</div>";

  // Active File
  if (sdCardInitialized) {
    data += "<div>LOG: " + fileName.substring(1) + "</div>";
  }

  data += "</div>";  // End scanner-status

  // GPS data if available
  if (gpsFixObtained) {
    data += "<div class='status-grid'>";
    data += "<div class='status-item'>";
    data += "<h3>COORDINATES</h3>";
    data += "<div>" + String(gps.location.lat(), 6) + ", " + String(gps.location.lng(), 6) + "</div>";
    data += "</div>";

    data += "<div class='status-item'>";
    data += "<h3>SATELLITES</h3>";
    data += "<div>" + String(gps.satellites.value()) + "</div>";
    data += "</div>";

    data += "<div class='status-item'>";
    data += "<h3>SIGNAL QUALITY</h3>";
    data += "<div>HDOP: " + String(gps.hdop.value()) + "</div>";
    data += "</div>";

    data += "<div class='status-item'>";
    data += "<h3>ALTITUDE</h3>";
    data += "<div>" + String(gps.altitude.meters(), 1) + " m</div>";
    data += "</div>";
    data += "</div>";  // End status-grid
  }

  // Nodes status grid
  data += "<h2>NODE STATUS</h2>";
  data += "<div class='status-grid'>";

  unsigned long currentTime = millis();
  int activeNodes = 0;

  for (int i = 0; i < NUM_PORTS; i++) {
    String nodeClass = "never";
    String nodeStatus = "DISCONNECTED";
    String timeSinceStr = "NEVER SEEN";

    if (lastSeenSub[i] > 0) {
      // Calculate time since last seen
      unsigned long timeSince = currentTime - lastSeenSub[i];

      if (timeSince < 30000) {  // Within 30 seconds
        nodeClass = "online";
        nodeStatus = "ONLINE";
        activeNodes++;
      } else if (timeSince < 120000) {  // Within 2 minutes
        nodeClass = "idle";
        nodeStatus = "IDLE";
      } else {
        nodeClass = "offline";
        nodeStatus = "OFFLINE";
      }

      // Format time since
      if (timeSince < 60000) {
        timeSinceStr = String(timeSince / 1000) + " SEC AGO";
      } else if (timeSince < 3600000) {
        timeSinceStr = String(timeSince / 60000) + " MIN AGO";
      } else {
        timeSinceStr = String(timeSince / 3600000) + " HR AGO";
      }
    }

    data += "<div class='status-item'>";
    data += "<h3>NODE " + String(i) + "</h3>";
    data += "<div><span class='indicator " + nodeClass + "'></span>" + nodeStatus + "</div>";
    data += "<div>CAPTURES: " + String(networksSentBySub[i]) + "</div>";
    data += "<div>LAST CONTACT: " + timeSinceStr + "</div>";
    data += "</div>";
  }

  data += "</div>";  // End status-grid

  // Total network counter with blinking effect
  data += "<div class='total-counter'>";
  data += "<span class='scan-animation'></span> NETWORKS CAPTURED: " + String(totalNetworks);
  data += " | ACTIVE NODES: " + String(activeNodes) + "/" + String(NUM_PORTS);
  data += "</div>";

  data += "</div>";  // End panel

  server.send(200, "text/html", data);
}

void handleChannelData() {
  String data = "<div class='panel'>";
  data += "<h2>CHANNEL INTELLIGENCE</h2>";
  data += "<div class='chart'>";

  // Find the maximum value for scaling
  int maxNetworks = 1;  // Prevent division by zero
  for (int i = 0; i < 15; i++) {
    if (channelNetworks[i] > maxNetworks) {
      maxNetworks = channelNetworks[i];
    }
  }

  // Add horizontal grid lines
  for (int i = 0; i <= 4; i++) {
    int value = (maxNetworks / 4) * i;
    int position = 100 - (i * 25);
    data += "<div class='grid-line' style='bottom: " + String(position) + "%;'></div>";
    data += "<div class='grid-label' style='bottom: " + String(position - 5) + "%;'>" + String(value) + "</div>";
  }

  // Add bars for each channel
  for (int i = 0; i < 15; i++) {
    int height = channelNetworks[i] > 0 ? (channelNetworks[i] * 100 / maxNetworks) : 1;
    if (height < 1) height = 1;  // Ensure bar is visible

    data += "<div class='bar " + String(i == 0 ? "ble-bar" : "") + "' style='height: " + String(height) + "%; left: " + String(i * 6.5) + "%;'>";
    data += "<span>" + String(channelNetworks[i]) + "</span>";
    data += "</div>";

    // Add channel label under each bar
    data += "<div class='chart-label' style='left: " + String(i * 6.5 + 2.25) + "%;'>";
    data += i == 0 ? "BLE" : String(i);
    data += "</div>";
  }

  data += "</div>";  // End chart

  // Channel distribution table
  data += "<h3 style='margin-top: 40px;'>DETAILED CHANNEL DATA</h3>";
  data += "<table style='width:100%; border-collapse: collapse; margin-top: 15px;'>";
  data += "<tr style='border-bottom: 1px solid rgba(0, 255, 0, 0.3);'>";
  data += "<th style='text-align: left; padding: 8px;'>Channel</th>";
  data += "<th style='text-align: right; padding: 8px;'>Networks</th>";
  data += "<th style='text-align: right; padding: 8px;'>Percentage</th>";
  data += "</tr>";

  // Calculate total for percentage
  int sum = 0;
  for (int i = 0; i < 15; i++) {
    sum += channelNetworks[i];
  }

  // Add rows for each channel
  for (int i = 0; i < 15; i++) {
    float percentage = sum > 0 ? (channelNetworks[i] * 100.0 / sum) : 0;

    data += "<tr style='border-bottom: 1px solid rgba(0, 255, 0, 0.1);'>";
    data += "<td style='padding: 8px;'>" + (i == 0 ? String("BLE") : String("Channel ") + String(i)) + "</td>";
    data += "<td style='text-align: right; padding: 8px;'>" + String(channelNetworks[i]) + "</td>";
    data += "<td style='text-align: right; padding: 8px;'>" + String(percentage, 1) + "%</td>";
    data += "</tr>";
  }

  data += "<tr style='border-top: 2px solid rgba(0, 255, 0, 0.3);'>";
  data += "<td style='padding: 8px; font-weight: bold;'>Total</td>";
  data += "<td style='text-align: right; padding: 8px; font-weight: bold;'>" + String(sum) + "</td>";
  data += "<td style='text-align: right; padding: 8px; font-weight: bold;'>100%</td>";
  data += "</tr>";
  data += "</table>";

  data += "</div>";  // End panel

  server.send(200, "text/html", data);
}

// Display recent network captures
void handleNetworks() {
  String data = "<div class='panel'>";
  data += "<h2>RECENT NETWORK CAPTURES</h2>";

  // Create a table of network information from the most recent captures
  data += "<div style='overflow-x: auto;'>";
  data += "<table style='width: 100%; border-collapse: collapse; margin-top: 15px;'>";
  data += "<tr style='border-bottom: 2px solid rgba(0, 255, 0, 0.3);'>";
  data += "<th style='text-align: left; padding: 8px;'>Node</th>";
  data += "<th style='text-align: left; padding: 8px;'>SSID</th>";
  data += "<th style='text-align: left; padding: 8px;'>BSSID</th>";
  data += "<th style='text-align: center; padding: 8px;'>Type</th>";
  data += "<th style='text-align: center; padding: 8px;'>Ch</th>";
  data += "<th style='text-align: center; padding: 8px;'>RSSI</th>";
  data += "<th style='text-align: left; padding: 8px;'>Security</th>";
  data += "</tr>";

  // Flag to track if we have any valid networks to display
  bool hasValidNetworks = false;

  // Add only valid networks from each sub
  for (int i = 0; i < NUM_PORTS; i++) {
    if (lastSuccessfulRead[i] > 0 && networksSentBySub[i] > 0) {
      // Skip nodes with garbage data (invalid RSSI/channel values are common indicators)
      if (receivedNetworks[i].rssi == -1 || receivedNetworks[i].channel == 255) {
        continue;
      }

      hasValidNetworks = true;
      String rowColor = "";

      // Set row color based on last seen time
      unsigned long timeSince = millis() - lastSuccessfulRead[i];
      if (timeSince < 30000) {  // Within 30 seconds - green
        rowColor = "background-color: rgba(0, 30, 0, 0.4);";
      } else if (timeSince < 120000) {  // Within 2 minutes - yellow
        rowColor = "background-color: rgba(30, 30, 0, 0.3);";
      } else {  // Older - red
        rowColor = "background-color: rgba(30, 0, 0, 0.3);";
      }

      data += "<tr style='border-bottom: 1px solid rgba(0, 255, 0, 0.1); " + rowColor + "'>";
      data += "<td style='padding: 8px;'>Node " + String(i) + "</td>";

      // SSID column - properly sanitize
      String ssid = "";
      for (int j = 0; j < 32 && receivedNetworks[i].ssid[j] != 0; j++) {
        char c = receivedNetworks[i].ssid[j];
        if (c >= 32 && c <= 126) {
          ssid += c;
        }
      }
      if (ssid.length() == 0) ssid = "[Hidden]";
      if (ssid.length() > 20) ssid = ssid.substring(0, 17) + "...";
      data += "<td style='padding: 8px;'>" + ssid + "</td>";

      // BSSID column - format MAC address
      char bssidStr[18] = { 0 };
      sprintf(bssidStr, "%02X:%02X:%02X:%02X:%02X:%02X",
              receivedNetworks[i].bssid[0], receivedNetworks[i].bssid[1],
              receivedNetworks[i].bssid[2], receivedNetworks[i].bssid[3],
              receivedNetworks[i].bssid[4], receivedNetworks[i].bssid[5]);
      data += "<td style='padding: 8px; font-family: monospace;'>" + String(bssidStr) + "</td>";

      // Type column
      String typeText = "WiFi";
      String typeColor = "color: #0af;";
      if (receivedNetworks[i].type == 'b') {
        typeText = "BLE";
        typeColor = "color: #f0f;";
      }
      data += "<td style='text-align: center; padding: 8px; " + typeColor + "'>" + typeText + "</td>";

      // Channel column
      data += "<td style='text-align: center; padding: 8px;'>" + String(receivedNetworks[i].channel) + "</td>";

      // RSSI column with signal strength indicator
      int rssi = receivedNetworks[i].rssi;
      String signalColor;

      if (rssi > -50) {
        signalColor = "color: #0f0;";
      } else if (rssi > -70) {
        signalColor = "color: #af0;";
      } else if (rssi > -80) {
        signalColor = "color: #ff0;";
      } else {
        signalColor = "color: #f00;";
      }

      data += "<td style='text-align: center; padding: 8px; " + signalColor + "'>" + String(rssi) + " dBm</td>";

      // Security column - safely display security type
      String security = "";
      for (int j = 0; j < 20 && receivedNetworks[i].security[j] != 0; j++) {
        char c = receivedNetworks[i].security[j];
        if (c >= 32 && c <= 126) {
          security += c;
        }
      }
      if (security.length() == 0) security = "Unknown";
      if (security.length() > 20) security = security.substring(0, 17) + "...";
      data += "<td style='padding: 8px;'>" + security + "</td>";

      data += "</tr>";
    }
  }

  data += "</table>";
  data += "</div>";

  // Show message if no valid networks
  if (!hasValidNetworks) {
    data += "<div style='text-align: center; padding: 20px; color: #ff0;'>";
    data += "No valid network data received yet. Waiting for captures...";
    data += "</div>";
  }

  // Add stats about network collection
  data += "<div style='margin-top: 20px;'>";
  data += "<p>Total networks captured: " + String(totalNetworks) + "</p>";

  // Add summary per node
  data += "<h3 style='margin-top: 15px;'>Networks Per Node</h3>";
  data += "<div style='display: flex; flex-wrap: wrap;'>";

  for (int i = 0; i < NUM_PORTS; i++) {
    data += "<div style='margin: 10px; padding: 10px; background: rgba(0, 20, 0, 0.4); border: 1px solid rgba(0, 255, 0, 0.2);'>";
    data += "<strong>Node " + String(i) + ":</strong> " + String(networksSentBySub[i]) + " networks";
    data += "</div>";
  }

  data += "</div>";
  data += "</div>";

  data += "</div>";  // End panel

  server.send(200, "text/html", data);
}

void handleSystemStatus() {
  String data = "<div class='panel'>";
  data += "<h2>SYSTEM STATUS</h2>";

  // System statistics grid
  data += "<div class='system-stats'>";

  // Uptime
  data += "<div>";
  data += "<h3>UPTIME</h3>";
  data += "<div class='value'>" + getUptimeString() + "</div>";
  data += "</div>";

  // Free Memory
  data += "<div>";
  data += "<h3>FREE MEMORY</h3>";
  data += "<div class='value'>" + String(ESP.getFreeHeap() / 1024) + " KB</div>";
  data += "</div>";

  // I2C Status
  data += "<div>";
  data += "<h3>I2C STATUS</h3>";
  data += "<div class='value'>" + String(i2cErrorCount) + " errors</div>";
  data += "</div>";

  // CPU Temperature
  data += "<div>";
  data += "<h3>CPU TEMP</h3>";
  data += "<div class='value'>" + String(temperatureRead(), 1) + " °C</div>";
  data += "</div>";

  // Active Nodes
  int activeNodes = 0;
  for (int i = 0; i < NUM_PORTS; i++) {
    if (isPortActive(i)) activeNodes++;
  }

  data += "<div>";
  data += "<h3>ACTIVE NODES</h3>";
  data += "<div class='value'>" + String(activeNodes) + "/" + String(NUM_PORTS) + "</div>";
  data += "</div>";

  data += "</div>";  // End system-stats

  // Network statistics
  data += "<h2 style='margin-top: 20px;'>NETWORK STATISTICS</h2>";
  data += "<div class='system-stats'>";

  // Collection Rate
  unsigned long uptime = millis() / 1000;  // in seconds
  float ratePerHour = uptime > 0 ? (totalNetworks * 3600.0 / uptime) : 0;

  data += "<div>";
  data += "<h3>COLLECTION RATE</h3>";
  data += "<div class='value'>" + String(ratePerHour, 1) + "/hr</div>";
  data += "</div>";

  // Total Networks
  data += "<div>";
  data += "<h3>TOTAL NETWORKS</h3>";
  data += "<div class='value'>" + String(totalNetworks) + "</div>";
  data += "</div>";

  // WiFi Networks
  int wifiNetworks = totalNetworks - channelNetworks[0];  // All except BLE

  data += "<div>";
  data += "<h3>WIFI NETWORKS</h3>";
  data += "<div class='value'>" + String(wifiNetworks) + "</div>";
  data += "</div>";

  // BLE Devices
  data += "<div>";
  data += "<h3>BLE DEVICES</h3>";
  data += "<div class='value'>" + String(channelNetworks[0]) + "</div>";
  data += "</div>";

  data += "</div>";  // End system-stats

  // Node communication status
  data += "<h2 style='margin-top: 20px;'>NODE COMMUNICATION</h2>";
  data += "<table style='width: 100%; border-collapse: collapse; margin-top: 15px;'>";
  data += "<tr style='border-bottom: 2px solid rgba(0, 255, 0, 0.3);'>";
  data += "<th style='text-align: left; padding: 8px;'>Node</th>";
  data += "<th style='text-align: center; padding: 8px;'>Status</th>";
  data += "<th style='text-align: right; padding: 8px;'>Networks</th>";
  data += "<th style='text-align: right; padding: 8px;'>Last Seen</th>";
  data += "</tr>";

  for (int i = 0; i < NUM_PORTS; i++) {
    String status;
    String statusColor;
    String lastSeenText;

    if (lastSeenSub[i] == 0) {
      status = "NEVER SEEN";
      statusColor = "color: #777;";
      lastSeenText = "Never";
    } else {
      unsigned long timeSince = millis() - lastSeenSub[i];

      if (timeSince < 30000) {
        status = "ONLINE";
        statusColor = "color: #0f0;";
      } else if (timeSince < 120000) {
        status = "IDLE";
        statusColor = "color: #ff0;";
      } else {
        status = "OFFLINE";
        statusColor = "color: #f00;";
      }

      // Format last seen time
      if (timeSince < 60000) {
        lastSeenText = String(timeSince / 1000) + " sec ago";
      } else if (timeSince < 3600000) {
        lastSeenText = String(timeSince / 60000) + " min ago";
      } else {
        lastSeenText = String(timeSince / 3600000) + " hr ago";
      }
    }

    data += "<tr style='border-bottom: 1px solid rgba(0, 255, 0, 0.1);'>";
    data += "<td style='padding: 8px;'>Node " + String(i) + "</td>";
    data += "<td style='text-align: center; padding: 8px; " + statusColor + "'>" + status + "</td>";
    data += "<td style='text-align: right; padding: 8px;'>" + String(networksSentBySub[i]) + "</td>";
    data += "<td style='text-align: right; padding: 8px;'>" + lastSeenText + "</td>";
    data += "</tr>";
  }

  data += "</table>";

  // Debug info
  data += "<h2 style='margin-top: 20px;'>DEBUG INFORMATION</h2>";
  data += "<div class='debug-log'>";
  data += "<p>I²C bus last reset: " + (lastI2CResetTime > 0 ? String((millis() - lastI2CResetTime) / 1000) + " seconds ago" : "Never") + "</p>";
  data += "<p>I²C error count: " + String(i2cErrorCount) + "</p>";
  data += "<p>SD card status: " + String(sdCardInitialized ? "Initialized" : "Failed") + "</p>";
  data += "<p>GPS fix status: " + String(gpsFixObtained ? "Obtained" : "Not available") + "</p>";
  data += "<p>Active log file: " + (sdCardInitialized ? fileName : "None") + "</p>";
  data += "<p>ESP32 SDK: " + String(ESP.getSdkVersion()) + "</p>";
  data += "<p>ESP32 chip revision: " + String(ESP.getChipRevision()) + "</p>";
  data += "<p>Flash size: " + String(ESP.getFlashChipSize() / 1024 / 1024) + " MB</p>";
  data += "</div>";

  data += "</div>";  // End panel

  server.send(200, "text/html", data);
}

void handleScanI2C() {
  String result = "<!DOCTYPE html><html><head><title>I2C BUS SCAN</title>";
  result += "<meta http-equiv='refresh' content='5;url=/' />";  // Redirect back after 5 seconds
  result += "<style>";
  result += "* {margin: 0; padding: 0; box-sizing: border-box; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;}";
  result += "body {background: #000; color: #0f0; padding: 20px;}";
  result += "h1, h2 {color: #0f0; text-shadow: 0 0 5px #0f0; text-transform: uppercase; letter-spacing: 2px; margin: 15px 0;}";
  result += "pre {background: rgba(0, 15, 0, 0.3); padding: 15px; border: 1px solid #0f0; font-family: monospace; margin-top: 20px; max-height: 400px; overflow-y: auto;}";
  result += ".address {color: #0af;}";
  result += ".found {color: #0f0; text-shadow: 0 0 3px #0f0;}";
  result += ".error {color: #f00;}";
  result += ".back {display: inline-block; margin-top: 20px; padding: 10px 15px; background: rgba(0, 30, 0, 0.8); color: #0f0; text-decoration: none; border: 1px solid #0f0; text-transform: uppercase; letter-spacing: 1px;}";
  result += ".back:hover {background: rgba(0, 40, 0, 0.9); box-shadow: 0 0 10px rgba(0, 255, 0, 0.5);}";
  result += ".scan-progress {display: block; width: 100%; height: 5px; background: rgba(0, 255, 0, 0.2); margin: 10px 0; position: relative; overflow: hidden;}";
  result += ".scan-progress::after {content: ''; position: absolute; top: 0; left: -50%; width: 50%; height: 100%; background: rgba(0, 255, 0, 0.5); animation: scan 2s linear infinite;}";
  result += "@keyframes scan {0% {left: -50%;} 100% {left: 100%;}}";
  result += ".scan-title {display: flex; align-items: center;}";
  result += ".scan-pulse {display: inline-block; width: 12px; height: 12px; background: #0f0; border-radius: 50%; margin-right: 10px; animation: pulse 1.5s ease-out infinite;}";
  result += "@keyframes pulse {0% {transform: scale(0.8); opacity: 1;} 100% {transform: scale(2); opacity: 0;}}";
  result += "</style>";
  result += "</head><body>";

  result += "<div class='scan-title'><span class='scan-pulse'></span><h1>I²C BUS SCAN</h1></div>";
  result += "<div class='scan-progress'></div>";

  // Perform the scan
  result += "<pre id='scanResults'>";

  // Test the multiplexer
  result += "[-] Testing TCA9548A multiplexer at address <span class='address'>0x" + String(TCAADDR, HEX) + "</span>... ";
  Wire.beginTransmission(TCAADDR);
  byte error = Wire.endTransmission();
  if (error == 0) {
    result += "<span class='found'>FOUND</span>\n";

    // Now test each port
    for (uint8_t port = 0; port < NUM_PORTS; port++) {
      result += "\n[+] Scanning port " + String(port) + ":\n";

      if (tcaselect(port)) {
        // Check for slave device at I2C_SLAVE_ADDRESS
        result += "[-] Checking for Hydra Sub at <span class='address'>0x" + String(I2C_SLAVE_ADDRESS, HEX) + "</span>... ";
        Wire.beginTransmission(I2C_SLAVE_ADDRESS);
        error = Wire.endTransmission();

        if (error == 0) {
          result += "<span class='found'>FOUND</span>\n";

          // Now scan for all other devices on this port
          for (byte address = 1; address < 127; address++) {
            if (address == TCAADDR || address == I2C_SLAVE_ADDRESS) continue;

            result += "[-] Address <span class='address'>0x";
            if (address < 16) result += "0";
            result += String(address, HEX) + "</span>... ";

            Wire.beginTransmission(address);
            error = Wire.endTransmission();

            if (error == 0) {
              result += "<span class='found'>FOUND</span>\n";
            } else {
              result += "\n";
            }
          }
        } else {
          result += "<span class='error'>NOT FOUND (error " + String(error) + ")</span>\n";
        }
      } else {
        result += "<span class='error'>Cannot access port " + String(port) + "</span>\n";
      }
    }
  } else {
    result += "<span class='error'>NOT FOUND (error " + String(error) + ")</span>\n";
    result += "\nTCA9548A multiplexer not found. This is a critical component for the Hydra system.\n";
    result += "Please check:\n";
    result += "1. I²C wiring connections\n";
    result += "2. Power to the multiplexer board\n";
    result += "3. Verify the correct I²C address (0x70)\n";
  }

  result += "\n[+] Scan complete. Returning to main page in 5 seconds.</pre>";
  result += "<a href='/' class='back'>RETURN TO COMMAND CENTER</a>";
  result += "</body></html>";

  server.send(200, "text/html", result);

  // Also run the normal scan function to output to Serial
  scanI2C();
}

void handleResetCounters() {
  // Reset network counters
  totalNetworks = 0;
  for (int i = 0; i < 15; i++) {
    channelNetworks[i] = 0;
  }

  for (int i = 0; i < NUM_PORTS; i++) {
    networksSentBySub[i] = 0;
  }

  // Show confirmation page
  String response = "<!DOCTYPE html><html><head><title>COUNTERS RESET</title>";
  response += "<meta http-equiv='refresh' content='3;url=/' />";
  response += "<style>";
  response += "* {margin: 0; padding: 0; box-sizing: border-box; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;}";
  response += "body {background: #000; color: #0f0; display: flex; align-items: center; justify-content: center; height: 100vh; flex-direction: column;}";
  response += "h1 {color: #0f0; text-shadow: 0 0 5px #0f0; text-transform: uppercase; letter-spacing: 2px; margin-bottom: 20px;}";
  response += ".reset-box {background: rgba(0, 20, 0, 0.4); border: 1px solid #0f0; padding: 30px; text-align: center; max-width: 500px; position: relative; box-shadow: 0 0 20px rgba(0, 255, 0, 0.2); overflow: hidden;}";
  response += ".reset-box::before {content: ''; position: absolute; top: 0; left: 0; width: 100%; height: 2px; background: linear-gradient(90deg, transparent, #0f0, transparent); animation: scan 2s linear infinite;}";
  response += "@keyframes scan {0% {transform: translateX(-100%);} 100% {transform: translateX(100%);}}";
  response += ".return-text {margin-top: 20px; opacity: 0.7;}";
  response += "</style>";
  response += "</head><body>";
  response += "<div class='reset-box'>";
  response += "<h1>SYSTEM RESET COMPLETE</h1>";
  response += "<p>All counters have been reset to zero.</p>";
  response += "<p class='return-text'>Returning to command center...</p>";
  response += "</div>";
  response += "</body></html>";

  server.send(200, "text/html", response);

  Serial.println("Network counters reset via web interface");
  blinkLED(CRGB::Blue, 3, 100);  // Visual feedback
}

void handleResetI2C() {
  // Reset the I2C bus
  resetI2CBus();

  // Show confirmation page
  String response = "<!DOCTYPE html><html><head><title>I2C BUS RESET</title>";
  response += "<meta http-equiv='refresh' content='3;url=/' />";
  response += "<style>";
  response += "* {margin: 0; padding: 0; box-sizing: border-box; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;}";
  response += "body {background: #000; color: #0f0; display: flex; align-items: center; justify-content: center; height: 100vh; flex-direction: column;}";
  response += "h1 {color: #0f0; text-shadow: 0 0 5px #0f0; text-transform: uppercase; letter-spacing: 2px; margin-bottom: 20px;}";
  response += ".reset-box {background: rgba(0, 20, 0, 0.4); border: 1px solid #0f0; padding: 30px; text-align: center; max-width: 500px; position: relative; box-shadow: 0 0 20px rgba(0, 255, 0, 0.2); overflow: hidden;}";
  response += ".reset-box::before {content: ''; position: absolute; top: 0; left: 0; width: 100%; height: 2px; background: linear-gradient(90deg, transparent, #0f0, transparent); animation: scan 2s linear infinite;}";
  response += "@keyframes scan {0% {transform: translateX(-100%);} 100% {transform: translateX(100%);}}";
  response += ".return-text {margin-top: 20px; opacity: 0.7;}";
  response += "</style>";
  response += "</head><body>";
  response += "<div class='reset-box'>";
  response += "<h1>I²C BUS RESET COMPLETE</h1>";
  response += "<p>The I²C bus has been reset. Communication with nodes should be restored.</p>";
  response += "<p class='return-text'>Returning to command center...</p>";
  response += "</div>";
  response += "</body></html>";

  server.send(200, "text/html", response);
}

void handleDownloadCSV() {
  if (!sdCardInitialized || fileName.length() == 0) {
    server.send(404, "text/plain", "ERROR: NO DATA FILE AVAILABLE");
    return;
  }

  File dataFile = SD.open(fileName);
  if (!dataFile) {
    server.send(404, "text/plain", "ERROR: FAILED TO ACCESS DATA FILE");
    return;
  }

  // Set appropriate headers for file download
  server.sendHeader("Content-Disposition", "attachment; filename=" + fileName.substring(1));
  server.sendHeader("Connection", "close");
  server.streamFile(dataFile, "text/csv");

  dataFile.close();
}

void handleNotFound() {
  String message = "<!DOCTYPE html><html><head><title>ERROR 404</title>";
  message += "<style>";
  message += "* {margin: 0; padding: 0; box-sizing: border-box; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;}";
  message += "body {background: #000; color: #f00; padding: 20px; text-align: center; display: flex; align-items: center; justify-content: center; height: 100vh; flex-direction: column;}";
  message += "h1 {color: #f00; text-shadow: 0 0 5px #f00; text-transform: uppercase; letter-spacing: 2px; margin-bottom: 20px;}";
  message += "pre {background: rgba(20, 0, 0, 0.4); border: 1px solid #f00; padding: 20px; text-align: left; max-width: 80%; margin: 20px auto; color: #f00;}";
  message += ".back {display: inline-block; margin-top: 20px; padding: 10px 15px; background: rgba(30, 0, 0, 0.8); color: #f00; text-decoration: none; border: 1px solid #f00; text-transform: uppercase; letter-spacing: 1px;}";
  message += ".back:hover {background: rgba(40, 0, 0, 0.9); box-shadow: 0 0 10px rgba(255, 0, 0, 0.5);}";
  message += "</style>";
  message += "</head><body>";
  message += "<h1>ERROR 404 - TARGET NOT FOUND</h1>";
  message += "<pre>";
  message += "REQUESTED URI: " + server.uri() + "\n";
  message += "METHOD: " + String((server.method() == HTTP_GET) ? "GET" : "POST") + "\n";
  message += "ARGUMENTS: " + String(server.args()) + "\n\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  message += "</pre>";
  message += "<a href='/' class='back'>RETURN TO COMMAND CENTER</a>";
  message += "</body></html>";

  server.send(404, "text/html", message);
}
#endif

void setup() {
  // Start time for uptime calculation
  startTime = millis();

  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);  // Allow time for serial monitor to connect

  Serial.println("\n\n=== M5Atom-Hydra DOM (v2.0) ===");
  Serial.println("Starting initialization sequence...");

  // Initialize LED
  Serial.println("Initializing LED...");
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);  // Medium brightness to save power
  setLED(CRGB::Blue);         // Blue during initialization

  // Initialize button
  Serial.println("Initializing button...");
  pinMode(BTN, INPUT_PULLUP);

  // Initialize I2C bus with custom function
  Serial.println("Initializing I2C bus...");
  initI2C();

#ifdef DOM_SERVER
  // Set up WiFi access point
  Serial.println("Setting up WiFi access point...");
  WiFi.softAP(DOM_SSID, DOM_PASS);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("AP started with SSID: " + String(DOM_SSID));
  Serial.println("AP IP Address: " + IP.toString());

  // Set up mDNS responder for easier access
  if (MDNS.begin("hydra")) {
    Serial.println("mDNS responder started - you can access at http://hydra.local");
  }

  // Set up web server routes
  Serial.println("Setting up web server...");
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/channel_data", handleChannelData);
  server.on("/networks", handleNetworks);
  server.on("/scan_i2c", handleScanI2C);
  server.on("/reset_counters", handleResetCounters);
  server.on("/reset_i2c", handleResetI2C);
  server.on("/download_csv", handleDownloadCSV);
  server.on("/system_status", handleSystemStatus);
  server.onNotFound(handleNotFound);

  // Start server
  server.begin();
  Serial.println("Web server started");
#endif

  // Initialize SPI for SD card
  Serial.println("Initializing SPI for SD card...");
  SPI.begin(SD_CLK, SD_MISO, SD_MOSI);

  // Initialize SD card with improved error handling
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS, SPI, 40000000)) {
    Serial.println("ERROR: SD Card initialization failed!");
    blinkLED(CRGB::Red, 5, 200);
  } else {
    Serial.println("SUCCESS: SD Card initialized");
    sdCardInitialized = true;
    blinkLED(CRGB::Green, 3, 200);
  }

  // Initialize GPS
  Serial.println("Initializing GPS module...");
  GPSSERIAL.begin(9600, SERIAL_8N1, GPS_RX, -1);

  // Wait for GPS fix
  waitForGPSFix();

  // Initialize file for logging
  if (sdCardInitialized) {
    Serial.println("Initializing log file...");
    initializeFile();
  }

  // Test I2C communication
  Serial.println("Testing I2C communication with subs...");
  int connectedSubs = 0;
  for (uint8_t port = 0; port < NUM_PORTS; port++) {
    if (tcaselect(port)) {
      Wire.beginTransmission(I2C_SLAVE_ADDRESS);
      byte error = Wire.endTransmission();
      if (error == 0) {
        connectedSubs++;
        Serial.println("- Sub found on port " + String(port));
        blinkLED(CRGB::Green, 1, 50);
      }
    }
    delay(50);  // Small delay between ports
  }

  Serial.println("Found " + String(connectedSubs) + " connected subs out of " + String(NUM_PORTS));

  // Final initialization message
  Serial.println("Setup complete. Starting main loop...");
  Serial.println("========================================");

  // Startup sequence complete
  blinkLED(CRGB::Green, 5, 100);
}

void loop() {
  // Process any GPS data
  while (GPSSERIAL.available() > 0) {
    gps.encode(GPSSERIAL.read());
  }

  // Check button for user interaction
  checkButton();
  buttonPressed = false;  // Reset for next loop

  // Non-blocking network fetch from subs
  if (millis() - lastNetworkFetchTime >= NETWORK_FETCH_INTERVAL) {
    lastNetworkFetchTime = millis();

    // Try to poll multiple ports per interval instead of just one
    // This helps ensure all nodes get a chance to report
    for (int attempt = 0; attempt < 3; attempt++) {  // Try 3 ports per cycle
      static uint8_t currentPort = 0;

      if (tcaselect(currentPort)) {
        if (requestNetworkData(currentPort)) {
          // Log data if valid
          logData(receivedNetworks[currentPort], currentPort);
        }
      }

      // Move to next port
      currentPort = (currentPort + 1) % NUM_PORTS;
    }
  }

  // Periodic I2C bus health check
  if (millis() - lastI2CResetTime > I2C_RESET_INTERVAL) {
    if (i2cErrorCount > 50) {
      Serial.println("High I2C error count detected (" + String(i2cErrorCount) + "), resetting bus");
      resetI2CBus();
      i2cErrorCount = 0;
    }

    // Update I2C health status
    i2cHealthy = (i2cErrorCount < 10);
  }

  // Periodic status update
  if (millis() - lastStatusUpdateTime >= STATUS_UPDATE_INTERVAL) {
    lastStatusUpdateTime = millis();

    // Count active nodes
    int activeNodes = 0;
    for (int i = 0; i < NUM_PORTS; i++) {
      if (isPortActive(i)) activeNodes++;
    }

    // Log status
    Serial.println("Status update - Runtime: " + getUptimeString());
    Serial.println("- Total networks: " + String(totalNetworks));
    Serial.println("- Active nodes: " + String(activeNodes) + "/" + String(NUM_PORTS));
    Serial.println("- I2C errors: " + String(i2cErrorCount));
    Serial.println("- Free memory: " + String(ESP.getFreeHeap() / 1024) + " KB");
  }

#ifdef DOM_SERVER
  // Handle any incoming client requests
  server.handleClient();
#endif
}