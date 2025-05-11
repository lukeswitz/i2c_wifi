/*
   M5Atom-Hydra (Dom)
   Optimized for M5Atom Lite 
*/

// CHOOSE COMMUNICATION
#define COMM_I2C
//#define COMM_NOW

// CHOOSE WEBSERVER
#define DomServer

// CHOOSE HARDWARE - Using Atom Lite for Dom
#define ATOMLITE

#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <FastLED.h>

#ifdef DomServer
#include <WebServer.h>
#define DOM_SSID "HydraNode"
#define DOM_PASS "12345678"
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

// I2C multiplexer address
#define TCAADDR 0x70
#define NUM_PORTS 6  // Support for 6 subs
const int i2c_slave_address = 0x55;

// Single LED for Atom Lite
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
int channelNetworks[15] = {0};  // Count per channel (0 = BLE, 1-14 = WiFi channels)
bool gpsFixObtained = false;
bool sdCardInitialized = false;

// For tracking sub activity
unsigned long lastSeenSub[NUM_PORTS] = {0};
int networksSentBySub[NUM_PORTS] = {0};
NetworkInfo receivedNetworks[NUM_PORTS];

// For button management
bool buttonPressed = false;
int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// LED colors
CRGB statusColors[] = {
  CRGB::Blue,   // Idle/Starting
  CRGB::Green,  // Good/Success
  CRGB::Red,    // Error
  CRGB::Purple, // Bluetooth activity
  CRGB::Yellow, // Warning
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

#ifdef DomServer
void handleRoot();
void handleData();
void handleScanI2C();
void handleResetCounters();
void handleDownloadCSV();
#endif

// Select TCA multiplexer channel
bool tcaselect(uint8_t i) {
  if (i >= NUM_PORTS) return false;
  
  // Serial.print("[MASTER] Switching to I2C port: " + String(i) + "... ");
  
  unsigned long startTime = millis();
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  byte error = Wire.endTransmission();
  
  // Check if this operation took too long
  if (millis() - startTime > 300) {
    Serial.println("TIMEOUT!");
    // Reset the I2C bus
    Wire.end();
    Wire.begin(SUB_SDA, SUB_SCL);
    delay(10);
    return false;
  }
  
  if (error == 0) {
    Serial.println("SUCCESS");
    return true;
  } else {
    Serial.println("ERROR: " + String(error));
    return false;
  }
}

// Request network data from a sub via I2C
bool requestNetworkData(uint8_t port) {
  // Set LED to indicate we're requesting data
  // setLED(statusColors[5]); // White
  
  Wire.requestFrom(i2c_slave_address, sizeof(NetworkInfo));
  delay(10); // Small delay to allow for response
  
  if (Wire.available() < sizeof(NetworkInfo)) {
    setLED(CRGB::Black);
    return false;
  }

  Wire.readBytes((byte*)&receivedNetworks[port], sizeof(NetworkInfo));

  // Basic validation - check if channel is valid
  if (receivedNetworks[port].channel > 14) {
    setLED(CRGB::Black);
    return false;
  }

  // LED pattern based on channel
  if (receivedNetworks[port].type == 'b') {
    blinkLED(statusColors[3], 1, 50); // Purple for BLE
  } else if (receivedNetworks[port].channel <= 5) {
    blinkLED(statusColors[0], 1, 50); // Blue for channels 1-5
  } else if (receivedNetworks[port].channel <= 10) {
    blinkLED(statusColors[1], 1, 50); // Green for channels 6-10
  } else {
    blinkLED(statusColors[4], 1, 50); // Yellow for channels 11-14
  }

  // Update sub status
  lastSeenSub[port] = millis();
  networksSentBySub[port]++;
  
  return true;
}

// Scan for I2C devices (for debugging)
void scanI2C() {
  byte error, address;
  int deviceCount = 0;
  unsigned long scanStart = millis();
  
  Serial.println("Scanning I2C bus...");
  
  // Test for the TCA multiplexer first
  Serial.print("Testing TCA9548A multiplexer at address 0x");
  Serial.print(TCAADDR, HEX);
  Serial.print("... ");
  
  unsigned long startTime = millis();
  Wire.beginTransmission(TCAADDR);
  error = Wire.endTransmission();
  
  if (millis() - startTime > 100) {
    Serial.println("TIMEOUT!");
    Serial.println("I2C bus may be locked. Attempting recovery...");
    
    // Recovery sequence
    Wire.end();
    pinMode(SUB_SDA, INPUT_PULLUP);
    pinMode(SUB_SCL, INPUT_PULLUP);
    delay(100);
    
    // Toggle SCL to release stuck devices
    pinMode(SUB_SCL, OUTPUT);
    for (int i = 0; i < 10; i++) {
      digitalWrite(SUB_SCL, HIGH);
      delayMicroseconds(5);
      digitalWrite(SUB_SCL, LOW);
      delayMicroseconds(5);
    }
    digitalWrite(SUB_SCL, HIGH);
    
    // Reinitialize I2C
    Wire.begin(SUB_SDA, SUB_SCL);
    delay(100);
    Serial.println("I2C recovery attempt complete. Continuing scan...");
  } else {
    if (error == 0) {
      Serial.println("FOUND!");
      deviceCount++;
    } else {
      Serial.print("NOT FOUND (error ");
      Serial.print(error);
      Serial.println(")");
    }
  }
  
  // Now scan the rest of the bus
  for (address = 1; address < 127; address++) {
    // Skip the TCA address since we already tested it
    if (address == TCAADDR) continue;
    
    // Add a timeout check for the whole scan
    if (millis() - scanStart > 5000) {
      Serial.println("Scan taking too long! Aborting...");
      break;
    }
    
    Serial.print("Testing address 0x");
    if (address < 16) Serial.print("0");
    Serial.print(address, HEX);
    Serial.print("... ");
    
    startTime = millis();
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    // Check if this operation took too long
    if (millis() - startTime > 100) {
      Serial.println("TIMEOUT!");
      continue;
    }
    
    if (error == 0) {
      Serial.println("FOUND!");
      deviceCount++;
    } else {
      Serial.println("");
    }
    
    // Brief delay between scans
    delay(1);
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

// Wait for GPS fix
void waitForGPSFix() {
  Serial.println("Waiting for GPS fix...");
  unsigned long startTime = millis();
  const unsigned long gpsTimeout = 180000; // 3 minutes timeout
  
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
    
    // Check button for early bypass
    checkButton();
    if (buttonPressed) {
      Serial.println("Button pressed, skipping GPS wait");
      break;
    }
    
    delay(10);
  }
  
  if (gps.location.isValid()) {
    gpsFixObtained = true;
    Serial.println("GPS fix obtained!");
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
    Serial.println("GPS fix not obtained. Continuing without GPS.");
    // Show warning
    blinkLED(CRGB::Yellow, 5, 100);
  }
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
    strcpy(fileDateStamp, "0000-00-00-");
  }
  
  // Find an available filename
  do {
    fileName = "/wifi-scans-" + String(fileDateStamp) + String(fileNumber) + ".csv";
    isNewFile = !SD.exists(fileName);
    fileNumber++;
  } while (!isNewFile);
  
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
  }
}

// Log network data to SD card
void logData(const NetworkInfo& network, uint8_t port) {
  // Skip logging our own WiFi AP if present
  if (strcmp("HydraNode", network.ssid) == 0 || strcmp("PicoHydra", network.ssid) == 0 || strcmp("OledHydra", network.ssid) == 0) {
    return;
  }
  
  if (gpsFixObtained && sdCardInitialized) {
    // Format timestamp
    String utc = String(gps.date.year()) + "-" + 
                 String(gps.date.month()) + "-" + 
                 String(gps.date.day()) + " " + 
                 String(gps.time.hour()) + ":" + 
                 String(gps.time.minute()) + ":" + 
                 String(gps.time.second());
    
    // Create CSV row
    String dataString = String(network.bssid) + "," + 
                       "\"" + network.ssid + "\"" + "," + 
                       network.security + "," + 
                       utc + "," + 
                       String(network.channel) + "," + 
                       String(network.rssi) + "," + 
                       String(gps.location.lat(), 6) + "," + 
                       String(gps.location.lng(), 6) + "," + 
                       String(gps.altitude.meters(), 2) + "," + 
                       String(gps.hdop.hdop(), 2);
    
    // Add network type
    if (network.type == 'w') {
      dataString += ",WIFI";
    } else if (network.type == 'b') {
      dataString += ",BLE";
    }
    
    // Write to SD card
    File dataFile = SD.open(fileName, FILE_APPEND);
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      
      // Update counters
      totalNetworks++;
      channelNetworks[network.channel]++;
      
      Serial.println("Data written: " + dataString);
      blinkLED(CRGB::Green, 1, 20);
    } else {
      Serial.println("Error writing to " + fileName);
      blinkLED(CRGB::Red, 1, 50);
    }
  } else {
    // Cannot log - either no GPS fix or SD card issue
    blinkLED(CRGB::Yellow, 1, 50);
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

#ifdef DomServer
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
  html += "</style>";
  
  // Add JavaScript for auto refresh and effects
  html += "<script>";
  html += "function updateData() {";
  html += "  fetch('/data').then(response => response.text()).then(data => {";
  html += "    document.getElementById('data-container').innerHTML = data;";
  html += "  });";
  html += "}";
  html += "setInterval(updateData, 5000);"; // Refresh every 5 seconds
  html += "</script>";
  
  html += "</head><body>";
  
  // Header withstyling
  html += "<div class='header'>";
  html += "<div class='wrapper'>";
  html += "<h1>HYDRA NETWORK SCANNER</h1>";
  html += "<p>Wireless Intelligence Collection System</p>";
  html += "<div class='version'>v1.0-ATOM</div>";
  html += "</div>";
  html += "</div>";
  
  html += "<div class='wrapper'>";
  
  // Main content container that will be updated via JavaScript
  html += "<div class='content'>";
  html += "<div id='data-container'>Loading data...</div>";
  
  // Control panel
  html += "<div class='panel'>";
  html += "<h2>CONTROLS</h2>";
  html += "<a href='/scan_i2c' class='btn btn-scan'>SCAN I²C BUS</a> ";
  html += "<a href='/reset_counters' class='btn btn-reset'>RESET COUNTERS</a> ";
  html += "<a href='/download_csv' class='btn'>EXPORT DATA</a>";
  html += "</div>";
  
  // Server information section with IP address
  html += "<div class='server-info'>";
  html += "SERVER: HYDRA COMMAND NODE | IP: <a href='http://192.168.4.1'>192.168.4.1</a> | SSID: " + String(DOM_SSID);
  html += "</div>";
  
  html += "</div>";
  html += "</div>";
  
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
  
  // Active File
  if (sdCardInitialized) {
    data += "<div>LOG: " + fileName.substring(1) + "</div>";
  }
  
  data += "</div>"; // End scanner-status
  
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
    data += "</div>"; // End status-grid
  }
  
  // Total network counter with blinking effect
  data += "<div class='total-counter'>";
  data += "<span class='scan-animation'></span> NETWORKS CAPTURED: " + String(totalNetworks);
  data += "</div>";
  
  data += "</div>"; // End panel
  
  // Channel distribution visualizer
  data += "<div class='panel'>";
  data += "<h2>CHANNEL INTELLIGENCE</h2>";
  data += "<div class='chart'>";
  
  // Find the maximum value for scaling
  int maxNetworks = 1; // Prevent division by zero
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
  
  for (int i = 0; i < 15; i++) {
    int height = channelNetworks[i] > 0 ? (channelNetworks[i] * 100 / maxNetworks) : 1;
    if (height < 1) height = 1; // Ensure bar is visible
    
    data += "<div class='bar " + String(i == 0 ? "ble-bar" : "") + "' style='height: " + String(height) + "%; left: " + String(i * 6.5) + "%;'>";
    data += "<span>" + String(channelNetworks[i]) + "</span>";
    data += "</div>";
    
    // Add channel label under each bar - FIX: Make sure both operands are strings
    data += "<div class='chart-label' style='left: " + String(i * 6.5 + 2.25) + "%;'>";
    data += i == 0 ? String("BLE") : String(i);
    data += "</div>";
  }
  
  data += "</div>"; // End chart
  data += "</div>"; // End panel
  
  data += "<div class='panel'>";
  data += "<h2>NODE STATUS</h2>";
  data += "<div class='status-grid'>";
  
  unsigned long currentTime = millis();
  for (int i = 0; i < NUM_PORTS; i++) {
    String nodeClass = "never";
    String nodeStatus = "DISCONNECTED";
    String timeSinceStr = "NEVER SEEN";
    
    if (lastSeenSub[i] > 0) {
      // Calculate time since last seen
      unsigned long timeSince = currentTime - lastSeenSub[i];
      
      if (timeSince < 30000) { // Within 30 seconds
        nodeClass = "online";
        nodeStatus = "ONLINE";
      } else if (timeSince < 120000) { // Within 2 minutes
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
  
  data += "</div>"; // End status-grid
  data += "</div>"; // End panel
  
  server.send(200, "text/html", data);
}

void handleScanI2C() {
  String result = "<!DOCTYPE html><html><head><title>I2C BUS SCAN</title>";
  result += "<meta http-equiv='refresh' content='5;url=/' />"; // Redirect back after 5 seconds
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
  } else {
    result += "<span class='error'>NOT FOUND (error " + String(error) + ")</span>\n";
  }
  
  // Scan all other addresses
  for (byte address = 1; address < 127; address++) {
    if (address == TCAADDR) continue; // Skip the TCA address we already tested
    
    result += "[-] Testing address <span class='address'>0x";
    if (address < 16) result += "0";
    result += String(address, HEX) + "</span>... ";
    
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      result += "<span class='found'>FOUND</span>\n";
    } else {
      result += "not found\n";
    }
    
    // Small delay to prevent browser rendering issues with very long text
    if (address % 20 == 0) {
      result += "\n"; // Add line break every 20 addresses
    }
  }
  
  result += "\n[+] Scan complete. Execute `scanI2C()` in serial console for detailed results.</pre>";
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
  blinkLED(CRGB::Blue, 3, 100); // Visual feedback
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
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting AtomHydra Dom (Atom Lite version)...");
  
  // Initialize LED
  FastLED.addLeds<WS2812, LED_PIN, GRB>(&leds[0], NUM_LEDS);
  setLED(CRGB::Blue);  // Blue during initialization
  
  // Initialize button
  pinMode(BTN, INPUT);
  
  // Reset I2C pins to known state first
  pinMode(SUB_SDA, OUTPUT);
  pinMode(SUB_SCL, OUTPUT);
  digitalWrite(SUB_SDA, HIGH);
  digitalWrite(SUB_SCL, HIGH);
  delay(10);
  
  // Now initialize I2C as master
  Wire.begin(SUB_SDA, SUB_SCL);
  Serial.println("[MASTER] I2C Master Initialized");
  
  #ifdef DomServer
  // Set up WiFi access point
  WiFi.softAP(DOM_SSID, DOM_PASS);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("AP IP Address: " + IP.toString());
  
  // Set up web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/scan_i2c", handleScanI2C);
  server.on("/reset_counters", handleResetCounters);
  server.on("/download_csv", handleDownloadCSV);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Web server started");
  Serial.println("Connect to http://192.168.4.1 to view the dashboard");
  #endif
  
  // Initialize SPI for SD card
  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, -1);
  
  // Initialize SD card
  Serial.println("Initializing SD card...");
  if (!SD.begin(15, SPI, 40000000)) {
    Serial.println("SD Card initialization failed!");
    blinkLED(CRGB::Red, 5, 200);
  } else {
    Serial.println("SD Card initialized successfully.");
    sdCardInitialized = true;
    blinkLED(CRGB::Green, 3, 200);
  }
  
  // Initialize GPS
  GPSSERIAL.begin(9600, SERIAL_8N1, GPS_RX, -1);
  
  // Wait for GPS fix
  waitForGPSFix();
  
  // Initialize file for logging if GPS fix was obtained
  if (gpsFixObtained && sdCardInitialized) {
    initializeFile();
  }
  
  Serial.println("Setup complete. Starting main loop...");
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
  
  // Scan I2C ports for data from subs
  for (uint8_t port = 0; port < NUM_PORTS; port++) {
    if (tcaselect(port)) {
      delay(5);  // Small delay for stability
      
      if (requestNetworkData(port)) {
        // If we got valid data, log it
        logData(receivedNetworks[port], port);
      }
    }
  }
  
  #ifdef DomServer
  // Handle any incoming client requests
  server.handleClient();
  #endif
  
  // Brief status indicator based on total networks
  if (totalNetworks > 0 && (millis() % 10000) < 100) {  // Every 10 seconds, briefly
    // Flash LED based on total networks - green for >100, blue for <100
    if (totalNetworks > 100) {
      blinkLED(CRGB::Green, 2, 100);
    } else {
      blinkLED(CRGB::Blue, 2, 100);
    }
    
    // Print periodic stats
    Serial.println("Status update - Total networks: " + String(totalNetworks));
    
    // Check for inactive subs (no activity for 2 minutes)
    for (int i = 0; i < NUM_PORTS; i++) {
      if (lastSeenSub[i] > 0 && millis() - lastSeenSub[i] > 120000) {
        Serial.println("Warning: Sub " + String(i) + " appears to be inactive");
      }
    }
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}