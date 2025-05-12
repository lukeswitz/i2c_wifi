/*
   M5Atom-Hydra (Sub)
   Optimized for M5Atom Matrix hardware with enhanced LED display
*/

// CHOOSE COMMUNICATION
#define COMM_I2C
//#define COMM_NOW

// CHOOSE HARDWARE
// #define ATOMLITE
#define MATRIX
//#define S3LITE

// CHOOSE NODE ID (1-6)
#define NODEID 1

// Node Plot
//     1---2
//    /     \
//   6       3
//    \     /
//     5---4

// Channel distribution for 6 subs
#if NODEID==1
const int channels[] = {1, 2};
#elif NODEID==2
const int channels[] = {3, 4};
#elif NODEID==3
const int channels[] = {5, 6};
#elif NODEID==4
const int channels[] = {7, 8};
#elif NODEID==5
const int channels[] = {9, 10, 11};
#elif NODEID==6
const int channels[] = {12, 13, 14};
#define enableBLE  // Only node 6 handles BLE to avoid duplication
#endif

#include <WiFi.h>
#include <FastLED.h>

#ifdef COMM_I2C
#include <Wire.h>
#endif

#ifdef COMM_NOW
#include <esp_now.h>
#endif

// Pin definitions for each hardware type
#ifdef ATOMLITE
#define LED_PIN 27
#define SUB_SDA 26
#define SUB_SCL 32
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];
#endif

#ifdef MATRIX
#define LED_PIN 27
#define SUB_SDA 26
#define SUB_SCL 32
#define NUM_LEDS 25
CRGB leds[NUM_LEDS];
// Define matrix indices for easy access (5x5 grid)
int matrix[5][5] = {
  {0, 1, 2, 3, 4},
  {5, 6, 7, 8, 9},
  {10, 11, 12, 13, 14},
  {15, 16, 17, 18, 19},
  {20, 21, 22, 23, 24}
};
#endif

#ifdef S3LITE
#include <M5AtomS3.h>
#define SUB_SDA 2
#define SUB_SCL 1
#endif

// Constants for network scanning
#define MAX_NETWORKS 300
#define MAX_MAC_HISTORY 400

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

// Network scanning variables
NetworkInfo networks[MAX_NETWORKS];
int networkCount = 0;
int currentNetworkIndex = 0;
volatile bool scan = false;

// MAC address tracking
String macAddressArray[MAX_MAC_HISTORY];
int macArrayIndex = 0;
bool overFlow = false;

// Timing variables for adaptive scanning
const int FEW_NETWORKS_THRESHOLD = 1;
const int MANY_NETWORKS_THRESHOLD = 8;
const int POP_INC = 75;   // Higher increment for popular channels
const int STD_INC = 50;   // Standard increment
const int RARE_INC = 30;  // Lower increment for rare channels
const int MAX_TIME = 500;
const int MIN_TIME = 50;

// Initial time per channel (will be adjusted dynamically)
int timePerChannel[15] = { 50, 300, 200, 200, 200, 200, 300, 200, 200, 200, 200, 300, 200, 200, 200 };
int incrementPerChannel[15] = {0, POP_INC, STD_INC, STD_INC, STD_INC, STD_INC, POP_INC, STD_INC, STD_INC, STD_INC, STD_INC, POP_INC, RARE_INC, RARE_INC, RARE_INC};

// Performance optimization variables
unsigned long lastScanTime = 0;
const int scanDelay = 20; // Small delay between scanning attempts
int channelIndex = 0;
const int channelCount = sizeof(channels) / sizeof(channels[0]);
bool isScanning = false;

// Animation tracking variables
int animFrame = 0;
unsigned long lastAnimUpdate = 0;
unsigned long lastCountDisplay = 0;
int totalNetworksFound = 0;

#ifdef COMM_I2C
const int i2c_slave_address = 0x55;
#endif

#ifdef enableBLE
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

BLEScan* pBLEScan;

// BLE MAC address tracking
struct mac_addr {
  unsigned char bytes[6];
};

#define mac_history_len 256
struct mac_addr mac_history[mac_history_len];
unsigned int mac_history_cursor = 0;

// BLE helper functions
void save_mac(unsigned char* mac) {
  if (mac_history_cursor >= mac_history_len) {
    mac_history_cursor = 0;
  }
  struct mac_addr tmp;
  for (int x = 0; x < 6; x++) {
    tmp.bytes[x] = mac[x];
  }

  mac_history[mac_history_cursor] = tmp;
  mac_history_cursor++;
}

boolean seen_mac(unsigned char* mac) {
  struct mac_addr tmp;
  for (int x = 0; x < 6; x++) {
    tmp.bytes[x] = mac[x];
  }

  for (int x = 0; x < mac_history_len; x++) {
    if (mac_cmp(tmp, mac_history[x])) {
      return true;
    }
  }
  return false;
}

void print_mac(struct mac_addr mac) {
  for (int x = 0; x < 6; x++) {
    Serial.print(mac.bytes[x], HEX);
    Serial.print(":");
  }
}

boolean mac_cmp(struct mac_addr addr1, struct mac_addr addr2) {
  for (int y = 0; y < 6; y++) {
    if (addr1.bytes[y] != addr2.bytes[y]) {
      return false;
    }
  }
  return true;
}

void clear_mac_history() {
  struct mac_addr tmp;
  for (int x = 0; x < 6; x++) {
    tmp.bytes[x] = 0;
  }

  for (int x = 0; x < mac_history_len; x++) {
    mac_history[x] = tmp;
  }

  mac_history_cursor = 0;
}

// BLE device callback
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      unsigned char mac_bytes[6];
      int values[6];

      if (6 == sscanf(advertisedDevice.getAddress().toString().c_str(), "%x:%x:%x:%x:%x:%x%*c", &values[0], &values[1], &values[2], &values[3], &values[4], &values[5])) {
        for (int i = 0; i < 6; ++i ) {
          mac_bytes[i] = (unsigned char) values[i];
        }

        if (!seen_mac(mac_bytes)) {
          save_mac(mac_bytes);
          addBleNetwork(advertisedDevice.getName().c_str(), advertisedDevice.getAddress().toString().c_str(), advertisedDevice.getRSSI());
        }
      }
    }
};
#endif

// Handle LED functions based on board type
void setLed(CRGB c) {
#ifdef S3LITE
  AtomS3.dis.drawpix(0, c.r, c.g, c.b);
#else
  #ifdef MATRIX
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = c;
  }
  #else
  leds[0] = c;
  #endif
  FastLED.show();
#endif
}

void clearLed() {
#ifdef S3LITE
  AtomS3.dis.drawpix(0, 0, 0, 0);
#else
  #ifdef MATRIX
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  #else
  leds[0] = CRGB::Black;
  #endif
  FastLED.show();
#endif
}

#ifdef MATRIX
// Function to display a specific digit (0-9) at an offset position
void displayDigit(int digit, int offsetX, int offsetY, CRGB color) {
  // Patterns for 0-9 digits (3x5 pixel format)
  const uint8_t digitPatterns[10][5] = {
    // 0
    {0b111,
     0b101,
     0b101,
     0b101,
     0b111},
    // 1
    {0b010,
     0b110,
     0b010,
     0b010,
     0b111},
    // 2
    {0b111,
     0b001,
     0b111,
     0b100,
     0b111},
    // 3
    {0b111,
     0b001,
     0b111,
     0b001,
     0b111},
    // 4
    {0b101,
     0b101,
     0b111,
     0b001,
     0b001},
    // 5
    {0b111,
     0b100,
     0b111,
     0b001,
     0b111},
    // 6
    {0b111,
     0b100,
     0b111,
     0b101,
     0b111},
    // 7
    {0b111,
     0b001,
     0b001,
     0b001,
     0b001},
    // 8
    {0b111,
     0b101,
     0b111,
     0b101,
     0b111},
    // 9
    {0b111,
     0b101,
     0b111,
     0b001,
     0b111}
  };

  // Display the digit pattern on the matrix
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 3; x++) {
      if (digitPatterns[digit][y] & (1 << (2 - x))) {
        // Calculate position with offset
        int matrixX = x + 1 + offsetX;
        int matrixY = y + offsetY;
        
        // Only set LED if within bounds
        if (matrixX >= 0 && matrixX < 5 && matrixY >= 0 && matrixY < 5) {
          leds[matrix[matrixY][matrixX]] = color;
        }
      }
    }
  }
}

// Display a number (0-99) on the matrix
void displayNumber(int number) {
  clearLed();
  
  // Cap at 99 for display purposes
  if (number > 99) number = 99;
  
  if (number < 10) {
    // Single digit - center it
    displayDigit(number, 1, 0, CRGB::Green);
  } else {
    // Two digits - side by side
    int tens = number / 10;
    int ones = number % 10;
    displayDigit(tens, -1, 0, CRGB::Blue);
    displayDigit(ones, 2, 0, CRGB::Green);
  }
  
  FastLED.show();
}

// Scanning animation - shows a radar-like pattern
void scanningAnimation() {
  clearLed();
  
  // Center point always on
  leds[matrix[2][2]] = CRGB(0, 50, 0);
  
  // Positions for the scanning beam (8 positions in a circle)
  int positions[8][2] = {
    {2, 0}, // Top
    {4, 0}, // Top-right
    {4, 2}, // Right
    {4, 4}, // Bottom-right
    {2, 4}, // Bottom
    {0, 4}, // Bottom-left
    {0, 2}, // Left
    {0, 0}  // Top-left
  };
  
  // Get current position
  int frame = animFrame % 8;
  int x = positions[frame][0];
  int y = positions[frame][1];
  
  // Draw beam
  leds[matrix[y][x]] = CRGB::Green;
  
  // Draw path from center to current position
  int cx = 2, cy = 2; // Center coordinates
  int dx = (x > cx) ? 1 : ((x < cx) ? -1 : 0);
  int dy = (y > cy) ? 1 : ((y < cy) ? -1 : 0);
  
  int ix = cx, iy = cy;
  while (ix != x || iy != y) {
    if (ix != x) ix += dx;
    if (iy != y) iy += dy;
    
    // Skip center as we already set it
    if (!(ix == cx && iy == cy)) {
      leds[matrix[iy][ix]] = CRGB(0, 20, 0);
    }
  }
  
  // Increment animation frame for next time
  animFrame = (animFrame + 1) % 8;
  
  FastLED.show();
}

// Show channel distribution on matrix
void displayChannelMap() {
  clearLed();
  
  // Color channels based on range
  for (int i = 0; i < channelCount; i++) {
    int ch = channels[i];
    CRGB channelColor;
    
    // Set color based on channel range
    if (ch <= 5) {
      channelColor = CRGB::Blue;
    } else if (ch <= 10) {
      channelColor = CRGB::Green;
    } else {
      channelColor = CRGB::Yellow;
    }
    
    // Map channel numbers logically to the matrix
    int x, y;
    if (ch <= 5) {
      x = ch - 1;
      y = 0;
    } else if (ch <= 10) {
      x = ch - 6;
      y = 1;
    } else {
      x = ch - 11;
      y = 2;
    }
    
    // Make sure we're within bounds
    if (x >= 0 && x < 5 && y >= 0 && y < 5) {
      leds[matrix[y][x]] = channelColor;
    }
  }
  
  // Show node ID in corner
  for (int i = 0; i < min(NODEID, 5); i++) {
    leds[matrix[4][i]] = CRGB::Red;
  }
  
  // Show BLE indicator if enabled
  #ifdef enableBLE
  leds[matrix[0][4]] = CRGB::Purple;
  #endif
  
  FastLED.show();
}

// Show a progress bar visualization
void displayProgressBar(int percent) {
  clearLed();
  
  int ledsToLight = map(percent, 0, 100, 0, 25);
  
  // Fill the matrix progressively (left-to-right, top-to-bottom)
  for (int i = 0; i < ledsToLight; i++) {
    int y = i / 5;
    int x = i % 5;
    
    // Create color gradient based on percentage
    CRGB color;
    if (percent < 33) {
      color = CRGB::Green;
    } else if (percent < 66) {
      color = CRGB::Yellow;
    } else {
      color = CRGB::Red;
    }
    
    leds[matrix[y][x]] = color;
  }
  
  FastLED.show();
}

// Display WiFi symbol to indicate scanning
void displayWifiSymbol() {
  clearLed();
  
  // Create a WiFi-like arc symbol (3 arcs)
  // Bottom arc
  leds[matrix[4][2]] = CRGB::Blue; // Center bottom
  
  // First arc (smallest)
  leds[matrix[3][1]] = CRGB::Blue;
  leds[matrix[3][2]] = CRGB::Blue;
  leds[matrix[3][3]] = CRGB::Blue;
  
  // Second arc (medium)
  leds[matrix[2][0]] = CRGB::Blue;
  leds[matrix[2][1]] = CRGB::Blue;
  leds[matrix[2][2]] = CRGB::Blue;
  leds[matrix[2][3]] = CRGB::Blue;
  leds[matrix[2][4]] = CRGB::Blue;
  
  // Third arc (largest)
  leds[matrix[1][0]] = CRGB::Blue;
  leds[matrix[1][1]] = CRGB::Blue;
  leds[matrix[1][2]] = CRGB::Blue;
  leds[matrix[1][3]] = CRGB::Blue;
  leds[matrix[1][4]] = CRGB::Blue;
  
  // Add some animation based on time
  static uint8_t brightness = 150;
  static bool increasing = true;
  
  // Create pulsing effect
  if (increasing) {
    brightness += 5;
    if (brightness >= 250) increasing = false;
  } else {
    brightness -= 5;
    if (brightness <= 100) increasing = true;
  }
  
  // Apply brightness
  for (int i = 0; i < NUM_LEDS; i++) {
    if (leds[i].r > 0 || leds[i].g > 0 || leds[i].b > 0) {
      leds[i].nscale8(brightness);
    }
  }
  
  FastLED.show();
}
#endif

void blinkLED(CRGB color) {
  setLed(color);
  delay(50);
  clearLed();
}

void blinkLEDWhite() { blinkLED(CRGB::White); }
void blinkLEDGreen() { blinkLED(CRGB::Green); }
void blinkLEDBlue() { blinkLED(CRGB::Blue); }
void blinkLEDRed() { blinkLED(CRGB::Red); }
void blinkLEDPurple() { blinkLED(CRGB::Purple); }
void blinkLEDYellow() { blinkLED(CRGB::Yellow); }

// Check if a MAC address is already seen
bool isMACSeen(const String& mac) {
  for (int i = 0; i < (overFlow ? MAX_MAC_HISTORY : macArrayIndex); i++) {
    if (macAddressArray[i] == mac) {
      return true;
    }
  }
  return false;
}

#ifdef COMM_I2C
void requestEvent() {
  if (!scan) {
    // Only start scanning if dom is ready
    scan = true;
    blinkLEDGreen();
  }
  
  if (currentNetworkIndex < networkCount) {
    Wire.write((byte*)&networks[currentNetworkIndex], sizeof(NetworkInfo));
    Serial.println("[SUB] Sending network: " + String(networks[currentNetworkIndex].ssid));
    currentNetworkIndex++;
  } else {
    // If we have nothing to send, start scanning again
    Serial.println("[SUB] No new networks to send");
    currentNetworkIndex = 0;
    networkCount = 0;
  }
}
#endif

// Add a BLE network to our list
void addBleNetwork(const String& ssid, const String& bssid, int32_t rssi) {
  if (addNetwork(ssid, bssid, rssi, "[BLE]", 0, 'b')) {
    Serial.println("[SUB] Added BLE device: SSID: " + ssid + ", BSSID: " + bssid + ", RSSI: " + String(rssi));
    totalNetworksFound++;
    
    #ifdef MATRIX
    // Flash purple for BLE
    leds[matrix[0][4]] = CRGB::Purple;
    FastLED.show();
    delay(10);
    leds[matrix[0][4]] = CRGB::Black;
    FastLED.show();
    #else
    blinkLEDPurple();
    #endif
  }
}

// Add a WiFi network to our list
void addWifiNetwork(const String& ssid, const String& bssid, int32_t rssi, wifi_auth_mode_t encryptionType, uint8_t channel) {
  if (addNetwork(ssid, bssid, rssi, getAuthType(encryptionType), channel, 'w')) {
    Serial.println("[SUB] Added WiFi network: SSID: " + ssid + ", BSSID: " + bssid + ", RSSI: " + String(rssi) + ", Channel: " + String(channel));
    totalNetworksFound++;
    
    #ifdef MATRIX
    // Update display for milestone counts
    if (networkCount % 10 == 0) {
      displayNumber(networkCount);
      delay(200);
    } else {
      // Flash color based on channel
      int x = (channel % 5);
      int y = (channel / 5) % 5;
      
      CRGB channelColor;
      if (channel <= 5) {
        channelColor = CRGB::Blue;
      } else if (channel <= 10) {
        channelColor = CRGB::Green;
      } else {
        channelColor = CRGB::Yellow;
      }
      
      leds[matrix[y][x]] = channelColor;
      FastLED.show();
      delay(10);
      leds[matrix[y][x]] = CRGB::Black;
      FastLED.show();
    }
    #else
    // For non-matrix devices
    if (channel <= 5) {
      blinkLEDBlue();
    } else if (channel <= 10) {
      blinkLEDGreen();
    } else {
      blinkLEDYellow();
    }
    #endif
  }
}

// Generic function to add a network
bool addNetwork(const String& ssid, const String& bssid, int32_t rssi, const String& encryptionType, uint8_t channel, char type) {
  if (!isNetworkInList(bssid) && networkCount < MAX_NETWORKS) {
    strncpy(networks[networkCount].ssid, ssid.c_str(), sizeof(networks[networkCount].ssid) - 1);
    networks[networkCount].ssid[sizeof(networks[networkCount].ssid) - 1] = '\0';
    
    strncpy(networks[networkCount].bssid, bssid.c_str(), sizeof(networks[networkCount].bssid) - 1);
    networks[networkCount].bssid[sizeof(networks[networkCount].bssid) - 1] = '\0';
    
    networks[networkCount].rssi = rssi;
    
    strncpy(networks[networkCount].security, encryptionType.c_str(), sizeof(networks[networkCount].security) - 1);
    networks[networkCount].security[sizeof(networks[networkCount].security) - 1] = '\0';
    
    networks[networkCount].channel = channel;
    networks[networkCount].type = type;
    
    #ifdef COMM_NOW
    networks[networkCount].boardId = NODEID;
    #endif
    
    networkCount++;
    return true;
  }
  return false;
}

// Check if network is already in our list
bool isNetworkInList(const String& bssid) {
  for (int i = 0; i < networkCount; ++i) {
    if (strcmp(networks[i].bssid, bssid.c_str()) == 0) {
      return true;
    }
  }
  return false;
}

// Convert WiFi auth mode to string
const char* getAuthType(uint8_t wifiAuth) {
  switch (wifiAuth) {
    case WIFI_AUTH_OPEN:
      return "[OPEN]";
    case WIFI_AUTH_WEP:
      return "[WEP]";
    case WIFI_AUTH_WPA_PSK:
      return "[WPA_PSK]";
    case WIFI_AUTH_WPA2_PSK:
      return "[WPA2_PSK]";
    case WIFI_AUTH_WPA_WPA2_PSK:
      return "[WPA_WPA2_PSK]";
    case WIFI_AUTH_WPA2_ENTERPRISE:
      return "[WPA2_ENTERPRISE]";
    case WIFI_AUTH_WPA3_PSK:
      return "[WPA3_PSK]";
    case WIFI_AUTH_WPA2_WPA3_PSK:
      return "[WPA2_WPA3_PSK]";
    case WIFI_AUTH_WAPI_PSK:
      return "[WAPI_PSK]";
    default:
      return "[UNDEFINED]";
  }
}

// Adjust scan time based on networks found
void updateTimePerChannel(int channel, int networksFound) {
  int timeIncrement = 0;
  
  // Adjust the time per channel based on the number of networks found
  if (networksFound >= MANY_NETWORKS_THRESHOLD) {
    timeIncrement = incrementPerChannel[channel];
  } else if (networksFound <= FEW_NETWORKS_THRESHOLD) {
    timeIncrement = -incrementPerChannel[channel];
  }
  
  int timePerChannelOld = timePerChannel[channel];
  timePerChannel[channel] += timeIncrement;
  
  if (timePerChannel[channel] > MAX_TIME) {
    timePerChannel[channel] = MAX_TIME;
  } else if (timePerChannel[channel] < MIN_TIME) {
    timePerChannel[channel] = MIN_TIME;
  }
  
  if (timePerChannelOld != timePerChannel[channel]) {
    Serial.print("Saw ");
    Serial.print(networksFound);
    Serial.print(" networks, updated timePerChannel for channel ");
    Serial.print(channel);
    Serial.print(" by ");
    Serial.print(timeIncrement);
    Serial.print(" to ");
    Serial.println(timePerChannel[channel]);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("[SUB] Starting up Sub Node " + String(NODEID));

  // Initialize LED
#ifdef S3LITE
  auto cfg = M5.config();
  AtomS3.begin(cfg);
  clearLed();
#else
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(40); // Dimmer to save power
  clearLed();
#endif

  // Initialize WiFi in station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  #ifdef COMM_I2C
  #if defined(S3LITE)
  Wire.begin(i2c_slave_address, SUB_SDA, SUB_SCL);
  #else
  Wire.begin(i2c_slave_address);
  #endif
  Wire.onRequest(requestEvent);
  Serial.println("[SUB] I2C initialized");
  #endif

  #ifdef enableBLE
  Serial.println("Setting up Bluetooth scanning");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false); //active scan uses more power
  pBLEScan->setInterval(timePerChannel[0]);
  pBLEScan->setWindow(40);  // less or equal setInterval value
  #endif
  
  #ifdef MATRIX
  // Startup sequence for Matrix
  // 1. Count up animation
  for (int i = 0; i <= 9; i++) {
    displayNumber(i);
    delay(100);
  }
  delay(500);
  
  // 2. Display node ID
  clearLed();
  displayNumber(NODEID);
  delay(1000);
  
  // 3. Display channel map
  displayChannelMap();
  delay(1500);
  
  // 4. WiFi symbol
  displayWifiSymbol();
  delay(1000);
  
  // 5. Ready to scan
  scanningAnimation();
  #else
  // Startup sequence for non-Matrix
  for (int i = 0; i < NODEID; i++) {
    setLed(CRGB::Blue);
    delay(200);
    clearLed();
    delay(200);
  }
  #endif
  
  Serial.println("Hydra Sub Node " + String(NODEID) + " started! Assigned channels:");
  for (int i = 0; i < channelCount; i++) {
    Serial.print(channels[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  #ifdef enableBLE
  Serial.println("This node will also scan for BLE devices");
  #endif
}

void loop() {
  if (!scan) {
    // Wait for signal from Dom to start scanning
    #ifdef MATRIX
    if (millis() - lastAnimUpdate > 1000) {
      displayWifiSymbol();
      lastAnimUpdate = millis();
    }
    #endif
    delay(100);
    return;
  }
  
  // Avoid scanning too frequently
  if (millis() - lastScanTime < scanDelay) {
    #ifdef MATRIX
    // Update animation while waiting
    if (millis() - lastAnimUpdate > 150) {
      scanningAnimation();
      lastAnimUpdate = millis();
    }
    #endif
    return;
  }
  
  lastScanTime = millis();
  
// Only scan if we have room for more networks
  if (networkCount < MAX_NETWORKS && !isScanning) {
    isScanning = true;
    
    // Scan for BLE devices first if enabled
    #ifdef enableBLE
    static bool bleScanDone = false;
    
    if (!bleScanDone) {
      #ifdef MATRIX
      // Show BLE scan indicator
      clearLed();
      for (int i = 0; i < 25; i++) {
        leds[i] = CRGB(20, 0, 20); // Dim purple
      }
      leds[matrix[2][2]] = CRGB::Purple; // Bright center
      FastLED.show();
      #else
      setLed(CRGB::Purple);
      #endif
      
      BLEScanResults foundDevices = pBLEScan->start(2.5, false);
      Serial.print("BLE devices found: ");
      Serial.println(mac_history_cursor);
      updateTimePerChannel(0, mac_history_cursor);
      Serial.println("BLE scan done!");
      pBLEScan->clearResults();
      
      bleScanDone = true;
      clearLed();
    }
    #endif
    
    // Scan WiFi channels in sequence
    int channel = channels[channelIndex];
    
    // #ifdef MATRIX
    // // Show channel indicator on matrix
    // clearLed();
    // // Show channel number
    // int digitX = 1;
    // CRGB channelColor;
    // if (channel <= 5) {
    //   channelColor = CRGB::Blue;
    // } else if (channel <= 10) {
    //   channelColor = CRGB::Green;
    // } else {
    //   channelColor = CRGB::Yellow;
    // }
    
    // if (channel < 10) {
    //   displayDigit(channel, 1, 0, channelColor);
    // } else {
    //   displayDigit(1, 0, 0, channelColor);
    //   displayDigit(channel % 10, 2, 0, channelColor);
    // }
    // FastLED.show();
    // #else
    // // Non-matrix indicator
    // if (channel <= 5) {
    //   setLed(CRGB::Blue);
    // } else if (channel <= 10) {
    //   setLed(CRGB::Green);
    // } else {
    //   setLed(CRGB::Yellow);
    // }
    // #endif
    
    Serial.print("[SUB] Scanning channel ");
    Serial.println(channel);
    
    int n = WiFi.scanNetworks(false, true, false, timePerChannel[channel], channel);
    
    if (n >= 0) {
      for (int i = 0; i < n; ++i) {
        String currentMAC = WiFi.BSSIDstr(i);
        if (isMACSeen(currentMAC)) {
          continue;
        }
        
        macAddressArray[macArrayIndex++] = currentMAC;
        if (macArrayIndex >= MAX_MAC_HISTORY) {
          macArrayIndex = 0;
          overFlow = true;
        }
        
        addWifiNetwork(WiFi.SSID(i), WiFi.BSSIDstr(i), WiFi.RSSI(i), WiFi.encryptionType(i), WiFi.channel(i));
      }
    }
    
    updateTimePerChannel(channel, n);
    clearLed();
    
    // Move to next channel
    channelIndex = (channelIndex + 1) % channelCount;
    
    // Reset BLE scan flag if we've gone through all WiFi channels
    #ifdef enableBLE
    if (channelIndex == 0) {
      bleScanDone = false;
    }
    #endif
    
    // Show network count periodically
    #ifdef MATRIX
    if (millis() - lastCountDisplay > 5000) {
      displayNumber(networkCount);
      delay(1000);
      lastCountDisplay = millis();
    }
    #endif
    
    isScanning = false;
  }
  
  // If we've filled the network buffer, display the count until reset
  if (networkCount >= MAX_NETWORKS) {
    #ifdef MATRIX
    if (millis() - lastAnimUpdate > 2000) {
      // Alternate between showing count and full indicator
      if ((millis() / 2000) % 2 == 0) {
        displayNumber(networkCount);
      } else {
        // Show "full" pattern - all LEDs red
        for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CRGB::Red;
        }
        FastLED.show();
      }
      lastAnimUpdate = millis();
    }
    #else
    // For non-matrix, just blink red to indicate full
    if (millis() - lastAnimUpdate > 1000) {
      blinkLEDRed();
      lastAnimUpdate = millis();
    }
    #endif
  } else {
    // Normal scan animation between scans
    #ifdef MATRIX
    if (millis() - lastAnimUpdate > 150) {
      scanningAnimation();
      lastAnimUpdate = millis();
    }
    #endif
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}