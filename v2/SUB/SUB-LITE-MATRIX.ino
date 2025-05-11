/*
   M5Atom-Hydra (Sub)
   Optimized for M5Atom Lite/Matrix hardware
*/

#define COMM_I2C
//#define COMM_NOW

// CHOOSE HARDWARE
// #define ATOMLITE
#define MATRIX
//#define S3LITE

// CHOOSE NODE ID (1-6)
#define NODEID 2

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
// No LED_PIN needed for S3LITE
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

#ifdef COMM_I2C
const int i2c_slave_address = 0x55;
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
  setLed(CRGB::Black);
#endif
}

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

#ifdef MATRIX
// Special pattern for Matrix LED display
void showMatrixPattern(int pattern) {
  clearLed();
  
  switch (pattern) {
    case 0: // Node ID pattern
      for (int i = 0; i < NODEID && i < 5; i++) {
        leds[i] = CRGB::Blue;
      }
      break;
      
    case 1: // Scanning animation
      leds[12] = CRGB::Green;
      for (int i = 0; i < 8; i++) {
        int pos = (i * 3) % 24;
        leds[pos] = CRGB(0, 50, 0);
      }
      break;
      
    case 2: // Channel activity - highlight WiFi channels being scanned
      for (int i = 0; i < channelCount; i++) {
        int ch = channels[i];
        if (ch <= 5) {
          leds[ch * 2] = CRGB::Blue;
        } else if (ch <= 10) {
          leds[ch] = CRGB::Green;
        } else {
          leds[ch + 4] = CRGB::Yellow;
        }
      }
      break;
  }
  
  FastLED.show();
}
#endif

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
    blinkLEDWhite();
  } else {
    // If we have nothing to send, start scanning again
    Serial.println("[SUB] No new networks to send");
    currentNetworkIndex = 0;
    networkCount = 0;
  }
}
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
          blinkLEDPurple();
        }
      }
    }
};
#endif

// Add a BLE network to our list
void addBleNetwork(const String& ssid, const String& bssid, int32_t rssi) {
  if (addNetwork(ssid, bssid, rssi, "[BLE]", 0, 'b')) {
    Serial.println("[SUB] Added BLE device: SSID: " + ssid + ", BSSID: " + bssid + ", RSSI: " + String(rssi));
  }
}

// Add a WiFi network to our list
void addWifiNetwork(const String& ssid, const String& bssid, int32_t rssi, wifi_auth_mode_t encryptionType, uint8_t channel) {
  if (addNetwork(ssid, bssid, rssi, getAuthType(encryptionType), channel, 'w')) {
    Serial.println("[SUB] Added WiFi network: SSID: " + ssid + ", BSSID: " + bssid + ", RSSI: " + String(rssi) + ", Channel: " + String(channel));
    
    // Blink LED based on channel
    if (channel <= 5) {
      blinkLEDBlue();
    } else if (channel <= 10) {
      blinkLEDGreen();
    } else {
      blinkLEDYellow();
    }
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
  FastLED.setBrightness(40); // Slightly dimmer for better power efficiency
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
  pBLEScan->setActiveScan(false); //active scan uses more power, but get results faster
  pBLEScan->setInterval(timePerChannel[0]);
  pBLEScan->setWindow(40);  // less or equal setInterval value
  #endif
  
  // Blink LED pattern to show which node ID this is
  for (int i = 0; i < NODEID; i++) {
    setLed(CRGB::Blue);
    delay(200);
    clearLed();
    delay(200);
  }
  
  #ifdef MATRIX
  // Special startup animation for Matrix
  showMatrixPattern(0); // Show node ID
  delay(1000);
  showMatrixPattern(2); // Show channel distribution
  delay(2000);
  clearLed();
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
    // Wait for signal from dom to start scanning
    delay(100);
    return;
  }
  
  // Avoid scanning too frequently
  if (millis() - lastScanTime < scanDelay) {
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
      // Indicate BLE scanning
      setLed(CRGB::Purple);
      
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
    
    // Indicate channel scanning
    if (channel <= 5) {
      setLed(CRGB::Blue);
    } else if (channel <= 10) {
      setLed(CRGB::Green);
    } else {
      setLed(CRGB::Yellow);
    }
    
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
    
    isScanning = false;
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}