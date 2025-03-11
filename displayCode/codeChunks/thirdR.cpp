// Client code for the second ESP32
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEServer.h>
#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include <lv_conf.h>
#include <TFT_eSPI.h>

// uncomment a library for display driver
#define USE_TFT_ESPI_LIBRARY
// #define USE_ARDUINO_GFX_LIBRARY

#include "lv_xiao_round_screen.h"
#include "lv_hardware_test.h"

BLEClient* pClient;
bool doConnect = false;
bool connected = false;
bool doScan = true;  // Added flag to control scanning
BLEAdvertisedDevice* myDevice = nullptr;

//BLE Server name (the other ESP32 name running the server sketch)
#define bleServerName "XIAOESP32S3_BLE"

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;

BLEUUID serviceUUID("181A"); // Environmental Sensing
BLEUUID charUUID("2A59");    // Analog Output

char lufs_val[1024];

bool connectToServer() {
    Serial.println("Attempting to connect to server...");
    
    // Create a connection to the server
    pClient = BLEDevice::createClient();
    Serial.println("Created client");
    
    // Connect to the remote BLE Server
    try {
        // Try connecting using the device address
        if (myDevice == nullptr) {
            Serial.println("ERROR: myDevice is null");
            return false;
        }
        
        Serial.print("Connecting to device: ");
        Serial.println(myDevice->getAddress().toString().c_str());
        
        // Try connecting using different methods
        bool success = false;
        
        // Method 1: Connect by address
        try {
            success = pClient->connect(myDevice->getAddress());
            if (success) {
                Serial.println("Connected using address method");
            } else {
                Serial.println("Connection failed using address method");
            }
        } catch (...) {
            Serial.println("Exception with address method");
        }
        
        if (!success) {
            Serial.println("All connection methods failed");
            return false;
        }
    } catch (const std::exception& e) {
        Serial.print("Exception during connection: ");
        Serial.println(e.what());
        return false;
    } catch (...) {
        Serial.println("Unknown exception during connection");
        return false;
    }
    
    Serial.println("Connected to server");
    
    // Obtain a reference to the service we are after in the remote BLE server
    Serial.print("Looking for service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
        Serial.print("Failed to find our service UUID: ");
        Serial.println(serviceUUID.toString().c_str());
        Serial.println("Available services:");
        
        // List available services for debugging
        std::map<std::string, BLERemoteService*>* serviceMap = pClient->getServices();
        if (serviceMap != nullptr) {
            for (auto& kv : *serviceMap) {
                Serial.print("  - ");
                Serial.println(kv.first.c_str());
            }
        } else {
            Serial.println("  (No services found)");
        }
        
        pClient->disconnect();
        return false;
    }
    
    Serial.print("Found service, looking for characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    
    // Obtain a reference to the characteristics in the service of the remote BLE server
    BLERemoteCharacteristic* pCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pCharacteristic == nullptr) {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(charUUID.toString().c_str());
        Serial.println("Available characteristics:");
        
        // List available characteristics for debugging
        std::map<std::string, BLERemoteCharacteristic*>* charMap = pRemoteService->getCharacteristics();
        if (charMap != nullptr) {
            for (auto& kv : *charMap) {
                Serial.print("  - ");
                Serial.println(kv.first.c_str());
            }
        } else {
            Serial.println("  (No characteristics found)");
        }
        
        pClient->disconnect();
        return false;
    }
    
    // Register for notifications with improved JSON handling
Serial.println("Registering for notifications");
pCharacteristic->registerForNotify([](BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                   uint8_t* pData, size_t length, bool isNotify) {
    Serial.println("Notify received");
    
    // Create a null-terminated string from the data
    char jsonStr[length + 1];
    memcpy(jsonStr, pData, length);
    jsonStr[length] = '\0';
    
    // Print the received JSON string
    Serial.print("Received JSON: ");
    Serial.println(jsonStr);
    
    // Store the raw JSON string
    snprintf(lufs_val, sizeof(lufs_val), "%s", jsonStr);
    
    // If you need to parse the JSON and extract specific values,
    // you could use ArduinoJson library or manual parsing here
    
    // Example of simple manual parsing for key-value pairs
    // This assumes JSON format like {"key1":"value1","key2":123}
    char* valueStart = strstr(jsonStr, "\"value\":");
    if (valueStart) {
        valueStart += 8; // Skip over "value":
        // Extract numeric value or string value depending on format
        int extractedValue = atoi(valueStart);
        Serial.print("Extracted value: ");
        Serial.println(extractedValue);
    }
});
    
    connected = true;
    Serial.println("Successfully connected and set up notifications");
    return true;
}

//Callback function that gets called when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("Device found: ");
        if (advertisedDevice.haveName()) {
            Serial.print(advertisedDevice.getName().c_str());
        } else {
            Serial.print("(No name)");
        }
        Serial.print(" (");
        Serial.print(advertisedDevice.getAddress().toString().c_str());
        Serial.println(")");
        
        // Debug all device names for troubleshooting
        if (advertisedDevice.haveName()) {
            Serial.print("Comparing found name: '");
            Serial.print(advertisedDevice.getName().c_str());
            Serial.print("' with target name: '");
            Serial.print(bleServerName);
            Serial.println("'");
        }
        
        if (advertisedDevice.haveName() && advertisedDevice.getName() == bleServerName) {
            Serial.println("******** TARGET DEVICE FOUND! ********");
            advertisedDevice.getScan()->stop(); // Stop scanning
            myDevice = new BLEAdvertisedDevice(advertisedDevice); // Save the device
            doConnect = true; // Set flag to connect to this device
            doScan = false;   // Stop scanning
        }
    }
};

void setup() {
    Serial.begin(115200);
    
    // Wait for serial connection
    unsigned long startTime = millis();
    while (!Serial && millis() - startTime < 5000) {
        delay(100);
    }
    
    Serial.println("\n\n--------------------");
    Serial.println("Starting BLE Client application...");
    
    BLEDevice::init("LUFS_Client");
    Serial.println("BLE Device initialized");
}

void loop() {
    // If we should scan for devices
    if (doScan) {
        Serial.println("Starting new BLE scan...");
        BLEScan* pBLEScan = BLEDevice::getScan();
        pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
        pBLEScan->setInterval(100);
        pBLEScan->setWindow(99);
        pBLEScan->setActiveScan(true);
        Serial.print("Looking for server named: ");
        Serial.println(bleServerName);
        pBLEScan->start(5, false); // Scan for 5 seconds
        Serial.println("Scan started");
        delay(6000); // Wait a bit after the scan
    }
    
    // If we found a device to connect to, connect to it
    if (doConnect) {
        Serial.println("doConnect flag is true, attempting connection");
        if (connectToServer()) {
            Serial.println("Connected to LUFS_Meter server successfully");
        } else {
            Serial.println("Failed to connect, retrying in 5 seconds...");
            delay(5000);
            doScan = true;  // Re-enable scanning
        }
        doConnect = false;
    }
  
    // If we're disconnected, reconnect
    if (!connected && myDevice != nullptr) {
        Serial.println("Device disconnected, trying to reconnect...");
        if (connectToServer()) {
            Serial.println("Reconnection successful");
        } else {
            Serial.println("Reconnection failed, will retry scanning");
            doScan = true;  // Re-enable scanning
        }
    }
    
    static unsigned long lastReport = 0;
    if (millis() - lastReport > 5000) {
        Serial.println("Still alive... Connection status: " + String(connected ? "Connected" : "Disconnected"));
        lastReport = millis();
    }
    
    delay(1000);
}