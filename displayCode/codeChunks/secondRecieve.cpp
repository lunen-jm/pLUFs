// Client code for the second ESP32
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

// The remote service and characteristic we wish to connect to
static BLEUUID SERVICE_UUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID LUFS_CHAR_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

static BLEClient* pClient = nullptr;
static BLERemoteCharacteristic* pRemoteLufsCharacteristic = nullptr;
static bool doConnect = false;
static bool connected = false;
static BLEAdvertisedDevice* myDevice = nullptr;

// Notification callback
static void notifyCallback(BLERemoteCharacteristic* pCharacteristic, 
                           uint8_t* pData, size_t length, bool isNotify) {
  // Convert the data to a string
  std::string value = pCharacteristic->readValue();
  Serial.print("Received notification: ");
  Serial.println(value.c_str());
  
  // Here you can process the JSON data
  // For a simple project, this might be enough
  // For a more complex project, consider using ArduinoJson library
}

// Connect to the BLE Server
bool connectToServer() {
  Serial.print("Connecting to ");
  Serial.println(myDevice->getAddress().toString().c_str());
  
  pClient = BLEDevice::createClient();
  
  // Connect to the remote BLE Server
  if (!pClient->connect(myDevice)) {
    Serial.println("Connection failed");
    return false;
  }
  Serial.println("Connected to server");

  // Obtain a reference to the service we are after
  BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(SERVICE_UUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println("Found our service");

  // Obtain a reference to the characteristic
  pRemoteLufsCharacteristic = pRemoteService->getCharacteristic(LUFS_CHAR_UUID);
  if (pRemoteLufsCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(LUFS_CHAR_UUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println("Found our characteristic");

  // Read the value of the characteristic
  if (pRemoteLufsCharacteristic->canRead()) {
    std::string value = pRemoteLufsCharacteristic->readValue();
    Serial.print("Initial value: ");
    Serial.println(value.c_str());
  }

  // Register for notifications
  if (pRemoteLufsCharacteristic->canNotify()) {
    pRemoteLufsCharacteristic->registerForNotify(notifyCallback);
    Serial.println("Registered for notifications");
  }

  connected = true;
  return true;
}

// Scan for BLE servers and find the first one that advertises the service we are looking for
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // Check if the device contains the service we are looking for
    if (advertisedDevice.haveServiceUUID() && 
        advertisedDevice.isAdvertisingService(SERVICE_UUID)) {
      Serial.println("Found LUFS_Meter device!");
      
      // Save the device and stop scanning
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      
      // Stop scanning
      BLEDevice::getScan()->stop();
    }
  }
};

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  
  Serial.println("Starting BLE Client application...");
  
  BLEDevice::init("LUFS_Client");

  // Retrieve a Scanner and set the callback
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  Serial.println("Starting scan for LUFS_Meter devices...");
  pBLEScan->start(5, false); // Scan for 5 seconds
}

void loop() {
  // If we found a device to connect to, connect to it
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Connected to LUFS_Meter server");
    } else {
      Serial.println("Failed to connect, retrying in 5 seconds...");
      delay(5000);
      BLEDevice::getScan()->start(5, false); // Scan for 5 seconds
    }
    doConnect = false;
  }

  // If we're disconnected, reconnect
  if (!connected && myDevice != nullptr) {
    Serial.println("Device disconnected, trying to reconnect...");
    connectToServer();
  }
  
  delay(1000);
}