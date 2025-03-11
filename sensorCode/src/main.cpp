//server
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEServer.h>
#include <Wire.h>
#include <Arduino.h>
#include <I2S.h>

//BLE Server name (the other ESP32 name running the server sketch)
#define bleServerName "XIAOESP32S3_BLE"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

int lufs = 0;

// Constants for LUFS measurement
const int BLOCK_SIZE = 400;  // 25ms at 16kHz
const float REFERENCE_LEVEL = 32767.0;  // Max value for 16-bit audio
const int NUM_BLOCKS = 10;   // Number of blocks to average (about 250ms total)

// Buffer to store samples
int16_t samples[BLOCK_SIZE];
float blockLoudness[NUM_BLOCKS];
int blockIndex = 0;
int sampleIndex = 0;

// Timestamp to track when we last sent data
unsigned long lastSendTime = 0;
unsigned long lastConnectionCheck = 0;
const int sendInterval = 333; // Send data every 333ms
const int connectionCheckInterval = 5000; // Check connection every 5 seconds

// BLE server callbacks
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Client connected!");
  };
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Client disconnected!");
  }
};

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  
  Serial.println("Initializing BLE LUFS Meter Server...");

  // Create the BLE Device
  BLEDevice::init(bleServerName);
  
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create the BLE Service
  BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x181A)); // Environmental Sensing
  
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    BLEUUID((uint16_t)0x2A59), // Analog Output
    BLECharacteristic::PROPERTY_NOTIFY
  );
  
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());
  
  // Start the service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);
  pAdvertising->setMinPreferred(0x1F);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE server started, waiting for connections...");

  // Start I2S at 16 kHz with 16-bits per sample
  Serial.println("Initializing I2S...");
  I2S.setAllPins(-1, 42, 41, -1, -1);
  if (!I2S.begin(PDM_MONO_MODE, 16000, 16)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
  Serial.println("I2S initialized successfully!");
}

// Apply simplified K-weighting (high-pass filter)
float kWeight(int sample) {
  static float prev_sample = 0;
  float output = 0.9 * (float)sample - 0.9 * prev_sample;
  prev_sample = sample;
  return output;
}

// Calculate LUFS value from a block of samples
float calculateBlockLoudness(int16_t* blockSamples, int blockSize) {
  float sumSquared = 0.0;
  
  for (int i = 0; i < blockSize; i++) {
    // Apply K-weighting filter
    float weightedSample = kWeight(blockSamples[i]);
    
    // Square the sample and accumulate
    sumSquared += weightedSample * weightedSample;
  }
  
  // Mean square
  float meanSquare = sumSquared / blockSize;
  
  // Convert to LUFS (simplified)
  if (meanSquare <= 0) return -70.0; // Silence
  
  return -0.691 + 10.0 * log10(meanSquare / (REFERENCE_LEVEL * REFERENCE_LEVEL)) - 10.0;
}

// Calculate integrated LUFS from block loudness values
float calculateIntegratedLUFS() {
  float sum = 0;
  int validBlocks = 0;
  
  // Simple average (a proper LUFS would use gating)
  for (int i = 0; i < NUM_BLOCKS; i++) {
    if (blockLoudness[i] > -70.0) {  // Ignore silent blocks
      sum += blockLoudness[i];
      validBlocks++;
    }
  }
  
  if (validBlocks == 0) return -70.0;
  return sum / validBlocks;
}

// Convert LUFS to descriptive label
String getLoudnessLabel(float lufs) {
  if (lufs < -60) return "Silent";
  if (lufs < -50) return "Very quiet";
  if (lufs < -40) return "Quiet";
  if (lufs < -30) return "Moderate";
  if (lufs < -20) return "Loud";
  if (lufs < -10) return "Very loud";
  return "Extremely loud";
}

void loop() {
  unsigned long currentTime = millis();
  
  // Handle connection state changes
  if (!deviceConnected && oldDeviceConnected) {
    // Give the Bluetooth stack time to get things ready
    delay(500);
    // Restart advertising
    Serial.println("Restarting advertising...");
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  
  // If connecting
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("Connection established - starting to send data");
  }
  
  // Periodically check connection status
  if (currentTime - lastConnectionCheck >= connectionCheckInterval) {
    lastConnectionCheck = currentTime;
    if (deviceConnected) {
      Serial.println("Connection status: Connected");
    } else {
      Serial.println("Connection status: Disconnected");
    }
  }
  
  // Read samples into buffer
  if (I2S.available()) {
    int sample = I2S.read();
    
    if (sample != 0 && sample != -1 && sample != 1) {  // Skip 0, -1, and 1 values (likely noise)
      samples[sampleIndex++] = sample;
      
      // When buffer is full, process the block
      if (sampleIndex >= BLOCK_SIZE) {
        blockLoudness[blockIndex] = calculateBlockLoudness(samples, BLOCK_SIZE);
        blockIndex = (blockIndex + 1) % NUM_BLOCKS;
        
        // Calculate and display LUFS every NUM_BLOCKS
        if (blockIndex == 0) {
          float lufs = calculateIntegratedLUFS();
          String loudnessLabel = getLoudnessLabel(lufs);
          
          // Print to Serial monitor
          Serial.print("Approximate LUFS: ");
          Serial.print(lufs);
          Serial.print(" (");
          Serial.print(loudnessLabel);
          Serial.println(")");

          // Send via BLE if client is connected
          if (deviceConnected && (currentTime - lastSendTime >= sendInterval)) {
            lastSendTime = currentTime;
            
            // Create a smaller, more efficient buffer
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "{\"lufs\":%.1f,\"lvl\":\"%s\"}", 
                     lufs, loudnessLabel.c_str());
                     
            try {
              pCharacteristic->setValue(buffer);
              pCharacteristic->notify();
              Serial.print("BLE Notify: ");
              Serial.println(buffer);
            } catch (std::exception &e) {
              Serial.print("BLE notify error: ");
              Serial.println(e.what());
            }
          }
        }
        
        sampleIndex = 0;
      }
    }
  }
}