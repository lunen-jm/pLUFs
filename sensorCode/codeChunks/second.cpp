#include <I2S.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define LUFS_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Global BLE variables
BLEServer *pServer = NULL;
BLECharacteristic *pLufsCharacteristic = NULL;
bool deviceConnected = false;

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
const int sendInterval = 1000; // Send data every second (1000ms)

// BLE server callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Client connected");
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Client disconnected");
      // Restart advertising when disconnected
      pServer->getAdvertising()->start();
      Serial.println("Advertising restarted");
    }
};

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  // Initialize BLE
  BLEDevice::init("LUFS_Meter");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Create a BLE characteristic for LUFS data
  pLufsCharacteristic = pService->createCharacteristic(
                          LUFS_CHAR_UUID,
                          BLECharacteristic::PROPERTY_READ |
                          BLECharacteristic::PROPERTY_NOTIFY
                       );
                       
  // Create a BLE descriptor
  pLufsCharacteristic->addDescriptor(new BLE2902());
  
  // Start the service
  pService->start();
  
  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("BLE server started, waiting for connections...");

  // Start I2S at 16 kHz with 16-bits per sample
  I2S.setAllPins(-1, 42, 41, -1, -1);
  if (!I2S.begin(PDM_MONO_MODE, 16000, 16)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
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
  // Read samples into buffer
  if (I2S.available()) {
    int sample = I2S.read();
    
    if (sample != 0) {  // Skip 0 values
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

          // Send via BLE if client is connected (throttled to once per second)
          unsigned long currentTime = millis();
          if (deviceConnected && (currentTime - lastSendTime >= sendInterval)) {
            lastSendTime = currentTime;
            
            // Format data as JSON for easy parsing
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "{\"lufs\":%.1f,\"level\":\"%s\"}", 
                     lufs, loudnessLabel.c_str());
                     
            pLufsCharacteristic->setValue(buffer);
            pLufsCharacteristic->notify();
            
            Serial.print("BLE Notify: ");
            Serial.println(buffer);
          }
        }
        
        sampleIndex = 0;
      }
    }
  }
}