#include <Arduino.h>
#include <BluetoothSerial.h>
#include <AudioTools.h>
#include <AudioAnalyzer.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// Audio configuration
constexpr int SAMPLE_RATE = 16000;
constexpr int CHANNELS = 1;
constexpr int BITS_PER_SAMPLE = 16;

// LUFS calculation parameters
const int BUFFER_SIZE = 512;
constexpr int FFT_SIZE = 512;
constexpr int MEASURE_INTERVAL_MS = 200; // How often we calculate LUFS

// I2S microphone pins for ESP32S3 XIAO
const int I2S_MIC_SERIAL_CLOCK = D8;  // SCK
const int I2S_MIC_LEFT_RIGHT_CLOCK = D9;  // WS 
const int I2S_MIC_SERIAL_DATA = D10; // SD

// Audio objects
I2SStream i2s_stream;
AudioAnalyzer analyzer;
int16_t buffer[BUFFER_SIZE];

// Timing variables
unsigned long lastMeasureTime = 0;
float currentLUFS = -60.0; // Default quiet value

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32S3 LUFS Transmitter");

  // Start Bluetooth with device name
  SerialBT.begin("ESP32S3_LUFS");
  Serial.println("Bluetooth device started, connect to ESP32S3_LUFS");

  // Setup I2S for microphone
  auto i2s_config = i2s_stream.defaultConfig();
  i2s_config.i2s_format = I2S_STD_FORMAT;
  i2s_config.sample_rate = SAMPLE_RATE;
  i2s_config.channels = CHANNELS;
  i2s_config.bits_per_sample = BITS_PER_SAMPLE;
  i2s_config.pin_ws = I2S_MIC_LEFT_RIGHT_CLOCK;
  i2s_config.pin_bck = I2S_MIC_SERIAL_CLOCK;
  i2s_config.pin_data = I2S_MIC_SERIAL_DATA;
  i2s_config.pin_data_in = I2S_MIC_SERIAL_DATA;
  i2s_config.is_master = true;
  i2s_config.mode = I2S_MODE_MASTER | I2S_MODE_RX;
  i2s_config.use_apll = false;
  
  // Initialize I2S
  i2s_stream.begin(i2s_config);
  
  // Initialize analyzer for LUFS measurement
  analyzer.begin(SAMPLE_RATE, BUFFER_SIZE);
  
  Serial.println("Audio system initialized");
}

// Calculate LUFS from audio buffer
float calculateLUFS(int16_t* buffer, size_t length) {
  // Simple RMS calculation (a simplified approach to LUFS)
  float sum = 0;
  for (size_t i = 0; i < length; i++) {
    float sample = buffer[i] / 32768.0; // Normalize to -1.0 to 1.0
    sum += sample * sample;
  }
  
  float rms = sqrt(sum / length);
  float db = 20 * log10(rms);
  
  // Apply K-weighting (simplified)
  float lufs = db - 0.691; // Simple offset for K-weighting
  
  return lufs;
}

void loop() {
  // Read audio data from I2S microphone
  size_t bytes_read = i2s_stream.readBytes((uint8_t*)buffer, BUFFER_SIZE * sizeof(int16_t));

  if (bytes_read > 0) {
    // Calculate LUFS at specified intervals
    if (millis() - lastMeasureTime > MEASURE_INTERVAL_MS) {
      currentLUFS = calculateLUFS(buffer, BUFFER_SIZE);
      
      // Send LUFS value via Bluetooth
      SerialBT.print("LUFS:");
      SerialBT.println(currentLUFS);
      
      // Also print to serial console for debugging
      Serial.print("LUFS: ");
      Serial.println(currentLUFS);
      
      lastMeasureTime = millis();
    }
  }
}