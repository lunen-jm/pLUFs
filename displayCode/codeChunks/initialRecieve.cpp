#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi settings - match the transmitter's AP
const char* ssid = "LUFS_Transmitter";
const char* password = "12345678";

// UDP setup
WiFiUDP udp;
const int udpPort = 4210;
char packetBuffer[255];

// LED indicator
const int LED_PIN = 10; // Default GPIO for LED on ESP32C3

// LUFS data
float receivedLUFS = -60.0;
unsigned long lastDataTime = 0;
const int CONNECTION_TIMEOUT = 5000; // milliseconds
bool isConnected = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("ESP32C3 LUFS Receiver");
  
  // Setup LED indicator
  pinMode(LED_PIN, OUTPUT);
  
  // Connect to WiFi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  // Wait for connection with visual indication
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Start UDP
  udp.begin(udpPort);
  Serial.printf("Listening on UDP port %d\n", udpPort);
}

void loop() {
  // Check if WiFi is still connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost. Reconnecting...");
    WiFi.reconnect();
    
    // Flash LED rapidly to indicate disconnection
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  // Check for UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // We've received data
    isConnected = true;
    lastDataTime = millis();
    
    // Read the packet into our buffer
    int len = udp.read(packetBuffer, 254);
    if (len > 0) {
      packetBuffer[len] = 0; // Null terminator
    }
    
    // Process the packet if it's a LUFS value
    String message = String(packetBuffer);
    if (message.startsWith("LUFS:")) {
      String lufsValue = message.substring(5); // Extract value after "LUFS:"
      receivedLUFS = lufsValue.toFloat();
      
      // Print the received value
      Serial.print("Received LUFS: ");
      Serial.println(receivedLUFS);
      
      // Map LUFS value to LED brightness
      int ledValue = map(constrain(receivedLUFS, -60, 0), -60, 0, 0, 255);
      analogWrite(LED_PIN, ledValue);
    }
  } else {
    // Check for timeout if previously connected
    if (isConnected && (millis() - lastDataTime > CONNECTION_TIMEOUT)) {
      Serial.println("Connection timeout - no data received recently");
      isConnected = false;
      
      // Indicate disconnection with LED pulse
      for (int i = 0; i < 2; i++) {
        // Fade up
        for (int brightness = 0; brightness < 255; brightness++) {
          analogWrite(LED_PIN, brightness);
          delay(2);
        }
        // Fade down
        for (int brightness = 255; brightness >= 0; brightness--) {
          analogWrite(LED_PIN, brightness);
          delay(2);
        }
      }
      analogWrite(LED_PIN, 0); // Turn off LED
    }
  }
}