#include <WiFi.h>

/* === GRAB THE CHANNEL FROM PLATFORMIO.INI === */
// If for some reason it wasn't defined, default to 1
#ifndef WIFI_CHANNEL
  #define WIFI_CHANNEL 1 
#endif

const int channel = WIFI_CHANNEL;      
/* ============================================ */

String dynamicSSID = "Channel_" + String(channel);
const char* password = "CORA1234";

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- ESP32 Distance Target Booting ---");

  WiFi.mode(WIFI_AP); 
  WiFi.softAP(dynamicSSID.c_str(), password, channel, 0); 

  Serial.print("Access Point Started! SSID: ");
  Serial.println(dynamicSSID);
  Serial.print("Channel: ");
  Serial.println(channel);
}

void loop() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 5000) {
    Serial.printf("Broadcasting %s on Ch %d... Connected stations: %d\n", dynamicSSID.c_str(), channel, WiFi.softAPgetStationNum());
    lastUpdate = millis();
  }
}