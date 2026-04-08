// Server_Wifi.cpp

#include <WiFi.h>

// This must match the TARGET_SSID and TARGET_PASSWORD in your client code
const char* ssid = "Test3";
const char* password = "CORA1234";

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- ESP32 Distance Target Booting ---");

  // Set the device as an Access Point
  // We use channel 1 to keep the signal consistent for testing
  WiFi.softAP(ssid, password, 1, 0); 

  Serial.print("Access Point Started! SSID: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  // The server doesn't need to do anything in the loop for RSSI testing.
  // Your client device just needs to see the signal it's broadcasting.
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 5000) {
    Serial.printf("Broadcasting... Connected stations: %d\n", WiFi.softAPgetStationNum());
    lastUpdate = millis();
  }
}