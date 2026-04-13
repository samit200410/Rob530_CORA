// Client_Wifi.cpp

#include <WiFi.h>
#include <math.h>

/* ================= BEACON CONFIGURATION ================= */
struct Beacon {
  String ssid;
  float x;          
  float y;          
  float x_est;      
  float P;          
  float distance;   
  bool visible;     
};

Beacon beacons[3] = {
  {"Channel_1", 0.0, 0.0, -60.0, 1.0, 0.0, false},   
  {"Channel_6", 5.0, 0.0, -60.0, 1.0, 0.0, false},   
  {"Channel_11", 2.5, 5.0, -60.0, 1.0, 0.0, false}    
};

/* ================= GLOBAL STATE ================= */
float posX = 0.0;
float posY = 0.0;

/* ================= USER SET UP INPUTS================= */
const float RSSI_1M = -55.0;
const float PATH_LOSS = 3.0;
const int SCAN_DELAY = 500;
const float Q = 0.05; 
const float R = 8.0;  

/* ================= FILTER & DISTANCE ================= */
float kalmanFilter(float z, Beacon &b) {
  b.P = b.P + Q;
  float K = b.P / (b.P + R);
  b.x_est = b.x_est + K * (z - b.x_est);
  b.P = (1 - K) * b.P;
  return b.x_est;
}

float rssiToDistance(float rssi) {
  return pow(10.0, (RSSI_1M - rssi) / (10.0 * PATH_LOSS));
}

/* ================= TELEMETRY PROTOCOL ================= */
void sendTelemetry() {
  char payload[128];
  
  // Format the payload: DATA,esp_ms,d1,d6,d11,x,y
  snprintf(payload, sizeof(payload), "DATA,%lu,%.2f,%.2f,%.2f,%.2f,%.2f", 
           millis(), 
           beacons[0].distance, 
           beacons[1].distance, 
           beacons[2].distance, 
           posX, 
           posY);

  // Calculate XOR Checksum
  uint8_t checksum = 0;
  for (int i = 0; payload[i] != '\0'; i++) {
    checksum ^= payload[i];
  }

  // Send packet with Sync ($) and Checksum (*XX)
  Serial.printf("$%s*%02X\n", payload, checksum);
}

/* ================= TRILATERATION ================= */
bool calculatePosition() {
  float x1 = beacons[0].x; float y1 = beacons[0].y; float d1 = beacons[0].distance;
  float x2 = beacons[1].x; float y2 = beacons[1].y; float d2 = beacons[1].distance;
  float x3 = beacons[2].x; float y3 = beacons[2].y; float d3 = beacons[2].distance;

  float A = 2 * x2 - 2 * x1;
  float B = 2 * y2 - 2 * y1;
  float C = pow(d1, 2) - pow(d2, 2) - pow(x1, 2) + pow(x2, 2) - pow(y1, 2) + pow(y2, 2);
  
  float D = 2 * x3 - 2 * x2;
  float E = 2 * y3 - 2 * y2;
  float F = pow(d2, 2) - pow(d3, 2) - pow(x2, 2) + pow(x3, 2) - pow(y2, 2) + pow(y3, 2);

  float denominator = (A * E - B * D);
  
  if (abs(denominator) < 0.0001) {
    Serial.println("Math Error: Beacons are collinear. Move them to form a triangle.");
    return false;
  }

  posX = (C * E - F * B) / denominator;
  posY = (C * D - A * F) / (B * D - A * E);
  return true;
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  Serial.println("Scanner Ready. Waiting for beacons...");
}

/* ================= LOOP ================= */
void loop() {
  for (int i = 0; i < 3; i++) beacons[i].visible = false;

  int n = WiFi.scanNetworks();
  
  if (n > 0) {
    for (int i = 0; i < n; ++i) {
      String foundSSID = WiFi.SSID(i);
      int rawRSSI = WiFi.RSSI(i);

      for (int b = 0; b < 3; b++) {
        if (foundSSID == beacons[b].ssid) {
          float filtered = kalmanFilter(rawRSSI, beacons[b]);
          beacons[b].distance = rssiToDistance(filtered);
          beacons[b].visible = true;
        }
      }
    }
  }

  // Only calculate and send data if we have a full positional lock
  if (beacons[0].visible && beacons[1].visible && beacons[2].visible) {
    if (calculatePosition()) {
      sendTelemetry(); // Dispatches the $DATA payload with CRC
    }
  } else {
    Serial.println("Waiting for all 3 beacons...");
  }

  WiFi.scanDelete();
  delay(SCAN_DELAY);
}