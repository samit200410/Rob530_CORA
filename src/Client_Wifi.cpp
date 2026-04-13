// Client_Wifi.cpp

#include <WiFi.h>
#include <math.h>

/* ================= BEACON CONFIGURATION ================= */
// Structure to hold individual beacon data and Kalman states
struct Beacon {
  String ssid;
  float x;          // Physical X coordinate in room (meters)
  float y;          // Physical Y coordinate in room (meters)
  
  float x_est;      // Kalman state: estimated RSSI
  float P;          // Kalman state: error covariance
  float distance;   // Last computed distance
  bool visible;     // Was it seen in the current scan?
};

// Define your 3 servers here. 
// YOU MUST UPDATE THE X AND Y VALUES based on your actual room layout.
Beacon beacons[3] = {
  {"Channel_1", 0.0, 0.0, -60.0, 1.0, 0.0, false},   // Server 1 at origin (0,0)
  {"Channel_6", 5.0, 0.0, -60.0, 1.0, 0.0, false},   // Server 2 at 5m on X axis
  {"Channel_11", 2.5, 5.0, -60.0, 1.0, 0.0, false}    // Server 3 at (2.5, 5)
};

/* ================= USER SET UP INPUTS================= */
const float RSSI_1M = -55.0;
const float PATH_LOSS = 3.0;
const int SCAN_DELAY = 500;

/* ================= KALMAN CONSTANTS ================= */
const float Q = 0.05; // process noise
const float R = 8.0;  // measurement noise

/* ================= FILTER & DISTANCE ================= */
// Pass the specific beacon reference into the filter
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

/* ================= TRILATERATION ================= */
void calculatePosition() {
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
  
  // Prevent division by zero if beacons are perfectly linear
  if (abs(denominator) < 0.0001) {
    Serial.println("Math Error: Beacons are collinear (in a straight line). Move them to form a triangle.");
    return;
  }

  float posX = (C * E - F * B) / denominator;
  float posY = (C * D - A * F) / (B * D - A * E);

  Serial.print("=> POSITION: X: ");
  Serial.print(posX, 2);
  Serial.print(" m | Y: ");
  Serial.print(posY, 2);
  Serial.println(" m");
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  
  // We don't connect. We disconnect to ensure the radio is free to scan.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  
  Serial.println("Scanner Ready. Waiting for beacons...");
}

/* ================= LOOP ================= */
void loop() {
  // Reset visibility for this loop iteration
  for (int i = 0; i < 3; i++) beacons[i].visible = false;

  // Scan for Wi-Fi networks
  int n = WiFi.scanNetworks();
  
  if (n == 0) {
    Serial.println("No networks found.");
  } else {
    // Check all found networks against our beacon list
    for (int i = 0; i < n; ++i) {
      String foundSSID = WiFi.SSID(i);
      int rawRSSI = WiFi.RSSI(i);

      for (int b = 0; b < 3; b++) {
        if (foundSSID == beacons[b].ssid) {
          float filtered = kalmanFilter(rawRSSI, beacons[b]);
          beacons[b].distance = rssiToDistance(filtered);
          beacons[b].visible = true;
          
          Serial.printf("%s: %.2fm | ", beacons[b].ssid.c_str(), beacons[b].distance);
        }
      }
    }
    Serial.println(); // Newline after listing distances
  }

  // Only calculate (X,Y) if all 3 beacons were seen in this scan
  if (beacons[0].visible && beacons[1].visible && beacons[2].visible) {
    calculatePosition();
  } else {
    Serial.println("Waiting for all 3 beacons...");
  }

  // Clear memory from the scan so it doesn't leak
  WiFi.scanDelete();
  delay(SCAN_DELAY);
}