// Client_Wifi.cpp

#include <WiFi.h>
#include <math.h>

/* ================= BEACON CONFIGURATION ================= */
struct Beacon {
  String ssid;
  float raw_rssi;   // Added: Stores the actual raw reading
  float x_est;      
  float P;          
  float distance;   
  bool visible;     
};

// 5 servers using staggered channels
Beacon beacons[5] = {
  {"Channel_1", 0.0, -60.0, 1.0, 0.0, false},   
  {"Channel_4", 0.0, -60.0, 1.0, 0.0, false},   
  {"Channel_6", 0.0, -60.0, 1.0, 0.0, false},   
  {"Channel_8", 0.0, -60.0, 1.0, 0.0, false},
  {"Channel_11", 0.0, -60.0, 1.0, 0.0, false}
};

/* ================= SYSTEM PARAMETERS ================= */
const int SCAN_DELAY = 500;

// KALMAN TUNING: Adjusted to trust raw measurements more
const float Q = 0.1;  // Increased: Filter expects the environment to change faster
const float R = 2.0;  // Decreased: Filter trusts the raw scan values more

// These will be calculated dynamically in setup()
float CALIBRATED_A = -55.0; 
float CALIBRATED_N = 3.0;   
const float CALIBRATION_DIST_2 = 3.0; // Stage 2 distance in meters

/* ================= KALMAN FILTER ================= */
float kalmanFilter(float z, Beacon &b) {
  b.P = b.P + Q;
  float K = b.P / (b.P + R);
  b.x_est = b.x_est + K * (z - b.x_est);
  b.P = (1 - K) * b.P;
  return b.x_est;
}

/* ================= DISTANCE CALCULATION ================= */
float rssiToDistance(float rssi) {
  return pow(10.0, (CALIBRATED_A - rssi) / (10.0 * CALIBRATED_N));
}

/* ================= CALIBRATION ROUTINE ================= */
float sampleCalibrationRSSI(String targetSSID, int samples) {
  float sumRSSI = 0;
  int validSamples = 0;
  
  Serial.printf("Sampling %s...\n", targetSSID.c_str());
  
  while (validSamples < samples) {
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; ++i) {
      if (WiFi.SSID(i) == targetSSID) {
        sumRSSI += WiFi.RSSI(i);
        validSamples++;
        Serial.print(".");
      }
    }
    WiFi.scanDelete();
    delay(200); 
  }
  Serial.println();
  return sumRSSI / validSamples;
}

void runCalibration() {
  Serial.println("\n=== STARTING CALIBRATION ===");
  
  // STAGE 1: Find 'A'
  Serial.println("STAGE 1: Place the ESP32 exactly 1.0 meter from 'Channel_1'.");
  Serial.println("You have 10 seconds to position it...");
  for(int i=10; i>0; i--) { Serial.printf("%d... ", i); delay(1000); }
  Serial.println("\nCalculating A parameter...");
  
  CALIBRATED_A = sampleCalibrationRSSI("Channel_1", 20);
  Serial.printf("=> Calibrated A (RSSI at 1m): %.2f dBm\n\n", CALIBRATED_A);

  // STAGE 2: Find 'n'
  Serial.printf("STAGE 2: Move the ESP32 exactly %.1f meters away from 'Channel_1'.\n", CALIBRATION_DIST_2);
  Serial.println("You have 10 seconds to move it...");
  for(int i=10; i>0; i--) { Serial.printf("%d... ", i); delay(1000); }
  Serial.println("\nCalculating Path Loss Exponent (n)...");
  
  float rssi_d2 = sampleCalibrationRSSI("Channel_1", 20);
  Serial.printf("RSSI at %.1fm: %.2f dBm\n", CALIBRATION_DIST_2, rssi_d2);
  
  CALIBRATED_N = (CALIBRATED_A - rssi_d2) / (10.0 * log10(CALIBRATION_DIST_2));
  
  if (CALIBRATED_N < 1.0 || CALIBRATED_N > 6.0) {
    Serial.println("WARNING: Calculated 'n' is outside typical indoor bounds. Check for heavy interference.");
  }
  
  Serial.printf("=> Calibrated n (Path Loss): %.2f\n", CALIBRATED_N);
  Serial.println("=== CALIBRATION COMPLETE. STARTING SCANNER ===\n");
}

/* ================= TERMINAL PRINTING ================= */
void printBeaconData() {
  Serial.println("\n--- Current Beacon Status ---");
  for (int i = 0; i < 5; i++) {
    // Formatted to show Raw, Filtered, and Distance cleanly
    Serial.printf("%-10s | Raw: %6.2f dBm | Filtered: %6.2f dBm | Dist: %5.2f m\n", 
                  beacons[i].ssid.c_str(), 
                  beacons[i].raw_rssi,
                  beacons[i].x_est, 
                  beacons[i].distance);
  }
  Serial.println("-----------------------------");
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  
  runCalibration();
}

/* ================= LOOP ================= */
void loop() {
  for (int i = 0; i < 5; i++) beacons[i].visible = false;

  int n = WiFi.scanNetworks();
  
  if (n > 0) {
    for (int i = 0; i < n; ++i) {
      String foundSSID = WiFi.SSID(i);
      int rawRSSI = WiFi.RSSI(i);

      for (int b = 0; b < 5; b++) {
        if (foundSSID == beacons[b].ssid) {
          beacons[b].raw_rssi = rawRSSI; // Save the raw measurement for printing
          float filtered = kalmanFilter(rawRSSI, beacons[b]);
          beacons[b].distance = rssiToDistance(filtered); // Calculate distance using filtered RSSI
          beacons[b].visible = true;
        }
      }
    }
  }

  bool allVisible = true;
  for (int i = 0; i < 5; i++) {
    if (!beacons[i].visible) allVisible = false;
  }

  if (allVisible) {
    printBeaconData(); 
  } else {
    Serial.println("Waiting for all 5 beacons to appear in a single scan...");
  }

  WiFi.scanDelete();
  delay(SCAN_DELAY);
}