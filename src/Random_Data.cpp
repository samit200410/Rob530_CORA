// Random_Data.cpp
//
// PlatformIO version of random_data.ino.
// Emits fake WiFi-trilateration data over USB Serial in exactly the same
// format as Client_Wifi.cpp so serial_dump.py can be tested without servers.
//
// How the fake data is generated:
//   1. A random (posX, posY) is picked inside the beacon triangle each scan.
//   2. True Euclidean distances to each beacon are computed from that point.
//   3. Small Gaussian-like noise is added so values look like real
//      Kalman-filtered RSSI measurements.
//   4. Both the human-readable lines AND the DATA, CSV line are printed —
//      output is byte-for-byte compatible with Client_Wifi.cpp.

#include <Arduino.h>   // required for PlatformIO .cpp files
#include <math.h>

/* ================= BEACON LAYOUT (must match Client_Wifi.cpp) ================= */
const float BX[3] = {0.0, 5.0, 2.5};
const float BY[3] = {0.0, 0.0, 5.0};
const char* BNAME[3] = {"Test1", "Test2", "Test3"};

/* ================= TIMING (must match Client_Wifi.cpp) ================= */
const int SCAN_DELAY = 500;

/* ================= NOISE ================= */
const float NOISE_STD = 0.15;  // distance noise std-dev in metres

/* ================= HELPERS ================= */

// Simple Box-Muller normal sample (mean=0, std=1)
float randNormal() {
  float u1 = (float)random(1, 100000) / 100000.0f;
  float u2 = (float)random(1, 100000) / 100000.0f;
  return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
}

// Euclidean distance between two 2-D points
float dist2D(float ax, float ay, float bx, float by) {
  float dx = ax - bx;
  float dy = ay - by;
  return sqrtf(dx * dx + dy * dy);
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));
  Serial.println("RandomData: Scanner Ready. Waiting for beacons...");
}

/* ================= LOOP ================= */
void loop() {
  // Pick a random position inside the bounding box of the beacon triangle
  float posX = 0.2f + ((float)random(0, 10000) / 10000.0f) * 4.6f;
  float posY = 0.2f + ((float)random(0, 10000) / 10000.0f) * 4.6f;

  // Compute noisy distances to each beacon
  float d[3];
  for (int i = 0; i < 3; i++) {
    float trueDist = dist2D(posX, posY, BX[i], BY[i]);
    d[i] = max(0.05f, trueDist + randNormal() * NOISE_STD);
  }

  // Human-readable distance line (same format as Client_Wifi.cpp)
  for (int i = 0; i < 3; i++) {
    Serial.printf("%s: %.2fm | ", BNAME[i], d[i]);
  }
  Serial.println();

  // Human-readable position line
  Serial.print("=> POSITION: X: ");
  Serial.print(posX, 2);
  Serial.print(" m | Y: ");
  Serial.print(posY, 2);
  Serial.println(" m");

  // Structured CSV line consumed by serial_dump.py
  Serial.printf("DATA,%lu,%.4f,%.4f,%.4f,%.4f,%.4f\n",
    millis(), d[0], d[1], d[2], posX, posY);

  delay(SCAN_DELAY);
}