// Finware.ino  — minimal example using IMU_BNO085 driver
// This must be copied into main.cpp of src before runtime. Storing example here prevents multiple definitions of setup and loop.
#include <Arduino.h>
#include "IMU_BNO085.hpp"

// Pinout from your demo
const uint8_t BNO_CS  = 10;
const uint8_t BNO_INT = 9;
const int8_t  BNO_RST = 5;

// Driver instance
IMU_BNO085 imu(BNO_CS, BNO_INT, BNO_RST);

// Print rate control
static uint32_t nextPrint_us = 0;
static const uint32_t PRINT_PERIOD_US = 20000; // 50 Hz prints

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB CDC */ }

  Serial.println("\n[BNO085] bring-up...");

  // Start with the more accurate rotation vector at ~200 Hz (5000 us interval)
  if (!imu.begin(SH2_ARVR_STABILIZED_RV, 5000)) {
    Serial.println("ERROR: BNO085 init failed");
    // You could continue without IMU, but we'll stop here for the demo:
    while (true) { delay(1000); }
  }
  Serial.println("BNO085 ready.");
}

void loop() {
  // Non-blocking poll; at most one event per call is processed
  imu.tick();

  // Optional: detect and report sensor resets
  if (imu.wasReset()) {
    Serial.println("[BNO085] sensor reset detected; report re-enabled.");
  }

  // Print Euler angles at 50 Hz
  uint32_t now = micros();
  if ((int32_t)(now - nextPrint_us) >= 0) {
    nextPrint_us += PRINT_PERIOD_US;

    // Read latest quaternion-derived Euler (deg)
    IMU_EulerDeg e = imu.latestEulerDegrees();

    // Diagnostics
    uint32_t seq = imu.sequence();
    // uint8_t cal = imu.calibration();

    Serial.print("seq=");
    Serial.print(seq);
    // Serial.print(" cal=");
    // Serial.print(cal);
    Serial.print(" | yaw=");
    Serial.print(e.yaw, 2);
    Serial.print(" pitch=");
    Serial.print(e.pitch, 2);
    Serial.print(" roll=");
    Serial.println(e.roll, 2);
  }

  // (Optional) Example of switching to the faster report at runtime:
  // static bool switched = false;
  // if (!switched && millis() > 5000) {
  //   imu.setReport(SH2_GYRO_INTEGRATED_RV, 2000); // ~500 Hz target (variable)
  //   Serial.println("[BNO085] switched to GYRO_INTEGRATED_RV @ 2000 us");
  //   switched = true;
  // }
}
