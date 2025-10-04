#include <Arduino.h>
#include <Wire.h>
#include "services/SensorsFacade.hpp"   // adjust path as needed
#include "drivers/SDLogger.h"           // provides sdBegin, openNextLog, writeSnapshot, flushLog, closeLog

using namespace finware;

// -------------------
// Configure pins / addrs  (adjust to your wiring/board!)
// -------------------
constexpr uint8_t IMU_CS    = 11;   // BNO085 CS (SPI)  — make sure this pin isn’t used by SD CS
constexpr uint8_t IMU_INT   = 9;    // BNO085 INT
constexpr int8_t  IMU_RST   = 5;    // BNO085 RST
constexpr uint8_t BARO_ADDR = 0x5D; // LPS22HB/HD I2C
constexpr uint8_t GNSS_ADDR = 0x42; // u-blox I2C

char logFilename[20];

// -------------------
// Create facade
// -------------------
SensorsFacade sensors(IMU_CS, IMU_INT, IMU_RST, BARO_ADDR, GNSS_ADDR);

void setup() {
  Serial.begin(115200);
  while (!Serial) {} // wait for USB if needed

  Wire.begin();
  // If your IMU uses SPI in your facade, that code should call SPI.begin() internally.
  // If not, uncomment next line:
  // SPI.begin();

  delay(500); // short settle
  Serial.println("Initializing sensors...");

  if (!sensors.begin()) {
    Serial.println("Sensors init failed!");
    delay(2000);
    // you might want to retry or safe-halt here
  }
  Serial.println("Sensors initialized.");

  if (!sdBegin()) {
    Serial.println("SD init failed!");
    while (true) { delay(1000); } // halt
  }
  if (!openNextLog(logFilename)) {
    Serial.println("Failed to open next log file!");
    while (true) { delay(1000); }
  }
  Serial.print("Logging to "); Serial.println(logFilename);
}

void loop() {
  // Tick/update all sensors; ensure this is non-blocking inside your facade
  sensors.tick();

  // Write one binary snapshot each loop
  const SensorsSnapshot snap_now = sensors.snapshot();
  if (!writeRecord(snap_now)) {
    // optional: increment an error counter, light LED, etc.
    Serial.println("SD write failed!");
  }

  // Periodic flush to keep FAT consistent without killing performance
  static uint32_t lastFlush = 0;
  if (millis() - lastFlush >= 1000) { // flush every 1 s
    flushLog();
    lastFlush = millis();
  }

  // Periodically print a human-readable line for debugging
  static uint32_t t_next_print = 0;
  if (millis() >= t_next_print) {
    const SensorsSnapshot snap = sensors.snapshot(); // snapshot again if values change rapidly

    Serial.print("t_us: ");        Serial.print(snap.t_us);
    Serial.print(" | q1: ");       Serial.print(snap.imu.q[1]);
    Serial.print(" | P[hPa]: ");   Serial.print(snap.baro.pressure_hPa);
    Serial.print(" | lat: ");      Serial.println(snap.gnss.lat, 6);

    t_next_print = millis() + 200; // every 200 ms
  }

  // … your loop timing policy here (delay or fixed-rate scheduler)
}