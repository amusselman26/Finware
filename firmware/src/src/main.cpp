#include <Arduino.h>
#include <Wire.h>

#include "services/SensorsFacade.hpp"  // your facade
#include "drivers/SDLogger.h"          // sdBegin, openNextLog, writeSnapshot, flushLog, closeLog
#include "drivers/LoRaRadio.h"         // wrapper around RH_RF95 from earlier
#include "app/StateMachine.h"         // FSM
#include "drivers/FinDriver.h"         // add this

using namespace finware;

// ------------------- Pins / Addresses (adjust as needed) -------------------
constexpr uint8_t IMU_CS     = 10;     // BNO085 CS (SPI) — must not conflict with SD CS
constexpr uint8_t IMU_INT    = 9;      // BNO085 INT
constexpr int8_t  IMU_RST    = 5;      // BNO085 RST
constexpr uint8_t BARO_ADDR  = 0x5D;   // LPS22 I2C
constexpr uint8_t GNSS_ADDR  = 0x42;   // u-blox I2C

// LoRa (RFM95) — avoid conflict with SD CS=10 by NOT using 10 for RST/CS
constexpr uint8_t LORA_CS    = 12;
constexpr uint8_t LORA_INT   = 6;
constexpr uint8_t LORA_RST   = 13;
constexpr float   LORA_FREQ  = 915.0f; // MHz

// ------------------- Globals -------------------
char txtFilename[20];
char binFilename[20];

SensorsFacade sensors(IMU_CS, IMU_INT, IMU_RST, BARO_ADDR, GNSS_ADDR);
LoRaRadio     LoRa(LORA_CS, LORA_INT, LORA_RST, LORA_FREQ);
StateMachine  fsm;
FinDriver     fins;              // add this

// Minimal binary packet (little-endian)
struct __attribute__((packed)) TelemetryLLA {
  int32_t lat_e7;   // degrees * 1e7
  int32_t lon_e7;   // degrees * 1e7
  float   alt_m;    // meters
  uint32_t t_ms;    // sender timestamp
};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // off until we know we're healthy
  delay(5000); // 5 seconds
  digitalWrite(LED_BUILTIN, LOW); // on to indicate we're starting setup
  Serial.begin(115200);
  while (!Serial) {}   // ok to keep setup prints for bring-up
  Serial.println("Finware Flight Computer Starting...");\

  Wire.begin();
  Serial.println("I2C initialized.");

  // --- Sensors ---
  if (!sensors.begin()) {
    Serial.println("Sensors init failed!");
    while (true) delay(1000);
  }
  Serial.println("Sensors initialized.");

  
  // --- LoRa ---
  if (!LoRa.begin(/*txPower=*/23)) {
    Serial.println("LoRa init failed!");
    while (true) delay(1000);
  }
  Serial.println("LoRa ready.");

  // --- SD ---
  if (!sdBegin()) {
    Serial.println("SD init failed!");
    while (true) delay(1000);
  }
  if (!openNextLog(binFilename, txtFilename)) {
    Serial.println("Failed to open next log file!");
    while (true) delay(1000);
  }
  Serial.print("Logging to "); Serial.println(txtFilename);

  // --- Fins ---
  Serial.println("I'm finning it");
  fins.begin();
  fins.finTestSequence(fins);   // run 0, -10, +10 test
  Serial.println("Fin test sequence complete.");

}

void loop() {
  // Update sensors
  sensors.tick();
  fsm.update(sensors);

  // Log a full snapshot to SD (binary)
  const SensorsSnapshot snap_now = sensors.snapshot();
  (void)writeRecord(snap_now);

  if (snap_now.baro.altitude_m < -5) {
    sensors.baro_.calibrateAtm();
    sensors.baro_.tick();
  }

  const uint32_t nowMs = millis();

  // --- Loops per second over Serial ---
  static uint32_t loopCount = 0;
  static uint32_t lastLpsMs = 0;
  loopCount++;
  if (nowMs - lastLpsMs >= 1000u) {
    Serial.print("Loops per second: ");
    Serial.println(loopCount);
    loopCount = 0;
    lastLpsMs = nowMs;
  }

  // Periodic SD flush (1 s)
  static uint32_t lastFlushMs = 0;
  if (nowMs - lastFlushMs >= 1000u) {
    flushLog();
    lastFlushMs = nowMs;
  }

  // Send lat, lon, alt over LoRa every 1 s
  static uint32_t lastTxMs = 0;
  if (nowMs - lastTxMs >= 1000u) {
    TelemetryLLA pkt;
    pkt.lat_e7 = static_cast<int32_t>(snap_now.gnss.lat);
    pkt.lon_e7 = static_cast<int32_t>(snap_now.gnss.lon);
    pkt.alt_m  = snap_now.gnss.alt_m;
    pkt.t_ms   = nowMs;

    LoRa.sendMessage(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    lastTxMs = nowMs;
  }
}
