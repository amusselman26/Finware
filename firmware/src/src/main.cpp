#include <Arduino.h>
#include <Wire.h>

#include "services/SensorsFacade.hpp"  // Sensor facade
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

// Minimal binary packet (little-endian) for LLA telemetry
// Must match the ground receiver's TelemetryLLA layout.
struct __attribute__((packed)) TelemetryLLA {
  int32_t  lat_e7;   // degrees * 1e7
  int32_t  lon_e7;   // degrees * 1e7
  float    alt_m;    // meters
  uint32_t t_ms;     // sender timestamp
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
  static SystemState lastState = fsm.getState();
  
  // Check for LoRa commands (TEXT packets with 1-byte type header)
  String cmd;
  if (LoRa.receiveMessage(cmd, 1)) {      // 0 ms = poll, no blocking
    cmd.trim();
    Serial.print("LoRa RX: ");
    Serial.println(cmd);

    if (cmd == "ARM") {
      fsm.setArmCommand(true);
        // After all sensors are initialized, switch IMU to gyro-integrated RV
      if (!sensors.setIMUReport(SH2_GYRO_INTEGRATED_RV, 5000)) {
        Serial.println("Failed to set IMU report to GYRO_INTEGRATED_RV");
      } else {
        LoRa.sendText("Armed. IMU report set to GYRO_INTEGRATED_RV.");
      }
    }

    else if (cmd == "ABORT") {
      fsm.transitionTo(SystemState::ABORT, &sensors); 
      LoRa.sendText("Abort command received. Transitioning to ABORT state.");
    }

    else if (cmd == "CALIB_BARO") {
      sensors.baro_.calibrateAtm();
      sensors.baro_.tick(); // update immediately after calibration
      float alt = sensors.baro().altitude_m;
      String msg = "Barometer calibrated. Current altitude: " + String(alt, 2) + " m";
      LoRa.sendText(msg);
    }

    else if (cmd == "TEST") {
      fins.finTestSequence(fins);
      LoRa.sendText("Fin test sequence executed.");
    }
    
    else {
      Serial.println("LoRa: Unknown command");
    }
  }

  // Radio messages for state transitions
  SystemState current = fsm.getState();
  if (current != lastState) {
    String message = "STATE CHANGED TO: " + fsm.stateName(current);
    LoRa.sendText(message);
    lastState = current;   // <-- update so it won't repeat
  }


  // Update sensors
  sensors.tick();
  fsm.update(sensors);

  // Log a full snapshot to SD (binary)
  const SensorsSnapshot snap_now = sensors.snapshot();
  if(!writeRecord(snap_now)) {
    LoRa.sendText("Failed to write log record.");
    
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
    // Degrees * 1e7 to match receiver expectations
    pkt.lat_e7 = static_cast<int32_t>(snap_now.gnss.lat * 1e7);
    pkt.lon_e7 = static_cast<int32_t>(snap_now.gnss.lon * 1e7);
    pkt.alt_m  = snap_now.baro.altitude_m;
    pkt.t_ms   = nowMs;

    // Send as a typed LLA packet (1-byte header + TelemetryLLA payload)
    LoRa.sendTyped(LoRaPacketType::LLA,
                   reinterpret_cast<const uint8_t*>(&pkt),
                   static_cast<uint8_t>(sizeof(pkt)));
    lastTxMs = nowMs;
  }
}
