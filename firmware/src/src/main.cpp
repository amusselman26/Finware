#include <Arduino.h>
#include <Wire.h>
#include <algorithm>
#include <cmath>

#include "services/SensorsFacade.hpp"  // Sensor facade
#include "drivers/SDLogger.h"          // sdBegin, openNextLog, writeSnapshot, flushLog, closeLog
#include "drivers/LoRaRadio.h"         // wrapper around RH_RF95 from earlier
#include "app/StateMachine.h"         // FSM
#include "drivers/FinDriver.h"
#include "app/RocketAttitudeController.hpp"

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
FinDriver     fins;  // commented out to disable fin commands

const finware::app::AttitudeControllerConfig g_attCfg = {
  .roll = {.kp = 0.6f, .kd = 0.35f},
  .pitch = {.kp = 2.0f, .kd = 0.3f},
  .yaw = {.kp = 2.0f, .kd = 0.3f},
  .fin_limit_rad = 10.0f * static_cast<float>(M_PI) / 180.0f
};

finware::app::RocketAttitudeController g_attController(g_attCfg);
const finware::app::Vec3 g_cmdEulerRpyRad = {
  0.0f,
  0.5f * static_cast<float>(M_PI),
  0.0f
};
bool g_attControlEnabled = false;

// Minimal binary packet (little-endian) for LLA telemetry
// Must match the ground receiver's TelemetryLLA layout.
struct __attribute__((packed)) TelemetryLLA {
  int32_t  lat_e7;   // degrees * 1e7
  int32_t  lon_e7;   // degrees * 1e7
  float    alt_m;    // meters
  uint32_t t_ms;     // sender timestamp
};

struct __attribute__((packed)) FlightRecord {
  SensorsSnapshot snap;    // existing snapshot (unchanged contents)
  float fin_cmd_rad;       // max abs commanded fin angle [rad]
};

static finware::app::Mat3 quatToDcmBE(const IMU_Sample& imu) {
  const float qw = imu.q[0];
  const float qx = imu.q[1];
  const float qy = imu.q[2];
  const float qz = imu.q[3];

  const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
  const float roll = atan2f(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (qw * qy - qz * qx);
  sinp = finware::app::clamp(sinp, -1.0f, 1.0f);
  const float pitch = asinf(sinp);

  const float siny_cosp = 2.0f * (qw * qz + qx * qy);
  const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  const float yaw = atan2f(siny_cosp, cosy_cosp);

  return finware::app::euler321ToDCM(roll, pitch, yaw);
}

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

  // --- Fins (disabled) ---
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
      g_attControlEnabled = false;
    }

    else if (cmd == "ABORT") {
      fsm.transitionTo(SystemState::ABORT, &sensors); 
      LoRa.sendText("Abort command received. Transitioning to ABORT state.");
      g_attControlEnabled = false;
      fins.commandNeutral();
    }

    else if (cmd.equalsIgnoreCase("CALIB_BARO")) {
      sensors.calibrateAltitudeReferences();
      float baroAlt = sensors.baro().altitude_m;
      float gnssAlt = sensors.gnss().alt_m;
      String msg = "Baro+GNSS calibrated. Baro alt: " + String(baroAlt, 2) + " m, GNSS alt: " + String(gnssAlt, 2) + " m";
      LoRa.sendText(msg);
    }

    else if (cmd == "TEST") {
      fins.finTestSequence(fins);
      LoRa.sendText("Fin test sequence executed (fins disabled).");
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
    // Enable closed-loop fin control at launch.
    if (current == SystemState::LAUNCH) {
      g_attControlEnabled = true;
      Serial.println("Launch detected: attitude control enabled.");
    }
    lastState = current;   // <-- update so it won't repeat

    // Disable closed-loop control at apogee.
    if (current == SystemState::APOGEE) {
      g_attControlEnabled = false;
      Serial.println("Apogee detected: attitude control disabled.");
    }
  }


  // Update sensors
  sensors.tick();
  fsm.update(sensors);

  // Log a full snapshot to SD (binary)
  const SensorsSnapshot snap_now = sensors.snapshot();

  const uint32_t nowMs = millis();

  finware::app::FinCommands fin_cmd_rad{0.0f, 0.0f, 0.0f, 0.0f};

  const IMU_Sample& imu = sensors.imu();
  if (imu.t_us != 0) {
    if (g_attControlEnabled) {
      const finware::app::Mat3 current_dcm_be = quatToDcmBE(imu);
      const finware::app::Vec3 body_rates_pqr = {imu.gx, imu.gy, imu.gz};
      fin_cmd_rad = g_attController.update(current_dcm_be, g_cmdEulerRpyRad, body_rates_pqr);
    }
  }

  static float u_sent_rad = 0.0f; // for logging
  // Fin command at 50 hz
  static uint32_t lastFinUpdateMs = 0;
  if (nowMs - lastFinUpdateMs >= 20u) {   // 50 Hz
    float angles[NUM_FINS];
    const float rad_to_deg = 180.0f / static_cast<float>(M_PI);
    angles[FIN_TOP] = fin_cmd_rad.d1 * rad_to_deg;
    angles[FIN_RIGHT] = fin_cmd_rad.d2 * rad_to_deg;
    angles[FIN_BOTTOM] = fin_cmd_rad.d3 * rad_to_deg;
    angles[FIN_LEFT] = fin_cmd_rad.d4 * rad_to_deg;
    fins.setAllFinAngles(angles);
    lastFinUpdateMs = nowMs;
    u_sent_rad = std::max(
        std::max(fabsf(fin_cmd_rad.d1), fabsf(fin_cmd_rad.d2)),
        std::max(fabsf(fin_cmd_rad.d3), fabsf(fin_cmd_rad.d4)));
  }

  
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

  // Serial.println(snap_now.imu.ax);
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

  // Log a combined flight record (snapshot + fin command) to SD
  FlightRecord rec;
  rec.snap = snap_now;  // copy existing snapshot contents
  rec.fin_cmd_rad  = u_sent_rad;

  if (!writeRecordEx(&rec, sizeof(rec))) {
    LoRa.sendText("Failed to write extend ed log record.");
  }
}
